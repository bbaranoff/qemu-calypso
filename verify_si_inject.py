#!/usr/bin/env python3
"""
verify_si_inject.py - Cycle tous les SI types via gdb + verifie en live
si le mobile les recoit.

Pour chaque SI in {1, 2, 3, 4, 5, 6, 13} :
  1. Probe NDB avant
  2. Inject SI<n> dans a_cd[] (+ d_task_d=ALLC pour signaler ALLC done)
  3. Wait + probe a_cd[] post-write (= survives ? overwritten ?)
  4. Grep mobile.log lignes ajoutees + grep osmocon.log L1CTL_DATA_IND
  5. Diff state mobile (show ms) avant/apres

Output : tableau visuel par SI type + verdict global "marche / pas marche".

Usage :
  ./verify_si_inject.py
  ./verify_si_inject.py --types 3,4         # restreint a SI3, SI4
  ./verify_si_inject.py --iters 100         # plus de tentatives par SI
"""

from __future__ import annotations
import argparse
import sys
import time
from typing import Optional

try:
    import inject as inj
    import mobile_ctl as mc
except ImportError as e:
    print(f"[verify_si] ERROR : {e}", file=sys.stderr)
    sys.exit(1)


ALLC_DSP_TASK = 24


def banner(msg):
    print(f"\n{'='*70}\n[verify_si] {msg}\n{'='*70}")


def grep_count(path: str, pattern: str) -> int:
    try:
        import re
        with open(path, errors="ignore") as f:
            return len(re.findall(pattern, f.read()))
    except FileNotFoundError:
        return 0


def show_state_short(v: Optional[mc.Vty], ms: str) -> str:
    if v is None:
        return "(no VTY)"
    try:
        txt = mc.show_ms(v, ms)
        for line in txt.splitlines():
            s = line.strip()
            if "service" in s.lower():
                return s.split(",")[-1].strip() if "," in s else s
        return "(unknown)"
    except Exception:
        return "(VTY err)"


def inject_si_with_task(g, si_type: int, iterations: int, interval_ms: int):
    """Inject SI<n> + signal d_task_d=ALLC_DONE for ARM to read."""
    def fn(gg):
        ok_si = inj.inject_si(gg, si_type)
        # Marquer d_task_d sur les deux pages pour augmenter chances
        ok_t0 = inj.inject_d_task(gg, ALLC_DSP_TASK, page=0)
        ok_t1 = inj.inject_d_task(gg, ALLC_DSP_TASK, page=1)
        return ok_si and ok_t0 and ok_t1
    return inj.burst_inject(g, fn,
                             iterations=iterations,
                             interval_ms=interval_ms)


def test_one_si(g, v, ms: str, si_type: int, iters: int, interval_ms: int,
                mobile_log: str = "/tmp/mobile.log",
                osmocon_log: str = "/tmp/osmocon.log") -> dict:
    """Test injection of one SI type, return result dict."""
    # Pre-state
    snap_before = inj.probe_ndb(g)
    mobile_before = grep_count(mobile_log, r".")
    osmocon_data_ind_before = grep_count(osmocon_log, r"L1CTL_DATA_IND")
    state_before = show_state_short(v, ms)

    # Inject
    payload = inj.synth_si(si_type)
    print(f"  payload (23B) : {payload.hex()}")
    print(f"  inject SI{si_type} x {iters} (delay {interval_ms}ms)")
    inject_si_with_task(g, si_type, iters, interval_ms)

    # Post-state
    time.sleep(1.0)  # let ARM/mobile process
    snap_after = inj.probe_ndb(g)
    mobile_after = grep_count(mobile_log, r".")
    osmocon_data_ind_after = grep_count(osmocon_log, r"L1CTL_DATA_IND")
    state_after = show_state_short(v, ms)

    a_cd_before = snap_before.get("a_cd[0..14]")
    a_cd_after = snap_after.get("a_cd[0..14]")
    a_cd_matches_payload = a_cd_after.startswith(payload[:20].hex())

    return {
        "si_type":            si_type,
        "payload":            payload.hex(),
        "a_cd_before":        a_cd_before,
        "a_cd_after":         a_cd_after,
        "a_cd_match_payload": a_cd_matches_payload,
        "mobile_lines_delta": mobile_after - mobile_before,
        "data_ind_delta":     osmocon_data_ind_after - osmocon_data_ind_before,
        "state_before":       state_before,
        "state_after":        state_after,
    }


def main():
    ap = argparse.ArgumentParser(description="Verify SI inject live for each SI type")
    ap.add_argument("--types", default="1,2,3,4,5,6,13",
                    help="SI types to test (comma-sep)")
    ap.add_argument("--iters", type=int, default=30,
                    help="iterations per SI type")
    ap.add_argument("--interval-ms", type=int, default=80)
    ap.add_argument("--gdb-host", default="127.0.0.1")
    ap.add_argument("--gdb-port", type=int, default=1234)
    ap.add_argument("--vty-host", default="127.0.0.1")
    ap.add_argument("--vty-port", type=int, default=4247)
    ap.add_argument("--ms", default="1")
    ap.add_argument("--mobile-log", default="/tmp/mobile.log")
    ap.add_argument("--osmocon-log", default="/tmp/osmocon.log")
    args = ap.parse_args()

    si_types = [int(x) for x in args.types.split(",") if x.strip()]
    banner(f"Verify SI inject : types={si_types} iters={args.iters}")

    # Open sessions
    try:
        g = inj.open_session(host=args.gdb_host, port=args.gdb_port, activate=False)
    except Exception:
        g = None
    if g is None:
        print(f"  ERROR gdb-stub {args.gdb_host}:{args.gdb_port} pas dispo", file=sys.stderr)
        sys.exit(2)

    v = None
    try:
        v = mc.Vty(host=args.vty_host, port=args.vty_port)
        v.connect()
    except Exception as e:
        print(f"  WARN VTY {args.vty_host}:{args.vty_port} pas dispo : {e}")
        v = None

    results = []
    try:
        for si in si_types:
            banner(f"SI{si} test")
            r = test_one_si(g, v, args.ms, si, args.iters, args.interval_ms,
                            args.mobile_log, args.osmocon_log)
            results.append(r)
            print(f"  a_cd_before  : {r['a_cd_before'][:40]}...")
            print(f"  a_cd_after   : {r['a_cd_after'][:40]}...")
            print(f"  match payload: {r['a_cd_match_payload']}")
            print(f"  mobile +lines: {r['mobile_lines_delta']}")
            print(f"  L1CTL_DATA_IND delta : {r['data_ind_delta']}")
            print(f"  state: {r['state_before']!r} -> {r['state_after']!r}")
    finally:
        inj.close_session(g)
        if v is not None:
            v.close()

    # Final table
    banner("RESULTS TABLE")
    print(f"  {'SI':>3} | {'match':5} | {'mob+':>5} | {'D_IND+':>7} | state change")
    print(f"  {'-'*3:>3}-+-{'-'*5:5}-+-{'-'*5:>5}-+-{'-'*7:>7}-+-{'-'*30}")
    for r in results:
        ok = "Y" if r["a_cd_match_payload"] else "N"
        change = "yes" if r["state_before"] != r["state_after"] else "no"
        print(f"  SI{r['si_type']:>1} |   {ok}   | {r['mobile_lines_delta']:>5} | "
              f"{r['data_ind_delta']:>7} | {change}")

    # Verdict
    banner("VERDICT")
    any_match = any(r["a_cd_match_payload"] for r in results)
    any_data_ind = any(r["data_ind_delta"] > 0 for r in results)
    if any_match and any_data_ind:
        print("  V SI inject visible cote DSP RAM + mobile recoit L1CTL_DATA_IND")
    elif any_match:
        print("  ~ SI present en RAM mais mobile ne recoit pas DATA_IND")
        print("    -> ARM/firmware ne lit pas a_cd[] (pas de ALLC task scheduled ?)")
    else:
        print("  X SI ecrasee immediatement -- DSP race ou wrong addr ?")
        print("    -> Try mode shunt (CALYPSO_MODE=shunt-ipc) pour gater c54x")


if __name__ == "__main__":
    main()
