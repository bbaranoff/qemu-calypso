#!/usr/bin/env python3
"""
call_via_gdb.py - Pousse un call jusqu'au RR via inject gdb-stub + VTY.

Sequence :
  1. Probe initial mobile state (VTY)
  2. Inject SI3 (+ SI4) dans a_cd[] via gdb-stub
     Boucle x N : ARM halt -> write NDB -> resume, repete pour augmenter
     les chances que ARM read a_cd[] dans la bonne fenetre TDMA.
  3. Inject FB/SB found pour maintenir le sync DSP
  4. Wait + check VTY que mobile passe en 'normal service'
  5. VTY: place call
  6. Show ms state apres call (= a-t-il atteint RR EST ?)
  7. Hangup + show final

Usage :
  ./call_via_gdb.py --call 0123456789
  ./call_via_gdb.py --call 0123456789 --inject-iters 50 --wait 5
"""

from __future__ import annotations
import argparse
import sys
import time

try:
    import inject as inj
    import mobile_ctl as mc
except ImportError as e:
    print(f"[call_via_gdb] ERROR : missing module ({e}). Run from qemu-src/", file=sys.stderr)
    sys.exit(1)


def banner(msg):
    print(f"\n{'='*70}\n[call_via_gdb] {msg}\n{'='*70}")


def show_state(v: mc.Vty, ms: str) -> str:
    """Get mobile state as 1-line summary."""
    txt = mc.show_ms(v, ms)
    state_lines = []
    for line in txt.splitlines():
        if any(k in line for k in ("MS '", "service", "cell selection state",
                                    "radio resource layer", "mobility management")):
            state_lines.append(line.strip())
    return " | ".join(state_lines)


def main():
    ap = argparse.ArgumentParser(description="Push a call through via gdb inject + VTY")
    ap.add_argument("--call", default="0123456789", help="number to dial")
    ap.add_argument("--inject-iters", type=int, default=30,
                    help="iterations FBSB + SI inject")
    ap.add_argument("--interval-ms", type=int, default=80)
    ap.add_argument("--wait", type=float, default=3.0,
                    help="seconds entre inject et call")
    ap.add_argument("--gdb-host", default="127.0.0.1")
    ap.add_argument("--gdb-port", type=int, default=1234)
    ap.add_argument("--vty-host", default="127.0.0.1")
    ap.add_argument("--vty-port", type=int, default=4247)
    ap.add_argument("--ms", default="1")
    args = ap.parse_args()

    # ---- 1. Probe initial mobile state ----
    banner("1. Probe initial mobile state")
    try:
        v = mc.Vty(host=args.vty_host, port=args.vty_port)
        v.connect()
    except Exception as e:
        print(f"  VTY unreachable : {e}", file=sys.stderr)
        sys.exit(2)
    state_before = show_state(v, args.ms)
    print(f"  before : {state_before}")

    # ---- 2. Inject SI3 + SI4 + FBSB via gdb ----
    banner("2. Inject SI3+SI4 + FBSB found x {n} via gdb-stub".format(n=args.inject_iters))
    try:
        g = inj.open_session(host=args.gdb_host, port=args.gdb_port, activate=False)
    except Exception:
        g = None
    if g is None:
        print(f"  ERROR gdb-stub {args.gdb_host}:{args.gdb_port} pas dispo", file=sys.stderr)
        v.close()
        sys.exit(3)

    try:
        # FBSB sync maintenu (au cas ou shunt off)
        inj.burst_inject(g, inj.inject_fbsb_fb_found,
                         iterations=args.inject_iters,
                         interval_ms=args.interval_ms)
        inj.burst_inject(g, inj.inject_fbsb_sb_found,
                         iterations=args.inject_iters // 2,
                         interval_ms=args.interval_ms)
        # SI3 dans a_cd[]
        print("  inject SI3 in a_cd[]")
        inj.burst_inject(g, lambda gg: inj.inject_si(gg, 3),
                         iterations=args.inject_iters,
                         interval_ms=args.interval_ms)
        # SI4 (LAI/CI confirmation)
        print("  inject SI4 in a_cd[]")
        inj.burst_inject(g, lambda gg: inj.inject_si(gg, 4),
                         iterations=args.inject_iters // 2,
                         interval_ms=args.interval_ms)
        # Final probe NDB
        snap = inj.probe_ndb(g)
        print(f"  a_cd[0..14] post : {snap.get('a_cd[0..14]')}")
    finally:
        inj.close_session(g)

    # ---- 3. Wait for mobile to ingest L1CTL_DATA_IND ----
    banner(f"3. Wait {args.wait}s pour propagation L1CTL_DATA_IND -> mobile L3")
    time.sleep(args.wait)
    state_post_inject = show_state(v, args.ms)
    print(f"  after inject : {state_post_inject}")

    # ---- 4. Place call ----
    banner(f"4. Place call to {args.call}")
    v.enable()
    resp = mc.make_call(v, args.call, args.ms)
    print(f"  VTY response : {resp.strip()}")

    # ---- 5. Wait + observe ----
    banner("5. Wait 4s + show ms state")
    time.sleep(4)
    state_after_call = show_state(v, args.ms)
    print(f"  after call : {state_after_call}")
    full = mc.show_ms(v, args.ms)
    for line in full.splitlines():
        print(f"    {line}")

    # ---- 6. Hangup ----
    banner("6. Hangup + final state")
    print(mc.hangup(v, args.ms).strip())
    time.sleep(1)
    print(f"  final : {show_state(v, args.ms)}")

    v.close()

    # ---- Verdict ----
    banner("VERDICT")
    if "no cell available" in state_after_call:
        print("  X mobile bloque en 'no cell available' -- SI3 inject n'a pas atteint L3")
        print("    Possibles : DSP overwrite trop rapide / ARM ne lit pas a_cd[]")
        print("    Essayer : mode shunt (c54x gate), augmenter --inject-iters")
    elif "MMCC_EST_REQ" in state_after_call or "MM_CONN" in state_after_call:
        print("  ! call en cours (RR EST initie)")
    elif "MM idle" in state_after_call and "no cell" not in state_after_call:
        print("  V mobile en MM idle normal service -- attempt a probablement reussi a aller en RR")
    else:
        print(f"  ? etat inattendu : {state_after_call}")


if __name__ == "__main__":
    main()
