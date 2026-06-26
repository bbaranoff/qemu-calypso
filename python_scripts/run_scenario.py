#!/usr/bin/env python3
"""
run_scenario.py - Sequence FBSB + SI + lua-batch + call dans un seul appel.

Usage :
  ./run_scenario.py                     # scenario par defaut
  ./run_scenario.py --skip-call         # sans tentative call
  ./run_scenario.py --call 0123456789   # call number custom
  ./run_scenario.py --si 1,2,3,4        # SI types a injecter
  ./run_scenario.py --lua-script /tmp/scenario.txt   # batch VTY custom

Phases :
  1. FBSB     : inject_fbsb_fb_found + inject_fbsb_sb_found via inject.py
  2. SI       : pour chaque type N : inject_si(N) via inject.py
  3. Lua      : execute un batch VTY (= sequence "lua-like" mobile_ctl.py)
  4. Call     : place un appel via mobile VTY

Chaque phase loggee + delta indicateurs.
"""

from __future__ import annotations
import argparse
import sys
import time

# Import direct des modules locaux
try:
    import inject as inj
    import mobile_ctl as mc
except ImportError as e:
    print(f"[scenario] ERROR : missing module ({e}). Run from qemu-src/", file=sys.stderr)
    sys.exit(1)


def phase(label: str):
    bar = "=" * 70
    print(f"\n{bar}\n[scenario] PHASE : {label}\n{bar}")


def main():
    ap = argparse.ArgumentParser(description="FBSB + SI + Lua + Call sequence")
    ap.add_argument("--si", default="1,2,3,4",
                    help="comma-sep SI types to inject (default: 1,2,3,4)")
    ap.add_argument("--call", default="0123456789",
                    help="number to call (default 0123456789)")
    ap.add_argument("--skip-fbsb", action="store_true")
    ap.add_argument("--skip-si", action="store_true")
    ap.add_argument("--skip-lua", action="store_true")
    ap.add_argument("--skip-call", action="store_true")
    ap.add_argument("--iterations", type=int, default=10,
                    help="iterations FBSB/SI inject loops")
    ap.add_argument("--interval-ms", type=int, default=80)
    ap.add_argument("--lua-script", help="VTY batch script path (one cmd per line)")
    ap.add_argument("--gdb-host", default="127.0.0.1")
    ap.add_argument("--gdb-port", type=int, default=1234)
    ap.add_argument("--vty-host", default="127.0.0.1")
    ap.add_argument("--vty-port", type=int, default=4247)
    ap.add_argument("--ms", default="1")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    # ---- Single gdb session pour Phase 1 + 2 (evite race close/reopen) ----
    need_gdb = (not args.skip_fbsb) or (not args.skip_si)
    g = None
    if need_gdb:
        try:
            g = inj.open_session(host=args.gdb_host, port=args.gdb_port,
                                 activate=False)
        except Exception:
            g = None
        if g is None:
            print(f"\n  [SKIP gdb phases] gdb-stub {args.gdb_host}:{args.gdb_port} "
                  f"pas dispo (lance ./run.sh d'abord)")

    # ---- Phase 1 : FBSB ----
    if not args.skip_fbsb and g is not None:
        phase("1. FBSB inject (FB found + SB found)")
        snap = inj.probe_ndb(g)
        print(f"  before : d_fb_det={snap.get('d_fb_det')} "
              f"a_sync_PM={snap.get('a_sync_PM')}")
        print(f"  inject_fbsb_fb_found x {args.iterations}")
        inj.burst_inject(g, inj.inject_fbsb_fb_found,
                         iterations=args.iterations,
                         interval_ms=args.interval_ms)
        print(f"  inject_fbsb_sb_found x {args.iterations}")
        inj.burst_inject(g, inj.inject_fbsb_sb_found,
                         iterations=args.iterations,
                         interval_ms=args.interval_ms)
        snap = inj.probe_ndb(g)
        print(f"  after  : d_fb_det={snap.get('d_fb_det')} "
              f"a_sync_PM={snap.get('a_sync_PM')}")

    # ---- Phase 2 : SI ----
    if not args.skip_si and g is not None:
        phase(f"2. SI inject ({args.si})")
        si_types = [int(x) for x in args.si.split(",") if x.strip()]
        for n in si_types:
            fn = lambda gg, _n=n: inj.inject_si(gg, _n)
            print(f"  inject_si({n}) x {args.iterations}")
            inj.burst_inject(g, fn,
                             iterations=args.iterations,
                             interval_ms=args.interval_ms)
        print(f"  a_cd[0..14] : {inj.probe_ndb(g).get('a_cd[0..14]')}")

    # Close gdb session apres Phase 2 (avant phases VTY)
    if g is not None:
        inj.close_session(g)

    # ---- Single VTY session pour Phase 3 + 4 ----
    need_vty = (not args.skip_lua) or (not args.skip_call)
    v = None
    if need_vty:
        try:
            v = mc.Vty(host=args.vty_host, port=args.vty_port, timeout=5.0)
            v.connect()
        except (ConnectionRefusedError, TimeoutError, OSError) as e:
            print(f"\n  [SKIP VTY phases] {args.vty_host}:{args.vty_port} pas dispo : {e}")
            v = None

    # ---- Phase 3 : Lua-like batch via VTY ----
    if not args.skip_lua and v is not None:
        phase("3. Lua-like batch (VTY sequence)")
        if args.lua_script:
            print(f"  running script {args.lua_script}")
            mc.exec_script(v, args.lua_script,
                           delay_ms=args.interval_ms,
                           verbose=True)
        else:
            default_cmds = [
                f"show ms {args.ms}",
                f"show cell {args.ms}",
                f"show subscriber {args.ms}",
                "sleep 500",
                f"show ms {args.ms}",
            ]
            print(f"  default sequence : {len(default_cmds)} cmds")
            mc.exec_batch(v, default_cmds,
                          delay_ms=args.interval_ms,
                          verbose=True)

    # ---- Phase 4 : Call ----
    if not args.skip_call and v is not None:
        phase(f"4. Call : {args.call}")
        v.enable()
        # Power-on if needed (state "down" or "power off")
        state = mc.show_ms(v, args.ms)
        if "is down" in state.lower() or "power off" in state.lower() or "radio is not started" in state.lower():
            print("  power-on first")
            print(mc.power_on(v, args.ms))
            time.sleep(3)
            print("  wait for camp...")
            time.sleep(5)
        # Place call
        resp = mc.make_call(v, args.call, args.ms)
        print(f"  call response : {resp}")
        # Wait + status
        time.sleep(3)
        print(f"  ms state after call :")
        for line in mc.show_ms(v, args.ms).splitlines()[:10]:
            print(f"    {line}")
        # Hangup
        print(f"  hangup")
        print(mc.hangup(v, args.ms))

    # Close VTY session
    if v is not None:
        v.close()

    print("\n[scenario] DONE")


if __name__ == "__main__":
    main()
