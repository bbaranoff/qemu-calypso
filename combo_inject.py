#!/usr/bin/env python3
"""
combo_inject.py - Inject via gdb + halt complet via HMP (socat-style).

GDB seul halt l'ARM mais pas le c54x emule (CPU separe). Resultat : c54x
overwrite les writes. Solution combo :

  HMP `stop`  -> halts TOUS les CPUs (ARM + c54x)
  GDB write   -> writes memory en safe
  HMP `info registers` ou gdb probe -> verify
  GDB read    -> readback (still matches !)
  HMP `cont`  -> resumes tout

Usage :
  ./combo_inject.py --si 3 --iters 5
  ./combo_inject.py --action probe
  ./combo_inject.py --action si --types 1,2,3,4 --iters 3
"""

from __future__ import annotations
import argparse
import socket
import sys
import time

try:
    import inject as inj
except ImportError as e:
    print(f"[combo] ERROR : {e}", file=sys.stderr)
    sys.exit(1)


MON_SOCK_DEFAULT = "/tmp/qemu-calypso-mon.sock"
MON_SOCK = MON_SOCK_DEFAULT


def hmp(cmd: str, sock_path: str = None, timeout: float = 2.0) -> str:
    if sock_path is None:
        sock_path = MON_SOCK
    """Send HMP command via /tmp/qemu-calypso-mon.sock, return response."""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(sock_path)
    except Exception as e:
        raise RuntimeError(f"HMP socket {sock_path} unreachable : {e}")
    out = bytearray()
    # Read welcome (until first (qemu) prompt)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = s.recv(4096)
            if not chunk:
                break
            out.extend(chunk)
            if b"(qemu)" in out:
                break
        except socket.timeout:
            break
    out.clear()
    # Send command
    s.sendall((cmd + "\n").encode())
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = s.recv(4096)
            if not chunk:
                break
            out.extend(chunk)
            if out.rstrip().endswith(b"(qemu)"):
                break
        except socket.timeout:
            break
    s.close()
    text = out.decode(errors="replace")
    # Strip echo + trailing prompt
    lines = text.split("\n")
    lines = [l for l in lines if not l.strip().startswith("(qemu)")]
    if lines and lines[0].strip() == cmd:
        lines = lines[1:]
    return "\n".join(lines).rstrip()


def halt_all_cpus():
    """HMP `stop` halts all CPUs (ARM + c54x + others)."""
    return hmp("stop")


def resume_all_cpus():
    """HMP `cont` resumes execution."""
    return hmp("cont")


def hmp_xp(addr: int, n_bytes: int) -> str:
    """HMP examine physical memory."""
    return hmp(f"xp/{n_bytes}bx 0x{addr:x}")


def combo_write_si(si_type: int, iterations: int = 5,
                   interval_ms: int = 100,
                   gdb_host: str = "127.0.0.1", gdb_port: int = 1234):
    """Halt all CPUs, write SI<n> via gdb, verify, resume, verify."""
    print(f"\n=== Combo inject SI{si_type} x {iterations} ===")
    payload = inj.synth_si(si_type)
    print(f"  payload : {payload.hex()}")

    # Open gdb session (without ensure_gdbstub via monitor, it's already up)
    g = inj.open_session(host=gdb_host, port=gdb_port, activate=False)
    if g is None:
        raise RuntimeError(f"gdb-stub {gdb_host}:{gdb_port} unreachable")

    try:
        for i in range(iterations):
            # 1. HMP stop = halts ALL CPUs (ARM + c54x)
            r = halt_all_cpus()
            # 2. GDB writes (safe from race)
            ok = inj.inject_a_cd(g, payload)
            # 3. Readback via gdb (= still under halt)
            data = g.readmem(inj.ADDR_A_CD, len(payload))
            survives = data.hex() == payload.hex()
            print(f"  iter {i+1}/{iterations} : write_ok={ok} halt_readback_match={survives}")
            # 4. Resume
            resume_all_cpus()
            # 5. Wait + readback while running
            time.sleep(interval_ms / 1000)
            data2 = g.readmem(inj.ADDR_A_CD, len(payload))
            run_match = data2.hex() == payload.hex()
            print(f"            run_readback_match={run_match} actual={data2.hex()[:40]}...")
    finally:
        inj.close_session(g)


def main():
    ap = argparse.ArgumentParser(description="Combo HMP halt + GDB inject")
    ap.add_argument("--action", choices=["probe", "si", "fbsb", "all"],
                    default="si")
    ap.add_argument("--types", default="3,4", help="SI types to inject")
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--interval-ms", type=int, default=100)
    ap.add_argument("--gdb-host", default="127.0.0.1")
    ap.add_argument("--gdb-port", type=int, default=1234)
    ap.add_argument("--mon-sock", default=MON_SOCK_DEFAULT)
    args = ap.parse_args()

    global MON_SOCK
    MON_SOCK = args.mon_sock

    if args.action == "probe":
        # Halt + probe + resume
        print("HMP stop ...")
        print(halt_all_cpus())
        print()
        g = inj.open_session(host=args.gdb_host, port=args.gdb_port,
                             activate=False)
        snap = inj.probe_ndb(g)
        for k, v in snap.items():
            print(f"  {k:20s} = {v}")
        inj.close_session(g)
        print("\nHMP cont ...")
        print(resume_all_cpus())

    elif args.action == "si":
        for si in [int(x) for x in args.types.split(",")]:
            combo_write_si(si, iterations=args.iters,
                           interval_ms=args.interval_ms,
                           gdb_host=args.gdb_host, gdb_port=args.gdb_port)

    elif args.action == "fbsb":
        print("=== Combo halt + FBSB inject ===")
        g = inj.open_session(host=args.gdb_host, port=args.gdb_port,
                             activate=False)
        try:
            for i in range(args.iters):
                halt_all_cpus()
                inj.inject_fbsb_fb_found(g)
                snap = inj.probe_ndb(g)
                print(f"  iter {i+1} : halt d_fb_det={snap['d_fb_det']}")
                resume_all_cpus()
                time.sleep(args.interval_ms / 1000)
                snap2 = inj.probe_ndb(g)
                print(f"            run  d_fb_det={snap2['d_fb_det']}")
        finally:
            inj.close_session(g)

    elif args.action == "all":
        # Combo all : FBSB + SI3 + SI4
        for fn in ["fbsb", "si"]:
            args.action = fn
            main()


if __name__ == "__main__":
    main()
