#!/usr/bin/env python3
"""
inject.py — Module + CLI : injection NDB / a_cd[] / FBSB / SI / tasks via
le gdb-stub QEMU Calypso (port 1234, activé via monitor HMP).

Utilisation comme module (depuis validating.py ou autres) :

    import inject
    g = inject.open_session()           # active gdbstub + connect
    inject.inject_fbsb_fb_found(g)      # mime publish_fb_found
    inject.inject_si(g, 4)              # push SI4 dans a_cd[]
    inject.close_session(g)

Ou en CLI (équivalent à l'ancien voie2.py) :

    python3 inject.py --action probe
    python3 inject.py --action fbsb-fb --iterations 50 --interval-ms 80
    python3 inject.py --action si4 --iterations 30
    python3 inject.py --action all
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import subprocess
import sys
import time
from contextlib import contextmanager
from typing import Optional

# -----------------------------------------------------------------------------
# Defaults & registers
# -----------------------------------------------------------------------------
GDB_HOST_DEFAULT = "127.0.0.1"
GDB_PORT_DEFAULT = 1234
MON_SOCK_DEFAULT = "/tmp/qemu-calypso-mon.sock"
QEMU_LOG_DEFAULT = "/root/qemu.log"

# Calypso ARM-physical addresses (CALYPSO_DSP_BASE=0xFFD00000)
ADDR_D_FB_DET   = 0xFFD001F0   # DSP word 0x08F8
ADDR_D_FB_MODE  = 0xFFD001F2   # DSP word 0x08F9
ADDR_A_SYNC_TOA = 0xFFD001F4
ADDR_A_SYNC_PM  = 0xFFD001F6
ADDR_A_SYNC_ANG = 0xFFD001F8
ADDR_A_SYNC_SNR = 0xFFD001FA
ADDR_A_CD       = 0xFFD003A0   # a_cd[0..14] = 30 bytes (15 words)
ADDR_D_BURST_D  = 0xFFD00002   # DB_W_D_BURST_D page0 (×2 = byte off)
ADDR_D_TASK_D   = 0xFFD00000   # DB_W_D_TASK_D page0
ADDR_D_TASK_MD  = 0xFFD00008   # DB_W_D_TASK_MD page0
ADDR_D_TASK_RA  = 0xFFD0000E   # DB_W_D_TASK_RA page0
ADDR_INTH_BASE  = 0xFFFFFA00
ADDR_INTH_MASKL = 0xFFFFFA08

# DSP task IDs (cf. firmware osmocom-bb dsp.h)
TASK_NONE  = 0
TASK_PM    = 1
TASK_NB    = 2
TASK_FB    = 5
TASK_SB    = 6
TASK_ALLC  = 24  # CCCH demod
TASK_RACH  = 28

__all__ = [
    "GdbRemote",
    "open_session", "close_session", "session",
    "hmp", "gdbstub_active", "ensure_gdbstub",
    "discover_gdb_endpoint",
    "probe_ndb",
    "inject_fbsb_fb_found", "inject_fbsb_sb_found",
    "inject_a_cd", "inject_si",
    "inject_d_task", "inject_d_burst",
    "inject_clear_ndb",
    "synth_si",
]


# -----------------------------------------------------------------------------
# Discovery — find a reachable gdb-stub endpoint
# -----------------------------------------------------------------------------
def _docker_container_ips(name: str) -> list[str]:
    """Return list of IPs of a docker container, empty on error."""
    try:
        rc = subprocess.run(
            ["docker", "inspect", name, "--format",
             "{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}"],
            capture_output=True, text=True, timeout=2)
        return [ip for ip in rc.stdout.strip().split() if ip]
    except Exception:
        return []


def discover_gdb_endpoint(port: int = GDB_PORT_DEFAULT,
                          container: str = "trying") -> Optional[tuple[str, int]]:
    """Probe candidate hosts to find one with a reachable gdb-stub.

    Order :
      1. 127.0.0.1 (works if pytest runs inside the container or if a port
         mapping exists)
      2. localhost docker host gateway (host.docker.internal — meaningless
         on Linux usually, skip)
      3. Each IP of the named docker container (docker network bridges)

    Returns (host, port) or None if none reachable.
    """
    candidates = ["127.0.0.1"]
    candidates.extend(_docker_container_ips(container))
    seen = set()
    for host in candidates:
        if host in seen: continue
        seen.add(host)
        if gdbstub_active(host, port):
            return (host, port)
    return None

# -----------------------------------------------------------------------------
# gdb-remote protocol
# -----------------------------------------------------------------------------
class GdbRemote:
    def __init__(self, host: str = GDB_HOST_DEFAULT, port: int = GDB_PORT_DEFAULT,
                 timeout: float = 3.0):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.sock = socket.socket()
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(self.timeout)

    def close(self):
        try:
            if self.sock: self.sock.close()
        except Exception: pass

    def _send(self, body):
        if isinstance(body, str): body = body.encode()
        csum = sum(body) % 256
        self.sock.sendall(b"$" + body + b"#" + ("%02x" % csum).encode())
        # consume ack
        self.sock.recv(1)

    def _recv(self) -> bytes:
        buf = b""
        while True:
            c = self.sock.recv(1)
            if not c: return b""
            if c in (b"+", b"-"): continue
            if c == b"$": break
        while True:
            c = self.sock.recv(1)
            if c == b"#":
                self.sock.recv(2)
                self.sock.sendall(b"+")
                return buf
            buf += c

    def rpc(self, cmd: str) -> bytes:
        self._send(cmd); return self._recv()

    def writemem(self, addr: int, data: bytes) -> bool:
        r = self.rpc(f"M{addr:x},{len(data):x}:{data.hex()}")
        return r == b"OK"

    def readmem(self, addr: int, length: int) -> bytes:
        r = self.rpc(f"m{addr:x},{length:x}")
        try: return bytes.fromhex(r.decode())
        except Exception: return b""

    def cont(self):
        """Send continue ('c'). Tolerant of missing ACK (target may already be running)."""
        body = b"c"
        csum = sum(body) % 256
        try:
            self.sock.sendall(b"$" + body + b"#" + ("%02x" % csum).encode())
        except Exception:
            return
        old = self.sock.gettimeout()
        self.sock.settimeout(0.3)
        try:
            self.sock.recv(1)
        except (socket.timeout, OSError):
            pass
        finally:
            try: self.sock.settimeout(old)
            except Exception: pass

    def stop(self) -> bytes:
        """Send Ctrl-C interrupt. Tolerant if target already halted (no T05 reply)."""
        self.sock.sendall(b"\x03")
        old = self.sock.gettimeout()
        self.sock.settimeout(0.5)
        try:
            return self._recv()
        except socket.timeout:
            return b""  # already halted — no reply
        finally:
            self.sock.settimeout(old)

    def halt_reason(self) -> bytes:
        return self.rpc("?")

# -----------------------------------------------------------------------------
# QEMU monitor HMP — to (re)activate gdbserver from outside
# -----------------------------------------------------------------------------
def hmp(cmd: str, sock_path: str = MON_SOCK_DEFAULT, timeout: float = 2.0) -> str:
    try:
        s = socket.socket(socket.AF_UNIX)
        s.connect(sock_path)
        s.settimeout(timeout)
        s.sendall(cmd.encode() + b"\n")
        out = b""
        t_end = time.time() + timeout
        while time.time() < t_end:
            try:
                d = s.recv(4096)
                if not d: break
                out += d
            except socket.timeout: break
        s.close()
        return out.decode(errors="replace")
    except Exception as e:
        return f"<hmp error: {e}>"

def gdbstub_active(host: str = GDB_HOST_DEFAULT, port: int = GDB_PORT_DEFAULT) -> bool:
    try:
        s = socket.socket(); s.settimeout(0.5); s.connect((host, port)); s.close()
        return True
    except Exception:
        return False

def ensure_gdbstub(host: str = GDB_HOST_DEFAULT, port: int = GDB_PORT_DEFAULT,
                   mon_sock: str = MON_SOCK_DEFAULT, verbose: bool = True) -> bool:
    if gdbstub_active(host, port):
        if verbose: print(f"[gdb] already on {host}:{port}")
        return True
    if verbose: print(f"[gdb] activating via {mon_sock}")
    hmp(f"gdbserver tcp::{port}", mon_sock)
    time.sleep(0.5)
    return gdbstub_active(host, port)

# -----------------------------------------------------------------------------
# Session helpers
# -----------------------------------------------------------------------------
def open_session(host: str = GDB_HOST_DEFAULT, port: int = GDB_PORT_DEFAULT,
                 mon_sock: str = MON_SOCK_DEFAULT, activate: bool = True,
                 verbose: bool = False) -> Optional[GdbRemote]:
    """Connect + (optionnel) activate gdbstub. Returns None on failure."""
    if activate and not ensure_gdbstub(host, port, mon_sock, verbose):
        return None
    if not gdbstub_active(host, port):
        return None
    g = GdbRemote(host, port)
    try:
        g.connect()
        g.halt_reason()  # consume initial T05 stop notification
        return g
    except Exception:
        return None

def close_session(g: GdbRemote, resume: bool = True):
    """Continue exec and close socket."""
    try:
        if resume: g.cont()
    finally:
        g.close()

@contextmanager
def session(host: str = GDB_HOST_DEFAULT, port: int = GDB_PORT_DEFAULT,
            mon_sock: str = MON_SOCK_DEFAULT, activate: bool = True):
    """Context manager : `with session() as g: ...`"""
    g = open_session(host, port, mon_sock, activate)
    if g is None:
        raise RuntimeError(f"gdbstub not reachable at {host}:{port}")
    try:
        yield g
    finally:
        close_session(g)

# -----------------------------------------------------------------------------
# Probe (read-only NDB snapshot)
# -----------------------------------------------------------------------------
def probe_ndb(g: GdbRemote) -> dict:
    """Read all key NDB cells, return as dict {name: hex_str_or_int}."""
    snap = {
        "d_fb_det":   g.readmem(ADDR_D_FB_DET, 2).hex(),
        "d_fb_mode":  g.readmem(ADDR_D_FB_MODE, 2).hex(),
        "a_sync_TOA": g.readmem(ADDR_A_SYNC_TOA, 2).hex(),
        "a_sync_PM":  g.readmem(ADDR_A_SYNC_PM, 2).hex(),
        "a_sync_ANG": g.readmem(ADDR_A_SYNC_ANG, 2).hex(),
        "a_sync_SNR": g.readmem(ADDR_A_SYNC_SNR, 2).hex(),
        "a_cd[0..14]": g.readmem(ADDR_A_CD, 30).hex(),
        "inth_mask_l": g.readmem(ADDR_INTH_MASKL, 2).hex(),
    }
    return snap

# -----------------------------------------------------------------------------
# Inject primitives
# -----------------------------------------------------------------------------
def inject_fbsb_fb_found(g: GdbRemote, toa: int = 0, pm: int = 80,
                         angle: int = 0, snr: int = 100,
                         fb_mode: int = 0) -> int:
    """Mime calypso_fbsb_publish_fb_found : d_fb_det=1, a_sync_demod[*]. Returns nb cells OK."""
    cells = [
        (ADDR_D_FB_DET,   struct.pack("<H", 1)),
        (ADDR_D_FB_MODE,  struct.pack("<H", fb_mode)),
        (ADDR_A_SYNC_TOA, struct.pack("<H", toa)),
        (ADDR_A_SYNC_PM,  struct.pack("<H", pm)),
        (ADDR_A_SYNC_ANG, struct.pack("<H", angle)),
        (ADDR_A_SYNC_SNR, struct.pack("<H", snr)),
    ]
    return sum(1 for a, d in cells if g.writemem(a, d))

def inject_fbsb_sb_found(g: GdbRemote, bsic: int = 10) -> int:
    """SB found marker : d_fb_det=2 (SB level), a_sync_TOA=bsic-marker. Returns nb cells OK."""
    cells = [
        (ADDR_D_FB_DET,   struct.pack("<H", 2)),
        (ADDR_D_FB_MODE,  struct.pack("<H", 1)),       # mode 1 = SB phase
        (ADDR_A_SYNC_TOA, struct.pack("<H", bsic)),
        (ADDR_A_SYNC_SNR, struct.pack("<H", 100)),
    ]
    return sum(1 for a, d in cells if g.writemem(a, d))

def inject_a_cd(g: GdbRemote, payload23_or_30: bytes) -> bool:
    """Write a_cd[0..14] (15 words = 30 bytes). Accept 23 or 30B input.

    a_cd[] est la zone NDB où le DSP écrit le résultat brut du CCCH demod
    (soft bits ou hard bits selon firmware version). On y pousse 30 bytes.
    Input 23B (L2 frame size) est padé à 30 avec 0x2B.
    """
    if len(payload23_or_30) == 23:
        data = payload23_or_30 + bytes([0x2B] * 7)
    elif len(payload23_or_30) == 30:
        data = payload23_or_30
    else:
        raise ValueError(f"a_cd payload must be 23 or 30 bytes, got {len(payload23_or_30)}")
    return g.writemem(ADDR_A_CD, data)

def synth_si(si_type: int) -> bytes:
    """Return a 23-byte synthetic SI<n> frame (L2 + L3 layout).

    Format (L2 pseudo-length + L3 RR SI<n>) :
      [0]=0x49 (LI=18 << 2 | M=0 | EL=1)
      [1]=0x06 (RR protocol disc)
      [2]=msg_type (SI1=0x19, SI2=0x1A, SI3=0x1B, SI4=0x1C, SI5=0x05, SI6=0x06, SI13=0x00)
      [3..] payload partiel + padding 0x2B
    """
    msg_types = {1: 0x19, 2: 0x1A, 3: 0x1B, 4: 0x1C, 5: 0x05, 6: 0x06, 13: 0x00}
    if si_type not in msg_types:
        raise ValueError(f"unknown SI type {si_type}")
    body = bytearray([0x49, 0x06, msg_types[si_type]])
    # SI3/SI4 carry the LAI/CI/RACH ctrl. Provide minimal-valid bytes.
    if si_type == 3:
        body += bytes([
            0x00, 0x01,         # CI
            0x00, 0xF1, 0x10,   # MCC=001 MNC=01
            0x00, 0x01,         # LAC=1
            0x01, 0x00,         # cell options + cell select
            0x18, 0xFF, 0xFF,   # RACH ctrl
        ])
    elif si_type == 4:
        body += bytes([
            0x00, 0xF1, 0x10,   # MCC MNC
            0x00, 0x01,         # LAC
            0x00, 0x00,         # cell select
            0x18, 0xFF, 0xFF,   # RACH ctrl
        ])
    # pad to 23
    body += bytes([0x2B] * (23 - len(body)))
    return bytes(body)

def inject_si(g: GdbRemote, si_type: int) -> bool:
    """Build SI<n> and write it into a_cd[]."""
    return inject_a_cd(g, synth_si(si_type))

def inject_d_task(g: GdbRemote, task_id: int, page: int = 0) -> bool:
    """Write d_task_md (page0 or page1) — make ARM/DSP believe task X started."""
    base = ADDR_D_TASK_MD if page == 0 else (ADDR_D_TASK_MD + 0x28)
    return g.writemem(base, struct.pack("<H", task_id))

def inject_d_burst(g: GdbRemote, burst_val: int, page: int = 0) -> bool:
    """Write d_burst_d (per-burst dispatcher cell). page0 or page1."""
    base = ADDR_D_BURST_D if page == 0 else (ADDR_D_BURST_D + 0x28)
    return g.writemem(base, struct.pack("<H", burst_val))

def inject_clear_ndb(g: GdbRemote) -> int:
    """Reset d_fb_det/mode + a_sync_demod[] + a_cd[] to zero. Returns nb writes OK."""
    z2 = b"\x00\x00"
    z30 = bytes(30)
    cells = [
        (ADDR_D_FB_DET, z2), (ADDR_D_FB_MODE, z2),
        (ADDR_A_SYNC_TOA, z2), (ADDR_A_SYNC_PM, z2),
        (ADDR_A_SYNC_ANG, z2), (ADDR_A_SYNC_SNR, z2),
        (ADDR_A_CD, z30),
    ]
    return sum(1 for a, d in cells if g.writemem(a, d))

# -----------------------------------------------------------------------------
# Higher-level helpers : burst loop pattern (halt-write-resume race vs DSP)
# -----------------------------------------------------------------------------
def burst_inject(g: GdbRemote, inject_fn, iterations: int = 50,
                 interval_ms: int = 80) -> int:
    """Run inject_fn(g) `iterations` times with halt-resume cycles.

    Returns total ok writes (sum over iterations).
    """
    total_ok = 0
    for _ in range(iterations):
        rc = inject_fn(g)
        if isinstance(rc, int):
            total_ok += rc
        elif rc:
            total_ok += 1
        g.cont()
        time.sleep(interval_ms / 1000)
        g.stop()
    return total_ok

# -----------------------------------------------------------------------------
# Log observation helper (for CLI/standalone — also reusable by validating.py)
# -----------------------------------------------------------------------------
def grep_count_log(path: str, pattern: str, tail: int = 8000) -> int:
    try:
        out = subprocess.run(
            ["bash", "-c", f"tail -n {tail} {path} 2>/dev/null | grep -cE '{pattern}'"],
            capture_output=True, text=True, timeout=2)
        return int(out.stdout.strip() or "0")
    except Exception:
        return 0

# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def _action_probe(args, g: GdbRemote):
    snap = probe_ndb(g)
    for k, v in snap.items():
        print(f"  {k:14} = {v}")
    return 0

def _do_loop(args, g: GdbRemote, inject_fn, label: str):
    """Generic loop with before/after log delta accounting."""
    log = args.qemu_log
    bef = {
        "ARM_RD_d_fb_det=1": grep_count_log(log, r"ARM RD d_fb_det.*= 0x0001"),
        "task=24":           grep_count_log(log, r"task=24"),
        "DATA_IND":          grep_count_log(log, r"DATA_IND"),
        "ARM_RD_a_cd":       grep_count_log(log, r"ARM RD a_cd"),
    }
    if args.verbose:
        print(f"[{label}] BEFORE {bef}")
    t0 = time.time()
    ok = burst_inject(g, inject_fn, args.iterations, args.interval_ms)
    t1 = time.time()
    time.sleep(0.5)
    aft = {k: grep_count_log(log, p) for k, p in [
        ("ARM_RD_d_fb_det=1", r"ARM RD d_fb_det.*= 0x0001"),
        ("task=24",           r"task=24"),
        ("DATA_IND",          r"DATA_IND"),
        ("ARM_RD_a_cd",       r"ARM RD a_cd"),
    ]}
    delta = {k: aft[k] - bef[k] for k in bef}
    print(f"[{label}] iters={args.iterations} writes_ok={ok} dt={t1-t0:.1f}s")
    for k, v in delta.items():
        print(f"  Δ {k:18} = +{v}")
    return 0 if any(v > 0 for v in delta.values()) else 1

def _action_fbsb_fb(args, g): return _do_loop(args, g, inject_fbsb_fb_found, "fbsb-fb")
def _action_fbsb_sb(args, g): return _do_loop(args, g, inject_fbsb_sb_found, "fbsb-sb")
def _action_si(n):
    def f(args, g): return _do_loop(args, g, lambda gg: 1 if inject_si(gg, n) else 0, f"si{n}")
    return f
def _action_clear(args, g):
    n = inject_clear_ndb(g); print(f"cleared {n} cells"); return 0
def _action_all(args, g):
    rc = 0
    for action in ("fbsb-fb", "fbsb-sb", "si1", "si3", "si4"):
        print(f"\n--- {action} ---")
        rc |= ACTIONS[action](args, g)
    return rc

ACTIONS = {
    "probe":   _action_probe,
    "fbsb-fb": _action_fbsb_fb,
    "fbsb-sb": _action_fbsb_sb,
    "si1":     _action_si(1),
    "si2":     _action_si(2),
    "si3":     _action_si(3),
    "si4":     _action_si(4),
    "si5":     _action_si(5),
    "si6":     _action_si(6),
    "clear":   _action_clear,
    "all":     _action_all,
}

def main():
    p = argparse.ArgumentParser(
        description="QEMU Calypso NDB injection via gdb-stub.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--action", default="probe", choices=list(ACTIONS),
                   help="probe = read snapshot ; clear = zero out ; fbsb-* / si* = halt-write-resume loop ; all = run several")
    p.add_argument("--iterations", type=int, default=30)
    p.add_argument("--interval-ms", type=int, default=80)
    p.add_argument("--gdb-host", default=GDB_HOST_DEFAULT)
    p.add_argument("--gdb-port", type=int, default=GDB_PORT_DEFAULT)
    p.add_argument("--mon-sock", default=MON_SOCK_DEFAULT)
    p.add_argument("--qemu-log", default=QEMU_LOG_DEFAULT)
    p.add_argument("--no-activate", action="store_true",
                   help="don't try to enable gdbstub via monitor")
    p.add_argument("--no-keep", action="store_true",
                   help="disable gdbstub at end (default: leave active)")
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()

    if not args.no_activate:
        if not ensure_gdbstub(args.gdb_host, args.gdb_port, args.mon_sock, args.verbose):
            print(f"[!] gdbstub unreachable", file=sys.stderr); return 2
    elif not gdbstub_active(args.gdb_host, args.gdb_port):
        print(f"[!] gdbstub down (--no-activate)", file=sys.stderr); return 2

    g = GdbRemote(args.gdb_host, args.gdb_port)
    g.connect(); g.halt_reason()
    try:
        rc = ACTIONS[args.action](args, g)
    finally:
        try: g.cont()
        finally: g.close()
        if args.no_keep:
            hmp("gdbserver none", args.mon_sock)
            if args.verbose: print("[gdb] deactivated (--no-keep)")
        elif args.verbose:
            print(f"[gdb] left active on :{args.gdb_port}")
    sys.exit(rc)

if __name__ == "__main__":
    main()
