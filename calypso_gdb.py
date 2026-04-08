#!/usr/bin/env python3
"""
calypso_gdb.py — clean GDB Remote Serial Protocol client for Calypso QEMU.

Connects to QEMU's gdbstub on tcp::1234, sets observation-only breakpoints
inside the OsmocomBB layer1 firmware, and logs register values when the
breakpoints fire. NEVER patches registers or PC. Pure observation.

For the bourrin version that forces FBSB success by patching registers,
use calypso_hack_gdb.py instead.

Usage:
    python3 calypso_gdb.py [host[:port]]    # default 127.0.0.1:1234

Run alongside run_debug.sh (with QEMU started under -S -gdb tcp::1234).
"""

import os
import socket
import subprocess
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 1234

# ARM register indices in the GDB Remote Serial Protocol stream.
ARM_R0  = 0
ARM_R1  = 1
ARM_R2  = 2
ARM_R3  = 3
ARM_R8  = 8
ARM_PC  = 15
ARM_CPSR = 25

# Default observation breakpoints (no patching)
DEFAULT_FW = "/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BP_FBDET_RESP_ENTRY = 0x00826434  # l1s_fbdet_resp+0x10 (after ldrh r8)
BP_FBSB_COMPL_ENTRY = 0x00826754  # l1a_fb_compl+0x08
BP_FB1_FREQ         = 0x008266c8  # FB1 freq cmp
BP_FB1_SB_TEST      = 0x00826630  # FB1 SB-test beq

# ────────────────────────── RSP protocol ──────────────────────────

def _checksum(payload: bytes) -> bytes:
    s = sum(payload) & 0xff
    return f"{s:02x}".encode()

def rsp_pack(payload: str) -> bytes:
    p = payload.encode()
    return b"$" + p + b"#" + _checksum(p)

class GdbStub:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.buf = b""

    def _read(self, n: int = 4096) -> bytes:
        data = self.sock.recv(n)
        if not data:
            raise EOFError("gdbstub closed")
        return data

    def _read_packet(self) -> str:
        while True:
            while b"$" not in self.buf:
                self.buf += self._read()
            i = self.buf.index(b"$")
            self.buf = self.buf[i:]
            i = 0
            while b"#" not in self.buf[i+1:]:
                self.buf += self._read()
            try:
                j = self.buf.index(b"#", i + 1)
            except ValueError:
                self.buf += self._read()
                continue
            if len(self.buf) < j + 3:
                self.buf += self._read()
                continue
            payload = self.buf[i + 1 : j]
            self.buf = self.buf[j + 3 :]
            self.sock.sendall(b"+")
            return payload.decode(errors="replace")

    def send(self, payload: str) -> None:
        self.sock.sendall(rsp_pack(payload))

    def cmd(self, payload: str) -> str:
        self.send(payload)
        return self._read_packet()

    def set_sw_bp(self, addr: int, kind: int = 4) -> None:
        r = self.cmd(f"Z0,{addr:x},{kind}")
        if r != "OK":
            print(f"[obs] Z0@{addr:#x} → {r!r}", file=sys.stderr)

    def cont(self) -> str:
        return self.cmd("c")

    def read_reg(self, idx: int) -> int:
        r = self.cmd(f"p{idx:x}")
        return int.from_bytes(bytes.fromhex(r[:8]), "little")

    def read_pc(self) -> int:
        return self.read_reg(ARM_PC)

# ────────────────────────── main loop ──────────────────────────

def main() -> int:
    host, port = DEFAULT_HOST, DEFAULT_PORT
    if len(sys.argv) > 1:
        s = sys.argv[1]
        if ":" in s:
            host, p = s.split(":", 1)
            port = int(p)
        else:
            host = s

    print("calypso_gdb.py — observation-only GDB client")
    print(f"[obs] connecting to gdbstub @ {host}:{port} ...")
    g = GdbStub(host, port)
    print(f"[obs] connected")

    s = g.cmd("?")
    print(f"[obs] initial stop: {s}")
    pc0 = g.read_pc()
    print(f"[obs] ARM PC = {pc0:#010x}")

    print("[obs] arming observation breakpoints (Z0, no patching) ...")
    g.set_sw_bp(BP_FBDET_RESP_ENTRY)
    print(f"[obs]   {BP_FBDET_RESP_ENTRY:#010x}  l1s_fbdet_resp+0x10")
    g.set_sw_bp(BP_FBSB_COMPL_ENTRY)
    print(f"[obs]   {BP_FBSB_COMPL_ENTRY:#010x}  l1a_fb_compl+0x08")
    g.set_sw_bp(BP_FB1_FREQ)
    print(f"[obs]   {BP_FB1_FREQ:#010x}  FB1 freq cmp")
    g.set_sw_bp(BP_FB1_SB_TEST)
    print(f"[obs]   {BP_FB1_SB_TEST:#010x}  FB1 SB-test beq")
    print("[obs] resuming target")

    counts = {
        BP_FBDET_RESP_ENTRY: 0,
        BP_FBSB_COMPL_ENTRY: 0,
        BP_FB1_FREQ: 0,
        BP_FB1_SB_TEST: 0,
    }
    names = {
        BP_FBDET_RESP_ENTRY: "FBDET_RESP",
        BP_FBSB_COMPL_ENTRY: "FBSB_COMPL",
        BP_FB1_FREQ:         "FB1_FREQ",
        BP_FB1_SB_TEST:      "FB1_SB_TEST",
    }
    t_start = time.time()

    while True:
        stop = g.cont()
        if not stop.startswith(("T", "S")):
            print(f"[obs] unexpected stop: {stop!r}")
            break
        pc = g.read_pc()
        elapsed = time.time() - t_start

        if pc not in counts:
            print(f"[obs] stop at unexpected pc={pc:#010x}")
            break

        counts[pc] += 1
        # Read interesting registers for context
        r0 = g.read_reg(ARM_R0)
        r1 = g.read_reg(ARM_R1)
        r2 = g.read_reg(ARM_R2)
        r3 = g.read_reg(ARM_R3)
        r8 = g.read_reg(ARM_R8)
        print(
            f"[obs] [{counts[pc]:04d}] {names[pc]:11s} "
            f"pc={pc:#010x} r0={r0:#x} r1={r1:#x} r2={r2:#x} r3={r3:#x} r8={r8:#x} "
            f"t+{elapsed:6.2f}s"
        )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[obs] interrupted")
        sys.exit(0)
