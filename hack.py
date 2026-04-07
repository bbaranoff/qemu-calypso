#!/usr/bin/env python3
"""
hack.py — direct QEMU GDB-stub client for the Calypso emulator.

Connects to the QEMU gdbstub on tcp::1234, sets two hardware breakpoints
inside the OsmocomBB layer1 firmware, and forces the FB-detection path
to always succeed by patching ARM registers at the right moments.

Targets (from disas of layer1.highram.elf, build of 2026-04-07):

  l1s_fbdet_resp+0x10 = 0x00826434
      ldrh r8, [r3, #72]   ; r8 = dsp_api.ndb->d_fb_det
      lsl  r2, r2, #16
      cmp  r8, #0          ; if d_fb_det == 0 → fail path
      bne  +90 <fb_found>  ; else → fb_found
      → at 0x826434, r8 has just been loaded; we set r8 = 1.

  l1a_fb_compl+0x8    = 0x00826754
      ldr  r3, [r3, #4]    ; r3 = last_fb->attempt
      cmp  r3, #12         ; if attempt > 12 → result=255
      bgt  +20             ; else → fbinfo2cellinfo + l1ctl_fbsb_resp(0)
      → at 0x826754, r3 has just been loaded; we set r3 = 0.

If both patches fire, the firmware sends FBSB_CONF result=0 to mobile.

Usage:
    python3 hack.py [host[:port]]    # default 127.0.0.1:1234

This is a debug-only contraption, kept here so the rest of the codebase
stays free of hacks. Run it in parallel with run_debug.sh (with QEMU
started under -S -gdb tcp::1234).
"""

import socket
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 1234

# ARM register indices in the GDB Remote Serial Protocol stream.
ARM_R0  = 0
ARM_R1  = 1
ARM_R3  = 3
ARM_R8  = 8
ARM_PC  = 15
ARM_CPSR = 25  # GDB ARM target XML usually exposes CPSR at index 25

BP_FBDET_RESP = 0x00826434
BP_FBSB_COMPL = 0x00826754

# NDB shared memory (ARM side). dsp_api.ndb base = 0xFFD001A8
# d_fb_det at +0x48 (word 36) = 0xFFD001F0
# d_fb_mode at +0x4A         = 0xFFD001F2
NDB_D_FB_DET  = 0xFFD001F0
NDB_D_FB_MODE = 0xFFD001F2

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
        """Read one full $...#XX packet, ignoring acks."""
        while True:
            while b"$" not in self.buf:
                self.buf += self._read()
            i = self.buf.index(b"$")
            # drop everything before $ (acks, leftovers)
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

    # ── helpers ──

    def set_hw_bp(self, addr: int, kind: int = 4) -> None:
        r = self.cmd(f"Z1,{addr:x},{kind}")
        if r != "OK":
            print(f"[hack] Z1@{addr:#x} → {r!r}", file=sys.stderr)

    def cont(self) -> str:
        return self.cmd("c")

    def read_reg(self, idx: int) -> int:
        r = self.cmd(f"p{idx:x}")
        # ARM regs are 4 bytes little-endian hex
        return int.from_bytes(bytes.fromhex(r[:8]), "little")

    def write_reg(self, idx: int, value: int) -> None:
        v = value.to_bytes(4, "little").hex()
        r = self.cmd(f"P{idx:x}={v}")
        if r != "OK":
            print(f"[hack] P{idx}={v} → {r!r}", file=sys.stderr)

    def read_pc(self) -> int:
        return self.read_reg(ARM_PC)

    def write_mem(self, addr: int, data: bytes) -> bool:
        r = self.cmd(f"M{addr:x},{len(data):x}:{data.hex()}")
        if r != "OK":
            print(f"[hack] M@{addr:#x} → {r!r} (probably MMIO, ignored)", file=sys.stderr)
            return False
        return True

    def write_u16(self, addr: int, value: int) -> bool:
        return self.write_mem(addr, value.to_bytes(2, "little"))

    def read_mem(self, addr: int, n: int) -> bytes:
        r = self.cmd(f"m{addr:x},{n:x}")
        try:
            return bytes.fromhex(r)
        except ValueError:
            return b""

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

    banner = r"""
   ┌────────────────────────────────────────────────────────────┐
   │   hack.py — Calypso FBSB forcer (GDB-stub direct RSP)      │
   │   "le hack, mais uniquement ici. PAS AILLEURS !!!"         │
   └────────────────────────────────────────────────────────────┘
    """
    print(banner)
    print(f"[hack] ░ connecting to gdbstub @ {host}:{port} ...")
    t0 = time.time()
    g = GdbStub(host, port)
    print(f"[hack] ✓ connected in {(time.time()-t0)*1000:.1f} ms")

    # Initial query
    s = g.cmd("?")
    print(f"[hack] ░ initial stop reply: {s}")
    pc0 = g.read_pc()
    print(f"[hack] ░ ARM PC = {pc0:#010x}")

    print(f"[hack] ░ arming hardware breakpoints ...")
    g.set_hw_bp(BP_FBDET_RESP)
    print(f"[hack]   ✓ HW#1  {BP_FBDET_RESP:#010x}  l1s_fbdet_resp+0x10  →  r8 := 1   (force d_fb_det == 1)")
    g.set_hw_bp(BP_FBSB_COMPL)
    print(f"[hack]   ✓ HW#2  {BP_FBSB_COMPL:#010x}  l1a_fb_compl+0x08    →  r3 := 0   (force attempt == 0  ⇒  FBSB result=0)")
    print(f"[hack] ░ state engaged: [FB-FORCE armed] [COMPL-OVERRIDE armed]")
    print(f"[hack] ░ resuming target ... 5")
    for i in (4, 3, 2, 1):
        time.sleep(0.15)
        print(f"[hack]                          {i}")
    time.sleep(0.15)
    print(f"[hack] ░ GO ─────────────────────────────────────────────")

    n_fb_force = 0
    n_compl_force = 0
    t_start = time.time()

    while True:
        # Continue
        stop = g.cont()
        # Stop reasons: T05 thread:01;hwbreak:; or S05 etc.
        if not stop.startswith(("T", "S")):
            print(f"[hack] unexpected stop: {stop!r}")
            break
        pc = g.read_pc()

        if pc == BP_FBDET_RESP:
            r1 = g.read_reg(ARM_R1)
            attempt = r1 & 0xff
            old_r8 = g.read_reg(ARM_R8)
            g.write_reg(ARM_R8, 1)
            # NB: ne pas tenter d'écrire NDB via M packet — c'est du MMIO
            # côté QEMU et ça ferme le gdbstub. Le patch r8 suffit.
            n_fb_force += 1
            elapsed = time.time() - t_start
            print(
                f"[hack] ▲ [{n_fb_force:04d}] FB-FORCE   "
                f"pc={pc:#010x}  r8 {old_r8}→1  attempt={attempt}  "
                f"t+{elapsed:6.2f}s"
            )

        elif pc == BP_FBSB_COMPL:
            old_r3 = g.read_reg(ARM_R3)
            g.write_reg(ARM_R3, 0)
            n_compl_force += 1
            elapsed = time.time() - t_start
            print(
                f"[hack] ★ [{n_compl_force:04d}] COMPL-FORCE "
                f"pc={pc:#010x}  r3 {old_r3}→0  "
                f"⇒ FBSB_CONF result=0  t+{elapsed:6.2f}s"
            )
            print(
                f"[hack]   └─ dashboard: FB-forces={n_fb_force}  "
                f"COMPL-forces={n_compl_force}  rate={n_fb_force/max(elapsed,0.01):.1f}/s"
            )

        else:
            print(f"[hack] stop at unexpected pc={pc:#010x}")
            # remove ourselves and continue
            break

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[hack] ░ interrupted by user — au revoir")
        sys.exit(0)
