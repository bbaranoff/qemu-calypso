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
ARM_CPSR = 25  # GDB ARM target XML usually exposes CPSR at index 25

# ────────────────── BP address resolution (default vs discovery) ──────────────────
# Defaults: hardcoded, valid for layer1.highram.elf build of 2026-04-07
DEFAULT_BP_FBDET_RESP = 0x00826434  # l1s_fbdet_resp + 0x10  (after ldrh r8,[r3,#72])
DEFAULT_BP_FBSB_COMPL = 0x00826754  # l1a_fb_compl   + 0x08  (after attempt load)
# New BPs from disas of l1s_fbdet_resp:
DEFAULT_BP_FREQ_BGE   = 0x008265bc  # bge 0x8265d8 — skip to fall-through (FB0)
DEFAULT_BP_SNR_CMP    = 0x008265c4  # cmp r3, #0 — patch r3=1 so bne taken (FB0)
# Schedule-redirect: 0x826704 is `bl tdma_schedule_set` reached by ALL success
# paths (FB0→FB1, FB1→SB, FB1 retry). Patch r0=L1_COMPL_FB(=0) and PC=0x82670c
# (bl l1s_compl_sched) to force the FBSB completion callback.
DEFAULT_BP_SCHED_REDIRECT = 0x00826704
COMPL_SCHED_PC = 0x0082670c  # bl l1s_compl_sched in l1s_fbdet_resp

DEFAULT_FW = "/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"

# Virtual attack interval (seconds since GO). Outside this window, the script
# observes but does NOT patch registers/PC. Override via env:
#   HACK_ATTACK_FROM=2.0   (start patching after t+2.0s, default 0 = immediately)
#   HACK_ATTACK_UNTIL=999  (stop patching after t+999s, default infinity)
ATTACK_FROM  = float(os.environ.get("HACK_ATTACK_FROM",  "0"))
ATTACK_UNTIL = float(os.environ.get("HACK_ATTACK_UNTIL", "1e9"))

def discover_bps(fw_path: str):
    """Discover BP addresses from ELF symbol table via objdump.
    Returns (bp_fbdet, bp_fbcompl). Falls back to defaults on error.
    """
    syms = {}
    try:
        out = subprocess.check_output(
            ["objdump", "-t", fw_path], stderr=subprocess.DEVNULL, text=True
        )
        for line in out.splitlines():
            parts = line.split()
            if len(parts) < 6:
                continue
            try:
                addr = int(parts[0], 16)
            except ValueError:
                continue
            name = parts[-1]
            syms[name] = addr
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        print(f"[hack] discovery failed ({e}), using defaults", file=sys.stderr)
        return DEFAULT_BP_FBDET_RESP, DEFAULT_BP_FBSB_COMPL

    fbdet = syms.get("l1s_fbdet_resp")
    fbcompl = syms.get("l1a_fb_compl")
    if fbdet is None or fbcompl is None:
        print("[hack] symbols not found, using defaults", file=sys.stderr)
        return DEFAULT_BP_FBDET_RESP, DEFAULT_BP_FBSB_COMPL
    return fbdet + 0x10, fbcompl + 0x08

def _env_addr(name, default):
    v = os.environ.get(name)
    if not v:
        return default
    return int(v, 0)

# Resolution order:
#   1. Explicit env override:  HACK_BP_FBDET / HACK_BP_FBCOMPL
#   2. HACK_DISCOVER=1: read ELF symbol table at HACK_FW (default firmware path)
#   3. Hardcoded defaults
if os.environ.get("HACK_DISCOVER") == "1":
    fw = os.environ.get("HACK_FW", DEFAULT_FW)
    BP_FBDET_RESP, BP_FBSB_COMPL = discover_bps(fw)
    BP_SOURCE = f"discovered from {fw}"
else:
    BP_FBDET_RESP = DEFAULT_BP_FBDET_RESP
    BP_FBSB_COMPL = DEFAULT_BP_FBSB_COMPL
    BP_SOURCE = "hardcoded defaults"

# These two are always hardcoded (no symbol in objdump for the cmp/bge offsets)
BP_FREQ_BGE       = DEFAULT_BP_FREQ_BGE
BP_SNR_CMP        = DEFAULT_BP_SNR_CMP
BP_SCHED_REDIRECT = DEFAULT_BP_SCHED_REDIRECT

BP_FBDET_RESP = _env_addr("HACK_BP_FBDET",   BP_FBDET_RESP)
BP_FBSB_COMPL = _env_addr("HACK_BP_FBCOMPL", BP_FBSB_COMPL)
BP_FREQ_BGE   = _env_addr("HACK_BP_FREQ",    BP_FREQ_BGE)
BP_SNR_CMP    = _env_addr("HACK_BP_SNR",     BP_SNR_CMP)
if "HACK_BP_FBDET" in os.environ or "HACK_BP_FBCOMPL" in os.environ:
    BP_SOURCE = "env override"

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
        # Use SW BPs (Z0) — unlimited, more reliable than HW (Z1) on QEMU gdbstub.
        r = self.cmd(f"Z0,{addr:x},{kind}")
        if r != "OK":
            print(f"[hack] Z0@{addr:#x} → {r!r}", file=sys.stderr)

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

    print(f"[hack] ░ arming hardware breakpoints ({BP_SOURCE}) ...")
    g.set_hw_bp(BP_FBDET_RESP)
    print(f"[hack]   ✓ HW#1  {BP_FBDET_RESP:#010x}  l1s_fbdet_resp+0x10  →  r8 := 1   (force d_fb_det == 1)")
    g.set_hw_bp(BP_FREQ_BGE)
    print(f"[hack]   ✓ HW#2  {BP_FREQ_BGE:#010x}  bge after cmp |freq_diff|,thresh1  →  PC := PC+4   (skip bge, force PASS)")
    g.set_hw_bp(BP_SNR_CMP)
    print(f"[hack]   ✓ HW#3  {BP_SNR_CMP:#010x}  cmp r3,#0 (snr)  →  r3 := 1   (force snr > 0, take success bne)")
    g.set_hw_bp(BP_SCHED_REDIRECT)
    print(f"[hack]   ✓ HW#4  {BP_SCHED_REDIRECT:#010x}  bl tdma_schedule_set (success exits)  →  r0:=0, PC:={COMPL_SCHED_PC:#x}  (jump to bl l1s_compl_sched)")
    g.set_hw_bp(BP_FBSB_COMPL)
    print(f"[hack]   ✓ HW#5  {BP_FBSB_COMPL:#010x}  l1a_fb_compl+0x08    →  r3 := 0   (force attempt < 13  ⇒  FBSB result=0)")
    print(f"[hack] ░ state engaged: [FB-FORCE] [FREQ-SKIP] [SNR-FORCE] [SCHED-REDIRECT] [COMPL-OVERRIDE]")
    print(f"[hack] ░ resuming target ... 5")
    for i in (4, 3, 2, 1):
        time.sleep(0.15)
        print(f"[hack]                          {i}")
    time.sleep(0.15)
    print(f"[hack] ░ GO ─────────────────────────────────────────────")

    n_fb_force = 0
    n_freq_skip = 0
    n_snr_force = 0
    n_sched_redir = 0
    n_compl_force = 0
    n_observed_only = 0
    t_start = time.time()
    print(f"[hack] ░ attack window: t+{ATTACK_FROM:.1f}s → t+{ATTACK_UNTIL:.1f}s")

    while True:
        # Continue
        stop = g.cont()
        # Stop reasons: T05 thread:01;hwbreak:; or S05 etc.
        if not stop.startswith(("T", "S")):
            print(f"[hack] unexpected stop: {stop!r}")
            break
        pc = g.read_pc()

        elapsed = time.time() - t_start
        attack_active = ATTACK_FROM <= elapsed <= ATTACK_UNTIL
        tag = "" if attack_active else "  [obs]"

        if pc == BP_FBDET_RESP:
            r1 = g.read_reg(ARM_R1)
            r2 = g.read_reg(ARM_R2)
            attempt = r1 & 0xff
            fb_mode = r2 & 0xffff
            old_r8 = g.read_reg(ARM_R8)
            if attack_active:
                g.write_reg(ARM_R8, 1)
                n_fb_force += 1
            else:
                n_observed_only += 1
            print(
                f"[hack] ▲ [{n_fb_force:04d}] FB-FORCE{tag}   "
                f"pc={pc:#010x}  r8={old_r8}{'→1' if attack_active else ''}  "
                f"attempt={attempt} fb_mode={fb_mode}  t+{elapsed:6.2f}s"
            )

        elif pc == BP_FREQ_BGE:
            if attack_active:
                g.write_reg(ARM_PC, pc + 4)
                n_freq_skip += 1
            else:
                n_observed_only += 1
            print(
                f"[hack] ⇒ [{n_freq_skip:04d}] FREQ-SKIP{tag}  "
                f"pc={pc:#010x}{'→'+hex(pc+4) if attack_active else ''}  "
                f"t+{elapsed:6.2f}s"
            )

        elif pc == BP_SNR_CMP:
            old_r3 = g.read_reg(ARM_R3)
            if attack_active:
                g.write_reg(ARM_R3, 1)
                n_snr_force += 1
            else:
                n_observed_only += 1
            print(
                f"[hack] ◆ [{n_snr_force:04d}] SNR-FORCE{tag}  "
                f"pc={pc:#010x}  r3={old_r3}{'→1' if attack_active else ''}  "
                f"t+{elapsed:6.2f}s"
            )

        elif pc == BP_SCHED_REDIRECT:
            if attack_active:
                g.write_reg(ARM_R0, 0)
                g.write_reg(ARM_PC, COMPL_SCHED_PC)
                n_sched_redir += 1
            else:
                n_observed_only += 1
            print(
                f"[hack] ✦ [{n_sched_redir:04d}] SCHED-REDIRECT{tag} "
                f"pc={pc:#010x}{'→'+hex(COMPL_SCHED_PC) if attack_active else ''}  "
                f"t+{elapsed:6.2f}s"
            )

        elif pc == BP_FBSB_COMPL:
            old_r3 = g.read_reg(ARM_R3)
            if attack_active:
                g.write_reg(ARM_R3, 0)
                n_compl_force += 1
            else:
                n_observed_only += 1
            elapsed = time.time() - t_start
            print(
                f"[hack] ★ [{n_compl_force:04d}] COMPL-FORCE "
                f"pc={pc:#010x}  r3 {old_r3}→0  "
                f"⇒ FBSB_CONF result=0  t+{elapsed:6.2f}s"
            )
            print(
                f"[hack]   └─ dashboard: FB={n_fb_force} FREQ={n_freq_skip} "
                f"SNR={n_snr_force} SCHED={n_sched_redir} COMPL={n_compl_force} "
                f"rate={n_fb_force/max(elapsed,0.01):.1f}/s"
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
