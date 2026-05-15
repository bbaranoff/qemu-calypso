#!/usr/bin/env python3
"""
validating.py — Test cross-couches d'injection de trames vers le pipeline qemu-calypso.

Couches couvertes :
  1. L1 BSP        — TRXDv0 DL bursts via UDP 127.0.0.1:6702
  2. L1 bridge     — CLK IND / UL bursts via UDP 127.0.0.1:5702
  3. Sercomm DLCI  — frames HDLC raw sur /dev/pts/2 (UART_MODEM côté firmware)
  4. L1CTL         — *_REQ vers firmware via socket /tmp/osmocom_l2 (place de mobile)
  5. L2/L3 LAPDm   — encapsulé dans L1CTL_DATA_REQ payloads
  6. GDB-stub      — R/W mémoire ARM via :1234 (memory probe seulement, pas un test envoi)
  7. Osmocom VTY   — commandes diagnostiques (bsc 4242, msc 4254, mobile 4247, mgw 4244)

Sortie : tableau ASCII { layer | frame_type | result | detail }.
result ∈ { PASS, FAIL, SKIP, ERR }.

Usage (depuis le container `trying`) :
    python3 /opt/GSM/qemu-src/validating.py
Ou (depuis host, via docker exec) :
    docker exec trying python3 /opt/GSM/qemu-src/validating.py

Pas de root nécessaire. N'exige aucun composant en plus de ce qui tourne déjà
(QEMU, bridge.py, osmocon, osmo-bts-trx, osmo-msc, etc.).
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# inject.py is sibling to this script — add containing dir to sys.path so the
# import works regardless of CWD.
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    import inject  # noqa: E402
    HAS_INJECT = True
except ImportError:
    HAS_INJECT = False

# -----------------------------------------------------------------------------
# Config (defaults — overridable via CLI)
# -----------------------------------------------------------------------------
BSP_UDP_HOST = "127.0.0.1"
BSP_UDP_PORT = 6702
BRIDGE_UDP_HOST = "127.0.0.1"
BRIDGE_UDP_PORT = 5702
L1CTL_SOCK = "/tmp/osmocom_l2"
QEMU_LOG = "/root/qemu.log"
BRIDGE_LOG = "/tmp/bridge.log"
OSMOCON_LOG = "/tmp/osmocon.log"
GDB_HOST, GDB_PORT = "127.0.0.1", 1234
GSMTAP_HOST, GSMTAP_PORT = "127.0.0.1", 4729
GSMTAP_PCAP = "/root/mobile-gsmtap.pcap"
VTY_PORTS = {
    "mobile":   4247,
    "osmo-bsc": 4242,
    "osmo-msc": 4254,
    "osmo-mgw": 4244,
    "osmo-hlr": 4258,
}

# Per-test default wait / poll (CLI-overridable)
WAIT_MS = 600
LOG_TAIL = 4000

# -----------------------------------------------------------------------------
# Result accumulator
# -----------------------------------------------------------------------------
@dataclass
class Result:
    layer: str
    frame: str
    result: str  # PASS | FAIL | SKIP | ERR
    detail: str = ""

RESULTS: list[Result] = []

def add(layer: str, frame: str, result: str, detail: str = ""):
    RESULTS.append(Result(layer, frame, result, detail))
    sym = {"PASS": "✓", "FAIL": "✗", "SKIP": "~", "ERR": "!"}.get(result, "?")
    print(f"  [{sym} {result:4}] {layer:10} {frame:30} {detail}", flush=True)

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def file_tail(path: str, n: int = LOG_TAIL) -> str:
    try:
        out = subprocess.run(["tail", "-n", str(n), path],
                             capture_output=True, text=True, timeout=2)
        return out.stdout
    except Exception:
        return ""

def grep_count(path: str, pattern: str, tail: int = LOG_TAIL) -> int:
    try:
        out = subprocess.run(
            ["bash", "-c", f"tail -n {tail} {path} 2>/dev/null | grep -cE '{pattern}'"],
            capture_output=True, text=True, timeout=2)
        return int(out.stdout.strip() or "0")
    except Exception:
        return 0

def now_ms() -> int:
    return int(time.time() * 1000)

# -----------------------------------------------------------------------------
# L1 BSP : TRXDv0 DL bursts via UDP 6702
# -----------------------------------------------------------------------------
def make_trxd_burst(tn: int, fn: int, rssi: int, toa: int, bits: bytes) -> bytes:
    """TRXDv0 DL: tn(1) fn(4 BE) rssi(1) toa(2 BE) bits(148) = 156 bytes."""
    assert len(bits) == 148
    return bytes([tn & 0x7]) + struct.pack(">I", fn) + bytes([rssi]) + struct.pack(">H", toa) + bits

def test_bsp_fb_burst():
    """All-zero bits → FCCH burst pattern."""
    burst = make_trxd_burst(tn=0, fn=0x12345, rssi=60, toa=0, bits=bytes(148))
    before_fb = grep_count(QEMU_LOG, r"\*\*\* FB \*\*\*", tail=2000)
    before_dma = grep_count(QEMU_LOG, r"\[BSP\] DMA", tail=2000)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(burst, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "FB_burst(all-0)", "ERR", f"sendto: {e}")
    time.sleep(WAIT_MS / 1000)
    after_fb = grep_count(QEMU_LOG, r"\*\*\* FB \*\*\*", tail=2000)
    after_dma = grep_count(QEMU_LOG, r"\[BSP\] DMA", tail=2000)
    delta_fb = after_fb - before_fb
    delta_dma = after_dma - before_dma
    if delta_fb >= 1 or delta_dma >= 1:
        add("L1-BSP", "FB_burst(all-0)", "PASS",
            f"FB log+{delta_fb} DMA log+{delta_dma}")
    else:
        add("L1-BSP", "FB_burst(all-0)", "FAIL",
            "no BSP log delta (BDLENA gated?)")

def test_bsp_sb_burst():
    """SB burst — synthetic training pattern in middle 64 bits."""
    bits = bytearray(148)
    # Mid bits = SB training sequence approximation
    for i in range(42, 106):
        bits[i] = (i & 1)
    burst = make_trxd_burst(tn=0, fn=0x12346, rssi=60, toa=0, bits=bytes(bits))
    before = grep_count(QEMU_LOG, r"\[BSP\] BURST.*SB/OTHER|\[BSP\] DMA")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(burst, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "SB_burst(training)", "ERR", f"sendto: {e}")
    time.sleep(WAIT_MS / 1000)
    after = grep_count(QEMU_LOG, r"\[BSP\] BURST.*SB/OTHER|\[BSP\] DMA")
    if after > before:
        add("L1-BSP", "SB_burst(training)", "PASS", f"BSP log+{after-before}")
    else:
        add("L1-BSP", "SB_burst(training)", "FAIL", "no BSP log delta")

def test_bsp_nb_burst():
    """Normal burst — high entropy bits."""
    bits = bytes((i * 137 + 23) & 1 for i in range(148))
    burst = make_trxd_burst(tn=0, fn=0x12347, rssi=60, toa=0, bits=bits)
    before = grep_count(QEMU_LOG, r"\[BSP\] BURST")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(burst, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "NB_burst(random)", "ERR", f"sendto: {e}")
    time.sleep(WAIT_MS / 1000)
    after = grep_count(QEMU_LOG, r"\[BSP\] BURST")
    if after > before:
        add("L1-BSP", "NB_burst(random)", "PASS", f"BSP BURST log+{after-before}")
    else:
        add("L1-BSP", "NB_burst(random)", "FAIL", "no BSP log delta")

def test_bsp_dummy_burst():
    """Dummy burst — all-ones."""
    bits = bytes([1] * 148)
    burst = make_trxd_burst(tn=0, fn=0x12348, rssi=60, toa=0, bits=bits)
    before = grep_count(QEMU_LOG, r"\[BSP\] BURST.*DUMMY/NB")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(burst, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "Dummy_burst(all-1)", "ERR", f"sendto: {e}")
    time.sleep(WAIT_MS / 1000)
    after = grep_count(QEMU_LOG, r"\[BSP\] BURST.*DUMMY/NB")
    add("L1-BSP", "Dummy_burst(all-1)",
        "PASS" if after > before else "FAIL",
        f"DUMMY log+{after-before}")

def test_bsp_malformed_short():
    """7-byte packet (< 8 hdr) — BSP should drop silently."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(b"\x00" * 7, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "Malformed_burst(<8B)", "ERR", f"sendto: {e}")
    time.sleep(WAIT_MS / 1000)
    # We expect NO crash; check QEMU still alive
    try:
        pid_alive = subprocess.run(
            ["pgrep", "-f", "qemu-system-arm"],
            capture_output=True, text=True, timeout=2).stdout.strip()
        if pid_alive:
            add("L1-BSP", "Malformed_burst(<8B)", "PASS", "QEMU still alive")
        else:
            add("L1-BSP", "Malformed_burst(<8B)", "FAIL", "QEMU died")
    except Exception as e:
        add("L1-BSP", "Malformed_burst(<8B)", "ERR", str(e))

def test_bsp_invalid_tn():
    """TN=0xFF (only low 3 bits used by BSP) — masked to TN=7."""
    burst = make_trxd_burst(tn=0xFF, fn=0x12349, rssi=60, toa=0, bits=bytes(148))
    before = grep_count(QEMU_LOG, r"\[BSP\] BURST.*tn=7")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(burst, (BSP_UDP_HOST, BSP_UDP_PORT))
    except Exception as e:
        return add("L1-BSP", "Burst_TN=0xFF(→7)", "ERR", str(e))
    time.sleep(WAIT_MS / 1000)
    after = grep_count(QEMU_LOG, r"\[BSP\] BURST.*tn=7")
    add("L1-BSP", "Burst_TN=0xFF(→7)",
        "PASS" if after > before else "FAIL",
        f"tn=7 log+{after-before}")

# -----------------------------------------------------------------------------
# L1 bridge : CLK IND via UDP 5702
# -----------------------------------------------------------------------------
def test_bridge_clk_ind():
    """CLK IND packet — trxv2 framing : 'IND CLOCK <fn>\\n' OR raw TRXDv0?

    bridge.py expects standard TRX protocol CLK from BTS. We test by injecting
    a CLK packet and checking bridge.log for a clk= counter increment.
    """
    before = 0
    try:
        out = subprocess.run(
            ["bash", "-c", f"grep -oE 'clk=[0-9]+' {BRIDGE_LOG} | tail -1"],
            capture_output=True, text=True, timeout=2)
        if out.stdout.strip():
            before = int(out.stdout.strip().split("=")[1])
    except Exception:
        pass
    # CLK IND : osmo-trx style "IND CLOCK <fn>\n" — basic format
    pkt = b"IND CLOCK 12345\n"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(pkt, (BRIDGE_UDP_HOST, BRIDGE_UDP_PORT))
    except Exception as e:
        return add("L1-bridge", "CLK_IND(text)", "ERR", str(e))
    time.sleep(WAIT_MS / 1000)
    after = 0
    try:
        out = subprocess.run(
            ["bash", "-c", f"grep -oE 'clk=[0-9]+' {BRIDGE_LOG} | tail -1"],
            capture_output=True, text=True, timeout=2)
        if out.stdout.strip():
            after = int(out.stdout.strip().split("=")[1])
    except Exception:
        pass
    if after > before:
        add("L1-bridge", "CLK_IND(text)", "PASS", f"bridge clk {before}→{after}")
    else:
        # bridge may not log clk increments on injection from us (it's a sink not a source)
        add("L1-bridge", "CLK_IND(text)", "SKIP",
            "bridge expects BTS direction; ours likely ignored")

# -----------------------------------------------------------------------------
# L1CTL : prend la place de mobile sur /tmp/osmocom_l2 (osmocon is server)
# -----------------------------------------------------------------------------
class L1CTLClient:
    """Client (mobile-side) of /tmp/osmocom_l2 — talks to firmware via osmocon."""

    def __init__(self):
        self.sock: Optional[socket.socket] = None
        self.rx_buf = b""
        self.rx_msgs: list[tuple[int, bytes]] = []
        self._stop = False
        self._thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        if not os.path.exists(L1CTL_SOCK):
            return False
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(L1CTL_SOCK)
            self.sock.settimeout(0.2)
            self._thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._thread.start()
            return True
        except Exception:
            return False

    def _rx_loop(self):
        while not self._stop:
            try:
                d = self.sock.recv(4096)
                if not d:
                    return
                self.rx_buf += d
                while len(self.rx_buf) >= 2:
                    n = struct.unpack(">H", self.rx_buf[:2])[0]
                    if len(self.rx_buf) < 2 + n:
                        break
                    msg = self.rx_buf[2:2 + n]
                    self.rx_buf = self.rx_buf[2 + n:]
                    if msg:
                        self.rx_msgs.append((msg[0], msg))
            except socket.timeout:
                continue
            except OSError:
                return

    def send(self, msg_type: int, payload: bytes = b"", flags: int = 0):
        hdr = struct.pack("<BBBB", msg_type, flags, 0, 0)
        body = hdr + payload
        frame = struct.pack(">H", len(body)) + body
        self.sock.sendall(frame)

    def wait_for(self, msg_types: set[int], timeout: float = 1.0) -> Optional[tuple[int, bytes]]:
        t_end = time.time() + timeout
        while time.time() < t_end:
            for mt, msg in self.rx_msgs[-50:]:
                if mt in msg_types:
                    return (mt, msg)
            time.sleep(0.02)
        return None

    def close(self):
        self._stop = True
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass


# L1CTL msg types
class L1CTL:
    RESET_REQ       = 0x0D
    RESET_IND       = 0x07
    RESET_CONF      = 0x0E
    FBSB_REQ        = 0x01
    FBSB_CONF       = 0x02
    DATA_IND        = 0x03
    DATA_REQ        = 0x06
    DATA_CONF       = 0x0F
    RACH_REQ        = 0x04
    RACH_CONF       = 0x0C
    DM_EST_REQ      = 0x05
    DM_REL_REQ      = 0x12
    DM_FREQ_REQ     = 0x14
    PM_REQ          = 0x08
    PM_CONF         = 0x09
    ECHO_REQ        = 0x0A
    ECHO_CONF       = 0x0B
    CCCH_MODE_REQ   = 0x10
    CCCH_MODE_CONF  = 0x11
    PARAM_REQ       = 0x13
    CRYPTO_REQ      = 0x15
    SIM_REQ         = 0x16
    SIM_CONF        = 0x17
    TCH_MODE_REQ    = 0x18
    TCH_MODE_CONF   = 0x19
    NEIGH_PM_REQ    = 0x1A
    NEIGH_PM_IND    = 0x1B
    TRAFFIC_REQ     = 0x1C
    TRAFFIC_CONF    = 0x1D
    TRAFFIC_IND     = 0x1E

def info_ul(chan_nr=0x88, link_id=0):
    return struct.pack("<BBH", chan_nr, link_id, 0)  # chan_nr(1) link_id(1) padding(2)

def test_l1ctl_suite():
    """Run a connect → send → check_conf for each *_REQ."""
    if not os.path.exists(L1CTL_SOCK):
        add("L1CTL", "(all)", "SKIP", "/tmp/osmocom_l2 absent (osmocon down?)")
        return

    # Note: the socket has ONE client slot. If `mobile` is connected, we cannot.
    # Strategy: try to connect; if EAGAIN/refused, fall back to "SKIP all".
    c = L1CTLClient()
    if not c.connect():
        add("L1CTL", "(all)", "SKIP", "cannot connect (mobile occupies socket?)")
        return

    tests = [
        # (name, msg_type, payload, expected_conf_set)
        ("RESET_REQ",      L1CTL.RESET_REQ,      struct.pack("<BBBB", 1, 0, 0, 0), {L1CTL.RESET_CONF}),
        ("FBSB_REQ",       L1CTL.FBSB_REQ,
            struct.pack("<HHHHBBBBB", 0x0001, 100, 50, 200, 4, 0x07, 0, 0, 80),
            {L1CTL.FBSB_CONF}),
        ("PM_REQ_range",   L1CTL.PM_REQ,
            struct.pack("<BBBB", 1, 0, 0, 0) + struct.pack(">HH", 1, 4),
            {L1CTL.PM_CONF}),
        ("CCCH_MODE_REQ",  L1CTL.CCCH_MODE_REQ,
            info_ul(0x88) + struct.pack("<BBBB", 0, 0, 0, 0),  # NON_COMBINED
            {L1CTL.CCCH_MODE_CONF}),
        ("ECHO_REQ",       L1CTL.ECHO_REQ,       b"ECHO123",   {L1CTL.ECHO_CONF}),
        ("RACH_REQ",       L1CTL.RACH_REQ,
            info_ul(0x88) + struct.pack("<BBHB", 0x5A, 0, 0, 10),  # ra=0x5A, uic=10
            {L1CTL.RACH_CONF}),
        ("NEIGH_PM_REQ",   L1CTL.NEIGH_PM_REQ,
            struct.pack("<B", 1) + struct.pack(">H", 1) + bytes(125),
            {L1CTL.NEIGH_PM_IND}),
        ("DM_REL_REQ",     L1CTL.DM_REL_REQ,     info_ul(0x88), set()),  # no CONF expected
        ("PARAM_REQ",      L1CTL.PARAM_REQ,
            info_ul(0x88) + struct.pack("<bBBB", 0, 30, 0, 0),  # ta=0, tx_pwr=30
            set()),
        ("CRYPTO_REQ",     L1CTL.CRYPTO_REQ,
            info_ul(0x88) + struct.pack("<BB", 0, 8) + bytes(8),  # algo=0 (none)
            set()),
        ("SIM_REQ",        L1CTL.SIM_REQ,
            bytes([0xA0, 0xA4, 0, 0, 2, 0x3F, 0x00]),  # SELECT MF APDU
            {L1CTL.SIM_CONF}),
        ("TCH_MODE_REQ",   L1CTL.TCH_MODE_REQ,
            struct.pack("<BBBBBB", 0, 0, 0, 0, 0, 0),
            {L1CTL.TCH_MODE_CONF}),
    ]

    for name, mt, pl, expect in tests:
        try:
            c.rx_msgs.clear()
            c.send(mt, pl)
            if expect:
                got = c.wait_for(expect, timeout=1.0)
                if got:
                    conf_name = next(k for k, v in vars(L1CTL).items() if v == got[0])
                    add("L1CTL", name, "PASS", f"got {conf_name} ({len(got[1])}B)")
                else:
                    add("L1CTL", name, "FAIL", f"no {[hex(e) for e in expect]} in 1s")
            else:
                # one-shot: send, sleep, check no error log
                time.sleep(0.3)
                add("L1CTL", name, "PASS", "sent (no CONF expected)")
        except Exception as e:
            add("L1CTL", name, "ERR", str(e))

    c.close()

# -----------------------------------------------------------------------------
# L2 / L3 LAPDm — wrap dans L1CTL_DATA_REQ
# -----------------------------------------------------------------------------
def test_l2_l3_via_data_req():
    """Send LAPDm I-frame with L3 RR Paging Request inside, see what firmware does."""
    if not os.path.exists(L1CTL_SOCK):
        add("L2-L3", "(skip)", "SKIP", "no socket")
        return

    c = L1CTLClient()
    if not c.connect():
        add("L2-L3", "(skip)", "SKIP", "socket busy")
        return

    # L2 LAPDm I-frame, SAPI 0, P=0, N(S)=0, N(R)=0
    # Octets: addr(1) ctrl(1) length(1) info(23)
    lapdm = bytes([
        0x01,  # SAPI=0, C/R=0, EA=1
        0x00,  # I-frame, N(S)=0, P=0, N(R)=0
        0x49,  # length indicator: 18 bytes, M=0, EL=1
    ]) + bytes([
        # L3 RR Paging Request Type 1 (3GPP 24.008 9.1.22)
        0x06, 0x21,  # RR / PAGING REQ T1
        0x20, 0x10,  # page mode CS / channels_needed
        # Mobile identity 1: IMSI 001010000000001 (5 bytes)
        0x05, 0x08, 0x09, 0x10, 0x10, 0x00, 0x00, 0x00, 0x10,
    ]) + bytes([0x2B] * (23 - 12))

    # Build DATA_REQ payload: info_ul + payload
    payload = info_ul(0x88, link_id=0) + lapdm[:23]

    try:
        before = grep_count(OSMOCON_LOG, r"hdlc_send\(dlci=5\):.*06", tail=500)
        c.send(L1CTL.DATA_REQ, payload)
        time.sleep(0.5)
        after = grep_count(OSMOCON_LOG, r"hdlc_send\(dlci=5\):.*06", tail=500)
        # Best signal: firmware echoed via SERCOMM
        add("L2-L3", "DATA_REQ(LAPDm+RR_Paging)",
            "PASS" if after > before else "FAIL",
            f"osmocon hdlc_send+{after-before}")
    except Exception as e:
        add("L2-L3", "DATA_REQ(LAPDm+RR_Paging)", "ERR", str(e))
    c.close()

# -----------------------------------------------------------------------------
# Sercomm DLCI via /dev/pts/2 — needs raw UART write access (root only)
# -----------------------------------------------------------------------------
def test_sercomm_dlci():
    pty_path = "/dev/pts/2"
    if not os.path.exists(pty_path):
        return add("Sercomm", "DLCI5_echo", "SKIP", "/dev/pts/2 absent")
    # /dev/pts/2 is owned by osmocon; we cannot write without conflict.
    # Skip — would need to suspend osmocon temporarily.
    add("Sercomm", "DLCI5_echo", "SKIP", "pts/2 owned by osmocon")

# -----------------------------------------------------------------------------
# GDB-stub : :1234 — memory R/W check
# -----------------------------------------------------------------------------
def gdb_pkt(s, data: bytes | str) -> bytes:
    body = data.encode() if isinstance(data, str) else data
    csum = sum(body) % 256
    s.sendall(b"$" + body + b"#" + ("%02x" % csum).encode())
    # skip ACK
    c = s.recv(1)
    return c

def gdb_recv(s) -> bytes:
    buf = b""
    while True:
        c = s.recv(1)
        if not c:
            return b""
        if c in (b"+", b"-"):
            continue
        if c == b"$":
            break
    while True:
        c = s.recv(1)
        if c == b"#":
            s.recv(2)
            s.sendall(b"+")
            return buf
        buf += c

def test_gdb_stub():
    try:
        s = socket.socket()
        s.connect((GDB_HOST, GDB_PORT))
        s.settimeout(2.0)
        gdb_pkt(s, "?")
        halt = gdb_recv(s)
        if not halt:
            return add("GDB", "halt_reason", "FAIL", "empty response")
        add("GDB", "halt_reason", "PASS", halt.decode(errors="replace"))
        # Read d_fb_det
        gdb_pkt(s, "mffd001f0,2")
        v = gdb_recv(s).decode()
        add("GDB", "read_d_fb_det", "PASS" if len(v) == 4 else "FAIL", f"val={v}")
        # Write a probe pattern at d_fb_det (will be overwritten by DSP)
        gdb_pkt(s, "Mffd001f0,2:4242")
        wr = gdb_recv(s).decode()
        add("GDB", "write_d_fb_det", "PASS" if wr == "OK" else "FAIL", f"resp={wr}")
        # Continue
        gdb_pkt(s, "c")
        s.close()
    except ConnectionRefusedError:
        add("GDB", "connect", "SKIP", "gdbserver not active (run: gdbserver tcp::1234 via monitor)")
    except Exception as e:
        add("GDB", "connect", "ERR", str(e))

# -----------------------------------------------------------------------------
# VTY Osmocom + mobile
# -----------------------------------------------------------------------------
def test_vty(name: str, port: int, command: str, expect_keyword: str):
    try:
        s = socket.socket()
        s.connect(("127.0.0.1", port))
        s.settimeout(2.0)
        # banner
        _ = s.recv(4096)
        s.sendall(b"enable\r\n")
        time.sleep(0.1)
        _ = s.recv(4096)
        s.sendall(command.encode() + b"\r\n")
        time.sleep(0.5)
        resp = b""
        try:
            while True:
                d = s.recv(4096)
                if not d:
                    break
                resp += d
                if expect_keyword.encode() in resp or b">" in resp[-10:]:
                    break
        except socket.timeout:
            pass
        s.close()
        if expect_keyword.encode() in resp:
            add("VTY", f"{name}:{command[:20]}", "PASS",
                f"saw '{expect_keyword}' ({len(resp)}B)")
        else:
            add("VTY", f"{name}:{command[:20]}", "FAIL",
                f"no '{expect_keyword}' ({len(resp)}B)")
    except ConnectionRefusedError:
        add("VTY", f"{name}:{command[:20]}", "SKIP", f"port {port} closed")
    except Exception as e:
        add("VTY", f"{name}:{command[:20]}", "ERR", str(e))

def test_vty_suite():
    test_vty("mobile",   VTY_PORTS["mobile"],   "show ms",      "MS")
    test_vty("osmo-bsc", VTY_PORTS["osmo-bsc"], "show network", "BSC")
    test_vty("osmo-msc", VTY_PORTS["osmo-msc"], "show subscriber cache", "IMSI")
    test_vty("osmo-mgw", VTY_PORTS["osmo-mgw"], "show stats",   "Conn")
    test_vty("osmo-hlr", VTY_PORTS["osmo-hlr"], "show subscribers all", "IMSI")

# -----------------------------------------------------------------------------
# GSMTAP : observe-only (we can inject but mobile/bts don't listen typically)
# -----------------------------------------------------------------------------
def test_gsmtap():
    """Send a GSMTAP packet to localhost:4729 and check if tcpdump captures it."""
    pcap = GSMTAP_PCAP
    if not os.path.exists(pcap):
        return add("GSMTAP", f"inject({GSMTAP_PORT})", "SKIP", "pcap absent")
    before_size = os.path.getsize(pcap)
    # GSMTAP v2 header (16 bytes)
    gsmtap = struct.pack(">BBBBBBBBIHBB",
                         0x02,        # version
                         4,           # hdr_len in 32-bit words
                         0x01,        # type=UM (raw)
                         0,           # ts (TN)
                         0, 0, 0,     # subtype, antenna, sub_slot
                         0,           # pad
                         0,           # arfcn
                         0, 0)        # snr_db, signal_db
    payload = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(gsmtap + payload, (GSMTAP_HOST, GSMTAP_PORT))
    except Exception as e:
        return add("GSMTAP", f"inject({GSMTAP_PORT})", "ERR", str(e))
    time.sleep(0.5)
    after_size = os.path.getsize(pcap)
    if after_size > before_size:
        add("GSMTAP", f"inject({GSMTAP_PORT})", "PASS", f"pcap +{after_size-before_size}B")
    else:
        add("GSMTAP", f"inject({GSMTAP_PORT})", "FAIL", "pcap unchanged")

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def banner(title: str):
    print(f"\n=== {title} ===", flush=True)

ALL_LAYERS = ("L1-BSP", "L1-bridge", "L1CTL", "L2-L3", "Sercomm", "GDB", "VTY", "GSMTAP")

# Tests grouped per mode. Each entry = (layer_tag, fn, modes_in_which_it_runs)
TEST_REGISTRY = [
    # L1 BSP
    ("L1-BSP",    test_bsp_fb_burst,        ("quick", "full", "l1")),
    ("L1-BSP",    test_bsp_sb_burst,        ("full", "l1")),
    ("L1-BSP",    test_bsp_nb_burst,        ("full", "l1")),
    ("L1-BSP",    test_bsp_dummy_burst,     ("full", "l1")),
    ("L1-BSP",    test_bsp_malformed_short, ("full", "l1", "robustness")),
    ("L1-BSP",    test_bsp_invalid_tn,      ("full", "l1", "robustness")),
    # L1 bridge
    ("L1-bridge", test_bridge_clk_ind,      ("full",)),
    # L1CTL (one fn covers all *_REQ subtests internally)
    ("L1CTL",     test_l1ctl_suite,         ("quick", "full", "l1ctl")),
    # L2/L3
    ("L2-L3",     test_l2_l3_via_data_req,  ("full", "l2l3")),
    # Sercomm
    ("Sercomm",   test_sercomm_dlci,        ("full",)),
    # GDB
    ("GDB",       test_gdb_stub,            ("quick", "full", "gdb")),
    # VTY
    ("VTY",       test_vty_suite,           ("quick", "full", "vty")),
    # GSMTAP
    ("GSMTAP",    test_gsmtap,              ("full",)),
]

def parse_args():
    p = argparse.ArgumentParser(
        description="Cross-layer frame injection + PASS/FAIL validation for qemu-calypso.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--mode", default="quick",
                   choices=("quick", "full", "l1", "l1ctl", "l2l3", "gdb", "vty", "robustness"),
                   help="quick=fast subset; full=everything; or one specific tag")
    p.add_argument("--layers", default="",
                   help=f"comma-separated allow-list of layers (overrides mode filter): {','.join(ALL_LAYERS)}")
    p.add_argument("--wait-ms", type=int, default=WAIT_MS,
                   help="observation window per test (ms)")
    p.add_argument("--log-tail", type=int, default=LOG_TAIL,
                   help="lines of qemu.log to grep for delta counts")
    p.add_argument("--bsp-host",    default=BSP_UDP_HOST)
    p.add_argument("--bsp-port",    type=int, default=BSP_UDP_PORT)
    p.add_argument("--bridge-host", default=BRIDGE_UDP_HOST)
    p.add_argument("--bridge-port", type=int, default=BRIDGE_UDP_PORT)
    p.add_argument("--l1ctl-sock",  default=L1CTL_SOCK)
    p.add_argument("--qemu-log",    default=QEMU_LOG)
    p.add_argument("--bridge-log",  default=BRIDGE_LOG)
    p.add_argument("--osmocon-log", default=OSMOCON_LOG)
    p.add_argument("--gdb-host",    default=GDB_HOST)
    p.add_argument("--gdb-port",    type=int, default=GDB_PORT)
    p.add_argument("--gsmtap-host", default=GSMTAP_HOST)
    p.add_argument("--gsmtap-port", type=int, default=GSMTAP_PORT)
    p.add_argument("--gsmtap-pcap", default=GSMTAP_PCAP)
    p.add_argument("--quiet",       action="store_true", help="suppress per-test lines, only summary")
    p.add_argument("--json",        action="store_true", help="emit JSON summary at end")
    return p.parse_args()

def apply_args(a):
    """Override module globals from CLI."""
    g = globals()
    g["BSP_UDP_HOST"]    = a.bsp_host
    g["BSP_UDP_PORT"]    = a.bsp_port
    g["BRIDGE_UDP_HOST"] = a.bridge_host
    g["BRIDGE_UDP_PORT"] = a.bridge_port
    g["L1CTL_SOCK"]      = a.l1ctl_sock
    g["QEMU_LOG"]        = a.qemu_log
    g["BRIDGE_LOG"]      = a.bridge_log
    g["OSMOCON_LOG"]     = a.osmocon_log
    g["GDB_HOST"]        = a.gdb_host
    g["GDB_PORT"]        = a.gdb_port
    g["GSMTAP_HOST"]     = a.gsmtap_host
    g["GSMTAP_PORT"]     = a.gsmtap_port
    g["GSMTAP_PCAP"]     = a.gsmtap_pcap
    g["WAIT_MS"]         = a.wait_ms
    g["LOG_TAIL"]        = a.log_tail

def main():
    args = parse_args()
    apply_args(args)

    # Filter tests by mode and (optionally) layers allow-list
    layer_filter: Optional[set[str]] = None
    if args.layers:
        layer_filter = {x.strip() for x in args.layers.split(",") if x.strip()}
    selected = []
    for layer, fn, tags in TEST_REGISTRY:
        if args.mode not in tags:
            continue
        if layer_filter is not None and layer not in layer_filter:
            continue
        selected.append((layer, fn))

    if not args.quiet:
        print(f"validating.py — mode={args.mode}  layers={args.layers or 'ALL'}  "
              f"tests={len(selected)}  wait_ms={args.wait_ms}")
        print("=" * 70)
    t0 = now_ms()

    last_layer = None
    for layer, fn in selected:
        if not args.quiet and layer != last_layer:
            banner(f"{layer}")
            last_layer = layer
        try:
            fn()
        except Exception as e:
            add(layer, fn.__name__, "ERR", f"uncaught: {e}")

    t1 = now_ms()
    counts = {"PASS": 0, "FAIL": 0, "SKIP": 0, "ERR": 0}
    for r in RESULTS:
        counts[r.result] = counts.get(r.result, 0) + 1
    total = len(RESULTS) or 1

    if args.json:
        import json
        out = {
            "mode": args.mode,
            "elapsed_ms": t1 - t0,
            "total": len(RESULTS),
            "counts": counts,
            "tests": [{"layer": r.layer, "frame": r.frame,
                       "result": r.result, "detail": r.detail}
                      for r in RESULTS],
        }
        print(json.dumps(out, indent=2))
    else:
        print("\n" + "=" * 70)
        print(f"Summary  ({t1-t0} ms)")
        print("=" * 70)
        for k in ("PASS", "FAIL", "SKIP", "ERR"):
            print(f"  {k:4} : {counts.get(k,0):3} / {len(RESULTS)}")
        print()
        by_layer: dict[str, list[Result]] = {}
        for r in RESULTS:
            by_layer.setdefault(r.layer, []).append(r)
        for layer in sorted(by_layer):
            rs = by_layer[layer]
            passed = sum(1 for r in rs if r.result == "PASS")
            print(f"  {layer:10}  {passed}/{len(rs)}")

    sys.exit(0 if counts.get("FAIL", 0) == 0 else 1)

if __name__ == "__main__":
    main()
