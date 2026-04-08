#!/usr/bin/env python3
"""
bridge.py — L1CTL + TRX bridge for Calypso QEMU.

Roles:
  1. L1CTL  : mobile ─ unix(/tmp/osmocom_l2_1) ─ bridge ─ PTY (sercomm DLCI 5) ─ QEMU
  2. TRXC   : osmo-bts-trx → bridge replies locally (handshake-only proxy).
              The DSP/firmware does NOT see TRXC traffic — it owns the air via BSP.
  3. CLK    : bridge sources "IND CLOCK <fn>" toward BTS at GSM frame rate.
  4. TRXD   : bridge ⇄ QEMU gate UDP, both directions.
              BTS → gate (DL bursts → BSP)  : 5702 → 6702
              gate → BTS (UL bursts)        : 6702 → 5702 (forwarded as-is)

Port layout:
  osmo-bts-trx local 5800/5801/5802 (binds), remote 5700/5701/5702 (sends to)
  bridge BTS-side bind:  5700 (CLK out), 5701 (TRXC), 5702 (TRXD)
  bridge BB-side  bind:  6802 (TRXD)  ← forwards bursts to gate at 6702
  gate (QEMU)     bind:  6702 (TRXD only — TRXC/CLK no longer used in QEMU)
"""

import argparse
try:
    import gsm_mod  # GMSK modulator (gr-gsm + gnuradio.digital.gmsk_mod)
except ImportError as _e:
    gsm_mod = None
    print(f"WARNING: gsm_mod not loadable ({_e}); DL bursts will be forwarded raw", flush=True)
import errno
import fcntl
import os
import select
import socket
import struct
import sys
import termios
import threading
import time

# ────────────────────────── constants ──────────────────────────

DLCI_L1CTL = 5
DLCI_TRXC  = 4   # TRXC ASCII over sercomm; intercepted by sercomm_gate.c

# L1CTL message types (subset)
MSG_FBSB_REQ  = 0x01
MSG_FBSB_CONF = 0x02
MSG_DATA_IND  = 0x03
TARGET_ARFCN  = int(os.environ.get("TARGET_ARFCN", "514"))

# ──────── DL CCCH decoder (libosmocoding via ctypes) ────────
import ctypes as _ct
try:
    _osmocoding = _ct.CDLL("/usr/local/lib/libosmocoding.so")
except OSError:
    try:
        _osmocoding = _ct.CDLL("libosmocoding.so.0")
    except OSError:
        _osmocoding = None

def extract_burst_data(bits_148):
    """Hard or soft 148 bits → 114 coded data bits (positions 3..59 / 88..144)."""
    raw = list(bits_148[:148])
    if len(raw) < 148:
        raw.extend([0] * (148 - len(raw)))
    if max(raw) > 1:
        hard = [1 if b > 127 else 0 for b in raw]
    else:
        hard = raw
    return hard[3:60] + hard[88:145]

def decode_ccch_block(bursts_data):
    """4 × 114 hard bits → 23-byte L2 via gsm0503_xcch_decode."""
    if _osmocoding is None or len(bursts_data) != 4:
        return None
    iB = (_ct.c_int8 * (4 * 116))()
    for i in range(4):
        for j in range(57):
            iB[i*116 + j]      = -127 if bursts_data[i][j] else 127
        iB[i*116 + 57] = 0
        iB[i*116 + 58] = 0
        for j in range(57):
            iB[i*116 + 59 + j] = -127 if bursts_data[i][57+j] else 127
    l2     = (_ct.c_uint8 * 23)()
    n_err  = _ct.c_int(0)
    n_bits = _ct.c_int(0)
    _osmocoding.gsm0503_xcch_decode(l2, iB,
                                    _ct.byref(n_err), _ct.byref(n_bits))
    return bytes(l2)


L1CTL_NAMES = {
    0x01: "FBSB_REQ", 0x02: "FBSB_CONF", 0x03: "DATA_IND",
    0x04: "RACH_REQ", 0x05: "DM_EST_REQ", 0x06: "DATA_REQ",
    0x07: "RESET_IND", 0x08: "PM_REQ", 0x09: "PM_CONF",
    0x0a: "ECHO_REQ", 0x0b: "ECHO_CONF", 0x0c: "RACH_CONF",
    0x0d: "RESET_REQ", 0x0e: "RESET_CONF", 0x0f: "DATA_CONF",
    0x10: "CCCH_MODE_REQ", 0x11: "CCCH_MODE_CONF",
    0x12: "DM_REL_REQ", 0x13: "PARAM_REQ", 0x14: "DM_FREQ_REQ",
}
CTRL       = 0x03
FLAG       = 0x7E
ESCAPE     = 0x7D
ESCAPE_XOR = 0x20

GSM_FRAME_US    = 4615         # 4.615 ms TDMA frame
GSM_HYPERFRAME  = 2715648
CLK_IND_PERIOD  = 216          # send IND CLOCK every 216 frames (~1 s)
                                # — fewer messages = less Python-jitter
                                #   for the BTS's own free-running clock.

# ────────────────────────── sercomm framing ──────────────────────────

def sercomm_escape(data):
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)

def sercomm_unescape(data):
    out = bytearray()
    i = 0
    while i < len(data):
        if data[i] == ESCAPE:
            i += 1
            if i < len(data):
                out.append(data[i] ^ ESCAPE_XOR)
        else:
            out.append(data[i])
        i += 1
    return bytes(out)

def sercomm_wrap(dlci, payload):
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        out = []
        while True:
            try:
                start = self.buf.index(FLAG)
            except ValueError:
                self.buf.clear()
                break
            if start > 0:
                del self.buf[:start]
            try:
                end = self.buf.index(FLAG, 1)
            except ValueError:
                break
            raw = bytes(self.buf[1:end])
            del self.buf[:end + 1]
            if not raw:
                continue
            frame = sercomm_unescape(raw)
            if len(frame) < 2:
                continue
            out.append((frame[0], frame[2:]))
        return out

class LengthPrefixReader:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        msgs = []
        while len(self.buf) >= 2:
            n = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + n:
                break
            msgs.append(bytes(self.buf[2:2 + n]))
            del self.buf[:2 + n]
        return msgs

# ────────────────────────── PTY/UDP helpers ──────────────────────────

def pty_set_raw(fd):
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    a[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    a[3] = 0
    a[4] = termios.B115200
    a[5] = termios.B115200
    a[6][termios.VMIN]  = 1
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)

def set_nonblock(fd):
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

def udp_bind(port, addr="127.0.0.1"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((addr, port))
    s.setblocking(False)
    return s

# ────────────────────────── TRXC handler ──────────────────────────

def trxc_response(raw):
    """Build a 'RSP ...' string for an incoming 'CMD ...' from osmo-bts-trx.
    Mirrors sercomm_udp.py and fake_trx behaviour: handshake-only, success
    for everything except commands that have a documented payload format.
    """
    s = raw.strip(b'\x00').decode(errors='replace').strip()
    if not s.startswith("CMD "):
        return None
    parts = s[4:].split()
    if not parts:
        return None
    verb = parts[0]
    args = parts[1:]

    if verb == "POWERON":
        return "RSP POWERON 0"
    if verb == "POWEROFF":
        return "RSP POWEROFF 0"
    if verb == "SETFORMAT":
        # Force TRXDv0 (8-byte UL header). Our calypso_bsp UL emitter
        # only knows the v0 wire format; agreeing to v1 would leave the
        # MTS+C/I 3-byte tail unfilled and the BTS would drop bursts.
        return "RSP SETFORMAT 0 0"
    if verb == "NOMTXPOWER":
        return "RSP NOMTXPOWER 0 50"
    if verb == "MEASURE":
        freq = args[0] if args else "0"
        return f"RSP MEASURE 0 {freq} -60"
    # Default: status 0, echo args verbatim. Covers RXTUNE, TXTUNE, SETSLOT,
    # SETPOWER, ADJPOWER, SETMAXDLY, SETTSC, SETBSIC, RFMUTE, NOHANDOVER, …
    if args:
        return f"RSP {verb} 0 " + " ".join(args)
    return f"RSP {verb} 0"

# ────────────────────────── main ──────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pty", help="QEMU PTY device (e.g. /dev/pts/0)")
    ap.add_argument("--sock", default="/tmp/osmocom_l2_1")
    ap.add_argument("--bts-base", type=int, default=5700)
    ap.add_argument("--bb-base",  type=int, default=6700)
    ap.add_argument("--bts-addr", default="127.0.0.1")
    ap.add_argument("--bb-addr",  default="127.0.0.1")
    args = ap.parse_args()

    BTS, BB = args.bts_base, args.bb_base

    # ── Modem channel ──
    # Three modes:
    #   1. /dev/pts/N      → real PTY (legacy, broken: char-pty connected bug)
    #   2. unix:/path      → connect to existing unix server (QEMU server mode)
    #   3. server:/path    → create listening unix server (QEMU client mode)
    #      This is the recommended mode: QEMU is launched with
    #      `-chardev socket,...,server=off,wait=on` and blocks until
    #      bridge accepts. Eliminates the boot-time race where firmware
    #      writes its first sercomm frame before bridge has connected.
    _modem_sock = None
    _modem_listen = None
    if args.pty.startswith("server:"):
        path = args.pty[len("server:"):]
        try: os.unlink(path)
        except FileNotFoundError: pass
        _modem_listen = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        _modem_listen.bind(path)
        _modem_listen.listen(1)
        print(f"modem: AF_UNIX listening {path} (waiting for QEMU)", flush=True)
        _modem_sock, _ = _modem_listen.accept()
        _modem_sock.setblocking(False)
        pty_fd = _modem_sock.fileno()
        print(f"modem: QEMU connected on {path}", flush=True)
    elif args.pty.startswith("unix:") or args.pty.startswith("@") or (
            os.path.exists(args.pty) and not args.pty.startswith("/dev/pts/")):
        path = args.pty[5:] if args.pty.startswith("unix:") else args.pty
        _modem_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(40):
            try:
                _modem_sock.connect(path); break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.25)
        else:
            print(f"FATAL: cannot connect modem unix socket {path}", flush=True)
            sys.exit(1)
        _modem_sock.setblocking(False)
        pty_fd = _modem_sock.fileno()
        print(f"modem: AF_UNIX client {path}", flush=True)
    else:
        pty_fd = os.open(args.pty, os.O_RDWR | os.O_NOCTTY)
        pty_set_raw(pty_fd)
        set_nonblock(pty_fd)
        print(f"modem: PTY {args.pty}", flush=True)

    # ── L1CTL unix server: RE-ENABLED 2026-04-07 (revert) ──
    # bridge.py owns /tmp/osmocom_l2_1 again. mobile connects here,
    # L1CTL frames are wrapped in sercomm DLCI 5 and written to the
    # PTY; sercomm_gate.c in QEMU parses them and injects into the
    # firmware UART RX (real osmocon path).
    try:
        os.unlink(args.sock)
    except FileNotFoundError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.sock)
    srv.listen(1)
    srv.setblocking(False)
    print(f"L1CTL unix server: {args.sock}", flush=True)

    # ── BTS-side TRX sockets ──
    bts_clk  = udp_bind(BTS + 0)   # 5700 — bridge sends IND CLOCK from here
    bts_trxc = udp_bind(BTS + 1)   # 5701 — TRXC: forwarded to PTY (DLCI 4)
    bts_trxd = udp_bind(BTS + 2)   # 5702 — TRXD bursts (relayed to gate)

    # ── BB-side TRX sockets ──
    # Burst path bridge↔bsp:
    #   bridge binds 6702 (UL recv from bsp), sendto bsp:6802 for DL
    bb_trxd  = udp_bind(BB + 2)    # 6702 — bridge bind for UL bursts from bsp
    # NEW: UDP 6801 ↔ sercomm DLCI 5 — bridge multiplexes the L1CTL mobile
    # path on the same UART chardev as the burst path. Mobile (or anyone)
    # sends length-prefixed L1CTL frames as UDP datagrams to 6801, bridge
    # wraps them in sercomm DLCI 5 and writes to the UART. Inverse: when
    # the firmware emits a DLCI 5 frame on the UART, bridge unwraps and
    # sends the L1CTL bytes as UDP datagram to the last sender of 6801.
    bb_l1ctl = udp_bind(BB + 101)  # 6801 — bridge ↔ mobile L1CTL (UDP)
    bb_l1ctl_dst = (args.bb_addr, BB + 100)  # default; updated on first inbound

    print(f"bridge: pty={args.pty} sock={args.sock} BTS={BTS} BB={BB}",
          flush=True)

    # ── L1CTL "firmware speaks first" handshake ──
    # The firmware UART RX FIFO will accept bytes immediately, but the
    # firmware's L1A_L23 callback isn't registered until the L1S thread
    # is up. If mobile sends RESET_REQ before the firmware is ready, the
    # bytes get dropped or corrupt the early sercomm state. Mirror what
    # l1ctl_sock.c used to do: buffer mobile bytes until we see the
    # firmware emit its first DLCI 5 frame on the PTY (typically the
    # boot RESET_IND), then drain & start forwarding.
    firmware_ready = False
    pending_l1ctl  = []   # list of L1CTL payloads buffered before ready

    # ── Cleanup on SIGINT/SIGTERM/exit ──
    stop = threading.Event()
    _all_sockets = [srv, bts_clk, bts_trxc, bts_trxd, bb_trxd, bb_l1ctl]
    def _cleanup(*_):
        print("bridge: cleaning up sockets", flush=True)
        stop.set()
        for sk in _all_sockets:
            try: sk.close()
            except Exception: pass
        try: os.close(pty_fd)
        except Exception: pass
        try: os.unlink(args.sock)
        except Exception: pass
        sys.exit(0)
    import atexit, signal
    atexit.register(lambda: [sk.close() for sk in _all_sockets if sk])
    signal.signal(signal.SIGINT,  _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    # State
    sercomm = SercommParser()
    cli = None
    cli_rd = LengthPrefixReader()
    bts_clk_dst   = (args.bts_addr, BTS + 100)   # default; updated on first BTS pkt
    bts_trxc_dst  = (args.bts_addr, BTS + 101)
    bts_trxd_dst  = (args.bts_addr, BTS + 102)
    bb_trxd_dst   = (args.bb_addr,  BB  + 102)   # bsp DL bind = 6802
    powered = False
    fn = 0
    stats = {"l1ctl→pty": 0, "pty→l1ctl": 0,
             "trxc": 0, "clk": 0, "dl": 0, "ul": 0, "decoded": 0}
    burst_buf = {}  # block_id → list[4] of 114 hard bits
    last_print = time.monotonic()
    l1ctl_connected_at = None  # stats["dl"] snapshot when L1CTL client connects

    # ── CLK source thread ──
    # Strict monotonic frame ticker. osmo-bts-trx demands an IND CLOCK
    # every CLK_IND_PERIOD frames; if it stops, the scheduler shuts the
    # BTS down with "GSM clock stopped". Use absolute deadlines so jitter
    # doesn't accumulate.
    def clk_loop():
        nonlocal fn
        tick_ns = GSM_FRAME_US * 1000  # 4615 µs per TDMA frame
        t_next  = time.monotonic_ns()
        while not stop.is_set():
            t_next += tick_ns
            now = time.monotonic_ns()
            dt = t_next - now
            if dt > 0:
                time.sleep(dt / 1e9)
            elif dt < -tick_ns * 50:
                # Fell more than 50 frames behind: hard resync.
                t_next = now
            fn = (fn + 1) % GSM_HYPERFRAME
            if powered and (fn % CLK_IND_PERIOD) == 0:
                # −3 FN bias: BTS reports "3 FN slower than TRX" so it
                # processes our IND CLOCK as if it were 3 frames in the
                # past — push that forward by retarding the FN we send.
                try:
                    fn_adj = (fn - 3) % GSM_HYPERFRAME
                    bts_clk.sendto(f"IND CLOCK {fn_adj}\0".encode(),
                                   bts_clk_dst)
                    stats["clk"] += 1
                except OSError:
                    pass
    t = threading.Thread(target=clk_loop, daemon=True)
    t.start()

    poll = select.poll()
    poll.register(pty_fd,            select.POLLIN)
    if srv is not None:
        poll.register(srv.fileno(),  select.POLLIN)
    for sk in (bts_trxc, bts_trxd, bb_trxd, bb_l1ctl):
        poll.register(sk.fileno(),    select.POLLIN)

    while True:
        events = poll.poll(1000)
        for fd, _ in events:
            # ── L1CTL: new client ──
            if srv is not None and fd == srv.fileno():
                if cli is not None:
                    try: cli.close()
                    except Exception: pass
                cli, _ = srv.accept()
                cli.setblocking(False)
                poll.register(cli.fileno(), select.POLLIN)
                cli_rd = LengthPrefixReader()
                print("L1CTL client connected", flush=True)
                l1ctl_connected_at = stats["dl"]
                continue

            # ── L1CTL: mobile → PTY (wrap sercomm DLCI 5) ──
            if cli is not None and fd == cli.fileno():
                try:
                    data = cli.recv(4096)
                except BlockingIOError:
                    continue
                if not data:
                    print("L1CTL client disconnected", flush=True)
                    poll.unregister(fd)
                    cli.close()
                    cli = None
                    continue
                for msg in cli_rd.feed(data):
                    mt = msg[0] if msg else 0
                    name = L1CTL_NAMES.get(mt, f"0x{mt:02x}")
                    print(f"[pty-tx] mobile→fw L1CTL {name} "
                          f"len={len(msg)} {msg[:16].hex()}", flush=True)

                    if not firmware_ready:
                        pending_l1ctl.append(msg)
                        continue
                    frame = sercomm_wrap(DLCI_L1CTL, msg)
                    try:
                        os.write(pty_fd, frame)
                        stats["l1ctl→pty"] += 1
                    except BlockingIOError:
                        pass
                continue

            # ── PTY: firmware → mobile (parse sercomm, DLCI 5 only) ──
            if fd == pty_fd:
                try:
                    data = os.read(pty_fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    continue
                print(f"[pty-rx] {len(data)} bytes: {data[:32].hex()}"
                      f"{'...' if len(data) > 32 else ''}", flush=True)
                for dlci, payload in sercomm.feed(data):
                    if dlci == DLCI_L1CTL and payload:
                        mt = payload[0]
                        name = L1CTL_NAMES.get(mt, f"0x{mt:02x}")
                        print(f"[pty-rx] fw→mobile L1CTL {name} "
                              f"len={len(payload)} {payload[:16].hex()}",
                              flush=True)
                    else:
                        print(f"[pty-rx]   parsed DLCI={dlci} payload={payload[:16].hex()}",
                              flush=True)
                    # PTY → BTS: firmware emits TRXC RSP wrapped in DLCI 4
                    # → unwrap and forward to osmo-bts-trx UDP 5701.
                    if dlci == DLCI_TRXC:
                        rs = payload.rstrip(b'\x00').decode(errors='replace')
                        print(f"[pty→bts] {rs!r}", flush=True)
                        try:
                            bts_trxc.sendto(payload + b'\x00', bts_trxc_dst)
                        except OSError as e:
                            print(f"PTY→BTS fail: {e}", flush=True)
                        continue
                    # First DLCI 5 frame from firmware = "ready" signal.
                    if dlci == DLCI_L1CTL and not firmware_ready:
                        firmware_ready = True
                        print(f"firmware ready (saw DLCI 5 from PTY); "
                              f"draining {len(pending_l1ctl)} pending mobile msgs",
                              flush=True)
                        for buffered in pending_l1ctl:
                            try:
                                os.write(pty_fd,
                                         sercomm_wrap(DLCI_L1CTL, buffered))
                                stats["l1ctl→pty"] += 1
                            except BlockingIOError:
                                pass
                        pending_l1ctl.clear()
                    # L1CTL: firmware → bridge → mobile via unix socket
                    if dlci == DLCI_L1CTL and cli is not None:
                        frame = struct.pack(">H", len(payload)) + payload
                        try:
                            cli.sendall(frame)
                            stats["pty→l1ctl"] += 1
                        except Exception:
                            pass
                continue

            # ── TRXC BTS → bridge local stub ──
            # Bridge answers TRXC commands locally (handshake-only proxy,
            # like fake_trx / si4-working). QEMU is never involved.
            if fd == bts_trxc.fileno():
                try:
                    pkt, addr = bts_trxc.recvfrom(1024)
                except BlockingIOError:
                    continue
                if not pkt:
                    continue
                bts_trxc_dst = addr
                rsp = trxc_response(pkt)
                if rsp is None:
                    continue
                vs = pkt.strip(b'\x00').decode(errors='replace')
                if "POWERON" in vs:
                    powered = True
                elif "POWEROFF" in vs:
                    powered = False
                print(f"[trxc] {vs!r} -> {rsp!r}", flush=True)
                try:
                    bts_trxc.sendto((rsp + "\0").encode(), addr)
                    stats["trxc"] += 1
                except OSError as e:
                    print(f"TRXC RSP fail: {e}", flush=True)
                continue

            # ── TRXD BTS → gate path ──
            # Forward DL bursts received from osmo-bts-trx (BTS-side) to
            # the QEMU gate (6702). Default: forward (e2e mode). Set
            # BRIDGE_DROP_BTS_DL=1 to drain-and-drop (inject-only mode).
            # ── DL TRXDv0 from osmo-bts-trx → CCCH decode → DATA_IND ──
            # Wire: byte0=(ver<<4)|tn, [1..4]=fn BE, byte5=att,
            #       [6..153]=148 hard bits {0,1}.  Total 154 bytes.
            if fd == bts_trxd.fileno():
                try:
                    pkt, addr = bts_trxd.recvfrom(4096)
                except BlockingIOError:
                    continue
                if not pkt or len(pkt) < 6 + 148:
                    continue
                bts_trxd_dst = addr
                tn = pkt[0] & 0x07
                fn = struct.unpack(">I", pkt[1:5])[0]
                bits = pkt[6:6 + 148]
                stats["dl"] += 1
                if stats["dl"] <= 20 or stats["dl"] % 500 == 0:
                    print(f"[burst DL] tn={tn} fn={fn} "
                          f"hdr={pkt[:6].hex()} bits[:16]={bits[:16].hex()}",
                          flush=True)

                # Mirror raw burst into QEMU BSP on 6702 so the DSP DMA
                # path also receives it. The BSP DL CB parses the same
                # 6+148 wire format and writes hard bits into DARAM
                # (CALYPSO_BSP_DARAM_ADDR..+LEN). Bridge keeps decoding
                # CCCH locally for the L1CTL_DATA_IND path — both run
                # in parallel.
                try:
                    bb_trxd.sendto(pkt, bb_trxd_dst)
                except OSError:
                    pass

                # Only TS0 carries BCCH/CCCH
                if tn != 0:
                    continue
                fn51 = fn % 51
                # CCCH/BCCH 4-burst block boundaries on TS0
                block_starts = (2, 6, 12, 16)
                block_id  = None
                burst_idx = None
                chan_nr   = 0x80
                for st in block_starts:
                    if st <= fn51 < st + 4:
                        block_id  = (fn // 51, st)
                        burst_idx = fn51 - st
                        chan_nr   = 0x80 if st == 2 else 0x90
                        break
                if block_id is None:
                    continue

                if block_id not in burst_buf:
                    burst_buf[block_id] = [None] * 4
                burst_buf[block_id][burst_idx] = extract_burst_data(bits)

                if all(b is not None for b in burst_buf[block_id]):
                    bursts = burst_buf.pop(block_id)
                    l2 = decode_ccch_block(bursts)
                    if l2 and len(l2) == 23 and cli is not None:
                        hdr     = struct.pack("BBxx", MSG_DATA_IND, 0)
                        info_dl = struct.pack(">BBHIBBBB",
                                              chan_nr, 0, TARGET_ARFCN, fn,
                                              48, 20, 0, 0)
                        msg     = hdr + info_dl + l2
                        frame   = struct.pack(">H", len(msg)) + msg
                        try:
                            cli.sendall(frame)
                            stats["decoded"] += 1
                            if stats["decoded"] <= 10 or stats["decoded"] % 100 == 0:
                                print(f"[air→mobile] DATA_IND #{stats['decoded']} "
                                      f"fn={fn} ch=0x{chan_nr:02x} {l2.hex()}",
                                      flush=True)
                        except Exception as e:
                            print(f"[air→mobile] send fail: {e}", flush=True)

                # Garbage-collect stale blocks (>200 frames old)
                if len(burst_buf) > 64:
                    cutoff = (fn // 51) - 4
                    for k in [k for k in burst_buf if k[0] < cutoff]:
                        del burst_buf[k]
                continue
            # ── TRXD gate → BTS (UL bursts) ──
            if fd == bb_trxd.fileno():
                try:
                    pkt, addr = bb_trxd.recvfrom(4096)
                except BlockingIOError:
                    continue
                if not pkt:
                    continue
                bb_trxd_dst = addr
                if len(pkt) >= 8:
                    tn = pkt[0] & 0x07
                    fn_ = ((pkt[1]<<24)|(pkt[2]<<16)|(pkt[3]<<8)|pkt[4])
                    if stats["ul"] < 20 or stats["ul"] % 200 == 0:
                        print(f"[burst UL qemu→bts] tn={tn} fn={fn_} "
                              f"len={len(pkt)} {pkt[:16].hex()}", flush=True)
                try:
                    bts_trxd.sendto(pkt, bts_trxd_dst)
                    stats["ul"] += 1
                except OSError:
                    pass
                continue

            # ── 6801 (mobile via UDP) → UART sercomm DLCI 5 ──
            if fd == bb_l1ctl.fileno():
                try:
                    pkt, addr = bb_l1ctl.recvfrom(4096)
                except BlockingIOError:
                    continue
                if not pkt:
                    continue
                bb_l1ctl_dst = addr
                # Frames are length-prefixed L1CTL: parse them and wrap
                # each one in sercomm DLCI 5 toward the UART.
                _r = LengthPrefixReader()
                for msg in _r.feed(pkt):
                    if not msg:
                        continue
                    if not firmware_ready:
                        pending_l1ctl.append(msg)
                        continue
                    try:
                        os.write(pty_fd, sercomm_wrap(DLCI_L1CTL, msg))
                        stats["l1ctl→pty"] += 1
                        mt = msg[0]
                        name = L1CTL_NAMES.get(mt, f"0x{mt:02x}")
                        print(f"[6801→pty] L1CTL {name} len={len(msg)}",
                              flush=True)
                    except (BlockingIOError, OSError) as e:
                        print(f"[6801→pty] fail: {e}", flush=True)
                continue

        now = time.monotonic()
        if now - last_print > 5:
            last_print = now
            line = " ".join(f"{k}={v}" for k, v in stats.items() if v)
            if line:
                print("stats: " + line, flush=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
