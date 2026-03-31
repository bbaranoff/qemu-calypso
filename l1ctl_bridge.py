#!/usr/bin/env python3
"""
l1ctl_bridge.py — Bridge between mobile (layer23) and QEMU firmware via sercomm.

Firmware-first mode: all L1CTL messages are forwarded to the firmware.
Only RESET_REQ and PM_REQ are handled synthetically (firmware doesn't support them).
The firmware handles FBSB, CCCH, RACH, DM_EST, DATA, etc. via its real DSP/TPU.

Architecture:
  mobile <-> /tmp/osmocom_l2 (L1CTL) <-> this bridge <-> PTY (sercomm) <-> QEMU firmware

Usage:
  python3 l1ctl_bridge.py /dev/pts/X [/tmp/osmocom_l2_ms]
"""

import errno
import fcntl
import os
import select
import signal
import socket
import struct
import sys
import termios
import time

# ---- Sercomm constants ----
DLCI_L1CTL   = 5
DLCI_LOADER  = 9
DLCI_DEBUG   = 4
DLCI_CONSOLE = 10
CTRL         = 0x03

FLAG         = 0x7E
ESCAPE       = 0x7D
ESCAPE_XOR   = 0x20

L1CTL_SOCK   = "/tmp/osmocom_l2"

L1CTL_NAMES = {
    0x01: "FBSB_REQ", 0x02: "FBSB_CONF", 0x03: "DATA_IND",
    0x04: "RACH_REQ", 0x05: "RACH_CONF", 0x06: "DATA_REQ",
    0x07: "RESET_IND", 0x08: "PM_REQ", 0x09: "PM_CONF",
    0x0A: "ECHO_REQ", 0x0B: "ECHO_CONF", 0x0C: "DATA_CONF",
    0x0D: "RESET_REQ", 0x0E: "RESET_CONF",
    0x10: "CCCH_MODE_REQ", 0x11: "CCCH_MODE_CONF",
    0x14: "DM_EST_REQ", 0x15: "DM_FREQ_REQ", 0x16: "DM_REL_REQ",
    0x1D: "TCH_MODE_REQ", 0x1E: "TCH_MODE_CONF",
    0x21: "TRAFFIC_REQ", 0x22: "TRAFFIC_CONF", 0x23: "TRAFFIC_IND",
}

# Correct L1CTL message type values (from l1ctl_proto.h)
MSG_RESET_IND  = 0x07
MSG_PM_REQ     = 0x08
MSG_PM_CONF    = 0x09
MSG_RESET_REQ  = 0x0D
MSG_RESET_CONF = 0x0E
MSG_BURST_IND  = 0x1F  # L1CTL_BURST_IND (31) from sylvain/bursts_ind
MSG_DATA_IND   = 0x03  # L1CTL_DATA_IND


# ---- GSM channel decoding (from l1ctl_fake_trx_bridge.py) ----

def viterbi_decode(coded_bits, n_output):
    N_STATES = 16
    INF = float('inf')
    pm = [INF] * N_STATES
    pm[0] = 0
    paths = [[] for _ in range(N_STATES)]
    n_steps = len(coded_bits) // 2
    for i in range(n_steps):
        c0 = coded_bits[2 * i]
        c1 = coded_bits[2 * i + 1]
        new_pm = [INF] * N_STATES
        new_paths = [None] * N_STATES
        for state in range(N_STATES):
            if pm[state] == INF:
                continue
            for bit in (0, 1):
                reg = ((state << 1) | bit) & 0x1F
                g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1
                g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1
                bm = (c0 ^ g0) + (c1 ^ g1)
                ns = ((state << 1) | bit) & 0xF
                total = pm[state] + bm
                if total < new_pm[ns]:
                    new_pm[ns] = total
                    new_paths[ns] = paths[state] + [bit]
        pm = new_pm
        paths = new_paths
    best = min(range(N_STATES), key=lambda s: pm[s])
    result = paths[best]
    return result[:n_output] if result else []


def deinterleave_ccch(bursts_data):
    if len(bursts_data) != 4:
        return None
    coded = [0] * 456
    for k in range(456):
        coded[k] = bursts_data[k % 4][k // 4]
    return coded


def decode_ccch_block(bursts_data):
    """Decode using full libosmocoding (deinterleave + Viterbi + CRC)"""
    import ctypes
    try:
        lib = ctypes.CDLL('/usr/local/lib/libosmocoding.so')
    except Exception:
        return None
    # Build iB: 4 * 116 sbit_t
    iB = (ctypes.c_int8 * (4 * 116))()
    for i in range(4):
        for j in range(57):
            iB[i*116 + j] = -127 if bursts_data[i][j] else 127
        iB[i*116 + 57] = 0
        iB[i*116 + 58] = 0
        for j in range(57):
            iB[i*116 + 59 + j] = -127 if bursts_data[i][57+j] else 127
    # Full decode: gsm0503_xcch_decode does deinterleave + viterbi + CRC
    l2 = (ctypes.c_uint8 * 23)()
    n_err = ctypes.c_int(0)
    n_bits = ctypes.c_int(0)
    rv = lib.gsm0503_xcch_decode(l2, iB, ctypes.byref(n_err), ctypes.byref(n_bits))
    # Accept even with CRC errors (fire_crc will tell the mobile)
    return bytes(l2)


def extract_burst_data(bits_148):
    """Extract 114 coded bits from 148 bits of a normal burst.
    Handles both hard bits (0/1) and soft bits (0-255)."""
    raw = list(bits_148[:148])
    if len(raw) < 148:
        raw.extend([0] * (148 - len(raw)))
    # Detect format: if max > 1, it's soft bits
    if max(raw) > 1:
        hard = [1 if b > 127 else 0 for b in raw]
    else:
        hard = raw
    lower = hard[3:60]    # 57 data bits (positions 3-59)
    upper = hard[88:145]  # 57 data bits (positions 88-144)
    return lower + upper  # 114 coded bits

# ---- end GSM decoding ----


def hexdump(data, maxlen=32):
    h = " ".join(f"{b:02x}" for b in data[:maxlen])
    if len(data) > maxlen:
        h += f" ... ({len(data)} bytes)"
    return h


def set_raw(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def set_nonblock(fd):
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)


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
        frames = []
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
            frames.append((frame[0], frame[2:]))
        return frames


class LengthPrefixReader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        msgs = []
        while len(self.buf) >= 2:
            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break
            msgs.append(bytes(self.buf[2:2 + msglen]))
            del self.buf[:2 + msglen]
        return msgs


def l1ctl_send(client, msg_type, payload=b""):
    """Send L1CTL message to mobile with length prefix"""
    hdr = struct.pack("BBxx", msg_type, 0)
    msg = hdr + payload
    frame = struct.pack(">H", len(msg)) + msg
    try:
        client.sendall(frame)
    except Exception:
        pass


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path> [socket-path]", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]
    if len(sys.argv) > 2:
        global L1CTL_SOCK
        L1CTL_SOCK = sys.argv[2]

    try:
        os.unlink(L1CTL_SOCK)
    except FileNotFoundError:
        pass

    # Open PTY
    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    # L1CTL server socket
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(L1CTL_SOCK)
    srv.listen(1)
    srv.setblocking(False)

    # Air UDP socket — receive bursts and decode CCCH
    AIR_PORT = int(os.environ.get("BRIDGE_AIR_PORT", "4801"))
    air_sock = None
    if AIR_PORT > 0:
        air_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        air_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            air_sock.bind(("0.0.0.0", AIR_PORT + 100))
            air_sock.setblocking(False)
            print(f"l1ctl-bridge: air UDP on port {AIR_PORT + 100}", flush=True)
        except OSError:
            air_sock = None
            print(f"l1ctl-bridge: air UDP port {AIR_PORT + 100} busy",
                  flush=True)
    else:
        print(f"l1ctl-bridge: air UDP disabled", flush=True)

    print(f"l1ctl-bridge: PTY={pty_path} socket={L1CTL_SOCK}", flush=True)
    print(f"l1ctl-bridge: firmware-first mode (synth: RESET, PM only)", flush=True)

    parser = SercommParser()
    lp = LengthPrefixReader()
    client = None
    running = True
    stats = {"tx": 0, "rx": 0, "console": 0, "burst": 0, "decoded": 0}

    TARGET_ARFCN = 514

    # Burst accumulator for CCCH decoding (4 bursts per block)
    burst_buf = {}  # block_id → [burst0, burst1, burst2, burst3]

    def shutdown(_sig, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [srv, pty_fd]
            if air_sock:
                rlist.append(air_sock)
            if client is not None:
                rlist.append(client)

            try:
                readable, _, _ = select.select(rlist, [], [], 0.5)
            except (OSError, ValueError):
                break

            # ---- Accept new mobile connection ----
            if srv in readable:
                conn, _ = srv.accept()
                conn.setblocking(False)
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
                client = conn
                lp = LengthPrefixReader()
                print("l1ctl-bridge: mobile connected", flush=True)

            # ============================================================
            # mobile → firmware
            # ============================================================
            if client is not None and client in readable:
                try:
                    raw = client.recv(4096)
                except BlockingIOError:
                    raw = b""

                if not raw:
                    print("l1ctl-bridge: mobile disconnected", flush=True)
                    try:
                        client.close()
                    except OSError:
                        pass
                    client = None
                else:
                    for payload in lp.feed(raw):
                        stats["tx"] += 1
                        msg_type = payload[0] if payload else 0xFF
                        name = L1CTL_NAMES.get(msg_type, f"0x{msg_type:02x}")

                        # ---- RESET_REQ — synthetic only ----
                        # Firmware layer1 doesn't handle L1CTL RESET.
                        # Respond with RESET_CONF + RESET_IND to start the phone.
                        if msg_type == MSG_RESET_REQ:
                            reset_type = payload[4] if len(payload) > 4 else 1
                            l1ctl_send(client, MSG_RESET_CONF,
                                       struct.pack("Bxxx", reset_type))
                            print(f"  [synth] RESET_REQ type={reset_type} → RESET_CONF",
                                  flush=True)
                            continue  # don't forward

                        # ---- PM_REQ — synthetic only ----
                        # Firmware causes msgb pool exhaustion with PM.
                        # Return strong signal on TARGET_ARFCN, noise elsewhere.
                        if msg_type == MSG_PM_REQ and len(payload) >= 12:
                            arfcn_from = (payload[8] << 8) | payload[9]
                            arfcn_to = (payload[10] << 8) | payload[11]
                            entries = []
                            for a in range(arfcn_from, arfcn_to + 1):
                                rxlev = 0x30 if a == TARGET_ARFCN else 0x00
                                entries.append(struct.pack(">HBB", a, rxlev, rxlev))
                            # Send in batches
                            batch_size = 50
                            for i in range(0, len(entries), batch_size):
                                batch = entries[i:i + batch_size]
                                is_last = (i + batch_size >= len(entries))
                                hdr = bytes([MSG_PM_CONF,
                                             0x01 if is_last else 0x00,
                                             0x00, 0x00])
                                pm_data = hdr + b"".join(batch)
                                msg = struct.pack(">H", len(pm_data)) + pm_data
                                try:
                                    client.sendall(msg)
                                except (BrokenPipeError, OSError):
                                    pass
                            print(f"  [synth] PM_REQ {arfcn_from}-{arfcn_to} → PM_CONF",
                                  flush=True)
                            continue  # don't forward

                        # ---- Everything else → forward to firmware ----
                        frame = sercomm_wrap(DLCI_L1CTL, payload)
                        try:
                            os.write(pty_fd, frame)
                        except OSError as e:
                            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                raise
                        if True:
                            print(f"  [mobile→fw] #{stats['tx']} {name} "
                                  f"len={len(payload)} {hexdump(payload, 16)}",
                                  flush=True)

            # ============================================================
            # air bursts → BURST_IND to mobile
            # ============================================================
            if air_sock and air_sock in readable:
                try:
                    data, addr = air_sock.recvfrom(512)
                except OSError:
                    data = b""
                if len(data) >= 9 and client is not None:
                    tn = data[0] & 0x07  # TRXD: lower 3 bits = TN
                    fn = struct.unpack(">I", data[1:5])[0]
                    soft_bits = data[8:]  # TRXD: skip 8-byte header (TN+FN+RSSI+TOA)
                    stats["burst"] += 1

                    # Only process TS0 (BCCH/CCCH)
                    if tn != 0:
                        continue

                    # Extract 114 coded bits from the normal burst
                    coded = extract_burst_data(soft_bits)

                    # Determine block position in 51-multiframe
                    # Combined CCCH+SDCCH/4: BCCH=2-5, CCCH=6-9,12-15,16-19
                    fn51 = fn % 51
                    bcch_ccch_starts = [2, 6, 12, 16]
                    block_id = None
                    burst_idx = None
                    chan_nr = 0x80
                    for start in bcch_ccch_starts:
                        if start <= fn51 < start + 4:
                            burst_idx = fn51 - start
                            block_id = (fn // 51, start)
                            if start == 2:
                                chan_nr = 0x80  # BCCH
                            else:
                                chan_nr = 0x90  # CCCH
                            break

                    if block_id is not None and burst_idx is not None:
                        if block_id not in burst_buf:
                            burst_buf[block_id] = [None] * 4
                        burst_buf[block_id][burst_idx] = coded
                        if burst_idx == 3 and chan_nr == 0x80 and stats["decoded"] <= 2200:
                            # Dump full block for debug
                            print(f"  [BLOCK] FN={fn} fn51={fn51} ch=0x{chan_nr:02x}",
                                  flush=True)
                            for bi in range(4):
                                b = burst_buf[block_id][bi]
                                if b:
                                    print(f"    burst[{bi}]: {''.join(str(x) for x in b[:30])}...",
                                          flush=True)

                        # All 4 bursts received?
                        if all(b is not None for b in burst_buf[block_id]):
                            l2 = decode_ccch_block(burst_buf[block_id])
                            del burst_buf[block_id]

                            if l2 and len(l2) == 23:
                                # Send DATA_IND to mobile
                                hdr = struct.pack("BBxx", MSG_DATA_IND, 0)
                                # l1ctl_info_dl: chan_nr, link_id, arfcn(BE), fn(BE), rxlev, snr, num_biterr, fire_crc
                                info_dl = struct.pack(">BBHIBBBB",
                                    chan_nr, 0, TARGET_ARFCN, fn, 48, 20, 0, 0)
                                msg = hdr + info_dl + l2
                                frame = struct.pack(">H", len(msg)) + msg
                                try:
                                    client.sendall(frame)
                                except (BrokenPipeError, OSError):
                                    pass
                                stats["decoded"] += 1
                                if stats["decoded"] <= 10 or stats["decoded"] % 100 == 0:
                                    print(f"  [air→mobile] DATA_IND #{stats['decoded']} "
                                          f"FN={fn} ch=0x{chan_nr:02x} {l2.hex()}",
                                          flush=True)

                    # Clean old blocks
                    stale = [k for k in burst_buf if fn - k[0] * 51 > 200]
                    for k in stale:
                        del burst_buf[k]

                    if stats["burst"] <= 30 or stats["burst"] % 500 == 0:
                        fn51 = fn % 51
                        print(f"  [air] burst #{stats['burst']} FN={fn} "
                              f"fn51={fn51} decoded={stats['decoded']}",
                              flush=True)

            # ============================================================
            # firmware → mobile
            # ============================================================
            if pty_fd in readable:
                try:
                    raw = os.read(pty_fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        continue
                    raise

                if not raw:
                    print("l1ctl-bridge: PTY closed", flush=True)
                    break

                for dlci, payload in parser.feed(raw):
                    if dlci == DLCI_L1CTL:
                        # Filter out DATA_IND with chan_nr=0x00 from firmware
                        # (firmware sends bad NB results, bridge decodes CCCH)
                        if (payload and payload[0] == MSG_DATA_IND and
                                len(payload) >= 5 and payload[4] == 0x00):
                            continue
                        stats["rx"] += 1
                        if client is not None:
                            msg = struct.pack(">H", len(payload)) + payload
                            try:
                                client.sendall(msg)
                            except (BrokenPipeError, OSError):
                                print("l1ctl-bridge: mobile send error",
                                      flush=True)
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None

                        if stats["rx"] <= 50 or stats["rx"] % 50 == 0:
                            rx_name = L1CTL_NAMES.get(
                                payload[0] if payload else 0xFF,
                                f"0x{payload[0]:02x}" if payload else "?")
                            # For FBSB_CONF, show result byte
                            extra = ""
                            if payload and payload[0] == 0x02 and len(payload) >= 19:
                                extra = f" result={payload[18]} bsic={payload[19]}"
                            print(f"  [fw→mobile] #{stats['rx']} {rx_name} "
                                  f"len={len(payload)}{extra} {hexdump(payload, 20)}",
                                  flush=True)

                    elif dlci == DLCI_CONSOLE:
                        stats["console"] += 1
                        text_bytes = payload
                        if text_bytes and text_bytes[0] == 0x03:
                            text_bytes = text_bytes[1:]
                        try:
                            sys.stderr.write(text_bytes.decode("ascii",
                                                                errors="replace"))
                            sys.stderr.flush()
                        except Exception:
                            pass

                    elif dlci == DLCI_LOADER:
                        pass

    finally:
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        srv.close()
        try:
            os.close(pty_fd)
        except OSError:
            pass
        try:
            os.unlink(L1CTL_SOCK)
        except OSError:
            pass
        print(f"l1ctl-bridge: done (tx={stats['tx']} rx={stats['rx']} "
              f"console={stats['console']})", flush=True)


if __name__ == "__main__":
    main()
