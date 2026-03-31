#!/usr/bin/env python3
"""
l1ctl_bridge.py — Multi-client bridge: launches QEMU, detects PTYs, bridges L1CTL.

All-in-one: spawns QEMU firmware, grabs PTY from log, sends 'cont' via monitor
socket (socat), then creates N L1CTL unix sockets for mobile clients.

Firmware-first mode: all L1CTL messages are forwarded to the firmware.
Only RESET_REQ and PM_REQ are handled synthetically.

Architecture:
  QEMU (auto-launched) --(PTY sercomm)--> this bridge --(N unix sockets)--> mobile clients
                                                      --(air UDP)--> burst decode

Usage:
  python3 l1ctl_bridge.py -k KERNEL [-n 4] [-s /tmp/osmocom_l2] [--qemu-bin PATH]
  python3 l1ctl_bridge.py --pty /dev/pts/X [-n 4] [-s /tmp/osmocom_l2]  # skip QEMU launch
"""

import errno
import fcntl
import os
import re
import select
import signal
import socket
import struct
import subprocess
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


def l1ctl_send(sock, msg_type, payload=b""):
    """Send L1CTL message to a single client with length prefix"""
    hdr = struct.pack("BBxx", msg_type, 0)
    msg = hdr + payload
    frame = struct.pack(">H", len(msg)) + msg
    try:
        sock.sendall(frame)
    except Exception:
        pass


def broadcast_raw(clients, frame):
    """Send raw length-prefixed frame to all connected clients"""
    dead = []
    for cid, info in clients.items():
        try:
            info["sock"].sendall(frame)
        except (BrokenPipeError, OSError):
            dead.append(cid)
    for cid in dead:
        print(f"l1ctl-bridge: client#{cid} send error, removing", flush=True)
        try:
            clients[cid]["sock"].close()
        except OSError:
            pass
        del clients[cid]


def broadcast_l1ctl(clients, payload):
    """Broadcast an L1CTL payload (with length prefix) to all clients"""
    frame = struct.pack(">H", len(payload)) + payload
    broadcast_raw(clients, frame)


def launch_qemu(kernel, qemu_bin, mon_sock, env_extra=None, logfile="/tmp/qemu-bridge.log"):
    """Launch QEMU, wait for PTY, send 'cont' via monitor socket.
    Returns (pty_path, qemu_process)."""
    # Cleanup stale monitor socket
    try:
        os.unlink(mon_sock)
    except FileNotFoundError:
        pass

    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)

    cmd = [
        qemu_bin, "-M", "calypso", "-cpu", "arm946",
        "-display", "none",
        "-serial", "pty", "-serial", "pty",
        "-monitor", f"unix:{mon_sock},server,nowait",
        "-kernel", kernel,
    ]
    print(f"l1ctl-bridge: launching QEMU: {' '.join(cmd)}", flush=True)

    log_fd = open(logfile, "w")
    proc = subprocess.Popen(cmd, stdout=log_fd, stderr=subprocess.STDOUT, env=env)

    # Wait for PTY to appear in log (up to 10s)
    pty_path = None
    for _ in range(100):
        time.sleep(0.1)
        if proc.poll() is not None:
            log_fd.close()
            with open(logfile) as f:
                print(f"l1ctl-bridge: QEMU exited! log:\n{f.read()}", flush=True)
            sys.exit(1)
        try:
            with open(logfile) as f:
                content = f.read()
            ptys = re.findall(r"/dev/pts/\d+", content)
            if ptys:
                pty_path = ptys[0]
                break
        except FileNotFoundError:
            pass

    if not pty_path:
        print("l1ctl-bridge: FATAL: could not detect PTY from QEMU", flush=True)
        proc.kill()
        sys.exit(1)

    print(f"l1ctl-bridge: QEMU PTY detected: {pty_path}", flush=True)

    # Send 'cont' via monitor socket
    time.sleep(1)
    try:
        mon = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        mon.connect(mon_sock)
        mon.sendall(b"cont\n")
        time.sleep(0.2)
        mon.close()
        print(f"l1ctl-bridge: sent 'cont' to QEMU monitor", flush=True)
    except OSError as e:
        print(f"l1ctl-bridge: WARNING: monitor connect failed: {e}", flush=True)

    return pty_path, proc


def main():
    import argparse
    ap = argparse.ArgumentParser(
        description="Multi-client L1CTL bridge with optional QEMU auto-launch")
    ap.add_argument("-k", "--kernel", default=None,
                    help="Firmware ELF to launch QEMU with")
    ap.add_argument("--pty", default=None,
                    help="Use existing PTY (skip QEMU launch)")
    ap.add_argument("-n", "--num-clients", type=int, default=4,
                    help="Number of L1CTL client sockets (default: 4)")
    ap.add_argument("-s", "--socket", default="/tmp/osmocom_l2",
                    help="Base socket path (default: /tmp/osmocom_l2)")
    ap.add_argument("--qemu-bin", default="./build/qemu-system-arm",
                    help="Path to qemu-system-arm binary")
    ap.add_argument("--mon-sock", default="/tmp/qemu-calypso-mon-bridge.sock",
                    help="QEMU monitor socket path")
    ap.add_argument("--env", action="append", default=[],
                    help="Extra env vars for QEMU (KEY=VAL), repeatable")
    # Legacy positional args support: l1ctl_bridge.py /dev/pts/X [socket]
    ap.add_argument("legacy_pty", nargs="?", default=None)
    ap.add_argument("legacy_sock", nargs="?", default=None)
    args = ap.parse_args()

    # Resolve PTY source
    qemu_proc = None
    if args.kernel:
        env_extra = {}
        for e in args.env:
            k, v = e.split("=", 1)
            env_extra[k] = v
        pty_path, qemu_proc = launch_qemu(
            args.kernel, args.qemu_bin, args.mon_sock, env_extra)
    elif args.pty:
        pty_path = args.pty
    elif args.legacy_pty:
        pty_path = args.legacy_pty
        if args.legacy_sock:
            args.socket = args.legacy_sock
    else:
        ap.print_help()
        sys.exit(1)

    base_sock = args.socket
    num_clients = args.num_clients

    # Build socket paths: /tmp/osmocom_l2, /tmp/osmocom_l2.2, .3, .4 ...
    sock_paths = [base_sock]
    for n in range(2, num_clients + 1):
        sock_paths.append(f"{base_sock}.{n}")

    # Open PTY
    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    # Create one listening socket per client slot
    servers = {}  # sock_path -> {"srv": socket, "path": str, "slot": int}
    for slot, path in enumerate(sock_paths):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        srv.listen(1)
        srv.setblocking(False)
        servers[path] = {"srv": srv, "path": path, "slot": slot + 1}
        print(f"l1ctl-bridge: slot#{slot+1} listening on {path}", flush=True)

    # Air UDP socket — receive bursts and decode CCCH
    AIR_PORT = int(os.environ.get("BRIDGE_AIR_PORT", "6800"))
    air_sock = None
    if AIR_PORT > 0:
        air_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        air_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            air_sock.bind(("0.0.0.0", AIR_PORT))
            air_sock.setblocking(False)
            print(f"l1ctl-bridge: air UDP on port {AIR_PORT}", flush=True)
        except OSError:
            air_sock = None
            print(f"l1ctl-bridge: air UDP port {AIR_PORT} busy",
                  flush=True)
    else:
        print(f"l1ctl-bridge: air UDP disabled", flush=True)

    print(f"l1ctl-bridge: PTY={pty_path} sockets={num_clients}", flush=True)
    print(f"l1ctl-bridge: multi-client broadcast mode", flush=True)
    print(f"l1ctl-bridge: firmware-first (synth: RESET, PM only)", flush=True)

    TARGET_ARFCN = int(os.environ.get("TARGET_ARFCN", "514"))

    # Send POWERON to fake_trx from Python (QEMU's sendto doesn't flush)
    trxc_port = int(os.environ.get("CALYPSO_TRX_PORT", "6700"))
    if trxc_port > 0:
        trxc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        trxc_dest = ("127.0.0.1", trxc_port + 1)  # fake_trx CTRL = base+1
        # Compute DCS1800 / GSM900 frequencies
        if TARGET_ARFCN >= 512 and TARGET_ARFCN <= 885:
            dl_khz = 1805200 + (TARGET_ARFCN - 512) * 200
            ul_khz = 1710200 + (TARGET_ARFCN - 512) * 200
        else:
            dl_khz = 935000 + TARGET_ARFCN * 200
            ul_khz = 890000 + TARGET_ARFCN * 200
        for cmd in [f"CMD RXTUNE {dl_khz}", f"CMD TXTUNE {ul_khz}", "CMD POWERON"]:
            trxc_sock.sendto(cmd.encode() + b'\0', trxc_dest)
            print(f"l1ctl-bridge: → fake_trx: {cmd}", flush=True)
        # Read responses
        time.sleep(0.5)
        for _ in range(10):
            try:
                data, _ = trxc_sock.recvfrom(256)
                print(f"l1ctl-bridge: ← fake_trx: {data.decode(errors='replace').strip()}", flush=True)
            except BlockingIOError:
                break
        trxc_sock.setblocking(False)
        print(f"l1ctl-bridge: fake_trx POWERON done (ARFCN={TARGET_ARFCN})", flush=True)
    else:
        trxc_sock = None

    parser = SercommParser()
    # clients dict: client_id -> {"sock", "lp", "id", "slot", "path"}
    clients = {}
    next_client_id = 1
    running = True
    stats = {"tx": 0, "rx": 0, "console": 0, "burst": 0, "decoded": 0}

    # Burst accumulator for CCCH decoding (4 bursts per block)
    burst_buf = {}

    def shutdown(_sig, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [pty_fd]
            # Add all server sockets
            for info in servers.values():
                rlist.append(info["srv"])
            if air_sock:
                rlist.append(air_sock)
            for info in clients.values():
                rlist.append(info["sock"])

            try:
                readable, _, _ = select.select(rlist, [], [], 0.5)
            except (OSError, ValueError):
                break

            # ---- Accept new mobile connections on any server socket ----
            for path, sinfo in servers.items():
                if sinfo["srv"] in readable:
                    conn, _ = sinfo["srv"].accept()
                    conn.setblocking(False)
                    cid = next_client_id
                    next_client_id += 1
                    clients[cid] = {
                        "sock": conn,
                        "lp": LengthPrefixReader(),
                        "id": cid,
                        "slot": sinfo["slot"],
                        "path": path,
                    }
                    print(f"l1ctl-bridge: client#{cid} connected on "
                          f"{path} (slot#{sinfo['slot']}, "
                          f"total: {len(clients)})", flush=True)

            # ============================================================
            # mobile(s) → firmware  (any client can send)
            # ============================================================
            dead_clients = []
            for cid, info in list(clients.items()):
                csock = info["sock"]
                if csock not in readable:
                    continue
                try:
                    raw = csock.recv(4096)
                except BlockingIOError:
                    raw = b""

                if not raw:
                    print(f"l1ctl-bridge: client#{cid} disconnected", flush=True)
                    dead_clients.append(cid)
                    continue

                for payload in info["lp"].feed(raw):
                    stats["tx"] += 1
                    msg_type = payload[0] if payload else 0xFF
                    name = L1CTL_NAMES.get(msg_type, f"0x{msg_type:02x}")

                    # ---- RESET_REQ — synthetic, reply to requesting client ----
                    if msg_type == MSG_RESET_REQ:
                        reset_type = payload[4] if len(payload) > 4 else 1
                        l1ctl_send(csock, MSG_RESET_CONF,
                                   struct.pack("Bxxx", reset_type))
                        print(f"  [synth] client#{cid} RESET_REQ "
                              f"type={reset_type} → RESET_CONF", flush=True)
                        continue

                    # ---- PM_REQ — synthetic, reply to requesting client ----
                    if msg_type == MSG_PM_REQ and len(payload) >= 12:
                        arfcn_from = (payload[8] << 8) | payload[9]
                        arfcn_to = (payload[10] << 8) | payload[11]
                        entries = []
                        for a in range(arfcn_from, arfcn_to + 1):
                            rxlev = 0x30 if a == TARGET_ARFCN else 0x00
                            entries.append(struct.pack(">HBB", a, rxlev, rxlev))
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
                                csock.sendall(msg)
                            except (BrokenPipeError, OSError):
                                pass
                        print(f"  [synth] client#{cid} PM_REQ "
                              f"{arfcn_from}-{arfcn_to} → PM_CONF", flush=True)
                        continue

                    # ---- Everything else → forward to firmware ----
                    frame = sercomm_wrap(DLCI_L1CTL, payload)
                    try:
                        os.write(pty_fd, frame)
                    except OSError as e:
                        if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                            raise
                    print(f"  [client#{cid}→fw] #{stats['tx']} {name} "
                          f"len={len(payload)} {hexdump(payload, 16)}",
                          flush=True)

            # Clean up disconnected clients
            for cid in dead_clients:
                try:
                    clients[cid]["sock"].close()
                except OSError:
                    pass
                del clients[cid]

            # ============================================================
            # air bursts → DATA_IND broadcast to all clients
            # ============================================================
            if air_sock and air_sock in readable:
                try:
                    data, addr = air_sock.recvfrom(512)
                except OSError:
                    data = b""
                if len(data) >= 6 and clients:
                    tn = data[0]
                    fn = struct.unpack(">I", data[1:5])[0]
                    soft_bits = data[5:]
                    stats["burst"] += 1

                    if tn != 0:
                        pass  # Only process TS0
                    else:
                        coded = extract_burst_data(soft_bits)

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
                                    chan_nr = 0x80
                                else:
                                    chan_nr = 0x90
                                break

                        if block_id is not None and burst_idx is not None:
                            if block_id not in burst_buf:
                                burst_buf[block_id] = [None] * 4
                            burst_buf[block_id][burst_idx] = coded

                            if all(b is not None for b in burst_buf[block_id]):
                                l2 = decode_ccch_block(burst_buf[block_id])
                                del burst_buf[block_id]

                                if l2 and len(l2) == 23:
                                    hdr = struct.pack("BBxx", MSG_DATA_IND, 0)
                                    info_dl = struct.pack(">BBHIBBBB",
                                        chan_nr, 0, TARGET_ARFCN, fn,
                                        48, 20, 0, 0)
                                    msg = hdr + info_dl + l2
                                    frame = struct.pack(">H", len(msg)) + msg
                                    broadcast_raw(clients, frame)
                                    stats["decoded"] += 1
                                    if (stats["decoded"] <= 10 or
                                            stats["decoded"] % 100 == 0):
                                        print(f"  [air→ALL] DATA_IND "
                                              f"#{stats['decoded']} FN={fn} "
                                              f"ch=0x{chan_nr:02x} {l2.hex()}",
                                              flush=True)

                        stale = [k for k in burst_buf
                                 if fn - k[0] * 51 > 200]
                        for k in stale:
                            del burst_buf[k]

                        if stats["burst"] <= 30 or stats["burst"] % 500 == 0:
                            fn51 = fn % 51
                            print(f"  [air] burst #{stats['burst']} FN={fn} "
                                  f"fn51={fn51} decoded={stats['decoded']}",
                                  flush=True)

            # ============================================================
            # firmware → broadcast to ALL clients
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
                        if (payload and payload[0] == MSG_DATA_IND and
                                len(payload) >= 5 and payload[4] == 0x00):
                            continue
                        stats["rx"] += 1

                        # Broadcast to all connected clients
                        broadcast_l1ctl(clients, payload)

                        if stats["rx"] <= 50 or stats["rx"] % 50 == 0:
                            rx_name = L1CTL_NAMES.get(
                                payload[0] if payload else 0xFF,
                                f"0x{payload[0]:02x}" if payload else "?")
                            extra = ""
                            if payload and payload[0] == 0x02 and len(payload) >= 19:
                                extra = f" result={payload[18]} bsic={payload[19]}"
                            print(f"  [fw→ALL({len(clients)})] #{stats['rx']} "
                                  f"{rx_name} len={len(payload)}{extra} "
                                  f"{hexdump(payload, 20)}", flush=True)

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
        for cid, info in clients.items():
            try:
                info["sock"].close()
            except OSError:
                pass
        for path, sinfo in servers.items():
            try:
                sinfo["srv"].close()
            except OSError:
                pass
            try:
                os.unlink(path)
            except OSError:
                pass
        try:
            os.close(pty_fd)
        except OSError:
            pass
        if qemu_proc and qemu_proc.poll() is None:
            print("l1ctl-bridge: terminating QEMU...", flush=True)
            qemu_proc.terminate()
            try:
                qemu_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                qemu_proc.kill()
        print(f"l1ctl-bridge: done (tx={stats['tx']} rx={stats['rx']} "
              f"console={stats['console']} clients={len(clients)})", flush=True)


if __name__ == "__main__":
    main()
