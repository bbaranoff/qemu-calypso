#!/usr/bin/env python3
"""
l1ctl_bridge.py — Bridge between mobile (layer23) and QEMU firmware via sercomm.

Creates /tmp/osmocom_l2 and relays L1CTL messages:
  mobile  <-> /tmp/osmocom_l2 (length-prefixed) <-> this bridge <-> PTY (sercomm)

Sercomm DLCIs:
  4  = L1CTL (SC_DLCI_L1A_L23) → relayed to/from mobile
  9  = loader                   → ignored (loader phase is over)
  10 = console debug            → printed to stderr

Usage:
  python3 l1ctl_bridge.py /dev/pts/X
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
DLCI_L1CTL   = 5    # SC_DLCI_L1A_L23 (OsmocomBB: 5, not 4)
DLCI_LOADER  = 9
DLCI_DEBUG   = 4    # SC_DLCI_DEBUG
DLCI_CONSOLE = 10
CTRL         = 0x03

FLAG         = 0x7E
ESCAPE       = 0x7D
ESCAPE_XOR   = 0x20

L1CTL_SOCK   = "/tmp/osmocom_l2"  # overridden by argv[2] if given


def hexdump(data: bytes, maxlen: int = 32) -> str:
    h = " ".join(f"{b:02x}" for b in data[:maxlen])
    if len(data) > maxlen:
        h += f" ... ({len(data)} bytes)"
    return h


def set_raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                          # iflag
    attrs[1] = 0                                          # oflag
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0                                          # lflag
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def set_nonblock(fd: int) -> None:
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)


# ---- Sercomm framing ----

def sercomm_escape(data: bytes) -> bytes:
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def sercomm_unescape(data: bytes) -> bytes:
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


def sercomm_wrap(dlci: int, payload: bytes) -> bytes:
    """[FLAG][escaped: DLCI CTRL payload][FLAG]"""
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])


class SercommParser:
    """Extract sercomm frames from a byte stream."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
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
            dlci = frame[0]
            # frame[1] = CTRL byte, skip
            payload = frame[2:]
            frames.append((dlci, payload))
        return frames


class LengthPrefixReader:
    """Read [2-byte BE length][payload] messages from a stream socket."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        msgs = []
        while len(self.buf) >= 2:
            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break
            msgs.append(bytes(self.buf[2:2 + msglen]))
            del self.buf[:2 + msglen]
        return msgs


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path> [socket-path]", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]
    if len(sys.argv) > 2:
        global L1CTL_SOCK
        L1CTL_SOCK = sys.argv[2]

    # ---- Clean up old socket ----
    try:
        os.unlink(L1CTL_SOCK)
    except FileNotFoundError:
        pass

    # ---- Open serial path (PTY or Unix socket) ----
    if pty_path.endswith('.sock') or os.path.exists(pty_path) and not pty_path.startswith('/dev/'):
        # Unix socket mode
        import socket as sock_mod
        pty_sock = sock_mod.socket(sock_mod.AF_UNIX, sock_mod.SOCK_STREAM)
        pty_sock.connect(pty_path)
        pty_sock.setblocking(False)
        pty_fd = pty_sock.fileno()
        print(f"l1ctl-bridge: connected to socket {pty_path}", flush=True)
    else:
        # PTY mode
        pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        set_raw(pty_fd)
        set_nonblock(pty_fd)
        pty_sock = None

    # ---- Create L1CTL server socket ----
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(L1CTL_SOCK)
    srv.listen(1)
    srv.setblocking(False)

    print(f"l1ctl-bridge: listening on {L1CTL_SOCK}", flush=True)
    print(f"l1ctl-bridge: PTY={pty_path}", flush=True)
    print(f"l1ctl-bridge: L1CTL DLCI={DLCI_L1CTL}, console DLCI={DLCI_CONSOLE}",
          flush=True)
    print(f"l1ctl-bridge: waiting for mobile to connect...", flush=True)

    parser   = SercommParser()
    lp       = LengthPrefixReader()
    client   = None
    running  = True

    stats = {"tx": 0, "rx": 0, "console": 0}

    def shutdown(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [srv, pty_fd]
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

            # ---- mobile → firmware (L1CTL → sercomm DLCI 4) ----
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

                        # ---- Intercept L1CTL messages ----
                        import struct as st2

                        # ---- Intercept FBSB_REQ (0x01) ----
                        if len(payload) >= 4 and payload[0] == 0x01:
                            # Parse FBSB_REQ: payload[4:6] = band_arfcn
                            arfcn = 0
                            bsic = 7  # default BSIC
                            if len(payload) >= 6:
                                arfcn = (payload[4] << 8) | payload[5]
                            print(f"  [bridge] FBSB_REQ ARFCN={arfcn} (0x{arfcn:04x}) payload={payload[:8].hex()} → synthetic CONF(result=0)", flush=True)
                            # Build FBSB_CONF: l1ctl_hdr(4) + info_dl(12) + fbsb_conf(4)
                            hdr = bytes([0x02, 0x00, 0x00, 0x00])  # L1CTL_FBSB_CONF
                            info_dl = st2.pack(">BBHIBBBB",
                                0x80,                 # chan_nr (BCCH)
                                0,                    # link_id
                                arfcn,                # band_arfcn
                                0,                    # frame_nr
                                60,                   # rx_level
                                20,                   # snr
                                0,                    # num_biterr
                                0)                    # fire_crc
                            # BSIC: NCC=7, BCC=4 = 0x3c (match osmo-bsc config)
                            bsic = 7
                            fbsb_conf = st2.pack(">hBB",
                                0,                    # initial_freq_err
                                0,                    # result = 0 = SUCCESS
                                bsic)                 # bsic
                            resp = hdr + info_dl + fbsb_conf
                            msg = st2.pack(">H", len(resp)) + resp
                            if client is not None:
                                try:
                                    client.sendall(msg)
                                    stats["rx"] += 1
                                    print(f"  [bridge] → FBSB_CONF(result=0, BSIC={bsic})", flush=True)
                                except (BrokenPipeError, OSError):
                                    pass
                            # Start idle sender immediately after FBSB_CONF
                            import threading
                            if not hasattr(self if 'self' in dir() else type('',(),{'_idle_started':False}), '_idle_started') or True:
                                def _start_idle(cl, arfcn_val, delay=0.5):
                                    import time as _t
                                    _t.sleep(delay)  # Let FBSB_RESP be processed first
                                    fn_c = 1557053
                                    fc = 0
                                    si3_d = bytes([0x49,0x06,0x1b,0x00,0x01,0x00,0xF1,0x10,0x00,0x01,0x49,0x00,0x00,0x04,0x00,0x00,0x15,0x00,0x00,0x2b,0x2b,0x2b,0x2b])[:23]
                                    si1_d = bytes([0x55,0x06,0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15,0x00,0x00,0x00])[:23]
                                    si2_d = bytes([0x59,0x06,0x1a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x15,0x00,0x00])[:23]
                                    idle = bytes([0x01,0x03,0x01]) + bytes([0x2b]*20)
                                    si_rot = [si1_d, si2_d, si3_d]
                                    hd = bytes([0x03,0x00,0x00,0x00])
                                    while cl is not None and running:
                                        si_i = fc % 8
                                        if si_i < 3:
                                            cn = 0x80; pl = si_rot[si_i]
                                        else:
                                            cn = 0x90; pl = idle
                                        inf = st2.pack('>BBHIBBBB', cn, 0, arfcn_val, fn_c, 60, 20, 0, 0)
                                        mg = st2.pack('>H', len(hd)+len(inf)+len(pl)) + hd + inf + pl
                                        try:
                                            cl.sendall(mg)
                                        except (BrokenPipeError, OSError):
                                            break
                                        fc += 1; fn_c += 51
                                        if fn_c > 2715647: fn_c = 0
                                        _t.sleep(0.235)
                                t = threading.Thread(target=_start_idle, args=(client, arfcn), daemon=True)
                                t.start()
                                print('  [bridge] Started idle frame sender (SI1+SI2+SI3+PCH)', flush=True)
#SI_DELAYED#                             # Send a SI3 DATA_IND so mobile can camp
#SI_DELAYED#                             # SI3: MCC=001, MNC=01, LAC=1, CI=1
#SI_DELAYED#                             si3 = bytes([
#SI_DELAYED#                                 # -- L2 pseudo header (3 bytes) --
#SI_DELAYED#                                 0x49,             # l2_plen: L3 len=18, (18<<2)|0x01
#SI_DELAYED#                                 0x06,             # PD=6 (RR), skip_indicator=0
#SI_DELAYED#                                 0x1b,             # msg_type = SI3 (0x1b)
#SI_DELAYED#                                 # -- Cell Identity (2 bytes) --
#SI_DELAYED#                                 0x00, 0x01,       # CI = 1
#SI_DELAYED#                                 # -- LAI (5 bytes) --
#SI_DELAYED#                                 0x00, 0xF1, 0x10, # MCC=001, MNC=01 (BCD)
#SI_DELAYED#                                 0x00, 0x01,       # LAC = 1
#SI_DELAYED#                                 # -- Control Channel Desc (3 bytes) --
#SI_DELAYED#                                 0x49,             # ATT=1, BS_AG_BLKS_RES=1, CCCH_CONF=1(combined)
#SI_DELAYED#                                 0x00,             # BS_PA_MFRMS=0
#SI_DELAYED#                                 0x00,             # T3212=0 (no periodic LU)
#SI_DELAYED#                                 # -- Cell Options BCCH (1 byte) --
#SI_DELAYED#                                 0x04,             # PWRC=0, DTX=0, RADIO_LINK_TIMEOUT=4
#SI_DELAYED#                                 # -- Cell Selection Parameters (2 bytes) --
#SI_DELAYED#                                 0x00, 0x00,       # MS_TXPWR_MAX_CCCH=0, CELL_RESELECT_HYSTERESIS=0, RXLEV_ACCESS_MIN=0, NECI=0, ACS=0
#SI_DELAYED#                                 # -- RACH Control Parameters (3 bytes) --
#SI_DELAYED#                                 0x15, 0x00, 0x00, # RE=0, CELL_BAR=0, TX_INTEGER=5, MAX_TRANS=1, AC=none
#SI_DELAYED#                                 # -- SI3 Rest Octets (4 bytes, padding) --
#SI_DELAYED#                                 0x2b, 0x2b, 0x2b, 0x2b,
#SI_DELAYED#                             ])
#SI_DELAYED#                             si3_padded = si3[:23]
#SI_DELAYED# 
#SI_DELAYED#                             fn_si = 1557002  # Realistic FN, fn%51=2 (BCCH position)
#SI_DELAYED#                             hdr_d = bytes([0x03, 0x00, 0x00, 0x00])  # L1CTL_DATA_IND
#SI_DELAYED#                             info_dl_d = st2.pack(">BBHIBBBB",
#SI_DELAYED#                                 0x80, 0, arfcn, fn_si, 60, 20, 0, 0)
#SI_DELAYED#                             resp_d = hdr_d + info_dl_d + si3_padded
#SI_DELAYED#                             msg_d = st2.pack(">H", len(resp_d)) + resp_d
#SI_DELAYED#                             if client is not None:
#SI_DELAYED#                                 try:
#SI_DELAYED#                                     client.sendall(msg_d)
#SI_DELAYED#                                     stats["rx"] += 1
#SI_DELAYED#                                     print(f"  [bridge] → DATA_IND SI3", flush=True)
#SI_DELAYED#                                 except (BrokenPipeError, OSError):
#SI_DELAYED#                                     pass
#SI_DELAYED#                             # Send SI1 DATA_IND
#SI_DELAYED#                             si1 = bytes([
#SI_DELAYED#                                 # header: l2_plen, PD, msg_type
#SI_DELAYED#                                 0x55,  # l2_plen: (21<<2)|0x01 = 0x55
#SI_DELAYED#                                 0x06,  # PD=RR
#SI_DELAYED#                                 0x19,  # SI1
#SI_DELAYED#                                 # cell_channel_description (16 bytes) - all zeros = no channels
#SI_DELAYED#                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#SI_DELAYED#                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#SI_DELAYED#                                 # rach_control (3 bytes)
#SI_DELAYED#                                 0x15, 0x00, 0x00,  # same as SI3
#SI_DELAYED#                                 # rest octet
#SI_DELAYED#                             ])[:23]
#SI_DELAYED#                             hdr_s1 = bytes([0x03, 0x00, 0x00, 0x00])
#SI_DELAYED#                             info_s1 = st2.pack(">BBHIBBBB", 0x80, 0, arfcn, fn_si + 10, 60, 20, 0, 0)
#SI_DELAYED#                             resp_s1 = hdr_s1 + info_s1 + si1
#SI_DELAYED#                             msg_s1 = st2.pack(">H", len(resp_s1)) + resp_s1
#SI_DELAYED#                             if client is not None:
#SI_DELAYED#                                 try:
#SI_DELAYED#                                     client.sendall(msg_s1)
#SI_DELAYED#                                     stats["rx"] += 1
#SI_DELAYED#                                     print(f"  [bridge] → DATA_IND SI1", flush=True)
#SI_DELAYED#                                 except (BrokenPipeError, OSError):
#SI_DELAYED#                                     pass
#SI_DELAYED#                             # Send SI2 DATA_IND
#SI_DELAYED#                             si2 = bytes([
#SI_DELAYED#                                 # header
#SI_DELAYED#                                 0x59,  # l2_plen: (22<<2)|0x01 = 0x59
#SI_DELAYED#                                 0x06,  # PD=RR
#SI_DELAYED#                                 0x1a,  # SI2
#SI_DELAYED#                                 # bcch_frequency_list (16 bytes) - all zeros = no neighbors
#SI_DELAYED#                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#SI_DELAYED#                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#SI_DELAYED#                                 # ncc_permitted (1 byte)
#SI_DELAYED#                                 0xFF,  # all NCCs permitted
#SI_DELAYED#                                 # rach_control (3 bytes)
#SI_DELAYED#                                 0x15, 0x00, 0x00,
#SI_DELAYED#                             ])[:23]
#SI_DELAYED#                             hdr_s2 = bytes([0x03, 0x00, 0x00, 0x00])
#SI_DELAYED#                             info_s2 = st2.pack(">BBHIBBBB", 0x80, 0, arfcn, fn_si + 20, 60, 20, 0, 0)
#SI_DELAYED#                             resp_s2 = hdr_s2 + info_s2 + si2
#SI_DELAYED#                             msg_s2 = st2.pack(">H", len(resp_s2)) + resp_s2
#SI_DELAYED#                             if client is not None:
#SI_DELAYED#                                 try:
#SI_DELAYED#                                     client.sendall(msg_s2)
#SI_DELAYED#                                     stats["rx"] += 1
#SI_DELAYED#                                     print(f"  [bridge] → DATA_IND SI2", flush=True)
#SI_DELAYED#                                 except (BrokenPipeError, OSError):
#SI_DELAYED#                                     pass
#SI_DELAYED# 
#SI_DELAYED#                                 except (BrokenPipeError, OSError):
#SI_DELAYED#                                     pass

                            continue  # dont forward FBSB to firmware

                        # ---- Intercept RESET_REQ (0x0D) ----
                        # Respond with RESET_CONF since firmware is slow
                        if len(payload) >= 4 and payload[0] == 0x0D:
                            reset_type = payload[4] if len(payload) > 4 else 1
                            print(f"  [bridge] RESET_REQ type={reset_type} → synthetic CONF", flush=True)
                            hdr_r = bytes([0x0E, 0x00, 0x00, 0x00])  # L1CTL_RESET_CONF
                            conf_r = bytes([reset_type, 0x00, 0x00, 0x00])
                            resp_r = hdr_r + conf_r
                            msg_r = st2.pack(">H", len(resp_r)) + resp_r
                            if client is not None:
                                try:
                                    client.sendall(msg_r)
                                    stats["rx"] += 1
                                    print(f"  [bridge] → RESET_CONF", flush=True)
                                except (BrokenPipeError, OSError):
                                    pass
#DISABLED#                             # Also send RESET_IND
#DISABLED#                             hdr_i = bytes([0x07, 0x00, 0x00, 0x00])  # L1CTL_RESET_IND
#DISABLED#                             ind_i = bytes([0x00, 0x00, 0x00, 0x00])  # BOOT
#DISABLED#                             resp_i = hdr_i + ind_i
#DISABLED#                             msg_i = st2.pack(">H", len(resp_i)) + resp_i
#DISABLED#                             if client is not None:
#DISABLED#                                 try:
#DISABLED#                                     client.sendall(msg_i)
#DISABLED#                                     stats["rx"] += 1
#DISABLED#                                 except (BrokenPipeError, OSError):
#DISABLED#                                     pass
                            continue  # dont forward RESET to firmware

                        # ---- Intercept RACH_REQ (0x04) ----
                        if len(payload) >= 6 and payload[0] == 0x04:
                            # payload: hdr(4) + info_ul(4) + rach_req(ra, combined)
                            ra = payload[8] if len(payload) > 8 else 0
                            combined = payload[9] if len(payload) > 9 else 0
                            print(f"  [bridge] RACH_REQ ra=0x{ra:02x} combined={combined} payload={payload[:12].hex()}", flush=True)

                            # Send RACH_CONF (0x0C) with info_dl
                            fn_rach = 1557100
                            hdr_rc = bytes([0x0C, 0x00, 0x00, 0x00])
                            info_rc = st2.pack(">BBHIBBBB",
                                0x88, 0, arfcn if 'arfcn' in dir() else 100,
                                fn_rach, 60, 20, 0, 0)
                            resp_rc = hdr_rc + info_rc
                            msg_rc = st2.pack(">H", len(resp_rc)) + resp_rc
                            if client is not None:
                                try:
                                    client.sendall(msg_rc)
                                    stats["rx"] += 1
                                    print(f"  [bridge] → RACH_CONF", flush=True)
                                except (BrokenPipeError, OSError):
                                    pass

                            # Send Immediate Assignment on AGCH (chan_nr=0x90)
                            # Assign SDCCH/4 sub 0 on TS0: chan_nr = 0x20 (RSL_CHAN_SDCCH4_ACCH + sub0)
                            # GSM 04.08 9.1.18 Immediate Assignment
                            # chan_desc: chan_nr=0x20, h0: tsc=7, h=0, arfcn=100
                            sdcch_chan_nr = 0x20  # SDCCH/4 sub 0 TS0
                            tsc = 7
                            arfcn_val = arfcn if 'arfcn' in dir() else 100
                            # chan_desc byte 1: chan_nr
                            # chan_desc byte 2 (h0): tsc(3) | h(1)=0 | spare(2)=0 | arfcn_high(2)
                            cd_byte2 = (tsc << 5) | (0 << 4) | (0 << 2) | ((arfcn_val >> 8) & 0x03)
                            # chan_desc byte 3 (h0): arfcn_low
                            cd_byte3 = arfcn_val & 0xFF

                            # req_ref: ra, t1/t2/t3 from fn_rach
                            # fn = 51*((t3-t2+26)%26) + t3 + 26*51*t1
                            t1 = (fn_rach // (26*51)) % 32
                            t2 = fn_rach % 26
                            t3 = fn_rach % 51
                            rr_byte1 = ra
                            rr_byte2 = (t3 >> 3) | (t1 << 3)  # t3_high(3) | t1(5)
                            rr_byte3 = (t3 & 0x07) | (t2 << 3)  # t3_low(3) | t2(5) - WRONG ORDER
                            # Actually: byte2 = t1(5) | t3_high(3), byte3 = t2(5) | t3_low(3)
                            # Per struct: t3_high:3, t1:5 means t1 is high bits
                            rr_byte2 = ((t1 & 0x1F) << 3) | ((t3 >> 3) & 0x07)
                            rr_byte3 = ((t3 & 0x07) << 5) | (t2 & 0x1F)

                            imm_ass = bytes([
                                0x06,  # l2_plen (will be calculated)
                                0x06,  # PD=RR
                                0x3F,  # msg_type = Immediate Assignment
                                0x00,  # page_mode = normal
                                # chan_desc (3 bytes)
                                sdcch_chan_nr,
                                cd_byte2,
                                cd_byte3,
                                # req_ref (3 bytes)
                                rr_byte1, rr_byte2, rr_byte3,
                                # timing_advance
                                0x00,
                                # mob_alloc_len
                                0x00,
                                # no mobile allocation (non-hopping)
                            ])
                            # Fix l2_plen: length of L3 after l2_plen byte
                            l3_len = len(imm_ass) - 1  # exclude l2_plen itself
                            imm_ass = bytes([(l3_len << 2) | 0x01]) + imm_ass[1:]
                            imm_ass_padded = (imm_ass + bytes([0x2b] * 23))[:23]

                            import time as _time
                            _time.sleep(0.05)  # Small delay

                            hdr_ia = bytes([0x03, 0x00, 0x00, 0x00])
                            info_ia = st2.pack(">BBHIBBBB",
                                0x90, 0, arfcn_val, fn_rach + 4, 60, 20, 0, 0)
                            resp_ia = hdr_ia + info_ia + imm_ass_padded
                            msg_ia = st2.pack(">H", len(resp_ia)) + resp_ia
                            if client is not None:
                                try:
                                    client.sendall(msg_ia)
                                    stats["rx"] += 1
                                    print(f"  [bridge] → Immediate Assignment (SDCCH/4 TS0)", flush=True)
                                except (BrokenPipeError, OSError):
                                    pass
                            pass  # fall through to forward to firmware

                        # ---- Intercept DM_EST_REQ (0x05) ----
                        if len(payload) >= 4 and payload[0] == 0x05:
                            chan_nr_est = payload[4] if len(payload) > 4 else 0x20
                            print(f"  [bridge] DM_EST_REQ chan_nr=0x{chan_nr_est:02x} payload={payload[:16].hex()}", flush=True)
                            # No explicit confirmation needed - DM_EST is implicit
                            # The mobile will now send DATA_REQ with the LU request
                            # We need to forward DATA_REQ to handle LU
                            pass  # fall through to forward to firmware

                        # ---- Intercept DATA_REQ (0x06) ----
                        if len(payload) >= 4 and payload[0] == 0x06:
                            chan_nr_dr = payload[4] if len(payload) > 4 else 0
                            link_id_dr = payload[5] if len(payload) > 5 else 0
                            l3_data = payload[8:] if len(payload) > 8 else b''
                            print(f"  [bridge] DATA_REQ chan_nr=0x{chan_nr_dr:02x} link_id=0x{link_id_dr:02x} l3={l3_data[:20].hex()}", flush=True)
                            # Send DATA_CONF (0x0F) to confirm reception
                            hdr_dc = bytes([0x0F, 0x00, 0x00, 0x00])
                            info_dc = st2.pack(">BBHIBBBB",
                                chan_nr_dr, link_id_dr, arfcn if 'arfcn' in dir() else 100,
                                1557200, 60, 20, 0, 0)
                            resp_dc = hdr_dc + info_dc
                            msg_dc = st2.pack(">H", len(resp_dc)) + resp_dc
                            if client is not None:
                                try:
                                    client.sendall(msg_dc)
                                    stats["rx"] += 1
                                    print(f"  [bridge] -> DATA_CONF", flush=True)
                            # Detect SABM in DATA_REQ and respond with UA + LU Accept
                                except (BrokenPipeError, OSError):
                                    pass
                            if len(l3_data) >= 3 and l3_data[1] == 0x3F:
                                sabm_len = l3_data[2] >> 2
                                sabm_l3 = l3_data[3:3+sabm_len] if sabm_len > 0 else b''
                                print(f'  [bridge] ** SABM (len={sabm_len})', flush=True)
                                # Send UA
                                ua_frame = bytes([0x01, 0x73, l3_data[2]]) + sabm_l3
                                ua_padded = (ua_frame + bytes([0x2b] * 23))[:23]
                                hdr_ua = bytes([0x03, 0x00, 0x00, 0x00])
                                info_ua = st2.pack('>BBHIBBBB', 0x20, 0, arfcn if 'arfcn' in dir() else 100, 1557300, 60, 20, 0, 0)
                                msg_ua = st2.pack('>H', len(hdr_ua)+len(info_ua)+len(ua_padded)) + hdr_ua + info_ua + ua_padded
                                try:
                                    client.sendall(msg_ua)
                                    print('  [bridge] -> UA', flush=True)
                                except (BrokenPipeError, OSError):
                                    pass
                                # If LU Request (PD=5 MT=0x08), send LU Accept
                                if len(sabm_l3) >= 2 and (sabm_l3[0] & 0x0F) == 5 and sabm_l3[1] == 0x08:
                                    print('  [bridge] ** LU Request -> sending LU Accept', flush=True)
                                    import time as _t2; _t2.sleep(0.1)
                                    lu_l3 = bytes([0x05, 0x02, 0x00, 0xF1, 0x10, 0x00, 0x01])  # LU Accept LAI
                                    i_frame = bytes([0x03, 0x00, (len(lu_l3)<<2)|1]) + lu_l3
                                    i_padded = (i_frame + bytes([0x2b]*23))[:23]
                                    hdr_i = bytes([0x03, 0x00, 0x00, 0x00])
                                    info_i = st2.pack('>BBHIBBBB', 0x20, 0, arfcn if 'arfcn' in dir() else 100, 1557350, 60, 20, 0, 0)
                                    msg_i = st2.pack('>H', len(hdr_i)+len(info_i)+len(i_padded)) + hdr_i + info_i + i_padded
                                    try:
                                        client.sendall(msg_i)
                                        print('  [bridge] -> LU ACCEPT (I-frame)', flush=True)
                                    except (BrokenPipeError, OSError):
                                        pass
                        # ---- Intercept DM_REL_REQ (0x12) ----
                        if len(payload) >= 4 and payload[0] == 0x12:
                            print(f"  [bridge] DM_REL_REQ", flush=True)
                            pass  # fall through to forward

                        # ---- Intercept CCCH_MODE_REQ (0x10) ----
                        if len(payload) >= 4 and payload[0] == 0x10:
                            ccch_mode = payload[4] if len(payload) > 4 else 1
                            print(f"  [bridge] CCCH_MODE_REQ mode={ccch_mode} → CONF", flush=True)
                            resp_cc = bytes([0x11, 0x00, 0x00, 0x00, ccch_mode, 0x00, 0x00, 0x00])
                            msg_cc = st2.pack(">H", len(resp_cc)) + resp_cc
                            if client is not None:
                                try:
                                    client.sendall(msg_cc)
                                    stats["rx"] += 1
                                except (BrokenPipeError, OSError):
                                    pass

                            # After CCCH_MODE_CONF, send periodic idle/paging frames
                            # so the mobile doesn't lose sync and reset
                            import threading
                            def send_ccch_idle(cl, arfcn_val):
                                fn_counter = 1557053
                                frame_count = 0
                                # SI3
                                si3_data = bytes([
                                    0x49, 0x06, 0x1b, 0x00, 0x01,
                                    0x00, 0xF1, 0x10, 0x00, 0x01,
                                    0x49, 0x00, 0x00, 0x04,
                                    0x00, 0x00, 0x15, 0x00, 0x00,
                                    0x2b, 0x2b, 0x2b, 0x2b,
                                ])[:23]
                                # SI1
                                si1_data = bytes([
                                    0x55, 0x06, 0x19,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x15, 0x00, 0x00,
                                ])[:23]
                                # SI2
                                si2_data = bytes([
                                    0x59, 0x06, 0x1a,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0xFF,
                                    0x15, 0x00, 0x00,
                                ])[:23]
                                idle_l3 = bytes([0x01, 0x03, 0x01]) + bytes([0x2b] * 20)
                                si_rotation = [si1_data, si2_data, si3_data]
                                hdr = bytes([0x03, 0x00, 0x00, 0x00])
                                while cl is not None and running:
                                    si_idx = frame_count % 8
                                    if si_idx < 3:
                                        chan_nr = 0x80
                                        payload = si_rotation[si_idx]
                                    else:
                                        chan_nr = 0x90
                                        payload = idle_l3
                                    info = st2.pack(">BBHIBBBB", chan_nr, 0, arfcn_val, fn_counter, 60, 20, 0, 0)
                                    msg = st2.pack(">H", len(hdr) + len(info) + len(payload)) + hdr + info + payload
                                    try:
                                        cl.sendall(msg)
                                    except (BrokenPipeError, OSError):
                                        break
                                    frame_count += 1
                                    fn_counter += 51
                                    if fn_counter > 2715647:
                                        fn_counter = 0
                                    time.sleep(0.235)
                            if not hasattr(send_ccch_idle, '_started'):
                                send_ccch_idle._started = True
                                t = threading.Thread(target=send_ccch_idle, args=(client, arfcn), daemon=True)
                                t.start()
                                print("  [bridge] Started CCCH idle frame sender", flush=True)
                            pass  # fall through to forward

                        # ---- Intercept ALL PM_REQ ----
                        # Handle PM entirely in the bridge to avoid
                        # firmware msgb pool exhaustion from printf.
                        # PM_REQ = msg_type 0x08.
                        #   [8-9] band_arfcn_from  [10-11] band_arfcn_to
                        # Return -62 dBm (rxlev=48=0x30) for all ARFCNs.
                        # Don't forward to firmware.
                        if (len(payload) >= 12 and payload[0] == 0x08):
                            arfcn_from = (payload[8] << 8) | payload[9]
                            arfcn_to = (payload[10] << 8) | payload[11]
                            # Only target ARFCN gets strong signal.
                            # Matches the cfile capture at ARFCN 1022 (E-GSM).
                            PM_STRONG = 0x0030   # rxlev 48 = -62 dBm
                            PM_NOISE  = 0x0000   # rxlev 0  = -110 dBm
                            TARGET_ARFCN = 100  # E-GSM, matches cfile
                            strong_count = 0
                            print(f"  [bridge] PM_REQ ARFCN {arfcn_from}-{arfcn_to}",
                                  flush=True)
                            entries = []
                            for arfcn in range(arfcn_from, arfcn_to + 1):
                                if arfcn == TARGET_ARFCN:
                                    # 4 bytes: arfcn(BE) + pm1(byte) + pm2(byte)
                                    entries.append(struct.pack(">HBB", arfcn, PM_STRONG, PM_STRONG))
                                    strong_count += 1
                                else:
                                    entries.append(struct.pack(">HBB", arfcn, PM_NOISE, PM_NOISE))
                            print(f"  [bridge] → {len(entries)} entries "
                                  f"({strong_count} strong)", flush=True)
                            batch_size = 50
                            for batch_start in range(0, len(entries), batch_size):
                                batch = entries[batch_start:batch_start + batch_size]
                                is_last = (batch_start + batch_size >= len(entries))
                                hdr = bytes([0x09, 0x01 if is_last else 0x00,
                                             0x00, 0x00])
                                pm_data = hdr + b"".join(batch)
                                msg = struct.pack(">H", len(pm_data)) + pm_data
                                if client is not None:
                                    try:
                                        client.sendall(msg)
                                    except (BrokenPipeError, OSError):
                                        pass
                                stats["rx"] += 1
                            print(f"  [bridge] Sent {len(entries)} PM_CONF entries",
                                  flush=True)
                            continue  # Don't forward to firmware

                        frame = sercomm_wrap(DLCI_L1CTL, payload)
                        try:
                            os.write(pty_fd, frame)
                        except OSError as e:
                            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                raise
                        if stats["tx"] <= 5 or stats["tx"] % 100 == 0:
                            print(f"  [mobile→fw] L1CTL #{stats['tx']} "
                                  f"len={len(payload)} {hexdump(payload)}",
                                  flush=True)

            # ---- firmware → mobile (sercomm → L1CTL / console) ----
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
                        # ---- L1CTL message → forward to mobile ----
                        stats["rx"] += 1
                        if client is not None:
                            msg = struct.pack(">H", len(payload)) + payload
                            try:
                                client.sendall(msg)
                            except (BrokenPipeError, OSError):
                                print("l1ctl-bridge: mobile send error, "
                                      "disconnecting", flush=True)
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None

                        if stats["rx"] <= 5 or stats["rx"] % 100 == 0:
                            print(f"  [fw→mobile] L1CTL #{stats['rx']} "
                                  f"len={len(payload)} {hexdump(payload)}",
                                  flush=True)

                    elif dlci == DLCI_CONSOLE:
                        # ---- Console debug → stderr ----
                        stats["console"] += 1
                        # Strip leading 0x03 marker if present
                        text_bytes = payload
                        if text_bytes and text_bytes[0] == 0x03:
                            text_bytes = text_bytes[1:]
                        try:
                            text = text_bytes.decode("ascii", errors="replace")
                            sys.stderr.write(f"[console] {text}")
                            sys.stderr.flush()
                        except Exception:
                            pass

                    elif dlci == DLCI_LOADER:
                        # Loader frames after jump — ignore
                        pass

                    else:
                        # Unknown DLCI
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
