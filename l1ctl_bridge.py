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

L1CTL_SOCK   = "/tmp/osmocom_l2"


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
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <pty-path>", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]

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
                            TARGET_ARFCN = 1022  # E-GSM, matches cfile
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
