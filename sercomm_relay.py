#!/usr/bin/env python3
"""
Sercomm relay with HDLC control byte fix.

Frame format on wire: [FLAG] [DLCI] [CTRL=0x03] [payload (escaped)] [FLAG]
Socket format: [2-byte BE length] [payload]

The CTRL byte (0x03 = UI unnumbered information) is MANDATORY.
Without it, the loader consumes the first payload byte as CTRL.
"""

import os
import sys
import struct
import socket
import select
import signal
import errno
import termios
import fcntl

DLCI_TX = 9      # Host → loader (commands)
DLCI_RX = 10     # Loader → host (responses)
CTRL = 0x03      # HDLC UI frame control byte

FLAG = 0x7E
ESCAPE = 0x7D
ESCAPE_XOR = 0x20
SOCK_PATH = "/tmp/osmocom_loader"


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data)


def set_raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def set_nonblock(fd: int) -> None:
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


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
        b = data[i]
        if b == ESCAPE:
            i += 1
            if i >= len(data):
                break
            out.append(data[i] ^ ESCAPE_XOR)
        else:
            out.append(b)
        i += 1
    return bytes(out)


def sercomm_wrap(dlci: int, payload: bytes) -> bytes:
    """Wrap payload in HDLC sercomm frame: [FLAG][DLCI][CTRL][payload][FLAG]"""
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])


class SercommParser:
    """Parse HDLC sercomm frames: [FLAG][DLCI][CTRL][payload][FLAG]"""

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
                # Need at least DLCI + CTRL
                continue

            dlci = frame[0]
            ctrl = frame[1]
            payload = frame[2:]  # Skip CTRL byte
            frames.append((dlci, payload))

        return frames


class LengthPrefixReader:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        msgs = []

        while True:
            if len(self.buf) < 2:
                break
            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break
            payload = bytes(self.buf[2:2 + msglen])
            del self.buf[:2 + msglen]
            msgs.append(payload)

        return msgs


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <pty-path>", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]

    try:
        os.unlink(SOCK_PATH)
    except FileNotFoundError:
        pass

    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    srv.listen(1)
    srv.setblocking(False)

    print(
        f"sercomm-relay: listening on {SOCK_PATH}, PTY={pty_path}",
        flush=True,
    )
    print(
        f"sercomm-relay: TX DLCI={DLCI_TX}, RX DLCI={DLCI_RX}, CTRL=0x{CTRL:02x}",
        flush=True,
    )

    parser = SercommParser()
    lp_reader = LengthPrefixReader()
    client = None
    running = True

    def shutdown(sig, frame):
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

            if srv in readable:
                conn, _ = srv.accept()
                conn.setblocking(False)
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass
                client = conn
                lp_reader = LengthPrefixReader()
                print("sercomm-relay: client connected", flush=True)

            if client is not None and client in readable:
                try:
                    raw = client.recv(4096)
                except BlockingIOError:
                    raw = b""

                if not raw:
                    print("sercomm-relay: client disconnected", flush=True)
                    try:
                        client.close()
                    except OSError:
                        pass
                    client = None
                else:
                    print(f"  [sock raw] {hexdump(raw)}", flush=True)
                    for payload in lp_reader.feed(raw):
                        print(f"  [sock msg] payload={hexdump(payload)}", flush=True)

                        frame = bytes([FLAG, DLCI_TX]) + sercomm_escape(payload) + bytes([FLAG])
                                                                                                
                        print(f"  [->PTY] {hexdump(frame)}", flush=True)
                        try:
                            os.write(pty_fd, frame)
                        except OSError as e:
                            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                raise
            if pty_fd in readable:
                try:
                    raw = os.read(pty_fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        raw = b""
                    else:
                        raise

                if not raw:
                    print("sercomm-relay: PTY closed", flush=True)
                    break

                print(f"  [PTY raw] {hexdump(raw)}", flush=True)
                for dlci, payload in parser.feed(raw):
                    print(f"  [PTY frame] dlci={dlci} payload={hexdump(payload)}", flush=True)

                    if dlci != DLCI_RX or client is None:
                        continue

                    msg = struct.pack(">H", len(payload)) + payload
                    print(f"  [->sock] {hexdump(msg)}", flush=True)
                    try:
                        client.sendall(msg)
                    except (BrokenPipeError, OSError):
                        try:
                            client.close()
                        except OSError:
                            pass
                        client = None
    finally:
        try:
            if client is not None:
                client.close()
        except OSError:
            pass
        try:
            srv.close()
        except OSError:
            pass
        try:
            os.close(pty_fd)
        except OSError:
            pass
        try:
            os.unlink(SOCK_PATH)
        except OSError:
            pass
        print("sercomm-relay: done", flush=True)


if __name__ == "__main__":
    main()
