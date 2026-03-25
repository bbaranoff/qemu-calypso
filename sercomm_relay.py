#!/usr/bin/env python3
"""
Sercomm relay for Osmocom loader.

Socket side:
    [2-byte big-endian length][payload]

UART/PTTY side:
    [0x7e][dlci][ctrl=0x03][escaped payload][0x7e]

This relay keeps osmoload untouched, but splits large LOADER_MEM_WRITE
messages into smaller sub-writes on the wire, then synthesizes one final
reply back to osmoload.

Why:
- ping works already
- large memload frames fail with bad CRC on the loader side
- splitting at protocol level is safer than touching osmoload
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
from typing import Optional, Dict, List, Tuple, Any

# Sercomm framing constants
DLCI_TX = 9
DLCI_RX = 9
CTRL = 0x03
FLAG = 0x7E
ESCAPE = 0x7D
ESCAPE_XOR = 0x20

# Socket and protocol paths/constants
SOCK_PATH = "/tmp/osmocom_loader"

# Loader protocol commands
LOADER_PING = 0x01
LOADER_MEM_WRITE = 0x08

# Chunking configuration
# Keep this comfortably below the original 248-byte socket payload size.
# 64 data bytes per sub-write is a conservative starting point.
MEMWRITE_CHUNK = 64


def hexdump(data: bytes) -> str:
    """Convert binary data to a hex dump string."""
    return " ".join(f"{b:02x}" for b in data)


def set_raw(fd: int) -> None:
    """Configure terminal fd for raw 8N1 mode at 115200 baud."""
    attrs = termios.tcgetattr(fd)

    # iflag: disable all input processing
    attrs[0] = 0

    # oflag: disable all output processing
    attrs[1] = 0

    # cflag: 8N1, local, read enabled
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD

    # lflag: disable canonical mode, echoes, signals
    attrs[3] = 0

    # ispeed / ospeed: 115200 baud
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    # cc: VMIN=1 (block until 1 byte), VTIME=0 (no timeout)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def set_nonblock(fd: int) -> None:
    """Set file descriptor to non-blocking mode."""
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


def sercomm_escape(data: bytes) -> bytes:
    """Apply Sercomm HDLC-style escaping to data."""
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def sercomm_unescape(data: bytes) -> bytes:
    """Remove Sercomm HDLC-style escaping from data."""
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
    """Wrap payload in a Sercomm frame with DLCI and control byte."""
    inner = bytes([dlci, CTRL]) + payload
    return bytes([FLAG]) + sercomm_escape(inner) + bytes([FLAG])


def osmo_crc16(data: bytes) -> int:
    """
    CRC-16/CCITT as used by Osmocom loader code via osmo_crc16(0, ...).
    
    Init = 0x0000, poly = 0x1021, no xorout.
    """
    crc = 0x0000
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class SercommParser:
    """Parse Sercomm HDLC frames from a byte stream."""
    
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        """Feed data into parser and return list of (dlci, payload) tuples."""
        self.buf.extend(data)
        frames: List[Tuple[int, bytes]] = []

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
            del self.buf[: end + 1]

            if not raw:
                continue

            frame = sercomm_unescape(raw)
            if len(frame) < 2:
                continue

            dlci = frame[0]
            ctrl = frame[1]
            payload = frame[2:]

            # Keep permissive; log only if needed.
            _ = ctrl
            frames.append((dlci, payload))

        return frames


class LengthPrefixReader:
    """Parse length-prefixed messages (2-byte big-endian length)."""
    
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[bytes]:
        """Feed data into parser and return list of complete message payloads."""
        self.buf.extend(data)
        msgs: List[bytes] = []

        while True:
            if len(self.buf) < 2:
                break

            msglen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + msglen:
                break

            payload = bytes(self.buf[2 : 2 + msglen])
            del self.buf[: 2 + msglen]
            msgs.append(payload)

        return msgs


def split_memwrite(
    payload: bytes, chunk_size: int = MEMWRITE_CHUNK
) -> Optional[Dict[str, Any]]:
    """
    Split a LOADER_MEM_WRITE payload into smaller chunks.
    
    Input payload format from osmoload:
        [cmd=0x08][nbytes][crc16][address][data...]

    Args:
        payload: The original loader message payload.
        chunk_size: Maximum data bytes per chunk.
        
    Returns:
        None for non-MEM_WRITE payloads.
        A dict describing split chunks otherwise.
    """
    if len(payload) < 8:
        return None

    cmd = payload[0]
    if cmd != LOADER_MEM_WRITE:
        return None

    total_nbytes = payload[1]
    if len(payload) < 8 + total_nbytes:
        return None

    orig_crc = struct.unpack(">H", payload[2:4])[0]
    base_addr = struct.unpack(">I", payload[4:8])[0]
    data = payload[8 : 8 + total_nbytes]

    chunks: List[bytes] = []
    off = 0
    while off < len(data):
        chunk = data[off : off + chunk_size]
        chunk_crc = osmo_crc16(chunk)

        sub = bytearray()
        sub.append(LOADER_MEM_WRITE)
        sub.append(len(chunk))
        sub.extend(struct.pack(">H", chunk_crc))
        sub.extend(struct.pack(">I", base_addr + off))
        sub.extend(chunk)

        chunks.append(bytes(sub))
        off += len(chunk)

    return {
        "original_cmd": cmd,
        "original_nbytes": total_nbytes,
        "original_crc": orig_crc,
        "original_addr": base_addr,
        "chunks": chunks,
    }


def make_socket_msg(payload: bytes) -> bytes:
    """Wrap payload with 2-byte big-endian length prefix."""
    return struct.pack(">H", len(payload)) + payload


def main() -> None:
    """Main entry point for the Sercomm relay."""
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <pty-path>", file=sys.stderr)
        sys.exit(1)

    pty_path = sys.argv[1]

    # Clean up stale socket file
    try:
        os.unlink(SOCK_PATH)
    except FileNotFoundError:
        pass

    # Open and configure PTY
    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)

    # Create Unix domain socket server
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    srv.listen(1)
    srv.setblocking(False)

    print(f"sercomm-relay: listening on {SOCK_PATH}, PTY={pty_path}", flush=True)
    print(
        f"sercomm-relay: TX DLCI={DLCI_TX}, RX DLCI={DLCI_RX}, CTRL=0x{CTRL:02x}",
        flush=True,
    )

    parser = SercommParser()
    lp_reader = LengthPrefixReader()

    client: Optional[socket.socket] = None
    running = True

    # For split MEM_WRITE aggregation.
    pending_reply: Optional[Dict[str, Any]] = None

    def shutdown(_sig: int, _frame: Any) -> None:
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

            # Accept new client connections
            if srv in readable:
                conn, _ = srv.accept()
                conn.setblocking(False)

                # Close existing client if any
                if client is not None:
                    try:
                        client.close()
                    except OSError:
                        pass

                client = conn
                lp_reader = LengthPrefixReader()
                pending_reply = None
                print("sercomm-relay: client connected", flush=True)

            # Handle incoming socket messages
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
                    pending_reply = None
                else:
                    print(f"  [sock raw] {hexdump(raw)}", flush=True)

                    for payload in lp_reader.feed(raw):
                        print(f"  [sock msg] payload={hexdump(payload)}", flush=True)

                        split = split_memwrite(payload, chunk_size=MEMWRITE_CHUNK)

                        if split is None:
                            # Normal passthrough
                            frame = sercomm_wrap(DLCI_TX, payload)
                            print(f"  [->PTY] {hexdump(frame)}", flush=True)
                            try:
                                os.write(pty_fd, frame)
                            except OSError as e:
                                if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                    raise
                        else:
                            # Split MEM_WRITE into chunks
                            pending_reply = {
                                "cmd": split["original_cmd"],
                                "nbytes": split["original_nbytes"],
                                "crc": split["original_crc"],
                                "addr": split["original_addr"],
                                "remaining": len(split["chunks"]),
                            }

                            print(
                                f"  [memwrite split] "
                                f"addr=0x{pending_reply['addr']:08x} "
                                f"total={pending_reply['nbytes']} "
                                f"chunks={pending_reply['remaining']}",
                                flush=True,
                            )

                            for idx, sub in enumerate(split["chunks"], start=1):
                                frame = sercomm_wrap(DLCI_TX, sub)
                                print(
                                    f"  [->PTY sub {idx}/{len(split['chunks'])}] "
                                    f"{hexdump(frame)}",
                                    flush=True,
                                )
                                try:
                                    os.write(pty_fd, frame)
                                except OSError as e:
                                    if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                                        raise

            # Handle incoming PTY data
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

                    # Aggregate split MEM_WRITE replies and synthesize one final reply
                    # matching osmoload's original expectation.
                    if (
                        pending_reply is not None
                        and len(payload) >= 8
                        and payload[0] == LOADER_MEM_WRITE
                    ):
                        pending_reply["remaining"] -= 1
                        print(
                            f"  [memwrite ack] remaining={pending_reply['remaining']}",
                            flush=True,
                        )

                        if pending_reply["remaining"] == 0:
                            # All chunks acknowledged - send synthesized response
                            synth = bytearray()
                            synth.append(pending_reply["cmd"])
                            synth.append(pending_reply["nbytes"])
                            synth.extend(struct.pack(">H", pending_reply["crc"]))
                            synth.extend(struct.pack(">I", pending_reply["addr"]))

                            msg = make_socket_msg(bytes(synth))
                            print(f"  [->sock synth] {hexdump(msg)}", flush=True)
                            try:
                                client.sendall(msg)
                            except (BrokenPipeError, OSError):
                                try:
                                    client.close()
                                except OSError:
                                    pass
                                client = None

                            pending_reply = None

                        continue

                    # Normal passthrough reply
                    msg = make_socket_msg(payload)
                    print(f"  [->sock] {hexdump(msg)}", flush=True)
                    try:
                        client.sendall(msg)
                    except (BrokenPipeError, OSError):
                        try:
                            client.close()
                        except OSError:
                            pass
                        client = None
                        pending_reply = None

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
