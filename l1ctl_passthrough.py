#!/usr/bin/env python3
"""
l1ctl_passthrough.py — Relay between mobile (layer23) and QEMU firmware.

Synthesizes RESET_CONF/RESET_IND (firmware does not implement L1CTL reset).
Everything else is pure relay to/from the firmware.

Usage:
  python3 l1ctl_passthrough.py /dev/pts/X [/tmp/osmocom_l2_ms]
"""

import errno, fcntl, os, select, signal, socket, struct, sys, termios, time

DLCI_L1CTL   = 5
DLCI_DEBUG   = 4
DLCI_CONSOLE = 10
FLAG         = 0x7E
ESCAPE       = 0x7D
ESCAPE_XOR   = 0x20
L1CTL_SOCK   = "/tmp/osmocom_l2"

L1CTL_RESET_IND  = 7
L1CTL_RESET_REQ  = 8
L1CTL_RESET_CONF = 9
L1CTL_PM_REQ     = 11
L1CTL_PM_CONF    = 12

L1CTL_NAMES = {
    1:"FBSB_REQ", 2:"FBSB_CONF", 3:"DATA_IND", 4:"RACH_REQ", 5:"RACH_CONF",
    6:"DATA_REQ", 7:"RESET_IND", 8:"RESET_REQ", 9:"RESET_CONF", 10:"DATA_CONF",
    11:"PM_REQ", 12:"PM_CONF", 13:"ECHO_REQ", 14:"ECHO_CONF",
    18:"CCCH_MODE_REQ", 19:"CCCH_MODE_CONF",
    20:"DM_EST_REQ", 21:"DM_FREQ_REQ", 22:"DM_REL_REQ",
    29:"TCH_MODE_REQ", 30:"TCH_MODE_CONF",
    33:"TRAFFIC_REQ", 34:"TRAFFIC_CONF", 35:"TRAFFIC_IND",
}

def set_raw(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (attrs[2] & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) | termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def set_nonblock(fd):
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)

def sercomm_escape(data):
    out = bytearray()
    for b in data:
        if b in (FLAG, ESCAPE):
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)

def sercomm_wrap(dlci, payload):
    return bytes(bytearray([FLAG]) + sercomm_escape(bytes([dlci, 0x03]) + payload) + bytearray([FLAG]))

def l1ctl_send(client, msg_type, payload=b""):
    hdr = struct.pack("BBxx", msg_type, 0)
    msg = hdr + payload
    frame = struct.pack(">H", len(msg)) + msg
    try:
        client.sendall(frame)
    except Exception:
        pass

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, data):
        frames = []
        for b in data:
            if b == FLAG:
                if self.in_frame and len(self.buf) >= 2:
                    frames.append((self.buf[0], bytes(self.buf[2:])))
                self.buf.clear()
                self.in_frame = True
                self.escape = False
            elif self.in_frame:
                if self.escape:
                    self.buf.append(b ^ ESCAPE_XOR)
                    self.escape = False
                elif b == ESCAPE:
                    self.escape = True
                else:
                    self.buf.append(b)
        return frames

class LengthPrefixReader:
    def __init__(self):
        self.buf = b""

    def feed(self, data):
        self.buf += data
        msgs = []
        while len(self.buf) >= 2:
            mlen = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + mlen:
                break
            msgs.append(self.buf[2:2 + mlen])
            self.buf = self.buf[2 + mlen:]
        return msgs

def main():
    if len(sys.argv) < 2:
        print("Usage: %s /dev/pts/X [socket_path]" % sys.argv[0])
        sys.exit(1)

    pty_path = sys.argv[1]
    sock_path = sys.argv[2] if len(sys.argv) > 2 else L1CTL_SOCK

    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
    set_raw(pty_fd)
    set_nonblock(pty_fd)
    print("l1ctl-passthrough: PTY=%s" % pty_path, flush=True)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(sock_path)
    srv.listen(1)
    srv.setblocking(False)
    print("l1ctl-passthrough: listening on %s" % sock_path, flush=True)

    client = None
    parser = SercommParser()
    reader = LengthPrefixReader()
    tx_count = 0
    rx_count = 0
    running = True

    def shutdown(_s, _f):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while running:
        fds = [pty_fd, srv]
        if client:
            fds.append(client)
        try:
            readable, _, _ = select.select(fds, [], [], 0.5)
        except (ValueError, OSError):
            if client:
                client.close()
                client = None
            continue

        for fd in readable:
            # New mobile connection
            if fd is srv:
                if client:
                    client.close()
                client, _ = srv.accept()
                client.setblocking(False)
                reader = LengthPrefixReader()
                print("l1ctl-passthrough: mobile connected", flush=True)
                # Send RESET_IND(BOOT) on connect — firmware doesn't send it
                l1ctl_send(client, L1CTL_RESET_IND, struct.pack("Bxxx", 0))
                print("  [synth] -> RESET_IND(BOOT)", flush=True)
                continue

            # mobile -> firmware
            if fd is client:
                try:
                    chunk = client.recv(4096)
                except Exception:
                    chunk = b""
                if not chunk:
                    print("l1ctl-passthrough: mobile disconnected", flush=True)
                    client.close()
                    client = None
                    continue

                for msg in reader.feed(chunk):
                    if len(msg) < 1:
                        continue
                    msg_type = msg[0]
                    name = L1CTL_NAMES.get(msg_type, "0x%02x" % msg_type)
                    print("  [mobile->] %s len=%d" % (name, len(msg)), flush=True)

                    # Handle ECHO locally
                    if msg_type == 13:  # ECHO_REQ
                        l1ctl_send(client, 14, msg[4:])  # ECHO_CONF
                        continue

                    # Synthesize RESET_CONF
                    if msg_type == L1CTL_RESET_REQ:
                        reset_type = msg[4] if len(msg) > 4 else 1
                        l1ctl_send(client, L1CTL_RESET_CONF, struct.pack("Bxxx", reset_type))
                        print("  [synth] RESET_REQ type=%d -> RESET_CONF" % reset_type, flush=True)
                        # Don't forward RESET_REQ to firmware — it doesn't handle it
                        continue

                    # Synthesize PM_CONF (firmware doesn't handle L1CTL PM_REQ)
                    if msg_type == L1CTL_PM_REQ and len(msg) >= 12:
                        arfcn_from = (msg[8] << 8) | msg[9]
                        arfcn_to = (msg[10] << 8) | msg[11]
                        TARGET_ARFCN = 100
                        # Send in small batches (max 10 entries per PM_CONF)
                        batch = b""
                        count = 0
                        for a in range(arfcn_from, arfcn_to + 1):
                            rxlev = 48 if a == TARGET_ARFCN else 0
                            batch += struct.pack(">HBB", a, rxlev, rxlev)
                            count += 1
                            if count >= 10:
                                l1ctl_send(client, L1CTL_PM_CONF, batch)
                                batch = b""
                                count = 0
                        if batch:
                            l1ctl_send(client, L1CTL_PM_CONF, batch)
                        print("  [synth] PM_REQ %d-%d -> PM_CONF" % (arfcn_from, arfcn_to), flush=True)
                        continue

                    # Relay everything else to firmware
                    frame = sercomm_wrap(DLCI_L1CTL, msg)
                    try:
                        os.write(pty_fd, frame)
                    except Exception:
                        pass
                    tx_count += 1
                    if tx_count <= 50 or tx_count % 100 == 0:
                        hexd = " ".join("%02x" % b for b in msg[:16])
                        print("  [mobile->fw] #%d %s len=%d [%s]" % (tx_count, name, len(msg), hexd), flush=True)
                continue

            # firmware -> mobile
            if fd == pty_fd:
                try:
                    chunk = os.read(pty_fd, 4096)
                except Exception:
                    continue
                if not chunk:
                    continue

                for dlci, payload in parser.feed(chunk):
                    if dlci == DLCI_L1CTL and client and payload and len(payload) >= 4:
                        # Validate: L1CTL messages are at most ~200 bytes
                        if len(payload) > 512:
                            print("  [fw->mobile] SKIP oversized %d bytes" % len(payload), flush=True)
                            continue
                        # Skip ECHO_CONF from firmware — mobile doesn't expect it back
                        if payload[0] == 14:  # ECHO_CONF
                            continue
                        frame = struct.pack(">H", len(payload)) + payload
                        try:
                            client.sendall(frame)
                        except Exception:
                            pass
                        rx_count += 1
                        name = L1CTL_NAMES.get(payload[0], "0x%02x" % payload[0]) if payload else "?"
                        if rx_count <= 50 or rx_count % 100 == 0:
                            hexd = " ".join("%02x" % b for b in payload[:16])
                            print("  [fw->mobile] #%d %s len=%d [%s]" % (rx_count, name, len(payload), hexd), flush=True)
                    elif dlci in (DLCI_CONSOLE, DLCI_DEBUG):
                        try:
                            sys.stderr.buffer.write(payload)
                            sys.stderr.flush()
                        except Exception:
                            pass
                continue

    print("l1ctl-passthrough: done (tx=%d rx=%d)" % (tx_count, rx_count), flush=True)

if __name__ == "__main__":
    main()
