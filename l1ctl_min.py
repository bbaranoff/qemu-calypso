#!/usr/bin/env python3
"""
l1ctl_min.py — Minimal L1CTL bridge: QEMU firmware ↔ mobile
Synth: RESET_REQ, PM_REQ. Everything else forwarded to firmware.
"""

import errno, fcntl, os, re, select, signal, socket, struct, subprocess, sys, termios, time

# Sercomm
FLAG = 0x7E; ESCAPE = 0x7D; ESCAPE_XOR = 0x20
DLCI_L1CTL = 5; CTRL = 0x03

# L1CTL message types
MSG_RESET_REQ  = 0x0D; MSG_RESET_CONF = 0x0E
MSG_PM_REQ = 0x08; MSG_PM_CONF = 0x09

TARGET_ARFCN = int(os.environ.get("TARGET_ARFCN", "514"))


def sercomm_wrap(dlci, payload):
    frame = bytes([dlci, CTRL, len(payload) >> 8, len(payload) & 0xFF]) + payload
    out = bytearray([FLAG])
    for b in frame:
        if b in (FLAG, ESCAPE):
            out += bytes([ESCAPE, b ^ ESCAPE_XOR])
        else:
            out.append(b)
    out.append(FLAG)
    return bytes(out)


class SercommParser:
    def __init__(self):
        self.buf = bytearray()
        self.in_frame = False
        self.escape = False

    def feed(self, data):
        frames = []
        for b in data:
            if b == FLAG:
                if self.in_frame and len(self.buf) >= 4:
                    dlci = self.buf[0]
                    plen = (self.buf[2] << 8) | self.buf[3]
                    payload = bytes(self.buf[4:4+plen])
                    frames.append((dlci, payload))
                self.buf = bytearray()
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


def l1ctl_send(sock, msg_type, payload=b""):
    data = bytes([msg_type, 0, 0, 0]) + payload
    msg = struct.pack(">H", len(data)) + data
    sock.sendall(msg)


def launch_qemu(kernel, qemu_bin, mon_sock, env_extra=None):
    try: os.unlink(mon_sock)
    except FileNotFoundError: pass

    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)

    cmd = [qemu_bin, "-M", "calypso", "-cpu", "arm946", "-display", "none",
           "-serial", "pty", "-serial", "pty",
           "-monitor", f"unix:{mon_sock},server,nowait", "-kernel", kernel]
    print(f"l1ctl-min: QEMU: {' '.join(cmd)}", flush=True)

    log_fd = open("/tmp/qemu-min.log", "w")
    proc = subprocess.Popen(cmd, stdout=log_fd, stderr=subprocess.STDOUT, env=env)

    # Detect PTY from log
    pty_path = None
    for _ in range(100):
        time.sleep(0.1)
        with open("/tmp/qemu-min.log") as f:
            for line in f:
                m = re.search(r'char device redirected to (/dev/pts/\d+)', line)
                if m:
                    pty_path = m.group(1)
                    break
        if pty_path: break

    if not pty_path:
        print("l1ctl-min: ERROR: no PTY found", flush=True)
        proc.kill()
        sys.exit(1)

    print(f"l1ctl-min: PTY={pty_path}", flush=True)

    # Send 'cont' to QEMU monitor
    time.sleep(1)
    try:
        import subprocess as sp
        sp.run(["socat", "-", f"UNIX-CONNECT:{mon_sock}"],
               input=b"cont\n", timeout=3, capture_output=True)
        print("l1ctl-min: sent 'cont'", flush=True)
    except Exception:
        pass

    # Open PTY
    fd = os.open(pty_path, os.O_RDWR | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0; attrs[1] = 0; attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    return fd, proc


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("-k", "--kernel", required=True)
    p.add_argument("-s", "--socket", default="/tmp/osmocom_l2_1")
    p.add_argument("--qemu-bin", default="/opt/GSM/qemu/build/qemu-system-arm")
    p.add_argument("--env", action="append", default=[])
    args = p.parse_args()

    env_extra = {}
    for e in args.env:
        k, v = e.split("=", 1)
        env_extra[k] = v

    # POWERON to fake_trx
    trxc_port = int(os.environ.get("CALYPSO_TRX_PORT", "0"))
    if trxc_port > 0:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest = ("127.0.0.1", trxc_port + 1)
        if TARGET_ARFCN >= 512 and TARGET_ARFCN <= 885:
            dl = 1805200 + (TARGET_ARFCN - 512) * 200
            ul = 1710200 + (TARGET_ARFCN - 512) * 200
        else:
            dl = 935000 + TARGET_ARFCN * 200
            ul = 890000 + TARGET_ARFCN * 200
        for cmd in [f"CMD RXTUNE {dl}", f"CMD TXTUNE {ul}", "CMD POWERON"]:
            s.sendto(cmd.encode() + b'\0', dest)
            print(f"l1ctl-min: → fake_trx: {cmd}", flush=True)
        time.sleep(0.5)
        s.close()

    # Launch QEMU
    pty_fd, qemu_proc = launch_qemu(args.kernel, args.qemu_bin,
                                     "/tmp/qemu-calypso-mon-min.sock", env_extra)
    time.sleep(3)

    # L1CTL server socket
    sock_path = args.socket
    try: os.unlink(sock_path)
    except FileNotFoundError: pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(sock_path)
    srv.listen(1)
    srv.setblocking(False)
    print(f"l1ctl-min: listening on {sock_path}", flush=True)

    parser = SercommParser()
    client = None
    running = True

    def shutdown(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    while running:
        rlist = [pty_fd, srv]
        if client:
            rlist.append(client)
        try:
            readable, _, _ = select.select(rlist, [], [], 0.1)
        except (ValueError, OSError):
            break

        # Accept new client
        if srv in readable:
            conn, _ = srv.accept()
            conn.setblocking(False)
            if client:
                try: client.close()
                except: pass
            client = conn
            print("l1ctl-min: client connected", flush=True)

        # PTY → firmware sends L1CTL to mobile
        if pty_fd in readable:
            try:
                data = os.read(pty_fd, 4096)
            except OSError:
                break
            if not data:
                break
            for dlci, payload in parser.feed(data):
                if dlci == DLCI_L1CTL and client and len(payload) > 0:
                    msg_type = payload[0]
                    msg = struct.pack(">H", len(payload)) + payload
                    try:
                        client.sendall(msg)
                        print(f"  [fw→mobile] type=0x{msg_type:02x} len={len(payload)}", flush=True)
                    except (BrokenPipeError, OSError):
                        client = None

        # Mobile → L1CTL to firmware
        if client and client in readable:
            try:
                data = client.recv(4096)
            except (ConnectionResetError, OSError):
                data = b""
            if not data:
                print("l1ctl-min: client disconnected", flush=True)
                client = None
                continue

            # Parse L1CTL: 2-byte length + payload
            while len(data) >= 2:
                plen = struct.unpack(">H", data[:2])[0]
                if len(data) < 2 + plen:
                    break
                payload = data[2:2+plen]
                data = data[2+plen:]
                msg_type = payload[0] if payload else 0

                # Synth RESET_REQ
                if msg_type == MSG_RESET_REQ:
                    rt = payload[4] if len(payload) > 4 else 1
                    l1ctl_send(client, MSG_RESET_CONF, struct.pack("Bxxx", rt))
                    print(f"  [synth] RESET type={rt}", flush=True)
                    continue

                # Synth PM_REQ
                if msg_type == MSG_PM_REQ and len(payload) >= 12:
                    a_from = (payload[8] << 8) | payload[9]
                    a_to = (payload[10] << 8) | payload[11]
                    entries = []
                    for a in range(a_from, a_to + 1):
                        rxlev = 0x30 if a == TARGET_ARFCN else 0x00
                        entries.append(struct.pack(">HBB", a, rxlev, rxlev))
                    hdr = bytes([MSG_PM_CONF, 0x01, 0x00, 0x00])
                    pm_data = hdr + b"".join(entries)
                    msg = struct.pack(">H", len(pm_data)) + pm_data
                    client.sendall(msg)
                    print(f"  [synth] PM {a_from}-{a_to}", flush=True)
                    continue

                # Forward everything else to firmware
                frame = sercomm_wrap(DLCI_L1CTL, payload)
                try:
                    os.write(pty_fd, frame)
                    print(f"  [mobile→fw] type=0x{msg_type:02x} len={len(payload)}", flush=True)
                except OSError:
                    pass

    print("l1ctl-min: done", flush=True)
    qemu_proc.terminate()


if __name__ == "__main__":
    main()
