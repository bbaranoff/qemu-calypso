#!/usr/bin/env python3
"""
bridge.py — BTS TRX ↔ QEMU UART sercomm bridge
BTS side:  faketrx clone (CLK/TRXC/TRXD on 5700-5702)
MS side:   UART PTY (sercomm) — reads TX, writes RX
Usage: bridge.py <pty-path>
"""
import errno, fcntl, os, select, signal, socket, struct, sys, termios, threading, time

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615.0
CLK_IND_PERIOD = 102
SERCOMM_FLAG = 0x7E
SERCOMM_ESCAPE = 0x7D
SERCOMM_XOR = 0x20
DLCI_L1CTL = 5
DLCI_BURST = 4  # L1A — burst data

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s

def sercomm_wrap(dlci, payload):
    out = bytearray([SERCOMM_FLAG])
    for b in bytes([dlci, 0x03]) + payload:
        if b in (SERCOMM_FLAG, SERCOMM_ESCAPE):
            out.append(SERCOMM_ESCAPE)
            out.append(b ^ SERCOMM_XOR)
        else:
            out.append(b)
    out.append(SERCOMM_FLAG)
    return bytes(out)

class SercommParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            try: s = self.buf.index(SERCOMM_FLAG)
            except ValueError: self.buf.clear(); break
            if s > 0: del self.buf[:s]
            try: e = self.buf.index(SERCOMM_FLAG, 1)
            except ValueError: break
            raw = bytes(self.buf[1:e]); del self.buf[:e+1]
            if not raw: continue
            out = bytearray(); i = 0
            while i < len(raw):
                if raw[i] == SERCOMM_ESCAPE and i+1 < len(raw):
                    i += 1; out.append(raw[i] ^ SERCOMM_XOR)
                else: out.append(raw[i])
                i += 1
            if len(out) >= 2: frames.append((out[0], bytes(out[2:])))
        return frames

class Bridge:
    def __init__(self, pty_path, burst_pty_path=None, bts_base=5700):
        # UART PTY (L1CTL — modem)
        self.pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
        fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                    fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)

        # L1CTL unix socket (mobile ↔ bridge ↔ PTY)
        l1ctl_path = "/tmp/osmocom_l2_1"
        try: os.unlink(l1ctl_path)
        except: pass
        self.l1ctl_srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.l1ctl_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.l1ctl_srv.bind(l1ctl_path)
        self.l1ctl_srv.listen(1)
        self.l1ctl_srv.setblocking(False)
        self.l1ctl_cli = None
        self.l1ctl_buf = b""  # length-prefix accumulator

        # BTS TRX sockets
        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ("127.0.0.1", bts_base + 100)
        self.trxc_remote = None
        self.trxd_remote = None
        self.powered = False
        self.fn = 0
        self._stop = threading.Event()
        self.parser = SercommParser()
        self.stats = {"clk": 0, "trxc": 0, "dl": 0, "ul": 0}
        print(f"bridge: pty={pty_path} l1ctl={l1ctl_path} CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2}", flush=True)

    def start_clock(self):
        threading.Thread(target=self._clock_loop, daemon=True).start()

    def _clock_loop(self):
        tick_ns = int(GSM_FRAME_US * 1000)
        t_next = time.monotonic_ns()
        while not self._stop.is_set():
            t_next += tick_ns
            dt = t_next - time.monotonic_ns()
            if dt > 0: time.sleep(dt / 1e9)
            elif dt < -tick_ns * 10: t_next = time.monotonic_ns()
            self.fn = (self.fn + 1) % GSM_HYPERFRAME
            if self.fn % CLK_IND_PERIOD == 0 and self.powered:
                try:
                    self.clk_sock.sendto(f"IND CLOCK {self.fn}\0".encode(), self.bts_clk_addr)
                    self.stats["clk"] += 1
                except OSError: pass

    def handle_trxc(self):
        try: data, addr = self.trxc_sock.recvfrom(256)
        except OSError: return
        if not data: return
        self.trxc_remote = addr
        raw = data.strip(b'\x00').decode(errors='replace')
        if not raw.startswith("CMD "): return
        parts = raw[4:].split(); verb = parts[0]; args = parts[1:]
        self.stats["trxc"] += 1

        if verb == "POWERON":
            self.powered = True; print(f"BTS: POWERON", flush=True)
            rsp = "RSP POWERON 0"
        elif verb == "POWEROFF":
            self.powered = False; rsp = "RSP POWEROFF 0"
        elif verb == "SETFORMAT":
            rsp = f"RSP SETFORMAT 0 {' '.join(args)}"
        elif verb == "NOMTXPOWER":
            rsp = "RSP NOMTXPOWER 0 50"
        elif verb == "MEASURE":
            freq = args[0] if args else "0"
            rsp = f"RSP MEASURE 0 {freq} -60"
        else:
            rsp = f"RSP {verb} 0 {' '.join(args)}".rstrip()

        self.trxc_sock.sendto((rsp + "\0").encode(), addr)

    def handle_trxd_from_bts(self):
        """BTS DL burst → UDP 6802 → QEMU calypso_trx_rx_burst."""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 6: return
        self.trxd_remote = addr
        self.stats["dl"] += 1

        # Forward DL burst as sercomm DLCI 4 on PTY (MS side)
        tn = data[0] & 0x07
        if tn == 0:
            frame = sercomm_wrap(DLCI_BURST, data)
            try: os.write(self.pty_fd, frame)
            except OSError: pass

    def handle_uart_from_qemu(self):
        """QEMU firmware sercomm TX → parse → forward bursts to BTS."""
        try: raw = os.read(self.pty_fd, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): return
            raise
        if not raw: return

        for dlci, payload in self.parser.feed(raw):
            if dlci == DLCI_BURST and payload and len(payload) >= 6:
                # Firmware sent a burst → forward as TRXD to BTS
                if self.trxd_remote:
                    try: self.trxd_sock.sendto(payload, self.trxd_remote)
                    except OSError: pass
                    self.stats["ul"] += 1
            elif dlci == DLCI_L1CTL and payload:
                # Firmware sent L1CTL → forward to mobile
                self.handle_l1ctl_to_mobile(dlci, payload)

    def handle_l1ctl_accept(self):
        """Accept new mobile L1CTL client."""
        try:
            cli, _ = self.l1ctl_srv.accept()
            cli.setblocking(False)
        except OSError: return
        if self.l1ctl_cli:
            try: self.l1ctl_cli.close()
            except: pass
        self.l1ctl_cli = cli
        self.l1ctl_buf = b""
        print("bridge: L1CTL client connected", flush=True)
        # Send synthetic RESET_IND via sercomm DLCI 5 to PTY
        reset_ind = bytes([0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        frame = sercomm_wrap(DLCI_L1CTL, reset_ind)
        try: os.write(self.pty_fd, frame)
        except OSError: pass
        # Also send RESET_IND directly to mobile
        hdr = struct.pack(">H", 8)
        try: cli.sendall(hdr + reset_ind)
        except: pass

    def handle_l1ctl_from_mobile(self):
        """Mobile → L1CTL → sercomm DLCI 5 → PTY."""
        if not self.l1ctl_cli: return
        try:
            data = self.l1ctl_cli.recv(4096)
        except OSError: return
        if not data:
            print("bridge: L1CTL client disconnected", flush=True)
            self.l1ctl_cli.close()
            self.l1ctl_cli = None
            return
        self.l1ctl_buf += data
        # Parse length-prefixed L1CTL messages
        while len(self.l1ctl_buf) >= 2:
            msglen = struct.unpack(">H", self.l1ctl_buf[:2])[0]
            if len(self.l1ctl_buf) < 2 + msglen:
                break
            payload = self.l1ctl_buf[2:2+msglen]
            self.l1ctl_buf = self.l1ctl_buf[2+msglen:]
            # Wrap in sercomm DLCI 5 → PTY
            frame = sercomm_wrap(DLCI_L1CTL, payload)
            try: os.write(self.pty_fd, frame)
            except OSError: pass

    def handle_l1ctl_to_mobile(self, dlci, payload):
        """PTY sercomm DLCI 5 → L1CTL length-prefixed → mobile."""
        if not self.l1ctl_cli or not payload: return
        hdr = struct.pack(">H", len(payload))
        try: self.l1ctl_cli.sendall(hdr + payload)
        except OSError:
            print("bridge: L1CTL send error, closing", flush=True)
            self.l1ctl_cli.close()
            self.l1ctl_cli = None

    def run(self):
        self.start_clock()
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            fds = [self.pty_fd, self.trxc_sock, self.trxd_sock, self.l1ctl_srv]
            if self.l1ctl_cli:
                fds.append(self.l1ctl_cli)
            try:
                readable, _, _ = select.select(fds, [], [], 0.1)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()
            if self.pty_fd in readable: self.handle_uart_from_qemu()
            if self.l1ctl_srv in readable: self.handle_l1ctl_accept()
            if self.l1ctl_cli and self.l1ctl_cli in readable: self.handle_l1ctl_from_mobile()

            if self.stats["dl"] > 0 and self.stats["dl"] % 5000 == 0:
                print(f"bridge: clk={self.stats['clk']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

        self._stop.set()
        os.close(self.pty_fd)
        print(f"bridge: clk={self.stats['clk']} trxc={self.stats['trxc']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path>"); sys.exit(1)
    burst_pty = sys.argv[2] if len(sys.argv) > 2 else None
    Bridge(sys.argv[1], burst_pty).run()
