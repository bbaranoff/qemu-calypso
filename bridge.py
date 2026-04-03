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
    def __init__(self, pty_path, bts_base=5700):
        # UART PTY
        self.pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
        fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                    fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)

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
        print(f"bridge: pty={pty_path} CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2}", flush=True)

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
        """BTS DL burst → wrap in sercomm → inject into QEMU UART RX."""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 6: return
        self.trxd_remote = addr
        self.stats["dl"] += 1

        # Wrap raw TRXD in sercomm DLCI_BURST → write to PTY (UART RX)
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

    def run(self):
        self.start_clock()
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            try:
                readable, _, _ = select.select(
                    [self.pty_fd, self.trxc_sock, self.trxd_sock], [], [], 0.1)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()
            if self.pty_fd in readable: self.handle_uart_from_qemu()

            if self.stats["dl"] > 0 and self.stats["dl"] % 5000 == 0:
                print(f"bridge: clk={self.stats['clk']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

        self._stop.set()
        os.close(self.pty_fd)
        print(f"bridge: clk={self.stats['clk']} trxc={self.stats['trxc']} dl={self.stats['dl']} ul={self.stats['ul']}", flush=True)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <pty-path>"); sys.exit(1)
    Bridge(sys.argv[1]).run()
