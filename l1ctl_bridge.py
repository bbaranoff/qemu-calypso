#!/usr/bin/env python3
"""
l1ctl_bridge.py — Pure relay + timing translator between fake_trx and QEMU.

No L1 implementation — firmware handles everything.

Relay:  fake_trx(6700-6702) ↔ bridge(6800-6802/7700-7702) ↔ QEMU(7800-7802)
L1CTL:  mobile ↔ QEMU firmware (sercomm over chardev socket)
Clock:  bridge free-runs, syncs from fake_trx CLK, sends CLK to QEMU

Usage:
  python3 l1ctl_bridge.py <uart-sock> [l1ctl-sock] [--print-bursts] [ft-base] [qe-base]
"""

import errno, fcntl, os, select, signal, socket, struct, sys, termios, threading, time

# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615.0
L1CTL_SOCK = "/tmp/osmocom_l2"
SERCOMM_FLAG = 0x7E; SERCOMM_ESCAPE = 0x7D; SERCOMM_XOR = 0x20
DLCI_L1CTL = 5

# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port)); s.setblocking(False); return s

def udp_bind_connect(bind_port, dst_addr, dst_port):
    """Bind + connect UDP socket (like osmo-bts-trx: ESTAB in ss)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", bind_port))
    s.connect((dst_addr, dst_port))
    s.setblocking(False); return s

def sercomm_wrap(dlci, payload):
    out = bytearray([SERCOMM_FLAG])
    for b in bytes([dlci, 0x03]) + payload:
        if b in (SERCOMM_FLAG, SERCOMM_ESCAPE):
            out.append(SERCOMM_ESCAPE); out.append(b ^ SERCOMM_XOR)
        else: out.append(b)
    out.append(SERCOMM_FLAG); return bytes(out)

class SercommParser:
    def __init__(self): self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data); frames = []
        while True:
            try: s = self.buf.index(SERCOMM_FLAG)
            except ValueError: self.buf.clear(); break
            if s > 0: del self.buf[:s]
            try: e = self.buf.index(SERCOMM_FLAG, 1)
            except ValueError: break
            raw = bytes(self.buf[1:e]); del self.buf[:e+1]
            if not raw: continue
            # unescape
            out = bytearray(); i = 0
            while i < len(raw):
                if raw[i] == SERCOMM_ESCAPE: i += 1; out.append(raw[i] ^ SERCOMM_XOR) if i < len(raw) else None
                else: out.append(raw[i])
                i += 1
            if len(out) >= 2: frames.append((out[0], bytes(out[2:])))
        return frames

class LPReader:
    def __init__(self): self.buf = bytearray()
    def feed(self, data):
        self.buf.extend(data); msgs = []
        while len(self.buf) >= 2:
            ml = struct.unpack(">H", self.buf[:2])[0]
            if len(self.buf) < 2 + ml: break
            msgs.append(bytes(self.buf[2:2+ml])); del self.buf[:2+ml]
        return msgs

# ═══════════════════════════════════════════════════════════════════════════════
# Clock
# ═══════════════════════════════════════════════════════════════════════════════

class BridgeClock:
    """Clock + TRXD buffer: receives bursts from fake_trx, replays to QEMU at 4.615ms/frame."""

    def __init__(self, clk_sock, trxd_sock, ind_period=102):
        self.clk_sock = clk_sock; self.trxd_sock = trxd_sock
        self.ind_period = ind_period; self.fn = 0
        self.synced = False
        self._lock = threading.Lock()
        self._stop = threading.Event(); self._thread = None
        # TRXD FIFO: bursts in order, one frame (8 TS) per tick
        from collections import deque
        self._trxd_fifo = deque(maxlen=2000)
        self._last_sent_fn = -1

    def sync(self, fn):
        with self._lock:
            if not self.synced:
                print(f"clock: sync FN={fn}", flush=True)
                self.synced = True
            self.fn = fn

    def buffer_trxd(self, data):
        """Buffer a DL TRXD packet from fake_trx (FIFO order)."""
        self._trxd_fifo.append(data)

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread: self._thread.join(timeout=2)

    def _run(self):
        tick_ns = int(GSM_FRAME_US * 1000)
        t_next = time.monotonic_ns()
        while not self._stop.is_set():
            t_next += tick_ns
            dt = t_next - time.monotonic_ns()
            if dt < 0: t_next = time.monotonic_ns(); dt = 0
            if dt > 0: time.sleep(dt / 1e9)

            with self._lock:
                fn = self.fn
                self.fn = (self.fn + 1) % GSM_HYPERFRAME

            # Send CLK to QEMU
            if fn % self.ind_period == 0:
                self.clk_sock.send(f"IND CLOCK {fn}\0".encode())

            # Send TRXD from FIFO — 1 packet per tick (matches QEMU speed)
            if self._trxd_fifo:
                pkt = self._trxd_fifo.popleft()
                try: self.trxd_sock.send(pkt)
                except: pass

# ═══════════════════════════════════════════════════════════════════════════════
# TRX Relay (pure forward, no L1)
# ═══════════════════════════════════════════════════════════════════════════════

class TRXRelay:
    def __init__(self, ft_base=6700, qe_base=8700, print_bursts=False):
        # 6 sockets: bind+connect, each does RX+TX
        self.ft_clk  = udp_bind_connect(ft_base + 100, "127.0.0.1", ft_base)      # 6800↔6700
        self.ft_trxc = udp_bind_connect(ft_base + 101, "127.0.0.1", ft_base + 1)  # 6801↔6701
        self.ft_trxd = udp_bind_connect(ft_base + 102, "127.0.0.1", ft_base + 2)  # 6802↔6702
        self.qe_clk  = udp_bind_connect(qe_base,     "127.0.0.1", qe_base + 100)  # 8700↔8800
        self.qe_trxc = udp_bind_connect(qe_base + 1, "127.0.0.1", qe_base + 101)  # 8701↔8801
        self.qe_trxd = udp_bind_connect(qe_base + 2, "127.0.0.1", qe_base + 102)  # 8702↔8802

        self.print_bursts = print_bursts
        self.stats = {"dl": 0, "ul": 0}

        # Dedicated TX socket for buffered TRXD (not in select, avoids loop)
        self.qe_trxd_buf_tx = udp_bind_connect(8712, "127.0.0.1", 8802)  # 8712→8802
        self.clock = BridgeClock(self.qe_clk, self.qe_trxd_buf_tx)  # sends CLK+TRXD to QEMU
        self.clock.start()

        # Init fake_trx MS (bridge configures the connection, not QEMU)
        self._init_faketrx_ms()

        print(f"relay: ft({ft_base}) ↔ bridge({ft_base+100}/{qe_base}) ↔ qe({qe_base+100})", flush=True)

    def _init_faketrx_ms(self):
        """Configure fake_trx MS transceiver via TRXC."""
        import time as _t
        cmds = [b"CMD SETFORMAT 0\0", b"CMD RXTUNE 1805600\0",
                b"CMD TXTUNE 1710600\0", b"CMD POWERON\0"]
        for cmd in cmds:
            self.ft_trxc.send(cmd)
            _t.sleep(0.2)
            try:
                rsp = self.ft_trxc.recv(256)
                NUL = b'\x00'
                print(f"  [init] {cmd.strip(NUL).decode()} → {rsp.strip(NUL).decode()}", flush=True)
            except: pass

    def sockets(self):
        return [self.ft_clk, self.ft_trxc, self.ft_trxd,
                self.qe_clk, self.qe_trxc, self.qe_trxd]

    def process(self, readable):
        # DL: fake_trx → QEMU
        for sock, peer, ch in [
            (self.ft_clk,  self.qe_clk,  "CLK"),
            (self.ft_trxc, self.qe_trxc, "TRXC"),
            (self.ft_trxd, None,         "TRXD"),
        ]:
            if sock in readable:
                try: data = sock.recv(4096)
                except OSError: continue
                if not data: continue
                if ch == "TRXD":
                    self.clock.buffer_trxd(data)
                elif peer:
                    peer.send(data)
                self.stats["dl"] += 1
                if ch == "CLK":
                    try:
                        txt = data.strip(b'\x00').decode()
                        if txt.startswith("IND CLOCK "):
                            self.clock.sync(int(txt.split()[2]))
                    except: pass
                if self.print_bursts and ch == "TRXD" and len(data) >= 8:
                    tn = data[0] & 7; fn = struct.unpack(">I", data[1:5])[0]
                    nz = sum(1 for b in data[8:156] if b not in (0, 128))
                    print(f"  [DL] TN={tn} FN={fn} nz={nz}", flush=True)

        # UL: QEMU → fake_trx
        for sock, peer, ch in [
            (self.qe_clk,  self.ft_clk,  "CLK"),
            (self.qe_trxc, self.ft_trxc, "TRXC"),
            (self.qe_trxd, self.ft_trxd, "TRXD"),
        ]:
            if sock in readable:
                try: data = sock.recv(4096)
                except OSError: continue
                if not data: continue
                peer.send(data)
                self.stats["ul"] += 1
                if ch == "TRXC":
                    txt = data.strip(b'\x00').decode(errors='replace')
                    if self.stats["ul"] <= 20:
                        print(f"  [UL] TRXC: {txt}", flush=True)
                if self.print_bursts and ch == "TRXD" and len(data) >= 6:
                    tn = data[0] & 7; fn = struct.unpack(">I", data[1:5])[0]
                    print(f"  [UL] TN={tn} FN={fn}", flush=True)

        if self.stats["dl"] % 10000 == 0 and self.stats["dl"] > 0:
            print(f"  [stats] DL={self.stats['dl']} UL={self.stats['ul']}", flush=True)

    def close(self):
        self.clock.stop()
        for s in self.sockets():
            try: s.close()
            except: pass

# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <uart-sock> [l1ctl-sock] [--print-bursts] [ft-base] [qe-base]")
        sys.exit(1)

    pty_path = sys.argv[1]
    global L1CTL_SOCK
    if len(sys.argv) > 2 and not sys.argv[2].startswith("-"):
        L1CTL_SOCK = sys.argv[2]

    args = sys.argv[2:]
    print_bursts = "--print-bursts" in args
    nums = [a for a in args if a.isdigit()]
    ft_base = int(nums[0]) if len(nums) > 0 else 6700
    qe_base = int(nums[1]) if len(nums) > 1 else 7700

    try: os.unlink(L1CTL_SOCK)
    except FileNotFoundError: pass

    # QEMU uart
    import stat as stat_mod
    qemu_sock = None
    if os.path.exists(pty_path) and stat_mod.S_ISSOCK(os.stat(pty_path).st_mode):
        qemu_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        qemu_sock.connect(pty_path); qemu_sock.setblocking(False)
        pty_fd = qemu_sock.fileno()
    else:
        pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(pty_fd, termios.TCSANOW, attrs)
        fl = fcntl.fcntl(pty_fd, fcntl.F_GETFL)
        fcntl.fcntl(pty_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    # L1CTL server
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(L1CTL_SOCK); srv.listen(1); srv.setblocking(False)

    # TRX relay
    trx = TRXRelay(ft_base, qe_base, print_bursts)

    print(f"bridge: uart={pty_path} l1ctl={L1CTL_SOCK}", flush=True)

    parser = SercommParser(); lp = LPReader()
    client = None; running = True
    stats = {"tx": 0, "rx": 0}

    def shutdown(s, f): nonlocal running; running = False
    signal.signal(signal.SIGINT, shutdown); signal.signal(signal.SIGTERM, shutdown)

    try:
        while running:
            rlist = [srv, pty_fd] + trx.sockets()
            if client: rlist.append(client)
            try: readable, _, _ = select.select(rlist, [], [], 0.5)
            except (OSError, ValueError): break

            trx.process(readable)

            # Accept mobile
            if srv in readable:
                conn, _ = srv.accept(); conn.setblocking(False)
                if client:
                    try: client.close()
                    except: pass
                client = conn; lp = LPReader()
                print("mobile connected", flush=True)

            # L1CTL: mobile → firmware (pure forward)
            if client and client in readable:
                try: raw = client.recv(4096)
                except BlockingIOError: raw = b""
                if not raw:
                    print("mobile disconnected", flush=True)
                    try: client.close()
                    except: pass
                    client = None
                else:
                    for payload in lp.feed(raw):
                        stats["tx"] += 1
                        frame = sercomm_wrap(DLCI_L1CTL, payload)
                        try: os.write(pty_fd, frame)
                        except OSError as e:
                            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK): raise
                        if stats["tx"] <= 10 or stats["tx"] % 50 == 0:
                            mt = payload[0] if payload else 0xFF
                            print(f"  [mobile→fw] #{stats['tx']} type=0x{mt:02x}", flush=True)

            # L1CTL: firmware → mobile (pure forward)
            if pty_fd in readable:
                try: raw = os.read(pty_fd, 4096)
                except OSError as e:
                    if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): continue
                    raise
                if not raw: break
                for dlci, payload in parser.feed(raw):
                    if dlci == DLCI_L1CTL and client:
                        stats["rx"] += 1
                        try: client.sendall(struct.pack(">H", len(payload)) + payload)
                        except: client = None
                        if stats["rx"] <= 10 or stats["rx"] % 100 == 0:
                            mt = payload[0] if payload else 0xFF
                            print(f"  [fw→mobile] #{stats['rx']} type=0x{mt:02x}", flush=True)
    finally:
        if client: client.close()
        srv.close(); trx.close()
        if qemu_sock: qemu_sock.close()
        else: os.close(pty_fd)
        try: os.unlink(L1CTL_SOCK)
        except: pass
        print(f"done (l1ctl tx={stats['tx']} rx={stats['rx']} trx dl={trx.stats['dl']} ul={trx.stats['ul']})", flush=True)

if __name__ == "__main__":
    main()
