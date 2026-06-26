#!/usr/bin/env python3
"""
sercomm_udp.py — BTS TRX <-> QEMU UART sercomm bridge
BTS side:  OsmoTRX protocol (CLK/TRXC/TRXD on 5700-5702)
MS side:   UART PTY (sercomm DLCI 4=burst, DLCI 5=l1ctl)

DL: osmo-bts-trx sends TRXD soft bits -> convert to int16 -> sercomm DLCI 4 -> PTY
UL: PTY sercomm DLCI 4 -> TRXD hard bits -> osmo-bts-trx

Usage: sercomm_udp.py <pty-path> [bts-base-port]
"""
import errno, fcntl, os, select, signal, socket, struct, sys, termios, threading, time

GSM_HYPERFRAME = 2715648
GSM_FRAME_US = 4615.0
CLK_IND_PERIOD = 102
SERCOMM_FLAG = 0x7E
SERCOMM_ESCAPE = 0x7D
SERCOMM_XOR = 0x20
DLCI_L1CTL = 5
DLCI_BURST = 4

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('127.0.0.1', port))
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

def soft_to_int16(soft_bit):
    """Convert osmocom soft bit (0=strong 1, 255=strong 0) to int16.
    Maps: 0 -> +32767, 127 -> 0, 255 -> -32767"""
    return max(-32768, min(32767, int((127 - soft_bit) * 32767 / 127)))

def soft_bits_to_gmsk_iq(soft_bits_raw, scale=4096):
    """Convert TRXD soft bits to GMSK I/Q int16 samples for DSP BSP.
    Calypso ABB delivers baseband I/Q from antenna; we generate equivalent
    by GMSK-modulating the hard bits (BT=0.3, h=0.5, 1 sample/symbol)."""
    import numpy as np
    from scipy.ndimage import gaussian_filter1d
    n = len(soft_bits_raw)
    hard = np.array([0.0 if b < 128 else 1.0 for b in soft_bits_raw])
    nrz = 1.0 - 2.0 * hard  # 0->+1, 1->-1 (GSM convention)
    bt = 0.3
    filtered = gaussian_filter1d(nrz, 1.0 / (2.0 * 3.141592653589793 * bt))
    phase = np.cumsum(filtered) * 3.141592653589793 * 0.5
    iq = np.clip(np.cos(phase) * scale, -32768, 32767).astype(np.int16)
    return iq

class Bridge:
    def __init__(self, pty_path, bts_base=5700):
        self.pty_fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.pty_fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 1; attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
        fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                    fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)

        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ('127.0.0.1', bts_base + 100)
        self.trxc_remote = None
        self.trxd_remote = None
        self.powered = False
        self.fn = 0
        self._stop = threading.Event()
        self.parser = SercommParser()
        self.stats = {'clk': 0, 'trxc': 0, 'dl': 0, 'ul': 0}
        self.cfile_fd = open('/tmp/gsm_burst.cfile', 'wb')
        print('sercomm_udp: pty=%s CLK=%d TRXC=%d TRXD=%d' % (pty_path, bts_base, bts_base+1, bts_base+2), flush=True)

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
                    self.clk_sock.sendto(('IND CLOCK %d\0' % self.fn).encode(), self.bts_clk_addr)
                    self.stats['clk'] += 1
                except OSError: pass

    def handle_trxc(self):
        try: data, addr = self.trxc_sock.recvfrom(256)
        except OSError: return
        if not data: return
        self.trxc_remote = addr
        raw = data.strip(b'\x00').decode(errors='replace')
        if not raw.startswith('CMD '): return
        parts = raw[4:].split(); verb = parts[0]; args = parts[1:]
        self.stats['trxc'] += 1

        if verb == 'POWERON':
            self.powered = True; print('BTS: POWERON', flush=True)
            rsp = 'RSP POWERON 0'
        elif verb == 'POWEROFF':
            self.powered = False; rsp = 'RSP POWEROFF 0'
        elif verb == 'SETFORMAT':
            # Force TRXD v0 — we only support v0 format
            rsp = 'RSP SETFORMAT 0 0'
        elif verb == 'NOMTXPOWER':
            rsp = 'RSP NOMTXPOWER 0 50'
        elif verb == 'MEASURE':
            freq = args[0] if args else '0'
            rsp = 'RSP MEASURE 0 %s -60' % freq
        else:
            rsp = ('RSP %s 0 %s' % (verb, ' '.join(args))).rstrip()
        self.trxc_sock.sendto((rsp + '\0').encode(), addr)

    def handle_trxd_from_bts(self):
        """BTS DL: TRXD v0 soft bits -> int16 samples -> sercomm DLCI 4 -> PTY.
        TRXD v0 DL: TN(1) FN(4) RSSI(1) TOA256(2) soft_bits(148)
        Sercomm payload: TN(1) FN(4) RSSI(1) TOA256(2) int16_samples(148*2)"""
        try: data, addr = self.trxd_sock.recvfrom(512)
        except OSError: return
        if len(data) < 8: return
        self.trxd_remote = addr
        if not self.powered: return
        self.stats['dl'] += 1

        hdr = data[:8]
        soft_bits = data[8:]
        nbits = min(len(soft_bits), 148)

        # GMSK modulate soft bits → I/Q int16 samples for DSP BSP
        iq_samples = soft_bits_to_gmsk_iq(soft_bits[:nbits])
        payload = bytearray(hdr)
        payload.extend(iq_samples.tobytes())

        # Log I/Q as complex float32 cfile for grgsm_decode
        if self.cfile_fd is not None:
            import numpy as np
            cf = (iq_samples.astype(np.float32) / 4096.0).view(np.float32)
            # Write as complex: I + 0j (real-only for now)
            cplx = np.zeros(len(iq_samples) * 2, dtype=np.float32)
            cplx[0::2] = cf  # I
            # cplx[1::2] = 0  # Q = 0 (already zeroed)
            self.cfile_fd.write(cplx.tobytes())

        frame = sercomm_wrap(DLCI_BURST, bytes(payload))
        try: os.write(self.pty_fd, frame)
        except OSError: pass

    def handle_uart_from_qemu(self):
        try: raw = os.read(self.pty_fd, 4096)
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK): return
            raise
        if not raw: return
        for dlci, payload in self.parser.feed(raw):
            if dlci == DLCI_BURST and payload and len(payload) >= 6:
                if self.trxd_remote:
                    try: self.trxd_sock.sendto(payload, self.trxd_remote)
                    except OSError: pass
                    self.stats['ul'] += 1

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
            except (OSError, ValueError):
                break
            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd_from_bts()
            if self.pty_fd in readable: self.handle_uart_from_qemu()

        self._stop.set()
        os.close(self.pty_fd)
        print('sercomm_udp: stats=%s' % self.stats, flush=True)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: %s <pty-path>' % sys.argv[0]); sys.exit(1)
    Bridge(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 5700).run()
