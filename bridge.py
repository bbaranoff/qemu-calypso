#!/usr/bin/env python3
"""
bridge.py — BTS TRX UDP bridge for QEMU Calypso

BTS side:  osmo-bts-trx (CLK/TRXC/TRXD on 5700-5702)
QEMU side: BSP receives DL bursts on UDP 6702
           QEMU sends TDMA ticks on UDP 6700 (QEMU is clock master)
           QEMU sends UL bursts back on UDP 5702 (TRXD is bidirectional)

The bridge tracks QEMU's FN but sends CLK IND to the BTS at wall-clock
rate (~470ms = 102 frames). The BTS scheduler needs real-time CLK IND
to avoid drift, but the FN must come from QEMU for coherence.

TRXD socket (5702) is bidirectional:
  DL: BTS → bridge:5702 → QEMU:6702   (forward downlink to BSP)
  UL: QEMU:6702 → bridge:5702 → BTS   (forward uplink to BTS peer)

Usage: bridge.py
"""
import errno, os, select, signal, socket, struct, sys, time

GSM_HYPERFRAME = 2715648
# CLK IND period in frames (default 102 = stock GSM TDMA spec).
# In debug runs where QEMU is slower than wall-clock, send CLK IND more
# frequently (e.g. 51) so osmo-bts-trx scheduler accumulates less skew
# between two corrections — band-aid that buys time to capture RACH
# attempts before bts_shutdown_fsm fires "PC clock skew too high" or
# "No more clock from transceiver". Override via env BRIDGE_CLK_PERIOD.
CLK_IND_PERIOD = int(os.environ.get("BRIDGE_CLK_PERIOD", "102"))
CLK_IND_WALL_S = (CLK_IND_PERIOD * 4615) / 1_000_000

QEMU_BSP_ADDR = ("127.0.0.1", 6702)

# When set, drive CLK IND from QEMU FN advance instead of host wall-clock.
# Eliminates host-load jitter at the cost of CLK IND rate following QEMU
# emulation speed. Pair with -icount on QEMU for full determinism.
CLK_FROM_QEMU = os.environ.get("BRIDGE_CLK_FROM_QEMU", "0") == "1"

def udp_bind(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.setblocking(False)
    return s

class Bridge:
    def __init__(self, bts_base=5700):
        self.clk_sock  = udp_bind(bts_base)
        self.trxc_sock = udp_bind(bts_base + 1)
        self.trxd_sock = udp_bind(bts_base + 2)
        self.bts_clk_addr = ("127.0.0.1", bts_base + 100)
        self.trxc_remote = None
        # Pre-set TRXD remote to osmo-bts-trx convention (base+102=5802) so
        # UL packets from QEMU forward correctly even before the first DL
        # has arrived. Refined to the actual sender on first DL.
        self.trxd_remote = ("127.0.0.1", bts_base + 102)
        self.powered = False
        self.fn = 0
        # BTS starts its own FN at 0 on POWERON — several seconds after QEMU
        # has already advanced. Remember QEMU's FN at POWERON so we can re-
        # anchor BTS-tagged bursts to QEMU's timeline on the way in.
        self.fn_anchor = 0
        self.anchored = False
        self._stop = False
        self.stats = {"clk": 0, "trxc": 0, "dl": 0, "ul": 0, "tick": 0}

        # QEMU CLK tick receiver
        self.qemu_clk_sock = udp_bind(6700)

        # CLK IND is driven by wall-clock (see maybe_send_clk).
        self.last_clk_fn = None

        print(f"bridge: CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2} ↔ BSP@6702", flush=True)
        print(f"bridge: QEMU CLK ticks on UDP 6700 (clock-master, bridge slave)", flush=True)

    def handle_qemu_tick(self):
        """Receive TDMA tick from QEMU — 4 bytes big-endian FN."""
        try:
            data, addr = self.qemu_clk_sock.recvfrom(64)
        except OSError:
            return
        if len(data) < 4:
            return

        fn = int.from_bytes(data[0:4], 'big')
        self.fn = fn % GSM_HYPERFRAME
        self.stats["tick"] += 1

        if self.stats["tick"] <= 3 or self.stats["tick"] % 10000 == 0:
            print(f"bridge: QEMU tick #{self.stats['tick']} FN={self.fn}", flush=True)

        # In QEMU-driven CLK mode, send CLK IND on every tick crossing the
        # CLK_IND_PERIOD boundary. Eliminates wall-clock jitter.
        if CLK_FROM_QEMU and self.powered:
            if self.last_clk_fn is None or \
               (self.fn - self.last_clk_fn) % GSM_HYPERFRAME >= CLK_IND_PERIOD:
                self._send_clk_ind()
                self.last_clk_fn = self.fn

    def _send_clk_ind(self):
        clk_fn = (self.fn // CLK_IND_PERIOD) * CLK_IND_PERIOD
        try:
            self.clk_sock.sendto(
                f"IND CLOCK {clk_fn}\0".encode(), self.bts_clk_addr)
            self.stats["clk"] += 1
        except OSError:
            pass

    def maybe_send_clk(self):
        """Wall-clock-paced CLK IND.

        osmo-bts-trx's scheduler_trx.c monitors PC clock skew (real elapsed
        time between CLK INDs vs the GSM time they advertise). When the
        guest emulation is slower than real-time, sending CLK INDs at
        wall-clock cadence keeps the BTS scheduler from declaring skew —
        but introduces host-load-dependent jitter. Use BRIDGE_CLK_FROM_QEMU=1
        for deterministic runs (CLK driven from QEMU FN advance).
        """
        if CLK_FROM_QEMU or not self.powered:
            return
        now = time.monotonic()
        if not hasattr(self, '_last_clk_wall'):
            self._last_clk_wall = now - CLK_IND_WALL_S  # force first send
        if now - self._last_clk_wall < CLK_IND_WALL_S:
            return
        self._last_clk_wall += CLK_IND_WALL_S
        # Catch up if we slipped a long time (avoid runaway send burst)
        if now - self._last_clk_wall > CLK_IND_WALL_S * 4:
            self._last_clk_wall = now
        self._send_clk_ind()

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
            self.powered = True
            self.fn_anchor = self.fn
            self.anchored = True
            print(f"BTS: POWERON (FN anchor = {self.fn_anchor})", flush=True)
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

    def handle_trxd(self):
        """Bidirectional TRXD relay.

        Discriminates by source address:
          - From QEMU (127.0.0.1:6702) → UL burst → forward to BTS
          - From anyone else           → DL burst from BTS → forward to QEMU

        UL packets that arrive before the BTS peer is known (no DL yet)
        are dropped with a counter so we can detect the race in logs.
        """
        try:
            data, addr = self.trxd_sock.recvfrom(512)
        except OSError:
            return
        if len(data) < 6:
            return

        if addr == QEMU_BSP_ADDR:
            self._handle_ul(data)
        else:
            self._handle_dl(data, addr)

    def _handle_ul(self, data):
        """UL burst from QEMU → forward to BTS TRXD endpoint.

        Format (TRXDv0 UL, 156 bytes total, set in calypso_bsp_send_ul):
          [0]    TN
          [1:5]  FN (BE)
          [5]    RSSI offset (BTS sees -value dBm)
          [6:8]  ToA256 (BE)
          [8:]   148 soft bits (±127)

        trxd_remote is pre-set in __init__ so the first UL is never dropped.
        """
        self.stats["ul"] += 1
        tn = data[0] & 0x07
        fn = int.from_bytes(data[1:5], 'big')
        rssi = data[5] if len(data) > 5 else 0

        if self.stats["ul"] <= 10 or (self.stats["ul"] % 5000) == 0:
            print(f"bridge: UL #{self.stats['ul']} TN={tn} fn={fn} "
                  f"rssi=-{rssi} len={len(data)} → BTS {self.trxd_remote}",
                  flush=True)

        try:
            self.trxd_sock.sendto(data, self.trxd_remote)
        except OSError as e:
            print(f"bridge: UL send error: {e}", flush=True)

    def _handle_dl(self, data, addr):
        """DL burst from BTS → forward to QEMU BSP."""
        self.trxd_remote = addr
        self.stats["dl"] += 1

        tn = data[0] & 0x07
        bts_fn = int.from_bytes(data[1:5], 'big')

        # osmo-bts-trx maintains its own continuous FN (wall-clock based)
        # and locks onto our CLK_IND stream — so bts_fn is already on
        # QEMU's timeline. fn_anchor kept for telemetry only.
        fn = bts_fn

        # Log burst content: first 8 data bytes + check if FB (all zeros)
        hdr_bytes = data[:8] if len(data) >= 8 else data
        payload = data[8:] if len(data) > 8 else b''
        is_fb = all(b == 0 for b in payload) if payload else False
        if self.stats["dl"] <= 10 or self.stats["dl"] % 5000 == 0 or is_fb:
            print(f"bridge: DL #{self.stats['dl']} TN={tn} "
                  f"bts_fn={bts_fn} fn={fn} (anchor={self.fn_anchor}) "
                  f"len={len(data)} hdr={hdr_bytes[:8].hex()} "
                  f"bits[0:8]={list(payload[:8])} "
                  f"{'*** FB ***' if is_fb else ''}", flush=True)

        # Rewrite the FN in-place before forwarding (no-op currently, kept
        # in case BTS clock-slaves on its own counter again in the future).
        out = bytearray(data)
        out[1] = (fn >> 24) & 0xFF
        out[2] = (fn >> 16) & 0xFF
        out[3] = (fn >>  8) & 0xFF
        out[4] =  fn        & 0xFF
        try:
            self.trxd_sock.sendto(bytes(out), QEMU_BSP_ADDR)
        except OSError as e:
            print(f"bridge: DL send error: {e}", flush=True)

    def run(self):
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        while running:
            fds = [self.trxc_sock, self.trxd_sock, self.qemu_clk_sock]
            try:
                readable, _, _ = select.select(fds, [], [], 0.1)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.qemu_clk_sock in readable: self.handle_qemu_tick()
            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd()

            # Send CLK IND at wall-clock rate
            self.maybe_send_clk()

            if (self.stats["dl"] + self.stats["ul"]) > 0 and \
               ((self.stats["dl"] + self.stats["ul"]) % 5000) == 0:
                print(f"bridge: tick={self.stats['tick']} "
                      f"clk={self.stats['clk']} "
                      f"dl={self.stats['dl']} ul={self.stats['ul']}",
                      flush=True)

        self._stop = True
        print(f"bridge: tick={self.stats['tick']} clk={self.stats['clk']} "
              f"trxc={self.stats['trxc']} dl={self.stats['dl']} "
              f"ul={self.stats['ul']}", flush=True)

if __name__ == "__main__":
    Bridge().run()
