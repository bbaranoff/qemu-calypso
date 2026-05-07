#!/usr/bin/env python3
"""
bridge.py — BTS TRX UDP bridge for QEMU Calypso

BTS side:  osmo-bts-trx (CLK/TRXC/TRXD on 5700-5702)
QEMU side: BSP receives DL bursts on UDP 6702
           QEMU sends TDMA ticks on UDP 6700 (QEMU is clock master)
           QEMU sends UL bursts back on UDP 5702 (TRXD is bidirectional)

CLOCK DOMAIN BRIDGE
-------------------
QEMU runs ~2x slower than wall-clock in this build (DSP emulator cost).
osmo-bts-trx and osmo-bsc both run on wall-clock. Without a translation
layer, the FN counters diverge and the BTS scheduler shuts down with
"PC clock skew too high".

This bridge maintains its own *wall-clock-paced* FN counter (`wall_fn`)
that ticks at 217 Hz regardless of QEMU's emulation speed. CLK INDs to
the BTS use `wall_fn` so BTS↔BSC see consistent wall-paced GSM time.

UL bursts arriving from QEMU carry QEMU's lagged FN in their TRXD
header ; we rewrite the FN field to the current `wall_fn` before
forwarding to osmo-bts-trx, so the BTS RACH/SDCCH scheduler matches
the burst against its wall-clock-aligned window. Burst content (sync
sequence, FIRE-encoded data, parity) is FN-invariant so this rewrite
is safe.

DL bursts from BTS already carry wall_fn in their TRXD header. They
are forwarded to QEMU's BSP unchanged — the BSP queue uses a wide
match window (cf. BSP_FN_MATCH_WINDOW in calypso_bsp.c) so wall-clock-
tagged bursts are still picked up by QEMU at delivery time.

TRXD socket (5702) is bidirectional:
  DL: BTS → bridge:5702 → QEMU:6702   (forward downlink to BSP)
  UL: QEMU:6702 → bridge:5702 → BTS   (forward uplink to BTS, FN rewritten)

Usage: bridge.py
"""
import errno, os, select, signal, socket, struct, sys, time

GSM_HYPERFRAME = 2715648
GSM_TDMA_S = 4615 / 1_000_000  # 4.615 ms per TDMA frame, wall-clock

# CLK IND period in frames (default 102 = stock GSM TDMA spec).
# In debug runs where QEMU is slower than wall-clock, the bridge sends
# CLK IND at wall-clock pace using its own wall_fn counter — see CLOCK
# DOMAIN BRIDGE block above. CLK_IND_PERIOD just controls the cadence
# (every N wall-paced frames). Default 102 = standard GSM rate.
CLK_IND_PERIOD = int(os.environ.get("BRIDGE_CLK_PERIOD", "102"))
CLK_IND_WALL_S = (CLK_IND_PERIOD * 4615) / 1_000_000

QEMU_BSP_ADDR = ("127.0.0.1", 6702)

# Mode toggle. With the wall-clock-paced wall_fn (default), BRIDGE_CLK_FROM_QEMU
# is effectively ignored — both CLK IND and UL FN rewrite use wall_fn. Kept
# as env var for backward compat / future override.
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
        # QEMU-side FN — kept for telemetry only. NOT used to drive CLK IND
        # or UL FN rewrite anymore (both use wall_fn).
        self.fn = 0
        # BTS starts its own FN at 0 on POWERON — several seconds after the
        # bridge has already been running. Remember bridge wall_fn at
        # POWERON so BTS-tagged DL bursts can be matched against the right
        # timeline if needed (currently only telemetry).
        self.fn_anchor = 0
        self.anchored = False
        self._stop = False
        self.stats = {"clk": 0, "trxc": 0, "dl": 0, "ul": 0,
                      "tick": 0, "ul_fn_rewrite": 0}

        # QEMU CLK tick receiver
        self.qemu_clk_sock = udp_bind(6700)

        # Wall-clock-paced FN counter — independent of QEMU. Anchored to
        # POWERON so BTS sees fn=0 right when it asks to power up.
        self._wall_t0 = None       # set at POWERON
        self._last_clk_fn_sent = None

        print(f"bridge: CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2} ↔ BSP@6702", flush=True)
        print(f"bridge: wall-clock-paced wall_fn ({1/GSM_TDMA_S:.1f} Hz) — UL FN rewritten to wall_fn", flush=True)

    def wall_fn(self):
        """Compute current bridge wall-paced FN. Anchored at POWERON."""
        if self._wall_t0 is None:
            return 0
        elapsed = time.monotonic() - self._wall_t0
        return int(elapsed / GSM_TDMA_S) % GSM_HYPERFRAME

    def handle_qemu_tick(self):
        """Receive TDMA tick from QEMU — 4 bytes big-endian FN.
        Used as telemetry only ; CLK IND is wall_fn-paced now."""
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
            wfn = self.wall_fn()
            lag = (wfn - self.fn) % GSM_HYPERFRAME
            print(f"bridge: QEMU tick #{self.stats['tick']} qfn={self.fn} "
                  f"wall_fn={wfn} lag={lag}", flush=True)

    def _send_clk_ind(self):
        wfn = self.wall_fn()
        clk_fn = (wfn // CLK_IND_PERIOD) * CLK_IND_PERIOD
        try:
            self.clk_sock.sendto(
                f"IND CLOCK {clk_fn}\0".encode(), self.bts_clk_addr)
            self.stats["clk"] += 1
            self._last_clk_fn_sent = clk_fn
        except OSError:
            pass

    def maybe_send_clk(self):
        """Wall-clock-paced CLK IND — bridge's own wall_fn counter, no QEMU dep.

        Sent at exact GSM TDMA rate (every CLK_IND_PERIOD * 4.615 ms wall).
        BTS scheduler stays synchronized with bridge regardless of QEMU
        emulation speed.
        """
        if not self.powered:
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
            # Anchor wall-clock FN at POWERON so BTS sees fn=0 right when
            # it powers up — matches osmo-bts-trx scheduler expectation.
            self._wall_t0 = time.monotonic()
            self.fn_anchor = self.fn
            self.anchored = True
            print(f"BTS: POWERON (wall_fn anchor at t0, qfn={self.fn})", flush=True)
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
          [1:5]  FN (BE) — REWRITTEN by bridge to current wall_fn
          [5]    RSSI offset (BTS sees -value dBm)
          [6:8]  ToA256 (BE)
          [8:]   148 soft bits (±127)

        FN rewrite: QEMU runs ~2x slower than wall-clock so the FN in the
        incoming burst lags wall-clock by hundreds of frames. BTS scheduler
        only accepts bursts within a small window of its current wall-paced
        FN. We rewrite the header FN to wall_fn so the burst lands in the
        BTS scheduler window. Burst content is FN-invariant.
        """
        self.stats["ul"] += 1
        tn = data[0] & 0x07
        qemu_fn = int.from_bytes(data[1:5], 'big')
        rssi = data[5] if len(data) > 5 else 0
        toa = int.from_bytes(data[6:8], 'big', signed=True) if len(data) >= 8 else 0

        # Rewrite FN to wall_fn before forwarding
        wfn = self.wall_fn() if self.powered else qemu_fn
        out = bytearray(data)
        out[1] = (wfn >> 24) & 0xFF
        out[2] = (wfn >> 16) & 0xFF
        out[3] = (wfn >>  8) & 0xFF
        out[4] =  wfn        & 0xFF
        if wfn != qemu_fn:
            self.stats["ul_fn_rewrite"] += 1

        # Print full header + first/last bits of every UL burst (cap 200 to
        # avoid log flood). Show both FNs to track the rewrite.
        if self.stats["ul"] <= 200 or (self.stats["ul"] % 1000) == 0:
            hdr_in_hex  = data[:8].hex()
            hdr_out_hex = bytes(out[:8]).hex()
            payload = data[8:]
            head = ' '.join(f"{b - 256 if b >= 128 else b:+d}" for b in payload[:16])
            tail = ' '.join(f"{b - 256 if b >= 128 else b:+d}" for b in payload[-8:])
            print(f"bridge: UL #{self.stats['ul']} TN={tn} "
                  f"qfn={qemu_fn}→wfn={wfn} rssi=-{rssi} toa={toa} len={len(data)} "
                  f"hdr_in={hdr_in_hex} hdr_out={hdr_out_hex} "
                  f"bits[0:16]=[{head}] bits[140:148]=[{tail}] "
                  f"→ BTS {self.trxd_remote}", flush=True)

        try:
            self.trxd_sock.sendto(bytes(out), self.trxd_remote)
        except OSError as e:
            print(f"bridge: UL send error: {e}", flush=True)

    def _handle_dl(self, data, addr):
        """DL burst from BTS → forward to QEMU BSP.

        BTS sends DL with wall-clock-aligned FN. We forward unchanged ;
        QEMU's BSP queue uses a tolerant FN match window so bursts are
        still picked up at delivery time even with QEMU FN drift.
        """
        self.trxd_remote = addr
        self.stats["dl"] += 1

        tn = data[0] & 0x07
        bts_fn = int.from_bytes(data[1:5], 'big')
        fn = bts_fn  # forward as-is

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

        try:
            self.trxd_sock.sendto(data, QEMU_BSP_ADDR)
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
                readable, _, _ = select.select(fds, [], [], 0.05)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.qemu_clk_sock in readable: self.handle_qemu_tick()
            if self.trxc_sock in readable: self.handle_trxc()
            if self.trxd_sock in readable: self.handle_trxd()

            # Send CLK IND at wall-clock rate using bridge's wall_fn
            self.maybe_send_clk()

            if (self.stats["dl"] + self.stats["ul"]) > 0 and \
               ((self.stats["dl"] + self.stats["ul"]) % 5000) == 0:
                wfn = self.wall_fn()
                print(f"bridge: tick={self.stats['tick']} "
                      f"clk={self.stats['clk']} "
                      f"dl={self.stats['dl']} ul={self.stats['ul']} "
                      f"ul_rewrite={self.stats['ul_fn_rewrite']} "
                      f"wall_fn={wfn} qfn={self.fn}",
                      flush=True)

        self._stop = True
        print(f"bridge: tick={self.stats['tick']} clk={self.stats['clk']} "
              f"trxc={self.stats['trxc']} dl={self.stats['dl']} "
              f"ul={self.stats['ul']} ul_rewrite={self.stats['ul_fn_rewrite']}",
              flush=True)

if __name__ == "__main__":
    Bridge().run()
