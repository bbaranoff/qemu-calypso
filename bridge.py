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

SERCOMM + IQ PATH (re-merged 2026-05-24)
----------------------------------------
Historiquement le wire BTS↔QEMU passait par PTY/sercomm DLCI 4 avec
samples I/Q GMSK-modulés (cf sercomm_udp.py). Ce wire a été remplacé
par un UDP direct vers la BSP avec soft-bits bruts pour simplifier,
mais le DSP correlator (FB-det Q15) attend des I/Q baseband, pas des
soft bits — d'où d_fb_det=0 chronique.

Cette version re-merge sercomm_udp.py dans bridge.py :
  - sercomm_wrap / SercommParser / soft_bits_to_gmsk_iq : inline
  - env BRIDGE_BSP_IQ=1   : convertit soft-bits → IQ int16 avant UDP send
                            vers BSP (calypso_bsp.c lit IQ samples)
  - env BRIDGE_PTY=/dev/pts/X : optionnel, wire en plus du UDP, envoie
                            DL bursts en sercomm DLCI 4 + IQ samples sur
                            cette PTY (DLCI 5 L1CTL reste géré par
                            osmocon comme avant)

WIRE MAP COURANT
----------------
  BTS osmo-bts-trx ─── UDP 5700-5702 ─── bridge.py ─── UDP 6702 ─── QEMU BSP
                                               │
                                               └─ (optionnel) PTY sercomm DLCI 4
                                                  → QEMU UART → calypso_trx.c
"""
import errno, os, select, signal, socket, struct, sys, threading, time
import fcntl, termios

GSM_HYPERFRAME = 2715648
GSM_TDMA_S = 4615 / 1_000_000  # 4.615 ms per TDMA frame, wall-clock

# === sercomm framing + IQ modulation (merged from sercomm_udp.py) ============
SERCOMM_FLAG = 0x7E
SERCOMM_ESCAPE = 0x7D
SERCOMM_XOR = 0x20
DLCI_L1CTL = 5
DLCI_BURST = 4


def sercomm_wrap(dlci, payload):
    """Encode payload as a sercomm HDLC frame (0x7E flag + 0x7D escape)."""
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
    """Streaming sercomm de-framer. feed(bytes) → [(dlci, payload), ...]."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        frames = []
        while True:
            try:
                s = self.buf.index(SERCOMM_FLAG)
            except ValueError:
                self.buf.clear()
                break
            if s > 0:
                del self.buf[:s]
            try:
                e = self.buf.index(SERCOMM_FLAG, 1)
            except ValueError:
                break
            raw = bytes(self.buf[1:e])
            del self.buf[:e + 1]
            if not raw:
                continue
            out = bytearray()
            i = 0
            while i < len(raw):
                if raw[i] == SERCOMM_ESCAPE and i + 1 < len(raw):
                    i += 1
                    out.append(raw[i] ^ SERCOMM_XOR)
                else:
                    out.append(raw[i])
                i += 1
            if len(out) >= 2:
                frames.append((out[0], bytes(out[2:])))
        return frames


def soft_to_int16(soft_bit):
    """Convert osmocom soft bit (0=strong 1, 255=strong 0) to int16."""
    return max(-32768, min(32767, int((127 - soft_bit) * 32767 / 127)))


def soft_bits_to_gmsk_iq(soft_bits_raw, scale=4096):
    """Convert TRXD soft bits → GMSK I/Q int16 baseband pairs for DSP BSP.
    Calypso ABB delivers I/Q complex pairs from antenna ; ici on reconstruit
    l'équivalent en GMSK-modulant les hard-bits (BT=0.3, h=0.5,
    1 sample/symbol). Sortie : N soft-bits → 2N int16 = 4N octets, ordre
    interleaved I,Q,I,Q,... (= ce que le BSP C attend pour memcpy direct).
    Lazy import numpy/scipy : si absents (env minimal) on génère le pattern
    hard ±π/2 équivalent au modulateur interne BSP (fallback identique)."""
    n = len(soft_bits_raw)
    try:
        import numpy as np
        from scipy.ndimage import gaussian_filter1d
    except ImportError:
        # Fallback : reproduit le pattern interne ±π/2 du BSP (cos_tab/sin_tab)
        cos_tab = (0x7FFE, 0, -0x7FFE, 0)
        sin_tab = (0, 0x7FFE, 0, -0x7FFE)
        phase_idx = 0
        out = bytearray()
        for sb in soft_bits_raw:
            out += int(cos_tab[phase_idx]).to_bytes(2, 'little', signed=True)
            out += int(sin_tab[phase_idx]).to_bytes(2, 'little', signed=True)
            phase_idx = (phase_idx + (3 if sb >= 128 else 1)) & 3
        return bytes(out)
    hard = np.array([0.0 if b < 128 else 1.0 for b in soft_bits_raw])
    nrz = 1.0 - 2.0 * hard
    bt = 0.3
    filtered = gaussian_filter1d(nrz, 1.0 / (2.0 * 3.141592653589793 * bt))
    phase = np.cumsum(filtered) * 3.141592653589793 * 0.5
    i_samp = np.clip(np.cos(phase) * scale, -32768, 32767).astype(np.int16)
    q_samp = np.clip(np.sin(phase) * scale, -32768, 32767).astype(np.int16)
    # Interleave I,Q,I,Q,... : 2N int16 = 4N bytes
    iq = np.empty(2 * n, dtype=np.int16)
    iq[0::2] = i_samp
    iq[1::2] = q_samp
    return iq.tobytes()


# Env gate : convert DL soft-bits → IQ samples before UDP send to BSP.
# OFF par défaut pour ne rien casser ; ON pour exercer le DSP correlator.
BSP_IQ_MODE = os.environ.get("BRIDGE_BSP_IQ", "0") == "1"
# Env gate : optional PTY wire (sercomm DLCI 4 bursts). Empty = disabled.
# Quand non vide, bridge ouvre cette PTY et envoie DL bursts en parallèle
# du UDP (DLCI 5 / L1CTL reste géré par osmocon comme avant).
PTY_PATH = os.environ.get("BRIDGE_PTY", "")

# CLK IND period in frames (default 102 = stock GSM TDMA spec).
# In debug runs where QEMU is slower than wall-clock, the bridge sends
# CLK IND at wall-clock pace using its own wall_fn counter — see CLOCK
# DOMAIN BRIDGE block above. CLK_IND_PERIOD just controls the cadence
# (every N wall-paced frames). Default 102 = standard GSM rate.
CLK_IND_PERIOD = int(os.environ.get("BRIDGE_CLK_PERIOD", "102"))
CLK_IND_WALL_S = (CLK_IND_PERIOD * 4615) / 1_000_000

QEMU_BSP_ADDR = ("127.0.0.1", 6702)

# RACH-allowed slots in the 51-multiframe TN=0 uplink for combined CCCH+SDCCH8
# (TS 45.002 §7 Table 3). Mobile L3 announces "S(lots) 115" = these slots when
# combined=yes. A burst whose FN%51 falls outside this set is discarded by
# osmo-bts-trx scheduler before the RACH detector runs.
RACH_SLOTS_COMBINED = (
    set(range(4, 11)) | set(range(14, 21)) |
    set(range(24, 31)) | set(range(34, 41)) | set(range(44, 51))
)

# CLK IND source mode. Two clock domains live on the wire:
#   - wall-clock (BTS scheduler expects ~235ms intervals at default period)
#   - qfn (QEMU TDMA tick, ~2× slower than wall in this build)
#
#   "0" / unset → CLK IND fn = wall_fn rounded to CLK_IND_PERIOD (default).
#                 BTS scheduler advances wall-paced. UL bursts must be
#                 wall-paced too or they land outside the BTS RACH window.
#   "1"          → CLK IND fn = qfn rounded to CLK_IND_PERIOD. BTS scheduler
#                 advances qemu-paced (slow). Pair with passthrough UL
#                 (BRIDGE_UL_FN_REWRITE=0) so UL bursts in qfn match BTS
#                 scheduler window. Risk: BTS may shutdown on "PC clock
#                 skew too high" if elapsed_fn between consecutive CLK INDs
#                 deviates too much from CLK_IND_PERIOD; mitigate by lowering
#                 BRIDGE_CLK_PERIOD (e.g., 51 → 26).
CLK_FROM_QEMU = os.environ.get("BRIDGE_CLK_FROM_QEMU", "0") == "1"

# UL FN rewrite mode. Three modes:
#
#   "1" / "slot" / unset (default)
#       Slot-aware rewrite: sent_fn = next FN ≥ wall_fn whose (% 51) is a
#       valid combined-CCCH RACH slot. BTS scheduler is wall-paced (CLK IND
#       wall_fn) so we tag bursts a few frames in BTS's near-future at a
#       slot where RACH detection is scheduled. BTS holds the burst briefly
#       then processes it when its scheduler reaches that FN.
#       Caveat: only RACH-correct for TN=0 during LU phase. Once SDCCH UL
#       kicks in (post-IMM_ASS), needs a content-aware variant.
#
#   "0"
#       Passthrough qfn. UL burst sent unchanged from QEMU. Tags the past
#       from BTS POV → BTS likely drops as stale. Useful as a discriminant
#       (e.g., to see what BTS error log says).
#
#   "naive"
#       Blind wall_fn rewrite (legacy behavior pre-2026-05-08). Sent_fn =
#       wall_fn rounded to nothing. ~60% of bursts land off-slot for
#       combined CCCH+SDCCH8. Kept for A/B comparison.
UL_FN_REWRITE_MODE = os.environ.get("BRIDGE_UL_FN_REWRITE", "1").lower()
if UL_FN_REWRITE_MODE in ("1", "slot", "slot-aware", "true"):
    UL_FN_REWRITE_MODE = "slot"
elif UL_FN_REWRITE_MODE in ("0", "off", "passthrough", "false"):
    UL_FN_REWRITE_MODE = "off"
elif UL_FN_REWRITE_MODE in ("naive", "wall", "blind"):
    UL_FN_REWRITE_MODE = "naive"
else:
    UL_FN_REWRITE_MODE = "slot"  # unrecognized → safe default

# DL FN rewrite mode (symmetric to UL). The clock-domain split makes
# `bts_fn` (wall-paced) drift ~50 frames/s ahead of QEMU's `qfn`. BSP's
# default match window is 64 frames so DL bursts get dropped in seconds.
# See RUN_SNAPSHOT_2026-05-08.md for the measured delta=15000+ frames.
#
#   "slot" / "1" / unset (default)
#       Rewrite bts_fn → smallest qfn ≥ self.fn whose (% 51) matches
#       (bts_fn % 51). Preserves the slot type within the 51-multiframe
#       so DSP demod still types BCCH/CCCH/SDCCH correctly. If no qfn
#       in the lookahead window matches, the burst is dropped (BCCH
#       repeats so this is recoverable).
#
#   "naive"
#       Rewrite bts_fn → self.fn (current qfn). Ignores slot type — DSP
#       demod will mis-type bursts. Useful only as A/B comparison.
#
#   "off"
#       Passthrough bts_fn unchanged. BSP will reject ~all bursts due
#       to FN mismatch when QEMU is slow vs wall.
DL_FN_REWRITE_MODE = os.environ.get("BRIDGE_DL_FN_REWRITE", "slot").lower()
if DL_FN_REWRITE_MODE in ("1", "slot", "slot-aware", "true"):
    DL_FN_REWRITE_MODE = "slot"
elif DL_FN_REWRITE_MODE in ("0", "off", "passthrough", "false"):
    DL_FN_REWRITE_MODE = "off"
elif DL_FN_REWRITE_MODE in ("naive",):
    DL_FN_REWRITE_MODE = "naive"
else:
    DL_FN_REWRITE_MODE = "slot"

# How many qfn frames in the future we're willing to tag a DL burst.
# Bounded above by BSP_FN_MATCH_WINDOW (default 64 in calypso_bsp.c)
# minus a safety margin. Default 32 → at most ~50% of the BSP window
# leaves headroom for queue jitter. With lookahead=51 every burst can
# be slot-mapped (since each %51 value occurs once per 51-frame
# window), but bursts get tagged up to 50 frames in the qfn future.
DL_FN_LOOKAHEAD = int(os.environ.get("BRIDGE_DL_FN_LOOKAHEAD", "32"))


def next_rach_slot_fn(fn):
    """Return smallest FN ≥ fn whose (FN % 51) ∈ RACH_SLOTS_COMBINED.
    Worst case scans 51 candidates, typically finds one within 4."""
    for delta in range(0, 52):
        if (fn + delta) % 51 in RACH_SLOTS_COMBINED:
            return fn + delta
    return fn  # unreachable since RACH_SLOTS_COMBINED covers 35/51 slots


def dl_slot_aware_qfn(bts_fn, current_qfn, lookahead):
    """Map bts_fn to the smallest qfn ≥ current_qfn whose (qfn % 51) matches
    (bts_fn % 51). Preserves slot type for DSP demod typing.

    Returns the target qfn, or None if no match exists within `lookahead`
    frames (caller drops the burst — BCCH repeats so this is recoverable).

    O(1): delta = ((bts_fn - current_qfn) mod 51) is the always-non-negative
    offset to the next matching qfn slot ; either ≤ lookahead or we drop.
    """
    target_mod = bts_fn % 51
    delta = (target_mod - (current_qfn % 51)) % 51
    if delta > lookahead:
        return None
    return current_qfn + delta

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

        # Optional PTY wire (sercomm DLCI 4 burst path, parallel to UDP).
        # Ouvert seulement si BRIDGE_PTY non vide. L1CTL DLCI 5 reste géré
        # par osmocon sur sa propre PTY ; ici on ouvre une PTY séparée pour
        # injecter les bursts au DSP via l'UART du Calypso quand on veut
        # exercer le path sercomm complet (init / debug DSP correlator).
        self.pty_fd = -1
        self.pty_parser = SercommParser()
        if PTY_PATH:
            self._open_pty(PTY_PATH)
        if BSP_IQ_MODE:
            print("bridge: BSP_IQ_MODE=on — DL soft-bits → GMSK IQ samples "
                  "before UDP send", flush=True)
        if PTY_PATH:
            print(f"bridge: PTY wire = {PTY_PATH} (sercomm DLCI 4 burst path)",
                  flush=True)

        # Wall-clock-paced FN counter — independent of QEMU. Anchored to
        # POWERON so BTS sees fn=0 right when it asks to power up.
        self._wall_t0 = None       # set at POWERON
        self._last_clk_fn_sent = None

        print(f"bridge: CLK={bts_base} TRXC={bts_base+1} TRXD={bts_base+2} ↔ BSP@6702", flush=True)
        clk_desc = ('hybrid (wall-paced emit, virtual_fn anchored to qfn)'
                    if CLK_FROM_QEMU else
                    'wall_fn (wall-paced 217 Hz)')
        print(f"bridge: CLK IND source={clk_desc}, "
              f"period={CLK_IND_PERIOD} frames", flush=True)
        ul_desc = {
            "slot": "slot-aware (next RACH slot ≥ wall_fn — combined CCCH+SDCCH8)",
            "naive": "naive (wall_fn rounded, ignores slot mask)",
            "off":  "off (passthrough qemu_fn)",
        }[UL_FN_REWRITE_MODE]
        dl_desc = {
            "slot": f"slot-aware (next qfn ≥ self.fn matching bts_fn%%51, lookahead={DL_FN_LOOKAHEAD})",
            "naive": "naive (rewrite to current qfn, ignores slot type)",
            "off":  "off (passthrough bts_fn — BSP will reject ~all bursts)",
        }[DL_FN_REWRITE_MODE]
        print(f"bridge: UL FN rewrite mode={UL_FN_REWRITE_MODE} — {ul_desc}",
              flush=True)
        print(f"bridge: DL FN rewrite mode={DL_FN_REWRITE_MODE} — {dl_desc}",
              flush=True)

    def _open_pty(self, path):
        """Open PTY in raw 8N1 115200 non-blocking for sercomm burst injection."""
        try:
            self.pty_fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            attrs = termios.tcgetattr(self.pty_fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 1
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.pty_fd, termios.TCSANOW, attrs)
            fcntl.fcntl(self.pty_fd, fcntl.F_SETFL,
                        fcntl.fcntl(self.pty_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        except OSError as e:
            print(f"bridge: PTY open {path} FAILED: {e} — PTY wire disabled",
                  flush=True)
            self.pty_fd = -1

    def _pty_send_burst(self, hdr, iq_payload):
        """Send a DL burst via sercomm DLCI 4 on the optional PTY."""
        if self.pty_fd < 0:
            return
        frame = sercomm_wrap(DLCI_BURST, bytes(hdr) + iq_payload)
        try:
            os.write(self.pty_fd, frame)
            self.stats.setdefault("pty_burst_tx", 0)
            self.stats["pty_burst_tx"] += 1
        except OSError as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                print(f"bridge: PTY write error: {e}", flush=True)

    def wall_fn(self):
        """Compute current bridge wall-paced FN. Anchored at POWERON."""
        if self._wall_t0 is None:
            return 0
        elapsed = time.monotonic() - self._wall_t0
        return int(elapsed / GSM_TDMA_S) % GSM_HYPERFRAME

    def handle_qemu_tick(self):
        """Receive TDMA tick from QEMU — 4 bytes big-endian FN.
        Used as telemetry only ; CLK IND is wall_fn-paced now.
        REFAC 2026-05-24 : appelé depuis _tick_thread (pas le main loop)
        pour ne pas introduire de jitter dans le burst path."""
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

    def _tick_loop(self):
        """Background thread : pump QEMU CLK ticks decoupled from main
        burst path. Évite que le handle_qemu_tick (217 Hz wall) injecte
        du jitter dans le pipeline UL/DL en occupant le select main."""
        while not self._stop:
            try:
                rd, _, _ = select.select([self.qemu_clk_sock], [], [], 0.05)
                if rd:
                    self.handle_qemu_tick()
            except Exception:
                if self._stop:
                    return
                time.sleep(0.01)

    def _trxc_loop(self):
        """Background thread : TRXC control plane (BTS POWERON/RXTUNE/TXTUNE,
        responses async). Pas dans le burst path critique — isolé pour ne
        pas bloquer trxd. Modèle similaire au split DSP/BSP côté QEMU :
        chaque domaine son thread, sync minimale par socket state."""
        while not self._stop:
            try:
                rd, _, _ = select.select([self.trxc_sock], [], [], 0.05)
                if rd:
                    self.handle_trxc()
            except Exception:
                if self._stop:
                    return
                time.sleep(0.01)

    def _stats_loop(self):
        """Background thread : print stats périodiques (5s) sans bloquer
        le main loop. Le log de stats reste utile pour drift bts↔qfn et
        compteurs dl/ul/drop, mais n'a pas besoin d'être inline."""
        last_dl_ul = 0
        while not self._stop:
            time.sleep(5.0)
            if self._stop:
                return
            total = self.stats["dl"] + self.stats["ul"]
            if total == 0 or total == last_dl_ul:
                continue
            last_dl_ul = total
            wfn = self.wall_fn()
            in_s  = self.stats.get("ul_in_slot", 0)
            off_s = self.stats.get("ul_off_slot", 0)
            dl_rw = self.stats.get("dl_rewrite", 0)
            dl_dr = self.stats.get("dl_drop_no_slot", 0)
            print(f"bridge: tick={self.stats['tick']} "
                  f"clk={self.stats['clk']} "
                  f"dl={self.stats['dl']} ul={self.stats['ul']} "
                  f"ul_rewrite={self.stats['ul_fn_rewrite']} "
                  f"ul_slot_in/off={in_s}/{off_s} "
                  f"dl_rewrite={dl_rw} dl_drop={dl_dr} "
                  f"wall_fn={wfn} qfn={self.fn}",
                  flush=True)

    def _send_clk_ind(self, clk_fn):
        try:
            self.clk_sock.sendto(
                f"IND CLOCK {clk_fn}\0".encode(), self.bts_clk_addr)
            self.stats["clk"] += 1
            self._last_clk_fn_sent = clk_fn
            if self.stats["clk"] <= 5 or (self.stats["clk"] % 200) == 0:
                src = "hybrid" if CLK_FROM_QEMU else "wall_fn"
                print(f"bridge: CLK IND #{self.stats['clk']} fn={clk_fn} ({src})",
                      flush=True)
        except OSError:
            pass

    def maybe_send_clk(self):
        """CLK IND scheduler. Three modes:

        wall-paced (default, CLK_FROM_QEMU=False)
          Send every CLK_IND_WALL_S seconds with clk_fn = wall_fn rounded
          to CLK_IND_PERIOD. BTS sees consistent ~235ms intervals.

        qfn-paced (CLK_FROM_QEMU=True, legacy "1")
          [deprecated when QEMU slow] Send each time qfn has advanced by
          CLK_IND_PERIOD since last send. clk_fn = qfn rounded down. BTS
          dies via clock-skew shutdown when QEMU rate << 217 Hz wall.
          Kept only as discriminant / for fast emulator builds.

        hybrid (CLK_FROM_QEMU=True, default since 2026-05-23)
          Wall-paced emission (BTS-friendly cadence) BUT clk_fn = virtual
          fn that ticks at 217 Hz wall rate, anchored to actual qfn at
          startup. BTS sees consistent ~235ms intervals AND fn rate
          matching its scheduler expectation. The slot-aware UL/DL FN
          rewrite still uses self.fn (qfn) for slot alignment, so DSP-side
          path is unaffected. Trade-off: virtual fn drifts from qfn as run
          progresses (QEMU lag accumulates), but bursts are slot-rewritten
          to match the virtual fn domain via existing logic.
        """
        if not self.powered:
            return

        now = time.monotonic()

        if CLK_FROM_QEMU:
            # Hybrid mode : wall-paced emit + qfn-anchored virtual fn
            if not hasattr(self, '_hybrid_anchor_qfn'):
                self._hybrid_anchor_qfn = self.fn
                self._hybrid_anchor_wall = now
                self._last_clk_wall = now - CLK_IND_WALL_S
            if now - self._last_clk_wall < CLK_IND_WALL_S:
                return
            self._last_clk_wall += CLK_IND_WALL_S
            if now - self._last_clk_wall > CLK_IND_WALL_S * 4:
                self._last_clk_wall = now
            # Virtual fn = anchor + wall_elapsed × 217 Hz. Monotonic ↑ ;
            # never snap backward (BTS would reject fn going backward).
            elapsed = now - self._hybrid_anchor_wall
            virtual_fn = (self._hybrid_anchor_qfn
                          + int(elapsed / GSM_TDMA_S)) % GSM_HYPERFRAME
            target = (virtual_fn // CLK_IND_PERIOD) * CLK_IND_PERIOD
            # Suppress duplicates : if virtual_fn hasn't crossed a new
            # PERIOD boundary since last send, skip — BTS computes
            # elapsed_fn=0 between identical fn values and triggers
            # "GSM clock jitter" / "No clock since TRX started" shutdown.
            if self._last_clk_fn_sent is not None and target == self._last_clk_fn_sent:
                return
            self._send_clk_ind(target)
            return

        if not hasattr(self, '_last_clk_wall'):
            self._last_clk_wall = now - CLK_IND_WALL_S  # force first send
        if now - self._last_clk_wall < CLK_IND_WALL_S:
            return
        self._last_clk_wall += CLK_IND_WALL_S
        # Catch up if we slipped a long time (avoid runaway send burst)
        if now - self._last_clk_wall > CLK_IND_WALL_S * 4:
            self._last_clk_wall = now
        wfn = self.wall_fn()
        self._send_clk_ind((wfn // CLK_IND_PERIOD) * CLK_IND_PERIOD)

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
            # CRITICAL: bridge always emits TRXDv0 (8-byte header + 148 soft
            # bits = 156 bytes). If BTS asks for v1 and we echo "agreed v1",
            # BTS parses our v0 packets with v1 layout → silent drop before
            # RACH detector. Force v0 in the response regardless of request.
            requested = args[0] if args else "0"
            rsp = "RSP SETFORMAT 0 0 0"  # status=0 (ok), accepted=0, available=0 (v0)
            print(f"TRXC SETFORMAT requested=v{requested} → forced reply v0 "
                  f"(bridge sends TRXDv0 only)", flush=True)
        elif verb == "NOMTXPOWER":
            rsp = "RSP NOMTXPOWER 0 50"
        elif verb == "MEASURE":
            freq = args[0] if args else "0"
            rsp = f"RSP MEASURE 0 {freq} -60"
        else:
            rsp = f"RSP {verb} 0 {' '.join(args)}".rstrip()

        # Log all TRXC exchanges so init-time negotiation is visible
        if self.stats["trxc"] <= 50 or verb in ("POWERON", "POWEROFF"):
            print(f"TRXC < CMD {verb} {' '.join(args)}".rstrip()
                  + f"  →  > {rsp}", flush=True)

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

        # FN rewrite (or passthrough) per UL_FN_REWRITE_MODE.
        # The actual FN that goes to BTS is the one that matters for the
        # slot-validity diagnostic.
        if not self.powered or UL_FN_REWRITE_MODE == "off":
            sent_fn = qemu_fn
        elif UL_FN_REWRITE_MODE == "naive":
            sent_fn = self.wall_fn()
        else:  # "slot" — default
            sent_fn = next_rach_slot_fn(self.wall_fn())
        out = bytearray(data)
        out[1] = (sent_fn >> 24) & 0xFF
        out[2] = (sent_fn >> 16) & 0xFF
        out[3] = (sent_fn >>  8) & 0xFF
        out[4] =  sent_fn        & 0xFF
        if sent_fn != qemu_fn:
            self.stats["ul_fn_rewrite"] += 1

        # Slot-validity diagnostic: is sent_fn % 51 a valid RACH slot for
        # combined CCCH+SDCCH8 (the only mode mobile knows about)? Counters
        # printed in the rolling stats line so the distribution is visible.
        slot_mod51 = sent_fn % 51
        in_rach_slot = slot_mod51 in RACH_SLOTS_COMBINED
        if in_rach_slot:
            self.stats.setdefault("ul_in_slot", 0)
            self.stats["ul_in_slot"] += 1
        else:
            self.stats.setdefault("ul_off_slot", 0)
            self.stats["ul_off_slot"] += 1

        # Print full header + first/last bits of every UL burst (cap 200 to
        # avoid log flood). Show both FNs + slot validity to track the rewrite.
        if self.stats["ul"] <= 200 or (self.stats["ul"] % 1000) == 0:
            hdr_in_hex  = data[:8].hex()
            hdr_out_hex = bytes(out[:8]).hex()
            payload = data[8:]
            head = ' '.join(f"{b - 256 if b >= 128 else b:+d}" for b in payload[:16])
            tail = ' '.join(f"{b - 256 if b >= 128 else b:+d}" for b in payload[-8:])
            slot_mark = "RACH" if in_rach_slot else "OFF"
            arrow = "→" if sent_fn != qemu_fn else "="
            print(f"bridge: UL #{self.stats['ul']} TN={tn} "
                  f"qfn={qemu_fn}{arrow}sent={sent_fn} slot={slot_mod51:02d}/51={slot_mark} "
                  f"rssi=-{rssi} toa={toa} len={len(data)} "
                  f"hdr_in={hdr_in_hex} hdr_out={hdr_out_hex} "
                  f"bits[0:16]=[{head}] bits[140:148]=[{tail}] "
                  f"→ BTS {self.trxd_remote}", flush=True)

        try:
            self.trxd_sock.sendto(bytes(out), self.trxd_remote)
        except OSError as e:
            print(f"bridge: UL send error: {e}", flush=True)

    def _handle_dl(self, data, addr):
        """DL burst from BTS → forward to QEMU BSP, slot-aware FN-rewritten.

        Format (TRXDv0 DL, 154 bytes total) :
          [0]    TN
          [1:5]  FN (BE)  — REWRITTEN to a near-future qfn that preserves
                            (FN % 51) so DSP demod still types BCCH/CCCH
                            correctly. Burst dropped if no match within
                            DL_FN_LOOKAHEAD frames (BCCH repeats).
          [5]    RSSI / format flags (TRXDv0 omits ToA on DL, header is 6 B)
          [6:]   148 soft bits

        WHY: BTS scheduler runs at wall-clock rate, QEMU BSP at qfn (~half).
        delta=bts_fn-qfn grows ~50 frames/s and quickly exceeds BSP's match
        window (default 64 frames in calypso_bsp.c). Without rewrite, BSP
        rejects 100 % of bursts, DSP CCCH demod never runs, mobile L3 never
        receives SI, mobile stays in cell-search forever.
        """
        self.trxd_remote = addr
        self.stats["dl"] += 1

        tn = data[0] & 0x07
        bts_fn = int.from_bytes(data[1:5], 'big')

        # FN rewrite per DL_FN_REWRITE_MODE
        if DL_FN_REWRITE_MODE == "off" or self.fn == 0:
            sent_fn = bts_fn
            drop = False
        elif DL_FN_REWRITE_MODE == "naive":
            sent_fn = self.fn
            drop = False
        else:  # "slot" — default
            sent_fn = dl_slot_aware_qfn(bts_fn, self.fn, DL_FN_LOOKAHEAD)
            drop = sent_fn is None

        if drop:
            self.stats.setdefault("dl_drop_no_slot", 0)
            self.stats["dl_drop_no_slot"] += 1
            # Log first few drops + every 1000th so the pattern is visible
            n = self.stats["dl_drop_no_slot"]
            if n <= 5 or n % 1000 == 0:
                bts_mod = bts_fn % 51
                qfn_mod = self.fn % 51
                delta_to_match = (bts_mod - qfn_mod) % 51
                print(f"bridge: DL drop #{n} TN={tn} bts_fn={bts_fn} (mod {bts_mod}) "
                      f"qfn={self.fn} (mod {qfn_mod}) delta_to_match={delta_to_match} "
                      f"> lookahead={DL_FN_LOOKAHEAD}", flush=True)
            return

        if sent_fn != bts_fn:
            self.stats.setdefault("dl_rewrite", 0)
            self.stats["dl_rewrite"] += 1

        out = bytearray(data)
        out[1] = (sent_fn >> 24) & 0xFF
        out[2] = (sent_fn >> 16) & 0xFF
        out[3] = (sent_fn >>  8) & 0xFF
        out[4] =  sent_fn        & 0xFF

        # Log burst content: first 8 data bytes + check if FB (all zeros)
        hdr_bytes = bytes(out[:8]) if len(out) >= 8 else bytes(out)
        payload = data[8:] if len(data) > 8 else b''
        is_fb = all(b == 0 for b in payload) if payload else False
        if self.stats["dl"] <= 10 or self.stats["dl"] % 5000 == 0 or is_fb:
            arrow = "→" if sent_fn != bts_fn else "="
            print(f"bridge: DL #{self.stats['dl']} TN={tn} "
                  f"bts_fn={bts_fn}{arrow}qfn={sent_fn} (cur_qfn={self.fn}, anchor={self.fn_anchor}) "
                  f"len={len(out)} hdr={hdr_bytes[:8].hex()} "
                  f"bits[0:8]={list(payload[:8])} "
                  f"{'*** FB ***' if is_fb else ''}", flush=True)

        # Optional IQ conversion : soft-bits → GMSK I/Q int16. Le BSP
        # côté QEMU doit savoir interpréter IQ (le code C lit toujours
        # soft-bits par défaut ; activer BRIDGE_BSP_IQ après avoir
        # adapté calypso_bsp.c side, sinon les bursts deviennent garbage).
        if BSP_IQ_MODE and payload:
            iq_bytes = soft_bits_to_gmsk_iq(payload)
            # Pour N soft bits : 2N int16 = 4N bytes IQ payload (interleaved
            # I,Q,I,Q,...). Total UDP = 8 hdr + 4N. Pour 148 bits = 600 B,
            # pour 146 bits (BTS truncate) = 592 B. BSP attend iq_bytes >= 4*nbits.
            udp_out = bytes(out[:8]) + iq_bytes
        else:
            udp_out = bytes(out)

        try:
            self.trxd_sock.sendto(udp_out, QEMU_BSP_ADDR)
        except OSError as e:
            print(f"bridge: DL send error: {e}", flush=True)

        # Optional PTY wire (sercomm DLCI 4 burst, parallel to UDP).
        # Toujours IQ-modulé sur PTY car le path UART est le wire
        # historique du Calypso réel.
        if self.pty_fd >= 0 and payload:
            iq_for_pty = soft_bits_to_gmsk_iq(payload) if not BSP_IQ_MODE else iq_bytes
            self._pty_send_burst(out[:8], iq_for_pty)

    def run(self):
        running = True
        def shutdown(s, f):
            nonlocal running; running = False
        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        # REFAC 2026-05-24 : 3 threads isolés du burst path pour timing
        # propre. Avant : tout dans 1 select → tick handler 217 Hz wall
        # + stats print + trxc control = jitter sur trxd (burst data).
        #   _tick_thread   : qemu_clk_sock (CLK ticks QEMU → telemetry)
        #   _trxc_thread   : trxc_sock (BTS POWERON/RXTUNE control plane)
        #   _stats_thread  : print stats périodiques (5s)
        # Main loop = SEULEMENT trxd_sock (burst DL/UL = critical path)
        #             + maybe_send_clk (CLK IND vers BTS, low jitter requis).
        self._tick_thread = threading.Thread(
            target=self._tick_loop, daemon=True, name="bridge-tick")
        self._tick_thread.start()
        self._trxc_thread = threading.Thread(
            target=self._trxc_loop, daemon=True, name="bridge-trxc")
        self._trxc_thread.start()
        self._stats_thread = threading.Thread(
            target=self._stats_loop, daemon=True, name="bridge-stats")
        self._stats_thread.start()

        # Burst-only main loop : pas de jitter d'autres sockets.
        while running:
            try:
                # Timeout 1ms — réduit jitter CLK IND vu par osmo-bts.
                readable, _, _ = select.select([self.trxd_sock], [], [], 0.001)
            except (OSError, ValueError) as e:
                print(f"bridge: select error: {e}", flush=True)
                break

            if self.trxd_sock in readable: self.handle_trxd()

            # CLK IND emit (wall-paced, BTS-friendly cadence)
            self.maybe_send_clk()

        self._stop = True
        print(f"bridge: tick={self.stats['tick']} clk={self.stats['clk']} "
              f"trxc={self.stats['trxc']} dl={self.stats['dl']} "
              f"ul={self.stats['ul']} ul_rewrite={self.stats['ul_fn_rewrite']}",
              flush=True)

if __name__ == "__main__":
    Bridge().run()
