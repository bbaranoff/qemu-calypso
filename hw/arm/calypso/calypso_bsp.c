/*
 * Calypso BSP/RIF DMA module — implementation.
 *
 * On real hardware the BSP (Baseband Serial Port) is a synchronous serial
 * link that DMA-feeds I/Q samples from the IOTA RF frontend into C54x
 * DSP DARAM. The DSP code (FB/SB/burst detection in PROM0) reads them
 * from a fixed DARAM buffer and posts results into the NDB.
 *
 * In QEMU, DL bursts arrive via UDP (TRXDv0 from bridge.py on port 5702).
 * This module owns that socket, decodes the TRXDv0 header, converts hard
 * bits to I/Q samples, and DMA-writes them into DSP DARAM.
 *
 * L1CTL control (DLCI 5) goes through the UART — bursts never touch UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>   /* inet_aton */
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include "qemu/timer.h"
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "calypso_tint0.h"  /* GSM_HYPERFRAME */

/* calypso_trx_get_fn now provided by calypso_trx.h (included above). */

/* Forward decls for env-gated helpers used in calypso_bsp_init pre-warm. */
static uint32_t d_rach_word_offset(void);
static int rach_force_bsic(void);

#define BSP_LOG(fmt, ...) \
    do { fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

#define BSP_TRXD_PORT  6702   /* bridge forwards DL bursts here (5702 is bridge's own) */

/* Per-TN burst queue: FN-indexed ring so lookahead bursts from the BTS
 * (osmo-bts-trx schedules up to ~92 frames ahead) are preserved until the
 * QEMU virtual FN catches up to each burst's scheduled FN. On real hardware
 * BSP DMA is synchronous within the TDMA frame; in QEMU bursts arrive over
 * UDP from the bridge with their scheduled FN embedded in the TRXD header,
 * and delivery must happen at the exact virtual FN or the DSP correlates
 * samples against a frame boundary that does not match the modulator phase
 * (→ d_fb_det stays 0 indefinitely). */
#define BSP_NUM_TN     8                 /* one queue per timeslot */
#define BSP_QUEUE_LEN  128               /* lookahead depth per TN */
/* Match window: real BSP captures samples around BDLENA; exact FN match
 * is a QEMU artefact. ±4 frames tolerates bridge/BTS CLK IND jitter and
 * is narrow enough not to swap adjacent FCCH with non-FCCH in the 51-
 * multiframe pattern (FCCH appears every 10 frames on the BCCH slot). */
/* Was 4: too tight for BTS scheduler lookahead (observed delta=1..139 with
 * mean ~50). 99 % of bursts went stale before the QEMU virtual FN caught up.
 * 64 covers the typical lookahead and lets the queue drain fast enough that
 * BDLENA pulses actually consume bursts. (Bumped to 1024 in diag 2026-04-26
 * — confirmed not the bottleneck. Restored to 64.) */
#define BSP_FN_MATCH_WINDOW  64

/* === DARAM write-by-range instrumentation (2026-05-14) ===
 *
 * Plages observées dans le rapport 05-14 + finding 100% match PROM0 :
 *   low    : DARAM zone lue par AR3 stride +19 dans le correlator FB-det
 *   target : zone cible CALYPSO_BSP_DARAM_ADDR par défaut (0x3FB0..)
 *   wrap   : zone wrap circulaire AR2/AR7 BK=176 stride -19
 *   other  : ailleurs (incluant débord daram_len=296 vers [0x4000..0x40D7])
 *
 * Une stat tranche 3 hypothèses sur la priorité A :
 *   low=0 ET target=0 ET wrap=0 → BSP DMA jamais armée (bug amont TPU/INTH)
 *   target>>0 ET low=0          → BSP écrit mais env var ignorée
 *   low>>0                       → BSP écrit où il faut, mismatch est en contenu/timing
 *
 * Exposé via log line `[BSP] DARAM-WR-STATS ...` toutes les 1000 writes,
 * parseable par le harnais pytest (test_bsp_daram_write_distribution). */
#define BSP_BUCKET_LOW_LO     0x0000
#define BSP_BUCKET_LOW_HI     0x03A3
#define BSP_BUCKET_TARGET_LO  0x3FB0
#define BSP_BUCKET_TARGET_HI  0x3FFF
#define BSP_BUCKET_WRAP_LO    0xFC5D
#define BSP_BUCKET_WRAP_HI    0xFFED
#define BSP_DARAM_WR_LOG_EVERY 1000

typedef struct {
    int16_t  iq[296];  /* 148 I/Q pairs max */
    int      n;        /* number of int16 values */
    uint32_t fn;
    bool     valid;
} BspBurstSlot;

typedef struct {
    BspBurstSlot  slot[BSP_QUEUE_LEN];
} BspBurstQueue;

static struct {
    C54xState *dsp;
    uint16_t   daram_addr;
    uint16_t   daram_len;
    uint16_t   bypass_bdlena;
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
    uint64_t   bursts_dropped_no_window;
    uint64_t   bursts_dropped_queue_full;
    uint64_t   bursts_dropped_stale;
    int        trxd_fd;            /* UDP socket for TRXDv0 DL bursts */
    struct sockaddr_in trxd_peer;  /* BTS address (for UL replies) */
    bool       trxd_peer_valid;
    uint8_t    last_att;           /* last DL attenuation byte */

    /* FN-indexed queue per TN */
    BspBurstQueue  q[BSP_NUM_TN];

    /* DARAM write-by-range counters (cf. BSP_BUCKET_* + 2026-05-14 plan). */
    uint64_t   wr_low;
    uint64_t   wr_target;
    uint64_t   wr_wrap;
    uint64_t   wr_other;
    uint64_t   wr_total;
    uint64_t   wr_last_logged;

    /* Virtual-clock drain timer (revised 2026-05-24 PM): decouple BSP→DSP
     * DMA delivery from tdma_tick (which can be slow under icount=auto),
     * but stay on QEMU_CLOCK_VIRTUAL so BSP and ARM cur_fn share the same
     * time domain. Previously on REALTIME → drift ~1300 fr in 6 s wall
     * vs ARM (BTS livré au rythme wall, ARM compté au rythme icount). */
    QEMUTimer *drain_timer;
} bsp;

#define BSP_DRAIN_PERIOD_MS  5
/* 2026-05-24 fix drift BTS↔L1 : BSP drain timer passe REALTIME → VIRTUAL pour
 * tourner sur la même horloge qu'ARM fn (via TINT0) et tdma_tick. Avec
 * icount=auto, REALTIME avance ~9% plus vite que VIRTUAL → drift cumulatif
 * (~1300 fr / 6 sec wall observé, "1 seconde d'écart BTS↔L1"). NS variant pour
 * appairage avec QEMU_CLOCK_VIRTUAL (timer_new_ns / qemu_clock_get_ns). */
#define BSP_DRAIN_PERIOD_NS  (BSP_DRAIN_PERIOD_MS * 1000000ULL)

/* Incrémente le bucket selon `addr` puis émet une ligne stats périodiquement.
 * Appelé à chaque write DARAM côté BSP (rx_burst direct + deliver_buffered). */
static inline void bsp_daram_wr_bucket(uint16_t addr)
{
    bsp.wr_total++;
    if (addr <= BSP_BUCKET_LOW_HI) {
        bsp.wr_low++;
    } else if (addr >= BSP_BUCKET_TARGET_LO && addr <= BSP_BUCKET_TARGET_HI) {
        bsp.wr_target++;
    } else if (addr >= BSP_BUCKET_WRAP_LO && addr <= BSP_BUCKET_WRAP_HI) {
        bsp.wr_wrap++;
    } else {
        bsp.wr_other++;
    }
    if (bsp.wr_total - bsp.wr_last_logged >= BSP_DARAM_WR_LOG_EVERY) {
        bsp.wr_last_logged = bsp.wr_total;
        BSP_LOG("DARAM-WR-STATS low=%llu target=%llu wrap=%llu other=%llu total=%llu",
                (unsigned long long)bsp.wr_low,
                (unsigned long long)bsp.wr_target,
                (unsigned long long)bsp.wr_wrap,
                (unsigned long long)bsp.wr_other,
                (unsigned long long)bsp.wr_total);
    }
}

/* Signed hyperframe distance (entry_fn - reference_fn) in (-H/2, H/2]. */
static int32_t bsp_fn_delta(uint32_t entry_fn, uint32_t ref_fn)
{
    int32_t d = (int32_t)entry_fn - (int32_t)ref_fn;
    if (d > (int32_t)(GSM_HYPERFRAME / 2))       d -= GSM_HYPERFRAME;
    else if (d <= -(int32_t)(GSM_HYPERFRAME / 2)) d += GSM_HYPERFRAME;
    return d;
}

/* Enqueue a burst into queue[tn]. If a slot already carries the same FN,
 * overwrite it (duplicate retransmission from BTS). If the queue is full,
 * drop the oldest entry (smallest fn_delta relative to enqueue). */
static void bsp_enqueue(uint8_t tn, uint32_t fn, const int16_t *iq, int n)
{
    if (tn >= BSP_NUM_TN) return;
    BspBurstQueue *qq = &bsp.q[tn];

    int free_idx = -1;
    int oldest_idx = 0;
    int32_t oldest_delta = INT32_MAX;

    for (int i = 0; i < BSP_QUEUE_LEN; i++) {
        BspBurstSlot *s = &qq->slot[i];
        if (s->valid && s->fn == fn) {
            memcpy(s->iq, iq, n * sizeof(int16_t));
            s->n = n;
            return;
        }
        if (!s->valid) {
            if (free_idx < 0) free_idx = i;
        } else {
            int32_t d = bsp_fn_delta(s->fn, fn);
            if (d < oldest_delta) { oldest_delta = d; oldest_idx = i; }
        }
    }

    int idx;
    if (free_idx >= 0) {
        idx = free_idx;
    } else {
        idx = oldest_idx;
        bsp.bursts_dropped_queue_full++;
    }
    BspBurstSlot *s = &qq->slot[idx];
    memcpy(s->iq, iq, n * sizeof(int16_t));
    s->n = n;
    s->fn = fn;
    s->valid = true;
}

/* Purge entries older than the match window and return the slot whose FN
 * is closest to current_fn (within ±BSP_FN_MATCH_WINDOW) for this TN, or
 * NULL if none. Future bursts beyond the window stay queued. */
static BspBurstSlot *bsp_take_for_fn(uint8_t tn, uint32_t current_fn)
{
    if (tn >= BSP_NUM_TN) return NULL;
    BspBurstQueue *qq = &bsp.q[tn];
    BspBurstSlot *match = NULL;
    int32_t best_abs = INT32_MAX;

    for (int i = 0; i < BSP_QUEUE_LEN; i++) {
        BspBurstSlot *s = &qq->slot[i];
        if (!s->valid) continue;
        int32_t d = bsp_fn_delta(s->fn, current_fn);
        int32_t ad = d < 0 ? -d : d;
        if (d < -BSP_FN_MATCH_WINDOW) {
            s->valid = false;
            bsp.bursts_dropped_stale++;
        } else if (ad <= BSP_FN_MATCH_WINDOW && ad < best_abs) {
            match = s;
            best_abs = ad;
        }
    }
    /* Periodic stale ratio summary: a runaway ratio (e.g. 7000:1) is the
     * symptom of a stalled DSP — virtual fn isn't catching up to queued
     * burst FNs before the match window expires. Report every 5000 stales
     * so the spiral is visible without flooding the log. */
    {
        static uint64_t last_logged_stale;
        if (bsp.bursts_dropped_stale - last_logged_stale >= 5000) {
            last_logged_stale = bsp.bursts_dropped_stale;
            BSP_LOG("STALE ratio: stale=%llu written=%llu (cur_fn=%u)",
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_written,
                    current_fn);
        }
    }
    return match;
}

static uint16_t parse_uint_env(const char *name, uint16_t def)
{
    const char *v = getenv(name);
    if (!v || !*v) return def;
    return (uint16_t)strtoul(v, NULL, 0);
}

uint16_t calypso_bsp_get_daram_addr(void) { return bsp.daram_addr; }
uint16_t calypso_bsp_get_daram_len(void)  { return bsp.daram_len; }
uint8_t  calypso_bsp_get_last_att(void)   { return bsp.last_att; }

/* ---- UDP TRXDv0 DL receive callback ---- */

static void bsp_trxd_readable(void *opaque)
{
    uint8_t buf[512];
    struct sockaddr_in addr;
    socklen_t alen = sizeof(addr);

    ssize_t n = recvfrom(bsp.trxd_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&addr, &alen);
    if (n < 8) return;

    /* Refine UL peer to actual DL sender (init-time default is bridge
     * 127.0.0.1:5702 — DL source confirms it or replaces it). */
    if (addr.sin_addr.s_addr != bsp.trxd_peer.sin_addr.s_addr ||
        addr.sin_port != bsp.trxd_peer.sin_port) {
        bsp.trxd_peer = addr;
        BSP_LOG("TRXD peer learned: %s:%d",
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    }

    /* TRXDv0 DL: tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes.
     * (Confirmed empirically 2026-05-07 — earlier "asymmetric 6-byte
     * header" hypothesis was wrong : RX header IS 8 bytes like TX.
     * Even when BTS emits 154-byte packets, the n-8 skip + 148-bit
     * clamp keeps DSP demod aligned. n-6 broke mobile L1 sync —
     * mobile stayed in cell-selection loop and never reached LU.) */
    uint8_t  tn  = buf[0] & 0x07;
    uint32_t fn  = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|
                   ((uint32_t)buf[3]<<8)|buf[4];
    bsp.last_att = (n > 5) ? buf[5] : 0;

    int nbits = (int)n - 8;  /* TRXDv0 header is 8 bytes (TS+FN+RSSI+ToA) */
    if (nbits > 148) nbits = 148;
    if (nbits <= 0) return;

    const uint8_t *bits = buf + 8;

    /* Log burst type: check if all-zero (FB) or mixed (NB/SB) */
    {
        int zeros = 0, ones = 0;
        for (int i = 0; i < nbits; i++) {
            if (bits[i] == 0) zeros++;
            else ones++;
        }
        static int burst_log = 0;
        if (burst_log < 20 || (burst_log % 10000) == 0) {
            BSP_LOG("BURST fn=%u tn=%u zeros=%d ones=%d %s",
                    fn, tn, zeros, ones,
                    zeros == nbits ? "*** FB ***" :
                    ones > 100 ? "DUMMY/NB" : "SB/OTHER");
        }
        burst_log++;
    }

    /* FN-alignment instrumentation: measure burst arrival FN vs QEMU
     * virtual FN. A persistent negative delta means BTS is lagging
     * (bursts arrive for FNs that have already passed); a positive
     * delta is normal lookahead. */
    {
        uint32_t cur_fn = calypso_trx_get_fn();
        int32_t  delta  = bsp_fn_delta(fn, cur_fn);

        static int rx_log = 0;
        if (rx_log < 100 || (rx_log % 1000) == 0) {
            BSP_LOG("RX tn=%u fn=%u cur_fn=%u delta=%d",
                    tn, fn, cur_fn, delta);
        }
        rx_log++;

        /* Rolling summary over last 500 samples: min/max/mean */
        static int32_t  hist[500];
        static unsigned hist_pos = 0;
        static unsigned hist_seen = 0;
        hist[hist_pos] = delta;
        hist_pos = (hist_pos + 1) % 500;
        hist_seen++;
        if ((hist_seen % 500) == 0) {
            unsigned nh = hist_seen < 500 ? hist_seen : 500;
            int32_t mn = INT32_MAX, mx = INT32_MIN;
            int64_t sum = 0;
            for (unsigned i = 0; i < nh; i++) {
                int32_t d = hist[i];
                if (d < mn) mn = d;
                if (d > mx) mx = d;
                sum += d;
            }
            BSP_LOG("RX delta stats (last %u): min=%d max=%d mean=%lld",
                    nh, mn, mx, (long long)(sum / (int64_t)nh));
        }
    }

    /* GMSK modulation: convert TRXDv0 hard bits to I/Q samples.
     * GMSK with h=0.5: each bit adds ±π/2 to the phase.
     * NRZ encoding: bit 0 → phase += π/2, bit 1 → phase -= π/2.
     *
     * The Calypso IOTA chip delivers complex I/Q pairs to the BSP.
     * Phase increments are exactly ±π/2, so I=cos(φ) and Q=sin(φ)
     * cycle through {±1, 0}. We produce interleaved I,Q pairs.
     *
     * For FB (all-zero bits): phase advances π/2 per bit → pure tone.
     * I/Q sequence: (1,0),(0,1),(-1,0),(0,-1),(1,0),...
     *
     * IQ PASSTHROUGH (2026-05-24) : si le payload UDP fait >= 296 octets
     * et que CALYPSO_BSP_IQ_PASSTHROUGH=1, on interprète buf[8..] comme
     * int16 IQ pairs LE (bridge.py BRIDGE_BSP_IQ=1 envoie ce format,
     * GMSK-modulé scipy BT=0.3 réaliste vs notre ±π/2 hard-modulation).
     * Sinon : modulation interne historique (148 hard-bits → 296 int16). */
    int16_t iq[296];  /* 148 I/Q pairs = 296 values */
    int iq_count = 0;

    static int iq_pt_mode = -1;
    if (iq_pt_mode < 0) {
        const char *e = getenv("CALYPSO_BSP_IQ_PASSTHROUGH");
        iq_pt_mode = (e && *e == '1') ? 1 : 0;
        BSP_LOG("IQ_PASSTHROUGH=%d", iq_pt_mode);
    }
    int iq_bytes = (int)n - 8;  /* payload bytes after 8-byte hdr */
    /* Bridge envoie 2 int16 par soft-bit (I,Q interleaved). Pour 148 bits
     * = 296 int16 = 592 octets. Mais BTS peut envoyer 145..148 bits selon
     * format → on accepte tout payload >= 2*nbits*sizeof(int16_t). */
    int need_iq_bytes = 2 * nbits * (int)sizeof(int16_t);  /* = 4 × nbits */
    if (iq_pt_mode && iq_bytes >= need_iq_bytes) {
        /* Bridge pre-modulated path : copy 2*nbits I/Q values directly.
         * Bytes are int16 LE on x86 host = same as int16_t native. */
        int copy_count = 2 * nbits;
        if (copy_count > 296) copy_count = 296;
        memcpy(iq, buf + 8, copy_count * sizeof(int16_t));
        iq_count = copy_count;
        static int pt_log = 0;
        if (pt_log < 10 || (pt_log % 5000) == 0) {
            BSP_LOG("IQ passthrough #%d fn=%u tn=%u bytes=%d need=%d nbits=%d "
                    "iq[0..3]=%d,%d,%d,%d",
                    pt_log, fn, tn, iq_bytes, need_iq_bytes, nbits,
                    iq[0], iq[1], iq[2], iq[3]);
        }
        pt_log++;
    } else {
        /* Q15 full-scale amplitude: real BSP/IOTA delivers near-full-range Q15
         * samples. ±0x7FFE keeps one bit of headroom below INT16_MIN. */
        static const int16_t cos_tab[4] = { 0x7FFE, 0, -0x7FFE, 0 };
        static const int16_t sin_tab[4] = { 0, 0x7FFE, 0, -0x7FFE };
        int phase_idx = 0;
        for (int i = 0; i < nbits; i++) {
            /* Anomaly A fix (2026-05-08) : émettre AVANT advance, donc le premier
             * sample est à phase=0 au lieu de phase=π/2. Le code original
             * advance-then-emit décalait tout le burst de 90°, faisant que la
             * corrélation cohérente du DSP correlator tombait dans la partie
             * quadrature au lieu d'in-phase → d_fb_det principalement négatif
             * (pattern observé : +23k, +20k occasionnel puis 4× -5k consécutifs).
             * À valider sur le prochain run. */
            iq[iq_count++] = cos_tab[phase_idx];  /* I — phase_idx avant advance */
            iq[iq_count++] = sin_tab[phase_idx];  /* Q */
            phase_idx = (phase_idx + (bits[i] ? 3 : 1)) & 3;
        }
    }

    /* Enqueue the burst FN-indexed for this TN. With BTS lookahead of up
     * to ~92 frames, several bursts are in flight at once; each must be
     * delivered at the exact QEMU virtual FN it was scheduled for, or
     * the DSP correlator runs against incoherent samples. */
    bsp_enqueue(tn, fn, iq, iq_count);

    /* Delivery is handled exclusively by calypso_bsp_deliver_buffered()
     * called from the TDMA tick. No immediate delivery — it would
     * double-consume BDLENA pulses and race with the buffered path. */
}

/* ---- Init ---- */

/* Virtual-clock drain callback: pulls BSP UDP queue into DSP DMA at virtual
 * rate, locked to the same QEMU_CLOCK_VIRTUAL as TINT0/tdma_tick/ARM fn.
 * Pre-2026-05-24 this timer ran on QEMU_CLOCK_REALTIME, which caused a
 * cumulative drift vs ARM cur_fn under icount=auto (BTS delta grew ~1300 fr
 * in 6 s wall). Locking to VIRTUAL eliminates the drift at the source. */
static void bsp_drain_cb(void *opaque)
{
    if (bsp.dsp) {
        uint32_t cur_fn = calypso_trx_get_fn();
        calypso_bsp_deliver_buffered(cur_fn);
    }
    timer_mod(bsp.drain_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BSP_DRAIN_PERIOD_NS);
}

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp = dsp;
    /* DSP reads I/Q at DARAM 0x3fb3-0x3fbe (verified via DARAM RD HIST).
     * 0x3fc0 was off by 13 words — DSP saw zeros and never advanced past
     * the FB-det wait loop at PROM0 0x7700. */
    bsp.daram_addr     = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0x3fb0);
    bsp.daram_len      = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  296);
    bsp.bypass_bdlena  = parse_uint_env("CALYPSO_BSP_BYPASS_BDLENA", 0);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    bsp.bursts_dropped_no_window = 0;
    bsp.bursts_dropped_queue_full = 0;
    bsp.bursts_dropped_stale = 0;
    memset(bsp.q, 0, sizeof(bsp.q));
    bsp.trxd_fd = -1;
    /* Pre-set UL peer to bridge default (TRXDv0 listener on 127.0.0.1:5702).
     * Eliminates the race where ARM/DSP fires the first UL burst before any
     * DL has arrived to learn the peer addr. The peer is refined to the
     * actual sender on first DL receive (bsp_trxd_readable). */
    memset(&bsp.trxd_peer, 0, sizeof(bsp.trxd_peer));
    bsp.trxd_peer.sin_family = AF_INET;
    bsp.trxd_peer.sin_port   = htons(5702);
    bsp.trxd_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bsp.trxd_peer_valid = true;

    /* Bind UDP socket for TRXDv0 DL bursts from bridge/BTS.
     *
     * Default bind = 0.0.0.0 (was 127.0.0.1 hard-coded) so external
     * sources can inject bursts — bridge in the same netns still works,
     * and the host or other containers can reach BSP via the container
     * IP or via Docker port mapping (-p 6702:6702/udp).
     *
     * Override via env :
     *   CALYPSO_BSP_BIND_ADDR=<ip>  bind explicit IPv4 (e.g. 127.0.0.1,
     *                                172.20.0.11). Default: 0.0.0.0
     *   CALYPSO_BSP_BIND_LOOPBACK=1 legacy alias = 127.0.0.1 */
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        const char *bind_addr_env = getenv("CALYPSO_BSP_BIND_ADDR");
        const char *bind_lo_env   = getenv("CALYPSO_BSP_BIND_LOOPBACK");
        const char *bind_addr     = NULL;
        if (bind_addr_env && *bind_addr_env)
            bind_addr = bind_addr_env;
        else if (bind_lo_env && *bind_lo_env == '1')
            bind_addr = "127.0.0.1";
        else
            bind_addr = "0.0.0.0";

        struct sockaddr_in sa = {
            .sin_family = AF_INET,
            .sin_port = htons(BSP_TRXD_PORT),
        };
        if (inet_aton(bind_addr, &sa.sin_addr) == 0) {
            BSP_LOG("CALYPSO_BSP_BIND_ADDR=%s invalid, falling back to 0.0.0.0",
                    bind_addr);
            sa.sin_addr.s_addr = htonl(INADDR_ANY);
            bind_addr = "0.0.0.0";
        }
        if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            qemu_set_fd_handler(fd, bsp_trxd_readable, NULL, NULL);
            bsp.trxd_fd = fd;
            BSP_LOG("TRXD UDP listening on %s:%d", bind_addr, BSP_TRXD_PORT);
        } else {
            BSP_LOG("TRXD bind %s:%d failed: %s",
                    bind_addr, BSP_TRXD_PORT, strerror(errno));
            close(fd);
        }
    }

    /* Pre-init env-gated state so the first RACH burst doesn't pay the
     * cost of strtoul/getenv mid-run. Reportedly the static-cache pattern
     * had correlated runtime variability with LU success rate. */
    (void)d_rach_word_offset();
    (void)rach_force_bsic();

    /* Arm virtual-clock drain timer — locked to QEMU_CLOCK_VIRTUAL like
     * TINT0/tdma_tick/ARM fn. Fix 2026-05-24 e2e BTS↔L1 drift. */
    bsp.drain_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bsp_drain_cb, NULL);
    timer_mod(bsp.drain_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BSP_DRAIN_PERIOD_NS);
    BSP_LOG("BSP drain timer armed: %dms virtual period (VIRTUAL clock, drift-locked)",
            BSP_DRAIN_PERIOD_MS);

    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)",
            bsp.bypass_bdlena ? "  (BDLENA gate BYPASSED — debug)" : "");
}

/* ---- DL burst → DSP DARAM ---- */

void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16)
{
    bsp.bursts_seen++;

    if (!bsp.dsp) {
        if (bsp.bursts_seen <= 3)
            BSP_LOG("rx_burst: no DSP attached, dropping fn=%u tn=%u", fn, tn);
        return;
    }
    if (n_int16 <= 0 || iq == NULL) return;

    if (bsp.daram_addr == 0) {
        if (bsp.bursts_seen <= 5) {
            BSP_LOG("rx_burst fn=%u tn=%u n=%d (target unset)",
                    fn, tn, n_int16);
        }
        return;
    }

    /* On real hw the BSP serial link only carries samples while IOTA's
     * BDLENA pin is asserted. */
    if (!bsp.bypass_bdlena && !calypso_iota_take_bdl_pulse(tn)) {
        bsp.bursts_dropped_no_window++;
        if (bsp.bursts_dropped_no_window <= 5 ||
            (bsp.bursts_dropped_no_window % 100000) == 0) {
            BSP_LOG("DROP fn=%u tn=%u (no BDLENA window, dropped=%llu)",
                    fn, tn,
                    (unsigned long long)bsp.bursts_dropped_no_window);
        }
        return;
    }

    int n = n_int16 < (int)bsp.daram_len ? n_int16 : (int)bsp.daram_len;

    /* Load samples into BSP serial port buffer (PORTR PA=0x0034).
     * The DSP reads one sample per PORTR instruction from this buffer. */
    uint16_t samples[148];
    for (int i = 0; i < n && i < 148; i++)
        samples[i] = (uint16_t)iq[i];
    c54x_bsp_load(bsp.dsp, samples, n > 148 ? 148 : n);

    /* Also write to DARAM for code that reads samples directly. */
    static unsigned woff = 0;
    for (int i = 0; i < n; i++) {
        uint16_t a = (uint16_t)(bsp.daram_addr + woff);
        bsp.dsp->data[a] = (uint16_t)iq[i];
        bsp_daram_wr_bucket(a);
        woff++;
        if (woff >= bsp.daram_len) woff = 0;
    }
    bsp.bursts_written++;

    /* Log DARAM content after write for FB bursts */
    {
        if (bsp.bursts_written <= 3) {
            BSP_LOG("DARAM after write [0x%04x]: %d %d %d %d %d %d %d %d",
                    bsp.daram_addr,
                    n>0?(int16_t)bsp.dsp->data[bsp.daram_addr]:0,
                    n>1?(int16_t)bsp.dsp->data[bsp.daram_addr+1]:0,
                    n>2?(int16_t)bsp.dsp->data[bsp.daram_addr+2]:0,
                    n>3?(int16_t)bsp.dsp->data[bsp.daram_addr+3]:0,
                    n>4?(int16_t)bsp.dsp->data[bsp.daram_addr+4]:0,
                    n>5?(int16_t)bsp.dsp->data[bsp.daram_addr+5]:0,
                    n>6?(int16_t)bsp.dsp->data[bsp.daram_addr+6]:0,
                    n>7?(int16_t)bsp.dsp->data[bsp.daram_addr+7]:0);
        }
    }
    if (bsp.bursts_written <= 5 || (bsp.bursts_written % 1000) == 0) {
        BSP_LOG("DMA fn=%u tn=%u n=%d → DARAM[0x%04x..0x%04x] total=%llu "
                "iq[0..3]=%d,%d,%d,%d",
                fn, tn, n, bsp.daram_addr,
                (unsigned)(bsp.daram_addr + n - 1),
                (unsigned long long)bsp.bursts_written,
                n>0 ? iq[0] : 0, n>1 ? iq[1] : 0,
                n>2 ? iq[2] : 0, n>3 ? iq[3] : 0);
    }

    /* Fire BRINT0 — gated by BDLENA from the TPU/TSP/IOTA chain.
     * The firmware opens the RX window via TPU scenario → TSP write → IOTA BDLENA.
     * calypso_iota_take_bdl_pulse() consumed the window above.
     * BRINT0 fires once per window, rate-limited by IFR bit. */
    if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
        c54x_interrupt_ex(bsp.dsp, 21, 5);
        if (bsp.dsp->idle) bsp.dsp->idle = false;
    }
}

/* ---- Deliver buffered burst when BDLENA fires ---- */
/* Called from calypso_tdma_tick (calypso_trx.c) each frame.
 * For each TN: purge stale entries, then if a queued burst matches the
 * current QEMU virtual FN and a BDLENA pulse is pending, deliver it. */
void calypso_bsp_deliver_buffered(uint32_t current_fn)
{
    if (!bsp.dsp || bsp.daram_addr == 0) return;

    for (int tn = 0; tn < BSP_NUM_TN; tn++) {
        BspBurstSlot *sl = bsp_take_for_fn(tn, current_fn);
        if (!sl) continue;

        if (!bsp.bypass_bdlena && !calypso_iota_take_bdl_pulse(tn))
            continue;

        int n = sl->n < (int)bsp.daram_len ? sl->n : (int)bsp.daram_len;

        uint16_t samples[296];
        for (int i = 0; i < n && i < 296; i++)
            samples[i] = (uint16_t)sl->iq[i];
        c54x_bsp_load(bsp.dsp, samples, n > 296 ? 296 : n);

        static unsigned woff = 0;
        for (int i = 0; i < n; i++) {
            uint16_t a = (uint16_t)(bsp.daram_addr + woff);
            bsp.dsp->data[a] = (uint16_t)sl->iq[i];
            bsp_daram_wr_bucket(a);
            woff++;
            if (woff >= bsp.daram_len) woff = 0;
        }
        bsp.bursts_written++;
        sl->valid = false;  /* consumed */

        if (bsp.bursts_written <= 10 || (bsp.bursts_written % 1000) == 0) {
            BSP_LOG("DMA tn=%u fn=%u n=%d total=%llu stale=%llu qfull=%llu",
                    tn, sl->fn, n,
                    (unsigned long long)bsp.bursts_written,
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_dropped_queue_full);

            /* Dump first 8 words written so we can verify the I/Q
             * constellation actually landed in the DSP data memory at
             * daram_addr — independent of any ARM-side mapping. */
            BSP_LOG("DMA @0x%04x: %04x %04x %04x %04x %04x %04x %04x %04x",
                    bsp.daram_addr,
                    bsp.dsp->data[bsp.daram_addr + 0],
                    bsp.dsp->data[bsp.daram_addr + 1],
                    bsp.dsp->data[bsp.daram_addr + 2],
                    bsp.dsp->data[bsp.daram_addr + 3],
                    bsp.dsp->data[bsp.daram_addr + 4],
                    bsp.dsp->data[bsp.daram_addr + 5],
                    bsp.dsp->data[bsp.daram_addr + 6],
                    bsp.dsp->data[bsp.daram_addr + 7]);
        }

        /* Fire BRINT0 */
        if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
            c54x_interrupt_ex(bsp.dsp, 21, 5);
            if (bsp.dsp->idle) bsp.dsp->idle = false;
        }
    }
}

/* ---- UL burst → UDP to BTS ---- */

void calypso_bsp_send_ul(uint8_t tn, uint32_t fn, const uint8_t bits[148])
{
    if (bsp.trxd_fd < 0 || !bsp.trxd_peer_valid) return;

    /* TRXDv0 UL (TRX → BTS): tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes.
     *
     * The osmo-bts-trx TRXD protocol is *asymmetric* :
     *   - DL (BTS → TRX) : 6-byte header, 154 bytes total. No ToA.
     *   - UL (TRX → BTS) : 8-byte header WITH ToA, 156 bytes total. The
     *     ToA is needed by BTS RACH/SACCH timing-advance estimation.
     *
     * Sending 154-byte UL caused osmo-bts-trx::trx_if.c:821 to log
     *   "Rx TRXD PDU with odd burst length 146"
     * (BTS subtracts its 8-byte header from msg len, expects 148 body).
     * Always send 156 bytes for UL. */
    uint8_t pkt[8 + 148];
    pkt[0] = tn & 0x07;
    pkt[1] = (fn >> 24) & 0xff;
    pkt[2] = (fn >> 16) & 0xff;
    pkt[3] = (fn >>  8) & 0xff;
    pkt[4] =  fn        & 0xff;
    pkt[5] = 60;            /* RSSI → -60 dBm at the BTS */
    pkt[6] = 0; pkt[7] = 0; /* ToA256 = 0 (centered, no timing advance request) */
    for (int i = 0; i < 148; i++)
        pkt[8 + i] = bits[i] ? 127 : (uint8_t)(-127);

    /* Hex dump of every UL burst as it's sent — symmetric with the bridge.py
     * UL print, so we can correlate L1 → bridge → BTS at the byte level
     * when chasing TRXD framing or RACH parity issues. Cap at 200 to keep
     * log finite. */
    {
        static unsigned ul_log_count = 0;
        if (ul_log_count++ < 200 || (ul_log_count % 1000) == 0) {
            BSP_LOG("UL #%u TN=%u fn=%u rssi=-60 toa=0 len=%zu "
                    "hdr=%02x%02x%02x%02x%02x%02x%02x%02x "
                    "bits[0:16]=[%+d %+d %+d %+d %+d %+d %+d %+d "
                    "%+d %+d %+d %+d %+d %+d %+d %+d]",
                    ul_log_count, tn, fn, sizeof(pkt),
                    pkt[0], pkt[1], pkt[2], pkt[3],
                    pkt[4], pkt[5], pkt[6], pkt[7],
                    (int8_t)pkt[8], (int8_t)pkt[9], (int8_t)pkt[10],
                    (int8_t)pkt[11], (int8_t)pkt[12], (int8_t)pkt[13],
                    (int8_t)pkt[14], (int8_t)pkt[15],
                    (int8_t)pkt[16], (int8_t)pkt[17], (int8_t)pkt[18],
                    (int8_t)pkt[19], (int8_t)pkt[20], (int8_t)pkt[21],
                    (int8_t)pkt[22], (int8_t)pkt[23]);
        }
    }

    sendto(bsp.trxd_fd, pkt, sizeof(pkt), 0,
           (struct sockaddr *)&bsp.trxd_peer, sizeof(bsp.trxd_peer));
}

bool calypso_bsp_tx_burst(uint8_t tn, uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;

    /* On real Calypso, the DSP encodes the UL burst (channel coding +
     * interleaving + burst formation) and writes the 148 hard bits to a
     * DARAM buffer that the BSP TX DMA reads. The exact destination is
     * configured per task by TPU scenarios. We currently read from a
     * candidate location; if it's all-zero, the DSP encoder did not run
     * for this frame (timing miss or wrong addr) and we drop the burst. */
    bool any = false;
    for (int i = 0; i < 148; i++) {
        uint16_t w = bsp.dsp->data[0x0900 + i];
        bits[i] = (uint8_t)(w & 1);
        if (bits[i]) any = true;
    }

    return any;
}

/* ---- RACH access burst encoding ---- */
#include <osmocom/coding/gsm0503_coding.h>

/* d_rach lives in NDB at a struct offset that depends on the DSP version.
 * The firmware writes (uic|bsic)<<2 | (ra<<8) to ndb->d_rach right before
 * setting db_w->d_task_ra.
 *
 * Default 0x023A — confirmed empirically 2026-05-07 via D_RACH-FINDER ring
 * trace : ARM-side write at API byte 0x0474 (= DSP word 0x0A3A = word 0x23A
 * from API base) carries values 0x0300, 0x0f00, ... matching mobile L3
 * `RANDOM ACCESS ra 0xRR` log lines exactly.
 *
 * Cached via env var for ABI predictability — the old static-init+branch
 * pattern was reportedly correlated with worse LU success rate vs explicit
 * env set, so we now read env once and stash in bsp.* state at init. */
#define D_RACH_DEFAULT_OFFSET 0x023A
static uint32_t d_rach_word_offset(void)
{
    static uint32_t cached = 0;
    static bool     done = false;
    if (done) return cached;
    const char *e = getenv("CALYPSO_NDB_D_RACH_OFFSET");
    if (e && *e) {
        cached = (uint32_t)strtoul(e, NULL, 0);
        BSP_LOG("d_rach offset: 0x%04x (env=%s)", cached, e);
    } else {
        cached = D_RACH_DEFAULT_OFFSET;
        BSP_LOG("d_rach offset: 0x%04x (default macro — pinned 2026-05-07)", cached);
    }
    done = true;
    return cached;
}

/* CALYPSO_RACH_FORCE_BSIC=N forces the BSIC used by the RACH encoder to a
 * fixed value, overriding whatever is read from d_rach. Useful when the
 * d_rach offset is uncertain : if the BTS responds with IMM_ASS_CMD as
 * soon as we encode with the BSC's `base_station_id_code`, the chain is
 * proven and we know the only remaining bug is the d_rach offset.
 *
 * Returns -1 if unset, otherwise the forced BSIC value (0..63). */
static int rach_force_bsic(void)
{
    static int cached = -2;
    if (cached != -2) return cached;
    const char *e = getenv("CALYPSO_RACH_FORCE_BSIC");
    /* Same empty-string-as-unset handling as d_rach_word_offset(). */
    if (!e || !*e) {
        cached = -1;
        BSP_LOG("CALYPSO_RACH_FORCE_BSIC unset → BSIC read from d_rach");
        return cached;
    }
    long v = strtol(e, NULL, 0);
    if (v < 0 || v > 63) {
        BSP_LOG("CALYPSO_RACH_FORCE_BSIC=%s out of range [0..63] — ignored", e);
        cached = -1;
        return cached;
    }
    cached = (int)v;
    BSP_LOG("CALYPSO_RACH_FORCE_BSIC=%d (forcing all RACH bursts with this BSIC)", cached);
    return cached;
}

bool calypso_bsp_tx_rach_burst(uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;

    /* Read d_rach from NDB. dsp->data[] is the DSP-side word view; the
     * API RAM at DSP word 0x0800.. is shared with the ARM-visible page
     * at 0xFFD00000. We address via dsp->data[0x0800 + offset]. */
    uint32_t off = d_rach_word_offset();
    uint16_t d_rach = bsp.dsp->data[0x0800 + off];
    if (d_rach == 0) {
        /* Pre-LU : firmware hasn't written d_rach yet. Normal during cell
         * selection / SI decode phase. Don't alarm — just skip silently
         * (cap log to first 5 to keep it visible if there's a real issue). */
        static unsigned zero_log = 0;
        if (zero_log++ < 5) {
            BSP_LOG("RACH: d_rach@0x%04x is zero — skipping #%u "
                    "(normal pre-LU, mobile not yet in RR_EST_REQ)",
                    off, zero_log);
        }
        return false;
    }

    /* prim_rach.c:73 packs as:
     *   d_rach[7:0]  = uic<<2 (or bsic<<2)
     *   d_rach[15:8] = ra (8-bit RACH info) */
    uint8_t uic_or_bsic = (uint8_t)((d_rach & 0xFF) >> 2);
    uint8_t ra          = (uint8_t)((d_rach >> 8) & 0xFF);

    /* Optional BSIC override (probes whether wrong BSIC is the only blocker). */
    int forced = rach_force_bsic();
    if (forced >= 0) {
        uic_or_bsic = (uint8_t)forced;
    }

    /* gsm0503_rach_ext_encode writes 148 unpacked bits (ubit_t=uint8_t 0/1)
     * into burst[]. is_11bit=false → use 8-bit RACH (legacy GSM). */
    int rc = gsm0503_rach_ext_encode(bits, ra, uic_or_bsic, false);
    if (rc < 0) {
        BSP_LOG("RACH encode failed rc=%d ra=0x%02x bsic=0x%02x", rc, ra, uic_or_bsic);
        return false;
    }

    static int rach_log = 0;
    if (++rach_log <= 20) {
        BSP_LOG("RACH encode #%d fn=%u ra=0x%02x bsic=0x%02x d_rach=0x%04x",
                rach_log, fn, ra, uic_or_bsic, d_rach);
    }
    return true;
}
