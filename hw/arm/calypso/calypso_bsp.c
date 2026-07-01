/*
 * Calypso BSP/RIF DMA module — implementation.
 *
 * On real hardware the BSP (Baseband Serial Port) is a synchronous serial
 * link that DMA-feeds I/Q samples from the IOTA RF frontend into C54x
 * DSP DARAM. The DSP code (FB/SB/burst detection in PROM0) reads them
 * from a fixed DARAM buffer and posts results into the NDB.
 *
 * In QEMU, DL bursts arrive via UDP (TRXDv0 from calypso-ipc-device on port 5702).
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
#include <limits.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>   /* inet_aton */
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include "qemu/timer.h"
#include "hw/arm/calypso/calypso_bsp.h"
#include "calypso_orch.h"
#include "hw/arm/calypso/calypso_c54x.h"
extern int g_c54x_int3_src;  /* diag source INT3 (RO) */
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_twl3025.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "calypso_tint0.h"  /* GSM_HYPERFRAME */
#include "hw/arm/calypso/calypso_full_pcb.h"  /* DARAM lock helpers — voir pcb.h gap #3 */
#include "calypso_dsp_shunt.h"

/* calypso_trx_get_fn now provided by calypso_trx.h (included above). */

/* Forward decls for env-gated helpers used in calypso_bsp_init pre-warm. */
static uint32_t d_rach_word_offset(void);
static int rach_force_bsic(void);

#include "hw/arm/calypso/calypso_debug.h"
#define BSP_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("BSP")) \
        fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

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
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
    uint64_t   bursts_dropped_no_window;
    uint64_t   bursts_dropped_queue_full;
    uint64_t   bursts_dropped_stale;
    /* DL FN-LOCK (revival dsp) : offset constant burst_fn(base osmo-trx ts/5000)
     * vs current_fn(base g_wall_fn=0 au boot), auto-mesure 1x au 1er burst. */
    int32_t    dl_fnoff;
    int        dl_fnoff_done;
    uint8_t    inject_canary;     /* CALYPSO_BSP_INJECT_CANARY=1 :
                                      overwrite samples avec 0xCAFE pour
                                      identifier buffer cible via read trace */
    uint8_t    bypass_bdlena;      /* CALYPSO_BSP_BYPASS_BDLENA=1 :
                                      delivre tous les bursts sans attendre
                                      la fenetre BDLENA — debug-only pour
                                      sonder l'adresse DARAM cible.
                                      ATTENTION HACK env-gated. */
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

    /* === Real FB (FCCH tone) detection latch — wired to calypso_fbsb ===
     * Filled when the host-side correlator classifies a delivered burst as
     * TONAL_FB. calypso_fbsb reads these via calypso_bsp_get_fb_detection()
     * instead of synthesising constants. */
    int      fb_valid;        /* fresh real FB detection pending */
    int16_t  fb_toa;          /* latched DL ToA (quarter-bits, TRXDv0 hdr) */
    uint16_t fb_pm;           /* peak power proxy (from nmax) */
    int16_t  fb_ang;          /* angle (0 — AFC handled in TRX path) */
    uint16_t fb_snr;          /* tone quality (same_sign*10, 0..100) */
    int16_t  last_dl_toa_q4;  /* most recent DL ToA seen at decode */
} bsp;

/* Expose the latest real host-measured FB detection to calypso_fbsb.
 * Returns 1 and fills the out-params if a fresh TONAL_FB was latched
 * (consuming it); 0 otherwise. */
int calypso_bsp_get_fb_detection(int16_t *toa, uint16_t *pm,
                                 int16_t *ang, uint16_t *snr)
{
    if (!bsp.fb_valid) {
        return 0;
    }
    if (toa) *toa = bsp.fb_toa;
    if (pm)  *pm  = bsp.fb_pm;
    if (ang) *ang = bsp.fb_ang;
    if (snr) *snr = bsp.fb_snr;
    bsp.fb_valid = 0;
    return 1;
}

#define BSP_DRAIN_PERIOD_MS  5

/* === Deterministic replay (2026-05-28) ============================
 * Test discriminant : si CALYPSO_BSP_REPLAY_FILE est set, le BSP charge
 * un dump de bursts (format identique à BSP_DUMP_RX_FILE) et les injecte
 * sur QEMU_CLOCK_VIRTUAL à cadence fixe, AU LIEU d'écouter le socket UDP.
 * Source devient totalement déterministe.
 *
 * Workflow :
 *   1. Run normal avec BSP_DUMP_RX_FILE=/tmp/bsp_rx.dump → capture
 *   2. Re-run avec CALYPSO_BSP_REPLAY_FILE=/tmp/bsp_rx.dump → replay
 *   3. Comparer signature d_fb_det 2-3 runs replay → si identique entre
 *      runs, déterminisme restauré, course feed = root cause confirmée
 *
 * Cadence : 1 burst toutes les 576us virtuels = 1 burst par TN slot GSM
 * (8 slots × 217 frames/sec ≈ 1736 bursts/sec). Approximation suffisante
 * pour reproduire le rythme TDMA. */
typedef struct ReplayBurst {
    uint32_t fn;
    uint8_t  tn;
    uint16_t n;
    int16_t  iq[296];
} ReplayBurst;

static ReplayBurst *replay_bursts = NULL;
static size_t       replay_count  = 0;
static size_t       replay_idx    = 0;
static QEMUTimer   *replay_timer  = NULL;
#define BSP_REPLAY_PERIOD_NS  (576ULL * 1000ULL)  /* 576us per TN slot */
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
    /* target zone suit le runtime daram_addr (= ce que BSP écrit pour de vrai).
     * Anciennes bornes hardcodées 0x3FB0..0x3FFF étaient avant le canary fix
     * 2026-05-28 qui a changé le default à 0x2a00. Si daram_addr=0 (= discovery
     * mode), pas de target zone — tous les writes sont "other". */
    uint16_t tgt_lo = bsp.daram_addr;
    uint16_t tgt_hi = bsp.daram_addr ? (uint16_t)(bsp.daram_addr + bsp.daram_len - 1) : 0;
    if (addr <= BSP_BUCKET_LOW_HI) {
        bsp.wr_low++;
    } else if (tgt_lo && addr >= tgt_lo && addr <= tgt_hi) {
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
        /* DL FN-LOCK : auto-mesure l'offset d'epoque UNE FOIS (miroir cal_off UL /
         * du shunt qui lit l1s.fn). dl_fnoff = current_fn - burst_fn ; on l'ajoute
         * a burst_fn pour le match -> delta ~0 si offset constant. Override
         * CALYPSO_DL_FN_OFFSET=<n>. Si dl_fnoff mesure ~0 -> pas un offset
         * d'epoque (= back-pressure de drain), le flood stale persistera (auto-diag). */
        if (!bsp.dl_fnoff_done) {
            const char *e = getenv("CALYPSO_DL_FN_OFFSET");
            bsp.dl_fnoff = (e && *e) ? (int32_t)strtol(e, NULL, 0)
                                     : bsp_fn_delta(current_fn, s->fn);
            bsp.dl_fnoff_done = 1;
            fprintf(stderr, "[BSP] DL FN-LOCK dl_fnoff=%d (burst_fn=%u cur_fn=%u "
                    "%s)\n", bsp.dl_fnoff, s->fn, current_fn,
                    (e && *e) ? "env" : "auto");
        }
        int32_t d = bsp_fn_delta(s->fn + (uint32_t)bsp.dl_fnoff, current_fn);
        int32_t ad = d < 0 ? -d : d;
        if (d < -BSP_FN_MATCH_WINDOW) {
            s->valid = false;
            bsp.bursts_dropped_stale++;
            /* sonde residuelle : si ca droppe ENCORE apres l'offset -> drift
             * (back-pressure ou offset non constant). Logue raw vs ajuste. */
            { static uint64_t pn = 0;
              if (pn < 30 || (pn % 5000) == 0)
                fprintf(stderr, "[BSP] DL-OFFSET-PROBE burst_fn=%u cur_fn=%u "
                        "raw=%d adj=%d off=%d\n", s->fn, current_fn,
                        bsp_fn_delta(s->fn, current_fn), d, bsp.dl_fnoff);
              pn++; }
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
    /* Auto-detect hex even sans préfixe 0x : si la chaîne contient
     * un digit hex non-décimal (a-f / A-F), force base 16. Évite le
     * piège strtoul base=0 qui parse "2a00" comme décimal → 2. */
    int base = 0;
    for (const char *p = v; *p; p++) {
        if ((*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')) {
            base = 16;
            break;
        }
    }
    return (uint16_t)strtoul(v, NULL, base);
}

uint16_t calypso_bsp_get_daram_addr(void) { return bsp.daram_addr; }
uint16_t calypso_bsp_get_daram_len(void)  { return bsp.daram_len; }
uint8_t  calypso_bsp_get_last_att(void)   { return bsp.last_att; }

/* ---- UDP TRXDv0 DL receive callback ---- */

static void bsp_trxd_readable(void *opaque)
{
    /* BRIDGE_BSP_IQ=1 envoie 8 hdr + 4*148 IQ = 600 bytes. Le buffer 512
     * historique TRONQUAIT silencieusement → BSP recevait soft-bits non
     * convertis → IQ_PASSTHROUGH if-branch jamais prise → hard cos_tab
     * fallback → AFC rotation BSP totalement ineffective. */
    uint8_t buf[2048];
    struct sockaddr_in addr;
    socklen_t alen = sizeof(addr);

    ssize_t n = recvfrom(bsp.trxd_fd, buf, sizeof(buf), MSG_DONTWAIT,
                         (struct sockaddr *)&addr, &alen);
    if (n < 8) return;

    /* Tee I/Q vers le bridge de démod (gr-gsm py) sous shunt : le BSP gate
     * la livraison DARAM de toute façon (calypso_bsp.c ~990), donc on forwarde
     * le burst brut (8 hdr + I/Q cs16) vers CALYPSO_IQ_TEE_PORT (défaut 6703).
     * Le bridge démode → GSMTAP → 4730 → shunt feed_si → a_cd. ZÉRO hack :
     * c'est le VRAI signal du BTS qui transite, juste copié hors-bande.
     * Sert aussi au FFT live (lit le tee 6703). Gaté shunt only (diag). */
    if (calypso_dsp_shunt_active()) {
        static int tee_fd = -1;
        static struct sockaddr_in tee_dst;
        if (tee_fd == -1) {
            tee_fd = socket(AF_INET, SOCK_DGRAM, 0);
            const char *p = getenv("CALYPSO_IQ_TEE_PORT");
            int port = (p && *p) ? atoi(p) : 6703;
            /* Dest configurable : 127.0.0.1 (bridge in-container) par défaut,
             * ou CALYPSO_IQ_TEE_HOST=172.20.0.1 (gateway gsm-inter) pour viser
             * l'hôte → FFT live pop-up côté hôte (X natif, pas de X dans docker). */
            const char *h = getenv("CALYPSO_IQ_TEE_HOST");
            memset(&tee_dst, 0, sizeof(tee_dst));
            tee_dst.sin_family = AF_INET;
            tee_dst.sin_port = htons(port);
            tee_dst.sin_addr.s_addr = (h && *h) ? inet_addr(h)
                                                : htonl(INADDR_LOOPBACK);
            BSP_LOG("IQ-TEE -> %s:%d (bridge/FFT)", (h && *h) ? h : "127.0.0.1", port);
        }
        if (tee_fd >= 0)
            sendto(tee_fd, buf, n, MSG_DONTWAIT,
                   (struct sockaddr *)&tee_dst, sizeof(tee_dst));

        /* Buffer shm (pas UDP) : publie l'I/Q d'entree du DSP shunte pour
         * gr-gsm. buf[8..] = int16 I/Q entrelaces (cs16, mode passthrough),
         * comme le tee. gr-gsm lit ce buffer cote shm. */
        if (n > 8) {
            uint32_t _fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                           ((uint32_t)buf[3] << 8)  |  (uint32_t)buf[4];
            calypso_dsp_shunt_feed_iq(_fn, (const int16_t *)(buf + 8),
                                      (int)((n - 8) / 2));
        }
    }

    /* Diag : log first 10 recv sizes pour vérifier que bridge envoie bien
     * 600 bytes (= IQ mode) et que BSP buffer ne tronque pas. */
    {
        static int rxsz_log = 0;
        if (rxsz_log++ < 10) {
            BSP_LOG("RXSZ #%d recv=%zd from %s:%d", rxsz_log,
                    n, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
        }
    }

    /* Refine UL peer to actual DL sender (init-time default is bridge
     * 127.0.0.1:5702 — DL source confirms it or replaces it). */
    if (addr.sin_addr.s_addr != bsp.trxd_peer.sin_addr.s_addr ||
        addr.sin_port != bsp.trxd_peer.sin_port) {
        bsp.trxd_peer = addr;
        BSP_LOG("TRXD peer learned: %s:%d",
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    }

    /* "le shunt dsp ne doit pas shunt l'ipc" : shunt actif = le vrai DSP ne
     * consomme JAMAIS la DARAM (gr-gsm decode via le shm feed_iq). On a deja
     * draine l'UDP (recvfrom) + publie l'I/Q (feed_iq) + appris le peer UL.
     * On NE remplit donc PAS la queue DARAM (bsp_enqueue) -> sinon elle sature
     * (bursts_dropped_queue_full) -> backpressure qui remonte et TUE l'IPC/BTS.
     * On rend la main : l'IPC continue de couler, le shm est nourri. */
    if (calypso_dsp_shunt_active())
        return;

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

    /* DL-TOA probe (revival dsp 2026-06-22, read-only) : le ToA intra-slot du
     * burst DL (header TRXDv0 buf[6..7], int16 q4 BE) = le "2e offset" (l'analogue
     * DL de UL_SLOT_OFFSET), actuellement IGNORE par le BSP (qemu_wrap.c:98).
     * On le MESURE pour savoir de combien decaler la position du burst en DARAM.
     * Capé + periodique. q4 = quart-de-bit ; samples ~= toa_q4/4*SPS. */
    {
        int16_t dl_toa_q4 = (n >= 8) ? (int16_t)(((uint16_t)buf[6] << 8) | buf[7]) : 0;
        bsp.last_dl_toa_q4 = dl_toa_q4;
        static unsigned dltoa_n = 0;
        if (dltoa_n < 40 || (dltoa_n % 4000) == 0) {
            fprintf(stderr, "[BSP] DL-TOA tn=%u fn=%u rssi=%u toa_q4=%d "
                    "(bits~%d) n=%zd\n", tn, fn, (unsigned)bsp.last_att,
                    dl_toa_q4, dl_toa_q4 / 4, n);
            dltoa_n++;
        }
    }

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
     * int16 IQ pairs LE (calypso-ipc-device CALYPSO_BSP_IQ_PASSTHROUGH=1 envoie ce format,
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
    /* Bridge envoie 2 int16 par bit (I,Q interleaved). 4 bytes/bit.
     * Pour 146-148 bits = 584-592 octets payload.
     * Auto-détection : si bytes >= 4*146 → IQ mode (BTS-source 146 bits OK).
     * Sinon → soft bits 1 byte/bit. */
    int iq_min_bits = 146;
    if (iq_pt_mode && iq_bytes >= 4 * iq_min_bits) {
        /* Override nbits depuis payload IQ : 1 sample = 1 bit = 4 bytes. */
        nbits = iq_bytes / 4;
        if (nbits > 148) nbits = 148;
        /* Bridge pre-modulated path : copy 2*nbits I/Q values directly.
         * Bytes are int16 LE on x86 host = same as int16_t native. */
        int copy_count = 2 * nbits;
        if (copy_count > 296) copy_count = 296;
        memcpy(iq, buf + 8, copy_count * sizeof(int16_t));
        iq_count = copy_count;
        /* Apply AFC rotation : TWL3025 VCXO offset propagation. No-op si
         * CALYPSO_TWL3025_AFC != 1. Convergence AFC chain dépend de ça :
         * firmware applique AFC delta → DSP TSP → TWL3025 DAC → samples
         * rotated → DSP correlator voit la convergence. */
        /* ⚠️ TESTING 2026-05-29 : apply_phase déplacé décode -> delivery
         * (l'AFC doit s'appliquer quand le DSP voit les samples = dac courant,
         * pas au décode où le dac est stale de ~lookahead frames). */
        /* calypso_twl3025_apply_phase(iq, copy_count / 2, fn, tn); */
        static int pt_log = 0;
        if (pt_log < 10 || (pt_log % 5000) == 0) {
            BSP_LOG("IQ passthrough #%d fn=%u tn=%u bytes=%d nbits=%d "
                    "iq[0..3]=%d,%d,%d,%d",
                    pt_log, fn, tn, iq_bytes, nbits,
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

/* REALTIME drain callback (2026-05-29) : pulls BSP UDP queue into DSP DMA
 * à la cadence wall-clock 5ms (= 200/sec). Monotonic anti-drift rearm sur
 * `last_target + period` pour éviter accumulation de jitter dispatcher.
 *
 * Historique : pre-2026-05-24 c'était REALTIME → drift vs VIRTUAL sous
 * icount=auto. Switch vers VIRTUAL fixait ce drift. 2026-05-29 : maintenant
 * que tdma_tick est REALTIME monotonic + clk_master pthread, virtual et
 * wall sont alignés. On peut repasser drain en REALTIME — la cadence wall
 * matche la cadence ARM frame_irq/tdma. Et surtout : sous load DSP heavy,
 * VIRTUAL tournait moins vite que wall → drain trop lent → BSP queue
 * overflow → 95% des bursts droppés. */
static void bsp_drain_cb(void *opaque)
{
    static int64_t last_target = 0;
    /* Drain la socket UDP DL ICI (timer REALTIME fiable, fix 2026-05-30).
     * Sous icount=auto le DSP (c54x_run) monopolise le thread mainloop →
     * l'iohandler bsp_trxd_readable n'est jamais servi → les paquets device
     * s'accumulent non-lus (Recv-Q monte) → BSP-DELIVER=0, D_BURST_D vide,
     * snr=0. On vide la socket à chaque tick drain (recvfrom MSG_DONTWAIT),
     * indépendant de la mainloop affamée. 64 = marge (≈1-2 bursts/5ms). */
    {
        /* Test décisif : PEEK direct sur bsp.trxd_fd — la data est-elle sur CE
         * fd ? (errno=EAGAIN/11 = rien ici ; >0 = data présente). */
        uint8_t tb[16]; struct sockaddr_in sa; socklen_t sl = sizeof(sa);
        errno = 0;
        ssize_t pk = (bsp.trxd_fd >= 0)
            ? recvfrom(bsp.trxd_fd, tb, sizeof(tb), MSG_DONTWAIT | MSG_PEEK,
                       (struct sockaddr *)&sa, &sl)
            : -99;
        int e = errno;
        for (int i = 0; i < 64 && bsp.trxd_fd >= 0; i++)
            bsp_trxd_readable(NULL);
        static uint64_t dc = 0;
        if (dc < 30 || (dc % 2000) == 0)
            /* FIX 2026-06-02 : reporte le VRAI compteur de livraison
             * `bursts_written` (incr. dans deliver_buffered ligne 1107 = burst
             * réellement écrit en DARAM `dsp->data[a]`) au lieu du `bursts_seen`
             * MORT. bursts_seen vit dans calypso_bsp_rx_burst, que le refactor
             * 2026-05-29 a bypassé (deliver écrit inline) → seen=0 à vie = sonde
             * menteuse qui a coûté des heures de fausse piste "feed mort".
             * delivered>0 et qui monte = signal réellement livré au DSP. */
            fprintf(stderr, "[BSP] DRAIN-CB #%llu fd=%d PEEK=%zd errno=%d "
                    "delivered=%llu enq_drops(stale=%llu,full=%llu) seen_DEAD=%llu\n",
                    (unsigned long long)dc, bsp.trxd_fd, pk, e,
                    (unsigned long long)bsp.bursts_written,
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_dropped_queue_full,
                    (unsigned long long)bsp.bursts_seen);
        dc++;
    }
    if (bsp.dsp) {
        uint32_t cur_fn = calypso_trx_get_fn();
        calypso_bsp_deliver_buffered(cur_fn);
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    if (last_target == 0) last_target = now;
    int64_t target = last_target + BSP_DRAIN_PERIOD_NS;
    while (target <= now) {
        target += BSP_DRAIN_PERIOD_NS;
    }
    last_target = target;
    timer_mod(bsp.drain_timer, target);
}

/* Replay callback : enqueue 1 burst per virtual TN slot. */
static void bsp_replay_cb(void *opaque)
{
    if (replay_idx < replay_count) {
        ReplayBurst *r = &replay_bursts[replay_idx++];
        bsp_enqueue(r->tn, r->fn, r->iq, r->n);
        if (replay_idx <= 5 || (replay_idx % 1000) == 0) {
            BSP_LOG("REPLAY inject #%zu fn=%u tn=%u n=%u",
                    replay_idx, r->fn, (unsigned)r->tn, (unsigned)r->n);
        }
    } else if (replay_idx == replay_count && replay_count > 0) {
        BSP_LOG("REPLAY exhausted after %zu bursts (idle from now)",
                replay_count);
        replay_idx++;  /* prevent log spam */
    }
    timer_mod(replay_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BSP_REPLAY_PERIOD_NS);
}

/* Load all bursts from a BSP_DUMP_RX_FILE-format dump into memory.
 * Returns number loaded, 0 on failure. */
static size_t bsp_replay_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        BSP_LOG("REPLAY open '%s' failed: %s", path, strerror(errno));
        return 0;
    }
    size_t loaded = 0;
    size_t cap = 256;
    replay_bursts = calloc(cap, sizeof(ReplayBurst));
    if (!replay_bursts) { fclose(f); return 0; }
    while (1) {
        uint8_t hdr[12];
        if (fread(hdr, 1, 12, f) != 12) break;
        if (memcmp(hdr, "IQ16", 4) != 0) {
            BSP_LOG("REPLAY bad magic at burst %zu, stop", loaded);
            break;
        }
        uint32_t fn = (uint32_t)hdr[4]
                    | ((uint32_t)hdr[5] << 8)
                    | ((uint32_t)hdr[6] << 16)
                    | ((uint32_t)hdr[7] << 24);
        uint8_t  tn = hdr[8];
        uint16_t n  = (uint16_t)hdr[9] | ((uint16_t)hdr[10] << 8);
        if (n == 0 || n > 296) {
            BSP_LOG("REPLAY out-of-range n=%u at burst %zu, stop", n, loaded);
            break;
        }
        if (loaded >= cap) {
            cap *= 2;
            ReplayBurst *grown = realloc(replay_bursts,
                                         cap * sizeof(ReplayBurst));
            if (!grown) {
                BSP_LOG("REPLAY OOM at %zu bursts, stop", loaded);
                break;
            }
            replay_bursts = grown;
        }
        ReplayBurst *r = &replay_bursts[loaded];
        r->fn = fn;
        r->tn = tn;
        r->n  = n;
        if (fread(r->iq, sizeof(int16_t), n, f) != (size_t)n) {
            BSP_LOG("REPLAY truncated at burst %zu, stop", loaded);
            break;
        }
        loaded++;
    }
    fclose(f);
    return loaded;
}

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp = dsp;
    /* 2026-05-28 : ancien commentaire "DSP reads I/Q at 0x3fb3-0x3fbe"
     * obsolete. Discovery par CALYPSO_BSP_INJECT_CANARY a confirme que
     * le vrai buffer cote DSP est 0x2a00 (PC=0x93a5 consumer, AR3 post-inc
     * sur 0x2a00..0x2a13). Nouveau default ci-dessous. */
    /* DARAM target where BSP DMAs DL samples. Default 0x2a00, identifie via
     * methode 3 (CALYPSO_BSP_INJECT_CANARY 2026-05-28) :
     * 1. Static scan PROM0 : 0x2a00 = top STM #imm,ARx init (50 sites,
     *    AR1..AR6 ; companion BK=0x015e=350 = burst size GSM)
     * 2. Runtime canary injection : CALYPSO_BSP_INJECT_CANARY=1 →
     *    DSP READS 0xCAFE at addr=0x2a00..0x2a13 via PC=0x93a5 (= real
     *    consumer routine), AR3=0x2a00 post-incrementing. E2E proven.
     * Voir doc/BOOT_TO_FBSB_SEQUENCE.md. Override via env si besoin. */
    bsp.daram_addr     = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0x2a00);
    bsp.daram_len      = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  296);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    bsp.bursts_dropped_no_window = 0;
    /* ATTENTION HACK HACK HACK !!!!!!!!
     * CALYPSO_BSP_BYPASS_BDLENA=1 : bypass de la fenetre IOTA BDLENA.
     * Sur silicon, le BSP ne delivre les samples au DSP que pendant la
     * fenetre BDLENA assertee par IOTA. Sur emu, on a parfois besoin de
     * sonder quelle adresse DARAM le DSP correlator lit reellement
     * (CALYPSO_BSP_DARAM_ADDR mismatch suspecte) — ce flag desactive le
     * gate pour livrer TOUS les bursts. Default OFF.
     * Critere de retrait : DARAM target identifiee + a_pm/a_sync_demod
     * publies nonzero par DSP. Voir doc/TODO.md. */
    bsp.bypass_bdlena = (uint8_t)parse_uint_env("CALYPSO_BSP_BYPASS_BDLENA", 0);
    if (bsp.bypass_bdlena) {
        BSP_LOG("HACK: CALYPSO_BSP_BYPASS_BDLENA=1 — IOTA BDLENA gate DISABLED");
    }
    /* Canary injection (debug) : if CALYPSO_BSP_INJECT_CANARY=1, BSP
     * overwrites all samples with a recognizable marker (0xCAFE) before
     * DARAM write. Combined with data_read_locked canary watch in
     * calypso_c54x.c, this directly identifies WHERE the DSP reads from
     * the BSP buffer at runtime — no brute-force.
     * Disable in normal runs. Voir doc/TODO.md. */
    bsp.inject_canary = (uint8_t)parse_uint_env("CALYPSO_BSP_INJECT_CANARY", 0);
    if (bsp.inject_canary) {
        BSP_LOG("HACK: CALYPSO_BSP_INJECT_CANARY=1 — samples overwritten with 0xCAFE for buffer discovery");
    }
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

    /* Deterministic-replay short-circuit. If CALYPSO_BSP_REPLAY_FILE is
     * set, load it now and skip the UDP listener. Replay timer takes over
     * the supply role. */
    const char *replay_path = getenv("CALYPSO_BSP_REPLAY_FILE");
    if (replay_path && *replay_path) {
        replay_count = bsp_replay_load(replay_path);
        BSP_LOG("REPLAY mode: loaded %zu bursts from %s (UDP socket bypassed)",
                replay_count, replay_path);
        if (replay_count > 0) {
            replay_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, bsp_replay_cb,
                                        NULL);
            timer_mod(replay_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
                      + BSP_REPLAY_PERIOD_NS);
        }
        goto skip_udp_listener;
    }

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

        /* Port override : permet d'insérer un proxy Python (iq_proxy.py)
         * entre source et QEMU. Source unchanged (envoie sur 6702), QEMU
         * listen sur CALYPSO_BSP_PORT=6712 (par ex), proxy fait Doppler. */
        const char *port_env = getenv("CALYPSO_BSP_PORT");
        int bsp_port = BSP_TRXD_PORT;
        if (port_env && *port_env) {
            int p = atoi(port_env);
            if (p > 0 && p < 65536) bsp_port = p;
        }
        struct sockaddr_in sa = {
            .sin_family = AF_INET,
            .sin_port = htons(bsp_port),
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
            BSP_LOG("TRXD UDP listening on %s:%d", bind_addr, bsp_port);
        } else {
            BSP_LOG("TRXD bind %s:%d failed: %s",
                    bind_addr, bsp_port, strerror(errno));
            close(fd);
        }
    }

skip_udp_listener:
    /* Pre-init env-gated state so the first RACH burst doesn't pay the
     * cost of strtoul/getenv mid-run. Reportedly the static-cache pattern
     * had correlated runtime variability with LU success rate. */
    (void)d_rach_word_offset();
    (void)rach_force_bsic();

    /* Arm REALTIME drain timer — wall-paced 5ms, monotonic anti-drift dans
     * bsp_drain_cb. Aligné sur le même CLOCK_MONOTONIC que le pthread
     * clk_master (calypso_trx.c). 2026-05-29 : sortie de VIRTUAL parce
     * qu'on droppait 95% des bursts sous load DSP (virtual lag → drain
     * trop lent). */
    bsp.drain_timer = timer_new_ns(QEMU_CLOCK_REALTIME, bsp_drain_cb, NULL);
    timer_mod(bsp.drain_timer,
              qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + BSP_DRAIN_PERIOD_NS);
    BSP_LOG("BSP drain timer armed: %dms REALTIME wall-paced, monotonic",
            BSP_DRAIN_PERIOD_MS);

    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)",
            "");
}

/* ---- DL burst → DSP DARAM ---- */

void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16)
{
    bsp.bursts_seen++;
    /* ⚠️ TESTING 2026-05-29 : marqueur — si ça fire, rx_burst EST vivant
     * (et il faudra y appliquer l'AFC aussi). Sinon = code mort. */
    {
        static unsigned rxb_n;
        if (calypso_debug_enabled("BSP-RXBURST") &&
            (rxb_n <= 20 || rxb_n % 2000 == 0))
            fprintf(stderr, "[BSP] BSP-RXBURST #%u fn=%u tn=%u n=%d\n",
                    rxb_n, (unsigned)fn, (unsigned)tn, n_int16);
        rxb_n++;
    }

    /* GATE DSP_SHUNT : si le shunt est actif, le c54x ne tourne pas et
     * le mock écrit directement NDB/read-page. Toute écriture BSP vers
     * DARAM écraserait les valeurs canned du mock. */
    if (calypso_dsp_shunt_active()) {
        if (bsp.bursts_seen <= 3)
            BSP_LOG("rx_burst: DSP_SHUNT active, dropping fn=%u tn=%u", fn, tn);
        return;
    }

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

    /* 2026-05-29 : remplacement du gate BDLENA. Anciennement on droppait
     * le burst si pas de pulse IOTA matching ; maintenant on délivre
     * inconditionnellement. AUCUNE écriture de d_dsp_page ici — c'est
     * firmware qui pilote le page flip via dsp_end_scenario (MMIO WR
     * sur 0x01A8). On signale juste l'arrivée samples au DSP via INT3
     * (= silicon BDLENA→BSP→DSP arm_done equivalent).
     *
     * Probe read-only sur d_dsp_page : on log la valeur vue par DSP
     * au moment du burst (= ce que firmware a écrit). Sans modifier. */
    if (bsp.dsp && bsp.dsp->api_ram) {
        static uint32_t obs_n = 0;
        uint16_t cur = bsp.dsp->api_ram[0x08D4 - 0x0800];
        obs_n++;
        if (calypso_debug_enabled("PUMP") &&
            (obs_n <= 20 || obs_n % 37 == 0)) {
            fprintf(stderr, "[bsp-page] #%u rx_burst fn=%u tn=%u "
                    "d_dsp_page=0x%04x (B_GSM_TASK=%d w_page=%d)\n",
                    obs_n, fn, tn, cur, !!(cur & 2), !!(cur & 1));
            fflush(stderr);
        }
    }
    /* Gate INT3 fire : skip si IFR.bit3 déjà set = DSP pas encore servi
     * le précédent. Évite stacking d'IRQs quand DSP traite plus lentement
     * que BSP delivery rate. */
    if (!getenv("CALYPSO_BSP_INT3_OFF") && bsp.dsp && bsp.dsp->running && !(bsp.dsp->ifr & (1 << 3))) {
        g_c54x_int3_src = 2;
        c54x_interrupt_ex(bsp.dsp, 19, 3);  /* INT3 (frame) — vec 19, IMR bit 3 */
        if (bsp.dsp->idle) bsp.dsp->idle = false;
    }

    int n = n_int16 < (int)bsp.daram_len ? n_int16 : (int)bsp.daram_len;

    /* Load samples into BSP serial port buffer (PORTR PA=0x0034).
     * The DSP reads one sample per PORTR instruction from this buffer.
     * ⚠️ NON-DÉFINITIF / TESTING 2026-05-29 (hypothèse, à valider/débugger).
     * FIX 2026-05-29 : livrer le burst COMPLET (iq[] = I/Q interleaved,
     * 2*nbits int16, jusqu'à 296), PAS tronqué à 148. Tronquer à 148 int16
     * ne donnait au corrélateur que 74 symboles complexes = la MOITIÉ du
     * burst → la tonalité FCCH (FB) ne pouvait jamais corréler → FBSB_CONF
     * jamais émis. On borne sur n_int16 (taille réelle du burst), pas n
     * (qui était clampé à daram_len pour l'écriture DARAM). */
    {
        uint16_t samples[296];
        int ns = n_int16 > 296 ? 296 : n_int16;
        for (int i = 0; i < ns; i++)
            samples[i] = (uint16_t)iq[i];
        c54x_bsp_load(bsp.dsp, samples, ns);
    }

    /* Also write to DARAM for code that reads samples directly.
     * Wrap the whole burst write + post-write log in a single DARAM lock
     * section — sans ça, DSP thread (Phase 2 PCB) racerait avec ce write
     * et lirait des samples partiellement écrits. Cost = 1 mutex op pour
     * ~157 itérations ≈ négligeable. */
    /* ⚠️ NON-DÉFINITIF / TESTING 2026-05-29 (hypothèse, à valider/débugger).
     * FIX 2026-05-29 : woff LOCAL (était static) — chaque burst écrit aligné
     * à daram_addr[0..n-1]. Le static faisait rouler l'offset cross-burst :
     * le burst FB d'une frame atterrissait à un offset que le DSP ne lit pas
     * (fragmenté sur le wrap) → corrélateur sur données désalignées. */
    unsigned woff = 0;
    calypso_pcb_daram_lock_acquire();
    for (int i = 0; i < n; i++) {
        uint16_t a = (uint16_t)(bsp.daram_addr + woff);
        bsp.dsp->data[a] = (uint16_t)iq[i];
        bsp_daram_wr_bucket(a);
        woff++;
        if (woff >= bsp.daram_len) woff = 0;
    }
    bsp.bursts_written++;

    /* PROBE 2026-05-31 fork-1 : dump des bursts I/Q pour FFT offline (cherche
     * le pic FCCH à +67.7 kHz = 1625/24). Gated CALYPSO_IQDUMP. Dump bursts
     * 5..28 en raw int16 (un fichier par burst → l'un d'eux = FCCH). À RETIRER. */
    if (getenv("CALYPSO_IQDUMP")) {
        static unsigned rx_dump_n;
        if (rx_dump_n < 24) {
            char path[80];
            snprintf(path, sizeof(path), "/tmp/iq_rx_%03u.bin", rx_dump_n);
            FILE *f = fopen(path, "wb");
            if (f) {
                for (int i = 0; i < n; i++) {
                    int16_t s = (int16_t)iq[i];
                    fwrite(&s, sizeof(int16_t), 1, f);
                }
                fclose(f);
                BSP_LOG("IQDUMP rx #%u → %s (%d int16)", rx_dump_n, path, n);
            }
            rx_dump_n++;
        }
    }

    /* Log DARAM content after write for FB bursts (inside lock so values
     * read are consistent with what we just wrote). */
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
    calypso_pcb_daram_lock_release();
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
    /* GATE DSP_SHUNT (cf calypso_bsp_rx_burst). Idem ici : si le shunt
     * est actif, on ne livre aucun sample bufferisé — le mock owns la
     * DARAM API region pour ses canned responses. */
    if (calypso_dsp_shunt_active()) return;

    if (!bsp.dsp || bsp.daram_addr == 0) return;

    for (int tn = 0; tn < BSP_NUM_TN; tn++) {
        /* Drain ALL matchable bursts per call (2026-05-29 fix anti-stale).
         * Avant : 1 burst/appel → sous contention BQL le drain rate effectif
         * tombe sous le rate d'arrivée IPC → queue fills → bursts > 64 FN
         * derriere cur_fn marqués stale (= 87% drop observé).
         * Maintenant : drain catch-up jusqu'à plus aucun match. Bornage
         * via la fenêtre BSP_FN_MATCH_WINDOW dans bsp_take_for_fn — pas de
         * runaway. */
        BspBurstSlot *sl;
        while ((sl = bsp_take_for_fn(tn, current_fn)) != NULL) {

        /* 2026-05-29 : pas d'écriture d_dsp_page, juste INT3 (arm_done).
         * Probe read-only voir commentaire dans calypso_bsp_rx_burst. */
        if (bsp.dsp && bsp.dsp->api_ram) {
            static uint32_t obs_n = 0;
            uint16_t cur = bsp.dsp->api_ram[0x08D4 - 0x0800];
            obs_n++;
            if (calypso_debug_enabled("PUMP") &&
                (obs_n <= 20 || obs_n % 37 == 0)) {
                fprintf(stderr, "[bsp-page] #%u drain fn=%u tn=%u "
                        "d_dsp_page=0x%04x (B_GSM_TASK=%d w_page=%d)\n",
                        obs_n, current_fn, tn, cur,
                        !!(cur & 2), !!(cur & 1));
                fflush(stderr);
            }
        }
        /* Gate INT3 : skip si IFR.bit3 déjà set (cf rx_burst). */
        if (!getenv("CALYPSO_BSP_INT3_OFF") && bsp.dsp && bsp.dsp->running && !(bsp.dsp->ifr & (1 << 3))) {
            g_c54x_int3_src = 2;
            c54x_interrupt_ex(bsp.dsp, 19, 3);  /* INT3 (frame) */
            if (bsp.dsp->idle) bsp.dsp->idle = false;
        }

        int n = sl->n < (int)bsp.daram_len ? sl->n : (int)bsp.daram_len;

        /* === SB-INPUT discriminator (phase-based, 2026-05-28 v2) ===
         * GMSK = constant envelope → magnitude(I,Q) constant pour FCCH ET
         * SCH. Le seul discriminant qui sépare est la trajectoire de phase :
         *   FCCH  = tone pur → Δphase constant → cross[k]=I[k]*Q[k-1]-Q[k]*I[k-1]
         *           a même signe à tous les k (rotation monotone)
         *   SCH/NB = GMSK data → Δphase varie ±90°/sample → cross alterne
         * Compteur de cross-product de même signe que cross[0] sur 10 paires :
         *   ≥9 same-sign → TONAL_FB
         *   ≤8           → MODULATED
         * nmax conservé en plus pour détecter SILENT.  Cap 600. */
        {
            static unsigned db_log;
            const unsigned LIMIT = 600;
            {
                const int N = 22 < n / 2 ? 22 : n / 2;  /* N pairs ⇒ 2N samples */
                int nmax = 0;
                for (int i = 0; i < 2 * N && i < n; i++) {
                    int s = (int)sl->iq[i];
                    if (s < 0) s = -s;
                    if (s > nmax) nmax = s;
                }
                int same_sign = 0;
                int cross0 = 0;
                int cross_logged[8] = {0};
                int cross_log_cnt = 0;
                int n_cross = 0;
                for (int k = 1; k < N && n_cross < 11; k++) {
                    int I  = (int)sl->iq[2*k];
                    int Q  = (int)sl->iq[2*k + 1];
                    int Ip = (int)sl->iq[2*(k-1)];
                    int Qp = (int)sl->iq[2*(k-1) + 1];
                    /* Use int64 to avoid overflow : I*Q up to 1G, diff up to 2G. */
                    long cross_l = (long)I * (long)Qp - (long)Q * (long)Ip;
                    int cross = cross_l > 0 ? 1 : (cross_l < 0 ? -1 : 0);
                    if (n_cross == 0) cross0 = cross;
                    else if (cross != 0 && cross == cross0) same_sign++;
                    if (cross_log_cnt < 8) cross_logged[cross_log_cnt++] = cross;
                    n_cross++;
                }
                const char *cat;
                if (nmax < 64) cat = "SILENT";
                else if (same_sign >= 8) cat = "TONAL_FB";
                else cat = "MODULATED";
                if (db_log < LIMIT) {
                    BSP_LOG("BURST-IN fn=%u tn=%u %s nmax=%d cross0=%d same=%d/10 "
                            "signs=%d,%d,%d,%d,%d,%d,%d,%d",
                            (unsigned)sl->fn, (unsigned)tn, cat,
                            nmax, cross0, same_sign,
                            cross_logged[0], cross_logged[1], cross_logged[2],
                            cross_logged[3], cross_logged[4], cross_logged[5],
                            cross_logged[6], cross_logged[7]);
                    db_log++;
                    if (db_log == LIMIT)
                        BSP_LOG("BURST-IN log capped at %u", LIMIT);
                }
                /* Real FB detection (TONAL_FB == genuine FCCH tone, same_sign>=8)
                 * -> write the DSP's OWN NDB cells directly: the REAL DSP path
                 * the ARM reads via API RAM 0x01F0 (d_fb_det) / 0x01F4
                 * (a_sync_demod). Scales per calypso_layer1.c reference:
                 *   toa = 23 (on-time; ARM does toa-=23 -> 0)
                 *   pm  = 0x7000 full-scale Q15 (ARM reads >>3)
                 *   ang = 0 (residual ~0 for clean carrier-aligned IQ)
                 *   snr proportional to coherence, > AFC_SNR_THRESHOLD(2560) locked
                 * Runs EVERY burst (NOT gated by the log cap). */
                if (calypso_orch() && bsp.dsp) {   /* ORCH only: exe = real DSP owns d_fb_det */
                    calypso_pcb_daram_lock_acquire();
                    if (same_sign >= 8 && nmax >= 64) {
                        bsp.dsp->data[0x08FA] = (uint16_t)23;
                        bsp.dsp->data[0x08FB] = (uint16_t)0x7000;
                        bsp.dsp->data[0x08FC] = (uint16_t)0;
                        bsp.dsp->data[0x08FD] =
                            (uint16_t)((same_sign * 0x7000) / 10);
                        bsp.dsp->data[0x08F8] = 1;   /* d_fb_det = FOUND */
                    } else {
                        bsp.dsp->data[0x08F8] = 0;   /* not found */
                    }
                    calypso_pcb_daram_lock_release();
                }
            }
        }

        /* ⚠️ TESTING 2026-05-29 : marqueur (cette fonction boucle-t-elle ?)
         * + apply_phase ICI (delivery, dac courant) — théorie : le chemin
         * vivant n'appliquait pas l'AFC sur les samples livrés au corrélateur. */
        {
            static unsigned dlv_n;
            if (calypso_debug_enabled("BSP-DELIVER") &&
                (dlv_n <= 20 || dlv_n % 2000 == 0))
                fprintf(stderr, "[BSP] BSP-DELIVER #%u fn=%u tn=%u n=%d (apply AFC)\n",
                        dlv_n, (unsigned)sl->fn, (unsigned)tn, n);
            dlv_n++;
        }
        calypso_twl3025_apply_phase(sl->iq, sl->n / 2, sl->fn, (uint8_t)tn);

        uint16_t samples[296];
        for (int i = 0; i < n && i < 296; i++)
            samples[i] = (uint16_t)sl->iq[i];
        c54x_bsp_load(bsp.dsp, samples, n > 296 ? 296 : n);

        /* ⚠️ TESTING : woff LOCAL (était static rolling cross-burst). */
        unsigned woff = 0;
        calypso_pcb_daram_lock_acquire();
        for (int i = 0; i < n; i++) {
            uint16_t a = (uint16_t)(bsp.daram_addr + woff);
            /* HACK CALYPSO_BSP_INJECT_CANARY : overwrite avec marker 0xCAFE
             * pour identifier le vrai buffer cible cote DSP via le hook
             * canary-read en c54x. Voir doc/TODO.md. */
            uint16_t v = bsp.inject_canary ? 0xCAFE : (uint16_t)sl->iq[i];
            bsp.dsp->data[a] = v;
            bsp_daram_wr_bucket(a);
            woff++;
            if (woff >= bsp.daram_len) woff = 0;
        }
        calypso_pcb_daram_lock_release();
        bsp.bursts_written++;

        /* PROBE 2026-05-31 fork-1 : dump I/Q (chemin deliver_buffered, le VIVANT
         * = samples post-AFC livrés au corrélateur). Gated CALYPSO_IQDUMP,
         * compteur indépendant, préfixe iq_dlv. À RETIRER. */
        if (getenv("CALYPSO_IQDUMP")) {
            static unsigned dlv_dump_n;
            if (dlv_dump_n < 24) {
                char path[80];
                snprintf(path, sizeof(path), "/tmp/iq_dlv_%03u.bin", dlv_dump_n);
                FILE *f = fopen(path, "wb");
                if (f) {
                    for (int i = 0; i < n; i++) {
                        int16_t s = (int16_t)sl->iq[i];
                        fwrite(&s, sizeof(int16_t), 1, f);
                    }
                    fclose(f);
                    BSP_LOG("IQDUMP dlv #%u fn=%u tn=%u → %s (%d int16)",
                            dlv_dump_n, (unsigned)sl->fn, (unsigned)tn, path, n);
                }
                dlv_dump_n++;
            }
        }
        sl->valid = false;  /* consumed */

        /* === BRINT0 assert (2026-05-28) =====================================
         * Fire BRINT0 IRQ (vec 21, IMR bit 5) after DARAM write. Sur silicon,
         * BSP DMA-complete declenche cette IRQ qui reveille le DSP et execute
         * l'ISR a PROM1[0xFFD4 → CALL 0xf310]. Sans cet assert, le DSP ne sait
         * jamais qu'un burst est disponible et reste dans son dispatcher loop
         * polling data[0x3fab] eternellement (= 59M reads observed).
         * Confirme par chain analysis 2026-05-28 :
         *   1. Canary 0xCAFE prouve E2E BSP→DSP read at 0x2a00 (PC=0x93a5)
         *   2. DSP polls *(0x3fab) bits via dispatcher table at data[0x16b3]
         *   3. *(0x3fab) bits sont OR'ed par ISR triggered par BRINT0
         *   4. Sans BRINT0 → pas d'ISR → pas de bit set → loop infini */
        /* Gate : skip si BRINT0 précédent pas encore servi par DSP — évite
         * iota pending queue overflow quand DSP traite ISR plus lentement
         * que BSP rate (= GSM 217 Hz wall vs DSP-processed BRINT0). */
        if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
            c54x_interrupt_ex(bsp.dsp, 21, 5);
        }

        /* RX I/Q tap : si BSP_DUMP_RX_FILE est set, append le burst brut
         * (n int16_t LE I/Q interleaved) au fichier. Header 12B par burst :
         *   magic 'IQ16' (4B) | fn (4B LE) | tn (1B) | n_int16 (2B LE) | _pad (1B)
         * Permet ensuite python3 fcch_ref.py <dump> --fmt int16 --burst N. */
        {
            static FILE *rx_dump_f = NULL;
            static int   rx_dump_init = 0;
            if (!rx_dump_init) {
                rx_dump_init = 1;
                const char *p = getenv("BSP_DUMP_RX_FILE");
                if (p && *p) {
                    rx_dump_f = fopen(p, "ab");
                    BSP_LOG("BSP_DUMP_RX_FILE='%s' fopen=%s",
                            p, rx_dump_f ? "ok" : strerror(errno));
                } else {
                    BSP_LOG("BSP_DUMP_RX_FILE not set (p=%p p[0]=%c)",
                            (void *)p, p ? p[0] : '?');
                }
            }
            if (rx_dump_f) {
                uint8_t hdr[12] = {
                    'I','Q','1','6',
                    (uint8_t)(sl->fn      ), (uint8_t)(sl->fn >>  8),
                    (uint8_t)(sl->fn >> 16), (uint8_t)(sl->fn >> 24),
                    tn,
                    (uint8_t)(n      ), (uint8_t)(n >> 8),
                    0
                };
                fwrite(hdr, 1, 12, rx_dump_f);
                fwrite(sl->iq, sizeof(int16_t), n, rx_dump_f);
                fflush(rx_dump_f);
            }
        }

        if (bsp.bursts_written <= 10 || (bsp.bursts_written % 1000) == 0) {
            BSP_LOG("DMA tn=%u fn=%u n=%d total=%llu stale=%llu qfull=%llu",
                    tn, sl->fn, n,
                    (unsigned long long)bsp.bursts_written,
                    (unsigned long long)bsp.bursts_dropped_stale,
                    (unsigned long long)bsp.bursts_dropped_queue_full);

            /* Dump first 8 words written so we can verify the I/Q
             * constellation actually landed in the DSP data memory at
             * daram_addr — independent of any ARM-side mapping. */
            calypso_pcb_daram_lock_acquire();
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
            calypso_pcb_daram_lock_release();
        }

        /* Fire BRINT0 */
        if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
            c54x_interrupt_ex(bsp.dsp, 21, 5);
            if (bsp.dsp->idle) bsp.dsp->idle = false;
        }
        }  /* end while drain */
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

    /* Hex dump of every UL burst as it's sent — symmetric with the calypso-ipc-device
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
    calypso_pcb_daram_lock_acquire();
    for (int i = 0; i < 148; i++) {
        uint16_t w = bsp.dsp->data[0x0900 + i];
        bits[i] = (uint8_t)(w & 1);
        if (bits[i]) any = true;
    }
    calypso_pcb_daram_lock_release();

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
    uint16_t d_rach = calypso_dsp_daram_read(bsp.dsp, 0x0800 + off);
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
