/*
 * qemu_wrap.c — backend QEMU pour calypso-ipc-device.
 *
 * Remplace osmo-trx/.../ipc/uhdwrap.cpp : à la place d'un device UHD physique,
 * notre source de samples est le BSP QEMU émulé (UDP 6702).
 *
 * Phase 1 — Proof of Life (ce fichier dans son état actuel) :
 *   - Accepte le handshake greeting/info/open/start d'osmo-trx-ipc.
 *   - uhdwrap_read produit un heartbeat continu de zéros cs16 → ul_stream.
 *     Cadence l'horloge osmo-trx (qui lit les timestamps UL comme master clock).
 *   - uhdwrap_write consomme silencieusement les bursts DL shm
 *     (à câbler vers UDP 6702 en Phase 1.5 / Task #6).
 *   - Les autres hooks (gain, freq, txatt, start, stop) sont no-op success.
 *
 * Specs Calypso :
 *   1 channel, fs = 270 833 Hz (= 13e6/48), 1 SPS, cs16 I/Q entrelacé.
 *   148 samples par burst (matches BSP encoder window côté QEMU).
 *
 * SPDX-License-Identifier: 0BSD
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <osmocom/core/logging.h>
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>
#include <osmocom/gsm/a5.h>      /* osmo_a5() : chiffrement A5/1 UL */

#include "debug.h"
#include "ipc_shm.h"
#include "shm.h"
#include "uhdwrap.h"

/* Specs Calypso baseband GSM. */
#define CALYPSO_FS_NUM        13000000u   /* 13 MHz GSM master clock */
#define CALYPSO_FS_DEN        48u         /* /48 → 270 833.33 Hz */

/* osmo-trx-ipc has a hard-coded CHUNK=625 (radioInterface.cpp:36). It always
 * commits buffers of CHUNK*tx_sps samples to the device shm — at 1 SPS = 625
 * samples per write = 4 GSM timeslots = half TDMA frame. So our shm buffer
 * must be sized for that. We accept the 625 samples and extract only the
 * first 148 (TS=0) before forwarding to QEMU BSP (which expects 148-sample
 * bursts in its TRXD UDP datagram). The remaining 477 samples (TS 1..3 of
 * the half-frame) are dropped — FBSB only listens on C0 TN=0. */
#define CALYPSO_SHM_BUFSIZE   2500         /* samples per shm commit (matches osmo-trx CHUNK at 1 SPS) */
#define CALYPSO_TRX_OSR       4                                     /* 4 SPS natif */
#define CALYPSO_DL_BURSTLEN   (CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR)  /* 592 I/Q @ 4 SPS */
#define CALYPSO_FRAME_SAMPLES (1250 * CALYPSO_TRX_OSR)              /* 5000 samples/frame @ 4 SPS */
#define CALYPSO_BSP_BURSTLEN  148         /* samples per UDP datagram to QEMU BSP (= correlator window) */
/* FIX LU 2026-06-05 : guard de tete (complex samples) AVANT les bits actifs de
 * la RACH UL. Le correlateur RACH osmo-trx (sigProcLib.cpp:1683 TOA gate <3*sps,
 * :1788 target ~sym48) rejette un burst place a l'offset 0 du slot (pic en bord
 * -> rejete -> NOPE/-110). Un vrai access-burst a ~68 sym de guard avant la sync.
 * ~32 sym @ OSR4 = 128 samples placent la sync dans la fenetre du correlateur. */
#define CALYPSO_UL_SLOT_OFFSET 128

/* ---- Timing frame CANONIQUE (logique GSM, robuste) ----
 * 1 frame TDMA = CALYPSO_FRAME_QBITS qbits (1250 symboles x 4) = CALYPSO_FRAME_NS.
 * Le budget DSP n'est PAS hardcode : le gating se fait sur le qfn du firmware
 * (g_qemu_qfn), qui avance quand le firmware a fini sa frame = budget DSP consomme
 * implicitement. On suit la frame REELLE du firmware, pas une constante devinee. */
#define CALYPSO_FRAME_QBITS   5000
#define CALYPSO_FRAME_NS      4615384L   /* 5000 qbits / 1083333.33 qbits/s = 60/13 ms */
#define CALYPSO_NUM_CHANS     1
#define CALYPSO_PATH_NAME     "TX"        /* placeholder ; matches osmo-trx-ipc.cfg */
#define CALYPSO_RX_PATH_NAME  "RX"

/* QEMU BSP UDP endpoint. Matches the legacy calypso-ipc-device target — QEMU's
 * calypso_bsp.c binds on this. Override via env if needed. */
#define QEMU_BSP_HOST_DEFAULT "127.0.0.1"
#define QEMU_BSP_PORT_DEFAULT 6702

/* GSM TDMA timing at 1 SPS. 1 TS ≈ 156.25 samples, 8 TS per frame.
 * SAMPLES_PER_FRAME = 1250 = 8 × 156.25 (= 156.25 × 8).
 * Hyperframe = 2715648 frames (GSM 05.02 §3.1). */
#define SAMPLES_PER_FRAME     1250u
#define GSM_HYPERFRAME        2715648u

/* TRXDv0 datagram header = 8 bytes :
 *   [0]   version(4) | TN(4)        — calypso-ipc-device reads `tn = data[0] & 7`
 *   [1-4] FN, big-endian (4 bytes)
 *   [5]   RSSI (uint8 dBm-ish)      — not consumed by Calypso BSP for DL
 *   [6-7] ToA q4 (int16, optional)  — not consumed by Calypso BSP for DL
 * Payload = 4 × num_samples bytes (cs16 I,Q interleaved). */
#define TRXD_HDR_LEN          8

/* Heartbeat pacing. 148 samples × (CALYPSO_FS_DEN / CALYPSO_FS_NUM) sec
 * = 148 × 48 / 13e6 = 546.5 µs. usleep ≥ 1 ms granularity in pratique,
 * so we pace at 500 µs and let osmo-trx absorb the ~9 % overproduction
 * (it will read at its native rate and discard / buffer accordingly).
 */
#define READ_PACE_US          500

/* Shared with calypso_ipc_device.c : these are populated in ipc_rx_open_req
 * after ipc_shm_init_producer() / consumer(). */
extern struct ipc_shm_io *ios_tx_to_device[8];   /* DL stream : osmo-trx writes, we read */
extern struct ipc_shm_io *ios_rx_from_device[8]; /* UL stream : we write, osmo-trx reads */

struct qemu_dev {
    uint32_t num_chans;
    uint64_t rx_ts;          /* cumulative sample timestamp for UL writes */
    bool     started[8];
};

/* UDP socket to QEMU BSP. Lazy-init on first qemu_wrap_write call so we don't
 * need to thread it through open(). */
static int            g_bsp_fd = -1;
static struct sockaddr_in g_bsp_peer;
static pthread_mutex_t g_bsp_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ---- Fix D : DL FIFO qfn-paced ----
 *
 * Without this, the device read shm at osmo-trx wall pace (~209 chunks/s)
 * and forwarded each one to UDP 6702. QEMU (under icount=auto) consumed only
 * ~10 fn/s → 21 bursts tagged with the same qfn → 95 % dropped → FCCH
 * (5/51 frames) almost never reached the DSP correlator.
 *
 * Strategy : ordered FIFO, 1 burst per qfn, no phase match.
 *   - qemu_wrap_write : append TS=0 burst to FIFO tail (on-air order).
 *   - clk_listener : on each qfn tick, pop FIFO head, tag fn=qfn,
 *     sendto 6702. One burst per qfn → cadence calé sur QEMU.
 *
 * Why no qfn↔on-air phase match : during cold acquisition the MS does
 * not yet know on-air FN ; qfn is an arbitrary internal counter. The
 * mapping qfn↔on-air is exactly what FCCH+SCH establish. Phase-matching
 * before that requires data we don't have. The FIFO instead preserves
 * on-air order ; FB correlator scans tone-only (FN-agnostic) and locks
 * in ~1-2s ; once SCH is decoded, the MS adopts the on-air FN encoded
 * in it, and from then on its qfn matches the tag we're applying →
 * BCCH lecture devient cohérente automatiquement.
 *
 * Scope : ce fix donne FBSB_CONF + BCCH. PAS la LU — comme le device
 * lit à 20× le débit de consommation QEMU, la FIFO accumule un lag de
 * plusieurs secondes ; pour UL RACH ce lag est fatal (BTS rejette les
 * RACH au FN périmé). LU = autre combat, exige horloges réelles. */
#define DL_FIFO_SIZE 4096
/* Coussin de pré-fill (fix 2026-05-30) : on ne sert pas le 1er burst tant que
 * la FIFO DL n'a pas atteint DL_PREFILL entrées. Établit un buffer qui absorbe
 * les spikes de jitter entre l'horloge QEMU (clk_listener) et le heartbeat
 * device (uhdwrap_read) — deux horloges libres. Sans ça, la profondeur ~2 se
 * vide au moindre spike → "FIFO empty" → osmo-trx RX error → IPC LOST → le BSP
 * n'est jamais nourri (D_BURST_D vide, snr=0). 32 frames ≈ 148 ms de marge. */
#define DL_PREFILL 32
struct dl_fifo_entry {
    bool     is_fcch;  /* for diag log only */
    uint64_t ts;       /* internal osmo-trx ts (for diag) */
    /* Pre-built TRXDv0 packet, header rewritten at send time with qfn. */
    uint8_t  pkt[TRXD_HDR_LEN + CALYPSO_DL_BURSTLEN * 4];
};
static struct dl_fifo_entry g_dl_fifo[DL_FIFO_SIZE];
static volatile size_t      g_dl_fifo_head = 0;   /* next pop index */
static volatile size_t      g_dl_fifo_tail = 0;   /* next push index */
static pthread_mutex_t      g_dl_fifo_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile uint32_t    g_last_qfn_sent = UINT32_MAX;

/* GMSK signature : a FCCH burst (148 zero bits) has dphi = +π/2 every
 * sample at 1 SPS. We measure the fraction of positive dphi samples ;
 * ≥ 95 % positive = FCCH. Same logic as tools/dump_chunks_pattern.py. */
static bool is_fcch_burst_iq(const int16_t *iq, int n_samples)
{
    if (n_samples < 16) return false;
    int positives = 0;
    float prev_a = atan2f((float)iq[1], (float)iq[0]);
    for (int i = 1; i < n_samples; i++) {
        float a = atan2f((float)iq[2 * i + 1], (float)iq[2 * i]);
        float d = a - prev_a;
        while (d > (float)M_PI)  d -= 2.0f * (float)M_PI;
        while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
        if (d > 0.0f) positives++;
        prev_a = a;
    }
    return positives >= (n_samples - 1) * 95 / 100;
}

/* ---- QEMU clock sync (Option A) ----
 * QEMU sends a 4-byte BE FN to 127.0.0.1:6700 on every TDMA tick
 * (calypso_trx.c:1434+). We bind that port in a listener thread and use the
 * resulting FN to (1) pace the UL heartbeat so osmo-trx clock advances at
 * QEMU's effective rate (not wall-clock), and (2) tag outbound DL datagrams
 * with the QEMU current FN so the BSP queue accepts them (within its
 * 64-frame match window).
 *
 * Without this, under icount=auto QEMU runs ~25× slower than wall — our
 * heartbeat advanced rx_ts at 217 fn/s while QEMU was at ~8.4 fn/s. Result:
 * osmo-bts-trx bursts arrived with stale fn (delta thousands), all dropped,
 * and the scheduler spammed STALE log lines that caused the visible hang. */
#define QEMU_CLK_PORT 6700
static volatile uint32_t g_qemu_qfn = 0;
static volatile int      g_qfn_seen = 0;
static int               g_clk_fd = -1;
static pthread_t         g_clk_thread;
extern volatile int      ipc_exit_requested;

static void *clk_listener(void *arg)
{
    (void)arg;
    pthread_setname_np(pthread_self(), "qemu_clk_rx");

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        LOGP(DDEV, LOGL_ERROR, "clk_listener: socket() failed: %s\n", strerror(errno));
        return NULL;
    }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(QEMU_CLK_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOGP(DDEV, LOGL_ERROR, "clk_listener: bind 6700 failed: %s\n", strerror(errno));
        close(fd);
        return NULL;
    }
    g_clk_fd = fd;
    LOGP(DDEV, LOGL_NOTICE, "clk_listener: bound 127.0.0.1:%d, waiting QEMU ticks\n",
         QEMU_CLK_PORT);

    uint8_t pkt[64];
    while (!ipc_exit_requested) {
        ssize_t n = recvfrom(fd, pkt, sizeof(pkt), 0, NULL, NULL);
        if (n < 4) continue;
        uint32_t fn = ((uint32_t)pkt[0] << 24) | ((uint32_t)pkt[1] << 16)
                    | ((uint32_t)pkt[2] << 8)  |  (uint32_t)pkt[3];
        __atomic_store_n(&g_qemu_qfn, fn, __ATOMIC_RELEASE);
        if (!g_qfn_seen) {
            __atomic_store_n(&g_qfn_seen, 1, __ATOMIC_RELEASE);
            LOGP(DDEV, LOGL_NOTICE,
                 "clk_listener: first QEMU tick received, qfn=%u\n", fn);
        }

        /* ---- Fix D : pop FIFO head, tag with qfn, send ----
         * 1 burst per qfn tick from QEMU → cadence matches QEMU's
         * effective rate ; no overflow, no drop, no phase reasoning.
         * On-air order is preserved by the FIFO ; the MS will adopt the
         * encoded FN once it decodes SCH, locking the tag↔content. */
        if (g_bsp_fd < 0)
            continue;
        uint32_t last = __atomic_load_n(&g_last_qfn_sent, __ATOMIC_ACQUIRE);
        if (fn == last) continue; /* dedup duplicate qfn ticks */
        __atomic_store_n(&g_last_qfn_sent, fn, __ATOMIC_RELEASE);

        pthread_mutex_lock(&g_dl_fifo_mutex);
        size_t head = g_dl_fifo_head;
        size_t tail = g_dl_fifo_tail;
        /* Pré-fill : attendre un coussin DL_PREFILL avant de servir le 1er
         * burst (puis on sert normalement 1/tick). Le coussin absorbe ensuite
         * les spikes de jitter sans jamais retomber à 0. */
        static int s_prefilled = 0;
        if (!s_prefilled) {
            if (tail - head < DL_PREFILL) {
                pthread_mutex_unlock(&g_dl_fifo_mutex);
                continue;   /* laisse la FIFO se remplir, ne consomme pas le tick */
            }
            s_prefilled = 1;
            LOGP(DDEV, LOGL_NOTICE,
                 "DL FIFO pre-filled to %d, starting to serve at qfn=%u\n",
                 DL_PREFILL, fn);
        }
        if (head == tail) {
            /* Empty FIFO — nothing to serve this tick. */
            pthread_mutex_unlock(&g_dl_fifo_mutex);
            static uint64_t empty_count = 0;
            if (empty_count++ < 5)
                LOGP(DDEV, LOGL_INFO, "FIFO empty at qfn=%u\n", fn);
            continue;
        }
        struct dl_fifo_entry *e = &g_dl_fifo[head % DL_FIFO_SIZE];
        /* Patch fn into header : la VRAIE FN du burst (depuis e->ts), PAS le qfn
         * courant. Sinon la latence FIFO (DL_PREFILL=32) decale la FN de ~32
         * frames -> fn%51 faux -> blocs BCCH mal assembles -> decode foire.
         * LA derniere piece : fifo_depth=32 scramblait la FN. */
        uint32_t bfn = (uint32_t)(e->ts / ((uint64_t)CALYPSO_FRAME_SAMPLES));
        e->pkt[0] = 0; /* tn=0 */
        e->pkt[1] = (uint8_t)(bfn >> 24);
        e->pkt[2] = (uint8_t)(bfn >> 16);
        e->pkt[3] = (uint8_t)(bfn >>  8);
        e->pkt[4] = (uint8_t)(bfn);
        ssize_t sent = sendto(g_bsp_fd, e->pkt,
                              TRXD_HDR_LEN + CALYPSO_DL_BURSTLEN * 4, 0,
                              (struct sockaddr *)&g_bsp_peer,
                              sizeof(g_bsp_peer));
        bool was_fcch = e->is_fcch;
        uint64_t ets = e->ts;
        g_dl_fifo_head = head + 1;
        size_t depth = tail - g_dl_fifo_head;
        pthread_mutex_unlock(&g_dl_fifo_mutex);

        static uint64_t qsend_count = 0;
        if (qsend_count < 10 || (qsend_count % 500) == 0 || was_fcch) {
            LOGP(DDEV, LOGL_INFO,
                 "qfn-serve #%llu qfn=%u ts=%llu%s fifo_depth=%zu sent=%zd\n",
                 (unsigned long long)qsend_count, fn,
                 (unsigned long long)ets,
                 was_fcch ? " *FCCH*" : "", depth, sent);
        }
        qsend_count++;
    }
    close(fd);
    g_clk_fd = -1;
    return NULL;
}

static int bsp_udp_init(void)
{
    pthread_mutex_lock(&g_bsp_mutex);
    if (g_bsp_fd >= 0) {
        pthread_mutex_unlock(&g_bsp_mutex);
        return 0;
    }

    const char *host = getenv("CALYPSO_BSP_HOST");
    const char *port_s = getenv("CALYPSO_BSP_PORT");
    if (!host || !*host) host = QEMU_BSP_HOST_DEFAULT;
    uint16_t port = (port_s && *port_s) ? (uint16_t)atoi(port_s) : QEMU_BSP_PORT_DEFAULT;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        LOGP(DDEV, LOGL_ERROR, "bsp_udp_init: socket() failed: %s\n", strerror(errno));
        pthread_mutex_unlock(&g_bsp_mutex);
        return -1;
    }
    memset(&g_bsp_peer, 0, sizeof(g_bsp_peer));
    g_bsp_peer.sin_family = AF_INET;
    g_bsp_peer.sin_port = htons(port);
    if (inet_aton(host, &g_bsp_peer.sin_addr) == 0) {
        LOGP(DDEV, LOGL_ERROR, "bsp_udp_init: invalid host '%s'\n", host);
        close(fd);
        pthread_mutex_unlock(&g_bsp_mutex);
        return -1;
    }
    g_bsp_fd = fd;
    LOGP(DDEV, LOGL_NOTICE, "bsp_udp_init: TRXDv0 → %s:%u (fd=%d)\n", host, port, fd);
    pthread_mutex_unlock(&g_bsp_mutex);
    return 0;
}

/* Compute (FN, TN) from a sample timestamp. FBSB only listens on C0 TN=0 so
 * we tag all bursts with TN=0 — sufficient until SDCCH/RACH phase.
 * Currently unused (Phase 1 uses live g_qemu_qfn instead), kept for Phase 2
 * slot-rewrite that needs bts_fn % 51. */
__attribute__((unused))
static void ts_to_fn_tn(uint64_t ts, uint32_t *fn_out, uint8_t *tn_out)
{
    uint64_t frame = ts / SAMPLES_PER_FRAME;
    *fn_out = (uint32_t)(frame % GSM_HYPERFRAME);
    *tn_out = 0;
}

/* Build the 8-byte TRXDv0 header into out[0..7]. */
static void trxd_build_hdr(uint8_t out[TRXD_HDR_LEN], uint32_t fn, uint8_t tn)
{
    out[0] = (tn & 0x07);            /* version=0 in high nibble, TN in low 3 */
    out[1] = (uint8_t)(fn >> 24);
    out[2] = (uint8_t)(fn >> 16);
    out[3] = (uint8_t)(fn >> 8);
    out[4] = (uint8_t)(fn);
    out[5] = 0; /* RSSI placeholder */
    out[6] = 0; /* ToA hi */
    out[7] = 0; /* ToA lo */
}

/* ---- open / close ---- */

void *uhdwrap_open(struct ipc_sk_if_open_req *open_req)
{
    struct qemu_dev *d = calloc(1, sizeof(*d));
    if (!d) {
        LOGP(DDEV, LOGL_ERROR, "qemu_wrap_open: calloc failed\n");
        return NULL;
    }
    d->num_chans = open_req->num_chans;
    d->rx_ts = 0;

    LOGP(DDEV, LOGL_NOTICE,
         "qemu_wrap_open: num_chans=%u clockref=0x%x rx_fs=%u/%u tx_fs=%u/%u bw=%u\n",
         open_req->num_chans, open_req->clockref,
         open_req->rx_sample_freq_num, open_req->rx_sample_freq_den,
         open_req->tx_sample_freq_num, open_req->tx_sample_freq_den,
         open_req->bandwidth);

    /* Start the QEMU clock listener (binds UDP 6700, receives 4 B BE FN
     * on every QEMU tdma tick). Idempotent : skip if already running. */
    static bool clk_started = false;
    if (!clk_started) {
        if (pthread_create(&g_clk_thread, NULL, clk_listener, NULL) == 0) {
            clk_started = true;
        } else {
            LOGP(DDEV, LOGL_ERROR,
                 "qemu_wrap_open: pthread_create(clk_listener) failed\n");
        }
    }

    return d;
}

/* ---- info_cnf : reply to osmo-trx-ipc capability query ---- */

void uhdwrap_fill_info_cnf(struct ipc_sk_if *ipc_prim)
{
    struct ipc_sk_if_info_cnf *info = &ipc_prim->u.info_cnf;
    memset(info, 0, sizeof(*info));

    info->feature_mask = FEATURE_MASK_CLOCKREF_EXTERNAL;
    /* iq_scaling : cs16 full range 1.0 — we don't scale ourselves */
    info->iq_scaling_val_rx = 1.0;
    info->iq_scaling_val_tx = 1.0;
    info->max_num_chans = CALYPSO_NUM_CHANS;
    snprintf(info->dev_desc, sizeof(info->dev_desc),
             "calypso-ipc-device (QEMU UDP 6702 bridge), GSM %d SPS %.0f Hz",
             CALYPSO_TRX_OSR,
             (double)CALYPSO_FS_NUM / (double)CALYPSO_FS_DEN * CALYPSO_TRX_OSR);

    for (size_t i = 0; i < CALYPSO_NUM_CHANS; i++) {
        struct ipc_sk_if_info_chan *ci = &info->chan_info[i];
        snprintf(ci->tx_path[0], RF_PATH_NAME_SIZE, "%s", CALYPSO_PATH_NAME);
        snprintf(ci->rx_path[0], RF_PATH_NAME_SIZE, "%s", CALYPSO_RX_PATH_NAME);
        ci->min_rx_gain = 0.0;
        ci->max_rx_gain = 100.0;
        ci->min_tx_gain = 0.0;
        ci->max_tx_gain = 100.0;
        ci->nominal_tx_power = 0.0; /* dBm — placeholder */
    }

    LOGP(DDEV, LOGL_INFO, "qemu_wrap_fill_info_cnf: 1 chan, fs=%.0f Hz, %d SPS\n",
         (double)CALYPSO_FS_NUM / (double)CALYPSO_FS_DEN * CALYPSO_TRX_OSR, CALYPSO_TRX_OSR);
}

/* ---- buffer sizing + timing ---- */

int32_t uhdwrap_get_bufsizerx(void *dev)
{
    (void)dev;
    return CALYPSO_SHM_BUFSIZE;
}

int32_t uhdwrap_get_timingoffset(void *dev)
{
    (void)dev;
    return 0; /* no analog pipeline → no path delay to compensate */
}

/* ---- start / stop ---- */

int32_t uhdwrap_start(void *dev, int chan)
{
    struct qemu_dev *d = dev;
    if (!d || chan < 0 || chan >= 8) return 0;

    bool was_started = d->started[chan];
    d->started[chan] = true;

    LOGP(DDEV, LOGL_NOTICE, "qemu_wrap_start chan=%d (first=%d)\n",
         chan, !was_started);

    /* Convention ipc-driver-test (cf. ipc_rx_chan_start_req in our fork) :
     * a non-zero return on the FIRST chan_start triggers the global RX/TX
     * thread creation (uplink_thread + downlink_thread). Subsequent chan
     * starts return 0 so we don't spawn duplicate threads. */
    return was_started ? 0 : 1;
}

int32_t uhdwrap_stop(void *dev, int chan)
{
    struct qemu_dev *d = dev;
    if (!d || chan < 0 || chan >= 8) return 0;
    d->started[chan] = false;
    LOGP(DDEV, LOGL_NOTICE, "qemu_wrap_stop chan=%d\n", chan);
    return 1;
}

/* ---- gain / freq / txatt : no-op echoes ---- */

double uhdwrap_set_gain(void *dev, double g, size_t chan, bool for_tx)
{
    (void)dev;
    LOGP(DDEV, LOGL_INFO, "qemu_wrap_set_gain chan=%zu %s=%.1f (no-op)\n",
         chan, for_tx ? "tx" : "rx", g);
    return g;
}

double uhdwrap_set_freq(void *dev, double f, size_t chan, bool for_tx)
{
    (void)dev;
    LOGP(DDEV, LOGL_INFO, "qemu_wrap_set_freq chan=%zu %s=%.0f Hz (no-op)\n",
         chan, for_tx ? "tx" : "rx", f);
    /* ipc_rx_chan_setfreq_req does `return_code = rv ? 0 : 1`. So returning
     * 1.0 here (non-zero / true) yields return_code=0 → osmo-trx-ipc sees
     * success. Returning 0.0 would mean failure. */
    return 1.0;
}

double uhdwrap_set_txatt(void *dev, double a, size_t chan)
{
    (void)dev;
    LOGP(DDEV, LOGL_INFO, "qemu_wrap_set_txatt chan=%zu att=%.1f (no-op)\n",
         chan, a);
    return a;
}

/* ============================================================================
 * UL (IPC TX) : le BSP qemu envoie les bursts UL du mobile en TRXDv0 (8 hdr +
 * 148 soft-bits ±127) vers 127.0.0.1:5702. On les reçoit, on les MODULE en
 * GMSK I/Q (osmo-trx attend de l'I/Q), et on les injecte dans le slot TS0 du
 * chunk UL au lieu des zéros. Opt-in CALYPSO_IPC_UL=1 (défaut off → heartbeat).
 * Sync : best-effort — on place le dernier burst reçu sur le prochain chunk TS0.
 * L'alignement FN fin se règle quand le mobile TX réellement (post-camp).
 * ============================================================================ */
#include <math.h>
#define UL_TRXD_HDR      8
static int  g_ul_on   = -1;            /* CALYPSO_IPC_UL */
/* FIX OSR 2026-06-04 : osmo-trx tourne a CALYPSO_TRX_OSR=4 SPS. Le modulateur
 * DOIT produire 148 symboles * OSR samples (= 592 @ 4 SPS), sinon les 148
 * samples 1-SPS sont lus comme ~37 symboles de charabia -> aucune correlation
 * d'access-burst cote osmo-trx -> NOPE -> RACH jamais detectee. */
static int16_t g_ul_iq[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2];   /* dernier burst modulé @ OSR */
static volatile int g_ul_pending = 0;  /* 1 = un burst à injecter */
static volatile uint32_t g_ul_real_fn = 0;  /* FN firmware (sideband) du dernier RACH -> FN-lock */
/* === RACH waveform DEDIE (FIX MT-SMS 2026-06-09) ============================
 * g_ul_iq est ECRASE a CHAQUE frame par le chemin SDCCH-idle (ul_mod_laurent ->
 * g_ul_iq, ~119x/run). Pour le LU c'etait masque : le firmware re-livrait la RACH
 * 30x sur g_bsp_fd, donc g_ul_iq etait re-rempli juste avant un slot eligible.
 * La paging-response (RA=0x98) n'est encodee QU'UNE fois -> entre l'encode et le
 * 1er vrai slot RACH (max 51 frames), le SDCCH-idle clobbe g_ul_iq -> meme avec la
 * gate corrigee, le burst inject serait du SDCCH, pas la RACH. On latch donc la
 * waveform RACH dans un buffer SEPARE (g_rach_iq) et on l'arme STICKY pour quelques
 * slots RACH-eligibles, re-injectee sur le 1er fn_ok. */
static int16_t g_rach_iq[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2]; /* waveform RACH latchee */
static volatile int g_rach_pending = 0;       /* nb de slots RACH-eligibles restants a tenter */
static volatile uint32_t g_rach_arm_seq = 0;  /* incremente a chaque nouvel arm (debug) */

/* Record .cfile de l'I/Q UL synthetise (fc32), symetrique du DL dsp_iq. Ecrit
 * UNIQUEMENT depuis le thread uhdwrap_read (ul_drain + uhdwrap_read) -> pas de
 * race, lazy-open suffisant. */
static FILE *g_ul_rec      = NULL;
static int   g_ul_rec_init = 0;

/* MSK phase-continue a OSR samples/symbole : 148 soft-bits (±127) -> 148*OSR
 * cs16 I/Q. Increment de phase ±(π/2)/OSR par SAMPLE (convention osmo-trx :
 * bit 1 → +π/2 par symbole). Amplitude ~0.6 full-scale (override CALYPSO_UL_AMP). */
static void ul_gmsk_mod(const int8_t *bits, int16_t *iq)
{
    static double AMP = -1.0;
    static int ACT = -2;
    if (AMP < 0.0) { const char *e = getenv("CALYPSO_UL_AMP"); AMP = (e && *e) ? atof(e) : 20000.0; }
    if (ACT == -2) { const char *e = getenv("CALYPSO_UL_ACTIVE_SYMS"); ACT = (e && *e) ? atoi(e) : -1; }
    /* ACCESS BURST (RACH) : seulement 88 symboles ACTIFS (8 tail + 41 sync etendu
     * + 36 data + 3 tail), puis 60 symboles de GUARD = SILENCE (IQ=0, PAS du GMSK :
     * un 0 GMSK-module est un tone fc/4, le correlateur RACH veut un gap d'energie).
     * 88*OSR=352 GMSK + 60*OSR=240 zeros = 592 = burst. Auto-detection access-vs-
     * normal : tail[0..7]==0 ET guard[88..147]==0 -> access burst. Override
     * CALYPSO_UL_ACTIVE_SYMS (>0 force, -1/unset = auto). */
    static int INV = -1, USEG = -1;
    if (INV < 0)  { const char *e = getenv("CALYPSO_UL_INVERT"); INV = (e && *e == '1') ? 1 : 0; }
    if (USEG < 0) { const char *e = getenv("CALYPSO_UL_GMSK");   USEG = (!e || *e != '0'); }  /* defaut GMSK */
    const int N = CALYPSO_BSP_BURSTLEN, OSR = CALYPSO_TRX_OSR, NS = N * OSR;
    int active = N;
    if (ACT > 0) active = ACT;
    /* FIX RACH FANTÔMES : plus d'auto-détection access-burst depuis le motif de bits.
     * Le repli non-RACH (bits BSP idle, souvent tail0+guard0) était modulé en
     * access-burst -> osmo-trx détectait des RACH RA=3 FANTÔMES (mesuré : 90 CHAN RQD
     * pour 6 vrais RACH) -> canaux SDCCH alloués sans SABM -> WAIT_RLL timeout ->
     * fuite -> épuisement du pool -> le SMS MO n'obtient plus de canal. Le VRAI RACH
     * passe par ul_mod_laurent (waveform osmo-trx exacte), JAMAIS par ul_gmsk_mod ;
     * le self-test aussi. Donc ici = toujours burst normal 148 sym (override possible
     * via CALYPSO_UL_ACTIVE_SYMS pour debug). */
    if (active > N) active = N;

    if (!USEG) {
        /* MSK fallback (CALYPSO_UL_GMSK=0) */
        double ph = 0.0; int idx = 0;
        for (int i = 0; i < N; i++) {
            if (i >= active) { for (int s=0;s<OSR;s++){iq[2*idx]=0;iq[2*idx+1]=0;idx++;} continue; }
            int b = ((bits[i] > 0) ? 1 : 0) ^ INV;
            double step = (b ? 1.0 : -1.0) * (M_PI/2.0)/(double)OSR;
            for (int s=0;s<OSR;s++){iq[2*idx]=(int16_t)(cos(ph)*AMP);iq[2*idx+1]=(int16_t)(sin(ph)*AMP);ph+=step;idx++;}
        }
        return;
    }

    /* GMSK BT=0.3 : pulse de frequence gaussien (osmo-trx correle du GMSK, pas du MSK).
     * freq[n] = Sum_k alpha[k]*g(n-k*OSR) ; phi = cumsum(freq)*pi/2 ; I/Q=AMP*(cos,sin). */
    #define GMSK_L 4
    static double g_pulse[GMSK_L * CALYPSO_TRX_OSR];
    static int g_init = 0;
    if (!g_init) {
        const double BT = 0.3, ln2 = 0.6931471805599453, kk = 2.0*M_PI*BT/sqrt(ln2);
        int Lo = GMSK_L*OSR; double sum = 0;
        for (int m = 0; m < Lo; m++) {
            double t = ((double)m - Lo/2.0 + 0.5)/OSR;   /* symboles, centre */
            double q1 = 0.5*erfc(kk*(t-0.5)/sqrt(2.0));
            double q2 = 0.5*erfc(kk*(t+0.5)/sqrt(2.0));
            g_pulse[m] = q1 - q2; sum += g_pulse[m];
        }
        if (sum != 0.0) for (int m = 0; m < Lo; m++) g_pulse[m] /= sum;  /* Sigma=1 -> pi/2 par symbole */
        g_init = 1;
    }
    static double freq[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR];
    for (int n = 0; n < NS; n++) freq[n] = 0.0;
    int Lo = GMSK_L*OSR;
    for (int k = 0; k < active; k++) {
        double al = (((bits[k] > 0) ? 1 : 0) ^ INV) ? 1.0 : -1.0;
        int base = k*OSR - Lo/2;
        for (int m = 0; m < Lo; m++) { int pos = base+m; if (pos >= 0 && pos < NS) freq[pos] += al*g_pulse[m]; }
    }
    /* ROTATION GMSK (osmo-trx GMSKRotate) : +pi/2 par SYMBOLE = +pi/(2*OSR) par sample.
     * Sans elle, le signal est decale de fs/(2*OSR) -> le demod osmo-trx (qui dé-rote
     * de la meme quantite) recoit du garbage -> BER 456/456. C'etait LA piece manquante
     * (le RACH echouait pareil). Gate CALYPSO_UL_ROT (def 1), signe CALYPSO_UL_ROT_SGN. */
    static int ROT = -2; static double ROTSGN = 1.0;
    if (ROT == -2) {
        const char *e = getenv("CALYPSO_UL_ROT");     ROT = (!e || *e != '0') ? 1 : 0;
        const char *s = getenv("CALYPSO_UL_ROT_SGN"); ROTSGN = (s && atoi(s) < 0) ? -1.0 : 1.0;
    }
    const double ROTSTEP = ROTSGN * (M_PI / 2.0) / (double)OSR;   /* pi/8 @ OSR=4 */
    double phi = 0.0; int active_end = active*OSR + Lo; if (active_end > NS) active_end = NS;
    for (int n = 0; n < NS; n++) {
        phi += (M_PI/2.0)*freq[n];
        double pt = ROT ? (phi + ROTSTEP * (double)n) : phi;
        if (n < active_end) { iq[2*n] = (int16_t)(cos(pt)*AMP); iq[2*n+1] = (int16_t)(sin(pt)*AMP); }
        else { iq[2*n] = 0; iq[2*n+1] = 0; }   /* guard silence */
    }
}

/* === Modulateur GMSK Laurent EXACT d'osmo-trx (port C de sigProcLib::modulateBurstLaurent).
 * Le GMSK maison ne correle pas le detecteur osmo-trx (BER 456/456) ; CE modulateur si
 * (la table RACH = son dump -> rc=3). bits[nbits] soft +/-1 -> symboles +/-1 @ sps=4,
 * rotation pi/(2*sps)/sample, convolution pulse Laurent c0 (+ c1 = j*XOR(b[i-1],b[i-2])),
 * somme. Sortie cs16 (scale CALYPSO_UL_AMP, def 20000) sur CALYPSO_BSP_BURSTLEN*OSR samples.
 * convolve START_ONLY : out[n] = sum_{j=0}^{H-1} in[n-(H-1)+j]*h[j] (in=0 si index<0). */
static void ul_mod_laurent(const int8_t *bits, int nbits, int16_t *iq)
{
    const int sps = CALYPSO_TRX_OSR;        /* 4 */
    const int BL  = 625;                     /* burst_len osmo-trx */
    static const double C0[16] = {
        0.0, 4.46348606e-03, 2.84385729e-02, 1.03184855e-01, 2.56065552e-01,
        4.76375085e-01, 7.05961177e-01, 8.71291644e-01, 9.29453645e-01,
        8.71291644e-01, 7.05961177e-01, 4.76375085e-01, 2.56065552e-01,
        1.03184855e-01, 2.84385729e-02, 4.46348606e-03 };
    static const double C1[8] = {
        0.0, 8.16373112e-03, 2.84385729e-02, 5.64158904e-02,
        7.05463553e-02, 5.64158904e-02, 2.84385729e-02, 8.16373112e-03 };
    static double AMP = -1.0;
    if (AMP < 0.0) { const char *e = getenv("CALYPSO_UL_AMP"); AMP = (e && *e) ? atof(e) : 20000.0; }
    if (nbits > 156) nbits = 156;

    static double sym[625], c0r[625], c0i[625], c1r[625], c1i[625];
    for (int n = 0; n < BL; n++) { sym[n]=0; c0r[n]=0; c0i[n]=0; c1r[n]=0; c1i[n]=0; }
    int b[160]; for (int i = 0; i < nbits; i++) b[i] = (bits[i] > 0) ? 1 : 0;

    /* symboles +/-1 : index 0 = padding tail(-1), sps,2sps.. = 2*bit-1, puis padding tail. */
    int idx = 0;
    sym[idx] = -1.0; idx += sps;
    for (int i = 0; i < nbits; i++) { sym[idx] = 2.0*b[i]-1.0; idx += sps; }
    if (idx < BL) sym[idx] = -1.0;

    /* rotation GMSK : c0[n] = sym[n] * e^(j n pi/(2*sps)) */
    const double rstep = (M_PI/2.0)/(double)sps;
    for (int n = 0; n < BL; n++) { double ph = rstep*(double)n; c0r[n] = sym[n]*cos(ph); c0i[n] = sym[n]*sin(ph); }

    /* c1[k] = c0[k] * (j*phase) = -phase*c0i + j*phase*c0r ; phase=2*(b[i-1]^b[i-2])-1.
     * start magic (k=sps*2, phase=-1), i=2..nbits-1, end magic (i=nbits). */
    if (nbits >= 2) {
        int k = sps*2; double phase = -1.0;
        c1r[k] = -phase*c0i[k]; c1i[k] = phase*c0r[k]; k += sps;
        for (int i = 2; i < nbits; i++) {
            phase = 2.0*(double)(b[i-1]^b[i-2]) - 1.0;
            if (k < BL) { c1r[k] = -phase*c0i[k]; c1i[k] = phase*c0r[k]; }
            k += sps;
        }
        phase = 2.0*(double)(b[nbits-1]^b[nbits-2]) - 1.0;
        if (k < BL) { c1r[k] = -phase*c0i[k]; c1i[k] = phase*c0r[k]; }
    }

    int NS = CALYPSO_BSP_BURSTLEN * sps;     /* 592 */
    for (int n = 0; n < NS; n++) {
        double or_ = 0.0, oi_ = 0.0;
        for (int j = 0; j < 16; j++) { int s = n-15+j; if (s>=0 && s<BL) { or_ += c0r[s]*C0[j]; oi_ += c0i[s]*C0[j]; } }
        for (int j = 0; j < 8;  j++) { int s = n-7+j;  if (s>=0 && s<BL) { or_ += c1r[s]*C1[j]; oi_ += c1i[s]*C1[j]; } }
        double I = or_*AMP, Q = oi_*AMP;
        if (I>32767.0) I=32767.0; else if (I<-32768.0) I=-32768.0;
        if (Q>32767.0) Q=32767.0; else if (Q<-32768.0) Q=-32768.0;
        iq[2*n] = (int16_t)I; iq[2*n+1] = (int16_t)Q;
    }
}

/* Enregistre l'I/Q UL synthetise (cs16 -> fc32 normalise -1..1) dans un .cfile,
 * exactement comme le DL dsp_iq (gr_complex, 4 SPS = 1083333 Hz). Bursts
 * concatenes (pas de framing TDMA) : decodable comme le dsp_iq DL. Lazy-open ;
 * CALYPSO_UL_IQ_RECORD=<chemin> (defaut /dev/shm/dsp_ul_iq.cfile), vide=off.
 * nsamp = nb de samples COMPLEXES (592 = CALYPSO_BSP_BURSTLEN*OSR par burst). */
static void ul_iq_record(const int16_t *iq, int nsamp)
{
    if (!g_ul_rec_init) {
        g_ul_rec_init = 1;
        const char *p = getenv("CALYPSO_UL_IQ_RECORD");
        if (!p) p = "/dev/shm/dsp_ul_iq.cfile";
        if (*p) {
            g_ul_rec = fopen(p, "wb");
            if (g_ul_rec)
                LOGP(DDEV, LOGL_NOTICE, "[ul-rec] record I/Q UL -> %s (cfile fc32 @ 4 SPS)\n", p);
            else
                LOGP(DDEV, LOGL_ERROR, "[ul-rec] fopen(%s): %s\n", p, strerror(errno));
        }
    }
    if (!g_ul_rec || nsamp <= 0) return;
    static float fbuf[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2];
    int nf = nsamp * 2;
    if (nf > (int)(sizeof(fbuf)/sizeof(fbuf[0]))) nf = (int)(sizeof(fbuf)/sizeof(fbuf[0]));
    for (int i = 0; i < nf; i++) fbuf[i] = (float)iq[i] / 32768.0f;
    fwrite(fbuf, sizeof(float), (size_t)nf, g_ul_rec);
}

/* RACH access-burst complet en soft-bits +/-1 pour ul_gmsk_mod :
 * [8 tail][41 sync TS0][36 bits codes gsm0503_rach_ext_encode][3 tail], reste guard.
 * La sync = GSM::gRACHSynchSequenceTS0 (exactement ce que correle osmo-trx). Le DSP
 * Calypso fait normalement ce codage+sync ; shunte, on le refait ici. RA/BSIC env :
 * CALYPSO_UL_RA (defaut 3, fixe pour prouver rc>0), CALYPSO_UL_BSIC (defaut 7 = BSIC
 * reel ; colore la parite -> requis pour CHAN RQD cote osmo-bts, pas pour rc). */
static void ul_build_rach_ra(int8_t *ab, int ra_arg, int bsic_arg)
{
    static const char SYNC[] = "01001011011111111001100110101010001111000";  /* 41 */
    static int RA_env = -1, BSIC_env = -1;
    if (RA_env < 0)   { const char *e = getenv("CALYPSO_UL_RA");   RA_env   = (e && *e) ? (int)strtol(e, 0, 0) : 3; }
    if (BSIC_env < 0) { const char *e = getenv("CALYPSO_UL_BSIC"); BSIC_env = (e && *e) ? atoi(e) : 7; }
    int RA   = (ra_arg   >= 0) ? ra_arg   : RA_env;     /* RA reelle du mobile (paging: >=0x80) */
    int BSIC = (bsic_arg >= 0) ? bsic_arg : BSIC_env;
    ubit_t coded[40]; memset(coded, 0, sizeof(coded));
    gsm0503_rach_ext_encode(coded, (uint16_t)RA, (uint8_t)BSIC, false);   /* 36 bits codes */
    for (int i = 0; i < CALYPSO_BSP_BURSTLEN; i++) ab[i] = -1;            /* tail/guard par defaut */
    int p = 0;
    for (int i = 0; i < 8;  i++) ab[p++] = -1;                            /* extended tail */
    for (int i = 0; i < 41; i++) ab[p++] = (SYNC[i] == '1') ? 1 : -1;     /* synch sequence */
    for (int i = 0; i < 36; i++) ab[p++] = coded[i] ? 1 : -1;            /* RA codee (BSIC color) */
    for (int i = 0; i < 3;  i++) ab[p++] = -1;                            /* tail */
    /* p==88 ; [88..147]=-1 -> ul_gmsk_mod auto-detecte active=88 + guard silence */
}

/* compat : RA/BSIC depuis l'env (CALYPSO_UL_RA / _BSIC). */
static void ul_build_rach(int8_t *ab) { ul_build_rach_ra(ab, -1, -1); }

/* Construit le burst NORMAL #bid (0..3) du bloc SDCCH/SACCH depuis la L2 (23o) :
 * gsm0503_xcch_encode -> 4*116 bits e[] (GSM 05.03 conv+FIRE+interleave). Burst normal
 * = [3 tail][58 e (57 data + steal)][26 TSC7][58 e][3 tail] en soft-bits +/-1. Tout actif
 * (148) -> ul_gmsk_mod fait du GMSK plein (le motif != access-burst -> pas de guard). */
static void ul_build_sdcch_burst(int8_t *ab, const uint8_t *l2, int bid)
{
    static const uint8_t TSC7[26] = {
        1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0 };
    ubit_t e[4 * 116];
    memset(e, 0, sizeof(e));
    gsm0503_xcch_encode(e, l2);
    const ubit_t *cB = e + (bid & 3) * 116;
    int p = 0;
    for (int i = 0; i < 3;  i++) ab[p++] = -1;                  /* tail */
    for (int i = 0; i < 58; i++) ab[p++] = cB[i]      ? 1 : -1; /* data 1 (57 + steal) */
    for (int i = 0; i < 26; i++) ab[p++] = TSC7[i]    ? 1 : -1; /* midamble TSC7 */
    for (int i = 0; i < 58; i++) ab[p++] = cB[58 + i] ? 1 : -1; /* data 2 */
    for (int i = 0; i < 3;  i++) ab[p++] = -1;                  /* tail */
    /* p==148, tout actif -> GMSK plein */
}

/* SELF-TEST (#12) : module l'access-burst (le MÊME que rach_ref.cs16 = dump du vrai
 * modulateBurst osmo-trx) avec ul_mod_laurent et compare. maxdiff~0 => port correct.
 * Le pattern du diff isole le bug : echelle (AMP), decalage (convolution/TOA),
 * conjugue (signe rotation), renverse (sens convol). Appelé 1x. */
static void ul_laurent_selftest(void)
{
    int8_t ab[CALYPSO_BSP_BURSTLEN];
    ul_build_rach(ab);                                   /* 88 bits actifs = ceux de rach_ref */
    static int16_t my[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2];
    ul_mod_laurent(ab, 88, my);
    FILE *f = fopen("/root/rach_ref.cs16", "rb");
    if (!f) { LOGP(DDEV, LOGL_NOTICE, "LAURENT-SELFTEST: pas de /root/rach_ref.cs16\n"); return; }
    static int16_t ref[CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2];
    size_t got = fread(ref, 2 * sizeof(int16_t), CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR, f);
    fclose(f);
    int N = (int)got; if (N > CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR) N = CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR;
    long maxd = 0, sumd = 0;
    for (int i = 0; i < N * 2; i++) { long d = labs((long)my[i] - (long)ref[i]); if (d > maxd) maxd = d; sumd += d; }
    LOGP(DDEV, LOGL_NOTICE,
         "LAURENT-SELFTEST: N=%d maxdiff=%ld avgdiff=%ld | mine[0..7]=%d,%d %d,%d %d,%d %d,%d "
         "ref[0..7]=%d,%d %d,%d %d,%d %d,%d\n", N, maxd, sumd / (N * 2 + 1),
         my[0],my[1],my[2],my[3],my[4],my[5],my[6],my[7],
         ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7]);
}

/* Sideband RACH (NO-HARDCODE) : lit la VRAIE RA+BSIC+FN publiee par QEMU
 * (calypso_trx.c calypso_rach_publish) dans /dev/shm/calypso_rach. Fichier
 * REGULIER (pas un FIFO -> jamais bloquant). Layout 16o fige, partage avec QEMU :
 *   [0..3]=seq(u32 LE)  [4]=ra  [5]=bsic  [8..11]=fn(u32 LE). Retourne 1 si seq>0. */
static int calypso_rach_read(uint8_t *ra, uint8_t *bsic, uint32_t *fn)
{
    static int fd = -1;
    if (fd < 0) fd = open("/dev/shm/calypso_rach", O_RDONLY);   /* retry tant que QEMU ne l'a pas cree */
    if (fd < 0) return 0;
    uint8_t buf[16];
    if (pread(fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf)) return 0;
    uint32_t seq; memcpy(&seq, buf + 0, sizeof(seq));
    if (seq == 0) return 0;
    if (ra)   *ra   = buf[4];
    if (bsic) *bsic = buf[5];
    if (fn)   memcpy(fn, buf + 8, sizeof(*fn));
    return 1;
}

/* Lit /dev/shm/calypso_kc (ecrit par QEMU l1ctl_sock sur L1CTL_CRYPTO_REQ) :
 * [0..3]seq(LE) [4]algo [5]key_len [6..21]Kc. Retourne le seq (0 = pas de cipher
 * actif : aucun CIPHER MODE COMMAND, ou canal reset via DM_EST/DM_REL). */
static uint32_t calypso_kc_read(uint8_t *algo, uint8_t *kc, uint8_t *klen)
{
    static int fd = -1;
    if (fd < 0) fd = open("/dev/shm/calypso_kc", O_RDONLY);
    if (fd < 0) return 0;
    uint8_t buf[32];
    if (pread(fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf)) return 0;
    uint32_t seq; memcpy(&seq, buf, 4);
    if (seq == 0) return 0;
    if (algo) *algo = buf[4];
    if (klen) *klen = buf[5];
    if (kc)   memcpy(kc, buf + 6, 16);
    return seq;
}

/* SDCCH/SACCH UL sideband (#12 PIÈCE 2) : lit la L2 montante (a_cu) publiée par QEMU
 * (calypso_dsp_shunt) dans /dev/shm/calypso_sdcch_ul. Layout 48o : seq@0(u32)
 * l1s_fn@4(u32) fn@8(u32) task_u@12(u16) l1s%51@14(u8) l2[23]@16. Retourne 1 si seq>0. */
static int calypso_sdcch_ul_read(uint8_t *l2, uint8_t *l1s_mod51, uint32_t *l1s_fn, uint32_t *seq_out)
{
    static int fd = -1;
    if (fd < 0) fd = open("/dev/shm/calypso_sdcch_ul", O_RDONLY);
    if (fd < 0) return 0;
    uint8_t buf[48];
    if (pread(fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf)) return 0;
    uint32_t seq; memcpy(&seq, buf + 0, sizeof(seq));
    if (seq == 0) return 0;
    if (seq_out)   *seq_out = seq;
    if (l1s_fn)    memcpy(l1s_fn, buf + 4, sizeof(*l1s_fn));
    if (l1s_mod51) *l1s_mod51 = buf[14];
    if (l2)        memcpy(l2, buf + 16, 23);
    return 1;
}

/* Draine l'UL sur g_bsp_fd (le BSP renvoie l'UL à la source du DL = nous,
 * cf. calypso_bsp.c:381), module le dernier burst dispo. Non-bloquant. */
static void ul_drain(void)
{
    static int _stdone = 0;
    if (!_stdone) { _stdone = 1; ul_laurent_selftest(); }   /* #12 : valide le port Laurent 1x */
    if (g_bsp_fd < 0) return;
    uint8_t pkt[UL_TRXD_HDR + CALYPSO_BSP_BURSTLEN + 16];
    int got = 0, got_rach = 0;   /* got_rach : un VRAI access-burst RACH a ete draine */
    for (;;) {
        ssize_t n = recvfrom(g_bsp_fd, pkt, sizeof(pkt), MSG_DONTWAIT, NULL, NULL);
        if (n < (ssize_t)(UL_TRXD_HDR + CALYPSO_BSP_BURSTLEN)) break;
        const int8_t *bits = (const int8_t *)(pkt + UL_TRXD_HDR);
        /* RACH ENC (defaut ON) : reconstruit l'access-burst code+sync (le DSP shunte
         * ne le fait plus), au lieu de moduler les bits firmware (sans sync). */
        static int rach_enc = -1;
        if (rach_enc < 0) { const char *e = getenv("CALYPSO_UL_RACH_ENC"); rach_enc = (!e || *e != '0'); }

        /* === NO-HARDCODE : TABLE de modulation per-RA ===========================
         * La VRAIE RA du mobile (d_rach@0x0474, plombee via /dev/shm/calypso_rach)
         * varie a chaque burst. osmo-trx a pre-genere /root/rach_ref_RA<nn>.cs16
         * (sa modulation Laurent EXACTE, qui correle son detecteur) pour chaque RA.
         * On selectionne le ref de la VRAIE RA -> le BTS voit la bonne RA, plus le
         * RA=3 fixe. Repli : ancien rach_ref.cs16 (RA fixe), puis GMSK maison. */
        static int16_t ref_tab[16][CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR * 2];
        static int     ref_n[16];          /* 0=pas charge, <0=absent, >0=samples */
        static int     ref_init = 0;
        if (!ref_init) { for (int i = 0; i < 16; i++) ref_n[i] = 0; ref_init = 1; }

        int used_tab = 0;
        if (rach_enc) {
            uint8_t real_ra = 0xff, real_bsic = 0; uint32_t real_fn = 0;
            /* #SMS/paging : encode l'access-burst pour la VRAIE RA a la volee (tout RA
             * 0x00-0xff). La paging response a une RA >= 0x80 (cause "answer to paging") ;
             * l'ancien gate real_ra<16 + table per-RA 0x00-0x0f la ratait -> repli RA=3 ->
             * IMM ASS reqref mismatch -> echec. ul_build_rach_ra + ul_mod_laurent = forme
             * d'onde EXACTE osmo-trx (cf LAURENT-SELFTEST) ; la detection osmo-trx correle
             * la sync (RA-indep) puis decode RA+BSIC -> reqref correcte -> le mobile matche. */
            if (calypso_rach_read(&real_ra, &real_bsic, &real_fn)) {
                g_ul_real_fn = real_fn;            /* stash pour le FN-lock (uhdwrap_read) */
                int8_t ab_rach[CALYPSO_BSP_BURSTLEN];
                ul_build_rach_ra(ab_rach, (int)real_ra, (int)real_bsic);
                ul_mod_laurent(ab_rach, 88, g_ul_iq);   /* 88 bits actifs = access-burst */
                ul_iq_record(g_ul_iq, CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR);  /* record I/Q UL (RACH paging-resp) */
                /* FIX MT-SMS : latch la waveform RACH dans son buffer dedie et arme
                 * STICKY sur N slots RACH-eligibles. Le chemin SDCCH-idle ecrase g_ul_iq
                 * a chaque frame ; g_rach_iq lui reste intact -> la paging-response (un
                 * seul encode) survit jusqu'a un vrai slot RACH. CALYPSO_UL_RACH_STICKY =
                 * nb de slots eligibles a tenter (def 8 ~ couvre >1 multiframe-51). */
                memcpy(g_rach_iq, g_ul_iq, sizeof(g_rach_iq));
                { static int rsticky = -1;
                  if (rsticky < 0) { const char *e = getenv("CALYPSO_UL_RACH_STICKY"); rsticky = (e && *e) ? atoi(e) : 0;        /* defaut OFF (gate vide) : sticky retire */ }
                  g_rach_pending = rsticky; g_rach_arm_seq++; }
                used_tab = 1;
                static int last_ra = -1;
                if ((int)real_ra != last_ra) {
                    last_ra = real_ra;
                    LOGP(DDEV, LOGL_NOTICE,
                         "UL RACH RA REELLE=0x%02x bsic=0x%02x (encode a la volee, fn=%u)\n",
                         real_ra, real_bsic, real_fn);
                }
            }
        }

        /* (DECANNE 2026-06-07 : ancien repli rach_ref.cs16 RA=3 supprime) */
        if (used_tab) {
            /* g_ul_iq rempli par la VRAIE RA encodee a la volee */
        } else {
            /* DECANNE (2026-06-07) : plus de repli RA=3 canne (rach_ref.cs16 / maison).
             * Sans vraie RA publiee dans /dev/shm/calypso_rach, on module les bits BSP
             * bruts (sans sync RACH reconstruite) -> osmo-trx ne les detecte PAS comme un
             * Channel Request. Fin des RACH RA=3 fantomes qui inondaient le BSC en CHAN
             * RQD et saturaient le pool SDCCH. */
            ul_gmsk_mod(bits, g_ul_iq);
            ul_iq_record(g_ul_iq, CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR);  /* record I/Q UL (RACH brute BSP) */
        }
        got = 1;
        if (used_tab) got_rach = 1;   /* seul un VRAI RACH (RA publiee) arme g_ul_pending */
        /* INSTR 2026-06-04 : dump one-shot des 1ers bursts UL recus du BSP pour
         * VOIR si c'est un vrai access-burst (sync RACH 41b) ou autre chose, et
         * confirmer la sortie OSR=4. Couper via CALYPSO_UL_DEBUG=0. */
        static int ul_dbg = -1, ul_seen = 0;
        if (ul_dbg < 0) { const char *e = getenv("CALYPSO_UL_DEBUG"); ul_dbg = (!e || *e!='0'); }
        if (ul_dbg && ul_seen < 8) {
            ul_seen++;
            char bs[CALYPSO_BSP_BURSTLEN + 1];
            int nz = 0;
            for (int i = 0; i < CALYPSO_BSP_BURSTLEN; i++) { bs[i] = bits[i] > 0 ? '1':'0'; if (bits[i]) nz++; }
            bs[CALYPSO_BSP_BURSTLEN] = 0;
            int s0 = g_ul_iq[0], s1 = g_ul_iq[1], smid = g_ul_iq[CALYPSO_BSP_BURSTLEN*CALYPSO_TRX_OSR];
            LOGP(DDEV, LOGL_NOTICE,
                 "UL-DBG #%d in=%db(nz=%d) out=%d samp [%d,%d..mid=%d] bits=%s\n",
                 ul_seen, CALYPSO_BSP_BURSTLEN, nz,
                 CALYPSO_BSP_BURSTLEN*CALYPSO_TRX_OSR, s0, s1, smid, bs);
        }
    }
    /* FIX PHANTOM RACH : g_ul_pending (= déclenche la réinjection de g_rach_iq +
     * la calibration FN) ne doit s'armer que pour un VRAI RACH, pas pour chaque burst
     * SDCCH/idle drainé du BSP. Avant : tout burst -> g_ul_pending=1 -> réinjection du
     * dernier RACH à chaque frame -> osmo-trx corrèle en boucle (200 RACH-DET / 9 vrais)
     * -> CHAN RQD fantômes -> fuite SDCCH -> épuisement -> SMS sans canal. Le vrai RACH
     * du mobile (used_tab=1, RA publiée) arme toujours g_ul_pending -> LU/RACH intacts. */
    if (got_rach) g_ul_pending = 1;
}

/* === RELAIS I/Q CONTINU (mode full-grgsm) ===
 * CALYPSO_IPC_RELAY=1 : au lieu d'extraire un burst TS0 → TRXDv0 → BSP Calypso,
 * on RELAIE l'I/Q CONTINU (fc32) entre osmo-trx et le transceiver gr-gsm du
 * mobile (radio_if_udp). DL : chunk osmo-trx (cs16) → fc32 → UDP RX_PORT.
 * UL : UDP TX_PORT (fc32) → cs16 → ios_rx_from_device → osmo-trx.
 * Plus de DSP Calypso → plus de congestion. */
static int  g_relay_on    = -1;
static int  g_relay_dl_fd = -1;   /* send DL fc32 → radio_if_udp RX (5810) */
static struct sockaddr_in g_relay_dl_dst;
static int  g_relay_ul_fd = -1;   /* recv UL fc32 ← radio_if_udp TX (5811) */
static float g_relay_fbuf[CALYPSO_SHM_BUFSIZE * 2];

static void relay_init(void)
{
    if (g_relay_on >= 0) return;
    const char *e = getenv("CALYPSO_IPC_RELAY");
    g_relay_on = (e && *e == '1') ? 1 : 0;
    if (!g_relay_on) return;
    const char *host = getenv("CALYPSO_TRX_IQ_HOST");
    if (!host || !*host) host = "127.0.0.1";
    const char *rxp = getenv("CALYPSO_TRX_IQ_RX_PORT");
    const char *txp = getenv("CALYPSO_TRX_IQ_TX_PORT");
    int rx_port = (rxp && *rxp) ? atoi(rxp) : 5810;
    int tx_port = (txp && *txp) ? atoi(txp) : 5811;
    g_relay_dl_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&g_relay_dl_dst, 0, sizeof(g_relay_dl_dst));
    g_relay_dl_dst.sin_family = AF_INET;
    g_relay_dl_dst.sin_port   = htons(rx_port);
    g_relay_dl_dst.sin_addr.s_addr = inet_addr(host);
    g_relay_ul_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1; setsockopt(g_relay_ul_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET; a.sin_port = htons(tx_port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(g_relay_ul_fd, (struct sockaddr *)&a, sizeof(a)) < 0)
        LOGP(DDEV, LOGL_ERROR, "RELAY UL bind(:%d) failed\n", tx_port);
    LOGP(DDEV, LOGL_NOTICE,
         "IPC RELAY ON : DL fc32 → %s:%d, UL fc32 ← :%d (full-grgsm)\n",
         host, rx_port, tx_port);
}

/* === FIFO writer FRAME-ATOMIQUE, decouple du hot-path (fix SACCH grgsm) =======
 * BUG corrige (construction IQ "mauvaise a partir de la fifo") : l'ancien
 * write(O_NONBLOCK) direct sur le pipe (a) laissait passer des writes PARTIELS
 * (0<w<fbytes) -> desalignement byte PERMANENT du flux fc32 -> grgsm en garbage ;
 * (b) DROPpait des trames sur EAGAIN -> trous temporels -> grgsm perd la
 * 51-multitrame -> SDCCH/4 SACCH (SI5/SI6) jamais decodee (le BCCH/CCCH resync
 * lui via FCCH/SCH, d'ou "ca marche a moitie").
 * FIX : 1 thread writer DEDIE par FIFO + ring de TRAMES. Le hot-path DL pousse
 * une trame (memcpy sous lock court ~20KB) ou la DROP ENTIERE si le ring est
 * plein ; le writer fait des write() BLOQUANTS COMPLETS (jamais partiels) ->
 * alignement byte toujours correct, jamais d'underrun cote QEMU. */
enum { RELAY_NFIFO_MAX = 8, RELAY_RING = 64,   /* 8 : fft+grgsm+record+asciifft + grgsm_ciph (decipher DL) */
       RELAY_FRAME_FLOATS = CALYPSO_SHM_BUFSIZE * 2 };
typedef struct {
    char            path[128];
    int             fd;                 /* writer-thread-owned */
    pthread_t       th;
    pthread_mutex_t mtx;
    pthread_cond_t  cv;
    float           ring[RELAY_RING][RELAY_FRAME_FLOATS];
    size_t          rlen[RELAY_RING];
    unsigned        head, tail;         /* SPSC : producer=hot-path, consumer=thread */
    unsigned long   dropped, written;
} relay_fifo_t;
static relay_fifo_t g_rfifo[RELAY_NFIFO_MAX];
static int          g_rfifo_n = -1;

static void *relay_fifo_writer(void *arg)
{
    relay_fifo_t *rf = arg;
    static __thread float local[RELAY_FRAME_FLOATS];
    for (;;) {
        size_t nfloats;
        pthread_mutex_lock(&rf->mtx);
        while (rf->head == rf->tail)
            pthread_cond_wait(&rf->cv, &rf->mtx);
        unsigned h = rf->head % RELAY_RING;
        nfloats = rf->rlen[h];
        memcpy(local, rf->ring[h], nfloats * sizeof(float));
        rf->head++;
        pthread_mutex_unlock(&rf->mtx);

        if (rf->fd < 0) {
            /* OUVERTURE NON-BLOQUANTE : pas de lecteur (ENXIO) -> on DROP cette
             * trame et on retentera a la suivante. CRUCIAL : l'ancien open(O_WRONLY)
             * BLOQUANT figeait ce thread tant qu'aucun grgsm n'etait lecteur ; or
             * quand si_bridge TUE grgsm pour le respawn cipher, le lecteur
             * disparait -> ce writer + la cascade relay gelaient l'ipc-device
             * (DL FIFO plein -> osmo-trx SETPOWER no-response -> feed_iq gele ->
             * pas de LU accept). Avec O_NONBLOCK le churn de lecteur (kill/respawn
             * grgsm) est INOFFENSIF. Une fois ouvert, on RETIRE O_NONBLOCK pour
             * garder des write() BLOQUANTS COMPLETS (frame-atomique preserve). */
            int fd = open(rf->path, O_WRONLY | O_NONBLOCK);
            if (fd < 0) { rf->dropped++; continue; }   /* ENXIO : aucun lecteur */
            int fl = fcntl(fd, F_GETFL, 0);
            if (fl >= 0) fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);  /* writes bloquants */
            fcntl(fd, F_SETPIPE_SZ, 1 << 20);
            rf->fd = fd;
        }
        const char *p = (const char *)local;
        size_t left = nfloats * sizeof(float);
        while (left) {                            /* write COMPLET, jamais partiel */
            ssize_t w = write(rf->fd, p, left);
            if (w > 0) { p += (size_t)w; left -= (size_t)w; continue; }
            if (w < 0 && errno == EINTR) continue;
            close(rf->fd); rf->fd = -1; break;    /* EPIPE/EBADF : reader parti */
        }
        if (!left) rf->written++;
    }
    return NULL;
}

static void relay_fifo_init(void)
{
    if (g_rfifo_n >= 0) return;
    /* SIGPIPE ignore : un write() sur une FIFO dont le lecteur (grgsm) vient de
     * disparaitre doit renvoyer EPIPE (gere par la boucle write -> reopen), PAS
     * tuer le process. */
    signal(SIGPIPE, SIG_IGN);
    g_rfifo_n = 0;
    const char *e = getenv("CALYPSO_RELAY_FIFOS");
    const char *list = (e && *e) ? e
        : "/tmp/iq_fft.fifo:/tmp/iq_grgsm.fifo:/tmp/iq_record.fifo";
    char buf[512];
    strncpy(buf, list, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
    char *sav = NULL, *tok = strtok_r(buf, ":", &sav);
    while (tok && g_rfifo_n < RELAY_NFIFO_MAX) {
        relay_fifo_t *rf = &g_rfifo[g_rfifo_n];
        strncpy(rf->path, tok, 127); rf->path[127] = 0;
        rf->fd = -1; rf->head = rf->tail = 0; rf->dropped = rf->written = 0;
        pthread_mutex_init(&rf->mtx, NULL);
        pthread_cond_init(&rf->cv, NULL);
        mkfifo(rf->path, 0666);                  /* ignore EEXIST */
        pthread_create(&rf->th, NULL, relay_fifo_writer, rf);
        g_rfifo_n++;
        tok = strtok_r(NULL, ":", &sav);
    }
    LOGP(DDEV, LOGL_NOTICE,
         "RELAY FIFO: %d writer-threads (frame-atomic, ring=%d trames)\n",
         g_rfifo_n, RELAY_RING);
}

/* hot-path : pousse 1 trame dans chaque FIFO ; DROP entiere si ring plein. */
static void relay_fifo_push(const float *frame, size_t nfloats)
{
    if (g_rfifo_n < 0) relay_fifo_init();
    if (nfloats > (size_t)RELAY_FRAME_FLOATS) nfloats = RELAY_FRAME_FLOATS;
    for (int f = 0; f < g_rfifo_n; f++) {
        relay_fifo_t *rf = &g_rfifo[f];
        pthread_mutex_lock(&rf->mtx);
        if (rf->tail - rf->head >= RELAY_RING) {
            rf->dropped++;                        /* ring plein -> drop TRAME entiere */
            if ((rf->dropped % 500) == 1)
                LOGP(DDEV, LOGL_INFO, "RELAY FIFO %s drop=%lu written=%lu\n",
                     rf->path, rf->dropped, rf->written);
        } else {
            unsigned t = rf->tail % RELAY_RING;
            memcpy(rf->ring[t], frame, nfloats * sizeof(float));
            rf->rlen[t] = nfloats;
            rf->tail++;
            pthread_cond_signal(&rf->cv);
        }
        pthread_mutex_unlock(&rf->mtx);
    }
}

/* ---- RX (uplink_thread loop) : produces UL heartbeat zeros to osmo-trx ---- */

int32_t uhdwrap_read(void *dev, uint32_t num_chans)
{
    struct qemu_dev *d = dev;
    if (!d) return -1;

    static int16_t zeros_iq[CALYPSO_SHM_BUFSIZE * 2];
    static bool zeros_init = false;
    if (!zeros_init) {
        memset(zeros_iq, 0, sizeof(zeros_iq));
        zeros_init = true;
    }

    /* UL (IPC TX) init : pas de bind — l'UL arrive sur g_bsp_fd (le BSP renvoie
     * l'UL à la source du DL). On se contente du flag + drain. */
    if (g_ul_on < 0) {
        const char *e = getenv("CALYPSO_IPC_UL");
        g_ul_on = (e && *e == '1') ? 1 : 0;
        if (g_ul_on)
            LOGP(DDEV, LOGL_NOTICE,
                 "UL (IPC TX) ON : g_bsp_fd → mod GMSK → ios_rx_from_device\n");
    }
    if (g_ul_on) ul_drain();

    /* Chunk UL : zéros par défaut ; si un burst UL est dispo et qu'on est sur
     * un chunk TS0 (ts%1250==0), on l'injecte dans le slot TS0 (samples 0..147). */
    static int16_t ul_chunk[CALYPSO_SHM_BUFSIZE * 2];
    int16_t *ul_src = zeros_iq;
    /* --- reglages UL (sweepables sans rebuild) ---
     * CALYPSO_UL_FN_OFFSET : decalage FN device->osmo-trx (observe = 31).
     * CALYPSO_UL_FN_GATE   : 1 = n'injecter que sur un FN RACH-eligible (combined
     *                        CCCH+SDCCH4 : osmo_fn%51 in {4,5,14..36,45,46}).
     * CALYPSO_UL_SLOT_OFFSET : offset intra-slot (samples) du burst (TOA). */
    static int ul_fnoff = -99999, ul_fngate = -1, ul_slotoff = -1;
    if (ul_fnoff == -99999) { const char *e = getenv("CALYPSO_UL_FN_OFFSET"); ul_fnoff = (e && *e) ? atoi(e) : 36;       /* hardcode : offset FN device->osmo-trx (gate vide=36) */ }
    if (ul_fngate < 0)      { const char *e = getenv("CALYPSO_UL_FN_GATE");   ul_fngate = (!e || *e != '0'); }
    if (ul_slotoff < 0)     { const char *e = getenv("CALYPSO_UL_SLOT_OFFSET"); ul_slotoff = (e && *e) ? atoi(e) : 1875;   /* hardcode : TOA intra-slot RACH (gate vide=1875) */ }
    uint32_t internal_fn = (uint32_t)(d->rx_ts / (uint64_t)CALYPSO_FRAME_SAMPLES);
    uint32_t osmo_fn = internal_fn + (uint32_t)ul_fnoff;     /* FN tel que vu par osmo-trx (SDCCH only) */
    /* FN-GATE RACH (FIX MT-SMS 2026-06-09) : osmo-trx TAMPONNE+CORRELE le burst injecte
     * sur SON fn == internal_fn (PROUVE : la seule RACH-DET du run est a osmo-trx fn=4058,
     * exactement = inject#1 internal_fn=4058, que le gate avait etiquete osmo_fn=4094).
     * L'ancien gate testait osmo_fn%51 = (internal_fn+36)%51 -> il autorisait des slots
     * (internal%51 in {0,9,10,37..44,47..50}) ou osmo-trx fait tourner le correlateur
     * NORMAL-BURST, JAMAIS le correlateur RACH -> burst jamais detecte comme access-burst.
     * Le LU n'a marche QUE par coincidence (inject#1 tombait sur internal%51=29, un vrai
     * slot RACH). La paging-response (RA=0x98) n'a JAMAIS touche un vrai slot RACH -> aucune
     * 2e RACH-DET -> pas de CHAN RQD -> pas d'IMM ASS -> SMS jamais livre.
     * FIX : evaluer l'eligibilite RACH sur internal_fn (== osmo-trx fn), set combination-V
     * {4,5,14..36,45,46} = exactement osmo-trx Transceiver::expectedCorrType() case V.
     * NB : le bloc SDCCH (ligne ~1059) garde son propre osmo_fn+eff_ofs (calibre
     * independamment, SABM/UA OK) -> on NE touche QUE la gate RACH. */
    uint32_t m51 = internal_fn % 51;
    int fn_ok = !ul_fngate || (m51 == 4 || m51 == 5 || (m51 >= 14 && m51 <= 36) || m51 == 45 || m51 == 46);

    /* === FN-LOCK (NO-HARDCODE, env CALYPSO_UL_FN_LOCK=1 ; OFF par defaut) =======
     * Le mobile matche la request-reference de l'IMM ASSIGN sur (ra, T1/T2/T3) =
     * FN mod 42432 (=32*26*51). Il a memorise (real_fn-1) [prim_rach.c:94] ; osmo-trx
     * tamponne le burst injecte avec SA FN (= internal_fn + K_trx). Les 3 horloges
     * sont rate-lockees 1:1 (offset constant verifie ~2016926). On auto-mesure UNE
     * FOIS la congruence cible cal_off au 1er RACH (ZERO FN hardcode), puis on
     * n'injecte que sur le slot ou (internal_fn+cal_off)%42432 == (real_fn-1)%42432.
     * CALYPSO_UL_FN_ADJ = sweep +/- frames (le -1 prim_rach + SB2_LATENCY peut
     * decaler de 1-2). Invisible tant que l'IMM ASSIGN AGCH n'atteint pas le mobile. */
    static int ul_fnlock = -1, fn_adj = -99999;
    if (ul_fnlock < 0)      { const char *e = getenv("CALYPSO_UL_FN_LOCK"); ul_fnlock = (e && *e == '1') ? 1 : 0; }
    if (fn_adj == -99999)   { const char *e = getenv("CALYPSO_UL_FN_ADJ");  fn_adj = e ? atoi(e) : 0; }
    int fnlock_ok = 1;
    if (ul_fnlock) {
        uint32_t real_fn = g_ul_real_fn;
        static int cal_done = 0; static uint32_t cal_off = 0;
        if (!cal_done && real_fn && g_ul_pending) {
            cal_off = ((real_fn - 1u) - internal_fn) % 42432u;   /* live, magic-free */
            cal_done = 1;
            LOGP(DDEV, LOGL_NOTICE, "UL FN-LOCK cal_off=%u (internal_fn=%u real_fn=%u)\n",
                 cal_off, internal_fn, real_fn);
        }
        if (cal_done && real_fn) {
            int64_t w = ((int64_t)real_fn - 1 + fn_adj) % 42432; if (w < 0) w += 42432;
            uint32_t have = (internal_fn + cal_off) % 42432u;
            fnlock_ok = ((uint32_t)w == have);
        } else {
            fnlock_ok = 0;                /* pas encore calibre -> attendre un RACH */
        }
    }
    /* RACH a injecter : soit fraichement livre (g_ul_pending, comportement LU 30x),
     * soit latche STICKY pour la paging-response (g_rach_pending, un seul encode).
     * On prefere g_rach_iq (intact) a g_ul_iq (clobbe par le SDCCH-idle). */
    int rach_inject = (g_ul_pending || g_rach_pending > 0);
    if (g_ul_on && rach_inject && fn_ok && fnlock_ok && (d->rx_ts % ((uint64_t)CALYPSO_FRAME_SAMPLES)) == 0) {
        memset(ul_chunk, 0, sizeof(ul_chunk));
        int off = ul_slotoff < 0 ? 0 : ul_slotoff;
        if (2 * off + (int)sizeof(g_ul_iq) > (int)sizeof(ul_chunk)) off = 0;  /* borne */
        /* g_rach_iq survit au clobber SDCCH-idle -> source preferentielle */
        memcpy(ul_chunk + 2 * off, g_rach_iq, sizeof(g_rach_iq));
        ul_src = ul_chunk;
        g_ul_pending = 0;
        if (g_rach_pending > 0) g_rach_pending--;   /* consomme un slot eligible */
        static unsigned ul_inj = 0;
        if (ul_inj++ < 30 || (ul_inj % 100) == 0)
            LOGP(DDEV, LOGL_NOTICE,
                 "UL inject #%u → internal_fn=%u osmo_fn=%u (%%51=%u) slotoff=%d ts=%llu rach_pend=%d seq=%u\n",
                 ul_inj, internal_fn, osmo_fn, m51, off, (unsigned long long)d->rx_ts,
                 g_rach_pending, g_rach_arm_seq);
    }

    /* === SDCCH/SACCH UL (#12 PIÈCE 2) : burst NORMAL encodé sur le slot dédié =======
     * Le firmware met la L2 montante (SABM/SACCH/idle) dans a_cu -> sideband. On
     * l'encode (gsm0503_xcch + TSC7) et on l'injecte sur le slot SDCCH/4 SS0 UL
     * (osmo_fn%51 ∈ {37..40}, burst bid = osmo_fn%51-37). Priorité sur le relay
     * (ul_src=ul_chunk -> le relay 5811 skip via `ul_src != ul_chunk`). N'écrase PAS
     * le RACH (gate ul_src != ul_chunk). Tunables CALYPSO_UL_SDCCH(=1), _SDCCH_OFS. */
    static int ul_sdcch = -1, sd_ofs = -99999;
    if (ul_sdcch < 0)    { const char *e = getenv("CALYPSO_UL_SDCCH");     ul_sdcch = (!e || *e != '0') ? 1 : 0; }
    if (sd_ofs == -99999){ const char *e = getenv("CALYPSO_UL_SDCCH_OFS"); sd_ofs = e ? atoi(e) : 0; }
    if (ul_sdcch &&
        (d->rx_ts % ((uint64_t)CALYPSO_FRAME_SAMPLES)) == 0) {
        /* POLL le sideband a CHAQUE frame (pas seulement aux slots inject). La SABM
         * (ctrl 0x3f) est publiee a l1s%51={36-39} mais l'offset l1s<->osmo_fn faisait
         * que les reads gates sur les slots SDCCH tombaient sur de l'idle -> SABM
         * jamais vue. Poller chaque frame la capture des qu'elle est publiee -> cache
         * sticky, tenu CALYPSO_UL_SABM_TTL blocs, prefere a l'idle au latch bid 0 ->
         * elle part sur un bloc complet aligne -> osmo-bts Rx SABM -> UA.
         * CALYPSO_UL_SABM_STICKY=0 desactive. */
        static uint8_t pend_l2[23]; static int pend_valid = 0;   /* trame UL en attente (1 bloc) */
        static uint32_t last_seq = 0; static int sticky = -1;
        static uint8_t l1s51 = 0xff;
        static int sd_autoofs = -99999;          /* offset auto-calibre l1s%51 -> osmo s51 */
        if (sticky < 0) { const char *e = getenv("CALYPSO_UL_SABM_STICKY"); sticky = (!e || *e != '0') ? 1 : 0; }
        { uint8_t l2[23]; uint32_t lfn = 0, seq = 0;
          if (calypso_sdcch_ul_read(l2, &l1s51, &lfn, &seq)) {
              /* #2 UL DCCH, CONSUME-ONCE PAR SEQ : chaque transmission firmware = un seq
               * nouveau. On capture la trame une fois par seq nouveau (SAPI0 signalisation
               * OU SAPI3 SMS), injectee sur UN bloc SDCCH puis effacee (cf injection bid 0).
               * Remplace le buffer sticky a TTL partage qui faisait SAPI3 ecraser SAPI0
               * pendant le SMS -> lien principal down. Le firmware multiplexe deja SAPI0/
               * SAPI3 par bloc ; un miss -> retransmission T200 (nouveau seq) -> recapture.
               * Filtre : idle (UI 0x03) et SACCH SAPI1 (sapi=(a0>>2)&7) ecartes.
               * NB : depuis le fix PUBLISH-NO-IDLE cote QEMU, l'idle n'est PLUS publie ->
               * tout seq nouveau est porteur ; le filtre is_fill reste (defensif). */
              int sapi = (l2[0] >> 2) & 0x07;
              int is_fill = (l2[1] == 0x03);
              if (sticky && seq != last_seq && (sapi == 0 || sapi == 3) && !is_fill) {
                  last_seq = seq;
                  memcpy(pend_l2, l2, sizeof(pend_l2)); pend_valid = 1;
                  /* #2 v3 ALIGNEMENT DETERMINISTE (2026-06-09) : l'ancien auto-calib
                   * (sd_autoofs = 37 - osmo_fn%51) derivait l'offset de l'INSTANT ou le
                   * firmware publie la SABM, dicte par l1s.current_time.fn -> seede par
                   * le SCH FN que gr-gsm decode au sync -> RUN-VARIANT. SUPPRIME.
                   * Avec osmo-trx START_FN=0 ET IPCDevice ts_initial snappe a 102*5000,
                   * osmo_trx_fn == internal_fn (mod 102) ; ul_fnoff=36 -> eff_ofs FIXE=15
                   * place bid0..3 sur osmo_trx_fn%51 {37,38,39,40} = SDCCH/4 SS0 UL.
                   * CALYPSO_UL_SDCCH_OFS surcharge pour sweeper si besoin. */
              }
          } }
        int eff_ofs = (sd_ofs != 0) ? sd_ofs : 15;
        uint32_t s51 = (uint32_t)((((long)osmo_fn + eff_ofs) % 51 + 51) % 51);
        if (ul_src != ul_chunk && s51 >= 37 && s51 <= 40) {   /* SDCCH/4 SS0 UL block */
            int bid = (int)s51 - 37;
            /* COHÉRENCE DE BLOC (#2 v2) : osmo-bts desentrelace les 4 bursts osmo%51
             * {37,38,39,40} EN UN bloc L2 -> les 4 DOIVENT porter le MEME L2. On TIENT
             * donc la derniere trame signalisante captee (held_l2) de facon persistante
             * et on la snapshot -> blk_l2 UNIQUEMENT a bid 0, reutilisee pour bid 1..3.
             * Plus de latch mid-bloc (qui rendait le bloc incoherent : bid0=idle +
             * bid1-3=SABM -> CRC fail). held tient jusqu'a remplacement par une nouvelle
             * capture OU expiration (CALYPSO_UL_SABM_HOLD_TTL blocs, def 30 ~7s ; rafraichi
             * a chaque capture). La SABM etant retransmise ~1/s (T200) et injectee sur
             * ~4 blocs/s, osmo-bts voit plusieurs blocs SABM COHERENTS -> decode -> UA.
             * CALYPSO_UL_SABM_HOLD=0 = legacy (pas de hold persistant). */
            static uint8_t held_l2[23]; static int held_valid = 0; static int held_ttl = 0;
            static uint8_t blk_l2[23]; static int blk_valid = 0;
            static int hold_on = -1, hold_ttl_max = -1;
            if (hold_on < 0) { const char *e = getenv("CALYPSO_UL_SABM_HOLD"); hold_on = (!e || *e != '0') ? 1 : 0; }
            if (hold_ttl_max < 0) { const char *e = getenv("CALYPSO_UL_SABM_HOLD_TTL"); hold_ttl_max = (e && *e) ? atoi(e) : 30; }
            /* capture persistante : toute nouvelle trame signalisante remplace held_l2 */
            if (sticky && pend_valid) {
                memcpy(held_l2, pend_l2, sizeof(held_l2)); held_valid = 1;
                held_ttl = hold_on ? hold_ttl_max : 1; pend_valid = 0;
            }
            if (bid == 0) {                         /* snapshot 1×/bloc -> 4 bursts coherents */
                if (held_valid && held_ttl > 0) {
                    memcpy(blk_l2, held_l2, sizeof(blk_l2)); blk_valid = 1;
                    if (--held_ttl == 0) held_valid = 0;
                } else {
                    blk_l2[0] = 0x01; blk_l2[1] = 0x03; blk_l2[2] = 0x01;   /* idle UI SDCCH FIXE SAPI0 */
                    memset(blk_l2 + 3, 0x2b, sizeof(blk_l2) - 3); blk_valid = 1;
                }
            }
            if (blk_valid) {
                int8_t ab[CALYPSO_BSP_BURSTLEN];
                ul_build_sdcch_burst(ab, blk_l2, bid);
                /* === CHIFFREMENT A5 UL ============================================
                 * Si un Kc a ete capture (CIPHER MODE COMMAND -> /dev/shm/calypso_kc),
                 * on chiffre les 114 bits data du burst (mapping IDENTIQUE a osmo-bts
                 * scheduler.c:1614 : negation soft-bit ab[i+3] / ab[i+88]). osmo-bts
                 * dechiffre l'UL avec le FN du burst -> A5 est FN-keye : FN = osmo_fn
                 * (+ CALYPSO_CIPH_FN_ADJ, sweepable car l'alignement FN device<->bts
                 * est la variable critique). CALYPSO_CIPH_A5 force le n (debug). */
                {
                    static int ci_init = -1, ci_fn_adj = 0, ci_force_n = -1;
                    static uint8_t kc[16]; static int n_a5 = 0;
                    if (ci_init < 0) {
                        const char *e = getenv("CALYPSO_CIPH_FN_ADJ"); ci_fn_adj = e ? atoi(e) : 0;
                        const char *f = getenv("CALYPSO_CIPH_A5");     ci_force_n = (f && *f) ? atoi(f) : -1;
                        ci_init = 1;
                    }
                    uint8_t cgalgo = 0, cklen = 0;
                    if (calypso_kc_read(&cgalgo, kc, &cklen))
                        n_a5 = (ci_force_n >= 0) ? ci_force_n
                                                 : ((cgalgo >= 1 && cgalgo <= 3) ? cgalgo : 0);
                    else
                        n_a5 = 0;                       /* seq=0 -> cipher off */
                    if (n_a5 >= 1 && n_a5 <= 3) {
                        ubit_t ks[114];
                        /* FN du keystream = internal_fn (= horloge osmo-trx/osmo-bts,
                         * PROUVE par la RACH-DET a osmo-trx fn==internal_fn), PAS osmo_fn
                         * (=internal_fn+36, label legacy trompeur). osmo-bts dechiffre le
                         * burst UL recu avec SON FN = internal_fn ; on doit chiffrer au
                         * meme FN sinon keystream different (A5 FN-keye) -> CIPHER MODE
                         * COMPLETE illisible -> pas de LU ACCEPT. CALYPSO_CIPH_FN_ADJ=0. */
                        uint32_t fnc = (uint32_t)((long)internal_fn + ci_fn_adj);
                        osmo_a5(n_a5, kc, fnc, NULL, ks);       /* keystream UL */
                        for (int i = 0; i < 57; i++) {
                            if (ks[i])      ab[i + 3]  = (int8_t)-ab[i + 3];
                            if (ks[i + 57]) ab[i + 88] = (int8_t)-ab[i + 88];
                        }
                        static unsigned nci = 0;
                        if (nci++ < 30 || (nci % 100) == 0)
                            LOGP(DDEV, LOGL_NOTICE,
                                 "UL CIPHER A5/%d bid=%d fn=%u (adj=%d) Kc=%02x%02x%02x%02x..\n",
                                 n_a5, bid, fnc, ci_fn_adj, kc[0], kc[1], kc[2], kc[3]);
                    }
                }
                ul_mod_laurent(ab, CALYPSO_BSP_BURSTLEN, g_ul_iq);  /* modulateur EXACT osmo-trx */
                ul_iq_record(g_ul_iq, CALYPSO_BSP_BURSTLEN * CALYPSO_TRX_OSR);  /* record I/Q UL (SDCCH/SACCH) */
                memset(ul_chunk, 0, sizeof(ul_chunk));
                /* offset ÉCHANTILLON dédié au burst NORMAL (≠ access-burst RACH) : le
                 * détecteur normal-burst d'osmo-trx corrèle le TSC à une position
                 * différente du détecteur RACH -> au même offset que le RACH, le TSC
                 * tombe hors fenêtre. CALYPSO_UL_SDCCH_SMP_OFS=N (échantillons, sweepable,
                 * peut être négatif) décale le burst normal pour caler le TSC. */
                static int sd_smp = -999999;
                if (sd_smp == -999999) { const char *e = getenv("CALYPSO_UL_SDCCH_SMP_OFS"); sd_smp = e ? atoi(e) : 0; }
                int off = (ul_slotoff < 0 ? 0 : ul_slotoff) + sd_smp;
                if (off < 0) off = 0;
                if (2 * off + (int)sizeof(g_ul_iq) > (int)sizeof(ul_chunk)) off = 0;
                memcpy(ul_chunk + 2 * off, g_ul_iq, sizeof(g_ul_iq));
                ul_src = ul_chunk;
                static unsigned sd_inj = 0;
                int is_idle_inj = (blk_l2[0] == 0x01 && blk_l2[1] == 0x03 && blk_l2[2] == 0x01);
                if (sd_inj++ < 40 || !is_idle_inj || (sd_inj % 200) == 0)
                    LOGP(DDEV, LOGL_NOTICE,
                         "UL SDCCH inject #%u%s bid=%d osmo%%51=%u l1s%%51=%u eff_ofs=%d L2=%02x %02x %02x\n",
                         sd_inj, is_idle_inj ? "" : " *SABM/SIG*", bid, s51, l1s51,
                         eff_ofs, blk_l2[0], blk_l2[1], blk_l2[2]);
            }
        }
    }

    /* ---- WALL-PACED UL heartbeat (clock_nanosleep ABSTIME) ----
     *
     * Avant : `usleep(2300)` → wall-paced ~2.3ms mais usleep délivre
     * 2.4ms en moyenne sous charge → osmo-trx ts advance ~4% slow →
     * BTS reçoit CLK_IND à 208 FN/sec wall (drift -4.2%).
     *
     * Première tentative : qfn-paced spin-wait (sync sur QEMU FN ticks).
     * Échec : le spin time-out de 10ms quand QEMU lag → osmo-trx-ipc
     * starve → IPC socket disconnect → crash (vérifié dans run +94s).
     *
     * Cette version : clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
     * sur deadline absolue. Précision sub-µs, pas de spin, pas de starve.
     * Même horloge que le clk_master_thread côté QEMU (calypso_trx.c)
     * → les deux pacing restent alignés tant que le host kernel est
     * stable (= toujours, sauf charge extrême). */
    static struct timespec next_deadline = { .tv_sec = 0, .tv_nsec = 0 };
    /* 625 samples / 270833 sps = 2307.692 µs exact = 2307692 ns (= WALL_TDMA_NS/2,
     * le heartbeat est une demi-frame). Configurable via CALYPSO_TDMA_NS (la
     * MÊME var que le clk_master QEMU) pour ralentir la timeline uniformément
     * si l'émulation ne tient pas le temps réel → cohérence osmo-trx ↔ QEMU. */
    static long PERIOD_NS = 0, QFN_LEAD = 0, QFN_FLOOR_NS = 0;
    static int  QFN_FORCE = -1;
    static uint64_t local_half = 0;
    if (PERIOD_NS == 0) {
        PERIOD_NS = CALYPSO_FRAME_NS / 2;   /* demi-frame, budget firmware 4908 qbits */
        const char *e = getenv("CALYPSO_TDMA_NS");
        if (e && *e) { long long v = atoll(e); if (v >= CALYPSO_FRAME_NS) PERIOD_NS = (long)(v / 2); }
        const char *f = getenv("CALYPSO_QFN_FORCE");    QFN_FORCE    = (f && *f == '1') ? 1 : 0;
        const char *l = getenv("CALYPSO_QFN_LEAD");     QFN_LEAD     = (l && *l) ? atol(l) : 32;
        const char *g = getenv("CALYPSO_QFN_FLOOR_NS"); QFN_FLOOR_NS = (g && *g) ? atol(g) : 50000000L;
    }

    /* ---- LOCK SUR L'HORLOGE QEMU (CALYPSO_QFN_FORCE=1) : budget constant ----
     * Le device se cale sur le qfn de qemu (g_qemu_qfn, clk_listener port 6700).
     * osmo-trx (master clock = nos UL) ET le relay->gr-gsm verrouillent sur le
     * firmware. Budget = 148 cplx/frame (DARAM 0x2a00) ; heartbeat = demi-frame
     * -> 2/qfn. local_half <= qfn*2 + QFN_LEAD (sinon attend qemu, poll-sleep).
     * QFN_FLOOR_NS = anti-starve (pas de hard-timeout qui crashait). Defaut
     * (QFN_FORCE=0) = wall historique. */
    if (QFN_FORCE && __atomic_load_n(&g_qfn_seen, __ATOMIC_ACQUIRE)) {
        struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0);
        for (;;) {
            uint32_t qfn = __atomic_load_n(&g_qemu_qfn, __ATOMIC_ACQUIRE);
            if (local_half <= (uint64_t)qfn * 2 + (uint64_t)QFN_LEAD) break;
            struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
            long el = (now.tv_sec - t0.tv_sec) * 1000000000L + (now.tv_nsec - t0.tv_nsec);
            if (el >= QFN_FLOOR_NS) break;
            usleep(100);
        }
        local_half++;
    } else {
        if (next_deadline.tv_sec == 0) {
            clock_gettime(CLOCK_MONOTONIC, &next_deadline);
        }
        next_deadline.tv_nsec += PERIOD_NS;
        while (next_deadline.tv_nsec >= 1000000000L) {
            next_deadline.tv_nsec -= 1000000000L;
            next_deadline.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_deadline, NULL);
    }

    /* RELAIS UL : I/Q fc32 du transceiver gr-gsm → cs16 → osmo-trx. Si rien
     * ce tick, zéros (le clock doit avancer). Buffer local (thread DL séparé). */
    relay_init();
    int16_t relay_ul[CALYPSO_SHM_BUFSIZE * 2];
    /* FIX LU 2026-06-05 : PRIORITE ABSOLUE a la RACH injectee. Si ul_src==ul_chunk
     * (IPC_UL a injecte une RACH ce tick), on SAUTE entierement le bloc relais —
     * sinon recvfrom(5811) (le flowgraph gr-gsm emet sur 5811 en full-grgsm) renvoie
     * n>0 et `ul_src = relay_ul` ECRASAIT la RACH -> enqueue de zeros -> NOPE/-110.
     * (l'ancien garde ne protegeait que n<=0, pas n>0.) */
    if (g_relay_on && g_relay_ul_fd >= 0 && ul_src != ul_chunk) {
        float ulf[CALYPSO_SHM_BUFSIZE * 2];
        ssize_t n = recvfrom(g_relay_ul_fd, ulf, sizeof(ulf), MSG_DONTWAIT, NULL, NULL);
        if (n > 0) {
            /* Le relais (transceiver gr-gsm, 5811) a des donnees -> prioritaire. */
            memset(relay_ul, 0, sizeof(relay_ul));
            int ns = (int)(n / (2 * sizeof(float)));
            if (ns > CALYPSO_SHM_BUFSIZE) ns = CALYPSO_SHM_BUFSIZE;
            for (int i = 0; i < ns * 2; i++) {
                float v = ulf[i] * 32768.0f;
                if (v > 32767.0f) v = 32767.0f; else if (v < -32768.0f) v = -32768.0f;
                relay_ul[i] = (int16_t)v;
            }
            ul_src = relay_ul;
        } else if (ul_src != ul_chunk) {
            /* Pas de RACH injectee par IPC_UL ce tick -> zeros (l'horloge avance). */
            memset(relay_ul, 0, sizeof(relay_ul));
            ul_src = relay_ul;
        }
        /* FIX LU 2026-06-04 : sinon (relais vide MAIS IPC_UL a injecte une RACH)
         * on GARDE ul_chunk. Avant, ce bloc ecrasait INCONDITIONNELLEMENT ul_src
         * par relay_ul (vide en full-grgsm : transceiver 5811 absent) -> la RACH
         * du mobile (BSP -> ul_gmsk_mod -> ul_chunk) etait jetee -> osmo-trx
         * envoyait des NOPE -> BTS jamais de CHAN RQD -> Location Update echouait. */
    }

    /* INSTR : a l'enqueue, quand le burst RACH est dans ul_src (==ul_chunk),
     * prouver qu'il porte de l'energie ET qu'il part vers un stream RX valide. */
    {
        static int eqdbg = -1; if (eqdbg < 0) { const char *e = getenv("CALYPSO_UL_DEBUG"); eqdbg = (!e || *e != '0'); }
        if (eqdbg && ul_src == ul_chunk) {
            static unsigned eqn = 0;
            int nz = 0; for (int i = 0; i < CALYPSO_SHM_BUFSIZE * 2; i++) if (ul_src[i]) nz++;
            if (eqn++ < 40)
                LOGP(DDEV, LOGL_NOTICE,
                     "ENQ-RACH #%u ts=%llu num_chans=%u rx0=%p nz=%d s0=[%d,%d]\n",
                     eqn, (unsigned long long)d->rx_ts, num_chans,
                     (void *)ios_rx_from_device[0], nz, ul_src[0], ul_src[1]);
        }
    }
    for (uint32_t c = 0; c < num_chans && c < 8; c++) {
        if (!ios_rx_from_device[c]) continue;
        int32_t rc = ipc_shm_enqueue(ios_rx_from_device[c],
                                     d->rx_ts,
                                     CALYPSO_SHM_BUFSIZE,
                                     (uint16_t *)ul_src);
        if (rc < 0) {
            static unsigned overruns = 0;
            if (overruns++ < 5)
                LOGP(DDEV, LOGL_NOTICE,
                     "ul_stream enqueue rc=%d chan=%u ts=%llu\n",
                     rc, c, (unsigned long long)d->rx_ts);
        }
    }
    d->rx_ts += CALYPSO_SHM_BUFSIZE;

    return CALYPSO_SHM_BUFSIZE;
}

/* ---- TX (downlink_thread loop) : consumes DL bursts from osmo-trx ----
 * POL = drain silently. Phase 1.5 will sendto() to UDP 127.0.0.1:6702.
 */
/* DL read buffer : osmo-trx commits CHUNK*tx_sps = 625 samples per write at
 * 1 SPS. We read up to that. The first CALYPSO_BSP_BURSTLEN samples = TS=0
 * burst, forwarded to BSP. Rest is discarded for FBSB phase. */
#define DL_READ_SAMPLES       CALYPSO_SHM_BUFSIZE
static uint16_t dl_read_buf[DL_READ_SAMPLES * 2];   /* cs16 I,Q interleaved */
static uint8_t  dl_send_pkt[TRXD_HDR_LEN + CALYPSO_DL_BURSTLEN * 4];

int32_t uhdwrap_write(void *dev, uint32_t num_chans, bool *underrun)
{
    struct qemu_dev *d = dev;
    if (!d || !underrun) return -1;
    *underrun = false;
    bool any = false;

    if (g_bsp_fd < 0) bsp_udp_init();

    for (uint32_t c = 0; c < num_chans && c < 8; c++) {
        if (!ios_tx_to_device[c]) continue;

        uint64_t ts = 0;
        /* timeout_seconds = 0 → wait briefly (cond_timedwait clamps to wall now);
         * we don't want the downlink thread to spin if osmo-trx has no DL ready. */
        int32_t rv = ipc_shm_read(ios_tx_to_device[c], dl_read_buf,
                                  DL_READ_SAMPLES, &ts, 0);
        if (rv <= 0) {
            *underrun = true;
            continue;
        }
        any = true;

        /* RELAIS : I/Q continu (TOUS les samples du chunk, tous TS) → fc32 →
         * UDP vers le transceiver gr-gsm. gsm.receiver trouve lui-même le bon
         * timeslot/timing. On NE fait PAS l'extraction per-burst TRXDv0. */
        relay_init();
        if (g_relay_on) {
            int ns = (rv < DL_READ_SAMPLES) ? rv : DL_READ_SAMPLES;
            for (int i = 0; i < ns * 2; i++)
                g_relay_fbuf[i] = (float)((int16_t)dl_read_buf[i]) / 32768.0f;
            if (g_relay_dl_fd >= 0)
                sendto(g_relay_dl_fd, g_relay_fbuf, (size_t)ns * 2 * sizeof(float),
                       MSG_DONTWAIT, (struct sockaddr *)&g_relay_dl_dst,
                       sizeof(g_relay_dl_dst));
            /* FIFOs LIVE frame-par-frame -> writer-thread FRAME-ATOMIQUE
             * (cf relay_fifo_push / relay_fifo_writer ci-dessus). Plus de write
             * partiel ni de desalignement byte -> grgsm garde la 51-multitrame
             * -> SDCCH/4 SACCH (SI5/SI6) decodee. Drop = TRAME entiere si ring
             * plein (continuite byte preservee). CALYPSO_RELAY_FIFOS (':'-sep).*/
            relay_fifo_push(g_relay_fbuf, (size_t)ns * 2);
            /* RELAY+BSP (#3 cfile) : si CALYPSO_RELAY_ALSO_BSP=1, on NE
             * `continue` PAS — on tombe dans l'extraction TS0→TRXDv0→BSP pour
             * alimenter feed_iq (cfile + shm ring grgsm↔BSP). Defaut: relais pur. */
            static int also_bsp = -1;
            if (also_bsp < 0) { const char *e = getenv("CALYPSO_RELAY_ALSO_BSP");
                                also_bsp = (e && *e=='1') ? 1 : 0; }
            if (!also_bsp) continue;   /* relais pur */
        }

        /* TS=0 slice : SAMPLES_PER_FRAME=1250 at 1 SPS = 8 × 156.25.
         * osmo-trx commits half-frames (625 samples) → chunks pair at
         * ts%1250==0 carry TS0..3, chunks impair (ts%1250==625) carry
         * TS4..7. We only forward TS=0 (first 148 of pair chunks). */
        uint32_t ts_in_frame = (uint32_t)(ts % ((uint64_t)CALYPSO_FRAME_SAMPLES));
        int has_ts0 = (ts_in_frame == 0);
        if (!has_ts0) {
            static uint64_t skip_count = 0;
            if (skip_count < 5 || (skip_count % 5000) == 0) {
                LOGP(DDEV, LOGL_INFO,
                     "skip non-TS0 chunk #%llu ts=%llu ts_in_frame=%u\n",
                     (unsigned long long)skip_count, (unsigned long long)ts,
                     ts_in_frame);
            }
            skip_count++;
            continue;
        }

        /* Offset d'extraction du burst dans le chunk de 625 samples : le burst
         * actif TS0 n'est pas forcément à l'offset 0 du slot (156.25 samples).
         * Le démod gr-gsm a montré un décalage (TSC@62 au lieu de @61) → un
         * mauvais offset désaligne le FCCH/midambule pour le corrélateur FB-det
         * du DSP (d_fb_det reste 0 sur de vrais samples). Réglable via
         * CALYPSO_DL_BURST_OFFSET (samples, défaut 0) pour sweeper l'alignement. */
        static int burst_off = -1;
        static int iq_conj = -1;
        if (burst_off < 0) {
            const char *e = getenv("CALYPSO_DL_BURST_OFFSET");
            burst_off = (e && *e) ? atoi(e) : 0;
            if (burst_off < 0) burst_off = 0;
            const char *c = getenv("CALYPSO_DL_IQ_CONJ");
            iq_conj = (c && *c == '1') ? 1 : 0;
            LOGP(DDEV, LOGL_NOTICE,
                 "DL burst extraction offset = %d samples, iq_conj = %d "
                 "(CALYPSO_DL_BURST_OFFSET / CALYPSO_DL_IQ_CONJ)\n",
                 burst_off, iq_conj);
        }
        int avail = (int)rv - burst_off;
        int n_samples = (avail < CALYPSO_DL_BURSTLEN) ? avail : CALYPSO_DL_BURSTLEN;
        if (n_samples < 0) n_samples = 0;
        const int16_t *burst_src = (const int16_t *)dl_read_buf + 2 * burst_off;
        size_t payload_len = (size_t)n_samples * 4u;
        uint32_t internal_fn = (uint32_t)(ts / ((uint64_t)CALYPSO_FRAME_SAMPLES));

        /* Detect FCCH inline — purely for diag log (helps spot when
         * we serve an FCCH vs other bursts). Not used for routing. */
        bool is_fcch = is_fcch_burst_iq(burst_src, n_samples);

        /* Push TS=0 burst to FIFO tail. clk_listener will pop it and
         * tag with qfn when QEMU is ready. */
        pthread_mutex_lock(&g_dl_fifo_mutex);
        size_t tail = g_dl_fifo_tail;
        size_t depth = tail - g_dl_fifo_head;
        if (depth >= DL_FIFO_SIZE - 1) {
            /* FIFO full — drop oldest by advancing head. Backpressure
             * preferable to OOM. In steady state this shouldn't fire :
             * device reads ~209 burst/s, QEMU consumes ~10 fn/s, but
             * we only read+push when ipc_shm_read returns data, which
             * itself is paced by the consumer. */
            g_dl_fifo_head++;
            static uint64_t drop_count = 0;
            if (drop_count++ < 5)
                LOGP(DDEV, LOGL_NOTICE,
                     "DL FIFO full (size=%d), dropping oldest. #%llu\n",
                     DL_FIFO_SIZE, (unsigned long long)drop_count);
        }
        struct dl_fifo_entry *fe = &g_dl_fifo[tail % DL_FIFO_SIZE];
        fe->is_fcch = is_fcch;
        fe->ts = ts;
        /* Header placeholder (fn rewritten at send time in clk_listener). */
        fe->pkt[0] = 0;
        fe->pkt[1] = 0; fe->pkt[2] = 0; fe->pkt[3] = 0; fe->pkt[4] = 0;
        fe->pkt[5] = 0; fe->pkt[6] = 0; fe->pkt[7] = 0;
        memcpy(fe->pkt + TRXD_HDR_LEN, burst_src, payload_len);
        if (iq_conj) {
            /* Conjugaison I/Q (-Q) : le démod gr-gsm a montré rot=-1 (tone FCCH
             * de signe opposé à la réf du corrélateur DSP). Flip le signe de Q
             * remet le tone à la bonne fréquence pour le FB-det. */
            int16_t *p = (int16_t *)(fe->pkt + TRXD_HDR_LEN);
            for (int k = 0; k < n_samples; k++)
                p[2 * k + 1] = (int16_t)(-p[2 * k + 1]);
        }
        g_dl_fifo_tail = tail + 1;
        size_t new_depth = g_dl_fifo_tail - g_dl_fifo_head;
        pthread_mutex_unlock(&g_dl_fifo_mutex);

        /* ---- α : sweep 51 raw chunks (offset-agnostic FCCH search) ----
         * Capture N=51 consecutive RAW chunks (pre-slice) into per-chunk
         * files indexed by internal_fn = ts / SAMPLES_PER_FRAME. The
         * analyzer (tools/fcch_sweep.py) computes dphi_std per chunk and
         * sorts ascending : FCCH bursts (tone @ +π/2) have std≈0 and float
         * to the top. The internal_fn % 51 of these top hits gives X =
         * on-air ↔ internal frame offset (used by Phase 1.5 slot rewrite).
         *
         * Skip the first SKIP chunks to let osmo-bts-trx exit POWERUP
         * fillers and start real DL. Default CALYPSO_FCCH_DUMP_SKIP=2000
         * ≈ 5 s of TS0 chunks at wall pace.
         *
         * One meta file with the full index for fast lookup. */
        if (getenv("CALYPSO_FCCH_DUMP") && getenv("CALYPSO_FCCH_DUMP")[0] == '1') {
            static int  dump_skipped = 0;
            static int  dump_count = 0;
            static int  dump_done = 0;
            static FILE *idx_file = NULL;
            static int  skip_target = -1;
            static int  capture_target = -1;
            if (skip_target < 0) {
                const char *s = getenv("CALYPSO_FCCH_DUMP_SKIP");
                skip_target = (s && *s) ? atoi(s) : 2000;
                const char *c = getenv("CALYPSO_FCCH_DUMP_N");
                capture_target = (c && *c) ? atoi(c) : 51;
            }
            if (!dump_done) {
                if (dump_skipped < skip_target) {
                    dump_skipped++;
                } else {
                    if (!idx_file) {
                        idx_file = fopen("/tmp/fcch_sweep_index.txt", "w");
                        if (idx_file) {
                            fprintf(idx_file,
                                "# alpha sweep : %d raw chunks (pre-slice, 625 cs16 samples each)\n"
                                "# fields: idx ts internal_fn internal_fn_mod51 ts_in_frame qfn_tagged\n",
                                capture_target);
                        }
                        LOGP(DDEV, LOGL_NOTICE,
                             "alpha sweep START (skipped %d, will capture %d chunks)\n",
                             dump_skipped, capture_target);
                    }
                    uint64_t internal_fn = ts / ((uint64_t)CALYPSO_FRAME_SAMPLES);
                    char path[128];
                    snprintf(path, sizeof(path),
                             "/tmp/fcch_sweep_%03d.bin", dump_count);
                    FILE *f = fopen(path, "wb");
                    if (f) {
                        /* Raw chunk : 2 * rv cs16 samples = up to 1250 uint16 */
                        fwrite(dl_read_buf, sizeof(int16_t), 2 * rv, f);
                        fclose(f);
                    }
                    if (idx_file) {
                        uint32_t qfn_now = __atomic_load_n(&g_qemu_qfn, __ATOMIC_ACQUIRE);
                        fprintf(idx_file, "%03d %llu %llu %llu %u %u\n",
                                dump_count,
                                (unsigned long long)ts,
                                (unsigned long long)internal_fn,
                                (unsigned long long)(internal_fn % 51),
                                ts_in_frame, qfn_now);
                        fflush(idx_file);
                    }
                    dump_count++;
                    if (dump_count >= capture_target) {
                        if (idx_file) { fclose(idx_file); idx_file = NULL; }
                        dump_done = 1;
                        LOGP(DDEV, LOGL_NOTICE,
                             "alpha sweep DONE : %d raw chunks in /tmp/fcch_sweep_*.bin "
                             "+ index /tmp/fcch_sweep_index.txt\n",
                             dump_count);
                    }
                }
            }
        }

        /* Fix D : NO direct sendto here — clk_listener dispatches one
         * burst per qfn tick from the ring above. Direct send would
         * re-introduce the 209/s wall-paced flood that drowned QEMU. */

        static uint64_t dl_count = 0;
        if (dl_count < 5 || (dl_count % 1000) == 0) {
            LOGP(DDEV, LOGL_INFO,
                 "DL-push #%llu chan=%u int_fn=%u%s fifo_depth=%zu rv=%d\n",
                 (unsigned long long)dl_count, c, internal_fn,
                 is_fcch ? " *FCCH*" : "", new_depth, rv);
        }
        dl_count++;
    }

    /* If no DL was ready on any chan, brief sleep to avoid hot-spin. */
    if (!any) usleep(READ_PACE_US);

    return 0;
}
