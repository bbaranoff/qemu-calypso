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
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <osmocom/core/logging.h>

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
#define CALYPSO_SHM_BUFSIZE   625         /* samples per shm commit (matches osmo-trx CHUNK at 1 SPS) */
#define CALYPSO_BSP_BURSTLEN  148         /* samples per UDP datagram to QEMU BSP (= correlator window) */
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
    uint8_t  pkt[TRXD_HDR_LEN + CALYPSO_BSP_BURSTLEN * 4];
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
        /* Patch fn into header in place. */
        e->pkt[0] = 0; /* tn=0 */
        e->pkt[1] = (uint8_t)(fn >> 24);
        e->pkt[2] = (uint8_t)(fn >> 16);
        e->pkt[3] = (uint8_t)(fn >>  8);
        e->pkt[4] = (uint8_t)(fn);
        ssize_t sent = sendto(g_bsp_fd, e->pkt,
                              TRXD_HDR_LEN + CALYPSO_BSP_BURSTLEN * 4, 0,
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
             "calypso-ipc-device (QEMU UDP 6702 bridge), GSM 1 SPS %.0f Hz",
             (double)CALYPSO_FS_NUM / (double)CALYPSO_FS_DEN);

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

    LOGP(DDEV, LOGL_INFO, "qemu_wrap_fill_info_cnf: 1 chan, fs=%.0f Hz, 1 SPS\n",
         (double)CALYPSO_FS_NUM / (double)CALYPSO_FS_DEN);
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
    /* 625 samples / 270833 sps = 2307.692 µs exact = 2307692 ns. */
    static const long PERIOD_NS = 2307692L;

    if (next_deadline.tv_sec == 0) {
        clock_gettime(CLOCK_MONOTONIC, &next_deadline);
    }
    next_deadline.tv_nsec += PERIOD_NS;
    while (next_deadline.tv_nsec >= 1000000000L) {
        next_deadline.tv_nsec -= 1000000000L;
        next_deadline.tv_sec  += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_deadline, NULL);

    for (uint32_t c = 0; c < num_chans && c < 8; c++) {
        if (!ios_rx_from_device[c]) continue;
        int32_t rc = ipc_shm_enqueue(ios_rx_from_device[c],
                                     d->rx_ts,
                                     CALYPSO_SHM_BUFSIZE,
                                     (uint16_t *)zeros_iq);
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
static uint8_t  dl_send_pkt[TRXD_HDR_LEN + CALYPSO_BSP_BURSTLEN * 4];

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

        /* TS=0 slice : SAMPLES_PER_FRAME=1250 at 1 SPS = 8 × 156.25.
         * osmo-trx commits half-frames (625 samples) → chunks pair at
         * ts%1250==0 carry TS0..3, chunks impair (ts%1250==625) carry
         * TS4..7. We only forward TS=0 (first 148 of pair chunks). */
        uint32_t ts_in_frame = (uint32_t)(ts % 1250ULL);
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
        int n_samples = (avail < CALYPSO_BSP_BURSTLEN) ? avail : CALYPSO_BSP_BURSTLEN;
        if (n_samples < 0) n_samples = 0;
        const int16_t *burst_src = (const int16_t *)dl_read_buf + 2 * burst_off;
        size_t payload_len = (size_t)n_samples * 4u;
        uint32_t internal_fn = (uint32_t)(ts / 1250ULL);

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
                    uint64_t internal_fn = ts / 1250ULL;
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
