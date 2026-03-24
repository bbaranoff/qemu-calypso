/*
 * calypso_trx.c — Calypso DSP/TPU/TRX bridge for virtual GSM
 *
 * Fixes applied vs previous version:
 *   1. TPU_FRAME IRQ re-enabled (was commented out → L1 never woke up)
 *   2. Triple fn_synced block removed (dead code from copy-paste)
 *   3. TRXC socket added (port, ASCII commands/responses)
 *      osmo-bts-trx sends POWERON/RXTUNE/TXTUNE/SETSLOT before any burst
 *      Without TRXC ACK the BTS never sends TRXD data
 *   4. trx_send_burst now sends TRXD v0 to osmo-bts-trx (not only GSMTAP)
 *      GSMTAP (port 4729) kept in parallel for Wireshark monitoring
 *   5. TRXC RXTUNE/TXTUNE update sync_arfcn so SCH encodes correct ARFCN
 *   6. TRXD and TRXC sockets properly separated
 *
 * Port layout (configurable via trx_port property, default 6700):
 *   TRXC  = trx_port     (UDP, ASCII command/response)
 *   TRXD  = trx_port + 1 (UDP, binary burst data)
 *   GSMTAP= 4729         (UDP, Wireshark monitoring, fixed)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#include "hw/arm/calypso/calypso_trx.h"

/* =====================================================================
 * GSMTAP (Wireshark monitoring, fixed port 4729)
 * ===================================================================== */

#define GSMTAP_PORT     4729
#define GSMTAP_TYPE_UM  0x01

static int gsmtap_fd = -1;
static struct sockaddr_in gsmtap_addr;

struct __attribute__((packed)) gsmtap_hdr {
    uint8_t  version;
    uint8_t  hdr_len;
    uint8_t  type;
    uint8_t  timeslot;
    uint16_t arfcn;
    int8_t   signal_dbm;
    int8_t   snr_db;
    uint32_t frame_number;
    uint16_t sub_type;
    uint16_t antenna_nr;
    uint32_t res;
};

/* =====================================================================
 * Debug logging
 * ===================================================================== */

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

#define TRX_DEBUG_DSP    0
#define TRX_DEBUG_TPU    0
#define TRX_DEBUG_TSP    0
#define TRX_DEBUG_ULPD   0
#define TRX_DEBUG_TDMA   0
#define TRX_DEBUG_SYNC   1
#define TRX_DEBUG_TRXC   1

/* =====================================================================
 * TRX state
 * ===================================================================== */

typedef struct CalypsoTRX {
    qemu_irq *irqs;

    /* DSP API RAM */
    MemoryRegion dsp_iomem;
    uint16_t     dsp_ram[CALYPSO_DSP_SIZE / 2];
    uint8_t      dsp_page;

    /* TPU */
    MemoryRegion tpu_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[1024];
    bool         tpu_enabled;

    /* TSP */
    MemoryRegion tsp_iomem;
    uint16_t     tsp_regs[CALYPSO_TSP_SIZE / 2];

    /* ULPD */
    MemoryRegion ulpd_iomem;
    uint16_t     ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t     ulpd_counter;

    /* TDMA frame timing */
    QEMUTimer   *tdma_timer;
    uint32_t     fn;
    bool         tdma_running;

    /* DSP task completion timer */
    QEMUTimer   *dsp_timer;

    /* ------------------------------------------------------------------
     * TRXD socket (binary burst data, trx_port + 1)
     * ------------------------------------------------------------------ */
    int          trxd_fd;
    int          trx_port;
    struct sockaddr_in trxd_remote;
    bool         trxd_remote_known;

    /* ------------------------------------------------------------------
     * TRXC socket (ASCII command/response, trx_port)
     * ------------------------------------------------------------------ */
    int          trxc_fd;
    struct sockaddr_in trxc_remote;
    bool         trxc_remote_known;
    bool         powered_on;          /* BTS sent POWERON */

    /* Burst buffers */
    uint8_t      tx_burst[GSM_BURST_BITS];
    uint8_t      rx_burst[GSM_BURST_BITS];
    bool         rx_pending;
    uint8_t      rx_tn;
    int8_t       rx_rssi;
    int16_t      rx_toa;

    /* FN sync (single, clean) */
    bool         fn_synced;
    int32_t      fn_offset;

    /* ARFCN sync state machine */
    SyncState    sync_state;
    uint32_t     sync_fb_countdown;
    uint32_t     sync_sb_countdown;
    uint16_t     sync_arfcn;
    uint8_t      sync_bsic;
    int8_t       sync_rssi;
    uint32_t     sync_ref_fn;
    uint32_t     sync_task_count;
    uint32_t     sync_fb_tasks;
    uint32_t     sync_sb_tasks;
    bool         sync_dsp_booted;
    uint32_t     sync_boot_frame;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* Forward declarations */
static void calypso_dsp_done(void *opaque);
static void calypso_sync_tick(CalypsoTRX *s);

/* =====================================================================
 * TRXC — ASCII command/response interface
 * ===================================================================== */

/*
 * Send a TRXC response back to the BTS.
 * Format: "RSP <CMD> <status>\0"
 * status 0 = success.
 */
static void trxc_send_rsp(CalypsoTRX *s, const char *cmd, int status)
{
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "RSP %s %d", cmd, status);

    if (s->trxc_fd < 0 || !s->trxc_remote_known) {
        return;
    }

    sendto(s->trxc_fd, buf, len + 1, 0,
           (struct sockaddr *)&s->trxc_remote,
           sizeof(s->trxc_remote));

#if TRX_DEBUG_TRXC
    TRX_LOG("TRXC RSP → \"%s\"", buf);
#endif
}

/* Module-level ARFCN default — updated by TRXC RXTUNE */
static uint16_t s_arfcn_default = SYNC_DEFAULT_ARFCN;

/*
 * ARFCN (kHz uplink) → ARFCN number (GSM900 / DCS1800).
 * Best-effort; callers may pass 0 to get the current default.
 */
static uint16_t khz_to_arfcn(uint32_t khz)
{
    if (khz == 0) {
        return s_arfcn_default;
    }
    /* GSM900 uplink: 890.0–915.0 MHz */
    if (khz >= 890000 && khz <= 915000) {
        return (uint16_t)((khz - 890000) / 200 + 1);
    }
    /* DCS1800 uplink: 1710.2–1784.8 MHz */
    if (khz >= 1710200 && khz <= 1784800) {
        return (uint16_t)(((khz - 1710200) / 200) + 512);
    }
    return 1; /* unknown band, fallback */
}

/*
 * Handle one TRXC command from the BTS.
 * cmd points to a null-terminated ASCII string like "CMD POWERON".
 */
static void trxc_handle_cmd(CalypsoTRX *s, const char *cmd)
{
#if TRX_DEBUG_TRXC
    TRX_LOG("TRXC CMD ← \"%s\"", cmd);
#endif

    /* Skip "CMD " prefix */
    if (strncmp(cmd, "CMD ", 4) != 0) {
        return;
    }
    const char *verb = cmd + 4;

    if (strcmp(verb, "POWERON") == 0) {
        s->powered_on = true;
        TRX_LOG("TRXC: POWERON → starting TDMA");
        trxc_send_rsp(s, "POWERON", 0);

    } else if (strcmp(verb, "POWEROFF") == 0) {
        s->powered_on = false;
        trxc_send_rsp(s, "POWEROFF", 0);

    } else if (strncmp(verb, "RXTUNE ", 7) == 0) {
        uint32_t khz = (uint32_t)atoi(verb + 7);
        s->sync_arfcn = khz_to_arfcn(khz);
        s_arfcn_default = s->sync_arfcn;
        TRX_LOG("TRXC: RXTUNE %u kHz → ARFCN %u", khz, s->sync_arfcn);
        trxc_send_rsp(s, "RXTUNE", 0);

    } else if (strncmp(verb, "TXTUNE ", 7) == 0) {
        /* TX tune — we echo the ARFCN update from RX */
        trxc_send_rsp(s, "TXTUNE", 0);

    } else if (strncmp(verb, "SETSLOT ", 8) == 0) {
        trxc_send_rsp(s, "SETSLOT", 0);

    } else if (strncmp(verb, "SETPOWER ", 9) == 0) {
        trxc_send_rsp(s, "SETPOWER", 0);

    } else if (strncmp(verb, "ADJPOWER ", 9) == 0) {
        trxc_send_rsp(s, "ADJPOWER", 0);

    } else if (strncmp(verb, "SETMAXDLY ", 10) == 0) {
        trxc_send_rsp(s, "SETMAXDLY", 0);

    } else if (strncmp(verb, "SETTSC ", 7) == 0) {
        uint8_t tsc = (uint8_t)atoi(verb + 7);
        s->sync_bsic = (s->sync_bsic & 0x38) | (tsc & 0x07);
        trxc_send_rsp(s, "SETTSC", 0);

    } else if (strncmp(verb, "SETBSIC ", 8) == 0) {
        s->sync_bsic = (uint8_t)atoi(verb + 8) & 0x3F;
        trxc_send_rsp(s, "SETBSIC", 0);

    } else {
        /* Unknown command — still ACK with error=1 so BTS doesn't stall */
        char rsp_verb[64];
        sscanf(verb, "%63s", rsp_verb);
        trxc_send_rsp(s, rsp_verb, 1);
        TRX_LOG("TRXC: unknown cmd \"%s\"", verb);
    }
}

/* QEMU fd handler: incoming TRXC datagram */
static void trxc_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    char buf[256];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(s->trxc_fd, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &slen);
    if (n <= 0) {
        return;
    }
    buf[n] = '\0';

    /* Remember BTS TRXC address for responses */
    if (!s->trxc_remote_known) {
        s->trxc_remote = src;
        s->trxc_remote_known = true;
        TRX_LOG("TRXC remote: %s:%d",
                inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }

    trxc_handle_cmd(s, buf);
}

/* =====================================================================
 * TRXD — binary burst data
 * ===================================================================== */

/*
 * Send a burst to Wireshark via GSMTAP (monitoring only).
 */
static void gsmtap_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                               uint8_t *bits, int nbits, bool uplink)
{
    if (gsmtap_fd < 0) {
        return;
    }

    struct {
        struct gsmtap_hdr h;
        uint8_t payload[GSM_BURST_BITS];
    } pkt;

    memset(&pkt, 0, sizeof(pkt));
    pkt.h.version     = 0x02;
    pkt.h.hdr_len     = sizeof(struct gsmtap_hdr) / 4;
    pkt.h.type        = GSMTAP_TYPE_UM;
    pkt.h.timeslot    = tn;
    pkt.h.arfcn       = htons(s->sync_arfcn | (uplink ? 0x8000 : 0));
    pkt.h.signal_dbm  = s->sync_rssi;
    pkt.h.snr_db      = 20;
    pkt.h.frame_number = htonl(fn);

    if (nbits > GSM_BURST_BITS) nbits = GSM_BURST_BITS;
    memcpy(pkt.payload, bits, nbits);

    sendto(gsmtap_fd, &pkt, sizeof(pkt.h) + nbits, 0,
           (struct sockaddr *)&gsmtap_addr, sizeof(gsmtap_addr));
}

/*
 * Send a TRXD v0 RX burst to osmo-bts-trx.
 * Header layout (v0, 8 bytes):
 *   byte 0      : TN
 *   bytes 1..4  : FN (big-endian uint32)
 *   byte 5      : RSSI (unsigned, -dBm)
 *   bytes 6..7  : TOA256 (big-endian int16)
 */
static void trxd_send_rx_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                                int8_t rssi, int16_t toa,
                                uint8_t *bits, int nbits)
{
    if (s->trxd_fd < 0 || !s->trxd_remote_known) {
        return;
    }

    uint8_t buf[TRX_HDR_LEN_RX + GSM_BURST_BITS];
    buf[0] = tn;
    buf[1] = (fn >> 24) & 0xFF;
    buf[2] = (fn >> 16) & 0xFF;
    buf[3] = (fn >>  8) & 0xFF;
    buf[4] = (fn      ) & 0xFF;
    buf[5] = (uint8_t)(-(int)rssi);   /* RSSI as positive value */
    buf[6] = (toa >> 8) & 0xFF;
    buf[7] = (toa     ) & 0xFF;

    if (nbits > GSM_BURST_BITS) nbits = GSM_BURST_BITS;
    memcpy(&buf[TRX_HDR_LEN_RX], bits, nbits);

    sendto(s->trxd_fd, buf, TRX_HDR_LEN_RX + nbits, 0,
           (struct sockaddr *)&s->trxd_remote, sizeof(s->trxd_remote));
}

/*
 * Unified: send a TX (uplink) burst both to GSMTAP and TRXD.
 */
static void trx_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                            uint8_t *bits, int nbits)
{
    /* Wireshark */
    gsmtap_send_burst(s, tn, fn, bits, nbits, true);

    /* osmo-bts-trx via TRXD — emulate an RX burst seen by the virtual BTS */
    trxd_send_rx_burst(s, tn, fn, s->sync_rssi, 0, bits, nbits);
}

/*
 * QEMU fd handler: incoming TRXD datagram from osmo-bts-trx.
 * This carries DL bursts destined for the phone.
 *
 * TRXD TX header (BTS→TRX, v0, 6 bytes):
 *   byte 0      : TN
 *   bytes 1..4  : FN (big-endian uint32)
 *   byte 5      : TX power level
 */
static void trxd_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint8_t buf[512];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(s->trxd_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
    if (n < TRX_HDR_LEN_TX + 1) {
        return;
    }

    if (!s->trxd_remote_known) {
        s->trxd_remote = src;
        s->trxd_remote_known = true;
        TRX_LOG("TRXD remote: %s:%d",
                inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }

    s->rx_tn = buf[0];
    uint32_t trx_fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                      ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];

    /* FN synchronization — do it exactly once */
    if (!s->fn_synced) {
        s->fn_offset = (int32_t)trx_fn - (int32_t)s->fn;
        s->fn_synced = true;
        TRX_LOG("TRXD: FN sync trx=%u local=%u offset=%d",
                trx_fn, s->fn, s->fn_offset);
    }

    /* Realign local FN */
    uint32_t adjusted = (uint32_t)((int32_t)trx_fn - s->fn_offset);
    s->fn = adjusted % GSM_HYPERFRAME;

    int burst_len = (int)(n - TRX_HDR_LEN_TX);
    if (burst_len > GSM_BURST_BITS) burst_len = GSM_BURST_BITS;
    memcpy(s->rx_burst, &buf[TRX_HDR_LEN_TX], burst_len);
    s->rx_rssi  = -70;  /* synthetic */
    s->rx_toa   = 0;
    s->rx_pending = true;

#if TRX_DEBUG_TDMA
    TRX_LOG("TRXD RX TN=%d FN=%u len=%d", s->rx_tn, s->fn, burst_len);
#endif
}

/* =====================================================================
 * Socket initialization
 * ===================================================================== */

static int udp_socket_bind(int port)
{
    struct sockaddr_in addr;
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        return -1;
    }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void trx_sockets_init(CalypsoTRX *s, int base_port)
{
    s->trx_port = base_port;

    /* TRXC: base_port */
    s->trxc_fd = udp_socket_bind(base_port);
    if (s->trxc_fd < 0) {
        TRX_LOG("WARNING: Cannot bind TRXC port %d: %s",
                base_port, strerror(errno));
    } else {
        qemu_set_fd_handler(s->trxc_fd, trxc_receive_cb, NULL, s);
        TRX_LOG("TRXC listening on UDP port %d", base_port);
    }

    /* TRXD: base_port + 1 */
    int trxd_port = base_port + 1;
    s->trxd_fd = udp_socket_bind(trxd_port);
    if (s->trxd_fd < 0) {
        TRX_LOG("WARNING: Cannot bind TRXD port %d: %s",
                trxd_port, strerror(errno));
    } else {
        qemu_set_fd_handler(s->trxd_fd, trxd_receive_cb, NULL, s);
        TRX_LOG("TRXD listening on UDP port %d", trxd_port);
    }

    /* Default remote (localhost, osmo-bts-trx default ports) */
    memset(&s->trxc_remote, 0, sizeof(s->trxc_remote));
    s->trxc_remote.sin_family      = AF_INET;
    s->trxc_remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->trxc_remote.sin_port        = htons(base_port + 100);

    memset(&s->trxd_remote, 0, sizeof(s->trxd_remote));
    s->trxd_remote.sin_family      = AF_INET;
    s->trxd_remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->trxd_remote.sin_port        = htons(trxd_port + 100);
}

static void gsmtap_init(void)
{
    gsmtap_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (gsmtap_fd < 0) {
        return;
    }
    memset(&gsmtap_addr, 0, sizeof(gsmtap_addr));
    gsmtap_addr.sin_family      = AF_INET;
    gsmtap_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    gsmtap_addr.sin_port        = htons(GSMTAP_PORT);
    TRX_LOG("GSMTAP monitoring → UDP port %d", GSMTAP_PORT);
}

/* =====================================================================
 * DSP API RAM
 * ===================================================================== */

static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val;

    if (offset >= CALYPSO_DSP_SIZE) return 0;

    if (size == 2) {
        val = s->dsp_ram[offset / 2];
    } else if (size == 4) {
        val = s->dsp_ram[offset / 2] |
              ((uint32_t)s->dsp_ram[offset / 2 + 1] << 16);
    } else {
        val = ((uint8_t *)s->dsp_ram)[offset];
    }

    if (offset == DSP_DL_STATUS_ADDR && !s->sync_dsp_booted) {
        s->sync_boot_frame++;
        if (s->sync_boot_frame > 3 &&
            s->dsp_ram[DSP_DL_STATUS_ADDR / 2] == DSP_DL_STATUS_BOOT) {
            s->dsp_ram[DSP_DL_STATUS_ADDR / 2] = DSP_DL_STATUS_READY;
            s->dsp_ram[DSP_API_VER_ADDR  / 2]  = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR / 2]  = 0x0000;
            s->sync_dsp_booted = true;
            TRX_LOG("DSP boot: READY, version=0x%04x", DSP_API_VERSION);
            val = DSP_DL_STATUS_READY;
        }
    }

#if TRX_DEBUG_DSP
    TRX_LOG("DSP read  [0x%04x] = 0x%04x (sz=%d)",
            (unsigned)offset, (unsigned)val, size);
#endif
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;

#if TRX_DEBUG_DSP
    TRX_LOG("DSP write [0x%04x] = 0x%04x (sz=%d)",
            (unsigned)offset, (unsigned)value, size);
#endif

    if (size == 2) {
        s->dsp_ram[offset / 2] = (uint16_t)value;
    } else if (size == 4) {
        s->dsp_ram[offset / 2]     = (uint16_t)value;
        s->dsp_ram[offset / 2 + 1] = (uint16_t)(value >> 16);
    } else {
        ((uint8_t *)s->dsp_ram)[offset] = (uint8_t)value;
    }

    if (offset == DSP_API_NDB + NDB_W_D_DSP_PAGE * 2) {
        s->dsp_page = value & 1;
    }
    if (offset == DSP_DL_STATUS_ADDR && value == DSP_DL_STATUS_BOOT) {
        s->sync_boot_frame = 0;
    }
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read  = calypso_dsp_read,
    .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* =====================================================================
 * ARFCN sync state machine
 * ===================================================================== */

static void sync_inject_fb_result(CalypsoTRX *s)
{
    uint16_t *ndb = &s->dsp_ram[DSP_API_NDB / 2];

    ndb[NDB_W_D_FB_DET]   = 1;
    ndb[NDB_W_A_CD_TOA]   = 384;
    ndb[NDB_W_A_CD_PM]    = (uint16_t)((s->sync_rssi + 110) * 64);
    ndb[NDB_W_A_CD_ANGLE] = 500;
    ndb[NDB_W_A_CD_SNR]   = 2048;

#if TRX_DEBUG_SYNC
    TRX_LOG("SYNC: FB detected TOA=%d PM=%d FN=%u",
            ndb[NDB_W_A_CD_TOA], ndb[NDB_W_A_CD_PM], s->fn);
#endif
}

static void sync_inject_sb_result(CalypsoTRX *s)
{
    uint16_t *ndb = &s->dsp_ram[DSP_API_NDB / 2];

    s->sync_ref_fn = s->fn;

    uint16_t sch26[5];
    sch_encode(sch26, s->sync_bsic, s->fn);
    for (int i = 0; i < NDB_W_A_SCH26_LEN; i++) {
        ndb[NDB_W_A_SCH26 + i] = sch26[i];
    }

    ndb[NDB_W_A_CD_TOA]   = 27;
    ndb[NDB_W_A_CD_PM]    = (uint16_t)((s->sync_rssi + 110) * 64);
    ndb[NDB_W_A_CD_ANGLE] = 431;
    ndb[NDB_W_A_CD_SNR]   = 4096;

    uint32_t t1 = s->fn / (26 * 51);
    uint32_t t2 = s->fn % 26;
    uint32_t t3 = s->fn % 51;

#if TRX_DEBUG_SYNC
    TRX_LOG("SYNC: SCH decoded BSIC=%d(NCC=%d,BCC=%d) FN=%u T1=%u T2=%u T3=%u",
            s->sync_bsic,
            (s->sync_bsic >> 3) & 7, s->sync_bsic & 7,
            s->fn, t1, t2, t3);
    TRX_LOG("SYNC: a_sch26=[%04x %04x %04x %04x %04x]",
            sch26[0], sch26[1], sch26[2], sch26[3], sch26[4]);
#endif
}

static void calypso_sync_tick(CalypsoTRX *s)
{
    uint16_t *ndb = &s->dsp_ram[DSP_API_NDB / 2];

    switch (s->sync_state) {
    case SYNC_IDLE:
        break;

    case SYNC_FCCH_SEARCH:
        if (s->sync_fb_countdown > 0) {
            s->sync_fb_countdown--;
        }
        if (s->sync_fb_countdown == 0) {
            sync_inject_fb_result(s);
            s->sync_state = SYNC_FCCH_FOUND;
            TRX_LOG("SYNC: → FCCH_FOUND FN=%u", s->fn);
        }
        break;

    case SYNC_FCCH_FOUND:
        break;

    case SYNC_SCH_SEARCH:
        if (s->sync_sb_countdown > 0) {
            s->sync_sb_countdown--;
        }
        if (s->sync_sb_countdown == 0) {
            sync_inject_sb_result(s);
            s->sync_state = SYNC_LOCKED;
            TRX_LOG("SYNC: ★ TDMA LOCKED ★ ARFCN=%d BSIC=%d FN=%u",
                    s->sync_arfcn, s->sync_bsic, s->fn);
        }
        break;

    case SYNC_LOCKED:
        ndb[NDB_W_D_FN] = (uint16_t)(s->fn & 0xFFFF);
        break;
    }
}

/* =====================================================================
 * DSP task type detection
 * ===================================================================== */

typedef enum {
    TASK_NONE = 0,
    TASK_FB,
    TASK_SB,
    TASK_NB,
    TASK_RACH,
    TASK_OTHER,
} TaskType;

static TaskType detect_task_type(uint16_t task_d)
{
    if (task_d == 0) return TASK_NONE;

    uint8_t lo_nib = task_d & 0x0F;
    uint8_t lo3    = task_d & 0x07;

    if (lo_nib == 4 || lo_nib == 12)  return TASK_FB;
    if (lo_nib == 5 || lo_nib == 13)  return TASK_SB;
    if (lo_nib == 10 || lo_nib == 14) return TASK_NB;
    if (lo_nib == 8)                  return TASK_RACH;
    if (lo3 == 1 && task_d < 0x20)    return TASK_FB;
    if (lo3 == 2 && task_d < 0x20)    return TASK_SB;
    if (task_d == 0x000D || task_d == 0x000E) return TASK_FB;
    if (task_d == 0x001C)             return TASK_SB;

    return TASK_OTHER;
}

/* =====================================================================
 * DSP task processing
 * ===================================================================== */

static void calypso_dsp_process(CalypsoTRX *s)
{
    uint16_t *w_page, *r_page, *ndb;

    if (s->dsp_page == 0) {
        w_page = &s->dsp_ram[DSP_API_W_PAGE0 / 2];
        r_page = &s->dsp_ram[DSP_API_R_PAGE0 / 2];
    } else {
        w_page = &s->dsp_ram[DSP_API_W_PAGE1 / 2];
        r_page = &s->dsp_ram[DSP_API_R_PAGE1 / 2];
    }
    ndb = &s->dsp_ram[DSP_API_NDB / 2];

    uint16_t task_d = w_page[DB_W_D_TASK_D];
    uint16_t task_u = w_page[DB_W_D_TASK_U];

    if (task_d != 0 || task_u != 0) {
        s->sync_task_count++;
    }

    TaskType ttype = detect_task_type(task_d);

    switch (ttype) {
    case TASK_FB:
        s->sync_fb_tasks++;
#if TRX_DEBUG_SYNC
        TRX_LOG("SYNC: FB task d=0x%04x cnt=%u state=%d FN=%u",
                task_d, s->sync_fb_tasks, s->sync_state, s->fn);
#endif
        if (s->sync_state == SYNC_IDLE) {
            s->sync_fb_countdown = SYNC_FB_DETECT_DELAY;
            s->sync_state = SYNC_FCCH_SEARCH;
            TRX_LOG("SYNC: → FCCH_SEARCH (detect in %d frames)",
                    SYNC_FB_DETECT_DELAY);
        }
        break;

    case TASK_SB:
        s->sync_sb_tasks++;
#if TRX_DEBUG_SYNC
        TRX_LOG("SYNC: SB task d=0x%04x cnt=%u state=%d FN=%u",
                task_d, s->sync_sb_tasks, s->sync_state, s->fn);
#endif
        if (s->sync_state == SYNC_FCCH_FOUND) {
            s->sync_sb_countdown = SYNC_SB_DECODE_DELAY;
            s->sync_state = SYNC_SCH_SEARCH;
            TRX_LOG("SYNC: → SCH_SEARCH (decode in %d frames)",
                    SYNC_SB_DECODE_DELAY);
        }
        break;

    case TASK_NB:
        if (s->sync_state == SYNC_LOCKED) {
            if (s->rx_pending) {
                uint16_t *burst_r = &r_page[0x19];
                for (int i = 0; i < GSM_BURST_BITS; i++) {
                    burst_r[i] = s->rx_burst[i];
                }
                r_page[0] = 1;
                r_page[1] = 0;
                s->rx_pending = false;
            } else {
                uint16_t *burst_r = &r_page[0x19];
                for (int i = 0; i < GSM_BURST_BITS; i++) {
                    burst_r[i] = 128;
                }
                r_page[0] = 0;
                r_page[1] = 0;
            }
        }
        break;

    default:
        break;
    }

    /* TX (uplink) burst */
    if (task_u != 0 && s->sync_state == SYNC_LOCKED) {
        uint16_t *burst_w = &w_page[0x19];
        uint8_t bits[GSM_BURST_BITS];
        for (int i = 0; i < GSM_BURST_BITS; i++) {
            bits[i] = burst_w[i < 78 ? i : 78 + (i - 78)] & 1;
        }
        trx_send_burst(s, 0, s->fn, bits, GSM_BURST_BITS);
    }

    w_page[DB_W_D_TASK_D] = 0;
    w_page[DB_W_D_TASK_U] = 0;
    ndb[NDB_W_D_FN] = (uint16_t)(s->fn & 0xFFFF);
}

static void calypso_dsp_done(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_API]);
}

/* =====================================================================
 * TPU
 * ===================================================================== */

static uint64_t calypso_tpu_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    switch (offset) {
    case TPU_CTRL:     val = s->tpu_regs[TPU_CTRL / 2]; break;
    case TPU_IDLE:     val = 1; break;
    case TPU_INT_CTRL: val = s->tpu_regs[TPU_INT_CTRL / 2]; break;
    case TPU_INT_STAT: val = 0; break;
    case TPU_DSP_PAGE: val = s->dsp_page; break;
    case TPU_FRAME:    val = (uint16_t)(s->fn % GSM_HYPERFRAME); break;
    case TPU_OFFSET:   val = s->tpu_regs[TPU_OFFSET / 2]; break;
    case TPU_SYNCHRO:  val = s->tpu_regs[TPU_SYNCHRO / 2]; break;
    default:
        if (offset >= TPU_RAM_BASE &&
            offset < TPU_RAM_BASE + sizeof(s->tpu_ram)) {
            val = s->tpu_ram[(offset - TPU_RAM_BASE) / 2];
        } else if (offset / 2 < CALYPSO_TPU_SIZE / 2) {
            val = s->tpu_regs[offset / 2];
        }
        break;
    }
#if TRX_DEBUG_TPU
    TRX_LOG("TPU read  [0x%04x] = 0x%04x", (unsigned)offset, (unsigned)val);
#endif
    return val;
}

static void calypso_tpu_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

#if TRX_DEBUG_TPU
    TRX_LOG("TPU write [0x%04x] = 0x%04x", (unsigned)offset, (unsigned)value);
#endif

    if (offset / 2 < CALYPSO_TPU_SIZE / 2) {
        s->tpu_regs[offset / 2] = (uint16_t)value;
    }
    if (offset >= TPU_RAM_BASE &&
        offset < TPU_RAM_BASE + sizeof(s->tpu_ram)) {
        s->tpu_ram[(offset - TPU_RAM_BASE) / 2] = (uint16_t)value;
        return;
    }

    switch (offset) {
    case TPU_CTRL:
        if ((value & TPU_CTRL_ENABLE) && !s->tpu_enabled) {
            s->tpu_enabled = true;
            calypso_dsp_process(s);
            timer_mod_ns(s->dsp_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10000);
        }
        if (value & TPU_CTRL_RESET) {
            s->tpu_enabled = false;
            s->fn_synced   = false;
            s->fn_offset   = 0;
            TRX_LOG("TPU reset → FN sync reset");
        }
        break;
    case TPU_DSP_PAGE:
        s->dsp_page = value & 1;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps calypso_tpu_ops = {
    .read  = calypso_tpu_read,
    .write = calypso_tpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* =====================================================================
 * TSP
 * ===================================================================== */

static uint64_t calypso_tsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset == TSP_RX_REG) return 0xFFFF;
    return (offset / 2 < CALYPSO_TSP_SIZE / 2) ? s->tsp_regs[offset / 2] : 0;
}

static void calypso_tsp_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset / 2 < CALYPSO_TSP_SIZE / 2) {
        s->tsp_regs[offset / 2] = (uint16_t)value;
    }
}

static const MemoryRegionOps calypso_tsp_ops = {
    .read  = calypso_tsp_read,
    .write = calypso_tsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* =====================================================================
 * ULPD
 * ===================================================================== */

static uint64_t calypso_ulpd_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    if (offset >= 0x20 && offset <= 0x40) return 0; /* fake ABB ACK */

    switch (offset) {
    case ULPD_SETUP_CLK13:  val = 0x2003; break;
    case ULPD_COUNTER_HI:
        s->ulpd_counter += 100;
        val = (s->ulpd_counter >> 16) & 0xFFFF;
        break;
    case ULPD_COUNTER_LO:   val = s->ulpd_counter & 0xFFFF; break;
    case ULPD_GAUGING_CTRL: val = 0x0001; break;
    case ULPD_GSM_TIMER:    val = (uint16_t)(s->fn & 0xFFFF); break;
    default:
        if (offset / 2 < CALYPSO_ULPD_SIZE / 2) {
            val = s->ulpd_regs[offset / 2];
        }
        break;
    }
    return val;
}

static void calypso_ulpd_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset >= 0x20 && offset <= 0x40) return; /* fake ABB ACK */
    if (offset / 2 < CALYPSO_ULPD_SIZE / 2) {
        s->ulpd_regs[offset / 2] = (uint16_t)value;
    }
}

static const MemoryRegionOps calypso_ulpd_ops = {
    .read  = calypso_ulpd_read,
    .write = calypso_ulpd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl  = { .min_access_size = 1, .max_access_size = 2 },
};

/* =====================================================================
 * TDMA frame timer — drives L1 at 4.615 ms per frame
 * ===================================================================== */

static void calypso_tdma_tick(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    s->fn = (s->fn + 1) % GSM_HYPERFRAME;

    s->tpu_enabled = false;
    s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_ENABLE;

    calypso_sync_tick(s);

#if TRX_DEBUG_TDMA
    if ((s->fn % 5000) == 0) {
        TRX_LOG("TDMA FN=%u sync=%d tasks=%u fb=%u sb=%u powered=%d",
                s->fn, s->sync_state, s->sync_task_count,
                s->sync_fb_tasks, s->sync_sb_tasks, s->powered_on);
    }
#endif

    /* FIX #1: IRQ was commented out — re-enabled.
     * Without this pulse, the L1 firmware scheduler never wakes up
     * and the TDMA machinery is dead even though the timer fires. */
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

    if (s->tdma_running) {
        timer_mod_ns(s->tdma_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
    }
}

static void calypso_tdma_start(CalypsoTRX *s)
{
    if (s->tdma_running) return;
    s->tdma_running = true;
    s->fn_synced    = false;
    s->fn_offset    = 0;
    s->fn           = 0;
    TRX_LOG("TDMA started (4.615ms frame timer)");
    timer_mod_ns(s->tdma_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
}

/* =====================================================================
 * Init
 * ===================================================================== */

static void calypso_dsp_api_init(CalypsoTRX *s)
{
    memset(s->dsp_ram, 0, sizeof(s->dsp_ram));
    s->dsp_ram[DSP_DL_STATUS_ADDR / 2] = DSP_DL_STATUS_READY;
    s->sync_dsp_booted = true;
}

static void calypso_sync_init(CalypsoTRX *s)
{
    s->sync_state        = SYNC_IDLE;
    s->sync_fb_countdown = 0;
    s->sync_sb_countdown = 0;
    s->sync_arfcn        = SYNC_DEFAULT_ARFCN;
    s->sync_bsic         = SYNC_DEFAULT_BSIC;
    s->sync_rssi         = SYNC_DEFAULT_RSSI;
    s->sync_ref_fn       = 0;
    s->sync_task_count   = 0;
    s->sync_fb_tasks     = 0;
    s->sync_sb_tasks     = 0;
    s->sync_dsp_booted   = false;
    s->sync_boot_frame   = 0;

    TRX_LOG("Sync init: ARFCN=%d BSIC=0x%02x(NCC=%d,BCC=%d) RSSI=%d dBm",
            s->sync_arfcn, s->sync_bsic,
            (s->sync_bsic >> 3) & 7, s->sync_bsic & 7,
            s->sync_rssi);
}

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs, int trx_port)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx    = s;
    s->irqs  = irqs;
    s->trxd_fd = -1;
    s->trxc_fd = -1;

    TRX_LOG("=== Calypso TRX bridge init ===");
    TRX_LOG("  TRXC port : %d (ASCII commands)", trx_port);
    TRX_LOG("  TRXD port : %d (binary bursts)",  trx_port + 1);
    TRX_LOG("  GSMTAP    : %d (Wireshark)",       GSMTAP_PORT);

    gsmtap_init();

    /* DSP API RAM */
    memory_region_init_io(&s->dsp_iomem, NULL, &calypso_dsp_ops, s,
                          "calypso.dsp_api", CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_DSP_BASE, &s->dsp_iomem);
    calypso_dsp_api_init(s);

    calypso_sync_init(s);

    /* TPU */
    memory_region_init_io(&s->tpu_iomem, NULL, &calypso_tpu_ops, s,
                          "calypso.tpu", CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TPU_BASE, &s->tpu_iomem);

    /* TSP */
    memory_region_init_io(&s->tsp_iomem, NULL, &calypso_tsp_ops, s,
                          "calypso.tsp", CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TSP_BASE, &s->tsp_iomem);

    /* ULPD */
    memory_region_init_io(&s->ulpd_iomem, NULL, &calypso_ulpd_ops, s,
                          "calypso.ulpd", CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_ULPD_BASE, &s->ulpd_iomem);

    /* Timers */
    s->tdma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_tdma_tick, s);
    s->dsp_timer  = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_dsp_done, s);

    /* TRX sockets */
    if (trx_port > 0) {
        trx_sockets_init(s, trx_port);
    } else {
        TRX_LOG("TRX sockets disabled");
    }

    calypso_tdma_start(s);
    TRX_LOG("=== TRX bridge ready ===");
}
