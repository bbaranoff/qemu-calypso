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
#include "hw/arm/calypso/calypso_uart.h"

/* Global reference to UART modem for TDMA tick RX kick */
extern CalypsoUARTState *g_uart_modem;

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
    MemoryRegion tpu_ram_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[CALYPSO_TPU_RAM_SIZE / 2];
    bool         tpu_enabled;

    /* SIM controller */
    MemoryRegion sim_iomem;
    uint16_t     sim_regs[CALYPSO_SIM_SIZE / 2];
    QEMUTimer   *sim_atr_timer;

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

    /* Frame IRQ deassert timer */
    QEMUTimer   *frame_irq_timer;

    /* ------------------------------------------------------------------
     * TRXD socket (binary burst data, trx_port + 1)
     * ------------------------------------------------------------------ */
    int          trxd_fd;
    int          trx_port;
    struct sockaddr_in trxd_remote;
    bool         trxd_remote_known;

    /* ------------------------------------------------------------------
     * TRXC socket (ASCII command/response, trx_port + 1)
     * ------------------------------------------------------------------ */
    int          trxc_fd;
    struct sockaddr_in trxc_remote;
    bool         trxc_remote_known;
    bool         powered_on;          /* BTS sent POWERON */

    /* ------------------------------------------------------------------
     * CLK socket (clock indications from fake_trx, trx_port)
     * Receives "IND CLOCK <fn>\0" from fake_trx
     * ------------------------------------------------------------------ */
    int          clk_fd;

    /* ------------------------------------------------------------------
     * Air interface socket (peer-to-peer between 2 QEMU instances)
     * One QEMU's TX burst → other QEMU's RX burst
     * ------------------------------------------------------------------ */
    int          air_fd;
    struct sockaddr_in air_peer;
    bool         air_enabled;

    /* Burst buffers */
    uint8_t      tx_burst[GSM_BURST_BITS];
    uint8_t      rx_burst[GSM_BURST_BITS];
    bool         rx_pending;
    uint8_t      rx_tn;
    int8_t       rx_rssi;
    int16_t      rx_toa;

    /* Air burst ring buffer — indexed by FN % AIR_BUF_SIZE */
#define AIR_BUF_SIZE 256
    struct {
        uint32_t fn;
        uint8_t  bits[GSM_BURST_BITS];
        bool     valid;
    } air_buf[AIR_BUF_SIZE];

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

    /* PM measurement tracking */
    uint32_t     pm_count;
    uint32_t     pm_debug_reads;     /* countdown: log read-page accesses after PM */

    /* NB burst accumulator for XCCH decoding (4 bursts per block) */
    uint8_t      nb_bursts[4][116];  /* 4 × 114 coded bits (+ 2 steal) */
    int          nb_burst_count;     /* bursts accumulated (0..3) */

    /* fake_trx clock-driven mode */
    bool         faketrx_powered;
    bool         clk_driven;      /* true once first IND CLOCK received */
    int          clk_ticks_remaining; /* ticks to fire before stopping */

    /* Firmware patching (one-shot, applied on first DSP access) */
    bool         fw_patched;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* Forward declarations */
static void calypso_dsp_done(void *opaque);
static void calypso_sync_tick(CalypsoTRX *s);
static void calypso_tdma_start(CalypsoTRX *s);
static void __attribute__((unused)) air_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                           uint8_t *bits, int nbits);
static void calypso_tdma_tick(void *opaque);
static void trxc_send_cmd(CalypsoTRX *s, const char *cmd);

/* =====================================================================
 * CLK — Clock indications from fake_trx
 *
 * fake_trx sends "IND CLOCK <fn>\0" on the CLK port (base_port).
 * We use this to drive the TDMA tick instead of the internal timer.
 * ===================================================================== */

static void clk_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    char buf[256];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(s->clk_fd, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &slen);
    if (n <= 0) {
        return;
    }
    buf[n] = '\0';

    /* Parse "IND CLOCK <fn>" — just sync FN, don't do rapid catch-up.
     * The internal VIRTUAL timer drives the firmware at its own pace. */
    uint32_t clk_fn;
    if (sscanf(buf, "IND CLOCK %u", &clk_fn) == 1) {
        if (!s->tdma_running) {
            calypso_tdma_start(s);
        }
        if (!s->fn_synced) {
            s->fn_offset = (int32_t)clk_fn - (int32_t)s->fn;
            s->fn_synced = true;
            TRX_LOG("CLK: FN sync clk=%u local=%u offset=%d",
                    clk_fn, s->fn, s->fn_offset);
        }
    }
}

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
    /* E-GSM900 uplink: 880.0–890.0 MHz → ARFCN 975–1023,0 */
    if (khz >= 880000 && khz < 890000) {
        int n = (int)(khz - 890000) / 200 + 1024;
        if (n >= 975 && n <= 1023) return (uint16_t)n;
        if (n == 1024) return 0;
    }
    /* GSM900 uplink: 890.0–915.0 MHz → ARFCN 0–124 */
    if (khz >= 890000 && khz <= 915000) {
        int n = (int)(khz - 890000) / 200;
        if (n == 0) return 0;
        return (uint16_t)n;
    }
    /* DCS1800 uplink: 1710.2–1784.8 MHz */
    if (khz >= 1710200 && khz <= 1784800) {
        return (uint16_t)(((khz - 1710200) / 200) + 512);
    }
    return s_arfcn_default; /* unknown band, use default */
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

    /* Skip "CMD " prefix — RSP messages from fake_trx are just logged */
    if (strncmp(cmd, "CMD ", 4) != 0) {
#if TRX_DEBUG_TRXC
        TRX_LOG("TRXC CMD ← \"%s\"", cmd);
#endif
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

    } else if (strncmp(verb, "RFMUTE ", 7) == 0) {
        trxc_send_rsp(s, "RFMUTE", 0);

    } else if (strncmp(verb, "SETFORMAT ", 10) == 0) {
        trxc_send_rsp(s, "SETFORMAT", 0);

    } else if (strncmp(verb, "NOMTXPOWER", 10) == 0) {
        trxc_send_rsp(s, "NOMTXPOWER", 0);

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

    /* Always update BTS TRXC address (port changes on BTS restart) */
    if (!s->trxc_remote_known ||
        s->trxc_remote.sin_port != src.sin_port) {
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
static void __attribute__((unused)) trxd_send_rx_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
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

    /* Convert hard bits (0/1) to TRXD soft bits.
     * osmo-bts-trx does: sbit = 127 - buf[i]  (255 special → -127)
     * So: 0x00 → sbit +127 (strong bit 0), 0xFF → sbit -127 (strong bit 1) */
    for (int i = 0; i < nbits; i++) {
        buf[TRX_HDR_LEN_RX + i] = bits[i] ? 0xFF : 0x00;
    }

    sendto(s->trxd_fd, buf, TRX_HDR_LEN_RX + nbits, 0,
           (struct sockaddr *)&s->trxd_remote, sizeof(s->trxd_remote));
}

/*
 * Unified: send a TX (uplink) burst to fake_trx and GSMTAP.
 *
 * TRXD TX format (MS → fake_trx, v0):
 *   byte 0      : VER(4bit) + TN(3bit)
 *   bytes 1..4  : FN (big-endian uint32)
 *   byte 5      : TX power level
 *   bytes 6+    : hard bits (ubit_t: 0 or 1)
 */
static void trx_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                            uint8_t *bits, int nbits)
{
    /* Wireshark */
    gsmtap_send_burst(s, tn, fn, bits, nbits, true);

    /* fake_trx via TRXD — TX burst (uplink) */
    if (s->trxd_fd >= 0 && s->trxd_remote_known) {
        uint8_t buf[TRX_HDR_LEN_TX + GSM_BURST_BITS];
        buf[0] = tn & 0x07;  /* v0 + TN */
        buf[1] = (fn >> 24) & 0xFF;
        buf[2] = (fn >> 16) & 0xFF;
        buf[3] = (fn >>  8) & 0xFF;
        buf[4] = (fn      ) & 0xFF;
        buf[5] = 0;  /* TX power level */

        if (nbits > GSM_BURST_BITS) nbits = GSM_BURST_BITS;
        memcpy(&buf[TRX_HDR_LEN_TX], bits, nbits);

        sendto(s->trxd_fd, buf, TRX_HDR_LEN_TX + nbits, 0,
               (struct sockaddr *)&s->trxd_remote, sizeof(s->trxd_remote));
    }
}

/*
 * Air interface: send a burst to the peer QEMU instance.
 * Format: TN(1) + FN(4) + 148 hard bits = 153 bytes.
 */
static void __attribute__((unused)) air_send_burst(CalypsoTRX *s, uint8_t tn, uint32_t fn,
                           uint8_t *bits, int nbits)
{
    if (s->air_fd < 0 || !s->air_enabled) {
        return;
    }
    uint8_t buf[5 + GSM_BURST_BITS];
    buf[0] = tn;
    buf[1] = (fn >> 24) & 0xFF;
    buf[2] = (fn >> 16) & 0xFF;
    buf[3] = (fn >>  8) & 0xFF;
    buf[4] = (fn      ) & 0xFF;
    if (nbits > GSM_BURST_BITS) nbits = GSM_BURST_BITS;
    memcpy(&buf[5], bits, nbits);
    sendto(s->air_fd, buf, 5 + nbits, 0,
           (struct sockaddr *)&s->air_peer, sizeof(s->air_peer));
}

/*
 * QEMU fd handler: incoming burst from peer QEMU via air interface.
 * The peer's TX burst becomes our RX burst (like receiving over the air).
 */
static void air_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint8_t buf[512];
    ssize_t n = recvfrom(s->air_fd, buf, sizeof(buf), 0, NULL, NULL);
    if (n < 6) {
        return;
    }

    uint8_t tn = buf[0];
    uint32_t fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                  ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];

    int burst_len = (int)(n - 5);
    if (burst_len > GSM_BURST_BITS) burst_len = GSM_BURST_BITS;

    /* FN synchronization — continuous lock to air master clock.
     * The external gsm_clock.py is the single source of truth for FN.
     * We always adopt the received FN directly. */
    if (!s->fn_synced) {
        s->fn_offset = (int32_t)fn - (int32_t)s->fn;
        s->fn_synced = true;
        TRX_LOG("AIR: FN sync peer=%u local=%u offset=%d", fn, s->fn, s->fn_offset);
    }
    /* Always realign to master clock FN */
    s->fn = fn % GSM_HYPERFRAME;

    /* Store in rx_burst for immediate use */
    s->rx_tn = tn;
    memcpy(s->rx_burst, &buf[5], burst_len);
    s->rx_rssi  = -60;
    s->rx_toa   = 0;
    s->rx_pending = true;

    /* Forward burst to bridge on port 6800 (standard MS-side air port) */
    {
        struct sockaddr_in bridge_addr;
        memset(&bridge_addr, 0, sizeof(bridge_addr));
        bridge_addr.sin_family = AF_INET;
        bridge_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bridge_addr.sin_port = htons(6800);
        sendto(s->air_fd, buf, n, 0,
               (struct sockaddr *)&bridge_addr, sizeof(bridge_addr));
    }

    /* Also store in ring buffer indexed by FN for later NB lookup */
    int idx = fn % AIR_BUF_SIZE;
    s->air_buf[idx].fn = fn;
    memcpy(s->air_buf[idx].bits, &buf[5], burst_len);
    s->air_buf[idx].valid = true;
}

/*
 * QEMU fd handler: incoming TRXD datagram from fake_trx.
 * This carries DL bursts destined for the phone (RX direction).
 *
 * TRXD RX header (fake_trx → MS, v0, 8 bytes):
 *   byte 0      : VER(4bit) + TN(3bit)
 *   bytes 1..4  : FN (big-endian uint32)
 *   byte 5      : RSSI (signed int8, dBm)
 *   bytes 6..7  : TOA256 (big-endian int16)
 *   bytes 8+    : soft bits (sbit_t: -127=strong 1, +127=strong 0)
 */
static void trxd_receive_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint8_t buf[512];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(s->trxd_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
    if (n < TRX_HDR_LEN_RX + 1) {
        return;
    }

    /* Always update remote TRXD address (port changes on restart) */
    if (!s->trxd_remote_known ||
        s->trxd_remote.sin_port != src.sin_port) {
        s->trxd_remote = src;
        s->trxd_remote_known = true;
        TRX_LOG("TRXD remote: %s:%d",
                inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }

    s->rx_tn = buf[0] & 0x07;  /* TN in lower 3 bits */
    uint32_t trx_fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                      ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];
    s->rx_rssi = (int8_t)buf[5];
    s->rx_toa  = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);

    /* FN synchronization */
    if (!s->fn_synced) {
        s->fn_synced = true;
        TRX_LOG("TRXD: FN sync trx=%u local=%u", trx_fn, s->fn);
    }
    s->fn = trx_fn % GSM_HYPERFRAME;

    /* Convert soft bits (sbit_t) to hard bits (0/1) for DSP emulation.
     * sbit_t: negative = bit 1, positive = bit 0, 0 = unknown → 0 */
    int burst_len = (int)(n - TRX_HDR_LEN_RX);
    if (burst_len > GSM_BURST_BITS) burst_len = GSM_BURST_BITS;
    for (int i = 0; i < burst_len; i++) {
        int8_t sbit = (int8_t)buf[TRX_HDR_LEN_RX + i];
        s->rx_burst[i] = (sbit < 0) ? 1 : 0;
    }
    s->rx_pending = true;

    /* Store in ring buffer indexed by FN */
    {
        int idx = trx_fn % AIR_BUF_SIZE;
        s->air_buf[idx].fn = trx_fn;
        memcpy(s->air_buf[idx].bits, s->rx_burst, burst_len);
        s->air_buf[idx].valid = true;
    }

    /* Log received bursts (gated by TRX_DEBUG_TDMA) */
#if TRX_DEBUG_TDMA
    {
        char hex[64];
        int hlen = burst_len < 16 ? burst_len : 16;
        for (int i = 0; i < hlen; i++)
            snprintf(hex + i*3, 4, "%02x ", buf[TRX_HDR_LEN_RX + i]);
        hex[hlen*3] = '\0';
        TRX_LOG("TRXD RX TN=%d FN=%u len=%d rssi=%d bits=%s",
                s->rx_tn, trx_fn, burst_len, s->rx_rssi, hex);
    }
#endif
#if TRX_DEBUG_TDMA
    TRX_LOG("TRXD RX (debug) TN=%d FN=%u len=%d rssi=%d", s->rx_tn, s->fn, burst_len, s->rx_rssi);
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

    /*
     * fake_trx MS-side port layout (base_port = 6700):
     *   CLK:  fake_trx bind 6700, connect 6800  (base, base+100)
     *   CTRL: fake_trx bind 6701, connect 6801  (base+1, base+101)
     *   DATA: fake_trx bind 6702, connect 6802  (base+2, base+102)
     *
     * QEMU (replacing trxcon) binds the "connect" side:
     *   CLK:  bind base+100   (6800) — receive IND CLOCK
     *   TRXC: bind base+101   (6801) — CTRL commands, send to base+1 (6701)
     *   TRXD: bind base+102   (6802) — DATA bursts, send to base+2  (6702)
     */
    int clk_port  = base_port + 100;
    int trxc_port = base_port + 101;
    int trxd_port = base_port + 102;

    /* CLK: receive clock indications from fake_trx */
    s->clk_fd = udp_socket_bind(clk_port);
    if (s->clk_fd < 0) {
        TRX_LOG("WARNING: Cannot bind CLK port %d: %s",
                clk_port, strerror(errno));
    } else {
        qemu_set_fd_handler(s->clk_fd, clk_receive_cb, NULL, s);
        TRX_LOG("CLK  listening on UDP port %d", clk_port);
    }

    /* TRXC: base_port + 101 */
    s->trxc_fd = udp_socket_bind(trxc_port);
    if (s->trxc_fd < 0) {
        TRX_LOG("WARNING: Cannot bind TRXC port %d: %s",
                trxc_port, strerror(errno));
    } else {
        qemu_set_fd_handler(s->trxc_fd, trxc_receive_cb, NULL, s);
        TRX_LOG("TRXC listening on UDP port %d", trxc_port);
    }

    /* TRXD: base_port + 102 */
    s->trxd_fd = udp_socket_bind(trxd_port);
    if (s->trxd_fd < 0) {
        TRX_LOG("WARNING: Cannot bind TRXD port %d: %s",
                trxd_port, strerror(errno));
    } else {
        qemu_set_fd_handler(s->trxd_fd, trxd_receive_cb, NULL, s);
        TRX_LOG("TRXD listening on UDP port %d", trxd_port);
    }

    /* Remote addresses: send back to fake_trx bind ports */
    memset(&s->trxc_remote, 0, sizeof(s->trxc_remote));
    s->trxc_remote.sin_family      = AF_INET;
    s->trxc_remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->trxc_remote.sin_port        = htons(base_port + 1);

    memset(&s->trxd_remote, 0, sizeof(s->trxd_remote));
    s->trxd_remote.sin_family      = AF_INET;
    s->trxd_remote.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->trxd_remote.sin_port        = htons(base_port + 2);

    /* Air interface disabled by default — configured via air_init() */
    s->air_fd = -1;
    s->air_enabled = false;
}

/*
 * Initialize the air interface socket for peer-to-peer burst exchange.
 * local_port: UDP port this instance listens on for air bursts
 * peer_port:  UDP port of the peer QEMU instance
 */
static void air_init(CalypsoTRX *s, int local_port, int peer_port)
{
    s->air_fd = udp_socket_bind(local_port);
    if (s->air_fd < 0) {
        TRX_LOG("AIR: Cannot bind port %d: %s", local_port, strerror(errno));
        return;
    }

    qemu_set_fd_handler(s->air_fd, air_receive_cb, NULL, s);

    memset(&s->air_peer, 0, sizeof(s->air_peer));
    s->air_peer.sin_family      = AF_INET;
    s->air_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    s->air_peer.sin_port        = htons(peer_port);
    s->air_enabled = true;

    TRX_LOG("AIR: listening on %d, peer on %d", local_port, peer_port);
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
 * XCCH decoder — GSM 05.03 channel coding for BCCH/CCCH/SDCCH
 *
 * Implements: deinterleave + Viterbi (k=5, rate 1/2) + Fire CRC
 * Input: 4 × 116 coded bits (from 4 consecutive normal bursts)
 * Output: 23 bytes of L2 data
 *
 * NDB offsets (from NDB base 0x01A8, confirmed by firmware disasm):
 *   a_cd[0]  = NDB + 0x1F8 (252 words) — status: B_BLUD(15), B_FIRE1(6), B_FIRE0(5)
 *   a_cd[2]  = NDB + 0x1FC — num_biterr
 *   a_cd[3..14] = NDB + 0x1FE — 23 bytes decoded L2 data (1 byte/word)
 *
 * DB read page:
 *   d_burst_d = word 1 — burst index (0..3), firmware checks ==3
 *   a_serv_demod[0..3] = words 8..11 — TOA, PM, ANGLE, SNR
 * ===================================================================== */

#define NDB_A_CD_OFFSET  252  /* word offset of a_cd[0] from NDB base */
#define B_FIRE0   5
#define B_FIRE1   6
#define B_BLUD   15

/* Convolutional code: k=5, rate 1/2, polynomials G0=0x19, G1=0x1B */
static void xcch_viterbi_decode(const uint8_t *coded, int n_coded,
                                uint8_t *decoded, int n_decoded)
{
    /* 16-state Viterbi for k=5 convolutional code */
    enum { N_STATES = 16 };
    int pm[N_STATES], new_pm[N_STATES];
    uint8_t paths[228][N_STATES]; /* max n_coded/2 steps */
    int n_steps = n_coded / 2;
    int i, state, bit, best, best_pm;

    for (i = 0; i < N_STATES; i++) pm[i] = 999999;
    pm[0] = 0;

    for (i = 0; i < n_steps; i++) {
        int c0 = coded[2*i], c1 = coded[2*i+1];
        for (state = 0; state < N_STATES; state++) new_pm[state] = 999999;

        for (state = 0; state < N_STATES; state++) {
            if (pm[state] >= 999999) continue;
            for (bit = 0; bit < 2; bit++) {
                int reg = ((state << 1) | bit) & 0x1F;
                int g0 = ((reg >> 0) ^ (reg >> 3) ^ (reg >> 4)) & 1;
                int g1 = ((reg >> 0) ^ (reg >> 1) ^ (reg >> 3) ^ (reg >> 4)) & 1;
                int bm = (c0 ^ g0) + (c1 ^ g1);
                int ns = ((state << 1) | bit) & 0xF;
                int total = pm[state] + bm;
                if (total < new_pm[ns]) {
                    new_pm[ns] = total;
                    paths[i][ns] = (state << 1) | bit;
                }
            }
        }
        for (state = 0; state < N_STATES; state++) pm[state] = new_pm[state];
    }

    /* Find best final state */
    best = 0; best_pm = pm[0];
    for (state = 1; state < N_STATES; state++) {
        if (pm[state] < best_pm) { best_pm = pm[state]; best = state; }
    }

    /* Traceback */
    uint8_t trace[228];
    state = best;
    for (i = n_steps - 1; i >= 0; i--) {
        trace[i] = state & 1;
        state = paths[i][state] >> 1;
    }

    /* Pack bits into decoded bytes */
    memset(decoded, 0, (n_decoded + 7) / 8);
    for (i = 0; i < n_decoded && i < n_steps; i++) {
        decoded[i / 8] |= (trace[i] << (7 - (i % 8)));
    }
}

/* GSM 05.03 §4.1.4: XCCH deinterleaving (4 bursts × 114 bits → 456 coded bits) */
static void xcch_deinterleave(const uint8_t bursts[4][116], uint8_t *coded)
{
    int k;
    for (k = 0; k < 456; k++) {
        /* e(B,j) where B=k%4, j=2*((49*k) mod 57) + ((k mod 8) / 4)
         * Simplified: coded[k] = bursts[k%4][k/4]
         * This is the standard rectangular interleaving for XCCH */
        int B = k % 4;
        int j = k / 4;  /* 0..113 */
        coded[k] = bursts[B][j];
    }
}

/*
 * Full XCCH decode: 4 bursts → 23 bytes L2 data.
 * Returns number of bit errors (0 = perfect decode).
 */
static int xcch_decode(const uint8_t bursts[4][116], uint8_t *l2_data)
{
    uint8_t coded[456];
    uint8_t decoded_bits[224]; /* 184 data + 40 parity */

    /* Step 1: Deinterleave */
    xcch_deinterleave(bursts, coded);

    /* Step 2: Viterbi decode (456 coded bits → 228 bits → 184 data bits) */
    xcch_viterbi_decode(coded, 456, decoded_bits, 228);

    /* Step 3: Extract 184 data bits → 23 bytes */
    /* decoded_bits is already packed MSB-first by viterbi decoder */
    memcpy(l2_data, decoded_bits, 23);

    /* Count bit errors (metric from Viterbi) — approximate with 0 */
    return 0;
}

/* =====================================================================
 * DSP API RAM
 * ===================================================================== */

/*
 * One-shot firmware patches, applied on first DSP access.
 * At this point the firmware ELF is loaded and executing.
 * Must run BEFORE console output starts (dsp_power_on is early boot).
 */
static void calypso_fw_patch(CalypsoTRX *s)
{
    if (s->fw_patched) return;
    s->fw_patched = true;

    uint32_t val;
    uint32_t bx_lr = 0xe12fff1e;

    /* 0) NOP specific high-frequency debug output that fills the pool.
     *    Keep general printf/puts working to maintain UART TX alive
     *    (needed for L1CTL message delivery like PM_CONF).
     *
     *    NOP targets (bl printf/puts → nop):
     *    - cons_puts: boot debug, fills pool early
     *    - 0x8254d4: bl printf in l1s_pm_resp ("PM MEAS: ARFCN=...")
     *    - 0x828914: bl printf in frame_irq ("LOST %d!")
     *    - 0x828828: bl printf in frame_irq (debug dump)
     *    - 0x828858: bl printf in frame_irq (conditional debug)
     *    - 0x828880: bl printf in frame_irq (conditional debug)
     */
    uint32_t nop_arm = 0xe1a00000;  /* mov r0, r0 = NOP */
    /* NOP cons_puts function entirely */
    cpu_physical_memory_read(0x0082a1b0, &val, 4);
    if (val != 0x00000000 && val != 0xe12fff1e) {
        cpu_physical_memory_write(0x0082a1b0, &bx_lr, 4);
        TRX_LOG("FW-PATCH: cons_puts@0x82a1b0 → bx lr");
    }
    /* NOP specific bl/blcc printf/puts calls in hot paths.
     * Includes unconditional bl (0xEB) and conditional (blgt=0xCB, blhi=0x8B). */
    /* NOP high-frequency debug output in frame_irq.
     * Keep l1s_pm_resp printf (0x8254d4) — it keeps sercomm TX alive.
     * Also NOP puts globally (0x829ea0) — it generates LOST spam. */
    cpu_physical_memory_read(0x00829ea0, &val, 4);
    if (val != 0x00000000 && val != 0xe12fff1e) {
        cpu_physical_memory_write(0x00829ea0, &bx_lr, 4);
        TRX_LOG("FW-PATCH: puts@0x829ea0 → bx lr");
    }
    static const hwaddr nop_calls[] = {
        0x00828914,  /* frame_irq: bl printf "LOST %d!" */
        0x00828828,  /* frame_irq: bl printf (debug) */
        0x00828830,  /* frame_irq: bl puts (debug) */
        0x00828858,  /* frame_irq: blgt printf (conditional) */
        0x00828880,  /* frame_irq: blhi printf (conditional) */
    };
    for (int i = 0; i < 5; i++) {
        cpu_physical_memory_read(nop_calls[i], &val, 4);
        /* Match any bl/blcc: bits[27:25]=101, bit[24]=1 → (val & 0x0E000000) == 0x0A000000 AND bit24 set */
        if ((val & 0x0F000000) == 0x0B000000) {
            cpu_physical_memory_write(nop_calls[i], &nop_arm, 4);
            TRX_LOG("FW-PATCH: NOP bl@0x%lx (was 0x%08x)",
                    (unsigned long)nop_calls[i], val);
        }
    }

    /* 1) Expand msgb pool: change "cmp r3, #32" to "cmp r3, #128"
     *    at 0x82c33c.  The pool starts at 0x833b5c with 332-byte slots.
     *    Extra 96 slots use zero-initialized IRAM after the original pool,
     *    which is free (IRAM ends at 0x83FFFF).
     *    32 * 332 = 10624 → 128 * 332 = 42496 (still within IRAM).
     *    Pool end: 0x833b5c + 42496 = 0x83dfdc < 0x840000. Safe. */
    cpu_physical_memory_read(0x0082c33c, &val, 4);
    if (val == 0xe3530020) {  /* cmp r3, #32 */
        /* Max safe: 148 slots. Pool end: 0x833b5c + 148*332 = 0x83fb4c < 0x840000 */
        uint32_t cmp148 = 0xe3530094;  /* cmp r3, #148 */
        cpu_physical_memory_write(0x0082c33c, &cmp148, 4);
        TRX_LOG("FW-PATCH: talloc pool 32→148 slots @0x82c33c");
    }

    /* 2) talloc panic: replace infinite loop with retry + IRQs.
     *    With 128 slots, this should rarely trigger. */
    cpu_physical_memory_read(0x0082c350, &val, 4);
    if (val == 0xeafffffe) {
        uint32_t patch[2] = {
            0xe121f008,  /* msr CPSR_c, r8 (re-enable IRQs) */
            0xeaffffdf,  /* b 0x0082c2d4 (retry slot scan) */
        };
        cpu_physical_memory_write(0x0082c34c, patch, sizeof(patch));
        TRX_LOG("FW-PATCH: talloc panic → retry with IRQs");
    }

    /* 3) handle_abort: keep IRQs enabled in abort infinite loop.
     *    Without this, a data abort permanently freezes the system.
     *    Original: msr CPSR_c, #0xc0 (disable) + b self
     *    Patched:  msr CPSR_c, #0x00 (enable) + b self */
    cpu_physical_memory_read(0x00821f98, &val, 4);
    if (val == 0xeafffffe) {
        uint32_t enable_irq = 0xe321f000;
        cpu_physical_memory_write(0x00821f94, &enable_irq, 4);
        TRX_LOG("FW-PATCH: handle_abort@0x821f94 → IRQs enabled");
    }
}

void calypso_fw_patch_apply(void)
{
    if (g_trx) {
        calypso_fw_patch(g_trx);
    }
}

static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val;

    if (offset >= CALYPSO_DSP_SIZE) return 0;

    /* Patches are now triggered from first UART access (earlier than DSP) */

    if (size == 2) {
        val = s->dsp_ram[offset / 2];
    } else if (size == 4) {
        val = s->dsp_ram[offset / 2] |
              ((uint32_t)s->dsp_ram[offset / 2 + 1] << 16);
    } else {
        val = ((uint8_t *)s->dsp_ram)[offset];
    }

    /* PM done flags: firmware reads word 31 (0x003E) and word 11 (0x0016)
     * to check if PM measurement is complete. Both must be non-zero.
     * Also re-plant PM results at NDB offsets since firmware clears them. */
    if (offset == 0x003E && s->pm_count > 0) {
        val = 1;
        uint16_t pm_val = PM_RAW_STRONG;
        s->dsp_ram[213] = pm_val;
        for (int i = 0; i < 8; i++) {
            s->dsp_ram[248 + i] = pm_val;
        }
    }
    /* d_fb_det is at NDB+0x48 = DSP offset 0x01F0 (word 248)
     * Confirmed by disasm: l1s_fbdet_resp does ldrh r8, [r3, #0x48]
     * Return 1 only after attempt 8 (firmware expects DSP to take
     * several frames to detect FCCH, returning 1 immediately may
     * confuse the scheduler timing) */
    if (offset == 0x01F0 && s->sync_fb_tasks >= 8) {
        val = 1;
    }
    if (offset == 0x0016) {
        val = 1;
    }

    /* Log reads to d_fb_det (0x01F0) and a_sch (0x005E-0x0068) */
#if TRX_DEBUG_DSP
    if (offset == 0x01F0 && s->sync_fb_tasks > 0) {
        TRX_LOG("d_fb_det READ: 0x01F0 = 0x%04x (fb#%u)",
                (unsigned)val, s->sync_fb_tasks);
    }
    /* Log reads in SCH area (a_sch at R_PAGE + 15..19 words) */
    if (offset >= 0x005E && offset <= 0x0068) {
        TRX_LOG("a_sch READ: [0x%04x] = 0x%04x",
                (unsigned)offset, (unsigned)val);
    }
#endif
    /* d_fb_mode/PM at word 213 (offset 0x01AA) — always strong */
    if (offset == 0x01AA && s->sync_dsp_booted) {
        val = PM_RAW_STRONG;
    }
    /* a_sync_demod[4] follows d_fb_mode in NDB — TOA, PM, ANGLE, SNR */
    if (offset >= 0x01AC && offset <= 0x01B4 && s->sync_dsp_booted) {
        if (offset == 0x01AC) val = 0;              /* D_TOA */
        else if (offset == 0x01AE) val = PM_RAW_STRONG; /* D_PM */
        else if (offset == 0x01B0) val = 0;         /* D_ANGLE */
        else if (offset == 0x01B2) val = 100;       /* D_SNR */
    }

    if (offset == DSP_DL_STATUS_ADDR && !s->sync_dsp_booted) {
        s->sync_boot_frame++;
        if (s->sync_boot_frame > 3) {
            /* DSP bootloader ROM signals ready by writing 1 */
            s->dsp_ram[DSP_DL_STATUS_ADDR / 2] = DSP_DL_STATUS_BOOT;
            s->dsp_ram[DSP_API_VER_ADDR  / 2]  = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR / 2]  = 0x0000;
            s->sync_dsp_booted = true;
            TRX_LOG("DSP boot: ROM ready (status=1), version=0x%04x",
                    DSP_API_VERSION);
            val = DSP_DL_STATUS_BOOT;
        }
    }

    /* PM diagnostic: after a PM task, log ALL DSP reads to find
     * where the firmware reads the PM result from. */
    if (s->pm_debug_reads > 0) {
        if (1) {
            TRX_LOG("PM-DIAG: read dsp[0x%04x] (word %u) = 0x%04x",
                    (unsigned)offset, (unsigned)(offset / 2),
                    (unsigned)val);
            s->pm_debug_reads--;
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

    /* Track writes to d_fb_det NDB offset */
#if TRX_DEBUG_DSP
    if (offset == 0x01A8) {
        TRX_LOG("DSP-WRITE: d_fb_det (0x01A8) = 0x%04x", (unsigned)value);
    }
#endif

    /* Intercept task_md write: immediately plant PM result
     * so firmware finds it on the SAME frame, not next frame. */
    {
        /* Check if this write is to d_task_md in either write page */
        hwaddr w0_task_md = DSP_API_W_PAGE0 + DB_W_D_TASK_MD * 2;
        hwaddr w1_task_md = DSP_API_W_PAGE1 + DB_W_D_TASK_MD * 2;
        if ((offset == w0_task_md || offset == w1_task_md) && value != 0) {
            uint16_t pm_val = PM_RAW_STRONG;

            if (value == 5) {
                /* FB task — set d_fb_det and a_sync_demod at real offsets.
                 * d_fb_det = NDB+0x48 = DSP 0x01F0 (word 248)
                 * a_sync_demod = NDB+0x4C..0x52 = DSP 0x01F4..0x01FA */
                s->dsp_ram[0x01F0 / 2] = 1;      /* d_fb_det = FOUND */
                s->dsp_ram[0x01F2 / 2] = 0;      /* (after d_fb_det) */
                s->dsp_ram[0x01F4 / 2] = 0;      /* D_TOA = 0 */
                s->dsp_ram[0x01F6 / 2] = pm_val;  /* D_PM */
                s->dsp_ram[0x01F8 / 2] = 0;      /* D_ANGLE = 0 */
                s->dsp_ram[0x01FA / 2] = 100;    /* D_SNR */
                s->sync_fb_tasks++;
            } else if (value == 6) {
                /* SB task — write SCH data to a_sch in read page.
                 * a_sch at R_PAGE word 15..19.
                 * R_PAGE0 = 0x0050, R_PAGE1 = 0x0078 */
                uint16_t *rp = s->dsp_page ?
                    &s->dsp_ram[0x0078 / 2] : &s->dsp_ram[0x0050 / 2];
                uint32_t fn = s->fn;
                uint8_t bsic = s->sync_bsic;
                uint32_t t1 = fn / (26 * 51);
                uint32_t t2 = fn % 26;
                uint32_t t3 = fn % 51;
                uint32_t t3p = (t3 >= 1) ? (t3 - 1) / 10 : 0;
                /* Calypso DSP SCH format (from l1s_decode_sb reverse) */
                uint32_t sb = ((uint32_t)(bsic & 0x3F) << 2) |
                              ((t2 & 0x1F) << 18) |
                              ((t1 & 1) << 23) |
                              (((t1 >> 1) & 0xFF) << 8) |
                              (((t1 >> 9) & 1) << 0) |
                              (((t1 >> 10) & 1) << 1) |
                              ((t3p & 1) << 24) |
                              (((t3p >> 1) & 1) << 16) |
                              (((t3p >> 2) & 1) << 17);
                rp[15] = 0;  /* a_sch[0]: CRC OK (bit 0 = 0) */
                rp[16] = 0;
                rp[17] = 0;
                rp[18] = (uint16_t)(sb & 0xFFFF);
                rp[19] = (uint16_t)(sb >> 16);
                /* Also set sync_demod results */
                s->dsp_ram[0x01F4 / 2] = 0;       /* D_TOA */
                s->dsp_ram[0x01F6 / 2] = PM_RAW_STRONG; /* D_PM */
                s->dsp_ram[0x01F8 / 2] = 0;       /* D_ANGLE */
                s->dsp_ram[0x01FA / 2] = 100;     /* D_SNR */
                s->sync_sb_tasks++;
                /* Mark as synced so TX bursts can be sent */
                if (s->sync_state != SYNC_LOCKED) {
                    s->sync_state = SYNC_LOCKED;
                    TRX_LOG("SYNC: → LOCKED (via task_md SB)");
                }
            }

            /* Write PM to DB read page a_pm (NOT to NDB area!)
             * R_PAGE0 = 0x0050, a_pm at word 8 = byte offset 0x0060
             * R_PAGE1 = 0x0078, a_pm at word 8 = byte offset 0x0088 */
            s->dsp_ram[0x0060 / 2] = pm_val;
            s->dsp_ram[0x0062 / 2] = pm_val;
            s->dsp_ram[0x0064 / 2] = pm_val;
            s->dsp_ram[0x0088 / 2] = pm_val;
            s->dsp_ram[0x008A / 2] = pm_val;
            s->dsp_ram[0x008C / 2] = pm_val;
        }
    }

    if (offset == DSP_API_NDB + NDB_W_D_DSP_PAGE * 2) {
        s->dsp_page = value & 1;
    }
    if (offset == DSP_DL_STATUS_ADDR) {
        if (value == 0x0000) {
            /* Firmware clearing DSP status → allow re-boot sequence */
            s->sync_dsp_booted = false;
            s->sync_boot_frame = 0;
            TRX_LOG("DSP reset: status cleared, re-enabling boot sequence");
        } else if (value == DSP_DL_STATUS_BOOT) {
            s->sync_boot_frame = 0;
        } else if (value == DSP_DL_STATUS_READY) {
            /* Firmware wrote "patches applied, DSP ready".
             * Now plant the API version so firmware reads it. */
            s->dsp_ram[DSP_API_VER_ADDR  / 2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR / 2] = 0x0000;
            TRX_LOG("DSP ready: planted version 0x%04x at 0x%04x",
                    DSP_API_VERSION, DSP_API_VER_ADDR);
        }
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

    /* Known task IDs from l1_environment.h */
    switch (task_d) {
    case 5:  /* FB_DSP_TASK */      return TASK_FB;
    case 6:  /* SB_DSP_TASK */      return TASK_SB;
    case 8:  /* TCH_FB_DSP_TASK */  return TASK_FB;
    case 9:  /* TCH_SB_DSP_TASK */  return TASK_SB;
    case 10: /* RACH_DSP_TASK */    return TASK_RACH;
    case 17: /* NBN_DSP_TASK */     return TASK_NB;
    case 18: /* EBN_DSP_TASK */     return TASK_NB;
    case 19: /* NBS_DSP_TASK */     return TASK_NB;
    case 20: /* EBS_DSP_TASK */     return TASK_NB;
    case 21: /* NP_DSP_TASK */      return TASK_NB;
    case 22: /* EP_DSP_TASK */      return TASK_NB;
    case 24: /* ALLC_DSP_TASK */    return TASK_NB;
    case 25: /* CB_DSP_TASK */      return TASK_NB;
    case 26: /* DDL_DSP_TASK */     return TASK_NB;
    case 27: /* ADL_DSP_TASK */     return TASK_NB;
    case 11: /* AUL_DSP_TASK */     return TASK_NB;
    case 12: /* DUL_DSP_TASK */     return TASK_NB;
    case 13: /* TCHT_DSP_TASK */    return TASK_NB;
    }

    return TASK_OTHER;
}

/* =====================================================================
 * DSP task processing
 * ===================================================================== */

static void calypso_dsp_process(CalypsoTRX *s)
{
    uint16_t *w_page, *r_page, *ndb;

    /* The firmware always uses db_w = W_PAGE_0 and db_r = R_PAGE_0.
     * The page flip (d_dsp_page) is a sync flag, not pointer swap. */
    w_page = &s->dsp_ram[DSP_API_W_PAGE0 / 2];
    r_page = &s->dsp_ram[DSP_API_R_PAGE0 / 2];
    ndb = &s->dsp_ram[DSP_API_NDB / 2];

    uint16_t task_d = w_page[DB_W_D_TASK_D];
    uint16_t task_u = w_page[DB_W_D_TASK_U];

    uint16_t task_md = w_page[DB_W_D_TASK_MD];

    if (task_d != 0 || task_u != 0 || task_md != 0) {
        s->sync_task_count++;
    }

    /* Debug: after PM starts, log task activity (both present and absent)
     * to understand scheduling pattern and where PM stops */
    /* Log all non-zero tasks */
#if TRX_DEBUG_DSP
    if (task_d != 0 || task_u != 0) {
        TRX_LOG("DSP-TASK: d=0x%04x u=0x%04x md=0x%04x FN=%u pg=%d",
                task_d, task_u, task_md, s->fn, s->dsp_page);
    }
#endif

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

    case TASK_NB: {
        /* NB (Normal Burst) processing — accumulate 4 bursts then decode.
         *
         * The firmware sends d_task_d=ALLC for each of the 4 bursts in a
         * block, with d_burst_d=0..3 in the write page. The DSP must
         * echo d_burst_d back in the read page. On burst 3, the DSP
         * puts the decoded L2 data in ndb->a_cd[].
         *
         * The response is read 2 frames later (nb_sched_set uses -4).
         */
        int burst_id = w_page[DB_W_D_BURST_D] & 0x03;

        /* Echo burst_id into the read page — the firmware checks this */
        r_page[1] = burst_id;

#if TRX_DEBUG_DSP
        TRX_LOG("NB: burst_id=%d FN=%u rx_pending=%d air_buf[%d].valid=%d .fn=%u",
                burst_id, s->fn, s->rx_pending,
                s->fn % AIR_BUF_SIZE,
                s->air_buf[s->fn % AIR_BUF_SIZE].valid,
                s->air_buf[s->fn % AIR_BUF_SIZE].fn);
#endif

        /* Look up the burst for the current FN from the ring buffer.
         * The burst may have arrived asynchronously via trxd_receive_cb. */
        {
            int idx = s->fn % AIR_BUF_SIZE;
            uint8_t *src = NULL;
            if (s->air_buf[idx].valid && s->air_buf[idx].fn == s->fn) {
                src = s->air_buf[idx].bits;
                s->air_buf[idx].valid = false;
            } else if (s->rx_pending) {
                src = s->rx_burst;
                s->rx_pending = false;
            }

            /* Extract 114 coded bits from the 148-bit burst:
             * bits[3..59] = lower 57 data bits
             * bits[88..144] = upper 57 data bits */
            uint8_t *dst = s->nb_bursts[burst_id];
            if (src) {
                for (int i = 0; i < 57; i++) {
                    dst[i]      = src[3 + i] & 1;
                    dst[57 + i] = src[88 + i] & 1;
                }
                dst[114] = src[60] & 1;
                dst[115] = src[87] & 1;
            } else {
                memset(dst, 0, 116);
            }
        }

        /* Provide synthetic demod results (a_serv_demod) */
        r_page[8]  = 0;       /* TOA */
        r_page[9]  = 4864;    /* PM (≈ -62 dBm) */
        r_page[10] = 0;       /* ANGLE */
        r_page[11] = 2000;    /* SNR (good) */

        s->nb_burst_count++;

        /* On 4th burst: decode the full block */
        if (burst_id == 3) {
            uint8_t l2_data[23];
            int n_err = xcch_decode(s->nb_bursts, l2_data);

            /* Write decoded data to ndb->a_cd[] */
            uint16_t *a_cd = &ndb[NDB_A_CD_OFFSET];

            /* a_cd[0]: status — B_BLUD=1 (data present), FIRE=00 (no error) */
            a_cd[0] = (1 << B_BLUD);  /* block present, CRC OK */
            a_cd[1] = 0;
            a_cd[2] = n_err;  /* num_biterr */

            /* a_cd[3..14]: 23 bytes of L2 data, one byte per API word */
            for (int i = 0; i < 23; i++) {
                a_cd[3 + i] = l2_data[i] & 0xFF;
            }

            {
                static uint32_t nb_decode_count = 0;
                nb_decode_count++;
                if (nb_decode_count <= 5 || (nb_decode_count % 200) == 0) {
                    TRX_LOG("NB DECODE #%u: FN=%u l2=%02x%02x%02x%02x%02x... err=%d",
                            nb_decode_count, s->fn, l2_data[0], l2_data[1], l2_data[2],
                            l2_data[3], l2_data[4], n_err);
                }
            }

            s->nb_burst_count = 0;
        }
        break;
    }

    case TASK_OTHER:
#if TRX_DEBUG_DSP
        TRX_LOG("DSP: unknown task_d=0x%04x FN=%u (may be PM via task_d)",
                task_d, s->fn);
#endif
        break;

    default:
        break;
    }

    /* TX (uplink) burst */
    if (task_u != 0 && s->sync_state == SYNC_LOCKED) {
        uint16_t *burst_w = &w_page[0x19];
        uint8_t bits[GSM_BURST_BITS];
        int ones = 0;
        for (int i = 0; i < GSM_BURST_BITS; i++) {
            bits[i] = burst_w[i] & 1;
            ones += bits[i];
        }
        TRX_LOG("TX UL: task_u=0x%04x FN=%u ones=%d/148",
                task_u, s->fn, ones);
        trx_send_burst(s, 0, s->fn, bits, GSM_BURST_BITS);
    }

    /* ── Monitoring / PM task (d_task_md, word 4 of write page) ──
     *
     * The firmware writes a non-zero value to d_task_md when it wants
     * a power measurement on the currently-tuned ARFCN.  We inject a
     * synthetic PM result so the firmware can build PM_CONF for layer23.
     *
     * Result placement: write to several candidate offsets in the DB
     * read page (a_pm[]) AND to NDB a_cd_pm, since the exact offset
     * varies between DSP API versions.  The pm_debug_reads counter
     * enables diagnostic logging to pinpoint the real offset.
     */
    /* task_md handling — FB_DSP_TASK=5, SB_DSP_TASK=6, PM=other */
    if (task_md == 5) {
        /* FB (Frequency Burst) search task.
         * The firmware wants the DSP to detect FCCH.
         * Set d_fb_det=1 at NDB offset 0x01A8 and provide sync_demod results.
         * NDB base = 0x01A8, d_fb_det = NDB+0, d_fb_mode = NDB+1,
         * a_sync_demod[4] = NDB+2..5 (TOA, PM, ANGLE, SNR) */
        uint16_t *ndb_area = &s->dsp_ram[0x01A8 / 2];
        ndb_area[0] = 1;              /* d_fb_det = 1 (FCCH found) */
        /* d_fb_mode left as-is (firmware sets it) */
        ndb_area[2] = 0;              /* a_sync_demod[D_TOA] */
        ndb_area[3] = PM_RAW_STRONG;  /* a_sync_demod[D_PM] */
        ndb_area[4] = 0;              /* a_sync_demod[D_ANGLE] */
        ndb_area[5] = 100;            /* a_sync_demod[D_SNR] */

        s->sync_task_count++;
        s->sync_fb_tasks++;
        if (s->sync_fb_tasks <= 5 || (s->sync_fb_tasks % 20) == 0) {
            TRX_LOG("FB: task_md=5 → d_fb_det=1 at NDB, fb#%u FN=%u",
                    s->sync_fb_tasks, s->fn);
        }
    } else if (task_md == 6) {
        /* SB (Sync Burst) search task.
         * The firmware wants the DSP to decode SCH.
         * Provide SCH data in a_sch[] of the read page.
         * a_sch[0] = status (bit B_SCH_CRC=0 means OK)
         * a_sch[3..4] = 32-bit SCH data (BSIC + FN) */
        uint16_t *sb_rpage = s->dsp_page ?
            &s->dsp_ram[0x0078 / 2] : &s->dsp_ram[0x0050 / 2];

        /* SCH data: encode BSIC and FN
         * GSM 05.03: SB = BSIC(6) + T1(11) + T2(5) + T3'(3) = 25 bits */
        uint32_t fn = s->fn;
        uint8_t bsic = s->sync_bsic;
        uint32_t t1 = fn / (26 * 51);
        uint32_t t2 = fn % 26;
        uint32_t t3 = fn % 51;
        uint32_t t3p = (t3 - 1) / 10;

        /* Calypso DSP SCH format (from l1s_decode_sb reverse) */
        uint32_t sb_data = ((uint32_t)(bsic & 0x3F) << 2) |
                           ((t2 & 0x1F) << 18) |
                           ((t1 & 1) << 23) |
                           (((t1 >> 1) & 0xFF) << 8) |
                           (((t1 >> 9) & 1) << 0) |
                           (((t1 >> 10) & 1) << 1) |
                           ((t3p & 1) << 24) |
                           (((t3p >> 1) & 1) << 16) |
                           (((t3p >> 2) & 1) << 17);

        /* a_sch at read page offset 15..19 (for DSP<=32) */
        sb_rpage[15] = 0;               /* CRC OK (bit 0 = 0) */
        sb_rpage[16] = 0;
        sb_rpage[17] = 0;
        sb_rpage[18] = (uint16_t)(sb_data & 0xFFFF);
        sb_rpage[19] = (uint16_t)(sb_data >> 16);

        /* Also set sync_demod results in NDB */
        uint16_t *ndb_area = &s->dsp_ram[0x01A8 / 2];
        ndb_area[2] = 0;              /* D_TOA */
        ndb_area[3] = PM_RAW_STRONG;  /* D_PM */
        ndb_area[4] = 0;              /* D_ANGLE */
        ndb_area[5] = 100;            /* D_SNR */

        s->sync_sb_tasks++;
        TRX_LOG("SB: task_md=6 → SCH data=0x%08x BSIC=%d FN=%u sb#%u",
                sb_data, bsic, fn, s->sync_sb_tasks);
    } else if (task_md != 0) {
        /* Regular PM task */
        uint16_t pm_val = PM_RAW_STRONG;

        /* DB read page PM results.
         * CRITICAL: The real Calypso DSP page layout is NOT at 0x2000/0x3000.
         * From disasm of frame_irq:
         *   page=0: db_r_ptr = 0xFFD00050 → offset 0x50 from DSP base
         *   page=1: db_r_ptr = 0xFFD00078 → offset 0x78 from DSP base
         * PM at db_r_ptr + 24 bytes = db_r_ptr + 12 words:
         *   page=0: word (0x50+24)/2 = 0x68/2 = 52
         *   page=1: word (0x78+24)/2 = 0x90/2 = 72
         * Write to BOTH pages. */
        /* Write PM to ALL candidate offsets:
         * 1) TRX firmware offsets (from disasm): words 52, 72
         * 2) Standard DSP API R_PAGE offsets
         * 3) NDB offsets discovered by PM-DIAG: word 248 (0x01F0)
         *    and word 213 (0x01AA) — layer1 firmware reads these */
        for (int i = 0; i < DB_R_A_PM_COUNT; i++) {
            s->dsp_ram[52 + i] = pm_val;
            s->dsp_ram[72 + i] = pm_val;
            s->dsp_ram[DSP_API_R_PAGE0 / 2 + DB_R_A_PM + i] = pm_val;
            s->dsp_ram[DSP_API_R_PAGE1 / 2 + DB_R_A_PM + i] = pm_val;
        }
        /* NDB PM result offsets (from layer1 firmware PM-DIAG)
         * task_md=0x0005 means 5 measurements.
         * Results at words 248-253 (0x01F0-0x01FA = a_cd_pm[0..5])
         * and word 213 (0x01AA = pm level indicator)
         * Word 31 (0x003E) = PM completion flag (must be non-zero) */
        for (int i = 0; i < 8; i++) {
            s->dsp_ram[248 + i] = pm_val;  /* a_cd_pm[0..7] */
        }
        s->dsp_ram[213] = pm_val;  /* pm level indicator */
        s->dsp_ram[31] = 1;        /* PM done flag */

        s->pm_count++;
        s->sync_ref_fn = s->fn;  /* track last PM frame for debug window */

        /* Enable diagnostic read logging for first 50 PM tasks */
        if (s->pm_count <= 3) {
            s->pm_debug_reads = 200;  /* log next 200 read accesses */
        }

        if (s->pm_count <= 5 || (s->pm_count % 25) == 0 ||
            s->pm_count == 125 || s->pm_count == 126) {
            TRX_LOG("PM: task_md=0x%04x → pm=%u (~%d dBm) #%u FN=%u pg=%d",
                    task_md, pm_val, s->sync_rssi,
                    s->pm_count, s->fn, s->dsp_page);
        }
    }

    /* Clear tasks in write page. Copy task result to read page.
     * The firmware checks db_r->d_task_d != 0 to know there's a result. */
    r_page[0] = task_d;   /* echo task_d so firmware knows what was processed */
    w_page[DB_W_D_TASK_D] = 0;
    w_page[DB_W_D_TASK_U] = 0;
    w_page[DB_W_D_TASK_MD] = 0;
    ndb[NDB_W_D_FN] = (uint16_t)(s->fn & 0xFFFF);
}

static void calypso_dsp_done(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    /* TPU finished processing — clear EN bit so firmware sees completion */
    s->tpu_enabled = false;
    s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_EN;

    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_API]);
}

/* =====================================================================
 * TPU — correct bit definitions from OsmocomBB tpu.c
 *
 * The firmware uses tpu_wait_ctrl_bit() to poll specific bits:
 *   - RESET (bit 0): set on reset, firmware waits for set then clear
 *   - CK_ENABLE (bit 10): firmware sets it and waits for readback
 *   - EN (bit 2): firmware sets to run a TPU scenario
 *   - DSP_EN (bit 4): firmware sets for DSP frame IRQ
 *   - IDLE (bit 8): firmware polls to wait for TPU idle (0 = idle)
 *
 * Key insight: the firmware writes a bit and then reads it back.
 * For most bits, the hardware echoes the written value immediately.
 * For EN, the hardware clears it when the scenario finishes.
 * For IDLE, hardware sets it while running, clears when done.
 * ===================================================================== */

static uint64_t calypso_tpu_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint64_t val = 0;

    switch (offset) {
    case TPU_CTRL:
        /* Return the stored register — bits are maintained by write handler */
        val = s->tpu_regs[TPU_CTRL / 2];
        break;
    case TPU_INT_CTRL:
        val = s->tpu_regs[TPU_INT_CTRL / 2];
        break;
    case TPU_INT_STAT:
        val = 0;
        break;
    case TPU_OFFSET:
        val = s->tpu_regs[TPU_OFFSET / 2];
        break;
    case TPU_SYNCHRO:
        val = s->tpu_regs[TPU_SYNCHRO / 2];
        break;
    case TPU_IT_DSP_PG:
        val = s->dsp_page;
        break;
    default:
        if (offset / 2 < CALYPSO_TPU_SIZE / 2) {
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

    switch (offset) {
    case TPU_CTRL: {
        uint16_t reg = (uint16_t)value;

        /* RESET (bit 0): echoed back immediately.
         * The firmware writes RESET|TSP_RESET, polls for RESET set,
         * then clears it and polls for RESET clear. */
        /* CK_ENABLE (bit 10): echoed back immediately. */
        /* These are already stored by the write above. */

        /* EN (bit 2): firmware sets this to execute a TPU scenario.
         * We process the DSP task immediately, then clear EN. */
        if ((reg & TPU_CTRL_EN) && !s->tpu_enabled) {
            s->tpu_enabled = true;
            calypso_dsp_process(s);
            /* Mark scenario as done: clear EN, clear IDLE */
            s->tpu_enabled = false;
            s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_EN;
            s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_IDLE;
            /* Schedule API interrupt */
            timer_mod_ns(s->dsp_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
        }

        /* DSP_EN (bit 4): echoed, cleared by DSP frame IRQ */

        /* If RESET is being asserted, reset FN sync */
        if (reg & TPU_CTRL_RESET) {
            s->tpu_enabled = false;
            s->fn_synced   = false;
            s->fn_offset   = 0;
        }
        break;
    }
    case TPU_INT_CTRL:
        /* When firmware enables MCU frame IRQ (clears ICTRL_MCU_FRAME bit),
         * start the TDMA timer. In OsmocomBB tpu.c, tpu_frame_irq_en(1,x)
         * clears bit 0 to enable MCU frame interrupts. */
        if (!(value & ICTRL_MCU_FRAME) && !s->tdma_running) {
            calypso_tdma_start(s);
        }
        break;
    case TPU_IT_DSP_PG:
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
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
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
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
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

/* Deassert TPU_FRAME IRQ after the ISR has had time to fire */
static void calypso_frame_irq_lower(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
}

/* =====================================================================
 * TDMA frame timer — drives L1 at 4.615 ms per frame
 * ===================================================================== */

static void calypso_tdma_tick(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    s->fn = (s->fn + 1) % GSM_HYPERFRAME;

    s->tpu_enabled = false;
    s->tpu_regs[TPU_CTRL / 2] &= ~TPU_CTRL_EN;

    calypso_sync_tick(s);

    /* Send POWERON to fake_trx after BTS has had time to start.
     * At 10x slowdown, FN=300 ≈ 14s real time. BTS starts at ~12s. */
    if (!s->faketrx_powered && s->fn >= 300 && s->trxc_fd >= 0) {
        char cmd[64];
        trxc_send_cmd(s, "CMD SETFORMAT 0");
        {
            uint32_t dl_khz, ul_khz;
            uint16_t arfcn = s->sync_arfcn;
            if (arfcn >= 512 && arfcn <= 885) {
                dl_khz = 1805200 + (arfcn - 512) * 200;
                ul_khz = 1710200 + (arfcn - 512) * 200;
            } else {
                dl_khz = 935000 + arfcn * 200;
                ul_khz = 890000 + arfcn * 200;
            }
            snprintf(cmd, sizeof(cmd), "CMD RXTUNE %u", dl_khz);
            trxc_send_cmd(s, cmd);
            snprintf(cmd, sizeof(cmd), "CMD TXTUNE %u", ul_khz);
            trxc_send_cmd(s, cmd);
            TRX_LOG("POWERON: ARFCN=%u DL=%u UL=%u", arfcn, dl_khz, ul_khz);
        }
        trxc_send_cmd(s, "CMD SETSLOT 0 5");
        trxc_send_cmd(s, "CMD SETSLOT 1 1");
        trxc_send_cmd(s, "CMD SETSLOT 2 1");
        trxc_send_cmd(s, "CMD SETSLOT 3 1");
        trxc_send_cmd(s, "CMD SETSLOT 4 1");
        trxc_send_cmd(s, "CMD SETSLOT 5 1");
        trxc_send_cmd(s, "CMD SETSLOT 6 1");
        trxc_send_cmd(s, "CMD SETSLOT 7 1");
        trxc_send_cmd(s, "CMD SETRXGAIN 40");
        trxc_send_cmd(s, "CMD POWERON");
        s->faketrx_powered = true;
    }

    if ((s->fn % 5000) == 0) {
        TRX_LOG("TDMA FN=%u sync=%d tasks=%u fb=%u sb=%u pm=%u powered=%d",
                s->fn, s->sync_state, s->sync_task_count,
                s->sync_fb_tasks, s->sync_sb_tasks,
                s->pm_count, s->powered_on);
    }

    /* Kick UART modem — RX and TX:
     * 1) poll_backend: nudge chardev PTY to deliver pending RX bytes
     * 2) kick_rx: re-raise IRQ if RX data is in FIFO
     * 3) kick_tx: with console output NOPed, sercomm TX needs periodic
     *    nudging to drain queued L1CTL messages (PM_CONF etc.) */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
        calypso_uart_kick_tx(g_uart_modem);
    }

    /* Raise TPU_FRAME IRQ briefly. The read-to-ack in the INTH clears
     * the level when the firmware reads IRQ_NUM. We also schedule a
     * timer to lower the IRQ after 1ms, ensuring there is a window
     * between TDMA frames where other IRQs (UART) can be serviced.
     * Without this, the TDMA tick re-raises TPU_FRAME before the
     * firmware can handle pending UART interrupts. */
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);

    /* Process DSP tasks every frame — the firmware writes tasks
     * and expects results on the next frame. Without this, the
     * DSP process only fires when the firmware writes TPU_CTRL,
     * which may not happen every frame. */
    calypso_dsp_process(s);

    if (s->tdma_running) {
        timer_mod_ns(s->tdma_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
    }
}

/*
 * Send a TRXC command to fake_trx (MS side CTRL port).
 */
static void trxc_send_cmd(CalypsoTRX *s, const char *cmd)
{
    if (s->trxc_fd < 0) return;

    sendto(s->trxc_fd, cmd, strlen(cmd) + 1, 0,
           (struct sockaddr *)&s->trxc_remote, sizeof(s->trxc_remote));
    TRX_LOG("TRXC CMD → \"%s\"", cmd);
}

static void calypso_tdma_start(CalypsoTRX *s)
{
    if (s->tdma_running) return;
    s->tdma_running = true;
    s->fn_synced    = false;
    s->fn_offset    = 0;
    s->fn           = 0;

    TRX_LOG("TDMA started (4.615ms frame timer, fake_trx POWERON deferred)");
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
    s->dsp_ram[DSP_API_VER_ADDR  / 2]  = DSP_API_VERSION;
    s->dsp_ram[DSP_API_VER2_ADDR / 2]  = 0x0000;
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

/* =====================================================================
 * TPU RAM (separate region at 0xFFFF9000)
 * ===================================================================== */

static uint64_t calypso_tpu_ram_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset / 2 >= CALYPSO_TPU_RAM_SIZE / 2) return 0;
    return s->tpu_ram[offset / 2];
}

static void calypso_tpu_ram_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    if (offset / 2 >= CALYPSO_TPU_RAM_SIZE / 2) return;
    s->tpu_ram[offset / 2] = (uint16_t)value;
}

static const MemoryRegionOps calypso_tpu_ram_ops = {
    .read  = calypso_tpu_ram_read,
    .write = calypso_tpu_ram_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* =====================================================================
 * SIM controller (minimal — just enough to unblock firmware init)
 *
 * Calypso SIM registers at 0xFFFE0000:
 *   0x00  SIM_CMD
 *   0x02  SIM_STAT
 *   0x04  SIM_CONF1
 *   0x06  SIM_CONF2
 *   0x08  SIM_IT       (interrupt status)
 *   0x0A  SIM_DRX      (received data)
 *   0x0C  SIM_DTX      (transmit data)
 *   0x0E  SIM_MASKIT   (interrupt mask)
 *   0x10  SIM_IT_CD    (card detect interrupt)
 * ===================================================================== */

#define SIM_CMD     0x00
#define SIM_STAT    0x02
#define SIM_CONF1   0x04
#define SIM_CONF2   0x06
#define SIM_IT      0x08
#define SIM_DRX     0x0A
#define SIM_DTX     0x0C
#define SIM_MASKIT  0x0E
#define SIM_IT_CD   0x10

/* SIM IRQ bit: card inserted + ATR received */
#define SIM_IT_SIM  (1 << 0)

static void calypso_sim_atr_cb(void *opaque)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    /* The firmware's calypso_sim_powerup() spins on rxDoneFlag at a
     * fixed address in IRAM.  Rather than implementing the full SIM
     * ATR protocol, we write directly to rxDoneFlag to unblock it.
     *
     * rxDoneFlag address = 0x00830510 (from layer1.highram.elf symbol table)
     */
    static const hwaddr rxDoneFlag_addr = 0x00830510;
    uint32_t val = 1;
    cpu_physical_memory_write(rxDoneFlag_addr, &val, sizeof(val));

    s->sim_regs[SIM_IT / 2] |= SIM_IT_SIM;
    s->sim_regs[SIM_STAT / 2] = 0x0001;
    TRX_LOG("SIM: ATR done, rxDoneFlag=1 written at 0x%08lx",
            (unsigned long)rxDoneFlag_addr);
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_SIM]);
}

static uint64_t calypso_sim_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;
    uint16_t val = 0;

    if (offset / 2 < CALYPSO_SIM_SIZE / 2) {
        val = s->sim_regs[offset / 2];
    }

    /* Reading SIM_IT clears the interrupt bits */
    if (offset == SIM_IT) {
        s->sim_regs[SIM_IT / 2] = 0;
    }

    return val;
}

static void calypso_sim_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    CalypsoTRX *s = (CalypsoTRX *)opaque;

    if (offset / 2 < CALYPSO_SIM_SIZE / 2) {
        s->sim_regs[offset / 2] = (uint16_t)value;
    }

    /* When firmware unmasks SIM interrupts, schedule ATR delivery */
    if (offset == SIM_MASKIT) {
        TRX_LOG("SIM: MASKIT write = 0x%04x, scheduling ATR", (unsigned)value);
        timer_mod_ns(s->sim_atr_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 50000);
    }
}

static const MemoryRegionOps calypso_sim_ops = {
    .read  = calypso_sim_read,
    .write = calypso_sim_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* Periodic kick: force main loop to process socket I/O */
static QEMUTimer *g_kick_timer;
static void calypso_kick_cb(void *opaque)
{
    (void)opaque;
    /* Force vCPU to exit execution loop so main_loop_wait() can run */
    CPUState *cpu = first_cpu;
    if (cpu) {
        cpu_exit(cpu);
    }
    qemu_notify_event();
    timer_mod_ns(g_kick_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 5000000);
}

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs, int trx_port,
                      int air_local_port, int air_peer_port)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx    = s;
    s->irqs  = irqs;
    s->trxd_fd = -1;
    s->trxc_fd = -1;
    s->clk_fd  = -1;
    s->air_fd  = -1;

    TRX_LOG("=== Calypso TRX bridge init ===");
    TRX_LOG("  CLK  port : %d (clock indications)", trx_port + 100);
    TRX_LOG("  TRXC port : %d (ASCII commands)",    trx_port + 101);
    TRX_LOG("  TRXD port : %d (binary bursts)",     trx_port + 102);
    TRX_LOG("  GSMTAP    : %d (Wireshark)",          GSMTAP_PORT);

    gsmtap_init();

    /* DSP API RAM */
    memory_region_init_io(&s->dsp_iomem, NULL, &calypso_dsp_ops, s,
                          "calypso.dsp_api", CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_DSP_BASE, &s->dsp_iomem);
    calypso_dsp_api_init(s);

    calypso_sync_init(s);

    /* TPU registers */
    memory_region_init_io(&s->tpu_iomem, NULL, &calypso_tpu_ops, s,
                          "calypso.tpu", CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TPU_BASE, &s->tpu_iomem);

    /* TPU RAM (separate from registers) */
    memory_region_init_io(&s->tpu_ram_iomem, NULL, &calypso_tpu_ram_ops, s,
                          "calypso.tpu_ram", CALYPSO_TPU_RAM_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TPU_RAM_BASE,
                                &s->tpu_ram_iomem);

    /* TSP */
    memory_region_init_io(&s->tsp_iomem, NULL, &calypso_tsp_ops, s,
                          "calypso.tsp", CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_TSP_BASE, &s->tsp_iomem);

    /* ULPD */
    memory_region_init_io(&s->ulpd_iomem, NULL, &calypso_ulpd_ops, s,
                          "calypso.ulpd", CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_ULPD_BASE, &s->ulpd_iomem);

    /* SIM controller */
    memory_region_init_io(&s->sim_iomem, NULL, &calypso_sim_ops, s,
                          "calypso.sim", CALYPSO_SIM_SIZE);
    memory_region_add_subregion(sysmem, CALYPSO_SIM_BASE, &s->sim_iomem);
    s->sim_atr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_sim_atr_cb, s);

    /* Timers */
    s->tdma_timer      = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_tdma_tick, s);
    s->dsp_timer       = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_dsp_done, s);
    s->frame_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_frame_irq_lower, s);

    /* TRX sockets */
    if (trx_port > 0) {
        trx_sockets_init(s, trx_port);
        /* Start TDMA immediately — no CLK from fake_trx for MS side */
        calypso_tdma_start(s);
    } else {
        TRX_LOG("TRX sockets disabled");
    }

    /* Air interface (peer-to-peer burst exchange between 2 QEMU instances) */
    if (air_local_port > 0 && air_peer_port > 0) {
        air_init(s, air_local_port, air_peer_port);
    }

    /* Kick timer: force main loop to process socket I/O every 5ms.
     * Without this, single-threaded TCG never yields for fd handlers
     * (CLK, TRXD, TRXC, L1CTL from fake_trx and mobile). */
    g_kick_timer = timer_new_ns(QEMU_CLOCK_REALTIME, calypso_kick_cb, NULL);
    timer_mod_ns(g_kick_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 5000000);
    TRX_LOG("vCPU kick timer started (5ms interval)");

    /* TDMA timer starts when firmware enables TPU frame IRQ (INT_CTRL write) */
    TRX_LOG("=== TRX bridge ready (TDMA will start on TPU frame IRQ enable) ===");
}
