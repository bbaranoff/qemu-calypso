/*
 * sercomm_gate.c — Sercomm DLCI router (PTY) + TRX UDP endpoint
 *
 * Two completely separate roles, matching real Calypso hardware:
 *
 *   1. PTY (UART modem) — sercomm HDLC stream from host (mobile/ccch_scan).
 *      DLCIs are re-wrapped and pushed to the UART RX FIFO so the firmware's
 *      sercomm driver parses them via the real code path. L1CTL = DLCI 5.
 *      No DLCI on the PTY ever carries radio bursts.
 *
 *   2. UDP TRX endpoint — replaces fake_trx for the BSP path.
 *      Three loopback sockets (CLK / TRXC / TRXD) talk to osmo-bts-trx.
 *      Incoming TRXD bursts are converted (sbit_t → int16) and handed
 *      to calypso_trx_rx_burst(), which feeds the C54x BSP and fires BRINT0.
 *
 * Port layout (base_port, default 6700, fake_trx convention):
 *   CLK : bind  base+100 (6800)  ← receive  "IND CLOCK <fn>"
 *   TRXC: bind  base+101 (6801)  ↔ ASCII   "CMD ..."/"RSP ..."
 *   TRXD: bind  base+102 (6802)  ↔ binary  v0 burst frames
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/error-report.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/sercomm_gate.h"

#define GATE_LOG(fmt, ...) \
    fprintf(stderr, "[gate] " fmt "\n", ##__VA_ARGS__)

/* ============================================================
 * 1. PTY side — sercomm HDLC parser (L1CTL only)
 * ============================================================ */

#define SERCOMM_FLAG   0x7E
#define SERCOMM_ESCAPE 0x7D
#define SERCOMM_XOR    0x20

#define GATE_BUF_SIZE  512

enum gate_rx_state {
    GATE_WAIT_FLAG,
    GATE_IN_FRAME,
    GATE_ESCAPE,
};

static uint8_t  sc_buf[GATE_BUF_SIZE];
static int      sc_len;
static enum gate_rx_state sc_state;

/*
 * Re-wrap a decoded sercomm frame (DLCI + CTRL + payload) and push it
 * back into the UART RX FIFO. The firmware's sercomm driver will parse
 * it again — keeping the firmware code path 100% identical to hardware.
 */
static void gate_push_to_fifo(CalypsoUARTState *s,
                              const uint8_t *frame, int len)
{
    { uint8_t _b = SERCOMM_FLAG; calypso_uart_inject_raw(s, &_b, 1); }
    for (int i = 0; i < len; i++) {
        uint8_t c = frame[i];
        if (c == SERCOMM_FLAG || c == SERCOMM_ESCAPE || c == 0x00) {
            { uint8_t _e = SERCOMM_ESCAPE; calypso_uart_inject_raw(s, &_e, 1); }
            { uint8_t _x = c ^ SERCOMM_XOR; calypso_uart_inject_raw(s, &_x, 1); }
        } else {
            calypso_uart_inject_raw(s, &c, 1);
        }
    }
    { uint8_t _b = SERCOMM_FLAG; calypso_uart_inject_raw(s, &_b, 1); }
}

void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size)
{
    for (int i = 0; i < size; i++) {
        uint8_t b = buf[i];

        switch (sc_state) {
        case GATE_WAIT_FLAG:
            if (b == SERCOMM_FLAG) {
                sc_state = GATE_IN_FRAME;
                sc_len = 0;
            } else {
                /* Pre-sercomm raw bytes pass through (loader/console). */
                calypso_uart_inject_raw(s, &b, 1);
            }
            break;

        case GATE_ESCAPE:
            if (sc_len < GATE_BUF_SIZE)
                sc_buf[sc_len++] = b ^ SERCOMM_XOR;
            sc_state = GATE_IN_FRAME;
            break;

        case GATE_IN_FRAME:
            if (b == SERCOMM_FLAG) {
                if (sc_len >= 2) {
                    /* Forward EVERY DLCI to firmware FIFO — no burst path
                     * on the PTY. The firmware ignores DLCIs it doesn't
                     * have a callback for. */
                    gate_push_to_fifo(s, sc_buf, sc_len);
                }
                sc_len = 0;
            } else if (b == SERCOMM_ESCAPE) {
                sc_state = GATE_ESCAPE;
            } else {
                if (sc_len < GATE_BUF_SIZE)
                    sc_buf[sc_len++] = b;
            }
            break;
        }
    }
}

/* ============================================================
 * 2. UDP TRX endpoint — talks to osmo-bts-trx
 * ============================================================ */

#define GSM_BURST_BITS  148
#define TRX_HDR_LEN_RX  6        /* TN(1)+FN(4)+RSSI(1)+TOA(2) */

typedef struct {
    int  base_port;
    int  clk_fd;
    int  trxc_fd;
    int  trxd_fd;
    bool trxc_remote_known;
    bool trxd_remote_known;
    struct sockaddr_in trxc_remote;
    struct sockaddr_in trxd_remote;
} GateUDP;

static GateUDP gw;

static int udp_bind(int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return -1;

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port        = htons(port),
    };
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/* ---------- CLK ---------- */

static void clk_cb(void *opaque)
{
    char buf[128];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(gw.clk_fd, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &slen);
    if (n <= 0) return;
    buf[n] = '\0';
    /* Just log occasionally — calypso_trx owns its own FN counter. */
    static unsigned cnt;
    if ((cnt++ % 256) == 0) {
        GATE_LOG("CLK %s", buf);
    }
}

/* ---------- TRXC ---------- */



/* Generic ack: echoes args verbatim. Format: "RSP <cmd> <status> <args...>"
 * osmo-bts trx_if cmd_matches_rsp() does strict params check on SETSLOT
 * and SETFORMAT — must echo args identically. Other cmds are tolerant. */
static void trxc_send_ack(const char *verb, const char *args, int status)
{
    if (gw.trxc_fd < 0 || !gw.trxc_remote_known) return;
    char buf[256];
    int len;
    if (args && *args)
        len = snprintf(buf, sizeof(buf), "RSP %s %d %s", verb, status, args);
    else
        len = snprintf(buf, sizeof(buf), "RSP %s %d", verb, status);
    sendto(gw.trxc_fd, buf, len + 1, 0,
           (struct sockaddr *)&gw.trxc_remote, sizeof(gw.trxc_remote));
}

static void trxc_handle(const char *cmd)
{
    if (strncmp(cmd, "CMD ", 4) != 0) return;
    const char *verb_args = cmd + 4;

    /* Split verb and args */
    char vbuf[32] = {0};
    int vlen = 0;
    while (vlen < (int)sizeof(vbuf) - 1 && verb_args[vlen] && verb_args[vlen] != ' ') {
        vbuf[vlen] = verb_args[vlen];
        vlen++;
    }
    vbuf[vlen] = 0;
    const char *args = verb_args[vlen] == ' ' ? verb_args + vlen + 1 : "";
    /* Strip trailing whitespace/null from args */
    char abuf[160] = {0};
    int alen = 0;
    while (args[alen] && args[alen] != '\0' && args[alen] != '\n' && args[alen] != '\r' && alen < (int)sizeof(abuf)-1) {
        abuf[alen] = args[alen];
        alen++;
    }
    while (alen > 0 && (abuf[alen-1] == ' ' || abuf[alen-1] == '\t')) abuf[--alen] = 0;

    GATE_LOG("TRXC CMD %s args='%s'", vbuf, abuf);

    /* NOMTXPOWER: append nominal output power "50" after status */
    if (!strcmp(vbuf, "NOMTXPOWER")) {
        trxc_send_ack(vbuf, "50", 0);
        return;
    }
    /* MEASURE <freq>: append "<freq> -60" after status */
    if (!strcmp(vbuf, "MEASURE")) {
        char m[224]; snprintf(m, sizeof(m), "%s -60", abuf);
        trxc_send_ack(vbuf, m, 0);
        return;
    }
    /* All other commands: success, echo args verbatim. Covers
     * POWERON/POWEROFF, RXTUNE/TXTUNE, SETSLOT, SETPOWER, ADJPOWER,
     * SETMAXDLY, SETTSC, SETBSIC, RFMUTE, SETFORMAT,
     * HANDOVER/NOHANDOVER, SETRXGAIN, NOMRXLEV, etc. */
    trxc_send_ack(vbuf, abuf, 0);
}

static void trxc_cb(void *opaque)
{
    char buf[256];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(gw.trxc_fd, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr *)&src, &slen);
    if (n <= 0) return;
    buf[n] = '\0';

    if (!gw.trxc_remote_known || gw.trxc_remote.sin_port != src.sin_port) {
        gw.trxc_remote = src;
        gw.trxc_remote_known = true;
        GATE_LOG("TRXC remote %s:%u",
                 inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }
    trxc_handle(buf);
}

/* ---------- TRXD ---------- */

/*
 * Wire format from bridge.py (post GMSK modulation):
 *   bytes 0..5   : DL TRXDv0 header (TN, FN×4 BE, ATT)
 *   bytes 6..   : 148 syms × 4 sps = 592 complex samples
 *                  encoded as int16 LE I,Q,I,Q,...   (=2368 bytes)
 *
 * Total burst size: 6 + 2368 = 2374 bytes.
 *
 * Re-package to the format calypso_trx_rx_burst() still expects:
 *   bytes 0..7  : TN(1) FN(4 BE) RSSI(1) TOA(2 BE)   (RSSI/TOA = 0 here)
 *   bytes 8..   : N int16 LE samples (I and Q interleaved)
 */
#define GMSK_SPS         4
#define BURST_NSAMPLES   (148 * GMSK_SPS)        /* 592 complex samples */
#define BURST_IQ_BYTES   (BURST_NSAMPLES * 2 * 2) /* 2368 */
#define TRXD_PKT_BYTES   (6 + BURST_IQ_BYTES)    /* 2374 */

static void trxd_cb(void *opaque)
{
    uint8_t buf[4096];
    struct sockaddr_in src;
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(gw.trxd_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
    if (n < 6 + 2) return;  /* need at least header + 1 sample */

    if (!gw.trxd_remote_known || gw.trxd_remote.sin_port != src.sin_port) {
        gw.trxd_remote = src;
        gw.trxd_remote_known = true;
        GATE_LOG("TRXD remote %s:%u",
                 inet_ntoa(src.sin_addr), ntohs(src.sin_port));
    }

    /* Body: int16 LE samples after the 6-byte header.
     * nbytes_iq must be even; we count individual int16 (I and Q both). */
    int nbytes_iq = (int)n - 6;
    if (nbytes_iq < 0) nbytes_iq = 0;
    int nint16 = nbytes_iq / 2;
    if (nint16 > (int)(sizeof(buf) / 2)) nint16 = sizeof(buf) / 2;

    {
        uint8_t tn = buf[0] & 0x07;
        uint32_t fn_ = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|
                       ((uint32_t)buf[3]<<8)|buf[4];
        static int dbg = 0;
        if (dbg < 5) {
            int16_t s0 = (int16_t)(buf[6] | (buf[7]<<8));
            int16_t s1 = (int16_t)(buf[8] | (buf[9]<<8));
            GATE_LOG("TRXD n=%zd tn=%u fn=%u att=%u nint16=%d first=[%d,%d]",
                     n, tn, fn_, buf[5], nint16, s0, s1);
            dbg++;
        }
    }

    /* Hand the int16 I/Q stream straight to the BSP DMA module — no more
     * legacy header repackage, no NDB poking. The BSP either DMAs into
     * the configured DARAM target or runs in DISCOVERY mode. */
    uint8_t  tn = buf[0] & 0x07;
    uint32_t fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                  ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];
    calypso_bsp_rx_burst(tn, fn, (const int16_t *)(buf + 6), nint16);
}

/* ---------- init ---------- */

void sercomm_gate_init(int base_port)
{
    if (base_port <= 0) base_port = 6700;
    gw.base_port = base_port;

    int clk_port  = base_port + 0;
    int trxc_port = base_port + 1;
    int trxd_port = base_port + 2;

    gw.clk_fd = udp_bind(clk_port);
    if (gw.clk_fd < 0) {
        GATE_LOG("CLK bind %d failed: %s", clk_port, strerror(errno));
    } else {
        qemu_set_fd_handler(gw.clk_fd, clk_cb, NULL, NULL);
        GATE_LOG("CLK  listening UDP %d", clk_port);
    }

    gw.trxc_fd = udp_bind(trxc_port);
    if (gw.trxc_fd < 0) {
        GATE_LOG("TRXC bind %d failed: %s", trxc_port, strerror(errno));
    } else {
        qemu_set_fd_handler(gw.trxc_fd, trxc_cb, NULL, NULL);
        GATE_LOG("TRXC listening UDP %d", trxc_port);
    }

    gw.trxd_fd = udp_bind(trxd_port);
    if (gw.trxd_fd < 0) {
        GATE_LOG("TRXD bind %d failed: %s", trxd_port, strerror(errno));
    } else {
        qemu_set_fd_handler(gw.trxd_fd, trxd_cb, NULL, NULL);
        GATE_LOG("TRXD listening UDP %d", trxd_port);
    }
}
