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
#include "chardev/char-fe.h"
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

/* TRXC handling is NOT done by QEMU. bridge.py answers TRXC commands
 * locally (stub) on UDP 5701 — QEMU never sees them. The L1CTL/L23
 * path on PTY DLCI 5 is the only thing the modem UART carries. */

#define GATE_LOG(fmt, ...) \
    fprintf(stderr, "[gate] " fmt "\n", ##__VA_ARGS__)

/* UART pointer captured on the first sercomm_gate_feed() call. The TRXC
 * UDP callback uses it to push received sercomm-wrapped frames straight
 * into the firmware UART RX FIFO. */
static CalypsoUARTState *g_uart;

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
    fprintf(stderr,
            "[gate→fw-fifo] DLCI=%u CTRL=%02x payload=%d bytes (rx_count_before=%u)\n",
            len > 0 ? frame[0] : 0xff,
            len > 1 ? frame[1] : 0xff,
            len > 2 ? len - 2 : 0,
            (unsigned)s->rx_count);
    { uint8_t _b = SERCOMM_FLAG; calypso_uart_inject_raw(s, &_b, 1); }
    for (int i = 0; i < len; i++) {
        uint8_t c = frame[i];
        /* Standard sercomm: only escape FLAG and ESCAPE. Escaping
         * 0x00 was a bug — the firmware sercomm parser doesn't
         * expect it and would drop the frame. */
        if (c == SERCOMM_FLAG || c == SERCOMM_ESCAPE) {
            { uint8_t _e = SERCOMM_ESCAPE; calypso_uart_inject_raw(s, &_e, 1); }
            { uint8_t _x = c ^ SERCOMM_XOR; calypso_uart_inject_raw(s, &_x, 1); }
        } else {
            calypso_uart_inject_raw(s, &c, 1);
        }
    }
    { uint8_t _b = SERCOMM_FLAG; calypso_uart_inject_raw(s, &_b, 1); }
}

#define SERCOMM_DLCI_TRXC 4

/* Wrap a payload in sercomm DLCI 4 (TRXC) and send it back via the
 * chardev TX (→ PTY → bridge.py → osmo-bts-trx 5701). */
static void gate_send_trxc_rsp(CalypsoUARTState *s,
                                const uint8_t *payload, int plen)
{
    if (!s) return;
    uint8_t frame[1024];
    int pos = 0;
    frame[pos++] = SERCOMM_FLAG;
    uint8_t hdr[2] = { SERCOMM_DLCI_TRXC, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == SERCOMM_FLAG || hdr[i] == SERCOMM_ESCAPE) {
            frame[pos++] = SERCOMM_ESCAPE;
            frame[pos++] = hdr[i] ^ SERCOMM_XOR;
        } else {
            frame[pos++] = hdr[i];
        }
    }
    for (int i = 0; i < plen && pos + 2 < (int)sizeof(frame); i++) {
        uint8_t c = payload[i];
        if (c == SERCOMM_FLAG || c == SERCOMM_ESCAPE) {
            frame[pos++] = SERCOMM_ESCAPE;
            frame[pos++] = c ^ SERCOMM_XOR;
        } else {
            frame[pos++] = c;
        }
    }
    frame[pos++] = SERCOMM_FLAG;
    qemu_chr_fe_write_all(&s->chr, frame, pos);
    fprintf(stderr, "[gate-trxc] TX→bridge %d bytes (sercomm framed=%d)\n",
            plen, pos);
}

/* Parse a TRXC CMD string and produce a RSP string.
 * Mirrors bridge.py::trxc_response. Returns response length, or 0 if
 * the command is not a CMD (silently ignored). */
static int gate_trxc_handle(const uint8_t *cmd_buf, int cmd_len,
                             char *rsp, int rsp_size)
{
    /* Strip trailing \0 and find verb/args */
    int n = cmd_len;
    while (n > 0 && (cmd_buf[n-1] == 0 || cmd_buf[n-1] == "\n"[0])) n--;
    if (n < 4) return 0;
    if (memcmp(cmd_buf, "CMD ", 4) != 0) return 0;

    char tmp[512];
    int  tn = n - 4 < (int)sizeof(tmp) - 1 ? n - 4 : (int)sizeof(tmp) - 1;
    memcpy(tmp, cmd_buf + 4, tn);
    tmp[tn] = 0;

    /* Split verb and args */
    char *verb = tmp;
    char *args = strchr(tmp, " "[0]);
    if (args) { *args = 0; args++; }
    else args = (char *)"";

    fprintf(stderr, "[gate-trxc] RX←bridge CMD %s args=%s\n", verb, args);

    int rl;
    if (strcmp(verb, "POWERON") == 0)
        rl = snprintf(rsp, rsp_size, "RSP POWERON 0");
    else if (strcmp(verb, "POWEROFF") == 0)
        rl = snprintf(rsp, rsp_size, "RSP POWEROFF 0");
    else if (strcmp(verb, "SETFORMAT") == 0)
        rl = snprintf(rsp, rsp_size, "RSP SETFORMAT 0 %s", args[0] ? args : "0");
    else if (strcmp(verb, "NOMTXPOWER") == 0)
        rl = snprintf(rsp, rsp_size, "RSP NOMTXPOWER 0 50");
    else if (strcmp(verb, "MEASURE") == 0)
        rl = snprintf(rsp, rsp_size, "RSP MEASURE 0 %s -60", args[0] ? args : "0");
    else if (args[0])
        rl = snprintf(rsp, rsp_size, "RSP %s 0 %s", verb, args);
    else
        rl = snprintf(rsp, rsp_size, "RSP %s 0", verb);

    if (rl > 0 && rl < rsp_size) rsp[rl++] = 0;  /* trailing NUL like bridge */
    return rl;
}

void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size)
{
    if (!g_uart) g_uart = s;
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
                    /* DLCI 5 = L1CTL from mobile (via bridge).
                     * Trace, then push to firmware FIFO so the real
                     * sercomm parser dispatches it. All other DLCIs
                     * (console, debug, …) go straight to FIFO. */
                    if (sc_buf[0] == SERCOMM_DLCI_TRXC && sc_len >= 2) {
                        char rsp[512];
                        int rl = gate_trxc_handle(sc_buf + 2, sc_len - 2,
                                                   rsp, sizeof(rsp));
                        if (rl > 0)
                            gate_send_trxc_rsp(s, (uint8_t *)rsp, rl);
                        sc_len = 0;
                        break;
                    }
                    if (sc_buf[0] == 5) {
                        int plen = sc_len - 2;
                        uint8_t mt = plen > 0 ? sc_buf[2] : 0;
                        fprintf(stderr,
                                "[PTY-L1CTL] <<<RX %d bytes (mobile→fw) mt=0x%02x:",
                                plen, mt);
                        for (int j = 0; j < plen && j < 32; j++)
                            fprintf(stderr, " %02x", sc_buf[2 + j]);
                        if (plen > 32) fprintf(stderr, " ...");
                        fprintf(stderr, "\n");

                    }
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

/* ---------- TRXC ----------
 * Removed: bridge.py answers TRXC commands locally on UDP 5701 (stub).
 * QEMU never sees TRXC traffic. */

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

static void __attribute__((unused)) trxd_cb(void *opaque)
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

    /* Hand the burst to calypso_trx — the chef. It owns TDMA state and
     * decides whether to forward to BSP DMA or fast-path L1CTL_DATA_IND. */
    uint8_t  tn = buf[0] & 0x07;
    uint32_t fn = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16) |
                  ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];
    calypso_trx_on_dl_burst(tn, fn, (const int16_t *)(buf + 6), nint16);
}

/* ---------- UL burst send (BSP → bridge → BTS) ---------- */

void sercomm_gate_send_ul_burst(uint8_t tn, uint32_t fn,
                                const uint8_t *bits148)
{
    if (gw.trxd_fd < 0 || !bits148) return;

    uint8_t pkt[8 + 148];
    pkt[0] = tn & 0x07;
    pkt[1] = (fn >> 24) & 0xff;
    pkt[2] = (fn >> 16) & 0xff;
    pkt[3] = (fn >>  8) & 0xff;
    pkt[4] =  fn        & 0xff;
    pkt[5] = (uint8_t)(-60);
    pkt[6] = 0; pkt[7] = 0;
    memcpy(&pkt[8], bits148, 148);

    struct sockaddr_in dst = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port        = htons(6802),
    };
    ssize_t r = sendto(gw.trxd_fd, pkt, sizeof(pkt), 0,
                       (struct sockaddr *)&dst, sizeof(dst));
    static unsigned cnt;
    if (r < 0 || (cnt++ % 100) == 0) {
        GATE_LOG("UL→bridge tn=%u fn=%u r=%zd (#%u)", tn, fn, r, cnt);
    }
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

    /* TRXC: not handled by QEMU. bridge.py answers locally on UDP 5701. */
    gw.trxc_fd = -1;
    (void)trxc_port;
    GATE_LOG("TRXC: bridge.py local stub on :5701, QEMU not involved");

    /* TRXD UDP socket moved to calypso_bsp.c — BSP owns the transport
     * for both DL recv and UL send (symmetric, single-port). */
    gw.trxd_fd = -1;
    (void)trxd_port;
    GATE_LOG("TRXD: now owned by calypso_bsp.c on UDP 6702");
}
