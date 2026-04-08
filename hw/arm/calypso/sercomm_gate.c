/*
 * sercomm_gate.c — Sercomm DLCI router (PTY) + CLK UDP listener
 *
 * Two separate roles, matching the current QEMU split:
 *
 *   1. PTY (UART modem) — sercomm HDLC stream from host (mobile/ccch_scan).
 *      DLCIs are re-wrapped and pushed to the UART RX FIFO so the firmware's
 *      sercomm driver parses them via the real code path. L1CTL = DLCI 5,
 *      TRXC = DLCI 4 (intercepted here, stub responses wrapped back out).
 *      No DLCI on the PTY ever carries radio bursts.
 *
 *   2. UDP CLK listener — just drains "IND CLOCK <fn>" on the baseband
 *      side and logs it. calypso_trx owns its own FN counter; the CLK
 *      packets are purely informational here.
 *
 *      TRXC traffic is stubbed locally by bridge.py on UDP 5701 — QEMU
 *      never sees TRXC on UDP.
 *
 *      TRXD (burst) transport is owned by calypso_bsp.c: BSP binds
 *      127.0.0.1:6802 for DL recv and sends UL to 127.0.0.1:6702.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/error-report.h"
#include "chardev/char-fe.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#include "hw/arm/calypso/calypso_uart.h"
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
 * 2. UDP CLK listener — informational only
 * ============================================================
 *
 * TRXC is stubbed by bridge.py on UDP 5701; QEMU never sees it.
 * TRXD (bursts) is owned by calypso_bsp.c: bind 127.0.0.1:6802 for DL
 * recv, sendto 127.0.0.1:6702 for UL. See calypso_bsp.c for details.
 */

static int g_clk_fd = -1;

static int udp_bind_loopback(int port)
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

static void clk_cb(void *opaque)
{
    (void)opaque;
    char buf[128];
    ssize_t n = recv(g_clk_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;
    buf[n] = '\0';
    /* Just log occasionally — calypso_trx owns its own FN counter. */
    static unsigned cnt;
    if ((cnt++ % 256) == 0) {
        GATE_LOG("CLK %s", buf);
    }
}

/* ---------- init ---------- */

void sercomm_gate_init(int base_port)
{
    if (base_port <= 0) base_port = 6700;
    int clk_port = base_port + 0;

    g_clk_fd = udp_bind_loopback(clk_port);
    if (g_clk_fd < 0) {
        GATE_LOG("CLK bind %d failed: %s", clk_port, strerror(errno));
    } else {
        qemu_set_fd_handler(g_clk_fd, clk_cb, NULL, NULL);
        GATE_LOG("CLK  listening UDP %d", clk_port);
    }

    GATE_LOG("TRXC: bridge.py local stub on :5701, QEMU not involved");
    GATE_LOG("TRXD: owned by calypso_bsp.c "
             "(bind 127.0.0.1:6802 DL / sendto 127.0.0.1:6702 UL)");
}
