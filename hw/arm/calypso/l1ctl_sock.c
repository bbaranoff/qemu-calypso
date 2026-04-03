/*
 * l1ctl_sock.c — L1CTL unix socket server for Calypso QEMU
 *
 * Replaces the Python bridge: provides a unix socket at /tmp/osmocom_l2
 * that speaks L1CTL (length-prefixed messages) to OsmocomBB mobile.
 *
 * Internally translates between:
 *   - sercomm framing (FLAG/ESCAPE/DLCI) on the firmware UART side
 *   - L1CTL length-prefix on the mobile socket side
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "hw/arm/calypso/calypso_uart.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>

/* Sercomm constants */
#define SERCOMM_FLAG       0x7E
#define SERCOMM_ESCAPE     0x7D
#define SERCOMM_ESCAPE_XOR 0x20
#define SERCOMM_DLCI_L1CTL 5

/* L1CTL socket path */
#define L1CTL_SOCK_PATH    "/tmp/osmocom_l2_tmp"

#define L1CTL_LOG(fmt, ...) \
    fprintf(stderr, "[l1ctl-sock] " fmt "\n", ##__VA_ARGS__)

/* ---- Sercomm TX parser (firmware → mobile) ---- */

typedef enum {
    SC_IDLE,      /* waiting for FLAG */
    SC_IN_FRAME,  /* collecting frame bytes */
    SC_ESCAPE,    /* next byte is escaped */
} SercommState;

typedef struct L1CTLSock {
    /* Server socket */
    int srv_fd;

    /* Client connection */
    int cli_fd;

    /* Sercomm TX parser (firmware UART output → mobile) */
    SercommState sc_state;
    uint8_t  sc_buf[512];
    int      sc_len;

    /* L1CTL RX parser (mobile → firmware UART input) */
    uint8_t  lp_buf[4096];  /* length-prefix accumulator */
    int      lp_len;

    /* Reference to UART modem for RX injection */
    CalypsoUARTState *uart;
} L1CTLSock;

static L1CTLSock g_l1ctl;

/* ---- Sercomm helpers ---- */

static int sercomm_wrap(uint8_t dlci, const uint8_t *payload, int plen,
                        uint8_t *out, int out_size)
{
    int pos = 0;
    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;

    /* DLCI + CTRL */
    uint8_t hdr[2] = { dlci, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == SERCOMM_FLAG || hdr[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = hdr[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = hdr[i];
        }
    }

    /* Payload */
    for (int i = 0; i < plen; i++) {
        if (payload[i] == SERCOMM_FLAG || payload[i] == SERCOMM_ESCAPE) {
            if (pos + 2 > out_size) return -1;
            out[pos++] = SERCOMM_ESCAPE;
            out[pos++] = payload[i] ^ SERCOMM_ESCAPE_XOR;
        } else {
            if (pos + 1 > out_size) return -1;
            out[pos++] = payload[i];
        }
    }

    if (pos >= out_size) return -1;
    out[pos++] = SERCOMM_FLAG;
    return pos;
}

/* ---- Send L1CTL message to mobile (length-prefix) ---- */

static void l1ctl_send_to_mobile(L1CTLSock *s, const uint8_t *payload, int len)
{
    if (s->cli_fd < 0 || len <= 0 || len > 512) return;

    /* Single atomic write: length header + payload */
    uint8_t buf[514];
    buf[0] = (len >> 8) & 0xFF;
    buf[1] = len & 0xFF;
    memcpy(&buf[2], payload, len);

    int total = 2 + len;
    int sent = send(s->cli_fd, buf, total, MSG_NOSIGNAL);
    if (sent != total) {
        L1CTL_LOG("client send error (%d/%d), closing", sent, total);
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

/* ---- Process a complete sercomm frame from firmware TX ---- */

static void sercomm_frame_complete(L1CTLSock *s)
{
    if (s->sc_len < 2) return;  /* need at least DLCI + CTRL */

    uint8_t dlci = s->sc_buf[0];
    /* uint8_t ctrl = s->sc_buf[1]; */
    uint8_t *payload = &s->sc_buf[2];
    int plen = s->sc_len - 2;

    if (dlci == SERCOMM_DLCI_L1CTL && plen > 0) {
        L1CTL_LOG("TX→mobile: dlci=%d len=%d type=0x%02x", dlci, plen, payload[0]);
        l1ctl_send_to_mobile(s, payload, plen);
    }
    /* Ignore other DLCIs (debug console, loader, etc.) */
}

/* ---- Feed firmware UART TX bytes into sercomm parser ---- */

void l1ctl_sock_uart_tx_byte(uint8_t byte)
{
    L1CTLSock *s = &g_l1ctl;

    switch (s->sc_state) {
    case SC_IDLE:
        if (byte == SERCOMM_FLAG) {
            s->sc_state = SC_IN_FRAME;
            s->sc_len = 0;
        }
        break;

    case SC_IN_FRAME:
        if (byte == SERCOMM_FLAG) {
            if (s->sc_len > 0) {
                sercomm_frame_complete(s);
            }
            /* Stay in IN_FRAME for next frame */
            s->sc_len = 0;
        } else if (byte == SERCOMM_ESCAPE) {
            s->sc_state = SC_ESCAPE;
        } else {
            if (s->sc_len < (int)sizeof(s->sc_buf)) {
                s->sc_buf[s->sc_len++] = byte;
            }
        }
        break;

    case SC_ESCAPE:
        if (s->sc_len < (int)sizeof(s->sc_buf)) {
            s->sc_buf[s->sc_len++] = byte ^ SERCOMM_ESCAPE_XOR;
        }
        s->sc_state = SC_IN_FRAME;
        break;
    }
}

/* ---- Receive L1CTL from mobile, inject into firmware UART RX ---- */

static void l1ctl_client_readable(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    uint8_t tmp[4096];
    ssize_t n = recv(s->cli_fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
        L1CTL_LOG("client disconnected");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }

    /* Accumulate in length-prefix buffer */
    if (s->lp_len + (int)n > (int)sizeof(s->lp_buf)) {
        s->lp_len = 0;  /* overflow, reset */
    }
    memcpy(&s->lp_buf[s->lp_len], tmp, n);
    s->lp_len += (int)n;

    /* Parse complete L1CTL messages */
    while (s->lp_len >= 2) {
        int msglen = (s->lp_buf[0] << 8) | s->lp_buf[1];
        if (s->lp_len < 2 + msglen) break;  /* incomplete */

        uint8_t *payload = &s->lp_buf[2];

        /* Wrap in sercomm and inject into UART RX */
        uint8_t frame[1024];
        int flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen,
                                frame, sizeof(frame));
        if (flen > 0 && s->uart) {
            L1CTL_LOG("RX←mobile: len=%d type=0x%02x → sercomm %d bytes",
                      msglen, payload[0], flen);
            calypso_uart_receive(s->uart, frame, flen);
        }

        /* Consume from buffer */
        int consumed = 2 + msglen;
        memmove(s->lp_buf, &s->lp_buf[consumed], s->lp_len - consumed);
        s->lp_len -= consumed;
    }
}

/* ---- Accept new client connection ---- */

static void l1ctl_accept_cb(void *opaque)
{
    L1CTLSock *s = (L1CTLSock *)opaque;

    int fd = accept(s->srv_fd, NULL, NULL);
    if (fd < 0) return;

    /* Only one client at a time */
    if (s->cli_fd >= 0) {
        L1CTL_LOG("replacing existing client");
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
    }

    /* Set non-blocking */
    int flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    s->cli_fd = fd;
    s->lp_len = 0;
    s->sc_state = SC_IDLE;
    s->sc_len = 0;

    qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
    L1CTL_LOG("client connected (fd=%d)", fd);
}

/* ---- Init ---- */

void l1ctl_sock_init(CalypsoUARTState *uart, const char *path)
{
    L1CTLSock *s = &g_l1ctl;
    memset(s, 0, sizeof(*s));
    s->srv_fd = -1;
    s->cli_fd = -1;
    s->uart = uart;

    if (!path) path = L1CTL_SOCK_PATH;

    /* Remove stale socket */
    unlink(path);

    /* Create unix socket server */
    s->srv_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (s->srv_fd < 0) {
        L1CTL_LOG("ERROR: socket(): %s", strerror(errno));
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (bind(s->srv_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        L1CTL_LOG("ERROR: bind(%s): %s", path, strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    if (listen(s->srv_fd, 1) < 0) {
        L1CTL_LOG("ERROR: listen(): %s", strerror(errno));
        close(s->srv_fd);
        s->srv_fd = -1;
        return;
    }

    /* Set non-blocking */
    int flags = fcntl(s->srv_fd, F_GETFL);
    fcntl(s->srv_fd, F_SETFL, flags | O_NONBLOCK);

    qemu_set_fd_handler(s->srv_fd, l1ctl_accept_cb, NULL, s);
    L1CTL_LOG("listening on %s", path);
}
