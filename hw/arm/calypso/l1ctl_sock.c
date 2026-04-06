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
#include "sysemu/runstate.h"
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

    /* TX buffer: firmware messages before client connects */
    uint8_t  tx_pending[8192];
    int      tx_pending_len;

    /* Reference to UART modem for RX injection */
    CalypsoUARTState *uart;

    /* Client/server: client read handler deferred until firmware speaks first */
    bool cli_rx_enabled;

    /* Burst mode gating: needs BOTH fbsb_requested AND TPU_CTRL_EN */
    bool burst_mode;
    bool fbsb_requested;  /* set on FBSB_REQ, cleared on RESET_REQ/disconnect */
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
    if (s->cli_fd < 0) return;

    uint8_t hdr[2];
    hdr[0] = (len >> 8) & 0xFF;
    hdr[1] = len & 0xFF;

    /* Best-effort send */
    if (send(s->cli_fd, hdr, 2, MSG_NOSIGNAL) != 2 ||
        send(s->cli_fd, payload, len, MSG_NOSIGNAL) != len) {
        L1CTL_LOG("client send error, closing");
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

static void l1ctl_client_readable(void *opaque);

/* ---- Process a complete sercomm frame from firmware TX ---- */

static void sercomm_frame_complete(L1CTLSock *s)
{
    if (s->sc_len < 2) return;  /* need at least DLCI + CTRL */

    uint8_t dlci = s->sc_buf[0];
    /* uint8_t ctrl = s->sc_buf[1]; */
    uint8_t *payload = &s->sc_buf[2];
    int plen = s->sc_len - 2;

    if (dlci == SERCOMM_DLCI_L1CTL && plen > 0) {
        /* Firmware spoke first — enable client→firmware path */
        if (!s->cli_rx_enabled && s->cli_fd >= 0) {
            /* Drain stale data buffered before firmware was ready */
            uint8_t drain[4096];
            ssize_t drained = recv(s->cli_fd, drain, sizeof(drain), MSG_DONTWAIT);
            if (drained > 0)
                L1CTL_LOG("drained %zd stale bytes from mobile", drained);
            s->lp_len = 0;
            s->cli_rx_enabled = true;
            qemu_set_fd_handler(s->cli_fd, l1ctl_client_readable, NULL, s);
            L1CTL_LOG("firmware ready — accepting mobile input");
        }
        /* Parse L1CTL header: msg_type(1) flags(1) padding(2) */
        {
            static const char *l1ctl_names[] = {
                [0x00]="NONE", [0x01]="FBSB_REQ", [0x02]="FBSB_CONF",
                [0x03]="DATA_IND", [0x04]="RACH_REQ", [0x05]="RACH_CONF",
                [0x06]="DATA_REQ", [0x07]="RESET_IND", [0x08]="PM_REQ",
                [0x09]="PM_CONF", [0x0a]="ECHO_REQ", [0x0b]="ECHO_CONF",
                [0x0c]="DATA_CONF", [0x0d]="RESET_REQ", [0x0e]="RESET_CONF",
                [0x0f]="DATA_ABI", [0x10]="SIM_REQ", [0x11]="SIM_CONF",
                [0x12]="TCH_MODE_REQ", [0x13]="TCH_MODE_CONF",
                [0x14]="NEIGH_PM_REQ", [0x15]="NEIGH_PM_IND",
                [0x16]="TRAFFIC_REQ", [0x17]="TRAFFIC_CONF", [0x18]="TRAFFIC_IND",
            };
            uint8_t mt = payload[0];
            const char *name = (mt < 0x19) ? l1ctl_names[mt] : "UNKNOWN";
            L1CTL_LOG("TX→mobile: %s (0x%02x) len=%d", name, mt, plen);
            {
                char hex[3*64+8] = {0};
                int n = plen < 64 ? plen : 64;
                for (int i = 0; i < n; i++)
                    snprintf(hex + i*3, 4, "%02x ", payload[i]);
                L1CTL_LOG("  RAW [%d]: %s%s", plen, hex, plen > 64 ? "..." : "");
            }

            /* Decode specific messages */
            if (mt == 0x09 && plen >= 8) { /* PM_CONF */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                int16_t pm = (int16_t)(payload[6] | (payload[7] << 8));
                L1CTL_LOG("  PM_CONF: arfcn=%u pm=%d", arfcn & 0x3FF, pm);
            }
            if (mt == 0x02 && plen >= 20) { /* FBSB_CONF */
                /* layout: hdr(4) + l1ctl_info_dl(12) + l1ctl_fbsb_conf(4) */
                uint16_t arfcn = payload[6] | (payload[7] << 8);
                int8_t   snr   = (int8_t)payload[13];
                int16_t  ferr  = (int16_t)(payload[16] | (payload[17] << 8));
                uint8_t  result = payload[18];
                uint8_t  bsic   = payload[19];
                L1CTL_LOG("  FBSB_CONF: arfcn=%u snr=%d ferr=%d result=%u bsic=%u",
                          arfcn & 0x3FFF, snr, ferr, result, bsic);
            }
        }
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
        s->cli_rx_enabled = false;
        s->burst_mode = false;
        s->fbsb_requested = false;
        return;
    }

    /* Accumulate in length-prefix buffer */
    if (s->lp_len + (int)n > (int)sizeof(s->lp_buf)) {
        s->lp_len = 0;  /* overflow, reset */
    }
    memcpy(&s->lp_buf[s->lp_len], tmp, n);
    s->lp_len += (int)n;

    /* Parse ALL complete L1CTL messages in buffer */
    while (s->lp_len >= 2) {
        int msglen = (s->lp_buf[0] << 8) | s->lp_buf[1];
        if (s->lp_len < 2 + msglen)
            break;  /* incomplete message, wait for more data */

        uint8_t *payload = &s->lp_buf[2];

        /* Track L1CTL state for burst gating */
        if (msglen > 0) {
            if (payload[0] == 0x01) {  /* FBSB_REQ */
                s->fbsb_requested = true;
                L1CTL_LOG("FBSB_REQ → waiting for TPU_CTRL_EN");
            } else if (payload[0] == 0x0d) {  /* RESET_REQ */
                s->burst_mode = false;
                s->fbsb_requested = false;
                L1CTL_LOG("RESET_REQ → burst mode OFF");
            }
        }

        /* Wrap in sercomm and inject into UART RX */
        uint8_t frame[1024];
        int flen = sercomm_wrap(SERCOMM_DLCI_L1CTL, payload, msglen,
                                frame, sizeof(frame));
        if (flen > 0 && s->uart) {
            static const char *l1ctl_names[] = {
                [0x00]="NONE", [0x01]="FBSB_REQ", [0x02]="FBSB_CONF",
                [0x03]="DATA_IND", [0x04]="RACH_REQ", [0x05]="RACH_CONF",
                [0x06]="DATA_REQ", [0x07]="RESET_IND", [0x08]="PM_REQ",
                [0x09]="PM_CONF", [0x0d]="RESET_REQ", [0x0e]="RESET_CONF",
            };
            uint8_t mt = payload[0];
            const char *name = (mt < 0x0f) ? l1ctl_names[mt] : "UNKNOWN";
            if (!name) name = "UNKNOWN";
            L1CTL_LOG("RX←mobile: %s (0x%02x) len=%d → sercomm %d bytes",
                      name, mt, msglen, flen);

            if (mt == 0x01 && msglen >= 12) { /* FBSB_REQ */
                /* hdr(4) + band_arfcn(2) timeout(2) freq_err1(2) freq_err2(2) ... */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                uint16_t timeout = payload[6] | (payload[7] << 8);
                L1CTL_LOG("  FBSB_REQ: arfcn=%u band=%u timeout=%u",
                          arfcn & 0x3FF, (arfcn >> 10) & 0x3F, timeout);
            }
            if (mt == 0x08 && msglen >= 8) { /* PM_REQ */
                uint16_t arfcn = payload[4] | (payload[5] << 8);
                L1CTL_LOG("  PM_REQ: arfcn=%u", arfcn & 0x3FF);
            }

            calypso_uart_inject_raw(s->uart, frame, flen);
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
    /* Resume VM if it was started with -S (paused).
     * This ensures the firmware doesn't boot until a client is listening,
     * so the RESET_IND sent at boot reaches the mobile. */
    if (!runstate_is_running()) {
        s->cli_rx_enabled = false;
        L1CTL_LOG("client connected (fd=%d) — waiting for firmware boot", fd);
        vm_start();
    } else {
        /* VM already running (reconnect): enable reading immediately
         * so mobile can send RESET_REQ to the running firmware. */
        s->cli_rx_enabled = true;
        qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
        L1CTL_LOG("client reconnected (fd=%d) — firmware running", fd);
    }
}

/* Is the L1CTL client (mobile) connected and active? */
bool l1ctl_client_active(void)
{
    return g_l1ctl.cli_fd >= 0 && g_l1ctl.cli_rx_enabled;
}

/* Is the system in burst mode? */
bool l1ctl_burst_mode(void)
{
    return g_l1ctl.burst_mode;
}

/* Called by calypso_trx when firmware writes TPU_CTRL_EN.
 * Burst mode only activates if FBSB_REQ was received first. */
void l1ctl_set_burst_mode(bool on)
{
    if (on && g_l1ctl.fbsb_requested && !g_l1ctl.burst_mode) {
        g_l1ctl.burst_mode = true;
        L1CTL_LOG("burst mode ON (FBSB_REQ + TPU_CTRL_EN)");
    }
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
