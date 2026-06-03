/*
 * l1ctl_sock.c — L1CTL unix socket server (legacy QEMU-internal path)
 *
 * État runtime actuel (2026-05-25) : ce socket est INACTIF dans le run
 * orchestré par scripts/run.sh. run.sh:458 override l'env L1CTL_SOCK vers
 * /tmp/qemu_l1ctl_disabled pour le child QEMU, donc ce module crée son
 * socket à une adresse-poubelle et personne ne s'y connecte. Le VRAI
 * socket /tmp/osmocom_l2 que le mobile osmocom-bb utilise est créé par
 * osmocon (-m romload -s /tmp/osmocom_l2), pas par QEMU.
 *
 * Le path historique « Replaces the Python bridge » reste possible si on
 * lance QEMU sans override env — utile pour des tests sans osmocon, mais
 * pas le mode de fonctionnement principal. Voir doc/L1CTL_SOCK_FLOW.md
 * et le commentaire à run.sh:458.
 *
 * Quand actif : provides a unix socket at /tmp/osmocom_l2 that speaks
 * L1CTL (length-prefixed messages) to OsmocomBB mobile.
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
#define L1CTL_SOCK_PATH    "/tmp/osmocom_l2"

#define L1CTL_LOG(fmt, ...) \
    fprintf(stderr, "[l1ctl-sock] " fmt "\n", ##__VA_ARGS__)

/* Nom lisible des types L1CTL (l1ctl_proto.h) — diagnostic pur pour suivre la
 * conversation firmware↔mobile à l'œil. NB : ce mobile cause par osmocon/hdlc
 * (serial), pas par ce socket unix ; ce log ne voit que le sens firmware→mobile
 * via sercomm. Le vrai flux mobile↔firmware se lit dans osmocon.log (hdlc). */
static inline const char *l1ctl_tname(uint8_t t)
{
    switch (t) {
    case 0x01: return "FBSB_REQ";       case 0x02: return "FBSB_CONF";
    case 0x03: return "DATA_IND";       case 0x04: return "RACH_REQ";
    case 0x05: return "DM_EST_REQ";     case 0x06: return "DATA_REQ";
    case 0x07: return "RESET_IND";      case 0x08: return "PM_REQ";
    case 0x09: return "PM_CONF";        case 0x0c: return "RACH_CONF";
    case 0x0d: return "RESET_REQ";      case 0x0e: return "RESET_CONF";
    case 0x0f: return "DATA_CONF";      case 0x10: return "CCCH_MODE_REQ";
    case 0x11: return "CCCH_MODE_CONF"; case 0x12: return "DM_REL_REQ";
    case 0x13: return "PARAM_REQ";      default:   return "?";
    }
}

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
    if (s->cli_fd < 0 || len <= 0 || len > UINT16_MAX) return;

    uint8_t hdr[2] = { (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    struct iovec iov[2] = {
        { .iov_base = hdr,                  .iov_len = sizeof(hdr) },
        { .iov_base = (void *)payload,      .iov_len = (size_t)len },
    };
    struct msghdr msg = { .msg_iov = iov, .msg_iovlen = 2 };

    int total = (int)sizeof(hdr) + len;
    ssize_t sent = sendmsg(s->cli_fd, &msg, MSG_NOSIGNAL);
    if (sent != total) {
        L1CTL_LOG("client send error (%zd/%d), closing", sent, total);
        close(s->cli_fd);
        s->cli_fd = -1;
    }
}

/* Hop 5 : injection directe DL SI -> mobile en L1CTL DATA_IND (court-circuite
 * a_cd->ARM->UART qui perd des octets). Appele par le shunt GSMTAP listener. */
void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn)
{
    if (g_l1ctl.cli_fd < 0 || !l2 || l2len <= 0) return;
    if (l2len > 23) l2len = 23;
    uint8_t pl[16 + 23];
    memset(pl, 0, sizeof(pl));
    pl[0] = 0x03;                                  /* L1CTL_DATA_IND */
    pl[4] = 0x80;                                  /* chan_nr = BCCH */
    pl[6] = (uint8_t)(514 >> 8); pl[7] = (uint8_t)(514 & 0xFF);  /* band_arfcn 514 */
    pl[8]=(uint8_t)(fn>>24); pl[9]=(uint8_t)(fn>>16);
    pl[10]=(uint8_t)(fn>>8);  pl[11]=(uint8_t)fn;  /* frame_nr (BE) */
    pl[12] = 40;                                   /* rx_level */
    pl[13] = 30;                                   /* snr */
    /* pl[14]=num_biterr=0, pl[15]=fire_crc=0 (CRC OK) */
    memcpy(pl + 16, l2, l2len);
    l1ctl_send_to_mobile(&g_l1ctl, pl, 16 + l2len);
    L1CTL_LOG("INJECT DL DATA_IND BCCH fn=%u l2len=%d -> mobile", fn, l2len);
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
        /* ===== GATES de déblocage (oracle FORCE_TOA, gate-par-gate) =====
         * Le mobile reçoit par CE socket (mobile.cfg: layer2-socket
         * /tmp/osmocom_l2). Deux gates bridgent les trous du demod DSP, pour
         * prouver que tout l'aval (camp/SI/IMM-ASS) marche quand le DSP fournit
         * son résultat. Purement oracle — à retirer quand le DSP demod marche.
         *   CALYPSO_FORCE_FBSB=1 : bridge blocker #1 (Channel sync error) =
         *                          FBSB_CONF(0x02) result@[18] → 0=SUCCESS.
         *   CALYPSO_FORCE_AGCH=1 : bridge blocker #2 (pas de sysinfo) sur les
         *                          DATA_IND(0x03) : BCCH(chan 0x80) rote le type
         *                          SI ; AGCH/PCH(chan 0x90) injecte IMM ASS.
         *                          Port exact du GDB mutate_agch.
         * Layout payload : l1ctl_hdr(4) + l1ctl_info_dl(12) + corps ;
         * → FBSB result @18 ; DATA_IND chan_nr @4, L3 @16. */
        static int g_fbsb = -1, g_agch = -1;
        if (g_fbsb < 0) {
            const char *a = getenv("CALYPSO_FORCE_FBSB");
            const char *b = getenv("CALYPSO_FORCE_AGCH");
            g_fbsb = (a && *a == '1') ? 1 : 0;
            g_agch = (b && *b == '1') ? 1 : 0;
        }
        if (g_fbsb && payload[0] == 0x02 && plen >= 19 && payload[18] != 0) {
            L1CTL_LOG("GATE-FBSB #1: FBSB_CONF result 0x%02x → 0", payload[18]);
            payload[18] = 0;
        }
        if (g_agch && payload[0] == 0x03 && plen >= 16 + 3) {
            uint8_t chan_nr = payload[4];
            uint8_t *l3 = &payload[16];
            if (chan_nr == 0x80) {
                static const uint8_t si[4] = { 0x19, 0x1a, 0x1b, 0x1c };
                static int r = 0;
                l3[2] = si[r]; r = (r + 1) & 3;
                L1CTL_LOG("GATE-AGCH #2 bcch: SI type → 0x%02x", l3[2]);
            } else if (chan_nr == 0x90 && plen >= 16 + 23) {
                static const uint8_t imm[23] = {
                    0x2d, 0x06, 0x3f, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b, 0x2b,
                    0x2b, 0x2b, 0x2b };
                memcpy(l3, imm, sizeof(imm));
                L1CTL_LOG("GATE-AGCH #2 pch: IMM ASSIGNMENT injecté");
            }
        }
        L1CTL_LOG("TX→mobile: dlci=%d len=%d type=0x%02x %s", dlci, plen, payload[0],
                  l1ctl_tname(payload[0]));
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
    ssize_t n = recv(s->cli_fd, tmp, sizeof(tmp), MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return;  /* no data available yet */
        L1CTL_LOG("client recv error: %s", strerror(errno));
        qemu_set_fd_handler(s->cli_fd, NULL, NULL, NULL);
        close(s->cli_fd);
        s->cli_fd = -1;
        s->lp_len = 0;
        return;
    }
    if (n == 0) {
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
            L1CTL_LOG("RX←mobile: len=%d type=0x%02x %s → sercomm %d bytes",
                      msglen, payload[0], l1ctl_tname(payload[0]), flen);
            /* Hex dump of sercomm frame being injected */
            {
                fprintf(stderr, "[l1ctl-sock] INJECT %d bytes:", flen);
                for (int j = 0; j < flen && j < 32; j++)
                    fprintf(stderr, " %02x", frame[j]);
                if (flen > 32) fprintf(stderr, " ...");
                fprintf(stderr, "\n");
            }
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

/* ---- Manual poll (called from TDMA tick) ---- */

void l1ctl_sock_poll(void)
{
    L1CTLSock *s = &g_l1ctl;

    /* Try to accept a pending client */
    if (s->srv_fd >= 0 && s->cli_fd < 0) {
        int fd = accept(s->srv_fd, NULL, NULL);
        if (fd >= 0) {
            int flags = fcntl(fd, F_GETFL);
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            s->cli_fd = fd;
            s->lp_len = 0;
            s->sc_state = SC_IDLE;
            s->sc_len = 0;
            qemu_set_fd_handler(fd, l1ctl_client_readable, NULL, s);
            L1CTL_LOG("client connected via poll (fd=%d)", fd);
        }
    }

    /* Try to read from connected client */
    if (s->cli_fd >= 0) {
        l1ctl_client_readable(s);
    }
}

bool l1ctl_client_active(void)
{
    return g_l1ctl.cli_fd >= 0;
}
