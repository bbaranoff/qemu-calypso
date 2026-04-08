/*
 * Calypso BSP/RIF DMA module — implementation.
 *
 * On real hardware the BSP (Baseband Serial Port) is a synchronous serial
 * link that DMA-feeds I/Q samples coming from the IOTA RF frontend straight
 * into the C54x DSP DARAM. The DSP code (FB/SB/burst detection in PROM0)
 * reads them from a fixed DARAM buffer and posts results into the NDB.
 *
 * In QEMU we don't model the IOTA serial bus. Instead, sercomm_gate.c
 * receives pre-modulated I/Q from bridge.py over a UDP socket (TRXD)
 * and forwards them through this module, which writes them at the DSP
 * DARAM address the firmware expects.
 *
 * The destination DARAM address is identified at runtime by tracing
 * data reads issued by the DSP while PC ∈ [0x7730, 0x7990] (FB-det
 * handler in PROM0, see project_dsp_fb_det memory). It is then locked
 * via the env vars CALYPSO_BSP_DARAM_ADDR and CALYPSO_BSP_DARAM_LEN.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_trx.h"

#define BSP_LOG(fmt, ...) \
    do { fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

static struct {
    C54xState *dsp;
    uint16_t   daram_addr;       /* word address in DSP data space; 0 = unset */
    uint16_t   daram_len;        /* max words to copy per burst */
    uint16_t   bypass_bdlena;    /* 1 = ignore BDLENA gate (debug only) */
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
    uint64_t   bursts_dropped_no_window;
    /* UDP TRXD transport — BSP owns it (was in sercomm_gate).
     *   bind 127.0.0.1:6802 — DL bursts arrive here from bridge.py
     *   sendto 127.0.0.1:6702 — UL bursts go out to bridge.py
     * (fake_trx convention: BSP acts as transceiver side, base+100/101/102). */
    int        trxd_fd;
} bsp;

static uint16_t parse_uint_env(const char *name, uint16_t def)
{
    const char *v = getenv(name);
    if (!v || !*v) return def;
    return (uint16_t)strtoul(v, NULL, 0);
}

uint16_t calypso_bsp_get_daram_addr(void) { return bsp.daram_addr; }
uint16_t calypso_bsp_get_daram_len(void)  { return bsp.daram_len; }

/* UDP DL recv callback — registered with QEMU main loop.
 *
 * DL TRXDv0 wire (osmo-bts-trx → transceiver, forwarded by bridge.py):
 *   buf[0]      = (ver << 4) | tn
 *   buf[1..4]   = fn (BE u32)
 *   buf[5]      = att (attenuation dB)
 *   buf[6..153] = 148 hard ubits {0,1}
 * Total = 6 + 148 = 154 bytes.
 */
static void bsp_udp_dl_cb(void *opaque)
{
    (void)opaque;
    uint8_t buf[4096];
    ssize_t n = recv(bsp.trxd_fd, buf, sizeof(buf), 0);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
            BSP_LOG("DL recv err: %s", strerror(errno));
        return;
    }
    static unsigned cb_cnt = 0;
    cb_cnt++;
    if (cb_cnt <= 10 || (cb_cnt % 1000) == 0) {
        BSP_LOG("DL CB #%u n=%zd hdr=%02x%02x%02x%02x%02x%02x",
                cb_cnt, n,
                n>0?buf[0]:0, n>1?buf[1]:0, n>2?buf[2]:0,
                n>3?buf[3]:0, n>4?buf[4]:0, n>5?buf[5]:0);
    }
    if (n < 6 + 148) {
        if (cb_cnt <= 20)
            BSP_LOG("DL CB #%u DROP n=%zd < 154", cb_cnt, n);
        return;
    }
    uint8_t  tn = buf[0] & 0x07;
    uint32_t fn = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|
                  ((uint32_t)buf[3]<<8)|buf[4];
    const uint8_t *bits = buf + 6;

    /* Hard bits → int16 sample-shaped (just sign-extend ±1 for now;
     * the discovered FB-det loop reads 12 words at 0x0062..0x0078
     * and we store one bit per word). Real hardware gets I/Q soft
     * samples, but the FB-det DSP code only checks magnitudes. */
    int16_t iq[148];
    for (int i = 0; i < 148; i++)
        iq[i] = bits[i] ? +0x4000 : -0x4000;

    calypso_trx_on_dl_burst(tn, fn, iq, 148);
    calypso_bsp_rx_burst(tn, fn, iq, 148);
    calypso_trx_on_dl_l2(tn, fn, bits, 148);
}

/* UDP UL send: BSP owns the socket, ships 148 hard bits to bridge.
 *
 * osmo-bts-trx UL TRXDv0 wire:
 *   buf[0]       = tn
 *   buf[1..4]    = fn (BE u32)
 *   buf[5]       = positive RSSI byte; BTS computes -(int8_t)buf[5] dBm.
 *   buf[6..7]    = ToA256 BE int16.
 *   buf[8..155]  = 148 sbit_t (signed soft bits, -127..+127).
 * Total = 8 + 148 = 156 bytes.
 */
static void bsp_udp_ul_send(uint8_t tn, uint32_t fn, const uint8_t bits[148])
{
    if (bsp.trxd_fd < 0 || !bits) return;
    uint8_t pkt[8 + 148];
    pkt[0] = tn & 0x07;
    pkt[1] = (fn >> 24) & 0xff;
    pkt[2] = (fn >> 16) & 0xff;
    pkt[3] = (fn >>  8) & 0xff;
    pkt[4] =  fn        & 0xff;
    pkt[5] = 60;            /* → -60 dBm at the BTS */
    pkt[6] = 0; pkt[7] = 0;
    for (int i = 0; i < 148; i++)
        pkt[8 + i] = bits[i] ? 127 : (uint8_t)(-127);
    struct sockaddr_in dst = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port        = htons(6702),  /* bridge.py UL bind */
    };
    ssize_t r = sendto(bsp.trxd_fd, pkt, sizeof(pkt), 0,
                       (struct sockaddr *)&dst, sizeof(dst));
    if (r < 0) {
        static unsigned err_cnt;
        if (err_cnt++ < 5 || (err_cnt % 1000) == 0)
            BSP_LOG("UL sendto err #%u: %s", err_cnt, strerror(errno));
    }
}

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp        = dsp;
    bsp.trxd_fd    = -1;
    /* Bind 127.0.0.1:6802 — DL receive socket. Also used as UL send
     * source; bridge.py listens for UL on 127.0.0.1:6702. This matches
     * the fake_trx convention where the transceiver binds base+102 and
     * the peer (bridge, acting as BTS-side) binds base+2. */
    {
        int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (fd < 0) {
            BSP_LOG("UDP TRXD socket() failed: %s", strerror(errno));
        } else {
            int r = 1;
            setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &r, sizeof(r));
            struct sockaddr_in a = {
                .sin_family      = AF_INET,
                .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
                .sin_port        = htons(6802),
            };
            if (bind(fd, (struct sockaddr *)&a, sizeof(a)) == 0) {
                bsp.trxd_fd = fd;
                qemu_set_fd_handler(fd, bsp_udp_dl_cb, NULL, NULL);
                BSP_LOG("UDP TRXD bound 127.0.0.1:6802 (DL recv), "
                        "UL sendto 127.0.0.1:6702");
            } else {
                BSP_LOG("UDP TRXD bind 127.0.0.1:6802 failed: %s",
                        strerror(errno));
                close(fd);
            }
        }
    }
    /* Default refined 2026-04-07 (2nd histogram, after wedge fixes):
     * DSP now polls [0x3f80..0x3fbf] (real FB-det handler running).
     * Widen to 128 to cover [0x3f80..0x3fff]. */
    bsp.daram_addr     = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0x3f80);
    bsp.daram_len      = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  128);
    bsp.bypass_bdlena  = parse_uint_env("CALYPSO_BSP_BYPASS_BDLENA", 0);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)",
            bsp.bypass_bdlena ? "  (BDLENA gate BYPASSED — debug)" : "");
}

void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16)
{
    bsp.bursts_seen++;

    if (!bsp.dsp) {
        if (bsp.bursts_seen <= 3)
            BSP_LOG("rx_burst: no DSP attached, dropping fn=%u tn=%u", fn, tn);
        return;
    }
    if (n_int16 <= 0 || iq == NULL) return;

    if (bsp.daram_addr == 0) {
        if (bsp.bursts_seen <= 5) {
            BSP_LOG("rx_burst fn=%u tn=%u n=%d (target unset, sample[0]=%d sample[1]=%d)",
                    fn, tn, n_int16, iq[0], n_int16 > 1 ? iq[1] : 0);
        }
        return;
    }

    /* On real hw the BSP serial link only carries samples while IOTA's
     * BDLENA pin is asserted. Mirror that: only commit a burst into DARAM
     * if the firmware has issued a BDLENA rising edge that hasn't yet been
     * consumed by an earlier burst. */
    if (!bsp.bypass_bdlena && !calypso_iota_take_bdl_pulse(tn)) {
        bsp.bursts_dropped_no_window++;
        if (bsp.bursts_dropped_no_window <= 5 ||
            (bsp.bursts_dropped_no_window % 1000) == 0) {
            BSP_LOG("DROP fn=%u tn=%u (no BDLENA window, dropped=%llu)",
                    fn, tn,
                    (unsigned long long)bsp.bursts_dropped_no_window);
        }
        return;
    }

    int n = n_int16 < (int)bsp.daram_len ? n_int16 : (int)bsp.daram_len;

    /* Circular append: each successive burst writes at the previous
     * write offset within [daram_addr .. daram_addr+daram_len), wrapping
     * around. Models the BSP DMA filling a long FB sample buffer over
     * many serial bursts before the DSP correlator sweeps it. */
    static unsigned woff = 0;
    for (int i = 0; i < n; i++) {
        bsp.dsp->data[(uint16_t)(bsp.daram_addr + woff)] = (uint16_t)iq[i];
        woff++;
        if (woff >= bsp.daram_len) woff = 0;
    }
    bsp.bursts_written++;

    if (bsp.bursts_written <= 5 || (bsp.bursts_written % 100) == 0) {
        BSP_LOG("DMA fn=%u tn=%u n=%d → DARAM[0x%04x..0x%04x] total=%llu",
                fn, tn, n, bsp.daram_addr,
                (unsigned)(bsp.daram_addr + n - 1),
                (unsigned long long)bsp.bursts_written);
    }

    /* Fire BRINT0 — BSP receive complete (vec 21, IMR bit 5).
     * On real hw the BSP serial DMA generates this at end-of-burst;
     * the DSP's BRINT0 ISR runs the FB/SB correlator on the freshly
     * DMA'd DARAM page. BSP owns this wake now (was in calypso_trx).
     *
     * Gate on dsp->idle: if the DSP is currently running its boot
     * sequence (RPTB sweep, MVPD copies, init code), pre-empting it
     * with BRINT0 corrupts SP at ISR entry and locks the DSP. Wait
     * for the next IDLE window. */
    if (bsp.dsp && bsp.dsp->idle) {
        c54x_interrupt_ex(bsp.dsp, 21, 5);
        bsp.dsp->idle = false;
    }
}

/* Symmetric UL path: read 148 hard bits the DSP/L1 placed in its UL
 * buffer (DARAM 0x0900..0x0993, same convention used by the legacy
 * calypso_trx_send_ul_burst path) and hand them to the caller.
 * Returns true if any non-zero bit found (= "real" burst). */
bool calypso_bsp_tx_burst(uint8_t tn, uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;
    static uint64_t ul_seen, ul_sent;
    ul_seen++;

    bool any = false;
    for (int i = 0; i < 148; i++) {
        uint16_t w = bsp.dsp->data[0x0900 + i];
        bits[i] = (uint8_t)(w & 1);
        if (bits[i]) any = true;
    }

    /* Always ship — even an all-zero burst keeps the BTS happy and
     * lets us see UL packets on the wire. The DSP rarely writes real
     * UL while it's still in FB-det, but we don't want to lose the
     * cadence. */
    ul_sent++;
    calypso_trx_on_ul_burst(tn, fn, bits);
    bsp_udp_ul_send(tn, fn, bits);
    if (ul_sent <= 5 || (ul_sent % 100) == 0) {
        BSP_LOG("UL DMA fn=%u tn=%u any=%d (sent=%llu seen=%llu)",
                fn, tn, any,
                (unsigned long long)ul_sent,
                (unsigned long long)ul_seen);
    }
    return true;
}
