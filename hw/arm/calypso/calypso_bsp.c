/*
 * Calypso BSP/RIF DMA module — implementation.
 *
 * On real hardware the BSP (Baseband Serial Port) is a synchronous serial
 * link that DMA-feeds I/Q samples from the IOTA RF frontend into C54x
 * DSP DARAM. The DSP code (FB/SB/burst detection in PROM0) reads them
 * from a fixed DARAM buffer and posts results into the NDB.
 *
 * In QEMU, DL bursts arrive via UDP (TRXDv0 from bridge.py on port 5702).
 * This module owns that socket, decodes the TRXDv0 header, converts hard
 * bits to I/Q samples, and DMA-writes them into DSP DARAM.
 *
 * L1CTL control (DLCI 5) goes through the UART — bursts never touch UART.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_iota.h"

#define BSP_LOG(fmt, ...) \
    do { fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

#define BSP_TRXD_PORT  6702   /* bridge forwards DL bursts here (5702 is bridge's own) */

static struct {
    C54xState *dsp;
    uint16_t   daram_addr;
    uint16_t   daram_len;
    uint16_t   bypass_bdlena;
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
    uint64_t   bursts_dropped_no_window;
    int        trxd_fd;            /* UDP socket for TRXDv0 DL bursts */
    struct sockaddr_in trxd_peer;  /* BTS address (for UL replies) */
    bool       trxd_peer_valid;
    uint8_t    last_att;           /* last DL attenuation byte */
} bsp;

static uint16_t parse_uint_env(const char *name, uint16_t def)
{
    const char *v = getenv(name);
    if (!v || !*v) return def;
    return (uint16_t)strtoul(v, NULL, 0);
}

uint16_t calypso_bsp_get_daram_addr(void) { return bsp.daram_addr; }
uint16_t calypso_bsp_get_daram_len(void)  { return bsp.daram_len; }
uint8_t  calypso_bsp_get_last_att(void)   { return bsp.last_att; }

/* ---- UDP TRXDv0 DL receive callback ---- */

static void bsp_trxd_readable(void *opaque)
{
    uint8_t buf[512];
    struct sockaddr_in addr;
    socklen_t alen = sizeof(addr);

    ssize_t n = recvfrom(bsp.trxd_fd, buf, sizeof(buf), 0,
                         (struct sockaddr *)&addr, &alen);
    if (n < 8) return;

    /* Remember BTS peer for UL replies */
    if (!bsp.trxd_peer_valid) {
        bsp.trxd_peer = addr;
        bsp.trxd_peer_valid = true;
        BSP_LOG("TRXD peer: %s:%d",
                inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    }

    /* TRXDv0 DL: tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes */
    uint8_t  tn  = buf[0] & 0x07;
    uint32_t fn  = ((uint32_t)buf[1]<<24)|((uint32_t)buf[2]<<16)|
                   ((uint32_t)buf[3]<<8)|buf[4];
    bsp.last_att = (n > 5) ? buf[5] : 0;

    int nbits = (int)n - 8;  /* skip 8-byte header */
    if (nbits > 148) nbits = 148;
    if (nbits <= 0) return;

    const uint8_t *bits = buf + 8;

    /* Log burst type: check if all-zero (FB) or mixed (NB/SB) */
    {
        int zeros = 0, ones = 0;
        for (int i = 0; i < nbits; i++) {
            if (bits[i] == 0) zeros++;
            else ones++;
        }
        static int burst_log = 0;
        if (burst_log < 20 || (burst_log % 10000) == 0) {
            BSP_LOG("BURST fn=%u tn=%u zeros=%d ones=%d %s",
                    fn, tn, zeros, ones,
                    zeros == nbits ? "*** FB ***" :
                    ones > 100 ? "DUMMY/NB" : "SB/OTHER");
        }
        burst_log++;
    }

    /* GMSK modulation: convert TRXDv0 hard bits to I/Q samples.
     * GMSK with h=0.5: each bit adds ±π/2 to the phase.
     * NRZ encoding: bit 0 → phase += π/2, bit 1 → phase -= π/2.
     * I = cos(phase), Q = sin(phase), scaled to int16.
     *
     * Since phase increments are exactly ±π/2, cos/sin cycle through
     * {0, ±1} — we use a lookup table instead of floating point. */
    int16_t iq[148];
    static const int16_t cos_tab[4] = { 0x3FFF, 0, -0x3FFF, 0 };  /* cos(0,π/2,π,3π/2) */
    int phase_idx = 0;  /* phase / (π/2), mod 4 */
    for (int i = 0; i < nbits; i++) {
        /* NRZ: bit 0 → +1 (advance phase), bit 1 → -1 (retard phase) */
        phase_idx = (phase_idx + (bits[i] ? 3 : 1)) & 3;  /* +1 or -1 mod 4 */
        iq[i] = cos_tab[phase_idx];  /* I component only (DSP reads magnitude) */
    }

    calypso_bsp_rx_burst(tn, fn, iq, nbits);
}

/* ---- Init ---- */

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp = dsp;
    bsp.daram_addr     = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0x3fc0);
    bsp.daram_len      = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  64);
    bsp.bypass_bdlena  = parse_uint_env("CALYPSO_BSP_BYPASS_BDLENA", 0);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    bsp.trxd_fd = -1;
    bsp.trxd_peer_valid = false;

    /* Bind UDP socket for TRXDv0 DL bursts from bridge/BTS */
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        int one = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        struct sockaddr_in sa = {
            .sin_family = AF_INET,
            .sin_port = htons(BSP_TRXD_PORT),
            .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        };
        if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            qemu_set_fd_handler(fd, bsp_trxd_readable, NULL, NULL);
            bsp.trxd_fd = fd;
            BSP_LOG("TRXD UDP listening on 127.0.0.1:%d", BSP_TRXD_PORT);
        } else {
            BSP_LOG("TRXD bind port %d failed: %s", BSP_TRXD_PORT, strerror(errno));
            close(fd);
        }
    }

    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)",
            bsp.bypass_bdlena ? "  (BDLENA gate BYPASSED — debug)" : "");
}

/* ---- DL burst → DSP DARAM ---- */

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
            BSP_LOG("rx_burst fn=%u tn=%u n=%d (target unset)",
                    fn, tn, n_int16);
        }
        return;
    }

    /* On real hw the BSP serial link only carries samples while IOTA's
     * BDLENA pin is asserted. */
    if (!bsp.bypass_bdlena && !calypso_iota_take_bdl_pulse(tn)) {
        bsp.bursts_dropped_no_window++;
        if (bsp.bursts_dropped_no_window <= 5 ||
            (bsp.bursts_dropped_no_window % 100000) == 0) {
            BSP_LOG("DROP fn=%u tn=%u (no BDLENA window, dropped=%llu)",
                    fn, tn,
                    (unsigned long long)bsp.bursts_dropped_no_window);
        }
        return;
    }

    int n = n_int16 < (int)bsp.daram_len ? n_int16 : (int)bsp.daram_len;

    /* Load samples into BSP serial port buffer (PORTR PA=0x0034).
     * The DSP reads one sample per PORTR instruction from this buffer. */
    uint16_t samples[148];
    for (int i = 0; i < n && i < 148; i++)
        samples[i] = (uint16_t)iq[i];
    c54x_bsp_load(bsp.dsp, samples, n > 148 ? 148 : n);

    /* Also write to DARAM for code that reads samples directly. */
    static unsigned woff = 0;
    for (int i = 0; i < n; i++) {
        bsp.dsp->data[(uint16_t)(bsp.daram_addr + woff)] = (uint16_t)iq[i];
        woff++;
        if (woff >= bsp.daram_len) woff = 0;
    }
    bsp.bursts_written++;

    /* Log DARAM content after write for FB bursts */
    {
        if (bsp.bursts_written <= 3) {
            BSP_LOG("DARAM after write [0x%04x]: %d %d %d %d %d %d %d %d",
                    bsp.daram_addr,
                    n>0?(int16_t)bsp.dsp->data[bsp.daram_addr]:0,
                    n>1?(int16_t)bsp.dsp->data[bsp.daram_addr+1]:0,
                    n>2?(int16_t)bsp.dsp->data[bsp.daram_addr+2]:0,
                    n>3?(int16_t)bsp.dsp->data[bsp.daram_addr+3]:0,
                    n>4?(int16_t)bsp.dsp->data[bsp.daram_addr+4]:0,
                    n>5?(int16_t)bsp.dsp->data[bsp.daram_addr+5]:0,
                    n>6?(int16_t)bsp.dsp->data[bsp.daram_addr+6]:0,
                    n>7?(int16_t)bsp.dsp->data[bsp.daram_addr+7]:0);
        }
    }
    if (bsp.bursts_written <= 5 || (bsp.bursts_written % 1000) == 0) {
        BSP_LOG("DMA fn=%u tn=%u n=%d → DARAM[0x%04x..0x%04x] total=%llu "
                "iq[0..3]=%d,%d,%d,%d",
                fn, tn, n, bsp.daram_addr,
                (unsigned)(bsp.daram_addr + n - 1),
                (unsigned long long)bsp.bursts_written,
                n>0 ? iq[0] : 0, n>1 ? iq[1] : 0,
                n>2 ? iq[2] : 0, n>3 ? iq[3] : 0);
    }

    /* Fire BRINT0 — gated by BDLENA from the TPU/TSP/IOTA chain.
     * The firmware opens the RX window via TPU scenario → TSP write → IOTA BDLENA.
     * calypso_iota_take_bdl_pulse() consumed the window above.
     * BRINT0 fires once per window, rate-limited by IFR bit. */
    if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
        c54x_interrupt_ex(bsp.dsp, 21, 5);
        if (bsp.dsp->idle) bsp.dsp->idle = false;
    }
}

/* ---- UL burst → UDP to BTS ---- */

void calypso_bsp_send_ul(uint8_t tn, uint32_t fn, const uint8_t bits[148])
{
    if (bsp.trxd_fd < 0 || !bsp.trxd_peer_valid) return;

    /* TRXDv0 UL: tn(1) fn(4) rssi(1) toa(2) bits(148) = 156 bytes */
    uint8_t pkt[8 + 148];
    pkt[0] = tn & 0x07;
    pkt[1] = (fn >> 24) & 0xff;
    pkt[2] = (fn >> 16) & 0xff;
    pkt[3] = (fn >>  8) & 0xff;
    pkt[4] =  fn        & 0xff;
    pkt[5] = 60;            /* RSSI → -60 dBm at the BTS */
    pkt[6] = 0; pkt[7] = 0; /* ToA256 = 0 */
    for (int i = 0; i < 148; i++)
        pkt[8 + i] = bits[i] ? 127 : (uint8_t)(-127);

    sendto(bsp.trxd_fd, pkt, sizeof(pkt), 0,
           (struct sockaddr *)&bsp.trxd_peer, sizeof(bsp.trxd_peer));
}

bool calypso_bsp_tx_burst(uint8_t tn, uint32_t fn, uint8_t bits[148])
{
    if (!bsp.dsp || !bits) return false;

    bool any = false;
    for (int i = 0; i < 148; i++) {
        uint16_t w = bsp.dsp->data[0x0900 + i];
        bits[i] = (uint8_t)(w & 1);
        if (bits[i]) any = true;
    }

    return any;
}
