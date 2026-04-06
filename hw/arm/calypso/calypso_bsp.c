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
#include <stdio.h>
#include <stdlib.h>
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_c54x.h"

#define BSP_LOG(fmt, ...) \
    do { fprintf(stderr, "[BSP] " fmt "\n", ##__VA_ARGS__); } while (0)

static struct {
    C54xState *dsp;
    uint16_t   daram_addr;   /* word address in DSP data space; 0 = unset */
    uint16_t   daram_len;    /* max words to copy per burst */
    uint64_t   bursts_seen;
    uint64_t   bursts_written;
} bsp;

static uint16_t parse_uint_env(const char *name, uint16_t def)
{
    const char *v = getenv(name);
    if (!v || !*v) return def;
    return (uint16_t)strtoul(v, NULL, 0);
}

void calypso_bsp_init(C54xState *dsp)
{
    bsp.dsp        = dsp;
    bsp.daram_addr = parse_uint_env("CALYPSO_BSP_DARAM_ADDR", 0);
    bsp.daram_len  = parse_uint_env("CALYPSO_BSP_DARAM_LEN",  1184);
    bsp.bursts_seen = 0;
    bsp.bursts_written = 0;
    BSP_LOG("init dsp=%p daram_addr=0x%04x len=%u%s",
            (void *)dsp, bsp.daram_addr, bsp.daram_len,
            bsp.daram_addr ? "" : "  (DISCOVERY mode — no DMA)");
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

    int n = n_int16 < (int)bsp.daram_len ? n_int16 : (int)bsp.daram_len;

    /* Direct word copy: DSP DARAM is 16-bit; samples are signed int16. */
    for (int i = 0; i < n; i++) {
        bsp.dsp->data[(uint16_t)(bsp.daram_addr + i)] = (uint16_t)iq[i];
    }
    bsp.bursts_written++;

    if (bsp.bursts_written <= 5 || (bsp.bursts_written % 100) == 0) {
        BSP_LOG("DMA fn=%u tn=%u n=%d → DARAM[0x%04x..0x%04x] total=%llu",
                fn, tn, n, bsp.daram_addr,
                (unsigned)(bsp.daram_addr + n - 1),
                (unsigned long long)bsp.bursts_written);
    }
}
