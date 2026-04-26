/*
 * TWL3025 / IOTA model — implementation.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include <string.h>
#include "hw/arm/calypso/calypso_iota.h"

#define IOTA_LOG(fmt, ...) \
    do { fprintf(stderr, "[iota] " fmt "\n", ##__VA_ARGS__); } while (0)

/* Pending BDLENA windows queued by the TPU sequencer, waiting for a
 * matching downlink burst to arrive on the BSP. Sized for one full TDMA
 * frame's worth of slots so successive armings on different TNs queue up
 * cleanly. */
#define IOTA_PENDING_MAX 32

static struct {
    uint8_t  last_byte;     /* most recent TSP byte */
    bool     bdl_ena;       /* current state of BDLENA pin */
    bool     bul_ena;       /* current state of BULENA pin */
    uint32_t bdl_pulses;    /* total BDLENA rising edges */
    uint32_t writes_seen;
    /* Pending pulses: each holds the TN the L1 armed for. */
    uint8_t  pending_tn[IOTA_PENDING_MAX];
    int      pending_head, pending_tail;
} iota;

static int iota_pending_count(void)
{
    int n = iota.pending_tail - iota.pending_head;
    if (n < 0) n += IOTA_PENDING_MAX;
    return n;
}

static void iota_pending_push(uint8_t tn)
{
    if (iota_pending_count() >= IOTA_PENDING_MAX - 1) {
        /* Saturation: BSP isn't consuming pulses fast enough (typically
         * the DSP is stalled and the TDMA tick can't advance fn far enough
         * to match queued bursts in calypso_bsp_deliver_buffered).
         * Rate-limit the warning so it doesn't drown the log: first 5,
         * then every 1000th drop, plus a periodic count summary. */
        static uint64_t drops;
        drops++;
        if (drops <= 5 || (drops % 1000) == 0) {
            IOTA_LOG("WARN pending queue full, dropping oldest "
                     "(drops=%llu)", (unsigned long long)drops);
        }
        iota.pending_head = (iota.pending_head + 1) % IOTA_PENDING_MAX;
    }
    iota.pending_tn[iota.pending_tail] = tn;
    iota.pending_tail = (iota.pending_tail + 1) % IOTA_PENDING_MAX;
}

void calypso_iota_init(void)
{
    memset(&iota, 0, sizeof(iota));
    IOTA_LOG("init");
}

void calypso_iota_tsp_write(uint8_t data, uint8_t expected_tn)
{
    bool prev_bdl = iota.bdl_ena;
    bool prev_bul = iota.bul_ena;

    iota.last_byte = data;
    iota.bdl_ena   = !!(data & IOTA_TSP_BDLENA);
    iota.bul_ena   = !!(data & IOTA_TSP_BULENA);
    iota.writes_seen++;

    if (!prev_bdl && iota.bdl_ena) {
        iota.bdl_pulses++;
        iota_pending_push(expected_tn);
        if (iota.bdl_pulses <= 10 || (iota.bdl_pulses % 100) == 0) {
            IOTA_LOG("BDLENA rising edge #%u tn=%u pending=%d",
                     iota.bdl_pulses, expected_tn, iota_pending_count());
        }
    }
    if (prev_bul != iota.bul_ena && iota.writes_seen <= 10) {
        IOTA_LOG("BULENA -> %d (data=0x%02x)", iota.bul_ena, data);
    }
}

bool calypso_iota_bdl_ena(void)            { return iota.bdl_ena; }
uint32_t calypso_iota_bdl_ena_pulses(void) { return iota.bdl_pulses; }

bool calypso_iota_take_bdl_pulse(uint8_t tn)
{
    /* Walk the pending queue from oldest to newest looking for a TN
     * match. Newer pending pulses past the matched one stay queued. */
    int n = iota_pending_count();
    for (int i = 0; i < n; i++) {
        int idx = (iota.pending_head + i) % IOTA_PENDING_MAX;
        if (iota.pending_tn[idx] == tn) {
            /* Consume: shift everything from head..idx forward by 1 */
            for (int j = i; j > 0; j--) {
                int dst = (iota.pending_head + j) % IOTA_PENDING_MAX;
                int src = (iota.pending_head + j - 1) % IOTA_PENDING_MAX;
                iota.pending_tn[dst] = iota.pending_tn[src];
            }
            iota.pending_head = (iota.pending_head + 1) % IOTA_PENDING_MAX;
            return true;
        }
    }
    return false;
}
