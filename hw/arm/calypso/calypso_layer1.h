/*
 * calypso_layer1.h — HLE L1 model for the Calypso DSP (CALYPSO_L1=c).
 *
 * Intermediate FUNCTIONAL SCAFFOLD: models the DSP's L1 signal processing in C,
 * clocked by calypso_tdma_tick. It lets the upper stack (cell selection, BCCH,
 * SI) be exercised while the real-c54x revival (the LLE / IPTR-faithful arc)
 * stays the goal behind CALYPSO_L1 off. NOT the endgame — a conscious bridge.
 *
 * Reads I/Q from DARAM 0x2a00, writes results into the API RAM where the ARM L1
 * (osmocom-bb, unmodified) reads them.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_LAYER1_H
#define CALYPSO_LAYER1_H

#include "hw/arm/calypso/calypso_c54x.h"

/* CALYPSO_L1=c → 1 (memoized once). Default off → mock/revival untouched. */
int calypso_l1_c_active(void);

/* Per-frame hook, called from calypso_tdma_tick just before the TPU FRAME IRQ.
 * dsp->data = shared API RAM / DARAM (I/Q at 0x2a00, outputs at NDB offsets).
 * dsp_ram   = ARM-side mirror (write-page task fields d_task_md/d_task_d).
 * fn        = current TDMA frame number. */
void calypso_layer1_tick(C54xState *dsp, uint16_t *dsp_ram, uint32_t fn);

/* Latch the ARM's d_task_md write (called from calypso_dsp_write). The tick-time
 * poll misses the transient task=5; this captures it at write time. */
void calypso_layer1_on_task_write(uint16_t md);

#endif /* CALYPSO_LAYER1_H */
