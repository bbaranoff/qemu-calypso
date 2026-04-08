/*
 * calypso_tdma_hw.c — Calypso TDMA hardware counter for DSP
 *
 * Emulates the hardware TDMA frame counter and synchronization
 * registers that the C54x DSP reads during frame processing.
 *
 * On real Calypso hardware:
 * - TPU maintains the TDMA frame counter
 * - DSP reads it from fixed DARAM addresses
 * - ABB/DMA sets rx_ready when burst samples are captured
 * - DSP reads samples from the address pointed by rx_ptr
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "calypso_tdma_hw.h"
#include <string.h>
#include <stdio.h>

#define TDMA_LOG(fmt, ...) \
    fprintf(stderr, "[tdma-hw] " fmt "\n", ##__VA_ARGS__)

void tdma_hw_init(TdmaHW *hw, uint16_t *dsp_daram)
{
    memset(hw, 0, sizeof(*hw));
    hw->dsp_daram = dsp_daram;
    TDMA_LOG("init: DARAM ptrs set");
}

void tdma_hw_frame_tick(TdmaHW *hw, uint32_t fn)
{
    hw->fn = fn;
    hw->sync_flag = 1;  /* new frame available */
    hw->rx_ready = 0;   /* no rx samples yet this frame */
}

void tdma_hw_rx_burst_ready(TdmaHW *hw, uint16_t daram_ptr, uint16_t count)
{
    hw->rx_ready = 1;
    hw->rx_ptr = daram_ptr;
    hw->rx_count = count;
}

void tdma_hw_update_dsp(TdmaHW *hw)
{
    if (!hw->dsp_daram) return;

    /* Write hardware registers to DSP DARAM at fixed addresses */
    hw->dsp_daram[TDMA_HW_SYNC_ADDR] = hw->sync_flag;
    hw->dsp_daram[TDMA_HW_TN_ADDR]   = hw->tn;
    hw->dsp_daram[TDMA_HW_FN_ADDR]   = hw->fn & 0xFFFF;
    hw->dsp_daram[TDMA_HW_FN_HI_ADDR] = (hw->fn >> 16) & 0xFFFF;
    hw->dsp_daram[TDMA_HW_RX_READY]  = hw->rx_ready;
    hw->dsp_daram[TDMA_HW_RX_PTR]    = hw->rx_ptr;
    hw->dsp_daram[TDMA_HW_RX_COUNT]  = hw->rx_count;

    /* Clear sync flag after DSP has had a chance to read it */
    hw->sync_flag = 0;
}
