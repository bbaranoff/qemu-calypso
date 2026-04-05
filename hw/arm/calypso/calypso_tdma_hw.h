/*
 * calypso_tdma_hw.h — Calypso TDMA hardware counter for DSP
 *
 * The Calypso has a hardware TDMA frame counter that both ARM and DSP
 * can read. The DSP uses it to know which frame to process.
 *
 * On real hardware, the TPU maintains the TDMA counter and the DSP
 * reads it via a dedicated register or shared memory location.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TDMA_HW_H
#define CALYPSO_TDMA_HW_H

#include <stdint.h>

/* TDMA hardware registers accessible by DSP
 * These are in the DSP data address space at fixed addresses */

/* Frame number register — DSP reads this to know current FN */
#define TDMA_HW_FN_ADDR      0x0036  /* DSP data address for FN low 16 bits */
#define TDMA_HW_FN_HI_ADDR   0x0037  /* DSP data address for FN high bits */

/* Frame sync flag — set by TPU when new frame starts */
#define TDMA_HW_SYNC_ADDR    0x0034  /* DSP data address for frame sync flag */

/* Timeslot number — current timeslot being processed */
#define TDMA_HW_TN_ADDR      0x0035  /* DSP data address for timeslot */

/* Rx burst ready flag — set by ABB/DMA when burst samples are captured */
#define TDMA_HW_RX_READY     0x0038  /* DSP data address for rx ready flag */

/* Rx burst sample buffer pointer — where samples are in DARAM */
#define TDMA_HW_RX_PTR       0x0039  /* DSP data address for sample buffer ptr */

/* Rx burst sample count */
#define TDMA_HW_RX_COUNT     0x003A  /* DSP data address for sample count */

typedef struct TdmaHW {
    uint32_t fn;          /* current frame number */
    uint8_t  tn;          /* current timeslot */
    uint16_t sync_flag;   /* frame sync flag (non-zero = new frame) */
    uint16_t rx_ready;    /* rx samples available */
    uint16_t rx_ptr;      /* pointer to rx samples in DSP DARAM */
    uint16_t rx_count;    /* number of rx samples */

    /* Pointer to DSP DARAM for direct write */
    uint16_t *dsp_daram;
} TdmaHW;

/* Initialize TDMA hardware */
void tdma_hw_init(TdmaHW *hw, uint16_t *dsp_daram);

/* Advance to next frame — called from QEMU TDMA tick */
void tdma_hw_frame_tick(TdmaHW *hw, uint32_t fn);

/* Signal that rx burst samples are ready at given DARAM address */
void tdma_hw_rx_burst_ready(TdmaHW *hw, uint16_t daram_ptr, uint16_t count);

/* Update DSP DARAM registers — call before C54x runs */
void tdma_hw_update_dsp(TdmaHW *hw);

#endif /* CALYPSO_TDMA_HW_H */
