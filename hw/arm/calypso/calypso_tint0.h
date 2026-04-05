/*
 * calypso_tint0.h -- TINT0 master clock for Calypso GSM virtualization
 *
 * On real Calypso hardware, the C54x DSP Timer 0 (TIM/PRD/TCR at
 * DSP addresses 0x0024-0x0026) generates TINT0 (IFR bit 4, vector 20).
 * The DSP ROM configures Timer 0 to fire at TDMA frame rate (4.615ms).
 * TINT0 is the master clock that synchronizes everything:
 *
 *   TINT0 (DSP Timer 0, 4.615ms)
 *     +-- DSP: SINT17 frame interrupt -> process GSM tasks
 *     +-- TPU: frame sync -> burst scheduling
 *     +-- ARM: TPU_FRAME IRQ -> L1 scheduler
 *     +-- ARM: API IRQ -> read DSP results
 *     +-- UART: poll TX/RX
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TINT0_H
#define CALYPSO_TINT0_H

#include <stdint.h>
#include <stdbool.h>

/* Hardware constants from TMS320C54x Timer 0 */
#define TINT0_PERIOD_US     4615         /* 4.615 ms per TDMA frame */
#define TINT0_PERIOD_NS     4615000ULL   /* in nanoseconds */
#define GSM_HYPERFRAME      2715648      /* GSM hyperframe modulus */

/* C54x Timer 0 registers (DSP data addresses) */
#define TINT0_TIM_ADDR      0x0024       /* Timer counter */
#define TINT0_PRD_ADDR      0x0025       /* Timer period */
#define TINT0_TCR_ADDR      0x0026       /* Timer control */

/* C54x IFR/IMR bit for TINT0 */
#define TINT0_IFR_BIT       3            /* IFR/IMR bit 4 */
#define TINT0_VEC           19           /* Interrupt vector 20 (offset 0x50) */

/* Start the master clock */
void calypso_tint0_start(void);

/* Notify that TPU_CTRL_EN was written (ARM requests frame processing) */
void calypso_tint0_tpu_en(void);
bool calypso_tint0_tpu_en_pending(void);
void calypso_tint0_tpu_en_clear(void);

/* Frame number access */
uint32_t calypso_tint0_fn(void);
void calypso_tint0_set_fn(uint32_t fn);
bool calypso_tint0_running(void);

#endif /* CALYPSO_TINT0_H */
