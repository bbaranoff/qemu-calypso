/*
 * calypso_trx.h — Calypso DSP/TPU hardware emulation
 * Pure hardware — no sockets, no DSP processing. Firmware does everything.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TRX_H
#define CALYPSO_TRX_H

#include "hw/irq.h"
#include "exec/memory.h"

/* IRQ map */
#define CALYPSO_IRQ_WATCHDOG       0
#define CALYPSO_IRQ_TIMER1         1
#define CALYPSO_IRQ_TIMER2         2
#define CALYPSO_IRQ_TSP_RX         3
#define CALYPSO_IRQ_TPU_FRAME      4
#define CALYPSO_IRQ_TPU_PAGE       5
#define CALYPSO_IRQ_SIM            6
#define CALYPSO_IRQ_UART_MODEM     7
#define CALYPSO_IRQ_KEYPAD_GPIO    8
#define CALYPSO_IRQ_RTC_TIMER      9
#define CALYPSO_IRQ_RTC_ALARM      10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14
#define CALYPSO_IRQ_API            15
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_NUM_IRQS           32

/* Hardware addresses */
#define CALYPSO_DSP_BASE      0xFFD00000
#define CALYPSO_DSP_SIZE      (64 * 1024)
#define CALYPSO_TPU_BASE      0xFFFF1000
#define CALYPSO_TPU_SIZE      0x0100
#define CALYPSO_TPU_RAM_BASE  0xFFFF9000
#define CALYPSO_TPU_RAM_SIZE  0x0800
#define CALYPSO_TSP_BASE      0xFFFE0800
#define CALYPSO_TSP_SIZE      0x0100
#define CALYPSO_SIM_BASE      0xFFFE0000
#define CALYPSO_SIM_SIZE      0x0100
#define CALYPSO_ULPD_BASE     0xFFFE2800
#define CALYPSO_ULPD_SIZE     0x0100

/* TPU register offsets */
#define TPU_CTRL              0x0000
#define TPU_INT_CTRL          0x0002
#define TPU_INT_STAT          0x0004
#define TPU_OFFSET            0x000C
#define TPU_SYNCHRO           0x000E
#define TPU_IT_DSP_PG         0x0020

/* TPU_CTRL bits */
#define TPU_CTRL_RESET        (1 << 0)
#define TPU_CTRL_PAGE         (1 << 1)
#define TPU_CTRL_EN           (1 << 2)
#define TPU_CTRL_DSP_EN       (1 << 4)
#define TPU_CTRL_MCU_RAM_ACC  (1 << 6)
#define TPU_CTRL_TSP_RESET    (1 << 7)
#define TPU_CTRL_IDLE         (1 << 8)
#define TPU_CTRL_WAIT         (1 << 9)
#define TPU_CTRL_CK_ENABLE    (1 << 10)
#define TPU_CTRL_FULL_WRITE   (1 << 11)

/* TPU INT_CTRL bits */
#define ICTRL_MCU_FRAME       (1 << 0)
#define ICTRL_MCU_PAGE        (1 << 1)
#define ICTRL_DSP_FRAME       (1 << 2)
#define ICTRL_DSP_FRAME_FORCE (1 << 3)

/* TSP */
#define TSP_RX_REG            0x08

/* ULPD */
#define ULPD_SETUP_CLK13      0x00
#define ULPD_COUNTER_HI       0x1C
#define ULPD_COUNTER_LO       0x1E
#define ULPD_GAUGING_CTRL     0x24
#define ULPD_GSM_TIMER        0x28

/* GSM timing — real 4.615ms TDMA frame period */
#define GSM_TDMA_NS           4615000
#define GSM_HYPERFRAME        2715648

/* DSP boot */
#define DSP_DL_STATUS_ADDR    0x0FFE
#define DSP_API_VER_ADDR      0x01B4
#define DSP_API_VER2_ADDR     0x01B6
#define DSP_DL_STATUS_RESET   0x0000
#define DSP_DL_STATUS_BOOT    0x0001
#define DSP_DL_STATUS_READY   0x0002
#define DSP_API_VERSION       0x3606

void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs);

/* Sercomm burst transport (DLCI 4) — called by UART hardware */
void calypso_trx_rx_burst(const uint8_t *data, int len);
void calypso_trx_tx_burst_poll(void);

/* Current TDMA frame number (0..GSM_HYPERFRAME-1). Used by BSP for
 * FN-alignment of arriving DL bursts. Returns 0 before TDMA starts. */
uint32_t calypso_trx_get_fn(void);

#endif /* CALYPSO_TRX_H */
