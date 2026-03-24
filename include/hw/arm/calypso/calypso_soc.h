/*
 * calypso_soc.h - TI Calypso System-on-Chip
 *
 * Complete SoC device containing all integrated peripherals:
 * - Internal RAM (256 KiB)
 * - Interrupt Controller (INTH)
 * - 2x Timers
 * - 2x UARTs
 * - SPI + TWL3025 ABB
 * - DSP/TPU/TRX bridge
 *
 * This provides a modular alternative to the monolithic calypso_high.c
 * machine, with better separation between SoC and board-level components.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_CALYPSO_SOC_H
#define HW_ARM_CALYPSO_SOC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/arm/calypso/calypso_inth.h"
#include "hw/arm/calypso/calypso_timer.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_spi.h"

#define TYPE_CALYPSO_SOC "calypso-soc"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSoCState, CALYPSO_SOC)

/* Number of IRQ lines from SoC to CPU */
#define CALYPSO_SOC_NUM_IRQS  2  /* IRQ and FIQ */

struct CalypsoSoCState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    /* Internal RAM (256 KiB at 0x00800000) */
    MemoryRegion iram;
    MemoryRegion iram_alias;   /* ← AJOUTE ÇA */
    MemoryRegion ulpd_stub;   /* ← AJOUTE ÇA */
    /* Integrated peripherals (as embedded objects) */
    CalypsoINTHState  inth;
    CalypsoTimerState timer1;
    CalypsoTimerState timer2;
    CalypsoUARTState  uart_modem;
    CalypsoUARTState  uart_irda;
    CalypsoSPIState   spi;
    /* TRX bridge (created dynamically, not embedded) */
    void *trx;  /* Opaque pointer to avoid circular dependency */

    /* IRQ outputs to CPU (connected in machine init) */
    qemu_irq cpu_irq;
    qemu_irq cpu_fiq;

    /* Configuration properties */
    uint16_t trx_port;    /* TRX UDP port (0 = disabled) */
    bool enable_trx;      /* Enable TRX bridge */
};

#endif /* HW_ARM_CALYPSO_SOC_H */
