/*
 * calypso_soc.h - TI Calypso System-on-Chip
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

#define CALYPSO_SOC_NUM_IRQS  2

struct CalypsoSoCState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iram;

    CalypsoINTHState  inth;
    CalypsoTimerState timer1;
    CalypsoTimerState timer2;
    CalypsoUARTState  uart_modem;
    CalypsoUARTState  uart_irda;
    CalypsoSPIState   spi;

    void *trx;

    qemu_irq cpu_irq;
    qemu_irq cpu_fiq;

    /* IRAM-at-zero alias (controlled by CNTL register) */
    MemoryRegion iram_alias;
    bool iram_at_zero;
    MemoryRegion cntl_iomem;
    uint16_t extra_conf;

};

#endif /* HW_ARM_CALYPSO_SOC_H */
