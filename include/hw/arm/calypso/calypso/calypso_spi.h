/*
 * calypso_spi.h — Calypso SPI + TWL3025 ABB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_CALYPSO_SPI_H
#define HW_SSI_CALYPSO_SPI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CALYPSO_SPI "calypso-spi"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoSPIState, CALYPSO_SPI)

struct CalypsoSPIState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq     irq;

    /* Registers matching real Calypso SPI layout */
    uint16_t set1;      /* 0x00 SET1 */
    uint16_t set2;      /* 0x02 SET2 */
    uint16_t ctrl;      /* 0x04 CTRL */
    uint16_t status;    /* 0x06 STATUS */
    uint16_t tx_data;   /* 0x08/0x0A TX_LSB/MSB */
    uint16_t rx_data;   /* 0x0C/0x0E RX_LSB/MSB */

    /* TWL3025 shadow registers (256 possible addresses) */
    uint16_t abb_regs[256];
};

/* TWL3025 important register addresses */
#define ABB_VRPCDEV    0x01
#define ABB_VRPCSTS    0x02
#define ABB_VBUCTRL    0x03
#define ABB_VBDR1      0x04
#define ABB_TOGBR1     0x09
#define ABB_TOGBR2     0x0A
#define ABB_AUXLED     0x17
#define ABB_ITSTATREG  0x1B

/* SPI status bits (real Calypso) */
#define SPI_STATUS_RE        (1 << 1)   /* Ready / transfer complete */

/* Legacy compat */
#define SPI_STATUS_TX_READY  SPI_STATUS_RE
#define SPI_STATUS_RX_READY  SPI_STATUS_RE

#endif /* HW_SSI_CALYPSO_SPI_H */
