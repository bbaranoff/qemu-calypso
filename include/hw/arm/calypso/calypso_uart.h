/*
 * calypso_uart.h — Calypso UART device
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_CALYPSO_UART_H
#define HW_CHAR_CALYPSO_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_CALYPSO_UART "calypso-uart"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoUARTState, CALYPSO_UART)

/*
 * Large RX FIFO to tolerate Compal/sercomm bursts.
 */
#define CALYPSO_UART_RX_FIFO_SIZE 8192

typedef struct CalypsoUARTState {
    SysBusDevice parent_obj;

    /* MMIO */
    MemoryRegion iomem;

    /* QEMU backend */
    CharBackend chr;
    qemu_irq irq;

    /* Debug label ("modem", "irda") */
    char *label;

    /* Base registers */
    uint8_t ier;
    uint8_t iir;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;
    uint8_t msr;
    uint8_t spr;
    uint8_t dll;
    uint8_t dlh;
    uint8_t mdr1;

    /* Extended/banked registers used by Calypso loader/uart driver */
    uint8_t efr;
    uint8_t xon1;
    uint8_t xon2;
    uint8_t xoff1;
    uint8_t xoff2;
    uint8_t scr;
    uint8_t ssr;

    /* RX FIFO */
    uint8_t rx_fifo[CALYPSO_UART_RX_FIFO_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;

    /* IRQ output level tracking */
    bool irq_level;

    /* TX empty fires once per THR transition */
    bool thr_empty_pending;

} CalypsoUARTState;

/* Char backend callbacks */
int calypso_uart_can_receive(void *opaque);
void calypso_uart_receive(void *opaque, const uint8_t *buf, int size);

#endif /* HW_CHAR_CALYPSO_UART_H */
