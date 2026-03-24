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
 * Minimal RX FIFO
 * (Compal download protocol can burst multiple bytes)
 */
#define CALYPSO_UART_RX_FIFO_SIZE 16

typedef struct CalypsoUARTState {
    SysBusDevice parent_obj;

    /* MMIO */
    MemoryRegion iomem;

    /* QEMU backend */
    CharBackend chr;
    qemu_irq irq;

    /* Debug label ("modem", "irda") */
    char *label;

    /* Registers */
    uint8_t rbr;
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

    /* RX FIFO */
    uint8_t rx_fifo[CALYPSO_UART_RX_FIFO_SIZE];
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;

    /* IRQ output level tracking (avoid redundant transitions) */
    bool irq_level;

    /* TX empty fires only once per THR transition (16550 behavior) */
    bool thr_empty_pending;

} CalypsoUARTState;


/* Char backend callbacks (must be non-static in .c) */
int  calypso_uart_can_receive(void *opaque);
void calypso_uart_receive(void *opaque,
                          const uint8_t *buf,
                          int size);

#endif /* HW_CHAR_CALYPSO_UART_H */
