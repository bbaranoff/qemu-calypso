/*
 * calypso_uart.c — Calypso UART (NS16550-like)
 *
 * REWRITE: Maximum debug, minimum complexity.
 * NO MDR1 gating — bytes always flow through.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "chardev/char-fe.h"
#include "qemu/log.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/arm/calypso/calypso_uart.h"

/* Register offsets */
#define REG_RBR_THR   0x00
#define REG_IER       0x01
#define REG_IIR_FCR   0x02
#define REG_LCR       0x03
#define REG_MCR       0x04
#define REG_LSR       0x05
#define REG_MSR       0x06
#define REG_SPR       0x07
#define REG_MDR1      0x08

/* IER bits */
#define IER_RX_DATA   (1 << 0)
#define IER_TX_EMPTY  (1 << 1)
#define IER_RX_LINE   (1 << 2)

/* IIR values */
#define IIR_NO_INT    0x01
#define IIR_RX_LINE   0x06
#define IIR_RX_DATA   0x04
#define IIR_TX_EMPTY  0x02

/* LCR bits */
#define LCR_DLAB      (1 << 7)

/* LSR bits */
#define LSR_DR        (1 << 0)
#define LSR_OE        (1 << 1)
#define LSR_THRE      (1 << 5)
#define LSR_TEMT      (1 << 6)

/* MSR bits */
#define MSR_CTS       (1 << 4)
#define MSR_DSR       (1 << 5)
#define MSR_DCD       (1 << 7)

/* FCR bits */
#define FCR_RX_RESET  (1 << 1)

/* ---- FIFO ---- */

static void fifo_push(CalypsoUARTState *s, uint8_t data)
{
    if (s->rx_count >= CALYPSO_UART_RX_FIFO_SIZE) {
        s->lsr |= LSR_OE;
        return;
    }
    s->rx_fifo[s->rx_head] = data;
    s->rx_head = (s->rx_head + 1) % CALYPSO_UART_RX_FIFO_SIZE;
    s->rx_count++;
}

static uint8_t fifo_pop(CalypsoUARTState *s)
{
    uint8_t data;
    if (s->rx_count == 0) return 0;
    data = s->rx_fifo[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % CALYPSO_UART_RX_FIFO_SIZE;
    s->rx_count--;
    return data;
}

static void fifo_reset(CalypsoUARTState *s)
{
    s->rx_head = s->rx_tail = s->rx_count = 0;
}

/* ---- IRQ ---- */

static void calypso_uart_update_irq(CalypsoUARTState *s)
{
    uint8_t iir = IIR_NO_INT;
    bool want = false;

    if ((s->ier & IER_RX_LINE) && (s->lsr & LSR_OE)) {
        iir = IIR_RX_LINE; want = true;
    } else if ((s->ier & IER_RX_DATA) && (s->lsr & LSR_DR)) {
        iir = IIR_RX_DATA; want = true;
    } else if ((s->ier & IER_TX_EMPTY) && s->thr_empty_pending) {
        iir = IIR_TX_EMPTY; want = true;
    }
    s->iir = iir;

    if (want && !s->irq_level) {
        s->irq_level = true;
        qemu_irq_raise(s->irq);
    } else if (!want && s->irq_level) {
        s->irq_level = false;
        qemu_irq_lower(s->irq);
    }
}

/* ---- Chardev callbacks ---- */

int calypso_uart_can_receive(void *opaque)
{
    CalypsoUARTState *s = (CalypsoUARTState *)opaque;
    return CALYPSO_UART_RX_FIFO_SIZE - s->rx_count;
}

void calypso_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    CalypsoUARTState *s = (CalypsoUARTState *)opaque;

    fprintf(stderr, "[UART:%s] <<<RX %d bytes from host:",
            s->label ? s->label : "?", size);
    for (int i = 0; i < size && i < 8; i++)
        fprintf(stderr, " %02x", buf[i]);
    fprintf(stderr, "\n");

    for (int i = 0; i < size; i++)
        fifo_push(s, buf[i]);

    if (s->rx_count > 0) s->lsr |= LSR_DR;
    calypso_uart_update_irq(s);
}

/* ---- MMIO ---- */

static uint64_t calypso_uart_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoUARTState *s = CALYPSO_UART(opaque);
    uint64_t val = 0;

    switch (offset) {
    case REG_RBR_THR:
        if (s->lcr & LCR_DLAB) {
            val = s->dll;
        } else {
            val = fifo_pop(s);
            s->lsr = (s->rx_count > 0) ? (s->lsr | LSR_DR) : (s->lsr & ~LSR_DR);
            calypso_uart_update_irq(s);
        }
        break;
    case REG_IER:
        val = (s->lcr & LCR_DLAB) ? s->dlh : s->ier;
        break;
    case REG_IIR_FCR:
        val = s->iir;
        if ((s->iir & 0x0F) == IIR_TX_EMPTY) {
            s->thr_empty_pending = false;
            calypso_uart_update_irq(s);
        }
        break;
    case REG_LCR: val = s->lcr; break;
    case REG_MCR: val = s->mcr; break;
    case REG_LSR:
        val = s->lsr;
        s->lsr &= ~LSR_OE;
        break;
    case REG_MSR: val = MSR_CTS | MSR_DSR | MSR_DCD; break;
    case REG_SPR: val = s->spr; break;
    case REG_MDR1: val = s->mdr1; break;
    default: break;
    }
    return val;
}

static void calypso_uart_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    CalypsoUARTState *s = CALYPSO_UART(opaque);

    switch (offset) {
    case REG_RBR_THR:
        if (s->lcr & LCR_DLAB) {
            s->dll = value;
        } else {
            uint8_t ch = (uint8_t)value;
            fprintf(stderr, "[UART:%s] TX>>> 0x%02x '%c'\n",
                    s->label ? s->label : "?", ch,
                    (ch >= 0x20 && ch < 0x7f) ? ch : '.');
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
            s->lsr |= LSR_THRE | LSR_TEMT;
            s->thr_empty_pending = true;
            calypso_uart_update_irq(s);
        }
        break;
    case REG_IER:
        if (s->lcr & LCR_DLAB) {
            s->dlh = value;
        } else {
            uint8_t old = s->ier;
            s->ier = value & 0x0F;
            if (!(old & IER_TX_EMPTY) && (s->ier & IER_TX_EMPTY)
                && (s->lsr & LSR_THRE)) {
                s->thr_empty_pending = true;
            }
            calypso_uart_update_irq(s);
        }
        break;
    case REG_IIR_FCR:
        s->fcr = value;
        if (value & FCR_RX_RESET) {
            fifo_reset(s);
            s->lsr &= ~LSR_DR;
        }
        break;
    case REG_LCR: s->lcr = value; break;
    case REG_MCR: s->mcr = value; break;
    case REG_SPR: s->spr = value; break;
    case REG_MDR1:
        s->mdr1 = value;
        fprintf(stderr, "[UART:%s] MDR1=0x%02x\n",
                s->label ? s->label : "?", (unsigned)value);
        break;
    default: break;
    }
}

static const MemoryRegionOps calypso_uart_ops = {
    .read = calypso_uart_read,
    .write = calypso_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- QOM ---- */

static void calypso_uart_realize(DeviceState *dev, Error **errp)
{
    CalypsoUARTState *s = CALYPSO_UART(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_uart_ops, s,
                          "calypso-uart", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    bool connected = qemu_chr_fe_backend_connected(&s->chr);

    fprintf(stderr, "[UART:%s] realize: chardev %s\n",
            s->label ? s->label : "?",
            connected ? "CONNECTED" : "NONE");

    if (connected) {
        qemu_chr_fe_set_handlers(&s->chr,
                                 calypso_uart_can_receive,
                                 calypso_uart_receive,
                                 NULL, NULL,
                                 s,      /* opaque = THIS UART */
                                 NULL, true);
        fprintf(stderr, "[UART:%s] handlers installed, opaque=%p\n",
                s->label ? s->label : "?", (void *)s);
    }
}

static void calypso_uart_reset_state(DeviceState *dev)
{
    CalypsoUARTState *s = CALYPSO_UART(dev);
    s->ier = 0;
    s->iir = IIR_NO_INT;
    s->fcr = 0;
    s->lcr = 0;
    s->mcr = 0;
    s->lsr = LSR_THRE | LSR_TEMT;
    s->msr = MSR_CTS | MSR_DSR | MSR_DCD;
    s->spr = 0;
    s->dll = 0;
    s->dlh = 0;
    s->mdr1 = 0;  /* ENABLED at reset — no gating */
    s->irq_level = false;
    s->thr_empty_pending = false;
    fifo_reset(s);
}

static Property calypso_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", CalypsoUARTState, chr),
    DEFINE_PROP_STRING("label", CalypsoUARTState, label),
    DEFINE_PROP_END_OF_LIST(),
};

static void calypso_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = calypso_uart_realize;
    device_class_set_legacy_reset(dc, calypso_uart_reset_state);
    dc->desc = "Calypso UART (NS16550-like)";
    device_class_set_props(dc, calypso_uart_properties);
}

static const TypeInfo calypso_uart_info = {
    .name          = TYPE_CALYPSO_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoUARTState),
    .class_init    = calypso_uart_class_init,
};

static void calypso_uart_register_types(void)
{
    type_register_static(&calypso_uart_info);
}

type_init(calypso_uart_register_types)
