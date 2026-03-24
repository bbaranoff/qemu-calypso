/*
 * Calypso SoC - TI Calypso DBB (Digital Baseband)
 *
 * COMPLETE REWRITE with debug on every step.
 *
 * Chardev approach:
 *   qdev_prop_set_chr() BEFORE realize.
 *   calypso_uart_realize() sets handlers with correct opaque.
 *   NO late-binding. NO qemu_chr_fe_init in this file.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "hw/arm/calypso/calypso_soc.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "qemu/error-report.h"
#include "hw/arm/calypso/calypso_uart.h"

/* ---- Memory map ---- */
#define CALYPSO_IRAM_BASE     0x00800000
#define CALYPSO_IRAM_SIZE     (256 * 1024)

/* ---- Peripheral addresses ---- */
#define CALYPSO_INTH_BASE     0xFFFFFA00
#define CALYPSO_TIMER1_BASE   0xFFFE3800
#define CALYPSO_TIMER2_BASE   0xFFFE3C00
#define CALYPSO_SPI_BASE      0xFFFE3000
#define CALYPSO_KEYPAD_BASE   0xFFFE4800

/*
 * UART addresses — verified against TI Calypso datasheet:
 *   UART0 (IrDA)  = 0xFFFF5000
 *   UART1 (Modem) = 0xFFFF5800
 *
 * OsmocomBB firmware (loader.highram) uses UART1 for osmocon.
 */
#define CALYPSO_UART_IRDA     0xFFFF5000
#define CALYPSO_UART_MODEM    0xFFFF5800

/* ---- IRQ numbers (must match calypso_trx.h) ---- */
#define IRQ_TIMER1            1
#define IRQ_TIMER2            2
#define IRQ_UART_MODEM        7
#define IRQ_SPI               13
#define IRQ_UART_IRDA         18

/* ---- Stub MMIO ---- */

static uint64_t calypso_mmio8_read(void *o, hwaddr a, unsigned s) { return 0; }
static void calypso_mmio8_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_mmio8_ops = {
    .read = calypso_mmio8_read, .write = calypso_mmio8_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static uint64_t calypso_mmio16_read(void *o, hwaddr a, unsigned s) { return 0; }
static void calypso_mmio16_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_mmio16_ops = {
    .read = calypso_mmio16_read, .write = calypso_mmio16_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static uint64_t calypso_kp_read(void *o, hwaddr a, unsigned s) { return 0xFF; }
static void calypso_kp_write(void *o, hwaddr a, uint64_t v, unsigned s) {}
static const MemoryRegionOps calypso_keypad_ops = {
    .read = calypso_kp_read, .write = calypso_kp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void add_stub(MemoryRegion *sys, const char *name,
                     hwaddr base, const MemoryRegionOps *ops)
{
    MemoryRegion *mr = g_new(MemoryRegion, 1);
    memory_region_init_io(mr, NULL, ops, NULL, name, 0x100);
    memory_region_add_subregion(sys, base, mr);
}

/* ================================================================
 * SoC realize
 * ================================================================ */

static void calypso_soc_realize(DeviceState *dev, Error **errp)
{
    CalypsoSoCState *s = CALYPSO_SOC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    MemoryRegion *sysmem = get_system_memory();
    Error *err = NULL;

    fprintf(stderr, "[SOC] === calypso_soc_realize START ===\n");

    /* ---- IRAM ---- */
    memory_region_init_ram(&s->iram, OBJECT(dev), "calypso.iram",
                           CALYPSO_IRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_IRAM_BASE, &s->iram);

    memory_region_init_alias(&s->iram_alias, OBJECT(dev),
                             "calypso.iram.alias",
                             &s->iram, 0, CALYPSO_IRAM_SIZE);
    memory_region_add_subregion(sysmem, 0x00000000, &s->iram_alias);

    /* ---- INTH ---- */
    object_initialize_child(OBJECT(dev), "inth", &s->inth, TYPE_CALYPSO_INTH);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->inth), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->inth), 0, CALYPSO_INTH_BASE);

    sysbus_init_irq(sbd, &s->cpu_irq);
    sysbus_init_irq(sbd, &s->cpu_fiq);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->inth), 0, s->cpu_irq);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->inth), 1, s->cpu_fiq);

    #define INTH_IRQ(n) qdev_get_gpio_in(DEVICE(&s->inth), (n))

    /* ---- Timer 1 ---- */
    object_initialize_child(OBJECT(dev), "timer1", &s->timer1, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer1), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer1), 0, CALYPSO_TIMER1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer1), 0, INTH_IRQ(IRQ_TIMER1));

    /* ---- Timer 2 ---- */
    object_initialize_child(OBJECT(dev), "timer2", &s->timer2, TYPE_CALYPSO_TIMER);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer2), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer2), 0, CALYPSO_TIMER2_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer2), 0, INTH_IRQ(IRQ_TIMER2));

    /* ---- I2C stub ---- */
    DeviceState *i2c_dev = qdev_new("calypso-i2c");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(i2c_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(i2c_dev), 0, 0xFFFE1800);

    /* ---- SPI ---- */
    object_initialize_child(OBJECT(dev), "spi", &s->spi, TYPE_CALYPSO_SPI);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi), 0, CALYPSO_SPI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi), 0, INTH_IRQ(IRQ_SPI));

    /* ================================================================
     * UART MODEM @ 0xFFFF5800
     *
     * This is where osmocon talks. The chardev is bound via
     * qdev_prop_set_chr() BEFORE realize, so that
     * calypso_uart_realize() can set up handlers.
     * ================================================================ */
    {
        Chardev *chr = qemu_chr_find("modem");
        if (!chr) chr = serial_hd(0);

        fprintf(stderr, "[SOC] UART modem: chardev lookup → %s\n",
                chr ? (chr->label ? chr->label : "(no label)") : "NULL");

        object_initialize_child(OBJECT(dev), "uart-modem",
                                &s->uart_modem, TYPE_CALYPSO_UART);
        qdev_prop_set_string(DEVICE(&s->uart_modem), "label", "modem");

        if (chr) {
            qdev_prop_set_chr(DEVICE(&s->uart_modem), "chardev", chr);
            fprintf(stderr, "[SOC] UART modem: chardev SET to '%s'\n",
                    chr->label ? chr->label : "(no label)");
        } else {
            fprintf(stderr, "[SOC] UART modem: NO CHARDEV!\n");
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart_modem), &err)) {
            error_propagate(errp, err); return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart_modem), 0, CALYPSO_UART_MODEM);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart_modem), 0,
                           INTH_IRQ(IRQ_UART_MODEM));

        fprintf(stderr, "[SOC] UART modem: mapped @ 0x%08x, IRQ %d\n",
                CALYPSO_UART_MODEM, IRQ_UART_MODEM);
    }

    /* ================================================================
     * UART IRDA @ 0xFFFF5000 (optional, for debug console)
     * ================================================================ */
    {
        Chardev *chr = qemu_chr_find("irda");
        if (!chr) chr = serial_hd(1);

        object_initialize_child(OBJECT(dev), "uart-irda",
                                &s->uart_irda, TYPE_CALYPSO_UART);
        qdev_prop_set_string(DEVICE(&s->uart_irda), "label", "irda");

        if (chr) {
            qdev_prop_set_chr(DEVICE(&s->uart_irda), "chardev", chr);
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart_irda), &err)) {
            error_propagate(errp, err); return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart_irda), 0, CALYPSO_UART_IRDA);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart_irda), 0,
                           INTH_IRQ(IRQ_UART_IRDA));
    }

    /* ---- TRX bridge ---- */
    if (s->enable_trx) {
        qemu_irq *irqs = g_new0(qemu_irq, CALYPSO_NUM_IRQS);
        for (int i = 0; i < CALYPSO_NUM_IRQS; i++)
            irqs[i] = INTH_IRQ(i);
        calypso_trx_init(sysmem, irqs, s->trx_port);
    }

    #undef INTH_IRQ

    /* ---- Stubs ---- */
    add_stub(sysmem, "calypso.keypad",     CALYPSO_KEYPAD_BASE, &calypso_keypad_ops);
    add_stub(sysmem, "calypso.tmr6800",    0xFFFE6800, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.mmio_80xx",  0xFFFE8000, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.conf",       0xFFFEF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_98xx",  0xFFFF9800, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.dpll",       0xFFFFF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.rhea",       0xFFFFF900, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.clkm",       0xFFFFFB00, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_fcxx",  0xFFFFFC00, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.cntl",       0xFFFFFD00, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.dio",        0xFFFFFF00, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.low300",     0x00000300, &calypso_mmio16_ops);

    /* Catch-all (lowest priority) */
    {
        MemoryRegion *mr = g_new(MemoryRegion, 1);
        memory_region_init_io(mr, NULL, &calypso_mmio8_ops, NULL,
                              "calypso.catchall", 0x100000);
        memory_region_add_subregion_overlap(sysmem, 0xFFF00000, mr, -1);
    }

    fprintf(stderr, "[SOC] === calypso_soc_realize DONE ===\n");
}

/* ---- QOM boilerplate ---- */

static Property calypso_soc_properties[] = {
    DEFINE_PROP_BOOL("enable-trx", CalypsoSoCState, enable_trx, true),
    DEFINE_PROP_UINT16("trx-port", CalypsoSoCState, trx_port, 4729),
    DEFINE_PROP_END_OF_LIST(),
};

static void calypso_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = calypso_soc_realize;
    device_class_set_props(dc, calypso_soc_properties);
    dc->user_creatable = false;
}

static const TypeInfo calypso_soc_type_info = {
    .name          = TYPE_CALYPSO_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSoCState),
    .class_init    = calypso_soc_class_init,
};

static void calypso_soc_register_types(void)
{
    type_register_static(&calypso_soc_type_info);
}

type_init(calypso_soc_register_types)
