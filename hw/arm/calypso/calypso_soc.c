/*
 * Calypso SoC - TI Calypso DBB (Digital Baseband)
 * DEBUG BUILD — verbose memory map logging
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

/* Global references for TDMA tick to kick UARTs */
CalypsoUARTState *g_uart_modem;
CalypsoUARTState *g_uart_irda;
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

/* Firmware perspective: UART_IRDA=0 → 0xFFFF5000, UART_MODEM=1 → 0xFFFF5800.
 * Sercomm (L1CTL) is on UART_MODEM (0xFFFF5800). */
#define CALYPSO_UART_IRDA     0xFFFF5000
#define CALYPSO_UART_MODEM    0xFFFF5800

/* ---- IRQ numbers ---- */
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

/* ---- CNTL register (EXTRA_CONF) at 0xFFFFFD00 ----
 * Bit [8:9] = bootrom mapping control
 *   When cleared (0), IRAM is aliased at 0x00000000
 *   When set (3), internal ROM is mapped at 0x00000000
 *
 * On real Calypso, firmware calls calypso_bootrom(0) to disable
 * bootrom and enable IRAM at address 0 for exception vectors.
 */
static uint64_t calypso_cntl_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset == 0) return s->extra_conf;
    return 0;
}

static void calypso_cntl_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    CalypsoSoCState *s = CALYPSO_SOC(opaque);
    if (offset != 0) return;

    s->extra_conf = (uint16_t)value;

    /* Bits [9:8] control bootrom/IRAM mapping at address 0 */
    bool bootrom_enabled = (value >> 8) & 3;

    if (!bootrom_enabled && !s->iram_at_zero) {
        /* Map IRAM at address 0 (higher priority than flash) */
        MemoryRegion *sysmem = get_system_memory();
        memory_region_init_alias(&s->iram_alias, OBJECT(s),
                                  "calypso.iram_at_zero",
                                  &s->iram, 0, CALYPSO_IRAM_SIZE);
        memory_region_add_subregion_overlap(sysmem, 0x00000000,
                                             &s->iram_alias, 1);
        s->iram_at_zero = true;
        fprintf(stderr, "[SOC] CNTL: IRAM aliased at 0x00000000 (bootrom disabled)\n");
    } else if (bootrom_enabled && s->iram_at_zero) {
        /* Remove IRAM alias */
        memory_region_del_subregion(get_system_memory(), &s->iram_alias);
        object_unparent(OBJECT(&s->iram_alias));
        s->iram_at_zero = false;
        fprintf(stderr, "[SOC] CNTL: IRAM alias removed (bootrom enabled)\n");
    }
}

static const MemoryRegionOps calypso_cntl_ops = {
    .read = calypso_cntl_read,
    .write = calypso_cntl_write,
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
    fprintf(stderr, "[SOC] stub '%s' @ 0x%08lx (0x100)\n", name, (unsigned long)base);
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

    /* ---- IRAM at 0x00800000 ONLY ----
     * NO alias at 0x00000000 — flash lives there (board-level).
     */
    memory_region_init_ram(&s->iram, OBJECT(dev), "calypso.iram",
                           CALYPSO_IRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_IRAM_BASE, &s->iram);
    fprintf(stderr, "[SOC] IRAM @ 0x%08x (%d KiB) — NO alias at 0x00000000\n",
            CALYPSO_IRAM_BASE, CALYPSO_IRAM_SIZE / 1024);

    /* ---- INTH ---- */
    object_initialize_child(OBJECT(dev), "inth", &s->inth, TYPE_CALYPSO_INTH);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->inth), &err)) {
        error_propagate(errp, err); return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->inth), 0, CALYPSO_INTH_BASE);

    /* Pass INTH's output IRQs (parent_irq, parent_fiq) through
     * the SoC device so the board can connect them to the CPU.
     * This avoids the ordering issue where sysbus_connect_irq
     * captures a NULL qemu_irq before the board connects it. */
    sysbus_pass_irq(sbd, SYS_BUS_DEVICE(&s->inth));

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

    /* ---- UART MODEM ---- */
    {
        Chardev *chr = qemu_chr_find("modem");
        if (!chr) chr = serial_hd(0);

        fprintf(stderr, "[SOC] UART modem: chardev → %s\n",
                chr ? (chr->label ? chr->label : "(no label)") : "NULL");

        object_initialize_child(OBJECT(dev), "uart-modem",
                                &s->uart_modem, TYPE_CALYPSO_UART);
        qdev_prop_set_string(DEVICE(&s->uart_modem), "label", "modem");

        if (chr) {
            qdev_prop_set_chr(DEVICE(&s->uart_modem), "chardev", chr);
        }

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart_modem), &err)) {
            error_propagate(errp, err); return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart_modem), 0, CALYPSO_UART_MODEM);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart_modem), 0,
                           INTH_IRQ(IRQ_UART_MODEM));
        g_uart_modem = &s->uart_modem;

        /* L1CTL socket DISABLED 2026-04-07 (revert):
         * L1CTL now flows over PTY/sercomm DLCI 5 via bridge.py, the
         * classical osmocon path. bridge.py owns /tmp/osmocom_l2_1
         * and wraps L1CTL frames into sercomm; sercomm_gate.c on the
         * QEMU side parses them and pushes into the firmware UART RX. */
        /* l1ctl_sock_init(&s->uart_modem, "/tmp/osmocom_l2_1"); */
    }

    /* ---- UART IRDA ---- */
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
        g_uart_irda = &s->uart_irda;
    }

    /* ---- TRX bridge (pure hardware) ---- */
    {
        qemu_irq *irqs = g_new0(qemu_irq, CALYPSO_NUM_IRQS);
        for (int i = 0; i < CALYPSO_NUM_IRQS; i++)
            irqs[i] = INTH_IRQ(i);
        calypso_trx_init(sysmem, irqs);
    }

    #undef INTH_IRQ

    /* ---- Stubs ----
     *
     * IMPORTANT: NO stub at 0x00000300 ("calypso.low300")!
     * That address falls inside the flash range 0x00000000–0x003FFFFF
     * and would shadow pflash CFI queries → "Failed to initialize flash!"
     */
    add_stub(sysmem, "calypso.keypad",     CALYPSO_KEYPAD_BASE, &calypso_keypad_ops);
    add_stub(sysmem, "calypso.tmr6800",    0xFFFE6800, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.mmio_80xx",  0xFFFE8000, &calypso_mmio8_ops);
    add_stub(sysmem, "calypso.conf",       0xFFFEF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_98xx",  0xFFFF9800, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.dpll",       0xFFFFF000, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.rhea",       0xFFFFF900, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.clkm",       0xFFFFFB00, &calypso_mmio16_ops);
    add_stub(sysmem, "calypso.mmio_fcxx",  0xFFFFFC00, &calypso_mmio16_ops);
    /* CNTL (EXTRA_CONF) - controls IRAM-at-zero mapping */
    memory_region_init_io(&s->cntl_iomem, OBJECT(dev), &calypso_cntl_ops, s,
                          "calypso.cntl", 0x100);
    memory_region_add_subregion(sysmem, 0xFFFFFD00, &s->cntl_iomem);
    s->extra_conf = 0x0300; /* bootrom enabled at reset */
    s->iram_at_zero = false;
    add_stub(sysmem, "calypso.dio",        0xFFFFFF00, &calypso_mmio8_ops);
    /* NO calypso.low300 — it overlaps flash! */

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
