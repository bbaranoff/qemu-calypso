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
#include "hw/core/cpu.h"          /* current_cpu, CPUClass::get_pc — rxDone probe */
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/sysemu.h"
#include "hw/arm/calypso/calypso_soc.h"
#include "hw/arm/calypso/calypso_full_pcb.h"  /* PCB orchestrator init */
#include "hw/arm/calypso/calypso_trx.h"

/* Global references for TDMA tick to kick UART RX (both channels).
 * g_uart_irda must be polled too — under -icount, the per-UART REALTIME
 * rx_poll_timer fires at wall rate but CPU runs at virtual rate, so PTY
 * data backs up. Polling from tdma_tick (VIRTUAL clock) keeps IRDA RX
 * drained in lockstep with the rest of the emulator. */
CalypsoUARTState *g_uart_modem;
CalypsoUARTState *g_uart_irda;
#include "chardev/char-fe.h"
#include "chardev/char.h"
#include "qemu/error-report.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_debug.h"

/* ---- Memory map ---- */
#define CALYPSO_IRAM_BASE     0x00800000
#define CALYPSO_IRAM_SIZE     (256 * 1024)

/* ---- Peripheral addresses ---- */
#define CALYPSO_INTH_BASE     0xFFFFFA00
#define CALYPSO_TIMER1_BASE   0xFFFE3800
#define CALYPSO_TIMER2_BASE   0xFFFE3C00
#define CALYPSO_SPI_BASE      0xFFFE3000
#define CALYPSO_KEYPAD_BASE   0xFFFE4800

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

    /* CNTL_RST (offset 4) : registre reset firmware (RESET_DSP=bit1, RESET_ARM=bit0).
     * L'ému l'IGNORE actuellement (return ci-dessous sur offset!=0). DIAG GAP-0 :
     * logger les toggles assert/release du reset DSP → confirme que le firmware
     * atteint dsp_pre_boot et tente de piloter le boot DSP par le hardware. */
    if (offset == 4) {
        static unsigned rstw = 0;
        if (rstw < 40) { rstw++;
            fprintf(stderr, "[SOC] CNTL_RST #%u write val=0x%02x (RESET_DSP=%d RESET_ARM=%d)\n",
                    rstw, (unsigned)(value & 0xff),
                    (int)((value >> 1) & 1), (int)(value & 1));
        }
        /* OPTION 2 : câbler le reset hardware du DSP (RESET_DSP = bit 1) au c54x.
         * assert → tenir la ROM en reset ; release → boot frais synchronisé. */
        calypso_trx_dsp_hw_reset((int)((value >> 1) & 1));
    }

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
 * rxDoneFlag PROBE (env-gated CALYPSO_RXDONE_PROBE=1)
 * ================================================================
 * Overlay IO region 4 bytes @ 0x008302d4 (= rxDoneFlag, BSS firmware).
 * Intercepts reads/writes, logs PC+vtime+seqnum+value, forwards to backing.
 * Replicates ce que faisait `gdb watch *(unsigned int*)0x008302d4` + log,
 * sans gdb attaché (donc sans perturbation hbreak TCG).
 *
 * Caveat (c web 2026-05-27) : transforme cette 4-byte zone en MMIO,
 * perturbe le chemin d'accès TCG (slow-path au lieu de direct RAM load).
 * OK pour capturer la chronologie en un run ; ne pas laisser actif en prod.
 */
static uint32_t rxdone_probe_backing = 0;
static uint64_t rxdone_probe_seq = 0;

static uint64_t rxdone_probe_read(void *opaque, hwaddr addr, unsigned size)
{
    uint8_t *p = (uint8_t *)&rxdone_probe_backing;
    uint64_t val = 0;
    if (size == 4)      val = rxdone_probe_backing;
    else if (size == 2) val = *(uint16_t *)(p + addr);
    else                val = *(p + addr);
    /* RD log silenced by default (busy-poll = flood). Decommente si besoin. */
    /* fprintf(stderr, "[rxDone] RD +%lx size=%u = 0x%lx\n", addr, size, val); */
    return val;
}

static void rxdone_probe_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    uint8_t *p = (uint8_t *)&rxdone_probe_backing;
    uint32_t prev = rxdone_probe_backing;

    /* Guest PC at the write instant. cpu->cc->get_pc is the generic
     * CPUClass accessor (avoids target/arm/cpu.h include here). */
    uint64_t pc = 0;
    if (current_cpu && current_cpu->cc && current_cpu->cc->get_pc) {
        pc = current_cpu->cc->get_pc(current_cpu);
    }
    uint64_t vt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (size == 4)      rxdone_probe_backing = (uint32_t)val;
    else if (size == 2) *(uint16_t *)(p + addr) = (uint16_t)val;
    else                *(p + addr) = (uint8_t)val;

    rxdone_probe_seq++;
    fprintf(stderr,
            "[rxDone] #%llu WR +%lu size=%u val=0x%lx (was 0x%x) PC=0x%lx vt=%lu\n",
            (unsigned long long)rxdone_probe_seq,
            (unsigned long)addr, size,
            (unsigned long)val, prev,
            (unsigned long)pc,
            (unsigned long)vt);
}

static const MemoryRegionOps rxdone_probe_ops = {
    .read = rxdone_probe_read,
    .write = rxdone_probe_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static MemoryRegion rxdone_probe_mr;

static void rxdone_probe_install(MemoryRegion *sysmem, DeviceState *dev)
{
    const char *e = cdbg_env("RXDONE");
    if (!e || e[0] != '1') return;
    memory_region_init_io(&rxdone_probe_mr, OBJECT(dev), &rxdone_probe_ops,
                          NULL, "rxdone_probe", 4);
    /* Priority 1 > IRAM (priority 0) → ARM accès au 4-byte 0x008302d4
     * passe par cette IO region. Reads gdb (debug accès) idem. */
    memory_region_add_subregion_overlap(sysmem, 0x008302d4, &rxdone_probe_mr, 1);
    fprintf(stderr,
            "[rxDone] PROBE installed @ 0x008302d4 (overlay prio=1, "
            "covers 4 bytes rxDoneFlag)\n");
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

    /* rxDoneFlag debug probe — overlay IO sur 0x008302d4 si env activé.
     * Install APRÈS IRAM (subregion_overlap avec prio plus haute). */
    rxdone_probe_install(sysmem, dev);

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

        /* L1CTL socket: sercomm↔L1CTL relay for OsmocomBB mobile */
        {
            const char *l1ctl_path = getenv("L1CTL_SOCK");
            l1ctl_sock_init(&s->uart_modem, l1ctl_path ? l1ctl_path : "/tmp/osmocom_l2");
        }
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

    /* === PCB orchestrator init (threading PCB) ====================
     * Spawn les threads optionnels par puce/unité quand CALYPSO_PCB_THREADS=1
     * ou granulaire CALYPSO_PCB_THREAD_X=1 / CALYPSO_PCB_TICK_THREADS=1.
     * Sans wire ici, les gates dans calypso_trx.c / calypso_tint0.c
     * skip leur re-arm sans qu'aucun thread ne les remplace → ticks meurent. */
    {
        CalypsoPcb *pcb = calypso_pcb_init(NULL);
        if (pcb) {
            calypso_pcb_start_threads(pcb);
        }
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
