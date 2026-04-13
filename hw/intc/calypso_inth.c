/*
 * calypso_inth.c — Calypso INTH (Interrupt Handler)
 *
 * Level-sensitive interrupt controller at 0xFFFFFA00.
 * 32 IRQ inputs, priority-based arbitration, IRQ/FIQ routing via ILR.
 *
 * The Calypso INTH is LEVEL-SENSITIVE: it tracks the current level of
 * each input line. When a peripheral deasserts its IRQ (e.g. UART clears
 * TX_EMPTY by reading IIR), the INTH immediately sees the change.
 *
 * Simplified model: no nesting, no irq_in_service blocking. The ARM CPU's
 * own CPSR I-bit prevents re-entry. We just present the highest-priority
 * active IRQ at all times. Edge-triggered sources (TPU_FRAME=4, TPU_PAGE=5)
 * are cleared on IRQ_NUM read.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "hw/arm/calypso/calypso_inth.h"

/* ---- Priority arbitration ---- */

static void calypso_inth_update(CalypsoINTHState *s)
{
    uint32_t active = s->levels & ~s->mask;
    int best_irq = -1;
    int best_prio = 0x7F;
    int is_fiq = 0;

    /* Round-robin scan within same priority: start from rr_start so that
     * after servicing IRQ N, the next scan begins at N+1. This prevents
     * IRQ4 (TPU_FRAME, fires every tick) from starving IRQ7 (UART). */
    for (int j = 0; j < CALYPSO_INTH_NUM_IRQS; j++) {
        int i = (s->rr_start + j) % CALYPSO_INTH_NUM_IRQS;
        if (active & (1u << i)) {
            int prio = s->ilr[i] & 0x1F;
            if (prio < best_prio) {
                best_prio = prio;
                best_irq = i;
                is_fiq = (s->ilr[i] >> 8) & 1;
            }
        }
    }

    if (best_irq >= 0) {
        s->ith_v = best_irq;
        if (is_fiq) {
            qemu_irq_raise(s->parent_fiq);
            qemu_irq_lower(s->parent_irq);
        } else {
            qemu_irq_raise(s->parent_irq);
            qemu_irq_lower(s->parent_fiq);
        }
    } else {
        s->ith_v = 0;
        qemu_irq_lower(s->parent_irq);
        qemu_irq_lower(s->parent_fiq);
    }
}

/* ---- GPIO input handler (one per IRQ line) ---- */

static void calypso_inth_set_irq(void *opaque, int irq, int level)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    if (level) {
        s->levels |= (1u << irq);
    } else {
        s->levels &= ~(1u << irq);
    }

    calypso_inth_update(s);
}

/* ---- MMIO read/write ---- */

static uint64_t calypso_inth_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {
    case 0x00: /* IT_REG1 — active bits [15:0] */
        return s->levels & 0xFFFF;
    case 0x02: /* IT_REG2 — active bits [31:16] */
        return (s->levels >> 16) & 0xFFFF;
    case 0x08: /* MASK_IT_REG1 */
        return s->mask & 0xFFFF;
    case 0x0a: /* MASK_IT_REG2 */
        return (s->mask >> 16) & 0xFFFF;
    case 0x10: /* IRQ_NUM — read returns current highest-priority IRQ */
    case 0x80: /* IRQ_NUM (legacy) */
    {
        uint16_t num = s->ith_v;
        /* Clear level for edge-like sources (TPU_FRAME=4, TPU_PAGE=5, API=15).
         * These pulse once per event; clearing here prevents re-trigger
         * until the next event raises the line again. */
        if (num == 4 || num == 5 || num == 15) {
            s->levels &= ~(1u << num);
        }
        /* Re-evaluate immediately: if other active IRQs remain,
         * keep CPU IRQ line high so the firmware can chain ISRs
         * without returning to the main loop. */
        calypso_inth_update(s);
        {
            static uint32_t total = 0;
            static uint32_t irq7_count = 0;
            total++;
            if (num == 7) {
                irq7_count++;
                if (irq7_count <= 50 || (irq7_count % 100) == 0)
                    fprintf(stderr, "[INTH] IRQ7 dispatch #%u (total=%u) levels=0x%08x mask=0x%08x\n",
                            irq7_count, total, s->levels, s->mask);
            }
            if (total <= 20 || total == 100 || total == 500 || total == 1000)
                fprintf(stderr, "[INTH] IRQ_NUM=%u (#%u) levels=0x%08x mask=0x%08x\n",
                        num, total, s->levels, s->mask);
        }
        return num;
    }
    case 0x12: /* FIQ_NUM */
    case 0x82: /* FIQ_NUM (legacy) */
        return s->ith_v;
    case 0x14: /* IRQ_CTRL */
    case 0x84: /* IRQ_CTRL (legacy) */
        return 0;
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            return s->ilr[idx];
        }
        return 0;
    }
}

static void calypso_inth_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    CalypsoINTHState *s = CALYPSO_INTH(opaque);

    switch (offset) {
    case 0x08: /* MASK_IT_REG1 */
        s->mask = (s->mask & 0xFFFF0000) | (value & 0xFFFF);
        calypso_inth_update(s);
        break;
    case 0x0a: /* MASK_IT_REG2 */
        s->mask = (s->mask & 0x0000FFFF) | ((value & 0xFFFF) << 16);
        calypso_inth_update(s);
        break;
    case 0x14: /* IRQ_CTRL — end-of-service acknowledge */
    case 0x84:
    {
        /* Advance round-robin past the IRQ just serviced.
         * Only advance if the serviced IRQ was actually active
         * (not a spurious read of ith_v=0 when nothing was pending). */
        uint16_t svc = s->ith_v;
        if (svc > 0 || (s->levels & 1)) {
            /* Real IRQ was serviced — advance past it */
            s->rr_start = (svc + 1) % CALYPSO_INTH_NUM_IRQS;
        }
        calypso_inth_update(s);
        break;
    }
    default:
        if (offset >= 0x20 && offset < 0x60) {
            int idx = (offset - 0x20) / 2;
            s->ilr[idx] = value & 0x1FFF;
            /* Force UART (IRQ7) to same priority as TPU_FRAME (IRQ4).
             * Firmware sets IRQ7 to prio 31 which causes starvation. */
            if (idx == 7) {
                s->ilr[7] = (s->ilr[7] & ~0x1F) | (s->ilr[4] & 0x1F);
            }
        }
        break;
    }
}

static const MemoryRegionOps calypso_inth_ops = {
    .read = calypso_inth_read,
    .write = calypso_inth_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl  = { .min_access_size = 1, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_inth_realize(DeviceState *dev, Error **errp)
{
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_inth_ops, s,
                          "calypso-inth", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->parent_fiq);

    qdev_init_gpio_in(dev, calypso_inth_set_irq, CALYPSO_INTH_NUM_IRQS);
}

static void calypso_inth_reset(DeviceState *dev)
{
    CalypsoINTHState *s = CALYPSO_INTH(dev);

    s->levels = 0;
    s->mask = 0x00000000;
    s->ith_v = 0;
    s->irq_in_service = -1;
    s->rr_start = 0;
    memset(s->ilr, 0, sizeof(s->ilr));
}

static void calypso_inth_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_inth_realize;
    device_class_set_legacy_reset(dc, calypso_inth_reset);
    dc->desc = "Calypso INTH interrupt controller";
}

static const TypeInfo calypso_inth_info = {
    .name          = TYPE_CALYPSO_INTH,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoINTHState),
    .class_init    = calypso_inth_class_init,
};

static void calypso_inth_register_types(void)
{
    type_register_static(&calypso_inth_info);
}

type_init(calypso_inth_register_types)
