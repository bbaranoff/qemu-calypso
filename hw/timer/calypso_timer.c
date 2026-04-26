/*
 * calypso_timer.c — Calypso GP/Watchdog Timer
 *
 * 16-bit down-counter with auto-reload, prescaler, and IRQ.
 * Calypso base clock: 13 MHz. The silicon has a fixed /32 hardware
 * prescaler ahead of the user-visible PRESCALER field, so:
 *
 *   tick_freq = 13 MHz / (32 << PRESCALER)
 *
 * With PRESCALER=0 the timer ticks at 13e6/32 ≈ 406.25 kHz, which is
 * what osmocom-bb's check_lost_frame() expects (1875 ticks ≈ 4615 µs
 * = one TDMA frame). Without the fixed /32 the firmware sees thousands
 * of "LOST N!" because the timer wraps multiple times per frame.
 *
 * Register map (firmware uses byte access on CNTL, word access on LOAD/READ):
 *   0x00  CNTL  bit 0    = START
 *               bit 1    = AUTO_RELOAD
 *               bits 4:2 = PRESCALER (0..7) → user divider
 *               bit 5    = CLOCK_ENABLE   (timer ticks only when also START)
 *   0x02  LOAD  Reload value (16-bit)
 *   0x04  READ  Current count (16-bit, read-only)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "hw/arm/calypso/calypso_timer.h"

/* Layout matches osmocom-bb firmware (calypso/timer.c). The timer only
 * ticks when both START and CLOCK_ENABLE are set; hwtimer_read() polls
 * those exact bits before returning the count, so they MUST round-trip
 * through readb/writeb intact — otherwise the firmware sees the timer
 * as stopped and returns 0xFFFF, producing a constant "LOST 0!" stream
 * every TDMA frame because diff = 0xFFFF - 0xFFFF = 0. */
#define TIMER_CTRL_START         (1 << 0)
#define TIMER_CTRL_RELOAD        (1 << 1)
#define TIMER_CTRL_PRESCALER_SH  2
#define TIMER_CTRL_PRESCALER_MSK (0x7 << 2)
#define TIMER_CTRL_CLOCK_ENABLE  (1 << 5)

#define CALYPSO_BASE_CLK   13000000LL  /* 13 MHz */

static bool calypso_timer_should_run(CalypsoTimerState *s)
{
    return (s->ctrl & TIMER_CTRL_START) && (s->ctrl & TIMER_CTRL_CLOCK_ENABLE);
}

static void calypso_timer_recompute_tick(CalypsoTimerState *s)
{
    int prescaler = (s->ctrl & TIMER_CTRL_PRESCALER_MSK) >> TIMER_CTRL_PRESCALER_SH;
    /* Silicon has a fixed /32 hardware prescaler in front of PRESCALER. */
    int64_t divider = 32LL << prescaler;
    int64_t freq = CALYPSO_BASE_CLK / divider;
    if (freq <= 0) freq = 1;
    s->tick_ns = NANOSECONDS_PER_SECOND / freq;
}

/* Compute current count by interpolating virtual time elapsed since the
 * timer was (re)started — avoids scheduling one QEMU event per decrement,
 * which would coalesce on the QEMU virtual clock granularity and make the
 * effective tick rate roughly half of what the firmware expects. */
static uint16_t calypso_timer_current_count(CalypsoTimerState *s)
{
    if (!s->running) return s->count;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->epoch_ns;
    if (elapsed < 0) elapsed = 0;
    int64_t ticks = elapsed / s->tick_ns;
    int64_t period = (int64_t)s->load + 1;
    if (s->ctrl & TIMER_CTRL_RELOAD) {
        ticks %= period;
    } else if (ticks > s->load) {
        return 0;
    }
    return (uint16_t)(s->load - ticks);
}

static void calypso_timer_schedule_wrap(CalypsoTimerState *s)
{
    /* Schedule the next IRQ at the moment count would reach 0 from the
     * current virtual time. */
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint16_t cur = calypso_timer_current_count(s);
    int64_t ns_to_wrap = (int64_t)(cur + 1) * s->tick_ns;
    timer_mod(s->timer, now + ns_to_wrap);
}

static void calypso_timer_tick(void *opaque)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    if (!s->running) return;

    qemu_irq_raise(s->irq);

    if (s->ctrl & TIMER_CTRL_RELOAD) {
        /* Reanchor epoch to "now" so the next read sees count=load. */
        s->epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->count = s->load;
        timer_mod(s->timer, s->epoch_ns + (int64_t)(s->load + 1) * s->tick_ns);
    } else {
        s->running = false;
        s->count = 0;
    }
}

static void calypso_timer_start(CalypsoTimerState *s)
{
    if (s->load == 0) return;
    calypso_timer_recompute_tick(s);
    s->count = s->load;
    s->running = true;
    s->epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->timer, s->epoch_ns + (int64_t)(s->load + 1) * s->tick_ns);
}

/* ---- MMIO ---- */

static uint64_t calypso_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    switch (offset) {
    case 0x00: return s->ctrl;
    case 0x02: return s->load;
    case 0x04: return calypso_timer_current_count(s);
    default:   return 0;
    }
}

static void calypso_timer_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    switch (offset) {
    case 0x00: { /* CNTL — preserve all 8 bits the firmware writes */
        bool was_running = s->running;
        uint16_t old_ctrl = s->ctrl;
        s->ctrl = value & 0xFF;
        if (calypso_timer_should_run(s)) {
            if (!was_running) {
                calypso_timer_start(s);
            } else if ((old_ctrl & TIMER_CTRL_PRESCALER_MSK) !=
                       (s->ctrl & TIMER_CTRL_PRESCALER_MSK)) {
                /* prescaler changed mid-run — re-anchor at current count */
                s->count = calypso_timer_current_count(s);
                calypso_timer_recompute_tick(s);
                s->epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                              (int64_t)(s->load - s->count) * s->tick_ns;
                calypso_timer_schedule_wrap(s);
            }
        } else {
            s->count = calypso_timer_current_count(s);
            s->running = false;
            timer_del(s->timer);
        }
        break;
    }
    case 0x02: /* LOAD */
        s->load = value;
        break;
    }
}

static const MemoryRegionOps calypso_timer_ops = {
    .read = calypso_timer_read,
    .write = calypso_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
    .impl  = { .min_access_size = 1, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_timer_realize(DeviceState *dev, Error **errp)
{
    CalypsoTimerState *s = CALYPSO_TIMER(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_timer_ops, s,
                          "calypso-timer", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, calypso_timer_tick, s);
}

static void calypso_timer_reset(DeviceState *dev)
{
    CalypsoTimerState *s = CALYPSO_TIMER(dev);

    s->load = 0;
    s->count = 0;
    s->ctrl = 0;
    s->prescaler = 0;
    s->running = false;
    timer_del(s->timer);
}

static void calypso_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_timer_realize;
    device_class_set_legacy_reset(dc, calypso_timer_reset);
    dc->desc = "Calypso GP/Watchdog timer";
}

static const TypeInfo calypso_timer_info = {
    .name          = TYPE_CALYPSO_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoTimerState),
    .class_init    = calypso_timer_class_init,
};

static void calypso_timer_register_types(void)
{
    type_register_static(&calypso_timer_info);
}

type_init(calypso_timer_register_types)
