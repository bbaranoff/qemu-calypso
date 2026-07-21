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

/* Domaine de temps du hwtimer. DOIT être le MÊME que le tdma_tick/frame-IRQ
 * (calypso_trx.c calypso_tdma_clock()), sinon le firmware mesure les ticks
 * hwtimer (une horloge) entre deux frame-IRQ (une AUTRE horloge) -> le ratio
 * 1875 ticks/trame ne tient pas -> "LOST N!" en continu, et icount ne peut RIEN
 * (le hwtimer sur REALTIME est immunisé à icount). Défaut VIRTUAL (single-domain,
 * verrouillé sous icount=auto) ; opt-in wall via la MÊME gate que le tick. */
static QEMUClockType calypso_timer_clock(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("CALYPSO_TDMA_REALTIME");
        cached = (e && *e == '1') ? 1 : 0;
    }
    return cached ? QEMU_CLOCK_REALTIME : QEMU_CLOCK_VIRTUAL;
}

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
    int64_t now = qemu_clock_get_ns(calypso_timer_clock());
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
    int64_t now = qemu_clock_get_ns(calypso_timer_clock());
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
        s->epoch_ns = qemu_clock_get_ns(calypso_timer_clock());
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
    s->epoch_ns = qemu_clock_get_ns(calypso_timer_clock());
    timer_mod(s->timer, s->epoch_ns + (int64_t)(s->load + 1) * s->tick_ns);
}

/* ---- Frame-locked lost-frame latch (timer #1 only) ----
 * Firmware check_lost_frame() (layer1/sync.c) reads timer #1 once per TDMA
 * frame-IRQ and expects the count to advance by exactly 1875 ticks between two
 * IRQs (tolérance ±1). Sous CALYPSO_TDMA_REALTIME=1 + -icount auto, l'instant
 * d'échantillonnage jitte (±~13 ticks) -> "LOST N!" en continu. On fige la
 * valeur READ de timer #1 sur une grille dérivée du FN : count(fn) = load -
 * (fn mod 4)*1875 ∈ {7499,5624,3749,1874}. Deux frames consécutives diffèrent
 * de 1875 pile -> zéro LOST ; un vrai saut (fn +k) donne k*1875 -> LOST légitime
 * conservé. timer #1 est le SEUL hwtimer lu par check_lost_frame(). */
#define LOST_TICKS_PER_TDMA 1875

static CalypsoTimerState *g_lost_timer;   /* timer #1 @ 0xFFFE3800 */

static bool calypso_lost_latch_enabled(void)
{
    static int cached = -1;   /* défaut ON ; opt-out CALYPSO_LOST_LATCH=0 */
    if (cached < 0) {
        const char *e = getenv("CALYPSO_LOST_LATCH");
        cached = (e && *e == '0') ? 0 : 1;
    }
    return cached;
}

void calypso_timer_register_lost(DeviceState *d)
{
    g_lost_timer = CALYPSO_TIMER(d);
}

void calypso_timer_lost_frame_tick(uint32_t fn)
{
    CalypsoTimerState *s = g_lost_timer;
    if (!s || !s->running || !calypso_lost_latch_enabled()) {
        return;
    }
    uint32_t period = (uint32_t)s->load + 1;                 /* 7500 */
    uint32_t step = ((uint64_t)fn * LOST_TICKS_PER_TDMA) % period;
    s->lost_latch_count = (uint16_t)(s->load - step);        /* down-counter */
    s->lost_latch_active = true;
}

/* ---- MMIO ---- */

static uint64_t calypso_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTimerState *s = CALYPSO_TIMER(opaque);

    {
        static int rd_count = 0;
        static int64_t prev_t_virt = 0;
        if (rd_count < 0) {  /* DISABLED — re-enable by setting >0 */
            uint16_t live = calypso_timer_current_count(s);
            int64_t now = qemu_clock_get_ns(calypso_timer_clock());
            int64_t dt = prev_t_virt ? (now - prev_t_virt) : 0;
            fprintf(stderr, "[timer] RD ts=%p off=0x%02x live=%u stored=%u "
                    "running=%d tick_ns=%" PRId64 " epoch=%" PRId64
                    " t_virt=%" PRId64 " dt=%" PRId64 " rd#=%d\n",
                    (void *)s, (unsigned)offset, live, s->count, s->running,
                    s->tick_ns, s->epoch_ns, now, dt, rd_count);
            prev_t_virt = now;
            rd_count++;
        }
    }

    switch (offset) {
    case 0x00: return s->ctrl;
    case 0x02: return s->load;
    case 0x04:
        if (s->lost_latch_active) {   /* frame-locked value for check_lost_frame() */
            return s->lost_latch_count;
        }
        return calypso_timer_current_count(s);
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
                s->epoch_ns = qemu_clock_get_ns(calypso_timer_clock()) -
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

    s->timer = timer_new_ns(calypso_timer_clock(), calypso_timer_tick, s);
}

static void calypso_timer_reset(DeviceState *dev)
{
    CalypsoTimerState *s = CALYPSO_TIMER(dev);

    s->load = 0;
    s->count = 0;
    s->ctrl = 0;
    s->prescaler = 0;
    s->running = false;
    s->lost_latch_active = false;
    s->lost_latch_count = 0;
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
