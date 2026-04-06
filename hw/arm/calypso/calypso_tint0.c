/*
 * calypso_tint0.c -- TINT0 master clock for Calypso GSM virtualization
 *
 * Emulates the C54x DSP Timer 0 as a QEMU virtual timer.
 * On real hardware, Timer 0 runs off the 13 MHz DSP clock divided by
 * (PRD+1)*(TDDR+1) to produce a 4.615 ms TDMA frame tick (TINT0).
 * TINT0 drives the entire Calypso timing: DSP frame processing,
 * TPU sync, ARM frame IRQ, and UART polling.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/irq.h"
#include "calypso_tint0.h"
#include "calypso_c54x.h"
#include "hw/core/cpu.h"

#define TINT0_LOG(fmt, ...) \
    fprintf(stderr, "[tint0] " fmt "\n", ##__VA_ARGS__)

/* calypso_trx.c implements the actual frame work (DSP run, IRQs, UART) */
extern void calypso_tint0_do_tick(uint32_t fn);

/* ---- State ---- */
static struct {
    QEMUTimer *timer;
    uint32_t   fn;
    bool       running;
    bool       tpu_en_pending;
} tint0;

/* ---- Timer callback (fires every 4.615ms) ---- */
static void tint0_tick_cb(void *opaque)
{
    tint0.fn = (tint0.fn + 1) % GSM_HYPERFRAME;

    /* Delegate frame work to calypso_trx */
    calypso_tint0_do_tick(tint0.fn);

    /* Re-arm timer */
    if (tint0.running) {
        timer_mod_ns(tint0.timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
    }

    /* Kick ARM CPU to process pending IRQs */
    qemu_notify_event();
}

/* ---- Public API ---- */

void calypso_tint0_start(void)
{
    if (tint0.running) return;

    if (!tint0.timer) {
        tint0.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tint0_tick_cb, NULL);
    }

    tint0.running = true;
    tint0.fn = 0;
    TINT0_LOG("started (period=%.3f ms, IFR bit %d, vec %d)",
              TINT0_PERIOD_NS / 1e6, TINT0_IFR_BIT, TINT0_VEC);
    timer_mod_ns(tint0.timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + TINT0_PERIOD_NS);
}

void calypso_tint0_tpu_en(void)
{
    tint0.tpu_en_pending = true;
}

bool calypso_tint0_tpu_en_pending(void)
{
    return tint0.tpu_en_pending;
}

void calypso_tint0_tpu_en_clear(void)
{
    tint0.tpu_en_pending = false;
}

uint32_t calypso_tint0_fn(void)
{
    return tint0.fn;
}

void calypso_tint0_set_fn(uint32_t fn)
{
    tint0.fn = fn % GSM_HYPERFRAME;
}

bool calypso_tint0_running(void)
{
    return tint0.running;
}
