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
/* orch CLK→BTS is now driven from TINT0 (2× rate via internal half-tick). */

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

    /* No forced page tic-toc here: the DSP itself writes d_dsp_page
     * each frame (PC=0xf321 / 0xf5ec) — the trx api hook mirrors the
     * value into ARM space. We let the firmware drive the toggle. */

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
    /* Do NOT force tint0.fn = 0 here. A real GSM BTS never restarts the
     * frame counter at 0 — it only ever advances. Resetting on every
     * TINT0 start makes the firmware believe it just synchronized to a
     * fresh hyperframe each run, which is the "fn injection" hack the
     * user flagged 2026-04-07 night. Whoever owns the master clock
     * (calypso_tint0_set_fn from a network-derived source) should seed
     * fn before calling start; otherwise it inherits whatever value the
     * static struct holds (0 on first boot only). */
    TINT0_LOG("started (period=%.3f ms, IFR bit %d, vec %d) fn=%u",
              TINT0_PERIOD_NS / 1e6, TINT0_IFR_BIT, TINT0_VEC, tint0.fn);
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
