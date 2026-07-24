/*
 * calypso_full_pcb.c — Calypso PCB-level orchestrator (locks + async log)
 *
 * Provides shared mutexes used by the autonomous components (TRX, BSP,
 * DSP, FBSB) for cross-thread DARAM/API RAM access, plus an async log
 * queue that defers fprintf off the TCG main thread.
 *
 * Does NOT own any timer or clock. Timing (TDMA tick, BSP drain, etc.)
 * is owned by the respective .c file (calypso_trx.c, calypso_bsp.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/log.h"
#include "qemu/atomic.h"
#include "hw/irq.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"

#include "calypso_full_pcb.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "hw/arm/calypso/calypso_debug.h"
#define PCB_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("PCB")) \
        fprintf(stderr, "[pcb] " fmt "\n", ##__VA_ARGS__); } while (0)

/* === Shared locks (extern in .h) ========================================= */
QemuMutex calypso_pcb_daram_lock;
QemuMutex calypso_pcb_api_ram_lock;
QemuMutex calypso_pcb_sim_lock;
QemuMutex calypso_pcb_bsp_q_lock;
QemuMutex calypso_pcb_tpu_lock;

/* === PCB state =========================================================== */
struct CalypsoPcb {
    qemu_irq inth_inputs[CALYPSO_IRQ_MAX];
    uint64_t irq_raised[CALYPSO_IRQ_MAX];
    uint64_t irq_lowered[CALYPSO_IRQ_MAX];
};

static CalypsoPcb *g_pcb = NULL;

/* === Async log queue =====================================================
 * High-freq fprintf sites from ARM TCG main thread enqueue here, a
 * dedicated drain thread writes to stderr. Avoids stdio-lock+write
 * blocking TCG (~10-100µs per inline fprintf cumulating to jitter). */
#define ASYNC_LOG_QSIZE  4096
#define ASYNC_LOG_MAXMSG 256
typedef struct { char msg[ASYNC_LOG_MAXMSG]; } AsyncLogEntry;

static AsyncLogEntry async_log_q[ASYNC_LOG_QSIZE];
static unsigned     async_log_head = 0;
static unsigned     async_log_tail = 0;
static unsigned     async_log_dropped = 0;
static QemuMutex    async_log_lock;
static QemuCond     async_log_cond;
static QemuThread   async_log_thread;
static bool         async_log_inited = false;
static bool         async_log_stop = false;

static void *async_log_drain_fn(void *unused)
{
    (void)unused;
    qemu_mutex_lock(&async_log_lock);
    while (!async_log_stop) {
        while (async_log_head == async_log_tail && !async_log_stop) {
            qemu_cond_wait(&async_log_cond, &async_log_lock);
        }
        while (async_log_head != async_log_tail) {
            char msg[ASYNC_LOG_MAXMSG];
            memcpy(msg, async_log_q[async_log_head].msg, ASYNC_LOG_MAXMSG);
            async_log_head = (async_log_head + 1) % ASYNC_LOG_QSIZE;
            unsigned dropped_snap = async_log_dropped;
            async_log_dropped = 0;
            qemu_mutex_unlock(&async_log_lock);
            fputs(msg, stderr);
            if (dropped_snap > 0) {
                fprintf(stderr, "[pcb] async_log dropped %u msgs (queue full)\n",
                        dropped_snap);
            }
            qemu_mutex_lock(&async_log_lock);
        }
    }
    qemu_mutex_unlock(&async_log_lock);
    return NULL;
}

static void async_log_init(void)
{
    if (async_log_inited) return;
    qemu_mutex_init(&async_log_lock);
    qemu_cond_init(&async_log_cond);
    async_log_inited = true;
    qemu_thread_create(&async_log_thread, "cal-asynclog",
                       async_log_drain_fn, NULL, QEMU_THREAD_JOINABLE);
    PCB_LOG("async log queue armed (size=%d)", ASYNC_LOG_QSIZE);
}

void calypso_async_log(const char *fmt, ...)
{
    if (!async_log_inited) {
        async_log_init();
    }
    char buf[ASYNC_LOG_MAXMSG];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;

    qemu_mutex_lock(&async_log_lock);
    unsigned next = (async_log_tail + 1) % ASYNC_LOG_QSIZE;
    if (next == async_log_head) {
        async_log_dropped++;
    } else {
        memcpy(async_log_q[async_log_tail].msg, buf, ASYNC_LOG_MAXMSG);
        async_log_tail = next;
        qemu_cond_signal(&async_log_cond);
    }
    qemu_mutex_unlock(&async_log_lock);
}

/* === Init ================================================================ */
CalypsoPcb *calypso_pcb_init(qemu_irq *inth_inputs)
{
    if (g_pcb) {
        PCB_LOG("WARN: calypso_pcb_init called twice — returning existing");
        return g_pcb;
    }

    g_pcb = g_new0(CalypsoPcb, 1);

    async_log_init();

    qemu_mutex_init(&calypso_pcb_daram_lock);
    qemu_mutex_init(&calypso_pcb_api_ram_lock);
    qemu_mutex_init(&calypso_pcb_sim_lock);
    qemu_mutex_init(&calypso_pcb_bsp_q_lock);
    qemu_mutex_init(&calypso_pcb_tpu_lock);

    if (inth_inputs) {
        for (int i = 0; i < CALYPSO_IRQ_MAX; i++) {
            g_pcb->inth_inputs[i] = inth_inputs[i];
        }
    }

    PCB_LOG("init OK (locks armed, IRQ table %s)",
            inth_inputs ? "copied from INTH" : "none");
    return g_pcb;
}

/* No-op kept for API compat with calypso_soc.c. */
void calypso_pcb_start_threads(CalypsoPcb *pcb) { (void)pcb; }
void calypso_pcb_stop_threads(CalypsoPcb *pcb)  { (void)pcb; }

/* === DARAM cross-thread helpers ========================================== */
#include "calypso_c54x.h"

uint16_t calypso_dsp_daram_read(void *dsp_void, uint16_t addr)
{
    C54xState *dsp = (C54xState *)dsp_void;
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = dsp->data[addr];
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    return v;
}

void calypso_dsp_daram_write(void *dsp_void, uint16_t addr, uint16_t val)
{
    C54xState *dsp = (C54xState *)dsp_void;
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    dsp->data[addr] = val;
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

void calypso_pcb_daram_lock_acquire(void)
{
    qemu_mutex_lock(&calypso_pcb_daram_lock);
}

void calypso_pcb_daram_lock_release(void)
{
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

/* === IRQ helpers ========================================================= */
void calypso_pcb_raise_irq(CalypsoPcb *pcb, int irq_nr)
{
    if (!pcb || irq_nr < 0 || irq_nr >= CALYPSO_IRQ_MAX) return;
    qatomic_inc(&pcb->irq_raised[irq_nr]);
    if (pcb->inth_inputs[irq_nr]) {
        qemu_irq_raise(pcb->inth_inputs[irq_nr]);
    }
}

void calypso_pcb_lower_irq(CalypsoPcb *pcb, int irq_nr)
{
    if (!pcb || irq_nr < 0 || irq_nr >= CALYPSO_IRQ_MAX) return;
    qatomic_inc(&pcb->irq_lowered[irq_nr]);
    if (pcb->inth_inputs[irq_nr]) {
        qemu_irq_lower(pcb->inth_inputs[irq_nr]);
    }
}
