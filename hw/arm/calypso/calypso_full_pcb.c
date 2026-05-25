/*
 * calypso_full_pcb.c — Calypso PCB-level threading orchestrator
 *
 * Spawns one QemuThread per autonomous chip/unit on the Calypso PCB,
 * matching real silicon parallelism :
 *
 *                              ┌── IRQ_TPU_FRAME(4) ──┐
 *   tpu_thread ──tick TDMA─────┘                      ▼
 *       │                                       ARM thread
 *       │   ┌── IRQ_DMA(14) ──────────────────┐  (frame_irq,
 *       ▼   │   ┌── IRQ_API(15) ─────────────┤   l1s sched)
 *   bsp_thread ─┘   ┌── IRQ_SIMCARD(6) ─────┤
 *       │           │                         │
 *       ▼           ▼                         │
 *   DARAM       sim_thread                    │
 *   (locked)        │                         │
 *       ▲           ▼                         │
 *       │      sim FIFO + IT_WT               │
 *       │                                     │
 *   dsp_thread ◄────TPU "EN" signal──────────┘
 *       │
 *       ▼  write results to DARAM + raise IRQ_API
 *       │
 *       └── back to ARM ──┘
 *
 *                                    UART_MODEM(7) ◄── osmocon ◄── mobile L23
 *
 * Voir THREADING_TODO.md pour la doc complète.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/log.h"
#include "qemu/atomic.h"
#include "hw/irq.h"
#include "calypso_full_pcb.h"

#include "qemu/main-loop.h"  /* bql_lock / bql_unlock */
#include "qemu/timer.h"      /* QEMUTimer + QEMU_CLOCK_VIRTUAL */
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Public invokers depuis calypso_trx.c / calypso_tint0.c — appelés par les
 * pcb tick threads. Doivent être appelés avec BQL held. */
extern void calypso_trx_kick_invoke(void);
extern void calypso_trx_tdma_tick_invoke(void);
extern void calypso_trx_frame_irq_lower_invoke(void);
extern void calypso_tint0_tick_invoke(void);

#define PCB_LOG(fmt, ...) \
    fprintf(stderr, "[pcb] " fmt "\n", ##__VA_ARGS__)

/* === Shared locks (extern in .h) ========================================= */
QemuMutex calypso_pcb_daram_lock;
QemuMutex calypso_pcb_api_ram_lock;
QemuMutex calypso_pcb_sim_lock;
QemuMutex calypso_pcb_bsp_q_lock;
QemuMutex calypso_pcb_tpu_lock;

/* === PCB state =========================================================== */
struct CalypsoPcb {
    /* IRQ outputs vers INTH (one per IRQ source).
     * inth_inputs[CALYPSO_IRQ_X] est la qemu_irq line à .raise/.lower. */
    qemu_irq inth_inputs[CALYPSO_IRQ_MAX];

    /* Threads par composant. NULL = pas spawned (mode legacy). */
    QemuThread threads[CALYPSO_THREAD_MAX];
    bool       thread_running[CALYPSO_THREAD_MAX];
    bool       thread_enabled[CALYPSO_THREAD_MAX];

    /* Stop flag pour signal clean shutdown aux threads. Lu atomically. */
    bool stop;

    /* Stats IRQ (telemetry). */
    uint64_t irq_raised[CALYPSO_IRQ_MAX];
    uint64_t irq_lowered[CALYPSO_IRQ_MAX];
};

static CalypsoPcb *g_pcb = NULL;  /* singleton, set by calypso_pcb_init */

/* === Async log queue =====================================================
 * Tous les sites high-freq qui appelaient fprintf(stderr,...) inline depuis
 * ARM TCG main thread doivent passer par calypso_async_log() : enqueue +
 * un drain thread écrit vers stderr en arrière-plan.
 *
 * Sans ça : chaque fprintf bloque TCG sur stdio lock + write syscall
 * (~10-100µs). En cumul → jitter ARM. */
#define ASYNC_LOG_QSIZE  4096
#define ASYNC_LOG_MAXMSG 256
typedef struct { char msg[ASYNC_LOG_MAXMSG]; } AsyncLogEntry;

static AsyncLogEntry async_log_q[ASYNC_LOG_QSIZE];
static unsigned     async_log_head = 0;  /* drain reads here */
static unsigned     async_log_tail = 0;  /* writers append here */
static unsigned     async_log_dropped = 0;
static QemuMutex    async_log_lock;
static QemuCond     async_log_cond;
static QemuThread   async_log_thread;
static bool         async_log_inited = false;
static bool         async_log_stop = false;

static void *async_log_drain_fn(void *unused)
{
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

static void async_log_init(void);  /* fwd */

void calypso_async_log(const char *fmt, ...)
{
    /* Lazy init — premier appel arme la queue + drain thread. Idempotent
     * via async_log_inited check. Évite que calypso_pcb_init doive être
     * appelé avant les autres modules. */
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
        /* full — drop, count it */
        async_log_dropped++;
    } else {
        memcpy(async_log_q[async_log_tail].msg, buf, ASYNC_LOG_MAXMSG);
        async_log_tail = next;
        qemu_cond_signal(&async_log_cond);
    }
    qemu_mutex_unlock(&async_log_lock);
}

static void async_log_init(void)
{
    if (async_log_inited) return;
    qemu_mutex_init(&async_log_lock);
    qemu_cond_init(&async_log_cond);
    async_log_inited = true;
    qemu_thread_create(&async_log_thread, "cal-asynclog",
                       async_log_drain_fn, NULL, QEMU_THREAD_JOINABLE);
    PCB_LOG("async log queue armed (size=%d, drain thread started)",
            ASYNC_LOG_QSIZE);
}

/* === Init ================================================================ */
CalypsoPcb *calypso_pcb_init(qemu_irq *inth_inputs)
{
    if (g_pcb) {
        PCB_LOG("WARN: calypso_pcb_init called twice — returning existing");
        return g_pcb;
    }

    g_pcb = g_new0(CalypsoPcb, 1);

    /* Async log queue first — autres modules peuvent appeler calypso_async_log
     * dès leur init. Indépendant des thread flags. */
    async_log_init();

    /* Init locks (ordre canonique : daram < api_ram < sim < bsp_q < tpu). */
    qemu_mutex_init(&calypso_pcb_daram_lock);
    qemu_mutex_init(&calypso_pcb_api_ram_lock);
    qemu_mutex_init(&calypso_pcb_sim_lock);
    qemu_mutex_init(&calypso_pcb_bsp_q_lock);
    qemu_mutex_init(&calypso_pcb_tpu_lock);

    /* Copy IRQ inputs vers INTH (one per source). */
    if (inth_inputs) {
        for (int i = 0; i < CALYPSO_IRQ_MAX; i++) {
            g_pcb->inth_inputs[i] = inth_inputs[i];
        }
    }

    /* Init thread state — all OFF by default (legacy single-thread). */
    for (int i = 0; i < CALYPSO_THREAD_MAX; i++) {
        g_pcb->thread_running[i] = false;
        g_pcb->thread_enabled[i] = false;
    }

    /* Lire env pour décider quels threads activer. */
    const char *e_all = getenv("CALYPSO_PCB_THREADS");
    bool all_on = (e_all && e_all[0] == '1');

    const struct { CalypsoThreadId id; const char *env; const char *name; } map[] = {
        { CALYPSO_THREAD_SIM,  "CALYPSO_PCB_THREAD_SIM",  "sim"  },
        { CALYPSO_THREAD_BSP,  "CALYPSO_PCB_THREAD_BSP",  "bsp"  },
        { CALYPSO_THREAD_DSP,  "CALYPSO_PCB_THREAD_DSP",  "dsp"  },
        { CALYPSO_THREAD_TPU,  "CALYPSO_PCB_THREAD_TPU",  "tpu"  },
        { CALYPSO_THREAD_IOTA, "CALYPSO_PCB_THREAD_IOTA", "iota" },
    };
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
        const char *e = getenv(map[i].env);
        bool on = all_on || (e && e[0] == '1');
        g_pcb->thread_enabled[map[i].id] = on;
        if (on) PCB_LOG("thread enabled: %s", map[i].name);
    }

    PCB_LOG("init OK (locks armed, IRQ table copied %s)",
            inth_inputs ? "from INTH" : "(none — legacy)");
    return g_pcb;
}

/* === Thread spawn ======================================================== */
static void spawn_thread(CalypsoPcb *pcb, CalypsoThreadId id,
                         void *(*fn)(void*), const char *name)
{
    if (!pcb->thread_enabled[id]) return;
    qemu_thread_create(&pcb->threads[id], name, fn, pcb,
                       QEMU_THREAD_JOINABLE);
    pcb->thread_running[id] = true;
    PCB_LOG("spawned thread: %s", name);
}

/* Forward decl */
void calypso_pcb_start_tick_threads(CalypsoPcb *pcb);

void calypso_pcb_start_threads(CalypsoPcb *pcb)
{
    if (!pcb) return;
    /* Spawn ordre : Phase 1 (sim/iota) → Phase 3 (bsp) → Phase 2 (dsp) → Phase 4 (tpu).
     * Match l'ordre logique de THREADING_TODO.md (low risk → high impact). */
    spawn_thread(pcb, CALYPSO_THREAD_SIM,  calypso_pcb_sim_thread,  "cal-sim");
    spawn_thread(pcb, CALYPSO_THREAD_IOTA, calypso_pcb_iota_thread, "cal-iota");
    spawn_thread(pcb, CALYPSO_THREAD_BSP,  calypso_pcb_bsp_thread,  "cal-bsp");
    spawn_thread(pcb, CALYPSO_THREAD_DSP,  calypso_pcb_dsp_thread,  "cal-dsp");
    spawn_thread(pcb, CALYPSO_THREAD_TPU,  calypso_pcb_tpu_thread,  "cal-tpu");
    /* Tick threads (env CALYPSO_PCB_TICK_THREADS=1) — sortent les 4 ticks
     * périodiques du main TCG thread. */
    calypso_pcb_start_tick_threads(pcb);
}

void calypso_pcb_stop_threads(CalypsoPcb *pcb)
{
    if (!pcb) return;
    qatomic_set(&pcb->stop, true);
    for (int i = 0; i < CALYPSO_THREAD_MAX; i++) {
        if (pcb->thread_running[i]) {
            qemu_thread_join(&pcb->threads[i]);
            pcb->thread_running[i] = false;
        }
    }
    PCB_LOG("all threads joined");
}

/* === DARAM cross-thread helpers ==========================================
 *
 * Voir pcb.h pour la motivation. Ces fonctions encapsulent le mutex
 * daram_lock pour les sites hors calypso_c54x.c (qui a son propre wrapper
 * via data_read/data_write). */

/* Accès direct via le vrai type C54xState (data[] indexable). */
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

/* === Thread entry stubs ==================================================
 * Scaffolding — real bodies à implémenter dans les phases du TODO.
 * Pour l'instant, chaque stub :
 *   - log son arm
 *   - sleep + check stop flag (no-op loop)
 *   - log son exit
 * Quand on commence le rollout réel d'une phase, on remplace le stub
 * par un loop qui fait le vrai work (consommer FIFO SIM, pomper c54x_run,
 * etc.) avec les locks appropriés. */

static void *thread_stub(CalypsoPcb *pcb, const char *name)
{
    PCB_LOG("thread %s : start (stub, no real work yet)", name);
    while (!qatomic_read(&pcb->stop)) {
        g_usleep(50000);  /* 50 ms */
    }
    PCB_LOG("thread %s : exit", name);
    return NULL;
}

void *calypso_pcb_sim_thread(void *arg)  { return thread_stub(arg, "cal-sim");  }
void *calypso_pcb_iota_thread(void *arg) { return thread_stub(arg, "cal-iota"); }
void *calypso_pcb_bsp_thread(void *arg)  { return thread_stub(arg, "cal-bsp");  }
void *calypso_pcb_dsp_thread(void *arg)  { return thread_stub(arg, "cal-dsp");  }
void *calypso_pcb_tpu_thread(void *arg)  { return thread_stub(arg, "cal-tpu");  }

/* === Tick threads ========================================================
 * Self-paced threads pour les 4 ticks qui pollent l'ARM TCG main thread.
 * Chacun :
 *   1. usleep period_us
 *   2. bql_lock()  — needed pour shared state mutation safety
 *   3. invoke body (existant côté calypso_trx.c / calypso_tint0.c)
 *   4. bql_unlock()
 *
 * Gated par env CALYPSO_PCB_TICK_THREADS=1 (le côté trx/tint0 skip son
 * propre re-arm QEMUTimer pour éviter double-tick).
 *
 * Phase 5 MTTCG : le bql_lock dans le body limite la concurrence avec ARM
 * (sérialisé sur BQL). Pour vraie concurrence ARM↔tick, il faudrait
 * remplacer BQL par locks fins sur le state mutated (DARAM, IRQ lines,
 * registers). Voir THREADING_TODO.md Phase 5. */

/* periods en microsecondes */
#define PCB_TICK_TDMA_US    4615    /* 4.615 ms = TDMA frame */
#define PCB_TICK_TINT0_US   4615    /* TINT0 = TDMA frame rate */
#define PCB_TICK_KICK_US    5000    /* 5 ms wall = rxDoneFlag refresh */
#define PCB_TICK_FRAME_IRQ_US 1000  /* 1 ms = frame_irq lower re-arm */

/* Tick threads v2 (2026-05-25) — DÉTERMINISTES virtual-clock paced.
 *
 * Problème du v1 (g_usleep wall) : ticks fire en wall-clock, l'ordre vs
 * TCG slices dépendait du host scheduler → trajectoires firmware
 * différentes selon le run → IMR-W *ZERO* divergent (380k events run A,
 * 0 events run B avec même build/firmware).
 *
 * Fix v2 : un QEMUTimer (QEMU_CLOCK_VIRTUAL) côté TCG main thread arme
 * le tick à un instant virtual-clock. Son callback (sous BQL) signale
 * la condvar du pthread tick. Le pthread fait l'invoke() sous BQL.
 *
 * Limite assumée : sous TCG single-thread + BQL, le pthread ne gagne pas
 * en parallélisme (BQL serialize). Le gain est uniquement la
 * **déterminisme** d'ordre vs ARM TCG. Vrai speedup viendra Phase 2
 * (DSP thread + barrière TDMA). */

typedef struct {
    CalypsoPcb *pcb;
    const char *name;
    void      (*invoke)(void);
    unsigned    period_us;
    QEMUTimer  *timer;        /* virtual-clock timer; armed in TCG main */
    QemuMutex   mu;
    QemuCond    cond;
    bool        pending;      /* set by timer cb, cleared by pthread */
} PcbTickCtx;

static PcbTickCtx pcb_tick_ctx[4];

static void pcb_tick_timer_cb(void *opaque)
{
    PcbTickCtx *t = opaque;
    qemu_mutex_lock(&t->mu);
    t->pending = true;
    qemu_cond_signal(&t->cond);
    qemu_mutex_unlock(&t->mu);
    /* Re-arm at next virtual-clock instant. Period en ns. */
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(t->timer, now + (int64_t)t->period_us * 1000);
}

static void *pcb_tick_loop_v2(void *arg)
{
    PcbTickCtx *t = arg;
    PCB_LOG("tick thread %s : start (period=%uµs, virtual-clock paced)",
            t->name, t->period_us);
    while (!qatomic_read(&t->pcb->stop)) {
        qemu_mutex_lock(&t->mu);
        while (!t->pending && !qatomic_read(&t->pcb->stop)) {
            qemu_cond_wait(&t->cond, &t->mu);
        }
        t->pending = false;
        qemu_mutex_unlock(&t->mu);
        if (qatomic_read(&t->pcb->stop)) break;
        bql_lock();
        t->invoke();
        bql_unlock();
    }
    PCB_LOG("tick thread %s : exit", t->name);
    return NULL;
}

/* Spawn handles (statiques car le orchestrateur s'en charge). */
static QemuThread pcb_tick_threads[4];
static bool       pcb_tick_threads_running = false;

static void pcb_tick_init_one(int idx, const char *name, unsigned period_us,
                              void (*invoke)(void), CalypsoPcb *pcb)
{
    PcbTickCtx *t = &pcb_tick_ctx[idx];
    t->pcb       = pcb;
    t->name      = name;
    t->invoke    = invoke;
    t->period_us = period_us;
    t->pending   = false;
    qemu_mutex_init(&t->mu);
    qemu_cond_init(&t->cond);
    /* Timer must be created in TCG main thread context — calypso_pcb_start_
     * tick_threads runs from realize/init which IS the main thread. */
    t->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pcb_tick_timer_cb, t);
    /* Arm initial fire */
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(t->timer, now + (int64_t)period_us * 1000);
}

void calypso_pcb_start_tick_threads(CalypsoPcb *pcb)
{
    if (!pcb || pcb_tick_threads_running) return;
    const char *e = getenv("CALYPSO_PCB_TICK_THREADS");
    if (!e || e[0] != '1') {
        PCB_LOG("tick threads disabled (CALYPSO_PCB_TICK_THREADS!=1)");
        return;
    }
    /* Init ctx + arm virtual-clock timers BEFORE spawning pthreads, so the
     * first signal can't race with thread startup. */
    pcb_tick_init_one(0, "cal-tdma",  PCB_TICK_TDMA_US,
                      calypso_trx_tdma_tick_invoke,        pcb);
    pcb_tick_init_one(1, "cal-tint0", PCB_TICK_TINT0_US,
                      calypso_tint0_tick_invoke,           pcb);
    pcb_tick_init_one(2, "cal-kick",  PCB_TICK_KICK_US,
                      calypso_trx_kick_invoke,             pcb);
    pcb_tick_init_one(3, "cal-firq",  PCB_TICK_FRAME_IRQ_US,
                      calypso_trx_frame_irq_lower_invoke,  pcb);
    qemu_thread_create(&pcb_tick_threads[0], "cal-tdma",
                       pcb_tick_loop_v2, &pcb_tick_ctx[0], QEMU_THREAD_JOINABLE);
    qemu_thread_create(&pcb_tick_threads[1], "cal-tint0",
                       pcb_tick_loop_v2, &pcb_tick_ctx[1], QEMU_THREAD_JOINABLE);
    qemu_thread_create(&pcb_tick_threads[2], "cal-kick",
                       pcb_tick_loop_v2, &pcb_tick_ctx[2], QEMU_THREAD_JOINABLE);
    qemu_thread_create(&pcb_tick_threads[3], "cal-firq",
                       pcb_tick_loop_v2, &pcb_tick_ctx[3], QEMU_THREAD_JOINABLE);
    pcb_tick_threads_running = true;
    PCB_LOG("tick threads spawned v2 (virtual-clock paced via QEMUTimer + "
            "condvar) — déterministe, BQL-serialized");
}

/* === Plan de split (documentation inline) ================================
 *
 * À terme, calypso_trx.c (monolithe actuel) doit se découper :
 *
 *   calypso_trx.c              ── existing, ARM TRX glue, MMIO TPU/INTH
 *   ├─→ calypso_tpu.c          ── NEW : TPU thread (frame generator)
 *   ├─→ calypso_inth.c         ── NEW : IRQ dispatcher (clean from trx.c)
 *   └─→ (BSP déjà séparé dans calypso_bsp.c) — sera thread propre
 *
 *   calypso_c54x.c             ── DSP CPU : isoler c54x_run() pour le
 *                                  thread DSP qui le boucle (au lieu d'être
 *                                  appelé inline depuis tdma_tick).
 *
 *   calypso_sim.c              ── déjà clean ; ajouter le sim_thread qui
 *                                  pompe la FIFO + fire IT bits (au lieu
 *                                  du QEMUTimer actuel).
 *
 *   calypso_iota.c             ── déjà séparé ; thread pour SPI bus.
 *
 *   calypso_bsp.c              ── déjà séparé ; thread pour drain DARAM.
 *
 * Ce orchestrateur (calypso_full_pcb.c) :
 *   - Détient les mutex partagés
 *   - Spawn/join les threads
 *   - Centralise le raise/lower IRQ (traçable)
 *   - Pas de logique métier — pure orchestration
 */
