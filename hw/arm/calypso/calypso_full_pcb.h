/*
 * calypso_full_pcb.h — Calypso PCB-level threading orchestrator
 *
 * Interface entre les composants autonomes (DSP/BSP/TPU/SIM/IOTA) et
 * l'ARM main TCG thread. Wire les IRQ via la map osmocom-bb (irq.h) et
 * fournit les locks partagés (DARAM, API RAM, MMR).
 *
 * Voir THREADING_TODO.md pour le plan complet + chaîne IRQ.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_FULL_PCB_H
#define HW_ARM_CALYPSO_FULL_PCB_H

#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "hw/irq.h"

/* === IRQ map (mirror osmocom-bb include/calypso/irq.h) ===================
 * Référence canonique : firmware OsmocomBB. Used by qemu_irq dispatching
 * from device threads back to ARM (via INTH). */
#define CALYPSO_IRQ_WATCHDOG        0
#define CALYPSO_IRQ_TIMER1          1
#define CALYPSO_IRQ_TIMER2          2
#define CALYPSO_IRQ_TSP_RX          3
#define CALYPSO_IRQ_TPU_FRAME       4   /* TDMA frame tick → ARM frame_irq */
#define CALYPSO_IRQ_TPU_PAGE        5
#define CALYPSO_IRQ_SIMCARD         6   /* SIM IT_RX/WT/OV → sim_irq_handler */
#define CALYPSO_IRQ_UART_MODEM      7   /* osmocon L1CTL */
#define CALYPSO_IRQ_KEYPAD_GPIO     8
#define CALYPSO_IRQ_RTC_TIMER       9
#define CALYPSO_IRQ_RTC_ALARM_I2C  10
#define CALYPSO_IRQ_ULPD_GAUGING   11
#define CALYPSO_IRQ_EXTERNAL       12
#define CALYPSO_IRQ_SPI            13
#define CALYPSO_IRQ_DMA            14   /* BSP DMA done → ARM */
#define CALYPSO_IRQ_API            15   /* DSP↔ARM mailbox done */
#define CALYPSO_IRQ_SIM_DETECT     16
#define CALYPSO_IRQ_EXTERNAL_FIQ   17
#define CALYPSO_IRQ_UART_IRDA      18
#define CALYPSO_IRQ_ULPD_GSM_TIMER 19
#define CALYPSO_IRQ_GEA            20
#define CALYPSO_IRQ_MAX            21

/* === Locks partagés (à take/release par les thread entry points) ========
 * Convention d'ordre canonique pour éviter deadlock :
 *   daram_lock < api_ram_lock < sim_lock < bsp_q_lock < tpu_lock
 * Toujours acquire dans cet ordre, release dans l'inverse. */
extern QemuMutex calypso_pcb_daram_lock;    /* DARAM 0x0000-0x27FF */
extern QemuMutex calypso_pcb_api_ram_lock;  /* API mailbox 0x0800-0x0FFF */
extern QemuMutex calypso_pcb_sim_lock;      /* SIM controller it/fifo */
extern QemuMutex calypso_pcb_bsp_q_lock;    /* BSP UDP queue */
extern QemuMutex calypso_pcb_tpu_lock;      /* TPU registers + scenarios */

/* === Thread state (one per autonomous chip/unit) ======================== */
typedef enum {
    CALYPSO_THREAD_DSP = 0,
    CALYPSO_THREAD_BSP,
    CALYPSO_THREAD_TPU,
    CALYPSO_THREAD_SIM,
    CALYPSO_THREAD_IOTA,
    CALYPSO_THREAD_MAX
} CalypsoThreadId;

typedef struct CalypsoPcb CalypsoPcb;

/* === API publique ======================================================== */

/* Initialize PCB orchestrator: locks, thread state, IRQ routing table.
 * Called once during SoC init (from calypso_soc.c). */
CalypsoPcb *calypso_pcb_init(qemu_irq *inth_inputs);

/* Spawn threads per the threading plan. Gated by env CALYPSO_PCB_THREADS=1
 * (default OFF — legacy single-thread behavior).
 *
 * Si OFF : tous les composants restent dans le main TCG thread (existant).
 * Si ON  : chaque composant tourne dans son QemuThread, sync via locks +
 *          qemu_set_irq atomic.
 *
 * Phase rollout par env (selon THREADING_TODO.md) :
 *   CALYPSO_PCB_THREAD_SIM=1   → spawn sim_thread
 *   CALYPSO_PCB_THREAD_BSP=1   → spawn bsp_thread
 *   CALYPSO_PCB_THREAD_DSP=1   → spawn dsp_thread
 *   CALYPSO_PCB_THREAD_TPU=1   → spawn tpu_thread
 *   CALYPSO_PCB_THREAD_IOTA=1  → spawn iota_thread
 *   CALYPSO_PCB_THREADS=1      → tout activer
 */
void calypso_pcb_start_threads(CalypsoPcb *pcb);

/* Stop all threads cleanly (joins). Called at SoC shutdown. */
void calypso_pcb_stop_threads(CalypsoPcb *pcb);

/* IRQ helper : raise a Calypso IRQ from any thread.
 * Thread-safe (qemu_set_irq is atomic). Wrapper pour traçabilité +
 * logging optionnel. */
void calypso_pcb_raise_irq(CalypsoPcb *pcb, int irq_nr);
void calypso_pcb_lower_irq(CalypsoPcb *pcb, int irq_nr);

/* === Async log queue ====================================================
 * Pour les sites de log haute fréquence (UART IER, tdma tick, etc.) qui
 * fire depuis ARM TCG main thread. fprintf inline bloque le TCG (stdio
 * lock + write syscall). Cette queue les défère vers un drain thread
 * dédié — TCG juste enqueue (mutex bref) et continue.
 *
 * Toujours dispo (init dans calypso_pcb_init, indépendant des thread flags). */
void calypso_async_log(const char *fmt, ...) __attribute__((format(printf,1,2)));

/* Thread entry points — implémentés ailleurs, déclarés ici pour le
 * orchestrateur qui les spawn. Chacun prend `CalypsoPcb *` en arg. */
void *calypso_pcb_dsp_thread(void *arg);
void *calypso_pcb_bsp_thread(void *arg);
void *calypso_pcb_tpu_thread(void *arg);
void *calypso_pcb_sim_thread(void *arg);
void *calypso_pcb_iota_thread(void *arg);

/* === DARAM access helpers (cross-thread safety) =========================
 *
 * Wrappers pour les sites hors c54x.c qui lisent/écrivent dsp->data[] —
 * BSP IQ burst writes, TRX/FBSB API RAM mailbox mirror, etc.
 *
 * Avant 2026-05-25 : ces sites accédaient dsp->data[] direct sans lock.
 * En single-thread (no PCB DSP thread) c'est OK — pas de racer. Mais dès
 * qu'on active calypso_pcb_dsp_thread (qui boucle c54x_run en pthread
 * indépendant), DSP-side reads/writes (déjà locked par data_read/write
 * dans calypso_c54x.c) racent avec les writes externes non-protégés →
 * DARAM corruption non-déterministe.
 *
 * Usage :
 *   - Single-access : calypso_dsp_daram_read(dsp, addr) / _write(dsp,a,v)
 *     → encapsulent lock+access+unlock (impl dans pcb.c)
 *   - Burst (boucle 100+ iters) : lock externe via les pcb_daram_lock_*
 *     fonctions, accès direct dsp->data[] entre lock/unlock.
 *
 * Performance : qemu_mutex non-contended ~20-30ns. Pour ~300k ops/sec
 * overall (estimation), overhead < 1% wall. */

/* Burst lock helpers — pour les sections où on accède dsp->data[] en
 * boucle. Acquire une fois, accès directs entre, release une fois.
 *
 * IMPORTANT : pas de return/break dans la section sans release préalable. */
void calypso_pcb_daram_lock_acquire(void);
void calypso_pcb_daram_lock_release(void);

/* Single-access helpers — encapsulent lock+access+unlock.
 * dsp_void = C54xState* (typé void* pour éviter de tirer calypso_c54x.h
 * dans tous les .c qui includent pcb.h). */
uint16_t calypso_dsp_daram_read(void *dsp_void, uint16_t addr);
void     calypso_dsp_daram_write(void *dsp_void, uint16_t addr, uint16_t val);

#endif /* HW_ARM_CALYPSO_FULL_PCB_H */
