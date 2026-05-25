/*
 * calypso_c54x.c — TMS320C54x DSP emulator for Calypso
 *
 * Minimal C54x core: enough to run the Calypso DSP ROM for GSM
 * signal processing (Viterbi, deinterleaving, burst decode).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "calypso_c54x.h"
#include "hw/arm/calypso/calypso_full_pcb.h"  /* daram_lock, api_ram_lock */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* W1C latches for FB-detection iteration snapshot — defined in
 * calypso_trx.c. Set here in data_write when DSP writes a_sync_SNR
 * (LAST cell of fb-det sequence) from a real fb-det PC. Snapshot
 * captures all 6 cells coherent. Consumed by ARM read. */
extern uint16_t g_d_fb_det_latch;
extern uint16_t g_d_fb_mode_latch;
extern int calypso_w1c_latch_enabled(void);
extern uint16_t g_a_sync_TOA_latch;
extern uint16_t g_a_sync_PM_latch;
extern uint16_t g_a_sync_ANG_latch;
extern uint16_t g_a_sync_SNR_latch;
extern bool     g_a_sync_valid;

static int g_boot_trace = 0;

#define C54_LOG(fmt, ...) \
    fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Sign-extend 40-bit accumulator */
static inline int64_t sext40(int64_t v)
{
    if (v & ((int64_t)1 << 39))
        v |= ~(((int64_t)1 << 40) - 1);
    else
        v &= ((int64_t)1 << 40) - 1;
    return v;
}

/* Saturate 40-bit to 32-bit (OVM mode) */
static inline int64_t sat32(int64_t v)
{
    if (v > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (v < (int64_t)(int32_t)0x80000000) return (int64_t)(int32_t)0x80000000;
    return v;
}

/* Get ARP from ST0 */
static inline int arp(C54xState *s)
{
    return (s->st0 >> ST0_ARP_SHIFT) & 7;
}

/* Get DP from ST0 */
static inline uint16_t dp(C54xState *s)
{
    return s->st0 & ST0_DP_MASK;
}

/* Get ASM from ST1 (5-bit signed) */
static inline int asm_shift(C54xState *s)
{
    int v = s->st1 & ST1_ASM_MASK;
    if (v & 0x10) v |= ~0x1F;  /* sign extend */
    return v;
}

/* ================================================================
 * Memory access
 * ================================================================ */

/* Forward decl: used by data_write() VECDUMP at MMR_PMST. */
static uint16_t prog_read(C54xState *s, uint32_t addr);

/* Propagated by D_BURST_D probe, consumed by A_CD-BY-BURST correlation. */
static uint16_t g_last_d_burst_d;

/* === Generic watch-write zone helper (2026-05-15 matin) ===
 *
 * Factorisation du pattern COEFFS-WR / A_CD-WR / ... : pour chaque zone
 * mémoire surveillée, maintient per-PC counter + log throttled + summary
 * périodique. La 4e+ instrumentation devient triviale au call site.
 *
 * Cost : 512 KB de statique par zone (per_pc[0x10000] × uint64_t). Acceptable
 * pour debug. */
typedef struct {
    uint64_t per_pc[0x10000];
    uint64_t total;
    uint64_t last_log_insn;
    uint64_t last_summary_insn;
    uint16_t last_exec_pc;
} WatchWriteState;

static bool watch_write_zone_check(C54xState *s, uint16_t addr, uint16_t val,
                                   const char *name,
                                   uint16_t lo, uint16_t hi,
                                   WatchWriteState *st)
{
    if (addr < lo || addr > hi) return false;
    uint16_t exec_pc = s->last_exec_pc;
    st->per_pc[exec_pc]++;
    st->total++;
    bool should_log =
        st->total <= 500
        || exec_pc != st->last_exec_pc
        || (s->insn_count - st->last_log_insn) > 100000;
    if (should_log) {
        const char *wk_name[] = {
            "UNK", "F3", "8x", "77", "76", "PSHM",
            "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
        };
        uint8_t wk = s->writer_kind;
        const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                          ? wk_name[wk] : "??";
        fprintf(stderr,
                "[c54x] %s-WR #%llu addr=0x%04x val=0x%04x "
                "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x wk=%s "
                "AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                name, (unsigned long long)st->total, addr, val,
                exec_pc, s->pc, s->prog[s->pc], wkn,
                s->ar[3], s->ar[4], s->ar[5], s->insn_count);
        st->last_exec_pc = exec_pc;
        st->last_log_insn = s->insn_count;
    }
    if (s->insn_count - st->last_summary_insn >= 5000000) {
        st->last_summary_insn = s->insn_count;
        /* Format `<NAME>-WR-SUMMARY` (avec -WR- infix) pour cohérence avec
         * les per-hit lines `<NAME>-WR #N` et backward-compat regex tests. */
        fprintf(stderr, "[c54x] %s-WR-SUMMARY insn=%u total=%llu",
                name, s->insn_count, (unsigned long long)st->total);
        for (int p = 0; p < 0x10000; p++) {
            if (st->per_pc[p]) {
                fprintf(stderr, " pc[0x%04x]=%llu",
                        p, (unsigned long long)st->per_pc[p]);
            }
        }
        fprintf(stderr, "\n");
    }
    return true;
}

/* === FB-det timing/content stats (2026-05-14 night) ===
 *
 * Captures sur les ~928 fires de 0x8f51 (au lieu du cap 50 actuel) :
 *   - AR4 dans/hors zone [0x2bc0..0x2bff] → addressing vs timing
 *   - delta insn depuis dernier write par cluster (compute/clear/pattern)
 *   - histogramme val[AR4] : zero / 0xfffe sentinel / other
 *
 * Verdict :
 *   ar4_in_zone < 100% → bug d'addressing (AR4 pointe hors zone)
 *   delta_clear < delta_compute systématique → timing race (clear gagne)
 *   delta_compute >> 1M → compute jamais dans la fenêtre du fire
 *   other > 0 → certains sweeps voient des données — creuser ces fires
 */
static struct {
    /* Timing trackers (mis à jour dans data_write côté 0x2bc0..0x2bff) */
    uint64_t last_compute_insn;
    uint16_t last_compute_addr;
    uint64_t last_clear_insn;
    uint16_t last_clear_addr;
    uint64_t last_pattern_insn;
    uint16_t last_pattern_addr;
    /* Stats au moment du fire 0x8f51 */
    uint64_t fb_det_total;
    uint64_t fb_det_ar4_in_zone;
    uint64_t fb_det_ar4_outside;
    uint64_t fb_det_dar4_zero;
    uint64_t fb_det_dar4_sentinel;
    uint64_t fb_det_dar4_other;
    /* Sweep tracking : un sweep = 50 fires 0x8f51 consécutifs avec AR3
     * progressant 0..0x3A3 stride+19. Wrap (ar3 < last_ar3) = nouveau sweep. */
    uint16_t last_ar3_at_fire;
    uint64_t sweep_id;
    uint64_t sweep_nonzero_count;
} g_fb_det_timing;

/* === DSP throughput emission (2026-05-14 evening) ===
 *
 * Émet `[c54x] INSN-COUNT-STATS total=N delta=N elapsed_ms=N rate=N/s` toutes
 * les 1M insn. Lu en stéréo par :
 *   - test_dsp_throughput_5x (milestones, static)
 *   - test_dsp_throughput_above_threshold (observability, runtime)
 * Seuil pytest : 50M/s (marge ×2 sous les 100M/s historiques). */
#include <time.h>
static struct {
    uint64_t last_logged_insn;
    struct timespec last_logged_ts;
} g_throughput;

static inline void throughput_tick(uint64_t insn_count)
{
    if (insn_count - g_throughput.last_logged_insn < 1000000) return;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (g_throughput.last_logged_ts.tv_sec == 0 &&
        g_throughput.last_logged_ts.tv_nsec == 0) {
        g_throughput.last_logged_ts = now;
        g_throughput.last_logged_insn = insn_count;
        return;
    }
    int64_t delta_ns =
        (int64_t)(now.tv_sec - g_throughput.last_logged_ts.tv_sec) * 1000000000LL +
        (int64_t)(now.tv_nsec - g_throughput.last_logged_ts.tv_nsec);
    uint64_t delta_insn = insn_count - g_throughput.last_logged_insn;
    uint64_t rate = (delta_ns > 0)
        ? (delta_insn * 1000000000ULL / (uint64_t)delta_ns) : 0;
    fprintf(stderr,
            "[c54x] INSN-COUNT-STATS total=%llu delta=%llu elapsed_ms=%lld rate=%llu/s\n",
            (unsigned long long)insn_count,
            (unsigned long long)delta_insn,
            (long long)(delta_ns / 1000000),
            (unsigned long long)rate);
    g_throughput.last_logged_ts = now;
    g_throughput.last_logged_insn = insn_count;
}

/* === Read-by-range tracking for FB-det path analysis (2026-05-14 evening) ===
 *
 * Cible : identifier la zone DARAM lue par la routine FB-det sans préjuger.
 * Compteurs cumulatifs par plage + snapshot/delta à chaque "trigger PC"
 * (sites qui écrivent d_fb_det, identifiés par grep ZERO-WR + WR-SITE).
 *
 * Plages mutuellement exclusives :
 *   RR_MMRS   [0x0000..0x005F]  registres MMR C54x
 *   RR_LOW    [0x0060..0x03A3]  zone correlator linéaire (hypothèse 05-14)
 *   RR_APIRAM [0x0800..0x27FF]  API RAM partagée ARM/DSP (hypothèse β)
 *   RR_TARGET [0x3FB0..0x3FFF]  où BSP DMA écrit par défaut
 *   RR_WRAP   [0xFC5D..0xFFED]  zone correlator wrap BK=176 (AR2/AR7)
 *   RR_OTHER  tout le reste (incluant overlay 0x80..7FF, debord 0x4000+, etc.)
 *
 * Trigger PCs : 5 sites observés écrivant d_fb_det (4 ZERO-WR rares + 0x8f51
 * en boucle 50 fois). Le delta entre 2 triggers consécutifs = reads
 * cumulés dans la fenêtre amont. Cap à 200 triggers loggés pour ne pas
 * flooder. */
enum { RR_MMRS, RR_LOW, RR_APIRAM, RR_TARGET, RR_WRAP, RR_OTHER, RR_NUM };

static struct {
    uint64_t cumulative[RR_NUM];
    uint64_t snapshot[RR_NUM];
    uint64_t trigger_count;
} g_read_stats;

static inline void read_stats_record(uint16_t addr)
{
    int r;
    if      (addr <= 0x005F)                    r = RR_MMRS;
    else if (addr <= 0x03A3)                    r = RR_LOW;
    else if (addr >= 0x0800 && addr <= 0x27FF)  r = RR_APIRAM;
    else if (addr >= 0x3FB0 && addr <= 0x3FFF)  r = RR_TARGET;
    else if (addr >= 0xFC5D && addr <= 0xFFED)  r = RR_WRAP;
    else                                        r = RR_OTHER;
    g_read_stats.cumulative[r]++;
}

static void read_stats_trigger_check(C54xState *s)
{
    /* Trigger PC réduit à 0x8f51 uniquement (FB-det compute loop, 50 hits/sweep).
     * 2026-05-14 — Run précédent : trigger list large {0x8f51, 0x778a, 0x9ac0,
     * 0x9ad0, 0x9b00, 0x821a} → 0x821a en boot mailbox poll loop (14 insns
     * entre hits) a dévoré les 200 lignes de cap avant que 0x8f51 ne fire.
     * Les autres PCs étaient init/reset (1-3 hits chacun sur tout le run).
     * Cap remonté à 5000 pour couvrir plusieurs sweeps FB-det. */
    if (s->pc != 0x8f51) return;
    g_read_stats.trigger_count++;
    if (g_read_stats.trigger_count > 5000) return;
    uint64_t delta[RR_NUM];
    for (int r = 0; r < RR_NUM; r++) {
        delta[r] = g_read_stats.cumulative[r] - g_read_stats.snapshot[r];
        g_read_stats.snapshot[r] = g_read_stats.cumulative[r];
    }
    fprintf(stderr,
            "[c54x] READ-AMONT #%llu PC=0x%04x insn=%u "
            "mmrs=%llu low=%llu apiram=%llu target=%llu wrap=%llu other=%llu\n",
            (unsigned long long)g_read_stats.trigger_count, s->pc, s->insn_count,
            (unsigned long long)delta[RR_MMRS],
            (unsigned long long)delta[RR_LOW],
            (unsigned long long)delta[RR_APIRAM],
            (unsigned long long)delta[RR_TARGET],
            (unsigned long long)delta[RR_WRAP],
            (unsigned long long)delta[RR_OTHER]);
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr);

static uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* MTTCG : protège DARAM access (DSP + ARM-OVLY peuvent racer).
     * Sans MTTCG : mutex non contesté (overhead minimal). */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = data_read_locked(s, addr);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    return v;
}

static uint16_t data_read_locked(C54xState *s, uint16_t addr)
{
    read_stats_record(addr);
    /* D_BURST_D_W probe : DSP lit db_w->d_burst_d ?
     * 0x0801 (W_PAGE_0 + offset 1), 0x0815 (W_PAGE_1 + offset 1).
     * Si DSP read voit 0,1,2,3 séquentiel → ARM écrit correctement db_w.
     * Si DSP read voit 0 toujours → ARM ne configure pas burst_id.
     * Si DSP ne lit jamais → DSP ne consulte pas db_w pour le burst sequence. */
    if (addr == 0x0801 || addr == 0x0815) {
        static uint64_t dbw_total[2];
        static uint64_t dbw_per_val[2][16];
        static uint64_t dbw_last_log[2];
        static uint16_t dbw_last_val[2];
        int page = (addr == 0x0815) ? 1 : 0;
        uint16_t cur_val = s->data[addr] & 0xF;
        dbw_total[page]++;
        if (cur_val < 16) dbw_per_val[page][cur_val]++;
        bool changed = (cur_val != dbw_last_val[page]);
        dbw_last_val[page] = cur_val;
        if (dbw_total[page] <= 100 || changed
            || (s->insn_count - dbw_last_log[page]) > 1000000) {
            fprintf(stderr,
                    "[c54x] D_BURST_D_W-RD page=%d #%llu addr=0x%04x "
                    "val=0x%04x exec_pc=0x%04x insn=%u\n",
                    page, (unsigned long long)dbw_total[page], addr,
                    s->data[addr], s->last_exec_pc, s->insn_count);
            dbw_last_log[page] = s->insn_count;
        }
        /* Summary toutes les 50000 reads : histogramme valeurs lues */
        if ((dbw_total[page] % 50000) == 0) {
            fprintf(stderr,
                    "[c54x] D_BURST_D_W-SUMMARY page=%d total=%llu "
                    "val[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                    page, (unsigned long long)dbw_total[page],
                    (unsigned long long)dbw_per_val[page][0],
                    (unsigned long long)dbw_per_val[page][1],
                    (unsigned long long)dbw_per_val[page][2],
                    (unsigned long long)dbw_per_val[page][3],
                    (unsigned long long)(dbw_total[page]
                        - dbw_per_val[page][0] - dbw_per_val[page][1]
                        - dbw_per_val[page][2] - dbw_per_val[page][3]));
        }
    }
    /* PC-histogram pour identifier la routine PM. Deux ranges :
     *   [0x3fb0..0x3fbf] = buffer BSP (samples I/Q)
     *   [0x3dcf..0x3dd5] = buffer scratch dominant (78k+52k reads observés)
     * Compte par PC, dump top-10 toutes les 50k reads dans chaque range.
     * Plus compteur d'entrée par PC dominant pour distinguer
     * "PM cassée" vs "PM jamais appelée" (vu IRQ rate 1.5 Hz). */
    if (addr >= 0x3fb0 && addr <= 0x3fbf) {
        static uint32_t pc_hist_3fb[65536];
        static uint32_t total_3fb;
        pc_hist_3fb[s->pc]++;
        total_3fb++;
        if ((total_3fb % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3fb[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            fprintf(stderr, "[c54x] PC-HIST-3FB total=%u :", total_3fb);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    if (addr >= 0x3dcf && addr <= 0x3dd5) {
        static uint32_t pc_hist_3dd[65536];
        static uint32_t total_3dd;
        pc_hist_3dd[s->pc]++;
        total_3dd++;
        if ((total_3dd % 50000) == 0) {
            uint32_t top_pc[10] = {0};
            uint32_t top_cnt[10] = {0};
            for (uint32_t p = 0; p < 65536; p++) {
                uint32_t c = pc_hist_3dd[p];
                if (c == 0) continue;
                for (int i = 0; i < 10; i++) {
                    if (c > top_cnt[i]) {
                        for (int j = 9; j > i; j--) {
                            top_pc[j]  = top_pc[j-1];
                            top_cnt[j] = top_cnt[j-1];
                        }
                        top_pc[i]  = p;
                        top_cnt[i] = c;
                        break;
                    }
                }
            }
            fprintf(stderr, "[c54x] PC-HIST-3DD total=%u :", total_3dd);
            for (int i = 0; i < 10 && top_cnt[i]; i++) {
                fprintf(stderr, " %04x:%u", top_pc[i], top_cnt[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    /* Watch the mailbox slots that the firmware polls at PROM0 0xb41a
     * (LDU *(0x0ffe), A then BACC A) and 0xb41c (CMPM *(0x0fff), 4).
     * If these stay zero / 0x10 forever, ARM never wrote them. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x0ffc || addr == 0x0ffd) {
        static unsigned watch_count;
        watch_count++;
        if (watch_count <= 60 || (watch_count % 10000) == 0) {
            uint16_t vd = s->data[addr];
            uint16_t va = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0xDEAD;
            fprintf(stderr,
                    "[c54x] WATCH-READ #%u data[0x%04x] data=0x%04x api_ram=0x%04x api_set=%d PC=0x%04x insn=%u\n",
                    watch_count, addr, vd, va, s->api_ram ? 1 : 0, s->pc, s->insn_count);
        }
    }
    /* Wait-loop diagnostic: 0x3dd0 was found to absorb ~99.5 % of DARAM
     * reads after the first ~500k reads — the DSP is stuck polling it.
     * Log the first PCs and then sample once per million reads so we can
     * trace the loop without flooding the log. */
    if (addr == 0x3dd0) {
        static unsigned wait_log;
        static unsigned wait_seen;
        wait_seen++;
        if (wait_log < 20 || (wait_seen % 1000000) == 0) {
            wait_log++;
            fprintf(stderr,
                    "[c54x] WAIT-3DD0 #%u data[0x3dd0]=0x%04x PC=0x%04x AR2=%04x AR3=%04x insn=%u\n",
                    wait_seen, s->data[0x3dd0], s->pc,
                    s->ar[2], s->ar[3], s->insn_count);
        }
    }
    /* d_fb_det watch — REAL DSP word address is 0x08F8.
     * Mapping: ARM 0xFFD001F0 (BASE_API_NDB 0xFFD001A8 + 36 words × 2)
     *        = DSP word 0x0800 + 0x1F0/2 = 0x08F8.
     * Earlier 0x01F0 was the ARM byte-offset, NOT a DSP word address —
     * watching it logged unrelated DARAM 0x01F0 (junk). Now we trace
     * the real slot the firmware polls. */
    if (addr == 0x08F8) {
        static unsigned fb_read;
        if (fb_read++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-READ d_fb_det[0x08F8]=0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x08F8], s->pc, s->insn_count);
        }
    }
    /* === DIAG-FORCE-DARAM62 ===
     * Pinned diag (env-gated, default OFF) : when set, override the read
     * of daram[0x62] inside the dispatcher loop (PC ∈ 0xCC62..0xCC6F) to
     * return 1 instead of the actual stored value. Goal: force the
     * dispatcher's "branch if flag != 0" to fire and observe whether the
     * DSP escapes the loop and jumps to api[0x1f0c]=0x770c (the dispatch
     * target). Three outcomes (binary diagnostic) :
     *   - PC leaves cc62..cc6f → 0x770c and new code paths run :
     *     loop hypothesis correct, flag is the gate, INT3 ISR is the
     *     missing writer (next step: trace writes to confirm).
     *   - PC reaches 0x770c then returns to cc62 immediately :
     *     flag is set but handler bails because something else missing
     *     (a_cd[] init, NDB cell, ...).
     *   - No change : branch / compare is more subtle than read.
     * This is a force-test, not a fix — remove or env-leave-off after. */
    if (addr == 0x0062 && s->pc >= 0xCC62 && s->pc <= 0xCC6F) {
        static int force_cached = -1;
        if (force_cached < 0) {
            const char *e = getenv("CALYPSO_DSP_FORCE_DARAM62");
            force_cached = (e && *e == '1') ? 1 : 0;
            fprintf(stderr,
                    "[c54x] CALYPSO_DSP_FORCE_DARAM62=%d (%s)\n",
                    force_cached,
                    force_cached ? "FORCING daram[0x62]=1 in idle disp loop"
                                 : "real value (no force)");
            fflush(stderr);
        }
        if (force_cached) {
            static unsigned force_log;
            if (force_log < 5) {
                fprintf(stderr,
                        "[c54x] FORCE-DARAM62 #%u PC=0x%04x real=0x%04x → returning 0x0001 insn=%u\n",
                        force_log, s->pc, s->data[0x62], s->insn_count);
                force_log++;
            }
            return 1;
        }
    }

    /* === DSP idle dispatcher trace (PC ∈ 0xCC62..0xCC6F) ===
     * The DSP gets stuck in this PROM0 loop polling task slots. Dump the
     * exact (PC, addr, value, AR2..AR5) for the first N reads so we can
     * see WHICH memory location the dispatcher inspects to decide whether
     * to branch out (task_md ? db_r ? api_ram ? other ?). Capped to keep
     * log size manageable.
     *
     * Captures all reads (DARAM + API RAM + MMR) so we don't miss the
     * critical poll address. */
    if (s->pc >= 0xCC62 && s->pc <= 0xCC6F) {
        static unsigned idle_rd_log;
        const unsigned LIMIT = 200;
        if (idle_rd_log < LIMIT) {
            uint16_t v;
            const char *region;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
                region = "api";
            } else if (addr < 0x4000) {
                v = s->data[addr];
                region = "daram";
            } else {
                v = s->data[addr];
                region = "mmr/other";
            }
            fprintf(stderr,
                    "[c54x] IDLE-DISP RD #%u PC=0x%04x [%s 0x%04x]=0x%04x "
                    "AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
                    idle_rd_log, s->pc, region, addr, v,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            idle_rd_log++;
            if (idle_rd_log == LIMIT) {
                fprintf(stderr,
                        "[c54x] IDLE-DISP RD log capped at %u — pattern should be visible above\n",
                        LIMIT);
            }
        }
    }

    /* === DARAM discovery histogram ===
     * Track ALL data reads from DARAM (addr < 0x4000) regardless of PC.
     * The FB handler runs from both PROM0 (0xBD47) and DARAM overlay,
     * so filtering by PC misses critical reads. */
    if (addr < 0x4000 && addr >= 0x20) {  /* skip MMRs 0x00-0x1F */
        static unsigned hist[0x4000]; /* 16 KW DARAM */
        static unsigned reads;
        if (addr < 0x4000) {
            hist[addr]++;
            reads++;
            if ((reads % 50000) == 0) {
                /* find top-16 */
                unsigned best[16] = {0}; uint16_t baddr[16] = {0};
                for (uint16_t a = 0; a < 0x4000; a++) {
                    unsigned c = hist[a];
                    if (c <= best[15]) continue;
                    int p = 15;
                    while (p > 0 && best[p-1] < c) {
                        best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                    }
                    best[p] = c; baddr[p] = a;
                }
                fprintf(stderr,
                        "[c54x] DARAM RD HIST (FB-det, reads=%u): ",
                        reads);
                for (int i = 0; i < 16 && best[i]; i++)
                    fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                fprintf(stderr, "\n");
            }
        }
    }
    /* === BSP discovery: trace data reads in FB-det handler ===
     * Wide range over the PROM0 user-code area: handler PCs observed in
     * timeout traces cluster around 0x7e92..0x7eb8 (the FB-det inner
     * loop), so we widen the catch zone to 0x7000..0x7fff. */
    /* FB-det / dispatcher subroutine trace.
     * The 0x7e80..0x7eb8 wrapper CALLS into 0x81a5/0x81c8 with AR5=0x0e4c
     * (the FB sample buffer). Cover both ranges to catch both wrapper
     * polls and inner correlator reads. Skip the boot init phase. */
    if ((s->pc >= 0x7e80 && s->pc <= 0x7ec0) ||
        (s->pc >= 0x81a0 && s->pc <= 0x82ff)) {
        static int fbdet_rd_log = 0;
        if (s->insn_count > 50000000 && fbdet_rd_log < 2000) {
            uint16_t v;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
            else
                v = s->data[addr];
            C54_LOG("FBDET RD [0x%04x]=0x%04x PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                    addr, v, s->pc, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            fbdet_rd_log++;
        }
    }
    /* Log AR0..AR7 when entering FB-det subroutines to understand
     * what each AR points at (sample buffer? coeffs? status?). */
    if ((s->pc == 0x81a5 || s->pc == 0x81c8) && s->insn_count > 50000000) {
        static int ar_log = 0;
        if (ar_log < 10) {
            C54_LOG("FB-CALL PC=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x BK=%04x",
                    s->pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp, s->bk);
            ar_log++;
        }
    }
    /* d_spcx_rif (NDB word 2 = api 0xD6 = DSP data 0x08D6) */
    if (addr == 0x08D6) {
        static int spcx_rd = 0;
        if (spcx_rd < 32) {
            C54_LOG("d_spcx_rif RD = 0x%04x PC=0x%04x insn=%u",
                    s->api_ram ? s->api_ram[0xD6] : s->data[addr],
                    s->pc, s->insn_count);
            spcx_rd++;
        }
    }
    /* Log reads from API RAM at 0x08D4 (d_dsp_page) */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
    }
    /* Timer registers (0x0024-0x0026) — read returns current value */
    if (addr == TIM_ADDR) return s->data[TIM_ADDR];
    if (addr == PRD_ADDR) return s->data[PRD_ADDR];
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        return tcr;
    }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                /* bit 5 = BRINT0 per C54X header (vec 21). */
                C54_LOG("IFR READ=0x%04x (BRINT0 pending) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: return 0;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    return s->data[addr];
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val);

static void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    /* MTTCG lock : voir data_read ci-dessus. */
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    data_write_locked(s, addr, val);
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

static void data_write_locked(C54xState *s, uint16_t addr, uint16_t val)
{
    /* COEFFS-WR probe : watch-write sur la zone [0x2bc0..0x2bff] (64 mots).
     * 2026-05-14 evening — COEFFS-DUMP a montré une séquence init→clear→use :
     *   insn=2M  PC=0x9b05  vraies coeffs (f320, a660, ...)
     *   insn=3M  PC=0x9abc  pattern uniforme 0x0001 (suspect)
     *   insn=4M  PC=0x9abd  ALL ZERO (clear)
     *   insn=9M+ PC=0x8f51  ALL ZERO toujours (read par correlator)
     *
     * v2 (run précédent cap 200) : 3 clusters identifiés —
     *   0x8216 (wk=OTHER, 23 hits) = vraies coeffs
     *   0x9ace (wk=8, 64 hits)    = clear partiel
     *   0x9abf (wk=8x, 113 hits)  = pattern 0x0001
     * Cap atteint à insn=1.65M. On veut savoir si 0x8216 refire après.
     *
     * v3 (ce patch) :
     *   - Per-PC counter global sur tout le run (jamais capé)
     *   - Throttled log : 500 premiers + transitions PC + 1/100k insns
     *   - Summary périodique tous les 5M insns dump tous les PCs avec count>0
     * Tranche (1) re-fire 0x8216 manqué vs (2) one-shot définitif. */
    /* Timing trackers par cluster (utilisés par la sonde 0x8f51) — gardés
     * en dehors du helper car cluster-specific. Le watch_write_zone_check
     * factorise le compteur per-PC + log + summary. */
    if (addr >= 0x2bc0 && addr <= 0x2bff) {
        uint16_t exec_pc = s->last_exec_pc;
        if (exec_pc == 0x8216 || exec_pc == 0x8217 || exec_pc == 0x8218) {
            g_fb_det_timing.last_compute_insn = s->insn_count;
            g_fb_det_timing.last_compute_addr = addr;
        } else if (exec_pc == 0x9ace) {
            g_fb_det_timing.last_clear_insn = s->insn_count;
            g_fb_det_timing.last_clear_addr = addr;
        } else if (exec_pc == 0x9abf) {
            g_fb_det_timing.last_pattern_insn = s->insn_count;
            g_fb_det_timing.last_pattern_addr = addr;
        }
    }
    {
        static WatchWriteState wws_coeffs;
        watch_write_zone_check(s, addr, val, "COEFFS", 0x2bc0, 0x2bff, &wws_coeffs);
    }
    /* A_CD-WR : a_cd[15] in NDB starts at DSP word 0x09D0 (= API byte 0x03A0,
     * = NDB byte offset 0x1F8). 15 words = [0x09D0..0x09DE].
     * Cible : tracker si le DSP CCCH demod (DSP_TASK_ALLC) écrit ses résultats. */
    {
        static WatchWriteState wws_a_cd;
        watch_write_zone_check(s, addr, val, "A_CD", 0x09d0, 0x09de, &wws_a_cd);
        /* A_CD-BY-BURST : corrélation a_cd[] writes avec d_burst_d courant.
         * Si DSP fait burst 0→1→2→3 → ~25% des writes par burst_id.
         * Si on voit 0 writes avec burst=3 → DSP n'écrit jamais la fin de
         * séquence, d'où firmware ARM nb_resp bail (sous-cause #3). */
        if (addr >= 0x09d0 && addr <= 0x09de) {
            static uint64_t a_cd_by_burst[16];
            static uint64_t a_cd_corr_total;
            static uint64_t a_cd_corr_last_log;
            uint16_t b = g_last_d_burst_d & 0xF;
            a_cd_by_burst[b]++;
            a_cd_corr_total++;
            if (a_cd_corr_total - a_cd_corr_last_log >= 1000) {
                a_cd_corr_last_log = a_cd_corr_total;
                fprintf(stderr,
                        "[c54x] A_CD-BY-BURST total=%llu "
                        "burst[0]=%llu [1]=%llu [2]=%llu [3]=%llu other=%llu\n",
                        (unsigned long long)a_cd_corr_total,
                        (unsigned long long)a_cd_by_burst[0],
                        (unsigned long long)a_cd_by_burst[1],
                        (unsigned long long)a_cd_by_burst[2],
                        (unsigned long long)a_cd_by_burst[3],
                        (unsigned long long)(a_cd_corr_total -
                                             a_cd_by_burst[0] - a_cd_by_burst[1] -
                                             a_cd_by_burst[2] - a_cd_by_burst[3]));
            }
        }
    }
    /* D_BURST_D probe (2026-05-15 midi) — watch d_burst_d à 0x0829 (page 0)
     * et 0x083D (page 1). Mesures : per-PC counter, transition matrix,
     * histogramme. Tranche la sous-cause :
     *   0,1,2,3 séquentiel → DSP signale correct, bug ARM-side
     *   0,1,2 jamais 3     → DSP cale au 4e burst (sous-cause #3)
     *   pas de write       → DSP n'écrit pas cette cellule du tout
     */
    if (addr == 0x0829 || addr == 0x083D) {
        static uint64_t db_total[2];
        static uint64_t db_per_pc[2][0x10000];
        static uint16_t db_prev[2];
        static uint64_t db_trans[2][16][16];
        static uint64_t db_last_log[2];
        static uint64_t db_last_summary[2];
        int page = (addr == 0x083D) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = db_prev[page];
        uint16_t curr_val = val & 0xF;
        db_total[page]++;
        db_per_pc[page][exec_pc]++;
        if (prev_val < 16 && curr_val < 16) {
            db_trans[page][prev_val][curr_val]++;
        }
        db_prev[page] = curr_val;
        g_last_d_burst_d = curr_val;  /* propage pour A_CD-BY-BURST */
        bool should_log = db_total[page] <= 200
            || (s->insn_count - db_last_log[page]) > 100000;
        if (should_log) {
            fprintf(stderr,
                    "[c54x] D_BURST_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=%u curr=%u insn=%u\n",
                    page, (unsigned long long)db_total[page], addr, val,
                    exec_pc, prev_val, curr_val, s->insn_count);
            db_last_log[page] = s->insn_count;
        }
        if (s->insn_count - db_last_summary[page] >= 5000000) {
            db_last_summary[page] = s->insn_count;
            fprintf(stderr,
                    "[c54x] D_BURST_D-SUMMARY page=%d total=%llu trans:",
                    page, (unsigned long long)db_total[page]);
            for (int p = 0; p < 8; p++) {
                for (int c = 0; c < 8; c++) {
                    if (db_trans[page][p][c]) {
                        fprintf(stderr, " %u->%u=%llu",
                                p, c, (unsigned long long)db_trans[page][p][c]);
                    }
                }
            }
            fprintf(stderr, "\n");
        }
    }
    /* D_TASK_D probe (2026-05-15 fin journée) — watch d_task_d à 0x0828
     * (page 0) et 0x083C (page 1), READ side de la struct db_buf_r.
     * ARM L1 prim_rx_nb lit ce champ via dsp_api.db_r->d_task_d et bail
     * avec puts("EMPTY") si == 0. 60 EMPTY printf observés sous synth=1
     * banc d'essai déterministe : DSP n'écrit jamais cette cellule (ou
     * écrit 0). Cette probe trace : qui écrit ? quand ? avec quelle valeur ?
     * Distinguer entre :
     *   - DSP ne touche jamais 0x0828/0x083C
     *   - DSP écrit val=0 (clear/init seulement)
     *   - DSP écrit val=24 (DSP_TASK_ALLC) mais ARM lit avant le write
     */
    if (addr == 0x0828 || addr == 0x083C) {
        static uint64_t dt_total[2];
        static uint16_t dt_prev[2];
        static uint64_t dt_last_log[2];
        int page = (addr == 0x083C) ? 1 : 0;
        uint16_t exec_pc = s->last_exec_pc;
        uint16_t prev_val = dt_prev[page];
        uint16_t curr_val = val;
        dt_total[page]++;
        dt_prev[page] = curr_val;
        bool should_log = dt_total[page] <= 200
            || (s->insn_count - dt_last_log[page]) > 100000;
        if (should_log) {
            fprintf(stderr,
                    "[c54x] D_TASK_D-WR page=%d #%llu addr=0x%04x val=0x%04x "
                    "exec_pc=0x%04x prev=0x%04x insn=%u\n",
                    page, (unsigned long long)dt_total[page], addr, val,
                    exec_pc, prev_val, s->insn_count);
            dt_last_log[page] = s->insn_count;
        }
    }
    /* DATA-W-MMR : log every write into the low MMR window (addr <= 0x1F)
     * with full attribution context. Goal : disambiguate the IMR-W trace
     * cascade observed at PC=0x8eb9 (op=0xf3e1) and PC=0x9ad0 (op=0x8192).
     * The writer_kind field tells us *which path* triggered the write
     * (opcode family / IRQ ack / ARM MMIO / resolve_smem side effect).
     * Cap at 200 distinct events to avoid log flood. */
    if (addr <= 0x1F) {
        static unsigned mmrw_log;
        if (mmrw_log++ < 200) {
            const char *wk_name[] = {
                "UNK", "F3", "8x", "77", "76", "PSHM",
                "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
            };
            uint8_t wk = s->writer_kind;
            const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                              ? wk_name[wk] : "??";
            fprintf(stderr,
                    "[c54x] DATA-W-MMR addr=0x%02x val=0x%04x "
                    "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x "
                    "xpc=%d wk=%s "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "SP=%04x DP=%d INTM=%d insn=%u\n",
                    addr, val,
                    s->last_exec_pc, s->pc, s->prog[s->pc],
                    s->xpc, wkn,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    s->sp, dp(s),
                    !!(s->st1 & ST1_INTM),
                    s->insn_count);
        }
    }

    /* WATCH-WRITE on the same mailbox slots tracked in data_read.
     * Whoever writes them — DSP or ARM via api_ram alias — gets logged
     * so we can attribute the source of the value the firmware polls. */
    /* WATCH-WRITE 0x3dd2 — la cellule sur laquelle 0x75db poll en boucle
     * (37M reads/15s). Identifier qui écrit (et qui ne le fait pas).
     * Cas 1 : zéro write → un bloc compute ne fire jamais.
     * Cas 2 : write boot only → init OK mais set steady-state manquant.
     * Cas 3 : writes périodiques avec valeur jamais matchée par le test
     *         à 0x75db → bug dans le compute en amont. */
    if (addr == 0x3dd2) {
        static unsigned w3dd2;
        w3dd2++;
        if (w3dd2 <= 100 || (w3dd2 % 1000) == 0) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE 0x3dd2 #%u <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u INTM=%d\n",
                    w3dd2, val, s->data[addr], s->pc, s->insn_count,
                    !!(s->st1 & ST1_INTM));
        }
    }
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x01F0) {
        static unsigned wcount;
        if (wcount++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE data[0x%04x] <- 0x%04x  (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher pointer at data[0x3f65] — `LD *(0x3f65),A; CALA A` at
     * DARAM 0x008a-0x008c. When this slot holds 0xfff8/0x0000/garbage the
     * CALA jumps into PROM1 vec or boot stub NOPs and the SP runs away.
     * Trace every write so we can identify who populates / corrupts it. */
    if (addr == 0x3f65) {
        static unsigned dpw;
        if (dpw++ < 100) {
            fprintf(stderr,
                    "[c54x] DISP-PTR data[0x3f65] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher poll addresses — log ANY write so we identify the
     * code path that should populate them. Currently 0 PORTR PA=0xF430
     * fires because dispatcher reads 0 here forever. */
    if (addr == 0x4359 || addr == 0x3fab) {
        static unsigned dispw;
        if (dispw++ < 50) {
            fprintf(stderr,
                    "[c54x] DISP-WRITE data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* CALAD source zone 0x4180-0x41FF — LD-A-TRACE shows the firmware
     * reads 0x4189 (DP=0x83) but our emulation has it as 0. Log every
     * write to this range so we can tell whether (a) anyone is meant to
     * populate it and we missed the path, or (b) DP=0x83 is itself a
     * symptom upstream of an unrelated bug. */
    if (addr >= 0x4180 && addr <= 0x41FF) {
        static unsigned cwz;
        if (cwz++ < 5000) {
            fprintf(stderr,
                    "[c54x] CALAD-ZONE-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dedicated watch on 0x4189 — never capped. The LD-A loop reads this
     * slot in the CALAD trap; we want to know if/when *anyone* finally
     * writes a non-zero value, and from which PC. */
    if (addr == 0x4189) {
        fprintf(stderr,
                "[c54x] *** WR-0x4189 *** data[0x4189] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                val, s->data[addr], s->pc, s->insn_count);
    }
    /* === DARAM[0x40..0x90] watch — dispatcher flag area ===
     * The PROM0 idle dispatcher (0xCC62..0xCC6F) polls data[0x62] and
     * other slots in [0x60..0x70]. FORCE-DARAM62=1 (env) proves that
     * setting data[0x62]=1 makes the DSP escape and reach 0x770c, so
     * this range gates the runtime task pipeline. ARM-side writes to
     * the API page mirror at +0x0800 (calypso_trx.c calypso_dsp_write)
     * but never to DARAM 0x40..0x90 — so any value here must come from
     * DSP-self stores (ST/STH/STM/...) or stay zero forever. Capture
     * EVERY write with PC+INTM+insn so we can attribute the source.
     * INTM annotation lets us tell ISR-context writes from main code. */
    if (addr >= 0x0040 && addr <= 0x0090) {
        static unsigned daram_disp_w;
        if (daram_disp_w++ < 1000) {
            fprintf(stderr,
                    "[c54x] DISP-FLAG-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x INTM=%d IFR=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc,
                    !!(s->st1 & ST1_INTM), s->ifr, s->insn_count);
            if (daram_disp_w == 1000) {
                fprintf(stderr,
                        "[c54x] DISP-FLAG-W log capped at 1000 — pattern visible above\n");
            }
        }
    }
    /* Timer registers (0x0024-0x0026) — before MMR check */
    if (addr == TCR_ADDR) {
        /* TRB: write 1 → reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        return;
    }
    if (addr == TIM_ADDR) { s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) { s->data[PRD_ADDR] = val; return; }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:
            if (val != s->imr) {
                static unsigned imr_log = 0;
                /* Always log transitions TO zero (mask-everything) — that
                 * is the cascade root suspected in 2026-05-08 v2 diag :
                 * IMR=0 → INT3 IFR pending forever → RPTB at 0xe9ac never
                 * exits. We need the PC + opcode of every IMR=0 write,
                 * uncapped, so we can identify the buggy code path. */
                bool to_zero = (val == 0);
                if (imr_log++ < 50 || to_zero) {
                    fprintf(stderr,
                            "[c54x] IMR-W %s 0x%04x → 0x%04x PC=0x%04x "
                            "op=0x%04x prev_op=0x%04x SP=0x%04x INTM=%d "
                            "AR6=0x%04x AR7=0x%04x insn=%u\n",
                            to_zero ? "*ZERO*" : "      ",
                            s->imr, val, s->pc,
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc - 1)],
                            s->sp,
                            !!(s->st1 & ST1_INTM),
                            s->ar[6], s->ar[7],
                            s->insn_count);
                }
            }
            s->imr = val; return;
        case MMR_IFR:  s->ifr &= ~val; return;  /* write 1 to clear */
        case MMR_ST0:  s->st0 = val; return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:
            if (val >= 0x0800 && val < 0x0900) {
                fprintf(stderr,
                        "[c54x] SP-GUARD: refused MMR_SP write 0x%04x "
                        "(API mailbox); keeping 0x%04x PC=0x%04x\n",
                        val, s->sp, s->pc);
                return;
            }
            s->sp = val;
            return;
        case MMR_BK:   s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            {
                static unsigned pmst_wr_attempts = 0;
                if (pmst_wr_attempts++ < 100)
                    C54_LOG("PMST WR attempt #%u: val=0x%04x cur=0x%04x PC=0x%04x insn=%u",
                            pmst_wr_attempts, val, s->pmst, s->pc, s->insn_count);
            }
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                {
                    static unsigned pmst_log = 0;
                    if (pmst_log++ < 100)
                        C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u #%u/100",
                                s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count, pmst_log);
                }

                static uint16_t last_dumped_iptr = 0xFFFF;
                static unsigned vecdump_count = 0;
                /* Cap at 8 dumps total — firmware may oscillate between 2-3
                 * IPTR values thousands of times during a session, and each
                 * dump emits 32 fprintf lines. Without cap : 250k+ log lines
                 * = saturates host I/O = bridge stops emitting CLK INDs =
                 * BTS shutdown "No more clock from transceiver". */
                if (new_iptr != last_dumped_iptr && vecdump_count < 8) {
                    vecdump_count++;
                    last_dumped_iptr = new_iptr;
                    uint32_t base = (uint32_t)new_iptr << 7;
                    uint16_t saved_pmst = s->pmst;
                    s->pmst = val;
                    C54_LOG("VECDUMP IPTR=0x%03x base=0x%04x (32 vectors) #%u/8:",
                            new_iptr, (uint16_t)base, vecdump_count);
                    for (int vec = 0; vec < 32; vec++) {
                        uint32_t a = base + vec * 4;
                        uint16_t w0 = prog_read(s, a + 0);
                        uint16_t w1 = prog_read(s, a + 1);
                        uint16_t w2 = prog_read(s, a + 2);
                        uint16_t w3 = prog_read(s, a + 3);
                        fprintf(stderr,
                                "[c54x] vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                                vec, (uint16_t)a, w0, w1, w2, w3);
                    }
                    s->pmst = saved_pmst;
                }
            }
            s->pmst = val; return;
        case MMR_XPC:
            {
                static int xpc_log = 0;
                if (xpc_log++ < 50)
                    C54_LOG("MMR_XPC WR val=0x%04x (was %d) PC=0x%04x SP=0x%04x insn=%u",
                            val, s->xpc, s->pc, s->sp, s->insn_count);
            }
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* DMA sub-register bank (C54x DMA controller).
     * DMSA (0x0054): sets the sub-register address.
     * DMSDI (0x0055): writes sub-register data, auto-increments DMSA.
     * DMSDN (0x0057): writes sub-register data, no auto-increment.
     * DMA channel 0 sub-registers (BSP receive DMA):
     *   sub 0x00=DMSRC0, 0x01=DMDST0, 0x02=DMCTR0, 0x03=DMMCR0 */
    if (addr == 0x0054) {
        s->dma_subaddr = val;
        s->data[0x0054] = val;
        return;
    }
    if (addr == 0x0055 || addr == 0x0057) {
        uint16_t sa = s->dma_subaddr;
        if (sa < 24) {  /* 6 channels × 4 regs */
            s->dma_subregs[sa] = val;
            int ch = sa / 4;
            int reg = sa % 4;
            static const char *rnames[] = {"SRC","DST","CTR","MCR"};
            C54_LOG("DMA ch%d %s = 0x%04x (sub 0x%02x) PC=0x%04x",
                    ch, rnames[reg], val, sa, s->pc);
        }
        s->data[addr] = val;
        if (addr == 0x0055) s->dma_subaddr++;  /* auto-increment */
        return;
    }

    /* McBSP sub-register bank (serial port extended config).
     * SPSA (0x0038): sub-address. SPSD (0x0039): sub-data. */
    if (addr == 0x0038 || addr == 0x0039) {
        if (addr == 0x0038) s->spsa = val;
        else {
            C54_LOG("McBSP sub[0x%02x] = 0x%04x PC=0x%04x", s->spsa, val, s->pc);
        }
        s->data[addr] = val;
        return;
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        uint16_t woff = addr - C54X_API_BASE;
        if (s->api_ram)
            s->api_ram[woff] = val;
        /* Notify the ARM-side mailbox watcher (calypso_trx) so it can
         * pulse IRQ_API, mirror to dsp_ram, and run the d_fb_det hook.
         * Without this, DSP writes to NDB cells are invisible to ARM. */
        if (s->api_write_cb)
            s->api_write_cb(s->api_write_cb_opaque, woff, val);
        /* Stack-corruption watch: stack push landing in the NDB
         * mailbox region [0x0800..0x08FF]. Only fires when SP has
         * already been corrupted into that range. */
        if (addr == s->sp && addr >= 0x0800 && addr < 0x0900) {
            fprintf(stderr,
                    "[c54x] STACK-IN-NDB addr=0x%04x val=0x%04x SP=0x%04x "
                    "PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x\n",
                    addr, val, s->sp, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Always log writes to d_dsp_page (0x08D4) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_spcx_rif (NDB word 2 = DSP data 0x08D6) — BSP serial port config */
        if (addr == 0x08D6) {
            C54_LOG("DSP WR d_spcx_rif = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_fb_det (NDB word 36 = DSP data 0x08F8). The DSP correlator
         * output here is treated as Q15-signed by the firmware FB-det
         * path — small unsigned BSIC was a wrong assumption. Log every
         * write unconditionally (thinned past 200) and dump the
         * adjacent NDB cells [0x08F0..0x0900] so we can see correlator
         * + flag + a_sync_demod fields together. */
        /* Silent NDB cells watch — d_fb_mode (binary "FB matched" flag,
         * THE actual trigger ARM tests), a_sync_PM (power), a_sync_SNR
         * (SNR). All read as 0 by ARM during 200M run despite d_fb_det
         * varying. Confirms: DSP never declares valid detection.
         * Three discriminating outcomes:
         *   (α) never written → "FB confirmed" code path unreached
         *   (β) written =0 explicitly → DSP scans, never matches threshold
         *   (γ) written !=0 but ARM reads 0 → coherence bug */
        /* W1C latch on d_fb_mode for real fb-det PCs.
         * Race-window evidence (200M run, 2026-04-29) :
         *   DSP writes d_fb_mode = 0x0001 30× from PC=0x8d33/0x8f51
         *   ARM reads d_fb_mode 1× and sees 0x0000
         * → DSP sets, then clears within tight loop, ARM polls between
         *   → 100% of detections lost.
         * Latch: real-fb-det PC with non-zero val sets g_d_fb_mode_latch.
         * ARM read in calypso_trx.c::calypso_dsp_read consumes & clears. */
        /* Snapshot trigger: DSP writes a_sync_SNR (0x08FD, LAST cell of
         * fb-det iteration) from a real fb-det PC. At this moment all 6
         * cells (d_fb_det, d_fb_mode, a_sync_TOA/PM/ANG/SNR) are coherent
         * for the just-completed iteration. Snapshot atomically; survives
         * subsequent stack-stomp at PC=0x0662 etc.
         * Order observed: d_fb_det → d_fb_mode → a_sync_TOA → PM → ANG
         * → SNR (insn N..N+150). */
        if (calypso_w1c_latch_enabled() &&
            addr == 0x08FD && val != 0 &&
            (s->pc == 0x8d33 || s->pc == 0x8eb9 || s->pc == 0x8f51)) {
            g_d_fb_det_latch   = s->data[0x08F8];
            g_d_fb_mode_latch  = s->data[0x08F9];
            g_a_sync_TOA_latch = s->data[0x08FA];
            g_a_sync_PM_latch  = s->data[0x08FB];
            g_a_sync_ANG_latch = s->data[0x08FC];
            g_a_sync_SNR_latch = val;
            g_a_sync_valid     = true;
        }

        /* === Direct d_fb_det + a_sync_demod latch trigger (2026-05-23) ===
         *
         * Phase 1 (initial fix) : direct latch sur d_fb_det. ARM consume
         * empirique confirmé (LATCH-CONSUME = 0x001e fn=164). Mais FBSB
         * RESP result=255 (fail) parce que d_fb_mode + a_sync_TOA/PM/ANG/SNR
         * tous lus à 0 — pcap : "FBSB RESP: result=255".
         *
         * Phase 2 (extension this commit) : latch chaque cellule individu-
         * ellement quand DSP write avec val != 0, any PC. Le clobber 0x821a
         * (val=0) ne fire pas. Chaque cellule a son propre lifecycle
         * indépendant : DSP write → latch, ARM read → consume & clear.
         *
         * Cellules couvertes :
         *   0x08F8 d_fb_det     (FB detection magic value)
         *   0x08F9 d_fb_mode    (FB detection mode, BSIC etc.)
         *   0x08FA a_sync_TOA   (Time of Arrival)
         *   0x08FB a_sync_PM    (Power Measurement)
         *   0x08FC a_sync_ANG   (Angle/phase)
         *   0x08FD a_sync_SNR   (Signal-to-Noise Ratio)
         *
         * Pour SB sync à converger, firmware lit les 6 cellules en
         * séquence. Toutes doivent être valides post-FB-det iteration. */
        if (calypso_w1c_latch_enabled() && val != 0) {
            switch (addr) {
            case 0x08F8: g_d_fb_det_latch   = val; g_a_sync_valid = true; break;
            case 0x08F9: g_d_fb_mode_latch  = val; g_a_sync_valid = true; break;
            case 0x08FA: g_a_sync_TOA_latch = val; g_a_sync_valid = true; break;
            case 0x08FB: g_a_sync_PM_latch  = val; g_a_sync_valid = true; break;
            case 0x08FC: g_a_sync_ANG_latch = val; g_a_sync_valid = true; break;
            case 0x08FD: g_a_sync_SNR_latch = val; g_a_sync_valid = true; break;
            }
        }

        /* === Phase 3 atomic snapshot (2026-05-23) ===
         *
         * Phase 2 latch chaque cellule indépendamment, mais empiriquement
         * (pcap : FBSB RESP result=255 + firmware "TOA=0, Power=-138dBm"),
         * le DSP au PC=0x778a écrit 4 cellules sur 6 — TOA et PM restent
         * non-écrites (= cached old values, probablement 0).
         *
         * Le firmware lit TOA=0 + PM=0 → noise floor → FB jugé invalide.
         *
         * Phase 3 mimique le SNR-based trigger original : quand a_sync_SNR
         * (dernière cellule de la séquence DSP fb-det) fire non-zero, snap-
         * shot atomique des 6 cellules depuis s->data[] — même si TOA/PM
         * y sont à 0 (capture l'état réel post-iteration DSP). Ça remplit
         * tous les latches avec ce que DSP a effectivement écrit (et 0
         * pour ce qui n'a pas été touché), donc firmware lit ces 0 mais
         * AU MOINS les cellules qui ONT été écrites en Phase 2 sont
         * préservées dans les latches individuels (l'OR des deux gates). */
        if (calypso_w1c_latch_enabled() && addr == 0x08FD && val != 0) {
            /* Atomic snapshot all 6 cells - PHASE 3 ANY PC trigger.
             * Capture l'état DSP au moment où SNR (last cell) est écrit.
             * Si DSP path incomplet (TOA/PM = 0 dans s->data), le latch
             * capture quand même les zeros — le vrai fix est upstream
             * (timer management / cascade), pas une synthesis ici. */
            if (g_d_fb_det_latch   == 0) g_d_fb_det_latch   = s->data[0x08F8];
            if (g_d_fb_mode_latch  == 0) g_d_fb_mode_latch  = s->data[0x08F9];
            if (g_a_sync_TOA_latch == 0) g_a_sync_TOA_latch = s->data[0x08FA];
            if (g_a_sync_PM_latch  == 0) g_a_sync_PM_latch  = s->data[0x08FB];
            if (g_a_sync_ANG_latch == 0) g_a_sync_ANG_latch = s->data[0x08FC];
            g_a_sync_SNR_latch = val;
            g_a_sync_valid     = true;
        }
        /* Phase 3.5/3.6 RETIRÉES 2026-05-23 — étaient des synth hacks
         * (PM=0x280 si nul, SNR=0x64 si bit haut). User redirection :
         * "pas de hack, c'est la gestion du temps" — le vrai problème
         * est timer/IRQ management côté DSP (IMR=0 → INT3 jamais
         * servicé → DSP ne tourne pas ses tâches schedulées → FB-det
         * incomplet, ALLC jamais firé, pas de SI). Fix upstream pas
         * downstream. */
        /* Full a_sync_demod + d_fb_mode WR watch — every cell, no PC
         * filter (so we catch real-fb-det writes AND stomp candidates).
         * Stomp zone PC=0x06xx tagged for easy grep. */
        if (addr == 0x08F9 || addr == 0x08FA ||
            addr == 0x08FB || addr == 0x08FC || addr == 0x08FD) {
            static unsigned ts_log[5] = {0};
            static uint16_t prev_d_fb_mode = 0xFFFF;
            int idx = (addr == 0x08F9) ? 0 :
                      (addr == 0x08FA) ? 1 :
                      (addr == 0x08FB) ? 2 :
                      (addr == 0x08FC) ? 3 : 4;
            const char *name = (idx == 0) ? "d_fb_mode"  :
                               (idx == 1) ? "a_sync_TOA" :
                               (idx == 2) ? "a_sync_PM"  :
                               (idx == 3) ? "a_sync_ANG" : "a_sync_SNR";
            ts_log[idx]++;
            bool transition = (idx == 0) &&
                              (prev_d_fb_mode != 0xFFFF) &&
                              (prev_d_fb_mode != val) &&
                              (val != 0 || prev_d_fb_mode != 0);
            bool stomp_zone = (s->pc >= 0x0600 && s->pc < 0x0700);
            bool log_it = transition ||
                          (idx == 0 && val != 0) ||
                          (val != 0 && ts_log[idx] <= 50) ||
                          (ts_log[idx] % 1000) == 0;
            if (log_it) {
                C54_LOG("DSP WR %s = 0x%04x (s=%d) PC=0x%04x%s insn=%u #%u%s",
                        name, val, (int)(int16_t)val, s->pc,
                        stomp_zone ? " [STOMP?]" : "",
                        s->insn_count, ts_log[idx],
                        transition ? " *TRANSITION*" : "");
            }
            if (idx == 0) prev_d_fb_mode = val;
        }
        if (addr == 0x08F8) {
            static unsigned fbd_log = 0;
            /* Filter out stack-stomp at d_fb_det: only PCs known to be
             * actual fb-det correlator stores (0x8d33, 0x8eb9, 0x8f51) get
             * the full per-write log + NDB+DARAM dumps. Other PCs (e.g.
             * 0xb906 push site, 0x7763/0x7764 SP-overflow) get a counted
             * one-line tag so we don't lose visibility on them, but they
             * stop polluting the watch stream. */
            bool real_fbdet = (s->pc == 0x8d33 || s->pc == 0x8eb9 ||
                               s->pc == 0x8f51);
            /* FBDET-DIVERSITY: count distinct values per 1M-insn window.
             * 1 = DSP pegged on stale data. >5 = real scan. Discriminates
             * "BSP delivers fresh I/Q" from "DSP recorrelates same window". */
            if (real_fbdet) {
                static uint16_t recent_vals[8] = {0};
                static unsigned next_window = 1000000;
                static int n_distinct = 0;
                int seen = 0;
                for (int i = 0; i < 8; i++) {
                    if (recent_vals[i] == val) { seen = 1; break; }
                }
                if (!seen) {
                    recent_vals[n_distinct & 7] = val;
                    n_distinct++;
                }
                if (s->insn_count >= next_window) {
                    C54_LOG("FBDET-DIVERSITY window=%uM distinct=%d",
                            next_window / 1000000, n_distinct);
                    n_distinct = 0;
                    for (int i = 0; i < 8; i++) recent_vals[i] = 0;
                    next_window = (s->insn_count / 1000000 + 1) * 1000000;
                }
            }
            if (real_fbdet && (fbd_log < 200 || (fbd_log % 1000) == 0)) {
                C54_LOG("DSP WR d_fb_det = 0x%04x (s=%d) PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                        val, (int)(int16_t)val, s->pc, s->insn_count,
                        s->prog[(uint16_t)(s->pc - 2)],
                        s->prog[(uint16_t)(s->pc - 1)],
                        s->prog[s->pc],
                        s->prog[(uint16_t)(s->pc + 1)]);
                C54_LOG("  NDB[0x08F0..0x0900]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        s->data[0x08F0], s->data[0x08F1], s->data[0x08F2], s->data[0x08F3],
                        s->data[0x08F4], s->data[0x08F5], s->data[0x08F6], s->data[0x08F7],
                        val,             s->data[0x08F9], s->data[0x08FA], s->data[0x08FB],
                        s->data[0x08FC], s->data[0x08FD], s->data[0x08FE], s->data[0x08FF],
                        s->data[0x0900]);
                if (fbd_log < 5) {
                    C54_LOG("  DARAM[0x3FB0..0x3FBF]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->data[0x3FB0], s->data[0x3FB1], s->data[0x3FB2], s->data[0x3FB3],
                            s->data[0x3FB4], s->data[0x3FB5], s->data[0x3FB6], s->data[0x3FB7],
                            s->data[0x3FB8], s->data[0x3FB9], s->data[0x3FBA], s->data[0x3FBB],
                            s->data[0x3FBC], s->data[0x3FBD], s->data[0x3FBE], s->data[0x3FBF]);
                }
            } else if (!real_fbdet) {
                static unsigned other_pc_count = 0;
                other_pc_count++;
                if (other_pc_count == 1 || other_pc_count == 100 ||
                    other_pc_count == 10000 || other_pc_count == 1000000) {
                    C54_LOG("d_fb_det NON-FBDET-PC write #%u val=0x%04x PC=0x%04x SP=0x%04x",
                            other_pc_count, val, s->pc, s->sp);
                }
            }
            /* === D_FB_DET ZERO-OVERRIDE TRACE ===
             * Race-window observed (memory `project_fbdet_threshold_blocker`):
             * DSP writes high SNR (e.g. 0x7902, 0x7766) at fb-det PCs, then
             * SOMETHING zeroes d_fb_det before ARM reads. ARM sees 200×
             * 0x0000 → no FB found → endless L1CTL_FBSB_REQ retries.
             *
             * Capture EVERY write of val=0 to 0x08F8 with full context so
             * we identify the zero-ifying PCs and reconstruct the condition
             * (threshold check, post-correlation reset, error path, etc.).
             * Cap 200 events. */
            if (val == 0) {
                static unsigned zero_log = 0;
                if (zero_log < 200) {
                    C54_LOG("D_FB_DET ZERO-WR #%u PC=0x%04x op=0x%04x prev=0x%04x "
                            "A=%010llx B=%010llx T=0x%04x ST0=0x%04x ST1=0x%04x insn=%u",
                            zero_log + 1,
                            s->pc, s->prog[s->pc],
                            s->data[0x08F8],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                            s->t, s->st0, s->st1, s->insn_count);
                    zero_log++;
                }
            }
            /* Transition trace : non-zero → zero (the override moment).
             * Logs whenever d_fb_det was non-zero just before this write
             * but the new write makes it zero. Cap 100. */
            if (val == 0 && s->data[0x08F8] != 0) {
                static unsigned override_log = 0;
                if (override_log < 100) {
                    C54_LOG("D_FB_DET OVERRIDE #%u prev=0x%04x → 0 PC=0x%04x op=0x%04x "
                            "A=%010llx ST0=0x%04x insn=%u",
                            override_log + 1,
                            s->data[0x08F8], s->pc, s->prog[s->pc],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                            s->st0, s->insn_count);
                    override_log++;
                }
            }
            /* === NEW 2026-05-15 : SET non-zero trace + SET→CLEAR delta ===
             *
             * Symétrique au ZERO-WR. Capture chaque write de val != 0 à 0x08F8
             * pour identifier QUI set le FB found, et combien de cycles ça
             * tient avant qu'un PC clear ne l'écrase.
             *
             * Si delta SET→CLEAR < 100 insn → bug opcode tape immédiatement
             * (style POPM fix). Si delta = milliers d'insn → race timing
             * légitime entre DSP set et ARM read.
             */
            {
                static uint64_t last_set_insn;
                static uint16_t last_set_val;
                static uint16_t last_set_pc;
                static unsigned set_log_n = 0;
                static unsigned delta_log_n = 0;
                if (val != 0) {
                    /* SET event */
                    if (set_log_n < 500) {
                        C54_LOG("D_FB_DET SET #%u val=0x%04x PC=0x%04x op=0x%04x "
                                "prev=0x%04x A=%010llx insn=%u",
                                set_log_n + 1,
                                val, s->pc, s->prog[s->pc],
                                s->data[0x08F8],
                                (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                                s->insn_count);
                        set_log_n++;
                    }
                    last_set_insn = s->insn_count;
                    last_set_val = val;
                    last_set_pc = s->pc;
                } else if (s->data[0x08F8] != 0 && last_set_insn != 0) {
                    /* CLEAR after non-zero — log delta */
                    uint64_t delta = (uint64_t)s->insn_count - last_set_insn;
                    if (delta_log_n < 100) {
                        C54_LOG("D_FB_DET SET-TO-CLEAR-DELTA #%u "
                                "set_PC=0x%04x set_insn=%llu set_val=0x%04x "
                                "clear_PC=0x%04x clear_insn=%u delta=%llu cycles",
                                delta_log_n + 1,
                                last_set_pc, (unsigned long long)last_set_insn,
                                last_set_val, s->pc, s->insn_count,
                                (unsigned long long)delta);
                        delta_log_n++;
                    }
                }
            }
            fbd_log++;
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    s->data[addr] = val;
}

/* Instruction fetch: uses mirrored PROM1 at 0x8000-0xFFFF, ignores XPC */
static uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    /* OVLY: map DARAM into program space 0x0080-0x27FF only.
     * Calypso has 10K words DARAM (0x0000-0x27FF).
     * PROM0 (0x7000-0xDFFF) is always accessible in program space. */
    if ((s->pmst & PMST_OVLY) && pc >= 0x80 && pc < 0x2800)
        return s->data[pc];
    /* prog_fetch: PC is always 16-bit, never uses XPC banking.
     * Per version 222807: only OVLY overlay applies to instruction fetch.
     * XPC is only used by prog_read (data/operand reads). */
    return s->prog[pc];
}
static uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* OVLY: DARAM visible in program space for 0x0080-0x27FF */
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        return s->data[addr16];
    /* For addresses >= 0x8000: use XPC to select extended page.
     * prog_read is used for data/operand reads (MVPD, FIRS coeff, etc.)
     * which need XPC banking — unlike prog_fetch which is PC-only. */
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        return s->prog[ext];
    }
    return s->prog[addr16];
}

static void __attribute__((unused)) prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM — reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        s->data[addr16] = val;
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* Resolve Smem operand: direct or indirect addressing.
 * Returns the data memory address. */
static uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing.
         * Per SPRU131G §5.4.1 Table 5-5: bits 2:0 = ARF select the AR for
         * THIS instruction. ARP (in ST0) is then updated to ARF for the
         * NEXT direct-Smem reference. Earlier this code used arp(s) for
         * cur_arp, which made every indirect insn operate on the
         * PREVIOUS insn's ARF — off-by-one. Symptoms: BANZD *AR1- after
         * STL *AR2+ would decrement AR2 instead of AR1 (BANZD test
         * against AR2 stayed non-zero forever, AR1 frozen). Diagnosed
         * via 5×500M-insn STATE-DUMP showing AR1=0x1c / AR2=0x2b0c
         * frozen across 2B insns at PC=0xa2c2..0xa2ca. */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = nar;
        uint16_t addr = s->ar[cur_arp];

        /* Post-modify */
        switch (mod) {
        case 0x0: /* *ARn */
            break;
        case 0x1: /* *ARn- */
            s->ar[cur_arp]--;
            break;
        case 0x2: /* *ARn+ */
            s->ar[cur_arp]++;
            break;
        case 0x3: /* *+ARn */
            addr = ++s->ar[cur_arp];
            break;
        case 0x4: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x5: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x6: /* *ARn-0B (bit-reversed) */
            /* Simplified: just subtract */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x7: /* *ARn+0B (bit-reversed) */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x8: /* *ARn-% (circular) */
            if (s->bk == 0) s->ar[cur_arp]--;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]--;
                if (s->ar[cur_arp] < base) s->ar[cur_arp] = base + s->bk - 1;
            }
            break;
        case 0x9: /* *ARn+% (circular) */
            if (s->bk == 0) s->ar[cur_arp]++;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]++;
                if (s->ar[cur_arp] >= base + s->bk) s->ar[cur_arp] = base;
            }
            break;
        case 0xA: /* *ARn-0% */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0xB: /* *ARn+0% */
            s->ar[cur_arp] += s->ar[0];
            break;
        /* Indirect modes 12..15 use a long-immediate operand from the next
         * program word. Encoding per tic54x-dis.c (MOD field = bits 6:3 of
         * the smem byte) and SPRU131G Table 5-9:
         *   12 : *AR(x)(lk)        — addr = AR(x) + lk, NO modify
         *   13 : *+AR(x)(lk)       — premod: AR(x) += lk; addr = AR(x)
         *   14 : *+AR(x)(lk)%      — premod circular: AR(x) = circ(AR(x)+lk)
         *   15 : *(lk)             — ABSOLUTE long address (lk itself)
         *
         * The bootloader at PROM0 0xb429 uses MOD=15 (`LDU *(0x0ffe), A`)
         * to read BL_ADDR_LO. Misdecoding 15 as "AR + lk circular"
         * produced AR0+0x0ffe instead of 0x0ffe — one of the multiple
         * subtle off-by-AR bugs that left A=0 after the load. */
        case 0xC: /* *AR(x)(lk) */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+AR(x)(lk) */
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            addr = s->ar[cur_arp];
            s->lk_used = true;
            break;
        case 0xE: { /* *+AR(x)(lk)% — circular */
            uint16_t lk = prog_fetch(s, s->pc + 1);
            uint16_t v  = s->ar[cur_arp] + lk;
            if (s->bk) {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                if (v >= base + s->bk) v -= s->bk;
            }
            s->ar[cur_arp] = v;
            addr = v;
            s->lk_used = true;
            break;
        }
        case 0xF: /* *(lk) — absolute address */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* Update ARP */
        s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing: DP:offset */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
        return (dp(s) << 7) | offset;
    }
}

/* SP ledger for IRQ-asymmetry diag (web 2026-05-23).
 * Pushes/pops counted by SP delta sign in dispatch loop (c54x_run).
 * IRQ entries counted explicitly in c54x_interrupt_ex with word count.
 * Periodic dump in dispatch loop shows whether net_words ≈ 0 (balanced)
 * or drifts (indicates push/pop word-count asymmetry, e.g. IRQ entry
 * pushes 1 word but FRET pops 2 → drift -1/IRQ-cycle → SP wraps). */
static struct {
    uint64_t sp_pushes;        /* SP delta < 0 events */
    uint64_t sp_pops;          /* SP delta > 0 events */
    int64_t  net_words;        /* total words pushed - total words popped */
    uint64_t irq_entries;      /* count of c54x_interrupt_ex actual dispatches */
    uint64_t irq_words_pushed; /* words written by IRQ entry path (1 or 2 per APTS) */
    uint64_t last_dump_insn;
} g_sp_ledger;

/* Xmem operand decode per binutils tic54x.h (XMEM/XMOD/XARX macros) :
 *   XMEM(OP) = bits [7:4] of opcode (the Xmem 4-bit nibble)
 *   XMOD    = nibble bits [3:2] : 0=*AR, 1=*AR-, 2=*AR+, 3=*AR+0%
 *   XARX    = nibble bits [1:0] + 2 (= AR2..AR5 only, no AR0/AR1/AR6/AR7)
 *
 * Xmem is INDIRECT-ONLY (no DP-relative direct mode, unlike Smem). Using
 * resolve_smem on an Xmem operand mis-decodes the low byte as Smem direct
 * addressing whenever bit 7 is clear, which lands writes in MMR space
 * (0x00-0x1F) — empirically observed at PC=0x8a46 op=0x9918 (STL B,*AR2)
 * 2026-05-23, stomp SP=0x4800→0x0000 cascading to IMR=0 → DSP idle forever.
 *
 * Known debt : xmod=3 (*AR+0%) implemented as linear `addr + AR0`, not
 * circular (modulo BK). Matches MVDD handler at L3724 which has same
 * `(no circular here)` comment. Circular addressing for all Xmem handlers
 * is tracked as known-open, to fix together. */
static uint16_t resolve_xmem(C54xState *s, uint16_t op)
{
    uint8_t xmem  = (op >> 4) & 0xF;
    int     xar   = (xmem & 0x3) + 2;
    int     xmod  = (xmem & 0xC) >> 2;
    uint16_t addr = s->ar[xar];
    switch (xmod) {
    case 0: break;
    case 1: s->ar[xar] = addr - 1; break;
    case 2: s->ar[xar] = addr + 1; break;
    case 3: s->ar[xar] = addr + s->ar[0]; break; /* *AR+0% (no circular here) */
    }
    return addr;
}

/* ================================================================
 * Instruction execution
 * ================================================================ */

/* Execute one instruction. Returns number of words consumed (1 or 2). */
/* PC ring buffer for pre-IDLE trace */
static uint16_t pc_ring[256];
static int pc_ring_idx = 0;

static int c54x_exec_one(C54xState *s)
{
    uint16_t op = prog_fetch(s, s->pc);
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;
    s->lk_used = false;  /* reset before each instruction */
    s->writer_kind = WK_UNKNOWN;  /* attribution tag for DATA-W-MMR */

    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* Coarse default: any MMR write happening inside this opcode handler
     * gets attributed to the opcode family so we can read the trace. */
    if (hi8 == 0xF3)                    s->writer_kind = WK_OPCODE_F3;
    else if (hi8 >= 0x80 && hi8 <= 0x8F) s->writer_kind = WK_OPCODE_8x;
    else if (hi8 == 0x77)                s->writer_kind = WK_OPCODE_77;
    else if (hi8 == 0x76)                s->writer_kind = WK_OPCODE_76;
    else                                 s->writer_kind = WK_OPCODE_OTHER;

    /* INTM-TRANS probe : log toute transition INTM 0→1.
     * Le SSBX INTM orphelin se cache entre insn=89.83M (last write 0x3dd2)
     * et insn=98.38M (entrée wait permanente). Cap à 200 transitions pour
     * éviter le flood au boot ; capture le PC qui a fait passer INTM à 1
     * et l'adresse de retour stack pour identifier le caller. */
    {
        static int prev_intm = -1;
        static unsigned itrans_total;
        int cur_intm = !!(s->st1 & ST1_INTM);
        if (prev_intm == 0 && cur_intm == 1) {
            itrans_total++;
            if (itrans_total <= 200) {
                uint16_t ret = s->data[s->sp];
                uint16_t ret_p1 = s->data[(uint16_t)(s->sp + 1)];
                fprintf(stderr,
                        "[c54x] INTM-TRANS #%u 0->1 PC=0x%04x insn=%u SP=0x%04x "
                        "RET=%04x RET+1=%04x op=0x%04x IMR=0x%04x IFR=0x%04x\n",
                        itrans_total, s->pc, s->insn_count, s->sp,
                        ret, ret_p1, op, s->imr, s->ifr);
            }
        }
        prev_intm = cur_intm;
    }

    /* Detect when DSP enters DARAM code zone (0x0080-0x27FF) from ROM */
    {
        static uint16_t prev_pc = 0;
        static int daram_log = 0;
        if (s->pc >= 0x0080 && s->pc < 0x2800 && prev_pc >= 0x7000 && daram_log < 3) {
            C54_LOG("ROM->DARAM jump: 0x%04x->0x%04x op=0x%04x insn=%u SP=0x%04x XPC=%d",
                    prev_pc, s->pc, op, s->insn_count, s->sp, s->xpc);
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
            daram_log++;
        }
        /* 0x7700 entry tracer: log when PC enters 0x7700 from elsewhere
         * (i.e. prev_pc != 0x76FF, the natural sequential predecessor).
         * Reveals which CALL/B/RET sources land here. PC HIST shows
         * 7700/7701 as the hottest non-loop addresses — find the callers. */
        if (s->pc == 0x7700 && prev_pc != 0x76FF) {
            static uint64_t e7700;
            e7700++;
            if (e7700 <= 30 || (e7700 % 5000) == 0) {
                C54_LOG("ENTER-7700 #%llu from PC=0x%04x A=%010llx B=%010llx SP=0x%04x trail: %04x %04x %04x %04x %04x",
                        (unsigned long long)e7700, prev_pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        pc_ring[(pc_ring_idx-5)&255], pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255], pc_ring[(pc_ring_idx-2)&255],
                        pc_ring[(pc_ring_idx-1)&255]);
            }
        }
        /* === ENTER-770c — dispatcher target, post-flag entry ===
         * The PROM0 idle dispatcher at 0xCC62..0xCC6F polls data[0x62];
         * when set, it CALAs to api[0x1f0c]=0x770c. So 0x770c is the
         * runtime task handler entry. If DARAM[0x60..0x70] never gets
         * set, this PC is never reached. Its appearance in the log is
         * therefore the binary signal that the dispatcher gate has
         * unlocked. Log every entry with full AR/SP/INTM context.
         * Cap to avoid log explosion if it ever runs hot. */
        if (s->pc == 0x770c) {
            static uint64_t e770c;
            e770c++;
            if (e770c <= 30 || (e770c % 1000) == 0) {
                C54_LOG("ENTER-770c #%llu from PC=0x%04x SP=0x%04x INTM=%d "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x insn=%u",
                        (unsigned long long)e770c, prev_pc, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->insn_count);
            }
        }
        /* === MVDD-CASCADE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * PC=0x8e8c op=0xe5ba = MVDD-family — documented cascade writer
         * (`project_dtaskd_corruption_8e8x`) that writes garbage values
         * into NDB cells (random vals at d_fb_det vs legitimate 0x001e).
         * Track AR fields + B accumulator + source address read to find
         * if it's true firmware compute or corrupted indirect addressing. */
        if (s->pc == 0x8e8c) {
            static int probe_mvdd = -1;
            if (probe_mvdd < 0) {
                const char *e = getenv("CALYPSO_PROBE_BOOTSTUB");
                probe_mvdd = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_mvdd) {
                static uint32_t e_mvdd;
                e_mvdd++;
                if (e_mvdd <= 50 || (e_mvdd % 1000) == 0) {
                    fprintf(stderr,
                            "[c54x] MVDD-CASCADE #%u PC=0x8e8c op=0x%04x SP=0x%04x "
                            "A=0x%010llx B=0x%010llx T=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "data[AR4]=0x%04x data[AR5]=0x%04x "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_mvdd, op, s->sp,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->t,
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->data[s->ar[4]], s->data[s->ar[5]],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === DF92-LOOP probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Compute loop at PC=0xdf92-0xdfa3 = correlator accumulator with
         * 15× unrolled ADD *AR7+. Called via CALL 0xdfb1 from 0xdf90.
         * Probe at first PC=0xdf92 (loop entry) — log AR7, BRC, accumulator,
         * caller (from stack[SP]). If AR7 is corrupted or BRC mis-set, the
         * loop runs forever and blocks task=24 scheduling downstream.
         * Fire only on entries from non-loop-internal predecessors. */
        if (s->pc == 0xdf92 && (prev_pc < 0xdf90 || prev_pc > 0xdfa3)) {
            static int probe_df = -1;
            if (probe_df < 0) {
                const char *e = getenv("CALYPSO_PROBE_BOOTSTUB");
                probe_df = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_df) {
                static uint32_t e_df;
                e_df++;
                if (e_df <= 30) {
                    fprintf(stderr,
                            "[c54x] DF92-LOOP #%u entry from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x ret_addr=stk[SP]=0x%04x "
                            "A=0x%010llx B=0x%010llx "
                            "AR7=0x%04x BK=0x%04x BRC=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x\n",
                            e_df, prev_pc, s->prog[prev_pc],
                            s->sp, s->data[s->sp],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[7], s->bk, s->brc,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === BL-REENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * The DSP bootloader at PC=0xb41c polls data[0x0fff] for cmd code
         * 4 or 2. Legitimate loop entry comes from 0xb427 (BC NTC 0xb41c)
         * or 0xb433 (CALL 0xb41c). Any OTHER entry path indicates the DSP
         * has been routed back into the bootloader by mistake after boot
         * has completed — that's the post-cascade blocker. Logs prev_pc,
         * SP, stack contents, ARs, and a trail to identify the bad caller.
         * Caps: 50 events to avoid log flood. */
        if (s->pc == 0xb41c && prev_pc != 0xb427 && prev_pc != 0xb433) {
            static int probe_bl = -1;
            if (probe_bl < 0) {
                const char *e = getenv("CALYPSO_PROBE_BOOTSTUB");
                probe_bl = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_bl) {
                static uint32_t e_bl;
                e_bl++;
                if (e_bl <= 50) {
                    fprintf(stderr,
                            "[c54x] BL-REENTRY #%u from PC=0x%04x prev_op=0x%04x "
                            "SP=0x%04x stk[SP..+3]= %04x %04x %04x %04x "
                            "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_bl, prev_pc, s->prog[prev_pc],
                            s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[0x0fff], s->data[0x0ffe],
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === SEED-SOURCE probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Probe at PC=0xf8de (CALA B → 0x7700) — the SINGLE source event
         * that spawns the entire boot-stub RET-loop cascade (per session
         * 2026-05-24 BOOTSTUB-ENTRY analysis : 1 ENTER-7700 → 435 entries
         * to PC=0x0000). Captures full state BEFORE the CALA fires :
         *   - SP + stack contents (what subsequent POPs will pull)
         *   - A, B (B = jump target)
         *   - AR0..AR7, ST0, ST1
         *   - 10-PC trail (extends visibility upstream of 0xf8de).
         * Goal: identify whether the function containing 0xf8de was itself
         * called with proper push, and what was supposed to be on stack
         * when POPM ST0 + RCD UNC fire at dispatcher 0x7706/0x7707. */
        if (s->pc == 0xf8de) {
            static int probe_seed = -1;
            if (probe_seed < 0) {
                const char *e = getenv("CALYPSO_PROBE_BOOTSTUB");
                probe_seed = (e && e[0] == '1') ? 1 : 0;
            }
            if (probe_seed) {
                static uint32_t e_seed;
                e_seed++;
                if (e_seed <= 50) {
                    fprintf(stderr,
                            "[c54x] SEED-SOURCE #%u PC=0xf8de op=0x%04x "
                            "SP=0x%04x  stk[SP..+7]= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "A=0x%010llx B=0x%010llx "
                            "AR= %04x %04x %04x %04x %04x %04x %04x %04x  "
                            "ST0=0x%04x ST1=0x%04x  "
                            "trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
                            e_seed, op, s->sp,
                            s->data[(uint16_t)(s->sp+0)], s->data[(uint16_t)(s->sp+1)],
                            s->data[(uint16_t)(s->sp+2)], s->data[(uint16_t)(s->sp+3)],
                            s->data[(uint16_t)(s->sp+4)], s->data[(uint16_t)(s->sp+5)],
                            s->data[(uint16_t)(s->sp+6)], s->data[(uint16_t)(s->sp+7)],
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                            s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                            s->st0, s->st1,
                            pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                            pc_ring[(pc_ring_idx-8)&255],  pc_ring[(pc_ring_idx-7)&255],
                            pc_ring[(pc_ring_idx-6)&255],  pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255],  pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255],  pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }

        /* === BOOTSTUB-ENTRY probe (env-gated CALYPSO_PROBE_BOOTSTUB=1) ===
         * Traces every entry to PC=0x0000 (boot stub LDMM SP,B + RET).
         * Boot stub re-entered at runtime is the documented-never-nailed
         * seed of the SP-wrap → AR6=0 → IMR=0 cascade. Captures :
         *   - prev_pc + op@prev_pc  → who jumped to 0x0000
         *   - entry mechanism (RET-family / branch / other)
         *   - B accumulator (becomes SP via LDMM SP,B at 0x0000)
         *   - SP + stk[SP-1] (just-popped value if RET)
         *   - 6-entry PC trail (caller context). */
        if (s->pc == 0x0000) {
            static int probe_bootstub = -1;
            if (probe_bootstub < 0) {
                const char *e = getenv("CALYPSO_PROBE_BOOTSTUB");
                probe_bootstub = (e && e[0] == '1') ? 1 : 0;
                if (probe_bootstub)
                    fprintf(stderr, "[c54x] PROBE-BOOTSTUB enabled\n");
            }
            if (probe_bootstub) {
                static uint32_t e0;
                e0++;
                if (e0 <= 200 || (e0 % 500) == 0) {
                    uint16_t prev_op = s->prog[prev_pc];
                    const char *mech;
                    if (prev_op == 0xFC00)                          mech = "RET";
                    else if (prev_op == 0xF273)                     mech = "RETD";
                    else if (prev_op == 0xF4EB || prev_op == 0xF4E3) mech = "RETE";
                    else if (prev_op == 0xF4E4 || prev_op == 0xF4E5) mech = "FRET";
                    else if ((prev_op & 0xFF00) == 0xF800)          mech = "B/CC";
                    else if ((prev_op & 0xFF00) == 0xF000)          mech = "F0xx";
                    else if (prev_op == 0xF074)                     mech = "CALL";
                    else                                            mech = "OTHER";
                    /* Just-popped slot is at SP-1 after RET (SP was incremented). */
                    uint16_t stk_just_popped = s->data[(uint16_t)(s->sp - 1)];
                    fprintf(stderr,
                            "[c54x] BOOTSTUB-ENTRY #%u prev_PC=0x%04x prev_op=0x%04x "
                            "mech=%s B=0x%010llx B[31:16]=0x%04x SP=0x%04x "
                            "stk[SP-1]=0x%04x trail: %04x %04x %04x %04x %04x %04x\n",
                            e0, prev_pc, prev_op, mech,
                            (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                            (unsigned)((s->b >> 16) & 0xFFFF),
                            s->sp, stk_just_popped,
                            pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                            pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                            pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                }
            }
        }
        /* D_FB_DET-WR-SITE probe : à PC=0x8f51 (le PC qui écrit d_fb_det).
         * Snapshot AR0..AR7 + data[AR0/1/2] + BK + A pour identifier la
         * zone DARAM lue par le correlator FB-det au moment de produire
         * sa valeur d'output. Comparer la zone source avec le BSP DMA
         * target (default 0x3fb0..0x3fbf) :
         *   - zone source = BSP target → correlator lit bien les samples
         *   - zone source ≠ BSP target → mismatch source/sink, blocker
         *     structurel : DSP attend les samples ailleurs que là où le
         *     BSP les écrit. Suite : tracer init AR, table coeffs, ou
         *     MAC sur autre buffer. */
        /* COEFFS-TABLE-DUMP : 1× au tout début + à chaque sweep FB-det.
         * Dump data[0x2bc0..0x2bcF] (zone censée contenir les coefficients
         * du correlator selon AR4 observé). 2026-05-14 : capture étendue
         * D_FB_DET-WR-SITE a révélé data[AR4]=0x0000 sur 50 hits → la table
         * de coeffs est VIDE en mémoire. Vérifier ici si elle l'est aussi
         * en boot et si quelqu'un l'écrit jamais. */
        {
            static int coeffs_log_n;
            static uint64_t coeffs_last_insn;
            bool first_call = (coeffs_log_n == 0);
            bool periodic = (s->insn_count - coeffs_last_insn > 1000000);
            bool at_8f51 = (s->pc == 0x8f51);
            if ((first_call || periodic || at_8f51) && coeffs_log_n < 30) {
                coeffs_log_n++;
                coeffs_last_insn = s->insn_count;
                C54_LOG("COEFFS-DUMP #%d insn=%u PC=0x%04x "
                        "data[0x2bc0..0x2bcF]= %04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x",
                        coeffs_log_n, s->insn_count, s->pc,
                        s->data[0x2bc0], s->data[0x2bc1], s->data[0x2bc2], s->data[0x2bc3],
                        s->data[0x2bc4], s->data[0x2bc5], s->data[0x2bc6], s->data[0x2bc7],
                        s->data[0x2bc8], s->data[0x2bc9], s->data[0x2bca], s->data[0x2bcb],
                        s->data[0x2bcc], s->data[0x2bcd], s->data[0x2bce], s->data[0x2bcf]);
            }
        }
        if (s->pc == 0x8f51) {
            /* Cap bumpé 50 → 500 (2026-05-14 night) pour couvrir plusieurs
             * sweeps FB-det au lieu du seul premier. + stats agrégées sur
             * tous les fires (cap n'est que pour le log per-fire). */
            static int dfbwr_n;
            g_fb_det_timing.fb_det_total++;
            uint16_t ar4 = s->ar[4];
            uint16_t dAR4 = s->data[ar4];
            uint16_t ar3 = s->ar[3];
            bool ar4_in_zone = (ar4 >= 0x2bc0 && ar4 <= 0x2bff);
            if (ar4_in_zone) g_fb_det_timing.fb_det_ar4_in_zone++;
            else             g_fb_det_timing.fb_det_ar4_outside++;
            if (dAR4 == 0x0000)      g_fb_det_timing.fb_det_dar4_zero++;
            else if (dAR4 == 0xfffe) g_fb_det_timing.fb_det_dar4_sentinel++;
            else                     g_fb_det_timing.fb_det_dar4_other++;
            /* Sweep boundary detection : AR3 retombe en dessous de la
             * dernière valeur observée → nouveau sweep commence.
             * Log le sweep précédent (count non-zero + A final + insn). */
            uint64_t A_lo = (uint64_t)(s->a & 0xFFFFFFFFFFULL);
            if (ar3 < g_fb_det_timing.last_ar3_at_fire
                && g_fb_det_timing.last_ar3_at_fire > 0) {
                C54_LOG("D_FB_DET-SWEEP id=%llu nonzero=%llu/50 "
                        "A_final=0x%010llx insn=%u",
                        (unsigned long long)g_fb_det_timing.sweep_id,
                        (unsigned long long)g_fb_det_timing.sweep_nonzero_count,
                        (unsigned long long)A_lo, s->insn_count);
                g_fb_det_timing.sweep_id++;
                g_fb_det_timing.sweep_nonzero_count = 0;
            }
            g_fb_det_timing.last_ar3_at_fire = ar3;
            if (dAR4 != 0) g_fb_det_timing.sweep_nonzero_count++;
            int64_t delta_compute = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_compute_insn;
            int64_t delta_clear   = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_clear_insn;
            int64_t delta_pattern = (int64_t)s->insn_count -
                                    (int64_t)g_fb_det_timing.last_pattern_insn;
            if (dfbwr_n++ < 500) {
                C54_LOG("D_FB_DET-WR-SITE #%d AR0..AR7=%04x %04x %04x %04x %04x %04x %04x %04x "
                        "data[AR0]=%04x data[AR1]=%04x data[AR2]=%04x "
                        "data[AR3]=%04x data[AR4]=%04x data[AR5]=%04x "
                        "data[AR6]=%04x data[AR7]=%04x "
                        "BK=%04x A=0x%010llx "
                        "ar4_in_zone=%d dcompute=%lld dclear=%lld dpattern=%lld "
                        "last_compute_addr=0x%04x last_clear_addr=0x%04x "
                        "last_pattern_addr=0x%04x "
                        "insn=%u",
                        dfbwr_n, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->data[s->ar[0]], s->data[s->ar[1]], s->data[s->ar[2]],
                        s->data[s->ar[3]], s->data[s->ar[4]], s->data[s->ar[5]],
                        s->data[s->ar[6]], s->data[s->ar[7]],
                        s->bk, (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        ar4_in_zone ? 1 : 0,
                        (long long)delta_compute, (long long)delta_clear,
                        (long long)delta_pattern,
                        g_fb_det_timing.last_compute_addr,
                        g_fb_det_timing.last_clear_addr,
                        g_fb_det_timing.last_pattern_addr,
                        s->insn_count);
            }
            /* Stats summary toutes les 100 fires de 0x8f51 — distribution
             * AR4-in-zone + histogramme val[AR4] sur tout l'historique. */
            if ((g_fb_det_timing.fb_det_total % 100) == 0) {
                C54_LOG("D_FB_DET-STATS total=%llu "
                        "ar4_in_zone=%llu outside=%llu "
                        "dar4_zero=%llu sentinel=%llu other=%llu",
                        (unsigned long long)g_fb_det_timing.fb_det_total,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_in_zone,
                        (unsigned long long)g_fb_det_timing.fb_det_ar4_outside,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_zero,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_sentinel,
                        (unsigned long long)g_fb_det_timing.fb_det_dar4_other);
            }
        }
        /* READ-AMONT probe : à chaque trigger PC (sites d_fb_det), émet delta
         * des reads par plage depuis le trigger précédent. Tranche entre :
         *   - dominant LOW    → correlator lit la zone [0..0x3A3]
         *   - dominant APIRAM → samples viennent via API RAM (ARM-driven)
         *   - dominant WRAP   → correlator tourne sur le wrap PROM1 mirror
         *   - dominant OTHER  → zone non cataloguée à identifier */
        read_stats_trigger_check(s);
        throughput_tick(s->insn_count);
        /* WAIT-A21A probe : à PC=0xa21a, snapshot INTM + IMR + IFR.
         * Tranche H1/H2/H3 :
         *   INTM=1 + IFR=0  + IMR plein → H3 strict, hardware silencieux
         *   INTM=1 + IFR≠0  + IMR plein → H3 + IRQ pending bloquée (BUG)
         *   INTM=0                       → H1/H2 (IRQ servable mais path
         *                                  vers 0x7740 cassé en amont) */
        if (s->pc == 0xa21a) {
            static uint64_t a21a_total;
            a21a_total++;
            if (a21a_total <= 5 || (a21a_total % 100000) == 0) {
                C54_LOG("WAIT-A21A #%llu insn=%u INTM=%d IMR=0x%04x IFR=0x%04x "
                        "ST0=0x%04x ST1=0x%04x SP=0x%04x",
                        (unsigned long long)a21a_total, s->insn_count,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->st0, s->st1, s->sp);
            }
        }
        /* CALLER-7740 tracer : à l'entrée 0x7740, log le contexte caller.
         * data[sp] = adresse de retour pushée par le CALL/CALLD précédent.
         * INTM=1 → on est dans un IRQ context. Permet de distinguer
         * "appelé via IRQ ISR" vs "appelé via flow régulier", et de
         * remonter la chaîne caller→callee jusqu'à l'IRQ vector. */
        if (s->pc == 0x7740) {
            static uint64_t enter7740;
            enter7740++;
            uint16_t ret_addr = s->data[s->sp];
            uint16_t ret_addr_p1 = s->data[(uint16_t)(s->sp + 1)];
            C54_LOG("ENTER-7740 #%llu insn=%u SP=%04x RET=%04x RET+1=%04x "
                    "INTM=%d XPC=%02x AR2=%04x AR3=%04x BK=%04x",
                    (unsigned long long)enter7740, s->insn_count,
                    s->sp, ret_addr, ret_addr_p1,
                    !!(s->st1 & ST1_INTM), s->xpc,
                    s->ar[2], s->ar[3], s->bk);
        }
        /* MAC-7700 tracer: at PC=0x7700 (MAC *AR2-, A) we want to know
         * what AR2 points at, what data[AR2] holds, T, and A before/after.
         * Helps determine if AR2 references the BSP RX zone (correlator
         * FB-det) or somewhere else. Also dumps full AR0-AR7 + ST0/ST1. */
        if (s->pc == 0x7700) {
            static uint64_t mac7700_total;
            mac7700_total++;
            if (mac7700_total <= 50 || (mac7700_total % 5000) == 0) {
                uint16_t ar2 = s->ar[2];
                uint16_t v_at_ar2 = s->data[ar2];
                C54_LOG("MAC-7700 #%llu AR2=0x%04x data[AR2]=0x%04x T=0x%04x "
                        "A_pre=%010llx ST0=0x%04x ST1=0x%04x",
                        (unsigned long long)mac7700_total, ar2, v_at_ar2,
                        s->t,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->st0, s->st1);
                C54_LOG("MAC-7700 #%llu ARs: AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x",
                        (unsigned long long)mac7700_total,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp);
            }
        }
        /* RCD-75e8 tracer: when DSP arrives at PC=0x75e8 (cond=0x47 = LEQ),
         * log A. The RCD takes if A <= 0; report whether the loop will
         * exit this iteration. */
        if (s->pc == 0x75e8) {
            static uint64_t rcd75e8_total;
            rcd75e8_total++;
            if (rcd75e8_total <= 50 || (rcd75e8_total % 5000) == 0) {
                int64_t acc = sext40(s->a);
                C54_LOG("RCD-75e8 #%llu A=%010llx (signed=%lld) RCD-taken=%d AR2=%04x",
                        (unsigned long long)rcd75e8_total,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (long long)acc, (acc <= 0), s->ar[2]);
            }
        }
        prev_pc = s->pc;
        /* DARAM 0x1100-0x1130 tracer: dump first 64 visits */
        static int daram1110_log = 0;
        if (s->pc >= 0x1100 && s->pc <= 0x1130 && daram1110_log < 64) {
            C54_LOG("DARAM110x PC=0x%04x op=0x%04x A=%08x B=%08x AR2=%04x AR3=%04x AR4=%04x AR5=%04x BRC=%d",
                    s->pc, op, (uint32_t)s->a, (uint32_t)s->b,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->brc);
            daram1110_log++;
        }
    }
    if (s->pc >= 0xFE00 && s->pc <= 0xFFFF && op == 0x0000) {
        static int nop_slide = 0;
        if (nop_slide == 0) {
            C54_LOG("NOP-SLIDE PC=0x%04x insn=%u SP=0x%04x PMST=0x%04x XPC=%d OVLY=%d",
                    s->pc, s->insn_count, s->sp, s->pmst, s->xpc, !!(s->pmst & PMST_OVLY));
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
        }
        nop_slide++;
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed + s->lk_used;  /* NOP */

        /* XC n, cond — Execute Conditionally (SPRU172C p.4-198)
         * Opcode: 1111 11N1 CCCCCCCC
         * 0xFDxx = XC 1, cond (N=0, execute next 1 instruction)
         * 0xFFxx = XC 2, cond (N=1, execute next 2 instructions)
         * If condition true: execute normally. If false: skip n instructions. */
        if (hi8 == 0xFD || hi8 == 0xFF) {
            int n_insns = (hi8 == 0xFF) ? 2 : 1;
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition code per SPRU172C condition table */
            /* Conditions can be combined (OR'd bits), but common single conditions: */
            if (cc == 0x00)      cond = true;                          /* UNC */
            else if (cc == 0x0C) cond = (s->st0 & ST0_C) != 0;       /* C */
            else if (cc == 0x08) cond = !(s->st0 & ST0_C);            /* NC */
            else if (cc == 0x30) cond = (s->st0 & ST0_TC) != 0;       /* TC */
            else if (cc == 0x20) cond = !(s->st0 & ST0_TC);           /* NTC */
            else if (cc == 0x45) cond = (sext40(s->a) == 0);          /* AEQ */
            else if (cc == 0x44) cond = (sext40(s->a) != 0);          /* ANEQ */
            else if (cc == 0x46) cond = (sext40(s->a) > 0);           /* AGT */
            else if (cc == 0x42) cond = (sext40(s->a) >= 0);          /* AGEQ */
            else if (cc == 0x43) cond = (sext40(s->a) < 0);           /* ALT */
            else if (cc == 0x47) cond = (sext40(s->a) <= 0);          /* ALEQ */
            else if (cc == 0x4D) cond = (sext40(s->b) == 0);          /* BEQ */
            else if (cc == 0x4C) cond = (sext40(s->b) != 0);          /* BNEQ */
            else if (cc == 0x4E) cond = (sext40(s->b) > 0);           /* BGT */
            else if (cc == 0x4A) cond = (sext40(s->b) >= 0);          /* BGEQ */
            else if (cc == 0x4B) cond = (sext40(s->b) < 0);           /* BLT */
            else if (cc == 0x4F) cond = (sext40(s->b) <= 0);          /* BLEQ */
            else if (cc == 0x70) cond = (s->st0 & ST0_OVA) != 0;     /* AOV */
            else if (cc == 0x60) cond = !(s->st0 & ST0_OVA);          /* ANOV */
            else if (cc == 0x78) cond = (s->st0 & ST0_OVB) != 0;     /* BOV */
            else if (cc == 0x68) cond = !(s->st0 & ST0_OVB);          /* BNOV */
            else {
                /* Combined conditions: OR the individual condition bits */
                cond = false;
                if (cc & 0x0C) cond |= ((cc & 0x04) ? (s->st0 & ST0_C) != 0 : !(s->st0 & ST0_C));
                if (cc & 0x30) cond |= ((cc & 0x10) ? (s->st0 & ST0_TC) != 0 : !(s->st0 & ST0_TC));
                if (cc & 0x40) {
                    int64_t acc = (cc & 0x08) ? s->b : s->a;
                    int c3 = cc & 0x07;
                    switch (c3) {
                    case 0x5: cond |= (sext40(acc) == 0); break;
                    case 0x4: cond |= (sext40(acc) != 0); break;
                    case 0x6: cond |= (sext40(acc) > 0); break;
                    case 0x2: cond |= (sext40(acc) >= 0); break;
                    case 0x3: cond |= (sext40(acc) < 0); break;
                    case 0x7: cond |= (sext40(acc) <= 0); break;
                    default: cond = true; break;
                    }
                }
                if (cc & 0x70 && !(cc & 0x40)) {
                    if (cc & 0x08) cond |= (s->st0 & ST0_OVB) != 0;
                    else           cond |= (s->st0 & ST0_OVA) != 0;
                }
            }
            if (!cond) {
                /* Skip n instructions — count consumed words for skipped insns */
                /* Each skipped insn is 1 word (simplified — multi-word insns rare after XC) */
                return 1 + n_insns;
            }
            return consumed + s->lk_used;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
        /* F4E2 = BACC A, F5E2 = BACC B (per tic54x-opc.c, mask 0xFEFF) */
        /* F4E3 = CALA A, F5E3 = CALA B — push next-PC, jump to acc low 16 bits */
        /* DYN-CALL tracer: targets are computed at runtime, invisible to static
         * disasm. Log every BACC/CALA, plus an extra hot tag when the target
         * lands in any FB-det zone (PROM0 0x77xx-0x79xx, 0x88xx, 0xa0xx-0xa1xx). */
        if (op == 0xF4E2 || op == 0xF5E2 || op == 0xF4E3 || op == 0xF5E3) {
            int is_b = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            uint16_t tgt = (uint16_t)((is_b ? s->b : s->a) & 0xFFFF);
            uint16_t src_pc = s->pc;
            int fb_zone = (tgt >= 0x7730 && tgt <= 0x7990) ||
                          (tgt >= 0x8800 && tgt <= 0x88FF) ||
                          (tgt >= 0xA000 && tgt <= 0xA1FF);
            static uint64_t dyn_total = 0;
            static uint64_t dyn_fb = 0;
            dyn_total++;
            if (fb_zone) dyn_fb++;
            /* When OVLY=1 and src_pc in [0x80, 0x2800], the executed opcode
             * comes from data[] (DARAM), not prog[]. Reflect this in the
             * dump so we see the *actual* bytes that drove the CALA. */
            int ovly_active = (s->pmst & PMST_OVLY) && src_pc >= 0x80 && src_pc < 0x2800;
            uint16_t m0 = ovly_active ? s->data[(uint16_t)(src_pc - 2)] : s->prog[(uint16_t)(src_pc - 2)];
            uint16_t m1 = ovly_active ? s->data[(uint16_t)(src_pc - 1)] : s->prog[(uint16_t)(src_pc - 1)];
            uint16_t m2 = ovly_active ? s->data[src_pc] : s->prog[src_pc];
            uint16_t m3 = ovly_active ? s->data[(uint16_t)(src_pc + 1)] : s->prog[(uint16_t)(src_pc + 1)];
            if (dyn_total <= 200 || fb_zone || (dyn_total % 5000) == 0) {
                C54_LOG("DYN-CALL #%llu %s%c src=0x%04x tgt=0x%04x A=%010llx B=%010llx SP=0x%04x mem[%c]=%04x %04x %04x %04x%s",
                        (unsigned long long)dyn_total,
                        is_call ? "CALA" : "BACC",
                        is_b ? 'B' : 'A',
                        src_pc, tgt,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        ovly_active ? 'D' : 'P',
                        m0, m1, m2, m3,
                        fb_zone ? " *FB-ZONE*" : "");
            }
            if (is_call) {
                uint16_t ret_pc = src_pc + 1;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->pc = tgt;
            return 0;
        }
        /* F4E0-F4FF: RSBX/SSBX status bits — treat as NOP (most don't affect emulation) */
        if (op >= 0xF4E0 && op <= 0xF4FF && op != 0xF4E4 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt). Pop PC, pop XPC iff APTS=1.
         * Symmetric with c54x_interrupt_ex push order. */
        if (op == 0xF4EB) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            if (s->pmst & PMST_APTS) {
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
            }
            s->st1 &= ~ST1_INTM;
            {
                static uint64_t rete_count;
                rete_count++;
                if (rete_count <= 20 || (rete_count % 100) == 0)
                    C54_LOG("RETE #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)rete_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra; return 0;
        }
        /* 0xF4E4 = FRET (far return). Pop PC + XPC unconditionally.
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C Table 2-15:
         *   FRET[D]: XPC = TOS, ++SP, PC = TOS, ++SP
         * Symmetric with FCALL/FCALLD push (also unconditional, see below).
         * 2026-04-28 — fixed: was conditional on PMST_APTS (bit 4) which is
         * actually AVIS (Address Visibility) per SPRU131G — has no stack
         * semantics. The misnomer caused FRET to skip XPC pop when AVIS=0,
         * leading to stack imbalance against FCALL FAR which always pushes 2. */
        if (op == 0xF4E4) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            s->xpc = data_read(s, s->sp); s->sp++;
            if (s->xpc > 3) s->xpc &= 3;
            {
                static uint64_t fret_count;
                fret_count++;
                if (fret_count <= 30 || (fret_count % 1000) == 0)
                    C54_LOG("FRET #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)fret_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra;
            return 0;
        }
        /* IDLE 1/2/3: 0xF4E1, 0xF5E1, 0xF6E1, 0xF7E1 (mask 0xFCFF) */
        if ((op & 0xFCFF) == 0xF4E1) {
            int level = ((op >> 8) & 0x3) + 1;
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE%d @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        level, s->pc, !!(s->st1 & ST1_INTM),
                        s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            s->idle = true;
            return 0;
        }
        /* ================================================================
         * F[4-7]xx generic accumulator family — promoted from F4 block
         * to handle F5/F6/F7 variants. Handlers use bits 8/9 for src/dst,
         * with masks FCE0/FCFF/FEFF naturally covering all 4 combinations
         * (A->A, B->A, A->B, B->B). The matching handler bodies remain
         * inside the F4 block as dead code (never reached for arith ops
         * because of the early return here). 2026-04-28.
         * ================================================================ */
            /* F483/F583: SAT src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF483) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                return consumed + s->lk_used;
            }

            /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF484) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                return consumed + s->lk_used;
            }

            /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF485) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                if (val < 0) val = -val;
                if (dst) s->b = sext40(val); else s->a = sext40(val);
                return consumed + s->lk_used;
            }

            /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
             * Multiply T * A(high), accumulate into dst */
            if ((op & 0xFEFF) == 0xF48C) {
                int dst = (op >> 8) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF48D) {
                int dst = (op >> 8) & 1;
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t prod = (int64_t)ah * (int64_t)ah;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                return consumed + s->lk_used;
            }

            /* F48E/F58E: EXP src (mask FEFF, 1 word)
             * Count leading sign bits of accumulator, store in T */
            if ((op & 0xFEFF) == 0xF48E) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                int exp = 0;
                if (val == 0 || val == -1) { exp = 31; }
                else {
                    uint64_t uv = (val < 0) ? ~val : val;
                    uv &= 0xFFFFFFFFFFULL;
                    /* Count leading zeros from bit 38 down */
                    for (int i = 38; i >= 0; i--) {
                        if (uv & (1ULL << i)) break;
                        exp++;
                    }
                    exp -= 8; /* EXP = leading sign bits - 8 */
                }
                s->t = (uint16_t)(int16_t)exp;
                return consumed + s->lk_used;
            }

            /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
            if ((op & 0xFEFF) == 0xF492) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
            if ((op & 0xFEFF) == 0xF493) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
            if ((op & 0xFEFF) == 0xF49E) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val >= 0) { *acc = sext40((val << 1) + 1); }
                else { *acc = sext40(val << 1); }
                return consumed + s->lk_used;
            }

            /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
             * Per SPRU172C p.4-118: if the two MSBs of src accumulator
             * are different (not sign-extended), shift src left by 1
             * and decrement T. Otherwise do nothing. Used by the FB-det
             * correlator to normalize results; the loop exits when
             * NORM stops shifting (MSBs match = value is normalized). */
            if ((op & 0xFEFF) == 0xF48F) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                /* Check bits 39 and 38 — if they differ, shift left */
                int bit39 = (val >> 39) & 1;
                int bit38 = (val >> 38) & 1;
                if (bit39 != bit38) {
                    val = sext40(val << 1);
                    if (src) s->b = val; else s->a = val;
                    s->t = (uint16_t)(s->t - 1);
                }
                /* TC flag: set if shift occurred */
                if (bit39 != bit38)
                    s->st0 |= ST0_TC;
                else
                    s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            /* F490/F590: ROR src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF490) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1; /* carry */
                uint16_t lsb = *acc & 1;
                *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F491/F591: ROL src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF491) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1;
                uint16_t msb = (*acc >> 39) & 1;
                *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF488) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF486) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                return consumed + s->lk_used;
            }

            /* F487/F587: RND src (round, mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF487) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(*acc + 0x8000);
                return consumed + s->lk_used;
            }

            /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF480) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF481) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF482) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            /* F4xx accumulator shift/load (1-word, mask FCE0):
             * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
            if ((op & 0xFCE0) == 0xF400) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF420) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF440) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF460) {
                /* SFTA src,shift,dst — arithmetic shift accumulator */
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF4A0) {
                /* SFTL src,shift,dst — logical shift accumulator */
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                uv &= 0xFFFFFFFFFFULL;
                if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                return consumed + s->lk_used;
            }

        /* F494/F594: SFTC src (mask FEFF, 1 word).
         * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
         * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
         * — without it the correlator sums never normalise. */
        if ((op & 0xFEFF) == 0xF494) {
            int src = (op >> 8) & 1;
            int64_t *acc = src ? &s->b : &s->a;
            int64_t val = sext40(*acc);
            if (val != 0) {
                int b31 = (val >> 31) & 1;
                int b30 = (val >> 30) & 1;
                if (b31 == b30) *acc = sext40(val << 1);
            }
            return consumed + s->lk_used;
        }

        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call and special instructions.
             * Some F4xx instructions are 1-word (FRET, FRETE, RETE, TRAP, NOP, etc.)
             * Must check specific opcodes BEFORE the 2-word switch. */

            /* Note: 0xF4E4 = IDLE (handled above, not FRET).
             * Real FRET = 0xF072 (algebraic), handled in F0xx section. */
            /* NOP — F495 per SPRU172C p.4-121 */
            if (op == 0xF495) {
                return 1; /* 1-word NOP */
            }
            /* TRAP K — F4C0-F4DF per SPRU172C p.4-195:
             * SP-1, PC+1 → TOS, vector(IPTR*128 + K*4) → PC */
            if ((op & 0xFFE0) == 0xF4C0) {
                int k = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + k * 4;
                C54_LOG("TRAP #%d → PC=0x%04x (from PC=0x%04x)", k, s->pc,
                        (uint16_t)(s->pc - (iptr * 0x80 + k * 4) + 1 - 1));
                return 0;
            }

            /* F4xx arithmetic instructions (1-word, per tic54x-opc.c).
             * These MUST be checked before the 2-word branch/call switch. */
            {
                /* F483/F583: SAT src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF483) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                    else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                    return consumed + s->lk_used;
                }
                /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF484) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                    return consumed + s->lk_used;
                }
                /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF485) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (val < 0) val = -val;
                    if (dst) s->b = sext40(val); else s->a = sext40(val);
                    return consumed + s->lk_used;
                }
                /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
                 * Multiply T * A(high), accumulate into dst */
                if ((op & 0xFEFF) == 0xF48C) {
                    int dst = (op >> 8) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF48D) {
                    int dst = (op >> 8) & 1;
                    int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                    int64_t prod = (int64_t)ah * (int64_t)ah;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                    return consumed + s->lk_used;
                }
                /* F48E/F58E: EXP src (mask FEFF, 1 word)
                 * Count leading sign bits of accumulator, store in T */
                if ((op & 0xFEFF) == 0xF48E) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    int exp = 0;
                    if (val == 0 || val == -1) { exp = 31; }
                    else {
                        uint64_t uv = (val < 0) ? ~val : val;
                        uv &= 0xFFFFFFFFFFULL;
                        /* Count leading zeros from bit 38 down */
                        for (int i = 38; i >= 0; i--) {
                            if (uv & (1ULL << i)) break;
                            exp++;
                        }
                        exp -= 8; /* EXP = leading sign bits - 8 */
                    }
                    s->t = (uint16_t)(int16_t)exp;
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM — handled below (real implementation, not NOP) */
                /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
                if ((op & 0xFEFF) == 0xF492) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
                if ((op & 0xFEFF) == 0xF493) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
                if ((op & 0xFEFF) == 0xF49E) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val >= 0) { *acc = sext40((val << 1) + 1); }
                    else { *acc = sext40(val << 1); }
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
                 * Per SPRU172C p.4-118: if the two MSBs of src accumulator
                 * are different (not sign-extended), shift src left by 1
                 * and decrement T. Otherwise do nothing. Used by the FB-det
                 * correlator to normalize results; the loop exits when
                 * NORM stops shifting (MSBs match = value is normalized). */
                if ((op & 0xFEFF) == 0xF48F) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    /* Check bits 39 and 38 — if they differ, shift left */
                    int bit39 = (val >> 39) & 1;
                    int bit38 = (val >> 38) & 1;
                    if (bit39 != bit38) {
                        val = sext40(val << 1);
                        if (src) s->b = val; else s->a = val;
                        s->t = (uint16_t)(s->t - 1);
                    }
                    /* TC flag: set if shift occurred */
                    if (bit39 != bit38)
                        s->st0 |= ST0_TC;
                    else
                        s->st0 &= ~ST0_TC;
                    return consumed + s->lk_used;
                }
                /* F49F: DELAY (pipeline flush, NOP) */
                if (op == 0xF49F) { return consumed + s->lk_used; }
                /* F490/F590: ROR src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF490) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1; /* carry */
                    uint16_t lsb = *acc & 1;
                    *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                    if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F491/F591: ROL src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF491) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1;
                    uint16_t msb = (*acc >> 39) & 1;
                    *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                    if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF488) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF486) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                    return consumed + s->lk_used;
                }
                /* F487/F587: RND src (round, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF487) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(*acc + 0x8000);
                    return consumed + s->lk_used;
                }
                /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF480) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF481) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF482) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* F4xx accumulator shift/load (1-word, mask FCE0):
                 * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
                if ((op & 0xFCE0) == 0xF400) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF420) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF440) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF460) {
                    /* SFTA src,shift,dst — arithmetic shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF4A0) {
                    /* SFTL src,shift,dst — logical shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                    if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                    uv &= 0xFFFFFFFFFFULL;
                    if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                    return consumed + s->lk_used;
                }
            }
                        /* F4Bx: RSBX -- reset bit in ST0 (bit 9=0, bit 8=0).
             * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF4B0) {
                int bit = op & 0x0F;
                s->st0 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* F494/F594: SFTC src (mask FEFF, 1 word).
             * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
             * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
             * — without it the correlator sums never normalise. */
            if ((op & 0xFEFF) == 0xF494) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val != 0) {
                    int b31 = (val >> 31) & 1;
                    int b30 = (val >> 30) & 1;
                    if (b31 == b30) *acc = sext40(val << 1);
                }
                return consumed + s->lk_used;
            }
            /* Remaining F4xx: unhandled — treat as 1-word NOP */
            C54_LOG("F4xx unhandled: 0x%04x PC=0x%04x", op, s->pc);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xF0 || hi8 == 0xF1) {
            /* FIRS -- catch before other F1xx handlers */
            if (hi8 == 0xF1) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t xval = data_read(s, s->ar[xar]);
                uint16_t yval = data_read(s, s->ar[yar]);
                uint8_t xmod = (op >> 4) & 0xF;
                if ((xmod & 0x1) == 0) s->ar[xar]++; else s->ar[xar]--;
                if ((op & 0x08) == 0) s->ar[yar]++; else s->ar[yar]--;
                int16_t coeff = (int16_t)prog_read(s, op2);
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t product = (int64_t)ah * (int64_t)coeff;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->b = sext40(s->b + product);
                int32_t sum = (int32_t)(int16_t)xval + (int32_t)(int16_t)yval;
                s->a = sext40((int64_t)sum << 16);
                return consumed + s->lk_used;
            }
            /* F073: B pmad — unconditional branch (2-word).
             * Per tic54x-opc.c: 0xF073 mask 0xFFFF. */
            if (op == 0xF073) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F074: CALL pmad — unconditional call (2-word).
             * Per tic54x-opc.c: call 0xF074 mask 0xFFFF.
             * Push PC+2 (return address), branch to pmad.
             * NOTE: RETE is 0xF4EB (already handled above), NOT F074. */
            if (op == 0xF074) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }







            /* F072: RPTB pmad — block repeat (2-word, non-delayed).
             * Per tic54x-opc.c: 0xF072 mask 0xFFFF.
             * RSA = PC+2, REA = pmad. */
            if (op == 0xF072) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            /* F07x: RPT/RPTZ/misc (F072-F074 handled above) */
            if (op == 0xF070) {
                /* F070: RPT #lku — repeat next instruction lku+1 times (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if (op == 0xF071) {
                /* F071: RPTZ dst, #lku — zero accumulator and repeat (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int dst = (op >> 8) & 1; /* bit 8 via FEFF mask */
                if (dst) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if ((op & 0xFFF0) == 0xF070) {
                /* F075-F07F: undefined, treat as 1-word NOP */
                return consumed + s->lk_used;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed + s->lk_used;
            }
            /* F0xx/F1xx ALU with #lk immediate (2-word).
             * Per tic54x-opc.c: bits 7:4 = op (0=ADD,1=SUB,2=LD,3=AND,4=OR,5=XOR),
             * bit 8 = SRC (ADD/SUB/AND/OR/XOR) or DST (LD), bit 9 = DST,
             * bits 3:0 = shift. Second word = lk. */
            {
                uint8_t alu_op = (op >> 4) & 0xF;
                if (alu_op <= 5) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int shift = op & 0xF;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = (alu_op == 2)
                        ? (src_sel ? &s->b : &s->a)
                        : (dst_sel ? &s->b : &s->a);
                    int64_t lk_val;
                    if (alu_op <= 2)
                        lk_val = (int64_t)(int16_t)op2 << shift;
                    else
                        lk_val = (int64_t)(uint16_t)op2 << shift;
                    switch (alu_op) {
                    case 0: *dst = sext40(src_val + lk_val); break; /* ADD */
                    case 1: *dst = sext40(src_val - lk_val); break; /* SUB */
                    case 2: *dst = sext40(lk_val); break;           /* LD  */
                    case 3: *dst = src_val & lk_val; break;         /* AND */
                    case 4: *dst = src_val | lk_val; break;         /* OR  */
                    case 5: *dst = src_val ^ lk_val; break;         /* XOR */
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op == 6) {
                    /* F06x: ADD/SUB/LD/AND/OR/XOR #lk,16 + MPY/MAC #lk */
                    uint8_t sub6 = op & 0xF;
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    switch (sub6) {
                    case 0: *dst = sext40(src_val + ((int64_t)(int16_t)op2 << 16)); break;
                    case 1: *dst = sext40(src_val - ((int64_t)(int16_t)op2 << 16)); break;
                    case 2: dst = src_sel ? &s->b : &s->a;
                            *dst = sext40((int64_t)(int16_t)op2 << 16); break;
                    case 3: *dst = src_val & ((int64_t)(uint16_t)op2 << 16); break;
                    case 4: *dst = src_val | ((int64_t)(uint16_t)op2 << 16); break;
                    case 5: *dst = src_val ^ ((int64_t)(uint16_t)op2 << 16); break;
                    case 6: /* MPY #lk, dst */
                            dst = src_sel ? &s->b : &s->a;
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(p); } break;
                    case 7: /* MAC #lk, src[,dst] */
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(src_val + p); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op >= 8) {
                    /* F08x-F0Fx: accumulator-to-accumulator ops (1-word).
                     * bits 7:5 = op (100=AND,101=OR,110=XOR,111=SFTL)
                     * bits 4:0 = shift (signed 5-bit), bits 9:8 = src,dst */
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    switch (aop) {
                    case 4: *dst = sext40(sv) & sext40(shifted); break;
                    case 5: *dst = sext40(sv) | sext40(shifted); break;
                    case 6: *dst = sext40(sv) ^ sext40(shifted); break;
                    case 7: { uint64_t uv = (uint64_t)(sv & 0xFFFFFFFFFFULL);
                              if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                              *dst = sext40(uv & 0xFFFFFFFFFFULL); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words).
             * Delayed: 2 delay slots after the 2-word instruction.
             * RSA = PC + 4 (skip RPTBD + 2 delay slot words). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            { static int _rb=0; if (_rb<20) { C54_LOG("RPTBD PC=0x%04x REA=0x%04x RSA=0x%04x BRC=%d", s->pc, s->rea, s->rsa, s->brc); _rb++; } }
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words, 2 delay slots).
             * Push PC+4 (past CALLD + 2 delay slots), branch to pmad. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
            s->pc = op2;
            return 0;
        }
        if (op == 0xF273) {
            /* RETD — delayed return (1 word) */
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->pc = ra;
            return 0;
        }
        /* LMS Xmem, Ymem — Least Mean Square step (1-word dual-operand)
         * Encoding: 1111 001D XXXX YYYY
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem
         * Exclude F272 (RPTBD), F273 (RETD), F274 (CALLD) — exact-match
         * opcodes that share the F2xx range but are handled below. */
        /* REMOVED 2026-05-08 night : the previous "LMS Xmem,Ymem" handler
         * for hi8 ∈ {0xF2, 0xF3} (excluding F272/F273/F274) was mis-decoded
         * — it claimed encoding `1111 001D XXXX YYYY` but per binutils
         * tic54x-opc.c LMS is actually :
         *
         *   { "lms", 1,2,2, 0xE100, 0xFF00, {OP_Xmem,OP_Ymem}, ... }
         *
         * i.e. hi8 == 0xE1, NOT 0xF2/F3. The 0xE1 handler already exists
         * (line ~3247) and is correct.
         *
         * The F2xx/F3xx range per binutils contains only :
         *   F272 RPTBD, F273 RETD, F274 CALLD                (3 special-cases)
         *   F300-F31F INTR k                                 (handled below)
         *   F330-F35F AND/OR/XOR with shift  (mask FCF0)     (handled below)
         *   F360-F367 ADD/SUB/AND/OR/XOR/MAC #lk (mask FCFF) (handled below)
         *   F380-F3FF AND/OR/XOR/SFTL src,SHIFT,DST (FCE0)   (handled below)
         *   F320-F32F + F368-F37F unmapped (NOP fallback)
         *
         * The bogus LMS catch-all stole every F3xx instruction before the
         * proper F3 dispatch could see it. For 0xF3E1 (= SFTL B,1,B,
         * 4872 sites in firmware) it computed `new_ym = AH*T-derived junk`
         * and called data_write(s, AR1, new_ym). When AR1=0, that wrote
         * the junk to MMR_IMR. This is the IMR-thrash cascade observed
         * post-0x76-fix at PC=0x8eb9.
         *
         * Discovered after the 0x76 fix exposed the second-level cascade.
         * Trace evidence : IMR-W 0x0000→{0x0540, 0x0525, 0x082b, 0xfd57,
         * 0xfacf, ...} all PC=0x8eb9 op=0xf3e1, INTM-TRANS XPC=0
         * (confirms genuine PROM0 execution, not XPC artifact).
         *
         * Fix : let the existing F3 dispatch (line 2468+) handle F3xx
         * properly. F2xx (other than F272/3/4) falls through to F-class
         * NOP fallback — firmware does not appear to use it. */
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            /* F820 (624 sites) and F830 (543 sites) are BC pmad,cond per
             * tic54x-opc.c (bc = 0xF800 mask 0xFF00). The dispatcher at
             * PROM0 0xb968-0xb9a4 relies on these branching when the ACC
             * comparison succeeds. Cond 0x20 = C set, cond 0x30 = ?
             * (we treat both via ACC compare for now since dispatcher uses
             * cmp-style behaviour). The full F8xx range is BC per binutils
             * but historically the firmware tolerates the legacy decode
             * for the other sub-codes — surgical override here only.
             *
             * REVERTED 2026-05-15 nuit : tentative de fix vers SPRU172C-strict
             * cond eval (cond=0x20=NTC, cond=0x30=TC) a cassé le firmware DSP
             * Calypso (DSP stuck à PC=0xcc51 / 0xfa95 selon régime, task=24
             * tombait à 0). Le binaire DSP semble utiliser une convention
             * dialectale où F82x/F83x s'attend au comportement ACC-based.
             * Hypothèse alternative : BITF (0x61) émulé incorrectement, TC
             * jamais set correctement → cond NTC/TC ne donne pas le bon
             * résultat. Investiguer BITF avant de retenter le fix BC strict. */
            if (sub == 0x2 || sub == 0x3) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int64_t acc_signed = (s->a & 0x8000000000LL)
                                     ? (s->a | ~0xFFFFFFFFFFLL) : s->a;
                bool take = false;
                /* For now: cond=0x20 → branch if A != 0; cond=0x30 → A == 0.
                 * These are heuristics until we confirm the exact cond
                 * mapping from SPRU172C. Tweak based on observed dispatcher
                 * behaviour. */
                if (sub == 0x2)      take = (acc_signed != 0);
                else /* sub==0x3 */  take = (acc_signed == 0);
                if (take) { s->pc = op2; return 0; }
                return consumed + s->lk_used;
            }
            /* Per tic54x-opc.c:
             *   F880-F8FF mask FF80 = FB pmad (FAR branch unconditional)
             * The low 7 bits of the opcode word encode the target XPC bits.
             * Calypso uses 2-bit XPC, so & 0x3 is sufficient.
             *
             * Earlier this range was treated as plain B pmad — a bug that
             * kept XPC=0 forever (DSP never reached PROM1 user code). */
            if ((op & 0xFF80) == 0xF880) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fb_total;
                fb_total++;
                if (fb_total <= 30 || (fb_total % 5000) == 0) {
                    C54_LOG("FB FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u)",
                            (unsigned long long)fb_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* F88x..F8Bx (mask FF80=0): historic plain B pmad (NEAR), kept
             * for sub-codes that fall outside the FAR mask above. */
            if (sub >= 0x8 && sub <= 0xB) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F86x/F87x: BANZ *ARn, pmad — branch if ARn != 0 (2 words) */
            if (sub == 0x6 || sub == 0x7) {
                op2 = prog_fetch(s, s->pc + 1);
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;  /* skip 2 words, fall through */
            }
            /* F84x/F85x: BANZ with condition / CALL variants */
            if (sub == 0x4 || sub == 0x5) {
                op2 = prog_fetch(s, s->pc + 1);
                /* BANZ ARn, pmad */
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;
            }
            /* F8Cx-F8Fx: CALL/CALLD pmad (2 words) */
            if (sub >= 0xC) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            /* F80x-F81x: BANZ pmad, Smem (2 words)
             * Per SPRU172C + tic54x-opc.c: entire F8xx range is BANZ.
             * Sind operand selects AR via op[2:0] (nar). Test pre-mod
             * value; resolve_smem applies Sind post-mod. Same off-by-ARP
             * fix as 0x6C00 / 0x6E00 BANZ/BANZD. */
            if (sub <= 0x1) {
                int nar = op & 0x07;
                uint16_t old_ar = s->ar[nar];
                addr = resolve_smem(s, op, &ind);
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                if (old_ar != 0) {
                    s->pc = op2;
                    return 0;
                }
                return consumed + s->lk_used;
            }
            /* Fallback: RPT Smem (F8xx sub not handled above) */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_active = true;
            s->pc += consumed;
            return 0;
        }
        /* F3xx: dispatch per binutils tic54x-opc.c (verified against
         * insn_template struct include/opcode/tic54x.h:85-150).
         *
         * 8 sub-families:
         *   F300-F31F  INTR k                                 1-word
         *   F320-F32F  unmapped                               (NOP fallback)
         *   F330-F35F  AND/OR/XOR #lk,SHIFT,SRC,DST  mask FCF0 2-word
         *   F360-F367  ADD/SUB/AND/OR/XOR/MAC #lk var. FCFF   2-word
         *   F368-F37F  unmapped                               (NOP fallback)
         *   F380-F39F  AND  src,SHIFT,DST            mask FCE0 1-word
         *   F3A0-F3BF  OR   src,SHIFT,DST            mask FCE0 1-word
         *   F3C0-F3DF  XOR  src,SHIFT,DST            mask FCE0 1-word
         *   F3E0-F3FF  SFTL src,SHIFT,DST            mask FCE0 1-word
         *
         * Dispatch order: most-specific masks first (FCFF → FCF0 → FCE0).
         *
         * 2026-04-29 — replaces previous "F320+ → LD #k9, DP" fallback
         * which mass-mis-decoded 364 firmware sites. Wedge at PC=0x8eb9
         * (0xF3E1 SFTL B,1,B) was directly tied to this bug.
         * See doc/opcodes/0xF3.md for full spec. */
        if (hi8 == 0xF3) {
            /* F300-F31F: INTR k (preserve existing behavior) */
            if ((op & 0xFFE0) == 0xF300) {
                int vec = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                return 0;
            }

            /* F360-F367: 2-word with mask FCFF (#lk<<16 variants).
             * Most-specific mask, check first. */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD #lk<<16, src, [dst] */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC #lk, src, [dst] */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: { /* MAC: dst = src + T * lk */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod;
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F330-F35F: 2-word with mask FCF0 (#lk + 4-bit shift).
             * AND (sub=3), OR (sub=4), XOR (sub=5).
             * Note: ADD (sub=0) and SUB (sub=1) at F30x/F31x are caught
             * by INTR handler above (those ranges are INTR semantically). */
            if ((op & 0xFCF0) == 0xF030 ||  /* AND #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF040 ||  /* OR */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t lk_signed = (int16_t)op2;
                int64_t shifted = (shift >= 0) ? (lk_signed << shift)
                                               : (lk_signed >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x3: result = src & shifted; break;  /* AND */
                case 0x4: result = src | shifted; break;  /* OR  */
                case 0x5: result = src ^ shifted; break;  /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F380-F3FF: 1-word AND/OR/XOR/SFTL src,SHIFT,DST (mask FCE0).
             * Sub-opcode in bits 7-5: 100=AND, 101=OR, 110=XOR, 111=SFTL. */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { /* AND src,SHIFT,DST: DST = SRC & (DST_in << shift) */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src & sh;
                    break;
                }
                case 0x5: { /* OR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src | sh;
                    break;
                }
                case 0x6: { /* XOR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src ^ sh;
                    break;
                }
                case 0x7: { /* SFTL src,SHIFT,DST: DST = SRC << shift (logical) */
                    uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                    result = (int64_t)((shift >= 0) ? (usrc << shift) : (usrc >> (-shift)));
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F320-F32F + F368-F37F: unmapped per binutils. NOP fallback +
             * log-once for diagnostic. 9 firmware sites total. */
            {
                static int unmapped_log = 0;
                if (unmapped_log++ < 20)
                    C54_LOG("F3xx unmapped op=0x%04x PC=0x%04x (NOP)",
                            op, s->pc);
            }
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0xB) {
                /* F6Bx: RSBX -- reset bit in ST1 (bit 9=1, bit 8=0).
                 * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0 covers F6Bx. */
                int bit = op & 0x0F;
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Delayed branches/calls/returns from PROM (per tic54x-opc.c).
             * MUST be checked BEFORE the MVDD catch-all because they share
             * the high nibbles 0xE/0x9. Without these the DSP cannot return
             * from interrupt service routines — RETED in particular leaves
             * INTM=1 forever, blocking every subsequent INT3 and stalling
             * the firmware↔DSP frame loop (the original CLAUDE.md root bug).
             *
             * All delayed forms execute 2 delay-slot words before the jump
             * commits; we arm the existing delayed_pc/delay_slots machinery
             * (the same one RCD uses) so the slots run with the right PC. */
            if (op == 0xF6EB) {
                /* RETED — return from interrupt, enable interrupts, delayed.
                 * Pop PC, clear INTM, then run 2 delay slots before jumping. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static uint64_t reted_count;
                    reted_count++;
                    if (reted_count <= 20 || (reted_count % 100) == 0)
                        C54_LOG("RETED #%llu PC=0x%04x -> ra=0x%04x SP=0x%04x INTM=0",
                                (unsigned long long)reted_count,
                                s->pc, ra, s->sp);
                }
                return consumed + s->lk_used;
            }
            if (op == 0xF69B) {
                /* RETFD — fast return, delayed (no INTM change). */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E2 || op == 0xF6E3) {
                /* BACCD A / CALAD A — delayed branch/call to acc(low).
                 * 1-word op + 2 delay slots. CALAD pushes PC+3 (skip op +
                 * 2 delay slots) per TI convention (cf. CALLD which pushes
                 * PC+4 for its 2-word form). Branch is armed via the
                 * delayed_pc/delay_slots mechanism so the 2 slots run
                 * before PC commits to tgt. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                bool is_call = (op == 0xF6E3);
                static uint64_t bcd_total;
                bcd_total++;
                /* Pre-load context: dump the 8 words preceding PC (in OVLY
                 * the executor reads from DARAM, mirror that). Lets us see
                 * which LD/MAR sequence was supposed to put a valid target
                 * in A before the CALAD/BACCD. */
                int pre_ovly = (s->pmst & PMST_OVLY) && s->pc >= 0x80 && s->pc < 0x2800;
                uint16_t pre[8];
                for (int i = 0; i < 8; i++) {
                    uint16_t a = (uint16_t)(s->pc - 8 + i);
                    pre[i] = pre_ovly ? s->data[a] : s->prog[a];
                }
                if (bcd_total <= 60 || (bcd_total % 5000) == 0) {
                    C54_LOG("BCD/CAD F6E%c #%llu PC=0x%04x tgt=0x%04x A=%010llx SP=0x%04x DP=0x%03x mem[%c PC-8..-1]=%04x %04x %04x %04x %04x %04x %04x %04x%s",
                            is_call ? '3' : '2',
                            (unsigned long long)bcd_total,
                            s->pc, tgt,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            (s->st0 & 0x1FF),
                            pre_ovly ? 'D' : 'P',
                            pre[0], pre[1], pre[2], pre[3],
                            pre[4], pre[5], pre[6], pre[7],
                            is_call ? " CALAD" : " BACCD");
                }
                if (is_call) {
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E4 || op == 0xF6E5) {
                /* FRETD / FRETED — far return, delayed.
                 * Pop XPC + PC unconditionally (FL_FAR). FRETED also clears INTM.
                 * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                if (op == 0xF6E5) s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E6 || op == 0xF6E7) {
                /* FBACCD A / FCALAD A — far delayed branch/call to A.
                 * A(22:16) → XPC, A(15:0) → tgt. XPC update is immediate
                 * (mirrors FRETED at line ~1639). FCALAD pushes ret PC+3,
                 * and (when APTS) pushes XPC first (so RETF/FRETD pops in
                 * order). 2 delay slots. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                uint8_t  new_xpc = (uint8_t)((s->a >> 16) & 0xFF);
                if (new_xpc > 3) new_xpc &= 3;
                bool is_call = (op == 0xF6E7);
                static uint64_t fbcd_total;
                fbcd_total++;
                if (fbcd_total <= 10 || (fbcd_total % 5000) == 0) {
                    C54_LOG("FBCD/FCAD F6E%c #%llu PC=0x%04x tgt=0x%04x newXPC=%u A=%010llx SP=0x%04x%s",
                            is_call ? '7' : '6',
                            (unsigned long long)fbcd_total,
                            s->pc, tgt, new_xpc,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            is_call ? " FCALAD" : " FBACCD");
                }
                if (is_call) {
                    /* FCALAD (F6E7): push XPC + return PC unconditionally (FL_FAR).
                     * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->xpc         = new_xpc;
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (sub >= 0x8) {
                /* F68x-F6Fx: MVDD Xmem, Ymem — dual data-memory operand move
                 * Encoding: 1111 0110 XXXX YYYY
                 *   bit 7   = Xmod (0=inc, 1=dec)
                 *   bits 6:4 = Xar  (source AR register)
                 *   bit 3   = Ymod (0=inc, 1=dec)
                 *   bits 2:0 = Yar  (dest AR register) */
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t val = data_read(s, s->ar[xar]);
                data_write(s, s->ar[yar], val);
                if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
                if ((op >> 3) & 1) s->ar[yar]--; else s->ar[yar]++;
                return consumed + s->lk_used;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed + s->lk_used;
        }
        /* F5xx: SSBX or RPT #k */
        if (hi8 == 0xF5) {
            /* F5Bx: SSBX -- set bit in ST0 (bit 9=0, bit 8=1).
             * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF5B0) {
                int bit = op & 0x0F;
                s->st0 |= (1 << bit);
                return consumed + s->lk_used;
            }
            /* Note: 0xF5E2/F5E3 (BACC B / CALA B) are handled earlier alongside
             * their F4 counterparts, so they never reach this F5xx block. */
            /* RPT #k (short immediate) — kept as fallback, must advance PC. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        /* DIAG: log F7xx executions before the (buggy) LD #k8 dispatch.
         * Per tic54x-opc.c the F7xx range contains SSBX ST1 (0xF7Bx) and
         * other instructions, NOT LD #k8 (which is at E800-E9FF).
         * Caps at 5 per distinct sub-opcode to avoid spam. */
        if (hi8 == 0xF7) {
            static int f7xx_seen[256] = {0};
            int sub_idx = op & 0xFF;
            if (++f7xx_seen[sub_idx] <= 100 || (f7xx_seen[sub_idx] % 1000) == 0) {
                C54_LOG("F7xx EXEC op=0x%04x PC=0x%04x XPC=%d insn=%u",
                        op, s->pc, s->xpc, s->insn_count);
            }
        }
        /* F7Bx: SSBX bit, ST1 (incl. SSBX INTM at F7BB).
         * Per binutils tic54x-opc.c: opcode "ssbx" 0xF5B0 mask 0xFDF0,
         * where bit 9 selects ST0 (0xF5Bx) vs ST1 (0xF7Bx).
         * Symmetric counterpart of RSBX ST1 (F6Bx) handler above.
         * MUST be tested before the F7xx LD #k8 dispatch (which is
         * itself incorrect — per SPRU172C, LD #k8 lives at E800-E9FF). */
        if ((op & 0xFFF0) == 0xF7B0) {
            int bit = op & 0x0F;
            bool is_intm = (bit == 11);
            s->st1 |= (1 << bit);
            if (is_intm)
                C54_LOG("*** SSBX INTM (F7BB) *** PC=0x%04x ST1=0x%04x insn=%u",
                        s->pc, s->st1, s->insn_count);
            return consumed + s->lk_used;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
            uint16_t k = op & 0xFF;
            switch (sub) {
            case 0x0: /* F70x: LD #k8, ASM */
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | (k & ST1_ASM_MASK);
                break;
            case 0x1: /* F71x: LD #k8, AR0 */
                s->ar[0] = k; break;
            case 0x2: /* F72x: LD #k8, AR1 */
                s->ar[1] = k; break;
            case 0x3: s->ar[2] = k; break;
            case 0x4: s->ar[3] = k; break;
            case 0x5: s->ar[4] = k; break;
            case 0x6: s->ar[5] = k; break;
            case 0x7: s->ar[6] = k; break;
            case 0x8: /* F78x: LD #k8, T */
                s->t = (s->st1 & ST1_SXM) ? (uint16_t)(int8_t)k : k; break;
            case 0x9: /* F79x: LD #k8, DP */
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (k & ST0_DP_MASK); break;
            case 0xA: /* F7Ax: LD #k8, ARP */
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((k & 7) << ST0_ARP_SHIFT); break;
            case 0xB: s->ar[7] = k; break; /* F7Bx: LD #k8, AR7 */
            case 0xC: s->bk = k; break;
            case 0xD: s->sp = k; break;
            case 0xE: /* F7Ex: LD #k8, BRC */
                s->brc = k; break;
            case 0xF: /* F7Fx: LD #k8, ... */
                break;
            }
            return consumed + s->lk_used;
        }
        /* F9xx encoding split per tic54x-opc.c:
         *   F900-F97F mask FF00 = CC pmad cond (NEAR conditional call)
         *   F980-F9FF mask FF80 = FCALL pmad   (FAR call unconditional)
         * The bit 7 of the opcode low byte distinguishes them. */
        if (hi8 == 0xF9) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALL FAR : push XPC + return PC unconditionally (FL_FAR).
             * Per binutils tic54x-opc.c (fcall 0xF980 mask 0xFF80, FL_FAR)
             * and SPRU172C: FAR call always saves XPC for FRET to restore.
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics).
             * Old behavior caused 281 firmware FCALL FAR sites to push only PC,
             * imbalanced with 142 FRET pop expecting both PC + XPC. */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcall_total;
                fcall_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                if (fcall_total <= 30 || (fcall_total % 5000) == 0) {
                    C54_LOG("FCALL FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x)",
                            (unsigned long long)fcall_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            uint8_t cond_code = (op >> 4) & 0xF;
            uint8_t qual = op & 0xF;
            bool take = false;
            int64_t acc = (qual & 0x8) ? s->b : s->a;
            switch (cond_code) {
            case 0x0: take = true; break;
            case 0x1: take = (acc < 0); break;
            case 0x2: take = (acc <= 0); break;
            case 0x3: take = (acc != 0); break;
            case 0x4: take = (acc == 0); break;
            case 0x5: take = (acc >= 0); break;
            case 0x6: take = (acc > 0); break;
            case 0x8: take = !!(s->st0 & ST0_TC); break;
            case 0x9: take = !(s->st0 & ST0_TC); break;
            case 0xA: take = !!(s->st0 & ST0_C); break;
            case 0xB: take = !(s->st0 & ST0_C); break;
            default: take = true; break;
            }
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                /* CC leak tracer */
                {
                    static uint32_t cc_targets[64];
                    static uint32_t cc_counts[64];
                    static int cc_n = 0;
                    static uint32_t total_cc = 0;
                    bool found = false;
                    for (int i = 0; i < cc_n; i++) {
                        if (cc_targets[i] == op2) { cc_counts[i]++; found = true; break; }
                    }
                    if (!found && cc_n < 64) { cc_targets[cc_n] = op2; cc_counts[cc_n++] = 1; }
                    if ((++total_cc % 100) == 0) {
                        C54_LOG("F9xx CC TOP TARGETS (SP=0x%04x total=%u):", s->sp, total_cc);
                        for (int i = 0; i < cc_n && i < 10; i++)
                            C54_LOG("  CC→0x%04x count=%u", cc_targets[i], cc_counts[i]);
                    }
                }
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx encoding split per tic54x-opc.c:
         *   FA80-FAFF mask FF80 = FBD pmad (FAR branch delayed)
         *   FA00-FA7F = various NEAR delayed ops (treated as branch). */
        if (hi8 == 0xFA) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            if ((op & 0x80) != 0) {
                /* FBD FAR delayed branch — XPC change, no push */
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fbd_total;
                fbd_total++;
                if (fbd_total <= 30 || (fbd_total % 5000) == 0) {
                    C54_LOG("FBD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u, delayed 2 slots)",
                            (unsigned long long)fbd_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* REVERTED 2026-05-15 nuit : tentative de cond eval pour
             * FA00-FA7F (BCD NEAR delayed branch) cassait le firmware
             * (DSP stuck loops). Le binaire DSP Calypso semble dépendre
             * du comportement "branch always" pour ces opcodes. Cf comment
             * sur F820/F830 BC handler. */
            /* NEAR FAxx fallback: simplified treat as branch */
            s->pc = op2;
            return 0;
        }
        /* FBxx encoding split per tic54x-opc.c:
         *   FB80-FBFF mask FF80 = FCALLD pmad (FAR call delayed)
         *   FB00-FB7F mask FF00 = CCD pmad cond (NEAR conditional call delayed) */
        if (hi8 == 0xFB) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALLD FAR : push XPC + return PC+4 unconditionally (FL_FAR delayed).
             * Per binutils (fcalld 0xFB80 mask 0xFF80, FL_FAR|FL_DELAY).
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcalld_total;
                fcalld_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                if (fcalld_total <= 30 || (fcalld_total % 5000) == 0) {
                    C54_LOG("FCALLD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x, delayed)",
                            (unsigned long long)fcalld_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            uint8_t cond_code = (op >> 4) & 0xF;
            uint8_t qual = op & 0xF;
            bool take = false;
            int64_t acc = (qual & 0x8) ? s->b : s->a;
            switch (cond_code) {
            case 0x0: take = true; break;
            case 0x1: take = (acc < 0); break;
            case 0x2: take = (acc <= 0); break;
            case 0x3: take = (acc != 0); break;
            case 0x4: take = (acc == 0); break;
            case 0x5: take = (acc >= 0); break;
            case 0x6: take = (acc > 0); break;
            case 0x8: take = !!(s->st0 & ST0_TC); break;
            case 0x9: take = !(s->st0 & ST0_TC); break;
            case 0xA: take = !!(s->st0 & ST0_C); break;
            case 0xB: take = !(s->st0 & ST0_C); break;
            default: take = true; break;
            }
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 4)); /* past CCD + 2 delay slots */
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        /* FCxx: RC cond / RET -- return conditional (1-word).
         * Per tic54x-opc.c: RET=0xFC00, RC=0xFC00 mask 0xFF00. */
        if (hi8 == 0xFC) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                {
                    static int rc_log = 0;
                    if (rc_log < 50)
                        C54_LOG("RC/RET PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x",
                                s->pc, cc, ra, s->sp);
                    rc_log++;
                }
                /* POST-BOOTSTUB-RET : si on est en train de RET depuis le
                 * boot stub (PC ∈ 0x0000..0x0008), c'est la sortie du
                 * task-switch trampoline 0x701b/0x701d → 0x0000. Le ra
                 * poppé est le PC du task qui prend le contrôle. À insn≈90.2M
                 * (dernière transition INTM), ce PC = le task qui ne clear
                 * jamais INTM ensuite. */
                if (s->pc <= 0x0008) {
                    static unsigned bsr;
                    bsr++;
                    if (bsr <= 200 || (bsr % 50) == 0) {
                        fprintf(stderr,
                                "[c54x] POST-BOOTSTUB-RET #%u PC=0x%04x -> task=0x%04x "
                                "SP_new=0x%04x B=0x%010llx INTM=%d insn=%u\n",
                                bsr, s->pc, ra, s->sp,
                                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                                !!(s->st1 & ST1_INTM), s->insn_count);
                    }
                }
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: RCD cond / RETD -- return conditional delayed (1-word).
         * Per tic54x-opc.c: RETD=0xFE00, RCD=0xFE00 mask 0xFF00.
         * Simplified: immediate return (delay slots skipped). */
        if (hi8 == 0xFE) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                /* RCD is *delayed*: per SPRU172C the next 2 instructions
                 * after RCD execute before the return takes effect. The
                 * old "skip delay slots" implementation broke FB-detection
                 * because slots like `LD #0, B` at PROM0 0x75ea were never
                 * run, leaving accumulator state stale and the dispatcher
                 * at 0x7700 looping forever.
                 *
                 * Fix: arm the existing delayed_pc/delay_slots machinery —
                 * pop the return address now, advance PC normally so the
                 * next 2 instructions execute as delay slots, then the
                 * main loop forces PC = delayed_pc. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static int rcd_log = 0;
                    if (rcd_log < 50)
                        C54_LOG("RCD/RETD PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x (delayed)",
                                s->pc, cc, ra, s->sp);
                    rcd_log++;
                }
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FFxx is XC 2,cond — handled above with FDxx. No ADD here. */
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        /* CMPS src, Smem — Compare, Select, and Store (Viterbi)
         * Encoding: 1110 00SD IAAAAAAA (1 word)
         * Per SPRU172C p.4-35: if |A(32-16)| >= |Smem| then TC=1,
         * TRN = (TRN<<1)|1, dst=A; else TC=0, TRN=(TRN<<1), dst=Smem<<16 */
        if ((op & 0xFC00) == 0xE000) {
            int src_s = (op >> 9) & 1;
            int dst_d = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t acc = src_s ? s->b : s->a;
            int32_t ah = (int32_t)((acc >> 16) & 0xFFFF);
            if (ah < 0) ah = -ah;
            int32_t sv = (int16_t)val;
            if (sv < 0) sv = -sv;
            s->trn <<= 1;
            if (ah >= sv) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                int64_t nv = (int64_t)(int16_t)val << 16;
                if (dst_d) s->b = sext40(nv); else s->a = sext40(nv);
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFE00) == 0xEA00) {
            /* EAxx: LD #k9, DP — Load Data Page pointer (1-word).
             * Per tic54x-opc.c: ld 0xEA00 mask 0xFE00, 1 word. */
            uint16_t k9 = op & 0x01FF;
            uint16_t old_dp = s->st0 & ST0_DP_MASK;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            {
                static uint64_t dpc;
                dpc++;
                if (dpc <= 80 || (dpc % 5000) == 0 || k9 == 0x83) {
                    C54_LOG("DP-SET EAxx #%llu PC=0x%04x DP 0x%03x → 0x%03x %s",
                            (unsigned long long)dpc, s->pc,
                            old_dp, k9,
                            k9 == 0x83 ? "*** 0x83 (CALAD-zone base 0x4180) ***" : "");
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* ECxx: RPT #k8u — repeat next instruction k8u+1 times.
             * Per tic54x-opc.c: rpt 0xEC00 mask 0xFF00, single word.
             * Must advance PC past RPT now and return 0 so the dispatcher
             * re-executes the NEXT instruction (not RPT itself). */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVDD Xmem, Ymem  (per tic54x-opc.c, NOT MVMM)
             * 1-word, 2-cycle dual-operand data-to-data move:
             *   *Ymem = *Xmem
             * Per tic54x.h:
             *   XMEM = (op & 0xF0) >> 4
             *   YMEM = op & 0x0F
             *   XMOD/YMOD = (nibble & 0xC) >> 2  (0=*AR,1=*AR-,2=*AR+,3=*AR+0%)
             *   XARX/YARX = (nibble & 0x3) + 2   (AR2..AR5 only) */
            uint8_t xnib = (op >> 4) & 0xF;
            uint8_t ynib = op & 0xF;
            int xar = (xnib & 0x3) + 2;
            int yar = (ynib & 0x3) + 2;
            int xmod = (xnib & 0xC) >> 2;
            int ymod = (ynib & 0xC) >> 2;
            uint16_t xa = s->ar[xar];
            uint16_t ya = s->ar[yar];
            uint16_t v = data_read(s, xa);
            data_write(s, ya, v);
            /* Post-modify both ARs per their mod field */
            switch (xmod) {
                case 0: break;                        /* *AR     */
                case 1: s->ar[xar] = xa - 1; break;   /* *AR-    */
                case 2: s->ar[xar] = xa + 1; break;   /* *AR+    */
                case 3: s->ar[xar] = xa + s->ar[0]; break; /* *AR+0% (no circular here) */
            }
            switch (ymod) {
                case 0: break;
                case 1: s->ar[yar] = ya - 1; break;
                case 2: s->ar[yar] = ya + 1; break;
                case 3: s->ar[yar] = ya + s->ar[0]; break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE7) {
            /* E7xx: MVMM mmrx, mmry  (per tic54x-opc.c)
             * 1-word, 2-cycle, MMR-to-MMR move using a constrained set
             * (MMRX/MMRY operand types). */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst <= 7) s->ar[dst] = val;
            else if (dst == 8) s->sp = val;
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8 || hi8 == 0xE9) {
            /* E8xx/E9xx: LD #k8u, dst — Load 8-bit unsigned immediate (1-word).
             * Per tic54x-opc.c: ld 0xE800 mask 0xFE00.
             * bit 8 = dst (0=A, 1=B), bits 7:0 = k8u.
             * NOTE: This was previously decoded as CC (conditional call, 2-word)
             * which caused stack overflow by pushing return addresses in a loop. */
            int dst = (op >> 8) & 1;
            uint8_t k = op & 0xFF;
            int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int8_t)k : (int64_t)k;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE1) {
            /* E1xx: single-word acc ops — NEG, ABS, CMPL, SAT, EXP, etc. */
            uint8_t sub = op & 0xFF;
            switch (sub) {
            case 0xE0: s->a = ~s->a; s->a = sext40(s->a); break;  /* CMPL A */
            case 0xE1: s->b = ~s->b; s->b = sext40(s->b); break;  /* CMPL B */
            case 0xE2: s->a = -s->a; s->a = sext40(s->a); break;  /* NEG A */
            case 0xE3: s->b = -s->b; s->b = sext40(s->b); break;  /* NEG B */
            case 0xE4: /* SAT A */ if (s->st0 & ST0_OVA) s->a = (s->a < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE5: /* SAT B */ if (s->st0 & ST0_OVB) s->b = (s->b < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE8: /* ABS A */ s->a = (s->a < 0) ? -s->a : s->a; s->a = sext40(s->a); break;
            case 0xE9: /* ABS B */ s->b = (s->b < 0) ? -s->b : s->b; s->b = sext40(s->b); break;
            case 0xEA: /* ROR A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a >> 1) | ((int64_t)c << 39); s->a = sext40(s->a); } break;
            case 0xEB: /* ROL A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & ((int64_t)1<<39)) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a << 1) | c; s->a = sext40(s->a); } break;
            default:
                /* EXP A/B etc — return 0 for now */
                break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEF) {
            /* EFxx: RPTZ dst, #lk — Zero accumulator and repeat (2 words)
             * Per SPRU172C: dst = 0; RPT #lk
             * Encoding: 1110 1111 xxxx xxxx + lk_word */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int rptz_dst = (op >> 0) & 1;
            if (rptz_dst) s->b = 0; else s->a = 0;
            s->rpt_count = op2;
            s->rpt_active = true;
            s->pc += 2;
            return 0;
        }
        if (hi8 == 0xEB) {
            /* EBxx: RPTB[D] pmad — Block repeat (2 words)
             * Per SPRU172C: REA = pmad, RSA = PC+2, BRAF=1 */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE6) {
            /* E6xx: SFTA/SFTL acc, #shift (single-word immediate shift) */
            int shift = op & 0x1F;
            if (shift & 0x10) shift |= ~0x1F;  /* sign extend 5-bit */
            int dst = (op >> 5) & 1;
            int logical = (op >> 6) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            if (logical) {
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else {
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEE) {
            /* EExx: BCD pmad, cond (conditional delayed branch, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x00) take = true;  /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else if (cond == 0x20) take = (s->st0 & ST0_C) != 0;
            else if (cond == 0x21) take = !(s->st0 & ST0_C);
            else if ((cond & 0x3A) == 0x3A) take = true; /* unconditional-ish */
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if ((op & 0xFFE0) == 0xED00) {
            /* ED00-ED1F: LD #k5, ASM — load 5-bit immediate into ASM field of ST1.
             * Per tic54x-opc.c: ld 0xED00 mask 0xFFE0, 1 word.
             * NOT BCD (which is 0xFA00 mask 0xFF00). */
            uint8_t k5 = op & 0x1F;
            s->st1 = (s->st1 & ~ST1_ASM_MASK) | k5;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx (not ED00-ED1F): BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = true;            /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* 7Exx: READA Smem — read prog[A_low] → data[Smem]
         * Per tic54x-opc.c: reada 0x7E00 mask 0xFF00 (1 word).
         * Under RPT, the prog address auto-increments each iteration;
         * accumulator A is preserved (we mirror via mvpd_src state). */
        if (hi8 == 0x7E) {
            addr = resolve_smem(s, op, &ind);
            uint16_t psrc = s->rpt_active ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            uint16_t v = prog_read(s, psrc);
            data_write(s, addr, v);
            s->mvpd_src = psrc + 1;
            { static int reada_log = 0; if (reada_log++ < 20)
                C54_LOG("READA: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x rpt=%d insn=%u",
                        psrc, v, addr, s->pc, s->rpt_count, s->insn_count); }
            return consumed + s->lk_used;
        }
        /* 7Fxx: WRITA Smem — write data[Smem] → prog[A_low] (mirror of READA) */
        if (hi8 == 0x7F) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pdst = s->rpt_active ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            prog_write(s, pdst, data_read(s, addr));
            s->mvpd_src = pdst + 1;
            return consumed + s->lk_used;
        }
        /* 6Dxx: MAR Smem — modify address register (side effects only) */
        if (hi8 == 0x6D) {
            addr = resolve_smem(s, op, &ind);
            /* MAR only modifies AR via addressing mode, no data access */
            return consumed + s->lk_used;
        }
        /* 76xx: ST #lk, Smem  (2 or 3 words) — store 16-bit literal to data
         * memory. Per binutils tic54x-opc.c {st, 2,2,2, 0x7600, 0xFF00,
         * {OP_lk, OP_Smem}} and tic54x-dis.c get_insn_size = words +
         * has_lkaddr (extra word when Smem mode in 0xC..0xF).
         *
         * Encoding (verified via tic54x-dis.c:192-204):
         *   word 0 = opcode (0x76xx)
         *   word 1 = lkaddr  (Smem extension, only if mode in 0xC..0xF)
         *   word N = opcode2 (the #lk value being stored, last extension)
         *
         * Was previously misdecoded as LDM MMR,dst (1 word) — copy/paste
         * of the wrong mnemonic. The real LDM is 0x48xx mask 0xFE00,
         * already correctly handled in the 0x4 group. Misdecoding caused
         * PC to advance by 1 instead of 2-3 ; the literal then executed
         * as a stray opcode. In particular the 0x4F00 (DST B,Lmem with
         * DP=0 → MMR_IMR) stray write zeroed IMR forever, masking
         * INT3+BRINT0 → DSP parked in RPTB at e9ab..e9b6 awaiting a
         * frame interrupt that was never serviced. Fix 2026-05-08. */
        if (hi8 == 0x76) {
            static unsigned hit76_log;
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (hit76_log++ < 30) {
                fprintf(stderr,
                        "[c54x] HIT-76 PC=0x%04x op=0x%04x addr=0x%04x "
                        "lk=0x%04x lk_used=%d insn=%u\n",
                        s->pc, op, addr, op2, s->lk_used, s->insn_count);
            }
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        /* 77xx: STM #lk, MMR (2 words) */
        if (hi8 == 0x77) {
            uint8_t mmr = op & 0x7F;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* WATCH-ST1-WRITE : MMR 0x07 = ST1. Capture toutes les
             * écritures de ST1 (STM #lk, ST1) — incluant celles qui
             * ne changent pas la valeur d'INTM mais redéfinissent
             * tout le mot ST1. Sortie : valeur écrite, bit 11 (INTM),
             * delta vs current ST1. Cap 200 entries pour boot, puis
             * sample 1/100. */
            if (mmr == 0x07) {
                static unsigned st1w;
                st1w++;
                if (st1w <= 200 || (st1w % 100) == 0) {
                    int new_intm = !!(op2 & (1 << 11));
                    int cur_intm = !!(s->st1 & ST1_INTM);
                    fprintf(stderr,
                            "[c54x] ST1-WR #%u STM #0x%04x,ST1 PC=0x%04x "
                            "cur=0x%04x->0x%04x INTM:%d->%d insn=%u XPC=%d\n",
                            st1w, op2, s->pc, s->st1, op2,
                            cur_intm, new_intm, s->insn_count, s->xpc);
                }
            }
            data_write(s, mmr, op2);
            return consumed + s->lk_used;
        }
        /* REVERTED 2026-05-15 nuit : handlers 0x72/0x73 (MVDM/MVMD per
         * binutils tic54x-opc.c) RETIRÉS d'ici. Le fix sémantiquement
         * correct révèle un bug compensateur upstream du firmware qu'on n'a
         * pas le temps d'attaquer maintenant. Régressions empiriques :
         *   - A_CD-WR : 244 → 6 (-97%)
         *   - D_TASK_D val=1 : 2/run → 0
         *   - INT3 fire : 1583/min → 1 total
         *   - DSP busy state : compute path → reset vector loop
         * Voir doc/REVERT_MVMD_KNOWLEDGE.md pour analyse complète,
         * criteria de re-application, et plan d'attaque future.
         *
         * Le fallthrough vers `(op & 0xF800) == 0x7000` (= STL src,Smem)
         * est restoré, qui catch 0x72xx/0x73xx avec ce mask 0xF800.
         * Notamment site critique PC=0x8208 op=0x7317 redevient mis-décodé
         * comme STL B,*(DP+0x17) avec son side-effect compensatoire que
         * le firmware exploite. */
        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Fxx: STH src, Smem
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x6000-0x60FF: CMPM Smem, lk  (compare memory with long immediate)
         * Per tic54x-opc.c: { "cmpm", 2,2,2, 0x6000, 0xFF00 }
         * Sets TC = (data[Smem] == lk).
         *
         * The DSP bootloader at PROM0 0xb41c / 0xb424 polls
         *   CMPM *(0x0fff), 4   →  CMPM *(0x0fff), 2
         * to wait for ARM-side BL_CMD_STATUS write. Without TC being set
         * the subsequent BC NTC always branches back, looping forever.
         * Was previously folded into the generic 0x6000-0x67FF "LD" path
         * which set the accumulator instead and never updated TC. */
        if ((op & 0xFF00) == 0x6000) {
            addr = resolve_smem(s, op, &ind);
            uint16_t cmp_val = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            if (mem_val == cmp_val) s->st0 |= ST0_TC;
            else                    s->st0 &= ~ST0_TC;
            consumed = 2;  /* opcode + cmp_val (smem extra lk added via lk_used) */
            return consumed + s->lk_used;
        }
        /* 0x6100-0x61FF: BITF Smem, lk — bit-field test, TC = (Smem & lk)!=0 */
        if ((op & 0xFF00) == 0x6100) {
            addr = resolve_smem(s, op, &ind);
            uint16_t mask = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            bool tc_before = (s->st0 & ST0_TC) != 0;
            if (mem_val & mask) s->st0 |= ST0_TC;
            else                s->st0 &= ~ST0_TC;
            bool tc_after = (s->st0 & ST0_TC) != 0;
            consumed = 2;
            /* BITF instrumentation (2026-05-15 nuit) — pour confirmer si TC
             * est set correctement. Hypothèse : si BITF appelle souvent mais
             * tc_after=1 rarement → masque/mem_val pattern empêche TC=1,
             * ce qui fait que BC NTC branche toujours et `ST #1, d_task_d`
             * à PROM 0x9ab1 n'est jamais atteint. Format :
             *   BITF-PROBE #N PC=0xXXXX addr=0xXXXX mem=0xXXXX mask=0xXXXX
             *               tc_before=N tc_after=N
             * Cap 200 + 1/1000 ensuite. */
            {
                static uint64_t bitf_total;
                static uint64_t bitf_tc_set;
                static uint64_t bitf_tc_clear;
                bitf_total++;
                if (tc_after) bitf_tc_set++;
                else          bitf_tc_clear++;
                if (bitf_total <= 200 || (bitf_total % 1000) == 0) {
                    fprintf(stderr,
                            "[c54x] BITF-PROBE #%llu PC=0x%04x addr=0x%04x "
                            "mem=0x%04x mask=0x%04x tc_before=%d tc_after=%d "
                            "(total=%llu set=%llu clear=%llu)\n",
                            (unsigned long long)bitf_total, s->last_exec_pc,
                            addr, mem_val, mask, tc_before, tc_after,
                            (unsigned long long)bitf_total,
                            (unsigned long long)bitf_tc_set,
                            (unsigned long long)bitf_tc_clear);
                }
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx-67xx: LD Smem, dst (other variants — fallback) */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* 0x6800-0x6BFF + 0x6Cxx + 0x6Exx: companion to the 0x6F00 fix below.
         * Per binutils tic54x-opc.c (verified against insn_template struct):
         *   0x6800 ANDM  #lk, Smem      data[Smem] = data[Smem] & lk     (2-word)
         *   0x6900 ORM   #lk, Smem      data[Smem] = data[Smem] | lk     (2-word)
         *   0x6A00 XORM  #lku, Smem     data[Smem] = data[Smem] ^ lku    (2-word)
         *   0x6B00 ADDM  #lk, Smem      data[Smem] = data[Smem] + lk     (2-word)
         *   0x6C00 BANZ  pmad, Sind     if (ARx != 0) PC = pmad          (2-word)
         *   0x6E00 BANZD pmad, Sind     same as BANZ but with 2 delay slots
         *
         * Without these, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes them all as LD Smem,T (1-word), causing PC drift +1
         * word and the lk/pmad operand executing as parasitic instruction.
         * 1259 (ANDM/ORM/XORM/ADDM) + 304 (BANZ/BANZD) = 1563 sites in ROM.
         *
         * 2026-04-28 — companion fix to 0x6F00 already inserted below.
         * See doc/opcodes/0x68_0x6F.md for spec. */
        if ((op & 0xFF00) == 0x6800) {
            /* ANDM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v & lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6900) {
            /* ORM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v | lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6A00) {
            /* XORM #lku, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lku = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v ^ lku);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6B00) {
            /* ADDM #lk, Smem — add signed lk to memory (wrap mod 2^16) */
            addr = resolve_smem(s, op, &ind);
            int16_t lk = (int16_t)prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, (uint16_t)((int16_t)v + lk));
            consumed = 2;
            /* TODO: TC/OVM/SXM flag effects per SPRU172C (verify) */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6C00) {
            /* BANZ pmad, Sind — branch if ARx (selected by ARF in op[2:0])
             * is non-zero. Test on PRE-modify value; resolve_smem applies
             * post-mod regardless of branch outcome. Previously read ARP
             * from ST0 (the PREVIOUS instruction's nar) — wrong AR was
             * tested. Cf resolve_smem comment for the off-by-ARP bug. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->pc = pmad;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6E00) {
            /* BANZD pmad, Sind — delayed BANZ (2 slots after the 2-word op).
             * Same off-by-ARP fix as BANZ above. */
            int nar = op & 0x07;
            uint16_t pre = s->ar[nar];
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (pre != 0) {
                s->delayed_pc  = pmad;
                s->delay_slots = 2;
            }
            return consumed + s->lk_used;
        }
        /* 0x6F00-0x6FFF: Extended ADD/SUB/LD/STH/STL Smem, SHIFT, DST/SRC (2-word).
         * Per binutils tic54x-opc.c (verified against insn_template struct
         * include/opcode/tic54x.h:85-150):
         *   word0 = 0x6F00 mask 0xFF00 (Smem in low 7 bits)
         *   word1 = sub-opcode in bits 7:5, SRC=bit 9, DST/SRC1=bit 8,
         *           SHIFT=signed 5-bit in bits 4:0
         *     bits 7:5 = 000 → ADD Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 001 → SUB Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 010 → LD  Smem,SHIFT,DST
         *     bits 7:5 = 011 → STH SRC1,SHIFT,Smem
         *     bits 7:5 = 100 → STL SRC1,SHIFT,Smem
         *
         * Without this handler, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes 0x6Fxx as LD Smem,T (1-word), causing PC drift +1 word
         * and the lk-side operand to be executed as parasitic instruction.
         * 544 sites in firmware ROM. See doc/opcodes/0x68_0x6F.md for spec.
         *
         * 2026-04-28 — fix introduced for wedge at PC=0x8353 (CALAD A self-loop)
         * caused by 0x6F07 0x0C41 mis-decoded → 0x0C41 executed as parasitic
         * SUB Smem,TS,A → A_low=0xFFFA → A_low=0x8353 after subsequent ADD. */
        if ((op & 0xFF00) == 0x6F00) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            int sub = (op2 >> 5) & 0x7;
            int shift_raw = op2 & 0x1F;
            int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
            int dst_b = (op2 >> 8) & 1;   /* bit 8 = DST/SRC1 */
            int src_b = (op2 >> 9) & 1;   /* bit 9 = SRC (ADD/SUB only) */
            consumed = 2;

            switch (sub) {
            case 0: { /* ADD Smem,SHIFT,SRC,[DST]: DST = SRC + (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src + v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 1: { /* SUB Smem,SHIFT,SRC,[DST]: DST = SRC - (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src - v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 2: { /* LD Smem,SHIFT,DST: DST = data[Smem] << shift (SXM-aware) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                if (dst_b) s->b = sext40(v); else s->a = sext40(v);
                break;
            }
            case 3: { /* STH SRC1,SHIFT,Smem: data[Smem] = (SRC1 high 16) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
                                               : ((int64_t)high >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            case 4: { /* STL SRC1,SHIFT,Smem: data[Smem] = (SRC1 low) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            default:
                { static int unk6f = 0; if (unk6f++ < 10)
                    C54_LOG("0x6F unknown sub=%d op=0x%04x op2=0x%04x PC=0x%04x",
                            sub, op, op2, s->pc); }
                break;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* DEAD CODE since 2026-04-28: all 0x68xx-0x6Fxx now intercepted
             * by specific handlers above (ANDM/ORM/XORM/ADDM/BANZ/BANZD/
             * extended-0x6F00) plus the existing 0x6Dxx MAR. This generic
             * "LD Smem, T" fallback was the source of the 2107-site mass
             * mis-dispatch that caused PC drift on every 0x68xx-0x6Fxx
             * encounter. Kept here for safety in case a previously unseen
             * sub-encoding slips through; if you ever see this trigger,
             * the new handler above for the matching 0xNN00 prefix is
             * incomplete. See doc/opcodes/0x68_0x6F.md. */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1: {
        /* 1xxx: LD / LDU / LDR Smem, DST  (per tic54x-opc.c, all mask FE00):
         *   0x1000  LD  Smem, DST          — signed load (SXM-aware)
         *   0x1200  LDU Smem, DST          — unsigned load (zero-extend)
         *   0x1400  LD  Smem, TS, DST      — load shifted by T low bits
         *   0x1600  LDR Smem, DST          — load with rounding
         *
         * Critical: bootloader at PROM0 0xb429 does `LDU *(0x0ffe), A`
         * (op=0x12f8 + lk=0x0ffe) to read BL_ADDR_LO, then BACC A to that
         * target. The previous "case 0x1: SUB" decoded this as a subtract,
         * leaving A=0 and the BACC dropping into boot-stub NOPs. */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* selects LD/LDU/LD,TS/LDR within case 1 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        switch (sub) {
        case 0x0:  /* 0x1000: LD Smem, DST — signed (SXM honoured) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        case 0x1: { /* 0x1200: LDU Smem, DST — always zero-extended */
            v = (uint16_t)val;
            break;
        }
        case 0x2: { /* 0x1400: LD Smem, TS, DST — shift by T[5:0] (signed) */
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            int64_t base = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (ts >= 0) ? (base << ts) : (base >> -ts);
            break;
        }
        case 0x3: { /* 0x1600: LDR Smem, DST — load with rounding (+0x8000) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (v << 16) + 0x8000;
            v &= 0xFFFFFFFF0000LL;  /* clear low 16 after rounding */
            if (dst) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        default:
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        }
        if (dst) s->b = sext40(v); else s->a = sext40(v);
        /* CALAD-zone LD trace: every LD/LDU/LDR that targets A while
         * executing in DARAM near the CALAD cluster. Reveals what
         * address/value is feeding A right before each CALAD A. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t ldA_total;
            ldA_total++;
            if (ldA_total <= 60 || (ldA_total % 5000) == 0) {
                C54_LOG("LD-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=0x%04x DP=0x%03x",
                        (unsigned long long)ldA_total,
                        s->pc, op, sub, addr, val,
                        (uint16_t)(s->a & 0xFFFF),
                        (s->st0 & 0x1FF));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x0: {
        /* 0xxx: ADD / ADDS / ADD,TS / SUB / SUBS / SUB,TS  (mask FE00):
         *   0x0000 ADD  Smem, SRC1 (no shift, SXM honoured)
         *   0x0200 ADDS Smem, SRC1 (no shift, zero-extended)
         *   0x0400 ADD  Smem, TS, SRC1
         *   0x0800 SUB  Smem, SRC1
         *   0x0A00 SUBS Smem, SRC1
         *   0x0C00 SUB  Smem, TS, SRC1
         * Previous handler always shifted by 16 — wrong for plain ADD/SUB.
         */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* 0..7 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        bool is_sub = (sub & 0x4) != 0;
        bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
        bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
        v = is_unsigned ? (uint16_t)val
                        : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
        if (ts_shift) {
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            v = (ts >= 0) ? (v << ts) : (v >> -ts);
        }
        if (is_sub) {
            if (dst) s->b = sext40(s->b - v);
            else     s->a = sext40(s->a - v);
        } else {
            if (dst) s->b = sext40(s->b + v);
            else     s->a = sext40(s->a + v);
        }
        /* CALAD-zone ADD/SUB trace: same scope as LD-A-TRACE. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t addA_total;
            addA_total++;
            if (addA_total <= 30 || (addA_total % 5000) == 0) {
                C54_LOG("ADDSUB-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=%010llx",
                        (unsigned long long)addA_total,
                        s->pc, op, sub, addr, val,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x3:
        /* 3xxx: MAC / MAS */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            if (dst) s->b = sext40(s->b + product);
            else     s->a = sext40(s->a + product);
        }
        return consumed + s->lk_used;

    case 0x2:
        /* 2xxx: MPY, SQUR, MAS, MAC variants */
        {
            int sub = (op >> 8) & 0xF;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t product;
            int dst;
            switch (sub) {
            case 0x0: case 0x1: /* MPY Smem, A/B */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x4: case 0x5: /* SQUR Smem, A/B */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed + s->lk_used;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed + s->lk_used;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed + s->lk_used;
            }
        }

    case 0x4:
        /* 0x4xxx group — per binutils tic54x-opc.c:
         *   0x40-0x43  SUB Smem,16,src[,dst]    (mask 0xFC00)
         *   0x44-0x45  LD  Smem,16,dst          (mask 0xFE00)
         *   0x4600     LD  Smem,DP              (mask 0xFF00)
         *   0x4700     RPT Smem                 (mask 0xFF00)
         *   0x48-0x49  LDM MMR,dst              (mask 0xFE00)
         *   0x4A00     PSHM MMR                 (mask 0xFF00)
         *   0x4B00     PSHD Smem                (mask 0xFF00)
         *   0x4C00     LTD Smem                 (mask 0xFF00)
         *   0x4D00     DELAY Smem               (mask 0xFF00)
         *   0x4E-0x4F  DST src,Lmem             (mask 0xFE00) */
        {
            uint8_t op8 = hi8;            /* (op >> 8) & 0xFF */
            int dst_b = op8 & 0x01;        /* bit8 = src/dst select (A=0, B=1) */
            int64_t *acc_dst = dst_b ? &s->b : &s->a;

            if (op8 >= 0x40 && op8 <= 0x43) {
                /* SUB Smem << 16, src, dst — sub of shifted Smem from acc */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(*acc_dst - val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x44 || op8 == 0x45) {
                /* LD Smem << 16, dst */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x46) {
                /* LD Smem, DP — load DP from low 9 bits of Smem */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (val & ST0_DP_MASK);
                return consumed + s->lk_used;
            }
            if (op8 == 0x47) {
                /* RPT Smem — load BRC from mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->brc = val;
                s->rpt_active = (val != 0);
                return consumed + s->lk_used;
            }
            if (op8 == 0x48 || op8 == 0x49) {
                /* LDM MMR, dst — load accumulator from a memory-mapped reg */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                *acc_dst = sext40((int16_t)val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4A) {
                /* PSHM MMR — push memory-mapped reg onto stack */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4B) {
                /* PSHD Smem — push data memory onto stack */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4C) {
                /* LTD Smem — T = mem[Smem]; mem[Smem+1] = mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->t = val;
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4D) {
                /* DELAY Smem — mem[Smem+1] = mem[Smem] (delay-line shift) */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4E || op8 == 0x4F) {
                /* DST src, Lmem — store accumulator to long memory.
                 * Lmem = even-aligned 32-bit pair: mem[L]=high, mem[L+1]=low */
                addr = resolve_smem(s, op, &ind) & 0xFFFE;
                int64_t v = *acc_dst;
                data_write(s, addr,         (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, (addr+1)&0xFFFF, (uint16_t)(v & 0xFFFF));
                return consumed + s->lk_used;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 5xxx: shifts — SFTA, SFTL, various forms.
         * NOTE: 0x56xx/0x57xx are SFTL/SFTA with Smem (1-word), NOT MVPD.
         * MVPD is at 0x8Cxx (hi8=0x8C). The old 0x56 MVPD decode was wrong
         * and caused writes to MMR_SP via resolve_smem, corrupting the stack. */
        {
            int dst = (op >> 8) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            int sub = (op >> 9) & 0x7;
            if (sub <= 1) {
                /* 50xx/51xx: SFTA src, ASM shift */
                int shift = asm_shift(s);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 2 || sub == 3) {
                /* 54xx/55xx: SFTA src, #shift (immediate in Smem) */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 4 || sub == 5) {
                /* 58xx/59xx: SFTL src, ASM shift (logical) */
                int shift = asm_shift(s);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else if (sub == 6 || sub == 7) {
                /* 5Cxx/5Dxx/5Exx/5Fxx: SFTL with Smem or other */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
        }
        return consumed + s->lk_used;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */

        /* ---- Dual-operand MAC Xmem, Ymem, dst (1-word) ----
         * 0x90: MAC Xmem,Ymem,A   0x92: MAC Xmem,Ymem,B
         * 0x91: MACR Xmem,Ymem,A  0x93: MACR Xmem,Ymem,B
         * Same encoding as 0xA4 family: OOOO OOOD XXXX YYYY */
        if (hi8 == 0x90 || hi8 == 0x91 || hi8 == 0x92 || hi8 == 0x93) {
            int xar_m = (op >> 4) & 0x07;
            int yar_m = op & 0x07;
            uint16_t xval_m = data_read(s, s->ar[xar_m]);
            uint16_t yval_m = data_read(s, s->ar[yar_m]);
            if ((op >> 7) & 1) s->ar[xar_m]--; else s->ar[xar_m]++;
            if ((op & 0x08) == 0) s->ar[yar_m]++; else s->ar[yar_m]--;
            int64_t prod_m = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_m;
            if (s->st1 & ST1_FRCT) prod_m <<= 1;
            if (hi8 & 0x01) prod_m += 0x8000; /* round */
            int dst_m = (hi8 & 0x02) ? 1 : 0;
            if (dst_m) s->b = sext40(s->b + prod_m);
            else       s->a = sext40(s->a + prod_m);
            s->t = yval_m;
            return consumed + s->lk_used;
        }

        /* 94xx: MVDK Smem, dmad — Move data(Smem) to data(dmad) (2 words) */
        if (hi8 == 0x94) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 95xx: MVKD dmad, Smem — Move data(dmad) to data(Smem) (2 words) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 96xx: MVDP Smem, pmad — Move data to program (2 words) */
        if (hi8 == 0x96) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->prog[op2] = data_read(s, addr);
            return consumed + s->lk_used;
        }

        /* AUDIT FIX 2026-05-08 night : STL ↔ STH swap.
         * Per binutils tic54x-opc.c :
         *   { "stl", 1,3,3, 0x9800, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         *   { "sth", 1,3,3, 0x9A00, 0xFE00, {OP_SRC1,OP_SHFT,OP_Xmem} }
         * Old decoder claimed 0x98/99=STH and 0x9A/9B=STL — exactly inverted.
         * Effect: every STL/STH-with-shift in firmware wrote the WRONG half
         * of the accumulator. Hot pattern in DSP code (post-MAC scaling),
         * so this corrupted ~half of all data writes from compute paths.
         * Shift application is intentionally simplified (no SHFT decode)
         * matching prior-art handlers — Tier B will add proper 4-bit shift
         * decode from low nibble. Mirror swap : write low for 0x98/99,
         * write high for 0x9A/9B, src bit 8 selects A/B. */
        if (hi8 == 0x98 || hi8 == 0x99) {
            /* STL src, SHFT, Xmem — store LOW (acc&0xFFFF).
             * FIX 2026-05-23 : Xmem operand decoded via resolve_xmem (per
             * binutils OP_Xmem), not resolve_smem. The latter mis-mapped
             * low byte 0x00-0x1F with bit 7=0 to MMR space, clobbering SP/
             * IMR/IFR. Empirical proof : PC=0x8a46 op=0x9918 stomp SP→0
             * captured by existing SP-CATASTROPHE probe. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            /* STH src, SHFT, Xmem — store HIGH (acc>>16).
             * FIX 2026-05-23 : same as STL above — Xmem decoded via
             * resolve_xmem (per binutils), not resolve_smem. See STL block. */
            addr = resolve_xmem(s, op);
            int src = hi8 & 1;
            int64_t acc = src ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* SACCD src, Xmem, cond — Conditional accumulator store
         * Encoding: 1001 11SD XXXX COND per SPRU172C p.4-152 */
        if ((op & 0xFC00) == 0x9C00) {
            int src_s = (op >> 9) & 1;
            int64_t acc = src_s ? s->b : s->a;
            int xar_s = (op >> 4) & 0x07;
            uint16_t xaddr = s->ar[xar_s];
            int cond = op & 0x0F;
            /* Evaluate condition */
            int take = 0;
            switch (cond) {
            case 0x0: take = (acc == 0); break;    /* EQ */
            case 0x1: take = (acc != 0); break;    /* NEQ */
            case 0x2: take = (acc > 0); break;     /* GT */
            case 0x3: take = (acc < 0); break;     /* LT */
            case 0x4: take = (acc >= 0); break;    /* GEQ */
            case 0x5: take = (acc == 0); break;    /* AEQ */
            case 0x6: take = (acc > 0); break;     /* AGT */
            case 0x7: take = (acc <= 0); break;    /* LEQ/ALEQ */
            default: take = 0; break;
            }
            int asm_val = asm_shift(s);
            if (take) {
                /* Store shifted accumulator high part */
                int64_t shifted = acc << (asm_val > 0 ? asm_val : 0);
                if (asm_val < 0) shifted = acc >> (-asm_val);
                uint16_t val = (uint16_t)((shifted >> 16) & 0xFFFF);
                data_write(s, xaddr, val);
            } else {
                /* Read and write back (no change) */
                uint16_t val = data_read(s, xaddr);
                data_write(s, xaddr, val);
            }
            /* Xmem post-modify */
            if ((op >> 7) & 1) s->ar[xar_s]--; else s->ar[xar_s]++;
            return consumed + s->lk_used;
        }
        /* POPM MMR — pop top-of-stack into MMR (1-word).
         * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
         * Per SPRU172C section 4 : value at SP popped to MMR, SP++.
         *
         * Bug fix 2026-05-08 : 0x8Axx était précédemment mal décodé en
         * MVDK Smem,dmad (qui est en réalité 0x7100 mask 0xFF00). Le
         * pattern PSHM/POPM symétrique du firmware (e.g. PROM0 0x7013-0x7023
         * sauve/restaure 6 MMRs autour d'un CALA) ne fonctionnait jamais
         * post-CALA → ST1 jamais restauré → INTM=1 dwell perpétuel
         * → IRQ vectoring bloqué → DSP wait stuck → L1 mort.
         * Le case MVDK ci-dessous devient dead code mais est laissé pour
         * référence historique. */
        if ((op & 0xFF00) == 0x8A00) {
            uint16_t mmr = op & 0x7F;
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            data_write(s, mmr, val);
            return consumed + s->lk_used;
        }
        /* OBSOLETE — superseded by POPM above. The 0x8Axx range belongs to
         * POPM per tic54x-opc.c, not MVDK (which is 0x7100 mask 0xFF00).
         * Kept commented for one revision so any caller depending on the
         * old (incorrect) behaviour is forced to be re-examined. */
        if (0 && hi8 == 0x8A) {
            /* MVDK Smem, dmad — INCORRECT for 0x8Axx, see POPM above */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 0x88xx-0x89xx: STLM src, MMR  (1-word!)
         * Per tic54x-opc.c: { "stlm", 1,2,2, 0x8800, 0xFE00, ... }
         *   bits 9-15 = fixed (0x44)
         *   bit 8     = src (0 = A, 1 = B)
         *   bits 0-6  = MMR address (0x00..0x7F)
         *
         * Critical for the DSP bootloader at PROM0 0xb42d (`STLM B, AR1`):
         * if decoded as 2-word MVDM the emulator eats the next opcode
         * (0xb42e = 0xf84c, a BC), then jumps into 0xb431 (MACR family)
         * with an uninitialised T register, producing A=0x10 — which
         * the immediately-following BACC A at 0xb430 then uses as the
         * jump target, dropping the DSP into the boot-stub NOPs at
         * PC=0x0010 instead of continuing the bootloader handshake. */
        if (hi8 == 0x88 || hi8 == 0x89) {
            int src = (op >> 8) & 1;  /* 0 = A, 1 = B */
            int mmr = op & 0x7F;
            uint16_t val = src ? (uint16_t)(s->b & 0xFFFF)
                               : (uint16_t)(s->a & 0xFFFF);
            data_write(s, (uint16_t)mmr, val);  /* MMRs alias addr 0x00..0x1F */
            return consumed + s->lk_used;
        }
        if (hi8 == 0x80) {
            /* AUDIT FIX 2026-05-08 night : was stubbed NOP because old
             * decoder claimed MVDD (2-word, wrong). Per binutils tic54x-opc.c :
             *   { "stl", 1,2,2, 0x8000, 0xFE00, {OP_SRC1,OP_Smem}, 0, REST }
             * 0x80xx/0x81xx = STL src, Smem (1-word, no shift). bit 8 = src.
             * Range 0x8000-0x80FF = STL A, Smem (since bit 8 = 0 here).
             * Stubbing this silently dropped every STL A in the firmware ;
             * variables that should have been written to DARAM kept stale
             * values (junk-state cascade). Mirror of the existing 0x82
             * STH-with-shift handler but no shift here. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->a & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* AUDIT FIX 2026-05-08 night : was MVPD pmad,Smem (2 mots,
             * prog→data move). Per binutils tic54x-opc.c :
             *   { "mvpd", 2,2,2, 0x7C00, 0xFF00, {OP_pmad,OP_Smem}, 0, REST }
             *   { "st",   1,2,2, 0x8C00, 0xFF00, {OP_T,OP_Smem},    0, REST }
             * Real MVPD is at 0x7C — the 0x8C handler should be ST T, Smem
             * (1 mot, store T register to data memory). Run-trace confirms
             * 0 MVPD hits with the old handler, meaning firmware did not
             * issue any 0x7Cxx → our wrong 0x8C MVPD was never triggered
             * for legitimate MVPD anyway (PROM0 OVLY happens via DSP
             * bootloader, not via 0x7C MVPD instruction). Switching to
             * ST T,Smem is safe and unblocks the legitimate ST T pattern
             * used after MAC for T persistence. Old MVPD-LOG instrumentation
             * removed — was dead-code in current run. */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, s->t);
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8E) {
            /* MVDP Smem, pmad (data→prog) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            prog_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8F) {
            /* PORTR PA, Smem — read I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* BSP RX data register — return next burst sample.
             * The DSP firmware uses PORTR PA=0xF430 (64 sites in PROM0,
             * verified from ROM dump). We also accept 0x0034 for legacy
             * compatibility with earlier QEMU experiments. */
            uint16_t portr_val;
            bool is_bsp_pa = (op2 == 0xF430 || op2 == 0x0034);
            if (is_bsp_pa && s->bsp_pos < s->bsp_len) {
                portr_val = s->bsp_buf[s->bsp_pos++];
                data_write(s, addr, portr_val);
            } else {
                portr_val = 0;
                data_write(s, addr, 0);
            }
            /* Per-PA counters so we can see which I/O ports the DSP polls
             * and how often. */
            {
                static uint64_t portr_total[16];
                static uint64_t portr_since_summary;
                int pa_bucket = (op2 >> 4) & 0xF;
                portr_total[pa_bucket]++;
                portr_since_summary++;

                static int portr_log = 0;
                if (portr_log < 50) {
                    C54_LOG("PORTR PA=0x%04x → [0x%04x] val=0x%04x "
                            "bsp_pos=%u/%u PC=0x%04x",
                            op2, addr, portr_val,
                            (unsigned)s->bsp_pos, (unsigned)s->bsp_len,
                            s->pc);
                    portr_log++;
                }
                if ((portr_since_summary % 10000) == 0) {
                    C54_LOG("PORTR summary (last 10000): "
                            "PA0x=%llu 1x=%llu 2x=%llu 3x=%llu 4x=%llu "
                            "5x=%llu 6x=%llu 7x=%llu",
                            (unsigned long long)portr_total[0],
                            (unsigned long long)portr_total[1],
                            (unsigned long long)portr_total[2],
                            (unsigned long long)portr_total[3],
                            (unsigned long long)portr_total[4],
                            (unsigned long long)portr_total[5],
                            (unsigned long long)portr_total[6],
                            (unsigned long long)portr_total[7]);
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Log I/O port writes */
            {
                uint16_t wval = data_read(s, addr);
                static int portw_log = 0;
                if (portw_log < 30) {
                    C54_LOG("PORTW PA=0x%04x val=0x%04x PC=0x%04x", op2, wval, s->pc);
                    portw_log++;
                }
            }
            return consumed + s->lk_used;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed + s->lk_used;
        }
        /* REVERTED 2026-05-15 nuit : handlers 0x72/0x73 RETIRÉS.
         * Voir doc/REVERT_MVMD_KNOWLEDGE.md et premier emplacement revert
         * ci-dessus (avant le bloc `(op & 0xF800) == 0x7000`). 0x86/0x87
         * restent comme avant (DUPLICATE MVDM/MVMD au lieu de STH A/B ASM
         * vrai — non swappés). */

        /* 86xx: MVDM dmad, MMR — DUPLICATE per current emulation
         * (vrai 0x86 per binutils = STH A, ASM, Smem — TODO swap) */
        if (hi8 == 0x86) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x7F;
            data_write(s, mmr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 87xx: MVMD MMR, dmad — DUPLICATE per current emulation
         * (vrai 0x87 per binutils = STH B, ASM, Smem — TODO swap) */
        if (hi8 == 0x87) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x7F;
            data_write(s, op2, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        /* AUDIT FIX 2026-05-15 fin journée : 0x81/0x82/0x83 mal décodés.
         * Per tic54x-opc.c + SPRU172C :
         *   stl 0x8000 / 0xFE00 → 0x80..0x81 STL src,Smem (no shift)
         *   sth 0x8200 / 0xFE00 → 0x82..0x83 STH src,Smem (no shift)
         *   stl 0x8400 / 0xFE00 → 0x84..0x85 STL src,ASM,Smem (with shift) [TODO]
         *   sth 0x8600 / 0xFE00 → 0x86..0x87 STH src,ASM,Smem (with shift) [TODO]
         * bit 8 = src (0=A, 1=B). Old code applied asm_shift incorrectly
         * to 0x81/0x82 (basic variants — no shift) AND used s->a for 0x81
         * (should be s->b). Le bug causait toutes les STL B / STH * vers
         * adressing indirect *ARn à écrire la mauvaise valeur ; en particulier
         * d_burst_d (DSP word 0x0829/0x083D) et d_task_d (0x0828/0x083C) du
         * NDB CCCH demod ARM bail dans prim_rx_nb.c::l1s_nb_resp avec
         * "EMPTY" et "BURST ID 33414!=N" sous synth=1 banc d'essai. */

        /* 0x81xx: STL B, Smem  (src=B, no shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)(s->b & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x82xx: STH A, Smem  (src=A, no shift) */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Bxx: MVDK with long address */
        if (hi8 == 0x8B) {
            /* STUB-NOP : tic54x dit 0x8B = POPD Smem (1-word).
             * Ancienne classification qemu = MVDK long-addr 2-word (incorrect).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé. */
            return 1;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* AUDIT FIX 2026-05-15 fin journée : 0x83 misclassifié comme WRITA
         * (qui est en réalité 0x7F per tic54x-opc.c). Vrai 0x83 = STH B, Smem.
         * Et 0x84 misclassifié comme READA (vrai = 0x7E). Vrai 0x84 = STL A,
         * ASM, Smem (with shift). 0x85..0x87 idem TODO. */
        /* 0x83xx: STH B, Smem  (src=B, no shift) */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->b >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 0x84xx: STL A, ASM, Smem (src=A, with ASM shift) — TODO compléter
         * variantes 0x85 (STL B), 0x86 (STH A), 0x87 (STH B) with ASM shift.
         * Pour l'instant fix uniquement 0x84 vers la sémantique tic54x correcte. */
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 97xx: ST #lk, Smem (2-word). 0x96xx is caught above as MVDP. */
        if (hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * MAC:  dst += T * Xmem; T = Ymem
         * MACR: dst += rnd(T * Xmem); T = Ymem
         * MAS:  dst -= T * Xmem; T = Ymem
         * MASR: dst -= rnd(T * Xmem); T = Ymem
         * Encoding: OOOO OOOD XXXX YYYY (1 word)
         *   Xmem: AR[ARP], post-mod by bit4 (0=inc,1=dec)
         *   Ymem: AR[bits2:0], post-mod by bit3 (0=inc,1=dec)
         *   D: 0=A, 1=B
         * hi8 mapping per SPRU172C:
         *   0xA4/0xA5: MAC[R] Xmem,Ymem,A   0xA6/0xA7: MAC[R] Xmem,Ymem,B
         *   0xB4/0xB5: MAS[R] Xmem,Ymem,A   0xB6/0xB7: MAS[R] Xmem,Ymem,B
         *   0xB0/0xB1: MAC[R] Xmem,Ymem,A (alt)  0xB2/0xB3 already handled
         */
        if (hi8 == 0xA4 || hi8 == 0xA5 || hi8 == 0xA6 || hi8 == 0xA7 ||
            hi8 == 0xB4 || hi8 == 0xB5 || hi8 == 0xB6 || hi8 == 0xB7 ||
            hi8 == 0xB0 || hi8 == 0xB1 || hi8 == 0xB2) {
            int xar_d = (op >> 4) & 0x07;
            int yar_d = op & 0x07;
            uint16_t xval_d = data_read(s, s->ar[xar_d]);
            uint16_t yval_d = data_read(s, s->ar[yar_d]);
            /* Post-modify */
            if ((op >> 7) & 1) s->ar[xar_d]--; else s->ar[xar_d]++;
            if ((op & 0x08) == 0) s->ar[yar_d]++; else s->ar[yar_d]--;
            /* Multiply T * Xmem */
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_d;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            /* Round if R bit set (odd hi8) */
            if (hi8 & 0x01) prod += 0x8000;
            /* Determine dest and operation */
            int is_sub = (hi8 >= 0xB4 && hi8 <= 0xB7);
            int dst_b;
            if (hi8 >= 0xA4 && hi8 <= 0xA7) dst_b = (hi8 >= 0xA6);
            else if (hi8 >= 0xB4 && hi8 <= 0xB7) dst_b = (hi8 >= 0xB6);
            else dst_b = (hi8 & 0x02) ? 1 : 0; /* 0xB0/B1→A, 0xB2/B3→B */
            if (dst_b) {
                if (is_sub) s->b = sext40(s->b - prod);
                else        s->b = sext40(s->b + prod);
            } else {
                if (is_sub) s->a = sext40(s->a - prod);
                else        s->a = sext40(s->a + prod);
            }
            /* T = Ymem */
            s->t = yval_d;
            return consumed + s->lk_used;
        }

        /* SQDST Xmem, Ymem — Squared Distance (1-word dual-operand)
         * Encoding: 1010 0001 XXXX YYYY
         * Per SPRU172C: B += (AH - Xmem)^2; A = Ymem << 16; T = Xmem */
        if (hi8 == 0xA1) {
            int xar_sq = (op >> 4) & 0x07;
            int yar_sq = op & 0x07;
            uint16_t xval_sq = data_read(s, s->ar[xar_sq]);
            uint16_t yval_sq = data_read(s, s->ar[yar_sq]);
            if ((op >> 7) & 1) s->ar[xar_sq]--; else s->ar[xar_sq]++;
            if ((op & 0x08) == 0) s->ar[yar_sq]++; else s->ar[yar_sq]--;
            int16_t ah_sq = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t diff = (int32_t)ah_sq - (int32_t)(int16_t)xval_sq;
            int64_t sq = (int64_t)diff * (int64_t)diff;
            if (s->st1 & ST1_FRCT) sq <<= 1;
            s->b = sext40(s->b + sq);
            s->a = sext40((int64_t)(int16_t)yval_sq << 16);
            s->t = xval_sq;
            return consumed + s->lk_used;
        }

        /* POLY Xmem, Ymem — Polynomial evaluation (1-word dual-operand)
         * Encoding: 1011 110D XXXX YYYY (0xBC=A, 0xBD=B)
         *           1011 111D XXXX YYYY (0xBE/0xBF variants — ABDST or POLY)
         * Per SPRU172C: B += AH * T (with round); A = Xmem << 16; T = Ymem */
        if (hi8 == 0xBC || hi8 == 0xBD || hi8 == 0xBE || hi8 == 0xBF) {
            int xar_p = (op >> 4) & 0x07;
            int yar_p = op & 0x07;
            uint16_t xval_p = data_read(s, s->ar[xar_p]);
            uint16_t yval_p = data_read(s, s->ar[yar_p]);
            if ((op >> 7) & 1) s->ar[xar_p]--; else s->ar[xar_p]++;
            if ((op & 0x08) == 0) s->ar[yar_p]++; else s->ar[yar_p]--;
            int16_t ah_p = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t prod_p = (int64_t)ah_p * (int64_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) prod_p <<= 1;
            prod_p += 0x8000; /* round */
            s->b = sext40(s->b + prod_p);
            s->a = sext40((int64_t)(int16_t)xval_p << 16);
            s->t = yval_p;
            return consumed + s->lk_used;
        }

        /* B8-BB: MAS/MASR Xmem, Ymem (subtract variants) or POLY-like */
        if (hi8 == 0xB8 || hi8 == 0xB9 || hi8 == 0xBA || hi8 == 0xBB) {
            /* Check if it's actually LDMM (BA) or POPM (BD) — those are handled below */
            if (hi8 == 0xBA) goto ba_handler;
            int xar_b8 = (op >> 4) & 0x07;
            int yar_b8 = op & 0x07;
            uint16_t xval_b8 = data_read(s, s->ar[xar_b8]);
            uint16_t yval_b8 = data_read(s, s->ar[yar_b8]);
            if ((op >> 7) & 1) s->ar[xar_b8]--; else s->ar[xar_b8]++;
            if ((op & 0x08) == 0) s->ar[yar_b8]++; else s->ar[yar_b8]--;
            int64_t prod_b8 = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_b8;
            if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
            if (hi8 & 0x01) prod_b8 += 0x8000;
            int dst_b8 = (hi8 & 0x02) ? 1 : 0;
            /* MAS: subtract */
            if (dst_b8) s->b = sext40(s->b - prod_b8);
            else        s->a = sext40(s->a - prod_b8);
            s->t = yval_b8;
            return consumed + s->lk_used;
        }
ba_handler:
        if (hi8 == 0xAA || hi8 == 0xAB) {
            /* STUB-NOP : tic54x dit 0xAA/AB = LD variant.
             * Ancienne classification qemu = STLM src,MMR (incorrect — STLM
             * est en 0x88/0x89, déjà correctement décodé ligne 4046).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst — load MMR value into accumulator
             * Per SPRU172C: dst[15:0] = MMR, dst[31:16] = sign-ext (SXM)/0
             * BUG FIX 2026-05-24 : was `sext40(v << 16)` which put MMR in
             * dst[31:16], wrong half. The boot-stub at 0x0000 (LDMM SP,B)
             * is supposed to return B = current SP for caller's use ; with
             * the << 16 bug, B[31:16] = SP, B[15:0] = 0, breaking any
             * downstream caller that reads B as 16-bit SP value. */
            uint16_t mmr = op & 0x7F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v);
            else     s->a = sext40(v);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA8 || hi8 == 0xA9) {
            /* A8xx/A9xx: AND #lk, src[, dst] (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            *acc = sext40(*acc & ((int64_t)op2 << 16));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA0) {
            /* A0xx: accumulator operations — LD/NEG/ABS/NOT/SFTA/SFTL/SAT
             * Per SPRU172C:
             *   A000/A001: LD B,A / LD A,B
             *   A004/A005: NOT A / NOT B
             *   A008/A009: NEG A / NEG B
             *   A00A/A00B: ABS A / ABS B
             *   A00C/A00D: MAX A / MAX B (sat + clip)
             *   A00E/A00F: MIN A / MIN B
             *   bit7=0: SFTA dst, SHIFT — 1010 0000 0SSS SSSD (arith shift)
             *   bit7=1: SFTL dst, SHIFT — 1010 0000 1SSS SSSD (logical shift)
             *   A098/A099: SAT A / SAT B
             */
            uint8_t sub = op & 0xFF;
            if (sub == 0x00) { s->a = s->b; }
            else if (sub == 0x01) { s->b = s->a; }
            else if (sub == 0x04) { s->a = sext40(~s->a); } /* NOT A */
            else if (sub == 0x05) { s->b = sext40(~s->b); } /* NOT B */
            else if (sub == 0x08) { s->a = sext40(-s->a); } /* NEG A */
            else if (sub == 0x09) { s->b = sext40(-s->b); } /* NEG B */
            else if (sub == 0x0A) { s->a = sext40((s->a < 0) ? -s->a : s->a); } /* ABS A */
            else if (sub == 0x0B) { s->b = sext40((s->b < 0) ? -s->b : s->b); } /* ABS B */
            else if (sub == 0x98) { /* SAT A */
                if (s->a > 0x7FFFFFFFFFLL) s->a = 0x7FFFFFFFFFLL;
                else if (s->a < -0x8000000000LL) s->a = -0x8000000000LL;
                s->st0 &= ~ST0_OVA;
            }
            else if (sub == 0x99) { /* SAT B */
                if (s->b > 0x7FFFFFFFFFLL) s->b = 0x7FFFFFFFFFLL;
                else if (s->b < -0x8000000000LL) s->b = -0x8000000000LL;
                s->st0 &= ~ST0_OVB;
            }
            else if (sub & 0x80) {
                /* SFTL dst, SHIFT — logical shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
            else if (sub >= 0x10) {
                /* SFTA dst, SHIFT — arithmetic shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA5) {
            /* CMPS src, Smem — compare and select (Viterbi) */
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int src = (op >> 4) & 1;
            int64_t acc = src ? s->b : s->a;
            int64_t cmp = (int64_t)(int16_t)val << 16;
            /* TRN shift left, TC set based on comparison */
            s->trn <<= 1;
            if (acc >= cmp) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                if (src) s->b = cmp; else s->a = cmp;
            }
            return consumed + s->lk_used;
        }
        /* AExx/AFxx: MACD Smem, pmad, dst — MAC + data move (2 words)
         * dst += T * Smem, then data(Smem) → data(dmad)
         * pmad in second word auto-increments during RPT */
        if (hi8 == 0xAE || hi8 == 0xAF) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Data move: read from prog[pmad], write to data[addr] */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            data_write(s, addr, prog_fetch(s, psrc));
            s->mvpd_src = psrc + 1;
            s->t = sval;  /* T = old Smem value (before overwrite) */
            return consumed + s->lk_used;
        }
        /* ACxx/ADxx: MACP Smem, pmad, dst — MAC + program fetch (2 words) */
        if (hi8 == 0xAC || hi8 == 0xAD) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Coeff fetch from program memory */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            s->t = prog_fetch(s, psrc);
            s->mvpd_src = psrc + 1;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xB3) {
            /* LD #lk, dst (long immediate, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = (op >> 0) & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        /* ADD #lk, src[, dst] */
        if (hi8 == 0xA2) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b + (v << 16));
            else     s->a = sext40(s->a + (v << 16));
            return consumed + s->lk_used;
        }
        /* SUB #lk */
        if (hi8 == 0xA3) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b - (v << 16));
            else     s->a = sext40(s->a - (v << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * 0xD0: MAC Xmem,Ymem,A   0xD2: MAC Xmem,Ymem,B
         * 0xD1: MACR Xmem,Ymem,A  0xD3: MACR Xmem,Ymem,B
         * 0xD4-0xD7: MAS variants (subtract)
         *
         * Encoding per binutils tic54x.h (XARX/YARX = ((C&0x3)+2)) :
         *   bits 7:6 Xmod  | 5:4 Xar (AR2..AR5) | 3:2 Ymod | 1:0 Yar (AR2..AR5)
         * Was 3-bit AR raw — same bug as C8/CB had (fixed 2026-05-08). Now
         * aligned with binutils. Expected aftermath : new SP-CATASTROPHE on
         * D-class opcodes when firmware ARs land at MMR — same root pattern
         * as 0xc8be at PC=0xa0e7. That's correct exposure, not regression. */
        if (hi8 >= 0xD0 && hi8 <= 0xD9 && hi8 != 0xDA) {
            int xmod_c = (op >> 6) & 0x03;
            int xar_c  = ((op >> 4) & 0x03) + 2;
            int ymod_c = (op >> 2) & 0x03;
            int yar_c  = (op & 0x03) + 2;
            uint16_t xval_c = data_read(s, s->ar[xar_c]);
            uint16_t yval_c = data_read(s, s->ar[yar_c]);
            switch (xmod_c) {
            case 0: break;
            case 1: s->ar[xar_c]++; break;
            case 2: s->ar[xar_c]--; break;
            case 3: s->ar[xar_c] += s->ar[0]; break;
            }
            switch (ymod_c) {
            case 0: break;
            case 1: s->ar[yar_c]++; break;
            case 2: s->ar[yar_c]--; break;
            case 3: s->ar[yar_c] += s->ar[0]; break;
            }
            /* MAC dual-mem formula : T × Xmem (pas X × Y per SPRU pure).
             *
             * 2026-05-08 retest empirique avec pipeline stable :
             *   T×X  : BRC variable, A/B accumulator drift, d_fb_det reaches
             *          high SNR values (0x7902 / 0x7766) at moments
             *   X×Y  : BRC=0 uniforme (201/201), A=B=0 forever, d_fb_det
             *          mostly 0 — correlation produces only zeros
             *
             * Le firmware Calypso s'appuie sur le pipeline c54x : T est
             * latched depuis Ymem du MAC précédent (T = Y(post)). Ainsi
             * MAC dual-mem effectivement calcule `T_old × X_current` =
             * `Y[n-1] × X[n]`. Notre `prod = T × X` reproduit fidèlement
             * cet effet pipelined. `X × Y` (les 2 du buffer courant) ne
             * matche pas la sémantique attendue par le firmware. */
            int64_t prod_c = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_c;
            if (s->st1 & ST1_FRCT) prod_c <<= 1;
            if (hi8 & 0x01) prod_c += 0x8000; /* round */
            int is_sub_c = (hi8 >= 0xD4);
            int dst_c = (hi8 & 0x02) ? 1 : 0;
            if (dst_c) {
                if (is_sub_c) s->b = sext40(s->b - prod_c);
                else          s->b = sext40(s->b + prod_c);
            } else {
                if (is_sub_c) s->a = sext40(s->a - prod_c);
                else          s->a = sext40(s->a + prod_c);
            }
            s->t = yval_c;
            return consumed + s->lk_used;
        }

        /* DBxx: MASA Xmem, Ymem, dst — MAC with accumulator sign extension
         * Per SPRU172C: same as MAC but T loaded from Xmem instead of Ymem.
         * dst += T * Xmem, T = Xmem
         * Encoding fixed 2026-05-08 : same 2-bit AR + offset 2 + 2-bit mod
         * format as the rest of the dual-operand class. */
        if (hi8 == 0xDB) {
            int xmod_db = (op >> 6) & 0x03;
            int xar_db  = ((op >> 4) & 0x03) + 2;
            int ymod_db = (op >> 2) & 0x03;
            int yar_db  = (op & 0x03) + 2;
            uint16_t xval_db = data_read(s, s->ar[xar_db]);
            (void)data_read(s, s->ar[yar_db]); /* Ymem read (unused) */
            switch (xmod_db) {
            case 0: break;
            case 1: s->ar[xar_db]++; break;
            case 2: s->ar[xar_db]--; break;
            case 3: s->ar[xar_db] += s->ar[0]; break;
            }
            switch (ymod_db) {
            case 0: break;
            case 1: s->ar[yar_db]++; break;
            case 2: s->ar[yar_db]--; break;
            case 3: s->ar[yar_db] += s->ar[0]; break;
            }
            int64_t prod_db = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_db;
            if (s->st1 & ST1_FRCT) prod_db <<= 1;
            s->a = sext40(s->a + prod_db);
            s->t = xval_db;
            return consumed + s->lk_used;
        }

        /* DCxx: SQUR Xmem, dst — Square and accumulate (1-word dual-operand)
         * Per SPRU172C p.4-165: T = Xmem, dst = dst + T * T
         * Encoding fixed 2026-05-08 : same dual-operand format as D0-D9. */
        if (hi8 == 0xDC) {
            int xmod_dc = (op >> 6) & 0x03;
            int xar_dc  = ((op >> 4) & 0x03) + 2;
            int ymod_dc = (op >> 2) & 0x03;
            int yar_dc  = (op & 0x03) + 2;
            uint16_t xval_dc = data_read(s, s->ar[xar_dc]);
            (void)data_read(s, s->ar[yar_dc]); /* Ymem pipeline read */
            switch (xmod_dc) {
            case 0: break;
            case 1: s->ar[xar_dc]++; break;
            case 2: s->ar[xar_dc]--; break;
            case 3: s->ar[xar_dc] += s->ar[0]; break;
            }
            switch (ymod_dc) {
            case 0: break;
            case 1: s->ar[yar_dc]++; break;
            case 2: s->ar[yar_dc]--; break;
            case 3: s->ar[yar_dc] += s->ar[0]; break;
            }
            s->t = xval_dc;
            int64_t prod_dc = (int64_t)(int16_t)xval_dc * (int64_t)(int16_t)xval_dc;
            if (s->st1 & ST1_FRCT) prod_dc <<= 1;
            s->a = sext40(s->a + prod_dc);
            return consumed + s->lk_used;
        }

        /* CA/CB handled by the unified C8/C9/CA/CB block below. */
        /* CF: variant parallel or DELAY */
        if (hi8 == 0xCF) {
            /* Treat as NOP for now — rare instruction */
            return consumed + s->lk_used;
        }
        /* RPTB[D] pmad — Block repeat (2 words)
         * C2xx: RPTB pmad, C3xx: RPTBD pmad (delayed)
         * Per SPRU172C: RSA = PC+2, REA = pmad, BRAF = 1 */
        if (hi8 == 0xC2 || hi8 == 0xC3 || hi8 == 0xC6 || hi8 == 0xC7) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC5) {
            /* STUB-NOP : tic54x dit 0xC5 = ST||family (parallel).
             * Ancienne classification qemu = PSHM MMR (incorrect — vrai
             * PSHM est en 0x4A, correctement décodé ligne 3816).
             * Le sp-- ici causait des pushes fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCD) {
            /* STUB-NOP : tic54x dit 0xCD = ST||family (parallel).
             * Ancienne classification qemu = POPM MMR (incorrect — vrai
             * POPM est en 0x8A, fixé 2026-05-08).
             * Le sp++ ici causait des pops fantômes. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xCE) {
            /* STUB-NOP : tic54x dit 0xCE = ST||family (parallel).
             * Ancienne classification qemu = FRAME #k (incorrect — FRAME
             * n'a pas de hi8 fixe, encodage différent).
             * Le sp+=k ici causait des sauts SP arbitraires. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC0 || hi8 == 0xC1) {
            /* PSHD Smem / RPT Smem variants */
            addr = resolve_smem(s, op, &ind);
            if (hi8 == 0xC0) {
                /* PSHD Smem */
                s->sp--;
                data_write(s, s->sp, data_read(s, addr));
            } else {
                /* RPT Smem */
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true;
                s->pc += consumed;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCC) {
            /* CCxx: SACCD Smem, ARmem — Store Acc Conditionally (1-word)
             * Per SPRU172C: conditionally store AH or BH to Smem.
             * Simplified: always store (condition always true). */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDD) {
            /* STUB-NOP : tic54x dit 0xDD = ST||family (parallel) — base
             * 0xDC00 mask 0xFC00. Ancienne classification qemu = POPD Smem
             * (incorrect — vrai POPD en 0x8B, neutralisé en stub).
             * Le sp++ ici causait le SP runaway post-POPM-fix observé
             * 2026-05-08 (~13k faux pops en 64k insn). Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDE) {
            /* STUB-NOP : tic54x dit 0xDE = ST||family (parallel).
             * Ancienne classification qemu = POPD dmad 2-word (incorrect).
             * Le sp++ ici causait le SP runaway. Neutralisé. */
            return 1;
        }
        if (hi8 == 0xDF) {
            /* DELAY Smem — shift delay line: data(Smem) → data(Smem+1)
             * Per SPRU172C: used with RPT for FIR filter delay lines */
            addr = resolve_smem(s, op, &ind);
            uint16_t dval = data_read(s, addr);
            data_write(s, addr + 1, dval);
            return consumed + s->lk_used;
        }
        /* 0xC8/C9/CA/CB: ST SRC, Ymem || LD Xmem, DST  (1-word parallel)
         *
         * Encoding per SPRU172C §5.5 (Parallel store + arithmetic format,
         * cross-checked against tic54x-opc.c entry "0xC800/0xFC00 st||ld") :
         *
         *   bit 15..10 : opcode (110010)
         *   bit  9     : reserved (used to disambiguate; here: 0 for C8/CA,
         *                bit 9 of 0xC9/CB still in opcode space — but the
         *                effective operand bits for parallel are 7:0)
         *   bit  8     : SRC accumulator select (0 = A, 1 = B)
         *   bits 7:6   : Xmod  (0=*ARi  1=*ARi+  2=*ARi-  3=*ARi+0%)
         *   bits 5:4   : Xar   (00=AR2, 01=AR3, 10=AR4, 11=AR5) — only AR2..AR5
         *   bits 3:2   : Ymod  (same encoding as Xmod)
         *   bits 1:0   : Yar   (same encoding as Xar)
         *
         * Bug fix 2026-05-08 v2 evidence (DUAL-OP-INTERPRET log) :
         *   Previously decoded as `xar=(op>>4)&7`, `yar=op&7` (3-bit AR
         *   field) with bit 7 = Xmod ±, bit 3 = Ymod ±. That picked
         *   AR0/AR1 instead of AR2/AR3 and made post-mod always ± with
         *   no support for "no mod" or `*ARi+0%`. When firmware loaded
         *   AR1=0x0018 (= MMR_SP) for an unrelated reason, the *AR1
         *   write landed on the SP MMR slot — observed catastrophes
         *   Δ=+16601 / -16640 at PC=0x7818 / 0x786b are the consequence.
         *
         * Note on 0xCA/CB : per tic54x-opc.c, 0xC800 mask 0xFC00 covers
         * 0xC800..0xCBFF for ST||LD (single instruction class). The
         * earlier emulator split CA/CB into a separate block — that
         * block is now removed, the C8..CB handler is unified here. */
        if (hi8 >= 0xC8 && hi8 <= 0xCB) {
            int s_acc = (hi8 & 0x01) ? 1 : 0;          /* C9/CB store from B */
            int xmod  = (op >> 6) & 0x03;
            int xar   = ((op >> 4) & 0x03) + 2;        /* AR2..AR5 */
            int ymod  = (op >> 2) & 0x03;
            int yar   = (op & 0x03) + 2;               /* AR2..AR5 */
            int d_acc = s_acc ? 0 : 1;                 /* LD into the OTHER acc */
            int64_t st_val = s_acc ? s->b : s->a;
            data_write(s, s->ar[yar], (uint16_t)(st_val & 0xFFFF));
            uint16_t ld_val = data_read(s, s->ar[xar]);
            int64_t loaded = (int64_t)(int16_t)ld_val << 16;
            if (d_acc) s->b = sext40(loaded); else s->a = sext40(loaded);
            switch (xmod) {
            case 0: break;                             /* *ARi (no mod) */
            case 1: s->ar[xar]++; break;               /* *ARi+ */
            case 2: s->ar[xar]--; break;               /* *ARi- */
            case 3: s->ar[xar] += s->ar[0]; break;     /* *ARi+0% (linear approx) */
            }
            switch (ymod) {
            case 0: break;
            case 1: s->ar[yar]++; break;
            case 2: s->ar[yar]--; break;
            case 3: s->ar[yar] += s->ar[0]; break;
            }
            return consumed + s->lk_used;
        }
        goto unimpl;

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 200 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed + s->lk_used;
}

/* ================================================================
 * Main execution loop
 * ================================================================ */

/* DSP idle fast-forward — simulator optimisation, NOT a hack.
 *
 * The Calypso DSP polls its task slots in NDB and write pages while
 * waiting for ARM/TPU to post work. Empirically this dispatcher loop
 * lives in PROM1 mirror at PC 0xe9ac..0xe9b7 (8-instruction body cycled
 * ~285k times per 1.4G insn window when nothing pending). Each iteration
 * costs C-level MAC/branch emulation that ends up consuming 80%+ of host
 * CPU for zero useful work, making QEMU run ~3x slower than wall-clock
 * GSM and starving the BTS scheduler of CLK INDs.
 *
 * Detection: PC inside the polling range AND all four task fields in
 * both write pages are zero AND no interrupt pending. When confirmed,
 * advance cycles/insn_count without invoking c54x_exec_one. The DSP
 * exits idle naturally next iteration if either:
 *   - ARM writes a task field (mirrored via calypso_dsp_write to
 *     s->data[0x0800+offset])
 *   - An IRQ fires (calypso_c54x_interrupt_ex sets s->ifr)
 *   - PC moves outside the range (shouldn't happen while polling)
 *
 * Env vars (default ON) :
 *   CALYPSO_DSP_IDLE_FF=0          disable
 *   CALYPSO_DSP_IDLE_RANGE=lo:hi   override hex PC range
 */
#define DSP_IDLE_FF_MAX_RANGES 4
static bool dsp_idle_fast_forward(C54xState *s, int *consumed_out)
{
    static int     ff_enabled = -1;
    static int     ff_n_ranges = 0;
    static uint16_t ff_lo[DSP_IDLE_FF_MAX_RANGES];
    static uint16_t ff_hi[DSP_IDLE_FF_MAX_RANGES];
    static uint64_t ff_hits = 0;

    if (ff_enabled < 0) {
        const char *e = getenv("CALYPSO_DSP_IDLE_FF");
        ff_enabled = (!e || *e != '0') ? 1 : 0;
        /* Defaults: two empirically observed dispatcher loops in the
         * stock layer1.highram.elf firmware:
         *   1) 0xe9ac..0xe9b7 — PROM1 mirror, init/SP-aware path
         *   2) 0xcc62..0xcc6f — PROM0 page 0, runtime mailbox poll loop
         * Override via CALYPSO_DSP_IDLE_RANGE="lo1:hi1,lo2:hi2,..."
         * (max 4 ranges). Each range is hex. Empty = use defaults. */
        const char *r = getenv("CALYPSO_DSP_IDLE_RANGE");
        if (r && *r) {
            const char *p = r;
            while (*p && ff_n_ranges < DSP_IDLE_FF_MAX_RANGES) {
                unsigned lo, hi;
                if (sscanf(p, "%x:%x", &lo, &hi) == 2 && lo <= hi &&
                    lo <= 0xFFFF && hi <= 0xFFFF) {
                    ff_lo[ff_n_ranges] = (uint16_t)lo;
                    ff_hi[ff_n_ranges] = (uint16_t)hi;
                    ff_n_ranges++;
                }
                while (*p && *p != ',') p++;
                if (*p == ',') p++;
            }
        }
        if (ff_n_ranges == 0) {
            ff_lo[0] = 0xe9ac; ff_hi[0] = 0xe9b7;
            ff_lo[1] = 0xcc62; ff_hi[1] = 0xcc6f;
            ff_n_ranges = 2;
        }
        char buf[160] = ""; int blen = 0;
        for (int i = 0; i < ff_n_ranges; i++) {
            blen += snprintf(buf + blen, sizeof(buf) - blen,
                             "%s0x%04x..0x%04x",
                             i ? "," : "", ff_lo[i], ff_hi[i]);
        }
        C54_LOG("DSP IDLE FF: %s, ranges=[%s]",
                ff_enabled ? "enabled" : "disabled", buf);
    }
    if (!ff_enabled) return false;
    bool in_range = false;
    for (int i = 0; i < ff_n_ranges; i++) {
        if (s->pc >= ff_lo[i] && s->pc <= ff_hi[i]) {
            in_range = true;
            break;
        }
    }
    if (!in_range) return false;

    /* Task slots in both write pages — DSP word addresses :
     *   page 0 : 0x0800 (d_task_d), 0x0802 (d_task_u),
     *            0x0804 (d_task_md), 0x0807 (d_task_ra)
     *   page 1 : 0x0814, 0x0816, 0x0818, 0x081B (offsets +0x14)
     */
    if (s->data[0x0800] | s->data[0x0802] | s->data[0x0804] | s->data[0x0807] |
        s->data[0x0814] | s->data[0x0816] | s->data[0x0818] | s->data[0x081B]) {
        return false;  /* something pending → exec normally */
    }
    /* Pending IRQ would also break us out of the dispatcher next iter. */
    if (!(s->st1 & ST1_INTM) && (s->ifr & s->imr)) {
        return false;
    }

    /* Fast-forward this dispatcher iteration.
     *
     * Cycle-budget calibration: real C54x at 65 MHz means 1 cycle ≈ 15 ns.
     * The dispatcher body is ~8 instructions per pass (matches the 8 hot
     * PCs observed). One pass ≈ 8 cycles ≈ 120 ns of *DSP* time.
     *
     * Per Claude web 2026-05-07 review: previously this returned a
     * fixed 8-cycle skip per call regardless of host wall time. Combined
     * with c54x_run(256000) that meant a single tick callback could
     * burn through 32k FF iterations in microseconds host time but
     * accumulate the full 256k cycles credit on the DSP — the net
     * effect on QEMU virtual time was minimal (DSP cycles aren't a
     * QEMU clock anyway), so this isn't itself the cause of the BTS
     * timing skew. But to match wall-clock more honestly we now cap
     * the FF run length per c54x_run invocation: at most enough skips
     * to consume the budget (n_insns) without overshooting.
     *
     * The actual wall-clock alignment (CLK IND cadence) is owned by
     * the TDMA timer in calypso_trx.c, not by this function. */
    *consumed_out = 8;
    ff_hits++;
    if ((ff_hits & 0xFFFFFFu) == 0) {
        C54_LOG("DSP IDLE FF: %llu skips so far (PC=0x%04x SP=0x%04x)",
                (unsigned long long)ff_hits, s->pc, s->sp);
    }
    return true;
}

/* === CALYPSO_TRAP_OOR hook v2 (root-cause probe SP descent 0..checkpoint) ===
 * v2 redesign: T1/T2 dropped (scheduler exonerated → SP clobber lives in
 * legit code, whitelist can't see it). Pure observability:
 *   - g_sp_trail[256] : SP changes with |Δ|>32 (scheduler reloads, large
 *     allocations) — skip push/pop ±1 noise.
 *   - sp_low watermark : every new low logged (PC-coalesced power-of-10)
 *     — catches BOTH absolute reloads AND push-drain runaway.
 *   - Per-event A_low captured (= candidate STL A,Smem source).
 *   - Halt at fixed checkpoint (env CALYPSO_TRAP_CHECKPOINT, default 4.2M
 *     = just after the insn=4.09M SP recovery 0x0008→0x2900). */

static struct {
    unsigned insn;
    uint16_t old_sp, new_sp, exec_pc, exec_op, a_low;
} g_sp_trail[256];
static unsigned g_sp_trail_idx = 0;

/* sp_low watermark — coalesced by PC */
static uint16_t g_sp_low = 0xFFFF;
static uint16_t g_sp_low_pc = 0xFFFF;
static unsigned g_sp_low_hits_at_pc = 0;
static unsigned g_sp_low_distinct_pcs = 0;

/* SP-decrement histogram per-PC (fix 2026-05-24 v3 — Claude web correction).
 *
 * Gating par VALEUR SP (pas insn_count) — robuste à :
 *   - DSP idle fast-forward (dsp_idle_fast_forward L5937 inflate insn_count
 *     sans exécuter d'opcodes)
 *   - jitter externe wall-clock (bridge/osmocon/BTS sur UDP+PTY → instant
 *     guest où arrive un burst varie run-à-run, insn_count des events
 *     déclenchés par bursts pas stable)
 *
 * Logique : armé quand SP descend SOUS le plateau (default < 0x2000, sous
 * 0x3fb0 où SP stationne 632k→3.5M insns du run jackpot). Reste armé
 * jusqu'au dump quand SP < 0x0100 (proche underflow). Pendant cette fenêtre,
 * compte chaque SP-décrement par PC.
 *
 * 3 bénéfices :
 *   1. Auto-aligne sur la descente quels que soient FF et jitter externe
 *   2. Rend la question FF caduque (descente = real insns, pas idle-poll)
 *   3. Exclut churn équilibré du plateau (PSHM/POP matched à PCs distincts
 *      pollue insn-window mais pas SP-window — leaker domine mécaniquement)
 *
 * Override env :
 *   CALYPSO_SP_HIST_ARM   (default 0x2000) — threshold pour armer
 *   CALYPSO_SP_HIST_DUMP  (default 0x0100) — threshold pour dumper
 *
 * Capture toutes voies SP-write : direct s->sp--, MMR_SP via data_write
 * callback, IRQ push (audit couverture 2026-05-24 : tous paths passent
 * par s->sp variable). */
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint32_t dec_count;
    int32_t  delta_sum;   /* négatif = drain net */
} SpDecEntry;
#define SP_HIST_MAX 512
static SpDecEntry g_sp_dec_hist[SP_HIST_MAX];
static unsigned   g_sp_dec_used = 0;
static unsigned   g_sp_dec_total_events = 0;
static uint16_t   g_sp_dec_arm_threshold = 0;
static uint16_t   g_sp_dec_dump_threshold = 0;
static int        g_sp_dec_enabled = -1;
static int        g_sp_dec_armed = 0;
static int        g_sp_dec_dumped = 0;
static unsigned   g_sp_dec_arm_insn = 0;   /* insn at which we armed */
static uint16_t   g_sp_dec_arm_sp = 0;     /* SP value at arm */

/* === Raw SP ring buffer (Patch 3 — 2026-05-25) ===
 * Per-iteration record of (insn, PC, SP, op) at top-of-loop, no filter,
 * no sign classification. Dumped edge-triggered when SP crosses below
 * the dump threshold (réutilise le trigger du histo, sans freeze).
 * Évite définitivement les bugs de wrap signé qui ont mordu l'histo
 * (underflow v3/v4, overflow v5/maintenant). Brut, donc honnête.
 * Env gates :
 *   CALYPSO_SP_RING=1     active (default OFF, zéro coût sinon)
 *   CALYPSO_SP_RING_MAX=N cap nombre de dumps par run (default 4) */
#define SP_RING_SZ 512  /* must be power of 2 */
typedef struct {
    unsigned insn;
    uint16_t pc;
    uint16_t sp;
    uint16_t op;
    uint16_t _pad;
} SpRingEntry;
static SpRingEntry g_sp_ring[SP_RING_SZ];
static unsigned    g_sp_ring_head = 0;
static uint64_t    g_sp_ring_total = 0;
static int         g_sp_ring_enabled = -1;
static unsigned    g_sp_ring_dump_count = 0;
static unsigned    g_sp_ring_dump_max = 0;

static void sp_ring_record(unsigned insn, uint16_t pc, uint16_t sp, uint16_t op)
{
    if (g_sp_ring_enabled <= 0) return;
    SpRingEntry *e = &g_sp_ring[g_sp_ring_head];
    e->insn = insn; e->pc = pc; e->sp = sp; e->op = op;
    g_sp_ring_head = (g_sp_ring_head + 1) & (SP_RING_SZ - 1);
    g_sp_ring_total++;
}

static void sp_ring_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    if (g_sp_ring_enabled <= 0) return;
    if (g_sp_ring_dump_max && g_sp_ring_dump_count >= g_sp_ring_dump_max) return;
    g_sp_ring_dump_count++;
    fprintf(stderr,
        "[c54x] SP-RING DUMP[%s] @insn=%u sp_now=0x%04x total_recorded=%llu "
        "dump#%u\n",
        trig, insn_now, sp_now,
        (unsigned long long)g_sp_ring_total, g_sp_ring_dump_count);
    unsigned n = (g_sp_ring_total < SP_RING_SZ)
                 ? (unsigned)g_sp_ring_total : SP_RING_SZ;
    unsigned start = (g_sp_ring_total < SP_RING_SZ) ? 0 : g_sp_ring_head;
    for (unsigned k = 0; k < n; k++) {
        unsigned idx = (start + k) & (SP_RING_SZ - 1);
        SpRingEntry *e = &g_sp_ring[idx];
        fprintf(stderr,
            "[c54x] SP-RING[%u] insn=%u PC=0x%04x SP=0x%04x op=0x%04x\n",
            k, e->insn, e->pc, e->sp, e->op);
    }
}

static void sp_ring_init_lazy(void)
{
    if (g_sp_ring_enabled >= 0) return;
    const char *e = getenv("CALYPSO_SP_RING");
    g_sp_ring_enabled = (e && *e == '1') ? 1 : 0;
    const char *m = getenv("CALYPSO_SP_RING_MAX");
    g_sp_ring_dump_max = (m && *m) ? (unsigned)strtoul(m, NULL, 0) : 4u;
    if (g_sp_ring_enabled) {
        fprintf(stderr,
            "[c54x] SP-RING enabled, sz=%u, max_dumps=%u\n",
            SP_RING_SZ, g_sp_ring_dump_max);
    }
}

static void sp_hist_dump(const char *trig, unsigned insn_now, uint16_t sp_now)
{
    fprintf(stderr,
        "[c54x] SP-HIST DUMP[%s] arm@(insn=%u,sp=0x%04x) now@(insn=%u,sp=0x%04x) "
        "events=%u distinct_pcs=%u\n",
        trig, g_sp_dec_arm_insn, g_sp_dec_arm_sp, insn_now, sp_now,
        g_sp_dec_total_events, g_sp_dec_used);

    /* Top-K par dec_count (trickle leak). */
    fprintf(stderr, "[c54x] SP-HIST TOP BY COUNT (corrupteur trickle):\n");
    for (unsigned k = 0; k < 20 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].dec_count > g_sp_dec_hist[best].dec_count)
                best = i;
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        fprintf(stderr,
            "[c54x] SP-HIST #%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }

    /* Top-K par |delta_sum| (single-event jump corrupteur — 1 event huge). */
    fprintf(stderr, "[c54x] SP-HIST TOP BY |delta_sum| (corrupteur single-jump):\n");
    for (unsigned k = 0; k < 10 && k < g_sp_dec_used; k++) {
        unsigned best = k;
        int32_t best_abs = g_sp_dec_hist[k].delta_sum < 0
                           ? -g_sp_dec_hist[k].delta_sum
                           :  g_sp_dec_hist[k].delta_sum;
        for (unsigned i = k + 1; i < g_sp_dec_used; i++) {
            int32_t a = g_sp_dec_hist[i].delta_sum < 0
                        ? -g_sp_dec_hist[i].delta_sum
                        :  g_sp_dec_hist[i].delta_sum;
            if (a > best_abs) { best = i; best_abs = a; }
        }
        if (best != k) {
            SpDecEntry tmp = g_sp_dec_hist[k];
            g_sp_dec_hist[k] = g_sp_dec_hist[best];
            g_sp_dec_hist[best] = tmp;
        }
        fprintf(stderr,
            "[c54x] SP-HIST D#%u pc=0x%04x op_last=0x%04x dec_count=%u "
            "delta_sum=%d\n",
            k + 1, g_sp_dec_hist[k].pc, g_sp_dec_hist[k].op_last,
            g_sp_dec_hist[k].dec_count, g_sp_dec_hist[k].delta_sum);
    }
    g_sp_dec_dumped = 1;
}

static void sp_hist_account(uint16_t exec_pc, uint16_t exec_op,
                            uint16_t sp_before, uint16_t sp_now,
                            unsigned insn)
{
    if (g_sp_dec_enabled < 0) {
        const char *e_arm = getenv("CALYPSO_SP_HIST_ARM");
        const char *e_dump = getenv("CALYPSO_SP_HIST_DUMP");
        unsigned arm  = (e_arm  && *e_arm)  ? (unsigned)strtoul(e_arm,  NULL, 0) : 0x2000u;
        unsigned dump = (e_dump && *e_dump) ? (unsigned)strtoul(e_dump, NULL, 0) : 0x0100u;
        if (arm > 0xFFFF) arm = 0xFFFF;
        if (dump > 0xFFFF) dump = 0xFFFF;
        if (dump >= arm) dump = (arm > 0x100) ? (arm - 0x100) : 0;
        g_sp_dec_arm_threshold  = (uint16_t)arm;
        g_sp_dec_dump_threshold = (uint16_t)dump;
        g_sp_dec_enabled = 1;
        fprintf(stderr,
            "[c54x] SP-HIST gating SP-value : ARM<0x%04x DUMP<0x%04x\n",
            g_sp_dec_arm_threshold, g_sp_dec_dump_threshold);
    }
    if (!g_sp_dec_enabled) return;
    /* Patch 1 (2026-05-25) : freeze RETIRÉ. L'ancien `if (g_sp_dec_dumped)
     * return;` faisait du one-shot, donc tous les events post-1er-dump
     * étaient perdus. Sans freeze, plusieurs dumps consécutifs si SP
     * reste sous threshold — c'est borné en pratique par le rate-limit
     * du sp_ring_dump_max et par le edge-detect dans le top-of-loop. */

    /* Patch 2 (2026-05-25) : drop le cast (int16_t). Le wrap signé
     * mis-classifiait les chutes high→low en pop. Pure int32 sub :
     *   0x9006→0x0000 : delta = -36870 (correct, descent capturé)
     *   0xC000→0x0000 : delta = -49152 (correct, descent capturé)
     * Note : casse l'underflow wrap (0x2bc0→0xfff8 = +52280, vu comme
     * pop), mais l'histo n'est plus la source de vérité pour le kill
     * — c'est le ring buffer qui tranche. Histo = drift trickle uniquement. */
    if (!g_sp_dec_armed) {
        int32_t first_check = (int32_t)sp_now - (int32_t)sp_before;
        if (first_check < 0) {
            g_sp_dec_armed = 1;
            g_sp_dec_arm_insn = insn;
            g_sp_dec_arm_sp = sp_now;
            fprintf(stderr,
                "[c54x] SP-HIST ARMED @insn=%u SP=0x%04x (sp_before=0x%04x delta=%d) "
                "pc=0x%04x op=0x%04x\n",
                insn, sp_now, sp_before, first_check, exec_pc, exec_op);
        } else {
            return;  /* no negative delta yet — wait */
        }
    }

    /* Record event AVANT le dump check (fix 2026-05-24 v4 — sinon un
     * single-event jump qui franchit DUMP en une instruction est perdu). */
    int32_t delta = (int32_t)sp_now - (int32_t)sp_before;
    if (delta < 0) {
        g_sp_dec_total_events++;
        unsigned i;
        for (i = 0; i < g_sp_dec_used; i++) {
            if (g_sp_dec_hist[i].pc == exec_pc) break;
        }
        if (i == g_sp_dec_used) {
            if (g_sp_dec_used >= SP_HIST_MAX) {
                static int sat_log = 0;
                if (!sat_log) {
                    fprintf(stderr,
                        "[c54x] SP-HIST saturated (>%u distinct PCs) — "
                        "broaden if needed\n", SP_HIST_MAX);
                    sat_log = 1;
                }
            } else {
                g_sp_dec_hist[i].pc = exec_pc;
                g_sp_dec_hist[i].op_last = exec_op;
                g_sp_dec_hist[i].dec_count = 0;
                g_sp_dec_hist[i].delta_sum = 0;
                g_sp_dec_used++;
            }
        }
        if (i < g_sp_dec_used) {
            g_sp_dec_hist[i].op_last = exec_op;
            g_sp_dec_hist[i].dec_count++;
            g_sp_dec_hist[i].delta_sum += delta;
        }
        /* Log first 10 events verbatim — for single-event jumps the corrupteur
         * est dans les premiers events (souvent un seul mot dans le histo). */
        if (g_sp_dec_total_events <= 10) {
            fprintf(stderr,
                "[c54x] SP-HIST EVENT #%u pc=0x%04x op=0x%04x "
                "sp_before=0x%04x sp_now=0x%04x delta=%d insn=%u\n",
                g_sp_dec_total_events, exec_pc, exec_op,
                sp_before, sp_now, delta, insn);
        }
    }

    /* DUMP : APRÈS l'accounting, vérifier seuil dump.
     * Patch 1 (2026-05-25) : edge-trigger only — dump quand SP croise
     * sous le floor (sp_before >= threshold && sp_now < threshold).
     * Sans cet edge, sans le freeze, on dumperait à chaque iter une
     * fois SP bas. Avec edge, on dump le plongeon mais pas les iters
     * suivantes où SP reste bas. Si SP remonte puis re-plonge, on
     * re-dump (utile pour observer recovery + re-descente). */
    if (sp_before >= g_sp_dec_dump_threshold && sp_now < g_sp_dec_dump_threshold) {
        sp_ring_dump("sp-floor-cross", insn, sp_now);
        sp_hist_dump("sp-floor-cross", insn, sp_now);
    }
}

static void dsp_trap_dump(C54xState *s, uint16_t exec_pc, uint16_t exec_op,
                          uint16_t sp_before, const char *trig)
{
    fprintf(stderr,
        "[c54x] TRAP[%s] insn=%u exec_pc=0x%04x exec_op=0x%04x "
        "next_pc=0x%04x sp_before=0x%04x sp_now=0x%04x INTM=%d\n",
        trig, s->insn_count, exec_pc, exec_op, s->pc,
        sp_before, s->sp, !!(s->st1 & ST1_INTM));
    fprintf(stderr, "[c54x] TRAP pc_ring[-16..-1]:");
    for (int i = 16; i >= 1; i--)
        fprintf(stderr, " %04x", pc_ring[(pc_ring_idx - i) & 255]);
    fprintf(stderr, "\n[c54x] TRAP sp_low=0x%04x at last_pc=0x%04x hits_at_pc=%u distinct_pcs=%u\n",
            g_sp_low, g_sp_low_pc, g_sp_low_hits_at_pc, g_sp_low_distinct_pcs);
    /* SP-HIST dump (fix v3 2026-05-24 — SP-windowed, no-insn-dep). */
    if (g_sp_dec_used > 0 && !g_sp_dec_dumped)
        sp_hist_dump("trap", s->insn_count, s->sp);
    fprintf(stderr, "[c54x] TRAP sp_trail[-256..-1] (|Δ|>32 only; insn old->new @pc op A_low):\n");
    for (int i = 256; i >= 1; i--) {
        unsigned k = (g_sp_trail_idx - i) & 255;
        if (g_sp_trail[k].insn == 0 && g_sp_trail[k].exec_pc == 0) continue;
        fprintf(stderr, "  %u  %04x->%04x  pc=%04x op=%04x A_low=%04x\n",
                g_sp_trail[k].insn, g_sp_trail[k].old_sp,
                g_sp_trail[k].new_sp, g_sp_trail[k].exec_pc,
                g_sp_trail[k].exec_op, g_sp_trail[k].a_low);
    }
    fprintf(stderr,
        "[c54x] TRAP regs A=%010llx B=%010llx T=%04x  "
        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x  "
        "BK=%04x ARP=%d DP=%d  ST0=%04x ST1=%04x PMST=%04x  "
        "RSA=%04x REA=%04x BRC=%d  IFR=%04x IMR=%04x XPC=%d\n",
        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
        s->t,
        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
        s->bk, (s->st0 >> 13) & 7, s->st0 & 0x1FF,
        s->st0, s->st1, s->pmst,
        s->rsa, s->rea, s->brc, s->ifr, s->imr, s->xpc);
    fprintf(stderr,
        "[c54x] TRAP prog[exec_pc..+3]=%04x %04x %04x %04x  "
        "prog[next_pc..+3]=%04x %04x %04x %04x\n",
        s->prog[exec_pc], s->prog[(uint16_t)(exec_pc+1)],
        s->prog[(uint16_t)(exec_pc+2)], s->prog[(uint16_t)(exec_pc+3)],
        s->prog[s->pc], s->prog[(uint16_t)(s->pc+1)],
        s->prog[(uint16_t)(s->pc+2)], s->prog[(uint16_t)(s->pc+3)]);
}

int c54x_run(C54xState *s, int n_insns)
{
    int executed = 0;

    /* Log first 10 instructions of each run (for 2nd cycle debug) */
    static int run_num = 0;
    run_num++;

    /* SP history ring buffer (64 entries × insn/PC/SP). Sampled every
     * 1M insns at top of run-loop. Dumped on STATE-DUMP. Reveals whether
     * SP descends monotonically (cumulative leak — each ISR entry leaks
     * one stack frame) or oscillates around a value (one big initial
     * drop then steady-state). Different fixes. */
    static struct { unsigned insn; uint16_t pc; uint16_t sp; } sp_ring[64];
    static unsigned sp_ring_idx = 0;
    static unsigned next_sp_sample = 1000000u;
    if (s->insn_count >= next_sp_sample) {
        next_sp_sample += 1000000u;
        sp_ring[sp_ring_idx & 63].insn = s->insn_count;
        sp_ring[sp_ring_idx & 63].pc   = s->pc;
        sp_ring[sp_ring_idx & 63].sp   = s->sp;
        sp_ring_idx++;
    }

    /* XPC tracking probe (2026-05-15 nuit, per Claude web Q1).
     * Hypothèse à valider : le path completion CCCH demod passe par PROM1
     * (XPC=1) via le B 0x9ab1 à 0x19aac. Si XPC=1 jamais atteint → bug
     * dans le route initial. Si atteint mais PC pas dans 0x9aac+ → entrée
     * OK mais pas cette zone. Tracking :
     *   - insn count par XPC (0..3)
     *   - dernier PC visité par XPC
     *   - first_visit_insn par XPC (= quand on entre en XPC=N pour la 1ère fois)
     *   - ring buffer 16 derniers PCs visités sous XPC=1 (zone d'intérêt)
     */
    {
        static uint64_t xpc_insn_count[4] = {0};
        static uint16_t xpc_last_pc[4]    = {0};
        static uint64_t xpc_first_insn[4] = {0,0,0,0};
        static uint16_t xpc1_pc_ring[16];
        static unsigned xpc1_pc_ring_idx = 0;
        static unsigned xpc1_pc_ring_count = 0;
        static unsigned next_xpc_dump = 100000000u;  /* 100M */
        uint8_t cur_xpc = s->xpc & 0x3;
        xpc_insn_count[cur_xpc]++;
        xpc_last_pc[cur_xpc] = s->pc;
        if (xpc_first_insn[cur_xpc] == 0)
            xpc_first_insn[cur_xpc] = s->insn_count;
        if (cur_xpc == 1) {
            xpc1_pc_ring[xpc1_pc_ring_idx & 15] = s->pc;
            xpc1_pc_ring_idx++;
            xpc1_pc_ring_count++;
        }
        if (s->insn_count >= next_xpc_dump) {
            next_xpc_dump += 100000000u;
            fprintf(stderr,
                    "[c54x] XPC-STATS insn=%u counts: 0=%llu 1=%llu 2=%llu 3=%llu | "
                    "first_insn: 0=%llu 1=%llu 2=%llu 3=%llu | last_pc: 0=0x%04x 1=0x%04x 2=0x%04x 3=0x%04x\n",
                    s->insn_count,
                    (unsigned long long)xpc_insn_count[0],
                    (unsigned long long)xpc_insn_count[1],
                    (unsigned long long)xpc_insn_count[2],
                    (unsigned long long)xpc_insn_count[3],
                    (unsigned long long)xpc_first_insn[0],
                    (unsigned long long)xpc_first_insn[1],
                    (unsigned long long)xpc_first_insn[2],
                    (unsigned long long)xpc_first_insn[3],
                    xpc_last_pc[0], xpc_last_pc[1], xpc_last_pc[2], xpc_last_pc[3]);
            if (xpc1_pc_ring_count > 0) {
                /* Dernier 16 PCs visités sous XPC=1 (ring buffer) */
                fprintf(stderr,
                        "[c54x] XPC1-PC-RING count=%u last16: "
                        "%04x %04x %04x %04x %04x %04x %04x %04x "
                        "%04x %04x %04x %04x %04x %04x %04x %04x\n",
                        xpc1_pc_ring_count,
                        xpc1_pc_ring[(xpc1_pc_ring_idx-16)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-15)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-14)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-13)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-12)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-11)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-10)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-9)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-8)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-7)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-6)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-5)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-4)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-3)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-2)&15],
                        xpc1_pc_ring[(xpc1_pc_ring_idx-1)&15]);
            }
        }
    }

    /* DISPATCH-CALLER probe (2026-05-15 nuit, per Claude web).
     * Les 3 callers de 0x9aaf identifiés par scan PROM :
     *   PC=0x8815 : f074 9aaf  (B 0x9aaf depuis table @0x8810)
     *   PC=0x9296 : f274 9aaf  (BD 0x9aaf depuis routine spécifique)
     *   PC=0x9418 : f274 9aaf  (BD 0x9aaf depuis autre routine)
     * Log A, AR0..2, data[0x0828/9] à chaque hit. */
    if (s->pc == 0x8815 || s->pc == 0x9296 || s->pc == 0x9418) {
        static unsigned hit_counts[3] = {0, 0, 0};
        int idx = (s->pc == 0x8815) ? 0 : (s->pc == 0x9296) ? 1 : 2;
        hit_counts[idx]++;
        if (hit_counts[idx] <= 20 || hit_counts[idx] % 100 == 0) {
            fprintf(stderr,
                    "[c54x] DISPATCH-CALLER hit=%u pc=0x%04x "
                    "A=0x%010llx AR0=0x%04x AR1=0x%04x AR2=0x%04x "
                    "data[0x0828]=0x%04x data[0x0829]=0x%04x "
                    "data[0x083c]=0x%04x data[0x083d]=0x%04x insn=%u\n",
                    hit_counts[idx], s->pc,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->ar[0], s->ar[1], s->ar[2],
                    s->data[0x0828], s->data[0x0829],
                    s->data[0x083c], s->data[0x083d],
                    s->insn_count);
        }
    }

    /* AR7-INIT-CHAIN + MVMD-AR7-BRC + RPTB-ARMED probe (Claude web 2026-05-15
     * nuit étape 3). Diagnostic : valeur AR7 au moment du MVMD AR7,BRC à
     * PC=0x8208, sa chaîne causale (16 derniers writes AR7), et l'état BRC
     * post-RPTBD setup. */
    {
        static uint16_t prev_ar7 = 0xFFFF;
        static struct {
            uint16_t pc;
            uint16_t old_val;
            uint16_t new_val;
            uint64_t insn;
            uint8_t  xpc;
        } ar7_history[16] = {{0}};
        static unsigned ar7_hist_idx = 0;

        if (s->ar[7] != prev_ar7) {
            ar7_history[ar7_hist_idx & 15].pc = s->pc;
            ar7_history[ar7_hist_idx & 15].old_val = prev_ar7;
            ar7_history[ar7_hist_idx & 15].new_val = s->ar[7];
            ar7_history[ar7_hist_idx & 15].insn = s->insn_count;
            ar7_history[ar7_hist_idx & 15].xpc = s->xpc & 0x3;
            ar7_hist_idx++;
            prev_ar7 = s->ar[7];
        }

        /* (b) Snapshot complet à chaque hit de PC=0x8208 (MVMD AR7, BRC) */
        if (s->pc == 0x8208) {
            static unsigned mvmd_hits = 0;
            mvmd_hits++;
            if (mvmd_hits <= 20 || (mvmd_hits % 100) == 0) {
                fprintf(stderr,
                        "[c54x] MVMD-AR7-BRC #%u AR7=0x%04x BRC_before=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR6=0x%04x DP=%d insn=%u\n",
                        mvmd_hits, s->ar[7], s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[6], dp(s),
                        s->insn_count);
                int n_hist = ar7_hist_idx < 16 ? ar7_hist_idx : 16;
                for (int i = 0; i < n_hist; i++) {
                    int slot = (ar7_hist_idx - n_hist + i) & 15;
                    fprintf(stderr,
                            "[c54x]   AR7-HIST[%d] pc=XPC%u:0x%04x 0x%04x->0x%04x insn=%llu\n",
                            i, ar7_history[slot].xpc, ar7_history[slot].pc,
                            ar7_history[slot].old_val, ar7_history[slot].new_val,
                            (unsigned long long)ar7_history[slot].insn);
                }
            }
        }

        /* (c) État RPTB après setup (PC=0x820c = delay slot post-RPTBD) */
        if (s->pc == 0x820c) {
            static unsigned rptb_hits = 0;
            rptb_hits++;
            if (rptb_hits <= 20 || (rptb_hits % 100) == 0) {
                fprintf(stderr,
                        "[c54x] RPTB-ARMED #%u BRC=0x%04x RSA=0x%04x REA=0x%04x "
                        "ST1=0x%04x (INTM=%d) insn=%u\n",
                        rptb_hits, s->brc, s->rsa, s->rea, s->st1,
                        (s->st1 >> 11) & 1, s->insn_count);
            }
        }
    }

    /* INT3-BLOCKED probe (Claude web 2026-05-15 nuit, étape 2).
     * Sample 1/1000 du context (PC/ST1/BRC/XPC) quand INT3 pending + INTM=1.
     * Discrimine : (a) opcode set INTM=1 sans clear (variante POPM),
     * (b) RPTB long non-interruptible (BRC > 0 partout),
     * (c) STM ST1 / MVDM ST1 brut. Cf matrice Claude web. */
    {
        static uint64_t blocked_count = 0;
        static uint16_t sample_pcs[32] = {0};
        static uint16_t sample_st1s[32] = {0};
        static uint16_t sample_brcs[32] = {0};
        static uint8_t  sample_xpcs[32] = {0};
        static unsigned sample_idx = 0;
        static unsigned next_blocked_dump = 100000000u;

        bool int3_pending_now = (s->ifr & 0x08) != 0;
        bool intm_set_now = ((s->st1 >> 11) & 1) != 0;

        if (int3_pending_now && intm_set_now) {
            blocked_count++;
            if ((blocked_count % 1000) == 0) {
                sample_pcs[sample_idx & 31] = s->pc;
                sample_st1s[sample_idx & 31] = s->st1;
                sample_brcs[sample_idx & 31] = s->brc;
                sample_xpcs[sample_idx & 31] = s->xpc & 0x3;
                sample_idx++;
            }
        }

        if (s->insn_count >= next_blocked_dump) {
            next_blocked_dump += 100000000u;
            fprintf(stderr,
                    "[c54x] INT3-BLOCKED insn=%u blocked_total=%llu blocked_samples=%u\n",
                    s->insn_count,
                    (unsigned long long)blocked_count,
                    sample_idx);
            int n = sample_idx < 32 ? sample_idx : 32;
            for (int i = 0; i < n; i++) {
                int slot = (sample_idx - n + i) & 31;
                fprintf(stderr,
                        "[c54x] INT3-BLOCKED-SAMPLE pc=XPC%u:0x%04x st1=0x%04x brc=0x%04x\n",
                        sample_xpcs[slot], sample_pcs[slot],
                        sample_st1s[slot], sample_brcs[slot]);
            }
        }
    }

    /* IRQ-FRAME-HEALTH probe (Claude web 2026-05-15 nuit, étape 1).
     * Diagnostic timing TDMA vs wall-clock : INT3 = frame interrupt
     * (IMR bit 3, vec 19, addr 0xFFCC). Mesure fire/serviced/missed/latency.
     * Discrimine : ISR mal vectorisée (service<fire), TPU/TSP fail (fire=0),
     * compute trop lent (missed>0). Cause root LOST 3468 + variance XPC. */
    {
        static uint64_t int3_fire_count = 0;
        static uint64_t int3_serviced_count = 0;
        static uint64_t int3_missed_count = 0;
        static uint64_t last_int3_fire_insn = 0;
        static uint64_t last_int3_service_insn = 0;
        static uint64_t total_service_latency_insn = 0;
        static bool int3_pending_prev = false;
        static unsigned next_irq_dump = 200000000u;

        bool int3_now_pending = (s->ifr & 0x08) != 0;
        bool int3_just_fired = int3_now_pending && !int3_pending_prev;
        /* ISR enter approximation : INT3 cleared from IFR while INTM=0 */
        bool int3_just_serviced = !int3_now_pending && int3_pending_prev &&
                                  ((s->st1 >> 11) & 1) == 0;

        if (int3_just_fired) {
            int3_fire_count++;
            if (int3_pending_prev) {
                int3_missed_count++;
            }
            last_int3_fire_insn = s->insn_count;
        }
        if (int3_just_serviced) {
            int3_serviced_count++;
            if (last_int3_fire_insn > last_int3_service_insn) {
                total_service_latency_insn += (s->insn_count - last_int3_fire_insn);
            }
            last_int3_service_insn = s->insn_count;
        }
        int3_pending_prev = int3_now_pending;

        if (s->insn_count >= next_irq_dump) {
            next_irq_dump += 200000000u;
            uint64_t avg_latency = int3_serviced_count > 0
                ? total_service_latency_insn / int3_serviced_count : 0;
            double service_ratio = int3_fire_count > 0
                ? (double)int3_serviced_count / int3_fire_count : 0.0;
            fprintf(stderr,
                    "[c54x] IRQ-FRAME-HEALTH insn=%u int3_fire=%llu int3_serviced=%llu "
                    "int3_missed=%llu avg_latency_insn=%llu service_ratio=%.2f\n",
                    s->insn_count,
                    (unsigned long long)int3_fire_count,
                    (unsigned long long)int3_serviced_count,
                    (unsigned long long)int3_missed_count,
                    (unsigned long long)avg_latency,
                    service_ratio);
        }
    }

    /* EXIT-COMPUTE + IRQ-DURING-COMPUTE probe (Claude web 2026-05-15 nuit).
     * Le DSP tourne en XPC=2 dans zone hot 0xdf80..0xdfc0 (CCCH demod MAC loop).
     * Discrimine entre 3 hypothèses :
     *   (1) compute jamais exit (threshold non franchi)
     *   (2) IRQ jamais fire (TPU/TSP source manquante)
     *   (3) IRQ fire mais pas serviced (INTM stuck ou ISR mal vectorisée)
     * Matrice de décision basée sur exits_count + irq_pending_in_compute. */
    {
        static uint16_t last_pc_sample = 0;
        static uint8_t  last_xpc_sample = 0;
        static unsigned exits_count = 0;
        static unsigned irqs_pending_during_compute = 0;
        static unsigned int3_pending_during_compute = 0;
        static uint64_t insns_in_compute = 0;
        static unsigned next_compute_dump = 200000000u;

        bool in_compute_now = ((s->xpc & 0x3) == 2 &&
                               s->pc >= 0xdf80 && s->pc <= 0xdfc0);
        bool was_in_compute = (last_xpc_sample == 2 &&
                               last_pc_sample >= 0xdf80 && last_pc_sample <= 0xdfc0);

        if (was_in_compute && !in_compute_now) {
            exits_count++;
            if (exits_count <= 30 || exits_count % 200 == 0) {
                fprintf(stderr,
                        "[c54x] EXIT-COMPUTE #%u from=XPC%u:0x%04x to=XPC%u:0x%04x "
                        "A=0x%010llx IFR=0x%04x INTM=%d insn=%u\n",
                        exits_count,
                        last_xpc_sample, last_pc_sample,
                        s->xpc & 0x3, s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->ifr, (s->st1 >> 11) & 1, s->insn_count);
            }
        }

        if (in_compute_now) {
            insns_in_compute++;
            if (s->ifr != 0) {
                irqs_pending_during_compute++;
                if (s->ifr & 0x08) int3_pending_during_compute++;
            }
        }

        if (s->insn_count >= next_compute_dump) {
            next_compute_dump += 200000000u;
            fprintf(stderr,
                    "[c54x] COMPUTE-STATS insn=%u in_compute=%llu exits=%u "
                    "irq_pending_in_compute=%u int3_pending_in_compute=%u\n",
                    s->insn_count,
                    (unsigned long long)insns_in_compute,
                    exits_count,
                    irqs_pending_during_compute,
                    int3_pending_during_compute);
        }

        last_pc_sample = s->pc;
        last_xpc_sample = s->xpc & 0x3;
    }

    /* DISPATCH-ENTRY probe (per Claude web option 3 hybride).
     * Le dispatcher caller saute vers 0x8810 + task_id*3, où chaque entry =
     * { 0xf4e4 (FRET ou padding), 0xf074 (B opcode), <target> }.
     * On probe le PC qui correspond au début d'un entry (PC = 0x8810 + N*3).
     * task_id estimé = (PC - 0x8810) / 3.
     * Si entry exec OK → on lit data[PC+2] qui est le target. */
    if (s->pc >= 0x8810 && s->pc < 0x8900 && ((s->pc - 0x8810) % 3) == 0) {
        static unsigned entry_hits = 0;
        entry_hits++;
        if (entry_hits <= 50 || entry_hits % 200 == 0) {
            uint16_t entry_idx = (s->pc - 0x8810) / 3;
            uint16_t header = s->prog[s->pc];     /* normally 0xf4e4 */
            uint16_t branch = s->prog[s->pc + 1]; /* normally 0xf074 */
            uint16_t target = s->prog[s->pc + 2];
            fprintf(stderr,
                    "[c54x] DISPATCH-ENTRY #%u pc=0x%04x entry_idx=%u "
                    "header=0x%04x branch=0x%04x target=0x%04x "
                    "A=0x%010llx insn=%u\n",
                    entry_hits, s->pc, entry_idx,
                    header, branch, target,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                    s->insn_count);
        }
    }

    /* Periodic DSP state dump (every 500M insns, starting at 500M).
     * Captures: state regs, hot-zone disasm (0xa2c0..0xa2d0 + 0xb8e0..0xb910),
     * vector table at current PMST IPTR base, hot-PC opcodes, SP history. */
    {
        static unsigned next_dump = 500000000u;
        if (s->insn_count >= next_dump) {
            next_dump += 500000000u;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            uint16_t vbase = iptr * 0x80;
            C54_LOG("STATE-DUMP insn=%u PC=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d IMR=0x%04x IFR=0x%04x XPC=%d PMST=0x%04x SP=0x%04x AR1=0x%04x AR2=0x%04x BRC=%d",
                    s->insn_count, s->pc, s->st0, s->st1,
                    !!(s->st1 & ST1_INTM),
                    s->imr, s->ifr, s->xpc, s->pmst, s->sp,
                    s->ar[1], s->ar[2], s->brc);
            C54_LOG("STATE-DUMP prog[0xa2c0..0xa2d0]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xa2c0], s->prog[0xa2c1], s->prog[0xa2c2], s->prog[0xa2c3],
                    s->prog[0xa2c4], s->prog[0xa2c5], s->prog[0xa2c6], s->prog[0xa2c7],
                    s->prog[0xa2c8], s->prog[0xa2c9], s->prog[0xa2ca], s->prog[0xa2cb],
                    s->prog[0xa2cc], s->prog[0xa2cd], s->prog[0xa2ce], s->prog[0xa2cf],
                    s->prog[0xa2d0]);
            /* Hot zone after ARP fix: b8e9..b906 (run 2, vec1 handler). */
            C54_LOG("STATE-DUMP prog[0xb8e0..0xb910]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb8e0], s->prog[0xb8e1], s->prog[0xb8e2], s->prog[0xb8e3],
                    s->prog[0xb8e4], s->prog[0xb8e5], s->prog[0xb8e6], s->prog[0xb8e7],
                    s->prog[0xb8e8], s->prog[0xb8e9], s->prog[0xb8ea], s->prog[0xb8eb],
                    s->prog[0xb8ec], s->prog[0xb8ed], s->prog[0xb8ee], s->prog[0xb8ef],
                    s->prog[0xb8f0], s->prog[0xb8f1], s->prog[0xb8f2], s->prog[0xb8f3],
                    s->prog[0xb8f4], s->prog[0xb8f5], s->prog[0xb8f6], s->prog[0xb8f7],
                    s->prog[0xb8f8], s->prog[0xb8f9], s->prog[0xb8fa], s->prog[0xb8fb],
                    s->prog[0xb8fc], s->prog[0xb8fd], s->prog[0xb8fe], s->prog[0xb8ff],
                    s->prog[0xb900]);
            C54_LOG("STATE-DUMP prog[0xb900..0xb920]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0xb900], s->prog[0xb901], s->prog[0xb902], s->prog[0xb903],
                    s->prog[0xb904], s->prog[0xb905], s->prog[0xb906], s->prog[0xb907],
                    s->prog[0xb908], s->prog[0xb909], s->prog[0xb90a], s->prog[0xb90b],
                    s->prog[0xb90c], s->prog[0xb90d], s->prog[0xb90e], s->prog[0xb90f],
                    s->prog[0xb910], s->prog[0xb911], s->prog[0xb912], s->prog[0xb913],
                    s->prog[0xb914], s->prog[0xb915], s->prog[0xb916], s->prog[0xb917],
                    s->prog[0xb918], s->prog[0xb919], s->prog[0xb91a], s->prog[0xb91b],
                    s->prog[0xb91c], s->prog[0xb91d], s->prog[0xb91e], s->prog[0xb91f],
                    s->prog[0xb920]);
            C54_LOG("STATE-DUMP vbase=0x%04x prog[vbase..vbase+0x18]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    vbase,
                    s->prog[vbase+0x00], s->prog[vbase+0x01], s->prog[vbase+0x02], s->prog[vbase+0x03],
                    s->prog[vbase+0x04], s->prog[vbase+0x05], s->prog[vbase+0x06], s->prog[vbase+0x07],
                    s->prog[vbase+0x08], s->prog[vbase+0x09], s->prog[vbase+0x0a], s->prog[vbase+0x0b],
                    s->prog[vbase+0x0c], s->prog[vbase+0x0d], s->prog[vbase+0x0e], s->prog[vbase+0x0f],
                    s->prog[vbase+0x10], s->prog[vbase+0x11], s->prog[vbase+0x12], s->prog[vbase+0x13],
                    s->prog[vbase+0x14], s->prog[vbase+0x15], s->prog[vbase+0x16], s->prog[vbase+0x17],
                    s->prog[vbase+0x18]);
            /* Hot-PC opcode dump for known correlator/handler sites */
            C54_LOG("STATE-DUMP HOT-OPS: 0x8d33=%04x 0x8eb9=%04x 0x8f51=%04x 0xa2c7=%04x 0xa2c8=%04x 0xb8e9=%04x 0xb8eb=%04x 0xb8f4=%04x 0xb8f5=%04x 0xb906=%04x",
                    s->prog[0x8d33], s->prog[0x8eb9], s->prog[0x8f51],
                    s->prog[0xa2c7], s->prog[0xa2c8],
                    s->prog[0xb8e9], s->prog[0xb8eb], s->prog[0xb8f4],
                    s->prog[0xb8f5], s->prog[0xb906]);
            /* DARAM 0x066F..0x0682 wait-loop disasm (run 3 stuck zone).
             * Looking for B-self (f073 066f) vs IDLE n (f7e1/f7e2/f7e3)
             * vs poll-and-branch. If IDLE found → emulator IDLE handler
             * is the real bug (3 runs all hit the same opcode, terminate
             * in different bassins because PMST/IPTR varies). */
            C54_LOG("STATE-DUMP prog[0x0660..0x0690]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x0660], s->prog[0x0661], s->prog[0x0662], s->prog[0x0663],
                    s->prog[0x0664], s->prog[0x0665], s->prog[0x0666], s->prog[0x0667],
                    s->prog[0x0668], s->prog[0x0669], s->prog[0x066a], s->prog[0x066b],
                    s->prog[0x066c], s->prog[0x066d], s->prog[0x066e], s->prog[0x066f],
                    s->prog[0x0670], s->prog[0x0671], s->prog[0x0672], s->prog[0x0673],
                    s->prog[0x0674], s->prog[0x0675], s->prog[0x0676], s->prog[0x0677],
                    s->prog[0x0678], s->prog[0x0679], s->prog[0x067a], s->prog[0x067b],
                    s->prog[0x067c], s->prog[0x067d], s->prog[0x067e], s->prog[0x067f],
                    s->prog[0x0680]);
            /* Same range but data[] view in case OVLY=1 routes fetches
             * to data array (different memory than prog). */
            C54_LOG("STATE-DUMP data[0x0660..0x0680]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->data[0x0660], s->data[0x0661], s->data[0x0662], s->data[0x0663],
                    s->data[0x0664], s->data[0x0665], s->data[0x0666], s->data[0x0667],
                    s->data[0x0668], s->data[0x0669], s->data[0x066a], s->data[0x066b],
                    s->data[0x066c], s->data[0x066d], s->data[0x066e], s->data[0x066f],
                    s->data[0x0670], s->data[0x0671], s->data[0x0672], s->data[0x0673],
                    s->data[0x0674], s->data[0x0675], s->data[0x0676], s->data[0x0677],
                    s->data[0x0678], s->data[0x0679], s->data[0x067a], s->data[0x067b],
                    s->data[0x067c], s->data[0x067d], s->data[0x067e], s->data[0x067f],
                    s->data[0x0680]);
            /* IRQ entry handler at PC=0x1854 (last 0→1 transition) */
            C54_LOG("STATE-DUMP prog[0x1850..0x1860]: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    s->prog[0x1850], s->prog[0x1851], s->prog[0x1852], s->prog[0x1853],
                    s->prog[0x1854], s->prog[0x1855], s->prog[0x1856], s->prog[0x1857],
                    s->prog[0x1858], s->prog[0x1859], s->prog[0x185a], s->prog[0x185b],
                    s->prog[0x185c], s->prog[0x185d], s->prog[0x185e], s->prog[0x185f],
                    s->prog[0x1860]);
            /* SP history ring (last 32 sampled at 1M-insn intervals) */
            {
                char buf[2048]; int o = 0;
                int start = (sp_ring_idx >= 32) ? (sp_ring_idx - 32) : 0;
                for (unsigned i = start; i < sp_ring_idx; i++) {
                    int idx = i & 63;
                    o += snprintf(buf+o, sizeof(buf)-o,
                                  "[%u:PC=%04x SP=%04x] ",
                                  sp_ring[idx].insn,
                                  sp_ring[idx].pc,
                                  sp_ring[idx].sp);
                    if (o > (int)sizeof(buf) - 64) break;
                }
                C54_LOG("STATE-DUMP SP-RING (last %d): %s",
                        (int)(sp_ring_idx >= 32 ? 32 : sp_ring_idx), buf);
            }
        }
    }

    while (executed < n_insns && s->running && !s->idle) {
        /* === TOP-OF-LOOP SP CHOKEPOINT (fix 2026-05-24 v6 Claude web) ===
         * Le hook SP existant est en BAS de boucle (L7471). Toute
         * instruction qui sort tôt (goto unimpl, return, continue, handler
         * qui sort de la dispatch chain) bypasse le hook → l'écriture SP
         * a lieu mais n'est pas comptabilisée. Hier l'audit "tout passe
         * par s->sp" était correct sur les SITES d'écriture mais ne
         * vérifiait pas si le hook tourne pour ces instructions.
         *
         * Symptôme : 61 events captés vs descente attendue de 11k+ mots
         * → la descente passe par bypass(es). Fix : observer s->sp à un
         * CHOKEPOINT obligé (top de boucle), comparer avec la valeur de
         * l'itération précédente. Bypass-proof par construction : on
         * regarde la VALEUR à un point de passage, pas le SITE.
         *
         * Implementation : statics (persistent inter-c54x_run-calls). */
        {
            static uint16_t topgate_last_sp = 0;
            static uint16_t topgate_last_pc = 0;
            static uint16_t topgate_last_op = 0;
            static int      topgate_valid   = 0;

            if (topgate_valid && s->sp != topgate_last_sp) {
                /* Compte l'instruction PRÉCÉDENTE qui a changé SP, quelle
                 * que soit sa voie de sortie (early-exit, return, etc.) */
                sp_hist_account(topgate_last_pc, topgate_last_op,
                                topgate_last_sp, s->sp, s->insn_count);
            }
            topgate_last_sp = s->sp;
            topgate_last_pc = s->pc;
            topgate_last_op = prog_fetch(s, s->pc);
            topgate_valid   = 1;

            /* Patch 3 (2026-05-25) : raw ring buffer record.
             * Inconditionnel per-iter (pas de filtre Δ, pas de classification
             * signée). Env-gated CALYPSO_SP_RING=1, zéro coût si OFF.
             * Le ring est dumpé edge-triggered par sp_hist_account quand
             * SP croise sous le floor — fournit les ~512 dernières
             * (insn,PC,SP,op) AVANT le plongeon, donc la trajectoire
             * d'approche et l'instruction exacte qui plonge. */
            sp_ring_init_lazy();
            sp_ring_record(s->insn_count, s->pc, s->sp, topgate_last_op);
        }

        /* DSP idle fast-forward — see dsp_idle_fast_forward() comment.
         * Skips MAC simulation when DSP is in its empty-task-slot
         * polling loop, returning host CPU to the rest of QEMU. */
        {
            int ff_cyc;
            if (dsp_idle_fast_forward(s, &ff_cyc)) {
                s->cycles    += ff_cyc;
                s->insn_count += ff_cyc;
                executed     += ff_cyc;
                continue;
            }
        }

        /* CALYPSO_DSP_FBDET_SKIP — completion of the SYNTH strategy.
         * When the firmware enters the fb-det routine (PC range
         * 0x8d00..0x8f80) AND we already publish synthetic FB results
         * via CALYPSO_FBSB_SYNTH=1, executing the routine is pure
         * waste : the caller will read NDB, find the synth d_fb_det=1,
         * and ignore whatever the routine would have computed.
         *
         * Mechanism : at the FIRST PC entering the range from outside,
         * pop the return addr from stack (CALL convention) and jump to
         * it. Subsequent PCs inside the range fall back to normal exec
         * (we only short-circuit the entry point per call).
         *
         * Default OFF. Enable with CALYPSO_DSP_FBDET_SKIP=1 alongside
         * CALYPSO_FBSB_SYNTH=1 for B2B / demo runs where you accept the
         * shunt for performance. Silently no-op without FBSB_SYNTH so
         * the user can leave it set without affecting real-DSP runs.
         */
        {
            static int fbdet_skip_enabled = -1;
            static int fbsb_synth_active = -1;
            static uint16_t prev_pc_fbdet = 0;
            static uint64_t fbdet_skip_count = 0;

            if (fbdet_skip_enabled < 0) {
                const char *e1 = getenv("CALYPSO_DSP_FBDET_SKIP");
                fbdet_skip_enabled = (e1 && *e1 == '1') ? 1 : 0;
                const char *e2 = getenv("CALYPSO_FBSB_SYNTH");
                fbsb_synth_active = (e2 && *e2 == '1') ? 1 : 0;
                C54_LOG("DSP FBDET SKIP: %s%s",
                        fbdet_skip_enabled ? "enabled" : "disabled",
                        fbdet_skip_enabled && !fbsb_synth_active
                            ? " (but FBSB_SYNTH=0 — skip is no-op)" : "");
            }
            if (fbdet_skip_enabled && fbsb_synth_active &&
                s->pc >= 0x8d00 && s->pc < 0x8f80 &&
                (prev_pc_fbdet < 0x8d00 || prev_pc_fbdet >= 0x8f80)) {
                uint16_t ra = data_read(s, s->sp);
                s->sp = (uint16_t)(s->sp + 1);
                fbdet_skip_count++;
                if (fbdet_skip_count <= 5 || (fbdet_skip_count % 10000) == 0) {
                    C54_LOG("FBDET-SKIP #%llu entry_pc=0x%04x ra=0x%04x SP=0x%04x",
                            (unsigned long long)fbdet_skip_count,
                            s->pc, ra, s->sp);
                }
                s->pc = ra;
                prev_pc_fbdet = ra;
                s->cycles += 5;
                executed += 5;
                continue;
            }
            prev_pc_fbdet = s->pc;
        }

        /* Replay any interrupt that fired while INTM=1.
         * c54x_interrupt_ex sets IFR but does nothing else when INTM=1;
         * the real C54x re-evaluates pending interrupts every cycle, so
         * as soon as INTM clears (via RETE or RSBX INTM) a pending
         * BRINT0/TINT0/... must dispatch. Without this, a BRINT0 that
         * arrived inside another ISR is lost and the FB correlator never
         * receives its I/Q samples (d_fb_det stays 0). */
        if (!(s->st1 & ST1_INTM)) {
            uint16_t pending = s->ifr & s->imr;
            if (pending) {
                int imr_bit = __builtin_ctz(pending);
                int vec = imr_bit + 16;
                s->ifr &= ~(1 << imr_bit);
                s->sp--;
                data_write(s, s->sp, s->pc);
                if (s->pmst & PMST_APTS) {
                    s->sp--;
                    data_write(s, s->sp, s->xpc);
                }
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                static int pending_log = 0;
                if (pending_log < 20) {
                    C54_LOG("PENDING IRQ replay vec=%d bit=%d PC->0x%04x SP=0x%04x insn=%u",
                            vec, imr_bit, s->pc, s->sp, s->insn_count);
                    pending_log++;
                }
            }
        }

        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* Push counter at PC=0xb906 (and other suspected push sites).
         * Logs at powers of 10 to track cadence. SP captured at hit. */
        {
            static unsigned hit_b906 = 0;
            if (s->pc == 0xb906) {
                hit_b906++;
                if (hit_b906 == 1 || hit_b906 == 10 || hit_b906 == 100 ||
                    hit_b906 == 1000 || hit_b906 == 10000 ||
                    hit_b906 == 100000 || hit_b906 == 1000000) {
                    C54_LOG("HIT-b906 #%u op=0x%04x SP=0x%04x XPC=%d insn=%u",
                            hit_b906, s->prog[0xb906], s->sp, s->xpc,
                            s->insn_count);
                }
            }
        }

        /* INTM transition tracer: every change of ST1 bit 11 with
         * surrounding state. Identifies which IRQ entered the trap and
         * whether RETE / RSBX paths ever execute again. On each 0->1
         * (IRQ entry), also dump prog[PC..PC+8] and the 4 most-recently
         * pushed stack words (data[SP..SP+3]) so we can see what handler
         * we're entering and why it never RETEs.
         *
         * NOTE: this block runs BEFORE c54x_exec_one of the current
         * iteration. So when a transition is observed, the cause was
         * either (a) the previous iteration's exec_one (RETE, RSBX INTM
         * etc. — INTM 1→0), or (b) the pending-IRQ replay block above
         * (INTM 0→1, PC moved to vector). For (a), s->pc has already
         * advanced past the cause — log the previous iteration's
         * exec_pc/exec_op (captured at end of loop into last_exec_*) so
         * the cause is unambiguous. For (b), s->pc IS the vector entry
         * and is informative as-is. */
        {
            static int intm_log = 0;
            static uint16_t prev_intm = 0xFFFF;
            uint16_t cur_intm = !!(s->st1 & ST1_INTM);
            if (prev_intm != 0xFFFF && cur_intm != prev_intm && intm_log < 200) {
                C54_LOG("INTM-TRANS %u->%u current PC=0x%04x op=0x%04x | "
                        "cause prev_exec PC=0x%04x op=0x%04x | "
                        "XPC=%d IFR=0x%04x SP=0x%04x insn=%u",
                        (unsigned)prev_intm, (unsigned)cur_intm,
                        s->pc, s->prog[s->pc],
                        s->last_exec_pc, s->last_exec_op,
                        s->xpc, s->ifr, s->sp,
                        s->insn_count);
                if (cur_intm == 1) {
                    C54_LOG("  HANDLER prog[PC..PC+8]: %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                            s->prog[s->pc],
                            s->prog[(uint16_t)(s->pc + 1)],
                            s->prog[(uint16_t)(s->pc + 2)],
                            s->prog[(uint16_t)(s->pc + 3)],
                            s->prog[(uint16_t)(s->pc + 4)],
                            s->prog[(uint16_t)(s->pc + 5)],
                            s->prog[(uint16_t)(s->pc + 6)],
                            s->prog[(uint16_t)(s->pc + 7)],
                            s->prog[(uint16_t)(s->pc + 8)]);
                    C54_LOG("  STACK data[SP..SP+3]: %04x %04x %04x %04x",
                            s->data[s->sp],
                            s->data[(uint16_t)(s->sp + 1)],
                            s->data[(uint16_t)(s->sp + 2)],
                            s->data[(uint16_t)(s->sp + 3)]);
                }
                intm_log++;
            }
            prev_intm = cur_intm;
        }

        /* SP-WATCH: log every transition where SP enters / leaves the
         * API mailbox region [0x0800..0x08FF]. This pinpoints the exact
         * instruction that corrupts the stack pointer so we don't have
         * to keep recoding to investigate. */
        {
            static uint16_t prev_sp = 0xFFFF;
            bool was_in = (prev_sp >= 0x0800 && prev_sp < 0x0900);
            bool is_in  = (s->sp  >= 0x0800 && s->sp  < 0x0900);
            if (was_in != is_in) {
                fprintf(stderr,
                        "[c54x] SP-WATCH %s SP=0x%04x (prev=0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        is_in ? "ENTER api" : "LEAVE api",
                        s->sp, prev_sp, s->pc, s->prog[s->pc], s->insn_count);
            }
            prev_sp = s->sp;
        }

        /* TRACE: dump entry into 0xe260 loop (first 5 hits) */
        if (s->pc == 0xe260 || s->pc == 0xe261) {
            static int e260_log = 0;
            if (e260_log < 5) {
                e260_log++;
                C54_LOG("E260-ENTRY #%d PC=0x%04x AR2=%04x AR5=%04x BRC=%d RSA=%04x REA=%04x rptb=%d IMR=%04x SP=%04x insn=%u",
                        e260_log, s->pc, s->ar[2], s->ar[5], s->brc, s->rsa, s->rea, s->rptb_active, s->imr, s->sp, s->insn_count);
                int idx = pc_ring_idx;
                char buf[1024]; int o = 0;
                for (int i = 50; i >= 1; i--) {
                    o += snprintf(buf+o, sizeof(buf)-o, "%04x ", pc_ring[(idx-i)&255]);
                }
                C54_LOG("E260-PCRING (last 50): %s", buf);
                /* Dump runtime opcodes 0xe255..0xe28f */
                char ob[1024]; int oo = 0;
                for (uint16_t a = 0xe255; a <= 0xe28f; a++) {
                    oo += snprintf(ob+oo, sizeof(ob)-oo, "%04x ", s->prog[a]);
                }
                C54_LOG("E260-PROG[e255..e28f]: %s", ob);
            }
        }

        /* CALA loop tracer: dump A and SP at PC=0xd24e and 0xd250 (first 40) */
        if (s->pc == 0xd24e || s->pc == 0xd250) {
            static int cala_log = 0;
            if (cala_log++ < 40) {
                C54_LOG("CALA-TRACE PC=0x%04x A=%08x SP=0x%04x BRC=%d AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                        s->pc, (uint32_t)(s->a & 0xFFFFFFFF), s->sp, s->brc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            }
        }

        /* PC histogram: count visits per PC, dump top 20 every 2M insns */
        {
            static uint32_t pc_hist[0x10000];
            static uint64_t hist_last_dump = 0;
            pc_hist[s->pc]++;
            if (s->insn_count - hist_last_dump >= 2000000) {
                hist_last_dump = s->insn_count;
                /* find top 20 */
                uint32_t top_cnt[20] = {0};
                uint16_t top_pc[20] = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_hist[i];
                    if (c == 0) continue;
                    for (int j = 0; j < 20; j++) {
                        if (c > top_cnt[j]) {
                            for (int k = 19; k > j; k--) {
                                top_cnt[k] = top_cnt[k-1];
                                top_pc[k] = top_pc[k-1];
                            }
                            top_cnt[j] = c;
                            top_pc[j] = (uint16_t)i;
                            break;
                        }
                    }
                }
                C54_LOG("PC HIST insn=%u top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        s->insn_count,
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1], top_pc[2], top_cnt[2],
                        top_pc[3], top_cnt[3], top_pc[4], top_cnt[4], top_pc[5], top_cnt[5],
                        top_pc[6], top_cnt[6], top_pc[7], top_cnt[7], top_pc[8], top_cnt[8],
                        top_pc[9], top_cnt[9]);
                C54_LOG("PC HIST cont:        %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[10], top_cnt[10], top_pc[11], top_cnt[11], top_pc[12], top_cnt[12],
                        top_pc[13], top_cnt[13], top_pc[14], top_cnt[14], top_pc[15], top_cnt[15],
                        top_pc[16], top_cnt[16], top_pc[17], top_cnt[17], top_pc[18], top_cnt[18],
                        top_pc[19], top_cnt[19]);
                memset(pc_hist, 0, sizeof(pc_hist));
            }
        }

        /* === Rolling PC sampler (v6 — find the REAL stuck zone) ===
         * The cumulative-since-boot PC HIST shows 0xa218..0xa222 dominant
         * because the init loop at 0xa222 (BANZD AR5, 60k iters) ran once
         * early. After that, the DSP moved on but the cumulative histogram
         * still shows those PCs at the top.
         *
         * BANZD-A222 traces (2026-05-08) confirmed AR5 was the actual loop
         * counter (61523→61499 in 25 iter), not AR1. Loop finishes in
         * ~984k insns (= 0.06% of a 1.7B run). Whatever IS currently
         * burning DSP cycles is in a different zone, invisible to the
         * cumulative top-N.
         *
         * Solution : rolling histogram per 100k-insn window. Resets each
         * window so we always see "what is the DSP doing RIGHT NOW".
         * Logs top-5 PCs of the most recent window. */
        {
            static uint32_t pc_recent[0x10000];
            static uint32_t recent_last_dump = 0;
            pc_recent[s->pc]++;
            if (s->insn_count - recent_last_dump >= 100000) {
                recent_last_dump = s->insn_count;
                uint32_t top_cnt[5] = {0};
                uint16_t top_pc[5]  = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_recent[i];
                    if (c <= top_cnt[4]) continue;
                    top_cnt[4] = c; top_pc[4] = (uint16_t)i;
                    for (int j = 4; j > 0 && top_cnt[j] > top_cnt[j-1]; j--) {
                        uint32_t tc = top_cnt[j]; top_cnt[j] = top_cnt[j-1]; top_cnt[j-1] = tc;
                        uint16_t tp = top_pc[j]; top_pc[j] = top_pc[j-1]; top_pc[j-1] = tp;
                    }
                }
                C54_LOG("PC RECENT (last 100k) top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1],
                        top_pc[2], top_cnt[2], top_pc[3], top_cnt[3],
                        top_pc[4], top_cnt[4]);
                memset(pc_recent, 0, sizeof(pc_recent));
            }
        }

        /* === ENTER-RPTB-A218 probe (Q-BRC investigation 2026-05-08 v5+v6) ===
         * v5 hypothesis (BRC≈30770) was REFUTED by first 20 events :
         *   BRC=0 systematic, AR1=0 systematic, AR2 increments by 2,
         *   16 insns between visits.
         * v6 expands to capture the late-run behaviour : the cap=20 saturated
         * at insn=48M while the run reached 2.4B. We now have :
         *   (a) cap=200 for early events
         *   (b) periodic sampler at 100k-visits intervals (late-run)
         *   (c) BANZD-A222 probe to capture the actual AR used by the
         *       branch-back instruction at 0xa222 op=0x6e81.
         * The !s->rpt_active guard avoids spurious mid-RPTB hits. */
        if (s->pc == 0xa218 && !s->rpt_active) {
            static unsigned a218_total = 0;
            static int a218_log = 0;
            a218_total++;
            bool log_now = (a218_log < 200) ||
                           (a218_total % 100000 == 0);
            if (log_now) {
                C54_LOG("ENTER-RPTB-A218 #%d total=%u BRC=%u (0x%04x) "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x A=%010llx T=0x%04x "
                        "ST0=0x%04x insn=%u",
                        a218_log + 1, a218_total, s->brc, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->t, s->st0, s->insn_count);
                a218_log++;
            }
        }
        /* === BANZD-A222 probe (v6) ===
         * 0xa222 op=0x6e81 + opnd 0x8208 = `BANZD pmad, *Sind`.
         * The *Sind operand decodes some AR but my v5 guess (AR1) was
         * unverified — capture all ARs so we see which one is non-zero
         * and how it evolves. If AR1=0 systematically, the branch test
         * uses a different AR. Cap=200, plus periodic 100k. */
        if (s->pc == 0xa222 && !s->rpt_active) {
            static unsigned a222_total = 0;
            static int a222_log = 0;
            a222_total++;
            bool log_now = (a222_log < 200) ||
                           (a222_total % 100000 == 0);
            if (log_now) {
                C54_LOG("BANZD-A222 #%d total=%u op=0x%04x op2=0x%04x "
                        "AR0=0x%04x AR1=0x%04x AR2=0x%04x AR3=0x%04x "
                        "AR4=0x%04x AR5=0x%04x AR6=0x%04x AR7=0x%04x "
                        "BRC=%u insn=%u",
                        a222_log + 1, a222_total,
                        s->prog[s->pc], s->prog[(uint16_t)(s->pc + 1)],
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                a222_log++;
            }
        }
        /* Companion probe at 0xa215 (BRC setup) and 0xa217 (outer entry).
         * 0xa215 op=0x4492 + 0xa216 opnd 0x0092 = `ADD/SUB Smem,16,dst` per
         * tic54x (2-word, mask FE00 base 0x4400). Logs A_pre / A_post and
         * the Smem read so we can trace what value lands in dst (may feed
         * BRC eventually). 30-event cap. */
        if (s->pc == 0xa215 || s->pc == 0xa217) {
            static int brc_setup_215 = 0;
            static int brc_setup_217 = 0;
            int *cnt = (s->pc == 0xa215) ? &brc_setup_215 : &brc_setup_217;
            if (*cnt < 30) {
                C54_LOG("ENTER-A%04x #%d AR0=%04x AR1=%04x AR2=%04x "
                        "A=%010llx B=%010llx T=%04x BRC=%u DP=0x%03x insn=%u",
                        s->pc, *cnt + 1,
                        s->ar[0], s->ar[1], s->ar[2],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t, s->brc, (s->st0 & 0x1FF), s->insn_count);
                (*cnt)++;
            }
        }

        /* === XC-COND probe at PC=0xa0e0 / 0xa0e4 (Q1 hypothesis test) ===
         * Per Claude web v3 diag (2026-05-08) : routine 0xa0e0..0xa0e9 ends
         * at PC=0xa0e7 op=0xc8be where AR4 is consistently 0x18 (=MMR_SP)
         * pre-instruction → ST||LD writes to SP, catastrophe.
         *
         * Static dump shows two `XC 1, cond` instructions before 0xc8be :
         *   0xa0e0 = 0xfd30  ; XC 1, cond=0x30 (TC)
         *   0xa0e4 = 0xfd43  ; XC 1, cond=0x43 (ALT, A<0)
         *
         * Hypothesis : if XC condition evaluates to FALSE (TC bit not set, or
         * A not negative), the conditional STM #lk, AR4 (likely at 0xa0e5) is
         * SKIPPED → AR4 keeps stale value of 0x18 from earlier code path.
         *
         * Log every visit with : cond byte, TC/A/B flag values, AR4 value,
         * and the next opcode (which would be skipped or executed). If the
         * "taken" decision is consistently false at one of these XCs, that's
         * the bug. Cap to 100 events per PC. */
        if (s->pc == 0xa0e0 || s->pc == 0xa0e4) {
            static unsigned xc_log_e0;
            static unsigned xc_log_e4;
            unsigned *cnt = (s->pc == 0xa0e0) ? &xc_log_e0 : &xc_log_e4;
            if (*cnt < 100) {
                uint16_t op_xc = s->prog[s->pc];
                uint8_t  cond_byte = op_xc & 0xFF;
                uint16_t next_op   = s->prog[(uint16_t)(s->pc + 1)];
                /* Mirror the condition decode from c54x_exec_one (case 0xF
                 * XC handler around line 1108+) — only the common subset. */
                bool cond = false;
                if      (cond_byte == 0x00) cond = true;
                else if (cond_byte == 0x0C) cond = (s->st0 & ST0_C) != 0;
                else if (cond_byte == 0x08) cond = !(s->st0 & ST0_C);
                else if (cond_byte == 0x30) cond = (s->st0 & ST0_TC) != 0;
                else if (cond_byte == 0x20) cond = !(s->st0 & ST0_TC);
                else if (cond_byte == 0x45) cond = (sext40(s->a) == 0);
                else if (cond_byte == 0x44) cond = (sext40(s->a) != 0);
                else if (cond_byte == 0x46) cond = (sext40(s->a) > 0);
                else if (cond_byte == 0x42) cond = (sext40(s->a) >= 0);
                else if (cond_byte == 0x43) cond = (sext40(s->a) < 0);
                else if (cond_byte == 0x47) cond = (sext40(s->a) <= 0);
                else if (cond_byte == 0x4D) cond = (sext40(s->b) == 0);
                else if (cond_byte == 0x4C) cond = (sext40(s->b) != 0);
                else if (cond_byte == 0x4E) cond = (sext40(s->b) > 0);
                else if (cond_byte == 0x4A) cond = (sext40(s->b) >= 0);
                else if (cond_byte == 0x4B) cond = (sext40(s->b) < 0);
                else if (cond_byte == 0x4F) cond = (sext40(s->b) <= 0);
                fprintf(stderr,
                        "[c54x] XC-COND #%u PC=0x%04x op=0x%04x cond=0x%02x "
                        "→ %s | TC=%d C=%d A=%010llx (sgn:%c) "
                        "B=%010llx (sgn:%c) AR4=0x%04x next_op=0x%04x insn=%u\n",
                        *cnt + 1, s->pc, op_xc, cond_byte,
                        cond ? "TAKEN " : "SKIPPED",
                        !!(s->st0 & ST0_TC),
                        !!(s->st0 & ST0_C),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        sext40(s->a) < 0 ? '-' : (sext40(s->a) == 0 ? '0' : '+'),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        sext40(s->b) < 0 ? '-' : (sext40(s->b) == 0 ? '0' : '+'),
                        s->ar[4], next_op, s->insn_count);
                (*cnt)++;
            }
        }

        /* === MAC-8d33 trace — FB-det inner correlator ===
         * The DSP loops indefinitely in 0x8d2d..0x8d36. Static dump shows :
         *   8d2d 0x771a 0x0004      ; (2-word) — likely setup
         *   8d2f 0xf072 0x8d33      ; RPTB pmad, end=0x8d33 (per tic54x)
         *   8d31 0xf461             ; F46x = SFTA src,shift,dst (1-word)
         *   8d32 0xf591             ; F591 = ROL B (per our decoder)
         *   8d33 0xf3e2             ; F3E0-F3FF = SFTL src,SHIFT,DST  ← writes a_sync_SNR
         *   8d34 0x6e89 0x8d2d      ; BANZD pmad=0x8d2d, *AR — outer back-branch
         *   8d36 0xf3e1             ; SFTL B,1,B (exit path)
         * PC HIST counts (105k outer / 526k inner = 5×) confirm the 5-iter
         * RPTB body is (0x8d32, 0x8d33, 0x8d34) repeated 5 times.
         *
         * Capture A_pre, T, AR2..AR5 at each PC inside this zone. Rate-limit :
         *   first 50 always (init + early convergence)
         *   every 5000th (steady-state cadence)
         *   when |A_after - last_logged_A| > 0x100000 (significant accumulator
         *   shift = convergence event worth dumping)
         * Plus a dedicated "ENTER 0x8d2d" outer-iter counter that always logs
         * A_pre at the OUTER entry, so we can tell whether the accumulator
         * is reset between FB-det attempts (Observation 1 from session diag). */
        if (s->pc >= 0x8d2c && s->pc <= 0x8d3a) {
            static uint64_t mac8d_count;
            static int64_t  last_logged_a;
            int64_t a_now = sext40(s->a);
            int64_t da = a_now - last_logged_a;
            if (da < 0) da = -da;
            mac8d_count++;
            bool log_now = (mac8d_count <= 50) ||
                           (mac8d_count % 5000) == 0 ||
                           da > 0x100000LL;
            if (log_now) {
                C54_LOG("MAC-8d33 #%llu PC=0x%04x op=0x%04x A_pre=%010llx B=%010llx "
                        "T=0x%04x ARs: %04x %04x %04x %04x %04x %04x BRC=%d insn=%u",
                        (unsigned long long)mac8d_count,
                        s->pc, s->prog[s->pc],
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->brc, s->insn_count);
                last_logged_a = a_now;
            }
        }
        /* Dedicated outer-entry tracer at PC=0x8d2d : ALWAYS log A_pre on
         * entry (cap to 200 events). If A is non-zero on outer entry,
         * the accumulator wasn't reset between attempts — observation 1
         * from 2026-05-08 session : 21× 0x2fb0 SNR could mean stuck
         * accumulator across attempts. */
        if (s->pc == 0x8d2d) {
            static uint64_t enter_8d2d;
            enter_8d2d++;
            if (enter_8d2d <= 200) {
                C54_LOG("ENTER-8d2d #%llu A_pre=%010llx B_pre=%010llx T=0x%04x "
                        "ARs: %04x %04x %04x %04x %04x %04x %04x %04x SP=0x%04x BRC=%d insn=%u",
                        (unsigned long long)enter_8d2d,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->t,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->sp, s->brc, s->insn_count);
            }
        }

        /* === HOT-OPS PROBE for 0xe9ac..0xe9b7 + 0xe981..0xe983 ===
         * Diag v2 2026-05-08 : DSP locked in deterministic 7-instruction
         * loop at 0xe9ac..0xe9b7 (PROM1 mirror), with outer 3-PC loop
         * 0xe981..0xe983 reloading a BRC counter — pattern consistent
         * with `RPTB end_addr` + outer reset. We need the actual opcodes
         * to confirm/refute the RPTB hypothesis. One-shot dump on first
         * entry into the body range, with surrounding context (a few
         * words before for the RPTB instruction itself, and the outer). */
        {
            static bool e9ac_dumped = false;
            if (!e9ac_dumped && s->pc >= 0xe9ac && s->pc <= 0xe9b7) {
                e9ac_dumped = true;
                fprintf(stderr,
                        "[c54x] HOT-OPS-DUMP triggered at PC=0x%04x insn=%u\n",
                        s->pc, s->insn_count);
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe9a0..0xe9bf]:");
                for (uint16_t a = 0xe9a0; a <= 0xe9bf; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS prog[0xe97c..0xe98f] (outer):");
                for (uint16_t a = 0xe97c; a <= 0xe98f; a++)
                    fprintf(stderr, " %04x", s->prog[a]);
                fprintf(stderr, "\n");
                fprintf(stderr,
                        "[c54x] HOT-OPS state: BRC=%d RSA=0x%04x REA=0x%04x "
                        "rptb_active=%d ST1=0x%04x AR0..7: %04x %04x %04x %04x "
                        "%04x %04x %04x %04x\n",
                        s->brc, s->rsa, s->rea, s->rptb_active, s->st1,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;

        /* Trace EB04 loop — dump first 20 iterations */
        if (s->pc == 0xEB04) {
            static int eb04_log = 0;
            if (eb04_log < 20) {
                C54_LOG("EB04 op=%04x A=0x%010llx B=0x%010llx T=%04x "
                        "INTM=%d IMR=%04x IFR=%04x rptb=%d RSA=%04x REA=%04x BRC=%d "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->rptb_active, s->rsa, s->rea, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                eb04_log++;
            }
        }

        /* Dump DSP state when stuck — triggers once after 500M instructions
         * if DSP hasn't reached IDLE yet */
        {
            static int dumped = 0;
            if (s->insn_count > 500000000 && !dumped && !s->idle) {
                dumped = 1;
                C54_LOG("DSP NO-IDLE dump at insn=%u PC=0x%04x:", s->insn_count, s->pc);
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  IMR=0x%04x IFR=0x%04x rptb=%d RSA=0x%04x REA=0x%04x BRC=%d",
                        s->imr, s->ifr, s->rptb_active, s->rsa, s->rea, s->brc);
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x XPC=%d",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t, s->xpc);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                /* Dump code around current PC (using prog_fetch for correct OVLY) */
                C54_LOG("  Code around PC:");
                for (int i = -4; i < 16; i++) {
                    uint16_t a = s->pc + i;
                    C54_LOG("  %c [0x%04x] = 0x%04x",
                            i == 0 ? '>' : ' ', a, prog_fetch(s, a));
                }
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* BSP read entry points — these functions contain PORTR PA=0xF430
         * (read BSP sample). If DSP never visits them, the FB-det chain is
         * dead. Targets identified by static analysis of PROM0 callers of
         * the 64 PORTR PA=0xF430 sites at 0x9b80+. */
        if (!s->rpt_active &&
            (s->pc == 0x9a78 || s->pc == 0x9aaf || s->pc == 0x9ad3 ||
             s->pc == 0x9b4c || s->pc == 0x8811)) {
            static unsigned bsp_visits[5];
            int idx = (s->pc == 0x9a78) ? 0 :
                      (s->pc == 0x9aaf) ? 1 :
                      (s->pc == 0x9ad3) ? 2 :
                      (s->pc == 0x9b4c) ? 3 : 4;
            if (bsp_visits[idx] < 5) {
                bsp_visits[idx]++;
                C54_LOG("BSP-ENTRY PC=0x%04x  A=0x%010llx ar0=%04x ar1=%04x "
                        "ar2=%04x ar3=%04x ar4=%04x SP=0x%04x insn=%u",
                        s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4],
                        s->sp, s->insn_count);
            }
        }

        /* Trace any write touching the dispatcher poll addresses
         * data[0x4359] / data[0x3fab]. We never see them go non-zero;
         * confirm whether ANY code path writes them. */
        /* (handled in data_write — see below) */

        /* Dispatcher hot loop trace at PROM0 0xb968-0xb9a4 — the state
         * machine the DSP spins in when waiting for ARM tasks. Logs the
         * first 8 visits per PC so we see the full conditional structure
         * (which addresses it polls, which constants it compares to). */
        if (s->pc >= 0xb968 && s->pc <= 0xb9a4 && !s->rpt_active) {
            static uint8_t disp_visits[64];
            int idx = s->pc - 0xb968;
            if (idx >= 0 && idx < 64 && disp_visits[idx] < 8) {
                disp_visits[idx]++;
                C54_LOG("DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx "
                        "B=0x%010llx ar0=%04x ar1=%04x ar2=%04x ar3=%04x "
                        "ar4=%04x ar5=%04x TC=%d",
                        s->pc, prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        !!(s->st0 & ST0_TC));
            }
        }

        /* IRQ vec area trace: log every PC visit in 0xFFCC-0xFFE0
         * (INT3 + TINT0 + BRINT0 vec slots). Captures the 3 actual
         * 4-word handlers our IRQ INT3 dispatch lands on at IPTR=0x1ff.
         * 80 unique PCs max, log first 4 visits each. */
        if (s->pc >= 0xFFCC && s->pc < 0xFFE0 && !s->rpt_active) {
            static uint8_t vec_visits[20];   /* index 0 = 0xffcc */
            int idx = s->pc - 0xFFCC;
            if (vec_visits[idx] < 4) {
                vec_visits[idx]++;
                C54_LOG("VEC-TRACE PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx "
                        "B=0x%010llx TC=%d INTM=%d ar7=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        !!(s->st0 & ST0_TC),
                        !!(s->st1 & ST1_INTM),
                        s->ar[7]);
            }
        }

        /* Trace DSP init - log once per unique PC in E900-E960 */
        if (s->pc >= 0xE900 && s->pc < 0xE960 && !s->rpt_active) {
            static uint16_t seen_pcs[96];
            int idx = s->pc - 0xE900;
            if (!seen_pcs[idx]) {
                seen_pcs[idx] = 1;
                C54_LOG("INIT PC=0x%04x op=0x%04x SP=0x%04x BRC=%d rptb=%d RSA=0x%04x REA=0x%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, s->brc,
                        s->rptb_active, s->rsa, s->rea);
            }
        }

        /* Trace SINT17 handler (0x8a00-0x8a5f) */
        if (s->pc >= 0x8a00 && s->pc < 0x8a60) {
            static int sint17_log = 0;
            if (sint17_log < 500) {
                C54_LOG("SINT17 PC=0x%04x op=0x%04x SP=0x%04x DP=0x%03x A=0x%010llx B=0x%010llx AR0=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, dp(s),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[0]);
                sint17_log++;
            }
        }

        /* Sample PC every 1M instructions to find stuck loops */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (run_num <= 2 && executed < 2000) {
            C54_LOG("BOOT[%d.%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx B=0x%010llx",
                    run_num, executed, s->pc, prog_fetch(s, s->pc), s->sp,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        }
        /* RPTB check moved below — must run AFTER `s->pc += consumed` so
         * that when the body's last instruction has executed and PC has
         * advanced to REA+1, the redirect to RSA is the FINAL operation
         * on PC for this iteration. The previous placement (before PC
         * advance) caused a 1-instruction off-by-one : redirect set
         * pc=RSA, then `s->pc += consumed` bumped it to RSA+1, so the
         * first body instruction was never re-executed across iterations
         * (PC HIST showed body=[RSA+1..REA+1] instead of [RSA..REA]). */

        /* Trace the IMR loop: how does the DSP reach 0x03F0? */
        /* Trace RPTB entry at 0x76FD: dump all AR values */
        if (s->pc == 0x76FD) {
            static int rptb_entry_log = 0;
            if (rptb_entry_log < 30)
                C54_LOG("RPTB-ENTRY PC=0x76FD AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x ARP=%d DP=%d BRC=%d SP=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        arp(s), dp(s), s->brc, s->sp);
            rptb_entry_log++;
        }
        if (s->pc == 0x03F0) {
            static int f3_log = 0;
            if (f3_log < 2) {
                C54_LOG("PC=0x03F0 op=0x%04x insn=%u SP=0x%04x IMR=0x%04x XPC=%d PMST=0x%04x",
                        prog_fetch(s, s->pc), s->insn_count, s->sp, s->imr, s->xpc, s->pmst);
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-20)&255], pc_ring[(pc_ring_idx-19)&255],
                        pc_ring[(pc_ring_idx-18)&255], pc_ring[(pc_ring_idx-17)&255],
                        pc_ring[(pc_ring_idx-16)&255], pc_ring[(pc_ring_idx-15)&255],
                        pc_ring[(pc_ring_idx-14)&255], pc_ring[(pc_ring_idx-13)&255],
                        pc_ring[(pc_ring_idx-12)&255], pc_ring[(pc_ring_idx-11)&255],
                        pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                        pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                        pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                        pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                        pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                f3_log++;
            }
        }

        /* Boot trace */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute instruction */
        int consumed;
        uint16_t exec_pc = s->pc;
        uint16_t exec_op = prog_fetch(s, s->pc);
        consumed = c54x_exec_one(s);

        /* Detect SP changes — only log after init (insn > 490M) */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* v2 SP observability — only when CALYPSO_TRAP_OOR=1.
         * (a) sp_trail[256] : |Δ|>32 events (scheduler reloads + big allocs)
         * (b) sp_low watermark : every new low, PC-coalesced power-of-10
         * Gated to insn>33754 (after init stack 0x9022→0x5ac8 normal). */
        {
            static int trap_armed = -1;
            if (trap_armed < 0) {
                const char *e = getenv("CALYPSO_TRAP_OOR");
                trap_armed = (e && *e == '1') ? 1 : 0;
            }
            if (trap_armed && s->sp != sp_before && s->insn_count > 33754) {
                int16_t delta = (int16_t)(s->sp - sp_before);
                uint16_t a_low = (uint16_t)(s->a & 0xFFFF);

                /* SP-HIST per-PC accounting déplacé en TOP-of-loop chokepoint
                 * (fix v6 2026-05-24) — bypass-proof. Voir L6773.
                 * Pas d'appel ici sinon double-count. */

                /* (a) trail — only big jumps (skip push/pop ±1..32 noise) */
                if (delta > 32 || delta < -32) {
                    unsigned k = g_sp_trail_idx & 255;
                    g_sp_trail[k].insn    = s->insn_count;
                    g_sp_trail[k].old_sp  = sp_before;
                    g_sp_trail[k].new_sp  = s->sp;
                    g_sp_trail[k].exec_pc = exec_pc;
                    g_sp_trail[k].exec_op = exec_op;
                    g_sp_trail[k].a_low   = a_low;
                    g_sp_trail_idx++;
                }

                /* (b) sp_low watermark — fires on any new low (incl Δ=-1). */
                if (s->sp < g_sp_low) {
                    g_sp_low = s->sp;
                    if (exec_pc == g_sp_low_pc) {
                        g_sp_low_hits_at_pc++;
                        unsigned n = g_sp_low_hits_at_pc;
                        bool milestone = (n == 1 || n == 10 || n == 100 ||
                                          n == 1000 || n == 10000 || n == 100000);
                        if (milestone) {
                            fprintf(stderr,
                                "[c54x] SP-LOW #%u @pc=0x%04x op=0x%04x "
                                "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                                n, exec_pc, exec_op,
                                sp_before, s->sp, a_low, s->insn_count);
                        }
                    } else {
                        g_sp_low_pc = exec_pc;
                        g_sp_low_hits_at_pc = 1;
                        g_sp_low_distinct_pcs++;
                        fprintf(stderr,
                            "[c54x] SP-LOW NEW (#%u distinct) @pc=0x%04x op=0x%04x "
                            "sp 0x%04x->0x%04x A_low=0x%04x insn=%u\n",
                            g_sp_low_distinct_pcs, exec_pc, exec_op,
                            sp_before, s->sp, a_low, s->insn_count);
                    }
                }
            }
        }

        /* SP-LEDGER + SP-INTO-MMR probes RETIRÉS 2026-05-23 :
         * info diagnostic déjà extraite (irq_entries=1 sur 144s, SP wrap
         * via stack-relative writes en MMR). Ces probes fire à CHAQUE
         * instruction → overhead non-négligeable sur DSP throughput
         * (mesuré 9.1M insn/s vs 10M required = 9% slow). Reviendront en
         * cas de régression. SP-CATASTROPHE garde la haut |Δ|>256. */

        /* === SP catastrophic delta tracer ===
         * Diag v2 2026-05-08 : SP went from 0x9c1e → 0x0001 in one window
         * (lost ~40k stack words). The progressive-leak log above caps at
         * 100 small deltas and misses the single catastrophic event.
         * This block flags any |Δ| > 100 in one instruction — never
         * capped — so the buggy STM/PSHM/POPM/RETE-corrupted-stack /
         * FRAME-with-huge-offset is unambiguously identified the FIRST
         * time it happens. ARs included so we can see if the ST/LD
         * destination resolved to an MMR slot (e.g. *AR=0x18 → MMR_SP).
         *
         * Threshold raised from 100→256 on 2026-05-08 to filter legitimate
         * FRAME #imm8s (signed 8-bit can be ±127). Real catastrophes from
         * dual-op writing to MMR_SP are always thousands of words. */
        {
            int32_t dsp = (int32_t)(int16_t)(s->sp - sp_before);
            if (dsp > 256 || dsp < -256) {
                fprintf(stderr,
                        "[c54x] SP-CATASTROPHE Δ=%+d PC=0x%04x op=0x%04x "
                        "SP 0x%04x → 0x%04x INTM=%d "
                        "AR0..7: %04x %04x %04x %04x %04x %04x %04x %04x "
                        "BK=%04x A=%010llx insn=%u\n",
                        (int)dsp, exec_pc, exec_op, sp_before, s->sp,
                        !!(s->st1 & ST1_INTM),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        s->bk,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->insn_count);
            }
        }
        /* === v2 TRAP-OOR firing point — fixed checkpoint halt ===
         * T1/T2 dropped (v1 v2 redesign: scheduler at 0xfd2a exonerated,
         * SP clobber lives in legit code → no PC whitelist nor SP edge
         * can catch it). Halt at fixed insn checkpoint, dump trail+sp_low
         * for offline analysis of full descent.
         * Checkpoint configurable via CALYPSO_TRAP_CHECKPOINT (default
         * 4200000 = just after the insn=4.09M SP recovery 0x0008→0x2900). */
        {
            static int trap_armed = -1;
            static int tripped = 0;
            static unsigned checkpoint = 0;
            if (trap_armed < 0) {
                const char *e = getenv("CALYPSO_TRAP_OOR");
                trap_armed = (e && *e == '1') ? 1 : 0;
                const char *c = getenv("CALYPSO_TRAP_CHECKPOINT");
                checkpoint = (c && *c) ? (unsigned)strtoul(c, NULL, 0) : 4200000u;
            }
            if (trap_armed && !tripped && s->insn_count >= checkpoint) {
                tripped = 1;
                dsp_trap_dump(s, exec_pc, exec_op, sp_before, "CHECKPOINT");
                s->running = 0;
            }
        }

        /* === DUAL-OP-INTERPRET diagnostic ===
         * Compare current decoder's AR field interpretation (3-bit fields)
         * with SPRU172C's dual-operand encoding (2-bit AR fields + offset 2,
         * AR2..AR5 only). If the two disagree on which AR is used and the
         * SP-CATASTROPHE just fired, we have evidence the encoding is
         * wrong. Cap to 100 entries to avoid log explosion. */
        if ((exec_op & 0xFC00) == 0xC800 && (
             (int32_t)(int16_t)(s->sp - sp_before) > 100 ||
             (int32_t)(int16_t)(s->sp - sp_before) < -100)) {
            static unsigned dop_log;
            if (dop_log++ < 100) {
                int xar_cur = (exec_op >> 4) & 0x07;
                int yar_cur = exec_op & 0x07;
                int xar_spru = ((exec_op >> 4) & 0x03) + 2;
                int yar_spru = (exec_op & 0x03) + 2;
                int xmod_spru = (exec_op >> 6) & 0x03;
                int ymod_spru = (exec_op >> 2) & 0x03;
                fprintf(stderr,
                        "[c54x] DUAL-OP-INTERPRET op=0x%04x PC=0x%04x : "
                        "current_dec X=AR%d Y=AR%d (3bit) | "
                        "SPRU172C    X=AR%d Y=AR%d xmod=%d ymod=%d (2bit+2) | "
                        "AR%d_cur=%04x AR%d_spru=%04x | "
                        "AR%d_cur=%04x AR%d_spru=%04x\n",
                        exec_op, exec_pc,
                        xar_cur, yar_cur,
                        xar_spru, yar_spru, xmod_spru, ymod_spru,
                        xar_cur, s->ar[xar_cur],
                        xar_spru, s->ar[xar_spru],
                        yar_cur, s->ar[yar_cur],
                        yar_spru, s->ar[yar_spru]);
            }
        }

        /* Snapshot the just-executed PC/op into C54xState so other
         * tracers (in particular INTM-TRANS at top of next iteration)
         * can attribute post-instruction state changes to the cause. */
        s->last_exec_pc = exec_pc;
        s->last_exec_op = exec_op;

        /* RPT: after executing an instruction while repeat is active,
         * re-execute the SAME instruction (don't advance PC) until count=0. */
        if (s->rpt_active && !s->idle) {
            if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Don't advance PC — re-execute same instruction next cycle */
                s->cycles++;
                executed++;
                if (s->rpt_count == 0) {
                    static int rpt_done_log = 0;
                    if (rpt_done_log < 10)
                        C54_LOG("RPT DONE PC=0x%04x op=0x%04x count_was=%d", s->pc, prog_fetch(s, s->pc), 0);
                    rpt_done_log++;
                }
                continue;
            } else {
                s->rpt_active = false;
                s->par_set = false;
            }
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        /* Delayed-branch slot countdown.
         * RCD (and later CALLD/RETD/BD/CCD if extended) sets delayed_pc and
         * delay_slots = 2. The two instructions following the RCD execute
         * as normal pipeline slots; once both have completed the branch
         * commits by forcing PC to delayed_pc. */
        if (s->delay_slots > 0) {
            s->delay_slots--;
            if (s->delay_slots == 0) {
                s->pc = s->delayed_pc;
            }
        }

        /* === RPTB (block repeat) end-of-body check ===
         * Must run AFTER PC advance and delayed-branch settle so the
         * redirect to RSA is the final word on s->pc for this iteration.
         * Triggers when PC has overshot REA (= reached REA+1 or beyond,
         * accounting for 2-word instructions at the body's tail). Skip
         * during RPT (single-instruction repeat has priority). */
        if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) {
                C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d",
                        s->pc, s->rsa, s->rea, s->brc);
                rptb_log++;
            }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false;
                { static int _re=0;
                  if (_re<50) {
                    C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x",
                            s->pc, s->rsa, s->rea, s->insn_count, s->sp);
                    _re++;
                  }
                }
                s->st1 &= ~ST1_BRAF;
            }
        }

        s->cycles++;
        s->insn_count++;

        executed++;
    }
    return executed;
}

/* ================================================================
 * ROM loader
 * ================================================================ */

int c54x_load_rom(C54xState *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        C54_LOG("Cannot open ROM dump: %s", path);
        return -1;
    }

    char line[1024];
    int section = -1; /* 0=regs, 1=DROM, 2=PDROM, 3-6=PROM0-3 */
    
    int total_words = 0;

    while (fgets(line, sizeof(line), f)) {
        /* Section headers */
        if (strstr(line, "DSP dump: Registers"))  { section = 0; continue; }
        if (strstr(line, "DSP dump: DROM"))        { section = 1; continue; }
        if (strstr(line, "DSP dump: PDROM"))       { section = 2; continue; }
        if (strstr(line, "DSP dump: PROM0"))       { section = 3; continue; }
        if (strstr(line, "DSP dump: PROM1"))       { section = 4; continue; }
        if (strstr(line, "DSP dump: PROM2"))       { section = 5; continue; }
        if (strstr(line, "DSP dump: PROM3"))       { section = 6; continue; }
        if (section < 0) continue;

        /* Parse data lines: "ADDR : XXXX XXXX XXXX ..." */
        uint32_t addr;
        if (sscanf(line, "%x :", &addr) != 1) continue;

        char *p = strchr(line, ':');
        if (!p) continue;
        p++;

        uint16_t word;
        while (sscanf(p, " %hx%n", &word, (int[]){0}) == 1) {
            int n;
            sscanf(p, " %hx%n", &word, &n);
            p += n;

            if (section == 0) {
                /* Registers: store in data memory */
                if (addr < 0x60) s->data[addr] = word;
            } else if (section == 1 || section == 2) {
                /* DROM/PDROM: data memory */
                if (addr < C54X_DATA_SIZE) s->data[addr] = word;
            } else {
                /* PROM: program memory.
                 * The dump uses extended addresses (XPC pages):
                 *   PROM0: 0x07000-0x0DFFF → prog space 0x7000-0xDFFF
                 *   PROM1: 0x18000-0x1FFFF → prog space 0x8000-0xFFFF (page 1)
                 *   PROM2: 0x28000-0x2FFFF → prog space 0x8000-0xFFFF (page 2)
                 *   PROM3: 0x38000-0x39FFF → prog space 0xF800-0xFFFF (page 3)
                 * For 16-bit PC access, map all PROM to lower 64K too.
                 * PROM0 is already at 0x7000. For PROM1-3, also mirror
                 * to the 16-bit alias (0x8000-0xFFFF). */
                if (addr < C54X_PROG_SIZE) s->prog[addr] = word;
                /* Mirror PROM1 (page 1: 0x18000-0x1FFFF) to 16-bit space.
                 * PROM0 occupies 0x7000-0xDFFF — only mirror PROM1 above that
                 * (0xE000-0xFFFF) to avoid overwriting PROM0 data.
                 * This gives us interrupt vectors at 0xFF80. */
                if (section == 4) {  /* PROM1 only */
                    uint16_t addr16 = addr & 0xFFFF;
                    /* Mirror PROM1 to 0xE000-0xFF7F only.
                     * 0xFF80-0xFFFF is the interrupt vector table,
                     * populated by the DSP boot ROM (not PROM1). */
                    if (addr16 >= 0xE000)
                        s->prog[addr16] = word;
                }
            }
            addr++;
            total_words++;
        }
    }

    fclose(f);
    C54_LOG("Loaded ROM: %d words from %s", total_words, path);
    return 0;
}

/* ================================================================
 * Init / Reset / Interrupts
 * ================================================================ */

C54xState *c54x_init(void)
{
    C54xState *s = calloc(1, sizeof(C54xState));
    if (!s) return NULL;
    return s;
}

void c54x_set_api_ram(C54xState *s, uint16_t *api_ram)
{
    s->api_ram = api_ram;
}

void c54x_reset(C54xState *s)
{
    g_boot_trace = 50;
    s->a = 0; s->b = 0;
    memset(s->ar, 0, sizeof(s->ar));
    s->t = 0; s->trn = 0;
    s->sp = 0x5AC8; s->bk = 0;  /* SP init per Calypso boot ROM
                                 * NOTE: silicon dumps show SP=0x1100 post-handshake.
                                 * 0x5AC8 is a shortcut anticipating osmocom firmware re-init.
                                 * See doc/datasheets/README.md §3-4. */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    /* MMR reset values aligned with Calypso silicon (3 FreeCalypso ROM dumps + local).
     * Empirically validated 2026-04-28. See doc/datasheets/README.md §3.
     * Previous QEMU values (st0=0, st1=ST1_INTM, pmst=0xFFE0) were partial. */
    s->st0  = 0x181F;                              /* DP=0x01F per silicon */
    s->st1  = ST1_INTM | ST1_SXM | ST1_XF;         /* 0x2900: INTM=1, SXM=1, XF=1 */
    s->pmst = 0xFFA8;                              /* IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 */
    s->imr = 0;
    s->ifr = 0;
    s->xpc = 0;
    s->timer_psc = 0;
    s->data[TCR_ADDR] = TCR_TSS;  /* Timer stopped at reset (TSS=1) per HW spec */
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; { static int _re=0; if (_re<50) { C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp); _re++; } }
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1.
     *
     * On real silicon, DARAM and API RAM share one physical memory in the
     * range 0x0800-0x27FF (DSP-words). Mirror the copy into api_ram so the
     * ARM-side view matches the DSP-side view from boot — without this
     * mirror, every ARM read into the overlay zone returns 0 while the
     * DSP executes the copied code, which silently splits the two views. */
    for (int i = 0; i < 0x2780; i++) {
        uint16_t addr = 0x0080 + i;
        uint16_t val = s->prog[0x7080 + i];
        s->data[addr] = val;
        if (s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = val;
    }

    /* Install boot ROM interrupt vectors at 0xFF80 (IPTR=0x1FF).
     * These are from the Calypso internal boot ROM, not in the PROM dump.
     * Vec0 (reset): B 0xB410 (bootloader entry) */
    s->prog[0xFF80] = 0xF880;  /* B pmad */
    s->prog[0xFF81] = 0xB410;  /* target: bootloader */
    s->prog[0xFF82] = 0xF495;  /* NOP */
    s->prog[0xFF83] = 0xF495;  /* NOP */
    /* Vec1-7: use PROM1 ROM vectors (already mirrored to 0xFF84-0xFFFF).
     * Do NOT overwrite — the ROM contains the real interrupt handlers. */

    /* Boot ROM stubs at 0x0000-0x007F.
     * Discriminant test 2026-04-26 confirmed FRET stub did NOT block the
     * firmware path to 0x0810 (reverting to NOPs gave identical PC HIST
     * + same IMR change=0). FRET stub kept: prevents stack runaway when
     * CALAA targets the stub area, with no downside.
     *
     * Fallback per slot:
     *   - 0x0000: LDMM SP, B (real boot ROM behaviour)
     *   - 0x0001: RET (paired with the CALL at 0x770A)
     *   - rest:   FRET (0xF4E4) — return immediately to caller. */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF4E4;  /* FRET fallback — return-from-far */
    s->prog[0x0000] = 0xBA18;  /* LDMM SP, B */
    s->prog[0x0001] = 0xFC00;  /* RET */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);

}

void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    s->ifr |= (1 << imr_bit);

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;

    /* Per SPRU131: IDLE exits on ANY interrupt (masked or unmasked).
     * - Unmasked: branch to vector, set INTM=1
     * - Masked: just resume after IDLE, IFR bit stays set */
    if (s->idle) {
        s->idle = false;
        if (unmasked) {
            /* Service the interrupt: branch to vector */
            s->ifr &= ~(1 << imr_bit);
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 1));
            g_sp_ledger.irq_words_pushed++;
            if (s->pmst & PMST_APTS) {
                s->sp--;
                data_write(s, s->sp, s->xpc);
                g_sp_ledger.irq_words_pushed++;
            }
            g_sp_ledger.irq_entries++;
            s->st1 |= ST1_INTM;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            s->pc = (iptr * 0x80) + vec * 4;
        }
        /* If masked: just wake, advance PC past IDLE */
        if (!unmasked) {
            s->pc++;  /* resume at instruction after IDLE */
        }
    } else if (!(s->st1 & ST1_INTM) && unmasked) {
        /* Normal (non-IDLE) interrupt servicing */
        s->ifr &= ~(1 << imr_bit);
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        g_sp_ledger.irq_words_pushed++;
        if (s->pmst & PMST_APTS) {
            s->sp--;
            data_write(s, s->sp, s->xpc);
            g_sp_ledger.irq_words_pushed++;
        }
        g_sp_ledger.irq_entries++;
        s->st1 |= ST1_INTM;
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;
    }

    /* Log interrupts: first 20 + every 100th, so we can count them.
     * PMST/IPTR included so we can correlate which vector base the IRQ
     * lands at — INT3 at IPTR=0x1ff (vec=0xffcc) hits a garbage ROM stub,
     * INT3 at IPTR=0x140 (vec=0xa04c) hits the firmware's real handler. */
    static uint64_t int_log_count;
    int_log_count++;
    if (int_log_count <= 20 || (int_log_count % 100) == 0) {
        uint16_t iptr_now = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        C54_LOG("IRQ #%llu vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x "
                "idle=%d PC=0x%04x PMST=0x%04x IPTR=0x%03x",
                (unsigned long long)int_log_count,
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                s->idle, s->pc, s->pmst, iptr_now);
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 2048) n = 2048;
    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;

    /* Confirm what the PORTR PA=0x0034 serving path will hand the DSP,
     * and also flag if the DSP consumed less than half of the previous
     * batch before a new one arrived (would indicate correlator starvation
     * or DSP never reading via PORTR at all). */
    static uint64_t load_count;
    load_count++;
    if (load_count <= 10 || (load_count % 1000) == 0) {
        C54_LOG("BSP LOAD #%llu n=%d: %04x %04x %04x %04x %04x %04x %04x %04x",
                (unsigned long long)load_count, n,
                n > 0 ? samples[0] : 0, n > 1 ? samples[1] : 0,
                n > 2 ? samples[2] : 0, n > 3 ? samples[3] : 0,
                n > 4 ? samples[4] : 0, n > 5 ? samples[5] : 0,
                n > 6 ? samples[6] : 0, n > 7 ? samples[7] : 0);
    }
}
