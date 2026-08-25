/*
 * calypso_c54x_trace.c - debug instrumentation lifted out of calypso_c54x.c.
 *
 * None of this participates in emulation. Every probe here is a ring buffer, a
 * histogram or a dump, gated on an environment token; with the gates off the
 * only thing they change is what reaches stderr. They were interleaved with the
 * decoder, which is why calypso_c54x.c had grown past 21k lines.
 *
 * Mechanical move: each block below was cut with sed from calypso_c54x.c at the
 * line ranges named in its banner and pasted verbatim. The only edits are the
 * `static` keywords dropped from the fifteen functions the decoder still calls
 * (now declared in calypso_c54x_trace.h). No probe logic was touched.
 *
 * Deliberately LEFT BEHIND in calypso_c54x.c, because their state is written or
 * read directly by the decoder and moving them would trade file-static symbols
 * for externs without shrinking anything: CORR, STKW, NOPGUARD, SP-RING/SP-HIST,
 * the boot and vec28 traces, THROUGHPUT and READSTATS.
 */

#include "qemu/osdep.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "calypso_c54x.h"
#include "calypso_c54x_trace.h"
#include "hw/arm/calypso/calypso_debug.h"


/* ==== ARTRACK — generic ARn write tracer with provenance   [calypso_c54x.c:207-331] ==== */

/* === Generic ARn write tracer with provenance (2026-05-25 v3 unified) ===
 * Tracer paramétré pour AR0..AR7. Remplace les ad-hoc ar2/ar4. Env mask
 * `CALYPSO_AR_TRACE` (hex, default 0) :
 *   =0xFF  → trace tous les AR0..AR7
 *   =0x14  → trace AR2 + AR4 seulement (bits 2 et 4 set)
 *   =0x04  → AR2 only
 *   =0     → désactivé (zéro coût)
 *
 * Hook : `case MMR_AR0..AR7` dans data_write_locked. Skip auto-modify
 * noise (Δ ∈ [-3, 3]). Classification opcode via decode + flag ZERO
 * automatique (= suspect clobber MMR via STL A,*AR-).
 *
 * Question résolue par cette sonde : qui pose ARn = mauvaise valeur ?
 *   STM-#lk    → immediate hardcoded ROM (silicon-intentional, le fix
 *                est ailleurs : étape ultérieure qui ré-set manque)
 *   MVDM-mem   → load depuis mem (slot uninit divergence QEMU vs silicon)
 *   MVMM       → copie d'un autre AR (remonter le tracer sur source)
 *   STM Smem   → load mem indirect (idem MVDM)
 *   STLM-A     → from accumulator A (vérifier d'où A vient) */
#define AR_HIST_MAX 64
typedef struct {
    uint16_t pc;
    uint16_t op_last;
    uint16_t val_last;
    uint32_t count;
} ArEntry;
static ArEntry  g_ar_hist[8][AR_HIST_MAX];
static unsigned g_ar_used[8]    = {0};
static unsigned g_ar_total[8]   = {0};
static unsigned g_ar_mask       = 0;
static int      g_ar_enabled    = -1;
static unsigned g_ar_log_cap    = 50;

void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val)
{
    /* AR3-PRELOAD (revival dsp 2026-06-22, read-only, toujours actif, cape 120) :
     * capture les LOADS d'AR3 dans/autour du buffer I/Q [0x2a00..0x2c00). Le
     * correlateur PC=0xee38 lit AR3=0x2b97 (HORS buffer, fin=0x2b28). ar_write_track
     * n'est appele QUE sur load MMR (STM/STLM/MVDM), PAS sur auto-increment.
     * VERDICT :
     *  - AR3 loade a ~0x2a00 (debut buffer) et AUCUN load >=0x2b28 ici -> le 0x2b97
     *    vient d'INCREMENTS = boucle trop longue / buffer trop court = FIX B (BSP).
     *  - AR3 loade directement >=0x2b28 (flag OUT-OF-BUF) -> instruction mal emulee
     *    = FIX A (decodage c54x), avec le PC/op coupable. */
    if (idx == 3 && new_val >= 0x2a00 && new_val < 0x2c00) {
        static unsigned ar3p_n = 0;
        if (ar3p_n < 120) {
            uint16_t op = prog_fetch(s, s->pc);
            fprintf(stderr, "[c54x] AR3-PRELOAD PC=0x%04x op=0x%04x AR3 %04x->%04x "
                    "%s insn=%u\n", s->pc, op, s->ar[3], new_val,
                    new_val >= 0x2b28 ? "*** OUT-OF-BUF ***" : "(in-buf)",
                    s->insn_count);
            ar3p_n++;
        }
    }
    if (g_ar_enabled < 0) {
        const char *e = getenv("CALYPSO_AR_TRACE");
        g_ar_mask = (e && *e) ? (unsigned)strtoul(e, NULL, 0) : 0xFFu;
        g_ar_enabled = calypso_debug_enabled("AR-TRACE") ? 1 : 0;
        if (g_ar_enabled) {
            fprintf(stderr,
                "[c54x] AR-TRACE enabled, mask=0x%02x (AR0..AR7), "
                "log_cap=%u hist_max=%u\n",
                g_ar_mask, g_ar_log_cap, AR_HIST_MAX);
        }
    }
    if (g_ar_enabled <= 0) return;
    if (idx >= 8) return;
    if (!(g_ar_mask & (1u << idx))) return;

    uint16_t old_val = s->ar[idx];
    int32_t delta = (int32_t)new_val - (int32_t)old_val;
    if (delta >= -3 && delta <= 3) return;  /* skip auto-modify noise */
    g_ar_total[idx]++;

    uint16_t op = prog_fetch(s, s->pc);
    const char *kind = "MISC";
    if ((op & 0xFF80) == 0x7700) kind = "STM-#lk";
    else if ((op & 0xFF00) == 0x8400) kind = "STLM-A";
    else if ((op & 0xFF00) == 0x8600) kind = "MVDM-mem";
    else if ((op & 0xFF00) == 0x8800) kind = "MVMM";
    else if ((op & 0xF800) == 0x8000) kind = "STL-A";

    unsigned i;
    for (i = 0; i < g_ar_used[idx]; i++) {
        if (g_ar_hist[idx][i].pc == s->pc) {
            g_ar_hist[idx][i].count++;
            g_ar_hist[idx][i].val_last = new_val;
            g_ar_hist[idx][i].op_last = op;
            break;
        }
    }
    if (i == g_ar_used[idx] && g_ar_used[idx] < AR_HIST_MAX) {
        g_ar_hist[idx][i].pc = s->pc;
        g_ar_hist[idx][i].op_last = op;
        g_ar_hist[idx][i].val_last = new_val;
        g_ar_hist[idx][i].count = 1;
        g_ar_used[idx]++;
    }
    if (g_ar_total[idx] <= g_ar_log_cap) {
        fprintf(stderr,
            "[c54x] AR%u-W #%u %s @insn=%u PC=0x%04x op=0x%04x  "
            "AR%u %04x → %04x (Δ=%+d)  A=%010llx SP=%04x\n",
            idx, g_ar_total[idx], kind, s->insn_count, s->pc, op,
            idx, old_val, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->sp);
    }
    /* Distinction sémantique critique (cf review Claude web 2026-05-25 v2) :
     * - STM-#lk    = deliberate AR update via immediate hardcoded ROM
     *                (silicon-intentional, l'AR change par design firmware)
     * - LD-#k      = idem (small immediate)
     * - STL-A / autres = side-effect d'un MMR write where AR happens to
     *                    self-alias (= AR pointing at its own MMR slot).
     *                    NOT an explicit AR update — coincidence pointer.
     * Label clairement pour ne pas confondre les 2 dans le hunt. */
    if (new_val == 0) {
        int deliberate = ((op & 0xFF80) == 0x7700);  /* STM-#lk only */
        fprintf(stderr,
            "[c54x] AR%u-W ZERO %s @insn=%u PC=0x%04x op=0x%04x AR%u←0 "
            "(kind=%s)\n",
            idx,
            deliberate ? "DELIBERATE" : "SIDE-EFFECT",
            s->insn_count, s->pc, op, idx, kind);
    }
}

/* ==== ATRACK — A accumulator provenance tracer   [calypso_c54x.c:333-395] ==== */

/* === A accumulator provenance tracer (2026-05-25 v3) ===
 * Capture le LAST WRITER de A via chokepoint top-of-loop : compare A
 * iter-à-iter, si change → mémorise PC + op du writer. Quand un trigger
 * PC fire (default = 0x9ac0 = STL A,*AR2- clobber IMR), dump A + last
 * writer. Réponse à la question Claude web : A=0 délibéré (= mask-all
 * design firmware) ou A=0 divergence (= A devait porter mask valide) ?
 *
 * Env : CALYPSO_A_TRACE_PC=0x9ac0 (hex PC trigger, default 0xFFFF=off)
 * Zéro coût si env non set. */
static int64_t  g_a_last_value      = 0;
static uint16_t g_a_last_writer_pc  = 0;
static uint16_t g_a_last_writer_op  = 0;
static unsigned g_a_last_writer_insn = 0;
static int      g_a_trace_enabled   = -1;
static uint16_t g_a_trace_pc        = 0xFFFF;
static unsigned g_a_trace_hits      = 0;
static unsigned g_a_trace_log_cap   = 50;

void a_track_init_lazy(void)
{
    if (g_a_trace_enabled >= 0) return;
    const char *e = getenv("CALYPSO_A_TRACE_PC");
    if (calypso_debug_enabled("A-TRACE")) {
        g_a_trace_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_a_trace_enabled = 1;
        fprintf(stderr,
            "[c54x] A-TRACE enabled, trigger PC=0x%04x log_cap=%u\n",
            g_a_trace_pc, g_a_trace_log_cap);
    } else {
        g_a_trace_enabled = 0;
    }
}

void a_track_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_a_trace_enabled <= 0) return;
    /* Detect A change → mémorise dernier writer */
    if (s->a != g_a_last_value) {
        g_a_last_writer_pc   = prev_pc;
        g_a_last_writer_op   = prev_op;
        g_a_last_writer_insn = s->insn_count;
        g_a_last_value       = s->a;
    }
    /* Trigger : PC about to execute matches target */
    if (s->pc == g_a_trace_pc) {
        g_a_trace_hits++;
        int a_zero = ((s->a & 0xFFFF) == 0);
        /* Log if (a) parmi les N premiers (contexte) OR (b) A_low=0 (= cas
         * suspect STL clobber zone). Évite cap log silencieux qui masque
         * les events critiques tardifs (cf cas insn=253328 IMR clobber). */
        if (g_a_trace_hits <= g_a_trace_log_cap || a_zero) {
            fprintf(stderr,
                "[c54x] A-AT-PC #%u @insn=%u PC=0x%04x  A=%010llx (low=0x%04x, %s) "
                "last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_a_trace_hits, s->insn_count, s->pc,
                (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                (unsigned)(s->a & 0xFFFF),
                a_zero ? "A_low=0 → STL clobber zone" : "A_low≠0",
                g_a_last_writer_pc, g_a_last_writer_op,
                g_a_last_writer_insn);
        }
    }
}

/* ==== AR6AT — AR6 windowed snapshot at trigger PC   [calypso_c54x.c:397-478] ==== */

/* === AR6 windowed snapshot at trigger PC (2026-05-25 v4) ===
 * Capture AR6 + B + source provenance à chaque fire d'un PC trigger,
 * fenêtré sur [insn_lo, insn_hi] pour éviter explosion log (le PC 0x821a
 * fire 10M+ fois). Réponse à la question Claude web : aux fires qui
 * clobber IMR à PC=0x821a, AR6 vaut 0 (= base divergence) ou 0x16
 * (= self-alias feedback) ?
 *
 * Tracking AR6's last writer (= what set AR6 to its current value)
 * via top-of-loop comparison (même pattern que A tracer).
 *
 * Env :
 *   CALYPSO_AR6_AT_PC=0x821a    PC trigger
 *   CALYPSO_AR6_WIN_LO=3619500  insn window start
 *   CALYPSO_AR6_WIN_HI=3619810  insn window end (one outer-loop iter)
 *   CALYPSO_AR6_AT_LOG_CAP=200  max log lines (default 200)
 */
static uint16_t g_ar6_last_value     = 0;
static uint16_t g_ar6_last_writer_pc = 0;
static uint16_t g_ar6_last_writer_op = 0;
static unsigned g_ar6_last_writer_insn = 0;
static int      g_ar6_at_enabled     = -1;
static uint16_t g_ar6_at_pc          = 0xFFFF;
static unsigned g_ar6_at_win_lo      = 0;
static unsigned g_ar6_at_win_hi      = 0;
static unsigned g_ar6_at_hits        = 0;
static unsigned g_ar6_at_log_cap     = 200;

void ar6_at_init_lazy(void)
{
    if (g_ar6_at_enabled >= 0) return;
    const char *e = getenv("CALYPSO_AR6_AT_PC");
    if (calypso_debug_enabled("AR6-AT")) {
        g_ar6_at_pc = (e && *e) ? (uint16_t)strtoul(e, NULL, 0) : 0;
        g_ar6_at_enabled = 1;
        const char *lo = getenv("CALYPSO_AR6_WIN_LO");
        const char *hi = getenv("CALYPSO_AR6_WIN_HI");
        const char *cap = getenv("CALYPSO_AR6_AT_LOG_CAP");
        g_ar6_at_win_lo = (lo && *lo) ? (unsigned)strtoul(lo, NULL, 0) : 0;
        g_ar6_at_win_hi = (hi && *hi) ? (unsigned)strtoul(hi, NULL, 0) : 0xFFFFFFFFu;
        g_ar6_at_log_cap = (cap && *cap) ? (unsigned)strtoul(cap, NULL, 0) : 200;
        fprintf(stderr,
            "[c54x] AR6-AT-PC enabled, trigger PC=0x%04x window=[%u..%u] cap=%u\n",
            g_ar6_at_pc, g_ar6_at_win_lo, g_ar6_at_win_hi, g_ar6_at_log_cap);
    } else {
        g_ar6_at_enabled = 0;
    }
}

void ar6_at_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op)
{
    if (g_ar6_at_enabled <= 0) return;
    /* Track AR6 last writer */
    if (s->ar[6] != g_ar6_last_value) {
        g_ar6_last_writer_pc   = prev_pc;
        g_ar6_last_writer_op   = prev_op;
        g_ar6_last_writer_insn = s->insn_count;
        g_ar6_last_value       = s->ar[6];
    }
    /* Trigger : PC about to execute matches AND within window */
    if (s->pc == g_ar6_at_pc &&
        s->insn_count >= g_ar6_at_win_lo &&
        s->insn_count <= g_ar6_at_win_hi) {
        g_ar6_at_hits++;
        if (g_ar6_at_hits <= g_ar6_at_log_cap) {
            uint16_t ar6 = s->ar[6];
            const char *regime;
            if (ar6 == 0)              regime = "AR6=0 → addr=IMR (BUFFER BASE DIVERGENCE)";
            else if (ar6 == 0x16)      regime = "AR6=0x16 → addr=MMR_AR6 (SELF-ALIAS)";
            else if (ar6 < 0x20)       regime = "AR6 in MMR zone";
            else                        regime = "AR6 normal";
            fprintf(stderr,
                "[c54x] AR6-AT-PC #%u @insn=%u PC=0x%04x  AR6=0x%04x (%s) "
                "B=%010llx (high=0x%04x)  last_writer: PC=0x%04x op=0x%04x @insn=%u\n",
                g_ar6_at_hits, s->insn_count, s->pc,
                ar6, regime,
                (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                (unsigned)((s->b >> 16) & 0xFFFF),
                g_ar6_last_writer_pc, g_ar6_last_writer_op,
                g_ar6_last_writer_insn);
        }
    }
}

/* ==== RSBX — INTM hits counter (state)   [calypso_c54x.c:481-483] ==== */

/* RSBX INTM hits counter (cheap probe, candidat 1 du doc §7). */
static uint64_t g_rsbx_intm_hits = 0;
static int      g_rsbx_intm_enabled = -1;

/* ==== RSBX — INTM check   [calypso_c54x.c:566-586] ==== */

void rsbx_intm_check(C54xState *s, uint16_t op)
{
    if (g_rsbx_intm_enabled < 0) {
        const char *e = cdbg_env("RSBX-INTM");
        g_rsbx_intm_enabled = (e && *e == '1') ? 1 : 0;
        if (g_rsbx_intm_enabled) {
            fprintf(stderr, "[c54x] RSBX-INTM-TRACE enabled (op=0xF6BB)\n");
        }
    }
    if (g_rsbx_intm_enabled <= 0) return;
    if (op == 0xF6BB) {
        g_rsbx_intm_hits++;
        if (g_rsbx_intm_hits <= 20 || (g_rsbx_intm_hits % 1000) == 0) {
            fprintf(stderr,
                "[c54x] RSBX-INTM #%llu @insn=%u PC=0x%04x  ST1 INTM 0x%04x → "
                "(cleared) — IRQ enable path atteint !\n",
                (unsigned long long)g_rsbx_intm_hits, s->insn_count, s->pc,
                s->st1);
        }
    }
}

/* ==== AR4 tracer — HISTORICAL NOTE ONLY, the tracer itself is gone   [calypso_c54x.c:588-602] ==== */

/* === AR4 write tracer with provenance (2026-05-25) ===
 * Hooke chaque write vers MMR_AR4 (= 0x14) via data_write_locked.
 * Logge (insn, PC, opcode courant, val écrite, ancien AR4) + tente une
 * classification de provenance via decode opcode :
 *   STM #lk, ARn  (0x77yx)  → immediate value depuis next prog word
 *   STLM src,ARn  (0x84yx)  → from accumulator A/B low
 *   MVDM dmad,ARn (0x86yx)  → from data memory absolute
 *   MVDD/MVMM     (autres)  → from another register
 * Flag SUSPECT si nouvelle valeur AR4 ∈ [0x2b80..0x2c00] (observed bug
 * zone) ou [0x3fb0..0x3fbf] (BSP buffer area). Env CALYPSO_AR4_TRACE=1.
 *
 * Question critique (cf Claude web) : provenance = const/mem/register ?
 * Si AR4 vient d'un mem load (LDM/MVDM), le corrupter remonte au TCB
 * en mémoire — pas l'instruction qui charge AR4, mais le TCB lui-même
 * (potentiellement uninitialized faute de re-init firmware sautée). */

/* ==== SPABS — SP absolute-write tracer   [calypso_c54x.c:603-678] ==== */

/* === SP absolute-write tracer (2026-05-25 — nohack hunt) ===
 * Logge chaque write SP via STL/STM/STLM absolute, FRAME #imm, MVMM
 * register transfer — c'est-à-dire les sites où SP est *téléporté* à une
 * valeur arbitraire, par opposition aux PUSH/POP/CALL/RET qui sont des
 * inc/dec de 1. Si un site téléporte SP=0x3fbe, on tient le corrupter
 * exact du bootstub-entry observé à insn=3995013.
 *
 * Hooké aux 3 sites identifiés :
 *   L1218 : data_write_locked case MMR_SP — STL/STM/STLM to MMR_SP
 *   L3875 : F7Dx case 0xD — LD #k8u, SP
 *   L4285 : MVMM register transfer — dst==8 (SP via MMR enc 3-bit)
 *
 * Env-gated CALYPSO_SP_ABS_TRACE=1, zéro coût si OFF.
 * Limite N premiers writes verbatim + histo per-PC (cap SP_ABS_HIST_MAX). */
#define SP_ABS_HIST_MAX 128
typedef struct {
    uint16_t pc;
    uint16_t value_last;
    uint32_t count;
    uint8_t  site;   /* 0=MMR_SP, 1=LDK8, 2=MVMM */
} SpAbsEntry;
static SpAbsEntry g_sp_abs_hist[SP_ABS_HIST_MAX];
static unsigned   g_sp_abs_used     = 0;
static unsigned   g_sp_abs_total    = 0;
static int        g_sp_abs_enabled  = -1;
static unsigned   g_sp_abs_log_cap  = 50;

void sp_abs_track(C54xState *s, uint16_t new_val, uint8_t site)
{
    if (g_sp_abs_enabled < 0) {
        const char *e = cdbg_env("SP-ABS");
        g_sp_abs_enabled = (e && *e == '1') ? 1 : 0;
        if (g_sp_abs_enabled) {
            fprintf(stderr, "[c54x] SP-ABS-TRACE enabled, log_cap=%u hist_max=%u\n",
                    g_sp_abs_log_cap, SP_ABS_HIST_MAX);
        }
    }
    if (g_sp_abs_enabled <= 0) return;
    g_sp_abs_total++;
    /* Per-PC histo */
    unsigned i;
    for (i = 0; i < g_sp_abs_used; i++) {
        if (g_sp_abs_hist[i].pc == s->pc && g_sp_abs_hist[i].site == site) {
            g_sp_abs_hist[i].count++;
            g_sp_abs_hist[i].value_last = new_val;
            break;
        }
    }
    if (i == g_sp_abs_used && g_sp_abs_used < SP_ABS_HIST_MAX) {
        g_sp_abs_hist[i].pc         = s->pc;
        g_sp_abs_hist[i].value_last = new_val;
        g_sp_abs_hist[i].count      = 1;
        g_sp_abs_hist[i].site       = site;
        g_sp_abs_used++;
    }
    /* Verbatim log first N */
    if (g_sp_abs_total <= g_sp_abs_log_cap) {
        const char *site_name = site == 0 ? "MMR_SP-W" :
                                site == 1 ? "LD-#k8-SP" : "MVMM-SP";
        int32_t delta = (int32_t)new_val - (int32_t)s->sp;
        fprintf(stderr,
            "[c54x] SP-ABS #%u %s @insn=%u PC=0x%04x SP %04x → %04x (Δ=%+d) "
            "A=%010llx AR4=%04x\n",
            g_sp_abs_total, site_name, s->insn_count, s->pc,
            s->sp, new_val, delta,
            (unsigned long long)(s->a & 0xFFFFFFFFFFULL), s->ar[4]);
    }
    /* Flag if SP lands in suspect zone (0x3fb0..0x3fbf = BSP read region
     * OR 0x2b80..0x2c00 = 0xfd2a A=AR4 historical) */
    if ((new_val >= 0x3fb0 && new_val <= 0x3fbf) ||
        (new_val >= 0x2b80 && new_val <= 0x2c00)) {
        fprintf(stderr,
            "[c54x] SP-ABS SUSPECT! @insn=%u PC=0x%04x SP←0x%04x (corrupter ?)\n",
            s->insn_count, s->pc, new_val);
    }
}

/* ==== STUCK — PC+XPC histogram probe   [calypso_c54x.c:906-990] ==== */

/* === STUCK-STATE PC+XPC histogram (c web reframe 2026-05-25 night3) =====
 *
 * Sonde diagnostique : quand le DSP est en "stuck state" (= INTM=1 ET
 * BRINT0 pending dans IFR), on enregistre PC+XPC. Permet d'identifier
 * la VRAIE boucle de blocage XPC-qualifiée — sans présumer que c'est
 * fc50 ou autre PC particulier.
 *
 * Le PC HIST classique ne distingue pas les pages XPC (= ambiguous "fc50"
 * peut être page 0x1F mirror ou 0x28/0x38 etc.).
 *
 * Entry/exit du stuck state logué (= delimit la fenêtre).
 * Top-20 PC+XPC dump périodique (= quand stuck dure).
 *
 * Env-gated CALYPSO_STUCK_PROBE=1. Coût off : 1 bit-check + 1 branch. */
#define STUCK_HIST_SIZE 64
typedef struct {
    uint16_t pc;
    uint8_t  xpc;
    uint32_t count;
} StuckHistEntry;
static StuckHistEntry g_stuck_hist[STUCK_HIST_SIZE];
static unsigned g_stuck_hist_used = 0;
static int g_stuck_probe_enabled = -1;
static int g_stuck_active = 0;
static uint32_t g_stuck_duration = 0;
static uint64_t g_stuck_start_insn = 0;
static unsigned g_stuck_dump_count = 0;

static void stuck_probe_init_lazy(void)
{
    if (g_stuck_probe_enabled >= 0) return;
    const char *e = cdbg_env("STUCK");
    g_stuck_probe_enabled = (e && *e == '1') ? 1 : 0;
    if (g_stuck_probe_enabled) {
        fprintf(stderr,
            "[c54x] STUCK-PROBE enabled : capture PC+XPC histogramme quand "
            "INTM=1 + IFR bit5 (BRINT0) pending\n");
    }
}

static void stuck_probe_record(uint16_t pc, uint8_t xpc)
{
    /* Linear scan small hist (cap 64). Insert or increment. */
    for (unsigned i = 0; i < g_stuck_hist_used; i++) {
        if (g_stuck_hist[i].pc == pc && g_stuck_hist[i].xpc == xpc) {
            g_stuck_hist[i].count++;
            return;
        }
    }
    if (g_stuck_hist_used < STUCK_HIST_SIZE) {
        g_stuck_hist[g_stuck_hist_used].pc = pc;
        g_stuck_hist[g_stuck_hist_used].xpc = xpc;
        g_stuck_hist[g_stuck_hist_used].count = 1;
        g_stuck_hist_used++;
    }
    /* If hist full, silently drop new PCs — top hot ones already captured. */
}

static void stuck_probe_dump(uint64_t cur_insn, const char *trig)
{
    /* Bubble sort by count desc (n<=64). */
    for (unsigned k = 0; k < g_stuck_hist_used; k++) {
        unsigned best = k;
        for (unsigned i = k + 1; i < g_stuck_hist_used; i++) {
            if (g_stuck_hist[i].count > g_stuck_hist[best].count) best = i;
        }
        if (best != k) {
            StuckHistEntry tmp = g_stuck_hist[k];
            g_stuck_hist[k] = g_stuck_hist[best];
            g_stuck_hist[best] = tmp;
        }
    }
    if (calypso_debug_enabled("STUCK-HIST")) fprintf(stderr,
        "[c54x] STUCK-HIST [%s] duration=%u insn since insn=%llu (now=%llu) top:\n",
        trig, g_stuck_duration,
        (unsigned long long)g_stuck_start_insn,
        (unsigned long long)cur_insn);
    unsigned n_show = g_stuck_hist_used > 20 ? 20 : g_stuck_hist_used;
    for (unsigned i = 0; i < n_show; i++) {
        fprintf(stderr,
            "[c54x]   #%2u PC=0x%04x XPC=%u  count=%u\n",
            i + 1, g_stuck_hist[i].pc, g_stuck_hist[i].xpc,
            g_stuck_hist[i].count);
    }
}

/* ==== INT3 — cycle tracer + control-flow signature   [calypso_c54x.c:1092-1280] ==== */

/* === INT3 cycle tracer + control-flow signature (c web reframe 2026-05-25 night5)
 *
 * Sonde décisive pour départager F1 (= pourquoi ISR INT3 ne RETE pas).
 *
 * Per cycle INT3 :
 *   - START : INT3 dispatched (vec=19) → reset trace, log cycle_id + entry PC
 *   - DURING : chaque branch conditionnelle exécutée → (PC, op, target, taken)
 *   - END (good) : RETE fire → dump trace tagged GOOD + insn count
 *   - END (orphan) : nouveau INT3 dispatch avant RETE → dump previous tagged
 *                   ORPHAN-NEXT-INT3 + reason
 *
 * Diff offline good_cycle vs orphan_cycle → 1ère branche qui diverge
 * = trigger du bug. À cette branche, lire l'état testé = la vraie cause.
 *
 * Cappé 256 branches/cycle (= overflow tagué pour borne).
 * Env-gated CALYPSO_INT3_CYCLE_TRACE=1. */
#define INT3_BRANCH_TRACE_MAX 1024
typedef struct {
    uint16_t pc;        /* PC of branch insn */
    uint16_t op;        /* opcode word 0 */
    uint16_t next_pc;   /* PC after exec (= branch taken target OR fall-through) */
    uint32_t insn_offset; /* delta from cycle start (first occurrence) */
    uint32_t repeat;    /* consecutive identical (pc,op,next_pc) collapsed count */
} Int3BranchEvent;
static Int3BranchEvent g_int3_trace[INT3_BRANCH_TRACE_MAX];
static unsigned g_int3_trace_count = 0;
static int g_int3_trace_overflow = 0;
static int g_int3_cycle_active = 0;
static uint64_t g_int3_cycle_id = 0;
static uint16_t g_int3_cycle_entry_pc = 0;
static uint64_t g_int3_cycle_entry_insn = 0;
static int g_int3_trace_enabled = -1;

static void int3_trace_init_lazy(void)
{
    if (g_int3_trace_enabled >= 0) return;
    const char *e = cdbg_env("INT3-CYCLE");
    g_int3_trace_enabled = (e && *e == '1') ? 1 : 0;
    if (g_int3_trace_enabled) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE-TRACE enabled : par cycle vec=19, log toutes "
            "branches conditionnelles + RETE/orphan tag. Cap=%u branches/cycle.\n",
            INT3_BRANCH_TRACE_MAX);
    }
}

/* Detect conditional branch / call / return family. Returns 1 if op
 * is in a tracked branch family, else 0. */
static int is_int3_traced_branch(uint16_t op)
{
    uint16_t hi = op & 0xFF00;
    if (hi == 0x6C00) return 1;  /* BANZ pmad,Sind */
    if (hi == 0x6E00) return 1;  /* BANZD pmad,Sind */
    if (hi == 0xF800) return 1;  /* BC pmad,cond */
    if (hi == 0xF900) return 1;  /* CC pmad,cond */
    if (hi == 0xFA00) return 1;  /* BCD pmad,cond */
    if (hi == 0xFB00) return 1;  /* CCD pmad,cond */
    if (hi == 0xFC00) {
        /* FC00 unconditional = RET ; FCxx where xx is cond = RC */
        if (op != 0xFC00) return 1;
        return 0;
    }
    if (hi == 0xFE00) {
        if (op != 0xFE00) return 1; /* RCD cond */
        return 0;
    }
    return 0;
}

/* Called from c54x_interrupt_ex when vec=19 (INT3 FRAME) dispatched. */
void int3_cycle_start(C54xState *s, uint16_t target_pc)
{
    if (g_int3_trace_enabled < 0) int3_trace_init_lazy();
    if (g_int3_trace_enabled <= 0) return;
    /* If previous cycle still active = orphan (= didn't RETE before re-entry) */
    if (g_int3_cycle_active) {
        fprintf(stderr,
            "[c54x] INT3-CYCLE #%llu ORPHAN-NEXT-INT3 — previous cycle didn't "
            "RETE, new entry @insn=%llu PC=0x%04x. Trace below (%u branches%s) :\n",
            (unsigned long long)g_int3_cycle_id,
            (unsigned long long)s->insn_count, target_pc,
            g_int3_trace_count,
            g_int3_trace_overflow ? "+ OVERFLOW" : "");
        for (unsigned i = 0; i < g_int3_trace_count; i++) {
            Int3BranchEvent *e = &g_int3_trace[i];
            /* 2-word branches (no lk_used here for simplicity) : BC/CC/BCD/CCD
             * (0xF8-0xFB) and BANZ/BANZD (0x6C/0x6E). 1-word : RET/RC/RCD. */
            uint16_t hi = e->op & 0xFF00;
            bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                            || hi == 0x6C00 || hi == 0x6E00;
            uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
            fprintf(stderr,
                "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
                i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
                (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
                e->repeat);
        }
    }
    g_int3_cycle_id++;
    g_int3_cycle_active = 1;
    g_int3_cycle_entry_pc = target_pc;
    g_int3_cycle_entry_insn = s->insn_count;
    g_int3_trace_count = 0;
    g_int3_trace_overflow = 0;
    if (calypso_debug_enabled("INT3-CYCLE")) fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu START @insn=%llu PC→0x%04x SP=0x%04x "
        "PMST=0x%04x IFR=0x%04x\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, target_pc, s->sp,
        s->pmst, s->ifr);
}

/* Called from c54x_run after c54x_exec_one. exec_pc/exec_op are the
 * instruction that just executed; s->pc is the resulting PC. */
void int3_cycle_track_branch(C54xState *s, uint16_t exec_pc,
                                    uint16_t exec_op, int consumed)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    if (!is_int3_traced_branch(exec_op)) return;
    /* Compute next_pc correctly across all branch-handler patterns :
     * 1. Non-delayed branch TAKEN  → handler set s->pc=target, returned consumed=0
     *    → s->pc already = target.
     * 2. Delayed branch TAKEN      → handler armed delay_slots=2 + delayed_pc,
     *    returned consumed>0; main loop hasn't run +=consumed yet
     *    → eventual target = s->delayed_pc.
     * 3. Branch FALL-THROUGH (any) → handler returned consumed>0, s->pc unchanged,
     *    delay_slots not set; main loop will += consumed → next insn
     *    → next = exec_pc + consumed. */
    uint16_t actual_next;
    if (consumed == 0) {
        actual_next = s->pc;
    } else if (s->delay_slots == 2) {
        actual_next = s->delayed_pc;
    } else {
        actual_next = (uint16_t)(exec_pc + consumed);
    }

    /* Dedup-pattern : look up to 4 slots back. Catches consecutive
     * identical (distance 1, AAA), strict alternation (distance 2,
     * ABAB), and short cycles up to length 4 (ABCDABCD). Each iteration
     * of the repeating pattern bumps the matched slot's repeat — total
     * iterations = max(repeat) across slots forming the cycle. */
    for (unsigned back = 1; back <= 4 && back <= g_int3_trace_count; back++) {
        Int3BranchEvent *cand = &g_int3_trace[g_int3_trace_count - back];
        if (cand->pc == exec_pc && cand->op == exec_op && cand->next_pc == actual_next) {
            cand->repeat++;
            return;
        }
    }
    if (g_int3_trace_count >= INT3_BRANCH_TRACE_MAX) {
        g_int3_trace_overflow = 1;
        return;
    }
    Int3BranchEvent *e = &g_int3_trace[g_int3_trace_count++];
    e->pc = exec_pc;
    e->op = exec_op;
    e->next_pc = actual_next;
    e->insn_offset = (uint32_t)(s->insn_count - g_int3_cycle_entry_insn);
    e->repeat = 1;
}

/* Called from RETE handler (L3300 area) BEFORE INTM is cleared. */
void int3_cycle_end_good(C54xState *s, uint16_t return_addr)
{
    if (g_int3_trace_enabled <= 0) return;
    if (!g_int3_cycle_active) return;
    uint64_t duration = s->insn_count - g_int3_cycle_entry_insn;
    fprintf(stderr,
        "[c54x] INT3-CYCLE #%llu RETE-GOOD @insn=%llu duration=%llu PC→0x%04x "
        "branches=%u%s\n",
        (unsigned long long)g_int3_cycle_id,
        (unsigned long long)s->insn_count, (unsigned long long)duration,
        return_addr, g_int3_trace_count,
        g_int3_trace_overflow ? "+ OVERFLOW" : "");
    for (unsigned i = 0; i < g_int3_trace_count; i++) {
        Int3BranchEvent *e = &g_int3_trace[i];
        uint16_t hi = e->op & 0xFF00;
        bool two_word = (hi >= 0xF800 && hi <= 0xFB00)
                        || hi == 0x6C00 || hi == 0x6E00;
        uint16_t fallthrough = e->pc + (two_word ? 2 : 1);
        fprintf(stderr,
            "[c54x]   #%3u Δ%u PC=0x%04x op=0x%04x → next=0x%04x %s ×%u\n",
            i + 1, e->insn_offset, e->pc, e->op, e->next_pc,
            (e->next_pc == fallthrough) ? "(NOT_TAKEN)" : "(TAKEN)",
            e->repeat);
    }
    g_int3_cycle_active = 0;
}

/* ==== STUCK — per-loop check   [calypso_c54x.c:1282-1321] ==== */

/* Called from c54x_run top-of-loop. */
void stuck_probe_check(C54xState *s)
{
    if (g_stuck_probe_enabled < 0) stuck_probe_init_lazy();
    if (g_stuck_probe_enabled <= 0) return;
    int intm_set = !!(s->st1 & ST1_INTM);
    int brint0_pending = !!(s->ifr & (1 << 5));
    int now_stuck = (intm_set && brint0_pending);
    if (now_stuck && !g_stuck_active) {
        g_stuck_active = 1;
        g_stuck_start_insn = s->insn_count;
        g_stuck_duration = 0;
        g_stuck_hist_used = 0;  /* fresh hist per stuck window */
        fprintf(stderr,
            "[c54x] STUCK-ENTER insn=%llu PC=0x%04x XPC=%u IFR=0x%04x IMR=0x%04x\n",
            (unsigned long long)s->insn_count, s->pc, s->xpc & 0xFF,
            s->ifr, s->imr);
    }
    if (now_stuck) {
        g_stuck_duration++;
        /* Sample every 100 insns to bound hist diversity */
        if ((g_stuck_duration % 100) == 0) {
            stuck_probe_record(s->pc, s->xpc & 0xFF);
        }
        /* Dump periodically while stuck */
        if ((g_stuck_duration % 5000000) == 0 && g_stuck_dump_count < 5) {
            g_stuck_dump_count++;
            stuck_probe_dump(s->insn_count, "periodic-5M");
        }
    } else if (g_stuck_active) {
        g_stuck_active = 0;
        fprintf(stderr,
            "[c54x] STUCK-EXIT insn=%llu duration=%u PC=0x%04x XPC=%u IFR=0x%04x\n",
            (unsigned long long)s->insn_count, g_stuck_duration,
            s->pc, s->xpc & 0xFF, s->ifr);
        if (g_stuck_duration >= 10000) {  /* only dump if long-ish stuck */
            stuck_probe_dump(s->insn_count, "on-exit");
        }
    }
}

/* ==== RMAP — aggregated map of addresses READ   [calypso_c54x.c:1844-1898] ==== */

/* [2026-07-28] RMAP : carte agregee des adresses LUES par les PC d une plage.
 * Symetrique de WMAP. Voir en-tete du patch rmap.py. */
#define RMAP_PCS 48
static struct { uint16_t pc; uint32_t n; uint16_t a[8]; uint8_t na;
                uint16_t amn, amx; } g_rmap[RMAP_PCS];
static int      g_rmap_n;
static uint32_t g_rmap_tot;
static int      g_rmap_on = -1;
static uint16_t g_rmap_pclo, g_rmap_pchi;

static void rmap_dump(void)
{
    fprintf(stderr, "[c54x] RMAP PC 0x%04x..0x%04x  lectures=%u  PCs=%d%s\n",
            g_rmap_pclo, g_rmap_pchi, g_rmap_tot, g_rmap_n,
            g_rmap_n >= RMAP_PCS ? "  *** SATUREE ***" : "");
    for (int i = 0; i < g_rmap_n; i++) {
        fprintf(stderr, "[c54x] RMAP   PC=0x%04x n=%-7u lit 0x%04x..0x%04x  ex:",
                g_rmap[i].pc, g_rmap[i].n, g_rmap[i].amn, g_rmap[i].amx);
        for (int k = 0; k < g_rmap[i].na && k < 8; k++)
            fprintf(stderr, " %04x", g_rmap[i].a[k]);
        fprintf(stderr, "\n");
    }
}

void rmap_note(uint16_t addr, uint16_t pc)
{
    if (g_rmap_on < 0) {
        const char *e = getenv("CALYPSO_RMAP");
        g_rmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = getenv("CALYPSO_RMAP_PCLO"), *hi = getenv("CALYPSO_RMAP_PCHI");
        g_rmap_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f00;
        g_rmap_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fff;
        if (g_rmap_on)
            fprintf(stderr, "[c54x] RMAP armed PC 0x%04x..0x%04x\n", g_rmap_pclo, g_rmap_pchi);
    }
    if (!g_rmap_on || pc < g_rmap_pclo || pc > g_rmap_pchi) return;

    int i;
    for (i = 0; i < g_rmap_n; i++) if (g_rmap[i].pc == pc) break;
    if (i == g_rmap_n) {
        if (g_rmap_n >= RMAP_PCS) return;
        g_rmap_n++;
        g_rmap[i].pc = pc; g_rmap[i].n = 0; g_rmap[i].na = 0;
        g_rmap[i].amn = 0xffff; g_rmap[i].amx = 0;
    }
    g_rmap[i].n++;
    if (addr < g_rmap[i].amn) g_rmap[i].amn = addr;
    if (addr > g_rmap[i].amx) g_rmap[i].amx = addr;
    if (g_rmap[i].na < 8) {
        int seen = 0;
        for (int k = 0; k < g_rmap[i].na; k++) if (g_rmap[i].a[k] == addr) { seen = 1; break; }
        if (!seen) g_rmap[i].a[g_rmap[i].na++] = addr;
    }
    if (++g_rmap_tot % 5000 == 0) rmap_dump();
}

/* ==== SCRATCHWR — scratch write probe   [calypso_c54x.c:2687-2744] ==== */

/* [2026-08-22] SCRATCH-WR (CALYPSO_SCRATCH_WR, defaut OFF) — la table de
 * coefficients du FIRS est-elle JAMAIS ecrite ?
 * FIRS-COEF a montre prog[0x61..0x66]=0xF4E4 (remplissage) ET data[0x61..0x66]=0
 * au moment de l execution. Le PROGRAMME 0x0000-0x6FFF n est couvert par aucun
 * fichier ROM (PROM0 debute a 0x7000) — c est exactement la que tombe pmad=0x61.
 *   aucune ecriture   -> segment ROM manquant ;
 *   ecriture tardive  -> ordonnancement.
 * On date (insn) et on situe (PC) chaque ecriture des DEUX espaces, avec bilan
 * periodique pour distinguer « rien ecrit » de « sonde muette ». LECTURE SEULE. */
static int scratchwr_on(void)
{
    static int g = -1;
    if (g < 0) {
        g = calypso_gate("CALYPSO_SCRATCH_WR", 0);
        fprintf(stderr, "[c54x] SCRATCH-WR %s : ecritures dans 0x0060-0x007F "
                "(scratch-pad DARAM + espace PROGRAMME via MVDP) — zone des "
                "coefficients du FIRS (pmad=0x0061)\n",
                g ? "ACTIVE" : "INACTIVE (defaut)");
    }
    return g;
}

void scratchwr_note(C54xState *s, uint16_t a, uint16_t v, const char *espace)
{
    if (!scratchwr_on()) return;
    /* [2026-08-22] v2 : plafond PAR ADRESSE, jamais global. La v1 avait un
     * plafond global de 40 lignes, consomme par data[0x74] a insn ~5000-10000 ;
     * les ecritures des coefficients (MVDD a insn ~1376674) n ont donc jamais
     * ete affichees et j ai lu ce plafond comme une absence. */
    int zone = -1;
    if (a >= 0x0060 && a <= 0x007F)      zone = 0;   /* destination */
    else if (a >= 0x2CB0 && a <= 0x2CBF) zone = 1;   /* SOURCE de la table */
    if (zone < 0) return;

    static unsigned long long cum[2] = {0, 0};
    static unsigned long long cum_coef = 0;   /* 0x0061..0x0066 seulement */
    static unsigned long long cum_src  = 0;   /* 0x2CB9..0x2CBF seulement */
    static unsigned char shown[2][0x20];

    cum[zone]++;
    if (a >= 0x0061 && a <= 0x0066) cum_coef++;
    if (a >= 0x2CB9 && a <= 0x2CBF) cum_src++;

    unsigned idx = (zone == 0) ? (unsigned)(a - 0x0060) : (unsigned)(a - 0x2CB0);
    if (idx < 0x20 && shown[zone][idx] < 3) {
        shown[zone][idx]++;
        fprintf(stderr, "[c54x] SCRATCH-WR %s[0x%04x] <- 0x%04x PC=0x%04x insn=%u%s\n",
                espace, a, v, s->pc, s->insn_count,
                (a >= 0x0061 && a <= 0x0066) ? "  <<<< COEFFICIENTS (destination)" :
                (a >= 0x2CB9 && a <= 0x2CBF) ? "  <<<< SOURCE de la table" : "");
    }
    if (((cum[0] + cum[1]) % 500) == 0)
        fprintf(stderr, "[c54x] SCRATCH-WR bilan : scratch(0x60-0x7F)=%llu "
                "source(0x2CB0-0x2CBF)=%llu | dont coefficients(0x61-0x66)=%llu "
                "source utile(0x2CB9-0x2CBF)=%llu\n",
                (unsigned long long)cum[0], (unsigned long long)cum[1],
                (unsigned long long)cum_coef, (unsigned long long)cum_src);
}

/* ==== WMAP — aggregated map of writers   [calypso_c54x.c:3141-3225] ==== */

/* [2026-07-28] WMAP : carte agregee des ecrivains d une plage data[].
 * Voir en-tete du patch wmap.py — un agregat, pas un flux : aucun plafond de
 * lignes, donc aucune fenetre a rater (les 3 sondes precedentes ont toutes ete
 * tronquees). Sortie periodique : PC, n, valeurs distinctes, min/max. */
#define WMAP_PCS 64
static struct { uint16_t pc; uint32_t n; uint16_t v[8]; uint8_t nv;
                uint16_t mn, mx; uint16_t addr0; uint32_t n0; } g_wmap[WMAP_PCS];
static int      g_wmap_n;
static uint32_t g_wmap_tot;
static int      g_wmap_on = -1;
static uint16_t g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2;

static void wmap_dump(void)
{
    fprintf(stderr, "[c54x] WMAP plages 0x%04x..0x%04x + 0x%04x..0x%04x  ecritures=%u  ecrivains=%d%s\n",
            g_wmap_lo, g_wmap_hi, g_wmap_lo2, g_wmap_hi2, g_wmap_tot, g_wmap_n,
            g_wmap_n >= WMAP_PCS ? "  *** TABLE SATUREE, ecrivains manquants ***" : "");
    for (int i = 0; i < g_wmap_n; i++) {
        fprintf(stderr, "[c54x] WMAP   PC=0x%04x n=%-7u @0x%04x(n=%u) distinct=%s%d  min=0x%04x max=0x%04x  ex:",
                g_wmap[i].pc, g_wmap[i].n, g_wmap[i].addr0, g_wmap[i].n0,
                g_wmap[i].nv >= 8 ? ">=" : "", g_wmap[i].nv,
                g_wmap[i].mn, g_wmap[i].mx);
        for (int k = 0; k < g_wmap[i].nv && k < 8; k++)
            fprintf(stderr, " %04x", g_wmap[i].v[k]);
        fprintf(stderr, "%s\n", g_wmap[i].n0 < 2 ? "   <= (trop peu d echantillons)"
                : g_wmap[i].nv == 1 ? "   <= CONSTANTE dans le temps"
                                    : "   <= VARIE dans le temps = PORTE DE LA DONNEE");
    }
}

/* v3 : battement — prouve que la sonde est VIVANTE meme a zero ecriture.
 * Appele sur chaque write hors plage ; n imprime qu une fois par tranche de
 * 5e6 writes globaux, en rappelant le total DANS la plage (0 = vrai zero). */
static void wmap_heartbeat(void)
{
    static uint64_t k;
    if (!g_wmap_on) return;
    if (++k % 5000000ULL) return;
    fprintf(stderr, "[c54x] WMAP heartbeat: writes DSP=%llu, dans plages=%u"
            " (0 = la plage n est jamais ecrite par une instruction DSP)\n",
            (unsigned long long)k, g_wmap_tot);
}

void wmap_note(uint16_t addr, uint16_t val, uint16_t pc)
{
    if (g_wmap_on < 0) {
        const char *e = getenv("CALYPSO_WMAP");
        g_wmap_on = (e && atoi(e) > 0) ? 1 : 0;
        const char *lo = getenv("CALYPSO_WMAP_LO"), *hi = getenv("CALYPSO_WMAP_HI");
        g_wmap_lo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x2c00;
        g_wmap_hi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x2c1f;
        const char *lo2 = getenv("CALYPSO_WMAP_LO2"), *hi2 = getenv("CALYPSO_WMAP_HI2");
        g_wmap_lo2 = lo2 ? (uint16_t)strtoul(lo2, NULL, 0) : 0xffff;
        g_wmap_hi2 = hi2 ? (uint16_t)strtoul(hi2, NULL, 0) : 0x0000;
        if (g_wmap_on)
            fprintf(stderr, "[c54x] WMAP armed 0x%04x..0x%04x\n", g_wmap_lo, g_wmap_hi);
    }
    if (!g_wmap_on) return;
    if (!((addr >= g_wmap_lo  && addr <= g_wmap_hi) ||
          (addr >= g_wmap_lo2 && addr <= g_wmap_hi2))) { wmap_heartbeat(); return; }

    int i;
    for (i = 0; i < g_wmap_n; i++) if (g_wmap[i].pc == pc) break;
    if (i == g_wmap_n) {
        if (g_wmap_n >= WMAP_PCS) return;
        g_wmap_n++;
        g_wmap[i].pc = pc; g_wmap[i].n = 0; g_wmap[i].nv = 0;
        g_wmap[i].mn = 0xffff; g_wmap[i].mx = 0;
        g_wmap[i].addr0 = addr; g_wmap[i].n0 = 0;   /* v2 : cellule temoin figee */
    }
    g_wmap[i].n++;
    if (val < g_wmap[i].mn) g_wmap[i].mn = val;
    if (val > g_wmap[i].mx) g_wmap[i].mx = val;
    /* v2 : le critere de SIGNAL est la variation dans le TEMPS a adresse FIXE. */
    if (addr != g_wmap[i].addr0) return;
    g_wmap[i].n0++;
    if (g_wmap[i].nv < 8) {
        int seen = 0;
        for (int k = 0; k < g_wmap[i].nv; k++) if (g_wmap[i].v[k] == val) { seen = 1; break; }
        if (!seen) g_wmap[i].v[g_wmap[i].nv++] = val;
    }
    /* v3 : seuil bas + tick temporel, pour que l ABSENCE soit mesurable. */
#define WMAP_TICK 2000
    if (++g_wmap_tot % WMAP_TICK == 0) wmap_dump();
}

/* ==== DIO — demod I/O trace   [calypso_c54x.c:3228-3278] ==== */

/* [2026-07-28] DEMODIO : voir en-tete du patch demodio.py. */
static int      g_dio_on = -1;
static uint64_t g_dio_after;
static unsigned g_dio_n;
static uint16_t g_dio_pclo, g_dio_pchi;

static void dio_init(void)
{
    const char *e = getenv("CALYPSO_DEMODIO");
    g_dio_on = (e && atoi(e) > 0) ? 1 : 0;
    const char *a = getenv("CALYPSO_DEMODIO_AFTER");
    g_dio_after = a && *a ? strtoull(a, NULL, 0) : 40000000ULL;
    const char *lo = getenv("CALYPSO_DEMODIO_PCLO"), *hi = getenv("CALYPSO_DEMODIO_PCHI");
    g_dio_pclo = lo ? (uint16_t)strtoul(lo, NULL, 0) : 0x9f95;
    g_dio_pchi = hi ? (uint16_t)strtoul(hi, NULL, 0) : 0x9fe2;
    if (g_dio_on)
        fprintf(stderr, "[c54x] DEMODIO armed PC 0x%04x..0x%04x apres insn=%llu\n",
                g_dio_pclo, g_dio_pchi, (unsigned long long)g_dio_after);
}

void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val)
{
    if (g_dio_on < 0) dio_init();
    if (!g_dio_on) return;
    if (s->pc < g_dio_pclo || s->pc > g_dio_pchi) return;
    if (s->insn_count < g_dio_after) return;
    if (g_dio_n >= 160) return;
    int64_t a = (s->a & 0x8000000000LL) ? (int64_t)(s->a | ~0xFFFFFFFFFFLL) : (int64_t)s->a;
    int64_t b = (s->b & 0x8000000000LL) ? (int64_t)(s->b | ~0xFFFFFFFFFFLL) : (int64_t)s->b;
    /* [2026-07-30] ASM ajoute a la ligne. Motif : la boucle 0x9fab-0x9fb8 fabrique
     * la reference du correlateur dans 0x2a00+ via
     *     0x9fb5  ld  *AR6-0%, TS, A     (A = table(+-1) << T,  T dans [16,31])
     *     0x9fb6  sfta A, +8
     *     0x9fb7  sfta A, -8             (les deux s'annulent)
     *     0x9fb8  sth *AR4+, A, ASM      (0x8694 = la variante ASM, cf. l.10528)
     * et elle n'ecrit que 0x0000 / 0xffff, soit {0, -1} au lieu de {+K, -K} —
     * une reference a |DC|/rms ~ 0,7, ce qui recoupe le 0,60 mesure sur
     * daram_2a00.cfile et explique qu'aucune correlation ne pique.
     * Or « mot haut de (A >> |ASM|) » vaut 0 pour toute magnitude < 2^16 mais
     * 0xffff pour toute valeur negative (extension de signe) : l'asymetrie vient
     * de la, et elle depend entierement de ASM et de T. T etait deja imprime,
     * ASM non — et le seul ASM que j'avais vu (-12) provenait d'une AUTRE sonde a
     * un AUTRE PC (SHADOW-DADST @0xa077), donc d'une autre fenetre : impossible de
     * conclure sans le mesurer ICI. On imprime donc ASM et ST1 bruts. */
    int _asm = s->st1 & 0x1F; if (_asm & 0x10) _asm |= ~0x1F;
    fprintf(stderr, "[c54x] DEMODIO %s PC=0x%04x op=0x%04x addr=0x%04x val=0x%04x "
            "A=%lld B=%lld T=0x%04x ASM=%d ST1=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u\n",
            rw, s->pc, prog_fetch(s, s->pc), addr, val, (long long)a, (long long)b,
            s->t, _asm, s->st1, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
    g_dio_n++;
}
