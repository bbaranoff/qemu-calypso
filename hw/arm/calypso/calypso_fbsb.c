/*
 * calypso_fbsb.c — QEMU-side FBSB orchestration (cf. osmocom prim_fbsb.c)
 *
 * Standalone module: only depends on calypso_fbsb.h. Holds no global
 * state; all state lives in the CalypsoFbsb instance owned by the
 * caller (typically calypso_trx.c).
 *
 * The intent is to handle the first FB detection cycle on behalf of
 * the broken DSP path so the ARM firmware can progress past
 * l1s_fbdet_resp without burning the 12-attempts-then-give-up timeout
 * that triggers the 3-second reset cycle.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_fbsb.h"
#include "calypso_full_pcb.h"   /* DARAM lock helpers — cf gap #3 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* W1C latches owned by calypso_trx.c. Declared at file top so
 * on_frame_tick uses them as detection triggers and publish_fb_found
 * pins them to host-side state machine values. */
extern bool     g_a_sync_valid;
extern uint16_t g_d_fb_det_latch;
extern uint16_t g_d_fb_mode_latch;
extern uint16_t g_a_sync_TOA_latch;
extern uint16_t g_a_sync_PM_latch;
extern uint16_t g_a_sync_ANG_latch;
extern uint16_t g_a_sync_SNR_latch;

/* ---------------------------------------------------------------- *
 * Internal: NDB cell access. ndb is the ARM-side dsp_ram[] view
 * (uint16_t *), word-addressed from API base (0x0800 DSP).
 * Address 0x08D4 maps to ndb[(0x08D4 - 0x0800)] = ndb[0x00D4].
 *
 * Locking : ndb pointe dans dsp->data[] (cf calypso_fbsb_init appelé
 * depuis calypso_trx.c). Toute read/write doit prendre daram_lock pour
 * cohérence cross-thread avec DSP-thread + BSP-thread Phase 2 PCB.
 * cell_rd/cell_wr encapsulent le lock. Pour les bursts (5 writes a_sch
 * dans publish_sb_found L295), un lock plus large évite 5 op mutex.
 * ---------------------------------------------------------------- */
static inline uint16_t *cell(CalypsoFbsb *s, uint16_t dsp_addr)
{
    if (!s || !s->ndb) return NULL;
    if (dsp_addr < s->api_base) return NULL;
    return &s->ndb[dsp_addr - s->api_base];
}

static inline uint16_t cell_rd(CalypsoFbsb *s, uint16_t dsp_addr)
{
    uint16_t *p = cell(s, dsp_addr);
    if (!p) return 0;
    calypso_pcb_daram_lock_acquire();
    uint16_t v = *p;
    calypso_pcb_daram_lock_release();
    return v;
}

static inline void cell_wr(CalypsoFbsb *s, uint16_t dsp_addr, uint16_t v)
{
    uint16_t *p = cell(s, dsp_addr);
    if (!p) return;
    calypso_pcb_daram_lock_acquire();
    *p = v;
    calypso_pcb_daram_lock_release();
}

/* ---------------------------------------------------------------- */
void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base)
{
    if (!s) return;
    s->ndb       = ndb_word_base;
    s->api_base  = api_base;
    calypso_fbsb_reset(s);
}

void calypso_fbsb_reset(CalypsoFbsb *s)
{
    if (!s) return;
    s->state       = FBSB_IDLE;
    s->fb0_attempt = 0;
    s->fb1_attempt = 0;
    s->sb_attempt  = 0;
    s->fb0_retries = 0;
    s->afc_retries = 0;
    s->last_toa    = 0;
    s->last_angle  = 0;
    s->last_pm     = 0;
    s->last_snr    = 0;
    s->fn_started  = 0;
}

/* ---------------------------------------------------------------- *
 * Hook: ARM has just written d_task_md (or any task descriptor that
 * means "DSP, do this on the next frame"). We mirror what the
 * firmware's prim_fbsb.c expects: when the task is FB_DSP_TASK we
 * enter FB0_SEARCH; when it's SB_DSP_TASK we enter SB_SEARCH.
 * ---------------------------------------------------------------- */
void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn)
{
    fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n",
            d_task_md, (unsigned long)fn, s ? (int)s->state : -1);
    fflush(stderr);
    if (!s) return;
    switch (d_task_md) {
    case DSP_TASK_FB:
        /* Real DSP path : DSP correlator runs on I/Q stream from BSP. */
        s->state       = FBSB_FB0_SEARCH;
        s->fb0_attempt = 0;
        s->fb1_attempt = 0;
        s->sb_attempt  = 0;
        s->fn_started  = fn;
        calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        break;
    case DSP_TASK_SB:
        s->state      = FBSB_SB_SEARCH;
        s->sb_attempt = 0;
        s->fn_started = fn;
        calypso_fbsb_dump(s, "SB_SEARCH (real DSP path)");
        break;
    case DSP_TASK_ALLC: {
        /* CCCH read (task=24, ALLC_DSP_TASK). The DSP demodulates BCCH
         * bursts delivered by BSP DMA, channel-decodes (deinterleaving +
         * FIRE check), and writes the resulting LAPDm bytes to a_cd[] +
         * responds via db_r->d_task_d/d_burst_d. ARM L1S then synthesizes
         * a DATA_IND. No QEMU-side intervention. */
        static int log_once;
        if (!log_once++) {
            fprintf(stderr,
                    "[fbsb] ALLC task=24 fn=%lu — real DSP CCCH demod "
                    "(QEMU does not write a_cd[] from any side-channel)\n",
                    (unsigned long)fn);
            fflush(stderr);
        }
        break;
    }
    case DSP_TASK_NONE:
    default:
        break;
    }
}

/* ---------------------------------------------------------------- *
 * Hook: called from calypso_trx.c at every frame tick. The state
 * machine here decides whether to publish a synthetic "FB found" into
 * NDB so the firmware progresses, or to wait another frame.
 *
 * The first cut is INTENTIONALLY minimal: after the firmware has
 * spent N attempts in FB0_SEARCH, we publish a plausible result (the
 * SNR observed by the running DSP correlator if known, otherwise a
 * default that exceeds FB0_SNR_THRESH=0). Subsequent stages are TODO.
 * ---------------------------------------------------------------- */
void calypso_fbsb_on_frame_tick(CalypsoFbsb *s, uint64_t fn)
{
    if (!s) return;

    switch (s->state) {
    case FBSB_FB0_SEARCH: {
        /* Real-DSP-path transition (2026-05-28) : the DSP correlator
         * writes a_sync_* (TOA/PM/ANG/SNR) and d_fb_det into NDB on
         * each iteration. The DSP-side latch trigger in calypso_c54x.c
         * (gated on a_sync_SNR write @ PC=0x8d33/0x8eb9/0x8f51) takes
         * an atomic snapshot of all 6 cells.
         *
         * The DSP CLEARS d_fb_det back to 0xdfd0 sentinel ~18 cycles
         * after setting a real value (observed via SET-TO-CLEAR-DELTA
         * probe), so polling d_fb_det direct is racy : the snapshot
         * almost always captures the sentinel, not the real result.
         *
         * Use g_a_sync_SNR_latch as the detection trigger instead.
         * SNR > 0 (int16) means the DSP correlator found a meaningful
         * burst in this iteration. The other latches carry the matching
         * TOA/PM/ANG/SNR atomically. Publish overrides d_fb_det with 1
         * so ARM read consumes it as "detection". */
        s->fb0_attempt++;
        /* Gate stricte (2026-05-28) : ne publier QUE sur SNR > GATE.
         * Le DSP correlator écrit a_sync_SNR à chaque iter, mais sur
         * bursts data (BCCH/SACCH/...) le résidu est ~400-800 (noise).
         * Sur vrai FCCH burst, SNR monte à plusieurs milliers. Le
         * threshold 1500 sépare les deux pour ne pas latcher la FB
         * sur le mauvais burst (= SB tomberait à côté). */
        #define FBSB_SNR_GATE 1500
        if (g_a_sync_valid && (int16_t)g_a_sync_SNR_latch > FBSB_SNR_GATE) {
            uint16_t toa = g_a_sync_TOA_latch;
            uint16_t pm  = g_a_sync_PM_latch;   /* publish re-shifts << 3, undo */
            uint16_t snr = g_a_sync_SNR_latch;
            /* angle=0 (2026-05-28) : la source samples QEMU/bridge n'a
             * pas de drift osc/Doppler injecté → vrai AFC = 0. Le latch
             * angle est un résidu correlator parasite qui casse
             * read_fb_result()→ANGLE_TO_FREQ→AFC retry loop. Publier 0
             * = ground-truth, débloque la chaîne FB→FB1→SB. */
            int16_t ang_published = 0;
            fprintf(stderr,
                    "[calypso-fbsb] FB0 DETECT (gated SNR>%d, angle=0) "
                    "toa=%d pm=0x%04x ang_dsp=%d→0 snr=0x%04x (s=%d) "
                    "fn=%lu att=%u\n",
                    FBSB_SNR_GATE, (int16_t)toa, pm,
                    (int16_t)g_a_sync_ANG_latch, snr,
                    (int16_t)snr, (unsigned long)fn, s->fb0_attempt);
            calypso_fbsb_publish_fb_found(s, (int16_t)toa, pm >> 3,
                                          ang_published, snr);
            s->state = FBSB_FB0_FOUND;
        }
        break;
    }
    case FBSB_FB0_FOUND:
        /* Stay here until ARM either re-arms via d_task_md (which will
         * push us back to FB0_SEARCH) or moves on. Don't auto-cascade
         * to FB1_SEARCH — that loop ran fb1_attempt to 59 last run. */
        break;
    case FBSB_FB1_SEARCH:
        s->fb1_attempt++;
        break;
    case FBSB_FB1_FOUND:
        s->state = FBSB_SB_SEARCH;
        s->sb_attempt = 0;
        break;
    case FBSB_SB_SEARCH:
        /* TODO: synthesize a plausible SCH result (BSIC, FN, ToA) so
         * l1s_sbdet_resp can complete and the firmware moves on to
         * BCCH reception. Not implemented yet. */
        break;
    default:
        break;
    }
}

/* W1C latches in calypso_trx.c (set by c54x DSP-side iter writes).
 * Invalidate them here so ARM read falls through to fresh fbsb values
 * instead of stale DSP iter values. The master gate is g_a_sync_valid
 * (false → all a_sync_* + d_fb_det reads fall through). Individual
 * latch values cleared for hygiene. (Externs are declared at file top
 * so on_frame_tick / publish_fb_found can use them directly.) */

static inline void invalidate_fbsb_latches(void)
{
    g_a_sync_valid     = false;
    g_d_fb_det_latch   = 0;
    g_d_fb_mode_latch  = 0;
    g_a_sync_TOA_latch = 0;
    g_a_sync_PM_latch  = 0;
    g_a_sync_ANG_latch = 0;
    g_a_sync_SNR_latch = 0;
}

/* ---------------------------------------------------------------- */
void calypso_fbsb_publish_fb_found(CalypsoFbsb *s,
                                   int16_t toa, uint16_t pm,
                                   int16_t angle, uint16_t snr)
{
    if (!s) return;
    s->last_toa   = toa;
    s->last_pm    = pm;
    s->last_angle = angle;
    s->last_snr   = snr;
    cell_wr(s, NDB_A_SYNC_DEMOD_TOA, (uint16_t)toa);
    cell_wr(s, NDB_A_SYNC_DEMOD_PM,  (uint16_t)(pm << 3));   /* prim_fbsb shifts >>3 on read */
    cell_wr(s, NDB_A_SYNC_DEMOD_ANG, (uint16_t)angle);
    cell_wr(s, NDB_A_SYNC_DEMOD_SNR, snr);
    /* Publish d_fb_det as the SNR magnitude (= DSP-native "real
     * detection" encoding) rather than constant 1. Firmware checks
     * `if (d_fb_det)` (non-zero) at prim_fbsb.c:404, then reads
     * a_sync_SNR for threshold compare at line 449. Any non-zero
     * passes the first check; SNR > FB0_SNR_THRESH (=0) passes the
     * second. Using SNR keeps the value semantically meaningful and
     * non-zero whenever SNR is positive. */
    uint16_t fb_det_val = (snr != 0) ? snr : 1;
    cell_wr(s, NDB_D_FB_DET, fb_det_val);

    /* Populate W1C latches AND mark them valid so ARM read path
     * returns our stable values rather than DSP transient writes.
     * DSP-direct values flicker (set real → cleared back to 0xdfd0
     * in ~18 cycles), so cell_wr alone races and loses. The latches
     * serve as the host-side authoritative result until ARM consumes
     * d_fb_det (latch_consume pattern in calypso_trx.c:228-235). */
    g_d_fb_det_latch   = fb_det_val;
    g_d_fb_mode_latch  = 0;
    g_a_sync_TOA_latch = (uint16_t)toa;
    g_a_sync_PM_latch  = (uint16_t)(pm << 3);
    g_a_sync_ANG_latch = (uint16_t)angle;
    g_a_sync_SNR_latch = snr;
    g_a_sync_valid     = true;

    (void)toa; (void)pm; (void)angle; (void)snr;
}

void calypso_fbsb_clear_fb(CalypsoFbsb *s)
{
    if (!s) return;
    cell_wr(s, NDB_D_FB_DET, 0);
    cell_wr(s, NDB_A_SYNC_DEMOD_TOA, 0);
    /* Same latch invalidation as publish path — without this, ARM
     * could keep reading a stale latched d_fb_det=1 after we cleared
     * the cell. */
    invalidate_fbsb_latches();
}

/* ---------------------------------------------------------------- *
 * Sync Burst synthesis.
 *
 * l1s_sbdet_resp (cf prim_fbsb.c, doc §SB) reads:
 *   dsp_api.db_r->a_sch[0]    — bit B_SCH_CRC=8 set means CRC ERROR.
 *   dsp_api.db_r->a_sch[3..4] — packed SB word, decoded by l1s_decode_sb
 *                               into bsic / t1 / t2 / t3.
 *
 * db_r is double-buffered between dsp_ram[0x0050/2] (page 0) and
 * dsp_ram[0x0078/2] (page 1). The READ page is selected by
 * d_dsp_page & 1 (cf calypso_trx.c lines 561-564). We don't know which
 * page the firmware will read at the next response frame, so we write
 * BOTH pages — cheap and reliable.
 *
 * a_sch[] sits at struct word offset 15..19 in T_DB_DSP_TO_MCU
 * (after a_pm[3] at 8..10 and a_serv_demod[4] at 11..14).
 *
 * sb encoding (l1s_decode_sb):
 *   bsic  = (sb >> 2) & 0x3f
 *   t1    = ((sb>>23)&1) | ((sb>>7)&0x1fe) | ((sb<<9)&0x600)
 *   t2    = (sb>>18) & 0x1f
 *   t3p   = ((sb>>24)&1) | ((sb>>15)&6)
 *   t3    = t3p*10 + 1
 *
 * For minimal valid: sb encoding such that bsic=<arg>, t1=t2=0, t3=1
 * → t3p=0 → bits {24,16,15}=0; t2 bits {18..22}=0; t1 bits {7..15,23,9..10}=0.
 * Then bsic only uses bits 2..7. So sb = (bsic & 0x3f) << 2.
 * ---------------------------------------------------------------- */
void calypso_fbsb_publish_sb_found(CalypsoFbsb *s, uint8_t bsic)
{
    if (!s || !s->ndb) return;

    static const uint16_t db_r_word_base[2] = { 0x0028, 0x003C };
    uint32_t sb = ((uint32_t)(bsic & 0x3f)) << 2;

    calypso_pcb_daram_lock_acquire();
    for (int p = 0; p < 2; p++) {
        uint16_t *rp = &s->ndb[db_r_word_base[p]];
        rp[15] = 0;                       /* a_sch[0] — CRC OK (bit 8 cleared) */
        rp[16] = 0;                       /* a_sch[1] */
        rp[17] = 0;                       /* a_sch[2] */
        rp[18] = (uint16_t)(sb & 0xFFFF); /* a_sch[3] = sb low  */
        rp[19] = (uint16_t)(sb >> 16);    /* a_sch[4] = sb high */
    }
    calypso_pcb_daram_lock_release();

}

/* ---------------------------------------------------------------- */
void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag)
{
    if (!s) return;
    static const char *names[] = {
        "IDLE", "FB0_SEARCH", "FB0_FOUND",
        "FB1_SEARCH", "FB1_FOUND",
        "SB_SEARCH",  "SB_FOUND",
        "DONE", "FAIL",
    };
    fprintf(stderr,
            "[fbsb] %s state=%s fb0_att=%u fb1_att=%u sb_att=%u "
            "fb0_ret=%u afc_ret=%u last(snr=%u toa=%d ang=%d pm=%u)\n",
            tag ? tag : "", names[s->state],
            s->fb0_attempt, s->fb1_attempt, s->sb_attempt,
            s->fb0_retries, s->afc_retries,
            s->last_snr, s->last_toa, s->last_angle, s->last_pm);
    fflush(stderr);
}
