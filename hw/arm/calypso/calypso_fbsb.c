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
#include <stdio.h>

/* ---------------------------------------------------------------- *
 * Internal: NDB cell access. ndb is the ARM-side dsp_ram[] view
 * (uint16_t *), word-addressed from API base (0x0800 DSP).
 * Address 0x08D4 maps to ndb[(0x08D4 - 0x0800)] = ndb[0x00D4].
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
    return p ? *p : 0;
}

static inline void cell_wr(CalypsoFbsb *s, uint16_t dsp_addr, uint16_t v)
{
    uint16_t *p = cell(s, dsp_addr);
    if (p) *p = v;
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
        /* Re-arm on every FB task write. Publish immediately —
         * prim_fbsb's first poll happens at frame N+2 (cf
         * docs/FBSB_FLOW.md §3), so d_fb_det=1 must already be set
         * by then. Waiting in on_frame_tick was racing the previous
         * cycle's result=255. */
        s->state       = FBSB_FB0_SEARCH;
        s->fb0_attempt = 0;
        s->fb1_attempt = 0;
        s->sb_attempt  = 0;
        s->fn_started  = fn;
        calypso_fbsb_publish_fb_found(s,
            /* toa   */ 0,
            /* pm    */ 80,
            /* angle */ 0,
            /* snr   */ 100);
        s->state = FBSB_FB0_FOUND;
        calypso_fbsb_dump(s, "FB0_FOUND (synth, immediate)");
        break;
    case DSP_TASK_SB:
        /* Synthesize a sync burst immediately so l1s_sbdet_resp finds
         * a_sch[0] with B_SCH_CRC=0 (CRC OK) and a decodable BSIC at
         * a_sch[3..4]. We use bsic=0 (valid placeholder). */
        s->state      = FBSB_SB_SEARCH;
        s->sb_attempt = 0;
        s->fn_started = fn;
        calypso_fbsb_publish_sb_found(s, /* bsic */ 0);
        s->state = FBSB_SB_FOUND;
        calypso_fbsb_dump(s, "SB_FOUND (synth, immediate)");
        break;
    case DSP_TASK_ALLC: {
        /* CCCH read (task=24, ALLC_DSP_TASK).
         *
         * Étape 1 : echo d_task_d/d_burst_d pour passer guard EMPTY de
         * prim_rx_nb.c:73-83 (db_r->d_task_d != 0, db_r->d_burst_d == K).
         *
         * Étape 2 — XXX TEMP HARDCODE replace with bridge intercept :
         * Hardcode d'un payload SI3 fixture (libosmocore tests). Vrai pipeline
         * existe (osmo-bts émet BCCH bursts via TRXD UDP 5702 → bridge.py).
         * Hardcode = stub (règle #1 CLAUDE.md). Documenté TODO.md avec
         * critère de retrait : intercept LAPDm dans bridge.py et push via
         * canal séparé vers QEMU.
         *
         * Critère retrait : bridge.py expose endpoint UDP recevant 23 bytes
         * LAPDm depuis osmo-bts (avant ou après interleaving), QEMU lit
         * dans calypso_fbsb au lieu de la fixture statique. */
        static uint16_t allc_burst_idx = 0;
        static const uint16_t db_r_word_base[2] = { 0x0028, 0x003C };

        /* XXX TEMP HARDCODE — SI3 fixture from libosmocore
         * tests/gb/gprs_bssgp_rim_test.c:376. 21 bytes + 0x2B 0x2B fill
         * = 23 bytes LAPDm UI frame for BCCH/CCCH downlink.
         * À retirer dès intercept bridge.py opérationnel. */
        static const uint8_t si3_blob[23] = {
            0x1b, 0x75, 0x30, 0x00, 0xf1, 0x10, 0x23, 0x6e,
            0xc9, 0x03, 0x3c, 0x27, 0x47, 0x40, 0x79, 0x00,
            0x00, 0x3c, 0x0b, 0x2b, 0x2b, 0x2b, 0x2b
        };

        /* Echo d_task_d / d_burst_d dans les deux read pages */
        for (int p = 0; p < 2; p++) {
            uint16_t *rp = &s->ndb[db_r_word_base[p]];
            rp[0] = DSP_TASK_ALLC;     /* d_task_d = 24 */
            rp[1] = allc_burst_idx;    /* d_burst_d cycling 0..3 */
        }

        /* a_cd[] dans NDB à word offset 0x01D2 (= NDB 0x01A8 + 0x1FC ARM byte).
         *
         * EMPIRIQUE 2026-04-29 nuit : project memory disait NDB+0x1F8 (word
         * 0x01D0) mais runs avec 0x01D0 montrent shift de 2 words côté ARM
         * (data starts at si3_blob[4] au lieu de [0], num_biterr=0x30 au
         * lieu de 0x00). Cause probable : derniers 2 words de a_ramp[16]
         * juste avant. Offset corrigé empiriquement à 0x01D2.
         *
         * Layout :
         *   a_cd[0] : flags. B_BLUD=bit15 (block present). FIRE=bits 5,6 (=0 NO ERROR).
         *   a_cd[1] : info block (skipped par firmware).
         *   a_cd[2] : num_biterr (lower 16 bits).
         *   a_cd[3..14] : 23 bytes LAPDm payload (12 words, dernier word demi-utilisé). */
        {
            uint16_t *acd = &s->ndb[0x01D2];
            acd[0] = 0x8000;           /* B_BLUD set, FIRE bits clear */
            acd[1] = 0x0000;
            acd[2] = 0x0000;           /* num_biterr = 0 */
            for (int i = 0; i < 12; i++) {
                uint16_t lo = (i*2     < 23) ? si3_blob[i*2]     : 0;
                uint16_t hi = (i*2 + 1 < 23) ? si3_blob[i*2 + 1] : 0;
                acd[3 + i] = (uint16_t)(lo | (hi << 8));
            }
        }

        fprintf(stderr, "[fbsb] ALLC echo+SI3 task=24 burst_d=%u fn=%lu\n",
                allc_burst_idx, (unsigned long)fn);
        fflush(stderr);
        allc_burst_idx = (allc_burst_idx + 1) & 3;
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
    case FBSB_FB0_SEARCH:
        /* Should be unreachable: on_dsp_task_change publishes
         * immediately and transitions to FB0_FOUND. Kept as a safety
         * net in case the publish path changes. */
        s->fb0_attempt++;
        break;
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
 * latch values cleared for hygiene. */
extern bool     g_a_sync_valid;
extern uint16_t g_d_fb_det_latch;
extern uint16_t g_d_fb_mode_latch;
extern uint16_t g_a_sync_TOA_latch;
extern uint16_t g_a_sync_PM_latch;
extern uint16_t g_a_sync_ANG_latch;
extern uint16_t g_a_sync_SNR_latch;

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
    cell_wr(s, NDB_D_FB_DET, 1);

    /* Invalidate W1C latches so ARM read returns these fresh values,
     * not stale DSP iter snapshot. */
    invalidate_fbsb_latches();

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

    for (int p = 0; p < 2; p++) {
        uint16_t *rp = &s->ndb[db_r_word_base[p]];
        rp[15] = 0;                       /* a_sch[0] — CRC OK (bit 8 cleared) */
        rp[16] = 0;                       /* a_sch[1] */
        rp[17] = 0;                       /* a_sch[2] */
        rp[18] = (uint16_t)(sb & 0xFFFF); /* a_sch[3] = sb low  */
        rp[19] = (uint16_t)(sb >> 16);    /* a_sch[4] = sb high */
    }

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
