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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>

/* ---------------------------------------------------------------- *
 * MMAP SI consumer (cf. doc/MMAP_SI_FORMAT.md v1).
 * Lit /dev/shm/calypso_si.bin (ou $CALYPSO_SI_MMAP_PATH si défini).
 * Layout : 16-byte header + 5×32-byte slots = 176 bytes total.
 * Magic 4 bytes "CSI1" = 0x31495343 LE.
 * Slots fixés : 0=SI1, 1=SI2, 2=SI3, 3=SI4, 4=SI13.
 * Si fichier absent / magic invalide / slot vide → no SI written
 * (caller skips a_cd write, mobile fails FBSB cleanly).
 * ---------------------------------------------------------------- */
#define CSI_MAGIC          0x31495343u  /* "CSI1" LE uint32 */
#define CSI_HEADER_SIZE    16
#define CSI_SLOT_SIZE      32
#define CSI_SLOT_COUNT     5
#define CSI_FILE_SIZE      (CSI_HEADER_SIZE + CSI_SLOT_COUNT * CSI_SLOT_SIZE)

#define CSI_SLOT_VALID     (1u << 0)

#define CSI_SLOT_SI1   0
#define CSI_SLOT_SI2   1
#define CSI_SLOT_SI3   2
#define CSI_SLOT_SI4   3
#define CSI_SLOT_SI13  4

static struct {
    int   fd;
    void *map;
    int   state;   /* 0=uninit, 1=ready, -1=disabled */
} g_csi = { -1, NULL, 0 };

static void csi_init_once(void)
{
    if (g_csi.state != 0) return;

    const char *path = getenv("CALYPSO_SI_MMAP_PATH");
    if (!path) path = "/dev/shm/calypso_si.bin";

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "[fbsb] CSI mmap: open(%s) failed: %s — SI writes disabled\n",
                path, strerror(errno));
        g_csi.state = -1;
        return;
    }
    struct stat st;
    if (fstat(fd, &st) < 0 || st.st_size < (off_t)CSI_FILE_SIZE) {
        fprintf(stderr, "[fbsb] CSI mmap: %s too small (%lld < %d) — SI writes disabled\n",
                path, (long long)st.st_size, (int)CSI_FILE_SIZE);
        close(fd);
        g_csi.state = -1;
        return;
    }
    void *map = mmap(NULL, CSI_FILE_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        fprintf(stderr, "[fbsb] CSI mmap: mmap failed: %s — SI writes disabled\n", strerror(errno));
        close(fd);
        g_csi.state = -1;
        return;
    }
    uint32_t magic;
    memcpy(&magic, map, 4);
    if (magic != CSI_MAGIC) {
        fprintf(stderr, "[fbsb] CSI mmap: bad magic 0x%08x (expected 0x%08x) — SI writes disabled\n",
                magic, CSI_MAGIC);
        munmap(map, CSI_FILE_SIZE);
        close(fd);
        g_csi.state = -1;
        return;
    }
    g_csi.fd = fd;
    g_csi.map = map;
    g_csi.state = 1;
    const uint8_t *h = (const uint8_t *)map;
    fprintf(stderr, "[fbsb] CSI mmap: ready (%s, ver=%u, slots=%u)\n",
            path, h[4], h[5]);
    fflush(stderr);
}

/* TS 44.018 §3.4 table 1 — TC → SI mapping for normal BCCH multiframe.
 * SI3 émis 3× par cycle (TC=2,4,6) car critique pour cell selection. */
static const uint8_t tc_to_slot[8] = {
    CSI_SLOT_SI1,   /* TC=0 */
    CSI_SLOT_SI2,   /* TC=1 */
    CSI_SLOT_SI3,   /* TC=2 */
    CSI_SLOT_SI4,   /* TC=3 */
    CSI_SLOT_SI3,   /* TC=4 */
    CSI_SLOT_SI2,   /* TC=5 */
    CSI_SLOT_SI3,   /* TC=6 */
    CSI_SLOT_SI4    /* TC=7 */
};

/* Returns pointer to 23-byte SI blob for given TC, or NULL if mmap unavailable
 * or slot empty/invalid. Caller skips a_cd write when NULL. */
static const uint8_t *csi_lookup_for_tc(uint8_t tc, uint8_t *out_si_type)
{
    if (g_csi.state != 1) return NULL;
    uint8_t slot_idx = tc_to_slot[tc & 7];
    const uint8_t *slot = (const uint8_t *)g_csi.map + CSI_HEADER_SIZE +
                          slot_idx * CSI_SLOT_SIZE;
    if (slot[0] == 0x00) return NULL;
    if (!(slot[1] & CSI_SLOT_VALID)) return NULL;
    if (slot[2] != 23) return NULL;
    if (out_si_type) *out_si_type = slot[0];
    return &slot[4];
}

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
/* CALYPSO_FBSB_SYNTH=1 → publish synthetic FB/SB results in NDB so the
 * firmware progresses past FBSB without waiting for the DSP correlator
 * to converge. Documented dev-assist for cases where the emulated DSP
 * fb-det doesn't converge on bridge-fed GMSK samples. Default 0 = real
 * DSP path. Read once at first call. */
static int fbsb_synth_mode(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("CALYPSO_FBSB_SYNTH");
        cached = (e && *e == '1') ? 1 : 0;
        fprintf(stderr, "[calypso-fbsb] CALYPSO_FBSB_SYNTH=%d (%s path)\n",
                cached, cached ? "synth, dev-assist" : "real DSP");
        fflush(stderr);
    }
    return cached;
}

/* CALYPSO_BCCH_INJECT=1 → in DSP_TASK_ALLC, echo db_r->d_task_d/d_burst_d
 * (passes prim_rx_nb EMPTY guard) AND inject SI bytes from mmap into
 * a_cd[]. Default 0 = real DSP CCCH demod path (DSP itself writes a_cd
 * after channel decoding the BCCH bursts the BSP delivered). The real
 * path doesn't currently work in QEMU (DSP CCCH demod not converging),
 * so opt-in is required to see SIs reach mobile L3.
 *
 * Pair with CALYPSO_FBSB_SYNTH=1 for the full DL chain to work end-to-end. */
static int bcch_inject_mode(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("CALYPSO_BCCH_INJECT");
        cached = (e && *e == '1') ? 1 : 0;
        fprintf(stderr, "[calypso-fbsb] CALYPSO_BCCH_INJECT=%d (%s)\n",
                cached, cached ? "db_r echo + a_cd mmap inject"
                               : "real DSP CCCH demod");
        fflush(stderr);
    }
    return cached;
}

void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn)
{
    fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n",
            d_task_md, (unsigned long)fn, s ? (int)s->state : -1);
    fflush(stderr);
    if (!s) return;
    int synth = fbsb_synth_mode();
    switch (d_task_md) {
    case DSP_TASK_FB:
        /* The DSP runs the real fb-det routine on the I/Q stream that
         * the BSP feeds from osmo-bts-trx (GMSK-modulated). Convergence
         * depends on the routine running fully each frame — see
         * doc/FBSB_FLOW.md and the icount/wall-clock notes in run.sh.
         * If CALYPSO_FBSB_SYNTH=1, publish synthetic results to bypass
         * the (currently non-converging) emulated correlator. */
        s->state       = FBSB_FB0_SEARCH;
        s->fb0_attempt = 0;
        s->fb1_attempt = 0;
        s->sb_attempt  = 0;
        s->fn_started  = fn;
        if (synth) {
            calypso_fbsb_publish_fb_found(s,
                /* toa */ 0, /* pm */ 80, /* angle */ 0, /* snr */ 100);
            s->state = FBSB_FB0_FOUND;
            calypso_fbsb_dump(s, "FB0_FOUND (synth)");
        } else {
            calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        }
        break;
    case DSP_TASK_SB:
        s->state      = FBSB_SB_SEARCH;
        s->sb_attempt = 0;
        s->fn_started = fn;
        if (synth) {
            calypso_fbsb_publish_sb_found(s, /* bsic */ 0);
            s->state = FBSB_SB_FOUND;
            calypso_fbsb_dump(s, "SB_FOUND (synth)");
        } else {
            calypso_fbsb_dump(s, "SB_SEARCH (real DSP path)");
        }
        break;
    case DSP_TASK_ALLC: {
        /* CCCH read (task=24, ALLC_DSP_TASK).
         *
         * Real DSP path : the DSP demodulates BCCH bursts delivered by
         * BSP DMA, channel-decodes (deinterleaving + FIRE check), and
         * writes the resulting LAPDm bytes to a_cd[] + responds via
         * db_r->d_task_d/d_burst_d. ARM L1S then synthesizes a DATA_IND.
         *
         * Currently the emulated DSP CCCH demod does not converge on the
         * bridge-fed GMSK samples (same family of problem as fb-det). The
         * env-gated bypass below short-circuits this :
         *   - echo db_r->d_task_d/d_burst_d to pass the EMPTY guard in
         *     prim_rx_nb.c:73-83 (`db_r->d_task_d != 0, db_r->d_burst_d == K`)
         *   - write a_cd[] from /dev/shm/calypso_si.bin (rsl_si_tap.py mmap),
         *     selecting SI type per TC (BCCH multiframe slot pattern)
         *
         * Default 0 = real DSP path (broken in QEMU but pure).
         * CALYPSO_BCCH_INJECT=1 = bypass active. */
        if (!bcch_inject_mode()) {
            static int log_once;
            if (!log_once++) {
                fprintf(stderr,
                        "[fbsb] ALLC task=24 fn=%lu — real DSP path "
                        "(set CALYPSO_BCCH_INJECT=1 to inject SIs from mmap)\n",
                        (unsigned long)fn);
                fflush(stderr);
            }
            break;
        }

        static const uint16_t db_r_word_base[2] = { 0x0028, 0x003C };

        /* d_burst_d = block index 0..3 inside the 4-burst BCCH block.
         * In a 51-multiframe, BCCH bursts occupy fn%51 in {2,3,4,5}, so
         * the burst index inside that block is (fn-2)%4 — but the simpler,
         * lockstep-safe expression is fn & 3 (block alignment is what
         * matters here, not absolute slot). */
        uint16_t burst_d = (uint16_t)(fn & 3);

        /* Echo d_task_d / d_burst_d dans les deux read pages */
        for (int p = 0; p < 2; p++) {
            uint16_t *rp = &s->ndb[db_r_word_base[p]];
            rp[0] = DSP_TASK_ALLC;
            rp[1] = burst_d;
        }

        /* Sélection SI selon TC (BCCH multiframe slot pattern) */
        csi_init_once();
        uint8_t tc = (uint8_t)((fn / 51) & 7);
        uint8_t si_type = 0;
        const uint8_t *blob = csi_lookup_for_tc(tc, &si_type);

        if (blob) {
            /* a_cd[] dans NDB à word offset 0x01D2.
             * Layout :
             *   a_cd[0] : flags B_BLUD=bit15 + FIRE bits 5,6 (=0 NO ERROR)
             *   a_cd[1] : info block (firmware skip)
             *   a_cd[2] : num_biterr
             *   a_cd[3..14] : 23 bytes LAPDm BCCH payload */
            uint16_t *acd = &s->ndb[0x01D2];
            acd[0] = 0x8000;
            acd[1] = 0x0000;
            acd[2] = 0x0000;
            for (int i = 0; i < 12; i++) {
                uint16_t lo = (i*2     < 23) ? blob[i*2]     : 0;
                uint16_t hi = (i*2 + 1 < 23) ? blob[i*2 + 1] : 0;
                acd[3 + i] = (uint16_t)(lo | (hi << 8));
            }
        }

        fprintf(stderr,
                "[fbsb] ALLC task=24 burst_d=%u fn=%lu TC=%u si_type=0x%02x %s\n",
                burst_d, (unsigned long)fn, tc, si_type,
                blob ? "(mmap inject)" : "(no SI — mmap unavailable)");
        fflush(stderr);
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
