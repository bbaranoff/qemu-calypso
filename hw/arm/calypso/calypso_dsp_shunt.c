/*
 * calypso_dsp_shunt.c — DSP-side mock honoring the ARM↔DSP API-RAM contract.
 *
 * When CALYPSO_DSP_SHUNT=1, the c54x emulator is skipped entirely (no opcode
 * execution, no INTM gymnastics, no DARAM-side compute). This file replaces
 * the DSP by a thin state machine that respects the only protocol the ARM
 * firmware actually sees:
 *
 *   1. ARM writes a task descriptor into W_PAGE_(w_page) — d_task_d /
 *      d_task_md / d_task_ra / d_burst_d / d_fn / ...
 *   2. ARM signals "go" by writing 0xFFD001A8 (NDB+0 = d_dsp_page) with
 *      bit 1 (B_GSM_TASK) set; bit 0 carries the page index.
 *   3. DSP (= us) consumes the task, computes the result, writes:
 *        - FB result into NDB:  d_fb_det @+0x48, a_sync_demod[4] @+0x4C
 *        - SB result into R_PAGE_(page_idx): a_sch[5] @ +0x1E, a_serv_demod
 *          [4] @ +0x10
 *      then the result is visible at the NEXT TDMA frame.
 *   4. No separate "DSP done" IRQ: the TPU FRAME IRQ (INTH bit 4) ticks
 *      every 1ms and the ARM polls there.
 *
 * Design notes (review by c-web 2026-05-26):
 *   - Latch on write to NDB+0, but SERVICE on the next FRAME IRQ tick.
 *     This respects the ARM firmware's poste-then-wait-frame model and
 *     gives multi-frame tasks (FB search) a natural cadence.
 *   - Disjoint write surfaces: FB goes to NDB only, SB goes to READ PAGE
 *     only. The fw's read sites (prim_fbsb.c:181/198/306/404) are the
 *     ground truth.
 *   - Offsets are DWARF-validated against THE container ELF
 *     (/opt/GSM/firmware/board/compal_e88/layer1.highram.elf — sha256
 *     27cd04...). NOT the host build — the container build was the one
 *     loaded by run.sh `-kernel`. Same offsets confirmed across both.
 *   - Canned phase 1 = dispatch each post on next FRAME IRQ. No
 *     simulated wide→narrow FB search; angle=0 keeps AFC loop from
 *     iterating. TOA tuned so synchronize_tdma yields bits_delta≈0.
 *   - ALLC/NB UL/RA UL = LOG_UNIMP. We don't need them to clear
 *     FBSB_CONF — those are downstream of the current wall.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"
#include "sysemu/dma.h"
#include "qemu/main-loop.h"
#include "calypso_dsp_shunt.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>

/* ---- Memory map (ARM-side addresses, from osmocom-bb dsp_api.h:18-23) ---- */
#define BASE_API_W_PAGE_0   0xFFD00000UL  /* 20 words MCU→DSP page 0 */
#define BASE_API_W_PAGE_1   0xFFD00028UL  /* 20 words MCU→DSP page 1 */
#define BASE_API_R_PAGE_0   0xFFD00050UL  /* 20 words DSP→MCU page 0 */
#define BASE_API_R_PAGE_1   0xFFD00078UL  /* 20 words DSP→MCU page 1 */
#define BASE_API_NDB        0xFFD001A8UL  /* 268 words persistent NDB */

/* ---- Write page (T_DB_MCU_TO_DSP) field offsets (DWARF-validated) ---- */
#define WP_D_TASK_D         0x00
#define WP_D_BURST_D        0x02
#define WP_D_TASK_U         0x04
#define WP_D_BURST_U        0x06
#define WP_D_TASK_MD        0x08
#define WP_D_TASK_RA        0x0E
#define WP_D_FN             0x10
#define WP_D_CTRL_SYSTEM    0x20

/* ---- Read page (T_DB_DSP_TO_MCU) field offsets ---- */
#define RP_D_TASK_D         0x00
#define RP_D_BURST_D        0x02
#define RP_D_TASK_MD        0x08
#define RP_A_SERV_DEMOD     0x10   /* [4] words: {D_TOA,D_PM,D_ANGLE,D_SNR} */
#define RP_A_PM             0x18   /* [3] words */
#define RP_A_SCH            0x1E   /* [5] words: SB header+info */

/* ---- NDB (T_NDB_MCU_DSP) field offsets ---- */
#define NDB_D_DSP_PAGE      0x00
#define NDB_D_ERROR_STATUS  0x02
#define NDB_D_FB_DET        0x48
#define NDB_D_FB_MODE       0x4A
#define NDB_A_SYNC_DEMOD    0x4C   /* [4] words */
#define NDB_A_CD            0x1FC  /* a_cd[15] : CCCH demod result.
                                       FIX 2026-06-02 : 0x1DC → 0x1FC (retour à la
                                       valeur DWARF autoritaire). Le DWARF de
                                       layer1.highram.elf donne offsetof(T_NDB_MCU_DSP,
                                       a_cd)=0x1FC (vérifié gdb : d_fb_det=0x48→ARM
                                       0x1F0 ✓ cohérent FORCE_TOA, a_sync_demod=0x4C ✓).
                                       Avec 0x1DC le firmware lisait num_biterr=0xff +
                                       CRC fail (a_cd écrit À CÔTÉ → SI3 canned jamais
                                       atteint). Le "FIX 2026-05-28 0x1FC→0x1DC" était
                                       faux (validé sur un autre build, pas cet ELF). */
#define NDB_A_SCH26         0x54   /* [5] words */

/* ---- l1_environment.h constants ---- */
#define B_GSM_PAGE          (1 << 0)
#define B_GSM_TASK          (1 << 1)
#define B_SCH_CRC           8

#define PM_DSP_TASK         1     /* power measurement (l1s pm_cmd) — lit a_pm */
#define FB_DSP_TASK         5
#define SB_DSP_TASK         6
#define ALLC_DSP_TASK       24

#define D_TOA               0
#define D_PM                1
#define D_ANGLE             2
#define D_SNR               3

/* ---- pending-task state ---- */
struct dsp_shunt_state {
    bool       active;                /* CALYPSO_DSP_SHUNT=1 */
    AddressSpace *as;                 /* ARM AS to peek/poke API RAM */
    /* latched task awaiting dispatch on next FRAME IRQ tick */
    bool       pending;
    uint8_t    page_idx;              /* 0 or 1 (B_GSM_PAGE) */
    uint16_t   d_task_md;             /* FB=5, SB=6, ... */
    uint16_t   d_task_d;              /* NB DL tasks */
    uint16_t   d_task_u;              /* NB UL */
    uint16_t   d_task_ra;             /* RACH */
    uint16_t   d_burst_d;
    uint16_t   d_fn;
    uint32_t   tick_cnt;              /* FRAME IRQ ticks since shunt enabled */
    /* SI réel injecté (gr-gsm ou démod C native) via calypso_dsp_shunt_feed_si.
     * Si si_valid, shunt_dispatch_allc écrit si_buf dans a_cd au lieu du canned. */
    uint8_t    si_buf[23];
    bool       si_valid;
    /* (A) Set SI COMPLET : un buffer par type (SI1/2/3/4/2bis/2ter). Sinon le
     * mobile ne reçoit qu'UN type → "No sysinfo yet" → sync timeout. On tourne
     * au début de chaque bloc (burst 0) pour tenir un type STABLE sur les 4
     * bursts (a_cd mono-frame ; sinon frame incohérente → CRC fail). */
    uint8_t    si_set[6][23];
    bool       si_set_have[6];
    int        si_rr;                 /* index round-robin du dernier type servi */
    /* Resultat de sync REEL poste par gr-gsm (= le DSP) via UDP SCH (4731,
     * shunt_sch_read). Remplace SHUNT_CANNED_BSIC dans shunt_dispatch_sb. */
    uint8_t    sb_bsic;               /* BSIC reel = ncc<<3|bcc (decode_sch gr-gsm) */
    uint32_t   sb_fn;                 /* FN reelle du SCH */
    bool       sb_valid;              /* gr-gsm a poste au moins un SCH reel */
};

/* FN TDMA reelle (calypso_trx.c) pour recoder la FN du shunt (LATCH d_fn=0). */
extern uint32_t calypso_trx_get_fn(void);
extern void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn);

static struct dsp_shunt_state g_shunt;

/* ---- Helpers : read/write API RAM via AddressSpace (16-bit LE) ---- */
static inline uint16_t shunt_read_w(uint32_t addr)
{
    uint16_t v = 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le16_to_cpu(v);
}

static inline void shunt_write_w(uint32_t addr, uint16_t v)
{
    uint16_t le = cpu_to_le16(v);
    dma_memory_write(g_shunt.as, addr, &le, sizeof(le), MEMTXATTRS_UNSPECIFIED);
}

static inline uint32_t wp_base(uint8_t page_idx) {
    return page_idx ? BASE_API_W_PAGE_1 : BASE_API_W_PAGE_0;
}
static inline uint32_t rp_base(uint8_t page_idx) {
    return page_idx ? BASE_API_R_PAGE_1 : BASE_API_R_PAGE_0;
}

static void shunt_dispatch_pm(uint8_t page_idx);   /* fwd : appelé depuis le latch */
static void shunt_poll_si_shm(void);                /* fwd : poll SI shm (gr-gsm→a_cd) */

/* ---- LATCH : called on ARM write to NDB+0 (d_dsp_page) ---- */
static void shunt_latch_task(uint16_t new_d_dsp_page)
{
    if (!(new_d_dsp_page & B_GSM_TASK)) {
        return; /* not a real task signal (might be d_dsp_page=0 reset) */
    }

    uint8_t  page_idx = (new_d_dsp_page & B_GSM_PAGE) ? 1 : 0;
    uint32_t wp       = wp_base(page_idx);

    g_shunt.page_idx  = page_idx;
    g_shunt.d_task_d  = shunt_read_w(wp + WP_D_TASK_D);
    g_shunt.d_burst_d = shunt_read_w(wp + WP_D_BURST_D);
    g_shunt.d_task_u  = shunt_read_w(wp + WP_D_TASK_U);
    g_shunt.d_task_md = shunt_read_w(wp + WP_D_TASK_MD);
    g_shunt.d_task_ra = shunt_read_w(wp + WP_D_TASK_RA);
    g_shunt.d_fn      = shunt_read_w(wp + WP_D_FN);
    /* RECODE FN (#4) : le firmware poste souvent d_fn=0 (FBSB = recherche, pas
     * de frame precise). On substitue la VRAIE FN TDMA pour le frame_nr aval
     * (DATA_IND / sync). */
    if (g_shunt.d_fn == 0)
        g_shunt.d_fn = (uint16_t)(calypso_trx_get_fn() & 0xFFFF);
    g_shunt.pending   = true;

    /* PM : valeur statique, écrite IMMÉDIATEMENT (pas de service déféré au
     * prochain frame IRQ). Sinon le firmware lit a_pm AVANT le dispatch déféré
     * → 0 stale → rxlev=-110. On écrit a_pm sur la page lue tout de suite. */
    if (g_shunt.d_task_md == PM_DSP_TASK)
        shunt_dispatch_pm(page_idx);

    fprintf(stderr,
        "[dsp-shunt] LATCH page=%u task_md=%u task_d=%u task_ra=%u fn=%u\n",
        page_idx, g_shunt.d_task_md, g_shunt.d_task_d,
        g_shunt.d_task_ra, g_shunt.d_fn);
}

/* ---- Canned tuning ----
 *
 * TOA target : prim_fbsb.c does `last_fb->toa -= 23` then derives ntdma/qbits.
 * Picking raw TOA=23 yields ntdma=0, qbits=0 → "perfectly on time", which
 * sidesteps the "DSP reports SB in bit that is N bits in the future" guard
 * and the `time_alignment` becomes 0 (clean baseline for synchronize_tdma).
 *
 * PM is shifted (>>3) by read_fb_result / read_sb_result. 0x7000 raw → 0xE00
 * after the shift, well above any AFC/threshold.
 *
 * SNR is read raw and compared against AFC_SNR_THRESHOLD. 0x7000 clears it
 * easily.
 *
 * ANGLE = 0 → ANGLE_TO_FREQ(0) = 0 → AFC correction null → the loop does
 * not re-iterate looking for AFC convergence (c-web's caution about
 * the AFC loop spinning if angle is non-zero but unchanged).
 *
 * BSIC = 63 (max, matches osmo-bsc.cfg default `base_station_id_code 63`).
 * t1=t2=t3=0 in encoded sb → l1s_decode_sb yields time->fn = 0 (seeds the
 * mobile's FN-counter at zero, which is FN-agnostic for canned dispatch).
 * Real FN coherence is a Phase 2 problem.
 */
#define SHUNT_CANNED_TOA     23     /* raw → "on time" after -23 */
#define SHUNT_CANNED_PM      0x7000
#define SHUNT_CANNED_SNR     0x7000
#define SHUNT_CANNED_ANGLE   0
#define SHUNT_CANNED_BSIC    63

/* Pack {bsic, t1, t2, t3} into 32-bit sb (inverse of prim_fbsb.c:125-144). */
static uint32_t shunt_encode_sb(uint8_t bsic, uint16_t t1, uint8_t t2, uint8_t t3)
{
    uint8_t t3p = (t3 == 0) ? 0 : ((t3 - 1) / 10);
    uint32_t sb = 0;
    sb |= ((uint32_t)(bsic & 0x3f)) << 2;
    sb |= ((uint32_t)(t1 & 0x001)) << 23;
    sb |= ((uint32_t)(t1 & 0x1fe)) << 7;
    sb |= ((uint32_t)(t1 & 0x600)) >> 9;
    sb |= ((uint32_t)(t2 & 0x1f))  << 18;
    sb |= ((uint32_t)(t3p & 1))    << 24;
    sb |= ((uint32_t)(t3p & 6))    << 15;
    return sb;
}

/* ---- DISPATCH : FB writes NDB only ---- */
static void shunt_dispatch_fb(uint8_t page_idx)
{
    /* d_fb_det = 1 ("FOUND"). prim_fbsb.c:404 reads this from NDB. */
    shunt_write_w(BASE_API_NDB + NDB_D_FB_DET, 1);

    /* a_sync_demod[4] @ NDB+0x4C, 4 consecutive 16-bit words. Read by
     * read_fb_result (prim_fbsb.c:306-309) from NDB. */
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_TOA   * 2, SHUNT_CANNED_TOA);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_PM    * 2, SHUNT_CANNED_PM);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_ANGLE * 2, SHUNT_CANNED_ANGLE);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);

    /* Ack on the read page (echo). Not strictly required for the FB path
     * (firmware reads d_fb_det from NDB, not read-page) but mirrors the
     * real DSP's task-completion echo. */
    shunt_write_w(rp_base(page_idx) + RP_D_TASK_MD, FB_DSP_TASK);

    fprintf(stderr,
        "[dsp-shunt] DISPATCH FB page=%u → d_fb_det=1 TOA=%d PM=0x%x "
        "ANGLE=%d SNR=0x%x (NDB only)\n",
        page_idx, SHUNT_CANNED_TOA, SHUNT_CANNED_PM,
        SHUNT_CANNED_ANGLE, SHUNT_CANNED_SNR);
}

/* ---- DISPATCH : SB writes READ PAGE only ---- */
static void shunt_dispatch_sb(uint8_t page_idx)
{
    uint32_t rp = rp_base(page_idx);

    /* gr-gsm (= le DSP) a-t-il poste un vrai SCH (BSIC/FN reels via UDP 4731) ?
     * En mode no-canned (full-grgsm), tant qu'aucun SCH reel n'est arrive on ne
     * dispatch PAS le SB : le firmware FBSB attend le vrai SCH, comme un vrai
     * mobile. Pas de BSIC canne -> aucun masquage d'echec de decode. */
    static int no_canned = -1;
    if (no_canned < 0) {
        const char *e = getenv("CALYPSO_SHUNT_NO_CANNED");
        no_canned = (e && *e == '1') ? 1 : 0;
    }
    if (!g_shunt.sb_valid && no_canned) {
        static unsigned waitlog = 0;
        if (waitlog++ < 10)
            fprintf(stderr, "[dsp-shunt] SB: pas encore de SCH reel (gr-gsm) "
                    "-> pas de dispatch (no-canned, le firmware attend)\n");
        return;
    }

    /* BSIC/FN : REELS (gr-gsm decode_sch) si dispo, sinon canned (legacy only).
     * FN -> {t1,t2,t3} GSM : T1=FN/(26*51), T2=FN%26, T3=FN%51 (encode_sb derive T3'). */
    uint8_t  bsic = g_shunt.sb_valid ? g_shunt.sb_bsic : SHUNT_CANNED_BSIC;
    uint32_t fn   = g_shunt.sb_valid ? g_shunt.sb_fn   : 0;
    uint16_t t1   = (uint16_t)(fn / (26u * 51u));
    uint8_t  t2   = (uint8_t)(fn % 26u);
    uint8_t  t3   = (uint8_t)(fn % 51u);

    /* a_sch[0] CRC bit clear = success (prim_fbsb.c:181, B_SCH_CRC=8). */
    shunt_write_w(rp + RP_A_SCH + 0 * 2, 0x0000);

    /* sb = encode_sb(bsic, t1, t2, t3) → a_sch[3] | a_sch[4]<<16
     * (prim_fbsb.c:198). Two separate 16-bit stores, both LE. */
    uint32_t sb = shunt_encode_sb(bsic, t1, t2, t3);
    shunt_write_w(rp + RP_A_SCH + 3 * 2, (uint16_t)(sb & 0xFFFF));
    shunt_write_w(rp + RP_A_SCH + 4 * 2, (uint16_t)(sb >> 16));

    /* a_sch[1] / a_sch[2] are unused by l1s_decode_sb; zero them. */
    shunt_write_w(rp + RP_A_SCH + 1 * 2, 0x0000);
    shunt_write_w(rp + RP_A_SCH + 2 * 2, 0x0000);

    /* a_serv_demod[4] @ +0x10. read_sb_result reads from READ PAGE here,
     * NOT NDB (prim_fbsb.c:148-151). Same canned tuning as FB. */
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, SHUNT_CANNED_TOA);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2, SHUNT_CANNED_PM);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, SHUNT_CANNED_ANGLE);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);

    /* Ack on read page. */
    shunt_write_w(rp + RP_D_TASK_MD, SB_DSP_TASK);

    fprintf(stderr,
        "[dsp-shunt] DISPATCH SB page=%u → sb=0x%08x BSIC=%u FN=%u %s TOA=%d\n",
        page_idx, sb, bsic, fn,
        g_shunt.sb_valid ? "(gr-gsm REEL)" : "(canned legacy)", SHUNT_CANNED_TOA);
}

/* Canned SI3 bytes — 23 L2-frame bytes (RR PD + SI3 mt + payload).
 * Format conforme a osmocom-bb prim_rx_nb.c:154 :
 *   dsp_memcpy_from_api(rxnb.di->data, &dsp_api.ndb->a_cd[3], 23, 0);
 * Donc a_cd[0..2] = STATUS (CRC, biterr), a_cd[3..14] = 23B L2 frame.
 *
 * Layout L2+L3 RR SI3 :
 *   [0]=0x49 LI=18 EL=1   [1]=0x06 RR PD   [2]=0x1B SI3 mt
 *   [3..4]=Cell ID
 *   [5..7]=MCC/MNC encoded (0x00 0xF1 0x10 = MCC 001 MNC 01)
 *   [8..9]=LAC
 *   [10..11]=cell options + cell select
 *   [12..14]=RACH ctrl
 *   [15..22] = padding 0x2B */
/* SHUNT_CANNED_SI3_L2 RETIRÉ (no-hack 2026-06-03) : le SI vient
 * UNIQUEMENT du vrai décode grgsm (g_shunt.si_buf via feed_si). */

static void shunt_dispatch_allc(uint8_t page_idx)
{
    /* a_cd layout (cf osmocom-bb prim_rx_nb.c) :
     *   a_cd[0]   = FIRE status bits (B_FIRE0/B_FIRE1) -> 0x0000 = CRC pass
     *   a_cd[1]   = (reserved / BLUD bit)              -> 0x0000
     *   a_cd[2]   = num_biterr                          -> 0x0000
     *   a_cd[3..14] = 23 bytes L2 frame (SI3 here)
     */
    uint32_t addr_a_cd = BASE_API_NDB + NDB_A_CD;

    /* "sans hack" : CALYPSO_SHUNT_NO_CANNED=1 → on n'injecte JAMAIS le SI3
     * canned. Tant que le démod réel (bridge gr-gsm via feed_si) n'a rien
     * livré (si_valid=0), on ne dispatch rien → le firmware bail (pas de
     * DATA_IND) → le mobile ne campe QUE sur le VRAI SI décodé de l'I/Q du
     * BTS. C'est ça qui rend la victoire non-truquée : si le démod casse,
     * rien ne campe (le bug est visible, pas masqué par le canned). */
    static int no_canned = -1;
    if (no_canned < 0) {
        const char *e = getenv("CALYPSO_SHUNT_NO_CANNED");
        no_canned = (e && *e == '1') ? 1 : 0;
    }
    if (no_canned && !g_shunt.si_valid)
        return;

    /* (A) ROTATION par bloc : au début du bloc (burst 0) on avance au prochain
     * type SI disponible et on le copie dans si_buf (STABLE pour les 4 bursts).
     * Le mobile collecte ainsi TOUT le set (SI1/2/3/4) au fil des blocs au lieu
     * du seul SI3. Round-robin = aucune dépendance FN (jitter-proof). */
    if (g_shunt.d_burst_d == 0) {
        for (int k = 1; k <= 6; k++) {
            int s = (g_shunt.si_rr + k) % 6;
            if (g_shunt.si_set_have[s]) {
                memcpy(g_shunt.si_buf, g_shunt.si_set[s], 23);
                g_shunt.si_rr = s;
                break;
            }
        }
    }

    /* #12 ORDONNANCEMENT BCCH (no-hack) : présenter le SI UNIQUEMENT sur les
     * blocs BCCH du multiframe-51 (TC = fn%51 ∈ [2,5]). Sur un bloc CCCH le SI3
     * fuiterait en PCH/AGCH ("Unknown PCH/AGCH message"). d_fn = vraie FN (#4).
     * Gated CALYPSO_SHUNT_BCCH_SCHED (défaut 1). */
    static int bcch_sched = -1, bcch_ofs = 0;
    if (bcch_sched < 0) {
        const char *e = getenv("CALYPSO_SHUNT_BCCH_SCHED");
        bcch_sched = (e && *e == '1') ? 1 : 0;        /* DEFAUT OFF (chan_nr pas le gate du camping) */
        const char *o = getenv("CALYPSO_SHUNT_BCCH_OFS");
        bcch_ofs = o ? atoi(o) : 0;
    }
    if (bcch_sched) {
        /* FN = le device (vraie FN GSM de la BTS, alignée mf-51), PAS d_fn
         * (que le firmware laisse à 0). Bloc BCCH non-combiné C0T0 = TC ∈ [2,5]
         * (FCCH@0/10/.., SCH@1/11/.., BCCH@2-5, CCCH@6-9/12-15..). Offset
         * réglable CALYPSO_SHUNT_BCCH_OFS si l'alignement dispatch≠bloc. */
        static unsigned long n_disp = 0, n_bcch = 0, n_since_bcch = 0;
        int tc = (int)((((long)calypso_trx_get_fn() + bcch_ofs) % 51 + 51) % 51);
        int is_bcch = (tc >= 2 && tc <= 5);
        n_disp++;
        if (is_bcch) { n_bcch++; n_since_bcch = 0; } else n_since_bcch++;
        if ((n_disp % 51) == 0)
            fprintf(stderr, "[dsp-shunt] #12 BCCH-sched: %lu disp / %lu BCCH "
                    "(tc=%d ofs=%d)\n", n_disp, n_bcch, tc, bcch_ofs);
        /* Garde anti-famine : grace au boot (200 disp) + si 0 BCCH depuis 102
         * dispatches (désalignement total) on présente quand même → dégrade
         * vers "SI partout" au lieu de famine totale. */
        if (!is_bcch && n_disp > 200 && n_since_bcch < 102) {
            uint32_t addr0 = BASE_API_NDB + NDB_A_CD;
            uint32_t rp_c  = rp_base(page_idx);
            shunt_write_w(addr0 + 0, 0x0003);          /* a_cd[0] FIRE = CRC fail */
            shunt_write_w(rp_c + RP_D_TASK_D,  ALLC_DSP_TASK);
            shunt_write_w(rp_c + RP_D_BURST_D, g_shunt.d_burst_d);
            return;                          /* pas de SI sur le CCCH */
        }
    }

    /* a_cd[0..2] = status words = 0 (CRC pass, no biterr) */
    shunt_write_w(addr_a_cd + 0, 0x0000);  /* a_cd[0] */
    shunt_write_w(addr_a_cd + 2, 0x0000);  /* a_cd[1] */
    shunt_write_w(addr_a_cd + 4, 0x0000);  /* a_cd[2] */

    /* a_cd[3..14] = 23B L2 frame, packé en 12 mots LE.
     * Source : le SI RÉEL démodulé (gr-gsm ou C natif via feed_si) si dispo,
     * sinon le SI3 canned (fallback). C'est le swap canned→réel = le "sans hack". */
    const uint8_t *si = g_shunt.si_buf;  /* no-hack : vrai SI grgsm seulement */
    for (int i = 0; i < 23; i += 2) {
        uint8_t lo = si[i];
        uint8_t hi = (i + 1 < 23) ? si[i + 1] : 0x2B;
        uint16_t w = lo | (hi << 8);
        shunt_write_w(addr_a_cd + 6 + i, w);   /* +6 = a_cd[3] base */
    }

    /* IMPORTANT : firmware prim_rx_nb.c:79 fait
     *   if (db_r->d_burst_d != burst_id) return 0;
     * et attend la sequence burst 0,1,2,3 pour assembler la frame.
     * On echo le d_burst_d que l'ARM a poste dans la read page pour que
     * le check passe. Sinon le firmware bail avant dsp_memcpy_from_api()
     * et n'envoie JAMAIS L1CTL_DATA_IND. */
    uint32_t rp = rp_base(page_idx);
    shunt_write_w(rp + RP_D_TASK_D,  ALLC_DSP_TASK);
    shunt_write_w(rp + RP_D_BURST_D, g_shunt.d_burst_d);

    /* a_serv_demod[4] = {TOA, PM, ANGLE, SNR} per-burst measurements.
     * Firmware prim_rx_nb.c:89-94 reads these. Canned : TOA=23, PM=high,
     * ANGLE=0 (AFC converged), SNR=high (passes AFC_SNR_THRESHOLD). */
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, 23);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2, 0x7000);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2, 0x7000);

    fprintf(stderr,
        "[dsp-shunt] DISPATCH ALLC page=%u burst_d=%u -> SI3 in a_cd[3..14] + "
        "a_serv_demod canned\n", page_idx, g_shunt.d_burst_d);
}

/* ---- DISPATCH PM : tâche power-measurement (md=1). Écrit a_pm[3] @ +0x18,
 * que le power scan (l1s pm_cmd) lit pour dériver le rxlev. Sans ça a_pm=0 →
 * rxlev=-110 (plancher) → la cellule est rejetée AVANT même la sync, quel que
 * soit le SI. Valeur réglable via CALYPSO_SHUNT_PM (défaut SHUNT_CANNED_PM,
 * haut → rxlev fort). C'est le pendant "scan" du PM canné FB/SB. ---- */
static void shunt_dispatch_pm(uint8_t page_idx)
{
    uint32_t rp = rp_base(page_idx);
    static int pm_val = -1;
    if (pm_val < 0) {
        const char *e = getenv("CALYPSO_SHUNT_PM");
        pm_val = (e && *e) ? (int)strtol(e, NULL, 0) : SHUNT_CANNED_PM;
    }
    shunt_write_w(rp + RP_A_PM + 0 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_A_PM + 1 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_A_PM + 2 * 2, (uint16_t)pm_val);
    shunt_write_w(rp + RP_D_TASK_MD, PM_DSP_TASK);
    static unsigned pm_log = 0;
    if (pm_log++ < 5)
        fprintf(stderr,
                "[dsp-shunt] DISPATCH PM page=%u → a_pm[0..2]=0x%04x (rxlev)\n",
                page_idx, (uint16_t)pm_val);
}

static void shunt_dispatch_nb(uint8_t page_idx, uint16_t task_d)
{
    /* TODO : NB DL = decoded BCCH/CCCH burst payload into NDB a_cd[].
     * NB UL = consume burst bits from DARAM for TX (forwarded to bridge). */
    fprintf(stderr,
        "[dsp-shunt] DISPATCH NB page=%u task_d=%u (TODO)\n",
        page_idx, task_d);
}

/* ---- Service hook : called from calypso_trx frame_irq tick ---- */
void calypso_dsp_shunt_on_frame_tick(void)
{
    if (!g_shunt.active)
        return;
    shunt_poll_si_shm();   /* gr-gsm a-t-il ecrit un nouveau SI dans le shm ? */
    if (!g_shunt.pending) {
        return;
    }
    g_shunt.tick_cnt++;

    uint8_t  page = g_shunt.page_idx;
    uint16_t md   = g_shunt.d_task_md;
    uint16_t td   = g_shunt.d_task_d;

    /* Priority order: md tasks (FB/SB) > NB DL > NB UL > ALLC.
     * Refine when canned policies land. */
    if (md == PM_DSP_TASK) {
        shunt_dispatch_pm(page);
    } else if (md == FB_DSP_TASK) {
        shunt_dispatch_fb(page);
    } else if (md == SB_DSP_TASK) {
        shunt_dispatch_sb(page);
    } else if (td == ALLC_DSP_TASK) {
        shunt_dispatch_allc(page);
    } else if (td != 0) {
        shunt_dispatch_nb(page, td);
    }
    /* RA UL (d_task_ra) handled separately — TBD when TX flow gated */

    /* Mock task done. Real DSP would keep its state for multi-attempt
     * tasks (FB search across 11 frames). Phase 1 canned can keep the
     * pending bit set for FB until d_fb_det is consumed (zeroed by ARM
     * in read_fb_result @ prim_fbsb.c:318). */
    g_shunt.pending = false;
}

/* ---- MMIO overlay on NDB+0 (d_dsp_page trigger) ---- */
static void shunt_d_dsp_page_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    /* Write also commits the value in the underlying RAM region; we
     * intercept here for the latch side-effect only. Caller's write
     * happens via the normal RAM path (this overlay is registered with
     * higher priority but pass-through semantics). */
    shunt_latch_task((uint16_t)value);
}

static uint64_t shunt_d_dsp_page_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    /* Read passes through to RAM — ARM polls this for handshake state.
     * We return the actual RAM value to be transparent. */
    return shunt_read_w(BASE_API_NDB + NDB_D_DSP_PAGE);
}

static const MemoryRegionOps shunt_ndb_trigger_ops = {
    .read  = shunt_d_dsp_page_read,
    .write = shunt_d_dsp_page_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl  = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- GSMTAP listener : reçoit le SI décodé par gr-gsm (front py) ----
 * gr-gsm (grgsm_decode -m BCCH) sort des frames GSMTAP. On écoute sur un port
 * UDP (CALYPSO_SHUNT_GSMTAP_PORT, défaut 4730 pour ne pas taper le 4729 du
 * BTS), on extrait le L2 (après le hdr GSMTAP de 16 o) des frames BCCH, et on
 * appelle feed_si → a_cd. = le pont gr-gsm→a_cd, côté qemu. */
#define GSMTAP_HDR_LEN          16
#define GSMTAP_TYPE_UM          0x01
#define GSMTAP_CHANNEL_BCCH     0x01
static int g_gsmtap_fd = -1;

static void shunt_gsmtap_read(void *opaque)
{
    uint8_t buf[512];
    for (;;) {
        ssize_t n = recv(g_gsmtap_fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            break;
        }
        if (n < GSMTAP_HDR_LEN + 1)
            continue;
        uint8_t type     = buf[2];   /* GSMTAP hdr : type @2 */
        uint8_t sub_type = buf[12];  /* channel @12 */
        if (type != GSMTAP_TYPE_UM || sub_type != GSMTAP_CHANNEL_BCCH)
            continue;                /* on ne prend que le BCCH (les SI) */
        /* (A) On accepte le SET BCCH COMPLET (SI1/2/3/4/2bis/2ter), pas juste
         * SI3 — sinon le mobile n'a jamais le set complet ("No sysinfo yet").
         * feed_si range chaque type dans son slot, dispatch_allc tourne dessus.
         * On EXCLUT paging/IMM-ASS (mt=0x21 etc.) qui ne va pas sur le slot BCCH.
         * L2 = buf+16 : [0]=pseudo-len, [1]=PD, [2]=message type. */
        if (n < GSMTAP_HDR_LEN + 3 || buf[GSMTAP_HDR_LEN + 1] != 0x06)
            continue;                               /* pas RR PD */
        switch (buf[GSMTAP_HDR_LEN + 2]) {
        case 0x19: case 0x1a: case 0x1b:            /* SI1 SI2 SI3 */
        case 0x1c: case 0x1d: case 0x1e:            /* SI4 SI2bis SI2ter */
            break;
        default:
            continue;                               /* paging, IMM-ASS, SI13... : drop */
        }
        calypso_dsp_shunt_feed_si(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
    }
}

static void shunt_gsmtap_init(void)
{
    const char *p = getenv("CALYPSO_SHUNT_GSMTAP_PORT");
    int port = (p && *p) ? atoi(p) : 4730;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        error_report("[dsp-shunt] GSMTAP socket() failed: %s", strerror(errno));
        return;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        error_report("[dsp-shunt] GSMTAP bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_gsmtap_fd = fd;
    qemu_set_fd_handler(fd, shunt_gsmtap_read, NULL, NULL);
    error_report("[dsp-shunt] GSMTAP listener udp:127.0.0.1:%d → feed_si(a_cd) "
                 "(gr-gsm grgsm_decode -m BCCH y envoie le SI réel)", port);
}

/* ---- SCH listener : recoit le BSIC/FN REELS decodes par gr-gsm (= le DSP) ----
 * grgsm_relay_decode.py forwarde le tuple ('sch',bsic,fn) du port `measurements`
 * de gsm.receiver en UDP {magic 'SCH1', int32 bsic, int32 fn, LE} sur ce port
 * (CALYPSO_SHUNT_SCH_PORT, defaut 4731 — distinct du GSMTAP 4730). On stocke le
 * resultat -> shunt_dispatch_sb encode le VRAI BSIC/FN au lieu de
 * SHUNT_CANNED_BSIC. C'est le "DSP qui poste son decode SCH dans le NDB". */
static int g_sch_fd = -1;

static void shunt_sch_read(void *opaque)
{
    uint8_t buf[64];
    for (;;) {
        ssize_t n = recv(g_sch_fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            break;
        }
        if (n < 12 || memcmp(buf, "SCH1", 4) != 0)
            continue;                       /* pas notre datagramme */
        uint32_t bsic_le, fn_le;
        memcpy(&bsic_le, buf + 4, 4);
        memcpy(&fn_le,   buf + 8, 4);
        int32_t bsic = (int32_t)le32_to_cpu(bsic_le);
        int32_t fn   = (int32_t)le32_to_cpu(fn_le);
        bool first = !g_shunt.sb_valid;
        g_shunt.sb_bsic  = (uint8_t)(bsic & 0x3f);
        g_shunt.sb_fn    = (uint32_t)fn;
        g_shunt.sb_valid = true;
        static unsigned schlog = 0;
        if (first || schlog++ < 20 || (schlog % 200) == 0)
            fprintf(stderr, "[dsp-shunt] SCH reel (gr-gsm): BSIC=%d "
                    "(ncc=%d bcc=%d) FN=%d%s\n", (int)g_shunt.sb_bsic,
                    (g_shunt.sb_bsic >> 3) & 7, g_shunt.sb_bsic & 7,
                    (int)fn, first ? " [1er]" : "");
    }
}

static void shunt_sch_init(void)
{
    const char *p = getenv("CALYPSO_SHUNT_SCH_PORT");
    int port = (p && *p) ? atoi(p) : 4731;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        error_report("[dsp-shunt] SCH socket() failed: %s", strerror(errno));
        return;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        error_report("[dsp-shunt] SCH bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_sch_fd = fd;
    qemu_set_fd_handler(fd, shunt_sch_read, NULL, NULL);
    error_report("[dsp-shunt] SCH listener udp:127.0.0.1:%d → feed_sb(BSIC/FN reels "
                 "gr-gsm) → shunt_dispatch_sb (remplace SHUNT_CANNED_BSIC)", port);
}

/* ========================================================================
 * Buffers partages (shm) — gr-gsm AU MILIEU du shunt DSP (pas de FIFO/UDP).
 *   ENTREE du DSP shunte : l'I/Q que la BSP livre (DARAM 0x2a00) est recopiee
 *     ici via calypso_dsp_shunt_feed_iq() ; gr-gsm la LIT.
 *   SORTIE du DSP shunte : gr-gsm ECRIT le SI decode ; le shunt le LIT au frame
 *     tick (shunt_poll_si_shm) et le pousse dans a_cd -> l'ARM le lit.
 * Semantique BUFFER (pas fifo) : un compteur de sequence par sens ; le lecteur
 * poll le seq et ne consomme que s'il a change. shm POSIX /calypso_dsp_shunt.
 * ====================================================================== */
#define SHM_NAME      "/calypso_dsp_shunt"
#define SHM_IQ_SLOTS  64           /* ring de bursts (absorbe les stalls du decode gr-gsm) */
#define SHM_IQ_LEN    320          /* int16 par slot (>= 296 = 148 complexes cs16) */

struct shm_iq_slot {
    uint32_t fn;                   /* frame number du burst */
    uint32_t n;                    /* nb d'int16 valides (I,Q entrelaces) */
    int16_t  iq[SHM_IQ_LEN];
};

struct dsp_shunt_shm {
    uint32_t magic;                /* 0x43445350 = 'CDSP' */
    /* --- ENTREE : ring de bursts I/Q (shunt ecrit <- BSP, gr-gsm lit) --- */
    volatile uint32_t iq_wr;       /* nb total de bursts ecrits (compteur write) */
    struct shm_iq_slot iq[SHM_IQ_SLOTS];
    /* --- SORTIE : SI decode (gr-gsm ecrit, shunt lit -> a_cd) --- */
    volatile uint32_t si_seq;      /* bumpe a chaque nouveau SI decode */
    uint32_t          si_len;      /* octets L2 (<=23) */
    uint8_t           si[32];
};

static struct dsp_shunt_shm *g_shm;
static uint32_t              g_shm_last_si_seq;
static FILE                 *g_iq_cfile;   /* enreg .cfile fc32 de l'I/Q d'entree */

static void shunt_shm_init(void)
{
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        error_report("[dsp-shunt] shm_open(%s): %s", SHM_NAME, strerror(errno));
        return;
    }
    if (ftruncate(fd, sizeof(struct dsp_shunt_shm)) != 0) {
        error_report("[dsp-shunt] ftruncate shm: %s", strerror(errno));
        close(fd);
        return;
    }
    void *m = mmap(NULL, sizeof(struct dsp_shunt_shm),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) {
        error_report("[dsp-shunt] mmap shm: %s", strerror(errno));
        return;
    }
    g_shm = m;
    g_shm->magic = 0x43445350;
    g_shm_last_si_seq = g_shm->si_seq;
    error_report("[dsp-shunt] shm %s (=/dev/shm%s, %zu o) : I/Q in (feed_iq->gr-gsm) "
                 "+ SI out (gr-gsm->a_cd). gr-gsm AU MILIEU du shunt.",
                 SHM_NAME, SHM_NAME, sizeof(struct dsp_shunt_shm));

    /* Enregistrement .cfile (gr_complex fc32 I,Q normalise) de l'I/Q d'entree
     * du DSP shunte, pour rejeu deterministe (grgsm_cfile_decode.py). Defaut
     * /tmp/dsp_iq.cfile ; CALYPSO_SHUNT_IQ_CFILE= (vide) pour desactiver. */
    const char *cf = getenv("CALYPSO_SHUNT_IQ_CFILE");
    if (!cf)
        cf = "/tmp/dsp_iq.cfile";
    if (*cf) {
        g_iq_cfile = fopen(cf, "wb");
        if (g_iq_cfile)
            error_report("[dsp-shunt] enregistre l'I/Q d'entree -> %s (cfile fc32)", cf);
        else
            error_report("[dsp-shunt] fopen(%s) cfile: %s", cf, strerror(errno));
    }
}

/* ENTREE du DSP shunte : la BSP appelle ceci avec l'I/Q DL (cs16, n int16
 * entrelaces I,Q) qu'elle DMA dans la DARAM. Publie dans le shm pour gr-gsm. */
void calypso_dsp_shunt_feed_iq(uint32_t fn, const int16_t *iq, int n)
{
    if (!g_shm || !iq || n <= 0)
        return;
    if (n > SHM_IQ_LEN)
        n = SHM_IQ_LEN;
    struct shm_iq_slot *slot = &g_shm->iq[g_shm->iq_wr % SHM_IQ_SLOTS];
    slot->fn = fn;
    slot->n  = (uint32_t)n;
    memcpy(slot->iq, iq, (size_t)n * sizeof(int16_t));
    __sync_synchronize();
    g_shm->iq_wr++;               /* publie le burst (le lecteur poll iq_wr) */

    /* Enregistre aussi l'I/Q en .cfile fc32 (rejeu deterministe). */
    if (g_iq_cfile) {
        float fbuf[SHM_IQ_LEN];
        for (int i = 0; i < n; i++)
            fbuf[i] = (float)iq[i] / 32768.0f;
        fwrite(fbuf, sizeof(float), (size_t)n, g_iq_cfile);
    }
}

/* SORTIE du DSP shunte : gr-gsm a-t-il ecrit un nouveau SI ? Si oui -> a_cd. */
static void shunt_poll_si_shm(void)
{
    if (!g_shm)
        return;
    uint32_t seq = g_shm->si_seq;
    if (seq == g_shm_last_si_seq)
        return;
    __sync_synchronize();
    g_shm_last_si_seq = seq;
    uint32_t len = g_shm->si_len;
    if (len == 0 || len > sizeof(g_shm->si))
        return;
    calypso_dsp_shunt_feed_si(g_shm->si, (int)len);
}

/* ---- init : called from machine setup when CALYPSO_DSP_SHUNT=1 ---- */
void calypso_dsp_shunt_init(MemoryRegion *system_memory, AddressSpace *as)
{
    const char *env = getenv("CALYPSO_DSP_SHUNT");
    if (!env || strcmp(env, "1") != 0) {
        g_shunt.active = false;
        return;
    }

    g_shunt.active = true;
    g_shunt.as     = as;
    g_shunt.pending = false;
    g_shunt.tick_cnt = 0;

    /* Overlay the single d_dsp_page word as IO. The rest of the API RAM
     * stays as plain RAM that the firmware reads/writes directly. */
    MemoryRegion *trigger = g_new0(MemoryRegion, 1);
    memory_region_init_io(trigger, NULL, &shunt_ndb_trigger_ops, NULL,
                          "calypso-dsp-shunt-trigger", 2);
    memory_region_add_subregion_overlap(system_memory,
                                        BASE_API_NDB + NDB_D_DSP_PAGE,
                                        trigger,
                                        /*priority=*/10);

    /* Pont gr-gsm → a_cd : écoute le SI décodé (GSMTAP) et l'injecte. */
    shunt_gsmtap_init();

    /* Pont gr-gsm → SB : écoute le BSIC/FN réels (SCH) et les injecte dans
     * shunt_dispatch_sb (remplace SHUNT_CANNED_BSIC). */
    shunt_sch_init();

    /* Buffers shm : gr-gsm au milieu du shunt (I/Q in + SI out, pas de fifo). */
    shunt_shm_init();

    error_report("[dsp-shunt] active — c54x emulator should be skipped, "
                 "BSP DMA→DARAM should be gated. Phase 1: canned dispatch "
                 "TODO. Watch /tmp/qemu.log for LATCH/DISPATCH lines.");
}

/* Phase-2 hook (IPC integration) — calypso-ipc-device will call this with
 * the result of GMSK demod from osmo-trx-ipc instead of canned values. */
void calypso_dsp_shunt_feed_fb_result(int found, int16_t toa,
                                      int16_t pm, int16_t angle, int16_t snr)
{
    /* TODO Phase 2 */
    (void)found; (void)toa; (void)pm; (void)angle; (void)snr;
}

/* Point d'injection COMMUN du SI réel (2026-06-02) : gr-gsm (via pont) OU la
 * démod C native appellent ceci avec une frame L2 de 23 octets décodée depuis
 * l'I/Q réel du BTS. Le shunt l'écrit ensuite dans a_cd (shunt_dispatch_allc)
 * à la place du SI3 canned → "sans hack", vrai signal. len doit être 23 (XCCH
 * L2). Réécrit à chaque nouveau SI (rotation SI1/2/3/4 du BCCH). */
void calypso_dsp_shunt_feed_si(const uint8_t *l2, int len)
{
    if (!l2 || len <= 0) {
        g_shunt.si_valid = false;
        return;
    }
    int n = len < 23 ? len : 23;
    /* (A) range la frame dans le slot de SON type (RR PD=0x06, mt=l2[2]) :
     *   SI1=0x19 SI2=0x1a SI3=0x1b SI4=0x1c SI2bis=0x1d SI2ter=0x1e.
     * shunt_dispatch_allc tourne ensuite sur les slots dispo (set complet). */
    int slot = -1;
    if (n >= 3 && l2[1] == 0x06) {
        switch (l2[2]) {
        case 0x19: slot = 0; break;  /* SI1   */
        case 0x1a: slot = 1; break;  /* SI2   */
        case 0x1b: slot = 2; break;  /* SI3   */
        case 0x1c: slot = 3; break;  /* SI4   */
        case 0x1d: slot = 4; break;  /* SI2bis*/
        case 0x1e: slot = 5; break;  /* SI2ter*/
        default:   break;
        }
    }
    if (slot >= 0) {
        memcpy(g_shunt.si_set[slot], l2, n);
        for (int i = n; i < 23; i++) g_shunt.si_set[slot][i] = 0x2B;
        g_shunt.si_set_have[slot] = true;
    }
    /* compat / fallback : si_buf = dernier reçu */
    memcpy(g_shunt.si_buf, l2, n);
    /* pad fin avec 0x2B (filler LAPDm) si la frame est plus courte */
    for (int i = n; i < 23; i++)
        g_shunt.si_buf[i] = 0x2B;
    g_shunt.si_valid = true;
    /* Hop 5 : injecte AUSSI directement en L1CTL DATA_IND -> mobile (gated
     * CALYPSO_SHUNT_DL_INJECT, defaut ON). FN reelle via calypso_trx_get_fn. */
    {
        static int inj = -1;
        if (inj < 0) { const char *e = getenv("CALYPSO_SHUNT_DL_INJECT");
                       inj = (e && *e == '0') ? 0 : 1; }
        if (inj) l1ctl_inject_dl_si(g_shunt.si_buf, 23, calypso_trx_get_fn());
    }
    fprintf(stderr, "[dsp-shunt] feed_si: SI réel %d o injecté → a_cd "
            "(L2[0..2]=%02x %02x %02x)\n", n, l2[0],
            n > 1 ? l2[1] : 0, n > 2 ? l2[2] : 0);
}

/* Public getter — gate condition for BSP/TPU DMA into DARAM. */
bool calypso_dsp_shunt_active(void)
{
    return g_shunt.active;
}
