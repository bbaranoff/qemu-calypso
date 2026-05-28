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
#include "calypso_dsp_shunt.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
#define NDB_A_CD            0x1DC  /* a_cd[15] : CCCH demod result.
                                       FIX 2026-05-28 : 0x1FC → 0x1DC.
                                       Validated end-to-end via DSP L1 stub (scripts/
                                       make_dsp_bin_L1.py) : stub écrit SI3 LAPDm bytes
                                       à NDB+0x1DC, mobile firmware decode et accepte les SI.
                                       Confirme DWARF 2026-05-26 (CLAUDE.md). L'empirique
                                       précédent à 0x1FC observait probablement un autre
                                       struct variant (a_cd doublé ou aliasé), pas le slot
                                       que la firmware ARM lit réellement. */
#define NDB_A_SCH26         0x54   /* [5] words */

/* ---- l1_environment.h constants ---- */
#define B_GSM_PAGE          (1 << 0)
#define B_GSM_TASK          (1 << 1)
#define B_SCH_CRC           8

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
};

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
    g_shunt.pending   = true;

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

    /* a_sch[0] CRC bit clear = success (prim_fbsb.c:181, B_SCH_CRC=8). */
    shunt_write_w(rp + RP_A_SCH + 0 * 2, 0x0000);

    /* sb = encode_sb(bsic, t1=0, t2=0, t3=0) → a_sch[3] | a_sch[4]<<16
     * (prim_fbsb.c:198). Two separate 16-bit stores, both LE. */
    uint32_t sb = shunt_encode_sb(SHUNT_CANNED_BSIC, 0, 0, 0);
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
        "[dsp-shunt] DISPATCH SB page=%u → sb=0x%08x BSIC=%u TOA=%d (read-page only)\n",
        page_idx, sb, SHUNT_CANNED_BSIC, SHUNT_CANNED_TOA);
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
static const uint8_t SHUNT_CANNED_SI3_L2[23] = {
    0x49, 0x06, 0x1B,                   /* L2 hdr + RR PD + SI3 mt */
    0x00, 0x01,                         /* Cell ID = 1 */
    0x00, 0xF1, 0x10,                   /* MCC=001 MNC=01 */
    0x00, 0x01,                         /* LAC = 1 */
    0x01, 0x00,                         /* cell opts + cell select */
    0x18, 0xFF, 0xFF,                   /* RACH ctrl */
    0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B, 0x2B
};

static void shunt_dispatch_allc(uint8_t page_idx)
{
    /* a_cd layout (cf osmocom-bb prim_rx_nb.c) :
     *   a_cd[0]   = FIRE status bits (B_FIRE0/B_FIRE1) -> 0x0000 = CRC pass
     *   a_cd[1]   = (reserved / BLUD bit)              -> 0x0000
     *   a_cd[2]   = num_biterr                          -> 0x0000
     *   a_cd[3..14] = 23 bytes L2 frame (SI3 here)
     */
    uint32_t addr_a_cd = BASE_API_NDB + NDB_A_CD;

    /* a_cd[0..2] = status words = 0 (CRC pass, no biterr) */
    shunt_write_w(addr_a_cd + 0, 0x0000);  /* a_cd[0] */
    shunt_write_w(addr_a_cd + 2, 0x0000);  /* a_cd[1] */
    shunt_write_w(addr_a_cd + 4, 0x0000);  /* a_cd[2] */

    /* a_cd[3..14] = 23B L2 frame, packed in 12 words LE */
    for (int i = 0; i < 23; i += 2) {
        uint8_t lo = SHUNT_CANNED_SI3_L2[i];
        uint8_t hi = (i + 1 < 23) ? SHUNT_CANNED_SI3_L2[i + 1] : 0x2B;
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
    if (!g_shunt.active || !g_shunt.pending) {
        return;
    }
    g_shunt.tick_cnt++;

    uint8_t  page = g_shunt.page_idx;
    uint16_t md   = g_shunt.d_task_md;
    uint16_t td   = g_shunt.d_task_d;

    /* Priority order: md tasks (FB/SB) > NB DL > NB UL > ALLC.
     * Refine when canned policies land. */
    if (md == FB_DSP_TASK) {
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

/* Public getter — gate condition for BSP/TPU DMA into DARAM. */
bool calypso_dsp_shunt_active(void)
{
    return g_shunt.active;
}
