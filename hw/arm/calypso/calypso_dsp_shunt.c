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
#include "calypso_c54x.h"   /* C54xState + c54x_bsp_load/run/interrupt_ex/wake (CALYPSO_DSP=c54x route) */
extern int g_c54x_int3_src;  /* diag source INT3 (RO) */
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
#include <sys/stat.h>
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

/* ---- TCH/F voix (a′). JALON 1 = DL seulement ; UL (a_du_1) = JALON 3, gated. ----
 * Le shunt fait UNIQUEMENT le relais NDB ; le codage canal (gsm0503_tch_fr) est cote
 * qemu_wrap/gr-gsm. task ids : l1_environment.h:50-76. dsp_task_iq_swap (dsp.h:46) peut
 * OR 0x8000 -> comparer avec (& 0x7FFF). */
#define DUL_DSP_TASK        12     /* SDCCH/SACCH UL (sideband calypso_sdcch_ul existant) */
#define TCHT_DSP_TASK       13     /* TCH traffic : RX(d_task_d) ET TX(d_task_u) — le vrai trafic */
#define TCHA_DSP_TASK       14     /* TCH SACCH */
#define TCHD_DSP_TASK       28     /* TCH dummy (RX-only, PAS de data UL) — ne PAS relayer en UL */
/* Offsets NDB (BASE_API_NDB), confirmes DWARF layer1.highram.elf (T_NDB sizeof=0x18d4). */
#define NDB_A_DD_0          0x238  /* DL traffic FR sub0 : [0]@+0 hdr, [2]@+4 biterr, [3]@+6 (33o) */
#define NDB_A_DU_1          0x134  /* PIEGE #1 : UL sub0 = a_du_1 (PAS a_du_0=0x2A0) cf prim_tch.c:485. JALON 3. */
#define NDB_D_TCH_MODE      0x006
/* a_dd_0[0] header bits (l1_environment.h:267-270) */
#define B_FIRE0             5
#define B_FIRE1             6
#define B_BLUD              15     /* data block present (1<<15 = 0x8000) */

#define D_TOA               0
#define D_PM                1
#define D_ANGLE             2
#define D_SNR               3

/* ---- pending-task state ---- */
#ifndef SHM_IQ_LEN
#define SHM_IQ_LEN    320          /* int16 par slot (>= 296 = 148 complexes cs16) */
#endif

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
    /* TCH/F DL (JALON 1) : derniere trame FR 33o lue du sideband /dev/shm/calypso_tch_dl */
    uint8_t    tch_dl_fr[33];
    bool       tch_dl_valid;
    uint32_t   tch_dl_seq;
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
    uint8_t    sacch_buf[23];   /* SI5/SI6 (B4) SACCH dediee SS0 : REEL via feed_sacch
                                 * (fallback = fabrique depuis SI3 tant que !sacch_real) */
    bool       sacch_have;
    bool       sacch_real;      /* true des qu'un SI5/SI6 REEL grgsm est arrive ->
                                 * la fabrication SI3->SI6 cesse de clobber sacch_buf */
    int        si_rr;                 /* index round-robin du dernier type servi */
    /* Resultat de sync REEL poste par gr-gsm (= le DSP) via UDP SCH (4731,
     * shunt_sch_read). Remplace SHUNT_CANNED_BSIC dans shunt_dispatch_sb. */
    uint8_t    sb_bsic;               /* BSIC reel = ncc<<3|bcc (decode_sch gr-gsm) */
    uint32_t   sb_fn;                 /* FN reelle du SCH */
    int16_t    sb_toa;                /* TOA reel mesure du SCH (base 23 = on-time) */
    bool       sb_valid;              /* gr-gsm a poste au moins un SCH reel */
    /* AGCH (#11) : IMM ASSIGN (si_bridge GSMTAP AGCH 0x04). Stocke a part des SI
     * (pas de clobber) ; presente dans a_cd sur un bloc CCCH -> firmware chan_nr=0x90. */
    uint8_t    agch_buf[23];
    bool       agch_valid;
    uint32_t   agch_tick;            /* tick_cnt a l'arrivee (TTL anti-stale) */
    /* SDCCH/4 SS0 DL (#2) : UA/AUTH forwardes par si_bridge (GSMTAP 0x07).
     * Distinct des SI/AGCH ; presente dans a_cd sur le bloc SDCCH/4 SS0
     * (fn%51 in {22-25}) -> firmware tague chan_nr=0x20 -> LAPDm dediee. */
    uint8_t    sdcch_buf[23];
    bool       sdcch_valid;
    uint32_t   sdcch_tick;           /* tick_cnt a l'arrivee (TTL anti-stale) */
    /* PM REEL (no-hardcode) : magnitude moyenne du dernier burst DL (feed_iq).
     * Remplace le canned 0x7000 / le 0=-110 : le firmware en derive le vrai rxlev. */
    uint16_t   last_pm;
    /* CALYPSO_DSP=c54x : handle du VRAI DSP (relie via calypso_dsp_shunt_set_c54x()
     * depuis calypso_mb.c). NULL => route c54x inactive (fallback mock). */
    C54xState *c54x;
    /* Dernier burst I/Q DL stashe par feed_iq pour pilotage au frame tick
     * (cs16 entrelace I,Q). Rejoue via c54x_bsp_load dans shunt_route_to_c54x(). */
    int16_t    last_iq[SHM_IQ_LEN];
    int        last_iq_n;
    uint32_t   last_iq_fn;
    bool       last_iq_valid;
};

/* FN TDMA reelle (calypso_trx.c) pour recoder la FN du shunt (LATCH d_fn=0). */
extern uint32_t calypso_trx_get_fn(void);
extern void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn);
/* FN-FIX : FN du dernier L1CTL_RACH_CONF (= memo exact du mobile), capture dans
 * l1ctl_sock.c au moment de l'envoi au mobile (race-free vs last_rach.fn@0x836500). */
extern volatile uint32_t g_last_rach_conf_fn;
extern volatile uint32_t g_rach_conf_fn[256];   /* per-ra : FN exact du RACH_CONF keye par ra (defini l1ctl_sock.c) */

static struct dsp_shunt_state g_shunt;

/* CALYPSO_DSP=c54x : route les ordres+I/Q vers le VRAI c54x (pas de mock).
 * getenv lu une seule fois (idiome memoize du fichier). */
static bool shunt_route_c54x(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_DSP");
        v = (e && strcmp(e, "c54x") == 0) ? 1 : 0;
    }
    return v;
}

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

/* Lit l1s.current_time.fn (FN L1 du firmware) en ARM RAM. current_time = champ 0
 * de struct l1s_state @ 0x836508 ; fn = champ 0 de struct gsm_time -> offset 0.
 * C'est LE FN que le firmware utilise pour ses blocs (BCCH/CCCH) et mémorise pour
 * la RACH. On gate la présentation a_cd dessus (et NON s->fn = calypso_trx_get_fn,
 * qui diffère de l1s d'un offset run-variant -> blocs CCCH décalés -> AGCH raté). */
static uint32_t shunt_l1s_fn(void)
{
    static uint32_t addr = 0;
    if (!addr) {
        const char *e = getenv("CALYPSO_L1S_FN_ADDR");
        addr = (e && *e) ? (uint32_t)strtoul(e, NULL, 0) : 0x836508;
    }
    uint32_t v = 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* Lit last_rach.fn : le FN EXACT que le firmware a memorise pour la DERNIERE RACH
 * (prim_rach.c:94 last_rach.fn = current_time.fn-1, pose au tick l1s_tx_rach_resp)
 * et qu'il a envoye au mobile via L1CTL_RACH_CONF (prim_rach.c:114). C'EST la valeur
 * que le mobile compare a la req-ref de l'IMM ASSIGN (gsm48_rr.c:3372). La lire
 * directement = match EXACT, sans le skew variable de g_rach_l1s_fn[ra] (capture au
 * tick d_rach/cmd, -4 frames AVANT que le memo soit pose au tick resp -> l'ecart
 * cmd<->resp varie par-RACH, c'est lui qui faisait derailler tout adj fixe).
 * struct { uint32_t fn; uint16_t band_arfcn; } last_rach @ 0x836500, fn @ offset 0. */
static uint32_t shunt_last_rach_fn(void)
{
    static uint32_t addr = 0;
    if (!addr) {
        const char *e = getenv("CALYPSO_LAST_RACH_FN_ADDR");
        addr = (e && *e) ? (uint32_t)strtoul(e, NULL, 0) : 0x836500;
    }
    uint32_t v = 0;
    dma_memory_read(g_shunt.as, addr, &v, sizeof(v), MEMTXATTRS_UNSPECIFIED);
    return le32_to_cpu(v);
}

/* SONDE B : table RA -> FN L1 firmware (l1s.current_time.fn) au moment de la RACH.
 * Remplie par calypso_trx.c (hook write d_rach). Sert à réécrire la req-ref de
 * l'IMM ASSIGN au FN exact que le mobile a mémorisé (preuve que le FN = dernier mur). */
static uint32_t g_rach_l1s_fn[256];
volatile uint8_t g_last_recorded_ra = 0;   /* per-ra FN-FIX : ra de la derniere RACH (lu par l1ctl_sock.c) */
static uint8_t  g_rach_l1s_valid[256];
void calypso_dsp_shunt_record_rach(uint8_t ra)
{
    if (!g_shunt.active) return;
    g_rach_l1s_fn[ra]    = shunt_l1s_fn();
    g_rach_l1s_valid[ra] = 1;
    g_last_recorded_ra   = ra;   /* per-ra FN-FIX : permet a l1ctl_sock de keyer le RACH_CONF par ra */
}

/* SDCCH/SACCH UL sideband (#12) : QEMU publie la L2 montante (a_cu[3..], 23o) vers
 * qemu_wrap via /dev/shm/calypso_sdcch_ul (fichier régulier, pas un FIFO). qemu_wrap
 * l'encode (gsm0503_xcch) + module (burst normal TSC7) + injecte sur le slot UL.
 * Layout 48o : seq@0(u32) l1s_fn@4(u32) fn@8(u32) task_u@12(u16) l1s%51@14(u8) l2[23]@16. */
static void calypso_sdcch_ul_publish(const uint8_t *l2, uint16_t task_u,
                                     uint32_t fn, uint32_t l1s_fn)
{
    static int fd = -2;
    if (fd == -2) {
        fd = open("/dev/shm/calypso_sdcch_ul", O_CREAT | O_RDWR, 0644);
        if (fd >= 0 && ftruncate(fd, 48) < 0) { /* best-effort */ }
    }
    if (fd < 0) return;
    /* #2 PUBLISH-NO-IDLE : NE PAS republier la trame de remplissage (UI, ctrl=0x03).
     * Le firmware poste pu_get_idle_frame()=01 03 01 dans a_cu entre les bursts SABM
     * (burst_id==0, rien en file L23). Chaque publish bumpait seq -> ecrasait la SABM
     * transitoire (~4 frames) dans la slot unique du sideband AVANT que le consommateur
     * (qemu_wrap, 1 pread/frame) ne l'echantillonne -> seul l'idle remontait, jamais la
     * SABM (01 3f) -> osmo-bts jamais de SABM -> jamais d'UA -> T200xN200 -> RR released.
     * En ne publiant QUE les trames signalisantes (ctrl != 0x03), tout nouveau seq est
     * porteur et ne peut plus etre clobbere par le fill -> la SABM tient jusqu'a ce que
     * le consommateur sticky la capture. CALYPSO_UL_PUB_IDLE=1 retablit l'ancien comportement. */
    static int pub_idle = -1;
    if (pub_idle < 0) { const char *e = getenv("CALYPSO_UL_PUB_IDLE"); pub_idle = (e && *e == '1') ? 1 : 0; }
    if (!pub_idle && l2[1] == 0x03) return;   /* trame de fill (UI) : ne pas ecraser la signalisation */
    static uint32_t seq = 0; seq++;
    uint8_t buf[48] = {0};
    memcpy(buf + 4,  &l1s_fn, sizeof(l1s_fn));
    memcpy(buf + 8,  &fn,     sizeof(fn));
    memcpy(buf + 12, &task_u, sizeof(task_u));
    buf[14] = (uint8_t)(l1s_fn % 51);
    memcpy(buf + 16, l2, 23);
    memcpy(buf + 0,  &seq, sizeof(seq));   /* seq en dernier (publication) */
    if (pwrite(fd, buf, sizeof(buf), 0) < 0) { /* best-effort */ }
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

    /* SDCCH/SACCH UL (#12 PIÈCE 1) : quand un NB UL est posté (d_task_u != 0,
     * DUL_DSP_TASK=12 en dédié), lire la L2 a_cu[3..] (23o @ NDB 0x264+6, octets
     * packés 2/mot) et la PUBLIER vers le sideband pour qemu_wrap (encode+module+
     * injecte). a_cu[0..2]=header. La L2 porte le SABM / SACCH meas / I-frames. */
    if (g_shunt.d_task_u != 0) {
        uint8_t l2[23];
        /* a_cu UL : l'offset exact de la trame LAPDm varie (header L1 SACCH 2o /
         * type SABM-I-fill / packing) -> un offset fixe rate. On lit une FENETRE et
         * on SCANNE le debut de trame : 1er octet = addr SDCCH valide (EA=1, SAPI 0/3)
         * suivi d'un control non-fill. Base fenetre = gate CALYPSO_UL_ACU_OFS (def 6). */
        static int acu_ofs = -1;
        if (acu_ofs < 0) { const char *e = getenv("CALYPSO_UL_ACU_OFS"); acu_ofs = (e && *e) ? atoi(e) : 6; }
        uint8_t win[30];
        uint32_t wbase = BASE_API_NDB + 0x264u + (uint32_t)acu_ofs;
        for (int i = 0; i < 30; i += 2) {
            uint16_t w = shunt_read_w(wbase + i);
            win[i] = (uint8_t)(w & 0xff);
            if (i + 1 < 30) win[i + 1] = (uint8_t)((w >> 8) & 0xff);
        }
        int kk = -1;
        for (int j = 0; j <= 6; j++) {
            uint8_t a = win[j], c = win[j + 1];
            int sapi = (a >> 2) & 7;
            if ((a & 0x01) && (sapi == 0 || sapi == 3) &&
                c != 0x2b && c != 0x00 && c != 0xff) { kk = j; break; }
        }
        if (kk < 0) kk = 0;
        for (int i = 0; i < 23; i++) l2[i] = win[kk + i];
        calypso_sdcch_ul_publish(l2, g_shunt.d_task_u,
                                 calypso_trx_get_fn(), shunt_l1s_fn());
        /* Log : quelques idle pour sanity (capé), mais TOUJOURS les frames NON-IDLE
         * (ctrl=l2[1] != 0x03) -> capte le SABM (ctrl 0x3F) et les I-frames. */
        static int ul_log = 0;
        int non_idle = (l2[1] != 0x03);
        if (non_idle || ul_log < 6) {
            if (!non_idle) ul_log++;
            fprintf(stderr, "[dsp-shunt] SDCCH-UL%s task_u=0x%04x l1s%%51=%u "
                    "L2: %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    non_idle ? " *NONIDLE*" : "", g_shunt.d_task_u,
                    (unsigned)(shunt_l1s_fn() % 51),
                    l2[0], l2[1], l2[2], l2[3], l2[4], l2[5], l2[6], l2[7]);
        }
    }

    /* PM : valeur statique, écrite IMMÉDIATEMENT (pas de service déféré au
     * prochain frame IRQ). Sinon le firmware lit a_pm AVANT le dispatch déféré
     * → 0 stale → rxlev=-110. On écrit a_pm sur la page lue tout de suite. */
    if (g_shunt.d_task_md == PM_DSP_TASK)
        shunt_dispatch_pm(page_idx);

    fprintf(stderr,
        "[dsp-shunt] LATCH page=%u task_md=%u task_d=%u task_u=%u task_ra=%u fn=%u\n",
        page_idx, g_shunt.d_task_md, g_shunt.d_task_d, g_shunt.d_task_u,
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

/* ---- CALYPSO_CANNED : énumère EXPLICITEMENT chaque sortie DSP encore
 * FABRIQUÉE (canned) par le shunt, au lieu de la cacher derrière une valeur
 * « plausiblement juste ». CSV insensible casse ; "FULL"/"ALL" = tout canné,
 * "NONE"/"=" vide = rien. Var absente = DÉFAUT = CAN_DEFAULT (désormais RIEN
 * canné : toutes les sorties sont pilotées par le vrai décode gr-gsm). On
 * re-canne sélectivement avec CALYPSO_CANNED=<token>. BSIC/SI ne sont PAS ici :
 * déjà réels via gr-gsm / feed_si ; leur état est loggué au boot. */
enum {
    CAN_FBDET = 1u << 0,   /* d_fb_det = 1 ("FB found") forcé          */
    CAN_TOA   = 1u << 1,   /* a_sync/serv_demod[TOA] = 23 ("on time")  */
    CAN_PM    = 1u << 2,   /* [PM] = 0x7000 (rxlev fort)               */
    CAN_SNR   = 1u << 3,   /* [SNR] = 0x7000 (passe AFC_SNR_THRESHOLD) */
    CAN_ANGLE = 1u << 4,   /* [ANGLE] = 0 (AFC convergé, pas de chasse)*/
    CAN_CRC   = 1u << 5,   /* a_sch[0]/a_cd status = 0 (CRC forcé pass)*/
};
#define CAN_ALL (CAN_FBDET|CAN_TOA|CAN_PM|CAN_SNR|CAN_ANGLE|CAN_CRC)
/* TOUT DÉGATÉ (testés, camping tient), piloté par le vrai état de décode gr-gsm :
 *   FBDET = sb_valid (FB trouvé ssi SCH décodé)
 *   TOA   = timing SCH réel gr-gsm
 *   ANGLE = 0 (résidu réel post-correction freq)
 *   CRC   = sb_valid/si_valid (pass ssi vraiment décodé, sinon fail)
 *   PM    = 0 (a_serv_demod[PM] FB/SB ne pilote pas le rxlev — vient de dispatch_pm)
 *   SNR   = sb_valid ? bon : 0 (gr-gsm a décodé = preuve SNR suffisant ≥ seuil ;
 *           magnitude pas mesurée mais conditionnée au vrai décode, pas fabriquée
 *           inconditionnellement → SI ne casse pas).
 * DÉFAUT = RIEN canné. CALYPSO_CANNED=<token> re-canne sélectivement ; FULL=CAN_ALL. */
#define CAN_DEFAULT (0u)

static unsigned g_canned = CAN_DEFAULT;   /* résolu dans calypso_dsp_shunt_init */

static int can_tok_eq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca -= 32;
        if (cb >= 'a' && cb <= 'z') cb -= 32;
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == 0 && *b == 0;
}

static unsigned shunt_parse_canned(void)
{
    const char *e = getenv("CALYPSO_CANNED");
    if (!e)                                          return CAN_DEFAULT;  /* var ABSENTE = défaut (= rien canné, tout réel) */
    if (!*e || can_tok_eq(e, "NONE"))                return 0;            /* "=" vide EXPLICITE = RIEN canné */
    if (can_tok_eq(e, "FULL") || can_tok_eq(e, "ALL")) return CAN_ALL;
    char buf[160];
    strncpy(buf, e, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    unsigned m = 0;
    for (char *t = strtok(buf, ", "); t; t = strtok(NULL, ", ")) {
        if      (can_tok_eq(t, "FBDET")) m |= CAN_FBDET;
        else if (can_tok_eq(t, "TOA"))   m |= CAN_TOA;
        else if (can_tok_eq(t, "PM"))    m |= CAN_PM;
        else if (can_tok_eq(t, "SNR"))   m |= CAN_SNR;
        else if (can_tok_eq(t, "ANGLE")) m |= CAN_ANGLE;
        else if (can_tok_eq(t, "CRC"))   m |= CAN_CRC;
        else if (can_tok_eq(t, "FULL") || can_tok_eq(t, "ALL")) m = CAN_ALL;
        else error_report("[dsp-shunt] CALYPSO_CANNED: token inconnu '%s' ignore", t);
    }
    return m;
}

static inline bool shunt_is_canned(unsigned bit) { return (g_canned & bit) != 0; }

/* Valeur TOA pour a_*_demod[TOA] : cannée (23 = on-time) si CAN_TOA, sinon le
 * TOA REEL mesuré par gr-gsm (sb_toa) dès qu'un SCH a été décodé ; fallback 23
 * tant qu'aucun SCH (pas 0 : évite de catastropher l'alignement avant lock). */
static inline int shunt_toa_val(void)
{
    if (shunt_is_canned(CAN_TOA))
        return SHUNT_CANNED_TOA;
    return g_shunt.sb_valid ? g_shunt.sb_toa : SHUNT_CANNED_TOA;
}

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
    /* d_fb_det = 1 ("FOUND"). prim_fbsb.c:404 reads this from NDB.
     * Canned CAN_FBDET = on force "trouvé" (pas de vrai détecteur FB ici). */
    /* FBDET non-canné = état RÉEL de détection gr-gsm : "trouvé" ssi un SCH a
     * été décodé (sb_valid). Avant lock → 0 (FB pas trouvé, comme un vrai DSP). */
    shunt_write_w(BASE_API_NDB + NDB_D_FB_DET,
                  (shunt_is_canned(CAN_FBDET) || g_shunt.sb_valid) ? 1 : 0);

    /* a_sync_demod[4] @ NDB+0x4C, 4 consecutive 16-bit words. Read by
     * read_fb_result (prim_fbsb.c:306-309) from NDB. Chaque mesure : valeur
     * cannée si son token est dans CALYPSO_CANNED, sinon 0 (pas encore de
     * vraie source → un-canner sans source casse, c'est voulu/visible). */
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_TOA   * 2, shunt_toa_val());
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM)    ? SHUNT_CANNED_PM    : g_shunt.last_pm);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
    shunt_write_w(BASE_API_NDB + NDB_A_SYNC_DEMOD + D_SNR   * 2, (shunt_is_canned(CAN_SNR) || g_shunt.sb_valid) ? SHUNT_CANNED_SNR : 0);

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

    /* a_sch[0] CRC bit clear = success (prim_fbsb.c:181, B_SCH_CRC=8).
     * CAN_CRC canné = on FORCE le pass (0). Non-canné = pas de faux succès :
     * sans vraie source CRC on écrit le bit d'échec → fail VISIBLE (le SB sera
     * rejeté) au lieu de masquer. Défaut canné → pass → camping inchangé. */
    shunt_write_w(rp + RP_A_SCH + 0 * 2,
                  (uint16_t)((shunt_is_canned(CAN_CRC) || g_shunt.sb_valid)
                             ? 0x0000 : B_SCH_CRC));   /* pass RÉEL ssi SCH décodé */

    /* sb = encode_sb(bsic, t1, t2, t3) → a_sch[3] | a_sch[4]<<16
     * (prim_fbsb.c:198). Two separate 16-bit stores, both LE. */
    uint32_t sb = shunt_encode_sb(bsic, t1, t2, t3);
    shunt_write_w(rp + RP_A_SCH + 3 * 2, (uint16_t)(sb & 0xFFFF));
    shunt_write_w(rp + RP_A_SCH + 4 * 2, (uint16_t)(sb >> 16));

    /* a_sch[1] / a_sch[2] are unused by l1s_decode_sb; zero them. */
    shunt_write_w(rp + RP_A_SCH + 1 * 2, 0x0000);
    shunt_write_w(rp + RP_A_SCH + 2 * 2, 0x0000);

    /* a_serv_demod[4] @ +0x10. read_sb_result reads from READ PAGE here,
     * NOT NDB (prim_fbsb.c:148-151). Chaque mesure cannée/0 selon CALYPSO_CANNED. */
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM)    ? SHUNT_CANNED_PM    : g_shunt.last_pm);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2, (shunt_is_canned(CAN_SNR) || g_shunt.sb_valid) ? SHUNT_CANNED_SNR : 0);

    /* Ack on read page. */
    shunt_write_w(rp + RP_D_TASK_MD, SB_DSP_TASK);

    fprintf(stderr,
        "[dsp-shunt] DISPATCH SB page=%u → sb=0x%08x BSIC=%u FN=%u %s TOA=%d\n",
        page_idx, sb, bsic, fn,
        g_shunt.sb_valid ? "(gr-gsm REEL)" : "(canned legacy)", shunt_toa_val());
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

    /* === AGCH (#11) : IMM ASSIGN présenté dans a_cd sur un bloc CCCH ===========
     * Si un IMM ASSIGN est en attente, on le présente A LA PLACE du SI sur les
     * blocs CCCH (combiné CCCH+SDCCH4 : fn%51 ∈ {6-9,12-19}). Le firmware, sur son
     * read CCCH_COMB, tague chan_nr=0x90 -> gsm48_rr_rx_pch_agch -> rx_imm_ass ->
     * gsm48_match_ra. Présenté sur CHAQUE bloc CCCH tant que valide (TTL) : le
     * firmware le lit une fois, multi-présentation = robuste à l'alignement FN
     * (RR dédup via cr_hist). Les SI restent inchangés (blocs BCCH). Tunables :
     * CALYPSO_SHUNT_AGCH(=1 def), _AGCH_OFS (offset FN), _AGCH_TTL (ticks, def 100). */
    static int agch_on = -1, agch_ofs = 0, agch_ttl = 100;
    if (agch_on < 0) {
        const char *e = getenv("CALYPSO_SHUNT_AGCH");     agch_on  = (!e || *e != '0') ? 1 : 0;
        const char *o = getenv("CALYPSO_SHUNT_AGCH_OFS"); agch_ofs = o ? atoi(o) : 0;
        const char *t = getenv("CALYPSO_SHUNT_AGCH_TTL"); if (t && *t) agch_ttl = atoi(t);
    }
    if (agch_on && g_shunt.agch_valid) {
        if ((uint32_t)(g_shunt.tick_cnt - g_shunt.agch_tick) > (uint32_t)agch_ttl) {
            g_shunt.agch_valid = false;                   /* périmé -> rendre la main aux SI */
        } else {
            /* gate sur le FN L1 FIRMWARE (l1s), pas s->fn : c'est l'horloge des
             * vrais blocs CCCH du firmware -> alignement run-invariant. */
            int tc = (int)((((long)shunt_l1s_fn() + agch_ofs) % 51 + 51) % 51);
            int is_ccch = (tc >= 6 && tc <= 9) || (tc >= 12 && tc <= 19);
            if (is_ccch) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000);            /* a_cd[0] FIRE = CRC pass */
                shunt_write_w(aa + 2, 0x0000);
                shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.agch_buf;
                for (int i = 0; i < 23; i += 2) {
                    uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B;
                    shunt_write_w(aa + 6 + i, lo | (hi << 8));
                }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, g_shunt.d_burst_d);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                static unsigned n_agch = 0;
                if (n_agch++ < 40 || (n_agch % 50) == 0)
                    fprintf(stderr, "[dsp-shunt] DISPATCH AGCH IMM-ASS #%u burst_d=%u "
                            "tc=%d -> a_cd (chan_nr=0x90 attendu)\n",
                            n_agch, g_shunt.d_burst_d, tc);
                return;                                   /* ce dispatch = l'IMM ASSIGN */
            }
        }
    }

    /* === SDCCH/4 SS0 DL (#2) : UA/AUTH presente dans a_cd sur le bloc SDCCH/4 ===
     * Miroir EXACT de la branche AGCH ci-dessus. Si un bloc SDCCH DL est en
     * attente (feed_sdcch), on le presente A LA PLACE du SI sur le bloc SDCCH/4
     * SS0 (fn%51 in {22-25}). Le firmware (l1s_nb_cmd pose ALLC_DSP_TASK=24 pour
     * TOUS les NB DL, SDCCH inclus) tourne MF_TASK_SDCCH4_0 a ce FN -> tague
     * chan_nr=0x20 -> lapdm_dcch -> UA/AUTH -> L3. Gate sur shunt_l1s_fn() (FN L1
     * firmware), PAS calypso_trx_get_fn(), comme l'AGCH. Tunables :
     * CALYPSO_SHUNT_SDCCH(=1 def), _SDCCH_OFS (offset FN), _SDCCH_TTL (def 100). */
    static int sdcch_on = -1, sdcch_ofs = 0, sdcch_ttl = 100;
    if (sdcch_on < 0) {
        const char *e = getenv("CALYPSO_SHUNT_SDCCH");     sdcch_on  = (!e || *e != '0') ? 1 : 0;
        const char *o = getenv("CALYPSO_SHUNT_SDCCH_OFS"); sdcch_ofs = o ? atoi(o) : 0;
        const char *t = getenv("CALYPSO_SHUNT_SDCCH_TTL"); if (t && *t) sdcch_ttl = atoi(t);
    }
    if (sdcch_on && g_shunt.sdcch_valid) {
        if ((uint32_t)(g_shunt.tick_cnt - g_shunt.sdcch_tick) > (uint32_t)sdcch_ttl) {
            g_shunt.sdcch_valid = false;                  /* perime -> rendre la main aux SI */
        } else {
            int tc = (int)((((long)shunt_l1s_fn() + sdcch_ofs) % 51 + 51) % 51);
            /* BURST-COVERAGE FIX (#2) : le firmware (prim_rx_nb.c l1s_nb_resp) ne
             * copie a_cd[3..14] dans L1CTL_DATA_IND qu'au 4eme burst du bloc
             * (d_burst_d==3) et tague alors chan_nr=0x20. Le bloc SDCCH/4 SS0 dure
             * 4 bursts (FN consecutifs) : ses bursts s'etalent sur fn%51 {25,26,27,28}
             * pour l'alignement 5216, donc l'ancien gate {22-25} ne matchait QUE le
             * burst_d=0 et le consume-once tc>=24 liberait le buffer AVANT le
             * burst_d==3 -> le SI3 ecrasait a_cd au moment ou le firmware lit. On
             * gate donc sur g_shunt.d_burst_d (le compteur de burst du firmware,
             * deja echo dans RP_D_BURST_D) : on presente le UA sur burst_d 0..3 du
             * bloc SDCCH/4 SS0 (un seul bloc), puis on libere APRES burst_d==3.
             * Ainsi a_cd tient le UA quand le firmware le copie au burst_d==3, et la
             * trame est presentee EXACTEMENT une fois (1 DATA_IND/bloc) -> pas de
             * re-presentation sur la multitrame suivante -> pas de UNSOL_UA. tc reste
             * une garde large {22-28} (les 4 bursts) en plus du burst_d pour ne pas
             * empieter sur les autres blocs. */
            int is_sdcch4_ss0 = (tc >= 22 && tc <= 28);
            if (is_sdcch4_ss0) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000);            /* a_cd[0] FIRE = CRC pass */
                shunt_write_w(aa + 2, 0x0000);
                shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.sdcch_buf;
                for (int i = 0; i < 23; i += 2) {
                    uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B;
                    shunt_write_w(aa + 6 + i, lo | (hi << 8));
                }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, g_shunt.d_burst_d);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                static unsigned n_sdcch = 0;
                if (n_sdcch++ < 40 || (n_sdcch % 50) == 0)
                    fprintf(stderr, "[dsp-shunt] DISPATCH SDCCH/4 SS0 #%u burst_d=%u "
                            "tc=%d -> a_cd (chan_nr=0x20 attendu)\n",
                            n_sdcch, g_shunt.d_burst_d, tc);
                /* CONSUME-ONCE (corrige) : presenter le UA sur TOUS les bursts du
                 * bloc (burst_d 0,1,2,3) pour qu'il soit TOUJOURS dans a_cd[3..14]
                 * quand le firmware le copie au burst_d==3 (prim_rx_nb.c), PUIS
                 * liberer APRES ce burst_d==3. Le firmware n'emet qu'UN
                 * L1CTL_DATA_IND par bloc (au burst_d==3), donc -> 1 seul UA cote
                 * LAPDm, et le buffer n'est PAS re-presente sur le bloc SS0 de la
                 * multitrame suivante -> pas de UNSOL_UA. (L'ancien code liberait au
                 * tc>=24, AVANT le burst_d==3 : le SI3 ecrasait alors a_cd.) Si le
                 * bloc est rate, la retransmission T200 de la BTS re-alimente
                 * feed_sdcch (et si_bridge re-forwarde le UA re-emis, FN distinct). */
                if (g_shunt.d_burst_d >= 3)
                    g_shunt.sdcch_valid = false;
                return;                                   /* ce dispatch = le bloc SDCCH DL */
            }
        }
    }

    /* === SACCH SDCCH/4 SS0 DL : presente le SI6 (B4) sur les slots SACCH du SS0 ===
     * Sinon le mobile lit du garbage sur la SACCH dediee -> 'Short header 0x07'.
     * Slots SACCH SS0 (combine CCCH+SDCCH/4, GSM 05.02) : fn%51 in {42-45} ET
     * (fn/51)%2==0. Gate CALYPSO_SHUNT_SACCH (def ON). */
    {
        static int sacch_on = -1;
        if (sacch_on < 0) { const char *e = getenv("CALYPSO_SHUNT_SACCH"); sacch_on = (!e || *e != '0') ? 1 : 0; }
        if (sacch_on && g_shunt.sacch_have) {
            long fn = shunt_l1s_fn();
            int tc    = (int)(((fn % 51) + 51) % 51);
            int mf102 = (int)(((fn / 51) % 2 + 2) % 2);
            if (tc >= 42 && tc <= 46 && mf102 == 0) {
                uint32_t aa = BASE_API_NDB + NDB_A_CD;
                shunt_write_w(aa + 0, 0x0000);
                shunt_write_w(aa + 2, 0x0000);
                shunt_write_w(aa + 4, 0x0000);
                const uint8_t *m = g_shunt.sacch_buf;
                for (int i = 0; i < 23; i += 2) {
                    uint8_t lo = m[i], hi = (i + 1 < 23) ? m[i + 1] : 0x2B;
                    shunt_write_w(aa + 6 + i, lo | (hi << 8));
                }
                uint32_t rpA = rp_base(page_idx);
                shunt_write_w(rpA + RP_D_TASK_D,  ALLC_DSP_TASK);
                shunt_write_w(rpA + RP_D_BURST_D, g_shunt.d_burst_d);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM) ? SHUNT_CANNED_PM : g_shunt.last_pm);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_ANGLE * 2, 0);
                shunt_write_w(rpA + RP_A_SERV_DEMOD + D_SNR   * 2, SHUNT_CANNED_SNR);
                static unsigned n_sacch = 0;
                if (n_sacch++ < 20 || (n_sacch % 50) == 0)
                    fprintf(stderr, "[dsp-shunt] DISPATCH SACCH SI6 #%u tc=%d -> a_cd\n", n_sacch, tc);
                return;
            }
        }
    }

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

    /* a_cd[0..2] = status words. CAN_CRC canné = CRC pass (0) ; non-canné =
     * pas de faux pass → FIRE=fail (0x0003) visible. a_cd[1/2] biterr = 0. */
    shunt_write_w(addr_a_cd + 0,
                  (shunt_is_canned(CAN_CRC) || g_shunt.si_valid) ? 0x0000 : 0x0003);  /* a_cd[0] FIRE : pass RÉEL ssi SI décodé */
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
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_TOA   * 2, shunt_toa_val());
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_PM    * 2, shunt_is_canned(CAN_PM)    ? SHUNT_CANNED_PM    : g_shunt.last_pm);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_ANGLE * 2, shunt_is_canned(CAN_ANGLE) ? SHUNT_CANNED_ANGLE : 0);
    shunt_write_w(rp + RP_A_SERV_DEMOD + D_SNR   * 2, (shunt_is_canned(CAN_SNR) || g_shunt.sb_valid) ? SHUNT_CANNED_SNR : 0);

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

/* ---- TCH/F DL (JALON 1) : sideband /dev/shm/calypso_tch_dl -> a_dd_0 ----
 * Producteur = qemu_wrap/gr-gsm (decode 8 bursts -> gsm0503_tch_fr_decode -> 33o FR) ou
 * l'injecteur de test (tone 600). Layout 48o : seq@0(u32 LE) fn@4(u32 LE) fr[33]@8.
 * Consume-once par seq (modele calypso_rach_read). */
static void calypso_tch_dl_poll(void)
{
    static int fd = -2;
    if (fd == -2)
        fd = open("/dev/shm/calypso_tch_dl", O_CREAT | O_RDWR, 0644); /* cree -> open une fois */
    if (fd < 0)
        return;
    uint8_t buf[48];
    if (pread(fd, buf, sizeof(buf), 0) != (ssize_t)sizeof(buf))
        return;
    uint32_t seq;
    memcpy(&seq, buf, 4);
    if (seq == 0 || seq == g_shunt.tch_dl_seq)
        return;                         /* pas de nouvelle trame */
    g_shunt.tch_dl_seq = seq;
    memcpy(g_shunt.tch_dl_fr, buf + 8, 33);
    g_shunt.tch_dl_valid = true;
}

/* Ecrit la trame FR 33o dans a_dd_0 (sub0). Le firmware (prim_tch.c:322 sub0->a_dd_0) la
 * lit en fin-de-bloc (FN%13 in {3,7,11}) ssi a_dd_0[0]&B_BLUD, puis L1CTL_TRAFFIC_IND->gapk.
 * PIEGE #2 : packing BIG-ENDIAN intra-mot (dsp_memcpy_from_api BE=1, dsp.c:278) = l'INVERSE
 * du a_cd existant (BE=0, lo|(hi<<8)). word = (fr[i]<<8)|fr[i+1] ; mot impair = fr[32]<<8. */
static void shunt_dispatch_tch_dl(uint8_t page_idx)
{
    (void)page_idx;                     /* a_dd_0 non page (T_NDB partage) */
    if (!g_shunt.tch_dl_valid)
        return;
    uint32_t aa = BASE_API_NDB + NDB_A_DD_0;
    shunt_write_w(aa + 0, (1u << B_BLUD));   /* a_dd_0[0] : B_BLUD=1, FIRE=00 (no error) */
    shunt_write_w(aa + 2, 0x0000);           /* a_dd_0[1] : inutilise */
    shunt_write_w(aa + 4, 0x0000);           /* a_dd_0[2] : num_biterr = 0 */
    const uint8_t *fr = g_shunt.tch_dl_fr;   /* a_dd_0[3..] = 33o @ aa+6, BE=1 */
    for (int i = 0; i < 32; i += 2)
        shunt_write_w(aa + 6 + i, ((uint16_t)fr[i] << 8) | fr[i + 1]);
    shunt_write_w(aa + 6 + 32, (uint16_t)fr[32] << 8);  /* octet impair en poids fort */
}

/* CALYPSO_DSP=c54x : pilote le VRAI DSP depuis le frame tick du shunt.
 * Les ordres (d_task_md/d_task_d/d_task_u/d_task_ra) sont DEJA dans l'api_ram
 * partagee (c54x->api_ram == dsp_ram cote ARM), donc pas de recopie du
 * descripteur ici. On (a) DMA la write-page API -> DARAM 0x0586 (le trx skippe
 * cette DMA quand le shunt est actif, on la refait nous-memes), (b) recharge le
 * dernier burst I/Q dans bsp_buf, (c) leve INT3 (FRAME) + wake, (d) execute le
 * budget c54x_run.
 *
 * FIX (verif report) : la write-page MCU->DSP est a BASE_API_W_PAGE_0/1
 * (0xFFD00000 / 0xFFD00028), PAS a BASE_API_NDB (0xFFD001A8). On reutilise le
 * helper wp_base() existant. Replique la DMA de trx calypso_dsp_done(@711) :
 * data[0x0584]=page, data[0x0585]=fn, data[0x0586+i]=wp[i] (i<20), et le mirror
 * api_ram[0x08D4 - C54X_API_BASE]=page (d_dsp_page cote DSP, lu par le firmware). */
static void shunt_route_to_c54x(uint8_t page_idx)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;

    /* (a) API write-page -> DARAM 0x0586 (replique de la DMA trx gatee a :711).
     * wp_base(page_idx) = adresse MMIO absolue de la write-page (== dsp_ram).
     * Le mot d_dsp_page (NDB+0 = 0xFFD001A8) est lu live (= s->dsp_ram[0x01A8/2]
     * cote trx) pour data[0x0584] et le mirror 0x08D4. */
    {
        uint32_t wbase    = wp_base(page_idx);
        uint16_t dsp_page = shunt_read_w(BASE_API_NDB + NDB_D_DSP_PAGE);
        dsp->data[0x0584] = dsp_page;
        dsp->data[0x0585] = (uint16_t)(g_shunt.d_fn & 0xFFFF);
        for (int i = 0; i < 20; i++)
            dsp->data[0x0586 + i] = shunt_read_w(wbase + (uint32_t)i * 2);
        /* mirror d_dsp_page cote DSP (le firmware le lit a api_ram 0x08D4). */
        if (dsp->api_ram)
            dsp->api_ram[0x08D4 - C54X_API_BASE] = dsp_page;
    }

    /* (b) rejoue le dernier burst I/Q (cs16 entrelace I,Q) dans bsp_buf. */
    if (g_shunt.last_iq_valid && g_shunt.last_iq_n > 0)
        c54x_bsp_load(dsp, (const uint16_t *)g_shunt.last_iq, g_shunt.last_iq_n);

    /* (c) INT3 FRAME + wake : reveille le DSP s'il etait idle/halt. */
    g_c54x_int3_src = 3;
    c54x_interrupt_ex(dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
    c54x_wake(dsp);

    /* (d) execute le budget (1 trame nominale ~256000 insns ; ajustable env). */
    {
        static int budget = -1;
        if (budget < 0) {
            const char *b = getenv("CALYPSO_DSP_BUDGET");
            budget = (b && *b) ? atoi(b) : 256000;
            if (budget <= 0) budget = 256000;
        }
        c54x_run(dsp, budget);
    }
}

/* ---- Service hook : called from calypso_trx frame_irq tick ---- */
void calypso_dsp_shunt_on_frame_tick(void)
{
    if (!g_shunt.active)
        return;
    shunt_poll_si_shm();   /* gr-gsm a-t-il ecrit un nouveau SI dans le shm ? */
    calypso_tch_dl_poll(); /* nouvelle trame FR DL dans le sideband ? (toujours, hors gate pending) */
    if (!g_shunt.pending) {
        return;
    }
    g_shunt.tick_cnt++;

    uint8_t  page = g_shunt.page_idx;
    uint16_t md   = g_shunt.d_task_md;
    uint16_t td   = g_shunt.d_task_d;

    /* Priority order: md tasks (FB/SB) > NB DL > NB UL > ALLC.
     * Refine when canned policies land. */
    if (shunt_route_c54x() && g_shunt.c54x) {
        /* CALYPSO_DSP=c54x : ne mocke PAS — route vers le VRAI DSP. */
        shunt_route_to_c54x(page);
    } else if (md == PM_DSP_TASK) {
        shunt_dispatch_pm(page);
    } else if (md == FB_DSP_TASK) {
        shunt_dispatch_fb(page);
    } else if (md == SB_DSP_TASK) {
        shunt_dispatch_sb(page);
    } else if (td == ALLC_DSP_TASK) {
        shunt_dispatch_allc(page);
    } else if ((td & 0x7FFF) == TCHT_DSP_TASK) {   /* TCH/F DL (JALON 1) : sub0 -> a_dd_0 */
        shunt_dispatch_tch_dl(page);
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
#define GSMTAP_CHANNEL_AGCH     0x04   /* IMM ASSIGN (PORTE 3a / #11) */
#define GSMTAP_CHANNEL_SDCCH4   0x07   /* SDCCH/4 SS0 DL (UA/AUTH / #2) */
#define GSMTAP_CHANNEL_ACCH     0x80   /* bit ACCH (SACCH) GSMTAP */
#define GSMTAP_CHANNEL_SACCH    (GSMTAP_CHANNEL_SDCCH4 | GSMTAP_CHANNEL_ACCH) /* 0x87 : SI5/SI6 reels */
static int g_gsmtap_fd = -1;

/* AGCH (#11) : range l'IMM ASSIGN forwardé par si_bridge (tag GSMTAP AGCH 0x04)
 * dans agch_buf — DISTINCT de si_buf, pour que la rotation des SI ne l'écrase pas.
 * shunt_dispatch_allc le présentera dans a_cd sur un bloc CCCH (le firmware tague
 * alors chan_nr=0x90 -> gsm48_rr_rx_pch_agch -> gsm48_rr_rx_imm_ass). */
static void calypso_dsp_shunt_feed_agch(const uint8_t *l2, int len)
{
    if (!l2 || len < 3) return;

    /* Priorite IMM ASSIGN : ne pas laisser un PAGING REQUEST (mt 0x21/0x22/0x24)
     * ecraser un IMM ASSIGN (0x3f/0x3a/0x3b) encore valide en attente de
     * presentation sur l'AGCH. L'IMM ASSIGN est la reponse time-critical au RACH
     * (un seul agch_buf partage) ; le paging est best-effort. Sur un reseau a
     * paging dense le flood clobberait sinon le grant -> RACH en boucle, pas de LU. */
    {
        uint8_t in_mt = l2[2];
        int in_is_imm = (in_mt == 0x3f || in_mt == 0x3a || in_mt == 0x3b);
        uint8_t cur_mt = g_shunt.agch_buf[2];
        int cur_is_imm = (cur_mt == 0x3f || cur_mt == 0x3a || cur_mt == 0x3b);
        if (!in_is_imm && g_shunt.agch_valid && cur_is_imm) {
            static int ttl = -1;
            if (ttl < 0) { const char *t = getenv("CALYPSO_SHUNT_AGCH_TTL");
                           ttl = (t && *t) ? atoi(t) : 100; }
            if ((uint32_t)(g_shunt.tick_cnt - g_shunt.agch_tick) <= (uint32_t)ttl) {
                static unsigned drop = 0;
                if (drop++ < 20 || (drop % 200) == 0)
                    fprintf(stderr, "[dsp-shunt] feed_agch: PAGING mt=0x%02x DROP "
                            "(IMM ASSIGN 0x%02x encore valide en attente)\n", in_mt, cur_mt);
                return;
            }
        }
    }

    int n = len < 23 ? len : 23;
    memcpy(g_shunt.agch_buf, l2, n);
    for (int i = n; i < 23; i++) g_shunt.agch_buf[i] = 0x2B;

    /* FN-FIX (le vrai fix, ON par defaut ; CALYPSO_REQREF_REWRITE=0 pour A/B) :
     * reecrit la request-reference de l'IMM ASSIGN (octets L2 [8],[9]) au FN EXACT que
     * le firmware a memorise pour la derniere RACH = last_rach.fn (@0x836500). RAISON :
     * le FN de la req-ref vit dans l'horloge osmo-trx (sample-position, base ~2465144),
     * le mobile le compare a SA propre horloge L1 (la valeur recue en L1CTL_RACH_CONF =
     * last_rach.fn, prim_rach.c:114) ; ces deux compteurs free-running ont une phase de
     * depart non controlee et variable par-RACH -> mismatch gsm48_rr.c:3382. Aucun
     * cal_off/ul_fnoff/fn_adj cote device ne peut les aligner.
     * On lit donc DIRECTEMENT last_rach.fn (la valeur que le mobile a memorisee, par
     * construction) au lieu de g_rach_l1s_fn[ra]+adj : ce dernier capturait current_time
     * au tick d_rach/cmd, soit -4 frames AVANT que le firmware pose last_rach.fn =
     * current_time-1 au tick rach_resp -> skew variable (constate : adj devait passer de
     * -1 a +1 entre deux runs). last_rach.fn n'a aucun skew ni collision RA (1 seule RACH
     * en vol cote mobile). Le check du mobile (gsm48_rr.c:3372) est PUREMENT local : il
     * matche (ra,T1,T2,T3) contre son propre cr_hist ; le FN est informationnel. Donc
     * req-ref := last_rach.fn => match exact, sans constante FN magique ni adj.
     * RA = L2[7] (log seulement). Encodage req-ref (04.08) :
     *   [8] = (T1'<<3) | (T3>>3) ; [9] = ((T3&7)<<5) | T2 ; T1'=(FN/1326)%32.
     * adj=0 par defaut (last_rach.fn EST le memo) ; surchargeable CALYPSO_REQREF_ADJ. */
    {
        static int reqref_rw = -1, reqref_perra = -1, rr_adj = -99999;
        if (reqref_rw < 0)    { const char *e = getenv("CALYPSO_REQREF_REWRITE"); reqref_rw = (e && *e == '1') ? 1 : 0; }  /* defaut OFF : ancien rewrite GLOBAL (50% multi-RACH) */
        if (reqref_perra < 0) { const char *e = getenv("CALYPSO_REQREF_PERRA");   reqref_perra = (e && *e == '0') ? 0 : 1; } /* defaut ON : req-ref PER-RA (FN exact du RACH_CONF keye par ra) */
        if (rr_adj == -99999) { const char *e = getenv("CALYPSO_REQREF_ADJ");     rr_adj = e ? atoi(e) : 0; }
        if ((reqref_perra || reqref_rw) && n >= 10 && g_shunt.agch_buf[2] == 0x3f) {
            uint8_t ra = g_shunt.agch_buf[7];
            uint32_t memo_fn = (reqref_perra && g_rach_conf_fn[ra]) ? g_rach_conf_fn[ra]
                             : (reqref_rw ? g_last_rach_conf_fn : 0);   /* per-ra exact, sinon fallback global */
            { static unsigned dbg = 0;
              if (dbg++ < 40)
                  fprintf(stderr, "[dsp-shunt] FN-FIX probe RA=0x%02x "
                          "memo_fn(RACH_CONF)=%u last_rach@500=%u l1s_fn=%u n=%d\n",
                          ra, memo_fn, shunt_last_rach_fn(), shunt_l1s_fn(), n); }
            if (memo_fn) {
                int64_t fn = (int64_t)memo_fn + rr_adj;
                if (fn < 0) fn = 0;
                uint16_t t1p = (uint16_t)(((uint32_t)fn / 1326u) % 32u);
                uint8_t  t2  = (uint8_t)((uint32_t)fn % 26u);
                uint8_t  t3  = (uint8_t)((uint32_t)fn % 51u);
                g_shunt.agch_buf[8] = (uint8_t)((t1p << 3) | ((t3 >> 3) & 7));
                g_shunt.agch_buf[9] = (uint8_t)(((t3 & 7) << 5) | (t2 & 0x1f));
                static unsigned rwlog = 0;
                if (rwlog++ < 30)
                    fprintf(stderr, "[dsp-shunt] FN-FIX req-ref RA=0x%02x reecrite -> "
                            "fn=%u (T1'=%u T2=%u T3=%u) adj=%d [last_rach.fn]\n",
                            ra, (uint32_t)fn, t1p, t2, t3, rr_adj);
            }
        }
    }

    g_shunt.agch_valid = true;
    g_shunt.agch_tick  = g_shunt.tick_cnt;
    fprintf(stderr, "[dsp-shunt] feed_agch: IMM-ASS mt=0x%02x -> agch_buf "
            "(a presenter sur bloc CCCH)\n", l2[2]);
}

/* SDCCH/4 SS0 DL (#2) : range le bloc L2 (UA/AUTH) forwarde par si_bridge
 * (tag GSMTAP SDCCH4 0x07) dans sdcch_buf -- DISTINCT de si_buf/agch_buf.
 * shunt_dispatch_allc le presentera dans a_cd sur le bloc SDCCH/4 SS0
 * (fn%51 in {22-25}) ; le firmware tague alors chan_nr=0x20 -> lapdm_dcch ->
 * UA/AUTH -> L3 (miroir de feed_agch, SANS la sonde req-ref). */
static void calypso_dsp_shunt_feed_sdcch(const uint8_t *l2, int len)
{
    if (!l2 || len < 3) return;
    int n = len < 23 ? len : 23;
    memcpy(g_shunt.sdcch_buf, l2, n);
    for (int i = n; i < 23; i++) g_shunt.sdcch_buf[i] = 0x2B;

    g_shunt.sdcch_valid = true;
    g_shunt.sdcch_tick  = g_shunt.tick_cnt;
    fprintf(stderr, "[dsp-shunt] feed_sdcch: SDCCH/4 SS0 DL a0=0x%02x c=0x%02x "
            "-> sdcch_buf (a presenter sur bloc fn%%51 22-25)\n", l2[0], l2[1]);
}

/* SACCH SS0 DL REELLE : SI5(0x1d)/SI6(0x1e) decodes par grgsm, forwardes par
 * si_bridge (sub_type 0x87). Le bloc grgsm = 23o : [L1 hdr 2][LAPDm: 03 03 len
 * 06 mt L3...] -> exactement le layout B4 attendu par le dispatch SACCH. On
 * garde la L3 REELLE mais on ZERO le header L1 (tx_power/TA) : les valeurs
 * osmo-bts ne sont pas pour notre air emule (idem fabrication). sacch_real=true
 * fait CESSER la fabrication SI3->SI6 (sinon SI3 du BCCH clobbe le SI5/SI6 reel). */
static void calypso_dsp_shunt_feed_sacch(const uint8_t *l2, int len)
{
    if (!l2 || len < 7) return;
    int n = len < 23 ? len : 23;
    /* trouve le RR header (06 1d / 06 1e) pour valider que c'est bien SI5/SI6 */
    int rr = -1;
    for (int i = 2; i + 1 < n && i < 8; i++)
        if (l2[i] == 0x06 && (l2[i + 1] == 0x1d || l2[i + 1] == 0x1e)) { rr = i; break; }
    if (rr < 0) return;                       /* pas un SI5/SI6 -> ignore */
    uint8_t *s = g_shunt.sacch_buf;
    memcpy(s, l2, n);
    for (int i = n; i < 23; i++) s[i] = 0x2b;
    s[0] = 0x00;                              /* L1 SACCH header : tx_power -> neutre */
    s[1] = 0x00;                              /* L1 SACCH header : TA       -> neutre */
    g_shunt.sacch_have = true;
    g_shunt.sacch_real = true;                /* coupe la fabrication SI3->SI6 */
    static unsigned nf = 0;
    if (nf++ < 20 || (nf % 50) == 0)
        fprintf(stderr, "[dsp-shunt] feed_sacch REEL: SI%d %do (mt=0x%02x) -> sacch_buf\n",
                (l2[rr + 1] == 0x1d) ? 5 : 6, n, l2[rr + 1]);
}

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
        if (type != GSMTAP_TYPE_UM)
            continue;
        /* L2 = buf+16 : [0]=pseudo-len, [1]=PD, [2]=message type.
         * RR PD (0x06) requis pour BCCH/AGCH (SI/IMM-ASS) ; le SDCCH/4 (#2)
         * porte de la LAPDm (buf[17]=controle, PAS un PD RR) -> on n'exige
         * PAS 0x06 pour sub_type 0x07. */
        if (n < GSMTAP_HDR_LEN + 3)
            continue;
        if (sub_type != GSMTAP_CHANNEL_SDCCH4 && sub_type != GSMTAP_CHANNEL_SACCH &&
            buf[GSMTAP_HDR_LEN + 1] != 0x06)
            continue;                               /* pas RR PD (BCCH/AGCH) */
        uint8_t mt = buf[GSMTAP_HDR_LEN + 2];
        if (sub_type == GSMTAP_CHANNEL_BCCH) {
            /* (A) SET BCCH COMPLET (SI1/2/3/4/2bis/2ter), pas juste SI3 — sinon le
             * mobile n'a jamais le set complet ("No sysinfo yet"). feed_si range
             * chaque type dans son slot, dispatch_allc tourne dessus. */
            switch (mt) {
            case 0x19: case 0x1a: case 0x1b:        /* SI1 SI2 SI3 */
            case 0x1c: case 0x1d: case 0x1e:        /* SI4 SI2bis SI2ter */
                break;
            default:
                continue;                           /* paging/SI13... sur BCCH : drop */
            }
            calypso_dsp_shunt_feed_si(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_AGCH) {
            /* (#11) IMM ASSIGN / EXT / REJ + (#SMS) PAGING REQ 1/2/3 -> agch_buf
             * (presente sur bloc CCCH -> firmware chan_nr=0x90 -> gsm48_rr_rx_pch_agch
             * qui dispatch IMM-ASS vs PAGING par msg type). 0x21=PAG_REQ_1 (pas SI13). */
            if (mt == 0x3f || mt == 0x39 || mt == 0x3a ||
                mt == 0x21 || mt == 0x22 || mt == 0x24)
                calypso_dsp_shunt_feed_agch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_SDCCH4) {
            /* (#2) SDCCH/4 SS0 DL (UA/AUTH) -> sdcch_buf (presente sur le bloc
             * SDCCH/4 SS0, fn%51 in {22-25}). LAPDm : aucun filtre message-type
             * (le gate canal = le FN cote si_bridge + le dispatch). */
            calypso_dsp_shunt_feed_sdcch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        } else if (sub_type == GSMTAP_CHANNEL_SACCH) {
            /* SACCH SS0 DL : SI5/SI6 REELS (si_bridge fn%51 {42-45}) -> sacch_buf
             * REEL (presente fn%51 {42-45}). Remplace la fabrication SI3->SI6. */
            calypso_dsp_shunt_feed_sacch(buf + GSMTAP_HDR_LEN, (int)n - GSMTAP_HDR_LEN);
        }
        /* autres canaux : drop */
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
        /* TOA reel : 3e int (>=16 o). Absent (ancien format 12 o) -> on garde 23.
         * Clamp a +-64 qbits autour de 23 : au-dela = gr-gsm desaligne, on retombe
         * sur 23 (on-time) pour ne pas catastropher l'alignement firmware. */
        int32_t toa = 23;
        if (n >= 16) {
            uint32_t toa_le; memcpy(&toa_le, buf + 12, 4);
            toa = (int32_t)le32_to_cpu(toa_le);
            if (toa < 23 - 64 || toa > 23 + 64) toa = 23;
        }
        bool first = !g_shunt.sb_valid;
        g_shunt.sb_bsic  = (uint8_t)(bsic & 0x3f);
        g_shunt.sb_fn    = (uint32_t)fn;
        g_shunt.sb_toa   = (int16_t)toa;
        g_shunt.sb_valid = true;
        static unsigned schlog = 0;
        if (first || schlog++ < 20 || (schlog % 200) == 0)
            fprintf(stderr, "[dsp-shunt] SCH reel (gr-gsm): BSIC=%d "
                    "(ncc=%d bcc=%d) FN=%d TOA=%d%s\n", (int)g_shunt.sb_bsic,
                    (g_shunt.sb_bsic >> 3) & 7, g_shunt.sb_bsic & 7,
                    (int)fn, (int)g_shunt.sb_toa, first ? " [1er]" : "");
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
#ifndef SHM_IQ_LEN
#define SHM_IQ_LEN    320          /* int16 par slot (>= 296 = 148 complexes cs16) */
#endif

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
static FILE                 *g_iq_cfile2;  /* cfile #2 FN-espace (zero-fill) -> test grgsm SACCH */
static int                   g_iq_fd      = -1;   /* fd brut I/Q : fichier ou FIFO live */
static int                   g_iq_is_fifo = 0;    /* 1 = FIFO -> non bloquant + drop */
static char                  g_iq_path[256];      /* chemin memorise pour retry FIFO */
static FILE                 *g_iq_rec;            /* record disque .cfile contigu (rejeu), EN PLUS du live */

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
        cf = "/root/dsp_iq.cfile";
    if (*cf) {
        struct stat st;
        g_iq_is_fifo = (stat(cf, &st) == 0 && S_ISFIFO(st.st_mode));
        snprintf(g_iq_path, sizeof(g_iq_path), "%s", cf);
        if (g_iq_is_fifo) {
            g_iq_fd = open(cf, O_WRONLY | O_NONBLOCK);          /* FIFO : jamais bloquant, pas de create */
            if (g_iq_fd >= 0)
                error_report("[dsp-shunt] I/Q -> %s (FIFO live fc32, non bloquant)", cf);
            else if (errno == ENXIO)
                error_report("[dsp-shunt] FIFO %s sans lecteur — open differe au feed", cf);
            else
                error_report("[dsp-shunt] open(%s) FIFO: %s", cf, strerror(errno));
        } else {
            g_iq_fd = open(cf, O_WRONLY | O_CREAT | O_TRUNC, 0644);   /* cfile rejeu */
            if (g_iq_fd >= 0)
                error_report("[dsp-shunt] enregistre l'I/Q -> %s (cfile fc32)", cf);
            else
                error_report("[dsp-shunt] open(%s) cfile: %s", cf, strerror(errno));
        }
    }
    /* Record disque .cfile contigu (capture brute fc32) EN PLUS de la sortie live :
     * la FIFO sert au live (FFT) sans rien garder, ce record garde tout pour le
     * rejeu deterministe (grgsm_cfile_decode.py). Fichier regulier -> fwrite jamais
     * bloquant. Defaut /dev/shm/dsp_iq.cfile ; CALYPSO_SHUNT_IQ_RECORD= (vide) pour
     * desactiver. On evite le double-open si le record vise le meme fichier que la
     * sortie live (cas live=fichier, pas FIFO). */
    const char *rec = getenv("CALYPSO_SHUNT_IQ_RECORD");
    if (!rec)
        rec = "/dev/shm/dsp_iq.cfile";
    if (*rec && !(g_iq_fd >= 0 && !g_iq_is_fifo && strcmp(rec, g_iq_path) == 0)) {
        g_iq_rec = fopen(rec, "wb");
        if (g_iq_rec)
            error_report("[dsp-shunt] record disque I/Q -> %s (cfile fc32 contigu)", rec);
        else
            error_report("[dsp-shunt] fopen(%s) record: %s", rec, strerror(errno));
    }
    /* cfile #2 : reconstruction FN-espacee (zero-fill des trames manquantes) pour
     * que grgsm retrouve la 51-mf et decode la SACCH (SI5/SI6). Test offline, ne
     * touche PAS au cfile live. Active via CALYPSO_SHUNT_IQ_CFILE2=<chemin>. */
    const char *cf2 = getenv("CALYPSO_SHUNT_IQ_CFILE2");
    if (cf2 && *cf2) {
        g_iq_cfile2 = fopen(cf2, "wb");
        if (g_iq_cfile2)
            error_report("[dsp-shunt] cfile #2 FN-espace -> %s (gap zero-fill)", cf2);
    }
}

/* ENTREE du DSP shunte : la BSP appelle ceci avec l'I/Q DL (cs16, n int16
 * entrelaces I,Q) qu'elle DMA dans la DARAM. Publie dans le shm pour gr-gsm. */
void calypso_dsp_shunt_feed_iq(uint32_t fn, const int16_t *iq, int n)
{
    if (!iq || n <= 0)
        return;
    if (!g_shm && !(shunt_route_c54x() && g_shunt.c54x))
        return;   /* sans shm ET sans route c54x, rien a faire */
    if (n > SHM_IQ_LEN)
        n = SHM_IQ_LEN;
    /* PM REEL : magnitude moyenne (MAV) du burst DL -> g_shunt.last_pm. Pas de
     * sqrt/math.h ; signal-derive, plus de 0x7000 canne. Le dispatch l'ecrit
     * dans a_serv_demod[D_PM] -> rxlev reel cote firmware. */
    {
        uint64_t acc = 0;
        for (int i = 0; i < n; i++) { int v = iq[i]; acc += (v < 0) ? (uint32_t)(-v) : (uint32_t)v; }
        uint32_t mav = (uint32_t)(acc / (uint32_t)n);
        g_shunt.last_pm = (mav > 0xffff) ? 0xffff : (uint16_t)mav;
    }

    /* CALYPSO_DSP=c54x : stash du dernier burst (cs16 I,Q) ; rejoue dans
     * bsp_buf depuis shunt_route_to_c54x() au frame tick. */
    if (shunt_route_c54x() && g_shunt.c54x) {
        int m = (n > SHM_IQ_LEN) ? SHM_IQ_LEN : n;
        memcpy(g_shunt.last_iq, iq, (size_t)m * sizeof(int16_t));
        g_shunt.last_iq_n     = m;
        g_shunt.last_iq_fn    = fn;
        g_shunt.last_iq_valid = true;
    }

    if (g_shm) {
        struct shm_iq_slot *slot = &g_shm->iq[g_shm->iq_wr % SHM_IQ_SLOTS];
        slot->fn = fn;
        slot->n  = (uint32_t)n;
        memcpy(slot->iq, iq, (size_t)n * sizeof(int16_t));
        __sync_synchronize();
        g_shm->iq_wr++;               /* publie le burst (le lecteur poll iq_wr) */
    }

    /* Sorties fc32 (I,Q normalise) : (a) live -> FIFO (FFT, drop) ou fichier, via
     * g_iq_fd ; (b) record disque contigu -> g_iq_rec (rejeu deterministe). fbuf
     * calcule une seule fois et diffuse aux deux. */
    if (g_iq_is_fifo && g_iq_fd < 0)
        g_iq_fd = open(g_iq_path, O_WRONLY | O_NONBLOCK);     /* retry : le lecteur est-il apparu ? */
    if (g_iq_fd >= 0 || g_iq_rec) {
        float fbuf[SHM_IQ_LEN];
        for (int i = 0; i < n; i++)
            fbuf[i] = (float)iq[i] / 32768.0f;
        if (g_iq_fd >= 0) {
            ssize_t w = write(g_iq_fd, fbuf, (size_t)n * sizeof(float));
            if (w < 0 && g_iq_is_fifo && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                /* pipe plein -> drop ce burst (FFT live, perte tolerable) */
            } else if (w < 0 && g_iq_is_fifo && (errno == EPIPE || errno == ENXIO)) {
                close(g_iq_fd); g_iq_fd = -1;   /* lecteur parti -> on reessaiera */
            }
        }
        if (g_iq_rec)                                  /* record disque : jamais bloquant */
            fwrite(fbuf, sizeof(float), (size_t)n, g_iq_rec);
    }
    /* cfile #2 FN-espace : chaque burst TS0 a sa position de trame
     * ((fn-base)*spf int16), trames manquantes zero-fillees -> grgsm retrouve la
     * 51-mf -> SACCH (SI5/SI6) decodable. spf = int16/trame TDMA (def 2500=1x,
     * sweepable via CALYPSO_IQ_CFILE_SPF pour le test offline). */
    if (g_iq_cfile2) {
        static int spf = -1; static uint32_t base_fn = 0; static int64_t pos = 0; static int have_base = 0;
        if (spf < 0) { const char *e = getenv("CALYPSO_IQ_CFILE_SPF"); spf = (e && *e) ? atoi(e) : 2500; }
        if (!have_base) { base_fn = fn; pos = 0; have_base = 1; }
        int64_t target = (int64_t)fn - (int64_t)base_fn;
        if (target < 0) target += 2715648;            /* hyperframe wrap */
        target *= spf;
        int64_t gap = target - pos;
        if (gap < 0 || gap > (int64_t)spf * 300) { base_fn = fn; pos = 0; gap = 0; }  /* rebase si saut anormal */
        static const float zeros[512] = {0};
        while (gap > 0) { int c = gap > 512 ? 512 : (int)gap; fwrite(zeros, sizeof(float), (size_t)c, g_iq_cfile2); pos += c; gap -= c; }
        float fbuf2[SHM_IQ_LEN];
        for (int i = 0; i < n; i++) fbuf2[i] = (float)iq[i] / 32768.0f;
        fwrite(fbuf2, sizeof(float), (size_t)n, g_iq_cfile2);
        pos += n;
    }
    /* cfile #2 FN-espace : chaque burst TS0 a sa position de trame
     * ((fn-base)*spf int16), trames manquantes zero-fillees -> grgsm retrouve la
     * 51-mf -> SACCH (SI5/SI6) decodable. spf = int16/trame TDMA (def 2500=1x,
     * sweepable via CALYPSO_IQ_CFILE_SPF pour le test offline). */
    if (g_iq_cfile2) {
        static int spf = -1; static uint32_t base_fn = 0; static int64_t pos = 0; static int have_base = 0;
        if (spf < 0) { const char *e = getenv("CALYPSO_IQ_CFILE_SPF"); spf = (e && *e) ? atoi(e) : 2500; }
        if (!have_base) { base_fn = fn; pos = 0; have_base = 1; }
        int64_t target = (int64_t)fn - (int64_t)base_fn;
        if (target < 0) target += 2715648;            /* hyperframe wrap */
        target *= spf;
        int64_t gap = target - pos;
        if (gap < 0 || gap > (int64_t)spf * 300) { base_fn = fn; pos = 0; gap = 0; }  /* rebase si saut anormal */
        static const float zeros[512] = {0};
        while (gap > 0) { int c = gap > 512 ? 512 : (int)gap; fwrite(zeros, sizeof(float), (size_t)c, g_iq_cfile2); pos += c; gap -= c; }
        float fbuf2[SHM_IQ_LEN];
        for (int i = 0; i < n; i++) fbuf2[i] = (float)iq[i] / 32768.0f;
        fwrite(fbuf2, sizeof(float), (size_t)n, g_iq_cfile2);
        pos += n;
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

    /* CALYPSO_CANNED : résoudre + ÉNUMÉRER explicitement la dette restante. */
    g_canned = shunt_parse_canned();
    {
        const char *no_canned = getenv("CALYPSO_SHUNT_NO_CANNED");
        error_report("[dsp-shunt] CALYPSO_CANNED (dette fabriquée EXPLICITE) : "
                     "FBDET=%d TOA=%d PM=%d SNR=%d ANGLE=%d CRC=%d  "
                     "[non-canné=valeur réelle/0]. Hors var : BSIC=%s, SI=%s.",
                     !!(g_canned & CAN_FBDET), !!(g_canned & CAN_TOA),
                     !!(g_canned & CAN_PM), !!(g_canned & CAN_SNR),
                     !!(g_canned & CAN_ANGLE), !!(g_canned & CAN_CRC),
                     "réel via gr-gsm (fallback 63 si pas no-canned)",
                     (no_canned && *no_canned == '1')
                        ? "réel via feed_si (no-canned, gate si absent)"
                        : "réel via feed_si (+ fallback legacy possible)");
    }

    error_report("[dsp-shunt] active — c54x emulator should be skipped, "
                 "BSP DMA→DARAM should be gated. Watch /tmp/qemu.log for "
                 "LATCH/DISPATCH lines.");
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
    /* SI3 (slot 2) -> SEED SI6 fabrique (B4) pour la SACCH dediee, UNIQUEMENT en
     * fallback tant qu'aucun SI5/SI6 REEL n'est arrive (g_shunt.sacch_real). Des
     * que feed_sacch recoit le vrai SI5/SI6 (grgsm), sacch_real=true et ce bloc
     * ne tourne plus -> le SI3 du BCCH ne clobbe plus le SACCH reel. Le seed evite
     * le 'Short header 0x07 unsupported' au tout debut d'un canal dedie (avant que
     * grgsm ait decode la 1ere SACCH ~480ms). */
    if (slot == 2 && n >= 10 && !g_shunt.sacch_real) {
        uint8_t *s6 = g_shunt.sacch_buf;
        memset(s6, 0x2b, sizeof(g_shunt.sacch_buf));
        /* Layout B4 reel (lapdm.c) : header L1 SACCH (2o, non strippe par la L1
         * osmocom-bb) + LAPDm addr + LAPDm control UI (-> fmt B4, l3len=19) + L3. */
        s6[0] = 0x00;                          /* L1 SACCH : tx_power */
        s6[1] = 0x00;                          /* L1 SACCH : TA */
        s6[2] = 0x03;                          /* LAPDm address : SAPI0, C/R, EA=1 */
        s6[3] = 0x03;                          /* LAPDm control : UI -> format B4 */
        s6[4] = (uint8_t)((11 << 2) | 0x01);   /* L3 pseudo-length L=11 */
        s6[5] = 0x06;                          /* RR PD, skip=0 */
        s6[6] = 0x1e;                          /* SYSTEM INFORMATION TYPE 6 */
        s6[7] = l2[3]; s6[8] = l2[4];          /* cell identity (SI3 @3..4) */
        s6[9]  = l2[5]; s6[10] = l2[6]; s6[11] = l2[7];
        s6[12] = l2[8]; s6[13] = l2[9];        /* LAI (SI3 @5..9) */
        s6[14] = 0x0f;                         /* cell options : radio-link-timeout long */
        s6[15] = 0xff;                         /* NCC permitted : tous */
        /* [16..22] = 0x2b rest octets (l3 total = [4..22] = 19o) */
        g_shunt.sacch_have = true;
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

/* CALYPSO_DSP=c54x : relie le handle du VRAI DSP (depuis calypso_mb.c). */
void calypso_dsp_shunt_set_c54x(C54xState *s)
{
    g_shunt.c54x = s;
}

/* Predicat dedie : shunt actif ET route c54x demandee. Utilise par
 * calypso_trx.c pour autoriser la DMA page->DARAM en mode c54x sans
 * reactiver le c54x_run du trx (le shunt possede c54x_run). */
bool calypso_dsp_shunt_route_c54x_active(void)
{
    return g_shunt.active && shunt_route_c54x();
}
