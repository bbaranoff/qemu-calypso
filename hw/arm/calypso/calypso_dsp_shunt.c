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
#include "calypso_layer1.h" /* calypso_l1_c_active() : ungate SB/SI (+FB) sous CALYPSO_L1=c */
#include "hw/arm/calypso/calypso_dsp_internal.h" /* shared state + NDB-write primitives (split) */
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

/* FN TDMA reelle (calypso_trx.c) pour recoder la FN du shunt (LATCH d_fn=0) :
 * declaree dans calypso_dsp_internal.h (partagee avec le helper). */
extern void l1ctl_inject_dl_si(const uint8_t *l2, int l2len, uint32_t fn);
/* FN-FIX : FN du dernier L1CTL_RACH_CONF (= memo exact du mobile), capture dans
 * l1ctl_sock.c au moment de l'envoi au mobile (race-free vs last_rach.fn@0x836500). */
extern volatile uint32_t g_last_rach_conf_fn;
extern volatile uint32_t g_rach_conf_fn[256];   /* per-ra : FN exact du RACH_CONF keye par ra (defini l1ctl_sock.c) */

struct dsp_shunt_state g_shunt;

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
            SHUNT_LOG("SDCCH-UL%s task_u=0x%04x l1s%%51=%u "
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

    SHUNT_LOG("LATCH page=%u task_md=%u task_d=%u task_u=%u task_ra=%u fn=%u\n",
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
/* ---- CALYPSO_CANNED : énumère EXPLICITEMENT chaque sortie DSP encore
 * FABRIQUÉE (canned) par le shunt, au lieu de la cacher derrière une valeur
 * « plausiblement juste ». CSV insensible casse ; "FULL"/"ALL" = tout canné,
 * "NONE"/"=" vide = rien. Var absente = DÉFAUT = CAN_DEFAULT (désormais RIEN
 * canné : toutes les sorties sont pilotées par le vrai décode gr-gsm). On
 * re-canne sélectivement avec CALYPSO_CANNED=<token>. BSIC/SI ne sont PAS ici :
 * déjà réels via gr-gsm / feed_si ; leur état est loggué au boot. */
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

unsigned g_canned = CAN_DEFAULT;   /* résolu dans calypso_dsp_shunt_init */

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
        else SHUNT_ERR("CALYPSO_CANNED: token inconnu '%s' ignore", t);
    }
    return m;
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

static void shunt_dispatch_nb(uint8_t page_idx, uint16_t task_d)
{
    /* TODO : NB DL = decoded BCCH/CCCH burst payload into NDB a_cd[].
     * NB UL = consume burst bits from DARAM for TX (forwarded to bridge). */
    SHUNT_LOG("DISPATCH NB page=%u task_d=%u (TODO)\n",
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
/* Lecture DIRECTE de l'espace data[] du c54x pour une adresse API ARM,
 * SANS round-trip MMIO calypso_dsp_read (qui prend calypso_pcb_daram_lock,
 * mutex non-recursif -> re-lock/abort quand on est deja dans le contexte
 * frame-tick). Meme mapping que calypso_dsp_read : ARM off O -> data[O/2+0x800]. */
static inline uint16_t shunt_c54x_api_rd(C54xState *dsp, uint32_t arm_addr)
{
    return dsp->data[((arm_addr - 0xFFD00000UL) >> 1) + 0x0800];
}

static void shunt_route_to_c54x(uint8_t page_idx)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;
    fprintf(stderr, "[c54x-route] enter page=%u dsp=%p\n", (unsigned)page_idx, (void*)dsp);

    /* (a) API write-page -> DARAM 0x0586 (replique de la DMA trx gatee a :711).
     * wp_base(page_idx) = adresse MMIO absolue de la write-page (== dsp_ram).
     * Le mot d_dsp_page (NDB+0 = 0xFFD001A8) est lu live (= s->dsp_ram[0x01A8/2]
     * cote trx) pour data[0x0584] et le mirror 0x08D4. */
    {
        uint32_t wbase    = wp_base(page_idx);
        fprintf(stderr, "[c54x-route] a1 wbase=0x%08x\n", wbase);
        uint16_t dsp_page = shunt_c54x_api_rd(dsp, BASE_API_NDB + NDB_D_DSP_PAGE);
        fprintf(stderr, "[c54x-route] a2 dsp_page=0x%04x data=%p api_ram=%p\n", dsp_page, (void*)dsp->data, (void*)dsp->api_ram);
        dsp->data[0x0584] = dsp_page;
        dsp->data[0x0585] = (uint16_t)(g_shunt.d_fn & 0xFFFF);
        fprintf(stderr, "[c54x-route] a3 data-hdr-ok\n");
        for (int i = 0; i < 20; i++)
            dsp->data[0x0586 + i] = shunt_c54x_api_rd(dsp, wbase + (uint32_t)i * 2);
        fprintf(stderr, "[c54x-route] a4 wp-copy-ok\n");
        /* mirror d_dsp_page cote DSP (le firmware le lit a api_ram 0x08D4). */
        if (dsp->api_ram)
            dsp->api_ram[0x08D4 - C54X_API_BASE] = dsp_page;
    }

    fprintf(stderr, "[c54x-route] a-daram-ok\n");
    /* (b) rejoue le dernier burst I/Q (cs16 entrelace I,Q) dans bsp_buf. */
    if (g_shunt.last_iq_valid && g_shunt.last_iq_n > 0)
        c54x_bsp_load(dsp, (const uint16_t *)g_shunt.last_iq, g_shunt.last_iq_n);

    fprintf(stderr, "[c54x-route] b-bsp-load-ok n=%d\n", g_shunt.last_iq_n);
    /* (c) INT3 FRAME + wake : reveille le DSP s'il etait idle/halt. */
    g_c54x_int3_src = 3;
    c54x_interrupt_ex(dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
    c54x_wake(dsp);
    /* revive: c54x_run loop gate = (running && !idle). c54x_wake ne clear que
     * idle ; en mode route_c54x le chemin trx qui posait running=true est gate
     * off -> forcer running ici sinon la boucle c54x_run est sautee (0 insn). */
    dsp->running = true;

    fprintf(stderr, "[c54x-route] c-wake-ok running=%d idle=%d\n", dsp->running, dsp->idle);
    /* (d) execute le budget (1 trame nominale ~256000 insns ; ajustable env). */
    {
        static int budget = -1;
        if (budget < 0) {
            const char *b = getenv("CALYPSO_DSP_BUDGET");
            budget = (b && *b) ? atoi(b) : 256000;
            if (budget <= 0) budget = 256000;
        }
        fprintf(stderr, "[c54x-route] d-pre-c54x_run budget=%d\n", budget);
        c54x_run(dsp, budget);
        fprintf(stderr, "[c54x-route] d-c54x_run-RETURNED\n");
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
        /* CALYPSO_DSP=c54x : overlay des écritures NDB gr-gsm (rxlev/FB/SB/SI réels)
         * par-dessus le poison 0x70c4 du c54x -> le mobile campe et fait sa LU.
         * Le RUN du VRAI c54x (route_to_c54x -> c54x_run) est OPT-IN car le revival
         * est encore instable (crash qemu). Défaut : overlay seul = réception via le
         * shunt, aucun c54x exécuté, pas de crash. CALYPSO_DSP_RUN_C54X=1 pour le lancer. */
        {
            static int run_c54x = -1;
            if (run_c54x < 0) {
                const char *e = getenv("CALYPSO_DSP_RUN_C54X");
                run_c54x = (e && *e == '1') ? 1 : 0;
                fprintf(stderr, "[c54x-gate] getenv RUN_C54X=%s CRASHPC=%s DSP=%s -> run_c54x=%d\n",
                        e ? e : "(null)",
                        getenv("CALYPSO_C54X_CRASHPC") ? getenv("CALYPSO_C54X_CRASHPC") : "(null)",
                        getenv("CALYPSO_DSP") ? getenv("CALYPSO_DSP") : "(null)", run_c54x);
            }
            if (run_c54x) shunt_route_to_c54x(page);
        }
        if (md == PM_DSP_TASK)                          shunt_dispatch_pm(page);
        else if (md == FB_DSP_TASK)                     shunt_dispatch_fb(page);
        else if (md == SB_DSP_TASK && g_shunt.sb_valid) shunt_dispatch_sb(page);
        if (td == ALLC_DSP_TASK)                        shunt_dispatch_allc(page);
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
                    SHUNT_LOG("feed_agch: PAGING mt=0x%02x DROP "
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
                  SHUNT_LOG("FN-FIX probe RA=0x%02x "
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
                    SHUNT_LOG("FN-FIX req-ref RA=0x%02x reecrite -> "
                            "fn=%u (T1'=%u T2=%u T3=%u) adj=%d [last_rach.fn]\n",
                            ra, (uint32_t)fn, t1p, t2, t3, rr_adj);
            }
        }
    }

    g_shunt.agch_valid = true;
    g_shunt.agch_tick  = g_shunt.tick_cnt;
    SHUNT_LOG("feed_agch: IMM-ASS mt=0x%02x -> agch_buf "
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
    SHUNT_LOG("feed_sdcch: SDCCH/4 SS0 DL a0=0x%02x c=0x%02x "
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
        SHUNT_LOG("feed_sacch REEL: SI%d %do (mt=0x%02x) -> sacch_buf\n",
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
        SHUNT_ERR("GSMTAP socket() failed: %s", strerror(errno));
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
        SHUNT_ERR("GSMTAP bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_gsmtap_fd = fd;
    qemu_set_fd_handler(fd, shunt_gsmtap_read, NULL, NULL);
    SHUNT_ERR("GSMTAP listener udp:127.0.0.1:%d → feed_si(a_cd) "
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
            SHUNT_LOG("SCH reel (gr-gsm): BSIC=%d "
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
        SHUNT_ERR("SCH socket() failed: %s", strerror(errno));
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
        SHUNT_ERR("SCH bind(:%d) failed: %s", port, strerror(errno));
        close(fd);
        return;
    }
    g_sch_fd = fd;
    qemu_set_fd_handler(fd, shunt_sch_read, NULL, NULL);
    SHUNT_ERR("SCH listener udp:127.0.0.1:%d → feed_sb(BSIC/FN reels "
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
        SHUNT_ERR("shm_open(%s): %s", SHM_NAME, strerror(errno));
        return;
    }
    if (ftruncate(fd, sizeof(struct dsp_shunt_shm)) != 0) {
        SHUNT_ERR("ftruncate shm: %s", strerror(errno));
        close(fd);
        return;
    }
    void *m = mmap(NULL, sizeof(struct dsp_shunt_shm),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED) {
        SHUNT_ERR("mmap shm: %s", strerror(errno));
        return;
    }
    g_shm = m;
    g_shm->magic = 0x43445350;
    g_shm_last_si_seq = g_shm->si_seq;
    SHUNT_ERR("shm %s (=/dev/shm%s, %zu o) : I/Q in (feed_iq->gr-gsm) "
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
                SHUNT_ERR("I/Q -> %s (FIFO live fc32, non bloquant)", cf);
            else if (errno == ENXIO)
                SHUNT_ERR("FIFO %s sans lecteur — open differe au feed", cf);
            else
                SHUNT_ERR("open(%s) FIFO: %s", cf, strerror(errno));
        } else {
            g_iq_fd = open(cf, O_WRONLY | O_CREAT | O_TRUNC, 0644);   /* cfile rejeu */
            if (g_iq_fd >= 0)
                SHUNT_ERR("enregistre l'I/Q -> %s (cfile fc32)", cf);
            else
                SHUNT_ERR("open(%s) cfile: %s", cf, strerror(errno));
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
            SHUNT_ERR("record disque I/Q -> %s (cfile fc32 contigu)", rec);
        else
            SHUNT_ERR("fopen(%s) record: %s", rec, strerror(errno));
    }
    /* cfile #2 : reconstruction FN-espacee (zero-fill des trames manquantes) pour
     * que grgsm retrouve la 51-mf et decode la SACCH (SI5/SI6). Test offline, ne
     * touche PAS au cfile live. Active via CALYPSO_SHUNT_IQ_CFILE2=<chemin>. */
    const char *cf2 = getenv("CALYPSO_SHUNT_IQ_CFILE2");
    if (cf2 && *cf2) {
        g_iq_cfile2 = fopen(cf2, "wb");
        if (g_iq_cfile2)
            SHUNT_ERR("cfile #2 FN-espace -> %s (gap zero-fill)", cf2);
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
    /* Actif si CALYPSO_DSP_SHUNT=1 OU CALYPSO_L1=c : dans ce dernier cas le HLE
     * (calypso_layer1.c) pilote le FB, mais SB (a_sch) + SI (a_cd) n'existent que
     * dans le shunt -> on l'arme aussi pour fournir le chemin réception prouvé
     * (FB+SB+SI) qui va jusqu'au LU accept. Le shunt on_frame_tick tourne ~1ms
     * après le tick L1=c, donc ses écritures d_fb_det/a_sch/a_cd priment. */
    const char *env = getenv("CALYPSO_DSP_SHUNT");
    bool shunt_env_on = (env && strcmp(env, "1") == 0);
    if (!shunt_env_on && !calypso_l1_c_active() && !shunt_route_c54x()) {
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
        SHUNT_ERR("CALYPSO_CANNED (dette fabriquée EXPLICITE) : "
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

    SHUNT_ERR("active — c54x emulator should be skipped, "
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
    SHUNT_LOG("feed_si: SI réel %d o injecté → a_cd "
            "(L2[0..2]=%02x %02x %02x)\n", n, l2[0],
            n > 1 ? l2[1] : 0, n > 2 ? l2[2] : 0);
}

/* Public getter — gate condition for BSP/TPU DMA into DARAM. */
bool calypso_dsp_shunt_active(void)
{
    return g_shunt.active;
}

/* CALYPSO_DSP=c54x : relie le handle du VRAI DSP (depuis calypso_mb.c). */
static bool g_c54x_early_booted = false;
bool calypso_dsp_shunt_early_booted(void) { return g_c54x_early_booted; }

void calypso_dsp_shunt_set_c54x(C54xState *s)
{
    g_shunt.c54x = s;

    /* [c54x-earlyboot] FIX race d'ordre golive (2026-07-20, mode B).
     * Root cause : l'ARM poste le golive (data[0x0fff]=cmd 2/4, data[0x0ffe]=entry)
     * a fn=0/+0.073s, MAIS le c54x ne bootait qu'a +5.6s (1er shunt_route_to_c54x)
     * -> son init-IDLE a 0xb419 (ST #1,*0xfff) ecrasait le 0x0002 de l'ARM -> spin
     * eternel a 0xb41c. Etat PERSISTE entre wakes (verifie : 0xb419 ne tourne
     * qu'une fois, insn accumule, meme objet DSP). Fix = booter le c54x ICI
     * (machine-init, AVANT que le vCPU ARM tourne -> AVANT le golive), pour qu'il
     * pose son IDLE et se parke a 0xb41c AVANT l'ecriture ARM. 0xb419 ne re-tourne
     * plus (PC persiste) -> le 0x0002 survit -> le 1er wake shunt le consomme ->
     * golive natif (le firmware fait son propre RSBX INTM). Zero FORCE_ : on force
     * le QUAND du boot, aucune valeur de mailbox. One-shot, gate mode revive. */
    if (s && shunt_route_c54x()) {
        static int rc = -1;
        if (rc < 0) { const char *e = getenv("CALYPSO_DSP_RUN_C54X"); rc = (e && *e == '1') ? 1 : 0; }
        if (rc) {
            uint16_t pc0 = s->pc;
            s->running = true;
            c54x_run(s, 2000);   /* reset(0xff80) -> 0xb419 (pose IDLE) -> park 0xb41c */
            if (s->pc >= 0xb41c && s->pc <= 0xb428) {
                g_c54x_early_booted = true;   /* gate le re-reset trx:701 */
                fprintf(stderr, "[c54x-earlyboot] PARK pc=0x%04x (de 0x%04x) insn=%u "
                        "data[0x0fff]=0x%04x data[0x0ffe]=0x%04x (attendu IDLE 0x0001)\n",
                        s->pc, pc0, s->insn_count, s->data[0x0fff], s->data[0x0ffe]);
            } else
                fprintf(stderr, "[c54x-earlyboot] WARN pas parque pc=0x%04x insn=%u "
                        "-> B invalide, basculer sur A (execution continue)\n",
                        s->pc, s->insn_count);
        }
    }
}

/* Predicat dedie : shunt actif ET route c54x demandee. Utilise par
 * calypso_trx.c pour autoriser la DMA page->DARAM en mode c54x sans
 * reactiver le c54x_run du trx (le shunt possede c54x_run). */
bool calypso_dsp_shunt_route_c54x_active(void)
{
    return g_shunt.active && shunt_route_c54x();
}
