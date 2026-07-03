/*
 * calypso_trx.c — Calypso hardware emulation + DSP C54x emulation
 * No sockets. Firmware speaks UART only. DSP results in shared RAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "calypso_arm2dsp.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"          /* runstate_is_running() — gate DSP tick on ARM halt */
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_c54x.h"
extern int g_c54x_int3_src;  /* diag source INT3 (RO) */
#include "hw/arm/calypso/calypso_full_pcb.h"  /* api_ram_lock pour MTTCG race fix */
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_twl3025.h"
#include "hw/arm/calypso/calypso_sim.h"
#include "hw/arm/calypso/calypso_fbsb.h"
#include "hw/arm/calypso/calypso_layer1.h"   /* HLE L1 scaffold (CALYPSO_L1=c) */
#include "calypso_orch.h"
#include "chardev/char-fe.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

extern CalypsoUARTState *g_uart_modem;
extern CalypsoUARTState *g_uart_irda;
/* calypso_dsp_shunt_record_rach() : prototype dans calypso_dsp_shunt.h */

#include "hw/arm/calypso/calypso_debug.h"
#define TRX_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TRX")) \
        fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__); } while (0)

/* CALYPSO_TIMER=1 enables timer-side fprintf tracing (frame_irq, tdma_tick,
 * kick). =0 (default) drops the calls entirely so the run is silent and
 * stderr-pipe backpressure (TSLOG → python flush-per-line) can't throttle
 * the TCG main thread. Cached once via getenv. */
static bool calypso_timer_log(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = cdbg_env("TIMER");
        on = (e && (*e == '1' || *e == 'y')) ? 1 : 0;
    }
    return on;
}

#define DSP_API_W_PAGE0  0x0000
#define DSP_API_W_PAGE1  0x0028
#define DSP_API_NDB      0x01A8
#define DB_W_D_TASK_D    0
#define DB_W_D_BURST_D   1
#define DB_W_D_TASK_U    2
#define DB_W_D_BURST_U   3
#define DB_W_D_TASK_MD   4
#define DB_W_D_BACKGROUND 5
#define DB_W_D_DEBUG     6
#define DB_W_D_TASK_RA   7   /* RACH access task — separate from d_task_u */
/* No PM/FB/SB stubs — the DSP handles everything via shared API RAM */

typedef struct CalypsoTRX {
    qemu_irq *irqs;
    MemoryRegion dsp_iomem;
    uint16_t     dsp_ram[CALYPSO_DSP_SIZE / 2];
    uint8_t      dsp_page;
    bool         dsp_booted;
    uint32_t     boot_frame;
    MemoryRegion tpu_iomem;
    MemoryRegion tpu_ram_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[CALYPSO_TPU_RAM_SIZE / 2];
    MemoryRegion tsp_iomem;
    uint16_t     tsp_regs[CALYPSO_TSP_SIZE / 2];
    MemoryRegion ulpd_iomem;
    uint16_t     ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t     ulpd_counter;
    MemoryRegion sim_iomem;
    CalypsoSim  *sim;
    QEMUTimer   *tdma_timer;
    QEMUTimer   *frame_irq_timer;
    QEMUTimer   *dsp_timer;
    uint32_t     fn;
    bool         tdma_running;

    /* C54x DSP emulator */
    C54xState   *dsp;
    bool         dsp_init_done;  /* DSP reached first IDLE after boot */

    /* CLK UDP: send each TDMA tick to bridge so it's clock-slave */
    int          clk_fd;
    struct sockaddr_in clk_peer;
} CalypsoTRX;

static CalypsoTRX *g_trx;

#include "qemu/atomic.h"
#include "calypso_dsp_shunt.h"

/* FBSB host-side orchestration. Reintroduced after preNoCell refactor
 * (28 Apr) accidentally removed the wire. The bridge delivers I/Q from
 * a fixed cos/sin LUT (no AFC DAC feedback in QEMU), so the DSP
 * correlator cannot converge across iterations. This wire publishes
 * synthetic clean FB/SB results at the NDB level when ARM dispatches
 * FB_DSP_TASK, allowing the L1→L2→L3 stack to progress toward Location
 * Update without requiring physical RF AFC simulation. */
static CalypsoFbsb g_fbsb;
static bool        g_fbsb_inited;
/* Définis dans calypso_c54x.c — posés ici quand l'ARM écrit d_task_md=5,
 * lus par la sonde D_TASK_MD-RD (test H1 timing/EA write-vs-read). */
extern uint32_t g_arm_taskmd5_insn;
extern uint16_t g_arm_taskmd5_ea;

/* All firmware patches removed — verified that the layer1.highram.elf
 * runs unmodified against the current QEMU emulation (PM scan, FBSB,
 * RESET cycle stable for >1 minute with NO patches applied).
 *
 * History — patches removed and why each was actually unnecessary:
 *   cons_puts NOP (0x82a1b0)  : function has a UART fall-through path
 *                                taken when its LCD ctx flag is 0 (the
 *                                default). printf_buffer is filled by
 *                                vsnprintf upstream and read by the
 *                                fw_console poller in fw_console.c.
 *   puts NOP (0x829ea0)        : puts is a one-instruction tail call to
 *                                sercomm_puts; it was never broken.
 *   5x BL NOP in frame_irq     : these are bl printf / bl puts calls
 *                                that became safe once cons_puts/puts
 *                                were left alone.
 *   talloc pool 32->148        : pool exhaustion never observed in the
 *                                current run profile.
 *   talloc retry loop          : same — never reached.
 *   abort_irqs inf-loop fixup  : handle_abort never entered with the
 *                                IRQ controller fixes from earlier
 *                                sessions.
 *   sim_handler -> BX LR       : l1a_l23_handler progresses through SIM
 *                                polling without blocking under the
 *                                current SIM register stub responses.
 *
 * If any of these regress, look first at the underlying QEMU subsystem
 * (LCD MMIO, talloc memory pool, IRQ controller, SIM stub) rather than
 * re-introducing a firmware patch.
 */

uint32_t calypso_trx_get_fn(void) { return g_trx ? g_trx->fn : 0; }

/* Real-ROM mode = CALYPSO_DSP_REG_MODE=c54x : the genuine DSP ROM derives its own
 * state and must drive the bootloader handshake itself. bin/hybrid (default) inject a
 * captured register snapshot (mock-ish) and the bridge fakes the handshake. Memoized,
 * init-order-safe (getenv) — matches calypso_c54x.c reg_mode==0. Only the
 * execution-affecting handshake fakes gate on this (snapshot-injection additionally
 * needs a .bin, i.e. reg_init_valid — not conflated here). */
static bool dsp_real_rom_mode(void)
{
    static int v = -1;
    if (v < 0) {
        /* The bridge fakes the boot handshake ONLY when NO real DSP ROM services it.
         * Runtime fact (qemu.log: "[dsp-shunt] active — c54x emulator should be
         * skipped"): CALYPSO_DSP_SHUNT=1 SKIPS the c54x entirely (gr-gsm does the
         * demod) → no ROM writes IDLE → the bridge MUST keep faking, else the
         * firmware's dsp_bl_wait_ready hangs → mobile breaks. SHUNT=0 → the genuine
         * c54x ROM runs and drives the real handshake → drop the fake. So the right
         * predicate is SHUNT, NOT CALYPSO_DSP / REG_MODE (both default to c54x even
         * in the shunt/mock run where the c54x is skipped). getenv → init-order-safe. */
        const char *sh = getenv("CALYPSO_DSP_SHUNT");
        bool shunt_on = sh && sh[0] == '1';
        v = shunt_on ? 0 : 1;
    }
    return v != 0;
}

/* ---- DSP API RAM ---- */
static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;

    /* === CO-RUN (fix scheduling, mech 2) — 2026-06-23 ===
     * Le firmware spin sur BL_CMD_STATUS (0x0ffe) dans dsp_bl_wait_ready
     * (while readw(0x0ffe) != IDLE=1). Ce spin tight côté ARM AFFAME le tick QEMU
     * qui exécute c54x_run → la ROM n'avance pas → n'écrit jamais son IDLE @0xb419 →
     * le firmware spin éternel (le read-fake ne fire qu'UNE fois, dsp_booted latch).
     * On fait avancer la ROM ICI, à chaque lecture de la cellule : elle atteint
     * 0xb419 (write IDLE=1) puis spin en la lisant → le firmware lit un VRAI IDLE →
     * sort de TOUS ses wait_ready → atteint dsp_bl_start_at (write cmd 2 @82be68) →
     * la ROM lit cmd 2 @0xb424 → BACC vers L1. Gaté revival (SHUNT=0), fake gardé. */
    if (offset == 0x0ffe && dsp_real_rom_mode()
        && s->dsp && s->dsp->running) {
        c54x_run(s->dsp, 1024);
    }

    /* === BL-READ (diag GAP-0) : l'ARM lit BL_CMD_STATUS (0x0ffe) en boucle dans
     * dsp_bl_wait_ready (while readw(0x0ffe) != IDLE). Voir la valeur réellement
     * lue (cell, APRÈS le co-run) + l'état du fake → tracer si le firmware sort du
     * spin (lit IDLE=1) et continue vers dsp_bl_start_at. Capé. Gaté revival. */
    if (offset == 0x0ffe && dsp_real_rom_mode()) {
        static unsigned blr = 0;
        if (blr < 120) { blr++;
            fprintf(stderr, "[trx] BL-READ #%u BL_CMD_STATUS(0x0ffe) cell=0x%04x booted=%d fn=%u\n",
                    blr, (unsigned)s->dsp_ram[offset/2], s->dsp_booted, s->fn);
        }
    }

    /* === Hypothesis #4 probe : ARM reads R_PAGE_X (= DSP responses) ===
     * ARM lit a_pm via R_PAGE_X. R_PAGE_0 = 0x0050, R_PAGE_1 = 0x0078.
     * Si firmware lit toujours R_PAGE_0 (jamais R_PAGE_1) → r_page jamais
     * flipped → reading garbage from previous page après DSP write.
     * Gated par CALYPSO_DEBUG=R_PAGE_SPLIT. */
    if (calypso_debug_enabled("R_PAGE_SPLIT")) {
        bool is_r0 = (offset >= 0x0050 && offset < 0x0078);
        bool is_r1 = (offset >= 0x0078 && offset < 0x00A0);
        if (is_r0 || is_r1) {
            static unsigned r0_count = 0, r1_count = 0;
            if (is_r0) r0_count++; else r1_count++;
            if ((r0_count + r1_count) <= 30 || ((r0_count + r1_count) % 500) == 0) {
                fprintf(stderr,
                    "[calypso-trx] R_PAGE_SPLIT r0=%u r1=%u (last off=0x%04x fn=%u)\n",
                    r0_count, r1_count, (unsigned)offset, s->fn);
            }
        }
    }

    /* === FIX 2026-05-15 : DSP→ARM mirror was missing ===
     *
     * Bug : `s->dsp_ram[]` et `s->dsp->data[]` sont deux arrays distincts.
     * Le write path (calypso_dsp_write line 258) mirror ARM→DSP, mais le
     * read path lisait seulement dsp_ram[] → toutes les écritures DSP étaient
     * invisibles pour ARM. Verrouille tout le projet depuis ~6 mois :
     * d_fb_det reste vu à 0 par firmware → FBSB_CONF=FAIL → mobile coincé.
     *
     * Fix : lire depuis dsp->data[] qui est la source de vérité (DSP writes
     * via opcode + ARM writes mirrorés par calypso_dsp_write).
     * Fallback sur dsp_ram[] si s->dsp pas encore alloué (pre-realize). */
    /* Sous lock daram_lock pour la lecture cohérente vs DSP-thread writes.
     * src est un pointeur DANS dsp->data[] ; on copie la valeur sous lock
     * puis on relâche avant le reste de la logique pour minimiser la
     * section critique. */
    uint64_t val;
    if (s->dsp && s->dsp->data) {
        calypso_pcb_daram_lock_acquire();
        uint16_t *src = &s->dsp->data[offset/2 + 0x0800];
        val = (size == 2) ? src[0] :
              (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
              ((uint8_t *)src)[offset & 1];
        calypso_pcb_daram_lock_release();
    } else {
        uint16_t *src = &s->dsp_ram[offset/2];
        val = (size == 2) ? src[0] :
              (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
              ((uint8_t *)src)[offset & 1];
    }
    /* === FBDET-RD (revival dsp) : l'ARM lit-il d_fb_det (offset 0x01F0) != 0 =
     * le résultat FB écrit par le DSP (data[0x08F8]=0x6b34) ? Log sur CHANGEMENT
     * de valeur (capté 100) → on voit la transition 0 -> FOUND. Lit la vraie
     * valeur (avant tout FORCE_TOA). */
    if (size == 2 && offset == 0x01F0) {
        static uint16_t fbdet_last = 0xFFFF;
        static unsigned fbdet_log;
        if ((uint16_t)val != fbdet_last && (fbdet_log < 100 || (fbdet_log % 20) == 0)) {
            fprintf(stderr,
                    "[FBDET-RD] ARM read d_fb_det(0x01F0)=0x%04x fn=%u\n",
                    (unsigned)val, (unsigned)s->fn);
            fbdet_last = (uint16_t)val;
            fbdet_log++;
        }
    }
    /* === FBSBRES-RD (read-only, 2026-06-22) : logue le BLOC RESULTAT FB et SB
     * lu par l'ARM, AVANT tout FORCE_TOA, sur les offsets MMIO CONFIRMES par
     * VerifyOffsets (identiques aux dérivés ; convention DSP word = 0x0800 +
     * off/2) :
     *   FB a_sync_demod (NDB) : D_TOA 0x01F4 / D_PM 0x01F6 / D_ANGLE 0x01F8 /
     *                           D_SNR 0x01FA
     *   SB a_serv_demod[D_TOA] : page0 0x0060 / page1 0x0088
     *   SB a_sch (BSIC/CRC) : a_sch[0] CRC 0x006E(p0)/0x0096(p1),
     *                         a_sch[3] 0x0074(p0)/0x009C(p1),
     *                         a_sch[4] 0x0076(p0)/0x009E(p1)
     * Permet de voir si ces words portent une valeur valide ou du garbage
     * (cause "SB N bits in the future"). Compteur capé ~150, ne spamme pas. */
    if (size == 2) {
        static unsigned fbsbres_log = 0;
        const char *tag = NULL;
        switch (offset) {
        /* --- bloc FB (a_sync_demod, NDB) --- */
        case 0x01F4: tag = "FB a_sync_demod[D_TOA]";   break;
        case 0x01F6: tag = "FB a_sync_demod[D_PM]";    break;
        case 0x01F8: tag = "FB a_sync_demod[D_ANGLE]"; break;
        case 0x01FA: tag = "FB a_sync_demod[D_SNR]";   break;
        /* --- bloc SB (a_serv_demod[D_TOA], db_r page0/page1) --- */
        case 0x0060: tag = "SB a_serv_demod[D_TOA] p0"; break;
        case 0x0088: tag = "SB a_serv_demod[D_TOA] p1"; break;
        /* --- bloc SB BSIC / CRC (a_sch, db_r page0/page1) --- */
        case 0x006E: tag = "SB a_sch[0] CRC p0"; break;
        case 0x0096: tag = "SB a_sch[0] CRC p1"; break;
        case 0x0074: tag = "SB a_sch[3] BSIC p0"; break;
        case 0x009C: tag = "SB a_sch[3] BSIC p1"; break;
        case 0x0076: tag = "SB a_sch[4] BSIC p0"; break;
        case 0x009E: tag = "SB a_sch[4] BSIC p1"; break;
        default: break;
        }
        if (tag) {
            /* SB (db_r 0x60..0x9E) = rare -> cap haut ; FB (a_sync_demod) =
             * frequent -> 150 premiers + echantillon 1/2000 pour voir le
             * steady-state (le TOA converge-t-il apres le fix DL FN-LOCK ?). */
            static unsigned sb_log = 0;
            bool is_sb = (offset >= 0x0060 && offset <= 0x009E);
            bool emit = is_sb ? (sb_log < 3000)
                              : (fbsbres_log < 150 || (fbsbres_log % 2000) == 0);
            if (is_sb) sb_log++; else fbsbres_log++;
            if (emit)
                fprintf(stderr, "[FBSBRES-RD] %s(0x%04x)=0x%04x fn=%u\n",
                        tag, (unsigned)offset, (unsigned)val, (unsigned)s->fn);
        }
    }
    /* CALYPSO_FORCE_TOA=<N> (env gated, rigolo) : force une détection FB
     * complète vue par l'ARM, sans toucher le DSP. osmocom prim_fbsb.c
     * n'atteint read_fb_result (lecture TOA dans ndb->a_sync_demod[D_TOA]
     * @0x01F4) QU'APRÈS que d_fb_det (@0x01F0) = "FOUND". Donc forcer le TOA
     * seul ne suffit pas : on force tout le bloc résultat FB sur le read ARM.
     *   0x01F0 d_fb_det = 1 (FOUND)   0x01F4 a_sync_TOA  = N (23 = on-time)
     *   0x01F8 a_sync_ANGLE = 0 (AFC ne diverge pas)  0x01FA a_sync_SNR = haut */
    /* Étendu 2026-06-02 : FORCE_TOA force le bloc FB (a_sync_demod @0x01F0-FA,
     * NDB) ET le bloc SB (a_serv_demod[D_TOA], db_r). Sinon le SB lit du garbage
     * → l1s_sbdet_resp calcule "SB N bits in the future?!?" → sync rejeté →
     * BSIC=0, pas de sysinfo. Forcer a_serv_demod[D_TOA]=force_toa (=23) fait
     * `toa-=23 → 0` → passe le check `toa > bits_delta`. db_r page0=0xFFD00050
     * (off 0x50) / page1=0xFFD00078 (off 0x78), struct DSP33-36 a_serv_demod
     * @word8 → D_TOA = off 0x60 (p0) / 0x88 (p1). */
    if (size == 2) {
        static int force_toa = -2;  /* -2 = uninit, -1 = off */
        if (force_toa == -2) {
            const char *e = getenv("CALYPSO_FORCE_TOA");
            force_toa = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
            if (force_toa >= 0)
                fprintf(stderr, "[calypso-trx] CALYPSO_FORCE_TOA=%d (FB a_sync_demod + SB a_serv_demod[D_TOA] forcés)\n", force_toa);
        }
        if (force_toa >= 0) {
            switch (offset) {
            /* --- bloc FB (a_sync_demod, NDB @0x01F0) --- */
            case 0x01F0: val = 1;                       break; /* d_fb_det = FOUND */
            case 0x01F4: val = (uint16_t)force_toa;      break; /* a_sync_TOA */
            case 0x01F8: val = 0;                        break; /* a_sync_ANGLE = 0 */
            case 0x01FA: val = 0x7000;                   break; /* a_sync_SNR high */
            /* --- bloc SB (a_serv_demod[D_TOA], db_r page 0 et 1) --- */
            case 0x0060: case 0x0088:
                val = (uint16_t)force_toa;               break; /* SB TOA → 23 : passe le check "future" */
            default: break;                                     /* 0x01F2/0x01F6 + reste inchangés */
            }
        }
    }
    /* CALYPSO_FORCE_NB=1 (gate NB demod, 2026-06-02) : l1s_nb_resp bail "EMPTY"
     * si db_r->d_task_d==0 (le DSP NB demod ne tourne pas) → jamais de DATA_IND
     * BCCH → pas de SI. Force d_task_d≠0 (word 0 du db_r : page0 off 0x50 /
     * page1 off 0x78) pour passer "EMPTY" → le firmware émet le DATA_IND (que
     * CALYPSO_FORCE_AGCH remplit ensuite avec un SI/IMM-ASS). Révèle ensuite le
     * check d_burst_d (offset 0x52/0x7A). */
    if (size == 2 && (offset == 0x0050 || offset == 0x0078 ||
                      offset == 0x0052 || offset == 0x007A)) {
        static int force_nb = -1;
        if (force_nb < 0) {
            const char *e = getenv("CALYPSO_FORCE_NB");
            force_nb = (calypso_orch() && !(e && e[0] == '0')) ? 1 : 0;
        }
        if (force_nb) {
            if ((offset == 0x0050 || offset == 0x0078) && val == 0) {
                val = 1;   /* d_task_d → non-zéro : passe le "EMPTY" */
            } else if (offset == 0x0052 || offset == 0x007A) {
                /* d_burst_d ← db_w->d_burst_d (= le burst_id que le firmware a
                 * commandé via dsp_load_rx_task) → passe le check
                 * d_burst_d != burst_id. db_w d_burst_d : p0 DSP word 0x801,
                 * p1 0x815 (db_w p0=0xFFD00000 off0x02, p1=0xFFD00028 off0x2A). */
                /* d_burst_d : le firmware (l1s_nb_resp) attend 0,1,2,3 sur les
                 * 4 bursts consécutifs d'un bloc et copie a_cd->DATA_IND au
                 * burst 3. L'écho db_w (0x801/0x815) était décalé d'un cran
                 * (double-buffer : db_w porte la commande du burst SUIVANT) ->
                 * "BURST ID X!=Y" -> jamais de DATA_IND. On dérive donc d_burst_d
                 * du FN : un bloc = 4 frames consécutives (fn..fn+3) ; un saut de
                 * FN = nouveau bloc -> reset à 0. (Vérifié sur la sonde : blocs =
                 * 1281,1282,1283,1284 puis gros saut.) Donne exactement 0,1,2,3. */
                static uint32_t bd_last_fn = 0xFFFFFFFFu;
                static uint8_t  bd_cnt = 0;
                if (s->fn != bd_last_fn) {
                    uint8_t prev = bd_cnt;
                    bd_cnt = (s->fn == bd_last_fn + 1) ? (uint8_t)((bd_cnt + 1) & 3)
                                                       : 0;
                    bd_last_fn = s->fn;
                    /* SONDE PARTITION (2) : le firmware atteint-il le burst 3 ?
                     * Si oui (compteur croît régulièrement) -> DATA_IND émis ->
                     * mur aval (contenu/FN). Si non -> reset avant burst 3 ->
                     * mur timing. On logge la transition ->3 avec le FN. */
                    if (bd_cnt == 3 && prev != 3) {
                        static unsigned n3 = 0;
                        if (n3++ < 60 || (n3 % 200) == 0)
                            fprintf(stderr, "[nb3] reached burst3 #%u fn=%u fn%%51=%u "
                                    "off=0x%04x\n", n3, (unsigned)s->fn,
                                    (unsigned)(s->fn % 51), (unsigned)offset);
                    }
                }
                val = bd_cnt;
            }
        }
    }
    /* DSP boot handshake: firmware polls DL_STATUS until it reads BOOT.
     * FAKE INCONDITIONNEL. PROUVÉ (2026-06-23) : virer le fake en revival (SHUNT=0)
     * → le c54x ne tourne PLUS (0 ligne, reproductible) alors que c54x_reset pose
     * running=true. Mécanisme = le spin tight du firmware `dsp_bl_wait_ready`
     * (while readw(0x0ffe)!=1) affame le tick QEMU qui exécute c54x_run → la ROM
     * n'écrit jamais son IDLE → deadlock scheduling. Le fake court-circuite le spin
     * (le firmware sort tôt) → QEMU schedule le tick → la ROM tourne. Donc le fake est
     * load-bearing POUR FAIRE TOURNER la ROM, pas juste pour mentir au firmware. */
    if (offset == DSP_DL_STATUS_ADDR && !s->dsp_booted) {
        if (++s->boot_frame > 3) {
            s->dsp_ram[DSP_DL_STATUS_ADDR/2] = DSP_DL_STATUS_BOOT;
            s->dsp_ram[DSP_API_VER_ADDR/2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR/2] = 0;
            s->dsp_booted = true;
            TRX_LOG("DSP boot ver=0x%04x", DSP_API_VERSION);
            val = DSP_DL_STATUS_BOOT;
        }
    }
    /* ARM-read trace on d_fb_det / d_fb_mode / a_sync_demod cells:
     *   0x01F0 = d_fb_det        (DSP word 0x08F8)
     *   0x01F2 = d_fb_mode       (DSP word 0x08F9)
     *   0x01F4..0x01FA = a_sync_demod[0..3] (TOA/PM/ANGLE/SNR)
     * Capped + thinned. Goal: confirm whether ARM polls these cells and
     * what value it sees vs what DSP wrote. If ARM never reads while DSP
     * writes 0x095b → ARM-side mapping/timing bug. */
    if (offset >= 0x01F0 && offset <= 0x01FE && (offset & 1) == 0) {
        static unsigned arm_rd_log = 0;
        static unsigned arm_rd_mode = 0;
        arm_rd_log++;
        bool is_mode = (offset == 0x01F2);
        if (is_mode) arm_rd_mode++;
        /* d_fb_mode: log EVERY read (no cap) — race-window check.
         * Other cells: thinned. */
        bool log_it = is_mode ||
                      (arm_rd_log <= 200 || (arm_rd_log % 5000) == 0) ||
                      (val != 0 && offset == 0x01F0);
        if (log_it) {
            const char *name =
                (offset == 0x01F0) ? "d_fb_det"   :
                (offset == 0x01F2) ? "d_fb_mode"  :
                (offset == 0x01F4) ? "a_sync_TOA" :
                (offset == 0x01F6) ? "a_sync_PM"  :
                (offset == 0x01F8) ? "a_sync_ANG" :
                (offset == 0x01FA) ? "a_sync_SNR" : "unk";
            TRX_LOG("ARM RD %s [arm=0x%04x dsp_word=0x%04x] = 0x%04x sz=%d fn=%u #%u",
                    name, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                    (unsigned)val, size, s->fn, arm_rd_log);
        }
    }

    /* ARM-read trace on a_cd[0..14] : CCCH demod result buffer (15 words).
     *   DSP words 0x09D0..0x09DE → ARM bytes 0x03A0..0x03BD.
     * Goal : confirmer si ARM L1 prim_rx_nb consomme effectivement a_cd[]
     * quand task=24 (ALLC) fire et A_CD-WR remplit le buffer. Si compteur=0
     * mais A_CD-WR>0, le mur DATA_IND est avant la lecture (firmware ne
     * s'arme pas sur l'event CCCH). Si compteur>0 mais DATA_IND=0, le
     * mur est downstream (check db_r->d_burst_d ou autre dans
     * prim_rx_nb.c::l1s_nb_resp). */
    if (offset >= 0x03A0 && offset <= 0x03BD && (offset & 1) == 0) {
        static unsigned arm_rd_a_cd = 0;
        arm_rd_a_cd++;
        if (arm_rd_a_cd <= 200 || (arm_rd_a_cd % 1000) == 0) {
            unsigned word_idx = (unsigned)((offset - 0x03A0) / 2);
            TRX_LOG("ARM RD a_cd[%u] [arm=0x%04x dsp_word=0x%04x] = 0x%04x sz=%d fn=%u #%u",
                    word_idx, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                    (unsigned)val, size, s->fn, arm_rd_a_cd);
        }
    }
    return val;
}

/* === Sideband RACH (NO-HARDCODE) ============================================
 * Le firmware ecrit la VRAIE RACH dans d_rach (mot NDB 0x023A = byte 0x0474) :
 * value = (ra<<8) | (bsic<<2). On la publie au device (qemu_wrap ul_drain) via
 * un fichier REGULIER /dev/shm/calypso_rach (PAS un FIFO -> pas de blocage).
 * Layout fige (16 octets), partage avec qemu_wrap.c. Single-writer/single-reader,
 * pwrite atomique 16o, seq ecrite en dernier. Remplace le RA=3 hardcode du device. */
static void calypso_rach_publish(uint8_t ra, uint8_t bsic, uint32_t fn)
{
    static int fd = -2;
    if (fd == -2) {
        fd = open("/dev/shm/calypso_rach", O_CREAT | O_RDWR, 0644);
        if (fd >= 0 && ftruncate(fd, 16) < 0) { /* best-effort */ }
    }
    if (fd < 0) return;
    static uint32_t seq = 0;
    seq++;
    uint8_t buf[16] = {0};
    buf[4] = ra;
    buf[5] = bsic;
    memcpy(buf + 8, &fn, sizeof(fn));
    memcpy(buf + 0, &seq, sizeof(seq));   /* seq en premier mais ecrit atomiquement */
    if (pwrite(fd, buf, sizeof(buf), 0) < 0) { /* best-effort */ }
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;

    /* SONDE GAP-1 : l'ARM écrit-il data[0x0c36] (= ARM byte 0x86c), le pointeur
     * de tâche que le DSP CALA à 0xb3a5 et qui est nul → derail ? */
    if (offset == 0x086c && size == 2) {
        static unsigned atw = 0;
        if (atw < 40) { atw++;
            fprintf(stderr, "[trx] TASKPTR-WR-ARM off=0x086c val=0x%04x fn=%u "
                    "(-> data[0x0c36])\n", (unsigned)(value & 0xFFFF), s->fn);
        }
    }

    /* === HS-ARM-WR : boot-handshake write discriminator (reg_mode==c54x) ===
     * insn 10-11 proved one full command transits end-to-end; the suspect is that
     * the firmware never RE-ARMS command #2 after the DSP republishes IDLE. This
     * logs every ARM write into the bootloader mailbox region [0x0FF0,0x0FFF] (cmd
     * cell DSP 0x0fff = ARM 0x0ffe ; BL_ADDR/SIZE companions just below) so a grep
     * answers the cheap question: does the ARM emit a write of 2/4 after IDLE, yes
     * or no? No write → ARM logic bails (read-back/stale-IDLE = Plan-A mech 1).
     * Write present but DSP never reads it (cross-check WATCH-READ) → scheduling
     * (Plan-A mech 2). Only fires in real-ROM mode; harmless to the mock path. */
    if (dsp_real_rom_mode() && offset >= 0x0FF0 && offset <= 0x0FFF) {
        static unsigned hsw = 0;
        if (hsw < 300) {
            hsw++;
            /* fprintf, PAS TRX_LOG : TRX_LOG est gaté CALYPSO_DEBUG=TRX → muet
             * dans le run banc → le "HS-ARM-WR=0" était un ARTEFACT de logging,
             * pas une preuve d'absence d'écriture. On force l'affichage. */
            fprintf(stderr, "[trx] HS-ARM-WR #%u off=0x%04x val=0x%04x fn=%u\n",
                    hsw, (unsigned)offset, (unsigned)(value & 0xFFFF), s->fn);
        }
    }

    /* === HS-ARM-WR-WIDE : sonde write ÉLARGIE (trancher §2.6) ===
     * Toute écriture ARM NON-NULLE dans l'API DSP (offset<0x1000). On filtre
     * value!=0 pour sauter le dsp_api_memset (8192 zéros @82bcd4) et ne capter
     * que NDB (d_dsp_state=3, etc.) + params + cmd 2. La DERNIÈRE écriture loggée
     * = jusqu'où le firmware progresse avant de hang. fprintf, capé, gaté revival. */
    if (dsp_real_rom_mode() && offset < 0x1000 && (value & 0xFFFF) != 0) {
        static unsigned wide = 0;
        if (wide < 220) {
            wide++;
            fprintf(stderr, "[trx] WR-WIDE #%u off=0x%04x val=0x%04x fn=%u\n",
                    wide, (unsigned)offset, (unsigned)(value & 0xFFFF), s->fn);
        }
    }

    /* === HS-ARM-GATE : read-only capture of the foreground go-live gating cells ===
     * The wait-loop at PROM 0xa4d4 exits only when data[0x3f70] bit1 is set by
     * one of the FOREGROUND setters (0xde9c/0xa5bd/0xb3ef/0x710c), which are
     * gated by CMPM/BC tests on the CONTROL cells data[0x098a] / data[0x098c].
     * Those DSP words map to ARM API offsets 0x0314 / 0x0318 (dsp_word =
     * offset/2 + 0x0800). This PINS whether the ARM osmocom/TI L1 DSP bring-up
     * ever writes those cells. READ-ONLY instrumentation: no DSP/ARM state is
     * poked here — the existing mirror below performs the genuine transport. */
    if (dsp_real_rom_mode() && (offset == 0x0314 || offset == 0x0318)) {
        static unsigned gate = 0;
        if (gate < 256) {
            gate++;
            fprintf(stderr,
                "[trx] HS-ARM-GATE #%u off=0x%04x dsp_word=0x%04x val=0x%04x fn=%u\n",
                gate, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                (unsigned)(value & 0xFFFF), s->fn);
        }
    }

    calypso_arm2dsp_on_arm_write((uint16_t)offset, (uint16_t)(value & 0xFFFF));

    /* FORCE-HS (etape A, gated CALYPSO_FORCE_HS=<hexval>) : override the ARM go-live
     * control cells 0x0314/0x0318 with a non-zero enable value, to test whether the
     * REAL ROM setter (0xde9c / 0xa5bd) then fires (F70-SETBIT1) and the DSP reaches
     * go-live. Runtime-configurable value so we can bracket without rebuild. */
    if (dsp_real_rom_mode() && (offset == 0x0314 || offset == 0x0318)) {
        static int fh = -1; static uint32_t fhv = 0;
        if (fh < 0) { const char *e = getenv("CALYPSO_FORCE_HS");
            fhv = (e && *e) ? (uint32_t)strtoul(e, NULL, 0) : 0; fh = fhv ? 1 : 0; }
        if (fh) {
            value = fhv;
            static unsigned fhc = 0;
            if (fhc++ < 12)
                fprintf(stderr, "[trx] FORCE-HS off=0x%04x -> val=0x%04x\n",
                        (unsigned)offset, (unsigned)(fhv & 0xFFFF));
        }
    }

    /* === Unconditional probe : count ALL writes by offset range ===
     * Gated par CALYPSO_DEBUG=DSP_WRITE_COUNT. Bucket par 0x40-byte zone
     * pour voir si ARM hit les bonnes zones (page 0 task 0x00-0x1F, page 1
     * task 0x28-0x47, NDB 0x1A8+). Si compteurs = 0 dans la PM zone alors
     * que pm_resp fire → write path ne passe PAS par ce hook. */
    if (calypso_debug_enabled("DSP_WRITE_COUNT")) {
        static uint64_t c_p0 = 0, c_p1 = 0, c_ndb = 0, c_other = 0;
        if (offset < 0x0028)      c_p0++;
        else if (offset < 0x0050) c_p1++;
        else if (offset >= 0x01A8 && offset < 0x0800) c_ndb++;
        else c_other++;
        uint64_t tot = c_p0 + c_p1 + c_ndb + c_other;
        if (tot <= 30 || (tot % 1000) == 0) {
            fprintf(stderr,
                "[calypso-trx] DSP_WRITE_COUNT p0=%llu p1=%llu ndb=%llu other=%llu "
                "(last off=0x%04x val=0x%llx sz=%u fn=%u)\n",
                (unsigned long long)c_p0, (unsigned long long)c_p1,
                (unsigned long long)c_ndb, (unsigned long long)c_other,
                (unsigned)offset, (unsigned long long)value, size, s->fn);
        }
    }

    if (size == 2) s->dsp_ram[offset/2] = value;
    else if (size == 4) { s->dsp_ram[offset/2] = value; s->dsp_ram[offset/2+1] = value >> 16; }
    else ((uint8_t *)s->dsp_ram)[offset] = value;

    /* Mirror to DSP s->data[] so prog_fetch in OVLY mode sees ARM writes
     * to the shared API/DARAM region. On real silicon dsp_ram and the DSP
     * DARAM share one physical memory; without this mirror, ARM writes
     * land in dsp_ram only and the DSP executes the stale (boot-time
     * MVPD-copied) value via prog_fetch. */
    if (s->dsp) {
        uint16_t dsp_word = offset/2 + 0x0800;
        calypso_pcb_daram_lock_acquire();
        if (size == 2) {
            s->dsp->data[dsp_word] = (uint16_t)value;
        } else if (size == 4) {
            s->dsp->data[dsp_word]     = (uint16_t)value;
            s->dsp->data[dsp_word + 1] = (uint16_t)(value >> 16);
        }
        calypso_pcb_daram_lock_release();
        /* size==1 byte: skip — sub-word writes to DSP data are unusual
         * and would need careful endianness handling; falls back to the
         * dsp_ram-only path which is fine for the sub-word case. */
    }

    /* === CO-RUN-on-WRITE (fix scheduling, symétrique du co-run lecture) — 2026-06-23 ===
     * Le firmware écrit cmd 2 (COPY_BLOCK) dans BL_CMD_STATUS (ARM 0x0ffe = DSP word
     * 0x0fff) APRÈS avoir posé BL_ADDR=0x7000 (0x0ffc) et BL_SIZE=0 (0x0ffa). Mais
     * il ne RELIT plus 0x0ffe ensuite → le co-run-lecture ne fire pas → le c54x ne
     * tourne plus assez pour repoller → ne consomme JAMAIS cmd 2 (PC 0xb429 vu 1 seule
     * fois, le passage garbage du boot). On force la ROM à avancer ICI, juste après le
     * write de la commande : elle repolle 0xb424 (cmpm #2 → match), lit les VRAIS
     * addr=0x7000/len=0 → bc bneq faux (len=0) → bacc 0x7000 → SORT du bootloader → L1.
     * Gaté revival (SHUNT=0), seulement sur écriture d'une commande (2=COPY_BLOCK,
     * 4=COPY_MODE), data[0x0fff] déjà à jour ci-dessus. */
    if (offset == 0x0FFE && dsp_real_rom_mode() && s->dsp && s->dsp->running
        && ((value & 0xFFFF) == 2 || (value & 0xFFFF) == 4)) {
        static unsigned cmw = 0;
        if (cmw < 40) { cmw++;
            fprintf(stderr, "[trx] CMD-CORUN #%u cmd=0x%04x addr(0x0ffe)=0x%04x "
                    "len(0x0ffd)=0x%04x fn=%u — running ROM to consume\n",
                    cmw, (unsigned)(value & 0xFFFF),
                    (unsigned)s->dsp->data[0x0ffe], (unsigned)s->dsp->data[0x0ffd], s->fn);
        }
        c54x_run(s->dsp, 4096);
    }

    /* Debug: log task-related writes to write pages (d_task_d/u/md/ra) */
    if ((offset == 0x0000 || offset == 0x0004 || offset == 0x0008 ||
         offset == 0x000E || offset == 0x0028 || offset == 0x002C ||
         offset == 0x0030 || offset == 0x0036) && value != 0) {
        static int wp_log = 0;
        if (++wp_log <= 100)
            TRX_LOG("DSP WR [0x%04x] = 0x%04x (sz=%d) fn=%u",
                    (unsigned)offset, (unsigned)value, size, s->fn);
    }

    /* === d_task_md probe — fires SANS filter value=0 ===
     * Si d_task_md write = 0 (= memset only), pm_cmd jamais appelé.
     * Si d_task_md write = 1 (= pm_cmd writes), notre probe ARM TASK WR
     * devrait fire — mais on voit count=0 → contradiction à investiguer.
     * Gated par CALYPSO_DEBUG=D_TASK_MD_ALL. */
    if ((offset == 0x0008 || offset == 0x0030) && size == 2) {
        /* Latch d_task_md pour calypso_layer1.c (CALYPSO_L1=c) : le poll tick-time
         * rate le transient task=5, on le capture ici au write-time. */
        if (calypso_l1_c_active()) {
            calypso_layer1_on_task_write((uint16_t)value);
        }
        if (calypso_debug_enabled("D_TASK_MD_ALL")) {
            static unsigned dtm_log = 0;
            if (dtm_log < 30 || (dtm_log % 100) == 0) {
                fprintf(stderr,
                    "[calypso-trx] D_TASK_MD_ALL #%u off=0x%04x val=0x%04x fn=%u\n",
                    dtm_log, (unsigned)offset, (unsigned)value, s->fn);
                dtm_log++;
            }
        }
    }

    /* === Hypothesis #1 probe : d_dsp_page WR (NDB+0 = ARM 0x01A8) ===
     * Écrit par dsp_end_scenario(): `ndb->d_dsp_page = B_GSM_TASK | w_page`.
     * Si jamais hit → dsp_end_scenario jamais fired → w_page stuck à 0.
     * Gated par CALYPSO_DEBUG=D_DSP_PAGE. */
    if (offset == 0x01A8 && size == 2) {
        if (calypso_debug_enabled("D_DSP_PAGE")) {
            static unsigned ddp_log = 0;
            if (ddp_log < 50) {
                fprintf(stderr,
                    "[calypso-trx] D_DSP_PAGE WR #%u val=0x%04x (B_GSM_TASK=%d w_page=%d) fn=%u\n",
                    ddp_log, (unsigned)value,
                    !!(value & 2), !!(value & 1),  /* B_GSM_TASK=(1<<1)=0x02, w_page=bit 0 */
                    s->fn);
                ddp_log++;
            }
        }
    }

    /* === Hypothesis #2 probe : ARM WR per-page split (= cur_bucket advance) ===
     * Si bucket n'avance pas, tous les ARM TASK WR continuent à page 0.
     * Compteur séparé page 0 vs page 1 sur task_d/task_md à chaque frame. */
    if (calypso_debug_enabled("PAGE_SPLIT")) {
        bool is_p0 = (offset == 0x0000 || offset == 0x0008 || offset == 0x000E ||
                      offset == 0x000A);
        bool is_p1 = (offset == 0x0028 || offset == 0x0030 || offset == 0x0036 ||
                      offset == 0x0032);
        if ((is_p0 || is_p1) && value != 0 && size == 2) {
            static unsigned p0_count = 0, p1_count = 0;
            if (is_p0) p0_count++; else p1_count++;
            if ((p0_count + p1_count) <= 30 || ((p0_count + p1_count) % 50) == 0) {
                fprintf(stderr,
                    "[calypso-trx] PAGE_SPLIT p0=%u p1=%u (last off=0x%04x val=%u fn=%u)\n",
                    p0_count, p1_count, (unsigned)offset, (unsigned)value, s->fn);
            }
        }
    }

    /* AFC hook : firmware afc_load_dsp() écrit dsp_api.db_w->d_afc.
     * Word 15 du WP : page0 = byte 0x001E, page1 = byte 0x0046.
     * Propage le DAC value vers TWL3025 → rotation samples BSP.
     * Chaîne complete : firmware → ce hook → twl3025 → BSP rotation. */
    if ((offset == 0x001E || offset == 0x0046) && size == 2) {
        int16_t dac_value = (int16_t)(uint16_t)value;
        calypso_twl3025_set_afc_dac(dac_value);
        static int afc_log = 0;
        if (++afc_log <= 50)
            TRX_LOG("AFC WR page=%d dac=%d hz=%.1f fn=%u",
                    (offset == 0x001E) ? 0 : 1, dac_value,
                    calypso_twl3025_get_afc_hz(), s->fn);
    }

    /* d_rach offset finder — circular buffer of recent NDB writes.
     * NDB starts at byte offset 0x01A8 in API RAM (= dsp_ram + 0x01A8).
     * We capture every non-zero ARM-side write to NDB range and dump the
     * last 16 entries when d_task_ra commits (0x000E page0 or 0x0036 page1).
     * The d_rach value matches the pattern (ra<<8) | (bsic<<2) — the ra
     * byte mirrors what the mobile L3 just announced in `RANDOM ACCESS`.
     * Once observed, set CALYPSO_NDB_D_RACH_OFFSET to the matching word
     * index (= (offset - 0x01A8) / 2 + 0xD4 in the convention used by
     * calypso_bsp.c). */
    {
        #define D_RACH_RING_SIZE 128
        struct ndb_wr_entry { hwaddr off; uint32_t val; uint32_t fn; uint32_t insn; uint8_t sz; };
        static struct ndb_wr_entry ring[D_RACH_RING_SIZE];
        static int idx;
        static int dump_count;

        /* Capture all sizes (1/2/4) over the full NDB + post-NDB region
         * (NDB extent varies by DSP firmware version; widen to 0x0800 to
         * be safe, restrict later once the actual d_rach offset is pinned).
         * Filter only zero-value writes to keep the ring useful. */
        if (offset >= 0x01A8 && offset < 0x0800 && value != 0 &&
            (size == 1 || size == 2 || size == 4)) {
            ring[idx % D_RACH_RING_SIZE] = (struct ndb_wr_entry){
                offset, (uint32_t)value, s->fn, s->dsp ? s->dsp->insn_count : 0,
                (uint8_t)size
            };
            idx++;
        }

        bool task_ra_commit =
            (offset == DSP_API_W_PAGE0 + DB_W_D_TASK_RA * 2 ||
             offset == DSP_API_W_PAGE1 + DB_W_D_TASK_RA * 2) && value != 0;
        if (task_ra_commit && dump_count < 30) {
            dump_count++;
            uint32_t commit_insn = s->dsp ? s->dsp->insn_count : 0;
            TRX_LOG("D_RACH-FINDER task_ra commit @0x%04x = 0x%04x fn=%u insn=%u — full ring (last 128 NDB writes):",
                    (unsigned)offset, (unsigned)value, s->fn, commit_insn);
            int n = (idx < D_RACH_RING_SIZE) ? idx : D_RACH_RING_SIZE;
            int start = idx - n;
            for (int i = 0; i < n; i++) {
                int k = (start + i) % D_RACH_RING_SIZE;
                uint32_t v   = ring[k].val;
                int32_t d_insn = (int32_t)(commit_insn - ring[k].insn);
                uint8_t  ra  = (uint8_t)((v >> 8) & 0xFF);
                uint8_t  low = (uint8_t)(v & 0xFF);
                uint8_t  bsic = low >> 2;
                /* Mark entries within the "RACH window" (last 1000 insn
                 * before commit) — those are the candidates worth scanning
                 * by eye for ra match against mobile L3 log. Older entries
                 * are init/unrelated but kept in the dump for offline
                 * correlation when filtering misses the d_rach write. */
                const char *tag = (d_insn >= 0 && d_insn <= 1000) ? "*HOT*" : "";
                fprintf(stderr,
                        "[trx] D_RACH-FINDER  #%d off=0x%04x val=0x%04x sz=%u "
                        "d_insn=%+d ra=0x%02x bsic=0x%02x fn=%u %s\n",
                        i, (unsigned)ring[k].off, v, ring[k].sz,
                        -d_insn, ra, bsic, ring[k].fn, tag);
            }
        }
    }

    /* NO-HARDCODE : publie la VRAIE RA+FN au mot d_rach (byte = word*2). Tire a
     * CHAQUE ecriture d_rach par le firmware -> fiable, independant de la voie
     * d_task_ra/page (qui rate cote shunt LATCH). value = (ra<<8)|(bsic<<2). */
    {
        static uint32_t dr_byte = 0;
        if (!dr_byte) {
            const char *e = getenv("CALYPSO_NDB_D_RACH_OFFSET");
            uint32_t w = (e && *e) ? (uint32_t)strtoul(e, NULL, 0) : 0x023A;
            dr_byte = w * 2;   /* 0x023A word -> 0x0474 ARM byte */
        }
        if (offset == dr_byte && value != 0 && (size == 2 || size == 4)) {
            uint8_t ra = (uint8_t)((value >> 8) & 0xFF);
            calypso_rach_publish(ra, (uint8_t)((value & 0xFF) >> 2), s->fn);
            calypso_dsp_shunt_record_rach(ra);   /* SONDE B : l1s.current_time.fn par RA */
        }
    }

    /* DSP bootloader mailbox writes (osmocom-bb dsp.c BL_*).
     * ARM byte → DSP word mapping (api_ram[w] ↔ ARM byte w*2):
     *   ARM 0x0FF8 BL_ADDR_HI    ↔ DSP word 0x0FFC
     *   ARM 0x0FFA BL_SIZE       ↔ DSP word 0x0FFD
     *   ARM 0x0FFC BL_ADDR_LO    ↔ DSP word 0x0FFE  (BACC target)
     *   ARM 0x0FFE BL_CMD_STATUS ↔ DSP word 0x0FFF  (poll value)
     * Trace every write so we can confirm the handshake actually reaches
     * the cells the bootloader at PROM0 0xb41c-0xb430 reads. */
    if (offset == 0x0FF8 || offset == 0x0FFA ||
        offset == 0x0FFC || offset == 0x0FFE) {
        const char *name = (offset == 0x0FF8) ? "BL_ADDR_HI"   :
                           (offset == 0x0FFA) ? "BL_SIZE"      :
                           (offset == 0x0FFC) ? "BL_ADDR_LO"   :
                                                "BL_CMD_STATUS";
        static unsigned bl_log;
        if (++bl_log <= 200)
            TRX_LOG("BL ARM WR %s [arm=0x%04x dsp_word=0x%04x] = 0x%04x sz=%d fn=%u",
                    name, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                    (unsigned)value, size, s->fn);
    }

    /* Log task writes for debugging — no interception, no faking.
     * The DSP handles all tasks via shared API RAM. */
    {
        hwaddr w0_md = DSP_API_W_PAGE0 + DB_W_D_TASK_MD * 2;
        hwaddr w1_md = DSP_API_W_PAGE1 + DB_W_D_TASK_MD * 2;
        hwaddr w0_d  = DSP_API_W_PAGE0 + DB_W_D_TASK_D * 2;
        hwaddr w1_d  = DSP_API_W_PAGE1 + DB_W_D_TASK_D * 2;
        if ((offset == w0_md || offset == w1_md ||
             offset == w0_d  || offset == w1_d) && value != 0) {
            static unsigned task_log = 0;
            /* Always log non-PM tasks (value != 1) so FB_TASK=5 / SB=6
             * surfaces no matter when it occurs. PM=1 thinned. */
            bool is_pm = (value == 1);
            if (!is_pm || task_log < 100 || (task_log % 500) == 0)
                TRX_LOG("ARM TASK WR [0x%04x] = %u fn=%u",
                        (unsigned)offset, (unsigned)value, s->fn);
            task_log++;

            /* Test H1 : mémorise insn DSP + EA data DSP quand l'ARM commande
             * FB (d_task_md=5), pour que la sonde D_TASK_MD-RD timestampe les
             * reads DSP par rapport à ce write et compare les EA. */
            if (value == 5 && s->dsp) {
                g_arm_taskmd5_insn = s->dsp->insn_count;
                g_arm_taskmd5_ea   = (uint16_t)(offset/2 + 0x0800);
            }

            /* === TASK6-IRQ snapshot (2026-05-28) ===
             * À chaque ARM TASK WR = 6 (SB demanded), snapshot IMR + IFR du
             * DSP. Bit 5 = BRINT0 (BSP RX DMA-complete). Discrimine deux
             * causes pour "SB jamais locké" :
             *   IMR_bit5 = 0 + IFR_bit5 = 0 → bit 5 jamais armé par firmware
             *     (= bug STM-vers-MMR upstream, ou firmware skip arm)
             *   IMR_bit5 = 1 + IFR_bit5 = 0 → bit 5 armé mais aucune source
             *     d'IT ne le set → émulateur McBSP DMA-complete pas modélisé
             *   IMR_bit5 = 1 + IFR_bit5 = 1 → bit 5 armé + pending, mais
             *     ISR ne dispatch pas vers PROM3 → bug dispatcher (item 5)
             *   IMR_bit5 = 0 + IFR_bit5 = 1 → impossible normalement (IFR
             *     set sans IMR = source assert sans arm — bug émulateur) */
            if (value == 6 && s->dsp) {
                static unsigned t6_log;
                if (t6_log < 50) {
                    uint16_t imr = s->dsp->imr;
                    uint16_t ifr = s->dsp->ifr;
                    TRX_LOG("TASK6-IRQ #%u fn=%u IMR=0x%04x (bit5=%d) "
                            "IFR=0x%04x (bit5=%d) insn=%u",
                            t6_log, s->fn, imr, !!(imr & (1<<5)),
                            ifr, !!(ifr & (1<<5)),
                            s->dsp->insn_count);
                    t6_log++;
                }
            }

            /* FBSB orchestration hook: ARM has just written d_task_md.
             * Initialise on first call, then log task changes (no host-
             * side synthesis remaining as of 2026-05-28 cleanup). */
            if (!g_fbsb_inited) {
                uint16_t *ndb_target = (s->dsp && s->dsp->data)
                                       ? &s->dsp->data[0x0800]
                                       : s->dsp_ram;
                calypso_fbsb_init(&g_fbsb, ndb_target, 0x0800);
                g_fbsb_inited = true;
                TRX_LOG("fbsb init ok ndb_base=0x0800 target=%s",
                        (s->dsp && s->dsp->data) ? "dsp->data" : "dsp_ram (fallback)");
            }
            if (g_fbsb_inited) {
                TRX_LOG("fbsb hook fired task=%u fn=%u",
                        (unsigned)value, s->fn);
                calypso_fbsb_on_dsp_task_change(&g_fbsb,
                                                (uint16_t)value,
                                                (uint64_t)s->fn);
            }

        }
    }
    /* DSP page */
    if (offset == DSP_API_NDB) s->dsp_page = value & 1;
    /* DSP status */
    if (offset == DSP_DL_STATUS_ADDR) {
        if (value == 0) { s->dsp_booted = false; s->boot_frame = 0; TRX_LOG("DSP reset"); }
        else if (value == DSP_DL_STATUS_READY) {
            s->dsp_ram[DSP_API_VER_ADDR/2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR/2] = 0;
            /* Unmask API IRQ (IRQ15) in INTH */
            {
                uint16_t mask;
                cpu_physical_memory_read(0xFFFFFA08, &mask, 2);
                mask &= ~(1 << 15);
                cpu_physical_memory_write(0xFFFFFA08, &mask, 2);
                TRX_LOG("DSP ready — unmasked API IRQ (mask=0x%04x)", mask);
            }
            /* Reset C54x DSP — boot runs in TDMA ticks (parallel with ARM).
             * Skip if dsp-blob fixture is active: another reset would
             * re-run the PROM→DARAM auto-copy and overwrite the loaded
             * blob plus the PC override. */
            if (s->dsp && !s->dsp->blob_loaded && !dsp_real_rom_mode()) {
                c54x_reset(s->dsp);
                s->dsp->running = true;
                s->dsp_init_done = false;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("C54x DSP reset — boot via TDMA ticks");
            } else if (s->dsp && dsp_real_rom_mode()) {
                /* === FIX collision COPY_BLOCK(2) ≡ DL_STATUS_READY(2) — 2026-06-23 ===
                 * En revival, la valeur 2 écrite ici N'EST PAS « le DSP dit READY » :
                 * c'est la commande bootloader BL_CMD_STATUS=2 (COPY_BLOCK) du firmware
                 * (dsp_bl_start_at). La vraie ROM est en plein bootloader ; le co-run
                 * (calypso_dsp_write/read) vient de lui faire consommer cmd 2 → bacc
                 * 0x7000 → L1. Un c54x_reset ICI EFFACERAIT ce bacc (observé :
                 * HIGHVEC-ENTRY #2 prev_PC=0xb424 juste après que le c54x lit api_ram=2).
                 * Donc en revival : NE PAS reset — laisser la ROM bacc dans L1. */
                static unsigned norst = 0;
                if (norst < 10) { norst++;
                    fprintf(stderr, "[trx] DL_STATUS=2 revival = bootloader COPY_BLOCK cmd, "
                            "NOT resetting c54x (let it bacc) fn=%u\n", s->fn);
                }
            } else if (s->dsp) {
                TRX_LOG("DSP_DL_STATUS_READY received but dsp-blob mode "
                        "active — skipping reset (PC=0x%04x preserved)",
                        s->dsp->pc);
            }
        }
    }
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read, .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size=1,.max_access_size=4}, .impl = {.min_access_size=1,.max_access_size=4},
};

/* ---- TPU ---- */
static void calypso_dsp_done(void *opaque) {
    CalypsoTRX *s = opaque;
    s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_EN;
    /* SONDE DSP-DONE (2026-06-24 diag livraison ordre, RO) : entree + gate. */
    {
        static unsigned dde = 0;
        if (dde++ < 60)
            fprintf(stderr, "[trx] DSP-DONE-ENTRY #%u d_dsp_page=0x%04x "
                    "dsp=%d shunt_active=%d fn=%u\n",
                    dde, (unsigned)s->dsp_ram[0x01A8/2], !!s->dsp,
                    calypso_dsp_shunt_active(), s->fn);
    }

    /* Hardware DMA: copy API write page → DSP DARAM 0x0586.
     * Triggered by firmware writing TPU_CTRL with EN bit (dsp_end_scenario).
     * This is the ONLY place DMA happens — same as real Calypso.
     *
     * GATED par CALYPSO_DSP_SHUNT : si le shunt est actif, on skip
     * complètement cette DMA — le mock écrit les résultats directement
     * dans NDB/read-page et le c54x est inactif (pas de consommateur). */
    if (s->dsp && s->dsp_ram[0x01A8/2] != 0 && !calypso_dsp_shunt_active()) {
        uint16_t page = s->dsp_ram[0x01A8/2] & 1;
        uint16_t *wp = page ?
            &s->dsp_ram[DSP_API_W_PAGE1/2] : &s->dsp_ram[DSP_API_W_PAGE0/2];

        /* Log proof that ARM wrote tasks before DMA */
        uint16_t task_d  = wp[DB_W_D_TASK_D];
        uint16_t task_u  = wp[DB_W_D_TASK_U];
        uint16_t task_md = wp[DB_W_D_TASK_MD];
        { static unsigned ddc = 0; if (ddc++ < 60)
            fprintf(stderr, "[trx] DSP-DONE-DMA #%u page=%u d_dsp_page=0x%04x "
                    "task_d=%u task_u=%u task_md=%u -> DARAM 0x0586 fn=%u\n",
                    ddc, page, (unsigned)s->dsp_ram[0x01A8/2],
                    task_d, task_u, task_md, s->fn); }
        if (task_d || task_u || task_md) {
            static int dma_task_log = 0;
            if (++dma_task_log <= 50)
                TRX_LOG("DMA proof: ARM wrote task_d=%u task_u=%u task_md=%u page=%u fn=%u",
                        task_d, task_u, task_md, page, s->fn);
        }

        /* Ordre canonique daram < api_ram. Section critique unique pour
         * la mirror DMA write page → DSP DARAM. */
        calypso_pcb_daram_lock_acquire();
        qemu_mutex_lock(&calypso_pcb_api_ram_lock);
        s->dsp->data[0x0584] = s->dsp_ram[0x01A8/2];
        s->dsp->data[0x0585] = s->fn & 0xFFFF;
        for (int i = 0; i < 20; i++)
            s->dsp->data[0x0586 + i] = wp[i];
        if (s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];
        qemu_mutex_unlock(&calypso_pcb_api_ram_lock);
        calypso_pcb_daram_lock_release();
    }

    /* Execute TPU RAM micro-instructions (TSP bus commands).
     * The firmware wrote a TPU scenario into TPU RAM. We scan it for
     * MOVE instructions that write to TSP registers. When we see
     * TSP_CTRL2 with WR bit, we send the TX byte to IOTA.
     *
     * IMPORTANT: The Calypso Rhea bus is 16-bit wide, mapped to the
     * 32-bit ARM bus at 2-byte stride. The firmware writes 16-bit TPU
     * instructions at ARM offsets 0, 2, 4, ..., which end up in
     * tpu_ram[0], tpu_ram[1], tpu_ram[2], ... However, the actual
     * physical layout has zero-padding between instructions (ARM 32-bit
     * alignment). We must skip zero words that are just bus padding,
     * not real SLEEP instructions. A real SLEEP (0x0000) always comes
     * after at least one non-zero instruction. */
    {
        uint8_t tsp_tx1 = 0;
        uint8_t tsp_ctrl1 = 0;
        bool seen_any = false;
        for (int i = 0; i < CALYPSO_TPU_RAM_SIZE / 2; i++) {
            uint16_t insn = s->tpu_ram[i];
            if (insn == 0x0000) {
                /* Skip zero words: they are either Rhea bus padding
                 * or the final SLEEP. Only break on SLEEP after we've
                 * seen real instructions, and only if the NEXT word
                 * is also zero (two consecutive zeros = real SLEEP). */
                if (seen_any) {
                    int next = i + 1;
                    if (next >= CALYPSO_TPU_RAM_SIZE / 2 ||
                        s->tpu_ram[next] == 0x0000)
                        break;  /* real SLEEP — end of scenario */
                }
                continue;
            }
            seen_any = true;
            uint8_t opcode = (insn >> 13) & 0x7;
            if (opcode == 4) {
                /* MOVE: addr = bits 4:0, data = bits 12:5 */
                uint8_t addr = insn & 0x1F;
                uint8_t data = (insn >> 5) & 0xFF;
                if (addr == 0x04) tsp_tx1 = data;     /* TPUI_TX_1 */
                if (addr == 0x00) tsp_ctrl1 = data;    /* TPUI_TSP_CTRL1 */
                if (addr == 0x01 && (data & 0x02)) {   /* TPUI_TSP_CTRL2 WR bit */
                    /* TSP write: send tsp_tx1 to the device.
                     * Device 0 (TWL3025): 7-bit data = tsp_tx1.
                     * This byte contains BDLON/BDLENA/BULENA bits. */
                    uint8_t dev = (tsp_ctrl1 >> 5) & 0x07;
                    if (dev == 0) {
                        calypso_iota_tsp_write(tsp_tx1, 0);
                    }
                }
            }
        }
    }

    qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
}
static void calypso_tdma_start(CalypsoTRX *s);

/* === CLK-master pthread =================================================
 *
 * Sends a 4-byte FN counter UDP packet to calypso-ipc-device every
 * 4.615 ms wall-clock. Uses clock_nanosleep(CLOCK_MONOTONIC, ABSTIME)
 * for sub-µs precision — bypasses the QEMU mainloop ±20ms jitter that
 * the previous in-tick send had.
 *
 * The CLK packet drives the qfn-paced UL in calypso-ipc-device
 * (qemu_wrap.c), which then advances osmo-trx-ipc's TX timeline and
 * generates CLK_IND to BTS. Précision wall ici = précision drift TRX↔BTS.
 *
 * The pthread maintains its own g_wall_fn counter. tdma_tick reads it
 * (via __atomic_load) so the DSP/BSP work uses wall-aligned FN values.
 */

#include <time.h>
#include <pthread.h>

static volatile uint32_t g_wall_fn = 0;
static volatile bool     g_clk_master_running = false;
static pthread_t         g_clk_master_thread;
static int               g_clk_master_fd = -1;
static struct sockaddr_in g_clk_master_peer;

/* GSM TDMA frame = 1250 samples / 270833.33 sps = 60/13 ms = 4 615 384,6 ns.
 * Fix 2026-05-30 : était 4615000 (arrondi 384 ns TROP RAPIDE/frame). Ce drain
 * QEMU plus rapide que le fill device (PERIOD_NS×2=4615384, calypso-ipc-device
 * qemu_wrap.c) vidait lentement la FIFO DL (profondeur ~2) → underrun ~+30s →
 * "FIFO empty" → IPC LOST → I/Q figées. Match exact = plus de drift structurel. */
/* Match EXACT le device osmo-trx (qemu_wrap.c PERIOD_NS=2307692 ×2 = 4615384)
 * pour biais ZÉRO sur la FIFO DL. NB : osmocom-bb trxcon (sched_trx.c) utilise
 * l'arrondi 4615000 côté host, mais le feed I/Q est cadencé par osmo-trx = la
 * radio = 4615384 sample-exact. C'est CETTE valeur qu'il faut matcher. */
#define WALL_TDMA_NS  4615384LL   /* = device osmo-trx (1250 smpl / 270833,33 sps) */

static void *clk_master_loop(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    /* Période TDMA configurable : si l'émulation c54x ne tient pas le 4.615ms
     * wall réel (→ osmocon LOST), ralentir UNIFORMÉMENT toute la timeline via
     * CALYPSO_TDMA_NS (le device heartbeat lit la MÊME var → osmo-trx/BTS
     * suivent → cohérent à vitesse réduite). Défaut = sample-exact réel. */
    long long wall_ns = WALL_TDMA_NS;
    const char *e = getenv("CALYPSO_TDMA_NS");
    if (e && *e) { long long v = atoll(e); if (v >= WALL_TDMA_NS) wall_ns = v; }

    fprintf(stderr,
            "[clk-master] pthread armed (CLOCK_MONOTONIC ABSTIME, %lld ns/frame%s)\n",
            wall_ns, (wall_ns != WALL_TDMA_NS) ? " [SLOWED via CALYPSO_TDMA_NS]" : "");

    while (g_clk_master_running) {
        next.tv_nsec += wall_ns;
        while (next.tv_nsec >= 1000000000LL) {
            next.tv_nsec -= 1000000000LL;
            next.tv_sec  += 1;
        }
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (rc != 0 && rc != EINTR) {
            /* Unrecoverable — log once and bail. */
            static int err_logged = 0;
            if (!err_logged++) {
                fprintf(stderr, "[clk-master] clock_nanosleep rc=%d, exiting\n", rc);
            }
            break;
        }

        uint32_t fn = __atomic_add_fetch(&g_wall_fn, 1, __ATOMIC_RELEASE)
                    % GSM_HYPERFRAME;
        if (g_clk_master_fd >= 0) {
            uint8_t pkt[4];
            pkt[0] = (fn >> 24) & 0xFF;
            pkt[1] = (fn >> 16) & 0xFF;
            pkt[2] = (fn >>  8) & 0xFF;
            pkt[3] =  fn        & 0xFF;
            (void)sendto(g_clk_master_fd, pkt, 4, 0,
                         (struct sockaddr *)&g_clk_master_peer,
                         sizeof(g_clk_master_peer));
        }
    }
    return NULL;
}

static void calypso_trx_start_clk_master_thread(CalypsoTRX *s)
{
    if (g_clk_master_running) return;
    g_clk_master_fd   = s->clk_fd;
    g_clk_master_peer = s->clk_peer;
    g_clk_master_running = true;
    pthread_create(&g_clk_master_thread, NULL, clk_master_loop, NULL);
    pthread_setname_np(g_clk_master_thread, "cal-clk-master");
    TRX_LOG("CLK-master pthread started (4.615ms wall, jitter-free)");
}

/* Called by calypso_tint0.c on each TDMA frame tick.
 * Forward declaration — actual tdma_tick is defined below. */
static void calypso_tdma_tick(void *opaque);
/* Prototype visible to tint0 (declared extern there) */
void calypso_tint0_do_tick(uint32_t fn);
void calypso_tint0_do_tick(uint32_t fn)
{
    if (!g_trx) return;
    g_trx->fn = fn;
    /* d_dsp_page is toggled by the DSP firmware itself (PC=0x1748),
     * NOT by ARM or the emulator. Don't touch it here. */
    calypso_tdma_tick(g_trx);
}

static uint64_t calypso_tpu_read(void *o, hwaddr off, unsigned sz) {
    CalypsoTRX *s=o; if (off==TPU_IT_DSP_PG) return s->dsp_page;
    return (off/2<CALYPSO_TPU_SIZE/2)?s->tpu_regs[off/2]:0;
}
static void calypso_tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz) {
    CalypsoTRX *s=o; if (off/2<CALYPSO_TPU_SIZE/2) s->tpu_regs[off/2]=val;
    if (off==TPU_CTRL) {
        static int tpu_log = 0;
        if (++tpu_log <= 50)   /* fprintf inconditionnel (GAP-1 ÉTAPE 0 : tracer si l'ARM arme le TPU) */
            fprintf(stderr, "[trx] TPU_CTRL WR val=0x%04x (EN=%d DSP_EN=%d) fn=%u\n",
                    (unsigned)val, !!(val&TPU_CTRL_EN), !!(val&TPU_CTRL_DSP_EN), s->fn);
    }
    if (off==TPU_CTRL && (val&TPU_CTRL_EN)) {
        s->tpu_regs[TPU_CTRL/2] &= ~(TPU_CTRL_EN|TPU_CTRL_IDLE);
        /* DMA immediately — no timer delay. The firmware has already
         * finished writing the write page before setting TPU_CTRL_EN.
         * A 1ns timer caused a race condition where the DMA would fire
         * before the write page was fully populated. */
        calypso_dsp_done(s);
    }
    if (off==TPU_INT_CTRL) {
        static int ictrl_log = 0;
        if (++ictrl_log <= 30)   /* fprintf inconditionnel (GAP-1 ÉTAPE 0 : DSP_FRAME armé ?) */
            fprintf(stderr, "[trx] INT_CTRL WR val=0x%02x (MCU_FRAME=%d DSP_FRAME=%d DSP_FORCE=%d) fn=%u\n",
                    (unsigned)val,
                    !!(val&ICTRL_MCU_FRAME), !!(val&ICTRL_DSP_FRAME),
                    !!(val&ICTRL_DSP_FRAME_FORCE), s->fn);
    }
    if (off==TPU_INT_CTRL && !(val&ICTRL_MCU_FRAME) && !s->tdma_running) calypso_tdma_start(s);
    if (off==TPU_IT_DSP_PG) s->dsp_page=val&1;
}
static const MemoryRegionOps calypso_tpu_ops = {
    .read=calypso_tpu_read,.write=calypso_tpu_write,.endianness=DEVICE_LITTLE_ENDIAN,
    .valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},
};
static uint64_t calypso_tpu_ram_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off/2<CALYPSO_TPU_RAM_SIZE/2)?s->tpu_ram[off/2]:0;}
static void calypso_tpu_ram_write(void *o,hwaddr off,uint64_t v,unsigned sz){
    CalypsoTRX*s=o;
    if(off/2<CALYPSO_TPU_RAM_SIZE/2) s->tpu_ram[off/2]=v;
    /* Probe gated par CALYPSO_DEBUG=TPU_RAM. Log les 50 premières writes
     * + chaque 1000ème pour visualiser le rythme de programmation TPU
     * par le firmware (l1s_rx_win_ctrl, tpu_enq_*, etc.). */
    static unsigned tpu_ram_wr = 0;
    tpu_ram_wr++;
    if (tpu_ram_wr <= 50 || (tpu_ram_wr % 1000) == 0) {
        if (calypso_debug_enabled("TPU_RAM")) {
            fprintf(stderr,
                "[calypso-trx] TPU_RAM WR #%u off=0x%04x val=0x%04x fn=%u\n",
                tpu_ram_wr, (unsigned)off, (unsigned)v, s->fn);
        }
    }
}
static const MemoryRegionOps calypso_tpu_ram_ops={.read=calypso_tpu_ram_read,.write=calypso_tpu_ram_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- TSP ---- */
static uint64_t calypso_tsp_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off==TSP_RX_REG)?0xFFFF:(off/2<CALYPSO_TSP_SIZE/2)?s->tsp_regs[off/2]:0;}
static void calypso_tsp_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TSP_SIZE/2)s->tsp_regs[off/2]=v;}
static const MemoryRegionOps calypso_tsp_ops={.read=calypso_tsp_read,.write=calypso_tsp_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- ULPD ---- */
static uint64_t calypso_ulpd_read(void *o,hwaddr off,unsigned sz){
    CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return 0;
    switch(off){case ULPD_SETUP_CLK13:return 0x2003;case ULPD_COUNTER_HI:s->ulpd_counter+=100;return(s->ulpd_counter>>16)&0xFFFF;
    case ULPD_COUNTER_LO:return s->ulpd_counter&0xFFFF;case ULPD_GAUGING_CTRL:return 1;case ULPD_GSM_TIMER:return s->fn&0xFFFF;
    default:return(off/2<CALYPSO_ULPD_SIZE/2)?s->ulpd_regs[off/2]:0;}
}
static void calypso_ulpd_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return;if(off/2<CALYPSO_ULPD_SIZE/2)s->ulpd_regs[off/2]=v;}
static const MemoryRegionOps calypso_ulpd_ops={.read=calypso_ulpd_read,.write=calypso_ulpd_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=2},.impl={.min_access_size=1,.max_access_size=2},};

/* ---- SIM (forwarded to calypso_sim.c) ---- */
static uint64_t calypso_sim_read(void *o, hwaddr off, unsigned sz)
{
    CalypsoTRX *s = o;
    return calypso_sim_reg_read(s->sim, off);
}
static void calypso_sim_write(void *o, hwaddr off, uint64_t v, unsigned sz)
{
    CalypsoTRX *s = o;
    calypso_sim_reg_write(s->sim, off, (uint16_t)v);
}
static const MemoryRegionOps calypso_sim_ops = {
    .read = calypso_sim_read,
    .write = calypso_sim_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- vCPU idle governor (host CPU-leak / thermal fix) -------------------
 * The osmocom-bb L1 firmware (apps/layer1/main.c) runs a side-effect-free
 * super-loop with NO WFI:
 *     while (1) { l1a_compl_execute(); osmo_timers_update();
 *                 sim_handler(); l1a_l23_handler(); }
 * On silicon a dedicated baseband core spinning is free. Under -icount auto
 * QEMU must emulate that spin at ~real-time and therefore pins one host core
 * at 99.9% forever (observed: the vCPU/TCG thread, PC bouncing across
 * l1a_compl_execute/l1a_l23_handler — the empty poll, not real work).
 *
 * Fix: we are called from the frame-IRQ *lower* callback (~1 ms after the
 * raise), i.e. once the per-frame work for this TDMA tick is done. If the
 * guest PC is back inside the L1 idle super-loop, park the vCPU
 * (cs->halted = 1). The next TPU-frame / UART (L1CTL) / SIM interrupt clears
 * halted and resumes execution exactly where it left off — invisible to the
 * guest because the loop is a pure poll. Under icount the halt lets QEMU
 * warp virtual time to the next timer and the host core sleeps.
 *
 * Safety:
 *  - cpu_handle_halt() refuses to halt while an IRQ is pending
 *    (cpu_has_work) → never stalls active interrupt servicing.
 *  - PC-gating to [lo,hi] → we only park while genuinely in the L1
 *    super-loop, never mid-DSP / mid-handler real work. Outside the window
 *    we do nothing, so other code paths cannot regress.
 *  - Opt-out: CALYPSO_CPU_IDLE=0. Window override (hex):
 *    CALYPSO_IDLE_PC_LO / CALYPSO_IDLE_PC_HI. Window 0 = halt whenever no
 *    IRQ is pending (rely on cpu_has_work only).
 */
static void calypso_cpu_idle_park(void)
{
    static int      enabled = -1;
    static uint64_t lo, hi, parked_n;
    if (enabled < 0) {
        const char *e = getenv("CALYPSO_CPU_IDLE");
        const char *l = getenv("CALYPSO_IDLE_PC_LO");
        const char *h = getenv("CALYPSO_IDLE_PC_HI");
        enabled = (e && *e == '0') ? 0 : 1;
        lo = l ? strtoull(l, NULL, 0) : 0x00823000ULL; /* l1a_l23_handler .. */
        hi = h ? strtoull(h, NULL, 0) : 0x00826000ULL; /* .. l1a_compl_execute */
        fprintf(stderr, "[cpu-idle] governor %s window=[0x%llx,0x%llx]\n",
                enabled ? "ON (opt-out CALYPSO_CPU_IDLE=0)" : "OFF",
                (unsigned long long)lo, (unsigned long long)hi);
    }
    if (!enabled) return;

    CPUState *cs = first_cpu;
    if (!cs) return;

    uint64_t pc = (cs->cc && cs->cc->get_pc) ? cs->cc->get_pc(cs) : 0;
    if (lo && hi && (pc < lo || pc >= hi))
        return;                 /* not in the L1 idle loop — leave it running */

    cs->halted = 1;
    cpu_exit(cs);               /* break the current TB so the halt takes now */

    if ((++parked_n % 5000) == 0 && calypso_timer_log())
        fprintf(stderr, "[cpu-idle] parked #%llu pc=0x%llx\n",
                (unsigned long long)parked_n, (unsigned long long)pc);
}

/* ---- TDMA ---- */
/* === NB -> DATA_IND : real BCCH SI from GSMTAP 4730 into a_cd (real DSP path)
 * ===========================================================================
 * Listens for grgsm-decoded System Information (RR, BCCH) on UDP GSMTAP 4730
 * and presents the 23-byte L2 block in the DSP a_cd cells (dsp word 0x09D0+),
 * which the ARM reads via API RAM 0x03A0+ when it runs task=24 (ALLC/CCCH).
 * Pairs with CALYPSO_FORCE_NB (supplies d_task_d/d_burst_d so prim_rx_nb does
 * not bail "EMPTY"). Same injection philosophy as the FB d_fb_det wiring: the
 * real DSP CCCH demod does not produce a_cd, so the real decoded SI is placed
 * in its own cells. Opt-in via CALYPSO_NB_DATAIND=1. The 23B SI source is the
 * genuine grgsm decode of the BTS I/Q (si_bridge -> GSMTAP 4730). */
#define NBDI_GSMTAP_HDR  16
#define NBDI_ACD_WORD    0x09D2   /* a_cd[0] in DSP data space (firmware: &a_cd=0xFFD003A4 -> data[0x3A4/2+0x800]=0x09D2; a_cd[3]/L2 byte0 -> 0x09D5/ARM 0x03A4) */
static int     g_nbdi      = -1;  /* -1 uninit, 0 off, 1 on */
static int     g_nbdi_fd   = -1;
static uint8_t g_nbdi_si[6][23];  /* latest SI per type: SI1,SI2,SI3,SI4,2bis,2ter */
static bool    g_nbdi_have[6];
static int     g_nbdi_rr;          /* round-robin cursor over available SI types */

static int nbdi_slot_for_mt(uint8_t mt)
{
    switch (mt) {
    case 0x19: return 0;  /* SI1    */
    case 0x1a: return 1;  /* SI2    */
    case 0x1b: return 2;  /* SI3    */
    case 0x1c: return 3;  /* SI4    */
    case 0x1d: return 4;  /* SI2bis */
    case 0x1e: return 5;  /* SI2ter */
    default:   return -1;
    }
}

static void nbdi_open(void)
{
    const char *e = getenv("CALYPSO_NB_DATAIND");
    g_nbdi = (calypso_orch() && !(e && e[0] == '0')) ? 1 : 0;
    if (!g_nbdi)
        return;
    const char *pe = getenv("CALYPSO_SHUNT_GSMTAP_PORT");
    int port = (pe && *pe) ? atoi(pe) : 4730;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { g_nbdi = 0; return; }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        fprintf(stderr, "[nb-di] bind(:%d) failed (port busy?) — NB->DATA_IND off\n", port);
        close(fd); g_nbdi = 0; return;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    g_nbdi_fd = fd;
    fprintf(stderr, "[nb-di] GSMTAP SI listener on :%d -> a_cd (real DSP path)\n", port);
}

/* Read l1s.current_time.fn (firmware L1 FN) from ARM RAM. This is the clock the
 * firmware schedules its BCCH-Norm block on and reads a_cd at burst 3 (fn%51==5);
 * nbdi must select the SI type and gate the a_cd write on THIS, not s->fn. Default
 * addr 0x836508, env override CALYPSO_L1S_FN_ADDR — identical to the shunt. */
static uint32_t nbdi_l1s_fn(void)
{
    static uint32_t addr = 0;
    if (!addr) {
        const char *e = getenv("CALYPSO_L1S_FN_ADDR");
        addr = (e && *e) ? (uint32_t)strtoul(e, NULL, 0) : 0x836508;
    }
    uint32_t v = 0;
    cpu_physical_memory_read(addr, &v, sizeof(v));
    return le32_to_cpu(v);
}

static void nbdi_poll_and_present(CalypsoTRX *s)
{
    if (g_nbdi < 0)
        nbdi_open();
    if (g_nbdi != 1 || g_nbdi_fd < 0 || !s || !s->dsp || !s->dsp->data)
        return;

    /* Drain all pending GSMTAP datagrams; latch SI blocks by type. */
    uint8_t buf[256];
    for (;;) {
        ssize_t n = recvfrom(g_nbdi_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0)
            break;
        if (n < NBDI_GSMTAP_HDR + 3)
            continue;
        const uint8_t *l2 = buf + NBDI_GSMTAP_HDR;
        int l2len = (int)n - NBDI_GSMTAP_HDR;
        if (l2[1] != 0x06)                 /* RR protocol discriminator only */
            continue;
        int slot = nbdi_slot_for_mt(l2[2]);
        if (slot < 0)                      /* SI types only (ignore AGCH/PCH) */
            continue;
        int cl = l2len < 23 ? l2len : 23;
        memcpy(g_nbdi_si[slot], l2, cl);
        for (int i = cl; i < 23; i++)
            g_nbdi_si[slot][i] = 0x2b;     /* L2 fill */
        g_nbdi_have[slot] = true;
    }

    /* Gate the a_cd write to the firmware's BCCH-Norm quad and pick the SI type
     * by the GSM 05.02 6.3.1.3 TC schedule, keyed off the FIRMWARE L1 FN — the
     * same clock the firmware arms BCCH_NORM (fn%51 in {2,3,4,5}) and reads a_cd
     * at burst 3 (fn%51==5). TC=(fn/51)%8 is CONSTANT across the 4-burst block,
     * so the chosen SI is stable across the block (no torn block, no RR). */
    uint32_t fn_fw = nbdi_l1s_fn();
    uint32_t pos = fn_fw % 51u;
    if (pos < 2u || pos > 5u)        /* only the BCCH-Norm quad (fn%51 in {2,3,4,5}) */
        return;
    unsigned tc = (unsigned)((fn_fw / 51u) % 8u);
    /* TC0->SI1 TC1->SI2 TC2->SI3 TC3->SI4 TC4->SI2ter TC5->SI2bis TC6->SI3 TC7->SI4 */
    static const int tc_slot[8] = { 0, 1, 2, 3, 5, 4, 2, 3 };
    int slot = tc_slot[tc];
    if (!g_nbdi_have[slot]) {          /* fall back to SI3, then any available type */
        if (g_nbdi_have[2]) {
            slot = 2;
        } else {
            slot = -1;
            for (int k = 0; k < 6; k++)
                if (g_nbdi_have[k]) { slot = k; break; }
        }
    }
    if (slot < 0)
        return;
    g_nbdi_rr = slot;                  /* retained only for the present-SI log line */

    const uint8_t *m = g_nbdi_si[slot];
    uint16_t *d = s->dsp->data;
    d[NBDI_ACD_WORD + 0] = 0x0000;         /* a_cd[0] FIRE bits -> CRC pass */
    d[NBDI_ACD_WORD + 1] = 0x0000;         /* a_cd[1] */
    d[NBDI_ACD_WORD + 2] = 0x0000;         /* a_cd[2] num_biterr */
    for (int k = 0; k < 12; k++) {         /* a_cd[3..14] = 23 bytes (12 words) */
        uint8_t lo = m[2 * k];
        uint8_t hi = (2 * k + 1 < 23) ? m[2 * k + 1] : 0x2b;
        d[NBDI_ACD_WORD + 3 + k] = (uint16_t)(lo | (hi << 8));
    }
    static unsigned nlog = 0;
    if (nlog++ < 40 || (nlog % 200) == 0)
        fprintf(stderr, "[nb-di] present SI mt=0x%02x slot=%d #%u -> a_cd[3..]\n",
                m[2], slot, nlog);
}

/* === SB sync: real BSIC+FN from grgsm (UDP 4731) -> a_sch (real DSP path) ===
 * ORCH-only. grgsm decodes the real SCH burst and sends "SCH1"+int32{bsic,fn,toa}
 * on UDP 4731. We encode the 25-bit SCH word (inverse of the firmware's
 * l1s_decode_sb) and write the DSP a_sch cells, so the mobile syncs to the REAL
 * BTS frame number instead of SB=0x00000000 -> BSIC=0/FN=52 -> BCCH window
 * misaligned -> sync timeout. a_sch is PAGED (p0/p1) -> write both. */
#define SBI_SCH_PORT 4731
static int      g_sbi      = -1;   /* -1 uninit, 0 off, 1 on */
static int      g_sbi_fd   = -1;
static int      g_sbi_bsic = -1;
static uint32_t g_sbi_fn   = 0;

static void sbi_open(void)
{
    g_sbi = calypso_orch() ? 1 : 0;
    if (!g_sbi)
        return;
    const char *pe = getenv("CALYPSO_SHUNT_SCH_PORT");
    int port = (pe && *pe) ? atoi(pe) : SBI_SCH_PORT;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { g_sbi = 0; return; }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        fprintf(stderr, "[sb-sync] bind(:%d) failed (busy?) — SB sync off\n", port);
        close(fd); g_sbi = 0; return;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    g_sbi_fd = fd;
    fprintf(stderr, "[sb-sync] SCH listener on :%d -> a_sch (real BSIC/FN)\n", port);
}

/* Encode the 25-bit SCH word from BSIC + GSM frame number (exact inverse of the
 * firmware l1s_decode_sb in prim_fbsb.c). */
static uint32_t sbi_encode(int bsic, uint32_t fn)
{
    uint32_t t1 = fn / 1326u;          /* 1326 = 51*26 */
    uint32_t t2 = fn % 26u;
    uint32_t t3 = fn % 51u;
    uint32_t t3p = (t3 >= 1u) ? ((t3 - 1u) / 10u) : 0u;
    uint32_t sb = 0;
    sb |= ((uint32_t)(bsic & 0x3f)) << 2;   /* BSIC -> sb[7:2]   */
    sb |= (t1 & 1u) << 23;                  /* t1[0]  -> sb[23]  */
    sb |= (t1 & 0x1feu) << 7;               /* t1[8:1]-> sb[15:8]*/
    sb |= (t1 >> 9) & 3u;                    /* t1[10:9]->sb[1:0] */
    sb |= (t2 & 0x1fu) << 18;               /* t2     -> sb[22:18]*/
    sb |= (t3p & 1u) << 24;                  /* t3p[0] -> sb[24]  */
    sb |= ((t3p >> 1) & 3u) << 16;           /* t3p[2:1]->sb[17:16]*/
    return sb;
}

static void sbi_poll_and_present(CalypsoTRX *s)
{
    if (g_sbi < 0)
        sbi_open();
    if (g_sbi != 1 || g_sbi_fd < 0 || !s || !s->dsp || !s->dsp->data)
        return;

    uint8_t buf[64];
    for (;;) {
        ssize_t n = recvfrom(g_sbi_fd, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0)
            break;
        if (n >= 16 && buf[0]=='S' && buf[1]=='C' && buf[2]=='H' && buf[3]=='1') {
            int32_t bsic, fn, toa;
            memcpy(&bsic, buf + 4, 4);
            memcpy(&fn,   buf + 8, 4);
            memcpy(&toa,  buf + 12, 4);
            (void)toa;
            g_sbi_bsic = (int)bsic;
            g_sbi_fn   = (uint32_t)fn;
        }
    }
    if (g_sbi_bsic < 0)
        return;

    uint32_t sb = sbi_encode(g_sbi_bsic, g_sbi_fn);
    uint16_t *d = s->dsp->data;
    /* a_sch[0] CRC (bit8=0 => OK): p0 0x0837 / p1 0x084B */
    d[0x0837] = 0x0000; d[0x084B] = 0x0000;
    /* a_sch[3] = sb[15:0] : p0 0x083A / p1 0x084E */
    d[0x083A] = (uint16_t)(sb & 0xFFFF); d[0x084E] = (uint16_t)(sb & 0xFFFF);
    /* a_sch[4] = sb[24:16] : p0 0x083B / p1 0x084F */
    d[0x083B] = (uint16_t)(sb >> 16); d[0x084F] = (uint16_t)(sb >> 16);
    /* a_serv_demod[D_TOA] on-time (23) so the SB passes the "future" check:
     * p0 0x0830 / p1 0x0844 */
    d[0x0830] = (uint16_t)23; d[0x0844] = (uint16_t)23;

    static unsigned nlog = 0;
    if (nlog++ < 40 || (nlog % 200) == 0)
        fprintf(stderr, "[sb-sync] a_sch <- bsic=%d fn=%u sb=0x%08x #%u\n",
                g_sbi_bsic, g_sbi_fn, (unsigned)sb, nlog);
}

static void calypso_frame_irq_lower(void *o){
    /* Frame IRQ lower counter — log thinned 1/1000 pour drift detection. */
    static uint64_t firq_lower_n = 0;
    firq_lower_n++;
    if ((firq_lower_n % 1000) == 0 && calypso_timer_log()) {
        fprintf(stderr, "[frame_irq] lower #%llu t_virt=%lld\n",
                (unsigned long long)firq_lower_n,
                (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    qemu_irq_lower(((CalypsoTRX*)o)->irqs[CALYPSO_IRQ_TPU_FRAME]);

    /* DSP shunt service hook (no-op si shunt off). Servir APRÈS le lower
     * pour que le mock écrive ses résultats entre deux ticks ARM. */
    calypso_dsp_shunt_on_frame_tick();

    /* NB -> DATA_IND : present real grgsm SI in a_cd (opt-in). */
    nbdi_poll_and_present((CalypsoTRX *)o);

    /* SB sync : present real grgsm BSIC/FN in a_sch (ORCH). */
    sbi_poll_and_present((CalypsoTRX *)o);

    /* Per-frame work for this tick is done — park the vCPU if the guest is
     * back in its idle super-loop, so the host core sleeps until the next
     * interrupt instead of spinning at 100%. See calypso_cpu_idle_park(). */
    calypso_cpu_idle_park();
}

/* CALYPSO_TDMA_REALTIME=1 : pin tdma_timer to QEMU_CLOCK_REALTIME so
 * the 4.6 ms GSM frame cadence is wall-clock, independent of guest
 * cycle rate. Fixes L23 sync timeouts under icount=auto (tdma_tick
 * was firing at ~17 Hz instead of 217 Hz when virtual time lagged).
 * Default unset = VIRTUAL clock (legacy behaviour). Decision made
 * once at first tick and cached. */
static QEMUClockType calypso_tdma_clock(void) {
    static int cached = -1;
    if (cached < 0) {
        /* DEFAULT VIRTUAL (2026-05-29 v2, single-domain) : tout le système
         * (ARM, DSP, radio via clk-master FN, mobile) doit partager UNE base
         * de temps = le temps virtuel QEMU, comme le HW partage l'horloge RF.
         * Le défaut REALTIME (wall-clock) faisait courir la radio/mobile à
         * 100% wall pendant que l'ARM virtuel traîne à ~7% → drift → LOST.
         * En VIRTUAL le tdma_tick devient maître du FN (cf section 0) et la
         * radio suit le rythme virtuel : lent en wall mais zéro drift, et
         * insensible au debug/charge host. Opt-in wall via CALYPSO_TDMA_REALTIME=1. */
        const char *e = getenv("CALYPSO_TDMA_REALTIME");
        cached = (e && *e == '1') ? 1 : 0;
        fprintf(stderr, "[calypso-trx] tdma_timer clock = %s\n",
                cached ? "REALTIME (wall-clock 217 Hz, opt-in)" : "VIRTUAL (single-domain, default)");
    }
    return cached ? QEMU_CLOCK_REALTIME : QEMU_CLOCK_VIRTUAL;
}

static void calypso_tdma_tick(void *opaque) {
    CalypsoTRX *s = opaque;

    /* Halt-sync : if the ARM CPU is paused (GDB stop, monitor stop),
     * also pause DSP ticking. Otherwise tdma_timer (REALTIME) keeps
     * firing, c54x_run keeps advancing the DSP, qemu.log keeps growing
     * — making GDB inspection useless because the system state drifts
     * under the breakpoint. Re-arm timer so we resume cleanly. */
    if (!runstate_is_running()) {
        if (s->tdma_running) {
            timer_mod_ns(s->tdma_timer,
                         qemu_clock_get_ns(calypso_tdma_clock()) + GSM_TDMA_NS);
        }
        return;
    }

    int64_t entry_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t t_clk = 0, t_uart = 0, t_dspboot = 0, t_dspirq = 0,
            t_bsp = 0, t_ul = 0;
    /* Sync s->fn to the wall-clock fn from clk_master_thread. The
     * pthread is the source of truth for "current GSM frame number" —
     * it ticks at exact 4.615ms wall using clock_nanosleep ABSTIME.
     * Si le pthread est encore en init (g_wall_fn=0), on garde notre
     * propre compteur en fallback pour ne pas freeze le DSP. */
    {
        if (calypso_tdma_clock() == QEMU_CLOCK_VIRTUAL) {
            /* SINGLE-DOMAIN (default) : le tdma_tick VIRTUAL est le MAÎTRE du
             * FN — il avance g_wall_fn d'une frame par tick virtuel ET envoie
             * le CLK à la radio (cf section 0). La radio (ipc-device/trx-ipc)
             * suit donc le temps virtuel de QEMU → zéro drift ARM↔radio↔mobile.
             * (Le pthread wall clk-master n'est PAS démarré dans ce mode.) */
            uint32_t fn = __atomic_add_fetch(&g_wall_fn, 1, __ATOMIC_RELEASE)
                        % GSM_HYPERFRAME;
            s->fn = fn;
            if (s->clk_fd >= 0) {
                uint8_t pkt[4];
                pkt[0] = (fn >> 24) & 0xFF; pkt[1] = (fn >> 16) & 0xFF;
                pkt[2] = (fn >>  8) & 0xFF; pkt[3] =  fn        & 0xFF;
                (void)sendto(s->clk_fd, pkt, 4, 0,
                             (struct sockaddr *)&s->clk_peer, sizeof(s->clk_peer));
            }
        } else {
            /* REALTIME (opt-in) : le pthread wall clk-master est maître, on le
             * suit. Si encore en init (g_wall_fn=0), fallback compteur local. */
            uint32_t wfn = __atomic_load_n(&g_wall_fn, __ATOMIC_ACQUIRE);
            if (wfn != 0) {
                s->fn = wfn % GSM_HYPERFRAME;
            } else {
                s->fn = (s->fn + 1) % GSM_HYPERFRAME;
            }
        }
    }

    /* TDMA tick counter — log thinned 1/1000 (~4.6s wall) pour drift detection.
     * Variables locales pour cumul DSP insn (utilisées plus bas). */
    static uint64_t tdma_ticks = 0;
    static uint64_t dsp_insn_total = 0;
    tdma_ticks++;
    int dsp_n_exec_2 = 0, dsp_n_exec_5 = 0; /* updated by c54x_run calls */

    /* ── 0. CLK send delegated to clk_master_thread (jitter-free) ── */
    t_clk = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* ── 1. UART poll: deliver pending chardev bytes to firmware ── */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
    }
    if (g_uart_irda) {
        calypso_uart_poll_backend(g_uart_irda);
        calypso_uart_kick_rx(g_uart_irda);
    }
    t_uart = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* ── 2. DSP boot phase ── */
    /* DSP budget per c54x_run call. 256000 ≈ 1 frame nominale du c54x réel
     * (≈104 MHz × 4.615 ms = 480k cycles total, ici budget par appel × 2 appels).
     * Sous DSP-overload (fb-det compute), 2× ce budget = ~18.6 ms wall sur le
     * tdma_tick alors que la frame GSM dure 4.615 ms → drift wall/qfn 3.6×.
     * Override via CALYPSO_DSP_BUDGET pour mesurer A/B sans recompiler. Voir
     * REPORT_CLAUDE_WEB_20260516_DSP_OVERRUN.md. */
    static int dsp_budget = -1;
    if (dsp_budget < 0) {
        const char *e = getenv("CALYPSO_DSP_BUDGET");
        dsp_budget = (e && *e) ? atoi(e) : 256000;
        if (dsp_budget < 1000) dsp_budget = 1000;
        TRX_LOG("CALYPSO_DSP_BUDGET = %d insn/c54x_run (default 256000)",
                dsp_budget);
    }
    /* GATE DSP_SHUNT : si le shunt est actif, le mock cote ARM remplace
     * la DSP. Skip TOUS les c54x_run -> le c54x emule n'execute aucune
     * instruction, ne touche pas a la DARAM, ne fabrique pas de d_dsp_page
     * concurrent avec le mock. */
    if (s->dsp && s->dsp->running && !s->dsp_init_done && !calypso_dsp_shunt_active()) {
        if (!s->dsp->idle)
            dsp_n_exec_2 = c54x_run(s->dsp, dsp_budget);
        if (s->dsp->idle) {
            s->dsp_init_done = true;
            TRX_LOG("DSP init complete (first IDLE reached)");
        }
    } else if (calypso_dsp_shunt_active() && !s->dsp_init_done) {
        /* En shunt mode, on saute l'init DSP "boot" — le mock prend le relais. */
        s->dsp_init_done = true;
    }
    t_dspboot = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* ── 3. DMA is NOT done here ──
     * On real Calypso, the TPU scenario triggers the DMA when the
     * firmware writes TPU_CTRL with EN bit. This happens in
     * calypso_dsp_done() (the TPU_CTRL_EN timer callback).
     * Doing DMA here would copy a STALE write page because the
     * firmware hasn't written the new tasks yet (it writes them
     * in l1s_compl() which runs in the IRQ4 handler AFTER this tick). */

    /* ── 4. DSP frame interrupt ──
     * Three conditions for periodic INT3 fire:
     *   - INT_CTRL.ICTRL_DSP_FRAME (bit 2) = persistent enable at TPU,
     *     polarity INVERTED (bit clear = enabled).
     *   - DSP IMR bit 3 (C54X_INT_FRAME_BIT) = mask enable at DSP.
     *     Empirically: firing INT3 while IMR bit 3 = 0 perturbs the
     *     firmware boot path (DSP wakes from IDLE without expecting it,
     *     takes wrong code path, never reaches IMR-init at PC=0x0810,
     *     dead-locks). Respecting IMR matches the "hardware INT line
     *     gated by IMR" model used on Calypso.
     *   - TPU_CTRL.DSP_EN (bit 4) = one-shot force, alternative path.
     *     Bypasses IMR (explicit hardware override). */
    if (s->dsp && s->dsp->running) {
        bool was_idle = s->dsp->idle;

        bool tpu_armed = !(s->tpu_regs[TPU_INT_CTRL/2] & ICTRL_DSP_FRAME);
        bool imr_armed = !!(s->dsp->imr & (1 << C54X_INT_FRAME_BIT));
        /* GAP-1 FIX FAITHFUL (2026-06-23) : l'IT trame DSP est une LIGNE INT3
         * CÂBLÉE pilotée par le TPU — elle n'est PAS gatée par l'IMR du DSP en
         * amont. L'ancien `&& imr_armed` (ajout empirique, cf 1419-1423) inversait
         * la causalité silicium et créait un VERROU CIRCULAIRE : le DSP a besoin de
         * recevoir l'IT pour atteindre son code d'armement IMR (ROM 0x702b
         * `OR #0x001d,IMR`), mais l'IT était gatée sur IMR déjà armé → prouvé au
         * runtime (tpu_armed=1, imr_armed=0, fire=0 à jamais). On délivre l'IT dès
         * que le TPU l'arme ; c'est le c54x (IMR/INTM dans c54x_interrupt_ex, déjà
         * fidèle SPRU131) qui décide de vectoriser ou juste sortir d'IDLE. Firmware
         * INCHANGÉ. `imr_armed` conservé pour la sonde FRAME-GATE seulement. */
        /* TEST 2026-06-23 : revert temporaire du fix imr_armed pour falsifier
         * l'hypothèse « firer l'IT masquée perturbe le DSP → derail ». */
        /* FIX FIDELE (restaure 2026-06-24) : l'IT trame DSP est une LIGNE INT3
         * CABLEE pilotee par le TPU, PAS gatee par l'IMR du DSP. Le `&& imr_armed`
         * (test reverte 2026-06-23, jamais restaure) cree un VERROU CIRCULAIRE
         * prouve au runtime (tpu_armed=1 imr_armed=0 fire=0 a jamais) : le DSP a
         * besoin de l'IT pour atteindre son armement IMR (ROM 0x702b OR #0x1d,IMR),
         * mais l'IT etait gatee sur IMR deja arme. On livre l'IT des que le TPU
         * l'arme ; c54x_interrupt_ex (deja fidele INTM/IMR/IDLE) decide de
         * vectoriser, poser IFR, ou sortir d'IDLE. Firmware INCHANGE = on EXECUTE
         * le DSP, on ne l'orchestre pas. Gated CALYPSO_FRAME_FAITHFUL pour A/B. */
        static int g_frame_faithful = -1;
        if (g_frame_faithful < 0)
            g_frame_faithful = getenv("CALYPSO_FRAME_FAITHFUL") ? 1 : 0;
        bool periodic_armed = g_frame_faithful ? tpu_armed
                                               : (tpu_armed && imr_armed);
        bool force_pulse    = !!(s->tpu_regs[TPU_CTRL/2] & TPU_CTRL_DSP_EN);
        /* GAP-1 ÉTAPE 0 : état du gate au runtime (le verrou circulaire tpu_armed
         * && imr_armed). Montre si l'IT trame est bloquée par imr_armed seul. */
        { static unsigned gate_log = 0;
          if (gate_log < 50) { gate_log++;
            fprintf(stderr, "[trx] FRAME-GATE tpu_armed=%d imr_armed=%d(IMR=0x%04x) "
                    "force=%d -> fire=%d fn=%u\n",
                    tpu_armed, imr_armed, (unsigned)s->dsp->imr,
                    force_pulse, (periodic_armed||force_pulse), s->fn); } }
        if (periodic_armed || force_pulse) {
            g_c54x_int3_src = 1;
            c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
            if (force_pulse)
                s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_DSP_EN;
            /* periodic_armed: do NOT clear — hardware-persistent enable. */
        }

        /* ── 5. Run DSP (RX path : FBSB demod, BCCH/CCCH decode) ──
         * Budget partagé avec section 2 via static `dsp_budget` (env var
         * CALYPSO_DSP_BUDGET). NE PAS supprimer ce 2e appel — il porte le
         * compute RX critique (Claude web review 2026-05-16).
         *
         * GATE DSP_SHUNT : skip si shunt actif (cf section 2 commentaire). */
        if (!s->dsp->idle && !calypso_dsp_shunt_active()) {
            dsp_n_exec_5 = c54x_run(s->dsp, dsp_budget);
        }

        /* Do NOT clear tasks here — the firmware's l1s_compl() does
         * dsp_api_memset() on the write page at the start of each frame,
         * before tdma_sched_execute() writes new tasks. Clearing here
         * would erase tasks that the scheduler just programmed. */

        /* Only pulse API IRQ when DSP naturally reaches IDLE. */
        if (!was_idle && s->dsp->idle) {
            qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
        }
    }
    t_dspirq = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* [tdma] log : drift detection + budget DSP réel consommé par tick.
     * Cadence 1/1000 ticks (~4.6s wall en steady state). Indique :
     *   - tick #N (compteur cumulé)
     *   - fn (frame number)
     *   - t_virt (entry timestamp en ns virtual)
     *   - dsp_n_exec_2 (insn DSP exec dans section 2 — DSP boot/idle phase)
     *   - dsp_n_exec_5 (insn DSP exec dans section 5 — RX path post-IRQ)
     *   - budget = CALYPSO_DSP_BUDGET (default 256000)
     * Si dsp_n_exec_* << dsp_budget en steady state, ça signifie que le
     * DSP atteint IDLE avant d'épuiser son budget — on peut réduire le
     * budget sans dégrader. Si dsp_n_exec_* == dsp_budget en steady state,
     * le DSP est saturé et réduire le budget va casser fb-det. */
    dsp_insn_total += (uint64_t)(dsp_n_exec_2 + dsp_n_exec_5);
    if ((tdma_ticks % 1000) == 0) {
        fprintf(stderr,
                "[tdma] tick #%llu fn=%u t_virt=%lld "
                "dsp_n_exec_2=%d dsp_n_exec_5=%d dsp_insn_total=%llu budget=%d\n",
                (unsigned long long)tdma_ticks, s->fn, (long long)entry_t,
                dsp_n_exec_2, dsp_n_exec_5,
                (unsigned long long)dsp_insn_total, dsp_budget);
    }

    /* === POST-WATCH (revival dsp) : l'ARM poste-t-il jamais d_task_d/d_burst_d
     * (db_w command), et le DSP les voit-il ? On lit les mots depuis dsp_ram[]
     * (partagé = ce que le DSP lit via api_ram) ET dsp->data[] (copie privée
     * prog-fetch) pour distinguer "post jamais émis" (tout=0) de "post non
     * miroité" (RAM != DATA). Log seulement quand non-nul, capé 200. */
    if (s->dsp && s->dsp->data) {
        uint16_t r_td0 = s->dsp_ram[0x00], r_bd0 = s->dsp_ram[0x01];
        uint16_t r_td1 = s->dsp_ram[0x14], r_bd1 = s->dsp_ram[0x15];
        uint16_t d_td0 = s->dsp->data[0x800], d_bd0 = s->dsp->data[0x801];
        uint16_t d_td1 = s->dsp->data[0x814], d_bd1 = s->dsp->data[0x815];
        if (r_td0|r_bd0|r_td1|r_bd1|d_td0|d_bd0|d_td1|d_bd1) {
            static unsigned pw_log;
            if (pw_log < 200) {
                fprintf(stderr,
                  "[POST-WATCH] fn=%u RAM td0=%04x bd0=%04x td1=%04x bd1=%04x "
                  "DATA td0=%04x bd0=%04x td1=%04x bd1=%04x%s\n",
                  (unsigned)s->fn, r_td0, r_bd0, r_td1, r_bd1,
                  d_td0, d_bd0, d_td1, d_bd1,
                  (r_td0 != d_td0 || r_bd0 != d_bd0 ||
                   r_td1 != d_td1 || r_bd1 != d_bd1)
                    ? " *MIRROR-MISMATCH*" : "");
                pw_log++;
            }
        }
    }

    /* ── 6. BSP DL delivery is now driven by wall-clock drain timer in
     * calypso_bsp.c (bsp_drain_cb @ 5ms REALTIME). Decoupling fix 2026-05-24:
     * under icount=auto, tdma_tick fires too slowly (17 Hz) to drain the
     * BTS UDP stream (217 Hz arrival) — bursts went stale before delivery.
     * The drain timer runs at wall rate matching BTS, while DSP continues
     * to process samples at its virtual-clock pace. */
    t_bsp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* ── 6b. UL burst poll ──
     * The MCU→DSP write page exposes three independent UL task fields:
     *   d_task_u  (word 2) — generic UL: SDCCH/SACCH/FACCH/TCH NB
     *   d_task_ra (word 7) — RACH access burst (8 info bits → AB)
     *   d_burst_u (word 3) — TN selector
     * Each UL kind has its own d_task_*; the firmware (prim_rach.c,
     * prim_tx_nb.c) writes whichever applies. We must poll all of them
     * — polling only d_task_u silently drops every RACH attempt. */
    {
        uint16_t *wp = s->dsp_page ?
            &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
        uint16_t task_u  = wp[DB_W_D_TASK_U];
        uint16_t task_ra = wp[DB_W_D_TASK_RA];
        uint8_t  tn      = wp[DB_W_D_BURST_U] & 0x07;

        if (task_ra != 0 && s->dsp) {
            /* RACH: dsp_task_iq_swap(RACH_DSP_TASK, arfcn, 1) packs
             * task ID + ARFCN. The 8-bit RACH info is in NDB d_rach.
             * Burst encoding (gsm0503_rach_ext_encode) belongs in the
             * BSP UL path — see calypso_bsp.c.
             *
             * IMPORTANT : zero-init bits[148] before encode. libosmocoding
             * fills only the 41-bit sync + 36-bit FIRE-encoded data + 3-bit
             * tail (~80 bits total in the AB structure). The remaining 60
             * bits of guard period (positions 88..147) are NOT written by
             * the encoder ; without zero-init we'd transmit stack garbage
             * in the guard period, which BTS RACH detector treats as
             * out-of-sync noise → silent drop. Confirmed empirically via
             * burst hex print : same 8 trailing bits across all RAs before
             * this fix. */
            uint8_t bits[148] = {0};
            if (calypso_bsp_tx_rach_burst(s->fn, bits)) {
                calypso_bsp_send_ul(tn, s->fn, bits);
                static int rach_log = 0;
                if (++rach_log <= 20)
                    TRX_LOG("UL RACH task=0x%04x tn=%u fn=%u",
                            task_ra, tn, s->fn);
            }
            wp[DB_W_D_TASK_RA] = 0;
        }

        if (task_u != 0 && s->dsp) {
            /* NB UL : same zero-init reasoning as RACH path. */
            uint8_t bits[148] = {0};
            if (calypso_bsp_tx_burst(tn, s->fn, bits)) {
                calypso_bsp_send_ul(tn, s->fn, bits);
                static int ul_log = 0;
                if (++ul_log <= 20)
                    TRX_LOG("UL NB task=0x%04x tn=%u fn=%u",
                            task_u, tn, s->fn);
            }
            wp[DB_W_D_TASK_U] = 0;
        }
    }
    t_ul = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* ── 6.5 calypso_layer1.c — HLE L1 scaffold (CALYPSO_L1=c, default off) ──
     * Modèle L1 du DSP en C, cadencé ICI : s->fn est posé, l'I/Q de la trame est
     * déjà en data[0x2a00], et on écrit AVANT l'IRQ qui déclenche l1s_fbdet_resp
     * côté ARM → résultats lus dans la fenêtre 1ms. Gaté strict → mock (SHUNT) et
     * revival (vrai c54x, arc LLE/IPTR) intacts. Scaffold intermédiaire, PAS
     * l'endgame : le revival reste le but faithful, gardé vivant derrière le gate
     * off. ÉTAPE 0 = read-only (ratio de cohérence FCCH loggé, zéro write). */
    if (calypso_l1_c_active() && s->dsp && s->dsp->data) {
        calypso_layer1_tick(s->dsp, s->dsp_ram, s->fn);
    }

    /* ── 7. TPU FRAME IRQ → ARM L1 scheduler ── */
    {
        static FILE *firq_log = NULL;
        static int firq_count = 0;
        static int64_t prev_firq_t = 0;
        if (firq_count  < 2000 && calypso_timer_log()) {  /* DISABLED for baseline — re-enable by setting >0 */
            if (!firq_log) firq_log = fopen("/tmp/frame_irq.log", "w");
            if (firq_log) {
                int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                int64_t dt = prev_firq_t ? (now - prev_firq_t) : 0;
                int64_t target = now + GSM_TDMA_NS;
                fprintf(firq_log, "[frame-irq] raise t_virt=%" PRId64
                        " dt=%" PRId64 " next_target=%" PRId64
                        " gap_to_target=%" PRId64 " fn=%u #%d\n",
                        now, dt, target, (target - now), s->fn, firq_count);
                prev_firq_t = now;
                firq_count++;
            }
        }
    }
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);

    /* ── 8. Re-arm TDMA timer ──
     * FIX: anchor on entry_t (start of tick), not on exit_t. Otherwise
     * the work_dt of the body cumulates into the deadline and the TDMA
     * cadence drifts to (work_dt + GSM_TDMA_NS) instead of staying at
     * GSM_TDMA_NS exact.
     *
     * Si déjà en retard (work_dt > GSM_TDMA_NS), sauter aux frames suivantes
     * pour rester aligné sur la grille TDMA. Mimique silicon : la(les) frame(s)
     * sont perdues mais le timer ne dérive pas et le main loop n'est pas saturé
     * par des back-to-back catch-up. */
    {
        /* === Monotonic timer (drift-free rearm) 2026-05-28 ===
         *
         * Previous code used `entry_t_clk + GSM_TDMA_NS` as the next target.
         * entry_t_clk = wall time when the handler was actually dispatched,
         * which already includes any BQL/IRQ/work latency from the previous
         * fire. Therefore target absorbed that latency : on every late
         * dispatch (~200µs typical at 217 ticks/s), the next deadline
         * drifted by +200µs. After 1 wall second : ~45ms accumulated drift.
         * BTS measured 207 FN/sec wall vs expected 217 FN/sec — exactly
         * the 4.6% gap.
         *
         * Fix : anchor target on `last_target + GSM_TDMA_NS` (the IDEAL
         * deadline of the previous tick), not on `now`. Drift no longer
         * accumulates. If a deadline is already in the past at wake-up
         * (handler took >4.615ms), skip frames to stay on the absolute
         * TDMA grid and advance FN to match (mimics silicon : late frames
         * are *lost*, not retransmitted, but the timeline never lags).
         *
         * Activé seulement quand CALYPSO_TDMA_REALTIME=1 (= REALTIME clock).
         * En mode VIRTUAL legacy, virtual time advance is already lockstep
         * with guest cycles → drift par construction.
         */
        QEMUClockType tclk = calypso_tdma_clock();
        int64_t now = qemu_clock_get_ns(tclk);
        static int64_t last_target = 0;
        if (last_target == 0) {
            /* First tick: seed last_target from entry time so initial
             * scheduling is normal-paced. */
            last_target = (tclk == QEMU_CLOCK_REALTIME)
                          ? now
                          : entry_t;
        }
        int64_t target = last_target + GSM_TDMA_NS;
        int skipped = 0;
        while (target <= now) {
            target += GSM_TDMA_NS;
            skipped++;
        }
        last_target = target;

        /* No FN catchup needed — s->fn is sync'd to g_wall_fn at entry,
         * which is incremented by clk_master_thread independently. */

        {
            static int rearm_log_count = 0;
            if (rearm_log_count < 50) {
                fprintf(stderr, "[rearm-fix] last_target=%" PRId64 " target=%" PRId64
                        " now=%" PRId64 " gap_to_now=%" PRId64 " skipped=%d fn=%u\n",
                        last_target - (int64_t)GSM_TDMA_NS, target, now,
                        target - now, skipped, s->fn);
                rearm_log_count++;
            }
        }

        if (skipped > 0 && (s->fn % 100 == 0) && calypso_timer_log()) {
            fprintf(stderr, "[tdma-skip] fn=%u skipped=%d work_dt=%" PRId64 "\n",
                    s->fn, skipped, now - entry_t);
        }

        if (s->tdma_running) {
            timer_mod_ns(s->tdma_timer, target);
        }
    }

    {
        static FILE *tick_log = NULL;
        static int tick_count = 0;
        if (tick_count < 500 && calypso_timer_log()) {
            if (!tick_log) tick_log = fopen("/tmp/tdma_tick.log", "w");
            if (tick_log) {
                int64_t exit_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                fprintf(tick_log, "[tdma-tick] entry=%" PRId64 " exit=%" PRId64
                        " work_dt=%" PRId64 " fn=%u #%d\n",
                        entry_t, exit_t, (exit_t - entry_t), s->fn, tick_count);
                tick_count++;
            }
        }
    }

    /* Profile per sub-block: identifie quelle section consomme work_dt. */
    {
        static FILE *prof_log = NULL;
        static int prof_count = 0;
        if (prof_count < 200) {
            if (!prof_log) prof_log = fopen("/tmp/tdma_profile.log", "w");
            if (prof_log) {
                int64_t exit_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                fprintf(prof_log, "[prof] fn=%u clk=%" PRId64 " uart=%" PRId64
                        " dspboot=%" PRId64 " dspirq=%" PRId64 " bsp=%" PRId64
                        " ul=%" PRId64 " irq=%" PRId64 " total=%" PRId64
                        " #%d\n",
                        s->fn,
                        t_clk - entry_t,
                        t_uart - t_clk,
                        t_dspboot - t_uart,
                        t_dspirq - t_dspboot,
                        t_bsp - t_dspirq,
                        t_ul - t_bsp,
                        exit_t - t_ul,
                        exit_t - entry_t,
                        prof_count);
                prof_count++;
            }
        }
    }
}

static void calypso_tdma_start(CalypsoTRX *s)
{
    if (s->tdma_running) return;
    s->tdma_running = true;
    s->fn = 0;
    TRX_LOG("TDMA started");
    timer_mod_ns(s->tdma_timer,
                 qemu_clock_get_ns(calypso_tdma_clock()) + GSM_TDMA_NS);
}

/* ---- kick ----
 * Periodic CPU exit + main-loop wake. Whose role is to force the event
 * loop to service fd handlers (UDP bridge sockets, chardev) even when
 * the guest is in long TCG bursts.
 *
 * AUDIT FIX 2026-05-08 night : reverted to QEMU_CLOCK_REALTIME (was
 * moved to VIRTUAL on 2026-05-07 based on a faulty diagnosis).
 *
 * Rationale per Claude web event-loop audit :
 *  - Under -icount, VIRTUAL warps with guest progress. A VIRTUAL-clock
 *    kick fires "in sync" with the guest = tautologically useless,
 *    cpu_exit becomes a no-op (we're already in the main loop when the
 *    timer dispatches), and the kick contributes nothing.
 *  - REALTIME on the other hand advances independently and guarantees
 *    that fd handlers are serviced at wall-time intervals regardless
 *    of guest TCG burst length. This is precisely the original purpose.
 *  - The 2026-05-07 claim that REALTIME-driven cpu_exit was blocking
 *    VIRTUAL TDMA timers was wrong : cpu_exit terminates the current
 *    burst, the main loop runs the next one immediately, and virtual
 *    time is not gated on cpu_exit calls.
 *
 * The real culprit blocking the bridge under icount was the
 * `main_loop_wait(false)` recursive call in calypso_uart_rx_poll
 * (fixed in calypso_uart.c same session), not this kick timer.
 */
static QEMUTimer *g_kick_timer;
static void calypso_kick_cb(void *o){
    /* AUDIT INSTRUMENTATION 2026-05-08 night : confirm kick fires under
     * -icount auto. Per Claude web : if 0 hits in 5s wall → REALTIME timer
     * not armed correctly with icount. If N≈1000 hits/5s (5ms period) →
     * timer fires but cpu_exit/notify don't propagate to scheduler. */
    static unsigned kick_n;
    kick_n++;
    if ((kick_n <= 30 || (kick_n % 200) == 0) && calypso_timer_log()) {
        uint64_t vt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t rt = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
        fprintf(stderr, "[kick] fire #%u vt=%lu rt=%lu\n",
                kick_n, (unsigned long)vt, (unsigned long)rt);
    }

    CPUState*cpu=first_cpu;if(cpu)cpu_exit(cpu);qemu_notify_event();
    {
        static int pcb_threaded = -1;
        if (pcb_threaded < 0) {
            const char *e = getenv("CALYPSO_PCB_TICK_THREADS");
            pcb_threaded = (e && e[0] == '1') ? 1 : 0;
        }
        if (!pcb_threaded) {
            timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);
        }
    }
}

/* === Public invokers pour pcb tick threads ============================
 * Appelés depuis calypso_full_pcb.c thread bodies, avec BQL held. */
void calypso_trx_kick_invoke(void);
void calypso_trx_tdma_tick_invoke(void);
void calypso_trx_frame_irq_lower_invoke(void);

void calypso_trx_kick_invoke(void)
{
    calypso_kick_cb(NULL);
}

void calypso_trx_tdma_tick_invoke(void)
{
    if (g_trx) calypso_tdma_tick(g_trx);
}

void calypso_trx_frame_irq_lower_invoke(void)
{
    if (g_trx) calypso_frame_irq_lower(g_trx);
}

/* ---- Sercomm burst transport (DLCI 4) ---- */

/* RX burst from bridge (DL) — store in DSP RAM for firmware to read */
void calypso_trx_rx_burst(const uint8_t *data, int len)
{
    if (!g_trx || len < 8) return;
    CalypsoTRX *s = g_trx;

    uint8_t tn = data[0] & 0x07;
    uint32_t fn = ((uint32_t)data[1]<<24)|((uint32_t)data[2]<<16)|
                  ((uint32_t)data[3]<<8)|(uint32_t)data[4];

    /* Sync FN */
    s->fn = fn % GSM_HYPERFRAME;

    static int rx_count = 0;
    if (++rx_count <= 5 || (rx_count % 1000) == 0)
        TRX_LOG("RX_BURST #%d TN=%d FN=%u len=%d", rx_count, tn, fn, len);

    /* No stubs — bursts go to BSP via UDP (calypso_bsp.c), not here.
     * The DSP processes them and writes results to shared API RAM. */
    (void)tn;
}

/* TX burst: send UL burst from DSP write page via UART TX as sercomm DLCI 4 */
static void calypso_trx_send_ul_burst(CalypsoTRX *s, uint16_t task_u)
{
    if (!g_uart_modem || task_u == 0) return;

    /* Read UL burst from write page.
     * d_burst_u at word 3, burst data follows in NDB a_cu area. */
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];

    /* Build TRXD v0 TX packet: TN(1) FN(4) PWR(1) bits(148) */
    uint8_t pkt[6 + 148];
    uint8_t tn = wp[3] & 0x07;  /* d_burst_u has TN info */
    uint32_t fn = s->fn;

    pkt[0] = tn;
    pkt[1] = (fn >> 24) & 0xFF;
    pkt[2] = (fn >> 16) & 0xFF;
    pkt[3] = (fn >> 8) & 0xFF;
    pkt[4] = fn & 0xFF;
    pkt[5] = 0;  /* TX power */

    /* Read burst bits from NDB UL area — for now send dummy burst */
    memset(&pkt[6], 0, 148);

    /* Wrap in sercomm DLCI 4 and send via UART TX */
    uint8_t frame[512];
    int pos = 0;
    frame[pos++] = 0x7E;  /* FLAG */
    /* Header: DLCI + CTRL, with escaping */
    uint8_t hdr[2] = { 0x04, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == 0x7E || hdr[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = hdr[i] ^ 0x20;
        } else {
            frame[pos++] = hdr[i];
        }
    }
    /* Payload with escaping */
    int pkt_len = 6 + 148;
    for (int i = 0; i < pkt_len && pos < 500; i++) {
        if (pkt[i] == 0x7E || pkt[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = pkt[i] ^ 0x20;
        } else {
            frame[pos++] = pkt[i];
        }
    }
    frame[pos++] = 0x7E;  /* FLAG */

    /* Write to UART chardev (goes to PTY → bridge reads it) */
    qemu_chr_fe_write_all(&g_uart_modem->chr, frame, pos);
}

void calypso_trx_tx_burst_poll(void)
{
    if (!g_trx) return;
    /* Check if firmware wrote a UL task */
    CalypsoTRX *s = g_trx;
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
    uint16_t task_u = wp[DB_W_D_TASK_U];
    if (task_u != 0) {
        calypso_trx_send_ul_burst(s, task_u);
        wp[DB_W_D_TASK_U] = 0;  /* clear after sending */
    }
}

/* Expose DSP state to machine_init for the `-M calypso,dsp-blob=` fixture.
 * Returns NULL if calypso_trx_init() hasn't run or the ROM load failed. */
C54xState *calypso_trx_get_dsp(void)
{
    return g_trx ? g_trx->dsp : NULL;
}

/* DSP hardware reset, driven by the ARM firmware via CNTL_RST (RESET_DSP bit) at
 * 0xfffffd04 — appelé par calypso_cntl_write (soc). Sur silicium le firmware tient
 * le DSP en reset (dsp_pre_boot : assert RESET_DSP) puis le release → la ROM boote
 * FRESH, synchronisée à la timeline firmware. L'ému ignorait ce registre → la ROM
 * tournait en roue libre depuis le realize, désynchro (GAP-0). Ici : assert →
 * running=false (tenir) ; release (transition 1→0) → c54x_reset (boot frais, pose
 * running=true). Gaté revival (SHUNT=0) ; le fake reste en place. */
void calypso_trx_dsp_hw_reset(int assert_reset)
{
    static unsigned hrdbg = 0;
    if (hrdbg < 12) { hrdbg++;
        fprintf(stderr, "[trx] HW-RESET-CALL assert=%d g_trx=%p dsp=%p revival=%d\n",
                assert_reset, (void *)g_trx,
                (void *)(g_trx ? g_trx->dsp : NULL), dsp_real_rom_mode());
    }
    if (!g_trx || !g_trx->dsp || !dsp_real_rom_mode()) return;
    static int in_reset = 0;
    if (assert_reset) {
        if (!in_reset) {
            g_trx->dsp->running = false;
            in_reset = 1;
            TRX_LOG("DSP HW reset ASSERTED (firmware CNTL_RST RESET_DSP=1) — c54x held");
        }
    } else {
        if (in_reset) {
            c54x_reset(g_trx->dsp);   /* fresh boot ; c54x_reset pose running=true */
            in_reset = 0;
            TRX_LOG("DSP HW reset RELEASED — c54x fresh boot, synced to firmware timeline");
        }
    }
}

/* Per-section ROM bin paths, set by mb.c machine_init BEFORE sysbus_realize
 * so that trx_init can load each section into prog[]/data[] **before**
 * c54x_reset() — the reset's PROM→DARAM auto-copy needs prog[] populated. */
static const char *g_section_prom0;
static const char *g_section_prom1;
static const char *g_section_prom2;
static const char *g_section_prom3;
static const char *g_section_drom;
static const char *g_section_pdrom;
static const char *g_section_registers;

void calypso_trx_set_section_paths(const char *prom0, const char *prom1,
                                   const char *prom2, const char *prom3,
                                   const char *drom,  const char *pdrom)
{
    g_section_prom0 = prom0;
    g_section_prom1 = prom1;
    g_section_prom2 = prom2;
    g_section_prom3 = prom3;
    g_section_drom  = drom;
    g_section_pdrom = pdrom;
}

void calypso_trx_set_registers_path(const char *registers)
{
    g_section_registers = registers;
}

/* ---- Init ---- */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s; s->irqs = irqs;
    s->clk_fd = -1;
    TRX_LOG("=== Calypso hardware init ===");

    memory_region_init_io(&s->dsp_iomem,NULL,&calypso_dsp_ops,s,"calypso.dsp_api",CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_DSP_BASE,&s->dsp_iomem);
    /* Pre-fake DSP booted — INCONDITIONNEL (cf read handler : load-bearing pour faire
     * tourner la ROM, le spin firmware affame sinon le tick c54x). */
    s->dsp_ram[DSP_DL_STATUS_ADDR/2]=DSP_DL_STATUS_READY; s->dsp_ram[DSP_API_VER_ADDR/2]=DSP_API_VERSION; s->dsp_booted=true;

    memory_region_init_io(&s->tpu_iomem,NULL,&calypso_tpu_ops,s,"calypso.tpu",CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_BASE,&s->tpu_iomem);
    memory_region_init_io(&s->tpu_ram_iomem,NULL,&calypso_tpu_ram_ops,s,"calypso.tpu_ram",CALYPSO_TPU_RAM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_RAM_BASE,&s->tpu_ram_iomem);
    memory_region_init_io(&s->tsp_iomem,NULL,&calypso_tsp_ops,s,"calypso.tsp",CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TSP_BASE,&s->tsp_iomem);
    memory_region_init_io(&s->ulpd_iomem,NULL,&calypso_ulpd_ops,s,"calypso.ulpd",CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_ULPD_BASE,&s->ulpd_iomem);
    s->sim = calypso_sim_new(s->irqs[CALYPSO_IRQ_SIM]);
    memory_region_init_io(&s->sim_iomem,NULL,&calypso_sim_ops,s,"calypso.sim",CALYPSO_SIM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_SIM_BASE,&s->sim_iomem);

    s->tdma_timer = timer_new_ns(calypso_tdma_clock(), calypso_tdma_tick, s);
    s->dsp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_dsp_done,s);
    s->frame_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_frame_irq_lower,s);

    g_kick_timer = timer_new_ns(QEMU_CLOCK_REALTIME,calypso_kick_cb,NULL);
    timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);

    /* C54x DSP emulator — explicit ROM loading only.
     *
     * Two modes, both opt-in (no implicit/hardcoded ROM path anymore) :
     *   1. Per-section (machine props dsp-prom0/prom1/prom2/prom3/drom/pdrom
     *      set via mb.c before sysbus_realize). Each section is written at
     *      its silicon-correct DSP address, BEFORE c54x_reset so the
     *      PROM→DARAM auto-copy sees the bytes.
     *   2. DARAM blob (-M calypso,dsp-blob=<path>). No ROM is loaded; the
     *      blob in DARAM[0x100..] is the only DSP code. mb.c applies it
     *      after c54x_reset.
     *
     * If neither is set, the DSP runs with empty prog[]/data[]. No more
     * legacy candidate-loop fallback (was: CALYPSO_DSP_ROM env + hardcoded
     * /opt/GSM/calypso_dsp.txt). Use dsp_txt2bin.py to produce per-section
     * .bin files from a legacy .txt dump if needed. */
    {
        s->dsp = c54x_init();
        if (s->dsp) {
            c54x_set_api_ram(s->dsp, s->dsp_ram);
            bool have_sections = g_section_prom0 || g_section_prom1 ||
                                 g_section_prom2 || g_section_prom3 ||
                                 g_section_drom  || g_section_pdrom;
            const char *blob = getenv("CALYPSO_DSP_BLOB");

            /* Blob wins over per-section: when both are set (shouldn't happen
             * if run.sh is used, but defensive), the DARAM blob is the only
             * code source, sections are ignored. The C54x emulator can't
             * sensibly execute both at once. */
            if (blob && *blob) {
                TRX_LOG("DSP ROM mode: dsp-blob (CALYPSO_DSP_BLOB=%s) — "
                        "no ROM loaded, blob in DARAM is the only code", blob);
                if (have_sections) {
                    TRX_LOG("  (per-section paths were also set but are "
                            "ignored — blob takes priority)");
                }
            } else if (have_sections) {
                TRX_LOG("DSP ROM mode: explicit per-section loads");
                if (g_section_prom0) {
                    c54x_load_section(s->dsp, g_section_prom0, 0x07000, true);
                }
                if (g_section_prom1) {
                    /* PROM1 = page 1, chargée en full-address 0x18000+
                     * (atteignable via XPC=1). PAS de mirror low-64K : la
                     * plage 0xE000-0xFFFF = PDROM (vecteurs IT), pas PROM1.
                     * (Fix 2026-05-29 : le mirror clobbait les vecteurs f4eb.) */
                    c54x_load_section(s->dsp, g_section_prom1, 0x18000, true);
                }
                if (g_section_prom2) {
                    c54x_load_section(s->dsp, g_section_prom2, 0x28000, true);
                }
                if (g_section_prom3) {
                    c54x_load_section(s->dsp, g_section_prom3, 0x38000, true);
                }
                if (g_section_drom) {
                    c54x_load_section(s->dsp, g_section_drom, 0x09000, false);
                }
                if (g_section_pdrom) {
                    /* PDROM = program-DATA ROM : visible côté DATA (0xE000+)
                     * ET côté PROGRAMME (page-0 haute 0xE000-0xFFFF) où vit la
                     * table de vecteurs IT (f4eb). Charge les deux espaces. */
                    c54x_load_section(s->dsp, g_section_pdrom, 0x0E000, false);
                    c54x_load_section(s->dsp, g_section_pdrom, 0x0E000, true);
                }
            } else {
                TRX_LOG("DSP ROM mode: NONE — empty prog[]/data[]. "
                        "Use -M calypso,dsp-prom0=.. (et al.) or dsp-blob=..");
            }
            /* Reset + bsp_init: silicon-valid state regardless of ROM mode.
             * machine_init may layer a DARAM blob via the dsp-blob hook
             * after this returns. */
            /* ROMMAP probe (CALYPSO_DEBUG=ROMMAP) : 4 cases qui tranchent
             * le mapping vecteur IT (PDROM vs mirror PROM1) — pré-loader-fix. */
            if (s->dsp && calypso_debug_enabled("ROMMAP"))
                fprintf(stderr,
                    "[c54x] ROMMAP data[0x0ffcc]=0x%04x prog[0x0ffcc]=0x%04x "
                    "prog[0x1ffcc]=0x%04x prog[0x2ffcc]=0x%04x\n",
                    s->dsp->data[0x0ffcc], s->dsp->prog[0x0ffcc],
                    s->dsp->prog[0x1ffcc], s->dsp->prog[0x2ffcc]);
            /* Register snapshot: load into reg_init[] BEFORE reset so
             * c54x_reset() applies it as the authoritative MMR reset state
             * (like the ROM sections above, but for the register file). */
            if (g_section_registers)
                c54x_load_registers(s->dsp, g_section_registers);
            c54x_reset(s->dsp);
            /* running reste false ici : le c54x ne démarre PAS de façon autonome
             * (running=false = dormant). En revival il ne BACC pas (GAP-0), donc le
             * laisser spinner n'achète rien et ajoute un writer sur data[0x0fff]
             * (race avec le fake). Quand GAP-0 tombera, re-dégainer le running=true
             * realive gaté. */
            calypso_bsp_init(s->dsp);
        }
    }

    TRX_LOG("=== Hardware ready ===");

    /* CLK UDP: QEMU sends TDMA ticks to bridge on port 6700.
     * Bridge is clock-slave — no independent timer.
     *
     * Le send est délégué à un pthread dédié (clk_master_thread) pour
     * éviter le jitter ±20ms du QEMU mainloop dispatcher sur le
     * tdma_timer callback. Le pthread utilise clock_nanosleep ABSTIME
     * sur CLOCK_MONOTONIC → précision sub-µs au déclenchement, contre
     * ~ms via QEMU timer. */
    {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd >= 0) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            s->clk_fd = fd;
            memset(&s->clk_peer, 0, sizeof(s->clk_peer));
            s->clk_peer.sin_family = AF_INET;
            s->clk_peer.sin_port = htons(6700);
            s->clk_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            TRX_LOG("CLK UDP → bridge 127.0.0.1:6700");

            /* Le pthread wall clk-master n'est démarré qu'en mode REALTIME.
             * En VIRTUAL (défaut), c'est le tdma_tick qui envoie le CLK (FN
             * virtuel-paced) — pas de pthread wall (sinon double-maître + drift). */
            if (calypso_tdma_clock() == QEMU_CLOCK_REALTIME) {
                calypso_trx_start_clk_master_thread(s);
                TRX_LOG("clk-master wall pthread started (REALTIME mode)");
            } else {
                TRX_LOG("clk-master = tdma_tick virtual-paced (VIRTUAL mode, no wall pthread)");
            }
        }
    }
}
