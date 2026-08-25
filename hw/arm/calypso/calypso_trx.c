/*
 * calypso_trx.c — Calypso hardware emulation + DSP C54x emulation
 * No sockets. Firmware speaks UART only. DSP results in shared RAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_arm2dsp.h"
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
#include "hw/arm/calypso/calypso_timer.h"   /* calypso_timer_lost_frame_tick() */
#include "hw/arm/calypso/calypso_full_pcb.h"  /* api_ram_lock pour MTTCG race fix */
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_twl3025.h"
#include "hw/arm/calypso/calypso_sim.h"
#include "hw/arm/calypso/calypso_fbsb.h"
#include "calypso_mailbox.h"
#include "calypso_dma.h"
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
#include "calypso_layer1.h"   /* CALYPSO_L1=c : HLE L1 scaffold (FB via corrélation host) */

/* FBSB host-side orchestration. Reintroduced after preNoCell refactor
 * (28 Apr) accidentally removed the wire. The bridge delivers I/Q from
 * a fixed cos/sin LUT (no AFC DAC feedback in QEMU), so the DSP
 * correlator cannot converge across iterations. This wire publishes
 * synthetic clean FB/SB results at the NDB level when ARM dispatches
 * FB_DSP_TASK, allowing the L1→L2→L3 stack to progress toward Location
 * Update without requiring physical RF AFC simulation. */
static CalypsoFbsb g_fbsb;

/* [2026-07-30] CLOBBER-WHO — qui ecrase la page de lecture DSP->ARM ?
 *
 * Gate CALYPSO_CLOBBER_WHO (defaut 1, diagnostic plafonne). Repond a UNE
 * question binaire : les ecritures qui pietinent la sortie du DSP viennent-elles
 * du FIRMWARE (CPU ARM) ou de NOTRE PLOMBERIE (shunt_write_w -> dma_memory_write,
 * qui passe par le meme bus et se deguise donc en « ARM>WR » dans mailbox.log) ?
 *
 * `current_cpu` est non-NULL uniquement quand un CPU execute le store. Un
 * dma_memory_write depuis un thread/timer hote laisse current_cpu a NULL.
 */
static void calypso_clobber_who(uint16_t mot, uint16_t val, uint16_t ancien,
                                uint32_t off, uint32_t fn)
{
    if (mot != 0x0829 && mot != 0x082C && mot != 0x083D && mot != 0x0840) {
        return;
    }
    static int en = -1;
    if (en < 0) {
        en = calypso_gate("CALYPSO_CLOBBER_WHO", 1);
        if (en)
            fprintf(stderr, "[trx] CLOBBER-WHO arme : source des ecritures sur "
                    "0x0829/0x082c/0x083d/0x0840 (CPU ARM = firmware, "
                    "DMA hote = notre plomberie)\n");
    }
    if (!en) {
        return;
    }
    static unsigned long long n_cpu = 0, n_dma = 0;
    static unsigned long long n_boot = 0, n_run = 0;
    static unsigned nlog = 0, nlog_run = 0;
    bool from_cpu = (current_cpu != NULL);
    /* [2026-07-30, v2] BOOT vs REGIME ETABLI. dsp_db_init() (osmocom-bb
     * calypso/dsp.c:437-440) fait QUATRE dsp_api_memset au demarrage, sur les
     * deux pages W et les deux pages R. Ca balaye ces cellules avec des valeurs
     * anciennes non initialisees (0x771a, 0x783f, 0xf47c...) et ecrit des zeros.
     * C'est legitime et ca ne dit RIEN du regime etabli — or les 30 premieres
     * lignes de la v1 etaient exactement ca. On separe. */
    bool boot = (fn < 100);
    if (from_cpu) { n_cpu++; } else { n_dma++; }
    if (boot) { n_boot++; } else { n_run++; }

    /* Les 10 premieres, quel que soit le regime, pour garder la trace du boot. */
    if (nlog < 10) {
        nlog++;
        fprintf(stderr, "[trx] CLOBBER-WHO(boot?) #%u mot=0x%04x 0x%04x -> 0x%04x "
                "off=0x%x fn=%u source=%s\n", nlog, mot, ancien, val, off, fn,
                from_cpu ? "CPU-ARM" : "DMA-HOTE");
    }
    /* Et surtout : les 40 premieres du REGIME ETABLI, celles qui comptent. */
    if (!boot && nlog_run < 40) {
        nlog_run++;
        fprintf(stderr, "[trx] CLOBBER-WHO-RUN #%u mot=0x%04x 0x%04x -> 0x%04x "
                "off=0x%x fn=%u source=%s\n", nlog_run, mot, ancien, val, off, fn,
                from_cpu ? "CPU-ARM" : "DMA-HOTE");
    }
    if (((n_cpu + n_dma) % 200) == 0) {
        fprintf(stderr, "[trx] CLOBBER-WHO resume : cpu_arm=%llu dma_hote=%llu "
                "| boot=%llu regime_etabli=%llu\n",
                (unsigned long long)n_cpu, (unsigned long long)n_dma,
                (unsigned long long)n_boot, (unsigned long long)n_run);
    }
}

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

/* [2026-07-30] Commit direct d'un mot de la fenetre API dans les DEUX banques,
 * sans round-trip MMIO.
 *
 * Existe pour reparer un trou precis : calypso_dsp_shunt.c superpose une region
 * IO de 2 octets sur 0xFFD001A8 (d_dsp_page) en priorite 10, et son handler
 * d'ecriture ne stockait rien -- il croyait a des « pass-through semantics »
 * qui n'existent pas dans QEMU (la plus haute priorite traite l'acces
 * EXCLUSIVEMENT). L'ecriture de dsp_end_scenario() n'atteignait donc jamais
 * calypso_dsp_write() et la cellule gardait son dechet de boot 0xf600.
 *
 * Les DEUX banques sont necessaires :
 *   dsp_ram[]  = ce que l'ARM relit, et la SOURCE du miroir par tick plus bas
 *                dans ce fichier (api_ram[d_dsp_page] = dsp_ram[0x01A8/2]) ;
 *                sans elle, la valeur serait ecrasee au tick suivant ;
 *   dsp->data[]= ce que le DSP lit.
 *
 * Pas de calypso_pcb_daram_lock ici, volontairement : le mutex DARAM n'est pas
 * recursif et cette fonction est appelee depuis un handler MMIO qui peut deja
 * etre dans le contexte frame-tick -> re-lock = abort. Meme raisonnement, et
 * meme precedent, que shunt_c54x_api_rd() dans calypso_dsp_shunt.c. Un store
 * 16 bits aligne ne se dechire pas, et le c54x tourne sur ce meme thread. */
void calypso_trx_api_commit_w(uint32_t arm_offset, uint16_t value)
{
    CalypsoTRX *s = g_trx;
    uint16_t mot, ancien;

    if (!s || (arm_offset + 1u) >= CALYPSO_DSP_SIZE) {
        return;
    }
    mot = (uint16_t)(arm_offset / 2 + 0x0800);
    ancien = s->dsp ? s->dsp->data[mot] : s->dsp_ram[arm_offset / 2];

    /* [2026-07-30] Journaliser le sens ARM>WR, comme le fait calypso_dsp_write().
     * Sans ca le journal MENT par asymetrie : on voit le DSP relire la cellule
     * changer de valeur sans qu'aucune ecriture n'apparaisse jamais — la valeur
     * a l'air de bouger par magie, et les sondes posees sur le chemin MMIO
     * (DDP-ANY, WR-OP, DPAGE_HUNT) restent muettes alors que le commit a bien
     * lieu. Meme contexte que le hook de calypso_dsp_write (ecriture MMIO depuis
     * le CPU), donc meme innocuite. */
    calypso_mbx(MBX_ARM_WR, mot, value, ancien, arm_offset, s->fn,
                s->dsp ? s->dsp->insn_count : 0);
    calypso_clobber_who(mot, (uint16_t)value, ancien, arm_offset, s->fn);

    s->dsp_ram[arm_offset / 2] = value;
    if (s->dsp) {
        s->dsp->data[mot] = value;
    }
}

/* [2026-08-22] AUTO-RECALAGE FN sur la SCH — LE VRAI FIX (remplace la béquille
 * DL_FN_OFFSET codée en dur). Un vrai mobile adopte la FN du BTS portée par le SCH
 * (T1/T2/T3). calypso_trx_autosync_fn() reçoit la FN du SCH décodé et, au 1er SCH,
 * fige offset = sch_fn - trx_fn -> l'horloge émulée se cale à la source, sans valeur
 * codée en dur. Gate CALYPSO_DL_FN_AUTOSYNC (défaut 1). Un CALYPSO_DL_FN_OFFSET
 * manuel désactive l'auto (override explicite, ancien comportement). */
static int64_t g_auto_fn_off = 0;
static int     g_auto_fn_armed = 0;

/* Appelee depuis calypso_dsp_shunt.c (chemin SCH gr-gsm), qui la declare en
 * `extern`. Sans prototype visible ici, -Werror=missing-prototypes casse le
 * build. Meme convention que calypso_dsp_shunt_set_dcch plus haut. */
void calypso_trx_autosync_fn(uint32_t sch_fn);   /* -Werror=missing-prototypes */

void calypso_trx_autosync_fn(uint32_t sch_fn)
{
    if (g_auto_fn_armed || !g_trx) return;
    if (getenv("CALYPSO_DL_FN_OFFSET")) { g_auto_fn_armed = 1; return; } /* override manuel gagne */
    static int gate = -1;
    if (gate < 0) {
        const char *e = getenv("CALYPSO_DL_FN_AUTOSYNC");
        gate = (e && *e == '0') ? 0 : 1;   /* défaut ON */
    }
    if (!gate) { g_auto_fn_armed = 1; return; }
    g_auto_fn_off = (int64_t)sch_fn - (int64_t)g_trx->fn;
    g_auto_fn_armed = 1;
    fprintf(stderr, "[trx] AUTO-SYNC FN sur SCH : sch_fn=%u trx_fn=%u -> offset=%lld "
            "(recalage a la source, remplace la bequille DL_FN_OFFSET)\n",
            sch_fn, (unsigned)g_trx->fn, (long long)g_auto_fn_off);
}

uint32_t calypso_trx_get_fn(void)
{
    if (!g_trx) {
        return 0;
    }
    /* RANK4 recale FN (gate CALYPSO_DL_FN_OFFSET, DEFAUT 0 = inerte -> identique
     * au clean). Offset signe applique a la reference FN de tous les consumers
     * (shunt feed, BSP match, FN-ALIGN) pour caler la FN DSP sur la SCH BTS. A
     * -556, la FCCH tombe dans la bonne trame. NB : trop large (touche aussi la
     * FN UL/DATA_IND) -> a n'activer que pour l'alignement correlateur. */
    /* @BEQUILLE — DL_FN_OFFSET  (CALYPSO_DL_FN_OFFSET, VALEUR, defaut 0 = inerte)
     *   masque  : l'absence de synchronisation de la FN emulee sur la SCH du BTS.
     *             L'offset signe est applique a la SOURCE de FN, donc a tous les
     *             consommateurs (BSP match, feed shunt, FN-ALIGN, et aussi UL /
     *             DATA_IND — porte trop large, assume dans le commentaire d'origine).
     *   retirer : quand la FN est calee sur la SCH recue (recalage a la source),
     *             l'offset mesure tombant a 0 dans FN-PROBE / FN-ALIGN.
     */
    static int off = 0, off_init = 0;
    if (!off_init) {
        off_init = 1;
        const char *e = getenv("CALYPSO_DL_FN_OFFSET");
        if (e && *e) {
            off = atoi(e);
        }
    }
    return (uint32_t)((int64_t)g_trx->fn + off + g_auto_fn_off);
}

/* ---- DSP API RAM ---- */
/* [2026-07-26 camp] Latch per-page du d_burst_d COMMANDE par l'ARM (db_w),
 * pour l'echo per-burst reel (CALYPSO_SHUNT_BURST_ECHO=2). Index = parite de
 * page : [0]=write off 0x0002 (wp p0), [1]=off 0x002A (wp p1). resp(b) (prio -4)
 * lit AVANT cmd(b+2) (prio 0) dans la meme trame -> le latch tient le burst
 * courant -> echo 0,1,2,3 exact, sans jitter, sans OFS. */
uint32_t shunt_l1s_fn(void);   /* decl (calypso_dsp_internal.h) */
/* [2026-07-26 camp] FIFO des burst_id commandes par l'ARM (db_w->d_burst_d) :
 * push sur write WP (calypso_dsp_write), pop sur lecture d_task_d (1x/nb_resp),
 * reset au debut de bloc BCCH (gap shunt_l1s_fn). Distingue burst 0 de burst 2
 * (une latch/parite ne le peut pas) + immunise le double-read (pop 1x/nb_resp,
 * valeur figee s_burst_cur). Deterministe, sans OFS/FN/ECHO. */
static uint8_t  s_bd_ring[8];
static unsigned s_bd_w = 0, s_bd_r = 0;
static uint16_t s_burst_cur = 0;
static uint32_t s_bd_last_wfn = 0xFFFFFFFF;

static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;
    /* [2026-07-29] Moniteur mailbox : ce que l'ARM LIT de la mailbox — le sens
     * qu'aucune sonde ne couvrait, alors que « quel résultat l'ARM voit-il ? »
     * est la moitié de toutes les questions de la journée. */
    if (s->dsp_ram && (offset & 1) == 0) {
        calypso_mbx(MBX_ARM_RD, (uint16_t)(0x0800 + offset / 2),
                    s->dsp_ram[offset / 2], 0, (uint32_t)offset, s->fn,
                    s->dsp ? s->dsp->insn_count : 0);
    }
    {   /* [2026-07-28] FIND32 : voir en-tete du patch. */
        static int _f3 = -1; static unsigned _f3n = 0; static uint16_t _f3v = 0x0020;
        if (_f3 < 0) { _f3 = calypso_gate("CALYPSO_FIND32", 0);
                       const char *v = getenv("CALYPSO_FIND32_VAL");
                       if (v && *v) _f3v = (uint16_t)strtol(v, NULL, 0); }
        if (_f3 && _f3n < 40 && s->dsp_ram && (offset & 1) == 0) {
            uint16_t _v = s->dsp_ram[offset / 2];
            if (_v == _f3v) {
                _f3n++;
                unsigned _dspw = 0x0800 + (unsigned)(offset / 2);
                fprintf(stderr, "[calypso-trx] FIND32 off=0x%04x (mot DSP 0x%04x) = 0x%04x "
                        "| NDB+%d mots | fn=%u\n", (unsigned)offset, _dspw, _v,
                        (int)(((int)offset - 0x01A8) / 2), s->fn);
            }
        }
    }
    {   /* [2026-07-28] ERRREAD : voir en-tete du patch. */
        static int _er = -1; static unsigned _ern = 0;
        if (_er < 0) _er = calypso_gate("CALYPSO_ERRREAD", 0);
        if (_er && offset >= 0x01A8 && offset <= 0x01AE && _ern < 40) {
            _ern++;
            unsigned _w = (unsigned)(offset / 2);
            unsigned _dspw = 0x0800 + _w;
            uint16_t _arm = s->dsp_ram ? s->dsp_ram[_w] : 0xDEAD;
            uint16_t _dsp = (s->dsp && _dspw < C54X_DATA_SIZE) ? s->dsp->data[_dspw] : 0xDEAD;
            fprintf(stderr, "[calypso-trx] ERRREAD off=0x%04x (mot DSP 0x%04x, %s) "
                    "vue_ARM=0x%04x vue_DSP=0x%04x %s fn=%u\n",
                    (unsigned)offset, _dspw,
                    offset == 0x01A8 ? "d_dsp_page" :
                    offset == 0x01AA ? "d_error_status" : "(voisin)",
                    _arm, _dsp,
                    (_arm != _dsp) ? "<<<< LES DEUX VUES DIVERGENT" : "(coherent)", s->fn);
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
    /* [2026-07-22] Injection READ-SIDE REAL_FB/SB : PRECEDE (et court-circuite)
     * le FORCE_TOA canned. Livre la derniere detection FCCH reelle (g_shunt.rx_*)
     * sur le read MMIO ARM -> immunise d_fb_det/a_sync_demod/SB-TOA contre
     * l'ordonnancement intra-trame. Gate CALYPSO_SHUNT_REAL_FB. */
    bool real_fb_hit = false;
    if (size == 2) {
        uint16_t rv;
        if (calypso_dsp_shunt_real_fb_read((uint32_t)offset, &rv)) {
            val = rv;
            real_fb_hit = true;
        }
    }
    /* [2026-07-26 camp] db_r->d_task_d (read page 0 @off 0x50 / page 1 @off 0x78,
     * word 0) : le DSP clear la commande NB -> l1s_nb_resp lit 0 -> puts("EMPTY")
     * et bail avant a_cd. Sous SHUNT_LEGIT + si_valid (a_cd rempli), si le firmware
     * lit d_task_d=0, retourner ALLC_DSP_TASK(24) -> il continue vers a_cd. d_burst_d
     * (off 0x52/0x7A) reste la valeur du firmware (db_r==db_w) -> match burst_id. */
    if (size == 2 && (offset == 0x0050 || offset == 0x0078)) {
        static int _cl = -1;
        if (_cl < 0) { const char *l = getenv("CALYPSO_SHUNT_LEGIT"); const char *nl = getenv("CALYPSO_SHUNT_NO_LEGIT"); _cl = ((l && *l=='1') || (nl && *nl=='1')) ? 1 : 0; }
        /* [2026-07-30] DEUX CHOSES DE NATURES DIFFERENTES, separees.
         *
         * (a) le POP de l'anneau : c'est de la MODELISATION. `d_task_d` est lu une
         *     fois par nb_resp (prim_rx_nb.c:77), donc c'est le bon moment pour
         *     avancer d'un burst. Sans ce pop, `s_burst_cur` reste a 0 et la
         *     lecture sert `(0+3)&3 = 3` — un burst-id CONSTANT, que le firmware
         *     rejette 3 fois sur 4. Mesure du 30/07 : la valeur servie est passee
         *     de 2 (desaliasage eteint) a 3 (desaliasage allume, anneau jamais
         *     depile) — meme symptome, cause deplacee d'un cran. Meme gate que le
         *     push et la lecture : CALYPSO_BURST_ID_DEALIAS, defaut ON.
         *
         * (b) `val = 24` : c'est une BEQUILLE — on fabrique ALLC_DSP_TASK quand le
         *     firmware lit d_task_d=0, pour lui eviter le « EMPTY » et le faire
         *     continuer vers a_cd. Ca reste sous le parapluie, c'est du shunt.
         */
        {
            static int _cbp = -1;
            if (_cbp < 0) _cbp = calypso_gate("CALYPSO_BURST_ID_DEALIAS", 1);
            if (_cbp) {
                s_burst_cur = s_bd_ring[s_bd_r++ & 7u];   /* (a) modelisation */
            }
        }
        if (_cl && calypso_dsp_shunt_si_valid()) {
            if (val == 0) val = 24;   /* (b) BEQUILLE : ALLC_DSP_TASK, evite EMPTY */
        }
    }
    /* [2026-07-26 camp] db_r->d_burst_d (read page @off 0x52 / 0x7A) : le pipeline
     * nb_cmd/nb_resp decale le burst_id commande vs demodule -> "BURST ID x!=y" et
     * le firmware n'atteint jamais burst 3 (ou a_cd est lu). On retourne 3 :
     * nb_resp(3) matche (3==3) et lit a_cd/SI ; nb_resp(0/1/2) bail (mesures
     * non-critiques). Gate SHUNT_LEGIT + si_valid. */
    if (size == 2 && (offset == 0x0052 || offset == 0x007A)) {
        /* [2026-07-30] DECOUPLE. Avant : `_cb && si_valid()`, soit DEUX conditions
         * dont AUCUNE n'a de rapport avec la coherence d'un compteur de burst :
         *   · `_cb` = SHUNT_LEGIT || SHUNT_NO_LEGIT — un parapluie de shunt ;
         *   · `si_valid()` — « gr-gsm a-t-il decode un SI ? ».
         * Or le firmware exige la sequence 0,1,2,3 (prim_rx_nb.c:80 fait un early
         * return sinon, et jette TOUT l'aval : TOA, PM, SNR, AFC, TA, gain,
         * assemblage du bloc de 4). C'est une NECESSITE DE MODELISATION, pas une
         * bequille : elle doit valoir quel que soit qui produit les bursts.
         * Mesure du 30/07 : en `native_twl` avec des SI honnetes (FEED_SI=0),
         * si_valid() est faux -> desaliasage eteint -> d_burst_d fige a 2 (516 cas
         * sur 539) -> « BURST ID 2!=0 / 2!=1 / 2!=3 » -> 3 rapports sur 4 jetes ->
         * le bloc de 4 n'est JAMAIS assemble -> aucun CCCH ne remonte.
         * Gate dediee, defaut ON ; `=0` restaure l'ancien comportement. */
        static int _cb = -1;
        if (_cb < 0) _cb = calypso_gate("CALYPSO_BURST_ID_DEALIAS", 1);
        if (_cb) {
            /* SOURCE UNIQUE : miroir per-page du burst_id commande par l'ARM.
             * db_w->d_burst_d est latche par parite dans calypso_dsp_write :
             *   write 0x0002 -> s_wp_burst_d[0] (read-page 0, off 0x0052)
             *   write 0x002A -> s_wp_burst_d[1] (read-page 1, off 0x007A)
             * resp(b) lit RP(b&1) AVANT que cmd(b+2) (prio 0) ne reecrive la meme
             * parite -> valeur = burst b, FIGEE sur le double-read (line 83+113).
             * Deterministe, sans s->fn, sans OFS, sans compteur. */
            /* r_page = !(burst&1) (verifie runtime : resp(b) lit la read-page de
             * PARITE OPPOSEE au burst) -> lire le latch de parite inverse a l'offset :
             * read 0x0052 (RP0) -> latch[1] ; read 0x007A (RP1) -> latch[0]. */
            /* -1 : le reset FIFO se cale sur le 1er cmd du bloc (souvent burst 1,
             * burst 0=valeur 0), d'ou un offset de phase constant +1 -> on corrige. */
            val = (uint16_t)((s_burst_cur + 3) & 3);   /* FIFO -1 (phase) */
        }
    }
    if (!real_fb_hit && size == 2) {
        /* @BEQUILLE — FORCE_TOA  (CALYPSO_FORCE_TOA, VALEUR, defaut -1/OFF)
         *   masque  : tout le bloc resultat FB/SB (d_fb_det, a_sync_demod TOA/ANGLE/SNR,
         *             a_serv_demod[D_TOA]) : oracle canne cote read MMIO ARM.
         *   retirer : quand d_fb_det natif est ecrit par le DSP.
         *   PIEGE   : "0" ACTIVE le gate (TOA force a 0) ; seul unset/absent coupe.
         *   NB      : deja court-circuitee par SHUNT_REAL_FB/DECAN (garde !real_fb_hit).
         */
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
        /* @BEQUILLE — FORCE_NB  (CALYPSO_FORCE_NB, EQ1, defaut OFF)
         *   masque  : la publication DSP de db_r->d_task_d / d_burst_d. On falsifie le
         *             read ARM (d_task_d 0->1, d_burst_d recopie de db_w) pour passer
         *             le bail "EMPTY" de l1s_nb_resp.
         *   retirer : quand le DSP NB demod ecrit lui-meme la read-page.
         *   NB      : les memes offsets sont deja traites plus haut par le bloc
         *             SHUNT_LEGIT/NO_LEGIT + si_valid — conflit potentiel.
         */
        static int force_nb = -1;
        if (force_nb < 0) {
            const char *e = getenv("CALYPSO_FORCE_NB");
            force_nb = (e && *e == '1') ? 1 : 0;
        }
        if (force_nb) {
            if ((offset == 0x0050 || offset == 0x0078) && val == 0) {
                val = 1;   /* d_task_d → non-zéro : passe le "EMPTY" */
            } else if (offset == 0x0052 || offset == 0x007A) {
                /* d_burst_d ← db_w->d_burst_d (= le burst_id que le firmware a
                 * commandé via dsp_load_rx_task) → passe le check
                 * d_burst_d != burst_id. db_w d_burst_d : p0 DSP word 0x801,
                 * p1 0x815 (db_w p0=0xFFD00000 off0x02, p1=0xFFD00028 off0x2A). */
                if (s->dsp && s->dsp->data)
                    val = s->dsp->data[(offset == 0x0052) ? 0x801 : 0x815];
                else
                    val = s->dsp_ram[(offset == 0x0052) ? 0x01 : 0x15];
            }
        }
    }
    /* DSP boot handshake: firmware polls DL_STATUS until it reads BOOT */
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
        /* [2026-08-03] DEGATE. Cette ligne passait par TRX_LOG, donc par
         * `CALYPSO_DEBUG=TRX` — eteinte dans tous les runs courants. Consequence :
         * le compteur valait 0 au journal QUOI QU'IL ARRIVE, et ce zero pouvait se
         * lire comme « l'ARM ne lit jamais a_cd » alors qu'il ne disait rien du
         * tout. C'est le meme defaut que le compteur mort `fb0_ret` (§14.3) : un
         * temoin qui a l'air d'une mesure et n'est branche sur rien.
         *
         * Or cette sonde est la PATTE COMPLEMENTAIRE du juge `A_CD-WR` (cote DSP,
         * lui sans gate) — le commentaire ci-dessus decrit exactement la lecture
         * croisee des deux. Elle doit donc etre visible par defaut, comme son
         * pendant. Plafonnee (200 premieres + 1/1000), donc sans risque de flot. */
        if (arm_rd_a_cd <= 200 || (arm_rd_a_cd % 1000) == 0) {
            unsigned word_idx = (unsigned)((offset - 0x03A0) / 2);
            fprintf(stderr,
                    "[calypso-trx] ARM RD a_cd[%u] [arm=0x%04x dsp_word=0x%04x] "
                    "= 0x%04x sz=%d fn=%u #%u\n",
                    word_idx, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                    (unsigned)val, size, s->fn, arm_rd_a_cd);
        }
    }
    {   /* [2026-08-03] DTASKD-WATCH patte 2/3 : ce que `l1s_nb_resp()` LIT
         * REELLEMENT (db_r->d_task_d), APRES toute la logique de ce chemin —
         * donc bequille comprise.
         *
         * ⚠ BEQUILLE SUR CE CHEMIN, a connaitre avant d'interpreter : quelques
         * dizaines de lignes plus haut, `if (val == 0) val = 24;` fabrique
         * ALLC_DSP_TASK pour eviter precisement le `EMPTY` de prim_rx_nb.c:74.
         * Elle est conditionnee a (SHUNT_LEGIT || SHUNT_NO_LEGIT) && si_valid().
         * En `native_twl` ces deux gates valent 0 : la bequille est ETEINTE, et
         * c'est pour ca qu'on voit `EMPTY`. Le `EMPTY` n'est donc pas un fait
         * nouveau — c'est une condition connue, habituellement masquee.
         * La sonde imprime la valeur SERVIE : si elle vaut 24 sans que le DSP
         * n'ait rien ecrit (patte 3 vide), c'est la bequille qu'on regarde. */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 2/3 armee (ARM<RD db_r) : "
                        "R p0=off0x0050 R p1=off0x0078\n");
            }
        }
        if (_dw && size == 2 && (offset == 0x0050 || offset == 0x0078)) {
            static unsigned long long _n = 0, _nz = 0;
            _n++;
            if (val) _nz++;
            if (_n <= 40 || (_n % 500) == 0) {
                fprintf(stderr,
                        "[dtaskd] ARM<RD  R p%d  off=0x%04x (mot 0x%04x) -> 0x%04x  "
                        "%s(total=%llu non_nuls=%llu)  fn=%u\n",
                        (offset == 0x0050) ? 0 : 1, (unsigned)offset,
                        (unsigned)(0x0800 + offset / 2), (unsigned)val,
                        val ? "" : "EMPTY-> ", _n, _nz, s->fn);
            }
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

/* [2026-08-03] DTASKD-WATCH — CALYPSO_DTASKD_WATCH=1, defaut 0, LECTURE SEULE.
 *
 * Trois pattes, deux fichiers :
 *   1/3  ARM>WR sur db_w->d_task_d   (ici, calypso_dsp_write)   off 0x0000/0x0028
 *   2/3  ARM<RD sur db_r->d_task_d   (ici, calypso_dsp_read)    off 0x0050/0x0078
 *   3/3  DSP>WR sur db_r->d_task_d   (calypso_c54x.c, data_write_locked)
 *
 * POURQUOI TROIS. Le firmware ecrit la commande RX dans db_w et relit db_r —
 * deux structures DIFFERENTES (dsp_api.h:20-23), pas deux vues d'une meme
 * cellule. La patte 3 est celle qui tranche : si le DSP n'ecrit jamais
 * data[0x0828]/[0x083C], `EMPTY` (prim_rx_nb.c:74) est explique et le probleme
 * est « le DSP n'acquitte pas la tache », pas « l'ecriture ARM se perd ».
 *
 * VERDICT ATTENDU, ecrit d'avance pour ne pas l'ajuster apres coup :
 *   - patte 1 non nulle + patte 3 vide  -> le DSP n'acquitte jamais.
 *   - patte 1 non nulle + patte 3 non nulle + patte 2 lit 0 -> quelqu'un efface
 *     entre l'ecriture DSP et la lecture ARM (chercher l'effaceur, cf. le
 *     precedent « page R ecrasee avant lecture »).
 *   - patte 1 vide -> l1s_nb_cmd n'ecrit pas ce qu'on croit ; tout le reste
 *     de l'analyse du 03/08 est a refaire.
 *
 * SUSPECT ANNEXE (non couvert par cette sonde) : CAL000 §7.2.1, en mode HOM le
 * DSP n'accede plus a la RAM API. Le firmware bascule HOM<->SAM a CHAQUE trame
 * et le modele ignore l'arbitrage — donc dans le modele aucune ecriture n'est
 * perdue de ce fait, mais sur silicium le timing compterait. */
static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;
    {   /* DTASKD-WATCH patte 1/3 : ce que la L1 COMMANDE (db_w->d_task_d). */
        static int _dw = -1;
        if (_dw < 0) {
            _dw = calypso_gate("CALYPSO_DTASKD_WATCH", 0);
            if (_dw) {
                fprintf(stderr, "[dtaskd] patte 1/3 armee (ARM>WR db_w) : "
                        "W p0=off0x0000 W p1=off0x0028\n");
            }
        }
        if (_dw && size == 2 && (offset == 0x0000 || offset == 0x0028)) {
            static unsigned long long _n = 0, _nz = 0;
            _n++;
            if (value) _nz++;
            if (_n <= 40 || (_n % 500) == 0) {
                fprintf(stderr,
                        "[dtaskd] ARM>WR  W p%d  off=0x%04x (mot 0x%04x) <- 0x%04x  "
                        "(total=%llu non_nuls=%llu)  fn=%u\n",
                        (offset == 0x0000) ? 0 : 1, (unsigned)offset,
                        (unsigned)(0x0800 + offset / 2), (unsigned)value,
                        _n, _nz, s->fn);
            }
        }
    }
    {   /* [2026-07-28] BOOTCMD cote ARM : commande bootloader DSP (voir en-tete). */
        static int _bc = -1; static unsigned _bcn = 0;
        if (_bc < 0) _bc = calypso_gate("CALYPSO_BOOTCMD", 0);
        if (_bc && offset >= 0x0FF8 && offset <= 0x0FFF && _bcn < 40) {
            _bcn++;
            const char *_nm = (offset == 0x0FFE) ? "BL_CMD_STATUS (2/4=download)" :
                              (offset == 0x0FFC) ? "BL_ADDR_LO" :
                              (offset == 0x0FFA) ? "BL_SIZE" :
                              (offset == 0x0FF8) ? "BL_ADDR_HI" : "(autre)";
            fprintf(stderr, "[calypso-trx] BOOTCMD ARM off=0x%04x %s <- 0x%04x fn=%u\n",
                    (unsigned)offset, _nm, (unsigned)value, s->fn);
        }
    }
    {   /* [2026-07-28] FBDET-API (b) cote ARM : ecriture MMIO de d_fb_det
         * (mot DSP 0x08F8 -> offset 0x01F0) et du bloc a_sync_demod. */
        static int _fb = -1; static unsigned _fbn = 0;
        if (_fb < 0) _fb = calypso_gate("CALYPSO_FBDET_API", 0);
        if (_fb && offset >= 0x01F0 && offset <= 0x01FB && _fbn < 40) {
            _fbn++;
            fprintf(stderr, "[calypso-trx] FBDET-API ARM off=0x%04x (mot 0x%04x, %s)"
                    " <- 0x%04x size=%u fn=%u\n", (unsigned)offset,
                    (unsigned)(0x0800 + offset / 2),
                    offset == 0x01F0 ? "d_fb_det" : "a_sync_demod",
                    (unsigned)value, size, s->fn);
        }
    }
    /* [2026-07-22] de-alias burst-ID : mirror d_burst_d par commande */
    calypso_dsp_shunt_wp_burst_write((uint32_t)offset, (uint16_t)value);
    /* [2026-07-26 camp] PUSH FIFO du burst_id commande (db_w->d_burst_d, word1 :
     * page0 @0x0002 / page1 @0x002A). Reset au debut d'un bloc BCCH : les cmd0..3
     * sont a frames L1 CONSECUTIVES (shunt_l1s_fn +1) ; gros trou avant cmd0 du
     * bloc suivant -> fn != last+1 => reset FIFO -> alignement 0,1,2,3 sans OFS. */
    if (size == 2 && ((uint32_t)offset == 0x0002 || (uint32_t)offset == 0x002A)) {
        /* [2026-07-30] Meme decouplage cote push : l'anneau doit se remplir des
         * que l'ARM commande un burst, independamment des SI. Sinon le
         * desaliasage lit un anneau vide. */
        {
            uint32_t wfn = shunt_l1s_fn();
            if (wfn != s_bd_last_wfn + 1) { s_bd_w = 0; s_bd_r = 0; }  /* nouveau bloc */
            s_bd_last_wfn = wfn;
            s_bd_ring[s_bd_w++ & 7u] = (uint8_t)(value & 3);
        }
    }

    /* [2026-07-22] WR-RAW (ungated, cap 60) : voir TOUS les writes ARM qui
     * passent par ce hook -> l'ARM commande-t-il le DSP ici, ou tout bypasse ? */
    {
        /* [2026-07-22] cible les writes OPERATIONNELS (fn>100, hors zeroisage boot) :
         * write-page (task_d/md 0x00-0x50) + NDB (d_dsp_page 0x1A8+). Voit-on
         * l'ARM commander le DSP (task + B_GSM_TASK) ? */
        static unsigned wraw = 0;
        /* CIBLE demod-command uniquement (task_d p0=0x00/p1=0x28, task_md p0=0x08/p1=0x30,
         * d_dsp_page 0x1A8), val!=0, TOUTES trames -> l'ARM commande-t-il jamais, et
         * SEED5AC8 change-t-il ca ? Skip AFC/ABB (bruit). */
        /* Elargi : toute la plage NDB (0x01A8-0x0210) non-nulle -> trouver le VRAI
         * offset de d_dsp_page (valeur 0x0002/0x0003 = B_GSM_TASK|w_page). */
        if (value != 0 && ((offset >= 0x01A8 && offset < 0x0210) ||
             offset==0x0000||offset==0x0008||offset==0x0028||offset==0x0030)
            && wraw < 80) {
            wraw++;
            const char *z = (value==0x0002||value==0x0003) ? " <== B_GSM_TASK! (d_dsp_page?)" :
                            (offset == 0x01A8) ? " <== NDB+0" :
                            (offset == 0x0000 || offset == 0x0028) ? " <== task_d" :
                            " <== task_md";
            fprintf(stderr, "[calypso-trx] WR-OP off=0x%04x val=0x%04x size=%u fn=%u%s\n",
                    (unsigned)offset, (unsigned)value, size, s->fn, z);
        }
    }

    /* [2026-07-30] DPAGE-HUNT (CALYPSO_DPAGE_HUNT, defaut 0) — ou atterrit
     * l ecriture ARM de d_dsp_page ?
     *
     * CONSTAT qui motive la sonde : le firmware EXECUTE forcement
     * `dsp_api.ndb->d_dsp_page = B_GSM_TASK | dsp_api.w_page` (dsp.c:471), car
     * la ligne suivante `w_page ^= 1` (dsp.c:472) est le SEUL site de flip du
     * firmware et l ARM ecrit bien task_md alternativement en page 0 (off
     * 0x0008) et page 1 (off 0x0030) — vu par WR-OP et par DSP_WRITE_COUNT.
     * Et pourtant aucune ecriture n arrive jamais a l offset 0x01A8 (= NDB+0 =
     * cellule DSP 0x08D4), ni au moniteur mailbox, ni a WR-OP qui surveille
     * pourtant 0x01A8-0x0210 avec val!=0. Le DSP lit donc UNE fois 0x08D4 et y
     * trouve le dechet de boot 0xf600 (B_GSM_TASK absent) pour tout le run.
     *
     * Deux sondes, deux questions distinctes :
     *   (a) NDB+0 : est-ce que le store passe par CE hook, oui ou non ?
     *       Declencheur = offset == 0x01A8 exactement, TOUTES valeurs (0 compris,
     *       contrairement a WR-OP qui filtre val!=0 et rate donc les remises a 0
     *       de l_1s_reset_hw). Une ligne = un evenement, plafond 40.
     *   (b) BALAYAGE : ou tombe la valeur B_GSM_TASK|w_page (0x0002/0x0003) ?
     *       Declencheur = valeur 2 ou 3, n importe ou dans la fenetre API. La
     *       plage 0x01A8-0x0210 de WR-OP etait un choix arbitraire : si le store
     *       atterrit ailleurs, seul un balayage sans borne le nomme.
     *       Replie PAR OFFSET (un offset nouveau = une ligne, 32 max) pour ne
     *       pas tronquer le journal, plus un bilan tous les 2000 coups (10 max)
     *       afin que l ABSENCE soit mesurable et pas confondue avec un plafond.
     *
     * A LIRE : une ligne (a) => le store arrive ici, le bug est en AVAL (miroir
     * vers data[], ou calypso_trx.c qui repose 0xf600 a chaque tick). Zero ligne
     * (a) mais un offset en (b) => le store part ailleurs, l offset affiche dit
     * ou. Zero ligne des deux => le store ne traverse pas ce hook du tout, il
     * faut regarder les memory_region de la fenetre API. */
    if (calypso_gate("CALYPSO_DPAGE_HUNT", 0)) {
        static unsigned n_a8 = 0;
        if ((uint32_t)offset == 0x01A8 && n_a8 < 40) {
            n_a8++;
            fprintf(stderr, "[calypso-trx] DPAGE-HUNT NDB+0 off=0x01a8 "
                    "val=0x%04x size=%u fn=%u insn=%llu\n",
                    (unsigned)(value & 0xFFFFu), size, s->fn,
                    (unsigned long long)(s->dsp ? s->dsp->insn_count : 0));
        }

        if (size >= 2) {
            static uint32_t vus[32];
            static unsigned n_vus = 0, n_hit = 0, n_bilan = 0;
            int nmots = (size == 4) ? 2 : 1;
            for (int k = 0; k < nmots; k++) {
                uint16_t vv = (uint16_t)(value >> (16 * k));
                uint32_t oo = (uint32_t)offset + 2u * (uint32_t)k;
                unsigned i;
                if (vv != 0x0002 && vv != 0x0003) continue;
                n_hit++;
                for (i = 0; i < n_vus; i++) if (vus[i] == oo) break;
                if (i == n_vus && n_vus < 32) {
                    vus[n_vus++] = oo;
                    fprintf(stderr, "[calypso-trx] DPAGE-HUNT val=0x%04x a un "
                            "offset NOUVEAU 0x%04x (= cellule DSP 0x%04x%s) "
                            "size=%u fn=%u insn=%llu\n",
                            (unsigned)vv, (unsigned)oo,
                            (unsigned)(0x0800u + oo / 2u),
                            (oo == 0x01A8) ? ", NDB+0 = d_dsp_page" : "",
                            size, s->fn,
                            (unsigned long long)(s->dsp ? s->dsp->insn_count : 0));
                }
                if ((n_hit % 2000u) == 0 && n_bilan < 10) {
                    n_bilan++;
                    fprintf(stderr, "[calypso-trx] DPAGE-HUNT bilan : %u coups "
                            "val=2/3, %u offsets distincts, fn=%u\n",
                            n_hit, n_vus, s->fn);
                }
            }
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
        /* [2026-07-29] ARM-WRITE-0810 retirée : une seule cellule, un seul
         * sens. Le moniteur mailbox ci-dessous la couvre et bien davantage —
         *   grep 'd_ctrl_system' mailbox.log
         */
        /* [2026-07-29] Moniteur mailbox — remplace la sonde ARM-API-WR posée
         * plus tôt le même jour, qu'il subsume (voir calypso_mailbox.h). */
        calypso_mbx(MBX_ARM_WR, dsp_word, (uint16_t)value,
                    s->dsp->data[dsp_word], (uint32_t)offset, s->fn,
                    s->dsp ? s->dsp->insn_count : 0);
        calypso_clobber_who(dsp_word, (uint16_t)value,
                            s->dsp->data[dsp_word], (uint32_t)offset, s->fn);

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
    if (offset == 0x01A8) {   /* [2026-07-22] ungated any-size : l'ARM ecrit-il d_dsp_page ? */
        static unsigned ddp_any = 0;
        if (ddp_any++ < 30)
            fprintf(stderr, "[calypso-trx] DDP-ANY WR val=0x%04x size=%u (B_GSM_TASK=%d) fn=%u insn-arm\n",
                    (unsigned)value, size, !!(value & 2), s->fn);
    }
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
            /* [2026-07-26 PORT LU] SHUNT_LEGIT avale d_task_ra -> le poll UL natif
             * ne tire jamais (RACH encode #0). On emet l'access-burst depuis le
             * signal FIABLE = l'ecriture d_rach. 1 write = 1 burst (pas de sticky). */
            {
                /* @BEQUILLE — UL_RACH_FROM_DRACH  (CALYPSO_UL_RACH_FROM_DRACH ; si absente,
                 *              retombe sur CALYPSO_SHUNT_LEGIT ; shunt_no_legit.env:=1)
                 *   masque  : le poll UL natif, qui ne tire jamais l'access-burst parce que
                 *             SHUNT_LEGIT avale d_task_ra. On emet le burst depuis l'ecriture
                 *             ARM de d_rach (1 write = 1 burst, sans sticky).
                 *   retirer : quand d_task_ra atteint le producteur UL sans etre consomme par
                 *             le shunt.
                 *   IDIOME  : "if (e) ulr = (*e=='1'); else ulr = SHUNT_LEGIT" -> poser =0 la
                 *             coupe MEME sous SHUNT_LEGIT=1, contrairement aux INJECT_*.
                 */
                static int ulr = -1;
                if (ulr < 0) {
                    const char *e = getenv("CALYPSO_UL_RACH_FROM_DRACH");
                    if (e) ulr = (*e == '1');
                    else { const char *l = getenv("CALYPSO_SHUNT_LEGIT"); ulr = (l && *l == '1'); }
                }
                if (ulr) calypso_bsp_send_rach_ra(ra, (uint8_t)((value & 0xFF) >> 2), s->fn, 0);
            }
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
            /* CALYPSO_L1=c : latch le d_task_md écrit par l'ARM (le poll tick-time
             * rate ce transient, l1s efface la write-page chaque frame). */
            if (calypso_l1_c_active() && (offset == w0_md || offset == w1_md)) {
                calypso_layer1_on_task_write((uint16_t)value);
            }
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
                calypso_fbsb_init(&g_fbsb, ndb_target, 0x0800,
                                  s->dsp ? s->dsp->api_ram : NULL);
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
            if (s->dsp && calypso_dsp_shunt_early_booted()) {
                /* revive c54x : DSP deja boote+parke a machine-init (early-boot).
                 * NE PAS re-reset : le re-boot re-ecrirait 0xb419 ST #1 = IDLE(1)
                 * PAR-DESSUS la cmd bootloader COPY_BLOCK(2)+entry de l'ARM. En la
                 * preservant, le DSP parke lit 2 -> 0xb424 LDU/BACC -> saute a l'entry. */
                TRX_LOG("C54x DSP reset SKIPPED — early-booted, preserve bootloader cmd");
            } else if (s->dsp && !s->dsp->blob_loaded) {
                c54x_reset(s->dsp);
                s->dsp->running = true;
                s->dsp_init_done = false;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("C54x DSP reset — boot via TDMA ticks");
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

    /* Hardware DMA: copy API write page → DSP DARAM 0x0586.
     * Triggered by firmware writing TPU_CTRL with EN bit (dsp_end_scenario).
     * This is the ONLY place DMA happens — same as real Calypso.
     *
     * GATED par CALYPSO_DSP_SHUNT : si le shunt est actif, on skip
     * complètement cette DMA — le mock écrit les résultats directement
     * dans NDB/read-page et le c54x est inactif (pas de consommateur).
     * HYBRIDE (RANK2, CALYPSO_TPU_RX_WIRE=1) : on lève ce gate pour laisser la
     * commande de tâche ARM (task_md=5 FB) atteindre le DSP DARAM 0x0586 même
     * sous shunt, condition pour que le vrai corrélateur DSP soit dispatché.
     * Réversible : sans l'env, comportement inchangé. */
    /* @BEQUILLE — TPU_RX_WIRE (DMA de tache ARM->DARAM 0x0586)  (CALYPSO_TPU_RX_WIRE,
     *              EXISTS, defaut OFF ; calypso_wire.env:=1)
     *   masque  : sous shunt la DMA page-ecriture ARM->DSP est fermee, donc la
     *             commande de tache (task_md=5 FB) n'atteint jamais le DSP et le
     *             correlateur entre sans mission. Le meme gate pose plus bas le bit
     *             tache FB d[0x3f92]|=0x0800 a la place de l'ORM natif 0xa539.
     *   retirer : quand le shunt ne substitue plus le DSP (la DMA redevient legitime)
     *             et que 0xa539 s'execute reellement.
     */
    static int trx_rxw = -1;
    if (trx_rxw < 0) trx_rxw = calypso_gate("CALYPSO_TPU_RX_WIRE", 0);
    if (s->dsp && s->dsp_ram[0x01A8/2] != 0 &&
        (!calypso_dsp_shunt_active() || trx_rxw)) {
        uint16_t page = s->dsp_ram[0x01A8/2] & 1;
        uint16_t *wp = page ?
            &s->dsp_ram[DSP_API_W_PAGE1/2] : &s->dsp_ram[DSP_API_W_PAGE0/2];

        /* Log proof that ARM wrote tasks before DMA */
        uint16_t task_d  = wp[DB_W_D_TASK_D];
        uint16_t task_u  = wp[DB_W_D_TASK_U];
        uint16_t task_md = wp[DB_W_D_TASK_MD];
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
        /* WIRE d[0x3f92] (RANK2, CALYPSO_TPU_RX_WIRE) : quand l'ARM commande la
         * tâche FB (task_md=5), poser le bit tâche FB dans le task-word du
         * scheduler DSP d[0x3f92]|=0x0800. Le setter natif (ORM 0xa539) est skippé
         * car d[5a00]==0x88 -> sans ça d[3f92] reste 0 à vie. Fires à chaque DMA
         * de commande FB (task_md=5), indépendant de BDLENA. */
        if (trx_rxw && task_md == 5)
            s->dsp->data[0x3f92] |= 0x0800;
        qemu_mutex_unlock(&calypso_pcb_api_ram_lock);
        calypso_pcb_daram_lock_release();
    }

    /* TPU sequencer scenario interpretation lives in calypso_tpu.c (full
     * opcode set: AT/WAIT/SYNCHRO/OFFSET/MOVE/SLEEP, replayed across real
     * TDMA frame ticks -- see calypso_tpu_sequencer_tick() below). */
    calypso_tpu_run_scenario_regs(s->tpu_ram, s->dsp, s->fn, s->tpu_regs);

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
        if (++tpu_log <= 50)
            TRX_LOG("TPU_CTRL WR val=0x%04x (EN=%d DSP_EN=%d) fn=%u",
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
        if (++ictrl_log <= 30)
            TRX_LOG("INT_CTRL WR val=0x%02x (MCU_FRAME=%d DSP_FRAME=%d DSP_FORCE=%d) fn=%u",
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
    /* [2026-07-29] Un tick DMA par trame TDMA. C'est le signal de complétion
     * qui manquait : sans lui le firmware DSP empile ses requêtes dans sa file
     * de 14 entrées, personne ne dépile, l'anneau sature et il lève
     * DSP_ERR_DMA_PROG. Inerte tant que CALYPSO_DMA n'est pas posé. */
    {
        CalypsoTRX *_s_dma = opaque;
        if (_s_dma && _s_dma->dsp) {
            calypso_dma_tick(_s_dma->dsp);
        }
    }
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

    /* TPU sequencer: advance any AT/WAIT-paused scenario by one real TDMA
     * frame (the 11x tpu_enq_at(0) FB-window delay is now genuinely
     * spread across 11 ticks instead of firing instantly). */
    calypso_tpu_sequencer_tick(s->fn);

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
    if (s->dsp && s->dsp->running && !s->dsp_init_done && !calypso_dsp_shunt_substitutes()) {
        if (!s->dsp->idle)
            dsp_n_exec_2 = c54x_run(s->dsp, dsp_budget);
        if (s->dsp->idle) {
            s->dsp_init_done = true;
            TRX_LOG("DSP init complete (first IDLE reached)");
        }
    } else if (calypso_dsp_shunt_substitutes() && !s->dsp_init_done) {
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
        static int _natfr = -1; if (_natfr < 0) _natfr = (getenv("CALYPSO_FRAME_IT_NATIVE") || getenv("CALYPSO_DSP_FRAME_VEC28")) ? 1 : 0;
        bool imr_armed = !!(s->dsp->imr & (1 << (_natfr ? 12 : C54X_INT_FRAME_BIT)));  /* [2026-07-23] bit12 en natif (remap) */
        bool periodic_armed = tpu_armed && imr_armed;
        bool force_pulse    = !!(s->tpu_regs[TPU_CTRL/2] & TPU_CTRL_DSP_EN);
        /* FIX DOUBLE-INT3 : quand la route c54x du shunt est active, c'est
         * shunt_route_to_c54x() qui fire l'INT3 frame. Ne PAS le double-firer ici,
         * sinon le c54x reçoit 2 IT frame/tick -> déraille -> crash qemu. */
        if ((periodic_armed || force_pulse) && !calypso_dsp_shunt_route_c54x_active()) {
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
        if (!s->dsp->idle && !calypso_dsp_shunt_substitutes()) {
            dsp_n_exec_5 = c54x_run(s->dsp, dsp_budget);
        }

        /* CALYPSO_L1=c : pilote le modèle L1 HLE APRÈS le c54x RX (qui ne produit
         * rien d'exploitable) -> d_fb_det + a_sync_demod sont les dernières écritures
         * de la frame. Lit l'I/Q injectée en DARAM 0x2a00 et corrèle le FCCH. */
        /* Ne PAS piloter le modèle L1=c quand le shunt est actif : il écrirait
         * d_fb_det=0 par-dessus le d_fb_det=1 du shunt (clobber -> FB perdu).
         * Le shunt possède alors la réception (FB+SB+SI). Le modèle L1=c ne tourne
         * que sans shunt (chemin HLE pur). */
        if (calypso_l1_c_active() && !calypso_dsp_shunt_active()) {
            calypso_layer1_tick(s->dsp, s->dsp_ram, s->fn);
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
    /* Fige timer #1 sur la grille de trame pour cette IRQ délivrée -> le firmware
     * check_lost_frame() voit un pas de 1875 exact (fin du spam LOST). */
    calypso_timer_lost_frame_tick(s->fn);
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
