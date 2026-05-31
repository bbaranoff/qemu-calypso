/*
 * calypso_trx.c — Calypso hardware emulation + DSP C54x emulation
 * No sockets. Firmware speaks UART only. DSP results in shared RAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
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
#include "hw/arm/calypso/calypso_full_pcb.h"  /* api_ram_lock pour MTTCG race fix */
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "hw/arm/calypso/calypso_twl3025.h"
#include "hw/arm/calypso/calypso_sim.h"
#include "hw/arm/calypso/calypso_fbsb.h"
#include "chardev/char-fe.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

extern CalypsoUARTState *g_uart_modem;
extern CalypsoUARTState *g_uart_irda;

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

/* ---- DSP API RAM ---- */
static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;

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
    if (offset >= 0x01F0 && offset <= 0x01FA && size == 2) {
        static int force_toa = -2;  /* -2 = uninit, -1 = off */
        if (force_toa == -2) {
            const char *e = getenv("CALYPSO_FORCE_TOA");
            force_toa = (e && *e) ? (int)strtol(e, NULL, 0) : -1;
            if (force_toa >= 0)
                fprintf(stderr, "[calypso-trx] CALYPSO_FORCE_TOA=%d (bloc FB-result forcé : d_fb_det=1 TOA=%d ANGLE=0 SNR=high)\n", force_toa, force_toa);
        }
        if (force_toa >= 0) {
            switch (offset) {
            case 0x01F0: val = 1;                       break; /* d_fb_det = FOUND */
            case 0x01F4: val = (uint16_t)force_toa;      break; /* a_sync_TOA */
            case 0x01F8: val = 0;                        break; /* a_sync_ANGLE = 0 */
            case 0x01FA: val = 0x7000;                   break; /* a_sync_SNR high */
            default: break;                                     /* 0x01F2/0x01F6 inchangés */
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
        if (arm_rd_a_cd <= 200 || (arm_rd_a_cd % 1000) == 0) {
            unsigned word_idx = (unsigned)((offset - 0x03A0) / 2);
            TRX_LOG("ARM RD a_cd[%u] [arm=0x%04x dsp_word=0x%04x] = 0x%04x sz=%d fn=%u #%u",
                    word_idx, (unsigned)offset, (unsigned)(offset/2 + 0x0800),
                    (unsigned)val, size, s->fn, arm_rd_a_cd);
        }
    }
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;

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
            if (s->dsp && !s->dsp->blob_loaded) {
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
     * dans NDB/read-page et le c54x est inactif (pas de consommateur). */
    if (s->dsp && s->dsp_ram[0x01A8/2] != 0 && !calypso_dsp_shunt_active()) {
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

    fprintf(stderr,
            "[clk-master] pthread armed (CLOCK_MONOTONIC ABSTIME, %lld ns/frame)\n",
            (long long)WALL_TDMA_NS);

    while (g_clk_master_running) {
        next.tv_nsec += WALL_TDMA_NS;
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
        bool periodic_armed = tpu_armed && imr_armed;
        bool force_pulse    = !!(s->tpu_regs[TPU_CTRL/2] & TPU_CTRL_DSP_EN);
        if (periodic_armed || force_pulse) {
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
