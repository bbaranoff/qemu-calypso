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
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
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

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

/* CALYPSO_TIMER=1 enables timer-side fprintf tracing (frame_irq, tdma_tick,
 * kick). =0 (default) drops the calls entirely so the run is silent and
 * stderr-pipe backpressure (TSLOG → python flush-per-line) can't throttle
 * the TCG main thread. Cached once via getenv. */
static bool calypso_timer_log(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("CALYPSO_TIMER");
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
    uint8_t      sync_bsic;

    /* C54x DSP emulator */
    C54xState   *dsp;
    bool         dsp_init_done;  /* DSP reached first IDLE after boot */

    /* CLK UDP: send each TDMA tick to bridge so it's clock-slave */
    int          clk_fd;
    struct sockaddr_in clk_peer;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* FBSB host-side orchestration. Reintroduced after preNoCell refactor
 * (28 Apr) accidentally removed the wire. The bridge delivers I/Q from
 * a fixed cos/sin LUT (no AFC DAC feedback in QEMU), so the DSP
 * correlator cannot converge across iterations. This wire publishes
 * synthetic clean FB/SB results at the NDB level when ARM dispatches
 * FB_DSP_TASK, allowing the L1→L2→L3 stack to progress toward Location
 * Update without requiring physical RF AFC simulation. */
static CalypsoFbsb g_fbsb;
static bool        g_fbsb_inited;

/* W1C latches for FB-detection result snapshot.
 * Set by c54x data_write when DSP writes a_sync_SNR (LAST cell of
 * fb-det iteration sequence) from a real fb-det PC. Snapshot captures
 * d_fb_det/d_fb_mode/a_sync_demod[*] coherent for the iteration.
 * Consumed by ARM read; survives DSP-side clears and stack-stomp at
 * PC=0x0662. Order in DSP firmware:
 *   d_fb_det → d_fb_mode → a_sync_TOA → PM → ANG → SNR (insn N..N+150)
 * Snapshot at SNR ensures all values are this-iter values. */
uint16_t g_d_fb_det_latch;
uint16_t g_d_fb_mode_latch;
uint16_t g_a_sync_TOA_latch;
uint16_t g_a_sync_PM_latch;
uint16_t g_a_sync_ANG_latch;
uint16_t g_a_sync_SNR_latch;
bool     g_a_sync_valid;

/* CALYPSO_W1C_LATCH=1 enables the W1C latch system : DSP writes at the
 * a_sync_demod iteration end (PCs 0x8d33/0x8eb9/0x8f51) snapshot all 6
 * cells, ARM reads consume them. Mitigates a race window where DSP sets
 * d_fb_mode then clears within a tight loop, and ARM polls between.
 * Default 0 = ARM reads NDB directly. Read once at first call, cached. */
int calypso_w1c_latch_enabled(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *e = getenv("CALYPSO_W1C_LATCH");
        cached = (e && *e == '1') ? 1 : 0;
        fprintf(stderr,
                "[calypso-trx] CALYPSO_W1C_LATCH=%d (%s)\n",
                cached, cached ? "latch on a_sync_SNR snapshot, consume on ARM read"
                               : "ARM reads NDB direct");
    }
    return cached;
}

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
    uint16_t *src = (s->dsp && s->dsp->data)
                    ? &s->dsp->data[offset/2 + 0x0800]
                    : &s->dsp_ram[offset/2];
    uint64_t val = (size == 2) ? src[0] :
                   (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
                   ((uint8_t *)src)[offset & 1];
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
    /* W1C latch consume — snapshot at fb-det iteration end (a_sync_SNR
     * write by real fb-det PC).
     * d_fb_det read consumes the latch (ARM acks detection); a_sync_*
     * remain valid for the subsequent burst-read until next snapshot
     * overwrites them. */
    if (calypso_w1c_latch_enabled() &&
        offset == 0x01F0 && size == 2 && g_a_sync_valid &&
        g_d_fb_det_latch != 0) {
        uint16_t v = g_d_fb_det_latch;
        g_d_fb_det_latch = 0;
        TRX_LOG("ARM RD d_fb_det LATCH-CONSUME = 0x%04x (cleared) fn=%u",
                v, s->fn);
        return v;
    }
    if (calypso_w1c_latch_enabled() && g_a_sync_valid && size == 2) {
        uint16_t v = 0;
        const char *name = NULL;
        switch (offset) {
        case 0x01F2: v = g_d_fb_mode_latch;  name = "d_fb_mode";  break;
        case 0x01F4: v = g_a_sync_TOA_latch; name = "a_sync_TOA"; break;
        case 0x01F6: v = g_a_sync_PM_latch;  name = "a_sync_PM";  break;
        case 0x01F8: v = g_a_sync_ANG_latch; name = "a_sync_ANG"; break;
        case 0x01FA: v = g_a_sync_SNR_latch; name = "a_sync_SNR"; break;
        }
        if (name) {
            TRX_LOG("ARM RD %s LATCH = 0x%04x (s=%d) fn=%u",
                    name, v, (int)(int16_t)v, s->fn);
            return v;
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
        if (size == 2) {
            s->dsp->data[dsp_word] = (uint16_t)value;
        } else if (size == 4) {
            s->dsp->data[dsp_word]     = (uint16_t)value;
            s->dsp->data[dsp_word + 1] = (uint16_t)(value >> 16);
        }
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

            /* FBSB orchestration hook: ARM has just written d_task_md.
             * Initialise on first call, then dispatch to the host-side
             * state machine which publishes synthetic FB/SB results
             * into NDB so ARM can progress past l1s_fbdet_resp. */
            if (!g_fbsb_inited) {
                /* 2026-05-15 fix : pointer le synth sur s->dsp->data[] qui
                 * est désormais la source de vérité pour ARM reads (cf fix
                 * DSP→ARM mirror dans calypso_dsp_read). Sinon le synth écrit
                 * dans s->dsp_ram[] (ARM-side legacy array) qu'ARM ne lit
                 * plus → publish_fb_found/sb_found invisibles à firmware. */
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
            /* Reset C54x DSP — boot runs in TDMA ticks (parallel with ARM) */
            if (s->dsp) {
                c54x_reset(s->dsp);
                s->dsp->running = true;
                s->dsp_init_done = false;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("C54x DSP reset — boot via TDMA ticks");
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
     * This is the ONLY place DMA happens — same as real Calypso. */
    if (s->dsp && s->dsp_ram[0x01A8/2] != 0) {
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

        s->dsp->data[0x0584] = s->dsp_ram[0x01A8/2];
        s->dsp->data[0x0585] = s->fn & 0xFFFF;
        for (int i = 0; i < 20; i++)
            s->dsp->data[0x0586 + i] = wp[i];
        if (s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];
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
static void calypso_tpu_ram_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TPU_RAM_SIZE/2)s->tpu_ram[off/2]=v;}
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
}

static void calypso_tdma_tick(void *opaque) {
    CalypsoTRX *s = opaque;
    int64_t entry_t = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t t_clk = 0, t_uart = 0, t_dspboot = 0, t_dspirq = 0,
            t_bsp = 0, t_ul = 0;
    s->fn = (s->fn+1) % GSM_HYPERFRAME;

    /* TDMA tick counter — log thinned 1/1000 (~4.6s wall) pour drift detection.
     * Variables locales pour cumul DSP insn (utilisées plus bas). */
    static uint64_t tdma_ticks = 0;
    static uint64_t dsp_insn_total = 0;
    tdma_ticks++;
    int dsp_n_exec_2 = 0, dsp_n_exec_5 = 0; /* updated by c54x_run calls */

    /* ── 0. Send CLK tick to bridge (QEMU is clock master) ── */
    if (s->clk_fd >= 0) {
        uint8_t pkt[4];
        pkt[0] = (s->fn >> 24) & 0xFF;
        pkt[1] = (s->fn >> 16) & 0xFF;
        pkt[2] = (s->fn >>  8) & 0xFF;
        pkt[3] =  s->fn        & 0xFF;
        sendto(s->clk_fd, pkt, 4, 0,
               (struct sockaddr *)&s->clk_peer, sizeof(s->clk_peer));
    }
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
    if (s->dsp && s->dsp->running && !s->dsp_init_done) {
        if (!s->dsp->idle)
            dsp_n_exec_2 = c54x_run(s->dsp, dsp_budget);
        if (s->dsp->idle) {
            s->dsp_init_done = true;
            TRX_LOG("DSP init complete (first IDLE reached)");
        }
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
         * compute RX critique (Claude web review 2026-05-16). */
        if (!s->dsp->idle) {
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
        int64_t target = entry_t + GSM_TDMA_NS;
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int skipped = 0;
        while (target <= now) {
            target += GSM_TDMA_NS;
            skipped++;
        }

        {
            static int rearm_log_count = 0;
            if (rearm_log_count < 50) {
                fprintf(stderr, "[rearm-fix] entry_t=%" PRId64 " target=%" PRId64
                        " now=%" PRId64 " gap_to_now=%" PRId64 " skipped=%d\n",
                        entry_t, target, now, target - now, skipped);
                rearm_log_count++;
            }
        }

        if (skipped > 0 && (s->fn % 100 == 0) && calypso_timer_log()) {
            fprintf(stderr, "[tdma-skip] fn=%u skipped=%d work_dt=%" PRId64 "\n",
                    s->fn, skipped, now - entry_t);
        }

        if (s->tdma_running)
            timer_mod_ns(s->tdma_timer, target);
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
    timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
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

    /* === rxDoneFlag periodic refresh (5 ms wall TDMA kick) ===
     *
     * Complément au write SIM-driven précis dans calypso_sim.c (sur IT_WT).
     * Ce write périodique 200 Hz a deux fonctions distinctes :
     *
     *   (1) Couvre les busy-loops firmware qui se forment AVANT l'IT_WT
     *       SIM (e.g., entre clear rxDoneFlag=0 et le premier IT_WT) —
     *       garantit que le polling firmware finit toujours par voir 1.
     *
     *   (2) Side-effect cpu_physical_memory_write : sous icount=auto, le
     *       firmware busy-loop accumule des insns sans yield ; le write
     *       physique invalide TB/load caches, forçant ré-lecture fraîche.
     *       cpu_exit SEUL est insuffisant (interrompt la TB mais ne flush
     *       pas les caches).
     *
     * Par défaut ON. Désactiver via CALYPSO_FORCE_RX_DONE=0 pour test.
     * Adresse rxDoneFlag = 0x008302d4 (nm 2026-05-24), override via
     * CALYPSO_RXDONE_ADDR. Doit rester aligné avec calypso_sim.c. */
    {
        static int disabled = -1;
        if (disabled < 0) {
            const char *env = getenv("CALYPSO_FORCE_RX_DONE");
            disabled = (env && env[0] == '0') ? 1 : 0;
        }
        if (!disabled) {
            static uint32_t rxdone_addr;
            if (!rxdone_addr) {
                const char *aenv = getenv("CALYPSO_RXDONE_ADDR");
                rxdone_addr = aenv ? (uint32_t)strtoul(aenv, NULL, 0)
                                   : 0x008302d4;
            }
            const uint32_t one = 1;
            cpu_physical_memory_write(rxdone_addr, &one, sizeof(one));
        }
    }

    CPUState*cpu=first_cpu;if(cpu)cpu_exit(cpu);qemu_notify_event();
    timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);
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

/* ---- Init ---- */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s; s->irqs = irqs; s->sync_bsic = 7;
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

    s->tdma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_tdma_tick,s);
    s->dsp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_dsp_done,s);
    s->frame_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_frame_irq_lower,s);

    g_kick_timer = timer_new_ns(QEMU_CLOCK_REALTIME,calypso_kick_cb,NULL);
    timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);

    /* C54x DSP emulator */
    {
        const char *rom_path = getenv("CALYPSO_DSP_ROM");
        if (!rom_path) rom_path = "/opt/GSM/calypso_dsp.txt";
        s->dsp = c54x_init();
        if (s->dsp) {
            c54x_set_api_ram(s->dsp, s->dsp_ram);
            if (c54x_load_rom(s->dsp, rom_path) == 0) {
                c54x_reset(s->dsp);
                calypso_bsp_init(s->dsp);
                TRX_LOG("C54x DSP loaded from %s", rom_path);
            } else {
                TRX_LOG("C54x DSP ROM not found at %s", rom_path);
                free(s->dsp);
                s->dsp = NULL;
            }
        }
    }

    TRX_LOG("=== Hardware ready ===");

    /* CLK UDP: QEMU sends TDMA ticks to bridge on port 6700.
     * Bridge is clock-slave — no independent timer. */
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
        }
    }
}
