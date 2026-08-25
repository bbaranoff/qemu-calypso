/*
 * calypso_dsp_legit.c — the "legit" route: hand the posted task to the REAL
 * c54x emulator instead of letting the shunt fake the result.
 *
 * Split out of calypso_dsp_shunt.c. That file is the shunt GLUE: it answers
 * the ARM↔DSP API-RAM contract with data gr-gsm already demodulated, so the
 * c54x never runs. This file is the opposite path — it copies the ARM's task
 * descriptor into DSP DARAM, raises the frame interrupt and actually executes
 * c54x opcodes.
 *
 * Both entry points are gated on CALYPSO_DSP_RUN_C54X=1 at their call sites in
 * calypso_dsp_shunt.c. The default mode (shunt_legit) sets that to 0, so
 * nothing here runs unless the c54x route is explicitly asked for.
 *
 * Pure mechanical split: every statement below was moved VERBATIM from
 * calypso_dsp_shunt.c:1750-1907. Comments were reworded and translated; no
 * logic was changed. Only the two entry points lost `static` so their original
 * callers can still reach them.
 */

#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"      /* calypso_gate() */
#include "calypso_c54x.h"                      /* C54xState, c54x_run/wake/interrupt_ex/bsp_load */
#include "hw/arm/calypso/calypso_dsp_internal.h" /* g_shunt, wp_base(), B_GSM_TASK, NDB_D_DSP_PAGE */
#include <stdint.h>
#include <stdlib.h>

extern int g_c54x_int3_src;  /* INT3 diagnostic source (RO) */

/* d_dsp_page as a c54x DATA WORD address (what api_ram[] is indexed by), built
 * from the NDB byte offset with the same NDB_W() conversion the rest of the
 * shunt uses. NDB_D_DSP_PAGE alone is a byte offset INSIDE the NDB, so it must
 * never be handed to `- C54X_API_BASE` directly. */
#define D_DSP_PAGE_W  ((uint16_t)NDB_W(NDB_D_DSP_PAGE))

/* Read the c54x data[] space DIRECTLY for an ARM-side API address, WITHOUT the
 * calypso_dsp_read MMIO round-trip. That round-trip takes
 * calypso_pcb_daram_lock, a non-recursive mutex — taking it again while
 * already inside the frame-tick context re-locks and aborts. Same mapping as
 * calypso_dsp_read: ARM offset O -> data[O/2 + 0x800]. */
static inline uint16_t shunt_c54x_api_rd(C54xState *dsp, uint32_t arm_addr)
{
    return dsp->data[((arm_addr - 0xFFD00000UL) >> 1) + 0x0800];
}

/* [2026-07-24] CADENCE SPLIT. The old single shunt_route_to_c54x() only ran
 * when g_shunt.pending was set (= the ARM had just posted a NEW d_dsp_page),
 * i.e. roughly once every ~12 real frames. Measured: 708 DSP insn per turn but
 * ~57ms of real time per turn, so the DSP's go-live retry loop got only a
 * fraction of the real time it needs to catch the survival window of
 * data[0x0810]/d_ctrl_system. Native does not have this problem:
 * calypso_tdma_tick() calls c54x_run() unconditionally, gated only on
 * running/idle/shunt_active, never on "is there a fresh task".
 *
 * The fix is to separate the two halves:
 *   - the header (part A) needs a FRESH page_idx/d_fn, so it stays gated on
 *     pending — see the staleness fix from the same day, below;
 *   - the wake+run (part B) does not depend on a fresh dispatch. It replays
 *     the last known I/Q, raises the frame IT and burns the DSP budget —
 *     exactly what native does every frame. So part B became callable on
 *     EVERY frame, pending or not, which restores a cadence close to native
 *     (~217Hz instead of ~17Hz). */
void shunt_route_to_c54x_header(uint8_t page_idx)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;
    fprintf(stderr, "[c54x-route] enter page=%u dsp=%p\n", (unsigned)page_idx, (void*)dsp);

    /* (a) API write-page -> DARAM 0x0586 (mirrors the trx DMA gated at :711).
     * wp_base(page_idx) = absolute MMIO address of the write page (== dsp_ram).
     * The d_dsp_page word (NDB+0 = 0xFFD001A8) is read live (= s->dsp_ram[0x01A8/2]
     * on the trx side) for data[0x0584] and the 0x08E2 mirror. */
    {
        uint32_t wbase    = wp_base(page_idx);
        fprintf(stderr, "[c54x-route] a1 wbase=0x%08x\n", wbase);
        /* [2026-07-24] RACE FIX. The old code read dsp->data[NDB_D_DSP_PAGE]
         * (= data[0x08E2]) RIGHT HERE, mid-frame — a cell that data[0x08E2]
         * toggles between this moment and the FRAME-IT that follows in this
         * SAME invocation. Confirmed at runtime: a2 saw 0x0000 186/186 times
         * while FRAME-IT# saw 0x0003 559/559 times on the SAME cell — not an
         * address bug, a timing/staleness bug. The page_idx parameter is
         * ALREADY the fresh value, captured when the ARM wrote it (see ~line
         * 135: page_idx = (new_d_dsp_page & B_GSM_PAGE) ? 1 : 0). So rebuild
         * dsp_page from that known-fresh value instead of re-reading a cell
         * subject to the race, which avoids propagating stale data into
         * data[0x0584]/api_ram[0x08E2] below. */
        uint16_t dsp_page = B_GSM_TASK | page_idx;
        /* [2026-07-29] The "race" described just above was never a race: a2
         * printed data[0x08E2] (never written) while FRAME-IT printed
         * api_ram[0x08E2] (written by the mirror below) — two distinct arrays,
         * two values, no concurrency involved. And 0x08E2 was not the right
         * cell anyway: d_dsp_page = 0x08D4. We now print the cell the ROM
         * actually reads, in the array it actually reads (api_ram).
         *
         * [2026-08-25] FIXED. NDB_D_DSP_PAGE is a BYTE offset inside the NDB
         * (0x00), not a DSP word address, so `NDB_D_DSP_PAGE - C54X_API_BASE`
         * indexed api_ram[-2048] — 4 KiB BEFORE CalypsoTRX::dsp_ram, an
         * out-of-bounds read here and an out-of-bounds WRITE on the mirror
         * below. NDB_W() is the conversion this file was missing:
         *   NDB_W(NDB_D_DSP_PAGE) == 0x0800 + (0x01A8 >> 1) == 0x08D4,
         * so the api_ram index is 0x08D4 - C54X_API_BASE == 0xD4. */
        fprintf(stderr, "[c54x-route] a2 dsp_page=0x%04x (from page_idx=%u, "
                "api_ram[0x%04x]=0x%04x) api_ram=%p\n",
                dsp_page, (unsigned)page_idx, D_DSP_PAGE_W,
                dsp->api_ram ? dsp->api_ram[D_DSP_PAGE_W - C54X_API_BASE]
                             : dsp->data[D_DSP_PAGE_W],
                (void*)dsp->api_ram);
        dsp->data[0x0584] = dsp_page;
        dsp->data[0x0585] = (uint16_t)(g_shunt.d_fn & 0xFFFF);
        fprintf(stderr, "[c54x-route] a3 data-hdr-ok\n");
        for (int i = 0; i < 20; i++)
            dsp->data[0x0586 + i] = shunt_c54x_api_rd(dsp, wbase + (uint32_t)i * 2);
        fprintf(stderr, "[c54x-route] a4 wp-copy-ok\n");
        /* [2026-07-29] DSP-side d_dsp_page mirror. This used to write 0x08E2 =
         * d_dsp_state: the page overwrote the C_DSP_IDLE3 posted by the ARM
         * (dsp.c:215), and the ROM, which reads 0x08D4 (0xa51c/0xc8ea), saw
         * nothing. Fixed at the source — which satisfied the condition for
         * removing the FIX_DPAGE_OFF crutch (calypso_c54x.c), dropped in the
         * same change.
         *
         * [2026-08-25] The mirror used to index api_ram[NDB_D_DSP_PAGE -
         * C54X_API_BASE] = api_ram[-2048] and corrupted 4 KiB of heap in front
         * of CalypsoTRX::dsp_ram every time the c54x route posted a page. It
         * now goes through D_DSP_PAGE_W (see above). */
        if (dsp->api_ram)
            dsp->api_ram[D_DSP_PAGE_W - C54X_API_BASE] = dsp_page;
    }
    fprintf(stderr, "[c54x-route] a-daram-ok\n");
}

void shunt_route_to_c54x_run(void)
{
    C54xState *dsp = g_shunt.c54x;
    if (!dsp)
        return;

    /* (b) replay the last I/Q burst (cs16, interleaved I,Q) into bsp_buf. */
    if (g_shunt.last_iq_valid && g_shunt.last_iq_n > 0)
        c54x_bsp_load(dsp, (const uint16_t *)g_shunt.last_iq, g_shunt.last_iq_n);

    fprintf(stderr, "[c54x-route] b-bsp-load-ok n=%d\n", g_shunt.last_iq_n);
    /* (c) INT3 FRAME + wake: bring the DSP back if it was idle/halted. */
    g_c54x_int3_src = 3;
    /* [2026-07-22] FRAME_IT_NATIVE: clean tick — deliver the frame scheduler
     * (vec28/bit12) DIRECTLY on the frame tick rather than through the 19/3
     * remap. c54x_irq_level_check picks it up when INTM=0 (natural take, no
     * forcing). That is the real hardware frame-sync primitive. Otherwise
     * (legacy): vec19/bit3 (+ VEC28 remap). */
    {
        /* @CRUTCH — FRAME_IT_NATIVE  (CALYPSO_FRAME_IT_NATIVE, EXISTS; := 1 in
         *           native / native_helped / wire)
         *   hides   : the missing frame-TPU -> DSP vector wiring. We call
         *             c54x_interrupt_ex(dsp,28,12) directly on the frame tick
         *             instead of 19/3 (the vec19 stub is a RETE).
         *   remove  : once the TPU delivers the frame IT on the right vector by
         *             itself (same as VEC28_REMAP on the calypso_c54x.c side).
         */
        static int fin = -1;
        if (fin < 0) fin = calypso_gate("CALYPSO_FRAME_IT_NATIVE", 0);
        if (fin)
            c54x_interrupt_ex(dsp, 28, 12);   /* frame scheduler IT, clean tick */
        else
            c54x_interrupt_ex(dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
        /* [2026-07-23] TINT MASTER CLOCK frame sync: fire TINT on the SAME TDMA
         * tick (not per-2000-insn). Handler 0x72d3 = slot driver op.
         * [2026-08-03] CAL000 §5.1: TINT = bit3/vec19, not bit4/vec20 (which is
         * RINT / SPI receive). Switched under the CALYPSO_IT_TABLE_DOC airlock,
         * see calypso_c54x.c. */
        {
            /* @CRUTCH — TINT0_MASTER (fire on the frame tick)
             *           (CALYPSO_TINT0_MASTER, EXISTS, default OFF outside the WIRE profile)
             *   hides   : the ROM configuring/starting TIMER0. The firmware stops the
             *             timer (TSS=1) in an init path that never runs; we manufacture
             *             TINT at frame cadence.
             *   remove  : once the ROM's TIMER0 init sequence executes (TCR programmed).
             */
            static int _t0m = -1;
            if (_t0m < 0) _t0m = calypso_gate("CALYPSO_TINT0_MASTER", 0);
            if (_t0m) {
                static int _doc = -1;
                if (_doc < 0) _doc = calypso_gate("CALYPSO_IT_TABLE_DOC", 0);
                if (_doc)   /* §5.1: TINT = IMR bit 3 / vec 19 */
                    c54x_interrupt_ex(dsp, C54X_IT_TINT_VEC, C54X_IT_TINT_BIT);
                else        /* legacy SPRU131: actually RINT / SPI receive */
                    c54x_interrupt_ex(dsp, C54X_IT_SPI_RX_VEC, C54X_IT_SPI_RX_BIT);
            }
        }
    }
    c54x_wake(dsp);
    /* revive: the c54x_run loop gate is (running && !idle). c54x_wake only
     * clears idle; on the route_c54x path the trx code that used to set
     * running=true is gated off — so force it here, otherwise the c54x_run
     * loop is skipped entirely (0 insn). */
    dsp->running = true;

    fprintf(stderr, "[c54x-route] c-wake-ok running=%d idle=%d\n", dsp->running, dsp->idle);
    /* (d) burn the budget (one nominal frame ~256000 insns; env-adjustable). */
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
