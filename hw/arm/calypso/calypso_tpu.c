/*
 * calypso_tpu.c -- TPU (Time Processing Unit) sequencer emulation
 *
 * The ARM firmware (osmocom-bb-transceiver include/calypso/tpu.h:
 * tpu_enq_at/tpu_enq_move/tpu_enq_sync/tpu_enq_wait/tpu_enq_offset/
 * tpu_enq_dsp_irq/tpu_enq_sleep, calypso/tsp.c) programs a small
 * micro-instruction scenario into TPU RAM, then commits it by writing
 * TPU_CTRL_EN (dsp_end_scenario(), see calypso_dsp_done() in
 * calypso_trx.c). On real silicon the TPU sequencer walks this program
 * against its own qbit clock (QBITS_PER_TDMA=5000 per tpu.h): AT/WAIT
 * advance or pause the sequencer's position within the current TDMA
 * frame (wrapping into the next frame when the target has already
 * passed), SYNCHRO/OFFSET load bias registers, MOVE writes TSP/DSP
 * peripheral registers.
 *
 * [2026-07-23] Full rewrite (v3) implementing every opcode in
 * osmocom-bb-transceiver's `enum tpu_instr`/`enum tpu_reg_int`, per user
 * direction ("on doit implementer toutes les operations TPU, check
 * osmo" / "sinon ou tu veux qu'il fasse ses operations si le TPU n'est
 * emule nulle part"). v1 only handled MOVE to TSP_CTRL1/CTRL2/TX_1
 * (single byte) and dropped AT/OFFSET/SYNCHRO/WAIT entirely (instant,
 * whole-scenario execution at commit time) -- confirmed by the user's
 * own reading of the INTM-TRANS retry-loop logs: "on n'a implemente
 * nulle part ce truc des onze trames", i.e. l1s_rx_win_ctrl()'s
 * `for(i=0;i<11;i++) tpu_enq_at(0)` (comment: "Delay 11 full TDMA
 * frames") had zero timing effect.
 *
 * v3 : real qbit-position sequencer, replayed across true TDMA frame
 * ticks (calypso_tpu_sequencer_tick(), called once per frame from
 * calypso_tdma_tick()) instead of firing the whole scenario instantly.
 * AT(t)/WAIT(t) advance a per-scenario qbit cursor (0..4999); when the
 * target has already been passed (AT) or the relative advance overflows
 * 5000 (WAIT), that means N real frame boundaries must elapse first --
 * modeled as N frame-tick pauses (our tick granularity is one frame,
 * not one qbit, so sub-frame ordering within a single tick is still
 * coalesced, but cross-frame delays are now genuinely real). Eleven
 * consecutive AT(0) each force exactly one wrap (the cursor is always
 * already at/after 0) = 11 real frame ticks, matching the "11 frames"
 * intent.
 *
 * Full MOVE address map (enum tpu_reg_int): TX_1..TX_4 (multi-byte TSP
 * payload per calypso/tsp.c's tsp_write(), MSB-first, byte count from
 * TSP_CTRL1's low 5 bits), TSP_CTRL1 (dev_idx<<5 | bitlen-1), TSP_CTRL2
 * (WR bit triggers the transfer), TSP_ACT_L/U (tsp_act_update() enable
 * lines, 16-bit state), TSP_SET1-3 (tsp_setup() static clock/CS config),
 * DSP_INT_PG (tpu_enq_dsp_irq(), TX-multislot only -- confirmed via
 * source: only caller is l1s_tx_multi_win_ctrl(), not l1s_rx_win_ctrl()),
 * GAUGING_EN. Only dev_idx==0 (TWL3025, the sole consumer wired up
 * today via calypso_iota_tsp_write) has a downstream hardware model;
 * everything else is correctly parsed/decoded and logged rather than
 * silently dropped, so a gap is visible instead of hidden.
 *
 * One physical sequencer exists on real silicon (TPU RAM = one buffer
 * the firmware overwrites before each re-arm) : a new commit
 * (TPU_CTRL_EN) supersedes whatever was in flight, even mid-wait --
 * faithful to hardware, not a simplification.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_tsp.h"

#define TPU_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[calypso-tpu] " fmt "\n", ##__VA_ARGS__); } while (0)

/* TPU-native MOVE addresses (not TSP device registers -- those are owned
 * by calypso_tsp.h/calypso_tsp_owns_addr()). */
#define TPUI_DSP_INT_PG  0x10   /* tpu_enq_dsp_irq() = tpu_enq_move(this, 1) */
#define TPUI_GAUGING_EN  0x11

/* TPU instruction opcode field (bits 15:13), osmocom's `enum tpu_instr`. */
#define TPU_OP_SLEEP     0
#define TPU_OP_AT        1
#define TPU_OP_OFFSET    2
#define TPU_OP_SYNCHRO   3
#define TPU_OP_MOVE      4
#define TPU_OP_WAIT      5

#define QBITS_PER_TDMA   5000

/* ---- Sequencer state : one physical sequencer, like real silicon ---- */
static struct {
    uint16_t   insns[CALYPSO_TPU_RAM_SIZE / 2];
    int        len;
    int        cursor;
    int        qbit;          /* position within the current TDMA frame */
    int        wait_frames;   /* >0 : paused, resume on tick when it hits 0 */
    bool       active;
    C54xState *dsp;
    uint16_t  *tpu_regs;      /* CalypsoTRX's regs[], for SYNCHRO/OFFSET */
} seq;

static void seq_exec_move(uint8_t addr, uint8_t data, uint32_t fn)
{
    if (calypso_tsp_owns_addr(addr)) {
        calypso_tsp_move(addr, data, fn);
        return;
    }
    switch (addr) {
    case TPUI_DSP_INT_PG:
        if (seq.dsp && (data & 0x01)) {
            /* tpu_enq_dsp_irq(): sequenceur -> DSP directement. Seul
             * appelant reel (tpu_window.c:199) = l1s_tx_multi_win_ctrl()
             * (TX multislot), PAS l1s_rx_win_ctrl() (RX/FB) -- confirme
             * runtime (2026-07-23, 0 hit sur un run FB/SB natif complet).
             * Geree quand meme (vrai trou de modelisation sinon). BRINT0
             * (vec21, IMR bit5) = seul vecteur natif dont le handler ROM
             * est installe (stub RETE) mais jamais pris avant ce fix.
             * Log inconditionnel (pas besoin de CALYPSO_DEBUG) : evenement
             * rare/diagnostique cle. */
            static int dsp_int_pg_log = 0;
            if (++dsp_int_pg_log <= 20)
                fprintf(stderr, "[calypso-tpu] DSP_INT_PG MOVE data=0x%02x fn=%u "
                        "-> BRINT0 (vec21) #%d\n", data, fn, dsp_int_pg_log);
            c54x_interrupt_ex(seq.dsp, 21, 5);
        }
        break;
    case TPUI_GAUGING_EN:
        TPU_LOG("GAUGING_EN <- 0x%02x fn=%u (no consumer)", data, fn);
        break;
    default:
        TPU_LOG("MOVE unknown addr=0x%02x data=0x%02x fn=%u", addr, data, fn);
        break;
    }
}

/* Run from the current cursor until AT/WAIT pauses us again (frame
 * boundary must elapse first), or we reach the end of the scenario (real
 * SLEEP / two padding zeros). */
static void seq_run(uint32_t fn)
{
    while (seq.active && seq.cursor < seq.len) {
        uint16_t insn = seq.insns[seq.cursor];
        if (insn == 0x0000) {
            /* Skip zero words: Rhea bus 32-bit-alignment padding or the
             * final SLEEP (TPU_INSTR_SLEEP is literally encoded as 0x0000).
             * Only stop once we've seen at least one real instruction, and
             * only if the NEXT word is also zero (two consecutive zeros =
             * real SLEEP, not padding) -- heuristic proven necessary by
             * the original single-shot implementation. */
            int next = seq.cursor + 1;
            bool had_any = seq.cursor > 0;
            if (had_any && (next >= seq.len || seq.insns[next] == 0x0000)) {
                seq.active = false;   /* SLEEP: stop the sequencer */
                return;
            }
            seq.cursor++;
            continue;
        }
        uint8_t opcode = (insn >> 13) & 0x7;
        uint16_t payload = insn & 0x1FFF;   /* 13-bit time/data field */

        if (opcode == TPU_OP_AT) {
            /* AT(t): tpu_enq_at() already reduces t via tpu_mod5000(), so
             * payload is the absolute target qbit (0..4999) within a TDMA
             * frame. If we've already passed it this frame, reaching it
             * again means waiting for the NEXT frame's occurrence -- one
             * real frame tick. l1s_rx_win_ctrl()'s `for(11) tpu_enq_at(0)`
             * hits this branch all 11 times (cursor is never < 0). */
            seq.cursor++;
            if (payload > seq.qbit) {
                seq.qbit = payload;
                continue;
            }
            seq.qbit = payload;
            seq.wait_frames = 1;
            return;
        }
        if (opcode == TPU_OP_WAIT) {
            /* WAIT(t): "wait a certain period, in GSM qbits" -- relative
             * advance from the current cursor. */
            seq.cursor++;
            int target = seq.qbit + (int)payload;
            if (target < QBITS_PER_TDMA) {
                seq.qbit = target;
                continue;
            }
            seq.wait_frames = target / QBITS_PER_TDMA;
            seq.qbit = target % QBITS_PER_TDMA;
            return;
        }
        if (opcode == TPU_OP_SYNCHRO || opcode == TPU_OP_OFFSET) {
            /* "Loading delta synchro/offset value in TPU register" --
             * pure register load, no cursor/time effect of its own.
             * TPU_SYNCHRO=0x000E, TPU_OFFSET=0x000C (byte offsets, /2 for
             * the uint16 regs[] array) per calypso_trx.h. */
            if (seq.tpu_regs) {
                if (opcode == TPU_OP_SYNCHRO) seq.tpu_regs[TPU_SYNCHRO / 2] = payload;
                else                          seq.tpu_regs[TPU_OFFSET / 2]  = payload;
            }
            TPU_LOG("%s <- 0x%04x fn=%u", opcode == TPU_OP_SYNCHRO ? "SYNCHRO" : "OFFSET",
                     payload, fn);
            seq.cursor++;
            continue;
        }
        if (opcode == TPU_OP_MOVE) {
            uint8_t addr = insn & 0x1F;
            uint8_t data = (insn >> 5) & 0xFF;
            seq_exec_move(addr, data, fn);
            seq.cursor++;
            continue;
        }
        /* opcode == TPU_OP_SLEEP but insn != 0x0000 can't happen (SLEEP's
         * only bits are the opcode field, always encodes as plain 0x0000,
         * handled above) -- unreachable in practice, skip defensively. */
        seq.cursor++;
    }
    seq.active = false;
}

void calypso_tpu_run_scenario(uint16_t *tpu_ram, C54xState *dsp, uint32_t fn)
{
    calypso_tpu_run_scenario_regs(tpu_ram, dsp, fn, NULL);
}

void calypso_tpu_run_scenario_regs(uint16_t *tpu_ram, C54xState *dsp,
                                    uint32_t fn, uint16_t *tpu_regs)
{
    /* New commit supersedes whatever was in flight -- one physical
     * sequencer, firmware overwrote TPU RAM before re-arming EN, exactly
     * like real hardware would. tsp_act (TSPACT enable lines) is the one
     * piece of state that legitimately survives across commits, matching
     * tsp.c's static tspact_state. */
    memcpy(seq.insns, tpu_ram, sizeof(seq.insns));
    seq.len = CALYPSO_TPU_RAM_SIZE / 2;
    seq.cursor = 0;
    seq.qbit = 0;
    seq.wait_frames = 0;
    seq.dsp = dsp;
    seq.tpu_regs = tpu_regs;
    seq.active = true;
    seq_run(fn);
}

void calypso_tpu_sequencer_tick(uint32_t fn)
{
    if (!seq.active || seq.wait_frames <= 0)
        return;
    seq.wait_frames--;
    if (seq.wait_frames > 0)
        return;
    seq_run(fn);
}
