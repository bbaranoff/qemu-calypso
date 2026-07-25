/*
 * calypso_arm2dsp — ARM->DSP task-post bridge (faithful data wire).
 *
 * On real Calypso the ARM (osmocom L1) orchestrates the C54x DSP through the
 * shared API RAM: it commands the FB/FCCH search by writing d_dsp_page bit1
 * (B_GSM_TASK). The DSP runs a software task dispatcher (ROM 0xb41c) that polls
 * the task-ready word data[0x0fff]: when bit1 (0x0002) is set, it dequeues and
 * runs the posted task (0xb42a) via its own ROM code.
 *
 * Earlier this module FORCED go-live by redirecting the DSP PC into a setter
 * and poking IMR/INTM/SP. That approach was proven a dead-end (2026-07-02
 * STATUS addendum 8): forcing the frame IT out of a clean idle context pushes a
 * bogus return PC and self-loops PC=0x0000. It is removed.
 *
 * The faithful wire only posts DATA the DSP already polls: on the ARM
 * B_GSM_TASK write we set the DSP task-ready flag (data[0x0fff] bit1, mirrored
 * into api_ram). No PC redirect, no IMR/INTM/SP poke, no vector poke — the DSP
 * dispatches and goes live entirely through its own ROM.
 *
 * Env (no rebuild):
 *   CALYPSO_ARM2DSP=1               enable (off by default)
 *   CALYPSO_ARM2DSP_TASKWORD=0xfff  DSP task-ready word (default 0x0fff)
 *   CALYPSO_ARM2DSP_TASKBIT=0x2     bit to post (default 0x0002 = dispatcher bit1)
 *
 * Fix A (2026-07-24) — faithful ARM background-enable handshake:
 *   CALYPSO_ARM2DSP_BGEN=1          post d_background_enable/state cells (off by default)
 *   CALYPSO_ARM2DSP_BGEN_A=0x098a   d_background_enable word
 *   CALYPSO_ARM2DSP_BGEN_C=0x098c   d_background_state  word
 *   CALYPSO_ARM2DSP_BGEN_VAL=0x1    enable value posted into both cells
 *   CALYPSO_ARM2DSP_BGEN_POLLPC=0xdddb  DSP PC of the phase-SM cell poll
 *   CALYPSO_ARM2DSP_BGEN_ONESHOT=1  post exactly once (single go-live transition)
 */
#include "qemu/osdep.h"
#include "calypso_arm2dsp.h"

#include <stdlib.h>
#include <stdio.h>

/* ARM byte offset of d_dsp_page (DSP word 0x08E2). B_GSM_TASK = bit1. */
#define A2D_DSP_PAGE_OFF   0x01A8
#define A2D_B_GSM_TASK     0x0002

/* DSP API region base: words >= 0x0800 are read by the DSP from api_ram. */
#define A2D_API_BASE       0x0800

static int      a2d_on = -1;      /* -1 = unresolved, 0/1 = disabled/enabled     */
static uint16_t a2d_word;         /* DSP task-ready word (default 0x0fff)         */
static uint16_t a2d_bit;          /* task-ready bit to post (default 0x0002)      */

static volatile int a2d_pending;  /* ARM posted B_GSM_TASK, awaiting DSP step     */
static unsigned     a2d_posts;    /* how many task-posts applied                  */
static int          a2d_cont = -1; /* continuous post while d_dsp_page bit1 set     */

/* ---- Fix A (2026-07-24): faithful ARM background-enable handshake ------------
 * VERIFIED on the live full-grgsm run (PHASE-SM trace, not the code comments):
 *   PHASE-SM pc=0xdddb d[3f70]=0000 d[098a]=0000 d[098c]=0000 d[0fff]=0002
 *   PHASE-SM pc=0xddeb d[3f70]=0001 d[098a]=0000 ...
 *   PHASE-SM pc=0xde8b A=0x0000    d[3f70]=0001 d[098a]=0000 ...   <- reset branch
 * The DSP go-live phase-SM (0xdddb -> 0xddeb -> 0xde8b) polls d[0x098a]
 * (d_background_enable) / d[0x098c] (d_background_state). While the ARM leaves
 * them 0 the SM takes the reset branch and never reaches the GO setter 0xde9c
 * (ST #2) that would raise d[0x3f70] bit1 — the flag the go-live wait-loop test
 * at 0xa4d4 reads to leave the loop. Consequence (all measured on the live log):
 * 0xde9c = 0 hits, F70-SETBIT1 = 0, the wait-loop 0xa4ca/0xa4d0 spins 300k+
 * times toggling INTM ("blocs de 10 en 10"), and the FB correlator [0x8d00..
 * 0x9000] is never entered (0 hits). The ARM never writes near 0x098a in the
 * live run. On real Calypso the ARM posts these cells as part of the go-live
 * handshake; here we post them — DATA the DSP already polls, no PC / IMR / INTM
 * / SP poke — exactly ONCE, gated on the ARM having commanded the task (the
 * dispatcher bit d[0x0fff] & 0x0002 is set). The DSP then reaches 0xde9c, sets
 * d[0x3f70] bit1 itself and leaves the wait-loop natively: a SINGLE go-live
 * transition instead of the per-frame re-fire. Off by default (reversible). */
static int          a2d_bgen = -1;       /* -1 unresolved, 0/1 disabled/enabled  */
static uint16_t     a2d_bgen_a;          /* d_background_enable word (0x098a)     */
static uint16_t     a2d_bgen_c;          /* d_background_state  word (0x098c)     */
static uint16_t     a2d_bgen_val;        /* enable value to post     (0x0001)     */
static uint16_t     a2d_bgen_pollpc;     /* phase-SM poll PC         (0xdddb)     */
static int          a2d_bgen_oneshot = -1; /* 1 = one transition only (default)   */
static int          a2d_bgen_done;       /* one-shot latch                        */
static unsigned     a2d_bgen_posts;      /* how many background-enable posts       */

/* ---- CTRLSYS wire (RANK1): ARM->DSP d_ctrl_system 0x0810 bit15 ----------------
 * The go-live gate at DSP 0xa53c is BITF *(AR1+0x10),0x8000 with AR1=0x0800, i.e.
 * it tests data[0x0810] (d_ctrl_system, dsp_api.h "Control Register RESET/RESUME").
 * bit15 SET  -> a541..a544..0x09bc..0xd247 = bootstrap/operational path (reaches
 *               the FB dispatch we need).
 * bit15 CLEAR-> BC 0xa575 = short-circuit, never bootstrap (current: data[0x0810]=0
 *               -> DSP loops the go-live init -> double IMR, correlator never set up).
 * The real ARM asserts this bit in l1s_reset(), but the emulated ARM->DSP API bridge
 * never propagates it. We model that write HERE (ARM side), not as a DSP-core poke. */
static int          a2d_ctrlsys = -1;    /* -1 unresolved, 0/1 disabled/enabled   */
static uint16_t     a2d_ctrlsys_cell;    /* d_ctrl_system cell      (0x0810)      */
static uint16_t     a2d_ctrlsys_bit;     /* bit to assert           (0x8000)      */
static uint16_t     a2d_ctrlsys_pollpc;  /* DSP PC just before gate (0xa537)      */
static unsigned     a2d_ctrlsys_posts;   /* how many asserts (log cap)            */

static uint16_t a2d_env_u16(const char *name, uint16_t def)
{
    const char *e = getenv(name);
    if (!e || !*e) {
        return def;
    }
    return (uint16_t)strtoul(e, NULL, 0);
}

static void a2d_resolve(void)
{
    if (a2d_on >= 0) {
        return;
    }
    const char *e = getenv("CALYPSO_ARM2DSP");
    a2d_on   = (e && atoi(e) > 0) ? 1 : 0;
    a2d_word = a2d_env_u16("CALYPSO_ARM2DSP_TASKWORD", 0x0fff);
    a2d_bit  = a2d_env_u16("CALYPSO_ARM2DSP_TASKBIT", 0x0002);

    /* Fix A: faithful background-enable handshake (independent of a2d_on). */
    const char *eb = getenv("CALYPSO_ARM2DSP_BGEN");
    a2d_bgen        = (eb && atoi(eb) > 0) ? 1 : 0;
    a2d_bgen_a      = a2d_env_u16("CALYPSO_ARM2DSP_BGEN_A",      0x098a);
    a2d_bgen_c      = a2d_env_u16("CALYPSO_ARM2DSP_BGEN_C",      0x098c);
    a2d_bgen_val    = a2d_env_u16("CALYPSO_ARM2DSP_BGEN_VAL",    0x0001);
    a2d_bgen_pollpc = a2d_env_u16("CALYPSO_ARM2DSP_BGEN_POLLPC", 0xdddb);
    const char *eo  = getenv("CALYPSO_ARM2DSP_BGEN_ONESHOT");
    a2d_bgen_oneshot = (eo && *eo) ? (atoi(eo) > 0 ? 1 : 0) : 1;

    /* CTRLSYS wire (RANK1): model the ARM's l1s_reset() write of d_ctrl_system
     * bit15 so the DSP go-live gate 0xa53c (BITF data[0x0810],#0x8000) falls
     * through to the bootstrap/FB-dispatch path instead of short-circuiting to
     * 0xa575. Cross-validated: minimal correct value is exactly 0x8000. */
    const char *ec  = getenv("CALYPSO_ARM2DSP_CTRLSYS");
    a2d_ctrlsys        = (ec && atoi(ec) > 0) ? 1 : 0;
    a2d_ctrlsys_cell   = a2d_env_u16("CALYPSO_ARM2DSP_CTRLSYS_CELL",   0x0810);
    a2d_ctrlsys_bit    = a2d_env_u16("CALYPSO_ARM2DSP_CTRLSYS_VAL",    0x8000);
    a2d_ctrlsys_pollpc = a2d_env_u16("CALYPSO_ARM2DSP_CTRLSYS_POLLPC", 0xa537);

    if (a2d_on) {
        fprintf(stderr,
                "[arm2dsp] enabled (faithful task-post): d_dsp_page(0x%04x) bit1 "
                "-> set data[0x%04x] |= 0x%04x (DSP dispatches via ROM)\n",
                A2D_DSP_PAGE_OFF, a2d_word, a2d_bit);
    }
    if (a2d_bgen) {
        fprintf(stderr,
                "[arm2dsp] BGEN enabled (Fix A): on ARM task-cmd post "
                "data[0x%04x]=data[0x%04x]=0x%04x @DSP-PC=0x%04x (oneshot=%d) "
                "-> DSP phase-SM reaches 0xde9c, raises d[0x3f70] bit1\n",
                a2d_bgen_a, a2d_bgen_c, a2d_bgen_val, a2d_bgen_pollpc,
                a2d_bgen_oneshot);
    }
    if (a2d_ctrlsys) {
        fprintf(stderr,
                "[arm2dsp] CTRLSYS enabled (RANK1): assert data[0x%04x] |= 0x%04x "
                "@DSP-PC=0x%04x -> go-live gate 0xa53c falls through to bootstrap/FB\n",
                a2d_ctrlsys_cell, a2d_ctrlsys_bit, a2d_ctrlsys_pollpc);
    }
}

/* ARM->DSP API-RAM write path (calypso_trx.c). offset = ARM byte offset into the
 * DSP API window; value = 16-bit written value. */
void calypso_arm2dsp_on_arm_write(uint16_t offset, uint16_t value)
{
    a2d_resolve();
    if (!a2d_on) {
        return;
    }
    /* The ARM commands the FB task by writing d_dsp_page bit1 (B_GSM_TASK).
     * Each such write re-posts the task (the ARM re-issues it per frame). */
    if (offset == A2D_DSP_PAGE_OFF && (value & A2D_B_GSM_TASK)) {
        a2d_pending = 1;
    }
}

/* Once per DSP instruction step (calypso_c54x.c). When the ARM has posted the
 * task, set the DSP task-ready bit so the DSP's own dispatcher runs it. */
void calypso_arm2dsp_on_dsp_step(C54xState *s, uint16_t exec_pc)
{
    a2d_resolve();
    if (!a2d_on && !a2d_bgen && !a2d_ctrlsys) {
        return;
    }

    /* ---- CTRLSYS: assert d_ctrl_system bit15 at the go-live gate --------------
     * When the DSP reaches the instruction just before the 0xa53c BITF gate, make
     * sure data[0x0810] bit15 is set so BITF sets TC and the DSP falls through to
     * the bootstrap/FB-dispatch path (else BC NTC 0xa575 short-circuits). Modeled
     * as the ARM's write; re-asserted each pass (persists — nothing clears it, but
     * this is robust if the DSP ever does). */
    if (a2d_ctrlsys && exec_pc == a2d_ctrlsys_pollpc &&
        !(s->data[a2d_ctrlsys_cell] & a2d_ctrlsys_bit)) {
        s->data[a2d_ctrlsys_cell] |= a2d_ctrlsys_bit;
        if (a2d_ctrlsys_cell >= A2D_API_BASE && s->api_ram) {
            s->api_ram[a2d_ctrlsys_cell - A2D_API_BASE] |= a2d_ctrlsys_bit;
        }
        if (a2d_ctrlsys_posts++ < 8) {
            fprintf(stderr,
                    "[arm2dsp] CTRLSYS: data[0x%04x] |= 0x%04x (go-live gate "
                    "0xa53c bit15 SET -> bootstrap path) @DSP-PC=0x%04x insn=%u\n",
                    a2d_ctrlsys_cell, a2d_ctrlsys_bit, exec_pc, s->insn_count);
        }
    }

    /* ---- Fix A: faithful background-enable handshake --------------------------
     * Post d_background_enable/state the moment the DSP phase-SM is about to poll
     * them (exec_pc == pollpc), gated on the ARM having commanded the task (the
     * dispatcher bit is set in the task-ready word). One-shot by default: a
     * SINGLE go-live transition, not a per-frame re-fire. */
    if (a2d_bgen && exec_pc == a2d_bgen_pollpc &&
        (!a2d_bgen_oneshot || !a2d_bgen_done)) {
        uint16_t taskw = (a2d_word >= A2D_API_BASE && s->api_ram)
                         ? s->api_ram[a2d_word - A2D_API_BASE]
                         : s->data[a2d_word];
        int armed = (s->data[a2d_word] & a2d_bit) || (taskw & a2d_bit);
        if (armed) {
            s->data[a2d_bgen_a] = a2d_bgen_val;
            s->data[a2d_bgen_c] = a2d_bgen_val;
            if (a2d_bgen_a >= A2D_API_BASE && s->api_ram) {
                s->api_ram[a2d_bgen_a - A2D_API_BASE] = a2d_bgen_val;
            }
            if (a2d_bgen_c >= A2D_API_BASE && s->api_ram) {
                s->api_ram[a2d_bgen_c - A2D_API_BASE] = a2d_bgen_val;
            }
            a2d_bgen_done = 1;
            a2d_bgen_posts++;
            if (a2d_bgen_posts <= 8) {
                fprintf(stderr,
                        "[arm2dsp] BGEN post #%u: data[0x%04x]=data[0x%04x]="
                        "0x%04x @DSP-PC=0x%04x d[0x3f70]=0x%04x insn=%u\n",
                        a2d_bgen_posts, a2d_bgen_a, a2d_bgen_c, a2d_bgen_val,
                        exec_pc, s->data[0x3f70], s->insn_count);
            }
        }
    }

    if (!a2d_on) {
        return;
    }
    if (a2d_cont < 0) a2d_cont = getenv("CALYPSO_ARM2DSP_CONT") ? 1 : 0;
    /* CONT mode : re-post every step while the ARM's B_GSM_TASK is asserted in
     * DSP memory (d_dsp_page word 0x08E2 bit1), so the task-ready bit is set when
     * the dispatcher checks it (b424) despite b419 clearing it. Non-CONT : one
     * post per ARM write (a2d_pending). */
    if (a2d_cont) {
        uint16_t page = s->api_ram ? s->api_ram[0x08E2 - A2D_API_BASE]
                                   : s->data[0x08E2];
        if (!(page & A2D_B_GSM_TASK)) {
            return;
        }
    } else if (!a2d_pending) {
        return;
    }
    a2d_pending = 0;
    a2d_posts++;

    uint16_t before = s->data[a2d_word];
    s->data[a2d_word] |= a2d_bit;
    /* words >= 0x0800 are read by the DSP from api_ram (shared DARAM/API RAM) */
    if (a2d_word >= A2D_API_BASE && s->api_ram) {
        s->api_ram[a2d_word - A2D_API_BASE] |= a2d_bit;
    }

    if (a2d_posts <= 10 || (a2d_posts % 20000) == 0) {
        fprintf(stderr,
                "[arm2dsp] task-post #%u: data[0x%04x] 0x%04x->0x%04x "
                "@DSP-PC=0x%04x insn=%u\n",
                a2d_posts, a2d_word, before, s->data[a2d_word],
                exec_pc, s->insn_count);
    }
}
