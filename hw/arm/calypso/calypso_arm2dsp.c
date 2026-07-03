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
 */
#include "qemu/osdep.h"
#include "calypso_arm2dsp.h"

#include <stdlib.h>
#include <stdio.h>

/* ARM byte offset of d_dsp_page (DSP word 0x08D4). B_GSM_TASK = bit1. */
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
    if (a2d_on) {
        fprintf(stderr,
                "[arm2dsp] enabled (faithful task-post): d_dsp_page(0x%04x) bit1 "
                "-> set data[0x%04x] |= 0x%04x (DSP dispatches via ROM)\n",
                A2D_DSP_PAGE_OFF, a2d_word, a2d_bit);
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
    if (!a2d_on) {
        return;
    }
    if (a2d_cont < 0) a2d_cont = getenv("CALYPSO_ARM2DSP_CONT") ? 1 : 0;
    /* CONT mode : re-post every step while the ARM's B_GSM_TASK is asserted in
     * DSP memory (d_dsp_page word 0x08D4 bit1), so the task-ready bit is set when
     * the dispatcher checks it (b424) despite b419 clearing it. Non-CONT : one
     * post per ARM write (a2d_pending). */
    if (a2d_cont) {
        uint16_t page = s->api_ram ? s->api_ram[0x08D4 - A2D_API_BASE]
                                   : s->data[0x08D4];
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
    /* words >= 0x0800 are read by the DSP from api_ram (shared DARAM/API RAM).
     * Bound the index against the API-RAM backing store (C54X_API_SIZE words):
     * a2d_word comes from CALYPSO_ARM2DSP_TASKWORD (env, up to 0xFFFF) and
     * without the upper check an out-of-range value would OR-write past the end
     * of api_ram[]. */
    if (a2d_word >= A2D_API_BASE && s->api_ram &&
        (unsigned)(a2d_word - A2D_API_BASE) < C54X_API_SIZE) {
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
