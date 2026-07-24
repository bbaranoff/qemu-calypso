#ifndef CALYPSO_ARM2DSP_H
#define CALYPSO_ARM2DSP_H

#include "calypso_c54x.h"

/*
 * calypso_arm2dsp — ARM->DSP orchestration bridge.
 *
 * On real Calypso the ARM (osmocom L1) orchestrates the C54x DSP: it uploads
 * the DSP tasks and commands them via the shared API RAM. The FB/FCCH search is
 * commanded by writing d_dsp_page bit1 (B_GSM_TASK). In this emulator the DSP
 * stalls in its go-live wait-loop (0xa4d4, BITF data[0x3f70] #0x0002) because the
 * foreground setter that would set bit1 (0xde9c/0xa5bd/0xb3ef/0x710c) is never
 * dispatched — the ARM->DSP go-live handshake is not wired.
 *
 * This module wires that handshake: when the ARM issues the FB-task orchestration
 * write, we drive the DSP into a setter path so it sets 0x3f70 bit1, runs the
 * go-live follow-through (ST #2,0x3f70 ; CALL 0xa9ea ; ... ; ORM #,IMR / RSBX INTM)
 * and arms its own IMR. Gated by CALYPSO_ARM2DSP=1 (off by default).
 *
 * Env overrides (for bracketing, no rebuild):
 *   CALYPSO_ARM2DSP=1            enable
 *   CALYPSO_ARM2DSP_TGT=0xb3ec   redirect target PC (default 0xb3ec setter path)
 *   CALYPSO_ARM2DSP_AT=0xa4d4    DSP PC at which to apply the redirect (default 0xa4d4)
 *   CALYPSO_ARM2DSP_SP=0x1100    if set, force SP to this before redirect (context)
 *   CALYPSO_ARM2DSP_N=1          how many times to drive (default 1)
 */

/* Called from the ARM->DSP API-RAM write path (calypso_trx.c) on every ARM write.
 * offset = ARM byte offset into the DSP API window; value = 16-bit written value. */
void calypso_arm2dsp_on_arm_write(uint16_t offset, uint16_t value);

/* Called once per DSP instruction step (calypso_c54x.c) with the DSP state and the
 * PC about to execute. Applies a pending go-live drive at the configured PC. */
void calypso_arm2dsp_on_dsp_step(C54xState *s, uint16_t exec_pc);

#endif /* CALYPSO_ARM2DSP_H */
