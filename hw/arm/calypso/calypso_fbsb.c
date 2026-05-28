/*
 * calypso_fbsb.c — QEMU-side FBSB state tracking (logs only)
 *
 * 2026-05-28 cleanup : all host-side synthesis (publish_fb_found /
 * publish_sb_found / clear_fb / W1C latches / on_frame_tick state
 * machine) removed. fbsb.c now only logs DSP task changes. FB/SB
 * detection is driven entirely by the DSP (real ROM or L1 stub via
 * CALYPSO_DSP_L1_STUB=1) writing NDB cells, and ARM reads them
 * directly. The only env-gated hack on this path is
 * CALYPSO_FORCE_ANGLE_ZERO (calypso_trx.c).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_fbsb.h"
#include "calypso_full_pcb.h"   /* DARAM lock helpers — cf gap #3 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base)
{
    if (!s) return;
    s->ndb       = ndb_word_base;
    s->api_base  = api_base;
    calypso_fbsb_reset(s);
}

void calypso_fbsb_reset(CalypsoFbsb *s)
{
    if (!s) return;
    s->state       = FBSB_IDLE;
    s->fb0_attempt = 0;
    s->fb1_attempt = 0;
    s->sb_attempt  = 0;
    s->fb0_retries = 0;
    s->afc_retries = 0;
    s->last_toa    = 0;
    s->last_angle  = 0;
    s->last_pm     = 0;
    s->last_snr    = 0;
    s->fn_started  = 0;
}

void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn)
{
    fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n",
            d_task_md, (unsigned long)fn, s ? (int)s->state : -1);
    fflush(stderr);
    if (!s) return;
    switch (d_task_md) {
    case DSP_TASK_FB:
        s->state       = FBSB_FB0_SEARCH;
        s->fn_started  = fn;
        calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        break;
    case DSP_TASK_SB:
        s->state      = FBSB_SB_SEARCH;
        s->fn_started = fn;
        calypso_fbsb_dump(s, "SB_SEARCH (real DSP path)");
        break;
    case DSP_TASK_ALLC: {
        static int log_once;
        if (!log_once++) {
            fprintf(stderr,
                    "[fbsb] ALLC task=24 fn=%lu — real DSP CCCH demod\n",
                    (unsigned long)fn);
            fflush(stderr);
        }
        break;
    }
    case DSP_TASK_NONE:
    default:
        break;
    }
}

void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag)
{
    if (!s) return;
    static const char *names[] = {
        "IDLE", "FB0_SEARCH", "FB0_FOUND",
        "FB1_SEARCH", "FB1_FOUND",
        "SB_SEARCH",  "SB_FOUND",
        "DONE", "FAIL",
    };
    fprintf(stderr,
            "[fbsb] %s state=%s fb0_att=%u fb1_att=%u sb_att=%u "
            "fb0_ret=%u afc_ret=%u last(snr=%u toa=%d ang=%d pm=%u)\n",
            tag ? tag : "", names[s->state],
            s->fb0_attempt, s->fb1_attempt, s->sb_attempt,
            s->fb0_retries, s->afc_retries,
            s->last_snr, s->last_toa, s->last_angle, s->last_pm);
    fflush(stderr);
}
