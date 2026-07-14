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
#include "hw/arm/calypso/calypso_full_pcb.h"   /* DARAM lock helpers — cf gap #3 */
#include "calypso_orch.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Real host-measured FB detection from the BSP FCCH correlator. */
extern int calypso_bsp_get_fb_detection(int16_t *toa, uint16_t *pm,
                                        int16_t *ang, uint16_t *snr);

/* Synthèse FB côté hôte : OFF par défaut (le DSP/ROM est censé écrire la
 * NDB). On l'active pour le chemin stub L1, où personne ne pose le TOA. */
/* True only if the env var is set to a non-empty, non-"0" value.
 * getenv() alone returns non-NULL even for "=0", which wrongly kept the
 * host synth on when the user set CALYPSO_DSP_L1_STUB=0 to disable it. */
static int fbsb_env_on(const char *name)
{
    const char *e = getenv(name);
    return (e && e[0] && e[0] != '0') ? 1 : 0;
}

static int fbsb_synth_enabled(void)
{
    static int v = -1;
    if (v < 0)
        v = (fbsb_env_on("CALYPSO_SYNTH_FBSB") ||
             fbsb_env_on("CALYPSO_DSP_L1_STUB")) ? 1 : 0;
    return v;
}

/* Ré-introduit : écrit d_fb_det + a_sync_demod_{toa,pm,ang,snr} dans la NDB,
 * exactement ce que CALL_FLOW.md étape 4 attend. */
void calypso_fbsb_publish_fb_found(CalypsoFbsb *s, int16_t toa, uint16_t pm,
                                   int16_t ang, uint16_t snr)
{
    if (!calypso_orch()) return;   /* exe: real DSP writes the NDB */
    if (!s || !s->ndb) return;

    calypso_pcb_daram_lock_acquire();

    s->ndb[NDB_D_FB_DET]            = 1;
    s->ndb[NDB_A_SYNC_DEMOD_TOA]   = (uint16_t)toa;   /* <-- le wire manquant */
    s->ndb[NDB_A_SYNC_DEMOD_PM]    = pm;
    s->ndb[NDB_A_SYNC_DEMOD_ANG]   = (uint16_t)ang;
    s->ndb[NDB_A_SYNC_DEMOD_SNR]   = snr;

    calypso_pcb_daram_lock_release();

    s->last_toa   = toa;
    s->last_pm    = pm;
    s->last_angle = ang;
    s->last_snr   = snr;
    s->state      = FBSB_FB0_FOUND;

    calypso_fbsb_dump(s, "FB0_FOUND (host synth)");
}

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
        s->fb0_attempt++;
        calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        /* Rien n'écrit les cellules sync/demod sur le chemin stub →
         * l'ARM relit un TOA périmé. On republie la détection synthétique
         * (CALL_FLOW.md étape 4) si activé. */
        if (calypso_orch() && fbsb_synth_enabled()) {
            int16_t toa, ang;
            uint16_t pm, snr;
            /* Wire the REAL host-measured FB detection (BSP FCCH correlator)
             * in place of synth constants — publish only on a genuine FB. */
            if (calypso_bsp_get_fb_detection(&toa, &pm, &ang, &snr))
                calypso_fbsb_publish_fb_found(s, toa, pm, ang, snr);
        }
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
    /* NOTE: les vraies valeurs FB (toa/pm/ang/snr) sont écrites dans les
     * cellules DSP a_sync_demod par le BSP (real DSP path) ; le synth host est
     * mort. On ne logge donc PLUS s->last_* (toujours 0, trompeur). */
    fprintf(stderr,
            "[fbsb] %s state=%s fb0_att=%u fb1_att=%u sb_att=%u "
            "fb0_ret=%u afc_ret=%u\n",
            tag ? tag : "", names[s->state],
            s->fb0_attempt, s->fb1_attempt, s->sb_attempt,
            s->fb0_retries, s->afc_retries);
    fflush(stderr);
}
