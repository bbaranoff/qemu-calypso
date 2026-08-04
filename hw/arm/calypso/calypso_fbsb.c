/*
 * calypso_fbsb.c — QEMU-side FBSB state tracking (logs only)
 *
 * 2026-05-28 cleanup : all host-side synthesis (publish_fb_found /
 * publish_sb_found / clear_fb / W1C latches / on_frame_tick state
 * machine) removed. fbsb.c logs DSP task changes ; fb0_attempt/sb_attempt sont des compteurs REELS
 * de dispatch (2026-07-27, avant : figes a 0 = red herring qui trompait le diag). FB/SB
 * detection is driven entirely by the DSP (real ROM or L1 stub via
 * CALYPSO_DSP_L1_STUB=1) writing NDB cells, and ARM reads them
 * directly. The only env-gated hack on this path is
 * CALYPSO_FORCE_ANGLE_ZERO (calypso_trx.c).
 *
 * [2026-07-29] LOGGER REBRANCHE SUR DU VIVANT. Le quadruplet « last(...) »
 * etait MORT : les champs last_toa/last_angle/last_pm/last_snr etaient mis a 0
 * par calypso_fbsb_reset() et plus jamais ecrits par personne. La ligne
 * « last(snr=0 toa=0 ang=0 pm=0) » a donc ete lue pendant des semaines comme
 * « le DSP ne rend aucun resultat », alors qu elle ne disait rien du tout —
 * red herring documente. Le dump lit desormais les cellules NDB VIVANTES, et
 * aux DEUX endroits, parce que ces deux vues peuvent diverger :
 *
 *   data[] : vue DSP   — ce que le correlateur natif / le shunt ecrivent
 *   api[]  : vue ARM   — ce que le firmware lit REELLEMENT (prim_fbsb.c:306
 *                        read_fb_result() tape dans api_ram, PAS dans data[])
 *
 * Une divergence entre les deux colonnes n est pas un artefact de sonde :
 * c est le diagnostic. « data[] plein / api[] vide » = le resultat est calcule
 * et n arrive jamais au firmware.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_fbsb.h"
#include "calypso_full_pcb.h"   /* DARAM lock helpers — cf gap #3 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base, uint16_t *api_ram)
{
    if (!s) return;
    s->ndb       = ndb_word_base;
    s->api       = api_ram;
    s->api_base  = api_base;
    calypso_fbsb_reset(s);
}

void calypso_fbsb_reset(CalypsoFbsb *s)
{
    if (!s) return;
    /* NB : ne remet PAS a zero ndb / api / api_base — ce sont des liaisons,
     * pas de l etat de session. */
    s->state       = FBSB_IDLE;
    s->fb0_attempt = 0;
    s->fb1_attempt = 0;
    s->sb_attempt  = 0;
    /* [2026-08-03] fb0_retries / afc_retries supprimes : jamais incrementes,
     * donc toujours 0. Voir la note dans calypso_fbsb_dump(). */
    s->fn_started  = 0;
}

void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn)
{
    /* [2026-08-03] DEDUPE — meme motif que DISPATCH SB et LATCH : 2 554 lignes
     * sur 20 000, pour une tache et un etat qui ne changent pas d'une trame a
     * l'autre. On n'imprime que les transitions (c'est tout l'interet de la
     * sonde : voir la tache CHANGER) plus un resume periodique. Le `fflush`
     * par ligne coutait en plus a chaque appel. */
    {
        static uint32_t l_key = 0xFFFFFFFFu; static unsigned long long rep = 0;
        uint32_t key = ((uint32_t)d_task_md << 8) ^ (uint32_t)(s ? s->state : 0xFF);
        if (key != l_key) {
            if (rep)
                fprintf(stderr, "[calypso-fbsb] on_dsp_task_change × %llu "
                        "(identique, non repete)\n", rep);
            l_key = key; rep = 0;
            fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n",
                    d_task_md, (unsigned long)fn, s ? (int)s->state : -1);
            fflush(stderr);
        } else if (++rep % 2000 == 0) {
            fprintf(stderr, "[calypso-fbsb] on_dsp_task_change × %llu "
                    "(task=%u state=%d, fn=%lu)\n",
                    rep, d_task_md, s ? (int)s->state : -1, (unsigned long)fn);
            fflush(stderr);
        }
    }
    if (!s) return;
    switch (d_task_md) {
    case DSP_TASK_FB:
        s->fb0_attempt++;   /* [2026-07-27] compteur REEL : nb de dispatch tache FB (etait fige a 0 = red herring) */
        s->state       = FBSB_FB0_SEARCH;
        s->fn_started  = fn;
        calypso_fbsb_dump(s, "FB0_SEARCH (real DSP path)");
        break;
    case DSP_TASK_SB:
        s->sb_attempt++;    /* [2026-07-27] compteur REEL : nb de dispatch tache SB */
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

/* Lecture d une cellule NDB dans une des deux vues. `base` vaut s->api_base
 * (0x0800) pour les deux : data[] est indexe depuis &data[0x0800] et api_ram
 * depuis C54X_API_BASE, qui vaut la meme chose. Rend -1 si la vue est absente
 * (pointeur nul) pour distinguer « non liee » de « lue a zero » — cette
 * distinction est exactement celle qui manquait a l ancienne ligne morte. */
static int fbsb_cell(const uint16_t *view, uint16_t base, uint16_t cell)
{
    return view ? (int)view[cell - base] : -1;
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
    const uint16_t b = s->api_base;

    int d_det = fbsb_cell(s->ndb, b, NDB_D_FB_DET);
    int d_toa = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_TOA);
    int d_pm  = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_PM);
    int d_ang = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_ANG);
    int d_snr = fbsb_cell(s->ndb, b, NDB_A_SYNC_DEMOD_SNR);

    int a_det = fbsb_cell(s->api, b, NDB_D_FB_DET);
    int a_toa = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_TOA);
    int a_pm  = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_PM);
    int a_ang = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_ANG);
    int a_snr = fbsb_cell(s->api, b, NDB_A_SYNC_DEMOD_SNR);

    /* TOA et ANGLE sont signes cote firmware.
     *
     * [2026-08-03] `fb0_ret` et `afc_ret` RETIRES de cette ligne. Ils etaient
     * declares (calypso_fbsb.h), remis a zero dans calypso_fbsb_reset(), imprimes
     * ici — et INCREMENTES NULLE PART dans tout l'arbre. Ils valaient donc
     * structurellement 0 a chaque impression, quoi que fasse le firmware.
     *
     * Pourquoi ca comptait : ce zero a ete lu comme une MESURE et cite comme tel
     * dans au moins six endroits, dont le statut de reference lui-meme
     * (doc/ETAT_ACTUEL.md §13.1, « Cause amont : fb0_att=22, fb0_ret=0 [...] la L1
     * est encore en phase de SYNCHRONISATION »). La conclusion tiree de ce champ
     * etait fausse : la console firmware montre FB0, FB1 puis SB qui aboutissent
     * (BSIC=7, `Synchronize_TDMA`) — la L1 depasse bien la synchro.
     *
     * On RETIRE au lieu de cabler : il n'existe aucune notion de « retry » dans ce
     * module, en inventer une serait fabriquer une mesure de plus. Un compteur
     * absent est honnete ; un compteur fige a 0 ment. */
    fprintf(stderr,
            "[fbsb] %s state=%s fb0_att=%u fb1_att=%u sb_att=%u "
            "data[](det=%d toa=%d pm=%d ang=%d snr=0x%04x) "
            "api[](det=%d toa=%d pm=%d ang=%d snr=0x%04x)%s\n",
            tag ? tag : "", names[s->state],
            s->fb0_attempt, s->fb1_attempt, s->sb_attempt,
            d_det, (int)(int16_t)d_toa, d_pm, (int)(int16_t)d_ang, d_snr & 0xFFFF,
            a_det, (int)(int16_t)a_toa, a_pm, (int)(int16_t)a_ang, a_snr & 0xFFFF,
            (d_det > 0 && a_det <= 0) ? "  <<<< DIVERGENCE data/api" : "");
    fflush(stderr);
}
