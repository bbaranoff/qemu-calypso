/*
 * calypso_fbsb.h — QEMU-side FBSB (FCCH/SCH burst search) orchestration
 *
 * Mirrors the firmware's prim_fbsb.c state machine but lives entirely in
 * QEMU. The goal is to handle the FBSB sequence autonomously when the
 * emulated DSP cannot drive the NDB cells correctly (because of opcode
 * gaps, missing C548 extensions, etc).
 *
 * The "real" osmocom-bb flow is in:
 *   src/target/firmware/layer1/prim_fbsb.c
 *
 * That file's state machine, mirrored here:
 *
 *   IDLE
 *     │
 *     │  ARM writes d_task_md = FB_DSP_TASK (mode 0)
 *     ▼
 *   FB0_SEARCH ── correlator finds burst ──► FB0_FOUND
 *     │                                         │
 *     │ 12 attempts no FB                       │ ferr small enough
 *     │                                         ▼
 *     ▼                                       FB1_SEARCH ──► FB1_FOUND
 *   FAIL (result=255)                            │              │
 *                                                ▼              ▼
 *                                              FAIL          SB_SEARCH ──► SB_FOUND
 *                                                                │            │
 *                                                                ▼            ▼
 *                                                              FAIL        SUCCESS
 *
 * NDB cells we read/write (offsets in DSP data words from API base 0x0800):
 *   d_dsp_page         0x08E2    (page toggle from ARM)
 *   d_fb_det           0x08F9    (DSP → ARM: non-zero = FB found)
 *   d_fb_mode          0x08FA    (ARM → DSP: 0 = wideband search, 1 = narrow)
 *   a_sync_demod[0]    0x08FB    D_TOA  — time-of-arrival
 *   a_sync_demod[1]    0x08FB    D_PM   — power measurement
 *     [2026-07-30] Corrige : ce commentaire disait 0x08FC, en desaccord d'UN MOT
 *     avec le #define NDB_A_SYNC_DEMOD_PM (0x08FB) plus bas. Le firmware tranche
 *     (include/calypso/dsp_api.h:202-204) : d_fb_det, d_fb_mode, puis
 *     a_sync_demod[4] = {D_TOA, D_PM, D_ANGLE, D_SNR} -> 0x08F8, 0x08F9, puis
 *     0x08FA..0x08FD. Le define etait bon, le commentaire faux. Meme famille
 *     d'erreur que l'offset d_dsp_page corrige le 29/07 : un decalage d'un mot
 *     dans un commentaire finit par etre lu comme une adresse.
 *   a_sync_demod[2]    0x08FD    D_ANGLE — frequency phase angle
 *   a_sync_demod[3]    0x08FE    D_SNR  — signal-to-noise ratio
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_FBSB_H
#define CALYPSO_FBSB_H

#include <stdint.h>
#include <stdbool.h>

/* NDB cell offsets — DSP data word addresses. */
/* Offsets verified against calypso_trx.c lines 575-581: ARM sees NDB
 * starting at byte 0x01A8 (= dsp_ram word 0xD4), and d_fb_det is at
 * dsp_ram[0xF8]. With api_base=0x0800 → DSP word = 0x0800 + 0xF8. */
/* [2026-07-29] CORRECTION D'ADRESSE — d_dsp_page etait defini a 0x08E2, soit
 * NDB+14 mots : c'est d_dsp_state. Trois sources concordantes :
 *   1. osmocom-bb dsp_api.h:18  BASE_API_NDB = 0xFFD001A8 -> mot DSP 0x0800 +
 *      0x1A8/2 = 0x08D4, et d_dsp_page est le champ 0 de T_NDB_MCU_DSP ;
 *      d_dsp_state est le champ 14 -> 0x08E2.
 *   2. Runtime : la seule ecriture ARM sur 0x08E2 vient de dsp.c:215
 *      (ndb->d_dsp_state = 3 = C_DSP_IDLE3), a l'offset ARM 0x01C4.
 *   3. La ROM DSP lit 0x08d4 (0xa51c = selecteur de page bit0 -> pages
 *      0x0800/0x0828 vs 0x0814/0x083c ; 0xc8ea) et ne reference JAMAIS 0x08e2
 *      dans les 5 ROM programme.
 * Le commentaire ci-dessus disait deja « NDB = mot 0xD4 » — seule la constante
 * etait fausse. Le chemin ARM principal, lui, etait correct (api_ram==dsp_ram,
 * cf calypso_trx.c:2033), donc seuls les ecrivains/sondes SECONDAIRES tapaient
 * a cote. */
#define NDB_D_DSP_PAGE       0x08D4
#define NDB_D_DSP_STATE      0x08E2
#define NDB_D_FB_DET         0x08F8
#define NDB_D_FB_MODE        0x08F9
#define NDB_A_SYNC_DEMOD_TOA 0x08FA
#define NDB_A_SYNC_DEMOD_PM  0x08FB
#define NDB_A_SYNC_DEMOD_ANG 0x08FC
#define NDB_A_SYNC_DEMOD_SNR 0x08FD

/* d_task_md values used by the firmware (subset).
 * From osmocom-bb dsp_api.h — verified against l1s_pm_cmd / l1s_fbdet_cmd. */
#define DSP_TASK_NONE       0
#define DSP_TASK_PM         1   /* PM_DSP_TASK (power measurement, NOT FB) */
#define DSP_TASK_FB         5   /* FB_DSP_TASK (frequency burst, idle) */
#define DSP_TASK_SB         6   /* SB_DSP_TASK (sync burst, idle)      */
#define DSP_TASK_TCH_FB     8   /* TCH_FB_DSP_TASK (dedicated) */
#define DSP_TASK_TCH_SB     9   /* TCH_SB_DSP_TASK (dedicated) */
#define DSP_TASK_ALLC      24   /* ALLC_DSP_TASK (CCCH read while FULL BCCH/CCCH) */

/* FBSB orchestration state. One instance per Calypso. */
typedef enum {
    FBSB_IDLE = 0,
    FBSB_FB0_SEARCH,
    FBSB_FB0_FOUND,
    FBSB_FB1_SEARCH,
    FBSB_FB1_FOUND,
    FBSB_SB_SEARCH,
    FBSB_SB_FOUND,
    FBSB_DONE,
    FBSB_FAIL,
} CalypsoFbsbState;

typedef struct CalypsoFbsb {
    CalypsoFbsbState state;
    uint16_t        *ndb;          /* points into ARM dsp_ram[] (word-addressed) */
    uint16_t         api_base;     /* DSP-side word base (0x0800) */

    /* Per-attempt counters mirroring prim_fbsb.c. Ceux-ci sont REELLEMENT
     * incrementes (calypso_fbsb_on_dsp_task_change). */
    uint8_t          fb0_attempt;
    uint8_t          fb1_attempt;
    uint8_t          sb_attempt;
    /* [2026-08-03] fb0_retries / afc_retries SUPPRIMES — meme defaut que les
     * quatre champs last_* decrits juste dessous, et passe inapercu au meme
     * menage : declares, remis a 0, imprimes, jamais incrementes. Le `fb0_ret=0`
     * qui en sortait a ete cite comme mesure dans six endroits, dont
     * doc/ETAT_ACTUEL.md §13.1. Detail : note dans calypso_fbsb_dump(). */

    /* [2026-07-29] Vue ARM. `ndb` pointe sur data[] cote DSP ; `api` pointe sur
     * api_ram, c est-a-dire ce que le firmware lit REELLEMENT (prim_fbsb.c
     * read_fb_result). Les deux vues peuvent diverger — c est le diagnostic.
     * Remplace les 4 champs last_* qui etaient MORTS (mis a 0 au reset, jamais
     * ecrits) et faisaient lire « last(snr=0 toa=0 ang=0 pm=0) » comme « le DSP
     * ne rend rien » alors que la ligne ne disait rien du tout. */
    uint16_t        *api;

    /* Bookkeeping. */
    uint64_t         fn_started;
} CalypsoFbsb;

/* Lifecycle. */
void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base, uint16_t *api_ram);
void calypso_fbsb_reset(CalypsoFbsb *s);

/* Hooks. */
void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn);

/* Trace helper — single-line dump of current state. */
void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag);

#endif /* CALYPSO_FBSB_H */
