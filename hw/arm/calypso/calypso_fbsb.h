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
 *   d_dsp_page         0x08D4    (page toggle from ARM)
 *   d_fb_det           0x08F9    (DSP → ARM: non-zero = FB found)
 *   d_fb_mode          0x08FA    (ARM → DSP: 0 = wideband search, 1 = narrow)
 *   a_sync_demod[0]    0x08FB    D_TOA  — time-of-arrival
 *   a_sync_demod[1]    0x08FC    D_PM   — power measurement
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
#define NDB_D_DSP_PAGE       0x08D4
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

    /* Per-attempt counters mirroring prim_fbsb.c. */
    uint8_t          fb0_attempt;
    uint8_t          fb1_attempt;
    uint8_t          sb_attempt;
    uint8_t          fb0_retries;
    uint8_t          afc_retries;

    /* Last DSP result snapshot (what we'd write to a_sync_demod). */
    int16_t          last_toa;
    int16_t          last_angle;
    uint16_t         last_pm;
    uint16_t         last_snr;

    /* Bookkeeping. */
    uint64_t         fn_started;
} CalypsoFbsb;

/* Lifecycle. */
void calypso_fbsb_init(CalypsoFbsb *s, uint16_t *ndb_word_base,
                       uint16_t api_base);
void calypso_fbsb_reset(CalypsoFbsb *s);

/* Hooks. */
void calypso_fbsb_on_dsp_task_change(CalypsoFbsb *s, uint16_t d_task_md,
                                     uint64_t fn);
void calypso_fbsb_on_frame_tick(CalypsoFbsb *s, uint64_t fn);

/* Direct write helpers (used by the orchestration to publish results
 * into NDB so the ARM firmware sees them). */
void calypso_fbsb_publish_fb_found(CalypsoFbsb *s,
                                   int16_t toa, uint16_t pm,
                                   int16_t angle, uint16_t snr);
void calypso_fbsb_clear_fb(CalypsoFbsb *s);

/* SB synthesis: writes a_sch[0..4] in BOTH db_r pages so l1s_sbdet_resp
 * sees CRC=OK + a decodable sync burst regardless of d_dsp_page state. */
void calypso_fbsb_publish_sb_found(CalypsoFbsb *s, uint8_t bsic);

/* Trace helper — single-line dump of current state. */
void calypso_fbsb_dump(const CalypsoFbsb *s, const char *tag);

#endif /* CALYPSO_FBSB_H */
