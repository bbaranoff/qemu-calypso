/*
 * calypso_trf6151.c — modele de gain RF frontend TRF6151.
 *
 * Porte de osmocom-bb src/target/firmware/rf/trf6151.c (get_gain_reg) +
 * board/compal/rffe_dualband.c (SYSTEM_INHERENT_GAIN) + layer1/agc.c
 * (agc_inp_dbm8_by_pm) + layer1/prim_pm.c (l1ddsp_meas_read : pm = a_pm>>3).
 *
 * Chaine firmware :
 *   pm_level  = dsp_api.db_r->a_pm[i] >> 3            (prim_pm.c)
 *   bb_dbm    = pm_level / 8                          (1/8 dBm -> dBm)
 *   rf_dbm    = bb_dbm - (SYSTEM_INHERENT_GAIN + trf6151_get_gain())   (agc.c)
 * => a_pm = bb_dbm * 64 = (rf_dbm + total_gain) * 64.
 *
 * Le firmware programme trf6151 par TSP (dev 1) : tsp_write(uid,16,reg|val).
 * REG_RX (adresse 0) porte le gain (FE high/low bits[10:9], VGA bits[15:11]).
 * On decode REG_RX a chaque write pour suivre le gain vivant (l'AGC le baisse
 * quand le signal est fort) -> a_pm recalcule -> rf cible tenu quel que soit
 * le gain choisi par le firmware.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdlib.h>
#include "hw/arm/calypso/calypso_trf6151.h"

/* ---- constantes portees telles quelles du firmware ---- */
#define SYSTEM_INHERENT_GAIN    71   /* rffe_dualband.c */
#define TRF6151_FE_GAIN_LOW      7   /* trf6151.c */
#define TRF6151_FE_GAIN_HIGH    27
#define TRF6151_VGA_GAIN_MIN    14
#define RX_VGA_GAIN_SHIFT       11   /* REG_RX bits[15:11] */
/* REG_RX addresse = 0 dans l'enum trf6151_reg ; le firmware transmet reg|val,
 * donc les 3 bits de poids faible du mot = l'adresse registre. */
#define TRF6151_REG_RX           0
#define TRF6151_REG_ADDR_MASK    0x7

/* REG_RX au reset (trf6151_reg_cache) : vga=40, FE high -> gain 67. */
#define TRF6151_REG_RX_RESET    0x9E00

static uint16_t g_reg_rx = TRF6151_REG_RX_RESET;

/* Port exact de trf6151_get_gain_reg() : gain frontend depuis REG_RX. */
static int trf6151_gain_from_reg(uint16_t reg_rx)
{
    int gain = 0;
    unsigned vga;

    switch ((reg_rx >> 9) & 3) {
    case 0: gain += TRF6151_FE_GAIN_LOW;  break;
    case 3: gain += TRF6151_FE_GAIN_HIGH; break;
    default: /* valeurs intermediaires non utilisees par le firmware */ break;
    }

    vga = (reg_rx >> RX_VGA_GAIN_SHIFT) & 0x1f;
    if (vga < 6) {
        vga = 6;
    }
    gain += TRF6151_VGA_GAIN_MIN + (int)(vga - 6) * 2;

    return gain;
}

void calypso_trf6151_tsp_write(uint8_t dev_idx, uint32_t word)
{
    /* Le trf6151 est le device RF frontend. En pratique dev 1 (dev 0 = TWL3025
     * ABB, deja consomme ailleurs). On accepte 1..7 et on ne retient que les
     * writes REG_RX (adresse 0) qui portent le gain. */
    static int trf_dev = -1;
    if (trf_dev < 0) {
        const char *e = getenv("CALYPSO_TRF_TSP_DEV");
        trf_dev = (e && *e) ? atoi(e) : 1;
    }
    if (dev_idx != (uint8_t)trf_dev) {
        return;
    }
    if ((word & TRF6151_REG_ADDR_MASK) != TRF6151_REG_RX) {
        return; /* pas un write REG_RX -> pas le registre de gain */
    }
    g_reg_rx = (uint16_t)word;
}

int calypso_trf6151_total_gain_db(void)
{
    return SYSTEM_INHERENT_GAIN + trf6151_gain_from_reg(g_reg_rx);
}

uint16_t calypso_trf6151_apm_for_rf(int target_rf_dbm)
{
    int total_gain = calypso_trf6151_total_gain_db();
    int bb_dbm = target_rf_dbm + total_gain;     /* baseband dBm vise */
    int apm;

    if (bb_dbm < 0) {
        bb_dbm = 0;      /* le baseband ne descend pas sous 0 dBm ici */
    }
    apm = bb_dbm * 64;   /* a_pm = bb_dbm*64 (l1ddsp_meas_read: pm=a_pm>>3, bb=pm/8) */
    if (apm > 0xFFFF) {
        apm = 0xFFFF;
    }
    return (uint16_t)apm;
}
