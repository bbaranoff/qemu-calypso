/*
 * calypso_trf6151.h — modele de gain RF frontend TRF6151 (porte du firmware
 * osmocom-bb src/target/firmware/rf/trf6151.c).
 *
 * But : le firmware calcule le niveau RF recu par
 *   rf_dbm = pm_baseband_dbm - (system_inherent_gain(71) + trf6151_get_gain())
 * (layer1/agc.c:agc_inp_dbm8_by_pm + board/compal/rffe_dualband.c).
 * Le gain trf6151 est programme par le firmware via des writes TSP (dev 1)
 * dans REG_RX. QEMU suit ces writes, reconstruit le gain, et fournit la
 * valeur a_pm a poser pour qu'un niveau RF cible soit rapporte -> rxlev correct.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_TRF6151_H
#define HW_ARM_CALYPSO_TRF6151_H

#include <stdint.h>

/* Appele depuis calypso_tsp.c a chaque write TSP vers un device != TWL3025.
 * word = mot 16 bits transmis (reg|val). Si c'est un write REG_RX, met a jour
 * le gain courant. dev_idx sert au filtrage (trf6151 = dev 1 en pratique). */
void calypso_trf6151_tsp_write(uint8_t dev_idx, uint32_t word);

/* Gain total courant du frontend RF en dB (system_inherent_gain + trf6151),
 * = ce que le firmware soustrait au baseband. Defaut 71+67=138 (REG_RX=0x9E00,
 * vga=40 + FE_GAIN_HIGH=27, gain max au demarrage du scan). */
int calypso_trf6151_total_gain_db(void);

/* Valeur a ecrire dans a_pm[] (format DSP : l1ddsp_meas_read fait a_pm>>3, puis
 * baseband_dbm = pm/8, donc a_pm = baseband_dbm*64) pour que le firmware
 * rapporte target_rf_dbm au RF : a_pm = (target_rf_dbm + total_gain)*64, borne. */
uint16_t calypso_trf6151_apm_for_rf(int target_rf_dbm);

#endif /* HW_ARM_CALYPSO_TRF6151_H */
