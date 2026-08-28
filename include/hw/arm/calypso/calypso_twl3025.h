/*
 * calypso_twl3025.h — TWL3025 ABB AFC propagation API.
 *
 * Modèle minimal qui propage l'AFC (Automatic Frequency Correction)
 * du firmware vers les samples BSP. Voir calypso_twl3025.c pour la
 * chaîne complète.
 *
 * Activation : env CALYPSO_TWL3025_AFC=1.
 * Test forçage : env CALYPSO_TWL3025_AFC_HZ=N (= offset constant Hz).
 */
#ifndef CALYPSO_TWL3025_H
#define CALYPSO_TWL3025_H

#include "qemu/osdep.h"

/* Update DAC value (received from DSP via TSP serial, decoded TWL3025 reg).
 * Phase 1 API direct : à appeler quand on détecte une écriture d_afc côté
 * QEMU. Phase 2 : à appeler depuis hook TSP byte stream. */
void calypso_twl3025_set_afc_dac(int16_t dac_value);

/* Read current DAC (signed, ±4095). */
int16_t calypso_twl3025_get_afc_dac(void);

/* Get current AFC offset in Hz (DAC * slope). */
double calypso_twl3025_get_afc_hz(void);

/* Phase step per sample (rad) for sample rotation in BSP receive path.
 * Returns 0.0 si AFC propagation désactivée (CALYPSO_TWL3025_AFC != 1).
 * Sign convention : VCXO + → samples reçus shift opposite, donc step est
 * la rotation à appliquer aux samples bruts pour matcher le local osc. */
double calypso_twl3025_get_afc_phase_step(void);

/* Apply AFC rotation in-place to N I/Q samples (interleaved int16).
 * No-op si AFC désactivée OU dac_value == 0.
 *
 * Phase computation FN/TN-based (2026-05-28) : la phase au sample i du burst
 * est determinee uniquement par (fn, tn, i), pas par un accumulateur :
 *   sample_offset = fn * SAMPLES_PER_FRAME + tn * SAMPLES_PER_SLOT
 *   phase(i) = step * (sample_offset + i)
 * Avantages vs ancien phase_accum :
 *   - Pas de drift (chaque burst recalcule de zero a partir de fn)
 *   - Pas de race si bursts arrivent out-of-order ou doublons
 *   - Gaps TDMA inter-bursts pris en compte automatiquement (= fn avance) */
void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                 uint32_t fn, uint8_t tn);

/* Reset DAC + phase accumulator. À appeler au reset DSP/BSP. */
void calypso_twl3025_reset(void);

#endif /* CALYPSO_TWL3025_H */
