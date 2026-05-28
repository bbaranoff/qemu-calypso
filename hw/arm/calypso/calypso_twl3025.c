/*
 * calypso_twl3025.c — TWL3025 (Iota) ABB AFC chip model
 *
 * Modèle minimal du TWL3025 (Analog Baseband) pour propager l'AFC du
 * firmware vers les samples BSP en quasi-temps réel.
 *
 * Chaîne silicon :
 *   firmware afc_load_dsp() écrit dsp_api.db_w->d_afc = dac_value
 *   DSP lit d_afc et le serialise vers TWL3025 via TSP
 *   TWL3025 décode le register write → AFC DAC (13 bits, ±4096)
 *   DAC pilote VCXO 13 MHz → décalage fréquence baseband
 *
 * En QEMU :
 *   - calypso_twl3025_set_afc_dac()   appelé depuis la chaîne TSP avec
 *                                     la valeur DAC écrite par le firmware
 *   - calypso_twl3025_apply_phase()   rotation des samples BSP par
 *                                     dac_value × afc_slope (Hz), au taux
 *                                     baseband 270.833 kHz, avec offset
 *                                     déterministe basé sur FN/TN
 *
 * Le modèle est ARMÉ PAR DÉFAUT (= fait son taf chip-level, pas d'env
 * gate). Override via CALYPSO_TWL3025_AFC_HZ=N pour injecter un offset
 * constant (= diag, court-circuite la chaîne DAC).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>

#include "hw/arm/calypso/calypso_twl3025.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* TWL3025 / Compal E88 vcxocal constants (rf/vcxocal.h osmocom-bb) :
 *   afc_slope = 287 Hz/LSB (= dérivée linéaire VCXO à 900 MHz GSM)
 *   afc_initial_dac_value = -700 (= calibration board E88, compense le
 *                                  trim du quartz physique)
 *   d_afc range : ±4095 (13-bit signed DAC)
 *
 * En QEMU les samples osmo-bts arrivent déjà à la freq carrier exacte
 * (= comme si le silicon avait DEJA appliqué la calibration -700). Donc
 * on interprète le DAC firmware *relativement à la calibration baseline* :
 *   effective_dac = dac_value - AFC_INITIAL_DAC_VALUE
 * DAC=-700 (= calib idéale) → effective=0 → no rotation
 * DAC=0    (= "+700 LSB au-dessus de la calib") → effective=+700 → +200900Hz
 */
#define TWL3025_AFC_SLOPE_HZ_PER_LSB    287.0
#define TWL3025_AFC_INITIAL_DAC_VALUE   (-700)
#define GSM_SAMPLE_RATE_HZ              270833.0  /* 1 sps GSM baseband */

/* GSM TDMA timing constants (1 sample/bit BSP rate) :
 *   1 frame  = 4.615 ms = 1250 samples
 *   1 slot   = 156.25 symbols ≈ 156 samples (rounded down)
 */
#define SAMPLES_PER_FRAME   1250U
#define SAMPLES_PER_SLOT    156U

static struct {
    int16_t  dac_value;     /* current DAC reg value (= dernière écriture firmware) */
    int      force_hz;      /* CALYPSO_TWL3025_AFC_HZ override (diag, default 0 = off) */
    bool     env_loaded;
    uint64_t dac_writes;    /* compteur diag */
    uint64_t apply_calls;
} twl;

static void twl3025_lazy_env(void)
{
    if (twl.env_loaded) return;
    twl.env_loaded = true;
    const char *h = getenv("CALYPSO_TWL3025_AFC_HZ");
    twl.force_hz = (h && *h) ? atoi(h) : 0;
    fprintf(stderr,
            "[twl3025] chip model armed (slope=%.1f Hz/LSB, force_hz=%d)\n",
            TWL3025_AFC_SLOPE_HZ_PER_LSB, twl.force_hz);
}

void calypso_twl3025_set_afc_dac(int16_t dac_value)
{
    twl3025_lazy_env();
    if (dac_value > 4095)  dac_value = 4095;
    if (dac_value < -4096) dac_value = -4096;

    /* Filter dual-write 0-clobber pattern : firmware writes d_afc to
     * BOTH dsp_api pages each frame (page A=0 inherited init, page B=
     * real value). Sur silicon, le TWL3025 register est sticky via
     * TSP serialization — le 0 transient n'a pas le temps de propager
     * avant le -700 effectif. Notre modèle voit les 2 writes en MMIO
     * direct → oscillation 0↔-700.
     *
     * Heuristique : ignorer un write dac=0 si la valeur courante est
     * déjà set (non-zero). Le firmware reset légitime via afc_reset
     * écrit afc_initial_dac_value (=-700 sur Compal E88), pas 0. */
    if (dac_value == 0 && twl.dac_value != 0) {
        return;
    }

    if (twl.dac_value != dac_value) {
        twl.dac_writes++;
        /* Throttled log : initial 20 writes puis toutes les 1000 pour
         * visualiser convergence sans noyer le log AFC. */
        if (twl.dac_writes <= 20 || (twl.dac_writes % 1000) == 0) {
            fprintf(stderr,
                "[twl3025] DAC %d → %d (%.1f Hz, write #%" PRIu64 ")\n",
                twl.dac_value, dac_value,
                dac_value * TWL3025_AFC_SLOPE_HZ_PER_LSB,
                twl.dac_writes);
        }
        twl.dac_value = dac_value;
    }
}

int16_t calypso_twl3025_get_afc_dac(void)
{
    return twl.dac_value;
}

double calypso_twl3025_get_afc_hz(void)
{
    twl3025_lazy_env();
    if (twl.force_hz != 0) return (double)twl.force_hz;
    /* Effective DAC = écriture firmware - calibration baseline.
     * Cf comment en tête du fichier sur la sémantique baseline. */
    int32_t effective_dac = (int32_t)twl.dac_value - TWL3025_AFC_INITIAL_DAC_VALUE;
    return (double)effective_dac * TWL3025_AFC_SLOPE_HZ_PER_LSB;
}

double calypso_twl3025_get_afc_phase_step(void)
{
    twl3025_lazy_env();
    double hz = calypso_twl3025_get_afc_hz();
    if (hz == 0.0) return 0.0;
    /* Phase step per sample = 2π × freq / fs.
     * Signe : VCXO + → osc UP → received baseband freq DOWN (down-conv).
     * Compensation = rotation samples par -phase_step. */
    return -2.0 * M_PI * hz / GSM_SAMPLE_RATE_HZ;
}

void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                 uint32_t fn, uint8_t tn)
{
    twl3025_lazy_env();
    twl.apply_calls++;

    double step = calypso_twl3025_get_afc_phase_step();
    if (step == 0.0) return;   /* DAC=0 et pas de force_hz : no-op */

    /* Sample offset absolu depuis FN=0,TN=0 = phase de reference système.
     * Permet la continuité phase entre bursts : burst N+1 starts at
     * (N+1) × 1250 + tn × 156, donc cos/sin restent cohérents. */
    uint64_t sample_offset = (uint64_t)fn * SAMPLES_PER_FRAME
                           + (uint64_t)tn * SAMPLES_PER_SLOT;

    for (int i = 0; i < n_samples; i++) {
        double ph = step * (double)(sample_offset + (uint64_t)i);
        double c = cos(ph), s = sin(ph);
        int16_t I = iq_samples[2 * i];
        int16_t Q = iq_samples[2 * i + 1];
        double new_I = I * c - Q * s;
        double new_Q = I * s + Q * c;
        if (new_I >  32767.0) new_I =  32767.0;
        if (new_I < -32768.0) new_I = -32768.0;
        if (new_Q >  32767.0) new_Q =  32767.0;
        if (new_Q < -32768.0) new_Q = -32768.0;
        iq_samples[2 * i]     = (int16_t)new_I;
        iq_samples[2 * i + 1] = (int16_t)new_Q;
    }
}

void calypso_twl3025_reset(void)
{
    twl.dac_value   = 0;
    twl.dac_writes  = 0;
    twl.apply_calls = 0;
    /* env_loaded gardé : on ne recharge pas l'env au reset (état chip). */
}
