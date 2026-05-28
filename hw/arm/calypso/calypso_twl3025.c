/*
 * calypso_twl3025.c — TWL3025 (Iota) ABB AFC model.
 *
 * Modèle minimal du TWL3025 (Analog Baseband) pour propager l'AFC
 * du firmware vers les samples BSP.
 *
 * Chaîne en vrai HW :
 *   firmware afc_load_dsp() écrit dsp_api.db_w->d_afc = dac_value
 *   DSP lit d_afc et le serialise vers TWL3025 via TSP
 *   TWL3025 décode le register write → AFC DAC (13 bits, ±4096)
 *   DAC pilote VCXO 13 MHz → décalage fréquence baseband
 *
 * En QEMU :
 *   - Phase 1 (ce fichier) : API directe `calypso_twl3025_set_afc_dac()`
 *     pour test, et `calypso_twl3025_get_afc_phase_step()` pour BSP.
 *   - Phase 2 (TODO) : hook TSP byte stream pour décoder vraies écritures
 *     TWL3025 registers et extraire DAC value automatiquement.
 *   - Phase 3 (TODO) : convertir DAC → Hz via afc_slope du firmware
 *     (vcxocal Compal E88 standard) puis rotation samples.
 *
 * Pour tester : CALYPSO_TWL3025_AFC=1 active la rotation BSP.
 *               CALYPSO_TWL3025_AFC_HZ=N force un offset constant (diag).
 */

#include "qemu/osdep.h"
#include <math.h>

#include "hw/arm/calypso/calypso_twl3025.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* TWL3025 / Compal E88 vcxocal constants (from osmocom-bb rf/vcxocal.h) :
 *   afc_slope = 287 (= Hz per DAC LSB at GSM 900 reference)
 *   d_afc range : ±4095 (13-bit signed DAC value)
 *
 * Per firmware afc_correct() : delta = (afc_norm_factor * freq_error) / afc_slope.
 * Inverse : freq_hz = dac_value * afc_slope / afc_norm_factor.
 *
 * AFC_NORM_FACTOR (firmware afc.h) : GSM 900 = 0x0F38 = 3896.
 * Mais en pratique pour notre simulation, on prend afc_slope direct (= ~287 Hz/LSB).
 */
#define TWL3025_AFC_SLOPE_HZ_PER_LSB    287.0
#define GSM_SAMPLE_RATE_HZ              270833.0  /* 1 sps GSM baseband */

static struct {
    bool        enabled;
    bool        env_loaded;
    int         force_hz;   /* CALYPSO_TWL3025_AFC_HZ override (diag) */
    int16_t     dac_value;
    double      phase_accum; /* phase continue inter-bursts pour stabilité */
} twl;

static void twl3025_lazy_env(void)
{
    if (twl.env_loaded) return;
    twl.env_loaded = true;
    const char *e = getenv("CALYPSO_TWL3025_AFC");
    twl.enabled = (e && *e == '1');
    const char *h = getenv("CALYPSO_TWL3025_AFC_HZ");
    twl.force_hz = (h && *h) ? atoi(h) : 0;
    if (twl.enabled) {
        fprintf(stderr,
            "[twl3025] AFC propagation enabled (force_hz=%d, slope=%.1f Hz/LSB)\n",
            twl.force_hz, TWL3025_AFC_SLOPE_HZ_PER_LSB);
    }
}

void calypso_twl3025_set_afc_dac(int16_t dac_value)
{
    twl3025_lazy_env();
    if (dac_value > 4095) dac_value = 4095;
    if (dac_value < -4096) dac_value = -4096;
    if (twl.dac_value != dac_value) {
        twl.dac_value = dac_value;
        if (twl.enabled) {
            fprintf(stderr,
                "[twl3025] AFC DAC set to %d (= %.1f Hz offset)\n",
                dac_value, dac_value * TWL3025_AFC_SLOPE_HZ_PER_LSB);
        }
    }
}

int16_t calypso_twl3025_get_afc_dac(void)
{
    return twl.dac_value;
}

double calypso_twl3025_get_afc_hz(void)
{
    twl3025_lazy_env();
    if (!twl.enabled) return 0.0;
    if (twl.force_hz != 0) return (double)twl.force_hz;
    return (double)twl.dac_value * TWL3025_AFC_SLOPE_HZ_PER_LSB;
}

double calypso_twl3025_get_afc_phase_step(void)
{
    twl3025_lazy_env();
    if (!twl.enabled) return 0.0;
    double hz = calypso_twl3025_get_afc_hz();
    /* Phase step per sample = 2π * freq / fs.
     * Signe : VCXO + → osc UP → received baseband freq DOWN (down-conversion).
     * Donc on rotate samples par -phase_step pour compenser.
     * Pour AFC convergence : firmware ajoute delta DAC pour réduire freq_error,
     * donc samples doivent shifter de l'AFC appliqué pour que correlator voit
     * la convergence. */
    return -2.0 * M_PI * hz / GSM_SAMPLE_RATE_HZ;
}

/* GSM TDMA timing constants (at 1 sample/bit BSP rate)
 *   1 frame  = 4.615 ms = 1250 symbols = 1250 samples
 *   1 slot   = 156.25 symbols ≈ 156 samples (rounded down)
 * Pour notre BSP livrant 148 samples/burst, c'est l'echelle "1 sample = 1 bit"
 * de la chaîne TRX-IPC. */
#define SAMPLES_PER_FRAME   1250U
#define SAMPLES_PER_SLOT    156U

void calypso_twl3025_apply_phase(int16_t *iq_samples, int n_samples,
                                 uint32_t fn, uint8_t tn)
{
    twl3025_lazy_env();
    if (!twl.enabled) return;
    double step = calypso_twl3025_get_afc_phase_step();
    if (step == 0.0) return;

    /* Sample offset absolu depuis FN=0,TN=0 (= phase de reference du systeme).
     * Wrap implicite par la nature periodique de cos/sin — fn * 1250 peut
     * deborder uint64 jamais raisonnablement (fn 21-bit GSM hyperframe max). */
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
    /* phase_accum legacy keep zero, no longer used for cross-burst continuity */
    twl.phase_accum = 0.0;
}

void calypso_twl3025_reset(void)
{
    twl.dac_value = 0;
    twl.phase_accum = 0.0;
}
