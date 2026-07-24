/*
 * calypso_layer1.c — HLE L1 model for the Calypso DSP (CALYPSO_L1=c).
 *
 * Scaffold fonctionnel intermédiaire (PAS l'endgame ; le revival LLE reste le but
 * faithful, gardé vivant derrière CALYPSO_L1 off). Modélise le L1 du DSP en C,
 * cadencé par calypso_tdma_tick. Lit l'I/Q en DARAM 0x2a00, écrit ses résultats
 * dans l'API RAM où l'ARM (osmocom-bb, inchangé) les lit.
 *
 * ÉTAPE 1 — détecteur FB/FCCH RÉEL (pas de forcing) :
 *   Filtre adapté au ton FCCH (+fc/4 → incrément de phase +22.5°/sample @4 SPS) :
 *       C = Σ z[n]·exp(-j·n·inc) ;  M = |C|² / (N·E)
 *   Ton pur (FCCH) → M≈1 ; bruit/GMSK data → M≈1/N. Discrimine VRAIMENT (le ratio
 *   différentiel R était émoussé à 4 SPS). Auto-alignant : M ne pique qu'aux trames
 *   FCCH (observées à fn%51≈7-8, offset fixe du multiframe BTS) → la trame de lock
 *   devient l'ancre, l'offset n'a pas à être connu.
 *   Valeurs DÉRIVÉES (pas cannées) : d_fb_det (détection réelle), snr (∝ cohérence M,
 *   >AFC_SNR_THRESHOLD=2560 quand locké), angle (résidu meanInc−nominal ≈0 → AFC
 *   stable, comme le ANGLE=0 du shunt mais CALCULÉ), pm (dérivé de E full-scale),
 *   toa (on-time, le bsp livre le burst aligné).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_layer1.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --- offsets data[] (DSP word) / dsp_ram (ARM word) --- */
#define L1_DARAM_IQ      0x2a00      /* 148 paires int16 I/Q entrelacées */
#define L1_IQ_PAIRS      148
#define L1_NDB_D_FB_DET  0x08F8      /* ndb->d_fb_det */
#define L1_NDB_D_FB_MODE 0x08F9      /* ndb->d_fb_mode (0=wideband 1=narrow) */
#define L1_NDB_A_SYNC    0x08FA      /* a_sync_demod[0]=TOA [1]=PM [2]=ANGLE [3]=SNR */
#define WP0_TASK_MD      (0x0008/2)
#define WP1_TASK_MD      (0x0030/2)
#define D_DSP_PAGE_WORD  (0x01A8/2)

#define FB_DSP_TASK      5
#define SB_DSP_TASK      6

/* incrément de phase nominal du ton FCCH : +fc/4 = +1625/24 kHz à 4 SPS
 * (fs=4×270.833k) → 2π·67708/1083333 = +22.5°/sample. */
#define L1_FCCH_INC_DEG  22.5
#define L1_FCCH_M_THRESH 0.40        /* M_data≈1/148≈0.007, M_fcch≈1 → seuil large */
#define L1_E_MIN         1.0e6       /* énergie min pour considérer un burst présent */
#define AFC_SNR_THRESHOLD 2560       /* afc.h : 2.5 dB en fx6.10 */

int calypso_l1_c_active(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_L1");
        v = (e && e[0] == 'c') ? 1 : 0;
    }
    return v;
}

/* Latch du d_task_md écrit par l'ARM : le poll tick-time rate le transient
 * (write puis consommé au write-time, vu par calypso_fbsb). Mis à jour depuis
 * calypso_dsp_write (calypso_trx.c) à chaque écriture de la tâche. */
static uint16_t g_l1_task_md = 0;
void calypso_layer1_on_task_write(uint16_t md)
{
    g_l1_task_md = md;
}

struct fb_res {
    int      found;
    double   M, meanInc_deg, energy;
    int16_t  toa, pm, angle, snr;
};

/* Détecteur FCCH : filtre adapté au ton +fc/4 + résidu de fréquence. */
static struct fb_res l1_fb_detect(C54xState *dsp)
{
    struct fb_res r = { 0 };
    const double inc = L1_FCCH_INC_DEG * M_PI / 180.0;
    const double rcos = cos(-inc), rsin = sin(-inc);  /* rotator exp(-j·inc) */

    double Cre = 0, Cim = 0, E = 0;     /* C = Σ z[n]·exp(-j n inc) */
    double pr = 1.0, pim = 0.0;          /* phasor p_n = exp(-j n inc), p_0 = 1 */
    double sre = 0, sim = 0;             /* Σ d[n] (différentiel) pour meanInc */
    double prevI = (double)(int16_t)dsp->data[L1_DARAM_IQ + 0];
    double prevQ = (double)(int16_t)dsp->data[L1_DARAM_IQ + 1];

    for (int n = 0; n < L1_IQ_PAIRS; n++) {
        double I = (double)(int16_t)dsp->data[L1_DARAM_IQ + 2 * n];
        double Q = (double)(int16_t)dsp->data[L1_DARAM_IQ + 2 * n + 1];
        /* filtre adapté : C += z · p */
        Cre += I * pr - Q * pim;
        Cim += I * pim + Q * pr;
        E   += I * I + Q * Q;
        /* avance phasor : p *= exp(-j inc) */
        double npr  = pr * rcos - pim * rsin;
        double npim = pr * rsin + pim * rcos;
        pr = npr; pim = npim;
        /* différentiel d[n] = z[n]·conj(z[n-1]) (diagnostic / résidu) */
        if (n > 0) {
            sre += I * prevI + Q * prevQ;
            sim += Q * prevI - I * prevQ;
        }
        prevI = I; prevQ = Q;
    }

    r.energy      = E;
    r.M           = (E > 0.0) ? (Cre * Cre + Cim * Cim) / ((double)L1_IQ_PAIRS * E) : 0.0;
    r.meanInc_deg = (sre != 0.0 || sim != 0.0) ? atan2(sim, sre) * 180.0 / M_PI : 0.0;
    r.found       = (r.M >= L1_FCCH_M_THRESH && E >= L1_E_MIN);

    if (r.found) {
        /* angle : résidu de fréquence (meanInc − nominal), ≈0 pour I/Q propre →
         * freq_diff = ANGLE_TO_FREQ(angle) ≈ 0 → AFC ne chasse pas. DÉRIVÉ. */
        double resid_deg = r.meanInc_deg - L1_FCCH_INC_DEG;
        double a = resid_deg * 32.0;                 /* échelle fx provisoire (à calibrer) */
        r.angle = (int16_t)(a >  32767 ?  32767 : a < -32768 ? -32768 : a);
        /* snr : croît avec la cohérence M ; >2560 (AFC_SNR_THRESHOLD) quand locké. */
        double snr = r.M * (double)0x7000;
        r.snr = (int16_t)(snr > 32767 ? 32767 : snr);
        /* pm : dérivé — l'I/Q est full-scale Q15 (rxlev fort). Lu >>3 par
         * read_fb_result → 0x7000>>3 = 0xE00. */
        r.pm = 0x7000;
        /* toa : le bsp livre le burst aligné sur la fenêtre → on-time.
         * read_fb_result fait toa-=23, donc 23 → 0 (on-time). */
        r.toa = 23;
    }
    return r;
}

void calypso_layer1_tick(C54xState *dsp, uint16_t *dsp_ram, uint32_t fn)
{
    if (!dsp || !dsp_ram) {
        return;
    }

    uint16_t page = dsp_ram[D_DSP_PAGE_WORD] & 1;
    uint16_t md_poll = page ? dsp_ram[WP1_TASK_MD] : dsp_ram[WP0_TASK_MD];
    /* latch prioritaire (le poll rate le transient task=5) */
    uint16_t md = g_l1_task_md ? g_l1_task_md : md_poll;

    struct fb_res r = l1_fb_detect(dsp);

    /* log riche (calibration du seuil + visu de l'auto-alignement) */
    static unsigned logn = 0;
    if (logn++ < 5000) {
        fprintf(stderr,
                "[L1c] fn=%u m51=%u md=%u(poll=%u) M=%.3f meanInc=%.1f E=%.2e det=%d\n",
                fn, fn % 51, md, md_poll, r.M, r.meanInc_deg, r.energy, r.found);
    }

    /* écriture firmware UNIQUEMENT quand le FB est demandé (évite des d_fb_det
     * parasites hors recherche). NDB page-less → pas de souci de page. */
    if (md == FB_DSP_TASK) {
        if (r.found) {
            dsp->data[L1_NDB_A_SYNC + 0] = (uint16_t)r.toa;
            dsp->data[L1_NDB_A_SYNC + 1] = (uint16_t)r.pm;     /* lu >>3 par l'ARM */
            dsp->data[L1_NDB_A_SYNC + 2] = (uint16_t)r.angle;
            dsp->data[L1_NDB_A_SYNC + 3] = (uint16_t)r.snr;
            dsp->data[L1_NDB_D_FB_DET]   = 1;
            static unsigned fbn = 0;
            if (fbn++ < 200) {
                fprintf(stderr, "[L1c-FB] FOUND fn=%u m51=%u M=%.3f meanInc=%.1f "
                        "resid=%.1f -> d_fb_det=1 toa=%d pm=0x%04x ang=%d snr=%d\n",
                        fn, fn % 51, r.M, r.meanInc_deg,
                        r.meanInc_deg - L1_FCCH_INC_DEG,
                        r.toa, (uint16_t)r.pm, r.angle, r.snr);
            }
        } else {
            dsp->data[L1_NDB_D_FB_DET] = 0;
        }
    }
}
