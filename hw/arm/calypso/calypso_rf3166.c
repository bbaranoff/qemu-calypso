/*
 * calypso_rf3166.c — amplificateur de puissance RF3166.
 *
 * Cf. hw/arm/calypso/doc/CHAINE_RF_MATERIELLE.md §3 et calypso_rf3166.h.
 * Enable = PA_ENABLE = TSPACT(1), ACTIF HAUT (rffe_dualband.c).
 * Sequence de rffe_mode() : au repos `tspact &= ~PA_ENABLE` ; en emission
 * `tspact &= ~TRENA` (commutateur) puis `tspact |= PA_ENABLE` — donc le PA
 * s'allume APRES que l'antenne a ete commutee. C'est cet ordre qu'on verifie.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include "hw/arm/calypso/calypso_rf3166.h"
#include "hw/arm/calypso/calypso_asm4532.h"
#include "hw/arm/calypso/calypso_debug.h"

#define PA_ENABLE   (1u << 1)   /* TSPACT(1), actif HAUT */

#define PA_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[rf3166] " fmt "\n", ##__VA_ARGS__); } while (0)

/* Modele de rampe : 0..255 -> 0..33 dBm. ⚠️ INVENTE, pas calibre. */
#define RF3166_MAX_DBM  33

static struct {
    bool     on;
    bool     apc_known;
    uint8_t  apc;
    uint32_t bursts;        /* nombre d'activations */
    uint32_t faults;        /* activations sans commutateur en position TX */
} pa;

void calypso_rf3166_tspact_update(uint16_t tspact, uint32_t fn)
{
    bool on_new = !!(tspact & PA_ENABLE);

    if (on_new == pa.on) {
        return;
    }

    if (on_new) {
        pa.bursts++;
        /* Recoupement avec le commutateur : le firmware commute AVANT
         * d'allumer. Si l'antenne n'est pas sur le PA, on emet dans le vide. */
        if (!calypso_asm4532_tx_connected()) {
            pa.faults++;
            fprintf(stderr, "[rf3166] ⚠ PA_ENABLE assertee alors que l'antenne "
                    "n'est PAS commutee sur le PA (TRENA relachee) — fn=%u, "
                    "faute #%u\n", fn, pa.faults);
        } else {
            PA_LOG("PA ON  (bande %s) fn=%u  [burst #%u]",
                   calypso_asm4532_band_gsm() ? "GSM900" : "DCS1800",
                   fn, pa.bursts);
        }
    } else {
        PA_LOG("PA OFF fn=%u", fn);
    }

    pa.on = on_new;
}

void calypso_rf3166_set_apc(uint8_t apc_level)
{
    pa.apc = apc_level;
    pa.apc_known = true;
}

bool calypso_rf3166_enabled(void) { return pa.on; }

int32_t calypso_rf3166_out_dbm(void)
{
    if (!pa.on || !pa.apc_known) {
        return INT32_MIN;   /* eteint, ou APC jamais pose : inconnu */
    }
    return ((int32_t)pa.apc * RF3166_MAX_DBM) / 255;
}

uint32_t calypso_rf3166_faults(void) { return pa.faults; }
