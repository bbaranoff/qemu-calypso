/*
 * calypso_asm4532.c — commutateur d'antenne ASM4532.
 *
 * Cf. hw/arm/calypso/doc/CHAINE_RF_MATERIELLE.md §2.1 et §3.
 * Piloté par les lignes TSPACT, pas par le TSP série. Mapping porté tel quel de
 * osmocom-bb board/compal/rffe_dualband.c (board C123) :
 *   TRENA    = TSPACT(6)  Transmit Enable (Antenna Switch)  -- ACTIF BAS
 *   GSM_TXEN = TSPACT(8)  GSM (par opposition a DCS)        -- ACTIF BAS
 *
 * Ce modele est PASSIF : il latche l'etat, compte les fenetres, et signale les
 * incoherences. Il ne gate rien — le downlink est synthetique, donc gater le Rx
 * sur la position du commutateur ne ferait que casser des bancs qui marchent,
 * sans rien mesurer de plus. Il devient utile le jour ou l'UL est reellement
 * emise : c'est lui qui dira si l'antenne etait commutee pendant le burst.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include "hw/arm/calypso/calypso_asm4532.h"
#include "hw/arm/calypso/calypso_debug.h"

/* Lignes TSPACT, portees du firmware (TSPACT(n) = 1 << n). */
#define ASM_TRENA       (1u << 6)   /* actif BAS */
#define ASM_GSM_TXEN    (1u << 8)   /* actif BAS */

#define ASM_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[asm4532] " fmt "\n", ##__VA_ARGS__); } while (0)

static struct {
    bool     init;
    uint16_t tspact;        /* dernier etat vu */
    bool     tx;            /* TRENA assertee  -> antenne sur le PA */
    bool     gsm;           /* GSM_TXEN asserte -> bande GSM900 */
    uint32_t tx_windows;    /* nombre de fenetres TX ouvertes */
} asm4532;

void calypso_asm4532_tspact_update(uint16_t tspact, uint32_t fn)
{
    bool tx_new, gsm_new;

    /* Actif BAS : la ligne est ASSERTEE quand le bit est a 0. */
    tx_new  = !(tspact & ASM_TRENA);
    gsm_new = !(tspact & ASM_GSM_TXEN);

    if (!asm4532.init) {
        asm4532.init = true;
        ASM_LOG("etat initial : tspact=0x%04x TX=%d bande=%s fn=%u",
                tspact, tx_new, gsm_new ? "GSM900" : "DCS1800", fn);
    }

    if (tx_new != asm4532.tx) {
        if (tx_new) {
            asm4532.tx_windows++;
        }
        ASM_LOG("TRENA %s -> antenne sur %s (bande %s) fn=%u  [fenetre #%u]",
                tx_new ? "ASSERTEE" : "relachee",
                tx_new ? "le PA (TX)" : "le chemin Rx",
                gsm_new ? "GSM900" : "DCS1800", fn, asm4532.tx_windows);
    } else if (gsm_new != asm4532.gsm && tx_new) {
        /* Changement de bande PENDANT une fenetre TX : le firmware ne fait pas
         * ca (rffe_mode pose les deux ensemble). On le signale plutot que de
         * l'absorber en silence. */
        ASM_LOG("⚠ bande changee PENDANT la fenetre TX : %s -> %s fn=%u",
                asm4532.gsm ? "GSM900" : "DCS1800",
                gsm_new ? "GSM900" : "DCS1800", fn);
    }

    asm4532.tspact = tspact;
    asm4532.tx     = tx_new;
    asm4532.gsm    = gsm_new;
}

bool     calypso_asm4532_tx_connected(void) { return asm4532.tx; }
bool     calypso_asm4532_band_gsm(void)     { return asm4532.gsm; }
uint32_t calypso_asm4532_tx_windows(void)   { return asm4532.tx_windows; }
