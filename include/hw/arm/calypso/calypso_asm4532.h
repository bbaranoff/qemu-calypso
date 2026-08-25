/*
 * calypso_asm4532.h — modele du commutateur d'antenne ASM4532.
 *
 * Position dans la chaine (wiki osmocom TypicalCalypsoModemDesign) :
 *   antenne <-> ASM4532 <-> [SAW Rx -> TRF6151]  ou  [RF3166 -> TX]
 * C'est lui qui relie l'antenne SOIT a l'un des chemins Rx GSM/DCS/PCS,
 * SOIT a la sortie du PA pendant la duree du burst.
 *
 * Pilotage : lignes TSPACT, pas le TSP serie. Mapping porte tel quel de
 * osmocom-bb board/compal/rffe_dualband.c (le board C123 qu'on emule) :
 *   TRENA    = TSPACT(6)  Transmit Enable (Antenna Switch)  -- ACTIF BAS
 *   GSM_TXEN = TSPACT(8)  GSM (par opposition a DCS)        -- ACTIF BAS
 * et la sequence de rffe_mode() :
 *   tspact |= TRENA | GSM_TXEN;          (repos : les deux desassertees)
 *   si TX : tspact &= ~TRENA;            (assertion)
 *           si GSM900 : tspact &= ~GSM_TXEN;
 *
 * ⚠️ PORTEE : ce bloc est cote TX / selection de bande. Il n'a AUCUN effet sur
 *    l'acquisition FB/SB : avec un downlink synthetique, le chemin Rx est deja
 *    suppose connecte. Le modeliser sert (a) a rendre la sequence TX lisible,
 *    (b) a detecter une emission demandee alors que l'antenne n'est pas
 *    commutee — une faute silencieuse aujourd'hui.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_ASM4532_H
#define HW_ARM_CALYPSO_ASM4532_H

#include <stdint.h>
#include <stdbool.h>

/* Appele depuis calypso_tsp.c a chaque tsp_act_update() (ecriture TSP_ACT_L/U),
 * avec l'etat COMPLET des 16 lignes TSPACT. */
void calypso_asm4532_tspact_update(uint16_t tspact, uint32_t fn);

/* true = l'antenne est commutee sur la sortie du PA (TRENA assertee). */
bool calypso_asm4532_tx_connected(void);

/* true = bande GSM900 selectionnee pour l'emission (GSM_TXEN assertee),
 * false = DCS1800. N'a de sens que si tx_connected(). */
bool calypso_asm4532_band_gsm(void);

/* Compteurs pour les barrieres et le diagnostic. */
uint32_t calypso_asm4532_tx_windows(void);

#endif /* HW_ARM_CALYPSO_ASM4532_H */
