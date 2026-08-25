/*
 * calypso_rf3166.h — modele de l'amplificateur de puissance RF3166.
 *
 * Position dans la chaine (doc/CHAINE_RF_MATERIELLE.md §3) :
 *   TRF6151 --RF--> RF3166 --RF--> ASM4532 --> antenne
 * Le wiki dit : « amplifies the signal according to the analog level of the APC »
 * — l'APC (Automatic Power Correction) est l'enveloppe de puissance produite par
 * l'ABB (TWL3025), en ANALOGIQUE. Le PA n'a donc que deux entrees utiles pour
 * nous : son enable numerique, et ce niveau analogique.
 *
 * Enable : ligne TSPACT, porte de osmocom-bb board/compal/rffe_dualband.c :
 *   PA_ENABLE = TSPACT(1)   -- ACTIF HAUT
 *
 * ⚠️ PORTEE : bloc purement TX. Il n'est meme PAS dans le chemin Rx, donc il ne
 *    peut rien expliquer de l'acquisition FB/SB. Ce qu'il apporte : rendre la
 *    sequence d'emission verifiable, et surtout permettre le RECOUPEMENT avec le
 *    commutateur d'antenne — un PA actif alors que l'antenne n'est pas commutee
 *    sur lui est une faute que rien ne detectait jusqu'ici.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_RF3166_H
#define HW_ARM_CALYPSO_RF3166_H

#include <stdint.h>
#include <stdbool.h>

/* Appele depuis calypso_tsp.c a chaque tsp_act_update(), avec l'etat COMPLET
 * des 16 lignes TSPACT. */
void calypso_rf3166_tspact_update(uint16_t tspact, uint32_t fn);

/* Niveau APC, a appeler par le modele d'ABB le jour ou il le produit.
 * Unite : celle du DAC APC de l'ABB (0..255), 0 = pas d'emission.
 * Tant que personne n'appelle, le niveau reste INCONNU et out_dbm() le dit. */
void calypso_rf3166_set_apc(uint8_t apc_level);

/* true = PA_ENABLE assertee. */
bool calypso_rf3166_enabled(void);

/* Puissance de sortie estimee, en dBm. Renvoie INT32_MIN si l'APC n'a jamais
 * ete pose (= inconnu) ou si le PA est eteint. ⚠️ MODELE, pas mesure : rampe
 * lineaire 0..255 -> 0..33 dBm, a remplacer par une vraie table de calibration
 * le jour ou l'UL est mesuree. */
int32_t calypso_rf3166_out_dbm(void);

/* Nombre de fois ou le PA a ete active alors que l'antenne n'etait PAS commutee
 * sur lui (recoupement avec calypso_asm4532). Doit rester a 0. */
uint32_t calypso_rf3166_faults(void);

#endif /* HW_ARM_CALYPSO_RF3166_H */
