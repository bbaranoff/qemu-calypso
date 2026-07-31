/*
 * calypso_dma.h — contrôleur DMA interne du TMS320C54x
 *
 * [2026-07-29] Pourquoi ce module existe.
 *
 *   Le firmware DSP empile ses demandes de transfert dans une file circulaire
 *   de 14 entrées (pointeurs `data[0x433e]` lecture / `data[0x433f]` écriture,
 *   producteur en `0xaa75`, consommateur en `0xaa87`). Mesuré le 2026-07-29 :
 *   le producteur empile, le pointeur de lecture n'avance JAMAIS, l'anneau se
 *   remplit, l'écriture rattrape la lecture et le firmware lève
 *   `DSP_ERR_DMA_PROG` (bit 3 de `data[0x3f92]`, publié dans `d_error_status`
 *   = `0x08d5`, imprimé par l'ARM : « DSP Error Status: 8 », 605 fois).
 *
 *   Les instructions de cette séquence sont TOUTES correctement émulées —
 *   `MVDM`, `MAR *ARn-0` et `BANZ` ont été vérifiés un par un, registres et
 *   cellules relevés au PC du test. Ce qui manque n'est pas du décodage, c'est
 *   un OBJET : le contrôleur DMA lui-même. Le modèle accepte la programmation
 *   (banc de sous-registres, `calypso_c54x.c`) mais n'exécute aucun transfert
 *   et ne lève jamais `DMAC0..5`. Rien ne signale donc la complétion, le
 *   consommateur n'a jamais de raison de dépiler, et la file sature.
 *
 * ⚠️ CONFLIT DE MAPPING, à trancher par la mesure.
 *   SPRU131 place  DMPREC=0x54  DMSA=0x55  DMSDI=0x56  DMSDN=0x57.
 *   `calypso_c54x.c` utilise 0x54 comme DMSA (décalage d'un registre) — la
 *   sonde DMAWATCH le signale déjà par son libellé « DMPREC?(modele:DMSA) ».
 *   Ce module implémente le mapping du MANUEL et reste OPTIONNEL
 *   (`CALYPSO_DMA=1`, défaut OFF) : tant qu'il est éteint, le décodage
 *   existant tourne inchangé et aucune régression n'est possible.
 *
 * ⚠️ CE QUI N'EST PAS MODÉLISÉ, et doit le rester tant que ce n'est pas mesuré :
 *   - la synchronisation par événement (DMSFC : McBSP, timer, ligne externe) —
 *     ici le transfert est fait EN BLOC au tick de trame, pas au rythme réel ;
 *   - les transferts vers/depuis l'espace PROGRAMME et l'espace I/O (DMMCR) ;
 *   - l'entrelacement des priorités entre canaux (DPRC) ;
 *   - le mode ABU (adressage circulaire matériel du canal).
 *   Chacun de ces points est une approximation ASSUMÉE, pas un oubli. Le seul
 *   objectif ici est de fermer la boucle « programme -> transfère -> signale ».
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_DMA_H
#define CALYPSO_DMA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct C54xState C54xState;

/* Adresses MMR selon SPRU131. */
#define C54X_DMPREC   0x0054   /* contrôle/priorité, bits 0..5 = canal actif */
#define C54X_DMSA     0x0055   /* sous-adresse */
#define C54X_DMSDI    0x0056   /* données, auto-incrémente DMSA */
#define C54X_DMSDN    0x0057   /* données, sans auto-incrément */

/* Sous-registres, 5 par canal (SPRU131 §DMA). Le banc existant de
 * calypso_c54x.c n'en compte que 4 (SRC/DST/CTR/MCR) — DMSFC manque, d'où
 * l'impossibilité de modéliser la synchronisation par événement. */
#define C54X_DMA_CANAUX   6

/* Le module a-t-il été activé ? Testé en ligne sur les chemins chauds. */
extern int calypso_dma_actif;

/* Lit la configuration (CALYPSO_DMA). Idempotent, appelé paresseusement. */
void calypso_dma_init(void);

/* Interception d'une écriture MMR. Rend true si le module l'a traitée — dans
 * ce cas l'appelant ne fait rien de plus. Rend false quand le module est
 * éteint, laissant le décodage historique s'appliquer. */
bool calypso_dma_mmr_write(C54xState *s, uint16_t addr, uint16_t val);

/* À appeler une fois par trame TDMA : exécute les canaux actifs et lève
 * l'interruption de fin de transfert. */
void calypso_dma_tick(C54xState *s);

#endif /* CALYPSO_DMA_H */
