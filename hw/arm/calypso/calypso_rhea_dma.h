/*
 * calypso_rhea_dma.h — contrôleur DMA RHEA du Calypso, côté MCU (FFFF:FC00).
 * Source : CAL207 §11 (ti-calypso2.pdf).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_RHEA_DMA_H
#define CALYPSO_RHEA_DMA_H

#include "exec/hwaddr.h"
#include <stdint.h>
#include <stdbool.h>

#define CALYPSO_RHEA_DMA_BASE 0xFFFFFC00

uint64_t calypso_rhea_dma_read(void *opaque, hwaddr off, unsigned size);
void     calypso_rhea_dma_write(void *opaque, hwaddr off, uint64_t val, unsigned size);

/* Meme banc de registres vu du bus Rhea du DSP (XIO:FC00..FCFF, §11.1).
 * Rend true si PA appartient a la fenetre DMA. */
bool     calypso_rhea_dma_xio(bool write, uint16_t pa, uint16_t *val, uint16_t pc);

/* [2026-08-03] Requete de reception : le RIF signale qu'un burst est disponible
 * et que le firmware a choisi le mode DMA (RDMA_MASK=0). Le controleur vide
 * alors le recepteur vers la memoire API a l'adresse DMA2_AAD, pose IRQ_STATE
 * et leve INT10n si IRQ_MODE le demande.
 *
 * Gate `CALYPSO_RHEA_DMA_XFER` (defaut 0) : sans lui le module reste l'instrument
 * de lecture qu'il etait, il journalise sans rien transferer. C'est un changement
 * de NATURE (d'instrument a piece de materiel), donc opt-in d'abord. */
struct C54xState;
void     calypso_rhea_dma_rx_request(struct C54xState *s);

/* [2026-08-04] Niveau de la ligne INT10n. CAL000 §5.1 : « INT10n (level) -> DMA
 * interrupt ». La ligne reste assertee tant que le canal a son IRQ_STATE pose ;
 * c'est la LECTURE du registre qui l'efface (CAL207 §11.3.5). Sans ca le modele
 * traite l'IT comme un FRONT et la perd des qu'elle arrive avec INTM=1 — ce qui
 * est le cas 15 fois sur 15 dans les runs mesures. */
bool     calypso_rhea_dma_irq_level(void);

#endif /* CALYPSO_RHEA_DMA_H */
