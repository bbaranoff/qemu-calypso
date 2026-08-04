/*
 * calypso_rif.h — Radio InterFace (RIF) du Calypso, vue DSP (espace XIO).
 *
 * Source : CAL207 « Register Mapping » §12 (ti-calypso2.pdf) + CAL000 §3.7.1
 * et §5.1 (ti-calypso1.pdf).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_RIF_H
#define CALYPSO_RIF_H

#include <stdint.h>
#include <stdbool.h>
#include "hw/arm/calypso/calypso_c54x.h"

/* Adresses XIO du §12.1 (Table 19). */
#define RIF_XIO_DXR    0x0000   /* Transmit Data Register   — 16b R/W        */
#define RIF_XIO_DRR    0x0001   /* Receive Data Register    — 16b R          */
#define RIF_XIO_SPCX   0x0002   /* Control Register (TX)    — 15b, rst 0x059E */
#define RIF_XIO_SPCR   0x0003   /* Control Register (RX)    — 15b, rst 0x3CA2 */

/* true si le module est arme (CALYPSO_RIF_XIO, defaut 1). */
bool calypso_rif_on(void);

/* PORTR PA,Smem / PORTW Smem,PA : rendent true si PA appartient au RIF.
 * portr renseigne *out avec la valeur lue. */
bool calypso_rif_portr(C54xState *s, uint16_t pa, uint16_t *out);
bool calypso_rif_portw(C54xState *s, uint16_t pa, uint16_t val);

/* Un burst arrive du front radio : les mots sont mis en attente et coulent
 * dans la FIFO de reception a mesure que le DSP lit DRR. */
void calypso_rif_rx_burst(C54xState *s, const uint16_t *w, int n);

/* [2026-08-03] Drainage par le DMA. En mode DMA (RDMA_MASK=0), ce n'est pas le
 * DSP qui lit DRR mot a mot : le controleur RHEA vide le recepteur vers la
 * memoire API (CAL000 §3.7.1, « an end-DMA request is sent »). Cette fonction
 * est le seul chemin par lequel calypso_rhea_dma.c preleve les echantillons —
 * la FIFO et l'etage d'attente restent prives au RIF.
 *
 * Rend le nombre de mots reellement copies (0 si le recepteur est vide), et
 * realimente la FIFO depuis l'etage d'attente, exactement comme le fait la
 * lecture de DRR. Aucun effet de bord sur SPCR : c'est l'appelant qui decide
 * de la notification, pas le RIF. */
int calypso_rif_drain(uint16_t *dst, int max);

#endif /* CALYPSO_RIF_H */
