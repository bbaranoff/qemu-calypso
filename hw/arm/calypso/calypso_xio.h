/*
 * calypso_xio.h — fenêtres XIO du DSP non modélisées ailleurs (API Control F900,
 * INTH du DSP FA00). Source : CAL207 §7.2.2, §11.3.5 (note), CAL000 §3.7.6.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_XIO_H
#define CALYPSO_XIO_H

#include <stdint.h>
#include <stdbool.h>

/* Rend true si PA appartient à l'une des fenêtres traitées ici. */
bool calypso_xio_misc(bool write, uint16_t pa, uint16_t *val, uint16_t pc);

/* true si le DSP a mis la RAM API en mode HOM (Host Only Mode, §9.1 bit 1) :
 * la fenetre API est alors reservee a l'ARM et au DMA, CAL000 §7.2.1. */
bool calypso_xio_api_hom(void);

#endif /* CALYPSO_XIO_H */
