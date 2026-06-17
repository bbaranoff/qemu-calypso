/*
 * calypso_dsp_shunt.h — public API for the DSP shunt mock.
 *
 * Active only when CALYPSO_DSP_SHUNT=1. When active, all code paths
 * that write into API RAM / DARAM (BSP DMA, ARM scenario-end DMA, etc)
 * must skip themselves to avoid trampling on the mock's responses.
 */

#ifndef CALYPSO_DSP_SHUNT_H
#define CALYPSO_DSP_SHUNT_H

#include "qemu/osdep.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include <stdbool.h>

/* Init at machine setup. Called from calypso.c. */
void calypso_dsp_shunt_init(MemoryRegion *system_memory, AddressSpace *as);

/* Called by calypso_trx frame_irq tick to service any pending task. */
void calypso_dsp_shunt_on_frame_tick(void);

/* True if CALYPSO_DSP_SHUNT=1 in env. Use to gate BSP/TPU DMA into DARAM. */
bool calypso_dsp_shunt_active(void);

/* Phase 2 future hook (IPC fed). */
void calypso_dsp_shunt_feed_fb_result(int found, int16_t toa,
                                      int16_t pm, int16_t angle, int16_t snr);

/* Injection du SI RÉEL (gr-gsm via pont, ou démod C native) : frame L2 23 o
 * décodée depuis l'I/Q réel du BTS, écrite dans a_cd à la place du SI3 canned.
 * Point d'injection commun aux deux fronts de démod. */
void calypso_dsp_shunt_feed_si(const uint8_t *l2, int len);

/* ENTREE du DSP shunte : la BSP pousse l'I/Q DL (cs16, n int16 entrelaces
 * I,Q) dans le buffer shm pour que gr-gsm (le DSP) la lise et la decode. */
void calypso_dsp_shunt_feed_iq(uint32_t fn, const int16_t *iq, int n);

/* SONDE B : enregistre la FN TDMA reelle par RA lors dun RACH UL. */
void calypso_dsp_shunt_record_rach(uint8_t ra);

#endif /* CALYPSO_DSP_SHUNT_H */
