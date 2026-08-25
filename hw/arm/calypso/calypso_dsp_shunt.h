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
/* le shunt REMPLACE le DSP (mock) — seul ce predicat doit gater les c54x_run */
bool calypso_dsp_shunt_substitutes(void);
bool calypso_dsp_shunt_sb_valid(void);  /* gr-gsm a decode la SCH (detection reelle) */
bool calypso_dsp_shunt_si_valid(void);  /* gr-gsm a decode un SI (a_cd pret) */
uint16_t calypso_dsp_shunt_burst_d(void);  /* d_burst_d echote (gates BURST_OFS/FN) */
/* Mission courante du DSP (d_task_md) : FB=5 SB=6 TCH_FB=8 TCH_SB=9, 0=aucune.
 * Sert a gater les wires inter-blocs (BSP BRINT0) sur la mission FB/SB. */
uint16_t calypso_dsp_shunt_get_task_md(void);

/* Modèle intégrateur RSSI HW : a_pm calibré depuis la vraie magnitude MAV du DL
 * (g_shunt.last_pm). Vrai pm_meas natif (voir calypso_dsp_shunt.c). */
uint16_t calypso_dsp_shunt_rssi_apm(void);

/* Phase 2 future hook (IPC fed). */
void calypso_dsp_shunt_feed_fb_result(int found, int16_t toa,
                                      int16_t pm, int16_t angle, int16_t snr);

/* Injection du SI RÉEL (gr-gsm via pont, ou démod C native) : frame L2 23 o
 * décodée depuis l'I/Q réel du BTS, écrite dans a_cd à la place du SI3 canned.
 * Point d'injection commun aux deux fronts de démod. */
void calypso_dsp_shunt_feed_si(const uint8_t *l2, int len);
/* [2026-07-22] Mirror d_burst_d PAR COMMANDE (de-alias) : appele depuis
 * calypso_dsp_write a chaque write ARM de la write-page. off = MMIO offset. */
void calypso_dsp_shunt_wp_burst_write(uint32_t off, uint16_t value);

/* ENTREE du DSP shunte : la BSP pousse l'I/Q DL (cs16, n int16 entrelaces
 * I,Q) dans le buffer shm pour que gr-gsm (le DSP) la lise et la decode. */
void calypso_dsp_shunt_feed_iq(uint32_t fn, const int16_t *iq, int n);
bool calypso_dsp_shunt_fb_stream_next(uint16_t *outI, uint16_t *outQ); /* FB-STREAM */

/* [2026-07-22] Injection READ-SIDE des resultats FB/SB REELS (gate
 * CALYPSO_SHUNT_REAL_FB) : appelee depuis calypso_dsp_read sur le read MMIO
 * ARM, immunise contre l'ordonnancement intra-trame (l'ARM lit TOUJOURS la
 * derniere detection reelle). off = ARM byte offset dans l'API RAM.
 * Retourne true si elle a override *out. */
bool calypso_dsp_shunt_real_fb_read(uint32_t off, uint16_t *out);

/* SONDE B : enregistre la FN TDMA reelle par RA lors dun RACH UL. */
void calypso_dsp_shunt_record_rach(uint8_t ra);

/* CALYPSO_DSP=c54x : relie le handle du VRAI DSP au shunt (type opaque pour
 * ne pas tirer calypso_c54x.h). Appelé depuis calypso_mb.c après l'init. */
struct C54xState;
void calypso_dsp_shunt_l1_reset(void);
void calypso_dsp_shunt_set_c54x(struct C54xState *s);
bool calypso_dsp_shunt_route_c54x_active(void);
bool calypso_dsp_shunt_early_booted(void);

#endif /* CALYPSO_DSP_SHUNT_H */
