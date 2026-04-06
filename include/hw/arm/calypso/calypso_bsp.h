/*
 * Calypso BSP/RIF DMA — public interface.
 *
 * Faithful path for downlink I/Q samples between sercomm_gate (the QEMU
 * surrogate of the IOTA RF frontend wired through bridge.py) and the
 * Calypso DSP DARAM. No NDB result hacking — the DSP code itself is
 * expected to find FB/SB and post results in the NDB.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_BSP_H
#define HW_ARM_CALYPSO_BSP_H

#include <stdint.h>

struct C54xState;

/*
 * Initialise the BSP DMA module. Call once after the C54x has been created.
 *
 * Two env vars control the DMA destination:
 *   CALYPSO_BSP_DARAM_ADDR — word address inside DSP data space (hex/dec)
 *   CALYPSO_BSP_DARAM_LEN  — max number of int16 words to copy per burst
 *
 * When CALYPSO_BSP_DARAM_ADDR is unset/zero the BSP runs in DISCOVERY mode:
 * it logs every received burst but writes nothing into DARAM. This lets the
 * c54x.c FBDET data-read tracer reveal the real buffer location before we
 * lock the address.
 */
void calypso_bsp_init(struct C54xState *dsp);

/*
 * Receive a downlink burst.
 *
 *   tn       — timeslot number (0..7)
 *   fn       — TDMA frame number
 *   iq       — interleaved int16 I,Q,I,Q,... in DSP-native (host) endianness
 *   n_int16  — number of int16 elements in iq[]  (= 2 * n_complex_samples)
 */
void calypso_bsp_rx_burst(uint8_t tn, uint32_t fn,
                          const int16_t *iq, int n_int16);

#endif /* HW_ARM_CALYPSO_BSP_H */
