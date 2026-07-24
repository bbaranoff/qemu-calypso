/*
 * calypso_tsp.h -- Calypso DBB internal TSP (Time Serial Port) device layer
 *
 * Mirrors osmocom-bb-transceiver's src/target/firmware/calypso/tsp.c: the
 * TSP protocol built on top of raw TPU MOVE instructions (tpu_enq_move)
 * to talk to companion chips (TWL3025/Iota, TRF6151 RF frontend, ...)
 * over the Calypso's internal serial bus. The TPU sequencer
 * (calypso_tpu.c) decodes AT/WAIT/SYNCHRO/OFFSET/MOVE timing; MOVE
 * instructions whose address falls in the TSP device register range are
 * handed off here for protocol-level decoding (multi-byte TX assembly,
 * dev_idx/bitlen extraction, TSPACT enable-line tracking).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_TSP_H
#define CALYPSO_TSP_H

#include <stdint.h>

/* TSP device register addresses, reached via tpu_enq_move(addr, data) --
 * osmocom-bb-transceiver's include/calypso/tpu.h `enum tpu_reg_int`,
 * verbatim (the subset owned by the TSP protocol layer; TPUI_DSP_INT_PG
 * and TPUI_GAUGING_EN are handled directly by calypso_tpu.c). */
#define TPUI_TSP_CTRL1   0x00
#define TPUI_TSP_CTRL2   0x01
#define TPUI_TX_3        0x02
#define TPUI_TX_2        0x03
#define TPUI_TX_1        0x04
#define TPUI_TX_4        0x05
#define TPUI_TSP_ACT_L   0x06
#define TPUI_TSP_ACT_U   0x07
#define TPUI_TSP_SET1    0x09
#define TPUI_TSP_SET2    0x0a
#define TPUI_TSP_SET3    0x0b

#define TPUI_CTRL2_RD    (1 << 0)
#define TPUI_CTRL2_WR    (1 << 1)

/* True if `addr` belongs to the TSP device register range (as opposed to
 * TPU-native addresses like TPUI_DSP_INT_PG/TPUI_GAUGING_EN). */
bool calypso_tsp_owns_addr(uint8_t addr);

/* Feed one decoded MOVE(addr, data) into the TSP protocol state machine.
 * Only valid for addr where calypso_tsp_owns_addr() is true. On a CTRL2
 * WR pulse, reconstructs the full multi-byte payload (tsp_write()'s
 * MSB-first TX_1..TX_4 assembly per bitlen) and dispatches it: dev_idx==0
 * (TWL3025) goes to calypso_iota_tsp_write(); other devices have no
 * downstream hardware model yet and are logged instead of silently
 * dropped. */
void calypso_tsp_move(uint8_t addr, uint8_t data, uint32_t fn);

#endif /* CALYPSO_TSP_H */
