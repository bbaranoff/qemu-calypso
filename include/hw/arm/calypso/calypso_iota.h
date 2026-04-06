/*
 * TWL3025 / IOTA — analog baseband chip model.
 *
 * Sits on the Calypso TSP serial bus (dev_idx 0). The L1 firmware programs
 * BDLON / BDLENA / BULON / BULENA via single-byte TSP writes; this model
 * tracks the resulting downlink/uplink window state so other QEMU
 * components (notably calypso_bsp) can gate sample DMA the way the real
 * BSP serial link is gated by IOTA's BDLENA pin.
 *
 * Reference: osmocom-bb src/target/firmware/include/abb/twl3025.h
 *            (BDLON/BDLENA/BULON/BULENA bit values).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_IOTA_H
#define HW_ARM_CALYPSO_IOTA_H

#include <stdbool.h>
#include <stdint.h>

/* Bit values written by twl3025_tsp_write() — straight from twl3025.h */
#define IOTA_TSP_BULON      0x80
#define IOTA_TSP_BULENA     0x40
#define IOTA_TSP_BULCAL     0x20
#define IOTA_TSP_BDLON      0x10
#define IOTA_TSP_BDLCAL     0x08
#define IOTA_TSP_BDLENA     0x04

void calypso_iota_init(void);

/* Process one TSP write (TPU sequencer feeds us). `data` is the 7-bit byte
 * the firmware passed to twl3025_tsp_write(). `expected_tn` is the GSM
 * timeslot the L1 has armed this BDLENA window for, derived by the TPU
 * sequencer from the AT instruction immediately preceding this MOVE. */
void calypso_iota_tsp_write(uint8_t data, uint8_t expected_tn);

/* True while the firmware has BDLENA asserted on IOTA. */
bool calypso_iota_bdl_ena(void);

/* Number of times BDLENA has been asserted (rising edge counter). */
uint32_t calypso_iota_bdl_ena_pulses(void);

/* Atomically take one bdl_ena rising-edge credit if it matches the burst's
 * timeslot. Returns true if a credit existed for this `tn` and was consumed,
 * false otherwise. */
bool calypso_iota_take_bdl_pulse(uint8_t tn);

#endif /* HW_ARM_CALYPSO_IOTA_H */
