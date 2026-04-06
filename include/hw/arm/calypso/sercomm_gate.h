/*
 * sercomm_gate.h — Sercomm DLCI router (HDLC demux)
 *
 * Emulates the hardware separation between:
 *   - BSP path (bursts) : DLCI 4 → calypso_trx_rx_burst → c54x BSP
 *   - UART path (L1CTL) : all other DLCIs → re-wrap → FIFO → firmware
 *
 * On real Calypso, bursts come from the ABB via BSP hardware, not UART.
 * In QEMU, the bridge sends them as sercomm DLCI 4 on the PTY, so the
 * gate intercepts them and routes to the BSP emulation layer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef SERCOMM_GATE_H
#define SERCOMM_GATE_H

#include "hw/arm/calypso/calypso_uart.h"

/* Feed raw bytes from PTY into the sercomm HDLC parser.
 * Complete frames are routed by DLCI:
 *   4 (bursts) → calypso_trx_rx_burst (BSP emulation)
 *   * (L1CTL, debug, console) → re-wrap → UART FIFO (firmware) */
void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size);

/* Bind UDP TRX endpoints (CLK/TRXC/TRXD) starting at base_port. */
void sercomm_gate_init(int base_port);

#endif /* SERCOMM_GATE_H */
