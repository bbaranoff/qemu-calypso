/*
 * sercomm_gate.c — Sercomm DLCI router (HDLC demux)
 *
 * On real Calypso hardware, two separate paths exist:
 *   1. BSP (Baseband Serial Port) — radio bursts from ABB → DSP via PORTR 0xF430
 *   2. UART sercomm — L1CTL commands from host → firmware ARM via DLCI callbacks
 *
 * In QEMU, the bridge sends everything on one PTY:
 *   - Bursts as sercomm DLCI 4 (emulating BSP path)
 *   - L1CTL as sercomm DLCI 5 (real sercomm path)
 *
 * This gate separates them:
 *   DLCI 4 → calypso_trx_rx_burst() → c54x_bsp_load() + BRINT0  (BSP emulation)
 *   Others → re-wrap HDLC → UART RX FIFO → firmware sercomm layer (real path)
 *
 * Frame format (per osmocom sercomm.c):
 *   FLAG(7E) | DLCI(1) | CTRL(03) | DATA(N) | FLAG(7E)
 *   Escaping: 7E, 7D, 00 → 7D + (byte ^ 0x20)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/sercomm_gate.h"

#define SERCOMM_FLAG   0x7E
#define SERCOMM_ESCAPE 0x7D
#define SERCOMM_XOR    0x20

#define DLCI_BURST     4   /* SC_DLCI_DEBUG in osmocom — used for BSP emulation */

#define GATE_BUF_SIZE  512

/* HDLC parser state — matches osmocom sercomm.c rx_state */
enum gate_rx_state {
    GATE_WAIT_FLAG,   /* waiting for opening 0x7E */
    GATE_IN_FRAME,    /* collecting frame bytes */
    GATE_ESCAPE,      /* next byte is escaped (XOR 0x20) */
};

static uint8_t  sc_buf[GATE_BUF_SIZE];
static int      sc_len;
static enum gate_rx_state sc_state;

/*
 * Re-wrap a decoded frame and push into UART RX FIFO.
 * The firmware's sercomm_drv_rx_char() will parse it again — this is
 * intentional: it preserves the real firmware code path exactly.
 */
static void gate_push_to_fifo(CalypsoUARTState *s,
                              const uint8_t *frame, int len)
{
    calypso_uart_fifo_push(s, SERCOMM_FLAG);
    for (int i = 0; i < len; i++) {
        uint8_t c = frame[i];
        if (c == SERCOMM_FLAG || c == SERCOMM_ESCAPE || c == 0x00) {
            calypso_uart_fifo_push(s, SERCOMM_ESCAPE);
            calypso_uart_fifo_push(s, c ^ SERCOMM_XOR);
        } else {
            calypso_uart_fifo_push(s, c);
        }
    }
    calypso_uart_fifo_push(s, SERCOMM_FLAG);
}

/*
 * Dispatch a complete decoded sercomm frame.
 * frame[0] = DLCI, frame[1] = CTRL (0x03), frame[2..] = payload
 */
static void gate_dispatch(CalypsoUARTState *s,
                          const uint8_t *frame, int len)
{
    if (len < 2) return;

    uint8_t dlci = frame[0];

    if (dlci == DLCI_BURST) {
        /* BSP emulation path — skip DLCI(1) + CTRL(1) header */
        calypso_trx_rx_burst(&frame[2], len - 2);
        return;
    }

    /* All other DLCIs → firmware via FIFO (L1CTL, debug, console, echo) */
    gate_push_to_fifo(s, frame, len);
}

void sercomm_gate_feed(CalypsoUARTState *s, const uint8_t *buf, int size)
{
    for (int i = 0; i < size; i++) {
        uint8_t b = buf[i];

        switch (sc_state) {
        case GATE_WAIT_FLAG:
            if (b == SERCOMM_FLAG) {
                sc_state = GATE_IN_FRAME;
                sc_len = 0;
            } else {
                /* Pre-sercomm raw bytes → FIFO directly */
                calypso_uart_fifo_push(s, b);
            }
            break;

        case GATE_ESCAPE:
            if (sc_len < GATE_BUF_SIZE)
                sc_buf[sc_len++] = b ^ SERCOMM_XOR;
            sc_state = GATE_IN_FRAME;
            break;

        case GATE_IN_FRAME:
            if (b == SERCOMM_FLAG) {
                /* End of frame — dispatch if non-empty */
                if (sc_len > 0)
                    gate_dispatch(s, sc_buf, sc_len);
                sc_len = 0;
                /* Stay in GATE_IN_FRAME — next flag starts new frame */
            } else if (b == SERCOMM_ESCAPE) {
                sc_state = GATE_ESCAPE;
            } else {
                if (sc_len < GATE_BUF_SIZE)
                    sc_buf[sc_len++] = b;
            }
            break;
        }
    }
}
