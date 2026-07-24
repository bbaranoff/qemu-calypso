/*
 * calypso_tsp.c -- Calypso DBB internal TSP (Time Serial Port) device layer
 *
 * See calypso_tsp.h. Mirrors osmocom-bb-transceiver's calypso/tsp.c
 * (tsp_write/tsp_setup/tsp_act_update) from the receiving end: decodes
 * the MOVE instructions the TPU sequencer (calypso_tpu.c) hands us back
 * into the higher-level TSP transactions the firmware originally issued.
 *
 * [2026-07-23] Extracted from calypso_tpu.c per user direction (split
 * mirrors the real firmware's tpu.c/tsp.c separation; "calypso_tsp.c
 * etc" -- keep growing this as more radio-chip protocol layers get
 * properly modeled instead of stubbed).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_tsp.h"
#include "hw/arm/calypso/calypso_debug.h"

/* calypso_iota.c -- the only downstream consumer wired up today (TWL3025,
 * TSP device 0: BDLENA/BULENA burst-window enable byte). */
void calypso_iota_tsp_write(uint8_t data, uint8_t expected_tn);

#define TSP_LOG(fmt, ...) \
    do { if (calypso_debug_enabled("TPU")) \
        fprintf(stderr, "[calypso-tsp] " fmt "\n", ##__VA_ARGS__); } while (0)

/* Latched state, persists across TPU scenarios exactly like the real
 * firmware's statics (tsp.c's `tspact_state`) and the real TSP shift
 * registers (TX_n/CTRL1 stay loaded until the next MOVE overwrites them,
 * independent of which TPU-RAM scenario wrote them). */
static struct {
    uint8_t  tx[4];       /* TX_1..TX_4 -> tx[0..3] */
    uint8_t  ctrl1;
    uint16_t act;         /* TSPACT enable-line state (tsp_act_update()) */
} tsp;

bool calypso_tsp_owns_addr(uint8_t addr)
{
    switch (addr) {
    case TPUI_TSP_CTRL1: case TPUI_TSP_CTRL2:
    case TPUI_TX_1: case TPUI_TX_2: case TPUI_TX_3: case TPUI_TX_4:
    case TPUI_TSP_ACT_L: case TPUI_TSP_ACT_U:
    case TPUI_TSP_SET1: case TPUI_TSP_SET2: case TPUI_TSP_SET3:
        return true;
    default:
        return false;
    }
}

void calypso_tsp_move(uint8_t addr, uint8_t data, uint32_t fn)
{
    switch (addr) {
    case TPUI_TX_1: tsp.tx[0] = data; break;
    case TPUI_TX_2: tsp.tx[1] = data; break;
    case TPUI_TX_3: tsp.tx[2] = data; break;
    case TPUI_TX_4: tsp.tx[3] = data; break;
    case TPUI_TSP_CTRL1:
        tsp.ctrl1 = data;
        break;
    case TPUI_TSP_CTRL2:
        if (data & TPUI_CTRL2_WR) {
            /* tsp_write(dev_idx, bitlen, dout) in calypso/tsp.c: CTRL1 =
             * (dev_idx<<5)|(bitlen-1), written just before this WR pulse.
             * Reconstruct dout MSB-first from however many TX_n bytes
             * bitlen actually spans. */
            uint8_t dev_idx = (tsp.ctrl1 >> 5) & 0x07;
            uint8_t bitlen  = (tsp.ctrl1 & 0x1F) + 1;
            uint32_t dout;
            if (bitlen <= 8)       dout = tsp.tx[0];
            else if (bitlen <= 16) dout = ((uint32_t)tsp.tx[0] << 8) | tsp.tx[1];
            else if (bitlen <= 24) dout = ((uint32_t)tsp.tx[0] << 16) |
                                            ((uint32_t)tsp.tx[1] << 8) | tsp.tx[2];
            else                   dout = ((uint32_t)tsp.tx[0] << 24) |
                                            ((uint32_t)tsp.tx[1] << 16) |
                                            ((uint32_t)tsp.tx[2] << 8) | tsp.tx[3];
            if (dev_idx == 0) {
                /* TWL3025 (audio/ABB companion): BDLENA/BULENA control byte,
                 * always bitlen<=8 in practice -> tx[0] alone is correct
                 * (matches the original single-byte implementation). */
                calypso_iota_tsp_write(tsp.tx[0], 0);
            } else {
                /* No downstream hardware model for other TSP devices (e.g.
                 * dev 1 = RF frontend tuning words) -- decoded correctly and
                 * made visible instead of silently dropped. */
                TSP_LOG("WR dev=%u bitlen=%u dout=0x%08x fn=%u (no consumer)",
                         dev_idx, bitlen, dout, fn);
            }
        }
        break;
    case TPUI_TSP_ACT_L:
        if (data != (tsp.act & 0xff)) {
            tsp.act = (tsp.act & 0xff00) | data;
            TSP_LOG("ACT_L -> 0x%02x (tspact=0x%04x) fn=%u", data, tsp.act, fn);
        }
        break;
    case TPUI_TSP_ACT_U:
        if (data != (tsp.act >> 8)) {
            tsp.act = (tsp.act & 0x00ff) | ((uint16_t)data << 8);
            TSP_LOG("ACT_U -> 0x%02x (tspact=0x%04x) fn=%u", data, tsp.act, fn);
        }
        break;
    case TPUI_TSP_SET1: case TPUI_TSP_SET2: case TPUI_TSP_SET3:
        /* tsp_setup(): static clock-edge/CS-polarity config, not part of
         * the runtime burst-timing path. Logged for visibility only. */
        TSP_LOG("SET%d <- 0x%02x fn=%u", addr - TPUI_TSP_SET1 + 1, data, fn);
        break;
    default:
        TSP_LOG("move to unowned addr=0x%02x data=0x%02x fn=%u", addr, data, fn);
        break;
    }
}
