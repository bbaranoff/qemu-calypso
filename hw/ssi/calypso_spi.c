/*
 * calypso_spi.c — Calypso SPI + TWL3025 ABB
 *
 * REWRITE: Correct register map + poweroff blocking.
 *
 * The OsmocomBB loader calls twl3025_power_off() (writes TOGBR1 bit 0)
 * whenever flash_init() fails. In QEMU we block this to keep the
 * loader alive so osmoload can still inject firmware.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/log.h"
#include "hw/arm/calypso/calypso_spi.h"

/* Register offsets */
#define SPI_REG_SET1     0x00
#define SPI_REG_SET2     0x02
#define SPI_REG_CTRL     0x04
#define SPI_REG_STATUS   0x06
#define SPI_REG_TX_LSB   0x08
#define SPI_REG_TX_MSB   0x0A
#define SPI_REG_RX_LSB   0x0C
#define SPI_REG_RX_MSB   0x0E

/* CTRL bits */
#define SPI_CTRL_START   (1 << 0)

/* ---- TWL3025 ABB SPI transaction ---- */

static uint16_t twl3025_spi_xfer(CalypsoSPIState *s, uint16_t tx)
{
    int read  = (tx >> 15) & 1;
    int addr  = (tx >> 6) & 0x1FF;
    int wdata = tx & 0x3F;

    if (addr >= 256) {
        addr = 0;
    }

    if (read) {
        fprintf(stderr, "[SPI] ABB read  addr=0x%02x → 0x%04x\n",
                addr, s->abb_regs[addr]);
        return s->abb_regs[addr];
    } else {
        fprintf(stderr, "[SPI] ABB write addr=0x%02x data=0x%02x", addr, wdata);

        /* ---- TOGBR1 (0x09): power control toggle ----
         * Bit 0 (TOGB) = power off the phone.
         * The loader calls twl3025_power_off() which writes 1 here
         * whenever flash_init() fails.
         * We BLOCK this to keep the loader alive in QEMU.
         */
        if (addr == ABB_TOGBR1 && (wdata & 0x01)) {
            fprintf(stderr, " *** POWEROFF BLOCKED (TOGBR1 bit 0) ***\n");
            return 0;  /* Don't store, don't poweroff */
        }

        /* ---- TOGBR2 (0x0A): other toggles ---- */
        if (addr == ABB_TOGBR2) {
            fprintf(stderr, " (TOGBR2)\n");
            s->abb_regs[addr] = wdata;
            return 0;
        }

        fprintf(stderr, "\n");

        s->abb_regs[addr] = wdata;

        if (addr == ABB_VRPCDEV) {
            s->abb_regs[ABB_VRPCSTS] = 0x1F;
        }
        return 0;
    }
}

/* ---- MMIO read ---- */

static uint64_t calypso_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        return s->set1;
    case SPI_REG_SET2:
        return s->set2;
    case SPI_REG_CTRL:
        return s->ctrl;
    case SPI_REG_STATUS:
        return SPI_STATUS_RE;
    case SPI_REG_TX_LSB:
        return s->tx_data & 0xFF;
    case SPI_REG_TX_MSB:
        return (s->tx_data >> 8) & 0xFF;
    case SPI_REG_RX_LSB:
        return s->rx_data & 0xFF;
    case SPI_REG_RX_MSB:
        return (s->rx_data >> 8) & 0xFF;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: read at 0x%02x\n",
                       (unsigned)offset);
        return 0;
    }
}

/* ---- MMIO write ---- */

static void calypso_spi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    CalypsoSPIState *s = CALYPSO_SPI(opaque);

    switch (offset) {
    case SPI_REG_SET1:
        s->set1 = value & 0xFFFF;
        break;
    case SPI_REG_SET2:
        s->set2 = value & 0xFFFF;
        break;
    case SPI_REG_CTRL:
        s->ctrl = value & 0xFFFF;
        if (value & SPI_CTRL_START) {
            s->rx_data = twl3025_spi_xfer(s, s->tx_data);
            qemu_irq_pulse(s->irq);
        }
        break;
    case SPI_REG_STATUS:
        break;
    case SPI_REG_TX_LSB:
        s->tx_data = (s->tx_data & 0xFF00) | (value & 0xFF);
        break;
    case SPI_REG_TX_MSB:
        s->tx_data = (s->tx_data & 0x00FF) | ((value & 0xFF) << 8);
        break;
    case SPI_REG_RX_LSB:
    case SPI_REG_RX_MSB:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "calypso-spi: write 0x%04x at 0x%02x\n",
                       (unsigned)value, (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps calypso_spi_ops = {
    .read = calypso_spi_read,
    .write = calypso_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

/* ---- QOM lifecycle ---- */

static void calypso_spi_realize(DeviceState *dev, Error **errp)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_spi_ops, s,
                          "calypso-spi", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static void calypso_spi_reset(DeviceState *dev)
{
    CalypsoSPIState *s = CALYPSO_SPI(dev);

    s->set1 = 0;
    s->set2 = 0;
    s->ctrl = 0;
    s->status = SPI_STATUS_RE;
    s->tx_data = 0;
    s->rx_data = 0;
    memset(s->abb_regs, 0, sizeof(s->abb_regs));

    s->abb_regs[ABB_VRPCSTS] = 0x1F;
    s->abb_regs[ABB_ITSTATREG] = 0x00;
}

static void calypso_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = calypso_spi_realize;
    device_class_set_legacy_reset(dc, calypso_spi_reset);
    dc->desc = "Calypso SPI controller + TWL3025 ABB";
}

static const TypeInfo calypso_spi_info = {
    .name          = TYPE_CALYPSO_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoSPIState),
    .class_init    = calypso_spi_class_init,
};

static void calypso_spi_register_types(void)
{
    type_register_static(&calypso_spi_info);
}

type_init(calypso_spi_register_types)
