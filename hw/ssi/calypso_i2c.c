/*
 * Calypso I2C Controller - Minimal stub
 * Returns "ready" immediately to avoid firmware blocking
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/log.h"

#define TYPE_CALYPSO_I2C "calypso-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoI2CState, CALYPSO_I2C)

struct CalypsoI2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
};

static uint64_t calypso_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    switch (offset) {
    case 0x04: /* STATUS - always ready */
        return 0x04; /* ARDY (access ready) */
    default:
        return 0;
    }
}

static void calypso_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    /* Accept all writes silently */
}

static const MemoryRegionOps calypso_i2c_ops = {
    .read = calypso_i2c_read,
    .write = calypso_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = { .min_access_size = 2, .max_access_size = 2 },
};

static void calypso_i2c_realize(DeviceState *dev, Error **errp)
{
    CalypsoI2CState *s = CALYPSO_I2C(dev);
    
    memory_region_init_io(&s->iomem, OBJECT(dev), &calypso_i2c_ops, s,
                          "calypso-i2c", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void calypso_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = calypso_i2c_realize;
    dc->desc = "Calypso I2C stub";
}

static const TypeInfo calypso_i2c_info = {
    .name          = TYPE_CALYPSO_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CalypsoI2CState),
    .class_init    = calypso_i2c_class_init,
};

static void calypso_i2c_register_types(void)
{
    type_register_static(&calypso_i2c_info);
}

type_init(calypso_i2c_register_types)
