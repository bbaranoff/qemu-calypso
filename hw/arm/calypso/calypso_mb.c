/*
 * calypso_mb.c - Calypso development board machine
 *
 * Complete machine definition with:
 * - ARM946E-S CPU
 * - Calypso SoC (with all integrated peripherals)
 * - External RAM (8 MiB at 0x01000000)
 * - Flash memory (4 MiB NOR at 0x02000000)
 * - Firmware loading support
 *
 * Usage:
 *   qemu-system-arm -M calypso \
 *     -cpu arm946 \
 *     -kernel loader.highram.elf \
 *     -serial pty \
 *     -monitor stdio \
 *     -nographic
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/block/flash.h"
#include "hw/char/serial.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "elf.h"
#include "target/arm/cpu.h"

#include "hw/arm/calypso/calypso_soc.h"

/* ========================================================================
 * Memory map (board-level, external to SoC)
 * ======================================================================== */

/* External RAM */
#define CALYPSO_XRAM_BASE     0x01000000
#define CALYPSO_XRAM_SIZE     (8 * 1024 * 1024)

/* Flash */
#define CALYPSO_FLASH_BASE    0x02000000
#define CALYPSO_FLASH_SIZE    (4 * 1024 * 1024)

/* ========================================================================
 * Machine state
 * ======================================================================== */

typedef struct CalypsoMachineState {
    MachineState parent;

    ARMCPU *cpu;
    CalypsoSoCState soc;

    /* External memory */
    MemoryRegion xram;
    MemoryRegion flash;

    /* Memory aliases for boot/high vectors */
    MemoryRegion ram_alias0;
    MemoryRegion high_vectors;
} CalypsoMachineState;

#define TYPE_CALYPSO_MACHINE MACHINE_TYPE_NAME("calypso")
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoMachineState, CALYPSO_MACHINE)

/* ========================================================================
 * Machine initialization
 * ======================================================================== */

static void calypso_machine_init(MachineState *machine)
{
    CalypsoMachineState *s = CALYPSO_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    Error *err = NULL;

    /* -------------------------------------------------
     * CPU: ARM946E-S
     * ------------------------------------------------- */
    cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);

    if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
        error_report_err(err);
        exit(1);
    }

    /* -------------------------------------------------
     * SoC
     * ------------------------------------------------- */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_CALYPSO_SOC);

    qdev_prop_set_int32(DEVICE(&s->soc.parent_obj), "trx-port", 4729);
    qdev_prop_set_bit(DEVICE(&s->soc.parent_obj), "enable-trx", true);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->soc), &err)) {
        error_report_err(err);
        exit(1);
    }

    /* -------------------------------------------------
     * IRQ / FIQ vers CPU
     * ------------------------------------------------- */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 0,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_IRQ));

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 1,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_FIQ));

    /* -------------------------------------------------
     * External RAM : 8 MiB @ 0x01000000
     * ------------------------------------------------- */
    memory_region_init_ram(&s->xram,
                           OBJECT(&s->soc.parent_obj),
                           "calypso.xram",
                           CALYPSO_XRAM_SIZE,
                           &error_fatal);

    memory_region_add_subregion(sysmem, CALYPSO_XRAM_BASE, &s->xram);

    /* -------------------------------------------------
     * Flash NOR
     * ------------------------------------------------- */
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    pflash_cfi01_register(CALYPSO_FLASH_BASE,
                          "calypso.flash",
                          CALYPSO_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          64 * 1024,
                          1,
                          0x0089,
                          0x0018,
                          0, 0, 0);

    /* -------------------------------------------------
     * Vector aliases (LOW vectors only)
     * ------------------------------------------------- */

    /* Low vectors @ 0x00000000 */
    memory_region_init_alias(&s->ram_alias0,
                             OBJECT(&s->soc.parent_obj),
                             "calypso.ram_alias0",
                             &s->soc.iram,
                             0,
                             128 * 1024);

    memory_region_add_subregion_overlap(sysmem, 0x00000000,
                                        &s->ram_alias0, 1);

    /* IMPORTANT:
     * Do NOT map high vectors.
     * Compal loader expects vectors at 0x00000000.
     */
    // memory_region_init_alias(&s->high_vectors, ...);
    // memory_region_add_subregion(sysmem, 0xFFFF0000, &s->high_vectors);

    /* -------------------------------------------------
     * Firmware load
     * ------------------------------------------------- */
    if (machine->kernel_filename) {
        uint64_t entry;
        int ret;

        ret = load_elf(machine->kernel_filename, NULL, NULL, NULL,
                       &entry, NULL, NULL, NULL,
                       0, EM_ARM, 1, 0);

        if (ret < 0) {
            ret = load_image_targphys(machine->kernel_filename,
                                      CALYPSO_XRAM_BASE,
                                      CALYPSO_XRAM_SIZE);
            if (ret < 0) {
                error_report("Could not load firmware '%s'",
                             machine->kernel_filename);
                exit(1);
            }
            entry = CALYPSO_XRAM_BASE;
        }

        cpu_set_pc(CPU(s->cpu), entry);

        printf("Calypso firmware loaded:\n");
        printf("  Entry: 0x%08lx\n", (unsigned long)entry);
        printf("  Size:  %d bytes\n", ret);
    }

    printf("\nCalypso machine ready.\n");
}

/* ========================================================================
 * Machine class
 * ======================================================================== */

static void calypso_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Calypso SoC development board (modular architecture)";
    mc->init = calypso_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm946");
    mc->default_ram_size = 0;  /* RAM is fixed in the machine */
    mc->alias = "calypso-high";
}

static const TypeInfo calypso_machine_info = {
    .name          = TYPE_CALYPSO_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(CalypsoMachineState),
    .class_init    = calypso_machine_class_init,
};

static void calypso_machine_register_types(void)
{
    type_register_static(&calypso_machine_info);
}

/* Existing machine */
type_init(calypso_machine_register_types)

