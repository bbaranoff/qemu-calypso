/*
 * calypso_mb.c - Calypso development board machine
 * DEBUG BUILD — verbose flash/memory debug
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
#include "sysemu/reset.h"

#include "hw/arm/calypso/calypso_soc.h"

#define CALYPSO_XRAM_BASE     0x01000000
#define CALYPSO_XRAM_SIZE     (8 * 1024 * 1024)

#define CALYPSO_FLASH_BASE    0x00000000
#define CALYPSO_FLASH_SIZE    (4 * 1024 * 1024)

typedef struct CalypsoMachineState {
    MachineState parent;
    ARMCPU *cpu;
    CalypsoSoCState soc;
    MemoryRegion xram;
    MemoryRegion bootrom;
} CalypsoMachineState;

#define TYPE_CALYPSO_MACHINE MACHINE_TYPE_NAME("calypso")
OBJECT_DECLARE_SIMPLE_TYPE(CalypsoMachineState, CALYPSO_MACHINE)

/*
 * Firmware patches applied after ROM blobs are loaded into memory.
 * Called from qemu_system_reset() which runs after machine_init.
 *
 * 1) NOP cons_puts: prevents console output from filling the 32-slot
 *    msgb pool, which causes talloc panic during boot.
 *
 * 2) Talloc panic → retry with IRQs: if the pool fills despite (1),
 *    re-enable IRQs and retry instead of halting. The NOP at the
 *    cons_puts call site prevents recursive allocation.
 *
 * 3) handle_abort → loop with IRQs enabled: prevents a stray data
 *    abort from permanently disabling IRQs and halting the system.
 */
static void calypso_machine_init(MachineState *machine)
{
    CalypsoMachineState *s = CALYPSO_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    Error *err = NULL;

    fprintf(stderr, "[MB] === calypso_machine_init START ===\n");

    /* ---- CPU ---- */
    cpuobj = object_new(machine->cpu_type);
    s->cpu = ARM_CPU(cpuobj);
    if (!qdev_realize(DEVICE(cpuobj), NULL, &err)) {
        error_report_err(err);
        exit(1);
    }

    /* ---- SoC ---- */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_CALYPSO_SOC);
    qdev_prop_set_int32(DEVICE(&s->soc.parent_obj), "trx-port", 6700);
    qdev_prop_set_bit(DEVICE(&s->soc.parent_obj), "enable-trx", true);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->soc), &err)) {
        error_report_err(err);
        exit(1);
    }

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 0,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc), 1,
        qdev_get_gpio_in(DEVICE(&s->cpu->parent_obj), ARM_CPU_FIQ));

    /* ---- External RAM ---- */
    memory_region_init_ram(&s->xram,
                           OBJECT(&s->soc.parent_obj),
                           "calypso.xram",
                           CALYPSO_XRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, CALYPSO_XRAM_BASE, &s->xram);
    fprintf(stderr, "[MB] XRAM @ 0x%08x (%d MiB)\n",
            CALYPSO_XRAM_BASE, CALYPSO_XRAM_SIZE / (1024*1024));

    /* ---- Flash NOR @ 0x00000000 ----
     *
     * Real Compal E88: Intel 28F320 (4 MiB) on CS0 at 0x00000000.
     * 16-bit bus width (Calypso CS0 is 16-bit).
     * Manufacturer 0x0089 = Intel, Device 0x0018 = 28F320J3.
     * 64 KiB sectors.
     *
     * The loader does CFI queries here. If there's no pflash or
     * something else shadows this address, we get "Failed to
     * initialize flash!".
     */
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    fprintf(stderr, "[MB] Flash: registering pflash_cfi01 @ 0x%08x\n",
            CALYPSO_FLASH_BASE);
    fprintf(stderr, "[MB]   size=%d MiB, sector=64K, width=2 (16-bit)\n",
            CALYPSO_FLASH_SIZE / (1024*1024));
    fprintf(stderr, "[MB]   mfr=0x0089 (Intel), dev=0x0018 (28F320J3)\n");
    fprintf(stderr, "[MB]   drive=%s\n", dinfo ? "attached" : "NONE (blank 0xFF)");

    pflash_cfi01_register(CALYPSO_FLASH_BASE,
                          "calypso.flash",
                          CALYPSO_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          64 * 1024,   /* sector size */
                          1,           /* 8-bit bus width */
                          0x0089,      /* Intel */
                          0x0018,      /* 28F320J3 */
                          0, 0, 0);

    fprintf(stderr, "[MB] Flash: pflash_cfi01 registered OK\n");

    /* ---- Synthetic boot ROM at address 0 ----
     *
     * The real Calypso has internal ROM at 0x00000000 containing
     * exception vector stubs that branch to IRAM exception handlers.
     * OsmocomBB firmware installs handlers at IRAM+0x1C through IRAM+0x34.
     * The boot ROM vectors use: ldr pc, [pc, #0x18] + address table.
     *
     * Layout (0x00-0x3F):
     *   0x00-0x1C: ldr pc, [pc, #0x18] for each exception
     *   0x20-0x3C: handler addresses in IRAM
     */
    {
        uint32_t bootrom_data[16];
        /* ARM instruction: ldr pc, [pc, #0x18] = 0xe59ff018 */
        for (int i = 0; i < 8; i++) {
            bootrom_data[i] = 0xe59ff018;
        }
        /* Handler addresses (read via the ldr pc above):
         * Each vector at offset N reads from offset N+0x20 */
        bootrom_data[8]  = 0x00820000;  /* reset → _start */
        bootrom_data[9]  = 0x0080001C;  /* undef → IRAM _undef_instr */
        bootrom_data[10] = 0x00800020;  /* SWI → IRAM _sw_interr */
        bootrom_data[11] = 0x00800024;  /* prefetch abort → IRAM */
        bootrom_data[12] = 0x00800028;  /* data abort → IRAM */
        bootrom_data[13] = 0x0080002C;  /* reserved → IRAM */
        bootrom_data[14] = 0x00800030;  /* IRQ → IRAM _irq */
        bootrom_data[15] = 0x00800034;  /* FIQ → IRAM _fiq */

        memory_region_init_ram(&s->bootrom, NULL,
                                "calypso.bootrom", 64, &error_fatal);
        memory_region_add_subregion_overlap(sysmem, 0x00000000,
                                             &s->bootrom, 1);
        /* Write vector table into the boot ROM RAM */
        {
            void *ptr = memory_region_get_ram_ptr(&s->bootrom);
            memcpy(ptr, bootrom_data, sizeof(bootrom_data));
        }
        fprintf(stderr, "[MB] Boot ROM @ 0x00000000 (64 bytes, exception vectors)\n");
    }

    /* ---- Firmware load ---- */
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

        fprintf(stderr, "[MB] Firmware: '%s'\n", machine->kernel_filename);
        fprintf(stderr, "[MB]   entry=0x%08lx  size=%d bytes\n",
                (unsigned long)entry, ret);

    }

    fprintf(stderr, "[MB] === Machine ready ===\n");
    fprintf(stderr, "[MB]   Flash:  0x%08x–0x%08x (%d MiB pflash_cfi01)\n",
            CALYPSO_FLASH_BASE,
            CALYPSO_FLASH_BASE + CALYPSO_FLASH_SIZE - 1,
            CALYPSO_FLASH_SIZE / (1024*1024));
    fprintf(stderr, "[MB]   IRAM:   0x00800000–0x0083FFFF (256 KiB)\n");
    fprintf(stderr, "[MB]   XRAM:   0x%08x–0x%08x (%d MiB)\n",
            CALYPSO_XRAM_BASE,
            CALYPSO_XRAM_BASE + CALYPSO_XRAM_SIZE - 1,
            CALYPSO_XRAM_SIZE / (1024*1024));
}

static void calypso_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Calypso SoC development board (modular architecture)";
    mc->init = calypso_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm946");
    mc->default_ram_size = 0;
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

type_init(calypso_machine_register_types)
