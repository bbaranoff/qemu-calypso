/*
 * calypso_trx.c — Calypso hardware emulation + DSP C54x emulation
 * No sockets. Firmware speaks UART only. DSP results in shared RAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/arm/calypso/calypso_trx.h"
#include "hw/arm/calypso/calypso_uart.h"
#include "hw/arm/calypso/sercomm_gate.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "calypso_fbsb.h"
#include "calypso_tint0.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"

/* prim_fbsb emulator-side state — driven both by ARM writes to
 * d_task_md and by DSP writes to d_fb_det. Lazy-init on first use. */
static CalypsoFbsb g_fbsb;
static bool        g_fbsb_inited;

/* Snapshot ARM PC + LR right now (current_cpu must be set, i.e. inside
 * an MMIO callback fired by the running CPU). Returns 0 if unavailable. */
static inline uint32_t arm_now_pc(uint32_t *lr_out)
{
    if (!current_cpu) { if (lr_out) *lr_out = 0; return 0; }
    ARMCPU *acpu = ARM_CPU(current_cpu);
    if (lr_out) *lr_out = acpu->env.regs[14];
    return acpu->env.regs[15];
}
void calypso_tint0_do_tick(uint32_t fn);
#include "chardev/char-fe.h"

extern CalypsoUARTState *g_uart_modem;
extern CalypsoUARTState *g_uart_irda;

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

#define DSP_API_W_PAGE0  0x0000
#define DSP_API_W_PAGE1  0x0028

#define DSP_API_NDB      0x01A8
#define DB_W_D_TASK_D    0
#define DB_W_D_TASK_U    2
#define DB_W_D_TASK_MD   4

typedef struct CalypsoTRX {
    qemu_irq *irqs;
    MemoryRegion dsp_iomem;
    uint16_t     dsp_ram[CALYPSO_DSP_SIZE / 2];
    uint8_t      dsp_page;
    bool         dsp_booted;
    uint32_t     boot_frame;
    MemoryRegion tpu_iomem;
    MemoryRegion tpu_ram_iomem;
    uint16_t     tpu_regs[CALYPSO_TPU_SIZE / 2];
    uint16_t     tpu_ram[CALYPSO_TPU_RAM_SIZE / 2];
    MemoryRegion tsp_iomem;
    uint16_t     tsp_regs[CALYPSO_TSP_SIZE / 2];
    MemoryRegion ulpd_iomem;
    uint16_t     ulpd_regs[CALYPSO_ULPD_SIZE / 2];
    uint32_t     ulpd_counter;
    MemoryRegion sim_iomem;
    uint16_t     sim_regs[CALYPSO_SIM_SIZE / 2];
    QEMUTimer   *sim_atr_timer;
    uint32_t     fn;
    bool         tdma_running;
    bool         tpu_en_pending;  /* set by TPU_CTRL_EN, cleared by tint0 tick */
    bool         dsp_init_done;  /* true after DSP reaches first IDLE */

    /* C54x DSP emulator */
    C54xState   *dsp;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* ---- Console stub ----
 * The firmware has debug logging (cons_puts/puts) that busy-waits on
 * UART TX. No console is connected, so it blocks forever.
 * Replace with BX LR (immediate return) — equivalent to -DNDEBUG. */
void calypso_stub_console(void)
{
    uint32_t val, bx_lr = 0xe12fff1e, nop = 0xe1a00000;

    /* Stub cons_puts and puts — they busy-wait on UART TX with no console */
    cpu_physical_memory_read(0x0082a1b0, &val, 4);
    if (val && val != bx_lr) {
        cpu_physical_memory_write(0x0082a1b0, &bx_lr, 4);
        TRX_LOG("stub: cons_puts → BX LR");
    }
    cpu_physical_memory_read(0x00829ea0, &val, 4);
    if (val && val != bx_lr) {
        cpu_physical_memory_write(0x00829ea0, &bx_lr, 4);
        TRX_LOG("stub: puts → BX LR");
    }

    /* NOP debug BL calls that target console functions.
     * These are BL instructions (0x0Bxxxxxx) in the init path
     * that call logging functions — without console they deadlock. */
    static const hwaddr bl[] = {0x828914, 0x828828, 0x828830, 0x828858, 0x828880};
    for (int i = 0; i < 5; i++) {
        cpu_physical_memory_read(bl[i], &val, 4);
        if ((val & 0x0F000000) == 0x0B000000) {
            cpu_physical_memory_write(bl[i], &nop, 4);
        }
    }
}

/* ---- DSP API RAM ---- */
static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;
    uint64_t val = (size == 2) ? s->dsp_ram[offset/2] :
                   (size == 4) ? (s->dsp_ram[offset/2] | ((uint32_t)s->dsp_ram[offset/2+1] << 16)) :
                   ((uint8_t *)s->dsp_ram)[offset];
    /* DSP boot */
    if (offset == DSP_DL_STATUS_ADDR && !s->dsp_booted) {
        if (++s->boot_frame > 3) {
            s->dsp_ram[DSP_DL_STATUS_ADDR/2] = DSP_DL_STATUS_BOOT;
            s->dsp_ram[DSP_API_VER_ADDR/2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR/2] = 0;
            s->dsp_booted = true;
            TRX_LOG("DSP boot ver=0x%04x", DSP_API_VERSION);
            val = DSP_DL_STATUS_BOOT;
        }
    }
    /* Pure read — C54x DSP handles all results via shared API RAM */
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;
    if (size == 2) s->dsp_ram[offset/2] = value;
    else if (size == 4) { s->dsp_ram[offset/2] = value; s->dsp_ram[offset/2+1] = value >> 16; }
    else ((uint8_t *)s->dsp_ram)[offset] = value;

    /* Sync d_dsp_page writes to DSP API RAM immediately */
    if (offset == 0x01A8) {
        static int page_wr_log = 0;
        if (page_wr_log < 20) {
            TRX_LOG("d_dsp_page WR = 0x%04x (sz=%d fn=%u)", (unsigned)value, size, s->fn);
            page_wr_log++;
        }
        if (s->dsp && s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];
    }
    /* d_task_md (page0=0xFFD00008, page1=0xFFD00030) — DL monitoring task.
     * Log only non-zero writes + the very first few zeros for sanity.
     * Also drive calypso_fbsb's emulator-side state machine: when ARM
     * writes DSP_TASK_FB / DSP_TASK_SB here it means "DSP, do FB/SB
     * detection on the next frame". We mirror that into prim_fbsb so
     * synthetic results land in NDB before the firmware polls. */
    if (offset == 0x0008 || offset == 0x0030) {
        static int tm_zero = 0, tm_nz = 0;
        if (value != 0) {
            if (tm_nz < 200) {
                TRX_LOG("ARM WR d_task_md[p%d] = 0x%04x (NZ) fn=%u",
                        offset == 0x0008 ? 0 : 1,
                        (unsigned)value, s->fn);
                tm_nz++;
            }
            if (!g_fbsb_inited && s->dsp_ram) {
                calypso_fbsb_init(&g_fbsb, s->dsp_ram, 0x0800);
                g_fbsb_inited = true;
            }
            if (g_fbsb_inited) {
                uint32_t lr_before, pc_before = arm_now_pc(&lr_before);
                calypso_fbsb_on_dsp_task_change(&g_fbsb,
                                                (uint16_t)value,
                                                (uint64_t)s->fn);
                uint32_t lr_after, pc_after  = arm_now_pc(&lr_after);
                TRX_LOG("FBSB-PUBLISH ctx fn=%u "
                        "ARM PC before=0x%08x LR=0x%08x "
                        "after=0x%08x LR=0x%08x value=0x%04x",
                        s->fn, pc_before, lr_before, pc_after, lr_after,
                        (unsigned)value);
            }
        } else if (tm_zero < 4) {
            TRX_LOG("ARM WR d_task_md[p%d] = 0 (init) fn=%u",
                    offset == 0x0008 ? 0 : 1, s->fn);
            tm_zero++;
        }
    }
    /* Also catch d_task_d (page0 word 0 = 0xFFD00000, page1 word 0 = 0xFFD00028)
     * to confirm the scheduler is alive at all. */
    if (offset == 0x0000 || offset == 0x0028) {
        static int td_nz = 0;
        if (value != 0 && td_nz < 60) {
            TRX_LOG("ARM WR d_task_d[p%d] = 0x%04x (NZ) fn=%u",
                    offset == 0x0000 ? 0 : 1,
                    (unsigned)value, s->fn);
            td_nz++;
        }
    }
    /* d_spcx_rif (NDB word 2 = ARM 0xFFD001AC) — BSP serial port config */
    if (offset == 0x01AC) {
        static int spcx_arm_log = 0;
        if (spcx_arm_log < 20) {
            TRX_LOG("ARM WR d_spcx_rif = 0x%04x (sz=%d fn=%u)",
                    (unsigned)value, size, s->fn);
            spcx_arm_log++;
        }
    }
    /* DSP bootloader protocol (BL_CMD_STATUS at offset 0x0FFE)
     * The real bootloader lives in DSP internal ROM (not in our dump).
     * ARM writes commands, bootloader responds with BL_STATUS_IDLE (1).
     * BL_CMD_COPY_BLOCK(2) with size=0: jump DSP PC to BL_ADDR_LO.
     * BL_CMD_COPY_BLOCK(2) with size>0: copy block (we let ARM write API directly).
     * BL_CMD_COPY_MODE(4): set data write mode (ack only). */
    if (offset == 0x0FFE) {
        uint16_t cmd = (uint16_t)value;
        TRX_LOG("DSP bootloader: ARM cmd=%d", cmd);
        if (cmd == 0 || cmd == 2 || cmd == 4) {
            /* Simulate BL protocol: immediately acknowledge */
            s->dsp_ram[0x0FFE/2] = 1;  /* BL_STATUS_IDLE */
        }
        if (cmd == 2 && s->dsp) {
            uint16_t bl_size = s->dsp_ram[0x0FFA/2];
            uint16_t bl_addr = s->dsp_ram[0x0FFC/2];
            if (bl_size == 0 && bl_addr != 0) {
                /* Jump: reset DSP (does MVPD copy), then set PC to target */
                TRX_LOG("DSP bootloader: JUMP to 0x%04x", bl_addr);
                c54x_reset(s->dsp);
                s->dsp->pc = bl_addr;
                /* Emulate Calypso internal boot ROM (0xFF80 + 0xB400).
                 * The real boot ROM runs before the bootloader JUMP and
                 * initializes registers. From ROM dump:
                 *   0xFF84: STM #0x5140, AR3 → AR3=0x5140
                 *   0xFF89: STM #0x56FD, AR3 → AR3=0x56FD (overwritten)
                 *   0xFF8F: STM #0x015E, BK  → BK=0x015E
                 *   0xB401: STM #0x1800, ST0 → ST0=0x1800
                 *   0xB403: STM #0x5AC8, SP  → SP=0x5AC8
                 *   0xB408: LD  #57, DP      → DP=57
                 *   0xB40F: RSBX INTM        → interrupts enabled */
                s->dsp->st0 = 0x1800;
                s->dsp->sp = 0x5AC8;
                s->dsp->ar[3] = 0x56FD;
                s->dsp->ar[0] = 0x0040;  /* above MMR range */
                s->dsp->ar[1] = 0x727D;  /* from B356 frame init */
                s->dsp->ar[2] = 0x0E4E;  /* from B356 frame init */
                s->dsp->bk = 0x015E;
                s->dsp->st1 &= ~ST1_INTM; /* RSBX INTM */
                s->dsp->running = true;
                s->dsp_booted = true;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("DSP boot: PC=0x%04x SP=0x%04x AR3=0x%04x BK=0x%04x",
                        bl_addr, s->dsp->sp, s->dsp->ar[3], s->dsp->bk);
            } else if (bl_size > 0) {
                TRX_LOG("DSP bootloader: COPY %d words → DSP 0x%04x", bl_size, bl_addr);
                for (int i = 0; i < bl_size && i < 0x2000; i++)
                    s->dsp->data[bl_addr + i] = s->dsp_ram[i];
            }
        }
    }

    /* DSP page */
    if (offset == DSP_API_NDB) s->dsp_page = value & 1;
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read, .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size=1,.max_access_size=4}, .impl = {.min_access_size=1,.max_access_size=4},
};

/* ---- TPU ---- */
/* calypso_tint0_start() is in calypso_tint0.c */

/* TPU instruction opcodes — match osmocom-bb include/calypso/tpu.h */
#define TPU_OP_SLEEP    (0u << 13)
#define TPU_OP_AT       (1u << 13)
#define TPU_OP_OFFSET   (2u << 13)
#define TPU_OP_SYNCHRO  (3u << 13)
#define TPU_OP_MOVE     (4u << 13)
#define TPU_OP_WAIT     (5u << 13)
#define TPU_OP_MASK     (7u << 13)

/* TPU MOVE target addresses — match osmocom-bb include/calypso/tpu.h */
#define TPUI_TSP_CTRL1  0x00
#define TPUI_TSP_CTRL2  0x01
#define TPUI_TX_3       0x02
#define TPUI_TX_2       0x03
#define TPUI_TX_1       0x04
#define TPUI_TX_4       0x05

#define TPUI_CTRL2_RD   (1u << 0)
#define TPUI_CTRL2_WR   (1u << 1)

/*
 * Walk the TPU script starting at TPU RAM offset 0, decode each instruction,
 * model the embedded TSP write state machine, and dispatch device-bound TSP
 * writes (currently only TWL3025 / IOTA, dev_idx 0). Time-related ops
 * (AT/WAIT/OFFSET/SYNCHRO) are honored as control flow but not as wall-clock
 * delays — each script invocation runs in zero virtual time, which is
 * sufficient to track IOTA's per-frame BDLENA windows.
 */
/* Layer1 timing constants — straight from osmocom-bb tpu_window.c /
 * abb/twl3025.c. Used to derive the GSM timeslot the L1 has armed
 * BDLENA for, from the AT instruction qbit value preceding the
 * BDLON|BDLENA TSP MOVE in the TPU script. */
#define L1_BURST_LENGTH_Q   625        /* qbits per burst slot */
#define L1_TDMA_LENGTH_Q    5000       /* qbits per TDMA frame */
#define DSP_SETUP_TIME      66         /* qbit at which TS0 receive starts */
#define TWL3025_TSP_DELAY   6          /* TSP_DELAY in twl3025.c */

/* Convert the qbit value of the AT instruction immediately preceding a
 * MOVE BDLON|BDLENA into the GSM TN the L1 expects samples from.
 *
 * twl3025_downlink(1, at) computes  bdl_ena = at - TSP_DELAY - 6
 * and the call site uses  at = DSP_SETUP_TIME + L1_BURST_LENGTH_Q * tn_ofs.
 *
 * So  qbit_at_BDLENA = bdl_ena = DSP_SETUP_TIME - TSP_DELAY - 6 + 625*tn
 *                              = 66 - 6 - 6 + 625*tn = 54 + 625*tn.
 */
static uint8_t qbit_to_tn(uint16_t qbit)
{
    int q = (int)qbit - (DSP_SETUP_TIME - TWL3025_TSP_DELAY - 6);
    if (q < 0) q += L1_TDMA_LENGTH_Q;
    int tn = (q + L1_BURST_LENGTH_Q / 2) / L1_BURST_LENGTH_Q; /* nearest */
    return (uint8_t)(tn & 0x7);
}

static void calypso_tpu_run_script(CalypsoTRX *s)
{
    uint8_t  tx[4]    = {0, 0, 0, 0};   /* accumulated TSP TX bytes */
    uint8_t  dev_idx  = 0;
    uint8_t  bitlen   = 0; (void)bitlen;
    uint16_t last_at  = DSP_SETUP_TIME - TWL3025_TSP_DELAY - 6; /* default tn=0 */
    int      max = CALYPSO_TPU_RAM_SIZE / 2;

    for (int i = 0; i < max; i++) {
        uint16_t instr = s->tpu_ram[i];
        uint16_t op    = instr & TPU_OP_MASK;

        if (op == TPU_OP_SLEEP) {
            break;
        }
        if (op == TPU_OP_AT) {
            /* AT operand is the qbit time, low 13 bits */
            last_at = instr & 0x1FFF;
            continue;
        }
        if (op != TPU_OP_MOVE) {
            continue;                          /* WAIT/OFFSET/SYNCHRO */
        }

        uint8_t addr = instr & 0x1f;
        uint8_t data = (instr >> 5) & 0xff;

        switch (addr) {
        case TPUI_TX_1: tx[0] = data; break;
        case TPUI_TX_2: tx[1] = data; break;
        case TPUI_TX_3: tx[2] = data; break;
        case TPUI_TX_4: tx[3] = data; break;
        case TPUI_TSP_CTRL1:
            dev_idx = (data >> 5) & 0x7;
            bitlen  = (data & 0x1f) + 1;
            break;
        case TPUI_TSP_CTRL2:
            if (data & TPUI_CTRL2_WR) {
                if (dev_idx == 0 /* TWL3025_TSP_DEV_IDX */) {
                    /* The expected timeslot for this BDLENA window is
                     * derived from the most recent AT instruction. */
                    uint8_t tn = qbit_to_tn(last_at);
                    calypso_iota_tsp_write(tx[0], tn);
                }
            }
            break;
        default:
            break;
        }
    }
}

static uint64_t calypso_tpu_read(void *o, hwaddr off, unsigned sz) {
    CalypsoTRX *s=o; if (off==TPU_IT_DSP_PG) return s->dsp_page;
    return (off/2<CALYPSO_TPU_SIZE/2)?s->tpu_regs[off/2]:0;
}
static void calypso_tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz) {
    CalypsoTRX *s=o; if (off/2<CALYPSO_TPU_SIZE/2) s->tpu_regs[off/2]=val;
    if (off==TPU_CTRL) {
        uint16_t reg = (uint16_t)val;
        { static int ctrl_log = 0; if (ctrl_log < 20) { TRX_LOG("TPU_CTRL write 0x%04x", reg); ctrl_log++; } }
        if (reg & TPU_CTRL_EN) {
            /* Flag for next TINT0 tick — no separate timer */
            s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_IDLE;
            calypso_tint0_tpu_en();
            l1ctl_set_burst_mode(true);
            static unsigned tpu_run_count = 0;
            tpu_run_count++;
            calypso_tpu_run_script(s);
            if (tpu_run_count <= 20 || (tpu_run_count % 200) == 0) {
                TRX_LOG("TPU_CTRL_EN #%u fn=%u", tpu_run_count, s->fn);
            }
        }
        /* DSP_EN handled by dsp_done via SINT17 */
    }
    if (off==TPU_INT_CTRL && !(val&ICTRL_MCU_FRAME) && !calypso_tint0_running()) calypso_tint0_start();
    if (off==TPU_IT_DSP_PG) s->dsp_page=val&1;
}
static const MemoryRegionOps calypso_tpu_ops = {
    .read=calypso_tpu_read,.write=calypso_tpu_write,.endianness=DEVICE_LITTLE_ENDIAN,
    .valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},
};
static uint64_t calypso_tpu_ram_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off/2<CALYPSO_TPU_RAM_SIZE/2)?s->tpu_ram[off/2]:0;}
static void calypso_tpu_ram_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TPU_RAM_SIZE/2)s->tpu_ram[off/2]=v;}
static const MemoryRegionOps calypso_tpu_ram_ops={.read=calypso_tpu_ram_read,.write=calypso_tpu_ram_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- TSP ---- */
static uint64_t calypso_tsp_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;return(off==TSP_RX_REG)?0xFFFF:(off/2<CALYPSO_TSP_SIZE/2)?s->tsp_regs[off/2]:0;}
static void calypso_tsp_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_TSP_SIZE/2)s->tsp_regs[off/2]=v;}
static const MemoryRegionOps calypso_tsp_ops={.read=calypso_tsp_read,.write=calypso_tsp_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- ULPD ---- */
static uint64_t calypso_ulpd_read(void *o,hwaddr off,unsigned sz){
    CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return 0;
    switch(off){case ULPD_SETUP_CLK13:return 0x2003;case ULPD_COUNTER_HI:s->ulpd_counter+=100;return(s->ulpd_counter>>16)&0xFFFF;
    case ULPD_COUNTER_LO:return s->ulpd_counter&0xFFFF;case ULPD_GAUGING_CTRL:return 1;case ULPD_GSM_TIMER:return s->fn&0xFFFF;
    default:return(off/2<CALYPSO_ULPD_SIZE/2)?s->ulpd_regs[off/2]:0;}
}
static void calypso_ulpd_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off>=0x20&&off<=0x40)return;if(off/2<CALYPSO_ULPD_SIZE/2)s->ulpd_regs[off/2]=v;}
static const MemoryRegionOps calypso_ulpd_ops={.read=calypso_ulpd_read,.write=calypso_ulpd_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=2},.impl={.min_access_size=1,.max_access_size=2},};

/* ---- SIM ---- */
#define SIM_IT 0x08
#define SIM_STAT 0x02
#define SIM_MASKIT 0x0E
static void calypso_sim_atr_cb(void *opaque){CalypsoTRX*s=opaque;uint32_t v=1;cpu_physical_memory_write(0x00830510,&v,4);s->sim_regs[SIM_IT/2]|=1;s->sim_regs[SIM_STAT/2]=1;TRX_LOG("SIM ATR");qemu_irq_pulse(s->irqs[CALYPSO_IRQ_SIM]);}
static uint64_t calypso_sim_read(void *o,hwaddr off,unsigned sz){CalypsoTRX*s=o;uint16_t v=(off/2<CALYPSO_SIM_SIZE/2)?s->sim_regs[off/2]:0;if(off==SIM_IT)s->sim_regs[SIM_IT/2]=0;return v;}
static void calypso_sim_write(void *o,hwaddr off,uint64_t v,unsigned sz){CalypsoTRX*s=o;if(off/2<CALYPSO_SIM_SIZE/2)s->sim_regs[off/2]=v;if(off==SIM_MASKIT)timer_mod_ns(s->sim_atr_timer,qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+50000);}
static const MemoryRegionOps calypso_sim_ops={.read=calypso_sim_read,.write=calypso_sim_write,.endianness=DEVICE_LITTLE_ENDIAN,.valid={.min_access_size=1,.max_access_size=4},.impl={.min_access_size=1,.max_access_size=4},};

/* ---- TINT0: single master clock for the entire Calypso ----
 * Ticks at GSM TDMA frame rate (4.615ms = 13MHz / 60000).
 * Drives: UART, DSP (SINT17 + run-until-idle), ARM (frame IRQ + API IRQ).
 * On real Calypso, ARM and DSP share the 13MHz oscillator.
 * TPU generates frame sync; DSP TINT0 (IMR bit 4, vec 20) is internal.
 * We unify both into one QEMU timer tick. */

/* Called by calypso_tint0.c on each TDMA frame tick */
void calypso_tint0_do_tick(uint32_t fn) {
    CalypsoTRX *s = g_trx;
    if (!s) return;
    s->fn = fn;

    /* DSP TINT0 (IMR bit 4 vec 20) is masked by the firmware (observed
     * IMR=0xFF88, bit 4 = 0). The DSP wakes ONLY on the TPU FRAME line
     * (INT3 = bit 3 vec 19), which is fired below in the dsp_should_run
     * block when ARM has called tpu_dsp_frameirq_enable(). Don't double-
     * fire TINT0 here — it would just sit in IFR forever. */

    /* Lower previous frame IRQ — allows re-raise at end of this tick */
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

    /* 1. UART — poll backend + kick RX only.
     * kick_tx removed 2026-04-07: it called update_irq unconditionally
     * which lowered+raised the IRQ line every frame tick → INTH IRQ 4
     * storm (1000+ hits in seconds, swamping the FBSB chain). TX IRQ
     * fires correctly on THR/IER writes; periodic re-pulse adds noise. */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
    }
    if (g_uart_irda) {
        calypso_uart_poll_backend(g_uart_irda);
        calypso_uart_kick_rx(g_uart_irda);
    }

    /* 2. UL burst poll */
    calypso_trx_tx_burst_poll();

    /* 3. DSP frame processing */
    bool dsp_ran = false;
    bool dsp_should_run = false;

    /* TPU is autonomous: once ARM sets TPU_CTRL_EN, the TPU emits a
     * frame trigger to the DSP every TDMA frame on its own. Run the
     * DSP frame path on EVERY tick while EN is set, not only on the
     * one-shot pending flag set by the ARM TPU_CTRL write. ARM clears
     * EN explicitly when it wants the TPU off. */
    bool tpu_en_now = (s->tpu_regs[TPU_CTRL/2] & TPU_CTRL_EN) != 0;
    if (s->dsp && (tpu_en_now || calypso_tint0_tpu_en_pending())) {
        calypso_tint0_tpu_en_clear();

        /* ── TIC-TOC page toggle ──
         * Real hw double-buffers d_dsp_page (NDB 0x01A8) every TDMA
         * frame: ARM reads page N while DSP writes page N^1, then
         * roles flip on the next TPU frame trigger. Without the
         * toggle, the L1S firmware sees stale results forever and
         * never enqueues the next task. We drive the toggle here on
         * each TINT0 tick where TPU is enabled. */
        s->dsp_ram[0x01A8/2] = (s->dsp_ram[0x01A8/2] ^ 1) & 1;
        if (s->dsp && s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];

        /* DMA: copy API write page to DSP DARAM */
        uint16_t page = s->dsp_ram[0x01A8/2] & 1;
        uint16_t *wp = page ?
            &s->dsp_ram[DSP_API_W_PAGE1/2] : &s->dsp_ram[DSP_API_W_PAGE0/2];
        s->dsp->data[0x0584] = s->dsp_ram[0x01A8/2];
        s->dsp->data[0x0585] = s->fn & 0xFFFF;
        for (int i = 0; i < 20; i++)
            s->dsp->data[0x0586 + i] = wp[i];

        /* Sync d_dsp_page to API RAM */
        if (s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];

        /* BSP: rewind position */
        s->dsp->bsp_pos = 0;

        /* SINT17 — frame interrupt to DSP. Only fire if DSP has reached
         * IDLE at least once (init complete) AND is currently in IDLE.
         * Pre-empting a running DSP (esp. during the boot RPTB sweep)
         * corrupts SP at the ISR entry and locks the DSP at PC=0x010a
         * with SP=0. Wait until the next idle window. */
        if (s->dsp_init_done && s->dsp->idle) {
            c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
            s->dsp->idle = false;
        }
        dsp_should_run = true;
    }
    /* Also run DSP if still initializing (not yet reached first IDLE) */
    if (s->dsp && s->dsp->running && !s->dsp->idle && !dsp_should_run) {
        dsp_should_run = true;
    }

    if (dsp_should_run && s->dsp) {

        /* Run DSP until IDLE — budget allows init MVPD copies to complete.
         * Real C54x @ 100 MHz ≈ 461K insn per 4.615ms frame.
         * During init (before first IDLE), give a fat budget so the
         * MVPD/RPTB sweeps can finish. After init, cap to ~1 frame so
         * a busy-looping DSP cannot starve the ARM (which runs the
         * L1S scheduler that posts d_task_md). */
        int budget = s->dsp_init_done ? 500000 : 5000000;
        int ran = 0, chunk;
        while (!s->dsp->idle && ran < budget) {
            chunk = c54x_run(s->dsp, 100000);
            ran += chunk;
            if (chunk == 0) break;
            if (g_uart_modem) {
                calypso_uart_poll_backend(g_uart_modem);
                calypso_uart_kick_rx(g_uart_modem);
                /* kick_tx removed — see comment above */
            }
            if (g_uart_irda) {
                calypso_uart_poll_backend(g_uart_irda);
                calypso_uart_kick_rx(g_uart_irda);
            }
        }
        /* If DSP didn't finish, dump last instructions for debugging */
        if (!s->dsp->idle && ran >= 500000000) {
            static int timeout_trace = 0;
            TRX_LOG("DSP TIMEOUT: %d insn PC=0x%04x XPC=%d IMR=0x%04x IFR=0x%04x INTM=%d SP=0x%04x PMST=0x%04x unimpl=%u last_unimpl=0x%04x",
                    ran, s->dsp->pc, s->dsp->xpc, s->dsp->imr, s->dsp->ifr,
                    (s->dsp->st1 >> 11) & 1, s->dsp->sp, s->dsp->pmst,
                    s->dsp->unimpl_count, s->dsp->last_unimpl);
            if (timeout_trace < 5) {
                /* Trace 50 instructions to see the loop */
                for (int t = 0; t < 50; t++) {
                    uint16_t pc0 = s->dsp->pc;
                    uint16_t op = s->dsp->prog[pc0];
                    (void)c54x_run(s->dsp, 1);
                    TRX_LOG("  T%02d PC=%04x op=%04x → PC=%04x SP=%04x %s",
                            t, pc0, op, s->dsp->pc, s->dsp->sp,
                            s->dsp->idle ? "IDLE!" : "");
                    if (s->dsp->idle) break;
                }
                timeout_trace++;
            }
        }
        dsp_ran = true;

        static int done_log = 0;
        if (done_log < 200) {
            uint32_t lr_now, pc_now = arm_now_pc(&lr_now);
            TRX_LOG("TINT0: fn=%u page=0x%04x ran=%d "
                    "DSP_PC=0x%04x idle=%d IMR=0x%04x  ARM_PC=0x%08x LR=0x%08x",
                    s->fn, s->dsp_ram[0x01A8/2], ran, s->dsp->pc,
                    s->dsp->idle, s->dsp->imr, pc_now, lr_now);
            done_log++;
        }
    }

    /* Detect first IDLE → DSP init complete. Clean up registers
     * corrupted by init RPTB sweep so SINT17 handler works. */
    if (s->dsp && s->dsp->idle && !s->dsp_init_done) {
        s->dsp_init_done = true;
        /* Reset registers corrupted by init RPTB at 0x76FE */
        s->dsp->sp = 0x5AC8;
        s->dsp->pmst = 0xFFA8;
        /* Leave IMR/IFR as the DSP configured them */
        /* Restore ARs to frame processing values (subroutine 0xB356) */
        s->dsp->ar[0] = 0x0040;
        s->dsp->ar[1] = 0x727D;
        s->dsp->ar[2] = 0x0E4E;
        s->dsp->ar[3] = 0x56FD;
        s->dsp->bk = 0x015E;
        /* Stop any active RPTB from init */
        s->dsp->rptb_active = false;
        s->dsp->st1 &= ~ST1_BRAF;
        s->dsp->rpt_active = false;
        TRX_LOG("DSP init complete (first IDLE) fn=%u PC=0x%04x SP=0x%04x IMR=0x%04x",
                s->fn, s->dsp->pc, s->dsp->sp, s->dsp->imr);
    }
    /* 4. API IRQ (IRQ15) — only when DSP actually processed a frame.
     * Lower first (edge-like), raise only if DSP ran this tick. */
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_API]);
    if (dsp_ran && s->dsp && s->dsp->idle) {
        qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
    }

    /* 5. TPU frame IRQ (IRQ4) — ARM TDMA scheduler.
     * Raise and hold — firmware ACKs via INTH register write. */
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

}  /* end calypso_tint0_do_tick */

/* TINT0 start is now in calypso_tint0.c */

/* ---- Sercomm burst transport (DLCI 4) ---- */

/* Condition check: is the firmware ready to receive a DL burst?
 * Used by UART filter to gate DLCI 4 frames.
 * Conditions: DSP booted, TDMA running, L1CTL client active.
 * Only 1 burst per TDMA frame — like real TPU/BSP. */
bool calypso_trx_burst_ready(void)
{
    if (!g_trx) return false;
    CalypsoTRX *s = g_trx;

    /* Restaurant closed: no client, no FBSB_REQ, no service */
    if (!s->dsp_booted || !calypso_tint0_running() || !l1ctl_burst_mode())
        return false;

    /* One burst per frame */
    static uint32_t last_fn = 0xFFFFFFFF;
    if (s->fn == last_fn) return false;
    last_fn = s->fn;
    return true;
}

/* RX burst from bridge (DL) — store in DSP RAM for firmware to read. */
void calypso_trx_rx_burst(const uint8_t *data, int len)
{
    if (!g_trx || len < 6) return;
    CalypsoTRX *s = g_trx;

    uint32_t fn = ((uint32_t)data[1]<<24)|((uint32_t)data[2]<<16)|
                  ((uint32_t)data[3]<<8)|(uint32_t)data[4];
    uint8_t tn = data[0] & 0x07; (void)tn;

    /* Sync FN */
    s->fn = fn % GSM_HYPERFRAME;

    /* DL format from gate (after gr-gsm modulation in bridge.py):
     *   bytes 0..7  : TN(1) FN(4) RSSI(1) TOA(2)  -- legacy hdr from gate
     *   bytes 8..   : N int16 LE samples, I and Q interleaved
     * sps=4, 148 syms => 592 complex samples => 1184 int16. */
    int nint16 = (len - 8) / 2;
    if (nint16 < 0) nint16 = 0;
    if (nint16 > 2000) nint16 = 2000;

    /* Load int16 stream into C54x BSP buffer (uint16_t reinterpreted). */
    if (s->dsp) {
        static uint16_t samples[2048];
        for (int i = 0; i < nint16; i++) {
            int16_t v = (int16_t)(data[8 + i*2] | (data[8 + i*2 + 1] << 8));
            samples[i] = (uint16_t)v;
        }
        c54x_bsp_load(s->dsp, samples, nint16);

        /* Fire BRINT0 — BSP receive complete (vec 21, IMR bit 5)
         * Only after init — during boot the DSP doesn't expect bursts */
        if (s->dsp_init_done)
            c54x_interrupt_ex(s->dsp, 21, 5);

        /* Mirror first chunk into legacy DARAM scratch areas (firmware probes). */
        int nm = nint16 < 296 ? nint16 : 296;  /* keep within DARAM page */
        uint16_t ptr = s->dsp->data[0x00B9];
        if (ptr >= 0x0020 && ptr < 0x0800 - nm) {
            for (int i = 0; i < nm; i++)
                s->dsp->data[ptr + i] = samples[i];
        }
        for (int i = 0; i < nm; i++) {
            s->dsp->data[0x03F0 + i] = samples[i];
            s->dsp->data[0x04F0 + i] = samples[i];
        }
    }

}

/* TX burst: send UL burst from DSP write page out via sercomm_gate UDP
 * (symmetric to the DL on_dl_burst path). The chef closes the loop:
 * ARM L1 writes d_task_u → DSP processes → DSP DARAM UL bits →
 * sercomm_gate_send_ul_burst → UDP 6802 → bridge → osmo-bts-trx. */
static void calypso_trx_send_ul_burst(CalypsoTRX *s, uint16_t task_u)
{
    if (task_u == 0) return;

    /* Read UL burst metadata from API write page (ARM L1 sets these). */
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
    uint8_t  tn = wp[3] & 0x07;
    uint32_t fn = s->fn;

    /* Symmetric to DL: BSP pulls bits from DSP DARAM UL buffer, then we
     * hand them off to sercomm_gate which ships via UDP to the bridge.
     * Both directions now flow through calypso_bsp_*_burst. */
    uint8_t bits[148];
    if (calypso_bsp_tx_burst(tn, fn, bits)) {
        sercomm_gate_send_ul_burst(tn, fn, bits);
    }
}

void calypso_trx_tx_burst_poll(void)
{
    if (!g_trx) return;
    /* Check if firmware wrote a UL task */
    CalypsoTRX *s = g_trx;
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
    uint16_t task_u = wp[DB_W_D_TASK_U];
    if (task_u != 0) {
        calypso_trx_send_ul_burst(s, task_u);
        wp[DB_W_D_TASK_U] = 0;  /* clear after sending */
    }
}

/* DSP → ARM api_ram write notifier — called from c54x data_write
 * whenever the DSP touches a shared mailbox cell. We pulse the API IRQ
 * so the ARM L1 ISR can run prim_*_resp without waiting for next frame. */
/* d_fb_det lives at DSP word addr 0x08F8 → ARM dsp_ram offset
 * (0x08F8 - 0x0800) = 0xF8 words. When the DSP correlator decides "FB
 * found" it writes a non-zero opcode-derived word here. The firmware
 * (l1s_fbdet_resp) only checks "non-zero", but it also reads neighbour
 * NDB cells (a_sync_demod TOA/PM/ANG/SNR). We override the raw write
 * with a clean published "FB found" so prim_fbsb's poll sees consistent
 * values and accepts the burst.                                          */
#define NDB_OFF_D_FB_DET   0x00F8

/* declared near top */

static void trx_dsp_api_write_cb(void *opaque, uint16_t woff, uint16_t val)
{
    CalypsoTRX *s = opaque;
    if (!s || !s->irqs) return;
    qemu_irq_pulse(s->irqs[CALYPSO_IRQ_API]);

    if (woff == NDB_OFF_D_FB_DET && val != 0) {
        if (!g_fbsb_inited) {
            calypso_fbsb_init(&g_fbsb, s->dsp_ram, 0x0800);
            g_fbsb_inited = true;
        }
        TRX_LOG("FB-DET hook: DSP wrote d_fb_det=0x%04x — publishing clean FB",
                val);
        calypso_fbsb_publish_fb_found(&g_fbsb,
                                      /* toa  */ 0,
                                      /* pm   */ 80,
                                      /* angle*/ 0,
                                      /* snr  */ 100);
    }
}

/* ============================================================
 * Orchestrator I/O — chef pour les bursts
 * ============================================================ */

/* Write side: wrap an L1CTL message in sercomm DLCI 5 and push it to
 * the modem PTY (toward bridge.py → mobile L23). */
void calypso_trx_l23_write(const uint8_t *l1ctl, int len)
{
    if (!g_uart_modem || len <= 0) return;

    uint8_t frame[1024];
    int pos = 0;
    frame[pos++] = 0x7E;
    uint8_t hdr[2] = { 0x05, 0x03 };  /* DLCI 5 (L1A_L23), CTRL */
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == 0x7E || hdr[i] == 0x7D) {
            frame[pos++] = 0x7D; frame[pos++] = hdr[i] ^ 0x20;
        } else {
            frame[pos++] = hdr[i];
        }
    }
    for (int i = 0; i < len && pos < (int)sizeof(frame) - 2; i++) {
        uint8_t c = l1ctl[i];
        if (c == 0x7E || c == 0x7D) {
            frame[pos++] = 0x7D; frame[pos++] = c ^ 0x20;
        } else {
            frame[pos++] = c;
        }
    }
    frame[pos++] = 0x7E;

    fprintf(stderr, "[trx→L23] L1CTL %d bytes (sercomm framed=%d)\n", len, pos);
    qemu_chr_fe_write_all(&g_uart_modem->chr, frame, pos);
}

/* Read side: a DL burst arrived from sercomm_gate (UDP TRXD).
 * The chef decides what to do with it: route to BSP DMA so the DSP path
 * processes it. Later this is where TDMA-window gating / fast-path
 * L1CTL_DATA_IND would live. */
void calypso_trx_on_dl_burst(uint8_t tn, uint32_t fn,
                             const int16_t *iq, int nint16)
{
    (void)iq; (void)nint16;
    static unsigned cnt;
    if ((cnt++ % 100) == 0) {
        TRX_LOG("on_dl_burst tn=%u fn=%u (#%u)", tn, fn, cnt);
    }

}

/* UL hook — symmetric to on_dl_burst, called by calypso_bsp.c just
 * before shipping a UL burst out via UDP. */
void calypso_trx_on_ul_burst(uint8_t tn, uint32_t fn, const uint8_t *bits148)
{
    (void)bits148;
    static unsigned cnt;
    if ((cnt++ % 100) == 0) {
        TRX_LOG("on_ul_burst tn=%u fn=%u (#%u)", tn, fn, cnt);
    }
}

void calypso_trx_on_dl_l2(uint8_t tn, uint32_t fn,
                          const uint8_t *l2, int len)
{
    (void)tn; (void)fn; (void)l2; (void)len;
}

/* ---- Init ---- */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s; s->irqs = irqs;
    TRX_LOG("=== Calypso hardware init ===");

    memory_region_init_io(&s->dsp_iomem,NULL,&calypso_dsp_ops,s,"calypso.dsp_api",CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_DSP_BASE,&s->dsp_iomem);
    /* DSP starts unbooted — ARM firmware drives the boot sequence */
    s->dsp_ram[DSP_DL_STATUS_ADDR/2] = 0; s->dsp_booted = false; s->dsp_init_done = false;

    memory_region_init_io(&s->tpu_iomem,NULL,&calypso_tpu_ops,s,"calypso.tpu",CALYPSO_TPU_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_BASE,&s->tpu_iomem);
    memory_region_init_io(&s->tpu_ram_iomem,NULL,&calypso_tpu_ram_ops,s,"calypso.tpu_ram",CALYPSO_TPU_RAM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TPU_RAM_BASE,&s->tpu_ram_iomem);
    memory_region_init_io(&s->tsp_iomem,NULL,&calypso_tsp_ops,s,"calypso.tsp",CALYPSO_TSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_TSP_BASE,&s->tsp_iomem);
    memory_region_init_io(&s->ulpd_iomem,NULL,&calypso_ulpd_ops,s,"calypso.ulpd",CALYPSO_ULPD_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_ULPD_BASE,&s->ulpd_iomem);
    memory_region_init_io(&s->sim_iomem,NULL,&calypso_sim_ops,s,"calypso.sim",CALYPSO_SIM_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_SIM_BASE,&s->sim_iomem);
    s->sim_atr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_sim_atr_cb,s);

    /* TINT0 master clock is created by calypso_tint0.c */

    /* C54x DSP emulator */
    {
        const char *rom_path = getenv("CALYPSO_DSP_ROM");
        if (!rom_path) rom_path = "/opt/GSM/calypso_dsp.txt";
        s->dsp = c54x_init();
        if (s->dsp) {
            c54x_set_api_ram(s->dsp, s->dsp_ram);
            /* DSP → ARM bidi: register notify callback so any DSP write
             * to the API mailbox pulses CALYPSO_IRQ_API on the ARM side. */
            s->dsp->api_write_cb = trx_dsp_api_write_cb;
            s->dsp->api_write_cb_opaque = s;
            if (c54x_load_rom(s->dsp, rom_path) == 0) {
                /* Don't reset/boot yet — wait for ARM to write DSP_DL_STATUS_READY */
                s->dsp->running = false;
                /* Hand the DSP off to the BSP DMA module */
                calypso_bsp_init(s->dsp);
                calypso_iota_init();
                TRX_LOG("BUILD 2026-04-05T20:30:16 F4EB=RETE IMR_keep");
                TRX_LOG("C54x DSP loaded from %s (waiting for ARM)", rom_path);
            } else {
                TRX_LOG("C54x DSP ROM not found at %s", rom_path);
                free(s->dsp);
                s->dsp = NULL;
            }
        }
    }

    TRX_LOG("=== Hardware ready ===");
}
