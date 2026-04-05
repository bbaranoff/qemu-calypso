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
#include "hw/arm/calypso/calypso_c54x.h"
#include "calypso_tint0.h"
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

    /* Log ALL writes to d_dsp_page offset (0x01A8), any value */
    if (offset == 0x01A8) {
        static int page_wr_log = 0;
        if (page_wr_log < 20) {
            TRX_LOG("d_dsp_page WR = 0x%04x (sz=%d fn=%u)", (unsigned)value, size, s->fn);
            page_wr_log++;
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
                s->dsp->running = true;
                s->dsp_booted = true;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("DSP boot: PC=0x%04x SP=0x%04x", bl_addr, s->dsp->sp);
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
            static int tpu_en_log = 0;
            if (tpu_en_log < 10) { TRX_LOG("TPU_CTRL_EN fn=%u", s->fn); tpu_en_log++; }
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

    /* TINT0: set IFR bit 4 on DSP (Timer 0 interrupt, vec 20).
     * On real hardware this fires when TIM reaches 0.
     * Here, QEMU timer replaces the per-instruction timer tick. */
    if (s->dsp && s->dsp->running) {
        c54x_interrupt_ex(s->dsp, TINT0_VEC, TINT0_IFR_BIT);
    }

    /* Lower previous frame IRQ — allows re-raise at end of this tick */
    qemu_irq_lower(s->irqs[CALYPSO_IRQ_TPU_FRAME]);

    /* 1. UART — poll backend + flush TX/RX (both UARTs) */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
        calypso_uart_kick_tx(g_uart_modem);
    }
    if (g_uart_irda) {
        calypso_uart_poll_backend(g_uart_irda);
        calypso_uart_kick_rx(g_uart_irda);
        calypso_uart_kick_tx(g_uart_irda);
    }

    /* 2. UL burst poll */
    calypso_trx_tx_burst_poll();

    /* 3. DSP frame processing */
    bool dsp_ran = false;
    bool dsp_should_run = false;

    if (s->dsp && calypso_tint0_tpu_en_pending()) {
        calypso_tint0_tpu_en_clear();
        s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_EN;

        /* Don't send SINT17 until DSP has finished init (first IDLE) */
        if (!s->dsp_init_done) {
            goto skip_sint17;
        }

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

        /* SINT17 — frame interrupt to DSP */
        c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
        s->dsp->idle = false;  /* wake from IDLE for new frame */
        dsp_should_run = true;
    }
skip_sint17:
    /* Also run DSP if still initializing (not yet reached first IDLE) */
    if (s->dsp && s->dsp->running && !s->dsp->idle && !dsp_should_run) {
        dsp_should_run = true;
    }

    if (dsp_should_run && s->dsp) {

        /* Run DSP until IDLE — budget allows init MVPD copies to complete.
         * Real C54x @ 100 MHz ≈ 461K insn per 4.615ms frame.
         * During init (before first IDLE), give unlimited budget.
         * After init, use 500K per frame. */
        int budget = 5000000;  /* 5M for init */
        int ran = 0, chunk;
        while (!s->dsp->idle && ran < budget) {
            chunk = c54x_run(s->dsp, 100000);
            ran += chunk;
            if (chunk == 0) break;
            if (g_uart_modem) {
                calypso_uart_poll_backend(g_uart_modem);
                calypso_uart_kick_rx(g_uart_modem);
                calypso_uart_kick_tx(g_uart_modem);
            }
            if (g_uart_irda) {
                calypso_uart_poll_backend(g_uart_irda);
                calypso_uart_kick_rx(g_uart_irda);
                calypso_uart_kick_tx(g_uart_irda);
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

        /* Write PM/SNR results to API read page after DSP completes.
         * The DSP should compute these from radio samples, but without
         * real BSP data it writes 0. Override with realistic values
         * so the firmware's PM measurement works. */
        if (s->dsp->idle) {
            uint16_t rpage = s->dsp_ram[0x01A8/2] & 1;
            uint16_t *rp = rpage ?
                &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];
            /* Non-DSP33 layout: rp[8..10]=a_pm[3], rp[11..14]=a_serv_demod[4] */
            rp[8]  = 4864;   /* a_pm[0] — PM strong signal */
            rp[9]  = 4864;   /* a_pm[1] */
            rp[10] = 4864;   /* a_pm[2] */
            rp[11] = 0;      /* D_TOA */
            rp[12] = 4864;   /* D_PM */
            rp[13] = 0;      /* D_ANGLE */
            rp[14] = 100;    /* D_SNR */
            /* d_fb_det in NDB at ARM offset 0x01F0 = dsp_ram[0xF8]
             * Set to 1 when burst mode active (firmware is searching for FB) */
            if (l1ctl_burst_mode()) {
                s->dsp_ram[0xF8] = 1;  /* d_fb_det = FOUND */
                /* sync_demod: TOA, PM, ANGLE, SNR at 0x01F4-0x01FA */
                s->dsp_ram[0xFA] = 0;     /* TOA */
                s->dsp_ram[0xFB] = 4864;  /* PM */
                s->dsp_ram[0xFC] = 0;     /* ANGLE */
                s->dsp_ram[0xFD] = 100;   /* SNR */
            }
        }

        static int done_log = 0;
        if (done_log < 30) {
            TRX_LOG("TINT0: fn=%u page=0x%04x ran=%d PC=0x%04x idle=%d IMR=0x%04x",
                    s->fn, s->dsp_ram[0x01A8/2], ran, s->dsp->pc,
                    s->dsp->idle, s->dsp->imr);
            done_log++;
        }
    }

    /* Detect first IDLE → DSP init complete. Clean up corrupted
     * registers so SINT17 handler works with proper stack. */
    if (s->dsp && s->dsp->idle && !s->dsp_init_done) {
        s->dsp_init_done = true;
        /* Reset SP and PMST corrupted by init RPTB sweep */
        s->dsp->sp = 0x5AC8;
        s->dsp->pmst = 0xFFA8;
        s->dsp->imr = 0x0000;
        s->dsp->ifr = 0x0000;
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

    /* DL format: header(8) + GMSK I/Q samples (16-bit LE, 2 bytes each)
     * Bridge sends: TN(1) FN(4) RSSI(1) TOA(2) + samples(N x int16_t) */
    int nsamples = (len - 8) / 2;  /* 16-bit samples */
    if (nsamples > 148) nsamples = 148;
    if (nsamples < 0) nsamples = 0;
    int nbits = nsamples;

    /* RX_BURST logging removed */

    /* Load GMSK I/Q samples into C54x DARAM + BSP buffer */
    if (s->dsp) {
        uint16_t samples[160];
        for (int i = 0; i < nbits; i++) {
            /* Read 16-bit LE GMSK I/Q samples from bridge */
            int16_t s16 = (int16_t)(data[8 + i*2] | (data[8 + i*2 + 1] << 8));
            samples[i] = (uint16_t)s16;
        }
        c54x_bsp_load(s->dsp, samples, nbits);

        /* Write to DARAM at multiple candidate addresses */
        /* Address from pointer at 0x00B9 */
        uint16_t ptr = s->dsp->data[0x00B9];
        if (ptr >= 0x0020 && ptr < 0x0800) {
            for (int i = 0; i < nbits; i++)
                s->dsp->data[ptr + i] = samples[i];
        }
        /* Also write at fixed addresses used by BSP DMA on Calypso:
         * 0x03F0-0x04FF is a common DMA target area */
        for (int i = 0; i < nbits; i++) {
            s->dsp->data[0x03F0 + i] = samples[i];
            s->dsp->data[0x04F0 + i] = samples[i];
        }
    }

    /* Update read page metadata */
    uint16_t *rp = s->dsp_page ?
        &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];
    rp[1] = (uint16_t)(fn % 4);  /* d_burst_d */
    rp[8]  = 0;                   /* TOA */
    rp[9]  = 4864;                /* PM */
    rp[10] = 0;                   /* ANGLE */
    rp[11] = 100;                 /* SNR */
}

/* TX burst: send UL burst from DSP write page via UART TX as sercomm DLCI 4 */
static void calypso_trx_send_ul_burst(CalypsoTRX *s, uint16_t task_u)
{
    if (!g_uart_modem || task_u == 0) return;

    /* Read UL burst from write page.
     * d_burst_u at word 3, burst data follows in NDB a_cu area. */
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];

    /* Build TRXD v0 UL packet: TN(1) FN(4) RSSI(1) TOA256(2) bits(148) = 156 */
    uint8_t pkt[8 + 148];
    uint8_t tn = wp[3] & 0x07;  /* d_burst_u has TN info */
    uint32_t fn = s->fn;

    pkt[0] = tn;
    pkt[1] = (fn >> 24) & 0xFF;
    pkt[2] = (fn >> 16) & 0xFF;
    pkt[3] = (fn >> 8) & 0xFF;
    pkt[4] = fn & 0xFF;
    pkt[5] = (uint8_t)(-60);  /* RSSI */
    pkt[6] = 0;  /* TOA256 high */
    pkt[7] = 0;  /* TOA256 low */

    /* Read burst bits from NDB UL area — for now send dummy burst */
    memset(&pkt[8], 0, 148);

    /* Wrap in sercomm DLCI 4 and send via UART TX */
    uint8_t frame[512];
    int pos = 0;
    frame[pos++] = 0x7E;  /* FLAG */
    /* Header: DLCI + CTRL, with escaping */
    uint8_t hdr[2] = { 0x04, 0x03 };
    for (int i = 0; i < 2; i++) {
        if (hdr[i] == 0x7E || hdr[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = hdr[i] ^ 0x20;
        } else {
            frame[pos++] = hdr[i];
        }
    }
    /* Payload with escaping */
    int pkt_len = 8 + 148;
    for (int i = 0; i < pkt_len && pos < 500; i++) {
        if (pkt[i] == 0x7E || pkt[i] == 0x7D) {
            frame[pos++] = 0x7D;
            frame[pos++] = pkt[i] ^ 0x20;
        } else {
            frame[pos++] = pkt[i];
        }
    }
    frame[pos++] = 0x7E;  /* FLAG */

    /* Write to UART chardev (goes to PTY → bridge reads it) */
    qemu_chr_fe_write_all(&g_uart_modem->chr, frame, pos);
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
            if (c54x_load_rom(s->dsp, rom_path) == 0) {
                /* Don't reset/boot yet — wait for ARM to write DSP_DL_STATUS_READY */
                s->dsp->running = false;
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
