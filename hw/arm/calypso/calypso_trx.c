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
#include "hw/arm/calypso/calypso_bsp.h"
#include "hw/arm/calypso/calypso_iota.h"
#include "chardev/char-fe.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

extern CalypsoUARTState *g_uart_modem;

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

#define DSP_API_W_PAGE0  0x0000
#define DSP_API_W_PAGE1  0x0028
#define DSP_API_NDB      0x01A8
#define DB_W_D_TASK_D    0
#define DB_W_D_BURST_D   1
#define DB_W_D_TASK_U    2
#define DB_W_D_BURST_U   3
#define DB_W_D_TASK_MD   4
/* No PM/FB/SB stubs — the DSP handles everything via shared API RAM */

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
    QEMUTimer   *tdma_timer;
    QEMUTimer   *frame_irq_timer;
    QEMUTimer   *dsp_timer;
    uint32_t     fn;
    bool         tdma_running;
    uint8_t      sync_bsic;
    bool         fw_patched;

    /* C54x DSP emulator */
    C54xState   *dsp;
    bool         dsp_init_done;  /* DSP reached first IDLE after boot */

    /* CLK UDP: send each TDMA tick to bridge so it's clock-slave */
    int          clk_fd;
    struct sockaddr_in clk_peer;
} CalypsoTRX;

static CalypsoTRX *g_trx;

/* ---- FW patches ---- */
static void calypso_fw_patch(CalypsoTRX *s)
{
    if (s->fw_patched) return;
    s->fw_patched = true;
    uint32_t val, bx_lr = 0xe12fff1e, nop = 0xe1a00000;
    cpu_physical_memory_read(0x0082a1b0, &val, 4);
    if (val && val != bx_lr) { cpu_physical_memory_write(0x0082a1b0, &bx_lr, 4); TRX_LOG("FW: cons_puts nop"); }
    cpu_physical_memory_read(0x00829ea0, &val, 4);
    if (val && val != bx_lr) { cpu_physical_memory_write(0x00829ea0, &bx_lr, 4); TRX_LOG("FW: puts nop"); }
    static const hwaddr bl[] = {0x828914,0x828828,0x828830,0x828858,0x828880};
    for (int i = 0; i < 5; i++) {
        cpu_physical_memory_read(bl[i], &val, 4);
        if ((val & 0x0F000000) == 0x0B000000) { cpu_physical_memory_write(bl[i], &nop, 4); TRX_LOG("FW: NOP @0x%lx", (unsigned long)bl[i]); }
    }
    cpu_physical_memory_read(0x0082c33c, &val, 4);
    if (val == 0xe3530020) { uint32_t v = 0xe3530094; cpu_physical_memory_write(0x0082c33c, &v, 4); TRX_LOG("FW: talloc 32->148"); }
    cpu_physical_memory_read(0x0082c350, &val, 4);
    if (val == 0xeafffffe) { uint32_t p[2] = {0xe121f008, 0xeaffffdf}; cpu_physical_memory_write(0x0082c34c, p, 8); TRX_LOG("FW: talloc retry"); }
    cpu_physical_memory_read(0x00821f98, &val, 4);
    if (val == 0xeafffffe) { uint32_t v = 0xe321f000; cpu_physical_memory_write(0x00821f94, &v, 4); TRX_LOG("FW: abort irqs"); }
    /* sim_handler (0x0082266c) blocks the main loop waiting for SIM
     * responses that never come in our emulation. Patch it to return
     * immediately (BX LR) so l1a_l23_handler() can process L1CTL. */
    cpu_physical_memory_read(0x0082266c, &val, 4);
    if (val && val != bx_lr) { cpu_physical_memory_write(0x0082266c, &bx_lr, 4); TRX_LOG("FW: sim_handler nop"); }
}
void calypso_fw_patch_apply(void) { if (g_trx) calypso_fw_patch(g_trx); }

/* ---- DSP API RAM ---- */
static uint64_t calypso_dsp_read(void *opaque, hwaddr offset, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return 0;
    uint64_t val = (size == 2) ? s->dsp_ram[offset/2] :
                   (size == 4) ? (s->dsp_ram[offset/2] | ((uint32_t)s->dsp_ram[offset/2+1] << 16)) :
                   ((uint8_t *)s->dsp_ram)[offset];
    /* DSP boot handshake: firmware polls DL_STATUS until it reads BOOT */
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
    /* No stubs — the DSP writes results into dsp_ram (shared API RAM)
     * and the firmware reads them directly. No intercepts. */
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;
    if (size == 2) s->dsp_ram[offset/2] = value;
    else if (size == 4) { s->dsp_ram[offset/2] = value; s->dsp_ram[offset/2+1] = value >> 16; }
    else ((uint8_t *)s->dsp_ram)[offset] = value;

    /* Debug: log task-related writes to write pages (d_task_d/u/md/ra) */
    if ((offset == 0x0000 || offset == 0x0004 || offset == 0x0008 ||
         offset == 0x000E || offset == 0x0028 || offset == 0x002C ||
         offset == 0x0030 || offset == 0x0036) && value != 0) {
        static int wp_log = 0;
        if (++wp_log <= 100)
            TRX_LOG("DSP WR [0x%04x] = 0x%04x (sz=%d) fn=%u",
                    (unsigned)offset, (unsigned)value, size, s->fn);
    }

    /* Log task writes for debugging — no interception, no faking.
     * The DSP handles all tasks via shared API RAM. */
    {
        hwaddr w0_md = DSP_API_W_PAGE0 + DB_W_D_TASK_MD * 2;
        hwaddr w1_md = DSP_API_W_PAGE1 + DB_W_D_TASK_MD * 2;
        hwaddr w0_d  = DSP_API_W_PAGE0 + DB_W_D_TASK_D * 2;
        hwaddr w1_d  = DSP_API_W_PAGE1 + DB_W_D_TASK_D * 2;
        if ((offset == w0_md || offset == w1_md ||
             offset == w0_d  || offset == w1_d) && value != 0) {
            static int task_log = 0;
            if (++task_log <= 100)
                TRX_LOG("ARM TASK WR [0x%04x] = %u fn=%u",
                        (unsigned)offset, (unsigned)value, s->fn);
        }
    }
    /* DSP page */
    if (offset == DSP_API_NDB) s->dsp_page = value & 1;
    /* DSP status */
    if (offset == DSP_DL_STATUS_ADDR) {
        if (value == 0) { s->dsp_booted = false; s->boot_frame = 0; TRX_LOG("DSP reset"); }
        else if (value == DSP_DL_STATUS_READY) {
            s->dsp_ram[DSP_API_VER_ADDR/2] = DSP_API_VERSION;
            s->dsp_ram[DSP_API_VER2_ADDR/2] = 0;
            /* Unmask API IRQ (IRQ15) in INTH */
            {
                uint16_t mask;
                cpu_physical_memory_read(0xFFFFFA08, &mask, 2);
                mask &= ~(1 << 15);
                cpu_physical_memory_write(0xFFFFFA08, &mask, 2);
                TRX_LOG("DSP ready — unmasked API IRQ (mask=0x%04x)", mask);
            }
            /* Reset C54x DSP — boot runs in TDMA ticks (parallel with ARM) */
            if (s->dsp) {
                c54x_reset(s->dsp);
                s->dsp->running = true;
                s->dsp_init_done = false;
                s->dsp_ram[0x01A8/2] = 0;
                TRX_LOG("C54x DSP reset — boot via TDMA ticks");
            }
        }
    }
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read, .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size=1,.max_access_size=4}, .impl = {.min_access_size=1,.max_access_size=4},
};

/* ---- TPU ---- */
static void calypso_dsp_done(void *opaque) {
    CalypsoTRX *s = opaque;
    s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_EN;

    /* Hardware DMA: copy API write page → DSP DARAM 0x0586.
     * Triggered by firmware writing TPU_CTRL with EN bit (dsp_end_scenario).
     * This is the ONLY place DMA happens — same as real Calypso. */
    if (s->dsp && s->dsp_ram[0x01A8/2] != 0) {
        uint16_t page = s->dsp_ram[0x01A8/2] & 1;
        uint16_t *wp = page ?
            &s->dsp_ram[DSP_API_W_PAGE1/2] : &s->dsp_ram[DSP_API_W_PAGE0/2];

        /* Log proof that ARM wrote tasks before DMA */
        uint16_t task_d  = wp[DB_W_D_TASK_D];
        uint16_t task_u  = wp[DB_W_D_TASK_U];
        uint16_t task_md = wp[DB_W_D_TASK_MD];
        if (task_d || task_u || task_md) {
            static int dma_task_log = 0;
            if (++dma_task_log <= 50)
                TRX_LOG("DMA proof: ARM wrote task_d=%u task_u=%u task_md=%u page=%u fn=%u",
                        task_d, task_u, task_md, page, s->fn);
        }

        s->dsp->data[0x0584] = s->dsp_ram[0x01A8/2];
        s->dsp->data[0x0585] = s->fn & 0xFFFF;
        for (int i = 0; i < 20; i++)
            s->dsp->data[0x0586 + i] = wp[i];
        if (s->dsp->api_ram)
            s->dsp->api_ram[0x08D4 - C54X_API_BASE] = s->dsp_ram[0x01A8/2];
    }

    /* Execute TPU RAM micro-instructions (TSP bus commands).
     * The firmware wrote a TPU scenario into TPU RAM. We scan it for
     * MOVE instructions that write to TSP registers. When we see
     * TSP_CTRL2 with WR bit, we send the TX byte to IOTA.
     *
     * IMPORTANT: The Calypso Rhea bus is 16-bit wide, mapped to the
     * 32-bit ARM bus at 2-byte stride. The firmware writes 16-bit TPU
     * instructions at ARM offsets 0, 2, 4, ..., which end up in
     * tpu_ram[0], tpu_ram[1], tpu_ram[2], ... However, the actual
     * physical layout has zero-padding between instructions (ARM 32-bit
     * alignment). We must skip zero words that are just bus padding,
     * not real SLEEP instructions. A real SLEEP (0x0000) always comes
     * after at least one non-zero instruction. */
    {
        uint8_t tsp_tx1 = 0;
        uint8_t tsp_ctrl1 = 0;
        bool seen_any = false;
        for (int i = 0; i < CALYPSO_TPU_RAM_SIZE / 2; i++) {
            uint16_t insn = s->tpu_ram[i];
            if (insn == 0x0000) {
                /* Skip zero words: they are either Rhea bus padding
                 * or the final SLEEP. Only break on SLEEP after we've
                 * seen real instructions, and only if the NEXT word
                 * is also zero (two consecutive zeros = real SLEEP). */
                if (seen_any) {
                    int next = i + 1;
                    if (next >= CALYPSO_TPU_RAM_SIZE / 2 ||
                        s->tpu_ram[next] == 0x0000)
                        break;  /* real SLEEP — end of scenario */
                }
                continue;
            }
            seen_any = true;
            uint8_t opcode = (insn >> 13) & 0x7;
            if (opcode == 4) {
                /* MOVE: addr = bits 4:0, data = bits 12:5 */
                uint8_t addr = insn & 0x1F;
                uint8_t data = (insn >> 5) & 0xFF;
                if (addr == 0x04) tsp_tx1 = data;     /* TPUI_TX_1 */
                if (addr == 0x00) tsp_ctrl1 = data;    /* TPUI_TSP_CTRL1 */
                if (addr == 0x01 && (data & 0x02)) {   /* TPUI_TSP_CTRL2 WR bit */
                    /* TSP write: send tsp_tx1 to the device.
                     * Device 0 (TWL3025): 7-bit data = tsp_tx1.
                     * This byte contains BDLON/BDLENA/BULENA bits. */
                    uint8_t dev = (tsp_ctrl1 >> 5) & 0x07;
                    if (dev == 0) {
                        calypso_iota_tsp_write(tsp_tx1, 0);
                    }
                }
            }
        }
    }

    qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
}
static void calypso_tdma_start(CalypsoTRX *s);

/* Called by calypso_tint0.c on each TDMA frame tick.
 * Forward declaration — actual tdma_tick is defined below. */
static void calypso_tdma_tick(void *opaque);
/* Prototype visible to tint0 (declared extern there) */
void calypso_tint0_do_tick(uint32_t fn);
void calypso_tint0_do_tick(uint32_t fn)
{
    if (!g_trx) return;
    g_trx->fn = fn;
    /* d_dsp_page is toggled by the DSP firmware itself (PC=0x1748),
     * NOT by ARM or the emulator. Don't touch it here. */
    calypso_tdma_tick(g_trx);
}

static uint64_t calypso_tpu_read(void *o, hwaddr off, unsigned sz) {
    CalypsoTRX *s=o; if (off==TPU_IT_DSP_PG) return s->dsp_page;
    return (off/2<CALYPSO_TPU_SIZE/2)?s->tpu_regs[off/2]:0;
}
static void calypso_tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz) {
    CalypsoTRX *s=o; if (off/2<CALYPSO_TPU_SIZE/2) s->tpu_regs[off/2]=val;
    if (off==TPU_CTRL) {
        static int tpu_log = 0;
        if (++tpu_log <= 50)
            TRX_LOG("TPU_CTRL WR val=0x%04x (EN=%d DSP_EN=%d) fn=%u",
                    (unsigned)val, !!(val&TPU_CTRL_EN), !!(val&TPU_CTRL_DSP_EN), s->fn);
    }
    if (off==TPU_CTRL && (val&TPU_CTRL_EN)) {
        s->tpu_regs[TPU_CTRL/2] &= ~(TPU_CTRL_EN|TPU_CTRL_IDLE);
        /* DMA immediately — no timer delay. The firmware has already
         * finished writing the write page before setting TPU_CTRL_EN.
         * A 1ns timer caused a race condition where the DMA would fire
         * before the write page was fully populated. */
        calypso_dsp_done(s);
    }
    if (off==TPU_INT_CTRL && !(val&ICTRL_MCU_FRAME) && !s->tdma_running) calypso_tdma_start(s);
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

/* ---- TDMA ---- */
static void calypso_frame_irq_lower(void *o){qemu_irq_lower(((CalypsoTRX*)o)->irqs[CALYPSO_IRQ_TPU_FRAME]);}

static void calypso_tdma_tick(void *opaque) {
    CalypsoTRX *s = opaque;
    s->fn = (s->fn+1) % GSM_HYPERFRAME;

    /* ── 0. Send CLK tick to bridge (QEMU is clock master) ── */
    if (s->clk_fd >= 0) {
        uint8_t pkt[4];
        pkt[0] = (s->fn >> 24) & 0xFF;
        pkt[1] = (s->fn >> 16) & 0xFF;
        pkt[2] = (s->fn >>  8) & 0xFF;
        pkt[3] =  s->fn        & 0xFF;
        sendto(s->clk_fd, pkt, 4, 0,
               (struct sockaddr *)&s->clk_peer, sizeof(s->clk_peer));
    }

    /* ── 1. UART poll: deliver pending chardev bytes to firmware ── */
    if (g_uart_modem) {
        calypso_uart_poll_backend(g_uart_modem);
        calypso_uart_kick_rx(g_uart_modem);
    }

    /* ── 2. DSP boot phase ── */
    if (s->dsp && s->dsp->running && !s->dsp_init_done) {
        if (!s->dsp->idle)
            c54x_run(s->dsp, 256000);
        if (s->dsp->idle) {
            s->dsp_init_done = true;
            TRX_LOG("DSP init complete (first IDLE reached)");
        }
    }

    /* ── 3. DMA is NOT done here ──
     * On real Calypso, the TPU scenario triggers the DMA when the
     * firmware writes TPU_CTRL with EN bit. This happens in
     * calypso_dsp_done() (the TPU_CTRL_EN timer callback).
     * Doing DMA here would copy a STALE write page because the
     * firmware hasn't written the new tasks yet (it writes them
     * in l1s_compl() which runs in the IRQ4 handler AFTER this tick). */

    /* ── 4. DSP frame interrupt ──
     * Send INT3 only when firmware requests it via TPU_CTRL_DSP_EN.
     * Sending INT3 every frame causes stack overflow (CALLD loop). */
    if (s->dsp && s->dsp->running && s->dsp_init_done) {
        bool was_idle = s->dsp->idle;

        if (s->tpu_regs[TPU_CTRL/2] & TPU_CTRL_DSP_EN) {
            c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
            s->tpu_regs[TPU_CTRL/2] &= ~TPU_CTRL_DSP_EN;
        }

        /* ── 5. Run DSP ── */
        if (!s->dsp->idle) {
            c54x_run(s->dsp, 256000);
        }

        /* Do NOT clear tasks here — the firmware's l1s_compl() does
         * dsp_api_memset() on the write page at the start of each frame,
         * before tdma_sched_execute() writes new tasks. Clearing here
         * would erase tasks that the scheduler just programmed. */

        /* Only pulse API IRQ when DSP naturally reaches IDLE. */
        if (!was_idle && s->dsp->idle) {
            qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]);
        }
    }

    /* ── 6. Deliver buffered DL bursts to DSP ──
     * Bursts from BTS arrive via UDP in real time, but BDLENA windows
     * open in virtual time (faster). This step pulls buffered bursts
     * and delivers them when BDLENA windows are available. */
    calypso_bsp_deliver_buffered(s->fn);

    /* ── 6b. UL burst poll ──
     * Check if the DSP wrote an UL task. If so, read bits from DSP
     * DARAM 0x0900 and send via UDP to BTS. */
    {
        uint16_t *wp = s->dsp_page ?
            &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];
        uint16_t task_u = wp[DB_W_D_TASK_U];
        if (task_u != 0 && s->dsp) {
            uint8_t tn = wp[DB_W_D_BURST_U] & 0x07;
            uint8_t bits[148];
            if (calypso_bsp_tx_burst(tn, s->fn, bits)) {
                calypso_bsp_send_ul(tn, s->fn, bits);
                static int ul_log = 0;
                if (++ul_log <= 20)
                    TRX_LOG("UL burst task=%u tn=%u fn=%u", task_u, tn, s->fn);
            }
            wp[DB_W_D_TASK_U] = 0;
        }
    }

    /* ── 7. TPU FRAME IRQ → ARM L1 scheduler ── */
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);

    /* ── 8. Re-arm TDMA timer ── */
    if (s->tdma_running)
        timer_mod_ns(s->tdma_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
}

static void calypso_tdma_start(CalypsoTRX *s)
{
    if (s->tdma_running) return;
    s->tdma_running = true;
    s->fn = 0;
    TRX_LOG("TDMA started");
    timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + GSM_TDMA_NS);
}

/* ---- kick ---- */
static QEMUTimer *g_kick_timer;
static void calypso_kick_cb(void *o){CPUState*cpu=first_cpu;if(cpu)cpu_exit(cpu);qemu_notify_event();timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);}

/* ---- Sercomm burst transport (DLCI 4) ---- */

/* RX burst from bridge (DL) — store in DSP RAM for firmware to read */
void calypso_trx_rx_burst(const uint8_t *data, int len)
{
    if (!g_trx || len < 8) return;
    CalypsoTRX *s = g_trx;

    uint8_t tn = data[0] & 0x07;
    uint32_t fn = ((uint32_t)data[1]<<24)|((uint32_t)data[2]<<16)|
                  ((uint32_t)data[3]<<8)|(uint32_t)data[4];

    /* Sync FN */
    s->fn = fn % GSM_HYPERFRAME;

    static int rx_count = 0;
    if (++rx_count <= 5 || (rx_count % 1000) == 0)
        TRX_LOG("RX_BURST #%d TN=%d FN=%u len=%d", rx_count, tn, fn, len);

    /* No stubs — bursts go to BSP via UDP (calypso_bsp.c), not here.
     * The DSP processes them and writes results to shared API RAM. */
    (void)tn;
}

/* TX burst: send UL burst from DSP write page via UART TX as sercomm DLCI 4 */
static void calypso_trx_send_ul_burst(CalypsoTRX *s, uint16_t task_u)
{
    if (!g_uart_modem || task_u == 0) return;

    /* Read UL burst from write page.
     * d_burst_u at word 3, burst data follows in NDB a_cu area. */
    uint16_t *wp = s->dsp_page ?
        &s->dsp_ram[DSP_API_W_PAGE1 / 2] : &s->dsp_ram[DSP_API_W_PAGE0 / 2];

    /* Build TRXD v0 TX packet: TN(1) FN(4) PWR(1) bits(148) */
    uint8_t pkt[6 + 148];
    uint8_t tn = wp[3] & 0x07;  /* d_burst_u has TN info */
    uint32_t fn = s->fn;

    pkt[0] = tn;
    pkt[1] = (fn >> 24) & 0xFF;
    pkt[2] = (fn >> 16) & 0xFF;
    pkt[3] = (fn >> 8) & 0xFF;
    pkt[4] = fn & 0xFF;
    pkt[5] = 0;  /* TX power */

    /* Read burst bits from NDB UL area — for now send dummy burst */
    memset(&pkt[6], 0, 148);

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
    int pkt_len = 6 + 148;
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
    g_trx = s; s->irqs = irqs; s->sync_bsic = 7;
    s->clk_fd = -1;
    TRX_LOG("=== Calypso hardware init ===");

    memory_region_init_io(&s->dsp_iomem,NULL,&calypso_dsp_ops,s,"calypso.dsp_api",CALYPSO_DSP_SIZE);
    memory_region_add_subregion(sysmem,CALYPSO_DSP_BASE,&s->dsp_iomem);
    s->dsp_ram[DSP_DL_STATUS_ADDR/2]=DSP_DL_STATUS_READY; s->dsp_ram[DSP_API_VER_ADDR/2]=DSP_API_VERSION; s->dsp_booted=true;

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

    s->tdma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_tdma_tick,s);
    s->dsp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_dsp_done,s);
    s->frame_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,calypso_frame_irq_lower,s);

    g_kick_timer = timer_new_ns(QEMU_CLOCK_REALTIME,calypso_kick_cb,NULL);
    timer_mod_ns(g_kick_timer,qemu_clock_get_ns(QEMU_CLOCK_REALTIME)+5000000);

    /* C54x DSP emulator */
    {
        const char *rom_path = getenv("CALYPSO_DSP_ROM");
        if (!rom_path) rom_path = "/opt/GSM/calypso_dsp.txt";
        s->dsp = c54x_init();
        if (s->dsp) {
            c54x_set_api_ram(s->dsp, s->dsp_ram);
            if (c54x_load_rom(s->dsp, rom_path) == 0) {
                c54x_reset(s->dsp);
                calypso_bsp_init(s->dsp);
                TRX_LOG("C54x DSP loaded from %s", rom_path);
            } else {
                TRX_LOG("C54x DSP ROM not found at %s", rom_path);
                free(s->dsp);
                s->dsp = NULL;
            }
        }
    }

    TRX_LOG("=== Hardware ready ===");

    /* CLK UDP: QEMU sends TDMA ticks to bridge on port 6700.
     * Bridge is clock-slave — no independent timer. */
    {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd >= 0) {
            fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
            s->clk_fd = fd;
            memset(&s->clk_peer, 0, sizeof(s->clk_peer));
            s->clk_peer.sin_family = AF_INET;
            s->clk_peer.sin_port = htons(6700);
            s->clk_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            TRX_LOG("CLK UDP → bridge 127.0.0.1:6700");
        }
    }
}
