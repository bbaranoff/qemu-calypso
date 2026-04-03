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

extern CalypsoUARTState *g_uart_modem;

#define TRX_LOG(fmt, ...) \
    fprintf(stderr, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)

#define DSP_API_W_PAGE0  0x0000
#define DSP_API_W_PAGE1  0x0028
#define DSP_API_NDB      0x01A8
#define DB_W_D_TASK_D    0
#define DB_W_D_TASK_U    2
#define DB_W_D_TASK_MD   4
#define PM_RAW_STRONG    4864

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
    uint32_t     fb_tasks;
    uint32_t     sb_tasks;
    uint8_t      sync_bsic;
    bool         fw_patched;
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
    /* PM done flags */
    if (offset == 0x003E || offset == 0x0016) val = 1;
    /* d_fb_det */
    if (offset == 0x01F0 && s->fb_tasks >= 1) val = 1;
    /* PM level */
    if (offset == 0x01AA && s->dsp_booted) val = PM_RAW_STRONG;
    return val;
}

static void calypso_dsp_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    CalypsoTRX *s = opaque;
    if (offset >= CALYPSO_DSP_SIZE) return;
    if (size == 2) s->dsp_ram[offset/2] = value;
    else if (size == 4) { s->dsp_ram[offset/2] = value; s->dsp_ram[offset/2+1] = value >> 16; }
    else ((uint8_t *)s->dsp_ram)[offset] = value;

    /* Intercept DSP tasks */
    {
        hwaddr w0 = DSP_API_W_PAGE0 + DB_W_D_TASK_MD * 2;
        hwaddr w1 = DSP_API_W_PAGE1 + DB_W_D_TASK_MD * 2;
        hwaddr w0d = DSP_API_W_PAGE0 + DB_W_D_TASK_D * 2;
        hwaddr w1d = DSP_API_W_PAGE1 + DB_W_D_TASK_D * 2;
        if ((offset == w0 || offset == w1 || offset == w0d || offset == w1d) && value != 0) {
            uint16_t pm = PM_RAW_STRONG;
            if (value == 5) {
                s->dsp_ram[0x01F0/2] = 1;
                s->dsp_ram[0x01F4/2] = 0;
                s->dsp_ram[0x01F6/2] = pm;
                s->dsp_ram[0x01F8/2] = 0;
                s->dsp_ram[0x01FA/2] = 100;
                s->fb_tasks++;
            } else if (value == 6) {
                uint16_t *rp = s->dsp_page ? &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];
                uint32_t fn = s->fn;
                uint8_t bsic = s->sync_bsic;
                uint32_t t1 = fn / (26*51), t2 = fn % 26, t3 = fn % 51;
                uint32_t t3p = (t3 >= 1) ? (t3-1)/10 : 0;
                uint32_t sb = ((uint32_t)(bsic&0x3F)<<2)|((t2&0x1F)<<18)|((t1&1)<<23)|(((t1>>1)&0xFF)<<8)|(((t1>>9)&1)<<0)|(((t1>>10)&1)<<1)|((t3p&1)<<24)|(((t3p>>1)&1)<<16)|(((t3p>>2)&1)<<17);
                rp[15]=0; rp[16]=0; rp[17]=0; rp[18]=sb&0xFFFF; rp[19]=sb>>16;
                s->dsp_ram[0x01F4/2]=0; s->dsp_ram[0x01F6/2]=pm; s->dsp_ram[0x01F8/2]=0; s->dsp_ram[0x01FA/2]=100;
                s->sb_tasks++;
            }
            s->dsp_ram[0x0060/2]=pm; s->dsp_ram[0x0062/2]=pm; s->dsp_ram[0x0064/2]=pm;
            s->dsp_ram[0x0088/2]=pm; s->dsp_ram[0x008A/2]=pm; s->dsp_ram[0x008C/2]=pm;
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
            TRX_LOG("DSP ready");
        }
    }
}

static const MemoryRegionOps calypso_dsp_ops = {
    .read = calypso_dsp_read, .write = calypso_dsp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {.min_access_size=1,.max_access_size=4}, .impl = {.min_access_size=1,.max_access_size=4},
};

/* ---- TPU ---- */
static void calypso_dsp_done(void *opaque) { qemu_irq_pulse(((CalypsoTRX*)opaque)->irqs[CALYPSO_IRQ_API]); }
static void calypso_tdma_start(CalypsoTRX *s);

static uint64_t calypso_tpu_read(void *o, hwaddr off, unsigned sz) {
    CalypsoTRX *s=o; if (off==TPU_IT_DSP_PG) return s->dsp_page;
    return (off/2<CALYPSO_TPU_SIZE/2)?s->tpu_regs[off/2]:0;
}
static void calypso_tpu_write(void *o, hwaddr off, uint64_t val, unsigned sz) {
    CalypsoTRX *s=o; if (off/2<CALYPSO_TPU_SIZE/2) s->tpu_regs[off/2]=val;
    if (off==TPU_CTRL && (val&TPU_CTRL_EN)) {
        s->tpu_regs[TPU_CTRL/2] &= ~(TPU_CTRL_EN|TPU_CTRL_IDLE);
        timer_mod_ns(s->dsp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+1);
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
    if (g_uart_modem) { calypso_uart_poll_backend(g_uart_modem); calypso_uart_kick_rx(g_uart_modem); }
    /* PM every frame */
    { uint16_t pm=PM_RAW_STRONG;
      for(int i=0;i<4;i++){s->dsp_ram[(0x0060+i*2)/2]=pm;s->dsp_ram[(0x0088+i*2)/2]=pm;}
      s->dsp_ram[213]=pm; for(int i=0;i<8;i++)s->dsp_ram[248+i]=pm; }
    qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME]);
    timer_mod_ns(s->frame_irq_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+1000000);
    if (s->tdma_running) timer_mod_ns(s->tdma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+GSM_TDMA_NS);
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

    /* Store burst in read page for NB tasks.
     * d_burst_d at word 1 of read page, burst data follows. */
    uint16_t *rp = s->dsp_page ?
        &s->dsp_ram[0x0078/2] : &s->dsp_ram[0x0050/2];

    int nbits = len - 8;
    if (nbits > 148) nbits = 148;

    /* Convert soft bits to hard bits and store in NDB a_cd area */
    uint16_t *ndb_cd = &s->dsp_ram[(0x01A8 + 0x1FC)/2]; /* NDB + a_cd offset */
    /* Store burst count */
    rp[1] = (uint16_t)(s->fn % 4); /* d_burst_d = burst index 0..3 */

    /* a_serv_demod: TOA, PM, ANGLE, SNR */
    rp[8]  = 0;              /* TOA */
    rp[9]  = PM_RAW_STRONG;  /* PM */
    rp[10] = 0;              /* ANGLE */
    rp[11] = 100;            /* SNR */

    (void)tn;
    (void)ndb_cd;
    (void)nbits;
}

/* TX burst poll — called each TDMA frame to check if firmware wrote a TX burst */
void calypso_trx_tx_burst_poll(void)
{
    if (!g_trx) return;
    /* TODO: read UL burst from DSP write page and send via UART TX as sercomm DLCI 4 */
}

/* ---- Init ---- */
void calypso_trx_init(MemoryRegion *sysmem, qemu_irq *irqs)
{
    CalypsoTRX *s = g_new0(CalypsoTRX, 1);
    g_trx = s; s->irqs = irqs; s->sync_bsic = 7;
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
    TRX_LOG("=== Hardware ready ===");
}
