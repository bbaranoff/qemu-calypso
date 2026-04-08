/*
 * calypso_c54x.h — TMS320C54x DSP emulator for Calypso
 *
 * Emulates the C54x DSP core found in the TI Calypso baseband chip.
 * Loads ROM dump, executes instructions, shares API RAM with ARM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_C54X_H
#define CALYPSO_C54X_H

#include <stdint.h>
#include <stdbool.h>

/* Memory sizes (in 16-bit words) */
#define C54X_PROG_SIZE   0x40000  /* 256K words program space */
#define C54X_DATA_SIZE   0x10000  /* 64K words data space */
#define C54X_IO_SIZE     0x10000  /* 64K words I/O space */

/* API RAM: shared between ARM and DSP */
#define C54X_API_BASE    0x0800   /* DSP data address of API RAM */
#define C54X_API_SIZE    0x2000   /* 8K words */

/* DSP start address (after boot) */
#define C54X_DSP_START   0x7000

/* MMR addresses (data memory 0x00-0x1F) */
#define MMR_IMR   0x00
#define MMR_IFR   0x01
#define MMR_ST0   0x06
#define MMR_ST1   0x07
#define MMR_AL    0x08
#define MMR_AH    0x09
#define MMR_AG    0x0A
#define MMR_BL    0x0B
#define MMR_BH    0x0C
#define MMR_BG    0x0D
#define MMR_T     0x0E
#define MMR_TRN   0x0F
#define MMR_AR0   0x10
#define MMR_AR1   0x11
#define MMR_AR2   0x12
#define MMR_AR3   0x13
#define MMR_AR4   0x14
#define MMR_AR5   0x15
#define MMR_AR6   0x16
#define MMR_AR7   0x17
#define MMR_SP    0x18
#define MMR_BK    0x19
#define MMR_BRC   0x1A
#define MMR_RSA   0x1B
#define MMR_REA   0x1C
#define MMR_PMST  0x1D
#define MMR_XPC   0x1E

/* Timer registers (memory-mapped at 0x0024-0x0026) */
#define TIM_ADDR  0x0024   /* Timer counter */
#define PRD_ADDR  0x0025   /* Timer period */
#define TCR_ADDR  0x0026   /* Timer control */

/* TCR bit positions (TMS320C54x hardware spec) */
#define TCR_TDDR_MASK  0x000F   /* bits 3:0 — prescaler reload value */
#define TCR_TSS        (1 << 4) /* bit 4 — Timer Stop Status (1=stopped) */
#define TCR_TRB        (1 << 5) /* bit 5 — Timer Reload (write 1 reloads) */
#define TCR_PSC_SHIFT  6        /* bits 9:6 — prescale counter */
#define TCR_PSC_MASK   (0xF << TCR_PSC_SHIFT)
#define TCR_SOFT       (1 << 10)
#define TCR_FREE       (1 << 11)

/* ST0 bit positions */
#define ST0_DP_MASK  0x01FF  /* bits 8-0: data page pointer */
#define ST0_OVB      (1 << 9)
#define ST0_OVA      (1 << 10)
#define ST0_C        (1 << 11)
#define ST0_TC       (1 << 12)
#define ST0_ARP_SHIFT 13
#define ST0_ARP_MASK (7 << ST0_ARP_SHIFT)

/* ST1 bit positions */
#define ST1_ASM_MASK 0x001F  /* bits 4-0: accumulator shift mode */
#define ST1_CMPT     (1 << 5)
#define ST1_FRCT     (1 << 6)
#define ST1_C16      (1 << 7)
#define ST1_SXM      (1 << 8)
#define ST1_OVM      (1 << 9)
#define ST1_INTM     (1 << 11)
#define ST1_HM       (1 << 12)
#define ST1_XF       (1 << 13)
#define ST1_BRAF     (1 << 14)

/* PMST bit positions (per SPRU131: SST=0 SMUL=1 CLKOFF=2 DROM=3 APTS=4 OVLY=5 MP/MC=6) */
#define PMST_SST     (1 << 0)
#define PMST_SMUL    (1 << 1)
#define PMST_CLKOFF  (1 << 2)
#define PMST_DROM    (1 << 3)
#define PMST_APTS    (1 << 4)
#define PMST_OVLY    (1 << 5)
#define PMST_MP_MC   (1 << 6)
#define PMST_IPTR_SHIFT 7
#define PMST_IPTR_MASK (0x1FF << PMST_IPTR_SHIFT)

/* Interrupt vectors */
#define C54X_INT_RESET   0
#define C54X_INT_NMI     1
/* TMS320C54x interrupt mapping: vector = IMR_bit + 2
 * IMR bit 0 → INT0 → vec 2    IMR bit 5 → BRINT0 → vec 7
 * IMR bit 1 → INT1 → vec 3    IMR bit 6 → BXINT0 → vec 8
 * IMR bit 2 → INT2 → vec 4    IMR bit 7 → DMAC0  → vec 9
 * IMR bit 3 → INT3 → vec 5    IMR bit 8 → DMAC1  → vec 10
 * IMR bit 4 → TINT0→ vec 6    IMR bit 9 → INT4   → vec 11
 *                              IMR bit 10→ INT5   → vec 12 */
/* TMS320C54x interrupt vector mapping (SPRU131):
 * Vec 0: RESET     Vec 16: INT0 (IMR bit 0)
 * Vec 1: NMI       Vec 17: INT1 (IMR bit 1)
 * Vec 2: SINT17    Vec 18: INT2 (IMR bit 2)
 * Vec 3: SINT18    Vec 19: INT3 (IMR bit 3)
 * Vec 4: SINT19    Vec 20: TINT0 (IMR bit 4)
 * Vec 5: SINT20    Vec 21: BRINT0 (IMR bit 5)
 * ...              Vec 22: BXINT0 (IMR bit 6)
 *                  Vec 23: DMAC0 (IMR bit 7)
 *                  Vec 24: DMAC1 (IMR bit 8)
 * Formula: vec = imr_bit + 16 */
/* Calypso DSP firmware enables IMR bits 3 + 7 + upper (observed IMR=0xFF88).
 * Bit 3 = INT3 = vec 19 — this is the external frame-sync line from the TPU,
 * the only "frame" interrupt the firmware actually unmasks. Use it. */
#define C54X_INT_FRAME_VEC   19  /* INT3 = vec (3+16) */
#define C54X_INT_FRAME_BIT   3   /* IMR bit 3 */
#define C54X_NUM_INTS        16

typedef struct C54xState {
    /* Accumulators (40-bit) stored as int64 for convenience */
    int64_t a;   /* A accumulator: bits 39-0 */
    int64_t b;   /* B accumulator: bits 39-0 */

    /* Auxiliary registers */
    uint16_t ar[8];

    /* Other registers */
    uint16_t t;      /* Temporary register */
    uint16_t trn;    /* Transition register (Viterbi) */
    uint16_t sp;
    uint16_t bk;     /* Circular buffer size */
    uint16_t brc;    /* Block repeat counter */
    uint16_t rsa;    /* Block repeat start address */
    uint16_t rea;    /* Block repeat end address */

    /* Status registers */
    uint16_t st0;
    uint16_t st1;
    uint16_t pmst;

    /* Interrupt registers */
    uint16_t imr;
    uint16_t ifr;

    /* Program counter */
    uint32_t pc;     /* 16-bit (or 23-bit with XPC) */
    uint16_t xpc;

    /* Timer0 prescale counter (PSC) — not memory-mapped directly */
    uint16_t timer_psc;

    /* RPT state */
    uint16_t rpt_count;  /* remaining RPT iterations */
    uint16_t rpt_pc;     /* PC of repeated instruction */
    bool     rpt_active;
    uint16_t par;        /* Program Address Register (for READA/WRITA/MACD/MACP) */
    bool     par_set;
    bool     lk_used;    /* resolve_smem consumed extra word for lk */
    uint16_t mvpd_src;   /* MVPD auto-increment source address during RPT */

    /* RPTB state */
    bool     rptb_active;

    /* Delayed-branch state (CALLD/RETD/BD/CCD/...): when set, the next
     * `delay_slots` instructions execute normally, then PC is forced to
     * `delayed_pc`. */
    uint16_t delayed_pc;
    uint8_t  delay_slots;

    /* Memory */
    uint16_t prog[C54X_PROG_SIZE];   /* Program memory */
    uint16_t data[C54X_DATA_SIZE];   /* Data memory */

    /* API RAM pointer (shared with ARM calypso_trx.c) */
    uint16_t *api_ram;  /* points into ARM's dsp_ram[] */

    /* DSP → ARM notify hook: called whenever the DSP writes to api_ram. */
    void (*api_write_cb)(void *opaque, uint16_t woff, uint16_t val);
    void  *api_write_cb_opaque;

    /* State */
    bool     running;
    bool     idle;       /* IDLE instruction executed */
    uint64_t cycles;
    uint32_t insn_count;

    /* BSP (Baseband Serial Port) — burst sample buffer */
    uint16_t bsp_buf[160];  /* burst samples from radio */
    int      bsp_len;       /* number of samples */
    int      bsp_pos;       /* read position */

    /* Debug */
    uint32_t unimpl_count;
    uint16_t last_unimpl;
} C54xState;

/* Feed burst samples to BSP (called by calypso_trx) */
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

/* Create and initialize C54x state */
C54xState *c54x_init(void);

/* Load ROM dump from text file */
int c54x_load_rom(C54xState *s, const char *path);

/* Link API RAM (shared memory with ARM) */
void c54x_set_api_ram(C54xState *s, uint16_t *api_ram);

/* Reset the DSP */
void c54x_reset(C54xState *s);

/* Execute N instructions (returns actual count executed) */
int c54x_run(C54xState *s, int n_insns);

/* Raise an interrupt */
/* Send interrupt: vec = vector number (for PC), imr_bit = bit in IMR/IFR */
void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit);

/* Wake from IDLE */
void c54x_wake(C54xState *s);

#endif /* CALYPSO_C54X_H */
