/*
 * calypso_c54x.c — TMS320C54x DSP emulator for Calypso
 *
 * Minimal C54x core: enough to run the Calypso DSP ROM for GSM
 * signal processing (Viterbi, deinterleaving, burst decode).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "calypso_c54x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_boot_trace = 0;

#define C54_LOG(fmt, ...) \
    fprintf(stderr, "[c54x] " fmt "\n", ##__VA_ARGS__)

/* ================================================================
 * Helpers
 * ================================================================ */

/* Sign-extend 40-bit accumulator */
static inline int64_t sext40(int64_t v)
{
    if (v & ((int64_t)1 << 39))
        v |= ~(((int64_t)1 << 40) - 1);
    else
        v &= ((int64_t)1 << 40) - 1;
    return v;
}

/* Saturate 40-bit to 32-bit (OVM mode) */
static inline int64_t sat32(int64_t v)
{
    if (v > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (v < (int64_t)(int32_t)0x80000000) return (int64_t)(int32_t)0x80000000;
    return v;
}

/* Get ARP from ST0 */
static inline int arp(C54xState *s)
{
    return (s->st0 >> ST0_ARP_SHIFT) & 7;
}

/* Get DP from ST0 */
static inline uint16_t dp(C54xState *s)
{
    return s->st0 & ST0_DP_MASK;
}

/* Get ASM from ST1 (5-bit signed) */
static inline int asm_shift(C54xState *s)
{
    int v = s->st1 & ST1_ASM_MASK;
    if (v & 0x10) v |= ~0x1F;  /* sign extend */
    return v;
}

/* ================================================================
 * Memory access
 * ================================================================ */

static uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* Log reads from API RAM at 0x08D4 (d_dsp_page) */
    if (addr == 0x08D4) {
        static int dsp_page_log = 0;
        if (dsp_page_log < 50) {
            C54_LOG("d_dsp_page RD = 0x%04x PC=0x%04x insn=%u SP=0x%04x",
                    s->api_ram ? s->api_ram[addr - 0x0800] : s->data[addr],
                    s->pc, s->insn_count, s->sp);
            dsp_page_log++;
        }
    }
    /* Timer registers (0x0024-0x0026) — read returns current value */
    if (addr == TIM_ADDR) return s->data[TIM_ADDR];
    if (addr == PRD_ADDR) return s->data[PRD_ADDR];
    if (addr == TCR_ADDR) {
        /* TCR: PSC is read from bits 9:6, rest from stored value */
        uint16_t tcr = s->data[TCR_ADDR] & ~TCR_PSC_MASK;
        tcr |= (s->timer_psc & 0xF) << TCR_PSC_SHIFT;
        return tcr;
    }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:  return s->imr;
        case MMR_IFR:
        {
            static int ifr_log = 0;
            if ((s->ifr & 0x0020) && ifr_log < 10) {
                C54_LOG("IFR READ=0x%04x (TINT0!) PC=0x%04x", s->ifr, s->pc);
                ifr_log++;
            }
            return s->ifr;
        }
        case MMR_ST0:  return s->st0;
        case MMR_ST1:  return s->st1;
        case MMR_AL:   return (uint16_t)(s->a & 0xFFFF);
        case MMR_AH:   return (uint16_t)((s->a >> 16) & 0xFFFF);
        case MMR_AG:   return (uint16_t)((s->a >> 32) & 0xFF);
        case MMR_BL:   return (uint16_t)(s->b & 0xFFFF);
        case MMR_BH:   return (uint16_t)((s->b >> 16) & 0xFFFF);
        case MMR_BG:   return (uint16_t)((s->b >> 32) & 0xFF);
        case MMR_T:    return s->t;
        case MMR_TRN:  return s->trn;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            return s->ar[addr - MMR_AR0];
        case MMR_SP:   return s->sp;
        case MMR_BK:   return s->bk;
        case MMR_BRC:  return s->brc;
        case MMR_RSA:  return s->rsa;
        case MMR_REA:  return s->rea;
        case MMR_PMST: return s->pmst;
        case MMR_XPC:  return s->xpc;
        default: return 0;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram) {
            uint16_t val = s->api_ram[addr - C54X_API_BASE];
            /* Log ALL API reads during interrupt handler (first 100) */
            static int api_rd_log = 0;
            if (api_rd_log < 100 && s->insn_count > 66000) {
                C54_LOG("API RD [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                        addr, val, s->pc, s->insn_count);
                api_rd_log++;
            }
            return val;
        }
    }

    /* Log data reads during SINT17 handler (PC in 0xFFC0-0xFFFF) */
    if (s->pc >= 0xFFC0 && s->insn_count > 66090) {
        static int handler_rd_log = 0;
        if (handler_rd_log < 30) {
            C54_LOG("H_RD [0x%04x]=0x%04x PC=0x%04x", addr, s->data[addr], s->pc);
            handler_rd_log++;
        }
    }

    return s->data[addr];
}

static void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    /* Timer registers (0x0024-0x0026) — before MMR check */
    if (addr == TCR_ADDR) {
        /* TRB: write 1 → reload TIM from PRD, PSC from TDDR */
        if (val & TCR_TRB) {
            s->data[TIM_ADDR] = s->data[PRD_ADDR];
            s->timer_psc = val & TCR_TDDR_MASK;
        }
        /* Store TCR without TRB (TRB is write-only, always reads 0) */
        s->data[TCR_ADDR] = val & ~TCR_TRB;
        return;
    }
    if (addr == TIM_ADDR) { s->data[TIM_ADDR] = val; return; }
    if (addr == PRD_ADDR) { s->data[PRD_ADDR] = val; return; }

    /* MMR region */
    if (addr < 0x20) {
        switch (addr) {
        case MMR_IMR:
            if (val != s->imr)
                C54_LOG("IMR change 0x%04x → 0x%04x PC=0x%04x", s->imr, val, s->pc);
            s->imr = val; return;
        case MMR_IFR:  s->ifr &= ~val; return;  /* write 1 to clear */
        case MMR_ST0:  s->st0 = val; return;
        case MMR_ST1:  s->st1 = val; return;
        case MMR_AL:   s->a = (s->a & ~0xFFFF) | val; return;
        case MMR_AH:   s->a = (s->a & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_AG:   s->a = (s->a & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_BL:   s->b = (s->b & ~0xFFFF) | val; return;
        case MMR_BH:   s->b = (s->b & ~((int64_t)0xFFFF << 16)) | ((int64_t)val << 16); return;
        case MMR_BG:   s->b = (s->b & 0xFFFFFFFF) | ((int64_t)(val & 0xFF) << 32); return;
        case MMR_T:    s->t = val; return;
        case MMR_TRN:  s->trn = val; return;
        case MMR_AR0: case MMR_AR1: case MMR_AR2: case MMR_AR3:
        case MMR_AR4: case MMR_AR5: case MMR_AR6: case MMR_AR7:
            s->ar[addr - MMR_AR0] = val; return;
        case MMR_SP:   s->sp = val; return;
        case MMR_BK:   s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u",
                        s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count);
                /* IPTR relocation is normal — ROM changes interrupt vector base */
            }
            s->pmst = val; return;
        case MMR_XPC:
            /* Calypso DSP doesn't use extended program space.
             * Some code writes arbitrary values here during block copies. */
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram)
            s->api_ram[addr - C54X_API_BASE] = val;
        /* Always log writes to d_dsp_page (0x08D4) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u", val, s->pc, s->insn_count);
        }
    }

    /* Log DARAM writes to code target area and count total */
    if (addr >= 0x0020 && addr < 0x0800) {
        static int dw_total = 0;
        dw_total++;
        if (addr >= 0x1200 && addr <= 0x1240) {
            C54_LOG("DARAM WR [0x%04x] = 0x%04x PC=0x%04x insn=%u",
                    addr, val, s->pc, s->insn_count);
        }
        if (dw_total == 1 || dw_total == 100 || dw_total == 1000 || dw_total == 10000)
            C54_LOG("DARAM write count: %d (last: [0x%04x]=0x%04x)", dw_total, addr, val);
    }

    s->data[addr] = val;
}

/* Instruction fetch: uses mirrored PROM1 at 0x8000-0xFFFF, ignores XPC */
static uint16_t prog_fetch(C54xState *s, uint16_t pc)
{
    /* OVLY: map DARAM into program space 0x0080-0x27FF only.
     * Calypso has 10K words DARAM (0x0000-0x27FF).
     * PROM0 (0x7000-0xDFFF) is always accessible in program space. */
    if ((s->pmst & PMST_OVLY) && pc >= 0x80 && pc < 0x2800)
        return s->data[pc];
    /* prog_fetch: PC is always 16-bit, never uses XPC banking.
     * Per version 222807: only OVLY overlay applies to instruction fetch.
     * XPC is only used by prog_read (data/operand reads). */
    return s->prog[pc];
}
static uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* OVLY: DARAM visible in program space for 0x0080-0x27FF */
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        return s->data[addr16];
    /* For addresses >= 0x8000: use XPC to select extended page.
     * prog_read is used for data/operand reads (MVPD, FIRS coeff, etc.)
     * which need XPC banking — unlike prog_fetch which is PC-only. */
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        return s->prog[ext];
    }
    return s->prog[addr16];
}

static void __attribute__((unused)) prog_write(C54xState *s, uint32_t addr, uint16_t val)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* PROM1 (0xE000-0xFFFF) is ROM — reject writes */
    if (addr16 >= 0xE000) return;
    if ((s->pmst & PMST_OVLY) && addr16 >= 0x80 && addr16 < 0x2800)
        s->data[addr16] = val;
    if (addr16 >= 0x8000) {
        uint32_t ext = ((uint32_t)s->xpc << 16) | addr16;
        ext &= (C54X_PROG_SIZE - 1);
        s->prog[ext] = val;
    }
    s->prog[addr16] = val;
}

/* ================================================================
 * Addressing mode helpers
 * ================================================================ */

/* Resolve Smem operand: direct or indirect addressing.
 * Returns the data memory address. */
static uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)
{
    if (opcode & 0x80) {
        /* Indirect addressing */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = arp(s);
        uint16_t addr = s->ar[cur_arp];

        /* Post-modify */
        switch (mod) {
        case 0x0: /* *ARn */
            break;
        case 0x1: /* *ARn- */
            s->ar[cur_arp]--;
            break;
        case 0x2: /* *ARn+ */
            s->ar[cur_arp]++;
            break;
        case 0x3: /* *+ARn */
            addr = ++s->ar[cur_arp];
            break;
        case 0x4: /* *ARn-0 */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x5: /* *ARn+0 */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x6: /* *ARn-0B (bit-reversed) */
            /* Simplified: just subtract */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0x7: /* *ARn+0B (bit-reversed) */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0x8: /* *ARn-% (circular) */
            if (s->bk == 0) s->ar[cur_arp]--;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]--;
                if (s->ar[cur_arp] < base) s->ar[cur_arp] = base + s->bk - 1;
            }
            break;
        case 0x9: /* *ARn+% (circular) */
            if (s->bk == 0) s->ar[cur_arp]++;
            else {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                s->ar[cur_arp]++;
                if (s->ar[cur_arp] >= base + s->bk) s->ar[cur_arp] = base;
            }
            break;
        case 0xA: /* *ARn-0% */
            s->ar[cur_arp] -= s->ar[0];
            break;
        case 0xB: /* *ARn+0% */
            s->ar[cur_arp] += s->ar[0];
            break;
        case 0xC: /* *(lk) — absolute address from next word */
            addr = prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xD: /* *+ARn(lk) — pre-add offset from next word */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xE: /* *ARn(lk) — post-add (addr=AR, then AR += lk) */
            addr = s->ar[cur_arp];
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        case 0xF: /* *+ARn(lk)% — circular with offset */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
        }

        /* Update ARP */
        s->st0 = (s->st0 & ~ST0_ARP_MASK) | (nar << ST0_ARP_SHIFT);

        return addr;
    } else {
        /* Direct addressing: DP:offset */
        *indirect = false;
        uint16_t offset = opcode & 0x7F;
        return (dp(s) << 7) | offset;
    }
}

/* ================================================================
 * Instruction execution
 * ================================================================ */

/* Execute one instruction. Returns number of words consumed (1 or 2). */
/* PC ring buffer for pre-IDLE trace */
static uint16_t pc_ring[256];
static int pc_ring_idx = 0;

static int c54x_exec_one(C54xState *s)
{
    uint16_t op = prog_fetch(s, s->pc);
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;
    s->lk_used = false;  /* reset before each instruction */

    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* Detect when DSP enters DARAM code zone (0x0080-0x27FF) from ROM */
    {
        static uint16_t prev_pc = 0;
        static int daram_log = 0;
        if (s->pc >= 0x0080 && s->pc < 0x2800 && prev_pc >= 0x7000 && daram_log < 3) {
            C54_LOG("ROM->DARAM jump: 0x%04x->0x%04x op=0x%04x insn=%u SP=0x%04x XPC=%d",
                    prev_pc, s->pc, op, s->insn_count, s->sp, s->xpc);
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
            daram_log++;
        }
        prev_pc = s->pc;
    }
    if (s->pc >= 0xFE00 && s->pc <= 0xFFFF && op == 0x0000) {
        static int nop_slide = 0;
        if (nop_slide == 0) {
            C54_LOG("NOP-SLIDE PC=0x%04x insn=%u SP=0x%04x PMST=0x%04x XPC=%d OVLY=%d",
                    s->pc, s->insn_count, s->sp, s->pmst, s->xpc, !!(s->pmst & PMST_OVLY));
            C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                    pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                    pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                    pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                    pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                    pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
        }
        nop_slide++;
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed + s->lk_used;  /* NOP */

        /* XC n, cond — Execute Conditionally (SPRU172C p.4-198)
         * Opcode: 1111 11N1 CCCCCCCC
         * 0xFDxx = XC 1, cond (N=0, execute next 1 instruction)
         * 0xFFxx = XC 2, cond (N=1, execute next 2 instructions)
         * If condition true: execute normally. If false: skip n instructions. */
        if (hi8 == 0xFD || hi8 == 0xFF) {
            int n_insns = (hi8 == 0xFF) ? 2 : 1;
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition code per SPRU172C condition table */
            /* Conditions can be combined (OR'd bits), but common single conditions: */
            if (cc == 0x00)      cond = true;                          /* UNC */
            else if (cc == 0x0C) cond = (s->st0 & ST0_C) != 0;       /* C */
            else if (cc == 0x08) cond = !(s->st0 & ST0_C);            /* NC */
            else if (cc == 0x30) cond = (s->st0 & ST0_TC) != 0;       /* TC */
            else if (cc == 0x20) cond = !(s->st0 & ST0_TC);           /* NTC */
            else if (cc == 0x45) cond = (sext40(s->a) == 0);          /* AEQ */
            else if (cc == 0x44) cond = (sext40(s->a) != 0);          /* ANEQ */
            else if (cc == 0x46) cond = (sext40(s->a) > 0);           /* AGT */
            else if (cc == 0x42) cond = (sext40(s->a) >= 0);          /* AGEQ */
            else if (cc == 0x43) cond = (sext40(s->a) < 0);           /* ALT */
            else if (cc == 0x47) cond = (sext40(s->a) <= 0);          /* ALEQ */
            else if (cc == 0x4D) cond = (sext40(s->b) == 0);          /* BEQ */
            else if (cc == 0x4C) cond = (sext40(s->b) != 0);          /* BNEQ */
            else if (cc == 0x4E) cond = (sext40(s->b) > 0);           /* BGT */
            else if (cc == 0x4A) cond = (sext40(s->b) >= 0);          /* BGEQ */
            else if (cc == 0x4B) cond = (sext40(s->b) < 0);           /* BLT */
            else if (cc == 0x4F) cond = (sext40(s->b) <= 0);          /* BLEQ */
            else if (cc == 0x70) cond = (s->st0 & ST0_OVA) != 0;     /* AOV */
            else if (cc == 0x60) cond = !(s->st0 & ST0_OVA);          /* ANOV */
            else if (cc == 0x78) cond = (s->st0 & ST0_OVB) != 0;     /* BOV */
            else if (cc == 0x68) cond = !(s->st0 & ST0_OVB);          /* BNOV */
            else {
                /* Combined conditions: OR the individual condition bits */
                cond = false;
                if (cc & 0x0C) cond |= ((cc & 0x04) ? (s->st0 & ST0_C) != 0 : !(s->st0 & ST0_C));
                if (cc & 0x30) cond |= ((cc & 0x10) ? (s->st0 & ST0_TC) != 0 : !(s->st0 & ST0_TC));
                if (cc & 0x40) {
                    int64_t acc = (cc & 0x08) ? s->b : s->a;
                    int c3 = cc & 0x07;
                    switch (c3) {
                    case 0x5: cond |= (sext40(acc) == 0); break;
                    case 0x4: cond |= (sext40(acc) != 0); break;
                    case 0x6: cond |= (sext40(acc) > 0); break;
                    case 0x2: cond |= (sext40(acc) >= 0); break;
                    case 0x3: cond |= (sext40(acc) < 0); break;
                    case 0x7: cond |= (sext40(acc) <= 0); break;
                    default: cond = true; break;
                    }
                }
                if (cc & 0x70 && !(cc & 0x40)) {
                    if (cc & 0x08) cond |= (s->st0 & ST0_OVB) != 0;
                    else           cond |= (s->st0 & ST0_OVA) != 0;
                }
            }
            if (!cond) {
                /* Skip n instructions — count consumed words for skipped insns */
                /* Each skipped insn is 1 word (simplified — multi-word insns rare after XC) */
                return 1 + n_insns;
            }
            return consumed + s->lk_used;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed + s->lk_used; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed + s->lk_used; }
        /* F4EB = RETE (return from interrupt, alternate encoding per tic54x-opc.c) */
        if (op == 0xF4EB) {
            if (s->pmst & PMST_APTS) {
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
            }
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->st1 &= ~ST1_INTM;
            s->pc = ra; return 0;
        }
        if (op == 0xF4E4) {
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        s->pc, !!(s->st1 & ST1_INTM), s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            /* TDMA slot table (0x8000-0x8020): skip IDLE, continue next slot.
             * All other IDLEs: halt. Wake behavior decided by calypso_trx. */
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            static int idle_total = 0;
            idle_total++;
            if (idle_total <= 10) {
                C54_LOG("IDLE#%d @0x%04x SP=0x%04x stack=[0x%04x] insn=%u",
                        idle_total, s->pc, s->sp, s->data[s->sp], s->insn_count);
                /* Dump last 10 PCs before this IDLE */
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-10)&15], pc_ring[(pc_ring_idx-9)&15],
                        pc_ring[(pc_ring_idx-8)&15], pc_ring[(pc_ring_idx-7)&15],
                        pc_ring[(pc_ring_idx-6)&15], pc_ring[(pc_ring_idx-5)&15],
                        pc_ring[(pc_ring_idx-4)&15], pc_ring[(pc_ring_idx-3)&15],
                        pc_ring[(pc_ring_idx-2)&15], pc_ring[(pc_ring_idx-1)&15]);
            }
            s->idle = true;
            return 0;  /* PC stays on IDLE; wake code advances PC */
        } /* IDLE */
        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call and special instructions.
             * Some F4xx instructions are 1-word (FRET, FRETE, RETE, TRAP, NOP, etc.)
             * Must check specific opcodes BEFORE the 2-word switch. */

            /* Note: 0xF4E4 = IDLE (handled above, not FRET).
             * Real FRET = 0xF072 (algebraic), handled in F0xx section. */
            /* NOP — F495 per SPRU172C p.4-121 */
            if (op == 0xF495) {
                return 1; /* 1-word NOP */
            }
            /* TRAP K — F4C0-F4DF per SPRU172C p.4-195:
             * SP-1, PC+1 → TOS, vector(IPTR*128 + K*4) → PC */
            if ((op & 0xFFE0) == 0xF4C0) {
                int k = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + k * 4;
                C54_LOG("TRAP #%d → PC=0x%04x (from PC=0x%04x)", k, s->pc,
                        (uint16_t)(s->pc - (iptr * 0x80 + k * 4) + 1 - 1));
                return 0;
            }

            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            switch (sub) {
            case 0x0: /* B pmad */
            case 0xE: /* BD pmad (delayed — simplified as immediate) */
                s->pc = op2;
                return 0;
            case 0x2: /* BACC src — branch to accumulator */
                s->pc = (uint16_t)(s->a & 0xFFFF);
                return 0;
            case 0x3: /* BACCD src */
                s->pc = (uint16_t)(s->a & 0xFFFF);
                return 0;
            case 0x6: /* CALL pmad — near call, push PC only */
            case 0x8: /* CALLD pmad */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            case 0x7: /* CALA src — near call to accumulator, push PC only */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                s->pc = (uint16_t)(s->a & 0xFFFF);
                return 0;
            case 0x9: /* F49x: BD pmad (delayed branch) */
                s->pc = op2;
                return 0;
            case 0xD: /* F4Dx: FB extpmad — far branch (2 words) per SPRU172C */
            {
                /* op2 = 7-bit XPC:16-bit pmad packed, or just pmad with XPC in low bits */
                /* Format: word2 has bits 22-16 = XPC, stored as: XPC in high bits?
                 * Actually FB extpmad is 2 words: word2 = pmad(15:0), XPC from low nibble of opcode
                 * Per doc: pmad(22-16) = 7-bit constant in word1, pmad(15-0) = word2 */
                s->pc = op2;  /* pmad(15:0) */
                /* XPC not changed by BD — only FB changes XPC */
                return 0;
            }
            case 0xF: /* F4Fx: FCALL extpmad — far call (2 words) per SPRU172C p.4-57
                       * ALWAYS 2 pushes: push PC+2, then push XPC */
            {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->sp--;
                data_write(s, s->sp, s->xpc);
                s->pc = op2;
                return 0;
            }
            case 0xA: /* F4Ax: BANZ with indirect */
            case 0xB: /* F4Bx */
            case 0xC: /* F4Cx */
                s->pc = op2;
                return 0;
            default:
                goto unimpl;
            }
        }
        if (hi8 == 0xF0 || hi8 == 0xF1) {
            /* FIRS -- catch before other F1xx handlers */
            if (hi8 == 0xF1) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t xval = data_read(s, s->ar[xar]);
                uint16_t yval = data_read(s, s->ar[yar]);
                uint8_t xmod = (op >> 4) & 0xF;
                if ((xmod & 0x1) == 0) s->ar[xar]++; else s->ar[xar]--;
                if ((op & 0x08) == 0) s->ar[yar]++; else s->ar[yar]--;
                int16_t coeff = (int16_t)prog_read(s, op2);
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t product = (int64_t)ah * (int64_t)coeff;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->b = sext40(s->b + product);
                int32_t sum = (int32_t)(int16_t)xval + (int32_t)(int16_t)yval;
                s->a = sext40((int64_t)sum << 16);
                return consumed + s->lk_used;
            }
            /* F073: RET — near return, pop PC only (matches CALL) */
            if (op == 0xF073) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra; return 0;
            }
            /* F074: RETE — return from interrupt.
             * APTS=1: pop XPC then PC (matches IRQ entry push order).
             * APTS=0: pop PC only. Always clear INTM. */
            if (op == 0xF074) {
                if (s->pmst & PMST_APTS) {
                    s->xpc = data_read(s, s->sp); s->sp++;
                    if (s->xpc > 3) s->xpc &= 3;
                }
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                s->pc = ra; return 0;
            }







            /* F072: FRET — far return. ALWAYS 2 pops per SPRU172C p.4-61:
             * (TOS) → XPC, SP+1, (TOS) → PC, SP+1 */
            if (op == 0xF072) {
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra; return 0;
            }
            /* F07x: other control flow — all 1 pop with APTS=0 */
            if ((op & 0xFFF0) == 0xF070) {
                /* APTS=1 variants would pop XPC too, but APTS=0 → 1 pop */
                if ((s->pmst & PMST_APTS) && (op == 0xF077 || op == 0xF076)) {
                    s->xpc = data_read(s, s->sp); s->sp++;
                    if (s->xpc > 3) s->xpc &= 3;
                }
                uint16_t ra = data_read(s, s->sp); s->sp++;
                if (op == 0xF076) s->st1 &= ~ST1_INTM; /* RETED */
                s->pc = ra; return 0;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed + s->lk_used;
            }
            /* F0xx: READA Smem — Read program memory addressed by accumulator A.
             * Per SPRU172C p.4-136: A → PAR, Pmem[PAR] → Smem.
             * When repeated: PAR auto-increments. A is NOT modified. */
            if ((op & 0xFF00) == 0xF000) {
                addr = resolve_smem(s, op, &ind);
                if (!s->par_set) {
                    s->par = (uint16_t)(s->a & 0xFFFF);
                    s->par_set = true;
                }
                uint16_t val = prog_read(s, s->par);
                data_write(s, addr, val);
                s->par++;
                return consumed + s->lk_used;
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 2));
            s->pc = op2;
            return 0;
        }
        if (op == 0xF273) {
            /* RETD — delayed return (1 word) */
            uint16_t ra = data_read(s, s->sp); s->sp++;
            s->pc = ra;
            return 0;
        }
        /* LMS Xmem, Ymem — Least Mean Square step (1-word dual-operand)
         * Encoding: 1111 001D XXXX YYYY
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem */
        if (hi8 == 0xF2 || hi8 == 0xF3) {
            int xar_l = (op >> 4) & 0x07;
            int yar_l = op & 0x07;
            uint16_t xval_l = data_read(s, s->ar[xar_l]);
            uint16_t yval_l = data_read(s, s->ar[yar_l]);
            /* MAC: dst += T * Xmem */
            int64_t prod_l = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_l;
            if (s->st1 & ST1_FRCT) prod_l <<= 1;
            int dst_l = hi8 & 1;
            if (dst_l) s->b = sext40(s->b + prod_l);
            else       s->a = sext40(s->a + prod_l);
            /* LMS coefficient update: Ymem += rnd(AH * T) */
            int16_t ah_l = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t update = (int32_t)ah_l * (int32_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) update <<= 1;
            update += 0x8000; /* round */
            int16_t new_ym = (int16_t)yval_l + (int16_t)(update >> 16);
            data_write(s, s->ar[yar_l], (uint16_t)new_ym);
            /* T = Xmem */
            s->t = xval_l;
            /* Post-modify */
            if ((op >> 7) & 1) s->ar[xar_l]--; else s->ar[xar_l]++;
            if ((op & 0x08) == 0) s->ar[yar_l]++; else s->ar[yar_l]--;
            return consumed + s->lk_used;
        }
        if (op == 0xF495) {
            /* NOP */
            return consumed + s->lk_used;
        }
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed + s->lk_used; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed + s->lk_used; }
        if (op == 0xF4E4) {
            /* IDLE */
            s->idle = true;
            return consumed + s->lk_used;  /* Advance PC past IDLE */
        }
        /* FXXX short immediates and misc */
        if (hi8 == 0xF0 || hi8 == 0xF1) {
        /* FIRS Xmem, Ymem, pmad -- Symmetric FIR filter step (2 words)
         * Encoding: 1111 0001 XXXX YYYY + pmad
         * Execution: B = rnd(B + A(32-16) * prog[pmad])
         *            A = (Xmem + Ymem) << 16
         * Per SPRU172C p.4-59 */
        if (hi8 == 0xF1 && (op & 0xFF00) == 0xF100) {
            uint8_t xmod = (op >> 4) & 0xF;
            uint8_t ymod = op & 0xF;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Resolve Xmem and Ymem via dual indirect addressing */
            /* Xmem uses ARP, Ymem uses AR(ymod & 7) */
            int xar = (op >> 4) & 0x07;
            int yar = ymod & 0x07;
            uint16_t xval = data_read(s, s->ar[xar]);
            uint16_t yval = data_read(s, s->ar[yar]);
            /* Post-modify: Xmem uses xmod, Ymem uses ymod */
            if ((xmod & 0x1) == 0) s->ar[xar]++; else s->ar[xar]--;
            if ((ymod & 0x8) == 0) s->ar[yar]++; else s->ar[yar]--;
            /* Read coefficient from program memory */
            int16_t coeff = (int16_t)prog_read(s, op2);
            /* B += A(32-16) * coeff */
            int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t product = (int64_t)ah * (int64_t)coeff;
            if (s->st1 & ST1_FRCT) product <<= 1;
            s->b = sext40(s->b + product);
            /* A = (Xmem + Ymem) << 16 */
            int32_t sum = (int32_t)(int16_t)xval + (int32_t)(int16_t)yval;
            s->a = sext40((int64_t)sum << 16);
            return consumed + s->lk_used;
        }
            /* F0xx/F1xx: various - SSBX, RSBX, etc. */
            if ((op & 0xFFF0) == 0xF070) {
                /* F07x: misc control */
                uint8_t sub = op & 0x0F;
                if (sub == 0x3) {
                    /* RET */
                    uint16_t ret_addr = data_read(s, s->sp);
                    s->sp++;
                    s->pc = ret_addr;
                    return 0;
                }
                if (sub == 0x4) {
                    /* RETE (return from interrupt) */
                    if (s->pmst & PMST_APTS) {
                        s->xpc = data_read(s, s->sp); s->sp++;
                        if (s->xpc > 3) s->xpc &= 3;
                    }
                    uint16_t ret_addr = data_read(s, s->sp);
                    s->sp++;
                    s->st1 &= ~ST1_INTM;
                    s->pc = ret_addr;
                    return 0;
                }








                goto unimpl;
            }
            /* F0Bx / F1Bx: RSBX/SSBX */
            if ((op & 0xFE00) == 0xF000 && (op & 0x00F0) == 0x00B0) {
                /* RSBX/SSBX bit in ST0/ST1 */
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;  /* 0=ST0, 1=ST1 */
                if (st == 0) {
                    if (set) s->st0 |= (1 << bit);
                    else     s->st0 &= ~(1 << bit);
                } else {
                    if (set) s->st1 |= (1 << bit);
                    else     s->st1 &= ~(1 << bit);
                }
                return consumed + s->lk_used;
            }
        }
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            if (sub == 0x2) {
                /* F82x: RPTB pmad */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            if (sub == 0x3) {
                /* F83x: RPT #k (short) — advance PC past RPT+imm */
                op2 = prog_fetch(s, s->pc + 1);
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            /* F88x/F89x: B pmad — unconditional branch (2 words)
             * F8Ax/F8Bx: BD pmad — delayed branch (2 words)
             * Per SPRU172C: 1111 1000 1SSS DDDD pmad */
            if (sub >= 0x8 && sub <= 0xB) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F86x/F87x: BANZ *ARn, pmad — branch if ARn != 0 (2 words) */
            if (sub == 0x6 || sub == 0x7) {
                op2 = prog_fetch(s, s->pc + 1);
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;  /* skip 2 words, fall through */
            }
            /* F84x/F85x: BANZ with condition / CALL variants */
            if (sub == 0x4 || sub == 0x5) {
                op2 = prog_fetch(s, s->pc + 1);
                /* BANZ ARn, pmad */
                int ar_idx = op & 0x07;
                if (s->ar[ar_idx] != 0) {
                    s->ar[ar_idx]--;
                    s->pc = op2;
                    return 0;
                }
                return 2;
            }
            /* F8Cx-F8Fx: CALL/CALLD pmad (2 words) */
            if (sub >= 0xC) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            /* F80x-F81x: BANZ pmad, Smem (2 words)
             * Per SPRU172C + tic54x-opc.c: entire F8xx range is BANZ.
             * Smem determines AR post-modify; test AR(ARP) != 0 → branch. */
            if (sub <= 0x1) {
                uint16_t ar_idx = arp(s);
                uint16_t old_ar = s->ar[ar_idx];
                addr = resolve_smem(s, op, &ind);
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->ar[ar_idx]--; /* BANZ always decrements AR(ARP) */
                if (old_ar != 0) {
                    s->pc = op2;
                    return 0;
                }
                return consumed + s->lk_used;
            }
            /* Fallback: RPT Smem (F8xx sub not handled above) */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_active = true;
            s->pc += consumed;
            return 0;
        }
        /* F2xx: NORM *ARn, dst — Normalize accumulator
         * Per SPRU172C p.4-120: 1111 0q10 0111 D000
         * Shifts accumulator left by 1 if MSBs match, decrements ARn.
         * Simplified: treat as NOP for DSP signal processing. */
        if (hi8 == 0xF2) {
            /* F2xx: per tic54x-opc.c (GDB binutils):
             * F272 = RPTBD pmad (repeat block delayed, 2 words)
             * F274 = CALLD pmad (call delayed, 2 words)
             * F273 = RETD (return delayed, 1 word) */
            if (op == 0xF272) {
                /* RPTBD pmad — delayed block repeat (2 words).
                 * REA = pmad, RSA = PC+2, uses BRC for count. */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            if (op == 0xF274) {
                /* CALLD pmad — delayed call (2 words).
                 * Push PC+2, branch to pmad. */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            if (op == 0xF273) {
                /* RETD — delayed return (1 word). Pop PC. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra;
                return 0;
            }
            /* Other F27x: NORM *ARn, dst */
            if ((op & 0xFFF0) == 0xF270) {
                int dst = (op >> 3) & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                int bit39 = (*acc >> 39) & 1;
                int bit38 = (*acc >> 38) & 1;
                if (bit39 == bit38) {
                    *acc = sext40(*acc << 1);
                    s->ar[arp(s)]--;
                    s->st0 &= ~ST0_TC;
                } else {
                    s->st0 |= ST0_TC;
                }
                return consumed + s->lk_used;
            }
            goto unimpl;
        }
        /* F3xx: various */
        if (hi8 == 0xF3) {
            uint8_t sub3 = (op >> 5) & 0x07;
            if (sub3 == 0) {
                /* F300-F31F: INTR k — software interrupt (branch to vector k) */
                int vec = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                return 0;
            }
            /* F320+: LD #k9, DP */
            uint16_t k9 = op & 0x01FF;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed + s->lk_used;
            }
            if (sub >= 0x8) {
                /* F68x-F6Fx: MVDD Xmem, Ymem — dual data-memory operand move
                 * Encoding: 1111 0110 XXXX YYYY
                 *   bit 7   = Xmod (0=inc, 1=dec)
                 *   bits 6:4 = Xar  (source AR register)
                 *   bit 3   = Ymod (0=inc, 1=dec)
                 *   bits 2:0 = Yar  (dest AR register) */
                int xar = (op >> 4) & 0x07;
                int yar = op & 0x07;
                uint16_t val = data_read(s, s->ar[xar]);
                data_write(s, s->ar[yar], val);
                if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
                if ((op >> 3) & 1) s->ar[yar]--; else s->ar[yar]++;
                return consumed + s->lk_used;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed + s->lk_used;
        }
        /* F8xx: BANZ pmad, Smem — Branch if AR not zero (2-word)
         * Per SPRU172C: modify AR per Smem addressing; if AR[ARP] != 0, branch to pmad
         * Encoding: 1111 1000 IAAA AAAA + pmad */
        if (hi8 == 0xF8) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t cur_arp_val = s->ar[arp(s)];
            {
                static int banz_log = 0;
                if (banz_log < 5)
                    C54_LOG("BANZ PC=0x%04x ARP=%d AR[ARP]=0x%04x target=0x%04x %s",
                            s->pc, arp(s), cur_arp_val, op2,
                            cur_arp_val != 0 ? "TAKEN" : "NOT TAKEN");
                banz_log++;
            }
            if (cur_arp_val != 0) {
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* F5xx: RPT #k (short immediate, k in low byte) */
        if (hi8 == 0xF5) {
            /* RPT #k (short immediate) — advance PC past RPT now,
             * the NEXT instruction at PC+1 will be repeated. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x0) {
                /* F600/F601: ABS src[,dst] — absolute value of accumulator */
                int src = op & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val < 0) val = -val;
                *acc = sext40(val);
                /* Set C if input was -2^39 (saturate), clear OVx if OVM */
                return consumed + s->lk_used;
            }
            uint16_t k = op & 0xFF;
            switch (sub) {
            case 0x0: /* F70x: LD #k8, ASM */
                s->st1 = (s->st1 & ~ST1_ASM_MASK) | (k & ST1_ASM_MASK);
                break;
            case 0x1: /* F71x: LD #k8, AR0 */
                s->ar[0] = k; break;
            case 0x2: /* F72x: LD #k8, AR1 */
                s->ar[1] = k; break;
            case 0x3: s->ar[2] = k; break;
            case 0x4: s->ar[3] = k; break;
            case 0x5: s->ar[4] = k; break;
            case 0x6: s->ar[5] = k; break;
            case 0x7: s->ar[6] = k; break;
            case 0x8: /* F78x: LD #k8, T */
                s->t = (s->st1 & ST1_SXM) ? (uint16_t)(int8_t)k : k; break;
            case 0x9: /* F79x: LD #k8, DP */
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (k & ST0_DP_MASK); break;
            case 0xA: /* F7Ax: LD #k8, ARP */
                s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((k & 7) << ST0_ARP_SHIFT); break;
            case 0xB: s->ar[7] = k; break;
            case 0xC: s->bk = k; break;
            case 0xD: s->sp = k; break;
            case 0xE: /* F7Ex: LD #k8, BRC */
                s->brc = k; break;
            case 0xF: /* F7Fx: LD #k8, ... */
                break;
            }
            return consumed + s->lk_used;
        }
        /* F9xx: RPT #lk (16-bit immediate) */
        if (hi8 == 0xF9) {
            /* F9xx: BC pmad, cond — conditional branch (2 words)
             * Per SPRU172C + tic54x-opc.c: F900 mask FF00 = BC.
             * Encoding: 1111 1001 CCCC QQQQ pmad */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond_code = (op >> 4) & 0xF;
            uint8_t qual = op & 0xF;
            bool take = false;
            int64_t acc = (qual & 0x8) ? s->b : s->a;
            switch (cond_code) {
            case 0x0: take = true; break;
            case 0x1: take = (acc < 0); break;
            case 0x2: take = (acc <= 0); break;
            case 0x3: take = (acc != 0); break;
            case 0x4: take = (acc == 0); break;
            case 0x5: take = (acc >= 0); break;
            case 0x6: take = (acc > 0); break;
            case 0x8: take = !!(s->st0 & ST0_TC); break;
            case 0x9: take = !(s->st0 & ST0_TC); break;
            case 0xA: take = !!(s->st0 & ST0_C); break;
            case 0xB: take = !(s->st0 & ST0_C); break;
            default: take = true; break;
            }
            if (take) {
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx: RPT Smem or conditional ops */
        if (hi8 == 0xFA) {
            /* FA3x: BC with delay, FA4x: conditional etc. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Simplified: treat as delayed branch */
            s->pc = op2;
            return 0;
        }
        /* FBxx: LD #k, 16, A/B (short immediate shift 16) */
        if (hi8 == 0xFB) {
            int8_t k = (int8_t)(op & 0xFF);
            int64_t v = (int64_t)k << 16;
            s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        if (hi8 == 0xFC) {
            int8_t k = (int8_t)(op & 0xFF);
            int64_t v = (int64_t)k << 16;
            s->b = sext40(v);
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: LD #k, B (no shift) */
        if (hi8 == 0xFE) {
            int8_t k = (int8_t)(op & 0xFF);
            s->b = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FFxx: ADD/SUB short immediate */
        if (hi8 == 0xFF) {
            int8_t k = (int8_t)(op & 0x7F);
            int dst = (op >> 7) & 1;
            /* Typically ADD #k, A or SUB */
            if (dst) s->b = sext40(s->b + ((int64_t)k << 16));
            else     s->a = sext40(s->a + ((int64_t)k << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        /* CMPS src, Smem — Compare, Select, and Store (Viterbi)
         * Encoding: 1110 00SD IAAAAAAA (1 word)
         * Per SPRU172C p.4-35: if |A(32-16)| >= |Smem| then TC=1,
         * TRN = (TRN<<1)|1, dst=A; else TC=0, TRN=(TRN<<1), dst=Smem<<16 */
        if ((op & 0xFC00) == 0xE000) {
            int src_s = (op >> 9) & 1;
            int dst_d = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t acc = src_s ? s->b : s->a;
            int32_t ah = (int32_t)((acc >> 16) & 0xFFFF);
            if (ah < 0) ah = -ah;
            int32_t sv = (int16_t)val;
            if (sv < 0) sv = -sv;
            s->trn <<= 1;
            if (ah >= sv) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                int64_t nv = (int64_t)(int16_t)val << 16;
                if (dst_d) s->b = sext40(nv); else s->a = sext40(nv);
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFE00) == 0xEA00) {
            /* LD #k9, DP -- Load Data Page pointer with 9-bit immediate.
             * Encoding: 1110 101k kkkk kkkk (1 word)
             * Per SPRU172C p.4-70 and GNU binutils tic54x-opc.c */
            uint16_t k9 = op & 0x01FF;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* EC: BC pmad, cond (conditional branch, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Simplified: evaluate common conditions */
            uint8_t cond = op & 0xFF;
            bool take = false;
            /* Condition evaluation (simplified) */
            if (cond == 0x03) take = (s->a == 0);        /* AEQ */
            else if (cond == 0x0B) take = (s->b == 0);   /* BEQ */
            else if (cond == 0x02) take = (s->a != 0);   /* ANEQ */
            else if (cond == 0x0A) take = (s->b != 0);   /* BNEQ */
            else if (cond == 0x00) take = true;            /* UNC (no cond bits set) */
            else if (cond == 0x08) take = (s->b < 0);    /* BLT */
            else if (cond == 0x04) take = (s->a > 0);    /* AGT */
            else if (cond == 0x0C) take = (s->b > 0);    /* BGT */
            else if (cond == 0x01) take = (s->a >= 0);   /* AGEQ */
            else if (cond == 0x09) take = (s->b >= 0);   /* BGEQ */
            else if (cond == 0x05) take = (s->a <= 0);   /* ALEQ */
            else if (cond == 0x0D) take = (s->b <= 0);   /* BLEQ */
            else if (cond == 0x40) take = (s->st0 & ST0_TC);   /* TC */
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);  /* NTC */
            else if (cond == 0x20) take = (s->st0 & ST0_C);    /* C */
            else if (cond == 0x21) take = !(s->st0 & ST0_C);   /* NC */
            else take = true;  /* unknown cond: take (safer than skip) */

            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVMM mmr, mmr */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src >= 0 && src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst >= 0 && dst <= 7) s->ar[dst] = val;
            else if (dst == 8) s->sp = val;
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE7) {
            /* E7xx: ST #k, Smem or LD #k,16, dst (short immediate) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE9) {
            /* E9xx: CC pmad, cond (conditional call, 2 words)
             * Condition codes per tic54x-dis.c cc2[] table (SPRU172C Table 3-3):
             * 0x00=UNC, 0x02=AGEQ, 0x03=ALT, 0x04=ANEQ, 0x05=AEQ,
             * 0x06=AGT, 0x07=ALEQ, 0x0A=BGEQ, 0x0B=BLT, 0x0C=BNEQ,
             * 0x0D=BEQ, 0x0E=BGT, 0x0F=BLEQ,
             * 0x20=NC, 0x21=C, 0x30=NTC, 0x31=TC,
             * 0x40=NBIO, 0x60=ANOV, 0x61=AOV, 0x68=BNOV, 0x69=BOV */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            int64_t sa = sext40(s->a), sb = sext40(s->b);
            bool take = false;
            switch (cond) {
            case 0x00: take = true; break;          /* UNC */
            case 0x02: take = (sa >= 0); break;     /* AGEQ */
            case 0x03: take = (sa < 0); break;      /* ALT */
            case 0x04: take = (sa != 0); break;     /* ANEQ */
            case 0x05: take = (sa == 0); break;     /* AEQ */
            case 0x06: take = (sa > 0); break;      /* AGT */
            case 0x07: take = (sa <= 0); break;     /* ALEQ */
            case 0x0A: take = (sb >= 0); break;     /* BGEQ */
            case 0x0B: take = (sb < 0); break;      /* BLT */
            case 0x0C: take = (sb != 0); break;     /* BNEQ */
            case 0x0D: take = (sb == 0); break;     /* BEQ */
            case 0x0E: take = (sb > 0); break;      /* BGT */
            case 0x0F: take = (sb <= 0); break;     /* BLEQ */
            case 0x20: take = !(s->st0 & ST0_C); break;  /* NC */
            case 0x21: take = (s->st0 & ST0_C) != 0; break; /* C */
            case 0x30: take = !(s->st0 & ST0_TC); break; /* NTC */
            case 0x31: take = (s->st0 & ST0_TC) != 0; break; /* TC */
            case 0x60: take = !(s->st0 & ST0_OVA); break; /* ANOV */
            case 0x61: take = (s->st0 & ST0_OVA) != 0; break; /* AOV */
            case 0x68: take = !(s->st0 & ST0_OVB); break; /* BNOV */
            case 0x69: take = (s->st0 & ST0_OVB) != 0; break; /* BOV */
            default: take = false; break;  /* unknown cond = don't take */
            }
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE1) {
            /* E1xx: single-word acc ops — NEG, ABS, CMPL, SAT, EXP, etc. */
            uint8_t sub = op & 0xFF;
            switch (sub) {
            case 0xE0: s->a = ~s->a; s->a = sext40(s->a); break;  /* CMPL A */
            case 0xE1: s->b = ~s->b; s->b = sext40(s->b); break;  /* CMPL B */
            case 0xE2: s->a = -s->a; s->a = sext40(s->a); break;  /* NEG A */
            case 0xE3: s->b = -s->b; s->b = sext40(s->b); break;  /* NEG B */
            case 0xE4: /* SAT A */ if (s->st0 & ST0_OVA) s->a = (s->a < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE5: /* SAT B */ if (s->st0 & ST0_OVB) s->b = (s->b < 0) ? (int64_t)0xFF80000000LL : 0x7FFFFFFFLL; break;
            case 0xE8: /* ABS A */ s->a = (s->a < 0) ? -s->a : s->a; s->a = sext40(s->a); break;
            case 0xE9: /* ABS B */ s->b = (s->b < 0) ? -s->b : s->b; s->b = sext40(s->b); break;
            case 0xEA: /* ROR A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & 1) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a >> 1) | ((int64_t)c << 39); s->a = sext40(s->a); } break;
            case 0xEB: /* ROL A */ { uint16_t c = s->st0 & ST0_C ? 1 : 0; if (s->a & ((int64_t)1<<39)) s->st0 |= ST0_C; else s->st0 &= ~ST0_C; s->a = (s->a << 1) | c; s->a = sext40(s->a); } break;
            default:
                /* EXP A/B etc — return 0 for now */
                break;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEF) {
            /* EFxx: RPTZ dst, #lk — Zero accumulator and repeat (2 words)
             * Per SPRU172C: dst = 0; RPT #lk
             * Encoding: 1110 1111 xxxx xxxx + lk_word */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int rptz_dst = (op >> 0) & 1;
            if (rptz_dst) s->b = 0; else s->a = 0;
            s->rpt_count = op2;
            s->rpt_active = true;
            s->pc += 2;
            return 0;
        }
        if (hi8 == 0xEB) {
            /* EBxx: RPTB[D] pmad — Block repeat (2 words)
             * Per SPRU172C: REA = pmad, RSA = PC+2, BRAF=1 */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE6) {
            /* E6xx: SFTA/SFTL acc, #shift (single-word immediate shift) */
            int shift = op & 0x1F;
            if (shift & 0x10) shift |= ~0x1F;  /* sign extend 5-bit */
            int dst = (op >> 5) & 1;
            int logical = (op >> 6) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            if (logical) {
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else {
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEE) {
            /* EExx: BCD pmad, cond (conditional delayed branch, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x00) take = true;  /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else if (cond == 0x20) take = (s->st0 & ST0_C) != 0;
            else if (cond == 0x21) take = !(s->st0 & ST0_C);
            else if ((cond & 0x3A) == 0x3A) take = true; /* unconditional-ish */
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx: BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = true;            /* UNC */
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else take = true;
            if (take) { s->pc = op2; return 0; }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8) {
            /* E8xx: CMPR cond, ARn */
            int cmp_cond = (op >> 4) & 3;
            int n = arp(s);
            bool result = false;
            switch (cmp_cond) {
            case 0: result = (s->ar[n] == s->ar[0]); break;
            case 1: result = (s->ar[n] < s->ar[0]); break;
            case 2: result = (s->ar[n] > s->ar[0]); break;
            case 3: result = (s->ar[n] != s->ar[0]); break;
            }
            if (result) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* 76xx: LDM MMR, dst */
        if (hi8 == 0x76) {
            uint8_t mmr = op & 0x7F;
            uint16_t val = data_read(s, mmr);
            s->a = (int64_t)(int16_t)val << 16;
            return consumed + s->lk_used;
        }
        /* 77xx: STM #lk, MMR (2 words) */
        if (hi8 == 0x77) {
            uint8_t mmr = op & 0x7F;
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, mmr, op2);
            return consumed + s->lk_used;
        }
        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Fxx: STH src, Smem
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx: LD Smem, dst */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* 68xx: LD Smem, T */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1:
        /* 1xxx: SUB variants */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            v <<= 16;
            if (dst) s->b = sext40(s->b - v);
            else     s->a = sext40(s->a - v);
        }
        return consumed + s->lk_used;

    case 0x0:
        /* 0xxx: ADD variants */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            v <<= 16;
            if (dst) s->b = sext40(s->b + v);
            else     s->a = sext40(s->a + v);
        }
        return consumed + s->lk_used;

    case 0x3:
        /* 3xxx: MAC / MAS */
        addr = resolve_smem(s, op, &ind);
        {
            int dst = (op >> 8) & 1;
            uint16_t val = data_read(s, addr);
            int64_t product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            if (dst) s->b = sext40(s->b + product);
            else     s->a = sext40(s->a + product);
        }
        return consumed + s->lk_used;

    case 0x2:
        /* 2xxx: MPY, SQUR, MAS, MAC variants */
        {
            int sub = (op >> 8) & 0xF;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t product;
            int dst;
            switch (sub) {
            case 0x0: case 0x1: /* MPY Smem, A/B */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x4: case 0x5: /* SQUR Smem, A/B */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed + s->lk_used;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed + s->lk_used;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed + s->lk_used;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed + s->lk_used;
            }
        }

    case 0x4:
        /* 4xxx: AND, OR, XOR */
        addr = resolve_smem(s, op, &ind);
        {
            int sub = (op >> 8) & 0xF;
            uint16_t val = data_read(s, addr);
            switch (sub & 0x3) {
            case 0: /* AND */
                if (sub & 4) s->b = (s->b & 0xFFFF0000) | (((uint16_t)s->b) & val);
                else         s->a = (s->a & 0xFFFF0000) | (((uint16_t)s->a) & val);
                break;
            case 1: /* OR */
                if (sub & 4) s->b |= val;
                else         s->a |= val;
                break;
            case 2: /* XOR */
                if (sub & 4) s->b ^= val;
                else         s->a ^= val;
                break;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 56xx: MVPD pmad, Smem — Move Program to Data (2 words) */
        if (hi8 == 0x56 || hi8 == 0x57) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t src = s->rpt_active ? s->mvpd_src : op2;
            uint16_t pval = prog_fetch(s, src);
            data_write(s, addr, pval);
            s->mvpd_src = src + 1;
            return consumed + s->lk_used;
        }
        /* 5xxx: shifts — SFTA, SFTL, various forms */
        {
            int dst = (op >> 8) & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            int sub = (op >> 9) & 0x7;
            if (sub <= 1) {
                /* 50xx/51xx: SFTA src, ASM shift */
                int shift = asm_shift(s);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 2 || sub == 3) {
                /* 54xx/55xx: SFTA src, #shift (immediate in Smem) */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            } else if (sub == 4 || sub == 5) {
                /* 58xx/59xx: SFTL src, ASM shift (logical) */
                int shift = asm_shift(s);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            } else if (sub == 6 || sub == 7) {
                /* 5Cxx/5Dxx/5Exx/5Fxx: SFTL with Smem or other */
                addr = resolve_smem(s, op, &ind);
                int shift = (int16_t)data_read(s, addr);
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
        }
        return consumed + s->lk_used;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */

        /* ---- Dual-operand MAC Xmem, Ymem, dst (1-word) ----
         * 0x90: MAC Xmem,Ymem,A   0x92: MAC Xmem,Ymem,B
         * 0x91: MACR Xmem,Ymem,A  0x93: MACR Xmem,Ymem,B
         * Same encoding as 0xA4 family: OOOO OOOD XXXX YYYY */
        if (hi8 == 0x90 || hi8 == 0x91 || hi8 == 0x92 || hi8 == 0x93) {
            int xar_m = (op >> 4) & 0x07;
            int yar_m = op & 0x07;
            uint16_t xval_m = data_read(s, s->ar[xar_m]);
            uint16_t yval_m = data_read(s, s->ar[yar_m]);
            if ((op >> 7) & 1) s->ar[xar_m]--; else s->ar[xar_m]++;
            if ((op & 0x08) == 0) s->ar[yar_m]++; else s->ar[yar_m]--;
            int64_t prod_m = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_m;
            if (s->st1 & ST1_FRCT) prod_m <<= 1;
            if (hi8 & 0x01) prod_m += 0x8000; /* round */
            int dst_m = (hi8 & 0x02) ? 1 : 0;
            if (dst_m) s->b = sext40(s->b + prod_m);
            else       s->a = sext40(s->a + prod_m);
            s->t = yval_m;
            return consumed + s->lk_used;
        }

        /* 94xx: MVDK Smem, dmad — Move data(Smem) to data(dmad) (2 words) */
        if (hi8 == 0x94) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 95xx: MVKD dmad, Smem — Move data(dmad) to data(Smem) (2 words) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 96xx: MVDP Smem, pmad — Move data to program (2 words) */
        if (hi8 == 0x96) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->prog[op2] = data_read(s, addr);
            return consumed + s->lk_used;
        }

        /* 0x98/0x99: STH src, Smem — store high accumulator (no shift)
         * 0x9A/0x9B: STL src, Smem — store low accumulator (no shift)
         * Per SPRU172C: 1001 100S = STH, 1001 101S = STL */
        if (hi8 == 0x98 || hi8 == 0x99) {
            addr = resolve_smem(s, op, &ind);
            int src_sth = hi8 & 1;
            int64_t acc = src_sth ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A || hi8 == 0x9B) {
            addr = resolve_smem(s, op, &ind);
            int src_stl = hi8 & 1;
            int64_t acc = src_stl ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }

        /* 0x9C-0x9F range: SACCD/SRCCD/STRCD — conditional stores */

        /* SACCD src, Xmem, cond — Conditional accumulator store
         * Encoding: 1001 11SD XXXX COND per SPRU172C p.4-152 */
        if ((op & 0xFC00) == 0x9C00) {
            int src_s = (op >> 9) & 1;
            int64_t acc = src_s ? s->b : s->a;
            int xar_s = (op >> 4) & 0x07;
            uint16_t xaddr = s->ar[xar_s];
            int cond = op & 0x0F;
            /* Evaluate condition */
            int take = 0;
            switch (cond) {
            case 0x0: take = (acc == 0); break;    /* EQ */
            case 0x1: take = (acc != 0); break;    /* NEQ */
            case 0x2: take = (acc > 0); break;     /* GT */
            case 0x3: take = (acc < 0); break;     /* LT */
            case 0x4: take = (acc >= 0); break;    /* GEQ */
            case 0x5: take = (acc == 0); break;    /* AEQ */
            case 0x6: take = (acc > 0); break;     /* AGT */
            case 0x7: take = (acc <= 0); break;    /* LEQ/ALEQ */
            default: take = 0; break;
            }
            int asm_val = asm_shift(s);
            if (take) {
                /* Store shifted accumulator high part */
                int64_t shifted = acc << (asm_val > 0 ? asm_val : 0);
                if (asm_val < 0) shifted = acc >> (-asm_val);
                uint16_t val = (uint16_t)((shifted >> 16) & 0xFFFF);
                data_write(s, xaddr, val);
            } else {
                /* Read and write back (no change) */
                uint16_t val = data_read(s, xaddr);
                data_write(s, xaddr, val);
            }
            /* Xmem post-modify */
            if ((op >> 7) & 1) s->ar[xar_s]--; else s->ar[xar_s]++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8A) {
            /* MVDK Smem, dmad */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9A) {
            /* MVKD dmad, Smem */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x88 || hi8 == 0x80) {
            /* MVDD Smem, Smem (data→data) — 2 address forms */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8C) {
            /* MVPD pmad, Smem (prog→data) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mvpd_val = prog_read(s, op2);
            data_write(s, addr, mvpd_val);
            { static int mvpd_log = 0; if (mvpd_log++ < 20)
                C54_LOG("MVPD: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x insn=%u",
                        op2, mvpd_val, addr, s->pc, s->insn_count); }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8E) {
            /* MVDP Smem, pmad (data→prog) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            prog_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x8F) {
            /* PORTR PA, Smem — read I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* PA=0xF430: BSP data register — return next burst sample */
            if (op2 == 0xF430 && s->bsp_pos < s->bsp_len) {
                data_write(s, addr, s->bsp_buf[s->bsp_pos++]);
            } else {
                data_write(s, addr, 0);
            }
            /* Log PORTR calls */
            {
                static int portr_log = 0;
                if (portr_log < 50) {
                    C54_LOG("PORTR PA=0x%04x → [0x%04x] val=0x%04x PC=0x%04x",
                            op2, addr, data_read(s, addr), s->pc);
                    portr_log++;
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* Log I/O port writes */
            {
                uint16_t wval = data_read(s, addr);
                static int portw_log = 0;
                if (portw_log < 30) {
                    C54_LOG("PORTW PA=0x%04x val=0x%04x PC=0x%04x", op2, wval, s->pc);
                    portw_log++;
                }
            }
            return consumed + s->lk_used;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 86xx: MVDM dmad, MMR */
        if (hi8 == 0x86) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 87xx: MVMD MMR, dmad */
        if (hi8 == 0x87) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, op2, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        /* 81xx: STL src, ASM, Smem (store with shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 82xx: STH src, ASM, Smem */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Bxx: MVDK with long address */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed + s->lk_used;
        }
        /* 83xx: WRITA Smem (write A to prog), 84xx: READA Smem */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            prog_write(s, (uint16_t)(s->a & 0xFFFF), data_read(s, addr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, prog_read(s, (uint16_t)(s->a & 0xFFFF)));
            return consumed + s->lk_used;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 95xx: ST #lk, Smem (another encoding) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        /* ST #lk, Smem (2-word) */
        if (hi8 == 0x96 || hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * MAC:  dst += T * Xmem; T = Ymem
         * MACR: dst += rnd(T * Xmem); T = Ymem
         * MAS:  dst -= T * Xmem; T = Ymem
         * MASR: dst -= rnd(T * Xmem); T = Ymem
         * Encoding: OOOO OOOD XXXX YYYY (1 word)
         *   Xmem: AR[ARP], post-mod by bit4 (0=inc,1=dec)
         *   Ymem: AR[bits2:0], post-mod by bit3 (0=inc,1=dec)
         *   D: 0=A, 1=B
         * hi8 mapping per SPRU172C:
         *   0xA4/0xA5: MAC[R] Xmem,Ymem,A   0xA6/0xA7: MAC[R] Xmem,Ymem,B
         *   0xB4/0xB5: MAS[R] Xmem,Ymem,A   0xB6/0xB7: MAS[R] Xmem,Ymem,B
         *   0xB0/0xB1: MAC[R] Xmem,Ymem,A (alt)  0xB2/0xB3 already handled
         */
        if (hi8 == 0xA4 || hi8 == 0xA5 || hi8 == 0xA6 || hi8 == 0xA7 ||
            hi8 == 0xB4 || hi8 == 0xB5 || hi8 == 0xB6 || hi8 == 0xB7 ||
            hi8 == 0xB0 || hi8 == 0xB1 || hi8 == 0xB2) {
            int xar_d = (op >> 4) & 0x07;
            int yar_d = op & 0x07;
            uint16_t xval_d = data_read(s, s->ar[xar_d]);
            uint16_t yval_d = data_read(s, s->ar[yar_d]);
            /* Post-modify */
            if ((op >> 7) & 1) s->ar[xar_d]--; else s->ar[xar_d]++;
            if ((op & 0x08) == 0) s->ar[yar_d]++; else s->ar[yar_d]--;
            /* Multiply T * Xmem */
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_d;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            /* Round if R bit set (odd hi8) */
            if (hi8 & 0x01) prod += 0x8000;
            /* Determine dest and operation */
            int is_sub = (hi8 >= 0xB4 && hi8 <= 0xB7);
            int dst_b;
            if (hi8 >= 0xA4 && hi8 <= 0xA7) dst_b = (hi8 >= 0xA6);
            else if (hi8 >= 0xB4 && hi8 <= 0xB7) dst_b = (hi8 >= 0xB6);
            else dst_b = (hi8 & 0x02) ? 1 : 0; /* 0xB0/B1→A, 0xB2/B3→B */
            if (dst_b) {
                if (is_sub) s->b = sext40(s->b - prod);
                else        s->b = sext40(s->b + prod);
            } else {
                if (is_sub) s->a = sext40(s->a - prod);
                else        s->a = sext40(s->a + prod);
            }
            /* T = Ymem */
            s->t = yval_d;
            return consumed + s->lk_used;
        }

        /* SQDST Xmem, Ymem — Squared Distance (1-word dual-operand)
         * Encoding: 1010 0001 XXXX YYYY
         * Per SPRU172C: B += (AH - Xmem)^2; A = Ymem << 16; T = Xmem */
        if (hi8 == 0xA1) {
            int xar_sq = (op >> 4) & 0x07;
            int yar_sq = op & 0x07;
            uint16_t xval_sq = data_read(s, s->ar[xar_sq]);
            uint16_t yval_sq = data_read(s, s->ar[yar_sq]);
            if ((op >> 7) & 1) s->ar[xar_sq]--; else s->ar[xar_sq]++;
            if ((op & 0x08) == 0) s->ar[yar_sq]++; else s->ar[yar_sq]--;
            int16_t ah_sq = (int16_t)((s->a >> 16) & 0xFFFF);
            int32_t diff = (int32_t)ah_sq - (int32_t)(int16_t)xval_sq;
            int64_t sq = (int64_t)diff * (int64_t)diff;
            if (s->st1 & ST1_FRCT) sq <<= 1;
            s->b = sext40(s->b + sq);
            s->a = sext40((int64_t)(int16_t)yval_sq << 16);
            s->t = xval_sq;
            return consumed + s->lk_used;
        }

        /* POLY Xmem, Ymem — Polynomial evaluation (1-word dual-operand)
         * Encoding: 1011 110D XXXX YYYY (0xBC=A, 0xBD=B)
         *           1011 111D XXXX YYYY (0xBE/0xBF variants — ABDST or POLY)
         * Per SPRU172C: B += AH * T (with round); A = Xmem << 16; T = Ymem */
        if (hi8 == 0xBC || hi8 == 0xBD || hi8 == 0xBE || hi8 == 0xBF) {
            int xar_p = (op >> 4) & 0x07;
            int yar_p = op & 0x07;
            uint16_t xval_p = data_read(s, s->ar[xar_p]);
            uint16_t yval_p = data_read(s, s->ar[yar_p]);
            if ((op >> 7) & 1) s->ar[xar_p]--; else s->ar[xar_p]++;
            if ((op & 0x08) == 0) s->ar[yar_p]++; else s->ar[yar_p]--;
            int16_t ah_p = (int16_t)((s->a >> 16) & 0xFFFF);
            int64_t prod_p = (int64_t)ah_p * (int64_t)(int16_t)s->t;
            if (s->st1 & ST1_FRCT) prod_p <<= 1;
            prod_p += 0x8000; /* round */
            s->b = sext40(s->b + prod_p);
            s->a = sext40((int64_t)(int16_t)xval_p << 16);
            s->t = yval_p;
            return consumed + s->lk_used;
        }

        /* B8-BB: MAS/MASR Xmem, Ymem (subtract variants) or POLY-like */
        if (hi8 == 0xB8 || hi8 == 0xB9 || hi8 == 0xBA || hi8 == 0xBB) {
            /* Check if it's actually LDMM (BA) or POPM (BD) — those are handled below */
            if (hi8 == 0xBA) goto ba_handler;
            int xar_b8 = (op >> 4) & 0x07;
            int yar_b8 = op & 0x07;
            uint16_t xval_b8 = data_read(s, s->ar[xar_b8]);
            uint16_t yval_b8 = data_read(s, s->ar[yar_b8]);
            if ((op >> 7) & 1) s->ar[xar_b8]--; else s->ar[xar_b8]++;
            if ((op & 0x08) == 0) s->ar[yar_b8]++; else s->ar[yar_b8]--;
            int64_t prod_b8 = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_b8;
            if (s->st1 & ST1_FRCT) prod_b8 <<= 1;
            if (hi8 & 0x01) prod_b8 += 0x8000;
            int dst_b8 = (hi8 & 0x02) ? 1 : 0;
            /* MAS: subtract */
            if (dst_b8) s->b = sext40(s->b - prod_b8);
            else        s->a = sext40(s->a - prod_b8);
            s->t = yval_b8;
            return consumed + s->lk_used;
        }
ba_handler:
        if (hi8 == 0xAA || hi8 == 0xAB) {
            /* STLM src, MMR — AA=A, AB=B */
            int src_acc = (hi8 == 0xAB) ? 1 : 0;
            uint16_t mmr = op & 0x1F;
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, mmr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xBD) {
            /* BDxx: POPM / delayed branch variants */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst */
            uint16_t mmr = op & 0x1F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA8 || hi8 == 0xA9) {
            /* A8xx/A9xx: AND #lk, src[, dst] (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            *acc = sext40(*acc & ((int64_t)op2 << 16));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA0) {
            /* A0xx: LD src, dst (accumulator to accumulator) or NEG/ABS/NOT */
            uint8_t sub = op & 0xFF;
            if (sub == 0x00) { s->a = s->b; }
            else if (sub == 0x01) { s->b = s->a; }
            else if (sub == 0x08) { s->a = -s->a; } /* NEG A */
            else if (sub == 0x09) { s->b = -s->b; } /* NEG B */
            else if (sub == 0x0A) { s->a = (s->a < 0) ? -s->a : s->a; } /* ABS A */
            else if (sub == 0x0B) { s->b = (s->b < 0) ? -s->b : s->b; } /* ABS B */
            return consumed + s->lk_used;
        }
        if (hi8 == 0xA5) {
            /* CMPS src, Smem — compare and select (Viterbi) */
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int src = (op >> 4) & 1;
            int64_t acc = src ? s->b : s->a;
            int64_t cmp = (int64_t)(int16_t)val << 16;
            /* TRN shift left, TC set based on comparison */
            s->trn <<= 1;
            if (acc >= cmp) {
                s->st0 |= ST0_TC;
                s->trn |= 1;
            } else {
                s->st0 &= ~ST0_TC;
                if (src) s->b = cmp; else s->a = cmp;
            }
            return consumed + s->lk_used;
        }
        /* AExx/AFxx: MACD Smem, pmad, dst — MAC + data move (2 words)
         * dst += T * Smem, then data(Smem) → data(dmad)
         * pmad in second word auto-increments during RPT */
        if (hi8 == 0xAE || hi8 == 0xAF) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Data move: read from prog[pmad], write to data[addr] */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            data_write(s, addr, prog_fetch(s, psrc));
            s->mvpd_src = psrc + 1;
            s->t = sval;  /* T = old Smem value (before overwrite) */
            return consumed + s->lk_used;
        }
        /* ACxx/ADxx: MACP Smem, pmad, dst — MAC + program fetch (2 words) */
        if (hi8 == 0xAC || hi8 == 0xAD) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t sval = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)sval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int dst = (hi8 & 0x01);
            if (dst) s->b = sext40(s->b + prod);
            else     s->a = sext40(s->a + prod);
            /* Coeff fetch from program memory */
            uint16_t psrc = s->rpt_active ? s->mvpd_src : op2;
            s->t = prog_fetch(s, psrc);
            s->mvpd_src = psrc + 1;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xB3) {
            /* LD #lk, dst (long immediate, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = (op >> 0) & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed + s->lk_used;
        }
        /* ADD #lk, src[, dst] */
        if (hi8 == 0xA2) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b + (v << 16));
            else     s->a = sext40(s->a + (v << 16));
            return consumed + s->lk_used;
        }
        /* SUB #lk */
        if (hi8 == 0xA3) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b - (v << 16));
            else     s->a = sext40(s->a - (v << 16));
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */

        /* ---- Dual-operand MAC/MAS Xmem, Ymem, dst (1-word) ----
         * 0xD0: MAC Xmem,Ymem,A   0xD2: MAC Xmem,Ymem,B
         * 0xD1: MACR Xmem,Ymem,A  0xD3: MACR Xmem,Ymem,B
         * 0xD4-0xD7: MAS variants (subtract) */
        if (hi8 >= 0xD0 && hi8 <= 0xD9 && hi8 != 0xDA) {
            int xar_c = (op >> 4) & 0x07;
            int yar_c = op & 0x07;
            uint16_t xval_c = data_read(s, s->ar[xar_c]);
            uint16_t yval_c = data_read(s, s->ar[yar_c]);
            if ((op >> 7) & 1) s->ar[xar_c]--; else s->ar[xar_c]++;
            if ((op & 0x08) == 0) s->ar[yar_c]++; else s->ar[yar_c]--;
            int64_t prod_c = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_c;
            if (s->st1 & ST1_FRCT) prod_c <<= 1;
            if (hi8 & 0x01) prod_c += 0x8000; /* round */
            int is_sub_c = (hi8 >= 0xD4);
            int dst_c = (hi8 & 0x02) ? 1 : 0;
            if (dst_c) {
                if (is_sub_c) s->b = sext40(s->b - prod_c);
                else          s->b = sext40(s->b + prod_c);
            } else {
                if (is_sub_c) s->a = sext40(s->a - prod_c);
                else          s->a = sext40(s->a + prod_c);
            }
            s->t = yval_c;
            return consumed + s->lk_used;
        }

        /* DBxx: MASA Xmem, Ymem, dst — MAC with accumulator sign extension
         * Per SPRU172C: same as MAC but T loaded from Xmem instead of Ymem.
         * dst += T * Xmem, T = Xmem */
        if (hi8 == 0xDB) {
            int xar_db = (op >> 4) & 0x07;
            int yar_db = op & 0x07;
            uint16_t xval_db = data_read(s, s->ar[xar_db]);
            (void)data_read(s, s->ar[yar_db]); /* Ymem read (unused) */
            if ((op >> 7) & 1) s->ar[xar_db]--; else s->ar[xar_db]++;
            if ((op & 0x08) == 0) s->ar[yar_db]++; else s->ar[yar_db]--;
            int64_t prod_db = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval_db;
            if (s->st1 & ST1_FRCT) prod_db <<= 1;
            s->a = sext40(s->a + prod_db);
            s->t = xval_db;
            return consumed + s->lk_used;
        }

        /* DCxx: SQUR Xmem, dst — Square and accumulate (1-word dual-operand)
         * Per SPRU172C p.4-165: T = Xmem, dst = dst + T * T
         * Encoding: 1101 1100 XXXX YYYY (Ymem read but unused for SQUR) */
        if (hi8 == 0xDC) {
            int xar_dc = (op >> 4) & 0x07;
            int yar_dc = op & 0x07;
            uint16_t xval_dc = data_read(s, s->ar[xar_dc]);
            (void)data_read(s, s->ar[yar_dc]); /* Ymem pipeline read */
            if ((op >> 7) & 1) s->ar[xar_dc]--; else s->ar[xar_dc]++;
            if ((op & 0x08) == 0) s->ar[yar_dc]++; else s->ar[yar_dc]--;
            s->t = xval_dc;
            int64_t prod_dc = (int64_t)(int16_t)xval_dc * (int64_t)(int16_t)xval_dc;
            if (s->st1 & ST1_FRCT) prod_dc <<= 1;
            s->a = sext40(s->a + prod_dc);
            return consumed + s->lk_used;
        }

        /* CA/CB: ST || LD parallel (dual-operand, 1-word) — like C8/C9 */
        if (hi8 == 0xCA || hi8 == 0xCB) {
            int s_acc_ca = (hi8 == 0xCB) ? 1 : 0;
            int xar_ca = (op >> 4) & 0x07;
            int yar_ca = op & 0x07;
            int64_t st_val_ca = s_acc_ca ? s->b : s->a;
            data_write(s, s->ar[yar_ca], (uint16_t)(st_val_ca & 0xFFFF));
            uint16_t ld_val_ca = data_read(s, s->ar[xar_ca]);
            int d_acc_ca = s_acc_ca ? 0 : 1;
            int64_t loaded_ca = (int64_t)(int16_t)ld_val_ca << 16;
            if (d_acc_ca) s->b = sext40(loaded_ca); else s->a = sext40(loaded_ca);
            if ((op >> 7) & 1) s->ar[xar_ca]--; else s->ar[xar_ca]++;
            if ((op & 0x08) == 0) s->ar[yar_ca]++; else s->ar[yar_ca]--;
            return consumed + s->lk_used;
        }
        /* CF: variant parallel or DELAY */
        if (hi8 == 0xCF) {
            /* Treat as NOP for now — rare instruction */
            return consumed + s->lk_used;
        }
        /* RPTB[D] pmad — Block repeat (2 words)
         * C2xx: RPTB pmad, C3xx: RPTBD pmad (delayed)
         * Per SPRU172C: RSA = PC+2, REA = pmad, BRAF = 1 */
        if (hi8 == 0xC2 || hi8 == 0xC3 || hi8 == 0xC6 || hi8 == 0xC7) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 2);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC5) {
            /* PSHM MMR */
            uint16_t mmr = op & 0x1F;
            s->sp--;
            data_write(s, s->sp, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCD) {
            /* POPM MMR */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCE) {
            /* FRAME #k (signed 8-bit) */
            int8_t k = (int8_t)(op & 0xFF);
            s->sp += k;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xC0 || hi8 == 0xC1) {
            /* PSHD Smem / RPT Smem variants */
            addr = resolve_smem(s, op, &ind);
            if (hi8 == 0xC0) {
                /* PSHD Smem */
                s->sp--;
                data_write(s, s->sp, data_read(s, addr));
            } else {
                /* RPT Smem */
                s->rpt_count = data_read(s, addr);
                s->rpt_active = true;
                s->pc += consumed;
                return 0;
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCC) {
            /* CCxx: SACCD Smem, ARmem — Store Acc Conditionally (1-word)
             * Per SPRU172C: conditionally store AH or BH to Smem.
             * Simplified: always store (condition always true). */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, (uint16_t)((s->a >> 16) & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDD) {
            /* POPD Smem */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDE) {
            /* DExx: POPD dmad (2-word) */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, s->sp));
            s->sp++;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xDF) {
            /* DELAY Smem — shift delay line: data(Smem) → data(Smem+1)
             * Per SPRU172C: used with RPT for FIR filter delay lines */
            addr = resolve_smem(s, op, &ind);
            uint16_t dval = data_read(s, addr);
            data_write(s, addr + 1, dval);
            return consumed + s->lk_used;
        }
        /* 0xC8/0xC9: ST || LD parallel instruction (1 word, dual indirect)
         * Store low acc to Ymem while loading Xmem to other acc */
        if (hi8 == 0xC8 || hi8 == 0xC9) {
            int s_acc = (hi8 == 0xC9) ? 1 : 0;
            int xar = (op >> 4) & 0x07;
            int yar = op & 0x07;
            int64_t st_val = s_acc ? s->b : s->a;
            data_write(s, s->ar[yar], (uint16_t)(st_val & 0xFFFF));
            uint16_t ld_val = data_read(s, s->ar[xar]);
            int d_acc = s_acc ? 0 : 1;
            int64_t loaded = (int64_t)(int16_t)ld_val << 16;
            if (d_acc) s->b = sext40(loaded); else s->a = sext40(loaded);
            if ((op >> 7) & 1) s->ar[xar]--; else s->ar[xar]++;
            if ((op & 0x08) == 0) s->ar[yar]++; else s->ar[yar]--;
            return consumed + s->lk_used;
        }
        goto unimpl;

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 200 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed + s->lk_used;
}

/* ================================================================
 * Main execution loop
 * ================================================================ */

int c54x_run(C54xState *s, int n_insns)
{
    int executed = 0;

    /* Log first 10 instructions of each run (for 2nd cycle debug) */
    static int run_num = 0;
    run_num++;

    while (executed < n_insns && s->running && !s->idle) {
        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;

        /* Trace DSP init - log once per unique PC in E900-E960 */
        if (s->pc >= 0xE900 && s->pc < 0xE960 && !s->rpt_active) {
            static uint16_t seen_pcs[96];
            int idx = s->pc - 0xE900;
            if (!seen_pcs[idx]) {
                seen_pcs[idx] = 1;
                C54_LOG("INIT PC=0x%04x op=0x%04x SP=0x%04x BRC=%d rptb=%d RSA=0x%04x REA=0x%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, s->brc,
                        s->rptb_active, s->rsa, s->rea);
            }
        }

        /* Trace SINT17 handler (0x8a00-0x8a5f) */
        if (s->pc >= 0x8a00 && s->pc < 0x8a60) {
            static int sint17_log = 0;
            if (sint17_log < 500) {
                C54_LOG("SINT17 PC=0x%04x op=0x%04x SP=0x%04x DP=0x%03x A=0x%010llx B=0x%010llx AR0=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp, dp(s),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->ar[0]);
                sint17_log++;
            }
        }

        /* Sample PC every 1M instructions to find stuck loops */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (run_num <= 2 && executed < 2000) {
            C54_LOG("BOOT[%d.%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx B=0x%010llx",
                    run_num, executed, s->pc, prog_fetch(s, s->pc), s->sp,
                    (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                    (unsigned long long)(s->b & 0xFFFFFFFFFFLL));
        }
        /* Check RPTB (block repeat) — skip during RPT (single repeat has priority) */
        if (s->rptb_active && !s->rpt_active && s->pc == s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) { C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d", s->pc, s->rsa, s->rea, s->brc); rptb_log++; }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false; C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp);
                s->st1 &= ~ST1_BRAF;
            }
        }

        /* Trace the IMR loop: how does the DSP reach 0x03F0? */
        /* Trace RPTB entry at 0x76FD: dump all AR values */
        if (s->pc == 0x76FD) {
            static int rptb_entry_log = 0;
            if (rptb_entry_log < 30)
                C54_LOG("RPTB-ENTRY PC=0x76FD AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x ARP=%d DP=%d BRC=%d SP=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                        arp(s), dp(s), s->brc, s->sp);
            rptb_entry_log++;
        }
        if (s->pc == 0x03F0) {
            static int f3_log = 0;
            if (f3_log < 2) {
                C54_LOG("PC=0x03F0 op=0x%04x insn=%u SP=0x%04x IMR=0x%04x XPC=%d PMST=0x%04x",
                        prog_fetch(s, s->pc), s->insn_count, s->sp, s->imr, s->xpc, s->pmst);
                C54_LOG("  trail: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
                        pc_ring[(pc_ring_idx-20)&255], pc_ring[(pc_ring_idx-19)&255],
                        pc_ring[(pc_ring_idx-18)&255], pc_ring[(pc_ring_idx-17)&255],
                        pc_ring[(pc_ring_idx-16)&255], pc_ring[(pc_ring_idx-15)&255],
                        pc_ring[(pc_ring_idx-14)&255], pc_ring[(pc_ring_idx-13)&255],
                        pc_ring[(pc_ring_idx-12)&255], pc_ring[(pc_ring_idx-11)&255],
                        pc_ring[(pc_ring_idx-10)&255], pc_ring[(pc_ring_idx-9)&255],
                        pc_ring[(pc_ring_idx-8)&255], pc_ring[(pc_ring_idx-7)&255],
                        pc_ring[(pc_ring_idx-6)&255], pc_ring[(pc_ring_idx-5)&255],
                        pc_ring[(pc_ring_idx-4)&255], pc_ring[(pc_ring_idx-3)&255],
                        pc_ring[(pc_ring_idx-2)&255], pc_ring[(pc_ring_idx-1)&255]);
                f3_log++;
            }
        }

        /* Boot trace */
        if (g_boot_trace > 0) {
            C54_LOG("BOOT[%d] PC=0x%04x op=0x%04x SP=0x%04x PMST=0x%04x",
                    51 - g_boot_trace, s->pc, prog_fetch(s, s->pc), s->sp, s->pmst);
            g_boot_trace--;
        }

        /* Execute instruction */
        int consumed;
        uint16_t exec_pc = s->pc;
        uint16_t exec_op = prog_fetch(s, s->pc);
        consumed = c54x_exec_one(s);

        /* Detect SP changes — only log after init (insn > 490M) */
        if (s->sp != sp_before && s->insn_count > 490000000) {
            static int sp_leak_log = 0;
            if (sp_leak_log < 100) {
                C54_LOG("SP %+d PC=0x%04x op=0x%04x SP 0x%04x→0x%04x insn=%u",
                        (int16_t)(s->sp - sp_before), exec_pc, exec_op, sp_before, s->sp, s->insn_count);
                sp_leak_log++;
            }
        }

        /* RPT: after executing an instruction while repeat is active,
         * re-execute the SAME instruction (don't advance PC) until count=0. */
        if (s->rpt_active && !s->idle) {
            if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Don't advance PC — re-execute same instruction next cycle */
                s->cycles++;
                executed++;
                if (s->rpt_count == 0) {
                    static int rpt_done_log = 0;
                    if (rpt_done_log < 10)
                        C54_LOG("RPT DONE PC=0x%04x op=0x%04x count_was=%d", s->pc, prog_fetch(s, s->pc), 0);
                    rpt_done_log++;
                }
                continue;
            } else {
                s->rpt_active = false;
                s->par_set = false;
            }
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        s->cycles++;
        s->insn_count++;
        executed++;
    }
    return executed;
}

/* ================================================================
 * ROM loader
 * ================================================================ */

int c54x_load_rom(C54xState *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        C54_LOG("Cannot open ROM dump: %s", path);
        return -1;
    }

    char line[1024];
    int section = -1; /* 0=regs, 1=DROM, 2=PDROM, 3-6=PROM0-3 */
    
    int total_words = 0;

    while (fgets(line, sizeof(line), f)) {
        /* Section headers */
        if (strstr(line, "DSP dump: Registers"))  { section = 0; continue; }
        if (strstr(line, "DSP dump: DROM"))        { section = 1; continue; }
        if (strstr(line, "DSP dump: PDROM"))       { section = 2; continue; }
        if (strstr(line, "DSP dump: PROM0"))       { section = 3; continue; }
        if (strstr(line, "DSP dump: PROM1"))       { section = 4; continue; }
        if (strstr(line, "DSP dump: PROM2"))       { section = 5; continue; }
        if (strstr(line, "DSP dump: PROM3"))       { section = 6; continue; }
        if (section < 0) continue;

        /* Parse data lines: "ADDR : XXXX XXXX XXXX ..." */
        uint32_t addr;
        if (sscanf(line, "%x :", &addr) != 1) continue;

        char *p = strchr(line, ':');
        if (!p) continue;
        p++;

        uint16_t word;
        while (sscanf(p, " %hx%n", &word, (int[]){0}) == 1) {
            int n;
            sscanf(p, " %hx%n", &word, &n);
            p += n;

            if (section == 0) {
                /* Registers: store in data memory */
                if (addr < 0x60) s->data[addr] = word;
            } else if (section == 1 || section == 2) {
                /* DROM/PDROM: data memory */
                if (addr < C54X_DATA_SIZE) s->data[addr] = word;
            } else {
                /* PROM: program memory.
                 * The dump uses extended addresses (XPC pages):
                 *   PROM0: 0x07000-0x0DFFF → prog space 0x7000-0xDFFF
                 *   PROM1: 0x18000-0x1FFFF → prog space 0x8000-0xFFFF (page 1)
                 *   PROM2: 0x28000-0x2FFFF → prog space 0x8000-0xFFFF (page 2)
                 *   PROM3: 0x38000-0x39FFF → prog space 0xF800-0xFFFF (page 3)
                 * For 16-bit PC access, map all PROM to lower 64K too.
                 * PROM0 is already at 0x7000. For PROM1-3, also mirror
                 * to the 16-bit alias (0x8000-0xFFFF). */
                if (addr < C54X_PROG_SIZE) s->prog[addr] = word;
                /* Mirror PROM1 (page 1: 0x18000-0x1FFFF) to 16-bit space.
                 * PROM0 occupies 0x7000-0xDFFF — only mirror PROM1 above that
                 * (0xE000-0xFFFF) to avoid overwriting PROM0 data.
                 * This gives us interrupt vectors at 0xFF80. */
                if (section == 4) {  /* PROM1 only */
                    uint16_t addr16 = addr & 0xFFFF;
                    /* Mirror PROM1 to 0xE000-0xFF7F only.
                     * 0xFF80-0xFFFF is the interrupt vector table,
                     * populated by the DSP boot ROM (not PROM1). */
                    if (addr16 >= 0xE000)
                        s->prog[addr16] = word;
                }
            }
            addr++;
            total_words++;
        }
    }

    fclose(f);
    C54_LOG("Loaded ROM: %d words from %s", total_words, path);
    return 0;
}

/* ================================================================
 * Init / Reset / Interrupts
 * ================================================================ */

C54xState *c54x_init(void)
{
    C54xState *s = calloc(1, sizeof(C54xState));
    if (!s) return NULL;
    return s;
}

void c54x_set_api_ram(C54xState *s, uint16_t *api_ram)
{
    s->api_ram = api_ram;
}

void c54x_reset(C54xState *s)
{
    g_boot_trace = 50;
    s->a = 0; s->b = 0;
    memset(s->ar, 0, sizeof(s->ar));
    s->t = 0; s->trn = 0;
    s->sp = 0x5AC8; s->bk = 0;  /* SP init per Calypso boot ROM */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    s->st0 = 0;
    s->st1 = ST1_INTM;  /* interrupts disabled at reset */
    s->pmst = 0xFFE0;   /* IPTR = 0x1FF, OVLY = 0 at reset */
    s->imr = 0;
    s->ifr = 0;
    s->xpc = 0;
    s->timer_psc = 0;
    s->data[TCR_ADDR] = TCR_TSS;  /* Timer stopped at reset (TSS=1) per HW spec */
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp);
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1. */
    for (int i = 0; i < 0x2780; i++)
        s->data[0x0080 + i] = s->prog[0x7080 + i];

    /* Install boot ROM interrupt vectors at 0xFF80 (IPTR=0x1FF).
     * These are from the Calypso internal boot ROM, not in the PROM dump.
     * Vec0 (reset): B 0xB410 (bootloader entry) */
    s->prog[0xFF80] = 0xF880;  /* B pmad */
    s->prog[0xFF81] = 0xB410;  /* target: bootloader */
    s->prog[0xFF82] = 0xF495;  /* NOP */
    s->prog[0xFF83] = 0xF495;  /* NOP */
    /* Vec1-7: use PROM1 ROM vectors (already mirrored to 0xFF84-0xFFFF).
     * Do NOT overwrite — the ROM contains the real interrupt handlers. */

    /* Boot ROM stubs at 0x0000-0x007F.
     * The internal Calypso boot ROM occupies prog 0x0000-0x007F but is not
     * in the PROM dump. The DSP init code does CALA with A(low)=0x0000 to
     * call boot ROM routines. The handler at 0x8A07 expects B(high) = SP
     * (restored via STH B,SP at 0x8A46 after RETE). Stub: LDM SP,B; RET */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF073;  /* RET */
    /* Entry point 0x0000: save SP into B for the SINT17 handler */
    s->prog[0x0000] = 0xBA18;  /* LDMM SP, B — read MMR 0x18(SP) into B(high) */
    s->prog[0x0001] = 0xF073;  /* RET */

    /* Reset vector: IPTR * 0x80 */
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->pc = iptr * 0x80;  /* 0xFF80 for default PMST */

    C54_LOG("Reset: PC=0x%04x PMST=0x%04x SP=0x%04x prog[PC]=0x%04x",
            s->pc, s->pmst, s->sp, s->prog[s->pc]);
}

void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit)
{
    if (vec < 0 || vec >= 32) return;
    if (imr_bit < 0 || imr_bit >= 16) return;
    s->ifr |= (1 << imr_bit);

    bool unmasked = (s->imr & (1 << imr_bit)) != 0;

    /* Per SPRU131: IDLE exits on ANY interrupt (masked or unmasked).
     * - Unmasked: branch to vector, set INTM=1
     * - Masked: just resume after IDLE, IFR bit stays set */
    if (s->idle) {
        s->idle = false;
        if (unmasked) {
            /* Service the interrupt: branch to vector */
            s->ifr &= ~(1 << imr_bit);
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 1));
            if (s->pmst & PMST_APTS) {
                s->sp--;
                data_write(s, s->sp, s->xpc);
            }
            s->st1 |= ST1_INTM;
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            s->pc = (iptr * 0x80) + vec * 4;
        }
        /* If masked: just wake, advance PC past IDLE */
        if (!unmasked) {
            s->pc++;  /* resume at instruction after IDLE */
        }
    } else if (!(s->st1 & ST1_INTM) && unmasked) {
        /* Normal (non-IDLE) interrupt servicing */
        s->ifr &= ~(1 << imr_bit);
        s->sp--;
        data_write(s, s->sp, (uint16_t)s->pc);
        if (s->pmst & PMST_APTS) {
            s->sp--;
            data_write(s, s->sp, s->xpc);
        }
        s->st1 |= ST1_INTM;
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;
    }

    /* Log first few interrupts */
    static int int_log_count = 0;
    if (int_log_count < 5) {
        C54_LOG("IRQ vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x idle=%d PC=0x%04x",
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr, s->idle, s->pc);
        int_log_count++;
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 160) n = 160;
    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;
}
