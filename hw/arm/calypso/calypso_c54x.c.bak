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
            if (val != s->pmst)
                C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x OVLY=%d) PC=0x%04x",
                        s->pmst, val, (val >> PMST_IPTR_SHIFT) & 0x1FF, !!(val & PMST_OVLY), s->pc);
            s->pmst = val; return;
        case MMR_XPC:
            if (val > 3) {
                C54_LOG("XPC invalid write 0x%04x PC=0x%04x — clamped", val, s->pc);
                val &= 3;
            }
            s->xpc = val; return;
        default: return;
        }
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        if (s->api_ram)
            s->api_ram[addr - C54X_API_BASE] = val;
        /* Log API writes with context */
        static int api_log = 0;
        if (api_log < 50) {
            C54_LOG("API WR [0x%04x] = 0x%04x PC=0x%04x insn=%u", addr, val, s->pc, s->insn_count);
            api_log++;
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

static uint16_t prog_read(C54xState *s, uint32_t addr)
{
    uint16_t addr16 = addr & 0xFFFF;
    /* OVLY: DARAM visible in program space for 0x0080-0x7FFF */
    if ((s->pmst & PMST_OVLY) && addr16 < 0x8000 && addr16 >= 0x80)
        return s->data[addr16];
    /* For addresses >= 0x8000: use XPC to select extended page.
     * Extended address = (XPC << 16) | addr16.
     * PROM1 at 0x18000, PROM2 at 0x28000, PROM3 at 0x38000. */
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
    if ((s->pmst & PMST_OVLY) && addr16 < 0x8000 && addr16 >= 0x80)
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
        case 0xC: /* *(lk) — next word is address */
            /* handled by caller */
            break;
        case 0xD: /* *+ARn(lk) */
            /* handled by caller */
            break;
        case 0xE: /* *ARn(lk) */
            /* handled by caller */
            break;
        case 0xF: /* *+ARn(lk)% */
            /* handled by caller */
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
    uint16_t op = prog_read(s, s->pc);
    uint16_t op2;
    bool ind;
    uint16_t addr;
    int consumed = 1;

    uint8_t hi4 = (op >> 12) & 0xF;
    uint8_t hi8 = (op >> 8) & 0xFF;

    /* Log every instruction of SINT17 handler (short runs after IDLE) */
    if (s->pc >= 0xFFC0 && s->pc <= 0xFFFF) {
        static int h_log = 0;
        if (h_log < 10) {
            C54_LOG("HIGH_PC PC=0x%04x op=0x%04x insn=%u idle=%d rptb=%d rpt=%d",
                    s->pc, op, s->insn_count, s->idle, s->rptb_active, s->rpt_active);
            h_log++;
        }
    }

    switch (hi4) {
    case 0xF:
        /* 0xF --- large group: branches, misc, short immediates */
        if (op == 0xF495) return consumed;  /* NOP */

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
            return consumed;  /* condition true: just advance past XC, execute next normally */
        }

        /* F4E2 = RSBX INTM (enable interrupts), F4E3 = SSBX INTM (disable interrupts) */
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed; }
        if (op == 0xF4E4) {
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        s->pc, !!(s->st1 & ST1_INTM), s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            /* TDMA slot table (0x8000-0x8020): skip IDLE, continue next slot.
             * All other IDLEs: halt. Wake behavior decided by calypso_trx. */
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed;
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
            return 0;
        } /* IDLE */
        if (hi8 == 0xF4) {
            /* F4xx: unconditional branch/call */
            uint8_t sub = (op >> 4) & 0xF;
            op2 = prog_read(s, s->pc + 1);
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
            case 0x6: /* CALL pmad */
            case 0x8: /* CALLD pmad */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            case 0x7: /* CALA src — call to accumulator */
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
            case 0xF: /* F4Fx: FCALL extpmad — far call (2 words) per SPRU172C p.4-57 */
            {
                /* Push PC+2 then XPC (2 pushes) */
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->sp--;
                data_write(s, s->sp, s->xpc);
                s->pc = op2;  /* pmad(15:0) */
                /* XPC set from bits in opcode or second word — depends on encoding variant */
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
            /* F073: RET */
            if (op == 0xF073) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra; return 0;
            }
            /* F074: RETE */
            if (op == 0xF074) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                static int rete_log = 0;
                if (rete_log < 10)
                    C54_LOG("RETE SP=0x%04x→0x%04x RA=0x%04x insn=%u",
                            s->sp-1, s->sp, ra, s->insn_count);
                rete_log++;
                s->pc = ra; return 0;
            }
            /* F072: FRET — pop XPC then PC (2 pops) per SPRU172C p.4-61 */
            if (op == 0xF072) {
                s->xpc = data_read(s, s->sp); s->sp++;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra; return 0;
            }
            /* F07x: other control flow */
            if ((op & 0xFFF0) == 0xF070) {
                /* F075: RETD, F076: RETED, etc. — treat as RET */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->pc = ra; return 0;
            }
            /* F0Bx/F1Bx: RSBX/SSBX */
            if ((op & 0x00F0) == 0x00B0) {
                int bit = op & 0x0F;
                int set = (op >> 8) & 1;
                int st = (op >> 5) & 1;
                if (st == 0) { if (set) s->st0 |= (1<<bit); else s->st0 &= ~(1<<bit); }
                else         { if (set) s->st1 |= (1<<bit); else s->st1 &= ~(1<<bit); }
                return consumed;
            }
            /* F0xx Smem: various single-operand ops */
            if ((op & 0xFF00) == 0xF000) {
                /* F0xx with Smem: could be NORM, etc. */
                addr = resolve_smem(s, op, &ind);
                op2 = prog_read(s, s->pc + 1);
                consumed = 2;
                /* Treat as RET/branch to op2 (many F0xx are control flow with 2 words) */
                s->pc = op2;
                return 0;
            }
            /* F010: NOP or TRAP */
            if (op == 0xF010) return consumed;
            goto unimpl;
        }
        if (op == 0xF495) {
            /* NOP */
            return consumed;
        }
        if (op == 0xF4E2) { s->st1 &= ~ST1_INTM; return consumed; }
        if (op == 0xF4E3) { s->st1 |= ST1_INTM; return consumed; }
        if (op == 0xF4E4) {
            /* IDLE */
            s->idle = true;
            return consumed;  /* Advance PC past IDLE */
        }
        /* FXXX short immediates and misc */
        if (hi8 == 0xF0 || hi8 == 0xF1) {
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
                    uint16_t ret_addr = data_read(s, s->sp);
                    s->sp++;
                    s->st1 &= ~ST1_INTM;  /* re-enable interrupts */
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
                return consumed;
            }
        }
        /* F8xx: RPT */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x2) {
                /* F82x: RPTB pmad */
                op2 = prog_read(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed;
            }
            if (sub == 0x3) {
                /* F83x: RPT #k (short) */
                /* Or other F83x variants */
                op2 = prog_read(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_pc = (uint16_t)(s->pc + 2);
                s->rpt_active = true;
                return consumed;
            }
            /* F80x-F83x fallthrough: RPT Smem */
            addr = resolve_smem(s, op, &ind);
            s->rpt_count = data_read(s, addr);
            s->rpt_pc = (uint16_t)(s->pc + consumed);
            s->rpt_active = true;
            return consumed;
        }
        /* F2xx: various — NORM, CMPS, etc. */
        if (hi8 == 0xF2) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x7) {
                /* F27x: RET/RETE variants with condition */
                op2 = prog_read(s, s->pc + 1);
                consumed = 2;
                uint16_t ret_addr = data_read(s, s->sp);
                s->sp++;
                s->pc = ret_addr;
                return 0;
            }
            goto unimpl;
        }
        /* F3xx: various */
        if (hi8 == 0xF3) {
            /* LD #k9, DP or LD #k9, ARP etc */
            uint16_t k9 = op & 0x1FF;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            return consumed;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
            if (sub == 0x2) {
                /* F62x: LD A, dst_shift, B or LD B, dst_shift, A */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed;
            }
            if (sub == 0x6) {
                /* F66x: LD A/B with shift to other acc */
                int dst = op & 1;
                if (dst) s->b = s->a; else s->a = s->b;
                return consumed;
            }
            /* Other F6xx: treat as NOP for now */
            return consumed;
        }
        /* F5xx: RPT #k (short immediate, k in low byte) */
        if (hi8 == 0xF5) {
            s->rpt_count = op & 0xFF;
            s->rpt_pc = (uint16_t)(s->pc + 1);
            s->rpt_active = true;
            return consumed;
        }
        /* F7xx: LD/ST #k to various registers */
        if (hi8 == 0xF7) {
            uint8_t sub = (op >> 4) & 0xF;
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
            return consumed;
        }
        /* F9xx: RPT #lk (16-bit immediate) */
        if (hi8 == 0xF9) {
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            s->rpt_count = op2;
            s->rpt_pc = (uint16_t)(s->pc + 2);
            s->rpt_active = true;
            return consumed;
        }
        /* FAxx: RPT Smem or conditional ops */
        if (hi8 == 0xFA) {
            /* FA3x: BC with delay, FA4x: conditional etc. */
            op2 = prog_read(s, s->pc + 1);
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
            return consumed;
        }
        /* FCxx: LD #k, 16, B */
        if (hi8 == 0xFC) {
            int8_t k = (int8_t)(op & 0xFF);
            int64_t v = (int64_t)k << 16;
            s->b = sext40(v);
            return consumed;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed;
        }
        /* FExx: LD #k, B (no shift) */
        if (hi8 == 0xFE) {
            int8_t k = (int8_t)(op & 0xFF);
            s->b = sext40((int64_t)k);
            return consumed;
        }
        /* FFxx: ADD/SUB short immediate */
        if (hi8 == 0xFF) {
            int8_t k = (int8_t)(op & 0x7F);
            int dst = (op >> 7) & 1;
            /* Typically ADD #k, A or SUB */
            if (dst) s->b = sext40(s->b + ((int64_t)k << 16));
            else     s->a = sext40(s->a + ((int64_t)k << 16));
            return consumed;
        }
        goto unimpl;

    case 0xE:
        /* Exxxx: single-word ALU, status, misc */
        if (hi8 == 0xEA) {
            /* BANZ pmad, Sind (2 words) — Calypso ROM uses 0xEA encoding.
             * Per SPRU172C p.4-16, standard BANZ is 0x78xx but the Calypso
             * DSP ROM uses 0xEA as an alternate BANZ encoding.
             * if (AR[arp] != 0) then pmad→PC; AR always decremented */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            int n = arp(s);
            if (s->ar[n] != 0) {
                s->ar[n]--;
                s->pc = op2;
                return 0;
            }
            s->ar[n]--;
            return consumed;
        }
        if (hi8 == 0xEC) {
            /* EC: BC pmad, cond (conditional branch, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            /* Simplified: evaluate common conditions */
            uint8_t cond = op & 0xFF;
            bool take = false;
            /* Condition evaluation (simplified) */
            if (cond == 0x03) take = (s->a == 0);        /* AEQ */
            else if (cond == 0x0B) take = (s->b == 0);   /* BEQ */
            else if (cond == 0x02) take = (s->a != 0);   /* ANEQ */
            else if (cond == 0x0A) take = (s->b != 0);   /* BNEQ */
            else if (cond == 0x00) take = (s->a < 0);    /* ALT */
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
            return consumed;
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
            return consumed;
        }
        if (hi8 == 0xE4) {
            /* E4xx: BITF Smem, #lk (2-word) or BIT Smem, bit */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint16_t val = data_read(s, addr);
            s->st0 = (val & op2) ? (s->st0 | ST0_TC) : (s->st0 & ~ST0_TC);
            return consumed;
        }
        if (hi8 == 0xE7) {
            /* E7xx: ST #k, Smem or LD #k,16, dst (short immediate) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed;
        }
        if (hi8 == 0xE9) {
            /* E9xx: CC pmad, cond (conditional call, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x00) take = (s->a < 0);
            else if (cond == 0x08) take = (s->b < 0);
            else if (cond == 0x04) take = (s->a > 0);
            else if (cond == 0x0C) take = (s->b > 0);
            else if (cond == 0x40) take = (s->st0 & ST0_TC) != 0;
            else if (cond == 0x41) take = !(s->st0 & ST0_TC);
            else if (cond == 0x20) take = (s->st0 & ST0_C) != 0;
            else if (cond == 0x21) take = !(s->st0 & ST0_C);
            else take = true;
            if (take) {
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }
            return consumed;
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
            return consumed;
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
            return consumed;
        }
        if (hi8 == 0xEE) {
            /* EExx: BCD pmad, cond (conditional delayed branch, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x03) take = (s->a == 0);
            else if (cond == 0x0B) take = (s->b == 0);
            else if (cond == 0x02) take = (s->a != 0);
            else if (cond == 0x0A) take = (s->b != 0);
            else if (cond == 0x00) take = (s->a < 0);
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
            return consumed;
        }
        if (hi8 == 0xED) {
            /* EDxx: BCD pmad, cond (conditional branch delayed, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint8_t cond = op & 0xFF;
            bool take = false;
            if (cond == 0x00) take = (s->a < 0);
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
            return consumed;
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
            return consumed;
        }
        goto unimpl;

    case 0x6: case 0x7:
        /* LD / ST operations */
        if ((op & 0xF800) == 0x7000) {
            /* 70xx: STL src, Smem */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)(acc & 0xFFFF));
            return consumed;
        }
        if ((op & 0xF800) == 0x7800) {
            /* 78xx-7Fxx: STH src, Smem
             * Note: BANZ (0x78xx per doc) shares this range but is handled
             * via F84x (BANZ with condition) in the F8xx group. */
            int src_acc = (op >> 9) & 1;
            addr = resolve_smem(s, op, &ind);
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, addr, (uint16_t)((acc >> 16) & 0xFFFF));
            return consumed;
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
            return consumed;
        }
        if ((op & 0xF800) == 0x6800) {
            /* 68xx: LD Smem, T */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed;
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
        return consumed;

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
        return consumed;

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
        return consumed;

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
                return consumed;
            case 0x4: case 0x5: /* SQUR Smem, A/B */
                product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                s->t = val;
                if (sub & 1) s->b = sext40(product);
                else         s->a = sext40(product);
                return consumed;
            case 0x8: case 0x9: /* MPYA Smem (A = T * Smem, B += A) or variants */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (sub & 1) { s->a += s->b; s->b = sext40(product); }
                else         { s->b += s->a; s->a = sext40(product); }
                return consumed;
            case 0xA: case 0xB: /* MACA[R] Smem, A/B (A += B * Smem then B = T * Smem) */
                dst = sub & 1;
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                if (dst) { s->a = sext40(s->a + s->b); s->b = sext40(product); }
                else     { s->b = sext40(s->b + s->a); s->a = sext40(product); }
                s->t = val;
                return consumed;
            default:
                /* MAS variants and others */
                product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
                if (s->st1 & ST1_FRCT) product <<= 1;
                dst = sub & 1;
                if (dst) s->b = sext40(s->b - product);
                else     s->a = sext40(s->a - product);
                return consumed;
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
        return consumed;

    case 0x5:
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
        return consumed;

    case 0x8: case 0x9:
        /* 8xxx/9xxx: Memory moves, PORTR/PORTW */
        if (hi8 == 0x8A) {
            /* MVDK Smem, dmad */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed;
        }
        if (hi8 == 0x9A) {
            /* MVKD dmad, Smem */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed;
        }
        if (hi8 == 0x88 || hi8 == 0x80) {
            /* MVDD Smem, Smem (data→data) — 2 address forms */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed;
        }
        if (hi8 == 0x8C) {
            /* MVPD pmad, Smem (prog→data) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed;
        }
        if (hi8 == 0x8E) {
            /* MVDP Smem, pmad (data→prog) */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            prog_write(s, op2, data_read(s, addr));
            return consumed;
        }
        if (hi8 == 0x8F) {
            /* PORTR PA, Smem — read I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
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
            return consumed;
        }
        if (hi8 == 0x9F) {
            /* PORTW Smem, PA — write I/O port */
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
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
            return consumed;
        }
        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
        if (hi8 == 0x85) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, prog_read(s, op2));
            return consumed;
        }
        /* 86xx: MVDM dmad, MMR */
        if (hi8 == 0x86) {
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, op2));
            return consumed;
        }
        /* 87xx: MVMD MMR, dmad */
        if (hi8 == 0x87) {
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x1F;
            data_write(s, op2, data_read(s, mmr));
            return consumed;
        }
        /* 81xx: STL src, ASM, Smem (store with shift) */
        if (hi8 == 0x81) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)(v & 0xFFFF));
            return consumed;
        }
        /* 82xx: STH src, ASM, Smem */
        if (hi8 == 0x82) {
            addr = resolve_smem(s, op, &ind);
            int shift = asm_shift(s);
            int64_t v = s->a;
            if (shift >= 0) v <<= shift; else v >>= (-shift);
            data_write(s, addr, (uint16_t)((v >> 16) & 0xFFFF));
            return consumed;
        }
        /* 89xx: ST src, Smem with shift or MVDK variants */
        if (hi8 == 0x89) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed;
        }
        /* 8Bxx: MVDK with long address */
        if (hi8 == 0x8B) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed;
        }
        /* 8Dxx: MVDD Smem, Smem */
        if (hi8 == 0x8D) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, addr));
            return consumed;
        }
        /* 83xx: WRITA Smem (write A to prog), 84xx: READA Smem */
        if (hi8 == 0x83) {
            addr = resolve_smem(s, op, &ind);
            prog_write(s, (uint16_t)(s->a & 0xFFFF), data_read(s, addr));
            return consumed;
        }
        if (hi8 == 0x84) {
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, prog_read(s, (uint16_t)(s->a & 0xFFFF)));
            return consumed;
        }
        /* 91xx: MVKD dmad, Smem (another encoding) */
        if (hi8 == 0x91) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, data_read(s, op2));
            return consumed;
        }
        /* 95xx: ST #lk, Smem (another encoding) */
        if (hi8 == 0x95) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed;
        }
        /* ST #lk, Smem (2-word) */
        if (hi8 == 0x96 || hi8 == 0x97) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, addr, op2);
            return consumed;
        }
        goto unimpl;

    case 0xA: case 0xB:
        /* Axx/Bxx: STLM, LDMM, misc accumulator ops */
        if (hi8 == 0xAA) {
            /* STLM src, MMR */
            int src_acc = (op >> 4) & 1;
            uint16_t mmr = op & 0x1F;
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, mmr, (uint16_t)(acc & 0xFFFF));
            return consumed;
        }
        if (hi8 == 0xBD) {
            /* BDxx: POPM / delayed branch variants */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst */
            uint16_t mmr = op & 0x1F;
            int dst = (op >> 4) & 1;
            int64_t v = (int64_t)(int16_t)data_read(s, mmr);
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed;
        }
        if (hi8 == 0xA8 || hi8 == 0xA9) {
            /* A8xx/A9xx: AND #lk, src[, dst] (2-word) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t *acc = dst ? &s->b : &s->a;
            *acc = sext40(*acc & ((int64_t)op2 << 16));
            return consumed;
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
            return consumed;
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
            return consumed;
        }
        if (hi8 == 0xB3) {
            /* LD #lk, dst (long immediate, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            int dst = (op >> 0) & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
            return consumed;
        }
        /* ADD #lk, src[, dst] */
        if (hi8 == 0xA2) {
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b + (v << 16));
            else     s->a = sext40(s->a + (v << 16));
            return consumed;
        }
        /* SUB #lk */
        if (hi8 == 0xA3) {
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            int dst = op & 1;
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)op2 : op2;
            if (dst) s->b = sext40(s->b - (v << 16));
            else     s->a = sext40(s->a - (v << 16));
            return consumed;
        }
        goto unimpl;

    case 0xC: case 0xD:
        /* C/Dxxx: PSHM, POPM, PSHD, POPD, RPT, FRAME, etc. */
        if (hi8 == 0xC5) {
            /* PSHM MMR */
            uint16_t mmr = op & 0x1F;
            s->sp--;
            data_write(s, s->sp, data_read(s, mmr));
            return consumed;
        }
        if (hi8 == 0xCD) {
            /* POPM MMR */
            uint16_t mmr = op & 0x1F;
            data_write(s, mmr, data_read(s, s->sp));
            s->sp++;
            return consumed;
        }
        if (hi8 == 0xCE) {
            /* FRAME #k (signed 8-bit) */
            int8_t k = (int8_t)(op & 0xFF);
            s->sp += k;
            return consumed;
        }
        if (hi8 == 0xC4) {
            /* C4xx: PSHD dmad (push data from absolute addr) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed;
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
                s->rpt_pc = (uint16_t)(s->pc + consumed);
                s->rpt_active = true;
            }
            return consumed;
        }
        if (hi8 == 0xCC) {
            /* PSHD dmad (2-word) or other CC variants */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, data_read(s, op2));
            return consumed;
        }
        if (hi8 == 0xDA) {
            /* DAxx: RPTBD pmad (block repeat delayed, 2 words) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4); /* delayed: skip 2 delay slots */
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            return consumed;
        }
        if (hi8 == 0xDD) {
            /* POPD Smem */
            addr = resolve_smem(s, op, &ind);
            data_write(s, addr, data_read(s, s->sp));
            s->sp++;
            return consumed;
        }
        if (hi8 == 0xDE) {
            /* DExx: POPD dmad (2-word) */
            op2 = prog_read(s, s->pc + 1);
            consumed = 2;
            data_write(s, op2, data_read(s, s->sp));
            s->sp++;
            return consumed;
        }
        goto unimpl;

    default:
        break;
    }

unimpl:
    s->unimpl_count++;
    if (s->unimpl_count <= 20 || op != s->last_unimpl) {
        C54_LOG("UNIMPL @0x%04x: 0x%04x (hi8=0x%02x) [#%u]",
                s->pc, op, hi8, s->unimpl_count);
        s->last_unimpl = op;
    }
    return consumed;
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

        /* Sample PC every 1M instructions to find stuck loops */
        if (executed > 0 && (executed % 1000000) == 0) {
            static int sample_log = 0;
            if (sample_log < 20)
                C54_LOG("@%dM: PC=0x%04x op=0x%04x SP=0x%04x insn=%u",
                        executed/1000000, s->pc, prog_read(s, s->pc), s->sp, s->insn_count);
            sample_log++;
        }
        if (0 && executed < 30) {
            C54_LOG("HDL[%d] PC=0x%04x op=0x%04x SP=0x%04x A=0x%08x%04x",
                    executed, s->pc, prog_read(s, s->pc), s->sp,
                    (uint32_t)(s->a >> 16), (uint16_t)(s->a & 0xFFFF));
        }
        /* Check RPTB (block repeat) */
        if (s->rptb_active && s->pc == s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 3) { C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d", s->pc, s->rsa, s->rea, s->brc); rptb_log++; }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false;
                s->st1 &= ~ST1_BRAF;
            }
        }

        /* Execute instruction */
        int consumed;
        if (s->rpt_active) {
            /* RPT: repeat single instruction */
            consumed = c54x_exec_one(s);
            if (s->rpt_count > 0) {
                s->rpt_count--;
                /* Don't advance PC — re-execute same instruction */
                s->cycles++;
                executed++;
                continue;
            } else {
                s->rpt_active = false;
            }
        } else {
            consumed = c54x_exec_one(s);
        }

        if (consumed > 0)
            s->pc += consumed;
        s->pc &= 0xFFFF;  /* C54x has 16-bit PC (23-bit with XPC, but wrap at 16-bit) */
        /* consumed == 0 means PC was set by branch */

        /* Timer0 tick — real TMS320C54x hardware behavior:
         * 1. If TSS=1 (timer stopped), do nothing
         * 2. PSC decrements each CPU clock cycle
         * 3. When PSC reaches 0: reload PSC from TDDR, decrement TIM
         * 4. When TIM reaches 0: reload TIM from PRD, set IFR bit 5 (TINT0)
         */
        if (!(s->data[TCR_ADDR] & TCR_TSS)) {
            if (s->timer_psc > 0) {
                s->timer_psc--;
            } else {
                /* PSC expired — reload from TDDR */
                s->timer_psc = s->data[TCR_ADDR] & TCR_TDDR_MASK;
                /* Decrement TIM */
                if (s->data[TIM_ADDR] > 0) {
                    s->data[TIM_ADDR]--;
                }
                if (s->data[TIM_ADDR] == 0 && s->data[PRD_ADDR] > 0) {
                    /* TIM expired — reload from PRD, fire TINT0 */
                    s->data[TIM_ADDR] = s->data[PRD_ADDR];
                    s->ifr |= (1 << 4); /* TINT0 = IMR bit 4, vec 20 */

                    /* Increment GSM frame number in DARAM (DSP-side FN counter) */
                    uint16_t fn_lo = s->data[0x0585];
                    uint16_t fn_hi = s->data[0x0584] & 0x00FF; /* upper bits of FN */
                    uint32_t fn = ((uint32_t)fn_hi << 16) | fn_lo;
                    fn = (fn + 1) % 2715648; /* GSM hyperframe */
                    s->data[0x0585] = fn & 0xFFFF;
                    s->data[0x0584] = (s->data[0x0584] & 0xFF00) | ((fn >> 16) & 0xFF);
                }
            }
        }

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
                /* Mirror PROM1 (page 1: 0x18000-0x1FFFF) to 16-bit space 0x8000-0xFFFF.
                 * PROM1 contains the interrupt vectors at 0xFF80.
                 * Don't mirror PROM2/PROM3 — they overlap and would overwrite. */
                if (section == 4) {  /* PROM1 only */
                    uint16_t addr16 = addr & 0xFFFF;
                    if (addr16 >= 0x8000)
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
    s->a = 0; s->b = 0;
    memset(s->ar, 0, sizeof(s->ar));
    s->t = 0; s->trn = 0;
    s->sp = 0; s->bk = 0;
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
    s->rptb_active = false;
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

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

    /* If not masked and interrupts enabled, take it */
    if (!(s->st1 & ST1_INTM) && (s->imr & (1 << imr_bit))) {
        s->ifr &= ~(1 << imr_bit);

        /* Push return address: PC+1 if waking from IDLE (resume after IDLE),
         * PC if interrupting normal execution (resume at interrupted insn) */
        uint16_t ret_addr = s->idle ? (uint16_t)(s->pc + 1) : (uint16_t)s->pc;
        s->sp--;
        data_write(s, s->sp, ret_addr);
        s->st1 |= ST1_INTM;

        /* Jump to vector */
        uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        s->pc = (iptr * 0x80) + vec * 4;

        /* Wake from IDLE */
        s->idle = false;
    } else if (s->idle && (s->imr & (1 << imr_bit))) {
        /* IDLE wakes on any unmasked interrupt even if INTM=1.
         * Per TMS320C54x spec: IDLE is special — when an unmasked interrupt
         * arrives during IDLE, the CPU always services it (branches to vector)
         * regardless of INTM state. INTM is set to 1 on entry. */
        s->idle = false;
        s->ifr &= ~(1 << imr_bit);
        s->sp--;
        data_write(s, s->sp, (uint16_t)(s->pc + 1)); /* return AFTER IDLE */
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
