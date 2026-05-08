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
        /* Indirect addressing.
         * Per SPRU131G §5.4.1 Table 5-5: bits 2:0 = ARF select the AR for
         * THIS instruction. ARP (in ST0) is then updated to ARF for the
         * NEXT direct-Smem reference. Earlier this code used arp(s) for
         * cur_arp, which made every indirect insn operate on the
         * PREVIOUS insn's ARF — off-by-one. Symptoms: BANZD *AR1- after
         * STL *AR2+ would decrement AR2 instead of AR1 (BANZD test
         * against AR2 stayed non-zero forever, AR1 frozen). Diagnosed
         * via 5×500M-insn STATE-DUMP showing AR1=0x1c / AR2=0x2b0c
         * frozen across 2B insns at PC=0xa2c2..0xa2ca. */
        *indirect = true;
        int mod = (opcode >> 3) & 0x0F;
        int nar = opcode & 0x07;
        int cur_arp = nar;
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
        /* Indirect modes 12..15 use a long-immediate operand from the next
         * program word. Encoding per tic54x-dis.c (MOD field = bits 6:3 of
         * the smem byte) and SPRU131G Table 5-9:
         *   12 : *AR(x)(lk)        — addr = AR(x) + lk, NO modify
         *   13 : *+AR(x)(lk)       — premod: AR(x) += lk; addr = AR(x)
         *   14 : *+AR(x)(lk)%      — premod circular: AR(x) = circ(AR(x)+lk)
         *   15 : *(lk)             — ABSOLUTE long address (lk itself)
         *
         * The bootloader at PROM0 0xb429 uses MOD=15 (`LDU *(0x0ffe), A`)
         * to read BL_ADDR_LO. Misdecoding 15 as "AR + lk circular"
         * produced AR0+0x0ffe instead of 0x0ffe — one of the multiple
         * subtle off-by-AR bugs that left A=0 after the load. */
        case 0xC: /* *AR(x)(lk) */
            addr = s->ar[cur_arp] + prog_fetch(s, s->pc + 1);
            s->lk_used = true;
            break;
