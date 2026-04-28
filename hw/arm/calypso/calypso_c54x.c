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

/* Forward decl: used by data_write() VECDUMP at MMR_PMST. */
static uint16_t prog_read(C54xState *s, uint32_t addr);

static uint16_t data_read(C54xState *s, uint16_t addr)
{
    /* Watch the mailbox slots that the firmware polls at PROM0 0xb41a
     * (LDU *(0x0ffe), A then BACC A) and 0xb41c (CMPM *(0x0fff), 4).
     * If these stay zero / 0x10 forever, ARM never wrote them. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x0ffc || addr == 0x0ffd) {
        static unsigned watch_count;
        watch_count++;
        if (watch_count <= 60 || (watch_count % 10000) == 0) {
            uint16_t vd = s->data[addr];
            uint16_t va = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0xDEAD;
            fprintf(stderr,
                    "[c54x] WATCH-READ #%u data[0x%04x] data=0x%04x api_ram=0x%04x api_set=%d PC=0x%04x insn=%u\n",
                    watch_count, addr, vd, va, s->api_ram ? 1 : 0, s->pc, s->insn_count);
        }
    }
    /* Wait-loop diagnostic: 0x3dd0 was found to absorb ~99.5 % of DARAM
     * reads after the first ~500k reads — the DSP is stuck polling it.
     * Log the first PCs and then sample once per million reads so we can
     * trace the loop without flooding the log. */
    if (addr == 0x3dd0) {
        static unsigned wait_log;
        static unsigned wait_seen;
        wait_seen++;
        if (wait_log < 20 || (wait_seen % 1000000) == 0) {
            wait_log++;
            fprintf(stderr,
                    "[c54x] WAIT-3DD0 #%u data[0x3dd0]=0x%04x PC=0x%04x AR2=%04x AR3=%04x insn=%u\n",
                    wait_seen, s->data[0x3dd0], s->pc,
                    s->ar[2], s->ar[3], s->insn_count);
        }
    }
    /* d_fb_det watch — REAL DSP word address is 0x08F8.
     * Mapping: ARM 0xFFD001F0 (BASE_API_NDB 0xFFD001A8 + 36 words × 2)
     *        = DSP word 0x0800 + 0x1F0/2 = 0x08F8.
     * Earlier 0x01F0 was the ARM byte-offset, NOT a DSP word address —
     * watching it logged unrelated DARAM 0x01F0 (junk). Now we trace
     * the real slot the firmware polls. */
    if (addr == 0x08F8) {
        static unsigned fb_read;
        if (fb_read++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-READ d_fb_det[0x08F8]=0x%04x PC=0x%04x insn=%u\n",
                    s->data[0x08F8], s->pc, s->insn_count);
        }
    }
    /* === DARAM discovery histogram ===
     * Track ALL data reads from DARAM (addr < 0x4000) regardless of PC.
     * The FB handler runs from both PROM0 (0xBD47) and DARAM overlay,
     * so filtering by PC misses critical reads. */
    if (addr < 0x4000 && addr >= 0x20) {  /* skip MMRs 0x00-0x1F */
        static unsigned hist[0x4000]; /* 16 KW DARAM */
        static unsigned reads;
        if (addr < 0x4000) {
            hist[addr]++;
            reads++;
            if ((reads % 50000) == 0) {
                /* find top-16 */
                unsigned best[16] = {0}; uint16_t baddr[16] = {0};
                for (uint16_t a = 0; a < 0x4000; a++) {
                    unsigned c = hist[a];
                    if (c <= best[15]) continue;
                    int p = 15;
                    while (p > 0 && best[p-1] < c) {
                        best[p] = best[p-1]; baddr[p] = baddr[p-1]; p--;
                    }
                    best[p] = c; baddr[p] = a;
                }
                fprintf(stderr,
                        "[c54x] DARAM RD HIST (FB-det, reads=%u): ",
                        reads);
                for (int i = 0; i < 16 && best[i]; i++)
                    fprintf(stderr, "%04x:%u ", baddr[i], best[i]);
                fprintf(stderr, "\n");
            }
        }
    }
    /* === BSP discovery: trace data reads in FB-det handler ===
     * Wide range over the PROM0 user-code area: handler PCs observed in
     * timeout traces cluster around 0x7e92..0x7eb8 (the FB-det inner
     * loop), so we widen the catch zone to 0x7000..0x7fff. */
    /* FB-det / dispatcher subroutine trace.
     * The 0x7e80..0x7eb8 wrapper CALLS into 0x81a5/0x81c8 with AR5=0x0e4c
     * (the FB sample buffer). Cover both ranges to catch both wrapper
     * polls and inner correlator reads. Skip the boot init phase. */
    if ((s->pc >= 0x7e80 && s->pc <= 0x7ec0) ||
        (s->pc >= 0x81a0 && s->pc <= 0x82ff)) {
        static int fbdet_rd_log = 0;
        if (s->insn_count > 50000000 && fbdet_rd_log < 2000) {
            uint16_t v;
            if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
                v = s->api_ram ? s->api_ram[addr - C54X_API_BASE] : 0;
            else
                v = s->data[addr];
            C54_LOG("FBDET RD [0x%04x]=0x%04x PC=0x%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                    addr, v, s->pc, s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            fbdet_rd_log++;
        }
    }
    /* Log AR0..AR7 when entering FB-det subroutines to understand
     * what each AR points at (sample buffer? coeffs? status?). */
    if ((s->pc == 0x81a5 || s->pc == 0x81c8) && s->insn_count > 50000000) {
        static int ar_log = 0;
        if (ar_log < 10) {
            C54_LOG("FB-CALL PC=0x%04x AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x BK=%04x",
                    s->pc, s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp, s->bk);
            ar_log++;
        }
    }
    /* d_spcx_rif (NDB word 2 = api 0xD6 = DSP data 0x08D6) */
    if (addr == 0x08D6) {
        static int spcx_rd = 0;
        if (spcx_rd < 32) {
            C54_LOG("d_spcx_rif RD = 0x%04x PC=0x%04x insn=%u",
                    s->api_ram ? s->api_ram[0xD6] : s->data[addr],
                    s->pc, s->insn_count);
            spcx_rd++;
        }
    }
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
                /* bit 5 = BRINT0 per C54X header (vec 21). */
                C54_LOG("IFR READ=0x%04x (BRINT0 pending) PC=0x%04x", s->ifr, s->pc);
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
    /* WATCH-WRITE on the same mailbox slots tracked in data_read.
     * Whoever writes them — DSP or ARM via api_ram alias — gets logged
     * so we can attribute the source of the value the firmware polls. */
    if (addr == 0x0ffe || addr == 0x0fff || addr == 0x01F0) {
        static unsigned wcount;
        if (wcount++ < 30) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE data[0x%04x] <- 0x%04x  (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher pointer at data[0x3f65] — `LD *(0x3f65),A; CALA A` at
     * DARAM 0x008a-0x008c. When this slot holds 0xfff8/0x0000/garbage the
     * CALA jumps into PROM1 vec or boot stub NOPs and the SP runs away.
     * Trace every write so we can identify who populates / corrupts it. */
    if (addr == 0x3f65) {
        static unsigned dpw;
        if (dpw++ < 100) {
            fprintf(stderr,
                    "[c54x] DISP-PTR data[0x3f65] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dispatcher poll addresses — log ANY write so we identify the
     * code path that should populate them. Currently 0 PORTR PA=0xF430
     * fires because dispatcher reads 0 here forever. */
    if (addr == 0x4359 || addr == 0x3fab) {
        static unsigned dispw;
        if (dispw++ < 50) {
            fprintf(stderr,
                    "[c54x] DISP-WRITE data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* CALAD source zone 0x4180-0x41FF — LD-A-TRACE shows the firmware
     * reads 0x4189 (DP=0x83) but our emulation has it as 0. Log every
     * write to this range so we can tell whether (a) anyone is meant to
     * populate it and we missed the path, or (b) DP=0x83 is itself a
     * symptom upstream of an unrelated bug. */
    if (addr >= 0x4180 && addr <= 0x41FF) {
        static unsigned cwz;
        if (cwz++ < 5000) {
            fprintf(stderr,
                    "[c54x] CALAD-ZONE-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc, s->insn_count);
        }
    }
    /* Dedicated watch on 0x4189 — never capped. The LD-A loop reads this
     * slot in the CALAD trap; we want to know if/when *anyone* finally
     * writes a non-zero value, and from which PC. */
    if (addr == 0x4189) {
        fprintf(stderr,
                "[c54x] *** WR-0x4189 *** data[0x4189] <- 0x%04x (was 0x%04x) PC=0x%04x insn=%u\n",
                val, s->data[addr], s->pc, s->insn_count);
    }
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
        case MMR_SP:
            if (val >= 0x0800 && val < 0x0900) {
                fprintf(stderr,
                        "[c54x] SP-GUARD: refused MMR_SP write 0x%04x "
                        "(API mailbox); keeping 0x%04x PC=0x%04x\n",
                        val, s->sp, s->pc);
                return;
            }
            s->sp = val;
            return;
        case MMR_BK:   s->bk = val; return;
        case MMR_BRC:  s->brc = val; return;
        case MMR_RSA:  s->rsa = val; return;
        case MMR_REA:  s->rea = val; return;
        case MMR_PMST:
            {
                static unsigned pmst_wr_attempts = 0;
                if (pmst_wr_attempts++ < 100)
                    C54_LOG("PMST WR attempt #%u: val=0x%04x cur=0x%04x PC=0x%04x insn=%u",
                            pmst_wr_attempts, val, s->pmst, s->pc, s->insn_count);
            }
            if (val != s->pmst) {
                uint16_t old_iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint16_t new_iptr = (val >> PMST_IPTR_SHIFT) & 0x1FF;
                C54_LOG("PMST change 0x%04x → 0x%04x (IPTR=0x%03x→0x%03x OVLY=%d) PC=0x%04x SP=0x%04x insn=%u",
                        s->pmst, val, old_iptr, new_iptr, !!(val & PMST_OVLY), s->pc, s->sp, s->insn_count);

                static uint16_t last_dumped_iptr = 0xFFFF;
                if (new_iptr != last_dumped_iptr) {
                    last_dumped_iptr = new_iptr;
                    uint32_t base = (uint32_t)new_iptr << 7;
                    uint16_t saved_pmst = s->pmst;
                    s->pmst = val;
                    C54_LOG("VECDUMP IPTR=0x%03x base=0x%04x (32 vectors):",
                            new_iptr, (uint16_t)base);
                    for (int vec = 0; vec < 32; vec++) {
                        uint32_t a = base + vec * 4;
                        uint16_t w0 = prog_read(s, a + 0);
                        uint16_t w1 = prog_read(s, a + 1);
                        uint16_t w2 = prog_read(s, a + 2);
                        uint16_t w3 = prog_read(s, a + 3);
                        fprintf(stderr,
                                "[c54x] vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                                vec, (uint16_t)a, w0, w1, w2, w3);
                    }
                    s->pmst = saved_pmst;
                }
            }
            s->pmst = val; return;
        case MMR_XPC:
            {
                static int xpc_log = 0;
                if (xpc_log++ < 50)
                    C54_LOG("MMR_XPC WR val=0x%04x (was %d) PC=0x%04x SP=0x%04x insn=%u",
                            val, s->xpc, s->pc, s->sp, s->insn_count);
            }
            s->xpc = val & 3;
            return;
        default: return;
        }
    }

    /* DMA sub-register bank (C54x DMA controller).
     * DMSA (0x0054): sets the sub-register address.
     * DMSDI (0x0055): writes sub-register data, auto-increments DMSA.
     * DMSDN (0x0057): writes sub-register data, no auto-increment.
     * DMA channel 0 sub-registers (BSP receive DMA):
     *   sub 0x00=DMSRC0, 0x01=DMDST0, 0x02=DMCTR0, 0x03=DMMCR0 */
    if (addr == 0x0054) {
        s->dma_subaddr = val;
        s->data[0x0054] = val;
        return;
    }
    if (addr == 0x0055 || addr == 0x0057) {
        uint16_t sa = s->dma_subaddr;
        if (sa < 24) {  /* 6 channels × 4 regs */
            s->dma_subregs[sa] = val;
            int ch = sa / 4;
            int reg = sa % 4;
            static const char *rnames[] = {"SRC","DST","CTR","MCR"};
            C54_LOG("DMA ch%d %s = 0x%04x (sub 0x%02x) PC=0x%04x",
                    ch, rnames[reg], val, sa, s->pc);
        }
        s->data[addr] = val;
        if (addr == 0x0055) s->dma_subaddr++;  /* auto-increment */
        return;
    }

    /* McBSP sub-register bank (serial port extended config).
     * SPSA (0x0038): sub-address. SPSD (0x0039): sub-data. */
    if (addr == 0x0038 || addr == 0x0039) {
        if (addr == 0x0038) s->spsa = val;
        else {
            C54_LOG("McBSP sub[0x%02x] = 0x%04x PC=0x%04x", s->spsa, val, s->pc);
        }
        s->data[addr] = val;
        return;
    }

    /* API RAM (shared with ARM) */
    if (addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE) {
        uint16_t woff = addr - C54X_API_BASE;
        if (s->api_ram)
            s->api_ram[woff] = val;
        /* Notify the ARM-side mailbox watcher (calypso_trx) so it can
         * pulse IRQ_API, mirror to dsp_ram, and run the d_fb_det hook.
         * Without this, DSP writes to NDB cells are invisible to ARM. */
        if (s->api_write_cb)
            s->api_write_cb(s->api_write_cb_opaque, woff, val);
        /* Stack-corruption watch: stack push landing in the NDB
         * mailbox region [0x0800..0x08FF]. Only fires when SP has
         * already been corrupted into that range. */
        if (addr == s->sp && addr >= 0x0800 && addr < 0x0900) {
            fprintf(stderr,
                    "[c54x] STACK-IN-NDB addr=0x%04x val=0x%04x SP=0x%04x "
                    "PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x\n",
                    addr, val, s->sp, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* Always log writes to d_dsp_page (0x08D4) */
        if (addr == 0x08D4) {
            C54_LOG("DSP WR d_dsp_page = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_spcx_rif (NDB word 2 = DSP data 0x08D6) — BSP serial port config */
        if (addr == 0x08D6) {
            C54_LOG("DSP WR d_spcx_rif = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x",
                    val, s->pc, s->insn_count,
                    s->prog[(uint16_t)(s->pc - 2)],
                    s->prog[(uint16_t)(s->pc - 1)],
                    s->prog[s->pc],
                    s->prog[(uint16_t)(s->pc + 1)]);
        }
        /* d_fb_det (NDB word 36 = DSP data 0x08F8). Real firmware writes
         * a small unsigned value here (BSIC byte, status). Spurious writes
         * caused by parallel/indirect store side-effects in nearby code
         * can blast garbage like 0xC000 — only treat 1..0xFF as a real
         * FB-detected event so we don't get false positives. */
        if (addr == 0x08F8) {
            static int fbd_log = 0;
            bool plausible = (val >= 1 && val <= 0xFF);
            if (plausible || fbd_log < 5) {
                C54_LOG("DSP WR d_fb_det = 0x%04x PC=0x%04x insn=%u op[pc-2..pc+1]=%04x %04x %04x %04x %s",
                        val, s->pc, s->insn_count,
                        s->prog[(uint16_t)(s->pc - 2)],
                        s->prog[(uint16_t)(s->pc - 1)],
                        s->prog[s->pc],
                        s->prog[(uint16_t)(s->pc + 1)],
                        plausible ? "*** FB DETECTED ***" :
                        (val == 0 ? "(clear)" : "(spurious, ignored)"));
                fbd_log++;
            }
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
        case 0xD: /* *+AR(x)(lk) */
            s->ar[cur_arp] += prog_fetch(s, s->pc + 1);
            addr = s->ar[cur_arp];
            s->lk_used = true;
            break;
        case 0xE: { /* *+AR(x)(lk)% — circular */
            uint16_t lk = prog_fetch(s, s->pc + 1);
            uint16_t v  = s->ar[cur_arp] + lk;
            if (s->bk) {
                uint16_t base = s->ar[cur_arp] - (s->ar[cur_arp] % s->bk);
                if (v >= base + s->bk) v -= s->bk;
            }
            s->ar[cur_arp] = v;
            addr = v;
            s->lk_used = true;
            break;
        }
        case 0xF: /* *(lk) — absolute address */
            addr = prog_fetch(s, s->pc + 1);
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
        /* 0x7700 entry tracer: log when PC enters 0x7700 from elsewhere
         * (i.e. prev_pc != 0x76FF, the natural sequential predecessor).
         * Reveals which CALL/B/RET sources land here. PC HIST shows
         * 7700/7701 as the hottest non-loop addresses — find the callers. */
        if (s->pc == 0x7700 && prev_pc != 0x76FF) {
            static uint64_t e7700;
            e7700++;
            if (e7700 <= 30 || (e7700 % 5000) == 0) {
                C54_LOG("ENTER-7700 #%llu from PC=0x%04x A=%010llx B=%010llx SP=0x%04x trail: %04x %04x %04x %04x %04x",
                        (unsigned long long)e7700, prev_pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        pc_ring[(pc_ring_idx-5)&255], pc_ring[(pc_ring_idx-4)&255],
                        pc_ring[(pc_ring_idx-3)&255], pc_ring[(pc_ring_idx-2)&255],
                        pc_ring[(pc_ring_idx-1)&255]);
            }
        }
        /* MAC-7700 tracer: at PC=0x7700 (MAC *AR2-, A) we want to know
         * what AR2 points at, what data[AR2] holds, T, and A before/after.
         * Helps determine if AR2 references the BSP RX zone (correlator
         * FB-det) or somewhere else. Also dumps full AR0-AR7 + ST0/ST1. */
        if (s->pc == 0x7700) {
            static uint64_t mac7700_total;
            mac7700_total++;
            if (mac7700_total <= 50 || (mac7700_total % 5000) == 0) {
                uint16_t ar2 = s->ar[2];
                uint16_t v_at_ar2 = s->data[ar2];
                C54_LOG("MAC-7700 #%llu AR2=0x%04x data[AR2]=0x%04x T=0x%04x "
                        "A_pre=%010llx ST0=0x%04x ST1=0x%04x",
                        (unsigned long long)mac7700_total, ar2, v_at_ar2,
                        s->t,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        s->st0, s->st1);
                C54_LOG("MAC-7700 #%llu ARs: AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                        "AR4=%04x AR5=%04x AR6=%04x AR7=%04x SP=%04x",
                        (unsigned long long)mac7700_total,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7], s->sp);
            }
        }
        /* RCD-75e8 tracer: when DSP arrives at PC=0x75e8 (cond=0x47 = LEQ),
         * log A. The RCD takes if A <= 0; report whether the loop will
         * exit this iteration. */
        if (s->pc == 0x75e8) {
            static uint64_t rcd75e8_total;
            rcd75e8_total++;
            if (rcd75e8_total <= 50 || (rcd75e8_total % 5000) == 0) {
                int64_t acc = sext40(s->a);
                C54_LOG("RCD-75e8 #%llu A=%010llx (signed=%lld) RCD-taken=%d AR2=%04x",
                        (unsigned long long)rcd75e8_total,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (long long)acc, (acc <= 0), s->ar[2]);
            }
        }
        prev_pc = s->pc;
        /* DARAM 0x1100-0x1130 tracer: dump first 64 visits */
        static int daram1110_log = 0;
        if (s->pc >= 0x1100 && s->pc <= 0x1130 && daram1110_log < 64) {
            C54_LOG("DARAM110x PC=0x%04x op=0x%04x A=%08x B=%08x AR2=%04x AR3=%04x AR4=%04x AR5=%04x BRC=%d",
                    s->pc, op, (uint32_t)s->a, (uint32_t)s->b,
                    s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->brc);
            daram1110_log++;
        }
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
        /* F4E2 = BACC A, F5E2 = BACC B (per tic54x-opc.c, mask 0xFEFF) */
        /* F4E3 = CALA A, F5E3 = CALA B — push next-PC, jump to acc low 16 bits */
        /* DYN-CALL tracer: targets are computed at runtime, invisible to static
         * disasm. Log every BACC/CALA, plus an extra hot tag when the target
         * lands in any FB-det zone (PROM0 0x77xx-0x79xx, 0x88xx, 0xa0xx-0xa1xx). */
        if (op == 0xF4E2 || op == 0xF5E2 || op == 0xF4E3 || op == 0xF5E3) {
            int is_b = (op & 0x0100) != 0;
            int is_call = (op & 1) != 0;
            uint16_t tgt = (uint16_t)((is_b ? s->b : s->a) & 0xFFFF);
            uint16_t src_pc = s->pc;
            int fb_zone = (tgt >= 0x7730 && tgt <= 0x7990) ||
                          (tgt >= 0x8800 && tgt <= 0x88FF) ||
                          (tgt >= 0xA000 && tgt <= 0xA1FF);
            static uint64_t dyn_total = 0;
            static uint64_t dyn_fb = 0;
            dyn_total++;
            if (fb_zone) dyn_fb++;
            /* When OVLY=1 and src_pc in [0x80, 0x2800], the executed opcode
             * comes from data[] (DARAM), not prog[]. Reflect this in the
             * dump so we see the *actual* bytes that drove the CALA. */
            int ovly_active = (s->pmst & PMST_OVLY) && src_pc >= 0x80 && src_pc < 0x2800;
            uint16_t m0 = ovly_active ? s->data[(uint16_t)(src_pc - 2)] : s->prog[(uint16_t)(src_pc - 2)];
            uint16_t m1 = ovly_active ? s->data[(uint16_t)(src_pc - 1)] : s->prog[(uint16_t)(src_pc - 1)];
            uint16_t m2 = ovly_active ? s->data[src_pc] : s->prog[src_pc];
            uint16_t m3 = ovly_active ? s->data[(uint16_t)(src_pc + 1)] : s->prog[(uint16_t)(src_pc + 1)];
            if (dyn_total <= 200 || fb_zone || (dyn_total % 5000) == 0) {
                C54_LOG("DYN-CALL #%llu %s%c src=0x%04x tgt=0x%04x A=%010llx B=%010llx SP=0x%04x mem[%c]=%04x %04x %04x %04x%s",
                        (unsigned long long)dyn_total,
                        is_call ? "CALA" : "BACC",
                        is_b ? 'B' : 'A',
                        src_pc, tgt,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFULL),
                        s->sp,
                        ovly_active ? 'D' : 'P',
                        m0, m1, m2, m3,
                        fb_zone ? " *FB-ZONE*" : "");
            }
            if (is_call) {
                uint16_t ret_pc = src_pc + 1;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, ret_pc);
            }
            s->pc = tgt;
            return 0;
        }
        /* F4E0-F4FF: RSBX/SSBX status bits — treat as NOP (most don't affect emulation) */
        if (op >= 0xF4E0 && op <= 0xF4FF && op != 0xF4E4 && op != 0xF4EB) {
            return consumed + s->lk_used;
        }
        /* F4EB = RETE (return from interrupt). Pop PC, pop XPC iff APTS=1.
         * Symmetric with c54x_interrupt_ex push order. */
        if (op == 0xF4EB) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            if (s->pmst & PMST_APTS) {
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
            }
            s->st1 &= ~ST1_INTM;
            {
                static uint64_t rete_count;
                rete_count++;
                if (rete_count <= 20 || (rete_count % 100) == 0)
                    C54_LOG("RETE #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)rete_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra; return 0;
        }
        /* 0xF4E4 = FRET (far return). Pop PC + XPC unconditionally.
         * Per binutils tic54x-opc.c (FL_FAR flag) and SPRU172C Table 2-15:
         *   FRET[D]: XPC = TOS, ++SP, PC = TOS, ++SP
         * Symmetric with FCALL/FCALLD push (also unconditional, see below).
         * 2026-04-28 — fixed: was conditional on PMST_APTS (bit 4) which is
         * actually AVIS (Address Visibility) per SPRU131G — has no stack
         * semantics. The misnomer caused FRET to skip XPC pop when AVIS=0,
         * leading to stack imbalance against FCALL FAR which always pushes 2. */
        if (op == 0xF4E4) {
            uint16_t ra = data_read(s, s->sp); s->sp++;
            uint16_t prev_xpc = s->xpc;
            s->xpc = data_read(s, s->sp); s->sp++;
            if (s->xpc > 3) s->xpc &= 3;
            {
                static uint64_t fret_count;
                fret_count++;
                if (fret_count <= 30 || (fret_count % 1000) == 0)
                    C54_LOG("FRET #%llu PC=0x%04x -> ra=0x%04x XPC=%u→%u SP=0x%04x",
                            (unsigned long long)fret_count,
                            s->pc, ra, prev_xpc, s->xpc, s->sp);
            }
            s->pc = ra;
            return 0;
        }
        /* IDLE 1/2/3: 0xF4E1, 0xF5E1, 0xF6E1, 0xF7E1 (mask 0xFCFF) */
        if ((op & 0xFCFF) == 0xF4E1) {
            int level = ((op >> 8) & 0x3) + 1;
            static int idle_log = 0;
            if (idle_log < 20)
                C54_LOG("IDLE%d @0x%04x INTM=%d IMR=0x%04x SP=0x%04x insns=%u XPC=%d",
                        level, s->pc, !!(s->st1 & ST1_INTM),
                        s->imr, s->sp, s->insn_count, s->xpc);
            idle_log++;
            if (s->pc >= 0x8000 && s->pc < 0x8020) {
                return consumed + s->lk_used;
            }
            s->idle = true;
            return 0;
        }
        /* ================================================================
         * F[4-7]xx generic accumulator family — promoted from F4 block
         * to handle F5/F6/F7 variants. Handlers use bits 8/9 for src/dst,
         * with masks FCE0/FCFF/FEFF naturally covering all 4 combinations
         * (A->A, B->A, A->B, B->B). The matching handler bodies remain
         * inside the F4 block as dead code (never reached for arith ops
         * because of the early return here). 2026-04-28.
         * ================================================================ */
            /* F483/F583: SAT src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF483) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                return consumed + s->lk_used;
            }

            /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF484) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                return consumed + s->lk_used;
            }

            /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF485) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                if (val < 0) val = -val;
                if (dst) s->b = sext40(val); else s->a = sext40(val);
                return consumed + s->lk_used;
            }

            /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
             * Multiply T * A(high), accumulate into dst */
            if ((op & 0xFEFF) == 0xF48C) {
                int dst = (op >> 8) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF48D) {
                int dst = (op >> 8) & 1;
                int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                int64_t prod = (int64_t)ah * (int64_t)ah;
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                return consumed + s->lk_used;
            }

            /* F48E/F58E: EXP src (mask FEFF, 1 word)
             * Count leading sign bits of accumulator, store in T */
            if ((op & 0xFEFF) == 0xF48E) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                int exp = 0;
                if (val == 0 || val == -1) { exp = 31; }
                else {
                    uint64_t uv = (val < 0) ? ~val : val;
                    uv &= 0xFFFFFFFFFFULL;
                    /* Count leading zeros from bit 38 down */
                    for (int i = 38; i >= 0; i--) {
                        if (uv & (1ULL << i)) break;
                        exp++;
                    }
                    exp -= 8; /* EXP = leading sign bits - 8 */
                }
                s->t = (uint16_t)(int16_t)exp;
                return consumed + s->lk_used;
            }

            /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
            if ((op & 0xFEFF) == 0xF492) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
            if ((op & 0xFEFF) == 0xF493) {
                int64_t sa = sext40(s->a), sb = sext40(s->b);
                if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                else { s->st0 &= ~ST0_C; }
                return consumed + s->lk_used;
            }

            /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
            if ((op & 0xFEFF) == 0xF49E) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val >= 0) { *acc = sext40((val << 1) + 1); }
                else { *acc = sext40(val << 1); }
                return consumed + s->lk_used;
            }

            /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
             * Per SPRU172C p.4-118: if the two MSBs of src accumulator
             * are different (not sign-extended), shift src left by 1
             * and decrement T. Otherwise do nothing. Used by the FB-det
             * correlator to normalize results; the loop exits when
             * NORM stops shifting (MSBs match = value is normalized). */
            if ((op & 0xFEFF) == 0xF48F) {
                int src = (op >> 8) & 1;
                int64_t val = sext40(src ? s->b : s->a);
                /* Check bits 39 and 38 — if they differ, shift left */
                int bit39 = (val >> 39) & 1;
                int bit38 = (val >> 38) & 1;
                if (bit39 != bit38) {
                    val = sext40(val << 1);
                    if (src) s->b = val; else s->a = val;
                    s->t = (uint16_t)(s->t - 1);
                }
                /* TC flag: set if shift occurred */
                if (bit39 != bit38)
                    s->st0 |= ST0_TC;
                else
                    s->st0 &= ~ST0_TC;
                return consumed + s->lk_used;
            }

            /* F490/F590: ROR src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF490) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1; /* carry */
                uint16_t lsb = *acc & 1;
                *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F491/F591: ROL src (mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF491) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                uint16_t c = (s->st0 >> 8) & 1;
                uint16_t msb = (*acc >> 39) & 1;
                *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                return consumed + s->lk_used;
            }

            /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF488) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                if (s->st1 & ST1_FRCT) prod <<= 1;
                if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                return consumed + s->lk_used;
            }

            /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF486) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                return consumed + s->lk_used;
            }

            /* F487/F587: RND src (round, mask FEFF, 1 word) */
            if ((op & 0xFEFF) == 0xF487) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                *acc = sext40(*acc + 0x8000);
                return consumed + s->lk_used;
            }

            /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF480) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF481) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
            if ((op & 0xFCFF) == 0xF482) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int64_t sv = sext40(src ? s->b : s->a);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            /* F4xx accumulator shift/load (1-word, mask FCE0):
             * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
            if ((op & 0xFCE0) == 0xF400) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF420) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF440) {
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF460) {
                /* SFTA src,shift,dst — arithmetic shift accumulator */
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                int64_t sv = sext40(src ? s->b : s->a);
                if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                return consumed + s->lk_used;
            }

            if ((op & 0xFCE0) == 0xF4A0) {
                /* SFTL src,shift,dst — logical shift accumulator */
                int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                int shift = op & 0x1F; if (shift > 15) shift -= 32;
                uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                uv &= 0xFFFFFFFFFFULL;
                if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                return consumed + s->lk_used;
            }

        /* F494/F594: SFTC src (mask FEFF, 1 word).
         * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
         * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
         * — without it the correlator sums never normalise. */
        if ((op & 0xFEFF) == 0xF494) {
            int src = (op >> 8) & 1;
            int64_t *acc = src ? &s->b : &s->a;
            int64_t val = sext40(*acc);
            if (val != 0) {
                int b31 = (val >> 31) & 1;
                int b30 = (val >> 30) & 1;
                if (b31 == b30) *acc = sext40(val << 1);
            }
            return consumed + s->lk_used;
        }

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

            /* F4xx arithmetic instructions (1-word, per tic54x-opc.c).
             * These MUST be checked before the 2-word branch/call switch. */
            {
                /* F483/F583: SAT src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF483) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val > 0x7FFFFFFFLL) *acc = sext40(0x7FFFFFFFLL);
                    else if (val < -0x80000000LL) *acc = sext40(-0x80000000LL);
                    return consumed + s->lk_used;
                }
                /* F484/F584: NEG src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF484) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(-val); else s->a = sext40(-val);
                    return consumed + s->lk_used;
                }
                /* F485/F585: ABS src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF485) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    if (val < 0) val = -val;
                    if (dst) s->b = sext40(val); else s->a = sext40(val);
                    return consumed + s->lk_used;
                }
                /* F48C/F58C: MPYA dst (mask FEFF, 1 word)
                 * Multiply T * A(high), accumulate into dst */
                if ((op & 0xFEFF) == 0xF48C) {
                    int dst = (op >> 8) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F48D/F58D: SQUR A,dst (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF48D) {
                    int dst = (op >> 8) & 1;
                    int16_t ah = (int16_t)((s->a >> 16) & 0xFFFF);
                    int64_t prod = (int64_t)ah * (int64_t)ah;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(prod); else s->a = sext40(prod);
                    return consumed + s->lk_used;
                }
                /* F48E/F58E: EXP src (mask FEFF, 1 word)
                 * Count leading sign bits of accumulator, store in T */
                if ((op & 0xFEFF) == 0xF48E) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    int exp = 0;
                    if (val == 0 || val == -1) { exp = 31; }
                    else {
                        uint64_t uv = (val < 0) ? ~val : val;
                        uv &= 0xFFFFFFFFFFULL;
                        /* Count leading zeros from bit 38 down */
                        for (int i = 38; i >= 0; i--) {
                            if (uv & (1ULL << i)) break;
                            exp++;
                        }
                        exp -= 8; /* EXP = leading sign bits - 8 */
                    }
                    s->t = (uint16_t)(int16_t)exp;
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM — handled below (real implementation, not NOP) */
                /* F492/F592: MAX src (mask FEFF, 1 word) — keep max of A,B */
                if ((op & 0xFEFF) == 0xF492) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa < sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F493/F593: MIN src (mask FEFF, 1 word) — keep min of A,B */
                if ((op & 0xFEFF) == 0xF493) {
                    int64_t sa = sext40(s->a), sb = sext40(s->b);
                    if (sa > sb) { s->a = s->b; s->st0 |= ST0_C; }
                    else { s->st0 &= ~ST0_C; }
                    return consumed + s->lk_used;
                }
                /* F49E/F59E: SUBC src (mask FEFF, 1 word) — conditional subtract for division */
                if ((op & 0xFEFF) == 0xF49E) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    int64_t val = sext40(*acc);
                    if (val >= 0) { *acc = sext40((val << 1) + 1); }
                    else { *acc = sext40(val << 1); }
                    return consumed + s->lk_used;
                }
                /* F48F/F58F: NORM src[, dst] (mask FEFF, 1 word)
                 * Per SPRU172C p.4-118: if the two MSBs of src accumulator
                 * are different (not sign-extended), shift src left by 1
                 * and decrement T. Otherwise do nothing. Used by the FB-det
                 * correlator to normalize results; the loop exits when
                 * NORM stops shifting (MSBs match = value is normalized). */
                if ((op & 0xFEFF) == 0xF48F) {
                    int src = (op >> 8) & 1;
                    int64_t val = sext40(src ? s->b : s->a);
                    /* Check bits 39 and 38 — if they differ, shift left */
                    int bit39 = (val >> 39) & 1;
                    int bit38 = (val >> 38) & 1;
                    if (bit39 != bit38) {
                        val = sext40(val << 1);
                        if (src) s->b = val; else s->a = val;
                        s->t = (uint16_t)(s->t - 1);
                    }
                    /* TC flag: set if shift occurred */
                    if (bit39 != bit38)
                        s->st0 |= ST0_TC;
                    else
                        s->st0 &= ~ST0_TC;
                    return consumed + s->lk_used;
                }
                /* F49F: DELAY (pipeline flush, NOP) */
                if (op == 0xF49F) { return consumed + s->lk_used; }
                /* F490/F590: ROR src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF490) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1; /* carry */
                    uint16_t lsb = *acc & 1;
                    *acc = sext40(((uint64_t)(*acc & 0xFFFFFFFFFFULL) >> 1) | ((uint64_t)c << 39));
                    if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F491/F591: ROL src (mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF491) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    uint16_t c = (s->st0 >> 8) & 1;
                    uint16_t msb = (*acc >> 39) & 1;
                    *acc = sext40(((*acc << 1) & 0xFFFFFFFFFFULL) | c);
                    if (msb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
                    return consumed + s->lk_used;
                }
                /* F488/F588: MACA T,src[,dst] (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF488) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((src ? s->b : s->a) >> 16);
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    if (dst) s->b = sext40(s->b + prod); else s->a = sext40(s->a + prod);
                    return consumed + s->lk_used;
                }
                /* F486/F586: CMPL src (complement, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF486) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(~(*acc) & 0xFFFFFFFFFFULL);
                    return consumed + s->lk_used;
                }
                /* F487/F587: RND src (round, mask FEFF, 1 word) */
                if ((op & 0xFEFF) == 0xF487) {
                    int src = (op >> 8) & 1;
                    int64_t *acc = src ? &s->b : &s->a;
                    *acc = sext40(*acc + 0x8000);
                    return consumed + s->lk_used;
                }
                /* F480/F580: ADD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF480) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                /* F481/F581: SUB src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF481) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                /* F482/F582: LD src,ASM,dst (mask FCFF, 1 word) */
                if ((op & 0xFCFF) == 0xF482) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                /* F4xx accumulator shift/load (1-word, mask FCE0):
                 * F400: ADD src,shift,dst  F420: SUB  F440: LD  F460: SFTA */
                if ((op & 0xFCE0) == 0xF400) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b + sv); else s->a = sext40(s->a + sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF420) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(s->b - sv); else s->a = sext40(s->a - sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF440) {
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF460) {
                    /* SFTA src,shift,dst — arithmetic shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    int64_t sv = sext40(src ? s->b : s->a);
                    if (shift >= 0) sv <<= shift; else sv >>= (-shift);
                    if (dst) s->b = sext40(sv); else s->a = sext40(sv);
                    return consumed + s->lk_used;
                }
                if ((op & 0xFCE0) == 0xF4A0) {
                    /* SFTL src,shift,dst — logical shift accumulator */
                    int src = (op >> 8) & 1, dst = (op >> 9) & 1;
                    int shift = op & 0x1F; if (shift > 15) shift -= 32;
                    uint64_t uv = (uint64_t)((src ? s->b : s->a) & 0xFFFFFFFFFFULL);
                    if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                    uv &= 0xFFFFFFFFFFULL;
                    if (dst) s->b = sext40(uv); else s->a = sext40(uv);
                    return consumed + s->lk_used;
                }
            }
                        /* F4Bx: RSBX -- reset bit in ST0 (bit 9=0, bit 8=0).
             * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF4B0) {
                int bit = op & 0x0F;
                s->st0 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* F494/F594: SFTC src (mask FEFF, 1 word).
             * Per SPRU172C p.4-264: shift src left by 1 if src(31)==src(30)
             * and src!=0. Used by FB-det normalisation around PC=0x10e5..0x10f4
             * — without it the correlator sums never normalise. */
            if ((op & 0xFEFF) == 0xF494) {
                int src = (op >> 8) & 1;
                int64_t *acc = src ? &s->b : &s->a;
                int64_t val = sext40(*acc);
                if (val != 0) {
                    int b31 = (val >> 31) & 1;
                    int b30 = (val >> 30) & 1;
                    if (b31 == b30) *acc = sext40(val << 1);
                }
                return consumed + s->lk_used;
            }
            /* Remaining F4xx: unhandled — treat as 1-word NOP */
            C54_LOG("F4xx unhandled: 0x%04x PC=0x%04x", op, s->pc);
            return consumed + s->lk_used;
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
            /* F073: B pmad — unconditional branch (2-word).
             * Per tic54x-opc.c: 0xF073 mask 0xFFFF. */
            if (op == 0xF073) {
                op2 = prog_fetch(s, s->pc + 1);
                s->pc = op2;
                return 0;
            }
            /* F074: CALL pmad — unconditional call (2-word).
             * Per tic54x-opc.c: call 0xF074 mask 0xFFFF.
             * Push PC+2 (return address), branch to pmad.
             * NOTE: RETE is 0xF4EB (already handled above), NOT F074. */
            if (op == 0xF074) {
                op2 = prog_fetch(s, s->pc + 1);
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                s->pc = op2;
                return 0;
            }







            /* F072: RPTB pmad — block repeat (2-word, non-delayed).
             * Per tic54x-opc.c: 0xF072 mask 0xFFFF.
             * RSA = PC+2, REA = pmad. */
            if (op == 0xF072) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            /* F07x: RPT/RPTZ/misc (F072-F074 handled above) */
            if (op == 0xF070) {
                /* F070: RPT #lku — repeat next instruction lku+1 times (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if (op == 0xF071) {
                /* F071: RPTZ dst, #lku — zero accumulator and repeat (2-word) */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int dst = (op >> 8) & 1; /* bit 8 via FEFF mask */
                if (dst) s->b = 0; else s->a = 0;
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            if ((op & 0xFFF0) == 0xF070) {
                /* F075-F07F: undefined, treat as 1-word NOP */
                return consumed + s->lk_used;
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
            /* F0xx/F1xx ALU with #lk immediate (2-word).
             * Per tic54x-opc.c: bits 7:4 = op (0=ADD,1=SUB,2=LD,3=AND,4=OR,5=XOR),
             * bit 8 = SRC (ADD/SUB/AND/OR/XOR) or DST (LD), bit 9 = DST,
             * bits 3:0 = shift. Second word = lk. */
            {
                uint8_t alu_op = (op >> 4) & 0xF;
                if (alu_op <= 5) {
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int shift = op & 0xF;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = (alu_op == 2)
                        ? (src_sel ? &s->b : &s->a)
                        : (dst_sel ? &s->b : &s->a);
                    int64_t lk_val;
                    if (alu_op <= 2)
                        lk_val = (int64_t)(int16_t)op2 << shift;
                    else
                        lk_val = (int64_t)(uint16_t)op2 << shift;
                    switch (alu_op) {
                    case 0: *dst = sext40(src_val + lk_val); break; /* ADD */
                    case 1: *dst = sext40(src_val - lk_val); break; /* SUB */
                    case 2: *dst = sext40(lk_val); break;           /* LD  */
                    case 3: *dst = src_val & lk_val; break;         /* AND */
                    case 4: *dst = src_val | lk_val; break;         /* OR  */
                    case 5: *dst = src_val ^ lk_val; break;         /* XOR */
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op == 6) {
                    /* F06x: ADD/SUB/LD/AND/OR/XOR #lk,16 + MPY/MAC #lk */
                    uint8_t sub6 = op & 0xF;
                    op2 = prog_fetch(s, s->pc + 1);
                    consumed = 2;
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t src_val = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    switch (sub6) {
                    case 0: *dst = sext40(src_val + ((int64_t)(int16_t)op2 << 16)); break;
                    case 1: *dst = sext40(src_val - ((int64_t)(int16_t)op2 << 16)); break;
                    case 2: dst = src_sel ? &s->b : &s->a;
                            *dst = sext40((int64_t)(int16_t)op2 << 16); break;
                    case 3: *dst = src_val & ((int64_t)(uint16_t)op2 << 16); break;
                    case 4: *dst = src_val | ((int64_t)(uint16_t)op2 << 16); break;
                    case 5: *dst = src_val ^ ((int64_t)(uint16_t)op2 << 16); break;
                    case 6: /* MPY #lk, dst */
                            dst = src_sel ? &s->b : &s->a;
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(p); } break;
                    case 7: /* MAC #lk, src[,dst] */
                            { int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                              if (s->st1 & ST1_FRCT) p <<= 1;
                              *dst = sext40(src_val + p); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
                if (alu_op >= 8) {
                    /* F08x-F0Fx: accumulator-to-accumulator ops (1-word).
                     * bits 7:5 = op (100=AND,101=OR,110=XOR,111=SFTL)
                     * bits 4:0 = shift (signed 5-bit), bits 9:8 = src,dst */
                    int src_sel = (op >> 8) & 1;
                    int dst_sel = (op >> 9) & 1;
                    int64_t sv = src_sel ? s->b : s->a;
                    int64_t *dst = dst_sel ? &s->b : &s->a;
                    int shift = op & 0x1F;
                    if (shift > 15) shift -= 32;
                    uint8_t aop = (op >> 5) & 0x7;
                    int64_t shifted;
                    if (shift >= 0) shifted = sv << shift;
                    else            shifted = sv >> (-shift);
                    switch (aop) {
                    case 4: *dst = sext40(sv) & sext40(shifted); break;
                    case 5: *dst = sext40(sv) | sext40(shifted); break;
                    case 6: *dst = sext40(sv) ^ sext40(shifted); break;
                    case 7: { uint64_t uv = (uint64_t)(sv & 0xFFFFFFFFFFULL);
                              if (shift >= 0) uv <<= shift; else uv >>= (-shift);
                              *dst = sext40(uv & 0xFFFFFFFFFFULL); } break;
                    default: break;
                    }
                    return consumed + s->lk_used;
                }
            }
            goto unimpl;
        }
        /* F272/F274/F273: RPTBD/CALLD/RETD — must check BEFORE LMS */
        if (op == 0xF272) {
            /* RPTBD pmad — delayed block repeat (2 words).
             * Delayed: 2 delay slots after the 2-word instruction.
             * RSA = PC + 4 (skip RPTBD + 2 delay slot words). */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->rea = op2;
            s->rsa = (uint16_t)(s->pc + 4);
            s->rptb_active = true;
            s->st1 |= ST1_BRAF;
            { static int _rb=0; if (_rb<20) { C54_LOG("RPTBD PC=0x%04x REA=0x%04x RSA=0x%04x BRC=%d", s->pc, s->rea, s->rsa, s->brc); _rb++; } }
            return consumed + s->lk_used;
        }
        if (op == 0xF274) {
            /* CALLD pmad — delayed call (2 words, 2 delay slots).
             * Push PC+4 (past CALLD + 2 delay slots), branch to pmad. */
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            s->sp--;
            data_write(s, s->sp, (uint16_t)(s->pc + 4));
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
         * Per SPRU172C: dst += T * Xmem; Ymem += rnd(AH * T); T = Xmem
         * Exclude F272 (RPTBD), F273 (RETD), F274 (CALLD) — exact-match
         * opcodes that share the F2xx range but are handled below. */
        if ((hi8 == 0xF2 || hi8 == 0xF3) &&
            op != 0xF272 && op != 0xF273 && op != 0xF274) {
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
        /* F8xx: branches, RPT, BANZ, CALL, RET variants */
        if (hi8 == 0xF8) {
            uint8_t sub = (op >> 4) & 0xF;
            /* F820 (624 sites) and F830 (543 sites) are BC pmad,cond per
             * tic54x-opc.c (bc = 0xF800 mask 0xFF00). The dispatcher at
             * PROM0 0xb968-0xb9a4 relies on these branching when the ACC
             * comparison succeeds. Cond 0x20 = C set, cond 0x30 = ?
             * (we treat both via ACC compare for now since dispatcher uses
             * cmp-style behaviour). The full F8xx range is BC per binutils
             * but historically the firmware tolerates the legacy decode
             * for the other sub-codes — surgical override here only. */
            if (sub == 0x2 || sub == 0x3) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                int64_t acc_signed = (s->a & 0x8000000000LL)
                                     ? (s->a | ~0xFFFFFFFFFFLL) : s->a;
                bool take = false;
                /* For now: cond=0x20 → branch if A != 0; cond=0x30 → A == 0.
                 * These are heuristics until we confirm the exact cond
                 * mapping from SPRU172C. Tweak based on observed dispatcher
                 * behaviour. */
                if (sub == 0x2)      take = (acc_signed != 0);
                else /* sub==0x3 */  take = (acc_signed == 0);
                if (take) { s->pc = op2; return 0; }
                return consumed + s->lk_used;
            }
            if (sub == 0x2) {
                /* Unreachable now — kept for clarity in case we revert. */
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                s->rea = op2;
                s->rsa = (uint16_t)(s->pc + 2);
                s->rptb_active = true;
                s->st1 |= ST1_BRAF;
                return consumed + s->lk_used;
            }
            if (sub == 0x3) {
                /* Unreachable now. */
                op2 = prog_fetch(s, s->pc + 1);
                s->rpt_count = op2;
                s->rpt_active = true;
                s->pc += 2;
                return 0;
            }
            /* Per tic54x-opc.c:
             *   F880-F8FF mask FF80 = FB pmad (FAR branch unconditional)
             * The low 7 bits of the opcode word encode the target XPC bits.
             * Calypso uses 2-bit XPC, so & 0x3 is sufficient.
             *
             * Earlier this range was treated as plain B pmad — a bug that
             * kept XPC=0 forever (DSP never reached PROM1 user code). */
            if ((op & 0xFF80) == 0xF880) {
                op2 = prog_fetch(s, s->pc + 1);
                consumed = 2;
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fb_total;
                fb_total++;
                if (fb_total <= 30 || (fb_total % 5000) == 0) {
                    C54_LOG("FB FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u)",
                            (unsigned long long)fb_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
            /* F88x..F8Bx (mask FF80=0): historic plain B pmad (NEAR), kept
             * for sub-codes that fall outside the FAR mask above. */
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
        /* F3xx: dispatch per binutils tic54x-opc.c (verified against
         * insn_template struct include/opcode/tic54x.h:85-150).
         *
         * 8 sub-families:
         *   F300-F31F  INTR k                                 1-word
         *   F320-F32F  unmapped                               (NOP fallback)
         *   F330-F35F  AND/OR/XOR #lk,SHIFT,SRC,DST  mask FCF0 2-word
         *   F360-F367  ADD/SUB/AND/OR/XOR/MAC #lk var. FCFF   2-word
         *   F368-F37F  unmapped                               (NOP fallback)
         *   F380-F39F  AND  src,SHIFT,DST            mask FCE0 1-word
         *   F3A0-F3BF  OR   src,SHIFT,DST            mask FCE0 1-word
         *   F3C0-F3DF  XOR  src,SHIFT,DST            mask FCE0 1-word
         *   F3E0-F3FF  SFTL src,SHIFT,DST            mask FCE0 1-word
         *
         * Dispatch order: most-specific masks first (FCFF → FCF0 → FCE0).
         *
         * 2026-04-29 — replaces previous "F320+ → LD #k9, DP" fallback
         * which mass-mis-decoded 364 firmware sites. Wedge at PC=0x8eb9
         * (0xF3E1 SFTL B,1,B) was directly tied to this bug.
         * See doc/opcodes/0xF3.md for full spec. */
        if (hi8 == 0xF3) {
            /* F300-F31F: INTR k (preserve existing behavior) */
            if ((op & 0xFFE0) == 0xF300) {
                int vec = op & 0x1F;
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 1));
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                return 0;
            }

            /* F360-F367: 2-word with mask FCFF (#lk<<16 variants).
             * Most-specific mask, check first. */
            if ((op & 0xFCFF) == 0xF060 ||  /* ADD #lk<<16, src, [dst] */
                (op & 0xFCFF) == 0xF061 ||  /* SUB */
                (op & 0xFCFF) == 0xF063 ||  /* AND */
                (op & 0xFCFF) == 0xF064 ||  /* OR  */
                (op & 0xFCFF) == 0xF065 ||  /* XOR */
                (op & 0xFCFF) == 0xF067) {  /* MAC #lk, src, [dst] */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int sub = op & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x0: result = src + ((int64_t)(int16_t)op2 << 16); break;
                case 0x1: result = src - ((int64_t)(int16_t)op2 << 16); break;
                case 0x3: result = src & (((int64_t)op2) << 16); break;
                case 0x4: result = src | (((int64_t)op2) << 16); break;
                case 0x5: result = src ^ (((int64_t)op2) << 16); break;
                case 0x7: { /* MAC: dst = src + T * lk */
                    int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)op2;
                    if (s->st1 & ST1_FRCT) prod <<= 1;
                    result = src + prod;
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F330-F35F: 2-word with mask FCF0 (#lk + 4-bit shift).
             * AND (sub=3), OR (sub=4), XOR (sub=5).
             * Note: ADD (sub=0) and SUB (sub=1) at F30x/F31x are caught
             * by INTR handler above (those ranges are INTR semantically). */
            if ((op & 0xFCF0) == 0xF030 ||  /* AND #lk, SHIFT, src, [dst] */
                (op & 0xFCF0) == 0xF040 ||  /* OR */
                (op & 0xFCF0) == 0xF050) {  /* XOR */
                op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
                consumed = 2;
                int subop = (op >> 4) & 0xF;
                int shift_raw = op & 0xF;
                int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int64_t src = src_b ? s->b : s->a;
                int64_t lk_signed = (int16_t)op2;
                int64_t shifted = (shift >= 0) ? (lk_signed << shift)
                                               : (lk_signed >> (-shift));
                int64_t result = src;
                switch (subop) {
                case 0x3: result = src & shifted; break;  /* AND */
                case 0x4: result = src | shifted; break;  /* OR  */
                case 0x5: result = src ^ shifted; break;  /* XOR */
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F380-F3FF: 1-word AND/OR/XOR/SFTL src,SHIFT,DST (mask FCE0).
             * Sub-opcode in bits 7-5: 100=AND, 101=OR, 110=XOR, 111=SFTL. */
            if ((op & 0xFCE0) == 0xF080 ||  /* AND */
                (op & 0xFCE0) == 0xF0A0 ||  /* OR  */
                (op & 0xFCE0) == 0xF0C0 ||  /* XOR */
                (op & 0xFCE0) == 0xF0E0) {  /* SFTL */
                int sub = (op >> 5) & 0x7;
                int src_b = (op >> 9) & 1;
                int dst_b = (op >> 8) & 1;
                int shift_raw = op & 0x1F;
                int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
                int64_t src = src_b ? s->b : s->a;
                int64_t result = src;
                switch (sub) {
                case 0x4: { /* AND src,SHIFT,DST: DST = SRC & (DST_in << shift) */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src & sh;
                    break;
                }
                case 0x5: { /* OR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src | sh;
                    break;
                }
                case 0x6: { /* XOR */
                    int64_t dst_in = dst_b ? s->b : s->a;
                    int64_t sh = (shift >= 0) ? (dst_in << shift) : (dst_in >> (-shift));
                    result = src ^ sh;
                    break;
                }
                case 0x7: { /* SFTL src,SHIFT,DST: DST = SRC << shift (logical) */
                    uint64_t usrc = (uint64_t)src & 0xFFFFFFFFFFULL;
                    result = (int64_t)((shift >= 0) ? (usrc << shift) : (usrc >> (-shift)));
                    break;
                }
                }
                if (dst_b) s->b = sext40(result); else s->a = sext40(result);
                return consumed + s->lk_used;
            }

            /* F320-F32F + F368-F37F: unmapped per binutils. NOP fallback +
             * log-once for diagnostic. 9 firmware sites total. */
            {
                static int unmapped_log = 0;
                if (unmapped_log++ < 20)
                    C54_LOG("F3xx unmapped op=0x%04x PC=0x%04x (NOP)",
                            op, s->pc);
            }
            return consumed + s->lk_used;
        }
        /* F6xx: various — LD/ST acc-acc, ABDST, SACCD, etc. */
        if (hi8 == 0xF6) {
            uint8_t sub = (op >> 4) & 0xF;
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
            if (sub == 0xB) {
                /* F6Bx: RSBX -- reset bit in ST1 (bit 9=1, bit 8=0).
                 * Per tic54x-opc.c: RSBX 0xF4B0 mask 0xFDF0 covers F6Bx. */
                int bit = op & 0x0F;
                s->st1 &= ~(1 << bit);
                return consumed + s->lk_used;
            }
            /* Delayed branches/calls/returns from PROM (per tic54x-opc.c).
             * MUST be checked BEFORE the MVDD catch-all because they share
             * the high nibbles 0xE/0x9. Without these the DSP cannot return
             * from interrupt service routines — RETED in particular leaves
             * INTM=1 forever, blocking every subsequent INT3 and stalling
             * the firmware↔DSP frame loop (the original CLAUDE.md root bug).
             *
             * All delayed forms execute 2 delay-slot words before the jump
             * commits; we arm the existing delayed_pc/delay_slots machinery
             * (the same one RCD uses) so the slots run with the right PC. */
            if (op == 0xF6EB) {
                /* RETED — return from interrupt, enable interrupts, delayed.
                 * Pop PC, clear INTM, then run 2 delay slots before jumping. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static uint64_t reted_count;
                    reted_count++;
                    if (reted_count <= 20 || (reted_count % 100) == 0)
                        C54_LOG("RETED #%llu PC=0x%04x -> ra=0x%04x SP=0x%04x INTM=0",
                                (unsigned long long)reted_count,
                                s->pc, ra, s->sp);
                }
                return consumed + s->lk_used;
            }
            if (op == 0xF69B) {
                /* RETFD — fast return, delayed (no INTM change). */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E2 || op == 0xF6E3) {
                /* BACCD A / CALAD A — delayed branch/call to acc(low).
                 * 1-word op + 2 delay slots. CALAD pushes PC+3 (skip op +
                 * 2 delay slots) per TI convention (cf. CALLD which pushes
                 * PC+4 for its 2-word form). Branch is armed via the
                 * delayed_pc/delay_slots mechanism so the 2 slots run
                 * before PC commits to tgt. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                bool is_call = (op == 0xF6E3);
                static uint64_t bcd_total;
                bcd_total++;
                /* Pre-load context: dump the 8 words preceding PC (in OVLY
                 * the executor reads from DARAM, mirror that). Lets us see
                 * which LD/MAR sequence was supposed to put a valid target
                 * in A before the CALAD/BACCD. */
                int pre_ovly = (s->pmst & PMST_OVLY) && s->pc >= 0x80 && s->pc < 0x2800;
                uint16_t pre[8];
                for (int i = 0; i < 8; i++) {
                    uint16_t a = (uint16_t)(s->pc - 8 + i);
                    pre[i] = pre_ovly ? s->data[a] : s->prog[a];
                }
                if (bcd_total <= 60 || (bcd_total % 5000) == 0) {
                    C54_LOG("BCD/CAD F6E%c #%llu PC=0x%04x tgt=0x%04x A=%010llx SP=0x%04x DP=0x%03x mem[%c PC-8..-1]=%04x %04x %04x %04x %04x %04x %04x %04x%s",
                            is_call ? '3' : '2',
                            (unsigned long long)bcd_total,
                            s->pc, tgt,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            (s->st0 & 0x1FF),
                            pre_ovly ? 'D' : 'P',
                            pre[0], pre[1], pre[2], pre[3],
                            pre[4], pre[5], pre[6], pre[7],
                            is_call ? " CALAD" : " BACCD");
                }
                if (is_call) {
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E4 || op == 0xF6E5) {
                /* FRETD / FRETED — far return, delayed.
                 * Pop XPC + PC unconditionally (FL_FAR). FRETED also clears INTM.
                 * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                s->xpc = data_read(s, s->sp); s->sp++;
                if (s->xpc > 3) s->xpc &= 3;
                uint16_t ra = data_read(s, s->sp); s->sp++;
                if (op == 0xF6E5) s->st1 &= ~ST1_INTM;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            if (op == 0xF6E6 || op == 0xF6E7) {
                /* FBACCD A / FCALAD A — far delayed branch/call to A.
                 * A(22:16) → XPC, A(15:0) → tgt. XPC update is immediate
                 * (mirrors FRETED at line ~1639). FCALAD pushes ret PC+3,
                 * and (when APTS) pushes XPC first (so RETF/FRETD pops in
                 * order). 2 delay slots. */
                uint16_t tgt = (uint16_t)(s->a & 0xFFFF);
                uint8_t  new_xpc = (uint8_t)((s->a >> 16) & 0xFF);
                if (new_xpc > 3) new_xpc &= 3;
                bool is_call = (op == 0xF6E7);
                static uint64_t fbcd_total;
                fbcd_total++;
                if (fbcd_total <= 10 || (fbcd_total % 5000) == 0) {
                    C54_LOG("FBCD/FCAD F6E%c #%llu PC=0x%04x tgt=0x%04x newXPC=%u A=%010llx SP=0x%04x%s",
                            is_call ? '7' : '6',
                            (unsigned long long)fbcd_total,
                            s->pc, tgt, new_xpc,
                            (unsigned long long)(s->a & 0xFFFFFFFFFFULL),
                            s->sp,
                            is_call ? " FCALAD" : " FBACCD");
                }
                if (is_call) {
                    /* FCALAD (F6E7): push XPC + return PC unconditionally (FL_FAR).
                     * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, s->xpc);
                    uint16_t ret_pc = (uint16_t)(s->pc + 3);
                    s->sp = (s->sp - 1) & 0xFFFF;
                    data_write(s, s->sp, ret_pc);
                }
                s->xpc         = new_xpc;
                s->delayed_pc  = tgt;
                s->delay_slots = 2;
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
        /* F5xx: SSBX or RPT #k */
        if (hi8 == 0xF5) {
            /* F5Bx: SSBX -- set bit in ST0 (bit 9=0, bit 8=1).
             * Per tic54x-opc.c: SSBX 0xF5B0 mask 0xFDF0. */
            if ((op & 0xFFF0) == 0xF5B0) {
                int bit = op & 0x0F;
                s->st0 |= (1 << bit);
                return consumed + s->lk_used;
            }
            /* Note: 0xF5E2/F5E3 (BACC B / CALA B) are handled earlier alongside
             * their F4 counterparts, so they never reach this F5xx block. */
            /* RPT #k (short immediate) — kept as fallback, must advance PC. */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        /* DIAG: log F7xx executions before the (buggy) LD #k8 dispatch.
         * Per tic54x-opc.c the F7xx range contains SSBX ST1 (0xF7Bx) and
         * other instructions, NOT LD #k8 (which is at E800-E9FF).
         * Caps at 5 per distinct sub-opcode to avoid spam. */
        if (hi8 == 0xF7) {
            static int f7xx_seen[256] = {0};
            int sub_idx = op & 0xFF;
            if (++f7xx_seen[sub_idx] <= 100 || (f7xx_seen[sub_idx] % 1000) == 0) {
                C54_LOG("F7xx EXEC op=0x%04x PC=0x%04x XPC=%d insn=%u",
                        op, s->pc, s->xpc, s->insn_count);
            }
        }
        /* F7Bx: SSBX bit, ST1 (incl. SSBX INTM at F7BB).
         * Per binutils tic54x-opc.c: opcode "ssbx" 0xF5B0 mask 0xFDF0,
         * where bit 9 selects ST0 (0xF5Bx) vs ST1 (0xF7Bx).
         * Symmetric counterpart of RSBX ST1 (F6Bx) handler above.
         * MUST be tested before the F7xx LD #k8 dispatch (which is
         * itself incorrect — per SPRU172C, LD #k8 lives at E800-E9FF). */
        if ((op & 0xFFF0) == 0xF7B0) {
            int bit = op & 0x0F;
            bool is_intm = (bit == 11);
            s->st1 |= (1 << bit);
            if (is_intm)
                C54_LOG("*** SSBX INTM (F7BB) *** PC=0x%04x ST1=0x%04x insn=%u",
                        s->pc, s->st1, s->insn_count);
            return consumed + s->lk_used;
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
            case 0xB: s->ar[7] = k; break; /* F7Bx: LD #k8, AR7 */
            case 0xC: s->bk = k; break;
            case 0xD: s->sp = k; break;
            case 0xE: /* F7Ex: LD #k8, BRC */
                s->brc = k; break;
            case 0xF: /* F7Fx: LD #k8, ... */
                break;
            }
            return consumed + s->lk_used;
        }
        /* F9xx encoding split per tic54x-opc.c:
         *   F900-F97F mask FF00 = CC pmad cond (NEAR conditional call)
         *   F980-F9FF mask FF80 = FCALL pmad   (FAR call unconditional)
         * The bit 7 of the opcode low byte distinguishes them. */
        if (hi8 == 0xF9) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALL FAR : push XPC + return PC unconditionally (FL_FAR).
             * Per binutils tic54x-opc.c (fcall 0xF980 mask 0xFF80, FL_FAR)
             * and SPRU172C: FAR call always saves XPC for FRET to restore.
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics).
             * Old behavior caused 281 firmware FCALL FAR sites to push only PC,
             * imbalanced with 142 FRET pop expecting both PC + XPC. */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcall_total;
                fcall_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                if (fcall_total <= 30 || (fcall_total % 5000) == 0) {
                    C54_LOG("FCALL FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x)",
                            (unsigned long long)fcall_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->pc  = op2;
                return 0;
            }
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
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 2));
                /* CC leak tracer */
                {
                    static uint32_t cc_targets[64];
                    static uint32_t cc_counts[64];
                    static int cc_n = 0;
                    static uint32_t total_cc = 0;
                    bool found = false;
                    for (int i = 0; i < cc_n; i++) {
                        if (cc_targets[i] == op2) { cc_counts[i]++; found = true; break; }
                    }
                    if (!found && cc_n < 64) { cc_targets[cc_n] = op2; cc_counts[cc_n++] = 1; }
                    if ((++total_cc % 100) == 0) {
                        C54_LOG("F9xx CC TOP TARGETS (SP=0x%04x total=%u):", s->sp, total_cc);
                        for (int i = 0; i < cc_n && i < 10; i++)
                            C54_LOG("  CC→0x%04x count=%u", cc_targets[i], cc_counts[i]);
                    }
                }
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FAxx encoding split per tic54x-opc.c:
         *   FA80-FAFF mask FF80 = FBD pmad (FAR branch delayed)
         *   FA00-FA7F = various NEAR delayed ops (treated as branch). */
        if (hi8 == 0xFA) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            if ((op & 0x80) != 0) {
                /* FBD FAR delayed branch — XPC change, no push */
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fbd_total;
                fbd_total++;
                if (fbd_total <= 30 || (fbd_total % 5000) == 0) {
                    C54_LOG("FBD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u, delayed 2 slots)",
                            (unsigned long long)fbd_total, s->pc,
                            new_xpc, op2, s->xpc);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
            /* NEAR FAxx fallback: simplified treat as branch */
            s->pc = op2;
            return 0;
        }
        /* FBxx encoding split per tic54x-opc.c:
         *   FB80-FBFF mask FF80 = FCALLD pmad (FAR call delayed)
         *   FB00-FB7F mask FF00 = CCD pmad cond (NEAR conditional call delayed) */
        if (hi8 == 0xFB) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            /* FCALLD FAR : push XPC + return PC+4 unconditionally (FL_FAR delayed).
             * Per binutils (fcalld 0xFB80 mask 0xFF80, FL_FAR|FL_DELAY).
             * 2026-04-28 — fixed: was APTS-gated (= AVIS, no stack semantics). */
            if ((op & 0x80) != 0) {
                uint8_t new_xpc = (op & 0x7F) & 0x03;
                static uint64_t fcalld_total;
                fcalld_total++;
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, s->xpc);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, (uint16_t)(s->pc + 4));
                if (fcalld_total <= 30 || (fcalld_total % 5000) == 0) {
                    C54_LOG("FCALLD FAR #%llu PC=0x%04x → XPC=%u PC=0x%04x (was XPC=%u SP=0x%04x, delayed)",
                            (unsigned long long)fcalld_total, s->pc,
                            new_xpc, op2, s->xpc, s->sp);
                }
                s->xpc = new_xpc;
                s->delayed_pc  = op2;
                s->delay_slots = 2;
                return consumed + s->lk_used;
            }
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
                s->sp--;
                data_write(s, s->sp, (uint16_t)(s->pc + 4)); /* past CCD + 2 delay slots */
                s->pc = op2;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FCxx: LD #k, 16, B */
        /* FCxx: RC cond / RET -- return conditional (1-word).
         * Per tic54x-opc.c: RET=0xFC00, RC=0xFC00 mask 0xFF00. */
        if (hi8 == 0xFC) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                uint16_t ra = data_read(s, s->sp); s->sp++;
                {
                    static int rc_log = 0;
                    if (rc_log < 50)
                        C54_LOG("RC/RET PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x",
                                s->pc, cc, ra, s->sp);
                    rc_log++;
                }
                s->pc = ra;
                return 0;
            }
            return consumed + s->lk_used;
        }
        /* FDxx: LD #k, A (no shift) */
        if (hi8 == 0xFD) {
            int8_t k = (int8_t)(op & 0xFF);
            s->a = sext40((int64_t)k);
            return consumed + s->lk_used;
        }
        /* FExx: RCD cond / RETD -- return conditional delayed (1-word).
         * Per tic54x-opc.c: RETD=0xFE00, RCD=0xFE00 mask 0xFF00.
         * Simplified: immediate return (delay slots skipped). */
        if (hi8 == 0xFE) {
            uint8_t cc = op & 0xFF;
            bool cond = false;
            /* Evaluate condition per tic54x-opc.c encoding:
             * CC1=0x40: accumulator test, CCB=0x08: use B (else A)
             * EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
             * OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08 */
            if (cc == 0x00) cond = true; /* UNC */
            else if (cc & 0x40) {
                /* Accumulator condition */
                int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
                uint8_t test = cc & 0x07;
                bool ov = (cc & 0x08) ? (s->st0 & (1<<9)/*OVB*/) : (s->st0 & (1<<8)/*OVA*/);
                if ((cc & 0x70) == 0x70) cond = ov;        /* AOV/BOV */
                else if ((cc & 0x70) == 0x60) cond = !ov;  /* ANOV/BNOV */
                else {
                    switch (test) {
                    case 0x05: cond = (acc == 0); break;  /* EQ */
                    case 0x04: cond = (acc != 0); break;  /* NEQ */
                    case 0x03: cond = (acc < 0); break;   /* LT */
                    case 0x07: cond = (acc <= 0); break;  /* LEQ */
                    case 0x06: cond = (acc > 0); break;   /* GT */
                    case 0x02: cond = (acc >= 0); break;  /* GEQ */
                    default: cond = true; break;
                    }
                }
            }
            else if ((cc & 0x30) == 0x30) cond = (s->st0 & ST0_TC) != 0; /* TC */
            else if ((cc & 0x30) == 0x20) cond = !(s->st0 & ST0_TC);     /* NTC */
            else if ((cc & 0x0C) == 0x0C) cond = (s->st0 & ST0_C) != 0;  /* C */
            else if ((cc & 0x0C) == 0x08) cond = !(s->st0 & ST0_C);      /* NC */
            else cond = true; /* unknown: take it */
            if (cond) {
                /* RCD is *delayed*: per SPRU172C the next 2 instructions
                 * after RCD execute before the return takes effect. The
                 * old "skip delay slots" implementation broke FB-detection
                 * because slots like `LD #0, B` at PROM0 0x75ea were never
                 * run, leaving accumulator state stale and the dispatcher
                 * at 0x7700 looping forever.
                 *
                 * Fix: arm the existing delayed_pc/delay_slots machinery —
                 * pop the return address now, advance PC normally so the
                 * next 2 instructions execute as delay slots, then the
                 * main loop forces PC = delayed_pc. */
                uint16_t ra = data_read(s, s->sp); s->sp++;
                s->delayed_pc  = ra;
                s->delay_slots = 2;
                {
                    static int rcd_log = 0;
                    if (rcd_log < 50)
                        C54_LOG("RCD/RETD PC=0x%04x cc=0x%02x -> ra=0x%04x SP=0x%04x (delayed)",
                                s->pc, cc, ra, s->sp);
                    rcd_log++;
                }
                return consumed + s->lk_used;
            }
            return consumed + s->lk_used;
        }
        /* FFxx is XC 2,cond — handled above with FDxx. No ADD here. */
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
            /* EAxx: LD #k9, DP — Load Data Page pointer (1-word).
             * Per tic54x-opc.c: ld 0xEA00 mask 0xFE00, 1 word. */
            uint16_t k9 = op & 0x01FF;
            uint16_t old_dp = s->st0 & ST0_DP_MASK;
            s->st0 = (s->st0 & ~ST0_DP_MASK) | k9;
            {
                static uint64_t dpc;
                dpc++;
                if (dpc <= 80 || (dpc % 5000) == 0 || k9 == 0x83) {
                    C54_LOG("DP-SET EAxx #%llu PC=0x%04x DP 0x%03x → 0x%03x %s",
                            (unsigned long long)dpc, s->pc,
                            old_dp, k9,
                            k9 == 0x83 ? "*** 0x83 (CALAD-zone base 0x4180) ***" : "");
                }
            }
            return consumed + s->lk_used;
        }
        if (hi8 == 0xEC) {
            /* ECxx: RPT #k8u — repeat next instruction k8u+1 times.
             * Per tic54x-opc.c: rpt 0xEC00 mask 0xFF00, single word.
             * Must advance PC past RPT now and return 0 so the dispatcher
             * re-executes the NEXT instruction (not RPT itself). */
            s->rpt_count = op & 0xFF;
            s->rpt_active = true;
            s->pc += 1;
            return 0;
        }
        if (hi8 == 0xE5) {
            /* E5xx: MVDD Xmem, Ymem  (per tic54x-opc.c, NOT MVMM)
             * 1-word, 2-cycle dual-operand data-to-data move:
             *   *Ymem = *Xmem
             * Per tic54x.h:
             *   XMEM = (op & 0xF0) >> 4
             *   YMEM = op & 0x0F
             *   XMOD/YMOD = (nibble & 0xC) >> 2  (0=*AR,1=*AR-,2=*AR+,3=*AR+0%)
             *   XARX/YARX = (nibble & 0x3) + 2   (AR2..AR5 only) */
            uint8_t xnib = (op >> 4) & 0xF;
            uint8_t ynib = op & 0xF;
            int xar = (xnib & 0x3) + 2;
            int yar = (ynib & 0x3) + 2;
            int xmod = (xnib & 0xC) >> 2;
            int ymod = (ynib & 0xC) >> 2;
            uint16_t xa = s->ar[xar];
            uint16_t ya = s->ar[yar];
            uint16_t v = data_read(s, xa);
            data_write(s, ya, v);
            /* Post-modify both ARs per their mod field */
            switch (xmod) {
                case 0: break;                        /* *AR     */
                case 1: s->ar[xar] = xa - 1; break;   /* *AR-    */
                case 2: s->ar[xar] = xa + 1; break;   /* *AR+    */
                case 3: s->ar[xar] = xa + s->ar[0]; break; /* *AR+0% (no circular here) */
            }
            switch (ymod) {
                case 0: break;
                case 1: s->ar[yar] = ya - 1; break;
                case 2: s->ar[yar] = ya + 1; break;
                case 3: s->ar[yar] = ya + s->ar[0]; break;
            }
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
            /* E7xx: MVMM mmrx, mmry  (per tic54x-opc.c)
             * 1-word, 2-cycle, MMR-to-MMR move using a constrained set
             * (MMRX/MMRY operand types). */
            int src = (op >> 4) & 0xF;
            int dst = op & 0xF;
            uint16_t val;
            if (src <= 7) val = s->ar[src];
            else if (src == 8) val = s->sp;
            else val = data_read(s, src + 0x10);
            if (dst <= 7) s->ar[dst] = val;
            else if (dst == 8) s->sp = val;
            else data_write(s, dst + 0x10, val);
            return consumed + s->lk_used;
        }
        if (hi8 == 0xE8 || hi8 == 0xE9) {
            /* E8xx/E9xx: LD #k8u, dst — Load 8-bit unsigned immediate (1-word).
             * Per tic54x-opc.c: ld 0xE800 mask 0xFE00.
             * bit 8 = dst (0=A, 1=B), bits 7:0 = k8u.
             * NOTE: This was previously decoded as CC (conditional call, 2-word)
             * which caused stack overflow by pushing return addresses in a loop. */
            int dst = (op >> 8) & 1;
            uint8_t k = op & 0xFF;
            int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int8_t)k : (int64_t)k;
            if (dst) s->b = sext40(v << 16);
            else     s->a = sext40(v << 16);
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
        if ((op & 0xFFE0) == 0xED00) {
            /* ED00-ED1F: LD #k5, ASM — load 5-bit immediate into ASM field of ST1.
             * Per tic54x-opc.c: ld 0xED00 mask 0xFFE0, 1 word.
             * NOT BCD (which is 0xFA00 mask 0xFF00). */
            uint8_t k5 = op & 0x1F;
            s->st1 = (s->st1 & ~ST1_ASM_MASK) | k5;
            return consumed + s->lk_used;
        }
        if (hi8 == 0xED) {
            /* EDxx (not ED00-ED1F): BCD pmad, cond (conditional branch delayed, 2 words) */
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
        goto unimpl;

    case 0x6: case 0x7:
        /* 7Exx: READA Smem — read prog[A_low] → data[Smem]
         * Per tic54x-opc.c: reada 0x7E00 mask 0xFF00 (1 word).
         * Under RPT, the prog address auto-increments each iteration;
         * accumulator A is preserved (we mirror via mvpd_src state). */
        if (hi8 == 0x7E) {
            addr = resolve_smem(s, op, &ind);
            uint16_t psrc = s->rpt_active ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            uint16_t v = prog_read(s, psrc);
            data_write(s, addr, v);
            s->mvpd_src = psrc + 1;
            { static int reada_log = 0; if (reada_log++ < 20)
                C54_LOG("READA: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x rpt=%d insn=%u",
                        psrc, v, addr, s->pc, s->rpt_count, s->insn_count); }
            return consumed + s->lk_used;
        }
        /* 7Fxx: WRITA Smem — write data[Smem] → prog[A_low] (mirror of READA) */
        if (hi8 == 0x7F) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pdst = s->rpt_active ? s->mvpd_src : (uint16_t)(s->a & 0xFFFF);
            prog_write(s, pdst, data_read(s, addr));
            s->mvpd_src = pdst + 1;
            return consumed + s->lk_used;
        }
        /* 6Dxx: MAR Smem — modify address register (side effects only) */
        if (hi8 == 0x6D) {
            addr = resolve_smem(s, op, &ind);
            /* MAR only modifies AR via addressing mode, no data access */
            return consumed + s->lk_used;
        }
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
        /* 0x6000-0x60FF: CMPM Smem, lk  (compare memory with long immediate)
         * Per tic54x-opc.c: { "cmpm", 2,2,2, 0x6000, 0xFF00 }
         * Sets TC = (data[Smem] == lk).
         *
         * The DSP bootloader at PROM0 0xb41c / 0xb424 polls
         *   CMPM *(0x0fff), 4   →  CMPM *(0x0fff), 2
         * to wait for ARM-side BL_CMD_STATUS write. Without TC being set
         * the subsequent BC NTC always branches back, looping forever.
         * Was previously folded into the generic 0x6000-0x67FF "LD" path
         * which set the accumulator instead and never updated TC. */
        if ((op & 0xFF00) == 0x6000) {
            addr = resolve_smem(s, op, &ind);
            uint16_t cmp_val = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            if (mem_val == cmp_val) s->st0 |= ST0_TC;
            else                    s->st0 &= ~ST0_TC;
            consumed = 2;  /* opcode + cmp_val (smem extra lk added via lk_used) */
            return consumed + s->lk_used;
        }
        /* 0x6100-0x61FF: BITF Smem, lk — bit-field test, TC = (Smem & lk)!=0 */
        if ((op & 0xFF00) == 0x6100) {
            addr = resolve_smem(s, op, &ind);
            uint16_t mask = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mem_val = data_read(s, addr);
            if (mem_val & mask) s->st0 |= ST0_TC;
            else                s->st0 &= ~ST0_TC;
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6000) {
            /* 60xx-67xx: LD Smem, dst (other variants — fallback) */
            int dst_acc = (op >> 9) & 1;
            int shift = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t val = data_read(s, addr);
            int64_t v = (s->st1 & ST1_SXM) ? (int16_t)val : val;
            if (shift) v <<= 16;  /* LD Smem, 16, dst */
            if (dst_acc) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        /* 0x6800-0x6BFF + 0x6Cxx + 0x6Exx: companion to the 0x6F00 fix below.
         * Per binutils tic54x-opc.c (verified against insn_template struct):
         *   0x6800 ANDM  #lk, Smem      data[Smem] = data[Smem] & lk     (2-word)
         *   0x6900 ORM   #lk, Smem      data[Smem] = data[Smem] | lk     (2-word)
         *   0x6A00 XORM  #lku, Smem     data[Smem] = data[Smem] ^ lku    (2-word)
         *   0x6B00 ADDM  #lk, Smem      data[Smem] = data[Smem] + lk     (2-word)
         *   0x6C00 BANZ  pmad, Sind     if (ARx != 0) PC = pmad          (2-word)
         *   0x6E00 BANZD pmad, Sind     same as BANZ but with 2 delay slots
         *
         * Without these, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes them all as LD Smem,T (1-word), causing PC drift +1
         * word and the lk/pmad operand executing as parasitic instruction.
         * 1259 (ANDM/ORM/XORM/ADDM) + 304 (BANZ/BANZD) = 1563 sites in ROM.
         *
         * 2026-04-28 — companion fix to 0x6F00 already inserted below.
         * See doc/opcodes/0x68_0x6F.md for spec. */
        if ((op & 0xFF00) == 0x6800) {
            /* ANDM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v & lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6900) {
            /* ORM #lk, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v | lk);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6A00) {
            /* XORM #lku, Smem */
            addr = resolve_smem(s, op, &ind);
            uint16_t lku = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, v ^ lku);
            consumed = 2;
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6B00) {
            /* ADDM #lk, Smem — add signed lk to memory (wrap mod 2^16) */
            addr = resolve_smem(s, op, &ind);
            int16_t lk = (int16_t)prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t v = data_read(s, addr);
            data_write(s, addr, (uint16_t)((int16_t)v + lk));
            consumed = 2;
            /* TODO: TC/OVM/SXM flag effects per SPRU172C (verify) */
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6C00) {
            /* BANZ pmad, Sind — branch if ARx != 0 after indirect-mode applied.
             * Per SPRU172C p.4-15: resolve_smem applies the indirect mode (may
             * modify ARx pre/post depending on mode); BANZ tests resulting
             * ARx and branches to pmad if non-zero, else falls through. */
            int arp = (s->st0 >> ST0_ARP_SHIFT) & 0x7;
            resolve_smem(s, op, &ind);  /* side-effect on s->ar[arp] */
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (s->ar[arp] != 0) {
                s->pc = pmad;
                return 0;  /* PC set directly */
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xFF00) == 0x6E00) {
            /* BANZD pmad, Sind — delayed BANZ (2 slots after the 2-word op).
             * Reuses the existing delayed_pc/delay_slots machinery (same one
             * CALAD/CALLD/RPTBD use) so the 2 delay slots run before PC
             * commits to pmad. */
            int arp = (s->st0 >> ST0_ARP_SHIFT) & 0x7;
            resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            consumed = 2;
            if (s->ar[arp] != 0) {
                s->delayed_pc  = pmad;
                s->delay_slots = 2;
            }
            return consumed + s->lk_used;
        }
        /* 0x6F00-0x6FFF: Extended ADD/SUB/LD/STH/STL Smem, SHIFT, DST/SRC (2-word).
         * Per binutils tic54x-opc.c (verified against insn_template struct
         * include/opcode/tic54x.h:85-150):
         *   word0 = 0x6F00 mask 0xFF00 (Smem in low 7 bits)
         *   word1 = sub-opcode in bits 7:5, SRC=bit 9, DST/SRC1=bit 8,
         *           SHIFT=signed 5-bit in bits 4:0
         *     bits 7:5 = 000 → ADD Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 001 → SUB Smem,SHIFT,SRC,[DST]
         *     bits 7:5 = 010 → LD  Smem,SHIFT,DST
         *     bits 7:5 = 011 → STH SRC1,SHIFT,Smem
         *     bits 7:5 = 100 → STL SRC1,SHIFT,Smem
         *
         * Without this handler, the fallback at (op & 0xF800) == 0x6800 below
         * mis-decodes 0x6Fxx as LD Smem,T (1-word), causing PC drift +1 word
         * and the lk-side operand to be executed as parasitic instruction.
         * 544 sites in firmware ROM. See doc/opcodes/0x68_0x6F.md for spec.
         *
         * 2026-04-28 — fix introduced for wedge at PC=0x8353 (CALAD A self-loop)
         * caused by 0x6F07 0x0C41 mis-decoded → 0x0C41 executed as parasitic
         * SUB Smem,TS,A → A_low=0xFFFA → A_low=0x8353 after subsequent ADD. */
        if ((op & 0xFF00) == 0x6F00) {
            addr = resolve_smem(s, op, &ind);
            op2 = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            int sub = (op2 >> 5) & 0x7;
            int shift_raw = op2 & 0x1F;
            int shift = (shift_raw & 0x10) ? (shift_raw - 32) : shift_raw;
            int dst_b = (op2 >> 8) & 1;   /* bit 8 = DST/SRC1 */
            int src_b = (op2 >> 9) & 1;   /* bit 9 = SRC (ADD/SUB only) */
            consumed = 2;

            switch (sub) {
            case 0: { /* ADD Smem,SHIFT,SRC,[DST]: DST = SRC + (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src + v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 1: { /* SUB Smem,SHIFT,SRC,[DST]: DST = SRC - (data[Smem]<<shift) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                int64_t src = src_b ? s->b : s->a;
                int64_t result = sext40(src - v);
                if (dst_b) s->b = result; else s->a = result;
                break;
            }
            case 2: { /* LD Smem,SHIFT,DST: DST = data[Smem] << shift (SXM-aware) */
                uint16_t mv = data_read(s, addr);
                int64_t v = (s->st1 & ST1_SXM) ? (int64_t)(int16_t)mv : (int64_t)mv;
                v = (shift >= 0) ? (v << shift) : (v >> (-shift));
                if (dst_b) s->b = sext40(v); else s->a = sext40(v);
                break;
            }
            case 3: { /* STH SRC1,SHIFT,Smem: data[Smem] = (SRC1 high 16) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
                                               : ((int64_t)high >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            case 4: { /* STL SRC1,SHIFT,Smem: data[Smem] = (SRC1 low) << shift */
                int64_t src = dst_b ? s->b : s->a;
                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
                break;
            }
            default:
                { static int unk6f = 0; if (unk6f++ < 10)
                    C54_LOG("0x6F unknown sub=%d op=0x%04x op2=0x%04x PC=0x%04x",
                            sub, op, op2, s->pc); }
                break;
            }
            return consumed + s->lk_used;
        }
        if ((op & 0xF800) == 0x6800) {
            /* DEAD CODE since 2026-04-28: all 0x68xx-0x6Fxx now intercepted
             * by specific handlers above (ANDM/ORM/XORM/ADDM/BANZ/BANZD/
             * extended-0x6F00) plus the existing 0x6Dxx MAR. This generic
             * "LD Smem, T" fallback was the source of the 2107-site mass
             * mis-dispatch that caused PC drift on every 0x68xx-0x6Fxx
             * encounter. Kept here for safety in case a previously unseen
             * sub-encoding slips through; if you ever see this trigger,
             * the new handler above for the matching 0xNN00 prefix is
             * incomplete. See doc/opcodes/0x68_0x6F.md. */
            addr = resolve_smem(s, op, &ind);
            s->t = data_read(s, addr);
            return consumed + s->lk_used;
        }
        goto unimpl;

    case 0x1: {
        /* 1xxx: LD / LDU / LDR Smem, DST  (per tic54x-opc.c, all mask FE00):
         *   0x1000  LD  Smem, DST          — signed load (SXM-aware)
         *   0x1200  LDU Smem, DST          — unsigned load (zero-extend)
         *   0x1400  LD  Smem, TS, DST      — load shifted by T low bits
         *   0x1600  LDR Smem, DST          — load with rounding
         *
         * Critical: bootloader at PROM0 0xb429 does `LDU *(0x0ffe), A`
         * (op=0x12f8 + lk=0x0ffe) to read BL_ADDR_LO, then BACC A to that
         * target. The previous "case 0x1: SUB" decoded this as a subtract,
         * leaving A=0 and the BACC dropping into boot-stub NOPs. */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* selects LD/LDU/LD,TS/LDR within case 1 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        switch (sub) {
        case 0x0:  /* 0x1000: LD Smem, DST — signed (SXM honoured) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        case 0x1: { /* 0x1200: LDU Smem, DST — always zero-extended */
            v = (uint16_t)val;
            break;
        }
        case 0x2: { /* 0x1400: LD Smem, TS, DST — shift by T[5:0] (signed) */
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            int64_t base = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (ts >= 0) ? (base << ts) : (base >> -ts);
            break;
        }
        case 0x3: { /* 0x1600: LDR Smem, DST — load with rounding (+0x8000) */
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            v = (v << 16) + 0x8000;
            v &= 0xFFFFFFFF0000LL;  /* clear low 16 after rounding */
            if (dst) s->b = sext40(v); else s->a = sext40(v);
            return consumed + s->lk_used;
        }
        default:
            v = (s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val;
            break;
        }
        if (dst) s->b = sext40(v); else s->a = sext40(v);
        /* CALAD-zone LD trace: every LD/LDU/LDR that targets A while
         * executing in DARAM near the CALAD cluster. Reveals what
         * address/value is feeding A right before each CALAD A. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t ldA_total;
            ldA_total++;
            if (ldA_total <= 60 || (ldA_total % 5000) == 0) {
                C54_LOG("LD-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=0x%04x DP=0x%03x",
                        (unsigned long long)ldA_total,
                        s->pc, op, sub, addr, val,
                        (uint16_t)(s->a & 0xFFFF),
                        (s->st0 & 0x1FF));
            }
        }
        return consumed + s->lk_used;
    }

    case 0x0: {
        /* 0xxx: ADD / ADDS / ADD,TS / SUB / SUBS / SUB,TS  (mask FE00):
         *   0x0000 ADD  Smem, SRC1 (no shift, SXM honoured)
         *   0x0200 ADDS Smem, SRC1 (no shift, zero-extended)
         *   0x0400 ADD  Smem, TS, SRC1
         *   0x0800 SUB  Smem, SRC1
         *   0x0A00 SUBS Smem, SRC1
         *   0x0C00 SUB  Smem, TS, SRC1
         * Previous handler always shifted by 16 — wrong for plain ADD/SUB.
         */
        addr = resolve_smem(s, op, &ind);
        int dst = (op >> 8) & 1;
        int sub = (op >> 9) & 0x07;  /* 0..7 */
        uint16_t val = data_read(s, addr);
        int64_t v;
        bool is_sub = (sub & 0x4) != 0;
        bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
        bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
        v = is_unsigned ? (uint16_t)val
                        : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
        if (ts_shift) {
            int8_t ts = (int8_t)((s->t & 0x3F) | ((s->t & 0x20) ? 0xC0 : 0));
            v = (ts >= 0) ? (v << ts) : (v >> -ts);
        }
        if (is_sub) {
            if (dst) s->b = sext40(s->b - v);
            else     s->a = sext40(s->a - v);
        } else {
            if (dst) s->b = sext40(s->b + v);
            else     s->a = sext40(s->a + v);
        }
        /* CALAD-zone ADD/SUB trace: same scope as LD-A-TRACE. */
        if (dst == 0 && (s->pmst & PMST_OVLY) &&
            s->pc >= 0x10b0 && s->pc < 0x1100) {
            static uint64_t addA_total;
            addA_total++;
            if (addA_total <= 30 || (addA_total % 5000) == 0) {
                C54_LOG("ADDSUB-A-TRACE #%llu PC=0x%04x op=0x%04x sub=%d addr=0x%04x val=0x%04x A_after=%010llx",
                        (unsigned long long)addA_total,
                        s->pc, op, sub, addr, val,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFULL));
            }
        }
        return consumed + s->lk_used;
    }

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
        /* 0x4xxx group — per binutils tic54x-opc.c:
         *   0x40-0x43  SUB Smem,16,src[,dst]    (mask 0xFC00)
         *   0x44-0x45  LD  Smem,16,dst          (mask 0xFE00)
         *   0x4600     LD  Smem,DP              (mask 0xFF00)
         *   0x4700     RPT Smem                 (mask 0xFF00)
         *   0x48-0x49  LDM MMR,dst              (mask 0xFE00)
         *   0x4A00     PSHM MMR                 (mask 0xFF00)
         *   0x4B00     PSHD Smem                (mask 0xFF00)
         *   0x4C00     LTD Smem                 (mask 0xFF00)
         *   0x4D00     DELAY Smem               (mask 0xFF00)
         *   0x4E-0x4F  DST src,Lmem             (mask 0xFE00) */
        {
            uint8_t op8 = hi8;            /* (op >> 8) & 0xFF */
            int dst_b = op8 & 0x01;        /* bit8 = src/dst select (A=0, B=1) */
            int64_t *acc_dst = dst_b ? &s->b : &s->a;

            if (op8 >= 0x40 && op8 <= 0x43) {
                /* SUB Smem << 16, src, dst — sub of shifted Smem from acc */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(*acc_dst - val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x44 || op8 == 0x45) {
                /* LD Smem << 16, dst */
                addr = resolve_smem(s, op, &ind);
                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
                *acc_dst = sext40(val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x46) {
                /* LD Smem, DP — load DP from low 9 bits of Smem */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->st0 = (s->st0 & ~ST0_DP_MASK) | (val & ST0_DP_MASK);
                return consumed + s->lk_used;
            }
            if (op8 == 0x47) {
                /* RPT Smem — load BRC from mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->brc = val;
                s->rpt_active = (val != 0);
                return consumed + s->lk_used;
            }
            if (op8 == 0x48 || op8 == 0x49) {
                /* LDM MMR, dst — load accumulator from a memory-mapped reg */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                *acc_dst = sext40((int16_t)val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4A) {
                /* PSHM MMR — push memory-mapped reg onto stack */
                int mmr = op & 0x7F;
                uint16_t val = data_read(s, mmr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4B) {
                /* PSHD Smem — push data memory onto stack */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->sp = (s->sp - 1) & 0xFFFF;
                data_write(s, s->sp, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4C) {
                /* LTD Smem — T = mem[Smem]; mem[Smem+1] = mem[Smem] */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                s->t = val;
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4D) {
                /* DELAY Smem — mem[Smem+1] = mem[Smem] (delay-line shift) */
                addr = resolve_smem(s, op, &ind);
                uint16_t val = data_read(s, addr);
                data_write(s, (addr + 1) & 0xFFFF, val);
                return consumed + s->lk_used;
            }
            if (op8 == 0x4E || op8 == 0x4F) {
                /* DST src, Lmem — store accumulator to long memory.
                 * Lmem = even-aligned 32-bit pair: mem[L]=high, mem[L+1]=low */
                addr = resolve_smem(s, op, &ind) & 0xFFFE;
                int64_t v = *acc_dst;
                data_write(s, addr,         (uint16_t)((v >> 16) & 0xFFFF));
                data_write(s, (addr+1)&0xFFFF, (uint16_t)(v & 0xFFFF));
                return consumed + s->lk_used;
            }
        }
        return consumed + s->lk_used;

    case 0x5:
        /* 5xxx: shifts — SFTA, SFTL, various forms.
         * NOTE: 0x56xx/0x57xx are SFTL/SFTA with Smem (1-word), NOT MVPD.
         * MVPD is at 0x8Cxx (hi8=0x8C). The old 0x56 MVPD decode was wrong
         * and caused writes to MMR_SP via resolve_smem, corrupting the stack. */
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
        /* 0x88xx-0x89xx: STLM src, MMR  (1-word!)
         * Per tic54x-opc.c: { "stlm", 1,2,2, 0x8800, 0xFE00, ... }
         *   bits 9-15 = fixed (0x44)
         *   bit 8     = src (0 = A, 1 = B)
         *   bits 0-6  = MMR address (0x00..0x7F)
         *
         * Critical for the DSP bootloader at PROM0 0xb42d (`STLM B, AR1`):
         * if decoded as 2-word MVDM the emulator eats the next opcode
         * (0xb42e = 0xf84c, a BC), then jumps into 0xb431 (MACR family)
         * with an uninitialised T register, producing A=0x10 — which
         * the immediately-following BACC A at 0xb430 then uses as the
         * jump target, dropping the DSP into the boot-stub NOPs at
         * PC=0x0010 instead of continuing the bootloader handshake. */
        if (hi8 == 0x88 || hi8 == 0x89) {
            int src = (op >> 8) & 1;  /* 0 = A, 1 = B */
            int mmr = op & 0x7F;
            uint16_t val = src ? (uint16_t)(s->b & 0xFFFF)
                               : (uint16_t)(s->a & 0xFFFF);
            data_write(s, (uint16_t)mmr, val);  /* MMRs alias addr 0x00..0x1F */
            return consumed + s->lk_used;
        }
        if (hi8 == 0x80) {
            /* MVDD Smem, Smem (data→data) — 2-word */
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
            {
                static unsigned mvpd_log = 0;
                static unsigned mvpd_total;
                static uint16_t src_min = 0xFFFF, src_max;
                static uint16_t dst_min = 0xFFFF, dst_max;
                static unsigned hits_a040;
                mvpd_total++;
                if (op2 < src_min) src_min = op2;
                if (op2 > src_max) src_max = op2;
                if (addr < dst_min) dst_min = addr;
                if (addr > dst_max) dst_max = addr;
                if (addr >= 0xa040 && addr <= 0xa080) hits_a040++;
                if (mvpd_log++ < 500 ||
                    (addr >= 0xa040 && addr <= 0xa080) ||
                    (mvpd_total % 1000) == 0)
                    C54_LOG("MVPD#%u: prog[0x%04x]=0x%04x → data[0x%04x] PC=0x%04x insn=%u%s",
                            mvpd_total, op2, mvpd_val, addr, s->pc, s->insn_count,
                            (addr >= 0xa040 && addr <= 0xa080) ? " *A040*" : "");
                if ((mvpd_total % 500) == 0)
                    C54_LOG("MVPD-SUMMARY total=%u src=[0x%04x..0x%04x] dst=[0x%04x..0x%04x] hits_a040=%u",
                            mvpd_total, src_min, src_max, dst_min, dst_max, hits_a040);
            }
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
            /* BSP RX data register — return next burst sample.
             * The DSP firmware uses PORTR PA=0xF430 (64 sites in PROM0,
             * verified from ROM dump). We also accept 0x0034 for legacy
             * compatibility with earlier QEMU experiments. */
            uint16_t portr_val;
            bool is_bsp_pa = (op2 == 0xF430 || op2 == 0x0034);
            if (is_bsp_pa && s->bsp_pos < s->bsp_len) {
                portr_val = s->bsp_buf[s->bsp_pos++];
                data_write(s, addr, portr_val);
            } else {
                portr_val = 0;
                data_write(s, addr, 0);
            }
            /* Per-PA counters so we can see which I/O ports the DSP polls
             * and how often. */
            {
                static uint64_t portr_total[16];
                static uint64_t portr_since_summary;
                int pa_bucket = (op2 >> 4) & 0xF;
                portr_total[pa_bucket]++;
                portr_since_summary++;

                static int portr_log = 0;
                if (portr_log < 50) {
                    C54_LOG("PORTR PA=0x%04x → [0x%04x] val=0x%04x "
                            "bsp_pos=%u/%u PC=0x%04x",
                            op2, addr, portr_val,
                            (unsigned)s->bsp_pos, (unsigned)s->bsp_len,
                            s->pc);
                    portr_log++;
                }
                if ((portr_since_summary % 10000) == 0) {
                    C54_LOG("PORTR summary (last 10000): "
                            "PA0x=%llu 1x=%llu 2x=%llu 3x=%llu 4x=%llu "
                            "5x=%llu 6x=%llu 7x=%llu",
                            (unsigned long long)portr_total[0],
                            (unsigned long long)portr_total[1],
                            (unsigned long long)portr_total[2],
                            (unsigned long long)portr_total[3],
                            (unsigned long long)portr_total[4],
                            (unsigned long long)portr_total[5],
                            (unsigned long long)portr_total[6],
                            (unsigned long long)portr_total[7]);
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
            uint16_t mmr = op & 0x7F;
            data_write(s, mmr, data_read(s, op2));
            return consumed + s->lk_used;
        }
        /* 87xx: MVMD MMR, dmad */
        if (hi8 == 0x87) {
            op2 = prog_fetch(s, s->pc + 1);
            consumed = 2;
            uint16_t mmr = op & 0x7F;
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
        /* 97xx: ST #lk, Smem (2-word). 0x96xx is caught above as MVDP. */
        if (hi8 == 0x97) {
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
            uint16_t mmr = op & 0x7F;
            int64_t acc = src_acc ? s->b : s->a;
            data_write(s, mmr, (uint16_t)(acc & 0xFFFF));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xBA) {
            /* LDMM MMR, dst */
            uint16_t mmr = op & 0x7F;
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
            /* A0xx: accumulator operations — LD/NEG/ABS/NOT/SFTA/SFTL/SAT
             * Per SPRU172C:
             *   A000/A001: LD B,A / LD A,B
             *   A004/A005: NOT A / NOT B
             *   A008/A009: NEG A / NEG B
             *   A00A/A00B: ABS A / ABS B
             *   A00C/A00D: MAX A / MAX B (sat + clip)
             *   A00E/A00F: MIN A / MIN B
             *   bit7=0: SFTA dst, SHIFT — 1010 0000 0SSS SSSD (arith shift)
             *   bit7=1: SFTL dst, SHIFT — 1010 0000 1SSS SSSD (logical shift)
             *   A098/A099: SAT A / SAT B
             */
            uint8_t sub = op & 0xFF;
            if (sub == 0x00) { s->a = s->b; }
            else if (sub == 0x01) { s->b = s->a; }
            else if (sub == 0x04) { s->a = sext40(~s->a); } /* NOT A */
            else if (sub == 0x05) { s->b = sext40(~s->b); } /* NOT B */
            else if (sub == 0x08) { s->a = sext40(-s->a); } /* NEG A */
            else if (sub == 0x09) { s->b = sext40(-s->b); } /* NEG B */
            else if (sub == 0x0A) { s->a = sext40((s->a < 0) ? -s->a : s->a); } /* ABS A */
            else if (sub == 0x0B) { s->b = sext40((s->b < 0) ? -s->b : s->b); } /* ABS B */
            else if (sub == 0x98) { /* SAT A */
                if (s->a > 0x7FFFFFFFFFLL) s->a = 0x7FFFFFFFFFLL;
                else if (s->a < -0x8000000000LL) s->a = -0x8000000000LL;
                s->st0 &= ~ST0_OVA;
            }
            else if (sub == 0x99) { /* SAT B */
                if (s->b > 0x7FFFFFFFFFLL) s->b = 0x7FFFFFFFFFLL;
                else if (s->b < -0x8000000000LL) s->b = -0x8000000000LL;
                s->st0 &= ~ST0_OVB;
            }
            else if (sub & 0x80) {
                /* SFTL dst, SHIFT — logical shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                uint64_t u = (uint64_t)(*acc) & 0xFFFFFFFFFFULL;
                if (shift >= 0) *acc = sext40((int64_t)(u << shift));
                else            *acc = sext40((int64_t)(u >> (-shift)));
            }
            else if (sub >= 0x10) {
                /* SFTA dst, SHIFT — arithmetic shift, bits[6:1]=shift, bit[0]=dst */
                int shift = (sub >> 1) & 0x3F;
                if (shift & 0x20) shift |= ~0x3F;  /* sign-extend 6-bit */
                int dst = sub & 1;
                int64_t *acc = dst ? &s->b : &s->a;
                if (shift >= 0) *acc = sext40(*acc << shift);
                else            *acc = sext40(*acc >> (-shift));
            }
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
            uint16_t mmr = op & 0x7F;
            s->sp--;
            data_write(s, s->sp, data_read(s, mmr));
            return consumed + s->lk_used;
        }
        if (hi8 == 0xCD) {
            /* POPM MMR */
            uint16_t mmr = op & 0x7F;
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
        /* Replay any interrupt that fired while INTM=1.
         * c54x_interrupt_ex sets IFR but does nothing else when INTM=1;
         * the real C54x re-evaluates pending interrupts every cycle, so
         * as soon as INTM clears (via RETE or RSBX INTM) a pending
         * BRINT0/TINT0/... must dispatch. Without this, a BRINT0 that
         * arrived inside another ISR is lost and the FB correlator never
         * receives its I/Q samples (d_fb_det stays 0). */
        if (!(s->st1 & ST1_INTM)) {
            uint16_t pending = s->ifr & s->imr;
            if (pending) {
                int imr_bit = __builtin_ctz(pending);
                int vec = imr_bit + 16;
                s->ifr &= ~(1 << imr_bit);
                s->sp--;
                data_write(s, s->sp, s->pc);
                if (s->pmst & PMST_APTS) {
                    s->sp--;
                    data_write(s, s->sp, s->xpc);
                }
                s->st1 |= ST1_INTM;
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                s->pc = (iptr * 0x80) + vec * 4;
                static int pending_log = 0;
                if (pending_log < 20) {
                    C54_LOG("PENDING IRQ replay vec=%d bit=%d PC->0x%04x SP=0x%04x insn=%u",
                            vec, imr_bit, s->pc, s->sp, s->insn_count);
                    pending_log++;
                }
            }
        }

        /* Record PC in ring buffer */
        pc_ring[pc_ring_idx & 255] = s->pc;
        pc_ring_idx++;

        /* SP-WATCH: log every transition where SP enters / leaves the
         * API mailbox region [0x0800..0x08FF]. This pinpoints the exact
         * instruction that corrupts the stack pointer so we don't have
         * to keep recoding to investigate. */
        {
            static uint16_t prev_sp = 0xFFFF;
            bool was_in = (prev_sp >= 0x0800 && prev_sp < 0x0900);
            bool is_in  = (s->sp  >= 0x0800 && s->sp  < 0x0900);
            if (was_in != is_in) {
                fprintf(stderr,
                        "[c54x] SP-WATCH %s SP=0x%04x (prev=0x%04x) "
                        "PC=0x%04x op=0x%04x insn=%u\n",
                        is_in ? "ENTER api" : "LEAVE api",
                        s->sp, prev_sp, s->pc, s->prog[s->pc], s->insn_count);
            }
            prev_sp = s->sp;
        }

        /* TRACE: dump entry into 0xe260 loop (first 5 hits) */
        if (s->pc == 0xe260 || s->pc == 0xe261) {
            static int e260_log = 0;
            if (e260_log < 5) {
                e260_log++;
                C54_LOG("E260-ENTRY #%d PC=0x%04x AR2=%04x AR5=%04x BRC=%d RSA=%04x REA=%04x rptb=%d IMR=%04x SP=%04x insn=%u",
                        e260_log, s->pc, s->ar[2], s->ar[5], s->brc, s->rsa, s->rea, s->rptb_active, s->imr, s->sp, s->insn_count);
                int idx = pc_ring_idx;
                char buf[1024]; int o = 0;
                for (int i = 50; i >= 1; i--) {
                    o += snprintf(buf+o, sizeof(buf)-o, "%04x ", pc_ring[(idx-i)&255]);
                }
                C54_LOG("E260-PCRING (last 50): %s", buf);
                /* Dump runtime opcodes 0xe255..0xe28f */
                char ob[1024]; int oo = 0;
                for (uint16_t a = 0xe255; a <= 0xe28f; a++) {
                    oo += snprintf(ob+oo, sizeof(ob)-oo, "%04x ", s->prog[a]);
                }
                C54_LOG("E260-PROG[e255..e28f]: %s", ob);
            }
        }

        /* CALA loop tracer: dump A and SP at PC=0xd24e and 0xd250 (first 40) */
        if (s->pc == 0xd24e || s->pc == 0xd250) {
            static int cala_log = 0;
            if (cala_log++ < 40) {
                C54_LOG("CALA-TRACE PC=0x%04x A=%08x SP=0x%04x BRC=%d AR2=%04x AR3=%04x AR4=%04x AR5=%04x insn=%u",
                        s->pc, (uint32_t)(s->a & 0xFFFFFFFF), s->sp, s->brc,
                        s->ar[2], s->ar[3], s->ar[4], s->ar[5], s->insn_count);
            }
        }

        /* PC histogram: count visits per PC, dump top 20 every 2M insns */
        {
            static uint32_t pc_hist[0x10000];
            static uint64_t hist_last_dump = 0;
            pc_hist[s->pc]++;
            if (s->insn_count - hist_last_dump >= 2000000) {
                hist_last_dump = s->insn_count;
                /* find top 20 */
                uint32_t top_cnt[20] = {0};
                uint16_t top_pc[20] = {0};
                for (int i = 0; i < 0x10000; i++) {
                    uint32_t c = pc_hist[i];
                    if (c == 0) continue;
                    for (int j = 0; j < 20; j++) {
                        if (c > top_cnt[j]) {
                            for (int k = 19; k > j; k--) {
                                top_cnt[k] = top_cnt[k-1];
                                top_pc[k] = top_pc[k-1];
                            }
                            top_cnt[j] = c;
                            top_pc[j] = (uint16_t)i;
                            break;
                        }
                    }
                }
                C54_LOG("PC HIST insn=%u top: %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        s->insn_count,
                        top_pc[0], top_cnt[0], top_pc[1], top_cnt[1], top_pc[2], top_cnt[2],
                        top_pc[3], top_cnt[3], top_pc[4], top_cnt[4], top_pc[5], top_cnt[5],
                        top_pc[6], top_cnt[6], top_pc[7], top_cnt[7], top_pc[8], top_cnt[8],
                        top_pc[9], top_cnt[9]);
                C54_LOG("PC HIST cont:        %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u %04x:%u",
                        top_pc[10], top_cnt[10], top_pc[11], top_cnt[11], top_pc[12], top_cnt[12],
                        top_pc[13], top_cnt[13], top_pc[14], top_cnt[14], top_pc[15], top_cnt[15],
                        top_pc[16], top_cnt[16], top_pc[17], top_cnt[17], top_pc[18], top_cnt[18],
                        top_pc[19], top_cnt[19]);
                memset(pc_hist, 0, sizeof(pc_hist));
            }
        }

        /* Track SP changes inside RPTB loops */
        uint16_t sp_before = s->sp;

        /* Trace EB04 loop — dump first 20 iterations */
        if (s->pc == 0xEB04) {
            static int eb04_log = 0;
            if (eb04_log < 20) {
                C54_LOG("EB04 op=%04x A=0x%010llx B=0x%010llx T=%04x "
                        "INTM=%d IMR=%04x IFR=%04x rptb=%d RSA=%04x REA=%04x BRC=%d "
                        "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->t,
                        !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                        s->rptb_active, s->rsa, s->rea, s->brc,
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                eb04_log++;
            }
        }

        /* Dump DSP state when stuck — triggers once after 500M instructions
         * if DSP hasn't reached IDLE yet */
        {
            static int dumped = 0;
            if (s->insn_count > 500000000 && !dumped && !s->idle) {
                dumped = 1;
                C54_LOG("DSP NO-IDLE dump at insn=%u PC=0x%04x:", s->insn_count, s->pc);
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  IMR=0x%04x IFR=0x%04x rptb=%d RSA=0x%04x REA=0x%04x BRC=%d",
                        s->imr, s->ifr, s->rptb_active, s->rsa, s->rea, s->brc);
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x XPC=%d",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t, s->xpc);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
                /* Dump code around current PC (using prog_fetch for correct OVLY) */
                C54_LOG("  Code around PC:");
                for (int i = -4; i < 16; i++) {
                    uint16_t a = s->pc + i;
                    C54_LOG("  %c [0x%04x] = 0x%04x",
                            i == 0 ? '>' : ' ', a, prog_fetch(s, a));
                }
                C54_LOG("  ST0=0x%04x ST1=0x%04x PMST=0x%04x SP=0x%04x INTM=%d",
                        s->st0, s->st1, s->pmst, s->sp, !!(s->st1 & ST1_INTM));
                C54_LOG("  A=0x%010llx B=0x%010llx T=0x%04x",
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL), s->t);
                C54_LOG("  AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x",
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5], s->ar[6], s->ar[7]);
            }
        }

        /* BSP read entry points — these functions contain PORTR PA=0xF430
         * (read BSP sample). If DSP never visits them, the FB-det chain is
         * dead. Targets identified by static analysis of PROM0 callers of
         * the 64 PORTR PA=0xF430 sites at 0x9b80+. */
        if (!s->rpt_active &&
            (s->pc == 0x9a78 || s->pc == 0x9aaf || s->pc == 0x9ad3 ||
             s->pc == 0x9b4c || s->pc == 0x8811)) {
            static unsigned bsp_visits[5];
            int idx = (s->pc == 0x9a78) ? 0 :
                      (s->pc == 0x9aaf) ? 1 :
                      (s->pc == 0x9ad3) ? 2 :
                      (s->pc == 0x9b4c) ? 3 : 4;
            if (bsp_visits[idx] < 5) {
                bsp_visits[idx]++;
                C54_LOG("BSP-ENTRY PC=0x%04x  A=0x%010llx ar0=%04x ar1=%04x "
                        "ar2=%04x ar3=%04x ar4=%04x SP=0x%04x insn=%u",
                        s->pc,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3], s->ar[4],
                        s->sp, s->insn_count);
            }
        }

        /* Trace any write touching the dispatcher poll addresses
         * data[0x4359] / data[0x3fab]. We never see them go non-zero;
         * confirm whether ANY code path writes them. */
        /* (handled in data_write — see below) */

        /* Dispatcher hot loop trace at PROM0 0xb968-0xb9a4 — the state
         * machine the DSP spins in when waiting for ARM tasks. Logs the
         * first 8 visits per PC so we see the full conditional structure
         * (which addresses it polls, which constants it compares to). */
        if (s->pc >= 0xb968 && s->pc <= 0xb9a4 && !s->rpt_active) {
            static uint8_t disp_visits[64];
            int idx = s->pc - 0xb968;
            if (idx >= 0 && idx < 64 && disp_visits[idx] < 8) {
                disp_visits[idx]++;
                C54_LOG("DISP-TRACE PC=0x%04x op=0x%04x A=0x%010llx "
                        "B=0x%010llx ar0=%04x ar1=%04x ar2=%04x ar3=%04x "
                        "ar4=%04x ar5=%04x TC=%d",
                        s->pc, prog_fetch(s, s->pc),
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                        s->ar[4], s->ar[5],
                        !!(s->st0 & ST0_TC));
            }
        }

        /* IRQ vec area trace: log every PC visit in 0xFFCC-0xFFE0
         * (INT3 + TINT0 + BRINT0 vec slots). Captures the 3 actual
         * 4-word handlers our IRQ INT3 dispatch lands on at IPTR=0x1ff.
         * 80 unique PCs max, log first 4 visits each. */
        if (s->pc >= 0xFFCC && s->pc < 0xFFE0 && !s->rpt_active) {
            static uint8_t vec_visits[20];   /* index 0 = 0xffcc */
            int idx = s->pc - 0xFFCC;
            if (vec_visits[idx] < 4) {
                vec_visits[idx]++;
                C54_LOG("VEC-TRACE PC=0x%04x op=0x%04x SP=0x%04x A=0x%010llx "
                        "B=0x%010llx TC=%d INTM=%d ar7=%04x",
                        s->pc, prog_fetch(s, s->pc), s->sp,
                        (unsigned long long)(s->a & 0xFFFFFFFFFFLL),
                        (unsigned long long)(s->b & 0xFFFFFFFFFFLL),
                        !!(s->st0 & ST0_TC),
                        !!(s->st1 & ST1_INTM),
                        s->ar[7]);
            }
        }

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
        /* Check RPTB (block repeat) — skip during RPT (single repeat has priority).
         * Use >= instead of == : if the last instruction in the block is a
         * 2-word instruction (e.g. LD *(lk)), PC jumps by 2 and overshoots
         * REA+1. The real C54x pipeline catches this; we emulate with >=. */
        if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1) {
            static int rptb_log = 0;
            if (rptb_log < 20) { C54_LOG("RPTB redirect PC=0x%04x→RSA=0x%04x REA=0x%04x BRC=%d", s->pc, s->rsa, s->rea, s->brc); rptb_log++; }
            if (s->brc > 0) {
                s->brc--;
                s->pc = s->rsa;
            } else {
                s->rptb_active = false; { static int _re=0; if (_re<50) { C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp); _re++; } }
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

        /* Delayed-branch slot countdown.
         * RCD (and later CALLD/RETD/BD/CCD if extended) sets delayed_pc and
         * delay_slots = 2. The two instructions following the RCD execute
         * as normal pipeline slots; once both have completed the branch
         * commits by forcing PC to delayed_pc. */
        if (s->delay_slots > 0) {
            s->delay_slots--;
            if (s->delay_slots == 0) {
                s->pc = s->delayed_pc;
            }
        }

        s->cycles++;
        s->insn_count++;

        /* === DIAGNOSTIC HACK — TEMPORARY — REMOVE ASAP ===
         * Force-clear INTM at insn_count = $CALYPSO_FORCE_INTM_CLEAR_AT.
         * Discriminates the catch-22 hypothesis: if clearing INTM here
         * unblocks the FB-det path (RETED ≥1, d_fb_det written, FB1/FB2
         * print), the root cause is "boot init never executes RSBX INTM";
         * then we hunt the missing path (α/β/γ). If no change → INTM is
         * not the only blocker.
         *
         * Documented in hw/arm/calypso/doc/TODO.md "DIAGNOSTIC HACK" —
         * MUST BE REMOVED once the real RSBX path is identified.
         * Disabled by default (env var unset). */
        {
            static int hack_at = -2;
            static int hack_done = 0;
            if (hack_at == -2) {
                const char *e = getenv("CALYPSO_FORCE_INTM_CLEAR_AT");
                hack_at = e ? atoi(e) : -1;
                if (hack_at > 0)
                    C54_LOG("DIAG-HACK armed: will clear INTM at insn=%d", hack_at);
            }
            if (hack_at > 0 && !hack_done && (uint32_t)hack_at == s->insn_count) {
                uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
                uint32_t vec_base = (uint32_t)iptr << 7;
                C54_LOG("DIAG-HACK *** FIRE *** insn=%u PC=0x%04x SP=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d PMST=0x%04x IPTR=0x%03x MP_MC=%d OVLY=%d DROM=%d APTS=%d IMR=0x%04x IFR=0x%04x XPC=%d",
                        s->insn_count, s->pc, s->sp, s->st0, s->st1,
                        !!(s->st1 & ST1_INTM),
                        s->pmst, iptr,
                        !!(s->pmst & PMST_MP_MC),
                        !!(s->pmst & PMST_OVLY),
                        !!(s->pmst & PMST_DROM),
                        !!(s->pmst & PMST_APTS),
                        s->imr, s->ifr, s->xpc);
                /* Dump vec table at IPTR-derived base : reset, INT3 (vec19),
                 * TINT0 (vec20), BRINT0 (vec21), and a few more for context. */
                struct { const char *n; int v; } vecs[] = {
                    {"reset", 0}, {"NMI", 1}, {"INT0", 16}, {"INT1", 17},
                    {"INT2", 18}, {"INT3", 19}, {"TINT0", 20}, {"BRINT0", 21},
                    {"BXINT0", 22}, {"BRINT1", 23},
                };
                for (size_t i = 0; i < sizeof(vecs)/sizeof(vecs[0]); i++) {
                    uint32_t a = vec_base + (uint32_t)vecs[i].v * 4;
                    uint16_t w0 = prog_read(s, a + 0);
                    uint16_t w1 = prog_read(s, a + 1);
                    uint16_t w2 = prog_read(s, a + 2);
                    uint16_t w3 = prog_read(s, a + 3);
                    fprintf(stderr,
                            "[c54x] DIAG-HACK   vec%2d %-6s @ 0x%04x : %04x %04x %04x %04x\n",
                            vecs[i].v, vecs[i].n, (uint16_t)a, w0, w1, w2, w3);
                }
                /* Aliasing diagnostic: compare s->data[0x10c0..0x10F8] vs
                 * api_ram[0x8c0..0x8F8] (same DSP-word, two views).
                 * Diverging values prove ARM/DSP write paths populate
                 * different backing stores. */
                C54_LOG("DIAG-HACK ALIAS-CHECK DARAM 0x10c0-0x10f8 (s->data vs api_ram):");
                int diverge_count = 0;
                for (uint16_t a = 0x10c0; a <= 0x10f8; a++) {
                    uint16_t vd = s->data[a];
                    uint16_t va = s->api_ram ? s->api_ram[a - 0x0800] : 0xDEAD;
                    if (vd != va) {
                        if (diverge_count < 20)
                            fprintf(stderr,
                                    "[c54x] DIAG-HACK   DIVERGE @0x%04x: s->data=0x%04x api_ram=0x%04x\n",
                                    a, vd, va);
                        diverge_count++;
                    }
                }
                C54_LOG("DIAG-HACK   total diverging slots in 0x10c0-0x10f8: %d / %d", diverge_count, 0x10f8 - 0x10c0 + 1);
                /* Sample DARAM bootloader-mailbox slots too (0x0FFC-0x0FFF) */
                for (uint16_t a = 0x0FFC; a <= 0x0FFF; a++) {
                    uint16_t vd = s->data[a];
                    uint16_t va = s->api_ram ? s->api_ram[a - 0x0800] : 0xDEAD;
                    fprintf(stderr,
                            "[c54x] DIAG-HACK   mailbox @0x%04x: s->data=0x%04x api_ram=0x%04x %s\n",
                            a, vd, va, (vd == va) ? "OK" : "*** DIVERGE ***");
                }
                /* CALAD source slot 0x4189 (out of API range, DSP-only).
                 * Confirms it stays zero — the LD-A-TRACE feeds A=0 to CALAD. */
                fprintf(stderr,
                        "[c54x] DIAG-HACK   CALAD-source @0x4189: s->data=0x%04x (api range ends at 0x27FF, so api_ram NA)\n",
                        s->data[0x4189]);
                /* Also sample 0x41a0-0x41ab where we saw DSP writes earlier */
                for (uint16_t a = 0x41a0; a <= 0x41ab; a++) {
                    fprintf(stderr,
                            "[c54x] DIAG-HACK   CALAD-zone @0x%04x: s->data=0x%04x\n",
                            a, s->data[a]);
                }
                if (s->st1 & ST1_INTM) {
                    s->st1 &= ~ST1_INTM;
                    C54_LOG("DIAG-HACK *** INTM cleared *** ST1=0x%04x", s->st1);
                } else {
                    C54_LOG("DIAG-HACK INTM already 0, no clear needed");
                }
                hack_done = 1;
            }
        }

        /* One-shot diagnostic at boot+: dump 0xB900 vector table
         * (the relocated table the firmware should use if it sets
         * IPTR=0x172) plus DSP runtime state. Helps diagnose why the
         * IPTR relocation never fires in some runs. */
        if (s->insn_count == 100000) {
            uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
            C54_LOG("BOOT+100k STATE: PMST=0x%04x IPTR=0x%03x IMR=0x%04x IFR=0x%04x ST0=0x%04x ST1=0x%04x INTM=%d PC=0x%04x SP=0x%04x XPC=%d",
                    s->pmst, iptr, s->imr, s->ifr, s->st0, s->st1,
                    !!(s->st1 & ST1_INTM), s->pc, s->sp, s->xpc);
            C54_LOG("BOOT+100k VECDUMP-FORCED base=0xB900 (32 vectors, alt table @ IPTR=0x172):");
            for (int vec = 0; vec < 32; vec++) {
                uint32_t a = 0xB900 + vec * 4;
                uint16_t w0 = prog_read(s, a + 0);
                uint16_t w1 = prog_read(s, a + 1);
                uint16_t w2 = prog_read(s, a + 2);
                uint16_t w3 = prog_read(s, a + 3);
                fprintf(stderr,
                        "[c54x] alt vec %2d @ 0x%04x : %04x %04x %04x %04x\n",
                        vec, (uint16_t)a, w0, w1, w2, w3);
            }
        }

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
    s->sp = 0x5AC8; s->bk = 0;  /* SP init per Calypso boot ROM
                                 * NOTE: silicon dumps show SP=0x1100 post-handshake.
                                 * 0x5AC8 is a shortcut anticipating osmocom firmware re-init.
                                 * See doc/datasheets/README.md §3-4. */
    s->brc = 0; s->rsa = 0; s->rea = 0;
    /* MMR reset values aligned with Calypso silicon (3 FreeCalypso ROM dumps + local).
     * Empirically validated 2026-04-28. See doc/datasheets/README.md §3.
     * Previous QEMU values (st0=0, st1=ST1_INTM, pmst=0xFFE0) were partial. */
    s->st0  = 0x181F;                              /* DP=0x01F per silicon */
    s->st1  = ST1_INTM | ST1_SXM | ST1_XF;         /* 0x2900: INTM=1, SXM=1, XF=1 */
    s->pmst = 0xFFA8;                              /* IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1 */
    s->imr = 0;
    s->ifr = 0;
    s->xpc = 0;
    s->timer_psc = 0;
    s->data[TCR_ADDR] = TCR_TSS;  /* Timer stopped at reset (TSS=1) per HW spec */
    s->data[TIM_ADDR] = 0xFFFF;   /* TIM = max at reset */
    s->data[PRD_ADDR] = 0xFFFF;   /* PRD = max at reset */
    s->rpt_active = false;
    s->rptb_active = false; { static int _re=0; if (_re<50) { C54_LOG("RPTB EXIT PC=0x%04x RSA=0x%04x REA=0x%04x insn=%u SP=0x%04x", s->pc, s->rsa, s->rea, s->insn_count, s->sp); _re++; } }
    s->idle = false;
    s->running = true;
    s->cycles = 0;
    s->insn_count = 0;
    s->unimpl_count = 0;

    /* Boot ROM MVPD: copy PROM0 code to DARAM overlay.
     * On real Calypso, the internal boot ROM copies PROM0[0x7080..0x9FFF]
     * to DARAM data[0x0080..0x27FF] before jumping to user code.
     * This populates the DARAM code overlay that the DSP executes with OVLY=1.
     *
     * On real silicon, DARAM and API RAM share one physical memory in the
     * range 0x0800-0x27FF (DSP-words). Mirror the copy into api_ram so the
     * ARM-side view matches the DSP-side view from boot — without this
     * mirror, every ARM read into the overlay zone returns 0 while the
     * DSP executes the copied code, which silently splits the two views. */
    for (int i = 0; i < 0x2780; i++) {
        uint16_t addr = 0x0080 + i;
        uint16_t val = s->prog[0x7080 + i];
        s->data[addr] = val;
        if (s->api_ram &&
            addr >= C54X_API_BASE && addr < C54X_API_BASE + C54X_API_SIZE)
            s->api_ram[addr - C54X_API_BASE] = val;
    }

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
     * Discriminant test 2026-04-26 confirmed FRET stub did NOT block the
     * firmware path to 0x0810 (reverting to NOPs gave identical PC HIST
     * + same IMR change=0). FRET stub kept: prevents stack runaway when
     * CALAA targets the stub area, with no downside.
     *
     * Fallback per slot:
     *   - 0x0000: LDMM SP, B (real boot ROM behaviour)
     *   - 0x0001: RET (paired with the CALL at 0x770A)
     *   - rest:   FRET (0xF4E4) — return immediately to caller. */
    for (int i = 0; i < 0x80; i++)
        s->prog[i] = 0xF4E4;  /* FRET fallback — return-from-far */
    s->prog[0x0000] = 0xBA18;  /* LDMM SP, B */
    s->prog[0x0001] = 0xFC00;  /* RET */

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

    /* Log interrupts: first 20 + every 100th, so we can count them.
     * PMST/IPTR included so we can correlate which vector base the IRQ
     * lands at — INT3 at IPTR=0x1ff (vec=0xffcc) hits a garbage ROM stub,
     * INT3 at IPTR=0x140 (vec=0xa04c) hits the firmware's real handler. */
    static uint64_t int_log_count;
    int_log_count++;
    if (int_log_count <= 20 || (int_log_count % 100) == 0) {
        uint16_t iptr_now = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
        C54_LOG("IRQ #%llu vec=%d bit=%d: INTM=%d IMR=0x%04x IFR=0x%04x "
                "idle=%d PC=0x%04x PMST=0x%04x IPTR=0x%03x",
                (unsigned long long)int_log_count,
                vec, imr_bit, !!(s->st1 & ST1_INTM), s->imr, s->ifr,
                s->idle, s->pc, s->pmst, iptr_now);
    }
}

void c54x_wake(C54xState *s)
{
    s->idle = false;
}

void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n)
{
    if (n > 2048) n = 2048;
    memcpy(s->bsp_buf, samples, n * sizeof(uint16_t));
    s->bsp_len = n;
    s->bsp_pos = 0;

    /* Confirm what the PORTR PA=0x0034 serving path will hand the DSP,
     * and also flag if the DSP consumed less than half of the previous
     * batch before a new one arrived (would indicate correlator starvation
     * or DSP never reading via PORTR at all). */
    static uint64_t load_count;
    load_count++;
    if (load_count <= 10 || (load_count % 1000) == 0) {
        C54_LOG("BSP LOAD #%llu n=%d: %04x %04x %04x %04x %04x %04x %04x %04x",
                (unsigned long long)load_count, n,
                n > 0 ? samples[0] : 0, n > 1 ? samples[1] : 0,
                n > 2 ? samples[2] : 0, n > 3 ? samples[3] : 0,
                n > 4 ? samples[4] : 0, n > 5 ? samples[5] : 0,
                n > 6 ? samples[6] : 0, n > 7 ? samples[7] : 0);
    }
}
