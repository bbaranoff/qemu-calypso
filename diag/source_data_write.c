static void data_write(C54xState *s, uint16_t addr, uint16_t val)
{
    /* DATA-W-MMR : log every write into the low MMR window (addr <= 0x1F)
     * with full attribution context. Goal : disambiguate the IMR-W trace
     * cascade observed at PC=0x8eb9 (op=0xf3e1) and PC=0x9ad0 (op=0x8192).
     * The writer_kind field tells us *which path* triggered the write
     * (opcode family / IRQ ack / ARM MMIO / resolve_smem side effect).
     * Cap at 200 distinct events to avoid log flood. */
    if (addr <= 0x1F) {
        static unsigned mmrw_log;
        if (mmrw_log++ < 200) {
            const char *wk_name[] = {
                "UNK", "F3", "8x", "77", "76", "PSHM",
                "RET", "IRQ_ACK", "ARM_MMIO", "RES_AR", "OTHER"
            };
            uint8_t wk = s->writer_kind;
            const char *wkn = (wk < sizeof(wk_name)/sizeof(wk_name[0]))
                              ? wk_name[wk] : "??";
            fprintf(stderr,
                    "[c54x] DATA-W-MMR addr=0x%02x val=0x%04x "
                    "exec_pc=0x%04x cur_pc=0x%04x cur_op=0x%04x "
                    "xpc=%d wk=%s "
                    "AR0=%04x AR1=%04x AR2=%04x AR3=%04x "
                    "AR4=%04x AR5=%04x AR6=%04x AR7=%04x "
                    "SP=%04x DP=%d INTM=%d insn=%u\n",
                    addr, val,
                    s->last_exec_pc, s->pc, s->prog[s->pc],
                    s->xpc, wkn,
                    s->ar[0], s->ar[1], s->ar[2], s->ar[3],
                    s->ar[4], s->ar[5], s->ar[6], s->ar[7],
                    s->sp, dp(s),
                    !!(s->st1 & ST1_INTM),
                    s->insn_count);
        }
    }

    /* WATCH-WRITE on the same mailbox slots tracked in data_read.
     * Whoever writes them — DSP or ARM via api_ram alias — gets logged
     * so we can attribute the source of the value the firmware polls. */
    /* WATCH-WRITE 0x3dd2 — la cellule sur laquelle 0x75db poll en boucle
     * (37M reads/15s). Identifier qui écrit (et qui ne le fait pas).
     * Cas 1 : zéro write → un bloc compute ne fire jamais.
     * Cas 2 : write boot only → init OK mais set steady-state manquant.
     * Cas 3 : writes périodiques avec valeur jamais matchée par le test
     *         à 0x75db → bug dans le compute en amont. */
    if (addr == 0x3dd2) {
        static unsigned w3dd2;
        w3dd2++;
        if (w3dd2 <= 100 || (w3dd2 % 1000) == 0) {
            fprintf(stderr,
                    "[c54x] WATCH-WRITE 0x3dd2 #%u <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x insn=%u INTM=%d\n",
                    w3dd2, val, s->data[addr], s->pc, s->insn_count,
                    !!(s->st1 & ST1_INTM));
        }
    }
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
    /* === DARAM[0x40..0x90] watch — dispatcher flag area ===
     * The PROM0 idle dispatcher (0xCC62..0xCC6F) polls data[0x62] and
     * other slots in [0x60..0x70]. FORCE-DARAM62=1 (env) proves that
     * setting data[0x62]=1 makes the DSP escape and reach 0x770c, so
     * this range gates the runtime task pipeline. ARM-side writes to
     * the API page mirror at +0x0800 (calypso_trx.c calypso_dsp_write)
     * but never to DARAM 0x40..0x90 — so any value here must come from
     * DSP-self stores (ST/STH/STM/...) or stay zero forever. Capture
     * EVERY write with PC+INTM+insn so we can attribute the source.
     * INTM annotation lets us tell ISR-context writes from main code. */
    if (addr >= 0x0040 && addr <= 0x0090) {
        static unsigned daram_disp_w;
        if (daram_disp_w++ < 1000) {
            fprintf(stderr,
                    "[c54x] DISP-FLAG-W data[0x%04x] <- 0x%04x (was 0x%04x) "
                    "PC=0x%04x INTM=%d IFR=0x%04x insn=%u\n",
                    addr, val, s->data[addr], s->pc,
                    !!(s->st1 & ST1_INTM), s->ifr, s->insn_count);
            if (daram_disp_w == 1000) {
                fprintf(stderr,
                        "[c54x] DISP-FLAG-W log capped at 1000 — pattern visible above\n");
            }
        }
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
