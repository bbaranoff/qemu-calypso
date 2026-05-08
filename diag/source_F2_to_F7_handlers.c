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
