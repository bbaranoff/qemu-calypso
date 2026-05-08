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
        /* POPM MMR — pop top-of-stack into MMR (1-word).
         * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
         * Per SPRU172C section 4 : value at SP popped to MMR, SP++.
         *
         * Bug fix 2026-05-08 : 0x8Axx était précédemment mal décodé en
         * MVDK Smem,dmad (qui est en réalité 0x7100 mask 0xFF00). Le
         * pattern PSHM/POPM symétrique du firmware (e.g. PROM0 0x7013-0x7023
         * sauve/restaure 6 MMRs autour d'un CALA) ne fonctionnait jamais
         * post-CALA → ST1 jamais restauré → INTM=1 dwell perpétuel
         * → IRQ vectoring bloqué → DSP wait stuck → L1 mort.
         * Le case MVDK ci-dessous devient dead code mais est laissé pour
         * référence historique. */
        if ((op & 0xFF00) == 0x8A00) {
            uint16_t mmr = op & 0x7F;
            uint16_t val = data_read(s, s->sp);
            s->sp = (s->sp + 1) & 0xFFFF;
            data_write(s, mmr, val);
            return consumed + s->lk_used;
        }
        /* OBSOLETE — superseded by POPM above. The 0x8Axx range belongs to
         * POPM per tic54x-opc.c, not MVDK (which is 0x7100 mask 0xFF00).
         * Kept commented for one revision so any caller depending on the
         * old (incorrect) behaviour is forced to be re-examined. */
        if (0 && hi8 == 0x8A) {
            /* MVDK Smem, dmad — INCORRECT for 0x8Axx, see POPM above */
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
            /* STUB-NOP : tic54x dit 0x80 = STL src,Smem (1-word).
             * Ancienne classification qemu = MVDD 2-word (incorrect).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé pour éviter
             * les écritures mémoire fantômes en attendant impl correcte. */
            return 1;
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
            /* STUB-NOP : tic54x dit 0x8B = POPD Smem (1-word).
             * Ancienne classification qemu = MVDK long-addr 2-word (incorrect).
             * Voir doc/opcodes/tic54x_hi8_map.md. Neutralisé. */
            return 1;
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
