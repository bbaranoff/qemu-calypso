# C54x DSP Emulator - All Bugs and Fixes

## Session 2026-04-05 (this context)

### Bug 1: SP leak via RPTB EXIT
- **Symptom**: SP incremented +1 at each RPTB EXIT (0xe999->0xe9b0)
- **Root cause**: RPTB itself was not the issue, the SP leak came from other instructions
- **Fix**: Added SP change detector to identify culprit instructions

### Bug 2: SP corruption via STH B writing to MMR zone
- **Symptom**: SP drops from 0x5AC8 to 0x0000 at PC=0x8a46
- **Root cause**: STH B with DP=0 writes B(high)=0 to addr 0x18 = SP register (MMR)
- **Fix**: Set SP init to 0x5AC8 (Calypso boot ROM value). Later fixed properly via CALLD/RPTBD corrections.

### Bug 3: PROM1 mirror stopping at 0xFF7F
- **Symptom**: Interrupt vectors at 0xFF80-0xFFFF empty
- **Root cause**: Mirror range was `addr16 >= 0xE000 && addr16 < 0xFF80`
- **Fix**: Changed to `addr16 >= 0xE000` (include vectors)

### Bug 4: Interrupt vectors 1-7 overwritten by IDLE stubs
- **Symptom**: TINT0 and other interrupts branch to IDLE instead of real handlers
- **Root cause**: c54x_reset() installed IDLE at vec 1-7 positions, overwriting PROM1 ROM vectors
- **Fix**: Only install vec 0 (reset). Vec 1-7 come from PROM1 ROM mirror.

### Bug 5: F272 decoded as NORM instead of RPTBD
- **Symptom**: Code at 0x76FB consumed 1 word instead of 2, corrupting instruction flow
- **Root cause**: F272 = RPTBD (repeat block delayed, 2 words) per tic54x-opc.c. Was incorrectly decoded as NORM (normalize accumulator).
- **Fix**: Added specific checks for F272=RPTBD, F274=CALLD, F273=RETD before LMS handler

### Bug 6: F274 decoded as NORM instead of CALLD
- **Symptom**: Code at 0x7702 treated as normalize instead of delayed call
- **Root cause**: Same as Bug 5 - F274 is CALLD pmad (delayed call, 2 words)
- **Fix**: F274 now pushes PC+2 and branches to pmad

### Bug 7: Indirect addressing modes 0xC-0xF not consuming 2nd word
- **Symptom**: Instructions with `*(lk)`, `*+ARn(lk)`, `*ARn(lk)`, `*+ARn(lk)%` used AR as address without offset, and the offset word was executed as next instruction
- **Root cause**: resolve_smem had `/* handled by caller */` for modes C-F but no caller actually handled them
- **Fix**: resolve_smem now reads prog_fetch(s, s->pc+1) for modes C-F, sets s->lk_used=true. All `return consumed` changed to `return consumed + s->lk_used`.

### Bug 8: BANZ testing old AR value
- **Symptom**: BANZ at 0xB36B branching incorrectly
- **Root cause**: Code saved `old_arp_val = s->ar[arp(s)]` BEFORE resolve_smem modified AR, then tested old value
- **Fix**: Test `s->ar[arp(s)]` AFTER resolve_smem

### Bug 9: F6xx instructions treated as NOP
- **Symptom**: MVDD (data-to-data move) silently dropped
- **Root cause**: F6xx handler only had cases for sub=2 and sub=6, rest fell to NOP
- **Fix**: Added MVDD for sub >= 0x8

### Bug 10: RPT/RPTB interaction
- **Symptom**: RPT inside RPTB causes infinite loop - RPTB redirect fires during RPT
- **Root cause**: RPTB check (`pc == rea + 1`) runs before exec and redirects PC during active RPT
- **Fix**: Skip RPTB check when `rpt_active` is true

### Bug 11: RPT F5xx return value
- **Symptom**: Changing RPT from `s->pc += 1; return 0` to `return 1` caused RPT to re-execute itself infinitely
- **Root cause**: RPT handler in main loop does `continue` (skips pc += consumed). With return 1, PC never advances past RPT instruction.
- **Fix**: Reverted to `s->pc += 1; return 0` - RPT must advance PC itself since the main loop's RPT handler will prevent further PC advance.

### Bug 12: NORM bit extraction
- **Symptom**: NORM checking wrong bits of accumulator
- **Root cause**: Was checking bits 31/30 of 24-bit masked value instead of bits 39/38 of 40-bit accumulator
- **Fix**: Changed to `(*acc >> 39) & 1` and `(*acc >> 38) & 1`

### Bug 13: L1CTL RESET_IND lost before client connects
- **Symptom**: ccch_scan connects but never receives RESET_IND
- **Root cause**: QEMU boots immediately, firmware sends RESET_IND before client is listening
- **Fix**: QEMU starts with `-S` (paused). `vm_start()` called when client connects.

### Bug 14: L1CTL reconnect stuck at "waiting for firmware"
- **Symptom**: Kill and relaunch mobile -> stuck at "waiting for firmware"
- **Root cause**: `cli_rx_enabled = false` on reconnect, firmware won't re-send RESET_IND
- **Fix**: On reconnect (VM already running), enable `cli_rx_enabled` immediately

### Bug 15: soft_to_int16 overflow in sercomm_udp.py
- **Symptom**: soft_bit=255 produces -33025 (outside int16 range)
- **Root cause**: No clamping
- **Fix**: `max(-32768, min(32767, ...))`

### Bug 16: Boot ROM at prog[0x0000-0x007F] empty
- **Symptom**: CALA to A(low)=0x0000 executes garbage (all zeros)
- **Root cause**: Internal Calypso boot ROM not in PROM dump. prog[0x0000] was uninitialized.
- **Fix**: Fill prog[0x0000-0x007F] with RET (0xF073) stubs

### Bug 17: Duplicate F0/F1 handler blocks
- **Symptom**: Second F0/F1 handler block is dead code, never reached
- **Root cause**: First `if (hi8 == 0xF0 || hi8 == 0xF1)` block catches everything
- **Status**: Identified, not yet cleaned up

### Bug 18: TINT0 timer scattered across files
- **Symptom**: Timer logic in calypso_trx.c mixed with DSP/ARM glue code
- **Root cause**: No dedicated timer module
- **Fix**: Created calypso_tint0.c/h as master clock module. Removed calypso_tdma_hw.c/h (slave, never connected).

## Session 2026-04-05 night

### Bug 19: F4EB = RETE (alternate encoding)
- **Symptom**: Interrupt vector stubs (F4EB F495 F495 F495) fell through to BD handler, branching to 0xF495
- **Root cause**: F4EB not recognized as RETE. Per tic54x-opc.c it's the real RETE encoding.
- **Fix**: Added exact-match F4EB handler with APTS-aware pop

### Bug 20: RPTBD (F272) RSA = PC+2 instead of PC+4
- **Symptom**: Delay slots executed inside the repeat loop instead of once before it. Init MVDD sweep ran 64x corrupting MMRs. Dispatch RPTB at 0x8F8B included STM in loop body.
- **Root cause**: RPTBD is delayed — 2 delay slots after the 2-word instruction. RSA must be PC+4, not PC+2.
- **Fix**: Changed `s->rsa = (uint16_t)(s->pc + 4)` in both F272 handlers. THIS was the root cause of init MMR corruption and DSP never reaching IDLE.

### Bug 21: F4xx arithmetic decoded as branch/call
- **Symptom**: DSP never reached IDLE, infinite dispatch loops, stack corruption. F48E (EXP) treated as CALLD (push+branch), F483 (SAT) as unknown, F48C (MPYA) as CALLD, etc.
- **Root cause**: F4xx switch used `sub = (op >> 4) & 0xF` mapping sub=0→B, sub=6→CALL, sub=8→CALLD. But per tic54x-opc.c, F400-F4BF are arithmetic instructions (ADD, SUB, LD, SFTA, SFTL, EXP, NORM, MPYA, SAT, NEG, ABS, CMPL, RND, MAX, MIN, SUBC, ROR, ROL, MACA, SQUR).
- **Fix**: Replaced nibble switch with exact-match/mask-based handlers for all F4xx arithmetic. Added 20+ instruction implementations.

### Bug 22: MAR (0x6D) not implemented
- **Symptom**: 0x6DEC at 0xE906 treated as UNIMPL (1 word). Next word (0x0003 = lk) executed as instruction, causing PC misalignment and d_dsp_page corruption.
- **Root cause**: 0x6Dxx = MAR (Modify Address Register) per tic54x-opc.c. Was missing from decoder.
- **Fix**: Added MAR handler — calls resolve_smem for side effects only (AR modification), no data access.

### Bug 23: CC/BC condition tables wrong
- **Symptom**: Unknown conditions defaulted to `take=true`, causing wrong branches. cond=0x05 was always taken instead of AEQ (A==0).
- **Root cause**: Condition codes followed wrong mapping. Per tic54x-dis.c cc2[]: 0x02=AGEQ, 0x03=ALT, 0x04=ANEQ, 0x05=AEQ, etc.
- **Fix**: Rewrote CC and BC with correct switch table. Unknown compound conditions still default to true.

### Bug 24: prog_write allows PROM1 corruption
- **Symptom**: MVDP/WRITA could overwrite ROM at 0xE000-0xFFFF
- **Root cause**: No protection in prog_write for PROM1 range
- **Fix**: `if (addr16 >= 0xE000) return;`

### Bug 25: d_dsp_page not synced to DSP API RAM
- **Symptom**: DSP read d_dsp_page=0x0000 always, never saw ARM's 0x0002/0x0003
- **Root cause**: Sync to api_ram was inside SINT17 block (gated on dsp_init_done). Before init, DSP never saw the page value.
- **Fix**: Sync api_ram[0xD4] on ARM write to dsp_ram[0x01A8/2], not per-tick.

### Bug 26: BRINT0 not fired
- **Symptom**: DSP stuck in frame processing waiting for BSP data that never arrives
- **Root cause**: c54x_bsp_load loaded samples but never fired BRINT0 interrupt
- **Fix**: Fire c54x_interrupt_ex(dsp, 21, 5) after bsp_load, gated on dsp_init_done

### Bug 27: Double-nested include headers
- **Symptom**: include/hw/arm/calypso/calypso/ had divergent copies (GSM_TDMA_NS 10x off, missing irq_in_service)
- **Root cause**: Leftover from refactoring
- **Fix**: Removed nested calypso/calypso/ directory
