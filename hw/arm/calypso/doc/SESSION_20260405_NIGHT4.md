# Session 2026-04-05 Night 4 — C54x Opcode Audit

## Summary

Massive opcode audit of calypso_c54x.c against the authoritative `tic54x-opc.c`
from binutils-2.21.1 (found at `/var/lib/docker/overlay2/.../gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c`).

**17 bugs fixed.** DSP now boots correctly: IDLE reached, IMR configured, dispatch loop active.

## Fixes

### ALU Instructions (Fix 1)
- **F0xx was READA** → now ADD/SUB/LD/AND/OR/XOR #lk,shift,src,dst
- Also added F06x (ADD/SUB/LD #lk,16 + MPY/MAC #lk)
- Also added F08x-F0Fx (accumulator AND/OR/XOR/SFTL with shift)
- Encoding: bits 7:4=op, bits 9:8=src/dst, bits 3:0=shift, 2nd word=lk
- Source: tic54x-opc.c line 254 `{ "add", 0xF000, 0xFCF0 }`

### RSBX/SSBX (Fixes 2-5)
Per tic54x-opc.c: RSBX=0xF4B0 mask 0xFDF0, SSBX=0xF5B0 mask 0xFDF0.
This covers 4 opcode ranges:

| Fix | Opcode | Was | Now | Encoding |
|-----|--------|-----|-----|----------|
| 2 | F4Bx | NOP catch-all | RSBX ST0 | bit9=0, bit8=0 |
| 3 | F5Bx | RPT #0xBx (176+!) | SSBX ST0 | bit9=0, bit8=1 |
| 4 | F6Bx | MVDD Xmem,Ymem | RSBX ST1 | bit9=1, bit8=0 |
| 5 | F7Bx | LD #k8 → AR7 | SSBX ST1 | bit9=1, bit8=1 |

### Return/Branch Instructions (Fixes 6-12)

| Fix | Opcode | Was | Now | tic54x-opc.c |
|-----|--------|-----|-----|---------------|
| 6 | FC00 | LD #k<<16, B | RET (return) | line 391 |
| 7 | FE00 | LD #k, B | RETD (return delayed) | line 392 |
| 8 | F073 | RET (pop stack) | B pmad (branch, 2-word) | line 264 |
| 9 | F074 | RETE (pop+clr INTM) | CALL pmad (call, 2-word) | line 279 |
| 10 | F072 | FRET (far return) | RPTB pmad (block repeat) | line 410 |
| 11 | F070 | RET (catch-all) | RPT #lku (repeat, 2-word) | line 409 |
| 12 | F071 | RET (catch-all) | RPTZ dst,#lku (repeat-zero) | line 412 |

### CALLD Return Address (Fix 13)
- F274 CALLD pushed PC+2, should be **PC+4** (skip 2-word instruction + 2 delay slots)
- Both copies fixed (line 960 and 1210)

### IDLE/FRET (Fixes 14-16)

| Fix | Opcode | Was | Now | tic54x-opc.c |
|-----|--------|-----|-----|---------------|
| 14 | F4E4 | IDLE | FRET (far return) | line 306 |
| 15 | F4E1 | NOP (catch-all) | IDLE (the real one) | line 310 |
| 16 | F4E5 | NOP (catch-all) | FRETE (far return from IRQ) | line 308 |

### TINT0 Interrupt Mapping (Fix 17)
- `calypso_tint0.h`: IFR bit 4 → **bit 3**, vector 20 → **vector 19**
- Per TMS320C5410A datasheet: TINT0 = IFR/IMR bit 3, interrupt vector 19
- Previous mapping (bit 4) was BRINT0 (BSP receive), not TINT0

### Condition Evaluator (part of Fix 7)
Replaced incomplete if-else chain with proper tic54x-opc.c encoding:
- CC1=0x40 (accumulator test), CCB=0x08 (B vs A)
- EQ=0x05, NEQ=0x04, LT=0x03, LEQ=0x07, GT=0x06, GEQ=0x02
- OV=0x70, NOV=0x60, TC=0x30, NTC=0x20, C=0x0C, NC=0x08

## Results

### Before (all 17 bugs present)
- DSP stuck in dispatch loop at 0x81AF forever
- IMR=0x0000 (never configured)
- idle=0 (IDLE never reached)
- TINT0 on wrong interrupt bit

### After
- DSP boots in 173 instructions
- IMR=0x002D configured (bits 0,2,3,5 = INT0,INT2,TINT0,BXINT0)
- Dispatch loop active at 0x81AF
- TINT0 on correct bit 3 (enabled in IMR)

### Remaining Issue
- **SP drift**: 0x5AC8 → 0x8FFE after boot. FRET at 0x770C pops XPC+PC from
  empty stack (init code reached via branch, not FCALL). The popped values are
  garbage → DSP executes DARAM data as code → SP corrupts gradually.
- **No return to IDLE**: dispatch loop never exits because the handler RET/RETD
  pops garbage return addresses. The real IDLE (F4E1) at 0xA6A0 is never reached.
- **INTM=1 stuck**: handler sets INTM=1 (SSBX 1,8 at 0x7710) and never clears it,
  preventing interrupt servicing.

## Key Reference
tic54x-opc.c location in container:
```
/var/lib/docker/overlay2/a242f59.../diff/root/gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c
```

## Files Modified
- `hw/arm/calypso/calypso_c54x.c` — md5: 5a474083
- `hw/arm/calypso/calypso_tint0.h` — md5: 65a588f7
