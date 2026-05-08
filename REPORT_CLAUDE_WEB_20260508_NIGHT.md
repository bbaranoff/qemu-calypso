# Report for Claude Web — 2026-05-08 night session

## TL;DR

Found the smoking gun for IMR=0 / DSP-stuck-in-RPTB-loop blocker:
**opcode 0x76xx is misdecoded as `LDM MMR, dst` (1-word) when binutils
`tic54x-opc.c` says it's `ST #lk, Smem` (2-word).** This causes every
firmware `ST #lk, Smem` to advance PC by only 1 instead of 2; the literal
`lk` value then gets executed as a stray opcode, and one of those strays
writes 0 to MMR_IMR via `DST B, Lmem` with DP=0. Result: INT3 + BRINT0
both pending in IFR (10000+ IRQs counted) but never serviced. DSP parks
in RPTB at e9ab..e9b6 forever waiting for an interrupt.

I want your validation of the fix and a sanity check on the proposed
patch before applying.

## Live run state

Container `trying`, container has 4078M+ DSP insn run :

```
ARM  : fbsb hooks fire every frame, ARM TASK WR [0x0008/0x0030] = 5
       (FB0_SEARCH), fn now ~75088, mobile alive in cell-search
DSP  : RPTB tight loop e9ab..e9b6, PC HIST = 7 PCs at 14286 hits / 100k
       window, fb0_att=0 fb1_att=0 sb_att=0 (no FB ever detected)
BSP  : RX delta stats min=0 max=32 mean=6 — bursts arrive within window,
       NOT the blocker
```

## Root cause chain

### IFR / IMR trace

```
INTM-TRANS 1..10  : early boot, INTM 0→1, normal
IMR-W 0x0000 → 0x0010  PC=0xc471  op=0x69f8  insn=40684074  ← OR IMR,#0x10 (TINT0)
IMR-W *ZERO* 0x0010 → 0x0000  PC=0xc44b  op=0x68f8         ← AND IMR,#0
IMR-W 0x0000 → 0x0010  PC=0xc471  op=0x69f8                ← OR IMR,#0x10 again
IMR-W *ZERO* 0x0010 → 0x0000  PC=0x8859  op=0x4f00 prev_op=0x7615  ← THE BUG
                                                                     insn=40691264
```

After insn 40691264, IMR stayed 0 from then to insn=4078M+.

```
IRQ #1   vec=19 bit=3  IFR=0x0008 (INT3 pending)
IRQ #2   vec=21 bit=5  IFR=0x0028 (INT3 + BRINT0 pending)
...
IRQ #10300 (latest)    IFR=0x0008..0x0028, IMR=0x0000 throughout
```

10000+ interrupts raised, none serviced. Both INT3 (bit 3, frame) and
BRINT0 (bit 5, BSP RX) are masked because IMR is forever 0.

### ROM bytes at the IMR=0 site

```
PROM0[0x8854] = 0xf273
PROM0[0x8855] = 0x8866
PROM0[0x8856] = 0x7711   ← STM-ish 2-word: 0x77 mmr=0x11
PROM0[0x8857] = 0x4f00     value
PROM0[0x8858] = 0x7615   ← THIS — should be ST #0x4f00, Smem  (2-word)
PROM0[0x8859] = 0x4f00     value (the literal being stored)
PROM0[0x885a] = 0x7616   ← also a 2-word ST
PROM0[0x885b] = 0x0000
```

The IMR-W trace reports `PC=0x8859 op=0x4f00 prev_op=0x7615`. PC is at
0x8859 because the buggy `0x76` handler in our emulator only advanced PC
by 1, so 0x4f00 is being decoded as a fresh instruction instead of being
treated as the literal operand of the ST at 0x8858.

### Decoder bug

`hw/arm/calypso/calypso_c54x.c` lines 3348-3354 :

```c
/* 76xx: LDM MMR, dst */
if (hi8 == 0x76) {
    uint8_t mmr = op & 0x7F;
    uint16_t val = data_read(s, mmr);
    s->a = (int64_t)(int16_t)val << 16;
    return consumed + s->lk_used;   /* consumed = 1 */
}
```

Per binutils `tic54x-opc.c` (authoritative reference per CLAUDE.md
convention) :

```
{ "ldm",   1,2,2, 0x4800, 0xFE00, {OP_MMR,OP_DST},  0, REST}
{ "st",    2,2,2, 0x7600, 0xFF00, {OP_lk,OP_Smem},  0, REST}
{ "stm",   2,2,2, 0x7700, 0xFF00, {OP_lk,OP_MMR},   0, REST}
```

LDM is at 0x48xx (already correctly handled at line 3831). 0x76xx is
**ST #lk, Smem** (2-word), and the existing 0x77xx STM handler (line
3355) is correct. The 0x76 handler is just wrong — copy/paste of the
wrong mnemonic.

### Why this causes IMR=0

When the buggy 0x76 handler runs on `0x7615 0x4f00` :
1. Treats 0x7615 as LDM AR5 → A   (advances PC by 1)
2. Next iteration, fetches 0x4f00 at PC=0x8859 as a real opcode
3. 0x4f00 is in the 0x4Exx/0x4Fxx range = `DST src, Lmem` (line 3869)
4. `resolve_smem(0x4f00)` with DP=0 returns addr=0x00 = MMR_IMR
5. `data_write(addr=0x00, B_high)` — writes B's high word to IMR
6. B at that point is 0 → IMR := 0

(Same trick : `op8 = 0x4f` and `dst_b = 1` selects acc B; `addr & 0xFFFE`
forces even alignment; bit 7 = 0 in 0x4f00 → direct addressing → addr =
DP<<7 | 0 = 0 = MMR_IMR.)

## Proposed fix

Replace lines 3348-3354 :

```c
/* 76xx: ST #lk, Smem  (2 words) — store 16-bit literal to data memory.
 * Was incorrectly decoded as LDM MMR,dst. The real LDM is 0x48xx
 * (handled below). Per binutils tic54x-opc.c: 0x7600 mask 0xFF00. */
if (hi8 == 0x76) {
    op2 = prog_fetch(s, s->pc + 1);
    consumed = 2;
    addr = resolve_smem(s, op, &ind);
    data_write(s, addr, op2);
    return consumed + s->lk_used;
}
```

This matches the existing 0x77 handler exactly except :
- writes `op2` (literal lk) instead of fetching from MMR
- `addr` from `resolve_smem(op, …)` (Smem, not MMR & 0x7F)

## Open questions for you

1. **Sanity** : is my reading of `0x76 = ST #lk, Smem` correct ?
   The binutils entry is unambiguous but I want a second pair of eyes
   before clobbering a handler that's been there for weeks.

2. **Side effects** : changing 0x76 from 1-word to 2-word will shift PC
   alignment for any code path that previously skipped the literal as a
   stray "instruction". Is there a recommended way to canary this — e.g.
   diff PC HIST insn=0..1M before/after, or a specific symbol to watch ?
   The DSP boot path passes `OVLY` MVPD then enters runtime ; I'd expect
   the boot to break differently (or recover) under the fix.

3. **Other 0x76-confused sites** : I see other `IMR=0` and `IMR=0x10`
   writes at PC=0xc44b op=0x68f8 and PC=0xc471 op=0x69f8. Those have
   prev_op != 0x7615/0x7616 so probably aren't 0x76-induced. But should
   I also audit whether the `OR Smem,src` (0x69f8) and `AND Smem,src`
   (0x68f8) handlers correctly treat operand encoding ? The fact that
   they hit MMR_IMR with apparent intent (set bit 4 then clear it) might
   be the firmware doing a TINT0 toggle, but might also be more
   collateral from upstream PC drift.

4. **Test plan** : after applying the fix, what's the right "did it
   work" signal ?
     - IMR-W trace should show STM #0x????,IMR with non-zero values ?
     - INT3 vec=19 should service (idle=1 → 0 transition, PC jumping
       to 0xFFCC) ?
     - DSP should leave the RPTB loop e9ab..e9b6 ?
     - fb0_att should increment beyond 0 ?
   Or all of the above ?

5. **Memory note** : current `project_session_20260508_pm_fbdet_split.md`
   says "PM tourne mais ne transitionne jamais vers FB-det. IRQ rate
   1.5 Hz vs 217 Hz attendus". I read the rate gap as a *consequence*
   of IMR=0 (interrupts are raised but invisible to the DSP, so the
   firmware sees ~ARM-mediated polling rate), not the root cause. Agree ?

## Files / entry points if you want to dig

- `hw/arm/calypso/calypso_c54x.c:3348-3354`  — the buggy handler
- `hw/arm/calypso/calypso_c54x.c:3355-3382`  — the correct STM handler (model)
- `hw/arm/calypso/calypso_c54x.c:3869-3877`  — DST handler (the path
  that turned the stray 0x4f00 into an IMR write)
- `hw/arm/calypso/calypso_c54x.c:551-573`    — IMR-W trace
- ROM dump : `/opt/GSM/calypso_dsp.txt` in container, or
  `CALYPSO_DSP_ROM=/home/nirvana/qemu-src/calypso_dsp.txt
   bash /home/nirvana/qemu-calypso/dsp_read.sh prom0 0xADDR` on host
- Live log : `docker exec trying tail -200 /root/qemu.log`

## What I'd like you to do

1. Confirm or refute my 0x76 → ST #lk, Smem reading.
2. Suggest the canary / regression check for the patch.
3. Tell me whether to apply the fix in this session or wait for more
   data (any hidden code paths I'm missing where the current "LDM" decode
   is intentionally exploited ?).
