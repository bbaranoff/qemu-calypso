# C54x Decoder Audit — `calypso_c54x.c` (emulated TMS320C54x DSP)

> **Cross-link:** [`doc/FB_CORRELATOR_PIPELINE.md`](FB_CORRELATOR_PIPELINE.md) — the FB/FCCH
> correlator pipeline schema. This audit is the opcode-decoder companion to that pipeline doc;
> the headline correlator wall (`d_fb_det=0`) traces to a decode root described in §1 and §4.
>
> **Scope:** `/opt/GSM/qemu-src/hw/arm/calypso/calypso_c54x.c` (~12646 lines). READ-ONLY audit.
> **ISA authorities:** SPRU131G, SPRU172C (`/root/spru131g.txt`, `/root/spru172c.txt`),
> `doc/opcodes/tic54x_hi8_map.md`, binutils `tic54x-opc.c`.
> **Method:** every claim cites `c54x.c:line`. Findings are tagged **CONFIRMED** (checked against
> ISA bit-fields and/or re-read in source this audit) or **SUSPECT** (flagged, not fully proven).
> Adversarial verdicts (a second pass that *tried to refute* each bug) are folded into §4.

Date: 2026-06-22. Branch `dsp_revival`.

---

## 1. TL;DR — decoder health

- **Foundation is solid.** The data-bus map, MMR/peripheral MMIO, XPC program banking, and the four
  operand resolvers (`resolve_smem` `c54x.c:3578`, `resolve_lmem` `c54x.c:3778`, `resolve_xmem`
  `c54x.c:3842`, `c54x_cond_true` `c54x.c:3876`) are ISA-faithful field-for-field. ~70% of the file
  is env/PC-gated read-only diagnostic probes with **no** effect on execution.
- **~120 opcode handler blocks audited.** **~30 CONFIRMED bugs** (wrong identity, dead code, or wrong
  semantics), of which **~14 are HIGH/CRITICAL**. Two adversarially-checked "bugs" were down-graded to
  *inert dead code* (no runtime effect) — flagged honestly in §4.
- **THE worst, by impact (headline #1, correlator):** an **entire 0x30–0x37 MAC/LD/BITT cluster is
  dead code mislocated under `case 0xF:`** (`c54x.c:5056–5191`), while the *reachable* `case 0x3:`
  (`c54x.c:7698`) blindly runs **every** `0x3xxx` as a generic MAC (`acc += T*Smem`) — so `LD Smem,T`,
  `BITT`, `MACA`, `MPYA`, `POLY` are all silently mis-executed. This directly starves the FCCH
  correlator's coefficient/TC path. **CONFIRMED.**
- **Headline #2 (correlator, partially fixed):** `case 0x5` (`c54x.c:7856`) — DADST/DSADT
  (`0x5A/5B/5E/5F`) were fixed this session (`c54x.c:7878`, proper Lmem + C16 branch) but
  `0x50–0x59` and `0x5C–0x5D` **still fall through to SFTA/SFTL** (`c54x.c:7902+`) → any remaining
  dual long-word form is flattened to `dst>>=ASM`. **CONFIRMED.**
- **Headline #3 (control flow):** `hi8==0xF8` (`c54x.c:6062`) decodes the BC/FB branch family by
  *nibble* into fabricated BANZ/CALL/RPT/ACC-heuristic handlers, **never reading the 8-bit cond
  field**, and *decrements an arbitrary AR* on taken "BANZ" branches (`c54x.c:6127/6138`). **CONFIRMED HIGH.**
- **Headline #4 (IRQ deadlock, §2):** an interrupt taken with `IPTR=0x1FF` vectors into a RET-
  terminated ROM stub; the `FCxx RET` handler (`c54x.c:6866`) correctly does **not** clear INTM,
  so INTM sticks at 1 forever and no further IRQ is serviced. **Root cause is upstream IPTR, not the
  RET handler.** **CONFIRMED.**
- **Catch-all hazards:** `0xE000–0xE3FF` mask `0xFC00` mis-runs FIRS/LMS/SQDST/ABDST as a 1-word
  "CMPS" (`c54x.c:6945`) — and FIRS is 2-word, so PC desyncs. `0xFD` decoded as fictional `LD #k,A`
  (`c54x.c:6870`, *but inert* — real XC handled at `c54x.c:4667`). `0xF6/0xF7` fabricated
  MVDD/LD-#k8 dispatches that corrupt regs on reserved opcodes.

---

## 2. THE BOOTSTUB / IRQ LEAD (headline writeup)

**User hypothesis:** an interrupt path leaves the DSP stuck in the boot stub with interrupts masked,
which is why the ARs are frozen in low memory (AR5=0x80, AR3=0x000b) and the real FRAME handler never
runs. **VERDICT: CONFIRMED.** The decoder is *not* the bug — the bug is the **wrong vector base
(`IPTR=0x1FF`)** combined with **ROM interrupt stubs that terminate in `RET` (0xFC00) instead of
`RETE`**, which the (correct) RET handler faithfully refuses to lift INTM on.

### 2.1 The "boot stub" is a PC-region state, not a flag
There is no `post_bootstub` variable. The boot stub is ROM at `PC 0x0000–0x007F`:
`0x0000 = LDMM SP,B`, `0x0001 = RET (0xfc00)`, then `0x0002..0x007F` is a `0xf495` NOP slide.
"Post-bootstub" = PC having left `[0x0000..0x007F]`. Every probe keys on exactly that:
`sp_ring_check_bootstub_entry` `was/now_inside = (pc<=0x007F)` (`c54x.c:9416-9418`),
`BOOTSTUB-ENTRY` on `s->pc==0x0000` (`c54x.c:4328`),
`POST-BOOTSTUB-RET` on `s->pc<=0x0008` inside the FCxx RET handler (`c54x.c:6820`).

### 2.2 Reset leaves the ROM vector page active with INTM masked  — **CONFIRMED**
`c54x_reset` (`c54x.c:12243`) sets `PMST=0xFFA8` → `IPTR=0x1FF`, and `ST1` with `INTM=1`
(`ST1_INTM|ST1_SXM|ST1_XF`). No later code ever writes `IPTR=0x140`. Live `PMST` transitions only ever
show IPTR `0x1ff / 0x0ee / 0x0f2 / 0x000` — **never `0x140`**, so the firmware's RAM vector table at
`0xA04C` is never selected.

### 2.3 The fatal vector formula  — **CONFIRMED at `c54x.c:12591`** (replay at `c54x.c:10452`)
```
s->pc = (iptr * 0x80) + vec * 4;     // c54x.c:12591
```
With `IPTR=0x1FF` → base `0xFF80`: **vec19 (INT3 FRAME) → 0xFFCC**, **vec21 → 0xFFD4** — the ROM/boot-
stub vector page, **not** the firmware handler at `0xA04C`. The code's own comment says exactly this
(`c54x.c:12606`: "INT3 at IPTR=0x1ff (vec=0xffcc) hits a garbage ROM stub; … 0x140 (vec=0xa04c) hits
the firmware real handler"). Dispatch sets `ST1_INTM` on entry (`c54x.c:12591`) — correct per ISA.

### 2.4 The culprit handler: `FCxx RC/RET` at `c54x.c:6779–6867`  — **CONFIRMED**
The terminating instruction of those `IPTR=0x1FF` ROM stubs is `0xFC00` (RET). The handler pops the
return address and does `s->pc = ra; return 0;` at **`c54x.c:6866`** with **no `ST1_INTM` clear**.
The popped `ra` is `0x0000`, dropping the DSP into the boot stub with **INTM permanently = 1**.

- This is **ISA-correct**: plain `RET` must *not* clear INTM. Contrast the INTM-clearing returns that a
  real ISR uses: `RETE` `c54x.c:4888` (`st1 &= ~ST1_INTM`), `RETED` `c54x.c:6389`, `FRETED`
  `c54x.c:6480`. The ROM stubs simply aren't ISRs — they end in `RET`, not `RETE`.
- Once INTM is stuck high, **both** servicing guards stop firing: the non-IDLE dispatch guard
  `!(ST1_INTM) && unmasked && delay_slots==0` (`c54x.c:12574`) and the in-loop pending replay guard
  `!(ST1_INTM)` (`c54x.c:10437`). No further interrupt is ever taken.

### 2.5 Why the ARs are stuck in low memory
The **real INT3/FRAME handler at `0xA04C` is the code that sets up the AR pointers** for the frame/
correlator buffers (AR5→I/Q buffer, AR3/AR4→ref taps, per `FB_CORRELATOR_PIPELINE.md` §2). Because
IRQs vector to the RET-terminated ROM stubs at `0xFF80+` instead, that handler **never runs even
once**, so the ARs are never initialized off their reset/garbage values (AR5=0x80, AR3=0x000b). After
the first ROM-ISR `RET`, INTM is stuck and the situation is permanent. Live log corroboration:

- `qemu.log:610-612` — `IRQ vec=21 … PC=0xffd4` → `IRQ vec=19 … INTM=1 … PC=0xffd4` → `BOOT[2.0]
  PC=0xffd4 op=0x007f`: the IRQ vectored into the ROM page.
- `qemu.log BOOT[2.1..2.19]` — `0xffd5 op=0xf074 (CALL)` → ROM ISR `0xf310..0xf322` → `0xf323
  op=0xfc00 (RET)` → `BOOT[2.19] PC=0x0000 op=0xba18` (boot stub) → `0x0001 RET` → NOP slide: the ROM
  ISR's RET returned **into the boot stub**.
- `qemu.log:2782` — `RC/RET PC=0xf323 … ra=0x0003` and `:2757` `RC/RET PC=0xffcd … ra=0x770d`: the ROM
  page returns land at the boot stub / post-IDLE, **always via plain RC, never RETE**.

### 2.6 Recommendation (do NOT patch the RET handler)
The divergence is **upstream of `c54x_interrupt_ex`**, which itself is correct. Two correct-target
options:
1. **Primary:** ensure `IPTR` is relocated to the firmware RAM vector page (`0x140` → base `0xA000`,
   INT3 at `0xA04C`) *before* the first IRQ is serviced — trace the PMST/IPTR write path and the API
   handshake, not the DSP core. The bootloader PMST write that should set `IPTR=0x140` is being lost
   or never issued.
2. **Defensive:** treat dispatching an IRQ into `IPTR=0x1FF` (PC `0xFF80..0xFFFF`) as a dead end
   (RET-terminated stub) — refuse to dispatch there, or assert.

**Do NOT** make `FCxx` clear INTM — that would violate ISA semantics and merely mask the real bug.

---

## 3. DECODER MAP — per-region handler table

Grouped by hi8 family. `★` = session-fixed and CONFIRMED correct. Severity: C=critical, H=high,
M=medium, L=low, –=none. Verdict: OK / BUG / DEAD (unreachable) / FIX (session-fixed) / SUSP (suspect).

### 3.0 Foundation — addressing & data bus (`c54x.c:1590–3901`) — mostly OK
| line | unit | verdict | sev | note |
|---|---|---|---|---|
| 1662 | `data_read_locked` MMR/API/DARAM read map | OK | – | MMR defs verified vs header (IMR=0…XPC=0x1E); accumulator slices correct |
| 2386 | `data_write_locked` MMR/DMA/McBSP/API write | OK | L | IFR write-1-clear, AG/BG 8-bit, XPC 2-bit masking correct; **SP-GUARD** rejects MMR_SP writes in `[0x0800..0x0900]` (`c54x.c:3040`) — non-silicon firmware hack |
| 2879 | DROM LUT-col write protect (offset 0x07) | SUSP | M | protects only offset-0x07 cells, ignores `PMST.DROM` — surgical workaround |
| 3513 | `c54x_prog_xlate` XPC banking ★ | FIX | – | XPC banking limited to `0x8000–0xDFFF`, `0xE000+` fixed PROM1 — Calypso-correct |
| 3528/3535 | `prog_fetch`/`prog_read` OVLY mirror | OK | L | OVLY window narrowed to `[0x80..0x2800)` vs silicon `[0x80..0x7FFF]` — overlays above 0x2800 fetch stale PROM |
| 3543 | `prog_write` | SUSP | L | `__attribute__((unused))` dead; would double-write + uses pre-fix banking if revived |
| 3566 | `c54x_circ_ref` circular modulo | SUSP | **M** | uses `ar % bk` instead of silicon 2^N-LSB-mask EFB (SPRU131G 5.5.3.4) — **diverges for non-power-of-2 BK, silently**; propagates to every circular MOD |
| 3578 | `resolve_smem` indirect MOD decode | OK | – | all 16 MODs verified vs SPRU131G Table 5-4; ARP←nar |
| 3651 | `resolve_smem` MOD 4/7 (`*ARx±0B`) | **BUG** | **M** | bit-reversed implemented as flat ±AR0, **reverse-carry IGNORED** → FFT/bit-reverse mis-addresses silently |
| 3697 | `resolve_smem` MOD 14 (`*+ARx(lk)%`) | SUSP | M | hand-rolled circular handles OVERFLOW only, not underflow, despite signed `lk` |
| 3778 | `resolve_lmem` long-word ±2 ★ | FIX | L | ±2 step verified vs SPRU172C; MOD 4/7 share bitrev gap (exotic for long) |
| 3842 | `resolve_xmem` dual-operand ★ | FIX | – | xmod 3 now circular; xar=field+2 (AR2–AR5) verified vs Table 5-7/5-8 |
| 3876 | `c54x_cond_true` | OK | – | verified vs SPRU172C App-A condition table |

### 3.1 `0xF4xx` accumulator/single-word group (`c54x.c:4659–5660`)
| line | opcode | instr | verdict | sev | note |
|---|---|---|---|---|---|
| 4659/4660 | 0xF495 | NOP | OK | – | exact match |
| 4667 | 0xFD/0xFF | **XC n,cond** (the REAL one) | OK | L | single cond codes verified vs SPRU172C; combined-cond OV fallback flawed (M); word-skip assumes 1 word/insn |
| 4732 | F4E2/3/F5E2/3 | BACC/CALA A/B | OK | – | |
| 4828 | F4E6/7/F5E6/7 | FBACC/FCALA A/B | OK | – | far; XPC←acc[22:16] |
| 4878 | F4EB | RETE | OK | – | pops XPC then PC, clears INTM |
| 4910 | F4E4 | FRET ★ | FIX | – | pops PC then XPC unconditionally |
| 4930 | F4E1.. | IDLE 1/2/3 | OK | – | |
| 4954 | F483 | SAT | OK | – | |
| 4964/4972 | F484/F485 | NEG/ABS | OK | – | |
| 4982/4991 | F48C/F48D | MPYA/SQUR (reg form) | OK | – | uses A.hi correctly |
| 5001 | F48E | EXP | OK→SUSP | L | `src==0` returns T=31, ISA says **T=0** (`c54x.c:5454` dup) |
| 5024/5035 | F486/F487 | MAX/MIN ★ | FIX | – | identity moved from wrong F492/F493 — verified |
| **5056–5191** | 0x28/0x2A/0x2E/0x30–0x37 | MAC/MAS/MACA/MPYA/LD/BITT | **DEAD** | **H** | **all unreachable** (inside `case 0xF:`, masks test `0x2xxx/0x3xxx`) — the "MAC family fix" never runs; real (wrong) impl in case 0x2/0x3. See §4-A |
| 5205 | F49E | "SUBC" (BOGUS) | **BUG** | **H** | wrong identity; real SUBC=Smem op `0x1E00`, implemented **nowhere** |
| 5215/5502 | F48F | NORM | **BUG** | **H** | hand-rolled 1-bit shift + decrements T; ISA = `dst=src<<TS`; wrong mask `0xFEFF` drops F68F/F78F |
| 5223/5543 | F488/F588 | MACA T,src | BUG | M | B-variant multiplies by B.hi; ISA multiplicand is **always A.hi** |
| 5239/5250 | F490/F491 | ROR/ROL | OK | L | carry from bit39 not bit31, guard bits not cleared |
| 5267 | F493 | CMPL ★ | FIX | – | |
| 5277 | F49F | RND ★ | FIX | L | adds 0x8000 but doesn't zero low half |
| 5286/5294/5302 | F480/F481/F482 | ADD/SUB/LD src,ASM,dst | BUG | M | **ASM shift ignored** (does `dst±=src`) |
| 5311–5347 | F400/F420/F440/F460/F4A0 | ADD/SUB/LD/SFTA/SFTL src,SHIFT,dst | OK | L | signed 5-bit shift applied; SXM/OVM/C not modeled |
| 5359/5642 | F494 | SFTC ★ | FIX | – | `src(31)==src(30)→<<1` matches SPRU172C |
| 5383–5450 | F4xx | duplicate F4 block | OK | L | mostly unreachable duplicates (maintenance hazard — two copies can drift) |
| 5474/5481 | F492/F593 | MAX/MIN (dup) | BUG | M | ignore dst bit, always write A → MAX B / MIN B wrong |
| 5633 | F4Bx | RSBX (narrow dup) | SUSP | M | only clears ST0; the real ST0/ST1 path is `c54x.c:5744` |

### 3.2 `0xF0/0xF1/0xF2/0xF3` ALU-immediate (`c54x.c:5657–6340`)
| line | opcode | instr | verdict | sev | note |
|---|---|---|---|---|---|
| 5683/5692/5709 | F073/F074/F072 | B/CALL/RPTB pmad | OK | – | |
| 5719/5728 | F070/F071 | RPT #lk / RPTZ | OK | L | confirm +1 repeat applied in run loop |
| 5744 | F0Bx/F1Bx | RSBX/SSBX N,SBIT (general) | OK | L | set=bit8, st-select=bit5 — correct path |
| 5756/5783 | F0/F1 2-word #lk | ADD/SUB/LD/AND/OR/XOR #lk,SHIFT | **BUG** | **H** | **src/dst REVERSED** (bit8=src,bit9=dst) vs ISA bit9=src/bit8=dst; unsigned shift; AND/OR/XOR skip sext40. Documented compensating-bug |
| 5813 | F08x–F0Fx 1-word shift ★ | AND/OR/XOR/SFTL src,SHIFT,dst | FIX | M | strict src=bit9/dst=bit8 (IMR-thrash fix) — correct, but now **inconsistent** with the reversed 2-word block above |
| 5852/5865/5882 | F272/F274/F273 | RPTBD/CALLD/BD ★ | FIX | – | delay-slot machinery armed (was immediate/RETD) |
| 5922–6017 | F2xx | ALU #lk (strict) | OK | L | binutils-strict convention; signed 4/5-bit shift; signed/unsigned-#lk split |
| 5984/6300 | F280–F2FF / F380–F3FF | AND/OR/XOR/SFTL | SUSP | L | for AND/OR/XOR shifts the **dst** input not src — verify operand role |
| 6232–6300 | F3xx ALU #lk ★ | ADD/SUB/LD/AND/OR/XOR/MAC #lk | FIX | – | INTR-k-at-F300 + LMS catch-all removed; strict dispatch |
| 6314 | F080/A0/C0/E0 | AND/OR/XOR shift | BUG | M | shifts **dst_in** not src — only correct when shift==0 |

### 3.3 `0xF8` branch family (`c54x.c:6062–6190`) — **systemically wrong**
| line | opcode | claimed | reality | verdict | sev |
|---|---|---|---|---|---|
| 6062 | F8xx | nibble-split | **whole range is BC; F880-F8FF=FB** — cond field never read | **BUG** | **H** |
| 6075 | F820/F830 | ACC!=0 / ACC==0 | real cond = **NTC / TC** (test ST0.TC) | **BUG** | **H** |
| 6103 | F880–F8FF | FB pmad (far) | OK (only correct path) | OK | L |
| 6120 | F88x–F8Bx | near-B | BC cond branch | BUG | M |
| 6127 | F86x/F87x | "BANZ *ARn" + **AR decrement** | BC (ANOV/AOV/BNOV/BOV) | **BUG** | **H** |
| 6138 | F84x/F85x | "BANZ ARn" + **AR decrement** | BC (AEQ/ANEQ/AGT/BEQ…) | **BUG** | **H** |
| 6150 | F8Cx–F8Fx | "CALL" + stack push | dead (FB mask catches all bit7=1 first) | DEAD | – |
| 6164 | F80x/F81x | "BANZ Smem" + AR post-mod | BC (UNC/C/NC/BIO) | **BUG** | **H** |
| 6188 | F8xx fallback | "RPT Smem" | unreachable | DEAD | M |

### 3.4 `0xF5/F6/F7/F9/FA/FB/FC/FD/FE/FF` calls/returns/status (`c54x.c:6353–6938`)
| line | opcode | instr | verdict | sev | note |
|---|---|---|---|---|---|
| 6367 | F6Bx | RSBX ST1 | OK | – | |
| 6385/6402 | F6EB/F69B | RETED/RETFD | OK | – | near, 2 delay slots |
| 6409 | F6E2/3 | BACCD/CALAD A | OK | L | |
| **6473** | F6E4/F6E5 | FRETD/FRETED | **BUG** | **H** | **pop order INVERTED** — pops XPC before PC; every far call pushes PC on top; non-delayed FRET pops PC first → swaps PC↔XPC after any FCALL. See §4-B |
| 6485 | F6E6/F6E7 | FBACCD/FCALAD A | OK | L | |
| 6520 | F68x–F6Fx | "MVDD" catch-all | **BUG** | **H** | MVDD is `0xE5xx`; F6xx has no data-move; raw 3-bit AR, no circular — corrupts mem/AR on reserved opcodes |
| 6353/6361 | F62x/F66x | acc-move | BUG | M | fabricated; reserved → should NOP |
| 6542 | F5Bx | SSBX ST0 | OK | – | (RPT #k fallback for other F5xx is fabricated — SUSP/M) |
| 6573 | F7Bx | SSBX ST1 (incl INTM) | OK | – | |
| 6583 | F70x–F7Dx | "LD #k8→reg" | **BUG** | **H** | **fabricated**; no such opcodes; **real INTR K (F7C0-F7DF) falls into case 0xC/0xD and clobbers BK/SP**. See §4-C |
| 6641 | F9xx | CC pmad,cond / FCALL ★ | FIX | L | low-byte cond via `c54x_cond_true` |
| 6701 | FAxx | FBD / BCD | SUSP | M | far OK; **near FA00-FA7F treated unconditional + non-delayed** (deliberate revert) |
| 6731 | FBxx | FCALLD / CCD ★ | FIX | – | delayed CCD armed |
| 6776 | FCxx | **RET / RC cond** | OK | L | `s->pc=ra` `c54x.c:6866`, **no INTM clear (correct)** — the §2 culprit by position, not by bug |
| 6870 | FDxx | "LD #k,A" | **BUG (inert)** | – | wrong identity, but **unreachable** — real XC at `c54x.c:4667` returns first. See §4-D |
| 6878 | FExx | RETD / RCD cond ★ | FIX | L | delayed return armed |
| 6936 | FFxx | XC 2,cond | OK | – | handled at `c54x.c:4667`; `goto unimpl` here is unreachable |

### 3.5 `0xE` group (`c54x.c:6939–7170`)
| line | opcode | instr | verdict | sev | note |
|---|---|---|---|---|---|
| **6945** | E000–E3FF (mask FC00) | "CMPS" | **BUG** | **C** | actually **FIRS/LMS/SQDST/ABDST**; CMPS is `0x8C/0x8E`. **FIRS is 2-word; pmad word never fetched → PC desync.** TC polarity also inverted |
| 6966 | EA00/EB00 | LD #k9,DP | OK | L | shadows the (wrong) 7099 RPTB handler — fine, it IS LD #k9,DP |
| 6985 | EC | RPT #K8 | OK | – | |
| 6995 | E5 | MVDD Xmem,Ymem ★ | FIX | – | proper (n&3)+2 dual-operand + circular BK |
| 7029 | E4 | "BITF" (2-word) | **BUG** | **H** | wrong identity; real BITF=`0x61xx` (handled at 7334); E4 is a parallel-store form; **over-consumes a word** |
| 7038 | E7 | MVMM MMRx,MMRy | OK | – | |
| 7053 | E8/E9 | LD #k8u,dst | **BUG** | **H** | `<<16` wrong (loads HIGH half; ISA loads low) **and** SXM sign-extend wrong (short imm always zero-filled) |
| 7066 | E1 (sub-switch) | "NEG/ABS/CMPL…" | **BUG** | M | dead (shadowed by 6945 catch-all) + wrong identity (E1=LMS) |

### 3.6 `0x6/0x7`, `0x1`, `0x0`, `0x3`, `0x2`, `0x4`, `0x5` low-nibble groups
| line | opcode | instr | verdict | sev | note |
|---|---|---|---|---|---|
| **7698** | case 0x3 (all 0x3xxx) | generic MAC `acc+=T*Smem` | **BUG** | **H** | **ignores sub-opcode byte**; mis-runs LD Smem,T (0x30), LD Smem,ASM (0x32), BITT (0x34), MACA/MACAR (0x35/0x37 — should use A.hi, not T), MPYA (0x31), POLY (0x36). See §4-A |
| 7711 | case 0x2 | MPY/SQUR/MPYA/MACA/MAS | OK→SUSP | M | sub-decoded, but MAC family multiplicand/round nuances unverified; MAS default branch broad |
| 7758 | case 0x4 | SUB/ADD/... Smem,16 + PSHM/PSHD/LTD/DELAY/DST | OK | L | 0x4A–0x4F verified (PSHM/PSHD/LTD/DELAY/DST) `c54x.c:7862-7901` |
| **7856** | case 0x5 | SFTA/SFTL + DADST/DSADT ★ | FIX/BUG | **H** | **0x5A/5B/5E/5F DADST/DSADT fixed** (`c54x.c:7878`, Lmem + C16); **0x50-0x59, 0x5C-0x5D STILL fall through to SFTA/SFTL** (`c54x.c:7902+`). See §4-E |
| 7931 | case 0x8/0x9 | MAC Xmem,Ymem / moves / CMPS (0x8E) | OK→SUSP | M | dual-MAC 2-bit decode fixed this session; CMPS at 0x8E used by correlator |
| 8363 | A4-A7/B0-B7 | dual-MAC ★ | FIX | – | raw 3-bit AR → 2-bit Xmem/Ymem |
| 8499/8520/8540 | A1/BC-BF/B8-BB | SQDST/POLY/MAS ★ | FIX | – | raw → 2-bit decode |
| 8768–9052 | D0-D9/DB/DC, C8-CB | ST‖LD MOD ★ | FIX | – | MOD 1/2 were inverted (1=inc 2=dec), now SPRU131G (1=dec 2=inc) |

---

## 4. CONFIRMED BUGS (adversarially verified) — prioritized

Each was checked by a second pass that *attempted to refute* it. Severity ordering by FB-correlator
and control-flow impact.

### 4-A. [CRITICAL, correlator] `0x3xxx` runs as blind MAC; correct decode is dead code
- **Where:** reachable handler `case 0x3:` `c54x.c:7698-7708`; dead "correct" cluster
  `c54x.c:5056–5191` (inside `case 0xF:`).
- **What:** the `switch(hi4)` dispatches on `(op>>12)&0xF` (`c54x.c:3988`). The well-formed handlers
  for `0x30 LD Smem,T`, `0x31 MPYA`, `0x32 LD Smem,ASM`, `0x33 MASA`, `0x34 BITT`, `0x35 MACA`,
  `0x37 MACAR` sit under `case 0xF:`, where `op` is always `0xFxxx`, so their `(op&0xFF00)==0x3x00`
  guards are **statically unsatisfiable → dead code**. The genuinely-reachable `case 0x3:` ignores the
  sub-opcode byte entirely and runs **every** `0x3xxx` as `acc += T*Smem`.
- **ISA:** SPRU172C: `LD Smem,T` (`T=Smem`), `BITT Smem` (`TC=Smem(15-T(3:0))`), `MACA Smem,B`
  (`B=B+Smem*A(32-16)` — multiplicand **A.hi**, not T). `doc/opcodes/tic54x_hi8_map.md` confirms the
  0x3x encodings.
- **Impact:** **directly starves the FCCH correlator** — `LD coeff,T` is mis-run, `BITT` never sets TC
  (downstream ROLTC/angle path stale), MACA/MPYA accumulate the wrong product. The session "MAC family
  fix" and "BITT fix" are in the **wrong switch case** and never execute. **CONFIRMED** (refutation
  failed: mutually-exclusive `hi4` conditions, `op` never reassigned, no fall-through).

### 4-B. [HIGH, control flow] FRETD/FRETED inverted PC/XPC pop
- **Where:** `c54x.c:6473-6484` (0xF6E4/0xF6E5).
- **What:** pops `XPC` then `PC`. But every far-call push site pushes XPC-then-PC leaving **PC on TOS**
  (FCALL `c54x.c:6658`, FCALLD `c54x.c:6744`, FCALAD `c54x.c:6510`), and the non-delayed sibling FRET
  pops PC-first (`c54x.c:4911`). FRETD reads PC's slot into `s->xpc` (then `&=3` destroying it) and the
  2-bit XPC into the return PC. **Note:** in isolation FRETD matches *silicon* TOS order, but the
  emulator's whole push convention is the opposite, and FRET agrees with the push side — FRETD is the
  lone break in internal consistency.
- **Impact:** PC↔XPC swap on every far-delayed return → control flow lands near address 0 after any
  FCALL/FCALLD/FCALAD. **CONFIRMED.**

### 4-C. [HIGH] `0xF7xx` fabricated LD-#k8 dispatch corrupts BK/SP on real INTR K
- **Where:** `c54x.c:6583-6639`.
- **What:** decodes `F70x..F7Dx` as `LD #k8 → ASM/AR/T/DP/ARP/BK/SP`. No such opcodes exist (LD #k9,DP=
  `0xEA00`, LD #k5,ASM=`0xED00`, LD #k3,ARP=`0xF4A0`). The NOP guard (`c54x.c:6593`) covers only
  `F7E0-F7FF`, so the **real `INTR K` (`0xF7C0-0xF7DF`)** — which has no handler — falls into
  `case 0xC: s->bk=k` / `case 0xD: s->sp=k`, silently clobbering BK/SP. The `case 0xB` (AR7) is dead
  (shadowed by the `F7Bx` SSBX-ST1 handler at `c54x.c:6573`).
- **ISA:** SPRU172C `INTR K = 1111 0111 110 KKKKK` (`0xF7C0/0xFFE0`). **CONFIRMED.**

### 4-D. [HIGH identity / inert] `0xFDxx` decoded as fictional `LD #k,A`
- **Where:** `c54x.c:6870-6874`.
- **What:** loads sign-extended byte into A. `0xFD/0xFF` is **XC** (`1111 11N1 cccccccc`). **However**
  the correct XC handler at `c54x.c:4667` matches both `0xFD` and `0xFF` and returns *before* this
  block (same `case 0xF:`, only the F495 NOP precedes it). So the `LD #k,A` handler is **unreachable
  dead code** — the audit's behavioral "clobbers A on every XC" is **refuted**; impact is a latent
  maintenance hazard, not a runtime bug. **CONFIRMED dead / inert.**

### 4-E. [HIGH, correlator] `0x50-0x59 / 0x5C-0x5D` long-word ops still fall to SFTA/SFTL
- **Where:** `case 0x5` `c54x.c:7856`; dual long-word fix at `c54x.c:7878`; fallthrough at
  `c54x.c:7902+`.
- **What:** the session fix correctly routes **DADST/DSADT (0x5A/5B/5E/5F)** through `resolve_lmem` +
  C16 branch — verified against the SPRU172C worked examples (C16=1 dual-16; C16=0 double-precision).
  But `DADD/DSUB/DLD/DRSUB/DSUBT` (`0x50-0x59`, `0x5C-0x5D`) **are not special-cased** and drop into
  the generic SFTA/SFTL body (`sub=(op>>9)&7`), so any such dual long-word op is flattened to
  `dst>>=ASM`. Per `FB_CORRELATOR_PIPELINE.md`, the correlator kernel alternates DADST/DSADT — those
  are now correct; remaining DADD/DSUB families are the open TODO. **CONFIRMED.**

### 4-F. [CRITICAL, PC desync] `0xE000-0xE3FF` mask `0xFC00` mis-runs FIRS/LMS/SQDST/ABDST as "CMPS"
- **Where:** `c54x.c:6945`.
- **What:** mask `0xFC00` swallows 4 distinct dual-operand DSP ops (FIRS=0xE000 **2-word**, LMS=0xE100,
  SQDST=0xE200, ABDST=0xE300) and runs them all as a 1-word single-Smem "CMPS". Real CMPS is
  `0x8C/0x8E`. FIRS's pmad extension word is never fetched → **PC desync** on every 0xE0xx. CMPS
  semantics here are also wrong (compares acc.high vs Smem instead of high-vs-low of one acc; TC
  polarity inverted). **CONFIRMED.**

### 4-G. [HIGH, control flow] `0xF8` BC/FB family decoded by nibble into BANZ/CALL/RPT + AR decrement
- **Where:** `c54x.c:6062` (dispatch), `6075/6127/6138/6164` (sub-handlers).
- **What:** the entire `0xF8` byte is BC (`1111 10Z0 cccccccc`, low byte = 8-bit cond); `F880-F8FF` is
  FB. The handler never reads the cond field — it nibble-splits into ACC!=0/==0 heuristics (F820/F830,
  should be NTC/TC), "BANZ *ARn"/"BANZ ARn" (F84x-F87x — these are ACC/OV cond BC, and the code
  **decrements an arbitrary AR** which BC must never do), near-B, CALL (spurious push), BANZ-Smem
  (spurious AR post-mod), and an RPT-Smem fallback. Only the FB-far path (`c54x.c:6103`) is correct.
- **ISA:** BANZ=`0x6C/0x6E`, RPT=`0x47`, CALL=`0xF074` — none are `0xF8`. **CONFIRMED HIGH**
  (documented as a tolerated firmware dialect; a strict-cond attempt was reverted 2026-05-15).

### 4-H. [HIGH] `0xF0/F1` 2-word #lk handlers use REVERSED src/dst
- **Where:** `c54x.c:5756/5783`.
- **What:** legacy path uses `src=bit8, dst=bit9`; ISA form 7 is `src=bit9, dst=bit8` (and the strict
  F2/F3 siblings + the session-fixed 1-word block at `c54x.c:5813` use the correct convention). Also
  unsigned 4-bit shift, AND/OR/XOR skip sext40. So F0/F1 is **internally inconsistent**. Documented
  compensating-bug ("firmware s'y est calé"). **CONFIRMED.**

### 4-I. [MEDIUM] Misc reachable semantic bugs
- `0xF49E` "SUBC" — bogus identity, real SUBC=`0x1E00` implemented nowhere (`c54x.c:5205`). CONFIRMED.
- `F48F NORM` — 1-bit conditional shift + T-decrement instead of `dst=src<<TS`; mask drops 2 of 4
  encodings (`c54x.c:5215/5502`). CONFIRMED.
- `F480/F481/F482 ADD/SUB/LD src,ASM,dst` — ASM shift ignored (`c54x.c:5286`). CONFIRMED.
- `F488/F588 MACA` — B-variant uses B.hi not A.hi (`c54x.c:5223`). CONFIRMED.
- `MAX/MIN dup` (`c54x.c:5474/5481`) — ignore dst bit, always write A. CONFIRMED.
- `E8/E9 LD #k8u` — `<<16` + wrong SXM (`c54x.c:7053`). CONFIRMED.
- `E4 "BITF"` — wrong identity + over-consume word (`c54x.c:7029`). CONFIRMED.
- `c54x_circ_ref` modulo-BK (`c54x.c:3566`), `resolve_smem` bitrev 4/7 (`c54x.c:3651`) — silent
  mis-address for non-pow2 BK / FFT walks. CONFIRMED.

### Down-graded by adversarial pass (honest negatives)
- `0xF8Cx-F8Fx "CALL"` (`c54x.c:6150`) — wrong/misleading source but **dead code**: every bit7=1
  opcode is caught by the FB mask at `c54x.c:6103` first. No runtime effect. **UNCERTAIN→inert.**
- `0xFDxx "LD #k,A"` (`c54x.c:6870`) — see §4-D, inert (real XC at 4667).
- `0xFFxx` (`c54x.c:6936`) — the claim it is unhandled is **refuted**; it is handled at `c54x.c:4667`.

---

## 5. SESSION FIXES — each with a one-line correctness confirmation

| Fix | line | one-line confirmation |
|---|---|---|
| DADST/DSADT dual long-word | `c54x.c:7878` | **CONFIRMED** — Lmem decode + C16 branch matches SPRU172C worked examples; ±2 long post-mod via `resolve_lmem` |
| dual-MAC A4-A7/B0-B7 | `c54x.c:8363` | **CONFIRMED** — raw 3-bit AR → 2-bit Xmem/Ymem decode per Table 5-6/5-8 |
| 0x90-93 MAC, SQDST 0xA1, POLY 0xBC-BF, MAS 0xB8-BB | `c54x.c:7939/8499/8520/8540` | **CONFIRMED** — raw → 2-bit dual-operand decode |
| ST‖LD MOD 1/2 inversion | `c54x.c:8768/9034` | **CONFIRMED** — now 1=dec 2=inc per SPRU131G |
| `c54x_prog_xlate` XPC banking | `c54x.c:3513` | **CONFIRMED** — `0x8000-0xDFFF` banked, `0xE000+` fixed PROM1 (Calypso layout) |
| `resolve_lmem` ±2 step | `c54x.c:3778` | **CONFIRMED** — long-operand step verified vs SPRU172C footnote |
| `resolve_xmem` xmod 3 circular | `c54x.c:3842` | **CONFIRMED** — `*AR+0%` now routes through `c54x_circ_ref` |
| MVDD at 0xE5 | `c54x.c:6995` | **CONFIRMED** — proper `(n&3)+2` dual-operand + BK circular (replaces wrong 0xF6 decode) |
| MAX F486 / MIN F487 / ROLTC F492 / CMPL F493 / RND F49F | `c54x.c:5024/5035/5196/5267/5277` | **CONFIRMED** — identities moved to correct SPRU172C opcode rows |
| SFTC F494 | `c54x.c:5359/5642` | **CONFIRMED** — `src(31)==src(30)→<<1` |
| FRET F4E4 unconditional pop | `c54x.c:4910` | **CONFIRMED** — pops PC then XPC, no APTS gating |
| CALLD F274 / BD F273 / RPTBD F272 | `c54x.c:5852/5882/5865` | **CONFIRMED** — arm delay slots instead of immediate jump |
| CCD FB00 / RCD FE00 delayed | `c54x.c:6760/6878` | **CONFIRMED** — pushes return + arms 2 delay slots |
| CC F9xx low-byte cond via `c54x_cond_true` | `c54x.c:6641` | **CONFIRMED** — replaces wrong `(op>>4)&0xF` |
| F0/F1 1-word shift strict src/dst | `c54x.c:5813` | **CONFIRMED** — src=bit9/dst=bit8 (IMR-thrash fix), *but conflicts with the still-reversed 2-word block* (§4-H) |
| F2xx/F3xx strict ALU-#lk dispatch | `c54x.c:5922/6232` | **CONFIRMED** — binutils-strict; removed bogus INTR-k-at-F300 + LMS catch-all |

---

## 6. KNOWN-SUSPECT / TODO

- **`0x50-0x59`, `0x5C-0x5D` (DADD/DSUB/DLD/DRSUB/DSUBT) still fall through to SFTA/SFTL**
  (`c54x.c:7902+`) — the remaining half of the dual long-word family. **Highest-value TODO** after the
  DADST/DSADT fix; same correlator path. (§4-E)
- **`case 0x3` blind-MAC** (`c54x.c:7698`) — needs per-sub-opcode decode for 0x30-0x37; the correct
  bodies already exist (dead) at `c54x.c:5056-5191` and can be relocated into `case 0x3`. (§4-A)
- **`0xE000-0xE3FF` CMPS catch-all** (`c54x.c:6945`) — split into FIRS(2-word)/LMS/SQDST/ABDST; fix the
  FIRS PC-desync. Real CMPS belongs at `0x8C/0x8E`. (§4-F)
- **`0xF8` BC/FB dialect** (`c54x.c:6062`) — replace nibble-split with a real 8-bit cond-field eval via
  `c54x_cond_true`; remove AR decrements. A strict attempt was reverted 2026-05-15 (suspected upstream
  BITF/TC bug — itself caused by §4-A's BITT-not-implemented). Fixing §4-A may unblock this. (§4-G)
- **`0xF6` mis-id "MVDD"** (`c54x.c:6520`) — F6xx has no data-move; should NOP reserved opcodes. MVDD is
  correctly at `0xE5` now. Documented SUSPECT confirmed. (§3.4)
- **`0xF7xx` LD-#k8 fabrication** (`c54x.c:6583`) — add a real `INTR K` handler at `F7C0`; stop
  clobbering BK/SP. (§4-C)
- **bitrev MOD 4/7** (`c54x.c:3651`) — reverse-carry ignored (flat ±AR0); FFT/bit-reverse mis-addresses
  silently. The MOD-MISMATCH probe (`c54x.c:3717`) is *blind* to this (its own reference is also flat).
- **`c54x_circ_ref` modulo-BK** (`c54x.c:3566`) — use 2^N-LSB-masked EFB (SPRU131G 5.5.3.4); diverges
  for non-power-of-2 BK across all circular MODs.
- **MVDM/MVMD `0x72/0x73` + MVDK `0x7193`** — reverted to STL 1-word via the over-broad
  `(op&0xF800)==0x7000` catch-all (`c54x.c:7258`); documented compensating-bug
  (`doc/REVERT_MVMD_KNOWLEDGE.md`).
- **FRETD/FRETED pop order** (`c54x.c:6473`) — invert to PC-then-XPC to match the emulator's push
  convention. (§4-B)
- **F0/F1 src/dst inconsistency** (`c54x.c:5756` reversed vs `c54x.c:5813` strict) — unify on the
  binutils-strict convention once firmware dependence is re-validated.

---

*Generated 2026-06-22, read-only audit. Every claim cites `calypso_c54x.c:line`. CONFIRMED = checked
against SPRU131G/SPRU172C/tic54x_hi8_map or re-read in source this pass; SUSPECT = flagged, not fully
proven. Adversarial verdicts (refutation pass) folded into §4.*
