# QEMU Calypso — Claude Code Context

## Architecture

Dual-core GSM baseband emulator:
- **ARM7TDMI** runs osmocom-bb `layer1.highram.elf` firmware
- **TMS320C54x DSP** runs real Calypso ROM (`calypso_dsp.txt`)
- **API RAM** shared memory at DSP 0x0800 (ARM 0xFFD00000), 8K words
- **BSP** receives I/Q via UDP 6702, serves to DSP via PORTR PA=0x0034
- **TPU→TSP→IOTA** chain gates BDLENA for RX windows
- **Bridge** (Python) relays BTS UDP (5700-5702) ↔ BSP, clock-slave of QEMU

## Memory Map (DSP side)

| Range | Type | Content |
|-------|------|---------|
| 0x0000-0x007F | Boot ROM stubs | LDMM SP,B + RET at 0x0000, NOP elsewhere |
| 0x0080-0x27FF | DARAM overlay (OVLY) | Code + data, loaded by MVPD at boot |
| 0x2800-0x6FFF | Unmapped | Reads as 0x0000 |
| 0x7000-0xDFFF | PROM0 | DSP ROM (always readable) |
| 0xE000-0xFF7F | PROM1 mirror | Mirrored from page 1 (0x18000+) |
| 0xFF80-0xFFFF | Interrupt vectors | From PROM1, IPTR=0x1FF |
| 0x0800-0x27FF | API RAM | Shared with ARM (NDB, write/read pages) |

## Key DSP API Offsets (byte offsets from 0xFFD00000)

- Write Page 0: 0x0000 (20 words: d_task_d, d_burst_d, d_task_u, d_burst_u, d_task_md, ...)
- Write Page 1: 0x0028
- Read Page 0: 0x0050 (20 words: same + a_serv_demod[4] at +8, a_pm[3] at +12)
- Read Page 1: 0x0078
- NDB: 0x01A8 (d_dsp_page, d_error_status, d_spcx_rif, ...)
- d_fb_det: NDB + 0x48 = 0x01F0
- a_cd[15]: NDB + 0x1F8 = 0x03A0

## Interrupt Vectors (IPTR=0x1FF → base 0xFF80)

Vec = imr_bit + 16. Formula: addr = 0xFF80 + vec * 4
- INT3 (frame): vec 19, IMR bit 3 → 0xFFCC
- TINT0: vec 20, IMR bit 4 → 0xFFD0
- BRINT0 (BSP): vec 21, IMR bit 5 → 0xFFD4

## Known Fixed Opcode Bugs

Always verify against `tic54x-opc.c` (binutils) before changing any opcode:

| Opcode | Wrong decode | Correct decode | Impact |
|--------|-------------|----------------|--------|
| 0xF074 | RETE (1w) | CALL pmad (2w) | Was reverting, now correct |
| 0xE8xx/E9xx | CC cond call (2w) | LD #k8u,dst (1w) | Stack overflow — ROOT CAUSE |
| 0xED00-ED1F | BCD branch (2w) | LD #k5,ASM (1w) | Skipped DSP init code |
| 0x56xx | MVPD (2w) | SFTL shift (1w) | Wrote to SP via MMR |
| 0xF7Bx | SSBX (1w) | LD #k8,AR7 (1w) | Corrupted ST1 |
| MMR mask | & 0x1F | & 0x7F | STLM/POPM/PSHM wrong address |
| PORTR PA | 0xF430 | 0x0034 | DSP read wrong BSP port |
| 0xF4EB | — | RETE (correct) | The REAL rete opcode |
| 0xF9xx | BC branch (no push) | CC conditional call (push) | Lost return addresses |
| 0xFBxx | LD #k,16,A | CCD conditional call delayed (push) | Lost return addresses |
| NORM L799 | NOP (dead) | — removed, real NORM at L832 | FB correlator broken |

## Current Bug

**INTM=1 forever — DSP frame interrupts (INT3) never serviced.** This is the
remaining blocker for FB/SB detection. `c54x_interrupt_ex(dsp, 19, 3)` is
called ~1/frame (verified via uncapped IRQ log), TPU_FRAME wiring works,
firmware writes `TPU_CTRL.DSP_EN` correctly. But `INTM=1` (ST1 bit 11) is
never cleared, so every INT3 is rejected and `c54x_exec_one` continues in
its tight RPTBD loop at PROM1-mirror PCs `0xeb04-0xeb0b` (= flat 0x18B04+),
which is a dispatcher table.

Consequences:
- `RETE` count = 0 ever (no IRQ ever services, so no RETE ever returns)
- `d_task_md=5` armed by ARM but DSP never dispatches FB correlator
- `d_fb_det=0` permanent
- Mobile loops FBSB_REQ → FBSB_CONF(result=255) → RESET_REQ until it gives
  up; with `stick <arfcn>` in mobile config the retry is infinite

The DSP ROM has 12 RSBX INTM (`0xf6bb`) instances at PROM0 page 0 addresses
(0xa4d0, 0xa510, 0xa6c0, 0xc660, 0xd130, 0xd140, 0xd270, 0xd9e0, 0xdb40,
0xdde0) and 2 in PROM1 page 1 (0x1aad0, 0x1ab40). The DSP currently runs
in PROM1-mirror page 1, never branches across pages to reach any of them.

A diagnostic hack that bypassed the INTM check in `c54x_interrupt_ex`
(commit 306d6ec, reverted in f0dec53) caused massive NDB memory corruption
(every word from 0x01AC to 0x03B0 became 0xb908) — confirming INTM=1 marks
a critical section the DSP genuinely depends on. Don't bypass it.

### Next session entry points

1. **Disasm dispatcher at 0x18B00+** (PROM1 page 1, mirrored to 0xEB00) —
   pattern `76f8 XXXX YYYY` repeating looks like an STM-based jump table.
   Identify which entries lead anywhere useful and why none branch to the
   PROM0 init sequence containing RSBX INTM.
2. **Trace DSP boot from reset vector (0xFF80)** to the loop entry. There
   is a conditional somewhere that bypasses the RSBX INTM init block; find
   the condition.
3. **Inspect what flag the dispatcher polls** that should change to break
   it out of the loop. `d_dsp_page` cycles 0x0002↔0x0003 (= B_GSM_TASK |
   w_page) but the DSP doesn't act on it.
4. **Verify the `0x18B04` table entries point** to PROM1 (page-1 local) or
   PROM0 (cross-page). If purely page-1-local, the DSP can never reach the
   PROM0 RSBX INTM without an explicit XPC switch.

### Branch state (claude/refactor-and-cleanup-RopS8)

11 commits, all pushed:

| Commit | Subject | Status |
|--------|---------|--------|
| a72266d | cleanup opcode duplicates + .bak | ✓ keep |
| dab7925 | bsp: FN-indexed queue per TN (lookahead) | ✓ keep |
| 558e55f | bsp: POWERON re-anchor + tolerance window + delta log | ✓ keep |
| dbdff10 | bsp: rename shadowed local n→nh | ✓ keep |
| b2255fd | bridge: CLK IND from QEMU FN, not wall-clock | ✓ keep |
| a8218a8 | l1ctl_sock: sendmsg+iovec, no buf[514] hack | ✓ keep |
| 2241e01 | bsp: bump GMSK amplitude to full Q15 (±0x7FFE) | ✓ keep |
| 301f17c | bsp+c54x: instrument DARAM/PORTR/BSP-load | ✓ keep |
| 306d6ec | c54x: TEMP debug accept IRQ with INTM=1 | ✗ reverted |
| 82aee83 | c54x: uncap IRQ/RETE logs | ✓ keep |
| f0dec53 | c54x: revert INTM-bypass hack | ✓ keep (the revert) |

Live build at `/opt/GSM/qemu-src/` needs all `.c`/`.h` files from packaged
repo `/opt/GSM/qemu-calypso/` to be re-copied + ninja rebuild after pulling
the branch.

### Run config that revealed the most signal

```bash
# Mobile config must have `stick <arfcn>` in `ms 1` block, otherwise
# mobile abandons FBSB after 2 retries → d_task_md stays at 1.
CALYPSO_BSP_DARAM_ADDR=0x3fb0 ./run.sh
# DARAM 0x3fb0 covers the DSP-read range 0x3fb3-0x3fbf (verified via
# DARAM RD HIST). 0x3fc0 was off by 16 words.
```

## Old bugs (resolved)

**SP slow leak:** SP descends ~3 words per IDLE cycle (5AC7 → 5AC4). Introduced by F9xx CC fix (push now correct). Likely some CC calls in ROM where the callee doesn't RET properly — need to trace which CC target doesn't return.

**27 duplicate opcode handlers** in c54x_exec_one — consolidated in
a72266d (-150 lines, no behavior change).

## Conventions

- **No stubs** — the DSP handles PM/FB/SB/NB via real ROM code and shared API RAM
- **No hacks** — BDLENA gated by real TPU→TSP→IOTA chain, not bypass
- **QEMU is clock master** — bridge is slave, BTS receives CLK IND at wall-clock rate
- **Verify opcodes** against `tic54x-opc.c` before any C54x decode change
- **Test after each edit** — build in Docker, check DSP IDLE + SP + IMR

## Build

```bash
docker exec CONTAINER bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"
```

## Key Files

- `hw/arm/calypso/calypso_c54x.c` — DSP emulator (3500+ lines, opcode switch)
- `hw/arm/calypso/calypso_trx.c` — TRX/TPU/TSP/ULPD/SIM + TDMA tick + DMA
- `hw/arm/calypso/calypso_bsp.c` — BSP DMA + PORTR buffer + UDP 6702
- `hw/arm/calypso/calypso_iota.c` — IOTA BDLENA gate
- `hw/arm/calypso/l1ctl_sock.c` — L1CTL unix socket (/tmp/osmocom_l2)
- `hw/arm/calypso/sercomm_gate.c` — Sercomm DLCI router (PTY → FIFO)
- `hw/intc/calypso_inth.c` — INTH interrupt controller
- `hw/char/calypso_uart.c` — UART with RX FIFO + sercomm
- `bridge.py` — BTS UDP bridge (clock-slave)
- `run.sh` — Orchestrated launch (QEMU → bridge → BTS → mobile)
