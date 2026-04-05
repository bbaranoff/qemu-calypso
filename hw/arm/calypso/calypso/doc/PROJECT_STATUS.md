# Calypso GSM Baseband Emulator — Project Status

## Goal
Run real OsmocomBB layer1.highram.elf firmware on emulated TI Calypso (ARM7 + TMS320C54x DSP) in QEMU. Connect to a real BTS via TRX protocol through a bridge. Mobile/ccch_scan sees the BTS and decodes BCCH/CCCH.

## Architecture
```
BTS (osmo-bts-trx)
  ↕ UDP (TRXC 5701, TRXD 5702, CLK 5700)
bridge.py
  ↕ PTY/UART (sercomm DLCI 4 = bursts, DLCI 5 = L1CTL)
QEMU (ARM7 calypso_trx.c + C54x calypso_c54x.c)
  ↕ Unix socket /tmp/osmocom_l2_1 (L1CTL length-prefixed)
mobile / ccch_scan (OsmocomBB host tools)
```

## Key Files
| File | Role |
|------|------|
| `calypso_c54x.c` | TMS320C54x DSP emulator (~1800 lines) |
| `calypso_c54x.h` | DSP state struct, constants, API |
| `calypso_trx.c` | Calypso SoC: DSP API RAM, TPU, TDMA, burst RX |
| `l1ctl_sock.c` | Unix socket L1CTL ↔ sercomm bridge |
| `calypso_uart.c` | UART with sercomm DLCI routing |
| `calypso_inth.c` | ARM interrupt controller |
| `calypso_soc.c` | SoC glue, memory map |
| `calypso_mb.c` | Machine/board definition |
| `bridge.py` | BTS TRX UDP ↔ UART sercomm bridge |
| `run.sh` | Launches QEMU + bridge + BTS + ccch_scan in tmux |
| `sync.sh` | Syncs files between host and docker container |
| `l1ctl_test.py` | Direct L1CTL test script |

## DSP ROM
- Dumped from real Motorola C1xx phone via osmocon + ESP32
- File: `/opt/GSM/calypso_dsp.txt` (131168 words)
- Sections: Registers [00000-0005f], DROM [09000-0dfff], PDROM [0e000-0ffff], PROM0 [07000-0dfff], PROM1 [18000-1ffff], PROM2 [28000-2ffff], PROM3 [38000-39fff]
- DSP version: 0x3606

## Memory Map (DSP side)
| DSP Address | ARM Address | Content |
|-------------|-------------|---------|
| 0x0000-0x001F | — | MMR (registers) |
| 0x0020-0x007F | — | DARAM (low, scratch) |
| 0x0080-0x07FF | — | DARAM (overlay with prog when OVLY=1) |
| 0x0800-0x27FF | 0xFFD00000-0xFFD03FFF | API RAM (shared ARM↔DSP) |
| 0x7000-0xDFFF | — | PROM0 (program ROM) |
| 0x8000-0xFFFF | — | PROM1 mirror (program ROM page 1) |
| 0x9000-0xDFFF | — | DROM (data ROM) |
| 0xE000-0xFFFF | — | PDROM (patch data ROM) |

## API RAM Layout
| ARM Offset | DSP Address | Name |
|------------|-------------|------|
| 0x0000 (W_PAGE_0) | 0x0800 | Write page 0 (MCU→DSP, 20 words) |
| 0x0028 (W_PAGE_1) | 0x0814 | Write page 1 |
| 0x0050 (R_PAGE_0) | 0x0828 | Read page 0 (DSP→MCU, 20 words) |
| 0x0078 (R_PAGE_1) | 0x083C | Read page 1 |
| 0x01A8 (NDB) | 0x08D4 | d_dsp_page (first word of NDB) |
| 0x0862 (PARAM) | 0x0C31 | DSP parameters |

## d_dsp_page values
- `0x0000` = no task
- `0x0002` = B_GSM_TASK | page 0
- `0x0003` = B_GSM_TASK | page 1
- B_GSM_TASK = (1 << 1) = 0x0002

## Write page structure (T_DB_MCU_TO_DSP, 20 words)
| Offset | Field |
|--------|-------|
| 0 | d_task_d (downlink task) |
| 1 | d_burst_d |
| 2 | d_task_u (uplink task) |
| 3 | d_burst_u |
| 4 | d_task_md (monitoring/FB/SB task: 5=FB, 6=SB) |
| 5 | d_background |
| 6 | d_debug |
| 7 | d_task_ra |
| 8-19 | results area |

## DSP Boot Sequence
1. ARM writes DSP_DL_STATUS_READY
2. `c54x_reset()` → PMST=0xFFE0, IPTR=0x1FF, OVLY=0, PC=0xFF80
3. `c54x_run(10M)` → DSP executes PROM1 reset code → calls PROM0 init → 86K insns → IDLE@0xFFFE
4. ARM continues, firmware initializes
5. Each TDMA frame: ARM writes d_dsp_page + tasks, fires TPU_CTRL_EN
6. `calypso_dsp_done()` → copies write page to DARAM 0x0586, wakes DSP
7. DSP jumps to 0x8000 (TDMA slot table), processes, returns to IDLE

## TDMA Slot Table (PROM1 0x8000-0x801F)
```
0x8000: 12f8 3fc5 f4e3 f4e4  → slot 0: SUB data[0x3FC5], A; SSBX INTM; IDLE
0x8004: 12f8 322a f4e3 f4e4  → slot 1
...8 slots of 4 words each...
0x8020: processing code starts (reads d_dsp_page, dispatches tasks)
```

## Frame Dispatch Routine (PROM0)
- Entry: 0xC8E7 → reads d_dsp_page at DSP 0x08D4
- Configures page pointers (0x0800/0x0814/0x0828/0x083C)
- Called via BANZ at 0xC8CD
- d_dsp_page also read at 0xA51C during init

## Bugs Fixed This Session
1. **Timer HW (TCR/PSC/TDDR)** — Real C54x timer behavior, TSS=1 at reset
2. **IDLE PC** — return 0 (stay at IDLE addr for interrupt handler)
3. **PC wrap 16-bit** — `s->pc &= 0xFFFF`
4. **F4E2/F4E3 (RSBX/SSBX INTM)** — 98 occurrences decoded as BD instead of interrupt enable/disable. CRITICAL BUG.
5. **Interrupt vec = bit + 16** then reverted to **SINT17 vec 2 bit 1** — correct vector for TPU frame
6. **FRET** — Must pop 2 words (XPC then PC) per SPRU172C p.4-61
7. **FCALL** — Must push 2 words (PC+2 then XPC) per SPRU172C p.4-57
8. **XC (Execute Conditionally)** — Opcode 0xFDxx (n=1) / 0xFFxx (n=2), NOT 0xEAxx. 0xEA = BANZ.
9. **TRXD header** — 8 bytes (TN+FN+RSSI+TOA), not 6. Soft bit conversion.
10. **L1CTL sequencing** — 1 message per callback to let ARM process between messages
11. **OVLY activation** — Enabled after boot for DARAM code execution
12. **IDLE wake** — Jump to 0x8000 (TDMA loop) on wake from boot IDLE
13. **Interrupt PC+1** — Push PC+1 when waking from IDLE (resume after IDLE)
14. **Timer FN increment** — TINT0 increments frame number in DARAM

## Current State (2026-04-03)
- Base: no_cell_found (API RAM intercepts for FB/SB/PM)
- No_cell_found loop works: RESET→PM→FBSB cycle running
- C54x DSP boots in parallel via TDMA ticks (2000 insns/frame)
- SINT17 controlled by TPU_CTRL_DSP_EN (per hardware spec)
- 0 UNIMPL instructions — all opcodes emulated
- SP=0x06AA-0x0C2E after boot (correct, in DARAM)
- DSP reaches PROM0 0x7000 init + PROM1 0x8159 processing

## Bugs Fixed (session 2026-04-03)
1. **RC conditionnel (F2xx)** — was unconditional RET, now eval_condition
2. **PROM0 read protection** — prog_read returns prog[] for 0x7000-0xDFFF
3. **PROM0 write protection** — prog_write ignores writes to ROM area
4. **prog_write double-write** — return after prog[ext] for addr>=0x8000
5. **DELAY instruction (D4/D5)** — pipeline delay, was UNIMPL
6. **BCD 0xEF** — added to EE/ED branch conditional handler
7. **XOR/OR #lk16 (B0/B1/B8/B9)** — was UNIMPL
8. **Timer0 hardware** — TIM/PRD/TCR with prescaler and TINT0
9. **IDLE skip TDMA slots** — 0x8000-0x801F IDLE treated as NOP
10. **Parallel DSP boot** — no blocking c54x_run(10M), boot via TDMA ticks
11. **DSP_EN SINT17** — interrupt only when firmware sets TPU_CTRL_DSP_EN
12. **eval_condition** — full XC/RC condition decoder per SPRU172C Table 3-2
13. **Interrupt dispatch** — check IFR&IMR in main loop each cycle
14. **API IRQ** — raise IRQ15 in dsp_done, unmask at boot

## d_fb_det Location
- ARM offset: 0x01F0 (NDB + 0x48)
- DSP address: 0x08F8
- DSP ROM writes it at PROM0 0x7730-0x7990

## Next Steps
1. DSP boot must reach IDLE@0xFFFE — currently runs but doesn't converge
2. Once boot works, remove API RAM intercepts (let DSP produce results)
3. Verify burst samples reach DSP at correct DARAM address
4. Achieve FBSB_CONF(result=0) → SB decode → BCCH → mobile registered

## Docker
- Container: `osmo-operator-1` (always running)
- Images: `calypso-qemu:YYYYMMDD-HHMM` (snapshots)
- Build: `ninja -C /opt/GSM/qemu-src/build`
- Run: `bash /opt/GSM/qemu-src/run.sh`
- DSP ROM: `/opt/GSM/calypso_dsp.txt`
- Firmware: `/opt/GSM/firmware/board/compal_e88/layer1.highram.elf`

## Reference Documents
- `doc/spru172c.pdf` — TMS320C54x DSP Reference Set Volume 2: Mnemonic Instruction Set
- `doc/C54X_INSTRUCTIONS.md` — Key instruction encodings extracted from SPRU172C
