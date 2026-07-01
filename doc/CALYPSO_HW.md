# Calypso Hardware Reference

## Chip Overview
TI Calypso (TWL3014/DBB) — GSM baseband processor
- ARM7TDMI CPU (ARM926EJ-S in QEMU, close enough)
- TMS320C54x DSP core
- Shared API RAM between ARM and DSP
- TPU (Time Processing Unit) for radio timing
- ABB (Analog Baseband) for RF
- BSP (Baseband Serial Port) for DSP↔ABB data
- DMA controller

## ARM Memory Map
| Address | Size | Peripheral |
|---------|------|-----------|
| 0x00000000 | 2MB | Internal ROM |
| 0x00800000 | 256KB | Internal SRAM |
| 0xFFFE0000 | | TPU registers |
| 0xFFFF0000 | | INTH (interrupt controller) |
| 0xFFFF1000 | | TPU control |
| 0xFFFC0000 | | UART modem |
| 0xFFFC8000 | | UART IrDA |
| 0xFFD00000 | 64KB | DSP API RAM (shared with C54x) |

## DSP API RAM (0xFFD00000)
- Total: 64KB (32K words of 16-bit)
- ARM accesses as bytes, DSP accesses as 16-bit words
- ARM byte offset X = DSP word address 0x0800 + X/2
- Double-buffered pages: Write page 0/1, Read page 0/1

## TPU
- Controls radio timing sequences
- TPU_CTRL register:
  - Bit 0: TPU_CTRL_EN (enable scenario execution)
  - Bit 2: TPU_CTRL_IDLE (idle status)
  - Bit 4: TPU_CTRL_DSP_EN (enable DSP frame interrupt)
- Firmware writes TPU scenarios then sets EN
- When scenario completes: clears EN, fires FRAME interrupt
- DSP_EN causes FRAME interrupt to also wake DSP

## INTH (Interrupt Controller)
- Level-sensitive
- IRQ 4: TPU_FRAME
- IRQ 0: Watchdog
- IRQ 7: UART

## TDMA Timing
- GSM frame: 4.615ms (216.7 frames/sec)
- 8 timeslots per frame
- Hyperframe: 2715648 frames
- QEMU uses 10x slowed timing for emulation stability

## DSP C54x Integration
- DSP boots from internal ROM at IPTR*128 (0xFF80 for IPTR=0x1FF)
- ARM communicates via API RAM (shared memory)
- ARM signals DSP via:
  - Writing d_dsp_page in NDB
  - Setting TPU_CTRL_DSP_EN
  - TPU generates FRAME interrupt to DSP (SINT17, vec 2)
- DSP signals ARM by writing results in Read page

## OsmocomBB Layer1 Flow
1. `l1_sync()` called every TDMA frame (TPU FRAME IRQ)
2. Updates page pointers (db_w, db_r)
3. Runs TDMA scheduler (`tdma_sched_execute`)
4. If tasks scheduled: writes d_task_d/d_task_md, calls `dsp_end_scenario()`
5. `dsp_end_scenario()`: writes d_dsp_page = B_GSM_TASK | w_page, toggles w_page
6. Enables TPU DSP frame interrupt
7. DSP wakes, reads d_dsp_page, dispatches task, writes results
8. Next frame: ARM reads results from db_r

## TRXD Protocol (BTS ↔ Bridge)
### DL (BTS → MS) — TRXD v0
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | TN (timeslot number, 3 bits) |
| 1-4 | 4 | FN (frame number, big-endian) |
| 5 | 1 | RSSI |
| 6-7 | 2 | TOA (big-endian) |
| 8+ | 148 | Soft bits (0=strong 1, 127=uncertain, 255=strong 0) |

### UL (MS → BTS) — TRXD v0
| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | TN |
| 1-4 | 4 | FN (big-endian) |
| 5 | 1 | PWR |
| 6+ | 148 | Hard bits (0/1) |

## Sercomm Protocol
- Flag: 0x7E
- Escape: 0x7D (next byte XOR 0x20)
- Frame: FLAG + DLCI + CTRL(0x03) + payload + FLAG
- DLCI 4: burst data
- DLCI 5: L1CTL messages

## L1CTL Protocol (mobile ↔ firmware)
- Length-prefixed: 2-byte big-endian length + message
- Message: type(1) + flags(1) + padding(2) + payload
- Key types:
  - 0x01: FBSB_REQ
  - 0x02: FBSB_CONF (result byte: 0=success, 255=fail)
  - 0x03: DATA_IND
  - 0x04: RACH_REQ
  - 0x05: DM_EST_REQ
  - 0x06: DATA_REQ
  - 0x07: RESET_IND (sent on boot)
  - 0x08: PM_REQ
  - 0x09: PM_CONF
  - 0x0D: RESET_REQ (payload: reset_type, 1=full)
  - 0x0E: RESET_CONF
  - 0x10: CCCH_MODE_REQ
  - 0x11: CCCH_MODE_CONF
