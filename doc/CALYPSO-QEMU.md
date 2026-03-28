# Calypso GSM Baseband Emulation in QEMU

## Overview

This project emulates a TI Calypso GSM baseband processor in QEMU, running the OsmocomBB Layer 1 firmware. The emulation includes the full hardware stack needed for GSM cell selection: DSP API, TPU timing, interrupt controller, UART/sercomm, and SIM controller.

The system connects to OsmocomBB's `mobile` application via the standard L1CTL protocol, enabling a complete GSM phone stack to run against emulated (or real, via SDR) radio data.

```
┌─────────────┐     L1CTL/unix      ┌──────────────┐    sercomm/PTY     ┌─────────────────┐
│   mobile    │◄────────────────────►│ l1ctl_bridge │◄──────────────────►│  QEMU Calypso   │
│ (layer23)   │   /tmp/osmocom_l2   │    (.py)     │   /dev/pts/X      │  (firmware L1)  │
└─────────────┘                     └──────────────┘                    └────────┬────────┘
                                         │ PM intercept                          │
                                         │ (fake PM_CONF)                        │ TRXD/TRXC
                                                                                 │ UDP 6700/6701
                                                                           ┌─────▼─────┐
                                                                           │  BTS/SDR  │
                                                                           │ (future)  │
                                                                           └───────────┘
```

## Quick Start

```bash
# Build
./configure --target-list=arm-softmmu --disable-docs
cd build && ninja

# Run (no BTS needed for basic L1CTL testing)
cd .. && ./run-mobile.sh

# Or manually:
./build/qemu-system-arm -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor unix:/tmp/qemu-calypso-mon.sock,server,nowait \
  -kernel /path/to/layer1.highram.elf
```

## Architecture

### Emulated Hardware

| Component | File | Base Address | Description |
|-----------|------|-------------|-------------|
| **DSP API** | `calypso_trx.c` | `0xFFD00000` | Shared memory MCU↔DSP, task scheduling |
| **TPU** | `calypso_trx.c` | `0xFFFF1000` | Timing/sequencer, frame IRQ, scenario execution |
| **TSP** | `calypso_trx.c` | `0xFFFE0800` | RF synthesizer control (stub) |
| **ULPD** | `calypso_trx.c` | `0xFFFE2800` | Clock management, GSM timer |
| **SIM** | `calypso_trx.c` | `0xFFFE0000` | SIM card controller (ATR stub) |
| **INTH** | `calypso_inth.c` | `0xFFFFFA00` | 32-input interrupt controller |
| **UART×2** | `calypso_uart.c` | `0xFFFF5800/5000` | Modem + IrDA UARTs |
| **Timer×2** | `calypso_timer.c` | `0xFFFE3800/3C00` | Hardware timers |
| **SPI** | `calypso_spi.c` | — | ABB/TWL3025 interface |
| **I2C** | `calypso_i2c.c` | `0xFFFE1800` | I2C controller |
| **Flash** | pflash_cfi01 | `0x00000000` | 4 MiB Intel 28F320J3 |
| **IRAM** | — | `0x00800000` | 256 KiB internal RAM |
| **XRAM** | — | `0x01000000` | 8 MiB external RAM |

### DSP API Memory Layout

**Critical discovery**: The Calypso DSP page layout is NOT at the standard TCS211 offsets (0x2000/0x3000). The real layout, confirmed by firmware disassembly:

```
DSP Base: 0xFFD00000

Write Pages (MCU → DSP):
  W_PAGE0: offset 0x0000  (firmware writes d_task_d/u/md here)
  W_PAGE1: offset 0x0028

Read Pages (DSP → MCU):
  R_PAGE0: offset 0x0050  (firmware reads a_pm[] from here)
  R_PAGE1: offset 0x0078

NDB:       offset 0x4000  (shared non-paged data)
PARAM:     offset 0x4800
```

**PM result location**: `a_pm[0]` at read_page + 24 bytes (word offset 12 from page base):
- R_PAGE0: `dsp_ram[52]` (0xFFD00068)
- R_PAGE1: `dsp_ram[72]` (0xFFD00090)

### DB Write Page Structure (word offsets)

```
Word 0: d_task_d    — DL task command
Word 1: d_burst_d   — DL burst ID
Word 2: d_task_u    — UL task command
Word 3: d_burst_u   — UL burst ID
Word 4: d_task_md   — Monitoring/PM task
```

## L1CTL Protocol Flow

### Complete Sequence (verified working)

```
Mobile              Bridge              Firmware
  │                   │                    │
  ├─ RESET_REQ ──────►├─── sercomm ───────►│ Boot, DSP init
  │                   │                    │ SIM ATR
  ◄─ RESET_CONF ─────┤◄── sercomm ────────┤
  │                   │                    │
  ├─ PM_REQ (E-GSM) ─►│ INTERCEPT         │
  ◄─ PM_CONF ────────┤ ARFCN 1022=-62dBm  │ (not forwarded)
  │                   │                    │
  ├─ PM_REQ (GSM900) ►│ INTERCEPT         │
  ◄─ PM_CONF ────────┤ all noise          │
  │                   │                    │
  ├─ PM_REQ (DCS1800)►│ INTERCEPT         │
  ◄─ PM_CONF ────────┤ all noise          │
  │                   │                    │
  │ cell selection: ARFCN 1022            │
  │                   │                    │
  ├─ FBSB_REQ ───────►├─── sercomm ───────►│ FCCH/SCH search
  ◄─ FBSB_CONF ──────┤◄── sercomm ────────┤ result=255 (no BTS)
```

### L1CTL Message Types

| Type | Hex | Direction | Description |
|------|-----|-----------|-------------|
| RESET_REQ | 0x0D | mob→fw | Reset L1, type=FULL(1) |
| RESET_CONF | 0x0E | fw→mob | Reset acknowledged |
| PM_REQ | 0x08 | mob→fw | Power measurement request (ARFCN range) |
| PM_CONF | 0x09 | fw→mob | PM results (batches of 50) |
| FBSB_REQ | 0x03 | mob→fw | Frequency/Sync burst search |
| FBSB_CONF | 0x04 | fw→mob | FBSB result (0=success, 255=fail) |

### PM_CONF Entry Format

```
Offset  Size  Field
0       2     band_arfcn (big-endian)
2       1     pm1 (rxlev, 0-63)
3       1     pm2 (rxlev, second measurement)
```

rxlev to dBm: `dBm = -110 + rxlev`

## Firmware Binary Patches

The emulator applies runtime patches to the OsmocomBB firmware at first UART access (before any console output). These are necessary because the emulated hardware timing differs from real Calypso.

### Patch Summary

| Address | Original | Patched | Purpose |
|---------|----------|---------|---------|
| `0x82a1b0` | `push {r4-r8,lr}` | `bx lr` | NOP `cons_puts` — boot debug fills msgb pool |
| `0x829ea0` | `b sercomm_puts` | `bx lr` | NOP `puts` — LOST messages fill pool |
| `0x82c33c` | `cmp r3, #32` | `cmp r3, #148` | Expand msgb pool 32→148 slots |
| `0x82c34c` | `bl cons_puts` | `msr CPSR_c, r8` | Talloc panic: re-enable IRQs |
| `0x82c350` | `b self` | `b 0x82c2d4` | Talloc panic: retry slot scan |
| `0x821f94` | `msr CPSR_c, #0xc0` | `msr CPSR_c, #0` | Data abort: keep IRQs enabled |
| `0x828914` | `bl printf` | `nop` | NOP "LOST %d!" in frame_irq |
| `0x828828` | `bl printf` | `nop` | NOP debug printf in frame_irq |
| `0x828830` | `bl puts` | `nop` | NOP debug puts in frame_irq |
| `0x828858` | `blgt printf` | `nop` | NOP conditional debug in frame_irq |
| `0x828880` | `blhi printf` | `nop` | NOP conditional debug in frame_irq |

### Why These Patches Are Needed

**The msgb pool problem**: OsmocomBB firmware uses a 32-slot talloc pool (332 bytes/slot at `0x833b5c`) for all message buffers. Console output (`printf`, `puts`, `cons_puts`) allocates from this pool via sercomm. During boot and PM measurement, debug output fills the pool faster than UART TX can drain it, causing a talloc panic (infinite loop at `0x82c350`).

**Solution**:
1. Suppress high-frequency debug output (NOP specific `bl printf/puts` calls)
2. Expand pool to 148 slots (max safe within IRAM: `0x833b5c + 148×332 = 0x83fb4c < 0x840000`)
3. Replace talloc panic with retry loop that re-enables IRQs
4. Keep data abort handler's IRQ enabled to prevent system freeze

### Key Firmware Symbols

```
0x82a1b0  cons_puts         — Console string output via sercomm
0x829ee8  printf            — Formatted output via sercomm
0x829ea0  puts              — String output (calls sercomm_puts)
0x82c2bc  _talloc_zero      — Pool allocator (32→148 slots)
0x833b5c  talloc pool       — 148 × 332 byte slots in IRAM
0x8287b4  frame_irq         — TDMA frame interrupt handler (L1S)
0x825454  l1s_pm_resp       — PM measurement result handler
0x8284c8  agc_inp_dbm8_by_pm — PM raw → dBm×8 conversion
0x821f5c  handle_abort      — Data/prefetch abort handler
0x821f9c  irq_entry         — IRQ exception entry point
0x82f9c4  l1s_dsp_com       — DSP communication struct (+4=db_r_ptr, +8=db_w_ptr)
```

## PM Calibration

The firmware converts raw DSP PM values to dBm using `agc_inp_dbm8_by_pm`:

```
result_dbm8 = (PM_raw >> 3) - (rffe_gain + agc_byte) × 8
```

With default gain ≈ 138:
- `PM_raw = 0` → 0 dBm baseband, -138 dBm RF
- `PM_raw = 4864` → 608 dBm baseband, **-62 dBm RF** ← target value

## Interrupt Controller (INTH)

32 inputs, level-sensitive, priority-based arbitration.

| IRQ | Name | Notes |
|-----|------|-------|
| 0 | WATCHDOG | Periodic, handled by firmware |
| 4 | TPU_FRAME | TDMA frame tick (46.15ms slowed 10×) |
| 5 | TPU_PAGE | TPU page completion |
| 6 | SIM | SIM card events |
| 7 | UART_MODEM | Sercomm TX/RX |
| 15 | API | DSP task completion |
| 18 | UART_IRDA | Secondary UART |

**Read-to-ack**: Reading `IRQ_NUM` (offset 0x10) returns the highest-priority pending IRQ and clears its level for edge-like sources (TPU_FRAME, TPU_PAGE). True level-sensitive sources (UART) are controlled by their peripheral.

## UART Emulation

### TX Burst Drain

The UART implements a "burst drain" optimization: when the firmware reads IIR and sees TX_EMPTY, `thr_empty_pending` is NOT cleared on the first read. This allows the firmware's sercomm ISR to loop and drain multiple TX bytes per interrupt invocation instead of one byte per ISR call.

A `tx_empty_reads` counter tracks consecutive IIR reads without THR writes. After 2 reads without a write (ISR has nothing left to send), pending is cleared.

### RX Polling

A REALTIME timer polls the chardev backend every 50ms to ensure PTY data is delivered even when the CPU is in a tight loop. The TDMA tick also calls `poll_backend()` and `kick_rx()` each frame.

## TRX Interface

UDP sockets for future BTS/SDR integration:

| Port | Protocol | Direction | Description |
|------|----------|-----------|-------------|
| 6700 | TRXC | ASCII | Command/response (POWERON, RXTUNE, etc.) |
| 6701 | TRXD | Binary | Burst data (TRXD v0 format) |
| 4729 | GSMTAP | Binary | Wireshark monitoring |

## ARFCN Sync State Machine

Built-in state machine for FCCH/SCH synchronization (not yet connected to real radio data):

```
SYNC_IDLE → SYNC_FCCH_SEARCH → SYNC_FCCH_FOUND → SYNC_SCH_SEARCH → SYNC_LOCKED
```

- FB detection: injects synthetic result after `SYNC_FB_DETECT_DELAY` frames
- SB decode: encodes SCH with BSIC/FN using `sch_encode()`
- Default: ARFCN 1022, BSIC 0x3C (NCC=7, BCC=4), RSSI -62 dBm

## File Map

```
hw/arm/calypso/
├── calypso_mb.c          Machine/board definition
├── calypso_soc.c         SoC: memory map, peripheral instantiation
├── calypso_trx.c         DSP/TPU/TRX/SIM emulation + firmware patches
├── meson.build
└── Kconfig

hw/char/calypso_uart.c    UART with sercomm-aware TX burst drain
hw/intc/calypso_inth.c    32-input interrupt controller
hw/timer/calypso_timer.c  Hardware timers
hw/ssi/calypso_spi.c      SPI (ABB/TWL3025)
hw/ssi/calypso_i2c.c      I2C controller

include/hw/arm/calypso/
├── calypso_trx.h         DSP API defines, PM calibration, sync FSM
├── calypso_uart.h        UART state with TX burst drain
├── calypso_soc.h         SoC type declarations
├── calypso_inth.h        INTH type declarations
├── calypso_timer.h       Timer type declarations
└── calypso_spi.h         SPI type declarations

l1ctl_bridge.py           Sercomm↔L1CTL relay + PM intercept
gsm_cfile_bridge.py       GSM cfile→TRXD bridge (via grgsm_decode)
run-mobile.sh             One-command launcher
test-mobile.sh            Automated test script
```

## Building

```bash
# Prerequisites: standard QEMU build deps + arm cross toolchain for firmware
sudo apt install build-essential ninja-build python3-venv libglib2.0-dev libpixman-1-dev

# Configure (arm-softmmu only for faster build)
./configure --target-list=arm-softmmu --disable-docs
cd build && ninja -j$(nproc)

# Verify
./qemu-system-arm -M help | grep calypso
```

## Known Limitations

- **No real radio**: FBSB returns result=255 without BTS/SDR connected
- **TDMA slowed 10×**: 46.15ms frame period (vs 4.615ms real GSM) for emulation headroom
- **DSP page toggle**: Always page 0 during PM (firmware doesn't toggle for monitoring tasks)
- **Firmware-specific patches**: Addresses hardcoded for `layer1.highram.elf` (osmocon_v0.0.0-1784-g9516a0e5-modified)
- **PM via bridge**: Power measurements are faked by the bridge, not by DSP emulation
