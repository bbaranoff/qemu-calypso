# Calypso Hardware Map

## Memory

| Address | Size | Description |
|---------|------|-------------|
| `0x00000000` | 64 B | Boot ROM (exception vectors → IRAM handlers) |
| `0x00000000` | 4 MiB | Flash (pflash_cfi01, Intel 28F320J3, overlapped by boot ROM) |
| `0x00800000` | 256 KiB | IRAM (firmware code + data + msgb pool) |
| `0x01000000` | 8 MiB | XRAM |

## Peripherals

| Address | Size | Peripheral | File |
|---------|------|------------|------|
| `0xFFFE0000` | 0x100 | SIM controller | `calypso_trx.c` |
| `0xFFFE0800` | 0x100 | TSP (Time Serial Port) | `calypso_trx.c` |
| `0xFFFE1800` | 0x100 | I2C | `calypso_i2c.c` |
| `0xFFFE2800` | 0x100 | ULPD (clock/power) | `calypso_trx.c` |
| `0xFFFE3000` | 0x100 | SPI + TWL3025 ABB | `calypso_spi.c` |
| `0xFFFE3800` | 0x100 | Timer 1 | `calypso_timer.c` |
| `0xFFFE3C00` | 0x100 | Timer 2 | `calypso_timer.c` |
| `0xFFFE4800` | 0x100 | Keypad (stub) | `calypso_soc.c` |
| `0xFFFE6800` | 0x100 | Timer 6800 (stub) | `calypso_soc.c` |
| `0xFFD00000` | 64 KiB | DSP API RAM | `calypso_trx.c` |
| `0xFFFF1000` | 0x100 | TPU registers | `calypso_trx.c` |
| `0xFFFF5000` | 0x100 | UART IrDA | `calypso_uart.c` |
| `0xFFFF5800` | 0x100 | UART Modem (sercomm) | `calypso_uart.c` |
| `0xFFFF9000` | 0x800 | TPU RAM | `calypso_trx.c` |
| `0xFFFFFA00` | 0x100 | INTH (interrupt controller) | `calypso_inth.c` |
| `0xFFFFF000` | 0x100 | DPLL (stub) | `calypso_soc.c` |
| `0xFFFFF900` | 0x100 | RHEA bridge (stub) | `calypso_soc.c` |
| `0xFFFFFB00` | 0x100 | CLKM (stub) | `calypso_soc.c` |
| `0xFFFFFF00` | 0x100 | DIO (stub) | `calypso_soc.c` |

## IRQ Map (INTH)

| IRQ | Source | Notes |
|-----|--------|-------|
| 0 | Watchdog | Periodic |
| 1 | Timer 1 | |
| 2 | Timer 2 | |
| 3 | TSP_RX | |
| 4 | **TPU_FRAME** | TDMA frame tick, edge-like ack |
| 5 | TPU_PAGE | Edge-like ack |
| 6 | SIM | |
| 7 | **UART_MODEM** | Sercomm TX/RX, level-sensitive |
| 8 | Keypad/GPIO | |
| 9 | RTC Timer | |
| 13 | SPI | |
| 15 | **API** | DSP task completion |
| 18 | UART_IRDA | |

## DSP API Layout (confirmed by disassembly)

**Base: `0xFFD00000`**

The page layout is NOT at standard TCS211 offsets. Pages are small and interleaved:

```
Offset  Name        Description
0x0000  W_PAGE0     MCU→DSP write page 0 (d_task_d/u/md at words 0-4)
0x0028  W_PAGE1     MCU→DSP write page 1
0x0050  R_PAGE0     DSP→MCU read page 0 (a_pm[] at +24 bytes = word 12)
0x0078  R_PAGE1     DSP→MCU read page 1
0x4000  NDB         Non-paged shared data (d_fb_det, a_cd[], a_sch26[])
0x4800  PARAM       DSP parameters
```

### PM result location

| Page | a_pm[0] address | dsp_ram index |
|------|----------------|---------------|
| 0 | `0xFFD00068` | `dsp_ram[52]` |
| 1 | `0xFFD00090` | `dsp_ram[72]` |

### Key firmware addresses (layer1.highram.elf)

```
0x820000  _start (entry point)
0x821f5c  handle_abort
0x821f9c  irq_entry
0x825424  l1s_pm_test
0x825454  l1s_pm_resp (reads a_pm from DSP)
0x8284c8  agc_inp_dbm8_by_pm
0x8287b4  frame_irq (L1S TDMA handler)
0x829ea0  puts → sercomm_puts
0x829ee8  printf
0x82a1b0  cons_puts
0x82c2bc  _talloc_zero (msgb pool allocator)
0x833b5c  msgb pool base (332 bytes/slot, expanded to 148 slots)
0x82f9c4  l1s_dsp_com (+4=db_r_ptr, +8=db_w_ptr, +0x14=page flag)
```
