# TINT0 Master Clock Implementation Guide

## Context

The Calypso GSM phone has a single master clock: **TINT0** (Timer 0 Interrupt of the TMS320C54x DSP). On real hardware, the C54x Timer 0 (registers TIM/PRD/TCR at DSP addresses 0x0024-0x0026) runs off the 13 MHz oscillator and fires TINT0 every 4.615 ms (one TDMA frame). TINT0 is the heartbeat that synchronizes everything.

## Architecture

```
TINT0 (C54x Timer 0, 4.615ms)
  |
  +-- DSP: IFR bit 4 set -> if IMR bit 4 enabled and IDLE, wake DSP
  |        DSP ROM dispatcher reads d_dsp_page, processes GSM tasks
  |
  +-- TPU: frame sync signal -> burst scheduling for next frame
  |
  +-- ARM: TPU_FRAME IRQ (IRQ4) -> firmware L1 scheduler runs
  |        API IRQ (IRQ15) -> firmware reads DSP results
  |
  +-- UART: poll TX/RX for sercomm (L1CTL + burst data)
```

## Current State

### Done
- `calypso_tint0.c` / `calypso_tint0.h` created as dedicated TINT0 module
- QEMU timer fires at 4.615ms, calls `calypso_tint0_do_tick(fn)` in calypso_trx.c
- `calypso_tdma_hw.c/.h` removed (was a slave, never connected)
- Build passes, everything compiles

### What Still Needs Work

1. **DSP internal timer (TIM/PRD/TCR) should be driven by TINT0, not by instruction count**
   - Currently: `calypso_c54x.c` has a per-instruction timer tick in the main exec loop (lines ~2490-2520) that decrements TIM every instruction cycle. This is wrong.
   - Should be: TINT0 fires the DSP Timer 0 directly. The DSP ROM configures PRD/TCR but the actual tick comes from the QEMU timer, not from counting instructions.
   - The per-instruction timer tick in c54x.c should be removed. Instead, `calypso_tint0_do_tick()` should set `s->ifr |= (1 << 4)` directly (TINT0 bit in IFR).

2. **Remove duplicate timer logic in c54x main loop**
   - File: `/opt/GSM/qemu-src/hw/arm/calypso/calypso_c54x.c`
   - Lines ~2490-2520: the timer tick that decrements TIM/PRD per instruction
   - Also lines ~2515-2520: GSM frame number increment tied to TIM expiry
   - All of this should be removed - TINT0 handles frame timing

3. **TINT0 should directly set IFR bit 4 on the DSP**
   - In `calypso_tint0_do_tick()` or `calypso_tint0.c`:
     ```c
     s->dsp->ifr |= (1 << TINT0_IFR_BIT);  // bit 4
     ```
   - Then if `!(s->dsp->st1 & ST1_INTM) && (s->dsp->imr & (1 << 4))`:
     - Branch to interrupt vector TINT0_VEC (20) = IPTR*128 + 20*4
   - If DSP is IDLE: wake it

4. **Frame number should come from TINT0, not from DSP timer**
   - `calypso_tint0_fn()` is the authoritative frame number
   - Remove FN increment from c54x timer tick
   - The DSP reads FN from API RAM (written by ARM firmware via `dsp_end_scenario()`)

5. **Verify interrupt vector for TINT0**
   - TINT0 = IFR/IMR bit 4, vector 20 (offset 0x50 from IPTR base)
   - Currently `C54X_INT_FRAME_VEC=2, C54X_INT_FRAME_BIT=1` in calypso_c54x.h - this is **SINT17** (the frame interrupt from ARM/TPU), NOT TINT0
   - SINT17 (vec 2, bit 1) is triggered by ARM via TPU_CTRL_EN
   - TINT0 (vec 20, bit 4) is the DSP's own timer interrupt
   - Both exist on real hardware. SINT17 tells DSP "new frame data ready". TINT0 tells DSP "4.615ms elapsed".
   - The DSP ROM uses TINT0 internally for timing, and SINT17 for frame dispatch

## Files

| File | Role |
|------|------|
| `calypso_tint0.c` | QEMU timer, fires every 4.615ms, calls do_tick |
| `calypso_tint0.h` | Constants (TINT0_PERIOD_NS, IFR_BIT, VEC, etc.) |
| `calypso_trx.c` | `calypso_tint0_do_tick(fn)` - DSP run, SINT17, IRQs, UART |
| `calypso_c54x.c` | DSP emulator - remove internal timer tick |
| `calypso_c54x.h` | DSP state struct, interrupt constants |

## Docker

- Container: `trying`
- Source: `/opt/GSM/qemu-src/hw/arm/calypso/`
- Build: `cd /opt/GSM/qemu-src/build && ninja`
- Host sync: `docker cp trying:/opt/GSM/qemu-src/. /home/nirvana/QEMU/`

## Key Constants

```c
#define TINT0_PERIOD_NS  4615000ULL  // 4.615 ms
#define TINT0_IFR_BIT    4           // IFR/IMR bit 4
#define TINT0_VEC        20          // Interrupt vector 20
#define TINT0_TIM_ADDR   0x0024      // DSP Timer counter
#define TINT0_PRD_ADDR   0x0025      // DSP Timer period
#define TINT0_TCR_ADDR   0x0026      // DSP Timer control
#define GSM_HYPERFRAME   2715648     // GSM hyperframe modulus
```

## Current DSP Status

- DSP boots, SP stable at 0x5AC6, executes real ROM code
- F272=RPTBD and F274=CALLD correctly decoded
- d_dsp_page WR=0x0002 received from ARM
- DSP running init code at PC=0x1200-0x1C00 (DARAM), IMR=0x0000
- DSP never reaches IDLE - init loop may be waiting for TINT0 to fire
- Once TINT0 properly sets IFR bit 4, the DSP should detect it, configure IMR, and start processing frames
