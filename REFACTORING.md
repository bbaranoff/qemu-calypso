# Code Refactoring Guide

## Current File Structure

```
hw/arm/calypso/
  calypso_c54x.c    (~2600 lines) - TMS320C54x DSP emulator
  calypso_c54x.h    (~170 lines)  - DSP state struct, constants
  calypso_trx.c     (~610 lines)  - Calypso HW glue (DSP-ARM, TPU, TSP, SIM)
  calypso_trx.h     (~80 lines)   - TRX definitions, IRQ numbers
  calypso_tint0.c   (~100 lines)  - TINT0 master clock (NEW)
  calypso_tint0.h   (~55 lines)   - TINT0 constants (NEW)
  calypso_mb.c                     - Machine board
  calypso_soc.c                    - SoC init
  l1ctl_sock.c                     - L1CTL unix socket (firmware <-> mobile)
  meson.build                      - Build config
```

## What Was Refactored

### TINT0 Extraction
- **Before**: Timer creation, tick callback, start/stop all inside calypso_trx.c
- **After**: calypso_tint0.c owns the QEMU timer. calypso_trx.c exposes `calypso_tint0_do_tick(fn)` as the work callback.
- **Removed**: calypso_tdma_hw.c/h (was a "slave" TDMA counter, never connected to build)

### What Still Needs Refactoring

1. **Remove DSP internal timer from c54x main loop**
   - The per-instruction TIM/PRD/TCR tick at lines ~2490-2520 of calypso_c54x.c
   - TINT0 should be the only timer source
   - Remove the FN increment from timer expiry (FN comes from calypso_tint0)

2. **Clean up duplicate F0/F1 handler blocks in c54x**
   - Two identical `if (hi8 == 0xF0 || hi8 == 0xF1)` blocks
   - The second one (starting around line 736) is dead code
   - Delete the second block entirely

3. **Separate DSP instruction decoder into its own file**
   - calypso_c54x.c is 2600 lines and growing
   - Could split: c54x_decode.c (instruction handlers), c54x_core.c (exec loop, memory, interrupts), c54x_rom.c (ROM loader)

4. **calypso_trx.c does too much**
   - Currently handles: DSP API, TPU regs, TSP regs, ULPD regs, SIM regs, bootloader protocol, burst I/O, frame processing
   - Could split: calypso_tpu.c, calypso_sim.c, calypso_dsp_api.c

5. **Normalize header includes**
   - Some files use `#include "hw/arm/calypso/foo.h"`, others use `#include "foo.h"`
   - Standardize to one convention
