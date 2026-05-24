# DIAG — DSP CCCH demod blocker (2026-05-08)

QEMU Calypso emulator. State after the **2026-05-08 hack purge**.

## What happened

The `rsl_si_tap.py` + `/dev/shm/calypso_si.bin` mmap + `CALYPSO_BCCH_INJECT`
shortcut was removed entirely. It sniffed the BSC↔BTS RSL TCP stream,
extracted BCCH SI bytes, and wrote them to a mmap that QEMU's
`calypso_fbsb.c` read on every `DSP_TASK_ALLC` to populate `a_cd[]` in
NDB directly, bypassing the DSP CCCH demod.

That shortcut "worked" but had two problems:
1. mmap survived BTS death → mobile camped on stale cache even after
   the BTS process exited. The "DL works end-to-end" claim was off cached
   bytes, not live BTS broadcast.
2. It hid the real DSP demod failure. As long as the inject worked, no
   one had to fix the DSP CCCH demod path.

Now removed:
- `scripts/rsl_si_tap.py` deleted
- `CALYPSO_BCCH_INJECT` env + `csi_*` + ALLC-inject block in `calypso_fbsb.c` deleted
- `CALYPSO_SI_MMAP_PATH` env deleted
- `run_si.sh` no longer launches the tap and clears `/dev/shm/calypso_si.bin` at startup

## Current state of the DL chain

```
osmo-bsc                 ← config source (cell_identity 888)
   ↓ RSL TCP (BCCH_INFO)
osmo-bts-trx             ← encodes BCCH SIs into GSM bursts
   ↓ TRXD UDP 5702       ← real DL bursts (encoded GMSK soft-bits, 154 B each)
bridge.py                ← UDP relay to QEMU BSP
   ↓ UDP 6702
QEMU BSP                 ← receives bursts, queues by FN
   ↓ DMA (PORTR PA=0x0034)
DSP CCCH demod           ← ⚠️ DOES NOT CONVERGE on bridge-fed GMSK ⚠️
   ↓ a_cd[] in NDB       ← never written
ARM L1                   ← reads a_cd[], finds no valid LAPDm
   ↓ L1CTL_DATA_IND      ← never sent
mobile L3                ← never receives SI → cell-search forever
```

## Effect on the test

Mobile no longer camps :
- `mobile.log` shows `MM_EVENT_NO_CELL_FOUND` repeatedly
- No `MON: ... CGI=001-01-1-888` line
- No `RR_EST_REQ` because mobile never gets to MM_IDLE/Camped state
- No RACH burst path exercised

UL chain (RACH/AGCH) cannot be tested as long as DL is broken.

## What needs to be debugged next

The DSP CCCH demod on bridge-fed GMSK. Concretely :

1. **Verify BSP receives the bursts** : count how many DL bursts hit
   the BSP UDP socket per second. Should be ~8/frame × 217 frame/s ≈
   1730/s wall but at qfn rate (so half).

2. **Verify BSP DMA queues them by FN match** : the queue match window
   `BSP_FN_MATCH_WINDOW` was thrashed (4 → 1024 → 64) without
   calibration. Wall_fn from BTS vs qfn from QEMU diverge by thousands
   of frames once running, so a 64-frame window will reject everything.
   Histograms of `delta_fn = bts_fn - qfn` on burst arrival are needed.

3. **Verify DSP reads samples via PORTR PA=0x0034** : trace ARM-side
   DARAM writes when DSP polls BSP. Confirm I/Q samples reach the DSP
   memory.

4. **DSP CCCH demod path itself** : does the demod converge ? FIRE
   check on decoded LAPDm — does it pass ? The DSP ROM at 0x7000-0xDFFF
   handles CCCH demod ; opcodes are emulated by `c54x_exec_one`.
   Suspect families : LD/STL on GMSK input buffer, channel-coding
   inverse, deinterleaver index table reads.

## Other knobs still available

| Env | Effect |
|---|---|
| `CALYPSO_FBSB_SYNTH=1` | Synth FB/SB publish (still env-gated, still useful for FB/SB phase). Doesn't help CCCH which is the new blocker. |
| `CALYPSO_W1C_LATCH=1` | W1C latch on a_sync_demod cells |
| `CALYPSO_NDB_D_RACH_OFFSET=0xNNN` | Override d_rach word index |
| `CALYPSO_RACH_FORCE_BSIC=N` | Force BSIC in RACH encoder |
| `BRIDGE_CLK_FROM_QEMU=0` (default) | CLK IND wall-paced. Default safe. |
| `BRIDGE_UL_FN_REWRITE=slot` (default) | Slot-aware rewrite, not exercised since no LU is reached. |

## Reproducer

```bash
cd /opt/GSM/qemu-src
unset CALYPSO_NDB_D_RACH_OFFSET CALYPSO_DSP_IDLE_RANGE CALYPSO_W1C_LATCH \
      CALYPSO_UART_TRACE BRIDGE_CLK_PERIOD BRIDGE_UL_FN_REWRITE \
      BRIDGE_CLK_FROM_QEMU
CALYPSO_FBSB_SYNTH=1 \
CALYPSO_DSP_FBDET_SKIP=1 CALYPSO_ICOUNT=off \
./run_si.sh
```

Expected mobile.log :
```
MM_EVENT_NO_CELL_FOUND  (repeated)
Changing CCCH_MODE to 2 (repeated)
using DSC of 90       (repeated)
```

NO `MON: ... CGI=...` line. That's the new ground-truth state.

## Files relevant to next work

| File | Why |
|---|---|
| `hw/arm/calypso/calypso_bsp.c` | DL receive (TRXD UDP) + DMA enqueue. `BSP_FN_MATCH_WINDOW` constant, `calypso_bsp_dl_enqueue` logic |
| `hw/arm/calypso/calypso_c54x.c` | DSP emulator, CCCH demod opcode execution |
| `hw/arm/calypso/calypso_trx.c` | BSP→DSP DMA tick, PORTR read |
| `bridge.py` | UDP relay (no parsing — pure transparent) |

## Open structural concerns from earlier review (still valid)

- `calypso_c54x.c` 4000 lines monolithic switch with dead code, duplicate F4xx handler 700 lines apart
- 3-way memory aliasing dsp->data ↔ api_ram ↔ MVPD reset path
- stderr writes in real-time DSP path can saturate host I/O
- `dsp_idle_fast_forward` can mask new bugs (no ff_hits/cycles metric)
- `BSP_FN_MATCH_WINDOW=64` unjustified (no fn_delta histogram)
- `gsm0503_rach_ext_encode(..., false)` hard-codes 8-bit RACH
- W1C latches without explicit memory barrier
- `calypso_uart.c::main_loop_wait(false)` re-entry risk
