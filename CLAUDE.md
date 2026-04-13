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

## Current Bug

**2nd PM_REQ not scheduled.** Mobile sends PM_REQ (type=0x08), firmware dequeues it from l23_rx_queue, but `l1s_pm_test()` is never called. GDB breakpoint on 0x825424 not hit. First PM works perfectly. Likely another opcode bug in the l1ctl_rx_pm_req → l1s_pm_test path, or a state corruption during 1st PM that prevents 2nd.

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
