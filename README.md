# qemu-calypso

QEMU emulation of the **TI Calypso GSM baseband** chipset (ARM7TDMI + TMS320C54x DSP),
the SoC used in the OpenMoko Neo / Compal e88 family. Runs unmodified real DSP ROM and
the [osmocom-bb](https://osmocom.org/projects/baseband/) `layer1.highram.elf` firmware
on the ARM side, with a Python bridge that connects to `osmo-bts-trx` for full
GSM cell simulation.

## Latest update — Session 2026-04-29

Five structural opcode-dispatch fixes validated empirically. ~2530 firmware sites
unblocked. DSP CPU emulation now silicon-aligned against binutils tic54x-opc.c +
SPRU172C/131G + 3 FreeCalypso ROM dumps.

| # | Fix | Impact |
|---|---|---|
| 1 | Silicon-aligned reset (PMST=0xFFA8, ST0=0x181F, ST1=0x2900) | DSP enters PROM1 init zone |
| 2 | 0x6F00 ext dispatch | Wedge PC=0x8353 (2.2G iter) eliminated |
| 3 | 0x68-0x6E handlers (ANDM/ORM/XORM/ADDM/BANZ/BANZD) | 1563 sites unblocked |
| 4 | APTS misnomer fix (bit 4 PMST = AVIS, no stack semantics) | Stack leak 1.96M events → 0 |
| 5 | F3xx complete (AND/OR/XOR/SFTL + #lk variants) | 364 sites, wedge PC=0x8eb9 unblocked |

**Final blocker:** INTM=1 forever (silicon mechanism for initial INTM clear not
documented in public corpus). The diagnostic instrument
`CALYPSO_FORCE_INTM_CLEAR_AT=N` is intentionally retained — see
[`SESSION_20260429.md`](hw/arm/calypso/doc/SESSION_20260429.md) and
[`datasheets/README.md`](hw/arm/calypso/doc/datasheets/README.md) for the full
reasoning, evidence corpus (5 TI PDFs + 3 ROM dumps + Calypso overview), and
suggested investigation paths.

Specs traceable against binutils:
- [`opcodes/0x68_0x6F.md`](hw/arm/calypso/doc/opcodes/0x68_0x6F.md) — verified v2 with `insn_template` struct
- [`opcodes/0xF3.md`](hw/arm/calypso/doc/opcodes/0xF3.md) — verified v1 with bit-by-bit decoding

PHY-side test tool: [`scripts/inject_fcch.py`](scripts/inject_fcch.py) — synthesizes
FCCH bursts in 3 modes (bytes_zero / soft_neg127 / iq_raw) for independent
DSP correlator testing.

---


This is a packaged snapshot of the Calypso-specific files extracted from a full QEMU
source tree. The active development tree lives separately (see [Repository layout](#repository-layout)).

## What's in this snapshot

| File | Purpose |
|---|---|
| [`hw/arm/calypso/calypso_c54x.c`](hw/arm/calypso/calypso_c54x.c) | TMS320C54x DSP emulator (~4500 lines, opcode dispatch + memory model) |
| [`hw/arm/calypso/calypso_c54x.h`](hw/arm/calypso/calypso_c54x.h) | DSP state struct, memory map constants |
| [`hw/arm/calypso/calypso_trx.c`](hw/arm/calypso/calypso_trx.c) | TRX / TPU / TSP / ULPD / SIM + TDMA tick + DMA orchestration |
| [`hw/arm/calypso/calypso_bsp.c`](hw/arm/calypso/calypso_bsp.c) | BSP DMA + PORTR buffer + UDP listener (port 6702) |
| [`hw/arm/calypso/calypso_iota.c`](hw/arm/calypso/calypso_iota.c) | IOTA chip BDLENA gating |
| [`hw/arm/calypso/calypso_tint0.c`](hw/arm/calypso/calypso_tint0.c) | TINT0 timer (master clock) |
| [`hw/arm/calypso/calypso_sim.c`](hw/arm/calypso/calypso_sim.c) | ISO 7816 SIM card emulation (IMSI/Ki) |
| [`hw/arm/calypso/calypso_fbsb.c`](hw/arm/calypso/calypso_fbsb.c) | FB/SB detection helper |
| [`hw/arm/calypso/calypso_tdma_hw.c`](hw/arm/calypso/calypso_tdma_hw.c) | TDMA hardware model |
| [`hw/arm/calypso/calypso_mb.c`](hw/arm/calypso/calypso_mb.c) | Mailbox / shared register region |
| [`hw/arm/calypso/calypso_soc.c`](hw/arm/calypso/calypso_soc.c) | SoC composition (wires up all peripherals) |
| [`hw/arm/calypso/calypso_dbg.c`](hw/arm/calypso/calypso_dbg.c) | Debug helpers / instrumentation |
| [`hw/arm/calypso/sercomm_gate.c`](hw/arm/calypso/sercomm_gate.c) | Sercomm DLCI router (PTY ↔ FIFO) |
| [`hw/arm/calypso/l1ctl_sock.c`](hw/arm/calypso/l1ctl_sock.c) | L1CTL Unix socket (`/tmp/osmocom_l2`) |
| [`hw/arm/calypso/fw_console.c`](hw/arm/calypso/fw_console.c) | Firmware console capture (decodes "LOST N!" etc) |
| [`hw/intc/calypso_inth.c`](hw/intc/calypso_inth.c) | INTH interrupt controller |
| [`hw/char/calypso_uart.c`](hw/char/calypso_uart.c) | UART with RX FIFO + sercomm + romload stub |
| [`hw/timer/calypso_timer.c`](hw/timer/calypso_timer.c) | Generic Calypso timer block |
| [`hw/ssi/calypso_spi.c`](hw/ssi/calypso_spi.c) | SPI controller |
| [`hw/ssi/calypso_i2c.c`](hw/ssi/calypso_i2c.c) | I2C controller |
| [`bridge.py`](bridge.py) | Python relay BTS UDP (5700-5702) ↔ BSP (6702), QEMU clock-slave |
| [`run.sh`](run.sh) | Orchestrated launch script (QEMU → bridge → BTS → mobile) |
| [`dsp_read.sh`](dsp_read.sh) | DSP ROM dump reader helper |
| [`scripts/inject_fb.py`](scripts/inject_fb.py) | Diagnostic FB burst injector |
| [`hack.patch`](hack.patch) | Diagnostic INTM force-clear patch (env-var driven, **temporary**) |
| [`calypso_dsp.txt`](calypso_dsp.txt) | DSP ROM dump (PROM0/1/2/3 + DROM/PDROM, 132K words) |
| [`configs/devices/`](configs/devices/) | QEMU device tree configs |

## Documentation

| File | Content |
|---|---|
| [`calypso.md`](calypso.md) | Architecture pipeline complet (mermaid diagrams) — pipeline, séquence boot, memory map, état des composants |
| [`CLAUDE.md`](CLAUDE.md) | Top-level project context (architecture, conventions, current bug, build) |
| [`hw/arm/calypso/CLAUDE.md`](hw/arm/calypso/CLAUDE.md) | Calypso-HW-specific context (opcode debug workflow, ROM reader, firmware symbols) |

The active development tree (separate repo) carries deeper documentation:
`qemu-src/hw/arm/calypso/doc/` contains TODO.md (current investigation state),
BSP_DMA.md, C54X_INSTRUCTIONS.md, CALYPSO_HW.md, DSP_ROM_MAP.md,
SERCOMM_GATE_ARCHITECTURE.md, and `spru172c.pdf` (TI C54x reference manual).

## Architecture

Dual-core GSM baseband emulator:

- **ARM7TDMI** runs `osmocom-bb` Layer 1 firmware (`layer1.highram.elf`)
- **TMS320C54x DSP** runs the real Calypso DSP ROM (`calypso_dsp.txt`)
- **API RAM** shared memory at DSP `0x0800` / ARM `0xFFD00000`, 8K words
- **BSP** receives I/Q bursts via UDP 6702, serves DSP via PORTR PA=0x0034
- **TPU → TSP → IOTA** chain gates BDLENA for RX windows
- **Bridge** (Python) relays BTS UDP 5700-5702 ↔ BSP, clock-slave of QEMU
- **osmo-bts-trx** acts as the cell, paired with `osmo-stp/hlr/msc/bsc/...` for full network
- **mobile** (osmocom L23) talks L1CTL via `/tmp/osmocom_l2` socket through `osmocon`

Full pipeline + sequence diagrams in [`calypso.md`](calypso.md).

## Quick start

The repository is meant to be unpacked into a Docker container with all GSM toolchain
prerequisites (osmocom-bb, osmo-bts-trx, osmo-msc/bsc/hlr/mgw, osmocon, mobile).
See `run.sh` for the orchestrated launch:

```bash
./run.sh
```

The script kills any stale state, brings up the osmocom core network, launches QEMU
with the firmware ELF, starts `osmocon` to bridge mobile↔UART, starts the Python bridge
between QEMU BSP and `osmo-bts-trx`, and attaches a tmux session.

## Status (April 2026)

The emulator boots both ARM and DSP, exchanges mailbox handshake, the DSP enters user
code, and reaches FB/SB detection. The full firmware path executes including:

- Bootloader handshake (`BL_CMD=2`, BACC to user code at `0x7000`)
- Init sequence with RSBX/SSBX cascades, IMR setup, MVPD copy
- BSP DMA delivery from `osmo-bts-trx` via the bridge
- L1S frame scheduler running

**Current issue**: a stochastic timing fragility (Cas B) makes Location Update
reproducibility approximately `p ≈ 0.75` per run on identical config + identical binary.
Diagnostic complete: site of fragility identified at `PC=0x771c` (CC pmad=0x777a cond=0x45
in PROM0), branching on accumulator computed from a read of DARAM[0x3FB4/5] (BSP DMA zone).

Causal chain: host scheduler → QEMU TPU tick → CLK IND → BTS osmo-bts-trx → bridge UDP →
BSP DMA → DARAM contents → DSP read → branch.

The fix direction is to align the external chain (TPU tick origin, BTS replacement or
buffering, bridge in-process) on the DSP's ICOUNT virtual time, instead of letting it run
on host wall-clock. Scope is sub-specified pending answers to three architectural questions
documented in `qemu-src/hw/arm/calypso/doc/TODO.md` ("Fix scope architecturale" section).

The `hack.patch` file enables a diagnostic env-var-driven INTM force-clear that arms the
breakthrough cascade. To use:

```bash
patch -p1 < hack.patch                       # apply
CALYPSO_FORCE_INTM_CLEAR_AT=2000000 ./run.sh # arm at insn=2M
patch -p1 -R < hack.patch                    # revert when done
```

The hack arms but does not deterministically trigger breakthrough — see TODO.md for the
full diagnostic.

## Repository layout

```
qemu-calypso/             ← this snapshot, packaged for distribution / licensing
├── hw/arm/calypso/       ← Calypso SoC + DSP emulator (Calypso-specific)
├── hw/{intc,char,timer,ssi}/  ← Generic peripherals adapted for Calypso
├── include/              ← Header overrides
├── bridge.py             ← Python BTS↔BSP relay
├── run.sh                ← Launch orchestration
├── calypso_dsp.txt       ← DSP ROM dump (132K words)
├── calypso.md            ← Architecture pipeline + diagrams
├── CLAUDE.md             ← AI-assistant context (top-level)
└── hack.patch            ← Diagnostic INTM clear (temporary)
```

The active development happens in a separate full QEMU tree at `qemu-src/`. Apply changes
there, then copy back to this snapshot for distribution. Three known mirrors exist locally
(`qemu-src/`, `qemu/`, `qemu-calypso/`) that drift apart between sync operations — see
the TODO.md "Doublons qemu-src/qemu/qemu-calypso" entry for the cleanup plan.

## Build

The QEMU build uses the standard meson/ninja flow on a full QEMU source tree. This snapshot
contains only the Calypso-specific source — to build, drop these files into a recent QEMU
checkout and:

```bash
cd qemu-build
ninja qemu-system-arm
```

The container `bastienbaranoff/free-bb:latest` has a pre-built environment with all GSM
toolchain dependencies and the QEMU build at `/opt/GSM/qemu-src/build/`.

## License & attribution

- **QEMU base**: GPL-2.0-or-later (upstream QEMU)
- **Calypso emulator additions**: GPL-2.0-or-later
- **osmocom-bb firmware**: GPL (used unmodified, not redistributed here)
- **Calypso DSP ROM** (`calypso_dsp.txt`): TI proprietary, dumped from physical device for
  research and interoperability purposes (osmocom DSP dumper). Not redistributable
  commercially without TI authorization.

## Related projects

- [osmocom-bb](https://osmocom.org/projects/baseband/) — open-source GSM baseband stack (ARM Layer 1 + L23)
- [osmo-bts](https://osmocom.org/projects/osmobts/) — open-source GSM BTS
- [osmo-cn](https://osmocom.org/projects/cellular-infrastructure/) — full open-source cellular core network
- [QEMU](https://www.qemu.org/) — generic emulation framework
- [tic54x binutils](https://sourceware.org/binutils/) — TMS320C54x assembler/disassembler reference
