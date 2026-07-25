# qemu-calypso

**QEMU emulation of the TI Calypso GSM baseband chipset** — the dual-core SoC
(ARM7TDMI/ARM946 + TMS320C54x DSP) used in the OpenMoko Neo, Motorola C1xx /
Compal e88 family, and arguably the most reverse-engineered cellular modem in
open-source history.

Runs the **real TI Calypso DSP mask-ROM** (not a stub) and the unmodified
[osmocom-bb](https://osmocom.org/projects/baseband/) firmware on the ARM side. A
relay connects the BSP serial port to a simulated air interface (grgsm / a real
`osmo-bts-trx`), so the emulator camps on a fully software-defined GSM cell with
osmo-msc/hlr/bsc/stp behind it.

No firmware patching. No `#ifdef QEMU`. The same binaries that run on a physical
Motorola C123 run here — and when something doesn't work, that's where the bug
lives, not in a convenient stub.

> *"You can't actually emulate a GSM phone."*
> — about half the people who looked at this when it started.

---

## What it does, concretely

```
   ┌────────────────────┐         ┌──────────────────────┐
   │  osmocom-bb mobile │ ←L1CTL→ │  layer1 firmware     │   ← ARM, real binary
   │       (L23)        │         │  (ARM + osmo L1)     │
   └────────────────────┘         └──────────┬───────────┘
                                             │ API RAM (dual-port DARAM)
                                             ▼
                                  ┌──────────────────────┐
                                  │ Calypso DSP (C54x)   │   ← runs the real TI ROM
                                  │ + TPU/TSP/IOTA/BSP   │   ← modelled peripherals
                                  └──────────┬───────────┘
                                             │ IPC relay (I/Q)
                                             ▼
                                  ┌──────────────────────┐
                                  │  grgsm / osmo-bts-trx │   ← the air interface
                                  │  + osmo-msc/hlr/bsc   │   ← real core network
                                  └──────────────────────┘
```

Full GSM stack, end to end, in software, with the DSP doing (or being made to do)
actual FCCH/SCH correlation on actual I/Q coming from an actual GSM core network.

---

## Two operating modes (read this first)

The single most important thing to understand about the current tree is that
there are **two distinct paths** through L1, selected by `CALYPSO_DSP_SHUNT`:

| Mode | `CALYPSO_DSP_SHUNT` | How the phone camps | Status |
|------|--------------------|--------------------|--------|
| **Plumbing / shunt** (`full-grgsm`) | `1` | A host-side shunt decodes the continuous I/Q stream (grgsm), forces `d_fb_det`, and injects SI directly. The C54x is largely bypassed. | ✅ **camps** — SI1-4, RACH, LU |
| **Native DSP** | `0` (goal) | The real C54x mask-ROM runs the FB/FCCH correlator on I/Q delivered through the native TPU→TSP→IOTA→BSP→DARAM chain, produces a real `d_fb_det`, and the ARM L1 syncs on it. | 🚧 **work in progress** |

The shunt is what makes the demo work today. The active engineering frontier is
retiring it, one native wire at a time, until the DSP itself acquires the cell.
Most of the env-gated code in this tree exists to bridge that gap incrementally
and reversibly — **every hack is gated, defaults are documented, and nothing is
poked that can be wired.**

---

## Where the project is right now (2026-07-25)

| Layer | Status |
|-------|--------|
| QEMU SoC + ARM + C54x DSP cores | ✅ stable |
| DSP boot + DARAM overlay + opcode decode | ✅ stable (POPM/MVDK/F3xx audits landed) |
| Shunt path (`full-grgsm`) — phone camps on the cell | ✅ SI1-4 + RACH + Location Updating |
| DSP go-live: IMR armed, INTM native toggle, frame-IT (vec28) | ✅ IT taken, `IMR=0x52fd`, shadow `0x435b` peuplé |
| **RANK1** — ARM→DSP `d_ctrl_system 0x0810` bit15 gate (`0xa53c`) | ✅ **wired (CTRLSYS)**, gate falls through to bootstrap |
| FB correlator reached (native `CALA` dispatch at `0xb01e→0x8d00`) | ✅ runs every frame |
| **RANK2** — native RX window fills I/Q buffer `0x2a00` + `d[0x3f92]` task word | 🚧 **in progress** (TPU_RX_WIRE) |
| Correlator reads real I/Q (buffer full, `AR5→0x2a00`) | ❌ buffer starved (shunt gates the RX chain) |
| `d_fb_det` from the **native** correlator | ❌ pending RANK2 |
| RANK3 dispatch / RANK4 FN recale / RANK5 RF frontend | ⏭ pending |

See [`hw/arm/calypso/doc/`](hw/arm/calypso/doc/) for the dated go-live reports and
grafcets, and `MEMORY.md`/`CLAUDE.md` for the running investigation state.

---

## The go-live effort (RANK1 → RANK5)

Getting the native DSP to acquire the cell decomposes into a ranked wiring list.
Each item is an **inter-block wire that the shunt currently short-circuits**, not
a missing algorithm — the mask-ROM already contains the correlator.

**RANK1 — ARM→DSP control bridge `d_ctrl_system 0x0810` ✅ done.**
The go-live init at `0xa4c7..0xa582` (`ORM #0x3000,IMR` / `RSBX INTM` / `STL A,IMR`)
reaches a gate at `0xa53c`:

```
a53c: 61e1 0010 8000   BITF *AR1(0x10), #0x8000   ; AR1=0x0800 → tests data[0x0810] bit15
a53f: f820 a575        BC   0xa575, NTC           ; bit15 CLEAR → short-circuit (stuck)
                                                   ; bit15 SET   → bootstrap → FB dispatch
```

`data[0x0810]` (osmocom `d_ctrl_system`, the RESET/RESUME control word) is written
by the ARM in `l1s_reset()`, but the emulated ARM→DSP API bridge never propagated
it. It is now modelled ARM-side in `calypso_arm2dsp.c` (the **CTRLSYS** wire, same
family as the BGEN `0x098a/0x098c` bridge), asserting `data[0x0810] |= 0x8000` at
the gate. Cross-verified from PROM0.bin: the minimal correct mask is **exactly
`0x8000`** (bit15) — the earlier `0x0002`/B_TASK_ABORT poke had *zero* effect
because the `BITF` only tests bit15.

**RANK2 — native RX window → I/Q buffer + scheduler task word 🚧.**
The FB correlator is dispatched **natively** (`CALA A` at `0xb01e`, target read from
dispatch cell `data[0x43d8]`) and runs every frame — but on a starved input.
Reverse-engineering established:

- the I/Q input pointer is **`AR5`** (hard-coded `STM #0x2a00,AR5` in the ROM),
  *not* `AR3` (which is the CMPS peak pointer; its garbage value `0x4bd0` is decode
  corruption, not a missing wire);
- the input buffer is **`0x2a00..0x2b27`, 296 int16** (interleaved I/Q);
- **the DSP shunt gates the entire native RX chain**: the ARM→DSP task DMA
  (`calypso_trx.c`, `!shunt_active`) and the BSP sample delivery
  (`calypso_bsp_deliver_buffered`, early-returns under the shunt). With the shunt
  on, `task_md=5` (FB) never reaches DSP DARAM and `0x2a00` never fills;
- the TPU RX-window's BDLENA pulse (TSP CTRL2 → IOTA) had a consumer
  (`calypso_iota_take_bdl_pulse`) with **zero callers**.

The **TPU_RX_WIRE** work-in-progress lifts those shunt gates (hybrid, reversible),
wires the BDLENA consumer to deliver the nearest buffered burst into `0x2a00`, and
sets the DSP scheduler task word `data[0x3f92] |= 0x0800` when the ARM commands the
FB task (the native setter `ORM` at `0xa539` is skipped because `data[0x5a00]==0x88`).

**RANK3–5** — native dispatch LUT install at `0x8341` (currently unreachable via the
`0x013b` overlay derail), DSP/TRX frame-number recale on the BTS SCH, and the
TSP→RF frontend (PLL/band/gain) — pending RANK2.

---

## Quick start

```bash
# Shunt path — the phone camps (full-grgsm). This is the working demo.
CALYPSO_MODE=full-grgsm ./run.sh      # via the container orchestration

# Native-DSP wiring under test (go-live + RX chain), env in calypso_wire.env:
#   CALYPSO_ARM2DSP_CTRLSYS=1   RANK1 gate 0x0810
#   CALYPSO_TPU_RX_WIRE=1       RANK2 RX chain + d[0x3f92]
#   CALYPSO_KEEP_IMR=1  CALYPSO_FRAME_IT_NATIVE=1  CALYPSO_TINT0_MASTER=1
```

The runtime env lives in **`calypso_wire.env`** (sourced by `calypso.env`), using
the `: "${VAR:=default}"` idiom so a value on the CLI always wins. Gates read via
`getenv()!=NULL` must be **`unset`** to disable — an empty string is truthy.

Container `bastienbaranoff/free-bb:latest` (a.k.a. `osmo-operator-1`) ships the
whole GSM toolchain pre-built plus the QEMU build tree at `/opt/GSM/qemu-src/build/`.

---

## Environment variables (current)

Full-fidelity list is in `calypso_wire.env`. The load-bearing ones:

| Variable | Default | Effect |
|----------|---------|--------|
| `CALYPSO_MODE` | `full-grgsm` | Air-interface / shunt profile. |
| `CALYPSO_DSP_SHUNT` | `1` | The host-side L1 shunt (plumbing camp). `0` = full native (goal). |
| `CALYPSO_ARM2DSP_CTRLSYS` | `1` | **RANK1**: ARM writes `d_ctrl_system 0x0810` bit15 → go-live gate `0xa53c` falls through. |
| `CALYPSO_ARM2DSP_BGEN` | `1` | ARM posts `d_background_enable/state` (`0x098a/0x098c`) → DSP leaves its wait-loop. |
| `CALYPSO_TPU_RX_WIRE` | `1` | **RANK2** (WIP): lift shunt gates on the native RX chain, consume BDLENA → fill `0x2a00`, set `d[0x3f92]`. |
| `CALYPSO_KEEP_IMR` | `1` | Re-arm `IMR=0x52fd` (bit5/BRINT0) when the ROM clears it (`0xb37e`/`0xa509`). |
| `CALYPSO_FRAME_IT_NATIVE` | `1` | Frame interrupt on the native vector 28 (bit12), not the vec19 stub. |
| `CALYPSO_TINT0_MASTER` | `1` | TINT0 (vec20) pending → native INTM releases. |
| `CALYPSO_FORCE_INTM_ONESHOT` | `1` | Clear INTM once when BRINT0 pending → bootstraps `0xa51b RSBX`. |
| `CALYPSO_BSP_DARAM_ADDR` | `0x2a00` | DARAM target of the BSP I/Q DMA (the correlator input buffer). |
| `CALYPSO_ICOUNT` | `auto` | QEMU `-icount` mode. |
| `CALYPSO_SIM_CFG` | `~/.osmocom/bb/…cfg` | SIM IMSI/Ki config. |

Diagnostic-only (read-only probes): `CALYPSO_INTM_TRANS`, `CALYPSO_CORRELATOR_TRACE`,
`CALYPSO_BSP_FN_PROBE`, `CALYPSO_D247_TRACE_OFF`, `CALYPSO_SM_TRACE`. Retired pokes are
kept `unset` in `calypso_wire.env` with the reason (e.g. `FORCE_0810` set the wrong
bit; `ISR_TO_8341` stormed by skipping the overlay context save).

---

## Architecture

### Memory map (DSP side, C54x)

| Range | Type | Content |
|-------|------|---------|
| `0x0000-0x00FF` | Boot stubs | reset vector + low scratch |
| `0x0100-0x01FF` | DARAM overlay | ISR/dispatch code copied from ROM at boot (`OVLD`) |
| `0x0800-0x0FFF` | API RAM (shared) | NDB @`0x08D4`, `d_task_md@0x0804`, `d_fb_det@0x08F8`, `a_sync_demod@0x08FA`, write-page mirror @`0x0586` |
| `0x2a00-0x2b27` | I/Q input buffer | 296 int16, interleaved — read by the correlator via `AR5` |
| `0x2bc0+` | Coefficient window | FCCH-tone coeffs (`STM #0x2bc0,AR3`) |
| `0x3f00-0x3fff` | Scheduler soft-state | task word `0x3f92`, phase `0x5a00`, soft-vectors (`0x3fcd`…) |
| `0x7000-0xDFFF` | PROM0 (base **0x7000**) | DSP mask-ROM. FB dispatch `0x8d00`, MAC kernel `~0xa076`/`0x9a80`, go-live init `0xa4c7-0xa582`, native `CALA` dispatch `0xb01e` |
| `0xFF80-0xFFFF` | Interrupt vectors | from PROM1, `IPTR=0x1FF` |

> ⚠️ Any static read of `PROM0.bin` must use **base 0x7000** (byte offset
> `(pc-0x7000)*2`) and be validated against runtime logs — earlier mis-based
> disassembly (0x8000) produced phantom findings.

### Interrupt vectors

`vec = imr_bit + 16`, `addr = 0xFF80 + vec*4`.

| IRQ | Vec | IMR bit | Notes |
|-----|-----|---------|-------|
| INT3 (legacy frame) | 19 | 3 | stub path |
| Frame-IT (native) | 28 | 12 | `CALYPSO_FRAME_IT_NATIVE=1` |
| TINT0 | 20 | 4 | cadence master |
| BRINT0 (BSP) | 21 | 5 | RX-buffer-received → wakes FB-det ISR |

### Repository layout

```
qemu-calypso/                          ← the OVERLAY (git repo, source of truth for
│                                          the Calypso additions on top of vanilla QEMU)
├── hw/arm/calypso/
│   ├── calypso_c54x.c                 ← C54x DSP core (~15k lines) + go-live wiring
│   ├── calypso_arm2dsp.c              ← ARM→DSP API bridge (BGEN + CTRLSYS wires)
│   ├── calypso_trx.c                  ← TRX/TDMA + ARM→DSP task DMA (shunt-gated)
│   ├── calypso_tpu.c                  ← TPU sequencer (v3: real AT/WAIT + MOVE/TSP)
│   ├── calypso_tsp.c                  ← TSP serial protocol (CTRL1/2, BDLENA byte)
│   ├── calypso_iota.c                 ← IOTA/TWL3025 BDLENA window queue
│   ├── calypso_bsp.c                  ← BSP RX burst → DARAM 0x2a00 + BRINT0 + TPU_RX_WIRE
│   ├── calypso_dsp_shunt.c            ← the full-grgsm L1 shunt (plumbing camp)
│   ├── calypso_sim.c / l1ctl_sock.c / sercomm_gate.c
│   └── doc/                           ← dated go-live reports, grafcets, opcode maps,
│                                         FB_CORRELATOR_PIPELINE.md, spru172c.pdf
├── tests/                             ← pytest milestone harness + go-live grafcet test
├── calypso_wire.env                  ← runtime env (go-live/wire defaults)
├── make-overlay.sh                    ← back-port qemu-src (working tree) → this overlay
├── calypso_dsp.*                      ← DSP ROM dump (PROM0-3 / DROM / registers)
└── CLAUDE.md / MEMORY.md              ← assistant context + running investigation
```

---

## The overlay mechanism

This repo is an **overlay**: `vanilla QEMU + qemu-calypso = qemu-src` (the assembled
working tree). Two scripts keep them in sync:

- **`make-fork.sh`** : overlay + genuine QEMU → working tree (`qemu-src`).
- **`make-overlay.sh`** : working tree → overlay (back-port; `qemu-src` is the
  source of truth). Run it from `qemu-src`. It syncs every file the overlay
  *tracks* (`git ls-files`); **new** Calypso files must be `git add`-ed in the
  overlay once, then it keeps them in sync. Nothing is ever deleted.

Edit in `qemu-src`, build, then `make-overlay.sh` to persist into this repo.

---

## Runtime probes

The binary embeds **runtime-activable probes** on QEMU stderr, throttled to avoid
spam. This is the difference between "the project is stuck" and "the project tells
you *why* it is stuck". Current high-value tags:

| Tag | Target | What it tells you |
|-----|--------|-------------------|
| `arm2dsp CTRLSYS` | `data[0x0810]` assert | RANK1 gate wire fired |
| `CYCLE-TRACE a53c/a53f` | go-live gate | `BITF`/`BC` TC — does the gate pass? |
| `WATCH-0810-WR` | writes to `0x0810` | when/whether the control bit is set |
| `CALA-WIDE … DANS-CORRELATEUR` | computed `CALA` into `[0x8d00,0x9000)` | native FB dispatch, target + `task_md` |
| `COEFFS-WR` | correlator inner loop | AR3/AR4/AR5 + values (is it reading real I/Q?) |
| `TPU-RX-WIRE` | BDLENA consume | RANK2 wire: `d[0x3f92]` set + burst delivered |
| `FN-PROBE` | BSP burst FN vs dispatcher FN | constant delta = offset; drifting = clock bug |
| `INTM-TRANS` | INTM 0↔1 | cause (SSBX/RSBX/STM ST1) — congestion diagnosis |
| `BSP … DMA fn=` | DARAM write | I/Q actually landing at `0x2a00` |

---

## Methodology

Principles that have repeatedly paid off:

**Wire, don't poke.** A poke falsifies a function's return; a wire models the real
inter-block signal. The audit rule is *wire-only unless implementation is
unavoidable*. Every gated hack in `calypso_wire.env` names the real wire it stands
in for, and the ones that turned out to be dead ends are kept `unset` with the
reason.

**Base your disassembly, then trust runtime over comments.** `PROM0.bin` base is
`0x7000`. Code comments in this tree are a running lab notebook — some are wrong on
purpose (superseded hypotheses). Read them, don't trust "block" claims; verify
against the log.

**Decompose the wall into ranked wires.** "The correlator reads garbage" became
five concrete RANK items once the dispatch (native `0xb01e`), the pointer (`AR5`),
the buffer (`0x2a00`) and the gate (`0x0810`) were each pinned to a cited address.

**Deterministic shunt for downstream debugging.** The `full-grgsm` shunt is a
working camp you can lean on while wiring the native path underneath — one variable
changes per run.

**Test after every edit.** Build in Docker (`ninja qemu-system-arm`), the user
relaunches the pile, then read the log for the specific probe that the edit targets.

---

## Session history (recent)

### 2026-07-25 — go-live gate + native RX wiring
- **RANK1 done**: gate `0xa53c` = `BITF data[0x0810],#0x8000`. Root-caused from
  PROM0.bin + cross-validated with the emulator's own decoder; wired ARM-side
  (`CTRLSYS`, `calypso_arm2dsp.c`). Corrects the old "0x0810 = abort" belief — the
  gate tests **bit15 only**. Gate now passes (`TC=1`, no short-circuit to `0xa575`).
- **RANK2 in progress**: established that the DSP shunt gates the whole native RX
  chain (`trx.c` task DMA + `bsp.c` delivery), that `AR5` (not `AR3`) is the I/Q
  pointer, and that the BDLENA consumer had zero callers. Added the reversible
  `TPU_RX_WIRE` hybrid: lift the shunt gates, consume BDLENA → fill `0x2a00`, set
  `d[0x3f92]` on the ARM FB-task DMA. Awaiting run validation.

### Earlier (2026-05 → 07)
- **DSP↔ARM API-RAM mirror** fix (read path read `dsp_ram[]` not `dsp->data[]`) —
  unblocked real-path FBSB for the first time.
- **Opcode audits**: POPM (`0x8A00` was decoded as MVDK → INTM stuck), the `F3xx`
  family, the `0x68-0x6E` ANDM/ORM/XORM/ADDM/BANZ handlers, APTS/PMST reset
  alignment. See `hw/arm/calypso/doc/SESSION_*.md` and the opcode maps.
- **Hack purges**: BCCH inject, FBDET-SKIP, INTM force-clear, SI3 hardcode — each
  removed once it was shown to be hiding the real bug.

---

## Build

```bash
docker exec CONTAINER bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"
```

Then back-port into this overlay:

```bash
docker exec CONTAINER bash -c "cd /opt/GSM/qemu-src && ./make-overlay.sh"
```

---

## Conventions

- **Wire, don't poke.** No `#ifdef QEMU`. No shortcut that hides the real bug.
  Every hack is env-gated, off-by-default unless it is a validated wire, and
  documented in `calypso_wire.env`.
- **Verify opcodes** against `tic54x-opc.c` (binutils) before patching. See
  [`hw/arm/calypso/doc/opcodes/`](hw/arm/calypso/doc/opcodes/).
- **Disassemble from base `0x7000`**; validate static reads against runtime logs.
- **Reversible over clever.** A gated, back-out-able wire beats an irreversible
  edit to the DSP core.

---

## License & attribution

- **QEMU base** — GPL-2.0-or-later (upstream QEMU)
- **Calypso emulator additions** — GPL-2.0-or-later
- **osmocom-bb firmware** — GPL (used as-is, not redistributed here)
- **Calypso DSP ROM** (`calypso_dsp.*`) — TI proprietary. Physical-device dump for
  research and interoperability (osmocom DSP dumper). Not commercially
  redistributable without TI authorization.

---

## Related projects

- [osmocom-bb](https://osmocom.org/projects/baseband/) — the GSM baseband stack run unmodified
- [gr-gsm](https://github.com/ptrkrysik/gr-gsm) — the GNU Radio GSM receiver used on the shunt air interface
- [osmo-bts](https://osmocom.org/projects/osmobts/) / [osmo-cn](https://osmocom.org/projects/cellular-infrastructure/) — BTS + core network
- [QEMU](https://www.qemu.org/) — the emulation framework this builds on
- [tic54x binutils](https://sourceware.org/binutils/) — TMS320C54x opcode reference

---

> *If a piece of hardware exists, it can be emulated.*
> *If a chip's ROM is dumpable, its firmware can run anywhere.*
>
> *The phone camps on the plumbing today. Tomorrow the DSP does it itself.*
