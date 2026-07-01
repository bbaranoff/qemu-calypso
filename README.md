# qemu-calypso

**A silicon-faithful software emulation of the Texas Instruments Calypso GSM baseband — the ARM7 + TMS320C54x DSP chipset behind OsmocomBB phones — as a QEMU `arm-softmmu` machine.**

`qemu-calypso` boots a real OsmocomBB Layer 1 firmware image against an emulated Calypso SoC and a **real, dumped Calypso DSP ROM**, with no firmware modification and no internal-state poking. The goal is a virtual Calypso you can attach to a GSM network stack (osmo-bts-trx / osmo-bsc) for development, CI, and interoperability testing — without physical Motorola C1xx hardware.

> **Why this is hard.** Texas Instruments destroyed the Calypso/TMS320C54x DSP documentation in 2009. Much of what this emulator does — the ARM↔DSP mailbox protocol, the DSP go-live handshake, the C54x opcode behaviour — was reconstructed from instrumented silicon and observed behaviour, not from a datasheet. This repository is, in large part, that reconstruction turned into code.

---

## Status

| Area | State |
|---|---|
| `calypso` machine boots, ARM7 runs OsmocomBB L1 firmware | ✅ working |
| Calypso DSP ROM loaded from real dump (`calypso_dsp.txt`) | ✅ working |
| UART / sercomm, L1CTL unix socket, TWL3025 (ABB), SIM | ✅ working |
| ARM↔DSP shared API-RAM mailbox (bidirectional) | ✅ working |
| DSP bootloader protocol (ARM uploads code, jumps to `0x7000`) | ✅ working |
| Network bridges (sercomm↔BTS UDP, TRX orchestration) | ✅ working |
| **Real DSP FB/FCCH detection (`d_fb_det != 0`)** | 🔬 active frontier — see [The DSP revival frontier](#the-dsp-revival-frontier) |

This is an active research/engineering project, not a finished product. The end-to-end control path is alive; the current work is bringing the **real** emulated C54x DSP fully online so it detects the frequency/synchronisation bursts itself, rather than being shunted.

---

## Background: what is Calypso, and why emulate it?

The **TI Calypso** (bundled with the TWL3025 "Iota" analog baseband and TRF6151 RF frontend) is the chipset inside a family of early-2000s GSM phones — most famously the **Motorola C1xx** series — that became the reference hardware for the [OsmocomBB](https://osmocom.org/projects/baseband) free-software GSM baseband. Calypso pairs:

- an **ARM7TDMI** running the protocol-stack firmware (Layer 1 and up), and
- a **TMS320C54x DSP** running a fixed ROM that does the hard-real-time PHY work (correlation, (de)modulation, channel coding), talking to the ARM through a shared **API RAM** mailbox.

Emulating Calypso faithfully means emulating *both* processors and the mailbox between them — including the DSP ROM's expectations about interrupts, TPU/TDMA timing, and shared-memory cells. That DSP side is what makes this more than a typical QEMU board port.

---

## Architecture

```
            osmo-bsc
               │
          osmo-bts-trx
               │
      python bridges (user-space)
   sercomm↔UDP · TRX orchestration · clock
               │
 ┌─────────────────────────────────────────────┐
 │   qemu-system-arm  -M calypso                │
 │                                              │
 │   ARM7 (OsmocomBB L1 firmware .elf)          │
 │        │                                     │
 │   API RAM  ◄── shared mailbox ──►            │
 │        │                                     │
 │   TMS320C54x DSP (real ROM dump)             │
 │                                              │
 │   TPU/TDMA · BSP · UART/sercomm · TWL3025    │
 └─────────────────────────────────────────────┘
               │
        L1CTL unix socket  (/tmp/osmocom_l2_1)
               │
        mobile / ccch_scan  (osmocom-bb layer23)
```

`calypso_trx.c` acts as the central orchestrator (API mailbox + hardware hooks); `calypso_c54x.c` is the DSP core; `calypso_bsp.c` handles the RF/burst boundary. The ARM is master, the DSP is slave, coordinated through the API-RAM mailbox.

**Key source files** (`hw/arm/calypso/`):

| File | Lines | Role |
|---|--:|---|
| `calypso_c54x.c` | ~13.7k | TMS320C54x DSP core: instruction decode/execute, MMRs, interrupts |
| `calypso_trx.c` | ~2.5k | Central orchestrator: API mailbox, TDMA/TPU hooks, burst routing |
| `calypso_dsp_shunt.c` | ~1.8k | DSP shunt path (host-model PHY, used while the real DSP is brought online) |
| `calypso_bsp.c` | ~1.6k | Baseband signal processing / burst boundary, UDP TX/RX |
| `calypso_sim.c` | ~0.8k | SIM card emulation |
| `l1ctl_sock.c` | ~0.5k | L1CTL unix-socket transport to `mobile`/`ccch_scan` |
| `calypso_soc.c`, `calypso_mb.c` | ~0.9k | SoC wiring and the `calypso` machine definition |

Plus `twl3025` (analog baseband), `iota`, `fbsb`, `layer1`, `tint0` (frame tick), `sercomm_gate`, and more.

---

## Repository model: an overlay, not a fork

`qemu-calypso` is an **overlay** containing only the delta from upstream QEMU. Applied on a genuine QEMU tree it produces the working fork:

```
QEMU (genuine 9.2.x)  +  qemu-calypso (this overlay)  =  qemu (the fork)
```

### Build

```sh
# 1. Materialise the fork: copy genuine QEMU, apply this overlay on top.
./make-fork.sh [GENUINE_DIR] [OUT_DIR]
#    defaults: GENUINE_DIR=/opt/GSM/QEMU (v9.2.4)   OUT_DIR=/opt/GSM/qemu-fork

# 2. Configure & build just the ARM system emulator.
cd <OUT_DIR> && mkdir -p build && cd build
../configure --target-list=arm-softmmu
ninja qemu-system-arm
```

**Dependency:** the Calypso machine links against **`libosmocoding`** (from [libosmocore](https://osmocom.org/projects/libosmocore)) for RACH/normal-burst uplink encoding — `configure` fails fast if the dev package is missing.

### Run

```sh
./bash_scripts/start-clean.sh     # sources calypso.env, then execs run.sh
```

Confirmed defaults (see `run.sh`): `CALYPSO_DSP=c54x`, `CALYPSO_DSP_REG_MODE=c54x`, `CALYPSO_DSP_SHUNT=0`. Runtime log at `/root/qemu.log`. You'll also want the network side (`osmo-bts-trx`, bridges) up; see `doc/project/` for the full pipeline.

---

## The DSP revival frontier

The single open blocker is documented in detail in `doc/doc_master.md`, but in short:

The goal is to have the **real** emulated C54x DSP detect FB/FCCH and write `d_fb_det != 0` on its own — governed by **Rule #1: fix the emulator's wiring only; never hack or poke the DSP's internal state.**

The current wall is an interrupt-masking / vectoring issue, not a data-path issue:

- `IMR = 0x0000` for the entire run — the DSP interrupt mask is cleared at boot by the ROM (`STM #0,IMR`) and **never re-armed**, so every delivered frame IRQ is masked, no ISR is ever vectored, `INTM` never clears, and the per-frame correlator scheduler never runs.
- The TPU frame line is delivered to the **wrong vector** (`vec19`, wake-only) whereas the firmware FB scheduler waits on `vec28/bit12` → `CALL 0xa4e4` → correlator → `d_fb_det`.
- The control mailbox itself is proven healthy (I/Q samples land in DARAM), so the defect is squarely in WAKE/IRQ signalling (INTM/IMR/vector), not the mailbox and not the I/Q path.

This is the kind of problem where the fix is a few lines in the right place — once the ARM→DSP go-live handshake is decoded correctly. If you know the Calypso DSP ROM's interrupt-arming expectations, this is exactly where your input is worth the most.

---

## Documentation

- **Subsystem index (start here):** [`doc/doc_master.md`](doc/doc_master.md) — master index: README, schematics, correlator, decoder, status reports, archived sessions.
- **Hardware reference:** `doc/CALYPSO_HW.md`, `doc/datasheets/` (TI/FreeCalypso ground-truth for C54x).
- **Project flow & threading:** [`doc/project/`](doc/project/) — `ARCHITECTURE.md`, `PROJECT_STATUS.md`, `*_FLOW.md`, `BUGS_AND_FIXES.md`, opcode audits.

---

## Use cases

- **Virtual GSM baseband for CI / interop testing** — run OsmocomBB firmware and exercise a GSM stack without physical C1xx phones or SDR hardware in the loop.
- **Baseband / DSP reverse-engineering** — a fully instrumentable Calypso where every MMR write, mailbox field, and DSP instruction can be traced.
- **Teaching / research** — an end-to-end, inspectable model of a real GSM PHY↔protocol boundary.

---

## Prior art & credits

This work stands on the shoulders of the Osmocom ecosystem:

- [**OsmocomBB**](https://osmocom.org/projects/baseband) — the free-software Calypso baseband firmware this emulator runs.
- [**FreeCalypso**](https://www.freecalypso.org/) — invaluable C54x DSP ROM analysis and Calypso hardware documentation.
- [**libosmocore / libosmocoding**](https://osmocom.org/projects/libosmocore) — GSM channel coding.
- [**QEMU**](https://www.qemu.org/) — the emulation framework this overlays.

Deep thanks to the OsmocomBB and FreeCalypso communities, whose reverse-engineering made a project like this thinkable at all.

---

## License

Files added to the QEMU source tree are licensed under **GPLv2**, consistent with upstream QEMU. Project glue (scripts, docs) is provided under the same terms unless noted otherwise. The included DSP ROM dump (`calypso_dsp.txt`) is TI silicon content, included for interoperability/research; treat accordingly for your jurisdiction.

## Author

**Bastien Baranoff** ([@bbaranoff](https://github.com/bbaranoff)) — independent telecom-security researcher (GSM/EGPRS, SDR, SS7/SIGTRAN, baseband emulation).

Contributions, corrections to the DSP interrupt/handshake model, and issues are welcome.
