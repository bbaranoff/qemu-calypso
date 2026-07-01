# qemu-calypso

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> En particulier, le récit « bug racine DSP→ARM mirror corrigé le 2026-05-15, cascade débloquée, FBSB real-path réussi » est **contredit par le runtime** : le DSP déraille (`POST-BOOTSTUB-RET`, PC=0x0000), ne lance jamais le corrélateur, `d_fb_det` reste vu à 0 tout le run, et le handshake go-live ARM→DSP ne s'arme jamais (`api_write_cb` déclaré en `calypso_c54x.h:204` mais JAMAIS assigné — `grep 'api_write_cb =' = 0`, donc le hook DSP→ARM en `calypso_c54x.c:3357` ne fire jamais). Le hack CO-RUN daté `calypso_trx.c:184-195 (2026-06-23)` — un mois APRÈS la « percée » du 05-15 — existe précisément parce que le handshake de boot pend toujours.

**QEMU emulation of the TI Calypso GSM baseband chipset** — the dual-core SoC
(ARM7TDMI + TMS320C54x DSP) used in the OpenMoko Neo, Compal e88 family, and
arguably the most reverse-engineered cellular modem in open-source history.

Runs the **real TI Calypso DSP ROM** (not a stub) and the unmodified
[osmocom-bb](https://osmocom.org/projects/baseband/) `layer1.highram.elf`
firmware on the ARM side. A Python bridge connects the BSP to `osmo-bts-trx`,
allowing the emulator to camp on a fully simulated GSM cell with
osmo-msc/hlr/bsc/stp in the back.

No firmware patching. No `#ifdef QEMU`. The whole point is that the same binaries
that run on a physical Motorola C123 also run here — and if something doesn't
work, that's where the bug lives, not in a convenient stub.

> *"You can't actually emulate a GSM phone."*
> — about half the people who looked at this when it started.

---

## What it does, concretely

```
   ┌────────────────────┐         ┌──────────────────────┐
   │  osmocom-bb mobile │ ←L1CTL→ │  layer1.highram.elf  │   ← ARM, real firmware
   │       (L23)        │         │   (ARM7TDMI + L1)    │
   └────────────────────┘         └──────────┬───────────┘
                                             │ API RAM
                                             ▼
                                  ┌──────────────────────┐
                                  │ Calypso DSP (C54x)   │   ← runs real TI ROM
                                  │ + TPU/TSP/IOTA/BSP   │   ← gated peripherals
                                  └──────────┬───────────┘
                                             │ UDP 6702
                                             ▼
                                  ┌──────────────────────┐
                                  │  calypso-ipc-device (Python)  │   ← QEMU clock-slave
                                  └──────────┬───────────┘
                                             │ UDP 5700-5702
                                             ▼
                                  ┌──────────────────────┐
                                  │     osmo-bts-trx     │   ← real BTS stack
                                  │  + osmo-msc/hlr/bsc  │   ← real core network
                                  └──────────────────────┘
```

Full GSM stack, end to end, running in software, with the DSP doing actual
correlation work on actual I/Q bursts coming from an actual GSM core network.

---

## Where the project is right now (2026-05-15)

| Layer                                              | Status                                          |
|----------------------------------------------------|-------------------------------------------------|
| QEMU SoC + ARM946 + C54x DSP cores                 | ✅ stable                                       |
| TPU → TSP → IOTA → BSP gating                      | ✅ stable                                       |
| Bridge BTS↔BSP (UDP 5700-5702 / 6702)              | ✅ 24 997 DL bursts forwarded/run               |
| DSP boot + DARAM overlay + interrupt vectoring     | ✅ POPM fixed (05-08), INTM transitions clean   |
| DSP FB-det compute (Goertzel/DFT)                  | ❌ FAUX — DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), corrélateur jamais lancé |
| **DSP→ARM API RAM mirror**                         | ~~✅ fixed 2026-05-15~~ — le mirror read existe (`calypso_trx.c:229-257`) mais ne débloque RIEN : DSP ne produit pas de résultat, `d_fb_det` reste 0 |
| FBSB success on real path (no synth)               | ❌ FAUX — jamais réussi sur le vrai path ; `d_fb_det` reste 0 |
| `task_md=24` (DSP_TASK_ALLC) firing                | ❌ FAUX — non observé (DSP déraillé)            |
| DSP writes `a_cd[]` (CCCH demod result buffer)     | ❌ FAUX — non observé                           |
| ARM L1 `prim_rx_nb::l1s_nb_resp` invoked           | ❌ FAUX — handshake go-live jamais armé (`api_write_cb` jamais câblé) |
| `d_task_d` set to `DSP_TASK_ALLC` (24) at task end | ❌ never observed → **current wall**            |
| `L1CTL_DATA_IND` forwarded to mobile               | ❌ 0 (blocked by above)                         |
| Mobile decodes SI1-SI4                             | ⏭ pending DATA_IND                              |
| RACH / Immediate Assignment / SDCCH / LU Accept    | ⏭ pending                                       |

**Test harness**: 49 pytest milestones, **26 PASS stable**, 3 SKIP, 19 XFAIL,
1 FAIL (the current wall). Each milestone is a discrete, measurable bascule
point — they flip from XFAIL → PASS one at a time as the pipeline unlocks.

---

## ~~The 2026-05-15 breakthrough — DSP↔ARM mirror~~ — PÉRIMÉ, voir banner

> ⚠️ **Cette section décrit une « percée » qui ne tient pas au runtime.** Le mirror read a bien été ajouté (aujourd'hui `calypso_trx.c:229-257`, sous `calypso_pcb_daram_lock`), mais il ne débloque pas la cascade : le DSP déraille avant de produire un résultat, `d_fb_det` reste vu à 0 tout le run, le FBSB real-path n'a jamais réussi, et `task_md=24` n'est jamais observé. Le vrai verrou est le handshake go-live ARM→DSP (`api_write_cb` jamais enregistré). Texte d'origine conservé ci-dessous à titre d'historique.

A bug that had silently blocked the project for ~6 months was localized and
fixed in **5 lines**.

### Symptom

After two months of probes investigating "the data path is broken / BSP DMA
doesn't write where the correlator reads", the real story turned out to be very
different. The DSP **was** computing correctly. It **was** producing real
Goertzel results (`0xbd2e`, `0x2014`, `0x3bb6` from PC=0x8217 op=0x9ab1, a STH
instruction). The values were being written to `d_fb_det` at `0x08F8`. The ARM
firmware was reading `d_fb_det` 192 times per FBSB cycle, looking for
`d_fb_det != 0`.

ARM was always seeing zero. 187 out of 192 reads returned `0x0000` despite the
DSP setting non-zero values mid-window.

### Root cause

`calypso_dsp_read()` was reading from `s->dsp_ram[]` (an array embedded in the
`CalypsoTRX` state). `calypso_dsp_write()` was writing both to `s->dsp_ram[]`
*and* mirroring into `s->dsp->data[]` (the C54x state's data RAM). But the
read path never had a corresponding mirror in the reverse direction. The DSP
write path went to `s->dsp->data[]`, not `s->dsp_ram[]`.

Result: every DSP write to the entire API RAM region (NDB at DSP `0x0800+`)
was invisible to the ARM read path. The two arrays drifted apart from boot
onward.

### Fix

`hw/arm/calypso/calypso_trx.c:229-257` (bloc `FIX 2026-05-15`, if/else sous `calypso_pcb_daram_lock_acquire` — PAS le ternaire cité ci-dessous, qui est une approximation d'origine ; le code réel prend un lock daram autour de la lecture) :

```c
/* === FIX 2026-05-15 : DSP→ARM mirror was missing === (calypso_trx.c:229) */
uint64_t val;
if (s->dsp && s->dsp->data) {
    calypso_pcb_daram_lock_acquire();
    uint16_t *src = &s->dsp->data[offset/2 + 0x0800];
    val = (size == 2) ? src[0] :
          (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
          ((uint8_t *)src)[offset & 1];
    calypso_pcb_daram_lock_release();
} else {
    uint16_t *src = &s->dsp_ram[offset/2];
    val = (size == 2) ? src[0] :
          (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
          ((uint8_t *)src)[offset & 1];
}
```

> Note d'audit : `calypso_trx.c:163` (cité dans une version antérieure de ce doc) est en fait dans `dsp_real_rom_mode()` (`static int v = -1;`), pas le bloc du fix.

### Effect

> ⚠️ **FAUX au runtime.** Le tableau ci-dessous n'est plus vérifié : `d_fb_det` reste 0 (ratio 0 %), le FBSB real-path ne réussit jamais, `task_md=24` n'est pas observé. Le DSP déraille (`POST-BOOTSTUB-RET`) avant de produire quoi que ce soit. Conservé à titre d'historique.

Immediate, measurable on the same build:

| Metric                            | Before fix      | After fix                  |
|-----------------------------------|-----------------|----------------------------|
| ARM RD `d_fb_det` non-zero ratio  | 0.5 %           | 2.9 %                      |
| `task_md=5` (FB-det retries)      | 782             | 90 (down 8×)               |
| `task_md=24` (DSP_TASK_ALLC)      | 0               | 20+                        |
| FBSB success on real path         | never           | yes, deterministic w/synth |

~~Real-path FBSB worked **for the first time in the history of the project**.
Mobile transitioned from "blocked pre-FBSB" to "demanding CCCH mode".~~ — **FAUX** : le FBSB real-path n'a jamais réussi ; `d_fb_det` reste 0.

---

## Quick start

```bash
# Deterministic bench (what you want for downstream debugging)
CALYPSO_ICOUNT=off CALYPSO_FBSB_SYNTH=1 ./run.sh

# Real DSP path (variance ~1-2 successful runs per 5)
CALYPSO_ICOUNT=off CALYPSO_FBSB_SYNTH=0 ./run.sh

# Full deterministic (under active development, exposes INTM dwell bug)
CALYPSO_ICOUNT=auto CALYPSO_FORCE_RX_DONE=1 CALYPSO_FBSB_SYNTH=1 ./run.sh
```

Container: `bastienbaranoff/free-bb:latest` ships the whole GSM toolchain
pre-built (osmocom-bb, osmo-bts-trx, osmo-msc/bsc/hlr/mgw, osmocon, mobile)
plus the QEMU build tree at `/opt/GSM/qemu-src/build/`.

---

## Environment variables

| Variable                       | Default       | Effect                                                                                                        |
|--------------------------------|---------------|---------------------------------------------------------------------------------------------------------------|
| `CALYPSO_ICOUNT`               | `auto`        | QEMU `-icount` mode. `auto` exposes the INTM dwell bug, `off` lives with timing variance.                     |
| `CALYPSO_FBSB_SYNTH`           | `0`           | `1` publishes synthetic FB/SB into NDB to unblock FBSB deterministically. Use `0` to exercise real path.      |
| `CALYPSO_FORCE_RX_DONE`        | `0`           | Required workaround for a TCG bug on conditional STR @ 0x8224ac (SIM busy-poll) under `-icount=auto`.         |
| `CALYPSO_W1C_LATCH`            | `0`           | `1` latches `a_sync_demod` values (DSP-write/ARM-read race mitigation).                                       |
| `CALYPSO_BSP_DARAM_ADDR`       | `0x3fb0`      | DARAM target address for BSP DMA. Doesn't affect FB-det (AR init is firmware-imposed).                        |
| `CALYPSO_DSP_IDLE_FF`          | `1`           | Fast-forward DSP idle dispatcher (pure host optimization, no semantic change).                                |
| `CALYPSO_DSP_FBDET_SKIP`       | `0`           | Diagnostic option to skip FB-det inner loop entirely.                                                         |
| `CALYPSO_NDB_D_RACH_OFFSET`    | `0x01CB`      | Override word index of `d_rach` in NDB (DSP version-dependent).                                               |
| `CALYPSO_RACH_FORCE_BSIC`      | unset         | Force BSIC in RACH encoder (0-63). Match `osmo-bsc.cfg base_station_id_code`.                                 |
| `(removed)`         | `0`           | `1` → CLK IND driven by QEMU FN. Pair with `-icount` for fully virtual time.                                  |
| `(removed)`         | `slot`        | DL FN rewrite policy (slot-aware vs naive).                                                                   |
| `(removed)`       | `32`          | Lookahead margin for DL FN rewrite.                                                                           |
| `(removed)`         | `slot`        | UL FN rewrite policy.                                                                                         |
| `CALYPSO_DSP_ROM`              | `calypso_dsp.txt` | Path to DSP ROM dump.                                                                                     |
| `CALYPSO_SIM_CFG`              | `~/.osmocom/bb/sim.cfg` | SIM IMSI/Ki config.                                                                                 |
| `L1CTL_SOCK`                   | `/tmp/osmocom_l2` | Mobile↔QEMU L1CTL Unix socket.                                                                          |

---

## Architecture

### Memory map (DSP side)

| Range            | Type                   | Content                                                |
|------------------|------------------------|--------------------------------------------------------|
| `0x0000-0x007F`  | Boot ROM stubs         | `LDMM SP,B` + `RET` at 0x0000, NOP elsewhere           |
| `0x0080-0x27FF`  | DARAM overlay (OVLY)   | Code + data, loaded by `MVPD` at boot                  |
| `0x0800-0x27FF`  | API RAM (shared)       | NDB, db_buf_w (ARM→DSP), db_buf_r (DSP→ARM)            |
| `0x2800-0x6FFF`  | Unmapped               | Reads as `0x0000`                                      |
| `0x7000-0xDFFF`  | PROM0                  | DSP ROM                                                |
| `0xE000-0xFF7F`  | PROM1 mirror           | Mirrored from page 1 (0x18000+)                        |
| `0xFF80-0xFFFF`  | Interrupt vectors      | From PROM1, IPTR=0x1FF                                 |

### Interrupt vectors

`vec = imr_bit + 16`. `addr = 0xFF80 + vec * 4`

| IRQ                | Vec | IMR bit | Address  |
|--------------------|-----|---------|----------|
| INT3 (frame)       | 19  | 3       | `0xFFCC` |
| TINT0              | 20  | 4       | `0xFFD0` |
| BRINT0 (BSP)       | 21  | 5       | `0xFFD4` |

### Repository layout

```
qemu-calypso/
├── hw/arm/calypso/                ← Calypso SoC + DSP emulator
│   ├── calypso_c54x.c             ← C54x DSP core (~13700 lines)
│   ├── calypso_trx.c              ← TRX/TPU/TSP/TDMA + DSP↔ARM mirror
│   ├── calypso_bsp.c              ← BSP DMA + UDP 6702
│   ├── calypso_iota.c             ← IOTA BDLENA gating
│   ├── calypso_fbsb.c             ← FB/SB helper (ARM-side synth)
│   ├── calypso_sim.c              ← SIM ISO 7816
│   ├── l1ctl_sock.c               ← L1CTL Unix socket
│   ├── sercomm_gate.c             ← Sercomm DLCI router
│   └── doc/
│       ├── PROJECT_STATUS.md      ← detailed project state
│       ├── TODO.md                ← next actions
│       ├── opcodes/
│       │   └── tic54x_hi8_map.md  ← tic54x reference (binutils 2.21.1)
│       └── spru172c.pdf           ← TI C54x reference manual
├── hw/{intc,char,timer,ssi}/      ← peripherals
├── tests/                         ← pytest milestone harness (49 tests)
│   ├── test_calypso_milestones.py
│   └── test_run_observability.py
├── calypso-ipc-device                      ← BTS UDP ↔ BSP relay
├── run.sh                         ← launch orchestration
├── calypso_dsp.txt                ← DSP ROM dump (132K words)
├── calypso.md                     ← Pipeline + sequence diagrams
└── CLAUDE.md                      ← AI-assistant context
```

---

## Runtime probes

The binary embeds **runtime-activable probes** that emit on QEMU stderr.
Designed to be cheap when fired sparsely, with throttling to prevent log spam.
This is the difference between "the project is stuck" and "the project tells
you why it is stuck".

| Tag                         | Target                                  | What it tells you                              |
|-----------------------------|-----------------------------------------|------------------------------------------------|
| `PC-HIST-3FB`               | reads of `[0x3fb0..0x3fbf]`             | Top PCs reading BSP DMA zone                   |
| `PC-HIST-3DD`               | reads of `[0x3dcf..0x3dd5]`             | Top PCs reading dominant scratch zone          |
| `WATCH-WRITE 0x3dd2`        | writes to `0x3dd2`                      | Writer identity + values                       |
| `INTM-TRANS`                | INTM 0↔1 transitions                    | Cause of SSBX/RSBX/STM ST1                     |
| `WAIT-A21A`                 | PC=`0xa21a`                             | INTM/IMR/IFR/ST0/ST1/SP snapshot               |
| `ENTER-7740`                | PC=`0x7740`                             | Caller chain + AR + insn                       |
| `ST1-WR`                    | STM #lk, ST1 (op 0x7707)                | All ST1 writes                                 |
| `POST-BOOTSTUB-RET`         | RET from PC ≤ 0x0008                    | Task PC popped after boot stub                 |
| `D_FB_DET-WR-SITE`          | PC=`0x8f51`                             | AR0..AR7 + data[AR0/1/2] + BK + A              |
| `D_FB_DET SET / OVERRIDE`   | writes to `0x08F8`                      | Value + PC + delta-to-clear                    |
| `D_BURST_D-WR / SUMMARY`    | writes to `0x0829` / `0x083D`           | Sequence + transition matrix                   |
| `D_TASK_D-WR`               | writes to `0x0828` / `0x083C`           | Task status set by DSP at task end             |
| `ARM RD a_cd / d_fb_det`    | ARM reads of NDB                        | Confirms whether mirror works DSP→ARM          |
| `A_CD-WR / BY-BURST`        | DSP writes to `0x09D0..0x09DE`          | CCCH demod result buffer + per-burst histogram |
| `STATE-DUMP / SP-RING`      | every N insn                            | PC + ST0/ST1 + IMR/IFR/INTM + SP + AR snapshot |

---

## Test harness (pytest)

49 tests across two files, mapped to the L1 pipeline milestones:

```
PHASE 1 — Infrastructure
  test_all_expected_processes_present, test_qemu_log_is_fresh, ...     [PASS]
PHASE 2 — DSP boot + opcode integrity
  test_popm_decoder_active, test_tier_a_decoder_fixes_present, ...     [PASS]
PHASE 3 — DSP compute convergence
  test_d_fb_det_data_no_longer_zero, test_a_cd_writes_nonzero, ...     [PASS]
PHASE 4 — FBSB
  test_synth_zero_path_active                                          [XFAIL]
  test_fb0_att_nonzero                                                 [XFAIL]
PHASE 5 — CCCH / DATA_IND
  test_l1ctl_data_ind_received                                         [XFAIL → wall]
  test_l1ctl_data_ind_rate_vs_alc
PHASE 6 — RR / MM / LU
  test_immediate_assignment_decoded, test_rach_emitted,
  test_rr_sdcch_established, test_location_updating_request_sent,
  test_location_updating_accept_received                               [XFAIL]
```

Each milestone has **three semantic states**:

- **PASS** — milestone unlocked, measured value matches assertion
- **XFAIL** — milestone known-not-met, upstream conditions absent
- **FAIL** — milestone previously unlocked, has regressed (canary)

The harness uses container-side env detection so that `synth=0` and `synth=1`
runs are interpreted differently. UTF-8-safe subprocess wrappers handle binary
bytes in `qemu.log` (STATE-DUMP raw memory dumps).

---

## ~~The current wall — DATA_IND~~ — PÉRIMÉ

> ⚠️ **FAUX.** La cascade ne « tourne end-to-end » nulle part : le DSP déraille (`POST-BOOTSTUB-RET`, PC=0x0000), `task_md=24` / écritures `a_cd[]` / `l1s_nb_resp` ne sont pas observés. Le vrai mur est en amont : le handshake go-live ARM→DSP ne s'arme jamais (`api_write_cb` déclaré `calypso_c54x.h:204`, jamais assigné), IMR=0x0000 tout le run (jamais ré-armé après le clear de boot @0xb37e). Section conservée à titre d'historique.

~~After the 2026-05-15 mirror fix, the pipeline runs end-to-end up to here:~~

```
DSP CCCH demod fires (task_md=24)              ✓  73× / run
DSP writes a_cd[0..14] result buffer           ✓  251 writes / run
ARM L1 prim_rx_nb::l1s_nb_resp invoked         ✓  60+ calls / run
ARM reads dsp_api.db_r->d_task_d               ✓
                                                   ↓
                                            d_task_d == 0  →  puts("EMPTY")
                                                              return 0
```

**60 `EMPTY` printfs observed per minute of run** — `d_task_d` at `0x0828` is
never set to `DSP_TASK_ALLC` (24) at the end of the DSP CCCH task. Some PCs
in the DSP scheduler zone (`0x787d`, `0x7a03`, `0x79f1`, `0x7817`) write
*garbage* values (`0x8dd6`, `0xfef7`, …) to that cell via what looks like
parasitic indirect addressing.

Similarly `d_burst_d` at `0x0829` is corrupted with `0x8286` by PC=`0x8216`
(in the FB-det compute zone), producing 24 `BURST ID 33414!=N` printfs per
minute.

The pattern is identical to the `0x8A00 → POPM` fix of 2026-05-08: opcodes
likely misclassified in our C54x decoder are writing to the wrong address.
Audit ongoing.

---

## Methodology

A few principles that have repeatedly paid off:

**No stubs in critical paths.** The DSP runs the real ROM. The BSP is gated
by the real TPU→TSP→IOTA chain. The mobile goes through a real QEMU PTY.
Every shortcut taken in the past (BCCH inject, FBDET-SKIP, INTM force-clear,
SI3 fallback hardcode) was eventually purged because each was hiding the
real bug. The "no hacks" rule is enforced on commits.

**Verify opcodes against tic54x-opc.c (binutils) before patching.** The
`doc/opcodes/tic54x_hi8_map.md` reference catches our
decoder where we previously had POPM misclassified as MVDK. Always cross-check.

**QEMU is clock master.** The bridge is the slave, the BTS receives CLK IND
wall-paced. This eliminates the wall-clock vs virtual-clock desync class of
bugs that plagued earlier attempts.

**Hypothesis decomposition.** When stuck for two days on "the data path
is broken", we ran probes that ruled out four hypotheses (BSP DMA target,
ROM coeffs table read, DSP compute convergence, Tier B opcode overwrite)
before arriving at "DSP→ARM mirror missing in read path". The fix was
trivial *because* the diagnosis was precise.

**Deterministic bench for downstream debugging.** When the upstream layer
has variance, replace it with a synth (e.g. `CALYPSO_FBSB_SYNTH=1`) so that
exactly one variable changes per run. Don't try to debug two layers of
non-determinism at once.

**Test after every edit.** Build in Docker, verify DSP idle + SP + IMR +
RETE count, then run pytest. The harness flags regressions immediately.

---

## Historique des sessions

### 2026-05-15 — Bug racine DSP→ARM mirror identifié et corrigé

> ⚠️ Correctif d'audit (2026-07-01) : le mirror read a bien été ajouté (`calypso_trx.c:229-257`) mais n'a PAS débloqué la cascade. Runtime : `d_fb_det` reste 0, FBSB real-path jamais réussi, `task_md=24` jamais observé, DSP déraillé. Le vrai verrou reste le handshake go-live ARM→DSP (`api_write_cb` jamais câblé). Entrée conservée telle quelle ci-dessous.

- `calypso_dsp_read()` lisait `s->dsp_ram[]` (array séparé) au lieu de
  `s->dsp->data[]`. Toutes les écritures DSP étaient invisibles côté ARM
  pour la zone API RAM. Bug en place depuis l'introduction du dual-buffer.
- Fix : bloc dans `calypso_trx.c:229-257` (l'ancienne référence `:163` était erronée)
- ~~Effet : FBSB success real path pour la première fois ; `task_md=24` passe
  de 0 à 20+ ; cascade débloquée jusqu'au mur `d_task_d`~~ — FAUX au runtime (voir note ci-dessus)
- Pytest harnais étendu à 49 milestones, 26 PASS stables
- Test `icount=auto` exploratoire : architecture viable mais expose un
  bug INTM dwell systématique (à attaquer en session dédiée)

### 2026-05-08 — POPM fix + opcode audit

- `0x8A00` était décodé en `MVDK Smem,dmad` ; tic54x-opc.c l'identifie
  comme `POPM MMR`
- Conséquence : INTM stuck à 1 perpétuel après ~98M insn, depuis avril 2026
- Audit complet hi8 → mnémonique tic54x (binutils 2.21.1) créé en
  `doc/opcodes/tic54x_hi8_map.md`
- 8 opcodes additionnels stubés en NOP (`0x8B`, `0xAA/AB`, `0xC5`, `0xCD`,
  `0xCE`, `0xDD`, `0xDE`, `0x80`) pour stopper les writes parasites
- DSP throughput ×5

### 2026-05-07 — Purge des hacks

- `rsl_si_tap.py`, `CALYPSO_BCCH_INJECT`, `CALYPSO_SI_MMAP_PATH` supprimés
- `BOURRIN-FBDET-SKIP` supprimé
- `DIAG-HACK INTM force-clear` supprimé
- `si3_fallback[]` hardcode supprimé
- `allc_burst_idx` static cycle remplacé par `fn & 3`

### 2026-04-29 — Opcode dispatch baseline

5 fixes structurels validés empiriquement, ~2530 sites firmware débloqués :

| # | Fix | Impact |
|---|---|---|
| 1 | Reset silicon-aligné (PMST=0xFFA8, ST0=0x181F, ST1=0x2900) | DSP entre PROM1 init zone |
| 2 | `0x6F00` ext dispatch | Wedge PC=0x8353 (2.2G iter) éliminé |
| 3 | `0x68-0x6E` handlers (ANDM/ORM/XORM/ADDM/BANZ/BANZD) | 1563 sites unblocked |
| 4 | APTS misnomer fix (PMST bit 4 = AVIS, pas stack) | Stack leak 1.96M events → 0 |
| 5 | `F3xx` complet (AND/OR/XOR/SFTL + #lk variants) | 364 sites, wedge PC=0x8eb9 |

Sessions antérieures : voir `doc/SESSION_*.md`.

---

## Build

```bash
docker exec CONTAINER bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"
```

Workaround `-lm` link (intermittent) :

```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh && bash /tmp/link.sh
```

---

## Conventions

- **No stubs in critical paths.** No `#ifdef QEMU`. No "good enough"
  shortcuts that hide the real bug.
- **Verify opcodes against `tic54x-opc.c`** before patching. See
  [`doc/opcodes/tic54x_hi8_map.md`](doc/opcodes/tic54x_hi8_map.md).
- **QEMU is clock master.** Bridge is slave. BTS gets CLK IND wall-paced.
- **Test after every edit.** Build, run pytest, verify no regression on
  the 26 stable PASS milestones.
- **Document workarounds** in [`doc/TODO.md`](doc/TODO.md)
  with explicit removal criteria.

---

## License & attribution

- **QEMU base** : GPL-2.0-or-later (upstream QEMU)
- **Calypso emulator additions** : GPL-2.0-or-later
- **osmocom-bb firmware** : GPL (used as-is, not redistributed here)
- **Calypso DSP ROM** (`calypso_dsp.txt`) : TI proprietary. Physical device
  dump for research and interoperability purposes (osmocom DSP dumper).
  Not commercially redistributable without TI authorization.

---

## Related projects

- [osmocom-bb](https://osmocom.org/projects/baseband/) — the open-source GSM
  baseband stack this emulator runs unmodified
- [osmo-bts](https://osmocom.org/projects/osmobts/) — open-source GSM BTS
  (paired in the bridge architecture)
- [osmo-cn](https://osmocom.org/projects/cellular-infrastructure/) —
  open-source cellular core network
- [QEMU](https://www.qemu.org/) — generic emulation framework this is built on
- [tic54x binutils](https://sourceware.org/binutils/) — TMS320C54x assembler
  reference used for opcode audit

---

## Contact

Issues, patches, hardware questions: open an issue or get in touch.

For commercial licensing of the GSM-over-LoRa / baseband emulation work this
project enables, contact directly.

---

> *If a piece of hardware exists, it can be emulated.*
> *If a protocol exists, it can be simulated.*
> *If a chip's ROM is dumpable, its firmware can run anywhere.*
>
> *Six months of dwell. Five lines to unlock.*
