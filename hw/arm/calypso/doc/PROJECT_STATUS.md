# Calypso GSM Baseband Emulator — Project Status

## 2026-05-07 — UL pipeline wired + hack cleanup

**State** : DL milestone preserved, UL chain instrumented end-to-end, all
documented hacks either removed or env-gated. Next blocker discriminated to
`RACH→BTS→IMM_ASS_CMD` round-trip.

### Verified live (this session)

With `CALYPSO_FBSB_SYNTH=1 ./run.sh` :
- DL : `<0001> sysinfo.c New SYSTEM INFORMATION 3 (lai=001-01-1)` reappears
  in mobile log ; SI1/SI2/SI3/SI4 all decoded ; `Changing CCCH_MODE to 2`
- UL : 8 packets delivered to BTS via bridge UL, including
  `bridge: UL #7 TN=0 fn=1480` and `UL #8 TN=0 fn=1480` (RACH-slot,
  vs noise-slots TN=4/6 we saw on first instrumentation)
- bridge stats : `tick=5931 clk=68 dl=54996 ul=N` — CLK IND skew greatly
  reduced vs pre-icount (1-FN compensations vs 102-FN before)

Without `CALYPSO_FBSB_SYNTH` : DSP runs the real fb-det against bridge-fed
GMSK samples, doesn't converge → mobile loops `L1CTL_FBSB_REQ`.

### Wiring added

| Connection | File | Notes |
|---|---|---|
| `DB_W_D_TASK_RA` polling (write-page word 7) | `calypso_trx.c::tdma_tick` | RACH was silently dropped — `d_task_u` (word 2) is the SDCCH/SACCH/TCH lane, not RACH |
| `calypso_bsp_tx_rach_burst` | `calypso_bsp.c` | `gsm0503_rach_ext_encode` produces real 148-symbol AB ; `d_rach` read at `CALYPSO_NDB_D_RACH_OFFSET` (default 0x01CB) |
| libosmocoding link | `hw/arm/calypso/meson.build` | `dependency('libosmocoding')` |
| `bsp.trxd_peer` pre-set | `calypso_bsp.c::calypso_bsp_init` | UL works before first DL |
| `bridge.trxd_remote` pre-set | `bridge.py::Bridge.__init__` | UL forwards to (BTS, base+102) immediately |
| `BRIDGE_CLK_FROM_QEMU` env mode | `bridge.py` | CLK IND from QEMU FN advance, not host wall-clock |
| `-icount shift=auto,align=off,sleep=off` | `run.sh` | Deterministic virtual time |

### Hacks removed (per CLAUDE.md règle #1)

| Hack | File | Line span |
|---|---|---|
| BOURRIN-FBDET-SKIP block (PC range pop+jump bypass) | `calypso_c54x.c` | ~864-899 (36 lines) |
| DIAG-HACK INTM force-clear + ALIAS-CHECK + BOOT+100k VECDUMP | `calypso_c54x.c` | ~4950-5066 (~120 lines) |
| `si3_fallback[23]` hardcoded SI3 | `calypso_fbsb.c::DSP_TASK_ALLC` | inline literal |
| `allc_burst_idx` static cycle 0..3 | `calypso_fbsb.c::DSP_TASK_ALLC` | replaced by `fn & 3` |

### Env-gated (default OFF — real path) ; on retains DL milestone

| Env var | Effect |
|---|---|
| `CALYPSO_FBSB_SYNTH=1` | `calypso_fbsb_publish_fb_found` + `_publish_sb_found` re-enabled in `on_dsp_task_change`. Used while emulated DSP correlator does not converge on bridge GMSK. |
| `CALYPSO_NDB_D_RACH_OFFSET=0xNNN` | Override d_rach word index in NDB (verify offset for active DSP version). |
| `BRIDGE_CLK_FROM_QEMU=1` | Replaces wall-clock-paced CLK IND with QEMU-FN-paced. Pair with `-icount`. |

### Current blocker

Mobile sends RACH, no `IMM_ASS_CMD` reaches L3. Two failure modes not yet
discriminated :
- (a) BTS doesn't decode our UL RACH bursts (encoding off, FN off, BSIC
  mismatch, or burst arrives outside BTS RACH detect window)
- (b) BTS does decode and BSC sends IMM_ASS, but mobile DSP misses the
  AGCH sub-slot on the CCCH multiframe

Test discriminant : tcpdump GSMTAP during a `CALYPSO_FBSB_SYNTH=1` run.
- IMM_ASS visible on air → (b) — debug DL AGCH sub-slot scheduling
- Absent + `osmo-bts-trx` RACH detect counter at 0 → (a) — debug UL
  burst encoding (verify d_rach offset, BSIC source, FN tagging)

### Co-issue discovered : task_ra spurious values

`task_ra` and `task_u` both read non-zero at fn=104 with seemingly random
values (e.g. `task_ra=0x2d4e tn=6 fn=104`, `d_rach=0x19a9 ra=0x19 bsic=0x2a`).
BSIC varies run-to-run, RA looks plausible. Two hypotheses :
- d_rach offset wrong → reading garbage from a different NDB slot
- Firmware actually writes nonzero values to d_task_ra during init that we
  pick up as legitimate RACH triggers

Open : trace API RAM writes to (page word 7) and to candidate d_rach
offsets during a known RACH attempt (after Location Update L3 starts) to
filter noise from real RACH commands.

---

## 2026-04-30 13:00 — Milestone L3 mobile-side via dynamic RSL tap

- BCCH SI injection: 100% dynamic from osmo-bsc live RSL
  - `rsl_si_tap.py` sniffs lo:3003, parses RSL_MT_BCCH_INFO (0x11),
    extracts FULL_BCCH_INFO IE (0x27) by SYSTEM_INFO_TYPE IE (0x1e)
  - Writes `/dev/shm/calypso_si.bin` per `doc/MMAP_SI_FORMAT.md` v1
- QEMU mmap consumer in `calypso_fbsb.c` (`csi_init_once` + `csi_lookup_for_tc`)
- TC scheduling per TS 44.018 §3.4 table 1 (SI3 emitted 3× per cycle)
- Fallback hardcoded SI3 in `calypso_fbsb.c` retained as cold-start safety,
  inactive when tap operational
- `populate-si.sh` retained as manual debug tool (NOT in run_si.sh boot path
  any more; tap covers warm-start dès osmo-bts attach)

Mobile L23 reaches:
- `New SYSTEM INFORMATION 1, 2, 3` parsed (lai=001-01-1)
- `Changing CCCH_MODE to 2`
- MM cell-selected (CGI 001-01-1-6001)
- `RR_EST_REQ` → state idle → connection pending
- `CHANNEL REQUEST: 00 (Location Update with NECI)`
- `RANDOM ACCESS` bursts × 8 (max retransmits)
- T3126 timeout (5s) — no IMM ASSIGN downstream
- Cycle: T3126 fired → return idle → re-RR_EST_REQ → re-CHANNEL REQUEST

Blockers downstream:
1. UL chain broken (3 sub-issues per diag) :
   - DSP-emulated RACH encoding absent in `calypso_trx.c`
   - `d_task_u` read returns garbage (firmware doesn't write proper RACH task)
   - `bridge.py` has 0 UL forward handler (no UDP 5701 sender)
2. AGCH downlink not synthesized — IMM ASSIGN never reaches mobile

---

## 2026-04-30 nuit — BCCH pipeline end-to-end validé (étape 2)

**Milestone L2 atteint** : pipeline BCCH descend de QEMU jusqu'à L23 mobile,
avec payload structuré byte-à-byte vérifié.

| Indicateur | Valeur | Notes |
|---|---|---|
| `L1CTL_DATA_IND` (msg_type 0x03) traversent osmocon | **88** | vs 0 hier matin |
| `L1CTL_FBSB_CONF` result=0 | **24** | régulier |
| ALLC echo+SI3 hooks fired | 360 | task=24 dispatched |
| `num_biterr` | `0x00` systématique | a_cd[2] write OK |
| `fire_crc` | `0x00` systématique (NO ERROR) | a_cd[0] FIRE bits OK |
| Payload byte-match si3_blob[0..22] | **23/23 bytes exact** | NDB→ARM mailbox validé |

**3 fixes nouveaux cette session** :

| # | Fix | File | Empirical impact |
|---|---|---|---|
| 9 | **ALLC echo + DSP_TASK_ALLC handler** | `calypso_fbsb.c` + `.h` | Passe guard `EMPTY` de `prim_rx_nb.c:73` (était EMPTY×56 avant fix). |
| 10 | **a_cd[] base offset shift +2 words** (0x01D0→0x01D2) | `calypso_fbsb.c` | Project memory disait NDB+0x1F8 mais runs montrent shift de 2 words. Empirique : data byte 0 maintenant = si3_blob[0] (=0x1b), num_biterr=0 propre. |
| 11 | **Fix re-arm TDMA timer** (`entry_t + while-skip`) | `calypso_trx.c::calypso_tdma_tick` | now=exit_t accumulait work_dt → cadence drift 11ms. entry_t + while-skip aligné grille. Forward progress firmware (vs fix-a saturation). |

**XXX TEMP HARDCODE — à retirer impérativement** (cf. `hacks.md` + `TODO.md`) :
- `si3_blob[]` fixture libosmocore BSSGP — viole règle #1 "no stubs". Critère
  retrait : intercept bridge.py / osmo-bts pour SI3 réel du BSC.
- `allc_burst_idx` static counter — off-by-one connu (75% mismatch). À
  remplacer par frame-tick scheduled write basé sur fn modulo 51.

**Validation byte-à-byte** :
```
si3_blob[]   : 1b 75 30 00 f1 10 23 6e c9 03 3c 27 47 40 79 00 00 3c 0b 2b 2b 2b 2b
DATA_IND obs : 1b 75 30 00 f1 10 23 6e c9 03 3c 27 47 40 79 00 00 3c 0b 2b 2b 2b 2b
                ─────────────── 23 bytes MATCH ───────────────
```

**NON validé** : L23 SI3 parsing côté mobile (pas de marker "system info" /
"cell synced" / "LU REQ" visible dans logs locaux). Fixture libosmocore est
BSSGP test, pas BCCH air-interface — format LAPDm pseudo-length probablement
incompatible. Trois hypothèses (a/b/c) à départager via verbosity mobile +
GSMTAP capture, ou directement par intercept osmo-bts.

**Doc nouvelle** : `doc/hacks.md` — inventaire honnête des 3 niveaux de
hacks/workarounds par fichier. Référence pour audit B2B / licensing.

**Prochaine session** :
1. Verbosity mobile + GSMTAP capture pour départager (a)/(b)/(c) du parsing SI3.
2. Si fixture invalide : intercept osmo-bts → bridge.py → QEMU pour SI3 réel.
3. Fix off-by-one `allc_burst_idx` (×4 gain DATA_IND si nécessaire pour LU).

---

## 2026-04-29 afternoon — FBSB chain validée E2E

**Milestone L1 atteint** : 46× `L1CTL_FBSB_CONF` result=0 traversent firmware → osmocon → cell_log. Cell_log entre `SCAN_STATE_READ` (vérifié par lecture directe `cell_log.c:402`, transition inconditionnelle).

**3 fixes nouveaux cette demi-session** :

| # | Fix | File | Empirical impact |
|---|---|---|---|
| 6 | **ARP off-by-one** dans `resolve_smem` + BANZ/BANZD/F80x | `calypso_c54x.c` | `cur_arp = nar` (était `arp(s)`). DSP exécute correctement les indirect addressing modes. |
| 7 | **fbsb wire reintroduction** au site ARM TASK WR=5 | `calypso_trx.c::calypso_dsp_write` + `#include "calypso_fbsb.h"` | Module `calypso_fbsb` était linké mais pas wire (perdu collatéralement preNoCell refactor 28/04). Publish synthétique FB+SB sur task=5/6. |
| 8 | **W1C latch invalidation** dans `publish_fb_found` / `clear_fb` | `calypso_fbsb.c` | Sans invalidation, latches DSP iter masquent valeurs synthétiques fbsb (ARM lit angle=-29569 stale au lieu de 0 fresh). |

**Validation E2E** :
- `osmocon -d r` dump : 46× `hdlc_recv(dlci=5): 02 ...` (msg_type FBSB_CONF, result=0)
- `cell_log.c:412 case S_L1CTL_FBSB_RESP` : `state = SCAN_STATE_READ` + `start_timer(READ_WAIT)` inconditionnel
- Compteur "syncs left" décrémente 47→36 sur run continu
- `LOGP(DRR, ...)` "Synchronized, start reading" filtré par `app_cell_log.c:86 log_parse_category_mask("DSUM")` — invisible mais code path exécuté

**Blocker LU restant** : 0× `L1CTL_DATA_IND` (msg_type 0x03) → BCCH read jamais complet → READ_WAIT timeout → cell_log re-scan ARFCN. Cause double :
1. **Stabilité timing TDMA insuffisante** (priority 1) : ~12000+ "LOST N!" sur run, distribution bimodale ~2470/~5020 (pas du bruit aléatoire). 28% des frames TDMA marquées LOST. Probabilité tenir 10 frames clean consécutives (BCCH multiframe) ≈ 3.7%.
2. `calypso_fbsb` ne synthétise pas BCCH bursts (DSP_TASK_NB_RX falls-through default).

**Prochaine session** : timer fidelity AVANT BCCH synthesis. Sans timing stable, BCCH synth sera rejeté.

---

## Latest update — 2026-04-29 (see `SESSION_20260429.md` for full report)

5 structural opcode-dispatch fixes validated empirically. ~2530 firmware sites unblocked.
DSP CPU emulation, opcode dispatch, reset values, and ISR mechanism are all
silicon-aligned (binutils tic54x-opc.c + SPRU172C/131G + 3 FreeCalypso ROM dumps).

| # | Fix | Sites unblocked | Marker |
|---|---|---|---|
| 1 | Silicon-aligned reset (PMST=0xFFA8, ST0=0x181F, ST1=0x2900) | — | DSP enters PROM1 init zone |
| 2 | 0x6F00 ext dispatch (ADD/SUB/LD/STH/STL Smem,SHIFT,DST) | 544 | Wedge PC=0x8353 → 0 |
| 3 | 0x68-0x6E handlers (ANDM/ORM/XORM/ADDM/BANZ/BANZD) | 1563 | DSP traverses init |
| 4 | APTS misnomer fix (5 FAR opcodes always push/pop XPC) | — | Stack leak 1.96M → 0 |
| 5 | F3xx complete dispatch (AND/OR/XOR/SFTL + #lk variants) | 364 | Wedge PC=0x8eb9 unblocked |

**Final blocker:** INTM=1 forever (3 mesures empiriques, 100% strict). Mécanisme
silicon de clear non documenté publiquement (TI Calypso DBB datasheet privée).
Diagnostic instrument `CALYPSO_FORCE_INTM_CLEAR_AT=N` documenté comme tel,
pas un workaround. Voir `SESSION_20260429.md` § "INTM=1 final blocker".

**Files added this session:** `doc/datasheets/` (5 PDFs TI + Calypso overview FreeCalypso
+ 3 ROM dumps + README), `doc/opcodes/0x68_0x6F.md`, `doc/opcodes/0xF3.md`,
`doc/SESSION_20260429.md`, `scripts/inject_fcch.py`.

---

## Goal
Run real OsmocomBB layer1.highram.elf firmware on emulated TI Calypso (ARM7 + TMS320C54x DSP) in QEMU. Connect to a real BTS via TRX protocol through a bridge. Mobile/ccch_scan sees the BTS and decodes BCCH/CCCH.

## Architecture
```
BTS (osmo-bts-trx)
  ↕ UDP (TRXC 5701, TRXD 5702, CLK 5700)
bridge.py
  ↕ PTY/UART (sercomm DLCI 4 = bursts, DLCI 5 = L1CTL)
QEMU (ARM7 calypso_trx.c + C54x calypso_c54x.c)
  ↕ Unix socket /tmp/osmocom_l2_1 (L1CTL length-prefixed)
mobile / ccch_scan (OsmocomBB host tools)
```

## Key Files
| File | Role |
|------|------|
| `calypso_c54x.c` | TMS320C54x DSP emulator (~1800 lines) |
| `calypso_c54x.h` | DSP state struct, constants, API |
| `calypso_trx.c` | Calypso SoC: DSP API RAM, TPU, TDMA, burst RX |
| `l1ctl_sock.c` | Unix socket L1CTL ↔ sercomm bridge |
| `calypso_uart.c` | UART with sercomm DLCI routing |
| `calypso_inth.c` | ARM interrupt controller |
| `calypso_soc.c` | SoC glue, memory map |
| `calypso_mb.c` | Machine/board definition |
| `bridge.py` | BTS TRX UDP ↔ UART sercomm bridge |
| `run.sh` | Launches QEMU + bridge + BTS + ccch_scan in tmux |
| `sync.sh` | Syncs files between host and docker container |
| `l1ctl_test.py` | Direct L1CTL test script |

## DSP ROM
- Dumped from real Motorola C1xx phone via osmocon + ESP32
- File: `/opt/GSM/calypso_dsp.txt` (131168 words)
- Sections: Registers [00000-0005f], DROM [09000-0dfff], PDROM [0e000-0ffff], PROM0 [07000-0dfff], PROM1 [18000-1ffff], PROM2 [28000-2ffff], PROM3 [38000-39fff]
- DSP version: 0x3606

## Memory Map (DSP side)
| DSP Address | ARM Address | Content |
|-------------|-------------|---------|
| 0x0000-0x001F | — | MMR (registers) |
| 0x0020-0x007F | — | DARAM (low, scratch) |
| 0x0080-0x07FF | — | DARAM (overlay with prog when OVLY=1) |
| 0x0800-0x27FF | 0xFFD00000-0xFFD03FFF | API RAM (shared ARM↔DSP) |
| 0x7000-0xDFFF | — | PROM0 (program ROM) |
| 0x8000-0xFFFF | — | PROM1 mirror (program ROM page 1) |
| 0x9000-0xDFFF | — | DROM (data ROM) |
| 0xE000-0xFFFF | — | PDROM (patch data ROM) |

## API RAM Layout
| ARM Offset | DSP Address | Name |
|------------|-------------|------|
| 0x0000 (W_PAGE_0) | 0x0800 | Write page 0 (MCU→DSP, 20 words) |
| 0x0028 (W_PAGE_1) | 0x0814 | Write page 1 |
| 0x0050 (R_PAGE_0) | 0x0828 | Read page 0 (DSP→MCU, 20 words) |
| 0x0078 (R_PAGE_1) | 0x083C | Read page 1 |
| 0x01A8 (NDB) | 0x08D4 | d_dsp_page (first word of NDB) |
| 0x0862 (PARAM) | 0x0C31 | DSP parameters |

## d_dsp_page values
- `0x0000` = no task
- `0x0002` = B_GSM_TASK | page 0
- `0x0003` = B_GSM_TASK | page 1
- B_GSM_TASK = (1 << 1) = 0x0002

## Write page structure (T_DB_MCU_TO_DSP, 20 words)
| Offset | Field |
|--------|-------|
| 0 | d_task_d (downlink task) |
| 1 | d_burst_d |
| 2 | d_task_u (uplink task) |
| 3 | d_burst_u |
| 4 | d_task_md (monitoring/FB/SB task: 5=FB, 6=SB) |
| 5 | d_background |
| 6 | d_debug |
| 7 | d_task_ra |
| 8-19 | results area |

## DSP Boot Sequence
1. ARM writes DSP_DL_STATUS_READY
2. `c54x_reset()` → PMST=0xFFE0, IPTR=0x1FF, OVLY=0, PC=0xFF80
3. `c54x_run(10M)` → DSP executes PROM1 reset code → calls PROM0 init → 86K insns → IDLE@0xFFFE
4. ARM continues, firmware initializes
5. Each TDMA frame: ARM writes d_dsp_page + tasks, fires TPU_CTRL_EN
6. `calypso_dsp_done()` → copies write page to DARAM 0x0586, wakes DSP
7. DSP jumps to 0x8000 (TDMA slot table), processes, returns to IDLE

## TDMA Slot Table (PROM1 0x8000-0x801F)
```
0x8000: 12f8 3fc5 f4e3 f4e4  → slot 0: SUB data[0x3FC5], A; SSBX INTM; IDLE
0x8004: 12f8 322a f4e3 f4e4  → slot 1
...8 slots of 4 words each...
0x8020: processing code starts (reads d_dsp_page, dispatches tasks)
```

## Frame Dispatch Routine (PROM0)
- Entry: 0xC8E7 → reads d_dsp_page at DSP 0x08D4
- Configures page pointers (0x0800/0x0814/0x0828/0x083C)
- Called via BANZ at 0xC8CD
- d_dsp_page also read at 0xA51C during init

## Bugs Fixed This Session
1. **Timer HW (TCR/PSC/TDDR)** — Real C54x timer behavior, TSS=1 at reset
2. **IDLE PC** — return 0 (stay at IDLE addr for interrupt handler)
3. **PC wrap 16-bit** — `s->pc &= 0xFFFF`
4. **F4E2/F4E3 (RSBX/SSBX INTM)** — 98 occurrences decoded as BD instead of interrupt enable/disable. CRITICAL BUG.
5. **Interrupt vec = bit + 16** then reverted to **SINT17 vec 2 bit 1** — correct vector for TPU frame
6. **FRET** — Must pop 2 words (XPC then PC) per SPRU172C p.4-61
7. **FCALL** — Must push 2 words (PC+2 then XPC) per SPRU172C p.4-57
8. **XC (Execute Conditionally)** — Opcode 0xFDxx (n=1) / 0xFFxx (n=2), NOT 0xEAxx. 0xEA = BANZ.
9. **TRXD header** — 8 bytes (TN+FN+RSSI+TOA), not 6. Soft bit conversion.
10. **L1CTL sequencing** — 1 message per callback to let ARM process between messages
11. **OVLY activation** — Enabled after boot for DARAM code execution
12. **IDLE wake** — Jump to 0x8000 (TDMA loop) on wake from boot IDLE
13. **Interrupt PC+1** — Push PC+1 when waking from IDLE (resume after IDLE)
14. **Timer FN increment** — TINT0 increments frame number in DARAM

## Current State (2026-04-03)
- Base: no_cell_found (API RAM intercepts for FB/SB/PM)
- No_cell_found loop works: RESET→PM→FBSB cycle running
- C54x DSP boots in parallel via TDMA ticks (2000 insns/frame)
- SINT17 controlled by TPU_CTRL_DSP_EN (per hardware spec)
- 0 UNIMPL instructions — all opcodes emulated
- SP=0x06AA-0x0C2E after boot (correct, in DARAM)
- DSP reaches PROM0 0x7000 init + PROM1 0x8159 processing

## Bugs Fixed (session 2026-04-03)
1. **RC conditionnel (F2xx)** — was unconditional RET, now eval_condition
2. **PROM0 read protection** — prog_read returns prog[] for 0x7000-0xDFFF
3. **PROM0 write protection** — prog_write ignores writes to ROM area
4. **prog_write double-write** — return after prog[ext] for addr>=0x8000
5. **DELAY instruction (D4/D5)** — pipeline delay, was UNIMPL
6. **BCD 0xEF** — added to EE/ED branch conditional handler
7. **XOR/OR #lk16 (B0/B1/B8/B9)** — was UNIMPL
8. **Timer0 hardware** — TIM/PRD/TCR with prescaler and TINT0
9. **IDLE skip TDMA slots** — 0x8000-0x801F IDLE treated as NOP
10. **Parallel DSP boot** — no blocking c54x_run(10M), boot via TDMA ticks
11. **DSP_EN SINT17** — interrupt only when firmware sets TPU_CTRL_DSP_EN
12. **eval_condition** — full XC/RC condition decoder per SPRU172C Table 3-2
13. **Interrupt dispatch** — check IFR&IMR in main loop each cycle
14. **API IRQ** — raise IRQ15 in dsp_done, unmask at boot

## d_fb_det Location
- ARM offset: 0x01F0 (NDB + 0x48)
- DSP address: 0x08F8
- DSP ROM writes it at PROM0 0x7730-0x7990

## Next Steps
1. DSP boot must reach IDLE@0xFFFE — currently runs but doesn't converge
2. Once boot works, remove API RAM intercepts (let DSP produce results)
3. Verify burst samples reach DSP at correct DARAM address
4. Achieve FBSB_CONF(result=0) → SB decode → BCCH → mobile registered

## Docker
- Container: `osmo-operator-1` (always running)
- Images: `calypso-qemu:YYYYMMDD-HHMM` (snapshots)
- Build: `ninja -C /opt/GSM/qemu-src/build`
- Run: `bash /opt/GSM/qemu-src/run.sh`
- DSP ROM: `/opt/GSM/calypso_dsp.txt`
- Firmware: `/opt/GSM/firmware/board/compal_e88/layer1.highram.elf`

## Reference Documents
- `doc/spru172c.pdf` — TMS320C54x DSP Reference Set Volume 2: Mnemonic Instruction Set
- `doc/C54X_INSTRUCTIONS.md` — Key instruction encodings extracted from SPRU172C
