# QEMU Calypso — Claude Code Context

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), IMR=0x0000 tout le run (jamais ré-armé après le clear boot @0xb37e), api_write_cb jamais câblé, pas de bus ORCH. Le handshake ARM→DSP go-live ne s'arme jamais (l'ARM n'écrit que 0x0000 dans API 0x0314/0x0318). Corrections ci-dessous.

## 🔄 SYNC MIROIRS — À FAIRE À CHAQUE ÉDITION

Le **latest / autoritaire** = `trying:/opt/GSM/qemu-src` (conteneur, où on build).
**Après CHAQUE fichier édité**, propager depuis ce latest vers :
- `/home/nirvana/qemu-src` (miroir full host)
- `/home/nirvana/qemu-calypso` (subset curated host, même layout `hw/arm/calypso/` + `include/` + `run.sh`)

**On IGNORE `/opt/GSM/qemu-calypso`.** Commande type (depuis latest) :
```bash
for dst in /home/nirvana/qemu-src /home/nirvana/qemu-calypso; do
  mkdir -p "$dst/$(dirname FILE)"; docker cp trying:/opt/GSM/qemu-src/FILE "$dst/FILE"; done
```
Inclure ce CLAUDE.md lui-même dans la propagation.

## Working style with user

Quand l'user me donne un nouvel ordre, je **continue les taches precedentes en parallele**. Je n'abandonne PAS le contexte courant. Si un fix etait en cours et l'user demande autre chose, je termine le fix ET je traite la nouvelle demande. La file de taches s'enchaine, elle ne se reset pas.

## ⚠️ RUNS NON-DÉTERMINISTES — RÈGLE INVARIANTE

**Le runtime QEMU+stack est non-déterministe**. Démontré 2026-05-28 par 3 runs
manuels du même binaire produisant 3 signatures complètement différentes :
- `d_fb_det LATCH-CONSUME` varie (87× 0x1ef8 / 96× 0x1255 / 0 / 27× 0x0c4a / ...)
- `BURST-IN` count varie (10..600+)
- `SOFT-RESET-TRIG` count varie (6, 60, 26)
- Hot poll cells varient (0x2e80 vs 0xFFEx vs ailleurs)
- Mêmes `a_pm` valeurs jamais reproduites entre runs

**Cause** : pipeline real-time, BSP recvfrom socket UDP wall-clock-paced, drain
timer virtual mais supply async. Tout cascade.

**RÈGLE pour Code dans cette session ET futures** :
- **JAMAIS conclure d'un seul run** qu'un fix a marché ou échoué.
- Une signature observée dans 1 run = un *tirage* parmi la distribution possible.
- Pour valider un fix : observer un **invariant structurel** (= ce qui se passe
  dans CHAQUE run même si les valeurs précises varient). Exemples d'invariants
  valides :
  - Existence d'un type d'event qui n'existait pas (BRINT0 fire avant/après)
  - Disparition d'un hot poll dominant (0x16b3 polling stopped post-BRINT0 fix)
  - Type de routine atteint (PC=0xda79 real publisher vs stale-AR 0x821a)
- Pour confirmer un blocage : 2-3 runs same-binary, vérifier que le pattern
  RESTE même si les valeurs précises varient.
- Si une seule run est interprétée → préciser à l'user "cette observation
  vient d'1 run, peut varier au prochain".

**Anti-pattern à éviter** : dire "telle valeur prouve telle chose" ou
"ce fix a marché" sur la base d'1 run. Ce qu'il faut dire : "cette valeur
observée dans 1 run suggère X ; à confirmer cross-run".

## Dual-agent setup

L'user travaille en parallèle avec **Claude Code** (ici, accès fichiers/git/build)
et **Claude.ai web** (chat, dit "c web"). Il précise quand il bascule. **c web
est le reviewer** : il challenge l'approche/les hypothèses/les hacks avant
qu'on code. Quand le user paste un diag ou un plan de c web : prendre au
sérieux mais croiser avec `git diff` et l'état réel du code avant fix. Quand
c web demande des headers/grep, sortir propre (file:line, struct entière) pour
que le user puisse copier-coller dans le chat web. Cf. mémoire
[[feedback_dual_agent_division]].

## Session pickup (2026-05-08 night → next session)

```
DONE
- DL FN rewrite slot-aware implemented in calypso-ipc-device
  → BSP delta passed from 15000 → 0..32 (within BSP_FN_MATCH_WINDOW=64)
  → env (removed)=slot|naive|off, default slot
  → env (removed)=32 (half BSP window margin)
- BCCH_INJECT / FBSB_SYNTH purge clean
  → scripts/rsl_si_tap.py deleted entirely
  → CALYPSO_BCCH_INJECT + CALYPSO_SI_MMAP_PATH env vars deleted
  → calypso_fbsb.c: csi_*, bcch_inject_*, ALLC inject block deleted
  → run_si.sh clears /dev/shm/calypso_si.bin at startup (inter-run hygiene)
- Force test daram[0x62]=1 (CALYPSO_DSP_FORCE_DARAM62=1 in calypso_c54x.c
  data_read override, env-gated) :
  → DSP escapes idle dispatcher loop cc62..cc6f
  → traps in NEW polling loop at df93..dfb1
  → DARAM RD HIST pattern shifts radically (0060/2460/2413/2ee0/3dd2 family)
  → Confirms gate hypothesis : INT3 ISR (or equivalent) is not writing
    dispatcher flags. Forcing one flag bypasses one gate, but more gates
    sit downstream — the ISR writes multiple flags per frame, not just 0x62.

NEXT
- Trace WRITES on DARAM[0x60..0x70] in calypso_c54x.c data_write hook
  → If zero writes: INT3 wiring DSP path is the culprit, open the
    interrupt wiring front (calypso_inth.c IMR/IRQ routing to DSP)
  → If some writes but cleared too fast: timing / race
- Cross-check PROM0 handler at 0x770c (the dispatch target read at
  api[0x1f0c] = api[0x1f00] = 0x770c) — what does it expect on entry ?
- Same instrumentation pattern as IDLE-DISP RD : add IDLE-DISP-2 trace
  for PC ∈ 0xDF93..0xDFB1, observe what flag IT polls.

KEY FILES TOUCHED
- calypso-ipc-device                        (DL FN rewrite + slot-aware UL retained)
- run_si.sh                        (cleanup + new envs + ENV summary)
- hw/arm/calypso/calypso_fbsb.c    (mmap consumer + BCCH_INJECT removed)
- hw/arm/calypso/calypso_c54x.c    (IDLE-DISP RD trace + FORCE-DARAM62)

DROPPED ENVS (do not re-set)
  CALYPSO_BCCH_INJECT  CALYPSO_SI_MMAP_PATH

DEFAULTS (run_si.sh)
  (removed)=0           (wall-paced, BTS happy)
  (removed)=51             (BTS skew tolerance)
  (removed)=slot
  (removed)=slot
  (removed)=32
  CALYPSO_FBSB_SYNTH=0             (set =1 to keep mobile past FBSB phase)
```

> **⚠️ PAS DE HACK — règle #1.**
> Pas d'injection, pas de stub, pas de bypass, pas de "TEMPORARY", pas de
> hardcode pour faire avancer un état. Le DSP exécute le vrai ROM, la BSP
> est gated par TPU→TSP→IOTA, le mobile passe par la PTY QEMU. Si une
> instruction/opcode/registre semble cassé : **vérifier contre `tic54x-opc.c`
> et SPRU172C avant de patcher**, jamais contourner. Tout contournement
> temporaire jugé inévitable doit être documenté dans `doc/TODO.md`
> avec un critère de retrait.
>
> **Disclosure obligatoire.** Si malgré tout un hack/synth/bypass est
> introduit (même temporaire, même env-gated), écrire EXACTEMENT en
> tête de la réponse utilisateur :
>
> `ATTENTION HACK HACK HACK !!!!!!!!`
>
> Puis nommer le hack, son site (file:line), et le critère de retrait.
> L'absence de ce préambule = manquement à la règle.

## Architecture

Dual-core GSM baseband emulator:
- **ARM7TDMI** runs osmocom-bb `layer1.highram.elf` firmware
- **TMS320C54x DSP** runs real Calypso ROM (`calypso_dsp.txt`)
- **API RAM** shared memory at DSP 0x0800 (ARM 0xFFD00000), 8K words
- **BSP** receives I/Q via UDP 6702, serves to DSP via PORTR PA=0xF430 (opcode 0x74 ; 0x0034 = alias legacy accepté) — cf. `calypso_c54x.c:7508`
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
- a_cd[15]: NDB + 0x1FC = 0x03A4  (offsetof DWARF autoritaire, `calypso_dsp_shunt.c:94` `NDB_A_CD 0x1FC`). ~~NDB + 0x1DC = 0x0384~~ — FAUX : le « FIX 2026-05-28 0x1FC→0x1DC » a été reverté le 2026-06-02 (0x1DC écrivait a_cd à côté → num_biterr=0xff + CRC fail). Note drift code↔code : le watch de `calypso_c54x.c:2731` utilise encore 0x1F8/0x03A0 (à réaligner sur 0x1FC).

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
| PORTR PA | 0x0034 (alias legacy) | 0xF430 (vrai port RX BSP, 64 sites PROM0) | ~~DSP read wrong BSP port~~ — INVERSÉ : 0xF430 est le VRAI port. Handler déplacé à l'opcode 0x74 (`calypso_c54x.c:7508`) ; l'ancien chemin 0x8F est dead-code `if(0)` (`calypso_c54x.c:8603`). |
| 0xF4EB | — | RETE (correct) | The REAL rete opcode |
| 0xF9xx | BC branch (no push) | CC conditional call (push) | Lost return addresses |
| 0xFBxx | LD #k,16,A | CCD conditional call delayed (push) | Lost return addresses |
| NORM L799 | NOP (dead) | — removed, real NORM at L832 | FB correlator broken |

## Current Bug

**UL RACH / IMM_ASS chain — Location Update stalled.**

DL was validated end-to-end up to 2026-05-07 via the
`rsl_si_tap.py + /dev/shm mmap + CALYPSO_BCCH_INJECT` shortcut, but the
shortcut was a hack: it bypassed the DSP CCCH demod path and let the
mobile camp even after BTS death (mmap persistence). It was removed on
2026-05-08. The legitimate path (BTS → bridge UDP relay → QEMU BSP DMA
→ DSP CCCH demod → a_cd[] → ARM L1 → L1CTL_DATA_IND) currently does
not converge on bridge-fed GMSK in QEMU — fixing that is the new
prerequisite for any L3 progress.

The mobile then issues `RR_EST_REQ` (Location Update) and sends RACH
bursts (`ra 0x01`, `ra 0x05`, retries 8→7) but never receives an
`IMM_ASS_CMD`. Two failure modes are not yet discriminated:
  (a) UL: RACH burst is generated by ARM/DSP but doesn't reach
      `osmo-bts-trx` decode (DSP TX buffer wrong addr, BSP UL UDP path
      broken, bridge UL forwarding broken).
  (b) DL AGCH: BTS sends IMM_ASS but mobile DSP misses the AGCH
      sub-slot (BCCH passes because it's repetitive; AGCH is
      event-driven and a single miss is fatal).

**Test discriminant (not yet run):** tcpdump GSMTAP during a session.
IMM_ASS visible on air → (b). Absent + osmo-bts-trx RACH counter at 0
→ (a).

### UL pipeline as of 2026-05-07

`calypso_trx.c` polls all three UL task fields:
  - `d_task_u`  (write-page word 2) — generic SDCCH/SACCH/FACCH/TCH NB
  - `d_task_ra` (write-page word 7) — RACH access burst
  - `d_burst_u` (write-page word 3) — TN selector

RACH path uses `gsm0503_rach_ext_encode` (libosmocoding) reading
`d_rach` from NDB. **MAJ 2026-07-01 :** l'encodage RACH vit désormais
dans le chemin UL BSP (`calypso_bsp.c:1550`
`gsm0503_rach_ext_encode(bits, ra, uic_or_bsic, false)`) ; `calypso_trx.c:1888`
ne fait plus que dispatcher (commentaire « Burst encoding … belongs in the BSP »).
The d_rach offset defaults to word 0x01CB from API
base (DSP==33 layout walk); override via `CALYPSO_NDB_D_RACH_OFFSET=0xNNN`
if the firmware uses a different DSP version.

NB UL still reads encoded bursts from DSP DARAM 0x0900 (candidate
location — needs verification by tracing DSP encoder writes during a
real SDCCH UL).

### Removed in 2026-05-07 cleanup (no more hacks)

- BOURRIN-FBDET-SKIP block in `c54x_exec_one` (range 0x8d00..0x8f80
  pop-and-jump). DSP now runs the full fb-det routine — performance
  cost mitigated by `-icount` on the QEMU command line.
- DIAG-HACK env-gated INTM force-clear + ALIAS-CHECK dump in
  `c54x_run_until_idle_or_n`.
- `publish_fb_found` / `publish_sb_found` synthetic NDB writes in
  `calypso_fbsb_on_dsp_task_change`. DSP demod runs for real on the
  GMSK-modulated I/Q the BSP feeds from `osmo-bts-trx`.
- `si3_fallback[]` hardcoded BCCH SI3 in `calypso_fbsb_on_dsp_task_change`.
- `allc_burst_idx` static cycle 0..3 counter — replaced by
  `burst_d = fn & 3` (FN-derived, lockstep-safe).

### Removed in 2026-05-08 cleanup (hack purge)

- `scripts/rsl_si_tap.py` deleted entirely. It sniffed the BSC↔BTS RSL
  TCP stream, parsed BCCH_INFO messages, and wrote `/dev/shm/calypso_si.bin`
  with the raw SI bytes.
- `CALYPSO_BCCH_INJECT` env var + the `csi_init_once` / `csi_lookup_for_tc`
  / `bcch_inject_consume` block in `calypso_fbsb.c` deleted. They read
  the mmap and wrote `a_cd[]` directly in NDB during DSP_TASK_ALLC,
  bypassing the real DSP CCCH demod.
- `CALYPSO_SI_MMAP_PATH` env var deleted (no consumer left).
- `doc/MMAP_SI_FORMAT.md` is now historical (kept for reference but not
  applicable to any live path).
- `run_si.sh` no longer launches `rsl_si_tap.py` and clears the legacy
  `/dev/shm/calypso_si.bin` from prior runs at startup.

Why removed : the mmap survived BTS death, so the mobile kept camping on
a stale cache even after the BTS process exited. The "DL works" claim
was therefore not honest — it worked off cached SI bytes, not off live
BTS broadcast. Removing the shortcut forces the DSP CCCH demod path to
be the only data flow, which is currently broken on bridge-fed GMSK
samples (= the new top blocker).

### Stability config

`run.sh` launches QEMU with `-icount shift=auto,align=off,sleep=off` for
deterministic virtual time and calypso-ipc-device with `(removed)=1`
for QEMU-driven CLK IND. The pair eliminates host-load jitter that was
producing 28% LOST timer events.

### Env-gated dev assists

| Env | Effect |
|---|---|
| `CALYPSO_FBSB_SYNTH=1` | Synth FB/SB publish in `on_dsp_task_change` (default OFF) |
| `CALYPSO_W1C_LATCH=1` | W1C latch on a_sync_demod cells (default OFF) |
| `CALYPSO_NDB_D_RACH_OFFSET=0xNNN` | Override d_rach word index (default 0x01CB) |
| `CALYPSO_RACH_FORCE_BSIC=N` | Force BSIC in RACH encoder to N (0..63), overriding d_rach byte. Match `osmo-bsc.cfg base_station_id_code` |
| `(removed)=1` | CLK IND from QEMU FN (default OFF = wall-clock) |
| `CALYPSO_ICOUNT=auto/off/shift=N` | QEMU icount mode (default `auto`). Kick timer is on VIRTUAL clock so icount doesn't freeze TDMA. |

As of 2026-05-08, no env-gated dev-assist can deliver SIs to mobile L3
anymore. The legitimate path (BTS → bridge → BSP → DSP CCCH demod →
a_cd[]) is the only one wired. It currently does not converge on the
bridge-fed GMSK samples — mobile stays in cell-search until the demod
is fixed.

### Run config

```bash
# Mobile config must have `stick <arfcn>` in `ms 1` block, otherwise
# mobile abandons FBSB after 2 retries → d_task_md stays at 1.
# 2026-05-28 : default DARAM addr fixe a 0x2a00 (via canary discovery, cf
# doc/BOOT_TO_FBSB_SEQUENCE.md + commentaire dans calypso_bsp_init). L'ancien
# 0x3fb0 etait FAUX (zone unmapped). Override via CALYPSO_BSP_DARAM_ADDR si besoin.
./run.sh
# Le default 0x2a00 couvre la zone DSP-read 0x2a00-0x2b5e (= BK=0x015e=350 mots,
# = burst GSM oversample size). Confirme via canary 0xCAFE injecte → DSP lu
# par PC=0x93a5 AR3=0x2a00+offset.
```

### Old blockers (⚠️ PAS résolu — voir correction)

**INTM=1 forever / DSP INT3 never serviced** — was the 2026-04 blocker.
~~Resolved naturally once DSP frame interrupt wiring + BSP RX delivery
converged.~~ — **FAUX (audit 2026-07-01).** Le wiring N'A PAS convergé.
Vérité-terrain : `IMR=0x0000` tout le run (jamais ré-armé après le clear
boot @0xb37e), le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000). L'IT frame
du modèle tape sur vec19/bit3 = stub RETE ; le vrai scheduler per-frame
est vec28/bit12 mais n'est jamais vectorisé sans le levier expérimental
`CALYPSO_DSP_FRAME_VEC28` — et ce levier est un **CUL-DE-SAC** : il force la
vectorisation mais atterrit dans l'épilogue ISR → dérail vers le boot-stub,
IMR reste 0x0000 (voir `calypso_c54x.c:13537`). Le vrai trou est le handshake
go-live ARM→DSP : `api_write_cb` est déclaré (`calypso_c54x.h`) mais JAMAIS
câblé (`grep 'api_write_cb *=' = 0`), et l'ARM n'écrit que 0x0000 dans API
0x0314/0x0318. Le bypass diagnostic INTM (commit 306d6ec, reverté f0dec53)
reste à proscrire (corruption NDB).

## Old bugs (resolved)

**SP slow leak:** SP descends ~3 words per IDLE cycle (5AC7 → 5AC4). Introduced by F9xx CC fix (push now correct). Likely some CC calls in ROM where the callee doesn't RET properly — need to trace which CC target doesn't return.

**27 duplicate opcode handlers** in c54x_exec_one — consolidated in
a72266d (-150 lines, no behavior change).

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

- `hw/arm/calypso/calypso_c54x.c` — DSP emulator (~13697 lines, opcode switch)
- `hw/arm/calypso/calypso_trx.c` — TRX/TPU/TSP/ULPD/SIM + TDMA tick + DMA
- `hw/arm/calypso/calypso_bsp.c` — BSP DMA + PORTR buffer + UDP 6702
- `hw/arm/calypso/calypso_iota.c` — IOTA BDLENA gate
- `hw/arm/calypso/l1ctl_sock.c` — L1CTL unix socket (/tmp/osmocom_l2)
- `hw/arm/calypso/sercomm_gate.c` — Sercomm DLCI router (PTY → FIFO)
- `hw/intc/calypso_inth.c` — INTH interrupt controller
- `hw/char/calypso_uart.c` — UART with RX FIFO + sercomm
- `calypso-ipc-device` — BTS UDP bridge (clock-slave)
- `run.sh` — Orchestrated launch (QEMU → bridge → BTS → mobile)

## DIAG 2026-06-02 (pour Claude web) — Bug #3 décodeur : 0x86/0x87 MVDM→STH

### Ce que le diag `run_diag_debug_all.sh` a prouvé (combo full/mobile/c54x)
- **Localisation = 100% dans le décodeur c54x du corrélateur** (gagnée par
  élimination, pas raccourci) : cohérence ARM↔DSP prouvée (sentinelle 0xDEAD),
  consommation ARM OK, livraison BSP byte-identique, AFC `hz=0.0` stable.
  `I/Q VARIE` (|max|=23340, 6 val/61 addr = vraie forme d'onde) → corrélateur
  aval suspecté, PAS l'input. `rxlev<=-110` est un leurre (reporting PM = aval
  du corrélateur cassé).
- **Symptôme dominant = runaway SP** : SP-LEDGER passe **0x0097 → 0xcade** entre
  insn 262.8M et 282.8M puis **gèle** (pushes/pops/irq figés 379/2681/1048).
  pops>pushes mais net_words explose → **une instruction écrit SP=0xcade**
  (pas un déséquilibre push/pop) = désync de flux, pas une fuite.
- **AR3=0000 figé** dans IQ-READ (`PC=0x7e6f`, op voisin `0x8693 @0x7e71`).
- Contexte : le wedge arrive **juste après** dispatch FB (`task_md=5`,
  `on_dsp_task_change task=5`) → le pipeline FB est enfin armé, mais wedge avant
  d'écrire `d_fb_det` (ARM lit `d_fb_det=0x0000` en boucle).

### Root cause (vérifié ISA, règle #1)
`0x8693 @0x7e71` = **`STH A, ASM, *AR3+`** (1 mot, AR3 post-incr) per
`doc/opcodes/tic54x_hi8_map.md` L95 (`sth 0x8600/0xFE00`). L'ému le décodait en
**MVDM/MVMD 2-mots** (`hi8==0x86/0x87`, marqué « TODO swap » par l'auteur) :
1. **longueur 2 au lieu de 1** → bouffe l'opcode suivant → **désync cascade**
   (= SP→0xcade) ;
2. n'incrémente jamais l'AR du Smem → **AR3 figé/0** dans la boucle FB.
Le « op=0000 » qu'on cherchait avant était une trace **pré-fix** (XPC=3 fetchait
prog[0x3ee00] vide) — clos par le fix `c54x_prog_xlate` (0xE000+ ignore XPC) +
PDROM chargé. Pas un bug de sonde : `op=prog_fetch(s,s->pc)` disait vrai.

### Fix (commit en cours)
`c54x_exec_one` : `0x86/0x87` → `STH src,ASM,Smem` 1-mot, mirror exact du
handler `0x84` (STL A,ASM,Smem) déjà validé, store HIGH word, `src=hi8&1`.
**Orthogonal au revert 0x72/0x73** (`REVERT_MVMD_KNOWLEDGE.md`) : l'assembleur
TI encode MVDM=0x72/MVMD=0x73, JAMAIS à 0x86/0x87 → aucun 0x86xx ROM n'est une
vraie MVDM.

### Proof-gate (à mesurer cross-run, runtime non-déterministe)
1. **AR3 BALAYE** le buffer I/Q dans IQ-READ (0x2a00→0x2a.. incrémente, plus figé/0).
2. SP-LEDGER ne slamme plus à 0xcade (pas de gel post-262M).
3. corrélateur complète une passe → **`d_fb_det` bit15 armé sur input réel**
   (PAS forcé), propagé à l'ARM → `Channel synched` BSIC/SNR réels.

### Prochain étage = audit différentiel décodeur (au lieu de peler bug par bug)
Même classe que SACCD / XPC-paging / AR3-zero : trous **longueur/mode/mnémo**.
TODO connus dans `tic54x_hi8_map.md` (col. misclassification) : 0x85 (MVPD→STL B),
0x8B (POPD), 0xDD/0xDE/0xCD/0xC5/0xCE (parallel-store mal décodés), 0xAA-AB.
Pas de désassembleur `tic54x` dispo (binutils host sans la cible) → audit
table-driven : logger `(PC, op, consumed)` sur l'overlay 0x8000-0x9fff,
diff la colonne **longueur** vs map authoritative. Priorité longueur (cascade).

## DIAG 2026-06-02 SOIR (bilan session) — cause (c) localisée, MVDM bloqué

Chaîne **mesurée saine boundary-par-boundary** (BTS→ipc→bridge→BSP→DARAM→DSP→ARM→L2).
Le mur n'est PAS une intersection — c'est UNE boîte : la décision FB-det dans le DSP.

**Fixes validés gardés :** 0x86/0x87=STH (SP wedge mort, AR3 balaie), 0x8E/0x8F=CMPS
(nécessaire pas suffisant), sonde `DRAIN-CB delivered=` (vrai compteur `bursts_written`
vs `bursts_seen` MORT = red herring). Outils : DECODE-AUDIT (CALYPSO_DA_LO/HI/INSN),
token FBDET.

**Cause (c) prouvée :** DECODE-AUDIT gaté insn>250M → `PC=0xf564 op=0x7215 MVDM`
dans la boucle dispatch FB décodé 1-mot (0x72/0x73 retirés par revert) → opérande
exécutée comme opcode → desync → AR5 (ptr handler) jamais chargé → tâche FB jamais
dispatchée → `0x9ac0` jamais ré-atteint past-boot (DETECTOR-RUN 5× boot) → ANGLE-WR=0
(SNR/PM jamais écrits) → d_fb_det jamais armé.

**MVDM 2-mots ISA-correct RÉGRESSE (gardé reverté) :** avance dispatch à PC=0xee38
(task=5 sur 2 pages) mais **AR3 hors-buffer 0x2b97>0x2b28** → A=0 → delivered=0 →
INT3 mort (irq 3860→4) = deadlock (bug compensateur upstream, REVERT_MVMD_KNOWLEDGE.md).

**NEXT LEAD :** d'où vient AR3=0x2b97 (hors-buffer) à 0xee38 ? Le fixer débloque le
MVDM ISA → (c) résolu. Garder 0x86/87+CMPS, ré-appliquer 0x72/0x73, tracer le setup AR3.
**AFC figée = esclave de la FB-det, PAS la cause** (FB établit la réf fréquence).
