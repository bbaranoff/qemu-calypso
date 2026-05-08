# qemu-calypso

QEMU emulation of the **TI Calypso GSM baseband** chipset (ARM7TDMI + TMS320C54x DSP),
the SoC used in the OpenMoko Neo / Compal e88 family. Runs unmodified real DSP ROM and
the [osmocom-bb](https://osmocom.org/projects/baseband/) `layer1.highram.elf` firmware
on the ARM side, with a Python bridge that connects to `osmo-bts-trx` for full
GSM cell simulation.

## Latest update — Session 2026-05-08 (POPM fix + opcode audit)

**INTM=1 dwell perpétuel résolu. Blocker déplacé sur l'init AR du correlator FB-det.**

L'émulateur DSP avait 9 opcodes misclassifiés (depuis le début du projet). Le plus
critique : **`0x8A00` était décodé en MVDK Smem,dmad alors que tic54x-opc.c
l'identifie comme POPM MMR**. Conséquence : le pattern PSHM/POPM symétrique du
firmware DSP (sauve ST1, entre zone critique avec INTM=1, restore ST1) ne
fonctionnait jamais — ST1 jamais restauré → INTM stuck à 1 → IRQ vectoring bloqué
→ DSP mort dans une wait à `0xa21x` après ~98M insn. Symptôme persistant depuis
avril 2026.

### Audit opcode

Référence créée : [`hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md`](hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md)
— table complète tic54x officielle (binutils 2.21.1) hi8 → mnémonique pour les 256 valeurs.

| Opcode | qemu-calypso (avant) | tic54x officiel | Statut 2026-05-08 |
|---|---|---|---|
| `0x8A` | MVDK Smem,dmad (2-mot) | **popm MMR** (1-mot) | **fixé** |
| `0x8B` | MVDK long-addr (2-mot) | popd Smem (1-mot) | stubé NOP |
| `0xAA/AB` | STLM duplicate | ld variant | stubé NOP |
| `0xC5` | PSHM MMR | st parallel | stubé NOP (sp-- fantôme) |
| `0xCD` | POPM MMR | st parallel | stubé NOP (sp++ fantôme) |
| `0xCE` | FRAME #k | st parallel | stubé NOP (sp+=k arbitraire) |
| `0xDD` | POPD Smem | st parallel | stubé NOP (sp++ fantôme — **cause SP runaway** post-POPM) |
| `0xDE` | POPD dmad (2-mot) | st parallel | stubé NOP |
| `0x80` | MVDD Smem,Smem | stl src,Smem | stubé NOP |

Stratégie « stop-the-bleed » : les opcodes mal classifiés qui causaient des
push/pop fantômes ou écritures mémoire pourries sont neutralisés en NOP 1-mot.
Implémentation sémantique correcte des familles ST||OP parallèles (0xC0..0xDF)
reportée — non bloquant tant que le SP est stable.

### Symptômes débloqués

| Avant | Après |
|---|---|
| INTM=1 perpétuel après insn=90.2M | INTM=0 après chaque coroutine swap ✓ |
| WAIT-A21A : 5.7 millions d'iters bloquées | 0 ✓ |
| ENTER-7740 : 37k figé | 0 (path différent) |
| DSP throughput | **5× plus rapide** (4.3B insn / 44s) |
| RETE count | toujours 0 (pending IRQ replay non déclenché) |
| `fb0_att` / `L1CTL_DATA_IND` | toujours 0 (correlator lit zone vide) |

### Blocker actuel — D_FB_DET-WR-SITE

Probe instrumenté à `PC=0x8f51` (= PC qui écrit `d_fb_det`). 50 captures révèlent
les pointers AR au moment du write :

```
#1   AR1=001c AR2=0000 AR3=0000 AR4=2bc0 AR7=0000  data[AR1]=bbef BK=00b0
#50  AR1=004a AR2=fc5d AR3=03a3 AR4=2bc3 AR7=fc5d  data[AR1]=0000 BK=00b0
```

- AR3 monte 0x0000 → 0x03A3 par stride **+19** → lit DARAM linéaire `[0..0x3A3]`
- AR2/AR7 wrappent `[0xfc5d..0xffed]` par stride **−19**, BK=176 (circulaire)
- AR4 ≈ 0x2bc0 quasi-fixe → table coefficients ROM
- **Aucun AR n'atteint la zone BSP DMA target.**

Tests A/B avec `CALYPSO_BSP_DARAM_ADDR` (`0x3fb0` / `0x2bc0` / `0x0080`) →
D_FB_DET-WR-SITE **bit-pour-bit identique**. Le correlator FB-det ignore où le
BSP livre. L'init AR du firmware impose les pointers internes — le mismatch est
structurel.

### Mobile L23

| Mode | État |
|---|---|
| `CALYPSO_FBSB_SYNTH=0` | mobile bloqué pré-FBSB (firmware n'émet pas FBSB_CONF) |
| `CALYPSO_FBSB_SYNTH=1` | mobile passe FBSB → atteint **gsm322 cell selection (DSC=90)** → bloqué (pas de BCCH décodé) |

## Quick start

```bash
# Real DSP path. Mobile reste pré-FBSB sans synth.
./run.sh

# Synth FB/SB pour faire camper le mobile en gsm322
CALYPSO_FBSB_SYNTH=1 ./run.sh
```

### Variables d'environnement

| Env | Default | Effet |
|---|---|---|
| `CALYPSO_FBSB_SYNTH` | `0` | `1` publie FB/SB synthétique côté NDB pour faire passer le mobile FBSB → gsm322. Fallback dev tant que le correlator DSP réel ne converge pas. |
| `CALYPSO_BSP_DARAM_ADDR` | `0x3fb0` | Adresse DARAM cible des DMA RX BSP. **Sans effet sur le correlator FB-det actuel** (AR init firmware-dependent). |
| `CALYPSO_W1C_LATCH` | `0` | `1` active le latch capture sur les cellules `a_sync_demod` (race DSP-write/ARM-read mitigation). |
| `CALYPSO_NDB_D_RACH_OFFSET` | `0x01CB` | Override word index de `d_rach` dans NDB (DSP version-dependent). |
| `CALYPSO_RACH_FORCE_BSIC` | unset | Force la BSIC dans l'encoder RACH (0..63). Match avec `osmo-bsc.cfg base_station_id_code`. |
| `CALYPSO_DSP_IDLE_FF` | `1` | Fast-forward DSP idle dispatcher. Optimisation purement host-side, pas un hack. Set `0` pour debug DSP boot complet. |
| `CALYPSO_DSP_IDLE_RANGE` | `e9ac:e9b7` | Range PC du dispatcher idle skipée par le fast-forward. |
| `CALYPSO_DSP_FBDET_SKIP` | `0` | (option de diag) Force-skip du fb-det inner loop. |
| `CALYPSO_ICOUNT` | `auto` | Mode `-icount` QEMU. `auto` = `shift=auto,sleep=on,align=off`. Le kick timer TDMA est sur `QEMU_CLOCK_VIRTUAL` donc icount ne fige pas la TDMA tick. |
| `BRIDGE_CLK_FROM_QEMU` | `0` | `1` → CLK IND piloté par FN QEMU. À pairer avec `-icount`. Default = wall-clock (BTS happy). |
| `BRIDGE_DL_FN_REWRITE` | `slot` | Réécriture FN downlink slot-aware (DL FN→qFN cohérent avec TPU). `naive` ou `off` pour debug. |
| `BRIDGE_DL_FN_LOOKAHEAD` | `32` | Marge demi-fenêtre BSP_FN_MATCH_WINDOW pour DL FN rewrite. |
| `BRIDGE_UL_FN_REWRITE` | `slot` | Idem pour UL. |
| `CALYPSO_DSP_ROM` | `calypso_dsp.txt` | Path du dump DSP ROM. |
| `CALYPSO_SIM_CFG` | `~/.osmocom/bb/sim.cfg` | Config SIM IMSI/Ki. |
| `L1CTL_SOCK` | `/tmp/osmocom_l2` | Mobile↔QEMU L1CTL Unix socket. |

## Probes runtime instrumentés

Le binaire embarque 9 probes activables via stderr du QEMU. Tous présents dans
`hw/arm/calypso/calypso_c54x.c` :

| Tag | Cible | Usage |
|-----|-------|-------|
| `PC-HIST-3FB` | reads addr ∈ `[0x3fb0..0x3fbf]` | Top PCs lecteurs zone BSP target |
| `PC-HIST-3DD` | reads addr ∈ `[0x3dcf..0x3dd5]` | Top PCs zone scratch dominante |
| `WATCH-WRITE 0x3dd2` | writes à 0x3dd2 | Identifie writers + valeurs |
| `INTM-TRANS` | transitions INTM 0↔1 | Cause du SSBX/RSBX/STM ST1 |
| `WAIT-A21A` | PC=0xa21a | Snapshot INTM/IMR/IFR/ST0/ST1/SP |
| `ENTER-7740` | PC=0x7740 | Caller chain + AR + insn |
| `ST1-WR` | STM #lk, ST1 (op 0x7707) | Toutes écritures de ST1 |
| `POST-BOOTSTUB-RET` | RET depuis PC ≤ 0x0008 | Task PC poppé après boot stub |
| `D_FB_DET-WR-SITE` | PC=0x8f51 | AR0..AR7 + data[AR0/1/2] + BK + A |

## Repository layout

```
qemu-calypso/                         ← snapshot Calypso-specific
├── hw/arm/calypso/                   ← SoC + DSP emulator
│   ├── calypso_c54x.c                ← C54x DSP (~6000 lignes)
│   ├── calypso_trx.c                 ← TRX/TPU/TSP/TDMA
│   ├── calypso_bsp.c                 ← BSP DMA + UDP 6702
│   ├── calypso_iota.c                ← IOTA BDLENA gating
│   ├── calypso_fbsb.c                ← FB/SB helper côté ARM
│   ├── calypso_sim.c                 ← SIM ISO 7816
│   ├── l1ctl_sock.c                  ← L1CTL Unix socket
│   ├── sercomm_gate.c                ← Sercomm DLCI router
│   └── doc/
│       ├── PROJECT_STATUS.md         ← état détaillé du projet
│       ├── TODO.md                   ← prochaines actions
│       ├── opcodes/
│       │   └── tic54x_hi8_map.md     ← référence tic54x complète (NEW 2026-05-08)
│       └── spru172c.pdf              ← TI C54x manual
├── hw/{intc,char,timer,ssi}/         ← peripherals
├── include/                          ← headers
├── bridge.py                         ← BTS UDP ↔ BSP relay
├── run.sh                            ← launch orchestration
├── calypso_dsp.txt                   ← DSP ROM dump (132K words)
├── calypso.md                        ← Architecture pipeline + diagrams
└── CLAUDE.md                         ← AI-assistant context
```

L'arbre de dev actif vit séparément (`/home/nirvana/qemu-src/`). Sync via `cp` host
+ `docker cp` container ; vérifier md5 après chaque sync.

## Architecture

Dual-core GSM baseband emulator :

- **ARM7TDMI** runs `osmocom-bb` Layer 1 firmware (`layer1.highram.elf`)
- **TMS320C54x DSP** runs the real Calypso DSP ROM (`calypso_dsp.txt`)
- **API RAM** shared memory at DSP `0x0800` / ARM `0xFFD00000`, 8K words
- **BSP** receives I/Q bursts via UDP 6702, serves DSP via PORTR PA=0x0034
- **TPU → TSP → IOTA** chain gates BDLENA for RX windows
- **Bridge** (Python) relays BTS UDP 5700-5702 ↔ BSP, clock-slave of QEMU
- **osmo-bts-trx** acts as the cell, paired with `osmo-stp/hlr/msc/bsc/...`
- **mobile** (osmocom L23) talks L1CTL via `/tmp/osmocom_l2` socket through `osmocon`

Pipeline détaillé + sequence diagrams dans [`calypso.md`](calypso.md).

### Memory map (DSP side)

| Range | Type | Content |
|-------|------|---------|
| 0x0000-0x007F | Boot ROM stubs | LDMM SP,B + RET at 0x0000, NOP elsewhere |
| 0x0080-0x27FF | DARAM overlay (OVLY) | Code + data, loaded by MVPD at boot |
| 0x2800-0x6FFF | Unmapped | Reads as 0x0000 |
| 0x7000-0xDFFF | PROM0 | DSP ROM (always readable) |
| 0xE000-0xFF7F | PROM1 mirror | Mirrored from page 1 (0x18000+) |
| 0xFF80-0xFFFF | Interrupt vectors | From PROM1, IPTR=0x1FF |
| 0x0800-0x27FF | API RAM | Shared with ARM (NDB, write/read pages) |

### Interrupt vectors (IPTR=0x1FF → base 0xFF80)

`vec = imr_bit + 16`. `addr = 0xFF80 + vec * 4`
- INT3 (frame): vec 19, IMR bit 3 → 0xFFCC
- TINT0: vec 20, IMR bit 4 → 0xFFD0
- BRINT0 (BSP): vec 21, IMR bit 5 → 0xFFD4

## Conventions du projet

- **No stubs / no hacks** dans les paths critiques. Le DSP exécute le vrai ROM,
  la BSP est gated par TPU→TSP→IOTA, le mobile passe par la PTY QEMU.
- **Vérifier les opcodes** contre `tic54x-opc.c` (binutils) AVANT de patcher.
  Voir [`doc/opcodes/tic54x_hi8_map.md`](hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md).
- **QEMU = clock master**, le bridge est slave, le BTS reçoit CLK IND wall-paced.
- **Test après chaque édit** — build dans Docker, vérifier DSP IDLE + SP + IMR + RETE count.
- Tout contournement temporaire jugé inévitable doit être documenté dans
  [`hw/arm/calypso/doc/TODO.md`](hw/arm/calypso/doc/TODO.md) avec critère de retrait.

## Build

```bash
docker exec CONTAINER bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"
```

Workaround link `-lm` cassé :

```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh && bash /tmp/link.sh
```

Le container `bastienbaranoff/free-bb:latest` a un environnement pré-built avec
toute la chaîne d'outils GSM (osmocom-bb, osmo-bts-trx, osmo-msc/bsc/hlr/mgw,
osmocon, mobile) et le build QEMU à `/opt/GSM/qemu-src/build/`.

## Historique

### 2026-05-08 — POPM fix + audit opcode (cette session)

- POPM (0x8A) fixé : INTM dwell perpétuel résolu
- 8 stubs NOP sur opcodes misclassifiés (stop-the-bleed)
- `tic54x_hi8_map.md` créé (référence complète)
- DSP throughput ×5
- Blocker isolé : correlator FB-det lit DARAM internes indépendamment du BSP target

### 2026-05-07 — Hacks purgés

- `rsl_si_tap.py` + `CALYPSO_BCCH_INJECT` + `CALYPSO_SI_MMAP_PATH` supprimés
- `BOURRIN-FBDET-SKIP` block supprimé
- DIAG-HACK INTM force-clear supprimé
- `si3_fallback[]` hardcode supprimé
- `allc_burst_idx` static cycle remplacé par `fn & 3`

### 2026-04-29 — Opcode dispatch baseline

5 fixes structurels validés empiriquement, ~2530 sites firmware débloqués :

| # | Fix | Impact |
|---|---|---|
| 1 | Reset silicon-aligné (PMST=0xFFA8, ST0=0x181F, ST1=0x2900) | DSP entre PROM1 init zone |
| 2 | 0x6F00 ext dispatch | Wedge PC=0x8353 (2.2G iter) éliminé |
| 3 | 0x68-0x6E handlers (ANDM/ORM/XORM/ADDM/BANZ/BANZD) | 1563 sites unblocked |
| 4 | APTS misnomer fix (PMST bit 4 = AVIS, sans stack semantics) | Stack leak 1.96M events → 0 |
| 5 | F3xx complet (AND/OR/XOR/SFTL + #lk variants) | 364 sites, wedge PC=0x8eb9 |

### Sessions précédentes

Voir `hw/arm/calypso/doc/SESSION_*.md` et `/root/.claude/projects/-home-nirvana/memory/`.

## License & attribution

- **QEMU base** : GPL-2.0-or-later (upstream QEMU)
- **Calypso emulator additions** : GPL-2.0-or-later
- **osmocom-bb firmware** : GPL (utilisé tel quel, non redistribué ici)
- **Calypso DSP ROM** (`calypso_dsp.txt`) : TI proprietary, dump physique d'un
  device pour recherche et interopérabilité (osmocom DSP dumper). Non redistribuable
  commercialement sans autorisation TI.

## Related projects

- [osmocom-bb](https://osmocom.org/projects/baseband/) — open-source GSM baseband stack
- [osmo-bts](https://osmocom.org/projects/osmobts/) — open-source GSM BTS
- [osmo-cn](https://osmocom.org/projects/cellular-infrastructure/) — open-source cellular core network
- [QEMU](https://www.qemu.org/) — generic emulation framework
- [tic54x binutils](https://sourceware.org/binutils/) — TMS320C54x assembler/disassembler reference
