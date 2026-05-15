# qemu-calypso — Synthèse

Émulation QEMU du chipset TI Calypso (ARM7 + DSP TMS320C54x) faisant tourner
le firmware **osmocom-bb `layer1.highram.elf` non patché**, connecté à un BTS
réel via le protocole TRX. L'objectif est qu'un mobile L23 (mobile / ccch_scan)
puisse camper sur une cellule simulée bout en bout.

---

## Vue d'ensemble

```
BTS (osmo-bts-trx)
   │ UDP : TRXC 5701, TRXD 5702, CLK 5700
   ▼
bridge.py
   │ PTY / UART (sercomm DLCI 4 = bursts, DLCI 5 = L1CTL)
   ▼
QEMU Calypso
   ├── ARM946 (firmware layer1.highram.elf)
   ├── DSP C54x (calypso_c54x.c, ~6000 lignes)
   ├── Peripherals (UART, INTH, SPI/TWL3025, SIM, TPU, TSP, ULPD, IOTA, BSP)
   ▼
Unix socket /tmp/osmocom_l2_1 (L1CTL length-prefix)
   ▼
mobile / ccch_scan (OsmocomBB host tools)
```

QEMU est **clock master** : il génère TINT0 toutes les 4.615 ms (une frame TDMA),
envoie un paquet FN au bridge, le bridge sert de relais wall-paced vers la BTS.

---

## Quick start

```bash
# Build
cd qemu && mkdir build && cd build
../configure --target-list=arm-softmmu && ninja

# Lancement (orchestré, tmux session "calypso")
docker exec osmo-operator-1 /opt/GSM/qemu-src/run.sh

# Variantes
CALYPSO_FBSB_SYNTH=1 ./run.sh                              # synth FB/SB pour débloquer mobile en gsm322
CALYPSO_ICOUNT=off CALYPSO_FBSB_SYNTH=1 ./run.sh           # bench déterministe
CALYPSO_FORCE_RX_DONE=1 CALYPSO_FBSB_SYNTH=1 ./run.sh      # workaround SIM busy-poll
```

Container `trying` (image `osmo-qemu`). Source `/opt/GSM/qemu-src/`,
build `cd /opt/GSM/qemu-src/build && ninja qemu-system-arm`.

---

## Architecture — composants

### Fichiers hardware QEMU

| Fichier | Rôle |
|---|---|
| `hw/arm/calypso/calypso_mb.c` | Machine board (CPU, RAM, Flash, boot) |
| `hw/arm/calypso/calypso_soc.c` | SoC, instancie les périphériques |
| `hw/arm/calypso/calypso_trx.c` | TRX bridge, DSP API, TPU, TSP, SIM, ULPD |
| `hw/arm/calypso/calypso_c54x.c` | Émulateur DSP TMS320C54x |
| `hw/arm/calypso/calypso_bsp.c` | BSP DMA + UDP 6702 |
| `hw/arm/calypso/calypso_iota.c` | IOTA BDLENA gating |
| `hw/arm/calypso/calypso_fbsb.c` | Helper FB/SB synth ARM-side |
| `hw/arm/calypso/calypso_sim.c` | SIM ISO 7816 |
| `hw/arm/calypso/calypso_tint0.c` | Master clock 4.615 ms |
| `hw/arm/calypso/sercomm_gate.c` | Sercomm DLCI router |
| `hw/arm/calypso/l1ctl_sock.c` | L1CTL Unix socket |
| `hw/char/calypso_uart.c` | UART 16550-like avec sercomm |
| `hw/intc/calypso_inth.c` | Interrupt controller level-sensitive |
| `hw/ssi/calypso_spi.c` | SPI + TWL3025 ABB |
| `hw/timer/calypso_timer.c` | Timers HW |

### Scripts & bridge

| Fichier | Rôle |
|---|---|
| `bridge.py` | Relay sercomm/PTY ↔ L1CTL unix + UDP TRX (CLK/TRXC/TRXD) + GMSK |
| `run.sh` | Orchestration : kill → QEMU → bridge → osmo-bts-trx → mobile |
| `calypso_dsp.txt` | DSP ROM dump (Motorola C1xx via osmocon + ESP32) |

---

## Memory map

### ARM (côté firmware)

| Adresse | Taille | Rôle |
|---|---|---|
| `0x00000000` | 4 MiB | Flash (pflash_cfi01, Intel 28F320J3) |
| `0x00800000` | 256 KiB | IRAM (aliasable via CNTL) |
| `0x01000000` | 8 MiB | XRAM |
| `0xFFD00000` | 64 KiB | DSP API RAM (partagée avec C54x) |
| `0xFFFE0000` | 0x100 | SIM controller |
| `0xFFFE3000` | 0x100 | SPI + TWL3025 ABB |
| `0xFFFE3800/3C00` | 0x100 | Timers 1/2 |
| `0xFFFF0000` | 0x2000 | DSP API RAM (mirror) |
| `0xFFFF1000` | 0x100 | TPU registers |
| `0xFFFF5000/5800` | 0x100 | UART IrDA / Modem |
| `0xFFFF9000` | 0x800 | TPU RAM |
| `0xFFFFA800` | 0x100 | ULPD |
| `0xFFFFB000` | 0x100 | TSP |
| `0xFFFFFA00` | 0x100 | INTH |

### DSP (côté C54x)

| Plage | Type | Contenu |
|---|---|---|
| `0x0000-0x007F` | Boot ROM stubs | `LDMM SP,B` + `RET` à 0x0000, NOP ailleurs |
| `0x0080-0x27FF` | DARAM overlay (OVLY) | Code + data, chargé par MVPD au boot |
| `0x0800-0x27FF` | API RAM shared | NDB, db_buf_w (ARM→DSP), db_buf_r (DSP→ARM) |
| `0x2800-0x6FFF` | Unmapped | Reads → 0x0000 |
| `0x7000-0xDFFF` | PROM0 | DSP ROM |
| `0xE000-0xFF7F` | PROM1 mirror | Mirror de la page 1 (0x18000+) |
| `0xFF80-0xFFFF` | Interrupt vectors | depuis PROM1, IPTR=0x1FF |

### DSP ROM sections (dans `calypso_dsp.txt`)

| Section | Plage | Words | Chargée dans |
|---|---|---|---|
| Registers | `0x00000-0x0005F` | 96 | `data[0x00-0x5F]` |
| DROM | `0x09000-0x0DFFF` | 20480 | `data[0x9000-0xDFFF]` |
| PDROM | `0x0E000-0x0FFFF` | 8192 | `data[0xE000-0xFFFF]` |
| PROM0 | `0x07000-0x0DFFF` | 28672 | `prog[0x7000-0xDFFF]` |
| PROM1 | `0x18000-0x1FFFF` | 32768 | `prog[0x18000-0x1FFFF]` + mirror `prog[0x8000-0xFFFF]` |
| PROM2 | `0x28000-0x2FFFF` | 32768 | `prog[0x28000-0x2FFFF]` |
| PROM3 | `0x38000-0x39FFF` | 8192 | `prog[0x38000-0x39FFF]` |

### Adresses DSP clés (API RAM / NDB)

| Adresse | Contenu |
|---|---|
| `0x08D4` | `d_dsp_page` (NDB offset 0) |
| `0x08D5` | `d_error_status` |
| `0x0800-0x0813` | Write page 0 |
| `0x0814-0x0827` | Write page 1 |
| `0x0828-0x083B` | Read page 0 |
| `0x083C-0x084F` | Read page 1 |
| `0x08F8` | `d_fb_det` (NDB+36 words = ARM 0xFFD001F0) |
| `0x09D0+` | `a_cd[]` (CCCH demod result buffer) |
| `0x3FC0-0x3FFF` | BSP DMA target (DARAM) |

### IRQ map (ARM)

| IRQ | Périphérique |
|---|---|
| 0 | Watchdog |
| 1 | Timer 1 |
| 2 | Timer 2 |
| 4 | TPU Frame |
| 5 | DSP API |
| 7 | UART Modem |
| 8 | SIM |
| 13 | SPI |
| 18 | UART IrDA |

### Vecteurs d'interruption DSP

`vec = imr_bit + 16`, `addr = 0xFF80 + vec × 4`.

| IRQ DSP | Vec | IMR bit | Adresse |
|---|---|---|---|
| INT3 (frame) | 19 | 3 | `0xFFCC` |
| TINT0 | 20 | 4 | `0xFFD0` |
| BRINT0 (BSP) | 21 | 5 | `0xFFD4` |

### Wake table

| Wake | Vec | IMR bit | Source | Gate |
|---|---|---|---|---|
| SINT17 | 19 | 3 | `calypso_trx.c` `calypso_tint0_do_tick` | `dsp_init_done && idle` |
| BRINT0 | 21 | 5 | `calypso_bsp.c` `bsp_rx_burst` | `dsp->idle && dsp_init_done` |
| TINT0 | 20 | 4 | masked by firmware | inactive |

---

## Pipeline GSM L1 — flux L1CTL

```
L23 → SOCK : RESET_REQ
SOCK → UART : sercomm DLCI5
UART → ARM : IRQ
ARM → SOCK : RESET_CONF → L23

L23 → ARM : PM_REQ
ARM → DSP : d_task_md=1 (PM)
DSP → ARM : a_pm[]
ARM → L23 : PM_CONF

L23 → ARM : FBSB_REQ
ARM → DSP : d_task_md=5 (FB)
BTS → BSP : DL burst (5702→6702)
BSP → DSP : DMA + BRINT0 (gated dsp_init_done)
DSP → ARM : d_fb_det=1
ARM → L23 : FBSB_CONF
```

Path UL : `DSP → BSP (DARAM 0x0900) → bridge → BTS (UDP 6802→5802)`.

### L1CTL message types (length-prefix BE16)

| Type | Nom |
|---|---|
| 0x01 | FBSB_REQ |
| 0x02 | FBSB_CONF (result 0=succès, 255=échec) |
| 0x03 | DATA_IND |
| 0x04 | RACH_REQ |
| 0x05 | DM_EST_REQ |
| 0x07 | RESET_IND (boot) |
| 0x08 | PM_REQ |
| 0x09 | PM_CONF |
| 0x0D | RESET_REQ |
| 0x0E | RESET_CONF |
| 0x10 | CCCH_MODE_REQ |
| 0x11 | CCCH_MODE_CONF |

### TRXD v0 (BTS ↔ bridge)

DL (BTS→MS) : `TN(1) FN(4 BE) RSSI(1) TOA(2 BE) soft_bits(148)`
UL (MS→BTS) : `TN(1) FN(4 BE) PWR(1) hard_bits(148)`

---

## Historique des bugs majeurs

### Session 2026-04-03 — 0xEA = LD #k9,DP

`0xEA` était décodé comme **BANZ** (faux). En réalité, BANZ = `0x6C/0x6E` ;
`0xEA00/0xFE00` = `LD #k9, DP`. 202 occurrences dans le ROM. Sans le fix,
le DSP branchait sur des adresses DARAM aléatoires. Après le fix : DSP exécute
197M instructions de boot (avant : 86K), DARAM correctement peuplée,
`task_md=5` (FB search) dispatché vers le DSP.

### Session 2026-04-04 — 22 bugs INTH/UART/DSP

- INTH **level-sensitive** (le bug le plus important — firmware ne fait
  jamais d'acknowledge IRQ_CTRL)
- IDLE wake sur n'importe quelle interruption
- UART address mapping, sercomm DLCI filter
- PROM1 mirror, OVLY range
- Bootloader protocol, F8xx branch, XPC dans `prog_fetch`
- 12 fix opcodes from SPRU172C

### Session 2026-04-05 night 4 — Audit opcode complet (17 bugs)

Audit massif de `calypso_c54x.c` contre `tic54x-opc.c` (binutils 2.21.1).

| Fix | Opcode | Avant | Après |
|---|---|---|---|
| 1 | F0xx | READA | ADD/SUB/LD/AND/OR/XOR #lk,shift,src,dst |
| 2 | F4Bx | NOP | RSBX ST0 |
| 3 | F5Bx | RPT #0xBx | SSBX ST0 |
| 4 | F6Bx | MVDD Xmem,Ymem | RSBX ST1 |
| 5 | F7Bx | LD #k8→AR7 | SSBX ST1 |
| 6 | FC00 | LD #k<<16,B | RET |
| 7 | FE00 | LD #k,B | RETD |
| 8 | F073 | RET | B pmad (2-mot) |
| 9 | F074 | RETE | CALL pmad (2-mot) |
| 10 | F072 | FRET | RPTB pmad |
| 11 | F070 | RET catch-all | RPT #lku |
| 12 | F071 | RET catch-all | RPTZ dst,#lku |
| 13 | F274 | CALLD push PC+2 | push PC+4 |
| 14 | F4E4 | IDLE | FRET |
| 15 | F4E1 | NOP | IDLE (le vrai) |
| 16 | F4E5 | NOP | FRETE |
| 17 | TINT0 | IFR bit 4 vec 20 | bit 3 vec 19 |

Résultat : DSP boot en 173 instructions, `IMR=0x002D` configurée, dispatch
loop active. Restait : SP drift, INTM=1 stuck (handler `0x7710` set INTM
mais ne clear pas).

### Session 2026-04-07 — Pipeline FBSB et wedge 0x76FE

- Module `calypso_fbsb.c/h` créé, branché dans `calypso_trx.c`
- Hook `on_dsp_task_change` publie `d_fb_det=1` + `a_sync_demod[]`
  dès que ARM écrit `d_task_md=FB_DSP_TASK`
- Offsets NDB validés (`d_fb_det = NDB+36 words = dsp_ram[0xF8]`)
- Découverte `CALYPSO_BSP_DARAM_ADDR=0x3FC0` : 99.7% des reads DSP dans
  cette zone, hardcodé par défaut
- **Blocage final identifié** : boucle MVDD `f6b9` à `PROM0 0x76FE` corrompt
  IMR à 0x0000. BRC=63, `ar1` finit par valoir 0 (= adresse MMR IMR),
  les writes MVDD écrasent IMR. Tous les interrupts DSP masqués → BRINT0
  ignoré → pas de FB_DET → FBSB échoue (`result=255`)
- Opcodes manquants identifiés sur le chemin critique FB-det :
  `MVPD 0x75xx`, F2xx `0xf210`, BC group 1 (cond=0x42/0x44/0x45/0x4d),
  `0xf58e` F5xx hors dispatch

### Session 2026-05-08 — POPM fix + audit hi8 complet

**INTM=1 dwell perpétuel résolu.** L'émulateur DSP avait 9 opcodes
misclassifiés depuis le début. Le critique : **`0x8A00` décodé en MVDK
Smem,dmad alors que tic54x-opc.c l'identifie comme POPM MMR**. Le pattern
PSHM/POPM symétrique du firmware (sauve ST1, entre zone critique avec
INTM=1, restore ST1) ne marchait jamais → ST1 jamais restauré → INTM stuck.

| Opcode | qemu-calypso (avant) | tic54x officiel | Statut |
|---|---|---|---|
| `0x8A` | MVDK Smem,dmad (2-mot) | **popm MMR** (1-mot) | **fixé** |
| `0x8B` | MVDK long-addr (2-mot) | popd Smem | stub NOP |
| `0xAA/AB` | STLM duplicate | ld variant | stub NOP |
| `0xC5` | PSHM MMR | st parallel | stub NOP (sp-- fantôme) |
| `0xCD` | POPM MMR | st parallel | stub NOP (sp++ fantôme) |
| `0xCE` | FRAME #k | st parallel | stub NOP |
| `0xDD` | POPD Smem | st parallel | stub NOP (sp++ fantôme = **SP runaway**) |
| `0xDE` | POPD dmad | st parallel | stub NOP |
| `0x80` | MVDD Smem,Smem | stl src,Smem | stub NOP |

Référence créée : `hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md` — table
complète hi8→mnémonique pour les 256 valeurs (binutils 2.21.1).

**Symptômes débloqués :**
- WAIT-A21A : 5.7M iters bloquées → 0
- ENTER-7740 : 37k figés → 0
- DSP throughput : **×5 plus rapide** (4.3B insn / 44s)

**Restant après cette session :**
- `fb0_att` / `L1CTL_DATA_IND` toujours 0
- Le correlator FB-det lit une zone vide : AR3 monte `0x0000→0x03A3`
  par stride +19, AR2/AR7 wrappent `[0xfc5d..0xffed]` stride −19 ;
  aucun AR n'atteint la zone BSP DMA target. Mismatch structurel
  entre init AR du firmware et adresse de livraison BSP.

---

## État actuel

### Ce qui marche
- Boot complet du firmware osmocom-bb non patché
- Console debug sercomm DLCI 10
- UART TX/RX level-sensitive
- SPI/ABB, SIM (injection ATR), DSP boot + re-boot
- TPU TDMA 4.615 ms, IRQ TPU_FRAME
- TRX bridge UDP compatible osmo-bts-trx
- `bridge.py` relais sercomm ↔ `/tmp/osmocom_l2`
- Mobile connecté, envoie `L1CTL_RESET_REQ`, reçoit `RESET_CONF`
- DSP atteint IDLE proprement (post-Session 2026-05-08)
- `task_md=24` (DSP_TASK_ALLC) fire correctement (CCCH demod dispatché)
- Sous `CALYPSO_FBSB_SYNTH=1` : mobile passe FBSB, atteint
  gsm322 cell selection (DSC=90)

### Ce qui ne marche pas encore
- `L1CTL_DATA_IND` = 0 (mur principal actuel)
- Vrai correlator FB-det : `fb0_att=0`, seuil de décision non franchi
- ARM `prim_rx_nb::l1s_nb_resp` invoqué mais `d_task_d` jamais set à 24
- Mobile L23 sous `FBSB_SYNTH=0` : bloqué pré-FBSB

### Blocage actuel — DATA_IND / `d_task_d`

```
DSP CCCH demod fires (task_md=24)              ✓  73× / run
DSP writes a_cd[0..14] result buffer           ✓  251 writes / run
ARM L1 prim_rx_nb::l1s_nb_resp invoqué         ✓  60+ calls / run
ARM reads dsp_api.db_r->d_task_d               ✓
                                                   ↓
                                            d_task_d == 0  →  puts("EMPTY")
                                                              return 0
```

Plusieurs PCs DSP (`0x787d`, `0x7a03`, `0x79f1`, `0x7817`) écrivent des
**valeurs garbage** (`0x8dd6`, `0xfef7`, …) à la cellule `d_task_d` (0x0828)
via ce qui ressemble à des modes d'adressage indirect mis-décodés. Pattern
identique au fix POPM : opcode mal classifié → mauvaise adresse effective.

`d_burst_d` à `0x0829` est aussi corrompu par `PC=0x8216` (zone FB-det).
Audit en cours.

---

## Patches en cours

### CONFIRMED — à appliquer

- **C-1.** Dédup `RETD` handler (F273) : deux implémentations byte-identiques,
  la 2e est dead code (`calypso_c54x.c` lignes 1701-1705).
- **C-2.** Dédup `CALLD` handler (F274) : pareil, lignes 1691-1699.
- **C-3.** Dédup `RPTBD` handler (F272) : cinq implémentations à auditer
  (1461, 1680, 2258, 3408, 3469).
- **C-4.** Fix `prev_sp` initial du SP-OOR tracer : `0xEEFF` → `s->sp`
  (tracer DBG_SP, risque nul).

### SPECULATIVE — non appliqué

- **S-1.** SP runaway PROM0 `0xb906` (F074/F274) : 6/10 runs baseline ont
  un SP runaway de ~13000 events, tous à `prev_PC=0xb906 prev_op=0xf074`.
  SP descend de `0x5AC8` vers `0x0000`, atteint `0x08F8` (= `d_fb_det`),
  écrase avec return addresses. Trois hypothèses (self-loop firmware,
  CALL dans RPTB, bug prog_fetch) non vérifiées.
- **S-2.** F5E3 SP runaway PROM0 `0xc12a` — symptôme similaire.

---

## Variables d'environnement

| Variable | Default | Effet |
|---|---|---|
| `CALYPSO_ICOUNT` | `auto` | Mode `-icount` QEMU. `auto` expose le bug INTM dwell ; `off` vit avec timing variance |
| `CALYPSO_FBSB_SYNTH` | `0` | `1` publie FB/SB synth dans NDB pour passer FBSB déterministe |
| `CALYPSO_FORCE_RX_DONE` | `0` | Workaround TCG bug sur STR conditionnel `0x8224ac` (SIM busy-poll) sous `icount=auto` |
| `CALYPSO_W1C_LATCH` | `0` | `1` latche valeurs `a_sync_demod` (mitigation race DSP-write/ARM-read) |
| `CALYPSO_BSP_DARAM_ADDR` | `0x3fb0` | Adresse DARAM cible des DMA RX BSP (sans effet sur le FB-det actuel — AR init firmware-dependent) |
| `CALYPSO_DSP_IDLE_FF` | `1` | Fast-forward DSP idle dispatcher (optim host-side) |
| `CALYPSO_DSP_FBDET_SKIP` | `0` | Option diag — skip FB-det inner loop |
| `CALYPSO_NDB_D_RACH_OFFSET` | `0x01CB` | Override word index `d_rach` (DSP version-dependent) |
| `CALYPSO_RACH_FORCE_BSIC` | unset | Force BSIC RACH (0-63) |
| `BRIDGE_CLK_FROM_QEMU` | `0` | `1` : CLK IND piloté par FN QEMU |
| `BRIDGE_DL_FN_REWRITE` | `slot` | Politique de réécriture FN DL (slot-aware vs naïve) |
| `BRIDGE_DL_FN_LOOKAHEAD` | `32` | Marge lookahead réécriture FN DL |
| `CALYPSO_DSP_ROM` | `calypso_dsp.txt` | Chemin du dump DSP ROM |
| `L1CTL_SOCK` | `/tmp/osmocom_l2` | Mobile↔QEMU L1CTL Unix socket |

---

## Probes runtime (qemu.log)

| Tag | Cible | Émet |
|---|---|---|
| `PC-HIST-3FB` | reads de `[0x3fb0..0x3fbf]` | Top PCs lisant zone BSP DMA |
| `PC-HIST-3DD` | reads de `[0x3dcf..0x3dd5]` | Top PCs scratch dominant |
| `WATCH-WRITE 0x3dd2` | writes à `0x3dd2` | Identité writer + valeurs |
| `INTM-TRANS` | transitions INTM 0↔1 | Cause SSBX/RSBX/STM ST1 |
| `WAIT-A21A` | PC=`0xa21a` | Snapshot INTM/IMR/IFR/ST0/ST1/SP |
| `ENTER-7740` | PC=`0x7740` | Caller chain + AR + insn |
| `ST1-WR` | STM #lk, ST1 (op 0x7707) | Tous les writes ST1 |
| `D_FB_DET-WR-SITE` | PC=`0x8f51` | AR0..AR7 + data[AR0/1/2] + BK + A |
| `D_FB_DET SET / OVERRIDE` | writes à `0x08F8` | Valeur + PC + delta-to-clear |
| `D_BURST_D-WR / SUMMARY` | writes à `0x0829 / 0x083D` | Séquence + transition matrix |
| `D_TASK_D-WR` | writes à `0x0828 / 0x083C` | Task status fin de task |
| `ARM RD a_cd / d_fb_det` | ARM reads NDB | Confirme mirror DSP→ARM |
| `A_CD-WR / BY-BURST` | writes DSP `0x09D0..0x09DE` | Buffer CCCH demod + histogramme |
| `STATE-DUMP / SP-RING` | toutes les N insn | PC + ST0/ST1 + IMR/IFR/INTM + SP + AR |

### Commandes diagnostic utiles

```bash
# Corruptions IMR
grep -c "IMR change" /tmp/qemu.log

# Top PCs source corruptions
grep "IMR change" /tmp/qemu.log | awk '{print $NF}' | sort | uniq -c | sort -rn | head

# Dernier état DSP
grep "TINT0:" /tmp/qemu.log | tail -3

# Stats bridge
tail /tmp/bridge.log

# IRQ DSP (vec 21 = BRINT0)
grep -E "c54x.*IRQ vec" /tmp/qemu.log | tail
```

---

## Conventions

- **No stubs in critical paths.** No `#ifdef QEMU`. Tous les shortcuts
  passés (BCCH inject, FBDET-SKIP, INTM force-clear, SI3 fallback) ont
  été purgés. Chacun masquait le vrai bug.
- **Verify opcodes against `tic54x-opc.c`** (binutils 2.21.1) avant tout
  patch. Référence : `hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md`.
- **QEMU est clock master.** Le bridge est slave. La BTS reçoit CLK IND
  wall-paced. Élimine le wall-clock vs virtual-clock desync.
- **Test après chaque edit.** Build Docker → vérifier DSP idle + SP + IMR
  + RETE count → pytest (49 milestones, 26 PASS stables).
- **Document workarounds** dans `hw/arm/calypso/doc/TODO.md` avec critères
  de retrait explicites.

---

## Roadmap

1. Solveur du mur `d_task_d` actuel — audit opcode des PCs `0x787d`,
   `0x7a03`, `0x79f1`, `0x7817`, `0x8216`.
2. Cascade L1 d'aval : `DATA_IND → RACH → Immediate Assignment → SDCCH
   → LU Accept`. Chaque étape va débloquer de nouveaux bugs.
3. `icount=auto` stable sans `FORCE_RX_DONE` (le bug INTM dwell résiduel).
4. DSP path réel sans `FBSB_SYNTH=1` : déterminisme 5/5 (actuellement
   1-2/5).
5. Performance : un run CI doit prendre des secondes, pas des minutes.
6. Compatibilité versions QEMU upstream + osmocom-bb.
7. Image Docker propre, packaging, install une commande.
8. Doc d'usage : "debug osmocom-bb avec qemu-calypso".
9. Tests CI publics (GitHub Actions), badge green.
10. Refacto pour respecter le style QEMU si ambition upstream.
11. Présentation OsmoDevCon — slide deck + démo live.

---

## License & attribution

- **QEMU base** : GPL-2.0-or-later
- **Calypso emulator additions** : GPL-2.0-or-later
- **osmocom-bb firmware** : GPL (utilisé tel quel, pas redistribué)
- **Calypso DSP ROM** (`calypso_dsp.txt`) : TI propriétaire. Dump physique
  pour recherche et interopérabilité (osmocom DSP dumper). Non redistribuable
  commercialement sans autorisation TI.
