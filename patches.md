# Calypso QEMU — Patches & TODO (2026-04-07)

Session de debug FBSB/FB-detect côté DSP/ARM. 27 runs exécutés, agents Explore
sur calypso C + osmocom firmware. Findings consolidés ci-dessous.

---

## 1. Découvertes (factuel, vérifié)

### 1.1 Variance entre runs = 1 bug DSP

3 régimes observés sur 10 runs baseline (`run_all.sh`):

| Régime | Runs (sur 10) | Caractéristique |
|--------|---------------|-----------------|
| **A. Mort précoce** | 004 | FBSB=0, l1ctl=13. L1 scheduler jamais armé. max_fn ~52k |
| **B. SNR=0 propre** | 005, 007, 009 | FBSB 64-70, snr=0, **0 SP runaway**, 0 d_fb_det WR |
| **C. Garbage NDB** | 006, 008, 010-013 | FBSB 32-44, snr=46 const, **~13k SP_events**, **700+ d_fb_det WR** |

**Corrélation parfaite SP_runaway ↔ d_fb_det WR**: 6/10 runs ont les deux,
4/10 n'ont ni l'un ni l'autre.

### 1.2 Bug DSP — opcode 0xF074 / 0xF274 et SP runaway

Trace observée :
```
[DBG_SP] SP-STEP: 2c00->2bff prev_PC=0xb906 prev_op=0xf074
[DBG_SP] SP-STEP: 2800->27ff prev_PC=0xb906 prev_op=0xf074
...
[DBG_CORRUPT] DSP WR d_fb_det = 0xb908 PC=0xb906 SP=0x08f8 *** GARBAGE ***
```

- **0xF074** = `CALL pmad` (binutils tic54x-opc.c:279, mask 0xFFFF)
- **0xF274** = `CALLD pmad` (delayed call, 2 delay slots)
- **0xb908** poussé sur la stack = adresse de retour = PC + 2

Cascade :
1. CALL ou CALLD à PC=0xb906 push une return address sur la stack
2. SP descend à chaque appel
3. Le callee ne RET jamais (ou re-CALL en boucle)
4. Quand SP atteint 0x08F8 (= adresse DSP de `d_fb_det` dans la NDB), chaque
   PUSH écrase `d_fb_det` avec une return address ≠ 0
5. ARM lit `d_fb_det != 0` → croit avoir trouvé un FB
6. Lit snr/freq depuis NDB pourrie → `FBSB_CONF result=255`

**Implémentation actuelle (calypso_c54x.c)** : F074 et F274 sont **codés correctement**
en isolation (push return, set PC=target, return 0 = "PC déjà set, ne pas avancer"). Lignes 1311-1317
et 1473-1481.

**Hypothèses pour le mécanisme exact (non confirmées)** :
- a. **Self-loop firmware** : `prog[0xb907] == 0xb906` (call à soi-même). → vérifier en dumpant PROM[0xb906..0xb910]
- b. **Bug RPTB+CALL** : CALL à PC=REA d'un bloc RPTB. Le wrap check (ligne 3842 :
  `if (s->pc == s->rea + 1)`) ne déclenche jamais car le CALL met PC=target (≠ REA+1)
- c. **Bug prog_fetch** : 0xb907 lu mal (mais XPC fix vérifié OK)

### 1.3 Bug DSP secondaire — opcode 0xF5E3

Run 014 (avec hack r8=1) expose un **deuxième** site de SP runaway :
```
[DBG_SP] SP-STEP: 5000->4fff prev_PC=0xc12a prev_op=0xf5e3
```

- 0xF5E3 = pas immédiatement identifiable dans tic54x-opc.c (zone F5xx)
- Cluster `0xc121-0xc12a` = fonction DSP **réellement appelée par hack only**
  (la baseline bail à `if (d_fb_det==0)` avant de l'atteindre)
- Le hack a donc **révélé** ce 2e bug en ouvrant un nouveau chemin d'exécution
- Run 014 a **69086 SP_events** vs ~13k baseline → 5× plus de bleed

### 1.4 Décision tree FBSB côté firmware ARM

Source : `/opt/GSM/osmocom-bb/src/target/firmware/layer1/prim_fbsb.c`

```
l1s_fbdet_resp (ligne 399)
├─ ligne 404: if (!d_fb_det) → bail (BP#1 patche r8=1 ici)
├─ read_fb_result → lit snr/freq_diff depuis NDB (a_sync_demod[D_SNR/D_ANGLE])
├─ ligne 449: if (abs(freq_diff) < freq_err_thresh1 && snr > FB0_SNR_THRESH=0)
│   ├─ PASS → schedule FB1
│   └─ FAIL → afc_retries++; si retries >= 30 → attempt=13, l1s_compl_sched
├─ ligne 488: if (abs(freq_diff) < freq_err_thresh2 && snr > FB1_SNR_THRESH=0)
│   ├─ PASS → schedule SB
│   └─ FAIL → reschedule FB1
└─ l1a_fb_compl (ligne 523, async via l1s_compl_sched(L1_COMPL_FB))
    ├─ if (attempt >= 13) → l1ctl_fbsb_resp(255)  ← FAIL
    └─ else → l1ctl_fbsb_resp(0)                   ← SUCCESS
```

### 1.5 Disas du chemin critique l1s_fbdet_resp

```asm
0x826424 <l1s_fbdet_resp>:
  826430: ldrh r8, [r3, #72]   ; r8 = ndb->d_fb_det
  826434: lsl r2, r2, #16        ← BP#1 (après ldrh)
  826438: cmp r8, #0
  826444: bne 0x826490            (FB found path)
  826448: cmp r7, #11             (attempt vs 11)
  ...
  ; FB found, après plein de read NDB:
  8265a8: ldrsh r2, [r4, #16]   ; r2 = freq_diff (signed)
  8265ac: ldrh  r3, [r4, #36]   ; r3 = freq_err_thresh1
  8265b0: cmp r2, #0
  8265b4: rsblt r2, r2, #0       ; r2 = abs(freq_diff)
  8265b8: cmp r2, r3
  8265bc: bge 0x8265d8            ← BP#2 (skip bge → force PASS)
  8265c0: ldrh r3, [r4, #14]    ; r3 = fb.snr
  8265c4: cmp r3, #0              ← BP#3 (force r3=1)
  8265c8: movne r0, #1
  8265d0: movne r2, r0
  8265d4: bne 0x826704             (success → schedule FB1)
```

## 2. Patchs appliqués

### 2.1 `hack_gdb.py` — 4 breakpoints (mode hardcoded par défaut)

| BP | Adresse | Action | But |
|----|---------|--------|-----|
| #1 | `0x826434` | `r8 := 1` | Force `d_fb_det == 1` (l1s_fbdet_resp+0x10) |
| #2 | `0x8265bc` | `PC := PC+4` | Skip `bge`, force `\|fd\| < thresh` |
| #3 | `0x8265c4` | `r3 := 1` | Force `snr > 0` |
| #4 | `0x826754` | `r3 := 0` | Force `attempt < 13` (l1a_fb_compl+0x8) |

**Modes de résolution** (par ordre de priorité):
1. `HACK_BP_FBDET=0xADDR` / `HACK_BP_FBCOMPL=0xADDR` env override
2. `HACK_DISCOVER=1` → resolve via objdump symbol table de `layer1.highram.elf`
3. Hardcoded defaults (mode normal)

### 2.2 `run_all_debug.sh` / `run_all.sh`

- Loop N runs en headless
- Logs incrémentés `/root/logs/{qemu,bridge,inject,mobile,hack}_NNN.log`
- Symlink `/tmp/qemu.log` → run courant
- Watchdog `QEMU_LOG_MAX` (50 MB par défaut) + `RUN_TIMEOUT` (45s par défaut)
- Cleanup propre SIGTERM puis SIGKILL
- Tail live de qemu.log dans le terminal courant
- Affiche `HACK_MODE` au démarrage

## 3. TODO (par ordre)

### 3.1 Test des nouveaux BPs (immédiat)
- [ ] Lancer `bash run_all_debug.sh 5` après ce build
- [ ] Vérifier dans `hack_NNN.log` : compteurs FB / FREQ-SKIP / SNR-FORCE / COMPL > 0
- [ ] Vérifier dans `qemu_NNN.log` : `FBSB_CONF result=0` apparaît au moins 1 fois
- [ ] Si compl_force > 0 mais result=255 → bug downstream (afc_retries reset par mobile retransmit)

### 3.2 Investigation DSP F074/F274 (priorité 1)
- [ ] **Dumper PROM[0xb906..0xb910]** depuis le firmware DSP bin pour voir si `prog[0xb907] == 0xb906` (self-loop)
- [ ] Si pas self-loop → tracer le RPTB state au moment où PC=0xb906 (rea, rsa, brc, rptb_active)
- [ ] Tester l'hypothèse RPTB+CALL : ajouter un trace `[F074-IN-RPTB]` quand `op==0xF074 && rptb_active && pc == rea`
- [ ] Si RPTB+CALL confirmé → fix dans calypso_c54x.c : déclencher le wrap_check au moment du CALL si CALL est à REA

### 3.3 Investigation opcode 0xF5E3
- [ ] Décoder 0xF5E3 dans tic54x-opc.c (zone F5xx peu peuplée)
- [ ] Si non répertorié → c'est probablement un C548-only (extension TI undocumented)
- [ ] Localiser le handler actuel dans calypso_c54x.c (peut-être traité comme NOP)

### 3.4 Cleanup safe (calypso_c54x.c)
- [ ] **RETD dupliqué** : lignes 1483-1487 et 1701-1705 sont **byte-identiques**. La 2e est dead code (jamais atteinte). Supprimer la 2e.
- [ ] **RPTBD dupliqué** : lignes 1461 et 1680 sont **byte-identiques** pour F272. Supprimer la 2e.
- [ ] **CALLD dupliqué** : lignes 1473 et 1691 idem F274. Supprimer la 2e.
- [ ] (Cleanup ≠ bugfix : ne change pas le comportement, juste le code mort.)

### 3.5 Tracer fixes
- [ ] SP-OOR tracer ligne 3633 : `prev_sp = 0xEEFF` initial alors que boot SP = 0x5AC8 → premier événement faussé. Fix : initialiser `prev_sp = s->sp` au reset.
- [ ] Multiples blocs `static int _re=0; if (_re<50)` inline dans c54x_exec_one — extraire en helper macro.

### 3.6 ARM-side investigation (régime A)
- [ ] Pourquoi runs 004 / 015-018 ont FBSB=0 et seulement 13 events l1ctl ?
- [ ] Race au boot entre injection cfile / TPU clock / IMR mask ?
- [ ] À investiguer **après** fix DSP (régime A pourrait disparaître).

## 4. Métriques cibles (avant/après fix DSP)

| Métrique | Baseline médian | Cible |
|----------|-----------------|-------|
| SP_events / run | 13000 (régime C) | 0 |
| d_fb_det WR garbage | 760 (régime C) | 0 |
| FBSB result=0 | 0 / 27 runs | ≥ 1 / 5 runs (avec hack) |
| DATA_IND | 0 | ≥ 1 (rêve à long terme) |

## 5. Règle inviolable

> **Le code C est sacré.** Aucun fix C sans:
> 1. Cause-racine identifiée et reproductible
> 2. Référence datasheet (SPRU172C) ou binutils (tic54x-opc.c)
> 3. Test sur ≥ 5 runs avant et après
>
> Tout hack doit vivre exclusivement dans `hack.py` / `hack_gdb.py`.
