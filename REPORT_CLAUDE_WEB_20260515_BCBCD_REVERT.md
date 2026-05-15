# Rapport Claude web — 2026-05-15 nuit (post-fix BC/BCD, revert)

> Session 2026-05-15 fin journée → nuit. Suite directe des breakthroughs
> DSP→ARM mirror + STL/STH 0x81/0x83. Tentative fix BC/BCD cond NTC/TC
> écartée après mesure post-rebuild.

## TL;DR

**Acquis solides cette session** :
1. ✅ **Fix STL/STH 0x81/0x82/0x83/0x84** (md5 c54x.c `70cfbbae` puis `7ef805f6`)
   — d_burst_d corruption éliminée, `BURST ID` printf 24→0, transitions
   d_burst_d 0→1 propres apparaissent.
2. ✅ **Fix UTF-8 docker_exec** dans test harness, `test_intm_dwell_no_regression`
   passe.
3. ✅ **test_d_fb_det_pattern_unchanged** conditional sur synth=1 → XFAIL propre.
4. ✅ **Probes ajoutées** : ARM RD a_cd[] (calypso_trx.c), D_TASK_D-WR
   (calypso_c54x.c).
5. ✅ **Identifié pourquoi d_task_d=0** : Le firmware utilise `ST #1, Smem` à
   PROM 0x9ab1 pour NB task. MF_TASK_CCCH côté ARM utilise NB_QUAD_DL → dispatch
   TASK_NB=1 (pas TASK_ALLC=24). Donc la valeur attendue dans d_task_d est 1,
   pas 24.

**Tentative écartée** :
- ❌ **Fix BC/BCD cond NTC/TC** : implémenté per SPRU172C strict, mais le
  firmware DSP Calypso **dépend du comportement ACC-based historique**.
  Régression complète sous synth=1 ET icount=auto. Revert appliqué (md5
  `69524ae5`).

**Mur DATA_IND restant** :
- DSP write d_task_d=0x0001 fire ~2 fois en 80s (vs 168 EMPTY printf ARM-side)
- Ratio set/empty = 1.2% — le code DSP qui set d_task_d=1 quasi jamais atteint
- Cause root suspectée : BITF (0x61) ou path conditionnel amont mal émulé

## Chronologie détaillée

### Étape 1 — Setup banc d'essai déterministe

`CALYPSO_ICOUNT=off + FBSB_SYNTH=1 + FORCE_RX_DONE=0 + W1C_LATCH=0` :
- task=24 fire massivement (~74/s, déterministe)
- A_CD-WR fire massivement (~16/s)
- DATA_IND = 0 (mur)

### Étape 2 — Identification du mur

Probe `ARM RD a_cd[]` ajoutée → fire jamais. Donc ARM bail avant la lecture
a_cd[]. Grep prim_rx_nb.c → check ligne 73 `if (db_r->d_task_d == 0) { puts("EMPTY"); return 0; }`. EMPTY printf = 168 dans osmocon.log → **bail confirmé sur d_task_d=0**.

### Étape 3 — Identification du writer NB

`D_TASK_D-WR val=0x0001 exec_pc=0x9abf` × 2 occurrences. PROM dump autour de
0x9abf révèle séquence :
```
0x9ab1: 7683  ST #lk, Smem (opcode 0x76)
0x9ab2: 0001  literal TASK_NB=1
```

**Découverte clé** : aucun `ST #0x0018` (TASK_ALLC=24) dans tout le PROM →
firmware ne fait jamais write d_task_d=24. Recherche source : `mframe_sched.c`
montre `MF_TASK_CCCH` utilise `NB_QUAD_DL` scheduler → dispatch TASK_NB=1,
pas TASK_ALLC=24. Donc valeur attendue dans d_task_d est 1 (cohérent avec les
2 SETs observés).

### Étape 4 — Identification des conditionnelles upstream

PROM dump 0x9aa0..0x9ab2 :
```
0x9aa0: 6160       BITF (set TC)
0x9aa1: 0001       bit mask
0x9aa2: fa20       BCD pmad, NTC  (delayed)
0x9aa3: 9aad       target
0x9aa4: 8edc       (delayed slot)
0x9aa5: 8fdb       (delayed slot)
0x9aa6: 611a       BITF
0x9aa7: 0001       bit mask
0x9aa8: f820       BC pmad, NTC
0x9aa9: 9aad       target (early RET path)
0x9aaa: ec0f       RPT 16
...
0x9aad: 8d96       ST T, Smem (early RET body)
0x9aae: fc00       RET
...
0x9ab1: 7683 0001  ST #1, Smem (notre cible — seulement si BC NTC fall-through)
```

Le path vers `ST #1` à 0x9ab1 nécessite que les 2 BC NTC fall-through, ce qui
demande TC=1 sur 2 BITF successifs.

### Étape 5 — Tentative fix BC/BCD (écartée)

Inspection calypso_c54x.c:2873-2897 (handler BC F8xx) :
```c
if (sub == 0x2 || sub == 0x3) {
    /* For now: cond=0x20 → branch if A != 0; cond=0x30 → A == 0.
     * These are heuristics until we confirm the exact cond
     * mapping from SPRU172C. Tweak based on observed dispatcher behaviour. */
    if (sub == 0x2)      take = (acc_signed != 0);
    else /* sub==0x3 */  take = (acc_signed == 0);
}
```

Le commentaire historique avoue heuristique non-conforme SPRU172C. Per spec :
- `cond=0x20` = NTC (branch if !TC)
- `cond=0x30` = TC (branch if TC)

Fix appliqué :
- F820/F830 (BC sub=0x2/0x3) → cond eval correcte TC/NTC
- FA20 (BCD NEAR fallback) → cond eval (était unconditional branch)

Total sites PROM impactés : F820=654 + F830=557 + FA20=115 + FA30=119 = **1445**.

### Étape 6 — Régression post-rebuild (rebuild compilation correcte après fix
brace mismatch)

**Run 1 synth=1 + icount=off + BC/BCD fix (uptime 35s)** :
- task=24 : 241 (vs 5957 baseline équivalent) — drop ×24
- D_TASK_D val=1 SET : **0** (vs 2 baseline)
- A_CD-WR : 154 (vs 1269 baseline équivalent)
- EMPTY printf : 120 (rate similaire)

**Run 2 synth=1 + icount=off + BC/BCD fix (uptime 2 min)** :
- task=24 : 1067 (= 8.3/s vs baseline ~74/s) — drop ×9
- DSP stuck à PC=0xcc51 (zone IDLE dispatcher)
  ```
  STATE-DUMP PC=0xcc51 INTM=1 IMR=0xaa28 IFR=0x0028 SP=0x0102
  AR1 cycle 0x9993→0xcb93→0xfd93 (loop counter)
  ```
- Insn rate normal 36M/s — pas régression performance, régression cascade

**Run 3 icount=auto + W1C_LATCH=1 + BC/BCD fix (uptime 48s)** :
- task=24 : **0** (vs intermittent baseline)
- DSP stuck à PC=0xfa95 (RPTB loop)
  ```
  STATE-DUMP PC=0xfa95 ST0=ST1=IMR=0x5c1f (registres suspects identiques)
  BRC décroît 13651→12662→8780 (block-repeat counter)
  AR1 décroît (loop)
  ```
- Invariants OK : WAIT-A21A=0, ENTER-7740=0

**Verdict** : le fix BC/BCD casse le firmware DSP dans les **deux régimes** :
- synth=1 + icount=off : DSP stuck PC=0xcc51
- icount=auto : DSP stuck PC=0xfa95 (RPTB)
- Position différente du loop confirme régression structurelle, pas locale

### Étape 7 — Revert

Reverted to original heuristic-based BC handler. BCD revert to unconditional
fallback. Fix STL/STH **conservé** (validé par disparition BURST ID printf
24→0 multi-runs). md5 c54x.c final : `69524ae5`.

## Pourquoi le fix BC/BCD régresse — hypothèses ordonnées

### (a) Dialecte Calypso TI [probable]

Le binaire DSP Calypso n'est **pas un binaire SPRU172C strict**. Les opcodes
F820/F830 peuvent avoir sémantique custom où les bits cond byte ne sont pas
NTC/TC mais autre chose. Le commentaire historique disait "dispatcher uses
cmp-style behaviour" — c'est cohérent.

### (b) BITF (0x61) émulation incorrecte [possible, à mesurer]

Si notre BITF ne set pas TC correctement, alors cond NTC est toujours vrai
(TC stuck à 0), donc :
- Ancien BC : branchait sur A!=0 (= souvent)
- Nouveau BC : branche sur TC=0 (= toujours)
- Le DSP boucle parce que la branche NTC est toujours prise → mauvais code path

Vérification possible : ajouter probe BITF (compteur appel + valeur TC
result). Si TC quasi jamais 1 → BITF buggy ou condition test impossible
à satisfaire dans notre émulation.

### (c) Multiple opcodes interagissent [moins probable]

Le firmware peut combiner BC/BCD avec d'autres opcodes status-modifying
(SSBX TC, RSBX TC explicite) dont l'émulation interagit mal avec BC strict.

## Pour comparer avec real HW

Sur vrai Calypso (osmocom-bb FreeCalypso), DSP fonctionne correctement et
ARM L1 reçoit DATA_IND. Donc le binaire DSP **fonctionne avec BC standard
SPRU172C** sur le vrai HW. Donc :
- (a) dialecte TI : moins probable (le HW exec correctement SPRU172C-strict)
- (b) BITF émulé incorrectement : plus probable

Direction d'investigation : audit BITF + état status word ST0.TC dans notre
émulateur.

## État final du projet

### Acquis cette session (post-revert)
1. Fix STL/STH 0x81/0x83 → BURST_ID printf éliminé
2. Banc d'essai déterministe `icount=off + synth=1` fonctionnel
3. Localisation précise du mur DATA_IND : `d_task_d=0` côté ARM, set rare côté DSP
4. PROM décodé pour la séquence NB write d_task_d=1
5. Identification de la conditionnelle critique (BITF + BC NTC à 0x9aa0-0x9aa8)
6. Documentation TI complète localisée (`doc/opcodes/`, `spru172c.pdf`)

### Le mur restant
- 2 d_task_d=1 SETs en 80s vs 168 EMPTY = ratio 1.2%
- DSP completion path à PROM 0x9ab1 quasi jamais atteint
- Cascade DATA_IND morte par cette voie

## Questions pour toi (Claude web)

1. **Sur l'hypothèse BITF buggé** : tu valides la direction "instrumenter BITF
   d'abord pour vérifier qu'il set TC correctement" avant toute autre tentative
   sur BC/BCD ? Probe simple à ajouter dans calypso_c54x.c ligne 4040-4048 :
   compteur appels + TC result + count "TC monte à 1" vs "TC reste 0".

2. **Sur la stratégie pour atteindre PROM 0x9ab1** :
   - **(a)** Fix BITF si bug identifié, puis revérifier fix BC/BCD strict
   - **(b)** Workaround W1C-style : intercepter `D_TASK_D-WR val=0x0001`
     côté QEMU et latch sticky jusqu'à read ARM (= relâcher la rareté du
     SET en faisant durer plus longtemps)
   - **(c)** Audit complet famille opcodes status-modifying (BITF, SSBX,
     RSBX, BC, BCD) pour identifier si c'est une suite de bugs cohérents
   - **(d)** Patch firmware ARM : modifier prim_rx_nb.c:73 pour accepter
     `d_task_d` ancienne valeur (ne pas bail sur 0)

   Tu vois autre chose ?

3. **Sur l'impact 1445 sites BC/BCD impactés** : c'est massif comparé au F3
   fix (371 sites). Si la cause est dialecte Calypso (hypothèse a), refaire
   ce fix proprement demanderait un audit per-site, pas une réimplémentation
   famille. Tu estimes que c'est faisable en 1-2 jours dédiés, ou c'est un
   projet en soi qui requiert reverse engineering du firmware DSP ?

4. **Bonus** : sur le run icount=auto post-fix, observation que ST0=ST1=IMR=0x5c1f
   simultanément à PC=0xfa95. Pattern de corruption massive registres. Tu
   reconnais ce pattern ? Possible signature d'un fault path (TRAP) ou
   d'une exception mal gérée ?

## Annexe — métriques numérisées

| Run | uptime | task=24 | A_CD-WR | D_TASK_D=1 | EMPTY | BURST_ID | DATA_IND |
|---|---|---|---|---|---|---|---|
| Baseline pré-STL fix | 80s | 5957 | 1269 | — | — | many | 0 |
| Post-STL fix (synth=1) | 35s | 121 | 251 | 0 | 8 | 0 ✓ | 0 |
| Post-STL fix (synth=1) | 80s | 5957 | 1269 | 2 | 168 | 0 | 0 |
| Post-BC/BCD fix (synth=1, 35s) | 35s | 241 | 154 | 0 | 120 | 0 | 0 |
| Post-BC/BCD fix (synth=1, 2min) | 128s | 1067 | 111 | 0 | 532 | 0 | 0 |
| Post-BC/BCD fix (icount=auto, 48s) | 48s | 0 | 154 | 0 | 0 | 0 | 0 |

L'observation `BURST ID printf = 0 multi-runs` confirme que le **fix STL/STH
tient** (d_burst_d corruption éliminée). C'est le seul acquis solide de cette
session post-revert.

## Diff total cette session (après revert)

```
hw/arm/calypso/calypso_c54x.c
  + D_TASK_D-WR probe à 0x0828/0x083C (lignes 805-840)
  + Fix STL/STH 0x81/0x82/0x83/0x84 (lignes 4707-4720, 4839-4862, 4881-4903)
  + Commentaire revert BC/BCD à 2873-2913, 3478-3486

hw/arm/calypso/calypso_trx.c
  + Probe ARM RD a_cd[0..14] à offset 0x03A0..0x03BD (lignes 250-269)

tests/test_calypso_milestones.py
  + Fix UTF-8 docker_exec (errors='replace')
  + Fix sémantique XFAIL-3-états test_l1ctl_data_ind_received

tests/test_run_observability.py
  + Fix UTF-8 dexec_sh
  + Conditional synth=1 → xfail test_d_fb_det_pattern_unchanged
  + Idem fix sémantique XFAIL-3-états DATA_IND
```

md5 c54x.c : `69524ae5089906e39c693641d5876399`
