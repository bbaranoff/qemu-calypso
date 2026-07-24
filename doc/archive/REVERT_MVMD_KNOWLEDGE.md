# MVMD/MVDM Revert Knowledge — 2026-05-15 nuit

> Doc de session, à NE PAS perdre. Le revert est pragmatique, pas un constat
> d'erreur sémantique. Re-lire avant de retoucher 0x72/0x73 dans c54x.c.

## TL;DR

Le fix 0x72/0x73 = MVDM/MVMD (per binutils tic54x-opc.c) est **sémantiquement
correct** mais a **régressé le runtime** parce que le firmware Calypso DSP
dépend du **side-effect compensatoire** du mal-décodage.

État revert :
- 0x72/0x73 handlers RETIRÉS
- STL/STH 0x81/0x82/0x83/0x84 fixes CONSERVÉS (validés)
- F3 fix CONSERVÉ (validé)
- BC/BCD heuristique ACC-based CONSERVÉ (revert précédent — même pattern)

## Analyse sémantique du bug

### Spec officielle

Per `tic54x_hi8_map.md` ligne 80-85 :
```
| 0x72 | mvdm | 0x7200 / 0xFF00 |  (move dmad → MMR)
| 0x73 | mvmd | 0x7300 / 0xFF00 |  (move MMR → dmad)
```

### État avant fix (= état post-revert)

- Pas de handler explicite pour `hi8 == 0x72` ni `hi8 == 0x73`
- Fallthrough vers `if ((op & 0xF800) == 0x7000)` ligne 3999 → décode comme
  **STL src, Smem** (avec mask 0xF800 qui catch 0x70-0x77 trop largement)
- Site critique : PC=0x8208 op=`0x7317` (= MVMD AR7, BRC=0x001a)
  → décodé comme `STL B, *(DP+0x17)` (bit 9=1 = src=B, bits 7:0=0x17 = direct DP)
  → écrit B low part dans data[DP+0x17]
  → side-effect : écrit dans une MMR random (selon DP), parfois corrupt AR7

### État après fix (= état régression)

- `hi8 == 0x73` correctement catch → MVMD AR7, BRC=0x001a → BRC = AR7 value
- Plus de corruption side-effect
- **Mais le firmware boucle dans la zone init/clear (PC=0xfd23/0xfd25)
  au lieu d'atteindre PC=0x8208 RPTB compute**

## Mesures empiriques

| Métrique | Pre-fix | Post-fix | Lecture |
|---|---|---|---|
| INTM-TRANS PC=0x8208 op=0x7317 | 363/run | **0** | Bug INTM stuck éliminé ✓ |
| A_CD-WR total | 244 | **6** (-97%) | Quasi plus de compute réel |
| A_CD-WR valeurs | mix | tous 0x0000 (clears) | Init buffer uniquement |
| A_CD-WR PCs | 0x9abc/0x9ace + parasites | 0xfd23/0xfd25 | Path différent |
| D_TASK_D val=1 SET | 2/run occasionnel | **0** | Path completion jamais atteint |
| task=24 fire | 195-337 | 65-89 | Dispatch ralenti |
| INT3 fire | 1583/min | 1 total | TPU/TSP ne génère plus |
| LOST printf | 1788+ | 1756 | Similar |
| DSP busy state | INTM dwell à 0x8208 | Reset vector loop (PC=0x0000) | Pire |
| DATA_IND | 0 | **0** | Mur identique |

## Pourquoi le firmware dépend du bug

Hypothèse forte (Claude web validée par les chiffres) :

Le mal-décodage `STL B, *(DP+0x17)` au site 0x8208 écrivait régulièrement la
partie basse de l'accumulateur B dans data[DP+0x17] :
- Si DP=0, ça écrit dans MMR à l'offset 0x17 = AR7
- Si DP=1, ça écrit dans data[0x97] (= zone scratch DARAM)

Cette écriture involontaire **maintenait AR7 (ou une cellule de travail) dans
un état qui permettait au firmware d'atteindre 0x8208 dans les cycles
suivants**. Sans le bug, AR7 reste à sa "vraie" valeur (peut-être 0), ce qui
fait diverger le control flow ailleurs (vers init/reset).

**Le firmware est écrit en supposant cette écriture continue**. Ce n'est pas
notre fix qui est faux ; c'est que le firmware binaire compilé pour Calypso
exploite (consciemment ou non) le side-effect d'une instruction qui
*aujourd'hui* est documentée différemment dans binutils.

OU : il existe un opcode UPSTREAM (avant 0x8208) qui était supposé
initialiser AR7 mais qu'on émule incorrectement → AR7 reste 0/garbage → MVMD
fixe écrit 0 dans BRC → RPTB tourne 1 fois → firmware diverge.

Sans probe AR7 active sur tout le run, impossible de trancher entre ces deux
hypothèses.

## Findings à PRÉSERVER

### 1. PC=0x8208 = MVMD AR7, BRC + RPTBD à 0x820a

PROM dump (vérifié) :
```
0x8208: 7317 001a    ← MVMD AR7, BRC (op 0x7317 + dmad 0x001a = BRC MMR)
0x820a: f272 820f    ← RPTBD pmad=0x820f (block repeat delayed)
0x820c: a428         ← delay slot 1
0x820d: a5a9         ← delay slot 2
0x820e: b028         ← RSA (start of repeated block)
0x820f: b3a9         ← REA (end of repeated block)
0x8218: 6e89 8208    ← BANZD 0x8208 (loop back, outer loop)
```

### 2. CALLD 0x8207 sites — entrée vers la fonction

```
PROM 0x07d50 : ... f274 8207 7717 001d f7b6 ...
PROM 0x08000 : ... f274 8207 7717 001d ...
```

- `f274 8207` = CALLD 0x8207 (delayed call)
- Delay slot : `7717 001d` = STM #0x001d (=29), AR7

Donc AR7 doit valoir 29 à l'entrée de 0x8207 selon le firmware-design intent.

### 3. Zone alternative trouvée — PROM1 (XPC=1)

```
PROM 0x19aa0 + 12 : f272 9ab1
```
= `BD 0x9ab1` unconditional dans PROM1 — route alternative qui bypass les
BITF gates à 0x9aa0/0x9aa6 pour atteindre `ST #1, d_task_d` à 0x9ab1.

XPC=1 visité **1 seule fois** sur 928M insns dans nos tests = dead code dans
notre émulation. Cette route n'est pas active.

### 4. BITF fonctionne correctement

Audit complet effectué : BITF (opcode 0x61xx) set TC bien quand `(mem & mask)
!= 0`, clear sinon. Sur 104 BITF calls observés, 9.62% TC=1 — cohérent avec
les valeurs mémoire majoritairement zéro à ces sites.

PC=0x9aa0 et PC=0x9aa6 (= les BITFs qui gateraient `ST #1, d_task_d` à
0x9ab1) **JAMAIS appelés** sur 51 min de run. Le path BITF-gated n'est pas
celui qu'utilise le firmware en pratique.

### 5. IRQ-FRAME-HEALTH baseline pattern (pre-fix)

```
int3_fire = 25/s parfait (cadence TDMA 4.6ms)
int3_serviced = 2 (early boot only)
service_ratio = 0.13% (= 2 ISR servies sur 1583 fires)
avg_latency = 238506 insn (6.3 ms wall)
```

Donc même avec le bug INTM stuck éliminé, le DSP ne sert que 2 INT3 totaux —
la racine du problème est plus profonde que le seul mal-décodage 0x73.

## Criteria de re-application du fix MVMD/MVDM

Le fix DOIT être réappliqué un jour. Conditions pour la prochaine tentative :

1. **PC=0x8208 atteint ≥10× par run** avec fix appliqué (signe que le
   firmware progresse correctement même avec MVMD propre)
2. **A_CD-WR avec fix ≥ A_CD-WR sans fix** (mesure objective de progression
   CCCH demod)
3. **Opcode upstream identifié et fixé** — le bug compensateur du firmware
   doit être remplacé par le vrai comportement (probablement un autre
   opcode initialisation AR7 mal décodé)
4. **Probe AR7 active sur tout le run** pour mesurer la chaîne causale
   complète avant retest

## Plan d'attaque future

### Étape 1 — Probe AR7 globale (pas juste à 0x8208)

Étendre la probe AR7-INIT-CHAIN pour échantillonner régulièrement (toutes
les 100M insn) la valeur d'AR7 et le ring de modifications, indépendamment
de PC. But : voir si AR7 est initialisé à 29 quelque part dans le flow
normal.

### Étape 2 — Trace divergente runs avec/sans fix

Run identique 2× avec et sans le fix 0x72/0x73, dump les 1000 premiers PCs
exécutés. Le point de divergence identifie l'opcode qui produit l'état
différent → c'est probablement le coupable upstream.

### Étape 3 — Cross-reference osmocom-bb firmware source

Le firmware Calypso peut être partiellement source-visible dans
osmocom-bb (au moins l'ARM-side qui dispatche). Identifier les routines qui
écrivent dans la zone autour de 0x8200 et les patterns d'init d'AR7
attendus.

### Étape 4 — Audit famille opcodes status-modifying

Comme suggéré par Claude web précédemment :
- SSBX / RSBX / RETE / BC[D] conditionnels
- Tous ceux qui dépendent du status word
- Audit per-opcode vs binutils

Le bug compensateur trouvé peut faire partie d'une famille plus large.

## Reference au commit du revert

Voir le commit `<HASH>` (à remplir après commit). Ce commit retire les
handlers explicites pour hi8 == 0x72 et hi8 == 0x73 (ajoutés dans le
commit précédent `<HASH>` du fix MVMD/MVDM).

Le state pre-fix est restoré : 0x72/0x73 fallthrough sur la generic STL
au mask 0xF800 ligne 3999.

## État conservé après revert

- STL/STH fixes 0x81/0x82/0x83/0x84 → KEPT
- F3 family fix (lines 3020-3143) → KEPT
- BC/BCD revert (heuristique ACC-based pour sub==0x2/0x3) → KEPT
- Probes BITF, XPC, DISPATCH-CALLER, DISPATCH-ENTRY, EXIT-COMPUTE,
  COMPUTE-STATS, IRQ-FRAME-HEALTH, INT3-BLOCKED, AR7-INIT-CHAIN,
  MVMD-AR7-BRC, RPTB-ARMED → KEPT (dormants quand le code ne fire pas)
- Patches harness test_run_observability.py (5 patches) + test_calypso_milestones.py (`|| true` fix) → KEPT

## Le mur reste

DATA_IND = 0 stable. Le mur upstream qui empêche le firmware d'atteindre
le path completion (ST #1, d_task_d à 0x9ab1) n'est pas résolu. Mais
l'arsenal de probes est en place pour la prochaine session.
