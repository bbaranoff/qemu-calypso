---
name: Boot-to-FBSB sequence (silicon reset → FB lock → SB lock)
description: End-to-end DSP+ARM control flow from silicon reset to FBSB completion
type: project
---

# Calypso DSP+ARM : Silicon Reset → FBSB Lock

Cartographie complète du chemin que doit prendre l'ensemble DSP+ARM
pour passer du reset hardware jusqu'à un FBSB_CONF émis vers L2/L3.
Établie 2026-05-28 par triangulation BOOT-BRANCH / BOOT-MMR-WR /
SOFT-RESET-TRIG / SP-CATASTROPHE / fbsb host tracker.

## Vue d'ensemble

```
[silicon reset] → [boot init] → [scheduler ARM] → [DSP FB-det] → [FB lock] →
[SB demod] → [SCH decode] → [a_sch[] write] → [ARM consume] → [FBSB_CONF]
   ↑                                                                ↓
   └─────────── soft-reset si état détecté incohérent ──────────────┘
                (cycle qui nous a piégés tour 12-15)
```

## Phase 0 — Silicon reset (insn=0)

| Étape | État |
|---|---|
| `PMST.IPTR = 0x1FF` (par défaut) | Vecteur reset à PC = 0xFF80 |
| `SP = 0x1100` | Valeur silicon reset (= MMR_AR0 area, *invalide* pour stack) |
| `IMR = 0x52FD` | Pattern silicon reset (= partiellement masqué) |
| `B = 0x0` | Accu zero |

**Silicon-boot-ROM-redirect émulé** (calypso_c54x.c, option A) :
```c
if (s->pc == 0xFF80 && s->sp == 0x1100) {
    s->pc = 0x7120;   /* PROM0 boot init */
}
```
Gardé sur `SP==0x1100` pour ne pas hijacker les walks séquentiels firmware
qui passent légitimement par 0xff80 plus tard.

## Phase 1 — DSP boot init (PROM0 0x7120, insn=1..~50k)

PROM0[0x7120..0x7128] :
```
0x7120  STM #0x5AC8, SP     ← SP = 0x5AC8 ✓ (insn=1)
0x7122  ST  #lk, *(AR0+lk)  ← init data RAM
0x7127  BACC A              ← branch via A
```

`BACC A` avec A=0x0000 → PC=0x0000 (boot ROM stub).

PROM0[0x0000..0x0007F] (stubs synthétiques émulateur) :
```
0x0000  LDMM SP, B   (BA18)
0x0001  RET          (FC00)
```

Le cycle `0x0000 → 0x0001 → RET pop → ...` est la **boot ROM serial loader
emulation** : c'est correct, le DSP attend une commande de chargement.
SP monte progressivement (chaque RET pop +1) jusqu'à passer le contrôle
au vrai firmware via une adresse poussée préalablement sur la pile.

À la fin de cette phase, on a SP propre (~0x5AC8), AR0..AR7 initialisés
via STM (vu BOOT-MMR-WR #5-#16 : AR1=2ae4, AR2=33ed, AR3=2d10, AR4=2c10,
AR5=2c00, AR6=2c82, AR0=0009, BK=0020).

**Plus de boot init MMR à insn=190M.** Le firmware exécute une routine
légitime qui passe sequentially par 0xff80 — sans l'option A, notre
ancien override hijackait ça en faux soft-reset → SP redescendait à
0x5AC8 et tout dérapait. Option A neutralise ce piège.

## Phase 2 — Mainloop ARM + scheduler L1S (~insn 50k..)

Côté ARM (`layer1.highram.elf`) :
1. `main` (0x820190) → init L1 state
2. `l1s_pm_test` (0x825424) → planifie task=1 (PM) via API
3. `l1s_fbsb_req` (0x826778) → quand L23 demande cell selection
4. `l1s_fbdet_cmd` (0x8262cc) → écrit `d_task_md = 5` (FB) dans NDB

Côté DSP : voit `d_task_md=5` apparaître dans API RAM, va exécuter la
routine FB-det.

### Boucle de tâches ARM↔DSP

```
ARM tdma_sched_execute (0x828ef8)
   │
   ├─ écrit d_task_md WP0/WP1  (offset 0x08/0x28 byte)
   ├─ écrit d_task_d / d_task_ra / d_burst_d / d_burst_u
   ├─ déclenche IRQ_API vers DSP via mailbox
   │
   ↓
DSP fetch d_task_md → dispatch dans le code PROM
   │
   ├─ task=1 (PM)   → power measure, écrit a_pm[3]
   ├─ task=5 (FB)   → FB-det, écrit d_fb_det + a_sync_demod[4]
   ├─ task=6 (SB)   → SB demod, écrit a_sch[5] + a_serv_demod[4]
   ├─ task=7+ (CCCH/BCCH/SDCCH/SACCH) → demod NB, écrit a_cd[15]
   ├─ task=24 (TCH) → TCH burst handling
   │
   ↓ pulse IRQ_API back to ARM
ARM consume NDB cells via watcher (calypso_trx.c LATCH-CONSUME)
```

## Phase 3 — FB lock (FB_TASK=5)

**Côté DSP** :
- Entry candidate dans PROM0 (multiple) ; runtime trace montre PC hot dans
  PROM0 0x7e80..0x7ec0 et 0x81a0..0x82ff (FB correlator inner).
- Lit samples I/Q depuis BSP RX zone à DARAM [`CALYPSO_BSP_DARAM_ADDR`] = 0x3fb0
  (donc 0x3fb0..0x3fbe, vérifié par DARAM RD HIST).
- Correle contre porteuse FCCH 67.7 kHz.
- Écrit le résultat dans NDB cells :
  - `d_fb_det` = NDB+0x48 (byte) = DSP word 0x08F8
  - `a_sync_demod[4]` (TOA/PM/ANG/SNR) à NDB+0x4C..0x53

**Côté ARM** :
- `prim_fbsb.c` polle `d_fb_det` à chaque frame
- Quand `d_fb_det != 0` → consume (W1C latch si CALYPSO_W1C_LATCH=1),
  lit a_sync_demod, calcule AFC offset
- Met à jour le state fbsb (`FB0_SEARCH → FB1_SEARCH → SB_SEARCH`)

**Signature d'un vrai FB lock** (run "good" 2026-05-28) :
```
a_sync_demod = (snr=28762 toa=27767 ang=1 pm=3595)   ← ang=1 = pas de dérive
fb0_att=1
d_fb_det LATCH-CONSUME values : 87× 0x1ef8 (dominant)
```

## Phase 4 — SB lock (SB_DSP_TASK=6)

**Pré-requis** :
- FB locké (sync_demod stable, ang ≈ 0)
- ARM bascule `d_task_md = 6`
- DSP doit avoir BRINT0 (IMR bit 5) armé pour recevoir le burst SB

**Côté DSP entries candidats** (PROM0 / PROM3) :
- 0x8167 (dispatcher SB) — à vérifier via PROM3-VISIT probe
- 0x81ff (SCH inner)
- 0x82b8 (a_sch[] writer)

**Écritures DSP attendues** :
- `a_sch[5]` à NDB+0x54..0x5D : résultat SCH decode (BSIC + RFN)
- `a_serv_demod[4]` : signale "je sers ce slot"
- d_fb_det re-armé pour confirmation lock

**Côté ARM** :
- `prim_fbsb.c` consume `a_sch[]`, valide BSIC + computes T1/T2/T3'
- Sur succès → émet `L1CTL_FBSB_CONF` vers L2 (via sercomm DLCI 1)
- Mobile L3 reçoit, lance Location Update

## Phase 5 — CCCH demod (task=7+) [hors scope FBSB]

Une fois SB locké, le DSP démod en continu les NB bursts du CCCH
(positions 2..49 du multiframe 51), écrit dans `a_cd[15]` (NDB+0x1DC ou
0x1FC selon variante DWARF). ARM lit a_cd, dépaquette les RR messages
(SI3, SI4, IMM_ASS, PAGING_REQ), passe à L3.

## Items bloquants identifiés à ce stade (2026-05-28)

| # | Item | État |
|---|---|---|
| 1 | Boot init (SP/IMR/MMR) | ✅ résolu via override option A |
| 2 | Soft-reset loop (override trop large) | ✅ résolu via gating SP==0x1100 |
| 3 | FB lock | ✅ vu dans run good (87× d_fb_det=0x1ef8, snr=28762 ang=1) |
| 4 | SB lock | ❌ jamais observé (sb_att=0 dans tous runs) |
| 5 | BRINT0 (IMR bit 5) | ❓ à vérifier au moment de task=6 via TASK6-IRQ probe |
| 6 | Bridge source : SCH bursts | ❓ BURST-IN catégoriseur dit 96/96 TONAL — vrai ou biais à 1 sample/bit ? |
| 7 | PROM3 dispatch | ❓ jamais atteint dans runs observés (PROM3-VISIT=0) |

## Items déjà résolus

- **DSP opcodes** : 9 mis-decodes fixés (cf CLAUDE.md "Known Fixed Opcode Bugs")
- **Boot init skipped** : option A (override conditionnel runtime)
- **IMR=0 cascade via STH B,*AR4+/AR5+ stale** : résolu par boot init correct
  (AR4/AR5 désormais initialisés à 0x2c10/0x2c00 par STM #lk,ARx)
- **MMR mask & 0x7F** au lieu de & 0x1F (STLM/POPM/PSHM)
- **Stub boot ROM 0x0000-0x007F** : LDMM+RET+FRET fillers
- **0x6F00 family** : ADD/SUB/LD/STH/STL Smem,SHIFT (cf doc/opcodes/0x68_0x6F.md)

## Probes en place pour la suite

| Probe | Site | Cap | Discrimine |
|---|---|---|---|
| BOOT-BRANCH | exec loop (insn≤300k) | 8000 | Path control-flow boot |
| BOOT-MMR-WR | data_write_locked MMR (insn≤300k) | 800 | Reach+effect init STMs |
| DSP-API-WR | data_write_locked API RAM | 400 | DSP→ARM mailbox writes |
| WAIT-A21A | PC=0xa21a | uncapped | DSP spin sur poll flag |
| IMR-W | MMR_IMR writes (ZERO + transitions) | uncapped | IMR pattern dyn |
| SP-Δ / SP-HIST / SP-CATASTROPHE / SP-FLOOR | divers | divers | SP runaway detection |
| SOFT-RESET-TRIG | PC ∈ {0xff80, 0x7120} (insn>100k) | 30 | Re-entry boot via firmware |
| BURST-IN (v2 phase) | calypso_bsp_deliver_buffered | 600 | Catégorie burst à DSP input |
| TASK6-IRQ | ARM TASK WR=6 | 50 | IMR/IFR bit 5 au moment SB demand |
| PROM3-VISIT | PC ∈ {0x8167, 0x81ff, 0x82b8} | uncapped | Dispatch SB-decode reach |
| SILICON-BOOT-REDIRECT | PC=0xff80 && SP=0x1100 | 3 | Compteur trigger redirect |

## Prochaines étapes

1. Relancer post-option-A pour valider que SP reste stable (plus de
   SOFT-RESET-TRIG après #0).
2. Vérifier que FB lock se produit ET reste stable (sans le reset loop).
3. Si FB stable → laisser tourner jusqu'à task=6 → décoder TASK6-IRQ +
   PROM3-VISIT pour discriminer items 5/6/7.
4. Fix le catégoriseur BURST-IN à 1 sample/bit (déjà bien sur fond mais
   risque de biais sur certains patterns).
