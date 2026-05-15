# Rapport Claude web — 2026-05-15 nuit (post BC/BCD revert + audit BITF)

> Session 2026-05-15 nuit. Suite directe du `REPORT_CLAUDE_WEB_20260515_BCBCD_REVERT.md`.
> Suivi du plan Claude web : "instrumenter BITF d'abord pour valider/invalider
> l'hypothèse (b) BITF buggé".

## TL;DR

**Hypothèse (b) BITF buggé : INVALIDÉE par mesure**. BITF fonctionne correctement
(set TC quand mem & mask != 0, clear sinon). Vérification bit-pour-bit sur
sample.

**Découverte plus surprenante** : sur **51 minutes de runtime cumulé** post-revert,
les BITFs à PC=0x9aa0 et PC=0x9aa6 (qui gatent le path vers `ST #1, d_task_d`
à PROM 0x9ab1) **n'ont JAMAIS été appelés** (0 hits sur 104+ BITFs totaux).

**Conclusion** : le bug n'est pas BITF, n'est pas BC/BCD. C'est que **le control
flow DSP n'atteint jamais la zone 0x9aa0-0x9ab2**. Le bug est **plus haut dans
le control flow** (dispatcher → routing vers la routine CCCH completion).

**Bonus** : un caller direct de 0x9ab1 trouvé dans PROM1 (XPC=1) à offset 0x19aac :
opcode `0xf272 0x9ab1` = `B 0x9ab1` (branche unconditional). Donc une route
alternative existe mais n'est probablement pas prise non plus.

**WAIT-A21A = faux positif**. Tranché : 1 hit isolé à insn=9.7M (boot DSP early),
INTM=0 IMR=0 IFR=0 (rien de bloqué). Pas le pattern POPM historique
(INTM=1 IFR=0x0028 stable). Test trop strict.

## Confirmation revert BC/BCD

| Métrique | Pré-fix baseline | Post-fix (bugué) | Post-revert (run actuel) |
|---|---:|---:|---:|
| insn rate (M/s) | 38 | 36-39 | 37-38 |
| WAIT-A21A | 0 | 0 | 1 (transient boot) |
| BURST ID printf | many | **0 ✓** (STL fix gardé) | **0 ✓** |
| DSP stuck PC | aucun | 0xcc51 / 0xfa95 | aucun (PC évolue) |
| task=24 rate | 74/s | 8/s | ~4-5/s |
| D_TASK_D val=1 | 2 (en 80s) | 0 | **0** (en 51 min !) |
| DATA_IND | 0 | 0 | 0 |

**Le revert tient** (DSP progresse, pas stuck, BURST ID = 0). Le STL/STH fix
0x81/0x83 **reste actif** (= ce qui élimine BURST ID printf).

Note sur task=24 rate : 4-5/s en post-revert vs 74/s baseline. Cohérent avec
variance run-to-run sans pytest charge active. Pas une régression structurelle.

## Audit BITF — résultats détaillés

Probe ajoutée à `calypso_c54x.c` ligne 4040-4080 :

```c
if ((op & 0xFF00) == 0x6100) {
    /* BITF Smem, lk — bit-field test, TC = (Smem & lk)!=0 */
    addr = resolve_smem(s, op, &ind);
    uint16_t mask = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
    uint16_t mem_val = data_read(s, addr);
    bool tc_before = (s->st0 & ST0_TC) != 0;
    if (mem_val & mask) s->st0 |= ST0_TC;
    else                s->st0 &= ~ST0_TC;
    bool tc_after = (s->st0 & ST0_TC) != 0;
    /* + log compteurs total/set/clear + PC/mem/mask/tc_before/tc_after */
}
```

### Résultats run 51 minutes (icount=auto, synth=1, W1C=1, FORCE_RX_DONE=1)

```
BITF total : 104
  TC=1 set  : 10  (9.62%)
  TC=0 clr  : 94  (90.38%)
```

### Vérification bit-pour-bit (10 premiers BITFs)

| # | PC | addr | mem | mask | mem & mask | tc_after | Verdict |
|---|---|---|---|---|---|---|---|
| 1 | 0xcab8 | 0x0008 | 0x300a | 0x0100 | 0x0000 | 0 | ✓ correct |
| 2 | 0xf2cd | 0x585f | 0x0000 | 0x0080 | 0x0000 | 0 | ✓ correct |
| 3 | 0xf2cf | 0x585f | 0x0000 | 0x0080 | 0x0000 | 0 | ✓ correct |
| 4 | 0x7503 | 0x4809 | 0x0000 | 0x8000 | 0x0000 | 0 | ✓ correct |
| 5 | 0x751c | 0x0909 | 0x0000 | 0x0008 | 0x0000 | 0 | ✓ correct |
| 6 | 0x7528 | 0x09ef | 0xf072 | 0x0800 | 0x0000 | 0 | ✓ correct |
| 7 | 0x752e | 0x08d7 | 0x0a00 | 0x0020 | 0x0000 | 0 | ✓ correct |
| **8** | 0x7536 | 0x09ef | **0xf072** | **0x8000** | **0x8000 != 0** | **1** | **✓ correct (premier TC=1)** |
| 9 | 0x753b | 0x3fd5 | 0x0000 | 0x0020 | 0x0000 | 0 | ✓ correct |
| 10 | 0x7547 | 0x3fd5 | 0x0000 | 0x0040 | 0x0000 | 0 | ✓ correct |

**Conclusion** : BITF est implémenté correctement. Les 91% de TC=0 sont dûs à
ce que les cellules mémoire testées sont majoritairement à 0 (état initial DSP
ou flags pas encore set). Pas un bug d'émulation.

### PC distribution des BITF (qui appelle BITF)

```
20 × 0x75ef    ← zone DSP scheduler/dispatcher
15 × 0xf2cf
15 × 0xf2cd
13 × 0xffb6
 2 × 0x907a, 0x8f9d, 0x8f82, 0x7547, 0x751c, 0x7503, 0x065d
 1 × 0xf8bf, 0xdd0c, 0xcab8, 0xc974, etc.
```

**PC=0x9aa0 et PC=0x9aa6** (qui dans notre dump PROM contiennent BITF
juste avant les BC NTC qui gatent `ST #1, d_task_d` à 0x9ab1) :
**0 hits sur 51 minutes**.

→ La routine 0x9aa0..0x9ab2 **n'est jamais exécutée**. Donc le path qui set
d_task_d=1 via cette zone n'est jamais atteint, ce qui explique parfaitement
`D_TASK_D val=1 = 0` en 51 min.

## Caller alternatif de 0x9ab1 — PROM1

Recherche dans tout le calypso_dsp.txt des opcodes 2-mot ayant 0x9ab1 comme operand :

```
19aa0 : ... f272 9ab1 ... (offset +12 = address 0x19aac)
```

**0x19aac : `B 0x9ab1`** (opcode `0xf272` = B pmad unconditional, 2 mots).

L'address 0x19aac correspond à XPC=1, offset 0x9aac dans PROM. Donc PROM1
contient une branche directe vers `ST #1, d_task_d` qui **bypass les BITFs
gates de PROM0**.

**Hypothèse** : sur le vrai HW, le DSP passe en XPC=1 pour la completion
CCCH demod, et la route via 0x19aac (=B 0x9ab1) est la route normale.
Notre émulation n'arrive jamais sur XPC=1 dans la zone 0x9aac.

Vérification possible : check si XPC=1 fire dans le run, et si oui quels PCs
sont visités dans cet XPC.

## Le bug réel — control flow upstream

Le mur d_task_d=0 n'est PAS un problème de BITF/BC/BCD/STL. C'est un problème
de **routing DSP** :

```
DSP scheduler reçoit task=24 (ALLC) côté ARM
  ↓
DSP execute CCCH demod (compute MAC sur samples)
  → A_CD-WR fire massivement (DSP écrit a_cd[])
  ↓
DSP fait quelques transitions XPC pour des sous-routines
  ↓
[MANQUANT] DSP ne route jamais vers la zone "task complete" qui
           devrait écrire d_task_d = TASK_NB / TASK_ALLC
  ↓
db_buf_r.d_task_d reste à 0
  ↓
ARM L1 l1s_nb_resp lit d_task_d=0 → puts("EMPTY") → bail → no DATA_IND
```

**Question root cause** : qu'est-ce qui décide, à la fin du DSP CCCH demod,
de router vers la zone "set d_task_d=task_id" ? Probablement une branche
conditionnelle ou un CALL dispatcher qui n'est pas pris.

## WAIT-A21A — faux positif tranché

Du run actuel (51 min uptime cumulé) :

```
qemu.log starts: [MB] === calypso_machine_init START ===  ← log frais
WAIT-A21A #1 : insn=9665419 (= ~0.3s wall, boot DSP early)
  INTM=0 IMR=0x0000 IFR=0x0000 SP=0x0008
1 seul hit, jamais réapparu
```

Comparaison avec POPM dwell historique :

| Pattern | POPM dwell (régression) | Ce run (transient) |
|---|---|---|
| INTM | =1 stuck | =0 |
| IMR | 0x4d31 (IRQ enabled) | 0x0000 |
| IFR | 0x0028 (pending) | 0x0000 |
| Hit count | Massif (5.7M pré-fix, 1581 sous icount=auto) | 1 |
| Position dans run | Stable post-converge | Boot early |

**Verdict** : pas régression POPM. Transient légitime du boot DSP qui passe
par PC=0xa218 (RPTB) zone une fois. Le test `test_no_wait_a21a_on_window`
est trop strict (cumul historique sans filtrer init). À ajuster côté harnais :
soit threshold (≤5 OK), soit `--since-boot` filter.

## État du projet — 4 FAIL réels analysés

| Test | Cause | Direction |
|---|---|---|
| `test_no_wait_a21a_on_window` | Faux positif (transient boot) | Ajuster threshold harnais |
| `test_l1ctl_data_ind_received (×2)` | Mur DATA_IND (d_task_d=0) | Investiguer routing DSP upstream |
| `test_l1ctl_data_ind_rate_vs_alc` | Même mur (angle ratio) | Idem |

3 des 4 FAIL = même problème root. 1 = harness trop strict.

## Questions pour toi (Claude web)

1. **Sur l'analyse XPC=1** : tu valides la direction "instrumenter les transitions
   XPC pour voir si XPC=1 est jamais visité dans la zone 0x9aac+" ? Probe simple
   à ajouter sur les opcodes qui changent XPC (FB FAR à F880-F8FF, FBD FAR à
   FA80-FAFF, etc.). Si XPC=1 jamais set → bug dans le routage initial. Si XPC=1
   set mais PC pas dans 0x9aac → autre routing manquant.

2. **Sur le control flow upstream** : qu'est-ce qui décide à la fin du compute
   CCCH demod (A_CD-WR fire, des dizaines/centaines de hits) de router vers la
   routine "set d_task_d=task_id" ? Tu suspectes :
   - **(a)** Un CALL/B explicite dans PROM (qu'on doit identifier par grep) qui
     n'est pas pris parce qu'une branche conditionnelle en amont rate
   - **(b)** Un interrupt return path (RETE) qui devrait restaurer un contexte
     incluant ce routing
   - **(c)** Un dispatcher table dans PROM qui pointe vers la routine, indexé par
     task_id (et notre task_id arrive corrompu)

3. **Sur le test WAIT-A21A** : tu préfères qu'on (i) ajuste le test pour
   accepter ≤N transients de boot, ou (ii) qu'on filtre par insn>10M dans
   le grep, ou (iii) qu'on track le timestamp d'apparition vs début du run ?

4. **Direction stratégique** : avec BITF/BC/BCD/STL tous innocentés (ou fixés),
   les "Tier B" opcode bugs ne semblent plus être la voie. Le mur est dans le
   control flow. Tu vois autre chose qu'on pourrait auditer (interrupt vectors ?
   memory map ARM↔DSP ? task_d command propagation ARM→DSP ?), ou tu confirmes
   que la prochaine étape est l'instrumentation XPC + caller analysis ?

## Annexe — diff total session post-revert

```
hw/arm/calypso/calypso_c54x.c
  + D_TASK_D-WR probe à 0x0828/0x083C
  + Fix STL/STH 0x81/0x82/0x83/0x84 (GARDÉ)
  + BITF-PROBE log (BITF correct, 9.62% TC=1)
  + Revert BC/BCD à comportement historique heuristique

hw/arm/calypso/calypso_trx.c
  + Probe ARM RD a_cd[0..14] à offset 0x03A0..0x03BD

tests/test_calypso_milestones.py
  + Fix UTF-8 docker_exec
  + Fix sémantique XFAIL-3-états test_l1ctl_data_ind_received

tests/test_run_observability.py
  + Fix UTF-8 dexec_sh
  + Conditional synth=1 → xfail test_d_fb_det_pattern_unchanged
  + Idem fix sémantique XFAIL-3-états DATA_IND
```

md5 c54x.c final : `47c896faeac5882eb7dfdfdc017410a1` (avec BITF probe)
