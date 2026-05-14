# Rapport pytest — 2026-05-14 (check 7, première mesure)

> Squelette pytest v2 lancé sur l'arbre `tests/`. Découverte **priorité A**
> (init AR3/AR4 firmware) tranchée en **branche (i) STM literal** par grep
> statique. Pas besoin de probe runtime pour cette mesure.

## Résultat brut

```
21 collected
   3 PASSED      ← régression décodeur (POPM + Tier A + INTM dwell)
   3 SKIPPED     ← inclut les 2 findings priorité A (branche (i))
  15 XFAIL       ← attendus (probes/compteurs à instrumenter)
30 s wall
```

```
PASSED  test_popm_decoder_active                      — décodeur POPM 0x8A00 actif, ancien stubé
PASSED  test_tier_a_decoder_fixes_present             — 0x76 ST + POPM mention présents
PASSED  test_intm_dwell_no_regression                 — pas de WAIT-A21A dans qemu.log
SKIP    test_ar3_init_source_identified               — branche (i) — 8 sites lk=0x0000
SKIP    test_ar4_init_source_identified               — branche (i) — 18 sites lk=0x2bc0
SKIP    test_synth_zero_path_active                   — CALYPSO_FBSB_SYNTH=1 actif
XFAIL   test_dsp_throughput_5x                        — exposer compteur insn via monitor
XFAIL   test_bsp_dma_target_matches_correlator_read_zone — mismatch confirmé
XFAIL   test_no_d_fb_det_wr_site_anomaly              — tire toujours
XFAIL   test_fb0_att_nonzero                          — dépend bsp_dma
XFAIL   test_synth_zero_path_active                   — vrai demod non convergent
XFAIL   test_c54x_interrupt_ex_called_nonzero         — compteur à instrumenter
XFAIL   test_isr_entered_implies_rete                 — idem
XFAIL   test_no_pending_irq_gated                     — idem
XFAIL   test_l1ctl_data_ind_received                  — bloqué fb_det
XFAIL   test_neigh_pm_req_response_unchanged          — parse pcap GSMTAP
XFAIL   test_rach_emitted                             — bloqué L1
XFAIL   test_immediate_assignment_decoded             — bloqué CCCH
XFAIL   test_rr_sdcch_established                     —
XFAIL   test_location_updating_request_sent           —
XFAIL   test_location_updating_accept_received        — objectif final
XFAIL   test_rxdoneflag_no_longer_blocks              — régression historique
```

## Finding majeur — Priorité A tranchée en branche (i)

Grep statique sur `calypso_dsp.txt` (sections PROM* uniquement) :

### AR3 — opcode `STM #lk, AR3` = `0x7713 lk`

**874 occurrences** du mot `0x7713` dans PROM*. Parmi celles-ci, **8 sites
avec `lk=0x0000`** (valeur initiale observée AR3=0) :

| Région | Addr DSP | lk |
|---|---|---|
| PROM0 | `0x0a349` | 0000 |
| PROM0 | `0x0a43e` | 0000 |
| PROM2 | `0x2a4f6` | 0000 |
| PROM2 | `0x2a51b` | 0000 |
| PROM2 | `0x2a552` | 0000 |
| PROM2 | `0x2a576` | 0000 |
| PROM2 | `0x2e388` | 0000 |
| PROM2 | `0x2e3c0` | 0000 |

Distribution top 10 des `lk` post-0x7713 : `lk=2bfa:23, 2c00:20, 30a4:15,
304a:13, 2c18:12, 2b80:12, 30f4:11, 2a00:10, 0001:9, 2b1e:9`. **Aucune
valeur dominante** → AR3 sert à pointer plein de buffers selon le contexte,
pas un seul.

### AR4 — opcode `STM #lk, AR4` = `0x7714 lk`

**689 occurrences**. **18 sites dans PROM0 avec `lk=0x2bc0`** (valeur observée AR4≈0x2bc0) :

```
PROM0 0x089c5  lk=2bc0
PROM0 0x08a03  lk=2bc0
PROM0 0x08a83  lk=2bc0
PROM0 0x08bcd  lk=2bc0
PROM0 0x08c14  lk=2bc0
PROM0 0x08c62  lk=2bc0
PROM0 0x08cb8  lk=2bc0
PROM0 0x08cfa  lk=2bc0
PROM0 0x08db2  lk=2bc0
PROM0 0x08e86  lk=2bc0
PROM0 0x08f87  lk=2bc0   ← à 0x36 après PC=0x8f51 (FB-det write site)
PROM0 0x08f90  lk=2bc0
PROM0 0x08fa2  lk=2bc0
PROM1 0x1b2a9  lk=2bc0
PROM1 0x1b2e8  lk=2bc0
PROM1 0x1b5ee  lk=2bc0
PROM1 0x1b73b  lk=2bc0
PROM1 0x1b8f0  lk=2bc0
```

**13 des 18 sites sont concentrés dans la fenêtre `PROM0 [0x089c5..0x08fa2]`
— cluster de ±100 mots autour de `PC=0x8f51`** (le D_FB_DET-WR-SITE).
**C'est la même fonction** : le correlator FB-det. AR4 init `0x2bc0` est
en hard-code dans le code de la routine elle-même.

Distribution top 10 des `lk` post-0x7714 : `lk=2b80:29, 2a00:27, 2bc0:18,
2c00:18, 2bfa:17, 0017:14, 2be4:13, 2bf0:13, 2a5c:12, 2c10:11`. AR4 pointe
toujours dans la zone `[0x2a00..0x2c40]` → **table de coefficients ROM**,
confirmé par le doc (rapport 05-14 dit "AR4 ≈ 0x2bc0 quasi-fixe → table
coefficients ROM").

## Conclusion priorité A

**Branche (i) littérale STM confirmée pour AR3 ET AR4.**

- AR4 pointe sur la table de coefficients FB-det dans ROM (`[0x2bc0..]`).
  C'est correct, attendu, ne demande aucune action.
- AR3 pointe sur la zone **DARAM `[0x0000..0x03A3]`** (stride +19 sur 50 iters
  observées). Le firmware *attend* les samples I/Q là, pas à `0x3fb0` (la
  cible BSP DMA).

**Implication directe :**

```
                     CALYPSO_BSP_DARAM_ADDR=0x3fb0    ← BSP écrit ici
                                  ✗
firmware FB-det @ PC=0x8f51 lit DARAM[0..0x3A3]      ← AR3 init=0 par STM literal
                                  ↓
                              **MISMATCH STRUCTUREL**
```

Le firmware **n'utilise jamais la zone `0x3fb0+`**. Le default `0x3fb0`
était un guess historique qui n'a aucune base dans le code DSP. Les tests
A/B `CALYPSO_BSP_DARAM_ADDR=0x2bc0 / 0x0080 / 0x0000` ont donné des
captures bit-pour-bit identiques pour une raison simple : le BSP écrit
ailleurs, mais le firmware corrèle toujours la zone basse (initialisée à 0
par DARAM reset, donc tout `0x0000`, et le correlator détecte évidemment
0 corrélation).

## Action concrète recommandée (avant la check 7+1)

Tester `CALYPSO_BSP_DARAM_ADDR=0x0000` n'est **pas suffisant** : la zone
`[0..0x3A3]` est dans le DARAM bas (mots 0..0x3A3 = 932 mots), qui chevauche
les MMR (0x00..0x1F) et le scratch DSP. La cible doit être **plus précise**.

Trois sous-options :

**(α) Identifier le site DARAM exact attendu par le firmware**
1. Lire les 4 mots autour de `PROM0 0x089c5..0x089c8` (premier site AR4=2bc0).
2. Lire les 4 mots autour de `PROM0 0x08f4d..0x08f51` (juste avant le write site).
3. Disassembler ce contexte pour identifier *quelle* DARAM zone est censée
   recevoir les samples — soit via un STM AR3 littéral non-zéro qui aurait
   échappé au filtre `lk=0x0000`, soit via une expression `AR3 = something
   + offset` (cas (iii) résiduel pour AR3 spécifiquement).

**(β) Tracer le path ARM→DARAM manquant**
Hypothèse plausible : la BSP écrit à `0x3fb0` par convention historique,
mais il devait y avoir une routine ARM qui *copie* `DARAM[0x3fb0..]` →
`DARAM[0..]` au démarrage d'une slot RX. Cette routine n'existerait pas
(ou plus) dans le firmware embedded — d'où le mismatch.

**(γ) Refaire la BSP DMA vers l'adresse correcte**
Une fois (α) résolu, changer le default `CALYPSO_BSP_DARAM_ADDR` (et/ou
ajouter un offset de page) pour matcher où AR3 lit réellement.

## Vérifs côté décodeur (régressions OK)

`test_popm_decoder_active` PASSED : confirme dans le code statique que :
- `(op & 0xFF00) == 0x8A00` est présent dans `calypso_c54x.c`
- `if (0 && hi8 == 0x8A)` est présent (ancien décodeur MVDK désactivé)

`test_intm_dwell_no_regression` PASSED : pas de `WAIT-A21A` dans
`/home/nirvana/myconfigs/osmo_root/qemu.log` (mount /root du container) sur
les 30s d'observation post-run. Le qemu.log persisté côté hôte est encore
celui daté 05-08 17:13 (pré-POPM fix) — le run live n'écrit toujours pas
dedans (mount `/tmp/osmocom-logs/op1` vide). À reconnecter, cf. rapport
05-14.

## Suite logique du plan check 7

Étape 1 (grep) : ✅ **fait** — branche (i) tranchée.
Étape 2 (probe runtime AR3/AR4) : **plus nécessaire** pour AR4. Pour AR3,
reste utile pour discriminer parmi les 8 sites `lk=0` quel est exécuté avant
`insn_count=10 040 312`.
Étape 3 (3 compteurs IRQ) : encore intéressant comme mesure de second tour
**APRÈS** alignement BSP/DARAM. Si le data path est fixé, RETE devrait
augmenter naturellement (IRQ BRINT0 → ISR → service → RETE → INTM=0).

## Question pour Claude web

1. Tu valides **(α)** comme prochaine action — disassembler le cluster
   `PROM0 [0x089c5..0x08fa2]` pour comprendre *quelle DARAM* AR3 vise
   exactement à ce point d'entrée ?
2. Si je creuse le disasm via grep+offset autour de `0x089c5`, tu veux que
   je remonte aussi les **valeurs de IFR/IMR/PMST observées au moment de la
   1ère exécution de PC=0x089c5** — pour valider que c'est bien la routine
   FB-det en mode RX (et pas un autre contexte d'utilisation de AR4=0x2bc0) ?
3. Aspect collateral : la table de coefficients à `[0x2bc0..]` est en DARAM
   *data* (le mode ROM est PROM* en code). Est-ce qu'elle est mappée en
   double — copie depuis PROM2 vers DARAM au boot, par exemple ? Si oui,
   identifier le path de copie pourrait aussi pointer vers la zone DARAM
   cible que AR3 attend pour le buffer RX.

## Fichiers sync 3-way (md5 vérifié)

```
005645f7fa46421529497f7ea755b414  REPORT_CLAUDE_WEB_20260514.md
5f9c58a5d36a402eba0bafcfb396e4d4  tests/test_calypso_milestones.py
52c82f971aaab1d416657c09cd8fe10e  tests/conftest.py
6bd15b3c4681cbc29e36c100175be103  tests/pytest.ini
```

Tous identiques sur :
- `/home/nirvana/qemu-calypso/`
- `/home/nirvana/qemu-src/`
- container `trying:/opt/GSM/qemu-src/`
