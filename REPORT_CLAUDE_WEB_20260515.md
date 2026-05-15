# Rapport Claude web — 2026-05-15 (root cause + fix DSP→ARM mirror)

> Session 2026-05-14 nuit → 2026-05-15 après-midi (~16h). Bug racine de
> ~6 mois identifié et fixé en 5 lignes. 7 milestones bascule au harnais.

## TL;DR

**Bug racine** : `calypso_dsp_read()` lisait depuis `s->dsp_ram[]`
(array embarqué dans CalypsoTRX) tandis que `calypso_dsp_write()` mirrorait
ARM→DSP via `s->dsp->data[]`. **Le sens DSP→ARM n'avait pas de mirror** —
toutes les écritures DSP au NDB (d_fb_det, a_sync_demod, a_cd[], etc.)
étaient invisibles aux reads ARM.

Conséquence : le check firmware `if (!d_fb_det)` voyait toujours 0 →
FBSB_CONF=FAIL en boucle → mobile coincé en cell-search depuis la mise en
place du dual-buffer.

**Fix** (5 lignes dans `calypso_trx.c:163`) : faire que `calypso_dsp_read`
lise depuis `s->dsp->data[offset/2 + 0x0800]`. Plus 6 lignes pour aligner
le synth FBSB sur la même source. Total : **11 lignes**.

**Effet observé** : `task_md=24` (DSP_TASK_ALLC = CCCH demod) passe de **0**
à **20+** sur un run de quelques minutes — première fois en mois que le
mobile atteint le CCCH demod via le real DSP path.

## Trajectoire de la session

### Hier matin (2026-05-14)

Rapport 05-14 disait : "blocker BSP DMA n'écrit pas où le correlator lit",
"D_FB_DET-WR-SITE révèle mismatch source/sink, AR3/AR4 init firmware-imposed,
correlator DSP lit DARAM `[0..0x3A3]` linéaire + wrap `[0xfc5d..]` BK=176".

### Décomposition pendant la session — diagnostic invalidé en 4 étapes

| Étape | Hypothèse | Verdict |
|---|---|---|
| 1 | BSP DMA target faux (0x3FB0 vs zone correlator) | **Invalidé** — `CALYPSO_BSP_DARAM_ADDR=0x0080` ne change rien fonctionnellement |
| 2 | AR1 lit table coeffs ROM (pas samples) | **Confirmé bit-pour-bit** : `data[AR1]` = `fff6/8fd7/d9ec/bbef` = `PROM0[0x0019..0x001c]` |
| 3 | Convergence DSP timing | **Validé** : `data[AR0/AR2]` passe 0%→50%+ sample-valides après 3 min runtime |
| 4 | Compute marche, problème ailleurs | **Confirmé** : 14 D_FB_DET SET events observés avec vraies valeurs (0xbd2e, 0x2014, 0x3bb6 depuis PC=0x8217 `STH`) |

### Le pivot

Le rapport disait "data path cassé" → en fait **le data path marche**.
Le compute DSP produit des résultats structurés et les écrit dans NDB.
**Mais ARM ne les lit pas** parce que le mirror DSP→ARM est absent.

Découvert via instrumentation `ARM RD d_fb_det` (sondage côté
`calypso_trx.c::calypso_dsp_read`) : 192 reads, 187 valent `0x0000` malgré
14 writes DSP non-zéro pendant la même fenêtre.

## Le bug en détail

`include/hw/arm/calypso/calypso_trx.h:48` :
```c
uint16_t dsp_ram[CALYPSO_DSP_SIZE / 2];  // dans CalypsoTRX
```

C'est un array **séparé** de `s->dsp->data[0x10000]` dans `C54xState`.

`calypso_trx.c:242 calypso_dsp_write()` (ARM→DSP) — déjà correct :
```c
// Ligne 246 — écrit dsp_ram (legacy ARM-side)
s->dsp_ram[offset/2] = value;
// Ligne 258 — mirror vers dsp->data (DSP-side) ✓
s->dsp->data[dsp_word] = (uint16_t)value;
```

`calypso_trx.c:159 calypso_dsp_read()` (DSP→ARM) — **bugué** :
```c
// Ligne 163 — lit dsp_ram[] qui n'est jamais sync'd depuis dsp->data
uint64_t val = s->dsp_ram[offset/2];
```

**Asymétrie complète** : ARM writes propagent vers DSP, mais DSP writes
restent invisibles à ARM.

## Le fix

`calypso_trx.c:163` :
```c
uint16_t *src = (s->dsp && s->dsp->data)
                ? &s->dsp->data[offset/2 + 0x0800]
                : &s->dsp_ram[offset/2];
uint64_t val = (size == 2) ? src[0] :
               (size == 4) ? ((uint32_t)src[0] | ((uint32_t)src[1] << 16)) :
               ((uint8_t *)src)[offset & 1];
```

Et `calypso_trx.c:397` (synth init) :
```c
uint16_t *ndb_target = (s->dsp && s->dsp->data)
                       ? &s->dsp->data[0x0800]
                       : s->dsp_ram;
calypso_fbsb_init(&g_fbsb, ndb_target, 0x0800);
```

Le synth FBSB cell_wr écrit maintenant directement dans `dsp->data`, cohérent
avec le read path.

## Validation empirique

### Run avant fix (CALYPSO_FBSB_SYNTH=0)

```
task_md=5 (FB-det)  : 782    ← retries massifs
task_md=1 (NB)      : 195
task_md=24 (ALLC)   : 0      ← jamais atteint
FBSB_CONF (0x02)    : 193    ← FAIL en boucle
DATA_IND (0x03)     : 0
ARM RD d_fb_det     : 187/192 = 0x0000 (97.4%)
                      5 reads non-zéro (= ARM lit ses propres writes)
Mobile              : C6, MM idle, no cell available
```

### Run après fix DSP→ARM mirror (CALYPSO_FBSB_SYNTH=0)

```
task_md=5           : 680    ← drop relatif
task_md=1           : 169
task_md=24 (ALLC)   : 20     ← FIRE pour la 1ère fois !
FBSB_CONF (0x02)    : 169
DATA_IND (0x03)     : 0      ← prochain layer
ARM RD d_fb_det     : 5 non-zéro values vues (0xc94a, 0xc92e, 0x5e2a, ...)
Mobile              : C6 (mais transitoirement passe en CCCH demod)
```

### Run après fix DSP→ARM mirror + fix synth target (CALYPSO_FBSB_SYNTH=1)

```
task_md=24 (ALLC)   : 34     ← ×1.7 plus haut, déterministe
task_md=6 (SB)      : 6
task_md=5           : 6      ← drop énorme (synth bypass FB-det)
task_md=1           : 3
ARM RD d_fb_det     : 6/6 = 0x0001 (100% synth value visible)
DATA_IND (0x03)     : 0      ← reste bloqué — bug en aval
FB0_FOUND (synth)   : firing  ← synth path fonctionne
fb0_att (firmware)  : 0 toujours  ← le firmware accepte FBSB sans incrémenter ?
```

## Cascade débloquée jusqu'où

```
DSP FB-det compute              ✓ converge
DSP écrit d_fb_det non-zéro     ✓ (14 SET events sur run de référence)
ARM lit d_fb_det                ✓ NOUVEAU — voit vraies valeurs
Firmware FBSB success           ✓ NOUVEAU — réussit sans synth
mframe_enable(MF_TASK_CCCH)     ✓ NOUVEAU — armé
task_md=24 (DSP_TASK_ALLC)      ✓ NOUVEAU — fire 20-34 fois/run
ARM lit a_cd[]                  ? (fix devrait s'appliquer aussi)
Firmware envoie L1CTL_DATA_IND  ✗ STILL 0
Mobile L23 décode SI            ✗ stuck en cell-search C6
```

Le prochain blocker est dans le path **CCCH demod result → DATA_IND**. Soit :
- `prim_rx_nb.c::l1s_nb_resp` bail sur `db_r->d_burst_d != burst_id` (DSP
  n'écrit pas la séquence 0→1→2→3 — déjà observé)
- DSP a_cd[] writes n'ont pas le bon contenu en mode ALLC
- Un autre check firmware avant `l1_queue_for_l2(rxnb.msg)`

## Milestones bascule au harnais pytest (en 36h)

1. `test_d_fb_det_data_no_longer_zero` — convergence data path
2. `test_bsp_dma_target_matches_correlator_read_zone` — BSP target reach
3. `test_no_d_fb_det_wr_site_anomaly` — sweep FB-det nominal
4. `test_a_cd_writes_nonzero` — DSP CCCH écrit a_cd[]
5. `test_a_cd_write_pc_includes_ccch_demod` — cluster CCCH demod identifié
6. **`task_md=24` fire pour la 1ère fois sans synth** — fix DSP→ARM mirror
7. **`task_md=24` déterministe avec synth+fix** — 34+ fires en 21s

## Histoire du diagnostic — décomposition propre

On a éliminé hypothèse après hypothèse :
- Convergence DSP — débloquée par patience (3 min uptime)
- Seuil A vs threshold — pas un seuil, juste `if (!d_fb_det)` binaire
- Overwrite parasite immédiat — non, deltas SET→CLEAR de 7-40ms
- Opcode Tier B mal décodé — non, addresses correctes, valeurs cohérentes
- MMIO timing — pas un timing, juste les deux arrays distincts non sync

Pour atterrir sur **"deux arrays distincts non synchronisés en lecture"**.
Le fix est trivial *parce que* le diagnostic est précis. Pattern identique
au POPM fix 05-08.

## Pour la check 8

**Direction prochaine** : creuser le path CCCH demod → DATA_IND avec
l'instrumentation déjà en place. Avec synth=1+fix on a maintenant un
**régime déterministe** pour l'investigation : pas de variance amont.

Vérif à faire :
1. ARM read a_cd[] (DSP word 0x09D0+) — voit-il les writes DSP correctement
   maintenant que le mirror est fixé ?
2. `db_r->d_burst_d` (0x0829 / 0x083D) — DSP fait-il la séquence 0,1,2,3 ?
3. Le check `if (dsp_api.db_r->d_burst_d != burst_id) return 0;` dans
   prim_rx_nb.c — bail-il toujours ?

Bonus probe envisagée : `A_CD-WR-COMPLETE` sur write a_cd[14] (dernière
cell) — capture l'état complet du buffer au moment où DSP signale fin de
demod. Si ARM lit immédiatement après, on saura si la séquence est
synchronisée.

## Diff total du fix

```diff
 hw/arm/calypso/calypso_trx.c | +26 lignes (11 fix + 15 commentaires)
```

Le commit primaire est de 5 lignes (read fix). Le commit secondaire est
de 6 lignes (synth target). Le reste = commentaires + instrumentation.

## Question pour toi (Claude web)

1. **L'asymétrie write-mirror/read-no-mirror** est-elle un pattern classique
   dans les émulateurs QEMU dual-buffer, ou une régression spécifique
   à ce projet ? Si pattern, est-ce une class d'erreurs à grepper
   ailleurs (calypso_uart, calypso_inth, etc.) ?

2. Le prochain blocker DATA_IND=0 — vu que la chaîne arrive jusqu'à
   `task=24` × 34 fois sans aucun DATA_IND, et que `prim_rx_nb` a un
   check rigide `db_r->d_burst_d != burst_id` qu'on a vu fail
   systématiquement (DSP écrit val=0 majoritaire, pas séquence 1,2,3) —
   tu vois lequel des 3 attaquer en premier :
   - (a) Forcer la séquence d_burst_d côté QEMU (autre patch type "mirror")
   - (b) Comprendre côté firmware/DSP pourquoi la séquence n'est pas faite
     (différent en mode ALLC vs mode NB pur ?)
   - (c) Skip le check dans prim_rx_nb par un autre env-gated workaround

3. La variance run-to-run qu'on observe (parfois task=24=20, parfois 0) —
   c'est cohérent avec icount=off (timing chaotique). Tu validerais une
   tentative d'icount=auto avec un mode spécifique qui pourrait marcher,
   ou c'est sans espoir vu que le TDMA bridge dépend du wall-clock ?
