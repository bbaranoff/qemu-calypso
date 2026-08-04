# Rapport — `d_fb_det = 0` en mode natif

> **⚠️ RÉVISION MAJEURE du 2026-07-27, soir.** La version précédente de ce rapport
> (enquête multi-agents, 11 agents) désignait comme cause racine le reroute
> `CALYPSO_FB_ENERGY`. **Cette conclusion est retirée** : elle reposait sur des
> mesures prises alors que le DSP **n'exécutait pas son firmware**, et sur un test
> portant sur une cellule hors API RAM. Le détail de ce qui tombe est au §6.
>
> Ce qui suit est établi **par l'exécution** (traces d'instructions, désassemblage
> PROM, backtrace gdb), pas par lecture de table ni par conjecture.
>
> Voir aussi `run_results.md` (mesures chiffrées, règles de décision, reproduction).

---

## 1. LE FAIT CENTRAL — le mode natif n'avait jamais tourné

`calypso.env:102` pose `CALYPSO_DSP=c54x` **par défaut**. Or `shunt_route_c54x()`
teste exactement cette valeur, et suffit à mettre `g_shunt.active = true` — **même
avec `CALYPSO_DSP_SHUNT=0`**. Tout ce qui a été mesuré « en natif » jusqu'ici l'a donc
été avec le shunt armé.

Et désarmer le shunt était impossible : **QEMU crashait au boot**. Quatre défauts, tous
de la même famille — *du code emprunté par le chemin natif qui dépend d'un état
alimenté uniquement par le shunt* :

| # | Défaut | Symptôme | Preuve | Correctif |
|---|---|---|---|---|
| 1 | `calypso_dsp_shunt_wp_burst_write()` appelé **sans garde** depuis `calypso_dsp_write()` (MMIO natif) → `dma_memory_write(g_shunt.as = NULL)` | **SIGSEGV à +0,054 s**, pendant le boot SIM | backtrace gdb : `address_space_write as=0x0`, `physmem.c:2972` | garde `!g_shunt.active` + 4 gardes `!g_shunt.as` dans les helpers |
| 2 | early-boot c54x gaté sur `shunt_route_c54x()` → en natif `c54x_reset()` écrase la commande bootloader de l'ARM | DSP en **boucle de park** `0xb41c/0xb41f/0xb424/0xb427` — **firmware L1 jamais chargé** | `data[0x0fff]=0x0001` (IDLE) en boucle ; `BRANCH-TRACE` 100 % dans la boucle | gate sur `CALYPSO_DSP_RUN_C54X` seul |
| 3 | `calypso_dsp_shunt_get_task_md()` renvoie `g_shunt.d_task_md`, posé par `shunt_latch_task()` uniquement | renvoie 0 en permanence hors shunt → **tous ses appelants morts** | `BSP-DISPATCH-FB` = 0 tir malgré la gate active | fallback API RAM `0x0804`/`0x0818` |
| 4 | `g_canned` reste `CAN_DEFAULT` hors shunt (`shunt_parse_canned()` est **après** l'early-return) | `CALYPSO_SHUNT_NO_CANNED` **sans aucun effet** en natif | ligne `CALYPSO_CANNED (dette…)` = 0 occurrence | latent, non corrigé |

**Conséquence à retenir : aucune mesure « native » antérieure à ce soir n'est
interprétable.** Ce n'est pas une opinion — le processus mourait à 54 ms, et quand il
survivait, le DSP tournait la boucle de park de son bootloader.

*(Piège associé : 2,7 milliards d'instructions DSP avaient été relevées comme preuve que
« le DSP tourne enfin ». C'était la boucle de park. Un compteur d'instructions ne prouve
pas qu'un firmware s'exécute.)*

---

## 2. RANK3 — TRANCHÉ, sur preuve d'exécution

**La cellule de dispatch de la tâche FB est `data[0x43d8]`**, chargée en **adressage
absolu** — ce n'est ni un index calculé, ni `0x4387`, ni `0x43c0` :

```
0xb01c:  10f8 43d8        LD *(0x43d8), A
```

Scan mot-à-mot des **4 banks** de la PROM : exactement **2** références à `0x43d8` —
le lecteur ci-dessus, et **un unique installateur** :

```
0xbb00:  76f8 43d8 ab38   ST #0xab38, *(0x43d8)
```

Et `0xab38` commence par `fc00` = `RET`. **Le handler de la tâche FB est un stub qui
retourne immédiatement** — confirmé à l'exécution : `0xb01c → 0xab38 → 0xb01f`.

### Chaîne complète, mesurée
```
ARM d_task_md=5 (FB_DSP_TASK, ×22)
  → vec 0x00f0 (vec28)  →  scheduler 0x7234  →  prologue ISR 0x013b
  → 0xa4e4  →  dispatcher 0xb0xx  →  LD *(0x43d8)  →  0xab38 = RET
  ✗ la routine résultat FB n'est jamais atteinte
```
`fb0_att ≈ 190`, `snr/toa/ang/pm = 0`.

> ⚠️ **[2026-08-03]** ce paragraphe citait aussi `fb0_ret = 0`. C'était un
> **compteur mort** (déclaré, remis à 0, imprimé, jamais incrémenté) : il valait 0
> quoi qu'il arrive et n'étayait rien. Retiré ici et dans les cinq autres endroits
> qui le citaient. Cf. `doc/ETAT_ACTUEL.md` §14.3.

---

## 3. STRUCTURE DE LA ROUTINE FB (désassemblée)

```
0x76fb:  f272 7700    BD 0x7700          entrée
0x7702:  f274 75e8    CALLD 0x75e8
0x7707:  fe00                            fin du préambule
0x7708:  81f8 3fb2    STH A, *(0x3fb2)   <<< LE CORPS COMMENCE ICI
0x770a:  f074 770d    CALL 0x770d
0x7720:  107e         LD dma(0x7e), A    ┐
0x7721:  f010 0004    SUB #4, A          ├ LA GARDE (veut 4)
0x7723:  f844 7729    BC 0x7729 si ≠0    ┘
0x7725:  f074 795f    CALL 0x795f        corrélation + publication
0x798c:  76f8 08fd 4000  ST #0x4000, *(0x08fd)   SNR — INCONDITIONNEL
0x79e3:  fc44         XC (conditionnel)
0x79e4:  69f8 08f8 0001  ORM #0x0001, *(0x08f8)  ← LE PUBLISHER de d_fb_det
0x778a:  68f8 08f8 fffe  ANDM #0xfffe, *(0x08f8) ← le clear apparié
```

**`0x7708` n'a aucun appelant dans les 4 banks** (aucune référence précédée de `f074`
`CALL` ou `f272` `BD`) : le corps n'est atteignable que **par chute** depuis `0x7707`.
L'entrée légitime est donc `0x76fb`.

---

## 4. VALIDATION PAR BÉQUILLE — ce qu'elle prouve et où elle s'arrête

`CALYPSO_BSP_DISPATCH_FB=1` installe une cible dans `0x43c0/0x4387/0x43d8` quand
`task_md ∈ {5,6,8,9}` (+ `CALYPSO_BSP_DISPATCH_NOIMR=1`, ajouté pour séparer
l'installation du démasquage IMR).

| Cible installée | Résultat mesuré |
|---|---|
| `0x76fb` (entrée légitime) | routine entrée ; **préemptée à `0x7706`** par une IT → vecteur `0x00d4` ; ne reprend jamais |
| `0x7708` (corps, raccourci) | high-water `0x7708` → **`0x7843`** ; garde `0x7720` atteinte ×3 — mais **rejette** |

Pourquoi la garde rejette avec `0x7708` : `dma(0x7e)` est **DP-relatif**, et entrer au
milieu de la routine se fait **sans le contexte de l'appelant** → `DP = 0x189`, la garde
lit `data[0xc4fe] = 0x003e` au lieu d'une cellule NDB (`DP = 0x11` donnerait `0x08fe`,
six mots après `d_fb_det`). **C'est une limite de la béquille, pas un défaut du firmware.**

`data[0x08f8]` est touché **290 fois — uniquement par le CLEAR `0x778a`** (`0x0000 →
0x0000`). Le publisher `0x79e4` n'a jamais tourné.

**Ce que la béquille établit malgré tout :** le slot `0x43d8` est bien le verrou, et la
chaîne amont (IT → scheduler → ISR → dispatcher) est fonctionnelle.

---

## 5. LE VERROU SUIVANT — préemption vec21 (BRINT0)

Sur le chemin **légitime** (`0x76fb`), la routine est préemptée six instructions après
son entrée :

```
0x7706  →  *** vers 0x00d4 ***  (op = 0xf4eb)
```

`0x00d4` = `0x0080 + 21×4` = **vec 21**. Le code le documente déjà
(`calypso_c54x.c:2645`) : *« vec19(FRAME)@0xcc et vec21(BRINT0)@0xd4 sont des stubs
RETE(0xf4eb)/NOP à froid »*. `f4eb` = `RETE`. C'est donc **l'arrivée de nouveaux
échantillons I/Q (BRINT0) qui interrompt la routine FB**, et son handler est un simple
retour d'interruption.

**Question ouverte, à mesurer :** un `RETE` doit rendre la main à l'instruction
interrompue (`0x7707`) et la routine devrait tomber dans son corps `0x7708`. Or elle
repart de `0x76fb` au passage suivant. **Où le `RETE` retombe-t-il réellement ?**
(sonde : les PC suivant la sortie + le sommet de pile).

---

## 6. CE QUI TOMBE DE LA VERSION PRÉCÉDENTE

| Affirmation retirée | Pourquoi |
|---|---|
| « Cause racine #1 = le reroute `FB_ENERGY` vers un noyau bancarisé » | **retirée comme cause racine, pas comme fait** : les mesures qui la fondaient ont été prises alors que le DSP n'exécutait pas son firmware (§1). Le reroute lui-même **fonctionne** (2 tirs mesurés, +5,601 s et +6,170 s) et amène réellement le flux au corrélateur — `DADST`/`DETECTOR-RUN` le prouvent. Mais il y arrive par un chemin que la ROM n'emprunte pas, en court-circuitant l'étage qui publie : `d_fb_det` restait 0. |
| « `CALA@0xb01e` = le dispatch FB » | elle fire à +0,111 s avec `d_task_md(0x0804/0x0818) = 0` — c'est de l'init. (Sa sonde etait plafonnee a 40 tirs, tous consommes au boot.) **Superseded 2026-07-28** : c'est la MESURE de +0,111 s qui ne prouvait rien (plafond de sonde) ; le reroute a fire plus tard (+5,601 s / +6,170 s) et le S2 montre `0xb01c/0xb01e` bien sur le chemin FB. Lire « la sonde etait aveugle », PAS « ce n'est pas le dispatch FB » (cf. `ETAT_ACTUEL.md` S3 M6). |
| « slot de dispatch `0x4387`/`0x43c0` » | le slot effectif est **`0x43d8`**, en adressage absolu |
| « 30+ writers de `0x08f8` dans la PROM » | la quasi-totalité sont l'**opcode** `ADD *(lk),A` (ex. `@0x772b : 08f8 3fb3`), pas une adresse. Writers réels : `0x79e4` (set), `0x778a` (clear), `0xb2cd` (reset NDB). |
| « 0 FCCH sur 200 dumps du buffer » | artefact de fenêtre : les 200 records ont été pris pendant `d_fb_mode = 0` (le DSP ne cherchait pas de FCCH). Corrigé : la sonde est désormais gatée `d_fb_mode != 0`. |

### ⚠️ Correction d'une sur-affirmation (2026-07-27, tard)

J'avais écrit que le gate du reroute (`s->data[0x058a] == 5`) testait une cellule valant
« `0x4000` en permanence ». **C'est faux.** Les 40 échantillons de la sonde `CALA-FB`
étaient tous pris entre +0,111 s et +0,176 s (le plafond de sonde était consommé au
boot) ; or le reroute a firé plus tard, à +5,601 s et +6,170 s — donc `data[0x058a]`
**prend bien la valeur 5**. Ce qui reste exact : `0x058a` est **sous `0x0800`**, donc en
DARAM interne du DSP et **non** dans l'API RAM — ce n'est pas la cellule `d_task_md` de
l'interface ARM (`0x0804` page0 / `0x0818` page1), mais rien n'interdit au firmware d'y
cacher le mode courant. **À mesurer avant toute nouvelle affirmation.**

### Ce que `DADST` / `SHADOW-DADST` prouvent, et ce qu'ils ne prouvent pas

La sonde `SHADOW-DADST` (`calypso_c54x.c:14538` pre-capture / `:14691` impression, non gatée) fire sur l'exécution d'un
`DADST`/`DSADT` = le noyau corrélateur calcule. Elle **sort** en mode
`NATIVE_HELPED` + `FB_ENERGY=1` (reroute actif) et **disparaît** en natif pur
(`CALYPSO_DSP=none`, `FB_ENERGY=0`), où `DETECTOR-RUN` = 0.

- ✅ ce que ça prouve : **le corrélateur émulé fonctionne** et peut calculer.
- ❌ ce que ça ne prouve pas : que le firmware l'atteint. En natif il n'est appelable
  que par `0x7725 : CALL 0x795f`, **après la garde `0x7720`** — qui rejette.

Corollaire rétrospectif : la mesure « B2 : le corrélateur CALCULE, |A|=294908 », pilier
de l'ancien dossier, décrivait un corrélateur **alimenté de force**. Elle ne dit rien du
chemin natif.

**`SHADOW-DADST` est donc le bon test de non-régression d'aval** : le jour où il refire
*sans* `FB_ENERGY`, c'est que la garde est franchie et que le corrélateur travaille pour
de vrai.

Reste valide de la version précédente : le recoupement osmocom (§7), et le constat que
`d_fb_det` et `a_sync_demod[]` proviennent du même étage — leurs deux zéros sont **un
seul** producteur absent, pas deux symptômes.

---

## 7bis. REPRODUCTION

```bash
cd ${QEMU_TREE}

# mode natif VRAI (shunt désarmé) — impossible avant les correctifs du §1
CALYPSO_NATIVE_HELPED=1 CALYPSO_DSP=none CALYPSO_FB_ENERGY=0 \
  CALYPSO_FBCALL=1 CALYPSO_FBROUTE=1 CALYPSO_B4=1 ./start-clean.sh

# graphe d'appels de la tâche FB (déclenché sur le front d_task_md → 5)
grep -A60 "FBCALL === tache FB #1" /root/qemu.log

# RANK3 : le slot et son unique installateur
CALYPSO_SLOTSRC=1 CALYPSO_SCAN43D8=1 ...   # SLOTSRC-RD / SCAN43D8

# béquille de validation (test, PAS un correctif)
CALYPSO_BSP_DISPATCH_FB=1 CALYPSO_BSP_DISPATCH_FB_TGT=0x7708 \
  CALYPSO_BSP_DISPATCH_NOIMR=1 CALYPSO_FBROUTE=1 ...
grep -oE "FBROUTE high-water PC=0x[0-9a-f]+" /root/qemu.log | tail -1
grep "FBROUTE jalon PC=0x7720" /root/qemu.log   # DP + adresse effective + valeur
```

Sondes livrées, **toutes gatées par env et inactives par défaut** : `CALYPSO_FBROUTE`,
`FBCALL`, `TASKGO`, `FBENTRY`, `DISPTAB`, `DISPIDX`, `SLOTSRC`, `SCAN43D8`, `SCANFB`,
`CALA_FB`, `DARAM_DUMP` (+ verdict `DARAM-SANITY`), `BSP_DISPATCH_NOIMR`,
`BSP_IQ_SHIFT`. À nettoyer une fois le dossier clos.

## 7. RECOUPEMENT AVEC OSMOCOM-BB — comment le vrai firmware obtient `d_fb_det`

Source : `${GSM_ROOT}/osmocom-bb/src/target/firmware`. Ce recoupement n'était pas dans le
périmètre du workflow ; il **corrobore** la §1 et **ouvre une piste** que le rapport n'a
pas explorée.

**Déclaration.** `include/calypso/dsp_api.h:202` :
```c
API d_fb_det;      // FB detection result. (1 for FOUND).
```
Cellule **NDB** (non double-bufferisée) : écrite par le DSP, lue par l'ARM sans page-flip.

**Commande** — `layer1/prim_fbsb.c:364-386`, `l1s_fbdet_cmd()` fait **trois** choses :
```c
rffe_compute_gain(rxlev2dbm(fbs.req.rxlev_exp), CAL_DSP_TGT_BB_LVL);  /* AGC */
dsp_api.db_w->d_task_md = FB_DSP_TASK;      /* = 5  (l1_environment.h:73) */
dsp_api.ndb->d_fb_mode  = fb_mode;
l1s_rx_win_ctrl(fbs.req.band_arfcn, L1_RXWIN_FB, 0);   /* programme le TPU */
```

**Lecture** — `prim_fbsb.c:404`, `l1s_fbdet_resp()` : `if (!dsp_api.ndb->d_fb_det)`,
jusqu'à 12 tentatives par set puis `FB0_RETRY_COUNT` re-planifications. Sur détection,
`read_fb_result()` (`:305`) lit `a_sync_demod[D_TOA/D_PM/D_ANGLE/D_SNR]` **puis remet
`d_fb_det = 0`**.

### Ce que ça apporte au diagnostic

| # | Constat osmocom | Conséquence |
|---|---|---|
| a | `FB_DSP_TASK = 5` (`l1_environment.h:73`) | confirme **indépendamment** le `d_task_md == 5` du reroute (§1.5) |
| b | `d_fb_det` **et** `a_sync_demod[]` sont lus dans la même routine, et produits par le même étage DSP | corrobore §1.2 : `FBDET-WR = 0` et `ANGLE-WR = 0` ne sont pas deux symptômes mais **un seul producteur absent** |
| c | **`l1s_rx_win_ctrl(..., L1_RXWIN_FB, ...)` programme le TPU en même temps que le DSP** | la détection FB = DSP **+ fenêtre TPU**. Piste non explorée par le workflow |
| d | l'AGC est réglée *avant*, en visant `CAL_DSP_TGT_BB_LVL` (**niveau bande de base cible**) | fonde la piste amplitude : notre chaîne livre 99,3 % de la pleine échelle **sans AGC** |

**Le point (c) est le plus important.** Il désigne un candidat sérieux pour **S1** (« la
tâche `0x7700` n'est jamais dispatchée ») : dans le vrai firmware, la tâche FB n'est pas
seulement commandée par `d_task_md`, elle est **cadencée par une fenêtre RX programmée au
TPU**. Or c'est précisément le câblage identifié comme manquant dans la dette du projet
(`l1s_rx_win_ctrl → tpu_enq_dsp_irq` = 0 hit ; tâche **RANK2 — Fenêtre RX BDLENA**, encore
ouverte). Si le DSP n'est jamais réveillé au bon instant par le TPU, désactiver le reroute
`FB_ENERGY` ne suffira pas — ce que le test du §4 tranchera par le cas « `FBDET-WR = 0` ET
`ANGLE-WR = 0` ».

**Le point (d)** donne un fondement au levier `CALYPSO_BSP_IQ_SHIFT=n` (ajouté le 27/07,
défaut 0) : osmocom présuppose un niveau bande de base **calibré par l'AGC**, pas un signal
à pleine échelle.

### Reproduire ce recoupement
```bash
docker exec osmo-operator-1 bash -lc 'cd ${GSM_ROOT}/osmocom-bb/src/target/firmware && \
  grep -n "d_fb_det" include/calypso/dsp_api.h layer1/prim_fbsb.c calypso/dsp.c && \
  sed -n 364,386p layer1/prim_fbsb.c && grep -n "FB_DSP_TASK" include/calypso/l1_environment.h'
```

---

## 8. BASELINE NATIF NU — ce que le firmware fait, sans aucune prothèse

**[MISE A JOUR 2026-07-28, verifiee]** le profil WIRE est desormais **opt-in** :
`calypso.env:220` le garde derriere `CALYPSO_WIRE=1` (defaut 0) et ne source
`calypso_wire.env` qu'a la ligne `:227`. Sans `CALYPSO_WIRE=1`, **aucune** des
bequilles ci-dessous n'est posee. Instrument : `grep -n CALYPSO_WIRE calypso.env`
et le `[calypso-manifest]` du run. Ce qui suit decrit l'etat du 27/07, ou
`calypso.env` sourcait **inconditionnellement** `calypso_wire.env`, qui activait par
défaut une douzaine de béquilles : `ARM2DSP_BGEN`, `ARM2DSP_CTRLSYS`, `KEEP_IMR`,
`TINT0_MASTER`, `FORCE_INTM_ONESHOT`, `BSP_DIRECT_BRINT0`, `BSP_DISPATCH_FB`,
`BSP_DARAM_FORCE`, `FIX_3FCD`, `SEED5AC8_VAL`… Tous nos runs « natifs » les portaient.

**Run de référence, toutes coupées** (`FORCE_INTM_ONESHOT=0 BSP_DIRECT_BRINT0=0
KEEP_IMR=0 TINT0_MASTER=0 BSP_DISPATCH_FB=0 ARM2DSP_CTRLSYS=0`, `DSP=none`,
`FB_ENERGY=0`) — DSP vivant : 9,4 M instructions, 22 266 lignes `[c54x]`, ARM postant
`task_md=5` 53×, DSP l'enregistrant 84× :

```
d_task_md=5 → vec 0x00f0 → sched 0x7234 → 0x013b  (DÉRAIL, au lieu de 0x8341)
  → 0xa4e4 → 0xa4fd → 0xb522 → 0xa501 → 0xa51c → 0xa531 → 0xa534 → 0xa53c
      BITF *(AR1+0x10),0x8000 : data[0x0810]=0x0000 → TC=0 → BC 0xa575 PRIS
  → 0xa575 → 0xaff9 → 0xd294 → 0xaffd → 0xb01c
      LD *(0x43d8),A = 0xab38 → CALL → RET immédiat
FBROUTE=0  SHADOW-DADST=0  FBDET-WR=0  ANGLE-WR=0
```

**Le graphe est IDENTIQUE à celui obtenu béquilles activées.** Seule `BSP_DISPATCH_FB`
avait un effet mesurable (la `CALA` résolvait alors vers `0x8d00`). Les onze autres sont
**inertes sur le chemin FB** — elles ont été écrites à une époque où le DSP n'exécutait
pas son firmware.

**Deux effets de bord de nos propres béquilles, identifiés au passage :**
- `BSP_DIRECT_BRINT0=1` faisait lever vec21 par le BSP → c'est **notre** hack qui
  préemptait la routine FB à `0x7706`, pas le firmware.
- `BSP_DISPATCH_FB` était **déjà à 1 par défaut** ; si elle ne tirait pas, c'était à
  cause de `get_task_md()` (défaut n°3 du §1), pas de la gate.

### Ce que fait réellement `0x8d00` (sonde ARWATCH)

Quand on force la `CALA` vers `0x8d00`, la routine **calcule vraiment** : chargement de
coefficients (`0x8e8c`, `COEFFS-WR` → `0x2bc0..0x2bc7`), boucle MAC `0x8e97 ↔ 0x8ea8`
avec accumulateur variant. Mais **aucun registre d'adresse ne pointe jamais dans le
buffer IQ `[0x2a00..0x2b27]`** sur toute son exécution :

```
AR3 = 0x4bd0     source coefficients
AR4 = 0x2bc0..   destination (workzone 0x2b28..0x2c00)
AR5 = 0xdb7b..   opérande, +2 par itération — mémoire haute, hors buffer IQ
```

⚠️ **Ceci tranche une contradiction entre deux sources du projet**, en faveur du code :
`calypso_c54x.c:6041` (« `0x8d00` ne touche jamais le buffer IQ `0x2a00` ») est
**confirmé** ; `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md`, qui désigne `0x8d00` comme
la cible de dispatch requise, est **infirmé sur ce point** — `0x8d00` corrèle autre
chose. Corrobore la note `correlator-ar5-not-in-buffer-rank3` (AR5=0xdb7b).

Le correlateur du chemin energie est celui qui **ecrit** `0x2a00` : `0x2a00..0x2b27`
est sa **workzone de SORTIE** (`STH A, ASM, *AR4+` @`0x9fb8`), pas son entree — **ne
jamais y feeder** (cf. `hw/arm/calypso/doc/ETAT_ACTUEL.md` S3 M8 et S3.1). Instruments :
`AR-FIRSTUSE AR4=0x2a00 PC=0x9fb8` et `SHADOW-DADST=370` en mode *helped*. Son entrée
**référencée en ROM** est `0x94f5` (`@0x87e7 f930 94f5`) ; `0x9500`, la valeur imposée
par `calypso_native_helped.env`, n'apparaît **nulle part** dans les 28 672 mots — on
saute 11 mots de mise en place (`ST1`/`DP`/`ARP` possibles).

### LA question ouverte, désormais unique et bien posée

> Pourquoi l'ordonnanceur de trame `0x7234` part-il vers `0x013b` au lieu de tomber sur
> `0x8341` (la LUT FB), qui est la seule à installer un vrai handler dans le slot de
> dispatch `0x43d8` ?

C'est la formulation de `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md` — **désormais
confirmée par la mesure sur un DSP qui exécute son firmware**, ce qui était impossible
avant les correctifs du §1.


---

## 9. INVALIDATION de `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md`

Ce document désignait la cause racine ainsi : *« le frame scheduler `0x7234` DÉRAILLE
vers l'overlay `0x013b` via soft-vector au lieu de tomber sur `0x8341` (la LUT FB), seule
à installer `0x8d00` dans le slot de dispatch »*, et proposait comme correctif un
événement TPU redirigeant `0x7234 → 0x8341`.

**Les deux affirmations sont fausses, mesurées séparément.**

### (a) `0x8341` n'est atteignable par rien

Scan des **4 banks** PROM (sonde `CALYPSO_SCANREF=0x8341`) : **`total = 0`**. Aucune
référence — ni `CALL` (`f074`), ni `B` (`f820`/`f880`), ni même comme mot de donnée. Le
code y est pourtant réel (`0x8341: 7624 0400 7625 0800 f7b9 f6b6 7711 2f22 …`), mais
**personne ne peut y sauter**. Un correctif qui redirige vers `0x8341` redirigerait vers
du code que le firmware n'appelle jamais.

### (b) `0x7234 → 0x013b` n'est pas un dérail

Désassemblage (`CALYPSO_TRACEFROM=0x7234`) :

```
0x7234:  f074 013b     CALL 0x013b        <- INCONDITIONNEL, en ROM, aucune branche
0x7236:  7707 2900
0x7238:  7706 1800
0x723a:  68f8 001d fffc
0x723d:  f074 a4e4     CALL 0xa4e4
```

Et `0x013b` est une **routine de sauvegarde de contexte** (dump overlay) : `8bf8 3fcd`
puis la rafale `4a06 4a1c 4a1b …` (`PSHM`), terminée par `4bf8 3fcd f495 fc00` (`RET`).
Elle **retourne** en `0x7236`, et l'exécution enchaîne sur `CALL 0xa4e4` — exactement ce
que montre la trace `FBCALL`. C'est le fonctionnement nominal de la ROM, pas une dérive.

### (c) Aucun writer caché du slot

L'exclusion est forte : `DISPATCH-CELL-RESEED` est une watchpoint dans
`data_write_locked`, donc elle voit **toutes** les écritures DSP de `data[0x43d8]`, quel
que soit le mode d'adressage (indirect compris — un scan de mot littéral, lui, ne les
verrait pas). Relevé : **uniquement `0xbb00 → 0xab38`, 15 fois, toutes dans les 0,18
première seconde**. Rien ne remplace jamais le stub.

### (d) Recoupement osmocom : la ROM seule est censée suffire

`calypso/dsp_bootcode.c` :

```c
/* We don't really need any DSP boot code, it happily works with its own ROM */
static const struct dsp_section *dsp_bootcode = NULL;
```

`dsp_pre_boot(NULL)` ne téléverse donc **rien**, et `dsp.c:205` porte un
`/* FIXME: Implement Patch download, if any */` — le patch DSP n'est pas implémenté non
plus. Sur vrai matériel, osmocom fait fonctionner la FBSB avec **la ROM seule**.

### Ce que ça laisse

> **La ROM seule suffit sur le téléphone réel. Notre émulation n'atteint pas le chemin
> qui installe un vrai handler dans `data[0x43d8]`.**

Les hypothèses « handler téléversé » et « dérail du scheduler » sont écartées. Restent :
1. un **événement d'initialisation** non émulé, qui déclencherait une autre séquence
   d'installation du slot ;
2. le traitement FB **ne passe pas par ce slot** dans le firmware réel — auquel cas
   `0xb01c` n'est pas le bon point d'observation ;
3. une **divergence d'exécution** (décodage, banques, timing) qui fait rater au firmware
   la branche qui installerait le handler.

Aucune n'est départageable avec les mesures actuelles. Ce qui est acquis, en revanche,
c'est qu'on ne perdra plus de temps sur le TPU ni sur `0x8341`.



---

# Session 2026-07-28 — la chaine d'entree, mesuree de bout en bout

Tout ce qui suit est **mesure**, jamais deduit. Chaque ligne nomme son instrument et
la commande qui la rejoue.

## 1. Acquis

| # | Fait mesure | Instrument |
|---|---|---|
| 1 | Le BSP jetait les bursts RX : **3 gates**, pas un seul. `calypso_bsp.c:474` et `:997` se levent avec `CALYPSO_BSP_DARAM_FORCE`, mais `:1359` (la **livraison** vers `data[]`) ne connaissait que `CALYPSO_TPU_RX_WIRE`. `DARAM_FORCE=1` ouvrait donc **2 verrous sur 3** et rien n'arrivait. Aligne le 2026-07-28 ; **defaut inchange**. | `[bsp] deliver: gate shunt LEVE (rxw=1)` |
| 2 | `CALYPSO_BSP_IQ_DECIM=1` est une **regression** : le feed arrivait a **4 SPS** (`dphi=+0.25x pi/2`). Avec `DECIM=4` : `VERDICT: FCCH @1SPS PROPRE`. Corollaire : `DARAM_LEN=296` (638 ne valait que pour le 4 SPS non decime). | `corr_iq.py --src bursts` |
| 3 | **L'entree demod est bien `0x4c00`** : `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a` — **stride 5** (= le polyphase 6 taps) — avec des valeurs reelles (`ff6e`, `c1fb`, `d147`). | `CALYPSO_WATCH_9F00_RD=1` |
| 4 | `0x9260/0x9261` (cibles de `CALYPSO_FB_STREAM_CELL`) ne sont **jamais lues** dans cette config. `FB_STREAM` reste inerte (`fb_stream_next` jamais appele). **Piste abandonnee** : le demod consomme le buffer directement. | `WATCH-9F00-RD` + absence de `FB-STREAM addr=` |
| 5 | **Sortie demod = DC quasi pur et figee**, avec `DECIM=1` **comme** avec `DECIM=4` : `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004`. Cellule temoin invariante sur 157-203 bursts (`0x9fb8@0x2a00=0x0000`, `0x9fe2@0x2a00=0x52ed`). | `corr_iq.py --src ddump` + `WMAP` |
| 6 | Noyau MAC `0xa079..0xa09d` : **60 000 ecritures pour 4 a 7 valeurs distinctes**, toutes dans `{0001,0002,001f,003e}` ± bit `0xe000`. `003e=2x001f`, `0002=2x0001` : un accumulateur qui double une constante, pas une correlation. | `WMAP` |
| 7 | `0x9fd5` depose une **table de coefficients constante** (`0xffc8`, 5/5 a cellule figee). Son apparente variation etait un artefact d'agregation par PC (une courbe sur 15 cellules). | `WMAP v2` |
| 8 | `d_fb_mode[08f9]=0x0001` observe : le detecteur **est** arme, la fenetre est bonne. L'ancien « 0 FCCH sur 200 dumps » etait un artefact de `d_fb_mode=0`. | `DETECTOR-RUN` |

**Etat resultant : entree vivante, sortie morte.** Le signal reel arrive au bon endroit,
est lu par le bon code, au bon rythme — et l'etage demod n'en produit rien.

## 2. Invalide cette session (mes propres conclusions)

- **« `0x4c00` est gele »** — lecture faite a `0xFFD08800` via le monitor, **hors fenetre
  API RAM**. Signature de l'artefact, reconnaissable : `peak` exactement `0x8000` et 54 %
  de zeros. `corr_iq.py` portait deja l'avertissement (`do_daram`). La mesure valide se
  prend **a l'interieur** (BSP_LOG au point d'ecriture, ou `ddump`).
- **« `Q == 0` »** — conclu sur les 2 premiers mots d'un burst, la ou l'amplitude est faible
  par construction. Sur le burst entier : `zeros=0%`.
- **« `0xa042` detruit le signal avant que le noyau ne le lise »** — `0x2c00` est du scratch ;
  il n'y avait pas de signal a detruire.

## 3. Regles de sonde (payees 4 fois cette session)

1. **Une sonde se concoit par sa CONDITION DE DECLENCHEMENT, pas par son adresse.** Un
   plafond global est mange par le PC le plus bruyant : `0xa079` (48 lignes) a masque
   `0x9fd5` (1 ligne) et rendu le test « constante ou signal ? » indecidable.
2. **Preferer un AGREGAT a un FLUX plafonne.** `WMAP` compte tout le run et n'imprime
   qu'un tableau : aucune fenetre a rater. Y prevoir un **temoin de saturation**
   (`ecrivains=24` = exactement `WMAP_PCS` signalait une table pleine, donc tronquee).
3. **Distinguer « varie dans l'espace » de « varie dans le temps ».** Un PC qui ecrit une
   courbe sur 15 cellules parait varier ; a **cellule figee**, il est constant. Seule la
   variation temporelle est un signal. (`WMAP v2` : champ `@0xADDR(n=...)`.)
4. **« Pas de log » n'est jamais « pas d'evenement »** tant que la sonde n'est pas verifiee
   vivante **et** sa fenetre verifiee couvrante. Quatre causes distinctes rencontrees :
   plafond sature, seuil de dump trop haut, plage ecrite **cote hote** (invisible depuis
   `data_write_locked`), variable d'env absente du manifeste.

## 4. Cible suivante

Entree vivante + sortie morte ⇒ le suspect est l'**emulation des instructions
`0x9f95..0x9fe2`**. Une sortie rigoureusement constante ressemble a un calcul dont le
resultat ne depend pas des operandes lus. Sonde `DEMODIO` (gate `CALYPSO_DEMODIO=1`) :
correle, sur une meme fenetre, ce que le demod LIT et ce qu'il ECRIT, avec A/B/T et les AR.

Deux issues, toutes deux exploitables :
- la sortie ne bouge pas quand les entrees bougent ⇒ decodage/emulation fautif ;
- la sortie bouge dans `data[]` mais `ddump` reste plat ⇒ le buffer de sortie est ecrase
  ailleurs, et il faut trouver par qui.

## 5. Reproduire

```bash
cd ${QEMU_TREE}

# run de reference (chaine d'entree correcte, mesuree)
CALYPSO_NATIVE_HELPED=1 CALYPSO_FB_CORR_ENTRY=0x94f5 \
CALYPSO_DSP_RUN_C54X=1 CALYPSO_BSP_DARAM_FORCE=1 \
CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 CALYPSO_BSP_IQ_DECIM=4 \
CALYPSO_SHUNT_REAL_FB=1 CALYPSO_DEBUG=BSP ./start-clean.sh

grep -E "deliver: gate shunt LEVE|dropping fn=" /root/qemu.log   # gate levee, 0 drop
grep -E "DMA fn=" /root/qemu.log | tail -2                        # le BSP depose
cd ${QEMU_TREE}/tools && python3 corr_iq.py --src bursts | grep VERDICT         # FCCH @1SPS PROPRE
cd ${QEMU_TREE}/tools && python3 corr_iq.py --src ddump  | tail -3              # CONFORMITE KERNEL
```

Sondes disponibles (toutes gatees, **defaut OFF**) : `CALYPSO_WMAP` (+`_LO/_HI/_LO2/_HI2`),
`CALYPSO_RMAP` (+`_PCLO/_PCHI`), `CALYPSO_DEMODIO` (+`_AFTER/_PCLO/_PCHI`),
`CALYPSO_WATCH_9F00_RD`, `CALYPSO_DARAM_DUMP`, `CALYPSO_B2IN`, `CALYPSO_DEBUG=BSP`.
