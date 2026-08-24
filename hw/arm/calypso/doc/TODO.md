# TODO — état au 2026-08-24

> Refonte. Le P0 précédent (nomenclature d'interruption, 2026-07-28) est
> **résolu** : l'IMR est armé, les ITs sont vectorisées, et le DSP acquiert le
> FB tout seul. Le verrou a bougé d'un cran. Ce document repart de ce qui est
> **mesuré**, et nomme pour chaque entrée la mesure qui la trancherait.

---

## ✅ Résolu depuis la dernière refonte

| Ancien verrou | Preuve de résolution |
|---|---|
| **P0 — l'IT trame ne vectorise pas** (`IFR ∩ IMR = 0`) | `IMR-ARM 0x0000 -> 0x3000 PC=0xa4c7` puis `0x52ed` ; **vec28 / bit12** armé ; 18 757 `INTM-TRANS`, vectorisation observée à `PC=0x00f0` |
| **`d_fb_det` reste à 0** | **437** transitions `0x0000 -> 0x0001`, **toutes** écrites par la mask-ROM à `PC=0x79e4`. Côté ARM : `FB1 (…): TOA=39, Power=-52dBm, Angle=-1Hz`, `fn_offset` calculé, `Synchronize_TDMA` appelé. Aucune gate d'injection posée |
| « le corrélateur n'est jamais ordonnancé » | il l'est ; la tâche SB s'exécute et écrit ses slots |

C1 (`C54X_INT_FRAME_VEC/BIT = 28/12`) est donc **appliqué et validé par la
mesure**. `RAPPORT_DFBDET.md` analyse un symptôme désormais éteint : le garder
pour sa méthode, pas pour son verdict.

---

## P0 — Nourrit-on le bon bout du démodulateur ?

**C'est la question la plus rentable du moment** : une seule mesure, et elle
explique potentiellement les trois murs restants d'un coup.

**Fait établi (désassemblage vérifié)** — le front-end démod `0x9f95-0x9fcb` est
un filtre multi-cadence :

```
0x9f9f  stm  #0x027e, BK      ; anneau d'entree, 638 mots
0x9fa9  rptb 0x9fc7
  0x9fb5  ld   *AR6-0%, A     ; LIT  l'entree  (circulaire)
  0x9fb8  sth  A, *AR4+       ; ECRIT la sortie (lineaire)
0x9fcb  banz *AR7-, 0x9fa7
```

**Deux mesures du dépôt, prises à six mois d'écart et jamais confrontées :**

| adresse | ce qui l'atteste | rôle probable |
|---|---|---|
| `0x4c00` | ancien P2 : « `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a`, stride 5 = polyphase 6 taps » | **entrée** du démod |
| `0x2a00` | `calypso_bsp.c:1123-1129` : « DSP READS `0xCAFE` at `0x2a00..0x2a13` via `PC=0x93a5`, AR3 post-incrémenté » | **sortie** du démod, lue par le corrélateur |

Elles **ne se contredisent pas** : ce sont les deux extrémités du même étage. Or
le défaut actuel dépose en `0x2a00` (`calypso_bsp.c:1131`), et le run de
référence de juillet posait `CALYPSO_BSP_DARAM_ADDR=0x4c00`.

- [ ] **D1 — trancher par `DEMOD-IO-PROBE`** (posée, lecture seule, gate
      `CALYPSO_DEMODIO=1`) : relever `AR6` en `0x9fb5` et `AR4` en `0x9fb8`.
      `AR4 = 0x2a00` ⇒ on alimente la sortie, l'entrée n'est nourrie par personne,
      **et le Viterbi mange de l'I/Q**.
- [ ] **D2 — si D1 confirme** : rebaser la livraison BSP sur la base d'`AR6`,
      avec `daram_len = BK = 638`, et ne laisser **qu'un seul** écrivain de
      `0x2a00` — le démod lui-même. `FB_IQ_OWNS`, `DEMOD_NOCLOBBER` et
      `DARAM_FCCH_ONLY` deviennent alors caducs.
- [ ] **D3 — si D1 infirme** : chercher en aval, à `0x9a78` (démod → bits
      souples) puis `0x9aaf` (Viterbi). Critère : écart max−min de
      `data[0x2c00..0x2c0f]` juste après le retour de `0x9a78`. Plat ⇒ front-end.
      Net ⇒ traceback.

---

## P1 — Le décodage du SCH sort une constante

**Fait mesuré** : la tâche SB s'exécute, mesure un TOA plausible, écrit ses
slots — et le firmware l'écarte.

```
a_sch[0] = 0x8100   = B_BLUD | B_SCH_CRC   ->  prim_fbsb.c:181 abandonne
a_sch[3] = 0xf8d8   CONSTANT sur 21/21 ecritures
```

**Mesure décisive déjà faite** : 10 contenus de burst **distincts** présentés
(10 énergies distinctes, 15 trames SCH sur 16), **sortie identique au bit près**.
Un décodeur dont la sortie ne dépend pas de l'entrée ne décode pas.

Hypothèses **écartées, avec leur preuve** :

| hypothèse | pourquoi elle tombe |
|---|---|
| *timing* — « 9 bursts non-FCCH écrasent le tampon entre deux FCCH » | corrigé par `CALYPSO_BSP_DARAM_FCCH_ONLY=1` : la tâche SB reçoit 15 SCH sur 16, **sans effet sur le CRC** |
| *écrêtage* — les échantillons saturent | rendre `IQ_SHIFT` effectif **tue l'acquisition FB** (`Synchronize_TDMA` 8 → 0). Le DSP veut la pleine échelle |
| *entrée figée* | 10 énergies distinctes présentées, sortie constante |
| *le relais `a_sch26 → a_sch` corrompt* | `0xb213 RPT #4` / `0xb214 MVDD` = transport pur, vérifié à l'exécution |
| *poison de boot `dsp.c:424` sur `a_sch26[0]`* | **faux** : `0x9868` est un `ST` plein qui écrase |

- [ ] **S1 — dépend de D1.** Si le front-end est mal nourri, ce mur tombe seul.
      Ne pas instrumenter le Viterbi avant d'avoir tranché P0.

---

## P2 — Le DMA du DSP n'existe pas dans le modèle

**Fait mesuré** : `DSP Error Status: 8` = `DSP_ERR_DMA_PROG`, **404 occurrences**
en natif, **0 en `shunt_legit`**, et présent même quand la tâche SB ne tourne
quasiment pas — donc **indépendant** du mur SCH.

`d_error_status == data[0x3f92] & 0x0FFF` ; publié par `0xb10a`.
Cause documentée dans `calypso_dma.h` : le firmware DSP empile ses demandes dans
un anneau de 14 entrées (`data[0x433e]` lecture / `0x433f` écriture, producteur
`0xaa75`, consommateur `0xaa87`) ; le modèle **accepte la programmation mais
n'exécute aucun transfert et ne lève jamais `DMAC0..5`** → rien ne signale la
complétion → l'anneau sature → `0xaa83` pose le bit 3.

- [ ] **M1 — activer `calypso_dma.c`** (`CALYPSO_DMA=1`, présent, défaut OFF) et
      mesurer l'effondrement de l'erreur 8.
- [ ] **M2 — trancher le conflit de mapping MMR** : SPRU131 donne
      `DMPREC=0x54 DMSA=0x55 DMSDI=0x56 DMSDN=0x57` ; `calypso_c54x.c` utilise
      `0x54` comme `DMSA` (décalage d'un registre). La sonde `DMAWATCH` le
      signale déjà par son libellé « DMPREC?(modele:DMSA) ».
- [ ] **M3 — non modélisé, à assumer explicitement** : synchronisation par
      événement (`DMSFC`), transferts vers l'espace programme/IO, priorités
      (`DPRC`), mode ABU.

---

## P3 — Le sas ISA `CALYPSO_FIX_LK_SHFT`

Le décodeur applique au champ de décalage des formes `ADD/SUB/LD/AND/OR/XOR #lk`
— **4 bits** (`op & 0xF`) — la règle de signe d'un champ de **5 bits**. On ne
représente pas −16..15 sur 4 bits. `doc/opcodes/tic54x-opc.c` donne `OP_SHFT`
(non signé) pour `ld`, `sub`, `and`, `or`, `xor` ; seul `add` porte `OP_SHIFT`.

| | sas OFF | sas ON |
|---|---|---|
| `A` avant le `SFTA` (`0x7d1b`) | `0x0000000000` | `0x0000008000` |
| quotient (`0x7d1e`) | `0x0000` | `0xfff4` |
| `T` au `MPY` (`0x81e4`) | `0x0000` (27/27) | `0xfff4` |
| `Synchronize_TDMA` | 8 | 13 (pas de régression) |

- [x] niveau 1 — formel (binutils + encodage)
- [x] niveau 2 — grandeur physique
- [ ] **niveau 3 — chemin fonctionnel** (camp → LU → SMS). Réunir « c54x actif »
      et « chaîne complète » est possible : `calypso_shunt_legit.env` pose
      `CALYPSO_DSP_RUN_C54X:=0`, donc un `=1` en CLI réveille le c54x dans le
      banc shunt.
- [ ] **`ADD` (sous-code 0)** volontairement non touché : le corriger en même
      temps rendrait la mesure ininterprétable.

---

## P4 — Autres écarts relevés au différentiel `shunt_legit` ↔ `native`

- [ ] **L'IT trame a perdu son front.** `calypso_tpu.c:322` *teste*
      `TPU_CTRL.DSP_EN` et ne l'efface jamais ; le seul effacement
      (`calypso_trx.c:1854`) est dans un bloc inactif en natif. Le DSP est
      réveillé à *chaque* tick au lieu d'une fois par trame — et l'anneau de
      tâches déborde. **Le correctif RETIRE un événement fabriqué**, il n'en
      ajoute pas. À faire avant M1 : les deux touchent le même symptôme.
- [ ] **On livre un burst, pas une fenêtre.** `calypso_bsp.c:1568` remet
      `woff = 0` à chaque rafale, donc le « à l'heure » du modèle vaut 0 là où le
      contrat vaut **23**. D'où le TOA errant et le SNR au plancher qui fait
      échouer la porte `prim_fbsb.c:489`.
- [ ] **Angles morts d'instrumentation.** Aucune sonde sur `a_sch[0]`, `a_sch26`,
      les 4 lectures de `d_task_md`, `a_fd`. Et `SM-DPAGE` **regarde le mauvais
      tableau** : `calypso_c54x.c:16740` imprime `s->data[0x08d4]` alors que la
      ROM lit `s->api_ram[…]`.

---

## Chantiers parallèles

- [x] **Voix TCH/F** — appel **10001 ↔ 10002** abouti en `shunt_legit`, état
      `ACTIVE` des deux côtés, GAPK codec `fr`.
- [x] **SMS MO et MT** — les deux sens, 0 erreur.
- [ ] **CP-ACK final du MT jamais reçu** (3/3) : le réseau libère le canal avant.
      Cosmétique — le SMS et son RP-ACK sont déjà passés — mais c'est un écart
      24.011 réel côté osmo-msc. Mesure : `logging level lsms debug` +
      capture GSMTAP 4729.
- [ ] **`bssmap:cipher_mode_complete = 0`** au MSC alors que le mobile enregistre
      7 `CIPHERING MODE COMPLETE`. Écart de **comptage**, pas absence de
      chiffrement — l'air dit le contraire.

---

## Réfuté — ne pas re-soulever

| Affirmation | Pourquoi elle tombe |
|---|---|
| « verrou = BRINT0 / IMR bit 5 » | `vec 21 = BXINT0` (émission). BRINT0 = `vec 20 / bit 4` |
| « le DSP n'acquiert pas le FB » | **437** écritures de la ROM à `PC=0x79e4`, mesurées |
| « `a_sch26[0]` verrouillé par le poison de boot `dsp.c:424` » | `0x9868` est un `ST` plein qui écrase |
| « aucun site de la ROM n'écrit `db_r+0x00/+0x01` » | `0xb003` / `0xb007` le font — 382 occurrences runtime |
| « le SMS MT échoue » (`dropping pending message`) | **l'extrait commençait après la livraison** : `RX SMS` + `TX: SMS RP ACK` sont deux lignes plus haut. Queue structurellement bénigne (`gsm0411_smr.c:236-237`) |
| « `CALYPSO_BSP_IQ_SHIFT` règle l'amplitude vue par le DSP » | il est appliqué dans `calypso_bsp_rx_burst`, **dont l'écriture est écrasée par le DSP lui-même** (`writer_kind=10`). Mesure : `\|max\|brut=30252` → `\|max\|daram=119` côté hôte, `±32296` côté tâche SB |
| « `d_fb_det` natif = injection hôte » | `CALYPSO_INJECT_*=0`, `CALYPSO_CANNED` annonce `FBDET=0 TOA=0 PM=0 SNR=0 ANGLE=0` |
| « `WATCH_9F00_RD=0` = corrélateur affamé » | **conséquence**, pas cause |

**À NE PAS TOUCHER** — le natif y est conforme, parfois meilleur que la
référence : `d_fb_det` · `D_ANGLE` (0…2 Hz, passe les deux portes) · `D_PM`
(natif −52 dBm ; le shunt sort **+173 dBm**, hors domaine) · TOA multiple de 48
en FB0 (nominal) · le relais `a_sch26 → a_sch` · les seuils
`data[0x0c3d]/0x0c3e/0x0c41/0x0c42`.

---

## Règles de travail

1. **Lire le MANIFESTE, jamais la ligne de commande.** `qemu-manifest.log` du run
   analysé — et *pas* `/proc/<qemu>/environ` quand on regarde une photo
   archivée : cela décrit le run *vivant*, pas celui qu'on lit.
2. **Chaque mode a sa porte d'entrée, et elles écrivent ailleurs.**
   `start-direct.sh` → `/tmp/osmo-nitb/logs` · `run.sh` → `/tmp/calypso/logs`.
   Analyser le mauvais répertoire donne des compteurs périmés qui ressemblent à
   des pannes.
3. **Toute sonde muette exige un CONTRÔLE armé** sur un site dont on sait qu'il
   s'exécute. Sans lui, « la sonde n'a rien vu » et « le code ne tourne pas »
   sont indiscernables. Une conclusion entièrement fausse a été tirée ainsi sur
   une division jamais exécutée.
4. **Une sonde se conçoit par sa condition de déclenchement**, pas par son
   adresse ; préférer un agrégat à un flux plafonné ; distinguer « varie dans
   l'espace » de « varie dans le temps ». Un plafond de 6 lignes qui ne capture
   que le silence de démarrage conclut faux.
5. **Ne jamais lire une machine à états par sa fin.** Le SMS a été déclaré en
   échec sur une queue de nettoyage située *après* la livraison réussie.
6. **Un correctif à la fois pour la mesure**, mais tous d'un coup dans le sas.
7. **Non-régression obligatoire** après chaque correctif : `shunt_legit` par sa
   porte, puis `tools/rapport-run.sh` → `BSIC=7` + `SYSTEM INFORMATION` +
   `LOCATION UPDATING ACCEPT`, **sans exception**.
8. **Toute béquille porte `@BEQUILLE`** — `grep -rn "@BEQUILLE" hw/arm/calypso/*.c *.env`.
9. **Autorité opcodes** : `doc/opcodes/tic54x-opc.c` (binutils, le champ MOTS
   fait foi) > `spru172c.pdf` (sémantique) > le code > les tableaux de synthèse.
   **Ne jamais conclure depuis un commentaire** : plusieurs se sont avérés
   périmés, y compris ceux qui décrivent une intention jamais implémentée.
10. **Ne pas mesurer sous gdbserver.** Il perturbe l'acquisition : un « ça cale
    plus tôt » entièrement imputable au débogueur a coûté une soirée.
