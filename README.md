# QEMU-Calypso

Let me read the top-level docs to describe it accurately.Voici ce que fait ce projet.

## En une phrase

**QEMU-Calypso** émule le baseband GSM **TI Calypso** (celui des Motorola C1xx / Compal E88) à l'intérieur de QEMU. Sa particularité : il fait tourner **deux cœurs en même temps**, exactement comme la vraie puce — un **ARM** qui exécute le firmware [osmocom-bb](https://osmocom.org/projects/baseband) *d'origine et non patché* (la Layer 1 temps réel), et un **DSP TMS320C54x** qui exécute la **vraie mask-ROM DSP de TI**. Les deux se parlent par la mailbox à l'adresse `0xFFD00000`, comme sur le silicium réel.

L'intérêt revendiqué : aucun autre émulateur de baseband ne ferait tourner *à la fois* le firmware constructeur intact *et* le DSP de couche 1. FirmWire émule l'applicatif et bouchonne la L1 ; le `virt_phy` d'osmocom-bb n'exécute aucun code ARM. Ici, les deux cœurs sont réellement émulés.

## Ce qu'il arrive à faire — ça dépend entièrement du *mode*

C'est le point central du projet. Il y a deux familles de fonctionnement, et elles ne prouvent pas la même chose :

| Famille | Qui acquiert la synchro (FB/SB) | Qui décode les infos réseau | Ce que ça démontre |
|---|---|---|---|
| **shunt** | l'hôte (gr-gsm démodule) | gr-gsm | la pile complète, de bout en bout, jusqu'à l'audio — **mais rien sur le DSP** |
| **natif** | le vrai DSP | le DSP | la vérité sur l'acquisition — **ne "campe" pas encore aujourd'hui** |

**En mode shunt** (le mode par défaut, `shunt_legit`), la chaîne GSM entière fonctionne : le téléphone émulé s'accroche au réseau, fait sa *Location Update*, s'authentifie (COMP128v1), chiffre en **A5/1**, échange des SMS, et tient même un **appel voix avec audio entre deux abonnés** (10001 ↔ 10002). Le firmware ARM tourne pour de vrai — mais c'est gr-gsm côté hôte qui démodule, le DSP est court-circuité.

**En mode natif**, le DSP fait le vrai travail. L'état, mis à jour au 2026-08-24 :
- ✅ **le FB (Frequency Burst, la première synchro) est bien acquis par le vrai DSP** — mesuré 437 fois, écrit par la mask-ROM elle-même, sans aucune injection truquée.
- 🔧 le mur actuel est le **décodage du SCH** : la tâche tourne mais son décodeur sort une valeur constante (`0xf8d8`) quelle que soit l'entrée — donc il ne décode pas vraiment.
- ⚠️ le DMA du DSP est modélisé mais désactivé par défaut, ce qui laisse une erreur permanente `DSP_ERR_DMA_PROG`.

## Comment c'est branché

```
 Réseau GSM réel (osmo-bts / fake_trx)  ──►  downlink I/Q
                                              │
              ┌───────────────────────────────┴──────────────┐
              │  QEMU-Calypso                                 │
              │   ARM ◄──── mailbox 0xFFD00000 ────► DSP C54x │
              │  (firmware osmocom-bb non patché)             │
              │   + gr-gsm + modèles RF (twl3025, trf6151)    │
              └───────────────────────────────────────────────┘
```

Le firmware ARM est **interchangeable** : n'importe quel `layer1.highram.elf` issu de l'arbre osmocom-bb boote sans adaptation. C'est le test qui distingue « j'émule la plateforme » de « j'ai fait marcher *ce* binaire précis ».

## Ce que contient le dépôt (les 290 fichiers du bundle)

- **Sources C** (`hw/arm/calypso/`) : le cœur DSP c54x (décodeur d'instructions + MMIO + interruptions), le pont ARM↔DSP, le BSP (livraison des bursts), le séquenceur TPU/TSP (frontend RF), les modèles RF (trf6151, twl3025), le contrôleur DMA.
- **Un pont IPC** (`tools/calypso-ipc-device/`) qui relie l'I/Q de QEMU à la mémoire partagée du transceiver.
- **Beaucoup de scripts shell** (89) : orchestration du lancement, profils de modes, visualiseurs de logs en tmux (fenêtres cœur/DSP/asm/radio/voix pour deux abonnés).
- **Scripts Python** (51) + **tests** (18).
- **Documentation abondante** (46 fichiers, en français) : cartes d'adresses DSP, jeu d'instructions c54x, rapports de run datés, plans d'implémentation voix, architecture des "béquilles" (`crutches`) qui stubbent les branches non encore implémentées.

## L'esprit du projet

Ce qui ressort nettement de la doc, c'est une **discipline anti-illusion** : le projet insiste partout sur le fait qu'un compteur à zéro n'est *pas* une preuve d'absence, que la vérité d'un run est le manifeste imprimé par le modèle (pas la ligne de commande), et que les résultats "shunt" ne prouvent rien sur le DSP. Il y a un système de "sas" (`CALYPSO_FIXES`) pour mettre en quarantaine les correctifs non validés, et des sondes de diagnostic inertes par défaut. C'est un travail de rétro-ingénierie où chaque affirmation est présentée comme vérifiable — et cassable.

En résumé : c'est un banc d'émulation matérielle fidèle du baseband Calypso, capable de faire tourner une pile GSM complète jusqu'à la voix en mode "assisté", et qui pousse progressivement le vrai DSP à faire lui-même l'acquisition radio en mode "natif".

Veux-tu que je détaille un aspect précis — l'architecture DSP/ARM, la chaîne RF, le décodage SCH bloqué, ou le fonctionnement des modes ?

Émulation du **baseband GSM TI Calypso** (Compal E88 / Motorola C1xx) sous QEMU.
Deux cœurs tournent ensemble, reliés par la mailbox API RAM du vrai SoC :

- **ARM** — exécute le firmware [osmocom-bb](https://osmocom.org/projects/baseband)
  **d'origine, non patché** (Layer 1 temps réel).
- **TMS320C54x** — exécute la **vraie mask-ROM DSP** de TI.

À ma connaissance, aucun autre émulateur de baseband ne fait tourner à la fois le
firmware constructeur non modifié *et* le DSP de couche 1. FirmWire (Shannon,
MediaTek) émule le cœur applicatif et stubbe la L1 ; `virt_phy` d'osmocom-bb
n'exécute aucun code ARM et injecte au-dessus de la couche 1. Ici les deux cœurs
sont émulés et se parlent par la mailbox `0xFFD00000`.

> **Le firmware est interchangeable.** N'importe quel `layer1.highram.elf` issu de
> l'arbre osmocom-bb boote sans adaptation — aucune version précise requise, aucun
> patch. C'est le test qui distingue « j'émule la plateforme » de « j'ai fait
> marcher *ce* binaire-là ». Voir [Vérifier soi-même](#vérifier-soi-même).

---

## Où en est le projet, honnêtement

Le statut **dépend entièrement du mode**, et c'est le point le plus important de ce
README. Deux familles :

| | qui acquiert FB/SB | qui décode les SI | ce que ça prouve |
|---|---|---|---|
| **shunt** | hôte | gr-gsm | la pile de bout en bout, jusqu'à l'audio. **Rien sur le DSP.** |
| **natif** | DSP | DSP | la vérité sur l'acquisition. **Ne campe pas aujourd'hui.** |

En famille **shunt**, le mobile campe, fait sa Location Update
(`LOCATION UPDATING ACCEPT` + `TMSI REALLOC COMPLETE`), s'authentifie en
COMP128v1, chiffre en **A5/1**, échange des SMS et tient un **appel voix entre
deux abonnés** avec audio. Mais c'est gr-gsm qui démodule : le firmware ARM
tourne pour de vrai, le DSP non.

En mode **natif**, l'état a changé le **2026-08-24** et ce README a été corrigé en
conséquence :

- ✅ **le FB est acquis par le vrai DSP.** `d_fb_det[0x08f8]` passe à 1 — mesuré
  **437 fois**, toutes écrites par la mask-ROM elle-même à `PC=0x79e4`. L'ARM
  reçoit des `FB0`/`FB1` avec TOA, puissance et angle **réels et variables**,
  calcule son `fn_offset` et appelle `Synchronize_TDMA`.
- ✅ **l'IMR n'est plus à zéro**, les interruptions sont vectorisées, le DSP
  tourne stable (~500 k instructions/s, aucun opcode non implémenté).
- 🔧 **le mur est maintenant le décodage du SCH.** La tâche SB s'exécute, mesure
  un TOA plausible, écrit ses slots — mais `a_sch[0] = 0x8100`
  (`B_BLUD | B_SCH_CRC`), donc `prim_fbsb.c:181` abandonne avant même
  d'assembler le mot SB. Et `a_sch[3]` sort **`0xf8d8`, constant sur 21/21
  écritures**, alors que 10 contenus de burst distincts lui ont été présentés.
  Un décodeur dont la sortie ne dépend pas de l'entrée ne décode pas.
- ⚠️ `d_error_status` vaut **8 = `DSP_ERR_DMA_PROG`** en permanence (404
  occurrences), y compris quand la tâche SB ne tourne quasiment pas. Cause
  documentée dans `hw/arm/calypso/calypso_dma.h` : la file DMA du firmware DSP
  sature parce que le modèle accepte la programmation mais n'exécute aucun
  transfert et ne lève jamais `DMAC0..5`. Module `calypso_dma.c` présent, gaté
  `CALYPSO_DMA=1`, **défaut OFF**.

### La question la plus rentable du moment

Le front-end du démodulateur de la ROM (`0x9f95-0x9fcb`) est un filtre
multi-cadence : il **lit** une entrée circulaire de 638 mots via `AR6` (`0x9fb5`)
et **écrit** sa sortie linéairement via `AR4` (`0x9fb8`) — désassemblage vérifié.

Or il existe **deux mesures du dépôt, prises à six mois d'écart et jamais
confrontées** : `0x4c00` (« `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a`, stride 5 =
polyphase 6 taps ») et `0x2a00` (« DSP READS `0xCAFE` at `0x2a00..0x2a13` via
`PC=0x93a5` »). Elles ne se contredisent pas : ce sont les **deux extrémités du
même étage**. Le défaut actuel dépose en `0x2a00`.

Si `AR4` vaut `0x2a00`, alors on alimente la **sortie** du démodulateur, son
entrée n'est nourrie par personne, et le Viterbi mange de l'I/Q — ce qui
expliquerait d'un coup la constante `0xf8d8` et le CRC toujours faux. La sonde
`DEMOD-IO-PROBE` (`CALYPSO_DEMODIO=1`, lecture seule) tranche en un run.

Autre limite à connaître : l'ARM est modélisé par un cœur ARM946 (ARMv5TE) là où le
Calypso a un ARM7TDMI (ARMv4T). Choix assumé, voir `hw/arm/calypso/calypso_mb.c:303`.

### Matrice détaillée

| # | Objectif | shunt | natif | preuve |
|---|---|---|---|---|
| 1 | **FB (sync)** | ✅ | **✅** | natif : `FBDET-WR 0x0000 -> 0x0001` ×437 à `PC=0x79e4`, `FB1 (…): TOA=39, Power=-52dBm` |
| 1b | **SB / SCH** | ✅ | 🔧 WIP | shunt : `=> SB 0x0125011c: BSIC=7` ; natif : `a_sch[0]=0x8100`, CRC toujours faux |
| 2 | RXLEV serving | ✅ | ✅ | `RLA_C -53 dBm`, C1/C2 > 0 |
| 3 | Camp (C3) | ✅ | ⬜ | `C3 camped normally` |
| 4 | Location Update | ✅ | ⬜ | `LOCATION UPDATING ACCEPT (lai=001-01-1)` |
| 5 | Authentification | ✅ | ⬜ | COMP128v1, `gsup:rx:auth_tuples = 10` |
| 6 | **Chiffrement A5/1** | ✅ | ⬜ | `CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)` ×7 |
| 7 | SMS MO / MT | ✅ | ⬜ | `SMS from 10002` |
| 8 | Ctrl-C / re-camp | ✅ | ⬜ | reset L1 câblé (`d_dsp_page=0`) |
| 9 | Union SDCCH SS0-7 | ✅ | — | LU quelle que soit la sous-voie |
| 10 | **Voix TCH/F** | ✅ | ⬜ | appel **10001 ↔ 10002** abouti, état `ACTIVE`, GAPK codec `fr` des deux côtés |
| 11 | FB-STREAM `0x9213/0x9215` | ✅ | ✅ | rampe relue par le démod |
| 12 | `d_fb_det` natif | — | **✅** | **résolu 2026-08-24** |
| 13 | Décodage SCH natif | — | ⬜ | sortie constante `0xf8d8`, indépendante de l'entrée |
| 14 | DMA du DSP | — | ⬜ | `DSP_ERR_DMA_PROG` permanent ; `calypso_dma.c` gaté, défaut OFF |

Source de vérité par mode :
**[`RUN RESULTS.md`](https://github.com/bbaranoff/qemu-calypso/blob/main/run_results.md)**. En cas de conflit entre
docs, c'est lui qui prime.

---

## Démarrer

```bash
git clone https://github.com/bbaranoff/osmo_egprs
cd osmo_egprs
./build.sh
```

Puis, **chaque mode a sa porte d'entrée — elles ne sont pas interchangeables** :

```bash
# shunt_legit (le defaut) : la pile complete, jusqu'a l'appel voix
cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' ./start-direct.sh

# native : le vrai DSP a la manoeuvre
cd /opt/GSM/qemu-src && CALYPSO_MODE=native ./run.sh
```

Pour arrêter, la même porte avec `--stop`. **Si une relance coince** — port déjà
pris, pile à moitié morte : faire le `--stop`, **puis tuer les python restants**,
qui survivent au teardown et retiennent les FIFO I/Q et les sockets :

```bash
pkill -f python3 ; pkill -f python ; sleep 2 ; pgrep -af 'python|qemu-system-arm'
```

> ⚠️ **Le lancement n'est pas cosmétique.** `shunt_legit` démarré par
> `run.sh --restart` sans `CALYPSO_BRIDGE=pont` ne monte pas : la BTS n'atteint
> pas le transceiver (`send() failed on TRXD … Connection refused`, mesuré 1268
> fois), le réseau ne répond jamais et la Location Update **expire** (`T3211`
> ×74) au lieu d'être acceptée. Par la bonne porte : 3 erreurs BTS, LU acceptée.

> ⚠️ **Les deux lanceurs n'écrivent pas au même endroit.**
> `run.sh` → `/tmp/calypso/logs` · `start-direct.sh` → **`/tmp/osmo-nitb/logs`**.
> Analyser le mauvais répertoire donne des compteurs périmés qui ressemblent à
> des pannes.

Build seul : `cd build && ninja qemu-system-arm`.
Détail des modes et commandes : **[QUICK_START.md](QUICK_START.md)**.

## Vérifier soi-même

Ce projet fait des affirmations vérifiables en quelques minutes. Elles sont faites
pour être cassées :

**1. Le firmware est le vrai, non modifié.** Prenez un `layer1.highram.elf`
construit depuis n'importe quel commit d'osmocom-bb, remplacez celui fourni,
relancez. S'il ne boote pas, l'affirmation est fausse — ouvrez une issue.

**2. Les ✅ shunt ne prouvent rien sur le DSP.** Passez en `CALYPSO_MODE=native` :
le mobile ne campe pas.

**3. Le FB natif est bien produit par le DSP, pas injecté.** En `native`, comptez
les écritures de la mask-ROM elle-même :

```bash
grep -c 'FBDET-WR .*0x0000 -> 0x0001' /tmp/calypso/logs/qemu.log   # ~437
grep    'FBDET-WR .*0x0000 -> 0x0001' /tmp/calypso/logs/qemu.log | \
  grep -oE 'PC=0x[0-9a-f]+' | sort -u                              # PC=0x79e4
```

Aucune gate d'injection n'est posée (`CALYPSO_INJECT_*=0`, `CALYPSO_CANNED`
annonce `FBDET=0 TOA=0 PM=0 SNR=0 ANGLE=0`). Si vous trouvez une béquille qui
produit ce compteur, l'affirmation est fausse.

## Rapport de run

`tools/rapport-run.sh` produit un état complet d'un run, en lecture seule,
pendant que la pile tourne :

```bash
tools/rapport-run.sh                        # run courant, LOG_DIR auto-detecte
tools/rapport-run.sh /tmp/ref-shunt-legit   # une photo archivee
tools/rapport-run.sh <dir> <sortie.txt>
```

Sa règle de conception : **un compteur nul n'est jamais une absence prouvée**. Il
affiche pour chaque grandeur la valeur et une ligne témoin, et annonce un zéro
comme « motif jamais vu ». Il signale aussi un journal qui ne grossit pas alors
que QEMU tourne — signe qu'on regarde le mauvais répertoire.

## Résultats chiffrés

- **[`RAPPORT_COMPLET_20260824.md`](hw/arm/calypso/doc/RAPPORT_COMPLET_20260824.md)**
  — relevé complet du banc `shunt_legit` : chaîne de bout en bout, valeurs
  d'authentification HLR/MSC (Ki, RAND, SRES, **Kc**), A5/1, appel voix
  10001 ↔ 10002 avec les deux machines à états CC.
- [`run_results.md`](run_results.md) — mesures reproductibles (profondeur ring,
  débits, temps LU, éviction), chaque chiffre confronté à une règle de décision
  posée d'avance.
- [`RAPPORT_DFBDET.md`](RAPPORT_DFBDET.md) — enquête historique sur `d_fb_det = 0`.
  ⚠️ **Le symptôme qu'il analyse est résolu depuis le 2026-08-24** ; le document
  reste pour sa méthode et ses hypothèses écartées.

---

## Sondes de diagnostic DSP

Toutes en lecture seule, inertes par défaut, dans `calypso_c54x.c` :

| gate | sonde | ce qu'elle répond |
|---|---|---|
| `CALYPSO_SBFN=1` | `SBFN-PROBE` | quelle trame est en DARAM quand la tâche SB la lit (`fn`, `fn%51`, dépôts depuis le SB précédent, amplitude) |
| `CALYPSO_SUBC=1` | `SUBC-PROBE` | dividende, quotient et `T` de la division qui alimente les coefficients — **avec un contrôle armé sur `0x989f`** pour attester que la sonde est vivante |
| `CALYPSO_SUBC=1` | `MVDD-PROBE` | la garde `0x7ccd` et les copies `0x7ce0`/`0x7ce4` qui remplissent les blocs 3 et 4 |

### Sas ISA ouvert : `CALYPSO_FIX_LK_SHFT`

Défaut **OFF**. Le décodeur applique au champ de décalage des formes
`ADD/SUB/LD/AND/OR/XOR #lk` — **4 bits** (`op & 0xF`) — la règle de signe d'un
champ de **5 bits**. On ne représente pas −16..15 sur 4 bits : la conversion est
incohérente en soi, et la table binutils versée au dépôt
(`doc/opcodes/tic54x-opc.c`) donne bien `OP_SHFT` (non signé) pour `ld`.

Effet mesuré en `0x7d19` (`f02f 0001` = `LD #1, SHFT=15, A`) :

| | sas OFF | sas ON |
|---|---|---|
| `A` avant le `SFTA` | `0x0000000000` | `0x0000008000` |
| quotient en `0x7d1e` | `0x0000` | `0xfff4` |
| `T` au `MPY` `0x81e4` | `0x0000` (27/27) | `0xfff4` |
| `Synchronize_TDMA` | 8 | 13 (pas de régression) |

Validé aux niveaux 1 (formel) et 2 (grandeur physique) de
`environnement/fixes.env` ; le niveau 3 reste à faire. La portée est
volontairement étroite (sous-codes 1–5, `ADD` non touché) pour que la mesure
reste interprétable.

---

## Architecture

```
   osmo-bts / fake_trx / osmo-trx  ──►  downlink GSM réel (I/Q 4 SPS)
                                          │
                    ┌─────────────────────┴───────────────────┐
                    │  QEMU-Calypso (machine `calypso`)        │
                    │                                          │
                    │   ARM ◄────API RAM (0xFFD00000)────► DSP │
                    │   (osmocom-bb, non patché)        (C54x) │
                    │        ▲                                 │
                    │        │ real_fb_read (intercept MMIO)   │
                    │   gr-gsm + twl3025/DECAN + trf6151       │
                    └──────────────────────────────────────────┘
```

**Chaîne host** (modes shunt) : gr-gsm décode SB/SI, les modèles RF (twl3025/DECAN,
gain trf6151) produisent AFC/PM/rxlev, le tout injecté dans l'API RAM → l'ARM campe.

**DSP natif** : le vrai corrélateur mask-ROM acquiert le FB depuis la DARAM
`0x2a00`. ⚠️ **L'écrivain effectif de ce tampon est le DSP lui-même** (il recopie
depuis son port BSP par `PORTR`, `writer_kind=10`), et non l'hôte : une écriture
hôte y est écrasée. Conséquence directe, `CALYPSO_BSP_IQ_SHIFT` **n'a aucun effet
sur le chemin vivant** — toute lecture d'amplitude passant par cette gate est
fausse.

### Composants (`hw/arm/calypso/`)

| Fichier | Rôle |
|---|---|
| `calypso_c54x.c` | Cœur DSP TMS320C54x (décodeur ISA, MMIO, IT) + sondes de diagnostic |
| `calypso_dsp_shunt.c` | Chaîne host FBSB : real_fb_read, feed_iq, feed_si |
| `calypso_bsp.c` | BSP : livraison bursts DARAM, tee IQ, BRINT0 |
| `calypso_dma.c` | Contrôleur DMA interne du C54x — **gaté `CALYPSO_DMA=1`, défaut OFF** |
| `calypso_trx.c` | Horloge/FN, miroir MMIO ARM↔DSP |
| `calypso_arm2dsp.c` | Pont ARM→DSP (go-live, `d_ctrl_system`) |
| `calypso_tpu.c` / `calypso_tsp.c` | Séquenceur TPU + protocole TSP (frontend RF) |

---

## Modes

| profil | FB/SB | SI | ce qu'il prouve |
|---|---|---|---|
| `shunt_legit` | hôte | gr-gsm | **le défaut.** La pile de bout en bout : camp, LU, auth, A5/1, SMS, appel voix + audio |
| `native_twl` | hôte / TWL | **DSP** | le DSP traite-t-il le SI ? On lui donne la synchro pour poser la question |
| `native` | DSP | DSP | la vérité sur l'acquisition. **FB acquis**, SCH pas encore décodé |
| `native_helped` | DSP, entrée reroutée | DSP | ⚠️ **sous béquille** — toute mesure prise ici doit être citée comme telle |
| `empty` | rien | rien | banc gate par gate ; neutralise `modes.env`, et lui seul |

**Frontière à ne pas franchir sans changer de nom** : dès que les SI viennent de
gr-gsm, on shunte le DSP — c'est `shunt_legit`, pas un mode natif. Les deux seules
portes sont `CALYPSO_SHUNT_FEED_SI` et `CALYPSO_INJECT_ACD` ;
`run_modules/01-profil.sh` proteste si elles sont rallumées sous un profil natif.

### Piège connu : notation en prose ≠ valeur

La combinaison « SHUNT_LEGIT + DSP natif + NO_CANNED » est une **description**, pas
une valeur d'environnement. Posé littéralement,
`CALYPSO_SHUNT_LEGIT=DSP,NO_CANNED` **ne fait rien** : les 16 tests du modèle
comparent `*e == '1'` et `calypso.env:19` teste `= "1"`. Toute autre valeur est
fausse partout — on croit tourner en famille shunt alors qu'on est en natif nu.

La combinaison se pose par `CALYPSO_MODE=shunt_legit_no_inject` ou `native_twl`,
qui la traduisent en gates individuelles (vérifié le 30/07).

### Variables

Tout se pilote par `CALYPSO_*` (voir `environnement/`), **toutes overridables en
CLI**. Un profil ne pose que des `:=` : la CLI garde le dernier mot, et **la seule
source de vérité sur ce qu'un run a réellement obtenu est le manifeste imprimé par
le modèle** (`qemu-manifest.log`), jamais la ligne de commande.

Corollaire mesuré : `calypso_shunt_legit.env` pose `CALYPSO_DSP_RUN_C54X:=0`, donc
un `CALYPSO_DSP_RUN_C54X=1` en CLI **réveille le c54x dans le banc shunt** — c'est
le seul moyen aujourd'hui de réunir « c54x actif » et « chaîne complète ».

---

## Documentation

Toute la doc vit sous `hw/arm/calypso/doc/`.

| Doc | Contenu |
|---|---|
| **[RAPPORT_COMPLET_20260824.md](hw/arm/calypso/doc/RAPPORT_COMPLET_20260824.md)** | Relevé complet du banc shunt_legit : chaîne, Kc/HLR/MSC, A5/1, appel voix |
| [QUICK_START.md](QUICK_START.md) | Build, lancer, modes, vérifications |
| [TODO.md](hw/arm/calypso/doc/TODO.md) | TODO consolidé (P0/P1/P2), cadré par mode |
| [SHUNT_LEGIT_ADDRESS_MAP.md](hw/arm/calypso/doc/SHUNT_LEGIT_ADDRESS_MAP.md) | Mapping canonique des cellules DSP, chaîne FB/SB/rxlev/a_cd |
| [DSP_ADDRESS_MAP.md](hw/arm/calypso/doc/DSP_ADDRESS_MAP.md) · [DSP_ARM_LINKAGE.md](hw/arm/calypso/doc/DSP_ARM_LINKAGE.md) | Cartes d'adresses DSP + correspondance ARM↔DSP |
| [VOIX_PLAN.md](hw/arm/calypso/doc/VOIX_PLAN.md) | Plan d'implémentation TCH/F host-side |
| [CALYPSO_HW.md](hw/arm/calypso/doc/CALYPSO_HW.md) · [hardware-map.md](hw/arm/calypso/doc/hardware-map.md) | Carte matérielle du SoC |
| [C54X_INSTRUCTIONS.md](hw/arm/calypso/doc/C54X_INSTRUCTIONS.md) · [opcodes/](hw/arm/calypso/doc/opcodes/) | Jeu d'instructions TMS320C54x |
| [DSP_ROM_MAP.md](hw/arm/calypso/doc/DSP_ROM_MAP.md) | Carte de la mask-ROM DSP |
| [SERCOMM_GATE_ARCHITECTURE.md](hw/arm/calypso/doc/SERCOMM_GATE_ARCHITECTURE.md) | Canal Sercomm/L1CTL |
| [datasheets/](hw/arm/calypso/doc/datasheets/README.md) | PDF constructeur (TI SPRU, FreeCalypso) |
| [archive/](hw/arm/calypso/doc/archive/README.md) | Doc historique, pistes closes |

---

*Basé sur QEMU. La documentation amont d'origine est conservée sous `docs/`.*
