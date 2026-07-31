# La chaîne RF Calypso — matériel réel, modèle QEMU, et ce qu'on a mesuré

> **Deux sources, fusionnées maillon par maillon.**
>
> 1. Le matériel : wiki Osmocom, *Typical TI Calypso baseband modem design*
>    <https://osmocom.org/projects/baseband/wiki/TypicalCalypsoModemDesign>
>    (design standard des Motorola C1xx / Compal, Openmoko GTA01 et GTA02).
> 2. Le modèle et les mesures : ce dépôt, relevés du 2026-07-28 au 30.
>
> Chaque maillon est donné trois fois — **ce que fait le silicium**, **ce que
> fait notre modèle**, **ce qu'on a mesuré** — puis un verdict. C'est la
> confrontation qui a de la valeur : elle a déjà retourné trois conclusions
> qu'on tenait pour acquises (§6).

---

## 1. Schéma-bloc

```
            ┌──────────────────────────────────────────── RFCLK (26 MHz) ──────────┐
            │                                                                      │
 ┌──────────┴───────────┐                        ┌──────────────┐        ┌─────────┴──────┐
 │       CALYPSO        │──── CLK13M ───────────▶│   TWL3025    │        │    TRF6151     │
 │        (DBB)         │──── CLK32K ───────────▶│    (Iota)    │        │    (Rita)      │
 │                      │                        │              │        │  Transceiver   │
 │  ┌────────────────┐  │◀───── BSP ────────────▶│     ABB      │        │  Mixers / VCO  │
 │  │  ARM7  +  DSP  │  │◀───── USP ────────────▶│              │        │      PLL       │
 │  │  API RAM / RIF │  │                        │   BDL / BUL  │        │                │
 │  └────────────────┘  │                        │      │       │        │                │
 │                      │─────── TSP ───────────▶│      └── I/Q Analog ─▶│                │
 └──────────┬───────────┘                        │         AFC Analog ──▶│                │
            │                                    │         APC Analog ─┐ └────────┬───────┘
            │  TSP Serial                        └──────────────────────┼──────────┘
            │  TSP Parallel                                            │      GSM / DCS·PCS
            │                                                          ▼            │
            │                                              ┌────────────────┐       │
            ├─────────────────────────────────────────────▶│    RF3166      │       │
            │                                              │    RF PA       │       │
            │                                              └───────┬────────┘       │
            │                                                      │ DCS·PCS / GSM  │
            │                                              ┌───────▼────────┐       │
            └─────────────────────────────────────────────▶│    ASM4532     │◀──────┘
                                                           │ Antenna Switch │
                                                           └───────┬────────┘
                                                                   │
                                                                 ╲ │ ╱  antenne
```

| couleur (schéma d'origine) | signification |
|---|---|
| **Jaune** | horloges — `RFCLK`, `CLK13M`, `CLK32K` |
| **Rouge** | interfaces série numériques, type SPI — `BSP`, `USP` |
| **Vert** | I/Q analogique différentiel en bande de base, et `AFC` / `APC` analogiques |
| **Magenta** | signaux RF |
| **Bleu** | `TSP`, *time sequence port* — parfois parallèle, parfois série |

---

## 2. Chemin de réception — maillon par maillon

### 2.1 Antenne → ASM4532 → filtres SAW

**Matériel.** Le signal de la BTS est capté par l'antenne et atteint le
commutateur d'antenne (diodes ou MEMS), configuré pour relier l'antenne à **l'un**
des chemins Rx GSM / DCS / PCS. Puis filtres SAW, qui retirent le hors-bande.

**Modèle.** **Absent.** Aucun fichier ne modélise l'ASM4532 ni les SAW.
Le mapping des lignes de commande existe pourtant côté firmware
(`board/compal/rffe_dualband.c`) : `TRENA` = TSPACT(6), **actif bas** ;
`GSM_TXEN` = TSPACT(8), **actif bas**.

**Mesure.** `calypso_tsp.c` met `tsp.act` à jour sur `TPUI_TSP_ACT_L/U` puis se
contente de le **journaliser** — aucun consommateur en aval.

**Verdict.** Sans effet sur la réception : le downlink est synthétique, le chemin
Rx est supposé connecté d'office. Compte pour l'UL et pour détecter une émission
demandée sans commutation d'antenne — faute aujourd'hui invisible.

### 2.2 TRF6151 / Rita — transceiver zero-IF

**Matériel.** Amplifie, filtre, **mélange avec son VCO interne**, exporte en
**I/Q analogique**. Architecture **zero-IF** : la bande de base est centrée sur DC.

**Modèle.** `calypso_trf6151.c` — modèle de **gain** uniquement, porté de
`rf/trf6151.c` : le firmware programme `REG_RX` par TSP (dev 1), on décode
FE high/low (bits 10:9) et VGA (bits 15:11), on recompose
`total_gain = SYSTEM_INHERENT_GAIN(71) + trf6151_gain`, et on en déduit l'`a_pm`
à poser pour qu'un niveau RF cible soit rapporté.

**Mesure.** Le `rxlev` suit : `a_pm = (rf + 71 + gain) × 64`, et le mobile sort
`snr=28` avec un `BSIC=7` réel sur le banc de référence.

**Verdict.** Suffisant pour l'AGC et le rxlev. Le mélange et le VCO ne sont pas
modélisés — inutile, la source fournit déjà de la bande de base.
⚠️ **Le zero-IF confirme la lecture de `corr_iq`** : un `|DC|` faible est attendu,
et le pic FCCH à **+67 708 Hz** est un décalage réel par rapport à DC.

### 2.3 TWL3025 / Iota — l'ABB

**Matériel.** Échantillonne l'I/Q analogique par **ADC** et l'envoie en **flux
série de paires I+Q vers le DBB par le BSP**. C'est aussi lui qui produit
l'**AFC** (accord du **VCXO**) et l'**APC**.

**Modèle.** `calypso_twl3025.c` + `calypso_iota.c`. La rotation AFC/phase est
appliquée **sur les échantillons livrés**, dans `calypso_bsp.c:1595`
(`calypso_twl3025_apply_phase()` juste avant `c54x_bsp_load`) — **hors de toute
garde de shunt**, donc active y compris en profil natif.

**Mesure.** Le corrélateur natif reçoit donc des échantillons corrigés en
AFC/phase. `REAL-FB … coh=0.999 dphi=0.387 AFC=-186` sur le banc de référence.

**Verdict, avec une nuance nommée.** Le glossaire dit *« AFC : accord du VCXO par
l'ABB »*. Notre modèle applique une **rotation de phase sur les samples**. Un
accord de VCXO décale **tout**, y compris la base de temps ; une rotation ne
décale que la porteuse. Suffisant pour démoduler, **faux pour ce qui dépend du
timing**. À retenir avant d'accuser un désalignement de trame.

Corollaire acquis : le TWL est un **front mixte** (convertisseurs + AFC/APC
analogiques), pas un bloc numérique. L'idée que « le TWL pourrait faire le FBSB »
est définitivement périmée — voir `ETAT_ACTUEL §12.9`.

### 2.4 BSP → RIF → DMA → API RAM  ⭐ **l'écart principal**

**Matériel.** Dans le DBB, le signal est *« **reçu sur le RIF** (Radio Inter Face)
et **transféré par DMA dans l'API RAM du DSP** »*, puis traité par le cœur DSP,
puis converti en résultats envoyés à l'ARM **via l'API RAM**.

**Modèle.** `CALYPSO_BSP_DARAM_FORCE=1` écrit les 296 mots **directement en DARAM
`0x4c00`** (`CALYPSO_BSP_DARAM_ADDR`). Le mot **RIF** n'apparaît nulle part dans
le modèle. `calypso_dma.c` existe mais n'est pas sur ce chemin.

**Mesures**, toutes du 30/07 :

| question | mesure |
|---|---|
| le contenu est-il bon ? | **oui** — capture non racy depuis le thread CPU : 389 bursts, dont **40 à `coh=0.998`, `dphi=+1.567` (FCCH@1SPS exact)**, pic `+67 708 Hz` |
| côté hôte aussi ? | **oui** — `bursts.cfile` 400/400 cohérents, `rxdump` 24/24 |
| le DSP lit-il `0x4c00` ? | **jamais** — `CALYPSO_WATCH_RD_ADDR=0x4c40` : **0 lecture** sur 26,8 M d'instructions, sonde prouvée sur le chemin des opcodes (`data_read()` → `data_read_locked()`, `:1757`) |
| la machinerie de port tourne-t-elle ? | **non** — `BSP LOAD=0` dans **tous** les runs |
| la routine d'acquisition est-elle appelée ? | **non** — `data[0x43d9]` = `0xbb0e` (`portw` / `rpt` / **`portr` = lecture des échantillons**), jamais atteinte |

**Verdict — et c'est un recadrage.** « Personne ne lit `0x4c00` » n'est **pas une
anomalie** : sur silicium, le DSP **ne lit pas** les échantillons, on les lui
**dépose**. La bonne question n'est donc pas « qui lit ? » mais **« la DMA a-t-elle
signalé un dépôt, et à la bonne adresse ? »**. Or le matériel dépose dans l'**API
RAM**, qui fait **8 kWords (16 ko)** — et nous écrivons en DARAM `0x4c00`, **en
dehors**. ⚠️ Notre moniteur mailbox ne couvre que `0x0800..0x0FFF`, soit **le quart**
de l'API RAM : son silence au-delà ne prouve rien.

C'est la catégorie « wire-only » de l'audit du 26/07 : du câblage, pas du calcul.

### 2.5 Le cœur DSP

**Matériel.** Traite le signal et produit des résultats (par exemple un bloc MAC)
transmis à l'ARM via l'API RAM. Il exécute les commandes de l'ARM **à
l'interruption de trame TDMA suivante**.

**Modèle.** `calypso_c54x.c`, interprété. L'IT trame TPU→DSP n'était **pas
livrée** avant le 30/07 : `calypso_tpu.c` la délivre désormais
(`CALYPSO_TPU_DSP_FRAME_IT`), en respectant `TPU_CTRL_DSP_EN` et `ICTRL_DSP_FRAME`
(actif bas). Le wiki **confirme** que c'est du câblage manquant, pas une béquille :
sans cette IT, aucune commande de l'ARM ne peut être exécutée au bon moment.

**Mesures.**

| | |
|---|---|
| vitesse | **~96× plus lent** que le silicium (680 k insn/s contre 65 M/s) → tick TDMA à ~10 Hz au lieu de 217 |
| IT trame prise | `PC=0x00f0` — 5 349 passages sur un run de 234 s |
| tâches reçues | `task=5` (FB), `task=6`, `task=1`, et **`task=24`** (ALLC / CCCH) jusqu'à 62 fois |
| ce qu'il publie | `d_fb_det` **1 396 écritures, 100 % à `0x0000`** (depuis `0xb2cc` et `0x778a`) ; `a_pm` 1 351, toutes nulles (`0xb2d2`) ; `a_cd` **0** |
| ce qu'il fait à la place | `0x08dc` **14 486** écritures `0x0074 → 0x0074` — valeur **inchangée** ; `d_error_status` 7 244 fois `0x0000 → 0x0000` |

**Verdict.** Le DSP tourne, il est servi, il est sollicité — et sa boucle réécrit
les mêmes valeurs sans jamais produire de résultat.

### 2.6 L'ARM et la pile

**Matériel.** L'ARM fait tourner la pile GSM.

**Mesure — un défaut distinct, trouvé en route.** Le firmware **jette** les
rapports de burst : `prim_rx_nb.c:80`, `l1s_nb_resp()` fait un *early return*
quand `d_burst_d != burst_id`. Tout l'aval est sauté — TOA, PM, `freq_err`, SNR,
entrée de la boucle AFC, boucle TA, gain, et l'assemblage du bloc de 4 bursts.
Côté modèle, `d_burst_d` (`0x0829`) est écrit **1 017 fois depuis `@0xb007`**, et
le firmware **lit 2 quand il attend 1** — d'où les lignes `BURST ID 2!=1` d'osmocon.

**Verdict.** Le compteur de burst du modèle n'est pas en phase avec celui du
firmware. Indépendant du corrélateur, avec sa propre piste.

---

## 3. Chemin d'émission — plus court, et moins avancé

1. L'**ARM** écrit données + commandes dans l'API RAM.
2. Le **DSP** exécute à l'IT de trame suivante : FEC, entrelacement, chiffrement
   optionnel, puis **envoie les bits du burst à l'ABB par le BSP**.
3. Le **TWL3025** range les bits dans son *burst buffer*, les passe dans un
   **modulateur GMSK matériel** déclenché par **BULENA sur le TSP**, et sort un
   I/Q analogique GMSK. → notre `calypso_iota.c` suit BDLENA/BULENA mais ne
   modélise ni le buffer ni le modulateur. Cela explique pourquoi, côté DSP, il
   n'y a « que » des bits : la modulation n'est pas son travail.
4. Le **TRF6151** mélange avec le VCO. → non modélisé côté Tx.
5. Le **RF3166** amplifie **selon le niveau analogique de l'APC**. → **absent**.
   Ligne de commande : `PA_ENABLE` = TSPACT(1), **actif haut**.
6. L'**ASM4532** relie le PA à l'antenne **pour la durée du burst**. → **absent**.
   Le caractère *fenêtré* est l'essentiel : un PA actif hors fenêtre est une faute
   que rien ne détecte aujourd'hui.

Séquence exacte de `rffe_mode()` (firmware) : au repos
`tspact |= TRENA | GSM_TXEN` (les deux désassertées, actif bas) et
`tspact &= ~PA_ENABLE` ; en émission `tspact &= ~TRENA`, plus
`tspact &= ~GSM_TXEN` si GSM900, et `tspact |= PA_ENABLE`.

---

## 4. Couverture du modèle, en un tableau

| bloc réel | fichier | état |
|---|---|---|
| CALYPSO ARM7 + périphériques | `calypso_mb.c`, `calypso_soc.c` | modélisé |
| Cœur DSP C54x | `calypso_c54x.c` | interprété — ⚠️ ~96× trop lent |
| API RAM ARM↔DSP | `calypso_trx.c`, `calypso_mailbox.c` | modélisée ; moniteur limité aux mots `0x0800..0x0FFF` |
| **RIF + DMA** | `calypso_dma.c` | **partiel — écart principal, §2.4** |
| TPU / TSP | `calypso_tpu.c`, `calypso_tsp.c` | séquenceur AT/WAIT, TSPACT latché mais **sans consommateur** |
| BSP | `calypso_bsp.c` | **contourné** (écriture DARAM directe) |
| TWL3025 / Iota | `calypso_twl3025.c`, `calypso_iota.c` | BDLENA/BULENA, rotation AFC/phase |
| TRF6151 / Rita | `calypso_trf6151.c` | modèle de gain → `rxlev` |
| **RF3166** (PA) | — | **absent** |
| **ASM4532** (commutateur) | — | **absent** |
| USP | — | **absent** (hors chemin des samples) |
| SAW, antenne | — | hors périmètre |

Non modélisé côté horloges : **`RFCLK` (26 MHz) est généré par le transceiver** et
alimente le Calypso ; `CLK13M` est fourni **par le DBB** à l'ABB ; `CLK32K` = RTC.
Conséquence de séquencement qu'on ne peut pas reproduire : `RITA_RESET` = TSPACT(0)
couperait en principe l'horloge maître du DBB.

---

## 5. Les écarts, par gravité

1. **Le dépôt des échantillons** (§2.4) — matériel : RIF + DMA → **API RAM**.
   Nous : écriture directe en **DARAM `0x4c00`**, jamais lue, sans DMA. C'est le
   seul écart dont dépend la question ouverte du projet.
2. **Les lignes TSPACT ne pilotent rien** — `tsp.act` est latché puis journalisé.
   Bloque la modélisation du PA et du commutateur, donc l'UL.
3. **L'AFC est une rotation de phase, pas un accord de VCXO** — approximation
   correcte pour la démodulation, fausse pour le timing.
4. **Le compteur `d_burst_d` est déphasé** — le firmware rejette les rapports
   (`prim_rx_nb.c:80`), silencieusement.
5. **RF3166 et ASM4532 absents** — côté Tx uniquement.

---

## 6. Ce que cette page a retourné dans nos conclusions

- **« Personne ne lit `0x4c00` » n'est pas une anomalie.** Mesuré comme un mur,
  c'est en fait le comportement conforme : le DSP ne lit pas, on lui dépose.
- **Le TWL ne peut pas « faire le FBSB »** — c'est un front mixte. La formulation
  de l'échelle de fidélité en sort confirmée.
- **`CALYPSO_TPU_DSP_FRAME_IT` est du câblage**, pas une béquille : le wiki dit
  explicitement que le DSP exécute les commandes à l'IT de trame.

À rapprocher des rétractations déjà consignées dans `TODO.md` §0bis-§0septies —
`0xab38` n'est pas un bouchon mais un RET **partagé** écrit par une init littérale,
la racine « IMR shadow `0x435b` » est démentie (`0x52ed` sain, storm indépendant),
et le storm n'était pas le conflit béquille/init.

---

## 7. Glossaire

| sigle | signification |
|---|---|
| **ABB** | *Analog Base Band* — le TWL3025 / Iota |
| **DBB** | *Digital Base Band* — le Calypso |
| **AFC** | *Automatic Frequency Correction* — accord du VCXO **par l'ABB** |
| **APC** | *Automatic Power Correction* — enveloppe de puissance Tx, ABB → PA |
| **BSP** | *Baseband Serial Port*, type SPI |
| **USP** | *uController Serial Port* |
| **TSP** | *Time Sequence Port* — séquencement piloté par le TPU |
| **RIF** | *Radio Inter Face* — l'entrée du flux BSP dans le DBB, côté DMA |
| **VCO** | *Voltage Controlled Oscillator* |
| **RFCLK** | horloge maître 26 MHz, **générée par le transceiver** |
| **CLK13M** | horloge système 13 MHz, fournie par le DBB |
| **CLK32K** | horloge RTC 32,768 kHz |

Implémentation réelle : PCB du Motorola C123. Silicium Calypso (couche métal
supérieure) : <https://siliconpr0n.org/archive/doku.php?id=infosecdj:ti:d751749zhh>

---

## 8. Règles de lecture, rappelées ici parce qu'elles ont coûté cher

- `data[]` = ce que le **DSP** a écrit ; `api[]` = ce que l'**hôte** a écrit. Aucun
  jugement sur le corrélateur ne se prend sur `api[]`.
- Le **manifeste** imprimé par le modèle est la seule source de vérité sur ce
  qu'un run a obtenu — jamais la ligne de commande.
- Le moniteur mailbox ne couvre que **`0x0800..0x0FFF`**. Son silence sur la DARAM
  (`0x2a00`, `0x4c00`, `0x3f92`, `0x43d8`) ne prouve rien : utiliser
  `WATCH-WR` / `WATCH-RD`.
- Le **rendu du désassembleur est le premier producteur d'erreurs** du projet
  (`TODO.md` §0). Une correspondance *mesurée* (une écriture observée qui colle à
  un littéral) vaut ; une lecture d'opcode seule ne vaut pas.

La question ouverte — « le DSP émulé corrèle-t-il et publie-t-il ? » — vit dans
`TODO.md` §0bis et suivants, avec ses mesures, ses règles de décision posées
d'avance, et ses rétractations.
