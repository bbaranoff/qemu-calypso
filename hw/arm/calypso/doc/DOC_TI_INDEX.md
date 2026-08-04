# Documents TI — index et correspondance avec le code

Les trois PDF de ce répertoire sont la **source de vérité matérielle** du modèle.
Quand un commentaire du code et un de ces documents divergent, c'est le document
qui gagne — et si la mesure contredit les deux, c'est la mesure. Trois erreurs
majeures corrigées le 2026-08-03 venaient toutes de la même cause : un commentaire
affirmait un fait matériel que personne n'avait vérifié dans le doc.

| Fichier | Réf. TI | Contenu | Statut |
|---|---|---|---|
| `ti-calypso1.pdf` | **CAL000** — *HERCROM400G2*, Ver 1.3, 51 p. | Spécification système : blocs, interruptions, DMA, cartes mémoire ARM et DSP | ⚠️ marqué *UNDER NON DISCLOSURE AGREEMENT — DO NOT COPY* |
| `ti-calypso2.pdf` | **CAL207** — *Register Mapping*, Ver 1.05, 178 p. | Carte des registres, champ par champ, avec valeurs de reset | ⚠️ idem |
| `spru172c.pdf` | SPRU172C | Jeu d'instructions TMS320C54x (document TI public) | — |

> **Note pratique** : `hw/arm/calypso/doc/` n'est pas dans `.gitignore` et ce dépôt
> pousse vers un remote public. Les deux documents CAL portent une mention de
> non-divulgation. À arbitrer avant le prochain `git add` — une ligne
> `hw/arm/calypso/doc/ti-calypso*.pdf` dans `.gitignore` suffit si on veut les
> garder en local sans les publier.

## Extraction

```bash
pdftotext -layout doc/ti-calypso1.pdf /tmp/cal1.txt   # CAL000
pdftotext -layout doc/ti-calypso2.pdf /tmp/cal2.txt   # CAL207
```

## Sections utilisées, et ce qu'elles ont tranché

### CAL000 (`ti-calypso1.pdf`)

| § | Sujet | Conséquence dans le code |
|---|---|---|
| **5.1** | **Table des 17 interruptions du DSP** | A remplacé la table SPRU131 du C54x *générique* qui était dans `calypso_c54x.h` — elle était **décalée d'un cran** (le C54x générique a 4 lignes externes avant TINT, le Calypso 3). D'où `BRINT0`/`DMAC0`, des noms qui n'existent pas ici. Constantes `C54X_IT_*`. |
| 5.2 | Table des IRQ du MCU (IRQ0..IRQ20) | À confronter aux `IRQ_*` de `calypso_soc.c` — pas encore fait |
| 3.5.15 | DMA : 4 canaux, config **depuis le MCU uniquement** | `calypso_rhea_dma.c` |
| **3.7.1** | RIF : XIO mot-à-mot (IT vers le DSP) **ou** API en mode DMA (end-DMA vers l'**ARM**) | A cadré le débat vec16/vec30 — tranché ensuite par la mesure : le DSP fait du **polling**, il ne veut aucune des deux IT |
| 3.7.6 | INTH des périphériques DSP | `calypso_xio.c` — champs **non localisés**, valeur brute |
| 4.3 | Le DSP démarre sur le vecteur reset en `0xFF80` | confirme `C54X_INT_RESET` |
| **6** | Table 2 : allocation des canaux DMA (0=RIF TX, 1=RIF RX, 2/3=UARTs) | Croisée avec §11 (`DMA1..DMA4`) → `DMA1`=RIF TX, `DMA2`=RIF RX. **Confirmé par la mesure** : `DMA2_RAD=0x7002` = l'adresse de `DRR` |
| 7.1 | Carte mémoire ARM ; API RAM `FFD0:0000–FFD0:3FFF` | 16 Ko = 8 kmots ✓ `C54X_API_SIZE` |
| 7.2 | Carte mémoire DSP : DARAM 2K, **API 8K @ 0x0800**, DARAM 18K, PROM 28K @ 0x7000 | confirme `prom0-base-address-0x7000` |
| **7.2.1** | SAM / HOM : en **HOM le DSP n'accède plus à la RAM API** | ⚠️ **non modélisé** — cf. §9.1 ci-dessous |
| 7.2.2 | Carte XIO du DSP : RIF `0000`, API Control `F900`, INTH `FA00`, DMA `FC00` | `calypso_rif.c`, `calypso_xio.c`, `calypso_rhea_dma.c` |

### CAL207 (`ti-calypso2.pdf`)

| § | Sujet | Conséquence dans le code |
|---|---|---|
| **9.1** | **`API_CONF` @ XIO:F900** — bit1 `API_HOM`, bit2 `BRIDGE_CLK_EN` | `calypso_xio.c`. **Mesuré** : le DSP bascule HOM↔SAM **à chaque trame** (`0x0002` en `0xa693`, `0x0000` en `0xa4e7`) — l'arbitrage n'existe pas dans le modèle |
| 9.2.3 | BSCR (MMR data `0x0029`) bits 2/3 = SMODE/HINT de l'APIC | explique `orm *(0x0029),#4` en `0xa68d` |
| 11.1–11.3 | Registres DMA, MCU **et** DSP (`FFFF:FCxx` / `XIO:FCxx`) | `calypso_rhea_dma.c`. Reset `DMAn_CTRL` = **0x04A2** (recoupé champ par champ), `DIRECTION=1` = Rhea→API |
| 11.3.5 note | `BRIDGE_CLK_EN` obligatoire avant tout accès aux registres DMA | le firmware la respecte à la lettre (`0xb3b1`/`0xb3c6`) |
| **12.1** | Registres RIF : `DXR` 0x0000, `DRR` 0x0001, `SPCX` 0x0002, `SPCR` 0x0003 | `calypso_rif.c`. `PORTR` était un **no-op** : 30 lectures sur 30 visaient `SPCR` |
| **12.6** | Champs de `SPCR`, reset **0x3CA2** (recoupé champ par champ) | Le firmware exécute la séquence RRST à deux écritures **à chaque trame**, et laisse `RINT_MASK`+`RDMA_MASK` à 1 → **le RX du DSP est du polling** |
| 16.5 / 16.9 | INTH du MCU (`FFFF:FA08`, niveaux `FA20..FA46`) | pas encore confronté au modèle |

## Contradictions relevées dans les documents eux-mêmes

Signalées ici pour qu'on ne les redécouvre pas trois fois.

1. **Mode par défaut de l'API.** CAL207 §9.1 donne `API_HOM` à **1** au reset (HOM) ;
   CAL000 §7.2.1 affirme *« SAM mode is the default configuration when the DSP exits
   from a reset phase »*. Non tranché. Le firmware écrit explicitement `0x0000` (SAM)
   tôt au boot, comme s'il ne faisait confiance ni à l'un ni à l'autre.
2. **Taille de l'API.** §7.2.1 et §7.2 disent 8 kmots ; le §6 de CAL000 et le §10 de
   CAL207 parlent d'une mémoire partagée « 6K-word ». La carte mémoire (8K entre
   `0x0800` et `0x27FF`) et le firmware (`API_SIZE 0x2000`) donnent 8K.
3. **`SPCR` bit 5 `FIFO_EMPTY`.** La prose dit « 0 = receive FIFO is empty » alors que
   la valeur de reset est 1 et que la FIFO est vide au reset. La prose est inversée ;
   on suit le reset.
4. **`SPCR` bit 2 `RRDY`.** Le §12.6 le note *Unused* ; le §12.3 s'en sert
   (« DRR is updated when RSR is ready to be read and RRDY=1 »). Gate
   `CALYPSO_RIF_RRDY_UNUSED` pour revenir à la lettre du §12.6.
5. **`CONTROLLER_CONFIG` reset.** Table 18 : `11 111?` ; liste des champs du §11.2.1 :
   `DMA_BURST=0x1` + `PRIORITY_ENABLE=1` = `0x24`. On suit la liste des champs.

## Ce que la mesure a tranché contre une lecture du doc

Un seul cas à ce jour, et il mérite d'être connu : le §5.1 place `AINT` en bit 12 et
`INT8n` (IT trame TPU) en bit 11. Mesuré sur ~30 000 relevés, l'IMR du ROM n'ouvre
**jamais** le bit 11 et ouvre **toujours** le bit 12, et le vecteur réellement pris
est le slot 28 (`PC=0x00f0`). Comme osmocom ne signale QUE par l'IT trame du TPU et
n'utilise jamais AINT, l'ordre de la liste en prose du §5.1 est décalé d'un cran dans
cette zone : **IT trame = bit 12 / vec 28**, AINT = bit 11 / vec 27. Le reste de la
table (TINT bit 3, SPI bits 4/5, DMA bit 14) n'est pas affecté.
