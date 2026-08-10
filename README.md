# QEMU-Calypso

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

En famille **shunt**, le mobile campe sur la cellule, effectue sa Location Update
(`LOCATION UPDATING ACCEPT` + `TMSI REALLOC COMPLETE`), fait du SMS MO et MT, et
tient un TCH/F avec audio bidirectionnel.
Mais dans ce mode c'est gr-gsm qui démodule : le firmware ARM tourne pour de vrai,
le DSP non.

En mode **natif**, le corrélateur mask-ROM s'exécute réellement (il atteint
`0x8d00`, `DETECTOR-RUN @0x9ac0`, `d_fb_mode=1`) mais `d_fb_det[0x08f8]` reste à 0 :
pas encore d'acquisition. Le buffer d'échantillons IQ du DSP n'est pas câblé au
récepteur on-chip. C'est **le** chantier ouvert.

**La voix (TCH/F) passe en `shunt_legit`, avec audio.** Le firmware d'origine traite
l'`ASSIGNMENT COMMAND`, reconfigure sa L1, tient le canal et transporte des trames
voix : le test d'écho (Asterisk, 600) renvoie un audio audible, ce qui prouve la
boucle complète — encodage uplink, transport, décodage downlink, rendu. À lire avec
la même réserve que le reste de la famille shunt : la démodulation du downlink est
faite par gr-gsm, donc le résultat porte sur la pile L1/L2/L3 du firmware réel,
**pas sur le DSP**. Non atteint en natif.

Autre limite à connaître : l'ARM est modélisé par un cœur ARM946 (ARMv5TE) là où le
Calypso a un ARM7TDMI (ARMv4T). Choix assumé, voir `hw/arm/calypso/calypso_mb.c:303`.

### Matrice détaillée

| # | Objectif | shunt | natif | preuve |
|---|---|---|---|---|
| 1 | FB/SB (sync) | ✅ | 🔧 WIP | `DISPATCH SB BSIC=7` ; natif : corrélateur tourne, `d_fb_det=0` |
| 2 | RXLEV serving | ✅ | ✅ | `RLA_C -53 dBm`, C1/C2 > 0 |
| 3 | Camp (C3) | ✅ | ⬜ | `normal service` |
| 4 | Location Update | ✅ | ⬜ | `LOCATION UPDATING ACCEPT`, `On Network` |
| 5 | SMS MO | ✅ | ⬜ | `SMS successful` (flaky en `DSP,NO_CANNED`) |
| 6 | SMS MT | ✅ | ⬜ | `SMS from 777` / `SMS from 10002` |
| 7 | Service tenu post-SMS | ✅ | ⬜ | `MM connection active → MM IDLE` |
| 8 | Ctrl-C / re-camp | ✅ | ⬜ | reset L1 câblé (`d_dsp_page=0`) |
| 9 | Union SDCCH SS0-7 (/4 + /8) | ✅ | — | LU quelle que soit la sous-voie |
| 10 | Reset L1 câblé | ✅ | — | SI reprend + Ctrl-C recover |
| 11 | FB-STREAM, entrée `0x9213/0x9215` | ✅ | ✅ | rampe relue par le démod |
| 12 | **Voix (TCH/F)** | ✅ | ⬜ | audio en sortie sur le test d'écho (Asterisk 600) — boucle UL+DL complète ; démod DL par gr-gsm |
| 13 | SMS MT occasionnel perdu | ⬜ | ⬜ | 1 `MMSMS_REL_IND` prématuré résiduel |
| 14 | `d_fb_det` natif | — | ⬜ | corrélateur tourne, fenêtre + dispatch par-frame à finir |

Source de vérité par mode :
**[`ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md)**. En cas de conflit entre
docs, c'est lui qui prime.

---

## Démarrer

```bash
git clone https://github.com/bbaranoff/osmo_egprs
cd osmo_egprs
./build.sh
```

Puis `osmo-nitb-for-calypso/start-direct.sh`. Build seul :
`cd build && ninja qemu-system-arm`.

Détail des modes, commandes et vérifications :
**[QUICK_START.md](QUICK_START.md)**.

## Vérifier soi-même

Ce projet fait deux affirmations vérifiables en quelques minutes. Les deux sont
faites pour être cassées :

**1. Le firmware est le vrai, non modifié.** Prenez un `layer1.highram.elf`
construit depuis n'importe quel commit d'osmocom-bb, remplacez celui fourni,
relancez. S'il ne boote pas, l'affirmation est fausse — ouvrez une issue.

**2. Les ✅ shunt ne prouvent rien sur le DSP.** C'est écrit partout ici, mais
vérifiez-le : passez en `CALYPSO_MODE=native`, le mobile ne campe pas.

## Résultats chiffrés

- **[`run_results.md`](run_results.md)** — mesures reproductibles (profondeur ring,
  débits, temps LU, éviction), chaque chiffre confronté à une règle de décision
  posée d'avance.
- **[`RAPPORT_DFBDET.md`](RAPPORT_DFBDET.md)** — enquête sur la cause racine de
  `d_fb_det = 0` en natif : chaîne causale citée ligne à ligne, hypothèses écartées
  **avec leur preuve d'écartement**, test décisif à règle posée d'avance. Diagnostic
  en lecture seule — rien n'a été corrigé sur la foi de ce rapport.

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

**DSP natif** : vrai corrélateur mask-ROM ; son buffer IQ n'est pas câblé au
récepteur on-chip sous QEMU. Point d'injection prouvé inscriptible :
`data[0x9213]/[0x9215]` (cf. `CALYPSO_FB_STREAM`).

### Composants (`hw/arm/calypso/`)

| Fichier | Rôle |
|---|---|
| `calypso_c54x.c` | Cœur DSP TMS320C54x (décodeur ISA, MMIO, IT) |
| `calypso_dsp_shunt.c` | Chaîne host FBSB : real_fb_read, feed_iq, feed_si |
| `calypso_bsp.c` | BSP : livraison bursts DARAM, tee IQ, BRINT0 |
| `calypso_trx.c` | Horloge/FN, miroir MMIO ARM↔DSP |
| `calypso_arm2dsp.c` | Pont ARM→DSP (go-live, `d_ctrl_system`) |
| `calypso_tpu.c` / `calypso_tsp.c` | Séquenceur TPU + protocole TSP (frontend RF) |

---

## Modes

| profil | FB/SB | SI | ce qu'il prouve |
|---|---|---|---|
| `shunt_legit` | hôte | gr-gsm | la pile de bout en bout : camp, LU, SMS, TCH/F + audio out |
| `native_twl` | hôte / TWL | **DSP** | le DSP traite-t-il le SI ? On lui donne la synchro pour poser la question sans attendre le FB/SB natif |
| `native` | DSP | DSP | la vérité sur l'acquisition. Ne campe pas aujourd'hui |
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

Tout se pilote par `CALYPSO_*` (voir `calypso.env`), **toutes overridables en CLI**.
Un profil ne pose que des `:=` : la CLI garde le dernier mot, et **la seule source de
vérité sur ce qu'un run a réellement obtenu est le manifeste imprimé par le
modèle**, jamais la ligne de commande.

---

## Documentation

Toute la doc vit sous `hw/arm/calypso/doc/`.

| Doc | Contenu |
|---|---|
| **[ETAT_ACTUEL.md](hw/arm/calypso/doc/ETAT_ACTUEL.md)** | ⭐ Source de vérité : ce qui marche par mode, l'architecture réelle, les fausses pistes |
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
