# QEMU-Calypso

Fork de QEMU émulant le **baseband GSM TI Calypso** d'un téléphone Osmocom
(Compal E88 / Motorola C1xx). Deux processeurs émulés tournent ensemble :

- **ARM946** — cœur QEMU (ARMv5TE) qui **modélise le vrai ARM7TDMI / ARMv4T** du
  Calypso (choix assumé, cf. `hw/arm/calypso/calypso_mb.c:303`) ; exécute le firmware
  [osmocom-bb](https://osmocom.org/projects/baseband) (Layer 1 temps réel).
- **TMS320C54x** — exécute la **vraie mask-ROM DSP** de TI (FB/SB/démod).

Reliés par une **API RAM** MMIO (mailbox ARM↔DSP à `0xFFD00000`). En face, un cœur
réseau Osmocom (BTS + fake_trx/osmo-trx + gr-gsm) fournit un vrai downlink GSM.

**Résultat (modes shunt)** : le mobile émulé **campe sur la cellule et effectue sa
Location Update** (`LOCATION UPDATING ACCEPT`), pilote MT/MO SMS, via une chaîne host
qui décode le downlink réel et alimente l'API RAM du DSP au format natif. Le mode
**natif** (corrélateur DSP seul) reste en cours — voir la matrice statut × mode.

---

## Démarrage

👉 **[QUICK_START.md](QUICK_START.md)** — build, lancer, modes, vérifications.

## 📊 Résultats chiffrés

**[`run_results.md`](run_results.md)** — mesures reproductibles (profondeur ring,
débits, temps LU, éviction), chaque chiffre confronté à une règle de décision.
C'est là que se trouve le dur, pas dans les affirmations.

**[`RAPPORT_DFBDET.md`](RAPPORT_DFBDET.md)** — enquête multi-agents sur la cause
racine de `d_fb_det = 0` en mode natif : chaîne causale citée ligne à ligne,
hypothèses écartées **avec leur preuve d'écartement**, test décisif à règle posée
d'avance, et recoupement avec le firmware osmocom-bb réel. Diagnostic en lecture
seule — rien n'a été corrigé sur la foi de ce rapport.

## État — TODO / DONE

> ⚠️ **Le statut DÉPEND DU MODE.** Les ✅ ci-dessous valent pour la **famille shunt**
> (`SHUNT_LEGIT=1`, `SHUNT_NO_LEGIT=1`, `SHUNT_LEGIT=DSP,NO_CANNED`) ; en mode
> **natif** (`CALYPSO_NATIVE=1` / `NATIVE_HELPED`) plusieurs lignes sont WIP/TODO
> (FB/SB, Camp, LU, SMS, Ctrl-C). **La vérité-terrain par mode est la
> [matrice statut × mode d'`ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md)** —
> non dupliquée ici.

| # | Objectif | Statut (famille shunt) | Détail / preuve |
|---|---|---|---|
| 1 | FB/SB (sync) | ✅ DONE · natif WIP | `DISPATCH SB BSIC=7` (gr-gsm réel) ; natif : corrélateur tourne mais `d_fb_det=0` |
| 2 | RXLEV serving | ✅ DONE (tous modes) | `RLA_C -53 dBm`, C1/C2 > 0 |
| 3 | Camp (C3) | ✅ DONE · natif TODO | `normal service` |
| 4 | Location Update + registration | ✅ DONE · natif TODO | `LOCATION UPDATING ACCEPT` + `TMSI REALLOC COMPLETE`, `On Network` |
| 5 | SMS MO | ✅ DONE · `DSP,NO_CANNED` flaky · natif TODO | passe du 1er coup, `SMS successful` |
| 6 | SMS MT | ✅ DONE · `DSP,NO_CANNED` WIP · natif TODO | `SMS from 777` / `SMS from 10002` reçus (bidirectionnel) |
| 7 | Service tenu post-SMS | ✅ DONE | `MM connection active → MM IDLE, normal service` |
| 8 | Ctrl-C mobile / re-camp | ✅ DONE · natif TODO | reset L1 câblé (`d_dsp_page=0` API-RAM) |
| 9 | Union SDCCH SS0-7 (/4 + /8) | ✅ DONE | UA présentée sur toute la région SDCCH → LU quelle que soit la sous-voie |
| 10 | Reset L1 câblé (`shunt_latch_task`) | ✅ DONE | SI reprend + Ctrl-C recover (`CALYPSO_L1_RESET_WIRE=0` désactive) |
| 11 | Value-list env (`DSP` / `NO_CANNED`) | ✅ DONE | `CALYPSO_SHUNT_LEGIT=DSP,NO_CANNED` |
| 12 | FB-STREAM + entrée native `0x9213/0x9215` | ✅ DONE | rampe relue par le démod (prouvé) |
| 13 | **Voix (TCH/F)** | 🔧 WIP (tous modes) | `ASSIGNMENT COMMAND` atteint → `ASSIGNMENT FAILURE` (shunt ne présente pas le TCH DL) ; call `fake_trx` = ACTIVE + audio ⇒ réseau OK |
| 14 | SMS MT occasionnel à la trappe | ⬜ TODO | 1 transaction `MMSMS_REL_IND` prématurée résiduelle |
| 15 | `d_fb_det` natif (corrélateur DSP) | ⬜ TODO (natif) | corrélateur **tourne** (atteint `0x8d00`, DETECTOR-RUN `@0x9ac0`, `d_fb_mode=1`) mais `d_fb_det[0x08f8]=0` : reste fenêtre + dispatch par-frame |

> Détails : [`QUICK_START.md`](QUICK_START.md), [`hw/arm/calypso/doc/ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md) (matrice par mode) et [`hw/arm/calypso/doc/TODO.md`](hw/arm/calypso/doc/TODO.md).

## Documentation

Toute la doc vit sous `hw/arm/calypso/doc/`.

| Doc | Contenu |
|---|---|
| **[ETAT_ACTUEL.md](hw/arm/calypso/doc/ETAT_ACTUEL.md)** | ⭐ **Source de vérité unique** : ce qui marche (par mode), l'architecture réelle, l'état du DSP natif, les fausses pistes. |
| [QUICK_START.md](QUICK_START.md) | Build, lancer, modes, vérifications. |
| [TODO.md](hw/arm/calypso/doc/TODO.md) | TODO consolidé (P0/P1/P2) — cadré par mode. |
| [README.md (index doc)](hw/arm/calypso/doc/README.md) | Index de la doc courante sous `hw/arm/calypso/doc/`. |
| [SHUNT_LEGIT_ADDRESS_MAP.md](hw/arm/calypso/doc/SHUNT_LEGIT_ADDRESS_MAP.md) | Mapping canonique des cellules DSP (data/api_ram), chaîne FB/SB/rxlev/a_cd. |
| [DSP_ADDRESS_MAP.md](hw/arm/calypso/doc/DSP_ADDRESS_MAP.md) · [DSP_ARM_LINKAGE.md](hw/arm/calypso/doc/DSP_ARM_LINKAGE.md) | Cartes d'adresses DSP + correspondance ARM↔DSP (référence durable). |
| [VOIX_PLAN.md](hw/arm/calypso/doc/VOIX_PLAN.md) | Plan d'implémentation voix (TCH/F) host-side — pour le TODO P1 « Voix ». |
| [CALYPSO_HW.md](hw/arm/calypso/doc/CALYPSO_HW.md) · [hardware-map.md](hw/arm/calypso/doc/hardware-map.md) | Carte matérielle du SoC Calypso. |
| [C54X_INSTRUCTIONS.md](hw/arm/calypso/doc/C54X_INSTRUCTIONS.md) · [opcodes/](hw/arm/calypso/doc/opcodes/) | Jeu d'instructions TMS320C54x (décodeur). |
| [DSP_ROM_MAP.md](hw/arm/calypso/doc/DSP_ROM_MAP.md) | Carte de la mask-ROM DSP. |
| [SERCOMM_GATE_ARCHITECTURE.md](hw/arm/calypso/doc/SERCOMM_GATE_ARCHITECTURE.md) | Canal Sercomm/L1CTL. |
| [datasheets/](hw/arm/calypso/doc/datasheets/README.md) | PDF constructeur (TI SPRU, FreeCalypso). |
| [archive/](hw/arm/calypso/doc/archive/README.md) | Doc historique (sessions datées, rapports d'étape, pistes closes). |

> En cas de conflit entre docs, **`ETAT_ACTUEL.md` prime**.

---

## Architecture (résumé)

```
   osmo-bts / fake_trx / osmo-trx  ──►  downlink GSM réel (I/Q 4 SPS)
                                          │
                    ┌─────────────────────┴───────────────────┐
                    │  QEMU-Calypso (machine `calypso`)        │
                    │                                          │
                    │   ARM946 ◄──API RAM (0xFFD00000)──► DSP  │
                    │   (osmocom-bb)                    (C54x) │
                    │        ▲                                 │
                    │        │ real_fb_read (intercept MMIO)   │
                    │   gr-gsm + twl3025/DECAN + trf6151       │
                    └──────────────────────────────────────────┘
                                          │
                                    camp + Location Update
```

- **Chaîne host (opérationnelle, modes shunt)** : gr-gsm décode SB/SI, les modèles RF
  (twl3025/DECAN, gain trf6151) produisent AFC/PM/rxlev, le tout injecté dans l'API RAM
  → l'ARM campe.
- **DSP natif** : vrai corrélateur mask-ROM ; son buffer d'échantillons IQ n'est pas
  câblé au récepteur on-chip en QEMU. Point d'injection prouvé inscriptible =
  `data[0x9213]/[0x9215]` (voir `CALYPSO_FB_STREAM`).

## Composants clés (`hw/arm/calypso/`)

| Fichier | Rôle |
|---|---|
| `calypso_c54x.c` | Cœur DSP TMS320C54x (décodeur ISA, MMIO, IT) |
| `calypso_dsp_shunt.c` | Chaîne host FBSB : real_fb_read, feed_iq, feed_si, injections |
| `calypso_bsp.c` | BSP : livraison bursts DARAM, tee IQ, BRINT0 |
| `calypso_trx.c` | Horloge/FN, mirroir MMIO ARM↔DSP |
| `calypso_arm2dsp.c` | Pont ARM→DSP (go-live, d_ctrl_system) |
| `calypso_tpu.c` / `calypso_tsp.c` | Séquenceur TPU + protocole TSP (RF frontend) |

## Configuration

Tout se pilote par variables d'environnement `CALYPSO_*` (voir `calypso.env`, **toutes
overridables en CLI**). Lancement via `osmo_egprs/start-direct.sh` → `start-clean.sh`
(source `calypso.env`) → `run.sh`.

## Build

```bash
cd build && ninja qemu-system-arm
```

---

*Basé sur QEMU. La documentation QEMU amont d'origine est conservée sous `docs/`.*
