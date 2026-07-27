# QEMU-Calypso

Fork de QEMU émulant le **baseband GSM TI Calypso** d'un téléphone Osmocom
(Compal E88 / Motorola C1xx). Deux processeurs émulés tournent ensemble :

- **ARM946** — exécute le firmware [osmocom-bb](https://osmocom.org/projects/baseband) (Layer 1 temps réel).
- **TMS320C54x** — exécute la **vraie mask-ROM DSP** de TI (FB/SB/démod).

Reliés par une **API RAM** MMIO (mailbox ARM↔DSP à `0xFFD00000`). En face, un cœur
réseau Osmocom (BTS + fake_trx/osmo-trx + gr-gsm) fournit un vrai downlink GSM.

**Résultat** : le mobile émulé **campe sur la cellule et effectue sa Location Update**
(`LOCATION UPDATING ACCEPT`), pilote MT/MO SMS, via une chaîne host qui décode le
downlink réel et alimente l'API RAM du DSP au format natif.

---

## Démarrage

👉 **[QUICK_START.md](QUICK_START.md)** — build, lancer, modes, vérifications.

## Documentation

Toute la doc vit sous `hw/arm/calypso/doc/`.

| Doc | Contenu |
|---|---|
| **[ETAT_ACTUEL.md](hw/arm/calypso/doc/ETAT_ACTUEL.md)** | ⭐ **Source de vérité unique** : ce qui marche, l'architecture réelle, l'état du DSP natif, les fausses pistes. |
| [QUICK_START.md](QUICK_START.md) | Build, lancer, modes, vérifications. |
| [SHUNT_LEGIT_ADDRESS_MAP.md](hw/arm/calypso/doc/SHUNT_LEGIT_ADDRESS_MAP.md) | Mapping canonique des cellules DSP (data/api_ram), chaîne FB/SB/rxlev/a_cd. |
| [CALYPSO_HW.md](hw/arm/calypso/doc/CALYPSO_HW.md) · [hardware-map.md](hw/arm/calypso/doc/hardware-map.md) | Carte matérielle du SoC Calypso. |
| [C54X_INSTRUCTIONS.md](hw/arm/calypso/doc/C54X_INSTRUCTIONS.md) · [opcodes/](hw/arm/calypso/doc/opcodes/) | Jeu d'instructions TMS320C54x (décodeur). |
| [DSP_ROM_MAP.md](hw/arm/calypso/doc/DSP_ROM_MAP.md) | Carte de la mask-ROM DSP. |
| [SERCOMM_GATE_ARCHITECTURE.md](hw/arm/calypso/doc/SERCOMM_GATE_ARCHITECTURE.md) | Canal Sercomm/L1CTL. |
| [datasheets/](hw/arm/calypso/doc/datasheets/) | PDF constructeur (TI SPRU, FreeCalypso). |
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

- **Chaîne host (opérationnelle)** : gr-gsm décode SB/SI, les modèles RF (twl3025/DECAN,
  gain trf6151) produisent AFC/PM/rxlev, le tout injecté dans l'API RAM → l'ARM campe.
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
