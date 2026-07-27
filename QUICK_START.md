# QUICK START — QEMU-Calypso

Émulation QEMU du baseband GSM TI Calypso (ARM946 + DSP TMS320C54x mask-ROM)
faisant tourner le firmware osmocom-bb, face à un cœur réseau Osmocom complet.

> **Vérité courante** : [`hw/arm/calypso/doc/ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md).

---

## 1. Installation

Tout tourne dans un **conteneur** (`osmo-operator-1`), image bâtie depuis
`/opt/GSM/osmo_egprs/Dockerfile`. Il embarque :

| Composant | Rôle |
|---|---|
| Cœur Osmocom (STP/HLR/MSC/MGW/BSC + `osmo-bts`) | Réseau GSM |
| `fake_trx` / `osmo-trx` | Radio (downlink GSM réel I/Q) |
| `gr-gsm` | Décodage SB/SI du downlink |
| **`qemu-src`** (ce dépôt) | Le baseband Calypso émulé |
| `osmocom-bb-transceiver` | Firmware L1 (`layer1.highram.elf`) + blobs DSP-ROM |

### Reconstruire (depuis le conteneur)
```bash
cd /opt/GSM/qemu-src/build && ninja qemu-system-arm
```
Le binaire : `/opt/GSM/qemu-src/build/qemu-system-arm`. Firmware + ROM DSP
(`calypso_dsp.PROM0..3/DROM/PDROM`) dans `/opt/GSM`.

### Reconstruire l'image complète (hôte)
```bash
cd /opt/GSM/osmo_egprs && ./build.sh          # ou docker build -f Dockerfile
```

---

## 2. Lancer

Le launcher `osmo_egprs/start-direct.sh` monte cœur + radio + Calypso.

### Modes de plateforme (`MODE=`)
| Mode | Ce qu'il monte |
|---|---|
| `qemu` *(défaut)* | Cœur (no-process) + `qemu-src/start-clean.sh` (Calypso QEMU) |
| `faketrx` | Cœur + osmo-bts + fake_trx + trxcon + mobile (soft) |
| `virtphy` | Cœur + osmo-bts-virtual + virtphy + mobile |
| `faketrx-qemu` | Cœur + fake_trx vivant, Calypso QEMU attaché |

```bash
cd /opt/GSM/osmo_egprs
./start-direct.sh                 # mode qemu (défaut)
MODE=faketrx ./start-direct.sh
```

### Modes Calypso (socle `calypso.env` + `calypso_X.env`)
`start-clean.sh` source `calypso.env` (socle commun) qui, selon un flag, source
le fichier du mode. **Toutes les vars sont overridables en CLI** (idiome `:=`).

| Mode | Flag | Fichier | Chaîne FBSB |
|---|---|---|---|
| **Base (shunt_legit)** | *(défaut)* | `calypso.env` | Host : real_fb + gr-gsm → API RAM |
| **No-legit** | `CALYPSO_SHUNT_NO_LEGIT=1` | `calypso_shunt_no_legit.env` | Injections explicites, DSP shunté |
| **Natif pur** | `CALYPSO_NATIVE=1` | `calypso_native.env` | DSP : FB-STREAM → `data[0x9213/0x9215]` |
| **Natif aidé** | `CALYPSO_NATIVE_HELPED=1` | `calypso_native_helped.env` | DSP + `feed_iq` DARAM (`0x9210`) |

```bash
./start-direct.sh                              # base shunt_legit
CALYPSO_SHUNT_NO_LEGIT=1 ./start-direct.sh
CALYPSO_NATIVE=1 ./start-direct.sh
CALYPSO_NATIVE_HELPED=1 ./start-direct.sh
```

**Sous-modes (value-list)** — `CALYPSO_SHUNT_LEGIT` et `CALYPSO_SHUNT_NO_LEGIT`
acceptent une liste de tokens (virgule/espace, casse libre) :

| Valeur | Effet |
|---|---|
| `=1` | mode nu |
| `=DSP` | lance **aussi** le DSP c54x en // (`CALYPSO_DSP_RUN_C54X=1`) |
| `=NO_CANNED` | mode sans cannes (`CALYPSO_SHUNT_NO_CANNED=1`) |
| `=DSP,NO_CANNED` | les deux |

```bash
CALYPSO_SHUNT_LEGIT=DSP,NO_CANNED ./start-direct.sh
CALYPSO_SHUNT_NO_LEGIT=NO_CANNED ./start-direct.sh
```

Logs : `/root/qemu.log` (QEMU/DSP), `/root/mobile.log` (mobile Calypso).

---

## 3. État — TODO / DONE

> ⚠️ **Le statut DÉPEND DU MODE.** Le tableau ci-dessous vaut pour la **famille shunt** ; en **natif** (`CALYPSO_NATIVE=1` / `NATIVE_HELPED`) FB/SB, Camp, LU, SMS et Ctrl-C sont WIP/TODO. Vérité-terrain par mode = [matrice statut × mode d'`ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md).

| # | Objectif | Statut | Détail / preuve |
|---|---|---|---|
| 1 | FB/SB (sync) | ✅ DONE · natif WIP | `DISPATCH SB BSIC=7` (gr-gsm réel) |
| 2 | RXLEV serving | ✅ DONE | `RLA_C -53 dBm`, C1/C2 > 0 |
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
| 13 | **Voix (TCH/F)** | 🔧 WIP | squelette TCH DL présent (`shunt_dispatch_tch_dl`) ; manque UL FACCH + producteur DL |
| 14 | SMS MT occasionnel à la trappe | ⬜ TODO | 1 transaction `MMSMS_REL_IND` prématurée résiduelle |
| 15 | `d_fb_det` natif (corrélateur DSP) | ⬜ TODO | corrélateur = vrai ; entrée inscriptible ; reste completion FBSB + dispatch par-frame |

---

## 4. Pièges

- **NE PAS feed `0x2a00`** (workzone de SORTIE du démod, régénéré par le DSP) — feed la source `0x9213/0x9215`.
- `CALYPSO_ARM2DSP_CTRLSYS=1` **casse** le go-live natif (re-force B_TASK_ABORT) — laisser à `0`.
- Caps de log (`KEEP-IMR <8`, `SHADOW-DADST <40||%2000`) : un `grep` qui « s'arrête » ≠ perte réelle.
- `calypso_fbsb.c` : compteurs désormais réels (avant `0 0 0 0` = red herring).
