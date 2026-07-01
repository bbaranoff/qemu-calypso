# qemu-calypso — overlay du fork QEMU Calypso

`qemu-calypso` est **l'overlay** qui contient *tout ce qui diffère* de QEMU
upstream pour l'émulation du baseband **TI Calypso** (ARM7 + DSP TMS320C54x).
Appliqué sur un QEMU genuine, il produit le fork :

```
QEMU (genuine 9.2.x)  +  qemu-calypso (cet overlay)  =  qemu  (le fork)
```

## Fabriquer le fork

```sh
./make-fork.sh [GENUINE_DIR] [OUT_DIR]
# défauts : GENUINE_DIR=/opt/GSM/QEMU (v9.2.4)  OUT_DIR=/opt/GSM/qemu-fork
```

`make-fork.sh` copie le genuine puis applique cet overlay par-dessus (en
préservant la structure). Build ensuite :

```sh
cd <OUT_DIR> && mkdir -p build && cd build \
  && ../configure --target-list=arm-softmmu && ninja qemu-system-arm
```

## Contenu (le delta vs genuine QEMU)

**Arbre QEMU modifié / ajouté** — ce qui rend `qemu-system-arm` conscient du Calypso :

| Chemin | Rôle |
|---|---|
| `hw/arm/calypso/` | Le sous-système : DSP c54x, TPU/TDMA, BSP, API-RAM, machine. |
| `include/hw/arm/calypso/` | Headers publics du sous-système. |
| `hw/arm/{meson.build,Kconfig}` | Enregistrement de la machine `calypso` (`subdir('calypso')`, `config CALYPSO`). |
| `hw/{char,intc,ssi,timer}/meson.build` | Enregistrement des périphériques Calypso. |
| `configs/devices/arm-softmmu/default.mak` | Active la machine dans le build arm-softmmu. |
| `tests/`, `tools/`, `scripts/`, `subprojects/` | Ajouts intégrés à l'arbre QEMU. |

**Glue projet (runtime / dev)** — hors source QEMU, sert à lancer et déboguer :

| Chemin | Rôle |
|---|---|
| `calypso.env` | Variables d'environnement du pipeline (sourcé par `start-clean.sh`). |
| `bash_scripts/` | Lancement : `run.sh`, `start-clean.sh`, runners GSM. |
| `python_scripts/`, `gdb_scripts/`, `opt-gsm-scripts/`, `diag/` | Outils de décodage / GDB / sniff / diag. |
| `dsp_blobs/`, `calypso_dsp.txt` | ROM DSP (dump réel) + blobs chargés par la machine. |
| `cfgs/`, `patches/` | Configs et patch externe (osmo-bts skew). |
| `doc/project/` | Docs projet (flux, threading, statut). |

## Lancer le pipeline

```sh
./bash_scripts/start-clean.sh        # source calypso.env puis exec run.sh
```

Chemin DSP réel (défauts confirmés dans `run.sh`) : `CALYPSO_DSP=c54x`,
`CALYPSO_DSP_REG_MODE=c54x`, `CALYPSO_DSP_SHUNT=0`. Log runtime : `/root/qemu.log`.

## Documentation

- **Sous-système Calypso** (le cœur technique) : [`doc/doc_master.md`](doc/doc_master.md)
  — index maître (README, schematics, corrélateur, décodeur, rapports, sessions archivées).
- **Projet** (flux, threading, statut) : [`doc/project/`](doc/project/) (`CLAUDE.md`, `ARCHITECTURE.md`, `*_FLOW.md`, …).

## État courant (résumé)

Objectif : réveiller le vrai DSP c54x pour qu'il écrive `d_fb_det != 0` (détection
FB/FCCH), **sans hack** (règle #1 : on répare le câblage de l'émulateur, jamais
l'état interne du DSP). Blocker terminal : les interruptions DSP restent masquées
(`IMR=0x0000` tout le run, jamais ré-armé) → le handshake go-live ARM→DSP n'aboutit
pas. Détail + leviers : `doc/doc_master.md`.
