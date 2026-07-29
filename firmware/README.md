# `firmware/` — le firmware ARM embarqué

Le firmware layer1 d'osmocom-bb, **livré avec le dépôt** pour qu'un clone suffise à
démarrer. Sans lui, l'émulateur n'a rien à exécuter.

## Ce qu'il y a ici

| Fichier | | |
|---|---|---|
| `compal_e88/layer1.highram.elf` | 446 ko | ce que QEMU charge (`-kernel`) |
| `compal_e88/layer1.highram.bin` | 66 ko | l'image brute, pour osmocon |
| `compal_e88/layer1.highram.map` | 238 ko | table des symboles — nécessaire au débogage (`16-fwsyms`, `90-gdb`) |
| `compal_e88/layer1.highram.size` | | tailles des sections |

Empreintes de la version livrée :

```
sha256  1912688f5c02bc2f2e9e2a2441c2d4900d0fde9245f3cf214f2a4feefcd5a486  layer1.highram.bin
sha256  6fbe34ca4a95e32e15073b1665d65db553e13b0c66ec32cfdea5f48780666598  layer1.highram.elf
```

Vérifier la vôtre : `sha256sum firmware/compal_e88/layer1.highram.*`

## Provenance et licence

Compilé depuis **osmocom-bb** (`src/target/firmware`), cible `compal_e88` — le
Motorola C118/C123, la carte de référence de ce projet. C'est du logiciel libre
sous **GPLv2 ou ultérieure**, redistribuable à cette condition : le code source
correspondant est disponible sur https://gitea.osmocom.org/phone-side/osmocom-bb

Ce dépôt n'en est pas l'auteur ; il l'embarque pour la commodité.

## Le recompiler

Si vous modifiez la couche 1, ou pour une autre carte :

```bash
git clone https://gitea.osmocom.org/phone-side/osmocom-bb
cd osmocom-bb/src
make HOST_CC=gcc                    # nécessite un arm-none-eabi-gcc
cp target/firmware/board/compal_e88/layer1.highram.* \
   <ce-dépôt>/firmware/compal_e88/
```

Les autres cartes prises en charge par osmocom-bb (`compal_e86`, `pirelli_dpl10`,
`se_j100`, `mt62xx`…) ne sont **pas** livrées : seule `compal_e88` est émulée ici.

## Comment il est trouvé

`environnement/paths.env` regarde d'abord dans ce dépôt, puis retombe sur une
installation externe. Pour en imposer un autre :

```bash
FIRMWARE_ELF=/chemin/vers/layer1.highram.elf ./run.sh
```
