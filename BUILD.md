# Calypso QEMU — Build & Deploy Guide

## 1. Cloner QEMU

```bash
git clone https://gitlab.com/qemu-project/qemu.git ~/qemu
cd ~/qemu
git checkout v9.2.0          # version testée
```

---

## 2. Copier les fichiers depuis le zip

```bash
# Extraire le zip
unzip calypso_updated.zip -d /tmp/calypso_src

SRC=/tmp/calypso_src/calypso
QEMU=~/qemu

# ── hw/arm/calypso/ ──────────────────────────────────────────────────────────
mkdir -p $QEMU/hw/arm/calypso
cp $SRC/hw/arm/calypso/calypso_mb.c              $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/calypso_soc.c             $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/calypso_trx.c             $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/meson.build               $QEMU/hw/arm/calypso/
# calypso_socket_improved.c : optionnel, non compilé par défaut
# cp $SRC/hw/arm/calypso/calypso_socket_improved.c $QEMU/hw/arm/calypso/

# ── hw/char/ ─────────────────────────────────────────────────────────────────
cp $SRC/hw/char/calypso_uart.c                   $QEMU/hw/char/

# ── hw/i2c/ ─────────────────────────────────────────────────────────────────
cp $SRC/hw/i2c/calypso_i2c.c                     $QEMU/hw/i2c/
# (hw/ssi/calypso_i2c.c était un doublon — NE PAS copier)

# ── hw/intc/ ─────────────────────────────────────────────────────────────────
cp $SRC/hw/intc/calypso_inth.c                   $QEMU/hw/intc/

# ── hw/ssi/ ──────────────────────────────────────────────────────────────────
cp $SRC/hw/ssi/calypso_spi.c                     $QEMU/hw/ssi/

# ── hw/timer/ ────────────────────────────────────────────────────────────────
cp $SRC/hw/timer/calypso_timer.c                 $QEMU/hw/timer/

# ── include/ ─────────────────────────────────────────────────────────────────
mkdir -p $QEMU/include/hw/arm/calypso
cp $SRC/include/hw/arm/calypso/calypso_inth.h    $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_soc.h     $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_spi.h     $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_timer.h   $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_trx.h     $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_uart.h    $QEMU/include/hw/arm/calypso/
```

---

## 3. Patcher les meson.build existants

> Ces fichiers **existent déjà** dans QEMU. Ajouter les lignes suivantes
> dans les blocs existants (ne pas remplacer le fichier).

### hw/arm/meson.build
```meson
subdir('calypso')
```

### hw/char/meson.build  — ajouter dans le bloc system_ss :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_uart.c'))
```

### hw/i2c/meson.build  — ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_i2c.c'))
```

### hw/intc/meson.build — ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_inth.c'))
```

### hw/ssi/meson.build  — ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_spi.c'))
```

### hw/timer/meson.build — ajouter :
```meson
system_ss.add(when: 'CONFIG_CALYPSO', if_true: files('calypso_timer.c'))
```

---

## 4. Kconfig

### hw/arm/Kconfig — ajouter :
```kconfig
config CALYPSO
    bool "TI Calypso SoC (ARM946E-S GSM baseband)"
    default y
    depends on ARM
    select PFLASH_CFI01
```

### default-configs/devices/arm-softmmu.mak — ajouter :
```makefile
CONFIG_CALYPSO=y
```

---

## 5. Compiler QEMU

```bash
cd ~/qemu
mkdir -p build && cd build

../configure \
    --target-list=arm-softmmu \
    --enable-debug \
    --disable-werror

ninja -j$(nproc)
```

### Vérifier :
```bash
./qemu-system-arm -M help | grep calypso
# calypso    Calypso SoC development board (modular architecture)
```

---

## 6. Lancer le stack complet

### Terminal 1 — osmo_egprs (si pas déjà lancé)
```bash
cd ~/osmo_egprs && ./start.sh
```

### Terminal 2 — QEMU Calypso
```bash
cd ~/qemu/build
./qemu-system-arm -M calypso \
    -cpu arm946 \
    -kernel ~/compal_e88/loader.highram.elf \
    -serial pty \
    -monitor stdio \
    -nographic
# Note le PTY : /dev/pts/X
```

### Terminal 3 — osmocon (upload firmware)
```bash
osmocon -p /dev/pts/X -m c123xor ~/compal_e88/layer1.highram.bin
# → expose /tmp/osmocom_l2
```

### Terminal 4 — mobile (L2/L3)
```bash
mobile -c mobile.cfg
```

### Terminal 5 — Monitor HTML
```bash
pip3 install websockets
python3 ws_bridge.py --trxc-port 6700 --gsmtap-port 4729 --ws-port 9999
# Ouvrir index.html dans le navigateur
```

### Port mapping TRXC/TRXD
```
QEMU calypso_trx écoute :
  UDP 6700 → TRXC (commandes ASCII de osmo-bts-trx)
  UDP 6701 → TRXD (bursts binaires)

osmo-bts-trx doit pointer sur :
  -i 127.0.0.1 -p 6700
```

---

## 7. Script tout-en-un

```bash
bash launch_calypso.sh ~/compal_e88/layer1.highram.bin
```

---

## Notes

- `calypso_trx.c` : port TRXC par défaut = **6700** (pas 4729 qui est GSMTAP).
  Si ton osmo_egprs utilise un autre port, modifier `trx-port` dans
  `calypso_mb.c` : `qdev_prop_set_int32(..., "trx-port", 6700)`.

- `calypso_i2c.c` : présent **uniquement** dans `hw/i2c/`.
  Le doublon dans `hw/ssi/` a été supprimé.

- GSMTAP port 4729 : les bursts TX (uplink Calypso) sont envoyés là.
  `tshark -i lo -f "udp port 4729"` suffit pour sniffer.
