# qemu-calypso

Emulation du baseband TI Calypso pour QEMU. Fait tourner le firmware [OsmocomBB](https://osmocom.org/projects/baseband/wiki) Layer 1 et permet de connecter `mobile` (osmocom-bb layer23) via L1CTL.

## Pourquoi

Le Calypso est le baseband des Motorola C1xx / Compal E88. OsmocomBB fournit un firmware open-source (layer1) qui tourne dessus. Ce projet emule le hardware suffisamment pour booter le firmware sans telephone physique.

## Ce qui tourne

```
mobile (layer23)  <-- L1CTL -->  l1ctl_bridge.py  <-- sercomm/PTY -->  QEMU Calypso
                                                                          |
                                                          ARM946 @ layer1.highram.elf
                                                          UART, INTH, SPI/ABB, DSP, TPU
                                                          TRX bridge (UDP 4729/4730)
```

- Boot complet de `layer1.highram.elf` (init hardware, DSP, SIM ATR)
- UART TX/RX fonctionnel avec interrupts
- `mobile` se connecte et envoie L1CTL_RESET_REQ, le firmware le recoit
- TRX bridge compatible osmo-bts-trx (TRXC/TRXD sur UDP)

## Build

Appliquer les fichiers sur un clone QEMU 9.2 :

```bash
git clone https://gitlab.com/qemu-project/qemu.git -b v9.2.4
cp -r hw/ include/ qemu/
cp hw/arm/calypso/meson.build qemu/hw/arm/calypso/

cd qemu && mkdir build && cd build
../configure --target-list=arm-softmmu
ninja
```

## Usage

### Lancer le firmware seul

```bash
sudo bash run-layer1.sh
```

Affiche la console debug du firmware sur stdout.

### Tester avec mobile

```bash
sudo bash test-mobile.sh
```

Lance QEMU + `l1ctl_bridge.py` + `mobile`, attend 20s une reponse L1CTL, affiche les logs.

### Manuellement

```bash
# Terminal 1 : QEMU
sudo ./build/qemu-system-arm \
  -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor unix:/tmp/qemu-calypso-mon.sock,server,nowait \
  -kernel /path/to/layer1.highram.elf

# Unpauser
printf 'cont\n' | socat - UNIX-CONNECT:/tmp/qemu-calypso-mon.sock

# Terminal 2 : bridge
sudo python3 l1ctl_bridge.py /dev/pts/X

# Terminal 3 : mobile
sudo mobile
```

## Structure

```
hw/arm/calypso/
  calypso_mb.c          Machine board (CPU ARM946, RAM, Flash)
  calypso_soc.c         SoC (instancie les peripheriques)
  calypso_trx.c         TRX/DSP/TPU/TSP/SIM/ULPD (~1300 lignes)
  meson.build

hw/char/calypso_uart.c  UART 16550 (sercomm, IRQ level-sensitive)
hw/intc/calypso_inth.c  Interrupt controller (level-sensitive)
hw/ssi/calypso_spi.c    SPI + TWL3025 ABB
hw/ssi/calypso_i2c.c    I2C stub
hw/timer/calypso_timer.c

include/hw/arm/calypso/  Headers

l1ctl_bridge.py         Bridge sercomm <-> /tmp/osmocom_l2
sercomm_relay.py        Relay sercomm pour osmoload
run-layer1.sh           Lance QEMU avec layer1.elf
test-mobile.sh          Test QEMU + bridge + mobile

doc/                    Memory map, IRQ map, status detaille
```

## Status

**Fonctionne** : boot firmware, UART, SPI/ABB, DSP boot, SIM ATR, TDMA timer, TRX bridge, reception L1CTL

**WIP** : le firmware recoit L1CTL_RESET_REQ mais ne renvoie pas encore L1CTL_RESET_CONF (blocage dans le reset handler, hardware manquant)

Voir [doc/status.md](doc/status.md) pour les details.

## Dependances

- QEMU 9.2.x
- `socat`
- `osmocom-bb` : `mobile`, `osmoload` (optionnel)
- Python 3
- Firmware : `layer1.highram.elf` (build osmocom-bb pour compal_e88 highram)

## Licence

GPL-2.0-or-later (comme QEMU)
