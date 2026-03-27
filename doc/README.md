# QEMU Calypso - OsmocomBB Layer 1 Emulation

Emulation TI Calypso DBB (Digital Baseband) pour QEMU 9.2, permettant de faire tourner le firmware OsmocomBB Layer 1 et de connecter `mobile` (osmocom-bb layer23).

## Quickstart

```bash
# 1. Build QEMU avec le support Calypso
cd qemu && mkdir build && cd build
../configure --target-list=arm-softmmu && ninja

# 2. Lancer QEMU + bridge + mobile
sudo bash test-mobile.sh
```

## Architecture

```
 mobile (layer23)
    |  L1CTL (unix socket /tmp/osmocom_l2)
    v
 l1ctl_bridge.py
    |  sercomm frames (PTY /dev/pts/X)
    v
 QEMU Calypso
    |  ARM946 CPU running layer1.highram.elf
    +-- UART modem (sercomm TX/RX)
    +-- INTH (interrupt controller, level-sensitive)
    +-- SPI + TWL3025 ABB
    +-- DSP API RAM (shared memory)
    +-- TPU (Timing Program Unit)
    +-- TSP (Time Serial Port)
    +-- SIM controller (ATR injection)
    +-- TRX bridge (UDP, osmo-bts-trx compatible)
```

## Fichiers

### Hardware QEMU (copier dans qemu/)

| Fichier | Description |
|---------|-------------|
| `hw/arm/calypso/calypso_mb.c` | Machine board : CPU, RAM, Flash, boot |
| `hw/arm/calypso/calypso_soc.c` | SoC : instancie tous les peripheriques |
| `hw/arm/calypso/calypso_trx.c` | TRX bridge, DSP API, TPU, TSP, SIM, ULPD |
| `hw/arm/calypso/meson.build` | Build system |
| `hw/char/calypso_uart.c` | UART 16550-like avec sercomm |
| `hw/intc/calypso_inth.c` | Interrupt controller level-sensitive |
| `hw/ssi/calypso_spi.c` | SPI + TWL3025 ABB (power management) |
| `hw/ssi/calypso_i2c.c` | I2C stub |
| `hw/timer/calypso_timer.c` | Hardware timers |
| `include/hw/arm/calypso/*.h` | Headers |

### Scripts

| Fichier | Description |
|---------|-------------|
| `run-layer1.sh` | Lance QEMU avec layer1.elf, affiche la sortie console |
| `test-mobile.sh` | Test complet : QEMU + l1ctl_bridge + mobile |
| `l1ctl_bridge.py` | Bridge sercomm (DLCI 4) <-> unix socket L1CTL |
| `sercomm_relay.py` | Relay sercomm pour osmoload (loader phase) |
