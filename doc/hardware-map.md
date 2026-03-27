# Calypso Memory Map & Peripherals

## Memory Map

| Adresse | Taille | Description |
|---------|--------|-------------|
| 0x00000000 | 4 MiB | Flash (pflash_cfi01, Intel 28F320J3) |
| 0x00800000 | 256 KiB | IRAM (aliasable a 0x00000000 via CNTL) |
| 0x01000000 | 8 MiB | XRAM |

## Peripheriques

| Adresse | Taille | Peripherique | Fichier |
|---------|--------|-------------|---------|
| 0xFFFE0000 | 0x100 | SIM controller | calypso_trx.c |
| 0xFFFE1800 | 0x100 | I2C | calypso_i2c.c |
| 0xFFFE3000 | 0x100 | SPI + TWL3025 ABB | calypso_spi.c |
| 0xFFFE3800 | 0x100 | Timer 1 | calypso_timer.c |
| 0xFFFE3C00 | 0x100 | Timer 2 | calypso_timer.c |
| 0xFFFE4800 | 0x100 | Keypad (stub) | calypso_soc.c |
| 0xFFFE6800 | 0x100 | Timer 6800 (stub) | calypso_soc.c |
| 0xFFFE8000 | 0x100 | MMIO 80xx (stub) | calypso_soc.c |
| 0xFFFEF000 | 0x100 | CONF (stub) | calypso_soc.c |
| 0xFFFF0000 | 0x2000 | DSP API RAM | calypso_trx.c |
| 0xFFFF1000 | 0x100 | TPU registers | calypso_trx.c |
| 0xFFFF5000 | 0x100 | UART IrDA | calypso_uart.c |
| 0xFFFF5800 | 0x100 | UART Modem | calypso_uart.c |
| 0xFFFF9000 | 0x800 | TPU RAM | calypso_trx.c |
| 0xFFFF9800 | 0x100 | MMIO 98xx (stub) | calypso_soc.c |
| 0xFFFFA800 | 0x100 | ULPD | calypso_trx.c |
| 0xFFFFB000 | 0x100 | TSP | calypso_trx.c |
| 0xFFFFF000 | 0x100 | DPLL (stub) | calypso_soc.c |
| 0xFFFFF900 | 0x100 | RHEA bridge (stub) | calypso_soc.c |
| 0xFFFFFA00 | 0x100 | INTH | calypso_inth.c |
| 0xFFFFFB00 | 0x100 | CLKM (stub) | calypso_soc.c |
| 0xFFFFFD00 | 0x100 | CNTL (EXTRA_CONF) | calypso_soc.c |
| 0xFFFFFF00 | 0x100 | DIO (stub) | calypso_soc.c |

## IRQ Map

| IRQ | Peripherique |
|-----|-------------|
| 0 | (watchdog) |
| 1 | Timer 1 |
| 2 | Timer 2 |
| 4 | TPU Frame |
| 5 | DSP API |
| 7 | UART Modem |
| 8 | SIM |
| 13 | SPI |
| 18 | UART IrDA |

## INTH (Interrupt Controller)

**Level-sensitive** : suit les niveaux des entrees en temps reel.
Le firmware NE fait PAS d'acknowledge via IRQ_CTRL.
Quand le peripherique baisse son IRQ (ex: UART clear IIR), l'INTH le voit immediatement.

## DSP API

Shared memory a 0xFFFF0000, double-buffered (page 0 / page 1).
Boot sequence : firmware ecrit 0x0000 dans DSP_DL_STATUS (0x0FFE), QEMU repond avec 0x0001 (BOOT) apres 3 lectures, puis firmware ecrit 0x0002 (READY).

## TWL3025 ABB (via SPI)

Power management IC emule dans calypso_spi.c. Bloque les ecritures TOGBR1 bit 0 (poweroff) pour garder QEMU vivant.
