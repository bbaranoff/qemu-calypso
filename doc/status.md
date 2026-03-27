# Status & Known Issues

## Ce qui marche

- Boot complet du firmware layer1.highram.elf (OsmocomBB Layer 1)
- Console debug via sercomm DLCI 10
- UART TX/RX avec interrupts level-sensitive
- SPI/ABB : init TWL3025, blocage poweroff
- SIM : injection ATR via rxDoneFlag en IRAM
- DSP : boot sequence, API RAM, re-boot apres reset
- TPU : TDMA frame timer 4.615ms, IRQ TPU_FRAME
- TRX bridge : ports UDP TRXC/TRXD compatibles osmo-bts-trx
- l1ctl_bridge.py : relais sercomm <-> /tmp/osmocom_l2
- mobile se connecte et envoie L1CTL_RESET_REQ
- Le firmware recoit et lit les 12 bytes du L1CTL_RESET_REQ

## Ce qui ne marche pas encore

- **L1CTL_RESET_CONF** : le firmware recoit L1CTL_RESET_REQ mais ne repond pas.
  Le handler L1CTL_RESET essaie probablement de reinitialiser le DSP/TPU et
  se bloque sur un registre hardware manquant.

- **DSP API Version** : retourne 0x0000 au lieu de 0x3606 pendant l'init
  (timing issue : version ecrite trop tard dans le boot sequence)

- **TPU ctrl bit timeout** : le firmware imprime "Timeout while waiting for TPU ctrl bit!"
  pendant l'init (non-fatal, le boot continue)

- **Device IDs** : tous a 0x0000 (stubs retournent 0)

## Fixes cles appliques

### 1. INTH level-sensitive (calypso_inth.c)
Le plus important. L'INTH etait edge-triggered avec acknowledge IRQ_CTRL.
Le vrai Calypso est level-sensitive : le firmware ne fait jamais d'acknowledge.
Sans ce fix, l'IRQ UART restait bloquee derriere l'IRQ TPU_FRAME.

### 2. DSP re-boot (calypso_trx.c)
Quand le firmware ecrit 0x0000 dans DSP_DL_STATUS, on reset sync_dsp_booted
pour permettre un nouveau boot. Sans ca, le firmware se bloquait en polling
DSP_DL_STATUS apres un L1CTL_RESET.

### 3. TPU CTRL instant clear (calypso_trx.c)
Le firmware ecrit TPU_CTRL_ENABLE puis poll pour le voir cleared.
Le virtual time n'avance pas pendant les polling loops CPU.
On clear le bit immediatement dans le write handler.

### 4. UART simplifie (calypso_uart.c)
IRQ raise/lower direct au lieu du mecanisme irq_level + pulse.
Coherent avec l'INTH level-sensitive.
