# Extraire le zip
SRC=/home/nirvana/qemu-calypso
QEMU=~/qemu

# ── Créer les dossiers manquants ─────────────────────────────────────────────
mkdir -p $QEMU/hw/arm/calypso
mkdir -p $QEMU/include/hw/arm/calypso

# ── hw/arm/calypso/ ──────────────────────────────────────────────────────────
cp $SRC/hw/arm/calypso/calypso_mb.c       $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/calypso_soc.c      $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/calypso_trx.c      $QEMU/hw/arm/calypso/
cp $SRC/hw/arm/calypso/meson.build        $QEMU/hw/arm/calypso/

# ── hw/char/ ─────────────────────────────────────────────────────────────────
cp $SRC/hw/char/calypso_uart.c            $QEMU/hw/char/
# meson.build existe déjà dans QEMU → ajouter la ligne manuellement (voir bas)

# ── hw/i2c/ ──────────────────────────────────────────────────────────────────
cp $SRC/hw/i2c/calypso_i2c.c             $QEMU/hw/i2c/
# meson.build existe déjà → ajouter la ligne manuellement

# ── hw/intc/ ─────────────────────────────────────────────────────────────────
cp $SRC/hw/intc/calypso_inth.c           $QEMU/hw/intc/
# meson.build existe déjà → ajouter la ligne manuellement

# ── hw/ssi/ ──────────────────────────────────────────────────────────────────
cp $SRC/hw/ssi/calypso_spi.c             $QEMU/hw/ssi/
# meson.build existe déjà → ajouter la ligne manuellement

# ── hw/timer/ ────────────────────────────────────────────────────────────────
cp $SRC/hw/timer/calypso_timer.c         $QEMU/hw/timer/
# meson.build existe déjà → ajouter la ligne manuellement

# ── include/ ─────────────────────────────────────────────────────────────────
cp $SRC/include/hw/arm/calypso/calypso_inth.h   $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_soc.h    $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_spi.h    $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_timer.h  $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_trx.h    $QEMU/include/hw/arm/calypso/
cp $SRC/include/hw/arm/calypso/calypso_uart.h   $QEMU/include/hw/arm/calypso/

# ── Interface monitor ─────────────────────────────────────────────────────────
cp $SRC/calypso/index.html               ~/calypso_monitor/
cp $SRC/calypso/ws_bridge.py             ~/calypso_monitor/
