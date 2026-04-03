#!/bin/bash
# Sync qemu-src between host and osmo-operator-1 container
# Usage: ./sync.sh [push|pull|check]
#   pull  = container → host (default)
#   push  = host → container
#   check = compare only

CONTAINER="osmo-operator-1"
HOST_DIR="/home/nirvana/qemu-src"
CONT_DIR="/opt/GSM/qemu-src"

FILES=(
  hw/arm/calypso/calypso_trx.c
  hw/arm/calypso/calypso_c54x.c
  hw/arm/calypso/calypso_c54x.h
  hw/arm/calypso/calypso_tdma_hw.c
  hw/arm/calypso/calypso_tdma_hw.h
  hw/arm/calypso/calypso_soc.c
  hw/arm/calypso/calypso_mb.c
  hw/arm/calypso/l1ctl_sock.c
  hw/arm/calypso/meson.build
  hw/char/calypso_uart.c
  hw/intc/calypso_inth.c
  include/hw/arm/calypso/calypso_trx.h
  include/hw/arm/calypso/calypso_soc.h
  include/hw/arm/calypso/calypso_uart.h
  bridge.py
  run.sh
)

MODE="${1:-pull}"
DIRTY=0

for f in "${FILES[@]}"; do
  h=$(md5sum "$HOST_DIR/$f" 2>/dev/null | cut -d' ' -f1)
  c=$(docker exec "$CONTAINER" md5sum "$CONT_DIR/$f" 2>/dev/null | cut -d' ' -f1)

  if [ -z "$h" ] && [ -z "$c" ]; then
    continue
  elif [ "$h" = "$c" ]; then
    echo "✅ $f"
  else
    DIRTY=1
    if [ "$MODE" = "check" ]; then
      echo "❌ $f  host=${h:-MISSING} cont=${c:-MISSING}"
    elif [ "$MODE" = "pull" ]; then
      docker cp "$CONTAINER:$CONT_DIR/$f" "$HOST_DIR/$f"
      echo "⬇️  $f  (pulled)"
    elif [ "$MODE" = "push" ]; then
      docker cp "$HOST_DIR/$f" "$CONTAINER:$CONT_DIR/$f"
      echo "⬆️  $f  (pushed)"
    fi
  fi
done

if [ "$DIRTY" = "0" ]; then
  echo "All files in sync."
elif [ "$MODE" = "check" ]; then
  echo "Files out of sync. Use: ./sync.sh pull|push"
fi
