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

# Snapshot on push
if [ "$MODE" = "push" ] && [ "$DIRTY" = "1" ]; then
  STAMP=$(date +%Y%m%d-%H%M%S)
  EVENT="${2:-}"
  if [ -n "$EVENT" ]; then
    SNAP="/home/nirvana/ALL-QEMUs/qemu-calypso-${STAMP}-${EVENT}"
  else
    SNAP="/home/nirvana/ALL-QEMUs/qemu-calypso-${STAMP}"
  fi
  mkdir -p "$SNAP"
  for f in "${FILES[@]}"; do
    mkdir -p "$SNAP/$(dirname $f)"
    cp "$HOST_DIR/$f" "$SNAP/$f" 2>/dev/null
  done
  echo "$STAMP ${EVENT:-push}" >> "/home/nirvana/ALL-QEMUs/HISTORY.log"
  echo "📸 Snapshot: $SNAP"
fi

# Check binary age vs source files
BINARY="$CONT_DIR/build/qemu-system-arm"
BIN_TS=$(docker exec "$CONTAINER" stat -c %Y "$BINARY" 2>/dev/null)
if [ -n "$BIN_TS" ]; then
  BIN_DATE=$(docker exec "$CONTAINER" date -d "@$BIN_TS" "+%H:%M:%S" 2>/dev/null)
  STALE=0
  for f in "${FILES[@]}"; do
    [[ "$f" == *.py || "$f" == *.sh ]] && continue
    SRC_TS=$(docker exec "$CONTAINER" stat -c %Y "$CONT_DIR/$f" 2>/dev/null)
    if [ -n "$SRC_TS" ] && [ "$SRC_TS" -gt "$BIN_TS" ]; then
      SRC_DATE=$(docker exec "$CONTAINER" date -d "@$SRC_TS" "+%H:%M:%S" 2>/dev/null)
      echo "⚠️  STALE: $f ($SRC_DATE) newer than binary ($BIN_DATE)"
      STALE=1
    fi
  done
  if [ "$STALE" = "0" ]; then
    echo "🔨 Binary up to date ($BIN_DATE)"
  else
    echo "🔨 Binary OUTDATED ($BIN_DATE) — run: ninja -C build"
  fi
else
  echo "⚠️  Binary not found at $BINARY"
fi
