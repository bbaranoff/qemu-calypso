#!/bin/bash
# make-fork.sh — genuine QEMU (9.2.x) + this overlay (qemu-calypso) = the Calypso qemu fork.
#   QEMU + qemu-calypso = qemu
# Usage: ./make-fork.sh [GENUINE_DIR] [OUT_DIR]
set -euo pipefail
GEN="${1:-/opt/GSM/QEMU}"
OUT="${2:-/opt/GSM/qemu-fork}"
OVERLAY="$(cd "$(dirname "$0")" && pwd)"
[ -f "$GEN/VERSION" ] || { echo "!! genuine qemu (with VERSION) not found at: $GEN" >&2; exit 1; }
echo "base    : $GEN (QEMU $(cat "$GEN/VERSION"))"
echo "overlay : $OVERLAY"
echo "out     : $OUT"
rm -rf "$OUT"
cp -a "$GEN" "$OUT"
rm -rf "$OUT/.git"                 # fork is a working tree, not genuine git
cp -a "$OVERLAY/." "$OUT/"         # apply overlay (preserves layout)
rm -rf "$OUT/.git" "$OUT/make-fork.sh"   # drop overlay git + this script from the fork
echo "OK -> fork at $OUT"
echo "build: cd $OUT && mkdir -p build && cd build && ../configure --target-list=arm-softmmu && ninja qemu-system-arm"
