#!/bin/bash
# make-overlay.sh — back-port qemu-src (working tree, SOURCE OF TRUTH) into the
#   qemu-calypso overlay. Reverse of make-fork.sh (which does overlay -> fork).
#
#     genuine QEMU  +  qemu-calypso (overlay)   =  qemu-src (assembled working tree)
#     make-fork.sh    : overlay + genuine  ->  fork/working-tree
#     make-overlay.sh : working-tree       ->  overlay            (this script)
#
# Copies every file the overlay TRACKS (git ls-files) from qemu-src into the overlay,
# preserving layout. Files that live only in the overlay (backups, *.minimal) and all
# of vanilla QEMU are left untouched. Nothing is ever deleted.
#
# New Calypso files created in qemu-src (e.g. calypso.nu.env) are NOT auto-added:
# `git add` them in the overlay once, then this script keeps them in sync.
#
# Usage: ./make-overlay.sh [--dry-run] [OVERLAY_DIR] [SRC_DIR]
set -euo pipefail
DRY=0; [ "${1:-}" = "--dry-run" ] && { DRY=1; shift; }
OVERLAY="${1:-/opt/GSM/qemu-calypso}"
SRC="${2:-$(cd "$(dirname "$0")" && pwd)}"
[ -d "$OVERLAY/.git" ] || { echo "!! overlay git repo not found: $OVERLAY" >&2; exit 1; }
[ -f "$SRC/VERSION" ]  || { echo "!! qemu-src (VERSION) not found: $SRC" >&2; exit 1; }
echo "src (truth) : $SRC"
echo "overlay     : $OVERLAY"
[ "$DRY" = 1 ] && echo "mode        : DRY-RUN (no writes)"
echo "----"
sync=0; same=0; only=0
while IFS= read -r f; do
    if [ -f "$SRC/$f" ]; then
        if cmp -s "$SRC/$f" "$OVERLAY/$f" 2>/dev/null; then
            same=$((same+1))
        else
            echo "  sync  $f"
            if [ "$DRY" = 0 ]; then
                mkdir -p "$OVERLAY/$(dirname "$f")"
                cp -a "$SRC/$f" "$OVERLAY/$f"
            fi
            sync=$((sync+1))
        fi
    else
        only=$((only+1))
    fi
done < <(cd "$OVERLAY" && git ls-files)
echo "----"
if [ "$DRY" = 1 ]; then
    echo "$sync file(s) WOULD sync, $same unchanged, $only overlay-only (untouched)"
    echo "re-run without --dry-run to apply."
else
    echo "$sync file(s) synced, $same unchanged, $only overlay-only (untouched)"
fi
echo "review: cd $OVERLAY && git status -s && git diff --stat"
