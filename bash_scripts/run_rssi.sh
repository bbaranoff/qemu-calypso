#!/bin/bash
# run_rssi.sh — lance le firmware rssi à la place de layer1.
#
# Diffère de run.sh sur le seul chargement du firmware. Toute la plomberie
# (QEMU + bridge + osmocon + BTS + tmux) est partagée via exec run.sh.
#
# compal_e88 ne ship pas de rssi.highram dans ce build, on prend compal_e99
# (même SoC Calypso, rssi-firmware portable). Override via FW_BOARD si tu
# préfères un autre board (pirelli_dpl10, gta0x, se_j100).
#
# Usage:
#   ./run_rssi.sh                       # defaults: compal_e99 rssi
#   FW_BOARD=gta0x ./run_rssi.sh        # board override
#   FW_ELF=/path/to/elf FW_BIN=... ./run_rssi.sh   # full custom path

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FW_BOARD="${FW_BOARD:-compal_e99}"
FW_VARIANT="${FW_VARIANT:-rssi.highram}"

# Honor explicit FW_ELF/FW_BIN if set, otherwise compute from board+variant
export FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/${FW_BOARD}/${FW_VARIANT}.elf}"
export FW_BIN="${FW_BIN:-/opt/GSM/firmware/board/${FW_BOARD}/${FW_VARIANT}.bin}"

if [ ! -f "$FW_ELF" ]; then
    echo "[run_rssi] ERROR: $FW_ELF introuvable" >&2
    echo "[run_rssi] boards dispos avec rssi :" >&2
    find /opt/GSM/firmware/board -name "rssi.highram.elf" 2>/dev/null | sed 's|^|  |' >&2
    exit 1
fi

if [ ! -f "$FW_BIN" ]; then
    echo "[run_rssi] ERROR: $FW_BIN introuvable" >&2
    exit 1
fi

echo "[run_rssi] FW_ELF = $FW_ELF"
echo "[run_rssi] FW_BIN = $FW_BIN"

exec "$SCRIPT_DIR/run.sh" "$@"
