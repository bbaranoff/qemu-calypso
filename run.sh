#!/bin/bash
# run.sh — Calypso QEMU full pipeline in tmux.
#
# Topology:
#   mobile  ──unix /tmp/osmocom_l2_1── bridge ──unix /tmp/calypso_modem.sock── QEMU UART modem
#                                       │                                       │
#                                       └─UDP 5701/5702──── osmo-bts-trx        │
#                                       └─UDP 6802 ↔ 6702 ─────────────────── QEMU BSP
#
# Bridge is server on both unix sockets, started before QEMU.
# QEMU connects with reconnect=1 → no boot race.
#
# Usage:
#   ./run.sh         start full stack in tmux 'calypso'
#   ./run.sh kill    tear everything down
set -uo pipefail

SESSION="calypso"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"
MODEM_SOCK="/tmp/calypso_modem.sock"
L1CTL_SOCK="/tmp/osmocom_l2_1"
QEMU_LOG="/tmp/qemu.log"

VENV="/root/.env"
[ -f "$VENV/bin/activate" ] && source "$VENV/bin/activate"

kill_all() {
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    pkill -9 qemu-system-arm 2>/dev/null || true
    pkill -9 -f "$BRIDGE"    2>/dev/null || true
    pkill -9 -f osmo-bts-trx 2>/dev/null || true
    pkill -9 -f '^mobile'    2>/dev/null || true
    rm -f "$MODEM_SOCK" "$L1CTL_SOCK" /tmp/osmocom_l2_*
    sleep 0.3
}

if [ "${1:-}" = "kill" ]; then
    kill_all; echo "killed."; exit 0
fi

for f in "$QEMU" "$FW" "$BRIDGE"; do
    [ -e "$f" ] || { echo "FATAL: missing $f"; exit 1; }
done

kill_all
trap 'echo "[run.sh] cleanup"; kill_all; exit 130' INT TERM

/etc/osmocom/status.sh stop  >/dev/null 2>&1 || true
/etc/osmocom/osmo-start.sh   >/dev/null 2>&1 || true

# ── tmux layout ─────────────────────────────────────────────────────
tmux new-session -d -s "$SESSION" -n log
tmux send-keys -t "$SESSION:log" ": > $QEMU_LOG && tail -F $QEMU_LOG" C-m

# bridge: server on both unix sockets
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" \
    "python3 $BRIDGE server:$MODEM_SOCK --sock $L1CTL_SOCK" C-m

echo -n "waiting modem socket"
for i in $(seq 1 40); do
    [ -S "$MODEM_SOCK" ] && break
    sleep 0.25; echo -n "."
done
echo

# QEMU — DISCOVERY MODE
#
# CALYPSO_BSP_DARAM_ADDR=0  → BSP refuses to DMA bursts and only logs.
#                             c54x.c emits "[c54x] DARAM RD HIST ..."
#                             every 50k reads inside PC range
#                             0xe25e..0xe27f (the new FB-det loop).
# CALYPSO_BSP_BYPASS_BDLENA=1 → BSP ignores BDLENA gate (debug aid).
# CALYPSO_DBG=all / DEBUG=ALL → all c54x/trx/uart logs.
tmux new-window -t "$SESSION" -n qemu
tmux send-keys -t "$SESSION:qemu" \
    "CALYPSO_BSP_DARAM_ADDR=0x2e80 \
     CALYPSO_BSP_DARAM_LEN=64 \
     CALYPSO_BSP_BYPASS_BDLENA=1 \
     CALYPSO_DBG=all DEBUG=ALL \
     $QEMU -M calypso -kernel $FW \
        -chardev socket,id=modem,path=$MODEM_SOCK,server=off,reconnect=1 \
        -serial chardev:modem -display none 2>&1 | tee $QEMU_LOG" C-m

echo -n "waiting L1CTL socket"
for i in $(seq 1 40); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 0.25; echo -n "."
done
echo

tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" \
    "sleep 2 && osmo-bts-trx -c /etc/osmocom/osmo-bts-trx.cfg" C-m

tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" \
    "sleep 6 && mobile -c $MOBILE_CFG" C-m

# Discovery helper window: tail the histogram lines so you can copy
# the new DARAM address straight from the log.
tmux new-window -t "$SESSION" -n discover
tmux send-keys -t "$SESSION:discover" \
    "echo '── DARAM RD HIST (top cells read by PC e25e..e27f) ──' && \
     tail -F $QEMU_LOG | grep --line-buffered 'DARAM RD HIST'" C-m

tmux new-window -t "$SESSION" -n shell

tmux select-window -t "$SESSION:discover"
exec tmux attach -t "$SESSION"
