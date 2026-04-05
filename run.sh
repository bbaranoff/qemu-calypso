#!/bin/bash
# run.sh — QEMU Calypso pipeline
#
# QEMU (layer1 firmware) -serial pty→ sercomm_udp.py (5700-5702) ↔ osmo-bts-trx
# QEMU exposes /tmp/osmocom_l2_1 for mobile/ccch_scan
#
set -euo pipefail

SESSION="calypso"
QEMU="/root/qemu/build/qemu-system-arm"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BRIDGE="/opt/GSM/qemu-src/sercomm_udp.py"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
L1CTL_SOCK="/tmp/osmocom_l2_1"
QEMU_LOG="/tmp/qemu.log"

# ── Cleanup ──
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -9 qemu-system-arm 2>/dev/null || true
pkill -9 -f sercomm_udp.py 2>/dev/null || true
pkill -9 osmo-bts-trx 2>/dev/null || true
pkill -9 mobile 2>/dev/null || true
pkill -9 ccch_scan 2>/dev/null || true
systemctl stop osmo-bts-trx 2>/dev/null || true
rm -f "$L1CTL_SOCK"
sleep 1

# ── QEMU ──
$QEMU -M calypso \
  -kernel "$FW" \
  -serial pty \
    -S -display none \
  > "$QEMU_LOG" 2>&1 &
QEMU_PID=$!
sleep 2

PTY=$(grep -o "/dev/pts/[0-9]*" "$QEMU_LOG" | head -1)
if [ -z "$PTY" ]; then
    echo "FATAL: QEMU did not allocate a PTY"
    cat "$QEMU_LOG"
    exit 1
fi
echo "QEMU PID=$QEMU_PID PTY=$PTY"

# Wait for L1CTL socket
echo -n "Waiting for L1CTL socket..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
[ -S "$L1CTL_SOCK" ] && echo " OK" || echo " TIMEOUT (continuing)"

# ── tmux ──
tmux new-session -d -s "$SESSION" -n qemu
tmux send-keys -t "$SESSION:qemu" "tail -f $QEMU_LOG" C-m

tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" "python3 $BRIDGE $PTY" C-m
sleep 1

tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" "osmo-bts-trx -c $BTS_CFG" C-m

#tmux new-window -t "$SESSION" -n ccch
#tmux send-keys -t "$SESSION:ccch" "sleep 5 && ccch_scan -i 127.0.0.1 -a 514 -s $L1CTL_SOCK" C-m
tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" "sleep 5 && mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

tmux new-window -t "$SESSION" -n grgsm
tmux send-keys -t "$SESSION:grgsm" "echo 'grgsm_decode ready — run: grgsm_decode -c /tmp/gsm_burst.cfile -m BCCH_SDCCH4 -v'" C-m

tmux new-window -t "$SESSION" -n shell

tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"
