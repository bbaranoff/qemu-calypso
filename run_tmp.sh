#!/bin/bash
# run_tmp.sh — Lance fake_trx + BTS + QEMU + POWERON + mobile en tmux
set -e

SESSION="osmocom"
TMUX_SOCKET="/tmp/osmocom_tmux"

echo "=== [1] Kill old processes ==="
pkill -9 qemu-system-arm 2>/dev/null || true
pkill -9 mobile 2>/dev/null || true
pkill -f l1ctl_bridge 2>/dev/null || true
pkill -9 -f fake_trx 2>/dev/null || true
systemctl stop osmo-bts-trx 2>/dev/null || true
tmux -S "$TMUX_SOCKET" kill-session -t "$SESSION" 2>/dev/null || true
sleep 1
rm -f /tmp/osmocom_l2* /tmp/qemu-calypso-mon*.sock /tmp/osmocom_tmux

echo "=== [2] Create tmux ==="
tmux -S "$TMUX_SOCKET" new-session -d -s "$SESSION" -n faketrx
tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n qemu0
tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n ue_g1

echo "=== [3] Start fake_trx ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:faketrx" \
  "python3 /opt/GSM/osmocom-bb/src/target/trx_toolkit/fake_trx.py" C-m
sleep 3

echo "=== [4] Start osmo-bts-trx ==="
systemctl start osmo-bts-trx
sleep 5

echo "=== [5] Send RXTUNE+TXTUNE+POWERON to fake_trx MS (6701) ==="
echo -ne 'CMD RXTUNE 1805600\0' | socat - UDP:127.0.0.1:6701
echo -ne 'CMD TXTUNE 1710600\0' | socat - UDP:127.0.0.1:6701
echo -ne 'CMD POWERON\0' | socat - UDP:127.0.0.1:6701
sleep 1

echo "=== [6] Start QEMU ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:qemu0" \
  "CALYPSO_TRX_PORT=0 /opt/GSM/qemu/build/qemu-system-arm -M calypso -cpu arm946 -display none -serial pty -serial pty -monitor unix:/tmp/qemu-calypso-mon-ms1.sock,server,nowait -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf 2>&1 | tee /var/log/osmocom/qemu.log" C-m
sleep 5

echo "=== [6b] Detect PTY and start l1ctl_bridge ==="
PTY=$(grep -o '/dev/pts/[0-9]*' /var/log/osmocom/qemu.log 2>/dev/null | head -1)
if [ -z "$PTY" ]; then
  echo "ERROR: no PTY detected"
else
  echo "PTY=$PTY"
  # Send cont to QEMU monitor
  printf 'cont\n' | socat - UNIX-CONNECT:/tmp/qemu-calypso-mon-ms1.sock 2>/dev/null || true
  sleep 2
  tmux -S "$TMUX_SOCKET" new-window -t "$SESSION" -n bridge
  tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:bridge" \
    "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=6702 python3 l1ctl_bridge.py $PTY /tmp/osmocom_l2_1" C-m
fi
sleep 10

echo "=== [7] Start mobile ==="
tmux -S "$TMUX_SOCKET" send-keys -t "$SESSION:ue_g1" \
  "mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

echo ""
echo "╔════════════════════════════════════════════════════╗"
echo "║  Attach: tmux -S $TMUX_SOCKET attach               ║"
echo "║  Windows: faketrx | qemu0 | ue_g1                 ║"
echo "║  QEMU log:   /tmp/qemu-bridge.log                 ║"
echo "║  Bridge log: /var/log/osmocom/bridge.log           ║"
echo "║                                                    ║"
echo "║  tail -f /tmp/qemu-bridge.log  (dans un autre term)║"
echo "╚════════════════════════════════════════════════════╝"
