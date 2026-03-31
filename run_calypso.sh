#!/bin/bash
# Run Calypso GSM stack in Docker container osmo-operator-1
# Usage: docker exec -it osmo-operator-1 bash /opt/GSM/qemu/run_calypso.sh

set -e

FW=/opt/GSM/firmware/board/compal_e88
QEMU=/opt/GSM/qemu/build/qemu-system-arm
BRIDGE=/opt/GSM/qemu/l1ctl_bridge.py
XCVR=/opt/GSM/osmocom-bb-transceiver/src/host/layer23/src/transceiver/transceiver

# Kill previous
pkill -9 -f qemu-system-arm 2>/dev/null || true
pkill -9 -f transceiver 2>/dev/null || true
pkill -9 -f l1ctl_bridge 2>/dev/null || true
pkill -9 -f "mobile " 2>/dev/null || true
pkill -9 -f osmo-bts-trx 2>/dev/null || true
rm -f /tmp/osmocom_l2* /tmp/qemu-*.sock
tmux kill-session -t calypso 2>/dev/null || true
sleep 1

# Create tmux session
tmux new-session -d -s calypso -n trx0
tmux split-window -t calypso:trx0 -h
tmux new-window -t calypso -n mob
tmux split-window -t calypso:mob -h
tmux new-window -t calypso -n stack
tmux split-window -t calypso:stack -v
tmux split-window -t calypso:stack.1 -v

# Window layout:
#   trx0:  [QEMU TRX0 | Bridge TRX0]
#   mob:   [QEMU Mobile | Bridge Mobile]
#   stack: [Transceiver | BTS | Mobile app]

echo "Starting QEMU TRX0..."
tmux send-keys -t calypso:trx0.0 \
  "CALYPSO_TRX_PORT=0 CALYPSO_AIR_LOCAL=4800 CALYPSO_AIR_PEER=4801 $QEMU -M calypso -cpu arm946 -serial pty -serial pty -monitor unix:/tmp/qemu-trx0.sock,server,nowait -kernel $FW/trx.highram.elf" Enter
sleep 4

echo "Starting QEMU Mobile..."
tmux send-keys -t calypso:mob.0 \
  "CALYPSO_TRX_PORT=0 CALYPSO_AIR_LOCAL=4801 CALYPSO_AIR_PEER=4800 $QEMU -M calypso -cpu arm946 -serial pty -serial pty -monitor unix:/tmp/qemu-mobile.sock,server,nowait -kernel $FW/layer1.highram.elf" Enter
sleep 4

# Get PTYs
TRX0_S0=$(echo 'info chardev' | socat - UNIX-CONNECT:/tmp/qemu-trx0.sock 2>/dev/null | grep serial0 | grep -o '/dev/pts/[0-9]*')
MOB_S0=$(echo 'info chardev' | socat - UNIX-CONNECT:/tmp/qemu-mobile.sock 2>/dev/null | grep serial0 | grep -o '/dev/pts/[0-9]*')
echo "TRX0 PTY: $TRX0_S0"
echo "Mobile PTY: $MOB_S0"

if [ -z "$TRX0_S0" ] || [ -z "$MOB_S0" ]; then
    echo "ERROR: Could not get PTY paths"
    exit 1
fi

echo "Starting bridges..."
tmux send-keys -t calypso:trx0.1 \
  "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=0 python3 l1ctl_bridge.py $TRX0_S0" Enter
tmux send-keys -t calypso:mob.1 \
  "cd /opt/GSM/qemu && BRIDGE_AIR_PORT=0 python3 l1ctl_bridge.py $MOB_S0 /tmp/osmocom_l2_ms" Enter
sleep 2

echo "Starting transceiver..."
tmux send-keys -t calypso:stack.0 \
  "$XCVR -a 100" Enter
sleep 3

echo "Starting osmo-bts-trx..."
tmux send-keys -t calypso:stack.1 \
  "osmo-bts-trx -c /root/osmo-bts-trx.cfg" Enter
sleep 5

echo "Starting mobile..."
tmux send-keys -t calypso:stack.2 \
  "mobile -c /etc/osmocom/mobile.cfg" Enter
sleep 3

echo ""
echo "=== All started ==="
echo "tmux attach -t calypso"
echo ""
echo "Windows:"
echo "  0:trx0   - QEMU TRX0 | Bridge TRX0"
echo "  1:mob    - QEMU Mobile | Bridge Mobile"
echo "  2:stack  - Transceiver | BTS | Mobile app"
