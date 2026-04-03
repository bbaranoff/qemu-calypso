#!/bin/bash
# Pipeline: QEMU (layer1) ← UART/sercomm → mobile (/tmp/osmocom_l2_1)
#           bridge (5700-5702) ↔ osmo-bts-trx

SESSION="calypso"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
L1CTL_SOCK="/tmp/osmocom_l2_1"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"

# Cleanup
tmux kill-session -t $SESSION 2>/dev/null
killall -9 qemu-system-arm osmo-bts-trx mobile 2>/dev/null
pkill -9 -f bridge.py 2>/dev/null
rm -f "$L1CTL_SOCK" "$MON_SOCK"
sleep 1

# ---- QEMU ----
# L1CTL handled by bridge now, disable QEMU's l1ctl_sock
L1CTL_SOCK="/tmp/osmocom_l2_disabled" \
$QEMU -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor "unix:${MON_SOCK},server,nowait" \
  -kernel "$FW" > /root/qemu.log 2>&1 &
QEMU_PID=$!
# Wait for PTY to appear in log (DSP boot can take a few seconds)
for i in $(seq 1 30); do
  PTY=$(grep -o "/dev/pts/[0-9]*" /root/qemu.log 2>/dev/null | head -1)
  [ -n "$PTY" ] && break
  sleep 1
done
printf "cont\n" | socat - UNIX-CONNECT:"$MON_SOCK" 2>/dev/null
sleep 1
PTY2=$(grep -o "/dev/pts/[0-9]*" /root/qemu.log | tail -1)
ln -sf "$PTY" /tmp/qemu-uart-modem
echo "QEMU PID=$QEMU_PID PTY=$PTY PTY2=$PTY2"
[ -z "$PTY" ] && echo "FATAL: no PTY" && exit 1

# ---- tmux ----
tmux new-session -d -s $SESSION -n qemu
tmux send-keys -t $SESSION:0 "tail -f /root/qemu.log" C-m

# Window 1: bridge (faketrx BTS side on 5700-5702)
tmux new-window -t $SESSION -n bridge
tmux send-keys -t $SESSION:1 "python3 $BRIDGE $PTY" C-m
sleep 1

# Window 2: BTS
tmux new-window -t $SESSION -n bts
tmux send-keys -t $SESSION:2 "sleep 2 && osmo-bts-trx -c $BTS_CFG" C-m

# Window 3: mobile
tmux new-window -t $SESSION -n mobile
tmux send-keys -t $SESSION:3 "sleep 5 && mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

# Window 4: shell
tmux new-window -t $SESSION -n shell

tmux select-window -t $SESSION:1
exec tmux attach -t $SESSION
