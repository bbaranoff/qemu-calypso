#!/bin/bash
# Pipeline: QEMU (layer1 firmware + DSP + BSP)
#           bridge (BTS UDP 5700-5702 ↔ BSP UDP 6702, CLK slave on 6700)
#           mobile → /tmp/osmocom_l2 (direct to QEMU l1ctl_sock)
#
# Sequence matters:
#   1. QEMU starts → TDMA ticks begin → CLK UDP to bridge on 6700
#   2. Bridge starts → receives ticks → forwards CLK IND to BTS
#   3. Wait for bridge to confirm ticks flowing
#   4. BTS starts → receives CLK IND → sends POWERON → sends bursts
#   5. Mobile starts → connects to QEMU L1CTL → begins cell search

SESSION="calypso"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
L1CTL_SOCK="/tmp/osmocom_l2"

# Cleanup
tmux kill-session -t $SESSION 2>/dev/null
killall -9 qemu-system-arm osmo-bts-trx mobile 2>/dev/null
pkill -9 -f bridge.py 2>/dev/null
rm -f "$L1CTL_SOCK" "$MON_SOCK" /tmp/osmocom_l2_*
sleep 1

/etc/osmocom/status.sh stop 2>/dev/null
/etc/osmocom/osmo-start.sh 2>/dev/null

# ---- tmux ----
tmux new-session -d -s $SESSION -n qemu

# ---- 1. QEMU ----
CALYPSO_DSP_ROM=/opt/GSM/calypso_dsp.txt \
$QEMU -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor "unix:${MON_SOCK},server,nowait" \
  -kernel "$FW" > /root/qemu.log 2>&1 &
QEMU_PID=$!
tmux send-keys -t $SESSION:qemu "tail -f /root/qemu.log" C-m

echo -n "Waiting for QEMU L1CTL socket..."
for i in $(seq 1 30); do
  [ -S "$L1CTL_SOCK" ] && break
  sleep 1; echo -n "."
done
echo " OK (PID=$QEMU_PID)"

# ---- 2. Bridge (receives CLK ticks from QEMU on UDP 6700) ----
tmux new-window -t $SESSION -n bridge
tmux send-keys -t $SESSION:bridge "python3 $BRIDGE 2>&1 | tee /tmp/bridge.log" C-m

echo -n "Waiting for bridge to receive QEMU ticks..."
for i in $(seq 1 30); do
  grep -q "QEMU tick #1" /tmp/bridge.log 2>/dev/null && break
  sleep 1; echo -n "."
done
if grep -q "QEMU tick" /tmp/bridge.log 2>/dev/null; then
  echo " OK (ticks flowing)"
else
  echo " TIMEOUT (bridge not receiving ticks — check CLK UDP 6700)"
fi

# ---- 3. BTS (needs CLK IND from bridge before it times out) ----
tmux new-window -t $SESSION -n bts
tmux send-keys -t $SESSION:bts "osmo-bts-trx -c $BTS_CFG" C-m
sleep 2

# ---- 4. Mobile (connects directly to QEMU l1ctl_sock) ----
tmux new-window -t $SESSION -n mobile
tmux send-keys -t $SESSION:mobile "sleep 3 && mobile -c /root/.osmocom/bb/mobile_group1.cfg" C-m

# ---- Shell ----
tmux new-window -t $SESSION -n shell

tmux select-window -t $SESSION:qemu
exec tmux attach -t $SESSION
