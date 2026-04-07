#!/bin/bash
# run_tmp.sh — Calypso QEMU pipeline, but TRXD bursts come from a cfile
# played by inject_cfile.py instead of osmo-bts-trx + bridge TRXD relay.
#
#   QEMU (-serial pty) ← bridge.py → PTY (sercomm L1CTL DLCI 5)
#                      ← bridge.py → /tmp/osmocom_l2_1 (mobile)
#   inject_cfile.py → UDP 6702 → QEMU sercomm_gate → BSP DMA → DSP DARAM
#
# osmo-bts-trx is NOT started here. bridge.py still runs because it owns
# the L1CTL unix socket; the BTS-side TRXD path is just left idle.
set -uo pipefail

SESSION="calypso"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
INJECT="/opt/GSM/qemu-src/inject_cfile.py"
CFILE="/root/out_arfcn_100.cfile"
CFILE_RATE="${CFILE_RATE:-1024000}"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"
L1CTL_SOCK="/tmp/osmocom_l2_1"
LOG_DIR="/root/logs"
mkdir -p "$LOG_DIR"
# Auto-increment: find next free run number across all logs
RUN_N=1
while [ -e "$LOG_DIR/qemu_$(printf %03d $RUN_N).log" ]; do
    RUN_N=$((RUN_N+1))
done
RUN_TAG=$(printf %03d $RUN_N)
QEMU_LOG="$LOG_DIR/qemu_$RUN_TAG.log"
BRIDGE_LOG="$LOG_DIR/bridge_$RUN_TAG.log"
INJECT_LOG="$LOG_DIR/inject_$RUN_TAG.log"
MOBILE_LOG="$LOG_DIR/mobile_$RUN_TAG.log"
HACK_LOG="$LOG_DIR/hack_$RUN_TAG.log"
# Symlink /tmp/{qemu,bridge,inject,mobile,hack}.log → current run
ln -sf "$QEMU_LOG"   /tmp/qemu.log
ln -sf "$BRIDGE_LOG" /tmp/bridge.log
ln -sf "$INJECT_LOG" /tmp/inject.log
ln -sf "$MOBILE_LOG" /tmp/mobile.log
ln -sf "$HACK_LOG"   /tmp/hack.log
# Kill QEMU when log exceeds this many bytes (default 50 MB)
QEMU_LOG_MAX="${QEMU_LOG_MAX:-$((50*1024*1024))}"
VENV="/root/.env"

if [ -f "$VENV/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
fi

# ── Cleanup ──
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -9 -x qemu-system-arm  2>/dev/null || true
pkill -9 -f 'bridge\.py'     2>/dev/null || true
pkill -9 -f 'inject_cfile'   2>/dev/null || true
pkill -9 -x osmo-bts-trx     2>/dev/null || true
pkill -9 -x mobile           2>/dev/null || true
systemctl stop osmo-bts-trx  2>/dev/null || true

# Wait for processes to actually die before launching new QEMU
for i in 1 2 3 4 5 6 7 8 9 10; do
    if ! pgrep -x qemu-system-arm >/dev/null 2>&1 && 
       ! pgrep -f 'bridge\.py' >/dev/null 2>&1 && 
       ! pgrep -f 'inject_cfile' >/dev/null 2>&1 && 
       ! pgrep -x mobile >/dev/null 2>&1; then
        break
    fi
    sleep 0.3
done
rm -f /tmp/osmocom_l2_*
sleep 1
rm -f "$L1CTL_SOCK"
echo "[run_debug] run #$RUN_TAG → $QEMU_LOG (kill at ${QEMU_LOG_MAX} bytes)"
read -p "Press ENTER to start cfile-injection run... " _
sleep 1

# ── Sanity ──
[ -x "$QEMU" ]      || { echo "FATAL: $QEMU not found";   exit 1; }
[ -f "$FW" ]        || { echo "FATAL: $FW not found";     exit 1; }
[ -f "$BRIDGE" ]    || { echo "FATAL: $BRIDGE not found"; exit 1; }
[ -f "$INJECT" ]    || { echo "FATAL: $INJECT not found"; exit 1; }
[ -f "$CFILE" ]     || { echo "FATAL: $CFILE not found";  exit 1; }
[ -f "$MOBILE_CFG" ]|| echo "WARN: $MOBILE_CFG not found"

# ── QEMU ──
#CALYPSO_BSP_DARAM_ADDR=0x021f CALYPSO_BSP_DARAM_LEN=512 CALYPSO_BSP_BYPASS_BDLENA=1
CALYPSO_BSP_BYPASS_BDLENA=1 CALYPSO_BSP_DARAM_ADDR=0x021f CALYPSO_BSP_DARAM_LEN=512 CALYPSO_DBG=all $QEMU -M calypso -S -gdb tcp::1234 \
  -kernel "$FW" \
  -serial pty \
  -display none \
  > "$QEMU_LOG" 2>&1 &
QEMU_PID=$!

PTY=""
for i in $(seq 1 20); do
    sleep 0.25
    PTY=$(grep -o "/dev/pts/[0-9]*" "$QEMU_LOG" 2>/dev/null | head -1)
    [ -n "$PTY" ] && break
done
if [ -z "$PTY" ]; then
    echo "FATAL: QEMU did not allocate a PTY"
    cat "$QEMU_LOG"
    kill -9 "$QEMU_PID" 2>/dev/null
    exit 1
fi
echo "QEMU PID=$QEMU_PID PTY=$PTY"

# ── Bridge (still needed for L1CTL unix socket; TRXD relay sits idle) ──
python3 "$BRIDGE" "$PTY" --sock "$L1CTL_SOCK" \
        > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

echo -n "Waiting for L1CTL socket..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 0.25; echo -n "."
done
[ -S "$L1CTL_SOCK" ] && echo " OK" || { echo " TIMEOUT"; cat "$BRIDGE_LOG"; }

# Give the gate a moment to bind UDP 6702 before injection starts.
sleep 1

# ── tmux ──
ACT="source $VENV/bin/activate"

tmux new-session -d -s "$SESSION" -n qemu
tmux send-keys -t "$SESSION:qemu"  "$ACT && tail -F $QEMU_LOG" C-m

tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" "$ACT && tail -F $BRIDGE_LOG" C-m

tmux new-window -t "$SESSION" -n inject
tmux send-keys -t "$SESSION:inject" \
    "$ACT && python3 $INJECT $CFILE --rate $CFILE_RATE --loop 2>&1 | tee $INJECT_LOG" C-m
#tmux send-keys -t "$SESSION:inject" \
#    "$ACT && python3 $INJECT --tune --loop 2>&1 | tee $INJECT_LOG" C-m

tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" "$ACT && sleep 5 && mobile -c $MOBILE_CFG 2>&1 | tee $MOBILE_LOG" C-m

tmux new-window -t "$SESSION" -n gdb
tmux send-keys -t "$SESSION:gdb" "$ACT && sleep 2 && python3 -u /opt/GSM/qemu-src/hack_gdb.py" C-m

tmux new-window -t "$SESSION" -n shell
tmux send-keys -t "$SESSION:shell" "$ACT" C-m

tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"
