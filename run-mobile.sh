#!/bin/bash
# run-mobile.sh — Launch QEMU Calypso + L1CTL bridge + mobile
# No BTS needed. PM handled by bridge (ARFCN 1022 = -62 dBm).
# FBSB will return result=255 (no cell) until a BTS/cfile is connected.
set -euo pipefail

QEMU=./build/qemu-system-arm
FIRMWARE="${1:-/home/nirvana/compal_e88/layer1.highram.elf}"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOG=/tmp/qemu-calypso.log
BRIDGE_LOG=/tmp/qemu-l1ctl-bridge.log
MOBILE_LOG=/tmp/mobile.log

cleanup() {
    echo ""
    echo "=== Stopping ==="
    kill $MOBILE_PID $BRIDGE_PID $QEMU_PID 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

# ---- Cleanup old ----
killall -9 qemu-system-arm mobile 2>/dev/null || true
pkill -9 -f l1ctl_bridge 2>/dev/null || true
sleep 1
rm -f $MONSOCK $LOG $BRIDGE_LOG $MOBILE_LOG /tmp/qemu-modem-tx.raw /tmp/qemu-modem-rx.raw /tmp/osmocom_l2

# ---- QEMU ----
echo "[1] Starting QEMU..."
$QEMU -M calypso -cpu arm946 -serial pty -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -kernel "$FIRMWARE" >$LOG 2>&1 &
QEMU_PID=$!
sleep 3
if ! kill -0 $QEMU_PID 2>/dev/null; then
    echo "QEMU failed:"; tail -5 $LOG; exit 1
fi
MODEM_PTY=$(grep -o '/dev/pts/[0-9]*' $LOG | head -1)
echo "    PID=$QEMU_PID  MODEM=$MODEM_PTY"
printf 'cont\n' | socat - UNIX-CONNECT:$MONSOCK 2>/dev/null

# ---- Wait for firmware boot ----
echo "[2] Waiting for firmware boot..."
sleep 5

# ---- Bridge ----
echo "[3] Starting L1CTL bridge..."
python3 ./l1ctl_bridge.py "$MODEM_PTY" >$BRIDGE_LOG 2>&1 &
BRIDGE_PID=$!
sleep 1
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "Bridge failed:"; cat $BRIDGE_LOG; exit 1
fi
echo "    PID=$BRIDGE_PID"

# ---- Mobile ----
echo "[4] Starting mobile..."
/usr/local/bin/mobile >$MOBILE_LOG 2>&1 &
MOBILE_PID=$!
sleep 2
if ! kill -0 $MOBILE_PID 2>/dev/null; then
    echo "Mobile failed:"; cat $MOBILE_LOG; exit 1
fi
echo "    PID=$MOBILE_PID  telnet 127.0.0.1 4247"

# ---- Watch ----
echo ""
echo "=== Running (Ctrl+C to stop) ==="
echo "    Logs: $LOG | $BRIDGE_LOG | $MOBILE_LOG"
echo ""

while kill -0 $QEMU_PID 2>/dev/null; do
    sleep 5
    FW=$(grep -c "fw→mobile" $BRIDGE_LOG 2>/dev/null) || FW=0
    MOB=$(grep -c "mobile→fw" $BRIDGE_LOG 2>/dev/null) || MOB=0
    SYNC=$(grep -c "SYNC:" $LOG 2>/dev/null) || SYNC=0
    FBSB=$(grep -c "FBSB" $MOBILE_LOG 2>/dev/null) || FBSB=0
    printf "\r  fw→mob=%d  mob→fw=%d  sync=%d  fbsb=%d  " "$FW" "$MOB" "$SYNC" "$FBSB"
done
