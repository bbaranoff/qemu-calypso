#!/bin/bash
# launch_calypso.sh — Start QEMU + osmocon with proper timing
QEMU=./build/qemu-system-arm
LOADER=~/compal_e88/loader.highram.elf
FIRMWARE=${1:-~/compal_e88/layer1.highram.bin}
MONSOCK=/tmp/qemu-calypso-mon.sock
LOGFILE=/tmp/qemu-calypso.log

rm -f "$MONSOCK" "$LOGFILE"
killall -q qemu-system-arm 2>/dev/null
sleep 0.3

echo "=== Starting QEMU Calypso ==="

$QEMU -M calypso \
  -kernel "$LOADER" \
  -serial pty \
  -monitor unix:$MONSOCK,server,nowait \
  -S \
  >"$LOGFILE" 2>&1 &

QEMU_PID=$!

PTS=""
for i in $(seq 1 50); do
    [ -f "$LOGFILE" ] && PTS=$(grep -o '/dev/pts/[0-9]*' "$LOGFILE" | head -1)
    [ -n "$PTS" ] && break
    sleep 0.1
done

if [ -z "$PTS" ]; then
    echo "ERROR: Could not find QEMU pty"
    cat "$LOGFILE" 2>/dev/null
    kill $QEMU_PID 2>/dev/null
    exit 1
fi

echo "=== QEMU pty: $PTS ==="
sleep 0.5

echo "=== Starting osmocon on $PTS ==="
osmocon -p "$PTS" -m c123xor -c "$FIRMWARE" &
OSMO_PID=$!

# Give osmocon time to open pty
sleep 1.5

echo "=== Unpausing QEMU (firmware will handshake with osmocon) ==="
echo "cont" | socat - UNIX-CONNECT:$MONSOCK 2>/dev/null

# NO system_reset — cont is enough, reset would kill uploaded firmware!

echo "=== Running (QEMU=$QEMU_PID, osmocon=$OSMO_PID) ==="
echo "=== L1CTL socket: /tmp/osmocom_l2 ==="
echo "=== Press Ctrl+C to stop ==="

trap "kill $QEMU_PID $OSMO_PID 2>/dev/null; rm -f $MONSOCK; exit" INT TERM
wait $OSMO_PID 2>/dev/null
kill $QEMU_PID 2>/dev/null
rm -f "$MONSOCK"
