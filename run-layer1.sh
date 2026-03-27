#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
FIRMWARE="/home/nirvana/compal_e88/layer1.highram.elf"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOG=/tmp/qemu-calypso.log

# Cleanup
pkill -f "qemu-system-arm.*calypso" 2>/dev/null || true
sleep 0.5
rm -f "$MONSOCK" "$LOG"

echo "=== Starting QEMU Calypso with Layer1 ==="
"$QEMU" \
  -M calypso \
  -cpu arm946 \
  -serial pty \
  -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -kernel "$FIRMWARE" \
  >"$LOG" 2>&1 &
QEMU_PID=$!
echo "QEMU PID: $QEMU_PID"

sleep 3

# Get PTYs
MODEM_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | head -1)
IRDA_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | tail -1)
echo "MODEM: $MODEM_PTY"
echo "IRDA:  $IRDA_PTY"

# Unpause
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null
echo "=== Running... (Ctrl+C to stop) ==="

# Tail the TX output decoded
tail -f "$LOG" | grep --line-buffered "TX>>>" | while IFS= read -r line; do
    ch=$(echo "$line" | sed "s/.*'\(.\)'/\1/")
    printf '%s' "$ch"
done &
TAIL_PID=$!

trap "kill $TAIL_PID $QEMU_PID 2>/dev/null; wait $TAIL_PID $QEMU_PID 2>/dev/null; echo; echo '=== Stopped ==='" INT TERM EXIT

wait "$QEMU_PID" 2>/dev/null || true
