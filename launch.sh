#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
LOADER="/home/nirvana/compal_e88/loader.compalram.elf"
FIRMWARE="/home/nirvana/compal_e88/hello_world.compalram.bin"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOGFILE=/tmp/qemu-calypso.log
RELAY=./sercomm_relay.py

LOAD_ADDR=0x00820000

QEMU_PID=""
RELAY_PID=""

cleanup() {
    local ec=$?
    [ -n "${RELAY_PID:-}" ] && kill "$RELAY_PID" 2>/dev/null || true
    [ -n "${QEMU_PID:-}" ] && kill "$QEMU_PID" 2>/dev/null || true
    rm -f "$MONSOCK" /tmp/osmocom_loader
    exit "$ec"
}
trap cleanup INT TERM EXIT

rm -f "$MONSOCK" "$LOGFILE" /tmp/osmocom_loader
killall -q qemu-system-arm 2>/dev/null || true
sleep 0.3

echo "=== Starting QEMU Calypso (DEBUG) ==="

# stderr goes to logfile too so we see [MB]/[SOC] debug
"$QEMU" \
  -M calypso \
  -cpu arm946 \
  -kernel "$LOADER" \
  -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -S \
  >"$LOGFILE" 2>&1 &

QEMU_PID=$!

# ---- Wait for PTY ----
PTS=""
for i in $(seq 1 200); do
    if [ -f "$LOGFILE" ]; then
        PTS="$(grep -o '/dev/pts/[0-9]*' "$LOGFILE" | head -1 || true)"
        if [ -n "${PTS:-}" ] && [ -e "$PTS" ]; then
            break
        fi
    fi
    sleep 0.1
done

if [ -z "${PTS:-}" ] || [ ! -e "$PTS" ]; then
    echo "ERROR: Could not find QEMU pty"
    echo "=== QEMU log ==="
    cat "$LOGFILE" 2>/dev/null || true
    exit 1
fi

echo "=== QEMU pty: $PTS ==="
stty -F "$PTS" raw -echo 2>/dev/null || true

# ---- Show boot debug from stderr ----
echo ""
echo "=== QEMU boot debug ==="
grep -E "^\[MB\]|\[SOC\]|Flash:|pflash" "$LOGFILE" 2>/dev/null || echo "(no debug lines yet)"
echo ""

# ---- Dump memory map ----
echo "=== Memory map (info mtree) ==="
sleep 0.5
printf 'info mtree -f\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null | head -60 || true
echo ""

# ---- Check what's at 0x00000000 ----
echo "=== Checking 0x00000000 region ==="
printf 'info mtree -f\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null | grep -iE "0000000|flash|pflash|low300|alias" || true
echo ""

# ---- Unpause so loader boots ----
echo "=== Unpausing QEMU (loader init) ==="
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null || true
sleep 2

# ---- Show any new debug from loader running ----
echo "=== Post-boot debug ==="
grep -E "^\[MB\]|\[SOC\]|flash|Flash" "$LOGFILE" 2>/dev/null || true
echo ""

# ---- Start sercomm relay ----
echo "=== Starting sercomm relay ==="
python3 "$RELAY" "$PTS" &
RELAY_PID=$!
sleep 0.5

# ---- Ping / load / jump via osmoload ----
echo "=== Pinging loader ==="
osmoload ping

echo "=== Loading firmware at $LOAD_ADDR ==="
#osmoload memload "$LOAD_ADDR" "$FIRMWARE"

echo "=== Jumping to $LOAD_ADDR ==="
#osmoload jump "$LOAD_ADDR"

# ---- Kill relay, firmware now owns the UART ----
kill "$RELAY_PID" 2>/dev/null || true
RELAY_PID=""

echo "=== Running (QEMU=$QEMU_PID) ==="
echo "=== L1CTL socket: /tmp/osmocom_l2 ==="
echo "=== Serial PTY: $PTS ==="
echo "=== Press Ctrl+C to stop ==="

while kill -0 "$QEMU_PID" 2>/dev/null; do
    sleep 1
done

echo "=== QEMU exited ==="
