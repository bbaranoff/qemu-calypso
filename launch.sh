#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
LOADER="/home/nirvana/compal_e88/loader.compalram.elf"
FIRMWARE="/home/nirvana/compal_e88/hello_world.compalram.elf"
RELAY=./sercomm_relay.py

MONSOCK=/tmp/qemu-calypso-mon.sock
QEMU_LOG=/tmp/qemu-calypso.log
RELAY_LOG=/tmp/qemu-sercomm-relay.log
MODEM_UART_LOG=/tmp/qemu-modem-uart.log
IRDA_UART_LOG=/tmp/qemu-irda-uart.log
PTY_FILE=/tmp/qemu-calypso-ptys.txt

LOAD_ADDR=0x00800000
JUMP_ADDR=0x00800104

QEMU_PID=""
RELAY_PID=""
MODEM_CAT_PID=""
IRDA_CAT_PID=""

cleanup() {
    local ec=$?

    [ -n "${MODEM_CAT_PID:-}" ] && kill "$MODEM_CAT_PID" 2>/dev/null || true
    [ -n "${IRDA_CAT_PID:-}" ]  && kill "$IRDA_CAT_PID" 2>/dev/null || true
    [ -n "${RELAY_PID:-}" ]     && kill "$RELAY_PID" 2>/dev/null || true
    [ -n "${QEMU_PID:-}" ]      && kill "$QEMU_PID" 2>/dev/null || true

    wait "$MODEM_CAT_PID" 2>/dev/null || true
    wait "$IRDA_CAT_PID" 2>/dev/null || true
    wait "$RELAY_PID" 2>/dev/null || true
    wait "$QEMU_PID" 2>/dev/null || true

    rm -f "$MONSOCK" /tmp/osmocom_loader "$PTY_FILE"
    exit "$ec"
}
trap cleanup INT TERM EXIT

rm -f \
    "$MONSOCK" \
    "$QEMU_LOG" \
    "$RELAY_LOG" \
    "$MODEM_UART_LOG" \
    "$IRDA_UART_LOG" \
    "$PTY_FILE" \
    /tmp/osmocom_loader

killall -q qemu-system-arm 2>/dev/null || true
sleep 0.3

echo "=== Starting QEMU Calypso (DEBUG) ==="

"$QEMU" \
  -M calypso \
  -cpu arm946 \
  -kernel "$LOADER" \
  -serial pty \
  -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -S \
  >"$QEMU_LOG" 2>&1 &

QEMU_PID=$!

MODEM_PTY=""
IRDA_PTY=""

for i in $(seq 1 300); do
    if [ -f "$QEMU_LOG" ]; then
        mapfile -t PTYS < <(grep -o '/dev/pts/[0-9]*' "$QEMU_LOG" | awk '!seen[$0]++')
        if [ "${#PTYS[@]}" -ge 2 ]; then
            MODEM_PTY="${PTYS[0]}"
            IRDA_PTY="${PTYS[1]}"
            if [ -e "$MODEM_PTY" ] && [ -e "$IRDA_PTY" ]; then
                break
            fi
        fi
    fi
    sleep 0.1
done

if [ -z "${MODEM_PTY:-}" ] || [ -z "${IRDA_PTY:-}" ]; then
    echo "ERROR: Could not find both QEMU PTYs"
    echo "=== QEMU log ==="
    cat "$QEMU_LOG" 2>/dev/null || true
    exit 1
fi

{
    echo "MODEM_PTY=$MODEM_PTY"
    echo "IRDA_PTY=$IRDA_PTY"
} > "$PTY_FILE"

echo "=== MODEM PTY: $MODEM_PTY ==="
echo "=== IRDA  PTY: $IRDA_PTY ==="
echo "=== QEMU log:  $QEMU_LOG ==="
echo "=== Relay log: $RELAY_LOG ==="
echo "=== MODEM log: $MODEM_UART_LOG ==="
echo "=== IRDA log:  $IRDA_UART_LOG ==="

stty -F "$MODEM_PTY" raw -echo 2>/dev/null || true
stty -F "$IRDA_PTY"  raw -echo 2>/dev/null || true

echo
echo "=== QEMU boot debug ==="
grep -E "^\[MB\]|\[SOC\]|\[UART:|UART PATCH ACTIVE|Flash:|pflash" "$QEMU_LOG" 2>/dev/null || echo "(no debug lines yet)"
echo

echo "=== Memory map (info mtree) ==="
sleep 0.5
printf 'info mtree -f\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null | head -60 || true
echo

echo "=== Checking 0x00000000 region ==="
printf 'info mtree -f\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null | grep -iE "0000000|flash|pflash|low300|alias" || true
echo

echo "=== Unpausing QEMU (loader init) ==="
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null || true
sleep 2

echo "=== Post-boot debug ==="
grep -E "^\[MB\]|\[SOC\]|\[UART:|UART PATCH ACTIVE|flash|Flash" "$QEMU_LOG" 2>/dev/null || true
echo

echo "=== Starting sercomm relay on MODEM PTY ==="
python3 "$RELAY" "$MODEM_PTY" 2>&1 | tee "$RELAY_LOG" &
RELAY_PID=$!
sleep 0.5

echo "=== Pinging loader ==="
osmoload ping

echo "=== Loading firmware at $LOAD_ADDR ==="
osmoload memload "$LOAD_ADDR" "$FIRMWARE"

echo "=== Jumping to firmware entry $JUMP_ADDR ==="
osmoload jump "$JUMP_ADDR"

sleep 0.5
echo "=== Registers after jump ==="
printf 'info registers\n' | socat - UNIX-CONNECT:"$MONSOCK"

echo "=== Registers after 1s ==="
sleep 1
printf 'info registers\n' | socat - UNIX-CONNECT:"$MONSOCK"

echo "=== Stopping sercomm relay (firmware now owns UART) ==="
kill "$RELAY_PID" 2>/dev/null || true
wait "$RELAY_PID" 2>/dev/null || true
RELAY_PID=""

sleep 0.2
stty -F "$MODEM_PTY" 115200 raw -echo -icrnl -inlcr -ixon -ixoff 2>/dev/null || true
stty -F "$IRDA_PTY"  115200 raw -echo -icrnl -inlcr -ixon -ixoff 2>/dev/null || true

echo
echo "=== Capturing firmware UARTs ==="
echo "=== MODEM -> $MODEM_UART_LOG ==="
echo "=== IRDA  -> $IRDA_UART_LOG ==="
echo "=== Press Ctrl+C to stop ==="
echo

(
    stdbuf -o0 cat -v "$MODEM_PTY" | tee "$MODEM_UART_LOG"
) &
MODEM_CAT_PID=$!

(
    stdbuf -o0 cat -v "$IRDA_PTY" | tee "$IRDA_UART_LOG"
) &
IRDA_CAT_PID=$!

wait "$QEMU_PID"
