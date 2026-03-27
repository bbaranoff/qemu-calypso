#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
FIRMWARE="/home/nirvana/compal_e88/layer1.highram.elf"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOG=/tmp/qemu-calypso.log
L1CTL_BRIDGE=./l1ctl_bridge.py
MOBILE="/usr/local/bin/mobile"

# Cleanup
pkill -9 -f "qemu-system-arm" 2>/dev/null || true
pkill -9 -f "l1ctl_bridge" 2>/dev/null || true
pkill -9 -f "mobile$" 2>/dev/null || true
sleep 1
rm -f "$MONSOCK" "$LOG" /tmp/qemu-modem-tx.raw /tmp/qemu-modem-rx.raw /tmp/osmocom_l2

echo "=== Starting QEMU ==="
"$QEMU" \
  -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -kernel "$FIRMWARE" \
  >"$LOG" 2>&1 &
QEMU_PID=$!
echo "  PID: $QEMU_PID"
sleep 3

MODEM_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | head -1)
IRDA_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | tail -1)
echo "  MODEM: $MODEM_PTY"
echo "  IRDA:  $IRDA_PTY"

echo "=== Unpausing ==="
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null
sleep 8
echo "  Firmware settled"

echo "=== Starting L1CTL bridge ==="
python3 "$L1CTL_BRIDGE" "$MODEM_PTY" > /tmp/qemu-l1ctl-bridge.log 2>&1 &
L1CTL_PID=$!
sleep 1

echo "=== Starting mobile ==="
"$MOBILE" > /tmp/mobile.log 2>&1 &
MOBILE_PID=$!
sleep 1

echo "=== Waiting 20 seconds for L1CTL exchange ==="
for i in $(seq 1 20); do
    sleep 1
    # Check if bridge received something from firmware
    if grep -q "fw→mobile" /tmp/qemu-l1ctl-bridge.log 2>/dev/null; then
        echo "  ✓ Firmware responded at ${i}s!"
        break
    fi
    printf "."
done
echo ""

echo ""
echo "===================="
echo "=== Bridge log ==="
cat /tmp/qemu-l1ctl-bridge.log
echo ""
echo "=== Mobile log ==="
tail -10 /tmp/mobile.log 2>/dev/null
echo ""
echo "=== DLCI 4 frames ==="
python3 -c "
import os
if not os.path.exists('/tmp/qemu-modem-tx.raw'):
    print('no TX file'); exit()
data = open('/tmp/qemu-modem-tx.raw','rb').read()
count = data.count(bytes([0x7e, 0x04, 0x03]))
print(f'DLCI 4: {count}, TX: {len(data)} bytes')
" 2>/dev/null
echo ""
echo "=== Console messages ==="
python3 -c "
import os
if not os.path.exists('/tmp/qemu-modem-tx.raw'):
    exit()
data = open('/tmp/qemu-modem-tx.raw','rb').read()
i = 0
while i < len(data):
    if data[i] == 0x7e and i+1 < len(data) and data[i+1] != 0x7e:
        j = i+1
        while j < len(data) and data[j] != 0x7e: j += 1
        if j < len(data):
            frame = data[i+1:j]
            unesc = bytearray()
            k = 0
            while k < len(frame):
                if frame[k] == 0x7d and k+1 < len(frame):
                    unesc.append(frame[k+1] ^ 0x20); k += 2
                else: unesc.append(frame[k]); k += 1
            if len(unesc) >= 2:
                dlci = unesc[0]
                if dlci == 10:
                    text = bytes(unesc[2:]).decode('ascii', errors='replace').rstrip()
                    if text: print(text)
                elif dlci == 4:
                    print(f'[L1CTL] {bytes(unesc[2:]).hex()}')
            i = j + 1
        else: i += 1
    else: i += 1
" 2>/dev/null

# Cleanup
echo ""
echo "=== Cleaning up ==="
kill $MOBILE_PID 2>/dev/null || true
kill $L1CTL_PID 2>/dev/null || true
kill $QEMU_PID 2>/dev/null || true
wait 2>/dev/null
echo "Done"
