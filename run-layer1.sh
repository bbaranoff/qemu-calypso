#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
FIRMWARE="${1:-/home/nirvana/compal_e88/trx.highram.elf}"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOG=/tmp/qemu-calypso.log
TX_RAW=/tmp/qemu-modem-tx.raw

# ---- Colors ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
DIM='\033[2m'
BOLD='\033[1m'
NC='\033[0m'

# ---- Cleanup ----
pkill -f "qemu-system-arm.*calypso" 2>/dev/null || true
sleep 0.5
rm -f "$MONSOCK" "$LOG" "$TX_RAW" /tmp/qemu-modem-rx.raw

echo -e "${BOLD}=== QEMU Calypso — Layer1 ===${NC}"
echo ""

# ---- Check firmware ----
if [ ! -f "$FIRMWARE" ]; then
    echo -e "${RED}ERROR: firmware not found: $FIRMWARE${NC}"
    exit 1
fi
echo -e "${DIM}Firmware: $FIRMWARE ($(stat -c%s "$FIRMWARE") bytes)${NC}"

# ---- Start QEMU ----
echo -e "${CYAN}[1/3]${NC} Starting QEMU..."
"$QEMU" \
  -M calypso \
  -cpu arm946 \
  -serial pty \
  -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -kernel "$FIRMWARE" \
  >"$LOG" 2>&1 &
QEMU_PID=$!

sleep 3
if ! kill -0 "$QEMU_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: QEMU failed to start${NC}"
    cat "$LOG" 2>/dev/null
    exit 1
fi

MODEM_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | head -1)
IRDA_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | tail -1)
echo -e "       PID:   ${GREEN}$QEMU_PID${NC}"
echo -e "       MODEM: ${GREEN}$MODEM_PTY${NC}"
echo -e "       IRDA:  ${GREEN}$IRDA_PTY${NC}"

# ---- Unpause ----
echo -e "${CYAN}[2/3]${NC} Unpausing CPU..."
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null

# ---- Wait for firmware boot ----
echo -e "${CYAN}[3/3]${NC} Waiting for firmware boot..."
BOOT_OK=false
for i in $(seq 1 40); do
    sleep 0.25
    if [ -f "$TX_RAW" ] && python3 -c "
import sys
data = open('$TX_RAW','rb').read()
sys.exit(0 if b'Power up simcard' in data else 1)
" 2>/dev/null; then
        BOOT_OK=true
        break
    fi
done

if $BOOT_OK; then
    echo -e "       ${GREEN}Firmware booted${NC}"
else
    echo -e "       ${YELLOW}Firmware still booting (no 'Power up simcard' yet)${NC}"
fi

# ---- Extract & display console ----
echo ""
echo -e "${BOLD}=== Firmware Console ===${NC}"
python3 -c "
import os, sys
if not os.path.exists('$TX_RAW'):
    print('  (no output yet)'); sys.exit()
data = open('$TX_RAW','rb').read()
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
            if len(unesc) >= 2 and unesc[0] == 10:
                text = bytes(unesc[2:]).decode('ascii', errors='replace').rstrip()
                if text: print('  ' + text)
            i = j + 1
        else: i += 1
    else: i += 1
" 2>/dev/null

# ---- Live tail ----
echo ""
echo -e "${BOLD}=== Live console (Ctrl+C to stop) ===${NC}"
echo -e "${DIM}  Streaming sercomm console from $TX_RAW...${NC}"
echo ""

# Watch TX raw file for new console messages
python3 -u -c "
import os, sys, time

path = '$TX_RAW'
pos = os.path.getsize(path) if os.path.exists(path) else 0
buf = bytearray()

while True:
    try:
        time.sleep(0.2)
        if not os.path.exists(path):
            continue
        sz = os.path.getsize(path)
        if sz <= pos:
            continue
        with open(path, 'rb') as f:
            f.seek(pos)
            new = f.read()
            pos = f.tell()
        buf.extend(new)

        # Parse sercomm frames
        while True:
            try:
                start = buf.index(0x7e)
            except ValueError:
                buf.clear(); break
            if start > 0:
                del buf[:start]
            try:
                end = buf.index(0x7e, 1)
            except ValueError:
                break
            raw = bytes(buf[1:end])
            del buf[:end+1]
            if not raw:
                continue
            # unescape
            unesc = bytearray()
            k = 0
            while k < len(raw):
                if raw[k] == 0x7d and k+1 < len(raw):
                    unesc.append(raw[k+1] ^ 0x20); k += 2
                else: unesc.append(raw[k]); k += 1
            if len(unesc) < 2:
                continue
            dlci = unesc[0]
            payload = bytes(unesc[2:])
            if dlci == 10:
                text = payload.decode('ascii', errors='replace').rstrip()
                if text:
                    sys.stdout.write(text + '\n')
                    sys.stdout.flush()
            elif dlci == 4:
                sys.stdout.write(f'[L1CTL] {payload.hex()}\n')
                sys.stdout.flush()
    except KeyboardInterrupt:
        break
" &
TAIL_PID=$!

trap "kill $TAIL_PID $QEMU_PID 2>/dev/null; wait $TAIL_PID $QEMU_PID 2>/dev/null; echo; echo -e '${BOLD}=== Stopped ===${NC}'" INT TERM EXIT

wait "$QEMU_PID" 2>/dev/null || true
