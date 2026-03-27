#!/bin/bash
set -euo pipefail

QEMU=./build/qemu-system-arm
FIRMWARE="/home/nirvana/compal_e88/layer1.highram.elf"
MONSOCK=/tmp/qemu-calypso-mon.sock
LOG=/tmp/qemu-calypso.log
TX_RAW=/tmp/qemu-modem-tx.raw
L1CTL_BRIDGE=./l1ctl_bridge.py
L1CTL_LOG=/tmp/qemu-l1ctl-bridge.log
MOBILE="/usr/local/bin/mobile"
MOBILE_LOG=/tmp/mobile.log

# ---- Colors ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
DIM='\033[2m'
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${CYAN}[*]${NC} $1"; }
ok()    { echo -e "${GREEN}[+]${NC} $1"; }
warn()  { echo -e "${YELLOW}[!]${NC} $1"; }
fail()  { echo -e "${RED}[-]${NC} $1"; }
header(){ echo -e "\n${BOLD}=== $1 ===${NC}"; }

# ---- Sercomm console decoder (reused) ----
decode_console() {
    python3 -c "
import os, sys
path = '$TX_RAW'
if not os.path.exists(path):
    sys.exit()
data = open(path,'rb').read()
i = 0
l1ctl_count = 0
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
                    if text: print('  ' + text)
                elif dlci == 4:
                    l1ctl_count += 1
                    payload = bytes(unesc[2:])
                    msg_type = payload[0] if payload else 0
                    types = {0x07:'RESET_IND',0x0d:'RESET_REQ',0x0e:'RESET_CONF',
                             0x03:'FBSB_REQ',0x04:'FBSB_CONF',0x09:'CCCH_MODE_REQ'}
                    name = types.get(msg_type, f'0x{msg_type:02x}')
                    print(f'  [L1CTL] {name} ({len(payload)} bytes) {payload[:16].hex()}')
            i = j + 1
        else: i += 1
    else: i += 1
print(f'\n  --- TX total: {len(data)} bytes, L1CTL frames: {l1ctl_count} ---')
" 2>/dev/null
}

# ==============================================================
header "QEMU Calypso — Mobile Test"
echo ""

# ---- Pre-checks ----
info "Checking dependencies..."
missing=""
[ -x "$QEMU" ]         || missing="$missing qemu($QEMU)"
[ -f "$FIRMWARE" ]      || missing="$missing firmware($FIRMWARE)"
[ -f "$L1CTL_BRIDGE" ]  || missing="$missing l1ctl_bridge($L1CTL_BRIDGE)"
[ -x "$MOBILE" ]        || missing="$missing mobile($MOBILE)"
command -v socat >/dev/null || missing="$missing socat"
command -v python3 >/dev/null || missing="$missing python3"

if [ -n "$missing" ]; then
    fail "Missing:$missing"
    exit 1
fi
ok "All dependencies found"

# ---- Cleanup ----
info "Cleaning up old processes..."
pkill -9 -f "qemu-system-arm" 2>/dev/null || true
pkill -9 -f "l1ctl_bridge" 2>/dev/null || true
pkill -9 -f "mobile$" 2>/dev/null || true
sleep 1
rm -f "$MONSOCK" "$LOG" "$TX_RAW" /tmp/qemu-modem-rx.raw /tmp/osmocom_l2 "$L1CTL_LOG" "$MOBILE_LOG"

# ==============================================================
header "1. QEMU"
# ==============================================================

info "Starting QEMU..."
"$QEMU" \
  -M calypso -cpu arm946 \
  -serial pty -serial pty \
  -monitor "unix:$MONSOCK,server,nowait" \
  -kernel "$FIRMWARE" \
  >"$LOG" 2>&1 &
QEMU_PID=$!
sleep 3

if ! kill -0 "$QEMU_PID" 2>/dev/null; then
    fail "QEMU failed to start"
    echo -e "${DIM}$(tail -5 "$LOG" 2>/dev/null)${NC}"
    exit 1
fi

MODEM_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | head -1)
IRDA_PTY=$(grep -o '/dev/pts/[0-9]*' "$LOG" | tail -1)
ok "QEMU running (PID $QEMU_PID)"
echo -e "    MODEM: ${GREEN}$MODEM_PTY${NC}   IRDA: ${GREEN}$IRDA_PTY${NC}"

info "Unpausing CPU..."
printf 'cont\n' | socat - UNIX-CONNECT:"$MONSOCK" 2>/dev/null || true

info "Waiting for firmware boot..."
BOOT_OK=false
for i in $(seq 1 60); do
    sleep 0.25
    if ! kill -0 "$QEMU_PID" 2>/dev/null; then
        fail "QEMU died during boot"
        exit 1
    fi
    if [ -f "$TX_RAW" ] && python3 -c "
import sys; data = open('$TX_RAW','rb').read()
sys.exit(0 if b'Power up simcard' in data else 1)" 2>/dev/null; then
        BOOT_OK=true
        break
    fi
done

if $BOOT_OK; then
    ok "Firmware booted ($(printf '%.1f' "$(echo "$i * 0.25" | bc)")s)"
else
    warn "Boot timeout (firmware may still be running)"
fi

# Show boot console
echo -e "${DIM}"
decode_console
echo -e "${NC}"

# ==============================================================
header "2. L1CTL Bridge"
# ==============================================================

info "Starting l1ctl_bridge on $MODEM_PTY..."
python3 "$L1CTL_BRIDGE" "$MODEM_PTY" > "$L1CTL_LOG" 2>&1 &
L1CTL_PID=$!
sleep 1

if kill -0 "$L1CTL_PID" 2>/dev/null; then
    ok "Bridge running (PID $L1CTL_PID)"
    if [ -S /tmp/osmocom_l2 ]; then
        ok "/tmp/osmocom_l2 socket ready"
    else
        warn "/tmp/osmocom_l2 not found yet"
    fi
else
    fail "Bridge failed to start"
    echo -e "${DIM}$(cat "$L1CTL_LOG" 2>/dev/null)${NC}"
    kill "$QEMU_PID" 2>/dev/null || true
    exit 1
fi

# ==============================================================
header "3. Mobile (osmocom-bb)"
# ==============================================================

info "Starting mobile..."
"$MOBILE" > "$MOBILE_LOG" 2>&1 &
MOBILE_PID=$!
sleep 2

if kill -0 "$MOBILE_PID" 2>/dev/null; then
    ok "Mobile running (PID $MOBILE_PID)"
else
    fail "Mobile failed to start"
    echo -e "${DIM}$(cat "$MOBILE_LOG" 2>/dev/null)${NC}"
    kill "$L1CTL_PID" "$QEMU_PID" 2>/dev/null || true
    exit 1
fi

# ==============================================================
header "4. Waiting for L1CTL exchange"
# ==============================================================

info "Mobile sent L1CTL_RESET_REQ, waiting for firmware response..."
L1CTL_OK=false
for i in $(seq 1 30); do
    sleep 1

    # Check processes alive
    if ! kill -0 "$QEMU_PID" 2>/dev/null; then
        fail "QEMU died at ${i}s"
        break
    fi

    # Check for L1CTL response from firmware
    if grep -q "fw→mobile" "$L1CTL_LOG" 2>/dev/null; then
        L1CTL_OK=true
        ok "Firmware responded with L1CTL at ${i}s!"
        break
    fi

    # Check for RBR reads (firmware processing RX data)
    RBR_COUNT=$(grep -c "RBR<<<" "$LOG" 2>/dev/null || echo "0")
    if [ "$i" -eq 5 ] && [ "$RBR_COUNT" -gt 0 ]; then
        info "Firmware read $RBR_COUNT bytes from UART RX"
    fi

    printf "."
done
echo ""

if ! $L1CTL_OK; then
    warn "No L1CTL response from firmware after ${i}s"
fi

# ==============================================================
header "5. Results"
# ==============================================================

# ---- Bridge log ----
echo -e "${BOLD}--- Bridge ---${NC}"
while IFS= read -r line; do
    case "$line" in
        *"mobile connected"*)  echo -e "  ${GREEN}$line${NC}" ;;
        *"fw→mobile"*)         echo -e "  ${GREEN}$line${NC}" ;;
        *"mobile→fw"*)         echo -e "  ${CYAN}$line${NC}" ;;
        *"error"*|*"Error"*)   echo -e "  ${RED}$line${NC}" ;;
        *)                     echo -e "  ${DIM}$line${NC}" ;;
    esac
done < "$L1CTL_LOG" 2>/dev/null

# ---- Mobile log ----
echo ""
echo -e "${BOLD}--- Mobile ---${NC}"
while IFS= read -r line; do
    clean=$(echo "$line" | sed 's/\x1b\[[0-9;]*m//g')
    case "$clean" in
        *"initialized"*) echo -e "  ${GREEN}$clean${NC}" ;;
        *"WARNING"*|*"Warning"*) echo -e "  ${YELLOW}$clean${NC}" ;;
        *"ERROR"*|*"failed"*)    echo -e "  ${RED}$clean${NC}" ;;
        *"telnet"*)      echo -e "  ${CYAN}$clean${NC}" ;;
        *)               echo -e "  ${DIM}$clean${NC}" ;;
    esac
done < <(tail -10 "$MOBILE_LOG" 2>/dev/null)

# ---- Firmware console (post-boot, decode new output) ----
echo ""
echo -e "${BOLD}--- Firmware console ---${NC}"
decode_console

# ---- UART stats ----
echo ""
echo -e "${BOLD}--- UART Stats ---${NC}"
RBR_COUNT=$(grep -c "RBR<<<" "$LOG" 2>/dev/null || echo "0")
RX_COUNT=$(grep -c "<<<RX" "$LOG" 2>/dev/null || echo "0")
IER_COUNT=$(grep -c "IER=" "$LOG" 2>/dev/null || echo "0")
echo -e "  RX events:  $RX_COUNT"
echo -e "  RBR reads:  $RBR_COUNT"
echo -e "  IER writes: $IER_COUNT"

# ---- Summary ----
header "Summary"
echo -e "  QEMU:    $(kill -0 $QEMU_PID 2>/dev/null && echo -e "${GREEN}alive${NC}" || echo -e "${RED}dead${NC}")"
echo -e "  Bridge:  $(kill -0 $L1CTL_PID 2>/dev/null && echo -e "${GREEN}alive${NC}" || echo -e "${RED}dead${NC}")"
echo -e "  Mobile:  $(kill -0 $MOBILE_PID 2>/dev/null && echo -e "${GREEN}alive${NC}" || echo -e "${RED}dead${NC}")"
echo -e "  L1CTL:   $($L1CTL_OK && echo -e "${GREEN}firmware responded${NC}" || echo -e "${YELLOW}no response yet${NC}")"
echo ""
echo -e "${DIM}  Logs: $LOG | $L1CTL_LOG | $MOBILE_LOG${NC}"
echo -e "${DIM}  Telnet mobile: telnet 127.0.0.1 4247${NC}"
echo ""

# ---- Cleanup ----
header "Cleaning up"
kill "$MOBILE_PID" 2>/dev/null || true
kill "$L1CTL_PID" 2>/dev/null || true
kill "$QEMU_PID" 2>/dev/null || true
wait 2>/dev/null
ok "Done"
