#!/bin/bash
# run_si.sh — Calypso QEMU pipeline with mmap-based SI injection.
#
# Étape 2b/3 path : SI1/SI2/SI3/SI4/SI13 alimentés via mmap shared file
# /dev/shm/calypso_si.bin (cf. doc/MMAP_SI_FORMAT.md).
#
# Différences vs run_new.sh :
#   - Phase 0 : populate-si.sh écrit le mmap au boot avec les 5 SI
#               RSL-extracted (étape 2b stub disposable).
#   - QEMU : CALYPSO_SI_MMAP_PATH env var passée à QEMU.
#   - (Future étape 3) : remplacer populate-si.sh par rsl_si_tap.py
#                        en tmux window dédiée.
#
# IMPORTANT : populate-si.sh est transitoire. À supprimer une fois
# scripts/rsl_si_tap.py opérationnel en steady-state (sniff RSL live).

set -euo pipefail

SESSION="calypso"
FW_ELF="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
FW_BIN="/opt/GSM/firmware/board/compal_e88/layer1.highram.bin"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
OSMOCON="/opt/GSM/osmocom-bb/src/host/osmocon/osmocon"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"
POPULATE_SI="/opt/GSM/qemu-src/scripts/populate-si.sh"

# ---- mmap SI file ----
# Override possible : CALYPSO_SI_MMAP_PATH=/path/to/file run_si.sh
CALYPSO_SI_MMAP_PATH="${CALYPSO_SI_MMAP_PATH:-/dev/shm/calypso_si.bin}"
export CALYPSO_SI_MMAP_PATH

# ---- DSP / DIAG instruments (override at command line if needed) ----
CALYPSO_DSP_ROM="${CALYPSO_DSP_ROM:-/opt/GSM/calypso_dsp.txt}"
CALYPSO_BSP_DARAM_ADDR="${CALYPSO_BSP_DARAM_ADDR:-0x3fb0}"
CALYPSO_FORCE_INTM_CLEAR_AT="${CALYPSO_FORCE_INTM_CLEAR_AT:-}"
CALYPSO_SIM_CFG="${CALYPSO_SIM_CFG:-$MOBILE_CFG}"
export CALYPSO_DSP_ROM CALYPSO_BSP_DARAM_ADDR CALYPSO_FORCE_INTM_CLEAR_AT CALYPSO_SIM_CFG

# ---- log paths ----
QEMU_LOG="/root/qemu.log"
BRIDGE_LOG="/tmp/bridge.log"
OSMOCON_LOG="/tmp/osmocon.log"
MOBILE_LOG="/tmp/mobile.log"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
L1CTL_SOCK="/tmp/osmocom_l2"
QEMU_DUMMY_SOCK="/tmp/qemu_l1ctl_disabled"

# ---------- cleanup ----------
tmux kill-session -t "$SESSION" 2>/dev/null || true
killall -9 qemu-system-arm osmo-bts-trx mobile osmocon 2>/dev/null || true
pkill -9 -f bridge.py 2>/dev/null || true
pkill -9 -f rsl_si_tap.py 2>/dev/null || true
rm -f "$L1CTL_SOCK" "$MON_SOCK" "$QEMU_DUMMY_SOCK" /tmp/osmocom_l2_*
sleep 1

/etc/osmocom/status.sh stop 2>/dev/null || true
/etc/osmocom/osmo-start.sh 2>/dev/null || true

# ---------- 0. mmap SI populator (étape 2b stub) ----------
# Écrit /dev/shm/calypso_si.bin avec les 5 SI RSL-extracted.
# QEMU lit ce fichier pour injecter les bons SI selon TC (TS 44.018 §3.4).
echo -n "Populating mmap SI file ($CALYPSO_SI_MMAP_PATH)... "
if [ -x "$POPULATE_SI" ]; then
    "$POPULATE_SI" || { echo "FAIL"; exit 1; }
else
    echo "WARN — $POPULATE_SI not executable, fallback to hardcoded SI3"
fi

tmux new-session -d -s "$SESSION" -n qemu

# ---------- 1. QEMU ----------
# CALYPSO_SI_MMAP_PATH propagé via export en haut du script.
L1CTL_SOCK="$QEMU_DUMMY_SOCK" \
"$QEMU" -M calypso -cpu arm946 \
    -serial pty -serial pty \
    -monitor "unix:${MON_SOCK},server,nowait" \
    -kernel "$FW_ELF" \
    > "$QEMU_LOG" 2>&1 &
QEMU_PID=$!
tmux send-keys -t "$SESSION:qemu" "tail -f $QEMU_LOG" C-m

echo -n "Waiting for QEMU PTY allocation..."
PTY_MODEM=""
for i in $(seq 1 30); do
    if grep -q 'redirected to /dev/pts/.* (label serial0)' "$QEMU_LOG" 2>/dev/null; then
        PTY_MODEM=$(grep 'redirected to /dev/pts/.* (label serial0)' "$QEMU_LOG" \
                    | sed -E 's/.*redirected to (\/dev\/pts\/[0-9]+).*/\1/' | head -1)
        break
    fi
    sleep 1; echo -n "."
done
if [ -z "$PTY_MODEM" ]; then
    echo " TIMEOUT — no PTY in $QEMU_LOG"
    exit 1
fi
echo " OK ($PTY_MODEM, QEMU_PID=$QEMU_PID)"

# ---------- 2. osmocon ----------
tmux new-window -t "$SESSION" -n osmocon
tmux send-keys -t "$SESSION:osmocon" \
    "$OSMOCON -m romload -i 100 -p $PTY_MODEM -s $L1CTL_SOCK $FW_BIN -d tr 2>&1 | tee $OSMOCON_LOG" C-m

echo -n "Waiting for osmocon to expose $L1CTL_SOCK..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
if [ -S "$L1CTL_SOCK" ]; then echo " OK"; else echo " WARN — socket missing"; fi

# ---------- 3. bridge.py ----------
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" \
    "python3 $BRIDGE 2>&1 | tee $BRIDGE_LOG" C-m

echo -n "Waiting for bridge to receive QEMU ticks..."
for i in $(seq 1 30); do
    grep -q "QEMU tick" "$BRIDGE_LOG" 2>/dev/null && break
    sleep 1; echo -n "."
done
if grep -q "QEMU tick" "$BRIDGE_LOG" 2>/dev/null; then echo " OK"; else echo " TIMEOUT"; fi

# ---------- 4. osmo-bts-trx ----------
tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" "osmo-bts-trx -c $BTS_CFG" C-m
sleep 2

# ---------- 5. mobile ----------
tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" \
    "sleep 3 && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | tee $MOBILE_LOG" C-m

# ---------- 6. gsmtap capture ----------
tmux new-window -t "$SESSION" -n gsmtap
tmux send-keys -t "$SESSION:gsmtap" \
    "sleep 5 && tcpdump -i eth1 -w /root/mobile-gsmtap.pcap udp port 4729" C-m

# ---------- 7. (étape 3 future) rsl_si_tap.py ----------
# Dès que scripts/rsl_si_tap.py est implémenté, dé-commenter ce block et
# supprimer l'appel populate-si.sh phase 0 ci-dessus.
#
#tmux new-window -t "$SESSION" -n rsl_si_tap
#tmux send-keys -t "$SESSION:rsl_si_tap" \
#    "python3 /opt/GSM/qemu-src/scripts/rsl_si_tap.py 2>&1 | tee /tmp/rsl_si_tap.log" C-m

# ---------- shell + attach ----------
tmux new-window -t "$SESSION" -n shell

echo
echo "Pipeline launched. Attach with: tmux attach -t $SESSION"
echo "ENV summary:"
echo "  CALYPSO_SI_MMAP_PATH        = $CALYPSO_SI_MMAP_PATH"
echo "  CALYPSO_DSP_ROM             = $CALYPSO_DSP_ROM"
echo "  CALYPSO_BSP_DARAM_ADDR      = $CALYPSO_BSP_DARAM_ADDR"
echo "  CALYPSO_FORCE_INTM_CLEAR_AT = ${CALYPSO_FORCE_INTM_CLEAR_AT:-(unset)}"
echo "  CALYPSO_SIM_CFG             = $CALYPSO_SIM_CFG"
echo
echo "To force re-populate SI mmap during runtime :"
echo "  $POPULATE_SI"
echo

tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"
