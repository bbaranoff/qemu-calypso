#!/bin/bash
# run_si.sh — Calypso QEMU pipeline with live RSL SI injection.
#
# Étape 3 path : SI1/SI2/SI3/SI4/SI13 alimentés dynamiquement par
# rsl_si_tap.py (sniff RSL osmo-bsc → osmo-bts) → /dev/shm/calypso_si.bin
# (cf. doc/MMAP_SI_FORMAT.md).
#
# Différences vs run_new.sh :
#   - rsl_si_tap.py démarre AVANT osmo-bts pour capturer BCCH_INFO de l'attach
#   - QEMU : CALYPSO_SI_MMAP_PATH env var passée à QEMU
#   - mmap rempli byte-exact depuis le live BSC, pas de hardcode
#
# Note : populate-si.sh existe toujours dans scripts/ comme outil debug
# manuel pour tester l'interface mmap sans dépendre de osmo-bsc/RSL.
# Plus utilisé dans le path de boot normal.

set -euo pipefail

SESSION="calypso"
FW_ELF="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
FW_BIN="/opt/GSM/firmware/board/compal_e88/layer1.highram.bin"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
OSMOCON="/opt/GSM/osmocom-bb/src/host/osmocon/osmocon"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"

# ---- mmap SI file ----
# Override possible : CALYPSO_SI_MMAP_PATH=/path/to/file run_si.sh
CALYPSO_SI_MMAP_PATH="${CALYPSO_SI_MMAP_PATH:-/dev/shm/calypso_si.bin}"
export CALYPSO_SI_MMAP_PATH

# ---- DSP / DIAG instruments (override at command line if needed) ----
CALYPSO_DSP_ROM="${CALYPSO_DSP_ROM:-/opt/GSM/calypso_dsp.txt}"
CALYPSO_BSP_DARAM_ADDR="${CALYPSO_BSP_DARAM_ADDR:-0x3fb0}"
CALYPSO_SIM_CFG="${CALYPSO_SIM_CFG:-$MOBILE_CFG}"
export CALYPSO_DSP_ROM CALYPSO_BSP_DARAM_ADDR CALYPSO_SIM_CFG

# ---- Env-gated dev assists (default OFF = real path) ----
# Set =1 in the calling environment to enable. Cf. README.md "Env vars".
CALYPSO_FBSB_SYNTH="${CALYPSO_FBSB_SYNTH:-0}"
CALYPSO_BCCH_INJECT="${CALYPSO_BCCH_INJECT:-0}"
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-0}"
CALYPSO_NDB_D_RACH_OFFSET="${CALYPSO_NDB_D_RACH_OFFSET:-}"
CALYPSO_RACH_FORCE_BSIC="${CALYPSO_RACH_FORCE_BSIC:-}"
BRIDGE_CLK_FROM_QEMU="${BRIDGE_CLK_FROM_QEMU:-1}"
export CALYPSO_FBSB_SYNTH CALYPSO_BCCH_INJECT CALYPSO_W1C_LATCH \
       CALYPSO_NDB_D_RACH_OFFSET CALYPSO_RACH_FORCE_BSIC BRIDGE_CLK_FROM_QEMU

# ---- icount mode (deterministic virtual clock paced by instruction count) ----
# Default ON (auto = shift=auto,sleep=on,align=off). Set CALYPSO_ICOUNT=off
# to disable. The kick timer (calypso_kick_cb) was moved to
# QEMU_CLOCK_VIRTUAL so it no longer races with icount.
# Other accepted values:
#   auto              shift dynamic, wall-clock aligned (recommended)
#   shift=N,sleep=on  fixed shift (1<<N instr ≈ 1ns), explicit sleep
#   off               disable (legacy default-clock mode)
CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-auto}"
export CALYPSO_ICOUNT
if [ "$CALYPSO_ICOUNT" = "off" ]; then
    QEMU_ICOUNT_FLAG=""
else
    QEMU_ICOUNT_FLAG="-icount $CALYPSO_ICOUNT"
fi

# ---- log paths ----
QEMU_LOG="/root/qemu.log"
BRIDGE_LOG="/tmp/bridge.log"
OSMOCON_LOG="/tmp/osmocon.log"
MOBILE_LOG="/tmp/mobile.log"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
L1CTL_SOCK="/tmp/osmocom_l2"
QEMU_DUMMY_SOCK="/tmp/qemu_l1ctl_disabled"

# ---------- cleanup ----------
rm -f "$QEMU_LOG" "$BRIDGE_LOG" "$OSMOCON_LOG" "$MOBILE_LOG" \
      "$MON_SOCK" "$L1CTL_SOCK" "$QEMU_DUMMY_SOCK"

tmux kill-session -t "$SESSION" 2>/dev/null || true
killall -9 qemu-system-arm osmo-bts-trx mobile osmocon 2>/dev/null || true
pkill -9 -f bridge.py 2>/dev/null || true
pkill -9 -f rsl_si_tap.py 2>/dev/null || true
rm -f "$L1CTL_SOCK" "$MON_SOCK" "$QEMU_DUMMY_SOCK" /tmp/osmocom_l2_*
sleep 1

/etc/osmocom/status.sh stop 2>/dev/null || true
/etc/osmocom/osmo-start.sh 2>/dev/null || true

# Note : pas de pré-population mmap. rsl_si_tap.py (phase 4 ci-dessous)
# capture les vraies BCCH_INFO depuis le BSC dès l'attache osmo-bts.
# Si besoin de warm-start manuel pour debug, lancer scripts/populate-si.sh.

tmux new-session -d -s "$SESSION" -n qemu

# ---------- 1. QEMU ----------
# icount controlled by CALYPSO_ICOUNT env var (default 'auto'). The kick
# timer in calypso_trx.c was moved to QEMU_CLOCK_VIRTUAL so icount no
# longer freezes the TDMA tick → bridge UDP path. If you observe the
# bridge wait timeout again, fall back with CALYPSO_ICOUNT=off.
L1CTL_SOCK="$QEMU_DUMMY_SOCK" \
"$QEMU" -M calypso -cpu arm946 \
    $QEMU_ICOUNT_FLAG \
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

# ---------- 4. rsl_si_tap.py — BEFORE osmo-bts pour capturer BCCH_INFO du re-attach ----------
# Sniffe RSL osmo-bsc → osmo-bts (TCP lo:3003), parse BCCH_INFO, populate
# /dev/shm/calypso_si.bin dynamiquement. Doit démarrer AVANT osmo-bts pour
# capturer les BCCH_INFO émis par BSC au moment de l'attachement BTS.
RSL_SI_TAP="/opt/GSM/qemu-src/scripts/rsl_si_tap.py"
tmux new-window -t "$SESSION" -n rsl_si_tap
tmux send-keys -t "$SESSION:rsl_si_tap" \
    "python3 $RSL_SI_TAP 2>&1 | tee /tmp/rsl_si_tap.log" C-m
sleep 1  # let tap bind socket before BTS attaches

# ---------- 5. osmo-bts-trx ----------
tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" "osmo-bts-trx -c $BTS_CFG" C-m
sleep 2

# ---------- 6. mobile ----------
tmux new-window -t "$SESSION" -n mobile
tmux send-keys -t "$SESSION:mobile" \
    "sleep 3 && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | tee $MOBILE_LOG" C-m

# ---------- 7. gsmtap capture ----------
tmux new-window -t "$SESSION" -n gsmtap
tmux send-keys -t "$SESSION:gsmtap" \
    "sleep 5 && tcpdump -i eth1 -w /root/mobile-gsmtap.pcap udp port 4729" C-m

# ---------- shell + attach ----------
tmux new-window -t "$SESSION" -n shell

echo
echo "Pipeline launched. Attach with: tmux attach -t $SESSION"
echo "ENV summary:"
echo "  CALYPSO_SI_MMAP_PATH        = $CALYPSO_SI_MMAP_PATH"
echo "  CALYPSO_DSP_ROM             = $CALYPSO_DSP_ROM"
echo "  CALYPSO_BSP_DARAM_ADDR      = $CALYPSO_BSP_DARAM_ADDR"
echo "  CALYPSO_SIM_CFG             = $CALYPSO_SIM_CFG"
echo "  CALYPSO_FBSB_SYNTH          = $CALYPSO_FBSB_SYNTH"
echo "  CALYPSO_BCCH_INJECT         = $CALYPSO_BCCH_INJECT"
echo "  CALYPSO_W1C_LATCH           = $CALYPSO_W1C_LATCH"
echo "  CALYPSO_NDB_D_RACH_OFFSET   = ${CALYPSO_NDB_D_RACH_OFFSET:-(default 0x01CB)}"
echo "  CALYPSO_RACH_FORCE_BSIC     = ${CALYPSO_RACH_FORCE_BSIC:-(unset = use d_rach byte)}"
echo "  BRIDGE_CLK_FROM_QEMU        = $BRIDGE_CLK_FROM_QEMU"
echo "  CALYPSO_ICOUNT              = $CALYPSO_ICOUNT  (flag: ${QEMU_ICOUNT_FLAG:-(none)})"
echo
echo "Manual warm-start (debug, if BSC unavailable) :"
echo "  /opt/GSM/qemu-src/scripts/populate-si.sh"
echo

tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"
