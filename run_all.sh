#!/bin/bash
# run_all_debug.sh — headless looped Calypso runs.
#
#   Usage:  ./run_all_debug.sh [N]            (default N=10)
#           QEMU_LOG_MAX=20000000 ./run_all_debug.sh 5
#
# Each iteration:
#   - launches QEMU (-S -gdb), bridge.py, inject_cfile.py, mobile, hack.py
#   - watches the QEMU log; when it exceeds QEMU_LOG_MAX bytes, kills everything
#   - moves on to the next run with a fresh incremented log set in /root/logs/
#
# All logs land in /root/logs/{qemu,bridge,inject,mobile,hack}_NNN.log.
# /tmp/{qemu,bridge,inject,mobile,hack}.log are symlinked to the *current* run.
#
# Safe to Ctrl-C: traps cleanup.
set -uo pipefail

NRUNS="${1:-10}"
QEMU_LOG_MAX="${QEMU_LOG_MAX:-$((50*1024*1024))}"
RUN_TIMEOUT="${RUN_TIMEOUT:-45}"    # absolute wall-clock cap per run, seconds

QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
FW="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
INJECT="/opt/GSM/qemu-src/inject_cfile.py"
HACK="/opt/GSM/qemu-src/hack.py"
CFILE="/root/out_arfcn_100.cfile"
CFILE_RATE="${CFILE_RATE:-1024000}"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"
L1CTL_SOCK="/tmp/osmocom_l2_1"
LOG_DIR="/root/logs"
VENV="/root/.env"

mkdir -p "$LOG_DIR"
[ -f "$VENV/bin/activate" ] && source "$VENV/bin/activate"

PROC_PATTERNS=(
    "-x qemu-system-arm"
    "-f bridge\.py"
    "-f inject_cfile"
    "-x mobile"
    "-f hack\.py"
)

any_alive() {
    for pat in "${PROC_PATTERNS[@]}"; do
        # shellcheck disable=SC2086
        pgrep $pat >/dev/null 2>&1 && return 0
    done
    return 1
}

cleanup_procs() {
    # Polite SIGTERM first
    for pat in "${PROC_PATTERNS[@]}"; do
        # shellcheck disable=SC2086
        pkill $pat 2>/dev/null || true
    done
    # Wait up to 2s for graceful exit
    for i in 1 2 3 4 5 6 7 8; do
        any_alive || break
        sleep 0.25
    done
    # SIGKILL whatever is still around
    if any_alive; then
        for pat in "${PROC_PATTERNS[@]}"; do
            # shellcheck disable=SC2086
            pkill -9 $pat 2>/dev/null || true
        done
        sleep 0.3
    fi
    rm -f /tmp/osmocom_l2_* "$L1CTL_SOCK"
}

trap 'echo "[run_all] Ctrl-C, cleaning up"; cleanup_procs; exit 130' INT TERM

next_tag() {
    local n=1
    while [ -e "$LOG_DIR/qemu_$(printf %03d $n).log" ]; do
        n=$((n+1))
    done
    printf %03d $n
}

# ── Sanity ──
[ -x "$QEMU" ]   || { echo "FATAL: $QEMU not found"; exit 1; }
[ -f "$FW" ]     || { echo "FATAL: $FW not found"; exit 1; }
[ -f "$BRIDGE" ] || { echo "FATAL: $BRIDGE not found"; exit 1; }
[ -f "$INJECT" ] || { echo "FATAL: $INJECT not found"; exit 1; }
[ -f "$CFILE" ]  || { echo "FATAL: $CFILE not found"; exit 1; }

echo "[run_all] launching $NRUNS runs (kill at ${QEMU_LOG_MAX} bytes or ${RUN_TIMEOUT}s)"

for k in $(seq 1 "$NRUNS"); do
    cleanup_procs

    TAG=$(next_tag)
    QEMU_LOG="$LOG_DIR/qemu_$TAG.log"
    BRIDGE_LOG="$LOG_DIR/bridge_$TAG.log"
    INJECT_LOG="$LOG_DIR/inject_$TAG.log"
    MOBILE_LOG="$LOG_DIR/mobile_$TAG.log"
    HACK_LOG="$LOG_DIR/hack_$TAG.log"

    ln -sf "$QEMU_LOG"   /tmp/qemu.log
    ln -sf "$BRIDGE_LOG" /tmp/bridge.log
    ln -sf "$INJECT_LOG" /tmp/inject.log
    ln -sf "$MOBILE_LOG" /tmp/mobile.log
    ln -sf "$HACK_LOG"   /tmp/hack.log

    echo "─────────────────────────────────────────────────────────"
    echo "[run_all] run $k/$NRUNS  → tag=$TAG"
    echo "          $QEMU_LOG"

    # QEMU
    CALYPSO_BSP_BYPASS_BDLENA=1 \
    CALYPSO_BSP_DARAM_ADDR=0x021f \
    CALYPSO_BSP_DARAM_LEN=512 \
    CALYPSO_DBG=all \
    "$QEMU" -M calypso \
        -kernel "$FW" -serial pty -display none \
        > "$QEMU_LOG" 2>&1 &
    QEMU_PID=$!

    PTY=""
    for i in $(seq 1 20); do
        sleep 0.25
        PTY=$(grep -o "/dev/pts/[0-9]*" "$QEMU_LOG" 2>/dev/null | head -1)
        [ -n "$PTY" ] && break
    done
    if [ -z "$PTY" ]; then
        echo "[run_all] FATAL: QEMU did not allocate a PTY (run $k)"
        kill -9 "$QEMU_PID" 2>/dev/null
        continue
    fi
    echo "[run_all]   QEMU pid=$QEMU_PID pty=$PTY"

    # bridge
    python3 "$BRIDGE" "$PTY" --sock "$L1CTL_SOCK" > "$BRIDGE_LOG" 2>&1 &
    BRIDGE_PID=$!
    for i in $(seq 1 30); do
        [ -S "$L1CTL_SOCK" ] && break
        sleep 0.25
    done
    [ -S "$L1CTL_SOCK" ] || { echo "[run_all]   bridge socket timeout"; }

    sleep 1

    # inject (background, looping cfile)
    python3 "$INJECT" "$CFILE" --rate "$CFILE_RATE" --loop > "$INJECT_LOG" 2>&1 &
    INJECT_PID=$!

    # mobile (delayed)
    ( sleep 5; mobile -c "$MOBILE_CFG" > "$MOBILE_LOG" 2>&1 ) &
    MOBILE_PID=$!

    # no hack.py in this variant — pure baseline
    HACK_PID=""

    # Live tail of QEMU log to the main terminal for this run
    tail -F "$QEMU_LOG" 2>/dev/null &
    TAIL_PID=$!

    # Watchdog: kill when log size exceeds limit, or wall-clock cap, or QEMU dies
    t0=$(date +%s)
    while :; do
        if ! kill -0 "$QEMU_PID" 2>/dev/null; then
            echo "[run_all]   QEMU exited on its own"
            break
        fi
        sz=$(stat -c %s "$QEMU_LOG" 2>/dev/null || echo 0)
        if [ "$sz" -ge "$QEMU_LOG_MAX" ]; then
            echo "[run_all]   log limit reached (${sz} bytes), killing"
            break
        fi
        now=$(date +%s)
        if [ $((now - t0)) -ge "$RUN_TIMEOUT" ]; then
            echo "[run_all]   wall timeout reached, killing"
            break
        fi
        sleep 1
    done

    kill -9 "$TAIL_PID" 2>/dev/null || true
    cleanup_procs
    sleep 1
done

echo "[run_all] done. logs in $LOG_DIR/"
ls -la "$LOG_DIR/" | tail -20
