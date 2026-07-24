#!/bin/bash
# run-all-debug.sh — Version debug-friendly de run-all.sh.
#
# Différences avec run-all.sh :
#   - RUN_DURATION par défaut = 180s (vs 60s) pour stabilisation longue
#   - pytest verbosity = vv (détails complets)
#   - logs verbose (CALYPSO_TIMER=1, IRDA capture ON, gen-doc auto)
#   - sessions tmux NON-killées entre modes (à la fin tu peux attacher)
#   - dump des verdicts JSON + invocation select-verdict.sh à la fin
#   - sauvegarde COMPLÈTE des logs (qemu, mobile, bts, ipc, irda, frame_irq)
#
# Override : MODES="shunt full" ./run-all-debug.sh

set -uo pipefail

RUN_DURATION="${RUN_DURATION:-180}"
MODES="${MODES:-full shunt shunt-ipc bridge bare}"
OUT_DIR="${OUT_DIR:-/tmp/run-all-debug}"
SESSION="calypso"
HERE="$(cd "$(dirname "$0")" && pwd)"

# Force debug-friendly env
export CALYPSO_TIMER=1                  # log timer ticks
export CALYPSO_IRDA_CAPTURE=1
export CALYPSO_AUTO_GEN_DOC=1
export CALYPSO_PYTEST_VERBOSITY=vv
export CALYPSO_PYTEST_SCOPE=default

# DEBUG_FULL=1 (env ou --full) active toutes les sondes c54x lourdes
# (SP_RING, AR_TRACE, WATCH_3FBE, CORRELATOR, MVPD, RSBX). Coûteux en
# logs mais c'est le profil "diag FULL" du menu reproduit en CLI.
DEBUG_FULL="${DEBUG_FULL:-0}"
for arg in "$@"; do
    [ "$arg" = "--full" ] && DEBUG_FULL=1
done
if [ "$DEBUG_FULL" = "1" ]; then
    echo "[run-all-debug] DEBUG_FULL=1 → full c54x probes activées"
    export CALYPSO_PROBE_BOOTSTUB=0
    export CALYPSO_SP_RING=1
    export CALYPSO_SP_RING_TRIG=bootstub
    export CALYPSO_SP_RING_INSN_MIN=1000000
    export CALYPSO_SP_ABS_TRACE=1
    export CALYPSO_AR_TRACE=0xFF
    export CALYPSO_WATCH_3FBE=1
    export CALYPSO_CORRELATOR_TRACE=1
    export CALYPSO_MVPD_TRACE=1
    export CALYPSO_MVPD_BOOT_LIMIT=500000
    export CALYPSO_RSBX_INTM_TRACE=1
fi

mkdir -p "$OUT_DIR"
echo "===== run-all-debug.sh ====="
echo "Modes        : $MODES"
echo "Duration/run : ${RUN_DURATION}s  (debug → longer)"
echo "Verbosity    : ${CALYPSO_PYTEST_VERBOSITY}"
echo "Out dir      : $OUT_DIR"
echo "Total ETA    : ~$(( $(echo "$MODES" | wc -w) * RUN_DURATION ))s"
echo

_grep_count() {
    local pattern="$1" file="$2"
    [ -f "$file" ] || { echo 0; return; }
    grep -c -E "$pattern" "$file" 2>/dev/null || echo 0
}

declare -A RESULTS

for mode in $MODES; do
    echo "===== Mode: $mode (debug run) ====="
    mode_dir="$OUT_DIR/$mode"
    rm -rf "$mode_dir"
    mkdir -p "$mode_dir"

    # Cleanup avant
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
    pkill -9 -f calypso-ipc-device 2>/dev/null || true
    pkill -9 -f "bridge\.py" 2>/dev/null || true
    sleep 2

    echo "[$mode] Launching ./run.sh (CALYPSO_NO_ATTACH=1)..."
    CALYPSO_MODE="$mode" \
    CALYPSO_NO_ATTACH=1 \
    CALYPSO_L2_CLIENT=mobile \
    CALYPSO_MODE_TAG="$mode" \
        "$HERE/run.sh" > "$mode_dir/run.stdout.log" 2> "$mode_dir/run.stderr.log" &
    RUN_PID=$!

    echo "[$mode] Waiting ${RUN_DURATION}s for stabilization..."
    # On garde une fenêtre tmux 'monitor' qui tail le qemu.log live pour debug.
    # User peut attach via : tmux attach -t calypso
    sleep "$RUN_DURATION"

    # Sample logs (debug = on prend TOUT)
    for f in qemu.log mobile.log bts.log osmocon.log osmo-trx-ipc.log \
             calypso-ipc-device.log fw-irda.log bridge.py.log frame_irq.log; do
        src="/tmp/$f"; [ "$f" = "qemu.log" ] && src="/root/qemu.log"
        [ -f "$src" ] && cp "$src" "$mode_dir/$f" 2>/dev/null || true
    done

    # Per-mode pytest (vv verbosity, génère test_results_latest.zip + verdict_*.json)
    PYTEST_BIN=""
    for cand in /tmp/calypso-venv/bin/pytest /opt/GSM/calypso-venv/bin/pytest \
                /usr/local/bin/pytest /usr/bin/pytest; do
        [ -x "$cand" ] && PYTEST_BIN="$cand" && break
    done
    [ -z "$PYTEST_BIN" ] && python3 -c "import pytest" 2>/dev/null && PYTEST_BIN="python3 -m pytest"

    if [ -n "$PYTEST_BIN" ]; then
        echo "[$mode] Running per-mode pytest (vv)..."
        (
            cd "$HERE/tests" && \
            CALYPSO_TEST_OUT=/tmp \
            CALYPSO_REPO="$HERE" \
            CALYPSO_MODE_TAG="$mode" \
                $PYTEST_BIN -vv --tb=long --color=yes \
                    --ignore=functional --ignore=guest-debug \
                    --ignore=qemu-iotests --ignore=qtest --ignore=unit \
                    --ignore=tcg --ignore=migration --ignore=vm \
                    --ignore=avocado --ignore=fp \
                    --ignore=test_run_all_modes.py \
                    > "$mode_dir/pytest-per-mode.log" 2>&1
        ) || true
        # Collect verdict + zip
        [ -f /tmp/verdict_${mode}.json ] && cp /tmp/verdict_${mode}.json "$mode_dir/"
        [ -f /tmp/test_results_latest.zip ] && cp /tmp/test_results_latest.zip "$mode_dir/test_results_${mode}.zip"
    fi

    # Extract indicators
    qemu_log="$mode_dir/qemu.log"; mobile_log="$mode_dir/mobile.log"
    bts_log="$mode_dir/bts.log"; ipc_log="$mode_dir/calypso-ipc-device.log"
    trxipc_log="$mode_dir/osmo-trx-ipc.log"
    RESULTS[$mode]="shunt_latch=$(_grep_count '\[dsp-shunt\] LATCH' "$qemu_log") \
shunt_disp=$(_grep_count '\[dsp-shunt\] DISPATCH' "$qemu_log") \
fbsb_conf=$(_grep_count 'FBSB_CONF|fbsb.*conf' "$mobile_log") \
rr_est=$(_grep_count 'RR_EST|RR EST_REQ' "$mobile_log") \
lost=$(_grep_count 'LOST [0-9]' "$mobile_log") \
err_q=$(_grep_count 'ERR|FATAL|panic|assert' "$qemu_log") \
err_m=$(_grep_count 'ERR|FATAL|panic' "$mobile_log") \
bts_ok=$(_grep_count 'phy0\.0: Opening|DL1C NOTICE|FN faster than TRX|RACH received|Listening|TRX online' "$bts_log") \
ipc_ok=$(_grep_count 'GREETING|OPEN_CNF|info_cnf' "$ipc_log") \
ipc_err=$(_grep_count 'DDEV ERROR|chan num mismatch|antenna not found' "$trxipc_log")"

    echo "[$mode] ${RESULTS[$mode]}"

    # Cleanup
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
    pkill -9 -f calypso-ipc-device 2>/dev/null || true
    pkill -9 -f "bridge\.py" 2>/dev/null || true
    kill -9 "$RUN_PID" 2>/dev/null || true
    wait "$RUN_PID" 2>/dev/null || true
    sleep 2
    echo
done

# Build JSON
JSON="$OUT_DIR/results.json"
{
    echo "{"
    echo "  \"generated\": \"$(date -Iseconds)\","
    echo "  \"duration_seconds\": $RUN_DURATION,"
    echo "  \"debug_mode\": true,"
    echo "  \"modes\": {"
    first=1
    for mode in $MODES; do
        eval "${RESULTS[$mode]}"
        [ $first -eq 0 ] && echo ","
        first=0
        printf '    "%s": {"fbsb_conf": %d, "shunt_latch": %d, "shunt_disp": %d, "rr_est": %d, "lost": %d, "err_q": %d, "err_m": %d, "bts_ok": %d, "ipc_ok": %d, "ipc_err": %d}' \
            "$mode" "${fbsb_conf:-0}" "${shunt_latch:-0}" "${shunt_disp:-0}" \
            "${rr_est:-0}" "${lost:-0}" "${err_q:-0}" "${err_m:-0}" \
            "${bts_ok:-0}" "${ipc_ok:-0}" "${ipc_err:-0}"
    done
    echo
    echo "  }"
    echo "}"
} > "$JSON"
echo "JSON     : $JSON"

# Cross-mode pytest
PYTEST_BIN=""
for cand in /tmp/calypso-venv/bin/pytest /opt/GSM/calypso-venv/bin/pytest \
            /usr/local/bin/pytest /usr/bin/pytest; do
    [ -x "$cand" ] && PYTEST_BIN="$cand" && break
done
[ -z "$PYTEST_BIN" ] && python3 -c "import pytest" 2>/dev/null && PYTEST_BIN="python3 -m pytest"

if [ -n "$PYTEST_BIN" ]; then
    echo
    echo "===== Cross-mode pytest (debug vv) ====="
    RUN_ALL_JSON="$JSON" $PYTEST_BIN -vv --tb=long --color=yes \
        "$HERE/tests/test_run_all_modes.py" | tee "$OUT_DIR/pytest-cross.log"
fi

# Invoke select-verdict.sh
echo
if [ -x "$HERE/select-verdict.sh" ]; then
    echo "===== Verdict selector ====="
    CALYPSO_TEST_OUT=/tmp RUN_ALL_JSON="$JSON" "$HERE/select-verdict.sh"
else
    echo "[run-all-debug] select-verdict.sh absent ou non exécutable"
fi

echo
echo "===== run-all-debug DONE ====="
echo "Out : $OUT_DIR/"
echo "  - results.json"
echo "  - <mode>/{qemu,mobile,bts,...}.log + verdict_<mode>.json + test_results_<mode>.zip"
echo "  - pytest-cross.log"
echo "Verdict choisi : /tmp/selected_verdict.txt"
