#!/bin/bash
# run-all.sh — Smoke test des 5 modes CALYPSO_MODE (full/shunt/shunt-ipc/bridge/bare).
#
# Pour chaque mode :
#   1. Kill tout (cleanup)
#   2. Launch ./run.sh avec CALYPSO_MODE=X et CALYPSO_NO_ATTACH=1
#   3. Wait $RUN_DURATION secondes pour stabilisation
#   4. Sample les logs (qemu.log, mobile.log, bts.log, ipc-device.log)
#   5. Capture indicateurs : FBSB_CONF count, RR_EST, errors, mobile state
#   6. Kill la session tmux
#   7. Move logs vers /tmp/run-all-<mode>/
#
# Final : tableau récap des résultats.
#
# Override duration : RUN_DURATION=120 ./run-all.sh

set -uo pipefail   # PAS -e — on veut continuer même si un mode crash

RUN_DURATION="${RUN_DURATION:-60}"
MODES="${MODES:-full shunt shunt-ipc bridge bare}"
OUT_DIR="${OUT_DIR:-/tmp/run-all}"
SESSION="calypso"
HERE="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$OUT_DIR"
echo "===== run-all.sh ====="
echo "Modes        : $MODES"
echo "Duration/run : ${RUN_DURATION}s"
echo "Out dir      : $OUT_DIR"
echo "Total ETA    : ~$(( $(echo "$MODES" | wc -w) * RUN_DURATION ))s"
echo

# Indicators à capturer dans les logs
_grep_count() {
    local pattern="$1" file="$2"
    [ -f "$file" ] || { echo 0; return; }
    grep -c -E "$pattern" "$file" 2>/dev/null || echo 0
}

_extract_indicators() {
    local mode_dir="$1"
    local qemu="$mode_dir/qemu.log"
    local mobile="$mode_dir/mobile.log"
    local bts="$mode_dir/bts.log"
    local ipc="$mode_dir/calypso-ipc-device.log"
    local shunt_latch=$(_grep_count "\[dsp-shunt\] LATCH" "$qemu")
    local shunt_disp=$(_grep_count   "\[dsp-shunt\] DISPATCH" "$qemu")
    local fbsb_conf=$(_grep_count    "FBSB_CONF|fbsb.*conf" "$mobile")
    local rr_est=$(_grep_count       "RR_EST|RR EST_REQ" "$mobile")
    local lost=$(_grep_count         "LOST [0-9]" "$mobile")
    local err_q=$(_grep_count        "ERR|FATAL|panic|assert" "$qemu")
    local err_m=$(_grep_count        "ERR|FATAL|panic" "$mobile")
    # BTS alive markers: TRX connections opened, DL1C scheduler tick, FN compensating,
    # or any RACH/sysinfo activity. Tolerant to multi-mode log content.
    local bts_ok=$(_grep_count       "phy0\.0: Opening|DL1C NOTICE|FN faster than TRX|RACH received|Listening|TRX online" "$bts")
    local ipc_ok=$(_grep_count       "ipc_sock0|GREETING|OPEN_CNF" "$ipc")
    echo "shunt_latch=$shunt_latch shunt_disp=$shunt_disp fbsb_conf=$fbsb_conf rr_est=$rr_est lost=$lost err_q=$err_q err_m=$err_m bts_ok=$bts_ok ipc_ok=$ipc_ok"
}

declare -A RESULTS

for mode in $MODES; do
    echo "===== Mode: $mode ====="
    mode_dir="$OUT_DIR/$mode"
    rm -rf "$mode_dir"
    mkdir -p "$mode_dir"

    # Cleanup avant
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
    pkill -9 -f calypso-ipc-device 2>/dev/null || true
    pkill -9 -f "bridge\.py" 2>/dev/null || true
    sleep 2

    # Launch
    echo "[$mode] Launching ./run.sh (CALYPSO_NO_ATTACH=1) ..."
    CALYPSO_MODE="$mode" \
    CALYPSO_NO_ATTACH=1 \
    CALYPSO_L2_CLIENT=mobile \
    CALYPSO_AUTO_GEN_DOC=0 \
        "$HERE/run.sh" > "$mode_dir/run.stdout.log" 2> "$mode_dir/run.stderr.log" &
    RUN_PID=$!

    # Wait stabilization
    echo "[$mode] Waiting ${RUN_DURATION}s for stabilization..."
    sleep "$RUN_DURATION"

    # Sample logs
    for f in qemu.log mobile.log bts.log osmocon.log osmo-trx-ipc.log calypso-ipc-device.log fw-irda.log bridge.py.log; do
        src="/tmp/$f"; [ "$f" = "qemu.log" ] && src="/root/qemu.log"
        [ -f "$src" ] && cp "$src" "$mode_dir/$f" 2>/dev/null || true
    done

    # Per-mode pytest inline (génère /tmp/test_results_latest.zip).
    # On run le pytest synchronously plutôt que le gen-doc auto (qui est
    # async et risque de pas finir avant qu'on kill la session).
    PYTEST_BIN=""
    for cand in /tmp/calypso-venv/bin/pytest /opt/GSM/calypso-venv/bin/pytest \
                /usr/local/bin/pytest /usr/bin/pytest; do
        [ -x "$cand" ] && PYTEST_BIN="$cand" && break
    done
    [ -z "$PYTEST_BIN" ] && python3 -c "import pytest" 2>/dev/null && PYTEST_BIN="python3 -m pytest"

    if [ -n "$PYTEST_BIN" ]; then
        echo "[$mode] Running per-mode pytest (zip generation)..."
        (
            cd "$HERE/tests" && \
            CALYPSO_TEST_OUT=/tmp \
            CALYPSO_REPO="$HERE" \
            CALYPSO_MODE_TAG="$mode" \
                $PYTEST_BIN --tb=line --color=yes \
                    --ignore=functional --ignore=guest-debug \
                    --ignore=qemu-iotests --ignore=qtest --ignore=unit \
                    --ignore=tcg --ignore=migration --ignore=vm \
                    --ignore=avocado --ignore=fp \
                    --ignore=test_run_all_modes.py \
                    > "$mode_dir/pytest-per-mode.log" 2>&1
        ) || true
        # Collect zip into mode_dir
        if [ -f /tmp/test_results_latest.zip ]; then
            cp /tmp/test_results_latest.zip "$mode_dir/test_results_${mode}.zip"
            echo "[$mode] zip → $mode_dir/test_results_${mode}.zip"
        else
            echo "[$mode] WARN test_results_latest.zip absent"
        fi
    else
        echo "[$mode] pytest absent — skip per-mode zip"
    fi

    # Extract indicators
    IND=$(_extract_indicators "$mode_dir")
    RESULTS[$mode]="$IND"
    echo "[$mode] $IND"

    # Cleanup après
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
    pkill -9 -f calypso-ipc-device 2>/dev/null || true
    pkill -9 -f "bridge\.py" 2>/dev/null || true
    kill -9 "$RUN_PID" 2>/dev/null || true
    wait "$RUN_PID" 2>/dev/null || true
    sleep 2
    echo
done

# Final report
echo "==================================="
echo " run-all.sh RESULTS"
echo "==================================="
printf "%-12s %-12s %-12s %-10s %-8s %-6s %-6s %-7s\n" \
    MODE FBSB_CONF SHUNT_DISP RR_EST LOST ERR_Q ERR_M BTS_OK
for mode in $MODES; do
    eval "${RESULTS[$mode]}"
    printf "%-12s %-12d %-12d %-10d %-8d %-6d %-6d %-7d\n" \
        "$mode" "${fbsb_conf:-0}" "${shunt_disp:-0}" "${rr_est:-0}" \
        "${lost:-0}" "${err_q:-0}" "${err_m:-0}" "${bts_ok:-0}"
done
echo
echo "Logs detaillés : $OUT_DIR/<mode>/"
echo "Pour analyser un mode : cat $OUT_DIR/shunt/qemu.log | grep dsp-shunt"
echo

# Markdown summary (utilisable pour rapport)
MD="$OUT_DIR/results.md"
{
    echo "# run-all.sh results"
    echo
    echo "Generated $(date -Iseconds) — duration ${RUN_DURATION}s per mode"
    echo
    echo "| mode | fbsb_conf | shunt_disp | rr_est | lost | err_q | err_m | bts_ok | ipc_ok |"
    echo "|------|-----------|------------|--------|------|-------|-------|--------|--------|"
    for mode in $MODES; do
        eval "${RESULTS[$mode]}"
        echo "| $mode | ${fbsb_conf:-0} | ${shunt_disp:-0} | ${rr_est:-0} | ${lost:-0} | ${err_q:-0} | ${err_m:-0} | ${bts_ok:-0} | ${ipc_ok:-0} |"
    done
} > "$MD"
echo "Markdown : $MD"

# JSON pour pytest cross-mode
JSON="$OUT_DIR/results.json"
{
    echo "{"
    echo "  \"generated\": \"$(date -Iseconds)\","
    echo "  \"duration_seconds\": $RUN_DURATION,"
    echo "  \"modes\": {"
    first=1
    for mode in $MODES; do
        eval "${RESULTS[$mode]}"
        [ $first -eq 0 ] && echo ","
        first=0
        printf '    "%s": {"fbsb_conf": %d, "shunt_latch": %d, "shunt_disp": %d, "rr_est": %d, "lost": %d, "err_q": %d, "err_m": %d, "bts_ok": %d, "ipc_ok": %d}' \
            "$mode" "${fbsb_conf:-0}" "${shunt_latch:-0}" "${shunt_disp:-0}" \
            "${rr_est:-0}" "${lost:-0}" "${err_q:-0}" "${err_m:-0}" \
            "${bts_ok:-0}" "${ipc_ok:-0}"
    done
    echo
    echo "  }"
    echo "}"
} > "$JSON"
echo "JSON     : $JSON"

# Pytest cross-mode comparison
echo
echo "===== pytest cross-mode comparison ====="
PYTEST_BIN=""
for cand in /tmp/calypso-venv/bin/pytest /opt/GSM/calypso-venv/bin/pytest \
            /usr/local/bin/pytest /usr/bin/pytest; do
    [ -x "$cand" ] && PYTEST_BIN="$cand" && break
done
if [ -z "$PYTEST_BIN" ] && python3 -c "import pytest" 2>/dev/null; then
    PYTEST_BIN="python3 -m pytest"
fi

if [ -z "$PYTEST_BIN" ]; then
    echo "[run-all] pytest absent — skip cross-mode report"
else
    TEST_FILE="$HERE/tests/test_run_all_modes.py"
    if [ -f "$TEST_FILE" ]; then
        RUN_ALL_JSON="$JSON" $PYTEST_BIN -v --tb=short --color=yes "$TEST_FILE" | tee "$OUT_DIR/pytest.log"
    else
        echo "[run-all] $TEST_FILE absent — skip cross-mode report"
    fi
fi

echo
echo "===== run-all.sh DONE ====="
echo "Out : $OUT_DIR/"
echo "  - results.md       (table markdown)"
echo "  - results.json     (machine-readable)"
echo "  - pytest.log       (cross-mode comparison)"
echo "  - <mode>/qemu.log  (per-mode raw logs)"
