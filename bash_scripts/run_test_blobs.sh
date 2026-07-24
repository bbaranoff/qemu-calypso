#!/bin/bash
# run_test_blobs.sh — Test each DARAM DSP fixture blob in isolation.
#
# Each blob lives in dsp_blobs/*.bin and is loaded via -M calypso,dsp-blob=
# at DARAM[0x100], PC=0x100. mb.c force-starts TDMA so c54x_run gets
# ticked even without ARM firmware. Blobs write known marker patterns
# we then grep for in the captured stderr.
#
# Usage:
#   ./run_test_blobs.sh                  # run all blobs in dsp_blobs/
#   ./run_test_blobs.sh dsp-deadbeef     # single blob (no extension)
#   ./run_test_blobs.sh -v               # verbose — tail each log on FAIL
#
# Logs always kept at /root/blob_test_<name>.log
# Run summary printed at end.

set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
BLOB_DIR="${HERE}/dsp_blobs"
QEMU="${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}"
RUN_SECS="${RUN_SECS:-3}"
# Real ARM firmware (layer1.highram.elf). Loaded with -kernel so the ARM
# side behaves like a normal session. The DSP-blob fixture overrides
# DARAM[0x100] + PC=0x100 after sysbus_realize so the blob still wins on
# the c54x side regardless of what the firmware tries to do.
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
VERBOSE=0
SINGLE=""

for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)
            sed -n '2,16p' "$0"
            exit 0 ;;
        *) SINGLE="$arg" ;;
    esac
done

# Per-blob expectation (extended-regex, matched against the blob's full
# captured stderr). All blobs write through data_write_locked which logs
# BLOB-WR when the addr lands in [0x2000..0x200F]. For blobs that target
# other addresses we match on the unique marker value (CAFE, BEEF, ...) or
# on the AR-WR / data_write trace.
declare -A EXPECT
EXPECT["dsp-deadbeef"]='BLOB-WR data\[0x2000\] <- 0xdead'
EXPECT["dsp-spin-counter"]='BUILD-IDENT silicon-reset'   # blob writes only to MMRs; assert DSP at least booted
EXPECT["dsp-pushpop-test"]='BLOB-WR data\[0x[0-9a-f]+\] <- 0xcafe|data\[0x[0-9a-f]+\] <- 0xbeef'
EXPECT["dsp-fbsb-dummy"]='BLOB-WR data\[0x[0-9a-f]+\] <- 0x(7080|4000|0001)'
EXPECT["dsp-ccch-si3"]='BLOB-WR data\[0x[0-9a-f]+\] <- 0xcafe|data\[0x[0-9a-f]+\] <- 0x4906'
EXPECT["dsp-ccch-si-rotate"]='data\[0x[0-9a-f]+\] <- 0x2b2b|data\[0x[0-9a-f]+\] <- 0x4906'
EXPECT["dsp-ccch-si3-handshake"]='BLOB-WR data\[0x[0-9a-f]+\] <- 0xcafe'
EXPECT["dsp-ccch-si3-handshake-v3"]='BLOB-WR data\[0x[0-9a-f]+\] <- 0xcafe'
# Regression test for 0x86 STH A,ASM,*AR2+ fix. Expected after fix:
# 3 writes of 0xbeef to data[0x2A00..0x2A02]. AR2 should walk 0x2A00→0x2A03.
EXPECT["dsp-sth-ar-test"]='BLOB-WR data\[0x2a00\] <- 0xbeef'

# Bugs we must never see during a blob test
REJECT_ANY='SP-CATASTROPHE|PANIC|Assertion'

if [ ! -x "$QEMU" ]; then
    echo "ERROR: $QEMU not found / not executable" >&2; exit 2
fi
[ -d "$BLOB_DIR" ] || { echo "ERROR: blob dir $BLOB_DIR missing"; exit 2; }
if [ ! -r "$FW_ELF" ]; then
    echo "WARNING: FW_ELF=$FW_ELF unreadable — running without -kernel" >&2
fi
if ! command -v stdbuf >/dev/null; then
    echo "ERROR: stdbuf missing (install coreutils)" >&2; exit 2
fi
echo "[init] QEMU=$QEMU"
echo "[init] FW_ELF=$FW_ELF ($([ -r "$FW_ELF" ] && echo OK || echo MISSING))"

# Kill ALL stale qemu-system-arm at start (exact binary match — without
# -x, pkill would self-match this script's bash cmdline if it ever
# contained the string "qemu-system-arm"). Stale instances hold BSP
# UDP 6702 → new QEMU silently fails → empty log.
stale_pids=$(pgrep -x qemu-system-arm 2>/dev/null)
if [ -n "$stale_pids" ]; then
    echo "[init] killing stale qemu-system-arm pids: $stale_pids"
    kill -9 $stale_pids 2>/dev/null
    sleep 0.5
fi

# Port 6702 (BSP) free?
if command -v ss >/dev/null && ss -uln 2>/dev/null | grep -q ':6702 '; then
    echo "[init] WARNING: UDP port 6702 still in use after cleanup:"
    ss -ulnp 2>/dev/null | grep ':6702 '
fi

# Ctrl-C propagation: kill any in-flight QEMU and exit cleanly.
abort() {
    echo
    echo "=== Interrupted — killing QEMU and exiting ==="
    pkill -9 -x qemu-system-arm 2>/dev/null
    exit 130
}
trap abort INT TERM

PASS=0; FAIL=0; SKIP=0; declare -a FAIL_NAMES=()

run_one() {
    local blob="$1" name log exp
    name=$(basename "$blob" .bin)
    log="/root/blob_test_${name}.log"
    exp="${EXPECT[$name]:-}"

    if [ -z "$exp" ]; then
        printf "  [SKIP] %-32s no marker defined\n" "$name"
        SKIP=$((SKIP+1)); return
    fi

    # Clean and run. Stderr is the only output we need — QEMU stdout for
    # -nographic carries serial which we ignore. NO -d unimp (creates an
    # empty file that confused the size check in v1).
    rm -f "$log"
    local kernel_arg=()
    [ -r "$FW_ELF" ] && kernel_arg=(-kernel "$FW_ELF")
    # stdbuf -e0 = unbuffered stderr → SIGTERM at timeout doesn't lose
    # the buffered tail of c54x logs (the failure mode when the script
    # reports log=0B everywhere). >>"$log" 2>&1 captures both streams
    # via append so the file always exists before timeout fires.
    : > "$log"  # truncate so size check is meaningful
    # setsid puts QEMU in its own session/process group so SIGKILL doesn't
    # cause bash to print the "Killed: ..." reaper message (bash only does
    # that for direct children in its own session). Wait via PID poll.
    setsid stdbuf -o0 -e0 "$QEMU" \
        -M "calypso,dsp-blob=${blob}" \
        "${kernel_arg[@]}" \
        -nographic \
        >>"$log" 2>&1 </dev/null &
    local qemu_pid=$!
    disown "$qemu_pid" 2>/dev/null   # remove from job table → no reap msg
    local waited=0
    while [ "$waited" -lt "$RUN_SECS" ] && kill -0 "$qemu_pid" 2>/dev/null; do
        sleep 1
        waited=$((waited+1))
    done
    if kill -0 "$qemu_pid" 2>/dev/null; then
        kill -TERM "$qemu_pid" 2>/dev/null
        sleep 0.3
        kill -9 "$qemu_pid" 2>/dev/null
    fi
    local exit_code=0  # disowned, can't wait — assume normal timeout
    # 124 = SIGTERM from timeout (expected for spinning blobs)
    # 137 = SIGKILL from kill-after (also expected)
    # 0   = QEMU exited cleanly on its own (unusual for blob test)
    # 1/2 = option / setup error → log will say why
    if [ "$VERBOSE" = 1 ]; then
        printf "         cmd=%s -M calypso,dsp-blob=%s %s -nographic (timeout=%ss)\n" \
               "$(basename "$QEMU")" "$(basename "$blob")" \
               "${kernel_arg[*]}" "$RUN_SECS" >&2
    fi
    # ↑ exit code = 124 if timeout fired (= expected for spinning blobs)

    local sz; sz=$(wc -c < "$log" 2>/dev/null || echo 0)

    # Collect interesting stats from the log regardless of verdict.
    local blob_wr_n unique_addrs unique_vals pc_hot insn_max
    blob_wr_n=$(grep -cE 'BLOB-WR' "$log" 2>/dev/null | tr -d '\n ')
    [ -z "$blob_wr_n" ] && blob_wr_n=0
    unique_addrs=$(grep -oE 'data\[0x[0-9a-f]+\]' "$log" 2>/dev/null | sort -u | tr '\n' ' ')
    unique_vals=$(grep -oE 'BLOB-WR.*<- 0x[0-9a-f]+' "$log" 2>/dev/null \
                  | grep -oE '0x[0-9a-f]+$' | sort -u | tr '\n' ' ')
    pc_hot=$(grep -oE 'PC=0x[0-9a-f]+' "$log" 2>/dev/null | sort | uniq -c \
             | sort -rn | head -3 | awk '{printf "%s(x%s) ", $2, $1}')
    insn_max=$(grep -oE 'insn=[0-9]+' "$log" 2>/dev/null | grep -oE '[0-9]+' \
               | sort -n | tail -1)

    if grep -qE "$REJECT_ANY" "$log" 2>/dev/null; then
        printf "  [FAIL] %-32s rejected: %s\n" "$name" \
               "$(grep -oE "$REJECT_ANY" "$log" | head -1)"
        FAIL=$((FAIL+1)); FAIL_NAMES+=("$name")
    elif grep -qE "$exp" "$log" 2>/dev/null; then
        local hit
        hit=$(grep -oE "$exp" "$log" | head -1)
        printf "  [PASS] %-32s %s\n" "$name" "$hit"
        PASS=$((PASS+1))
    else
        printf "  [FAIL] %-32s marker not seen (log=%sB)\n" "$name" "$sz"
        FAIL=$((FAIL+1)); FAIL_NAMES+=("$name")
    fi

    # Always dump the interesting stats so the user has context per blob.
    if [ "$blob_wr_n" -gt 0 ] 2>/dev/null; then
        printf "         writes=%s  vals=[%s]\n" "$blob_wr_n" "${unique_vals% }"
    fi
    if [ -n "$insn_max" ]; then
        printf "         insn_max=%s  hot_pcs=%s\n" "$insn_max" "${pc_hot% }"
    fi
    # If log is empty, report it loudly so we know it's a capture failure.
    if [ "$sz" = "0" ]; then
        printf "         WARNING: log is 0B — QEMU produced no stderr. "
        printf "Check kernel/blob path or stale QEMU on port 6702.\n"
    fi
    if [ "$VERBOSE" = 1 ]; then
        printf "         log=%s addrs=[%s]\n" "$log" "${unique_addrs% }"
        echo "         --- last 5 lines ---"
        tail -5 "$log" | sed 's/^/         /'
    fi

    # Make sure any background QEMU launched by this blob is dead before
    # the next one runs — UDP 6702 contention otherwise.
    pkill -9 -x qemu-system-arm 2>/dev/null
    sleep 0.2
}

echo "=== Blob test harness ==="
echo "QEMU:    $QEMU"
echo "Blobs:   $BLOB_DIR"
echo "Runtime: ${RUN_SECS}s/blob"
echo "Logs:    /root/blob_test_<name>.log (kept on disk)"
echo

if [ -n "$SINGLE" ]; then
    target="${BLOB_DIR}/${SINGLE}.bin"
    [ -r "$target" ] || { echo "ERROR: $target not found"; exit 1; }
    run_one "$target"
else
    for b in "$BLOB_DIR"/*.bin; do
        # skip ARM helper binaries — these are kernels, not DSP blobs
        case "$(basename "$b")" in
            arm-*) continue ;;
        esac
        run_one "$b"
    done
fi

echo
echo "=== Summary : ${PASS} PASS, ${FAIL} FAIL, ${SKIP} SKIP ==="
if [ "$FAIL" -gt 0 ]; then
    echo "Failed: ${FAIL_NAMES[*]}"
    echo "Inspect: ls -la /root/blob_test_*.log"
    exit 1
fi
exit 0
