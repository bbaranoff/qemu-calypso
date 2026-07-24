#!/usr/bin/env bash
# run_test.sh — bounded CPU-leak profiler for the Calypso QEMU *shunt* run.
#
#   Launches the shunt pipeline non-interactively, gathers forensic data for
#   WINDOW seconds, then KILLS everything (bounded heat). Ctrl-C also cleans up.
#
#   Usage (inside the docker osmo-operator-1):
#       bash /root/run_test.sh [WINDOW_SECONDS]      # default 10s
#   Output: /tmp/leak_diag.txt  (+ live on stdout)
#
set +e
WINDOW="${1:-10}"
OUT="${OUT:-/tmp/leak_diag.txt}"
SESS=calypso
QEMU_BIN=qemu-system-arm
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
: > "$OUT"
log(){ echo "$@" | tee -a "$OUT"; }

# Nuke EVERYTHING the radio pipeline spawns + ALL python (no legit python runs
# in this docker) + capture helpers. Does NOT touch the core-network binaries
# (osmo-msc/bsc/hlr/mgw/ggsn/sgsn/pcu/stp/sip) — those are infra, not pipeline.
kill_all_pipeline(){
  tmux kill-session -t "$SESS" 2>/dev/null
  pkill -9 -x "$QEMU_BIN"        2>/dev/null
  pkill -9 -x osmocon            2>/dev/null
  pkill -9 -f calypso-ipc-device 2>/dev/null
  pkill -9 -f osmo-trx-ipc       2>/dev/null
  pkill -9 -f osmo-bts           2>/dev/null
  pkill -9 -f "mobile -c"        2>/dev/null
  pkill -9 -x mobile             2>/dev/null
  pkill -9 -f ccch_scan          2>/dev/null
  pkill -9 -f cell_log           2>/dev/null
  pkill -9 -x python  2>/dev/null; pkill -9 -x python3 2>/dev/null
  pkill -9 -f "\.py"  2>/dev/null
  pkill -9 -x tcpdump 2>/dev/null
  pkill -9 -x socat   2>/dev/null
  sleep 1
  local left
  left=$(pgrep -af "qemu-system-arm|osmocon|calypso-ipc-device|osmo-trx-ipc|osmo-bts|mobile -c|python|\.py" 2>/dev/null | grep -v pgrep)
  [ -n "$left" ] && echo "[cleanup] WARN survivors:
$left"
  return 0
}

cleanup(){
  echo "=== CLEANUP : killing pipeline + all python ===" | tee -a "$OUT"
  kill_all_pipeline
}
trap cleanup EXIT INT TERM

log "### run_test.sh CPU-leak profiler  window=${WINDOW}s  $(date) ###"

# ---- launch shunt pipeline non-interactively ------------------------------
log "=== launching CALYPSO_MODE=shunt (non-interactive) ==="
( CALYPSO_MODE=shunt CALYPSO_L2_CLIENT=mobile FORCE_RX_DONE=1 \
    CALYPSO_NO_ATTACH=1 \
    timeout $((WINDOW+45)) /opt/GSM/qemu-src/run.sh ) >/tmp/runsh.out 2>&1 &

# ---- wait for qemu --------------------------------------------------------
PID=""
for i in $(seq 1 60); do PID=$(pgrep -x "$QEMU_BIN" | head -1); [ -n "$PID" ] && break; sleep 0.5; done
if [ -z "$PID" ]; then
  log "!! QEMU never started — run.sh tail:"; tail -30 /tmp/runsh.out | tee -a "$OUT"; exit 1
fi
log "qemu pid=$PID (after ${i}x0.5s)"

CMDLINE=$(tr '\0' ' ' < /proc/$PID/cmdline)
log "=== cmdline ==="; log "$CMDLINE"
case "$CMDLINE" in
  *icount*) log ">> icount: PRESENT" ;;
  *)        log ">> icount: ABSENT  (vCPU runs TCG free-running -> pegs a core unless guest halts via WFI)" ;;
esac
MON=$(tr '\0' '\n' < /proc/$PID/cmdline | grep -oE '/tmp/[^,]+\.sock' | grep -i mon | head -1)
QLOG=$(ls -t /root/qemu.log /tmp/qemu*.log 2>/dev/null | head -1)
log "monitor=$MON  qemu_log=$QLOG"

log "=== warming up ${WINDOW}s to steady state ==="
sleep "$WINDOW"

# ---- whole-process / per-thread CPU ---------------------------------------
log "=== top -H snapshot (per-thread %CPU) ==="
top -b -n1 -H -p "$PID" 2>/dev/null | sed -n '7,22p' | tee -a "$OUT"

log "=== per-thread CPU (jiffies/s ; 100 = one full core saturated) ==="
unset A; declare -A A
for t in /proc/$PID/task/*; do tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); A[$tid]=$(( ${r% *}+${r#* } )); done
sleep 1
HOT=""; HOTV=0
{ for t in /proc/$PID/task/*; do
    tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); n=$(( ${r% *}+${r#* } ))
    c=$(tr -d '()' < "$t/comm" 2>/dev/null); d=$(( n-${A[$tid]:-0} ))
    echo "$d $tid $c"
    [ "$d" -gt "$HOTV" ] && { HOTV=$d; HOT=$tid; }
  done; } | sort -rn | head -8 | tee -a "$OUT"
log ">> hottest thread tid=$HOT (${HOTV} jiffies/s)"

# ---- guest ARM register / PC sampling via QEMU monitor --------------------
mon_cmd(){
  if command -v socat >/dev/null 2>&1; then
    printf '%s\n' "$1" | timeout 2 socat - "UNIX-CONNECT:$MON" 2>/dev/null
  else
    timeout 2 python3 - "$MON" "$1" <<'PY'
import socket,sys,time
s=socket.socket(socket.AF_UNIX); s.connect(sys.argv[1]); time.sleep(0.15)
s.sendall((sys.argv[2]+"\n").encode()); time.sleep(0.25)
data=b""
try:
    while True:
        s.settimeout(0.3); chunk=s.recv(65536)
        if not chunk: break
        data+=chunk
except Exception: pass
print(data.decode(errors="replace"))
PY
  fi
}
log "=== full guest register dump (CPSR shows IRQ mask / CPU mode) ==="
mon_cmd "info registers" | tee -a "$OUT"
log "=== guest PC (R15/PC) samples x10 — same value repeated == tight loop ==="
for k in $(seq 1 10); do mon_cmd "info registers" | grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' | head -1; sleep 0.3; done | tee -a "$OUT"

# ---- map sampled PCs to firmware symbols ----------------------------------
log "=== addr2line of sampled PCs (firmware function the guest spins in) ==="
if [ -f "$FW_ELF" ]; then
  for pc in $(grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' "$OUT" | grep -oiE '[0-9a-f]{6,}' | sort -u); do
    sym=$(addr2line -fe "$FW_ELF" "0x$pc" 2>/dev/null | tr '\n' ' ')
    log "  PC=0x$pc -> $sym"
  done
else
  log "  (firmware ELF not found at $FW_ELF — skipping)"
fi

# ---- host backtrace of all threads (identify hot thread by LWP) -----------
log "=== gdb backtraces of all threads (hot LWP = $HOT) ==="
for s in 1 2; do
  log "----- gdb sample $s -----"
  gdb -p "$PID" -batch -nx \
      -ex 'set pagination off' \
      -ex 'thread apply all bt' 2>/dev/null \
    | grep -E '^Thread|LWP|^#[0-9]+ ' | head -70 | tee -a "$OUT"
  sleep 0.5
done

# ---- forward progress + stuck-IRQ evidence --------------------------------
log "=== qemu.log growth (is sim time / frames advancing?) ==="
a1=$(wc -l <"$QLOG" 2>/dev/null); sleep 1; a2=$(wc -l <"$QLOG" 2>/dev/null)
log "lines: ${a1:-?} -> ${a2:-?} (+$(( ${a2:-0}-${a1:-0} ))/s)"
log "=== INTH 'levels' distribution (which IRQ lines stay asserted) ==="
grep -oE 'levels=0x[0-9a-f]+' "$QLOG" 2>/dev/null | sort | uniq -c | sort -rn | head | tee -a "$OUT"
log "=== IRQ dispatch rate sample ==="
grep -E 'INTH.*dispatch' "$QLOG" 2>/dev/null | tail -3 | tee -a "$OUT"
log "=== last qemu.log lines ==="; tail -5 "$QLOG" 2>/dev/null | tee -a "$OUT"

log "### DONE — full forensic output saved to $OUT ###"
