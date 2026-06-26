#!/usr/bin/env bash
# run_diag_debug.sh — exhaustive multi-mode forensic diag for the Calypso
# QEMU pipeline. Launches each CALYPSO_MODE in sequence, samples it for a
# bounded window, KILLS it, and appends everything to ONE diag.txt that can
# be handed to "claude web" (and read here) for collaborative debugging.
#
# Heat-safe: only one mode runs at a time and each is force-killed after its
# window. Ctrl-C cleans up.
#
# Usage (inside docker osmo-operator-1):
#     bash /root/run_diag_debug.sh                 # all modes, 9s each
#     WINDOW=12 bash /root/run_diag_debug.sh        # deeper window
#     bash /root/run_diag_debug.sh shunt full       # only these modes
#
# Output: /tmp/diag.txt
#
set +e
WINDOW="${WINDOW:-9}"
OUT="${OUT:-/tmp/diag.txt}"
SESS=calypso
QEMU_BIN=qemu-system-arm
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
MODES="$*"; [ -z "$MODES" ] && MODES="full shunt shunt-ipc bridge bare"

: > "$OUT"
log(){ echo "$@" | tee -a "$OUT"; }
hr(){  printf '%s\n' "================================================================" | tee -a "$OUT"; }

# Nuke EVERYTHING the radio pipeline spawns + ALL python (no legit python runs
# in this docker) + capture helpers. Leaves the core-network binaries
# (osmo-msc/bsc/hlr/mgw/ggsn/sgsn/pcu/stp/sip) untouched — infra, not pipeline.
killall_pipeline(){
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
  [ -n "$left" ] && log "[cleanup] WARN survivors: $left"
  return 0
}
trap 'echo "[diag] interrupted — cleaning up"; killall_pipeline; exit 130' INT TERM
trap killall_pipeline EXIT

mon_cmd(){ # $1 monitor sock, $2 command
  if command -v socat >/dev/null 2>&1; then
    printf '%s\n' "$2" | timeout 2 socat - "UNIX-CONNECT:$1" 2>/dev/null
  else
    timeout 2 python3 - "$1" "$2" <<'PY'
import socket,sys,time
s=socket.socket(socket.AF_UNIX); s.connect(sys.argv[1]); time.sleep(0.15)
s.sendall((sys.argv[2]+"\n").encode()); time.sleep(0.25); d=b""
try:
  while True:
    s.settimeout(0.3); c=s.recv(65536)
    if not c: break
    d+=c
except Exception: pass
print(d.decode(errors="replace"))
PY
  fi
}

sample_mode(){
  MODE="$1"
  hr; log "########## MODE = $MODE   window=${WINDOW}s   $(date) ##########"; hr
  killall_pipeline

  # Purge leftover logs run.sh does NOT clean (e.g. l2_client.log) so tails
  # can't show stale data from a previous run. Stamp the mode start so we can
  # flag any log that wasn't refreshed this run.
  rm -f /tmp/l2_client.log /tmp/runsh.$MODE.out 2>/dev/null
  MODE_START=$(date +%s)

  log "[diag] launching CALYPSO_MODE=$MODE"
  ( CALYPSO_MODE="$MODE" CALYPSO_L2_CLIENT=mobile FORCE_RX_DONE=1 \
      CALYPSO_NO_ATTACH=1 \
      timeout $((WINDOW+50)) /opt/GSM/qemu-src/run.sh ) >/tmp/runsh.$MODE.out 2>&1 &

  PID=""
  for i in $(seq 1 80); do PID=$(pgrep -x "$QEMU_BIN" | head -1); [ -n "$PID" ] && break; sleep 0.5; done
  if [ -z "$PID" ]; then
    log "!! QEMU never started for mode=$MODE. run.sh tail:"
    tail -25 /tmp/runsh.$MODE.out | tee -a "$OUT"
    killall_pipeline; return
  fi
  log "qemu pid=$PID (after ${i}x0.5s)"

  CMDLINE=$(tr '\0' ' ' < /proc/$PID/cmdline)
  log "--- cmdline ---"; log "$CMDLINE"
  case "$CMDLINE" in *icount*) log ">> icount: PRESENT" ;; *) log ">> icount: ABSENT" ;; esac
  MON=$(tr '\0' '\n' < /proc/$PID/cmdline | grep -oE '/tmp/[^,]+\.sock' | grep -i mon | head -1)
  QLOG=$(ls -t /root/qemu.log /tmp/qemu*.log 2>/dev/null | head -1)
  log ">> monitor=$MON  qemu_log=$QLOG"

  log "[diag] warming ${WINDOW}s ..."; sleep "$WINDOW"

  log "--- cpu-idle governor + DSP reg-mode banners (from qemu.log) ---"
  grep -E '\[cpu-idle\]|reg_mode|DSP_REG_MODE|tdma_timer clock' "$QLOG" 2>/dev/null | head -5 | tee -a "$OUT"

  log "--- top -H (per-thread %CPU) ---"
  top -b -n1 -H -p "$PID" 2>/dev/null | sed -n '7,20p' | tee -a "$OUT"

  log "--- per-thread CPU (jiffies/s ; 100 = one full core) ---"
  unset A; declare -A A
  for t in /proc/$PID/task/*; do tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); A[$tid]=$(( ${r% *}+${r#* } )); done
  sleep 1
  for t in /proc/$PID/task/*; do tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); n=$(( ${r% *}+${r#* } )); c=$(tr -d '()' < "$t/comm" 2>/dev/null); echo "$(( n-${A[$tid]:-0} )) tid=$tid $c"; done | sort -rn | head -6 | tee -a "$OUT"

  log "--- guest register dump (CPSR = mode + IRQ/FIQ mask) ---"
  mon_cmd "$MON" "info registers" | grep -E 'R0|R1|PSR|R15|PC=' | tee -a "$OUT"
  log "--- guest PC samples x10 (same value = tight loop) ---"
  for k in $(seq 1 10); do mon_cmd "$MON" "info registers" | grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' | head -1; sleep 0.3; done | tee -a "$OUT"

  log "--- addr2line of sampled PCs (firmware fn the guest is in) ---"
  if [ -f "$FW_ELF" ]; then
    for pc in $(grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' "$OUT" | grep -oiE '[0-9a-f]{6,}' | sort -u | tail -20); do
      log "  PC=0x$pc -> $(addr2line -fe "$FW_ELF" "0x$pc" 2>/dev/null | tr '\n' ' ')"
    done
  fi

  log "--- INTH 'levels' distribution (stuck IRQ lines) ---"
  grep -oE 'levels=0x[0-9a-f]+' "$QLOG" 2>/dev/null | sort | uniq -c | sort -rn | head | tee -a "$OUT"
  log "--- IRQ dispatch tail ---"
  grep -E 'INTH.*dispatch' "$QLOG" 2>/dev/null | tail -2 | tee -a "$OUT"

  log "--- pipeline markers (progress / blockers) ---"
  for pat in 'dsp-shunt' 'FBSB_CONF' 'NO_CELL' 'L1CTL_DATA_IND' 'data_ind' 'LOST' 'underrun' 'fifo' 'CCCH' 'BCCH' 'cell'; do
    n=$(grep -ic "$pat" "$QLOG" 2>/dev/null)
    [ "${n:-0}" -gt 0 ] && echo "  $pat: $n" | tee -a "$OUT"
  done
  log "--- ARFCN / cell-sync (expect ${ARFCN:-514}) ---"
  grep -hiE "Sync to ARFCN|Found signal|Scanning frequencies|Channel synched" /tmp/mobile.log 2>/dev/null | tail -4 | sed 's/^/    /' | tee -a "$OUT"

  log "--- qemu.log growth (sim advancing?) ---"
  a1=$(wc -l <"$QLOG" 2>/dev/null); sleep 1; a2=$(wc -l <"$QLOG" 2>/dev/null)
  log "  lines: ${a1:-?} -> ${a2:-?} (+$(( ${a2:-0}-${a1:-0} ))/s)"
  log "--- last 6 qemu.log lines ---"; tail -6 "$QLOG" 2>/dev/null | tee -a "$OUT"

  # L2 app log depends on the client: mobile->mobile.log, scanners->l2_client.log
  log "--- other service logs (freshness-checked tails) ---"
  for f in /tmp/osmocon.log /tmp/calypso-ipc-device.log /tmp/osmo-trx-ipc.log /tmp/bts.log /tmp/mobile.log /tmp/l2_client.log /tmp/runsh.$MODE.out; do
    [ -s "$f" ] || continue
    mt=$(stat -c %Y "$f" 2>/dev/null)
    if [ "${mt:-0}" -lt "${MODE_START:-0}" ]; then
      log "  >>> $f  [STALE — not written this run (mtime<start), skipped]"
    else
      log "  >>> $f (tail 5)"; tail -5 "$f" 2>/dev/null | sed 's/^/      /' | tee -a "$OUT"
    fi
  done

  log "[diag] killing mode=$MODE"
  killall_pipeline
  log ""
}

log "#################################################################"
log "#   Calypso QEMU exhaustive multi-mode diag"
log "#   date=$(date)   host=$(uname -a)"
log "#   modes=[$MODES]   window=${WINDOW}s each"
log "#   qemu=$(ls -l /opt/GSM/qemu-src/build/$QEMU_BIN 2>/dev/null | awk '{print $6,$7,$8}')"
log "#################################################################"
log ""

for m in $MODES; do sample_mode "$m"; done

hr
log "### ALL MODES DONE — full diag in $OUT  ($(wc -l <"$OUT") lines) ###"
hr
