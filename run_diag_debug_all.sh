#!/usr/bin/env bash
# run_diag_debug_all.sh — exhaustive COMBO-matrix forensic diag for the
# Calypso QEMU pipeline. Sweeps  MODE × DSP_REG_MODE × CPU_IDLE  and appends
# everything to ONE /tmp/diag_all.txt for hand-off to claude web (and me).
#
# Each combo: clean launch → bounded sampling → force-kill EVERYTHING
# (pipeline + all python). Heat-safe: one combo at a time. Ctrl-C cleans up.
#
# DSP_REG_MODE (c54x|bin|hybrid) only affects modes that actually emulate the
# c54x (full/bridge/bare); shunt/shunt-ipc bypass it → swept once (reg=n/a).
#
# DÉFAUT = la config qu'on veut faire marcher : full / mobile / c54x (1 combo,
# sondes décisives bas volume). Le reste (bin, scanners, bridge/bare, FORCE_TOA,
# icount off, governor off) est OPT-IN via les axes. Presets :
#   # contraste reg-mode :   REGMODES="c54x bin"
#   # cause A/B/C :          TOAMODES="off 23" DEBUG_TOKENS="TRX,WATCH-READ"
#   # icount on/off :        ICOUNTMODES="auto off"
#   # governor on/off :      IDLEMODES="1 0"
#   # sweep non-régression : MODES="full bridge bare" REGMODES="c54x bin"
#
set +e
WINDOW="${WINDOW:-15}"           # 7s ratait le dispatch FB (~+11-13s en c54x)
OUT="${OUT:-/tmp/diag_all.txt}"
SESS=calypso
QEMU_BIN=qemu-system-arm
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
MODES="${MODES:-full}"                       # défaut : chemin réel E2E
REGMODES="${REGMODES:-c54x}"                 # défaut : mode sain (vrai FB)
IDLEMODES="${IDLEMODES:-1}"                  # set "1 0" to compare governor on/off
L2CLIENTS="${L2CLIENTS:-mobile}"             # seul client qui drive un FBSB ciblé 514
ICOUNTMODES="${ICOUNTMODES:-auto}"           # CALYPSO_ICOUNT ; set "auto off" pour tester sans icount
AFCHZ="${AFCHZ:-}"                            # oracle AFC amont : CALYPSO_TWL3025_AFC_HZ=N (force offset). vide=off

# ---- CC-web requested knobs --------------------------------------------------
# DRAIN_PTYS=1   : drain the SPARE serial pty (the one osmocon does NOT hold) to
#                  /dev/null so a verbose firmware console can't backpressure the
#                  UART TX FIFO. Safe (does not steal osmocon's link). [default 1]
# DRAIN_ALL_PTYS=1: HARD test — drain BOTH ptys incl. osmocon's. Breaks L1CTL,
#                  use ONLY to isolate the uart_reg_read spin hypothesis. [default 0]
# TOAMODES=…     : AXE séparé FORCE_TOA. Valeurs balayées : "off" (pas de force)
#                  et/ou un TOA, ex. "off 23" → 2 sous-runs par combo pour le
#                  test cause-C (d_fb_det change-t-il quand on force le FB ?).
#                  [default "off"]. "23" => CALYPSO_FORCE_TOA=23.
# DEBUG_TOKENS=…  : passed as CALYPSO_DEBUG (comma list) to light up probes.
#                  Pour le test cause-A (ARM lit-il d_fb_det ?) : "TRX,WATCH-READ".
#                  [default off]
DRAIN_PTYS="${DRAIN_PTYS:-1}"
DRAIN_ALL_PTYS="${DRAIN_ALL_PTYS:-0}"
TOAMODES="${TOAMODES:-off}"      # axe FORCE_TOA séparé : "off" et/ou "23" (ex: TOAMODES="off 23")
DEBUG_TOKENS="${DEBUG_TOKENS:-TRX,AFC-APPLY,WATCH-READ,D_TASK_MD-RD}"   # AFC amont + handoff + ARM-read
ARFCN="${ARFCN:-514}"            # mobile is sticked to ARFCN 514 (DCS) — cfg ~/.osmocom/bb/mobile_group1.cfg
DRAIN_PIDS=""

: > "$OUT"
log(){ echo "$@" | tee -a "$OUT"; }
hr(){  printf '%s\n' "----------------------------------------------------------------" | tee -a "$OUT"; }

is_shunt(){ case "$1" in shunt|shunt-ipc) return 0;; *) return 1;; esac; }

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
  [ -n "$DRAIN_PIDS" ] && kill -9 $DRAIN_PIDS 2>/dev/null
  pkill -9 -f "cat /dev/pts" 2>/dev/null
  DRAIN_PIDS=""
  sleep 1
  local left
  left=$(pgrep -af "qemu-system-arm|osmocon|calypso-ipc-device|osmo-trx-ipc|osmo-bts|mobile -c|python|\.py" 2>/dev/null | grep -v pgrep)
  [ -n "$left" ] && log "[cleanup] WARN survivors: $left"
  return 0
}
trap 'echo "[diag] interrupted"; killall_pipeline; exit 130' INT TERM
trap killall_pipeline EXIT

mon_cmd(){
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

sample_combo(){
  local MODE="$1" L2="$2" REG="$3" IDLE="$4" TOA="$5" ICOUNT="$6"
  # L2 app log: mobile -> mobile.log ; ccch_scan/cell_log -> l2_client.log
  local L2LOG=/tmp/mobile.log; [ "$L2" != "mobile" ] && L2LOG=/tmp/l2_client.log
  hr
  log "########## MODE=$MODE  L2=$L2  DSP_REG_MODE=$REG  CPU_IDLE=$IDLE  FORCE_TOA=$TOA  ICOUNT=$ICOUNT  win=${WINDOW}s  $(date +%H:%M:%S) ##########"
  hr
  killall_pipeline
  rm -f /tmp/l2_client.log /tmp/runsh.combo.out 2>/dev/null
  local START; START=$(date +%s)

  # Build env. REG is "n/a" for shunt modes (c54x bypassed).
  local REGENV="" TOAENV="" DBGENV=""
  [ "$REG" != "n/a" ]    && REGENV="CALYPSO_DSP_REG_MODE=$REG"
  [ "$TOA" != "off" ] && [ -n "$TOA" ] && TOAENV="CALYPSO_FORCE_TOA=$TOA"
  [ -n "$DEBUG_TOKENS" ] && DBGENV="CALYPSO_DEBUG=$DEBUG_TOKENS"
  local AFCENV=""; [ -n "$AFCHZ" ] && AFCENV="CALYPSO_TWL3025_AFC_HZ=$AFCHZ"

  log "[diag] launch: CALYPSO_MODE=$MODE CALYPSO_L2_CLIENT=$L2 (ARFCN=$ARFCN) $REGENV CALYPSO_ICOUNT=$ICOUNT CALYPSO_CPU_IDLE=$IDLE $TOAENV $DBGENV"
  ( env CALYPSO_MODE="$MODE" CALYPSO_L2_CLIENT="$L2" FORCE_RX_DONE=1 \
        CALYPSO_CCCH_ARFCN="$ARFCN" CALYPSO_NO_ATTACH=1 CALYPSO_ICOUNT="$ICOUNT" \
        CALYPSO_CPU_IDLE="$IDLE" $REGENV $TOAENV $DBGENV $AFCENV \
        timeout $((WINDOW+50)) /opt/GSM/qemu-src/run.sh ) >/tmp/runsh.combo.out 2>&1 &

  local PID="" i
  for i in $(seq 1 80); do PID=$(pgrep -x "$QEMU_BIN" | head -1); [ -n "$PID" ] && break; sleep 0.5; done
  if [ -z "$PID" ]; then
    log "!! QEMU never started. run.sh tail:"; tail -20 /tmp/runsh.combo.out | tee -a "$OUT"
    killall_pipeline; log ""; return
  fi
  log "qemu pid=$PID (after ${i}x0.5s)"
  local CMDLINE; CMDLINE=$(tr '\0' ' ' < /proc/$PID/cmdline)
  case "$CMDLINE" in *icount*) log ">> icount: PRESENT" ;; *) log ">> icount: ABSENT" ;; esac
  local MON QLOG
  MON=$(tr '\0' '\n' < /proc/$PID/cmdline | grep -oE '/tmp/[^,]+\.sock' | grep -i mon | head -1)
  QLOG=$(ls -t /root/qemu.log /tmp/qemu*.log 2>/dev/null | head -1)

  # ---- pty backpressure test (CC-web #1) ----
  # Discover both serial ptys from this run's qemu.log; report who holds them;
  # drain the spare (and optionally both) so a verbose console can't wedge TX.
  sleep 1
  local P0 P1
  P0=$(grep -oE 'redirected to /dev/pts/[0-9]+ \(label serial0\)' "$QLOG" 2>/dev/null | grep -oE '/dev/pts/[0-9]+' | head -1)
  P1=$(grep -oE 'redirected to /dev/pts/[0-9]+ \(label serial1\)' "$QLOG" 2>/dev/null | grep -oE '/dev/pts/[0-9]+' | head -1)
  log "--- serial ptys: serial0(modem/osmocon)=$P0  serial1(irda)=$P1 ---"
  for p in "$P0" "$P1"; do
    [ -n "$p" ] && log "    $p readers: $(fuser "$p" 2>/dev/null | tr '\n' ' ')"
  done
  if [ "$DRAIN_ALL_PTYS" = "1" ]; then
    log "    [drain] HARD: draining BOTH ptys (osmocon link will break)"
    for p in "$P0" "$P1"; do [ -n "$p" ] && { cat "$p" >/dev/null 2>&1 & DRAIN_PIDS="$DRAIN_PIDS $!"; }; done
  elif [ "$DRAIN_PTYS" = "1" ]; then
    log "    [drain] spare pty serial1=$P1 -> /dev/null"
    [ -n "$P1" ] && { cat "$P1" >/dev/null 2>&1 & DRAIN_PIDS="$DRAIN_PIDS $!"; }
  fi

  sleep "$WINDOW"

  log "--- governor + reg-mode banners (qemu.log) ---"
  grep -E '\[cpu-idle\]|reg_mode|DSP_REG_MODE|tdma_timer clock' "$QLOG" 2>/dev/null | head -4 | tee -a "$OUT"

  log "--- per-thread CPU (jiffies/s ; 100 = one full core) ---"
  unset A; declare -A A
  for t in /proc/$PID/task/*; do tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); A[$tid]=$(( ${r% *}+${r#* } )); done
  sleep 1
  for t in /proc/$PID/task/*; do tid=${t##*/}; r=$(cut -d' ' -f14,15 "$t/stat" 2>/dev/null); n=$(( ${r% *}+${r#* } )); c=$(tr -d '()' < "$t/comm" 2>/dev/null); echo "$(( n-${A[$tid]:-0} )) tid=$tid $c"; done | sort -rn | head -5 | tee -a "$OUT"

  log "--- guest CPSR + PC samples x8 ---"
  mon_cmd "$MON" "info registers" | grep -oE 'PSR=[0-9a-f]+ [^ ]* [^ ]* [a-z0-9]+' | head -1 | tee -a "$OUT"
  for k in $(seq 1 8); do mon_cmd "$MON" "info registers" | grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' | head -1; sleep 0.25; done | tee -a "$OUT"

  log "--- addr2line of sampled PCs ---"
  if [ -f "$FW_ELF" ]; then
    for pc in $(grep -oiE 'R15=[0-9a-f]+|PC=[0-9a-f]+' "$OUT" | grep -oiE '[0-9a-f]{6,}' | sort -u | tail -12); do
      log "  PC=0x$pc -> $(addr2line -fe "$FW_ELF" "0x$pc" 2>/dev/null | tr '\n' ' ')"
    done
  fi

  log "--- INTH levels (stuck IRQ) + IRQ tail ---"
  grep -oE 'levels=0x[0-9a-f]+' "$QLOG" 2>/dev/null | sort | uniq -c | sort -rn | head -5 | tee -a "$OUT"
  grep -E 'INTH.*dispatch' "$QLOG" 2>/dev/null | tail -1 | tee -a "$OUT"

  log "--- pipeline markers (target ARFCN=$ARFCN) ---"
  for pat in 'dsp-shunt' 'FBSB_CONF' 'NO_CELL' 'L1CTL_DATA_IND' 'LOST' 'underrun' 'self-CALA' '28868' 'BCCH'; do
    n=$(grep -ic "$pat" "$QLOG" "$L2LOG" 2>/dev/null | awk -F: '{s+=$2}END{print s}')
    [ "${n:-0}" -gt 0 ] && echo "  $pat: $n" | tee -a "$OUT"
  done
  log "--- ARFCN / cell-sync (expect $ARFCN) ---"
  grep -hiE "Sync to ARFCN|Found signal|Scanning frequencies|Channel synched" "$L2LOG" 2>/dev/null | tail -4 | sed 's/^/    /' | tee -a "$OUT"
  if grep -qiE "ARFCN[ =]?$ARFCN" "$L2LOG" 2>/dev/null; then log "    >> OK: $ARFCN referenced"; else log "    >> WARN: ARFCN $ARFCN not seen in $L2LOG"; fi

  # ---- CC-web probe captures (only populated when DEBUG_TOKENS / FORCE_TOA set) ----
  log "--- SP-LEDGER trend (net_words vs insn — linear=real leak, flat=artifact) ---"
  grep -E 'SP-LEDGER' "$QLOG" 2>/dev/null | tail -8 | sed 's/^/    /' | tee -a "$OUT"
  log "--- ARM->DSP task handoff (manip#1: real DSP latch la tâche FB ?) ---"
  log "    [needs DEBUG_TOKENS=D_TASK_MD-RD,D_TASK_MD_ALL ; sinon vide]"
  log "    ARM writes (d_task_md):"; grep -hE 'ARM TASK WR|D_TASK_MD_ALL|DMA proof.*task_md' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/      /' | tee -a "$OUT"
  log "    DSP reads (d_task_md @0x0804/0x0818):"; grep -hE 'D_TASK_MD-RD' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/      /' | tee -a "$OUT"
  log "    task-change (fbsb orchestration):"; grep -hE 'on_dsp_task_change' "$QLOG" 2>/dev/null | tail -3 | sed 's/^/      /' | tee -a "$OUT"
  log "--- CAUSE-A : ARM lit-il d_fb_det ? (needs DEBUG_TOKENS=TRX) ---"
  log "    ARM RD (d_fb_det/d_fb_mode/a_sync @0x01F0+):"; grep -hE 'ARM RD (d_fb_det|d_fb_mode|a_sync)' "$QLOG" 2>/dev/null | tail -6 | sed 's/^/      /' | tee -a "$OUT"
  log "    DSP DETECTOR-RUN / WATCH-READ (DSP écrit d_fb_det):"; grep -hE 'DETECTOR-RUN|WATCH-READ d_fb_det' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/      /' | tee -a "$OUT"
  log "--- IPC->BSP SOURCE (osmo-trx-ipc/UDP 5702) — needs DEBUG_TOKENS=BSP ---"
  log "    passthrough on? (sinon BSP génère un ton synthétique fs/4=67.7kHz, IPC ignoré):"; grep -hE 'IQ_PASSTHROUGH=' "$QLOG" 2>/dev/null | tail -1 | sed 's/^/      /' | tee -a "$OUT"
  log "    RXSZ (paquets UDP reçus de l IPC + taille):"; grep -hE 'RXSZ' "$QLOG" 2>/dev/null | tail -3 | sed 's/^/      /' | tee -a "$OUT"
  log "    IQ passthrough iq[0..3] (samples VENANT de l IPC — varient=vraie waveform):"; grep -hE 'IQ passthrough' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/      /' | tee -a "$OUT"
  log "--- I/Q AMONT (puissance) — needs DEBUG_TOKENS=BSP-DELIVER,IQ-READ ---"
  log "    BSP-DELIVER (samples livrés BSP->DSP/frame):"; grep -hE 'BSP-DELIVER' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/      /' | tee -a "$OUT"
  log "    IQ-READ (valeurs I/Q lues par le DSP — 0/tiny = pas de puissance):"; grep -hE 'IQ-READ' "$QLOG" 2>/dev/null | tail -5 | sed 's/^/      /' | tee -a "$OUT"
  # Interprétation I/Q : amplitude ET variance (un DC constant = pas de forme
  # d'onde, ≠ vraie FCCH qui doit varier sample-à-sample), + fenêtre temporelle.
  {
    local iqv mx=0 nz=0 tot=0 v a distinct fi la
    iqv=$(grep -hE 'IQ-READ' "$QLOG" 2>/dev/null | grep -oiE 'val=0x[0-9a-f]+' | sed 's/[vV][aA][lL]=0[xX]//' | tail -300)
    if [ -z "$iqv" ]; then
      log "    >> INTERPRÉTATION I/Q : aucune ligne IQ-READ (token absent, ou le DSP ne lit jamais le buffer corrélateur)"
    else
      for h in $iqv; do
        v=$((16#$h)); [ "$v" -ge 32768 ] && v=$((v-65536)); a=${v#-}
        tot=$((tot+1)); [ "$a" -gt 0 ] && nz=$((nz+1)); [ "$a" -gt "$mx" ] && mx=$a
      done
      distinct=$(printf '%s\n' $iqv | sort -u | wc -l | tr -d ' ')
      local addrs; addrs=$(grep -hE 'IQ-READ' "$QLOG" 2>/dev/null | grep -oiE 'addr=0x[0-9a-f]+' | sort -u | wc -l | tr -d ' ')
      # fenêtre insn des samples (boot ~3M vs détection ~70M+)
      fi=$(grep -hE 'IQ-READ' "$QLOG" 2>/dev/null | grep -oiE 'insn=[0-9]+' | head -1 | sed 's/insn=//')
      la=$(grep -hE 'IQ-READ' "$QLOG" 2>/dev/null | grep -oiE 'insn=[0-9]+' | tail -1 | sed 's/insn=//')
      log "    >> INTERPRÉTATION I/Q : n=$tot non-nuls=$nz |max|=$mx valeurs_distinctes=$distinct adresses_distinctes=$addrs (insn ${fi:-?}..${la:-?}, int16 ±32767)"
      if [ "${addrs:-0}" -le 2 ]; then
        log "       ⚠ sondes concentrées sur $addrs adresse(s) → le DSP relit le même cell (début de passe, AR3 figé) ; on ne peut PAS juger la forme d'onde sur la variance des valeurs ici. Regarder data[] sur une plage d'adresses (0x2a00..0x2a52)."
      fi
      if   [ "$mx" -lt 16 ];  then
        log "       VERDICT: I/Q PLAT (~0) → AUCUNE puissance au DSP → perte device→BSP→DSP (livraison/c54x_bsp_load)"
      elif [ "${addrs:-0}" -ge 3 ] && [ "${distinct:-0}" -ge 3 ]; then
        log "       VERDICT: I/Q VARIE selon l'adresse ($distinct val / $addrs addr, |max|=$mx) → vraie forme d'onde dans data[] → livraison OK → suspecter CORRÉLATEUR/PM/seuil (aval)"
      else
        log "       VERDICT: non concluant ($distinct val / $addrs addr) — étendre la couverture d'adresses (la valeur seule ne suffit pas si AR3 figé)"
      fi
      [ -n "$fi" ] && [ "$la" -gt 50000000 ] 2>/dev/null && log "       ✓ couverture inclut l'instant détection (insn jusqu'à $la)"
      [ -n "$fi" ] && [ "$fi" -lt 10000000 ] && [ "${la:-0}" -lt 50000000 ] && log "       ⚠ samples seulement au BOOT (insn<50M) = stale ; relancer pour atteindre la détection."
    fi
  }
  log "--- AFC AMONT (convergence ?) — needs DEBUG_TOKENS=TRX,AFC-APPLY ---"
  log "    AFC WR (firmware -> dac/hz):"; grep -hE 'AFC WR' "$QLOG" 2>/dev/null | tail -5 | sed 's/^/      /' | tee -a "$OUT"
  log "    AFC-APPLY (offset Hz réellement appliqué/burst ; >±20kHz tue la FCCH):"; grep -hE 'AFC-APPLY' "$QLOG" 2>/dev/null | tail -5 | sed 's/^/      /' | tee -a "$OUT"
  log "    d_fb_mode (FBMODE-WR / DETECTOR-RUN — fenêtre détecteur):"; grep -hE 'FBMODE-WR|DETECTOR-RUN' "$QLOG" 2>/dev/null | tail -3 | sed 's/^/      /' | tee -a "$OUT"
  log "    rxlev plancher ? (mobile):"; grep -hE 'Found signal|rxlev' "$L2LOG" 2>/dev/null | tail -3 | sed 's/^/      /' | tee -a "$OUT"
  log "--- d_fb_det ARM/DSP reads + writes (shunt-ipc handshake) ---"
  grep -hE 'WATCH-READ d_fb_det|FBDET RD|FBWATCH-DET|d_fb_det' "$QLOG" 2>/dev/null | tail -6 | sed 's/^/    /' | tee -a "$OUT"
  log "--- shunt LATCH page read/write (page aliasing check) ---"
  grep -E 'LATCH page=|DISPATCH ALLC' "$QLOG" 2>/dev/null | tail -4 | sed 's/^/    /' | tee -a "$OUT"
  log "--- INTH IRQ7 dispatch (entries; total across IRQs) ---"
  grep -E 'INTH.*dispatch' "$QLOG" 2>/dev/null | tail -2 | sed 's/^/    /' | tee -a "$OUT"

  log "--- log growth + tails (freshness-checked) ---"
  local a1 a2; a1=$(wc -l <"$QLOG" 2>/dev/null); sleep 1; a2=$(wc -l <"$QLOG" 2>/dev/null)
  log "  qemu.log: ${a1:-?} -> ${a2:-?} (+$(( ${a2:-0}-${a1:-0} ))/s)"
  for f in "$L2LOG" /tmp/osmocon.log /tmp/calypso-ipc-device.log /tmp/runsh.combo.out; do
    [ -s "$f" ] || continue
    local mt; mt=$(stat -c %Y "$f" 2>/dev/null)
    if [ "${mt:-0}" -lt "$START" ]; then log "  >>> $f [STALE skipped]"
    else log "  >>> $f (tail 3)"; tail -3 "$f" 2>/dev/null | sed 's/^/      /' | tee -a "$OUT"; fi
  done

  killall_pipeline
  log ""
}

log "#################################################################"
log "#   Calypso QEMU exhaustive COMBO-matrix diag"
log "#   date=$(date)"
log "#   MODES=[$MODES]  L2CLIENTS=[$L2CLIENTS]  REGMODES=[$REGMODES]  IDLEMODES=[$IDLEMODES]  TOAMODES=[$TOAMODES]  ICOUNTMODES=[$ICOUNTMODES]  win=${WINDOW}s"
log "#   ARFCN=$ARFCN  DRAIN_PTYS=$DRAIN_PTYS  DRAIN_ALL_PTYS=$DRAIN_ALL_PTYS  DEBUG_TOKENS=[${DEBUG_TOKENS:-off}]"
log "#   qemu=$(ls -l /opt/GSM/qemu-src/build/$QEMU_BIN 2>/dev/null | awk '{print $6,$7,$8}')"
log "#################################################################"
log ""

N=0
for m in $MODES; do
  for l2 in $L2CLIENTS; do
    for idle in $IDLEMODES; do
      for toa in $TOAMODES; do
       for ic in $ICOUNTMODES; do
        # reg-sweep only for the functional mobile path on DSP-real modes;
        # scanners (ccch_scan/cell_log) + shunt run a single reg to limit heat.
        if is_shunt "$m"; then
          sample_combo "$m" "$l2" "n/a" "$idle" "$toa" "$ic"; N=$((N+1))
        elif [ "$l2" = "mobile" ]; then
          for reg in $REGMODES; do sample_combo "$m" "$l2" "$reg" "$idle" "$toa" "$ic"; N=$((N+1)); done
        else
          sample_combo "$m" "$l2" "bin" "$idle" "$toa" "$ic"; N=$((N+1))
        fi
       done
      done
    done
  done
done

hr
log "### DONE — $N combos — full matrix in $OUT ($(wc -l <"$OUT") lines) ###"
hr
