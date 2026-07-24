#!/bin/bash
# test_record_replay.sh — diagnostic determinisme : record live + N replays
#
# Workflow :
#   1. RECORD : run normal avec BSP_DUMP_RX_FILE=/tmp/bsp_rx.dump
#   2. REPLAY 1/2/3 : runs avec CALYPSO_BSP_REPLAY_FILE=/tmp/bsp_rx.dump
#   3. Affiche tableau comparatif des signatures d_fb_det / SOFT-RESET-TRIG /
#      BURST-IN / etc. pour decider : determinisme restaure (= course feed) ou
#      encore nondeterministe (= autre source).
#
# Usage : ./test_record_replay.sh [duration_sec]    (defaut 30)
#
# Hypotheses :
#   - run.sh est dans le PATH ou /root/run.sh
#   - CALYPSO_NO_ATTACH=1 supporte par run.sh pour spawn en background

set -u
DUR="${1:-30}"
DUMP=/tmp/bsp_rx.dump
LOGDIR=/tmp/record_replay_logs
mkdir -p "$LOGDIR"
RUN_SH="${RUN_SH:-/opt/GSM/qemu-src/run.sh}"
QEMU_LOG="${QEMU_LOG:-/root/qemu.log}"

# ---- helper : kill toute la stack et nettoyer ----
killall_run() {
    tmux kill-session -t calypso 2>/dev/null || true
    killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
    pkill -9 -f bridge.py 2>/dev/null || true
    pkill -9 -f calypso-ipc-device 2>/dev/null || true
    pkill -9 -f irda_capture.py 2>/dev/null || true
    sleep 1
}

# ---- helper : lance un run en background avec env, attend DUR secondes ----
do_run() {
    local label="$1"; shift
    echo "===== $label : start (timeout=${DUR}s) ====="
    killall_run

    # CALYPSO_NO_ATTACH=1 : run.sh exit apres tmux session lance en bg
    CALYPSO_NO_ATTACH=1 "$@" bash "$RUN_SH" > "$LOGDIR/$label.stdout" 2>&1 &
    local launcher_pid=$!

    # Attente DUR secondes, qemu.log se remplit
    sleep "$DUR"

    # Snapshot qemu.log et kill stack
    cp -f "$QEMU_LOG" "$LOGDIR/$label.qemu.log" 2>/dev/null || true
    killall_run
    wait "$launcher_pid" 2>/dev/null || true
    echo "===== $label : done, log=$LOGDIR/$label.qemu.log ($(wc -l < $LOGDIR/$label.qemu.log 2>/dev/null || echo 0) lines) ====="
    echo
}

# ---- helper : extrait une signature compact d'un qemu.log ----
sig() {
    local log="$1"
    if [ ! -s "$log" ]; then
        echo "  (vide ou inexistant)"
        return
    fi
    local boot_redirect soft_reset_trig fb_wr_total fb_wr_dom_pc fb_wr_dom_val
    local latch_consume_dom burst_mod burst_tonal burst_silent
    local task5 task6 sync_demod fbsb_state imr_zero last_insn

    boot_redirect=$(grep -c 'SILICON-BOOT-REDIRECT' "$log")
    soft_reset_trig=$(grep -c 'SOFT-RESET-TRIG' "$log")
    fb_wr_total=$(grep -c 'FB-DET-WR' "$log")
    fb_wr_dom_pc=$(grep 'FB-DET-WR' "$log" | grep -oE 'PC=0x[0-9a-f]+' | sort | uniq -c | sort -rn | head -1 | awk '{print $2"×"$1}')
    fb_wr_dom_val=$(grep 'FB-DET-WR' "$log" | grep -oE '<- 0x[0-9a-f]+' | sort | uniq -c | sort -rn | head -1 | awk '{print $3"×"$1}')
    latch_consume_dom=$(grep 'LATCH-CONSUME' "$log" | grep -oE '= 0x[0-9a-f]+' | sort | uniq -c | sort -rn | head -1 | awk '{print $3"×"$1}')
    burst_mod=$(grep 'BURST-IN' "$log" | grep -c MODULATED)
    burst_tonal=$(grep 'BURST-IN' "$log" | grep -c TONAL_FB)
    burst_silent=$(grep 'BURST-IN' "$log" | grep -c SILENT)
    task5=$(grep 'on_dsp_task_change' "$log" | grep -c 'task=5')
    task6=$(grep 'on_dsp_task_change' "$log" | grep -c 'task=6')
    sync_demod=$(grep '\[fbsb\]' "$log" | grep -oE 'last\(snr=[0-9]+ toa=[0-9]+ ang=[0-9]+ pm=[0-9]+\)' | sort | uniq | head -3 | tr '\n' ' | ')
    fbsb_state=$(grep '\[fbsb\]' "$log" | grep -oE 'state=[A-Z_0-9]+' | sort | uniq -c | tr '\n' ' ')
    imr_zero=$(grep -c 'IMR-W \*ZERO\*' "$log")
    last_insn=$(grep -oE 'insn=[0-9]+' "$log" | grep -oE '[0-9]+' | sort -n | tail -1)

    printf "  SILICON-BOOT-REDIRECT : %s\n" "$boot_redirect"
    printf "  SOFT-RESET-TRIG       : %s\n" "$soft_reset_trig"
    printf "  FB-DET-WR total       : %s\n" "$fb_wr_total"
    printf "  FB-DET-WR dom PC      : %s\n" "$fb_wr_dom_pc"
    printf "  FB-DET-WR dom val     : %s\n" "$fb_wr_dom_val"
    printf "  LATCH-CONSUME dom     : %s\n" "$latch_consume_dom"
    printf "  BURST-IN cat (M/T/S)  : %s / %s / %s\n" "$burst_mod" "$burst_tonal" "$burst_silent"
    printf "  ARM task=5 / task=6   : %s / %s\n" "$task5" "$task6"
    printf "  fbsb state distrib    : %s\n" "$fbsb_state"
    printf "  a_sync_demod uniq     : %s\n" "$sync_demod"
    printf "  IMR-W ZERO            : %s\n" "$imr_zero"
    printf "  Last insn_count       : %s\n" "$last_insn"
}

# ---- 1. RECORD ----
rm -f "$DUMP"
do_run RECORD env BSP_DUMP_RX_FILE="$DUMP"

if [ ! -s "$DUMP" ]; then
    echo "!!! RECORD failed: dump '$DUMP' vide ou absent (taille $(stat -c%s "$DUMP" 2>/dev/null || echo 0))"
    echo "Verifier qu'au moins 1 burst a ete livre au DSP (gate BDLENA)."
    exit 1
fi
echo ">>> RECORD dump size: $(stat -c%s "$DUMP") bytes"
echo

# ---- 2. REPLAY 1/2/3 ----
for i in 1 2 3; do
    do_run REPLAY$i env CALYPSO_BSP_REPLAY_FILE="$DUMP"
done

# ---- 3. Comparaison ----
echo "##############################################"
echo "##       SIGNATURE COMPARISON TABLE         ##"
echo "##############################################"
for label in RECORD REPLAY1 REPLAY2 REPLAY3; do
    echo "----- $label -----"
    sig "$LOGDIR/$label.qemu.log"
    echo
done

echo "##############################################"
echo "## Interpretation :                          ##"
echo "## - REPLAY 1/2/3 identiques => determinisme ##"
echo "##   retabli avec feed fixe => course feed   ##"
echo "##   est la root cause                       ##"
echo "## - REPLAY 1/2/3 differents => nondeter-    ##"
echo "##   minisme ailleurs (DSP/host thread/etc)  ##"
echo "##############################################"
