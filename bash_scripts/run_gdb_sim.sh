#!/bin/bash
# run_gdb_sim.sh — lance run.sh (icount=auto) puis attache gdb-multiarch
# avec breakpoints/watchpoints sur le path SIM IRQ -> rxDoneFlag.
#
# Sortie : session gdb interactive en foreground. Ctrl-C pour interrompre,
# 'quit' pour sortir. Le pipeline tourne dans tmux session 'calypso'
# (attache via : tmux a -t calypso).
#
# Logique des sondes :
#   - BP @ calypso_sim_powerup entry (0x00822f54)  : confirme firmware arrive
#   - BP @ sim_irq_handler entry    (0x008228b0)  : confirme IRQ delivered
#   - BP @ strne rxDoneFlag         (0x008228c4)  : log r1/r2/r3/cpsr + Z flag
#   - Watch on rxDoneFlag           (0x008302d4)  : capture tout write reel
#
# Sortie attendue (par hit) :
#   ENTER calypso_sim_powerup ...
#   BP handler ENTRY ...
#   BP strne ... Z=0 EXEC -> *0x008302d4 = 0x1
#   WATCH rxDoneFlag old=0x0 new=0x1 PC=0x...

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GDB_PORT="${CALYPSO_GDB_PORT:-1234}"
GDB_HOST=localhost
ELF=/opt/GSM/firmware/board/compal_e88/layer1.highram.elf
RUN_LOG=/tmp/run_gdb_sim.log
GDB_SCRIPT=/tmp/sim_probes.gdb

# Self-wrap via script(1) au 1er appel : capture TOUT (run.sh + nos msgs +
# gdb interactif) dans $RUN_LOG TOUT EN preservant le TTY. Sans ça, le
# `exec >>(tee)` casse le TTY -> gdb perd Ctrl-C / interactivite.
# script -q (quiet, pas de header), -f (flush apres chaque write, live tail).
if [ -z "${_RUN_GDB_SIM_WRAPPED:-}" ]; then
    export _RUN_GDB_SIM_WRAPPED=1
    exec script -q -f -c "$0 $*" "$RUN_LOG"
fi

# ---- 1) Kill ancien run + ancien gdb (best-effort) ----
# Kill TOUT vieux gdb-multiarch d'abord — QEMU gdb-stub n'accepte qu'un seul
# client à la fois, un vieux gdb attaché bloque le nouveau silencieusement.
if pgrep -f "gdb-multiarch.*:1234" >/dev/null 2>&1; then
  echo "[gdb-sim] Ancien gdb-multiarch détecté — kill"
  pkill -9 -f "gdb-multiarch.*:1234" 2>/dev/null || true
  sleep 1
fi
if pgrep -f "qemu-system-arm.*calypso" >/dev/null 2>&1; then
  echo "[gdb-sim] Ancien QEMU détecté — kill via tmux + pkill"
  tmux kill-session -t calypso 2>/dev/null || true
  pkill -f "qemu-system-arm.*calypso" 2>/dev/null || true
  pkill -f "osmocon" 2>/dev/null || true
  pkill -f "bridge.py" 2>/dev/null || true
  pkill -f "osmo-bts-trx" 2>/dev/null || true
  sleep 2
fi

# ---- 2) Lance run.sh avec icount=auto ----
# CALYPSO_NO_ATTACH=1 : run.sh exit 0 au lieu de `exec tmux attach` final
# (L1593-1599). Sinon le script bloque dans tmux et gdb ne fire jamais.
# CALYPSO_L2_CLIENT : skip le prompt "L2 client selection" (L1161 read -p).
export CALYPSO_ICOUNT=auto
export CALYPSO_NO_ATTACH=1
export CALYPSO_L2_CLIENT="${CALYPSO_L2_CLIENT:-mobile}"
# QEMU lancé HALTED (-S) : gdb attache, set BPs, 'c' pour démarrer firmware.
# Sans ça, firmware déjà en busy-poll calypso_sim_powerup quand gdb attache,
# BPs (sim_powerup entry, sim_irq_handler) ne re-firent jamais.
export CALYPSO_QEMU_HALT=1
echo "[gdb-sim] Launching run.sh (icount=auto, L2=$CALYPSO_L2_CLIENT, NO_ATTACH=1)..."
echo "[gdb-sim] ============== run.sh OUTPUT =============="
cd "$SCRIPT_DIR"
./run.sh 2>&1
echo "[gdb-sim] ============== run.sh DONE =============="
echo "[gdb-sim] log saved : $RUN_LOG"
echo "[gdb-sim] (full pipeline in tmux session 'calypso' : tmux a -t calypso)"

# ---- 3) Attente gdb-stub ----
echo "[gdb-sim] Waiting for QEMU gdb-stub on $GDB_HOST:$GDB_PORT (max 30s)..."
GDB_UP=0
for i in $(seq 1 30); do
  if exec 3<>/dev/tcp/$GDB_HOST/$GDB_PORT 2>/dev/null; then
    exec 3<&-
    exec 3>&-
    GDB_UP=1
    echo "[gdb-sim] gdb-stub up after ${i}s"
    break
  fi
  sleep 1
done

if [ "$GDB_UP" -eq 0 ]; then
  echo "[gdb-sim] ERROR: gdb-stub n'écoute pas après 30s."
  echo "[gdb-sim] Check $RUN_LOG et tmux a -t calypso pour diagnostiquer."
  exit 1
fi

# ---- 4) Génère le script gdb ----
cat > "$GDB_SCRIPT" <<'GDB_EOF'
set pagination off
set confirm off
set architecture armv5te
target remote :1234

# WATCHPOINT VIRÉ (c web 2026-05-27) : un WP TCG sous icount peut casser
# le commit du store qu'il prétend observer. Symptôme : handler strne EXEC
# fire (BP confirme), mais la valeur ne stick pas en mémoire car le WP a
# perturbé le commit guest. Hypothèse à tester sans WP : poll exit
# spontanément sur le write handler tout seul. Si oui = effet observateur.
# Si non = bug de commit réel, à investiguer (RAM vs IO sur 0x830000).

# BP #2 : entry calypso_sim_powerup
hbreak *0x00822f54
commands
silent
printf "[BP] ENTER calypso_sim_powerup volt=%d lr=0x%x\n", $r0, $lr
c
end

# BP #3 : entry sim_irq_handler
hbreak *0x008228b0
commands
silent
printf "[BP] ENTER sim_irq_handler lr=0x%x cpsr=0x%x\n", $lr, $cpsr
c
end

# BP #4 : strne rxDoneFlag (0x008228c4)
hbreak *0x008228c4
commands
silent
if ($cpsr & 0x40000000)
  printf "[BP] strne SKIP Z=1 r2=0x%x (IT_WT bit absent) PC=0x%x\n", $r2, $pc
else
  printf "[BP] strne EXEC Z=0 r1=0x%x -> *0x%x  (r2=0x%x at TST)\n", $r1, $r3, $r2
end
c
end

# BP #5 : sortie calypso_sim_powerup (mov r0,#0 ; pop)
hbreak *0x00822fb4
commands
silent
printf "[BP] EXIT calypso_sim_powerup (rxDoneFlag=1 vue, poll exited)\n"
c
end

# Reset QEMU pour que les BPs fire en sequence normale (sinon firmware
# deja en busy-poll, BPs sont des events passes/futurs jamais re-atteints).
monitor system_reset
printf "[gdb-sim] system_reset envoye - firmware redemarre, BPs vont fire\n"
c
GDB_EOF

# ---- 5) Lance gdb dans une window tmux split avec tail qemu.log ----
# Layout : window 'gdb' avec 2 panes vsplit :
#   - Top    : gdb-multiarch interactif
#   - Bottom : tail -f /root/qemu.log (sondes [sim], [c54x], [INTH], etc.)
# User attache la session calypso et passe en window 'gdb' (C-b w + 's').
echo ""
echo "[gdb-sim] Probes posées :"
echo "  - (WATCH rxDoneFlag VIRÉ — test effet observateur TCG/WP)"
echo "  - BP @ 0x00822f54 (calypso_sim_powerup entry)"
echo "  - BP @ 0x008228b0 (sim_irq_handler entry)"
echo "  - BP @ 0x008228c4 (strne rxDoneFlag) — log Z flag + r1/r2/r3"
echo "  - BP @ 0x00822fb4 (calypso_sim_powerup exit) ← si fire = poll exit AUTO"
echo ""
echo "[gdb-sim] Création window tmux 'gdb' (top=gdb / bottom=tail qemu.log)..."
tmux kill-window -t calypso:gdb 2>/dev/null || true
tmux new-window -t calypso -n gdb \
    "gdb-multiarch -q -iex 'set pagination off' -iex 'set confirm off' -x $GDB_SCRIPT $ELF ; echo '[gdb-sim] gdb exited, press Enter to close pane' ; read"
tmux split-window -t calypso:gdb -v -p 40 \
    "tail -f /root/qemu.log"
tmux select-pane -t calypso:gdb.0   # cursor sur le pane gdb (top)

tmux select-window -t calypso:gdb 2>/dev/null
echo "[gdb-sim] Session prête."
echo "  -> Window 'gdb' sélectionnée par défaut"
echo "  -> Top pane = gdb interactif (Ctrl-b o pour switcher panes)"
echo "  -> Bottom pane = tail -f /root/qemu.log live"
echo "[gdb-sim] gdb script : $GDB_SCRIPT"
echo "[gdb-sim] Pour killer : tmux kill-session -t calypso"
echo ""
echo "[gdb-sim] Attache automatique à tmux calypso (Ctrl-b d pour détacher)..."
sleep 1
exec tmux attach -t calypso
