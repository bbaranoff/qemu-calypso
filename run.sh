#!/bin/bash
# run.sh — Calypso QEMU pipeline (was run_si.sh, renamed 2026-05-08).
#
# All SI delivery to mobile L3 goes through the legitimate path now :
# osmo-bts-trx encodes BCCH bursts → bridge.py UDP relay → QEMU BSP
# DMA → DSP CCCH demod → a_cd[] in NDB → ARM L1 → L1CTL_DATA_IND.
#
# The previous rsl_si_tap.py + /dev/shm/calypso_si.bin mmap shortcut
# was removed (it was a hack that bypassed DSP demod and made the
# mobile camp on a stale cache even when BTS was dead).

set -euo pipefail

SESSION="calypso"

# ---- args : --gen-doc / --help -----------------------------------------------
# --gen-doc : ne (re)lance PAS le pipeline ; suppose qu'il tourne déjà dans le
# container et lance pytest pour produire la doc (report.md condensé +
# test_results.md/qmd complets). Crée une fenêtre tmux `gen-doc` dans la session
# `calypso` existante et y bascule pour montrer le run pytest en direct.
GEN_DOC_MODE=0
for arg in "$@"; do
    case "$arg" in
        --gen-doc-local) GEN_DOC_MODE=1 ;;
        -h|--help)
            cat <<EOF
Usage: $0 [--gen-doc-local]

Default : launch the full Calypso pipeline (QEMU + osmocon + bridge + bts +
mobile + gsmtap + irda capture) in a tmux session and attach to it.

--gen-doc-local : skip pipeline launch, run pytest in the container "trying"
            (no requirement that qemu-system-arm be running — tests can
            still fail/skip if dependencies missing). Output lands in a new
            tmux window 'gen-doc' of the calypso session (if it exists);
            otherwise streamed to the current terminal.
            Artefacts (inside container):
              /tmp/report.md           (machine-friendly summary)
              /tmp/test_results.md     (full GitHub-flavored)
              /tmp/test_results.qmd    (Quarto source)
              /tmp/test_results_<ts>/  (per-run folder with mmd, csv, json)
            Host copy: /tmp/calypso_report.md
EOF
            exit 0 ;;
    esac
done

# `--gen-doc-local` = synonyme de "lance le pipeline normal ET force la
# génération de doc auto" (qui se déclenche T+30s après QEMU launch dans une
# fenêtre tmux dédiée). Pas de mode "skip pipeline" : si tu veux régénérer la
# doc sans relancer, lance directement pytest dans le container.
if [ "$GEN_DOC_MODE" = "1" ]; then
    CALYPSO_AUTO_GEN_DOC=1
    export CALYPSO_AUTO_GEN_DOC
fi

# Détection pytest réutilisée par la fenêtre gen-doc auto plus bas.
detect_pytest() {
    local override="${CALYPSO_PYTEST:-}"
    if [ -n "$override" ]; then echo "$override"; return; fi
    for cand in /tmp/calypso-venv/bin/pytest \
                /opt/GSM/calypso-venv/bin/pytest \
                /opt/calypso-venv/bin/pytest \
                /root/.venv/bin/pytest \
                /usr/local/bin/pytest \
                /usr/bin/pytest \
                pytest; do
        if [ -x "$cand" ] || command -v "$cand" >/dev/null 2>&1; then
            echo "$cand"; return
        fi
    done
    if python3 -c "import pytest" >/dev/null 2>&1; then
        echo "python3 -m pytest"; return
    fi
    echo ""
}

if false; then  # ancien bloc gen-doc-local, conservé désactivé pour référence

    # In-container mode : `/.dockerenv` présent → on est DANS le container,
    # lance pytest direct sans préfixe docker.
    if [ -f /.dockerenv ]; then
        echo "=== --gen-doc-local : in-container mode (no docker prefix) ==="

        # Deps Python pour pytest + tests QEMU upstream (functional/, guest-debug/)
        pip_install_one() {
            local pkg="$1"
            python3 -c "import $pkg" 2>/dev/null && return 0
            (pip install "$pkg" 2>&1 \
             || pip3 install "$pkg" 2>&1 \
             || python3 -m pip install "$pkg" 2>&1 \
             || pip install --break-system-packages "$pkg" 2>&1) | tail -2
            python3 -c "import $pkg" 2>/dev/null && return 0
            return 1
        }
        PYTEST_BIN=$(detect_pytest)
        if [ -z "$PYTEST_BIN" ]; then
            echo "pytest absent — tentative d'installation..."
            pip_install_one pytest
            PYTEST_BIN=$(detect_pytest)
        fi
        if [ -z "$PYTEST_BIN" ]; then
            echo "ERR: pytest introuvable et install pip échouée." >&2
            echo "     Installer manuellement : pip install pytest" >&2
            echo "     Ou définir CALYPSO_PYTEST=/path/to/pytest" >&2
            exit 1
        fi
        # pycotap : requis par functional/qemu_test/testcase.py
        python3 -c "import pycotap" 2>/dev/null \
            || { echo "Installing pycotap..."; pip_install_one pycotap; }
        # gdb : module Python qui vient avec GDB system (pas via pip).
        # On vérifie que `gdb` (le binaire) est dispo, sinon apt install.
        if ! command -v gdb >/dev/null 2>&1; then
            echo "gdb absent — tentative d'installation via apt..."
            (apt-get update -qq 2>&1 && apt-get install -y -qq gdb 2>&1) | tail -3 || true
        fi
        echo "Using pytest: $PYTEST_BIN"
        # Construit la commande pytest avec ses env + ignores
        PYTEST_CMD="cd /opt/GSM/qemu-src/tests && \
CALYPSO_TEST_OUT=/tmp \
CALYPSO_REPO=\"\${CALYPSO_REPO:-/opt/GSM/qemu-src}\" \
CALYPSO_HOST_ROOT=\"\${CALYPSO_HOST_ROOT:-/root}\" \
$PYTEST_BIN -v --tb=short --color=yes \
    --ignore=functional --ignore=guest-debug \
    --ignore=qemu-iotests --ignore=qtest --ignore=unit \
    --ignore=tcg --ignore=migration --ignore=vm \
    --ignore=avocado --ignore=fp ; \
echo ; echo '=== Artefacts ===' ; \
ls -la /tmp/report.md /tmp/test_results.md /tmp/test_results.qmd 2>/dev/null ; \
echo ; echo '=== Dernier per-run folder ===' ; \
ls -dt /tmp/test_results_*/ 2>/dev/null | head -1 ; \
echo ; echo '[gen-doc done] Press <Enter> to keep this window open.' ; read -r _"

        # Si tmux est dispo et qu'on a une session 'calypso' existante : fenêtre dédiée.
        # Si tmux dispo mais pas de session : on en crée une 'gen-doc-session'.
        # Si pas de tmux du tout : on exécute en foreground.
        if command -v tmux >/dev/null 2>&1; then
            if tmux has-session -t "$SESSION" 2>/dev/null; then
                TARGET="$SESSION"
            else
                TARGET="gen-doc-session"
                tmux new-session -d -s "$TARGET" -n shell 2>/dev/null || true
            fi
            tmux kill-window -t "$TARGET:gen-doc" 2>/dev/null || true
            tmux new-window -t "$TARGET" -n gen-doc
            tmux send-keys -t "$TARGET:gen-doc" "$PYTEST_CMD" C-m
            tmux select-window -t "$TARGET:gen-doc"
            echo "=== --gen-doc-local lancé dans tmux '$TARGET:gen-doc' ==="
            if [ "$TARGET" = "gen-doc-session" ]; then
                echo "Attach : tmux attach -t $TARGET"
                # Si terminal interactif, attach directement
                [ -t 0 ] && exec tmux attach -t "$TARGET"
            else
                echo "Attach (si pas déjà dedans) : tmux attach -t $TARGET"
            fi
            exit 0
        else
            echo "=== tmux absent — exécution foreground ==="
            eval "$PYTEST_CMD"
            exit $?
        fi
    fi

    # Host mode : pytest dans le container via docker exec.
    CONTAINER="${CALYPSO_CONTAINER:-trying}"
    if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
        echo "ERR: container '$CONTAINER' not running." >&2
        exit 1
    fi
    GEN_DOC_CMD=$(cat <<EOF
echo '=== --gen-doc-local : pytest dans container $CONTAINER (pipeline pas requis) ==='
docker exec -e CALYPSO_TEST_OUT=/tmp $CONTAINER bash -c '
    cd /opt/GSM/qemu-src/tests && \
    /tmp/calypso-venv/bin/pytest -v --tb=short --color=yes
'
rc=\$?
echo
echo '=== Artefacts (dans le container) ==='
docker exec $CONTAINER ls -la /tmp/report.md /tmp/test_results.md /tmp/test_results.qmd 2>/dev/null
echo
echo '=== Dernier per-run folder ==='
docker exec $CONTAINER bash -c 'ls -dt /tmp/test_results_*/ 2>/dev/null | head -1'
echo
if docker exec $CONTAINER test -f /tmp/report.md 2>/dev/null; then
    docker exec $CONTAINER cat /tmp/report.md > /tmp/calypso_report.md
    echo "Host copy : /tmp/calypso_report.md (\$(wc -l < /tmp/calypso_report.md) lignes)"
fi
echo
echo "[exit=\$rc] Press <Enter> to keep this window open."
EOF
)
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-window -t "$SESSION:gen-doc" 2>/dev/null || true
        tmux new-window -t "$SESSION" -n gen-doc
        tmux send-keys -t "$SESSION:gen-doc" "$GEN_DOC_CMD; read -r _" C-m
        tmux select-window -t "$SESSION:gen-doc"
        echo "=== --gen-doc-local lancé dans tmux '$SESSION:gen-doc' ==="
        echo "Attach (si pas déjà dedans) : tmux attach -t $SESSION"
        exit 0
    else
        echo "=== tmux session '$SESSION' absente — exécution directe ==="
        eval "$GEN_DOC_CMD"
        exit $?
    fi
fi  # ancien bloc gen-doc-local — désactivé par `if false; then` plus haut

FW_ELF="/opt/GSM/firmware/board/compal_e88/layer1.highram.elf"
FW_BIN="/opt/GSM/firmware/board/compal_e88/layer1.highram.bin"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
BRIDGE="/opt/GSM/qemu-src/bridge.py"
OSMOCON="/opt/GSM/osmocom-bb/src/host/osmocon/osmocon"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"

# ---- DSP / DIAG instruments (override at command line if needed) ----
CALYPSO_DSP_ROM="${CALYPSO_DSP_ROM:-/opt/GSM/calypso_dsp.txt}"
CALYPSO_BSP_DARAM_ADDR="${CALYPSO_BSP_DARAM_ADDR:-0x3fb0}"
CALYPSO_SIM_CFG="${CALYPSO_SIM_CFG:-$MOBILE_CFG}"
export CALYPSO_DSP_ROM CALYPSO_BSP_DARAM_ADDR CALYPSO_SIM_CFG

# ---- Env-gated dev assists (default OFF = real path) ----
# Set =1 in the calling environment to enable. Cf. README.md "Env vars".
# CALYPSO_RXDONE_ADDR : auto-détection via nm sur le firmware elf. Permet de
# survivre aux rebuilds firmware qui shiftent l'adresse BSS de rxDoneFlag.
# Override via env si besoin (force une valeur fixe).
if [ -z "${CALYPSO_RXDONE_ADDR:-}" ]; then
    NM="/root/gnuarm/install/bin/arm-elf-nm"
    [ -x "$NM" ] || NM="arm-elf-nm"
    if command -v "$NM" >/dev/null 2>&1 || [ -x "$NM" ]; then
        _rxd_addr=$("$NM" -n "$FW_ELF" 2>/dev/null \
                    | awk '$3 == "rxDoneFlag" { print "0x"$1 ; exit }')
        if [ -n "$_rxd_addr" ]; then
            CALYPSO_RXDONE_ADDR="$_rxd_addr"
            echo "[run.sh] auto-detected CALYPSO_RXDONE_ADDR=$CALYPSO_RXDONE_ADDR (from nm $FW_ELF)"
        fi
    fi
fi
export CALYPSO_RXDONE_ADDR

CALYPSO_FBSB_SYNTH="${CALYPSO_FBSB_SYNTH:-0}"
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-0}"
CALYPSO_NDB_D_RACH_OFFSET="${CALYPSO_NDB_D_RACH_OFFSET:-}"
CALYPSO_RACH_FORCE_BSIC="${CALYPSO_RACH_FORCE_BSIC:-}"
CALYPSO_DSP_IDLE_FF="${CALYPSO_DSP_IDLE_FF:-1}"
CALYPSO_DSP_IDLE_RANGE="${CALYPSO_DSP_IDLE_RANGE:-}"
CALYPSO_DSP_FBDET_SKIP="${CALYPSO_DSP_FBDET_SKIP:-0}"
CALYPSO_UART_TRACE="${CALYPSO_UART_TRACE:-0}"
# BRIDGE_CLK_FROM_QEMU=0 (default): CLK IND wall-paced. BTS scheduler stays
# happy (no clock-skew shutdown). Pair with BRIDGE_UL_FN_REWRITE=1/slot for
# slot-aware UL rewrite that lands bursts in BTS RACH slot windows.
# =1 mode: CLK IND qfn-paced. Fundamentally incompatible with osmo-bts-trx
# clock-skew check at PERIOD>=3 (BTS shutdown). Only useful for experiments
# with PERIOD=1 (~217 CLK INDs/s wall, marginal but match guaranteed).
BRIDGE_CLK_FROM_QEMU="${BRIDGE_CLK_FROM_QEMU:-0}"
# UL FN rewrite mode: slot|naive|off. Default slot-aware.
BRIDGE_UL_FN_REWRITE="${BRIDGE_UL_FN_REWRITE:-slot}"
# DL FN rewrite mode: slot|naive|off. Default slot-aware (preserves
# bts_fn%%51 → BCCH/CCCH/SDCCH typing intact). Without it, BSP drops
# 100%% of DL bursts since bts_fn drifts ~50 frames/s ahead of qfn.
BRIDGE_DL_FN_REWRITE="${BRIDGE_DL_FN_REWRITE:-slot}"
# Max qfn-future frames a DL burst may be tagged. Bounded by
# BSP_FN_MATCH_WINDOW (=64 default in calypso_bsp.c). Default 32 leaves
# half-window safety margin. Set 51 to never drop (always find a slot).
BRIDGE_DL_FN_LOOKAHEAD="${BRIDGE_DL_FN_LOOKAHEAD:-32}"
# CLK IND period — default 51 (half of stock 102) to keep osmo-bts-trx
# scheduler happy when QEMU runs slower than wall-clock real-time. With
# a 102-frame period, the BTS accumulates skew between consecutive
# CLK INDs faster than they arrive → bts_shutdown_fsm "PC clock skew
# too high" or "No more clock from transceiver" within ~30 s. With 51,
# the correction rate doubles and BTS survives long enough for the
# mobile to complete a Location Update cycle.
# Set to 102 explicitly when QEMU runs near wall-clock (or in production).
BRIDGE_CLK_PERIOD="${BRIDGE_CLK_PERIOD:-51}"
export CALYPSO_FBSB_SYNTH CALYPSO_W1C_LATCH \
       CALYPSO_NDB_D_RACH_OFFSET CALYPSO_RACH_FORCE_BSIC \
       CALYPSO_DSP_IDLE_FF CALYPSO_DSP_IDLE_RANGE \
       CALYPSO_DSP_FBDET_SKIP CALYPSO_UART_TRACE \
       BRIDGE_CLK_FROM_QEMU BRIDGE_CLK_PERIOD \
       BRIDGE_UL_FN_REWRITE BRIDGE_DL_FN_REWRITE BRIDGE_DL_FN_LOOKAHEAD

# ---- icount mode (deterministic virtual clock paced by instruction count) ----
# Default ON (auto = shift=auto,sleep=on,align=off). Set CALYPSO_ICOUNT=off
# to disable. The kick timer (calypso_kick_cb) was moved to
# QEMU_CLOCK_VIRTUAL so it no longer races with icount.
# Other accepted values:
#   auto              shift dynamic, wall-clock aligned (recommended)
#   shift=N,sleep=on  fixed shift (1<<N instr ≈ 1ns), explicit sleep
#   off               disable (legacy default-clock mode)
CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-off}"
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
BTS_LOG="/tmp/bts.log"
L2_LOG="/tmp/l2_client.log"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
L1CTL_SOCK="/tmp/osmocom_l2"
QEMU_DUMMY_SOCK="/tmp/qemu_l1ctl_disabled"

# ---- Log timestamping ----
# Chaque ligne des logs (qemu.log, bridge.log, osmocon.log, mobile.log) est
# préfixée par `<epoch_sec> +<rel_sec>s ` (epoch_sec = horloge mur depuis 1970,
# rel_sec = secondes depuis le démarrage du wrapper). Permet de détecter les
# drifts temporels entre logs en comparant les timestamps colonne 1+2.
# Désactiver via CALYPSO_LOG_TS=0.
# Note : grep -E des tests n'est pas perturbé (le prefix est en début de
# ligne, les patterns sont en milieu).
CALYPSO_LOG_TS="${CALYPSO_LOG_TS:-1}"
TSLOG_SCRIPT=/tmp/calypso_tslog.py
cat > "$TSLOG_SCRIPT" <<'PYEOF'
#!/usr/bin/env python3
"""Préfixe stdin avec `<epoch_sec> +<rel_sec>s ` et flush ligne par ligne."""
import sys, time
t0 = time.time()
for line in sys.stdin:
    t = time.time()
    sys.stdout.write(f"{t:.3f} +{t-t0:.3f}s {line}")
    sys.stdout.flush()
PYEOF
chmod +x "$TSLOG_SCRIPT"
if [ "$CALYPSO_LOG_TS" = "1" ]; then
    TSLOG="python3 -u $TSLOG_SCRIPT"
else
    TSLOG="cat"
fi

# ---------- L2 client selection (menu) ----------
# Choix de l'application L23/L2 qui consomme L1CTL via /tmp/osmocom_l2 :
#   1. mobile    : osmocom-bb mobile complet (stack L23 + VTY) — default
#   2. ccch_scan : ccch_scan -a 1 (scan CCCH ARFCN 1, dump SI/IA)
#   3. cell_log  : cell_log (scan toutes les cells, mesures de power)
# Set CALYPSO_L2_CLIENT=mobile|ccch_scan|cell_log to override the prompt.
CALYPSO_L2_CLIENT="${CALYPSO_L2_CLIENT:-}"
if [ -z "$CALYPSO_L2_CLIENT" ]; then
    echo
    echo "===== L2 client selection ====="
    echo "  1) mobile     (osmocom-bb mobile, full L23 stack + VTY)"
    echo "  2) ccch_scan  (ccch_scan -a 1, scan CCCH ARFCN 1)"
    echo "  3) cell_log   (cell_log, scan toutes cells + power)"
    echo "==============================="
    read -r -p "Choix [1/2/3] (default 1) : " L2_CHOICE
    case "${L2_CHOICE:-1}" in
        2) CALYPSO_L2_CLIENT=ccch_scan ;;
        3) CALYPSO_L2_CLIENT=cell_log ;;
        *) CALYPSO_L2_CLIENT=mobile ;;
    esac
fi
echo "L2 client = $CALYPSO_L2_CLIENT"

# ---------- cleanup ----------
rm -f "$QEMU_LOG" "$BRIDGE_LOG" "$OSMOCON_LOG" "$MOBILE_LOG" "$BTS_LOG" \
      "$MON_SOCK" "$L1CTL_SOCK" "$QEMU_DUMMY_SOCK"

tmux kill-session -t "$SESSION" 2>/dev/null || true
killall -9 qemu-system-arm osmo-bts-trx mobile osmocon 2>/dev/null || true
pkill -9 -f bridge.py 2>/dev/null || true
pkill -9 -f irda_capture.py 2>/dev/null || true
rm -f "$L1CTL_SOCK" "$MON_SOCK" "$QEMU_DUMMY_SOCK" /tmp/osmocom_l2_*
rm -f /tmp/fw-irda.log /tmp/irda_capture.pid /tmp/irda.pty.link
# Drop the legacy mmap from previous runs — no longer used, but lying
# around in /dev/shm could confuse forensic forensics on old runs.
rm -f /dev/shm/calypso_si.bin
sleep 1

/etc/osmocom/status.sh stop 2>/dev/null || true
/etc/osmocom/osmo-start.sh 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n qemu

# ---------- 1. QEMU ----------
# icount controlled by CALYPSO_ICOUNT env var (default 'auto'). The kick
# timer in calypso_trx.c was moved to QEMU_CLOCK_VIRTUAL so icount no
# longer freezes the TDMA tick → bridge UDP path. If you observe the
# bridge wait timeout again, fall back with CALYPSO_ICOUNT=off.
# gdb-stub : activé d'office sur 0.0.0.0:1234 pour que les tests/scripts
# d'injection (inject.py / validating.py / tests/test_inject_frames.py)
# puissent s'y connecter sans avoir à passer par le monitor HMP. La syntaxe
# `tcp::1234` bind sur toutes les interfaces du container — utilisable
# depuis l'IP container (ex. 172.20.0.11:1234) côté host.
# Override via CALYPSO_GDB_PORT (set vide pour désactiver).
CALYPSO_GDB_PORT="${CALYPSO_GDB_PORT:-1234}"
if [ -n "$CALYPSO_GDB_PORT" ]; then
    QEMU_GDB_FLAG="-gdb tcp::$CALYPSO_GDB_PORT"
else
    QEMU_GDB_FLAG=""
fi

L1CTL_SOCK="$QEMU_DUMMY_SOCK" \
"$QEMU" -M calypso -cpu arm946 \
    $QEMU_ICOUNT_FLAG \
    $QEMU_GDB_FLAG \
    -serial pty -serial pty \
    -monitor "unix:${MON_SOCK},server,nowait" \
    -kernel "$FW_ELF" \
    > >($TSLOG > "$QEMU_LOG") 2>&1 &
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

# ---------- 1bis. IrDA capture (UART_IRDA = serial1, IRQ 18, 0xFFFF5000) ----------
# Phase 3 du plan PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md :
# le firmware compal_e88 fait déjà `cons_bind_uart(UART_IRDA)` (init.c:105),
# donc tout `printf()` côté fw passe par UART_IRDA. Ici on consomme le PTY
# serial1 alloué par QEMU et on l'archive dans /tmp/fw-irda.log avec le même
# préfixe timestamp que les autres logs.
#
# Désactivable via CALYPSO_IRDA_CAPTURE=0 (rare — utile si saturation diag).
CALYPSO_IRDA_CAPTURE="${CALYPSO_IRDA_CAPTURE:-1}"
if [ "$CALYPSO_IRDA_CAPTURE" = "1" ]; then
    echo -n "Waiting for QEMU PTY serial1 (UART_IRDA) allocation..."
    PTY_IRDA=""
    for i in $(seq 1 15); do
        if grep -q 'redirected to /dev/pts/.* (label serial1)' "$QEMU_LOG" 2>/dev/null; then
            PTY_IRDA=$(grep 'redirected to /dev/pts/.* (label serial1)' "$QEMU_LOG" \
                        | sed -E 's/.*redirected to (\/dev\/pts\/[0-9]+).*/\1/' | head -1)
            break
        fi
        sleep 0.5; echo -n "."
    done
    if [ -n "$PTY_IRDA" ]; then
        echo " OK ($PTY_IRDA)"
        ln -sf "$PTY_IRDA" /tmp/irda.pty.link
        : > /tmp/fw-irda.log
        tmux new-window -t "$SESSION" -n irda
        # Lance irda_capture en background (silencieux — il pose ses bytes
        # dans /tmp/fw-irda.log directement, pas sur stdout) puis suit le
        # log en live dans la fenêtre tmux pour debug visuel.
        tmux send-keys -t "$SESSION:irda" \
            "IRDA_PTY=/tmp/irda.pty.link FW_IRDA_LOG=/tmp/fw-irda.log python3 /opt/GSM/qemu-src/tools/irda_capture.py 2>/tmp/irda_capture.stderr.log & sleep 0.5 && echo '--- live tail /tmp/fw-irda.log ---' && tail -F /tmp/fw-irda.log" C-m
        echo -n "Waiting for irda_capture to register pid..."
        for i in $(seq 1 20); do
            [ -f /tmp/irda_capture.pid ] && break
            sleep 0.3; echo -n "."
        done
        if [ -f /tmp/irda_capture.pid ]; then
            echo " OK (pid=$(cat /tmp/irda_capture.pid))"
        else
            echo " WARN — pidfile absent (capture peut-être pas lancé)"
        fi
    else
        echo " WARN — no serial1 PTY detected → IrDA capture skipped"
    fi
fi

# Note : le marker `=== fw-irda boot OK ===` émis par cons_puts() ligne 119
# de compal_e88/init.c peut être perdu si irda_capture s'attache au slave PTY
# après que QEMU ait écrit (race window ~0.5–1.5s). Solution durable : passer
# par Phase 5 du plan IrDA (beacon hot path dans la main loop, qui ré-émet
# régulièrement et garantit la capture). Ne PAS tenter `-S` + `cont` ici : ça
# perturbe la séquence osmocon→firmware (le mobile ne camp plus sur la cell).

# ---------- 2. osmocon ----------
tmux new-window -t "$SESSION" -n osmocon
tmux send-keys -t "$SESSION:osmocon" \
    "$OSMOCON -m romload -i 100 -p $PTY_MODEM -s $L1CTL_SOCK $FW_BIN -d tr 2>&1 | $TSLOG | tee $OSMOCON_LOG" C-m

echo -n "Waiting for osmocon to expose $L1CTL_SOCK..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
if [ -S "$L1CTL_SOCK" ]; then echo " OK"; else echo " WARN — socket missing"; fi

# ---------- 3. bridge.py ----------
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" \
    "python3 $BRIDGE 2>&1 | $TSLOG | tee $BRIDGE_LOG" C-m

# Pas de wait `QEMU tick` ici : avec le hack CALYPSO_FORCE_RX_DONE, le
# firmware peut prendre du temps à libérer le polling SIM (plusieurs ops
# SELECT/READ_BINARY successifs), ce qui retarde le premier CLK IND. Le
# bridge et osmo-bts-trx ci-dessous gèrent l'attente naturellement
# (bridge n'émet CLK IND qu'après POWERON de bts-trx, et bts-trx
# patiente que bridge soit prêt).

# ---------- 4. osmo-bts-trx ----------
tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" \
    "osmo-bts-trx -c $BTS_CFG 2>&1 | $TSLOG | tee $BTS_LOG" C-m
# Sync barrier : attendre que osmo-bts-trx ait commencé à pousser des DL
# bursts au bridge — proxy fiable : `bridge: DL #` dans bridge.log
# (le bridge.py imprime ça à chaque burst TRXDv0 reçu de bts-trx).
echo -n "Waiting for osmo-bts-trx to attach to bridge..."
for i in $(seq 1 30); do
    grep -qE 'bridge: DL #|bridge: tick' "$BRIDGE_LOG" 2>/dev/null && break
    sleep 1; echo -n "."
done
if grep -qE 'bridge: DL #|bridge: tick' "$BRIDGE_LOG" 2>/dev/null; then
    echo " OK"
else
    echo " TIMEOUT (bts-trx pas attaché en 30s)"
fi

# ---------- 5. L2 client (mobile / ccch_scan / cell_log) ----------
# Sync barrier inline dans la cmd tmux : attendre que la socket l1ctl existe
# (créée par osmocon après son handshake romload avec le firmware) avant de
# lancer mobile. Évite le `sleep 3` arbitraire et le 51s spread observé.
L1CTL_WAIT='i=0; while [ ! -S '"$L1CTL_SOCK"' ] && [ $i -lt 60 ]; do sleep 0.5; i=$((i+1)); done'
tmux new-window -t "$SESSION" -n "$CALYPSO_L2_CLIENT"
case "$CALYPSO_L2_CLIENT" in
    mobile)
        tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
            "$L1CTL_WAIT && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
        ;;
    ccch_scan)
        tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
            "$L1CTL_WAIT && ccch_scan -a 1 2>&1 | $TSLOG | tee $L2_LOG" C-m
        ;;
    cell_log)
        tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
            "$L1CTL_WAIT && cell_log 2>&1 | $TSLOG | tee $L2_LOG" C-m
        ;;
    *)
        echo "WARN — CALYPSO_L2_CLIENT=$CALYPSO_L2_CLIENT inconnu, fallback mobile"
        tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
            "$L1CTL_WAIT && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
        ;;
esac

# ---------- 6. gsmtap capture (any iface — covers eth0 mobile/BTS + eth1) ----------
# `--print` affiche en live ET continue d'écrire le pcap ; `-U` flush par paquet
# pour que le pcap soit utilisable immédiatement (sinon buffer 4KB).
tmux new-window -t "$SESSION" -n gsmtap
tmux send-keys -t "$SESSION:gsmtap" \
    "sleep 5 && tcpdump -i any -U --print -X -w /root/mobile-gsmtap.pcap udp port 4729" C-m

# ---------- 7. window 'all' — agrège les 6 premières en 6 panes ----------
# Vue unique pour superviser qemu / irda / osmocon / bridge / bts / L2 client
# côte à côte. Chaque pane fait juste `tail -F` du log correspondant. Layout
# tiled = grille 2×3 par défaut. Les logs n'existent peut-être pas encore au
# moment de la création des panes — `tail -F` (majuscule) gère ce cas (suit
# le fichier dès qu'il apparaît) sans crash.
case "$CALYPSO_L2_CLIENT" in
    ccch_scan|cell_log) L2_TAIL_LOG="$L2_LOG" ;;
    *)                  L2_TAIL_LOG="$MOBILE_LOG" ;;
esac
# Création séquentielle : `select-layout tiled` après chaque split pour
# redistribuer l'espace, sinon la 3e/4e pane devient trop étroite et tmux
# rejette le split suivant avec "no space for new pane".
tmux new-window -t "$SESSION" -n all \
    "clear; echo '=== qemu ==='; tail -F $QEMU_LOG"
for spec in \
    "tcpdump|__TCPDUMP__" \
    "osmocon|$OSMOCON_LOG" \
    "bridge|$BRIDGE_LOG" \
    "bts|$BTS_LOG" \
    "$CALYPSO_L2_CLIENT|$L2_TAIL_LOG"
do
    name="${spec%%|*}"; log="${spec##*|}"
    if [ "$log" = "__TCPDUMP__" ]; then
        # Live tcpdump hex + ASCII (sans -w pour ne pas dupliquer le pcap
        # canonique géré par la fenêtre `gsmtap`).
        cmd="clear; echo '=== $name ==='; sleep 5 && tcpdump -i any -X udp port 4729"
    else
        cmd="clear; echo '=== $name ==='; tail -F $log"
    fi
    tmux split-window -t "$SESSION:all" "$cmd"
    tmux select-layout -t "$SESSION:all" tiled
done

# ---------- 8. gen-doc auto (30s après QEMU launch) ----------
# Une fenêtre tmux dédiée qui attend 30s que le pipeline se stabilise puis
# lance pytest in-container pour produire la doc en arrière-plan. Pas de
# rappel récursif à run.sh — on inline la commande pytest avec ses ignores
# et env vars. Désactivable via CALYPSO_AUTO_GEN_DOC=0.
CALYPSO_AUTO_GEN_DOC="${CALYPSO_AUTO_GEN_DOC:-1}"
if [ "$CALYPSO_AUTO_GEN_DOC" = "1" ]; then
    # Pré-install des deps Python (silencieux, idempotent)
    pip_install_silent() {
        python3 -c "import $1" 2>/dev/null && return 0
        (pip install "$1" 2>&1 \
         || pip3 install "$1" 2>&1 \
         || python3 -m pip install "$1" 2>&1) >/dev/null
        python3 -c "import $1" 2>/dev/null
    }
    pip_install_silent pytest  || true
    pip_install_silent pycotap || true

    GEN_DOC_PYTEST=$(detect_pytest)
    [ -z "$GEN_DOC_PYTEST" ] && GEN_DOC_PYTEST="python3 -m pytest"

    tmux new-window -t "$SESSION" -n gen-doc
    tmux send-keys -t "$SESSION:gen-doc" \
        "echo '[gen-doc] waiting 60s for pipeline to stabilize…'; sleep 60; \
         echo '[gen-doc] launching pytest in-container'; \
         cd /opt/GSM/qemu-src/tests && \
         CALYPSO_TEST_OUT=/tmp \
         CALYPSO_REPO=/opt/GSM/qemu-src \
         CALYPSO_HOST_ROOT=/root \
         $GEN_DOC_PYTEST -v --tb=short --color=yes \
            --ignore=functional --ignore=guest-debug \
            --ignore=qemu-iotests --ignore=qtest --ignore=unit \
            --ignore=tcg --ignore=migration --ignore=vm \
            --ignore=avocado --ignore=fp; \
         echo; echo '=== Artefacts ==='; \
         ls -la /tmp/report.md /tmp/test_results.md /tmp/test_results.qmd 2>/dev/null; \
         echo; echo '[gen-doc] done. Press <Enter> to keep window open.'; read -r _" C-m
fi

# ---------- shell + attach ----------
tmux new-window -t "$SESSION" -n shell

echo
echo "Pipeline launched. Attach with: tmux attach -t $SESSION"
echo "ENV summary:"
echo "  CALYPSO_DSP_ROM             = $CALYPSO_DSP_ROM"
echo "  CALYPSO_BSP_DARAM_ADDR      = $CALYPSO_BSP_DARAM_ADDR"
echo "  CALYPSO_SIM_CFG             = $CALYPSO_SIM_CFG"
echo "  CALYPSO_FBSB_SYNTH          = $CALYPSO_FBSB_SYNTH"
echo "  CALYPSO_W1C_LATCH           = $CALYPSO_W1C_LATCH"
echo "  CALYPSO_NDB_D_RACH_OFFSET   = ${CALYPSO_NDB_D_RACH_OFFSET:-(default 0x023a — pinned 2026-05-07)}"
echo "  CALYPSO_RACH_FORCE_BSIC     = ${CALYPSO_RACH_FORCE_BSIC:-(unset = use d_rach byte)}"
echo "  BRIDGE_CLK_FROM_QEMU        = $BRIDGE_CLK_FROM_QEMU  (0=wall-paced safe, 1=qfn-paced experimental)"
echo "  BRIDGE_UL_FN_REWRITE        = $BRIDGE_UL_FN_REWRITE  (slot=next RACH slot ≥ wall_fn, naive=blind wall_fn, off=passthrough)"
echo "  BRIDGE_DL_FN_REWRITE        = $BRIDGE_DL_FN_REWRITE  (slot=qfn matching bts_fn%51, naive=current qfn, off=passthrough — BSP rejects all)"
echo "  BRIDGE_DL_FN_LOOKAHEAD      = $BRIDGE_DL_FN_LOOKAHEAD  (max qfn-future frames before drop, BSP window=64)"
echo "  CALYPSO_ICOUNT              = $CALYPSO_ICOUNT  (flag: ${QEMU_ICOUNT_FLAG:-(none)})"
echo "  CALYPSO_DSP_IDLE_FF         = $CALYPSO_DSP_IDLE_FF  (1=fast-forward DSP idle dispatcher)"
echo "  CALYPSO_DSP_IDLE_RANGE      = ${CALYPSO_DSP_IDLE_RANGE:-(default 0xe9ac:0xe9b7,0xcc62:0xcc6f)}"
echo "  CALYPSO_FORCE_RX_DONE       = ${CALYPSO_FORCE_RX_DONE}"
echo "  CALYPSO_IRDA_CAPTURE        = $CALYPSO_IRDA_CAPTURE  (1=consume serial1 PTY → /tmp/fw-irda.log)"
if [ "$CALYPSO_IRDA_CAPTURE" = "1" ] && [ -n "${PTY_IRDA:-}" ]; then
    echo "  IrDA channel                = $PTY_IRDA → /tmp/irda.pty.link → /tmp/fw-irda.log"
fi
echo
echo "Manual warm-start (debug, if BSC unavailable) :"
echo "  /opt/GSM/qemu-src/scripts/populate-si.sh"
echo

tmux select-window -t "$SESSION:all" 2>/dev/null || tmux select-window -t "$SESSION:qemu"
exec tmux attach -t "$SESSION"
