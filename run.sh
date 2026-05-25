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
MENU_MODE=0
for arg in "$@"; do
    case "$arg" in
        --gen-doc-local) GEN_DOC_MODE=1 ;;
        --menu) MENU_MODE=1 ;;
        -h|--help)
            cat <<EOF
Usage: $0 [--gen-doc-local] [--menu]

Default : launch the full Calypso pipeline (QEMU + osmocon + bridge + bts +
mobile + gsmtap + irda capture) in a tmux session and attach to it.

--menu      : interactive profile selection before launch (hack-free /
              dev-assist / full-diag / custom). Sets CALYPSO_* env vars
              and re-execs.

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

# ---- --menu : interactive profile selection -----------------------------
# Set les CALYPSO_* vars selon profil choisi avant les defaults plus bas.
# Profils :
#   1) hack-free + diag minimal     (run nettoyé, peu de logs)
#   2) hack-free + diag complet     (= sondes hunt actuelles : SP/AR/IMR/RSBX/WATCH)
#   3) dev-assist PM_INJECT=800     (= mobile passe PM scan via hack)
#   4) dev-assist FBSB_SYNTH=1      (= DSP FB-det shunted, mobile gsm322)
#   5) custom (réponse var-par-var)
#   0) cancel
if [ "$MENU_MODE" = "1" ]; then
    echo
    echo "========================================="
    echo " Calypso run.sh — Profile selection"
    echo "========================================="
    echo
    echo "  1) hack-free + diag MINIMAL"
    echo "     (production-like, peu de logs)"
    echo
    echo "  2) hack-free + diag FULL"
    echo "     (sondes hunt actives : SP_RING + AR_TRACE + RSBX + WATCH_3FBE)"
    echo
    echo "  3) dev-assist PM_INJECT=800"
    echo "     (mobile passe PM scan via hack, atteint FBSB_REQ)"
    echo
    echo "  4) dev-assist FBSB_SYNTH=1"
    echo "     (DSP FB-det shunted via synth NDB write, mobile→gsm322)"
    echo
    echo "  5) custom (saisie var-par-var)"
    echo
    echo "  0) cancel"
    echo
    read -p "Choix [1-5, 0] : " CHOICE
    case "$CHOICE" in
        1)
            echo "[menu] Profil = hack-free + diag MINIMAL"
            export CALYPSO_FBSB_SYNTH=0
            export CALYPSO_DSP_FBDET_SKIP=0
            export CALYPSO_PM_INJECT=0
            export CALYPSO_DSP_FORCE_DARAM62=0
            export CALYPSO_PROBE_BOOTSTUB=0
            # Force icount=auto, MTTCG off pour déterminisme (cf
            # session 2026-05-25 : data MTTCG non-citable pour claims
            # d'état firmware. Run.sh:462 force ICOUNT=off si MTTCG=1).
            export CALYPSO_MTTCG=0
            export CALYPSO_ICOUNT=auto
            ;;
        2)
            echo "[menu] Profil = hack-free + diag FULL"
            export CALYPSO_FBSB_SYNTH=0
            export CALYPSO_DSP_FBDET_SKIP=0
            export CALYPSO_PM_INJECT=0
            export CALYPSO_DSP_FORCE_DARAM62=0
            export CALYPSO_PROBE_BOOTSTUB=0
            # Force déterminisme : icount=auto, MTTCG off (cf session
            # 2026-05-25 : data MTTCG non-citable pour state claims).
            export CALYPSO_MTTCG=0
            export CALYPSO_ICOUNT=auto
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
            ;;
        3)
            echo "[menu] Profil = dev-assist PM_INJECT=800"
            export CALYPSO_FBSB_SYNTH=0
            export CALYPSO_DSP_FBDET_SKIP=0
            export CALYPSO_PM_INJECT=800
            ;;
        4)
            echo "[menu] Profil = dev-assist FBSB_SYNTH=1"
            export CALYPSO_FBSB_SYNTH=1
            export CALYPSO_DSP_FBDET_SKIP=0
            export CALYPSO_PM_INJECT=800
            ;;
        5)
            echo "[menu] Profil = custom"
            _ask() {
                local var="$1" def="$2" prompt="$3"
                read -p "  $var [$def] ($prompt) : " val
                val="${val:-$def}"
                export "$var=$val"
            }
            echo "--- Hacks (0 = désactivé, no-hack) ---"
            _ask CALYPSO_PM_INJECT       0 "PM scan hack (800 = inject value)"
            _ask CALYPSO_FBSB_SYNTH      0 "synth FB/SB NDB hack"
            _ask CALYPSO_DSP_FBDET_SKIP  0 "skip DSP FB-det handler"
            echo "--- Diagnostic ---"
            _ask CALYPSO_SP_RING         0 "SP ring buffer"
            _ask CALYPSO_SP_ABS_TRACE    0 "SP absolute writes"
            _ask CALYPSO_AR_TRACE        0 "AR0..AR7 mask (0xFF = all)"
            _ask CALYPSO_WATCH_3FBE      0 "watch 0x3fb0..0x3fbf writes"
            _ask CALYPSO_CORRELATOR_TRACE 0 "FB-det reads + AR3/AR4 entry"
            _ask CALYPSO_MVPD_TRACE      0 "MVPD overlay occupancy"
            _ask CALYPSO_RSBX_INTM_TRACE 0 "RSBX INTM hits counter"
            echo "--- Misc ---"
            _ask CALYPSO_BSP_DARAM_ADDR  0x3fb0 "BSP DMA target addr"
            _ask CALYPSO_IQ              on    "IQ passthrough (on/off)"
            _ask CALYPSO_ICOUNT          auto  "icount mode"
            ;;
        0|*)
            echo "[menu] cancelled"
            exit 0
            ;;
    esac
    echo
    echo "[menu] Lancement avec :"
    env | grep -E '^CALYPSO_' | sort | sed 's/^/  /'
    echo
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

# Firmware paths configurable via env (FW_ELF/FW_BIN). Defaults to layer1
# pour compal_e88. run_rssi.sh override these to lancer rssi.highram à la place.
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
FW_BIN="${FW_BIN:-/opt/GSM/firmware/board/compal_e88/layer1.highram.bin}"
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

# === Defaults no-hack (2026-05-25) ===
# Politique : aucun bypass de chemin firmware par défaut. Les hacks
# dev-assist (PM_INJECT, FBSB_SYNTH, DSP_FBDET_SKIP) sont à 0 par défaut.
# Override explicite dans la cmdline si tu veux un dev-assist temporaire.
# Conforme à CLAUDE.md règle #1 : "PAS DE HACK". Voir aussi audit run env
# 2026-05-25 (l'absence de CALYPSO_PM_INJECT activait le hack=800 par défaut).
CALYPSO_FBSB_SYNTH="${CALYPSO_FBSB_SYNTH:-0}"        # no-hack : synth FB/SB désactivé
CALYPSO_DSP_FBDET_SKIP="${CALYPSO_DSP_FBDET_SKIP:-0}" # no-hack : FB-det handler exécuté
CALYPSO_PM_INJECT="${CALYPSO_PM_INJECT:-0}"          # no-hack : PM scan réel (code default 800 = HACK actif si absent)
CALYPSO_DSP_FORCE_DARAM62="${CALYPSO_DSP_FORCE_DARAM62:-0}" # no-hack : pas de force daram[0x62]=1
CALYPSO_PROBE_BOOTSTUB="${CALYPSO_PROBE_BOOTSTUB:-0}" # no-hack : pas de probe-injection bootstub

# === Stability / non-hack mais nécessaire ===
CALYPSO_FORCE_RX_DONE="${CALYPSO_FORCE_RX_DONE:-1}"   # load-bearing pour osmocon firmware download (mémoire [[project_force_rx_done_load_bearing]])
CALYPSO_DSP_IDLE_FF="${CALYPSO_DSP_IDLE_FF:-1}"       # perf : fast-forward DSP idle dispatcher (pas un hack)
CALYPSO_DSP_IDLE_RANGE="${CALYPSO_DSP_IDLE_RANGE:-}"

# === Diag / debug (no-hack) ===
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-0}"
CALYPSO_NDB_D_RACH_OFFSET="${CALYPSO_NDB_D_RACH_OFFSET:-}"
CALYPSO_RACH_FORCE_BSIC="${CALYPSO_RACH_FORCE_BSIC:-}"
CALYPSO_UART_TRACE="${CALYPSO_UART_TRACE:-0}"
# CALYPSO_IQ : master switch pour le chemin IQ passthrough.
#   on/1   = bridge GMSK-module les soft-bits (scipy BT=0.3) et envoie 296 B IQ
#            par burst ; QEMU BSP fait memcpy direct au DSP correlator au lieu
#            de la modulation ±π/2 hard interne. Cible : améliorer TOA/Angle
#            FB-det (vu DSP error 51536 + TOA=0xFFFFC9C0 en mode hard-mod).
#   off/0  = mode hard-mod historique (bridge soft-bits + BSP modulate ±π/2).
# Default : on (le hard-mod est connu pour casser le DSP correlator FCCH).
CALYPSO_IQ="${CALYPSO_IQ:-on}"
case "$CALYPSO_IQ" in
    on|1|true|yes)
        CALYPSO_BSP_IQ_PASSTHROUGH=1
        BRIDGE_BSP_IQ=1
        ;;
    off|0|false|no)
        CALYPSO_BSP_IQ_PASSTHROUGH=0
        BRIDGE_BSP_IQ=0
        ;;
    *)
        echo "[run.sh] CALYPSO_IQ='$CALYPSO_IQ' inconnu (attendu on/off) → fallback on" >&2
        CALYPSO_BSP_IQ_PASSTHROUGH=1
        BRIDGE_BSP_IQ=1
        ;;
esac
export CALYPSO_IQ CALYPSO_BSP_IQ_PASSTHROUGH BRIDGE_BSP_IQ
# CALYPSO_TIMER : gate les fprintf timer (frame_irq, tdma_tick, kick) côté
# calypso_trx.c. =0 (default) = runs silencieux, zéro stderr-pipe backpressure.
# =1 = mêmes lignes qu'avant. Cf. helper static calypso_timer_log() (cached).
CALYPSO_TIMER="${CALYPSO_TIMER:-0}"
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
# Default 51 (was 32) : couvre un cycle complet mod 51 → tout bts_fn trouve
# un qfn slot matchant dans la fenêtre. Sous icount=auto, drift bts↔qfn
# ~34k frames → avec 32 on dropait ~37% des DL bursts ("delta_to_match=33
# > lookahead=32"). 51 = burst latency max ~230ms, acceptable cell selection.
BRIDGE_DL_FN_LOOKAHEAD="${BRIDGE_DL_FN_LOOKAHEAD:-51}"
# CLK IND period — default 51 (half of stock 102) to keep osmo-bts-trx
# scheduler happy when QEMU runs slower than wall-clock real-time. With
# a 102-frame period, the BTS accumulates skew between consecutive
# CLK INDs faster than they arrive → bts_shutdown_fsm "PC clock skew
# too high" or "No more clock from transceiver" within ~30 s. With 51,
# the correction rate doubles and BTS survives long enough for the
# mobile to complete a Location Update cycle.
# Set to 102 explicitly when QEMU runs near wall-clock (or in production).
BRIDGE_CLK_PERIOD="${BRIDGE_CLK_PERIOD:-51}"
CALYPSO_DSP_BUDGET="${CALYPSO_DSP_BUDGET:-256000}"
export CALYPSO_FBSB_SYNTH CALYPSO_W1C_LATCH \
       CALYPSO_NDB_D_RACH_OFFSET CALYPSO_RACH_FORCE_BSIC \
       CALYPSO_DSP_IDLE_FF CALYPSO_DSP_IDLE_RANGE \
       CALYPSO_FORCE_RX_DONE \
       CALYPSO_DSP_FBDET_SKIP CALYPSO_UART_TRACE CALYPSO_TIMER \
       CALYPSO_PROBE_BOOTSTUB CALYPSO_DSP_BUDGET \
       CALYPSO_PM_INJECT CALYPSO_DSP_FORCE_DARAM62 \
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
CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-auto}"  # default auto = stability (CLAUDE.md Stability config)
export CALYPSO_ICOUNT

# ---- MTTCG mode (Phase C : multi-thread TCG, ARM en thread distinct
#                  des periph callbacks). Incompatible avec icount=auto.
#                  Voir MTTCG_AUDIT.md pour les locks fins requis. ----
CALYPSO_MTTCG="${CALYPSO_MTTCG:-0}"
export CALYPSO_MTTCG
if [ "$CALYPSO_MTTCG" = "1" ]; then
    if [ "$CALYPSO_ICOUNT" != "off" ]; then
        echo "[run.sh] CALYPSO_MTTCG=1 forces CALYPSO_ICOUNT=off (incompatible)" >&2
        CALYPSO_ICOUNT="off"
        export CALYPSO_ICOUNT
    fi
    if [ "${CALYPSO_PCB_TICK_THREADS:-0}" = "1" ]; then
        echo "[run.sh] CALYPSO_MTTCG=1 forces CALYPSO_PCB_TICK_THREADS=0 (redondant)" >&2
        CALYPSO_PCB_TICK_THREADS="0"
        export CALYPSO_PCB_TICK_THREADS
    fi
    QEMU_ACCEL_FLAG="-accel tcg,thread=multi"
else
    QEMU_ACCEL_FLAG=""
fi

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
echo "CALYPSO_IQ = $CALYPSO_IQ (CALYPSO_BSP_IQ_PASSTHROUGH=$CALYPSO_BSP_IQ_PASSTHROUGH, BRIDGE_BSP_IQ=$BRIDGE_BSP_IQ)"

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

# Override délibéré : pour le child QEMU seulement, L1CTL_SOCK pointe vers
# le dummy (/tmp/qemu_l1ctl_disabled). QEMU/l1ctl_sock.c crée son socket à
# cette adresse-poubelle (= L1CTL QEMU désactivé). Le VRAI socket L1CTL
# /tmp/osmocom_l2 est créé plus tard par osmocon (L541 : -s $L1CTL_SOCK,
# où L1CTL_SOCK garde sa valeur d'origine = /tmp/osmocom_l2). Hors de
# cette ligne, $L1CTL_SOCK reste = /tmp/osmocom_l2 partout dans run.sh.
L1CTL_SOCK="$QEMU_DUMMY_SOCK" \
"$QEMU" -M calypso -cpu arm946 \
    $QEMU_ICOUNT_FLAG \
    $QEMU_ACCEL_FLAG \
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
    ": > $OSMOCON_LOG && $OSMOCON -m romload -i 100 -p $PTY_MODEM -s $L1CTL_SOCK $FW_BIN -d tr 2>&1 | $TSLOG | tee $OSMOCON_LOG" C-m

echo -n "Waiting for osmocon to expose $L1CTL_SOCK..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
if [ -S "$L1CTL_SOCK" ]; then echo " OK"; else echo " WARN — socket missing"; fi

# ---------- 3. bridge.py ----------
tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION:bridge" \
    ": > $BRIDGE_LOG && BRIDGE_BSP_IQ=$BRIDGE_BSP_IQ python3 $BRIDGE 2>&1 | $TSLOG | tee $BRIDGE_LOG" C-m

# Pas de wait `QEMU tick` ici : avec le hack CALYPSO_FORCE_RX_DONE, le
# firmware peut prendre du temps à libérer le polling SIM (plusieurs ops
# SELECT/READ_BINARY successifs), ce qui retarde le premier CLK IND. Le
# bridge et osmo-bts-trx ci-dessous gèrent l'attente naturellement
# (bridge n'émet CLK IND qu'après POWERON de bts-trx, et bts-trx
# patiente que bridge soit prêt).

# ---------- 4. osmo-bts-trx ----------
tmux new-window -t "$SESSION" -n bts
tmux send-keys -t "$SESSION:bts" \
    ": > $BTS_LOG && osmo-bts-trx -c $BTS_CFG 2>&1 | $TSLOG | tee $BTS_LOG" C-m
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
#
# Crée aussi le symlink ${L1CTL_SOCK}_1 → ${L1CTL_SOCK} : le mobile
# (instance 1, "ms 1" dans mobile_group1.cfg → layer2-socket
# /tmp/osmocom_l2_1) cherche le path avec suffixe _1, alors qu'osmocon
# crée /tmp/osmocom_l2 (sans suffixe). Sans ce symlink, layer2_open()
# crash → parse cfg fatal → mobile sort, et osmocon stagne dans poll()
# en émettant des LOST N! sans pipeline L1CTL.
# La cleanup ligne 428 fait `rm -f /tmp/osmocom_l2_*` donc safe ici.
#
# Limitation : couvre uniquement "ms 1". Pour multi-instance (ms 2, etc.)
# il faudrait N symlinks. Pas d'enjeu actuel (cfg utilise ms 1 only).
#
# Trou connu — SAP socket non câblé : mobile_group1.cfg:36 a aussi
# `sap-socket /tmp/osmocom_sap_1`. Aucun producteur ne crée
# /tmp/osmocom_sap dans cette stack. Si SAP s'active un jour (SIM
# emulation via SAP au lieu de natif), il faut d'abord ajouter un
# producteur (mocksapd ou QEMU module), PUIS un symlink _1. Ajouter
# juste un `ln -sf` sans producteur = faux positif de couverture.
L1CTL_WAIT='i=0; while [ ! -S '"$L1CTL_SOCK"' ] && [ $i -lt 60 ]; do sleep 0.5; i=$((i+1)); done; [ -S '"$L1CTL_SOCK"' ] && ln -sf '"$L1CTL_SOCK"' '"$L1CTL_SOCK"'_1'
tmux new-window -t "$SESSION" -n "$CALYPSO_L2_CLIENT"
case "$CALYPSO_L2_CLIENT" in
    mobile)
        tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
            ": > $MOBILE_LOG && $L1CTL_WAIT && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
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
            ": > $MOBILE_LOG && $L1CTL_WAIT && mobile -c $MOBILE_CFG -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
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

# Build identity dump (2026-05-25) — attribution causale dans report.
# Logged au stdout du wrapper + capture-able dans qemu.log via c54x_reset.
echo "===== BUILD IDENTITY ====="
QEMU_C54X_PATH="${QEMU_C54X_PATH:-/opt/GSM/qemu-src/hw/arm/calypso/calypso_c54x.c}"
if command -v git >/dev/null 2>&1 && [ -d "$(dirname "$QEMU_C54X_PATH")/../../../.git" ]; then
    cd "$(dirname "$QEMU_C54X_PATH")/../../.." 2>/dev/null && {
        echo "  git rev:         $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
        echo "  git status:      $(git diff --stat 2>/dev/null | tail -1 || echo unknown)"
        cd - >/dev/null
    }
fi
echo "  qemu binary mtime: $(stat -c %y "${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}" 2>/dev/null | cut -d. -f1)"
echo "  c54x source mtime: $(stat -c %y "$QEMU_C54X_PATH" 2>/dev/null | cut -d. -f1)"
# Marqueurs decoder fixes — si la string disparait du binaire, le fix est retiré
if [ -x "${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}" ]; then
    if strings "${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}" 2>/dev/null | grep -q 'FIRS catch RETIRÉ'; then
        echo "  decoder-fix F1xx FIRS catch:      ✓ REMOVED (2026-05-25)"
    else
        echo "  decoder-fix F1xx FIRS catch:      ⚠ NOT FOUND in binary"
    fi
    if strings "${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}" 2>/dev/null | grep -q 'Fix 2026-05-25 v4 : src/dst'; then
        echo "  decoder-fix L3609 src/dst swap:   ✓ APPLIED (2026-05-25 v4)"
    else
        echo "  decoder-fix L3609 src/dst swap:   ⚠ NOT FOUND in binary"
    fi
fi
echo "=========================="
echo

echo "ENV summary:"
echo "  CALYPSO_DSP_ROM             = $CALYPSO_DSP_ROM"
echo "  CALYPSO_BSP_DARAM_ADDR      = $CALYPSO_BSP_DARAM_ADDR"
echo "  CALYPSO_SIM_CFG             = $CALYPSO_SIM_CFG"
echo "  CALYPSO_FBSB_SYNTH          = $CALYPSO_FBSB_SYNTH        (0=real DSP path, 1=HACK synth FB/SB)"
echo "  CALYPSO_PM_INJECT           = $CALYPSO_PM_INJECT        (0=real PM scan, !=0=HACK inject value)"
echo "  CALYPSO_DSP_FBDET_SKIP      = $CALYPSO_DSP_FBDET_SKIP        (0=run FB-det handler, 1=HACK skip)"
echo "  CALYPSO_DSP_FORCE_DARAM62   = $CALYPSO_DSP_FORCE_DARAM62        (0=normal, 1=HACK force daram[0x62]=1)"
echo "  CALYPSO_PROBE_BOOTSTUB      = $CALYPSO_PROBE_BOOTSTUB        (0=normal, 1=HACK probe-inject bootstub)"
echo "  CALYPSO_W1C_LATCH           = $CALYPSO_W1C_LATCH"
echo "  CALYPSO_NDB_D_RACH_OFFSET   = ${CALYPSO_NDB_D_RACH_OFFSET:-(default 0x023a — pinned 2026-05-07)}"
echo "  CALYPSO_RACH_FORCE_BSIC     = ${CALYPSO_RACH_FORCE_BSIC:-(unset = use d_rach byte)}"
echo "  BRIDGE_CLK_FROM_QEMU        = $BRIDGE_CLK_FROM_QEMU  (0=wall-paced safe, 1=qfn-paced experimental)"
echo "  BRIDGE_UL_FN_REWRITE        = $BRIDGE_UL_FN_REWRITE  (slot=next RACH slot ≥ wall_fn, naive=blind wall_fn, off=passthrough)"
echo "  BRIDGE_DL_FN_REWRITE        = $BRIDGE_DL_FN_REWRITE  (slot=qfn matching bts_fn%51, naive=current qfn, off=passthrough — BSP rejects all)"
echo "  BRIDGE_DL_FN_LOOKAHEAD      = $BRIDGE_DL_FN_LOOKAHEAD  (max qfn-future frames before drop, BSP window=64)"
echo "  CALYPSO_ICOUNT              = $CALYPSO_ICOUNT  (flag: ${QEMU_ICOUNT_FLAG:-(none)})"
echo "  CALYPSO_MTTCG               = ${CALYPSO_MTTCG:-0}  $([ "${CALYPSO_MTTCG:-0}" = "1" ] && echo '⚠️ NON-DÉTERMINISTE — données non-citables pour state claims (cf session 2026-05-25)' || echo '(icount-deterministic ✓)')"
echo "  CALYPSO_TIMER               = $CALYPSO_TIMER  (1=fprintf tdma_tick/frame_irq/kick → qemu.log, 0=silent)"
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
