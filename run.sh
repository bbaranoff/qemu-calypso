#!/bin/bash
# run.sh -- Calypso QEMU pipeline.
#
# Data path : osmo-bts-trx : osmo-trx-ipc : calypso-ipc-device : QEMU BSP
# DMA : DSP CCCH demod : a_cd[] in NDB : ARM L1 : L1CTL_DATA_IND.
# Modem GMSK integre dans osmo-trx (chaine radio software-defined complete).

set -euo pipefail

SESSION="calypso"

# ---- args : --gen-doc / --help -----------------------------------------------
# --gen-doc : ne (re)lance PAS le pipeline ; suppose qu'il tourne deja dans le
# container et lance pytest pour produire la doc (report.md condense +
# test_results.md/qmd complets). Cree une fenetre tmux `gen-doc` dans la session
# `calypso` existante et y bascule pour montrer le run pytest en direct.
GEN_DOC_MODE=0
MENU_MODE=0
DEBUG_FULL_MODE=0
for arg in "$@"; do
    case "$arg" in
        --gen-doc-local) GEN_DOC_MODE=1 ;;
        --menu) MENU_MODE=1 ;;
        --debug-full) DEBUG_FULL_MODE=1 ;;
        --run-all)
            echo "[run.sh] --run-all : delegating to run-all.sh"
            exec "$(dirname "$0")/run-all.sh" "${@:2}"
            ;;
        --run-all-debug)
            echo "[run.sh] --run-all-debug : delegating to run-all-debug.sh"
            exec "$(dirname "$0")/run-all-debug.sh" "${@:2}"
            ;;
        --select-verdict)
            echo "[run.sh] --select-verdict : delegating to select-verdict.sh"
            exec "$(dirname "$0")/select-verdict.sh" "${@:2}"
            ;;
        -h|--help)
            cat <<'EOF'
Usage: ./run.sh [FLAG]

Lance le pipeline Calypso complet : QEMU (ARM + DSP c54x emulé) + osmocon
(romload stub) + bridge ipc-device|trx-ipc|bridge.py + osmo-bts-trx +
mobile/L2 client + gsmtap capture + irda capture. Tout dans une session
tmux 'calypso'. Par défaut attache la session en fin.

═══════════════════════════════════════════════════════════════════════════
FLAGS
═══════════════════════════════════════════════════════════════════════════

  --menu         Sélection interactive (whiptail) du profile avant launch.
                 Mode, components, L2 client, DIAG profile, advanced toggles.

  --debug-full   Active TOUTES les probes debug (CALYPSO_*_TRACE/PROBE)
                 sur le mode courant (default 'full', ou celui sélectionné
                 via --menu). icount=auto + DIAG full + gdb pane on.
                 Lourd en log (~1-2 GB/min qemu.log). SINGLE-SHOT.

  --gen-doc-local  Skip launch, lance pytest in-container, génère doc.
                   Artefacts /tmp/report.md, test_results.md, .qmd, etc.

  --run-all      Sweep tous les modes (full shunt shunt-ipc bridge bare)
                 via run-all.sh, 60s chacun, compare outputs. BENCH.
  --run-all-debug  Idem run-all mais debug-friendly : 180s/mode, logs
                   verbose, sessions tmux gardées. Ajouter --full pour
                   activer toutes les c54x probes (équiv. --debug-full
                   appliqué à chaque mode). COVERAGE + BENCH.
  --select-verdict  Outil de sélection verdict pour tests.

  -h, --help     Affiche ce help.

═══════════════════════════════════════════════════════════════════════════
ENV VARS (override default)
═══════════════════════════════════════════════════════════════════════════

QEMU accel:
  CALYPSO_ICOUNT=auto|off|shift=N,sleep=on   default auto (depuis 2026-05-27
                                              SIM fix)
  CALYPSO_MTTCG=0|1                          multi-thread TCG (XOR icount)
  CALYPSO_QEMU_HALT=0|1                      -S au launch (gdb attach avant
                                              boot, BPs fire dès reset)

Pipeline:
  CALYPSO_L2_CLIENT=mobile|ccch_scan|cell_log      L2 app
  CALYPSO_SKIP_IPC_DEVICE|TRX_IPC|BTS|L2|GSMTAP|BRIDGE_PY=0|1
  CALYPSO_DSP_SHUNT=0|1                            canned FB+SB shunt (skip DSP)
  CALYPSO_DSP_L1STUB=0|1                           patch PROM0@0x7120 = publisher
                                                   L1+L2 (FB+PM+SB+SI3 continus,
                                                   reste du ROM/blobs intact) →
                                                   le mobile campe sans I/Q.
                                                   Gère make_dsp_bin_L1.py + PROM0.
  CALYPSO_FORCE_TOA=<N>                             force a_sync_TOA lu par l'ARM
                                                   à N (ex 23) — env gated, rigolo.
  CALYPSO_IRDA_CAPTURE=0|1                         /tmp/fw-irda.log
  CALYPSO_BSP_IQ_PASSTHROUGH=0|1                   BSP DL soft-bits → GMSK IQ
  CALYPSO_NO_ATTACH=0|1                            skip final tmux attach

Tmux panes:
  CALYPSO_SKIP_GDB_PANE=0|1                  gdb pane in window 'all'
                                              (default 1 = skip)
  CALYPSO_GDB_PANE_SCRIPT=/path/to.gdb       auto-load gdb script
  CALYPSO_GDB_ELF=/path/to/firmware.elf      default highram.elf
  CALYPSO_GDB_PORT=N                          default 1234

Diag/probes  ->  TOUT via UNE seule var : CALYPSO_DEBUG
  CALYPSO_DEBUG=ALL            active toutes les sondes (= --debug-full)
  CALYPSO_DEBUG=tok1,tok2,...  liste de tokens (insensible casse ; - _ . / =>_)
  CALYPSO_DEBUG (unset)        aucune sonde (silencieux, production-like)
  Masters composant : C54X (tous C54_LOG c54x) | BSP | TRX | IOTA | PCB
  Tokens c54x (extrait) :
    SP-WATCH SP-RING SP-ABS SP-HIST SP-LOW SP-GUARD SP-CATASTROPHE STACK-IN-NDB
    INTM-TRANS INT3-CYCLE INT3-BLOCKED RSBX-INTM IMR-W ST1-WR POPM-ST1 SP-DRAIN
    BOOT-BRANCH BOOTSTUB BOOTSTUB-ENTRY DISPATCH-ENTRY IRQ-FRAME-HEALTH XC-COND
    FBDB CORRELATOR MVPD BITF-PROBE WATCH-3FBE RXDONE TIMER STUCK STUCK-HIST
    AR-TRACE AR6-AT AR7-HIST A-TRACE XPC-STATS XPC1-PC-RING DARAM UPPER-DARAM
    NDB-CTL-WR A_SCH-WR A_CD-BY-BURST D_TASK_D-WR D_BURST_D-WR DROM-W-DROP ...
  Params couples a un token (valeur, pas on/off) :
    CALYPSO_AR_TRACE=0xFF (mask AR, token AR-TRACE)  CALYPSO_A_TRACE_PC=0xPC
    CALYPSO_AR6_AT_PC/WIN_LO/WIN_HI/AT_LOG_CAP       CALYPSO_MVPD_BOOT_LIMIT=N
    CALYPSO_SP_RING_MAX/TRIG/INSN_MIN                CALYPSO_SP_HIST_ARM/DUMP=0xNNNN
    token TRAP-OOR + CALYPSO_TRAP_CHECKPOINT=N
  Liste exhaustive des tokens :
    grep -rho 'calypso_debug_enabled("[^"]*"' hw/arm/calypso | sort -u
  Autres debug (hors CALYPSO_DEBUG) :
    CALYPSO_DBG=corrupt,unimpl,...   masque debug generique c54x (calypso_dbg.c)
    CALYPSO_UART_TRACE=1             trace UART (hw/char/calypso_uart.c)


Overrides numériques:
  CALYPSO_FORCE_INTM_AT_PC=0xPC      force INTM clear when PC == valeur
  CALYPSO_FORCE_INTM_CLEAR_AT=N      insn count threshold
  CALYPSO_A_TRACE_PC=0xPC            trace A writes from this PC
  CALYPSO_NDB_D_RACH_OFFSET=0xNNN    DSP NDB d_rach offset (default 0x1CB)
  CALYPSO_RACH_FORCE_BSIC=N          force BSIC 0..63 in RACH encoder
  CALYPSO_STICK_ARFCN=N              pin mobile cfg ARFCN
  CALYPSO_BSP_DARAM_ADDR=0xNNNN      BSP DARAM addr (default 0x2a00, canary-validated 2026-05-28)
  CALYPSO_BSP_HOST=ip / CALYPSO_BSP_PORT=N   BSP UDP endpoint (def 6702)
  CALYPSO_TRAP_CHECKPOINT=...        c54x trap on checkpoint
  CALYPSO_TRAP_OOR=...               c54x trap out-of-range
  CALYPSO_TWL3025_AFC_HZ=N           force AFC offset Hz (TWL3025/Iota ABB, def 0=off)
  CALYPSO_DOPPLER_HZ=N               Doppler injection Hz dans bridge IQ
                                     (def 0 = no-op ; ex 200 = 200Hz drift)
  CALYPSO_SAMPLE_RATE=N              Hz sample rate (def 270833 = 1sps GSM)
  CALYPSO_BURST_PRINT=0|1            dump bursts I/Q (def 0 ; off recommend.)
  CALYPSO_BURST_MAX=N                cap dumps (def 50)
  CALYPSO_BURST_HEAD=N               nb premiers I/Q affichés (def 12)
  CALYPSO_BRIDGE_PYTHON=path         python avec gnuradio/gr-gsm/numpy
                                     (def /root/.env/bin/python3)

═══════════════════════════════════════════════════════════════════════════
EXEMPLES
═══════════════════════════════════════════════════════════════════════════

  ./run.sh                              # default (icount=auto, no probes)
  ./run.sh --menu                       # interactive setup
  ./run.sh --debug-full                 # all probes ON, gdb pane visible
  CALYPSO_L2_CLIENT=ccch_scan ./run.sh  # CCCH scanner au lieu de mobile
  CALYPSO_ICOUNT=off ./run.sh           # fallback wall-clock mode
  CALYPSO_NO_ATTACH=1 ./run.sh          # ne pas attacher tmux à la fin

═══════════════════════════════════════════════════════════════════════════
FICHIERS / LOGS
═══════════════════════════════════════════════════════════════════════════

  /root/qemu.log           QEMU + c54x DSP probes
  /tmp/osmocon.log         osmocon (romload + bridge)
  /tmp/bridge.py.log       bridge.py si mode bridge-py
  /tmp/calypso-ipc-device.log  si mode ipc-device
  /tmp/osmo-trx-ipc.log    GMSK modem
  /tmp/bts.log             osmo-bts-trx
  /tmp/mobile.log          mobile L2 (ou ccch_scan/cell_log)
  /tmp/fw-irda.log         IrDA capture (firmware D11)
  /tmp/mobile-gsmtap.pcap  GSMTAP capture (port 4729)

  Attach tmux : tmux a -t calypso
  Kill session : tmux kill-session -t calypso

EOF
            exit 0 ;;
    esac
done

# ---- --debug-full : export toutes les probes en une commande ----
if [ "$DEBUG_FULL_MODE" = "1" ]; then
    echo "[run.sh] --debug-full : CALYPSO_DEBUG=ALL (toutes les sondes)"
    export CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-off}"
    export CALYPSO_L2_CLIENT="${CALYPSO_L2_CLIENT:-mobile}"
    # UNE seule var pilote TOUTES les sondes diagnostic (c54x + BSP + TRX + ...).
    export CALYPSO_DEBUG="${CALYPSO_DEBUG:-ALL}"
    # Assists pipeline (non-probe) utiles en debug
    export CALYPSO_IRDA_CAPTURE=1
    export CALYPSO_SKIP_GDB_PANE=0
    export CALYPSO_AUTO_GEN_DOC=0
    echo "[run.sh] --debug-full : CALYPSO_DEBUG=$CALYPSO_DEBUG. Log : /root/qemu.log (1-2 GB/min)"
fi

# `--gen-doc-local` = synonyme de "lance le pipeline normal ET force la
# generation de doc auto" (qui se declenche T+30s apres QEMU launch dans une
# fenetre tmux dediee). Pas de mode "skip pipeline" : si tu veux regenerer la
# doc sans relancer, lance directement pytest dans le container.
if [ "$GEN_DOC_MODE" = "1" ]; then
    CALYPSO_AUTO_GEN_DOC=1
    export CALYPSO_AUTO_GEN_DOC
fi

# ---- --menu : kernel-menuconfig style (whiptail) -----------------------
# Hierarchie 3 niveaux avec heritage et conditions :
#
#   Niveau 1 : Mode general (5 presets + free)
#   Niveau 2 : Components -- pre-coches selon le mode, modifiables
#              (mode "free" = tout decoche par defaut)
#   Niveau 3 : Sub-options conditionnelles a ce qu'on a coche
#              (L2 client variant si L2 ON, icount config si MTTCG off, etc)
#   Niveau 4 : Resume + confirm
#
# Si whiptail absent : fallback text simple.
if [ "$MENU_MODE" = "1" ]; then

  # ---- Fix TERM + locale pour whiptail/newt box-drawing ----
  # Sans ca, le terminal affiche `qqqq` au lieu des bordures (DEC special
  # graphics non-rendus). Le container/tmux a souvent TERM=dumb ou un TERM
  # qui ne declare pas acsc.
  export LANG="${LANG:-C.UTF-8}"
  export LC_ALL="${LC_ALL:-C.UTF-8}"
  case "${TERM:-}" in
    dumb|"" ) export TERM=xterm-256color ;;
  esac
  # Disable NewT's UTF-8 box if locale still broken (fallback ASCII).
  # Decommente si les bordures restent en qqqq apres ce fix :
  # export NCURSES_NO_UTF8_ACS=1

  if ! command -v whiptail >/dev/null 2>&1; then
    echo "[menu] whiptail absent -- fallback text minimal"
    echo "  Modes disponibles : full | shunt | shunt-ipc | bridge | bare | free"
    read -p "  CALYPSO_MODE = " CALYPSO_MODE
    : "${CALYPSO_MODE:=full}"
    export CALYPSO_MODE
  else

    # ---- Theme palettes ----
    # Chaque theme = palette NEWT_COLORS. Choisi par l'user au 1er ecran.
    # Override via env CALYPSO_MENU_THEME=red|green|magenta|cyan|mono|amber.
    _theme_red() {
      # Red list focus + YELLOW button focus -> deux highlights distincts.
      export NEWT_COLORS='
        root=white,blue
        roottext=brightwhite,blue
        window=,lightgray
        border=blue,lightgray
        title=yellow,blue
        button=brightwhite,blue
        compactbutton=brightwhite,blue
        actbutton=black,brightyellow
        checkbox=black,lightgray
        actcheckbox=brightwhite,red
        entry=black,lightgray
        actentry=brightwhite,red
        listbox=black,lightgray
        actlistbox=brightwhite,red
        textbox=black,lightgray
        acttextbox=brightwhite,red
        helpline=brightwhite,blue
        sellistbox=brightwhite,red
        actsellistbox=brightwhite,red
      '
    }
    _theme_green() {
      export NEWT_COLORS='
        root=brightgreen,black
        roottext=brightgreen,black
        window=,black
        border=brightgreen,black
        title=brightyellow,black
        button=brightgreen,black
        compactbutton=green,black
        actbutton=black,brightyellow
        checkbox=brightgreen,black
        actcheckbox=black,brightgreen
        entry=brightgreen,black
        actentry=black,brightgreen
        listbox=brightgreen,black
        actlistbox=black,brightgreen
        textbox=brightgreen,black
        acttextbox=black,brightgreen
        helpline=brightgreen,black
        sellistbox=black,brightgreen
        actsellistbox=black,brightgreen
      '
    }
    _theme_magenta() {
      export NEWT_COLORS='
        root=white,magenta
        roottext=brightwhite,magenta
        window=,lightgray
        border=magenta,lightgray
        title=brightyellow,magenta
        button=black,lightgray
        compactbutton=black,lightgray
        actbutton=black,brightyellow
        checkbox=black,lightgray
        actcheckbox=brightwhite,magenta
        entry=black,lightgray
        actentry=brightwhite,magenta
        listbox=black,lightgray
        actlistbox=brightwhite,magenta
        textbox=black,lightgray
        acttextbox=brightwhite,magenta
        helpline=brightwhite,magenta
        sellistbox=brightwhite,magenta
        actsellistbox=brightwhite,magenta
      '
    }
    _theme_cyan() {
      export NEWT_COLORS='
        root=brightwhite,blue
        roottext=brightwhite,blue
        window=,lightgray
        border=cyan,lightgray
        title=brightyellow,cyan
        button=black,lightgray
        compactbutton=black,lightgray
        actbutton=black,brightyellow
        checkbox=black,lightgray
        actcheckbox=black,brightcyan
        entry=black,lightgray
        actentry=black,brightcyan
        listbox=black,lightgray
        actlistbox=black,brightcyan
        textbox=black,lightgray
        acttextbox=black,brightcyan
        helpline=brightwhite,blue
        sellistbox=black,brightcyan
        actsellistbox=black,brightcyan
      '
    }
    _theme_mono() {
      export NEWT_COLORS='
        root=white,black
        roottext=white,black
        window=,white
        border=black,white
        title=black,white
        button=black,white
        compactbutton=black,white
        actbutton=white,black
        checkbox=black,white
        actcheckbox=white,black
        entry=black,white
        actentry=white,black
        listbox=black,white
        actlistbox=white,black
        textbox=black,white
        acttextbox=white,black
        helpline=white,black
        sellistbox=white,black
        actsellistbox=white,black
      '
    }
    _theme_cool() {
      # Default cool theme : navy + cyan listbox accents, JAUNE pour focus
      # buttons -> distinct de actlistbox (qui reste cyan).
      export NEWT_COLORS='
        root=brightcyan,blue
        roottext=brightwhite,blue
        window=,black
        border=brightcyan,black
        title=brightyellow,blue
        button=brightwhite,blue
        compactbutton=brightwhite,blue
        actbutton=black,brightyellow
        checkbox=brightcyan,black
        actcheckbox=black,brightcyan
        entry=brightwhite,black
        actentry=black,brightcyan
        listbox=brightwhite,black
        actlistbox=black,brightcyan
        textbox=brightwhite,black
        acttextbox=black,brightcyan
        helpline=brightyellow,blue
        sellistbox=black,brightcyan
        actsellistbox=black,brightcyan
      '
    }
    _theme_amber() {
      export NEWT_COLORS='
        root=brightyellow,black
        roottext=brightyellow,black
        window=,black
        border=brightyellow,black
        title=brightwhite,brightyellow
        button=brightyellow,black
        compactbutton=yellow,black
        actbutton=black,brightyellow
        checkbox=brightyellow,black
        actcheckbox=black,brightyellow
        entry=brightyellow,black
        actentry=black,brightyellow
        listbox=brightyellow,black
        actlistbox=black,brightyellow
        textbox=brightyellow,black
        acttextbox=black,brightyellow
        helpline=brightyellow,black
        sellistbox=black,brightyellow
        actsellistbox=black,brightyellow
      '
    }
    # Default theme = red (high contrast). Override via env CALYPSO_MENU_THEME.
    case "${CALYPSO_MENU_THEME:-red}" in
      cool)    _theme_cool ;;
      red)     _theme_red ;;
      green)   _theme_green ;;
      magenta) _theme_magenta ;;
      cyan)    _theme_cyan ;;
      mono)    _theme_mono ;;
      amber)   _theme_amber ;;
    esac
    BACKTITLE="Calypso QEMU -- DSP shunt / IPC / Bridge -- menuconfig style"

    _cancel() { echo "[menu] cancelled by user" >&2; exit 0; }

    # ---- 0) THEME picker (skip si CALYPSO_MENU_THEME deja set par env) ----
    if [ -z "${CALYPSO_MENU_THEME:-}" ]; then
      CHOSEN_THEME=$(whiptail --backtitle "$BACKTITLE" \
        --title "[0/4] Theme" \
        --notags --menu \
        "\n Choisir le theme du menu (ENTER pour selectionner).\n" \
        17 72 7 \
        "red"     "Red       -- focus blanc sur rouge (default, high contrast)" \
        "green"   "Green     -- matrix-style, fond noir" \
        "cool"    "Cool      -- navy + cyan accents, sleek" \
        "magenta" "Magenta   -- focus blanc sur magenta" \
        "cyan"    "Cyan      -- focus noir sur cyan brillant" \
        "amber"   "Amber     -- terminal vintage jaune/noir" \
        "mono"    "Mono      -- noir et blanc seulement" \
        3>&1 1>&2 2>&3) || _cancel
      export CALYPSO_MENU_THEME="$CHOSEN_THEME"
      # Re-apply le theme choisi (la palette par defaut etait deja set
      # plus haut mais le user vient d'en choisir une autre).
      case "$CALYPSO_MENU_THEME" in
        cool)    _theme_cool ;;
        red)     _theme_red ;;
        green)   _theme_green ;;
        magenta) _theme_magenta ;;
        cyan)    _theme_cyan ;;
        mono)    _theme_mono ;;
        amber)   _theme_amber ;;
      esac
    fi

    # ---- 1) MODE general (--menu : ENTER directly selects) ----
    CALYPSO_MODE=$(whiptail --backtitle "$BACKTITLE" \
      --title "[1/4] General mode" \
      --default-item "full-grgsm" \
      --notags --menu \
      "\n Pick a preset (ENTER on item to select). Level 2 inherits it.\n\n \
 Fine override: env vars CALYPSO_SKIP_*, CALYPSO_DSP_SHUNT, ...\
" \
      20 84 7 \
      "full-grgsm" "gr-gsm = le DSP (archi par defaut) -- IPC pour BTS seul" \
      "full"       "Full radio pipeline + vrai c54x -- TO DEV / WIP (chantier)" \
      "shunt"      "DSP shunt canned -- bissection FBSB" \
      "shunt-ipc"  "DSP shunt + radio chain" \
      "bridge"     "Legacy bridge.py Python" \
      "bare"       "QEMU + osmocon only" \
      "free"       "Free mode -- pick everything yourself" \
      3>&1 1>&2 2>&3) || _cancel
    export CALYPSO_MODE

    # ---- Pre-cochage des composants selon le mode (heritage visible) ----
    # _PRE_X = "ON"/"OFF" selon le mode, sert de defaut au niveau 2.
    case "$CALYPSO_MODE" in
      full)
        _PRE_SHUNT=OFF; _PRE_IPC=ON;  _PRE_TRX=ON;  _PRE_BRIDGE=OFF
        _PRE_BTS=ON;    _PRE_L2=ON;   _PRE_GSMTAP=ON; _PRE_IRDA=ON
        _PRE_DOC=ON;    _PRE_MTTCG=OFF
        ;;
      full-grgsm)
        # qemu gardé (fw+CP210x), DSP SHUNT ON (c54x canné léger). Mobile =
        # trxcon + gr-gsm (radio). L2 ON (L1CTL via trxcon, pas osmocon).
        _PRE_SHUNT=ON;  _PRE_IPC=ON;  _PRE_TRX=ON;  _PRE_BRIDGE=OFF
        _PRE_BTS=ON;    _PRE_L2=ON;   _PRE_GSMTAP=OFF; _PRE_IRDA=OFF
        _PRE_DOC=OFF;   _PRE_MTTCG=OFF
        ;;
      shunt)
        _PRE_SHUNT=ON;  _PRE_IPC=OFF; _PRE_TRX=OFF; _PRE_BRIDGE=OFF
        _PRE_BTS=OFF;   _PRE_L2=ON;   _PRE_GSMTAP=OFF; _PRE_IRDA=ON
        _PRE_DOC=ON;    _PRE_MTTCG=OFF
        ;;
      shunt-ipc)
        _PRE_SHUNT=ON;  _PRE_IPC=ON;  _PRE_TRX=ON;  _PRE_BRIDGE=OFF
        _PRE_BTS=ON;    _PRE_L2=ON;   _PRE_GSMTAP=ON; _PRE_IRDA=ON
        _PRE_DOC=ON;    _PRE_MTTCG=OFF
        ;;
      bridge)
        _PRE_SHUNT=OFF; _PRE_IPC=OFF; _PRE_TRX=OFF; _PRE_BRIDGE=ON
        _PRE_BTS=ON;    _PRE_L2=ON;   _PRE_GSMTAP=ON; _PRE_IRDA=ON
        _PRE_DOC=ON;    _PRE_MTTCG=OFF
        ;;
      bare)
        _PRE_SHUNT=OFF; _PRE_IPC=OFF; _PRE_TRX=OFF; _PRE_BRIDGE=OFF
        _PRE_BTS=OFF;   _PRE_L2=OFF;  _PRE_GSMTAP=OFF; _PRE_IRDA=ON
        _PRE_DOC=OFF;   _PRE_MTTCG=OFF
        ;;
      free)
        _PRE_SHUNT=OFF; _PRE_IPC=OFF; _PRE_TRX=OFF; _PRE_BRIDGE=OFF
        _PRE_BTS=OFF;   _PRE_L2=OFF;  _PRE_GSMTAP=OFF; _PRE_IRDA=OFF
        _PRE_DOC=OFF;   _PRE_MTTCG=OFF
        ;;
    esac

    # ---- 2) COMPONENTS -- checklist heritee du mode ----
    # Une boucle de validation : si conflits mutex detectes, msg + retry.
    _comp_dialog() {
      whiptail --backtitle "$BACKTITLE" \
        --title "[2/4] Components (mode herite: $CALYPSO_MODE)" \
        --notags --checklist \
        "\n ESPACE = toggle, TAB = OK/Cancel\n\n \
 Coche par defaut selon le mode choisi.\n \
 Mutex auto-resolus a la sortie :\n  * IPC_DEVICE+TRX_IPC  XOR  BRIDGE_PY (pipeline radio)\n  * MTTCG  XOR  icount auto\n" \
        24 82 11 \
        "DSP_SHUNT"  "DSP mock -- skip c54x, ARM-only (canned FB+SB)" $_PRE_SHUNT \
        "IPC_DEVICE" "calypso-ipc-device -- UDP 6702 <-> shm bridge" $_PRE_IPC \
        "TRX_IPC"    "osmo-trx-ipc -- GMSK modem + TRXD vers BTS" $_PRE_TRX \
        "BRIDGE_PY"  "bridge.py -- pont Python legacy (mutex IPC)" $_PRE_BRIDGE \
        "BTS"        "osmo-bts-trx -- BTS radio + RSL vers BSC" $_PRE_BTS \
        "L2"         "L2 client -- mobile/ccch_scan/cell_log" $_PRE_L2 \
        "GSMTAP"     "tcpdump GSMTAP capture vers pcap" $_PRE_GSMTAP \
        "IRDA"       "IrDA serial1 capture (fw printf debug)" $_PRE_IRDA \
        "DOC"        "Auto-gen doc T+60s (pytest in-container)" $_PRE_DOC \
        "MTTCG"      "MTTCG multi-thread TCG (non-deterministe)" $_PRE_MTTCG \
        3>&1 1>&2 2>&3
    }

    # Pattern-match plus robuste que grep (gere bien le case " *" avec quotes)
    _has() { case " $COMPS " in *"\"$1\""*) return 0 ;; *) return 1 ;; esac; }

    while true; do
      COMPS=$(_comp_dialog) || _cancel

      # Mutex check : bridge.py XOR (ipc-device | trx-ipc)
      if _has BRIDGE_PY && { _has IPC_DEVICE || _has TRX_IPC; }; then
        whiptail --backtitle "$BACKTITLE" --title "Conflit mutex" --msgbox \
          "\n BRIDGE_PY est mutuellement exclusif avec IPC_DEVICE et TRX_IPC.\n\n \
 C'est l'ancien chemin Python (bridge.py) vs le moderne (osmo-trx-ipc).\n \
 Un seul peut tourner a la fois -- decoche l'un OU l'autre.\n" \
          14 76 3>&1 1>&2 2>&3 || _cancel
        # On reload avec les pre-cochages mis a jour pour montrer l'etat actuel
        _has DSP_SHUNT  && _PRE_SHUNT=ON  || _PRE_SHUNT=OFF
        _has IPC_DEVICE && _PRE_IPC=ON    || _PRE_IPC=OFF
        _has TRX_IPC    && _PRE_TRX=ON    || _PRE_TRX=OFF
        _has BRIDGE_PY  && _PRE_BRIDGE=ON || _PRE_BRIDGE=OFF
        _has BTS        && _PRE_BTS=ON    || _PRE_BTS=OFF
        _has L2         && _PRE_L2=ON     || _PRE_L2=OFF
        _has GSMTAP     && _PRE_GSMTAP=ON || _PRE_GSMTAP=OFF
        _has IRDA       && _PRE_IRDA=ON   || _PRE_IRDA=OFF
        _has DOC        && _PRE_DOC=ON    || _PRE_DOC=OFF
        _has MTTCG      && _PRE_MTTCG=ON  || _PRE_MTTCG=OFF
        continue
      fi

      # Implication : TRX_IPC sans IPC_DEVICE : osmo-trx-ipc va greeting-fail.
      if _has TRX_IPC && ! _has IPC_DEVICE; then
        whiptail --backtitle "$BACKTITLE" --title "Dependance manquante" --msgbox \
          "\n osmo-trx-ipc requiert calypso-ipc-device (master socket).\n\n \
 Sans le device, le greeting handshake fail et osmo-trx-ipc exit.\n \
 Active IPC_DEVICE ou desactive TRX_IPC.\n" \
          12 76 3>&1 1>&2 2>&3 || _cancel
        continue
      fi

      break
    done

    # Parse final : SKIP_* vars
    _has DSP_SHUNT  && CALYPSO_DSP_SHUNT=1        || CALYPSO_DSP_SHUNT=0
    _has IPC_DEVICE && CALYPSO_SKIP_IPC_DEVICE=0  || CALYPSO_SKIP_IPC_DEVICE=1
    _has TRX_IPC    && CALYPSO_SKIP_TRX_IPC=0     || CALYPSO_SKIP_TRX_IPC=1
    _has BRIDGE_PY  && CALYPSO_SKIP_BRIDGE_PY=0   || CALYPSO_SKIP_BRIDGE_PY=1
    _has BTS        && CALYPSO_SKIP_BTS=0         || CALYPSO_SKIP_BTS=1
    _has L2         && CALYPSO_SKIP_L2=0          || CALYPSO_SKIP_L2=1
    _has GSMTAP     && CALYPSO_SKIP_GSMTAP=0      || CALYPSO_SKIP_GSMTAP=1
    _has IRDA       && CALYPSO_IRDA_CAPTURE=1     || CALYPSO_IRDA_CAPTURE=0
    _has DOC        && CALYPSO_AUTO_GEN_DOC=1     || CALYPSO_AUTO_GEN_DOC=0
    _has MTTCG      && CALYPSO_MTTCG=1            || CALYPSO_MTTCG=0

    # ---- 3) SUB-OPTIONS conditionnelles (--menu : ENTER directly selects) ----
    # 3a) L2 variant (si L2 ON)
    if [ "$CALYPSO_SKIP_L2" = "0" ]; then
      CALYPSO_L2_CLIENT=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] L2 client variant" \
        --notags --menu \
        "\n Application qui consomme L1CTL via /tmp/osmocom_l2.\n" \
        14 76 3 \
        "mobile"    "mobile -- osmocom-bb stack L23 + VTY (default)" \
        "ccch_scan" "ccch_scan -- scan CCCH ARFCN, dump SI/IA" \
        "cell_log"  "cell_log -- scan cells + power measures" \
        3>&1 1>&2 2>&3) || _cancel
      export CALYPSO_L2_CLIENT
    fi

    # 3b) icount (toujours, sauf si MTTCG ON : forced off)
    if [ "$CALYPSO_MTTCG" = "1" ]; then
      export CALYPSO_ICOUNT=off
      whiptail --backtitle "$BACKTITLE" --title "MTTCG actif" --msgbox \
        "\n MTTCG ON : CALYPSO_ICOUNT force a 'off' (mutex).\n\n \
 Rappel CLAUDE.md session 2026-05-25 :\n MTTCG est NON-DETERMINISTE -- ses donnees ne sont pas\n citables pour des claims d'etat firmware.\n" \
        14 76 3>&1 1>&2 2>&3
    else
      ICOUNT=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] QEMU icount mode" \
        --default-item "off" \
        --notags --menu \
        "\n Pacing horloge virtuelle QEMU.\n" \
        14 76 3 \
        "off"   "off -- wall-clock (default, auto bloque actuellement)" \
        "auto"  "auto -- deterministe (broken under load, debug en cours)" \
        "shift" "shift=N -- pacing fixe (N saisi ensuite)" \
        3>&1 1>&2 2>&3) || _cancel
      case "$ICOUNT" in
        auto) export CALYPSO_ICOUNT=auto ;;
        off)  export CALYPSO_ICOUNT=off ;;
        shift)
          while true; do
            SHIFT_N=$(whiptail --backtitle "$BACKTITLE" --title "icount shift" \
              --inputbox "\n Valeur N : entier 1-16 (1<<N instr ~= 1ns)\n" \
              11 60 "8" 3>&1 1>&2 2>&3) || _cancel
            # Validate : entier dans [1, 16]
            if echo "$SHIFT_N" | grep -qE '^[0-9]+$' && [ "$SHIFT_N" -ge 1 ] && [ "$SHIFT_N" -le 16 ]; then
              break
            fi
            whiptail --backtitle "$BACKTITLE" --title "Invalid" --msgbox \
              "\n Valeur invalide : '$SHIFT_N'.\n Attendu : entier entre 1 et 16.\n" \
              10 60 3>&1 1>&2 2>&3
          done
          export CALYPSO_ICOUNT="shift=${SHIFT_N},sleep=on"
          ;;
      esac
    fi

    # 3c) Diag profile
    DIAG=$(whiptail --backtitle "$BACKTITLE" \
      --title "[3/4] Diagnostic profile" \
      --notags --menu \
      "\n Sondes c54x / instrumentation.\n" \
      14 76 4 \
      "none"    "none -- aucune sonde (production-like, reco)" \
      "minimal" "minimal -- INTM-TRANS,SP-WATCH,BOOT-BRANCH" \
      "all"     "all -- CALYPSO_DEBUG=ALL : toutes les sondes (verbeux)" \
      "custom"  "custom -- saisir une liste de tokens CALYPSO_DEBUG" \
      3>&1 1>&2 2>&3) || _cancel

    # 3d) Pytest config (si DOC ON)
    if [ "$CALYPSO_AUTO_GEN_DOC" = "1" ]; then
      PYTEST_VERBOSITY=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] Pytest verbosity" \
        --notags --menu \
        "\n Verbosity du pytest auto-gen-doc (T+60s).\n" \
        14 76 4 \
        "v"   "verbose -- un test par ligne (default)" \
        "q"   "quiet -- resume minimal" \
        "vv"  "very verbose -- details complets" \
        "vvv" "ultra -- debug pytest" \
        3>&1 1>&2 2>&3) || _cancel
      export CALYPSO_PYTEST_VERBOSITY="$PYTEST_VERBOSITY"

      PYTEST_SCOPE=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] Pytest scope" \
        --notags --menu \
        "\n Quels tests executer dans la fenetre gen-doc.\n" \
        14 76 4 \
        "default"   "Default -- exclusion functional/qemu-iotests/etc" \
        "smoke"     "Smoke -- juste test_run_all_modes.py (si JSON present)" \
        "all"       "All -- pas d'exclusion (lent)" \
        "calypso"   "Calypso only -- tests/calypso/ si present" \
        3>&1 1>&2 2>&3) || _cancel
      export CALYPSO_PYTEST_SCOPE="$PYTEST_SCOPE"
    fi
    case "$DIAG" in
      none)
        export CALYPSO_DEBUG=""
        ;;
      minimal)
        export CALYPSO_DEBUG="INTM-TRANS,SP-WATCH,BOOT-BRANCH"
        ;;
      all)
        export CALYPSO_DEBUG="ALL"
        ;;
      custom)
        CD=$(whiptail --backtitle "$BACKTITLE" \
          --title "[3/4] CALYPSO_DEBUG tokens" \
          --inputbox "\n Liste de tokens separes par virgule.\n ALL = tout. Vide = aucune sonde.\n Ex: SP-WATCH,INTM-TRANS,FBDB,CORRELATOR\n" \
          13 76 "" 3>&1 1>&2 2>&3) || _cancel
        [ -n "$CD" ] && export CALYPSO_DEBUG="$CD"
        ;;
    esac

    # ---- 3e) Advanced debug (optionnel, defaut OFF) ----
    # Expose 25 env vars C-side jamais exposees en menu auparavant. Gated
    # par yes/no pour ne pas polluer le flow normal. Cf. inventaire complet :
    # grep -r CALYPSO_ qemu-src + comm vs run.sh.
    if whiptail --backtitle "$BACKTITLE" --title "[3/4] Advanced debug" \
        --defaultno --yesno \
        "\n Acceder aux options de debug avancees ?\n\n Probes/traces fines, overrides, watchpoints, BSP network.\n Par defaut OFF (vide => env unset)." \
        12 70 3>&1 1>&2 2>&3; then

      # --- Trace toggles (checklist) ---
      ADV_TRACES=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] Toggles comportementaux / pipeline" \
        --notags --separate-output --checklist \
        "\n Toggles NON-probe (les sondes = profil DIAG / CALYPSO_DEBUG).\n ON par defaut = comportement nominal.\n" \
        18 78 8 \
        "DSP_IDLE_FF"        "DSP idle fast-forward (perf, nominal)"          ON \
        "W1C_LATCH"          "W1C latch a_sync_demod (nominal, requis FBSB)"  ON \
        "BSP_IQ_PASSTHROUGH" "BSP DL soft-bits -> GMSK IQ (nominal)"          ON \
        "FORCE_INTM_ONESHOT" "Clear INTM oneshot (sonde, NON-nominal)"        OFF \
        "UART_TRACE"         "UART read/write trace"                          OFF \
        "DBG"                "Generic c54x dbg mask (CALYPSO_DBG)"            OFF \
        "QEMU_HALT"          "QEMU start halted (-S), gdb avant boot"         OFF \
        "GDB_PANE"           "Tmux window gdb"                                OFF \
        3>&1 1>&2 2>&3) || _cancel
      # Toggles ON-par-defaut : reset a 0 puis re-applique la selection
      # (separate-output ne liste que les coches).
      export CALYPSO_DSP_IDLE_FF=0 CALYPSO_W1C_LATCH=0 CALYPSO_BSP_IQ_PASSTHROUGH=0
      for tr in $ADV_TRACES; do
        case "$tr" in
          GDB_PANE)  export CALYPSO_SKIP_GDB_PANE=0 ;;
          DBG)       export CALYPSO_DBG=1 ;;
          *)         export "CALYPSO_${tr}=1" ;;
        esac
      done

      # --- Override numerique : FORCE_INTM_AT_PC ---
      INTM_PC=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] FORCE_INTM_AT_PC (hex, vide=off)" \
        --inputbox "\n Forcer le clear INTM quand PC == valeur.\n Vide ou 0 = desactive.\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$INTM_PC" ] && [ "$INTM_PC" != "0" ] && export CALYPSO_FORCE_INTM_AT_PC="$INTM_PC"

      # --- Override numerique : A_TRACE_PC ---
      ATRACE=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] A_TRACE_PC (hex, vide=off)" \
        --inputbox "\n Trace A writes quand PC == valeur.\n Vide ou 0 = desactive.\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$ATRACE" ] && [ "$ATRACE" != "0" ] && export CALYPSO_A_TRACE_PC="$ATRACE"

      # --- Override numerique : NDB_D_RACH_OFFSET ---
      RACH_OFF=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] NDB_D_RACH_OFFSET (hex, vide=defaut 0x1CB)" \
        --inputbox "\n Override DSP NDB d_rach word offset.\n Default = 0x1CB (firmware DSP==33 layout).\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$RACH_OFF" ] && export CALYPSO_NDB_D_RACH_OFFSET="$RACH_OFF"

      # --- Override numerique : RACH_FORCE_BSIC ---
      BSIC=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] RACH_FORCE_BSIC (0..63, vide=off)" \
        --inputbox "\n Force BSIC dans RACH encoder.\n Match osmo-bsc.cfg base_station_id_code.\n Vide = override OFF.\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$BSIC" ] && export CALYPSO_RACH_FORCE_BSIC="$BSIC"

      # --- BSP network override (host:port) ---
      BSP_NET=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] BSP host:port (vide=defaut 127.0.0.1:6702)" \
        --inputbox "\n Override BSP UDP endpoint.\n Format : host:port. Vide = defaut.\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      if [ -n "$BSP_NET" ]; then
        export CALYPSO_BSP_HOST="${BSP_NET%%:*}"
        export CALYPSO_BSP_PORT="${BSP_NET##*:}"
      fi

      # --- Watchpoint : TRAP_CHECKPOINT ---
      TRAP_CP=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] TRAP_CHECKPOINT (vide=off)" \
        --inputbox "\n Trap c54x sur checkpoint (format module-specific).\n Vide = off.\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$TRAP_CP" ] && export CALYPSO_TRAP_CHECKPOINT="$TRAP_CP"

      # --- Override numerique : STICK_ARFCN (deja existant en run.sh) ---
      STICK=$(whiptail --backtitle "$BACKTITLE" \
        --title "[3/4] STICK_ARFCN (mobile cfg override, vide=defaut)" \
        --inputbox "\n Pin ARFCN pour mobile.cfg.\n Vide = pas d'override (mobile.cfg origine).\n" \
        12 70 "" 3>&1 1>&2 2>&3) || _cancel
      [ -n "$STICK" ] && export CALYPSO_STICK_ARFCN="$STICK"
    fi  # fin Advanced debug

    export CALYPSO_DSP_SHUNT CALYPSO_MODE CALYPSO_MTTCG \
           CALYPSO_IRDA_CAPTURE CALYPSO_AUTO_GEN_DOC
    export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
           CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY

    # Gate DATA_IND direct (court-circuit l1ctl_inject) : OFF par defaut (le SI
    # passe UNIQUEMENT par a_cd -> firmware L1 -> UART, le vrai chemin).
    : "${CALYPSO_SHUNT_DL_INJECT:=0}"
    export CALYPSO_SHUNT_DL_INJECT

    # ---- 3f) Expert : editer TOUTE variable CALYPSO_* (menuconfig-style) ----
    if whiptail --backtitle "$BACKTITLE" --title "[3/4] Expert : toutes les variables" \
        --defaultno --yesno \
        "\n Editer n'importe quelle variable CALYPSO_* ?\n\n Liste complete + edition unitaire (vide = unset).\n Les modes de base restent ceux choisis plus haut." \
        12 72 3>&1 1>&2 2>&3; then
      _ALLVARS="CALYPSO_DEBUG CALYPSO_MODE CALYPSO_ICOUNT CALYPSO_MTTCG \
CALYPSO_QEMU_HALT CALYPSO_L2_CLIENT CALYPSO_DSP_SHUNT CALYPSO_SHUNT_DL_INJECT CALYPSO_BSP_IQ_PASSTHROUGH \
CALYPSO_IRDA_CAPTURE CALYPSO_W1C_LATCH CALYPSO_DSP_IDLE_FF CALYPSO_DSP_IDLE_RANGE \
CALYPSO_TDMA_REALTIME CALYPSO_FORCE_INTM_ONESHOT CALYPSO_FORCE_INTM_AT_PC \
CALYPSO_BSP_DARAM_ADDR CALYPSO_NDB_D_RACH_OFFSET CALYPSO_RACH_FORCE_BSIC \
CALYPSO_STICK_ARFCN CALYPSO_BSP_HOST CALYPSO_BSP_PORT CALYPSO_TRAP_OOR \
CALYPSO_TRAP_CHECKPOINT CALYPSO_AR_TRACE CALYPSO_A_TRACE_PC CALYPSO_AR6_AT_PC \
CALYPSO_AR6_WIN_LO CALYPSO_AR6_WIN_HI CALYPSO_MVPD_BOOT_LIMIT CALYPSO_SP_RING_MAX \
CALYPSO_SP_RING_TRIG CALYPSO_SP_RING_INSN_MIN CALYPSO_SP_HIST_ARM CALYPSO_SP_HIST_DUMP \
CALYPSO_DBG CALYPSO_UART_TRACE CALYPSO_TWL3025_AFC_HZ CALYPSO_DOPPLER_HZ CALYPSO_SAMPLE_RATE \
CALYPSO_BURST_PRINT CALYPSO_BURST_MAX CALYPSO_BURST_HEAD CALYPSO_BRIDGE_PYTHON \
CALYPSO_SKIP_GDB_PANE CALYPSO_NO_ATTACH CALYPSO_AUTO_GEN_DOC \
CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS CALYPSO_SKIP_L2 \
CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY"
      while true; do
        _items=()
        for _v in $_ALLVARS; do _items+=("$_v" "= ${!_v:-(unset)}"); done
        _items+=("DONE" "<< terminer l'edition expert")
        _sel=$(whiptail --backtitle "$BACKTITLE" \
          --title "[3/4] Expert variables (ENTER = editer)" \
          --menu "\n Selectionne une variable (nom = valeur), ENTER pour editer.\n" \
          24 80 16 "${_items[@]}" 3>&1 1>&2 2>&3) || break
        [ "$_sel" = "DONE" ] && break
        _new=$(whiptail --backtitle "$BACKTITLE" --title "$_sel" \
          --inputbox "\n Nouvelle valeur pour $_sel.\n Vide = unset (revient au defaut).\n" \
          11 76 "${!_sel:-}" 3>&1 1>&2 2>&3) || continue
        if [ -z "$_new" ]; then unset "$_sel"; else export "$_sel=$_new"; fi
      done
    fi

    # ---- 4) RESUME + confirm ----
    _ind() { [ "$1" = "0" ] && echo "[ ]" || echo "[X]"; }
    _dsp_label() { [ "$CALYPSO_DSP_SHUNT" = "1" ] && echo "(canned FB+SB)" || echo "(c54x emule)"; }
    DSP_ON="$CALYPSO_DSP_SHUNT"
    IPC_ON=$((1 - CALYPSO_SKIP_IPC_DEVICE))
    TRX_ON=$((1 - CALYPSO_SKIP_TRX_IPC))
    BRG_ON=$((1 - CALYPSO_SKIP_BRIDGE_PY))
    BTS_ON=$((1 - CALYPSO_SKIP_BTS))
    L2_ON=$((1 - CALYPSO_SKIP_L2))
    GSM_ON=$((1 - CALYPSO_SKIP_GSMTAP))

    SUMMARY=" Mode             : $CALYPSO_MODE"
    SUMMARY+=$'\n'" Diag profile     : $DIAG"
    SUMMARY+=$'\n'" L2 variant       : ${CALYPSO_L2_CLIENT:-(skipped)}"
    SUMMARY+=$'\n'" Pytest           : verb=${CALYPSO_PYTEST_VERBOSITY:-(off)} scope=${CALYPSO_PYTEST_SCOPE:-(off)}"
    SUMMARY+=$'\n'
    SUMMARY+=$'\n'"  Components"
    SUMMARY+=$'\n'"   $(_ind $DSP_ON) DSP shunt        $(_dsp_label)"
    SUMMARY+=$'\n'"   $(_ind $IPC_ON) ipc-device       (UDP<->shm bridge)"
    SUMMARY+=$'\n'"   $(_ind $TRX_ON) osmo-trx-ipc     (GMSK modem)"
    SUMMARY+=$'\n'"   $(_ind $BRG_ON) bridge.py        (Python legacy)"
    SUMMARY+=$'\n'"   $(_ind $BTS_ON) osmo-bts-trx     (BTS radio)"
    SUMMARY+=$'\n'"   $(_ind $L2_ON) L2 client        (${CALYPSO_L2_CLIENT:---})"
    SUMMARY+=$'\n'"   $(_ind $GSM_ON) GSMTAP capture"
    SUMMARY+=$'\n'"   $(_ind $CALYPSO_IRDA_CAPTURE) IrDA capture"
    SUMMARY+=$'\n'"   $(_ind $CALYPSO_AUTO_GEN_DOC) gen-doc T+60s"
    SUMMARY+=$'\n'
    SUMMARY+=$'\n'"  QEMU accel"
    SUMMARY+=$'\n'"   icount           = $CALYPSO_ICOUNT"
    SUMMARY+=$'\n'"   MTTCG            = $CALYPSO_MTTCG"

    whiptail --backtitle "$BACKTITLE" --title "[4/4] Confirm launch" --yesno \
      "$SUMMARY" 26 80 3>&1 1>&2 2>&3 || _cancel

  fi  # fin whiptail branch
  echo
  echo "[menu] Lancement avec :"
  env | grep -E '^CALYPSO_' | sort | sed 's/^/  /'
  echo
fi

# Detection pytest reutilisee par la fenetre gen-doc auto plus bas.
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

if false; then  # ancien bloc gen-doc-local, conserve desactive pour reference

    # In-container mode : `/.dockerenv` present : on est DANS le container,
    # lance pytest direct sans prefixe docker.
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
            echo "pytest absent -- tentative d'installation..."
            pip_install_one pytest
            PYTEST_BIN=$(detect_pytest)
        fi
        if [ -z "$PYTEST_BIN" ]; then
            echo "ERR: pytest introuvable et install pip echouee." >&2
            echo "     Installer manuellement : pip install pytest" >&2
            echo "     Ou definir CALYPSO_PYTEST=/path/to/pytest" >&2
            exit 1
        fi
        # pycotap : requis par functional/qemu_test/testcase.py
        python3 -c "import pycotap" 2>/dev/null \
            || { echo "Installing pycotap..."; pip_install_one pycotap; }
        # gdb : module Python qui vient avec GDB system (pas via pip).
        # On verifie que `gdb` (le binaire) est dispo, sinon apt install.
        if ! command -v gdb >/dev/null 2>&1; then
            echo "gdb absent -- tentative d'installation via apt..."
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

        # Si tmux est dispo et qu'on a une session 'calypso' existante : fenetre dediee.
        # Si tmux dispo mais pas de session : on en cree une 'gen-doc-session'.
        # Si pas de tmux du tout : on execute en foreground.
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
            echo "=== --gen-doc-local lance dans tmux '$TARGET:gen-doc' ==="
            if [ "$TARGET" = "gen-doc-session" ]; then
                echo "Attach : tmux attach -t $TARGET"
                # Si terminal interactif, attach directement
                [ -t 0 ] && exec tmux attach -t "$TARGET"
            else
                echo "Attach (si pas deja dedans) : tmux attach -t $TARGET"
            fi
            exit 0
        else
            echo "=== tmux absent -- execution foreground ==="
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
        echo "=== --gen-doc-local lance dans tmux '$SESSION:gen-doc' ==="
        echo "Attach (si pas deja dedans) : tmux attach -t $SESSION"
        exit 0
    else
        echo "=== tmux session '$SESSION' absente -- execution directe ==="
        eval "$GEN_DOC_CMD"
        exit $?
    fi
fi  # ancien bloc gen-doc-local -- desactive par `if false; then` plus haut

# Firmware paths configurable via env (FW_ELF/FW_BIN). Defaults to layer1
# pour compal_e88. run_rssi.sh override these to lancer rssi.highram a la place.
FW_ELF="${FW_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
FW_BIN="${FW_BIN:-/opt/GSM/firmware/board/compal_e88/layer1.highram.bin}"
QEMU="/opt/GSM/qemu-src/build/qemu-system-arm"
OSMOCON="/opt/GSM/osmocom-bb/src/host/osmocon/osmocon"
BTS_CFG="/etc/osmocom/osmo-bts-trx.cfg"
MOBILE_CFG="/root/.osmocom/bb/mobile_group1.cfg"

# Copie la cfg mobile versionnee dans /root/.osmocom/bb/. Elle contient
# deja stick 1 + log stderr avec DMM/DRR/DCC/DSMS debug. Force a chaque
# run pour eviter le drift entre runs.
MOBILE_CFG_VERSIONED="${MOBILE_CFG_VERSIONED:-/opt/GSM/qemu-src/cfgs/mobile_group1.cfg}"
[ -f "$MOBILE_CFG_VERSIONED" ] || MOBILE_CFG_VERSIONED="/home/nirvana/qemu-src/cfgs/mobile_group1.cfg"
if [ -f "$MOBILE_CFG_VERSIONED" ] && [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" = "1" ]; then
    mkdir -p "$(dirname "$MOBILE_CFG")"
    cp "$MOBILE_CFG_VERSIONED" "$MOBILE_CFG"
    echo "[run.sh] mobile_group1.cfg synced from $MOBILE_CFG_VERSIONED"
    # full-grgsm : le mobile communique DIRECT avec osmocon (L1CTL firmware,
    # /tmp/osmocom_l2 = défaut du cfg). trxcon retiré → pas de redirect socket.
fi

# Override stick ARFCN si CALYPSO_STICK_ARFCN set (default = garde cfg versionnee).
CALYPSO_STICK_ARFCN="${CALYPSO_STICK_ARFCN:-}"
if [ -n "$CALYPSO_STICK_ARFCN" ] && [ "$CALYPSO_STICK_ARFCN" != "0" ] && [ -f "$MOBILE_CFG" ]; then
    if grep -qE "^ stick [0-9]+" "$MOBILE_CFG" 2>/dev/null; then
        sed -i "s/^ stick [0-9]\+$/ stick $CALYPSO_STICK_ARFCN/" "$MOBILE_CFG"
        echo "[run.sh] mobile_group1.cfg : stick forced to $CALYPSO_STICK_ARFCN"
    elif grep -qE "^ no stick" "$MOBILE_CFG" 2>/dev/null; then
        sed -i "s/^ no stick$/ stick $CALYPSO_STICK_ARFCN/" "$MOBILE_CFG"
    fi
fi
OSMO_TRX_IPC="${OSMO_TRX_IPC:-osmo-trx-ipc}"
# Default cfg = versioned 1-chan cfg dans qemu-src/cfgs/. Le /etc/osmocom/
# legacy declarait chan 0 + chan 1 : osmo-trx-ipc demande 2 chans, calypso
# en a 1 : DDEV ERROR chan num mismatch. Le fichier versionne a juste chan 0.
OSMO_TRX_IPC_CFG_DEFAULT="/opt/GSM/qemu-src/cfgs/osmo-trx-ipc.cfg"
[ -f "$OSMO_TRX_IPC_CFG_DEFAULT" ] || OSMO_TRX_IPC_CFG_DEFAULT="/etc/osmocom/osmo-trx-ipc.cfg"
OSMO_TRX_IPC_CFG="${OSMO_TRX_IPC_CFG:-$OSMO_TRX_IPC_CFG_DEFAULT}"
# calypso-ipc-device = pont QEMU UDP 6702 <-> osmo-trx-ipc shm. Default :
# le binaire compile dans tools/calypso-ipc-device/. Override via env si
# tu veux pointer un autre path, ou vide pour skip (mode legacy debug).
CALYPSO_IPC_DEVICE_DEFAULT="/opt/GSM/qemu-src/tools/calypso-ipc-device/calypso-ipc-device"
CALYPSO_IPC_DEVICE="${CALYPSO_IPC_DEVICE-$CALYPSO_IPC_DEVICE_DEFAULT}"
IPC_MSOCK_PATH="${IPC_MSOCK_PATH:-/tmp/ipc_sock0}"

# ---- CALYPSO_MODE preset (cf. doc en tete du fichier) ----
# Selectionne un preset de composants a lancer. Override fin via les
# CALYPSO_SKIP_* individuels (cf bloc plus bas).
#
#   full       (default) -- Full radio pipeline.
#                          QEMU + osmocon + ipc-device + osmo-trx-ipc
#                          + osmo-bts-trx + L2 client + gsmtap.
#                          C'est le mode legacy, rien ne change pour les
#                          runs qui ne set pas CALYPSO_MODE.
#
#   shunt      -- Bissection FBSB_CONF (Phase 1 canned, no real radio).
#                QEMU(+DSP_SHUNT) + osmocon + L2 client.
#                Skip ipc-device, osmo-trx-ipc, osmo-bts-trx, gsmtap.
#                Le mock cote QEMU (calypso_dsp_shunt.c) repond aux
#                taches DSP avec des valeurs canned. Objectif unique :
#                determiner si FBSB_CONF tire (= ARM path OK) ou non
#                (= bug cote ARM, pas DSP). Cf CLAUDE.md.
#
#   shunt-ipc  -- Phase 2 (futur). Comme `full` mais avec DSP_SHUNT actif
#                pour que les vrais I/Q d'osmo-trx-ipc soient demod-es
#                cote bridge plutot que par le c54x emule.
#
#   bridge     -- Legacy. Pipeline d'avant le passage a osmo-trx-ipc :
#                QEMU + osmocon + bridge.py (Python, 5700-5702/6700/6702)
#                + osmo-bts-trx + L2 client + gsmtap.
#                bridge.py est restaure depuis git (commit 30933d1^).
#                Skip ipc-device + osmo-trx-ipc, le bridge Python remplit
#                le role de pont BTS<->QEMU avec son propre wall-paced FN
#                + GMSK soft:IQ inline (cf BRIDGE_BSP_IQ).
#
#   bare       -- QEMU + osmocon seulement. Pour debug fw isole, gdb,
#                tests qui n'ont pas besoin du L2/L3.
# Défaut = full : le chemin NORMAL, ZÉRO HACK. Le vrai DSP c54x reçoit l'I/Q
# RÉELLE (BSP_IQ_PASSTHROUGH=1) et fait FB/SB/BCCH lui-même → vrais SI → a_cd →
# le mobile campe pour de vrai. Aucun shunt, aucun bridge, aucune injection.
# Chemins de debug opt-in : CALYPSO_MODE=shunt-ipc (bridge gr-gsm, démod externe),
# CALYPSO_MODE=shunt (FB/SB cannés). À n'utiliser que pour diag, pas par défaut.
CALYPSO_MODE="${CALYPSO_MODE:-full-grgsm}"   # defaut full-grgsm (pipeline grgsm decode valide 2026-06-03)
# icount OFF par défaut (wall-clock) : plus rapide/fluide pour le full mode DSP.
: "${CALYPSO_ICOUNT:=off}"; export CALYPSO_ICOUNT
# IPC TX (uplink) ON par défaut : le device module les bursts UL du mobile
# (BSP→5702) en GMSK et les injecte vers osmo-trx→BTS (sinon UL = zéros).
: "${CALYPSO_IPC_UL:=1}"; export CALYPSO_IPC_UL
case "$CALYPSO_MODE" in
    full|full-grgsm|shunt|shunt-ipc|bridge|bare|free) ;;
    *) echo "[run.sh] ERR : CALYPSO_MODE=$CALYPSO_MODE inconnu (full|full-grgsm|shunt|shunt-ipc|bridge|bare|free)" >&2; exit 1 ;;
esac

# Defaults derives du preset. Chaque var peut etre overridee explicitement
# par l'env avant l'invocation (l'override gagne sur le preset).
case "$CALYPSO_MODE" in
    full)
        : "${CALYPSO_DSP_SHUNT:=0}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"
        : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        ;;
    full-grgsm)
        # qemu (Calypso) GARDÉ (firmware chargé par osmocon + lien série CP210x),
        # mais la RADIO du mobile passe par gr-gsm : le device LIVRE le DL au BSP
        # qui le publie en buffer shm (feed_iq) ; grgsm_shm_decode.py decode -> a_cd.
        # DSP SHUNT ON : c54x canné (léger) → qemu ne congestionne pas et ne se
        # coince pas sur le DSP (sa radio n'est pas utilisée de toute façon).
        : "${CALYPSO_DSP_SHUNT:=1}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"
        : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_IPC_RELAY:=1}"            # 1 = device RELAIE le chunk CONTINU (625 samples, guard inclus) -> 5810 -> gr-gsm gsm.receiver (pas de troncature 148)
        : "${CALYPSO_BSP_IQ_PASSTHROUGH:=1}"   # BSP interprete le payload comme I/Q cs16 (requis par feed_iq)
        : "${CALYPSO_RELAY_ALSO_BSP:=1}"       # 1 = device relaie 5810 ET nourrit le BSP (feed_iq -> cfile) -> FFT live par defaut
        : "${CALYPSO_TWL3025_AFC_HZ:=16000}"   # force AFC FIXE 16kHz : stoppe la chasse AFC -> carrier stable (le drift que le demod constant ne peut pas suivre) -> demod
        export CALYPSO_TWL3025_AFC_HZ
        : "${CALYPSO_SHUNT_NO_CANNED:=1}"  # PAS de SI3 canned (l'ancien bypass DSP)
        # Non-truqué : aucune injection SI legacy ne doit concurrencer le feed_si
        # gr-gsm (a_cd). Override (=) et pas default (:=) → même un env exporté
        # périmé ne peut pas les rallumer.
        CALYPSO_SHUNT_NO_CANNED=1     # canned shunt OFF (verrouillé)
        CALYPSO_DSP_L1STUB=0          # pas de PROM0 publisher SI3 baked
        CALYPSO_DSP_L1_STUB=0
        CALYPSO_FORCE_FBSB=0          # pas d'oracle FBSB_CONF
        CALYPSO_FORCE_AGCH=0          # pas de réécriture DATA_IND BCCH SI
        # FIFOs live (1 par consommateur e2e) : fft(host) grgsm(decode) record(drainer) asciifft(fenetre run.sh)
        : "${CALYPSO_RELAY_FIFOS:=/tmp/iq_fft.fifo:/tmp/iq_grgsm.fifo:/tmp/iq_record.fifo:/tmp/iq_asciifft.fifo}"
        export CALYPSO_RELAY_FIFOS
        export CALYPSO_IPC_RELAY CALYPSO_BSP_IQ_PASSTHROUGH CALYPSO_RELAY_ALSO_BSP CALYPSO_SHUNT_NO_CANNED \
               CALYPSO_DSP_L1STUB CALYPSO_DSP_L1_STUB \
               CALYPSO_FORCE_FBSB CALYPSO_FORCE_AGCH
        ;;
    shunt)
        : "${CALYPSO_DSP_SHUNT:=1}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"
        : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=1}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=1}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        ;;
    shunt-ipc)
        # Archi : IPC TX → VRAI DSP (démod) → gr-gsm (décode) → SI. Plus de
        # shunt DSP (DSP_SHUNT=0) : le c54x tourne pour de vrai. La chaîne IPC
        # DL alimente le DSP ; gr-gsm décode la sortie. On coupe l'UL de l'IPC
        # et le TX de gr-gsm.
        : "${CALYPSO_DSP_SHUNT:=0}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"
        : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=0}"   # bridge gr-gsm (décode sortie DSP)
        : "${CALYPSO_SHUNT_NO_CANNED:=1}"
        ;;
    bridge)
        : "${CALYPSO_DSP_SHUNT:=0}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"
        : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=0}"
        ;;
    bare)
        : "${CALYPSO_DSP_SHUNT:=0}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"
        : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=1}"
        : "${CALYPSO_SKIP_L2:=1}"
        : "${CALYPSO_SKIP_GSMTAP:=1}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        ;;
    free)
        # Mode free : aucun preset. Le menu (niveau 2) ou l'env user
        # doivent setter les CALYPSO_SKIP_* explicitement. Defaults safe
        # = pipeline complet (comme full) pour eviter l'echec silencieux
        # si l'user lance `CALYPSO_MODE=free ./run.sh` sans menu.
        : "${CALYPSO_DSP_SHUNT:=0}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"
        : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        ;;
esac
# Defaults pour les autres modes (full/shunt/bridge/bare/free) : pas de bridge
# démod, et le canned reste dispo (NO_CANNED=0) pour ne pas casser le shunt pur.
: "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}"
: "${CALYPSO_SHUNT_NO_CANNED:=0}"
: "${CALYPSO_IQ_TEE_PORT:=6703}"
export CALYPSO_DSP_SHUNT CALYPSO_MODE
export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
       CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY \
       CALYPSO_SKIP_DEMOD_BRIDGE CALYPSO_SHUNT_NO_CANNED CALYPSO_IQ_TEE_PORT

# ---- Auto-resolution implications / exclusions ----
# Apres les presets + override env, on applique les regles de coherence.
# Chaque regle log explicitement ce qu'elle force pour pas surprendre.

# Regle 1 : DSP_SHUNT=1 : BTS chain inutile (canned ne touche pas la radio).
# N'override pas si user a explicitement set SKIP_BTS=0 (= "je veux la
# radio quand meme meme en shunt", cas shunt-ipc).
# EXEMPTION full-grgsm : le shunt est ON (c54x leger) mais la chaine radio
# (BTS->trx->ipc-device) DOIT tourner pour relayer l'I/Q vers gr-gsm (5810).
# Sans cette exemption, SKIP_BTS=1 starve gr-gsm -> feed_si ne tire jamais.
if [ "$CALYPSO_DSP_SHUNT" = "1" ] && [ "$CALYPSO_MODE" != "shunt-ipc" ] && [ "$CALYPSO_MODE" != "full-grgsm" ]; then
    if [ -z "${_USER_SET_SKIP_IPC_DEVICE:-}" ]; then
        [ "${CALYPSO_SKIP_IPC_DEVICE:-0}" = "0" ] && echo "[run.sh] DSP_SHUNT=1 : SKIP_IPC_DEVICE=1 (canned, pas besoin)"
        CALYPSO_SKIP_IPC_DEVICE=1
    fi
    if [ -z "${_USER_SET_SKIP_TRX_IPC:-}" ]; then
        [ "${CALYPSO_SKIP_TRX_IPC:-0}" = "0" ] && echo "[run.sh] DSP_SHUNT=1 : SKIP_TRX_IPC=1 (canned, pas besoin)"
        CALYPSO_SKIP_TRX_IPC=1
    fi
    if [ -z "${_USER_SET_SKIP_BTS:-}" ]; then
        [ "${CALYPSO_SKIP_BTS:-0}" = "0" ] && echo "[run.sh] DSP_SHUNT=1 : SKIP_BTS=1 (canned, pas besoin)"
        CALYPSO_SKIP_BTS=1
    fi
fi

# Regle 2 : bridge.py XOR (ipc-device | osmo-trx-ipc). Si conflit, bridge.py
# gagne (parce que c'est ce que l'user a choisi explicitement en mode bridge,
# ou exporte en env).
if [ "$CALYPSO_SKIP_BRIDGE_PY" = "0" ]; then
    if [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "0" ] || [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ]; then
        echo "[run.sh] MUTEX bridge.py <-> ipc/trx-ipc -- desactivation ipc/trx-ipc"
        CALYPSO_SKIP_IPC_DEVICE=1
        CALYPSO_SKIP_TRX_IPC=1
    fi
fi

# Regle 3 : osmo-trx-ipc requiert calypso-ipc-device (sinon il echoue
# greeting). Si on a trx-ipc actif et ipc-device skipped, warn + desactive.
if [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ] && [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "1" ]; then
    echo "[run.sh] WARN osmo-trx-ipc requires calypso-ipc-device -- desactivation trx-ipc"
    CALYPSO_SKIP_TRX_IPC=1
fi

# Regle 4 : osmo-bts-trx requiert un pont radio (ipc/trx-ipc OU bridge.py).
# Sinon il a personne a qui parler en TRXD. Ne touche pas si DSP_SHUNT
# (ou BTS est aussi skipped par regle 1).
if [ "${CALYPSO_SKIP_BTS:-1}" = "0" ] \
   && [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "1" ] \
   && [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" = "1" ]; then
    echo "[run.sh] WARN osmo-bts-trx sans pont radio (ni trx-ipc ni bridge.py) -- il restera en idle"
fi

# Regle 5 : MTTCG XOR icount auto. Deja gere ailleurs (CALYPSO_MTTCG=1
# force ICOUNT=off plus bas), mais logger ici pour tracabilite.
# Default icount = auto depuis 2026-05-27 (cf L1087), donc le fallback ici
# doit aussi être "auto" pour que le check mutex soit correct.
if [ "${CALYPSO_MTTCG:-0}" = "1" ] && [ "${CALYPSO_ICOUNT:-auto}" != "off" ]; then
    echo "[run.sh] MUTEX MTTCG <-> icount -- sera force ICOUNT=off (cf bloc MTTCG)"
fi

export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
       CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY

# ---- DSP / DIAG instruments (override at command line if needed) ----
# CALYPSO_DSP_ROM (legacy single-txt) supprime — utilise CALYPSO_DSP_ROM_TXT
# pour la source .txt auto-splitee en per-section bins (cf L1422+).
# Défaut 0x2a00 : adresse CORRECTE du buffer DMA I/Q du BSP, confirmée par
#   - le canary 0xCAFE E2E (DSP READS 0x2a00..0x2a13 via PC=0x93a5),
#   - le IQ-READ tracer calypso_c54x.c:1596 ([0x2a00..0x2b27] = buffer DMA).
# NB : 0x2bc0..0x2bff = coefficients corrélateur (PAS l'I/Q) ; 0x80..0x3A3 =
# pattern de réf FB (AR3 sweep). Le blocage FB-det (fb0_att=0) n'est PAS une
# question d'adresse mais un bug DSP (timing/coeffs/fenêtre compute, cf.
# instrumentation 2026-05-14 calypso_c54x.c:147).
CALYPSO_BSP_DARAM_ADDR="${CALYPSO_BSP_DARAM_ADDR:-0x2a00}"
CALYPSO_SIM_CFG="${CALYPSO_SIM_CFG:-$MOBILE_CFG}"
# tdma_timer = REALTIME by default (revert 2026-05-29 v2).
# Le single-domain VIRTUAL (QEMU esclave du temps virtuel) ne suffit PAS :
# le BTS (osmo-bts-trx) + osmo-trx-ipc restent des process WALL non esclavés
# au FN virtuel → en VIRTUAL ils poussent les DL à 217 Hz wall pendant que
# QEMU consomme à ~10 Hz virtuel → FIFO ipc-device overflow (4095) → LOST.
# REALTIME au moins aligne QEMU sur le BTS wall (reste le drift ARM icount,
# autre combat). Opt-in single-domain virtuel (incomplet) : CALYPSO_TDMA_REALTIME=0.
CALYPSO_TDMA_REALTIME="${CALYPSO_TDMA_REALTIME:-1}"
# W1C latch on by default : ARM reads of a_sync_* / d_fb_det return the
# host-side state machine's published values (stable) instead of DSP
# transient writes (which flicker set→clear in ~18 cycles, losing the
# race for ARM read). Required for FBSB_CONF emission.
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-1}"
export CALYPSO_BSP_DARAM_ADDR CALYPSO_SIM_CFG CALYPSO_TDMA_REALTIME CALYPSO_W1C_LATCH

# === Defaults no-hack ===
# Politique : aucun bypass de chemin firmware. Tous les hacks env-gated
# (PM_INJECT, FBSB_SYNTH, DSP_FBDET_SKIP, FORCE_RX_DONE, BYPASS_BDLENA,
# FORCE_DARAM62) ont ete retires du code source 2026-05-26.
# Conforme a CLAUDE.md regle #1 : "PAS DE HACK".
CALYPSO_PROBE_BOOTSTUB="${CALYPSO_PROBE_BOOTSTUB:-0}" # diag-only probe (no hack)

# === Stability / non-hack mais necessaire ===
CALYPSO_DSP_IDLE_FF="${CALYPSO_DSP_IDLE_FF:-1}"       # perf : fast-forward DSP idle dispatcher (pas un hack)
CALYPSO_DSP_IDLE_RANGE="${CALYPSO_DSP_IDLE_RANGE:-}"

# === Diag / debug (no-hack) ===
# W1C_LATCH=1 par defaut : NOMINAL (requis pour FBSB_CONF, cf bloc plus haut).
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-1}"
CALYPSO_NDB_D_RACH_OFFSET="${CALYPSO_NDB_D_RACH_OFFSET:-}"
CALYPSO_RACH_FORCE_BSIC="${CALYPSO_RACH_FORCE_BSIC:-}"
CALYPSO_UART_TRACE="${CALYPSO_UART_TRACE:-0}"
# CALYPSO_BSP_IQ_PASSTHROUGH : QEMU BSP accepte du cs16 I,Q entrelace en
# DMA direct vers le correlateur DSP (au lieu de moduler les 148 hard-bits
# lui-meme en +/-pi/2). Default ON : c'est ce que produit notre device IPC
# (osmo-trx natif modem GMSK). Si =0, le BSP reverte a sa modulation hard
# interne -- mode legacy non-utilise maintenant.
CALYPSO_BSP_IQ_PASSTHROUGH="${CALYPSO_BSP_IQ_PASSTHROUGH:-1}"
export CALYPSO_BSP_IQ_PASSTHROUGH
# CALYPSO_TIMER : gate les fprintf timer (frame_irq, tdma_tick, kick) cote
# calypso_trx.c. =0 (default) = runs silencieux, zero stderr-pipe backpressure.
# =1 = memes lignes qu'avant. Cf. helper static calypso_timer_log() (cached).
CALYPSO_TIMER="${CALYPSO_TIMER:-0}"
CALYPSO_DSP_BUDGET="${CALYPSO_DSP_BUDGET:-256000}"
export CALYPSO_W1C_LATCH \
       CALYPSO_NDB_D_RACH_OFFSET CALYPSO_RACH_FORCE_BSIC \
       CALYPSO_DSP_IDLE_FF CALYPSO_DSP_IDLE_RANGE \
       CALYPSO_UART_TRACE CALYPSO_TIMER \
       CALYPSO_PROBE_BOOTSTUB CALYPSO_DSP_BUDGET

# ---- icount mode (deterministic virtual clock paced by instruction count) ----
# Default ON (auto = shift=auto,sleep=on,align=off). Set CALYPSO_ICOUNT=off
# to disable. The kick timer (calypso_kick_cb) was moved to
# QEMU_CLOCK_VIRTUAL so it no longer races with icount.
# Other accepted values:
#   auto              shift dynamic, wall-clock aligned (recommended)
#   shift=N,sleep=on  fixed shift (1<<N instr ~= 1ns), explicit sleep
#   off               disable (legacy default-clock mode)
CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-off}"  # DEFAUT OFF 2026-06-03 (full-grgsm : DATA_IND via socket l1ctl, insensible au LOST UART ; cfile decode plus rapide). Override =auto si besoin AFC. Ancien default auto since 2026-05-27 SIM fix (read-to-clear + WT timer on MASKIT unmask, calypso_sim.c). Pipeline complet OK sous auto.
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

# ============================================================================
# kconfig-style cross-variable resolver (behavioral/probe relations).
# Pipeline component mutex/deps = Regles 1-5 plus haut. Ici : behavioral.
# S'execute APRES que tous les defaults nominaux soient fixes.
# ============================================================================
# R6 CONFLICT : DSP_SHUNT bypass le c54x emule -> sondes c54x muettes.
if [ "${CALYPSO_DSP_SHUNT:-0}" = "1" ] && [ -n "${CALYPSO_DEBUG:-}" ]; then
    echo "[kconfig] WARN DSP_SHUNT=1 court-circuite le c54x : sondes CALYPSO_DEBUG c54x sans effet" >&2
fi
# R7 SELECT : QEMU_HALT (-S) exige un pane gdb pour relancer (sinon boot fige).
if [ "${CALYPSO_QEMU_HALT:-0}" = "1" ] && [ "${CALYPSO_SKIP_GDB_PANE:-1}" = "1" ]; then
    echo "[kconfig] QEMU_HALT=1 SELECT pane gdb (SKIP_GDB_PANE=0) -- sinon QEMU reste halted" >&2
    CALYPSO_SKIP_GDB_PANE=0; export CALYPSO_SKIP_GDB_PANE
fi
# R8 CONFLICT : BSP_IQ_PASSTHROUGH alimente le correlateur ; incompatible
# avec DSP_SHUNT (court-circuite le BSP). SHUNT gagne.
# EXEMPTION full-grgsm : ici gr-gsm EST le consommateur de l'I/Q (feed_iq->shm),
# donc le passthrough a un sens meme sous shunt -> on NE le desactive pas.
if [ "${CALYPSO_DSP_SHUNT:-0}" = "1" ] && [ "${CALYPSO_BSP_IQ_PASSTHROUGH:-0}" = "1" ] \
   && [ "$CALYPSO_MODE" != "full-grgsm" ]; then
    echo "[kconfig] CONFLICT DSP_SHUNT <-> BSP_IQ_PASSTHROUGH -- IQ passthrough desactive" >&2
    CALYPSO_BSP_IQ_PASSTHROUGH=0; export CALYPSO_BSP_IQ_PASSTHROUGH
fi
# R9 WARN : FORCE_INTM_ONESHOT = NON-nominal (force le clear INTM, sonde).
if [ "${CALYPSO_FORCE_INTM_ONESHOT:-0}" = "1" ]; then
    echo "[kconfig] WARN FORCE_INTM_ONESHOT=1 : NON-nominal (force INTM clear)" >&2
fi
# R10 SELECT : DSP_IDLE_RANGE non-vide implique DSP_IDLE_FF actif.
if [ -n "${CALYPSO_DSP_IDLE_RANGE:-}" ] && [ "${CALYPSO_DSP_IDLE_FF:-1}" = "0" ]; then
    echo "[kconfig] DSP_IDLE_RANGE set SELECT DSP_IDLE_FF=1" >&2
    CALYPSO_DSP_IDLE_FF=1; export CALYPSO_DSP_IDLE_FF
fi

# ---- log paths ----
QEMU_LOG="/root/qemu.log"
OSMOCON_LOG="/tmp/osmocon.log"
MOBILE_LOG="/tmp/mobile.log"
BTS_LOG="/tmp/bts.log"
L2_LOG="/tmp/l2_client.log"
OSMO_TRX_IPC_LOG="/tmp/osmo-trx-ipc.log"
IPC_DEVICE_LOG="/tmp/calypso-ipc-device.log"
MON_SOCK="/tmp/qemu-calypso-mon.sock"
L1CTL_SOCK="/tmp/osmocom_l2"
QEMU_DUMMY_SOCK="/tmp/qemu_l1ctl_disabled"

# ---- Log timestamping ----
# Chaque ligne des logs (qemu.log, osmocon.log, mobile.log) est
# prefixee par `<epoch_sec> +<rel_sec>s ` (epoch_sec = horloge mur depuis 1970,
# rel_sec = secondes depuis le demarrage du wrapper). Permet de detecter les
# drifts temporels entre logs en comparant les timestamps colonne 1+2.
# Desactiver via CALYPSO_LOG_TS=0.
# Note : grep -E des tests n'est pas perturbe (le prefix est en debut de
# ligne, les patterns sont en milieu).
CALYPSO_LOG_TS="${CALYPSO_LOG_TS:-1}"
TSLOG_SCRIPT=/tmp/calypso_tslog.py
cat > "$TSLOG_SCRIPT" <<'PYEOF'
#!/usr/bin/env python3
"""Prefixe stdin avec `<epoch_sec> +<rel_sec>s ` et flush ligne par ligne."""
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
#   1. mobile    : osmocom-bb mobile complet (stack L23 + VTY) -- default
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
echo "CALYPSO_BSP_IQ_PASSTHROUGH = $CALYPSO_BSP_IQ_PASSTHROUGH"

# ---------- cleanup ----------
# Archive le log osmocon du run précédent AVANT de l'effacer : on veut
# garder TOUT osmocon.log pour CHAQUE run (sinon le rm/truncate ci-dessous
# n'en laisse qu'un, et on perd l'historique). Timestamp = run-id.
if [ -f "$OSMOCON_LOG" ] && [ -s "$OSMOCON_LOG" ]; then
    _OSMOCON_ARCHIVE="${OSMOCON_LOG%.log}.$(date +%Y%m%d_%H%M%S).log"
    mv "$OSMOCON_LOG" "$_OSMOCON_ARCHIVE" 2>/dev/null \
        && echo "[run.sh] osmocon.log précédent archivé → $_OSMOCON_ARCHIVE"
fi
rm -f "$QEMU_LOG" "$OSMOCON_LOG" "$MOBILE_LOG" "$BTS_LOG" \
      "$OSMO_TRX_IPC_LOG" "$IPC_DEVICE_LOG" \
      "$MON_SOCK" "$L1CTL_SOCK" "$QEMU_DUMMY_SOCK" \
      "$IPC_MSOCK_PATH" "${IPC_MSOCK_PATH}_0"

tmux kill-session -t "$SESSION" 2>/dev/null || true
killall -q -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc >/dev/null 2>&1 || true   # PAS python3 (tue le tslog -> Killed moche) ; les pkills cibles gerent grgsm/si_bridge
pkill -9 -f calypso-ipc-device 2>/dev/null || true
# full-grgsm : tuer les restes qui tiennent des ports (5810/5811/6700/6800/4730)
pkill -9 -f "grgsm_trx"   2>/dev/null || true
pkill -9 -f "grgsm_relay" 2>/dev/null || true
pkill -9 -f "inject.py"     2>/dev/null || true   # pas d'injecteur SI gdb concurrent (a_cd)
pkill -9 -f "validating.py" 2>/dev/null || true
pkill -9 -x trxcon        2>/dev/null || true
pkill -9 -f "cp210x_tee"  2>/dev/null || true
# --- Kill exhaustif (2026-06-03) : chaque run part 100% propre (plus de 3 decodeurs) ---
pkill -9 -f "grgsm_shm_decode"   2>/dev/null || true
pkill -9 -f "grgsm_fft_live"     2>/dev/null || true
pkill -9 -f "gsmtap_relay"       2>/dev/null || true
pkill -9 -f "bin/grgsm_decode"   2>/dev/null || true   # le tool standard (par chemin, pas de sur-match)
pkill -9 -f "grgsm_cfile"        2>/dev/null || true
pkill -9 -f "qemu-system-arm"    2>/dev/null || true
pkill -9 -f "osmo-trx-ipc"       2>/dev/null || true
pkill -9 -x osmo-bts-trx         2>/dev/null || true
pkill -9 -x osmocon              2>/dev/null || true
pkill -9 -x mobile               2>/dev/null || true
pkill -9 -f "inject_si3"         2>/dev/null || true
pkill -9 -f "si_bridge"        2>/dev/null || true
pkill -9 -f "relay_continu"     2>/dev/null || true
pkill -9 -f "record_drain"     2>/dev/null || true
rm -f /tmp/relay_continu.cfile /tmp/record.cfile 2>/dev/null || true  # vieux record (ring externalise maintenant)
sleep 1   # laisse les sockets UDP/TCP se libérer avant de relancer
pkill -9 -f irda_capture.py 2>/dev/null || true
rm -f "$L1CTL_SOCK" "$MON_SOCK" "$QEMU_DUMMY_SOCK" /tmp/osmocom_l2_*
rm -f /tmp/fw-irda.log /tmp/irda_capture.pid /tmp/irda.pty.link
# Drop the legacy mmap from previous runs -- no longer used, but lying
# around in /dev/shm could confuse forensic forensics on old runs.
rm -f /dev/shm/calypso_si.bin
sleep 1

/etc/osmocom/status.sh stop 2>/dev/null || true
/etc/osmocom/osmo-start.sh 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n qemu

# ---------- 1. QEMU ----------
# icount controlled by CALYPSO_ICOUNT env var (default 'auto'). The kick
# timer in calypso_trx.c was moved to QEMU_CLOCK_VIRTUAL so icount no
# longer freezes the TDMA tick. If you observe device/osmo-trx-ipc
# timeouts, fall back with CALYPSO_ICOUNT=off.
# gdb-stub : active d'office sur 0.0.0.0:1234 pour que les tests/scripts
# d'injection (inject.py / validating.py / tests/test_inject_frames.py)
# puissent s'y connecter sans avoir a passer par le monitor HMP. La syntaxe
# `tcp::1234` bind sur toutes les interfaces du container -- utilisable
# depuis l'IP container (ex. 172.20.0.11:1234) cote host.
# Override via CALYPSO_GDB_PORT (set vide pour desactiver).
CALYPSO_GDB_PORT="${CALYPSO_GDB_PORT:-1234}"
if [ -n "$CALYPSO_GDB_PORT" ]; then
    QEMU_GDB_FLAG="-gdb tcp::$CALYPSO_GDB_PORT"
else
    QEMU_GDB_FLAG=""
fi

# CALYPSO_QEMU_HALT=1 : ajoute -S (start halted) au launch QEMU.
# Permet à un gdb client d'attacher AVANT que le firmware tourne,
# pose ses BPs, puis 'c' pour lancer. Utilisé par run_gdb_sim.sh.
if [ "${CALYPSO_QEMU_HALT:-0}" = "1" ]; then
    QEMU_HALT_FLAG="-S"
    echo "[run.sh] CALYPSO_QEMU_HALT=1 -- QEMU lancé HALTED (-S). Attache gdb + 'c' pour démarrer firmware."
else
    QEMU_HALT_FLAG=""
fi

# DSP code-loading env vars (all opt-in, all forwarded to -M calypso,KEY=PATH) :
#
#   CALYPSO_DSP_BLOB=path   → DARAM fixture : raw 16-bit LE blob loaded at
#                             DARAM[0x100] with PC override. C54x emulator test
#                             fixture, ROM TI bypassed (legacy fallback also
#                             skipped). See c54x_load_blob_daram + set_initial_pc.
#
#   CALYPSO_DSP_PROM0=path  → prog[0x07000..] (28K words max)
#   CALYPSO_DSP_PROM1=path  → prog[0x18000..] (32K) + mirror prog[0xE000..]
#   CALYPSO_DSP_PROM2=path  → prog[0x28000..] (32K)
#   CALYPSO_DSP_PROM3=path  → prog[0x38000..] (8K)
#   CALYPSO_DSP_DROM=path   → data[0x09000..] (20K)
#   CALYPSO_DSP_PDROM=path  → data[0x0E000..] (8K)
#   CALYPSO_DSP_REGISTERS=path → MMR reset snapshot (words 0x00..0x1F): IMR,
#                                IFR, ST0/ST1, T, TRN, AR0-7, SP, BK, BRC,
#                                RSA, REA, PMST. Auto-detected as
#                                <base>.Registers.bin. Empty = use C hardcode.
#
# Per-section bins are produced by dsp_txt2bin.py from a legacy calypso_dsp.txt :
#   python3 dsp_txt2bin.py calypso_dsp.txt calypso_dsp.bin
# → calypso_dsp.PROM0.bin, calypso_dsp.PROM1.bin, …
#
# All env vars are OFF by default. With nothing set, DSP runs with empty
# prog[]/data[] (no implicit /opt/GSM/calypso_dsp.txt fallback anymore).
#
# Mutually exclusive : dsp-blob is for DARAM-only fixtures, per-section is for
# real ROM loads at silicon-correct addresses. Both at once = blob wins (no
# section loads), per code path in calypso_trx.c.

MACHINE_ARG="calypso"
: "${CALYPSO_DSP_REG_MODE:=c54x}"; export CALYPSO_DSP_REG_MODE  # diag 2026-06-02 : c54x=vrai FB (task_md=5,d_fb_det=0x9041) ; bin=self-CALA. opt-out CALYPSO_DSP_REG_MODE=bin

_dsp_check_file() {
    local var="$1"; local path="$2"
    if [ ! -r "$path" ]; then
        echo "[run.sh] $var=$path : fichier introuvable ou illisible" >&2
        exit 1
    fi
}

# When CALYPSO_DSP_BLOB is set, the blob is the sole DSP code source.
# Force-clear ALL per-section env vars before defaults run, so the launch sees
# only the blob (no ROM loaded into prog[], no overlay collision in DARAM).
# Done BEFORE the default block so the `${VAR+x}` defaultedness check below
# treats these as "set-empty" → defaults skipped → no section ends up loaded.
if [ -n "${CALYPSO_DSP_BLOB:-}" ]; then
    CALYPSO_DSP_PROM0=""
    CALYPSO_DSP_PROM1=""
    CALYPSO_DSP_PROM2=""
    CALYPSO_DSP_PROM3=""
    CALYPSO_DSP_DROM=""
    CALYPSO_DSP_PDROM=""
    CALYPSO_DSP_REGISTERS=""
    echo "[run.sh] CALYPSO_DSP_BLOB=$CALYPSO_DSP_BLOB → all dsp-prom*/drom/pdrom/registers force-disabled (blob is sole DSP code source)"
fi

# DSP txt source → auto-split to per-section .bins on demand.
# Source path (override via CALYPSO_DSP_ROM_TXT). Defaults to /opt/GSM/calypso_dsp.txt.
# The .bin files are generated next to the txt :
#   <dir>/<base>.PROM0.bin, .PROM1.bin, ..., .DROM.bin, .PDROM.bin
: "${CALYPSO_DSP_ROM_TXT:=/opt/GSM/calypso_dsp.txt}"
_DSP_TXT_DIR="$(dirname "$CALYPSO_DSP_ROM_TXT")"
_DSP_TXT_BASE="$(basename "$CALYPSO_DSP_ROM_TXT" .txt)"

if [ -r "$CALYPSO_DSP_ROM_TXT" ]; then
    _need_split=0
    for _sec in PROM0 PROM1 PROM2 PROM3 DROM PDROM; do
        _bin="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.${_sec}.bin"
        if [ ! -r "$_bin" ] || [ "$CALYPSO_DSP_ROM_TXT" -nt "$_bin" ]; then
            _need_split=1
            break
        fi
    done
    if [ "$_need_split" = "1" ]; then
        echo "[run.sh] auto-split $CALYPSO_DSP_ROM_TXT → ${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.{PROM0..3,DROM,PDROM}.bin"
        python3 "$(dirname "$0")/dsp_txt2bin.py" \
            "$CALYPSO_DSP_ROM_TXT" \
            "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.bin" || {
                echo "[run.sh] dsp_txt2bin.py failed — continuing without auto-split" >&2
            }
    fi
    unset _need_split _sec _bin
fi

# Per-section defaults — chacun pointe sur sa ROM précise (dérivée du txt).
# Logique par ligne : si l'env var est UNSET (pas juste vide) ET que le fichier
# existe, on assigne. Empty preserved → disable. User-set → kept.
[ -z "${CALYPSO_DSP_PROM0+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM0.bin" ] && CALYPSO_DSP_PROM0="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM0.bin"
[ -z "${CALYPSO_DSP_PROM1+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM1.bin" ] && CALYPSO_DSP_PROM1="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM1.bin"
[ -z "${CALYPSO_DSP_PROM2+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM2.bin" ] && CALYPSO_DSP_PROM2="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM2.bin"
[ -z "${CALYPSO_DSP_PROM3+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM3.bin" ] && CALYPSO_DSP_PROM3="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PROM3.bin"
[ -z "${CALYPSO_DSP_DROM+x}"  ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.DROM.bin"  ] && CALYPSO_DSP_DROM="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.DROM.bin"
[ -z "${CALYPSO_DSP_PDROM+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PDROM.bin" ] && CALYPSO_DSP_PDROM="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.PDROM.bin"
# Register snapshot (MMR reset state). Note the mixed-case ".Registers.bin"
# filename produced by dsp_txt2bin.py (the section loop above uses uppercase).
[ -z "${CALYPSO_DSP_REGISTERS+x}" ] && [ -r "${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.Registers.bin" ] && CALYPSO_DSP_REGISTERS="${_DSP_TXT_DIR}/${_DSP_TXT_BASE}.Registers.bin"
unset _DSP_TXT_DIR _DSP_TXT_BASE

# === L1-STUB ROM (opt-in CALYPSO_DSP_L1STUB=1) =========================
# Patche PROM0 @0x7120 avec un publisher synthétique L1+L2 (FB lock +
# PM + SB + BCCH SI3 écrits en continu) via scripts/make_dsp_bin_L1.py.
# Le RESTE du ROM est préservé ; toutes les autres sections/blobs intacts.
# But : faire camper le mobile (SI) sans dépendre de la corrélation I/Q.
if [ "${CALYPSO_DSP_L1STUB:-0}" = "1" ]; then
    _L1STUB_IN="${CALYPSO_DSP_PROM0:-/opt/GSM/calypso_dsp.PROM0.bin}"
    _L1STUB_OUT="/tmp/calypso_dsp_L1stub.PROM0.bin"
    _L1STUB_SCRIPT="$(dirname "$0")/scripts/make_dsp_bin_L1.py"
    [ -r "$_L1STUB_SCRIPT" ] || _L1STUB_SCRIPT="/opt/GSM/qemu-src/scripts/make_dsp_bin_L1.py"
    echo "[run.sh] CALYPSO_DSP_L1STUB=1 → patch $_L1STUB_IN → $_L1STUB_OUT"
    python3 "$_L1STUB_SCRIPT" "$_L1STUB_IN" "$_L1STUB_OUT" || {
        echo "[run.sh] make_dsp_bin_L1.py a échoué" >&2; exit 1; }
    CALYPSO_DSP_PROM0="$_L1STUB_OUT"
    echo "[run.sh] PROM0 → $_L1STUB_OUT (publisher L1+L2 actif, reste du ROM intact)"
    unset _L1STUB_IN _L1STUB_OUT _L1STUB_SCRIPT
fi

# CALYPSO_DSP_BLOB has NO default — DARAM fixture is opt-in only.
# Usage examples:
#   CALYPSO_DSP_PROM2= ./run.sh                          # disable PROM2
#   CALYPSO_DSP_PROM0=/tmp/patched.bin ./run.sh          # override PROM0
#   CALYPSO_DSP_BLOB=/path/to/blob.bin ./run.sh          # DARAM fixture mode
#   CALYPSO_DSP_ROM_TXT=/tmp/other-dsp.txt ./run.sh      # other ROM source
#   CALYPSO_DSP_L1_STUB=1 ./run.sh                       # use L1 synth stub PROM0
#                                                         # (FB+SB canned, real DSP-exec)

# ---- CALYPSO_DSP_L1_STUB=1 shortcut (2026-05-28) ----
# Switches PROM0 to the synthetic L1 stub generated by make_dsp_bin_L1.py.
# Other bins (PROM1/2/3/DROM/PDROM) unchanged (= real ROM kept).
# Auto-generates the stub if missing.
if [ "${CALYPSO_DSP_L1_STUB:-0}" = "1" ]; then
    _L1_STUB_BIN="/tmp/calypso_dsp_L1stub.PROM0.bin"
    if [ ! -r "$_L1_STUB_BIN" ]; then
        echo "[run.sh] CALYPSO_DSP_L1_STUB=1 → generating $_L1_STUB_BIN"
        python3 "$(dirname "$0")/scripts/make_dsp_bin_L1.py" \
            "/opt/GSM/calypso_dsp.PROM0.bin" \
            "$_L1_STUB_BIN" || {
                echo "[run.sh] L1 stub generation FAILED — falling back to real PROM0" >&2
                _L1_STUB_BIN=""
            }
    fi
    if [ -n "$_L1_STUB_BIN" ] && [ -r "$_L1_STUB_BIN" ]; then
        CALYPSO_DSP_PROM0="$_L1_STUB_BIN"
        echo "[run.sh] CALYPSO_DSP_L1_STUB=1 → PROM0=$_L1_STUB_BIN (FB+SB canned, others real ROM)"
    fi
    unset _L1_STUB_BIN
fi

if [ -n "${CALYPSO_DSP_BLOB:-}" ]; then
    _dsp_check_file CALYPSO_DSP_BLOB "$CALYPSO_DSP_BLOB"
    MACHINE_ARG="${MACHINE_ARG},dsp-blob=${CALYPSO_DSP_BLOB}"
    echo "[run.sh] CALYPSO_DSP_BLOB=$CALYPSO_DSP_BLOB (DARAM fixture)"
fi
for _sec in PROM0 PROM1 PROM2 PROM3 DROM PDROM REGISTERS; do
    _var="CALYPSO_DSP_${_sec}"
    _val="${!_var:-}"
    if [ -n "$_val" ]; then
        _dsp_check_file "$_var" "$_val"
        _prop="$(echo "$_sec" | tr '[:upper:]' '[:lower:]')"
        MACHINE_ARG="${MACHINE_ARG},dsp-${_prop}=${_val}"
        echo "[run.sh] $_var=$_val → -M ...,dsp-${_prop}=${_val}"
    fi
done
unset _sec _var _val _prop

# DATA_IND : par defaut on N INJECTE PAS le court-circuit l1ctl_inject_dl_si
# (CALYPSO_SHUNT_DL_INJECT=0) => le SI passe UNIQUEMENT par a_cd -> firmware L1
# -> UART (le vrai chemin). Mettre =1 pour reactiver le court-circuit direct.
: "${CALYPSO_SHUNT_DL_INJECT:=0}"
export CALYPSO_SHUNT_DL_INJECT

# Override delibere : pour le child QEMU seulement, L1CTL_SOCK pointe vers
# le dummy (/tmp/qemu_l1ctl_disabled). QEMU/l1ctl_sock.c cree son socket a
# cette adresse-poubelle (= L1CTL QEMU desactive). Le VRAI socket L1CTL
# /tmp/osmocom_l2 est cree plus tard par osmocon (L541 : -s $L1CTL_SOCK,
# ou L1CTL_SOCK garde sa valeur d'origine = /tmp/osmocom_l2). Hors de
# cette ligne, $L1CTL_SOCK reste = /tmp/osmocom_l2 partout dans run.sh.
L1CTL_SOCK="$QEMU_DUMMY_SOCK" \
"$QEMU" -M "$MACHINE_ARG" -cpu arm946 \
    $QEMU_ICOUNT_FLAG \
    $QEMU_ACCEL_FLAG \
    $QEMU_GDB_FLAG \
    $QEMU_HALT_FLAG \
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
    echo " TIMEOUT -- no PTY in $QEMU_LOG"
    exit 1
fi
echo " OK ($PTY_MODEM, QEMU_PID=$QEMU_PID)"

# ---------- 1ter. CP210x tee (mode full-grgsm) ----------
# Tee bidirectionnel du lien série CP210x (PTY série0 qemu) → osmocon + trxcon.
# osmocon prend /tmp/cp210x_osmocon ; trxcon prend /tmp/cp210x_trxcon.
OSMOCON_SERIAL="$PTY_MODEM"
# full-grgsm : trxcon RETIRE -> PAS de cp210x_tee, PAS de socket trxcon.
# Le tee 3-voies (osmocon+trxcon) ne servait qu'a partager le PTY serie entre
# osmocon ET trxcon. Sans trxcon c'est un middleman Python qui PERD des octets
# quand le firmware flood l'UART -> osmocon "LOST" en masse -> freeze mobile.
# osmocon parle DIRECT au PTY serie du firmware ($PTY_MODEM). "osmocon only".

# ---------- 1bis. IrDA capture (UART_IRDA = serial1, IRQ 18, 0xFFFF5000) ----------
# Phase 3 du plan PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md :
# le firmware compal_e88 fait deja `cons_bind_uart(UART_IRDA)` (init.c:105),
# donc tout `printf()` cote fw passe par UART_IRDA. Ici on consomme le PTY
# serial1 alloue par QEMU et on l'archive dans /tmp/fw-irda.log avec le meme
# prefixe timestamp que les autres logs.
#
# Desactivable via CALYPSO_IRDA_CAPTURE=0 (rare -- utile si saturation diag).
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
        # Lance irda_capture en background (silencieux -- il pose ses bytes
        # dans /tmp/fw-irda.log directement, pas sur stdout) puis suit le
        # log en live dans la fenetre tmux pour debug visuel.
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
            echo " WARN -- pidfile absent (capture peut-etre pas lance)"
        fi
    else
        echo " WARN -- no serial1 PTY detected : IrDA capture skipped"
    fi
fi

# Note : le marker `=== fw-irda boot OK ===` emis par cons_puts() ligne 119
# de compal_e88/init.c peut etre perdu si irda_capture s'attache au slave PTY
# apres que QEMU ait ecrit (race window ~0.5-1.5s). Solution durable : passer
# par Phase 5 du plan IrDA (beacon hot path dans la main loop, qui re-emet
# regulierement et garantit la capture). Ne PAS tenter `-S` + `cont` ici : ca
# perturbe la sequence osmocon:firmware (le mobile ne camp plus sur la cell).

# ---------- 2. osmocon ----------
tmux new-window -t "$SESSION" -n osmocon
tmux send-keys -t "$SESSION:osmocon" \
    ": > $OSMOCON_LOG && stdbuf -oL -eL $OSMOCON -m romload -i 100 -p $OSMOCON_SERIAL -s $L1CTL_SOCK $FW_BIN -d tr 2>&1 | $TSLOG | tee $OSMOCON_LOG" C-m

echo -n "Waiting for osmocon to expose $L1CTL_SOCK..."
for i in $(seq 1 30); do
    [ -S "$L1CTL_SOCK" ] && break
    sleep 1; echo -n "."
done
if [ -S "$L1CTL_SOCK" ]; then echo " OK"; else echo " WARN -- socket missing"; fi

# ---------- 3. calypso-ipc-device (Phase 1 du chantier osmo-trx-ipc) ----------
# Pont QEMU UDP 6702 <-> osmo-trx-ipc shm. Fork d'ipc-driver-test
# (/opt/GSM/osmo-trx/Transceiver52M/device/ipc/) ou le wrapper UHD est
# remplace par : DL : cs16 shm : UDP 6702 vers QEMU, UL : recv 6702 :
# heartbeat zeros vers shm (cf. session 2026-05-26 plan).
# Le device DOIT demarrer AVANT osmo-trx-ipc -- c'est lui qui cree le
# master socket Unix ($IPC_MSOCK_PATH), osmo-trx-ipc s'y connecte.
if [ "${CALYPSO_SKIP_IPC_DEVICE:-0}" != "1" ]; then
    tmux new-window -t "$SESSION" -n ipc-device
    if [ -n "$CALYPSO_IPC_DEVICE" ] && [ -x "$CALYPSO_IPC_DEVICE" ]; then
        tmux send-keys -t "$SESSION:ipc-device" \
            ": > $IPC_DEVICE_LOG && CALYPSO_IPC_RELAY=${CALYPSO_IPC_RELAY:-0} CALYPSO_TRX_IQ_HOST=${CALYPSO_TRX_IQ_HOST:-127.0.0.1} CALYPSO_TRX_IQ_RX_PORT=${CALYPSO_TRX_IQ_RX_PORT:-5810} CALYPSO_TRX_IQ_TX_PORT=${CALYPSO_TRX_IQ_TX_PORT:-5811} CALYPSO_RELAY_FIFOS=${CALYPSO_RELAY_FIFOS:-/tmp/iq_fft.fifo:/tmp/iq_grgsm.fifo:/tmp/iq_record.fifo:/tmp/iq_asciifft.fifo} $CALYPSO_IPC_DEVICE -u /tmp -n 0 2>&1 | $TSLOG | tee $IPC_DEVICE_LOG" C-m
        echo -n "Waiting for calypso-ipc-device master sock ($IPC_MSOCK_PATH)..."
        for i in $(seq 1 30); do
            [ -S "$IPC_MSOCK_PATH" ] && break
            sleep 0.5; echo -n "."
        done
        [ -S "$IPC_MSOCK_PATH" ] && echo " OK" || echo " WARN -- socket missing"
    else
        tmux send-keys -t "$SESSION:ipc-device" \
            "echo '[TODO] calypso-ipc-device pas encore implemente (Phase 1).' && \
             echo 'Pour activer : CALYPSO_IPC_DEVICE=/path/to/calypso-ipc-device ./run.sh' && \
             echo 'Voir : /opt/GSM/osmo-trx/Transceiver52M/device/ipc/ipc-driver-test.c' && \
             echo 'Sans ce device, osmo-trx-ipc sortira en erreur (pas de master sock).'" C-m
        echo "[run.sh] calypso-ipc-device pas configure (CALYPSO_IPC_DEVICE vide) -- osmo-trx-ipc va echouer"
    fi
else
    echo "[run.sh] SKIP_IPC_DEVICE=1 -- calypso-ipc-device non lance"
fi

# ---------- 3bis. osmo-trx-ipc ----------
# Connect to $IPC_MSOCK_PATH, expose TRXD UDP 5700-5702 vers osmo-bts-trx.
# Si calypso-ipc-device n'est pas demarre, osmo-trx-ipc exit immediatement
# (greeting_req sans reponse : erreur). C'est OK en transition : sa fenetre
# tmux reste, on voit le message d'erreur, on relance quand le device est pret.
if [ "${CALYPSO_SKIP_TRX_IPC:-0}" != "1" ]; then
    tmux new-window -t "$SESSION" -n osmo-trx-ipc
    tmux send-keys -t "$SESSION:osmo-trx-ipc" \
        ": > $OSMO_TRX_IPC_LOG && $OSMO_TRX_IPC -C $OSMO_TRX_IPC_CFG 2>&1 | $TSLOG | tee $OSMO_TRX_IPC_LOG" C-m
else
    echo "[run.sh] SKIP_TRX_IPC=1 -- osmo-trx-ipc non lance"
fi

# ---------- 3ter. bridge.py (legacy) ----------
# Mode bridge : pont Python pur (UDP 5700-5702 <-> UDP 6702) au lieu d'osmo-trx-ipc.
# Wall-clock-paced FN counter, sercomm soft:I/Q GMSK inline (BRIDGE_BSP_IQ=1).
if [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" != "1" ]; then
    BRIDGE_PY="${BRIDGE_PY:-/opt/GSM/qemu-src/bridge.py}"
    BRIDGE_LOG="${BRIDGE_LOG:-/tmp/bridge.py.log}"
    if [ -x "$BRIDGE_PY" ]; then
        tmux new-window -t "$SESSION" -n bridge-py
        # /root/.env = venv avec gnuradio/gr-gsm/numpy/scipy.
        # Fallback python3 si venv absent (env minimal).
        BRIDGE_PYTHON="${CALYPSO_BRIDGE_PYTHON:-/root/.env/bin/python3}"
        [ -x "$BRIDGE_PYTHON" ] || BRIDGE_PYTHON=python3
        tmux send-keys -t "$SESSION:bridge-py" \
            ": > $BRIDGE_LOG && BRIDGE_BSP_IQ=${BRIDGE_BSP_IQ:-1} CALYPSO_DOPPLER_HZ=${CALYPSO_DOPPLER_HZ:-0} CALYPSO_BURST_PRINT=${CALYPSO_BURST_PRINT:-0} $BRIDGE_PYTHON -u $BRIDGE_PY 2>&1 | $TSLOG | tee $BRIDGE_LOG" C-m
        echo "[run.sh] bridge.py lance (legacy mode, python=$BRIDGE_PYTHON)"
    else
        echo "[run.sh] WARN bridge.py introuvable ($BRIDGE_PY) -- mode bridge casse"
    fi
fi

# ---------- 4. osmo-bts-trx ----------
if [ "${CALYPSO_SKIP_BTS:-0}" != "1" ]; then
    tmux new-window -t "$SESSION" -n bts
    tmux send-keys -t "$SESSION:bts" \
        ": > $BTS_LOG && osmo-bts-trx -c $BTS_CFG 2>&1 | $TSLOG | tee $BTS_LOG" C-m
else
    echo "[run.sh] SKIP_BTS=1 -- osmo-bts-trx non lance"
fi

# ---------- 4bis. bridge de démod native (gr-gsm) ----------
# Chemin SANS HACK : le BTS émet l'I/Q DL réelle → le BSP qemu (sous shunt) la
# tee vers UDP $CALYPSO_IQ_TEE_PORT (6703). Ce bridge la démodule (GMSK diff) +
# la décode (gr-gsm gsm_bcch_ccch_demapper + control_channels_decoder) → GSMTAP
# 127.0.0.1:$CALYPSO_SHUNT_GSMTAP_PORT (4730) → le listener du shunt appelle
# feed_si → a_cd. Le mobile campe sur le VRAI SI démodulé du signal du BTS.
# Requiert : shunt actif (tee) + BTS présent (I/Q). venv /root/.env (gnuradio+gsm).
if [ "${CALYPSO_DSP_SHUNT:-0}" = "1" ] && [ "${CALYPSO_SKIP_BTS:-0}" != "1" ] \
   && [ "${CALYPSO_SKIP_DEMOD_BRIDGE:-1}" != "1" ]; then
    DEMOD_BRIDGE="${CALYPSO_DEMOD_BRIDGE:-/opt/GSM/qemu_bcch_grgsm.py}"
    DEMOD_BRIDGE_LOG="${DEMOD_BRIDGE_LOG:-/tmp/demod_bridge.log}"
    DEMOD_PYTHON="${CALYPSO_BRIDGE_PYTHON:-/root/.env/bin/python3}"
    [ -x "$DEMOD_PYTHON" ] || DEMOD_PYTHON=python3
    if [ -f "$DEMOD_BRIDGE" ]; then
        tmux new-window -t "$SESSION" -n demod-bridge
        tmux send-keys -t "$SESSION:demod-bridge" \
            ": > $DEMOD_BRIDGE_LOG && IQ_TEE_PORT=${CALYPSO_IQ_TEE_PORT:-6703} \
GSMTAP_PORT=${CALYPSO_SHUNT_GSMTAP_PORT:-4730} ARFCN=${CALYPSO_CCCH_ARFCN:-514} \
BIT_SIGN=${BIT_SIGN:-1} $DEMOD_PYTHON -u $DEMOD_BRIDGE 2>&1 | $TSLOG | tee $DEMOD_BRIDGE_LOG" C-m
        echo "[run.sh] demod-bridge (gr-gsm) : I/Q ${CALYPSO_IQ_TEE_PORT:-6703} -> GSMTAP ${CALYPSO_SHUNT_GSMTAP_PORT:-4730} -> a_cd (python=$DEMOD_PYTHON)"
    else
        echo "[run.sh] WARN demod-bridge introuvable ($DEMOD_BRIDGE) -- pas de démod native"
    fi
fi

# ---------- 5. L2 client (mobile / ccch_scan / cell_log) ----------
# Sync barrier inline dans la cmd tmux : attendre que la socket L1CTL
# existe (creee par osmocon apres son handshake romload avec le firmware)
# avant de lancer mobile. Evite le `sleep 3` arbitraire et le 51s spread
# observe.
#
# AUDIT 2026-05-26 : TOUS les mobile cfgs (host + container) utilisent
# `layer2-socket /tmp/osmocom_l2` SANS suffixe `_1`. Le symlink
# `${L1CTL_SOCK}_1 -> ${L1CTL_SOCK}` est conserve en DUMMY/safety net
# (au cas ou un cfg externe utiliserait l'ancien format `_1`). Sans
# producteur de ce path, il n'est pas obligatoire mais inoffensif.
#
# SAP socket : osef (pas de mocksapd, SIM natif via cfg, pas branche).
L1CTL_WAIT='i=0; while [ ! -S '"$L1CTL_SOCK"' ] && [ $i -lt 60 ]; do sleep 0.5; i=$((i+1)); done; [ -S '"$L1CTL_SOCK"' ] && ln -sf '"$L1CTL_SOCK"' '"$L1CTL_SOCK"'_1 2>/dev/null || true'

# Categories debug mobile : ajout DPLMN/DGS pour voir "no cell found",
# selection PLMN/cellule, etat MM/RR. Override via CALYPSO_MOBILE_DEBUG.
# Categories utiles (osmocom-bb common.c) :
#   DRR     radio resource (FBSB attempts)
#   DMM     mobility (Mobile_Mngt + no_cell_found events)
#   DCC     call control
#   DLAPDM  L2 LAPDm
#   DCS     cell selection (gsm322)
#   DSAP    SAP socket
#   DPAG    paging
#   DL1C    L1CTL trace (PM_REQ/CONF, FBSB_REQ/CONF, etc)
#   DSUM    summary
#   DSI     system info (SI3/4/5/6 reception)
#   DRSL    Radio Signalling Link
#   DNM     Network Management
#   DPLMN   PLMN selection (HPLMN search, no_plmn_found)
#   DGS     gsm subscriber state
#   DSMS    SMS
#   DSS     supplementary services
#   DGS     general state
CALYPSO_MOBILE_DEBUG="${CALYPSO_MOBILE_DEBUG:-DCS:DNB:DPLMN:DRR:DMM:DSIM:DCC:DMNCC:DSS:DLSMS:DPAG:DSUM:DSAP:DGPS:DMOB:DPRIM:DLUA:DGAPK}"
# `all` = alias du mask par defaut complet (cf `mobile -h`).
# Note : separateur = `:` pour osmocom-bb mobile (pas `,`).
if [ "$CALYPSO_MOBILE_DEBUG" = "all" ]; then
    CALYPSO_MOBILE_DEBUG="DCS:DNB:DPLMN:DRR:DMM:DSIM:DCC:DMNCC:DSS:DLSMS:DPAG:DSUM:DSAP:DGPS:DMOB:DPRIM:DLUA:DGAPK"
fi

# ---------- 3ter. DSP gr-gsm shm (mode full-grgsm) ----------
# gr-gsm EST le DSP, branché sur le BUFFER SHM (pas UDP/relay) : le BSP publie
# l'I/Q d'entrée du DSP shunté dans /dev/shm/calypso_dsp_shunt (ring, via
# feed_iq) ; grgsm_shm_decode.py lit le ring -> demod (theta) + decode BCCH ->
# ecrit le SI dans la zone sortie -> shunt_poll_si_shm -> feed_si -> a_cd -> le
# mobile (osmocon) campe sur la VRAIE SI decodee. mmap = fix shmat conteneur.
if [ "$CALYPSO_MODE" = "full-grgsm" ]; then
    # RECORD drainer : iq_record.fifo (ecrit NON-BLOQUANT par qemu, frame par
    # frame) -> /tmp/record.cfile (ring 128MB) HORS du hot-path qemu => PLUS
    # d underrun (c est le fwrite du ring DANS qemu qui les causait). si_bridge
    # relit record.cfile offline (= ancien comportement, SI preserve).
    pkill -9 -f record_drain.py 2>/dev/null || true
    ( python3 /opt/GSM/record_drain.py >/tmp/record_drain.log 2>&1 & )
    tmux new-window -t "$SESSION" -n grgsm-decode
    # Decodeur gr-gsm AUTO (defaut = relay). grgsm_relay_decode.py lit l'I/Q
    # continu relaye par l'IPC device (UDP 5810) -> gsm.receiver (sync FCCH/SCH,
    # patch grgsm-receiver-publish-bsic-fn) -> SI vers GSMTAP 4730 (feed_si) ET
    # BSIC/FN REELS vers UDP 4731 (feed_sb -> shunt_dispatch_sb, remplace
    # SHUNT_CANNED_BSIC). Subsume le si_bridge (grgsm_decode sur fifo = SI seul,
    # pas de SCH). CALYPSO_GRGSM_DECODER=si-bridge pour revenir au SI-only legacy.
    RELAY_DECODE=/opt/GSM/qemu-src/opt-gsm-scripts/grgsm_relay_decode.py
    # DEFAUT = si-bridge (PROUVE : grgsm_decode sur la FIFO /tmp/iq_grgsm.fifo,
    # SI OK). Le relay_decode lit UDP 5810 qui n'est PAS alimente ici -> SI mort
    # (regression 2026-06-04). CALYPSO_GRGSM_DECODER=relay seulement si 5810 est
    # reellement nourri. Le SCH/BSIC reel passe par un autre chemin (cf si_bridge).
    if [ "${CALYPSO_GRGSM_DECODER:-si-bridge}" = "si-bridge" ]; then
        tmux send-keys -t "$SESSION:grgsm-decode" \
            "sleep 15; bash /opt/GSM/si_bridge_loop.sh 2>&1 | $TSLOG | tee /tmp/grgsm_decode.log" C-m
    else
        tmux send-keys -t "$SESSION:grgsm-decode" \
            "sleep 15; source /root/.env/bin/activate; python3 -u $RELAY_DECODE 2>&1 | $TSLOG | tee /tmp/grgsm_decode.log" C-m
    fi
    echo "[run.sh] full-grgsm : decodeur gr-gsm (${CALYPSO_GRGSM_DECODER:-relay}) lance -> SI (4730) + SCH/BSIC reel (4731)"
    # Bridge SI : grgsm_decode lit le FIFO LIVE /tmp/iq_grgsm.fifo (flux continu pousse
    # -s 1083333 = 4 SPS) DECODE le vrai SI2/SI3/SI4/SI13 de la BTS -> parse ->
    # GSMTAP 4730 -> shunt feed_si -> a_cd + DATA_IND -> mobile. C'est CE chemin qui
    # casse le mur demod (le cfile continu, pas les bursts BSP discontinus).
    # Gated CALYPSO_SI_BRIDGE (defaut 1). sleep 15 = laisse la pile cfile se remplir.
fi

if [ "${CALYPSO_SKIP_L2:-0}" != "1" ]; then
    tmux new-window -t "$SESSION" -n "$CALYPSO_L2_CLIENT"
    case "$CALYPSO_L2_CLIENT" in
        mobile)
            # Timer 3s sur le mobile : après le wait-socket L1CTL, laisser le
            # firmware/pipeline se stabiliser avant que le mobile se connecte
            # (l'émulation tourne lent en wall ; le socket peut exister avant
            # que la L1 soit prête). Overridable via CALYPSO_MOBILE_DELAY.
            tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
                ": > $MOBILE_LOG && $L1CTL_WAIT && sleep ${CALYPSO_MOBILE_DELAY:-3} && mobile -c $MOBILE_CFG -d $CALYPSO_MOBILE_DEBUG 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
            ;;
        ccch_scan)
            tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
                "$L1CTL_WAIT && ccch_scan -a ${CALYPSO_CCCH_ARFCN:-1} 2>&1 | $TSLOG | tee $L2_LOG" C-m
            ;;
        cell_log)
            tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
                "$L1CTL_WAIT && cell_log 2>&1 | $TSLOG | tee $L2_LOG" C-m
            ;;
        *)
            echo "WARN -- CALYPSO_L2_CLIENT=$CALYPSO_L2_CLIENT inconnu, fallback mobile"
            tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
                ": > $MOBILE_LOG && $L1CTL_WAIT && mobile -c $MOBILE_CFG -d $CALYPSO_MOBILE_DEBUG 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
            ;;
    esac
else
    echo "[run.sh] SKIP_L2=1 -- L2 client non lance"
fi

# ---------- 6. gsmtap capture (any iface -- covers eth0 mobile/BTS + eth1) ----------
# `--print` affiche en live ET continue d'ecrire le pcap ; `-U` flush par paquet
# pour que le pcap soit utilisable immediatement (sinon buffer 4KB).
if [ "${CALYPSO_SKIP_GSMTAP:-0}" != "1" ]; then
tmux new-window -t "$SESSION" -n gsmtap
tmux send-keys -t "$SESSION:gsmtap" \
    "sleep 5 && tcpdump -i any -U --print -X -w /root/mobile-gsmtap.pcap udp port 4729" C-m
fi  # CALYPSO_SKIP_GSMTAP

# ---------- 7. window 'all' -- agrege les 6 premieres en 6 panes ----------
# Vue unique pour superviser qemu / irda / osmocon / bts / L2 client
# cote a cote. Chaque pane fait juste `tail -F` du log correspondant. Layout
# tiled = grille 2x3 par defaut. Les logs n'existent peut-etre pas encore au
# moment de la creation des panes -- `tail -F` (majuscule) gere ce cas (suit
# le fichier des qu'il apparait) sans crash.
case "$CALYPSO_L2_CLIENT" in
    ccch_scan|cell_log) L2_TAIL_LOG="$L2_LOG" ;;
    *)                  L2_TAIL_LOG="$MOBILE_LOG" ;;
esac
# Creation sequentielle : `select-layout tiled` apres chaque split pour
# redistribuer l'espace, sinon la 3e/4e pane devient trop etroite et tmux
# rejette le split suivant avec "no space for new pane".
tmux new-window -t "$SESSION" -n all \
    "clear; echo '=== qemu ==='; tail -F $QEMU_LOG"
# Build dynamic spec list selon ce qui tourne reellement.
_ALL_SPECS=("osmocon|$OSMOCON_LOG")
[ "${CALYPSO_SKIP_GSMTAP:-0}" != "1" ] && _ALL_SPECS+=("fft|__FFT__")
[ "${CALYPSO_SKIP_IPC_DEVICE:-0}" != "1" ] && _ALL_SPECS+=("ipc-device|$IPC_DEVICE_LOG")
[ "${CALYPSO_SKIP_TRX_IPC:-0}" != "1" ] && _ALL_SPECS+=("osmo-trx-ipc|$OSMO_TRX_IPC_LOG")
[ "${CALYPSO_SKIP_BRIDGE_PY:-1}" != "1" ] && _ALL_SPECS+=("bridge-py|${BRIDGE_LOG:-/tmp/bridge.py.log}")
[ "${CALYPSO_SKIP_BTS:-0}" != "1" ] && _ALL_SPECS+=("bts|$BTS_LOG")
[ "${CALYPSO_SKIP_L2:-0}" != "1" ] && _ALL_SPECS+=("$CALYPSO_L2_CLIENT|$L2_TAIL_LOG")
# gdb pane dans la window 'all'. Default OFF (CALYPSO_SKIP_GDB_PANE=1).
# Activer avec CALYPSO_SKIP_GDB_PANE=0 (= opt-in pour debug). Le pane gdb
# attache au gdb-stub QEMU au démarrage, prêt pour b/c/info reg.
[ "${CALYPSO_SKIP_GDB_PANE:-1}" != "1" ] && _ALL_SPECS+=("gdb|__GDB__")
for spec in "${_ALL_SPECS[@]}"; do
    name="${spec%%|*}"; log="${spec##*|}"
    if [ "$log" = "__FFT__" ]; then
        # FFT live du cfile BSP (lien grgsm<->BSP) — remplace tcpdump (vire pour l'instant).
        cmd="clear; source /root/.env/bin/activate 2>/dev/null; CFILE=/tmp/iq_asciifft.fifo FS=1083333 python3 /opt/GSM/grgsm_fft_live.py"
    elif [ "$log" = "__GDB__" ]; then
        # gdb-multiarch attaché au QEMU gdb-stub. Sleep 3 pour laisser QEMU
        # finir son init et binder le port. Pas de script -x : prompt vide
        # pour usage interactif. Override script via CALYPSO_GDB_PANE_SCRIPT.
        _GDB_ELF="${CALYPSO_GDB_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
        _GDB_PORT="${CALYPSO_GDB_PORT:-1234}"
        if [ -n "${CALYPSO_GDB_PANE_SCRIPT:-}" ]; then
            _GDB_X="-x $CALYPSO_GDB_PANE_SCRIPT"
        else
            _GDB_X=""
        fi
        cmd="clear; echo '=== $name (attach :$_GDB_PORT) ==='; sleep 3 && gdb-multiarch -q -iex 'set pagination off' -iex 'set confirm off' -iex 'set architecture armv5te' -iex 'target remote :$_GDB_PORT' $_GDB_X $_GDB_ELF ; echo '[gdb exited, press Enter]'; read"
    else
        cmd="clear; echo '=== $name ==='; tail -F $log"
    fi
    tmux split-window -t "$SESSION:all" "$cmd" 2>/dev/null || true
    tmux select-layout -t "$SESSION:all" tiled
done

# ---------- 8. gen-doc auto (30s apres QEMU launch) ----------
# Une fenetre tmux dediee qui attend 30s que le pipeline se stabilise puis
# lance pytest in-container pour produire la doc en arriere-plan. Pas de
# rappel recursif a run.sh -- on inline la commande pytest avec ses ignores
# et env vars. Desactivable via CALYPSO_AUTO_GEN_DOC=0.
CALYPSO_AUTO_GEN_DOC="${CALYPSO_AUTO_GEN_DOC:-1}"
if [ "$CALYPSO_AUTO_GEN_DOC" = "1" ]; then
    # Pre-install des deps Python (silencieux, idempotent)
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

    # Pytest verbosity (set par le menu, ou default v).
    case "${CALYPSO_PYTEST_VERBOSITY:-v}" in
      q)   PY_VERBOSITY="-q --no-header" ;;
      v)   PY_VERBOSITY="-v --tb=short --color=yes" ;;
      vv)  PY_VERBOSITY="-vv --tb=long --color=yes" ;;
      vvv) PY_VERBOSITY="-vvv --tb=long --color=yes --log-cli-level=DEBUG" ;;
      *)   PY_VERBOSITY="-v --tb=short --color=yes" ;;
    esac

    # Pytest scope (set par le menu, ou default).
    case "${CALYPSO_PYTEST_SCOPE:-default}" in
      smoke)
        PY_TARGET="test_run_all_modes.py"
        PY_IGNORES=""
        ;;
      all)
        PY_TARGET=""
        PY_IGNORES=""
        ;;
      calypso)
        PY_TARGET="calypso/"
        PY_IGNORES=""
        ;;
      default|*)
        PY_TARGET=""
        PY_IGNORES="--ignore=functional --ignore=guest-debug \
                    --ignore=qemu-iotests --ignore=qtest --ignore=unit \
                    --ignore=tcg --ignore=migration --ignore=vm \
                    --ignore=avocado --ignore=fp"
        ;;
    esac

    tmux new-window -t "$SESSION" -n gen-doc
    tmux send-keys -t "$SESSION:gen-doc" \
        "echo '[gen-doc] waiting 60s for pipeline to stabilize...'; sleep 60; \
         echo '[gen-doc] launching pytest in-container (verbosity=${CALYPSO_PYTEST_VERBOSITY:-v} scope=${CALYPSO_PYTEST_SCOPE:-default})'; \
         cd /opt/GSM/qemu-src/tests && \
         CALYPSO_TEST_OUT=/tmp \
         CALYPSO_REPO=/opt/GSM/qemu-src \
         CALYPSO_HOST_ROOT=/root \
         CALYPSO_MODE_TAG=$CALYPSO_MODE \
         $GEN_DOC_PYTEST $PY_VERBOSITY $PY_IGNORES $PY_TARGET; \
         echo; echo '=== Artefacts ==='; \
         ls -la /tmp/report.md /tmp/test_results.md /tmp/test_results.qmd /tmp/test_results_latest.zip 2>/dev/null; \
         echo; echo '[gen-doc] done. Press <Enter> to keep window open.'; read -r _" C-m
fi

# ---------- shell + attach ----------
tmux new-window -t "$SESSION" -n shell

# ---------- GDB window (CALYPSO_GDB_PANE=1) ----------
# Crée une window tmux 'gdb' avec gdb-multiarch attaché à QEMU + tail qemu.log
# en split. Optionnel, gated par CALYPSO_GDB_PANE=1 (menu Advanced "GDB_PANE").
# Si CALYPSO_QEMU_HALT=1 aussi, gdb attache à QEMU halted -> set BPs + 'c'.
# Script gdb optionnel via CALYPSO_GDB_SCRIPT=/path/to/script.gdb (default :
# script minimal qui attache et continue).
if [ "${CALYPSO_GDB_PANE:-0}" = "1" ]; then
    GDB_SCRIPT_DEFAULT=/tmp/run_sh_gdb.gdb
    if [ -z "${CALYPSO_GDB_SCRIPT:-}" ]; then
        cat > "$GDB_SCRIPT_DEFAULT" <<GDB_EOF
set pagination off
set confirm off
set architecture armv5te
target remote :${CALYPSO_GDB_PORT:-1234}
# Default : juste attacher et continuer. Override via CALYPSO_GDB_SCRIPT.
c
GDB_EOF
        CALYPSO_GDB_SCRIPT="$GDB_SCRIPT_DEFAULT"
    fi
    GDB_ELF="${CALYPSO_GDB_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
    tmux new-window -t "$SESSION" -n gdb \
        "gdb-multiarch -q -iex 'set pagination off' -iex 'set confirm off' \
         -x $CALYPSO_GDB_SCRIPT $GDB_ELF ; \
         echo '[run.sh] gdb exited, press Enter'; read"
    tmux split-window -t "$SESSION:gdb" -v -p 40 "tail -f $QEMU_LOG"
    tmux select-pane -t "$SESSION:gdb.0"
    echo "[run.sh] CALYPSO_GDB_PANE=1 -- window tmux 'gdb' créée (top=gdb / bottom=tail qemu.log)"
    echo "         gdb script : $CALYPSO_GDB_SCRIPT"
fi

echo
echo "Pipeline launched. Attach with: tmux attach -t $SESSION"

# Build identity dump (2026-05-25) -- attribution causale dans report.
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
# Marqueurs decoder fixes -- grep des strings reellement dans le binaire
# (= dans le format string BUILD-IDENT compile en .rodata, pas dans les
# commentaires C qui sont strippes). Si la string disparait = fix retire.
QEMU_BIN="${QEMU:-/opt/GSM/qemu-src/build/qemu-system-arm}"
if [ -x "$QEMU_BIN" ]; then
    BUILD_IDENT_LINE=$(strings "$QEMU_BIN" 2>/dev/null | grep 'BUILD-IDENT decoder-fixes:' | head -1)
    if [ -n "$BUILD_IDENT_LINE" ]; then
        case "$BUILD_IDENT_LINE" in
            *F1xx-FIRS-catch=REMOVED*)
                echo "  decoder-fix F1xx FIRS catch:      [X] REMOVED" ;;
            *)
                echo "  decoder-fix F1xx FIRS catch:      ! NOT FOUND in binary" ;;
        esac
        case "$BUILD_IDENT_LINE" in
            *L3609-src-dst=FIXED*)
                echo "  decoder-fix L3609 src/dst swap:   [X] APPLIED" ;;
            *)
                echo "  decoder-fix L3609 src/dst swap:   ! NOT FOUND in binary" ;;
        esac
        case "$BUILD_IDENT_LINE" in
            *F-AUDIT-v5=max-min-cmpl-rnd-roltc-fixed*)
                echo "  decoder-fix F-AUDIT v5:           [X] APPLIED (max/min/cmpl/rnd/roltc)" ;;
            *)
                echo "  decoder-fix F-AUDIT v5:           ! NOT FOUND in binary" ;;
        esac
        case "$BUILD_IDENT_LINE" in
            *F2xx-ALU-block=ADDED*)
                echo "  decoder-fix F2xx ALU block:       [X] ADDED (binutils-strict bit9=src bit8=dst)" ;;
            *)
                echo "  decoder-fix F2xx ALU block:       ! NOT FOUND in binary" ;;
        esac
        case "$BUILD_IDENT_LINE" in
            *F3xx-INTR-mis-REMOVED*)
                echo "  decoder-fix F3xx INTR mis-decode: [X] REMOVED (was F300/wrong, real INTR=F7C0)" ;;
            *)
                echo "  decoder-fix F3xx INTR mis-decode: ! NOT FOUND in binary" ;;
        esac
    else
        echo "  decoder-fix markers:              ! BUILD-IDENT not in binary (= old build)"
    fi
fi
echo "=========================="
echo

echo "ENV summary:"
echo "  CALYPSO_BSP_DARAM_ADDR      = $CALYPSO_BSP_DARAM_ADDR"
echo "  CALYPSO_SIM_CFG             = $CALYPSO_SIM_CFG"
echo "  CALYPSO_PROBE_BOOTSTUB      = $CALYPSO_PROBE_BOOTSTUB        (0=normal, 1=HACK probe-inject bootstub)"
echo "  CALYPSO_W1C_LATCH           = $CALYPSO_W1C_LATCH"
echo "  CALYPSO_NDB_D_RACH_OFFSET   = ${CALYPSO_NDB_D_RACH_OFFSET:-(default 0x023a -- pinned 2026-05-07)}"
echo "  CALYPSO_RACH_FORCE_BSIC     = ${CALYPSO_RACH_FORCE_BSIC:-(unset = use d_rach byte)}"
echo "  CALYPSO_ICOUNT              = $CALYPSO_ICOUNT  (flag: ${QEMU_ICOUNT_FLAG:-(none)})"
echo "  CALYPSO_MTTCG               = ${CALYPSO_MTTCG:-0}  $([ "${CALYPSO_MTTCG:-0}" = "1" ] && echo '! NON-DETERMINISTE -- donnees non-citables pour state claims (cf session 2026-05-25)' || echo '(icount-deterministic [X])')"
echo "  CALYPSO_TIMER               = $CALYPSO_TIMER  (1=fprintf tdma_tick/frame_irq/kick : qemu.log, 0=silent)"
echo "  CALYPSO_DSP_IDLE_FF         = $CALYPSO_DSP_IDLE_FF  (1=fast-forward DSP idle dispatcher)"
echo "  CALYPSO_DSP_IDLE_RANGE      = ${CALYPSO_DSP_IDLE_RANGE:-(default 0xe9ac:0xe9b7,0xcc62:0xcc6f)}"
echo "  CALYPSO_IRDA_CAPTURE        = $CALYPSO_IRDA_CAPTURE  (1=consume serial1 PTY : /tmp/fw-irda.log)"
echo "  OSMO_TRX_IPC                = $OSMO_TRX_IPC"
echo "  OSMO_TRX_IPC_CFG            = $OSMO_TRX_IPC_CFG"
echo "  CALYPSO_IPC_DEVICE          = ${CALYPSO_IPC_DEVICE:-(unset -- Phase 1 TODO, osmo-trx-ipc echouera)}"
echo "  IPC_MSOCK_PATH              = $IPC_MSOCK_PATH"
if [ "$CALYPSO_IRDA_CAPTURE" = "1" ] && [ -n "${PTY_IRDA:-}" ]; then
    echo "  IrDA channel                = $PTY_IRDA : /tmp/irda.pty.link : /tmp/fw-irda.log"
fi
echo
echo "Manual warm-start (debug, if BSC unavailable) :"
echo "  /opt/GSM/qemu-src/scripts/populate-si.sh"
echo

tmux select-window -t "$SESSION:all" 2>/dev/null || tmux select-window -t "$SESSION:qemu"

# CALYPSO_NO_ATTACH=1 : ne pas attacher tmux (mode non-interactif, utile
# pour run-all.sh ou tests pytest qui orchestrent plusieurs runs).
if [ "${CALYPSO_NO_ATTACH:-0}" = "1" ]; then
    echo "[run.sh] CALYPSO_NO_ATTACH=1 -- session tmux '$SESSION' tourne en background"
    echo "[run.sh] Attach manuel : tmux attach -t $SESSION"
    echo "[run.sh] Kill : tmux kill-session -t $SESSION"
    exit 0
fi
exec tmux attach -t "$SESSION"
