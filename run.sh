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
for arg in "$@"; do
    case "$arg" in
        --gen-doc-local) GEN_DOC_MODE=1 ;;
        --menu) MENU_MODE=1 ;;
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
            cat <<EOF
Usage: $0 [--gen-doc-local] [--menu]

Default : launch the full Calypso pipeline (QEMU + osmocon + calypso-ipc-device
+ osmo-trx-ipc + bts + mobile + gsmtap + irda) in a tmux session and attach.

--menu      : interactive profile selection before launch (hack-free /
              dev-assist / full-diag / custom). Sets CALYPSO_* env vars
              and re-execs.

--gen-doc-local : skip pipeline launch, run pytest in the container "trying"
            (no requirement that qemu-system-arm be running -- tests can
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
      --notags --menu \
      "\n Pick a preset (ENTER on item to select). Level 2 inherits it.\n\n \
 Fine override: env vars CALYPSO_SKIP_*, CALYPSO_DSP_SHUNT, ...\
" \
      20 84 7 \
      "full"      "Full radio pipeline (default legacy)" \
      "shunt"     "DSP shunt canned -- bissection FBSB" \
      "shunt-ipc" "DSP shunt + radio chain" \
      "bridge"    "Legacy bridge.py Python" \
      "bare"      "QEMU + osmocon only" \
      "free"      "Free mode -- pick everything yourself" \
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
      14 76 3 \
      "none"    "none -- production-like (recommended)" \
      "minimal" "minimal -- hack-free + diag minimal" \
      "full"    "full -- SP_RING + AR + WATCH + CORR + MVPD" \
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
      minimal)
        export CALYPSO_PROBE_BOOTSTUB=0
        ;;
      full)
        export CALYPSO_PROBE_BOOTSTUB=0
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
    esac

    export CALYPSO_DSP_SHUNT CALYPSO_MODE CALYPSO_MTTCG \
           CALYPSO_IRDA_CAPTURE CALYPSO_AUTO_GEN_DOC
    export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
           CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY

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
CALYPSO_MODE="${CALYPSO_MODE:-full}"
case "$CALYPSO_MODE" in
    full|shunt|shunt-ipc|bridge|bare|free) ;;
    *) echo "[run.sh] ERR : CALYPSO_MODE=$CALYPSO_MODE inconnu (full|shunt|shunt-ipc|bridge|bare|free)" >&2; exit 1 ;;
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
        : "${CALYPSO_DSP_SHUNT:=1}"
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"
        : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}"
        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}"
        : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
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
export CALYPSO_DSP_SHUNT CALYPSO_MODE
export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
       CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY

# ---- Auto-resolution implications / exclusions ----
# Apres les presets + override env, on applique les regles de coherence.
# Chaque regle log explicitement ce qu'elle force pour pas surprendre.

# Regle 1 : DSP_SHUNT=1 : BTS chain inutile (canned ne touche pas la radio).
# N'override pas si user a explicitement set SKIP_BTS=0 (= "je veux la
# radio quand meme meme en shunt", cas shunt-ipc).
if [ "$CALYPSO_DSP_SHUNT" = "1" ] && [ "$CALYPSO_MODE" != "shunt-ipc" ]; then
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
if [ "${CALYPSO_MTTCG:-0}" = "1" ] && [ "${CALYPSO_ICOUNT:-off}" != "off" ]; then
    echo "[run.sh] MUTEX MTTCG <-> icount -- sera force ICOUNT=off (cf bloc MTTCG)"
fi

export CALYPSO_SKIP_IPC_DEVICE CALYPSO_SKIP_TRX_IPC CALYPSO_SKIP_BTS \
       CALYPSO_SKIP_L2 CALYPSO_SKIP_GSMTAP CALYPSO_SKIP_BRIDGE_PY

# ---- DSP / DIAG instruments (override at command line if needed) ----
CALYPSO_DSP_ROM="${CALYPSO_DSP_ROM:-/opt/GSM/calypso_dsp.txt}"
CALYPSO_BSP_DARAM_ADDR="${CALYPSO_BSP_DARAM_ADDR:-0x3fb0}"
CALYPSO_SIM_CFG="${CALYPSO_SIM_CFG:-$MOBILE_CFG}"
export CALYPSO_DSP_ROM CALYPSO_BSP_DARAM_ADDR CALYPSO_SIM_CFG

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
CALYPSO_W1C_LATCH="${CALYPSO_W1C_LATCH:-0}"
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
CALYPSO_ICOUNT="${CALYPSO_ICOUNT:-off}"  # default off = wall-clock (auto bloque actuellement, debug en cours)
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
rm -f "$QEMU_LOG" "$OSMOCON_LOG" "$MOBILE_LOG" "$BTS_LOG" \
      "$OSMO_TRX_IPC_LOG" "$IPC_DEVICE_LOG" \
      "$MON_SOCK" "$L1CTL_SOCK" "$QEMU_DUMMY_SOCK" \
      "$IPC_MSOCK_PATH" "${IPC_MSOCK_PATH}_0"

tmux kill-session -t "$SESSION" 2>/dev/null || true
killall -9 qemu-system-arm osmo-bts-trx mobile osmocon osmo-trx-ipc 2>/dev/null || true
pkill -9 -f calypso-ipc-device 2>/dev/null || true
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

# Override delibere : pour le child QEMU seulement, L1CTL_SOCK pointe vers
# le dummy (/tmp/qemu_l1ctl_disabled). QEMU/l1ctl_sock.c cree son socket a
# cette adresse-poubelle (= L1CTL QEMU desactive). Le VRAI socket L1CTL
# /tmp/osmocom_l2 est cree plus tard par osmocon (L541 : -s $L1CTL_SOCK,
# ou L1CTL_SOCK garde sa valeur d'origine = /tmp/osmocom_l2). Hors de
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
    echo " TIMEOUT -- no PTY in $QEMU_LOG"
    exit 1
fi
echo " OK ($PTY_MODEM, QEMU_PID=$QEMU_PID)"

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
    ": > $OSMOCON_LOG && $OSMOCON -m romload -i 100 -p $PTY_MODEM -s $L1CTL_SOCK $FW_BIN -d tr 2>&1 | $TSLOG | tee $OSMOCON_LOG" C-m

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
            ": > $IPC_DEVICE_LOG && $CALYPSO_IPC_DEVICE -u /tmp -n 0 2>&1 | $TSLOG | tee $IPC_DEVICE_LOG" C-m
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
        tmux send-keys -t "$SESSION:bridge-py" \
            ": > $BRIDGE_LOG && BRIDGE_BSP_IQ=${BRIDGE_BSP_IQ:-1} python3 -u $BRIDGE_PY 2>&1 | $TSLOG | tee $BRIDGE_LOG" C-m
        echo "[run.sh] bridge.py lance (legacy mode)"
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

if [ "${CALYPSO_SKIP_L2:-0}" != "1" ]; then
    tmux new-window -t "$SESSION" -n "$CALYPSO_L2_CLIENT"
    case "$CALYPSO_L2_CLIENT" in
        mobile)
            tmux send-keys -t "$SESSION:$CALYPSO_L2_CLIENT" \
                ": > $MOBILE_LOG && $L1CTL_WAIT && mobile -c $MOBILE_CFG -d $CALYPSO_MOBILE_DEBUG 2>&1 | $TSLOG | tee $MOBILE_LOG" C-m
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
[ "${CALYPSO_SKIP_GSMTAP:-0}" != "1" ] && _ALL_SPECS+=("tcpdump|__TCPDUMP__")
[ "${CALYPSO_SKIP_IPC_DEVICE:-0}" != "1" ] && _ALL_SPECS+=("ipc-device|$IPC_DEVICE_LOG")
[ "${CALYPSO_SKIP_TRX_IPC:-0}" != "1" ] && _ALL_SPECS+=("osmo-trx-ipc|$OSMO_TRX_IPC_LOG")
[ "${CALYPSO_SKIP_BRIDGE_PY:-1}" != "1" ] && _ALL_SPECS+=("bridge-py|${BRIDGE_LOG:-/tmp/bridge.py.log}")
[ "${CALYPSO_SKIP_BTS:-0}" != "1" ] && _ALL_SPECS+=("bts|$BTS_LOG")
[ "${CALYPSO_SKIP_L2:-0}" != "1" ] && _ALL_SPECS+=("$CALYPSO_L2_CLIENT|$L2_TAIL_LOG")
for spec in "${_ALL_SPECS[@]}"; do
    name="${spec%%|*}"; log="${spec##*|}"
    if [ "$log" = "__TCPDUMP__" ]; then
        # Live tcpdump hex + ASCII (sans -w pour ne pas dupliquer le pcap
        # canonique gere par la fenetre `gsmtap`).
        cmd="clear; echo '=== $name ==='; sleep 5 && tcpdump -i any -X udp port 4729"
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
echo "  CALYPSO_DSP_ROM             = $CALYPSO_DSP_ROM"
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
