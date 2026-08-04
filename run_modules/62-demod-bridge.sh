# =============================================================================
#  62-demod-bridge — démodulation native gr-gsm de l'I/Q descendante
# =============================================================================
#
#  RÔLE (run.sh.legacy L2011-2038)
#      Le BTS émet l'I/Q descendante réelle ; le BSP de QEMU la duplique vers
#      UDP $CALYPSO_IQ_TEE_PORT (6703). Ce pont la démodule (GMSK différentiel)
#      et la décode (gr-gsm : bcch_ccch_demapper + control_channels_decoder) vers
#      GSMTAP 127.0.0.1:$CALYPSO_SHUNT_GSMTAP_PORT (4730). Le listener du shunt
#      appelle alors feed_si -> a_cd : le mobile campe sur le VRAI SI démodulé.
#
#  PRÉREQUIS
#      - un BTS qui émet (d'où MOD_DEPS=bts) ;
#      - le script de pont ET un python avec gnuradio+gr-gsm.
#
#  CRITÈRE DE SUCCÈS
#      Le processus est vivant après son initialisation gnuradio ET il a écrit
#      quelque chose dans son journal. Un import gnuradio raté tue le processus
#      dans la seconde : c'est exactement ce que le legacy ne voyait pas (il
#      lançait dans une fenêtre tmux et n'a jamais vérifié quoi que ce soit).
#
#  JOURNAL
#      $LOG_DIR/demod_bridge.log
#
#  POURQUOI mod_skip ET NON mod_fail QUAND LE SCRIPT MANQUE
#      qemu_bcch_grgsm.py n'existe ni dans ce dépôt ni sur cette machine
#      (vérifié : ${GSM_ROOT}/qemu_bcch_grgsm.py absent). Un chemin non installé
#      n'est pas une panne de la pile : on l'annonce et on continue, comme le
#      legacy le faisait avec son WARN L2036 — mais visiblement, cette fois.
# -----------------------------------------------------------------------------
MOD_REGISTER demod-bridge "Pont de démodulation gr-gsm"
MOD_REQUIRED[demod-bridge]=0
MOD_DEPS[demod-bridge]="bts"
MOD_PROFILES[demod-bridge]="calypso hybrid"
MOD_TIMEOUT[demod-bridge]=30
# Reprise exacte des trois conditions de L2021-2023.
MOD_ENABLED_IF[demod-bridge]='{ [ "${CALYPSO_DSP_SHUNT:-0}" = 1 ] || [ "${CALYPSO_FORCE_DEMOD_BRIDGE:-0}" = 1 ]; } && [ "${CALYPSO_SKIP_BTS:-0}" != 1 ] && { [ "${CALYPSO_SKIP_DEMOD_BRIDGE:-1}" != 1 ] || [ "${CALYPSO_FORCE_DEMOD_BRIDGE:-0}" = 1 ]; }'

# Chemins et réglages propres au module — idiome `:=` pour rester surchargeable
# depuis la ligne de commande, et pour ne rien poser dans environnement/.
# Le pont de demodulation est un script du depot RESEAU (gr-gsm), range dans
# tools/. On le cherche la, puis aux emplacements historiques.
if [ -r "${NITB_ROOT:-}/tools/qemu_bcch_grgsm.py" ]; then
    : "${CALYPSO_DEMOD_BRIDGE:=${NITB_ROOT}/tools/qemu_bcch_grgsm.py}"
elif [ -r "${NITB_ROOT:-}/qemu_bcch_grgsm.py" ]; then
    : "${CALYPSO_DEMOD_BRIDGE:=${NITB_ROOT}/qemu_bcch_grgsm.py}"
else
    : "${CALYPSO_DEMOD_BRIDGE:=${QEMU_TREE}/opt-gsm-scripts/qemu_bcch_grgsm.py}"
fi
: "${CALYPSO_BRIDGE_PYTHON:=/root/.env/bin/python3}"

# ATTENTION : environnement/bsp.env et shunt.env posent CALYPSO_IQ_TEE_PORT et
# CALYPSO_SHUNT_GSMTAP_PORT à la chaîne VIDE. La variable est donc *définie* :
# `:=` ne la remplacerait pas. On résout la valeur au point d'usage avec `:-`.
_demod_iq_port()     { printf '%s' "${CALYPSO_IQ_TEE_PORT:-6703}"; }
_demod_gsmtap_port() { printf '%s' "${CALYPSO_SHUNT_GSMTAP_PORT:-4730}"; }
_demod_log()         { printf '%s' "${LOG_DIR}/demod_bridge.log"; }
_demod_pat()         { printf '%s' "qemu_bcch_grgsm"; }

mod_demod_bridge_check() {
    if [ ! -f "$CALYPSO_DEMOD_BRIDGE" ]; then
        mod_hint "installez le script, ou posez CALYPSO_DEMOD_BRIDGE=/chemin/vers/qemu_bcch_grgsm.py"
        mod_skip "script de pont absent : $CALYPSO_DEMOD_BRIDGE — pas de démod native"
        return $MOD_RC_SKIP
    fi
    if [ ! -x "$CALYPSO_BRIDGE_PYTHON" ] && ! command -v python3 >/dev/null 2>&1; then
        mod_fail "aucun interpréteur python : ni $CALYPSO_BRIDGE_PYTHON ni python3"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_demod_bridge_status() { have_proc "$(_demod_pat)"; }

mod_demod_bridge_start() {
    local py="$CALYPSO_BRIDGE_PYTHON"
    [ -x "$py" ] || py=python3
    local log; log="$(_demod_log)"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    : > "$log"
    mod_say "pont     : $CALYPSO_DEMOD_BRIDGE (python=$py)"
    mod_say "chaîne   : I/Q $(_demod_iq_port) -> GSMTAP $(_demod_gsmtap_port) -> feed_si -> a_cd"

    IQ_TEE_PORT="$(_demod_iq_port)" \
    GSMTAP_PORT="$(_demod_gsmtap_port)" \
    ARFCN="${CALYPSO_CCCH_ARFCN:-514}" \
    BIT_SIGN="${BIT_SIGN:-1}" \
        "$py" -u "$CALYPSO_DEMOD_BRIDGE" >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR}/demod-bridge.pid"
    mod_ok
}

# BARRIÈRE — le legacy n'en avait AUCUNE (L2029-2033 : new-window + send-keys,
# puis on passait à la suite). Deux critères observables :
#   1. le processus vit encore (un `import grgsm` manquant le tue aussitôt) ;
#   2. il a produit au moins une ligne — preuve qu'il a dépassé ses imports.
mod_demod_bridge_wait() {
    local pid log; pid="$(cat "${RUN_DIR}/demod-bridge.pid" 2>/dev/null || echo 0)"
    log="$(_demod_log)"

    if ! wait_until "${MOD_TIMEOUT[demod-bridge]}" "sortie du pont gr-gsm" test -s "$log"; then
        if ! kill -0 "$pid" 2>/dev/null; then
            mod_hint "lisez $log : gnuradio/gr-gsm manquant dans $CALYPSO_BRIDGE_PYTHON est la cause habituelle"
            mod_fail "le pont gr-gsm s'est arrêté immédiatement"
        else
            mod_hint "vérifiez que le tee I/Q sort bien sur UDP $(_demod_iq_port) (CALYPSO_IQ_TEE_PORT)"
            mod_fail "le pont gr-gsm tourne mais n'écrit rien"
        fi
        return $MOD_RC_FAIL
    fi
    kill -0 "$pid" 2>/dev/null || { mod_hint "lisez $log"
                                    mod_fail "le pont gr-gsm a démarré puis s'est arrêté"
                                    return $MOD_RC_FAIL; }
    mod_ok
}

mod_demod_bridge_stop() {
    local pid; pid="$(cat "${RUN_DIR}/demod-bridge.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_demod_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/demod-bridge.pid"
    return 0
}
