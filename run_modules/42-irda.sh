# =============================================================================
#  42-irda — pair IrDA actif (rôle PRIMARY) sur l'UART secondaire du firmware
# =============================================================================
#
#  RÔLE        Lance tools/irda_peer.py en PRIMARY sur le PTY `serial1`.
#              Legacy : run.sh.legacy L1871-1919.
#
#  POURQUOI UN PAIR ACTIF  Le firmware compal_e88 est un IrDA SECONDARY
#              (irda_init → IrLAP NDM) : il ne parle QUE s'il reçoit une
#              découverte XID puis un SNRM. Une capture passive ne peut donc
#              rien produire — c'est pour cela que l'ancien log restait vide.
#
#  DÉSACTIVÉ PAR DÉFAUT (CALYPSO_IRDA_PEER=0). Un lien IrDA vivant génère des
#  IRQ RX en continu qui VOLENT le timing TDMA de la L1 GSM : bursts downlink
#  ratés → « no cell info » → pas de Location Update. À n'activer que pour
#  déboguer l'IrDA, en sachant que le GSM est alors perturbé.
#
#  PRÉREQUIS   PTY serial1 résolu (module pty), python3, irda_peer.py présent.
#  SUCCÈS      Le processus irda_peer.py est vivant et le lien PTY est en place.
#  JOURNAL     $FW_IRDA_LOG (flux IrCOMM reçu du firmware, via irda_puts) et
#              $LOG_DIR/irda_peer.stderr.log (trace du pair).
#
#  LECTURE DU JOURNAL : si $FW_IRDA_LOG ne contient que des SNRM répétés sans
#  UA, c'est le firmware qui ne répond pas (irphy.c/irlap.c) — pas le lancement.
# -----------------------------------------------------------------------------

MOD_REGISTER irda "Pair IrDA (debug UART secondaire)"
MOD_REQUIRED[irda]=0
MOD_DEPS[irda]="pty"
MOD_PROFILES[irda]="calypso hybrid"
MOD_TIMEOUT[irda]=10
MOD_ENABLED_IF[irda]='[ "${CALYPSO_IRDA_PEER:-0}" = "1" ]'

: "${IRDA_PEER:=${OSMOCOM_BB:-${GSM_ROOT}/osmocom-bb}/src/target/firmware/tools/irda_peer.py}"
: "${IRDA_PTY_LINK:=/tmp/irda.pty.link}"
: "${FW_IRDA_LOG:=${LOG_DIR:-/root/calypso/logs}/fw-irda.log}"

mod_irda_check() {
    # Script absent = on IGNORE, on n'échoue pas : c'est un outil de debug
    # optionnel, son absence n'est pas une panne de la pile.
    [ -r "$IRDA_PEER" ] || {
        mod_skip "irda_peer.py absent ($IRDA_PEER)"
        return $MOD_RC_SKIP
    }
    command -v python3 >/dev/null 2>&1 || {
        mod_hint "apt-get install python3 — ou laissez CALYPSO_IRDA_PEER=0"
        mod_fail "python3 introuvable"
        return $MOD_RC_FAIL
    }
    [ -n "${CALYPSO_IRDA_PTY:-}" ] && [ -c "$CALYPSO_IRDA_PTY" ] || {
        mod_hint "QEMU doit être lancé avec DEUX « -serial pty » ; vérifiez « info chardev » sur le moniteur"
        mod_fail "PTY serial1 non résolu : le module pty n'a vu qu'un seul UART"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_irda_status() { have_proc "irda_peer.py"; }

mod_irda_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" 2>/dev/null || true
    # Lien symbolique stable : le numéro de PTY change à chaque run, les outils
    # tiers (et le pair lui-même) suivent /tmp/irda.pty.link.
    ln -sf "$CALYPSO_IRDA_PTY" "$IRDA_PTY_LINK"
    : > "$FW_IRDA_LOG" 2>/dev/null || true

    IRDA_ROLE=primary IRDA_PTY="$IRDA_PTY_LINK" \
        python3 -u "$IRDA_PEER" \
        >>"$FW_IRDA_LOG" 2>>"${LOG_DIR:-/root/calypso/logs}/irda_peer.stderr.log" </dev/null &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/irda_peer.pid"
    mod_ok
}

# BARRIÈRE — remplace le `sleep 0.3` × 20 du legacy (L1909) et le `sleep 0.5`
# purement décoratif de la ligne de commande tmux (L1905) : on sonde le
# PROCESSUS, pas une durée.
mod_irda_wait() {
    wait_until "${MOD_TIMEOUT[irda]}" "pair irda_peer.py" have_proc "irda_peer.py" || {
        modb_tail "${LOG_DIR:-/root/calypso/logs}/irda_peer.stderr.log" 15
        mod_hint "python3 -u $IRDA_PEER à la main pour voir l'erreur d'import"
        mod_fail "irda_peer.py s'est arrêté immédiatement"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_irda_stop() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/irda_peer.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "irda_peer.py" 2>/dev/null
    rm -f "${RUN_DIR:-/tmp/calypso}/irda_peer.pid" "$IRDA_PTY_LINK"
    return 0
}
