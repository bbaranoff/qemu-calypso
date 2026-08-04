# =============================================================================
#  57-bridge-py — pont Python historique (alternative à osmo-trx-ipc)
# =============================================================================
#
#  RÔLE        Pont Python pur UDP 5700-5702 ⇄ UDP 6702, compteur de FN cadencé
#              à l'horloge murale, GMSK sercomm soft:I/Q en ligne
#              (BRIDGE_BSP_IQ=1). Legacy : run.sh.legacy L1982-2000.
#
#  MUTUELLEMENT EXCLUSIF avec la chaîne ipc-device + trx-ipc : les deux
#  voudraient binder les MÊMES ports TRXD et parler au MÊME UDP 6702 de QEMU.
#  Le second arrivé échoue au bind — ou pire, les deux se partagent les
#  datagrammes et la BTS reçoit un flux incohérent. La règle est posée par le
#  module `coherence` (bloc A) ; on la revérifie ici, parce qu'un module ne doit
#  jamais faire confiance à un module en amont pour sa propre sûreté.
#
#  DÉSACTIVÉ PAR DÉFAUT (CALYPSO_SKIP_BRIDGE_PY=1) — mode historique.
#
#  PRÉREQUIS   QEMU démarré ; bridge.py présent.
#  SUCCÈS      Le processus est vivant après démarrage.
#  JOURNAL     $BRIDGE_LOG (défaut $LOG_DIR/bridge.py.log)
#
#  ÉTAT SUR CETTE MACHINE : bridge.py N'EST PAS dans le dépôt cible (il vivait
#  dans ${QEMU_TREE}, hors périmètre). Le module rend donc mod_skip avec
#  la raison — jamais mod_fail : ce n'est pas une installation cassée, c'est un
#  composant qui n'a pas été porté.
# -----------------------------------------------------------------------------

MOD_REGISTER bridge-py "Pont Python (mode historique)"
MOD_REQUIRED[bridge-py]=0
MOD_DEPS[bridge-py]="qemu"
MOD_PROFILES[bridge-py]="calypso hybrid"
MOD_TIMEOUT[bridge-py]=15
MOD_ENABLED_IF[bridge-py]='[ "${CALYPSO_SKIP_BRIDGE_PY:-1}" != "1" ]'

: "${BRIDGE_PY:=${QEMU_TREE:-${QEMU_TREE}}/bridge.py}"
: "${BRIDGE_LOG:=${LOG_DIR:-/root/calypso/logs}/bridge.py.log}"
: "${BRIDGE_BSP_IQ:=1}"
_BRIDGE_PYTHON=""       # résolu par mod_bridge_py_check (lecture seule)

mod_bridge_py_check() {
    # Mutex, avant tout le reste : mieux vaut ne rien lancer que de lancer deux
    # producteurs pour le même flux.
    if [ "${CALYPSO_SKIP_IPC_DEVICE:-0}" != "1" ] || [ "${CALYPSO_SKIP_TRX_IPC:-0}" != "1" ]; then
        mod_hint "choisissez UNE chaîne : soit CALYPSO_SKIP_BRIDGE_PY=1 (osmo-trx-ipc), soit CALYPSO_SKIP_IPC_DEVICE=1 CALYPSO_SKIP_TRX_IPC=1 (bridge.py)"
        mod_fail "bridge.py et la chaîne ipc-device/osmo-trx-ipc sont actifs en même temps : ils se disputent les ports TRXD et l'UDP 6702"
        return $MOD_RC_FAIL
    fi
    [ -r "$BRIDGE_PY" ] || {
        mod_skip "bridge.py non porté dans ce dépôt ($BRIDGE_PY)"
        return $MOD_RC_SKIP
    }
    # Le venv gnuradio du legacy, sinon le python du système.
    _BRIDGE_PYTHON="${CALYPSO_BRIDGE_PYTHON:-/root/.env/bin/python3}"
    [ -x "$_BRIDGE_PYTHON" ] || _BRIDGE_PYTHON="$(command -v python3 2>/dev/null)"
    [ -n "$_BRIDGE_PYTHON" ] || {
        mod_fail "aucun interpréteur python3 (ni $CALYPSO_BRIDGE_PYTHON ni dans le PATH)"
        return $MOD_RC_FAIL
    }
    mod_say "python=$_BRIDGE_PYTHON script=$BRIDGE_PY"
    mod_ok
}

mod_bridge_py_status() { have_proc "bridge\.py"; }

mod_bridge_py_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" "$(dirname "$BRIDGE_LOG")" 2>/dev/null || true
    : > "$BRIDGE_LOG" 2>/dev/null || true
    BRIDGE_BSP_IQ="$BRIDGE_BSP_IQ" \
    CALYPSO_DOPPLER_HZ="${CALYPSO_DOPPLER_HZ:-0}" \
    CALYPSO_BURST_PRINT="${CALYPSO_BURST_PRINT:-0}" \
        "$_BRIDGE_PYTHON" -u "$BRIDGE_PY" >>"$BRIDGE_LOG" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/bridge-py.pid"
    mod_ok
}

# BARRIÈRE — le legacy se contentait d'un « bridge.py lancé » imprimé juste
# après le send-keys tmux, sans rien vérifier (L1996). Ici : le processus
# doit être encore là. HYPOTHÈSE ASSUMÉE : faute de bridge.py sur cette
# machine, on ne peut pas figer de marqueur de journal (ni de port bindé
# vérifié) ; la survie du processus est le seul critère qu'on puisse garantir
# sans inventer une chaîne de caractères. À durcir dès que le script est porté.
mod_bridge_py_wait() {
    wait_until "${MOD_TIMEOUT[bridge-py]}" "bridge.py" have_proc "bridge\.py" || {
        modb_tail "$BRIDGE_LOG" 20
        mod_hint "lancez-le à la main pour voir l'import manquant : $_BRIDGE_PYTHON -u $BRIDGE_PY"
        mod_fail "bridge.py s'est arrêté immédiatement (dépendance gnuradio/numpy ?)"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_bridge_py_stop() {
    local pidf="${RUN_DIR:-/tmp/calypso}/bridge-py.pid" pid
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "bridge\.py" 2>/dev/null
    rm -f "$pidf"
    return 0
}
