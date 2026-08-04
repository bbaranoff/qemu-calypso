# =============================================================================
#  56-trx-ipc — osmo-trx-ipc : le transceiver vu par la BTS
# =============================================================================
#
#  RÔLE        Se connecte au socket maître de calypso-ipc-device et expose
#              l'interface TRXD (UDP 5700-5702) vers osmo-bts-trx.
#              Legacy : run.sh.legacy L1969-1980.
#
#  CE QUE LE LEGACY NE FAISAIT PAS  Rien. Aucune vérification, pas même un
#  WARN : « si calypso-ipc-device n'est pas démarré, osmo-trx-ipc exit
#  immédiatement (greeting_req sans réponse) — c'est OK en transition, sa
#  fenêtre tmux reste » (commentaire L1971-1973). Autrement dit, la panne la
#  plus fréquente de la chaîne radio était invisible depuis le lancement.
#
#  PRÉREQUIS   calypso-ipc-device prêt (socket maître), binaire + cfg présents.
#  SUCCÈS      Processus vivant ET au moins un port UDP du bloc TRXD bindé.
#              POURQUOI CE CRITÈRE : le greeting échoué tue le processus en
#              une fraction de seconde ; les sockets TRXD, elles, ne sont
#              ouvertes qu'une fois le device accepté. « Vivant » seul ne
#              prouverait rien (il peut mourir juste après), « port ouvert »
#              seul non plus (un résidu du run précédent) — les deux ensemble
#              signifient : ce processus-ci sert réellement la BTS.
#
#  CONVENTION DE PORTS : osmo-bts-trx parle en local sur 5800+ et vise le
#  transceiver sur 5700+ (base-port remote par défaut). Le transceiver binde
#  donc 5700 (clock), 5701 (contrôle canal 0), 5702 (données canal 0). On
#  accepte l'un des trois : la répartition exacte dépend de la version.
#
#  JOURNAL     $OSMO_TRX_IPC_LOG (défaut $LOG_DIR/osmo-trx-ipc.log)
# -----------------------------------------------------------------------------

MOD_REGISTER trx-ipc "Transceiver osmo-trx-ipc"
MOD_REQUIRED[trx-ipc]=0
MOD_DEPS[trx-ipc]="ipc-device"
MOD_PROFILES[trx-ipc]="calypso hybrid"
MOD_TIMEOUT[trx-ipc]=20
MOD_ENABLED_IF[trx-ipc]='[ "${CALYPSO_SKIP_TRX_IPC:-0}" != "1" ]'

: "${OSMO_TRX_IPC:=}"
: "${OSMO_TRX_IPC_CFG:=}"
: "${OSMO_TRX_IPC_LOG:=${LOG_DIR:-/root/calypso/logs}/osmo-trx-ipc.log}"
: "${TRX_BASE_PORT:=5700}"

_trxipc_trxd_up() {
    modb_have_udp "$TRX_BASE_PORT" \
        || modb_have_udp $(( TRX_BASE_PORT + 1 )) \
        || modb_have_udp $(( TRX_BASE_PORT + 2 ))
}

mod_trx_ipc_check() {
    # Résolution du binaire — lecture seule, aucun effet de bord.
    if [ -z "$OSMO_TRX_IPC" ] || [ ! -x "$OSMO_TRX_IPC" ]; then
        if command -v osmo-trx-ipc >/dev/null 2>&1; then
            OSMO_TRX_IPC="$(command -v osmo-trx-ipc)"
        elif [ -x "${OSMO_TRX:-${GSM_ROOT}/osmo-trx}/Transceiver52M/osmo-trx-ipc" ]; then
            OSMO_TRX_IPC="${OSMO_TRX:-${GSM_ROOT}/osmo-trx}/Transceiver52M/osmo-trx-ipc"
        fi
    fi
    [ -n "$OSMO_TRX_IPC" ] && [ -x "$OSMO_TRX_IPC" ] || {
        mod_hint "installez osmo-trx (cible ipc) ou posez OSMO_TRX_IPC=<chemin> ; CALYPSO_SKIP_TRX_IPC=1 pour vous en passer"
        mod_fail "osmo-trx-ipc introuvable"
        return $MOD_RC_FAIL
    }

    # Résolution de la cfg. La cfg VERSIONNÉE du dépôt d'abord : celle de
    # /etc/osmocom déclarait chan 0 ET chan 1, alors que le Calypso n'a qu'un
    # canal — osmo-trx-ipc sort alors sur « DDEV chan num mismatch » (L1058-1062).
    if [ -z "$OSMO_TRX_IPC_CFG" ] || [ ! -r "$OSMO_TRX_IPC_CFG" ]; then
        local c
        for c in "${QEMU_CFGS:-${QEMU_TREE:-${QEMU_TREE}}/cfgs}/osmo-trx-ipc.cfg" \
                 "${OSMOCOM_CFG:-/etc/osmocom}/osmo-trx-ipc.cfg"; do
            [ -r "$c" ] && { OSMO_TRX_IPC_CFG="$c"; break; }
        done
    fi
    [ -n "$OSMO_TRX_IPC_CFG" ] && [ -r "$OSMO_TRX_IPC_CFG" ] || {
        mod_hint "attendu : cfgs/osmo-trx-ipc.cfg (un seul « chan 0 ») ; posez OSMO_TRX_IPC_CFG=<chemin>"
        mod_fail "configuration d'osmo-trx-ipc introuvable"
        return $MOD_RC_FAIL
    }

    # Le socket maître doit exister : sans lui, le greeting échoue et le
    # processus meurt aussitôt. Deux situations à ne PAS confondre :
    #   - le device est désactivé (CALYPSO_SKIP_IPC_DEVICE=1) : personne ne
    #     créera jamais ce socket, on refuse tout de suite ;
    #   - le device est actif : il vient de démarrer (c'est notre dépendance
    #     déclarée, donc sa barrière est passée) — ou bien on est en --dry-run,
    #     où il n'a été que simulé. Dans les deux cas, échouer ici serait faux :
    #     on note, et c'est la barrière qui tranchera sur du réel.
    if [ ! -S "${IPC_MSOCK_PATH:-/tmp/ipc_sock0}" ]; then
        if [ "${CALYPSO_SKIP_IPC_DEVICE:-0}" = "1" ]; then
            mod_hint "retirez CALYPSO_SKIP_IPC_DEVICE=1, ou fournissez un autre producteur du socket via IPC_MSOCK_PATH"
            mod_fail "socket maître IPC absent (${IPC_MSOCK_PATH:-/tmp/ipc_sock0}) et calypso-ipc-device est désactivé : personne ne le créera"
            return $MOD_RC_FAIL
        fi
        mod_say "socket maître ${IPC_MSOCK_PATH:-/tmp/ipc_sock0} pas encore présent — vérification reportée à la barrière"
    fi
    mod_say "binaire=$OSMO_TRX_IPC cfg=$OSMO_TRX_IPC_CFG"
    mod_ok
}

mod_trx_ipc_status() { have_proc "osmo-trx-ipc"; }

mod_trx_ipc_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" "$(dirname "$OSMO_TRX_IPC_LOG")" 2>/dev/null || true
    : > "$OSMO_TRX_IPC_LOG" 2>/dev/null || true
    stdbuf -oL -eL "$OSMO_TRX_IPC" -C "$OSMO_TRX_IPC_CFG" >>"$OSMO_TRX_IPC_LOG" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/trx-ipc.pid"
    mod_ok
}

# BARRIÈRE NEUVE — le legacy n'en avait aucune.
mod_trx_ipc_wait() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/trx-ipc.pid" 2>/dev/null || echo 0)"

    # Condition amont, vérifiée sur du réel cette fois : sans socket maître, le
    # greeting ne peut pas aboutir — autant nommer la vraie cause.
    if [ ! -S "${IPC_MSOCK_PATH:-/tmp/ipc_sock0}" ]; then
        modb_tail "$OSMO_TRX_IPC_LOG" 15
        mod_hint "lancez d'abord le device : ./run.sh --only ipc-device"
        mod_fail "socket maître IPC absent (${IPC_MSOCK_PATH:-/tmp/ipc_sock0}) : osmo-trx-ipc ne peut pas faire son greeting"
        return $MOD_RC_FAIL
    fi

    if ! wait_until "${MOD_TIMEOUT[trx-ipc]}" "ports TRXD ${TRX_BASE_PORT}-$(( TRX_BASE_PORT + 2 ))" \
            _trxipc_trxd_up; then
        modb_tail "$OSMO_TRX_IPC_LOG" 25
        if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
            mod_hint "cause la plus fréquente : greeting refusé par calypso-ipc-device (nombre de canaux, cfg à un seul « chan 0 »)"
            mod_fail "osmo-trx-ipc s'est arrêté avant d'ouvrir l'interface TRXD"
        else
            mod_hint "détail : $OSMO_TRX_IPC_LOG"
            mod_fail "aucun port TRXD ouvert après ${MOD_TIMEOUT[trx-ipc]}s"
        fi
        return $MOD_RC_FAIL
    fi

    if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
        modb_tail "$OSMO_TRX_IPC_LOG" 25
        mod_hint "un osmo-trx-ipc résiduel tient peut-être les ports : ./run.sh --stop"
        mod_fail "les ports TRXD sont ouverts mais PAS par ce processus (il est mort)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_trx_ipc_stop() {
    local pidf="${RUN_DIR:-/tmp/calypso}/trx-ipc.pid" pid
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "osmo-trx-ipc" 2>/dev/null
    rm -f "$pidf"
    return 0
}
