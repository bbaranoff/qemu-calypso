# =============================================================================
#  61-osmo-trx — transceiver réel (modem GMSK), alternative à fake_trx
# =============================================================================
#
#  RÔLE
#    Le vrai modem, celui qui module et démodule au lieu de recopier des bursts.
#    Il remplace fake_trx à la même place dans la chaîne, et sert le même TRXD
#    au BTS :
#
#        <source I/Q> --> osmo-trx --TRXD $TRX_BASE_PORT--> osmo-bts-trx
#
#    Source I/Q selon la variante du binaire : mémoire partagée pour
#    osmo-trx-ipc (via calypso-ipc-device), carte SDR pour osmo-trx-uhd.
#
#  PÉRIMÈTRE — NE PAS CONFONDRE AVEC 56-trx-ipc
#    56-trx-ipc.sh est l'osmo-trx-ipc de la chaîne de l'émulateur (profils
#    calypso/hybrid), avec sa dépendance à ipc-device. Le module ci-dessous est
#    le transceiver du profil « faketrx » : il n'est activé que si l'on demande
#    explicitement RADIO_TRANSCEIVER=osmo-trx, ce qui désactive alors fake-trx.
#    Les deux ne peuvent pas tourner ensemble — même plage de ports TRXD.
#
#  PRÉREQUIS — DÉPENDANCE CROISÉE, LE PIÈGE DE LA VARIANTE IPC
#    osmo-trx-ipc se CONNECTE au socket maître ($IPC_MSOCK_PATH) créé par
#    calypso-ipc-device. Si le device n'est pas là, le handshake « greeting »
#    échoue et osmo-trx-ipc sort immédiatement — sans que rien en aval ne le
#    signale. Le check l'exige donc explicitement, mais SEULEMENT pour cette
#    variante : une variante SDR n'a pas de socket maître.
#
#  CRITÈRE DE SUCCÈS (barrière)
#    Le processus vit ET a ouvert ses sockets TRXD. Le premier point à lui seul
#    ne suffit pas — un échec de greeting laisse une trace, pas un service.
#
#  JOURNAL     $LOG_DIR/osmo-trx.log
#
#  ORDRE       après la source I/Q (calypso-ipc-device pour la variante IPC),
#              avant osmo-bts-trx.
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER osmo-trx "Transceiver osmo-trx (IPC)"
MOD_REQUIRED[osmo-trx]=1
MOD_PROFILES[osmo-trx]="faketrx"
MOD_TIMEOUT[osmo-trx]=30

# Socket maître du device, exigé par la seule variante IPC (voir le check).
: "${IPC_MSOCK_PATH:=/tmp/ipc_sock0}"
MOD_ENABLED_IF[osmo-trx]='[ "${PHY_MODE:-faketrx}" != "virtphy" ] && [ "${RADIO_TRANSCEIVER:-fake}" = "osmo-trx" ]'

: "${OSMO_TRX_BIN:=osmo-trx-ipc}"
: "${OSMO_TRX_CFG:=${QEMU_CFGS:-${QEMU_TREE}/cfgs}/osmo-trx-ipc.cfg}"
MOD_LOG[osmo-trx]="${LOG_DIR}/osmo-trx.log"

mod_osmo_trx_check() {
    command -v "$OSMO_TRX_BIN" >/dev/null 2>&1 || [ -x "$OSMO_TRX_BIN" ] || {
        mod_fail "binaire introuvable : $OSMO_TRX_BIN"
        mod_hint "installez osmo-trx, ou posez OSMO_TRX_BIN=/chemin/vers/osmo-trx-ipc"
        return $MOD_RC_FAIL; }
    [ -r "$OSMO_TRX_CFG" ] || {
        mod_fail "configuration illisible : $OSMO_TRX_CFG"
        mod_hint "posez OSMO_TRX_CFG, ou copiez cfgs/osmo-trx-ipc.cfg"
        return $MOD_RC_FAIL; }
    # Exigence propre à la variante IPC : sans socket maître, le greeting
    # échoue et le processus sort — autant le dire ici plutôt qu'au timeout.
    case "$OSMO_TRX_BIN" in
        *ipc*) [ -S "$IPC_MSOCK_PATH" ] || {
                   mod_fail "socket maître IPC absent : $IPC_MSOCK_PATH"
                   mod_hint "démarrez calypso-ipc-device d'abord (il crée ce socket ; sans lui le greeting échoue)"
                   return $MOD_RC_FAIL; } ;;
    esac

    # Cohérence conf <-> plan d'adressage : la valeur du fichier fait foi, elle
    # est ce que le BTS devra viser. On la relit ici pour que la barrière et le
    # module BTS parlent du même port.
    local cfg_base; cfg_base="$(radio_cfg_val "$OSMO_TRX_CFG" "base-port")"
    [ -n "$cfg_base" ] && TRX_BASE_PORT="$cfg_base"

    if radio_foreign_holder "$TRX_BASE_PORT" "$OSMO_TRX_BIN"; then
        mod_fail "port TRXD $TRX_BASE_PORT déjà tenu (fake_trx ? autre osmo-trx ?)"
        mod_hint "un seul transceiver à la fois : ss -uanp | grep :$TRX_BASE_PORT"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_osmo_trx_status() { have_proc "$OSMO_TRX_BIN"; }

mod_osmo_trx_start() {
    radio_dirs
    local log; log="$(radio_log osmo-trx)"
    mod_say "commande : $OSMO_TRX_BIN -C $OSMO_TRX_CFG"
    case "$OSMO_TRX_BIN" in *ipc*) mod_say "socket maître : $IPC_MSOCK_PATH" ;; esac
    mod_say "journal  : $log"
    stdbuf -oL -eL "$OSMO_TRX_BIN" -C "$OSMO_TRX_CFG" >>"$log" 2>&1 &
    radio_save_pid osmo-trx $!
    mod_ok
}

# BARRIÈRE — un greeting raté ne laisse qu'un journal, pas un service.
mod_osmo_trx_wait() {
    local log; log="$(radio_log osmo-trx)"
    local cfg_base; cfg_base="$(radio_cfg_val "$OSMO_TRX_CFG" "base-port")"
    [ -n "$cfg_base" ] && TRX_BASE_PORT="$cfg_base"

    wait_until "${MOD_TIMEOUT[osmo-trx]}" "sockets TRXD ($TRX_BASE_PORT..+2)" \
        radio_udp_range "$TRX_BASE_PORT" || {
            mod_hint "fin de $log : « greeting » sans réponse = calypso-ipc-device absent ou muet"
            return $MOD_RC_FAIL; }

    radio_alive osmo-trx || {
        mod_fail "osmo-trx s'est arrêté après avoir ouvert ses sockets"
        mod_hint "fin de $log"
        return $MOD_RC_FAIL; }

    # Renseignement, pas critère : la VTY est un confort de diagnostic, son
    # absence ne condamne pas un transceiver qui sert déjà du TRXD.
    have_port "$OSMO_TRX_VTY_PORT" \
        && mod_say "VTY disponible sur 127.0.0.1:$OSMO_TRX_VTY_PORT" \
        || mod_say "VTY $OSMO_TRX_VTY_PORT non ouverte (sans conséquence sur le TRXD)"
    mod_ok
}

mod_osmo_trx_stop() { radio_kill osmo-trx "$OSMO_TRX_BIN"; }
