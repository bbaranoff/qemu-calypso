# =============================================================================
#  19-asterisk — le PBX SIP qui porte les appels
# =============================================================================
#  RÔLE      terminaison des appels : osmo-sip-connector traduit le MNCC du MSC
#            en SIP et le pousse vers Asterisk. Sans PBX, un appel MS→MS n'est
#            jamais raccordé. Optionnel : ni le rattachement ni le SMS n'en
#            dépendent.
#  PRÉREQUIS binaire asterisk ; /etc/asterisk/asterisk.conf.
#  SUCCÈS    la console de contrôle RÉPOND (« core show uptime ») — c'est le
#            seul critère qui prouve qu'Asterisk a fini de charger ses modules,
#            là où « actif » ne prouve que le fork. Plus : aucun redémarrage.
#  JOURNAL   journalctl -u asterisk ; /var/log/asterisk/
#
#  NOTE l'ancien lancement supprimait /var/lib/asterisk/astdb.sqlite3 à chaque
#  démarrage (scripts/run.sh:455). Ce module ne le fait PAS : détruire une base
#  n'est pas une étape de démarrage.
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER asterisk "Cœur — Asterisk (PBX SIP)"
MOD_REQUIRED[asterisk]=0
MOD_PROFILES[asterisk]="calypso faketrx hybrid core"
MOD_JOURNAL[asterisk]="asterisk"
MOD_TIMEOUT[asterisk]=45
MOD_ENABLED_IF[asterisk]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_VOICE:-1}" = 1 ]'

: "${ASTERISK_UNIT:=asterisk}"
: "${ASTERISK_CFG:=/etc/asterisk/asterisk.conf}"

_ast_cli() { asterisk -rx "$1" 2>/dev/null; }
_ast_ready() { _ast_cli "core show uptime" | grep -qi 'uptime'; }

mod_asterisk_check() {
    command -v asterisk >/dev/null 2>&1 || {
        mod_hint "installez asterisk, ou désactivez la voix : CORE_VOICE=0"
        mod_fail "binaire asterisk introuvable"; return $MOD_RC_FAIL; }
    [ -r "$ASTERISK_CFG" ] || {
        mod_fail "configuration illisible : $ASTERISK_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_asterisk_status() { _ast_ready; }

mod_asterisk_start() {
    core_svc_start "$ASTERISK_UNIT" "$(command -v asterisk)" -g -f -p -U asterisk \
        || { mod_fail "systemctl start $ASTERISK_UNIT a échoué"
             mod_hint "journalctl -u $ASTERISK_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — Asterisk met plusieurs secondes à charger ses modules ; la socket
# de contrôle existe avant qu'il ne réponde. On interroge donc la CLI.
mod_asterisk_wait() {
    if ! wait_until "${MOD_TIMEOUT[asterisk]}" "console Asterisk" _ast_ready; then
        mod_hint "asterisk -rx 'core show uptime' ; journalctl -u $ASTERISK_UNIT -n 40"
        return $MOD_RC_FAIL
    fi
    if core_restarted_since "$ASTERISK_UNIT"; then
        mod_hint "journalctl -u $ASTERISK_UNIT -n 50 : module ou conf en faute"
        mod_fail "Asterisk a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_asterisk_stop() {
    _ast_cli "core stop now" >/dev/null 2>&1
    core_svc_stop "$ASTERISK_UNIT" ""
}
