# =============================================================================
#  10-stp — OsmoSTP, le point de transfert sémaphore (SS7 / M3UA)
# =============================================================================
#  RÔLE      tout le SS7 du cœur passe par lui : le MSC (PC 1.1.1) et le BSC
#            (PC 1.1.3) s'y raccordent comme ASP M3UA. Tant qu'il n'écoute pas,
#            aucun des deux ne peut se parler — d'où sa place en tête du bloc.
#  PRÉREQUIS binaire osmo-stp ; $OSMOCOM_CFG/osmo-stp.cfg ; support SCTP dans le
#            noyau (l'écoute M3UA est du SCTP, pas du TCP).
#  SUCCÈS    VTY en écoute (4239) ET socket M3UA SCTP en écoute sur le port lu
#            dans la conf (« listen m3ua <port> ») ET aucun redémarrage de
#            l'unité depuis notre lancement.
#  JOURNAL   journalctl -u osmo-stp    (sans systemd : $LOG_DIR/osmo-stp.log)
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER stp "Cœur — OsmoSTP (SS7/M3UA)"
MOD_REQUIRED[stp]=1
MOD_PROFILES[stp]="calypso faketrx hybrid core"
MOD_JOURNAL[stp]="osmo-stp"
MOD_TIMEOUT[stp]=25
# Échappatoire héritée de start-direct.sh : NO_OSMO_START=1 saute tout le cœur.
MOD_ENABLED_IF[stp]='[ "${NO_OSMO_START:-0}" != 1 ]'

: "${STP_UNIT:=osmo-stp}"
: "${STP_VTY_PORT:=4239}"

_stp_cfg()  { core_cfg osmo-stp; }
_stp_m3ua() { core_cfg_field "$(_stp_cfg)" '^[[:space:]]*listen[[:space:]]+m3ua[[:space:]]' 3 2905; }

mod_stp_check() {
    core_bin osmo-stp >/dev/null || {
        mod_hint "paquet osmo-stp absent — vérifiez l'image, ou réglez OSMOCOM_CFG/PATH"
        mod_fail "binaire osmo-stp introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_stp_cfg)" ] || {
        mod_hint "déployez la configuration : cp configs/osmo-stp.cfg $OSMOCOM_CFG/"
        mod_fail "configuration illisible : $(_stp_cfg)"; return $MOD_RC_FAIL; }
    # Sans SCTP, osmo-stp démarre puis n'écoute jamais : autant le dire ici.
    if [ ! -e /proc/net/sctp/snmp ] && ! lsmod 2>/dev/null | grep -q '^sctp'; then
        mod_hint "chargez le module côté hôte : modprobe sctp (un conteneur ne peut pas le faire)"
        mod_fail "SCTP indisponible dans ce noyau — l'écoute M3UA est impossible"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_stp_status() { core_unit_active "$STP_UNIT" || core_vty_listen "$STP_VTY_PORT"; }

mod_stp_start() {
    core_svc_start "$STP_UNIT" "$(core_bin osmo-stp)" -c "$(_stp_cfg)" \
        || { mod_fail "systemctl start $STP_UNIT a échoué"
             mod_hint "journalctl -u $STP_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — « actif » ne suffit pas : l'unité porte Restart=always, donc un STP
# qui meurt à l'init réapparaît indéfiniment sans jamais ouvrir son écoute.
mod_stp_wait() {
    local to="${MOD_TIMEOUT[stp]}" m3ua; m3ua="$(_stp_m3ua)"

    if ! wait_until "$to" "VTY OsmoSTP ($STP_VTY_PORT)" core_vty_listen "$STP_VTY_PORT"; then
        mod_hint "journalctl -u $STP_UNIT -n 30 ; port $STP_VTY_PORT déjà pris ?"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "écoute M3UA SCTP ($m3ua)" core_sctp_listen "$m3ua"; then
        mod_hint "vérifiez « listen m3ua $m3ua » dans $(_stp_cfg) et le support SCTP (ss -ln --sctp)"
        return $MOD_RC_FAIL
    fi
    if core_restarted_since "$STP_UNIT"; then
        mod_hint "journalctl -u $STP_UNIT -n 50 : le service redémarre en boucle"
        mod_fail "OsmoSTP a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_stp_stop() { core_svc_stop "$STP_UNIT" "osmo-stp -c"; }
