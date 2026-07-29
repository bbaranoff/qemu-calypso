# =============================================================================
#  14-bsc — OsmoBSC, le contrôleur de station de base
# =============================================================================
#  RÔLE      pilote les BTS (OML/RSL sur A-bis) et parle au MSC via l'interface
#            A (SCCP/M3UA). C'est lui qui écoute les BTS : tant que ses ports
#            A-bis ne sont pas ouverts, aucune BTS — ni osmo-bts-trx, ni la
#            BTS émulée du Calypso — ne peut s'attacher.
#  PRÉREQUIS binaires et conf osmo-bsc ; STP et MSC prêts ; MGW prêt (le BSC
#            ouvre lui aussi une session MGCP).
#  SUCCÈS    VTY en écoute (4242) ET les deux écoutes A-bis IPA (OML 3002,
#            RSL 3003) ET association SCTP établie depuis le port local de son
#            ASP (lien M3UA vers le STP monté) ET aucun redémarrage.
#  JOURNAL   journalctl -u osmo-bsc    (sans systemd : $LOG_DIR/osmo-bsc.log)
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER bsc "Cœur — OsmoBSC (contrôleur BTS)"
MOD_REQUIRED[bsc]=1
MOD_DEPS[bsc]="stp mgw msc"
MOD_PROFILES[bsc]="calypso faketrx hybrid core"
MOD_JOURNAL[bsc]="osmo-bsc"
MOD_TIMEOUT[bsc]=40
MOD_ENABLED_IF[bsc]='[ "${NO_OSMO_START:-0}" != 1 ]'

: "${BSC_UNIT:=osmo-bsc}"
: "${BSC_VTY_PORT:=4242}"
# Ports A-bis du pilote e1_input « ipa » : fixés par le protocole IPA, pas par
# la conf (constaté à l'exécution : 0.0.0.0:3002 et 0.0.0.0:3003).
: "${BSC_OML_PORT:=3002}"
: "${BSC_RSL_PORT:=3003}"

_bsc_cfg()      { core_cfg osmo-bsc; }
_bsc_asp_port() { core_cfg_field "$(_bsc_cfg)" '^[[:space:]]*asp[[:space:]]+.*[[:space:]]m3ua[[:space:]]*$' 4 ""; }

mod_bsc_check() {
    core_bin osmo-bsc >/dev/null || { mod_fail "binaire osmo-bsc introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_bsc_cfg)" ] || {
        mod_hint "déployez la configuration : cp configs/osmo-bsc.cfg $OSMOCOM_CFG/"
        mod_fail "configuration illisible : $(_bsc_cfg)"; return $MOD_RC_FAIL; }
    # Un marqueur de gabarit non substitué produirait un BSC qui démarre puis
    # meurt sur une ligne de conf invalide : autant le voir avant.
    if grep -qa '__[A-Z_]\+__' "$(_bsc_cfg)" 2>/dev/null; then
        mod_hint "la configuration contient encore des jetons de gabarit — régénérez-la"
        mod_fail "$(_bsc_cfg) : gabarit non substitué"; return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_bsc_status() { core_unit_active "$BSC_UNIT" || core_vty_listen "$BSC_VTY_PORT"; }

mod_bsc_start() {
    core_svc_start "$BSC_UNIT" "$(core_bin osmo-bsc)" -c "$(_bsc_cfg)" -s \
        || { mod_fail "systemctl start $BSC_UNIT a échoué"
             mod_hint "journalctl -u $BSC_UNIT -n 30"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — c'est ici que l'ancien démarrage mentait le plus : le BSC était
# lancé, la BTS lancée juste après, et personne ne vérifiait que l'écoute A-bis
# existait. Une BTS qui se connecte trop tôt boucle en « OML link down » sans
# que rien ne l'indique.
mod_bsc_wait() {
    local to="${MOD_TIMEOUT[bsc]}" asp; asp="$(_bsc_asp_port)"

    if ! wait_until "$to" "VTY OsmoBSC ($BSC_VTY_PORT)" core_vty_listen "$BSC_VTY_PORT"; then
        mod_hint "journalctl -u $BSC_UNIT -n 30"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "écoute A-bis OML ($BSC_OML_PORT)" core_tcp_listen "$BSC_OML_PORT"; then
        mod_hint "port $BSC_OML_PORT déjà pris par un autre BSC ? ss -ltn | grep $BSC_OML_PORT"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "$to" "écoute A-bis RSL ($BSC_RSL_PORT)" core_tcp_listen "$BSC_RSL_PORT"; then
        mod_hint "port $BSC_RSL_PORT déjà pris ? ss -ltn | grep $BSC_RSL_PORT"
        return $MOD_RC_FAIL
    fi
    if [ -n "$asp" ]; then
        if ! wait_until "$to" "lien M3UA vers le STP (SCTP local :$asp)" core_sctp_estab "$asp"; then
            mod_hint "STP joignable ? ss -an --sctp | grep $asp ; vérifiez « asp ... m3ua » dans $(_bsc_cfg)"
            return $MOD_RC_FAIL
        fi
    else
        mod_say "aucun ASP m3ua dans $(_bsc_cfg) — barrière SS7 non applicable"
    fi
    if core_restarted_since "$BSC_UNIT"; then
        mod_hint "journalctl -u $BSC_UNIT -n 50 : le service redémarre en boucle"
        mod_fail "OsmoBSC a redémarré depuis le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_bsc_stop() { core_svc_stop "$BSC_UNIT" "osmo-bsc -c"; }
