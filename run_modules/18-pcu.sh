# =============================================================================
#  18-pcu — OsmoPCU, le contrôleur paquet côté BTS
# =============================================================================
#  RÔLE      ordonnance les blocs RLC/MAC de la data et relie la BTS au SGSN
#            (NS/BSSGP). Il se connecte à la socket PCU **créée par la BTS**
#            (pcu-socket, /tmp/pcu_bts) : il peut donc démarrer avant elle et
#            attendre — c'est pourquoi ce module ne dépend pas de la BTS, qui
#            n'appartient pas au bloc cœur.
#  PRÉREQUIS binaire et conf osmo-pcu ; SGSN prêt pour que la data serve à
#            quelque chose (dépendance déclarée mais non bloquante : optionnel).
#  SUCCÈS    service vivant, sans redémarrage depuis le lancement, et VTY en
#            écoute. Le numéro du VTY (4240) n'est PAS déclaré dans la conf :
#            s'il n'apparaît pas, on ne conclut pas à l'échec, on se replie sur
#            la sonde de vie — et on le dit dans le journal du module.
#  JOURNAL   journalctl -u osmo-pcu    (sans systemd : $LOG_DIR/osmo-pcu.log)
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER pcu "Cœur — OsmoPCU (ordonnanceur paquet)"
MOD_REQUIRED[pcu]=0
MOD_DEPS[pcu]="sgsn"
MOD_PROFILES[pcu]="calypso faketrx hybrid core"
MOD_JOURNAL[pcu]="osmo-pcu"
MOD_TIMEOUT[pcu]=20
MOD_ENABLED_IF[pcu]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_GPRS:-1}" = 1 ]'

: "${PCU_UNIT:=osmo-pcu}"
: "${PCU_VTY_PORT:=4240}"
: "${PCU_SOCKET:=/tmp/pcu_bts}"

_pcu_cfg() { core_cfg osmo-pcu; }

mod_pcu_check() {
    core_bin osmo-pcu >/dev/null || { mod_fail "binaire osmo-pcu introuvable"; return $MOD_RC_FAIL; }
    [ -r "$(_pcu_cfg)" ] || {
        mod_hint "déployez la configuration : cp configs/osmo-pcu.cfg $OSMOCOM_CFG/"
        mod_fail "configuration illisible : $(_pcu_cfg)"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_pcu_status() { core_unit_active "$PCU_UNIT" || core_vty_listen "$PCU_VTY_PORT"; }

mod_pcu_start() {
    core_svc_start "$PCU_UNIT" "$(core_bin osmo-pcu)" -c "$(_pcu_cfg)" \
        || { mod_fail "systemctl start $PCU_UNIT a échoué"
             mod_hint "journalctl -u $PCU_UNIT -n 30"; return $MOD_RC_FAIL; }
    # Reprise de osmo-start.sh:109 : la socket appartient à la BTS et n'est pas
    # forcément accessible à l'utilisateur osmocom. Sans effet si elle n'existe
    # pas encore (cas normal quand la BTS n'est pas démarrée).
    [ -e "$PCU_SOCKET" ] && chmod 777 "$PCU_SOCKET" 2>/dev/null
    mod_ok
}

# BARRIÈRE — un PCU qui meurt sur une conf invalide est relancé toutes les 2 s
# par systemd ; c'est ce que détecte core_restarted_since.
mod_pcu_wait() {
    if ! core_alive "$PCU_UNIT"; then
        mod_hint "journalctl -u $PCU_UNIT -n 30"
        mod_fail "osmo-pcu n'est pas resté en vie après le lancement"
        return $MOD_RC_FAIL
    fi
    if ! wait_until "${MOD_TIMEOUT[pcu]}" "VTY OsmoPCU ($PCU_VTY_PORT)" core_vty_listen "$PCU_VTY_PORT"; then
        mod_say "VTY $PCU_VTY_PORT non observé — port non déclaré dans $(_pcu_cfg), repli sur la sonde de vie"
    fi
    if core_restarted_since "$PCU_UNIT"; then
        mod_hint "journalctl -u $PCU_UNIT -n 50 : conf invalide ou socket PCU inaccessible"
        mod_fail "OsmoPCU redémarre en boucle"
        return $MOD_RC_FAIL
    fi
    core_unix_listen "$PCU_SOCKET" && mod_say "socket PCU $PCU_SOCKET présente (BTS déjà démarrée)"
    mod_ok
}

mod_pcu_stop() { core_svc_stop "$PCU_UNIT" "osmo-pcu -c"; }
