# =============================================================================
#  21-abonnes-hlr — provisionnement des abonnés dans le HLR
# =============================================================================
#  RÔLE      crée les IMSI/MSISDN/Ki que les mobiles présenteront. Ce n'est pas
#            un service mais une ÉTAPE du cœur : sans elle, le rattachement est
#            rejeté (« IMSI unknown in HLR ») et l'on croit à une panne radio.
#            Reprend la dérivation exacte de feed_hlr (start-direct.sh) :
#              IMSI   = MCC MNC %04d(op) %06d(ms)
#              MSISDN = op * 10000 + ms
#              Ki     = 00112233445566778899aabbccdd %02x(ms) %02x(op)
#  PRÉREQUIS HLR prêt (VTY joignable) ; socat ou telnet pour parler au VTY.
#  SUCCÈS    le DERNIER abonné de la série est réellement relu depuis le HLR —
#            par la base sqlite en lecture seule si elle est accessible, sinon
#            par « subscriber imsi <imsi> show » sur le VTY. Écrire n'est pas
#            réussir : la barrière relit.
#  JOURNAL   $LOGDIR/mod/abonnes-hlr.log
#
#  IDEMPOTENT : « subscriber create » sur un IMSI existant est sans effet
#  destructeur ; mod_*_status saute l'étape si le dernier abonné est déjà là.
#  L'arrêt ne supprime AUCUN abonné (ce serait détruire l'état du réseau).
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER abonnes-hlr "Cœur — abonnés provisionnés dans le HLR"
MOD_REQUIRED[abonnes-hlr]=0
MOD_DEPS[abonnes-hlr]="hlr"
MOD_PROFILES[abonnes-hlr]="calypso faketrx hybrid core"
MOD_TIMEOUT[abonnes-hlr]=30
MOD_ENABLED_IF[abonnes-hlr]='[ "${NO_OSMO_START:-0}" != 1 ]'

: "${HLR_VTY_PORT:=4258}"
: "${HLR_DB:=/var/lib/osmocom/hlr.db}"
: "${OPERATOR_ID:=1}"
: "${N_MS:=1}"

_abo_mcc() { printf '%s\n' "${MCC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*network country code[[:space:]]' 4 001)}"; }
_abo_mnc() { printf '%s\n' "${MNC:-$(core_cfg_field "$(core_cfg osmo-msc)" '^[[:space:]]*mobile network code[[:space:]]' 4 01)}"; }

_abo_imsi() { printf '%s%s%04d%06d\n' "$(_abo_mcc)" "$(_abo_mnc)" "$OPERATOR_ID" "$1"; }
_abo_ki()   { printf '00112233445566778899aabbccdd%02x%02x\n' "$1" "$OPERATOR_ID"; }

# Relecture de l'abonné : base sqlite en lecture seule d'abord (fiable et
# instantanée), VTY en repli si sqlite est indisponible.
_abo_present() {
    local imsi="$1" n=""
    if [ -r "$HLR_DB" ] && command -v sqlite3 >/dev/null 2>&1; then
        n="$(sqlite3 -readonly "$HLR_DB" "select count(*) from subscriber where imsi='$imsi';" 2>/dev/null)"
        case "$n" in ''|*[!0-9]*) n="" ;; esac
        if [ -n "$n" ]; then [ "$n" -ge 1 ]; return $?; fi
    fi
    core_vty_ask "$HLR_VTY_PORT" "enable" "subscriber imsi $imsi show" 2>/dev/null | grep -q "IMSI: $imsi"
}

mod_abonnes_hlr_check() {
    case "$N_MS" in ''|*[!0-9]*) mod_fail "N_MS invalide : « $N_MS »"; return $MOD_RC_FAIL ;; esac
    [ "$N_MS" -ge 1 ] || { mod_skip "N_MS=$N_MS : aucun abonné à créer"; return $MOD_RC_SKIP; }
    core_vty_listen "$HLR_VTY_PORT" || {
        mod_hint "le HLR doit être prêt : ./run.sh --only hlr"
        mod_fail "VTY HLR ($HLR_VTY_PORT) injoignable"; return $MOD_RC_FAIL; }
    command -v socat >/dev/null 2>&1 || command -v telnet >/dev/null 2>&1 || {
        mod_hint "installez socat (ou telnet) : le provisionnement passe par le VTY"
        mod_fail "ni socat ni telnet — impossible d'écrire dans le HLR"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_abonnes_hlr_status() { _abo_present "$(_abo_imsi "$N_MS")"; }

mod_abonnes_hlr_start() {
    local m cmds=() imsi msisdn
    cmds+=("enable")
    for m in $(seq 1 "$N_MS"); do
        imsi="$(_abo_imsi "$m")"
        msisdn=$(( OPERATOR_ID * 10000 + m ))
        cmds+=("subscriber imsi $imsi create")
        cmds+=("subscriber imsi $imsi update msisdn $msisdn")
        cmds+=("subscriber imsi $imsi update aud2g comp128v1 ki $(_abo_ki "$m")")
    done
    cmds+=("end")
    mod_say "provisionnement de $N_MS abonné(s), opérateur $OPERATOR_ID, PLMN $(_abo_mcc)-$(_abo_mnc)"
    core_vty_ask "$HLR_VTY_PORT" "${cmds[@]}" || {
        mod_hint "vérifiez le VTY : socat STDIO TCP:127.0.0.1:$HLR_VTY_PORT,crlf"
        mod_fail "le dialogue VTY avec le HLR n'a rien retourné"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — le VTY accepte des commandes invalides sans broncher : la seule
# preuve est la relecture de l'abonné créé en dernier.
mod_abonnes_hlr_wait() {
    local imsi; imsi="$(_abo_imsi "$N_MS")"
    if ! wait_until "${MOD_TIMEOUT[abonnes-hlr]}" "abonné $imsi présent dans le HLR" _abo_present "$imsi"; then
        mod_hint "essayez à la main : subscriber imsi $imsi show (VTY $HLR_VTY_PORT) ; base = $HLR_DB"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# Aucun arrêt : les abonnés sont de l'état persistant, pas un processus.
mod_abonnes_hlr_stop() { return 0; }
