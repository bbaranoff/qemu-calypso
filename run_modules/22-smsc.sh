# =============================================================================
#  22-smsc — passerelle SMS (proto-smsc-daemon + relay interop)
# =============================================================================
#  RÔLE      porte les SMS : le daemon se connecte au HLR en GSUP sous le nom
#            IPA « SMSC-OP<n> » (déclaré dans osmo-hlr.cfg : « smsc entity /
#            smsc default-route ») et expose la socket d'envoi MT ; le relay
#            python écoute en TCP pour les SMS inter-opérateurs.
#            Délègue à /etc/osmocom/smsc-start.sh — on ne réimplémente pas ce
#            qui marche, on l'encadre d'une barrière.
#  PRÉREQUIS proto-smsc-daemon ; smsc-start.sh ; HLR prêt (GSUP) ;
#            /etc/osmocom/sms-routing.conf pour le relay.
#  SUCCÈS    socket d'envoi MT présente ET port TCP du relay en écoute (ce
#            dernier seulement si le script du relay est installé).
#  JOURNAL   $LOG_DIR/smsc-op<n>.log ; journal des SMS MO reçus :
#            /var/log/osmocom/mo-sms-op<n>.log  (chemin réel, différent de
#            celui qu'annonçait start-direct.sh:493).
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER smsc "Cœur — passerelle SMS (proto-SMSC)"
MOD_REQUIRED[smsc]=0
MOD_DEPS[smsc]="hlr"
MOD_PROFILES[smsc]="calypso faketrx hybrid core"
MOD_TIMEOUT[smsc]=60
MOD_ENABLED_IF[smsc]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_SMS:-1}" = 1 ]'

: "${OPERATOR_ID:=1}"
: "${SMSC_SCRIPT:=/etc/osmocom/smsc-start.sh}"
: "${SMSC_RELAY_SCRIPT:=/etc/osmocom/sms-interop-relay.py}"
: "${SMSC_RELAY_PORT:=7890}"
: "${SMSC_SENDMT_SOCKET:=/tmp/sendmt_socket}"
: "${SMSC_HLR_IP:=127.0.0.2}"

mod_smsc_check() {
    command -v proto-smsc-daemon >/dev/null 2>&1 || {
        mod_hint "sans proto-smsc-daemon les SMS passent uniquement par le VTY du MSC ; CORE_SMS=0 pour ne plus l'essayer"
        mod_skip "proto-smsc-daemon absent"; return $MOD_RC_SKIP; }
    [ -x "$SMSC_SCRIPT" ] || {
        mod_hint "déployez les scripts : cp scripts/smsc-start.sh $OSMOCOM_CFG/"
        mod_fail "script de démarrage absent : $SMSC_SCRIPT"; return $MOD_RC_FAIL; }
    core_tcp_listen 4222 "$SMSC_HLR_IP" || {
        mod_hint "le HLR doit écouter en GSUP : ./run.sh --only hlr"
        mod_fail "GSUP HLR ($SMSC_HLR_IP:4222) injoignable"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_smsc_status() { have_proc 'proto-smsc-daemon'; }

mod_smsc_start() {
    mkdir -p "$RUN_DIR" "$LOG_DIR" /var/log/osmocom 2>/dev/null || true
    # Purge ciblée d'un daemon résiduel : deux instances se disputeraient la
    # même socket MT et le même nom IPA côté HLR.
    pkill -f 'proto-smsc-daemon' 2>/dev/null
    local log="$LOG_DIR/smsc-op${OPERATOR_ID}.log"
    mod_say "journal : $log ; SMS MO reçus : /var/log/osmocom/mo-sms-op${OPERATOR_ID}.log"
    setsid env OPERATOR_ID="$OPERATOR_ID" HLR_IP="$SMSC_HLR_IP" RELAY_PORT="$SMSC_RELAY_PORT" \
        stdbuf -oL -eL bash "$SMSC_SCRIPT" >"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "$RUN_DIR/smsc-op${OPERATOR_ID}.pid"
    mod_ok
}

# BARRIÈRE — smsc-start.sh attend lui-même le HLR puis temporise : « lancé » ne
# dit rien. Les deux critères sont les deux points d'entrée réellement utilisés
# ensuite (envoi MT par la socket, SMS entrants par le relay).
mod_smsc_wait() {
    local to="${MOD_TIMEOUT[smsc]}"

    if ! wait_until "$to" "socket d'envoi MT ($SMSC_SENDMT_SOCKET)" core_unix_listen "$SMSC_SENDMT_SOCKET"; then
        mod_hint "voir $LOG_DIR/smsc-op${OPERATOR_ID}.log : le daemon n'a pas pu se connecter au HLR ($SMSC_HLR_IP:4222)"
        return $MOD_RC_FAIL
    fi
    if [ -f "$SMSC_RELAY_SCRIPT" ]; then
        if ! wait_until "$to" "relay SMS interop (TCP $SMSC_RELAY_PORT)" core_tcp_listen "$SMSC_RELAY_PORT"; then
            mod_hint "port $SMSC_RELAY_PORT déjà pris, ou sms-routing.conf illisible : voir $LOG_DIR/smsc-op${OPERATOR_ID}.log"
            return $MOD_RC_FAIL
        fi
    else
        mod_say "$SMSC_RELAY_SCRIPT absent — pas de relay inter-opérateurs, barrière non applicable"
    fi
    mod_ok
}

mod_smsc_stop() {
    local pf="$RUN_DIR/smsc-op${OPERATOR_ID}.pid"
    [ -f "$pf" ] && { kill "$(cat "$pf" 2>/dev/null)" 2>/dev/null; rm -f "$pf"; }
    pkill -f 'proto-smsc-daemon' 2>/dev/null
    pkill -f 'sms-interop-relay.py' 2>/dev/null
    rm -f "$SMSC_SENDMT_SOCKET" 2>/dev/null
    return 0
}
