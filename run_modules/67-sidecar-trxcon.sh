# =============================================================================
#  67-sidecar-trxcon — la couche 1 du MS#2 (profil hybride)
# =============================================================================
#
#  RÔLE    trxcon tient le rôle que le firmware Calypso tient pour le MS#1 :
#          il parle TRXD à fake_trx d'un côté, et expose une socket L1CTL au
#          mobile de l'autre.
#
#              trxcon <--TRXD $SC_BB_PORT--> fake_trx        (côté air)
#              trxcon <--L1CTL $SC_L2_SOCK--> mobile MS#2    (côté pile)
#
#  SUCCÈS  la socket L1CTL existe. C'est trxcon qui la crée : sa présence est un
#          observable, là où un motif de journal dépendrait de la version.
#  JOURNAL $LOG_DIR/sidecar-trxcon.log
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-trxcon "Couche 1 du MS#2 (trxcon)"
MOD_REQUIRED[sidecar-trxcon]=1
MOD_DEPS[sidecar-trxcon]="sidecar-faketrx"
MOD_PROFILES[sidecar-trxcon]="hybrid"
MOD_TIMEOUT[sidecar-trxcon]=20

: "${SC_TRXCON_BIN:=trxcon}"
: "${SC_TRXCON_FN_ADVANCE:=100}"

mod_sidecar_trxcon_check() {
    command -v "$SC_TRXCON_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_TRXCON_BIN"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_trxcon_status() { radio_alive sidecar-trxcon && have_unix "$SC_L2_SOCK"; }

mod_sidecar_trxcon_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-trxcon)"
    # Une socket d'un run précédent ferait passer la barrière à tort.
    rm -f "$SC_L2_SOCK" 2>/dev/null
    mod_say "trxcon -p $SC_BB_PORT -s $SC_L2_SOCK (avance de trame $SC_TRXCON_FN_ADVANCE)"
    mod_say "journal : $log"
    stdbuf -oL -eL "$SC_TRXCON_BIN" -i "$TRX_BIND_IP" -b "$TRX_BB_IP" \
        -p "$SC_BB_PORT" -s "$SC_L2_SOCK" -C 1 -F "$SC_TRXCON_FN_ADVANCE" >>"$log" 2>&1 &
    radio_save_pid sidecar-trxcon $!
    mod_ok
}

mod_sidecar_trxcon_wait() {
    local log; log="$(radio_log sidecar-trxcon)"
    wait_until 5 "processus trxcon" radio_alive sidecar-trxcon || {
        modb_tail "$log" 20
        mod_fail "trxcon s'est arrêté aussitôt lancé"; return $MOD_RC_FAIL; }
    wait_until "${MOD_TIMEOUT[sidecar-trxcon]}" "socket L1CTL $SC_L2_SOCK" \
        have_unix "$SC_L2_SOCK" || {
        modb_tail "$log" 20
        mod_hint "fake_trx répond-il sur $SC_BB_PORT ? voir $(radio_log sidecar-faketrx)"
        mod_fail "socket L1CTL jamais créée"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_trxcon_stop() { radio_kill sidecar-trxcon; rm -f "$SC_L2_SOCK"; return 0; }
