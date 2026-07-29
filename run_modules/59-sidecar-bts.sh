# =============================================================================
#  59-sidecar-bts — le second osmo-bts-trx (BTS#1 du profil hybride)
# =============================================================================
#
#  RÔLE    La station de base du side-car : elle parle TRXD à fake_trx d'un côté
#          et Abis/IPA au MÊME BSC que le BTS#0 de l'autre. C'est ce partage du
#          BSC qui permet à MS#2 d'appeler MS#1.
#
#  ⚠ COURSE CONNUE « No clock since TRX was started »
#    osmo-bts-trx peut mourir QUELQUES SECONDES après son démarrage si fake_trx
#    ne lui fournit pas l'horloge à temps. Un simple « le processus est vivant »
#    juste après le lancement déclarerait donc le module prêt à tort. On exige
#    une FENÊTRE DE STABILITÉ, et on relance jusqu'à SC_BTS_RETRIES fois — c'est
#    un contournement de course, pas un correctif : la vraie question est
#    pourquoi fake_trx tarde à publier son horloge.
#
#  SUCCÈS  processus stable pendant SC_BTS_STAB_SECS, sans « No clock » au
#          journal, puis VTY $SC_BTS_VTY_PORT en écoute.
#  JOURNAL $LOG_DIR/sidecar-bts.log
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-bts "Station de base BTS#1 (side-car)"
MOD_REQUIRED[sidecar-bts]=1
MOD_DEPS[sidecar-bts]="sidecar-faketrx bsc"
MOD_PROFILES[sidecar-bts]="hybrid"
MOD_TIMEOUT[sidecar-bts]=30

: "${SC_BTS_BIN:=osmo-bts-trx}"
: "${SC_BTS_STAB_SECS:=8}"
: "${SC_BTS_RETRIES:=3}"

_sc_bts_mort() {   # le journal avoue-t-il la course ?
    grep -qE "No clock since TRX|BTS_SHUTDOWN.*Shutting down" "$(radio_log sidecar-bts)" 2>/dev/null
}

mod_sidecar_bts_check() {
    command -v "$SC_BTS_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_BTS_BIN"; return $MOD_RC_FAIL; }
    [ -r "$SC_BTS_CFG" ] || {
        mod_hint "le module « sidecar-cfg » doit avoir posé ce fichier"
        mod_fail "configuration illisible : $SC_BTS_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_bts_status() { radio_alive sidecar-bts; }

# Le lancement PORTE la reprise sur course : une seule tentative laisserait le
# module échouer alors qu'un simple redémarrage suffit dans la majorité des cas.
mod_sidecar_bts_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-bts)"
    local essai=0 reste
    while :; do
        essai=$(( essai + 1 ))
        : > "$log"
        stdbuf -oL -eL "$SC_BTS_BIN" -c "$SC_BTS_CFG" >>"$log" 2>&1 &
        radio_save_pid sidecar-bts $!
        reste="$SC_BTS_STAB_SECS"
        while [ "$reste" -gt 0 ]; do
            radio_alive sidecar-bts || break
            _sc_bts_mort && break
            sleep 1; reste=$(( reste - 1 ))
        done
        if [ "$reste" -eq 0 ] && radio_alive sidecar-bts && ! _sc_bts_mort; then
            mod_say "BTS#1 stable après ${SC_BTS_STAB_SECS}s (essai $essai)"
            mod_ok; return $MOD_RC_OK
        fi
        radio_kill sidecar-bts
        if [ "$essai" -ge "$SC_BTS_RETRIES" ]; then
            modb_tail "$log" 20
            mod_hint "fake_trx publie-t-il son horloge ? voir $(radio_log sidecar-faketrx)"
            mod_fail "BTS#1 instable après $essai tentatives (course « No clock »)"
            return $MOD_RC_FAIL
        fi
        mod_say "BTS#1 tombé (« No clock ») — nouvelle tentative $essai/$SC_BTS_RETRIES"
    done
}

mod_sidecar_bts_wait() {
    wait_until "${MOD_TIMEOUT[sidecar-bts]}" "VTY du BTS#1 ($SC_BTS_VTY_PORT)" \
        have_port "$SC_BTS_VTY_PORT" || {
        modb_tail "$(radio_log sidecar-bts)" 20
        mod_fail "VTY $SC_BTS_VTY_PORT jamais en écoute"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_bts_stop() { radio_kill sidecar-bts; return 0; }
