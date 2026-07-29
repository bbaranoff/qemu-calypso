# =============================================================================
#  65-virtphy — la couche 1 du mobile, côté Um virtuel
# =============================================================================
#
#  RÔLE
#    Équivalent de trxcon pour la chaîne multicast : il expose la même socket
#    L1CTL au mobile, mais côté réseau il rejoint les groupes multicast
#    d'osmo-bts-virtual au lieu de parler TRXD.
#
#        mobile <--L1CTL /tmp/osmocom_l2_N--> virtphy <--multicast UDP--> osmo-bts-virtual
#
#    Une instance par mobile ($N_MS). EXCLUSIF de trxcon : même socket, même
#    rôle — c'est PHY_MODE qui tranche.
#
#  PRÉREQUIS
#    PHY_MODE=virtphy, et osmo-bts-virtual démarré (c'est lui qui publie sur le
#    groupe ; virtphy seul ne produit rien et ne le dit pas).
#
#  CRITÈRE DE SUCCÈS (barrière)
#    Pour CHAQUE instance : le processus vit ET la socket L1CTL existe. C'est
#    exactement ce que l'ancien lancement remplaçait par « sleep 2 » avec le
#    commentaire « laisser virtphy créer les sockets » — la sonde dit la même
#    chose, mais elle est vraie.
#
#  JOURNAL     $LOG_DIR/virtphy-<n>.log
#
#  ORDRE       après osmo-bts-virtual, avant mobile.
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER virtphy "Couche 1 mobile virtuelle (virtphy)"
MOD_REQUIRED[virtphy]=1
MOD_DEPS[virtphy]="osmo-bts-virtual"
MOD_PROFILES[virtphy]="faketrx"
MOD_ENABLED_IF[virtphy]='[ "${PHY_MODE:-faketrx}" = "virtphy" ]'
MOD_TIMEOUT[virtphy]=30

: "${VIRTPHY_BIN:=virtphy}"
: "${VIRTPHY_EXTRA_ARGS:=}"
MOD_LOG[virtphy]="${LOG_DIR}/virtphy.log"

mod_virtphy_check() {
    command -v "$VIRTPHY_BIN" >/dev/null 2>&1 || [ -x "$VIRTPHY_BIN" ] || {
        mod_fail "binaire introuvable : $VIRTPHY_BIN"
        mod_hint "installez osmocom-bb (host/virt_phy), ou posez VIRTPHY_BIN"
        return $MOD_RC_FAIL; }

    if pgrep -x trxcon >/dev/null 2>&1; then
        mod_fail "trxcon tourne déjà sur les mêmes sockets L1CTL"
        mod_hint "les deux couches 1 sont exclusives : ./run.sh --stop, ou PHY_MODE=faketrx"
        return $MOD_RC_FAIL
    fi

    local n s
    for ((n=1; n<=N_MS; n++)); do
        s="$(radio_l2_sock "$n")"
        if [ -S "$s" ] && ! pgrep -f "$VIRTPHY_BIN.*$s" >/dev/null 2>&1; then
            mod_fail "socket L1CTL orpheline : $s (aucun virtphy derrière)"
            mod_hint "rm -f $s — ou ./run.sh --stop pour nettoyer proprement"
            return $MOD_RC_FAIL
        fi
    done
    mod_ok
}

mod_virtphy_status() { pgrep -x "$(basename "$VIRTPHY_BIN")" >/dev/null 2>&1; }

mod_virtphy_start() {
    radio_dirs
    local n s log
    for ((n=1; n<=N_MS; n++)); do
        s="$(radio_l2_sock "$n")"
        log="$(radio_log "virtphy-$n")"
        mod_say "ms$n : L1CTL $s ; journal $log"
        # shellcheck disable=SC2086
        stdbuf -oL -eL "$VIRTPHY_BIN" -s "$s" $VIRTPHY_EXTRA_ARGS >>"$log" 2>&1 &
        radio_save_pid "virtphy-$n" $!
    done
    mod_ok
}

# BARRIÈRE — une socket par instance, et l'instance encore vivante derrière.
mod_virtphy_wait() {
    local n s
    for ((n=1; n<=N_MS; n++)); do
        s="$(radio_l2_sock "$n")"
        wait_until "${MOD_TIMEOUT[virtphy]}" "socket L1CTL de ms$n ($s)" \
            have_unix "$s" || {
                mod_hint "fin de $(radio_log "virtphy-$n") : socket non créée (droits sur ${s%/*} ?)"
                return $MOD_RC_FAIL; }
        radio_alive "virtphy-$n" || {
            mod_fail "virtphy ms$n a créé sa socket puis s'est arrêté"
            mod_hint "fin de $(radio_log "virtphy-$n") ; multicast indisponible ?"
            return $MOD_RC_FAIL; }
    done
    mod_ok
}

mod_virtphy_stop() {
    local n s
    for ((n=1; n<=N_MS; n++)); do
        s="$(radio_l2_sock "$n")"
        radio_kill "virtphy-$n" "$VIRTPHY_BIN.*$s"
        rm -f "$s"
    done
    pkill -x "$(basename "$VIRTPHY_BIN")" 2>/dev/null
    return 0
}
