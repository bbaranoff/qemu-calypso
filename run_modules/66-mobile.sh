# =============================================================================
#  66-mobile — le mobile Osmocom (couches 2 et 3, layer23)
# =============================================================================
#
#  RÔLE
#    Le dernier maillon : la pile L2/L3 du téléphone. Il ne connaît qu'une seule
#    chose de tout ce qui précède — une socket L1CTL — et se moque de savoir qui
#    la sert :
#
#        osmocon (firmware Calypso sous QEMU)  ┐
#        trxcon  (chaîne TRXD)                 ├─> socket L1CTL ─> mobile ─> VTY 4247
#        virtphy (chaîne multicast)            ┘
#
#    Dans le profil « faketrx » ce producteur est trxcon ou virtphy selon
#    PHY_MODE : les deux sont déclarés en prérequis, et le moteur traite celui
#    qui est désactivé comme satisfait.
#
#  PÉRIMÈTRE — NE PAS CONFONDRE AVEC 70-l2
#    70-l2.sh lance le client de couche 2 de la chaîne de l'émulateur (profils
#    calypso/hybrid), derrière osmocon. Celui-ci est le mobile du profil
#    « faketrx ». Les deux se disputeraient la VTY 4247 : ils ne partagent
#    aucun profil.
#
#  PRÉREQUIS
#    $MOBILE_CFG lisible. Le module en EXTRAIT la socket L1CTL (`layer2-socket`)
#    et le port VTY (`line vty` / `bind`) plutôt que de les supposer : c'est la
#    conf qui fait foi, et c'est là qu'est l'erreur classique — osmocon expose
#    /tmp/osmocom_l2 (sans suffixe), trxcon et virtphy exposent
#    /tmp/osmocom_l2_<n>. Une conf qui vise la mauvaise reste muette pour
#    toujours.
#
#  CRITÈRE DE SUCCÈS (barrière)
#    Le processus vit ET sa VTY répond. La VTY n'est pas décorative : elle
#    n'ouvre qu'après l'initialisation complète de la pile, donc elle distingue
#    « lancé » de « prêt à recevoir des commandes ». La suite (sélection de
#    cellule, LU) n'est PAS un critère de démarrage : elle dépend du réseau,
#    pas de ce module.
#
#  JOURNAL     $LOG_DIR/mobile.log
#
#  ORDRE       en dernier — après trxcon ou virtphy.
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER mobile "Mobile Osmocom (layer23)"
MOD_REQUIRED[mobile]=1
MOD_PROFILES[mobile]="faketrx"
MOD_TIMEOUT[mobile]=60

# DÉPENDANCE CROISÉE : la socket L1CTL vient de trxcon OU de virtphy selon
# PHY_MODE. On déclare les deux ; celui qui est désactivé compte comme satisfait.
MOD_DEPS[mobile]="trxcon virtphy"

: "${MOBILE_BIN:=mobile}"
: "${MOBILE_CFG:=${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg}"
: "${MOBILE_L1CTL_TIMEOUT:=60}"
# Catégories de trace du mobile (séparateur « : », pas « , »).
: "${MOBILE_DEBUG:=DCS:DNB:DPLMN:DRR:DMM:DSIM:DCC:DMNCC:DSS:DLSMS:DPAG:DSUM:DSAP:DMOB:DPRIM}"
MOD_LOG[mobile]="${LOG_DIR}/mobile.log"

# --- ce que dit la conf, pas ce qu'on suppose --------------------------------
_mobile_l2_sock() { radio_cfg_val "$MOBILE_CFG" "layer2-socket"; }
_mobile_vty_port() {
    awk '/^[[:space:]]*line vty/{v=1; next}
         v && /^[[:space:]]*bind[[:space:]]/{print $3; exit}' "$MOBILE_CFG" 2>/dev/null
}
# Sockets L1CTL réellement présentes — sert à rendre l'échec actionnable.
_mobile_sockets_present() {
    local s found=""
    for s in "$L2_SOCK_BASE" "$L2_SOCK_BASE"_*; do
        [ -S "$s" ] && found="$found $s"
    done
    printf '%s' "${found:- aucune}"
}

mod_mobile_check() {
    command -v "$MOBILE_BIN" >/dev/null 2>&1 || [ -x "$MOBILE_BIN" ] || {
        mod_fail "binaire introuvable : $MOBILE_BIN"
        mod_hint "installez osmocom-bb (host/layer23), ou posez MOBILE_BIN"
        return $MOD_RC_FAIL; }
    [ -r "$MOBILE_CFG" ] || {
        mod_fail "configuration illisible : $MOBILE_CFG"
        mod_hint "posez MOBILE_CFG, ou déployez cfgs/mobile_group1.cfg dans ${OSMOCOM_HOME:-\$HOME/.osmocom}/bb/"
        return $MOD_RC_FAIL; }

    local s v; s="$(_mobile_l2_sock)"; v="$(_mobile_vty_port)"
    [ -n "$s" ] || {
        mod_fail "aucune directive « layer2-socket » dans $MOBILE_CFG"
        mod_hint "ajoutez « layer2-socket <chemin> » sous le bloc « ms 1 »"
        return $MOD_RC_FAIL; }
    # COHÉRENCE — l'erreur la plus coûteuse de la chaîne, attrapée AVANT le
    # lancement plutôt qu'après soixante secondes d'attente muette : la conf
    # nomme une socket que personne ne servira. On ne condamne que si elle
    # n'existe pas déjà (quelqu'un d'autre peut légitimement la servir).
    local exp; exp="$(radio_l2_sock 1)"
    if [ "$s" != "$exp" ] && [ ! -S "$s" ]; then
        mod_fail "la conf vise $s, mais la couche 1 du profil expose $exp"
        mod_hint "corrigez « layer2-socket $exp » dans $MOBILE_CFG, ou posez L2_SOCK_BASE=${s%_*}"
        return $MOD_RC_FAIL
    fi

    mod_say "socket L1CTL attendue : $s"
    mod_say "VTY déclarée : ${v:-$MOBILE_VTY_PORT (défaut, aucun « bind » dans la conf)}"
    mod_ok
}

mod_mobile_status() { pgrep -f "$MOBILE_BIN -c $MOBILE_CFG" >/dev/null 2>&1; }

# Le mobile doit attendre SA socket : la lancer avant, c'est un mobile qui sort
# en erreur « cannot connect » sans rien laisser d'exploitable. Cette attente est
# une synchronisation de démarrage, bornée, pas un `sleep`.
mod_mobile_start() {
    radio_dirs
    local log s; log="$(radio_log mobile)"; s="$(_mobile_l2_sock)"

    if ! wait_until "$MOBILE_L1CTL_TIMEOUT" "socket L1CTL $s" have_unix "$s"; then
        mod_fail "socket L1CTL jamais apparue : $s (présentes :$(_mobile_sockets_present) )"
        mod_hint "alignez « layer2-socket » de $MOBILE_CFG sur son producteur — osmocon expose ${L2_SOCK_BASE}, trxcon/virtphy exposent ${L2_SOCK_BASE}_<n>"
        return $MOD_RC_FAIL
    fi

    # La socket L1CTL existe avant qu'osmocon ne soit prêt à la servir : le
    # legacy (run.sh L2131) attendait la socket PUIS dormait 3 s. Sans ce
    # délai, le mobile se connecte trop tôt et repart sans jamais chercher de
    # cellule. Réglable par CALYPSO_MOBILE_DELAY, 0 pour l'annuler.
    local delai="${CALYPSO_MOBILE_DELAY:-3}"
    if [ "$delai" != 0 ]; then
        mod_say "attente de ${delai}s avant lancement (osmocon doit servir la socket)"
        sleep "$delai"
    fi

    mod_say "commande : $MOBILE_BIN -c $MOBILE_CFG -d $MOBILE_DEBUG"
    mod_say "journal  : $log"
    stdbuf -oL -eL "$MOBILE_BIN" -c "$MOBILE_CFG" -d "$MOBILE_DEBUG" >>"$log" 2>&1 &
    radio_save_pid mobile $!
    mod_ok
}

# BARRIÈRE — la VTY n'ouvre qu'une fois la pile initialisée.
mod_mobile_wait() {
    local log vty; log="$(radio_log mobile)"
    vty="$(_mobile_vty_port)"; [ -n "$vty" ] || vty="$MOBILE_VTY_PORT"

    wait_until "${MOD_TIMEOUT[mobile]}" "VTY du mobile ($vty)" have_port "$vty" || {
        mod_hint "fin de $log ; port déjà pris par un autre mobile ? ss -tlnp | grep :$vty"
        return $MOD_RC_FAIL; }

    radio_alive mobile || {
        mod_fail "le mobile a ouvert sa VTY puis s'est arrêté"
        mod_hint "fin de $log"
        return $MOD_RC_FAIL; }

    # Renseignement : la suite (cellule trouvée, LU) dépend du réseau, elle ne
    # peut pas conditionner le démarrage de ce module.
    mod_say "prêt — suivre la sélection de cellule : telnet 127.0.0.1 $vty, puis « show ms »"
    mod_ok
}

# Arrêt CIBLÉ : le motif inclut le fichier de conf. Un `pkill -f mobile` — six
# lettres sans ancrage, comme le faisait l'ancien teardown — emporte tout ce qui
# contient « mobile » dans sa ligne de commande, y compris ce qui n'est pas à
# nous.
mod_mobile_stop() {
    radio_kill mobile "$MOBILE_BIN -c $MOBILE_CFG"
    return 0
}
