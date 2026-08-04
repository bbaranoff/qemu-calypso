# =============================================================================
#  20-handoff — raccord du point d'entrée historique (start-clean.sh)  [PUR]
# =============================================================================
#
#  RÔLE
#      Garantir que le chemin qui marche aujourd'hui continue de marcher :
#
#          cd <arbre> && CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 \
#                        CALYPSO_SHUNT_REAL_FB=1 ./start-clean.sh
#          et, côté osmo_egprs :   MODE=qemu ./start-direct.sh
#
#      Le mode « qemu » de start-direct.sh ne fait qu'UNE chose : il se place
#      dans l'arbre QEMU et fait `exec ./start-clean.sh`. Tout ce que
#      l'utilisateur a mis en préfixe traverse par simple héritage
#      d'environnement, et start-clean.sh redonne la main à ce run.sh. Ce module
#      ne réimplémente donc RIEN — il vérifie que la couture tient :
#
#        1. start-clean.sh existe, est exécutable, et redonne la main à CE run.sh
#           (s'il pointait ailleurs, on croirait lancer cet arbre en en lançant
#           un autre — la panne la plus coûteuse à diagnostiquer) ;
#        2. aucune variable CALYPSO_* n'est VERROUILLÉE dans environnement/ par
#           une affectation sèche `VAR=…` : l'idiome obligatoire est
#           `: "${VAR:=…}"`, seul à laisser gagner la ligne de commande. Une
#           seule ligne fautive et le préfixe de l'utilisateur est ignoré en
#           silence, sans que rien ne le signale ;
#        3. l'arbre que start-direct.sh appelle réellement (QEMU_SRC, codé en dur
#           à ${QEMU_TREE}) est bien celui-ci — sinon on signale la
#           divergence, car les deux arbres ont chacun leur configuration.
#
#      Module PUR : il ne lance rien, ne modifie rien, et ne bloque pas le plan
#      (MOD_REQUIRED=0) — une divergence de raccord est un avertissement, pas une
#      panne de la pile.
#
#  PRÉREQUIS
#      profil.
#
#  CRITÈRE DE SUCCÈS
#      BARRIÈRE — un processus enfant voit au moins une variable CALYPSO_*, ce
#      qui prouve que la chaîne préfixe -> load.env -> export -> enfant est
#      continue. C'est exactement le trajet qu'empruntent les réglages jusqu'à
#      QEMU.
#
#  JOURNAL
#      $LOG_DIR/mod/handoff.log : points d'entrée trouvés, arbre appelé par
#      osmo_egprs, et la liste des CALYPSO_* qui franchiront la frontière.
# -----------------------------------------------------------------------------
MOD_REGISTER handoff "Raccord du point d'entrée historique"
MOD_REQUIRED[handoff]=0
MOD_PURE[handoff]=1
MOD_DEPS[handoff]="profil"
MOD_PROFILES[handoff]="calypso faketrx hybrid core"
MOD_TIMEOUT[handoff]=10

# Arbre visé par osmo_egprs/start-direct.sh (QEMU_SRC y est codé en dur).
: "${QEMU_SRC:=${QEMU_TREE}}"
: "${HANDOFF_ENTREES:=start-clean.sh start-oqc.sh}"

mod_handoff_check() {
    local tree="${QEMU_TREE:-}" e p manquantes=""

    for e in $HANDOFF_ENTREES; do
        p="$tree/$e"
        if [ ! -f "$p" ]; then manquantes="$manquantes $e"; continue; fi
        [ -x "$p" ] || { mod_hint "chmod +x $p"
                         mod_fail "$e existe mais n'est pas exécutable : la chaîne osmo_egprs MODE=qemu échouerait"
                         return $MOD_RC_FAIL; }
        grep -q 'run\.sh' "$p" || {
            mod_hint "ce point d'entrée doit se terminer par :  exec ./run.sh \"\$@\""
            mod_fail "$e ne redonne pas la main à run.sh"
            return $MOD_RC_FAIL; }
    done
    if [ -n "$manquantes" ]; then
        mod_hint "sans start-clean.sh, « MODE=qemu ./start-direct.sh » ne trouve plus de point d'entrée"
        mod_fail "point(s) d'entrée absent(s) :$manquantes"
        return $MOD_RC_FAIL
    fi

    # Contrat « la ligne de commande gagne toujours » : toute affectation sèche
    # d'une CALYPSO_* dans environnement/ la verrouille.
    local verrous
    verrous="$(grep -rnE '^[[:space:]]*(export[[:space:]]+)?CALYPSO_[A-Z0-9_]+=' \
                    "$tree/environnement" 2>/dev/null | grep -v '^\s*#' | head -n 10)"
    if [ -n "$verrous" ]; then
        mod_say "$verrous"
        mod_hint "remplacez ces lignes par l'idiome :  : \"\${VAR:=valeur}\"  (voir environnement/README.md)"
        mod_fail "variables CALYPSO_* verrouillées dans environnement/ : la ligne de commande serait ignorée"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_handoff_status() { return $MOD_RC_FAIL; }   # module pur

mod_handoff_start() {
    local e
    for e in $HANDOFF_ENTREES; do
        mod_say "point d'entrée : ${QEMU_TREE}/$e"
    done
    if [ -e "$QEMU_SRC" ] && [ "$(cd "$QEMU_SRC" 2>/dev/null && pwd -P)" != "$(cd "${QEMU_TREE}" 2>/dev/null && pwd -P)" ]; then
        mod_say "ATTENTION : osmo_egprs/start-direct.sh (MODE=qemu) lance $QEMU_SRC,"
        mod_say "            pas cet arbre (${QEMU_TREE}) — deux configurations parallèles."
    fi
    mod_say "--- réglages qui franchiront la frontière vers QEMU ---"
    env | grep '^CALYPSO_' | sort
    mod_ok
}

# La chaîne préfixe -> load.env -> export -> enfant est-elle continue ?
_handoff_enfant_voit() {
    local n
    n="$(bash -c 'env | grep -c "^CALYPSO_"' 2>/dev/null || echo 0)"
    [ "${n:-0}" -gt 0 ] 2>/dev/null
}

mod_handoff_wait() {
    if ! wait_until "${MOD_TIMEOUT[handoff]}" "réglages hérités par un enfant" _handoff_enfant_voit; then
        mod_hint "load.env doit être sourcé sous « set -a » (run.sh:75-80) ; vérifiez ensuite le manifeste : grep calypso-manifest sur le journal de QEMU"
        mod_fail "aucun réglage CALYPSO_* n'atteint les processus enfants"
        return $MOD_RC_FAIL
    fi
    mod_ok
}
