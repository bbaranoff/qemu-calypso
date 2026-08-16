# =============================================================================
#  08-gabarits — poser les configurations du cœur avant de démarrer un démon
# =============================================================================
#
#  RÔLE
#    Substituer les jetons des gabarits d'osmo_egprs (__ENCRYPTION__, __MCC__,
#    __MNC__, __KI__, __ARFCN__, __RCTX_*__ …) puis déposer le résultat dans
#    /etc/osmocom, /etc/asterisk et ~/.osmocom/bb. Reprend, sans les réécrire,
#    apply_config_templates + install_configs_native de start-direct.sh.legacy
#    (L174-258), extraites dans osmo_egprs/lib/gabarits.sh.
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. C'EST LA SEULE ÉTAPE DU LEGACY QUE PERSONNE D'AUTRE NE REPREND.
#     Les vingt lancements de start-direct.sh sont désormais des modules. La
#     pose des gabarits, non : aucun module ne touchait à /etc/osmocom. Sans
#     elle, les démons repartent sur les fichiers laissés par le run PRÉCÉDENT.
#     Conséquence observée dans le legacy : `ENCRYPTION="a5 1" ./start-direct.sh`
#     n'avait aucun effet si le run d'avant avait posé a5/0 — le réseau montait,
#     le LU passait, et le chiffrement demandé n'était nulle part. Une panne
#     silencieuse, exactement celle que ce découpage existe pour supprimer.
#
#  2. POURQUOI CE MODULE N'A PAS DE `status`.
#     « Déjà démarré » n'a pas de sens ici : les gabarits doivent être reposés
#     à CHAQUE run, puisque c'est la ligne de commande du run courant qui fixe
#     ENCRYPTION, MCC/MNC et le plan d'adressage. Un `status` qui répondrait
#     « déjà fait » figerait la configuration du premier run pour toujours.
#
#  3. LA BARRIÈRE VÉRIFIE LE RÉSULTAT, PAS LE `cp`.
#     Un `cp` réussi ne prouve rien : si un jeton n'a pas de valeur, le fichier
#     part en production avec `__MCC__` en toutes lettres et osmo-msc refuse de
#     démarrer avec un message qui ne parle pas de gabarit. On relit donc les
#     fichiers INSTALLÉS et on exige (a) qu'ils existent, (b) qu'il n'y reste
#     aucun jeton `__…__`, (c) que la valeur d'ENCRYPTION demandée s'y trouve.
#
#  4. MONO-OPÉRATEUR SEULEMENT — et c'est dit, pas caché.
#     Le legacy appelait ces mêmes fonctions par opérateur, avec un préfixe
#     /etc/netns/osmo-op<i>. Aucun module du cœur ne sait aujourd'hui démarrer
#     un démon DANS un netns : poser des configurations multi-opérateur que
#     personne ne consommerait serait un faux positif. Au-delà d'un opérateur,
#     ce module échoue en le disant.
#
#  PRÉREQUIS : osmo_egprs présent (ses gabarits et sa bibliothèque), root.
#  SUCCÈS    : cf. POURQUOI 3.
#  JOURNAL   : $LOG_DIR/mod/gabarits.log
# -----------------------------------------------------------------------------

MOD_REGISTER gabarits "Gabarits de configuration du cœur"
MOD_REQUIRED[gabarits]=0
MOD_DEPS[gabarits]="config"
MOD_PROFILES[gabarits]="calypso faketrx hybrid core"
MOD_TIMEOUT[gabarits]=20

# Échappatoires héritées : NO_OSMO_START=1 saute tout le cœur (donc ses
# configurations) ; NO_GABARITS=1 conserve les fichiers /etc en l'état, pour
# qui veut éditer /etc/osmocom à la main entre deux runs.
MOD_ENABLED_IF[gabarits]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${NO_GABARITS:-0}" != 1 ]'

: "${EGPRS_DIR:=${NITB_ROOT}}"
: "${EGPRS_LIB:=$EGPRS_DIR/lib/gabarits.sh}"
# Identité du réseau. Ces valeurs sont celles que run_single_op passait en dur
# (start-direct.sh.legacy L1043-1044, L1118-1120) ; elles restent surchargeables.
: "${OPERATOR_ID:=1}"
: "${MCC:=001}"
: "${MNC:=01}"
: "${OPERATOR_NAME:=OsmoDirect}"
: "${ENCRYPTION:=a5 0}"

# Fichiers dont on relit le résultat. osmo-bsc porte __ENCRYPTION__ et le plan
# radio, osmo-msc l'identité réseau : si ces deux-là sont propres, la
# substitution a fonctionné.
_gab_installes() {
    printf '%s\n' "${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg" \
                  "${OSMOCOM_CFG:-/etc/osmocom}/osmo-msc.cfg"
}

mod_gabarits_check() {
    if [ ! -d "$EGPRS_DIR/configs" ]; then
        mod_hint "posez EGPRS_DIR=<racine osmo_egprs> si l'arborescence a bougé"
        mod_skip "osmo_egprs introuvable ($EGPRS_DIR) : les gabarits ne sont pas de ce dépôt"
        return $MOD_RC_SKIP
    fi
    if [ ! -r "$EGPRS_LIB" ]; then
        mod_hint "la bibliothèque est l'extraction de start-direct.sh.legacy L174-258 ; elle doit accompagner osmo_egprs"
        mod_fail "bibliothèque de gabarits absente : $EGPRS_LIB"
        return $MOD_RC_FAIL
    fi
    if [ "$(id -u)" -ne 0 ]; then
        mod_fail "root requis pour écrire dans ${OSMOCOM_CFG:-/etc/osmocom} et /etc/asterisk"
        return $MOD_RC_FAIL
    fi
    if [ "${N_OPERATORS:-1}" -gt 1 ]; then
        mod_hint "le multi-opérateur pose une configuration par netns ; aucun module du cœur ne démarre encore dans un netns"
        mod_fail "N_OPERATORS=${N_OPERATORS} : ce module ne sait poser que le mono-opérateur (cf. POURQUOI 4)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# Pas de `status` : cf. POURQUOI 2. On le déclare quand même, en échec, pour
# que run.sh ne saute jamais l'étape ni ne la croie « déjà démarrée ».
mod_gabarits_status() { return $MOD_RC_FAIL; }

mod_gabarits_start() {
    # Les fonctions extraites lisent `configs/*.cfg` et `scripts/*` en chemin
    # RELATIF, comme dans l'original : on se place donc dans osmo_egprs, et on
    # revient d'où l'on vient — le moteur n'a pas à hériter de ce cd.
    local retour="$PWD" rc=0
    cd "$EGPRS_DIR" || { mod_fail "impossible d'entrer dans $EGPRS_DIR"; return $MOD_RC_FAIL; }

    # shellcheck source=/dev/null
    . "$EGPRS_LIB" || { cd "$retour"; mod_fail "bibliothèque illisible : $EGPRS_LIB"; return $MOD_RC_FAIL; }

    local tmpdir; tmpdir="$(mktemp -d)" || { cd "$retour"; mod_fail "mktemp -d a échoué"; return $MOD_RC_FAIL; }

    local cip gw
    cip="$(op_private_ip "$OPERATOR_ID")"
    gw="$(op_private_gw "$OPERATOR_ID")"

    mod_say "opérateur $OPERATOR_ID · MCC/MNC ${MCC}/${MNC} · chiffrement « ${ENCRYPTION} »"
    mod_say "gabarits  : $EGPRS_DIR/configs -> ${OSMOCOM_CFG:-/etc/osmocom}, /etc/asterisk, $HOME/.osmocom/bb"

    # Mono-opérateur : pas de backbone inter-op, l'ASP vers l'inter-STP reste
    # 'shutdown' — argument à l'identique de run_single_op (legacy L1071/L1119).
    if apply_config_templates "$tmpdir" "$cip" "$gw" "$OPERATOR_ID" \
            "1.1.1" "1.1.2" "1.1.3" "$MCC" "$MNC" "$OPERATOR_NAME" \
            "${INTER_STP_IP:-127.0.0.1}" "shutdown" "1"; then
        install_configs_native "$tmpdir" "" || rc=1
    else
        rc=1
    fi

    rm -rf "$tmpdir"
    cd "$retour"
    [ $rc -eq 0 ] || { mod_fail "la pose des gabarits a échoué (détail dans le journal du module)"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — cf. POURQUOI 3. Immédiate : `cp` est synchrone, il n'y a rien à
# attendre. Ce qu'on vérifie n'est donc pas une temporisation mais un RÉSULTAT,
# relu sur les fichiers réellement installés.
mod_gabarits_wait() {
    local f restants=""
    while IFS= read -r f; do
        if [ ! -f "$f" ]; then
            mod_hint "vérifiez que $EGPRS_DIR/configs contient bien le gabarit correspondant"
            mod_fail "configuration non installée : $f"
            return $MOD_RC_FAIL
        fi
        # Un jeton non substitué = une valeur qu'on a oublié de fournir.
        if grep -qE '__[A-Z0-9_]+__' "$f" 2>/dev/null; then
            restants="$restants $(basename "$f"):$(grep -oE '__[A-Z0-9_]+__' "$f" | sort -u | tr '\n' ',' )"
        fi
    done < <(_gab_installes)

    if [ -n "$restants" ]; then
        mod_hint "ces jetons n'ont pas de valeur dans apply_config_templates (lib/gabarits.sh)"
        mod_fail "jetons non substitués :$restants"
        return $MOD_RC_FAIL
    fi

    # Le chiffrement demandé est-il VRAIMENT dans la configuration posée ?
    # C'est le point 1 du POURQUOI : la panne silencieuse à supprimer.
    # [2026-08-14] On contrôle DÉSORMAIS osmo-bsc.cfg ET osmo-msc.cfg. Le gabarit
    # MSC porte lui aussi « encryption __ENCRYPTION__ » (ligne 155) : sa
    # substitution était réelle mais NON contrôlée — un gabarit MSC cassé serait
    # passé en silence, et en A5/1 c'est le MSC qui décide d'authentifier. Une
    # panne silencieuse de moins avant la bascule.
    local cfg
    for cfg in "${OSMOCOM_CFG:-/etc/osmocom}/osmo-bsc.cfg" \
               "${OSMOCOM_CFG:-/etc/osmocom}/osmo-msc.cfg"; do
        if grep -q '__ENCRYPTION__' "$cfg" 2>/dev/null || ! grep -qF "$ENCRYPTION" "$cfg" 2>/dev/null; then
            mod_hint "attendu « $ENCRYPTION » dans $cfg ; vérifiez que le gabarit porte bien __ENCRYPTION__"
            mod_fail "le chiffrement demandé n'est pas dans la configuration installée ($cfg)"
            return $MOD_RC_FAIL
        fi
        mod_say "chiffrement « $ENCRYPTION » confirmé dans $cfg"
    done
    mod_ok
}

# Volontairement SANS effet : arrêter la pile ne doit pas effacer /etc/osmocom.
# Un `--stop` qui laisse la machine sans configuration rendrait le run suivant
# dépendant de ce module — alors qu'il est optionnel.
mod_gabarits_stop() { return 0; }
