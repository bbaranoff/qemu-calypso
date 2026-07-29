# =============================================================================
#  31-hlr-feed — l'abonné de test dans le HLR (IMSI + Ki comp128v1)
# =============================================================================
#
#  RÔLE        Injecte, via le VTY du HLR, l'IMSI et la clé Ki lus dans la
#              configuration du mobile. Legacy : run.sh.legacy L1567-1586.
#
#  POURQUOI    Sans Ki dans le HLR, la MSC ne peut pas authentifier : pas de
#              triplet, donc pas de Kc, donc CIPHER MODE COMMAND rejeté et
#              Location Update chiffré KO. En clair (a5/0, sans authentification)
#              le LU passe quand même — c'est ce qui rendait la panne si
#              trompeuse : « ça marchait » jusqu'au jour où le réseau exigeait
#              le chiffrement.
#
#  PRÉREQUIS   Cœur démarré (VTY HLR 4258 ouvert) et cfg mobile lisible.
#
#  SUCCÈS      `subscriber imsi <IMSI> show` répond avec la fiche de l'abonné.
#              C'est l'état FINAL qu'on vérifie, pas le fait d'avoir écrit :
#              une commande VTY refusée renvoie « % ... » sans code d'erreur.
#
#  JOURNAL     $LOG_DIR/mod/hlr-feed.log (échanges VTY recopiés par mod_say).
#
#  NON OBLIGATOIRE : un échec ici n'empêche pas de camper ni de faire un LU en
#  clair ; il ne casse que le chiffrement. On le signale, on ne bloque pas.
# -----------------------------------------------------------------------------

MOD_REGISTER hlr-feed "Abonné de test dans le HLR"
MOD_REQUIRED[hlr-feed]=0
MOD_DEPS[hlr-feed]="core $(modb_dep_known mobile-cfg)"
MOD_PROFILES[hlr-feed]="calypso hybrid core"
MOD_TIMEOUT[hlr-feed]=20

# DOUBLON DE ROLE — 21-abonnes-hlr provisionne les abonnes du HLR sur le meme
# VTY. Deux modules qui ecrivent la meme fiche, c'est un verdict de trop et une
# course a l'ecriture. On cede la place quand il est enregistre ; s'il disparait
# de l'arbre, ce module reprend son role sans modification. Meme porte que
# 30-core, meme raison.
MOD_ENABLED_IF[hlr-feed]='[ -z "${MOD_DESC[abonnes-hlr]+x}" ]'

: "${CALYPSO_HLR_VTY_IP:=127.0.0.1}"
: "${OSMO_VTY_HLR:=4258}"

# Résolus (lecture seule) par mod_hlr_feed_check.
_HLRF_CFG=""
_HLRF_IMSI=""
_HLRF_KI=""

# _hlrf_vty <commande...> — ouvre le VTY, envoie, rend la réponse sur stdout.
# bash /dev/tcp plutôt que telnet/nc : aucune dépendance à installer, et c'est
# déjà l'idiome du legacy (L1579). `enable` d'abord : `subscriber ... create`
# est une commande privilégiée.
_hlrf_vty() {
    local out=""
    exec 9<>"/dev/tcp/${CALYPSO_HLR_VTY_IP}/${OSMO_VTY_HLR}" 2>/dev/null || return 1
    { printf 'enable\n'; printf '%s\n' "$@"; } >&9
    out="$(timeout 3 cat <&9 2>/dev/null)"
    exec 9>&- 2>/dev/null
    exec 9<&- 2>/dev/null
    printf '%s\n' "$out"
}

# L'abonné est-il DANS la base ? osmo-hlr répond « % No subscriber ... » quand
# il ne l'est pas ; on exige en plus de retrouver l'IMSI dans la fiche, pour ne
# pas prendre un message d'erreur inattendu pour un succès.
_hlrf_present() {
    [ -n "$_HLRF_IMSI" ] || return 1
    local out; out="$(_hlrf_vty "subscriber imsi $_HLRF_IMSI show")" || return 1
    case "$out" in *"No subscriber"*|*"% "*) return 1;; esac
    printf '%s' "$out" | grep -q "$_HLRF_IMSI"
}

mod_hlr_feed_check() {
    # Où est la cfg mobile : celle déployée par le module mobile-cfg, sinon la
    # version du dépôt. Lecture seule dans les deux cas.
    local c
    for c in "${MOBILE_CFG:-}" \
             "${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg" \
             "${QEMU_CFGS:-${QEMU_TREE:-${QEMU_TREE}}/cfgs}/mobile_group1.cfg"; do
        [ -n "$c" ] && [ -r "$c" ] && { _HLRF_CFG="$c"; break; }
    done
    [ -n "$_HLRF_CFG" ] || {
        mod_hint "déployez la cfg mobile (module mobile-cfg) ou posez MOBILE_CFG=<chemin>"
        mod_fail "configuration du mobile introuvable : impossible d'en tirer IMSI/Ki"
        return $MOD_RC_FAIL
    }

    # Format attendu (cfgs/mobile_group1.cfg L84-85) :
    #    imsi 001010001000001
    #    ki comp128 00 11 22 ...
    _HLRF_IMSI="$(awk '$1=="imsi"{print $2; exit}' "$_HLRF_CFG" 2>/dev/null)"
    _HLRF_KI="$(awk '$1=="ki" && $2=="comp128"{s=""; for(i=3;i<=NF;i++) s=s $i; print s; exit}' "$_HLRF_CFG" 2>/dev/null)"

    [ -n "$_HLRF_IMSI" ] || {
        mod_hint "ajoutez une ligne « imsi <15 chiffres> » dans $_HLRF_CFG"
        mod_fail "aucun IMSI dans $_HLRF_CFG"
        return $MOD_RC_FAIL
    }
    [ -n "$_HLRF_KI" ] || {
        mod_hint "ajoutez « ki comp128 <16 octets hex> » dans $_HLRF_CFG — sans Ki, pas de Kc, CIPHER MODE rejeté"
        mod_fail "aucune clé Ki comp128 dans $_HLRF_CFG"
        return $MOD_RC_FAIL
    }
    mod_say "cfg=$_HLRF_CFG imsi=$_HLRF_IMSI ki=${#_HLRF_KI} caractères hex"
    mod_ok
}

mod_hlr_feed_status() { _hlrf_present; }

mod_hlr_feed_start() {
    # Le legacy attendait ici le VTY 90 × 2 s en tâche de fond (L1578) : le HLR
    # pouvait ne pas être prêt. Ce sleep n'a plus lieu d'être — la barrière du
    # module `core` garantit déjà que 4258 écoute, sinon ce module est sauté.
    # Convention de la maquette : MSISDN = 1000 + rang du mobile, le rang étant
    # le dernier chiffre significatif de l'IMSI (…000001 -> 1, …000002 -> 2).
    # Prendre bêtement les 5 derniers chiffres de l'IMSI donnait « 00001 », un
    # numéro que personne n'appelle : « show subscriber msisdn 10001 » répondait
    # « No subscriber found » et tout SMS vers le MS#1 échouait en silence.
    # CALYPSO_MSISDN_BASE permet de changer de plan de numérotation sans toucher
    # au code.
    local rang msisdn out
    rang="$(printf '%s' "${_HLRF_IMSI: -4}" | sed 's/^0*//')"; : "${rang:=1}"
    msisdn="${CALYPSO_MSISDN_BASE:-1000}${rang}"
    out="$(_hlrf_vty \
        "subscriber imsi $_HLRF_IMSI create" \
        "subscriber imsi $_HLRF_IMSI update msisdn $msisdn" \
        "subscriber imsi $_HLRF_IMSI update aud2g comp128v1 ki $_HLRF_KI")" || {
        mod_hint "vérifiez que osmo-hlr écoute : ./run.sh --status"
        mod_fail "VTY HLR ${CALYPSO_HLR_VTY_IP}:${OSMO_VTY_HLR} injoignable"
        return $MOD_RC_FAIL
    }
    mod_say "$out"
    mod_ok
}

# BARRIÈRE — on relit la base : « la commande est partie » ne prouve rien, le
# VTY accepte la connexion puis refuse la commande sans code d'erreur.
mod_hlr_feed_wait() {
    wait_until "${MOD_TIMEOUT[hlr-feed]}" "abonné $_HLRF_IMSI dans le HLR" _hlrf_present && { mod_ok; return $MOD_RC_OK; }
    mod_hint "à la main : telnet ${CALYPSO_HLR_VTY_IP} ${OSMO_VTY_HLR} puis « subscriber imsi $_HLRF_IMSI show »"
    mod_fail "l'abonné $_HLRF_IMSI n'apparaît pas dans le HLR après injection"
    return $MOD_RC_FAIL
}

# Volontairement SANS effet : arrêter la pile ne doit pas détruire la base des
# abonnés. Supprimer l'abonné rendrait le run suivant dépendant de ce module,
# alors qu'il est optionnel.
mod_hlr_feed_stop() { return 0; }
