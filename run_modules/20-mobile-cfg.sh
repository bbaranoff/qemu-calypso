# =============================================================================
#  20-mobile-cfg — configuration du client mobile (osmocom-bb layer23)
# =============================================================================
#
#  RÔLE
#    Installer la configuration versionnée du mobile là où le binaire la
#    cherche, et y appliquer la seule surcharge utile au banc : l'ARFCN
#    verrouillé (`stick`). Reprend run.sh.legacy #9 (L1034-1045) et #10
#    (L1047-1056).
#
#  PRÉREQUIS  : 05-config (MOBILE_CFG_SRC et MOBILE_CFG résolus).
#  SUCCÈS     : $MOBILE_CFG existe, identique à la source (aux surcharges
#               près), et l'on peut en extraire l'IMSI et le Ki — sans quoi le
#               module 31 ne pourra pas alimenter le HLR.
#  JOURNAL    : $LOG_DIR/mod/mobile-cfg.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI RECOPIER À CHAQUE RUN.
#     Le fichier de $HOME est modifiable à la main et par le VTY du mobile
#     lui-même (`write terminal`). Sans recopie, deux runs successifs ne
#     partent pas de la même configuration et la comparaison de leurs journaux
#     ne veut plus rien dire. La copie est donc forcée (legacy L1035-1036) ;
#     CALYPSO_SYNC_MOBILE_CFG=0 pour garder délibérément un fichier bricolé.
#
#  2. POURQUOI L'IMSI ET LE Ki NE SONT PAS EXPORTÉS.
#     Le module 31 (alimentation du HLR) tourne dans le MÊME shell que celui-ci :
#     de simples variables lui suffisent. Les exporter les ferait entrer dans
#     l'environnement de QEMU et de tous les processus fils, où elles n'ont
#     rien à faire — un secret de SIM n'a pas à voyager plus loin que son
#     unique consommateur.
#
#  3. POURQUOI CE MODULE N'EST PAS OBLIGATOIRE.
#     Sans lui, le mobile démarre quand même avec la configuration déjà
#     installée. Ce qui casserait, c'est l'alimentation du HLR (donc le
#     chiffrement), et c'est le module 31 qui doit le dire.
# -----------------------------------------------------------------------------

MOD_REGISTER mobile-cfg "Configuration du mobile"
MOD_REQUIRED[mobile-cfg]=0
MOD_DEPS[mobile-cfg]="config"
# `core` AJOUTE : hlr-feed declare `mobile-cfg` en prerequis (c'est de ce fichier
# que sortent l'IMSI et le Ki). Hors du profil, run.sh voit une dependance
# "absente" et saute hlr-feed — dans un profil coeur, l'abonne de test n'etait
# donc JAMAIS provisionne, sans qu'aucune ligne ne l'explique. Installer une
# configuration de mobile dans un profil sans mobile est sans effet de bord :
# c'est une copie de fichier, et le module reste optionnel.
MOD_PROFILES[mobile-cfg]="calypso hybrid faketrx core"
MOD_TIMEOUT[mobile-cfg]=10

_mcfg_imsi() { grep -oE '^[[:space:]]*imsi [0-9]{15}' "${MOBILE_CFG:-}" 2>/dev/null | head -1 | grep -oE '[0-9]{15}'; }
_mcfg_ki()   { sed -n 's/^[[:space:]]*ki comp128 \([0-9a-fA-F ]*\)$/\1/p' "${MOBILE_CFG:-}" 2>/dev/null | head -1 | tr -d ' '; }
_mcfg_stick(){ sed -n 's/^[[:space:]]*stick \([0-9]\+\).*/\1/p' "${MOBILE_CFG:-}" 2>/dev/null | head -1; }

mod_mobile_cfg_check() {
    if [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" != "1" ]; then
        [ -r "${MOBILE_CFG:-}" ] && { mod_say "synchronisation désactivée, fichier existant conservé"; mod_ok; return $MOD_RC_OK; }
        mod_hint "CALYPSO_SYNC_MOBILE_CFG=1 pour réinstaller la version du dépôt"
        mod_fail "synchronisation désactivée et ${MOBILE_CFG:-<non défini>} absent"
        return $MOD_RC_FAIL
    fi
    if [ ! -r "${MOBILE_CFG_SRC:-}" ]; then
        mod_hint "attendu : \$QEMU_TREE/cfgs/mobile_group1.cfg — vérifiez QEMU_CFGS dans paths.env"
        mod_skip "configuration versionnée absente : ${MOBILE_CFG_SRC:-<non défini>}"
        return $MOD_RC_SKIP
    fi
    mod_ok
}

# Idempotence : « déjà fait » = la cible existe ET est identique à la source.
# Toute divergence (édition manuelle, ARFCN d'un run précédent) redéclenche la
# copie — c'est le point 1 du POURQUOI.
mod_mobile_cfg_status() {
    [ -r "${MOBILE_CFG:-}" ] || return 1
    [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" = "1" ] || return 0
    cmp -s "${MOBILE_CFG_SRC:-}" "$MOBILE_CFG" 2>/dev/null || return 1
    # Une surcharge d'ARFCN demandée rend la copie conforme… donc différente.
    [ -z "${CALYPSO_STICK_ARFCN:-}" ] || [ "${CALYPSO_STICK_ARFCN}" = "0" ]
}

mod_mobile_cfg_start() {
    if [ "${CALYPSO_SYNC_MOBILE_CFG:-1}" = "1" ]; then
        mkdir -p "$(dirname "$MOBILE_CFG")" 2>/dev/null
        if ! cp "$MOBILE_CFG_SRC" "$MOBILE_CFG" 2>/dev/null; then
            mod_hint "vérifiez les droits sur $(dirname "$MOBILE_CFG")"
            mod_fail "copie impossible : $MOBILE_CFG_SRC -> $MOBILE_CFG"
            return $MOD_RC_FAIL
        fi
        mod_say "installée : $MOBILE_CFG_SRC -> $MOBILE_CFG"
    fi

    # --- surcharge de l'ARFCN verrouillé (legacy L1047-1056) ----------------
    local a="${CALYPSO_STICK_ARFCN:-}"
    if [ -n "$a" ] && [ "$a" != "0" ]; then
        if grep -qE '^[[:space:]]*stick [0-9]+' "$MOBILE_CFG" 2>/dev/null; then
            sed -i "s/^\([[:space:]]*\)stick [0-9]\+/\1stick $a/" "$MOBILE_CFG"
        elif grep -qE '^[[:space:]]*no stick' "$MOBILE_CFG" 2>/dev/null; then
            sed -i "s/^\([[:space:]]*\)no stick/\1stick $a/" "$MOBILE_CFG"
        else
            mod_say "aucune directive stick/no stick dans $MOBILE_CFG : rien à surcharger"
        fi
        mod_say "stick demandé = $a, lu = $(_mcfg_stick)"
    fi

    # --- extraction pour le module 31 (cf. POURQUOI 2 : NON exportées) -------
    MOBILE_IMSI="$(_mcfg_imsi)"
    MOBILE_KI="$(_mcfg_ki)"
    mod_say "IMSI = ${MOBILE_IMSI:-<non trouvé>}"
    mod_say "Ki   = ${MOBILE_KI:+<présent, ${#MOBILE_KI} caractères>}${MOBILE_KI:-<non trouvé>}"
    mod_ok
}

# BARRIÈRE — critère observable : le fichier est en place, et il est EXPLOITABLE.
# « Présent » ne suffit pas : une configuration sans IMSI ni Ki laisse le module
# 31 sans rien à injecter, donc pas de Kc, donc un CIPHER MODE rejeté et une
# Location Update chiffrée qui échoue — trois couches plus loin.
_mcfg_ready() {
    [ -s "${MOBILE_CFG:-}" ] || return 1
    [ -n "$(_mcfg_imsi)" ] || return 1
    [ -n "$(_mcfg_ki)" ]   || return 1
    local a="${CALYPSO_STICK_ARFCN:-}"
    if [ -n "$a" ] && [ "$a" != "0" ] && grep -qE '^[[:space:]]*stick ' "$MOBILE_CFG" 2>/dev/null; then
        [ "$(_mcfg_stick)" = "$a" ] || return 1
    fi
    return 0
}

mod_mobile_cfg_wait() {
    if ! wait_until "${MOD_TIMEOUT[mobile-cfg]}" "configuration du mobile" _mcfg_ready; then
        if [ ! -s "${MOBILE_CFG:-}" ]; then
            mod_fail "$MOBILE_CFG absent ou vide après la copie"
        elif [ -n "${CALYPSO_STICK_ARFCN:-}" ] && [ "$(_mcfg_stick)" != "${CALYPSO_STICK_ARFCN}" ]; then
            mod_hint "vérifiez la ligne 'stick' de $MOBILE_CFG_SRC"
            mod_fail "ARFCN non appliqué : demandé ${CALYPSO_STICK_ARFCN}, lu $(_mcfg_stick)"
        else
            mod_hint "attendu dans $MOBILE_CFG : une ligne 'imsi <15 chiffres>' et une ligne 'ki comp128 <16 octets>'"
            mod_fail "IMSI ou Ki introuvable dans $MOBILE_CFG — le HLR ne pourra pas être alimenté"
        fi
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# Rien à arrêter : la configuration doit survivre au run (elle est relue à
# chaud par le VTY du mobile, et sert au diagnostic après coup).
mod_mobile_cfg_stop() { return 0; }
