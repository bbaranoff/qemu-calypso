# =============================================================================
#  16-fwsyms — adresses des symboles firmware lues dans l'ELF
# =============================================================================
#
#  RÔLE
#    Dériver de l'ELF du firmware les deux adresses dont le modèle a besoin :
#      l1s        -> CALYPSO_L1S_FN_ADDR
#      last_rach  -> CALYPSO_LAST_RACH_FN_ADDR
#    Reprend run.sh.legacy #31 (L1794-1810).
#
#  PRÉREQUIS  : 00-prereqs (FIRMWARE_ELF existe).
#  SUCCÈS     : les deux variables contiennent une adresse hexadécimale.
#  JOURNAL    : $LOG_DIR/mod/fwsyms.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI CES DEUX SYMBOLES, ET PAS D'AUTRES.
#     Ce sont les seuls que le modèle lit par getenv :
#     calypso_dsp_helper.c L124 (CALYPSO_L1S_FN_ADDR) et L147
#     (CALYPSO_LAST_RACH_FN_ADDR). Le shunt s'en sert pour gater la
#     présentation AGCH de l'IMMEDIATE ASSIGNMENT.
#
#  2. POURQUOI C'EST DÉRIVÉ ET NON CONFIGURÉ.
#     Ces adresses BOUGENT à chaque recompilation du firmware (l'ajout de la
#     pile IrDA a déplacé tout le segment). Périmées, elles ne provoquent
#     aucune erreur : le modèle lit 0, l'AGCH n'est jamais dispatchée, le
#     mobile refait des RACH en boucle et aucune Location Update n'aboutit.
#     Le symptôme est à trois couches du défaut. Les dériver de l'ELF chargé
#     supprime la classe d'erreur entière.
#
#  3. POURQUOI UN nm ABSENT EST UN « SKIP » ET NON UN ÉCHEC.
#     Sans nm, le modèle garde ses adresses codées en dur : la pile démarre,
#     seule l'AGCH est en péril. Un échec ici ferait sortir ./run.sh en code 1
#     pour une chaîne qui, elle, fonctionne. En revanche, si nm EST là et que
#     les symboles manquent, c'est un vrai défaut de l'ELF fourni : là, on
#     échoue (sans bloquer, le module n'étant pas obligatoire).
# -----------------------------------------------------------------------------

MOD_REGISTER fwsyms "Adresses des symboles firmware"
MOD_REQUIRED[fwsyms]=0
MOD_DEPS[fwsyms]="prereqs"
MOD_PROFILES[fwsyms]="calypso hybrid"
MOD_PURE[fwsyms]=1
MOD_TIMEOUT[fwsyms]=15

_fwsyms_nm() {
    local c
    for c in "${CALYPSO_NM:-}" arm-elf-nm arm-none-eabi-nm /root/gnuarm/install/bin/arm-elf-nm nm; do
        [ -n "$c" ] || continue
        if command -v "$c" >/dev/null 2>&1; then command -v "$c"; return 0; fi
        [ -x "$c" ] && { printf '%s' "$c"; return 0; }
    done
    return 1
}

_fwsyms_addr() {   # $1 = nom du symbole
    "$(_fwsyms_nm)" "${FIRMWARE_ELF}" 2>/dev/null \
        | awk -v s="$1" '$3==s {print "0x" $1; exit}'
}

mod_fwsyms_check() {
    [ -r "${FIRMWARE_ELF:-}" ] || {
        mod_hint "réglez FIRMWARE_ELF dans environnement/paths.env"
        mod_fail "ELF du firmware illisible : ${FIRMWARE_ELF:-<non défini>}"
        return $MOD_RC_FAIL; }
    if ! _fwsyms_nm >/dev/null; then
        mod_hint "installez binutils (arm-none-eabi-nm ou nm) pour supprimer ce risque"
        mod_skip "nm introuvable : le modèle gardera ses adresses par défaut (AGCH en péril si le firmware a été recompilé)"
        return $MOD_RC_SKIP
    fi
    mod_ok
}

mod_fwsyms_status() { return $MOD_RC_FAIL; }   # module pur

mod_fwsyms_start() {
    local l1s lr
    l1s="$(_fwsyms_addr l1s)"
    lr="$(_fwsyms_addr last_rach)"

    if [ -z "$l1s" ] || [ -z "$lr" ]; then
        mod_hint "recompilez le firmware, ou vérifiez que FIRMWARE_ELF est bien layer1.highram.elf (non strippé)"
        mod_fail "symboles absents de ${FIRMWARE_ELF} : l1s=${l1s:-introuvable} last_rach=${lr:-introuvable}"
        return $MOD_RC_FAIL
    fi

    export CALYPSO_L1S_FN_ADDR="$l1s"
    export CALYPSO_LAST_RACH_FN_ADDR="$lr"
    mod_say "nm        = $(_fwsyms_nm)"
    mod_say "ELF       = $FIRMWARE_ELF"
    mod_say "l1s       = $l1s      (calypso_dsp_helper.c:124)"
    mod_say "last_rach = $lr      (calypso_dsp_helper.c:147)"
    mod_ok
}

# BARRIÈRE — critère observable : les deux variables sont exportées et ont la
# FORME d'une adresse. Une chaîne vide ou tronquée serait lue comme 0 par le
# modèle, c'est-à-dire exactement le cas qu'on cherche à éliminer ; on refuse
# donc tout ce qui n'est pas 0x suivi de chiffres hexadécimaux.
_fwsyms_ok() {
    case "${CALYPSO_L1S_FN_ADDR:-}"       in 0x[0-9a-fA-F]*) ;; *) return 1;; esac
    case "${CALYPSO_LAST_RACH_FN_ADDR:-}" in 0x[0-9a-fA-F]*) ;; *) return 1;; esac
    return 0
}

mod_fwsyms_wait() {
    if ! wait_until "${MOD_TIMEOUT[fwsyms]}" "adresses firmware" _fwsyms_ok; then
        mod_hint "sans ces adresses : AGCH jamais dispatchée -> RACH en boucle -> pas de Location Update"
        mod_fail "adresses mal formées : l1s='${CALYPSO_L1S_FN_ADDR:-}' last_rach='${CALYPSO_LAST_RACH_FN_ADDR:-}'"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_fwsyms_stop() { return 0; }
