# =============================================================================
#  95-summary — identité du build et résumé de configuration
# =============================================================================
#
#  RÔLE (run.sh.legacy L2331-2414)
#      Deux relevés de diagnostic, sans effet de bord :
#        1. BUILD IDENTITY — révision git, état de l'arbre, dates du binaire et
#           de la source du C54x, et surtout les marqueurs « BUILD-IDENT
#           decoder-fixes: » extraits du BINAIRE. Ces chaînes vivent dans le
#           format compilé, pas dans un commentaire : si l'une disparaît, c'est
#           que le correctif correspondant n'est PAS dans le binaire qui tourne.
#           C'est l'attribution causale d'un résultat à un build donné.
#        2. RÉSUMÉ D'ENVIRONNEMENT — les variables dont dépend l'interprétation
#           d'une mesure (icount/MTTCG en tête : sous MTTCG rien n'est reproductible,
#           donc rien n'est citable).
#
#  PRÉREQUIS
#      Aucun. MOD_PURE=1 : aucune écriture, donc joué même en --dry-run — c'est
#      justement en simulation qu'on veut savoir sur quel build on raisonne.
#
#  CRITÈRE DE SUCCÈS
#      La ligne BUILD-IDENT est présente dans le binaire. Son absence n'arrête
#      rien (MOD_REQUIRED=0) : elle signale un build ancien, ce qui est un fait
#      utile, pas une panne.
#
#  JOURNAL
#      $LOG_DIR/mod/summary.log — tout passe par mod_say : la console appartient
#      à run.sh, un module n'y écrit jamais.
# -----------------------------------------------------------------------------
MOD_REGISTER summary "Identité du build et résumé"
MOD_REQUIRED[summary]=0
MOD_PROFILES[summary]="calypso faketrx hybrid core"
MOD_PURE[summary]=1
MOD_TIMEOUT[summary]=10

# 1 = l'absence de la ligne BUILD-IDENT (ou un binaire illisible) fait ECHOUER ce
# module. Defaut 0 — et c'est un correctif, pas une preference : run.sh sort en
# code 1 des que nb_fail > 0, quel que soit MOD_REQUIRED. Un `mod_fail` ici
# rendait donc `./run.sh` non nul alors que TOUTE la pile etait montee, du seul
# fait que le binaire datait d'avant l'ajout des marqueurs. Un module
# d'identification informe, il ne prononce pas le verdict du lancement.
: "${CALYPSO_REQUIRE_BUILD_IDENT:=0}"

: "${CALYPSO_C54X_SRC:=${QEMU_TREE:-${QEMU_TREE}}/hw/arm/calypso/calypso_c54x.c}"

# Les cinq marqueurs attendus dans la ligne BUILD-IDENT, sous la forme
# « motif|libellé ». Un motif absent = correctif absent du binaire.
_summary_markers() {
    printf '%s\n' \
        "F1xx-FIRS-catch=REMOVED|F1xx FIRS catch retiré" \
        "L3609-src-dst=FIXED|L3609 inversion src/dst corrigée" \
        "F-AUDIT-v5=max-min-cmpl-rnd-roltc-fixed|F-AUDIT v5 (max/min/cmpl/rnd/roltc)" \
        "F2xx-ALU-block=ADDED|F2xx bloc ALU (binutils-strict bit9=src bit8=dst)" \
        "F3xx-INTR-mis-REMOVED|F3xx INTR mal décodé retiré (vrai INTR=F7C0)"
}

mod_summary_check() { mod_ok; }
mod_summary_status() { return $MOD_RC_FAIL; }   # module pur : jamais « déjà démarré »

mod_summary_start() {
    mod_say "===== IDENTITÉ DU BUILD ====="
    if command -v git >/dev/null 2>&1 && [ -d "${QEMU_TREE}/.git" ]; then
        mod_say "  révision git    : $(git -C "$QEMU_TREE" rev-parse --short HEAD 2>/dev/null || echo inconnue)"
        mod_say "  arbre modifié   : $(git -C "$QEMU_TREE" diff --stat 2>/dev/null | tail -1)"
    fi
    mod_say "  binaire QEMU    : ${QEMU_BIN:-<non défini>}"
    mod_say "  date du binaire : $(stat -c %y "${QEMU_BIN:-/nonexistent}" 2>/dev/null | cut -d. -f1)"
    mod_say "  date source C54x: $(stat -c %y "$CALYPSO_C54X_SRC" 2>/dev/null | cut -d. -f1)"

    local rc=$MOD_RC_OK line m pat lab
    if [ -x "${QEMU_BIN:-}" ]; then
        line="$(strings "$QEMU_BIN" 2>/dev/null | grep 'BUILD-IDENT decoder-fixes:' | head -1)"
        if [ -n "$line" ]; then
            while IFS= read -r m; do
                pat="${m%%|*}"; lab="${m##*|}"
                case "$line" in
                    *"$pat"*) mod_say "  [x] $lab" ;;
                    *)        mod_say "  [ ] $lab — ABSENT du binaire" ;;
                esac
            done < <(_summary_markers)
        else
            mod_say "  ! ligne BUILD-IDENT absente du binaire (= build ancien)"
            mod_say "    pour disposer de l'attribution build -> résultat :"
            mod_say "    ninja -C build qemu-system-arm"
            if [ "$CALYPSO_REQUIRE_BUILD_IDENT" = 1 ]; then
                mod_hint "recompilez, ou retirez CALYPSO_REQUIRE_BUILD_IDENT=1"
                mod_fail "impossible d'attribuer ce build : marqueurs BUILD-IDENT absents"
                rc=$MOD_RC_FAIL
            fi
        fi
    else
        mod_say "  ! binaire QEMU non exécutable : pas d'identité de build"
        if [ "$CALYPSO_REQUIRE_BUILD_IDENT" = 1 ]; then
            mod_fail "binaire QEMU illisible : ${QEMU_BIN:-<non défini>}"
            rc=$MOD_RC_FAIL
        fi
    fi

    mod_say "===== RÉSUMÉ D'ENVIRONNEMENT ====="
    mod_say "  CALYPSO_MODE            = ${CALYPSO_MODE:-}   (profil d'émulation)"
    mod_say "  CALYPSO_PIPELINE        = ${CALYPSO_PIPELINE:-(non posé — défaut full-grgsm)}"
    mod_say "  CALYPSO_SHUNT_LEGIT     = ${CALYPSO_SHUNT_LEGIT:-0}"
    mod_say "  CALYPSO_SHUNT_NO_CANNED = ${CALYPSO_SHUNT_NO_CANNED:-0}"
    mod_say "  CALYPSO_SHUNT_REAL_FB   = ${CALYPSO_SHUNT_REAL_FB:-0}"
    mod_say "  CALYPSO_BSP_DARAM_ADDR  = ${CALYPSO_BSP_DARAM_ADDR:-}"
    mod_say "  CALYPSO_ICOUNT          = ${CALYPSO_ICOUNT:-}  (drapeau : ${QEMU_ICOUNT_FLAG:-aucun})"
    if [ "${CALYPSO_MTTCG:-0}" = 1 ]; then
        mod_say "  CALYPSO_MTTCG           = 1  ! NON DÉTERMINISTE — aucune mesure prise ici n'est citable"
    else
        mod_say "  CALYPSO_MTTCG           = 0  (déterministe sous icount)"
    fi
    mod_say "  CALYPSO_L2_CLIENT       = ${CALYPSO_L2_CLIENT:-mobile}"
    mod_say "  CALYPSO_GRGSM_DECODER   = ${CALYPSO_GRGSM_DECODER:-si-bridge}"
    mod_say "  LOG_DIR / RUN_DIR       = ${LOG_DIR:-} / ${RUN_DIR:-}"
    mod_say "  nombre de CALYPSO_*     = $(compgen -v | grep -c '^CALYPSO_')"

    return $rc
}
