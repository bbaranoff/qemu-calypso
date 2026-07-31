# =============================================================================
#  08-accel — drapeaux d'exécution de QEMU (horloge, accélérateur, gdb, halte)
# =============================================================================
#
#  RÔLE
#    Traduire quatre réglages d'émulation en quatre fragments de ligne de
#    commande QEMU, et un seul endroit où lire la traduction :
#      CALYPSO_ICOUNT     -> QEMU_ICOUNT_FLAG   (horloge virtuelle par instructions)
#      CALYPSO_MTTCG      -> QEMU_ACCEL_FLAG    (TCG multi-thread)
#      CALYPSO_GDB_PORT   -> QEMU_GDB_FLAG      (stub gdb)
#      CALYPSO_QEMU_HALT  -> QEMU_HALT_FLAG     (-S : démarrage halté)
#    Reprend run.sh.legacy #15 (L1355-1391) et #27 (L1590-1616).
#
#  PRÉREQUIS  : 07-coherence.
#  SUCCÈS     : les quatre drapeaux sont posés (éventuellement vides) et
#               mutuellement cohérents : MTTCG=1 => aucun -icount émis.
#  JOURNAL    : $LOG_DIR/mod/accel.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. ARBITRAGE MTTCG <-> icount : ON NE RÉÉCRIT PAS CALYPSO_ICOUNT.
#     Le legacy forçait `CALYPSO_ICOUNT=off` (L1374). Ici on laisse la variable
#     telle que l'opérateur ou environnement/ l'ont posée, et on n'ÉMET PAS le
#     drapeau. Le résultat pour QEMU est identique (pas de -icount sur la ligne
#     de commande), mais on ne ment plus : `env | grep CALYPSO_ICOUNT` continue
#     de montrer ce qui a été demandé, et le journal dit pourquoi ça n'a pas été
#     appliqué. C'est aussi la règle du bloc A : un module ne réaffecte jamais
#     une CALYPSO_*.
#
#  2. DÉFAUT D'ICOUNT : CE DÉPÔT DIT `auto`, LE LEGACY DISAIT `off`.
#     environnement/calypso.env L24 pose `: "${CALYPSO_ICOUNT:=auto}"` ;
#     run.sh.legacy L1363 posait `off` (« DEFAUT OFF 2026-06-03 »). Le module
#     n'arbitre pas : il applique ce que la configuration du dépôt dit et
#     l'écrit dans son journal. Trancher relève de environnement/calypso.env,
#     pas d'un module.
#
#  3. [CORRIGE 2026-07-30] CES DRAPEAUX ONT MAINTENANT UN CONSOMMATEUR.
#     Ce qui suit decrivait l'etat d'avant et est conserve pour l'historique :
#     40-qemu.sh (L54-57) construit sa ligne de commande lui-même et code en
#     dur `-gdb tcp::1234`, sans lire QEMU_*_FLAG. Tant que ce point n'est pas
#     arbitré, CALYPSO_ICOUNT / CALYPSO_MTTCG / CALYPSO_QEMU_HALT restent
#     inopérants sur le lancement réel. 40-qemu.sh est le fichier d'exemple du
#     dépôt : le modifier est hors périmètre ici. Le module pose donc les
#     drapeaux, les trace, et le dit — pour que l'écart soit visible plutôt que
#     silencieux.
# -----------------------------------------------------------------------------

MOD_REGISTER accel "Drapeaux d'exécution QEMU"
MOD_REQUIRED[accel]=1
MOD_DEPS[accel]="coherence"
MOD_PROFILES[accel]="calypso hybrid"
MOD_PURE[accel]=1
MOD_TIMEOUT[accel]=5

mod_accel_check() {
    # Lecture seule : la valeur d'icount est-elle une forme acceptée ?
    # Formes admises (legacy L1359-1362) : off | auto | shift=N[,...]
    local ic="${CALYPSO_ICOUNT:-auto}"
    case "$ic" in
        off|auto|shift=*) ;;
        *) mod_hint "valeurs : off | auto | shift=N,sleep=on[,align=off]"
           mod_fail "CALYPSO_ICOUNT='$ic' n'est pas une forme acceptée par QEMU"
           return $MOD_RC_FAIL ;;
    esac
    mod_ok
}

mod_accel_status() { return $MOD_RC_FAIL; }   # module pur

mod_accel_start() {
    local ic="${CALYPSO_ICOUNT:-auto}" mttcg="${CALYPSO_MTTCG:-0}"

    # --- accélérateur --------------------------------------------------------
    if [ "$mttcg" = "1" ]; then
        QEMU_ACCEL_FLAG="-accel tcg,thread=multi"
    else
        QEMU_ACCEL_FLAG=""
    fi

    # --- horloge -------------------------------------------------------------
    if [ "$mttcg" = "1" ] && [ "$ic" != "off" ]; then
        QEMU_ICOUNT_FLAG=""
        mod_say "MTTCG=1 : -icount NON émis bien que CALYPSO_ICOUNT=$ic (exclusifs, cf. POURQUOI 1)"
    elif [ "$ic" = "off" ]; then
        QEMU_ICOUNT_FLAG=""
    else
        QEMU_ICOUNT_FLAG="-icount $ic"
    fi

    # --- stub gdb (legacy L1595-1601) ---------------------------------------
    # Actif d'office : les outils d'injection et les tests s'y connectent sans
    # passer par le moniteur HMP. `tcp::PORT` écoute sur toutes les interfaces
    # du conteneur, donc joignable depuis l'hôte. CALYPSO_GDB_PORT vide = pas
    # de stub du tout.
    if [ -n "${CALYPSO_GDB_PORT:-${PORT_GDB:-1234}}" ]; then
        QEMU_GDB_FLAG="-gdb tcp::${CALYPSO_GDB_PORT:-${PORT_GDB:-1234}}"
    else
        QEMU_GDB_FLAG=""
    fi

    # --- démarrage halté (legacy L1602-1616) ---------------------------------
    if [ "${CALYPSO_QEMU_HALT:-0}" = "1" ]; then
        QEMU_HALT_FLAG="-S"
        mod_say "CALYPSO_QEMU_HALT=1 : QEMU démarre halté — il FAUT un 'continue' depuis gdb (cf. R7 en 07)"
    else
        QEMU_HALT_FLAG=""
    fi

    export QEMU_ICOUNT_FLAG QEMU_ACCEL_FLAG QEMU_GDB_FLAG QEMU_HALT_FLAG
    mod_say "QEMU_ICOUNT_FLAG = '${QEMU_ICOUNT_FLAG}'"
    mod_say "QEMU_ACCEL_FLAG  = '${QEMU_ACCEL_FLAG}'"
    mod_say "QEMU_GDB_FLAG    = '${QEMU_GDB_FLAG}'"
    mod_say "QEMU_HALT_FLAG   = '${QEMU_HALT_FLAG}'"
    # [2026-07-30] L'avertissement est LEVE : 40-qemu.sh consomme desormais ces
    # drapeaux (il les ajoute a sa ligne de commande et les journalise). En meme
    # temps, le defaut de calypso.env est passe de `auto` a `off`, pour que le
    # branchement soit neutre au lieu d'imposer `-icount auto` a tous les profils.
    mod_say "ces drapeaux sont APPLIQUES par 40-qemu.sh depuis le 2026-07-30"
    mod_ok
}

# BARRIÈRE — critère observable, immédiat : les quatre drapeaux sont définis
# (vides ou non, mais DÉFINIS : run.sh tourne sous `set -u`, une variable non
# posée ferait échouer le lancement de QEMU au lieu de le configurer), et
# l'exclusion MTTCG/icount est réellement matérialisée dans le résultat, pas
# seulement annoncée.
_accel_flags_defined() {
    [ -n "${QEMU_ICOUNT_FLAG+x}" ] && [ -n "${QEMU_ACCEL_FLAG+x}" ] \
        && [ -n "${QEMU_GDB_FLAG+x}" ] && [ -n "${QEMU_HALT_FLAG+x}" ]
}
_accel_mutex_ok() {
    case "${QEMU_ACCEL_FLAG:-}" in
        *thread=multi*) [ -z "${QEMU_ICOUNT_FLAG:-}" ] ;;
        *) return 0 ;;
    esac
}

mod_accel_wait() {
    if ! wait_until "${MOD_TIMEOUT[accel]}" "drapeaux QEMU posés" _accel_flags_defined; then
        mod_fail "drapeaux QEMU non posés — mod_accel_start n'a pas abouti"
        return $MOD_RC_FAIL
    fi
    if ! _accel_mutex_ok; then
        mod_hint "posez CALYPSO_ICOUNT=off, ou CALYPSO_MTTCG=0"
        mod_fail "MTTCG et icount sont tous deux actifs : '$QEMU_ACCEL_FLAG' + '$QEMU_ICOUNT_FLAG'"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_accel_stop() { return 0; }
