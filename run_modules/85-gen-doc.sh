# =============================================================================
#  85-gen-doc — génération de la documentation par la suite pytest
# =============================================================================
#
#  RÔLE (run.sh.legacy L2230-2296)
#      Lance pytest dans le conteneur, sur l'arbre de tests du dépôt, pour
#      produire les artefacts de rapport (report.md, test_results.md/.qmd, zip).
#      Verbosité et périmètre pilotés par CALYPSO_PYTEST_VERBOSITY et
#      CALYPSO_PYTEST_SCOPE.
#
#  PRÉREQUIS
#      La pile doit tourner : les tests observent l'émulation vivante. D'où
#      MOD_DEPS = qemu osmocon l2 — c'est CELA qui remplace le `sleep 60`.
#
#  CRITÈRE DE SUCCÈS
#      pytest a réellement démarré : sa bannière de session apparaît dans le
#      journal. On n'attend PAS la fin de la campagne — voir POURQUOI.
#
#  JOURNAL
#      $LOG_DIR/gen-doc.log ; artefacts dans $CALYPSO_TEST_OUT.
#
#  POURQUOI LE `sleep 60` DISPARAÎT SANS ÊTRE REMPLACÉ PAR UNE ATTENTE
#      Le legacy dormait 60 s « le temps que le pipeline se stabilise » (L2285).
#      Ce n'est pas une durée qu'il fallait attendre, c'est un ÉTAT : que QEMU,
#      osmocon et le client L2 soient passés. Le moteur le garantit déjà par le
#      graphe de dépendances — un module dont une dépendance a échoué n'est pas
#      joué du tout. Le sommeil devient donc une déclaration, pas une temporisation.
#
#  POURQUOI LA BARRIÈRE N'ATTEND PAS LA FIN DE PYTEST
#      Une campagne complète dure plusieurs minutes : bloquer le lancement de la
#      pile dessus n'aurait aucun sens. Ce qu'on veut distinguer, c'est « pytest
#      n'a pas démarré » (import cassé, répertoire de tests absent) de « pytest
#      tourne ». La bannière de session tranche cela en quelques secondes.
#
#  POURQUOI PAS DE `pip install`
#      Le legacy installait pytest et pycotap à la volée (L2238-2246). Un script
#      de lancement ne doit pas modifier l'environnement Python de la machine
#      sans le dire : on diagnostique, on ne répare pas dans le dos.
# -----------------------------------------------------------------------------
MOD_REGISTER gen-doc "Génération de la documentation (pytest)"
MOD_REQUIRED[gen-doc]=0
MOD_DEPS[gen-doc]="qemu osmocon l2"
MOD_PROFILES[gen-doc]="calypso hybrid"
MOD_TIMEOUT[gen-doc]=60
# Défaut OFF, à l'inverse du legacy qui lançait pytest à chaque démarrage : une
# campagne de tests n'a rien à faire dans le chemin nominal de mise en route.
MOD_ENABLED_IF[gen-doc]='[ "${CALYPSO_AUTO_GEN_DOC:-0}" = 1 ]'

: "${CALYPSO_PYTEST_VERBOSITY:=v}"
: "${CALYPSO_PYTEST_SCOPE:=default}"
: "${CALYPSO_TESTS_DIR:=${QEMU_TREE:-${QEMU_TREE}}/tests}"
: "${CALYPSO_TEST_OUT:=${LOG_DIR:-/root/calypso/logs}}"

_gendoc_pytest() {
    if command -v pytest >/dev/null 2>&1; then printf '%s' "pytest"
    elif python3 -c 'import pytest' >/dev/null 2>&1; then printf '%s' "python3 -m pytest"
    else return 1; fi
}
_gendoc_pat() { printf '%s' "pytest"; }

mod_gen_doc_check() {
    [ -d "$CALYPSO_TESTS_DIR" ] || {
        mod_hint "posez CALYPSO_TESTS_DIR vers l'arbre de tests"
        mod_fail "répertoire de tests introuvable : $CALYPSO_TESTS_DIR"
        return $MOD_RC_FAIL; }
    _gendoc_pytest >/dev/null || {
        mod_hint "installez-le vous-même : python3 -m pip install pytest pycotap"
        mod_fail "pytest introuvable (ni exécutable, ni module python3)"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gen_doc_status() { have_proc "$(_gendoc_pat)"; }

mod_gen_doc_start() {
    local log="${LOG_DIR}/gen-doc.log" py verb target ignores
    py="$(_gendoc_pytest)"
    mkdir -p "$LOG_DIR" "$RUN_DIR" "$CALYPSO_TEST_OUT" 2>/dev/null
    : > "$log"

    case "$CALYPSO_PYTEST_VERBOSITY" in
        q)   verb="-q --no-header" ;;
        vv)  verb="-vv --tb=long --color=yes" ;;
        vvv) verb="-vvv --tb=long --color=yes --log-cli-level=DEBUG" ;;
        *)   verb="-v --tb=short --color=yes" ;;
    esac
    case "$CALYPSO_PYTEST_SCOPE" in
        smoke)   target="test_run_all_modes.py"; ignores="" ;;
        all)     target="";                      ignores="" ;;
        calypso) target="calypso/";              ignores="" ;;
        *)       target=""
                 ignores="--ignore=functional --ignore=guest-debug --ignore=qemu-iotests
                          --ignore=qtest --ignore=unit --ignore=tcg --ignore=migration
                          --ignore=vm --ignore=avocado --ignore=fp" ;;
    esac
    mod_say "pytest   : $py $verb (scope=$CALYPSO_PYTEST_SCOPE)"
    mod_say "tests    : $CALYPSO_TESTS_DIR — artefacts dans $CALYPSO_TEST_OUT"

    (
        cd "$CALYPSO_TESTS_DIR" || exit 1
        CALYPSO_TEST_OUT="$CALYPSO_TEST_OUT" \
        CALYPSO_REPO="${QEMU_TREE:-${QEMU_TREE}}" \
        CALYPSO_HOST_ROOT="${HOME:-/root}" \
        CALYPSO_MODE_TAG="${CALYPSO_MODE:-}" \
            $py $verb $ignores $target
    ) >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR}/gen-doc.pid"
    mod_ok
}

# BARRIÈRE — pytest a-t-il démarré ? Sa bannière « test session starts » est
# émise dès la collecte ; son absence signe un échec d'import ou un répertoire
# de tests vide, cas que le legacy laissait passer sans un mot.
mod_gen_doc_wait() {
    local log="${LOG_DIR}/gen-doc.log" pid
    pid="$(cat "${RUN_DIR}/gen-doc.pid" 2>/dev/null || echo 0)"

    if ! wait_until "${MOD_TIMEOUT[gen-doc]}" "démarrage de pytest" \
                    log_has "$log" "test session starts"; then
        # Une campagne très courte peut s'être terminée avant notre première
        # sonde : dans ce cas le journal n'est pas vide et le processus est parti.
        if [ -s "$log" ] && ! kill -0 "$pid" 2>/dev/null; then
            mod_say "pytest a terminé avant la première sonde — voir $log"
            mod_ok; return $MOD_RC_OK
        fi
        mod_hint "lisez $log : collecte impossible ou dépendance de test manquante"
        mod_fail "pytest n'a pas produit de session de test"
        return $MOD_RC_FAIL
    fi
    mod_say "pytest en cours — les artefacts arriveront dans $CALYPSO_TEST_OUT"
    mod_ok
}

mod_gen_doc_stop() {
    local pid; pid="$(cat "${RUN_DIR}/gen-doc.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    rm -f "${RUN_DIR}/gen-doc.pid"
    return 0
}
