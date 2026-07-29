# =============================================================================
#  15-fifo-iq — tube nommé I/Q du shunt
# =============================================================================
#
#  RÔLE
#      Créer, avant QEMU, le tube nommé par lequel le modèle publie l'I/Q vu du
#      shunt (CALYPSO_SHUNT_IQ_CFILE, fc32). Il doit exister AVANT le lancement :
#      QEMU l'ouvre à l'initialisation, et un chemin absent le fait basculer
#      silencieusement en « pas de sortie I/Q » — ce qui se diagnostique très
#      loin en aval, sur une FFT vide ou un gr-gsm sans entrée.
#
#      CE QUI CHANGE PAR RAPPORT À L'ANCIEN CODE. start-direct.sh devait deviner
#      ce chemin avant que start-clean.sh ne source la configuration : il le
#      ré-extrayait au `sed` de calypso.env, avec un motif qui ne reconnaît pas
#      la forme réelle `: "${CALYPSO_SHUNT_IQ_CFILE:=…}"`. Il retombait donc
#      toujours sur le défaut littéral, et ne marchait que par coïncidence de
#      valeur. Ici la variable est déjà RÉSOLUE par environnement/load.env :
#      plus aucune analyse de texte.
#
#  PRÉREQUIS
#      rundir. Module inactif si CALYPSO_SHUNT_IQ_CFILE est vide (profil sans
#      publication I/Q).
#
#  CRITÈRE DE SUCCÈS
#      BARRIÈRE — le chemin est un TUBE (test -p) et il est inscriptible
#      (access(2), via test -w). On n'ouvre JAMAIS le tube pour vérifier : une
#      ouverture en écriture sans lecteur bloque indéfiniment — ce serait
#      transformer la barrière en interblocage.
#
#  JOURNAL
#      $LOG_DIR/mod/fifo-iq.log : chemin, mode, et rappel des fichiers
#      d'enregistrement associés (qui, eux, ne sont pas créés ici).
# -----------------------------------------------------------------------------
MOD_REGISTER fifo-iq "Tube I/Q du shunt"
MOD_REQUIRED[fifo-iq]=0
MOD_DEPS[fifo-iq]="rundir"
MOD_PROFILES[fifo-iq]="calypso hybrid"
MOD_TIMEOUT[fifo-iq]=10
MOD_ENABLED_IF[fifo-iq]='[ -n "${CALYPSO_SHUNT_IQ_CFILE:-}" ]'

: "${FIFO_IQ_MODE:=0666}"

mod_fifo_iq_check() {
    local f="${CALYPSO_SHUNT_IQ_CFILE:-}" d
    d="$(dirname "$f")"

    if [ -e "$f" ] && [ ! -p "$f" ]; then
        mod_hint "c'est probablement un enregistrement laissé par un run précédent :  rm -f $f"
        mod_fail "$f existe mais n'est pas un tube nommé"
        return $MOD_RC_FAIL
    fi
    if [ ! -d "$d" ]; then
        mod_hint "créez-le, ou changez CALYPSO_SHUNT_IQ_CFILE (environnement/calypso.env)"
        mod_fail "répertoire du tube absent : $d"
        return $MOD_RC_FAIL
    fi
    if [ ! -w "$d" ]; then
        mod_fail "répertoire du tube non inscriptible : $d"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_fifo_iq_status() { [ -p "${CALYPSO_SHUNT_IQ_CFILE:-}" ] && [ -w "${CALYPSO_SHUNT_IQ_CFILE:-}" ]; }

mod_fifo_iq_start() {
    local f="${CALYPSO_SHUNT_IQ_CFILE}"
    if [ ! -p "$f" ]; then
        mkfifo -m "$FIFO_IQ_MODE" "$f" 2>/dev/null || {
            mod_hint "vérifiez les droits sur $(dirname "$f") — /dev/shm est le défaut"
            mod_fail "création du tube impossible : $f"
            return $MOD_RC_FAIL; }
        mod_say "tube créé : $f (mode $FIFO_IQ_MODE)"
    else
        chmod "$FIFO_IQ_MODE" "$f" 2>/dev/null
        mod_say "tube déjà présent : $f"
    fi
    # Ces deux-là sont des FICHIERS d'enregistrement, écrits par le modèle
    # lui-même : on ne les crée pas, on rappelle seulement où ils atterrissent.
    mod_say "enregistrement DL : ${CALYPSO_SHUNT_IQ_RECORD:-<désactivé>}"
    mod_say "enregistrement UL : ${CALYPSO_UL_IQ_RECORD:-<désactivé>}"
    mod_ok
}

_fifo_iq_pret() { [ -p "${CALYPSO_SHUNT_IQ_CFILE}" ] && [ -w "${CALYPSO_SHUNT_IQ_CFILE}" ]; }

mod_fifo_iq_wait() {
    if ! wait_until "${MOD_TIMEOUT[fifo-iq]}" "tube I/Q" _fifo_iq_pret; then
        mod_hint "ls -l ${CALYPSO_SHUNT_IQ_CFILE} — le fichier doit apparaître avec un « p » en tête"
        mod_fail "le tube ${CALYPSO_SHUNT_IQ_CFILE} n'est pas exploitable"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# On retire le tube (il n'a de sens que pendant un run) mais PAS les
# enregistrements : ce sont des données, parfois les seules du run.
mod_fifo_iq_stop() {
    local f="${CALYPSO_SHUNT_IQ_CFILE:-}"
    [ -n "$f" ] && [ -p "$f" ] && rm -f "$f"
    return 0
}
