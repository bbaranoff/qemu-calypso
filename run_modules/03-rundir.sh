# =============================================================================
#  03-rundir — répertoire d'exécution (PID, sockets, verrous)
# =============================================================================
#
#  RÔLE
#      Garantir l'existence et l'utilisabilité de RUN_DIR, le seul endroit où la
#      pile écrit son ÉTAT VOLATIL : fichiers PID, sockets de rendez-vous
#      (qemu-monitor.sock), verrous. C'est le point d'appui de tout le reste —
#      40-qemu y écrit qemu.pid et y ouvre le socket de son moniteur, et sa
#      barrière relit ce PID pour distinguer « QEMU s'est arrêté » de « QEMU
#      tourne mais le DSP est figé ». Un RUN_DIR absent ou non inscriptible fait
#      donc échouer les modules suivants sur un symptôme sans rapport avec la
#      cause.
#
#      PÉRIMÈTRE. Ce module ne s'occupe PAS des journaux ni des captures :
#      09-logs en est propriétaire (arborescence, archivage, horodatage, liens).
#      Ici, et uniquement ici, l'état volatil.
#
#      paths.env place RUN_DIR sous ${XDG_RUNTIME_DIR:-/tmp}/calypso. L'ancien
#      lancement écrivait en dur dans /root : un tiers n'a aucune raison d'y
#      écrire, et deux piles ne pouvaient pas cohabiter.
#
#  PRÉREQUIS
#      profil (RUN_DIR est résolu par environnement/paths.env).
#
#  CRITÈRE DE SUCCÈS
#      BARRIÈRE — une écriture RÉELLE aboutit dans RUN_DIR (création puis
#      suppression d'un témoin). Tester les droits ne suffit pas : un montage
#      passé en lecture seule, ou un volume plein, satisfait `-w` et échoue à
#      l'écriture.
#
#  JOURNAL
#      $LOG_DIR/mod/rundir.log : chemin retenu et restes constatés.
# -----------------------------------------------------------------------------
MOD_REGISTER rundir "Répertoire d'exécution"
MOD_REQUIRED[rundir]=1
MOD_DEPS[rundir]="profil"
MOD_PROFILES[rundir]="calypso faketrx hybrid core"
MOD_TIMEOUT[rundir]=15

mod_rundir_check() {
    local d="${RUN_DIR:-}" parent
    [ -n "$d" ] || { mod_hint "RUN_DIR est posé par environnement/paths.env"
                     mod_fail "RUN_DIR non défini"
                     return $MOD_RC_FAIL; }
    if [ -e "$d" ] && [ ! -d "$d" ]; then
        mod_hint "rm -f $d   (ou choisissez un autre RUN_DIR)"
        mod_fail "$d existe et n'est pas un répertoire"
        return $MOD_RC_FAIL
    fi
    if [ ! -d "$d" ]; then
        parent="$(dirname "$d")"
        while [ ! -d "$parent" ] && [ "$parent" != "/" ]; do parent="$(dirname "$parent")"; done
        if [ ! -w "$parent" ]; then
            mod_hint "choisissez un emplacement inscriptible :  RUN_DIR=\$HOME/calypso ./start-oqc.sh"
            mod_fail "impossible de créer $d : $parent n'est pas inscriptible"
            return $MOD_RC_FAIL
        fi
    fi
    mod_ok
}

# « Déjà fait » = le répertoire est là et utilisable. Ce module ne laisse
# derrière lui aucun processus, seulement un état constaté.
mod_rundir_status() { [ -d "${RUN_DIR:-}" ] && [ -w "${RUN_DIR:-}" ]; }

mod_rundir_start() {
    mkdir -p "${RUN_DIR}" 2>/dev/null || { mod_fail "création impossible : ${RUN_DIR}"; return $MOD_RC_FAIL; }
    chmod 0755 "${RUN_DIR}" 2>/dev/null
    mod_say "répertoire d'exécution : ${RUN_DIR}"
    local f n=0
    for f in "${RUN_DIR}"/*.pid "${RUN_DIR}"/*.sock; do
        [ -e "$f" ] && { mod_say "reste du run précédent : $f"; n=$((n+1)); }
    done
    [ "$n" -gt 0 ] && mod_say "$n reste(s) — 04-restes puis 10-teardown s'en chargent"
    mod_ok
}

_rundir_ecriture_ok() {
    local t="${RUN_DIR}/.temoin.$$"
    : > "$t" 2>/dev/null || return 1
    rm -f "$t" 2>/dev/null
    return 0
}

mod_rundir_wait() {
    if ! wait_until "${MOD_TIMEOUT[rundir]}" "écriture dans ${RUN_DIR}" _rundir_ecriture_ok; then
        mod_hint "montage en lecture seule ou volume plein :  df -h ${RUN_DIR}"
        mod_fail "${RUN_DIR} existe mais l'écriture y échoue"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# On ne supprime pas le répertoire : il peut porter des traces utiles au
# diagnostic. Seuls partent les fichiers PID dont le processus est mort.
mod_rundir_stop() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        kill -0 "$pid" 2>/dev/null || rm -f "$f"
    done
    return 0
}
