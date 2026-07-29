# =============================================================================
#  04-restes — extinction DOUCE des processus enregistrés au run précédent
# =============================================================================
#
#  RÔLE
#      Terminer proprement les processus que CE moteur a lui-même enregistrés :
#      chaque module qui lance quelque chose écrit son PID dans
#      $RUN_DIR/<slug>.pid. Ce module leur envoie TERM, attend leur mort
#      observée, et n'emploie KILL qu'en dernier recours.
#
#      POURQUOI IL EXISTE À CÔTÉ DE 10-teardown, ET CE QU'IL NE FAIT PAS.
#      10-teardown balaie la machine par MOTIFS (noms de processus, ports,
#      sockets) et frappe au `kill -9` : c'est le filet, et il est indispensable
#      — un processus dont le PID n'a jamais été enregistré n'a aucune autre
#      chance d'être vu. Mais un -9 sur QEMU laisse son fichier de moniteur, ses
#      tampons non vidés et un journal tronqué en plein milieu : le run suivant
#      redémarre sur des restes, et le run précédent devient illisible.
#      04-restes passe donc AVANT, sur la liste EXACTE des PID connus, avec un
#      signal que les processus peuvent traiter. Quand il a fait son travail,
#      10-teardown ne trouve plus rien à tuer — et ce qu'il trouve encore est,
#      par construction, ce qui n'appartenait pas à ce moteur.
#      Ce module ne touche à AUCUN port, AUCUNE socket, AUCUN motif de ligne de
#      commande : c'est le périmètre de 10-teardown, et le dédoubler ferait deux
#      vérités concurrentes.
#
#      Il ne fait jamais `pkill python3` ni `pkill -f mobile` : des motifs non
#      ancrés qui, sur une machine partagée, tuent ce qui n'appartient pas à la
#      pile. C'est le défaut de l'ancien nettoyage, pas un modèle à reprendre.
#
#  PRÉREQUIS
#      rundir (les fichiers PID vivent dans RUN_DIR).
#      Neutralisable par NO_STARTUP_STOP=1 — même nom que dans l'ancien
#      start-direct.sh, pour ne pas dérouter qui le connaît.
#
#  CRITÈRE DE SUCCÈS
#      BARRIÈRE — plus AUCUN PID enregistré dans RUN_DIR ne répond à `kill -0`.
#      Sondé, avec plafond de temps ; jamais une attente au jugé. L'ancien code
#      envoyait un signal et passait à la suite sans jamais vérifier la mort.
#
#  JOURNAL
#      $LOG_DIR/mod/restes.log : PID trouvés avec leur ligne de commande, ce qui
#      a résisté au TERM, et les fichiers PID retirés.
# -----------------------------------------------------------------------------
MOD_REGISTER restes "Extinction des processus enregistrés"
MOD_REQUIRED[restes]=0
MOD_DEPS[restes]="rundir"
MOD_PROFILES[restes]="calypso faketrx hybrid core"
MOD_TIMEOUT[restes]=20
MOD_ENABLED_IF[restes]='[ "${NO_STARTUP_STOP:-0}" != 1 ]'

# PID enregistrés par les modules et encore vivants. Liste EXACTE : aucune
# heuristique, aucun motif — on ne tue que ce qu'on a nous-mêmes lancé.
_res_pids() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        [ "$pid" -gt 0 ] 2>/dev/null || continue
        [ "$pid" = "$$" ] && continue
        kill -0 "$pid" 2>/dev/null && printf '%s\n' "$pid"
    done
}
_res_vide() { [ -z "$(_res_pids)" ]; }

_res_cmdline() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | cut -c1-140; }

# Signal au GROUPE d'abord : un module lance son processus en arrière-plan, et
# celui-ci a des enfants (QEMU et son garde-fou de journal, osmocon et son pty).
# Ne viser que le père laisserait les enfants orphelins mais vivants — c'est
# ainsi qu'un « QEMU tué » continuait de tenir son port.
_res_signal() {
    local sig="$1" p
    for p in $(_res_pids); do
        kill "-$sig" -- "-$p" 2>/dev/null || kill "-$sig" "$p" 2>/dev/null
    done
    return 0
}

_res_menage() {
    local f pid
    for f in "${RUN_DIR:-/nonexistent}"/*.pid; do
        [ -f "$f" ] || continue
        pid="$(cat "$f" 2>/dev/null || echo 0)"
        kill -0 "$pid" 2>/dev/null || { rm -f "$f"; mod_say "fichier PID retiré : $f"; }
    done
    return 0
}

mod_restes_check() {
    command -v kill >/dev/null 2>&1 || { mod_fail "commande kill introuvable"; return $MOD_RC_FAIL; }
    mod_ok
}

# Sémantique assumée : « déjà fait » = il n'y a rien à éteindre. run.sh affiche
# alors SKIP, ce qui est l'information juste.
mod_restes_status() { _res_vide; }

mod_restes_start() {
    local liste p
    liste="$(_res_pids | tr '\n' ' ')"
    if [ -z "${liste// /}" ]; then
        _res_menage
        mod_already "aucun processus enregistré n'a survécu au run précédent"
        return $MOD_RC_ALREADY
    fi
    for p in $liste; do mod_say "encore vivant : $p  $(_res_cmdline "$p")"; done
    _res_signal TERM
    mod_ok
}

mod_restes_wait() {
    local moitie=$(( ${MOD_TIMEOUT[restes]} / 2 ))
    [ "$moitie" -lt 2 ] && moitie=2

    if ! wait_until "$moitie" "extinction douce" _res_vide; then
        mod_say "toujours vivants après TERM : $(_res_pids | tr '\n' ' ') — passage en KILL"
        _res_signal KILL
        if ! wait_until "$moitie" "extinction forcée" _res_vide; then
            mod_hint "identifiez le tenace :  ps -o pid,ppid,stat,cmd -p \$(cat ${RUN_DIR:-/tmp/calypso}/*.pid | tr '\\n' ',')"
            mod_fail "processus enregistrés impossibles à arrêter : $(_res_pids | tr '\n' ' ')"
            return $MOD_RC_FAIL
        fi
    fi
    _res_menage
    mod_ok
}

# À l'arrêt, run.sh joue le plan en ordre inverse : ce module passe donc APRÈS
# les _stop de chaque composant, et ramasse ceux qui n'ont pas obéi.
mod_restes_stop() {
    _res_vide && { _res_menage; return 0; }
    _res_signal TERM
    wait_until 5 "extinction douce" _res_vide || { _res_signal KILL; wait_until 5 "extinction forcée" _res_vide; }
    _res_menage
    return 0
}
