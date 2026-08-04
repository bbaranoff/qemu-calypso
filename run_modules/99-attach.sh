# =============================================================================
#  99-attach — fenêtre par défaut et reprise en main de la session
# =============================================================================
#
#  RÔLE (run.sh.legacy L2416-2426)
#      Le legacy terminait par `tmux select-window -t all` (repli sur `qemu`)
#      puis `exec tmux attach`. Ce module fait la première moitié — voir plus bas
#      pourquoi la seconde ne peut PAS vivre dans un module — et dépose la
#      commande de reprise en main là où on la retrouvera.
#
#  PRÉREQUIS
#      Une session tmux (module 35).
#
#  CRITÈRE DE SUCCÈS
#      La session existe et sa fenêtre courante est bien celle qu'on a demandée.
#
#  JOURNAL
#      $LOG_DIR/mod/attach.log
#
#  POURQUOI CE MODULE N'ATTACHE PAS RÉELLEMENT — LE POINT IMPORTANT
#      run.sh exécute chaque fonction de module avec sa sortie redirigée vers un
#      fichier (`"${p}_start" >>"$log" 2>&1`). Un module n'a donc PAS de terminal :
#      `tmux attach` y échouerait avec « open terminal failed: not a terminal ».
#      Et un `exec` depuis un module tuerait le moteur au milieu de son plan,
#      sans compte rendu ni code de sortie — précisément ce qu'on cherche à
#      supprimer. L'attachement reste donc un geste de l'opérateur : run.sh
#      l'affiche déjà dans son épilogue, et ce module écrit la commande dans
#      $RUN_DIR/attach.cmd pour les scripts qui voudraient l'enchaîner.
#
#      Conséquence pratique : `./run.sh` rend la main au lieu de basculer dans
#      tmux. C'est ce qui permet de l'appeler depuis un autre script ou une CI —
#      et notamment de ne pas bloquer osmo_egprs/start-direct.sh.
# -----------------------------------------------------------------------------
MOD_REGISTER attach "Reprise en main de la session tmux"
MOD_REQUIRED[attach]=0
MOD_DEPS[attach]="tmux"
MOD_PROFILES[attach]="calypso hybrid"
MOD_TIMEOUT[attach]=10
# `-t 1` est évalué par run.sh AVANT toute redirection : il teste bien le
# terminal du moteur, pas celui du module. Sans terminal, sélectionner une
# fenêtre n'aurait aucun destinataire — on saute.
MOD_ENABLED_IF[attach]='[ "${CALYPSO_NO_ATTACH:-0}" != 1 ] && [ -t 1 ]'

: "${TMUX_SESSION:=calypso}"
: "${CALYPSO_DEFAULT_WINDOW:=all}"

_attach_current_window() {
    tmux display-message -p -t "$TMUX_SESSION" '#W' 2>/dev/null
}

mod_attach_check() {
    command -v tmux >/dev/null 2>&1 || { mod_skip "tmux absent — rien à sélectionner"; return $MOD_RC_SKIP; }
    tmux has-session -t "$TMUX_SESSION" 2>/dev/null || {
        mod_hint "le module tmux (35) crée la session ; sans elle la pile tourne quand même, en arrière-plan"
        mod_skip "session tmux « $TMUX_SESSION » inexistante"
        return $MOD_RC_SKIP; }
    mod_ok
}

# Idempotent : si la fenêtre voulue est déjà la fenêtre courante, il n'y a rien
# à faire. On ne renvoie donc « déjà fait » que dans ce cas précis.
mod_attach_status() { [ "$(_attach_current_window)" = "$CALYPSO_DEFAULT_WINDOW" ]; }

mod_attach_start() {
    mkdir -p "$RUN_DIR" 2>/dev/null
    # Repli sur « qemu » comme le legacy (L2416) : la fenêtre « all » n'existe pas
    # si le module panes a été sauté.
    tmux select-window -t "$TMUX_SESSION:$CALYPSO_DEFAULT_WINDOW" 2>/dev/null \
        || tmux select-window -t "$TMUX_SESSION:qemu" 2>/dev/null \
        || mod_say "aucune des fenêtres « $CALYPSO_DEFAULT_WINDOW » / « qemu » n'existe"

    printf 'tmux attach -t %s\n' "$TMUX_SESSION" > "${RUN_DIR}/attach.cmd"
    mod_say "reprise en main : tmux attach -t $TMUX_SESSION"
    mod_say "fin de session  : tmux kill-session -t $TMUX_SESSION"
    mod_ok
}

# BARRIÈRE — la session répond-elle encore, et une fenêtre est-elle bien
# sélectionnée ? C'est le seul fait observable ici : le legacy ne vérifiait même
# pas que `select-window` avait abouti (il jetait l'erreur avec `2>/dev/null`).
mod_attach_wait() {
    wait_until "${MOD_TIMEOUT[attach]}" "session tmux « $TMUX_SESSION »" \
               tmux has-session -t "$TMUX_SESSION" || {
        mod_fail "la session tmux a disparu"
        return $MOD_RC_FAIL; }
    local w; w="$(_attach_current_window)"
    [ -n "$w" ] || { mod_fail "aucune fenêtre sélectionnable dans « $TMUX_SESSION »"
                     mod_hint "tmux list-windows -t $TMUX_SESSION"
                     return $MOD_RC_FAIL; }
    mod_say "fenêtre courante : $w"
    mod_ok
}

# Rien à arrêter : ce module n'a lancé aucun processus. La session elle-même
# appartient au module tmux (35), qui la ferme — la tuer ici emporterait tous
# les services avec elle, à contretemps de l'ordre inverse du plan.
mod_attach_stop() { return 0; }
