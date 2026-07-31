# =============================================================================
#  35-tmux — la session d'observation
# =============================================================================
#
#  RÔLE   Crée la session tmux « calypso » : le poste d'observation de la pile.
#
#  DIFFÉRENCE DE FOND AVEC L'ANCIEN LANCEMENT — et c'est ce qui justifie tout :
#  le legacy LANÇAIT les services DANS tmux (`send-keys ... C-m`). Un service
#  qui mourait laissait sa fenêtre ouverte et personne n'en savait rien : ni
#  PID, ni code de retour, ni rien d'observable. Ici les modules lancent leurs
#  processus eux-mêmes — PID, journal, barrière — et tmux ne sert plus qu'à
#  REGARDER. La session est donc cosmétique : jamais obligatoire, son absence
#  n'empêche aucun service de tourner.
#
#  ORGANISATION   ce module ne décrit AUCUNE fenêtre. Il charge la disposition
#  du profil courant depuis tmux_modules/ (un fichier par profil, plus
#  _commun.sh), puis appelle tmux_layout(). Ajouter un profil = ajouter un
#  fichier, sans toucher ici. Voir tmux_modules/README.md.
#
#  DISPOSITIONS DISPONIBLES :
#      calypso.sh   un téléphone émulé (QEMU) + le cœur
#      hybrid.sh    DEUX téléphones : MS#1 sur QEMU, MS#2 sur fake_trx
#      faketrx.sh   pas de QEMU : téléphone logiciel, radio simulée
#      defaut.sh    repli (profil « core », ou profil sans fichier dédié)
#
#  Les panes du cœur suivent /var/log/osmocom/osmo-*.log — les journaux des
#  SERVICES. Suivre $LOG_DIR/mod/*.log ne montrerait que les traces de démarrage
#  des modules, erreur qui a rendu ces fenêtres inutiles pendant des semaines.
#
#  SUCCÈS   `tmux has-session` répond ET la fenêtre radio existe.
# -----------------------------------------------------------------------------
MOD_REGISTER tmux "Session d'observation tmux"
MOD_REQUIRED[tmux]=0
MOD_DEPS[tmux]="$(modb_dep_known logs)"
MOD_PROFILES[tmux]="calypso hybrid faketrx"
MOD_TIMEOUT[tmux]=10
MOD_ENABLED_IF[tmux]='command -v tmux >/dev/null 2>&1'

: "${TMUX_SESSION:=calypso}"

mod_tmux_check() {
    command -v tmux >/dev/null 2>&1 || {
        mod_hint "apt-get install tmux — ou lancez sans : les services tournent quand même"
        mod_fail "tmux introuvable"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_tmux_status() { tmux has-session -t "$TMUX_SESSION" 2>/dev/null; }

# Où vivent les dispositions. Une par profil, plus « _commun.sh » : ce module ne
# décrit plus aucune fenêtre, il choisit et charge. Voir tmux_modules/README.md.
: "${TMUX_MODULES_DIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")/../tmux_modules" 2>/dev/null && pwd)}"

# _tmux_charger — pose _commun.sh puis la disposition du profil courant.
# Repli sur defaut.sh : un profil sans fichier dédié doit quand même donner une
# session utilisable. tmux ne fait que REGARDER — jamais bloquer un service.
_tmux_charger() {
    local d="${TMUX_MODULES_DIR:-}"
    [ -n "$d" ] && [ -r "$d/_commun.sh" ] || {
        mod_hint "attendu : <dépôt>/tmux_modules/_commun.sh"
        mod_fail "dispositions tmux introuvables (TMUX_MODULES_DIR=${d:-<vide>})"
        return $MOD_RC_FAIL; }
    . "$d/_commun.sh"

    local p="${CALYPSO_PROFILE:-calypso}" f
    if [ -r "$d/$p.sh" ]; then
        f="$d/$p.sh"
    else
        f="$d/defaut.sh"
        mod_say "aucune disposition pour « $p » — repli sur defaut.sh"
    fi
    . "$f"
    mod_say "disposition : $(basename "$f")"
    return 0
}

mod_tmux_start() {
    _tmux_charger || return $MOD_RC_FAIL

    # La première fenêtre naît AVEC la session : c'est la disposition qui dit
    # laquelle et ce qu'elle montre.
    tmux new-session -d -s "$TMUX_SESSION" -n "${TMUX_FENETRE_PREMIERE:-radio}" \
        "$(tmux_layout_premiere)" 2>/dev/null || {
        mod_hint "un serveur tmux périmé peut refuser la session : tmux kill-server puis relancez"
        mod_fail "création de la session « $TMUX_SESSION » refusée"; return $MOD_RC_FAIL; }

    # [2026-07-29] AVANT toute disposition : purger l environnement global du
    # serveur tmux et le reposer depuis le run courant. Sans ça le serveur garde
    # les CALYPSO_* du PREMIER run et les relances suivantes en héritent
    # silencieusement — le manifeste ne correspond plus à la ligne de commande.
    _tmux_env_repropre

    tmux_layout
    _tmux_style
    tmux select-window -t "$TMUX_SESSION:${TMUX_FENETRE_PREMIERE:-radio}" 2>/dev/null
    mod_say "${TMUX_RESUME:-session créée}"
    mod_ok
}

# BARRIÈRE — `new-session -d` peut rendre 0 puis laisser le serveur mourir
# (socket périmée, TERM invalide). On exige la session ET sa première fenêtre.
mod_tmux_wait() {
    wait_until "${MOD_TIMEOUT[tmux]}" "session tmux « $TMUX_SESSION »" \
        tmux has-session -t "$TMUX_SESSION" || return $MOD_RC_FAIL
    # PAS « grep radio » : la disposition choisit le nom de sa première fenêtre
    # (defaut.sh commence par « coeur »). On exige qu'il y ait AU MOINS une
    # fenêtre, ce qui est la seule chose vraie pour toutes les dispositions.
    [ "$(tmux list-windows -t "$TMUX_SESSION" 2>/dev/null | wc -l)" -ge 1 ] || {
        mod_fail "session créée mais aucune fenêtre"; return $MOD_RC_FAIL; }
    mod_ok
}

# kill-SESSION et non kill-SERVER : le serveur peut héberger d'autres sessions
# (celle de l'opérateur, un autre banc). Tuer le serveur est le rôle de
# `teardown`, pas du nôtre — le legacy le faisait, et coupait tout le monde.
mod_tmux_stop() { tmux kill-session -t "$TMUX_SESSION" 2>/dev/null; return 0; }
