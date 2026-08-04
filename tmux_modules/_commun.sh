# =============================================================================
#  tmux_modules/_commun.sh — l'outillage partagé par toutes les dispositions
# =============================================================================
#
#  Chargé par run_modules/35-tmux.sh AVANT la disposition du profil. Fournit les
#  couleurs, les deux constructeurs de pane et l'habillage de la session.
#  Aucune disposition ne crée de fenêtre autrement que par _w / _split.
#
#  COULEUR PAR RÔLE — la teinte dit ce que le pane MONTRE, pas où il se trouve.
#  Quatre `tail` côte à côte sont indiscernables : le titre se lit, la couleur se
#  reconnaît sans lire.
#      cyan    chaîne radio du téléphone ÉMULÉ (QEMU, osmocon, transceiver)
#      vert    cœur réseau Osmocom (MSC, BSC, HLR, MGW, STP)
#      ambre   ce que le premier abonné comprend (couche 2, gr-gsm, GSMTAP)
#      orange  un SECOND abonné et sa radio simulée
#      magenta chemin audio (GAPK, SIP)
# -----------------------------------------------------------------------------
C_RADIO=colour81      # cyan
C_COEUR=colour114     # vert
C_MS1=colour179       # ambre
C_MS2=colour215       # orange
C_VOIX=colour176      # magenta

# `tail -F` (majuscule) et non `-f` : le journal peut ne pas exister encore, et
# le garde-fou de 40-qemu le TRONQUE au-delà du plafond. `-F` reprend le suivi
# dans les deux cas au lieu de rester muet.
SUIVRE="tail -n 100 -F"

# Journaux des SERVICES du cœur. Volontairement hors de $LOG_DIR : le teardown
# efface ce répertoire, et un service déjà lancé garderait un descripteur sur un
# fichier supprimé (osmocom ne rouvre ses fichiers que sur SIGHUP).
: "${OSMO_LOG_DIR:=/var/log/osmocom}"

_paint() {  # _paint <fenêtre> <couleur>
    [ -n "${2:-}" ] || return 0
    tmux select-pane -t "$TMUX_SESSION:$1" -P "fg=$2" 2>/dev/null || true
}

_w() {  # _w <fenêtre> <titre-pane> <commande> [couleur]
    tmux new-window -t "$TMUX_SESSION" -n "$1" "$3" 2>/dev/null \
        && tmux select-pane -t "$TMUX_SESSION:$1" -T "$2" 2>/dev/null \
        && _paint "$1" "${4:-}"
}

_split() {  # _split <fenêtre> <titre-pane> <commande> [couleur]
    tmux split-window -t "$TMUX_SESSION:$1" "$3" 2>/dev/null \
        && tmux select-pane -t "$TMUX_SESSION:$1" -T "$2" 2>/dev/null \
        && _paint "$1" "${4:-}" \
        && tmux select-layout -t "$TMUX_SESSION:$1" tiled 2>/dev/null
}

# _tail <fichier…> — la commande d'un pane de suivi.
#
# `sleep infinity` en repli : sans lui, un journal absent ferait mourir le pane
# et la fenêtre se refermerait en emportant les autres.
#
# `tr -d '\\007'` : osmo-trx-ipc écrit des BEL dans son journal (932 sur un seul
# run). Sans ce filtre, chaque octet fait sonner le terminal — un bip continu et
# inutile, puisque ce sont des restes de formatage, pas un signal. `stdbuf -oL`
# parce qu'un `tr` en milieu de tube passerait en tampon par blocs et figerait
# l'affichage par paquets de 4 Ko.
_tail() {
    printf "%s %s 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "$SUIVRE" "$*"
}

# Fenêtres communes à toutes les dispositions, pour ne pas les récrire quatre
# fois. Le shell affiche les commandes utiles : on ouvre cette fenêtre justement
# quand on ne se souvient plus des ports.
_fenetre_coeur() {
    local O="${OSMO_LOG_DIR:-/var/log/osmocom}"
    _w     coeur "MSC | commutation circuit - appels, SMS, authentification" \
        "$(_tail "'$O/osmo-msc.log'")" "$C_COEUR"
    _split coeur "BSC | controleur de stations - allocation des canaux" \
        "$(_tail "'$O/osmo-bsc.log'")" "$C_COEUR"
    _split coeur "HLR | registre des abonnes - IMSI, cles, souscriptions" \
        "$(_tail "'$O/osmo-hlr.log'")" "$C_COEUR"
    _split coeur "MGW + STP | media MGCP/RTP et routage SS7" \
        "$(_tail "'$O/osmo-mgw.log' '$O/osmo-stp.log'")" "$C_COEUR"
}

_fenetre_dsp() {
    # [2026-07-29] La frontière ARM <-> DSP, la seule qui compte pour le natif.
    #
    #   mail_dissam.log = le croisement mailbox × désassemblage : quelle cellule,
    #     dans quel sens, combien de fois, et QUELLE instruction la touche. C'est
    #     un TABLEAU réécrit toutes les 2 s, pas un flux — donc `watch`, pas `tail`.
    #   mailbox.log = le flux brut, replié au changement de valeur.
    #
    # Fenêtre séparée à dessein : ces deux vues méritent de la place, et les
    # écraser dans `radio` rendrait les quatre panes existants illisibles.
    local L="${LOG_DIR:-/root/calypso/logs}"
    _w     dsp "mailbox x desassemblage | quelle instruction touche quelle cellule" \
        "while :; do clear; cat '$L/mail_dissam.log' 2>/dev/null || echo 'en attente du premier cycle...'; sleep 2; done" \
        "${C_RADIO:-}"
    _split dsp "mailbox | flux ARM<->DSP brut, replie au changement" \
        "$(_tail "'$L/mailbox.log'")" "${C_RADIO:-}"
}

_fenetre_asm() {
    # [2026-07-29] La frontière ARM<->DSP au niveau INSTRUCTION.
    #
    #   haut  — le TABLEAU agrégé (mail_dissam.log) : quelle cellule, combien de
    #           fois, par quelle instruction. Réécrit toutes les 2 s, donc
    #           `clear; cat` et non `tail` (le mv change l'inode : `tail -f` ne
    #           verrait jamais rien bouger).
    #   bas   — le FLUX annoté, au fil de l'eau : chaque accès mailbox suivi de
    #           l'instruction c54x qui le produit. C'est la vue « asm bas niveau »
    #           du dialogue ARM<->DSP.
    #
    # Le désassemblage est mis en cache par PC côté outil : une boucle serrée ne
    # coûte qu'une recherche, pas une par ligne.
    local L="${LOG_DIR:-/root/calypso/logs}"
    local T="${QEMU_TREE:-/opt/GSM/osmo-qemu-calypso}"
    _w     asm "croisement | cellule x instruction, agrege, rafraichi 2s" \
        "while :; do clear; cat '$L/mail_dissam.log' 2>/dev/null || echo 'en attente du premier cycle...'; sleep 2; done" \
        "${C_RADIO:-}"
    _split asm "mailbox ASM | chaque acces suivi de son instruction c54x" \
        "tail -n 40 -F '$L/mailbox.log' 2>/dev/null | stdbuf -oL python3 '$T/tools/mailbox-annote.py' --flux 2>/dev/null || sleep infinity" \
        "${C_RADIO:-}"
}

_fenetre_voix() {
    local L="${LOG_DIR:-/root/calypso/logs}" O="${OSMO_LOG_DIR:-/var/log/osmocom}"
    _w     voix "gapk | transcodage GSM-FR <-> PCM (actif pendant un appel)" \
        "$(_tail "'$L/gapk-auto.log'")" "$C_VOIX"
    _split voix "SIP | passerelle MNCC <-> SIP vers Asterisk" \
        "$(_tail "'$O/osmo-sip-connector.log'")" "$C_VOIX"
}

_fenetre_shell() {   # _fenetre_shell [ligne d'aide supplémentaire…]
    local aide=""
    local l; for l in "$@"; do aide="${aide}  ${l}\\n"; done
    _w shell "shell | depot osmo-qemu-calypso" \
        "cd '${QEMU_TREE:-.}' && printf '\n  %s\n  %s\n${aide}\n' \
             'etat  : ./run.sh --status' 'arret : ./run.sh --stop' \
         && exec ${SHELL:-/bin/bash}"
}

# Habillage. La barre de statut nomme PROFIL · MODE · PIPELINE : la question
# « on est en quel mode ? » se pose à chaque session, et la ligne de commande
# ment (les fichiers d'environnement peuvent la surcharger).

# -----------------------------------------------------------------------------
#  _tmux_env_repropre — rendre le run REPRODUCTIBLE depuis sa ligne de commande
# -----------------------------------------------------------------------------
#  [2026-07-29] Le serveur tmux HÉRITE de l environnement du premier « ./run.sh »
#  qui l a créé, et le conserve comme environnement GLOBAL. Toute relance faite
#  depuis un pane hérite donc des variables de la ligne d avant. Symptôme vécu :
#  « CALYPSO_MODE=native ./run.sh » produisait un manifeste portant INJECT_SB=1,
#  SHUNT_REAL_FB=1, FRAME_IT_NATIVE=1, AB38=1 — jamais tapées — et « la même
#  commande » donnait deux résultats différents à cinq minutes d intervalle.
#
#  On purge donc TOUTES les CALYPSO_* de l environnement global tmux, puis on
#  repose exactement celles du processus courant. Après ça, le manifeste reflète
#  la ligne de commande, et deux runs identiques le sont vraiment.
#
#  NB : on ne touche qu au préfixe CALYPSO_ — PATH, LANG, TERM et le reste
#  doivent continuer d être hérités normalement.
_tmux_env_repropre() {
    command -v tmux >/dev/null 2>&1 || return 0

    # 1. purge — « show-environment -g » préfixe d un « - » les variables déjà
    #    supprimées ; le sed ne retient que les lignes « NOM=valeur ».
    tmux show-environment -g 2>/dev/null \
        | sed -n 's/^\(CALYPSO_[A-Za-z0-9_]*\)=.*/\1/p' \
        | while IFS= read -r _v; do
              [ -n "$_v" ] && tmux set-environment -g -u "$_v" 2>/dev/null
          done

    # 2. et on NE REPOSE RIEN. Première version de ce correctif : je reposais
    #    l environnement du run courant — erreur. Après `load.env`, cet
    #    environnement contient les ~170 CALYPSO_* avec leurs défauts RÉSOLUS ;
    #    les reposer les rend collantes exactement comme le fossile qu on vient
    #    de purger, et « := » ne peut plus rien poser au run suivant. On aurait
    #    remplacé un fossile par un autre.
    #
    #    Aucun pane n en a besoin : les dispositions de tmux_modules/ n utilisent
    #    aucune CALYPSO_* en expansion différée (vérifié) — la ligne de statut est
    #    expansée à la construction, côté run.sh. Le serveur tmux garde donc une
    #    ardoise vierge, et la ligne de commande redevient la seule source.
    unset _v
}

_tmux_style() {
    local s="$TMUX_SESSION"
    # Locale de la session : sans elle, tmux translittere les caracteres non-ASCII
    # selon la locale du client. Les titres restent malgre tout en ASCII : un
    # titre illisible est pire qu un titre sobre.
    tmux set-environment -t "$s" LANG   "${LANG:-C.UTF-8}"   2>/dev/null
    tmux set-environment -t "$s" LC_ALL "${LC_ALL:-C.UTF-8}" 2>/dev/null
    tmux set -t "$s" -g mouse on            2>/dev/null
    tmux set -t "$s" -g base-index 1        2>/dev/null
    tmux set -t "$s" -g pane-base-index 1   2>/dev/null
    tmux set -t "$s" -g history-limit 20000 2>/dev/null
    tmux set -t "$s" -g renumber-windows on 2>/dev/null

    # La cloche : désarmée. osmo-trx-ipc écrit des BEL dans son journal, et
    # « bell-action any » les transforme en bip du terminal à chaque ligne
    # suivie. Le filtre de _tail les supprime déjà ; ceci couvre les autres
    # sources (un service qui en émettrait sur sa sortie directe).
    tmux set -t "$s" -g bell-action none  2>/dev/null
    tmux set -t "$s" -g visual-bell off   2>/dev/null
    tmux set -t "$s" -g monitor-bell off  2>/dev/null

    tmux set -t "$s" -g status-style        "bg=colour234,fg=colour250" 2>/dev/null
    tmux set -t "$s" -g status-left-length  40 2>/dev/null
    tmux set -t "$s" -g status-left \
        "#[bg=colour31,fg=colour15,bold] calypso #[default] " 2>/dev/null
    tmux set -t "$s" -g status-right-length 76 2>/dev/null
    tmux set -t "$s" -g status-right \
        "#[fg=colour215]${CALYPSO_PROFILE:-calypso}#[fg=colour244] · #[fg=colour114]${CALYPSO_MODE:-?}#[fg=colour244] · #[fg=colour81]${CALYPSO_PIPELINE:-?}#[default]  #[fg=colour250]%H:%M " 2>/dev/null
    tmux set -t "$s" -g window-status-format         " #I #W " 2>/dev/null
    tmux set -t "$s" -g window-status-current-format \
        "#[bg=colour31,fg=colour15,bold] #I #W #[default]" 2>/dev/null
    tmux set -t "$s" -g pane-border-style        "fg=colour238" 2>/dev/null
    tmux set -t "$s" -g pane-active-border-style "fg=colour31"  2>/dev/null
    # Les panes portent le nom de ce qu'ils montrent : sans ça, six `tail`
    # identiques deviennent indiscernables au bout de deux minutes.
    tmux set -t "$s" -g pane-border-status top 2>/dev/null
    tmux set -t "$s" -g pane-border-format " #{pane_title} " 2>/dev/null
}
