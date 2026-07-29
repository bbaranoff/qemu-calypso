# =============================================================================
#  tmux_modules/defaut.sh — repli quand le profil n'a pas de disposition
# =============================================================================
#  Utilisé pour « core » et pour tout profil ajouté sans fichier dédié. On montre
#  le cœur et un shell : c'est le plus petit ensemble utile.
#
#  Un repli plutôt qu'une erreur : l'absence de disposition ne doit jamais
#  empêcher la session d'exister, puisque tmux ici ne fait que REGARDER — aucun
#  service n'en dépend.
# -----------------------------------------------------------------------------
TMUX_FENETRE_PREMIERE=coeur
TMUX_RESUME="disposition de repli : coeur · shell"

tmux_layout_premiere() {
    printf "tail -n 200 -F '%s/osmo-msc.log' 2>/dev/null | stdbuf -oL tr -d '\\\\007' || sleep infinity" \
           "${OSMO_LOG_DIR:-/var/log/osmocom}"
}

tmux_layout() {
    local O="${OSMO_LOG_DIR:-/var/log/osmocom}"
    tmux select-pane -t "$TMUX_SESSION:coeur" -T \
        "MSC | commutation circuit - appels, SMS, authentification" 2>/dev/null
    _paint coeur "$C_COEUR"
    _split coeur "BSC | controleur de stations - allocation des canaux" \
        "$(_tail "'$O/osmo-bsc.log'")" "$C_COEUR"
    _split coeur "HLR | registre des abonnes - IMSI, cles, souscriptions" \
        "$(_tail "'$O/osmo-hlr.log'")" "$C_COEUR"
    _split coeur "MGW + STP | media MGCP/RTP et routage SS7" \
        "$(_tail "'$O/osmo-mgw.log' '$O/osmo-stp.log'")" "$C_COEUR"
    _fenetre_shell
}
