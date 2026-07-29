# =============================================================================
#  90-gdb — fenêtre tmux attachée au gdb-stub de QEMU
# =============================================================================
#
#  RÔLE (run.sh.legacy L2301-2329)
#      Crée une fenêtre « gdb » : en haut gdb-multiarch attaché au stub de QEMU,
#      en bas un `tail -f` de qemu.log. Si CALYPSO_QEMU_HALT=1, QEMU est arrêté
#      au premier cycle : on pose ses points d'arrêt puis on continue.
#
#  PRÉREQUIS
#      QEMU démarré avec son stub gdb (40-qemu passe `-gdb tcp::1234`),
#      gdb-multiarch installé, et l'ELF du firmware lisible (il porte les
#      symboles : sans lui, la session gdb ne montre que des adresses).
#
#  CRITÈRE DE SUCCÈS
#      Le port du stub gdb accepte une connexion, et la fenêtre existe.
#
#  JOURNAL
#      $LOG_DIR/mod/gdb.log (le dialogue gdb reste dans le pane, interactif).
#
#  POURQUOI LE PORT EST TESTÉ AVANT DE LANCER gdb
#      Le legacy dormait 3 s (L2222) puis lançait gdb en espérant que QEMU ait
#      bindé son port. Si le port n'était pas là, gdb sortait sur « Connection
#      refused » et la fenêtre se refermait sans que personne ne le sache. Ici la
#      condition est testée pour de bon, et un échec est nommé.
#
#  ATTENTION — VALEUR DU PORT
#      40-qemu.sh code aujourd'hui `-gdb tcp::1234` en dur (L54-57). Tant que ce
#      n'est pas paramétré, CALYPSO_GDB_PORT ne peut pas être changé : ce module
#      le dit plutôt que de laisser croire au réglage.
# -----------------------------------------------------------------------------
MOD_REGISTER gdb "Console gdb (stub QEMU)"
MOD_REQUIRED[gdb]=0
MOD_DEPS[gdb]="qemu"
MOD_PROFILES[gdb]="calypso hybrid"
MOD_TIMEOUT[gdb]=20
MOD_ENABLED_IF[gdb]='[ "${CALYPSO_GDB_PANE:-0}" = 1 ]'

: "${TMUX_SESSION:=calypso}"
: "${CALYPSO_GDB_PORT:=1234}"
# FIRMWARE_ELF est pose par environnement/paths.env, qui prend le firmware
# livre avec le depot. Le repli ne sert que si ce module est source hors run.sh.
: "${CALYPSO_GDB_ELF:=${FIRMWARE_ELF:-${QEMU_TREE:-.}/firmware/compal_e88/layer1.highram.elf}}"

_gdb_port_open() { have_port "$CALYPSO_GDB_PORT"; }

mod_gdb_check() {
    command -v gdb-multiarch >/dev/null 2>&1 || {
        mod_hint "apt-get install gdb-multiarch — ou lancez sans CALYPSO_GDB_PANE=1"
        mod_fail "gdb-multiarch absent"
        return $MOD_RC_FAIL; }
    if ! command -v tmux >/dev/null 2>&1; then
        mod_skip "tmux absent — pas de fenêtre où loger gdb"
        return $MOD_RC_SKIP
    fi
    [ -r "$CALYPSO_GDB_ELF" ] || {
        mod_hint "posez CALYPSO_GDB_ELF vers l'ELF du firmware (celui qui porte les symboles)"
        mod_fail "ELF de débogage illisible : $CALYPSO_GDB_ELF"
        return $MOD_RC_FAIL; }

    # Fin des prérequis STATIQUES (binaire, tmux, ELF). Ce qui suit porte sur
    # l'état d'exécution : en simulation, rien n'a été lancé, donc rien à
    # vérifier. DRY est la variable du moteur ; on la lit, on n'y touche pas.
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : session tmux et stub gdb non vérifiés"
        mod_ok; return $MOD_RC_OK
    fi

    if ! tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
        mod_hint "le module tmux (35) crée la session"
        mod_fail "session tmux « $TMUX_SESSION » inexistante — pas de fenêtre où mettre gdb"
        return $MOD_RC_FAIL
    fi

    # --- remplacement du `sleep 3` (L2222, et l'attente implicite de L2321) ---
    # Condition réelle : le stub gdb de QEMU écoute.
    wait_until "${MOD_TIMEOUT[gdb]}" "stub gdb sur le port $CALYPSO_GDB_PORT" _gdb_port_open || {
        mod_hint "40-qemu.sh passe « -gdb tcp::1234 » en dur : CALYPSO_GDB_PORT n'est pas encore honoré côté QEMU"
        mod_fail "aucun stub gdb sur 127.0.0.1:$CALYPSO_GDB_PORT"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gdb_status() { tmux list-windows -t "$TMUX_SESSION" 2>/dev/null | grep -q '^[0-9]*: gdb'; }

mod_gdb_start() {
    local script="${CALYPSO_GDB_SCRIPT:-}"
    if [ -z "$script" ]; then
        script="${RUN_DIR}/gdb.gdb"
        mkdir -p "$RUN_DIR" 2>/dev/null
        # Script minimal : attacher, puis continuer. Surchargez par
        # CALYPSO_GDB_SCRIPT pour poser vos points d'arrêt à l'attachement.
        cat > "$script" <<GDBRC
set pagination off
set confirm off
set architecture armv5te
target remote :${CALYPSO_GDB_PORT}
c
GDBRC
    fi
    mod_say "script   : $script"
    mod_say "elf      : $CALYPSO_GDB_ELF"

    tmux new-window -t "$TMUX_SESSION" -n gdb \
        "gdb-multiarch -q -iex 'set pagination off' -iex 'set confirm off' -x $script $CALYPSO_GDB_ELF; \
         echo '[gdb terminé, Entrée pour fermer]'; read -r _" || {
        mod_fail "tmux a refusé de créer la fenêtre « gdb »"
        return $MOD_RC_FAIL; }
    tmux split-window -t "$TMUX_SESSION:gdb" -v -p 40 "tail -f ${LOG_DIR}/qemu.log" 2>/dev/null
    tmux select-pane -t "$TMUX_SESSION:gdb.0" 2>/dev/null
    mod_ok
}

# BARRIÈRE — la fenêtre existe ET le stub répond toujours. Si gdb a échoué à
# s'attacher, sa fenêtre reste (le `read` la maintient) : on vérifie donc les
# deux, la fenêtre pour l'affichage et le port pour la connexion.
mod_gdb_wait() {
    wait_until "${MOD_TIMEOUT[gdb]}" "fenêtre tmux « gdb »" mod_gdb_status || {
        mod_fail "la fenêtre « gdb » ne s'est pas créée"
        return $MOD_RC_FAIL; }
    _gdb_port_open || {
        mod_hint "regardez le pane gdb : « Connection refused » = QEMU a perdu son stub"
        mod_fail "le stub gdb ne répond plus sur $CALYPSO_GDB_PORT"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gdb_stop() {
    tmux kill-window -t "$TMUX_SESSION:gdb" 2>/dev/null
    return 0
}
