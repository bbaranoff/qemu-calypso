# =============================================================================
#  80-panes — fenêtre tmux « all » : la vue de supervision
# =============================================================================
#
#  RÔLE (run.sh.legacy L2168-2228)
#      Une fenêtre unique en grille (layout tiled) qui agrège les journaux et les
#      deux sondes de décodage, pour surveiller la pile d'un coup d'œil :
#        osmocon (pane de base) · qemu · décodage BURST · décodage SI ·
#        bridge.py (si actif) · client L2 · gdb (opt-in).
#      Les processus, eux, vivent dans leurs propres modules : ici ce ne sont que
#      des `tail -F` et des sondes passives. Rien de fonctionnel n'en dépend.
#
#  PRÉREQUIS
#      Une session tmux existante (module 35). Les journaux peuvent ne pas encore
#      exister : `tail -F` (majuscule) suit un fichier dès son apparition.
#
#  CRITÈRE DE SUCCÈS
#      La fenêtre « all » existe et contient au moins deux panes.
#
#  JOURNAL
#      $LOG_DIR/mod/panes.log (trace du module ; les panes affichent, ils ne
#      journalisent pas).
#
#  POURQUOI CE MODULE N'EST JAMAIS OBLIGATOIRE
#      Il est purement cosmétique : une grille absente n'empêche ni le mobile de
#      camper, ni le BTS de tourner. Le faire échouer bloquerait un lancement
#      parfaitement fonctionnel — exactement le contraire du but recherché.
#
#  SPLITS SÉQUENTIELS, `select-layout` APRÈS CHACUN
#      Sans redistribution de l'espace après chaque split, le troisième ou le
#      quatrième pane devient trop étroit et tmux refuse le suivant avec
#      « no space for new pane » (constat du legacy, L2178-2180).
# -----------------------------------------------------------------------------
MOD_REGISTER panes "Vue de supervision (tmux)"
MOD_REQUIRED[panes]=0
MOD_DEPS[panes]="tmux"
MOD_PROFILES[panes]="calypso hybrid"
MOD_TIMEOUT[panes]=15
MOD_ENABLED_IF[panes]='[ "${CALYPSO_SKIP_PANES:-0}" != 1 ]'

: "${TMUX_SESSION:=calypso}"
: "${CALYPSO_GSM_SNIFF:=${QEMU_TREE:-${QEMU_TREE}}/opt-gsm-scripts/gsm_sniff.py}"
# FIRMWARE_ELF est pose par environnement/paths.env, qui prend le firmware
# livre avec le depot. Le repli ne sert que si ce module est source hors run.sh.
: "${CALYPSO_GDB_ELF:=${FIRMWARE_ELF:-${QEMU_TREE:-.}/firmware/compal_e88/layer1.highram.elf}}"
: "${CALYPSO_GDB_PORT:=1234}"

_panes_count() { tmux list-panes -t "$TMUX_SESSION:all" 2>/dev/null | wc -l; }
_panes_enough() { [ "$(_panes_count)" -ge 2 ]; }

mod_panes_check() {
    command -v tmux >/dev/null 2>&1 || { mod_skip "tmux absent — pas de vue de supervision"; return $MOD_RC_SKIP; }
    # En simulation, la session n'a pas été créée (le module tmux a été simulé) :
    # exiger son existence produirait un échec qui n'en est pas un. On lit DRY,
    # la variable du moteur, sans y toucher.
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : existence de la session tmux non vérifiée"
        mod_ok; return $MOD_RC_OK
    fi
    tmux has-session -t "$TMUX_SESSION" 2>/dev/null || {
        mod_hint "le module tmux (35) crée la session ; ./run.sh --only tmux"
        mod_fail "session tmux « $TMUX_SESSION » inexistante"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_panes_status() { tmux list-windows -t "$TMUX_SESSION" 2>/dev/null | grep -q '^[0-9]*: all'; }

mod_panes_start() {
    local l2log
    case "${CALYPSO_L2_CLIENT:-mobile}" in
        ccch_scan|bcch_scan|cell_log) l2log="${LOG_DIR}/l2_client.log" ;;
        *)                            l2log="${LOG_DIR}/mobile.log" ;;
    esac

    # Pane de base : osmocon. QEMU garde sa fenêtre dédiée ; ici, vue seulement.
    tmux new-window -t "$TMUX_SESSION" -n all \
        "clear; echo '=== osmocon ==='; tail -F ${LOG_DIR}/osmocon.log" || {
        mod_fail "tmux a refusé de créer la fenêtre « all »"
        return $MOD_RC_FAIL; }

    local specs=() spec name what cmd added=0
    specs+=("qemu|${LOG_DIR}/qemu.log")
    if [ "${CALYPSO_SKIP_DECODE_PANES:-0}" != 1 ] && [ -r "$CALYPSO_GSM_SNIFF" ]; then
        specs+=("burst|__BURST__")
        # En mode pont (CALYPSO_BRIDGE=pont) le decodeur SI (si_bridge/grgsm)
        # est eteint : le pane afficherait un flux vide. On le retire.
        [ "${CALYPSO_BRIDGE:-}" = pont ] || specs+=("si|__SI__")
    fi
    [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" != 1 ] && specs+=("bridge-py|${LOG_DIR}/bridge.py.log")
    [ "${CALYPSO_SKIP_L2:-0}"        != 1 ] && specs+=("${CALYPSO_L2_CLIENT:-mobile}|$l2log")
    [ "${CALYPSO_SKIP_GDB_PANE:-1}"  != 1 ] && specs+=("gdb|__GDB__")

    for spec in "${specs[@]}"; do
        name="${spec%%|*}"; what="${spec##*|}"
        case "$what" in
            __SI__)
                # Sonde PASSIVE (socket brute) : aucune FIFO, aucune perturbation
                # du flux. Le `sleep 16` du legacy (L2207) est SUPPRIMÉ : le
                # sniffer bloque déjà sur ses propres paquets, attendre avant de
                # le lancer ne faisait que perdre les premiers.
                cmd="clear; echo '=== SI decode (4729/4730) ==='; python3 -u $CALYPSO_GSM_SNIFF si" ;;
            __BURST__)
                # Idem pour le `sleep 16` de L2210.
                cmd="clear; echo '=== BURST decode (4731 + 5700-5702) ==='; python3 -u $CALYPSO_GSM_SNIFF burst" ;;
            __GDB__)
                # Le `sleep 3` de L2222 attendait que QEMU binde son port gdb.
                # Remplacé par une attente de CONDITION, dans le pane lui-même :
                # on boucle sur l'ouverture du port, avec un plafond de 30 s.
                cmd="clear; echo '=== gdb (attach :$CALYPSO_GDB_PORT) ==='; \
i=0; while ! (exec 3<>/dev/tcp/127.0.0.1/$CALYPSO_GDB_PORT) 2>/dev/null && [ \$i -lt 150 ]; do sleep 0.2; i=\$((i+1)); done; \
gdb-multiarch -q -iex 'set pagination off' -iex 'set confirm off' -iex 'set architecture armv5te' -iex 'target remote :$CALYPSO_GDB_PORT' $CALYPSO_GDB_ELF; \
echo '[gdb terminé, Entrée pour fermer]'; read -r _" ;;
            *)  cmd="clear; echo '=== $name ==='; tail -F $what" ;;
        esac
        if tmux split-window -t "$TMUX_SESSION:all" "$cmd" 2>/dev/null; then
            added=$((added + 1))
        else
            mod_say "pane « $name » refusé par tmux (place insuffisante)"
        fi
        tmux select-layout -t "$TMUX_SESSION:all" tiled >/dev/null 2>&1
    done
    mod_say "fenêtre « all » : $((added + 1)) panes demandés"
    mod_ok
}

# BARRIÈRE — la fenêtre existe-t-elle vraiment, avec ses panes ? Un pane dont la
# commande meurt aussitôt se referme : compter les panes est donc un critère
# observable, pas une formalité. On n'exige pas le compte exact (un sniffer sans
# droits peut légitimement disparaître) mais au moins le pane de base + un.
mod_panes_wait() {
    wait_until "${MOD_TIMEOUT[panes]}" "panes de la fenêtre « all »" _panes_enough || {
        mod_hint "tmux list-panes -t $TMUX_SESSION:all pour voir ce qui a survécu"
        mod_fail "la fenêtre « all » n'a pas gardé ses panes"
        return $MOD_RC_FAIL; }
    mod_say "panes vivants : $(_panes_count)"
    mod_ok
}

mod_panes_stop() {
    tmux kill-window -t "$TMUX_SESSION:all" 2>/dev/null
    return 0
}
