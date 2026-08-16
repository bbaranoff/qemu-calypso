# =============================================================================
#  10-teardown — repartir d'une machine propre
# =============================================================================
#
#  RÔLE
#    Éteindre tout ce qu'un run précédent a laissé derrière lui : processus,
#    sockets, FIFO, mémoire partagée, session tmux périmée. Reprend
#    run.sh.legacy #1 (L2), #21 (L1518-1546) et #23 (L1555-1562).
#
#  PRÉREQUIS  : 09-logs (les chemins à nettoyer sont connus).
#  SUCCÈS     : aucun des motifs de processus ne répond, les ports de la chaîne
#               sont libres, et les sockets de rendez-vous ont disparu.
#  JOURNAL    : $LOG_DIR/mod/teardown.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. C'EST ICI QUE LES DEUX `sleep 1` DU LEGACY DISPARAISSENT.
#     L1555 « laisse les sockets UDP/TCP se libérer » et L1562 attendaient une
#     CONSÉQUENCE du kill, jamais une durée. Une seconde suffisait d'habitude ;
#     sur une machine chargée, non — et le run repartait sur un port encore
#     tenu, ce qui se manifestait beaucoup plus loin, sous la forme d'un
#     osmo-trx-ipc muet. La barrière ci-dessous sonde la condition réelle :
#     plus de processus, plus de port, plus de socket.
#
#  2. POURQUOI ON NE TUE PAS python3 EN BLOC.
#     `killall python3` tuait aussi le filtre d'horodatage du run en cours
#     (« Killed » en plein journal). Les motifs sont donc nominatifs, comme la
#     note du legacy L1523 le demandait.
#
#  3. POURQUOI `tmux kill-server` EST CONDITIONNEL, CONTRAIREMENT AU LEGACY.
#     Un serveur tmux périmé (hérité d'un boot précédent) réutilisé par
#     `new-session` donne des panes sans environnement : d'où le kill-server du
#     legacy (L1518-1522). Mais si l'opérateur lance ./run.sh DEPUIS tmux, ce
#     kill-server tue sa propre session — le run meurt au milieu de son
#     nettoyage, sans rien afficher. Quand $TMUX est présent, on se limite donc
#     à la session cible. La raison du legacy est préservée (serveur frais)
#     sauf dans le seul cas où elle se retournait contre l'opérateur.
#
#  4. LE Kc PÉRIMÉ (legacy L2, tout premier geste du fichier).
#     /dev/shm/calypso_kc survit à l'arrêt de la pile. Réutilisé au run suivant,
#     le SABM montant part chiffré avec une clé que le réseau n'a plus : la
#     Location Update fraîche échoue sans message clair. On l'efface.
# -----------------------------------------------------------------------------

MOD_REGISTER teardown "Nettoyage du run précédent"
MOD_REQUIRED[teardown]=1
MOD_DEPS[teardown]="logs"
MOD_PROFILES[teardown]="calypso faketrx hybrid core"
MOD_TIMEOUT[teardown]=25

# Motifs de processus. Deux formes :
#   x:<nom>      correspondance EXACTE sur le nom (pgrep/pkill -x) — pour les
#                noms courts et communs (« mobile ») qu'un -f ferait sur-matcher.
#   f:<motif>    correspondance sur la ligne de commande (pgrep/pkill -f).
# [2026-08-16] `f:pont/pont.py` AJOUTE. Le mode CALYPSO_BRIDGE=pont fait tourner
# /opt/GSM/pont/pont.py, qui tient 5700-5702. `_td_ports` le DETECTAIT deja
# (ces trois ports y sont listes) mais aucun motif ne le TUAIT : on detectait
# sans executer, d'ou la boucle « restes apres 12 s : port:5700 port:5701
# port:5702 — deuxieme passe » puis l'abandon de la sequence.
# ⚠️ Le motif seul ne suffit PAS : start-direct.sh arme un relanceur DIFFERE
# (`( sleep N; ... exec setsid python3 -u "$_PONT" ) &`). Si le teardown dure
# plus longtemps que ce delai — c'est le cas des qu'il archive de gros
# journaux — le pont se rebinde EN PLEIN teardown et la deuxieme passe le
# retrouve. Le delai fixe a donc ete remplace, cote start-direct.sh, par une
# attente d'osmo-bts-trx : le pont ne binde plus qu'une fois la pile debout.
_td_patterns() {
    cat <<'EOF'
f:qemu-system-arm
f:calypso-ipc-device
f:osmo-trx-ipc
x:osmo-bts-trx
x:osmocon
x:mobile
x:trxcon
f:grgsm_trx
f:grgsm_relay
f:grgsm_shm_decode
f:grgsm_fft_live
f:bin/grgsm_decode
f:grgsm_cfile
f:gsmtap_relay
f:si_bridge
f:pont/pont.py
f:relay_continu
f:record_drain
f:inject.py
f:inject_si3
f:validating.py
f:cp210x_tee
f:irda_peer.py
f:ccch_scan
f:bcch_scan
x:cell_log
EOF
}

# Ports de la chaîne. La plupart sont en UDP : `have_port` (mod.sh) ouvre un
# /dev/tcp et ne les verrait donc JAMAIS occupés — un faux « libre » permanent.
# On sonde les deux familles avec ss.
_td_port_busy() {
    ss -ln 2>/dev/null | grep -qE "[:.]${1}[[:space:]]"
}
_td_ports() {
    printf '%s\n' "${PORT_TRXD_CLOCK:-5700}" "${PORT_TRXD_CTRL:-5701}" "${PORT_TRXD_DATA:-5702}" \
                  "${PORT_IQ_RELAY_RX:-5810}" "${PORT_IQ_RELAY_TX:-5811}" \
                  "${PORT_BSP_UDP:-6702}" "${PORT_IQ_TEE:-6703}" \
                  "${PORT_GSMTAP_SI:-4730}" "${PORT_GSMTAP_SCH:-4731}" \
                  "${PORT_GDB:-1234}"
}

_td_sockets() {
    printf '%s\n' "${L1CTL_SOCK_PATH:-/tmp/osmocom_l2}" \
                  "${QEMU_MON_SOCK:-${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}" \
                  "${QEMU_DUMMY_SOCK:-/tmp/qemu_l1ctl_disabled}" \
                  "${IPC_MSOCK_PATH:-/tmp/ipc_sock0}" \
                  "${IPC_MSOCK_PATH:-/tmp/ipc_sock0}_0"
}

# Ce qui reste debout, sous forme lisible — sert au message d'échec.
_td_leftovers() {
    local line kind pat out="" p
    while IFS= read -r line; do
        kind="${line%%:*}"; pat="${line#*:}"
        case "$kind" in
            x) pgrep -x "$pat" >/dev/null 2>&1 && out="$out $pat" ;;
            f) pgrep -f "$pat" >/dev/null 2>&1 && out="$out $pat" ;;
        esac
    done < <(_td_patterns)
    while IFS= read -r p; do
        _td_port_busy "$p" && out="$out port:$p"
    done < <(_td_ports)
    while IFS= read -r p; do
        [ -e "$p" ] && out="$out socket:$p"
    done < <(_td_sockets)
    printf '%s' "${out# }"
}
_td_clean() { [ -z "$(_td_leftovers)" ]; }

mod_teardown_check() {
    # Lecture seule : sans pkill/pgrep, ce module ne peut rien garantir, et le
    # run repartirait sur les restes du précédent — le mode de panne le plus
    # coûteux à diagnostiquer.
    local t
    for t in pgrep pkill; do
        command -v "$t" >/dev/null 2>&1 || {
            mod_hint "installez procps (paquet procps / procps-ng)"
            mod_fail "$t introuvable : impossible de garantir une machine propre"
            return $MOD_RC_FAIL
        }
    done
    command -v ss >/dev/null 2>&1 || mod_say "ss absent : la libération des ports UDP ne sera pas vérifiée"
    mod_ok
}

# « Déjà démarré » = « déjà propre » : rien à faire, et c'est idempotent.
mod_teardown_status() { _td_clean; }

_td_kill_all() {
    local line kind pat
    while IFS= read -r line; do
        kind="${line%%:*}"; pat="${line#*:}"
        case "$kind" in
            x) pgrep -x "$pat" >/dev/null 2>&1 && { mod_say "kill -x $pat"; pkill -9 -x "$pat" 2>/dev/null; } ;;
            f) pgrep -f "$pat" >/dev/null 2>&1 && { mod_say "kill -f $pat"; pkill -9 -f "$pat" 2>/dev/null; } ;;
        esac
    done < <(_td_patterns)
    return 0
}

# DERNIER RECOURS — tuer QUI TIENT LE PORT, quand aucun motif ne matche.
#
# POURQUOI. `_td_ports` DETECTE un port occupe, mais `_td_kill_all` ne frappe que
# des motifs NOMINATIFS. Tout binaire lance sous un chemin non prevu est donc vu
# et jamais tue — et c'est exactement la boucle « restes apres 12 s : port:5700
# port:5701 port:5702 — deuxieme passe » qui faisait abandonner la sequence.
# Le cas n'est pas theorique : start-direct.sh lit
# `_PONT="${PONT_PY:-/opt/GSM/pont/pont.py}"`, et /opt/GSM/pont/ contient deja
# plusieurs sauvegardes. Un `PONT_PY=...bak` passe a travers `f:pont/pont.py`
# sans etre inquiete. Ce qu'on sait DETECTER, on doit savoir le TUER : sinon la
# detection ne sert qu'a faire echouer le run.
#
# GARDE-FOUS. Jamais PID 1, jamais nous-memes, et on ANNONCE la ligne de commande
# de la victime avant de frapper : un kill silencieux par numero de port serait
# plus dangereux que le reste qu'il nettoie.
_td_kill_ports() {
    command -v ss >/dev/null 2>&1 || { mod_say "ss absent : passe par port impossible"; return 0; }
    local p pid cmd busy=0 named=0
    while IFS= read -r p; do
        _td_port_busy "$p" || continue
        busy=1
        for pid in $(ss -lnp "sport = :$p" 2>/dev/null \
                     | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
            named=1
            [ "$pid" -gt 1 ] 2>/dev/null || continue
            [ "$pid" = "$$" ] && continue
            cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
            mod_say "port $p tenu par PID $pid (${cmd:-cmdline illisible}) — kill -9"
            kill -9 "$pid" 2>/dev/null
        done
    done < <(_td_ports)
    # Une passe muette est indiscernable d'une passe absente : on dit POURQUOI
    # elle n'a rien fait. Le `ss` de busybox n'a pas -p et ne rend aucun pid=.
    if [ "$busy" = 1 ] && [ "$named" = 0 ]; then
        mod_say "ports occupes mais ss n'a rendu aucun pid= (ss sans -p, ou proprietaire hors de notre namespace) — inspectez : ss -lnp"
    fi
    return 0
}

mod_teardown_start() {
    # --- 1. tmux (cf. POURQUOI 3) --------------------------------------------
    if [ -n "${TMUX:-}" ]; then
        mod_say "lancé depuis tmux : kill-server évité, on ne touche qu'à la session ${TMUX_SESSION:-calypso}"
        tmux kill-session -t "${TMUX_SESSION:-calypso}" 2>/dev/null
    else
        tmux kill-server 2>/dev/null
    fi

    # --- 2. processus (cf. POURQUOI 2) ---------------------------------------
    _td_kill_all

    # --- 3. fichiers de rendez-vous et restes ---------------------------------
    local p
    while IFS= read -r p; do rm -f "$p" 2>/dev/null; done < <(_td_sockets)
    rm -f /tmp/osmocom_l2_* /tmp/irda.pty.link /tmp/irda_peer.pid 2>/dev/null
    rm -f /dev/shm/calypso_si.bin 2>/dev/null
    rm -f /dev/shm/calypso_kc 2>/dev/null            # cf. POURQUOI 4
    # ⛔ NE PAS EFFACER /dev/shm/pont.log ICI — essaye le 16/08, REVERTE le jour
    # meme. start-direct.sh ouvre la redirection `> /dev/shm/pont.log` AVANT de
    # passer la main a run.sh : quand ce teardown s'execute, le lanceur differe
    # du pont TIENT DEJA le descripteur. Un rm ne libere alors pas l'espace, il
    # DELIE l'inode — le pont ecrit ensuite dans un fichier invisible
    # (« /dev/shm/pont.log (deleted) » dans /proc/<pid>/fd/1) et le journal
    # disparait entierement. C'est le piege deja documente pour LOG_DIR,
    # transpose a /dev/shm. Le fichier n'a de toute facon pas besoin d'etre
    # efface : la redirection `>` le TRONQUE a chaque demarrage.
    rm -f /tmp/relay_continu.cfile /tmp/record.cfile /tmp/record.cfile.off /tmp/record.cfile.ring 2>/dev/null
    if [ -n "${RECORD_FILE:-}" ]; then
        rm -f "$RECORD_FILE" "$RECORD_FILE.off" "$RECORD_FILE.ring" 2>/dev/null
    fi
    rm -f "${RUN_DIR:-/tmp/calypso}/qemu.pid" 2>/dev/null
    mod_ok
}

# BARRIÈRE — remplace les `sleep 1` de L1555 et L1562 (cf. POURQUOI 1).
# Un kill -9 est asynchrone : le processus disparaît vite, mais le noyau garde
# ses sockets un instant, et un enfant peut survivre à son parent. On sonde donc
# les trois conséquences observables, et on RÉINSISTE une fois à mi-parcours :
# un processus relancé par un superviseur se voit ainsi, au lieu de se traduire
# par un port occupé dix modules plus loin.
mod_teardown_wait() {
    local half=$(( ${MOD_TIMEOUT[teardown]} / 2 ))
    if wait_until "$half" "machine propre" _td_clean; then mod_ok; return $MOD_RC_OK; fi
    mod_say "restes après $half s : $(_td_leftovers) — deuxième passe"
    _td_kill_all
    # Les motifs ont eu leur chance (une passe au start, une ici). Ce qui tient
    # encore un port n'a PAS de motif : on le prend par le port. C'est la seule
    # passe qu'un chemin inattendu ne peut pas mettre en defaut.
    _td_kill_ports
    if wait_until "$half" "machine propre" _td_clean; then mod_ok; return $MOD_RC_OK; fi
    mod_hint "identifiez le tenace : pgrep -a -f <motif> ; ss -lnp | grep <port>"
    mod_fail "restes du run précédent : $(_td_leftovers)"
    return $MOD_RC_FAIL
}

# Arrêt = même nettoyage, sans le tmux (l'opérateur peut vouloir garder ses
# panes pour lire les journaux après un ./run.sh --stop).
mod_teardown_stop() {
    _td_kill_all
    local p
    while IFS= read -r p; do rm -f "$p" 2>/dev/null; done < <(_td_sockets)
    return 0
}
