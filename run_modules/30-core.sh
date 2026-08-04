# =============================================================================
#  30-core — le cœur Osmocom (STP, HLR, MGW, MSC, BSC, GGSN, SGSN, PCU)
# =============================================================================
#
#  RÔLE        (Re)démarre les démons du réseau cœur, dans leur ordre de
#              dépendance, via les scripts fournis par l'image :
#              $OSMOCOM_CFG/status.sh stop  puis  $OSMOCOM_CFG/osmo-start.sh.
#              Legacy : run.sh.legacy L1564-1565 — DEUX lignes, `|| true` sur
#              les deux. Un cœur mort ne se voyait donc nulle part : le mobile
#              partait en RACH, le LU échouait, et rien dans le lancement ne
#              disait que le HLR n'avait jamais démarré.
#
#  PRÉREQUIS   systemctl utilisable, $OSMOCOM_CFG/{status.sh,osmo-start.sh}.
#
#  SUCCÈS      Les VTY HLR (4258), MSC (4254) et BSC (4242) ACCEPTENT une
#              connexion. POURQUOI CE CRITÈRE : `systemctl start` rend la main
#              dès le fork, il ne dit rien de l'état réel du démon — une cfg
#              invalide laisse le service « started » une fraction de seconde.
#              Le VTY, lui, n'écoute qu'une fois l'initialisation terminée.
#              C'est la seule preuve observable sans parser les journaux.
#              On exige les TROIS : le HLR seul ne prouve pas que la chaîne
#              A-interface (MSC↔BSC) est montée, et c'est elle qui porte le LU.
#
#  JOURNAL     $LOG_DIR/osmo-core.log (sortie de osmo-start.sh) ;
#              le détail par démon reste dans journalctl -u osmo-<nom>.
# -----------------------------------------------------------------------------

# =============================================================================
#  OUTILLAGE COMMUN DU BLOC B — défini ici (premier fichier du bloc), utilisé
#  par 31/41/42/55/56/57/60. run.sh source TOUS les modules avant d'exécuter
#  quoi que ce soit, et dans l'ordre des numéros : ces fonctions sont donc
#  disponibles dans tous les modules qui suivent, y compris sous --only.
#  Préfixe `modb_` volontairement distinct de `mod_<slug>_` pour ne pouvoir
#  entrer en collision avec aucun nom de fonction dérivé d'un slug.
# =============================================================================

# modb_dep_known <slug...> — ne garde que les dépendances RÉELLEMENT déclarées.
# POURQUOI : les modules de préparation (bloc A : teardown, logs, config,
# mobile-cfg) peuvent ne pas encore être présents dans l'arbre. run.sh traite
# une dépendance inconnue comme non satisfaite et saute le module — le bloc B
# entier deviendrait invisible. On déclare donc la dépendance quand elle existe,
# et on l'ignore sinon : l'ordre reste exact dès que le bloc A est en place,
# sans jamais rendre l'arbre inutilisable entre-temps.
if ! declare -F modb_dep_known >/dev/null 2>&1; then
modb_dep_known() {
    local d out=""
    for d in "$@"; do
        [ -n "${MOD_DESC[$d]+x}" ] && out="$out $d"
    done
    printf '%s' "${out# }"
}
fi

# modb_have_udp <port> — le port UDP est-il BINDÉ localement ?
# have_port (mod.sh) ouvre un /dev/tcp : inutilisable pour TRXD/IQ, qui sont en
# UDP. On lit /proc/net/udp{,6}, toujours présent, et on compare le port du
# champ `local_address` (champ 2, format HEX). Pas de dépendance à ss/netstat,
# pas de gawk (strtonum) : comparaison de chaînes hexadécimales.
if ! declare -F modb_have_udp >/dev/null 2>&1; then
modb_have_udp() {
    local port="$1" hex
    hex=$(printf '%04X' "$port" 2>/dev/null) || return 1
    awk -v h="$hex" '
        NR > 1 { split($2, a, ":"); if (toupper(a[2]) == h) { found = 1; exit } }
        END { exit(found ? 0 : 1) }
    ' /proc/net/udp /proc/net/udp6 2>/dev/null
}
fi

# modb_tail <fichier> [n] — recopie la fin d'un journal de service dans le
# journal du module. run.sh n'affiche que le chemin du journal du MODULE en cas
# d'échec : sans cela, la vraie cause (une ligne d'erreur du démon) resterait
# dans un autre fichier que l'opérateur ne saurait pas ouvrir.
if ! declare -F modb_tail >/dev/null 2>&1; then
modb_tail() {
    local f="$1" n="${2:-20}"
    if [ ! -r "$f" ]; then mod_say "journal absent ou illisible : $f"; return 0; fi
    mod_say "--- $n dernières lignes de $f ---"
    tail -n "$n" "$f" 2>/dev/null
    mod_say "--- fin de $f ---"
}
fi

# =============================================================================

MOD_REGISTER core "Cœur Osmocom (HLR/MSC/BSC/MGW/STP)"
MOD_REQUIRED[core]=1
MOD_DEPS[core]="$(modb_dep_known teardown)"
MOD_PROFILES[core]="calypso hybrid core"
MOD_TIMEOUT[core]=120

# DOUBLON DE ROLE — ce module lance le coeur EN BLOC (osmo-start.sh). Un bloc de
# modules a grain fin (10-stp, 11-hlr, 12-mgw, 13-msc, 14-bsc, ...) fait la meme
# chose demon par demon, avec un verdict par demon. Les deux dans le meme plan,
# c'est deux fois le meme demarrage — et surtout `mod_core_start` commence par
# `status.sh stop`, donc il ETEINDRAIT ce que les modules a grain fin viennent
# d'allumer (visible seulement sous --force, `mod_core_status` masquant le reste
# du temps le probleme derriere un SKIP). On cede donc la place des que le bloc
# decompose est enregistre. La porte est auto-adaptative : si ce bloc disparait
# de l'arbre, ce module reprend son role sans qu'on ait rien a modifier.
MOD_ENABLED_IF[core]='[ -z "${MOD_DESC[stp]+x}" ] && [ -z "${MOD_DESC[hlr]+x}" ] && [ -z "${MOD_DESC[msc]+x}" ] && [ -z "${MOD_DESC[bsc]+x}" ]'

: "${OSMO_CORE_STATUS:=${OSMOCOM_CFG:-/etc/osmocom}/status.sh}"
: "${OSMO_CORE_START:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-start.sh}"
: "${OSMO_CORE_LOG:=${LOG_DIR:-/root/calypso/logs}/osmo-core.log}"
# Ports VTY, tels que déclarés dans l'en-tête de osmo-start.sh.
: "${OSMO_VTY_HLR:=4258}"
: "${OSMO_VTY_MSC:=4254}"
: "${OSMO_VTY_BSC:=4242}"

_core_vty_up() {
    have_port "$OSMO_VTY_HLR" && have_port "$OSMO_VTY_MSC" && have_port "$OSMO_VTY_BSC"
}

mod_core_check() {
    command -v systemctl >/dev/null 2>&1 || {
        mod_hint "ce cœur est piloté par systemd ; hors conteneur systemd, lancez les démons à la main et relancez avec --skip core"
        mod_fail "systemctl introuvable"
        return $MOD_RC_FAIL
    }
    [ -x "$OSMO_CORE_START" ] || {
        mod_hint "réglez OSMOCOM_CFG dans environnement/paths.env (attendu : <OSMOCOM_CFG>/osmo-start.sh)"
        mod_fail "script de démarrage du cœur introuvable : $OSMO_CORE_START"
        return $MOD_RC_FAIL
    }
    mod_ok
}

# Idempotence : les trois VTY répondent déjà = le cœur tourne, on n'y touche pas.
mod_core_status() { _core_vty_up; }

mod_core_start() {
    mkdir -p "$(dirname "$OSMO_CORE_LOG")" 2>/dev/null || true
    : > "$OSMO_CORE_LOG" 2>/dev/null || true

    # Arrêt d'abord : le legacy le faisait (L1564) pour repartir d'un cœur frais
    # — un osmo-bsc qui garde une A-interface d'un run précédent refuse le
    # nouveau BTS. `|| true` : status.sh est en `set -e` et sort non nul dès
    # qu'un service était déjà arrêté, ce qui n'est pas une erreur ici.
    if [ -x "$OSMO_CORE_STATUS" ]; then
        timeout 60 "$OSMO_CORE_STATUS" stop >>"$OSMO_CORE_LOG" 2>&1 || true
    fi

    timeout 180 "$OSMO_CORE_START" >>"$OSMO_CORE_LOG" 2>&1
    local rc=$?
    case $rc in
        0) mod_ok ;;
        124) mod_hint "journalctl -u osmo-hlr -n 50 : un démon bloque au démarrage"
             mod_fail "osmo-start.sh n'a pas rendu la main en 180 s"
             return $MOD_RC_FAIL ;;
        *) modb_tail "$OSMO_CORE_LOG" 25
           mod_hint "détail : $OSMO_CORE_LOG, puis journalctl -u osmo-hlr -u osmo-msc -u osmo-bsc"
           mod_fail "osmo-start.sh a échoué (code $rc) — le HLR est indispensable, il sort 1 s'il ne monte pas"
           return $MOD_RC_FAIL ;;
    esac
}

# BARRIÈRE — remplace la confiance aveugle du legacy (L1564-1565, `|| true`).
# osmo-start.sh attend déjà chaque VTY, mais avec `|| true` sur STP/MGW/MSC/BSC :
# il peut rendre la main avec un MSC ou un BSC absent. On revérifie donc ici, et
# on nomme celui qui manque — c'est la différence entre « le cœur est cassé » et
# « osmo-msc n'écoute pas sur 4254 ».
mod_core_wait() {
    wait_until "${MOD_TIMEOUT[core]}" "VTY du cœur Osmocom" _core_vty_up && { mod_ok; return $MOD_RC_OK; }

    local manque=""
    have_port "$OSMO_VTY_HLR" || manque="$manque HLR:$OSMO_VTY_HLR"
    have_port "$OSMO_VTY_MSC" || manque="$manque MSC:$OSMO_VTY_MSC"
    have_port "$OSMO_VTY_BSC" || manque="$manque BSC:$OSMO_VTY_BSC"
    modb_tail "$OSMO_CORE_LOG" 25
    mod_hint "journalctl -u osmo-hlr -u osmo-msc -u osmo-bsc -n 50 --no-pager ; ou $OSMO_CORE_STATUS status"
    mod_fail "VTY injoignable après ${MOD_TIMEOUT[core]}s :$manque"
    return $MOD_RC_FAIL
}

mod_core_stop() {
    [ -x "$OSMO_CORE_STATUS" ] && timeout 60 "$OSMO_CORE_STATUS" stop >>"$OSMO_CORE_LOG" 2>&1
    return 0
}
