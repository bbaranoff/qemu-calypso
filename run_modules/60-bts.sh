# =============================================================================
#  60-bts — osmo-bts-trx : la station de base
# =============================================================================
#
#  RÔLE        Lance osmo-bts-trx sur la cfg TRX. Il parle TRXD au transceiver
#              (UDP 5700+ côté TRX, 5800+ côté BTS) et OML/RSL au BSC.
#              Legacy : run.sh.legacy L2002-2009 — trois lignes, aucune
#              vérification : une BTS qui n'arrive pas à joindre le BSC ou le
#              transceiver mourait en silence, et le mobile ne voyait jamais de
#              cellule sans que rien ne l'explique.
#
#  PRÉREQUIS   Transceiver prêt (module trx-ipc, ou sauté si on s'en passe),
#              cœur Osmocom démarré (le BSC accepte l'OML), cfg lisible.
#
#  SUCCÈS      Processus vivant ET VTY de la BTS (4241) joignable.
#              POURQUOI CE CRITÈRE : osmo-bts-trx sort tout de suite sur une
#              cfg invalide (phy inconnue, unit-id refusé) ; le VTY n'écoute
#              qu'après l'initialisation complète des phy/trx. C'est observable
#              sans dépendre d'une chaîne de journal, donc robuste aux versions.
#
#  CE QU'ON N'A PAS PU FIGER : le marqueur d'établissement RSL dans bts.log.
#  Aucun bts.log réel n'existe sur cette machine ; poser un `log_has` sur une
#  chaîne devinée aurait produit un faux échec permanent. On observe donc le
#  RSL de façon INFORMATIVE (mod_say), sans en faire une condition.
#
#  JOURNAL     $BTS_LOG (défaut $LOG_DIR/bts.log)
#
#  ATTENTION   osmo-bts-trx est aussi une unité systemd sur cette image, et
#              $OSMOCOM_CFG/status.sh l'arrête (module core). Si une unité est
#              active, mod_bts_status la voit et le module est sauté : il n'y
#              aura jamais deux BTS concurrentes.
# -----------------------------------------------------------------------------

MOD_REGISTER bts "Station de base osmo-bts-trx"
MOD_REQUIRED[bts]=0
MOD_DEPS[bts]="trx-ipc"
MOD_PROFILES[bts]="calypso hybrid"
MOD_TIMEOUT[bts]=30
MOD_ENABLED_IF[bts]='[ "${CALYPSO_SKIP_BTS:-0}" != "1" ]'

: "${OSMO_BTS_TRX:=}"
: "${BTS_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bts-trx.cfg}"
: "${BTS_LOG:=${LOG_DIR:-/root/calypso/logs}/bts.log}"
: "${OSMO_VTY_BTS:=4241}"

mod_bts_check() {
    if [ -z "$OSMO_BTS_TRX" ] || [ ! -x "$OSMO_BTS_TRX" ]; then
        OSMO_BTS_TRX="$(command -v osmo-bts-trx 2>/dev/null)"
    fi
    [ -n "$OSMO_BTS_TRX" ] && [ -x "$OSMO_BTS_TRX" ] || {
        mod_hint "installez osmo-bts (cible trx) ou posez OSMO_BTS_TRX=<chemin> ; CALYPSO_SKIP_BTS=1 pour vous en passer"
        mod_fail "osmo-bts-trx introuvable"
        return $MOD_RC_FAIL
    }
    [ -r "$BTS_CFG" ] || {
        mod_hint "attendu : <OSMOCOM_CFG>/osmo-bts-trx.cfg — ou posez BTS_CFG=<chemin>"
        mod_fail "configuration de la BTS illisible : $BTS_CFG"
        return $MOD_RC_FAIL
    }
    # Le BSC doit écouter l'OML, sinon la BTS boucle sur des reconnexions.
    # Non bloquant : c'est une information utile, pas une raison de ne pas
    # lancer (la BTS retente toute seule dès que le BSC arrive).
    have_port "${OSMO_VTY_BSC:-4242}" || mod_say "AVERTISSEMENT : le VTY du BSC (${OSMO_VTY_BSC:-4242}) ne répond pas — l'OML risque de boucler"
    mod_say "binaire=$OSMO_BTS_TRX cfg=$BTS_CFG"
    mod_ok
}

# -x : nom EXACT. `pgrep -f osmo-bts` attraperait osmo-bts-virtual ou une ligne
# de commande qui mentionne le binaire (un tail, un grep de l'opérateur).
# Le nom exact ne suffit plus : en profil hybride le side-car fait tourner un
# SECOND osmo-bts-trx (BTS#1), démarré avant celui-ci. Seul le fichier de
# configuration les distingue. Le PID enregistré passe en premier — c'est le
# seul témoin qui ne peut pas confondre deux processus homonymes.
_bts_est_le_notre() {   # _bts_est_le_notre <pid>
    tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -qF "$BTS_CFG"
}
mod_bts_status() {
    local pid; pid="$(cat "${RUN_DIR}/bts.pid" 2>/dev/null || echo 0)"
    if [ "$pid" != 0 ] && kill -0 "$pid" 2>/dev/null && _bts_est_le_notre "$pid"; then
        return 0
    fi
    local p
    for p in $(pgrep -x osmo-bts-trx 2>/dev/null); do
        _bts_est_le_notre "$p" && return 0
    done
    return 1
}

mod_bts_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" "$(dirname "$BTS_LOG")" 2>/dev/null || true
    : > "$BTS_LOG" 2>/dev/null || true
    stdbuf -oL -eL "$OSMO_BTS_TRX" -c "$BTS_CFG" >>"$BTS_LOG" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/bts.pid"
    mod_ok
}

# BARRIÈRE NEUVE — le legacy ne vérifiait rien du tout.
mod_bts_wait() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/bts.pid" 2>/dev/null || echo 0)"

    if ! wait_until "${MOD_TIMEOUT[bts]}" "VTY de la BTS ($OSMO_VTY_BTS)" have_port "$OSMO_VTY_BTS"; then
        modb_tail "$BTS_LOG" 25
        if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
            mod_hint "causes fréquentes : phy/instance absente de $BTS_CFG, ou unit-id déjà pris par une autre BTS"
            mod_fail "osmo-bts-trx s'est arrêté pendant son initialisation"
        else
            mod_hint "détail : $BTS_LOG"
            mod_fail "osmo-bts-trx tourne mais n'expose pas son VTY après ${MOD_TIMEOUT[bts]}s"
        fi
        return $MOD_RC_FAIL
    fi

    if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
        modb_tail "$BTS_LOG" 25
        mod_hint "une BTS résiduelle (unité systemd ?) tient le VTY : systemctl stop osmo-bts-trx"
        mod_fail "le VTY $OSMO_VTY_BTS répond, mais ce n'est PAS notre processus (il est mort)"
        return $MOD_RC_FAIL
    fi

    # Observation informative, jamais bloquante (cf. en-tête).
    if grep -qi 'rsl' "$BTS_LOG" 2>/dev/null; then
        mod_say "RSL mentionné dans $BTS_LOG (lien vers le BSC en cours d'établissement)"
    else
        mod_say "aucune mention de RSL dans $BTS_LOG pour l'instant — surveillez l'OML côté BSC"
    fi
    mod_ok
}

mod_bts_stop() {
    local pidf="${RUN_DIR:-/tmp/calypso}/bts.pid" pid
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    # PAS de `pkill -x` : il emporterait le BTS#1 du side-car. On ne tue que les
    # instances dont la ligne de commande porte NOTRE configuration.
    for _p in $(pgrep -x osmo-bts-trx 2>/dev/null); do
        _bts_est_le_notre "$_p" && kill "$_p" 2>/dev/null
    done
    rm -f "$pidf"
    return 0
}
