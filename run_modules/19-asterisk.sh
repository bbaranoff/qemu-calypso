# =============================================================================
#  19-asterisk — le PBX SIP qui porte les appels
# =============================================================================
#  RÔLE      terminaison des appels : osmo-sip-connector traduit le MNCC du MSC
#            en SIP et le pousse vers Asterisk. Sans PBX, un appel MS→MS n'est
#            jamais raccordé. Optionnel : ni le rattachement ni le SMS n'en
#            dépendent.
#  PRÉREQUIS binaire asterisk ; /etc/asterisk/asterisk.conf.
#  SUCCÈS    la console de contrôle RÉPOND (« core show uptime ») — c'est le
#            seul critère qui prouve qu'Asterisk a fini de charger ses modules,
#            là où « actif » ne prouve que le fork. Plus : aucun redémarrage.
#  JOURNAL   journalctl -u asterisk ; /var/log/asterisk/
#
#  NOTE l'ancien lancement supprimait /var/lib/asterisk/astdb.sqlite3 à chaque
#  démarrage (scripts/run.sh:455). Ce module ne le fait PAS : détruire une base
#  n'est pas une étape de démarrage.
# -----------------------------------------------------------------------------
: "${MODDIR:=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
. "$MODDIR/_lib/core.sh"

MOD_REGISTER asterisk "Cœur — Asterisk (PBX SIP)"
MOD_REQUIRED[asterisk]=0
MOD_PROFILES[asterisk]="calypso faketrx hybrid core"
MOD_JOURNAL[asterisk]="asterisk"
MOD_TIMEOUT[asterisk]=45
MOD_ENABLED_IF[asterisk]='[ "${NO_OSMO_START:-0}" != 1 ] && [ "${CORE_VOICE:-1}" = 1 ]'

: "${ASTERISK_UNIT:=asterisk}"
: "${ASTERISK_CFG:=/etc/asterisk/asterisk.conf}"

_ast_cli() { asterisk -rx "$1" 2>/dev/null; }
_ast_ready() { _ast_cli "core show uptime" | grep -qi 'uptime'; }

mod_asterisk_check() {
    command -v asterisk >/dev/null 2>&1 || {
        mod_hint "installez asterisk, ou désactivez la voix : CORE_VOICE=0"
        mod_fail "binaire asterisk introuvable"; return $MOD_RC_FAIL; }
    [ -r "$ASTERISK_CFG" ] || {
        mod_fail "configuration illisible : $ASTERISK_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_asterisk_status() { _ast_ready; }

mod_asterisk_start() {
    # [2026-08-09] LANCEMENT DIRECT — ce module ne passe PLUS par systemd.
    #
    # POURQUOI. L'unité systemd lançait /usr/sbin/asterisk -g -f -p -U asterisk
    # DERRIÈRE ce module. Mesuré : l'ancien scripts/run.sh tuait Asterisk puis
    # lançait `asterisk -cvvv` en tmux, et systemd le relançait aussitôt en
    # démon — le `-cvvv` finissait en simple `rasterisk` attaché à l'instance
    # de systemd. Deux propriétaires pour un seul /etc/asterisk.
    #
    # ⚠️ core_svc_start ne retombe sur le lancement direct que si l'unité
    # N'EXISTE PAS. Une unité MASQUÉE existe toujours : `systemctl start`
    # échoue, et le module échouait avec elle. On ne lui laisse donc plus le
    # choix — on lance le binaire, un point c'est tout.
    if _ast_ready; then mod_say "déjà actif — on ne relance pas"; mod_ok; return 0; fi

    local bin pf log
    bin="$(command -v asterisk)"
    pf="$(core_pidfile "$ASTERISK_UNIT")"
    log="${LOG_DIR}/${ASTERISK_UNIT}.log"
    mkdir -p "$RUN_DIR" "$LOG_DIR" 2>/dev/null || true

    # -f : reste au premier plan (pas de double fork), donc le PID qu'on note
    #      est bien celui d'Asterisk et core_alive peut le suivre.
    # -U : abandonne les privilèges vers l'utilisateur asterisk.
    # On n'utilise ni -p (priorité temps réel : refusée sans les capacités, et
    # elle ne sert à rien ici) ni -g (dump core).
    mod_say "lancement direct : $bin -f -U asterisk"
    setsid "$bin" -f -U asterisk >>"$log" 2>&1 </dev/null &
    printf '%s\n' "$!" > "$pf"

    sleep 1
    if ! kill -0 "$(cat "$pf" 2>/dev/null)" 2>/dev/null; then
        mod_hint "tail -30 $log"
        mod_fail "asterisk est mort dans la seconde qui a suivi le lancement"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# BARRIÈRE — Asterisk met plusieurs secondes à charger ses modules ; la socket
# de contrôle existe avant qu'il ne réponde. On interroge donc la CLI.
mod_asterisk_wait() {
    if ! wait_until "${MOD_TIMEOUT[asterisk]}" "console Asterisk" _ast_ready; then
        mod_hint "asterisk -rx 'core show uptime' ; journalctl -u $ASTERISK_UNIT -n 40"
        return $MOD_RC_FAIL
    fi
    # Le contrôle « a-t-il redémarré ? » reposait sur le compteur NRestarts de
    # systemd, qui n'a plus de sens en lancement direct : personne ne relance
    # Asterisk, donc une mort est définitive et se voit au PID. On vérifie donc
    # que le PID noté au démarrage est TOUJOURS celui qui tourne — un Asterisk
    # remplacé par un autre serait invisible au seul test « la CLI répond ».
    local pf; pf="$(core_pidfile "$ASTERISK_UNIT")"
    if [ -f "$pf" ] && ! kill -0 "$(cat "$pf" 2>/dev/null)" 2>/dev/null; then
        mod_hint "tail -50 ${LOG_DIR}/${ASTERISK_UNIT}.log"
        mod_fail "le PID lancé n'existe plus : Asterisk est mort ou a été remplacé"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_asterisk_stop() {
    _ast_cli "core stop now" >/dev/null 2>&1
    core_svc_stop "$ASTERISK_UNIT" ""
}
