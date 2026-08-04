# =============================================================================
#  41-pty — résolution des PTY série alloués par QEMU (serial0 / serial1)
# =============================================================================
#
#  RÔLE        Publier OSMOCON_PTY (= UART modem, `serial0`) et CALYPSO_IRDA_PTY
#              (= UART IrDA, `serial1`) pour les modules qui suivent.
#              QEMU est lancé avec `-serial pty -serial pty` : il alloue DEUX
#              pseudo-terminaux, dont le numéro change à chaque démarrage.
#
#  POURQUOI CE MODULE EXISTE — c'est le défaut le plus silencieux de la
#  découpe actuelle. Le legacy extrayait le PTY (L1833-1859) et le passait à
#  osmocon (`-p $OSMOCON_SERIAL`, L1930). Le module 50-osmocon, lui, utilise
#  `${OSMOCON_PTY:-/dev/pts/0}` : une DEVINETTE. Sur /dev/pts/0 (souvent le
#  terminal de quelqu'un d'autre), osmocon ne reçoit jamais le prompt romload,
#  le firmware n'est jamais chargé, et TOUT ce qui suit échoue sans qu'aucune
#  étape n'affiche autre chose que « OK ». Ce module produit la valeur ; il ne
#  modifie ni run.sh ni 50-osmocon (qui la consomme déjà par son `:-`).
#
#  PRÉREQUIS   QEMU démarré (module qemu).
#  SUCCÈS      $OSMOCON_PTY est un périphérique CARACTÈRE existant.
#  JOURNAL     $LOG_DIR/mod/pty.log
#
#  SOURCE DE VÉRITÉ : le moniteur QEMU (`info chardev`), pas le journal.
#  POURQUOI : le garde-fou de 40-qemu.sh (L35-43) RÉÉCRIT qemu.log au-delà du
#  plafond — la ligne « redirected to /dev/pts/N (label serial0) » disparaît
#  alors du fichier (constaté : 36 Mo de qemu.log, `grep -c pts` = 0). Le
#  journal reste utilisé en SECOURS, car il est correct tant que le plafond
#  n'est pas atteint, et il fonctionne même si le moniteur est déjà occupé
#  (`server,nowait` n'accepte qu'un client à la fois).
# -----------------------------------------------------------------------------

MOD_REGISTER pty "PTY série de l'émulateur"
MOD_REQUIRED[pty]=1
MOD_DEPS[pty]="qemu"
MOD_PROFILES[pty]="calypso hybrid"
MOD_TIMEOUT[pty]=60          # legacy : 60 tentatives × 1 s (L1839)

: "${QEMU_MON_SOCK:=${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}"

# _pty_from_monitor <socket> <label> — interroge le moniteur HMP.
# ⚠️ NE JAMAIS envoyer `quit` sur ce socket : la commande HMP `quit` ARRÊTE la
# machine émulée. On envoie `info chardev` et on laisse socat fermer de
# lui-même (-t 1 = délai de grâce après EOF, sinon la réponse est perdue).
_pty_from_monitor() {
    local sock="$1" label="$2"
    [ -S "$sock" ] || return 1
    command -v socat >/dev/null 2>&1 || return 1
    printf 'info chardev\n' \
        | timeout 3 socat -t 1 - "UNIX-CONNECT:$sock" 2>/dev/null \
        | grep -E "^${label}:" \
        | grep -oE '/dev/pts/[0-9]+' \
        | head -1
}

# _pty_from_log <journal> <label> — secours : la ligne d'allocation de QEMU.
_pty_from_log() {
    local log="$1" label="$2"
    [ -r "$log" ] || return 1
    grep -a "redirected to /dev/pts/.* (label ${label})" "$log" 2>/dev/null \
        | grep -oE '/dev/pts/[0-9]+' \
        | head -1
}

# Résout ce qui n'est pas déjà résolu. Rend 0 dès que serial0 est utilisable.
# Une valeur posée par l'opérateur (OSMOCON_PTY=... ./run.sh) est respectée :
# on ne recalcule que si elle est vide ou ne désigne plus un périphérique.
_pty_resolve() {
    local log="${LOG_DIR:-/root/calypso/logs}/qemu.log" p
    if [ -z "${OSMOCON_PTY:-}" ] || [ ! -c "${OSMOCON_PTY:-/nonexistent}" ]; then
        p="$(_pty_from_monitor "$QEMU_MON_SOCK" serial0)"
        [ -n "$p" ] || p="$(_pty_from_log "$log" serial0)"
        [ -n "$p" ] && OSMOCON_PTY="$p"
    fi
    if [ -z "${CALYPSO_IRDA_PTY:-}" ] || [ ! -c "${CALYPSO_IRDA_PTY:-/nonexistent}" ]; then
        p="$(_pty_from_monitor "$QEMU_MON_SOCK" serial1)"
        [ -n "$p" ] || p="$(_pty_from_log "$log" serial1)"
        [ -n "$p" ] && CALYPSO_IRDA_PTY="$p"
    fi
    [ -n "${OSMOCON_PTY:-}" ] && [ -c "$OSMOCON_PTY" ]
}

mod_pty_check() {
    if ! command -v socat >/dev/null 2>&1; then
        # Pas bloquant : le journal suffit tant qu'il n'a pas été tronqué.
        mod_say "socat absent : lecture du moniteur impossible, secours par le journal seulement"
    fi
    [ -S "$QEMU_MON_SOCK" ] || [ -r "${LOG_DIR:-/root/calypso/logs}/qemu.log" ] || {
        mod_hint "le module qemu doit avoir démarré : ./run.sh --only qemu"
        mod_fail "ni moniteur ($QEMU_MON_SOCK) ni journal QEMU : rien à interroger"
        return $MOD_RC_FAIL
    }
    mod_ok
}

# Idempotence : une valeur déjà utilisable (héritée de l'environnement ou d'un
# passage précédent) fait sauter le module.
# PAS de « déjà démarré » : c'est une DÉCOUVERTE, pas un service. QEMU alloue un
# nouveau PTY à chaque démarrage ; se fier à une valeur précédente fait parler
# osmocon à un PTY orphelin, qui existe toujours mais n'est plus relié à rien —
# le firmware ne répond alors jamais, sans qu'aucune erreur ne soit levée.
mod_pty_status() { return 1; }


# Rien à lancer : ce module RÉSOUT. Première tentative ici (cas nominal, le PTY
# est déjà alloué quand la barrière de 40-qemu est passée) ; la boucle est dans
# mod_pty_wait.
mod_pty_start() { _pty_resolve; mod_ok; }

# BARRIÈRE — remplace le `sleep 1` × 60 du legacy (L1851) : on sonde la
# PRÉSENCE du PTY, et on surveille QEMU en même temps (legacy L1845), pour
# distinguer « QEMU est mort pendant l'init » de « QEMU est lent ».
mod_pty_wait() {
    local deadline=$(( SECONDS + ${MOD_TIMEOUT[pty]} ))
    local qpid; qpid="$(cat "${RUN_DIR:-/tmp/calypso}/qemu.pid" 2>/dev/null || echo 0)"
    while (( SECONDS < deadline )); do
        if _pty_resolve; then
            export OSMOCON_PTY
            # Nom historique attendu par certains scripts auxiliaires.
            OSMOCON_SERIAL="$OSMOCON_PTY"; export OSMOCON_SERIAL
            [ -n "${CALYPSO_IRDA_PTY:-}" ] && export CALYPSO_IRDA_PTY
            mod_say "serial0=$OSMOCON_PTY serial1=${CALYPSO_IRDA_PTY:-<non alloué>}"
            mod_ok
            return $MOD_RC_OK
        fi
        if [ "$qpid" != 0 ] && ! kill -0 "$qpid" 2>/dev/null; then
            modb_tail "${LOG_DIR:-/root/calypso/logs}/qemu.log" 20
            mod_hint "cause typique : machine type, ROM du DSP ou firmware invalide"
            mod_fail "QEMU (pid $qpid) s'est arrêté avant d'allouer son PTY"
            return $MOD_RC_FAIL
        fi
        sleep 0.5
    done
    mod_hint "vérifiez que QEMU est lancé avec « -serial pty -serial pty » ; sinon : socat - UNIX-CONNECT:$QEMU_MON_SOCK puis « info chardev »"
    mod_fail "aucun PTY serial0 après ${MOD_TIMEOUT[pty]}s (moniteur $QEMU_MON_SOCK et journal muets)"
    return $MOD_RC_FAIL

}

# Aucun effet de bord à défaire : on n'a rien créé, seulement lu.
mod_pty_stop() { return 0; }
