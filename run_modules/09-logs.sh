# =============================================================================
#  09-logs — arborescence des journaux, archivage, horodatage, symlinks
# =============================================================================
#
#  RÔLE
#    Préparer tout ce dans quoi les blocs suivants vont ÉCRIRE : le répertoire
#    de journaux et de captures, le nom de chaque journal de service, le
#    filtre d'horodatage, l'archivage du journal osmocon du run précédent, et
#    les deux symlinks que QEMU exige. Reprend run.sh.legacy #17 (L1426-1446),
#    #18 (L1448-1473), #20 (L1501-1516) et #22 (L1547-1554).
#
#  PRÉREQUIS  : 05-config.
#  SUCCÈS     : $LOG_DIR est réellement inscriptible (prouvé par une écriture,
#               pas par un test de permission) ; le filtre d'horodatage est
#               exécutable ; les deux symlinks /tmp/qemu-*-tx.raw pointent dans
#               $LOG_DIR.
#  JOURNAL    : $LOG_DIR/mod/logs.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI DES SYMLINKS /tmp/qemu-{modem,irda}-tx.raw.
#     QEMU écrit ces deux fichiers à un chemin CODÉ EN DUR dans /tmp
#     (calypso_uart.c L693-695), sans borne de taille et sans jamais les
#     effacer : entre deux runs ils remplissaient le tmpfs, les écritures
#     échouaient, le firmware perdait la synchro et gelait. On les remplace par
#     des liens vers $LOG_DIR : QEMU suit le lien et écrit sur le disque, sans
#     recompilation. C'est la raison d'être exacte des lignes L1549-1554 du
#     legacy, conservée telle quelle.
#
#  2. POURQUOI ON ARCHIVE osmocon.log AVANT DE L'EFFACER.
#     C'est le seul journal qui porte le handshake romload complet ; l'effacer
#     à chaque run interdisait de comparer un run qui monte et un run qui
#     échoue. On archive avec un horodatage (= identifiant de run) et on borne
#     à dix archives, sinon elles remplissaient le volume (legacy L1502-1512).
#
#  3. CE MODULE N'EFFACE JAMAIS $LOG_DIR/mod/.
#     Les journaux des modules sont ouverts par run.sh AVANT que ce module ne
#     tourne, et écrits pendant toute la séquence. Le legacy effaçait « les
#     logs » en bloc ; ici la liste est nominative, service par service.
#
#  4. LE FILTRE D'HORODATAGE VA DANS $RUN_DIR, PAS DANS /tmp.
#     Le legacy l'écrivait en /tmp/calypso_tslog.py, partagé entre toutes les
#     instances. Sous $RUN_DIR il suit la même racine que les sockets et les
#     PID : une seule variable à changer pour isoler deux piles.
# -----------------------------------------------------------------------------

MOD_REGISTER logs "Journaux et espace de travail"
MOD_REQUIRED[logs]=1
MOD_DEPS[logs]="config"
MOD_PROFILES[logs]="calypso faketrx hybrid core"
MOD_TIMEOUT[logs]=10

: "${LOG_ARCHIVES_KEEP:=10}"      # nombre d'archives osmocon conservées (legacy L1512)

mod_logs_check() {
    # Lecture seule stricte : on ne crée rien ici, on vérifie seulement qu'un
    # parent existant est inscriptible. La création réelle est dans start.
    local d="${LOG_DIR:-}" p
    [ -n "$d" ] || { mod_fail "LOG_DIR non défini"; return $MOD_RC_FAIL; }
    p="$d"
    while [ ! -d "$p" ] && [ "$p" != "/" ] && [ "$p" != "." ]; do p="$(dirname "$p")"; done
    if [ ! -w "$p" ]; then
        mod_hint "posez LOG_DIR sur un chemin inscriptible (environnement/paths.env, LOG_DIR)"
        mod_fail "aucun parent inscriptible pour LOG_DIR=$d (bloqué à $p)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# « Déjà démarré » n'a pas de sens pour un module qui prépare des chemins : on
# veut l'archivage et la purge à CHAQUE run.
mod_logs_status() { return $MOD_RC_FAIL; }

mod_logs_start() {
    # Liens de compatibilité vers /root. Les journaux vivent dans $LOG_DIR
    # depuis le refactor, mais les habitudes et les notes pointent encore sur
    # /root/<nom>.log. Des LIENS et non des copies : rien à synchroniser, rien
    # qui puisse diverger. CALYPSO_LIENS_ROOT=0 pour s'en passer.
    if [ "${CALYPSO_LIENS_ROOT:-1}" = 1 ] && [ -w /root ]; then
        local _n _cible _lien _faits=0
        for _n in qemu osmocon mobile bts sidecar-mobile grgsm_decode \
                  calypso-ipc-device osmo-trx-ipc; do
            _cible="$LOG_DIR/$_n.log"; _lien="/root/$_n.log"
            # On ne remplace QUE nos propres liens : un vrai fichier posé là par
            # quelqu'un d'autre ne doit pas disparaître sans un mot.
            if [ -e "$_lien" ] && [ ! -L "$_lien" ]; then
                mod_say "⚠ /root/$_n.log est un vrai fichier — laissé en place"
                continue
            fi
            ln -sfn "$_cible" "$_lien" 2>/dev/null && _faits=$(( _faits + 1 ))
        done
        [ "$_faits" -gt 0 ] && mod_say "$_faits liens de compatibilité dans /root (→ $LOG_DIR)"
    fi

    # Journaux des services du cœur (osmo-*). Répertoire volontairement HORS de
    # $LOG_DIR : le teardown efface celui-ci, et un service déjà lancé garderait
    # un descripteur sur un fichier supprimé — osmocom ne rouvre ses fichiers que
    # sur SIGHUP. Le propriétaire doit être l'utilisateur des services, sinon la
    # cible « log file » de /etc/osmocom/*.cfg reste silencieusement vide.
    : "${OSMO_LOG_DIR:=/var/log/osmocom}"
    : "${OSMO_LOG_USER:=osmocom}"
    if [ ! -d "$OSMO_LOG_DIR" ]; then
        mkdir -p "$OSMO_LOG_DIR" 2>/dev/null || true
    fi
    if id "$OSMO_LOG_USER" >/dev/null 2>&1; then
        chown "$OSMO_LOG_USER":"$OSMO_LOG_USER" "$OSMO_LOG_DIR" 2>/dev/null || true
    fi
    if [ -d "$OSMO_LOG_DIR" ]; then
        mod_say "journaux des services : $OSMO_LOG_DIR (propriétaire $OSMO_LOG_USER)"
    else
        mod_say "⚠ $OSMO_LOG_DIR absent — les services n'écriront que sur journald"
    fi

    mkdir -p "$LOG_DIR" "$LOG_DIR/mod" "${RUN_DIR:-/tmp/calypso}" "${CAPTURE_DIR:-$RUN_DIR/captures}" 2>/dev/null

    # --- noms des journaux (legacy L1434-1446) -------------------------------
    : "${QEMU_LOG:=$LOG_DIR/qemu.log}"
    : "${OSMOCON_LOG:=$LOG_DIR/osmocon.log}"
    : "${MOBILE_LOG:=$LOG_DIR/mobile.log}"
    : "${BTS_LOG:=$LOG_DIR/bts.log}"
    : "${L2_LOG:=$LOG_DIR/l2_client.log}"
    : "${OSMO_TRX_IPC_LOG:=$LOG_DIR/osmo-trx-ipc.log}"
    : "${IPC_DEVICE_LOG:=$LOG_DIR/calypso-ipc-device.log}"
    : "${FW_IRDA_LOG:=$LOG_DIR/fw-irda.log}"
    : "${DEMOD_BRIDGE_LOG:=$LOG_DIR/demod_bridge.log}"
    : "${RECORD_DRAIN_LOG:=$LOG_DIR/record_drain.log}"
    : "${GRGSM_DECODE_LOG:=$LOG_DIR/grgsm_decode.log}"

    # Ring d'enregistrement I/Q. CALYPSO_RECORD_FILE est lu par record_drain.py
    # (opt-gsm-scripts/record_drain.py L15, défaut /tmp/record.cfile) et par lui
    # seul : aucune lecture côté modèle C, donc le poser ne touche pas
    # l'émulation.
    : "${CALYPSO_RECORD_FILE:=$LOG_DIR/record.cfile}"
    RECORD_FILE="$CALYPSO_RECORD_FILE"
    export CALYPSO_RECORD_FILE

    # --- archivage + purge du journal osmocon (cf. POURQUOI 2) ---------------
    if [ -s "$OSMOCON_LOG" ]; then
        local arch="${OSMOCON_LOG%.log}.$(date +%Y%m%d_%H%M%S).log"
        if mv "$OSMOCON_LOG" "$arch" 2>/dev/null; then
            mod_say "osmocon.log du run précédent archivé -> $arch"
        fi
    fi
    # `|| true` : `ls` échoue quand il n'y a aucune archive, et run.sh tourne
    # sous `pipefail`.
    ls -1t "${OSMOCON_LOG%.log}".[0-9]*.log 2>/dev/null \
        | tail -n +$((LOG_ARCHIVES_KEEP + 1)) | xargs -r rm -f 2>/dev/null || true

    # --- remise à zéro des journaux de service (liste NOMINATIVE, POURQUOI 3) -
    # TRONCATURE (`: >`) et NON suppression (`rm -f`). Ce module tourne AVANT le
    # module teardown : un processus du run précédent peut encore tenir un
    # descripteur sur ces fichiers. Supprimé, il continue d'écrire dans un inode
    # délié — ses dernières lignes, celles qui expliquent pourquoi il n'est pas
    # mort, sont perdues, et le fichier réapparaît vide sans que rien ne le dise.
    # Tronqué, le fichier reste le même : on garde ce qu'il écrit encore.
    local _f
    for _f in "$QEMU_LOG" "$OSMOCON_LOG" "$MOBILE_LOG" "$BTS_LOG" "$L2_LOG" \
              "$OSMO_TRX_IPC_LOG" "$IPC_DEVICE_LOG" "$FW_IRDA_LOG" \
              "$DEMOD_BRIDGE_LOG" "$RECORD_DRAIN_LOG" "$GRGSM_DECODE_LOG"; do
        [ -n "$_f" ] && : > "$_f" 2>/dev/null
    done

    # --- filtre d'horodatage (legacy L1448-1473, cf. POURQUOI 4) -------------
    TSLOG_SCRIPT="${RUN_DIR:-/tmp/calypso}/tslog.py"
    cat > "$TSLOG_SCRIPT" <<'PYEOF'
#!/usr/bin/env python3
"""Prefixe stdin avec `<epoch_sec> +<rel_sec>s ` et flush ligne par ligne.

Permet de comparer deux journaux colonne a colonne et de voir les derives
temporelles entre QEMU, osmocon et le client L2. Le prefixe est en DEBUT de
ligne : les `grep -E` des tests, qui cherchent en milieu de ligne, ne sont pas
perturbes.
"""
import sys, time
t0 = time.time()
for line in sys.stdin:
    t = time.time()
    sys.stdout.write(f"{t:.3f} +{t-t0:.3f}s {line}")
    sys.stdout.flush()
PYEOF
    chmod +x "$TSLOG_SCRIPT" 2>/dev/null
    if [ "${CALYPSO_LOG_TS:-1}" = "1" ]; then
        TSLOG="python3 -u $TSLOG_SCRIPT"
    else
        TSLOG="cat"
        mod_say "CALYPSO_LOG_TS=0 : journaux non horodatés"
    fi
    export TSLOG TSLOG_SCRIPT

    # --- symlinks imposés par QEMU (cf. POURQUOI 1) --------------------------
    local n
    for n in qemu-modem-tx.raw qemu-irda-tx.raw; do
        rm -f "/tmp/$n" "$LOG_DIR/$n" 2>/dev/null
        ln -s "$LOG_DIR/$n" "/tmp/$n" 2>/dev/null
    done

    mod_say "LOG_DIR    = $LOG_DIR"
    mod_say "RUN_DIR    = ${RUN_DIR:-/tmp/calypso}"
    mod_say "RECORD     = $RECORD_FILE"
    mod_say "TSLOG      = $TSLOG"
    mod_ok
}

# BARRIÈRE — critère observable. On ne se contente pas de `[ -w $LOG_DIR ]` :
# sur un volume plein, ou monté en lecture seule après coup, le test de
# permission réussit et l'écriture échoue — c'est exactement le mode de panne
# qui gelait le firmware quand /tmp (tmpfs 512 Mo) était saturé. On PROUVE
# l'écriture, on vérifie que le filtre d'horodatage existe, et que les deux
# symlinks pointent bien dans $LOG_DIR.
_logs_writable() {
    local probe="$LOG_DIR/.probe.$$"
    : > "$probe" 2>/dev/null || return 1
    rm -f "$probe" 2>/dev/null
    return 0
}
_logs_ready() {
    _logs_writable || return 1
    [ "${TSLOG:-cat}" = "cat" ] || [ -r "${TSLOG_SCRIPT:-}" ] || return 1
    [ "$(readlink /tmp/qemu-modem-tx.raw 2>/dev/null)" = "$LOG_DIR/qemu-modem-tx.raw" ] || return 1
    [ "$(readlink /tmp/qemu-irda-tx.raw  2>/dev/null)" = "$LOG_DIR/qemu-irda-tx.raw"  ] || return 1
    return 0
}

mod_logs_wait() {
    if ! wait_until "${MOD_TIMEOUT[logs]}" "espace de journaux" _logs_ready; then
        if ! _logs_writable; then
            mod_hint "df -h $LOG_DIR — volume plein ou monté en lecture seule ?"
            mod_fail "écriture impossible dans $LOG_DIR (test réel, pas un test de droits)"
        else
            mod_hint "vérifiez que /tmp est inscriptible : QEMU y écrit qemu-modem-tx.raw en dur"
            mod_fail "préparation incomplète : filtre d'horodatage ou symlinks /tmp/qemu-*-tx.raw absents"
        fi
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# Arrêt ciblé : on ne détruit AUCUN journal (c'est la matière du diagnostic
# post-mortem). On retire seulement les symlinks, qui n'ont de sens que pendant
# un run et qui, laissés là, feraient croire à un run en cours.
mod_logs_stop() {
    rm -f /tmp/qemu-modem-tx.raw /tmp/qemu-irda-tx.raw 2>/dev/null
    return 0
}
