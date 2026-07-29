# =============================================================================
#  65-record-drain — draineur du FIFO I/Q vers le ring sur disque
# =============================================================================
#
#  RÔLE (run.sh.legacy L2089-2094)
#      QEMU écrit l'I/Q descendante dans /tmp/iq_record.fifo par un write() NON
#      BLOQUANT, trame par trame. record_drain.py absorbe ce flux et l'écrit en
#      anneau de 128 Mo dans $CALYPSO_RECORD_FILE, HORS du hot-path de QEMU.
#      C'est l'externalisation du ring que QEMU écrivait lui-même auparavant —
#      c'est ce fwrite dans le hot-path DL qui causait les underruns.
#      Le décodeur gr-gsm (module 66) relit ensuite ce ring hors ligne.
#
#  PRÉREQUIS
#      QEMU démarré (c'est lui le producteur du FIFO), et record_drain.py.
#
#  CRITÈRE DE SUCCÈS
#      Le processus vit ET le fichier de sortie existe. La CROISSANCE du ring
#      est mesurée et journalisée, mais ne fait échouer le module que si
#      CALYPSO_RECORD_DRAIN_STRICT=1 — voir le POURQUOI ci-dessous.
#
#  JOURNAL
#      $LOG_DIR/record_drain.log
#
#  POURQUOI LA CROISSANCE N'EST PAS BLOQUANTE PAR DÉFAUT
#      Le ring ne grossit que si QEMU pousse réellement de l'I/Q, ce qui suppose
#      un BTS qui émet et le tee actif. Au moment où ce module tourne, la chaîne
#      radio peut être légitimement encore muette : exiger la croissance ferait
#      échouer un draineur parfaitement sain. Le critère dur est donc « vivant +
#      fichier créé » ; la croissance est un diagnostic, rendu exigeant à la
#      demande. HYPOTHÈSE assumée : voir le rendu de tâche.
#
#  NOTE — le legacy faisait `pkill -9 -f record_drain.py` juste avant de lancer
#  (L2093). Ce n'est pas le rôle d'un module de démarrage : la mise à mort
#  générale appartient au module `teardown` (bloc A). Ici l'idempotence est
#  assurée par mod_record_drain_status, qui rend « déjà démarré ».
# -----------------------------------------------------------------------------
MOD_REGISTER record-drain "Draineur I/Q vers le ring disque"
MOD_REQUIRED[record-drain]=0
MOD_DEPS[record-drain]="qemu"
MOD_PROFILES[record-drain]="calypso hybrid"
MOD_TIMEOUT[record-drain]=20
# L2088 : le bloc full-grgsm. Le preset de composants s'appelle CALYPSO_PIPELINE
# (module 06) et non CALYPSO_MODE, qui désigne déjà le profil d'émulation dans
# environnement/modes.env. Défaut de repli `full-grgsm` = comportement legacy.
MOD_ENABLED_IF[record-drain]='[ "${CALYPSO_PIPELINE:-full-grgsm}" = full-grgsm ] || [ "${CALYPSO_FORCE_DEMOD_BRIDGE:-0}" = 1 ]'

: "${CALYPSO_RECORD_DRAIN:=${QEMU_TREE:-${QEMU_TREE}}/opt-gsm-scripts/record_drain.py}"
: "${CALYPSO_RECORD_FIFO:=/tmp/iq_record.fifo}"
: "${CALYPSO_RECORD_RING:=$((128 * 1024 * 1024))}"
: "${CALYPSO_RECORD_DRAIN_STRICT:=0}"
# CALYPSO_RECORD_FILE est déjà posé par environnement/calypso.env
# (/dev/shm/record.cfile) ; on ne fait que le compléter s'il manquait.
: "${CALYPSO_RECORD_FILE:=/dev/shm/record.cfile}"

_recdrain_pat() { printf '%s' "record_drain.py"; }

mod_record_drain_check() {
    [ -r "$CALYPSO_RECORD_DRAIN" ] || {
        mod_hint "posez CALYPSO_RECORD_DRAIN=/chemin/vers/record_drain.py"
        mod_skip "record_drain.py absent : $CALYPSO_RECORD_DRAIN"
        return $MOD_RC_SKIP; }
    command -v python3 >/dev/null 2>&1 || {
        mod_fail "python3 introuvable — record_drain.py ne peut pas tourner"
        return $MOD_RC_FAIL; }
    # Le répertoire du ring doit être inscriptible (souvent /dev/shm).
    local d; d="$(dirname "$CALYPSO_RECORD_FILE")"
    [ -w "$d" ] || {
        mod_hint "choisissez un autre chemin : CALYPSO_RECORD_FILE=/tmp/record.cfile"
        mod_fail "répertoire du ring non inscriptible : $d"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_record_drain_status() { have_proc "$(_recdrain_pat)"; }

mod_record_drain_start() {
    local log="${LOG_DIR}/record_drain.log"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    mod_say "fifo     : $CALYPSO_RECORD_FIFO"
    mod_say "ring     : $CALYPSO_RECORD_FILE ($((CALYPSO_RECORD_RING / 1024 / 1024)) Mo)"

    CALYPSO_RECORD_FIFO="$CALYPSO_RECORD_FIFO" \
    CALYPSO_RECORD_FILE="$CALYPSO_RECORD_FILE" \
    CALYPSO_RECORD_RING="$CALYPSO_RECORD_RING" \
        python3 -u "$CALYPSO_RECORD_DRAIN" >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR}/record-drain.pid"
    mod_ok
}

# BARRIÈRE — le legacy lançait en sous-shell détaché (L2094) sans rien vérifier.
#   1. le processus vit (un FIFO impossible à créer le tue à l'ouverture) ;
#   2. le fichier de ring existe (record_drain.py l'ouvre en écriture au
#      démarrage : sa présence prouve que le script a passé son initialisation) ;
#   3. mesure de croissance, informative (ou bloquante si STRICT=1).
mod_record_drain_wait() {
    local pid t; pid="$(cat "${RUN_DIR}/record-drain.pid" 2>/dev/null || echo 0)"
    t="${MOD_TIMEOUT[record-drain]}"

    wait_until "$t" "fichier de ring $CALYPSO_RECORD_FILE" test -e "$CALYPSO_RECORD_FILE" || {
        mod_hint "lisez ${LOG_DIR}/record_drain.log ; vérifiez les droits sur $CALYPSO_RECORD_FIFO"
        mod_fail "le draineur n'a pas créé $CALYPSO_RECORD_FILE"
        return $MOD_RC_FAIL; }

    if ! kill -0 "$pid" 2>/dev/null; then
        mod_hint "lisez ${LOG_DIR}/record_drain.log"
        mod_fail "le draineur a démarré puis s'est arrêté"
        return $MOD_RC_FAIL
    fi

    # Croissance du ring entre deux relevés espacés d'une seconde.
    local a b
    a="$(stat -c %s "$CALYPSO_RECORD_FILE" 2>/dev/null || echo 0)"
    sleep 1
    b="$(stat -c %s "$CALYPSO_RECORD_FILE" 2>/dev/null || echo 0)"
    if [ "$a" = "$b" ]; then
        if [ "$CALYPSO_RECORD_DRAIN_STRICT" = 1 ]; then
            mod_hint "aucune I/Q ne sort de QEMU : vérifiez le BTS et le tee I/Q avant de relancer"
            mod_fail "ring figé à $b octets (CALYPSO_RECORD_DRAIN_STRICT=1)"
            return $MOD_RC_FAIL
        fi
        mod_say "AVERTISSEMENT : ring figé à $b octets — QEMU ne pousse pas encore d'I/Q"
    else
        mod_say "ring alimenté : $a -> $b octets en 1 s"
    fi
    mod_ok
}

mod_record_drain_stop() {
    local pid; pid="$(cat "${RUN_DIR}/record-drain.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_recdrain_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/record-drain.pid"
    return 0
}
