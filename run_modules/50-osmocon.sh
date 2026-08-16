# =============================================================================
#  50-osmocon — la passerelle série entre le firmware émulé et l'hôte
# =============================================================================
#
#  RÔLE      osmocon charge le firmware dans le baseband émulé puis sert de
#            transport L1CTL entre lui et la couche 2 côté hôte. Sans lui, QEMU
#            exécute une ROM qui n'a rien à dire : le mobile reste « no SIM ».
#
#  PRÉREQUIS Le PTY série de QEMU, découvert par le module `pty` (OSMOCON_PTY).
#            Ce PTY CHANGE À CHAQUE LANCEMENT.
#
#  SUCCÈS    La socket L1CTL existe. C'est osmocon qui la crée, et seulement
#            APRÈS avoir mené sa poignée de main romload jusqu'au bout : sa
#            présence prouve le chargement, là où un grep sur le journal ne
#            prouve rien.
#
#  JOURNAL   $LOG_DIR/osmocon.log
#
#  ────────────────────────────────────────────────────────────────────────────
#  LES QUATRE ARGUMENTS QU'ON NE PEUT PAS OMETTRE (legacy run.sh L1929-1930).
#  Une version antérieure de ce module lançait « osmocon -p PTY -m c123xor FW »
#  et échouait sans rien afficher. Chacun de ces manques suffit à tout bloquer :
#
#   -m romload   LE modèle. `c123xor` est le chargeur d'un VRAI Compal : il
#                attend le prompt du chargeur d'amorçage du téléphone, que la
#                ROM émulée n'émet jamais — osmocon attend alors indéfiniment,
#                vivant et muet. QEMU veut `romload`.
#   -s SOCKET    crée la socket L1CTL. Sans elle, osmocon charge peut-être le
#                firmware, mais 70-l2 et le mobile n'ont nulle part où se
#                brancher (cf. l'indice de 70-l2.sh L109).
#   -i 100       délai inter-octets. L'UART émulé perd des octets sans lui.
#   stdbuf -oL   sortie redirigée vers un FICHIER = bufferisée par blocs. Sans
#                cela le journal reste à 0 octet pendant des minutes, ce qui
#                rend tout diagnostic impossible (constaté).
#  ────────────────────────────────────────────────────────────────────────────
# -----------------------------------------------------------------------------
MOD_REGISTER osmocon "Passerelle osmocon"
MOD_REQUIRED[osmocon]=1
MOD_DEPS[osmocon]="qemu pty"
MOD_PROFILES[osmocon]="calypso hybrid"
MOD_TIMEOUT[osmocon]=30          # legacy : 30 tentatives × 1 s (L1933-1936)

# `romload` et non `c123xor` : voir l'encadré ci-dessus.
: "${OSMOCON_MODEL:=romload}"
: "${OSMOCON_INTER_BYTE_DELAY:=100}"
: "${OSMOCON_DEBUG:=tr}"
# Même chemin que 70-l2 et le mobile. Volontairement NON exportée : QEMU en
# hériterait et croirait devoir l'ouvrir lui-même (cf. 05-config.sh, POURQUOI 4).
: "${L1CTL_SOCK_PATH:=/tmp/osmocom_l2}"

_osmocon_log() { printf '%s\n' "${LOG_DIR:-/root/calypso/logs}/osmocon.log"; }

# `pgrep -f osmocon` attraperait aussi les « tail -F .../osmocon.log » de la
# session tmux — la barrière voyait alors un tail et déclarait le service prêt.
# On exige donc le binaire lui-même, avec son option -p.
_osmocon_vivant() { pgrep -f "osmocon .*-p " >/dev/null 2>&1; }

mod_osmocon_check() {
    [ -x "${OSMOCON:-}" ] || {
        mod_hint "réglez OSMOCOM_BB dans environnement/paths.env, ou compilez osmocon"
        mod_fail "osmocon introuvable : ${OSMOCON:-<non défini>}"; return $MOD_RC_FAIL; }
    [ -r "${FIRMWARE_BIN:-}" ] || {
        mod_hint "le firmware est livré avec le dépôt : firmware/compal_e88/layer1.highram.bin"
        mod_fail "image firmware illisible : ${FIRMWARE_BIN:-<non défini>}"; return $MOD_RC_FAIL; }
    [ -n "${OSMOCON_PTY:-}" ] && [ -c "$OSMOCON_PTY" ] || {
        mod_hint "le module « pty » doit avoir résolu le port série de QEMU ; regardez son journal"
        mod_fail "PTY série indisponible : ${OSMOCON_PTY:-<non résolu>}"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_osmocon_status() { _osmocon_vivant && have_unix "$L1CTL_SOCK_PATH"; }

mod_osmocon_start() {
    [ "${CALYPSO_BRIDGE:-}" = ipc ] && { mod_skip "CALYPSO_BRIDGE=ipc : MS#1 via osmo-trx-ms-ipc, osmocon non lance"; return $MOD_RC_SKIP; }
    local log; log="$(_osmocon_log)"
    # Socket d'un run précédent : osmocon refuse de créer par-dessus, et sa
    # présence ferait passer la barrière à tort.
    rm -f "$L1CTL_SOCK_PATH" 2>/dev/null
    mod_say "PTY      : $OSMOCON_PTY"
    mod_say "firmware : $FIRMWARE_BIN"
    mod_say "modèle   : $OSMOCON_MODEL (romload = ROM émulée ; c123xor = vrai téléphone)"
    mod_say "L1CTL    : $L1CTL_SOCK_PATH"
    stdbuf -oL -eL "$OSMOCON" \
        -m "$OSMOCON_MODEL" \
        -i "$OSMOCON_INTER_BYTE_DELAY" \
        -p "$OSMOCON_PTY" \
        -s "$L1CTL_SOCK_PATH" \
        "$FIRMWARE_BIN" \
        -d "$OSMOCON_DEBUG" >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/osmocon.pid"
    mod_ok
}

# BARRIÈRE — deux temps, parce qu'ils échouent pour des raisons différentes :
#   1. le processus survit-il ? (un PTY refusé le tue en moins d'une seconde) ;
#   2. la socket L1CTL apparaît-elle ? osmocon ne la crée qu'une fois le
#      firmware transféré. C'est le même critère que le legacy (L1933) et c'est
#      un OBSERVABLE, pas un motif de journal susceptible de changer de forme
#      d'une version d'osmocon à l'autre.
mod_osmocon_wait() {
    local log; log="$(_osmocon_log)"

    wait_until 5 "processus osmocon" _osmocon_vivant || {
        modb_tail "$log" 15
        mod_hint "PTY $OSMOCON_PTY refusé ? regardez $log"
        mod_fail "osmocon s'est arrêté aussitôt lancé"
        return $MOD_RC_FAIL
    }

    if ! wait_until "${MOD_TIMEOUT[osmocon]}" "socket L1CTL $L1CTL_SOCK_PATH" \
         have_unix "$L1CTL_SOCK_PATH"; then
        modb_tail "$log" 15
        if _osmocon_vivant; then
            mod_hint "osmocon vit mais n'a pas fini son romload : modèle inadapté (OSMOCON_MODEL=$OSMOCON_MODEL — « romload » attendu sous QEMU) ou PTY qui ne mène pas au baseband"
            mod_fail "aucune socket L1CTL : le firmware n'a pas été chargé"
        else
            mod_hint "regardez $log"
            mod_fail "osmocon est mort pendant le chargement"
        fi
        return $MOD_RC_FAIL
    fi
    mod_say "firmware chargé, socket L1CTL ouverte"
    mod_ok
}

mod_osmocon_stop() {
    local pid; pid="$(cat "${RUN_DIR:-/tmp/calypso}/osmocon.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "osmocon .*-p " 2>/dev/null
    rm -f "${RUN_DIR:-/tmp/calypso}/osmocon.pid" "$L1CTL_SOCK_PATH"
    return 0
}
