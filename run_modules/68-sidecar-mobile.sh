# =============================================================================
#  68-sidecar-mobile — le second abonné (MS#2, profil hybride)
# =============================================================================
#
#  RÔLE    Pile L2/L3 osmocom-bb du MS#2. Il campe sur le BTS#1 pendant que le
#          MS#1 campe sur le BTS#0 (QEMU) — même MSC, donc appels et SMS entre
#          les deux sans sortir de la maquette :
#              depuis le MS#1 : « call 1 10002 »   ·   depuis le MS#2 : « call 1 10001 »
#
#  PRÉREQUIS  la socket L1CTL de trxcon. Comme pour le MS#1, elle apparaît AVANT
#             que le producteur ne soit prêt à la servir : d'où le délai.
#
#  SUCCÈS  la VTY $SC_MOBILE_VTY_PORT répond (4247 est celle du MS#1).
#  JOURNAL $LOG_DIR/sidecar-mobile.log
# -----------------------------------------------------------------------------
. "$(dirname "${BASH_SOURCE[0]}")/_lib/radio.sh"

MOD_REGISTER sidecar-mobile "Second abonné (MS#2)"
MOD_REQUIRED[sidecar-mobile]=0
MOD_DEPS[sidecar-mobile]="sidecar-trxcon"
MOD_PROFILES[sidecar-mobile]="hybrid"
MOD_TIMEOUT[sidecar-mobile]=30

: "${SC_MOBILE_BIN:=mobile}"
: "${SC_MOBILE_DELAY:=3}"
# Géométrie du tampon ALSA — MÊME défaut et MÊME raison que pour le MS#1 : voir
# le bloc « DESCENDANT MUET » en tête de 70-l2.sh. Ce mobile-ci écrit dans le
# MÊME sink `gsm_audio`, il est exposé au même défaut de négociation du greffon.
: "${CALYPSO_PULSE_LATENCY_MSEC:=80}"

mod_sidecar_mobile_check() {
    command -v "$SC_MOBILE_BIN" >/dev/null 2>&1 || {
        mod_fail "binaire introuvable : $SC_MOBILE_BIN"; return $MOD_RC_FAIL; }
    [ -r "$SC_MOBILE_CFG" ] || {
        mod_hint "le module « sidecar-cfg » doit avoir posé ce fichier"
        mod_fail "configuration illisible : $SC_MOBILE_CFG"; return $MOD_RC_FAIL; }
    mod_ok
}

mod_sidecar_mobile_status() { radio_alive sidecar-mobile; }

mod_sidecar_mobile_start() {
    radio_dirs
    local log; log="$(radio_log sidecar-mobile)"
    wait_until 30 "socket L1CTL $SC_L2_SOCK" have_unix "$SC_L2_SOCK" || {
        mod_fail "socket L1CTL absente : $SC_L2_SOCK"; return $MOD_RC_FAIL; }
    # Même raison que pour le MS#1 : la socket existe avant que trxcon ne soit
    # prêt à la servir ; sans ce délai le mobile repart sans chercher de cellule.
    if [ "${SC_MOBILE_DELAY}" != 0 ]; then
        mod_say "attente de ${SC_MOBILE_DELAY}s avant lancement"
        sleep "$SC_MOBILE_DELAY"
    fi
    mod_say "mobile -c $SC_MOBILE_CFG (VTY $SC_MOBILE_VTY_PORT)"
    mod_say "journal : $log"
    mod_say "latence : PULSE_LATENCY_MSEC=${CALYPSO_PULSE_LATENCY_MSEC:-<defaut greffon>}"
    # [2026-08-12] -d AJOUTE. Ce mobile-ci demarrait SANS masque de categories :
    # il tombait sur le defaut compile, et son journal n'avait donc ni la couche 2
    # (DLLAPD) ni le meme perimetre que celui du MS#1. Deux MS cote a cote avec
    # deux journaux non comparables rendent bancal tout diagnostic differentiel
    # (« le sidecar campe, pas le Calypso ») : une ligne absente d'un cote ne
    # voulait rien dire. On reprend le MEME masque, defini une seule fois dans
    # 70-l2.sh — tous les modules sont sources avant tout demarrage (run.sh:323),
    # donc la variable est posee quand cette fonction s'execute.
    mod_say "debug    : $CALYPSO_MOBILE_DEBUG"
    PULSE_LATENCY_MSEC="$CALYPSO_PULSE_LATENCY_MSEC" \
        stdbuf -oL -eL "$SC_MOBILE_BIN" -c "$SC_MOBILE_CFG" \
        -d "$CALYPSO_MOBILE_DEBUG" >>"$log" 2>&1 &
    radio_save_pid sidecar-mobile $!
    mod_ok
}

mod_sidecar_mobile_wait() {
    wait_until "${MOD_TIMEOUT[sidecar-mobile]}" "VTY du MS#2 ($SC_MOBILE_VTY_PORT)" \
        have_port "$SC_MOBILE_VTY_PORT" || {
        modb_tail "$(radio_log sidecar-mobile)" 20
        mod_hint "port déjà pris ? ss -tlnp | grep :$SC_MOBILE_VTY_PORT"
        mod_fail "VTY $SC_MOBILE_VTY_PORT jamais en écoute"; return $MOD_RC_FAIL; }
    mod_say "prêt — telnet 127.0.0.1 $SC_MOBILE_VTY_PORT puis « show ms »"
    mod_ok
}

mod_sidecar_mobile_stop() { radio_kill sidecar-mobile; return 0; }
