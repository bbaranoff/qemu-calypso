# =============================================================================
#  70-l2 — client de couche 2 (mobile / ccch_scan / bcch_scan / cell_log)
# =============================================================================
#
#  RÔLE (run.sh.legacy L2040-2153, plus la sélection interactive L1475-1499)
#      Le client L2 se branche sur la socket L1CTL /tmp/osmocom_l2 ouverte par
#      osmocon après son handshake romload avec le firmware. `mobile` est le
#      client complet (sélection PLMN, MM/RR, LU, SMS) ; les trois autres sont
#      des sondes passives.
#
#  PRÉREQUIS
#      osmocon démarré et sa socket L1CTL créée.
#
#  CRITÈRE DE SUCCÈS
#      Le client vit ET a produit sa première trace. Un client qui meurt sur
#      « Cannot start MS … same layer2-socket » ou sur une cfg illisible sort en
#      moins d'une seconde : le legacy ne l'aurait pas vu (lancement via
#      send-keys dans une fenêtre tmux, aucun retour).
#
#  JOURNAL
#      $LOG_DIR/mobile.log (client mobile) ou $LOG_DIR/l2_client.log
#
#  DEUX CHOSES QUE CE MODULE SUPPRIME DÉLIBÉRÉMENT
#    1. LE `read -r` INTERACTIF (L1490). Le legacy demandait le client au clavier
#       quand CALYPSO_L2_CLIENT était vide : un lancement non interactif restait
#       bloqué indéfiniment. Ici le défaut est `mobile`, sans question.
#    2. LA BOUCLE INLINE `L1CTL_WAIT` (L2053), recopiée dans chaque commande
#       tmux. L'attente de la socket est remontée dans _check : une seule
#       implémentation, un message d'échec, et une dépendance déclarée.
# -----------------------------------------------------------------------------
MOD_REGISTER l2 "Client de couche 2"
MOD_REQUIRED[l2]=0
MOD_DEPS[l2]="osmocon"
# Profil `faketrx` RETIRE : dans cette chaine, c'est 66-mobile qui tient le role
# de client L2 (sur la socket de trxcon/virtphy), et l2 depend d'osmocon, absent
# du profil. Deux clients declares pour un seul role, c'est une ligne du plan qui
# ne peut que finir en SKIP — autant ne pas la promettre.
MOD_PROFILES[l2]="calypso hybrid"
MOD_TIMEOUT[l2]=30
MOD_ENABLED_IF[l2]='[ "${CALYPSO_SKIP_L2:-0}" != 1 ]'

# Défaut NON INTERACTIF — remplace la question de L1475-1499.
: "${CALYPSO_L2_CLIENT:=mobile}"
: "${CALYPSO_L1CTL_SOCK:=/tmp/osmocom_l2}"
# Masque de catégories du mobile (L2075) ; séparateur `:`, pas `,`.
# [2026-08-12] +DLLAPD. La liste ci-dessous etait, au caractere pres, le defaut
# compile du binaire (`mobile -h`) : elle n'ajoutait donc RIEN. DLLAPD est la
# categorie de `lapdm.c` / `lapd_core.c` (tag <001f>), la seule qui manquait pour
# voir la couche 2 : « Received frame for unsupported SAPI N », les MDL-ERROR-IND
# avec leur cause ET leur etat LAPD, les « N(S) sequence error », les
# « I frame ignored in state LAPD_STATE_IDLE ». Sans elle, une rupture de lien ne
# se lit que par ses consequences en couche 3, une fois qu'il est trop tard.
# Ce masque sert AUX DEUX MS : 68-sidecar-mobile.sh le reprend tel quel.
: "${CALYPSO_MOBILE_DEBUG:=DCS:DNB:DPLMN:DRR:DMM:DSIM:DCC:DMNCC:DSS:DLSMS:DPAG:DSUM:DSAP:DGPS:DMOB:DPRIM:DLUA:DGAPK:DLLAPD}"

# ── DESCENDANT MUET : géométrie du tampon ALSA du mobile ─────────────────────
# [2026-08-10] RACINE MESURÉE, pas déduite. `osmo-gapk/src/pq_alsa.c` (l.87-135)
# ouvre le périphérique en ne fixant QUE access/format/rate/channels : il ne
# contraint jamais buffer_size ni period_size et accepte ce que le greffon
# propose. Le greffon ALSA-PulseAudio accorde alors :
#     buffer_size = 2097152 trames (262144 ms !)   period_size = 2048 (256 ms)
# Face à des écritures de 160 trames (20 ms, une trame GSM), CHAQUE
# `snd_pcm_writei` rend -EPIPE. Or `pq_alsa.c` l.61-66 traite l'EPIPE par un
# `snd_pcm_prepare()` + réécriture, SANS AUCUN LOGP — et dans le greffon pulse
# ce `prepare` démonte et remonte le flux. D'où le tableau observé : sink-input
# PulseAudio détruit/recréé 50 fois par seconde, `Buffer Latency = 0`, RMS et
# crête EXACTEMENT nuls sur gsm_audio.monitor, et pas une ligne d'erreur nulle
# part. Tout l'amont est sain (50,1 TRAFFIC.ind/s, charges FR valides).
#
# Reproduit hors de toute la pile GSM par un programme de 60 lignes qui rejoue
# la même séquence d'ouverture (`/tmp/repro_alsa.c`) :
#     sans la variable : 749 EPIPE / 750 écritures
#     PULSE_LATENCY_MSEC=40 : 0/750    =60 : 1/750    =80 : 0/750
#
# 80 ms = 4 trames GSM : zéro underrun avec de la marge sur le jitter mesuré du
# descendant (médiane 18,58 ms, max 23,72 ms), et négligeable devant les ~240 ms
# qu'affiche naturellement un flux sain.
#
# CE N'EST PAS UNE BÉQUILLE : on ne masque pas un symptôme, on donne au greffon
# la géométrie que l'application aurait dû négocier. Le correctif propre serait
# un `snd_pcm_set_params()` dans pq_alsa.c — hors périmètre (infra osmo).
#
# JUGE (pendant un appel ÉTABLI) : l'index du sink-input du mobile doit rester
# FIXE. Il bouge de +50/s ⇒ le défaut est de retour.
#     pactl list short sink-inputs | awk '$2==1{print $1}'   # 2 relevés à 1 s
# Poser la variable à vide DÉSACTIVE le correctif (retour au défaut du greffon).
: "${CALYPSO_PULSE_LATENCY_MSEC:=80}"
# Marqueur « le firmware tourne » émis par osmocon à la fin du romload. Chaîne
# relevée dans le binaire osmocon : « Received DOWNLOAD ACK from phone, your
# code is running now! ». Remplace le `sleep 3` de L2131.
: "${CALYPSO_L1_READY_MARKER:=your code is running now}"
: "${CALYPSO_L1_READY_TIMEOUT:=20}"
# 1 = l'absence du marqueur fait échouer le module. Défaut 0 : voir POURQUOI.
: "${CALYPSO_L2_REQUIRE_L1_MARKER:=0}"

_l2_log() {
    case "$CALYPSO_L2_CLIENT" in
        mobile) printf '%s' "${LOG_DIR}/mobile.log" ;;
        *)      printf '%s' "${LOG_DIR}/l2_client.log" ;;
    esac
}
# Motif de reconnaissance du processus. Volontairement plus précis que le seul
# nom : un pane `tail -F …/mobile.log` contient « mobile » et donnerait un faux
# « déjà démarré ».
# Le motif doit désigner UN client, pas la famille. En profil hybride deux
# `mobile` tournent (MS#1 sur QEMU, MS#2 sur le side-car) et seul le chemin de
# configuration les sépare : sans lui, ce module adoptait le mobile du voisin.
_l2_pat() {
    case "$CALYPSO_L2_CLIENT" in
        mobile)               printf '%s' "mobile -c $(_l2_cfg)" ;;
        ccch_scan|bcch_scan)  printf '%s' "$CALYPSO_L2_CLIENT -a " ;;
        *)                    printf '%s' "^$CALYPSO_L2_CLIENT$" ;;
    esac
}
_l2_cfg() {
    if [ -n "${MOBILE_CFG:-}" ]; then printf '%s' "$MOBILE_CFG"
    elif [ -r "${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg" ]; then
        printf '%s' "${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg"
    else printf '%s' "${QEMU_CFGS:-${QEMU_TREE}/cfgs}/mobile_group1.cfg"; fi
}

mod_l2_check() {
    case "$CALYPSO_L2_CLIENT" in
        mobile|ccch_scan|bcch_scan|cell_log) ;;
        *) mod_hint "valeurs acceptées : mobile | ccch_scan | bcch_scan | cell_log"
           mod_fail "CALYPSO_L2_CLIENT inconnu : $CALYPSO_L2_CLIENT"
           return $MOD_RC_FAIL ;;
    esac
    command -v "$CALYPSO_L2_CLIENT" >/dev/null 2>&1 || {
        mod_hint "compilez osmocom-bb (layer23) ou ajoutez son répertoire au PATH"
        mod_fail "client L2 introuvable dans le PATH : $CALYPSO_L2_CLIENT"
        return $MOD_RC_FAIL; }
    if [ "$CALYPSO_L2_CLIENT" = mobile ]; then
        local cfg; cfg="$(_l2_cfg)"
        [ -r "$cfg" ] || {
            mod_hint "le module mobile-cfg (20) doit avoir copié cfgs/mobile_group1.cfg vers \$OSMOCOM_HOME/bb/"
            mod_fail "configuration du mobile illisible : $cfg"
            return $MOD_RC_FAIL; }
    fi

    # En simulation, on s'arrête ici : --dry-run doit vérifier que le plan tient,
    # pas attendre une socket que personne n'a démarrée. (DRY est la variable du
    # moteur ; les modules étant sourcés dans son shell, on la lit — on n'y
    # touche pas. Absente, elle vaut 0 et le comportement normal s'applique.)
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : attente de la socket L1CTL et du marqueur L1 non jouées"
        mod_ok; return $MOD_RC_OK
    fi

    # --- remplacement de L1CTL_WAIT (L2053 : 60 × sleep 0.5, inline tmux) -----
    wait_until "${MOD_TIMEOUT[l2]}" "socket L1CTL $CALYPSO_L1CTL_SOCK" \
               have_unix "$CALYPSO_L1CTL_SOCK" || {
        mod_hint "c'est osmocon qui crée cette socket, avec l'option -s : vérifiez que 50-osmocon.sh lance bien « osmocon -m romload -p <PTY> -s $CALYPSO_L1CTL_SOCK <FIRMWARE_BIN> » (legacy L1930) — sans -s, aucune socket n'est créée"
        mod_fail "socket L1CTL absente : $CALYPSO_L1CTL_SOCK"
        return $MOD_RC_FAIL; }

    # --- remplacement du `sleep ${CALYPSO_MOBILE_DELAY:-3}` (L2131) -----------
    # Le legacy dormait 3 s « pour laisser la L1 se stabiliser » : la socket peut
    # exister avant que le firmware ne tourne. La condition réelle est que le
    # romload d'osmocon ait abouti — ce que dit son propre journal.
    if ! wait_until "$CALYPSO_L1_READY_TIMEOUT" "L1 prête" \
                    log_has "${LOG_DIR}/osmocon.log" "$CALYPSO_L1_READY_MARKER"; then
        if [ "$CALYPSO_L2_REQUIRE_L1_MARKER" = 1 ]; then
            mod_hint "lisez ${LOG_DIR}/osmocon.log : le handshake romload n'a pas abouti"
            mod_fail "aucune confirmation de démarrage du firmware (« $CALYPSO_L1_READY_MARKER »)"
            return $MOD_RC_FAIL
        fi
        mod_say "AVERTISSEMENT : marqueur « $CALYPSO_L1_READY_MARKER » absent d'osmocon.log — on lance quand même"
    fi
    mod_ok
}

# Le PID enregistré d'abord : c'est le seul témoin qui ne peut pas confondre
# deux processus du même nom. Le motif ne sert que de secours, quand le fichier
# de PID a été perdu (arrêt brutal, run précédent).
mod_l2_status() {
    local pid; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    if [ "$pid" != 0 ] && kill -0 "$pid" 2>/dev/null; then
        tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qF "$(_l2_cfg)" && return 0
    fi
    have_proc "$(_l2_pat)"
}

mod_l2_start() {
    local log cfg arfcn
    log="$(_l2_log)"; cfg="$(_l2_cfg)"; arfcn="${CALYPSO_CCCH_ARFCN:-514}"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    : > "$log"
    mod_say "client   : $CALYPSO_L2_CLIENT"
    mod_say "journal  : $log"

    case "$CALYPSO_L2_CLIENT" in
        mobile)
            mod_say "cfg      : $cfg"
            mod_say "debug    : $CALYPSO_MOBILE_DEBUG"
            # PULSE_LATENCY_MSEC : cf. le bloc « DESCENDANT MUET » en tête de
            # module. Sans elle, le descendant est muet une fois sur deux.
            mod_say "latence  : PULSE_LATENCY_MSEC=${CALYPSO_PULSE_LATENCY_MSEC:-<defaut greffon>}"
            PULSE_LATENCY_MSEC="$CALYPSO_PULSE_LATENCY_MSEC" \
                mobile -c "$cfg" -d "$CALYPSO_MOBILE_DEBUG" >>"$log" 2>&1 & ;;
        ccch_scan)  ccch_scan -a "$arfcn" >>"$log" 2>&1 & ;;
        bcch_scan)  bcch_scan -a "$arfcn" >>"$log" 2>&1 & ;;
        cell_log)   cell_log            >>"$log" 2>&1 & ;;
    esac
    printf '%s\n' "$!" > "${RUN_DIR}/l2.pid"
    mod_ok
}

# BARRIÈRE — côté run.sh, le legacy n'en avait AUCUNE (tout partait en
# send-keys). Deux critères observables :
#   1. le client a écrit sa première trace (il a donc dépassé la lecture de sa
#      configuration et l'ouverture de la socket) ;
#   2. il est toujours vivant à ce moment-là.
# Un troisième indice, la connexion effective sur la socket L1CTL, est relevé
# quand `ss` est disponible — informatif seulement, voir POURQUOI.
mod_l2_wait() {
    local pid log; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    log="$(_l2_log)"

    wait_until "${MOD_TIMEOUT[l2]}" "première trace du client L2" test -s "$log" || {
        mod_hint "lisez $log ; « same layer2-socket » = un autre client tient déjà $CALYPSO_L1CTL_SOCK"
        mod_fail "$CALYPSO_L2_CLIENT n'a produit aucune trace"
        return $MOD_RC_FAIL; }

    kill -0 "$pid" 2>/dev/null || {
        mod_hint "lisez la fin de $log : la cause est presque toujours dans les 10 dernières lignes"
        mod_fail "$CALYPSO_L2_CLIENT a démarré puis s'est arrêté"
        return $MOD_RC_FAIL; }

    # POURQUOI CE TROISIÈME CRITÈRE N'EST QU'INFORMATIF : `ss -x` n'expose pas le
    # chemin du côté client d'une socket UNIX connectée (« Peer Address » = *).
    # L'appariement n'est donc pas garanti sur toutes les versions d'iproute2 :
    # on le journalise, on n'en fait pas une condition d'échec.
    if command -v ss >/dev/null 2>&1; then
        if ss -x 2>/dev/null | grep -q "$CALYPSO_L1CTL_SOCK"; then
            mod_say "socket L1CTL $CALYPSO_L1CTL_SOCK : connexion observée"
        else
            mod_say "socket L1CTL $CALYPSO_L1CTL_SOCK : pas de connexion visible via ss (indice, pas verdict)"
        fi
    fi
    mod_ok
}

mod_l2_stop() {
    local pid; pid="$(cat "${RUN_DIR}/l2.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_l2_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/l2.pid"
    return 0
}
