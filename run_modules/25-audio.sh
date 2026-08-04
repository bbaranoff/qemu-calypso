# =============================================================================
#  25-audio — le pont audio : RTP du MGW -> sink PulseAudio « gsm_audio »
# =============================================================================
#
#  RÔLE
#    Lever le démon PulseAudio système, créer le sink `gsm_audio`, puis lancer
#    osmo-gapk qui transcode le RTP du MGW vers ce sink. Reprend, sans les
#    réécrire, ensure_pulse / ensure_gapk / ensure_host_audio de
#    start-direct.sh.legacy (L566-722), extraites dans osmo_egprs/lib/audio.sh.
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. C'EST UNE PRÉCONDITION DU CHEMIN `qemu`, PAS UN AGRÉMENT.
#     Le legacy appelait ensure_gapk juste avant de passer la main à
#     osmo-qemu-calypso/start-clean.sh (L1077). Si le découpage l'avait perdue, la pile
#     serait montée exactement pareil et l'appel n'aurait produit aucun son —
#     une panne qui ne se voit qu'à l'oreille, jamais dans un journal.
#
#  2. POURQUOI L'ABSENCE D'AUDIO N'ARRÊTE RIEN.
#     osmo-gapk et PulseAudio manquent souvent sur une machine de
#     développement, et le reste de la pile (LU, SMS, données) n'en dépend pas.
#     Le module est donc optionnel et se DÉSACTIVE proprement quand l'outillage
#     manque, au lieu d'échouer : c'est le comportement de l'original, qui
#     rendait la main avec un simple avertissement.
#
#  3. LA BARRIÈRE NE SE CONTENTE PAS DE « la session tmux existe ».
#     `tmux new-session -d` réussit même si la commande qu'elle lance meurt
#     aussitôt : la session disparaît une fraction de seconde plus tard, ou
#     survit vide. On exige donc que le PROCESSUS osmo-gapk soit là, et que le
#     sink `gsm_audio` soit réellement enregistré auprès du serveur.
#
#  PRÉREQUIS : tmux, osmo-gapk, pactl/pulseaudio (sinon : désactivé).
#  SUCCÈS    : cf. POURQUOI 3.
#  JOURNAL   : $LOG_DIR/mod/audio.log (et $LOG_DIR/gapk-auto.log pour gapk).
# -----------------------------------------------------------------------------

MOD_REGISTER audio "Pont audio (PulseAudio + GAPK)"
MOD_REQUIRED[audio]=0
MOD_PROFILES[audio]="calypso faketrx hybrid core"
MOD_TIMEOUT[audio]=25

# AUDIO=0 : échappatoire héritée, elle coupe toute la mise en place audio.
MOD_ENABLED_IF[audio]='[ "${AUDIO:-1}" = 1 ]'

: "${EGPRS_DIR:=${NITB_ROOT}}"
: "${AUDIO_LIB:=$EGPRS_DIR/lib/audio.sh}"
: "${AUDIO_SINK:=gsm_audio}"
: "${AUDIO_TMUX_SESSION:=gapk}"

_audio_sink_present() {
    pactl list short sinks 2>/dev/null | grep -q "$AUDIO_SINK"
}
# Correspondance EXACTE sur le nom, jamais `pgrep -f`. Un `-f osmo-gapk`
# reconnaît n'importe quelle ligne de commande CONTENANT ces lettres — y
# compris le shell qui vient de taper `pgrep -a osmo-gapk` pour vérifier. Le
# module se croyait alors « déjà démarré » et sautait le lancement : un faux
# positif qui ne se manifeste que par une absence de son.
_audio_gapk_vivant() {
    # `pgrep -x` sur osmo-gapk ne conviendrait pas : le transcodeur n'existe
    # que pendant un appel. On observe le superviseur, qui lui reste en vie.
    pgrep -f "gapk-start.sh" >/dev/null 2>&1 \
        || tmux has-session -t "${AUDIO_TMUX:-gapk}" 2>/dev/null
}

mod_audio_check() {
    if [ ! -r "$AUDIO_LIB" ]; then
        mod_hint "la bibliothèque est l'extraction de start-direct.sh.legacy L566-722 ; elle accompagne osmo_egprs"
        mod_skip "bibliothèque audio absente ($AUDIO_LIB) : pont audio non monté"
        return $MOD_RC_SKIP
    fi
    local manquant=""
    command -v tmux      >/dev/null 2>&1 || manquant="$manquant tmux"
    command -v osmo-gapk >/dev/null 2>&1 || manquant="$manquant osmo-gapk"
    if [ -n "$manquant" ]; then
        # cf. POURQUOI 2 : on se désactive, on n'échoue pas.
        mod_hint "installez-les pour avoir le son :$manquant — ou AUDIO=0 pour ne plus l'essayer"
        mod_skip "outillage audio absent :$manquant"
        return $MOD_RC_SKIP
    fi
    mod_ok
}

mod_audio_status() { _audio_gapk_vivant && _audio_sink_present; }

mod_audio_start() {
    local retour="$PWD" rc=0
    # Les fonctions extraites cherchent scripts/gapk-start.sh en repli sur $HERE.
    cd "$EGPRS_DIR" || { mod_fail "impossible d'entrer dans $EGPRS_DIR"; return $MOD_RC_FAIL; }
    HERE="$EGPRS_DIR"
    # shellcheck source=/dev/null
    . "$AUDIO_LIB" || { cd "$retour"; mod_fail "bibliothèque illisible : $AUDIO_LIB"; return $MOD_RC_FAIL; }

    mod_say "sink visé : $AUDIO_SINK · session tmux : $AUDIO_TMUX_SESSION"
    ensure_gapk || rc=$?     # ensure_gapk appelle lui-même ensure_pulse
    cd "$retour"
    [ "$rc" -eq 0 ] || { mod_fail "ensure_gapk a rendu $rc"; return $MOD_RC_FAIL; }
    mod_ok
}

# BARRIÈRE — cf. POURQUOI 3.
mod_audio_wait() {
    wait_until "${MOD_TIMEOUT[audio]}" "démon PulseAudio" pactl info || {
        mod_hint "regardez $LOG_DIR/pulse-system.log : le démon système n'a pas démarré"
        mod_fail "PulseAudio ne répond pas"
        return $MOD_RC_FAIL
    }
    wait_until "${MOD_TIMEOUT[audio]}" "sink $AUDIO_SINK" _audio_sink_present || {
        mod_hint "pactl load-module module-null-sink sink_name=$AUDIO_SINK"
        mod_fail "le sink $AUDIO_SINK n'est pas enregistré"
        return $MOD_RC_FAIL
    }
    # Le SUPERVISEUR gapk, pas le transcodeur : `gapk-start.sh auto` ne lance
    # osmo-gapk qu'à l'établissement d'un appel. Exiger le binaire hors appel
    # était un faux négatif permanent — le pont était en place et déclaré mort.
    wait_until "${MOD_TIMEOUT[audio]}" "superviseur gapk" _audio_gapk_vivant || {
        mod_hint "regardez $LOG_DIR/gapk-auto.log : le superviseur s'est arrêté aussitôt lancé"
        mod_fail "le superviseur gapk ne tourne pas"
        return $MOD_RC_FAIL
    }
    mod_ok
}

# On arrête le pont, pas le serveur audio : PulseAudio est lancé en mode système
# avec --disallow-exit ; le tuer sortirait du périmètre de la pile GSM et
# couperait le son de tout ce qui tourne d'autre sur la machine.
mod_audio_stop() {
    tmux kill-session -t "$AUDIO_TMUX_SESSION" 2>/dev/null
    pkill -x osmo-gapk 2>/dev/null
    return 0
}
