# =============================================================================
#  66-grgsm-decode — le décodeur gr-gsm (gr-gsm tient le rôle du DSP)
# =============================================================================
#
#  RÔLE (run.sh.legacy L2095-2120)
#      Deux décodeurs possibles, choisis par CALYPSO_GRGSM_DECODER :
#        si-bridge (DÉFAUT) : si_bridge_loop.sh -> si_bridge.py lit le FIFO LIVE
#            /tmp/iq_grgsm.fifo (flux continu, 4 SPS), décode le vrai SI2/3/4/13
#            de la BTS, l'émet sur GSMTAP 4730 -> feed_si -> a_cd -> le mobile.
#            C'est CE chemin qui casse le mur de démodulation : le flux continu,
#            et non les bursts discontinus du BSP.
#        relay : grgsm_relay_decode.py lit l'I/Q relayée par l'IPC device (UDP
#            5810) -> gsm.receiver -> SI (4730) + BSIC/FN réels (4731).
#            Le legacy avertit (L2103-2106) : 5810 n'est pas alimenté dans cette
#            configuration, ce décodeur y produit un SI mort. D'où le défaut.
#
#  PRÉREQUIS
#      Le draineur (module 65) tourne, et sa source d'entrée est alimentée.
#
#  CRITÈRE DE SUCCÈS
#      L'entrée du décodeur existe (FIFO pour si-bridge, ring pour relay) avant
#      le lancement, puis le processus vit et a produit une ligne.
#
#  JOURNAL
#      $LOG_DIR/grgsm_decode.log
#
#  POURQUOI L'ATTENTE D'ENTRÉE EST DANS _check ET NON DANS _wait
#      Dans le legacy, le `sleep 15` PRÉCÉDAIT le lancement (L2109/L2112) : il
#      servait à laisser la pile se remplir AVANT que le décodeur ouvre sa
#      source. La condition équivalente doit donc être évaluée avant `start`,
#      c'est-à-dire dans `check` — qui reste en lecture seule (on ne fait que
#      sonder `test -p` / `stat`).
# -----------------------------------------------------------------------------
MOD_REGISTER grgsm-decode "Décodeur gr-gsm (SI + SCH)"
MOD_REQUIRED[grgsm-decode]=0
MOD_DEPS[grgsm-decode]="record-drain"
MOD_PROFILES[grgsm-decode]="calypso hybrid"
MOD_TIMEOUT[grgsm-decode]=30
MOD_ENABLED_IF[grgsm-decode]='[ "${CALYPSO_PIPELINE:-full-grgsm}" = full-grgsm ] || [ "${CALYPSO_FORCE_DEMOD_BRIDGE:-0}" = 1 ]'

: "${CALYPSO_GRGSM_DECODER:=si-bridge}"
: "${CALYPSO_SI_BRIDGE_LOOP:=${QEMU_TREE:-${QEMU_TREE}}/opt-gsm-scripts/si_bridge_loop.sh}"
: "${CALYPSO_RELAY_DECODE:=${QEMU_TREE:-${QEMU_TREE}}/opt-gsm-scripts/grgsm_relay_decode.py}"
: "${CALYPSO_GRGSM_FIFO:=/tmp/iq_grgsm.fifo}"
: "${CALYPSO_RECORD_FILE:=/dev/shm/record.cfile}"
# Seuil « la pile est assez remplie pour décoder », en octets. Remplace la durée
# arbitraire de 15 s du legacy. HYPOTHÈSE : 256 Ko de fc32 ≈ 32 k échantillons,
# soit largement plus d'un multiframe à 4 SPS. Ajustable.
: "${CALYPSO_GRGSM_MIN_RING:=262144}"
# Délai maximal d'attente de la source, en secondes (le legacy dormait 15 s en
# aveugle ; ici on sort dès que la condition est vraie, et on renonce au bout de
# ce délai en le disant).
: "${CALYPSO_GRGSM_INPUT_TIMEOUT:=45}"

_grgsm_script() {
    if [ "$CALYPSO_GRGSM_DECODER" = si-bridge ]; then printf '%s' "$CALYPSO_SI_BRIDGE_LOOP"
    else printf '%s' "$CALYPSO_RELAY_DECODE"; fi
}
_grgsm_pat() { printf '%s' "si_bridge|grgsm_relay_decode"; }
_grgsm_ring_ok() {
    local s; s="$(stat -c %s "$CALYPSO_RECORD_FILE" 2>/dev/null || echo 0)"
    [ "$s" -ge "$CALYPSO_GRGSM_MIN_RING" ]
}

mod_grgsm_decode_check() {
    local sc; sc="$(_grgsm_script)"
    case "$CALYPSO_GRGSM_DECODER" in
        si-bridge|relay) ;;
        *) mod_hint "valeurs acceptées : si-bridge | relay"
           mod_fail "CALYPSO_GRGSM_DECODER inconnu : $CALYPSO_GRGSM_DECODER"
           return $MOD_RC_FAIL ;;
    esac
    [ -r "$sc" ] || { mod_skip "décodeur absent : $sc"; return $MOD_RC_SKIP; }

    # En simulation, on ne joue pas l'attente de la source : --dry-run vérifie le
    # plan, pas la présence d'un flux I/Q. (DRY appartient au moteur ; on la lit
    # seulement — absente, elle vaut 0.)
    if [ "${DRY:-0}" = 1 ]; then
        mod_say "simulation : attente de la source d'entrée non jouée"
        mod_ok; return $MOD_RC_OK
    fi

    # --- remplacement du `sleep 15` (L2109 et L2112) --------------------------
    # si-bridge lit un FIFO : la condition réelle est que le FIFO existe, donc
    # que QEMU a créé sa sortie relais. On attend CETTE condition, pas 15 s.
    if [ "$CALYPSO_GRGSM_DECODER" = si-bridge ]; then
        wait_until "$CALYPSO_GRGSM_INPUT_TIMEOUT" "FIFO d'entrée $CALYPSO_GRGSM_FIFO" \
                   test -p "$CALYPSO_GRGSM_FIFO" || {
            mod_hint "le FIFO est créé par le relais I/Q de QEMU : vérifiez CALYPSO_RELAY_FIFOS et que QEMU tourne"
            mod_fail "aucune source à décoder : $CALYPSO_GRGSM_FIFO n'existe pas"
            return $MOD_RC_FAIL; }
    else
        # relay : la source utile est le ring alimenté par le module 65.
        wait_until "$CALYPSO_GRGSM_INPUT_TIMEOUT" "ring I/Q >= $CALYPSO_GRGSM_MIN_RING o" \
                   _grgsm_ring_ok || {
            mod_hint "CALYPSO_GRGSM_DECODER=si-bridge est le chemin prouvé ; « relay » exige que l'UDP 5810 soit réellement alimenté"
            mod_fail "ring $CALYPSO_RECORD_FILE toujours sous $CALYPSO_GRGSM_MIN_RING octets"
            return $MOD_RC_FAIL; }
    fi
    mod_ok
}

mod_grgsm_decode_status() { have_proc "$(_grgsm_pat)"; }

mod_grgsm_decode_start() {
    local log="${LOG_DIR}/grgsm_decode.log" sc; sc="$(_grgsm_script)"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    : > "$log"
    mod_say "décodeur : $CALYPSO_GRGSM_DECODER ($sc)"
    mod_say "sortie   : SI -> GSMTAP ${CALYPSO_SHUNT_GSMTAP_PORT:-4730} (feed_si) + SCH/BSIC -> 4731 (feed_sb)"

    if [ "$CALYPSO_GRGSM_DECODER" = si-bridge ]; then
        bash "$sc" >>"$log" 2>&1 &
    else
        python3 -u "$sc" >>"$log" 2>&1 &
    fi
    printf '%s\n' "$!" > "${RUN_DIR}/grgsm-decode.pid"
    mod_ok
}

# BARRIÈRE — le legacy n'avait que le `sleep 15` puis plus rien.
#   1. le processus vit ;
#   2. il a écrit au moins une ligne (si_bridge_loop.sh annonce sa bannière
#      « [si-bridge] 2 grgsm … » dès son démarrage : sa présence prouve que le
#      script a été trouvé et exécuté, pas seulement lancé).
mod_grgsm_decode_wait() {
    local pid log; pid="$(cat "${RUN_DIR}/grgsm-decode.pid" 2>/dev/null || echo 0)"
    log="${LOG_DIR}/grgsm_decode.log"

    wait_until "${MOD_TIMEOUT[grgsm-decode]}" "première sortie du décodeur" test -s "$log" || {
        mod_hint "lisez $log ; si-bridge exige le venv gnuradio (/root/.env) et si_bridge.py"
        mod_fail "le décodeur n'a rien écrit"
        return $MOD_RC_FAIL; }
    kill -0 "$pid" 2>/dev/null || { mod_hint "lisez $log"
                                    mod_fail "le décodeur a démarré puis s'est arrêté"
                                    return $MOD_RC_FAIL; }
    mod_ok
}

mod_grgsm_decode_stop() {
    local pid; pid="$(cat "${RUN_DIR}/grgsm-decode.pid" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    # si_bridge_loop.sh relance si_bridge.py en boucle : il faut tuer les deux,
    # sinon l'enfant survit à la mort de la boucle et continue de consommer le FIFO.
    pkill -f "si_bridge_loop.sh" 2>/dev/null
    pkill -f "si_bridge.py"      2>/dev/null
    pkill -f "grgsm_relay_decode.py" 2>/dev/null
    rm -f "${RUN_DIR}/grgsm-decode.pid"
    return 0
}
