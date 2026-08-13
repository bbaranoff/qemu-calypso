# =============================================================================
#  55-ipc-device — pont calypso-ipc-device (QEMU UDP ⇄ mémoire partagée IPC)
# =============================================================================
#
#  RÔLE        Lance le pont entre l'I/Q de QEMU (UDP 6702) et la mémoire
#              partagée attendue par osmo-trx-ipc. Legacy : L1939-1967.
#              Fork d'ipc-driver-test où le wrapper UHD est remplacé par :
#              DL = cs16 shm → UDP vers QEMU ; UL = recv UDP → shm.
#
#  ORDRE       Ce device DOIT démarrer AVANT osmo-trx-ipc : c'est LUI qui crée
#              le socket maître Unix ($IPC_MSOCK_PATH) auquel osmo-trx-ipc se
#              connecte. Inversé, osmo-trx-ipc sort immédiatement sur un
#              greeting sans réponse — panne classique et illisible.
#
#  PRÉREQUIS   QEMU démarré ; binaire calypso-ipc-device compilé.
#  SUCCÈS      Le socket maître existe ET le processus est toujours vivant.
#  JOURNAL     $IPC_DEVICE_LOG (défaut $LOG_DIR/calypso-ipc-device.log)
#
#  ÉCART ASSUMÉ AVEC LE LEGACY : binaire absent = ÉCHEC, pas un simple WARN.
#  Le legacy affichait « [TODO] pas encore implémenté » puis continuait ; la
#  chaîne radio était alors morte sans que la moindre étape ne le dise.
# -----------------------------------------------------------------------------

MOD_REGISTER ipc-device "Pont I/Q calypso-ipc-device"
MOD_REQUIRED[ipc-device]=0
MOD_DEPS[ipc-device]="qemu"
MOD_PROFILES[ipc-device]="calypso hybrid"
MOD_TIMEOUT[ipc-device]=20      # legacy : 30 tentatives × 0,5 s (L1952)
MOD_ENABLED_IF[ipc-device]='[ "${CALYPSO_SKIP_IPC_DEVICE:-0}" != "1" ]'

: "${CALYPSO_IPC_DEVICE:=${QEMU_TOOLS:-${QEMU_TREE:-${QEMU_TREE}}/tools}/calypso-ipc-device/calypso-ipc-device}"
: "${IPC_SOCK_DIR:=/tmp}"
: "${IPC_MSOCK_PATH:=$IPC_SOCK_DIR/ipc_sock0}"
: "${IPC_DEVICE_LOG:=${LOG_DIR:-/root/calypso/logs}/calypso-ipc-device.log}"
# Paramètres du pont, repris tels quels du legacy (L1950). Idiome `:=` : une
# valeur posée en ligne de commande ou par environnement/ gagne toujours.
: "${CALYPSO_IPC_RELAY:=0}"
: "${CALYPSO_TRX_IQ_HOST:=127.0.0.1}"
: "${CALYPSO_TRX_IQ_RX_PORT:=5810}"
: "${CALYPSO_TRX_IQ_TX_PORT:=5811}"
: "${CALYPSO_RELAY_FIFOS:=/tmp/iq_fft.fifo:/tmp/iq_grgsm.fifo:/tmp/iq_record.fifo:/tmp/iq_asciifft.fifo}"

mod_ipc_device_check() {
    [ -x "$CALYPSO_IPC_DEVICE" ] || {
        mod_hint "compilez-le : make -C ${QEMU_TREE:-.}/tools/calypso-ipc-device — ou CALYPSO_SKIP_IPC_DEVICE=1 pour vous en passer"
        mod_fail "calypso-ipc-device introuvable ou non exécutable : $CALYPSO_IPC_DEVICE"
        return $MOD_RC_FAIL
    }
    [ -d "$IPC_SOCK_DIR" ] || {
        mod_fail "répertoire de socket inexistant : $IPC_SOCK_DIR"
        return $MOD_RC_FAIL
    }
    mod_ok
}

mod_ipc_device_status() { have_proc "calypso-ipc-device"; }

mod_ipc_device_start() {
    mkdir -p "${RUN_DIR:-/tmp/calypso}" "$(dirname "$IPC_DEVICE_LOG")" 2>/dev/null || true
    : > "$IPC_DEVICE_LOG" 2>/dev/null || true
    export CALYPSO_IPC_RELAY CALYPSO_TRX_IQ_HOST CALYPSO_TRX_IQ_RX_PORT \
           CALYPSO_TRX_IQ_TX_PORT CALYPSO_RELAY_FIFOS
    # [2026-08-12] LES 5 EXPORTS CI-DESSUS NE SUFFISENT PAS.
    #
    # `qemu_wrap.c` lit 56 variables d'environnement — tout le montant : RACH,
    # SDCCH, TCH, SACCH, chiffrement A5, offsets de TOA, dedup SABM. Les cinq
    # ci-dessus sont les seules a avoir jamais ete exportees ici. Les autres ne
    # traversaient que si la chaine d'appel avait deja tout exporte (run.sh:286
    # source load.env sous `set -a`, et calypso.env:51 y charge le fichier de
    # profil). Cette condition N'EST PAS GARANTIE : mesure du 12/08, un
    # calypso-ipc-device vivant avait ZERO variable CALYPSO_* dans son environ
    # (`tr '\0' '\n' < /proc/<pid>/environ | grep -c '^CALYPSO_'`) quand QEMU en
    # avait 183 — il pendait sous un autre serveur tmux, donc sous un
    # environnement fossilise. Toutes ses gates tournaient au defaut compile,
    # EN SILENCE : rien dans aucun journal ne distingue « gate posee a 0 » de
    # « gate jamais recue ».
    #
    # On exporte donc explicitement, depuis le shell du module — qui, lui, a bien
    # les valeurs. `export` sur une variable non definie n'ajoute rien a
    # l'environnement : les gates non posees restent absentes et le binaire garde
    # son defaut, exactement comme avant. Aucun changement de comportement, juste
    # la garantie que ce qui EST pose ARRIVE.
    #
    # ⚠️ Cette liste doit suivre `qemu_wrap.c`. Pour la regenerer :
    #   grep -ohE 'getenv\("[A-Z0-9_]+"\)' tools/calypso-ipc-device/*.c \
    #     | sed 's/.*getenv("//;s/")//' | sort -u
    export CALYPSO_BSP_CONT_FORWARD CALYPSO_BSP_HOST CALYPSO_BSP_PORT \
           CALYPSO_CIPH_A5 CALYPSO_CIPH_FN_ADJ CALYPSO_DL_BURST_OFFSET \
           CALYPSO_DL_FIFO_CATCHUP_OFF CALYPSO_DL_IQ_CONJ \
           CALYPSO_FCCH_DUMP CALYPSO_FCCH_DUMP_N CALYPSO_FCCH_DUMP_SKIP \
           CALYPSO_IPC_UL CALYPSO_QFN_FLOOR_NS CALYPSO_QFN_FORCE \
           CALYPSO_QFN_LEAD CALYPSO_RELAY_ALSO_BSP \
           CALYPSO_TCH_SACCH_CAL CALYPSO_TCH_SACCH_CLOCK CALYPSO_TDMA_NS \
           CALYPSO_UL_ACTIVE_SYMS CALYPSO_UL_AMP CALYPSO_UL_BSIC \
           CALYPSO_UL_DEBUG CALYPSO_UL_FN_ADJ CALYPSO_UL_FN_GATE \
           CALYPSO_UL_FN_LOCK CALYPSO_UL_FN_OFFSET CALYPSO_UL_GMSK \
           CALYPSO_UL_HOLD_IFRAME CALYPSO_UL_INVERT CALYPSO_UL_IQ_RECORD \
           CALYPSO_UL_RA CALYPSO_UL_RACH_ENC CALYPSO_UL_RACH_ONCE \
           CALYPSO_UL_RACH_REPS CALYPSO_UL_RACH_STICKY CALYPSO_UL_ROT \
           CALYPSO_UL_ROT_SGN CALYPSO_UL_SABM_DEDUP CALYPSO_UL_SABM_HOLD \
           CALYPSO_UL_SABM_HOLD_TTL CALYPSO_UL_SABM_STICKY \
           CALYPSO_UL_SDCCH CALYPSO_UL_SDCCH_OFS CALYPSO_UL_SDCCH_SMP_OFS \
           CALYPSO_UL_SLOT_OFFSET CALYPSO_UL_TCH CALYPSO_UL_TCH_SACCH_BID \
           CALYPSO_UL_TCH_SACCH_OFS CALYPSO_UL_TCH_SMP_OFS
    mod_say "relay=$CALYPSO_IPC_RELAY iq=$CALYPSO_TRX_IQ_HOST rx=$CALYPSO_TRX_IQ_RX_PORT tx=$CALYPSO_TRX_IQ_TX_PORT"
    # Les deux offsets de TOA du burst normal decident de la boucle TA de la BTS
    # (toa constant -> TA qui rampe +2 par SACCH jusqu'a 63). On les annonce au
    # demarrage : sans ca, « la gate est-elle arrivee ? » n'est pas decidable.
    mod_say "TOA montant : SDCCH_SMP_OFS=${CALYPSO_UL_SDCCH_SMP_OFS:-<defaut 0>} TCH_SMP_OFS=${CALYPSO_UL_TCH_SMP_OFS:-<defaut 0>}"
    mod_say "socket maître attendu : $IPC_MSOCK_PATH"

    stdbuf -oL -eL "$CALYPSO_IPC_DEVICE" -u "$IPC_SOCK_DIR" -n 0 >>"$IPC_DEVICE_LOG" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR:-/tmp/calypso}/ipc-device.pid"
    mod_ok
}

# BARRIÈRE — remplace le `sleep 0.5` × 30 du legacy (L1954), qui se contentait
# d'un WARN si la socket manquait. Deux conditions, dans cet ordre :
#   1. le socket maître est créé (c'est le contrat vis-à-vis d'osmo-trx-ipc) ;
#   2. le processus est TOUJOURS vivant — un device qui crée sa socket puis
#      meurt sur un bind UDP déjà pris laisse un fichier socket trompeur,
#      exactement le faux positif corrigé dans 40-qemu.
mod_ipc_device_wait() {
    local pidf="${RUN_DIR:-/tmp/calypso}/ipc-device.pid" pid
    if ! wait_until "${MOD_TIMEOUT[ipc-device]}" "socket maître IPC ($IPC_MSOCK_PATH)" \
            have_unix "$IPC_MSOCK_PATH"; then
        modb_tail "$IPC_DEVICE_LOG" 20
        mod_hint "un ancien device peut encore tenir $IPC_MSOCK_PATH : ./run.sh --stop puis relancez"
        mod_fail "calypso-ipc-device n'a pas créé $IPC_MSOCK_PATH en ${MOD_TIMEOUT[ipc-device]}s"
        return $MOD_RC_FAIL
    fi
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    if [ "$pid" != 0 ] && ! kill -0 "$pid" 2>/dev/null; then
        modb_tail "$IPC_DEVICE_LOG" 20
        mod_hint "vérifiez que les ports UDP $CALYPSO_TRX_IQ_RX_PORT/$CALYPSO_TRX_IQ_TX_PORT sont libres"
        mod_fail "calypso-ipc-device a créé sa socket puis s'est arrêté"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_ipc_device_stop() {
    local pidf="${RUN_DIR:-/tmp/calypso}/ipc-device.pid" pid
    pid="$(cat "$pidf" 2>/dev/null || echo 0)"
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "calypso-ipc-device" 2>/dev/null
    # Le socket maître ne disparaît pas tout seul : laissé en place, il ferait
    # croire au run suivant que le device est déjà là.
    rm -f "$pidf" "$IPC_MSOCK_PATH" "${IPC_MSOCK_PATH}_0"
    return 0
}
