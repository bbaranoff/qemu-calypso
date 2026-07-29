# =============================================================================
#  07-coherence — règles d'exclusion et d'implication entre composants
# =============================================================================
#
#  RÔLE
#    Après le preset (06) et les surcharges de l'opérateur, certaines
#    combinaisons n'ont pas de sens : osmo-trx-ipc sans calypso-ipc-device
#    échoue à son greeting, bridge.py et ipc-device se disputent les mêmes
#    ports, un BTS sans pont radio reste muet. Ce module applique les règles 1-5
#    (legacy L1230-1291) et les règles comportementales R6-R10 (L1393-1424).
#
#  PRÉREQUIS  : 06-pipeline (les sept sélecteurs sont posés).
#  SUCCÈS     : aucune paire interdite ne subsiste en sortie ; chaque forçage
#               est écrit dans le journal avec sa raison.
#  JOURNAL    : $LOG_DIR/mod/coherence.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. CE MODULE NE FORCE QUE DES VARIABLES DE MOTEUR.
#     Les règles 1-4 et R7 ne touchent qu'aux CALYPSO_SKIP_* et à
#     CALYPSO_SKIP_GDB_PANE : des sélecteurs lus par run.sh seul, jamais par
#     hw/ (vérifié). Les forcer est sans effet sur l'émulation.
#     R8 et R10, elles, portaient sur CALYPSO_BSP_IQ_PASSTHROUGH et
#     CALYPSO_DSP_IDLE_FF — deux variables LUES PAR LE MODÈLE C. Les forcer
#     depuis ici changerait l'émulation du chemin de référence
#     (`… ./start-clean.sh`, qui fait `exec ./run.sh`) sans que personne l'ait
#     demandé, et contredirait la convention de environnement/ où une valeur
#     vide signifie « laisser le défaut du C ». Ces deux règles SIGNALENT donc,
#     et laissent la correction à environnement/ ou à la ligne de commande.
#
#  2. POURQUOI UN CHOIX EXPLICITE DE L'OPÉRATEUR N'EST JAMAIS ÉCRASÉ.
#     Le legacy consultait des _USER_SET_SKIP_* que seul le menu whiptail
#     posait : hors menu, la règle 1 écrasait en silence un `CALYPSO_SKIP_BTS=0`
#     passé en ligne de commande. 06 mémorise le vrai « posé par l'opérateur »
#     dans PIPELINE_USER_SET_* ; on le respecte, en le disant.
#
#  3. RIEN ICI N'EST FATAL.
#     Une incohérence se répare (on désactive le composant en trop) ou se
#     signale (le BTS restera en idle). Le seul échec possible est structurel :
#     les sélecteurs ne sont pas posés, c'est-à-dire que 06 n'a pas tourné.
# -----------------------------------------------------------------------------

MOD_REGISTER coherence "Cohérence des composants"
MOD_REQUIRED[coherence]=1
MOD_DEPS[coherence]="pipeline"
MOD_PROFILES[coherence]="calypso hybrid faketrx"
MOD_PURE[coherence]=1
MOD_TIMEOUT[coherence]=5

mod_coherence_check() {
    local s isset
    for s in IPC_DEVICE TRX_IPC BTS L2 GSMTAP BRIDGE_PY DEMOD_BRIDGE; do
        eval "isset=\${CALYPSO_SKIP_${s}+x}"
        if [ -z "$isset" ]; then
            mod_hint "ce module suppose 06-pipeline joué : ./run.sh --only pipeline,coherence"
            mod_fail "sélecteur CALYPSO_SKIP_${s} non posé — 06-pipeline n'a pas tourné"
            return $MOD_RC_FAIL
        fi
    done
    mod_ok
}

mod_coherence_status() { return $MOD_RC_FAIL; }   # module pur

# Force un sélecteur, sauf si l'opérateur l'a posé lui-même (cf. POURQUOI 2).
_coh_force() {
    local name="$1" val="$2" why="$3" cur user
    eval "cur=\${CALYPSO_SKIP_${name}:-}"
    eval "user=\${PIPELINE_USER_SET_${name}:-0}"
    [ "$cur" = "$val" ] && return 0
    if [ "$user" = "1" ]; then
        mod_say "RESPECTÉ : CALYPSO_SKIP_${name}=$cur posé par l'opérateur — la règle « $why » aurait mis $val"
        return 0
    fi
    eval "CALYPSO_SKIP_${name}=$val"
    mod_say "FORCÉ    : CALYPSO_SKIP_${name}=$val — $why"
}

mod_coherence_start() {
    local pipe="${CALYPSO_PIPELINE:-}" shunt="${CALYPSO_DSP_SHUNT:-}"

    # --- Règle 1 (legacy L1234-1253) -----------------------------------------
    # Shunt DSP explicite : le mock répond en canné, la chaîne radio ne sert à
    # rien. Deux exemptions d'origine : shunt-ipc (l'I/Q réelle alimente le DSP)
    # et full-grgsm (le shunt est léger mais gr-gsm CONSOMME l'I/Q relayée ;
    # sans la chaîne radio, feed_si ne tire jamais — legacy L1237-1239).
    # NB : la règle ne peut plus se déclencher « par preset », puisque 06 ne
    # pose plus CALYPSO_DSP_SHUNT ; elle ne joue que si l'opérateur l'a posé.
    if [ "$shunt" = "1" ] && [ "$pipe" != "shunt-ipc" ] && [ "$pipe" != "full-grgsm" ]; then
        _coh_force IPC_DEVICE 1 "CALYPSO_DSP_SHUNT=1 : sorties cannées, pas de radio"
        _coh_force TRX_IPC    1 "CALYPSO_DSP_SHUNT=1 : sorties cannées, pas de radio"
        _coh_force BTS        1 "CALYPSO_DSP_SHUNT=1 : sorties cannées, pas de radio"
    fi

    # --- Règle 2 (legacy L1255-1264) : bridge.py XOR (ipc-device | trx-ipc) --
    # Les deux ponts servent le même rôle sur les mêmes ports UDP 5700-5702.
    # bridge.py gagne : on ne l'active que par choix explicite (pipeline bridge).
    if [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" = "0" ]; then
        if [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "0" ] || [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ]; then
            _coh_force IPC_DEVICE 1 "MUTEX bridge.py <-> ipc-device (mêmes ports)"
            _coh_force TRX_IPC    1 "MUTEX bridge.py <-> osmo-trx-ipc (mêmes ports)"
        fi
    fi

    # --- Règle 3 (legacy L1266-1271) -----------------------------------------
    # osmo-trx-ipc se connecte au socket maître publié par calypso-ipc-device.
    # Sans lui, il échoue au greeting et meurt — en silence, dans le legacy.
    if [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ] && [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "1" ]; then
        _coh_force TRX_IPC 1 "osmo-trx-ipc exige calypso-ipc-device (greeting sur ${IPC_MSOCK_PATH:-/tmp/ipc_sock0})"
    fi

    # --- Règle 4 (legacy L1273-1280) : BTS sans pont radio -------------------
    # Non corrigeable automatiquement : lancer le BTS reste légitime (OML/RSL
    # montent, il attend un transceiver). On le dit, on ne touche à rien.
    if [ "${CALYPSO_SKIP_BTS:-1}" = "0" ] \
       && [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "1" ] \
       && [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" = "1" ]; then
        mod_say "SIGNALÉ  : osmo-bts-trx sans pont radio (ni trx-ipc ni bridge.py) — il restera en idle"
    fi

    # --- Règle 5 (legacy L1282-1288) : MTTCG <-> icount ----------------------
    # Arbitrage effectif dans 08-accel, qui construit les drapeaux. Ici on trace
    # seulement, pour que la contradiction apparaisse dans l'ordre de lecture.
    if [ "${CALYPSO_MTTCG:-0}" = "1" ] && [ "${CALYPSO_ICOUNT:-auto}" != "off" ]; then
        mod_say "SIGNALÉ  : MTTCG=1 et ICOUNT=${CALYPSO_ICOUNT:-auto} sont exclusifs — 08-accel n'émettra pas -icount"
    fi

    # --- R6 (legacy L1398-1401) ----------------------------------------------
    if [ "$shunt" = "1" ] && [ -n "${CALYPSO_DEBUG:-}" ]; then
        mod_say "SIGNALÉ  : CALYPSO_DSP_SHUNT=1 court-circuite le c54x — les sondes CALYPSO_DEBUG du c54x resteront muettes"
    fi

    # --- R7 (legacy L1402-1406) : -S sans pane gdb = boot figé ---------------
    # CALYPSO_SKIP_GDB_PANE est une variable de moteur (sélection d'un pane
    # tmux) : la forcer est sans effet sur l'émulation.
    if [ "${CALYPSO_QEMU_HALT:-0}" = "1" ] && [ "${CALYPSO_SKIP_GDB_PANE:-1}" = "1" ]; then
        CALYPSO_SKIP_GDB_PANE=0
        mod_say "FORCÉ    : CALYPSO_SKIP_GDB_PANE=0 — QEMU démarre halté (-S), sans pane gdb personne ne le relance"
    fi

    # --- R8 (legacy L1407-1415) : signalement seul, cf. POURQUOI 1 -----------
    if [ "$shunt" = "1" ] && [ "${CALYPSO_BSP_IQ_PASSTHROUGH:-}" = "1" ] && [ "$pipe" != "full-grgsm" ]; then
        mod_say "SIGNALÉ  : DSP_SHUNT=1 court-circuite le BSP, BSP_IQ_PASSTHROUGH=1 n'aura pas d'effet."
        mod_say "           correction : videz CALYPSO_BSP_IQ_PASSTHROUGH dans environnement/bsp.env (L51)"
        mod_say "           — non forcé ici : variable lue par le modèle C, hors périmètre d'un module de plomberie"
    fi

    # --- R9 (legacy L1416-1419) ----------------------------------------------
    if [ "${CALYPSO_FORCE_INTM_ONESHOT:-0}" = "1" ]; then
        mod_say "SIGNALÉ  : CALYPSO_FORCE_INTM_ONESHOT=1 est NON nominal (force le clear INTM) — mesure sous béquille"
    fi

    # --- R10 (legacy L1420-1424) : signalement seul, cf. POURQUOI 1 ----------
    if [ -n "${CALYPSO_DSP_IDLE_RANGE:-}" ] && [ "${CALYPSO_DSP_IDLE_FF:-}" = "0" ]; then
        mod_say "SIGNALÉ  : CALYPSO_DSP_IDLE_RANGE est posé mais CALYPSO_DSP_IDLE_FF=0 — la plage ne sera jamais utilisée."
        mod_say "           correction : CALYPSO_DSP_IDLE_FF=1 en préfixe, ou dans environnement/dsp.env"
    fi

    calypso_export_sweep
    mod_ok
}

# BARRIÈRE — critère observable : plus aucune paire interdite en sortie.
# On revérifie l'état APRÈS les forçages plutôt que de faire confiance à la
# séquence : c'est ce contrôle qui distingue « la règle a tourné » de « la règle
# a effectivement réparé ».
_coh_conflicts() {
    local out=""
    [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" = "0" ] && [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "0" ] \
        && out="$out bridge.py+ipc-device"
    [ "${CALYPSO_SKIP_BRIDGE_PY:-1}" = "0" ] && [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ] \
        && out="$out bridge.py+trx-ipc"
    [ "${CALYPSO_SKIP_TRX_IPC:-1}" = "0" ] && [ "${CALYPSO_SKIP_IPC_DEVICE:-1}" = "1" ] \
        && out="$out trx-ipc-sans-ipc-device"
    printf '%s' "${out# }"
}
_coh_clean() { [ -z "$(_coh_conflicts)" ]; }

mod_coherence_wait() {
    if ! wait_until "${MOD_TIMEOUT[coherence]}" "cohérence des composants" _coh_clean; then
        mod_hint "un CALYPSO_SKIP_* explicite bloque la réparation : retirez-le, ou changez de CALYPSO_PIPELINE"
        mod_fail "combinaison interdite conservée : $(_coh_conflicts)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_coherence_stop() { return 0; }
