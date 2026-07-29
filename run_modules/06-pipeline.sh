# =============================================================================
#  06-pipeline — quel jeu de composants on lance (preset ex-CALYPSO_MODE)
# =============================================================================
#
#  RÔLE
#    Choisir, en un seul mot, QUELS composants de la chaîne doivent tourner :
#    ipc-device, osmo-trx-ipc, osmo-bts-trx, client L2, gsmtap, bridge.py,
#    bridge de démodulation. Reprend run.sh.legacy #12 (L1071-1228).
#
#  PRÉREQUIS  : 05-config.
#  SUCCÈS     : CALYPSO_PIPELINE contient une valeur de la liste close, et les
#               sept variables de sélection sont posées et exportées.
#  JOURNAL    : $LOG_DIR/mod/pipeline.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI CE N'EST PLUS `CALYPSO_MODE`.
#     Collision de noms, sur deux axes ORTHOGONAUX :
#       - environnement/modes.env L9-11 : CALYPSO_MODE = shunt_legit | native |
#         native_helped — axe ÉMULATION (comment le DSP est traité) ;
#       - run.sh.legacy L1108-1117 : CALYPSO_MODE = full | full-grgsm | shunt |
#         shunt-ipc | bridge | bare | free — axe COMPOSANTS (qui est lancé).
#     Réutiliser le même nom ferait tomber `CALYPSO_MODE=full-grgsm ./run.sh`
#     dans le `*)` de modes.env (« mode inconnu ») et perdrait le profil
#     d'émulation. L'axe composants prend donc le nom CALYPSO_PIPELINE.
#     Compatibilité : si CALYPSO_MODE porte une valeur de l'ancienne liste, on
#     la récupère comme pipeline et on le dit — modes.env aura déjà râlé sur
#     stderr, c'est le seul symptôme visible et il est expliqué dans le journal.
#     Ni CALYPSO_MODE ni CALYPSO_PIPELINE ne sont lus par le modèle C
#     (vérifié : aucun getenv dans hw/), la collision est purement côté shell.
#
#  2. POURQUOI CALYPSO_DSP_SHUNT N'EST PAS POSÉ, CONTRAIREMENT AU LEGACY.
#     Le legacy posait `CALYPSO_DSP_SHUNT=1` dans le preset full-grgsm
#     (L1137). Cette variable-là EST lue par le modèle : calypso_dsp_shunt.c
#     :2034 (armement) et :2266 (calypso_dsp_shunt_substitutes → tous les
#     c54x_run natifs sont gatés). La poser ferait basculer le chemin de
#     référence — `CALYPSO_SHUNT_LEGIT=1 … ./start-clean.sh`, qui fait
#     `exec ./run.sh` — d'un c54x qui tourne à un c54x entièrement remplacé.
#     Ce serait un changement d'émulation décidé par un module de plomberie.
#     06 ne touche donc QUE les variables de sélection de composants (jamais
#     lues par hw/) et se contente de TRACER la valeur effective du shunt.
#     Le jour où on veut le comportement legacy : le poser dans
#     environnement/modes.env, au bon endroit, sur le bon axe.
#
#  3. POURQUOI ON MÉMORISE « L'OPÉRATEUR A CHOISI ».
#     Les règles de cohérence du legacy (07) consultaient des _USER_SET_SKIP_*
#     que seul le menu whiptail savait poser : hors menu, elles écrasaient
#     silencieusement un choix explicite. Ici la mémorisation se fait AVANT
#     d'appliquer le preset : ce qui est déjà dans l'environnement à cet
#     instant vient forcément de la ligne de commande ou du shell de
#     l'opérateur (aucun fichier de environnement/ ne pose de CALYPSO_SKIP_*).
# -----------------------------------------------------------------------------

MOD_REGISTER pipeline "Preset de composants"
MOD_REQUIRED[pipeline]=1
MOD_DEPS[pipeline]="config"
MOD_PROFILES[pipeline]="calypso hybrid faketrx"
MOD_PURE[pipeline]=1
MOD_TIMEOUT[pipeline]=5

: "${CALYPSO_PIPELINE_DEFAULT:=full-grgsm}"   # pipeline validé 2026-06-03 (legacy L1108)
_PIPELINE_VALUES="full full-grgsm shunt shunt-ipc bridge bare free"

mod_pipeline_check() {
    # Lecture seule : on ne valide QUE la valeur demandée. La récupération
    # depuis CALYPSO_MODE se fait ici pour que l'erreur tombe avant tout effet.
    local want="${CALYPSO_PIPELINE:-}"
    if [ -z "$want" ]; then
        case " $_PIPELINE_VALUES " in
            *" ${CALYPSO_MODE:-} "*) want="$CALYPSO_MODE" ;;   # compat, cf. POURQUOI 1
            *)                       want="$CALYPSO_PIPELINE_DEFAULT" ;;
        esac
    fi
    case " $_PIPELINE_VALUES " in
        *" $want "*) mod_ok ;;
        *) mod_hint "valeurs : $_PIPELINE_VALUES"
           mod_fail "CALYPSO_PIPELINE inconnu : '$want'"
           return $MOD_RC_FAIL ;;
    esac
}

mod_pipeline_status() { return $MOD_RC_FAIL; }   # module pur

mod_pipeline_start() {
    # --- 1. valeur retenue ---------------------------------------------------
    if [ -z "${CALYPSO_PIPELINE:-}" ]; then
        case " $_PIPELINE_VALUES " in
            *" ${CALYPSO_MODE:-} "*)
                CALYPSO_PIPELINE="$CALYPSO_MODE"
                mod_say "CALYPSO_MODE='$CALYPSO_MODE' est une valeur de l'ANCIEN axe composants."
                mod_say "  reprise comme CALYPSO_PIPELINE ; modes.env a déjà signalé « mode inconnu »"
                mod_say "  sur stderr — c'est attendu. Utilisez CALYPSO_PIPELINE=$CALYPSO_MODE."
                ;;
            *)  CALYPSO_PIPELINE="$CALYPSO_PIPELINE_DEFAULT" ;;
        esac
    fi

    # --- 2. mémoriser les choix explicites AVANT le preset (cf. POURQUOI 3) --
    local s isset
    for s in IPC_DEVICE TRX_IPC BTS L2 GSMTAP BRIDGE_PY DEMOD_BRIDGE; do
        eval "isset=\${CALYPSO_SKIP_${s}+x}"
        if [ -n "$isset" ]; then
            eval "PIPELINE_USER_SET_${s}=1"
            mod_say "choix explicite de l'opérateur : CALYPSO_SKIP_${s}=$(eval "printf '%s' \"\$CALYPSO_SKIP_${s}\"")"
        else
            eval "PIPELINE_USER_SET_${s}=0"
        fi
    done

    # --- 3. preset -----------------------------------------------------------
    # `:=` partout : une surcharge posée en préfixe gagne toujours sur le preset.
    local shunt_hint=0
    case "$CALYPSO_PIPELINE" in
      full)         # chaîne radio complète, DSP natif
        shunt_hint=0
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"; : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
      full-grgsm)   # radio réelle relayée vers gr-gsm, qui décode (legacy L1131-1163)
        shunt_hint=1
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"; : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
      shunt)        # bissection FBSB, aucune radio
        shunt_hint=1
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"; : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=1}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=1}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
      shunt-ipc)    # I/Q réelle -> DSP natif -> gr-gsm décode la sortie
        shunt_hint=0
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"; : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=0}" ;;
      bridge)       # pont Python historique, à la place d'ipc/trx-ipc
        shunt_hint=0
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"; : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=0}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}";     : "${CALYPSO_SKIP_BRIDGE_PY:=0}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
      bare)         # QEMU + osmocon seuls (debug firmware, gdb)
        shunt_hint=0
        : "${CALYPSO_SKIP_IPC_DEVICE:=1}"; : "${CALYPSO_SKIP_TRX_IPC:=1}"
        : "${CALYPSO_SKIP_BTS:=1}";        : "${CALYPSO_SKIP_L2:=1}"
        : "${CALYPSO_SKIP_GSMTAP:=1}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
      free)         # aucun preset : à l'opérateur de tout poser. Défauts sûrs.
        shunt_hint=0
        : "${CALYPSO_SKIP_IPC_DEVICE:=0}"; : "${CALYPSO_SKIP_TRX_IPC:=0}"
        : "${CALYPSO_SKIP_BTS:=0}";        : "${CALYPSO_SKIP_L2:=0}"
        : "${CALYPSO_SKIP_GSMTAP:=0}";     : "${CALYPSO_SKIP_BRIDGE_PY:=1}"
        : "${CALYPSO_SKIP_DEMOD_BRIDGE:=1}" ;;
    esac

    mod_say "CALYPSO_PIPELINE = $CALYPSO_PIPELINE"
    mod_say "  ipc-device=$(_pipeline_state "$CALYPSO_SKIP_IPC_DEVICE")  trx-ipc=$(_pipeline_state "$CALYPSO_SKIP_TRX_IPC")  bts=$(_pipeline_state "$CALYPSO_SKIP_BTS")"
    mod_say "  l2=$(_pipeline_state "$CALYPSO_SKIP_L2")  gsmtap=$(_pipeline_state "$CALYPSO_SKIP_GSMTAP")  bridge.py=$(_pipeline_state "$CALYPSO_SKIP_BRIDGE_PY")  demod-bridge=$(_pipeline_state "$CALYPSO_SKIP_DEMOD_BRIDGE")"

    # --- 4. le shunt DSP : tracé, jamais posé (cf. POURQUOI 2) ---------------
    if [ -n "${CALYPSO_DSP_SHUNT:-}" ]; then
        mod_say "CALYPSO_DSP_SHUNT = $CALYPSO_DSP_SHUNT (posé par l'opérateur ou par environnement/)"
    else
        mod_say "CALYPSO_DSP_SHUNT non posé -> défaut du modèle C (calypso_dsp_shunt.c:2034)."
        [ "$shunt_hint" = 1 ] && mod_say "  le preset legacy '$CALYPSO_PIPELINE' aurait posé 1 ; non repris : cela changerait l'émulation."
    fi

    calypso_export_sweep
    mod_ok
}

_pipeline_state() { [ "${1:-1}" = "0" ] && printf 'ON' || printf 'off'; }

# BARRIÈRE — critère observable et immédiat : les sept sélecteurs valent 0 ou 1.
# Une valeur non binaire (« yes », « true », une faute de frappe) serait lue
# comme « ne pas sauter » par les tests `= 1` des modules suivants : un composant
# se lancerait alors qu'on croyait l'avoir désactivé.
_pipeline_binary() {
    local s v
    for s in IPC_DEVICE TRX_IPC BTS L2 GSMTAP BRIDGE_PY DEMOD_BRIDGE; do
        eval "v=\${CALYPSO_SKIP_${s}:-}"
        case "$v" in 0|1) ;; *) return 1;; esac
    done
    return 0
}
_pipeline_bad() {
    local s v out=""
    for s in IPC_DEVICE TRX_IPC BTS L2 GSMTAP BRIDGE_PY DEMOD_BRIDGE; do
        eval "v=\${CALYPSO_SKIP_${s}:-}"
        case "$v" in 0|1) ;; *) out="$out CALYPSO_SKIP_${s}='$v'";; esac
    done
    printf '%s' "${out# }"
}

mod_pipeline_wait() {
    if ! wait_until "${MOD_TIMEOUT[pipeline]}" "sélecteurs de composants" _pipeline_binary; then
        mod_hint "chaque CALYPSO_SKIP_* vaut 0 (lancer) ou 1 (sauter) — rien d'autre"
        mod_fail "sélecteur non binaire : $(_pipeline_bad)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_pipeline_stop() { return 0; }
