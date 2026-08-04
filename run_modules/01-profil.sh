# =============================================================================
#  01-profil — profil d'émulation et propagation des réglages      (module PUR)
# =============================================================================
#
#  RÔLE
#      Ne lance rien, ne crée rien. Vérifie que environnement/load.env a produit
#      une configuration COHÉRENTE, et surtout qu'elle franchit la frontière de
#      processus : c'est par l'environnement exporté, et par rien d'autre, que
#      les CALYPSO_* atteignent QEMU. Un `set -a` oublié dans run.sh ne se
#      verrait nulle part — le modèle démarrerait avec ses défauts, et le
#      manifeste imprimé contredirait la ligne de commande sans que personne
#      ne le remarque.
#
#  PRÉREQUIS
#      run.sh a sourcé environnement/load.env sous `set -a` AVANT de sourcer les
#      modules (run.sh:75-80). Aucun autre module.
#
#  CRITÈRE DE SUCCÈS
#      1. CALYPSO_MODE vaut un des profils déclarés par modes.env ;
#      2. les variables structurantes sont non vides (GSM_ROOT, QEMU_TREE,
#         QEMU_BIN, RUN_DIR, LOG_DIR) ;
#      3. aucun mélange de profils (béquilles « natif aidé » allumées alors que
#         le profil demandé est shunt_legit, ou l'inverse) ;
#      4. BARRIÈRE — un processus ENFANT réel voit CALYPSO_MODE et QEMU_BIN dans
#         son environnement. C'est le seul critère qui prouve l'export.
#
#  JOURNAL
#      $LOG_DIR/mod/profil.log : manifeste des variables retenues, à comparer au
#      manifeste imprimé par le modèle (grep calypso-manifest sur qemu.log).
#      La vérité est le manifeste de QEMU, jamais la ligne de commande.
# -----------------------------------------------------------------------------
MOD_REGISTER profil "Profil d'émulation et propagation"
MOD_REQUIRED[profil]=1
MOD_PURE[profil]=1
MOD_PROFILES[profil]="calypso faketrx hybrid core"
MOD_TIMEOUT[profil]=10

# Profils connus — doit rester aligné sur le `case` de environnement/modes.env.
# ⚠️ [2026-08-03] CETTE LISTE EST LA TROISIÈME COPIE du même ensemble : il y a
# aussi le `case` de environnement/modes.env et la ligne de `run.sh` (--configure).
# Ajouter un mode dans le `case` seul ne suffit PAS — le garde-fou le rejette
# avant, avec « CALYPSO_MODE inconnu », alors que le profil est bel et bien écrit.
# Vécu le 03/08 en ajoutant native_twl_host_demod. Les trois doivent bouger ensemble.
: "${CONFIG_MODES_CONNUS:=empty none bare shunt_legit shunt_legit_no_inject native native_twl native_twl_host_demod native_helped}"

mod_profil_check() {
    local m="${CALYPSO_MODE:-}" connu=0 x

    if [ -z "$m" ]; then
        mod_hint "environnement/load.env n'a pas été sourcé : lancez ./start-oqc.sh, pas les modules à la main"
        mod_fail "CALYPSO_MODE non défini — la configuration n'a pas été chargée"
        return $MOD_RC_FAIL
    fi
    for x in $CONFIG_MODES_CONNUS; do [ "$x" = "$m" ] && connu=1; done
    if [ $connu -eq 0 ]; then
        mod_hint "profils disponibles : $CONFIG_MODES_CONNUS (voir environnement/modes.env)"
        mod_fail "CALYPSO_MODE inconnu : « $m » — aucun profil n'a été appliqué"
        return $MOD_RC_FAIL
    fi

    local manquantes=""
    for x in GSM_ROOT QEMU_TREE QEMU_BIN RUN_DIR LOG_DIR; do
        [ -n "${!x:-}" ] || manquantes="$manquantes $x"
    done
    if [ -n "$manquantes" ]; then
        mod_hint "ces variables sont posées par environnement/paths.env — vérifiez qu'il est lisible"
        mod_fail "configuration incomplète :$manquantes"
        return $MOD_RC_FAIL
    fi

    # Mélange de profils. modes.env n'utilise que `:=` : une variable héritée de
    # l'environnement de l'appelant survit au profil et le contredit en silence.
    if [ "$m" = "shunt_legit" ] && [ "${CALYPSO_NATIVE_HELPED:-0}" = 1 ]; then
        mod_hint "unset CALYPSO_NATIVE_HELPED, ou passez CALYPSO_MODE=native_helped"
        mod_fail "profils mélangés : CALYPSO_MODE=shunt_legit mais CALYPSO_NATIVE_HELPED=1"
        return $MOD_RC_FAIL
    fi
    # [2026-07-30, corrigé] native_twl ne pose PLUS SHUNT_LEGIT — la 1re version
    # le faisait et se détruisait (le parapluie pose DSP_SHUNT=1 / DSP_RUN_C54X=0).
    # On tolère quand même la valeur ici : quelqu'un peut vouloir comparer à la
    # main. Le contrat du mode, lui, est surveillé par le `case` suivant.
    case "$m" in
        shunt_legit|shunt_legit_no_inject) ;;   # SHUNT_LEGIT=1 légitime
        native_twl)
            # [2026-07-30] native_twl ne pose PLUS SHUNT_LEGIT (il pose ses gates
            # une par une). Sa présence est donc TOUJOURS une fuite d'environnement
            # — tmux fossilise l'env du 1er run de la session. Ce n'est pas
            # cosmétique : le 30/07, un SHUNT_LEGIT=1 hérité a fait tirer feed_si
            # 184 fois alors que FEED_SI=0 était au manifeste, et le banc a
            # « répondu » à sa propre question avec les SI de gr-gsm.
            # [2026-07-30, revu] SIGNALÉ, PAS FATAL. Premier jet : fatal. Trop
            # raide — l'hybride `native_twl` + SHUNT_LEGIT=1 est un banc qu'on
            # veut pouvoir composer à la main, et c'est même le seul raccourci
            # qui donne le transport TWL avec un DSP qui tourne.
            # Ce qui rendait la fuite DANGEREUSE est corrigé le même jour : le
            # repli du parapluie n'écrase plus un `=0` explicite (feed_si dans
            # calypso_dsp_shunt.c, INJECT_ACD dans calypso_dsp_helper.c, tous
            # deux passés à calypso_gate). Donc FEED_SI=0 coupe désormais VRAIMENT,
            # même sous SHUNT_LEGIT=1, et le banc ne peut plus répondre à sa
            # propre question avec les SI de gr-gsm.
            # On le dit quand même : le profil ne pose pas cette variable, donc
            # sa présence vient de l'environnement — tmux fossilise l'env du
            # premier run de la session (./run.sh --restart).
            if [ "${CALYPSO_SHUNT_LEGIT:-0}" = 1 ]; then
                mod_say "NOTE : CALYPSO_SHUNT_LEGIT=1 sous native_twl — le profil ne le"
                mod_say "       pose PAS, donc ça vient de l'environnement (tmux gardе"
                mod_say "       l'env du 1er run ; ./run.sh --restart pour repartir net)."
                mod_say "       Le parapluie sert de DÉFAUT aux injections ; vos FEED_SI=0"
                mod_say "       et INJECT_ACD=0 restent prioritaires depuis le 30/07."
            fi
            ;;
        *)
            if [ "${CALYPSO_SHUNT_LEGIT:-0}" = 1 ]; then
                mod_hint "unset CALYPSO_SHUNT_LEGIT, ou repassez en CALYPSO_MODE=shunt_legit"
                mod_hint "pour la synchro fournie par l'hôte avec des SI décodés par le DSP : CALYPSO_MODE=native_twl"
                mod_fail "profils mélangés : CALYPSO_MODE=$m mais CALYPSO_SHUNT_LEGIT=1"
                return $MOD_RC_FAIL
            fi
            ;;
    esac

    # Signalé, pas fatal : un mode natif peut être contaminé par les gates du
    # partage FB/SB posés à la main. Ce cas a coûté une demi-journée le 29/07 :
    # « CALYPSO_MODE=native » avec SHUNT_NO_LEGIT=1 n'est PAS du natif, et le
    # manifeste seul le disait. On le dit ici aussi, à l'endroit où on regarde.
    case "$m" in
        native|native_helped)
            for x in CALYPSO_SHUNT_NO_LEGIT CALYPSO_SHUNT_REAL_FB CALYPSO_INJECT_SB \
                     CALYPSO_SHUNT_FEED_SI CALYPSO_INJECT_ACD; do
                if [ "${!x:-0}" = 1 ]; then
                    mod_say "ATTENTION : $x=1 avec CALYPSO_MODE=$m — ce n'est plus du natif."
                    mod_say "            le profil prévu pour ça est CALYPSO_MODE=shunt_legit_no_inject"
                fi
            done
            ;;

        # [2026-07-30] CONTRAT, tel que l'utilisateur l'a posé en deux lignes :
        #     native_twl :  FB/SB = TWL (hôte)      SI = DSP
        #     native     :  FB/SB = DSP             SI = DSP
        # La question à laquelle ce banc doit répondre est « le DSP traite-t-il
        # le SI ? », puisqu'on n'obtient pas encore le FB/SB natif. Ce qui le
        # casse, ce n'est donc PAS la substitution FB/SB — elle est son contrat —
        # mais toute entrée de bloc gr-gsm dans a_cd : elle répond à la question
        # à la place du DSP. (Le garde-fou précédent disait l'inverse : il
        # surveillait PUBLISH_FB/REAL_FB. Il gardait l'ancien contrat, inversé.)
        native_twl)
            for x in CALYPSO_SHUNT_FEED_SI CALYPSO_INJECT_ACD; do
                if [ "${!x:-0}" = 1 ]; then
                    mod_say "ATTENTION : $x=1 avec CALYPSO_MODE=native_twl — les SI"
                    mod_say "            viennent alors de gr-gsm, pas du DSP : le banc ne"
                    mod_say "            peut plus répondre à « le DSP traite-t-il le SI ? »."
                    mod_say "            Ce réglage-là, c'est CALYPSO_MODE=shunt_legit."
                fi
            done
            if [ "${CALYPSO_SHUNT_REAL_FB:-0}" != 1 ] || [ "${CALYPSO_INJECT_SB:-0}" != 1 ]; then
                mod_say "NOTE : REAL_FB/INJECT_SB pas tous les deux à 1 — la synchro n'est"
                mod_say "       plus fournie par l'hôte. Sans elle le DSP ne voit pas de"
                mod_say "       burst normal, et le mode perd son objet (utilisez native)."
            fi
            mod_say "RAPPEL native_twl : data[0x08f8] n'est PAS un verdict ici (FB substitué)."
            ;;
    esac

    # Signalé, pas fatal : le binaire peut légitimement venir d'un arbre voisin
    # (paths.env retombe sur $GSM_ROOT/osmo-qemu-calypso/build s'il n'y a pas de build local).
    case "${QEMU_BIN:-}" in
        "${QEMU_TREE:-}"/*) : ;;
        *) mod_say "ATTENTION : QEMU_BIN ($QEMU_BIN) n'appartient pas à QEMU_TREE ($QEMU_TREE)" ;;
    esac
    mod_ok
}

# Module pur : il n'y a rien à « démarrer », donc jamais « déjà démarré ».
mod_profil_status() { return $MOD_RC_FAIL; }

# Aucun effet de bord : on ne fait qu'écrire le manifeste dans le journal du
# module (run.sh redirige la sortie ; les modules n'écrivent pas sur la console).
mod_profil_start() {
    mod_say "profil        : ${CALYPSO_MODE}"
    mod_say "arbre         : ${QEMU_TREE}"
    mod_say "binaire QEMU  : ${QEMU_BIN}"
    mod_say "exécution     : ${RUN_DIR}"
    mod_say "journaux      : ${LOG_DIR}"
    mod_say "--- CALYPSO_* exportés (le modèle ne verra que ceux-là) ---"
    env | grep '^CALYPSO_' | sort
    mod_ok
}

# Un enfant voit-il la configuration ? `bash -c` n'hérite que des variables
# EXPORTÉES : c'est exactement ce que verra QEMU.
_profil_enfant_voit() {
    local out
    out="$(bash -c 'printf "%s|%s" "${CALYPSO_MODE:-}" "${QEMU_BIN:-}"' 2>/dev/null)"
    case "$out" in
        ''|"|"*|*"|") return 1 ;;
    esac
    return 0
}

mod_profil_wait() {
    if ! wait_until "${MOD_TIMEOUT[profil]}" "configuration exportée" _profil_enfant_voit; then
        mod_hint "run.sh doit sourcer load.env sous « set -a » (run.sh:75-80) : sans export, QEMU démarre avec ses défauts"
        mod_fail "la configuration n'atteint pas les processus enfants"
        return $MOD_RC_FAIL
    fi
    mod_ok
}
