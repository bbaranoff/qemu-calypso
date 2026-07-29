# =============================================================================
#  05-config — points de rendez-vous : fichiers de configuration et ports
# =============================================================================
#
#  RÔLE
#    environnement/paths.env dit où sont les BINAIRES (QEMU, firmware, ROM du
#    DSP, osmocon). Il ne dit rien des FICHIERS DE CONFIGURATION consommés par
#    les services (osmo-bts-trx, osmo-trx-ipc, mobile), ni des PORTS sur
#    lesquels les blocs se rejoignent, ni des SOCKETS de rendez-vous. Ce module
#    comble ce trou, et rien d'autre : il ne lance rien, ne crée aucun fichier.
#    Reprend run.sh.legacy #8 (L1027-1032), #11 (L1057-1069) et la part « noms »
#    de #17 (L1426-1446).
#
#  PRÉREQUIS  : 00-prereqs (les binaires existent).
#  SUCCÈS     : toutes les variables ci-dessous sont posées ; les fichiers de
#               configuration résolus sont tracés dans le journal, lisibles ou
#               non.
#  JOURNAL    : $LOG_DIR/mod/config.log
#
#  ------------------------------------------------------------------ POURQUOI
#
#  1. POURQUOI CE MODULE N'ÉCHOUE PAS SUR UN .cfg ABSENT.
#     Il est obligatoire : le faire échouer arrêterait toute la séquence. Or un
#     osmo-trx-ipc.cfg manquant ne gêne QUE le module 56 — c'est à lui de le
#     dire, avec le message qui correspond à son composant. 05 résout et trace ;
#     il n'échoue que si une variable ne peut être résolue DU TOUT (chemin vide),
#     ce qui est un défaut de l'installation, pas d'un composant.
#
#  2. POURQUOI LES PORTS SONT DUPLIQUÉS SOUS DES NOMS SANS PRÉFIXE CALYPSO_.
#     environnement/ suit une convention stricte : `CALYPSO_X=""` signifie
#     « laisser le défaut codé en dur dans le modèle C » (bsp.env L53-54 :
#     « defaut : code BSP_TRXD_PORT=6702 » ; shunt.env L20-21 : « defaut : code
#     4730 »). Poser CALYPSO_BSP_PORT=6702 depuis un module casserait cette
#     convention et masquerait la valeur réelle du C. Mais les barrières des
#     blocs suivants doivent SONDER un numéro concret. D'où deux jeux distincts :
#     les CALYPSO_* restent intouchées, et les PORT_* donnent aux sondes la
#     valeur effective (la surcharge si elle existe, sinon le défaut documenté).
#
#  3. RÈGLE TRANSVERSE AU BLOC A — NE POSER AUCUNE VARIABLE LUE PAR LE MODÈLE C.
#     Le chemin de référence
#         CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 \
#         CALYPSO_SHUNT_REAL_FB=1 ./start-clean.sh
#     passe par ce moteur (start-clean.sh fait `exec ./run.sh`). Toute variable
#     que le modèle C lit par getenv() et qu'un module poserait changerait
#     l'émulation de ce chemin. Vérifié à la source : CALYPSO_SKIP_*,
#     CALYPSO_MODE et CALYPSO_PIPELINE ne sont lus NULLE PART dans hw/ —
#     ce sont des variables de moteur, libres à poser. CALYPSO_DSP_SHUNT, lui,
#     EST lu (calypso_dsp_shunt.c:2034 et :2266) : aucun module du bloc A ne le
#     pose. Voir 06-pipeline pour la conséquence.
#
#  4. POURQUOI L1CTL_SOCK N'EST PAS EXPORTÉE.
#     Le legacy lançait QEMU avec `L1CTL_SOCK=$QEMU_DUMMY_SOCK` en préfixe
#     (L1820-1821) : QEMU/l1ctl_sock.c crée SON socket à l'adresse indiquée, et
#     on l'envoyait volontairement dans une poubelle pour qu'il ne squatte pas
#     /tmp/osmocom_l2 — le vrai socket L1CTL, créé plus tard par osmocon
#     (L1930, -s). Exporter L1CTL_SOCK=/tmp/osmocom_l2 depuis ici ferait
#     exactement ce que le legacy évitait. La variable est donc posée sous le
#     nom L1CTL_SOCK_PATH, jamais exportée, à usage des sondes seulement.
# -----------------------------------------------------------------------------

MOD_REGISTER config "Résolution des chemins et des ports"
MOD_REQUIRED[config]=1
MOD_DEPS[config]="prereqs"
MOD_PROFILES[config]="calypso faketrx hybrid core"
MOD_PURE[config]=1
MOD_TIMEOUT[config]=5

# --- helper partagé par tout le bloc A ---------------------------------------
# Réplique de run.sh.legacy #32 (L1812-1818) : marque `export` toute CALYPSO_*
# visible, SANS la réaffecter, pour qu'elle atteigne l'environnement du child
# QEMU. Les modules sont sourcés après le `set +a` de run.sh, donc une variable
# posée par un module n'est PAS exportée d'office : ce balayage est ce qui rend
# la chaîne de configuration effective jusqu'à QEMU. Chaque module du bloc A qui
# pose une CALYPSO_* rappelle cette fonction en fin de start.
calypso_export_sweep() {
    local _v _n=0
    while IFS= read -r _v; do
        [ -n "$_v" ] || continue
        export "$_v" 2>/dev/null && _n=$((_n + 1))
    done < <(compgen -v | grep '^CALYPSO_')
    mod_say "env-propagation : $_n CALYPSO_* exportées vers QEMU"
}

mod_config_check() {
    # Lecture seule stricte : on ne vérifie que ce dont l'ABSENCE rendrait tout
    # le reste faux — l'arborescence du dépôt elle-même. QEMU_TREE est auto-
    # détecté par paths.env ; s'il pointe ailleurs, tous les chemins dérivés
    # (cfgs/, scripts/, tools/) sont faux et aucun module suivant ne le dirait.
    if [ ! -d "${QEMU_TREE:-}/run_modules" ]; then
        mod_hint "posez QEMU_TREE sur la racine du dépôt, ou lancez ./run.sh depuis cette racine"
        mod_fail "QEMU_TREE ne désigne pas ce dépôt : ${QEMU_TREE:-<non défini>}"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

mod_config_status() { return $MOD_RC_FAIL; }   # module pur : jamais « déjà démarré »

mod_config_start() {
    # --- fichiers de configuration des services (legacy #8, #11) -------------
    # Chaque entrée : d'abord la version du dépôt (versionnée, donc reproductible),
    # sinon celle du système. Le legacy faisait l'inverse pour osmo-trx-ipc.cfg et
    # se plantait : /etc/osmocom déclarait 2 canaux, calypso n'en a qu'un
    # (« DDEV ERROR chan num mismatch », L1058-1060). La version du dépôt gagne.
    : "${BTS_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-bts-trx.cfg}"

    : "${MOBILE_CFG_SRC:=${QEMU_CFGS:-$QEMU_TREE/cfgs}/mobile_group1.cfg}"
    : "${MOBILE_CFG:=${OSMOCOM_HOME:-$HOME/.osmocom}/bb/mobile_group1.cfg}"

    : "${OSMO_TRX_IPC_BIN:=osmo-trx-ipc}"
    if [ -r "${QEMU_CFGS:-$QEMU_TREE/cfgs}/osmo-trx-ipc.cfg" ]; then
        : "${OSMO_TRX_IPC_CFG:=${QEMU_CFGS:-$QEMU_TREE/cfgs}/osmo-trx-ipc.cfg}"
    else
        : "${OSMO_TRX_IPC_CFG:=${OSMOCOM_CFG:-/etc/osmocom}/osmo-trx-ipc.cfg}"
    fi

    : "${IPC_DEVICE_BIN:=$QEMU_TREE/tools/calypso-ipc-device/calypso-ipc-device}"
    : "${IPC_MSOCK_PATH:=/tmp/ipc_sock0}"

    # --- sockets de rendez-vous (legacy #17, L1444-1446) ---------------------
    # QEMU_MON_SOCK suit 40-qemu.sh (${RUN_DIR}/qemu-monitor.sock) et NON le
    # /tmp/qemu-calypso-mon.sock du legacy : c'est 40-qemu qui crée le socket,
    # donc c'est lui qui fait autorité sur son emplacement.
    : "${QEMU_MON_SOCK:=${RUN_DIR:-/tmp/calypso}/qemu-monitor.sock}"
    : "${L1CTL_SOCK_PATH:=/tmp/osmocom_l2}"     # jamais exportée — cf. POURQUOI 4
    : "${QEMU_DUMMY_SOCK:=/tmp/qemu_l1ctl_disabled}"

    # --- session tmux --------------------------------------------------------
    # run.sh (épilogue, L252) lit ${TMUX_SESSION:-calypso} mais personne ne la
    # pose : on la pose ici pour que l'épilogue, le module 35 et le module 99
    # parlent tous de la même session.
    : "${TMUX_SESSION:=calypso}"

    # --- ports (cf. POURQUOI 2) ----------------------------------------------
    : "${PORT_TRXD_CLOCK:=5700}"                                # osmo-bts-trx <-> transceiver
    : "${PORT_TRXD_CTRL:=5701}"
    : "${PORT_TRXD_DATA:=5702}"
    : "${PORT_BSP_UDP:=${CALYPSO_BSP_PORT:-6702}}"              # bsp.c:58,913
    : "${PORT_IQ_TEE:=${CALYPSO_IQ_TEE_PORT:-6703}}"            # bsp.c:405-406
    : "${PORT_IQ_RELAY_RX:=${CALYPSO_TRX_IQ_RX_PORT:-5810}}"    # legacy L1950
    : "${PORT_IQ_RELAY_TX:=${CALYPSO_TRX_IQ_TX_PORT:-5811}}"    # legacy L1950
    : "${PORT_GSMTAP:=4729}"                                    # capture mobile (legacy L2165)
    : "${PORT_GSMTAP_SI:=${CALYPSO_SHUNT_GSMTAP_PORT:-4730}}"   # SI gr-gsm -> feed_si
    : "${PORT_GSMTAP_SCH:=${CALYPSO_SHUNT_SCH_PORT:-4731}}"     # SCH/BSIC réel
    : "${PORT_HLR_VTY:=4258}"                                   # legacy L1578
    : "${PORT_GDB:=${CALYPSO_GDB_PORT:-1234}}"                  # legacy L1601

    mod_say "BTS_CFG          = $BTS_CFG"
    mod_say "MOBILE_CFG_SRC   = $MOBILE_CFG_SRC"
    mod_say "MOBILE_CFG       = $MOBILE_CFG"
    mod_say "OSMO_TRX_IPC_CFG = $OSMO_TRX_IPC_CFG"
    mod_say "IPC_DEVICE_BIN   = $IPC_DEVICE_BIN"
    mod_say "IPC_MSOCK_PATH   = $IPC_MSOCK_PATH"
    mod_say "QEMU_MON_SOCK    = $QEMU_MON_SOCK"
    mod_say "L1CTL_SOCK_PATH  = $L1CTL_SOCK_PATH (non exportée)"
    mod_say "ports            : TRXD $PORT_TRXD_CLOCK-$PORT_TRXD_DATA · BSP $PORT_BSP_UDP · tee $PORT_IQ_TEE"
    mod_say "                   relais I/Q $PORT_IQ_RELAY_RX/$PORT_IQ_RELAY_TX · GSMTAP $PORT_GSMTAP/$PORT_GSMTAP_SI/$PORT_GSMTAP_SCH"
    mod_say "                   HLR VTY $PORT_HLR_VTY · gdb $PORT_GDB"

    # Trace de lisibilité — informative, JAMAIS bloquante (cf. POURQUOI 1).
    local f
    for f in "$BTS_CFG" "$MOBILE_CFG_SRC" "$OSMO_TRX_IPC_CFG" "$IPC_DEVICE_BIN"; do
        [ -r "$f" ] && mod_say "lisible  : $f" || mod_say "ABSENT   : $f (le module qui l'utilise le signalera)"
    done

    calypso_export_sweep
    mod_ok
}

# BARRIÈRE — critère observable, sans aucune attente : le module est prêt quand
# TOUTES les variables de rendez-vous ont une valeur non vide. Un chemin vide se
# propage silencieusement en `-c ''` ou en socket `/` chez le consommateur : on
# le prend ici, pas trois modules plus loin.
_config_vars() {
    printf '%s\n' BTS_CFG MOBILE_CFG MOBILE_CFG_SRC OSMO_TRX_IPC_CFG IPC_MSOCK_PATH \
                  QEMU_MON_SOCK L1CTL_SOCK_PATH TMUX_SESSION \
                  PORT_TRXD_CLOCK PORT_BSP_UDP PORT_IQ_RELAY_RX PORT_GSMTAP PORT_HLR_VTY
}
_config_missing() {
    local v out=""
    while read -r v; do [ -n "${!v:-}" ] || out="$out $v"; done < <(_config_vars)
    printf '%s' "${out# }"
}
_config_all_set() { [ -z "$(_config_missing)" ]; }

mod_config_wait() {
    if ! wait_until "${MOD_TIMEOUT[config]}" "variables de configuration" _config_all_set; then
        mod_hint "vérifiez environnement/paths.env (GSM_ROOT, QEMU_TREE, OSMOCOM_CFG, OSMOCOM_HOME)"
        mod_fail "variables de configuration vides : $(_config_missing)"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# Module pur : rien à arrêter. Présent pour que `./run.sh --stop` ne saute pas
# une ligne dans son affichage.
mod_config_stop() { return 0; }
