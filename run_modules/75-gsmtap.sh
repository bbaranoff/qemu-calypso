# =============================================================================
#  75-gsmtap — capture pcap du trafic GSMTAP
# =============================================================================
#
#  RÔLE (run.sh.legacy L2155-2166)
#      tcpdump sur toutes les interfaces (couvre eth0 mobile/BTS et eth1), filtre
#      `udp port 4729`, écriture immédiate (`-U` : flush par paquet, le pcap est
#      exploitable sans attendre la fin). Pas de `--print -X` : cela déverserait
#      chaque paquet en hexadécimal dans le pane et rendrait la vue illisible.
#
#  PRÉREQUIS
#      tcpdump installé, et le droit de capturer (CAP_NET_RAW / root).
#
#  CRITÈRE DE SUCCÈS
#      Le fichier pcap existe et contient son en-tête (24 octets, écrits dès
#      l'ouverture). C'est le premier fait observable ; un tcpdump sans droits
#      meurt avant d'écrire quoi que ce soit.
#
#  JOURNAL
#      $LOG_DIR/gsmtap.log — la capture elle-même va dans $CAPTURE_DIR.
#
#  CE QUE CE MODULE NE FAIT PAS, ET POURQUOI
#      Le legacy installait tcpdump à la volée (L2162 : apt-get update && apt-get
#      install). Un script de lancement ne doit pas modifier la machine sous les
#      pieds de l'opérateur, encore moins depuis le réseau, et encore moins sans
#      le dire. Ici : diagnostic explicite et conduite à tenir.
# -----------------------------------------------------------------------------
MOD_REGISTER gsmtap "Capture GSMTAP (pcap)"
MOD_REQUIRED[gsmtap]=0
MOD_DEPS[gsmtap]="l2"
MOD_PROFILES[gsmtap]="calypso hybrid"
MOD_TIMEOUT[gsmtap]=15
MOD_ENABLED_IF[gsmtap]='[ "${CALYPSO_SKIP_GSMTAP:-0}" != 1 ]'

: "${CALYPSO_GSMTAP_PORT_CAPTURE:=4729}"
: "${CALYPSO_GSMTAP_IFACE:=any}"
: "${CAPTURE_DIR:=${RUN_DIR:-/tmp/calypso}/captures}"

_gsmtap_pat()  { printf '%s' "tcpdump .*udp port ${CALYPSO_GSMTAP_PORT_CAPTURE}"; }
_gsmtap_file() { cat "${RUN_DIR}/gsmtap.pcap.path" 2>/dev/null; }

mod_gsmtap_check() {
    command -v tcpdump >/dev/null 2>&1 || {
        mod_hint "installez-le vous-même : apt-get install tcpdump — ou lancez avec CALYPSO_SKIP_GSMTAP=1"
        mod_fail "tcpdump absent : aucune capture GSMTAP possible"
        return $MOD_RC_FAIL; }
    # LECTURE SEULE — on ne crée rien ici (c'est le rôle de _start) : on vérifie
    # que le répertoire de capture est inscriptible, ou à défaut qu'il pourra
    # être créé, c'est-à-dire que son parent l'est.
    local d="$CAPTURE_DIR"
    [ -d "$d" ] || d="$(dirname "$CAPTURE_DIR")"
    [ -w "$d" ] || {
        mod_hint "posez CAPTURE_DIR vers un répertoire inscriptible"
        mod_fail "répertoire de capture non inscriptible : $d"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gsmtap_status() { have_proc "$(_gsmtap_pat)"; }

mod_gsmtap_start() {
    local pcap log
    pcap="$CAPTURE_DIR/mobile-gsmtap-$(date +%Y%m%d_%H%M%S).pcap"
    log="${LOG_DIR}/gsmtap.log"
    mkdir -p "$LOG_DIR" "$RUN_DIR" 2>/dev/null
    printf '%s\n' "$pcap" > "${RUN_DIR}/gsmtap.pcap.path"
    mod_say "capture  : $pcap"
    mod_say "filtre   : udp port $CALYPSO_GSMTAP_PORT_CAPTURE sur $CALYPSO_GSMTAP_IFACE"

    # --- remplacement du `sleep 5` (L2165) -----------------------------------
    # Le legacy attendait 5 s « que les producteurs GSMTAP soient là ». Inutile :
    # tcpdump n'a pas besoin d'un producteur pour ouvrir sa capture, et rater les
    # cinq premières secondes de trafic est un défaut, pas une précaution. On
    # lance tout de suite ; la barrière porte sur le fichier réellement créé.
    tcpdump -i "$CALYPSO_GSMTAP_IFACE" -U \
            -w "$pcap" "udp port $CALYPSO_GSMTAP_PORT_CAPTURE" >>"$log" 2>&1 &
    printf '%s\n' "$!" > "${RUN_DIR}/gsmtap.pid"
    mod_ok
}

# BARRIÈRE — le pcap doit exister ET porter son en-tête (24 octets). tcpdump
# l'écrit à l'ouverture du fichier, avant tout paquet : c'est donc bien la
# preuve du démarrage de la capture, et non celle d'un trafic déjà présent.
mod_gsmtap_wait() {
    local pid pcap; pid="$(cat "${RUN_DIR}/gsmtap.pid" 2>/dev/null || echo 0)"
    pcap="$(_gsmtap_file)"

    wait_until "${MOD_TIMEOUT[gsmtap]}" "création du pcap" test -s "$pcap" || {
        mod_hint "lisez ${LOG_DIR}/gsmtap.log ; « permission denied » = capture sans droits (CAP_NET_RAW)"
        mod_fail "tcpdump n'a pas créé $pcap"
        return $MOD_RC_FAIL; }
    kill -0 "$pid" 2>/dev/null || {
        mod_fail "tcpdump a démarré puis s'est arrêté"
        mod_hint "lisez ${LOG_DIR}/gsmtap.log"
        return $MOD_RC_FAIL; }
    mod_ok
}

mod_gsmtap_stop() {
    local pid; pid="$(cat "${RUN_DIR}/gsmtap.pid" 2>/dev/null || echo 0)"
    # SIGTERM (pas -9) : tcpdump ferme proprement son pcap. Un -9 laisse un
    # fichier tronqué que wireshark refuse d'ouvrir.
    [ "$pid" != 0 ] && kill "$pid" 2>/dev/null
    pkill -f "$(_gsmtap_pat)" 2>/dev/null
    rm -f "${RUN_DIR}/gsmtap.pid"
    return 0
}
