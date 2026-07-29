# =============================================================================
#  11-reseau — interfaces et plan d'adressage local
# =============================================================================
#
#  RÔLE
#      Poser les conditions réseau du run mono-opérateur, et surtout constater
#      qu'elles sont réunies AVANT de lancer quoi que ce soit :
#        - la boucle locale porte bien tout 127.0.0.0/8 (la pile écrit en dur sur
#          127.0.0.2 pour le HLR et le contrôle du BTS : ce n'est pas un alias à
#          créer sous Linux, mais il faut le vérifier, pas le supposer) ;
#        - les ports que les modules suivants vont OUVRIR sont libres. Ce module
#          passe APRÈS 10-teardown : un port encore tenu ici signifie que le
#          balayage n'a pas suffi, et le tenant est nommé tout de suite — au lieu
#          de faire échouer QEMU dix secondes plus tard sur une écoute
#          impossible. On ne tue rien ici : le meurtre appartient à 10-teardown,
#          le constat à ce module ;
#        - /dev/net/tun existe si un module en aval en a besoin — c'est le cas de
#          15-apn0, qui crée l'interface TUN de l'APN paquet et échoue sans lui.
#          En conteneur non privilégié, sa création est impossible : ici c'est
#          signalé sans bloquer (le profil calypso n'a pas de trafic paquet), et
#          fatal seulement si NEED_TUN=1. Le module qui en a VRAIMENT besoin
#          échoue de son côté, avec son propre message.
#
#  PRÉREQUIS
#      rundir (RUN_DIR utilisable) ; commande `ip` (02-outils). Joué après
#      10-teardown, dont il constate le résultat sur les ports.
#
#  CRITÈRE DE SUCCÈS
#      BARRIÈRE — `lo` est UP, une route vers 127.0.0.2 est résolue, et chacun
#      des ports de RESEAU_PORTS_LIBRES est effectivement fermé (sonde TCP réelle,
#      pas une lecture de table). Si NEED_TUN=1, /dev/net/tun est un périphérique
#      caractère.
#
#  JOURNAL
#      $LOG_DIR/mod/reseau.log : état de lo, ports sondés, tenant éventuel.
# -----------------------------------------------------------------------------
MOD_REGISTER reseau "Interfaces et plan d'adressage"
MOD_REQUIRED[reseau]=1
MOD_DEPS[reseau]="rundir"
MOD_PROFILES[reseau]="calypso faketrx hybrid core"
MOD_TIMEOUT[reseau]=15

# Ports que le plan va ouvrir. 1234 = serveur gdb de 40-qemu (codé en dur dans ce
# module-là : cette liste ne fait que le CONSTATER, elle ne le configure pas).
: "${RESEAU_PORTS_LIBRES:=${PORT_GDB:-1234}}"
# Le cœur paquet (GGSN/SGSN, interface apn0) a besoin de TUN ; le profil calypso
# seul, non. Défaut : on signale sans bloquer.
: "${NEED_TUN:=0}"

_net_tenant() {   # $1 = port ; décrit qui l'occupe, au mieux de l'outillage présent
    local port="$1" out=""
    if command -v ss >/dev/null 2>&1; then
        out="$(ss -lntp "sport = :$port" 2>/dev/null | awk 'NR>1{print $0}')"
    fi
    [ -n "$out" ] && printf '%s' "$out" || printf 'tenant inconnu (ss absent ou sans droits)'
}

_net_lo_up() { ip link show lo 2>/dev/null | grep -q 'state UNKNOWN\|state UP\|<.*UP'; }
_net_route_loopback() { ip route get 127.0.0.2 >/dev/null 2>&1; }
# Sonde d'occupation. have_port (mod.sh) ouvre un /dev/tcp : il ne voit ni l'UDP
# ni un socket en écoute qui refuse la connexion. On préfère donc ss quand il est
# là, et on retombe sur have_port sinon — en le disant, plutôt qu'en rendant un
# « libre » qui n'engage à rien.
_net_port_occupe() {
    if command -v ss >/dev/null 2>&1; then
        ss -ln 2>/dev/null | grep -qE "[:.]${1}[[:space:]]" && return 0
        return 1
    fi
    have_port "$1"
}
_net_ports_libres() {
    local p
    for p in $RESEAU_PORTS_LIBRES; do
        _net_port_occupe "$p" && return 1
    done
    return 0
}
_net_tun_ok() { [ -c /dev/net/tun ]; }

mod_reseau_check() {
    command -v ip >/dev/null 2>&1 || {
        mod_hint "apt-get install -y iproute2"
        mod_fail "commande « ip » absente : le réseau ne peut pas être constaté"
        return $MOD_RC_FAIL; }

    local p occupes=""
    for p in $RESEAU_PORTS_LIBRES; do
        _net_port_occupe "$p" && occupes="$occupes $p"
    done
    if [ -n "$occupes" ]; then
        for p in $occupes; do mod_say "port $p occupé : $(_net_tenant "$p")"; done
        mod_hint "arrêtez le run précédent :  ./run.sh --stop   (10-teardown balaie au démarrage, sauf si NO_STARTUP_STOP=1)"
        mod_fail "ports déjà occupés :$occupes — le plan ne pourra pas les ouvrir"
        return $MOD_RC_FAIL
    fi
    mod_ok
}

# « Déjà démarré » = les conditions sont déjà réunies. Ce module ne laisse
# derrière lui qu'un état constaté, jamais un processus.
mod_reseau_status() {
    _net_lo_up || return $MOD_RC_FAIL
    _net_route_loopback || return $MOD_RC_FAIL
    if [ "${NEED_TUN}" = 1 ]; then _net_tun_ok || return $MOD_RC_FAIL; fi
    return $MOD_RC_OK
}

mod_reseau_start() {
    _net_lo_up || ip link set lo up 2>/dev/null

    if ! _net_tun_ok; then
        if [ "$(id -u)" != 0 ]; then
            mod_say "/dev/net/tun absent et nous ne sommes pas root : création impossible"
        else
            modprobe tun 2>/dev/null
            if ! _net_tun_ok; then
                mkdir -p /dev/net 2>/dev/null
                mknod /dev/net/tun c 10 200 2>/dev/null && chmod 0666 /dev/net/tun 2>/dev/null
            fi
        fi
        if _net_tun_ok; then
            mod_say "/dev/net/tun : créé"
        elif [ "${NEED_TUN}" = 1 ]; then
            mod_hint "relancez le conteneur avec --device /dev/net/tun (ou --privileged), ou posez NEED_TUN=0"
            mod_fail "/dev/net/tun indisponible alors que NEED_TUN=1"
            return $MOD_RC_FAIL
        else
            mod_say "/dev/net/tun indisponible — sans effet sur le profil courant (NEED_TUN=0)"
        fi
    fi

    mod_say "lo        : $(ip -o -4 addr show dev lo 2>/dev/null | awk '{print $4}' | tr '\n' ' ')"
    mod_say "ports     : $RESEAU_PORTS_LIBRES (attendus libres)"
    mod_ok
}

mod_reseau_wait() {
    wait_until "${MOD_TIMEOUT[reseau]}" "interface lo active" _net_lo_up || {
        mod_hint "ip link set lo up"
        mod_fail "la boucle locale n'est pas active"
        return $MOD_RC_FAIL; }

    wait_until "${MOD_TIMEOUT[reseau]}" "route vers 127.0.0.2" _net_route_loopback || {
        mod_hint "la pile écrit en dur sur 127.0.0.2 (HLR, contrôle BTS) : vérifiez  ip route get 127.0.0.2"
        mod_fail "127.0.0.2 n'est pas routable"
        return $MOD_RC_FAIL; }

    wait_until "${MOD_TIMEOUT[reseau]}" "libération des ports" _net_ports_libres || {
        mod_hint "un processus tient encore un port :  ss -lntp | grep -E '$(printf '%s' "$RESEAU_PORTS_LIBRES" | tr ' ' '|')'"
        mod_fail "ports toujours occupés parmi : $RESEAU_PORTS_LIBRES"
        return $MOD_RC_FAIL; }

    if [ "${NEED_TUN}" = 1 ]; then
        wait_until "${MOD_TIMEOUT[reseau]}" "périphérique TUN" _net_tun_ok || {
            mod_hint "--device /dev/net/tun au lancement du conteneur"
            mod_fail "/dev/net/tun absent (NEED_TUN=1)"
            return $MOD_RC_FAIL; }
    fi
    mod_ok
}

# On ne défait pas ce qu'on n'a pas fait : lo reste UP, et /dev/net/tun est un
# périphérique partagé avec le reste de la machine.
mod_reseau_stop() { return 0; }
