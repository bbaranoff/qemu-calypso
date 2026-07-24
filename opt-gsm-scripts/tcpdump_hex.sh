#!/bin/bash
# tcpdump_hex.sh — dump HEXA des trames GSMTAP decodees, SANS toucher de FIFO.
#
# L'ASTUCE : GSMTAP est de l'UDP. tcpdump ecoute le socket passivement -> aucune
# lecture de FIFO, aucune perturbation du relais/decode/camping (contrairement a
# relancer un grgsm_decode sur une fifo, qui vole l'I/Q et fige le pipeline).
#
# Sockets GSMTAP de ce pipeline (loopback) :
#   4729 = grgsm_decode -> GSMTAP decode (control/data, FN inclus)
#   4730 = feed_si (SI/L2 du si_bridge -> shunt -> mobile)
#   4731 = feed_sb (SCH : bsic/fn/toa)
#
# Usage :
#   ./tcpdump_hex.sh                 # 4729 + 4730 (defaut)
#   ./tcpdump_hex.sh 'port 4729'     # un seul port
#   IFACE=lo ./tcpdump_hex.sh        # interface explicite
#
# -X = hexa + ascii du paquet ; -l = ligne-bufferisee (live) ; -tttt = horodatage.
set -u
PORTS="${1:-port 4729 or port 4730}"
IFACE="${IFACE:-any}"
echo "[tcpdump-hex] iface=$IFACE filtre: udp and ( $PORTS )  (Ctrl-C pour quitter)"
exec tcpdump -i "$IFACE" -n -l -X -tttt "udp and ( $PORTS )"
