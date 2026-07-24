#!/bin/bash
# tcpdump_burst.sh — dump des BURSTS bruts sur les sockets UDP, SANS FIFO.
#
# L'ASTUCE : comme pour l'hexa, on tape des sockets UDP en passif -> zero fifo,
# zero perturbation.
#
# Sources de bursts (loopback) :
#   5700-5702 = TRXD osmo-bts-trx <-> osmo-trx-ipc : bursts radio bruts
#               (TRXDv0 : 8 o d'en-tete + 148 soft-bits ±127). DL + UL.
#   4731      = feed_sb : metadata burst SCH (magic 'SCH1' + bsic/fn/toa, LE).
#
# Usage :
#   ./tcpdump_burst.sh                          # TRXD 5700-5702 + SCH 4731 (defaut)
#   ./tcpdump_burst.sh 'portrange 5700-5702'    # TRXD seul (vrais bursts air)
#   ./tcpdump_burst.sh 'port 4731'              # SCH seul
#
# -X = hexa+ascii (on voit les 148 soft-bits / la struct SCH) ; -l live ; -tttt ts.
set -u
PORTS="${1:-portrange 5700-5702 or port 4731}"
IFACE="${IFACE:-any}"
echo "[tcpdump-burst] iface=$IFACE filtre: udp and ( $PORTS )  (Ctrl-C pour quitter)"
exec tcpdump -i "$IFACE" -n -l -X -tttt "udp and ( $PORTS )"
