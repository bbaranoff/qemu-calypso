#!/bin/bash
# burst.sh — TRANSFORME les sockets burst en burst decode, live.
# Sniff PASSIF (gsm_sniff.py, raw socket) : ne bind aucun port, aucune fifo.
# Sources : 4731 (SCH : bsic/fn/toa) + 5700-5702 (TRXD : ts/fn/148 soft-bits).
# Usage : ./burst.sh                 (def 4731,5700,5701,5702)
#         SNIFF_PORTS=4731 ./burst.sh
exec python3 -u /opt/GSM/gsm_sniff.py burst
