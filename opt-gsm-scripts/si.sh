#!/bin/bash
# si.sh — TRANSFORME le flux GSMTAP en SI decode (type + FN + hexa L2), live.
# Sniff PASSIF (gsm_sniff.py, raw socket) : ne bind aucun port, ne touche aucune
# fifo, ne perturbe pas le pipeline. Sources : 4729 (grgsm) + 4730 (feed_si).
# Usage : ./si.sh            (def 4729,4730)
#         SNIFF_PORTS=4730 ./si.sh
exec python3 -u /opt/GSM/gsm_sniff.py si
