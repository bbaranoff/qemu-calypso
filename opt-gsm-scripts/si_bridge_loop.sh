#!/bin/bash
# Decode le cfile continu en boucle, AFFICHE les SI + les transmet a feed_si.
source /root/.env/bin/activate 2>/dev/null
echo "[si-bridge] grgsm_decode -c /tmp/iq_grgsm.fifo -s 1083333 -> SI -> feed_si"
while true; do
  python3 /opt/GSM/si_bridge.py /tmp/iq_grgsm.fifo 2>&1 | grep -E "si-bridge|SI"
  sleep 2
done
