#!/bin/bash
# e2e continu : re-decode le cfile continu en boucle, transmet les SI a feed_si.
source /root/.env/bin/activate 2>/dev/null
while true; do
  python3 /opt/GSM/si_bridge.py /tmp/relay_continu.cfile 2>&1 | grep -E "SI3|fini" | tail -2
  sleep 2
done
