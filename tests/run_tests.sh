#!/bin/bash
# Runner tests Calypso — pile VIVANTE (lit /root/qemu.log, /root/osmocon.log…).
# COMPLET : tout tests/*.py (calypso + génériques top-level), on IGNORE seulement
# functional/ + guest-debug/ (suite d'intégration QEMU : binaires/downloads absents).
# --continue-on-collection-errors = ne bloque jamais.
# Sortie EN DIRECT (pytest -v + tee, line-buffered) — chaque test s'affiche au fil.
# Génère grafcet.html/md + rapport_final.pdf via hook conftest::pytest_sessionfinish.
#
# Usage : run_tests.sh [loop]
set -u
PY="${CALYPSO_PY:-/root/.env/bin/python3}"
TESTS=${QEMU_TREE}/tests
OUT="${CALYPSO_TEST_OUT:-/root/test_reports}"
PERIOD="${CALYPSO_TEST_PERIOD:-90}"
mkdir -p "$OUT"

run_once() {
  echo "==================== pytest COMPLET (pile vivante) $(date +%H:%M:%S) ===================="
  ( cd "$TESTS" && CALYPSO_TEST_OUT="$OUT" \
      stdbuf -oL -eL "$PY" -m pytest -v -ra --color=yes \
        -p no:cacheprovider --continue-on-collection-errors \
        --ignore=functional --ignore=guest-debug --ignore=migration \
        --ignore=qemu-iotests 2>&1 | stdbuf -oL tee "$OUT/pytest_last.log" )
  echo "-> $OUT/{grafcet.html, grafcet.md, rapport_final.pdf}  (log: $OUT/pytest_last.log)"
}

if [ "${1:-}" = "loop" ]; then
  echo "[run_tests] LOOP mode, période=${PERIOD}s (Ctrl-C pour stop) — sortie en direct"
  while true; do run_once; echo "[run_tests] prochain run dans ${PERIOD}s…"; sleep "$PERIOD"; done
else
  run_once
fi
