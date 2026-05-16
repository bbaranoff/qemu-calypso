#!/usr/bin/env bash
# make_diag.sh — snapshot rapide texte/markdown du run actif, conçu pour être
# appelé pendant pytest (par tests/conftest.py) et inséré en annexe du
# rapport. Ne produit PAS de tarball — pour ça utiliser make_diag_bundle.sh.
#
# Sortie : markdown sur stdout. Sections courtes, grep-friendly, low-noise.
#
# Usage:
#   ./make_diag.sh                     # défaut : container "trying"
#   CONTAINER=foo ./make_diag.sh
#   TAIL_LINES=20 ./make_diag.sh        # nombre de lignes en tail (défaut 10)

set -u

CONTAINER="${CONTAINER:-trying}"
TAIL_LINES="${TAIL_LINES:-10}"

_dexec() { docker exec "$CONTAINER" "$@" 2>/dev/null; }

if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
    echo "_diag: container \`$CONTAINER\` n'est pas running — snapshot impossible._"
    exit 0
fi

echo
echo "### Processes vivants"
echo
echo "\`\`\`"
_dexec ps -o pid,etime,cmd -C qemu-system-arm,osmocon,python3,mobile,osmo-bts-trx 2>/dev/null \
    | awk 'NR==1 || /qemu-system-arm|osmocon|bridge.py|irda_capture|mobile|bts-trx/' | head -20
echo "\`\`\`"
echo

echo "### Tailles des logs"
echo
echo "| Log | Bytes | Lines |"
echo "|---|---:|---:|"
for f in /root/qemu.log /tmp/bridge.log /tmp/osmocon.log /tmp/mobile.log \
         /tmp/bts.log /tmp/fw-irda.log /tmp/qemu-irda-tx.raw /tmp/qemu-modem-tx.raw \
         /tmp/irda_capture.stderr.log /tmp/frame_irq.log /tmp/tdma_tick.log; do
    sz=$(_dexec stat -c%s "$f" 2>/dev/null)
    ln=$(_dexec bash -c "wc -l < $f 2>/dev/null")
    if [ -n "$sz" ]; then
        printf "| \`%s\` | %s | %s |\n" "$f" "$sz" "${ln:-0}"
    else
        printf "| \`%s\` | _absent_ | — |\n" "$f"
    fi
done
echo

echo "### Grep invariants clés"
echo
echo "| Pattern | Source | Count |"
echo "|---|---|---:|"
for spec in \
    "BSP LOAD|/root/qemu.log" \
    "BSP DMA|/root/qemu.log" \
    "fbsb hook|/root/qemu.log" \
    "FB-det|/root/qemu.log" \
    "L1CTL_RESET|/tmp/osmocon.log" \
    "L1CTL_FBSB_REQ|/tmp/osmocon.log" \
    "L1CTL_PM_REQ|/tmp/osmocon.log" \
    "LOST|/tmp/osmocon.log" \
    "bridge: DL #|/tmp/bridge.log" \
    "fw-beacon|/tmp/fw-irda.log" \
    "fw-irda boot OK|/tmp/fw-irda.log"; do
    pat="${spec%%|*}"
    src="${spec##*|}"
    cnt=$(_dexec bash -c "grep -c \"$pat\" \"$src\" 2>/dev/null; true")
    printf "| \`%s\` | \`%s\` | %s |\n" "$pat" "$src" "${cnt:-0}"
done
echo

echo "### Blockers (counts d'erreurs critiques)"
echo
echo "| Pattern | Count |"
echo "|---|---:|"
for pat in "panic" "abort" "SP-CATASTROPHE" "ASSERT" "Out of memory" \
           "Connection refused" "BTS shutdown" "clock skew" "Layer2 socket"; do
    cnt=$(_dexec bash -c "grep -irc \"$pat\" /root/qemu.log /tmp/*.log 2>/dev/null | awk -F: '{s+=\$2} END {print s+0}'")
    printf "| \`%s\` | %s |\n" "$pat" "${cnt:-0}"
done
echo

echo "### Tail des logs (${TAIL_LINES} dernières lignes)"
echo
for f in /tmp/fw-irda.log /tmp/bridge.log /tmp/osmocon.log /tmp/mobile.log /tmp/bts.log; do
    echo "<details><summary>\`$f\`</summary>"
    echo
    echo "\`\`\`"
    _dexec tail -"$TAIL_LINES" "$f" 2>/dev/null || echo "(absent)"
    echo "\`\`\`"
    echo
    echo "</details>"
    echo
done
