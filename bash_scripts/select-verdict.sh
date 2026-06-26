#!/bin/bash
# select-verdict.sh — Présente les verdicts pytest via whiptail pour choix
# utilisateur. Appelé par run-all.sh à la fin, ou en standalone.
#
# Lit /tmp/verdict_<mode>.json (set par test_mode_verdict.py via
# CALYPSO_TEST_OUT) et /tmp/run-all/results.json si présent.
#
# Présente :
#   1. Verdict global cross-mode (top par evidence weight)
#   2. Verdicts per-mode (chacun avec ses évidences)
#   3. Possibilité de choisir un next_step à exécuter (whiptail)
#
# Usage :
#   ./select-verdict.sh               # interactive whiptail
#   ./select-verdict.sh --print       # juste affichage texte
#
# Output : /tmp/selected_verdict.txt avec le verdict choisi + next_step.

set -uo pipefail

PRINT_MODE=0
[ "${1:-}" = "--print" ] && PRINT_MODE=1

VERDICT_DIR="${CALYPSO_TEST_OUT:-/tmp}"
CROSS_JSON="${RUN_ALL_JSON:-/tmp/run-all/results.json}"

# Récupère les fichiers verdict_*.json
JSONS=$(ls "$VERDICT_DIR"/verdict_*.json 2>/dev/null)
if [ -z "$JSONS" ]; then
    echo "[select-verdict] aucun fichier verdict_*.json dans $VERDICT_DIR"
    echo "[select-verdict] Run d'abord ./run.sh ou ./run-all.sh"
    exit 1
fi

# Bash function : extraire un champ d'un verdict JSON via python
_extract_verdicts() {
    local f="$1"
    python3 -c "
import json, sys
with open('$f') as fh: d = json.load(fh)
mode = d.get('mode', '?')
for v in d.get('verdicts', []):
    print(f\"{mode}|{v['weight']}|{v['name']}|{v['conclusion']}|{' / '.join(v['next_steps'])}\")
"
}

# Collecte tous les verdicts
ALL_VERDICTS=()
for json_f in $JSONS; do
    while IFS= read -r line; do
        [ -n "$line" ] && ALL_VERDICTS+=("$line")
    done < <(_extract_verdicts "$json_f")
done

if [ ${#ALL_VERDICTS[@]} -eq 0 ]; then
    echo "[select-verdict] verdicts vides (aucun ne matche les évidences)"
    exit 0
fi

# Mode print : juste affichage texte
if [ $PRINT_MODE -eq 1 ]; then
    echo "========== ALL VERDICTS =========="
    for v in "${ALL_VERDICTS[@]}"; do
        mode="${v%%|*}"; rest="${v#*|}"
        weight="${rest%%|*}"; rest="${rest#*|}"
        name="${rest%%|*}"; rest="${rest#*|}"
        conclusion="${rest%%|*}"; next_steps="${rest#*|}"
        echo
        echo "  [${mode}] ${name} (weight ${weight})"
        echo "  → ${conclusion}"
        echo "    next_steps : ${next_steps}"
    done
    echo
    echo "=================================="
    exit 0
fi

# Mode interactif whiptail
if ! command -v whiptail >/dev/null 2>&1; then
    echo "[select-verdict] whiptail absent → fallback --print"
    exec "$0" --print
fi

export NEWT_COLORS='root=,blue title=blue,lightgray'
BACKTITLE="Calypso — Verdict Selector"

# Build radiolist whiptail
ARGS=()
i=0
for v in "${ALL_VERDICTS[@]}"; do
    mode="${v%%|*}"; rest="${v#*|}"
    weight="${rest%%|*}"; rest="${rest#*|}"
    name="${rest%%|*}"; rest="${rest#*|}"
    conclusion="${rest%%|*}"
    # Tag = index, label = "mode/name (weight N)"
    label="${mode}/${name} [w=${weight}]"
    desc="${conclusion:0:60}"
    ARGS+=("$i" "$label - $desc" $([ $i -eq 0 ] && echo ON || echo OFF))
    i=$((i + 1))
done

CHOICE=$(whiptail --backtitle "$BACKTITLE" \
    --title "Sélection verdict" \
    --notags --radiolist \
    "\n Choisir le verdict à examiner / exécuter le next_step.\n\n Les verdicts sont rankés par weight (= nombre d'évidences matchées).\n" \
    22 100 ${#ALL_VERDICTS[@]} \
    "${ARGS[@]}" \
    3>&1 1>&2 2>&3) || { echo "[select-verdict] cancelled"; exit 0; }

SEL="${ALL_VERDICTS[$CHOICE]}"
mode="${SEL%%|*}"; rest="${SEL#*|}"
weight="${rest%%|*}"; rest="${rest#*|}"
name="${rest%%|*}"; rest="${rest#*|}"
conclusion="${rest%%|*}"; next_steps="${rest#*|}"

DETAILS="
 Mode        : $mode
 Verdict     : $name
 Weight      : $weight

 Conclusion  :
 $conclusion

 Next steps  :
 $(echo "$next_steps" | tr '/' '\n' | sed 's/^/   /')
"
whiptail --backtitle "$BACKTITLE" \
    --title "$name" \
    --msgbox "$DETAILS" 22 84 3>&1 1>&2 2>&3

# Persist
OUT="/tmp/selected_verdict.txt"
{
    echo "# Selected verdict $(date -Iseconds)"
    echo "mode=$mode"
    echo "name=$name"
    echo "weight=$weight"
    echo "conclusion=$conclusion"
    echo "next_steps=$next_steps"
} > "$OUT"

echo "[select-verdict] saved → $OUT"
cat "$OUT"
