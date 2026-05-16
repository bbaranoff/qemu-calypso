#!/bin/bash
# bisect_dsp_bottleneck.sh — répond à la question Claude web :
#   "Si CALYPSO_FBSB_SYNTH=1 court-circuite le DSP : a-t-on A_CD-WR >= 15 et
#    DATA_IND > 0 ?"
# Si oui → le DSP est le bottleneck, optimiser c54x emulator.
# Si non → bug indépendant côté chaîne ARM → l1ctl, à isoler avant.
#
# Usage : ./bisect_dsp_bottleneck.sh [WINDOW_S=60]
# Exécute deux runs courts (60 s chacun par défaut) :
#   1. synth=0 (path DSP réel, état actuel)
#   2. synth=1 (court-circuit DSP via calypso_fbsb)
# Et compare A_CD-WR + DATA_IND count.

set -u
WINDOW="${1:-60}"
CONTAINER="${CALYPSO_CONTAINER:-trying}"

QEMU_LOG=/root/qemu.log

measure() {
    local label="$1"
    local synth="$2"
    echo "=================================================="
    echo "RUN [$label]  CALYPSO_FBSB_SYNTH=$synth  window=${WINDOW}s"
    echo "=================================================="
    # Stop éventuel run en cours
    docker exec "$CONTAINER" bash -c "pkill -9 -f qemu-system-arm 2>/dev/null; pkill -9 -f bridge.py 2>/dev/null; pkill -9 -f osmocon 2>/dev/null; pkill -9 -f 'mobile.*group1' 2>/dev/null; sleep 1"

    # Reset le log avant le run
    docker exec "$CONTAINER" bash -c "rm -f $QEMU_LOG"

    # Lance run.sh en arrière-plan avec env
    docker exec -d "$CONTAINER" bash -c "
        export CALYPSO_FBSB_SYNTH=$synth
        export CALYPSO_L2_CLIENT=mobile
        cd /opt/GSM/qemu-src && ./run.sh > /tmp/run_$label.log 2>&1
    "

    # Attendre $WINDOW + grace startup (20s)
    echo "Waiting ${WINDOW}s + 20s startup..."
    sleep $((WINDOW + 20))

    # Collecter métriques
    local a_cd a_cd_uniq task24 data_ind d_fb_det fb_det_nonzero
    a_cd=$(docker exec "$CONTAINER" bash -c "grep -c 'A_CD-WR' $QEMU_LOG 2>/dev/null")
    a_cd_uniq=$(docker exec "$CONTAINER" bash -c "grep -oE 'A_CD-WR[^|]*addr=0x[0-9a-f]+' $QEMU_LOG 2>/dev/null | sort -u | wc -l")
    task24=$(docker exec "$CONTAINER" bash -c "grep -c 'task=24\|TASK_MD=24\|d_task_md=24' $QEMU_LOG 2>/dev/null")
    data_ind=$(docker exec "$CONTAINER" bash -c "grep -c 'DATA_IND\|L1CTL_DATA_IND' $QEMU_LOG 2>/dev/null")
    fb_det_nonzero=$(docker exec "$CONTAINER" bash -c "grep -c 'ARM RD d_fb_det.*= 0x0001' $QEMU_LOG 2>/dev/null")

    echo
    echo "Métriques [$label / synth=$synth] après ${WINDOW}s :"
    printf '  A_CD-WR events           : %s\n'   "$a_cd"
    printf '  A_CD-WR unique addrs     : %s\n'   "$a_cd_uniq"
    printf '  task=24 (CCCH demod)     : %s\n'   "$task24"
    printf '  DATA_IND emitted         : %s\n'   "$data_ind"
    printf '  d_fb_det=1 (fb-det hit)  : %s\n'   "$fb_det_nonzero"

    echo "$label,$synth,$a_cd,$a_cd_uniq,$task24,$data_ind,$fb_det_nonzero"
}

# CSV header
echo "label,synth,A_CD_WR,A_CD_uniq,task24,DATA_IND,d_fb_det_1" > /tmp/bisect_results.csv

# Run 1 — synth=0 (état actuel, DSP réel)
out=$(measure "synth_off" 0 | tee /dev/tty | tail -1)
echo "$out" >> /tmp/bisect_results.csv

# Run 2 — synth=1 (court-circuit DSP)
out=$(measure "synth_on" 1 | tee /dev/tty | tail -1)
echo "$out" >> /tmp/bisect_results.csv

# Verdict
echo
echo "=================================================="
echo "VERDICT (cf. /tmp/bisect_results.csv)"
echo "=================================================="
cat /tmp/bisect_results.csv | column -t -s,

python3 << 'EOF'
import csv
rows = list(csv.DictReader(open("/tmp/bisect_results.csv")))
if len(rows) < 2:
    print("(pas assez de données)"); raise SystemExit
off = next(r for r in rows if r["synth"] == "0")
on  = next(r for r in rows if r["synth"] == "1")
print()
print(f"synth=0 (DSP réel)  : A_CD-WR={off['A_CD_WR']} DATA_IND={off['DATA_IND']}")
print(f"synth=1 (DSP bypass): A_CD-WR={on['A_CD_WR']}  DATA_IND={on['DATA_IND']}")
print()
a_cd_on, di_on = int(on["A_CD_WR"]), int(on["DATA_IND"])
if a_cd_on >= 15 and di_on > 0:
    print("→ DSP est LE bottleneck (chaîne ARM→l1ctl fonctionne en synth).")
    print("  Action : optimiser perf c54x emulator (decoder opcode, hot loops).")
elif a_cd_on < 15 and di_on == 0:
    print("→ Bug INDÉPENDANT du DSP : la chaîne ARM→l1ctl est cassée aussi en synth.")
    print("  Action : isoler le bug ARM/l1ctl AVANT de toucher au DSP.")
else:
    print("→ Résultat mitigé : DSP est UN bottleneck, mais pas le seul.")
    print(f"  (A_CD-WR_on={a_cd_on} vs cible 15 ; DATA_IND_on={di_on} vs cible > 0)")
EOF
