#!/usr/bin/env bash
# parse_dsp_log.sh — synthèse exhaustive du run actif Calypso/DSP.
#
# Usage:
#   ./parse_dsp_log.sh                        # log container "trying"
#   ./parse_dsp_log.sh /path/to/qemu.log      # log local
#   CONTAINER=foo ./parse_dsp_log.sh          # autre container
#   SECTIONS="markers pc sp"                  # restreindre les sections
#   COMPARE=/path/to/old.log                  # comparer avec un run précédent
#
# Sections (toutes par défaut, ordre = ordre d'affichage):
#   meta     — taille/mtime binaire et log + run age
#   env      — ENV vars détectées dans le log (FBSB_SYNTH, W1C_LATCH, etc.)
#   markers  — counts globaux (tous les markers DSP/BSP/L1)
#   pc       — derniers PC HIST + zone hot
#   stuck    — détecte stagnation (sum-delta < tolerance)
#   pc-zones — distribution des zones DSP visitées (0x8d, 0xeb, etc.)
#   bsp      — stats BSP DMA (stale ratio, BSP LOAD, DMA hits)
#   sp       — détail SP-CATASTROPHE + opcode breakdown + AR analysis
#   imr      — IMR-W *ZERO* par PC+op + détection PC=0x0888 firmware-intentional
#   dual     — DUAL-OP-INTERPRET + analyse current_dec vs SPRU
#   dispflag — DISP-FLAG-W top addresses (DARAM[0x40..0x90] writes)
#   snr      — a_sync_SNR DSP-side, distribution + write PCs + déterminisme
#   intm     — INTM-TRANS (last 6 with cause prev_exec)
#   mac      — MAC-7700, MAC-8d33, ENTER-* trace counts
#   irq      — IRQ events sequence
#   bridge   — bridge.log activity
#   summary  — verdict consolidé fin-de-rapport
#
# Couleurs : auto si TTY, sinon plain.

set -u

CONTAINER="${CONTAINER:-trying}"
LOG="${1:-}"
SECTIONS="${SECTIONS:-meta env markers pc stuck pc-zones bsp sp imr dual dispflag snr intm mac irq bridge summary}"
COMPARE="${COMPARE:-}"

# --- detect log source --------------------------------------------------
if [[ -z "$LOG" ]]; then
    if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
        echo "ERR: container '$CONTAINER' not running" >&2
        exit 1
    fi
    READ_LOG()    { docker exec "$CONTAINER" cat /root/qemu.log; }
    READ_BRIDGE() { docker exec "$CONTAINER" cat /tmp/bridge.log 2>/dev/null || true; }
    META_CMD()    { docker exec "$CONTAINER" bash -c \
        'ls -la /opt/GSM/qemu-src/build/qemu-system-arm /root/qemu.log /tmp/bridge.log 2>/dev/null'; }
else
    [[ ! -f "$LOG" ]] && { echo "ERR: log file not found: $LOG" >&2; exit 1; }
    READ_LOG()    { cat "$LOG"; }
    READ_BRIDGE() {
        local b="$(dirname "$LOG")/bridge.log"
        [[ -f "$b" ]] && cat "$b" || true
    }
    META_CMD()    { ls -la "$LOG"; }
fi

# Cache log content once, all sections grep over $TMP for consistency.
TMP=$(mktemp -t qemulog.XXXXXX)
BTMP="${TMP}.bridge"
CTMP="${TMP}.compare"
trap 'rm -f "$TMP" "$BTMP" "$CTMP"' EXIT
READ_LOG > "$TMP" || { echo "ERR: cannot read log" >&2; exit 1; }
READ_BRIDGE > "$BTMP" 2>/dev/null || true
[[ -n "$COMPARE" && -f "$COMPARE" ]] && cp "$COMPARE" "$CTMP" || : > "$CTMP"

# --- color helpers ------------------------------------------------------
if [[ -t 1 ]]; then
    BOLD=$'\e[1m'; DIM=$'\e[2m'; RED=$'\e[31m'; GRN=$'\e[32m'
    YEL=$'\e[33m'; BLU=$'\e[34m'; CYA=$'\e[36m'; MAG=$'\e[35m'; RST=$'\e[0m'
else
    BOLD=''; DIM=''; RED=''; GRN=''; YEL=''; BLU=''; CYA=''; MAG=''; RST=''
fi
hdr()  { printf '\n%s=== %s ===%s\n' "$BOLD" "$1" "$RST"; }
sub()  { printf '%s--- %s ---%s\n' "$DIM" "$1" "$RST"; }
warn() { printf '%s%s%s\n' "$YEL" "$1" "$RST"; }
crit() { printf '%s%s%s\n' "$RED" "$1" "$RST"; }
ok()   { printf '%s%s%s\n' "$GRN" "$1" "$RST"; }
dim()  { printf '%s%s%s\n' "$DIM" "$1" "$RST"; }

has_section() { [[ " $SECTIONS " == *" $1 "* ]]; }

# Robust grep -c (returns "0" without error / no concat)
grepc() {
    local n
    n=$(grep -c "$1" "$TMP" 2>/dev/null) || true
    n=${n:-0}
    echo "$n"
}

# --- meta --------------------------------------------------------------
if has_section meta; then
    hdr "META — binary + log"
    META_CMD
    last_insn=$(grep -E 'PC HIST insn=' "$TMP" | tail -1 | sed -E 's/.*insn=([0-9]+).*/\1/')
    log_size=$(wc -c < "$TMP")
    if [[ -n "$last_insn" ]]; then
        printf "Last insn=%s  log=%dKB" "$last_insn" $((log_size/1024))
        # Estimated wall-time at ~50M insn/s typical
        if [[ "$last_insn" -gt 0 ]]; then
            wall_s=$((last_insn / 50000000))
            printf "  ≈%ds DSP wall\n" "$wall_s"
        else
            echo
        fi
    else
        echo "(no PC HIST yet)"
    fi
    if [[ -s "$CTMP" ]]; then
        cmp_insn=$(grep -E 'PC HIST insn=' "$CTMP" | tail -1 | sed -E 's/.*insn=([0-9]+).*/\1/')
        printf "Compare run last insn=%s\n" "${cmp_insn:-?}"
    fi
fi

# --- env --------------------------------------------------------------
if has_section env; then
    hdr "ENV — variables détectées"
    grep -E '\[(BSP|calypso-trx|calypso-fbsb|c54x)\].*CALYPSO_|FORCE-DARAM62|BYPASS_BDLENA|ICOUNT' "$TMP" | head -10
    # Highlight conflicts (e.g. multiple FBSB_SYNTH lines)
    fbsb_n=$(grep -c 'CALYPSO_FBSB_SYNTH=' "$TMP" 2>/dev/null) || fbsb_n=0
    [[ "$fbsb_n" -gt 1 ]] && warn "  ⚠ Multiple FBSB_SYNTH lines — env might have flipped mid-run"
fi

# --- markers -----------------------------------------------------------
if has_section markers; then
    hdr "MARKERS — counts globaux"
    cnt() {
        local label="$1" pattern="$2" color="${3:-}"
        local n
        n=$(grepc "$pattern")
        local cmp=""
        if [[ -s "$CTMP" ]]; then
            local nc
            nc=$(grep -c "$pattern" "$CTMP" 2>/dev/null) || nc=0
            local d=$((n - nc))
            if [[ "$d" -gt 0 ]]; then cmp=" (+$d vs cmp)"; elif [[ "$d" -lt 0 ]]; then cmp=" ($d vs cmp)"; fi
        fi
        if [[ -z "$color" ]]; then
            printf "  %-28s %d%s\n" "$label" "$n" "$cmp"
        else
            printf "  %-28s %s%d%s%s\n" "$label" "$color" "$n" "$RST" "$cmp"
        fi
    }
    cnt "IMR-W *ZERO*"        'IMR-W \*ZERO\*'        "$RED"
    cnt "SP-CATASTROPHE"      'SP-CATASTROPHE'         "$RED"
    cnt "DUAL-OP-INTERPRET"   'DUAL-OP-INTERPRET'      "$YEL"
    cnt "HOT-OPS-DUMP"        'HOT-OPS-DUMP'           ""
    cnt "INTM-TRANS"          'cause prev_exec'        ""
    cnt "DISP-FLAG-W"         'DISP-FLAG-W'            ""
    cnt "ENTER-770c"          'ENTER-770c'             "$GRN"
    cnt "ENTER-7700"          'ENTER-7700'             ""
    cnt "ENTER-8d2d"          'ENTER-8d2d'             ""
    cnt "DSP WR a_sync_SNR"   'DSP WR a_sync_SNR'      "$GRN"
    cnt "DSP WR a_sync_TOA"   'a_sync_TOA'             ""
    cnt "d_fb_det WR"         'd_fb_det'               ""
    cnt "fbsb hook fired"     'fbsb hook fired'        "$GRN"
    cnt "L1CTL_DATA_IND"      'L1CTL_DATA_IND'         "$GRN"
    cnt "ARM TASK WR"         'ARM TASK WR'            ""
    cnt "IRQ events"          '\[c54x\] IRQ #'         ""
    cnt "BSP LOAD"            'BSP LOAD'               ""
    cnt "BSP DMA fn=*"        '\[BSP\] DMA fn='        ""
    cnt "STALE ratio"         'STALE ratio:'           ""
    cnt "MAC-7700"            'MAC-7700'               ""
    cnt "MAC-8d33"            'MAC-8d33'               ""
    cnt "VEC-TRACE"           'VEC-TRACE'              ""
    cnt "PENDING IRQ"         'PENDING IRQ'            ""
fi

# --- pc hist -----------------------------------------------------------
if has_section pc; then
    hdr "PC HIST — last 3 windows"
    grep 'PC HIST insn=' "$TMP" | tail -3
fi

# --- stagnation -------------------------------------------------------
if has_section stuck; then
    hdr "STAGNATION — DSP stuck detection"
    last2=$(grep 'PC HIST insn=' "$TMP" | tail -2)
    if [[ -n "$last2" ]]; then
        sum_of() {
            echo "$1" | grep -oE '[0-9a-f]{4}:[0-9]+' | \
                awk -F: '{s+=$2} END {print s+0}'
        }
        line1=$(echo "$last2" | head -1 | sed 's/.*top: //')
        line2=$(echo "$last2" | tail -1 | sed 's/.*top: //')
        sum1=$(sum_of "$line1")
        sum2=$(sum_of "$line2")
        delta=$(( sum2 - sum1 ))
        abs_delta=${delta#-}
        if (( abs_delta < 1000 )); then
            crit "  STUCK — sum-delta=$delta (< 1000) between last 2 PC HIST windows"
            top_pc=$(echo "$last2" | tail -1 | sed 's/.*top: //' | awk '{print $1}' | cut -d: -f1)
            echo "  Hot region: 0x$top_pc  (sum1=$sum1 sum2=$sum2)"
            # Detect zone classification
            case "$top_pc" in
                8d*) echo "  Zone: 0x8d3X = FB-det inner correlator (historical)";;
                eb*) echo "  Zone: 0xebXX = PROM1 mirror trap";;
                fc*) echo "  Zone: 0xfcXX = PROM1 mirror, often post-fbdet";;
                82*) echo "  Zone: 0x82XX = correlator routine (with W1C_LATCH=1)";;
                99*) echo "  Zone: 0x99XX = ?";;
                *)   echo "  Zone: unknown — investigate";;
            esac
        else
            ok "  PROGRESSING — sum-delta=$delta between last 2 windows"
        fi
    else
        dim "  not enough PC HIST data"
    fi
fi

# --- pc zones (which zones have been visited) -------------------------
if has_section pc-zones; then
    hdr "PC ZONES — high-level visit map"
    for prefix in 7700 0x8d 0xeb 0xfc 0x82 0x99 0x16 0x17 0xa0 0xc8 0xff; do
        zone_short="${prefix#0x}"
        n=$(grep 'PC HIST insn=' "$TMP" | grep -oE "${zone_short}[0-9a-f]{2}" | sort -u | wc -l 2>/dev/null) || n=0
        printf "  zone 0x%-4s : %3d distinct PCs visited in PC HIST\n" "$zone_short" "$n"
    done
fi

# --- bsp -------------------------------------------------------------
if has_section bsp; then
    hdr "BSP DMA — sample delivery to DSP"
    sub "Stats from STALE ratio log"
    grep 'STALE ratio' "$TMP" | tail -3
    sub "BSP DMA log (per-1000 + first 10) — sample of fn delivered"
    grep '\[BSP\] DMA fn=' "$TMP" | head -3
    echo "..."
    grep '\[BSP\] DMA fn=' "$TMP" | tail -3
    sub "fn%51 distribution — DSP-delivered (sparse, log-rate-limited)"
    grep '\[BSP\] DMA' "$TMP" | grep -oE 'fn=[0-9]+' | \
        awk -F= '{print $2 % 51}' | sort -n | uniq -c | sort -rn | head -8
    sub "fn%51 distribution — bridge sent (full)"
    if [[ -s "$BTMP" ]]; then
        grep 'DL #' "$BTMP" | grep -oE 'qfn=[0-9]+' | \
            awk -F= '{print $2 % 51}' | sort -n | uniq -c | sort -rn | head -8
    else
        dim "  no bridge.log"
    fi
fi

# --- sp catastrophe ----------------------------------------------------
if has_section sp; then
    hdr "SP-CATASTROPHE — opcode breakdown + AR analysis"
    n_sp=$(grepc 'SP-CATASTROPHE')
    if [[ "$n_sp" -gt 0 ]]; then
        sub "Top opcodes (count × op)"
        grep 'SP-CATASTROPHE' "$TMP" | awk '{
            for(i=1;i<=NF;i++) if ($i ~ /^op=/) { print $i; break }
        }' | sort | uniq -c | sort -rn | head -10
        sub "Top PCs (count × PC)"
        grep 'SP-CATASTROPHE' "$TMP" | awk '{
            for(i=1;i<=NF;i++) if ($i ~ /^PC=/) { print $i; break }
        }' | sort | uniq -c | sort -rn | head -10
        sub "Last 5 events (full)"
        grep 'SP-CATASTROPHE' "$TMP" | tail -5 | sed -E 's/^(.{120}).*/\1.../'
        sub "AR analysis : how often is some AR=0x0018 (MMR_SP) post-instruction?"
        ar18=$(grep 'SP-CATASTROPHE' "$TMP" | grep -cE 'AR[0-7]?: ([0-9a-f]{4} ){0,7}0018') || ar18=0
        echo "  events with any AR pos visible at 0x0018: $ar18 / $n_sp"
        ar00=$(grep 'SP-CATASTROPHE' "$TMP" | grep -cE 'AR[0-7]?: ([0-9a-f]{4} ){0,7}0000') || ar00=0
        echo "  events with any AR pos visible at 0x0000: $ar00 / $n_sp"
    else
        ok "  none"
    fi
fi

# --- imr=0 culprits ----------------------------------------------------
if has_section imr; then
    hdr "IMR-W *ZERO* — culprits"
    n_imr=$(grepc 'IMR-W \*ZERO\*')
    if [[ "$n_imr" -gt 0 ]]; then
        sub "Distinct PC+op (count × PC+op)"
        grep 'IMR-W \*ZERO\*' "$TMP" | awk '{
            pc=""; op="";
            for(i=1;i<=NF;i++) {
                if ($i ~ /^PC=/)  pc=$i;
                if ($i ~ /^op=/)  op=$i;
            }
            print pc, op;
        }' | sort | uniq -c | sort -rn | head -10
        # Detect known firmware-intentional sites
        f0888=$(grep -c 'IMR-W \*ZERO\*.*PC=0x0888' "$TMP" 2>/dev/null) || f0888=0
        if [[ "$f0888" -gt 30 ]]; then
            warn "  ℹ PC=0x0888 = $f0888 hits — likely firmware-intentional (STM #0,IMR in ISR critical section)"
        fi
        # Highlight C8/C9/CA/CB ops (= dual-op via my fix territory)
        sub "Dual-op (C8-CB) IMR=0 — relates to encoding fix"
        grep 'IMR-W \*ZERO\*' "$TMP" | grep -cE 'op=0xc[89ab]' | head -1 | xargs -I{} echo "  C8-CB hits: {}"
    else
        ok "  none"
    fi
fi

# --- dual-op interpret -------------------------------------------------
if has_section dual; then
    hdr "DUAL-OP-INTERPRET — encoding evidence"
    n_dop=$(grepc 'DUAL-OP-INTERPRET')
    if [[ "$n_dop" -gt 0 ]]; then
        warn "  $n_dop hits — SP catastrophes occurred on 0xC8xx opcodes despite C8/CB fix"
        echo "  (= firmware-driven AR pollution upstream, not encoding bug)"
        sub "Top PCs"
        grep 'DUAL-OP-INTERPRET' "$TMP" | awk '{
            for(i=1;i<=NF;i++) if ($i ~ /^PC=/) { print $i; break }
        }' | sort | uniq -c | sort -rn | head -5
        sub "Sample (first 3)"
        grep 'DUAL-OP-INTERPRET' "$TMP" | head -3 | sed -E 's/^(.{180}).*/\1.../'
    else
        ok "  0 hits — fix C8/C9/CA/CB tient (no SP-cat on 0xC8xx)"
    fi
fi

# --- disp flag writes --------------------------------------------------
if has_section dispflag; then
    hdr "DISP-FLAG-W — DARAM[0x40..0x90] writes"
    n=$(grepc 'DISP-FLAG-W')
    if [[ "$n" -gt 0 ]]; then
        sub "Top 12 addresses (count × addr)"
        grep 'DISP-FLAG-W' "$TMP" | awk '{print $3}' | \
            sort | uniq -c | sort -rn | head -12
        sub "Coverage of 0x60-0x65 zone (the supposed dispatcher poll)"
        for a in 0060 0061 0062 0063 0064 0065; do
            c=$(grep -c "DISP-FLAG-W data\[0x$a\]" "$TMP" 2>/dev/null) || c=0
            printf "  data[0x%s]: %d\n" "$a" "$c"
        done
    else
        dim "  no writes captured"
    fi
fi

# --- a_sync_SNR variation ---------------------------------------------
if has_section snr; then
    hdr "a_sync_SNR — DSP-side demod output"
    n=$(grepc 'DSP WR a_sync_SNR')
    if [[ "$n" -gt 0 ]]; then
        nv=$(grep 'DSP WR a_sync_SNR' "$TMP" | grep -oE '= 0x[0-9a-f]+' | sort -u | wc -l)
        if [[ "$nv" -le 1 ]]; then
            warn "  $n writes, $nv distinct value → demod ne converge plus / figé"
        else
            ok "  $n writes, $nv distinct values"
        fi
        sub "Distribution (count × value)"
        grep 'DSP WR a_sync_SNR' "$TMP" | grep -oE '= 0x[0-9a-f]+' | \
            sort | uniq -c | sort -rn | head -8
        sub "Write PCs (sites du correlator qui produit SNR)"
        grep 'DSP WR a_sync_SNR' "$TMP" | grep -oE 'PC=0x[0-9a-f]+' | \
            sort | uniq -c | sort -rn | head -6
        sub "Determinism check (signature 21× 0x2fb0 attendue)"
        d2fb0=$(grep 'DSP WR a_sync_SNR.*= 0x2fb0' "$TMP" | wc -l)
        d164e=$(grep 'DSP WR a_sync_SNR.*= 0x164e' "$TMP" | wc -l)
        echo "  0x2fb0=$d2fb0  0x164e=$d164e (cumulative — déterministe across runs)"
    else
        dim "  no FB-det output yet"
    fi
fi

# --- INTM transitions --------------------------------------------------
if has_section intm; then
    hdr "INTM-TRANS — last 6 (avec cause prev_exec)"
    grep 'cause prev_exec' "$TMP" | tail -6
fi

# --- MAC trace -------------------------------------------------------
if has_section mac; then
    hdr "MAC traces (FB-det correlator instrumentation)"
    sub "MAC-7700 (FB-det at PROM0 0x7700)"
    grep '\[c54x\] MAC-7700' "$TMP" | head -2
    grep '\[c54x\] MAC-7700' "$TMP" | tail -2
    sub "MAC-8d33 + ENTER-8d2d (instrumented zone — for runs that reach it)"
    n_8d33=$(grepc 'MAC-8d33')
    n_8d2d=$(grepc 'ENTER-8d2d')
    echo "  MAC-8d33: $n_8d33  ENTER-8d2d: $n_8d2d"
    if [[ "$n_8d33" -gt 0 ]]; then
        sub "ENTER-8d2d first 5 (A_pre check : reset vs persistent)"
        grep 'ENTER-8d2d' "$TMP" | head -5
    fi
fi

# --- IRQ activity ---------------------------------------------------
if has_section irq; then
    hdr "IRQ activity"
    n_irq=$(grepc '\[c54x\] IRQ #')
    if [[ "$n_irq" -gt 0 ]]; then
        sub "Last 3 IRQ events"
        grep '\[c54x\] IRQ #' "$TMP" | tail -3
        sub "IPTR distribution (which vector base did the IRQs land at?)"
        grep '\[c54x\] IRQ #' "$TMP" | grep -oE 'IPTR=0x[0-9a-f]+' | \
            sort | uniq -c | sort -rn | head -5
        sub "IMR distribution (was service ever possible?)"
        grep '\[c54x\] IRQ #' "$TMP" | grep -oE 'IMR=0x[0-9a-f]+' | \
            sort | uniq -c | sort -rn | head -5
    else
        dim "  no IRQ events"
    fi
fi

# --- bridge activity --------------------------------------------------
if has_section bridge; then
    hdr "BRIDGE — UDP samples / CLK IND activity"
    if [[ -s "$BTMP" ]]; then
        b_lines=$(wc -l < "$BTMP")
        printf "  bridge.log: %d lines total\n" "$b_lines"
        sub "Last tick summary"
        grep -E 'tick=.*clk=.*dl=.*ul=' "$BTMP" | tail -1
        sub "Last 3 DL frames"
        grep 'DL #' "$BTMP" | tail -3 | sed -E 's/^(.{160}).*/\1.../'
        sub "DL drops (lookahead)"
        grep -c 'DL drop' "$BTMP" 2>/dev/null
    else
        dim "  no bridge.log accessible"
    fi
fi

# --- summary -------------------------------------------------------
if has_section summary; then
    hdr "SUMMARY — verdict consolidé"
    last_insn=$(grep -E 'PC HIST insn=' "$TMP" | tail -1 | sed -E 's/.*insn=([0-9]+).*/\1/')
    last2=$(grep 'PC HIST insn=' "$TMP" | tail -2)
    sum2=$(echo "$last2" | tail -1 | sed 's/.*top: //' | grep -oE '[0-9a-f]{4}:[0-9]+' | awk -F: '{s+=$2} END {print s+0}')
    sum1=$(echo "$last2" | head -1 | sed 's/.*top: //' | grep -oE '[0-9a-f]{4}:[0-9]+' | awk -F: '{s+=$2} END {print s+0}')
    delta=$(( sum2 - sum1 )); adelta=${delta#-}
    n_imr=$(grepc 'IMR-W \*ZERO\*')
    n_sp=$(grepc 'SP-CATASTROPHE')
    n_dop=$(grepc 'DUAL-OP-INTERPRET')
    n_l1=$(grepc 'L1CTL_DATA_IND')
    n_fbsb=$(grepc 'fbsb hook fired')
    top_pc=$(echo "$last2" | tail -1 | sed 's/.*top: //' | awk '{print $1}' | cut -d: -f1 2>/dev/null)

    [[ "$adelta" -lt 1000 ]] && state="STUCK at 0x$top_pc" || state="PROGRESSING"
    printf "  State        : %s\n" "$state"
    printf "  Insn count   : %s\n" "${last_insn:-?}"
    printf "  L1 progress  : L1CTL_DATA_IND=%d  fbsb_hook=%d\n" "$n_l1" "$n_fbsb"
    printf "  DSP errors   : IMR=0=%d  SP-cat=%d  dual-op-cat=%d\n" "$n_imr" "$n_sp" "$n_dop"

    if [[ "$n_l1" -gt 0 ]]; then
        ok "  → BCCH/L1 has progressed ✓"
    elif [[ "$n_dop" -gt 50 ]]; then
        crit "  → MAJOR : $n_dop dual-op SP-catastrophes — firmware AR pollution at PC=0x?"
        crit "    Verify if AR4=0x18 (MMR_SP) shows up in DUAL-OP-INTERPRET log"
    elif [[ "$n_imr" -gt 100 ]]; then
        warn "  → $n_imr IMR=0 hits — DSP service pipeline likely broken"
    elif [[ "$adelta" -lt 1000 ]]; then
        warn "  → DSP STUCK at 0x$top_pc — investigate hot zone"
    else
        dim "  → DSP progressing but no L1 yet"
    fi
fi

echo
