#!/usr/bin/env bash
# make_diag_bundle.sh — collecte et package les données de diag du run actif.
#
# Output : un tar.gz dans /home/nirvana/ (ou $OUT_DIR) avec ownership nirvana.
# Inclut : logs filtrés, PC HIST complet, tail brut, static dumps des zones
# stuck (auto-detect), source excerpts des décodeurs, env+boot trace.
#
# Pas de prompt Claude web — juste les données. À toi de les analyser ou
# de les uploader où tu veux.
#
# Usage:
#   ./make_diag_bundle.sh                    # défaut : container "trying", out /home/nirvana/
#   CONTAINER=foo ./make_diag_bundle.sh      # autre container
#   OUT_DIR=/tmp ./make_diag_bundle.sh       # autre dest
#   TAG=v5_test ./make_diag_bundle.sh        # tag custom (défaut = timestamp)
#   ZONES="0xa0e0 0x8200 0xfc50" ./make_diag_bundle.sh   # zones static dump
#                                                          (défaut = auto-detect)
#   SKIP="osmocon frame_irq" ./make_diag_bundle.sh        # exclure des fichiers
#   TAIL_LINES=8000 ./make_diag_bundle.sh                  # taille du tail brut
#
# Pré-requis : docker exec dispo, dsp_read.sh dans le container.

set -u

CONTAINER="${CONTAINER:-trying}"
OUT_DIR="${OUT_DIR:-/home/nirvana}"
TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
ZONES="${ZONES:-}"
SKIP="${SKIP:-}"
TAIL_LINES="${TAIL_LINES:-4000}"
OWNER="${OWNER:-nirvana:nirvana}"

# --- preflight checks ------------------------------------------------
INSIDE_CONTAINER=0
[ -f /.dockerenv ] && INSIDE_CONTAINER=1

if [ "$INSIDE_CONTAINER" != "1" ]; then
    if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
        echo "ERR: container '$CONTAINER' not running" >&2
        exit 1
    fi
fi
if [[ ! -d "$OUT_DIR" ]]; then
    echo "ERR: output dir '$OUT_DIR' does not exist" >&2
    exit 1
fi

WORK=$(mktemp -d -t bundle.XXXXXX)
trap 'rm -rf "$WORK"' EXIT
echo "[bundle] work dir: $WORK"
echo "[bundle] tag: $TAG"
echo "[bundle] container: $CONTAINER"

skip() { [[ " $SKIP " == *" $1 "* ]]; }

# --- 1. Pull logs from container ------------------------------------
echo "[bundle] pulling logs from container..."
# In-container : PROC_ROOT="" (paths absolus directs). Out : via /proc/N/root.
if [ "$INSIDE_CONTAINER" = "1" ]; then
    PROC_ROOT=""
else
    PROC_PID=$(docker inspect --format '{{.State.Pid}}' "$CONTAINER" 2>/dev/null)
    PROC_ROOT="/proc/$PROC_PID/root"
    if [[ ! -d "$PROC_ROOT" ]]; then
        echo "ERR: cannot access /proc/$PROC_PID/root" >&2
        exit 1
    fi
fi

cp -a "$PROC_ROOT/root/qemu.log" "$WORK/qemu_full.log" 2>/dev/null || \
    { echo "ERR: cannot read /root/qemu.log in container"; exit 1; }

skip bridge   || cp -a "$PROC_ROOT/tmp/bridge.log"   "$WORK/bridge.log"   2>/dev/null || true
skip osmocon  || cp -a "$PROC_ROOT/tmp/osmocon.log"  "$WORK/osmocon.log"  2>/dev/null || true
skip frame_irq|| cp -a "$PROC_ROOT/tmp/frame_irq.log" "$WORK/frame_irq.log" 2>/dev/null || true
skip mobile   || cp -a "$PROC_ROOT/tmp/mobile.log"   "$WORK/mobile.log"   2>/dev/null || true
skip bts      || cp -a "$PROC_ROOT/tmp/bts.log"      "$WORK/bts.log"      2>/dev/null || true
skip tdma_profile || cp -a "$PROC_ROOT/tmp/tdma_profile.log" "$WORK/tdma_profile.log" 2>/dev/null || true
skip tdma_tick    || cp -a "$PROC_ROOT/tmp/tdma_tick.log"    "$WORK/tdma_tick.log"    2>/dev/null || true
skip qemu-fw      || cp -a "$PROC_ROOT/tmp/qemu-fw-console.log" "$WORK/qemu-fw-console.log" 2>/dev/null || true
# IrDA debug channel (Phase 3 PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md) :
# fw-irda.log = capture du PTY serial1 (UART_IRDA, 0xFFFF5000), tracé par
# tools/irda_capture.py via /tmp/irda.pty.link. Vide tant que Phase 0.5
# (cons_puts au boot) pas appliquée — utile quand-même pour confirmer que
# la capture tourne et que le canal n'est pas saturé.
skip fw-irda  || cp -a "$PROC_ROOT/tmp/fw-irda.log"  "$WORK/fw-irda.log"  2>/dev/null || true
skip fw-irda  || cp -a "$PROC_ROOT/tmp/irda_capture.stderr.log" "$WORK/irda_capture.stderr.log" 2>/dev/null || true
skip fw-irda  || cp -a "$PROC_ROOT/tmp/irda_capture.pid" "$WORK/irda_capture.pid" 2>/dev/null || true

raw_size=$(wc -c < "$WORK/qemu_full.log")
echo "[bundle] qemu.log raw size: $((raw_size/1024)) KB"

# --- 2. Filtered grep — all known markers (extensible) -------------
echo "[bundle] generating filtered logs..."
grep -nE 'XC-COND|DUAL-OP-INTERPRET|SP-CATASTROPHE|IMR-W|INTM-TRANS|cause prev_exec|DSP WR a_sync|ENTER-770c|ENTER-7700|ENTER-8d2d|MAC-8d33|fbsb hook|MAC-7700|IRQ #|STALE ratio|BSP LOAD|BSP DMA|DISP-FLAG-W|ARM TASK WR|ARM RD|d_fb_det|a_sync_TOA|VEC-TRACE|PENDING IRQ|WATCH-WRITE|WATCH-READ|HOT-OPS-DUMP|FORCE-DARAM62|PMST WR|IMR change|DISP-PTR|DISP-WRITE|RPTB' \
    "$WORK/qemu_full.log" > "$WORK/qemu_diag.log" || true

grep 'PC HIST insn=' "$WORK/qemu_full.log" > "$WORK/pc_hist.log" || true
tail -"$TAIL_LINES" "$WORK/qemu_full.log" > "$WORK/qemu_tail.log"

# ENV + boot trace
grep -iE 'CALYPSO_|^\[BSP\]|^\[calypso-trx\]|^\[calypso-fbsb\]|^\[c54x\] BOOT|^\[c54x\] Reset|PMST=|^\[BL ARM' \
    "$WORK/qemu_full.log" 2>/dev/null | head -300 > "$WORK/env_boot.log" || true

# Optionnellement : remove the full dump if it's huge (keep filtered + tail)
if [[ "$raw_size" -gt 10000000 ]]; then
    rm "$WORK/qemu_full.log"
    echo "[bundle] qemu_full.log removed (>10MB) — kept qemu_diag.log + qemu_tail.log"
fi

# --- 3. Auto-detect stuck zones if ZONES not provided --------------
if [[ -z "$ZONES" ]]; then
    echo "[bundle] auto-detecting stuck zones from PC HIST..."
    # Take last PC HIST window, extract top 5 distinct PCs (4 hex digits)
    last_pc_hist=$(tail -1 "$WORK/pc_hist.log" 2>/dev/null | sed 's/.*top: //')
    if [[ -n "$last_pc_hist" ]]; then
        # Extract first 5 unique 4-hex PCs
        ZONES=$(echo "$last_pc_hist" | grep -oE '[0-9a-f]{4}' | head -5 | sort -u | sed 's/^/0x/' | tr '\n' ' ')
        echo "[bundle] auto-zones: $ZONES"
    fi
    # Always include the canonical ones we've been investigating
    for z in 0xa0e0 0xa0e7 0x8208 0x8d2d 0xfc54; do
        case " $ZONES " in *" $z "*) :;; *) ZONES="$ZONES $z";; esac
    done
fi

# --- 4. Static prog dumps for the chosen zones ---------------------
echo "[bundle] static prog dumps for: $ZONES"
{
    for zone in $ZONES; do
        # Convert zone to integer (handle both 0x... and bare hex)
        zone_int=$((zone))
        zone_hex=$(printf '0x%04x' "$zone_int")
        echo "=== Static dump prog[${zone_hex}..$(printf '0x%04x' $((zone_int+0x1f)))] ==="
        # Determine section : prom0 = 0x7000-0xDFFF, prom1 = 0xE000-0xFF7F
        if (( zone_int >= 0xE000 && zone_int <= 0xFFFF )); then
            section=prom1
        else
            section=prom0
        fi
        for off in $(seq 0 31); do
            addr=$(printf '0x%04x' $((zone_int + off)))
            if [ "$INSIDE_CONTAINER" = "1" ]; then
                bash /opt/GSM/qemu-src/dsp_read.sh "$section" "$addr" 2>&1
            else
                docker exec "$CONTAINER" bash /opt/GSM/qemu-src/dsp_read.sh "$section" "$addr" 2>&1
            fi
        done
        echo
    done
} > "$WORK/static_decode.txt"

# --- 5. Source excerpts ---------------------------------------------
echo "[bundle] extracting source excerpts..."
SRC=/home/nirvana/qemu-src/hw/arm/calypso/calypso_c54x.c
{
    echo "=== calypso_c54x.c — version + size ==="
    ls -la "$SRC"
    echo
    if [[ -r "$SRC" ]]; then
        echo "=== ST||LD dual-op handler (C8-CB) ==="
        grep -n 'C8/C9/CA/CB' "$SRC" | head -5
        sed -n '4485,4545p' "$SRC"
        echo
        echo "=== MAC dual-op handler (D0-D9) ==="
        sed -n '4302,4350p' "$SRC"
        echo
        echo "=== MASA / SQUR (DB / DC) ==="
        sed -n '4360,4420p' "$SRC"
        echo
        echo "=== XC-COND probe (line ~5005-5070) ==="
        sed -n '5005,5080p' "$SRC"
        echo
        echo "=== SP-CATASTROPHE + DUAL-OP-INTERPRET tracer ==="
        grep -n 'SP-CATASTROPHE\|DUAL-OP-INTERPRET' "$SRC" | head -10
        echo
        echo "=== INTM-TRANS tracer (uses last_exec_pc) ==="
        grep -n 'INTM-TRANS' "$SRC" | head -5
    fi
} > "$WORK/source_excerpts.txt"

# --- 6. Markers summary (run output of parse_dsp_log.sh) -----------
echo "[bundle] running parse_dsp_log.sh for summary snapshot..."
if [[ -x /home/nirvana/qemu-src/parse_dsp_log.sh ]]; then
    SECTIONS="meta env markers pc stuck pc-zones bsp sp imr dual snr intm mac irq summary" \
        /home/nirvana/qemu-src/parse_dsp_log.sh > "$WORK/parse_summary.txt" 2>&1 || true
fi

# --- 7. Sizes report ------------------------------------------------
echo "[bundle] file sizes:"
( cd "$WORK" && wc -l *.log *.txt 2>/dev/null )
echo

# --- 8. Tarball + copy to OUT_DIR with owner -----------------------
TARBALL="diag_${TAG}.tar.gz"
( cd "$WORK" && tar czf "$OUT_DIR/$TARBALL" *.log *.txt )
SIZE=$(du -h "$OUT_DIR/$TARBALL" | awk '{print $1}')
chown "$OWNER" "$OUT_DIR/$TARBALL" 2>/dev/null || true

echo "[bundle] DONE"
echo "  → $OUT_DIR/$TARBALL ($SIZE, owner $OWNER)"
ls -la "$OUT_DIR/$TARBALL"
echo
echo "[bundle] contents :"
tar tzvf "$OUT_DIR/$TARBALL" | awk '{printf "  %10s  %s\n", $3, $NF}' | sort -k1 -n -r
