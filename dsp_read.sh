#!/bin/bash
# Read DSP ROM word at a given address from the dump file
# Usage: dsp_read.sh <section> <addr_hex>
# Sections: regs, drom, pdrom, prom0, prom1, prom2, prom3
DUMP="${CALYPSO_DSP_ROM:-/opt/GSM/calypso_dsp.txt}"
SECTION="${1:-prom0}"
ADDR="$2"

case "$SECTION" in
  regs)  HEADER="DSP dump: Registers" ;;
  drom)  HEADER="DSP dump: DROM" ;;
  pdrom) HEADER="DSP dump: PDROM" ;;
  prom0) HEADER="DSP dump: PROM0" ;;
  prom1) HEADER="DSP dump: PROM1" ;;
  prom2) HEADER="DSP dump: PROM2" ;;
  prom3) HEADER="DSP dump: PROM3" ;;
  *) echo "Unknown section: $SECTION"; exit 1 ;;
esac

python3 -c "
import sys
hdr = '$HEADER'
target = int('$ADDR', 16)
in_section = False
with open('$DUMP') as f:
    for line in f:
        if 'DSP dump:' in line:
            in_section = hdr in line
            continue
        if not in_section:
            continue
        parts = line.split()
        if len(parts) < 2 or parts[1] != ':':
            continue
        line_addr = int(parts[0], 16)
        if target >= line_addr and target < line_addr + 16:
            idx = target - line_addr
            if idx + 2 < len(parts):
                print(f'{hdr.split(\":\")[1].strip()}[0x{target:04x}] = 0x{parts[idx+2]}')
            break
"
