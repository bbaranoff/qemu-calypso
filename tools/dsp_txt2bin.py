#!/usr/bin/env python3
"""
dsp_txt2bin.py — Convert calypso_dsp.txt (OsmocomBB Compal DSP dumper format)
to raw 16-bit little-endian binary.

Input format expected (per `c54x_load_rom`, calypso_c54x.c:9798-9854):

    DSP dump: PROM0
    07000 : XXXX XXXX XXXX ... XXXX
    07010 : XXXX XXXX XXXX ... XXXX
    ...
    DSP dump: PROM1
    18000 : ...

Each data line is "ADDR : WORD WORD WORD ..." where ADDR is hex (no prefix)
and each WORD is 4 hex chars. Sections: regs, DROM, PDROM, PROM0..PROM3.

Usage:
  dsp_txt2bin.py INPUT.txt                       # summary only, no output
  dsp_txt2bin.py INPUT.txt OUT.bin                # emit all sections → OUT.SECTION.bin
  dsp_txt2bin.py INPUT.txt OUT.bin --section S   # emit one section, zero-filled
  dsp_txt2bin.py INPUT.txt OUT.bin --slice A L   # emit L words from addr A (hex/dec OK)

Output is raw LE words. Loadable as -M calypso,dsp-blob=OUT.bin if the slice
falls in DARAM's OVLY-accessible range (0x80..0x27FF) and the PC override is
honored.

Examples:
  # split-by-section dump (OUT.bin base → OUT.PROM0.bin, OUT.PROM1.bin, ...)
  dsp_txt2bin.py calypso_dsp.txt calypso_dsp.bin

  # extract first 0x100 words of PROM0 (0x7000..0x70FF) as a probe blob
  dsp_txt2bin.py calypso_dsp.txt prom0_head.bin --slice 0x7000 0x100

  # full PROM0 dump
  dsp_txt2bin.py calypso_dsp.txt prom0.bin --section PROM0
"""

import argparse
import re
import struct
import sys
from collections import defaultdict

SECTION_RE = re.compile(r"DSP dump:\s+(\w+)")
DATA_RE    = re.compile(r"^\s*([0-9a-fA-F]+)\s*:\s*(.+?)\s*$")
WORD_RE    = re.compile(r"[0-9a-fA-F]{1,4}")


def parse_txt(path):
    """Return dict {section_name: {addr: word}}."""
    sections = defaultdict(dict)
    cur = None
    with open(path) as fp:
        for line in fp:
            m = SECTION_RE.search(line)
            if m:
                cur = m.group(1)
                continue
            if cur is None:
                continue
            m = DATA_RE.match(line)
            if not m:
                continue
            base = int(m.group(1), 16)
            for i, w in enumerate(WORD_RE.findall(m.group(2))):
                sections[cur][base + i] = int(w, 16) & 0xFFFF
    return sections


def merge(sections):
    """Flatten all sections into one {addr: word} dict (last writer wins)."""
    out = {}
    for s in sections.values():
        out.update(s)
    return out


def emit_slice(words, start_addr, length, out_path):
    with open(out_path, "wb") as fp:
        for a in range(start_addr, start_addr + length):
            fp.write(struct.pack("<H", words.get(a, 0)))
    filled = sum(1 for a in range(start_addr, start_addr + length) if a in words)
    print(f"[txt2bin] slice {length} words at {start_addr:#06x} → {out_path} "
          f"({filled} filled, {length - filled} zero-filled gaps)")


def emit_section(words, name, out_path):
    if not words:
        sys.exit(f"[txt2bin] section '{name}' is empty")
    lo, hi = min(words), max(words)
    n = hi - lo + 1
    with open(out_path, "wb") as fp:
        for a in range(lo, hi + 1):
            fp.write(struct.pack("<H", words.get(a, 0)))
    filled = len(words)
    print(f"[txt2bin] section {name}: {n} words [{lo:#06x}..{hi:#06x}] → "
          f"{out_path} ({filled} filled, {n - filled} zero-filled gaps)")


def main():
    ap = argparse.ArgumentParser(
        description="Convert calypso_dsp.txt to raw 16-bit LE binary.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("input", help="path to calypso_dsp.txt")
    ap.add_argument("output", nargs="?", help="output .bin path")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--section", help="emit one section (regs/DROM/PDROM/PROM0..3)")
    grp.add_argument("--slice", nargs=2, metavar=("ADDR", "LEN"),
                     help="emit LEN words starting at ADDR (hex or decimal)")
    args = ap.parse_args()

    sections = parse_txt(args.input)

    if not args.output:
        print(f"[txt2bin] sections found in {args.input}:")
        for name in sorted(sections):
            ws = sections[name]
            if not ws:
                print(f"  {name}: (empty)")
                continue
            lo, hi = min(ws), max(ws)
            print(f"  {name:8s}: {len(ws):6d} words, "
                  f"{lo:#07x}..{hi:#07x}  (span {hi - lo + 1})")
        return

    if args.section:
        if args.section not in sections:
            sys.exit(f"[txt2bin] no section named '{args.section}'. "
                     f"Available: {', '.join(sorted(sections))}")
        emit_section(sections[args.section], args.section, args.output)
    elif args.slice:
        addr = int(args.slice[0], 0)
        length = int(args.slice[1], 0)
        emit_slice(merge(sections), addr, length, args.output)
    else:
        # No flags + output given → split each section into OUTPUT.SECTION.bin.
        # Sections live in different address spaces (data vs program) and may
        # overlap in DSP address, so a single merged flat file would be
        # ambiguous. One file per section keeps things semantically clean.
        base = args.output
        if base.lower().endswith(".bin"):
            base = base[:-4]
        any_emitted = False
        for name in sorted(sections):
            if not sections[name]:
                continue
            emit_section(sections[name], name, f"{base}.{name}.bin")
            any_emitted = True
        if not any_emitted:
            sys.exit("[txt2bin] no non-empty sections found in input")


if __name__ == "__main__":
    main()
