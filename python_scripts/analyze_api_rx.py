#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_api_rx.py — analyse un dump RX de l'API DSP (DARAM) capturé par
calypso_bsp.c via BSP_DUMP_RX_FILE.

Format du dump (append mode, multi-burst) :
  par burst : header 12B  ('IQ16' magic | fn[4 LE] | tn[1] | n_int16[2 LE] | _pad[1])
              payload    n_int16 × int16 LE   (I,Q interleaved)

Utilise les fonctions FFT de fcch_ref.py (importé en module).

Usage :
  python3 analyze_api_rx.py /tmp/api_rx.bin               # liste les bursts
  python3 analyze_api_rx.py /tmp/api_rx.bin --burst 0     # analyse burst N
  python3 analyze_api_rx.py /tmp/api_rx.bin --fcch-only   # filtre fn%51 ∈ {0,10,20,30,40}
  python3 analyze_api_rx.py /tmp/api_rx.bin --burst -1    # le dernier
"""
import sys, os, struct, argparse, numpy as np

# Import fcch_ref pour les constantes + analyze()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fcch_ref

HDR_FMT  = '<4sIBHB'        # magic(4) fn(4 LE) tn(1) n(2 LE) pad(1)
HDR_SIZE = struct.calcsize(HDR_FMT)
FCCH_FNS = {0, 10, 20, 30, 40}


def parse_dump(path):
    """Yield (fn, tn, iq_complex_array) for each burst in file."""
    with open(path, 'rb') as f:
        while True:
            hdr = f.read(HDR_SIZE)
            if len(hdr) < HDR_SIZE: break
            magic, fn, tn, n_int16, _pad = struct.unpack(HDR_FMT, hdr)
            if magic != b'IQ16':
                print(f"  bad magic at offset {f.tell()-HDR_SIZE}: {magic!r}", file=sys.stderr)
                return
            payload = f.read(n_int16 * 2)
            if len(payload) < n_int16 * 2:
                print(f"  short payload at fn={fn} tn={tn}", file=sys.stderr)
                return
            arr = np.frombuffer(payload, dtype='<i2')
            # Convention bridge : I0 Q0 I1 Q1 ... (interleaved int16 LE)
            iq = arr[0::2].astype(np.float64) + 1j*arr[1::2].astype(np.float64)
            yield fn, tn, iq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('file')
    ap.add_argument('--burst', type=int, default=None,
                    help="index burst à analyser (négatif = depuis la fin)")
    ap.add_argument('--fcch-only', action='store_true',
                    help="ne lister/analyser que les bursts fn%%51 ∈ {0,10,20,30,40}")
    ap.add_argument('--osr', type=int, default=1,
                    help="suréchantillonnage du dump (default 1 = 1 sample/symbol)")
    a = ap.parse_args()

    fs = fcch_ref.RB * a.osr
    bursts = list(parse_dump(a.file))
    if a.fcch_only:
        bursts = [b for b in bursts if (b[0] % 51) in FCCH_FNS]
    if not bursts:
        print("aucun burst trouvé"); sys.exit(1)

    if a.burst is None:
        # Liste résumée
        print(f"== {a.file} : {len(bursts)} bursts (fcch_only={a.fcch_only}) ==")
        for i, (fn, tn, iq) in enumerate(bursts[:50]):
            fcch_mark = " FCCH" if (fn % 51) in FCCH_FNS else ""
            mag = np.abs(iq).mean()
            print(f"  #{i:>4}  fn={fn:>7}  tn={tn}  n={iq.size:>4}  |iq|={mag:>7.1f}{fcch_mark}")
        if len(bursts) > 50:
            print(f"  ... (+{len(bursts)-50} more)")
        print(f"\nUsage : python3 {sys.argv[0]} {a.file} --burst N [--fcch-only]")
        return

    idx = a.burst if a.burst >= 0 else len(bursts) + a.burst
    if not 0 <= idx < len(bursts):
        print(f"index {a.burst} hors range [0..{len(bursts)-1}]"); sys.exit(1)
    fn, tn, iq = bursts[idx]
    fcch_mark = " (FCCH frame)" if (fn % 51) in FCCH_FNS else ""
    print(f"== burst #{idx} fn={fn} tn={tn} n={iq.size} fs={fs:.0f} Hz{fcch_mark} ==")
    print(f"   |iq| mean = {np.abs(iq).mean():.1f}  max = {np.abs(iq).max():.1f}")
    fcch_ref.analyze(iq, fs, f"burst#{idx} fn={fn}")
    print(f"\nRéférence attendue (FCCH propre) :")
    print(f"   pic            : +{fcch_ref.F_FCCH:.1f} Hz")
    print(f"   pureté tone    : >90 % (lock probable)")


if __name__ == '__main__':
    main()
