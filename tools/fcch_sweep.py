#!/usr/bin/env python3
"""α — sweep 51 raw chunks (pre-slice 625 cs16 samples each), classify by dphi_std.

Offset-agnostic FCCH search : on n'a pas le mapping internal_fn ↔ on-air fn,
mais une FCCH = tonalité pure +π/2/sample → dphi_std ≈ 0 et dphi_mean ≈ +π/2.
Tout le reste (BCCH NB, dummy, SACCH) a dphi_std ≥ 1.

Sortie :
- Tableau trié par std croissant.
- Identification automatique des FCCH (top hits avec std<0.1).
- Si ≥ 1 FCCH trouvée : X = internal_fn % 51 → c'est l'offset on-air↔interne
  à utiliser pour Phase 1.5 slot rewrite (X est constant dans un run, change
  au restart osmo-trx → re-détecter au boot ou sniffer TRXD).

Usage :
    CALYPSO_FCCH_DUMP=1 [CALYPSO_FCCH_DUMP_SKIP=2000] ./run.sh
    # attendre "alpha sweep DONE : 51 raw chunks ..."
    python3 tools/fcch_sweep.py
"""
import math
import os
import sys

import numpy as np

FS = 270833.0  # GSM symbol rate at 1 SPS
EXPECT_DPHI = math.pi / 2  # +π/2 rad/sample for FCCH (148 zero bits → +fs/4)
INDEX = "/tmp/fcch_sweep_index.txt"
PATTERN = "/tmp/fcch_sweep_{:03d}.bin"


def load_chunk(idx, ts0_only=True):
    """Load chunk. ts0_only=True extracts first 148 samples (= TS=0 burst).
    The full chunk is 625 samples = half TDMA frame = TS0+TS1+TS2+TS3 mixed,
    which dilutes FCCH signature. TS=0 slicing is what we forward to BSP."""
    path = PATTERN.format(idx)
    raw = np.fromfile(path, dtype="<i2").astype(np.float32)
    if len(raw) < 4 or len(raw) % 2 != 0:
        return None
    iq = raw[0::2] + 1j * raw[1::2]
    if ts0_only:
        iq = iq[:148]
    return iq


def analyze(iq):
    """Return (peak_freq, peak_mag, snr_db, dphi_mean, dphi_std, rms_mag)."""
    n = len(iq)
    fft = np.fft.fft(iq)
    mag = np.abs(fft)
    freqs = np.fft.fftfreq(n, 1.0 / FS)
    peak = int(np.argmax(mag))
    peak_freq = float(freqs[peak])
    peak_mag = float(mag[peak])
    floor = float(np.median(mag))
    snr_db = 20 * math.log10(peak_mag / floor) if floor > 0 else float("inf")

    phase = np.unwrap(np.angle(iq))
    dphi = np.diff(phase)
    if len(dphi) > 16:
        dphi_core = dphi[8:-8]  # trim edges (transient at burst boundary)
    else:
        dphi_core = dphi
    dphi_mean = float(np.mean(dphi_core))
    dphi_std = float(np.std(dphi_core))

    rms_mag = float(np.sqrt(np.mean(np.abs(iq) ** 2)))
    return peak_freq, peak_mag, snr_db, dphi_mean, dphi_std, rms_mag


def main():
    if not os.path.exists(INDEX):
        sys.exit(f"ERR: {INDEX} not found. Run pipeline with CALYPSO_FCCH_DUMP=1 first.")

    # Parse index : idx ts internal_fn internal_fn_mod51 ts_in_frame qfn_tagged
    index = []
    with open(INDEX) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 6:
                continue
            index.append({
                "idx": int(parts[0]),
                "ts": int(parts[1]),
                "internal_fn": int(parts[2]),
                "internal_fn_mod51": int(parts[3]),
                "ts_in_frame": int(parts[4]),
                "qfn_tagged": int(parts[5]),
            })

    if not index:
        sys.exit(f"ERR: {INDEX} has no entries.")

    rows = []
    for entry in index:
        iq = load_chunk(entry["idx"])
        if iq is None or len(iq) < 64:
            continue
        peak_freq, peak_mag, snr_db, dphi_mean, dphi_std, rms_mag = analyze(iq)
        rows.append({
            **entry,
            "n_samples": len(iq),
            "peak_freq": peak_freq,
            "peak_mag": peak_mag,
            "snr_db": snr_db,
            "dphi_mean": dphi_mean,
            "dphi_std": dphi_std,
            "rms_mag": rms_mag,
        })

    if not rows:
        sys.exit("ERR: no readable chunks.")

    # Sort by dphi_std ascending — FCCH should float to top.
    rows.sort(key=lambda r: r["dphi_std"])

    print(f"=== Sweep result : {len(rows)} chunks, sorted by dphi_std ascending ===")
    print(f"{'idx':>3} {'int_fn':>8} {'mod51':>5} {'ts_in_fr':>8} "
          f"{'peak_Hz':>10} {'SNR_dB':>7} {'dphi_mean':>10} {'dphi_std':>9} {'rms':>7}")
    print("-" * 86)
    FCCH_STD_THRESH = 0.30
    FCCH_MEAN_TOL   = 0.25
    for r in rows[:20]:  # top 20
        mark = ""
        if r["dphi_std"] < FCCH_STD_THRESH and abs(r["dphi_mean"] - EXPECT_DPHI) < FCCH_MEAN_TOL:
            mark = " ← FCCH (+π/2)"
        elif r["dphi_std"] < FCCH_STD_THRESH and abs(r["dphi_mean"] + EXPECT_DPHI) < FCCH_MEAN_TOL:
            mark = " ← FCCH MIRROR (-π/2) !!"
        print(f"{r['idx']:>3d} {r['internal_fn']:>8d} "
              f"{r['internal_fn_mod51']:>5d} {r['ts_in_frame']:>8d} "
              f"{r['peak_freq']:>+10.0f} {r['snr_db']:>7.1f} "
              f"{r['dphi_mean']:>+10.4f} {r['dphi_std']:>9.4f} "
              f"{r['rms_mag']:>7.0f}{mark}")

    # FCCH identification — thresholds relaxed (chunk 06 baseline was
    # dphi_std=0.14, dphi_mean=+1.55 ≠ exact +π/2 due to GMSK BT smearing
    # at burst edges. Real FCCH still floats clearly above non-FCCH
    # bursts in dphi_std ranking, just not always at <0.10).
    FCCH_STD_THRESH = 0.30        # was 0.10
    FCCH_MEAN_TOL   = 0.25        # was 0.10
    fcch = [r for r in rows
            if r["dphi_std"] < FCCH_STD_THRESH
            and abs(r["dphi_mean"] - EXPECT_DPHI) < FCCH_MEAN_TOL]
    fcch_mirror = [r for r in rows
                   if r["dphi_std"] < FCCH_STD_THRESH
                   and abs(r["dphi_mean"] + EXPECT_DPHI) < FCCH_MEAN_TOL]

    print()
    print("=" * 86)
    if fcch:
        print(f"✓ {len(fcch)} FCCH burst(s) detected (dphi_std<0.10, dphi_mean≈+π/2)")
        mods = sorted(set(r["internal_fn_mod51"] for r in fcch))
        ifns = sorted(r["internal_fn"] for r in fcch)
        spacings = [ifns[i+1] - ifns[i] for i in range(len(ifns) - 1)]
        print(f"  internal_fn list  : {ifns}")
        print(f"  internal_fn % 51  : {mods}")
        print(f"  spacings (frames) : {spacings}")
        # FCCH on-air = fn%51 ∈ {0,10,20,30,40} (combined CCCH+SDCCH8)
        # → if all internal_fn_mod51 == X for one value, X is the "0" position
        if len(mods) == 1:
            X = mods[0]
            print(f"  → X = {X} (single mod51 value, suggests one FCCH per 51 frames)")
        else:
            # multiple : align with {0,10,20,30,40} pattern
            best_offset = None
            best_score = -1
            for off in range(51):
                shifted = {(m - off) % 51 for m in mods}
                expected = {0, 10, 20, 30, 40}
                score = len(shifted & expected)
                if score > best_score:
                    best_score = score
                    best_offset = off
            print(f"  → best offset X = {best_offset} "
                  f"(matches {best_score}/5 FCCH slots)")
            print(f"    Phase 1.5 rewrite : on-air_fn = internal_fn - {best_offset}")
    elif fcch_mirror:
        print(f"⚠ {len(fcch_mirror)} MIRROR FCCH (-π/2). Source signal has flipped phase.")
        print("  → I/Q swap or polarity inversion somewhere in osmo-trx or our forward.")
    else:
        print("✗ NO FCCH detected in 51 chunks.")
        print("  → Either osmo-trx isn't generating FCCH (BTS not transmitting yet, ")
        print("    filler stuck on 'zero', config issue), OR the format we read is wrong.")
        print(f"  Best (lowest) dphi_std = {rows[0]['dphi_std']:.3f} on idx={rows[0]['idx']} "
              f"(dphi_mean={rows[0]['dphi_mean']:+.4f}, peak={rows[0]['peak_freq']:+.0f}Hz)")


if __name__ == "__main__":
    main()
