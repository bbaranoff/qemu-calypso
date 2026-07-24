#!/usr/bin/env python3
"""FCCH burst diagnostic — FFT + dphi + RMS sur capture bi-directionnelle calypso-ipc-device.

Diag pour décider si FBSB_CONF=0 est dû à un modulateur GMSK cassé (calypso-ipc-device
soft_bits_to_gmsk_iq), à un scheduling BTS faux, ou à un bug DSP-side. Sortie =
verdict en 3 branches.

Usage :
    1. Patcher calypso-ipc-device : CALYPSO_FCCH_DUMP=1 ./run.sh
    2. Attendre le print "bridge: [FCCH-DUMP] one-shot capture fn=N..."
    3. python3 tools/fcch_analyze.py --fn N
       (ou --bits /tmp/fcch_bits_fnN.bin --iq /tmp/fcch_iq_fnN.bin)

Convention attendue (cf. calypso-ipc-device:soft_bits_to_gmsk_iq) :
- bits   : N octets, soft-bit osmocom (0=strong 0, 255=strong 1). FCCH = tous à 0.
- iq     : 4N octets cs16 entrelacés I,Q,I,Q (scale=4096, modulation BT=0.3).
- N      : 148 (burst NB) ou 142/146 si BTS truncate.

Signal théorique FCCH (148 bits zéro) :
- pic spectral unique à +fs/4 = +67708 Hz (fs = 270833 Hz symbol rate, 1 SPS).
- dphi = +π/2 ≈ +1.5708 rad/sample, constant, sans saut.
- magnitude IQ ≈ scale=4096 (cos²+sin² × scale² = scale²).

Verdict :
- bits != tout-zéro                         → BUG SCHEDULING BTS (le bridge reçoit autre chose qu'une FCCH)
- bits OK, dphi miroir (-π/2)               → BUG MODULATEUR (swap I/Q ou polarité bit, FCCH à -67.7 kHz, corrélateur cherche +67.7 → invisible à la FFT seule)
- bits OK, dphi |≠ π/2| > 5 %               → BUG MODULATEUR (mauvaise fréquence)
- bits OK, dphi(std) > 0.1                  → BUG MODULATEUR (phase break aux bords, mauvais BT, ou troncature)
- bits OK, dphi propre, RMS très bas        → BUG MODULATEUR (clip ou scale anormal)
- bits OK, dphi propre, RMS OK, len OK      → BUG DSP-SIDE (corrélateur/AGC/timing/framing window)
"""
import argparse
import math
import os
import sys

import numpy as np

FS_SYM = 270833.0  # GSM symbol rate (Hz)
EXPECTED_DPHI = math.pi / 2  # +π/2 rad/sample pour FCCH 148b zéro à 1 SPS
EXPECTED_FREQ = FS_SYM / 4   # +67708.25 Hz


def load(bits_path, iq_path):
    with open(bits_path, "rb") as f:
        bits = np.frombuffer(f.read(), dtype=np.uint8)
    with open(iq_path, "rb") as f:
        raw = np.frombuffer(f.read(), dtype=np.int16)
    if len(raw) % 2 != 0:
        sys.exit(f"ERR: IQ stream length {len(raw)} int16 not even (expect I,Q,I,Q)")
    iq = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    return bits, iq


def analyze_bits(bits):
    """Returns (ok, msg)."""
    n = len(bits)
    # FCCH = tout-zéro en soft-bit osmocom (0 = strong "0")
    nonzero = int(np.count_nonzero(bits))
    ratio = nonzero / n if n else 1.0
    if nonzero == 0:
        return True, f"len={n} all-zero ✓ (FCCH bits propres)"
    if ratio < 0.01:
        return True, f"len={n} nonzero={nonzero} ({ratio*100:.2f} %) ≈ all-zero ✓"
    return False, (f"len={n} nonzero={nonzero} ({ratio*100:.1f} %) ✗ "
                   f"PAS UNE FCCH PROPRE — première occurrence non-zéro idx={int(np.argmax(bits != 0))}")


def analyze_iq(iq):
    """FFT + dphi + RMS. Returns dict."""
    n = len(iq)
    # FFT
    fft = np.fft.fft(iq)
    freqs = np.fft.fftfreq(n, d=1.0 / FS_SYM)
    mag = np.abs(fft)
    peak_idx = int(np.argmax(mag))
    peak_freq = freqs[peak_idx]
    peak_mag = mag[peak_idx]
    # Noise floor = mediane des bins hors d'une fenêtre ±2 autour du pic
    excl = set(range(max(0, peak_idx - 2), min(n, peak_idx + 3)))
    other = [mag[i] for i in range(n) if i not in excl]
    noise_med = float(np.median(other)) if other else 0.0
    snr_db = 20 * math.log10(peak_mag / noise_med) if noise_med > 0 else float('inf')

    # dphi
    phase = np.unwrap(np.angle(iq))
    dphi = np.diff(phase)
    # Trim 2 samples chaque bord (artefacts troncature)
    if len(dphi) > 8:
        dphi_core = dphi[2:-2]
    else:
        dphi_core = dphi

    # RMS I, Q, magnitude
    rms_i = float(np.sqrt(np.mean(iq.real ** 2)))
    rms_q = float(np.sqrt(np.mean(iq.imag ** 2)))
    rms_mag = float(np.sqrt(np.mean(np.abs(iq) ** 2)))
    peak_abs = float(np.max(np.abs(iq)))

    return {
        "n_samples": n,
        "peak_freq": float(peak_freq),
        "peak_mag": float(peak_mag),
        "noise_med": noise_med,
        "snr_db": snr_db,
        "dphi_mean": float(np.mean(dphi_core)),
        "dphi_std": float(np.std(dphi_core)),
        "dphi_min": float(np.min(dphi_core)),
        "dphi_max": float(np.max(dphi_core)),
        "dphi_full": dphi,
        "rms_i": rms_i,
        "rms_q": rms_q,
        "rms_mag": rms_mag,
        "peak_abs": peak_abs,
    }


def verdict(bits_ok, bits_msg, st):
    print()
    print("============ VERDICT ============")
    if not bits_ok:
        print("BRANCHE 0 : BUG SCHEDULING BTS")
        print(f"  → {bits_msg}")
        print("  → Le bridge reçoit autre chose qu'une FCCH sur cette frame.")
        print("  → Modulateur innocent. Investiguer osmo-bts-trx scheduling.")
        return

    dphi_err = abs(st["dphi_mean"] - EXPECTED_DPHI) / EXPECTED_DPHI
    dphi_err_neg = abs(st["dphi_mean"] - (-EXPECTED_DPHI)) / EXPECTED_DPHI
    freq_err = abs(abs(st["peak_freq"]) - EXPECTED_FREQ) / EXPECTED_FREQ
    sign_neg = st["peak_freq"] < 0 or st["dphi_mean"] < 0

    if sign_neg and dphi_err_neg < 0.05:
        print("BRANCHE 2a : BUG MODULATEUR — MIROIR (signe inversé)")
        print(f"  → dphi_mean = {st['dphi_mean']:+.4f} rad/sample (attendu {EXPECTED_DPHI:+.4f})")
        print(f"  → peak_freq = {st['peak_freq']:+.0f} Hz (attendu {EXPECTED_FREQ:+.0f} Hz)")
        print("  → Spectre parfaitement propre mais à la fréquence opposée.")
        print("  → Corrélateur cherche +67.7 kHz, voit -67.7 kHz → FBSB_CONF=0.")
        print("  → Cause probable : swap I/Q ou polarité de bit dans soft_bits_to_gmsk_iq.")
        print("  → osmo-trx-ipc résout (génère FCCH au bon signe).")
        return

    if dphi_err > 0.05:
        print("BRANCHE 2b : BUG MODULATEUR — FRÉQUENCE FAUSSE")
        print(f"  → dphi_mean = {st['dphi_mean']:+.4f} rad/sample, attendu {EXPECTED_DPHI:+.4f} "
              f"(écart {dphi_err*100:.1f} %)")
        print(f"  → peak_freq = {st['peak_freq']:+.0f} Hz, attendu {EXPECTED_FREQ:+.0f} Hz "
              f"(écart {freq_err*100:.1f} %)")
        print("  → osmo-trx-ipc résout.")
        return

    if st["dphi_std"] > 0.10:
        print("BRANCHE 2c : BUG MODULATEUR — DISCONTINUITÉ DE PHASE")
        print(f"  → dphi_std = {st['dphi_std']:.4f} rad (attendu < 0.1)")
        print(f"  → dphi range = [{st['dphi_min']:+.4f}, {st['dphi_max']:+.4f}]")
        print("  → Saut de phase au milieu ou aux bords. Vérifier BT, taps gaussiens, troncature.")
        print("  → osmo-trx-ipc résout.")
        return

    # Scale check : modulateur écrit scale=4096 → magnitude ≈ 4096
    # (cos²+sin² = 1 × scale²) Tolérance large à cause du filtre gaussien
    # qui peut amortir un peu.
    expected_mag = 4096.0
    if st["rms_mag"] < expected_mag * 0.3:
        print("BRANCHE 2d : BUG MODULATEUR — AMPLITUDE TROP FAIBLE")
        print(f"  → rms_mag = {st['rms_mag']:.0f}, attendu ~{expected_mag:.0f} "
              f"({st['rms_mag']/expected_mag*100:.0f} %)")
        print("  → AGC du corrélateur DSP peut décrocher.")
        return
    if st["peak_abs"] > 32700:
        print("BRANCHE 2e : BUG MODULATEUR — CLIP (peak == int16 max)")
        print(f"  → peak_abs = {st['peak_abs']:.0f}, near int16 max 32767")
        print("  → Distorsion harmonique, FCCH polluée.")
        return

    print("BRANCHE 1 : SOURCE I/Q PROPRE — BUG DSP-SIDE")
    print(f"  → bits propres, dphi = {st['dphi_mean']:+.4f} (≈ {EXPECTED_DPHI:+.4f}), "
          f"std = {st['dphi_std']:.4f}")
    print(f"  → peak {st['peak_freq']:+.0f} Hz @ mag {st['peak_mag']:.0f}, SNR {st['snr_db']:.1f} dB")
    print(f"  → RMS_mag {st['rms_mag']:.0f} (scale 4096 attendu)")
    print("  → osmo-trx-ipc N'AIDERA PAS sur FBSB. Chasser dans DSP/BSP :")
    print("    - taille de la fenêtre corrélateur côté DSP (148 reçus vs N attendus ?)")
    print("    - placement temporel du burst dans la fenêtre BDLENA")
    print("    - AGC/seuil de confiance du corrélateur FB")
    print("    - timing TPU→TSP→IOTA BDLENA")
    print(f"  → Calibration target pour Phase 1 device : RMS_mag = {st['rms_mag']:.0f}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fn", type=int, help="Charge /tmp/fcch_{bits,iq}_fn<FN>.bin")
    p.add_argument("--bits", help="Path explicit bits file (sinon dérivé de --fn)")
    p.add_argument("--iq",   help="Path explicit iq file (sinon dérivé de --fn)")
    args = p.parse_args()

    if args.fn is not None:
        bits_path = args.bits or f"/tmp/fcch_bits_fn{args.fn}.bin"
        iq_path   = args.iq   or f"/tmp/fcch_iq_fn{args.fn}.bin"
    else:
        if not args.bits or not args.iq:
            p.error("--fn N ou bien (--bits BITS_PATH --iq IQ_PATH)")
        bits_path, iq_path = args.bits, args.iq

    if not os.path.exists(bits_path):
        sys.exit(f"ERR: {bits_path} introuvable")
    if not os.path.exists(iq_path):
        sys.exit(f"ERR: {iq_path} introuvable")

    bits, iq = load(bits_path, iq_path)
    bits_ok, bits_msg = analyze_bits(bits)
    st = analyze_iq(iq)

    print(f"=== FCCH burst analysis ===")
    print(f"bits file : {bits_path}")
    print(f"iq file   : {iq_path}")
    print()
    print(f"--- Bits ---")
    print(f"  {bits_msg}")
    print()
    print(f"--- IQ samples ---")
    print(f"  N samples       = {st['n_samples']}")
    print(f"  peak freq       = {st['peak_freq']:+.1f} Hz   (FCCH attendue {EXPECTED_FREQ:+.1f} Hz)")
    print(f"  peak magnitude  = {st['peak_mag']:.1f}")
    print(f"  noise median    = {st['noise_med']:.2f}")
    print(f"  SNR             = {st['snr_db']:.1f} dB")
    print(f"  dphi mean       = {st['dphi_mean']:+.5f} rad/sample   (attendu {EXPECTED_DPHI:+.5f})")
    print(f"  dphi std        = {st['dphi_std']:.5f} rad")
    print(f"  dphi min/max    = [{st['dphi_min']:+.5f}, {st['dphi_max']:+.5f}]")
    print(f"  rms I           = {st['rms_i']:.1f}")
    print(f"  rms Q           = {st['rms_q']:.1f}")
    print(f"  rms |IQ|        = {st['rms_mag']:.1f}")
    print(f"  peak |IQ|       = {st['peak_abs']:.1f}   (int16 max 32767)")

    verdict(bits_ok, bits_msg, st)


if __name__ == "__main__":
    main()
