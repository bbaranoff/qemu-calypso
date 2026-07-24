"""Doppler / AFC rotation for GMSK I/Q samples.

Modèle hardware : VCO mobile shifte la baseband selon AFC trim. Bridge/
osmo-trx-ipc fournissent des samples GMSK "parfaits" (pas de drift osc) →
DSP correlator donne un résidu non-zéro qui ne converge jamais (firmware
applique AFC → samples inchangés → même résidu → boucle).

Cette routine injecte une rotation phase :
  sample[i] *= exp(j * 2π * doppler * i / fs)

Utilisé soit pour modéliser un vrai Doppler (CALYPSO_DOPPLER_HZ != 0),
soit pour réfléter la correction AFC firmware (sample[i] *= exp(-j*afc)).

Env vars :
  CALYPSO_DOPPLER_HZ  Hz de drift injecté (default 0)
  CALYPSO_SAMPLE_RATE Hz sample rate (default 270833 = 1 sps GSM)

Import-only API :
  apply_doppler(iq_bytes, doppler_hz=None, sample_rate=None, start_phase=0.0)
    -> rotated iq_bytes (même format int16 interleaved I,Q,I,Q,...)
"""
import math
import os

DOPPLER_HZ_DEFAULT = float(os.environ.get("CALYPSO_DOPPLER_HZ", "0"))
SAMPLE_RATE_DEFAULT = float(os.environ.get("CALYPSO_SAMPLE_RATE", "270833"))


def apply_doppler(iq_bytes, doppler_hz=None, sample_rate=None,
                  start_phase=0.0):
    """Tourne des samples int16 interleaved I,Q par exp(j*2π*f*i/fs).

    iq_bytes      : bytes contenant 2N int16 little-endian (I,Q,I,Q,...).
    doppler_hz    : décalage fréq Hz. None => env CALYPSO_DOPPLER_HZ.
    sample_rate   : Hz. None => env CALYPSO_SAMPLE_RATE (= 270833).
    start_phase   : phase initiale rad (continuité inter-bursts).

    Returns rotated iq_bytes (même longueur, même format).
    Si doppler=0 : retourne iq_bytes inchangé (no-op).
    """
    f = DOPPLER_HZ_DEFAULT if doppler_hz is None else float(doppler_hz)
    if f == 0.0:
        return iq_bytes
    fs = SAMPLE_RATE_DEFAULT if sample_rate is None else float(sample_rate)

    try:
        import numpy as np
    except ImportError:
        return iq_bytes  # silent fallback si numpy absent

    samples = np.frombuffer(iq_bytes, dtype=np.int16)
    n = len(samples) // 2
    if n == 0:
        return iq_bytes
    i_arr = samples[0::2].astype(np.float64)
    q_arr = samples[1::2].astype(np.float64)

    phase_step = 2.0 * math.pi * f / fs
    phases = start_phase + phase_step * np.arange(n)
    cos_p = np.cos(phases)
    sin_p = np.sin(phases)

    new_i = i_arr * cos_p - q_arr * sin_p
    new_q = i_arr * sin_p + q_arr * cos_p

    out = np.empty(2 * n, dtype=np.int16)
    out[0::2] = np.clip(new_i, -32768, 32767).astype(np.int16)
    out[1::2] = np.clip(new_q, -32768, 32767).astype(np.int16)
    return out.tobytes()


def next_phase(prev_phase, n_samples, doppler_hz=None, sample_rate=None):
    """Calcule la phase continue après n_samples (pour chainage bursts).

    Retourne la phase à utiliser comme start_phase du prochain burst.
    """
    f = DOPPLER_HZ_DEFAULT if doppler_hz is None else float(doppler_hz)
    if f == 0.0:
        return 0.0
    fs = SAMPLE_RATE_DEFAULT if sample_rate is None else float(sample_rate)
    phase_step = 2.0 * math.pi * f / fs
    return (prev_phase + phase_step * n_samples) % (2.0 * math.pi)


# Print env :
#   CALYPSO_BURST_PRINT  = 0/1     active dump (default 0)
#   CALYPSO_BURST_MAX    = N       cap nombre de dumps (default 50)
#   CALYPSO_BURST_HEAD   = N       montre N premiers I,Q (default 12)
_BURST_PRINT = os.environ.get("CALYPSO_BURST_PRINT", "0") == "1"
_BURST_MAX = int(os.environ.get("CALYPSO_BURST_MAX", "50"))
_BURST_HEAD = int(os.environ.get("CALYPSO_BURST_HEAD", "12"))
_burst_count = 0


def dump_burst(iq_bytes, tag="", fn=None, tn=None):
    """Pretty-print un burst I/Q pour diag GMSK.

    Affiche les N premiers samples I/Q en signed int16, plus stats globales
    (énergie, mean phase). Capped à CALYPSO_BURST_MAX dumps pour pas
    inonder le log.

    Env-gated par CALYPSO_BURST_PRINT=1.
    """
    global _burst_count
    if not _BURST_PRINT:
        return
    if _burst_count >= _BURST_MAX:
        return
    _burst_count += 1

    try:
        import numpy as np
    except ImportError:
        return

    samples = np.frombuffer(iq_bytes, dtype=np.int16)
    n = len(samples) // 2
    if n == 0:
        return
    i_arr = samples[0::2].astype(np.float64)
    q_arr = samples[1::2].astype(np.float64)

    energy = float(np.mean(i_arr * i_arr + q_arr * q_arr))
    rms = math.sqrt(energy)
    mag = np.sqrt(i_arr * i_arr + q_arr * q_arr)
    nonzero = int(np.sum(mag > 0))

    # Mean phase progression per sample (= AFC residue ~ Doppler)
    if n >= 2:
        ph = np.unwrap(np.arctan2(q_arr, i_arr))
        dph = np.diff(ph)
        mean_dph = float(np.mean(dph))
        # Convert rad/sample → Hz (fs = 270833)
        mean_freq_hz = mean_dph * SAMPLE_RATE_DEFAULT / (2.0 * math.pi)
    else:
        mean_dph = 0.0
        mean_freq_hz = 0.0

    head = min(_BURST_HEAD, n)
    pairs = " ".join(f"({i_arr[k]:+6.0f},{q_arr[k]:+6.0f})" for k in range(head))

    suffix_parts = []
    if tag:
        suffix_parts.append(tag)
    if fn is not None:
        suffix_parts.append(f"fn={fn}")
    if tn is not None:
        suffix_parts.append(f"tn={tn}")
    suffix = " ".join(suffix_parts)

    print(
        f"[doppler] BURST #{_burst_count} {suffix} "
        f"n={n} rms={rms:.0f} nonzero={nonzero} "
        f"dphi/sample={mean_dph:+.4f}rad freq={mean_freq_hz:+.0f}Hz "
        f"head=[{pairs}]",
        flush=True,
    )
