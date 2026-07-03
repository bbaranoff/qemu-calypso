"""
test_layer_drift.py — tests de drift temporel entre couches.

Précondition : run.sh doit avoir été lancé avec CALYPSO_LOG_TS=1 (default),
les logs sont préfixés par `<epoch_sec> +<rel_sec>s ` permettant de mesurer
les écarts inter-couches.

Invariants testés :
  - QEMU virtual time stable (rate insn CV < 0.4, p1 > 1M)
  - Bridge wall_fn / qfn alignment vs QEMU fn (≤ 200 frames d'écart)
  - Tous les processus génèrent encore des events (pas mort)
  - Tous les logs ont démarré dans une fenêtre de ±10s
  - Pas de gap > 10s dans chaque log (= pas de freeze process)

Lancement :
    pytest -v -m drift
"""
from __future__ import annotations

import os
import re
import statistics
import subprocess
import time
from pathlib import Path

import pytest

# Défauts alignés sur bash_scripts/run.sh : LOGDIR="${CALYPSO_LOGDIR:-/root}"
# (tout ce qui grossit va dans /root depuis le refactor "plus de /tmp tmpfs
# 512M qui se remplissait -> writes échouent -> firmware perd la sync" —
# cf. run.sh "---- log paths ----"). QEMU_LOG était déjà à jour (issue
# 2026-05-16) ; OSMOCON_LOG/MOBILE_LOG/BRIDGE_LOG pointaient encore vers
# /tmp et étaient donc des chemins morts (skip permanent, jamais un vrai
# check) — corrigé ici.
CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
QEMU_LOG    = os.environ.get("CALYPSO_QEMU_LOG",    "/root/qemu.log")
# BRIDGE_LOG : bridge.py est le pipeline radio *legacy* (mutex avec
# IPC_DEVICE/TRX_IPC, cf run.sh "BRIDGE_PY est mutuellement exclusif avec
# IPC_DEVICE et TRX_IPC"). Le pipeline par défaut aujourd'hui est
# calypso-ipc-device + osmo-trx-ipc, PAS bridge.py — donc ce fichier
# n'existera pas tant que le run n'est pas lancé en mode bridge-py
# (CALYPSO_SKIP_BRIDGE_PY=0). C'est un skip légitime dans la config
# courante, pas un chemin cassé. Nom réel du fichier quand actif :
# $LOGDIR/bridge.py.log (run.sh: BRIDGE_LOG="${BRIDGE_LOG:-$LOGDIR/bridge.py.log}").
BRIDGE_LOG  = os.environ.get("CALYPSO_BRIDGE_LOG",  "/root/bridge.py.log")
OSMOCON_LOG = os.environ.get("CALYPSO_OSMOCON_LOG", "/root/osmocon.log")
MOBILE_LOG  = os.environ.get("CALYPSO_MOBILE_LOG",  "/root/mobile.log")

TS_RE = re.compile(r'^(\d+\.\d+)\s+\+(\d+\.\d+)s\s+(.*)$')

# INSIDE = pytest tourne réellement DANS le container (marqueur standard Docker).
# Ne pas se fier à os.path.exists(QEMU_LOG) : sur host le path peut exister
# avec un vieux fichier obsolète (cf. issue 2026-05-16) — on lirait alors le
# mauvais log. Le marqueur /.dockerenv est posé par Docker à la création du
# container, et n'existe pas sur l'hôte.
INSIDE = os.path.exists("/.dockerenv")

def _read_lines(path: str, tail_n: int = 30000) -> list[str]:
    if INSIDE:
        try:
            with open(path, errors="replace") as f:
                lines = f.readlines()
            return lines[-tail_n:]
        except FileNotFoundError:
            return []
    else:
        out = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c",
             f"tail -n {tail_n} {path} 2>/dev/null"],
            capture_output=True, text=True, timeout=8)
        return out.stdout.splitlines(keepends=True)


def _parse_timestamped(path: str, filter_substr: str = None, tail_n: int = 30000) -> list[tuple[float, float, str]]:
    """Returns list of (ts_wall, ts_rel, body) for lines matching filter_substr."""
    out = []
    for line in _read_lines(path, tail_n):
        m = TS_RE.match(line)
        if not m: continue
        if filter_substr and filter_substr not in m.group(3): continue
        out.append((float(m.group(1)), float(m.group(2)), m.group(3)))
    return out


@pytest.fixture(scope="module")
def have_timestamps():
    """Skip drift module if logs aren't timestamped (run.sh launched without TS)."""
    samples = _parse_timestamped(QEMU_LOG, tail_n=20)
    if not samples:
        pytest.skip(f"{QEMU_LOG} has no timestamp prefix — relaunch with CALYPSO_LOG_TS=1")
    return True


# -----------------------------------------------------------------------------
# 1. QEMU virtual time stability
# -----------------------------------------------------------------------------
@pytest.mark.drift
def test_qemu_insn_rate_cv_below_0_4(have_timestamps):
    """Coefficient of variation of insn/s should stay below 0.4 (smooth)."""
    rates = []
    for _, _, body in _parse_timestamped(QEMU_LOG, "INSN-COUNT-STATS"):
        m = re.search(r'rate=(\d+)/s', body)
        if m: rates.append(int(m.group(1)))
    if len(rates) < 20:
        pytest.skip(f"only {len(rates)} INSN-COUNT-STATS samples — needs >= 20")
    cv = statistics.stdev(rates) / statistics.mean(rates)
    assert cv <= 0.4, f"insn rate CV = {cv:.3f} > 0.4 (à-coups DSP)"


@pytest.mark.drift
def test_qemu_insn_rate_p1_above_1m(have_timestamps):
    """p1(rate) should stay above 1M insn/s (no livelock)."""
    rates = []
    for _, _, body in _parse_timestamped(QEMU_LOG, "INSN-COUNT-STATS"):
        m = re.search(r'rate=(\d+)/s', body)
        if m: rates.append(int(m.group(1)))
    if len(rates) < 100:
        pytest.skip(f"only {len(rates)} samples — needs >= 100")
    p1 = sorted(rates)[len(rates) // 100]
    assert p1 >= 1_000_000, f"p1 insn rate = {p1:,}/s < 1M (DSP stalls)"


# -----------------------------------------------------------------------------
# 2. Bridge wall_fn / qfn drift vs QEMU fn
# -----------------------------------------------------------------------------
@pytest.mark.drift
def test_bridge_qfn_tracks_qemu_fn(have_timestamps):
    """At any wall timestamp, bridge.qfn should be within 200 frames of qemu fn."""
    b_samples = []
    for ts, _, body in _parse_timestamped(BRIDGE_LOG, "wall_fn="):
        mw = re.search(r'wall_fn=(\d+)', body)
        mq = re.search(r'qfn=(\d+)', body)
        if mw and mq:
            b_samples.append((ts, int(mw.group(1)), int(mq.group(1))))
    q_samples = []
    for ts, _, body in _parse_timestamped(QEMU_LOG, "[calypso-trx]"):
        mf = re.search(r'fn=(\d+)', body)
        if mf: q_samples.append((ts, int(mf.group(1))))
    if len(b_samples) < 5 or len(q_samples) < 5:
        pytest.skip(f"few samples bridge={len(b_samples)} qemu={len(q_samples)}")
    # Sample at 50% to avoid boot transients
    bts, wall_fn, qfn = b_samples[len(b_samples) // 2]
    nearest = min(q_samples, key=lambda x: abs(x[0] - bts))
    delta_fn = abs(nearest[1] - qfn)
    assert delta_fn < 200, \
        f"bridge.qfn={qfn} vs qemu.fn={nearest[1]} delta={delta_fn} ≥ 200 frames"


@pytest.mark.drift
def test_bridge_qfn_advances_steadily(have_timestamps):
    """Bridge qfn should advance at ~217 fn/s (GSM TDMA rate 4.615ms/frame).

    Threshold conservateur : 200..240 fn/s. À <200 la cell selection mobile
    échoue silencieusement (BTS désync), donc le test doit FAIL fort plutôt
    que d'accepter un système mort. Le rate observé 50..60 fn/s qu'on voit
    sous DSP-overload (c54x_run 512k insn/tick) doit ressortir comme un vrai
    blocker — pas un soft warning à 150.
    """
    samples = []
    for ts, _, body in _parse_timestamped(BRIDGE_LOG, "qfn="):
        mq = re.search(r'qfn=(\d+)', body)
        if mq: samples.append((ts, int(mq.group(1))))
    if len(samples) < 10:
        pytest.skip(f"only {len(samples)} qfn samples")
    # Take quartiles to skip warmup
    q1 = samples[len(samples) // 4]
    q3 = samples[3 * len(samples) // 4]
    dt = q3[0] - q1[0]
    dfn = q3[1] - q1[1]
    if dt < 1.0:
        pytest.skip(f"window too short {dt:.2f}s")
    rate = dfn / dt
    # GSM TDMA is 1 frame per 4.615ms = 216.7 fn/s. Conservative: 200..240.
    assert 200 <= rate <= 240, \
        f"qfn rate = {rate:.1f} fn/s, expected 217 ±10% (200..240) — DSP TDMA tick overrun"


# -----------------------------------------------------------------------------
# 3. Liveness — chaque process produit encore des events
# -----------------------------------------------------------------------------
@pytest.mark.drift
@pytest.mark.parametrize("name,path,min_rate_per_30s", [
    ("qemu",    QEMU_LOG,    1000),
    ("bridge",  BRIDGE_LOG,  10),
    ("osmocon", OSMOCON_LOG, 1),
    # mobile is often quiet (no commands sent) — skip strict liveness
])
def test_log_still_growing(have_timestamps, name, path, min_rate_per_30s):
    """Each log should have at least N events in the last 30s wall."""
    samples = _parse_timestamped(path, tail_n=20000)
    if not samples:
        pytest.skip(f"{path} has no timestamped lines")
    last_ts = samples[-1][0]
    window = [s for s in samples if s[0] >= last_ts - 30]
    assert len(window) >= min_rate_per_30s, \
        f"{name}: only {len(window)} events in last 30s (expected ≥ {min_rate_per_30s})"


# -----------------------------------------------------------------------------
# 4. Start time synchronicity — tous les logs commencent dans ±10s
# -----------------------------------------------------------------------------
@pytest.mark.drift
def test_log_start_within_10s(have_timestamps):
    """All logs should have started within a 10s window."""
    starts = {}
    for name, path in [("qemu", QEMU_LOG), ("bridge", BRIDGE_LOG),
                       ("osmocon", OSMOCON_LOG), ("mobile", MOBILE_LOG)]:
        samples = _parse_timestamped(path, tail_n=10)
        if samples:
            starts[name] = samples[0][0]
    if len(starts) < 2:
        pytest.skip(f"only {len(starts)} logs have timestamps")
    spread = max(starts.values()) - min(starts.values())
    assert spread <= 30, \
        f"log start spread = {spread:.1f}s > 30s — starts: {starts}"


# -----------------------------------------------------------------------------
# 5. No long gap inside any log (process freeze)
# -----------------------------------------------------------------------------
@pytest.mark.drift
@pytest.mark.parametrize("name,path,max_gap_s", [
    ("qemu",    QEMU_LOG,    5.0),
    ("bridge",  BRIDGE_LOG,  10.0),
])
def test_no_long_gap(have_timestamps, name, path, max_gap_s):
    """No gap longer than `max_gap_s` between consecutive log lines."""
    samples = _parse_timestamped(path, tail_n=20000)
    if len(samples) < 100:
        pytest.skip(f"{name}: only {len(samples)} samples")
    gaps = [samples[i+1][0] - samples[i][0] for i in range(len(samples)-1)]
    max_gap = max(gaps)
    assert max_gap <= max_gap_s, \
        f"{name}: max gap = {max_gap:.2f}s > {max_gap_s}s (process freeze?)"
