"""
test_timer_invariants.py — tests sur les compteurs de timers instrumentés.

Cible les logs `[tdma]`, `[frame_irq]`, `[kick]` produits par
`calypso_trx.c` + le CSV `/tmp/log_timeline.csv` produit par `log_timeline.py`.

Invariants testés :
  - log [tdma] présent (rebuild fait, env CALYPSO_DSP_BUDGET respecté)
  - dsp_n_exec_* <= dsp_budget (sanity)
  - dsp saturation : si dsp_n_exec_5 == budget sur N ticks consécutifs → signal
  - period virtual entre [tdma] thinned = 1000 × 4.615 ms ± 2%
  - ratio frame_irq/tdma ≈ 1
  - table log_timeline.csv produite, pas de bucket totalement vide

Lancement :
    pytest -v -m timer_invariant
"""
from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

CONTAINER  = os.environ.get("CALYPSO_CONTAINER", "trying")
QEMU_LOG   = os.environ.get("CALYPSO_QEMU_LOG",  "/root/qemu.log")
CSV_PATH   = os.environ.get("CALYPSO_TIMELINE_CSV", "/tmp/log_timeline.csv")
# Marqueur Docker standard : présent dans container, absent sur host.
INSIDE = os.path.exists("/.dockerenv")

TDMA_RE = re.compile(
    r'\[tdma\] tick #(\d+) fn=(\d+) t_virt=(\d+) '
    r'dsp_n_exec_2=(-?\d+) dsp_n_exec_5=(-?\d+) '
    r'dsp_insn_total=(\d+) budget=(\d+)'
)
FIRQ_RE = re.compile(r'\[frame_irq\] lower #(\d+) t_virt=(\d+)')
KICK_RE = re.compile(r'\[kick\] fire #(\d+) vt=(\d+) rt=(\d+)')

GSM_TDMA_NS = 4_615_000


def _read_lines(path: str, tail_n: int = 100000) -> list[str]:
    if INSIDE:
        try:
            with open(path, errors="replace") as f:
                return f.readlines()[-tail_n:]
        except FileNotFoundError:
            return []
    out = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-c",
         f"tail -n {tail_n} {path} 2>/dev/null"],
        capture_output=True, text=True, timeout=8)
    return out.stdout.splitlines(keepends=True)


def _parse_tdma() -> list[dict]:
    out = []
    for line in _read_lines(QEMU_LOG):
        m = TDMA_RE.search(line)
        if not m: continue
        out.append({
            "tick":          int(m.group(1)),
            "fn":            int(m.group(2)),
            "t_virt_ns":     int(m.group(3)),
            "dsp_n_exec_2":  int(m.group(4)),
            "dsp_n_exec_5":  int(m.group(5)),
            "dsp_insn_total": int(m.group(6)),
            "budget":        int(m.group(7)),
        })
    return out


def _parse_firq() -> list[dict]:
    out = []
    for line in _read_lines(QEMU_LOG):
        m = FIRQ_RE.search(line)
        if not m: continue
        out.append({"n": int(m.group(1)), "t_virt_ns": int(m.group(2))})
    return out


def _parse_kick() -> list[dict]:
    out = []
    for line in _read_lines(QEMU_LOG):
        m = KICK_RE.search(line)
        if not m: continue
        out.append({"n": int(m.group(1)),
                    "vt": int(m.group(2)),
                    "rt": int(m.group(3))})
    return out


@pytest.fixture(scope="module")
def tdma_samples():
    s = _parse_tdma()
    if not s:
        pytest.skip(f"no [tdma] log lines in {QEMU_LOG} — rebuild QEMU pas appliqué ?")
    return s

@pytest.fixture(scope="module")
def firq_samples():
    s = _parse_firq()
    if not s:
        pytest.skip("no [frame_irq] log lines")
    return s

@pytest.fixture(scope="module")
def kick_samples():
    s = _parse_kick()
    if not s:
        pytest.skip("no [kick] log lines")
    return s


# -----------------------------------------------------------------------------
# Tests timers
# -----------------------------------------------------------------------------
@pytest.mark.timer_invariant
def test_tdma_log_present(tdma_samples):
    """Au moins 2 lignes [tdma] (= 2000+ ticks observés)."""
    assert len(tdma_samples) >= 2, \
        f"only {len(tdma_samples)} [tdma] entries — wait for 2000+ ticks (~9s virtual)"


@pytest.mark.timer_invariant
def test_dsp_n_exec_within_budget(tdma_samples):
    """dsp_n_exec_2 et dsp_n_exec_5 doivent rester <= budget."""
    for s in tdma_samples:
        assert 0 <= s["dsp_n_exec_2"] <= s["budget"], \
            f"tick #{s['tick']}: dsp_n_exec_2={s['dsp_n_exec_2']} hors [0, {s['budget']}]"
        assert 0 <= s["dsp_n_exec_5"] <= s["budget"], \
            f"tick #{s['tick']}: dsp_n_exec_5={s['dsp_n_exec_5']} hors [0, {s['budget']}]"


@pytest.mark.timer_invariant
def test_dsp_budget_saturation_signal(tdma_samples):
    """Si dsp_n_exec_5 == budget sur les 3 dernières entries → DSP saturé,
    XFAIL avec message — c'est un état observable, pas un bug du test."""
    if len(tdma_samples) < 3:
        pytest.skip("need >= 3 samples")
    last3 = tdma_samples[-3:]
    saturated = all(s["dsp_n_exec_5"] == s["budget"] for s in last3)
    if saturated:
        pytest.xfail(
            f"DSP saturé: dsp_n_exec_5 == budget ({last3[0]['budget']}) sur "
            f"les 3 derniers [tdma] ticks. Descendre CALYPSO_DSP_BUDGET cassera "
            f"probablement la fb-det convergence. Voir REPORT_CLAUDE_WEB_20260516_DSP_OVERRUN.md.")
    # Else: pass — DSP non saturé en steady state
    assert True


@pytest.mark.timer_invariant
def test_tdma_period_virtual_close_to_target(tdma_samples):
    """Période virtual entre 2 [tdma] thinned (1000 ticks) ≈ 4.615 s ± 2%."""
    if len(tdma_samples) < 2:
        pytest.skip("need >= 2 samples")
    dt_virt_ns = tdma_samples[-1]["t_virt_ns"] - tdma_samples[-2]["t_virt_ns"]
    dtick = tdma_samples[-1]["tick"] - tdma_samples[-2]["tick"]
    if dtick != 1000:
        pytest.skip(f"non-1000-tick gap: {dtick}")
    expected_ns = 1000 * GSM_TDMA_NS
    rel_err = abs(dt_virt_ns - expected_ns) / expected_ns
    assert rel_err <= 0.02, \
        f"period_virtual = {dt_virt_ns:,} ns vs target {expected_ns:,} (err {rel_err*100:.2f}%)"


@pytest.mark.timer_invariant
def test_frame_irq_per_tdma_ratio(tdma_samples, firq_samples):
    """Ratio frame_irq/tdma ≈ 1 (le frame_irq lower fire à chaque tdma_tick)."""
    if not firq_samples:
        pytest.skip("no firq samples")
    n_tdma = tdma_samples[-1]["tick"]
    n_firq = firq_samples[-1]["n"]
    if n_tdma == 0:
        pytest.skip()
    ratio = n_firq / n_tdma
    # Tolérance large : 0.5..1.5 (frame_irq peut ne pas fire au boot avant que
    # le hardware-frame-arm soit set par firmware)
    assert 0.5 <= ratio <= 1.5, \
        f"frame_irq/tdma = {ratio:.2f} (firq={n_firq} / tdma={n_tdma})"


@pytest.mark.timer_invariant
def test_kick_realtime_cadence(kick_samples):
    """Kick fires sur REALTIME ~5 ms. Délta moyen entre 2 logs thinned 1/200
    devrait être ~1 s wall = ~1e9 ns realtime."""
    if len(kick_samples) < 3:
        pytest.skip("few kick samples")
    rts = [s["rt"] for s in kick_samples]
    deltas = [(rts[i+1] - rts[i]) for i in range(len(rts)-1)]
    mean = sum(deltas) / len(deltas)
    # 200 fires × 5 ms = 1 s = 1e9 ns. Tolérance ±50%.
    assert 0.5e9 <= mean <= 1.5e9, \
        f"kick realtime mean delta = {mean/1e9:.2f}s, expected ~1s (1e9 ns)"


# -----------------------------------------------------------------------------
# Tests sur la table log_timeline.csv
# -----------------------------------------------------------------------------
def _has_docker(): return shutil.which("docker") is not None

def _csv_exists() -> tuple[bool, str]:
    """(exists, where) — check CSV side : local ou container."""
    if Path(CSV_PATH).exists():
        return (True, "local")
    if _has_docker():
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "test", "-f", CSV_PATH],
            capture_output=True, timeout=4)
        if r.returncode == 0:
            return (True, "container")
    return (False, "none")

def _generate_csv() -> str:
    """Tente de produire le CSV. Priorité : container si docker dispo
    (qemu.log canonique côté container), sinon local SEULEMENT si in-container.
    Évite la race où on lit un /root/qemu.log host stale.
    """
    # 1) Container path (priorité) — sauf si on EST dans le container
    if _has_docker() and not INSIDE:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "python3",
             "/opt/GSM/qemu-src/log_timeline.py",
             "--bucket-s", "10", "--csv", CSV_PATH],
            capture_output=True, text=True, timeout=20)
        return r.stderr or r.stdout[:200]
    # 2) Local (in-container : /root/qemu.log est canonique)
    script_local = Path(__file__).resolve().parent.parent / "log_timeline.py"
    if script_local.exists() and Path("/root/qemu.log").exists():
        r = subprocess.run(
            ["python3", str(script_local), "--bucket-s", "10", "--csv", CSV_PATH],
            capture_output=True, text=True, timeout=20)
        return r.stderr or r.stdout[:200]
    return "no path to run log_timeline.py"

@pytest.mark.timer_invariant
def test_log_timeline_csv_produced():
    """log_timeline.py doit avoir produit le CSV (local ou container).
    Si absent, tente une génération à la volée puis re-check."""
    ok, where = _csv_exists()
    if ok:
        return
    err = _generate_csv()
    ok, where = _csv_exists()
    assert ok, f"{CSV_PATH} non produit (local={Path(CSV_PATH).exists()}, docker={_has_docker()}, err={err[:200]})"


def _read_csv_rows() -> list[dict]:
    if INSIDE:
        try:
            with open(CSV_PATH) as f:
                return list(csv.DictReader(f))
        except FileNotFoundError:
            return []
    out = subprocess.run(
        ["docker", "exec", CONTAINER, "cat", CSV_PATH],
        capture_output=True, text=True, timeout=5)
    return list(csv.DictReader(out.stdout.splitlines()))


@pytest.mark.timer_invariant
def test_log_timeline_csv_no_dead_bucket():
    """Aucun bucket avec 0 events all-sources confondus (= run pas stallé)."""
    rows = _read_csv_rows()
    if not rows:
        pytest.skip("CSV empty")
    dead = []
    sources = ["qemu", "bridge", "osmocon", "mobile"]
    for r in rows:
        total = sum(int(r.get(s, 0) or 0) for s in sources)
        if total == 0:
            dead.append(r["t_rel"])
    assert not dead, f"buckets totalement vides à t_rel = {dead[:5]} (run stallé ?)"


@pytest.mark.timer_invariant
def test_log_timeline_csv_steady_qemu_rate():
    """qemu rate par bucket doit rester dans une fenêtre raisonnable
    (pas de spike 10× ni de chute 10× sur la majorité des buckets)."""
    rows = _read_csv_rows()
    if len(rows) < 5:
        pytest.skip("too few buckets")
    qemu_rates = [int(r.get("qemu", 0) or 0) for r in rows[1:]]  # skip warmup
    if not qemu_rates: pytest.skip("no qemu rates")
    mean = sum(qemu_rates) / len(qemu_rates)
    outliers = [v for v in qemu_rates if v > 10*mean or v < mean/10]
    # Tolère jusqu'à 20% d'outliers
    assert len(outliers) <= 0.2 * len(qemu_rates), \
        f"{len(outliers)}/{len(qemu_rates)} buckets outliers (>10× ou <1/10 mean={mean:.0f})"
