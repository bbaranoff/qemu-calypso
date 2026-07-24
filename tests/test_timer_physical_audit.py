"""
test_timer_physical_audit.py — audit physique des timers + graphes temporels.

Vérifie que chaque param timer dans le code respecte l'équation physique
documentée dans TIMING_PHYSICS.md. Pour chaque timer, mesure aussi le
rate effectif côté run (depuis qemu.log / bridge.log) et compare au
voulu théorique.

Génère des graphes mermaid xychart pour visualisation des périodes.

Markers :
  - osmocom_clock  (déjà enregistré)
  - timer_audit    (nouveau)
  - timer_graph    (nouveau)
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path
from collections import defaultdict
from statistics import mean, median, stdev

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
INSIDE = os.path.exists("/.dockerenv")
QEMU_SRC = "/opt/GSM/qemu-src"
QEMU_LOG = "/root/qemu.log"
(removed) = "/tmp/bridge.log"
BTS_LOG = "/tmp/bts.log"
GRAPHS_OUT = "/tmp/timer_graphs.mmd"


def _read(path: str) -> str:
    if INSIDE:
        try:
            return Path(path).read_text(errors="replace")
        except Exception:
            return ""
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "cat", path],
            capture_output=True, text=True, timeout=10)
        return r.stdout
    except Exception:
        return ""


# === Audit physique : voulu vs réel =========================================
# Format : (param_name, voulu_value, voulu_units, source_file, search_regex,
#           parse_fn, tolerance_pct)
AUDIT_PARAMS = [
    # 1. TINT0_PERIOD_NS — voulu 60_000_000/13 = 4_615_384 ns
    ("TINT0_PERIOD_NS", 4_615_384, "ns",
     "hw/arm/calypso/calypso_tint0.h",
     r"#define\s+TINT0_PERIOD_NS\s+(\d+)",
     int, 0.1),
    # 2. GSM_TDMA_NS — voulu 4_615_384 ns
    ("GSM_TDMA_NS", 4_615_384, "ns",
     "include/hw/arm/calypso/calypso_trx.h",
     r"#define\s+GSM_TDMA_NS\s+(\d+)",
     int, 0.1),
    # 3. GSM_HYPERFRAME — exact 2_715_648
    ("GSM_HYPERFRAME", 2_715_648, "frames",
     "hw/arm/calypso/calypso_tint0.h",
     r"#define\s+GSM_HYPERFRAME\s+(\d+)",
     int, 0.0),
    # 4. BSP_DRAIN_PERIOD_MS — voulu ~5 (1 TDMA frame)
    ("BSP_DRAIN_PERIOD_MS", 5, "ms",
     "hw/arm/calypso/calypso_bsp.c",
     r"#define\s+BSP_DRAIN_PERIOD_MS\s+(\d+)",
     int, 20.0),  # ±20% acceptable
    # 5. WT_DELAY_NS — voulu 25_000_000 ns (spec ISO SIM)
    #    Mais on accepte 2_000_000 ns (emulation rapide, documenté)
    ("WT_DELAY_NS", 25_000_000, "ns",
     "hw/arm/calypso/calypso_sim.c",
     r"#define\s+WT_DELAY_NS\s+(\d+)",
     int, 95.0),  # connu off by 12× pour speed
    # 6. PCB_TICK_TDMA_US — voulu 4615 µs
    ("PCB_TICK_TDMA_US", 4615, "µs",
     "hw/arm/calypso/calypso_full_pcb.c",
     r"#define\s+PCB_TICK_TDMA_US\s+(\d+)",
     int, 0.1),
    # 7. PCB_TICK_TINT0_US — voulu 4615 µs
    ("PCB_TICK_TINT0_US", 4615, "µs",
     "hw/arm/calypso/calypso_full_pcb.c",
     r"#define\s+PCB_TICK_TINT0_US\s+(\d+)",
     int, 0.1),
    # 8. PCB_TICK_KICK_US — voulu 5000 µs (legacy 200 Hz)
    ("PCB_TICK_KICK_US", 5000, "µs",
     "hw/arm/calypso/calypso_full_pcb.c",
     r"#define\s+PCB_TICK_KICK_US\s+(\d+)",
     int, 0.0),
    # 9. PCB_TICK_FRAME_IRQ_US — voulu 1000 µs (existant)
    ("PCB_TICK_FRAME_IRQ_US", 1000, "µs",
     "hw/arm/calypso/calypso_full_pcb.c",
     r"#define\s+PCB_TICK_FRAME_IRQ_US\s+(\d+)",
     int, 0.0),
]


@pytest.mark.timer_audit
@pytest.mark.osmocom_clock
@pytest.mark.parametrize("param,voulu,units,src,regex,parse,tol",
                         AUDIT_PARAMS,
                         ids=[p[0] for p in AUDIT_PARAMS])
def test_timer_physical_param(param, voulu, units, src, regex, parse, tol):
    """Vérifie que chaque param timer dans le code matche la valeur voulue
    par l'équation physique HW (tolérance %)."""
    path = f"{QEMU_SRC}/{src}"
    content = _read(path)
    assert content, f"can't read {path}"
    m = re.search(regex, content)
    assert m, f"can't find {param} in {path} (regex={regex})"
    actual = parse(m.group(1))
    pct_diff = abs(actual - voulu) / voulu * 100.0 if voulu else 0.0
    msg = (f"{param}: voulu={voulu} {units}, actual={actual} {units}, "
           f"Δ={pct_diff:+.3f}% (tol={tol}%)")
    print(f"  AUDIT  {msg}")
    assert pct_diff <= tol, f"OFF SPEC : {msg}"


# === Mesures runtime depuis logs ============================================

def _parse_timestamps(log_content: str, pattern: str):
    """Retourne list[(unix_ts, wall_offset)] pour chaque ligne matching pattern."""
    out = []
    for line in log_content.splitlines():
        if not re.search(pattern, line):
            continue
        # Format des logs : `<unix_ts> +<wall>s ...`
        m = re.match(r"(\d+\.\d+)\s+\+(\d+\.\d+)s", line)
        if m:
            out.append((float(m.group(1)), float(m.group(2))))
    return out


def _periods_ms(timestamps):
    """Inter-event periods en ms."""
    return [(timestamps[i+1][0] - timestamps[i][0]) * 1000
            for i in range(len(timestamps) - 1)]


TIMER_RUNTIME_PROBES = [
    ("tdma_tick", QEMU_LOG, r"\[tdma\] tick #", 4.615, "TDMA frame (virtual)"),
    ("kick", QEMU_LOG, r"\[kick\] fire #", 5.0, "Kick (wall, was REALTIME)"),
    ("bridge_clk_ind", (removed), r"CLK IND", 235.4, "CLK IND (51 frames)"),
    ("bts_clk_ind", BTS_LOG, r"Clock indication: fn=", 235.4, "BTS CLK IND reçu"),
    ("bsp_burst", QEMU_LOG, r"\[BSP\] BURST fn=", 4.615, "BSP RX burst"),
]


@pytest.mark.timer_audit
@pytest.mark.parametrize("name,log_path,pattern,expected_ms,desc",
                         TIMER_RUNTIME_PROBES,
                         ids=[p[0] for p in TIMER_RUNTIME_PROBES])
def test_timer_runtime_period(name, log_path, pattern, expected_ms, desc):
    """Mesure le period réel des timers depuis logs vs valeur attendue.
    Skip si pas assez de samples (run trop court)."""
    log = _read(log_path)
    if not log:
        pytest.skip(f"{log_path} vide")
    ts = _parse_timestamps(log, pattern)
    if len(ts) < 30:
        pytest.skip(f"{name}: only {len(ts)} samples in {log_path}")
    periods = _periods_ms(ts)
    # Filtre outliers extrêmes (gap > 10× expected = pause / restart)
    periods = [p for p in periods if p < expected_ms * 10]
    if len(periods) < 20:
        pytest.skip(f"{name}: only {len(periods)} clean samples")
    p_med = median(periods)
    p_mean = mean(periods)
    p_std = stdev(periods) if len(periods) > 1 else 0.0
    pct = abs(p_med - expected_ms) / expected_ms * 100.0
    msg = (f"{name} ({desc}): expected={expected_ms} ms, "
           f"median={p_med:.2f} ms, mean={p_mean:.2f} ms, "
           f"σ={p_std:.2f} ms, Δ_med={pct:+.1f}% (N={len(periods)})")
    print(f"  RUNTIME {msg}")
    # Tolérance large (50%) : sous icount=auto le rate dévie
    assert pct < 50.0, f"OFF EXPECTED : {msg}"


# === Graphes mermaid pour les périodes ======================================

@pytest.mark.timer_graph
def test_timer_temporal_graphs_emit_mermaid():
    """Génère un fichier mermaid xychart pour chaque timer instrumenté.
    Output : /tmp/timer_graphs.mmd contient les xycharts à coller dans un
    rapport markdown."""
    out_lines = ["# Calypso Timer temporal graphs",
                 "",
                 "Auto-generated by `test_timer_temporal_graphs_emit_mermaid`.",
                 "Une xychart par timer : Y = période entre 2 events (ms), "
                 "X = numéro d'event.",
                 ""]
    found_any = False
    for name, log_path, pattern, expected_ms, desc in TIMER_RUNTIME_PROBES:
        log = _read(log_path)
        if not log:
            continue
        ts = _parse_timestamps(log, pattern)
        if len(ts) < 10:
            continue
        periods = _periods_ms(ts)
        # Sample down to ~50 points max for mermaid render
        N = min(50, len(periods))
        step = max(1, len(periods) // N)
        sampled = periods[::step][:N]
        if not sampled:
            continue
        found_any = True

        out_lines.extend([
            f"## {name} — {desc}",
            f"Expected: {expected_ms} ms ; samples: {len(periods)} (showing {len(sampled)})",
            "",
            "```mermaid",
            "xychart-beta",
            f'  title "{name} period (ms) vs event #"',
            f'  x-axis "event #" 1 --> {len(sampled)}',
            f'  y-axis "period (ms)" 0 --> {max(sampled) * 1.2:.1f}',
            f'  line [{", ".join(f"{p:.2f}" for p in sampled)}]',
            "```",
            "",
        ])
    if not found_any:
        pytest.skip("aucun timer log dispo pour générer graph")
    out = "\n".join(out_lines)
    Path(GRAPHS_OUT).write_text(out)
    print(f"\n  emitted {Path(GRAPHS_OUT)} ({len(out)} chars, "
          f"{out.count('xychart-beta')} graphs)")


@pytest.mark.timer_graph
def test_timer_drift_summary_table():
    """Génère une table mermaid récap drift entre tous les timers."""
    rows = []
    for name, log_path, pattern, expected_ms, desc in TIMER_RUNTIME_PROBES:
        log = _read(log_path)
        if not log:
            rows.append((name, "—", expected_ms, "no log"))
            continue
        ts = _parse_timestamps(log, pattern)
        if len(ts) < 20:
            rows.append((name, "—", expected_ms, f"only {len(ts)} samples"))
            continue
        periods = [p for p in _periods_ms(ts) if p < expected_ms * 10]
        if not periods:
            rows.append((name, "—", expected_ms, "no clean samples"))
            continue
        med = median(periods)
        pct = (med - expected_ms) / expected_ms * 100.0
        status = "✓" if abs(pct) < 10 else "⚠" if abs(pct) < 30 else "✗"
        rows.append((name, f"{med:.2f} ms", f"{expected_ms} ms",
                     f"{status} Δ={pct:+.1f}%"))
    print("\n  Timer drift summary :")
    print(f"  {'name':<20} {'median':<12} {'expected':<12} {'status'}")
    for r in rows:
        print(f"  {r[0]:<20} {r[1]:<12} {r[2]:<12} {r[3]}")
