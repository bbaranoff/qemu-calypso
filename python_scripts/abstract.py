#!/usr/bin/env python3
"""
abstract.py — audit indépendant qemu-calypso

À placer dans le dossier contenant results.json + log_timeline.csv
(typiquement le dossier où sortent les .qmd / .mmd / test_results.md).

Recompute tout depuis les données brutes — ne fait pas confiance
au markdown généré. Sortie console, ~80 lignes, peer-level.

Usage:
    python3 abstract.py              # cwd
    python3 abstract.py /path/dir    # autre dossier
"""

from __future__ import annotations

import csv
import json
import statistics as st
import sys
from collections import Counter, defaultdict
from pathlib import Path


# ───────────────────────────── helpers ─────────────────────────────

def _bar(pct: float, width: int = 20) -> str:
    n = int(round(pct / 100 * width))
    return "█" * n + "·" * (width - n)


def _fmt_num(v: float) -> str:
    if v >= 1000:
        return f"{v/1000:.1f}k"
    if v >= 100:
        return f"{v:.0f}"
    return f"{v:.1f}"


def _load_json(p: Path) -> dict:
    with p.open() as f:
        return json.load(f)


def _load_csv(p: Path) -> list[dict]:
    with p.open() as f:
        return list(csv.DictReader(f))


# ───────────────────────────── tests ─────────────────────────────

def audit_tests(results_path: Path) -> dict:
    """Recompute outcomes, fail/xfail clusters, layer health."""
    data = _load_json(results_path)
    tests = data["tests"]

    outcomes = Counter(t["outcome"] for t in tests)
    xfail = sum(1 for t in tests if t.get("wasxfail"))
    real_skip = outcomes["skipped"] - xfail
    total = len(tests)
    passed = outcomes["passed"]
    failed = outcomes["failed"]
    actionable = passed + failed
    fn_pct = 100 * passed / actionable if actionable else 0
    raw_pct = 100 * passed / total if total else 0

    # Fails groupés par fichier-source
    fails_by_file = defaultdict(list)
    for t in tests:
        if t["outcome"] == "failed":
            f = t["nodeid"].split("::")[0]
            fails_by_file[f].append(t)

    # Santé par layer (passed / actionable_in_layer)
    layer_stats = defaultdict(lambda: Counter())
    for t in tests:
        layer_stats[t["layer"]][t["outcome"]] += 1
    layer_health = {}
    for L, c in layer_stats.items():
        act = c["passed"] + c["failed"]
        layer_health[L] = {
            "pass": c["passed"],
            "fail": c["failed"],
            "skip": c["skipped"],
            "total": sum(c.values()),
            "fn_pct": 100 * c["passed"] / act if act else None,  # None = tout xfail/skip
        }

    # xfail clusters par marker (ce qui est connu-cassé)
    xfail_by_marker = Counter()
    for t in tests:
        if t.get("wasxfail"):
            for m in t["markers"] or ["(no-marker)"]:
                xfail_by_marker[m] += 1

    return {
        "total": total,
        "passed": passed,
        "failed": failed,
        "xfail": xfail,
        "skip": real_skip,
        "fn_pct": fn_pct,
        "raw_pct": raw_pct,
        "actionable": actionable,
        "fails_by_file": dict(fails_by_file),
        "layer_health": layer_health,
        "xfail_by_marker": xfail_by_marker,
        "self_reported": data.get("counts", {}),
    }


# ───────────────────────────── timeline ─────────────────────────────

def audit_timeline(csv_path: Path) -> dict:
    """Recompute signal du log timeline."""
    rows = _load_csv(csv_path)
    if not rows:
        return {}

    t_start = float(rows[0]["t_rel"])
    t_end = float(rows[-1]["t_rel"])
    n_buckets = len(rows)
    bucket_s = (t_end - t_start) / max(1, n_buckets - 1)

    cols = [c for c in rows[0].keys() if c != "t_rel"]
    stats = {}
    for c in cols:
        vals = [int(r[c]) for r in rows]
        stats[c] = {
            "sum": sum(vals),
            "mean": st.mean(vals),
            "median": st.median(vals),
            "max": max(vals),
            "min": min(vals),
            "nonzero_buckets": sum(1 for v in vals if v > 0),
            "n": len(vals),
        }
    return {
        "duration_s": t_end - t_start,
        "n_buckets": n_buckets,
        "bucket_s": bucket_s,
        "cols": cols,
        "stats": stats,
    }


# ───────────────────────────── diagnostic ─────────────────────────────

# Mots-clefs voie-1 (chemin MVP, blocker si fail) vs voie-2 (R&D, xfail OK)
VOIE1_MARKERS = {
    "milestone_l1ctl", "runtime_l1ctl",
    "drift", "timer_invariant",
    "runtime_log_grep", "runtime_bridge",
}
VOIE2_MARKERS = {
    "milestone_fb_det", "milestone_irq", "milestone_dsp_decoder",
    "milestone_mm_lu", "runtime_dsp", "runtime_irq",
    "inject_efficacy", "runtime_irda",
}


def diagnose(t: dict, tl: dict) -> list[str]:
    """Signaux de haut niveau, en français peer-level."""
    lines = []

    # Voie-1 vs voie-2 sur les fails
    v1, v2, other = 0, 0, 0
    for fs in t["fails_by_file"].values():
        for x in fs:
            ms = set(x["markers"] or [])
            if ms & VOIE1_MARKERS:
                v1 += 1
            elif ms & VOIE2_MARKERS:
                v2 += 1
            else:
                other += 1
    lines.append(
        f"fails sur chemin voie-1 (MVP) : {v1}/{t['failed']}  ·  "
        f"voie-2 : {v2}  ·  autre : {other}"
    )

    # Signal critique du timeline
    if tl and "stats" in tl:
        s = tl["stats"]
        if "stack_in_ndb" in s:
            ndb = s["stack_in_ndb"]
            if ndb["sum"] == 0:
                lines.append(
                    "stack_in_ndb=0 sur 100% des buckets → la pile mobile "
                    "n'entre jamais en NDB malgré injection active"
                )
            else:
                ratio = 100 * ndb["nonzero_buckets"] / ndb["n"]
                lines.append(f"stack_in_ndb actif {ratio:.0f}% du temps")

        if "fb_det_hit" in s and "stack_in_ndb" in s:
            fbd = s["fb_det_hit"]
            ndb = s["stack_in_ndb"]
            if fbd["sum"] > 0 and ndb["sum"] == 0:
                lines.append(
                    f"fb_det_hit stable ({fbd['mean']:.1f}/bucket) mais consommation NDB nulle "
                    "→ écriture OK, consommation par mobile manquante"
                )

        if "tdma" in s and "frame_irq" in s:
            td = s["tdma"]["mean"]
            fi = s["frame_irq"]["mean"]
            if td < 1 and fi < 1:
                lines.append(
                    f"TDMA/FIQ très rares ({td:.2f}/{fi:.2f} par bucket) "
                    "→ DSP IDLE (cohérent voie-2 connue)"
                )

        if "mobile" in s:
            m = s["mobile"]["mean"]
            if m < 5:
                lines.append(
                    f"mobile peu actif côté L1CTL ({m:.1f}/bucket) "
                    "→ MITM n'élève pas de DATA_IND"
                )

    # Cohérence interne : self-reported vs recompté
    sr = t.get("self_reported") or {}
    if sr:
        recount = {
            "passed": t["passed"], "failed": t["failed"],
            "xfailed": t["xfail"], "skipped": t["skip"],
        }
        diff = {k: (sr.get(k), recount[k]) for k in recount if sr.get(k) != recount[k]}
        if diff:
            lines.append(f"⚠ divergence self-reported vs recompté : {diff}")
        else:
            lines.append("self-reported counts == recompté (cohérent)")

    return lines


# ───────────────────────────── rendering ─────────────────────────────

def render(t: dict, tl: dict) -> str:
    out = []
    p = out.append

    p("=" * 72)
    p(f"  AUDIT INDÉPENDANT qemu-calypso — {t['total']} tests")
    p("=" * 72)

    # Bloc global
    p("")
    p(f"  passed   {t['passed']:>4}  {_bar(t['raw_pct'])}  {t['raw_pct']:>5.1f}% brut")
    p(f"  failed   {t['failed']:>4}")
    p(f"  xfailed  {t['xfail']:>4}   (R&D voie-2, accepté)")
    p(f"  skipped  {t['skip']:>4}")
    p("")
    p(f"  fonctionnel  {t['fn_pct']:>5.1f}%   (passed / (passed+failed))")
    p(f"  actionable   {100*t['actionable']/t['total']:>5.1f}%   "
      f"(non-xfail/skip)")

    # Fails par fichier
    p("")
    p("─── 9 fails par fichier-source ───")
    for fname, fs in sorted(t["fails_by_file"].items()):
        p(f"  {fname}")
        for x in fs:
            ms = ",".join(x["markers"]) if x["markers"] else "-"
            p(f"    ✗ {x['name']:<48} [{ms}]")

    # Layers
    p("")
    p("─── Santé par layer (fonctionnel = pass/(pass+fail)) ───")
    rows = sorted(
        t["layer_health"].items(),
        key=lambda kv: (kv[1]["fn_pct"] if kv[1]["fn_pct"] is not None else -1),
    )
    for L, h in rows:
        if h["fn_pct"] is None:
            tag = "n/a  (tout xfail/skip)"
            bar = "·" * 20
        else:
            tag = f"{h['fn_pct']:>5.1f}%"
            bar = _bar(h["fn_pct"])
        p(f"  {L:<18} {h['pass']:>2}/{h['total']:<2}  {bar}  {tag}"
          f"  (fail={h['fail']}, skip={h['skip']})")

    # xfail clusters
    p("")
    p("─── xfail clusters (où la voie-2 est connue cassée) ───")
    for m, n in t["xfail_by_marker"].most_common():
        p(f"  {n:>2}× {m}")

    # Timeline
    if tl:
        p("")
        p(f"─── Timeline {tl['duration_s']:.0f}s "
          f"({tl['n_buckets']} buckets de {tl['bucket_s']:.0f}s) ───")
        p(f"  {'signal':<16}{'sum':>10}{'mean/bucket':>14}{'max':>8}{'nonzero':>10}")
        order = ["qemu", "bridge", "osmocon", "mobile", "tdma",
                 "frame_irq", "kick", "fb_det_hit", "stack_in_ndb"]
        for c in order:
            if c not in tl["stats"]:
                continue
            s = tl["stats"][c]
            flag = ""
            if c == "stack_in_ndb" and s["sum"] == 0:
                flag = "  ← bloquant"
            if c == "fb_det_hit" and s["mean"] > 3:
                flag = "  ← FB OK"
            p(f"  {c:<16}{_fmt_num(s['sum']):>10}{s['mean']:>14.2f}"
              f"{s['max']:>8}{s['nonzero_buckets']:>4}/{s['n']:<4}{flag}")

    # Diagnostic synthèse
    p("")
    p("─── Diagnostic ───")
    for line in diagnose(t, tl):
        p(f"  • {line}")

    p("")
    p("=" * 72)
    return "\n".join(out)


# ───────────────────────────── main ─────────────────────────────

def main(argv: list[str]) -> int:
    here = Path(argv[1]) if len(argv) > 1 else Path.cwd()
    results = here / "results.json"
    timeline = here / "log_timeline.csv"

    if not results.exists():
        print(f"✗ results.json introuvable dans {here}", file=sys.stderr)
        return 1

    t = audit_tests(results)
    tl = audit_timeline(timeline) if timeline.exists() else {}
    if not tl:
        print(f"⚠ {timeline.name} absent — audit tests seul", file=sys.stderr)

    print(render(t, tl))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
