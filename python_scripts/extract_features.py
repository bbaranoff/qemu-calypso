#!/usr/bin/env python3
"""
extract_features.py — extrait les features (tests) et leur état OK/NOK
depuis la doc Calypso générée par pytest.

Source de vérité préférée : `results.json` (structure stable). Fallback :
parse de `test_results.md` ou `report.md` si results.json absent.

Output :
  - stdout : table markdown groupée par feature/couche
  - --json : JSON structuré
  - --csv : CSV plat (feature,name,marker,layer,category,state,duration,err)

Usage :
  python3 extract_features.py [path]              # cwd ou path → results.json
  python3 extract_features.py /tmp/test_results_*/results.json
  python3 extract_features.py /home/nirvana/myconfigs/osmo_root/
  python3 extract_features.py --json /tmp/results.json | jq
  python3 extract_features.py --csv  /tmp/results.json > features.csv
  python3 extract_features.py --group marker   # group by marker (default)
  python3 extract_features.py --group layer
  python3 extract_features.py --group category
  python3 extract_features.py --only-fails     # ne montre que NOK
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import zipfile
from collections import defaultdict
from io import StringIO
from pathlib import Path


# ─── Loading ────────────────────────────────────────────────────────────────

def _find_results_json(arg: str) -> Path | None:
    """Trouve un results.json à partir d'un arg flexible :
    - Path direct vers results.json
    - Path de dossier (cherche results.json dedans ou test_results_*/ )
    - Path de zip (cherche dedans)
    - Si arg est vide : cherche dans cwd ou /tmp ou /home/nirvana/myconfigs/osmo_root
    """
    if not arg:
        for cand in (Path.cwd() / "results.json",
                     Path("/tmp") / "results.json",
                     Path("/home/nirvana/myconfigs/osmo_root") / "results.json"):
            if cand.exists(): return cand
        # Cherche dans /tmp/test_results_* le plus récent
        candidates = sorted(Path("/tmp").glob("test_results_*/results.json"),
                            key=lambda p: p.stat().st_mtime, reverse=True)
        if candidates: return candidates[0]
        return None

    p = Path(arg)
    if p.is_file() and p.suffix == ".json":
        return p
    if p.is_file() and p.suffix == ".zip":
        return p  # géré séparément par _load_results
    if p.is_dir():
        for cand in (p / "results.json",):
            if cand.exists(): return cand
        # Cherche un test_results_*/ dans le dossier
        candidates = sorted(p.glob("test_results_*/results.json"),
                            key=lambda x: x.stat().st_mtime, reverse=True)
        if candidates: return candidates[0]
    return None


def _load_results(path: Path) -> dict:
    if path.suffix == ".zip":
        with zipfile.ZipFile(path) as zf:
            names = [n for n in zf.namelist() if n.endswith("results.json")]
            if not names:
                raise FileNotFoundError(f"pas de results.json dans {path}")
            with zf.open(names[0]) as f:
                return json.load(f)
    return json.loads(path.read_text())


# ─── Helpers ─────────────────────────────────────────────────────────────────

def _state(t: dict) -> str:
    """Mappe le record pytest à un état lisible (OK / NOK / XFAIL / SKIP)."""
    if t["outcome"] == "passed" and not t.get("wasxfail"):
        return "OK"
    if t["outcome"] == "failed" and not t.get("wasxfail"):
        return "NOK"
    if t.get("wasxfail"):
        return "XFAIL"
    if t["outcome"] == "skipped":
        return "SKIP"
    return t["outcome"].upper()


def _first_marker(t: dict) -> str:
    return (t.get("markers") or ["_unmarked_"])[0]


def _feature_label(t: dict, group_by: str) -> str:
    """Détermine la 'feature' selon le critère de groupement."""
    if group_by == "marker":
        return _first_marker(t)
    if group_by == "layer":
        return t.get("layer", "?")
    if group_by == "category":
        return t.get("category", "?")
    return "all"


# ─── Output renderers ────────────────────────────────────────────────────────

def render_markdown(data: dict, group_by: str, only_fails: bool) -> str:
    tests = data["tests"]
    if only_fails:
        tests = [t for t in tests if _state(t) == "NOK"]

    grouped: dict[str, list[dict]] = defaultdict(list)
    for t in tests:
        grouped[_feature_label(t, group_by)].append(t)

    out = []
    out.append(f"# Features et état OK/NOK — groupé par `{group_by}`\n")

    # Header counts
    counts = data.get("counts", {})
    if counts:
        out.append("**Compteurs globaux :**")
        items = " · ".join(f"{k}={v}" for k, v in counts.items() if v)
        out.append(f"`{items}`\n")

    for feat in sorted(grouped):
        items = grouped[feat]
        n_total = len(items)
        n_ok    = sum(1 for t in items if _state(t) == "OK")
        n_nok   = sum(1 for t in items if _state(t) == "NOK")
        n_xfail = sum(1 for t in items if _state(t) == "XFAIL")
        n_skip  = sum(1 for t in items if _state(t) == "SKIP")
        if n_nok > 0:
            icon = "🔴"
        elif n_total == n_ok:
            icon = "✅"
        elif n_ok > 0:
            icon = "🟡"
        else:
            icon = "⚪"

        out.append(f"## {icon} `{feat}` — {n_ok}/{n_total} OK"
                   f" ({n_nok} NOK, {n_xfail} XFAIL, {n_skip} SKIP)")
        out.append("")
        out.append("| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |")
        out.append("|---|---|---|---|---|---:|---|")
        for t in sorted(items, key=lambda x: (_state(x), x["name"])):
            state = _state(t)
            err = (t.get("err_short") or "").replace("|", "\\|")[:120]
            mks = ",".join(t.get("markers", []) or [])
            out.append(
                f"| {state} | `{t['name']}` | `{mks}` | "
                f"`{t.get('layer','?')}` | `{t.get('category','?')}` | "
                f"{t.get('duration_s',0):.2f}s | {err} |"
            )
        out.append("")

    return "\n".join(out)


def render_json(data: dict, group_by: str, only_fails: bool) -> str:
    tests = data["tests"]
    if only_fails:
        tests = [t for t in tests if _state(t) == "NOK"]
    grouped: dict[str, list[dict]] = defaultdict(list)
    for t in tests:
        entry = {
            "name":     t["name"],
            "state":    _state(t),
            "marker":   _first_marker(t),
            "layer":    t.get("layer", "?"),
            "category": t.get("category", "?"),
            "duration_s": t.get("duration_s", 0),
            "err":      t.get("err_short", "") or "",
            "nodeid":   t.get("nodeid", ""),
        }
        grouped[_feature_label(t, group_by)].append(entry)
    return json.dumps({
        "group_by": group_by,
        "counts": data.get("counts", {}),
        "features": dict(grouped),
    }, indent=2, ensure_ascii=False)


def render_csv(data: dict, group_by: str, only_fails: bool) -> str:
    tests = data["tests"]
    if only_fails:
        tests = [t for t in tests if _state(t) == "NOK"]
    buf = StringIO()
    w = csv.writer(buf)
    w.writerow(["feature", "name", "state", "marker", "layer",
                "category", "duration_s", "err_short", "nodeid"])
    for t in tests:
        w.writerow([
            _feature_label(t, group_by),
            t["name"],
            _state(t),
            _first_marker(t),
            t.get("layer", ""),
            t.get("category", ""),
            f"{t.get('duration_s', 0):.2f}",
            (t.get("err_short") or "").replace("\n", " "),
            t.get("nodeid", ""),
        ])
    return buf.getvalue()


# ─── Main ────────────────────────────────────────────────────────────────────

def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("path", nargs="?", default="",
                    help="Path to results.json or dir or zip (default: auto-detect)")
    ap.add_argument("--json", action="store_true", help="Output JSON")
    ap.add_argument("--csv",  action="store_true", help="Output CSV")
    ap.add_argument("--group", choices=("marker", "layer", "category"),
                    default="marker", help="Critère de groupement (default: marker)")
    ap.add_argument("--only-fails", action="store_true",
                    help="Ne montre que les tests NOK")
    args = ap.parse_args(argv[1:])

    res_path = _find_results_json(args.path)
    if not res_path:
        sys.stderr.write(f"✗ results.json introuvable (path={args.path!r})\n"
                         f"  Cherché dans cwd, /tmp, /home/nirvana/myconfigs/osmo_root,\n"
                         f"  et /tmp/test_results_*/.\n")
        return 2

    try:
        data = _load_results(res_path)
    except Exception as e:
        sys.stderr.write(f"✗ erreur lecture {res_path} : {type(e).__name__}: {e}\n")
        return 3

    if args.json:
        out = render_json(data, args.group, args.only_fails)
    elif args.csv:
        out = render_csv(data, args.group, args.only_fails)
    else:
        out = render_markdown(data, args.group, args.only_fails)
    sys.stdout.write(out)
    if not out.endswith("\n"):
        sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
