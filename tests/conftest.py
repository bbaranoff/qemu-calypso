"""
Pytest configuration for calypso milestone tests.

Marks are registered here so pytest doesn't warn about unknown markers.
Also: auto-generate Mermaid diagrams + a coherent markdown report bundled
in a per-run folder and a final .zip at session end.
"""
import datetime
import json
import os
import shutil
import subprocess
import zipfile
from pathlib import Path

import pytest


def pytest_configure(config):
    for marker in (
        # test_calypso_milestones.py
        "milestone_dsp_decoder: régression décodeur (POPM, Tier A, INTM dwell)",
        "milestone_bsp_dma:     data path ARM/BSP → DARAM (priorité A actuelle)",
        "milestone_fb_det:      correlator FB-det converge",
        "milestone_irq:         cycle IRQ DSP complet (SINT → ISR → RETE)",
        "milestone_l1ctl:       premier produit L1 vers mobile",
        "milestone_mm_lu:       location update bout-en-bout",
        # test_run_observability.py
        "runtime_health:        container alive + processus attendus",
        "runtime_dsp:           sample qemu.log + probes DSP",
        "runtime_bridge:        bridge.py drift FN + lookahead",
        "runtime_l1ctl:         pcap GSMTAP via tshark",
        "runtime_vty:           VTY mobile L23 (état RR/MM)",
        "runtime_irq:           compteurs IRQ via monitor QEMU",
        "runtime_summary:       snapshot consolidé (pytest -m runtime_summary -s)",
        # test_inject_frames.py
        "inject_frames:        injection NDB/FBSB/SI/task via inject.py (gdb-stub)",
        "inject_efficacy:      bonus — observe ARM-side reads in qemu.log post-inject",
    ):
        config.addinivalue_line("markers", marker)


# -----------------------------------------------------------------------------
# Mermaid diagram auto-generator
# -----------------------------------------------------------------------------
# Collect (nodeid, outcome, markers) per test as it runs.
_MERMAID_RESULTS: list[dict] = []

@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    out = yield
    rep = out.get_result()
    if rep.when != "call":
        # Only the "call" phase carries the real outcome. Skip setup/teardown
        # except when setup itself was skipped (covers @pytest.skip in fixture).
        if not (rep.when == "setup" and rep.outcome == "skipped"):
            return
    _MERMAID_RESULTS.append({
        "nodeid": item.nodeid,
        "name": item.name,
        "outcome": rep.outcome,           # passed | failed | skipped
        "markers": sorted({m.name for m in item.iter_markers()}),
        "duration_s": getattr(rep, "duration", 0.0),
        "wasxfail": hasattr(rep, "wasxfail"),
    })


def _sanitize_node_id(s: str) -> str:
    """Mermaid node IDs must be alphanumeric+underscore."""
    return "".join(c if c.isalnum() else "_" for c in s)[:48]


def _label(text: str) -> str:
    """Escape user text for safe use inside Mermaid quoted labels.

    GitHub's Mermaid parser accepts `id["..."]` with HTML-ish content.
    Common pitfalls : raw [], (), " inside the label, backticks.
    We:
      - replace " with &quot;
      - replace [ ] with ( )
      - replace ( ) keep but strip enclosing (which Mermaid uses for stadium)
    """
    return (
        text.replace('"', "&quot;")
            .replace("[", "(").replace("]", ")")
            .replace("\\", "/")
    )


# Hiérarchie : marker → (category, layer). Utilisée pour bâtir 3-level mermaid.
# Si un marker n'est pas listé, on tombe en "Other / unmarked".
_MARKER_TAXONOMY = {
    # Milestones (régressions persistantes)
    "milestone_dsp_decoder": ("Milestone", "DSP"),
    "milestone_bsp_dma":     ("Milestone", "L1-data"),
    "milestone_fb_det":      ("Milestone", "DSP"),
    "milestone_irq":         ("Milestone", "DSP"),
    "milestone_l1ctl":       ("Milestone", "L1-ctrl"),
    "milestone_mm_lu":       ("Milestone", "L3-MM"),
    # Runtime observability (snapshot du run live)
    "runtime_health":        ("Runtime", "Infra"),
    "runtime_dsp":           ("Runtime", "DSP"),
    "runtime_bridge":        ("Runtime", "L1-data"),
    "runtime_l1ctl":         ("Runtime", "L1-ctrl"),
    "runtime_vty":           ("Runtime", "Mgmt"),
    "runtime_irq":           ("Runtime", "DSP"),
    "runtime_summary":       ("Runtime", "Summary"),
    # Injection (forcer des trames via gdb-stub / UDP / l1ctl)
    "inject_frames":         ("Injection", "NDB"),
    "inject_efficacy":       ("Injection", "ARM-feedback"),
    # Timing
    "timing_invariant":      ("Timing", "Timers"),
}


def _classify(markers: list[str]) -> tuple[str, str]:
    """Mappe un set de markers vers (Category, Layer). Premier match gagne."""
    for m in markers:
        if m in _MARKER_TAXONOMY:
            return _MARKER_TAXONOMY[m]
    if not markers:
        return ("Other", "unmarked")
    # Fallback by prefix
    head = markers[0].split("_", 1)[0]
    return (head.capitalize() or "Other", "—")


# Pipeline order : ordre logique du flux GSM Calypso. Utilisé pour
# détecter automatiquement où la chaîne se rompt (1ère étape avec 0 pass).
_PIPELINE_ORDER = [
    "Infra",         # processus container alive
    "DSP",           # DSP boot / IRQ / FB-det converge
    "L1-data",       # BSP DMA / bursts DL bridge
    "L1-ctrl",       # SERCOMM L1CTL bus
    "Timers",        # TDMA / TINT0 / kick
    "NDB",           # injection NDB direct via gdb-stub
    "ARM-feedback",  # ARM L1 lit les cells injectées
    "L3-MM",         # mobile fait LU jusqu'au bout
    "Mgmt",          # VTY osmocom/mobile
    "Summary",
    "Infra,Misc",
]

def _layer_status(tests: list[dict]) -> tuple[str, int, int]:
    """(status, passed, total) où status ∈ {'pass','partial','fail','empty'}"""
    if not tests: return ("empty", 0, 0)
    p = sum(1 for t in tests if t["outcome"] == "passed")
    n = len(tests)
    if p == n: return ("pass", p, n)
    if p == 0: return ("fail", p, n)
    return ("partial", p, n)


def _build_mermaid() -> str:
    ts = datetime.datetime.now().isoformat(timespec="seconds")
    lines = [
        f"%% Generated by tests/conftest.py at {ts}",
        "graph TD",
        "  classDef pass    fill:#bef5b1,stroke:#1f7a1f,color:#0a3d0a;",
        "  classDef partial fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef fail    fill:#fbb4b4,stroke:#a31a1a,color:#5a0000;",
        "  classDef skip    fill:#e0e0e0,stroke:#6a6a6a,color:#333;",
        "  classDef xfail   fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef break_  fill:#ff5252,stroke:#990000,color:#fff,stroke-width:3px;",
        "  ROOT[Test Run]",
    ]
    if not _MERMAID_RESULTS:
        lines.append("  ROOT --> EMPTY[no tests collected]")
        return "\n".join(lines)

    # tree[category][layer] = [tests…]
    tree: dict[str, dict[str, list[dict]]] = {}
    for r in _MERMAID_RESULTS:
        cat, layer = _classify(r["markers"])
        tree.setdefault(cat, {}).setdefault(layer, []).append(r)

    # Build flat layer→tests for pipeline view (across categories)
    flat_layer: dict[str, list[dict]] = {}
    for cat, layers in tree.items():
        for layer, tests in layers.items():
            flat_layer.setdefault(layer, []).extend(tests)

    # PIPELINE VIEW — chaîne logique Infra → DSP → L1 → L2 → L3
    lines.append('  subgraph PIPELINE ["Pipeline — où ça casse"]')
    lines.append("    direction LR")
    prev = None
    broken = False
    for layer in _PIPELINE_ORDER:
        tests = flat_layer.get(layer, [])
        status, p, n = _layer_status(tests)
        if status == "empty":
            continue
        pid = "P_" + _sanitize_node_id(layer)
        cls = {"pass": "pass", "partial": "partial", "fail": "fail"}.get(status, "skip")
        lines.append(f'    {pid}["{_label(layer)}<br/>{p}/{n}"]')
        lines.append(f"    class {pid} {cls};")
        if prev is not None:
            lines.append(f"    {prev} --> {pid}")
        if status == "fail" and not broken:
            bid = "BREAK_" + _sanitize_node_id(layer)
            lines.append(f'    {pid} -.->|🛑| {bid}["BREAK HERE<br/>{_label(layer)} 0% pass"]')
            lines.append(f"    class {bid} break_;")
            broken = True
        prev = pid
    lines.append("  end")
    lines.append(f"  ROOT --> PIPELINE")

    # DETAIL TREE — category → layer → test
    lines.append('  subgraph DETAIL ["Détail — catégorie / couche / test"]')
    counter = 0
    for cat in sorted(tree):
        cid = "C_" + _sanitize_node_id(cat)
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        ccls = {"pass":"pass","partial":"partial","fail":"fail"}.get(cstatus,"skip")
        lines.append(f'    {cid}["{_label(cat)}<br/>{cp}/{cn}"]')
        lines.append(f"    class {cid} {ccls};")
        for layer in sorted(tree[cat]):
            lid = f"{cid}_L_" + _sanitize_node_id(layer)
            lt = tree[cat][layer]
            lstatus, lp, ln = _layer_status(lt)
            lcls = {"pass":"pass","partial":"partial","fail":"fail"}.get(lstatus,"skip")
            lines.append(f'    {lid}["{_label(layer)}<br/>{lp}/{ln}"]')
            lines.append(f"    class {lid} {lcls};")
            lines.append(f"    {cid} --> {lid}")
            for r in lt:
                counter += 1
                tid = f"T_{counter:03d}_" + _sanitize_node_id(r["name"])
                short = _label(r["name"][:38])
                marker_short = _label(",".join(r["markers"][:1]))
                label = f"{short}<br/>{marker_short}<br/>{r['duration_s']:.2f}s"
                cls = ("xfail" if r["wasxfail"]
                       else "pass" if r["outcome"] == "passed"
                       else "fail" if r["outcome"] == "failed"
                       else "skip")
                lines.append(f'    {lid} --> {tid}["{label}"]')
                lines.append(f"    class {tid} {cls};")
    lines.append("  end")
    lines.append(f"  ROOT --> DETAIL")

    return "\n".join(lines)


# -----------------------------------------------------------------------------
# Markdown report skeleton — coherent text generated from results.
# -----------------------------------------------------------------------------
_REPORT_SKELETON = """\
# Calypso test report — {timestamp}

> Auto-generated by `tests/conftest.py::pytest_sessionfinish`.
> Pasteable directly in a GitHub issue/PR (Mermaid blocks render natively).

## Status global

{global_status_block}

## Pipeline — où ça casse

Le pipeline ci-dessous trace le flux logique GSM Calypso, étape par étape.
Chaque étape est colorée selon le ratio de tests qui passent. **Si une étape
est rouge (0% pass), c'est le premier blocker** — tout ce qui vient après
ne peut pas être validé tant qu'elle n'est pas verte.

```mermaid
{pipeline_mermaid}
```

{pipeline_commentary}

## Blockers

{blockers_block}

## Couches — ce qui marche, ce qui casse

{layers_commentary}

## Diagramme détaillé (catégorie → couche → test)

```mermaid
{detail_mermaid}
```

## Skipped / xfailed

{skipped_block}

## Résultats bruts pytest

<details>
<summary>Cliquer pour déplier — sortie verbatim type <code>pytest -v</code></summary>

```
{raw_pytest_block}
```

</details>

## Reproduction

```bash
cd /home/nirvana/qemu-calypso/tests
/tmp/calypso-venv/bin/pytest -v --tb=short
# Filtrer par marker :
/tmp/calypso-venv/bin/pytest -v -m inject_frames
/tmp/calypso-venv/bin/pytest -v -m 'not inject_efficacy'
```

Le diagramme et ce rapport sont régénérés à chaque run et écrits dans
`{out_dir}/test_results.{{mmd,md}}` (override via `CALYPSO_TEST_OUT`).

---

_Run finished at {timestamp_end}._
"""


def _gen_global_status_block(counts: dict) -> str:
    """GitHub-flavored alert + summary table."""
    total = sum(counts.values()) or 1
    p = counts.get("passed", 0)
    f = counts.get("failed", 0)
    s = counts.get("skipped", 0)
    x = counts.get("xfailed", 0)
    pct = 100 * p / total
    if f == 0 and p > 0:
        alert_kind = "TIP"
        verdict = f"✅ **ALL PASS** — {p}/{total} ({pct:.0f} %)"
    elif p == 0:
        alert_kind = "CAUTION"
        verdict = f"🛑 **BLOCKED** — aucun test ne passe ({total} essais)"
    elif f > 0:
        alert_kind = "WARNING"
        verdict = f"⚠️ **PARTIAL** — {f} échec(s), {p}/{total} passent ({pct:.0f} %)"
    else:
        alert_kind = "NOTE"
        verdict = f"❔ état indéterminé — {p}/{total}"
    return (
        f"> [!{alert_kind}]\n> {verdict}\n\n"
        f"| résultat | nombre |\n|---|---:|\n"
        f"| ✅ passed | {p} |\n| ❌ failed | {f} |\n"
        f"| ⏭️ skipped | {s} |\n| ⚠️ xfailed | {x} |\n| **total** | **{total}** |\n"
    )


def _gen_pipeline_commentary(flat_layer: dict) -> str:
    """Texte interpretatif autour du pipeline mermaid."""
    parts = []
    last_pass_layer = None
    first_fail_layer = None
    for layer in _PIPELINE_ORDER:
        tests = flat_layer.get(layer, [])
        status, p, n = _layer_status(tests)
        if status == "empty":
            continue
        if status == "pass" and first_fail_layer is None:
            last_pass_layer = layer
        if status == "fail" and first_fail_layer is None:
            first_fail_layer = (layer, p, n)
    if first_fail_layer:
        parts.append(
            f"➡️ **Première rupture : `{first_fail_layer[0]}`** "
            f"({first_fail_layer[1]}/{first_fail_layer[2]} tests passent)."
        )
        if last_pass_layer:
            parts.append(
                f"Les étapes jusqu'à `{last_pass_layer}` sont OK ; "
                f"le bug à investiguer est dans la transition "
                f"`{last_pass_layer}` → `{first_fail_layer[0]}`."
            )
        else:
            parts.append(
                f"Aucune étape ne passe avant — la chaîne est cassée dès "
                f"l'entrée (`{first_fail_layer[0]}`)."
            )
    else:
        if last_pass_layer:
            parts.append(
                f"✅ Pipeline complet jusqu'à `{last_pass_layer}` — pas de "
                f"rupture détectée dans l'ordre logique."
            )
        else:
            parts.append("ℹ️ Aucun test classé dans le pipeline — taxonomie à compléter.")
    return "\n\n".join(parts)


def _gen_blockers_block(results: list[dict], flat_layer: dict) -> str:
    failed = [r for r in results if r["outcome"] == "failed" and not r["wasxfail"]]
    if not failed:
        return "_Aucun test en échec._"
    parts = [f"**{len(failed)} test(s) en échec** :\n"]
    # Order failed tests by pipeline layer (so the first one is the most "upstream")
    layer_order = {layer: i for i, layer in enumerate(_PIPELINE_ORDER)}
    def _key(r):
        cat, layer = _classify(r["markers"])
        return (layer_order.get(layer, 999), layer, r["nodeid"])
    for i, r in enumerate(sorted(failed, key=_key), 1):
        cat, layer = _classify(r["markers"])
        prefix = "🔴" if i == 1 else "🔻"
        parts.append(
            f"{prefix} **{i}. `{r['name']}`** — {cat} / {layer} — "
            f"marker(s): {','.join(r['markers']) or '_aucun_'} — "
            f"durée {r['duration_s']:.2f}s\n  - `{r['nodeid']}`"
        )
    return "\n".join(parts)


def _gen_layers_commentary(tree: dict) -> str:
    """Pour chaque couche, dire ce qui marche et ce qui casse."""
    out = []
    flat: dict[str, list[dict]] = {}
    for cat, layers in tree.items():
        for layer, tests in layers.items():
            flat.setdefault(layer, []).extend(tests)
    for layer in _PIPELINE_ORDER + [l for l in flat if l not in _PIPELINE_ORDER]:
        if layer not in flat:
            continue
        tests = flat[layer]
        status, p, n = _layer_status(tests)
        icon = {"pass":"✅","partial":"🟡","fail":"🛑","empty":"·"}.get(status,"·")
        out.append(f"### {icon} `{layer}` — {p}/{n}")
        if status == "pass":
            out.append("Tous les tests passent dans cette couche.")
        else:
            fails = [t["name"] for t in tests if t["outcome"] == "failed" and not t["wasxfail"]]
            skips = [t["name"] for t in tests if t["outcome"] == "skipped"]
            xfs   = [t["name"] for t in tests if t["wasxfail"]]
            if fails:
                out.append(f"❌ **Échecs :** " + ", ".join(f"`{n}`" for n in fails))
            if skips:
                out.append(f"⏭️ **Skipped :** " + ", ".join(f"`{n}`" for n in skips))
            if xfs:
                out.append(f"⚠️ **xfail :** " + ", ".join(f"`{n}`" for n in xfs))
        out.append("")
    return "\n".join(out) if out else "_aucune couche détectée._"


def _gen_raw_pytest_block(results: list[dict]) -> str:
    """Reproduit la sortie de `pytest -v` sous forme texte plein."""
    if not results:
        return "(no tests collected)"
    lines = []
    status_map = {"passed": "PASSED", "failed": "FAILED",
                  "skipped": "SKIPPED"}
    for r in results:
        st = "XFAIL" if r["wasxfail"] else status_map.get(r["outcome"], r["outcome"].upper())
        lines.append(f"{st:8} {r['nodeid']} ({r['duration_s']:.3f}s)")
    # Final summary line (à la pytest)
    counts = {"passed": 0, "failed": 0, "skipped": 0, "xfailed": 0}
    for r in results:
        if r["wasxfail"]:
            counts["xfailed"] += 1
        else:
            counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
    summary_parts = []
    if counts["passed"]: summary_parts.append(f"{counts['passed']} passed")
    if counts["failed"]: summary_parts.append(f"{counts['failed']} failed")
    if counts["skipped"]: summary_parts.append(f"{counts['skipped']} skipped")
    if counts["xfailed"]: summary_parts.append(f"{counts['xfailed']} xfailed")
    total_dur = sum(r["duration_s"] for r in results)
    lines.append("")
    lines.append("=" * 60)
    lines.append(", ".join(summary_parts) + f" in {total_dur:.2f}s")
    return "\n".join(lines)


def _gen_skipped_block(results: list[dict]) -> str:
    skipped = [r for r in results if r["outcome"] == "skipped"]
    xfs = [r for r in results if r["wasxfail"]]
    if not skipped and not xfs:
        return "_Aucun test skipped ou xfailed._"
    parts = []
    if skipped:
        parts.append(f"### {len(skipped)} skipped")
        for r in skipped:
            parts.append(f"- `{r['name']}` — {','.join(r['markers']) or 'no-marker'}")
    if xfs:
        parts.append(f"\n### {len(xfs)} xfailed")
        for r in xfs:
            parts.append(f"- `{r['name']}` — {','.join(r['markers']) or 'no-marker'}")
    return "\n".join(parts)


def _try_render_mermaid(mmd_path: Path, png_path: Path) -> bool:
    """Render .mmd → .png via mermaid-cli (mmdc) if available. Returns True on success."""
    mmdc = shutil.which("mmdc")
    if not mmdc:
        # try npx fallback
        npx = shutil.which("npx")
        if npx:
            try:
                rc = subprocess.run(
                    [npx, "--yes", "@mermaid-js/mermaid-cli",
                     "-i", str(mmd_path), "-o", str(png_path)],
                    timeout=45, capture_output=True)
                return rc.returncode == 0 and png_path.exists()
            except Exception:
                return False
        return False
    try:
        rc = subprocess.run([mmdc, "-i", str(mmd_path), "-o", str(png_path),
                             "-b", "transparent"],
                            timeout=45, capture_output=True)
        return rc.returncode == 0 and png_path.exists()
    except Exception:
        return False


def pytest_sessionfinish(session, exitstatus):
    """Write mermaid diagrams + coherent markdown report + bundle zip."""
    out_dir = Path(os.environ.get("CALYPSO_TEST_OUT", "/tmp"))
    out_dir.mkdir(parents=True, exist_ok=True)

    # Per-run folder + zip
    ts_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = out_dir / f"test_results_{ts_id}"
    folder.mkdir(parents=True, exist_ok=True)

    # Derive structures
    tree: dict[str, dict[str, list[dict]]] = {}
    for r in _MERMAID_RESULTS:
        cat, layer = _classify(r["markers"])
        tree.setdefault(cat, {}).setdefault(layer, []).append(r)
    flat_layer: dict[str, list[dict]] = {}
    for cat, layers in tree.items():
        for layer, tests in layers.items():
            flat_layer.setdefault(layer, []).extend(tests)
    counts = {"passed": 0, "failed": 0, "skipped": 0, "xfailed": 0}
    for r in _MERMAID_RESULTS:
        if r["wasxfail"]: counts["xfailed"] += 1
        else: counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1

    # Build diagrams
    full_diagram = _build_mermaid()
    pipeline_only = _build_pipeline_only_mermaid(flat_layer)
    detail_only   = _build_detail_only_mermaid(tree)

    # Write .mmd files
    full_mmd     = folder / "full.mmd"
    pipeline_mmd = folder / "pipeline.mmd"
    detail_mmd   = folder / "detail.mmd"
    full_mmd.write_text(full_diagram)
    pipeline_mmd.write_text(pipeline_only)
    detail_mmd.write_text(detail_only)

    # Try render PNG (best-effort — needs mmdc/npx; falls back silently)
    rendered: list[Path] = []
    for src, dst_name in [(pipeline_mmd, "pipeline.png"),
                          (detail_mmd,   "detail.png"),
                          (full_mmd,     "full.png")]:
        dst = folder / dst_name
        if _try_render_mermaid(src, dst):
            rendered.append(dst)

    # Markdown report
    ts_start = datetime.datetime.now().isoformat(timespec="seconds")
    md = _REPORT_SKELETON.format(
        timestamp        = ts_start,
        timestamp_end    = datetime.datetime.now().isoformat(timespec="seconds"),
        out_dir          = folder,
        global_status_block = _gen_global_status_block(counts),
        pipeline_mermaid    = pipeline_only,
        pipeline_commentary = _gen_pipeline_commentary(flat_layer),
        blockers_block      = _gen_blockers_block(_MERMAID_RESULTS, flat_layer),
        layers_commentary   = _gen_layers_commentary(tree),
        detail_mermaid      = detail_only,
        skipped_block       = _gen_skipped_block(_MERMAID_RESULTS),
        raw_pytest_block    = _gen_raw_pytest_block(_MERMAID_RESULTS),
    )
    md_path = folder / "test_results.md"
    md_path.write_text(md)

    # Raw JSON results for machine consumption
    (folder / "results.json").write_text(
        json.dumps({"counts": counts, "tests": _MERMAID_RESULTS}, indent=2, default=str))

    # Stable top-level copies (so pytest -v can be pointed at them deterministically)
    (out_dir / "test_results.md").write_text(md)
    (out_dir / "test_results.mmd").write_text(full_diagram)

    # Zip everything
    zip_path = out_dir / f"test_results_{ts_id}.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for entry in folder.iterdir():
            zf.write(entry, arcname=entry.name)
    (out_dir / "test_results_latest.zip").write_bytes(zip_path.read_bytes())

    terminal = session.config.pluginmanager.get_plugin("terminalreporter")
    if terminal is not None:
        terminal.write_sep("=", "Mermaid report bundle", purple=True)
        terminal.write_line(f"  folder : {folder}/")
        terminal.write_line(f"  md     : {md_path}")
        terminal.write_line(f"  zip    : {zip_path}")
        terminal.write_line(f"  latest : {out_dir/'test_results_latest.zip'}")
        terminal.write_line(f"  PNGs rendered : {len(rendered)}  "
                            f"({'mmdc found' if shutil.which('mmdc') else 'mmdc absent — only .mmd written'})")
        terminal.write_line(f"  counts : {counts}")


def _build_pipeline_only_mermaid(flat_layer: dict) -> str:
    lines = [
        "graph LR",
        "  classDef pass    fill:#bef5b1,stroke:#1f7a1f,color:#0a3d0a;",
        "  classDef partial fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef fail    fill:#fbb4b4,stroke:#a31a1a,color:#5a0000;",
        "  classDef skip    fill:#e0e0e0,stroke:#6a6a6a,color:#333;",
        "  classDef break_  fill:#ff5252,stroke:#990000,color:#fff,stroke-width:3px;",
    ]
    prev = None; broken = False
    for layer in _PIPELINE_ORDER:
        tests = flat_layer.get(layer, [])
        status, p, n = _layer_status(tests)
        if status == "empty": continue
        pid = "P_" + _sanitize_node_id(layer)
        cls = {"pass":"pass","partial":"partial","fail":"fail"}.get(status,"skip")
        lines.append(f'  {pid}["{_label(layer)}<br/>{p}/{n}"]')
        lines.append(f"  class {pid} {cls};")
        if prev is not None:
            lines.append(f"  {prev} --> {pid}")
        if status == "fail" and not broken:
            bid = "BREAK_" + _sanitize_node_id(layer)
            lines.append(f'  {pid} -.->|🛑| {bid}["BREAK HERE<br/>{_label(layer)} 0% pass"]')
            lines.append(f"  class {bid} break_;")
            broken = True
        prev = pid
    return "\n".join(lines)


def _build_detail_only_mermaid(tree: dict) -> str:
    lines = [
        "graph TD",
        "  classDef pass    fill:#bef5b1,stroke:#1f7a1f,color:#0a3d0a;",
        "  classDef partial fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef fail    fill:#fbb4b4,stroke:#a31a1a,color:#5a0000;",
        "  classDef skip    fill:#e0e0e0,stroke:#6a6a6a,color:#333;",
        "  classDef xfail   fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
    ]
    counter = 0
    for cat in sorted(tree):
        cid = "C_" + _sanitize_node_id(cat)
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        ccls = {"pass":"pass","partial":"partial","fail":"fail"}.get(cstatus,"skip")
        lines.append(f'  {cid}["{_label(cat)}<br/>{cp}/{cn}"]')
        lines.append(f"  class {cid} {ccls};")
        for layer in sorted(tree[cat]):
            lid = f"{cid}_L_" + _sanitize_node_id(layer)
            lt = tree[cat][layer]
            lstatus, lp, ln = _layer_status(lt)
            lcls = {"pass":"pass","partial":"partial","fail":"fail"}.get(lstatus,"skip")
            lines.append(f'  {lid}["{_label(layer)}<br/>{lp}/{ln}"]')
            lines.append(f"  class {lid} {lcls};")
            lines.append(f"  {cid} --> {lid}")
            for r in lt:
                counter += 1
                tid = f"T_{counter:03d}_" + _sanitize_node_id(r["name"])
                short = _label(r["name"][:38])
                marker_short = _label(",".join(r["markers"][:1]))
                label_txt = f"{short}<br/>{marker_short}<br/>{r['duration_s']:.2f}s"
                cls = ("xfail" if r["wasxfail"]
                       else "pass" if r["outcome"] == "passed"
                       else "fail" if r["outcome"] == "failed"
                       else "skip")
                lines.append(f'  {lid} --> {tid}["{label_txt}"]')
                lines.append(f"  class {tid} {cls};")
    return "\n".join(lines)
