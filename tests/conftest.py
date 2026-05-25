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


# ─── In-container shim : si on tourne dans le container `trying` (`/.dockerenv`
# existe), tous les `docker exec CONTAINER cmd...` deviennent un appel direct
# `cmd...` sans préfixe docker. Idem `docker inspect`/`docker ps`/`docker test`
# retournent des stubs raisonnables. Permet à `--gen-doc-local` de lancer
# pytest dans le container sans que les tests aient besoin de patcher leurs
# subprocess.run individuellement.
if os.path.exists("/.dockerenv"):
    _orig_subprocess_run = subprocess.run

    def _calypso_local_run(*args, **kwargs):
        cmd = args[0] if args else kwargs.get("args")
        if isinstance(cmd, (list, tuple)) and len(cmd) >= 2 and cmd[0] == "docker":
            sub = cmd[1]
            if sub == "exec":
                # "docker exec NAME [-e ENV=...] cmd...". Skip name + env flags.
                rest = list(cmd[2:])
                while rest and rest[0].startswith("-"):
                    flag = rest.pop(0)
                    if flag in ("-e", "--env", "-w", "--workdir", "-u", "--user"):
                        if rest:
                            rest.pop(0)
                if rest:
                    rest.pop(0)  # container name
                if not rest:
                    return _orig_subprocess_run(["true"], **{k: v for k, v in kwargs.items() if k != "args"})
                new_args = (rest,) + tuple(args[1:])
                return _orig_subprocess_run(*new_args, **kwargs)
            if sub == "info":
                # Faux succès : laisser les tests croire que docker marche
                # (ils enchaînent ensuite avec `docker exec NAME cmd...` qui
                # est lui aussi shimé). Sans ce returncode=0, _docker_cmd_or_none()
                # de test_calypso_milestones renvoie None et tous les tests skip.
                return subprocess.CompletedProcess(cmd, 0, stdout="Server:\n", stderr="")
            if sub == "inspect":
                # In-container = container tourne forcément (on est dedans).
                # Si le test demande `-f "{{.State.Running}}"`, répondre "true".
                if any(a == "{{.State.Running}}" for a in cmd):
                    return subprocess.CompletedProcess(cmd, 0, stdout="true\n", stderr="")
                if any(a == "{{.State.Pid}}" for a in cmd):
                    return subprocess.CompletedProcess(cmd, 0, stdout="1\n", stderr="")
                return subprocess.CompletedProcess(cmd, 0, stdout="[]\n", stderr="")
            if sub == "ps":
                return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")
            if sub == "cp":
                # `docker cp` dans le container : copie locale simple
                src = cmd[2].split(":", 1)[-1] if ":" in cmd[2] else cmd[2]
                dst = cmd[3].split(":", 1)[-1] if ":" in cmd[3] else cmd[3]
                return _orig_subprocess_run(["cp", "-a", src, dst], **kwargs)
            # Toutes les autres subcommands docker (info, version, image, …) :
            # retourne exit≠0 SANS appeler le vrai binaire (qui n'existe pas
            # dans le container), pour que les tests qui détectent docker via
            # `subprocess.run([...]).returncode != 0` skippent proprement.
            return subprocess.CompletedProcess(
                cmd, 1, stdout="",
                stderr=f"docker shim: subcommand '{sub}' not supported in-container\n")
        return _orig_subprocess_run(*args, **kwargs)

    subprocess.run = _calypso_local_run


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
        # test_layer_drift.py
        "drift:                drift temporel inter-couches via timestamps logs",
        # test_timer_invariants.py
        "timer_invariant:      compteurs timers QEMU ([tdma]/[frame_irq]/[kick]) + CSV log_timeline",
        # test_ar_imr_inth_invariants.py (2026-05-25)
        "ar_imr_invariant:     invariants DSP init silicon (SP/AR/IMR) + IRQ servicing + bootstub regression",
        # test_gdb_stub.py (Phase 2)
        "runtime_gdb:          GDB stub :1234 — handshake, regs, mem, PC, NDB",
        # extension monitor (Phase 2)
        "runtime_monitor:      QEMU monitor HMP — info status/chardev/qtree/mtree/qom-tree",
        # test_irda_channel.py (Phase 2)
        "runtime_irda:         canal IrDA fw debug — PTY, boot marker, throughput",
        # compléments observabilité (Phase 2)
        "runtime_net:          ports listening QEMU (1234 gdb, 4247 mobile VTY)",
        "runtime_fs:           FD usage qemu, log size cap, disk space container",
        # test_log_grep.py (Phase 2)
        "runtime_log_grep:     grep invariants + blockers sur tous les logs (qemu/bridge/osmocon/mobile/fw-irda)",
        # test_firmware_state.py
        "runtime_firmware:     état firmware live (PC, rxDoneFlag, pas de busy-wait)",
        "runtime_osmocon:      progression osmocon (download → L1CTL/Layer 1, pas de LOST spam)",
        # test_osmocom_workflow.py — alignement workflow OsmocomBB
        "osmocom_compliant:    point du workflow OsmocomBB respecté par notre QEMU",
        "osmocom_divergent:    point divergent (workflow non respecté ; xfail attendu)",
        "osmocom_sim:          sémantique SIM controller (IT bits, FIFO, ATR)",
        "osmocom_clock:        alignement clock domains (VIRTUAL vs REALTIME)",
        "osmocom_bridge:       bridge.py timing et CLK IND jitter",
        "osmocom_boot:         séquence boot ARM/DSP + handshake",
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
        if not (rep.when == "setup" and rep.outcome == "skipped"):
            return
    # Extract short error message if test failed/skipped — for annex section.
    err_short = ""
    err_long  = ""
    if rep.outcome in ("failed", "skipped") or hasattr(rep, "wasxfail"):
        try:
            lr = getattr(rep, "longreprtext", "") or str(getattr(rep, "longrepr", "") or "")
            # Garder la 1ère ligne non-vide significative (souvent AssertionError msg)
            for line in lr.splitlines():
                s = line.strip()
                if s and not s.startswith(("def ", "_ ", "@", ">")):
                    err_short = s[:200]
                    break
            if not err_short and lr:
                err_short = lr.strip().splitlines()[0][:200]
            # Stack trace complet (cap à 4 KB pour ne pas exploser le report
            # quand un test plante violemment avec un long traceback)
            err_long = lr[:4096]
        except Exception:
            pass
    _MERMAID_RESULTS.append({
        "nodeid": item.nodeid,
        "name": item.name,
        "outcome": rep.outcome,
        "markers": sorted({m.name for m in item.iter_markers()}),
        "duration_s": getattr(rep, "duration", 0.0),
        "wasxfail": hasattr(rep, "wasxfail"),
        "err_short": err_short,
        "err_long":  err_long,
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
    # Drift inter-couches
    "drift":                 ("Timing", "Drift"),
    # Timer counters
    "timer_invariant":       ("Timing", "Timers"),
    # Phase 2 : GDB stub / QEMU monitor / IrDA / net / fs
    "runtime_gdb":           ("Runtime", "GDB-introspect"),
    "runtime_monitor":       ("Runtime", "Monitor-extended"),
    "runtime_irda":          ("Runtime", "IrDA-channel"),
    "runtime_net":           ("Runtime", "Net"),
    "runtime_fs":            ("Runtime", "FS"),
    "runtime_log_grep":      ("Runtime", "Log-grep"),
    "runtime_firmware":      ("Runtime", "Firmware-state"),
    "runtime_osmocon":       ("Runtime", "Osmocon"),
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
    "Infra",            # processus container alive
    "GDB-introspect",   # GDB stub :1234 — surface d'introspection ARM
    "Monitor-extended", # QEMU monitor HMP — état QEMU lui-même
    "Firmware-state",   # PC ARM bouge, pas de busy-wait SIM
    "Osmocon",          # romload download → sercomm L1CTL passthrough
    "DSP",              # DSP boot / IRQ / FB-det converge
    "L1-data",          # BSP DMA / bursts DL bridge
    "L1-ctrl",          # SERCOMM L1CTL bus
    "Timers",           # TDMA / TINT0 / kick
    "Drift",            # drift inter-couches
    "NDB",              # injection NDB direct via gdb-stub
    "ARM-feedback",     # ARM L1 lit les cells injectées
    "IrDA-channel",     # canal IrDA fw debug — diagnostique le mur task=24→DATA_IND=0
    "L3-MM",            # mobile fait LU jusqu'au bout
    "Mgmt",             # VTY osmocom/mobile
    "Net",              # ports listening
    "FS",               # FD/disk/log size
    "Log-grep",         # grep invariants + blockers sur tous les logs
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
        "  classDef brk  fill:#ff5252,stroke:#990000,color:#fff,stroke-width:3px;",
        "  ROOT[Test Run]",
    ]
    if not _MERMAID_RESULTS:
        lines.append("  ROOT --> EMPTY[no tests collected]")
        return "\n".join(lines)

    # tree[category][layer] = [tests…]
    tree: dict[str, dict[str, list[dict]]] = {}
    for r in _MERMAID_RESULTS:
        cat, layer = _classify(r["markers"])
        r["category"] = cat
        r["layer"] = layer
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
            lines.append(f"    class {bid} brk;")
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
# YAML front-matter Quarto (RStudio render natif mermaid).
# include-after-body : injecte svg-pan-zoom pour ajouter boutons +/−/reset
# sur chaque SVG mermaid + drag pour pan + scroll-wheel pour zoom.
_QMD_FRONTMATTER = """\
---
title: "Calypso test report"
author: "auto-generated (tests/conftest.py)"
date: today
date-format: "YYYY-MM-DD HH:mm"
format:
  html:
    toc: true
    toc-depth: 3
    toc-location: left
    code-fold: true
    code-tools: true
    theme:
      light: cosmo
      dark: darkly
    embed-resources: true
    include-after-body:
      text: |
        <script src="https://cdn.jsdelivr.net/npm/svg-pan-zoom@3.6.1/dist/svg-pan-zoom.min.js"></script>
        <script>
        window.addEventListener('load', () => {
          setTimeout(() => {
            document.querySelectorAll('pre.mermaid svg, .cell-output-display svg').forEach(svg => {
              if (svg.getAttribute('aria-roledescription') === 'flowchart-v2'
                  || svg.id?.startsWith('mermaid')
                  || svg.closest('pre.mermaid')) {
                svg.style.maxWidth = '100%';
                svg.style.height = '600px';
                svgPanZoom(svg, {
                  zoomEnabled: true,
                  controlIconsEnabled: true,
                  fit: true,
                  center: true,
                  minZoom: 0.3,
                  maxZoom: 20,
                });
              }
            });
          }, 1500);
        });
        </script>
  gfm:
    preview-mode: raw
mermaid:
  theme: default
execute:
  echo: false
  warning: false
---

"""

_REPORT_SKELETON = """\
# Calypso test report — {timestamp}

> Auto-generated by `tests/conftest.py::pytest_sessionfinish`.
> Pasteable directly in a GitHub issue/PR (Mermaid blocks render natively).

## Status global

{global_status_block}

## Variables d'environnement du run

Toutes les variables Calypso/pipeline manipulées par `run.sh` (extraites du
script + os.environ). `env` = passée explicitement au lancement ; `default` =
valeur fallback de `run.sh`.

{env_block}

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

## Diagramme détaillé — un graphe par catégorie

Chaque catégorie est rendue dans un graphe séparé pour lisibilité (un seul
gros graphe avec subgraphs imbriqués devient illisible >50 tests). Les
catégories sont indépendantes côté layout — pas de chaîne causale entre
elles dans le détail (voir le pipeline plus haut pour ça).

{detail_per_category}

## Skipped / xfailed

{skipped_block}

## Catalogue des tests

Liste exhaustive des tests exécutés ce run, groupés par marker pytest.
Source : nodeids pytest + outcome. Pour chaque test, le statut, la durée
et le fichier source.

{test_catalog}

## Cadence des logs et timers dans le temps

Compte des événements par bucket {log_timeline_bucket}s wall, généré par
`log_timeline.py` sur les logs préfixés `<epoch_sec> +<rel_sec>s` par
`run.sh`. Source : `log_timeline.csv` dans ce dossier. Le plot est produit
côté RStudio/Quarto via le chunk R ci-dessous (no-op en markdown GitHub —
ouvrir le `.qmd` dans RStudio ou lancer `quarto render` pour le rendu).

```{{r log-timeline, fig.cap="Cadence logs (sources) + timers (dé-thinned vs nominal GSM)", fig.width=12, fig.height=9, echo=FALSE, message=FALSE, warning=FALSE}}
if (!requireNamespace("ggplot2", quietly=TRUE) ||
    !requireNamespace("tidyr",  quietly=TRUE)) {{
  message("ggplot2/tidyr absent — `install.packages(c('ggplot2','tidyr'))` dans RStudio")
}} else {{
  library(ggplot2); library(tidyr)
  df <- read.csv("log_timeline.csv")

  # Plot 1 — log volume per source (events/s)
  src_cols <- c("qemu", "bridge", "osmocon", "mobile")
  p1_data <- pivot_longer(df[, c("t_rel", src_cols)], cols=-t_rel,
                          names_to="source", values_to="events")
  p1_data$rate <- p1_data$events / {log_timeline_bucket}
  p1 <- ggplot(p1_data, aes(t_rel, rate, color=source)) +
    geom_line(linewidth=0.6) + scale_y_log10() +
    labs(title="Cadence brute des logs (events/s)",
         x="t_rel (s)", y="events / s (log scale)") +
    theme_minimal()

  # Plot 2 — timer rates dé-thinned vs nominal GSM
  # tdma & frame_irq loggués 1/1000 ; kick 1/200
  thinning <- c(tdma=1000, frame_irq=1000, kick=200)
  nominal  <- c(tdma=216.7, frame_irq=216.7, kick=200.0)
  tcols <- c("tdma", "frame_irq", "kick")
  p2_data <- pivot_longer(df[, c("t_rel", tcols)], cols=-t_rel,
                          names_to="timer", values_to="events")
  p2_data$rate <- (p2_data$events / {log_timeline_bucket}) *
                  thinning[p2_data$timer]
  nom_df <- data.frame(timer=tcols, nominal=as.numeric(nominal[tcols]))
  p2 <- ggplot(p2_data, aes(t_rel, rate, color=timer)) +
    geom_line(linewidth=0.8) +
    geom_hline(data=nom_df, aes(yintercept=nominal, color=timer),
               linetype="dashed", alpha=0.6) +
    scale_y_log10() +
    labs(title="Timers QEMU : mesurée (lignes) vs nominale (pointillés)",
         subtitle="dé-thinned ×1000 (tdma/frame_irq) ×200 (kick)",
         x="t_rel (s)", y="timer events / s (log scale)") +
    theme_minimal()

  # Plot 3 — DSP signals
  dsp_cols <- c("fb_det_hit", "stack_in_ndb")
  p3_data <- pivot_longer(df[, c("t_rel", dsp_cols)], cols=-t_rel,
                          names_to="signal", values_to="events")
  p3_data$rate <- p3_data$events / {log_timeline_bucket}
  p3 <- ggplot(p3_data, aes(t_rel, rate, color=signal)) +
    geom_line(linewidth=0.6) + scale_y_log10() +
    labs(title="DSP signals (fb-det convergence + stack runaway)",
         x="t_rel (s)", y="events / s (log scale)") +
    theme_minimal()

  # Stack vertical
  if (requireNamespace("patchwork", quietly=TRUE)) {{
    library(patchwork); print(p1 / p2 / p3)
  }} else {{
    print(p1); print(p2); print(p3)
  }}
}}
```

<details>
<summary>Table brute par bucket {log_timeline_bucket}s — cliquer pour déplier</summary>

```
{log_timeline_table}
```

</details>

## Résultats bruts pytest

<details>
<summary>Cliquer pour déplier — sortie verbatim type <code>pytest -v</code></summary>

```
{raw_pytest_block}
```

</details>

## Plan IrDA debug channel (Phase 2)

Statut auto-détecté depuis l'état du run.

{irda_plan_status}

Réf. `PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md`.

## Chapitre — Blockers détaillés

Pour chaque test en échec : catégorie, couche, marker, message d'assertion,
audit Python dynamique (greps des keywords du message contre tous les logs
container), et stack trace complet dépliable.

{blockers_chapter}

## Annexe — Audit indépendant (`abstract.py`)

Re-compute peer-level produit par `abstract.py` (script racine du repo) à
partir de `results.json` + `log_timeline.csv` du dossier de run. Ne fait pas
confiance au markdown généré — donne une 2ème lecture indépendante des
mêmes données.

<details markdown="1"><summary>Déplier — audit `abstract.py`</summary>

{abstract_audit}

</details>

## Annexe — Diag snapshot (état runtime au moment des tests)

Snapshot rapide produit par `make_diag.sh` au cours de la session pytest.
Pour un bundle complet (tar.gz avec tous les logs filtrés + dumps DSP), utiliser
`./make_diag_bundle.sh`.

<details markdown="1"><summary>Déplier — diag snapshot</summary>

{diag_snapshot}

</details>

## Annexe — Bundle make_diag_bundle.sh

Bundle généré pendant la session via `make_diag_bundle.sh` (inventaire + digests
texte embarqués). Les logs bruts (qemu_diag, bridge, osmocon, etc.) sont listés
seulement — récupérer le tar.gz pour les contenus complets.

<details markdown="1"><summary>Déplier — bundle make_diag_bundle.sh</summary>

{diag_bundle_annex}

</details>

## Annexe — Détail complet de tous les tests

<details markdown="1"><summary>Déplier — table complète de tous les tests</summary>

{full_test_annex}

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
    """GitHub-flavored alert + summary table + métrique fonctionnelle honnête.

    Trois métriques sont affichées :
      - pct_brut : passed / total (inclut xfailed/skipped — minore le score)
      - pct_fonctionnel : passed / (passed + failed) — exclut les "known broken"
        et les "not applicable" pour donner le vrai taux des tests qui prétendent
        valider quelque chose de fonctionnel
      - pct_actionnable : (passed + failed) / total — proportion de tests qui ne
        sont ni skipped ni xfailed (sinon, mémo-déclaratif vs bug actif)
    """
    total = sum(counts.values()) or 1
    p = counts.get("passed", 0)
    f = counts.get("failed", 0)
    s = counts.get("skipped", 0)
    x = counts.get("xfailed", 0)
    pct_brut = 100 * p / total
    actionable = p + f
    pct_fct = 100 * p / actionable if actionable else 0.0
    pct_act = 100 * actionable / total
    if f == 0 and p > 0:
        alert_kind = "TIP"
        verdict = f"✅ **ALL PASS** — {p}/{total} ({pct_brut:.0f} %)"
    elif p == 0:
        alert_kind = "CAUTION"
        verdict = f"🛑 **BLOCKED** — aucun test ne passe ({total} essais)"
    elif f > 0:
        alert_kind = "WARNING"
        verdict = f"⚠️ **PARTIAL** — {f} échec(s), {p}/{total} passent"
    else:
        alert_kind = "NOTE"
        verdict = f"❔ état indéterminé — {p}/{total}"
    return (
        f"> [!{alert_kind}]\n> {verdict}\n\n"
        f"| métrique | valeur | interprétation |\n|---|---:|---|\n"
        f"| pct brut       | {pct_brut:.0f} % | `passed / total` (inclut xfail+skip — minore) |\n"
        f"| **pct fonctionnel** | **{pct_fct:.0f} %** | `passed / (passed+failed)` — vrai taux des tests qui prétendent valider |\n"
        f"| pct actionnable | {pct_act:.0f} % | `(passed+failed) / total` — non-xfailed/non-skipped |\n\n"
        f"| résultat | nombre |\n|---|---:|\n"
        f"| ✅ passed | {p} |\n| ❌ failed | {f} |\n"
        f"| ⏭️ skipped | {s} |\n| ⚠️ xfailed | {x} |\n| **total** | **{total}** |\n"
    )


def _gen_pipeline_commentary(flat_layer: dict) -> str:
    """Texte interpretatif autour du pipeline mermaid.

    Note importante : `_PIPELINE_ORDER` n'est pas une dépendance causale
    stricte. Plusieurs layers peuvent avoir des dépendances "amont" qui
    ne sont pas reflétées par leur position. Exemple connu :
      `NDB` (couche d'injection mémoire via gdb-stub) est *amont* de
      `ARM-feedback` même si NDB apparaît plus haut dans la liste —
      ARM-feedback observe ce qui a été écrit par NDB, donc NDB partial
      → ARM-feedback fail mécanique. Si on voit ce pattern, on l'annote.
    """
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
            f"➡️ **Première rupture (linéaire) : `{first_fail_layer[0]}`** "
            f"({first_fail_layer[1]}/{first_fail_layer[2]} tests passent)."
        )
        if last_pass_layer:
            parts.append(
                f"Les étapes jusqu'à `{last_pass_layer}` sont OK ; "
                f"le bug à investiguer côté pipeline linéaire est dans la transition "
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

    # Dépendance non-linéaire connue : NDB → ARM-feedback → L1-ctrl
    # Si NDB est partial/fail ET ARM-feedback fail, signaler que la
    # vraie chaîne causale n'est pas le pipeline linéaire.
    ndb = flat_layer.get("NDB", [])
    arm_fb = flat_layer.get("ARM-feedback", [])
    if ndb and arm_fb:
        ndb_st, ndb_p, ndb_n = _layer_status(ndb)
        arm_st, _, _ = _layer_status(arm_fb)
        if ndb_st in ("partial", "fail") and arm_st == "fail":
            parts.append(
                "\n> [!IMPORTANT]\n"
                "> ⚠️ **Chaîne causale sérielle non-linéaire détectée** :\n"
                f"> `NDB` ({ndb_p}/{ndb_n}, {ndb_st}) → `ARM-feedback` (fail) → `L1-ctrl` (downstream).\n"
                "> \n"
                "> NDB est *amont* d'ARM-feedback malgré sa position plus haut dans le diagramme.\n"
                "> ARM-feedback observe ce que NDB a écrit — si l'injection NDB échoue, ARM-feedback\n"
                "> fail mécanique, et L1-ctrl en aval n'a rien à forward.\n"
                "> \n"
                "> **Ordre d'investigation recommandé** : commencer par les `test_inject_*` (NDB).\n"
                "> Les durées 0.3-0.6s (fail rapide) suggèrent soit erreur immédiate d'injection,\n"
                "> soit checkpoint post-write qui ne valide pas. Si tu fixes le path d'injection,\n"
                "> tu débloques potentiellement toute la chaîne en aval.\n"
            )

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
        err = (r.get("err_short") or "").strip()
        parts.append(
            f"{prefix} **{i}. `{r['name']}`** — {cat} / {layer} — "
            f"marker(s): {','.join(r['markers']) or '_aucun_'} — "
            f"durée {r['duration_s']:.2f}s\n  - `{r['nodeid']}`"
        )
        if err:
            parts.append(f"  - ❗ `{err}`")
    return "\n".join(parts)


def _gen_env_block() -> str:
    """Liste **toutes** les variables d'environnement que `run.sh` manipule
    (extrait du run.sh lui-même via parsing `X="${X:-default}"`). Pour chaque :
    valeur courante (si dans os.environ) sinon valeur par défaut, et la source
    (`env` = passée explicitement, `default` = fallback de run.sh).
    """
    import re as _re_env
    PREFIXES = ("CALYPSO_", "BRIDGE_", "FW_", "IRDA_", "QEMU_")
    run_sh = Path(__file__).resolve().parent.parent / "run.sh"
    run_vars: dict = {}
    if run_sh.exists():
        # Pattern : NAME="${NAME:-default}"  (avec ou sans quotes finales)
        pat = _re_env.compile(
            r'^([A-Z][A-Z0-9_]*)=\"?\$\{\1:-([^}]*)\}\"?\s*$',
            _re_env.MULTILINE)
        for m in pat.finditer(run_sh.read_text()):
            name, default = m.group(1), m.group(2)
            if name.startswith(PREFIXES):
                run_vars[name] = default
    # Union : noms définis dans run.sh + noms dans environ commençant par PREFIXES
    all_names = set(run_vars) | {k for k in os.environ if k.startswith(PREFIXES)}
    if not all_names:
        return "_Aucune variable Calypso/pipeline détectée._\n"
    rows = []
    for name in sorted(all_names):
        if name in os.environ:
            v = os.environ[name]
            src = "env"
        else:
            v = run_vars.get(name, "(unset)")
            src = "default"
        if len(v) > 200:
            v = v[:197] + "..."
        v_md = v.replace("|", "\\|")
        rows.append(f"| `{name}` | `{v_md}` | {src} |")
    return ("| Variable | Valeur | Source |\n|---|---|---|\n"
            + "\n".join(rows) + "\n")


def _gen_machine_report(md_full: str) -> str:
    """Distille le rapport markdown complet en un `report.md` LLM-friendly :
    on conserve uniquement les sections utiles à un agent (Stats, Pipeline,
    Blockers, Chapitre blockers détaillés, Audit indépendant, Diag snapshot,
    Plan IrDA, Log invariants si présent). On supprime mermaid blocks (illisible
    en texte), `<details>` (verbose), et sections décoratives (Catalogue, Skip
    list, Reproduction dupliquée, Détail complet de tous les tests, Bundle
    annexe). Préfixe : « machine-friendly ».
    """
    SKIP_SECTIONS = (
        "Diagramme détaillé",
        "Skipped / xfailed",
        "Catalogue des tests",
        "Cadence des logs",
        "Résultats bruts pytest",
        "Annexe — Bundle make_diag_bundle.sh",
        "Annexe — Détail complet de tous les tests",
        "Reproduction",
    )
    lines = md_full.splitlines()
    out, keep = [], True
    in_mm, in_det = False, False
    det_depth = 0  # supporte <details> imbriqués

    for line in lines:
        stripped = line.strip()
        # Mermaid blocks
        if stripped.startswith("```mermaid") or stripped.startswith("```{mermaid}"):
            in_mm = True
            continue
        if in_mm:
            if stripped == "```":
                in_mm = False
            continue
        # <details>…</details> imbriqués
        if "<details" in line.lower():
            det_depth += line.lower().count("<details")
            in_det = det_depth > 0
            continue
        if "</details>" in line.lower():
            det_depth -= line.lower().count("</details>")
            if det_depth <= 0:
                det_depth = 0
                in_det = False
            continue
        if in_det:
            continue
        # Niveau 2 = nouvelle section
        if line.startswith("## "):
            section = line[3:].strip()
            keep = not any(section.startswith(k) for k in SKIP_SECTIONS)
        if keep:
            out.append(line)

    body = "\n".join(out).rstrip() + "\n"
    header = ("> _Rapport condensé (machine-friendly) extrait de "
              "`test_results.md`. Pour la version complète : voir le `.md` ou "
              "`.qmd` à côté._\n\n")
    return header + body


def _audit_blocker(r: dict) -> str:
    """Audit Python dynamique d'un blocker : extrait les keywords distinctifs
    du message d'assertion, puis grep ces keywords dans tous les logs du
    container. Retourne un tableau markdown des comptes + dernier match.
    """
    import re as _re_a
    import shlex as _sh
    import subprocess as _sp_a
    err = (r.get("err_short") or "").strip()
    if not err:
        return ""
    # FIX 2026-05-24 : prioriser les VALEURS du message (key=value, hex
    # addresses, decimal nums) sur les tokens génériques (qui pèchent les
    # noms de tests / fichiers .py / classes Error). L'ancien algo greppait
    # `test_qemu_insn_rate_p1_above_1m` partout et retournait 0 — bruit.
    # Strip d'abord la 1ère ligne avec le node_id pytest.
    err_clean = "\n".join(
        l for l in err.splitlines()
        if not _re_a.match(r"^\s*[\w/]+\.py[:\d]*\s*(in\s+test_)?", l)
    )
    seen = set()
    kws = []
    # 1. key=value (le plus discriminant : task=24, rate=12345, fn=N)
    for k, v in _re_a.findall(r'\b([a-z_][a-z_0-9]*)=(\w+)', err_clean):
        kv = f"{k}={v}"
        if kv not in seen and len(kws) < 4:
            seen.add(kv); kws.append(kv)
    # 2. hex addresses (0x...)
    for h in _re_a.findall(r'\b(0x[0-9a-fA-F]{3,})\b', err_clean):
        if h not in seen and len(kws) < 6:
            seen.add(h); kws.append(h)
    # 3. tokens 4+ chars (filet de sécurité) — EXCLURE test names + modules
    STOP = {"assert", "True", "False", "None", "self", "test", "tests",
            "AssertionError", "ValueError", "TypeError", "Error",
            "Exception", "RuntimeError", "KeyError", "IndexError",
            "py", "pytest", "Traceback", "File", "line", "module"}
    for tok in _re_a.findall(r'\b([A-Za-z_][A-Za-z0-9_]{3,})\b', err_clean):
        if tok in STOP or tok in seen:
            continue
        if tok.startswith("test_") or tok.endswith("_test") or "." in tok:
            continue
        seen.add(tok); kws.append(tok)
        if len(kws) >= 8:
            break
    if not kws:
        return ""
    LOGS = [("qemu", "/root/qemu.log"),
            ("bridge", "/tmp/bridge.log"),
            ("osmocon", "/tmp/osmocon.log"),
            ("mobile", "/tmp/mobile.log"),
            ("bts", "/tmp/bts.log"),
            ("fw-irda", "/tmp/fw-irda.log")]
    # Une seule commande docker exec pour tout : économise le RTT
    parts_bash = []
    for kw in kws:
        for label, path in LOGS:
            parts_bash.append(
                f"echo '___ROW___|{kw}|{label}|'$(grep -c -F {_sh.quote(kw)} {path} 2>/dev/null || echo 0)"
            )
    bash_cmd = " ; ".join(parts_bash)
    try:
        rb = _sp_a.run(["docker", "exec", "trying", "bash", "-c", bash_cmd],
                       capture_output=True, text=True, timeout=15)
        rows = {}
        for line in rb.stdout.splitlines():
            if line.startswith("___ROW___|"):
                _, kw, lbl, cnt = line.split("|", 3)
                rows.setdefault(kw, {})[lbl] = cnt.strip() or "0"
    except Exception as e:
        return f"_Audit dynamique : exception {type(e).__name__}._\n"
    out = ["**Audit dynamique** (greps `docker exec` des keywords du message"
           " d'assertion contre les logs du run actif) :", "",
           "| Keyword | qemu | bridge | osmocon | mobile | bts | fw-irda |",
           "|---|---:|---:|---:|---:|---:|---:|"]
    for kw in kws:
        r2 = rows.get(kw, {})
        out.append("| `{}` | {} | {} | {} | {} | {} | {} |".format(
            kw,
            r2.get("qemu", "?"), r2.get("bridge", "?"),
            r2.get("osmocon", "?"), r2.get("mobile", "?"),
            r2.get("bts", "?"),    r2.get("fw-irda", "?")))
    return "\n".join(out) + "\n"


def _gen_blockers_chapter(results: list[dict]) -> str:
    """Chapitre dédié : un sous-chapitre par test en échec avec assertion
    courte + audit Python dynamique + traceback complet en `<details>`.
    """
    failed = [r for r in results if r["outcome"] == "failed" and not r["wasxfail"]]
    if not failed:
        return "_Aucun test en échec — pas de chapitre à générer._\n"
    layer_order = {layer: i for i, layer in enumerate(_PIPELINE_ORDER)}
    def _key(r):
        cat, layer = _classify(r["markers"])
        return (layer_order.get(layer, 999), layer, r["nodeid"])
    parts = [f"_{len(failed)} test(s) en échec, triés par ordre pipeline (le premier"
             f" est l'upstream le plus en amont). Chaque entrée inclut un audit"
             f" dynamique des keywords du message d'assertion._\n"]
    for i, r in enumerate(sorted(failed, key=_key), 1):
        cat, layer = _classify(r["markers"])
        err  = (r.get("err_short") or "").strip()
        long = (r.get("err_long")  or "").strip()
        parts.append(f"### 🛑 {i}. `{r['name']}`\n")
        parts.append(f"- **Catégorie / couche** : {cat} / {layer}")
        parts.append(f"- **Marker(s)** : {','.join(r['markers']) or '_aucun_'}")
        parts.append(f"- **Durée** : {r['duration_s']:.2f}s")
        parts.append(f"- **Nodeid** : `{r['nodeid']}`")
        if err:
            parts.append(f"- **Assertion** : `{err}`")
        audit = _audit_blocker(r)
        if audit:
            parts.append("")
            parts.append(audit)
        if long:
            parts.append("")
            parts.append("<details><summary>Stack trace complet (dépliable)</summary>")
            parts.append("")
            parts.append("```")
            parts.append(long)
            parts.append("```")
            parts.append("")
            parts.append("</details>")
        parts.append("")
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


def _gen_test_catalog(results: list[dict]) -> str:
    """Build a markdown catalogue : table per marker listing all tests + status."""
    if not results:
        return "_(no tests collected)_"
    icons = {"passed": "✅", "failed": "❌", "skipped": "⏭️"}
    by_marker: dict[str, list[dict]] = {}
    for r in results:
        keys = r["markers"] or ["unmarked"]
        for k in keys:
            by_marker.setdefault(k, []).append(r)
    out = []
    for marker in sorted(by_marker):
        rs = by_marker[marker]
        passed = sum(1 for r in rs if r["outcome"] == "passed")
        out.append(f"### `{marker}` — {passed}/{len(rs)}\n")
        out.append("| Status | Test | Durée | Fichier |")
        out.append("|---|---|---:|---|")
        for r in sorted(rs, key=lambda x: (x["outcome"], x["name"])):
            icon = "⚠️" if r["wasxfail"] else icons.get(r["outcome"], "?")
            status = "xfail" if r["wasxfail"] else r["outcome"]
            src = r["nodeid"].split("::")[0]
            out.append(f"| {icon} {status} | `{r['name']}` | {r['duration_s']:.2f}s | `{src}` |")
        out.append("")
    return "\n".join(out)


def _gen_abstract_audit(folder: Path) -> str:
    """Annexe audit : exécute `abstract.py <folder>` qui recompute des stats
    peer-level depuis `results.json` + `log_timeline.csv`. Sortie console
    embarquée en bloc code.
    """
    import subprocess as _sp
    script = Path(__file__).resolve().parent.parent / "abstract.py"
    if not script.exists():
        return "_`abstract.py` introuvable à la racine du repo._\n"
    try:
        r = _sp.run(["python3", str(script), str(folder)],
                    capture_output=True, text=True, timeout=30)
        body = r.stdout or "(no output)"
        if r.returncode != 0:
            body += f"\n\n[stderr]\n{r.stderr[:600]}"
        return f"```\n{body}\n```\n"
    except Exception as e:
        return f"_exception en exécutant `abstract.py` : {type(e).__name__}: {e}_\n"


def _gen_diag_bundle_annex() -> str:
    """Annexe bundle : appelle `make_diag_bundle.sh` puis embarque les
    *digests* texte du tar (parse_summary, source_excerpts, static_decode,
    env_boot) dans des `<details>` markdown. Les logs bruts (qemu_diag,
    bridge, osmocon, mobile, fw-irda, frame_irq, pc_hist) sont seulement
    listés avec leur taille — pas embarqués, pour ne pas exploser le report.
    Le tar complet reste dispo via `./make_diag_bundle.sh` à la main.
    """
    import os as _os
    import subprocess as _sp
    import tarfile as _tar
    import tempfile as _tmp
    script = Path(__file__).resolve().parent.parent / "make_diag_bundle.sh"
    if not script.exists():
        return "_`make_diag_bundle.sh` introuvable à la racine du repo._\n"
    try:
        td = _tmp.mkdtemp(prefix="diag_bundle_annex_")
        env = {**_os.environ, "OUT_DIR": td, "TAG": "annex"}
        r = _sp.run(["bash", str(script)], env=env,
                    capture_output=True, text=True, timeout=180)
        if r.returncode != 0:
            return (f"_make_diag_bundle.sh exit={r.returncode}._\n\n"
                    f"```\n{r.stderr[:800]}\n```\n")
        tar_files = sorted(Path(td).glob("*.tar.gz"))
        if not tar_files:
            return "_aucun tarball produit (script terminé sans erreur)._\n"
        tarball = tar_files[0]
        out = [f"_Bundle complet : `{tarball.name}` "
               f"({tarball.stat().st_size:,} bytes). Régénérer à la main : "
               f"`./make_diag_bundle.sh`._\n"]
        with _tar.open(tarball) as tf:
            members = sorted(tf.getmembers(), key=lambda m: m.name)
            # Inventaire
            out.append("\n### Inventaire du bundle\n")
            out.append("| Fichier | Bytes |")
            out.append("|---|---:|")
            for m in members:
                if m.isfile():
                    out.append(f"| `{m.name}` | {m.size:,} |")
            out.append("")
            # TOUS les logs bruts en <details> dépliables
            out.append("\n### Logs bruts (dépliables)\n")
            for m in members:
                if not m.isfile():
                    continue
                fp = tf.extractfile(m)
                if fp is None:
                    continue
                try:
                    txt = fp.read().decode("utf-8", errors="replace")
                except Exception:
                    continue
                lines = txt.splitlines()
                n = len(lines)
                # Tronquage proportionnel : gros fichiers = head + tail
                if m.size > 500_000:
                    body = ("\n".join(lines[:300]) +
                            f"\n\n[... {n-400} lines truncated "
                            f"(file too large : {m.size:,} bytes) ...]\n\n" +
                            "\n".join(lines[-100:]))
                elif m.size > 200_000:
                    body = ("\n".join(lines[:500]) +
                            f"\n\n[... {n-700} lines truncated ...]\n\n" +
                            "\n".join(lines[-200:]))
                else:
                    body = txt
                out.append(f"<details><summary><code>{m.name}</code> "
                           f"({m.size:,} bytes, {n} lignes)"
                           f"</summary>\n\n```\n{body}\n```\n\n</details>\n")
        return "\n".join(out)
    except Exception as e:
        return f"_exception en générant l'annexe bundle : {type(e).__name__}: {e}_\n"


def _gen_diag_snapshot() -> str:
    """Annexe diag : snapshot rapide de l'état du run au moment des tests
    (processes, log sizes, grep counts, blockers, tail des logs critiques).

    Délègue à `make_diag.sh` à la racine du repo. Si le script est absent ou
    Docker indisponible, retourne un placeholder explicite.
    """
    import subprocess as _sp
    script = Path(__file__).resolve().parent.parent / "make_diag.sh"
    if not script.exists():
        return "_diag : `make_diag.sh` introuvable à la racine du repo._\n"
    try:
        r = _sp.run(["bash", str(script)],
                    capture_output=True, text=True, timeout=20)
        out = r.stdout
        if r.returncode != 0:
            out += f"\n_diag : exit code {r.returncode}, stderr :_\n```\n{r.stderr[:600]}\n```\n"
        return out or "_diag : `make_diag.sh` n'a rien produit._\n"
    except Exception as e:
        return f"_diag : exception en appelant `make_diag.sh` — {type(e).__name__}: {e}_\n"


def _gen_full_annex(results: list[dict]) -> str:
    """Annexe complète : un tableau exhaustif de tous les tests avec
    nodeid, marker, outcome, durée, et message d'erreur court si fail/skip.

    Format markdown auto-pasteable. Trie par catégorie/couche puis par nom.
    """
    if not results:
        return "_(no tests collected)_"
    rows_by_cat: dict[str, list[dict]] = {}
    for r in results:
        cat, layer = _classify(r["markers"])
        key = f"{cat} / {layer}"
        rows_by_cat.setdefault(key, []).append(r)
    out = []
    icons = {"passed": "✅", "failed": "❌", "skipped": "⏭️"}
    for key in sorted(rows_by_cat):
        rs = rows_by_cat[key]
        passed = sum(1 for r in rs if r["outcome"] == "passed")
        out.append(f"### {key} — {passed}/{len(rs)}\n")
        out.append("| | Test | Markers | Durée | Erreur / raison (si non-pass) |")
        out.append("|---|---|---|---:|---|")
        for r in sorted(rs, key=lambda x: (x["outcome"], x["name"])):
            icon = "⚠️" if r["wasxfail"] else icons.get(r["outcome"], "?")
            mark = ",".join(r["markers"]) or "—"
            err = (r.get("err_short", "") or "").replace("|", "\\|").replace("\n", " ")
            if not err and r["outcome"] == "passed" and not r["wasxfail"]:
                err = ""
            out.append(f"| {icon} | `{r['name']}` | {mark} | {r['duration_s']:.2f}s | {err} |")
        out.append("")
    # Also append a flat list with full nodeids for completeness
    out.append("### Tous les nodeids (référence)")
    out.append("")
    out.append("<details>")
    out.append("<summary>Cliquer pour déplier — liste exhaustive nodeid → outcome</summary>")
    out.append("")
    out.append("```")
    for r in sorted(results, key=lambda x: x["nodeid"]):
        st = "XFAIL" if r["wasxfail"] else r["outcome"].upper()
        out.append(f"{st:8} {r['nodeid']}")
    out.append("```")
    out.append("")
    out.append("</details>")
    return "\n".join(out)


def _gen_irda_plan_status() -> str:
    """Détecte le statut de chaque phase du plan IrDA depuis l'état du run.

    Cf. PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md.
    Statuts dérivés :
      - Phase 0 (reco) : toujours ✅ (déjà fait)
      - Phase 0.5 (smoke test) : ✅ si "fw-irda boot" trouvé dans /tmp/fw-irda.log
      - Phase 1+2 (QEMU+fw mapping) : ✅ caduc (déjà en place via UART_IRDA)
      - Phase 3 (capture host) : ✅ si /tmp/irda_capture.pid existe
      - Phase 4 (tests integration) : ✅ si test_irda_channel.py existe
      - Phase 5 (instrumentation fw) : ✅ si > 100 bytes/min dans fw-irda.log
      - Phase 6 (viz Quarto) : 🔧 manuel
    """
    import subprocess
    container = os.environ.get("CALYPSO_CONTAINER", "trying")
    inside = os.path.exists("/.dockerenv")

    def _cat_ct(path: str) -> str:
        if inside:
            try: return Path(path).read_text(errors="replace")
            except Exception: return ""
        try:
            r = subprocess.run(["docker", "exec", container, "cat", path],
                               capture_output=True, text=True, timeout=3)
            return r.stdout if r.returncode == 0 else ""
        except Exception:
            return ""

    def _exists_ct(path: str) -> bool:
        if inside:
            return Path(path).exists()
        try:
            r = subprocess.run(["docker", "exec", container, "test", "-e", path],
                               capture_output=True, timeout=3)
            return r.returncode == 0
        except Exception:
            return False

    # Phase 0.5
    fwirda_text = _cat_ct("/tmp/fw-irda.log")
    has_boot = "fw-irda boot" in fwirda_text or "boot OK" in fwirda_text
    # Phase 3
    has_capture_pid = _exists_ct("/tmp/irda_capture.pid")
    has_fwirda_log = _exists_ct("/tmp/fw-irda.log")
    # Phase 5 (proxy : taille fw-irda.log > 100 octets sur run > 1min)
    sustained = len(fwirda_text) > 100
    # Phase 4 (tests présents)
    has_tests = (Path(__file__).parent / "test_irda_channel.py").exists()

    rows = [
        ("Phase 0",   "Reconnaissance UART_IRDA mapped + fw driver",
         "✅ done", "QEMU `calypso_soc.c:231-247`, fw `compal_e88/init.c:105`"),
        ("Phase 0.5", "Smoke test cons_puts boot marker",
         "✅" if has_boot else "🔧 à faire",
         "ajouter `cons_puts(\"=== fw-irda boot OK ===\\r\\n\")` après `cons_bind_uart(UART_IRDA)`"),
        ("Phase 1",   "QEMU side mapping chardev → UART_IRDA",
         "⏸️ caduc", "déjà en place via `-serial pty -serial pty` + `serial_hd(1)`"),
        ("Phase 2",   "Firmware side wrapper IrDA",
         "⏸️ caduc", "déjà en place via driver osmocom-bb existant"),
        ("Phase 3",   "Capture host `irda_capture.py` → `/tmp/fw-irda.log`",
         "✅" if has_capture_pid else ("🟡 script créé" if has_fwirda_log else "🔧 lancer le script"),
         "`python3 tools/irda_capture.py /dev/pts/3 &` (auto-lancé par run.sh à terme)"),
        ("Phase 4",   "Tests integration (`test_irda_channel.py`, marker runtime_irda)",
         "✅" if has_tests else "🔧",
         "8 tests gradés selon état (boot/produces/throughput/...)"),
        ("Phase 5",   "Instrumentation fw `cons_puts(UART_IRDA, ...)` dans hot path",
         "✅" if sustained else "🔧 à faire",
         "diagnostiquer le mur `task=24 → DATA_IND=0` — events `EVT_TASK24_*`"),
        ("Phase 6",   "Plot Quarto stacked area des events fw",
         "🔧 manuel", "extension R chunk dans `log_timeline.csv` parser fw-irda events"),
    ]
    out = [
        "| Phase | Objectif | Statut | Action / notes |",
        "|---|---|---|---|",
    ]
    for ph, obj, st, act in rows:
        out.append(f"| **{ph}** | {obj} | {st} | {act} |")
    out.append("")
    out.append(f"_Détection auto basée sur : `/tmp/fw-irda.log` ({'présent' if has_fwirda_log else 'absent'}, "
               f"{len(fwirda_text)} bytes), `irda_capture.pid` ({'oui' if has_capture_pid else 'non'}), "
               f"boot marker ({'détecté' if has_boot else 'absent'})._")
    return "\n".join(out)


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


def _run_log_timeline(folder: Path, bucket_s: float = 10.0) -> tuple[str, bool]:
    """Run log_timeline.py and return (ascii_table, csv_in_folder).

    Le PNG est intentionnellement supprimé : le plot est généré côté RStudio
    via un chunk R dans test_results.qmd qui lit log_timeline.csv. Cette
    fonction copie juste le CSV dans `folder` pour que le chunk Quarto
    le trouve en relatif au .qmd.
    """
    container = os.environ.get("CALYPSO_CONTAINER", "trying")
    script_host = Path(__file__).resolve().parent.parent / "log_timeline.py"
    script_ct = "/opt/GSM/qemu-src/log_timeline.py"
    csv_inside = "/tmp/log_timeline.csv"
    inside = os.path.exists("/.dockerenv")

    try:
        if inside:
            r = subprocess.run(
                ["python3", str(script_host),
                 "--bucket-s", str(bucket_s), "--csv", csv_inside],
                capture_output=True, text=True, timeout=30)
        else:
            r = subprocess.run(
                ["docker", "exec", container, "python3", script_ct,
                 "--bucket-s", str(bucket_s), "--csv", csv_inside],
                capture_output=True, text=True, timeout=30)
            # Pull CSV via docker exec cat (docker cp ne voit pas /tmp tmpfs)
            try:
                out = subprocess.run(
                    ["docker", "exec", container, "cat", csv_inside],
                    capture_output=True, timeout=15)
                if out.returncode == 0 and out.stdout:
                    (folder / "log_timeline.csv").write_bytes(out.stdout)
            except Exception:
                pass
        ascii_out = r.stdout
    except Exception as e:
        return (f"(log_timeline.py failed: {e})", False)

    if inside and os.path.exists(csv_inside):
        try: (folder / "log_timeline.csv").write_bytes(Path(csv_inside).read_bytes())
        except Exception: pass

    return (ascii_out, (folder / "log_timeline.csv").exists())


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
    """Write mermaid diagrams + coherent markdown report + bundle zip.

    Skip output entirely if no test actually ran (e.g. `--collect-only` or
    a fully-deselected run). Rotate older bundles : keep latest N (default 5,
    override via `CALYPSO_TEST_KEEP`).
    """
    # 0. Skip if nothing ran — avoids polluting /tmp with empty bundles on
    #    every `--collect-only` invocation.
    if not _MERMAID_RESULTS:
        terminal = session.config.pluginmanager.get_plugin("terminalreporter")
        if terminal is not None:
            terminal.write_sep("=", "Mermaid report skipped (no tests ran)", purple=True)
        return

    out_dir = Path(os.environ.get("CALYPSO_TEST_OUT", "/tmp"))
    out_dir.mkdir(parents=True, exist_ok=True)

    # 0b. Rotate : delete old test_results_* artifacts before creating new one.
    keep_n = int(os.environ.get("CALYPSO_TEST_KEEP", "5"))
    old_dirs = sorted(out_dir.glob("test_results_2*"),
                      key=lambda p: p.stat().st_mtime, reverse=True)
    old_zips = sorted(out_dir.glob("test_results_2*.zip"),
                      key=lambda p: p.stat().st_mtime, reverse=True)
    for stale in old_dirs[keep_n:]:
        if stale.is_dir():
            shutil.rmtree(stale, ignore_errors=True)
        else:
            try: stale.unlink()
            except Exception: pass
    for stale in old_zips[keep_n:]:
        try: stale.unlink()
        except Exception: pass

    # Per-run folder + zip
    ts_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = out_dir / f"test_results_{ts_id}"
    folder.mkdir(parents=True, exist_ok=True)

    # Derive structures (annotation `category`/`layer` sur chaque record pour
    # que results.json soit consommable par abstract.py et autres outils
    # externes qui lisent t["layer"] directement).
    tree: dict[str, dict[str, list[dict]]] = {}
    for r in _MERMAID_RESULTS:
        cat, layer = _classify(r["markers"])
        r["category"] = cat
        r["layer"] = layer
        tree.setdefault(cat, {}).setdefault(layer, []).append(r)
    flat_layer: dict[str, list[dict]] = {}
    for cat, layers in tree.items():
        for layer, tests in layers.items():
            flat_layer.setdefault(layer, []).extend(tests)
    counts = {"passed": 0, "failed": 0, "skipped": 0, "xfailed": 0}
    for r in _MERMAID_RESULTS:
        if r["wasxfail"]: counts["xfailed"] += 1
        else: counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1

    # Write results.json EARLY — _gen_abstract_audit() lit ce fichier pendant
    # la construction du markdown ; sans ça abstract.py reçoit folder vide.
    (folder / "results.json").write_text(
        json.dumps({"counts": counts, "tests": _MERMAID_RESULTS},
                   indent=2, default=str))

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

    # Run log_timeline.py — produces CSV in folder + ASCII table (no PNG;
    # plotting is done by RStudio/Quarto R chunk reading the CSV directly)
    log_timeline_table, _csv_ok = _run_log_timeline(folder, bucket_s=10.0)

    # Markdown report — GitHub-flavored (```mermaid blocks render natively)
    # Note : les edges entre catégories / layers utilisent `-.->` (dashed
    # visible discret) au lieu de `~~~` invisible. Pourquoi : GitHub Mermaid
    # parse `~~~` mais ne respecte pas le hint de layout vertical → les
    # subgraphs s'étalent horizontalement. Le `-.->` est visible mais
    # préserve l'ordre TB sur tous les renderers (GitHub, Quarto, mermaid.live).
    ts_start = datetime.datetime.now().isoformat(timespec="seconds")
    fmt_kwargs = dict(
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
        log_timeline_table  = log_timeline_table,
        log_timeline_bucket = "10",
        test_catalog        = _gen_test_catalog(_MERMAID_RESULTS),
        irda_plan_status    = _gen_irda_plan_status(),
        full_test_annex     = _gen_full_annex(_MERMAID_RESULTS),
        detail_per_category = _gen_detail_per_category_md(tree),
        diag_snapshot       = _gen_diag_snapshot(),
        diag_bundle_annex   = _gen_diag_bundle_annex(),
        abstract_audit      = _gen_abstract_audit(folder),
        blockers_chapter    = _gen_blockers_chapter(_MERMAID_RESULTS),
        env_block           = _gen_env_block(),
    )
    md = _REPORT_SKELETON.format(**fmt_kwargs)

    # Distillation : `report.md` (machine-friendly, sans mermaid ni details).
    # On le construit à partir du md complet, puis on l'ajoute aussi en annexe
    # finale du md/qmd (avec h2→h3 demote pour ne pas casser la hiérarchie).
    import re as _re_rep
    report_md = _gen_machine_report(md)
    report_md_demoted = _re_rep.sub(
        r'^(#+) ', r'#\1 ', report_md, flags=_re_rep.MULTILINE)
    annex_section = (
        "\n\n## Annexe — Report condensé (machine-friendly)\n\n"
        '<details markdown="1"><summary>Déplier — report condensé</summary>\n\n'
        + report_md_demoted +
        "\n\n</details>\n"
    )
    md = md + annex_section

    md_path = folder / "test_results.md"
    md_path.write_text(md)
    # Standalone copies (top-level + per-run folder) pour collage rapide
    (folder / "report.md").write_text(report_md)
    (out_dir / "report.md").write_text(report_md)

    # Quarto report (.qmd) — uses ```{mermaid} blocks + YAML front-matter so
    # RStudio / `quarto render` produit du HTML interactif avec les diagrammes
    # rendus. Le .md GitHub reste en parallèle pour collage direct.
    #
    # IMPORTANT — Quarto rendering quirk : Quarto/Pandoc HTML-escape le
    # contenu des blocs ```{mermaid} avant de le passer à mermaid.js. Les
    # tags `<br/>` deviennent `&lt;br/&gt;`, les guillemets `&quot;`, les
    # emojis dans edge labels passent mal, etc. Mermaid ne parse plus.
    #
    # Fix : pour les blocs Quarto, on produit une variante "safe" :
    #  - strip les guillemets internes des labels `id["X<br/>Y"]` → `id[X Y]`
    #  - le `<br/>` devient simple espace (Mermaid affiche le label en ligne)
    #  - emojis dans edge labels (`-.->|🛑|`) remplacés par texte plain
    #
    # Le .md GitHub garde la version riche (<br/>, emojis, quotes) qui s'y
    # rend nativement.
    import re as _re

    def _mermaid_for_quarto(block: str) -> str:
        out = []
        for line in block.splitlines():
            # 1. Replace ALL <br/> with space (handles multi-line labels with
            #    2+ <br/> like `name<br/>marker<br/>3.15s`).
            line = line.replace("<br/>", " ").replace("<br>", " ")
            # 2. Strip outer `["..."]` quotes in node labels (Quarto escapes
            #    them as &quot; which mermaid does not parse).
            line = _re.sub(r'\["([^"\n]*?)"\]',
                           lambda m: f'[{m.group(1)}]', line)
            # 3. Emojis dans edge labels   `-.->|🛑|`  →  `-.->|BREAK|`
            line = _re.sub(r'\|🛑\|', '|BREAK|', line)
            # 4. Parentheses inside `[...]` labels confuse Mermaid parser
            #    (interprétées comme stadium shape `id(...)`). Ex:
            #    `T_007[test_inject_si(1) inject_frames 0.33s]` casse.
            #    On les neutralise (espace) uniquement à l'intérieur des [].
            line = _re.sub(
                r'\[([^\[\]\n]*)\]',
                lambda m: '[' + m.group(1).replace('(', ' ').replace(')', ' ') + ']',
                line)
            out.append(line)
        return "\n".join(out)

    qmd_pipeline = _mermaid_for_quarto(pipeline_only)
    qmd_detail   = _mermaid_for_quarto(detail_only)
    qmd_kwargs = dict(fmt_kwargs)
    qmd_kwargs["pipeline_mermaid"] = qmd_pipeline
    qmd_kwargs["detail_per_category"] = _gen_detail_per_category_qmd(tree)
    qmd_body = _REPORT_SKELETON.format(**qmd_kwargs)
    # Quarto's YAML provides the title — drop the redundant first H1.
    lines = qmd_body.splitlines(keepends=True)
    if lines and lines[0].startswith("# "):
        qmd_body = "".join(lines[1:]).lstrip()
    # Replace GitHub-style mermaid fences with Quarto-style executable blocks
    qmd_body = qmd_body.replace("```mermaid\n", "```{mermaid}\n")
    # Idem md : annexe report condensé (utilise le report_md déjà calculé)
    qmd_body = qmd_body + annex_section
    # Quarto/Pandoc ferme implicitement les <details> dès qu'il rencontre un
    # `## heading` à l'intérieur (le markdown="1" attribute n'aide pas). On
    # transforme donc les <details markdown="1">…</details> top-level en
    # Quarto callouts collapsibles, qui acceptent du markdown imbriqué sans
    # fermeture implicite.
    import re as _re_d
    _det_pat = _re_d.compile(
        r'<details markdown="1">\s*<summary>([^<]*)</summary>(.+?)</details>',
        _re_d.DOTALL)
    def _to_callout(m):
        title = m.group(1).strip()
        content = m.group(2).strip()
        # Report condensé = priorité haute, déplié par défaut. Autres annexes
        # (logs, audit, bundle) collapsé par défaut pour ne pas noyer.
        collapse = "false" if "report condensé" in title.lower() else "true"
        return (f"::: {{.callout-note collapse=\"{collapse}\"}}\n"
                f"## {title}\n\n"
                f"{content}\n"
                ":::")
    qmd_body = _det_pat.sub(_to_callout, qmd_body)
    qmd = _QMD_FRONTMATTER + qmd_body
    qmd_path = folder / "test_results.qmd"
    qmd_path.write_text(qmd)
    # Also a stable top-level copy
    (out_dir / "test_results.qmd").write_text(qmd)
    # Also keep Quarto-friendly .mmd files (with \n labels) for direct paste
    (folder / "pipeline.qmd.mmd").write_text(qmd_pipeline)
    (folder / "detail.qmd.mmd").write_text(qmd_detail)

    # (results.json est écrit plus haut, avant la construction du markdown,
    # parce que `_gen_abstract_audit()` en a besoin pour son audit.)

    # Stable top-level copies (so pytest -v can be pointed at them deterministically)
    (out_dir / "test_results.md").write_text(md)
    (out_dir / "test_results.mmd").write_text(full_diagram)

    # Zip everything
    zip_path = out_dir / f"test_results_{ts_id}.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for entry in folder.iterdir():
            zf.write(entry, arcname=entry.name)
    (out_dir / "test_results_latest.zip").write_bytes(zip_path.read_bytes())

    # Si on tourne dans le container Calypso, /tmp est tmpfs (volatile) mais
    # /root est bind-monté sur /home/nirvana/myconfigs/osmo_root/ côté host.
    # On écrit donc une copie stable du zip dans /root/ pour qu'elle soit
    # accessible directement côté host sans `docker cp`. Override possible
    # via CALYPSO_HOST_PERSIST_DIR (path container côté bind-mount).
    persist_dir = Path(os.environ.get("CALYPSO_HOST_PERSIST_DIR", "/root"))
    if persist_dir.exists() and persist_dir.is_dir():
        try:
            persist_zip = persist_dir / "test_results_latest.zip"
            persist_zip.write_bytes(zip_path.read_bytes())
            persist_report = persist_dir / "report.md"
            persist_report.write_text(report_md)
        except Exception:
            pass

    terminal = session.config.pluginmanager.get_plugin("terminalreporter")
    if terminal is not None:
        terminal.write_sep("=", "Mermaid report bundle", purple=True)
        terminal.write_line(f"  folder : {folder}/")
        terminal.write_line(f"  md     : {md_path}")
        terminal.write_line(f"  qmd    : {qmd_path}  (RStudio/quarto render)")
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
        "  classDef brk  fill:#ff5252,stroke:#990000,color:#fff,stroke-width:3px;",
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
            lines.append(f"  class {bid} brk;")
            broken = True
        prev = pid
    return "\n".join(lines)


def _build_detail_per_category(tree: dict) -> dict[str, str]:
    """Retourne un dict {category_name: mermaid_block} — un graphe par catégorie.

    Format : un seul nœud racine = la catégorie, sous lequel pendent
    directement les tests (pas de hiérarchie par couche dans le détail —
    la couche reste affichée comme label secondaire sur chaque test pour
    ne pas perdre l'info). Les tests passing sont fusionnés en un seul
    nœud `PASS N` pour réduire la densité.
    """
    out: dict[str, str] = {}
    counter_base = 0

    header = [
        "  classDef pass    fill:#bef5b1,stroke:#1f7a1f,color:#0a3d0a;",
        "  classDef partial fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef fail    fill:#fbb4b4,stroke:#a31a1a,color:#5a0000;",
        "  classDef skip    fill:#e0e0e0,stroke:#6a6a6a,color:#333;",
        "  classDef xfail   fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
    ]

    for cat in sorted(tree):
        cid = "C_" + _sanitize_node_id(cat)
        body: list[str] = ["flowchart TB"] + header
        class_assigns: list[str] = []
        counter = counter_base
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        ccls = {"pass":"pass","partial":"partial","fail":"fail"}.get(cstatus,"skip")
        body.append(f'  {cid}["{_label(cat)}<br/>{cp}/{cn}"]')
        class_assigns.append(f"  class {cid} {ccls};")
        passing = [r for r in cat_tests if r["outcome"] == "passed" and not r["wasxfail"]]
        non_pass = [r for r in cat_tests if not (r["outcome"] == "passed" and not r["wasxfail"])]
        if passing:
            counter += 1
            pid = f"T_{counter:03d}_pass_grp_" + _sanitize_node_id(cat)
            total_d = sum(r["duration_s"] for r in passing)
            body.append(f'  {cid} --> {pid}["PASS {len(passing)}<br/>{total_d:.1f}s"]')
            class_assigns.append(f"  class {pid} pass;")
        for r in non_pass:
            counter += 1
            tid = f"T_{counter:03d}_" + _sanitize_node_id(r["name"])
            short = _label(r["name"][:38])
            marker_short = _label(",".join(r["markers"][:1]))
            layer_short = _label(r.get("layer", ""))
            label_txt = f"{short}<br/>{layer_short} · {marker_short}<br/>{r['duration_s']:.2f}s"
            cls = ("xfail" if r["wasxfail"]
                   else "fail" if r["outcome"] == "failed"
                   else "skip")
            body.append(f'  {cid} --> {tid}["{label_txt}"]')
            class_assigns.append(f"  class {tid} {cls};")
        out[cat] = "\n".join(body + class_assigns)
        counter_base = counter
    return out


def _gen_detail_per_category_md(tree: dict) -> str:
    """Markdown : `### <Category> N/M` + bloc ` ```mermaid ` pour chaque catégorie."""
    graphs = _build_detail_per_category(tree)
    out = []
    for cat in sorted(graphs):
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        icon = {"pass":"✅","partial":"🟡","fail":"🛑"}.get(cstatus,"·")
        out.append(f"### {icon} `{cat}` — {cp}/{cn}\n")
        out.append("```mermaid")
        out.append(graphs[cat])
        out.append("```")
        out.append("")
    return "\n".join(out)


def _gen_detail_per_category_qmd(tree: dict) -> str:
    """Variante Quarto : ` ```{mermaid} ` + transformation pour HTML-safe."""
    graphs = _build_detail_per_category(tree)
    out = []
    import re as _re_q
    def _to_qmd(block: str) -> str:
        lines = []
        for line in block.splitlines():
            line = line.replace("<br/>", " ").replace("<br>", " ")
            # Mermaid 10.2 (Quarto built-in) parse mal certains chars dans
            # labels sans guillemets : `·` (U+00B7) → rejeté, `|` → consommé
            # comme délimiteur de edge label, `(` `)` → stadium shape. Le
            # séparateur safe est `-` simple (rien qui ne soit syntaxe mermaid).
            line = line.replace(" · ", " - ")
            line = _re_q.sub(r'\["([^"\n]*?)"\]', lambda m: f'[{m.group(1)}]', line)
            line = _re_q.sub(r'\|🛑\|', '|BREAK|', line)
            line = _re_q.sub(
                r'\[([^\[\]\n]*)\]',
                lambda m: '[' + m.group(1).replace('(', ' ').replace(')', ' ') + ']',
                line)
            lines.append(line)
        return "\n".join(lines)
    for cat in sorted(graphs):
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        icon = {"pass":"✅","partial":"🟡","fail":"🛑"}.get(cstatus,"·")
        out.append(f"### {icon} `{cat}` — {cp}/{cn}\n")
        out.append("```{mermaid}")
        out.append(_to_qmd(graphs[cat]))
        out.append("```")
        out.append("")
    return "\n".join(out)


def _build_detail_only_mermaid(tree: dict) -> str:
    """Detail mermaid : 2 niveaux verticaux + 1 horizontal.
      Niveau 1 (flowchart TB) : 3 catégories empilées verticalement
      Niveau 2 (subgraph cat TB + invisible edges ~~~) : layers empilés vertical
      Niveau 3 : tests étalés horizontalement sous chaque layer-node

    Mermaid v10 caveats :
      - sub-subgraphs imbriqués + class assignments → parser plante
      - class assignments dans subgraph → parser plante
    Donc : pas de sub-subgraph, edges invisibles pour ordonner, class hors subgraph.
    """
    head = [
        "flowchart TB",
        "  classDef pass    fill:#bef5b1,stroke:#1f7a1f,color:#0a3d0a;",
        "  classDef partial fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
        "  classDef fail    fill:#fbb4b4,stroke:#a31a1a,color:#5a0000;",
        "  classDef skip    fill:#e0e0e0,stroke:#6a6a6a,color:#333;",
        "  classDef xfail   fill:#fde79c,stroke:#b07a00,color:#3a2c00;",
    ]
    body: list[str] = []
    class_assigns: list[str] = []
    counter = 0
    prev_cat_sgid = None  # to chain subgraphs vertically via invisible edges
    for cat in sorted(tree):
        cid = "C_" + _sanitize_node_id(cat)
        sgid = f"SG_{cid}"
        cat_tests = [t for layer in tree[cat].values() for t in layer]
        cstatus, cp, cn = _layer_status(cat_tests)
        if body:
            body.append("")
        body.append(f'  subgraph {sgid} ["{_label(cat)} {cp}/{cn}"]')
        body.append("    direction TB")
        prev_lid = None
        for layer in sorted(tree[cat]):
            lid = f"{cid}_L_" + _sanitize_node_id(layer)
            lt = tree[cat][layer]
            lstatus, lp, ln = _layer_status(lt)
            lcls = {"pass":"pass","partial":"partial","fail":"fail"}.get(lstatus,"skip")
            body.append(f'    {lid}["{_label(layer)}<br/>{lp}/{ln}"]')
            class_assigns.append(f"  class {lid} {lcls};")
            # Invisible edge between consecutive layers forces vertical stacking.
            if prev_lid is not None:
                body.append(f"    {prev_lid} -.-> {lid}")
            prev_lid = lid
            passing = [r for r in lt if r["outcome"] == "passed" and not r["wasxfail"]]
            non_pass = [r for r in lt if not (r["outcome"] == "passed" and not r["wasxfail"])]
            if passing:
                counter += 1
                pid = f"T_{counter:03d}_pass_grp_" + _sanitize_node_id(layer)
                total_d = sum(r["duration_s"] for r in passing)
                body.append(f'    {lid} --> {pid}["PASS {len(passing)}<br/>{total_d:.1f}s"]')
                class_assigns.append(f"  class {pid} pass;")
            for r in non_pass:
                counter += 1
                tid = f"T_{counter:03d}_" + _sanitize_node_id(r["name"])
                short = _label(r["name"][:38])
                marker_short = _label(",".join(r["markers"][:1]))
                label_txt = f"{short}<br/>{marker_short}<br/>{r['duration_s']:.2f}s"
                cls = ("xfail" if r["wasxfail"]
                       else "fail" if r["outcome"] == "failed"
                       else "skip")
                body.append(f'    {lid} --> {tid}["{label_txt}"]')
                class_assigns.append(f"  class {tid} {cls};")
        body.append("  end")
        # Invisible edge between consecutive category-subgraphs to force them
        # stacked vertically (Mermaid doesn't enforce TB layout for siblings
        # without edges, even with flowchart TB at the top).
        if prev_cat_sgid is not None:
            body.append(f"  {prev_cat_sgid} -.-> {sgid}")
        prev_cat_sgid = sgid
    return "\n".join(head + body + class_assigns)
