"""
grafcet_gen.py — produit LE grafcet go-live/IQ (HTML + MD) à partir de l'état
réel du run. Appelé par tests/conftest.py::pytest_sessionfinish, donc chaque
`pytest` régénère le grafcet avec le statut courant de chaque RANK dérivé des
logs (qemu.log / osmocon.log / mobile.log) — plus de grafcet lu à l'œil.

Statuts : ok (câblé, validé au run) · wip (posé, en cours) · bad (manquant/bloqué).
"""
from __future__ import annotations
import os
import re
from pathlib import Path

QEMU_LOG = os.environ.get("CALYPSO_QEMU_LOG", "/root/qemu.log")
OSMOCON_LOG = "/root/osmocon.log"
MOBILE_LOG = "/root/mobile.log"


def _read(p: str) -> str:
    try:
        return open(p, "rb").read().decode("latin-1", "replace")
    except OSError:
        return ""


def _deaccent(s):
    """Translit ASCII (evite mojibake Ã©/Ã¹ selon renderer). Pour qmd/Rmd."""
    import unicodedata as _ud
    m = {"\u2014":"-","\u2013":"-","\u00b7":"-","\u2026":"...","\u2192":"->",
         "\u2190":"<-","\u00ab":'"',"\u00bb":'"',"\u2019":"'","\u2018":"'",
         "\u201c":'"',"\u201d":'"',"\u00d7":"x","\U0001f534":"[X]","\U0001f7e2":"[OK]",
         "\U0001f7e0":"[~]","\u2705":"[OK]","\u274c":"[X]","\u26a0":"[!]","\ufe0f":""}
    for k,v in m.items(): s=s.replace(k,v)
    return _ud.normalize("NFKD", s or "").encode("ascii","ignore").decode("ascii")


def _tr(s):
    """Translit emoji/fleches -> marqueurs ASCII, garde les accents (latin-1 fpdf)."""
    repl = {"\U0001F534": "[X]", "\U0001F7E2": "[OK]", "\U0001F7E0": "[~]",
            "\U0001F7E1": "[~]", "\u2705": "[v]", "\u274c": "[x]", "\u26a0": "[!]",
            "\ufe0f": "", "\u2192": "->", "\u2190": "<-", "\u2014": "-", "\u2013": "-",
            "\u2019": "'", "\u2018": "'", "\u201c": '"', "\u201d": '"', "\u2026": "...",
            "\u2022": "-", "\u2265": ">=", "\u2264": "<=", "\u00d7": "x"}
    for k, v in repl.items():
        s = s.replace(k, v)
    return s.encode("latin-1", "replace").decode("latin-1")


def derive_status() -> dict:
    """Dérive le statut de chaque maillon depuis les logs du dernier run."""
    L = _read(QEMU_LOG)
    O = _read(OSMOCON_LOG)
    M = _read(MOBILE_LOG)

    def has(pat, s=L):
        return re.search(pat, s) is not None

    def count(pat, s=L):
        return len(re.findall(pat, s))

    # RANK1 — gate go-live 0x0810 (CTRLSYS) : a53f TC=1, pas de court-circuit
    rank1 = "ok" if (has(r"a53f\(BC a575.*TC=1") or has(r"arm2dsp\] CTRLSYS")) else "bad"

    # RANK4 — recalage FN : dernier delta FN-ALIGN dans la fenêtre
    deltas = [int(x) for x in re.findall(r"FN-ALIGN.*delta=(-?\d+)", L)]
    if deltas:
        rank4 = "ok" if abs(deltas[-1]) < 64 else "bad"
    else:
        rank4 = "bad"

    # LOST timer : compte des LOST (osmocon) — le fix read-driven doit les tuer
    lost_n = count(r"LOST \d+", O)
    lost = "ok" if lost_n < 50 else "bad"

    # RANK3 — dispatch handler->kernel : AR5 dans le buffer ?
    if has(r"AR5=2a[0-9a-f]{2}"):
        rank3 = "ok"
    elif "correlator_ar5_in_iq_buffer" in L:
        rank3 = "bad"
    else:
        rank3 = "wip"

    # RANK2 — entrée I/Q DL du BSP : le socket reçoit-il (RXSZ) ?
    rank2 = "ok" if count(r"RXSZ") > 0 else "bad"

    # RANK5 — PM légitime : PM MEAS non nul ?
    rank5 = "ok" if has(r"PM MEAS.*?[1-9]\d*\s+dBm at baseband", O) else "bad"

    # détection + camp (résultat)
    detect = "ok" if count(r"fb0_ret=[1-9]", L) else "bad"
    camp = "ok" if has(r"normal service", M) else "bad"

    # signaux annexes
    d3f92 = "ok" if has(r"d\[3f92\]=0x0800") else "bad"
    fben = "wip" if has(r"arm2dsp\] FBEN") else "bad"

    return dict(rank1=rank1, rank2=rank2, rank3=rank3, rank4=rank4, rank5=rank5,
                lost=lost, detect=detect, camp=camp, d3f92=d3f92, fben=fben,
                last_delta=(deltas[-1] if deltas else None), lost_n=lost_n)


# ---- rendu -----------------------------------------------------------------

_MERMAID_CLASS = {"ok": "ok", "wip": "wip", "bad": "bad"}


def _mermaid(st: dict) -> str:
    c = lambda k: _MERMAID_CLASS[st[k]]
    # branche BSP suit RANK2 ; convergence/détection suit RANK3
    b = c("rank2")
    k3 = c("rank3")
    return f"""flowchart TD
  classDef src  fill:#12242f,stroke:#6ea8d8,stroke-width:1px,color:#cfe3f2;
  classDef ok   fill:#0f2620,stroke:#46c39a,stroke-width:1px,color:#bfe9d7;
  classDef wip  fill:#2a2211,stroke:#e6a94e,stroke-width:1px,color:#f0d6a4;
  classDef bad  fill:#2a1412,stroke:#e86a5f,stroke-width:1.4px,color:#f2c3bd;
  classDef join fill:#101d22,stroke:#38d3c2,stroke-width:1.4px,color:#bff0ea;

  E0["BTS emet en continu<br/>SI3 - FCCH - SCH - FN reelle"]:::src
  E0 -->|I/Q air| P1["ipc-device sert l'I/Q<br/>qfn-serve - FCCH"]:::ok
  P1 -->|FIFO iq_grgsm| G1["gr-gsm decode la SCH<br/>BSIC=7 - sch_fn"]:::ok
  G1 -->|"DL_FN_OFFSET=-556"| F1["FN-ALIGN recale FN<br/>delta {st.get('last_delta','?')}"]:::{c('rank4')}
  P1 -. "UDP :6702 -- {'CABLE' if st['rank2']=='ok' else 'NON CABLE'}" .-> B1["BSP bsp_trxd_readable<br/>RXSZ {'>0' if st['rank2']=='ok' else '= 0'}"]:::{b}
  B1 --> B2["feed_iq -> last_pm<br/>PM {'ok' if st['rank5']=='ok' else '= 0 dBm'}"]:::{c('rank5')}
  B2 --> B3["rx_burst -> DARAM 0x2a00<br/>entree correlateur"]:::{b}

  A0["init go-live 0xa4c7<br/>ORM/RSBX/IMR"]:::ok
  A0 -->|"CTRLSYS pose 0x0810 bit15"| A1["gate 0xa53c BITF<br/>TC=1 - bootstrap"]:::{c('rank1')}
  A1 -->|"KEEP_IMR - FRAME_IT_NATIVE"| A2["IMR=0x52fd - BRINT0 arme<br/>INTM natif - LOST {'=0' if st['lost']=='ok' else 'spam'}"]:::{c('lost')}
  A2 -->|"TASKW d[3f92]=0x0800"| A3["task_md=5 dispatche<br/>CALA 0xb01e -> 0x8d00"]:::{c('d3f92')}
  A3 -->|"FBEN d[3fab] bit8"| A4["gate 0x3fab bit8<br/>vers kernel"]:::{c('fben')}

  B3 --> K["CORRELATEUR FB"]:::join
  A4 --> K
  K -. "AR5 {'=0x2a00' if st['rank3']=='ok' else '=0xdb7b (garbage)'}<br/>kernel 0xa076 {'atteint' if st['rank3']=='ok' else 'jamais atteint'}" .-> KB["fb0_ret {'>0' if st['detect']=='ok' else '= 0'}"]:::{k3}
  KB --> D["d_fb_det"]:::{c('detect')}
  D --> Mn["mobile campe<br/>normal service"]:::{c('camp')}

  linkStyle default stroke:#3a565d,stroke-width:1.3px;"""


_RANK_ROWS = [
    ("RANK1", "Pont ARM&#8594;DSP d_ctrl_system &#8212; gate go-live 0xa53c (bit15 de 0x0810)", "CALYPSO_ARM2DSP_CTRLSYS=1", "rank1"),
    ("RANK4", "Recalage FN DSP/TRX sur la SCH &#8212; offset constant &#8722;556", "CALYPSO_DL_FN_OFFSET=-556", "rank4"),
    ("&#8212;", "Cadence timer &#8212; LOST N! (batching frame-IRQ)", "latch read-driven (&#8722;1875/lecture)", "lost"),
    ("RANK3", "Dispatch handler 0x8d00 &#8594; kernel MAC 0xa076 (AR5=0x2a00)", "TASKW / FBEN (d[3f92], d[3fab] bit8)", "rank3"),
    ("RANK2", "Entree I/Q DL du BSP &#8212; forward producteur &#8594; UDP :6702", "&#8212; (aucun forward)", "rank2"),
    ("RANK5", "PM / rxlev legitime (last_pm depuis le vrai I/Q)", "depend de RANK2 (feed_iq)", "rank5"),
]

_PILL = {"ok": ("p-ok", "CABLE"), "wip": ("p-wip", "EN COURS"), "bad": ("p-bad", "MANQUANT")}


def _css() -> str:
    return """
  :root{--bg:#0c1418;--panel:#0f191e;--line:#20343b;--ink:#d7e3e5;--muted:#83999d;
    --faint:#5b7176;--cyan:#38d3c2;--ok:#46c39a;--wip:#e6a94e;--bad:#e86a5f;--src:#6ea8d8;
    --mono:ui-monospace,"SF Mono","JetBrains Mono",Menlo,Consolas,monospace;
    --sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--sans);line-height:1.55}
  .wrap{max-width:1080px;margin:0 auto;padding:48px 28px 80px}
  header{border-bottom:1px solid var(--line);padding-bottom:26px;margin-bottom:20px}
  .eyebrow{font-family:var(--mono);font-size:12px;letter-spacing:.22em;text-transform:uppercase;color:var(--cyan);margin:0 0 14px}
  h1{font-family:var(--mono);font-weight:600;font-size:clamp(24px,4vw,38px);margin:0 0 10px;color:#eef6f6}
  .sub{color:var(--muted);max-width:64ch;margin:0;font-size:15px}
  .meta{font-family:var(--mono);font-size:12px;color:var(--faint);margin-top:16px;display:flex;gap:20px;flex-wrap:wrap}
  .legend{display:flex;gap:8px 22px;flex-wrap:wrap;margin:24px 0 8px;font-family:var(--mono);font-size:12.5px}
  .legend span{display:inline-flex;align-items:center;gap:8px;color:var(--muted)}
  .dot{width:10px;height:10px;border-radius:2px}
  .d-ok{background:var(--ok)}.d-wip{background:var(--wip)}.d-bad{background:var(--bad)}.d-src{background:var(--src)}
  .diagram{background:#0f191e;border:1px solid var(--line);border-radius:12px;padding:14px;margin:10px 0 40px;overflow-x:auto}
  .mermaid{min-width:640px}
  h2{font-family:var(--mono);font-size:13px;letter-spacing:.16em;text-transform:uppercase;color:var(--cyan);margin:36px 0 16px}
  table{width:100%;border-collapse:collapse;font-size:14px}
  th,td{text-align:left;padding:11px 14px;border-bottom:1px solid var(--line);vertical-align:top}
  th{font-family:var(--mono);font-size:11.5px;letter-spacing:.1em;text-transform:uppercase;color:var(--faint)}
  td.rank{font-family:var(--mono);color:var(--cyan);white-space:nowrap;font-weight:600}
  td.env{font-family:var(--mono);font-size:12.5px;color:var(--muted)}
  .pill{font-family:var(--mono);font-size:11px;padding:2px 9px;border-radius:20px;display:inline-block;font-weight:600}
  .p-ok{background:rgba(70,195,154,.14);color:var(--ok);border:1px solid rgba(70,195,154,.35)}
  .p-wip{background:rgba(230,169,78,.13);color:var(--wip);border:1px solid rgba(230,169,78,.35)}
  .p-bad{background:rgba(232,106,95,.13);color:var(--bad);border:1px solid rgba(232,106,95,.38)}
  .foot{margin-top:40px;padding-top:22px;border-top:1px solid var(--line);font-family:var(--mono);font-size:12px;color:var(--faint)}
  code{font-family:var(--mono)}
"""


def render_html(st: dict, ts: str = "") -> str:
    rows = "\n".join(
        f'      <tr><td class="rank">{r}</td><td>{d}</td><td class="env">{e}</td>'
        f'<td><span class="pill {_PILL[st[k]][0]}">{_PILL[st[k]][1]}</span></td></tr>'
        for (r, d, e, k) in _RANK_ROWS
    )
    return f"""<title>GRAFCET &#8212; Calypso go-live &amp; chaine I/Q</title>
<style>{_css()}</style>
<div class="wrap">
  <header>
    <p class="eyebrow">TI Calypso &#183; emulation QEMU &#183; baseband GSM &#183; auto-genere par pytest</p>
    <h1>GRAFCET &#8212; go-live DSP &amp; chaine I/Q</h1>
    <p class="sub">Etat-transition de l'acquisition FB/FCCH, statut de chaque maillon derive des logs du dernier run.</p>
    <div class="meta"><span>{ts}</span><span>delta FN {st.get('last_delta','?')}</span><span>LOST {st.get('lost_n','?')}</span></div>
  </header>
  <div class="legend">
    <span><i class="dot d-src"></i>source</span><span><i class="dot d-ok"></i>cable</span>
    <span><i class="dot d-wip"></i>en cours</span><span><i class="dot d-bad"></i>manquant/bloque</span>
  </div>
  <div class="diagram"><pre class="mermaid">
{_mermaid(st)}
  </pre></div>
  <h2>Rang de blocage &#183; gates</h2>
  <table>
    <thead><tr><th>Rang</th><th>Maillon</th><th>Gate / wire</th><th>Etat</th></tr></thead>
    <tbody>
{rows}
    </tbody>
  </table>
  <p class="foot">Auto-genere par tests/conftest.py::pytest_sessionfinish &#8594; grafcet_gen.py &#183; statut derive de qemu/osmocon/mobile.log</p>
</div>
"""


def render_md(st: dict, ts: str = "") -> str:
    badge = {"ok": "`CABLE`", "wip": "`EN COURS`", "bad": "`MANQUANT`"}

    def deent(s):
        return (s.replace("&#8594;", "->").replace("&#8212;", "-")
                 .replace("&#8722;", "-").replace("&#183;", "-"))

    rows = "\n".join(
        f"| {deent(r)} | {deent(d)} | `{deent(e)}` | {badge[st[k]]} |"
        for (r, d, e, k) in _RANK_ROWS
    )
    return f"""# GRAFCET — go-live DSP & chaîne I/Q

*Auto-généré par `tests/conftest.py::pytest_sessionfinish` — statut dérivé des logs du dernier run ({ts}).*

- delta FN-ALIGN : **{st.get('last_delta','?')}**  ·  LOST : **{st.get('lost_n','?')}**

```mermaid
{_mermaid(st)}
```

## Rang de blocage · gates

| Rang | Maillon | Gate / wire | État |
|------|---------|-------------|------|
{rows}

> Convergence au corrélateur : dispatch (go-live) ∧ entrée I/Q (signal). Le verrou courant
> est l'entrée I/Q DL du BSP (`UDP :6702` non câblé) et le dispatch handler→kernel (`AR5`).
"""


def _ascii(s: str) -> str:
    rep = {"→": "->", "—": "-", "−": "-", "·": "-", "∧": "et",
           "≤": "<=", "≥": ">=", "‘": "'", "’": "'"}
    for k, v in rep.items():
        s = s.replace(k, v)
    return s.encode("latin-1", "replace").decode("latin-1")



# ---- Grafcet interprété (dessin natif fpdf) + UML pipeline (graphviz dot) --------
# Chemin ORDONNE + poids (%) : FBSB (d_fb_det) et camping portent le max de %,
# les etages infra/amont peu (le % reflete la progression reelle vers le but, pas
# le nombre de tests). Somme des poids = 100.
_STAGES = [
    ("E0", "BTS emet SI3/FCCH/SCH", "ok", 2),
    ("P1", "ipc-device -> FIFO iq_grgsm", "ok", 2),
    ("G1", "gr-gsm decode SCH (BSIC=7)", "ok", 3),
    ("F1", "FN-ALIGN recale FN", "rank4", 3),
    ("B1", "BSP :6702 feed_iq", "rank2", 4),
    ("B3", "rx_burst -> DARAM 0x2a00", "rank2", 4),
    ("A0", "init go-live (ORM/RSBX/IMR)", "ok", 4),
    ("A1", "gate 0x0810 bit15 (abort=0)", "rank1", 4),
    ("A2", "IMR=0x52fd, LOST=0", "lost", 4),
    ("A3", "task_md=5 -> 0x8d00", "d3f92", 5),
    ("F2", "frame-IT vec28 (PRIO)", "fben", 6),
    ("K",  "CORRELATEUR kernel 0xa076", "rank3", 12),
    ("D",  "FBSB / d_fb_det (fb0_ret>0)", "detect", 25),
    ("M",  "mobile campe (normal service)", "camp", 22),
]

def _stage_stt(st, key):
    return (st.get(key, "bad") if key in st else key) if key in ("ok", "wip", "bad") or key in st else "bad"

def _weighted_pct(st):
    """Progression ordonnee ponderee : ok=plein, wip=demi, bad=0.
    Retourne (pct_total, pct_cumul_chemin, [(code,label,w,stt,got)])."""
    rows = []
    got_sum = 0.0
    tot = sum(w for *_, w in _STAGES) or 1
    path_broken = False
    path_sum = 0.0
    for code, label, key, w in _STAGES:
        stt = st.get(key, "bad") if key in st else key
        if stt not in ("ok", "wip", "bad"):
            stt = "bad"
        got = 1.0 if stt == "ok" else (0.5 if stt == "wip" else 0.0)
        got_sum += got * w
        # chemin dans l'ordre : on cumule tant que la chaine tient (premier bad = mur)
        if not path_broken:
            path_sum += got * w
            if stt != "ok":
                path_broken = True
        rows.append((code, label, w, stt, got))
    return (round(100.0 * got_sum / tot), round(100.0 * path_sum / tot), rows)


def _draw_grafcet(pdf, st):
    """Grafcet GEMMA-like : etapes (boites carrees) + transitions verticales,
    couleur = statut. Rendu vectoriel natif (pas d'image)."""
    C = {"ok": (70, 195, 154), "wip": (230, 169, 78), "bad": (232, 106, 95)}
    pdf.add_page()
    pdf.set_font("Helvetica", "B", 12); pdf.set_text_color(20, 40, 46)
    pdf.cell(0, 8, "GRAFCET interprete (etat go-live -> correlateur -> camp)", ln=1)
    pdf.set_text_color(20, 20, 20)
    pct_tot, pct_path, _rows = _weighted_pct(st)
    # barre de progression ponderee (chemin ordonne : FBSB & camping = max %)
    pdf.set_font("Helvetica", "B", 9)
    pdf.cell(0, 6, _ascii(f"Progression chemin (ponderee) : {pct_path}%   "
                          f"[FBSB 25% + camping 22% = 47% du but]  -  total etapes atteintes {pct_tot}%"), ln=1)
    bx, by, bwd, bhh = 14.0, pdf.get_y() + 1, 182.0, 5.0
    pdf.set_draw_color(120, 130, 134); pdf.set_fill_color(232, 236, 236)
    pdf.rect(bx, by, bwd, bhh, "DF")
    pdf.set_fill_color(70, 195, 154)
    pdf.rect(bx, by, bwd * pct_path / 100.0, bhh, "F")
    pdf.set_y(by + bhh + 2)
    x0 = 26.0; y = pdf.get_y() + 4
    bw, bh = 150.0, 8.5; gap = 4.0
    for i, (code, label, key, w) in enumerate(_STAGES):
        stt = st.get(key, "bad") if key in st else key
        if stt not in C: stt = "bad"
        r, g, b = C[stt]
        # transition (trait vertical + barre) entre etapes
        if i > 0:
            pdf.set_draw_color(90, 100, 105); pdf.set_line_width(0.4)
            cx = x0 + 8
            pdf.line(cx, y - gap, cx, y)
            pdf.line(cx - 3, y - gap/2, cx + 3, y - gap/2)  # barre de transition
        # etape : liseré gauche coloré + boite
        pdf.set_fill_color(r, g, b)
        pdf.rect(x0, y, 6, bh, "F")                       # marqueur etat
        pdf.set_draw_color(60, 70, 74); pdf.set_line_width(0.3)
        pdf.set_fill_color(246, 248, 248)
        pdf.rect(x0 + 8, y, bw, bh, "DF")                 # corps etape
        pdf.set_xy(x0 + 11, y + 1.6)
        pdf.set_font("Helvetica", "B", 8); pdf.set_text_color(30, 40, 44)
        pdf.cell(12, 5, _ascii(code), 0, 0)
        pdf.set_font("Helvetica", "", 8); pdf.set_text_color(35, 35, 35)
        pdf.cell(0, 5, _ascii(label), 0, 0)
        pdf.set_font("Helvetica", "", 7); pdf.set_text_color(110, 110, 110)
        pdf.set_xy(x0 + 8 + bw - 62, y + 1.6)
        pdf.cell(20, 5, _ascii(f"{w}%"), 0, 0, "R")
        pdf.set_text_color(r, g, b); pdf.set_font("Helvetica", "B", 7)
        pdf.set_xy(x0 + 8 + bw - 30, y + 1.6)
        pdf.cell(28, 5, {"ok": "CABLE", "wip": "EN COURS", "bad": "MANQUANT"}[stt], 0, 0, "R")
        pdf.set_text_color(20, 20, 20)
        y += bh + gap
        if y > 250:
            pdf.add_page(); y = pdf.get_y() + 4

def _uml_png(out_dir, st):
    """Genere un diagramme pipeline (graphviz dot -> PNG) colore par statut.
    Retourne le chemin PNG ou None si dot indisponible."""
    import subprocess, shutil
    if not shutil.which("dot"):
        return None
    C = {"ok": "#46c39a", "wip": "#e6a94e", "bad": "#e86a5f"}
    def col(key):
        stt = st.get(key, "bad") if key in st else key
        return C.get(stt, "#e86a5f")
    nodes = [
        ("bts", "BTS\nSI3/FCCH/SCH", "ok"),
        ("ipc", "ipc-device\niq_grgsm", "ok"),
        ("grgsm", "gr-gsm\ndecode SCH", "ok"),
        ("bsp", "BSP :6702\nDARAM 0x2a00", "rank2"),
        ("golive", "DSP go-live\nIMR 0x52fd", "rank1"),
        ("frameit", "frame-IT\nvec28/PRIO", "fben"),
        ("corr", "CORRELATEUR\nkernel 0xa076", "rank3"),
        ("det", "d_fb_det", "detect"),
        ("camp", "mobile\ncampe", "camp"),
    ]
    edges = ["bts->ipc", "ipc->grgsm", "grgsm->bsp", "bsp->corr",
             "grgsm->golive", "golive->frameit", "frameit->corr",
             "corr->det", "det->camp"]
    dot = ['digraph pipeline {', 'rankdir=LR;', 'bgcolor="white";',
           'node [shape=box,style="filled,rounded",fontname="Helvetica",fontsize=10,color="#333333"];',
           'edge [color="#5a7176",penwidth=1.2];']
    for nid, lbl, key in nodes:
        dot.append(f'{nid} [label="{lbl}",fillcolor="{col(key)}"];')
    dot += edges
    dot.append('}')
    dpath = Path(out_dir) / "pipeline_uml.dot"
    ppath = Path(out_dir) / "pipeline_uml.png"
    dpath.write_text("\n".join(dot), encoding="utf-8")
    try:
        subprocess.run(["dot", "-Tpng", "-Gdpi=140", str(dpath), "-o", str(ppath)],
                       check=True, timeout=30, capture_output=True)
        return ppath if ppath.exists() else None
    except Exception:
        return None


def write_pdf(out_dir, ts: str = "") -> Path:
    """Rapport final PDF : grafcet (état) + pipeline + RANK + milestones + logs.
    Pur-Python (fpdf2), aucune dépendance système."""
    from fpdf import FPDF
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    st = derive_status()
    C = {"ok": (70, 195, 154), "wip": (230, 169, 78), "bad": (232, 106, 95)}
    MARK = {"ok": "[CABLE]", "wip": "[EN COURS]", "bad": "[MANQUANT]"}

    pdf = FPDF()
    pdf.set_auto_page_break(True, margin=16)
    pdf.add_page()
    pdf.set_fill_color(12, 20, 24)
    pdf.rect(0, 0, 210, 34, "F")
    pdf.set_text_color(56, 211, 194)
    pdf.set_font("Helvetica", "B", 9)
    pdf.set_xy(14, 9)
    pdf.cell(0, 5, "TI CALYPSO - EMULATION QEMU - BASEBAND GSM", ln=1)
    pdf.set_text_color(238, 246, 246)
    pdf.set_font("Helvetica", "B", 20)
    pdf.set_x(14)
    pdf.cell(0, 10, "GRAFCET - go-live DSP & chaine I/Q", ln=1)
    pdf.set_text_color(90, 113, 118)
    pdf.set_font("Helvetica", "", 9)
    pdf.set_x(14)
    pdf.cell(0, 6, _ascii(f"Rapport final auto-genere par pytest  -  {ts}  -  "
                          f"delta FN {st.get('last_delta')}  -  LOST {st.get('lost_n')}"), ln=1)
    pdf.ln(8)
    pdf.set_text_color(20, 20, 20)

    def h2(t):
        pdf.ln(3); pdf.set_font("Helvetica", "B", 12); pdf.set_text_color(20, 40, 46)
        pdf.cell(0, 8, _ascii(t), ln=1); pdf.set_text_color(20, 20, 20)

    # --- GRAFCET interprete (dessin natif) ---
    try:
        _draw_grafcet(pdf, st)
    except Exception:
        pass
    # --- UML pipeline (graphviz dot -> PNG embarque) ---
    try:
        _up = _uml_png(out_dir, st)
        if _up:
            pdf.add_page()
            h2("Diagramme pipeline (UML)")
            pdf.image(str(_up), x=14, w=182)
    except Exception:
        pass
    # --- diagrammes MERMAID (grafcet/sequence/timeline) + TEMPORELS (matplotlib) ---
    try:
        for _title, _png in _all_diagram_pngs(out_dir, st):
            pdf.add_page(); h2(_title)
            try: pdf.image(str(_png), x=14, w=182)
            except Exception: pass
    except Exception:
        pass
    # --- pipeline (grafcet en étapes texte) ---
    h2("Pipeline (etapes)")
    pdf.set_font("Courier", "", 9)
    d = st.get("last_delta", "?")
    steps = [
        ("ok",  "E0  BTS emet en continu : SI3 - FCCH - SCH - FN reelle"),
        ("ok",  "P1  ipc-device sert l'I/Q -> FIFO iq_grgsm"),
        ("ok",  "G1  gr-gsm decode la SCH  (BSIC=7, sch_fn)"),
        (st["rank4"], f"F1  FN-ALIGN recale la FN  (delta {d}, DL_FN_OFFSET=-556)"),
        (st["rank2"], "B1  BSP :6702  (feed_iq)   <-- UDP forward " + ("CABLE" if st["rank2"]=="ok" else "NON CABLE")),
        (st["rank5"], "B2  feed_iq -> last_pm (PM " + ("ok" if st["rank5"]=="ok" else "= 0 dBm") + ")"),
        (st["rank2"], "B3  rx_burst -> DARAM 0x2a00 (entree correlateur)"),
        ("ok",  "A0  init go-live 0xa4c7 (ORM/RSBX/IMR)"),
        (st["rank1"], "A1  gate 0xa53c BITF 0x0810 bit15 -> TC=1 (CTRLSYS)"),
        (st["lost"], "A2  IMR=0x52fd, BRINT0 arme, INTM natif, LOST " + ("0" if st["lost"]=="ok" else "spam")),
        (st["d3f92"], "A3  task_md=5 dispatche : CALA 0xb01e -> 0x8d00 (TASKW d[3f92])"),
        (st["fben"], "A4  gate 0x3fab bit8 -> kernel (FBEN)"),
        (st["rank3"], "K   CORRELATEUR : AR5 " + ("=0x2a00" if st["rank3"]=="ok" else "=0xdb7b (garbage)") + " -> kernel 0xa076 " + ("atteint" if st["rank3"]=="ok" else "jamais atteint")),
        (st["detect"], "D   d_fb_det  (fb0_ret " + (">0" if st["detect"]=="ok" else "= 0") + ")"),
        (st["camp"], "M   mobile campe (normal service)"),
    ]
    for stt, line in steps:
        r, g, b = C[stt]
        pdf.set_text_color(r, g, b)
        pdf.cell(4, 5, "*", 0, 0)
        pdf.set_text_color(30, 30, 30)
        pdf.cell(0, 5, _ascii(line), ln=1)

    # --- table RANK ---
    h2("Rang de blocage - gates")
    pdf.set_font("Helvetica", "B", 8)
    pdf.set_fill_color(240, 242, 242)
    pdf.cell(18, 7, "Rang", 1, 0, "L", True)
    pdf.cell(92, 7, "Maillon", 1, 0, "L", True)
    pdf.cell(50, 7, "Gate / wire", 1, 0, "L", True)
    pdf.cell(0, 7, "Etat", 1, 1, "L", True)
    pdf.set_font("Helvetica", "", 7.5)
    for (r, dsc, e, k) in _RANK_ROWS:
        rr, gg, bb = C[st[k]]
        y = pdf.get_y()
        pdf.multi_cell(18, 6, _ascii(re.sub("&#[0-9]+;", "-", r)), 1, "L", False)
        pdf.set_xy(28, y)
        pdf.multi_cell(92, 6, _ascii(re.sub("&#[0-9]+;", "-", dsc))[:78], 1, "L", False)
        pdf.set_xy(120, y)
        pdf.multi_cell(50, 6, _ascii(re.sub("&#[0-9]+;", "-", e))[:34], 1, "L", False)
        pdf.set_xy(170, y)
        pdf.set_text_color(rr, gg, bb); pdf.set_font("Helvetica", "B", 7.5)
        pdf.cell(0, 6, MARK[st[k]], 1, 1, "L")
        pdf.set_text_color(20, 20, 20); pdf.set_font("Helvetica", "", 7.5)

    # --- milestones (depuis report.md du run si présent) ---
    rep = Path(out_dir) / "report.md"
    if not rep.exists():
        rep = Path(out_dir).parent / "report.md"
    if rep.exists():
        h2("Milestones (resultats pytest)")
        pdf.set_font("Courier", "", 7.5)
        txt = _ascii(rep.read_text(encoding="utf-8", errors="replace"))
        for line in txt.splitlines():
            if any(t in line for t in ("PASS", "FAIL", "XFAIL", "XPASS", "SKIP", "milestone", "PHASE")):
                pdf.set_x(pdf.l_margin)
                pdf.multi_cell(0, 4.4, line[:110] or " ")

    # --- rapport de tests COMPLET : test_results.md en BLOC CODE COLORE (dark) ---
    trp = Path(out_dir) / "test_results.md"
    if not trp.exists():
        trp = Path(out_dir).parent / "test_results.md"
    try:
        trtxt = trp.read_text(encoding="utf-8", errors="replace")
    except Exception:
        trtxt = ""
    if trtxt:
        pdf.add_page()
        h2("Rapport de tests complet (test_results.md)")
        pdf.set_font("Courier", "", 6.0)
        # syntaxe light-theme (style GitHub clair) sur FOND BLANC
        BG = (255, 255, 255)
        COL = {"hdr": (0, 92, 197), "ok": (34, 134, 58), "bad": (203, 36, 49),
               "warn": (176, 122, 0), "key": (111, 66, 193), "code": (3, 102, 102),
               "txt": (36, 41, 46)}
        def _cls(s):
            u = s.upper(); st = s.lstrip()
            if st.startswith("#"): return "hdr"
            if any(t in u for t in ("PASS", "[OK]", "[V]")): return "ok"
            if any(t in u for t in ("FAIL", " MUR", "[X]", "ERROR")): return "bad"
            if any(t in u for t in ("XFAIL", "XPASS", "SKIP", "[~]")): return "warn"
            if st.startswith("- **") or st.startswith("**") or st.startswith("|"): return "key"
            if st.startswith("`") or st.startswith(">"): return "code"
            return "txt"
        for raw in trtxt.splitlines():
            l = _tr(raw)
            r, g, b = COL[_cls(l)]
            pdf.set_text_color(r, g, b)
            if not l:
                pdf.set_x(pdf.l_margin); pdf.multi_cell(0, 3.1, " "); continue
            for i in range(0, len(l), 140):
                pdf.set_x(pdf.l_margin)
                pdf.multi_cell(0, 3.1, l[i:i+140] or " ")
        pdf.set_text_color(20, 20, 20)

    # --- annexes : logs ---
    def appendix(title, path, filt=None, maxlines=70):
        txt = _read(path)
        if not txt:
            return
        lines = txt.splitlines()
        if filt:
            lines = [l for l in lines if any(f in l for f in filt)]
        lines = lines[-maxlines:]
        if not lines:
            return
        pdf.add_page()
        h2("Annexe - " + title)
        pdf.set_font("Courier", "", 6.3)
        pdf.set_text_color(40, 40, 40)
        for l in lines:
            pdf.set_x(pdf.l_margin)
            pdf.multi_cell(0, 3.3, _ascii(l)[:138] or " ")

    appendix("qemu.log  (manifest, invariants, FN-ALIGN, wires)", QEMU_LOG,
             filt=["[MANIFEST]", "[INVARIANT", "[INVARIANT-SUMMARY]", "FN-ALIGN",
                   "CTRLSYS", "FBEN", "TASKW", "DANS-CORRELATEUR"], maxlines=90)
    appendix("osmocon.log  (PM MEAS, FBSB, FCCH, LOST)", OSMOCON_LOG,
             filt=["PM MEAS", "FBSB", "FCCH", "LOST", "L1CTL"], maxlines=55)
    appendix("mobile.log  (selection cellule / camp)", MOBILE_LOG,
             filt=["cell", "service", "sysinfo", "MM_EVENT", "Location", "CCCH"],
             maxlines=45)

    out = Path(out_dir) / "rapport.pdf"
    pdf.output(str(out))
    return out


# ============ Rendu mermaid (mmdc) + diagrammes temporels + Rmd/qmd ============
_MMDC = "/opt/node/bin/mmdc"
_PUPPET = "/root/.puppeteer.json"

def _mermaid_png(mmd_text, out_png):
    import subprocess, os, shutil
    mmdc = _MMDC if os.path.exists(_MMDC) else shutil.which("mmdc")
    if not mmdc:
        return None
    src_p = Path(out_png).with_suffix(".mmd")
    try:
        src_p.write_text(mmd_text, encoding="utf-8")
        env = dict(os.environ, PATH="/opt/node/bin:" + os.environ.get("PATH", ""))
        cmd = [mmdc, "-i", str(src_p), "-o", str(out_png), "-b", "white"]
        if os.path.exists(_PUPPET):
            cmd += ["-p", _PUPPET]
        subprocess.run(cmd, check=True, timeout=120, capture_output=True, env=env)
        try:
            src_p.unlink()   # nettoie le .mmd temporaire (d_*.mmd) : seuls les vrais splits restent
        except Exception:
            pass
        return Path(out_png) if Path(out_png).exists() else None
    except Exception:
        return None

def _seq_diagram(st):
    corr = "AR5=0x2a00 OK" if st.get("rank3") == "ok" else "AR5=garbage MUR"
    det = "d_fb_det>0" if st.get("detect") == "ok" else "d_fb_det=0"
    return ("sequenceDiagram\n  autonumber\n"
            "  participant BTS\n  participant GRGSM as gr-gsm\n  participant ARM\n"
            "  participant DSP\n  participant CORR as Correlateur\n"
            "  BTS->>GRGSM: FCCH / SCH (FN reelle)\n"
            "  GRGSM->>ARM: SCH decodee (BSIC=7)\n"
            "  ARM->>DSP: 0x0810 B_TASK_ABORT=0\n"
            "  ARM->>DSP: task_md=5 (FB)\n"
            "  DSP->>DSP: go-live 0xa4e4 IMR=0x52fd\n"
            "  BTS-->>DSP: frame-IT vec28 (PRIO)\n"
            "  DSP->>CORR: CALA 0xb01e -> 0x8d00\n"
            "  CORR->>CORR: kernel 0xa076 (" + corr + ")\n"
            "  CORR-->>ARM: " + det + "\n")

def _timeline_diagram(st):
    def m(k):
        return "OK" if st.get(k) == "ok" else ("wip" if st.get(k) == "wip" else "MUR")
    return ("timeline\n  title Milestones go-live -> camp\n"
            "  Boot : BTS emet : ipc-device : gr-gsm SCH OK\n"
            "  Go-live : abort=0 " + m("rank1") + " : IMR 0x52fd " + m("lost") +
            " : frame-IT " + m("fben") + "\n"
            "  Correlateur : kernel 0xa076 " + m("rank3") + " : d_fb_det " + m("detect") + "\n"
            "  Camp : mobile campe " + m("camp") + "\n")

def _temporal_plots(out_dir):
    pngs = []
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import re
        qlog = _read(QEMU_LOG); olog = _read(OSMOCON_LOG)
        def series(text, pat, gv=True):
            t, v = [], []
            for line in text.splitlines():
                mt = re.search(pat, line)
                if mt:
                    try:
                        t.append(float(mt.group(1)))
                        v.append(float(mt.group(2)) if gv else 0.0)
                    except Exception:
                        pass
            return t, v
        specs = [
            (r"\+([\d.]+)s .*FN-ALIGN .*delta=(-?\d+)", True,
             "FN-ALIGN : delta (sch_fn - trx_fn) vs temps", "delta (trames)", "#b07a00", "t_fnalign.png"),
            (r"\+([\d.]+)s PM MEAS.*?(-?\d+)\s*dBm at baseband", True,
             "PM MEAS baseband (dBm) vs temps", "dBm", "#1f7a7a", "t_pm.png"),
            (r"\+([\d.]+)s .*FN-ALIGN sch_fn=(\d+)", True,
             "FN reelle BTS (sch_fn) vs temps", "FN", "#3a5fa0", "t_fn.png"),
        ]
        srcmap = {"t_pm.png": olog}
        for pat, gv, title, ylab, col, fn in specs:
            text = srcmap.get(fn, qlog)
            t, v = series(text, pat, gv)
            if not t:
                continue
            plt.figure(figsize=(9, 2.5)); plt.plot(t, v, lw=0.8, color=col)
            plt.title(title); plt.xlabel("t (s)"); plt.ylabel(ylab)
            plt.grid(alpha=.3); plt.tight_layout()
            p = Path(out_dir) / fn; plt.savefig(p, dpi=130); plt.close()
            pngs.append((title, p))
        # LOST cumulatif
        t, _ = series(olog, r"\+([\d.]+)s LOST", False)
        if t:
            plt.figure(figsize=(9, 2.5)); plt.plot(t, range(1, len(t) + 1), lw=0.9, color="#a31a1a")
            plt.title("LOST cumulatif vs temps"); plt.xlabel("t (s)"); plt.ylabel("LOST cumul")
            plt.grid(alpha=.3); plt.tight_layout()
            p = Path(out_dir) / "t_lost.png"; plt.savefig(p, dpi=130); plt.close()
            pngs.append(("LOST cumulatif vs temps", p))
    except Exception:
        pass
    return pngs

def _all_diagram_pngs(out_dir, st):
    res = []
    g = _mermaid_png(_mermaid(st), str(Path(out_dir) / "d_grafcet.png"))
    if g: res.append(("GRAFCET (mermaid interprete)", g))
    sq = _mermaid_png(_seq_diagram(st), str(Path(out_dir) / "d_sequence.png"))
    if sq: res.append(("Sequence go-live (ARM/DSP/correlateur)", sq))
    tl = _mermaid_png(_timeline_diagram(st), str(Path(out_dir) / "d_timeline.png"))
    if tl: res.append(("Timeline milestones", tl))
    for mmd, title in (("pipeline.qmd.mmd", "Pipeline (arbre mermaid)"),
                       ("detail.qmd.mmd", "Detail tests par module (UML)")):
        p = Path(out_dir) / mmd
        if not p.exists():
            p = Path(out_dir).parent / mmd
        if p.exists():
            out = str(Path(out_dir) / (mmd.replace(".mmd", "") + ".png"))
            r = _mermaid_png(p.read_text(encoding="utf-8", errors="replace"), out)
            if r: res.append((title, r))
    res += _temporal_plots(out_dir)
    return res

def _report_md_body(st, ts):
    tot, path, rows = _weighted_pct(st)
    out = ["**Progression chemin (ponderee, ordonnee) : %d %%**  " % path,
           "FBSB 25%% + camping 22%% = 47%% du but ; total etapes %d %%  " % tot,
           "delta FN %s - LOST %s - %s" % (st.get("last_delta"), st.get("lost_n"), ts), "",
           "| Etape | Poids | Etat | Description |", "|---|---|---|---|"]
    for code, label, w, stt, got in rows:
        badge = {"ok": "OK", "wip": "en cours", "bad": "MUR"}[stt]
        out.append("| %s | %d%% | %s | %s |" % (code, w, badge, label))
    return "\n".join(out)

def _strip_ctrl(s):
    """Retire les caracteres de controle C0/C1 (U+0080 etc.) que LaTeX refuse."""
    import re
    return re.sub(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]", "", s or "")

def _diagram_section(out_dir, st):
    """Markdown des diagrammes : uniquement les PNG REELLEMENT generes (existants).
    Evite les 'Could not fetch resource t_*.png' quand un plot n'a pas de donnees."""
    md = []
    for title, p in _all_diagram_pngs(out_dir, st):
        try:
            if Path(p).exists():
                md.append("## " + title + "\n\n![" + title + "](" + Path(p).name + ")\n")
        except Exception:
            pass
    return "\n".join(md) if md else "_(aucun diagramme genere)_\n"

def _write_mmd_files(out_dir, st):
    """Ecrit chaque diagramme mermaid dans son propre .mmd A COTE du qmd/Rmd."""
    out = Path(out_dir); files = {}
    for name, gen in (("grafcet", _mermaid), ("sequence", _seq_diagram), ("timeline", _timeline_diagram)):
        try:
            txt = _deaccent(gen(st)).replace("'", "")
            (out / (name + ".mmd")).write_text(txt, encoding="utf-8")
            files[name] = name + ".mmd"
        except Exception:
            pass
    return files

def _temporal_png_md(out_dir, st):
    md = []
    for title, p in _all_diagram_pngs(out_dir, st):
        nm = Path(p).name
        if nm.startswith("t_") and Path(p).exists():
            md.append("## " + title + "\n\n![" + title + "](" + nm + ")\n")
    return "\n".join(md) if md else "_(pas de donnees log pour les plots temporels)_\n"

def write_qmd(out_dir, ts=""):
    st = derive_status()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    tr = _read(str(Path(out_dir) / "test_results.md")) or _read(str(Path(out_dir).parent / "test_results.md"))
    parts = []
    parts.append("---\ntitle: \"Calypso GSM - GRAFCET go-live & tests\"\nsubtitle: \"%s\"\n"
                 "format:\n  html:\n    toc: true\n    code-fold: true\nmermaid:\n  theme: default\n---\n" % ts)
    parts.append("# Synthese\n\n" + _report_md_body(st, ts) + "\n")
    _write_mmd_files(out_dir, st)   # grafcet.mmd / sequence.mmd / timeline.mmd splittes A COTE (source editable)
    # Contenu INLINE (le %%| file: externe n'est pas supporte par tous les Quarto ;
    # l'inline rend partout, comme test_results.qmd). ASCII + <br/>->espace + sans quote-cassantes.
    def _mm_inline(txt):
        return _deaccent(txt).replace("<br/>", " ").replace("<br>", " ").replace("'", "")
    for _t, _g in (("GRAFCET interprete", _mermaid), ("Sequence go-live", _seq_diagram), ("Timeline milestones", _timeline_diagram)):
        parts.append("# " + _t + "\n\n```{mermaid}\n" + _mm_inline(_g(st)) + "\n```\n")
    parts.append("# Diagrammes temporels\n\n" + _temporal_png_md(out_dir, st))
    parts.append("# Rapport de tests COMPLET (test_results.md)\n\n" + _strip_ctrl((tr or "_(absent)_").replace("'", "")) + "\n")
    p = out / "rapport_final.qmd"
    p.write_text(_deaccent("\n".join(parts)), encoding="utf-8")
    return p

def write_rmd(out_dir, ts=""):
    st = derive_status()
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    tr = _read(str(Path(out_dir) / "test_results.md")) or _read(str(Path(out_dir).parent / "test_results.md"))
    parts = []
    parts.append("---\ntitle: \"Calypso GSM - GRAFCET go-live & tests\"\nsubtitle: \"%s\"\n"
                 "output:\n  html_document:\n    toc: true\n    toc_float: true\n---\n" % ts)
    parts.append("```{r setup, include=FALSE}\nknitr::opts_chunk$set(echo = FALSE)\n```\n")
    parts.append("# Synthese\n\n" + _report_md_body(st, ts) + "\n")
    # PNG rendus par mmdc (bulletproof, aucune dep) ; seulement ceux qui existent
    _mm = _write_mmd_files(out_dir, st)   # .mmd splittes A COTE du Rmd
    for _t, _k, _png in (("GRAFCET interprete", "grafcet", "d_grafcet.png"),
                         ("Sequence go-live", "sequence", "d_sequence.png"),
                         ("Timeline milestones", "timeline", "d_timeline.png")):
        if (Path(out_dir) / _png).exists():
            parts.append("# " + _t + "\n\n![" + _t + "](" + _png + ")  \n_source mermaid : " + _mm.get(_k, _k + ".mmd") + "_\n")
    parts.append("# Diagrammes temporels\n\n" + _temporal_png_md(out_dir, st))
    parts.append("# Rapport de tests COMPLET (test_results.md)\n\n```\n" + _strip_ctrl((tr or "(absent)").replace("'", "")) + "\n```\n")
    p = out / "rapport.Rmd"
    p.write_text(_deaccent("\n".join(parts)), encoding="utf-8")
    return p


def write_grafcet(out_dir, ts: str = "") -> list:
    """Écrit grafcet.html + grafcet.md (+ rapport_final.pdf) dans out_dir."""
    st = derive_status()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    h = out / "grafcet.html"
    m = out / "grafcet.md"
    h.write_text(render_html(st, ts), encoding="utf-8")
    m.write_text(render_md(st, ts), encoding="utf-8")
    paths = [h, m]
    try:
        paths.append(write_pdf(out_dir, ts))
    except Exception:
        pass  # PDF best-effort (fpdf2 requis)
    # qmd unifie produit par conftest (rapport.qmd) ; ici seulement le Rmd
    try: paths.append(write_rmd(out_dir, ts))
    except Exception: pass
    return paths


if __name__ == "__main__":
    import sys
    d = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
    paths = write_grafcet(d)
    print("grafcet écrit:", *[str(p) for p in paths])
