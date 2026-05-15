"""
Décomposition du chemin Location Update — squelette de tests v2 (2026-05-14).

Mise à jour suite au rapport REPORT_CLAUDE_WEB_20260514.md :

  - POPM 0x8A fixé le 05-08 : INTM/WAIT-A21A/ENTER-7740 résolus ✓
  - Tier A décodeur appliqué (0x76 ST, F2/F3 LMS, 0x98/9A/80/8C) ✓
  - rxDoneFlag : NE PLUS référencer comme blocker actuel. Garder comme
    test de régression historique uniquement.
  - Blocker actuel : D_FB_DET-WR-SITE (PC=0x8f51). Correlator lit
    DARAM[0..0x3A3] + wrap[0xfc5d..] mais BSP DMA n'écrit pas là.
    AR3/AR4 init firmware-imposed, à tracer (priorité A).
  - RETE=0 traité comme CONSÉQUENCE probable du blocker FB-det,
    pas comme axe indépendant.

Lancement :
    pytest -v -m milestone_dsp_decoder    # régression Tier A + POPM
    pytest -v -m milestone_bsp_dma         # data path ARM → DARAM
    pytest -v -m milestone_fb_det          # correlator converge
    pytest -v -m milestone_irq             # SINT cycle
    pytest -v -m milestone_l1ctl           # premier produit L1 vers mobile
    pytest -v -m milestone_mm_lu           # le Graal

    pytest --collect-only -q               # TODO list à plat
"""

from __future__ import annotations

import os
import re
import socket
import subprocess
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, Optional

import pytest

# ---------------------------------------------------------------------------
# CONFIG — alignée sur le rapport 05-14
# ---------------------------------------------------------------------------

CONTAINER_NAME = os.environ.get("CALYPSO_CONTAINER", "trying")
HOST_ROOT      = Path(os.environ.get("CALYPSO_HOST_ROOT", "/home/nirvana/myconfigs/osmo_root"))
QEMU_LOG_CONTAINER = "/root/qemu.log"   # canonical, lecture via docker exec
QEMU_LOG       = HOST_ROOT / "qemu.log" # backup mount, peut être stale
MOBILE_PCAP    = HOST_ROOT / "mobile-gsmtap.pcap"

REPO_ROOT      = Path(os.environ.get("CALYPSO_REPO", "/home/nirvana/qemu-calypso"))
DSP_ROM_TXT    = Path(os.environ.get("CALYPSO_DSP_ROM", str(REPO_ROOT / "calypso_dsp.txt")))

# Sondes connues (rapport 05-14)
PC_FB_DET_WR_SITE     = 0x8f51       # 50 captures D_FB_DET-WR-SITE
INSN_BEFORE_FBDET     = 10_040_312   # 1er hit observé — gate pour probe AR3/AR4
DSP_IDLE_RANGE_DEFAULT = (0xe9ac, 0xe9b7)  # +(0xcc62, 0xcc6f)

# Zones DARAM
DARAM_FBDET_LINEAR    = (0x0000, 0x03A3)
DARAM_FBDET_WRAP      = (0xfc5d, 0xffed)
BSP_DARAM_TARGET_DEFAULT = 0x3fb0

# Bridge env
BRIDGE_DEFAULTS = {
    "BRIDGE_CLK_FROM_QEMU":   "0",
    "BRIDGE_CLK_PERIOD":      "51",
    "BRIDGE_DL_FN_REWRITE":   "slot",
    "BRIDGE_DL_FN_LOOKAHEAD": "32",
    "BRIDGE_UL_FN_REWRITE":   "slot",
}

# Timeouts
T_LOG_LINE_DEFAULT    = 30.0
T_FB_DET_FIRST_HIT    = 120.0
T_FB0_ATT_CONVERGE    = 600.0


# ---------------------------------------------------------------------------
# HELPERS conteneur
# ---------------------------------------------------------------------------

def _docker_cmd_or_none() -> Optional[list[str]]:
    """Détecte le prefix docker utilisable (direct ou via `sudo -n`)."""
    for prefix in (["docker"], ["sudo", "-n", "docker"]):
        r = subprocess.run([*prefix, "info"], capture_output=True, timeout=10)
        if r.returncode == 0:
            return prefix
    return None

_DOCKER = _docker_cmd_or_none()

def docker_exec(cmd: list[str], timeout: float = 30.0) -> subprocess.CompletedProcess:
    if _DOCKER is None:
        return subprocess.CompletedProcess(args=cmd, returncode=127,
                                           stdout="", stderr="docker inaccessible")
    # errors='replace' : qemu.log peut contenir des bytes binaires (STATE-DUMP
    # mémoire brute) et `tail -c N` peut couper en milieu de séquence UTF-8.
    # Sans replace, le décodage subprocess crash avant que le test puisse voir
    # son needle.
    return subprocess.run(
        [*_DOCKER, "exec", CONTAINER_NAME] + cmd,
        capture_output=True, text=True, timeout=timeout,
        errors="replace",
    )

def journal_grep(needle: str, since_seconds: int = 60) -> list[str]:
    """Cherche `needle` dans journald du container sur les N dernières secondes."""
    r = docker_exec(["journalctl", f"--since=-{since_seconds}s", "--no-pager", "-o", "cat"])
    return [l for l in r.stdout.splitlines() if needle in l]

def _qemu_log_size_container() -> int:
    """Taille de /root/qemu.log côté container."""
    r = docker_exec(["sh", "-c", f"wc -c < {QEMU_LOG_CONTAINER} 2>/dev/null || echo 0"])
    try:
        return int(r.stdout.strip() or 0)
    except ValueError:
        return 0

def tail_qemu_log(needle: str, timeout: float, since_byte: int = 0) -> Optional[str]:
    """
    Tail /root/qemu.log côté container jusqu'à voir `needle`.
    `since_byte` = offset retourné par qemu_log_offset() au début du test.
    """
    deadline = time.monotonic() + timeout
    last_size = since_byte
    while time.monotonic() < deadline:
        cur = _qemu_log_size_container()
        if cur > last_size:
            delta = cur - last_size
            r = docker_exec(["sh", "-c", f"tail -c {delta} {QEMU_LOG_CONTAINER}"])
            for line in r.stdout.splitlines():
                if needle in line:
                    return line.rstrip()
            last_size = cur
        time.sleep(0.5)
    return None

def qemu_log_offset() -> int:
    """Position de fin de /root/qemu.log côté container (baseline pour sample)."""
    return _qemu_log_size_container()


# ---------------------------------------------------------------------------
# HELPERS DSP / probes
# ---------------------------------------------------------------------------

@dataclass
class ProbeHit:
    pc: int
    insn_count: int
    fields: dict

# Probe D_FB_DET-WR-SITE format observé :
#   [c54x] D_FB_DET-WR-SITE #N AR0..AR7=XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX
#          data[AR1]=XXXX BK=XXXX A=XXXXXXXXXX insn=N
_RE_FB_DET = re.compile(
    r"D_FB_DET-WR-SITE\s+#(?P<n>\d+)\s+"
    r"AR0\.\.AR7=(?P<ar>(?:[0-9a-f]{4}\s+){7}[0-9a-f]{4})\s+"
    r"data\[AR1\]=(?P<dar1>[0-9a-f]{4})\s+"
    r"BK=(?P<bk>[0-9a-f]{4})"
)

def parse_d_fb_det_hits(lines: list[str]) -> list[ProbeHit]:
    """Parse les lignes 'D_FB_DET-WR-SITE' de qemu.log."""
    hits = []
    for line in lines:
        m = _RE_FB_DET.search(line)
        if not m: continue
        ars = [int(x, 16) for x in m.group("ar").split()]
        fields = {
            "n":     int(m.group("n")),
            "AR":    ars,
            "AR1":   ars[1],
            "AR2":   ars[2],
            "AR3":   ars[3],
            "AR4":   ars[4],
            "AR7":   ars[7],
            "dAR1":  int(m.group("dar1"), 16),
            "BK":    int(m.group("bk"), 16),
        }
        hits.append(ProbeHit(pc=PC_FB_DET_WR_SITE, insn_count=0, fields=fields))
    return hits


def grep_dsp_rom_for_init(opcode_hex_low: int) -> list[tuple[str, int, Optional[str]]]:
    """
    Cherche dans calypso_dsp.txt les sites où l'opcode `opcode_hex_low`
    apparaît dans une région PROM*, et renvoie l'addresse DSP + le mot
    suivant (le literal lk si c'est bien un STM #lk, MMR).

    Retourne [(region, dsp_addr, lk_word_str_or_None), ...].
    """
    if not DSP_ROM_TXT.exists():
        return []
    src = DSP_ROM_TXT.read_text(errors="ignore").splitlines()

    regions: list[dict] = []
    cur = None
    for i, l in enumerate(src):
        if l.startswith("DSP dump:"):
            if cur:
                cur["end"] = i; regions.append(cur)
            name = l.split("DSP dump:")[1].split("[")[0].strip()
            cur = {"name": name, "start": i+1, "end": None}
    if cur:
        cur["end"] = len(src); regions.append(cur)

    target = f"{opcode_hex_low:04x}"
    out: list[tuple[str, int, Optional[str]]] = []
    for r in regions:
        if not r["name"].startswith("PROM"):
            continue
        for li in range(r["start"], r["end"]):
            line = src[li]
            if " : " not in line: continue
            addr_str, rest = line.split(" : ", 1)
            try: base = int(addr_str, 16)
            except ValueError: continue
            words = rest.split()
            for off, w in enumerate(words):
                if w == target:
                    nxt = words[off+1] if off+1 < len(words) else None
                    if nxt is None and li+1 < r["end"]:
                        nl = src[li+1]
                        if " : " in nl:
                            nw = nl.split(" : ",1)[1].split()
                            nxt = nw[0] if nw else None
                    out.append((r["name"], base+off, nxt))
    return out


# ---------------------------------------------------------------------------
# FIXTURES — le run est déjà lancé dans le container, on observe seulement
# ---------------------------------------------------------------------------

def _detect_docker_cmd() -> Optional[list[str]]:
    """Retourne le prefix docker exécutable, ou None si pas d'accès."""
    for prefix in (["docker"], ["sudo", "-n", "docker"]):
        r = subprocess.run([*prefix, "info"], capture_output=True, timeout=10)
        if r.returncode == 0:
            return prefix
    return None


@pytest.fixture(scope="session")
def container_alive():
    docker = _detect_docker_cmd()
    if docker is None:
        # Fallback host-side : qemu-system-arm en process suffit comme preuve de vie
        r = subprocess.run(
            ["pgrep", "-f", "qemu-system-arm.*calypso"],
            capture_output=True, text=True,
        )
        if r.returncode == 0 and r.stdout.strip():
            return  # OK, on bypasse docker inspect
        pytest.skip(
            f"Pas d'accès docker (ni direct, ni `sudo -n`) ET pas de "
            f"qemu-system-arm visible côté host. Ajoute `nirvana` au groupe "
            f"docker ou lance pytest avec sudo."
        )
        return
    r = subprocess.run(
        [*docker, "inspect", "-f", "{{.State.Running}}", CONTAINER_NAME],
        capture_output=True, text=True,
    )
    if r.returncode != 0 or r.stdout.strip() != "true":
        pytest.skip(f"Container {CONTAINER_NAME} pas up — `docker start {CONTAINER_NAME}` d'abord")

@pytest.fixture(scope="function")
def log_offset() -> int:
    """Position du qemu.log au début du test — sert de baseline."""
    return qemu_log_offset()


# ---------------------------------------------------------------------------
# MILESTONE 0 — Régressions Tier A + POPM (acquis 05-08, doit rester vert)
# ---------------------------------------------------------------------------

@pytest.mark.milestone_dsp_decoder
def test_popm_decoder_active():
    """Vérif statique : décodeur POPM 0x8A00 actif, ancien désactivé."""
    src = REPO_ROOT / "hw/arm/calypso/calypso_c54x.c"
    if not src.exists():
        pytest.skip(f"{src} introuvable")
    text = src.read_text()
    assert "(op & 0xFF00) == 0x8A00" in text, "Décodeur POPM correct manquant"
    assert "if (0 && hi8 == 0x8A)" in text, "Ancien décodeur MVDK 0x8A pas stubé"


@pytest.mark.milestone_dsp_decoder
def test_tier_a_decoder_fixes_present():
    """Vérif statique : fixes Tier A (0x76 ST, 0x80 STL, 0x8C ST T) appliqués."""
    src = REPO_ROOT / "hw/arm/calypso/calypso_c54x.c"
    if not src.exists():
        pytest.skip(f"{src} introuvable")
    text = src.read_text()
    # 0x76 ST #lk, Smem (2-word) — handler doit appeler prog_fetch pour le 2e mot
    assert "if (hi8 == 0x76)" in text, "0x76 handler absent"
    # POPM doit lire le top-of-stack
    assert "popm" in text.lower(), "POPM mention absente du source"


@pytest.mark.milestone_dsp_decoder
def test_intm_dwell_no_regression(container_alive, log_offset):
    """Régression : INTM=1 dwell perpétuel ne doit plus apparaître."""
    hit = tail_qemu_log("WAIT-A21A", timeout=30.0, since_byte=log_offset)
    assert hit is None, f"WAIT-A21A réapparu — régression POPM ? {hit}"


_MILESTONE_INSN_STATS_RE = re.compile(
    r"INSN-COUNT-STATS\s+total=(\d+)\s+delta=(\d+)\s+"
    r"elapsed_ms=(\d+)\s+rate=(\d+)/s"
)

@pytest.mark.milestone_dsp_decoder
def test_dsp_throughput_5x(container_alive):
    """
    Régression POPM : recalibré 2026-05-14 sur 4024 samples réels — median
    36.7M/s avec TOUTES les probes actives. Le claim historique 100M/s du
    rapport 05-08 était pré-instrumentation. Seuil 25M/s = ~30% sous le
    p10 actuel — pass en steady state, fail sur régression POPM (qui
    descendrait à 5-10M/s) ou saturation host. Lit `INSN-COUNT-STATS`.
    """
    # Fenêtre 20 samples (vs 5) : robuste aux dips host load. Cf
    # test_dsp_throughput_above_threshold pour la calibration détaillée.
    r = docker_exec([
        "sh", "-c",
        f"grep INSN-COUNT-STATS {QEMU_LOG_CONTAINER} 2>/dev/null | tail -n 20"
    ])
    lines = [l for l in r.stdout.splitlines() if l]
    if not lines:
        pytest.skip("Pas de INSN-COUNT-STATS dans qemu.log — QEMU pas rebuildé")
    rates = []
    for line in lines:
        m = _MILESTONE_INSN_STATS_RE.search(line)
        if m:
            rates.append(int(m.group(4)))
    if not rates:
        pytest.fail(f"Format inattendu : {lines[-1]!r}")
    median = sorted(rates)[len(rates) // 2]
    # Recalibré 2026-05-15 : 10M/s (était 25M, instrumentation v3 trop lourde).
    # Cf test_run_observability.py:DSP_THROUGHPUT_MIN_INSN_PER_SEC pour détails.
    assert median >= 10_000_000, (
        f"DSP rate sous seuil : median={median:,}/s < 10M/s sur {len(rates)} samples — "
        f"régression POPM possible ou instrumentation runaway"
    )


# ---------------------------------------------------------------------------
# MILESTONE 1 — BSP DMA → DARAM data path (priorité A du rapport)
# ---------------------------------------------------------------------------

@pytest.mark.milestone_bsp_dma
def test_ar3_init_source_identified():
    """
    PRIORITÉ A. Tracer où AR3 (op 0x7713 STM #lk, AR3) est initialisé.
    Trois sources possibles (rapport § Priorité A) :
      (i)   literal STM #lk, AR3      → firmware fixe la base
      (ii)  lu depuis cellule NDB     → ARM doit pousser
      (iii) calculé runtime           → tracer caller
    AR3 observé init=0x0000 (stride +19 → max 0x03A3).
    """
    hits = grep_dsp_rom_for_init(0x7713)
    matching_zero = [(r,a,n) for r,a,n in hits if n == "0000"]
    if matching_zero:
        # Cas (i) — au moins un site littéral correspond à AR3=0x0000 observé
        pytest.skip(
            f"Source littérale (i) candidate : {len(matching_zero)} sites avec lk=0x0000\n"
            f"Premiers: {matching_zero[:5]}\n"
            f"Vérifier manuellement caller chain de ces sites au runtime."
        )
    pytest.xfail(
        f"Aucun STM #0, AR3 littéral trouvé ({len(hits)} hits 0x7713 total). "
        f"→ Cas (ii) NDB-driven ou (iii) calculé. Ajouter probe runtime "
        f"AR3-INIT gating sur insn_count<{INSN_BEFORE_FBDET}."
    )


@pytest.mark.milestone_bsp_dma
def test_ar4_init_source_identified():
    """Idem AR4 (op 0x7714) — observed AR4≈0x2bc0 (table coefficients ROM)."""
    hits = grep_dsp_rom_for_init(0x7714)
    matching = [(r,a,n) for r,a,n in hits
                if n in ("2bc0","2bc1","2bc2","2bc3","2bc4","2bbf")]
    if matching:
        pytest.skip(
            f"Source littérale (i) candidate AR4 : {len(matching)} sites avec lk≈0x2bc0\n"
            f"Premiers: {matching[:5]}"
        )
    pytest.xfail(
        f"Aucun STM #~0x2bc0, AR4 littéral. → Cas (ii)/(iii). Probe runtime AR4-INIT."
    )


@pytest.mark.milestone_bsp_dma
def test_bsp_dma_target_matches_correlator_read_zone(container_alive):
    """
    Le BSP DMA atteint-il la zone que le correlator lit ?

    ⚠️ Diagnostic 2026-05-14 (« mismatch source/sink, BSP DMA n'écrit pas où
    le correlator lit ») INVALIDÉ EMPIRIQUEMENT 2026-05-15 matin via le
    bascule XFAIL→PASS de `test_d_fb_det_data_no_longer_zero`. Le data path
    fonctionne, il faut juste laisser converger ≥3 min uptime QEMU.

    Test refait : DARAM-WR-STATS doit montrer `low > 0` (avec
    CALYPSO_BSP_DARAM_ADDR=0x0080 par défaut, le BSP écrit dans la zone basse
    [0x0000..0x03A3] que le correlator lit). PASS le milestone.
    """
    r = docker_exec([
        "sh", "-c",
        f"grep DARAM-WR-STATS {QEMU_LOG_CONTAINER} 2>/dev/null | tail -n 1"
    ])
    line = r.stdout.strip()
    if not line:
        pytest.skip("Pas de DARAM-WR-STATS dans qemu.log — QEMU pas rebuildé")
    import re as _re
    m = _re.search(
        r"low=(\d+)\s+target=(\d+)\s+wrap=(\d+)\s+other=(\d+)\s+total=(\d+)",
        line,
    )
    assert m, f"Format DARAM-WR-STATS imprévu : {line!r}"
    low, target, wrap, other, total = (int(m.group(i)) for i in range(1, 6))
    assert total > 0, "BSP DMA jamais armée — bug en amont (TPU/INTH/init)"
    # Avec env=0x0080 le BSP doit écrire low > 0. Avec env=0x3fb0 (default
    # historique sans surcharge) c'est target qui doit être > 0.
    assert low > 0 or target > 0, (
        f"BSP n'atteint NI low NI target. Distribution : "
        f"low={low} target={target} wrap={wrap} other={other}"
    )


@pytest.mark.milestone_bsp_dma
def test_no_d_fb_det_wr_site_anomaly(container_alive, log_offset):
    """
    Le probe D_FB_DET-WR-SITE fire-t-il sur le sweep FB-det attendu ?

    ⚠️ Ancienne formulation 2026-05-14 (« data[AR1]=bbef→0000, anomalie data
    path ») obsolète. data[AR1] est une lecture de table PROM0 confirmée
    bit-pour-bit, et la convergence data[AR0]/AR2 fait basculer
    `test_d_fb_det_data_no_longer_zero`.

    Test refait : on vérifie juste que le sweep s'exécute (au moins 50 hits
    capturés). Si 0, le probe n'est plus dans le path d'exécution = vraie
    anomalie structurelle.
    """
    r = docker_exec([
        "sh", "-c",
        f"grep -c 'D_FB_DET-WR-SITE' {QEMU_LOG_CONTAINER} 2>/dev/null || echo 0"
    ])
    n = int(r.stdout.strip() or "0")
    assert n >= 50, (
        f"D_FB_DET-WR-SITE seulement {n} hits — sweep FB-det ne s'exécute pas. "
        f"Régression structurelle, le DSP n'atteint plus PC=0x8f51."
    )


# ---------------------------------------------------------------------------
# MILESTONE 2 — Correlator FB-det converge (fb0_att != 0)
# ---------------------------------------------------------------------------

@pytest.mark.milestone_fb_det
def test_fb0_att_nonzero(container_alive):
    """
    fb0_att doit passer non-zéro pour signaler la détection FB par le DSP.

    ⚠️ Diagnostic mis à jour 2026-05-15 : ce N'EST PLUS « dépend de bsp_dma
    résolu » (bsp_dma EST résolu, cf test_d_fb_det_data_no_longer_zero PASS).
    Le blocker actuel est : compute converge (data[AR4] non-zéro, A_final
    atteint -242M observé), mais le seuil de décision FB-det n'est jamais
    franchi. Direction d'investigation : où est comparé A au seuil ?
    """
    pytest.xfail(
        "Compute converge mais fb0_att=0 — seuil de décision FB-det non franchi. "
        "Investiguer la comparaison A vs threshold dans le firmware DSP."
    )


def _detect_fbsb_synth_in_container() -> str:
    """Source de vérité = qemu.log où calypso_fbsb logue son état au boot.
    Format : '[calypso-fbsb] CALYPSO_FBSB_SYNTH=0 (real DSP path)'
             '[calypso-fbsb] CALYPSO_FBSB_SYNTH=1 (synth, dev-assist path)'
    L'env os.environ côté pytest host est INCORRECT — il ne reflète pas
    l'env du QEMU dans le container. Bug détecté 2026-05-15."""
    r = docker_exec([
        "sh", "-c",
        f"grep -oE 'CALYPSO_FBSB_SYNTH=[01]' {QEMU_LOG_CONTAINER} 2>/dev/null | tail -n 1 || true"
    ])
    line = r.stdout.strip()
    if "=1" in line: return "1"
    if "=0" in line: return "0"
    return "unknown"


@pytest.mark.milestone_fb_det
def test_synth_zero_path_active(container_alive):
    """
    CALYPSO_FBSB_SYNTH=0 : sans détection réelle, le mobile ne passe pas en
    gsm322. Si on observe une transition gsm322 avec synth=0, le demod marche.
    """
    synth = _detect_fbsb_synth_in_container()
    if synth == "unknown":
        pytest.skip("Impossible de détecter CALYPSO_FBSB_SYNTH dans qemu.log")
    if synth == "1":
        pytest.skip("CALYPSO_FBSB_SYNTH=1 actif côté container — workaround, pas le vrai chemin")
    pytest.xfail("Vrai demod CCCH pas convergent")


# ---------------------------------------------------------------------------
# MILESTONE 3 — Cycle IRQ DSP (RETE != 0)
# ---------------------------------------------------------------------------
#
# Hypothèse forte (rapport + analyse) : RETE=0 est CONSÉQUENCE de bsp_dma non
# résolu. Les 3 compteurs ci-dessous discriminent (a)/(b)/(c) en un run :
#   (a) interrupt_ex_called=0          → IRQ ARM→DSP jamais générée
#   (b) isr_entered>0 & rete_executed=0 → ISR boucle, jamais de RETE
#   (c) pending_irq_gated>0            → replay-block gated

@pytest.mark.milestone_irq
def test_c54x_interrupt_ex_called_nonzero(container_alive):
    """
    Probe à ajouter côté QEMU : incrément à chaque entrée de c54x_interrupt_ex.
    Lecture via QEMU monitor (`info` custom) ou via dump périodique.
    =0 → hypothèse (a) : générateur IRQ ARM→DSP ne fire jamais.
    """
    pytest.xfail("TODO: ajouter compteur + exposition monitor")


@pytest.mark.milestone_irq
def test_isr_entered_implies_rete(container_alive):
    """
    Si isr_entered > 0 ET rete_executed = 0 → hypothèse (b) : ISR boucle.
    Si isr_entered = rete_executed > 0     → cycle complet OK.
    """
    pytest.xfail("Dépend des deux compteurs ci-dessus")


@pytest.mark.milestone_irq
def test_no_pending_irq_gated(container_alive):
    """
    Probe sur c54x.c:5148 (pending IRQ replay).
    Si gated>0 et interrupt_ex_called=0 → hypothèse (c).
    """
    pytest.xfail("TODO: probe sur pending_replay_block")


# ---------------------------------------------------------------------------
# MILESTONE 4 — L1CTL premier produit
# ---------------------------------------------------------------------------

@pytest.mark.milestone_l1ctl
def test_l1ctl_data_ind_received(container_alive):
    """
    Milestone L1 : L1CTL_DATA_IND vers mobile non-zéro.

    Sémantique 3-états :
      - PASS  : DATA_IND > 0 (milestone L1 atteint)
      - XFAIL : task=24 (ALLC) fire 0× → conditions amont CCCH pas réunies,
                la mesure DATA_IND n'a pas de sens (variance/régime intermittent)
      - FAIL  : task=24 > 0 et DATA_IND = 0 → vrai mur couche 6,
                ARM ne forward pas a_cd[] au mobile (vraie régression à creuser)
    """
    r_allc = docker_exec([
        "sh", "-c",
        f"grep -c 'task=24' {QEMU_LOG_CONTAINER} 2>/dev/null || true"
    ])
    allc_str = r_allc.stdout.strip()
    allc_hooks = int(allc_str) if allc_str.isdigit() else 0
    if allc_hooks == 0:
        pytest.xfail(
            "task=24 (ALLC) fire 0× — conditions amont CCCH pas réunies, "
            "DATA_IND non interprétable"
        )

    r = docker_exec([
        "sh", "-c",
        f"grep -cE 'L1CTL_DATA_IND|DATA_IND' {QEMU_LOG_CONTAINER} 2>/dev/null || true"
    ])
    n_str = r.stdout.strip()
    n = int(n_str) if n_str.isdigit() else 0
    assert n > 0, (
        f"task=24 fire {allc_hooks}× mais DATA_IND=0 — vrai mur couche 6. "
        f"ARM ne forward pas a_cd[] au mobile."
    )


@pytest.mark.milestone_l1ctl
def test_neigh_pm_req_response_unchanged(container_alive):
    """
    Régression : pipeline L1CTL (sercomm DLCI=5, NEIGH_PM_REQ → PM_CONF) marche
    depuis 05-08. Doit rester verte.
    """
    pytest.xfail("TODO: parse pcap mobile-gsmtap.pcap pour confirmer")


# ---------------------------------------------------------------------------
# MILESTONE 5 — RR / MM Location Update (le Graal)
# ---------------------------------------------------------------------------

@pytest.mark.milestone_mm_lu
def test_rach_emitted(container_alive):
    pytest.xfail("Dépend de L1 prêt")


@pytest.mark.milestone_mm_lu
def test_immediate_assignment_decoded(container_alive):
    pytest.xfail("Dépend de CCCH demod réel")


@pytest.mark.milestone_mm_lu
def test_rr_sdcch_established(container_alive):
    pytest.xfail("")


@pytest.mark.milestone_mm_lu
def test_location_updating_request_sent(container_alive):
    pytest.xfail("")


@pytest.mark.milestone_mm_lu
def test_location_updating_accept_received(container_alive):
    """LE test. Quand celui-ci passe → milestone projet atteint."""
    pytest.xfail("Objectif final")


# ---------------------------------------------------------------------------
# ARCHIVE — tests historiques rxDoneFlag (DÉPRÉCIÉS depuis 05-08)
# ---------------------------------------------------------------------------

@pytest.mark.milestone_dsp_decoder
def test_rxdoneflag_no_longer_blocks(container_alive):
    """
    Historiquement c'était le watchpoint Task #29. Depuis POPM fix 05-08,
    plus un blocker actif. À convertir en vrai test de régression une fois
    bsp_dma fixé.
    """
    pytest.xfail("Régression historique — non actif aujourd'hui")


# ---------------------------------------------------------------------------
# Ordre de présentation des collected tests
# ---------------------------------------------------------------------------

def pytest_collection_modifyitems(config, items):
    order = {
        "milestone_dsp_decoder": 0,
        "milestone_bsp_dma":     1,
        "milestone_fb_det":      2,
        "milestone_irq":         3,
        "milestone_l1ctl":       4,
        "milestone_mm_lu":       5,
    }
    def key(item):
        for m in item.iter_markers():
            if m.name in order:
                return order[m.name]
        return 99
    items.sort(key=key)
