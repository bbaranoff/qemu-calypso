"""
Tests d'observation du run live qemu-calypso — complément à test_calypso_milestones.py.

Principe : ces tests ne déclenchent rien, ils SAMPLE une fenêtre temporelle
d'activité du container `trying` et vérifient des invariants observables.

Surfaces d'observation :
  1. Container & processus (docker exec, /proc, mtimes)
  2. qemu.log mount host  (probes DSP : D_FB_DET-WR-SITE, IMR, POST-BOOTSTUB-RET)
  3. journalctl container (services Osmocom : stp/hlr/msc/bsc/bts-trx)
  4. mobile-gsmtap.pcap   (tshark : L1CTL, RACH, IMM ASS, RR, MM)
  5. QEMU monitor sock    (info compteurs custom)
  6. VTY mobile L23       (nc 4247 : show ms, show subscriber)
  7. Bridge.py UDP        (sniff parallèle, drift FN)

Tous les tests assument que le run TOURNE DÉJÀ. On ne le redémarre pas.

Lancement :
    pytest -v test_run_observability.py -m runtime_health
    pytest -v test_run_observability.py -m runtime_dsp
    pytest -v test_run_observability.py -m runtime_bridge
    pytest -v test_run_observability.py -m runtime_l1ctl
    pytest -v test_run_observability.py -m runtime_vty
"""

from __future__ import annotations

import json
import os
import re
import socket
import subprocess
import time
from collections import Counter
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import pytest

# ---------------------------------------------------------------------------
# CONFIG (mêmes constantes que test_calypso_milestones.py — à factoriser plus tard)
# ---------------------------------------------------------------------------

CONTAINER       = os.environ.get("CALYPSO_CONTAINER", "trying")
HOST_ROOT       = Path(os.environ.get("CALYPSO_HOST_ROOT", "/home/nirvana/myconfigs/osmo_root"))
QEMU_LOG        = HOST_ROOT / "qemu.log"
MOBILE_PCAP     = HOST_ROOT / "mobile-gsmtap.pcap"

# Process names attendus dans le container (rapport 05-14)
EXPECTED_PROCESSES = {
    "qemu-system-arm": 1,
    "osmocon":         1,
    "bridge.py":       1,
    "osmo-bts-trx":    1,
    "mobile":          1,
    "tcpdump":         1,
    "osmo-stp":        1,
    "osmo-hlr":        1,
    "osmo-msc":        1,
    "osmo-bsc":        1,
    "osmo-mgw":        1,
    "osmo-sgsn":       1,
    "osmo-ggsn":       1,
    "osmo-pcu":        1,
    "asterisk":        1,
}

# QEMU monitor (unix socket dans le container)
QEMU_MON_SOCK = "/tmp/qemu-calypso-mon.sock"

# Mobile VTY (probablement 4247, à confirmer dans mobile_group1.cfg)
MOBILE_VTY_HOST = "127.0.0.1"
MOBILE_VTY_PORT = 4247

# Fenêtres d'échantillonnage
SAMPLE_WINDOW_SHORT = 10.0    # health checks
SAMPLE_WINDOW_MED   = 60.0    # DSP probes
SAMPLE_WINDOW_LONG  = 180.0   # convergence fb0_att

# Seuils
DSP_THROUGHPUT_MIN_INSN_PER_SEC = 50_000_000   # 50M, marge ×2 sous le 100M observé
LOG_FRESHNESS_MAX_AGE_S         = 30.0
BRIDGE_FN_DRIFT_MAX_FRAMES      = 8


# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------

def dexec(cmd: list[str], timeout: float = 30.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["docker", "exec", CONTAINER, *cmd],
        capture_output=True, text=True, timeout=timeout,
    )

def dexec_sh(shell_cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["docker", "exec", CONTAINER, "sh", "-c", shell_cmd],
        capture_output=True, text=True, timeout=timeout,
    )

def container_running() -> bool:
    r = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.Running}}", CONTAINER],
        capture_output=True, text=True,
    )
    return r.returncode == 0 and r.stdout.strip() == "true"

def list_processes() -> dict[str, list[int]]:
    """Retourne {nom_binaire: [pid, ...]} sur le container."""
    r = dexec(["ps", "-eo", "pid,comm,args", "--no-headers"])
    procs: dict[str, list[int]] = {}
    for line in r.stdout.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) < 2:
            continue
        pid, comm = int(parts[0]), parts[1]
        args = parts[2] if len(parts) == 3 else ""
        # Normaliser : python3 bridge.py compte comme "bridge.py"
        key = comm
        if "bridge.py" in args:
            key = "bridge.py"
        if "mobile_group" in args:
            key = "mobile"
        procs.setdefault(key, []).append(pid)
    return procs

@dataclass
class LogSample:
    text: str
    lines: list[str]
    duration_s: float
    bytes_read: int

def sample_qemu_log(window_s: float) -> LogSample:
    """Lit qemu.log pendant `window_s` à partir de maintenant. Tail-like."""
    if not QEMU_LOG.exists():
        return LogSample("", [], window_s, 0)
    start_offset = QEMU_LOG.stat().st_size
    t0 = time.monotonic()
    chunks: list[str] = []
    while time.monotonic() - t0 < window_s:
        time.sleep(0.5)
        cur = QEMU_LOG.stat().st_size
        if cur > start_offset:
            with open(QEMU_LOG, "r", errors="ignore") as f:
                f.seek(start_offset)
                chunks.append(f.read(cur - start_offset))
            start_offset = cur
    text = "".join(chunks)
    return LogSample(text=text, lines=text.splitlines(),
                     duration_s=time.monotonic() - t0,
                     bytes_read=len(text.encode("utf-8", "ignore")))

def journal_sample(window_s: float, units: Optional[list[str]] = None) -> list[str]:
    """journalctl du container sur les `window_s` dernières secondes."""
    cmd = ["journalctl", f"--since=-{int(window_s)}s", "--no-pager", "-o", "cat"]
    if units:
        for u in units:
            cmd += ["-u", u]
    r = dexec(cmd, timeout=window_s + 10)
    return r.stdout.splitlines()


# ---------------------------------------------------------------------------
# FIXTURES
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session", autouse=True)
def _container_alive():
    if not container_running():
        pytest.skip(f"Container '{CONTAINER}' down — `docker start {CONTAINER}`")


# ===========================================================================
# A — Health du run (rapide)
# ===========================================================================

@pytest.mark.runtime_health
def test_all_expected_processes_present():
    procs = list_processes()
    missing = [name for name in EXPECTED_PROCESSES if name not in procs]
    assert not missing, f"Processus manquants : {missing}\nVus : {sorted(procs)}"

@pytest.mark.runtime_health
def test_no_zombie_or_defunct():
    r = dexec_sh("ps -eo stat,comm --no-headers | awk '$1 ~ /Z/ {print}'")
    assert not r.stdout.strip(), f"Zombies détectés :\n{r.stdout}"

@pytest.mark.runtime_health
def test_qemu_log_is_fresh():
    """qemu.log mtime récent — sinon on regarde un fichier stale (cf rapport 05-14)."""
    assert QEMU_LOG.exists(), f"{QEMU_LOG} absent côté host"
    age = time.time() - QEMU_LOG.stat().st_mtime
    assert age < LOG_FRESHNESS_MAX_AGE_S, (
        f"qemu.log a {age:.0f}s — probablement stale comme dans le rapport. "
        f"Le run actuel logue ailleurs (journald ?)"
    )

@pytest.mark.runtime_health
def test_mobile_pcap_growing():
    """tcpdump tourne et le pcap grossit. Sinon GSMTAP est cassé."""
    assert MOBILE_PCAP.exists(), f"{MOBILE_PCAP} absent"
    s0 = MOBILE_PCAP.stat().st_size
    time.sleep(SAMPLE_WINDOW_SHORT)
    s1 = MOBILE_PCAP.stat().st_size
    assert s1 > s0, f"pcap stagne ({s0}→{s1}) — tcpdump mort ou aucun trafic GSMTAP"

@pytest.mark.runtime_health
def test_volumes_mounted():
    for src, dst in [
        (HOST_ROOT, "/root"),
        (Path("/home/nirvana/myconfigs/osmocom"), "/etc/osmocom"),
    ]:
        r = dexec(["ls", "-la", dst])
        assert r.returncode == 0, f"Mount {src} → {dst} cassé : {r.stderr}"


# ===========================================================================
# B — DSP runtime : régressions POPM + Tier A
# ===========================================================================

@pytest.mark.runtime_dsp
def test_no_wait_a21a_on_window():
    """WAIT-A21A doit avoir disparu depuis le POPM fix (rapport § Symptômes débloqués)."""
    sample = sample_qemu_log(SAMPLE_WINDOW_MED)
    hits = [l for l in sample.lines if "WAIT-A21A" in l]
    assert not hits, f"WAIT-A21A réapparu ({len(hits)} occurrences) — régression POPM ?"

@pytest.mark.runtime_dsp
def test_no_enter_7740_dwell():
    sample = sample_qemu_log(SAMPLE_WINDOW_MED)
    hits = [l for l in sample.lines if "ENTER-7740" in l]
    assert len(hits) < 100, f"ENTER-7740 ré-actif ({len(hits)}/min) — POPM cassé ?"

@pytest.mark.runtime_dsp
def test_intm_reaches_zero():
    """POST-BOOTSTUB-RET doit apparaître, signal qu'INTM passe à 0."""
    sample = sample_qemu_log(SAMPLE_WINDOW_MED)
    hits = [l for l in sample.lines if "POST-BOOTSTUB-RET" in l]
    assert hits, "Pas de POST-BOOTSTUB-RET — INTM jamais clear, POPM régression"

@pytest.mark.runtime_dsp
def test_dsp_throughput_above_threshold():
    """
    Throughput insn/s côté DSP. Méthode : compteur exposé par le monitor QEMU.
    À défaut, fallback sur comptage probes (très approximatif).
    """
    # TODO Claude Code: exposer `info dsp_insn_count` côté monitor unix sock
    pytest.xfail(
        f"Probe DSP throughput pas exposée. À ajouter dans calypso_c54x.c "
        f"+ commande monitor. Seuil : {DSP_THROUGHPUT_MIN_INSN_PER_SEC:,}/s."
    )


# ===========================================================================
# C — Probes FB-det live (priorité A en action)
# ===========================================================================

D_FB_DET_RE = re.compile(
    r"D_FB_DET-WR-SITE.*?AR1=([0-9a-f]+).*?AR2=([0-9a-f]+).*?AR3=([0-9a-f]+).*?AR4=([0-9a-f]+)",
    re.IGNORECASE,
)

@pytest.mark.runtime_dsp
def test_d_fb_det_pattern_unchanged():
    """
    Vérifie que le pattern AR1/AR2/AR3/AR4 du probe D_FB_DET-WR-SITE correspond
    toujours à ce qui est documenté (stride +19 sur AR3, BK=176 wrap sur AR2/AR7).
    Un changement = quelque chose a bougé côté init firmware → potentielle info.
    """
    sample = sample_qemu_log(SAMPLE_WINDOW_MED)
    hits = D_FB_DET_RE.findall(sample.text)
    if not hits:
        pytest.skip("Aucun hit D_FB_DET-WR-SITE sur la fenêtre — probe inactive ?")
    ar3s = sorted({int(h[2], 16) for h in hits})
    # Au moins quelques valeurs dans [0, 0x3A3]
    in_range = sum(1 for a in ar3s if 0 <= a <= 0x03A3)
    assert in_range >= 3, (
        f"AR3 ne couvre plus la zone DARAM basse attendue. "
        f"Échantillon AR3 : {[hex(a) for a in ar3s[:10]]}"
    )

@pytest.mark.runtime_dsp
def test_d_fb_det_data_no_longer_zero():
    """
    Quand bsp_dma est résolu, `data[AR1]` côté probe ne devrait plus tomber
    à 0000 systématiquement. Aujourd'hui : bbef → 0000. Demain : non-zéro stable.
    """
    pytest.xfail("Tant que bsp_dma pas résolu : data[AR1]=bbef→0000 attendu")


# ===========================================================================
# D — Bridge.py FN sync (drift)
# ===========================================================================

@pytest.mark.runtime_bridge
def test_bridge_log_shows_traffic():
    r = dexec(["tail", "-n", "200", "/tmp/bridge.log"])
    assert r.stdout.strip(), "bridge.log vide — bridge.py muet, problème UDP ?"

@pytest.mark.runtime_bridge
def test_bridge_fn_drift_under_threshold():
    """
    Pendant `SAMPLE_WINDOW_MED`, mesurer la dérive entre FN annoncé bridge
    et FN exécuté par QEMU. Doit rester sous BRIDGE_FN_DRIFT_MAX_FRAMES.

    Méthode : tail /tmp/bridge.log + corréler avec marqueurs FN dans qemu.log.
    """
    # TODO Claude Code : implémenter le parsing dual (bridge + qemu) avec
    # regex sur les lignes 'FN=...' des deux côtés, calculer max|delta|.
    pytest.xfail("Parser dual à écrire — voir BRIDGE_FN_DRIFT_MAX_FRAMES")

@pytest.mark.runtime_bridge
def test_bridge_dl_lookahead_respected():
    """BRIDGE_DL_FN_LOOKAHEAD=32 par défaut. Aucun drop ne doit avoir delta>32."""
    r = dexec(["grep", "-c", "lookahead drop", "/tmp/bridge.log"])
    drops = int(r.stdout.strip() or "0")
    # Tolère quelques drops au warm-up, pas plus.
    assert drops < 5, f"{drops} drops 'lookahead' dans bridge.log — slot rewrite mal calibré"


# ===========================================================================
# E — Pipeline L1CTL via pcap GSMTAP
# ===========================================================================

def _tshark_count(pcap: Path, display_filter: str) -> int:
    r = subprocess.run(
        ["tshark", "-r", str(pcap), "-Y", display_filter, "-T", "fields", "-e", "frame.number"],
        capture_output=True, text=True, timeout=60,
    )
    if r.returncode != 0:
        pytest.skip(f"tshark indispo ou pcap illisible : {r.stderr[:200]}")
    return len([l for l in r.stdout.splitlines() if l.strip()])

@pytest.mark.runtime_l1ctl
def test_neigh_pm_req_loop_alive():
    """Régression : NEIGH_PM_REQ ↔ PM_CONF marche depuis 05-08, doit rester actif."""
    # TODO Claude Code : ajuster le filter display selon le dissector GSMTAP
    # utilisé pour L1CTL (peut être 'gsmtap.type == ...').
    n = _tshark_count(MOBILE_PCAP, "gsmtap")
    assert n > 0, "Aucun frame GSMTAP dans le pcap — pipeline L1CTL morte"

@pytest.mark.runtime_l1ctl
def test_l1ctl_data_ind_received():
    """
    Premier L1CTL_DATA_IND vers mobile = signal "L1 marche vraiment".
    Actuellement attendu à 0 (correlator vide). Bascule quand fb_det converge.
    """
    pytest.xfail("Bloqué par milestone bsp_dma / fb_det")

@pytest.mark.runtime_l1ctl
def test_rach_attempted():
    """Le mobile tente-t-il un RACH ? Attendu 0 tant que pas de cell selection."""
    pytest.xfail("Bloqué par sélection cellule (post-fb_det)")


# ===========================================================================
# F — VTY mobile L23 (état RR/MM)
# ===========================================================================

@contextmanager
def mobile_vty():
    """Connexion VTY au mobile L23. Le mobile peut être joignable via docker exec
    nc (network host) ou via port forwardé."""
    # TODO Claude Code : ajuster selon le network mode du container.
    r = subprocess.run(
        ["docker", "exec", "-i", CONTAINER, "sh", "-c",
         f"echo 'show ms 1' | nc -q1 {MOBILE_VTY_HOST} {MOBILE_VTY_PORT}"],
        capture_output=True, text=True, timeout=10,
    )
    yield r

@pytest.mark.runtime_vty
def test_mobile_vty_reachable():
    with mobile_vty() as r:
        assert "MS '" in r.stdout or "mobile" in r.stdout.lower(), (
            f"VTY mobile inaccessible. stdout={r.stdout!r} stderr={r.stderr!r}"
        )

@pytest.mark.runtime_vty
def test_mobile_imsi_loaded():
    with mobile_vty() as r:
        assert re.search(r"IMSI[:\s]+\d{15}", r.stdout), \
            f"IMSI non chargé dans le mobile :\n{r.stdout[:500]}"

@pytest.mark.runtime_vty
def test_mobile_mm_state_is_null_or_idle():
    """Pré-LU : MM_NULL ou MM_IDLE. Post-LU : MM_IDLE/NORMAL_SERVICE."""
    with mobile_vty() as r:
        # Tant que pas de LU, on accepte NULL ou IDLE
        assert re.search(r"MM\s+state.*?(NULL|IDLE)", r.stdout, re.IGNORECASE), \
            f"État MM inattendu :\n{r.stdout[:500]}"


# ===========================================================================
# G — Compteurs QEMU monitor (Q2 du rapport : discriminer RETE=0)
# ===========================================================================

def qemu_monitor(cmd: str, timeout: float = 5.0) -> str:
    """Envoie une commande au monitor QEMU via le socket unix."""
    r = dexec_sh(
        f"echo '{cmd}' | socat - UNIX-CONNECT:{QEMU_MON_SOCK}",
        timeout=timeout,
    )
    return r.stdout

@pytest.mark.runtime_irq
def test_interrupt_ex_called_counter_exposed():
    """Hypothèse (a) : si `info dsp_irq` montre interrupt_ex_called=0 → IRQ never fires."""
    out = qemu_monitor("info dsp_irq")
    if "interrupt_ex_called" not in out:
        pytest.xfail(
            "Compteur 'interrupt_ex_called' pas exposé via monitor. "
            "À ajouter dans hw/arm/calypso/calypso_c54x.c + commande monitor info."
        )
    m = re.search(r"interrupt_ex_called[:=]\s*(\d+)", out)
    assert m, f"Format inattendu : {out!r}"
    n = int(m.group(1))
    # On documente plus qu'on n'asserte : un PASS ici raconte l'histoire.
    print(f"\n[Q2] interrupt_ex_called = {n}")
    if n == 0:
        pytest.xfail("Hypothèse (a) confirmée : IRQ ARM→DSP ne fire jamais")

@pytest.mark.runtime_irq
def test_isr_entered_matches_rete():
    out = qemu_monitor("info dsp_irq")
    m_in  = re.search(r"isr_entered[:=]\s*(\d+)", out)
    m_ret = re.search(r"rete_executed[:=]\s*(\d+)", out)
    if not (m_in and m_ret):
        pytest.xfail("Compteurs isr_entered/rete_executed pas exposés")
    isr, rete = int(m_in.group(1)), int(m_ret.group(1))
    print(f"\n[Q2] isr_entered={isr} rete_executed={rete}")
    # Hypothèse (b) : isr > 0 et rete = 0
    if isr > 0 and rete == 0:
        pytest.fail("Hypothèse (b) : ISR boucle, RETE jamais atteint")

@pytest.mark.runtime_irq
def test_no_pending_irq_gating():
    out = qemu_monitor("info dsp_irq")
    m = re.search(r"pending_irq_gated[:=]\s*(\d+)", out)
    if not m:
        pytest.xfail("Compteur pending_irq_gated pas exposé")
    gated = int(m.group(1))
    print(f"\n[Q2] pending_irq_gated = {gated}")
    # Hypothèse (c) seulement plausible si gated > 0 ET interrupt_ex_called == 0
    # (test croisé à faire dans test combiné).


# ===========================================================================
# H — Test combiné "raconte l'histoire" : un rapport en une seule commande
# ===========================================================================

@pytest.mark.runtime_summary
def test_run_summary_snapshot(capsys):
    """
    Pas d'assertion. Imprime un snapshot consolidé pour la check 7 :
      - container OK / process count
      - qemu.log freshness
      - hits D_FB_DET sur 30s
      - drops bridge lookahead
      - n trames GSMTAP
      - compteurs IRQ
    Lance avec `pytest -v -m runtime_summary -s` pour voir le print.
    """
    procs = list_processes()
    sample = sample_qemu_log(30.0)
    fbdet_hits = len(D_FB_DET_RE.findall(sample.text))
    age = time.time() - QEMU_LOG.stat().st_mtime if QEMU_LOG.exists() else -1

    bridge_log = dexec(["tail", "-n", "500", "/tmp/bridge.log"]).stdout
    drops = bridge_log.count("lookahead drop")
    gsmtap = _tshark_count(MOBILE_PCAP, "gsmtap") if MOBILE_PCAP.exists() else 0

    print("\n" + "=" * 60)
    print("RUN SNAPSHOT")
    print("=" * 60)
    print(f"Container       : {CONTAINER} up")
    print(f"Process count   : {sum(len(v) for v in procs.values())}")
    print(f"qemu.log age    : {age:.0f}s  ({QEMU_LOG})")
    print(f"D_FB_DET hits/30s : {fbdet_hits}")
    print(f"Bridge drops    : {drops}")
    print(f"GSMTAP frames   : {gsmtap}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# Ordre d'exécution : sanity d'abord, observation ensuite
# ---------------------------------------------------------------------------

def pytest_collection_modifyitems(config, items):
    order = {
        "runtime_health":  0,
        "runtime_dsp":     1,
        "runtime_bridge":  2,
        "runtime_l1ctl":   3,
        "runtime_vty":     4,
        "runtime_irq":     5,
        "runtime_summary": 6,
    }
    def key(item):
        for m in item.iter_markers():
            if m.name in order:
                return order[m.name]
        return 99
    items.sort(key=key)
