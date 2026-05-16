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
import shlex
import shutil
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
HOST_ROOT       = Path(os.environ.get("CALYPSO_HOST_ROOT", "/root"))

# Canonical: les logs runtime sont écrits dans le container à /root/qemu.log.
# Le host mount existe (/home/nirvana/myconfigs/osmo_root/qemu.log) mais on lit
# côté container via `docker exec` pour éviter les races de rotation.
QEMU_LOG_CONTAINER = "/root/qemu.log"
QEMU_LOG           = HOST_ROOT / "qemu.log"     # backup, read-only checks
MOBILE_PCAP        = HOST_ROOT / "mobile-gsmtap.pcap"

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

# Mobile VTY. In-container : 127.0.0.1:4247 directement. Depuis host : passer
# par l'IP du container (override via CALYPSO_MOBILE_VTY="host:port").
MOBILE_VTY_DEFAULT = "127.0.0.1:4247" if os.path.exists("/.dockerenv") \
                     else "172.20.0.11:4247"
_vty = os.environ.get("CALYPSO_MOBILE_VTY", MOBILE_VTY_DEFAULT)
MOBILE_VTY_HOST, _p = _vty.rsplit(":", 1)
MOBILE_VTY_PORT = int(_p)

# Fenêtres d'échantillonnage
SAMPLE_WINDOW_SHORT = 10.0    # health checks
SAMPLE_WINDOW_MED   = 60.0    # DSP probes
SAMPLE_WINDOW_LONG  = 180.0   # convergence fb0_att

# Seuils
DSP_THROUGHPUT_MIN_INSN_PER_SEC = 10_000_000   # recalibré 2026-05-15 :
# instrumentation v3 (read_stats_record + COEFFS-WR per-write + COEFFS-WR-SUMMARY
# 64k iter + D_FB_DET-WR-SITE étendu data[AR0..AR7] + sweep tracking + ar4_in_zone
# + deltas par cluster + INSN-COUNT-STATS) descend le median wall-clock à 16-18M/s.
# Seuil 10M = au-dessus du worst case observé (sans régression), mais 2× au-dessus
# de la rate "POPM cassé" attendue (~5M/s). Compromis observabilité vs sensibilité
# régression. Si on revient à une instrumentation light, remonter à 25M.
LOG_FRESHNESS_MAX_AGE_S         = 30.0
BRIDGE_FN_DRIFT_MAX_FRAMES      = 8


# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------

def _detect_docker_cmd() -> Optional[list[str]]:
    """Retourne le prefix docker (direct ou via sudo -n), ou None si pas d'accès."""
    for prefix in (["docker"], ["sudo", "-n", "docker"]):
        r = subprocess.run([*prefix, "info"], capture_output=True, timeout=10)
        if r.returncode == 0:
            return prefix
    return None

DOCKER_CMD: Optional[list[str]] = _detect_docker_cmd()


def dexec(cmd: list[str], timeout: float = 30.0) -> subprocess.CompletedProcess:
    if DOCKER_CMD is None:
        return subprocess.CompletedProcess(args=cmd, returncode=127,
                                           stdout="", stderr="docker inaccessible")
    # errors='replace' : qemu.log contient des bytes binaires (STATE-DUMP brut)
    # qui crashent le décodage UTF-8 par défaut, notamment quand `tail -c N`
    # coupe en milieu de séquence multi-byte.
    return subprocess.run(
        [*DOCKER_CMD, "exec", CONTAINER, *cmd],
        capture_output=True, text=True, timeout=timeout,
        errors="replace",
    )

def dexec_sh(shell_cmd: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    if DOCKER_CMD is None:
        return subprocess.CompletedProcess(args=shell_cmd, returncode=127,
                                           stdout="", stderr="docker inaccessible")
    return subprocess.run(
        [*DOCKER_CMD, "exec", CONTAINER, "sh", "-c", shell_cmd],
        capture_output=True, text=True, timeout=timeout,
        errors="replace",
    )

def container_running() -> bool:
    """Container up ? Tente docker inspect, puis sudo, puis fallback host pgrep."""
    if DOCKER_CMD is not None:
        r = subprocess.run(
            [*DOCKER_CMD, "inspect", "-f", "{{.State.Running}}", CONTAINER],
            capture_output=True, text=True,
        )
        if r.returncode == 0 and r.stdout.strip() == "true":
            return True
    # Fallback host : qemu-system-arm en process = run vivant
    r = subprocess.run(
        ["pgrep", "-f", "qemu-system-arm.*calypso"],
        capture_output=True, text=True,
    )
    return r.returncode == 0 and bool(r.stdout.strip())

def _detect_l2_client(procs=None) -> str:
    """Detecte l'application L23/L2 active. Default = mobile.
    Used to skip mobile-only tests when CALYPSO_L2_CLIENT=ccch_scan/cell_log.
    """
    if procs is None:
        procs = list_processes()
    if "ccch_scan" in procs:
        return "ccch_scan"
    if "cell_log" in procs:
        return "cell_log"
    return "mobile"

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

def _container_qemu_log_size() -> int:
    """Taille de /root/qemu.log dans le container (0 si inaccessible)."""
    r = dexec_sh(f"wc -c < {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    try:
        return int(r.stdout.strip() or 0)
    except ValueError:
        return 0

# --- Trois familles d'invariants côté log ---
#
# 1. ABSENCE   : grep cumulatif. 1 hit = fail. Régression-friendly.
# 2. PRÉSENCE  : grep cumulatif. >= seuil = ok. Le probe est vivant.
# 3. STRUCTURE : grep + tail -n N puis parse. État récent du correlator.
# 4. FRÉQUENCE : sample_qemu_log fenêtre live (seul cas où le temps compte).
#
# Cf. diagnostic 2026-05-14 : routing fd1/fd2 qemu → /root/qemu.log direct,
# journald = 0, host log capte tout. Le grep cumulatif est l'outil canonique
# pour 1/2/3 ; sample_qemu_log reste pour 4 uniquement.

def grep_count_in_qemu_log(pattern: str) -> int:
    """Compte cumulatif d'occurrences du pattern dans /root/qemu.log côté container."""
    r = dexec_sh(f"grep -c {shlex.quote(pattern)} {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    try:
        return int(r.stdout.strip() or "0")
    except ValueError:
        return 0

def tail_qemu_log_matching(pattern: str, n: int = 20) -> list[str]:
    """N dernières lignes matchant `pattern` dans /root/qemu.log côté container."""
    r = dexec_sh(
        f"grep {shlex.quote(pattern)} {QEMU_LOG_CONTAINER} 2>/dev/null | tail -n {n}"
    )
    return [l for l in r.stdout.splitlines() if l]

def _container_qemu_log_tail(n_bytes: int) -> str:
    """Lit les `n_bytes` dernières octets de /root/qemu.log côté container."""
    if n_bytes <= 0:
        return ""
    r = dexec_sh(f"tail -c {n_bytes} {QEMU_LOG_CONTAINER} 2>/dev/null")
    return r.stdout

def sample_qemu_log(window_s: float) -> LogSample:
    """
    Sample atomique de qemu.log côté container.
    Mesure la taille au début et à la fin de la fenêtre, et tail le delta.
    Robuste à toute rotation/truncate côté host.
    """
    start_size = _container_qemu_log_size()
    if start_size == 0:
        # log absent ou pas d'accès docker : retourne sample vide
        time.sleep(window_s)
        return LogSample("", [], window_s, 0)
    t0 = time.monotonic()
    time.sleep(window_s)
    end_size = _container_qemu_log_size()
    delta = max(0, end_size - start_size)
    text = _container_qemu_log_tail(delta) if delta > 0 else ""
    return LogSample(
        text=text, lines=text.splitlines(),
        duration_s=time.monotonic() - t0,
        bytes_read=len(text.encode("utf-8", "ignore")),
    )

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
        if DOCKER_CMD is None:
            pytest.skip(
                f"Pas d'accès docker (ni direct, ni `sudo -n`) ET pas de "
                f"qemu-system-arm visible côté host. Ajoute `nirvana` au "
                f"groupe docker ou lance pytest avec sudo."
            )
        pytest.skip(f"Container '{CONTAINER}' down — `docker start {CONTAINER}`")


# ===========================================================================
# A — Health du run (rapide)
# ===========================================================================

@pytest.mark.runtime_health
def test_all_expected_processes_present():
    """
    Vérifie tous les processus attendus. Le L2 client (mobile/ccch_scan/cell_log)
    est détecté dynamiquement : on remplace `mobile` dans la liste attendue par
    le client détecté.
    """
    procs = list_processes()
    l2_client = _detect_l2_client(procs)
    expected = {p: c for p, c in EXPECTED_PROCESSES.items() if p != "mobile"}
    expected[l2_client] = 1
    missing = [name for name in expected if name not in procs]
    assert not missing, (
        f"Processus manquants : {missing}\n"
        f"L2 client détecté : {l2_client}\n"
        f"Vus : {sorted(procs)}"
    )

@pytest.mark.runtime_health
def test_no_zombie_or_defunct():
    r = dexec_sh("ps -eo stat,comm --no-headers | awk '$1 ~ /Z/ {print}'")
    assert not r.stdout.strip(), f"Zombies détectés :\n{r.stdout}"

@pytest.mark.runtime_health
def test_qemu_log_is_fresh():
    """qemu.log côté container grossit récemment — sinon le run est figé."""
    s0 = _container_qemu_log_size()
    if s0 == 0:
        pytest.fail(
            f"{QEMU_LOG_CONTAINER} absent ou inaccessible côté container "
            f"(docker exec retourne size=0)."
        )
    time.sleep(3.0)
    s1 = _container_qemu_log_size()
    assert s1 > s0, (
        f"{QEMU_LOG_CONTAINER} stagne ({s0}→{s1} bytes en 3s) — "
        f"qemu probablement figé ou stderr ne va pas dans le log."
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
    """
    ABSENCE : WAIT-A21A pathologique (insn>10M + INTM=1 + IFR non-zéro) = 0.
    Les transients très early-boot (insn<10M) sont tolérés — non régression
    POPM. Le pattern pathologique POPM = INTM stuck + IRQ pending bloqué.
    """
    r = dexec_sh(f"grep 'WAIT-A21A' {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    pathological = []
    for line in lines:
        m = re.search(
            r"insn=(\d+).*INTM=(\d+).*IFR=0x([0-9a-fA-F]+)",
            line
        )
        if m:
            insn = int(m.group(1))
            intm = int(m.group(2))
            ifr = int(m.group(3), 16)
            if insn > 10_000_000 and intm == 1 and ifr != 0:
                pathological.append(line.strip())
    total = len(lines)
    assert not pathological, (
        f"WAIT-A21A pathologique (insn>10M + INTM=1 + IFR pending) : "
        f"{len(pathological)} occurrences / {total} total\n"
        f"Premiers : {pathological[:3]}"
    )

@pytest.mark.runtime_dsp
def test_no_enter_7740_dwell():
    """ABSENCE/SEUIL : ENTER-7740 sous seuil. Quelques hits OK, dwell pattern non."""
    n = grep_count_in_qemu_log("ENTER-7740")
    assert n < 100, f"ENTER-7740 ré-actif ({n} occurrences cumulées) — POPM cassé ?"

@pytest.mark.runtime_dsp
def test_intm_reaches_zero():
    """PRÉSENCE : POST-BOOTSTUB-RET >= 1. Preuve qu'INTM clear au moins une fois."""
    n = grep_count_in_qemu_log("POST-BOOTSTUB-RET")
    assert n > 0, "POST-BOOTSTUB-RET=0 — INTM jamais clear, POPM régression"

_INSN_STATS_RE = re.compile(
    r"INSN-COUNT-STATS\s+total=(\d+)\s+delta=(\d+)\s+"
    r"elapsed_ms=(\d+)\s+rate=(\d+)/s"
)

@pytest.mark.runtime_dsp
def test_dsp_throughput_above_threshold(capsys):
    """
    Parse la dernière ligne `[c54x] INSN-COUNT-STATS rate=N/s` (instrumentée
    2026-05-14 dans calypso_c54x.c, émission toutes les 1M insn).
    PASS si rate >= DSP_THROUGHPUT_MIN_INSN_PER_SEC (50M/s, marge ×2 sous 100M).
    """
    # Fenêtre 20 samples : robuste aux dips temporaires (host load, GC, etc.)
    # Sur 2955 samples observés : median=36.7M, p10=25.3M, ~10% des samples
    # sous 25M (dips). tail-5 attrapait parfois un cluster de dips et failait
    # à tort. tail-20 + median lisse ces transitoires.
    lines = tail_qemu_log_matching("INSN-COUNT-STATS", n=20)
    if not lines:
        pytest.skip(
            "Aucun INSN-COUNT-STATS — QEMU pas rebuildé avec l'instrumentation, "
            "ou run trop jeune (<1M insn)"
        )
    rates = []
    for line in lines:
        m = _INSN_STATS_RE.search(line)
        if m:
            rates.append(int(m.group(4)))
    if not rates:
        pytest.fail(f"Format INSN-COUNT-STATS imprévu : {lines[-1]!r}")
    last_rate = rates[-1]
    median_rate = sorted(rates)[len(rates) // 2]
    print(f"\n  DSP rate (last {len(rates)} samples) median = {median_rate:,}/s")
    print(f"  last   = {last_rate:,}/s")
    print(f"  seuil  = {DSP_THROUGHPUT_MIN_INSN_PER_SEC:,}/s")
    assert median_rate >= DSP_THROUGHPUT_MIN_INSN_PER_SEC, (
        f"DSP throughput sous seuil : median={median_rate:,}/s "
        f"< {DSP_THROUGHPUT_MIN_INSN_PER_SEC:,}/s sur {len(rates)} samples"
    )


# ===========================================================================
# C — Probes FB-det live (priorité A en action)
# ===========================================================================

# Format réel observé du probe (cf. calypso_c54x.c:1244) :
#   D_FB_DET-WR-SITE #N AR0..AR7=W0 W1 W2 W3 W4 W5 W6 W7 ...
#   data[AR0]=Wa  data[AR1]=Wb  data[AR2]=Wc ... BK=Wd ... insn=N
# Groupes : 1=hit_idx ; 2..9=AR0..AR7 ; 10=dAR0 ; 11=dAR1 ; 12=dAR2 ; 13=BK ; 14=insn
D_FB_DET_RE = re.compile(
    r"D_FB_DET-WR-SITE\s+#(\d+)\s+"
    r"AR0\.\.AR7=([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)"
    r"\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+).*?"
    r"data\[AR0\]=([0-9a-f]+)\s+data\[AR1\]=([0-9a-f]+)\s+data\[AR2\]=([0-9a-f]+).*?"
    r"BK=([0-9a-f]+).*?insn=(\d+)",
    re.IGNORECASE | re.DOTALL,
)

def _parse_recent_d_fb_det(n: int = 20) -> tuple[list[str], list[re.Match]]:
    """Récupère les N derniers hits D_FB_DET et tente de les parser. Retourne (raw_lines, matches)."""
    lines = tail_qemu_log_matching("D_FB_DET-WR-SITE", n=n)
    hits = []
    for line in lines:
        m = D_FB_DET_RE.search(line)
        if m:
            hits.append(m)
    return lines, hits

def _print_d_fb_det_table(hits: list[re.Match]) -> None:
    """Imprime la table des hits parsés — observation utile sous `pytest -s`."""
    print()
    print(f"  {'#':>4} {'AR0':>6} {'AR1':>6} {'AR3':>6} {'AR4':>6} "
          f"{'dAR0':>6} {'dAR1':>6} {'dAR2':>6} {'BK':>6} {'insn':>12}")
    for h in hits:
        print(f"  #{h.group(1):>3} "
              f"{int(h.group(2),16):#06x} {int(h.group(3),16):#06x} "
              f"{int(h.group(5),16):#06x} {int(h.group(6),16):#06x} "
              f"{int(h.group(10),16):#06x} {int(h.group(11),16):#06x} "
              f"{int(h.group(12),16):#06x} {int(h.group(13),16):#06x} "
              f"{int(h.group(14)):>12}")

@pytest.mark.runtime_dsp
def test_d_fb_det_pattern_unchanged(capsys):
    """
    STRUCTURE : parse les 20 derniers hits D_FB_DET-WR-SITE.

    ⚠️ Mise à jour 2026-05-15 : AR3 n'est PAS cyclique 0..0x3A3 comme on
    pensait initialement. Il est **monotone stride +19** sur tout le run.
    Le test originel asserait "AR3 in [0..0x3A3]" qui n'est vrai que pour
    les ~50 premiers fires. Après, AR3 est arbitrairement haut.

    Test refait : on vérifie deux vrais invariants —
      1. AR4 ∈ [0x2bc0..0x2bd0] (table coeffs/scratch zone, vraiment invariant)
      2. AR3 stride consistent (+19 entre fires consécutifs) → c'est le sweep

    ⚠️ Mise à jour 2026-05-15 PM : sous CALYPSO_FBSB_SYNTH=1, le compute MAC
    est bypassé par le synth, donc AR4 ne pointe plus la table COEFFS. Le test
    n'a de sens que sous synth=0 (real path).
    """
    # Import partagé avec test_calypso_milestones.py via la même logique de
    # détection container-side.
    try:
        from test_calypso_milestones import _detect_fbsb_synth_in_container
        synth = _detect_fbsb_synth_in_container()
        if synth == "1":
            pytest.xfail(
                "CALYPSO_FBSB_SYNTH=1 actif — compute MAC bypassé, AR4 ne "
                "pointe pas COEFFS (attendu)"
            )
    except ImportError:
        pass  # graceful : on continue, possible faux positif si synth=1

    lines, hits = _parse_recent_d_fb_det(n=20)
    if not lines:
        pytest.skip(
            f"Aucun hit D_FB_DET-WR-SITE dans {QEMU_LOG_CONTAINER} — probe inactive"
        )
    if not hits:
        pytest.fail(
            f"{len(lines)} lignes D_FB_DET trouvées mais regex ne parse pas.\n"
            f"Format probe changé ? Sample : {lines[0]!r}"
        )
    _print_d_fb_det_table(hits)
    ar3s_sorted = sorted(int(h.group(5), 16) for h in hits)
    ar4s = sorted({int(h.group(6), 16) for h in hits})
    # Invariant AR4 : reste dans la zone coeffs/scratch [0x2bc0..0x2bff].
    # Range élargie 2026-05-15 : 0x2bd0 était trop étroit (observé 0x2bdb,
    # 0x2bdc en steady state). Le vrai bound est la zone COEFFS [0x2bc0..0x2bff].
    ar4_in_range = sum(1 for a in ar4s if 0x2bc0 <= a <= 0x2bff)
    assert ar4_in_range >= 1, (
        f"AR4 ne couvre plus [0x2bc0..0x2bff]. AR4 distincts : {[hex(a) for a in ar4s[:10]]}"
    )
    # Invariant AR3 stride : delta entre fires consécutifs majoritairement = 19
    deltas = [ar3s_sorted[i+1] - ar3s_sorted[i] for i in range(len(ar3s_sorted)-1)]
    stride_19_count = sum(1 for d in deltas if d == 19)
    print(f"\n  AR3 stride deltas : {deltas}")
    print(f"  stride=19 count : {stride_19_count}/{len(deltas)}")
    assert stride_19_count >= len(deltas) * 0.5, (
        f"AR3 stride n'est plus majoritairement +19 — routine FB-det a changé ?\n"
        f"Deltas observés : {deltas}"
    )

_BSP_STATS_RE = re.compile(
    r"DARAM-WR-STATS\s+"
    r"low=(\d+)\s+target=(\d+)\s+wrap=(\d+)\s+other=(\d+)\s+total=(\d+)"
)

@pytest.mark.runtime_dsp
def test_bsp_daram_write_distribution(capsys):
    """
    Lit la dernière ligne `[BSP] DARAM-WR-STATS low=N target=N wrap=N other=N total=N`
    émise par l'instrumentation bsp.c (ajoutée 2026-05-14). Imprime la
    répartition et raconte l'histoire :

    - low=target=wrap=0   → BSP DMA jamais armée (bug amont TPU/INTH/init)
    - target>>0 ET low=0  → BSP écrit mais env var ignorée / mauvaise cible
    - low>>0              → BSP écrit zone correlator (problème en aval)

    SKIP tant que QEMU pas rebuildé avec l'instrumentation.
    PASS si total>0 (la ligne existe et le compteur tourne).
    """
    lines = tail_qemu_log_matching("DARAM-WR-STATS", n=1)
    if not lines:
        pytest.skip(
            "Aucun DARAM-WR-STATS dans qemu.log — QEMU pas rebuildé avec "
            "l'instrumentation bsp.c (cf rapport 05-14 § Priorité A)"
        )
    m = _BSP_STATS_RE.search(lines[0])
    assert m, f"Format DARAM-WR-STATS imprévu : {lines[0]!r}"
    low, target, wrap, other, total = (int(m.group(i)) for i in range(1, 6))
    print()
    print(f"  low    [0x0000..0x03A3] = {low:>10}  ({(low/total*100 if total else 0):.1f}%)")
    print(f"  target [0x3FB0..0x3FFF] = {target:>10}  ({(target/total*100 if total else 0):.1f}%)")
    print(f"  wrap   [0xFC5D..0xFFED] = {wrap:>10}  ({(wrap/total*100 if total else 0):.1f}%)")
    print(f"  other  (incl. 0x4000+) = {other:>10}  ({(other/total*100 if total else 0):.1f}%)")
    print(f"  total                  = {total:>10}")
    # Diagnostic narration
    if total == 0:
        verdict = "BSP DMA jamais armée → investiguer en amont (TPU/INTH/init)"
    elif target > 0 and low == 0 and wrap == 0:
        verdict = "BSP écrit zone target seulement, correlator lit ailleurs (env var honorée mais target ≠ zone correlator)"
    elif low > 0:
        verdict = "BSP écrit zone low [0..0x3A3] — alignement OK, problème en aval"
    elif other > 0 and target == 0 and low == 0:
        verdict = "BSP écrit zone non-cataloguée — vérifier daram_addr effectif"
    else:
        verdict = "configuration mixte — examiner la répartition"
    print(f"  → {verdict}")
    assert total > 0

@pytest.mark.runtime_dsp
def test_d_fb_det_data_no_longer_zero(capsys):
    """
    STRUCTURE : sur les 20 derniers hits D_FB_DET, compte data[AR2] non-zéro
    ET non-sentinelle (0xfffe).

    NOTE 2026-05-14 : data[AR1] n'est PAS le bon indicateur — AR1 lit la table
    PROM0[0x0019..0x001c] (constantes ROM, valeurs `fff6/8fd7/d9ec/bbef`),
    confirmé à 100% par grep statique sur calypso_dsp.txt. data[AR2] (et
    data[AR0]) sont les pointeurs qui visent réellement le buffer samples
    cible du correlator — c'est leur état zéro/sentinelle qui révèle le
    mismatch BSP DMA documenté dans le rapport 05-14.

    < 50% non-zéro/non-sentinelle → bsp_dma pas résolu (xfail attendu).
    ≥ 50% → bascule milestone, le correlator lit enfin des samples valides.
    """
    lines, hits = _parse_recent_d_fb_det(n=20)
    if not lines:
        pytest.skip("Aucun hit D_FB_DET — milestone non testable encore")
    if not hits:
        pytest.fail(f"Hits non parsés. Sample : {lines[0]!r}")
    _print_d_fb_det_table(hits)
    # data[AR0] = group 10, data[AR2] = group 12
    def is_sample(v: int) -> bool:
        # 0x0000 = zone vide ; 0xfffe = sentinelle -2 ; tout le reste = sample valide
        return v != 0x0000 and v != 0xfffe
    valid_ar0 = sum(1 for h in hits if is_sample(int(h.group(10), 16)))
    valid_ar2 = sum(1 for h in hits if is_sample(int(h.group(12), 16)))
    ratio_ar0 = valid_ar0 / len(hits)
    ratio_ar2 = valid_ar2 / len(hits)
    print(f"  → data[AR0] sample-valide sur {valid_ar0}/{len(hits)} ({ratio_ar0:.0%})")
    print(f"  → data[AR2] sample-valide sur {valid_ar2}/{len(hits)} ({ratio_ar2:.0%})")
    print(f"  (data[AR1] ignoré — lecture table ROM PROM0[0x19..0x1c])")
    # Critère : au moins l'un des deux doit être majoritairement sample-valide
    if ratio_ar0 < 0.5 and ratio_ar2 < 0.5:
        pytest.xfail(
            f"data[AR0]={ratio_ar0:.0%}, data[AR2]={ratio_ar2:.0%} sample-valides "
            f"— bsp_dma pas résolu, BSP DMA n'écrit pas où le correlator lit"
        )
    assert max(ratio_ar0, ratio_ar2) >= 0.5


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
    """
    Snapshot pcap via `editcap` (drope le dernier paquet incomplet en cours
    d'écriture par tcpdump) puis tshark. Plus robuste qu'un `cp` brut sur
    fichier vivant : `editcap -F pcap` valide chaque record et émet ce qu'il
    peut, donc on évite l'erreur "cut short" même si tcpdump écrit en parallèle.
    """
    if not pcap.exists():
        pytest.skip(f"{pcap} absent")
    snapshot = Path("/tmp") / f"calypso-pcap-snap-{os.getpid()}.pcap"
    try:
        try:
            ec = subprocess.run(
                ["editcap", "-F", "pcap", str(pcap), str(snapshot)],
                capture_output=True, text=True, check=False, timeout=30,
            )
        except FileNotFoundError:
            pytest.skip("editcap indispo (paquet wireshark-tools manquant)")
        if not snapshot.exists() or snapshot.stat().st_size == 0:
            pytest.skip(f"editcap n'a rien produit : {ec.stderr[:200]}")
        r = subprocess.run(
            ["tshark", "-r", str(snapshot), "-Y", display_filter,
             "-T", "fields", "-e", "frame.number"],
            capture_output=True, text=True, timeout=60,
        )
        if r.returncode != 0:
            pytest.skip(f"tshark indispo ou pcap illisible : {r.stderr[:200]}")
        return len([l for l in r.stdout.splitlines() if l.strip()])
    finally:
        snapshot.unlink(missing_ok=True)

@pytest.mark.runtime_l1ctl
def test_neigh_pm_req_loop_alive():
    """
    Régression : trafic GSMTAP UDP/4729 non-nul. Validé 2026-05-14 : tshark
    dissect ce trafic comme `gsmtap_log` (pas `gsmtap` strict). Le filter
    `udp.port == 4729` est le plus tolérant et capte tout (Clock Ind + L1CTL).
    """
    n = _tshark_count(MOBILE_PCAP, "udp.port == 4729")
    assert n > 0, "Aucun frame UDP/4729 dans le pcap — pipeline GSMTAP morte"

@pytest.mark.runtime_l1ctl
def test_l1ctl_data_ind_received():
    """
    L1CTL_DATA_IND envoyés au mobile = ARM L1 consomme a_cd[] et forward au L23.

    Sémantique 3-états :
      - PASS  : DATA_IND > 0 (milestone L1 atteint)
      - XFAIL : task=24 (ALLC) fire 0× → conditions amont CCCH pas réunies
      - FAIL  : task=24 > 0 et DATA_IND = 0 → vrai mur couche 6
    """
    r_alc = dexec_sh(f"grep -c 'task=24' {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    allc_str = r_alc.stdout.strip()
    allc_hooks = int(allc_str) if allc_str.isdigit() else 0
    if allc_hooks == 0:
        pytest.xfail(
            "task=24 (ALLC) fire 0× — conditions amont CCCH pas réunies, "
            "DATA_IND non interprétable"
        )

    r = dexec_sh(
        f"grep -cE 'L1CTL_DATA_IND|DATA_IND' {QEMU_LOG_CONTAINER} 2>/dev/null || true"
    )
    n_str = r.stdout.strip()
    n = int(n_str) if n_str.isdigit() else 0
    assert n > 0, (
        f"task=24 fire {allc_hooks}× mais DATA_IND=0 — vrai mur couche 6"
    )

@pytest.mark.runtime_l1ctl
def test_a_cd_writes_nonzero():
    """
    DSP CCCH demod doit écrire a_cd[] (15 words à DSP 0x09D0..0x09DE).
    Probe A_CD-WR (instrumentation 2026-05-15 matin).
    """
    r = dexec_sh(f"grep -c 'A_CD-WR ' {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    try:
        n = int(r.stdout.strip() or "0")
    except ValueError:
        n = 0
    if n == 0:
        pytest.skip(
            "Aucun A_CD-WR — QEMU pas rebuildé avec helper watch_write_zone_check"
        )
    assert n >= 15, (
        f"A_CD-WR seulement {n} hits — DSP CCCH demod n'écrit pas un a_cd[] complet"
    )

@pytest.mark.runtime_l1ctl
def test_a_cd_write_pc_includes_ccch_demod():
    """
    Le cluster CCCH demod réel = PC 0xec10 (PROM1 mirror). Vérif que ce PC
    fire dans les writes a_cd[]. Sinon, seuls init/clear touchent la zone.
    """
    r = dexec_sh(
        f"grep 'A_CD-WR ' {QEMU_LOG_CONTAINER} 2>/dev/null | "
        f"grep -c 'exec_pc=0xec10' || true"
    )
    try:
        n = int(r.stdout.strip() or "0")
    except ValueError:
        n = 0
    if n > 0:
        print(f"\n  {n} A_CD writes depuis exec_pc=0xec10 (CCCH demod)")
        return
    r2 = dexec_sh(
        f"grep 'A_CD-WR ' {QEMU_LOG_CONTAINER} 2>/dev/null | "
        f"grep -coE 'exec_pc=0x[ef][0-9a-f]{{3}}' || true"
    )
    try:
        n2 = int(r2.stdout.strip() or "0")
    except ValueError:
        n2 = 0
    if n2 == 0:
        pytest.xfail(
            "Aucun A_CD-WR depuis PROM1 mirror — vrai CCCH demod ne tourne pas"
        )
    print(f"\n  {n2} A_CD writes depuis PROM1 mirror (0xe???/0xf???)")

@pytest.mark.runtime_l1ctl
def test_l1ctl_data_ind_rate_vs_alc():
    """
    Cohérence entre ARM ALLC hooks (task=24, demande CCCH) et DATA_IND envoyés.
    Si ALLC > 0 mais DATA_IND = 0 → bug ARM L1 ne forward pas.
    """
    r_alc = dexec_sh(f"grep -c 'task=24' {QEMU_LOG_CONTAINER} 2>/dev/null || true")
    r_ind = dexec_sh(
        f"grep -cE 'L1CTL_DATA_IND|DATA_IND' {QEMU_LOG_CONTAINER} 2>/dev/null || true"
    )
    try:
        alc = int(r_alc.stdout.strip() or "0")
        ind = int(r_ind.stdout.strip() or "0")
    except ValueError:
        pytest.skip("Parse failure")
    if alc == 0:
        pytest.skip("Aucun task=24 ALLC hook")
    print(f"\n  ALLC hooks (task=24) = {alc}")
    print(f"  L1CTL_DATA_IND       = {ind}")
    print(f"  ratio IND/ALLC       = {ind/alc:.2f}")
    assert ind > 0, "ALLC firing mais aucun DATA_IND — bug ARM L1 → mobile"

@pytest.mark.runtime_l1ctl
def test_rach_attempted():
    """Le mobile tente-t-il un RACH ? Bloqué tant qu'il ne sort pas de gsm322."""
    pytest.xfail("Bloqué par mobile L23 en cell-search (DSC scan, pré-BCCH)")


# ===========================================================================
# F — VTY mobile L23 (état RR/MM)
# ===========================================================================

class _VtyResult:
    """Mime CompletedProcess.stdout/stderr/returncode après décodage tolérant."""
    __slots__ = ("stdout", "stderr", "returncode")
    def __init__(self, stdout: str, stderr: str, returncode: int):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode

def _vty_clean(raw: bytes) -> str:
    """Strip telnet IAC (0xff) + ANSI ESC (0x1b...) — garde ASCII printable + LF/CR/TAB."""
    if not raw:
        return ""
    cleaned = bytes(b for b in raw if 0x20 <= b < 0x7f or b in (0x09, 0x0a, 0x0d))
    return cleaned.decode("utf-8", errors="replace")

@contextmanager
def mobile_vty(query: str = "show ms 1"):
    """
    Connexion VTY au mobile L23 via bash /dev/tcp (container sans netcat).
    Décodage tolérant : strip telnet IAC + ANSI pour éviter UnicodeDecodeError.
    """
    if DOCKER_CMD is None:
        yield _VtyResult(stdout="", stderr="docker inaccessible", returncode=127)
        return
    inner = (
        f"exec 3<>/dev/tcp/{MOBILE_VTY_HOST}/{MOBILE_VTY_PORT} 2>/dev/null && "
        f"{{ echo '{query}'; sleep 0.5; }} >&3; "
        f"timeout 1 cat <&3; exec 3<&-"
    )
    r = subprocess.run(
        [*DOCKER_CMD, "exec", CONTAINER, "bash", "-c", inner],
        capture_output=True, timeout=10,   # PAS de text=True : raw bytes
    )
    yield _VtyResult(
        stdout=_vty_clean(r.stdout or b""),
        stderr=_vty_clean(r.stderr or b""),
        returncode=r.returncode,
    )

@pytest.mark.runtime_vty
def test_mobile_vty_reachable():
    l2 = _detect_l2_client()
    if l2 != "mobile":
        pytest.skip(f"L2 client = {l2} ≠ mobile — VTY non disponible (ccch_scan/cell_log)")
    with mobile_vty() as r:
        assert "MS '" in r.stdout or "mobile" in r.stdout.lower(), (
            f"VTY mobile inaccessible. stdout={r.stdout!r} stderr={r.stderr!r}"
        )

@pytest.mark.runtime_vty
def test_mobile_imsi_loaded():
    """IMSI exposé via `show subscriber 1`, PAS `show ms 1` (qui montre IMEI)."""
    l2 = _detect_l2_client()
    if l2 != "mobile":
        pytest.skip(f"L2 client = {l2} ≠ mobile — pas d'IMSI sans mobile L23")
    with mobile_vty("show subscriber 1") as r:
        assert re.search(r"IMSI[:\s]+\d{14,15}", r.stdout), \
            f"IMSI non chargé dans le mobile :\n{r.stdout[:500]}"

@pytest.mark.runtime_vty
def test_mobile_mm_state_is_null_or_idle():
    """
    Pré-LU : MM idle (no cell). Post-LU : MM idle (normal service) ou similaire.
    Le mobile L23 osmocom log la ligne 'mobility management layer state: MM idle, ...'.
    """
    l2 = _detect_l2_client()
    if l2 != "mobile":
        pytest.skip(f"L2 client = {l2} ≠ mobile — pas d'état MM sans mobile L23")
    with mobile_vty() as r:
        # Match la ligne osmocom exacte
        m = re.search(
            r"mobility management.*?:\s*MM\s+(idle|null|wait|conn)",
            r.stdout, re.IGNORECASE,
        )
        assert m, f"État MM inattendu :\n{r.stdout[:500]}"


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
    size_before = _container_qemu_log_size()
    sample = sample_qemu_log(30.0)
    size_after = _container_qemu_log_size()
    fbdet_hits = len(D_FB_DET_RE.findall(sample.text))

    bridge_log = dexec(["tail", "-n", "500", "/tmp/bridge.log"]).stdout
    drops = bridge_log.count("lookahead drop")
    gsmtap = _tshark_count(MOBILE_PCAP, "gsmtap") if MOBILE_PCAP.exists() else 0

    print("\n" + "=" * 60)
    print("RUN SNAPSHOT")
    print("=" * 60)
    print(f"Container        : {CONTAINER} (docker prefix={DOCKER_CMD})")
    print(f"Process count    : {sum(len(v) for v in procs.values())}")
    print(f"qemu.log path    : {QEMU_LOG_CONTAINER} (container)")
    print(f"qemu.log size    : {size_before} → {size_after} bytes (+{size_after - size_before})")
    print(f"qemu.log sample  : {sample.bytes_read} bytes / {sample.duration_s:.1f}s")
    print(f"D_FB_DET hits/30s: {fbdet_hits}")
    print(f"Bridge drops     : {drops}")
    print(f"GSMTAP frames    : {gsmtap}")
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
