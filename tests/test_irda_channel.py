"""
test_irda_channel.py — canal IrDA fw debug (Phase 2 plan IrDA).

Marker : `runtime_irda`. Tests gradés selon l'état du déploiement :

  - Etat actuel (post-redesign run.sh du 2026-07-01, cf. bash_scripts/run.sh
    §1bis "IrDA active peer") : le canal IrDA est OFF par défaut
    (CALYPSO_IRDA_PEER=0) car le trafic IrDA live vole le timing TDMA de la
    L1 GSM. L'ANCIENNE capture passive (tools/irda_capture.py, /tmp/
    irda_capture.pid) a été retirée de l'orchestration run.sh : un sniffer
    passif ne peut pas maintenir un lien avec un firmware IrDA SECONDARY
    (il faut un PRIMARY actif qui fasse XID/SNRM). Le remplaçant prévu,
    un peer actif (tools/irda_peer.py, /tmp/irda_peer.pid), est référencé
    dans run.sh mais n'est pas encore implémenté sur ce host au moment de
    cette relecture (2026-07-03) : `find / -iname irda_peer.py` ne renvoie
    rien. => canal structurellement silencieux tant que l'un des deux n'est
    pas opérationnel : xfail attendu sur les tests qui dépendent d'un
    firmware IrDA actif, pass attendu sur ceux qui ne dépendent que du PTY
    UART_IRDA lui-même (toujours alloué par QEMU, cf. `label serial1` dans
    qemu.log, que quelqu'un y parle ou non).
  - Post-peer-actif (irda_peer.py implémenté + CALYPSO_IRDA_PEER=1) :
    fw-irda.log alimenté, boot marker et throughput testables.

Cf. PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md pour le détail des
phases historiques (doc introuvable sur ce host au moment de cette
relecture — référencée aussi depuis conftest.py, irda_capture.py,
run_backup.sh et make_diag_bundle.sh, donc probablement pas spécifique à
ce fichier ; à vérifier par un humain si elle doit être recréée/déplacée).
"""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
IRDA_PTY  = os.environ.get("CALYPSO_IRDA_PTY", "/tmp/irda.pty.link")
FW_IRDA_LOG = Path(os.environ.get("CALYPSO_FW_IRDA_LOG", "/tmp/fw-irda.log"))
QEMU_LOG = os.environ.get("CALYPSO_QEMU_LOG", "/root/qemu.log")
IRDA_PEER_PID = os.environ.get("CALYPSO_IRDA_PEER_PID", "/tmp/irda_peer.pid")
INSIDE = os.path.exists("/.dockerenv")


def _dexec(cmd: list[str], timeout: float = 4.0) -> subprocess.CompletedProcess:
    """Run a command inside the container."""
    return subprocess.run(["docker", "exec", CONTAINER] + cmd,
                          capture_output=True, text=True, timeout=timeout)


def _real_irda_pty() -> str:
    """Résout le VRAI PTY UART_IRDA (serial1) alloué par QEMU pour cette
    session, en parsant qemu.log — exactement comme le fait run.sh (§1bis,
    "Waiting for QEMU PTY serial1 (UART_IRDA) allocation") :
        "... char device redirected to /dev/ptsN (label serial1)"

    Remplace un ancien fallback hardcodé sur `/dev/pts/3`, vérifié FAUX en
    session (2026-07-03) : `/dev/pts/3` pointait en réalité sur un bash de
    `gapk-start.sh` (sans rapport avec QEMU), pendant que le vrai serial1
    était `/dev/pts/8` — les PTY sont alloués dynamiquement par le kernel
    selon TOUS les process du container, pas seulement QEMU, donc un numéro
    fixe ne peut pas être fiable d'une session à l'autre.
    """
    pattern = r"redirected to (/dev/pts/\d+) \(label serial1\)"
    if INSIDE:
        try:
            text = Path(QEMU_LOG).read_text(errors="replace")
        except Exception:
            return ""
    else:
        r = _dexec(["cat", QEMU_LOG])
        if r.returncode != 0:
            return ""
        text = r.stdout
    m = re.search(pattern, text)
    return m.group(1) if m else ""


def _pty_exists() -> tuple[bool, str]:
    """Find a usable PTY path : symlink CALYPSO_IRDA_PTY (peer actif monté)
    en priorité, sinon le vrai PTY serial1 (UART_IRDA) résolu depuis qemu.log."""
    candidates = [IRDA_PTY]
    real = _real_irda_pty()
    if real:
        candidates.append(real)
    # Inside container
    if INSIDE:
        for p in candidates:
            if Path(p).exists():
                return (True, p)
        return (False, "")
    # Host : check via docker exec
    for p in candidates:
        r = _dexec(["test", "-e", p])
        if r.returncode == 0:
            return (True, p)
    return (False, "")


def _read_pty_bytes(window_s: float, path: str = None) -> bytes:
    """Sample atomique : capture jusqu'à `window_s` secondes."""
    ok, pty_path = _pty_exists()
    if not ok and not path:
        return b""
    target = path or pty_path
    if INSIDE:
        try:
            with open(target, "rb", buffering=0) as f:
                # Non-blocking read avec timeout
                import select
                buf = b""
                t_end = time.time() + window_s
                while time.time() < t_end:
                    r, _, _ = select.select([f], [], [], 0.2)
                    if r:
                        chunk = os.read(f.fileno(), 4096)
                        if not chunk: break
                        buf += chunk
                return buf
        except Exception:
            return b""
    # Host : timeout cat via docker exec
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "timeout", str(window_s), "cat", target],
            capture_output=True, timeout=window_s + 2)
        return r.stdout
    except Exception:
        return b""


def _grep_fw_irda(pattern: str) -> int:
    """Compte les lignes matching dans fw-irda.log (côté approprié)."""
    if INSIDE:
        try:
            if not FW_IRDA_LOG.exists(): return 0
            text = FW_IRDA_LOG.read_text(errors="replace")
        except Exception:
            return 0
    else:
        r = _dexec(["cat", str(FW_IRDA_LOG)])
        if r.returncode != 0: return 0
        text = r.stdout
    return sum(1 for line in text.splitlines() if pattern in line)


def _fw_irda_log_exists() -> bool:
    if INSIDE:
        return FW_IRDA_LOG.exists()
    return _dexec(["test", "-f", str(FW_IRDA_LOG)]).returncode == 0


# -----------------------------------------------------------------------------
@pytest.mark.runtime_irda
def test_irda_pty_exists():
    """Un PTY IrDA est disponible (symlink peer actif, ou PTY serial1 réel résolu via qemu.log)."""
    ok, path = _pty_exists()
    assert ok, (f"aucun PTY IrDA trouvé (cherché : {IRDA_PTY}, "
                f"puis serial1 réel via {QEMU_LOG})")
    print(f"\n[irda] pty path = {path}")


@pytest.mark.runtime_irda
def test_irda_pty_readable():
    """Le PTY est ouvrable en lecture sans erreur."""
    ok, path = _pty_exists()
    if not ok:
        pytest.skip("PTY absent (cf. test_irda_pty_exists)")
    if INSIDE:
        with open(path, "rb", buffering=0) as f:
            pass  # ouvrir/fermer suffit
    else:
        # Host : essai lecture courte via docker exec
        r = _dexec(["timeout", "0.3", "cat", path], timeout=2)
        # `timeout 0.3 cat` fini avec code 124 (timeout normal) ou 0 si EOF
        assert r.returncode in (0, 124), \
            f"docker exec cat {path} returncode={r.returncode} stderr={r.stderr[:120]!r}"


@pytest.mark.runtime_irda
def test_irda_boot_marker_present():
    """Le marker 'fw-irda boot' apparaît dans fw-irda.log (Phase 0.5)."""
    if not _fw_irda_log_exists():
        pytest.xfail(
            "fw-irda.log absent — canal IrDA OFF par défaut "
            "(CALYPSO_IRDA_PEER=0, cf. run.sh §1bis) : l'ancienne capture "
            "passive (irda_capture.py) a été retirée de l'orchestration "
            "(un sniffer ne peut pas maintenir un lien avec un firmware "
            "IrDA SECONDARY), et le peer actif de remplacement "
            "(tools/irda_peer.py) n'est pas encore implémenté sur ce host "
            "(vérifié 2026-07-03). Cf. PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md")
    n = _grep_fw_irda("fw-irda boot") + _grep_fw_irda("boot OK")
    if n == 0:
        pytest.xfail("Boot marker absent — instrumenter cons_puts(UART_IRDA, ...) dans compal_e88/init.c:105")
    assert n >= 1


@pytest.mark.runtime_irda
def test_irda_channel_produces_bytes():
    """Sur 5s steady state, au moins quelques bytes arrivent du firmware."""
    data = _read_pty_bytes(5.0)
    if len(data) == 0:
        pytest.xfail(
            "Canal silencieux — le firmware compal_e88 est un IrDA SECONDARY "
            "qui ne parle qu'à un PRIMARY ayant fait XID/SNRM (cf. run.sh "
            "§1bis) ; sans peer actif connecté (CALYPSO_IRDA_PEER=0 par "
            "défaut, et tools/irda_peer.py pas encore implémenté sur ce "
            "host), le lien IrDA ne se monte jamais et rien n'est émis."
        )
    assert len(data) >= 16, f"trop peu de bytes en 5s : {len(data)}"
    print(f"\n[irda] received {len(data)} bytes in 5s")


@pytest.mark.runtime_irda
def test_irda_does_not_break_uart_modem():
    """L'activité IrDA ne perturbe pas le UART_MODEM (osmocon → L1CTL)."""
    # osmocon vivant ?
    r = _dexec(["pgrep", "-c", "osmocon"])
    n_osmocon = int(r.stdout.strip() or "0")
    assert n_osmocon >= 1, "osmocon mort — UART_MODEM cassé"

    # Pas d'erreur récente sur le canal modem dans bridge.log
    r = _dexec(["bash", "-c", "tail -n 50 /tmp/bridge.log 2>/dev/null"])
    tail = r.stdout.lower()
    for err in ("modem read error", "uart timeout", "broken pipe"):
        assert err not in tail, f"erreur UART_MODEM détectée : {err}"


@pytest.mark.runtime_irda
def test_irda_log_has_timestamp_prefix():
    """Le log fw-irda respecte le format `<epoch> +<rel>s [fw-irda] ...`."""
    if not _fw_irda_log_exists():
        pytest.skip("fw-irda.log absent (canal IrDA OFF par défaut / peer actif non démarré)")
    if INSIDE:
        try:
            first = next(iter(FW_IRDA_LOG.open()), "")
        except Exception:
            pytest.skip("lecture fw-irda.log échouée")
    else:
        r = _dexec(["bash", "-c", f"head -1 {FW_IRDA_LOG} 2>/dev/null"])
        first = r.stdout
    if not first.strip():
        pytest.skip("fw-irda.log vide")
    assert re.match(r"\d+\.\d+\s+\+\d+\.\d+s\s+\[fw-irda\]", first), \
        f"format inattendu : {first[:120]!r}"


@pytest.mark.runtime_irda
def test_irda_capture_process_alive():
    """irda_peer.py (peer actif UART_IRDA) tourne si CALYPSO_IRDA_PEER=1.

    Note (2026-07-03) : ce test vérifiait auparavant /tmp/irda_capture.pid,
    le PID de l'ANCIENNE capture passive (tools/irda_capture.py). Cette
    capture est retirée de l'orchestration run.sh depuis le redesign du
    2026-07-01 (cf. run.sh §1bis : "L'ANCIENNE capture passive est
    RETIREE : un sniffer ne peut pas faire vivre un secondary, donc le log
    restait vide"), remplacée par un peer actif (tools/irda_peer.py) qui
    écrit /tmp/irda_peer.pid. Vérifié en session : un processus
    irda_capture.py ORPHELIN d'une session précédente (2026-06-30, avant le
    redesign) restait vivant dans /proc en pointant sur un PTY qui n'existe
    plus, ce qui faisait passer ce test au vert sans rapport avec l'état
    réel du canal IrDA de la session QEMU courante — un faux positif. On
    vérifie donc désormais l'artefact produit par le mécanisme courant."""
    pid_file = IRDA_PEER_PID
    r = _dexec(["test", "-f", pid_file])
    if r.returncode != 0:
        pytest.xfail(
            f"Peer IrDA actif pas démarré (pas de {pid_file}) — "
            "CALYPSO_IRDA_PEER=0 par défaut (cf. run.sh §1bis) et/ou "
            "tools/irda_peer.py pas encore implémenté sur ce host "
            "(vérifié 2026-07-03)")
    pid_r = _dexec(["cat", pid_file])
    try:
        pid = int(pid_r.stdout.strip())
    except ValueError:
        pytest.fail(f"contenu PID invalide : {pid_r.stdout!r}")
    alive = _dexec(["test", "-d", f"/proc/{pid}"])
    assert alive.returncode == 0, f"process irda_peer pid={pid} mort"


@pytest.mark.runtime_irda
def test_irda_throughput_below_saturation():
    """Sur 10s, le canal n'approche pas la saturation 115200 bps (~14 KB/s)."""
    data = _read_pty_bytes(10.0)
    if len(data) == 0:
        pytest.skip("canal silencieux — pas de throughput à mesurer")
    rate_bps = (len(data) * 8) / 10.0
    # 115200 bps - 10% marge = 103 680 bps
    assert rate_bps < 103_680, \
        f"throughput {rate_bps:.0f} bps approche saturation 115200 (>{103_680})"
    print(f"\n[irda] {len(data)}B / 10s = {rate_bps:.0f} bps ({rate_bps/115200*100:.1f}% saturation)")
