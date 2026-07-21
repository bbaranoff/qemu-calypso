"""
test_irda_channel.py — canal IrDA fw debug (Phase 2 plan IrDA).

Marker : `runtime_irda`. Tests gradés selon l'état du déploiement :

  - Pre-Phase-0.5 (état actuel)        : 7 xfail + 1 fail boot marker
  - Post-Phase-0.5 (cons_puts au boot) : test_boot_marker passe
  - Post-Phase-3 (capture host actif)   : irda_capture.pid + fw-irda.log
  - Post-Phase-5 (instrumentation fw)   : tous passent

Cf. PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md pour le détail des phases.
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
INSIDE = os.path.exists("/.dockerenv")


def _dexec(cmd: list[str], timeout: float = 4.0) -> subprocess.CompletedProcess:
    """Run a command inside the container."""
    return subprocess.run(["docker", "exec", CONTAINER] + cmd,
                          capture_output=True, text=True, timeout=timeout)


def _pty_exists() -> tuple[bool, str]:
    """Find a usable PTY path, prefer symlink CALYPSO_IRDA_PTY, fallback /dev/pts/3."""
    # Inside container
    if INSIDE:
        for p in (IRDA_PTY, "/dev/pts/3"):
            if Path(p).exists():
                return (True, p)
        return (False, "")
    # Host : check via docker exec
    for p in (IRDA_PTY, "/dev/pts/3"):
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
    """Un PTY IrDA est disponible (symlink ou /dev/pts/3 par fallback)."""
    ok, path = _pty_exists()
    assert ok, f"aucun PTY IrDA trouvé (cherché : {IRDA_PTY}, /dev/pts/3)"
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
            "fw-irda.log absent — Phase 3 (capture host) pas activée "
            "ou Phase 0.5 (cons_puts au boot) pas appliquée. "
            "Cf. PLAN_CLAUDE_CODE_20260516_IRDA_DEBUG_CHANNEL.md")
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
            "Canal silencieux — pas d'instrumentation fw active. "
            "Phase 5 (cons_puts/dbg_irda dans hot path) à appliquer."
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
        pytest.skip("fw-irda.log absent (Phase 3 non activée)")
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
    """irda_capture.py tourne si Phase 3 est appliquée."""
    pid_file = "/tmp/irda_capture.pid"
    r = _dexec(["test", "-f", pid_file])
    if r.returncode != 0:
        pytest.xfail("Phase 3 capture host pas activée (pas de /tmp/irda_capture.pid)")
    pid_r = _dexec(["cat", pid_file])
    try:
        pid = int(pid_r.stdout.strip())
    except ValueError:
        pytest.fail(f"contenu PID invalide : {pid_r.stdout!r}")
    alive = _dexec(["test", "-d", f"/proc/{pid}"])
    assert alive.returncode == 0, f"process irda_capture pid={pid} mort"


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
