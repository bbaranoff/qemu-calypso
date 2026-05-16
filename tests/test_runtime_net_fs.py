"""
test_runtime_net_fs.py — surfaces réseau et FS du run (Phase 2 plan IrDA).

Markers : `runtime_net`, `runtime_fs`. Tests anti-régression sur des aspects
opérationnels du long-running (leak FDs, log size cap, ports inattendus,
disk container).
"""
from __future__ import annotations

import os
import subprocess

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
QEMU_LOG_CT = os.environ.get("CALYPSO_QEMU_LOG", "/root/qemu.log")
INSIDE = os.path.exists("/.dockerenv")


def _dexec_sh(cmd: str, timeout: float = 4.0) -> subprocess.CompletedProcess:
    """Exec un shell command à l'intérieur du container."""
    if INSIDE:
        return subprocess.run(["bash", "-c", cmd],
                              capture_output=True, text=True, timeout=timeout)
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-c", cmd],
                          capture_output=True, text=True, timeout=timeout)


# -----------------------------------------------------------------------------
@pytest.mark.runtime_net
def test_qemu_ports_listening():
    """Ports attendus listening : 1234 (gdb stub) + 4247 (mobile VTY)."""
    r = _dexec_sh("ss -tln 2>/dev/null || netstat -tln 2>/dev/null")
    out = r.stdout
    missing = []
    for port in ("1234", "4247"):
        if f":{port} " not in out and f":{port}\n" not in out:
            missing.append(port)
    assert not missing, f"ports manquants : {missing} — out : {out[:300]!r}"


@pytest.mark.runtime_net
def test_no_unexpected_high_ports():
    """Aucun listener sur port > 60000 (régression leak)."""
    r = _dexec_sh("ss -tln 2>/dev/null | awk 'NR>1 {print $4}' | grep -oE ':[0-9]+$' | sort -u")
    high = []
    for line in r.stdout.split():
        if line.startswith(":"):
            try:
                p = int(line[1:])
                if p > 60000:
                    high.append(p)
            except ValueError:
                continue
    assert not high, f"ports inattendus (>60000) : {high}"


@pytest.mark.runtime_fs
def test_qemu_fd_usage_below_limit():
    """qemu-system-arm doit avoir moins de 1024 fd (early warning leak)."""
    r = _dexec_sh("pid=$(pgrep qemu-system-arm | head -1); [ -n \"$pid\" ] && ls /proc/$pid/fd 2>/dev/null | wc -l")
    s = r.stdout.strip()
    if not s or s == "0":
        pytest.skip("qemu-system-arm pas trouvé")
    n_fds = int(s)
    assert n_fds < 1024, f"fd leak probable : {n_fds} fds (limite 1024)"
    print(f"\n[fs] qemu fd count = {n_fds}")


@pytest.mark.runtime_fs
def test_qemu_log_disk_size_under_2gb():
    """/root/qemu.log < 2 GB (anti-leak log)."""
    r = _dexec_sh(f"stat -c%s {QEMU_LOG_CT} 2>/dev/null")
    s = r.stdout.strip()
    if not s:
        pytest.skip(f"{QEMU_LOG_CT} absent")
    size_bytes = int(s)
    assert size_bytes < 2 * 1024**3, \
        f"qemu.log trop gros : {size_bytes / 1024**2:.0f} MB (limite 2 GB)"
    print(f"\n[fs] qemu.log = {size_bytes / 1024**2:.1f} MB")


@pytest.mark.runtime_fs
def test_container_disk_space_above_min():
    """Le filesystem `/` du container a au moins 500 MB libre."""
    r = _dexec_sh("df / | tail -1 | awk '{print $4}'")
    s = r.stdout.strip()
    if not s:
        pytest.skip("df pas dispo")
    free_kb = int(s)
    assert free_kb > 500_000, \
        f"disque container trop plein : {free_kb // 1024} MB libre (min 500 MB)"
    print(f"\n[fs] / free = {free_kb // 1024} MB")
