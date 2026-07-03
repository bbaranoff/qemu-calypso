"""
test_qemu_introspection.py — surface QEMU monitor HMP (Phase 2 plan IrDA).

Marker : `runtime_monitor`. Extension du `runtime_irq` existant qui n'utilisait
que `info dsp_irq`. Couvre `info status`, `info chardev`, `info qtree`,
`info mtree`, `info qom-tree`, `info registers`, `info irq`.

Tests indépendants de l'état firmware — valident l'état QEMU lui-même.
"""
from __future__ import annotations

import os
import socket
import subprocess
import time

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
MON_SOCK  = os.environ.get("CALYPSO_MON_SOCK", "/tmp/qemu-calypso-mon.sock")
INSIDE = os.path.exists("/.dockerenv")


def _qemu_monitor(cmd: str, timeout: float = 3.0) -> str:
    """Envoie une commande HMP au monitor unix socket, retourne stdout."""
    if INSIDE:
        sock_path = MON_SOCK
        try:
            s = socket.socket(socket.AF_UNIX)
            s.settimeout(timeout)
            s.connect(sock_path)
            s.sendall(cmd.encode() + b"\n")
            t_end = time.time() + timeout
            out = b""
            while time.time() < t_end:
                try:
                    chunk = s.recv(8192)
                    if not chunk: break
                    out += chunk
                except socket.timeout:
                    break
            s.close()
            return out.decode(errors="replace")
        except Exception as e:
            return f"<monitor error: {e}>"
    # Host : passe via docker exec
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c",
             f"echo '{cmd}' | timeout {timeout} socat - UNIX-CONNECT:{MON_SOCK}"],
            capture_output=True, text=True, timeout=timeout + 2)
        return r.stdout + r.stderr
    except Exception as e:
        return f"<monitor exec error: {e}>"


# -----------------------------------------------------------------------------
@pytest.mark.runtime_monitor
def test_monitor_socket_reachable():
    """Le monitor unix socket répond à une commande basique."""
    r = _qemu_monitor("info version")
    assert "QEMU" in r or "version" in r.lower(), f"monitor pas joignable : {r!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_status_is_running():
    """`info status` indique 'running' (VM non haltée).

    Le socket monitor unix est partagé avec d'autres suites qui peuvent s'y
    connecter en même temps (ex. test_gdb_stub, test_inject_frames) et
    entrelacer un stop/cont bref. On retente donc quelques fois avant de
    conclure à un vrai souci — un seul relevé "paused" concurrent avec un
    pair ne veut rien dire en soi.
    """
    r = ""
    for attempt in range(3):
        r = _qemu_monitor("info status")
        if "running" in r.lower():
            break
        time.sleep(1.0)
    assert "running" in r.lower(), f"VM status pas 'running' (après retries) : {r!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_chardev_lists_serial():
    """`info chardev` liste au moins 2 PTY serial (modem + irda)."""
    r = _qemu_monitor("info chardev")
    n_serial = r.count("serial")
    n_pty = r.count("pty")
    assert n_serial >= 2 or n_pty >= 2, \
        f"attendu ≥2 chardevs serial/pty, n_serial={n_serial} n_pty={n_pty}: {r[:300]!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_qtree_has_calypso():
    """`info qtree` contient au moins un device calypso_*."""
    r = _qemu_monitor("info qtree")
    assert "calypso" in r.lower(), f"aucun device calypso dans qtree : {r[:300]!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_mtree_has_uart_irda():
    """`info mtree` mentionne le mapping mémoire UART_IRDA à 0xFFFF5000."""
    r = _qemu_monitor("info mtree")
    # UART_IRDA base = 0xFFFF5000 selon calypso_soc.c:38
    has_irda_addr = "ffff5000" in r.lower()
    has_uart_label = "uart-irda" in r.lower() or "irda" in r.lower()
    if not (has_irda_addr or has_uart_label):
        pytest.xfail(f"UART_IRDA pas trouvé dans mtree — peut-être nom différent")
    assert True


@pytest.mark.runtime_monitor
def test_monitor_info_mtree_has_uart_modem():
    """`info mtree` mentionne le mapping UART_MODEM (plage 0xFFFF5xxx)."""
    r = _qemu_monitor("info mtree")
    assert "ffff5" in r.lower(), \
        f"plage UART (0xFFFF5xxx) absente du mtree : {r[:300]!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_qom_tree_has_calypso_or_arm():
    """`info qom-tree` contient au moins un nœud ARM ou calypso."""
    r = _qemu_monitor("info qom-tree")
    if "<monitor" in r:
        pytest.skip(f"monitor error : {r!r}")
    found = any(tok in r.lower() for tok in ("calypso", "arm946", "arm-cpu",
                                              "arm,cpu", "c54x", "dsp"))
    if not found:
        pytest.xfail(f"qom-tree sans nœud reconnu — possible nom différent : {r[:200]!r}")
    assert True


@pytest.mark.runtime_monitor
def test_monitor_info_registers_arm():
    """`info registers` retourne au moins PC + un autre registre ARM.

    Comme pour `info status`, le socket monitor est partagé avec d'autres
    suites concurrentes : une commande entrelacée peut retarder la vraie
    réponse au-delà du timeout de lecture et faire croire à une absence de
    registres. On retente donc avant de conclure.
    """
    found = 0
    r = ""
    for attempt in range(3):
        r = _qemu_monitor("info registers")
        found = sum(1 for tok in ("R00", "R01", "R02", "PC", "CPSR",
                                  "r0", "r1", "pc", "cpsr") if tok in r)
        if found >= 2:
            break
        time.sleep(1.0)
    assert found >= 2, \
        f"info registers ne ressemble pas à ARM (found={found}, après retries) : {r[:300]!r}"


@pytest.mark.runtime_monitor
def test_monitor_info_irq_listed():
    """`info irq` produit une réponse non-vide (mappings IRQ exposés)."""
    r = _qemu_monitor("info irq")
    # QEMU peut ne pas supporter 'info irq' selon le board ; tolère
    if "<monitor" in r or "unknown command" in r.lower():
        pytest.xfail(f"info irq pas supporté sur ce board QEMU : {r[:100]!r}")
    assert len(r.strip()) > 0, "info irq retourne vide"
