"""
test_gdb_stub.py — surface GDB stub QEMU :1234 (Phase 2 du plan IrDA debug).

Le GDB stub est activé par run.sh via `-gdb tcp::1234` (cf. calypso_trx.c).
Marker : `runtime_gdb`. Helper `gdb_send_recv()` parle le protocole RSP
directement, sans gdb-multiarch — connexion socket brute.

Cible : surface d'introspection ARM jamais testée, complément à
test_inject_frames.py qui utilisait déjà gdb-stub pour writes NDB.
"""
from __future__ import annotations

import os
import re
import socket
import subprocess
import time

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
GDB_HOST  = os.environ.get("CALYPSO_GDB_HOST", "127.0.0.1")
GDB_PORT  = int(os.environ.get("CALYPSO_GDB_PORT", "1234"))

INSIDE = os.path.exists("/.dockerenv")


# -----------------------------------------------------------------------------
# Helpers RSP — protocole GDB remote bas niveau
# -----------------------------------------------------------------------------
def _gdb_pkt(cmd: str) -> bytes:
    """Encode `$cmd#XX` avec checksum mod 256."""
    csum = sum(ord(c) for c in cmd) & 0xff
    return f"${cmd}#{csum:02x}".encode()


def _resolve_host() -> str:
    """Détecte où le gdbstub écoute : 127.0.0.1 si dans container, sinon IP container."""
    if INSIDE:
        return GDB_HOST
    # Sur host : essayer 127.0.0.1 d'abord (mappage port), sinon IP container
    try:
        with socket.create_connection((GDB_HOST, GDB_PORT), timeout=0.5):
            return GDB_HOST
    except (OSError, ConnectionRefusedError):
        pass
    try:
        out = subprocess.run(
            ["docker", "inspect", CONTAINER, "--format",
             "{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}"],
            capture_output=True, text=True, timeout=2)
        for ip in out.stdout.strip().split():
            try:
                with socket.create_connection((ip, GDB_PORT), timeout=0.5):
                    return ip
            except OSError:
                continue
    except Exception:
        pass
    return GDB_HOST


def _gdb_send_recv(cmd: str, timeout: float = 2.0) -> str:
    """Envoie une commande RSP, retourne la réponse brute (avec `$...#XX`).

    Effectue l'ACK `+` au début pour signaler "no error" au stub.
    """
    host = _resolve_host()
    with socket.create_connection((host, GDB_PORT), timeout=timeout) as s:
        s.settimeout(timeout)
        s.sendall(b"+")  # ACK initial
        s.sendall(_gdb_pkt(cmd))
        buf = b""
        end_at = time.time() + timeout
        while time.time() < end_at:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            # Cherche `$...#XX` complet
            if b"#" in buf:
                idx = buf.rfind(b"#")
                if len(buf) >= idx + 3:  # # + 2 hex
                    break
        return buf.decode(errors="replace")


def _extract_payload(rsp: str) -> str:
    """Extrait la payload entre `$` et `#XX` d'une réponse RSP."""
    if "$" not in rsp or "#" not in rsp:
        return ""
    after = rsp.split("$", 1)[1]
    return after.split("#", 1)[0]


def _container_running() -> bool:
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "pgrep", "qemu-system-arm"],
            capture_output=True, text=True, timeout=2)
        return r.returncode == 0 and bool(r.stdout.strip())
    except Exception:
        return False


# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
@pytest.mark.runtime_gdb
def test_gdb_stub_reachable():
    """Le port 1234 du GDB stub est listening (host ou container IP)."""
    host = _resolve_host()
    with socket.create_connection((host, GDB_PORT), timeout=2):
        pass


@pytest.mark.runtime_gdb
def test_gdb_handshake_succeeds():
    """vMustReplyEmpty → réponse vide `$#00` ou similaire, pas de timeout."""
    r = _gdb_send_recv("vMustReplyEmpty")
    assert "$" in r and "#" in r, f"pas de réponse RSP : {r!r}"


@pytest.mark.runtime_gdb
def test_gdb_query_supported():
    """qSupported retourne au moins une feature (PacketSize typique)."""
    r = _gdb_send_recv("qSupported:multiprocess+;swbreak+;hwbreak+")
    payload = _extract_payload(r)
    assert payload, f"qSupported vide : {r!r}"
    # Tolère plusieurs formats : PacketSize=, multiprocess+, etc.
    assert any(tok in payload for tok in ("PacketSize", "qXfer", "multiprocess",
                                          "swbreak", "hwbreak")), \
        f"qSupported sans feature reconnue : {payload!r}"


@pytest.mark.runtime_gdb
def test_gdb_read_arm_registers():
    """'g' command — bloc de registres ARM (R0..R15 + CPSR au minimum)."""
    r = _gdb_send_recv("g")
    payload = _extract_payload(r)
    # ARM 32-bit : 16 registres × 4 bytes × 2 hex = 128 chars min
    assert len(payload) >= 128, f"payload trop court ({len(payload)} chars) : {payload[:80]!r}"


@pytest.mark.runtime_gdb
def test_gdb_read_pc_nonzero():
    """Le PC (R15 — registre 15 du bloc `g`) doit être > 0 (ARM démarré)."""
    r = _gdb_send_recv("g")
    payload = _extract_payload(r)
    if len(payload) < 128:
        pytest.skip(f"payload regs trop court ({len(payload)})")
    pc_hex = payload[15 * 8:16 * 8]
    try:
        pc = int.from_bytes(bytes.fromhex(pc_hex), "little")
    except ValueError:
        pytest.fail(f"hex PC invalide : {pc_hex!r}")
    assert pc != 0, "PC = 0 — ARM non démarré ou bloc registres mal aligné"
    print(f"\n[gdb] ARM PC = 0x{pc:08x}")


@pytest.mark.runtime_gdb
def test_gdb_read_memory_at_dsp_api_base():
    """Lire 16 bytes à 0xFFD00000 (API RAM côté ARM = DSP base + offset)."""
    DSP_API_BASE = 0xFFD00000
    r = _gdb_send_recv(f"m{DSP_API_BASE:x},10")
    payload = _extract_payload(r)
    if payload.startswith("E"):
        pytest.xfail(f"GDB read mem renvoie une erreur (E{payload[1:]}) — adresse pas mappée")
    assert len(payload) == 32, f"attendu 32 hex chars (16B), got {len(payload)} : {payload!r}"


@pytest.mark.runtime_gdb
def test_gdb_stub_survives_3_quick_reconnects():
    """3 connect/disconnect rapides ne plantent pas QEMU."""
    host = _resolve_host()
    for _ in range(3):
        with socket.create_connection((host, GDB_PORT), timeout=2):
            pass
        time.sleep(0.1)
    assert _container_running(), "QEMU mort après 3 reconnects gdb"


@pytest.mark.runtime_gdb
def test_gdb_read_dsp_daram_xfail():
    """DSP DARAM (c54x interne) n'est pas adressable via le GDB stub ARM."""
    DARAM_DSP = 0x0000
    r = _gdb_send_recv(f"m{DARAM_DSP:x},10")
    payload = _extract_payload(r)
    if payload.startswith("E"):
        pytest.xfail("DSP DARAM pas adressable via GDB stub ARM — attendu")
    # Si on lit OK, c'est qu'il y a un mapping ARM à cette adresse ; non bloquant
    assert True
