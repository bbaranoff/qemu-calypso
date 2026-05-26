"""
test_firmware_state.py — vérifie que le firmware Calypso n'est pas figé,
que le dialog osmocon a progressé, et que le pipeline GSM enchaîne les
phases attendues.

Markers : `runtime_firmware`, `runtime_osmocon`.

Couvre :
  1. PC bouge entre 2 samples (= firmware exécute du code, pas figé)
  2. PC n'est pas dans la zone connue de busy-wait `calypso_sim_powerup+0x54`
  3. rxDoneFlag (auto-détecté via nm sur l'ELF) est non-zero ou re-poké
  4. osmocon a progressé au-delà du "starting download" (a vu Layer 1
     banner OU L1CTL_RESET_REQ — selon mode romload-passthrough ou
     -kernel direct)
  5. bridge.log a au moins 1 ligne `bridge: DL #` (proxy : osmo-bts-trx
     est connecté ET QEMU/bridge route les bursts)
"""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
INSIDE = os.path.exists("/.dockerenv")
MON_SOCK = "/tmp/qemu-calypso-mon.sock"
QEMU_LOG = "/root/qemu.log"
OSMOCON_LOG = "/tmp/osmocon.log"
(removed) = "/tmp/bridge.log"

# Zones de busy-wait connues côté firmware (à étendre quand on en trouve d'autres).
# Format : (addr_start, addr_end_inclusive) avec "près de calypso_sim_powerup+0x54".
KNOWN_BUSY_LOOPS = [
    (0x00822f70, 0x00822fc0),  # calypso_sim_powerup busy-wait sur rxDoneFlag
]

# Path firmware ELF pour résoudre rxDoneFlag via nm.
FW_ELF_CANDIDATES = [
    "/opt/GSM/firmware/board/compal_e88/layer1.highram.elf",
    os.environ.get("FW_ELF", ""),
]


def _hmp(cmd: str) -> str:
    """Envoie une commande HMP au monitor QEMU, retourne stdout brut."""
    if not Path(MON_SOCK).exists():
        return ""
    try:
        r = subprocess.run(
            ["socat", "-", f"UNIX-CONNECT:{MON_SOCK}"],
            input=cmd + "\n", capture_output=True, text=True, timeout=4)
        return r.stdout
    except Exception:
        return ""


def _read_pc() -> int | None:
    out = _hmp("info registers")
    m = re.search(r"R15=([0-9a-fA-F]+)", out)
    return int(m.group(1), 16) if m else None


def _resolve_symbol(name: str) -> int | None:
    """Résout un symbole firmware via nm. Cache silencieusement si nm absent."""
    nm = "/root/gnuarm/install/bin/arm-elf-nm"
    if not Path(nm).exists():
        return None
    for elf in [p for p in FW_ELF_CANDIDATES if p and Path(p).exists()]:
        try:
            r = subprocess.run([nm, "-n", elf], capture_output=True,
                               text=True, timeout=4)
            for line in r.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 3 and parts[2] == name:
                    return int(parts[0], 16)
        except Exception:
            continue
    return None


def _grep_count(path: str, pattern: str) -> int:
    try:
        with open(path, errors="replace") as f:
            return sum(1 for line in f if pattern in line)
    except FileNotFoundError:
        return 0


# -----------------------------------------------------------------------------
@pytest.mark.runtime_firmware
def test_qemu_monitor_responsive():
    """Le monitor QEMU répond à `info status` — preuve que QEMU n'est pas figé."""
    out = _hmp("info status")
    assert "VM status:" in out, f"monitor QEMU ne répond pas : {out[:200]!r}"


@pytest.mark.runtime_firmware
def test_pc_advances():
    """PC ARM change entre 2 samples espacés de 100ms — firmware exécute."""
    pc1 = _read_pc()
    if pc1 is None:
        pytest.skip("QEMU monitor pas dispo (PC unreadable)")
    time.sleep(0.1)
    pc2 = _read_pc()
    if pc2 is None:
        pytest.skip("PC unreadable 2nd sample")
    # On accepte un changement même petit ; si PC strictement égal sur 2
    # samples espacés de 100ms, c'est un busy-wait étroit (à investiguer).
    assert pc1 != pc2, (
        f"firmware figé : PC stable à 0x{pc1:08x} sur 100ms — "
        f"vérifier busy-wait (calypso_sim_powerup ?) et hack CALYPSO_FORCE_RX_DONE"
    )


@pytest.mark.runtime_firmware
def test_pc_not_in_known_busy_loop():
    """PC pas dans une zone de busy-wait connue (calypso_sim_powerup+0x54)."""
    pc = _read_pc()
    if pc is None:
        pytest.skip("monitor QEMU pas dispo")
    for lo, hi in KNOWN_BUSY_LOOPS:
        if lo <= pc <= hi:
            pytest.fail(
                f"PC=0x{pc:08x} dans zone busy-wait connue [0x{lo:08x},"
                f"0x{hi:08x}] — firmware bloqué sur SIM polling. "
                f"Vérifier CALYPSO_FORCE_RX_DONE=1 + CALYPSO_RXDONE_ADDR."
            )


@pytest.mark.runtime_firmware
def test_rxdoneflag_addr_resolvable():
    """L'adresse `rxDoneFlag` est résoluble via nm (= ELF firmware présent)."""
    addr = _resolve_symbol("rxDoneFlag")
    if addr is None:
        pytest.skip("nm/ELF non dispo — pas de vérification possible")
    # Vérifie cohérence avec CALYPSO_RXDONE_ADDR si défini
    env_addr_s = os.environ.get("CALYPSO_RXDONE_ADDR", "")
    if env_addr_s:
        env_addr = int(env_addr_s, 0)
        assert env_addr == addr, (
            f"mismatch : nm({addr:#x}) ≠ env CALYPSO_RXDONE_ADDR={env_addr_s} — "
            f"le hack écrit à la mauvaise adresse, le firmware bouclera"
        )
    print(f"\n[fw] rxDoneFlag @ 0x{addr:08x}")


# -----------------------------------------------------------------------------
@pytest.mark.runtime_osmocon
def test_osmocon_started_download():
    """osmocon a vu l'ident ack et lancé le download du firmware."""
    n = _grep_count(OSMOCON_LOG, "starting download")
    assert n >= 1, "osmocon n'a pas reçu l'ident ack du firmware"


@pytest.mark.runtime_osmocon
def test_osmocon_past_romload():
    """osmocon a dépassé la phase romload — soit Layer 1 banner (kernel mode
    rare), soit messages L1CTL côté sercomm (mode normal)."""
    has_l1ctl = _grep_count(OSMOCON_LOG, "L1CTL_") >= 1
    has_layer1 = _grep_count(OSMOCON_LOG, "OsmocomBB Layer 1") >= 1
    has_fb_sb = (_grep_count(OSMOCON_LOG, "=> SB") +
                 _grep_count(OSMOCON_LOG, "=>FB") +
                 _grep_count(OSMOCON_LOG, "Synchronize_TDMA")) >= 1
    assert has_l1ctl or has_layer1 or has_fb_sb, (
        "osmocon bloqué après 'starting download' — firmware n'a pas "
        "progressé vers sercomm. Probable busy-wait SIM. Vérifier "
        "CALYPSO_FORCE_RX_DONE=1 + CALYPSO_RXDONE_ADDR (run.sh auto-detect)."
    )


@pytest.mark.runtime_osmocon
def test_osmocon_no_recent_lost_spam():
    """LOST count < 10000/run — au-delà = sercomm framing complètement cassé.

    Note : quelques milliers de LOST sont normaux pendant le boot romload
    passthrough (osmocon synchronise sa fenêtre HDLC sur le flux firmware).
    Seuil tunable via env CALYPSO_OSMOCON_LOST_MAX.
    """
    threshold = int(os.environ.get("CALYPSO_OSMOCON_LOST_MAX", "10000"))
    n = _grep_count(OSMOCON_LOG, "LOST ")
    assert n < threshold, (
        f"{n}× 'LOST' dans osmocon.log (seuil {threshold}) — sercomm "
        f"désynchronisé (QEMU UART_MODEM corruption ou firmware crash silencieux)"
    )


# -----------------------------------------------------------------------------
@pytest.mark.runtime_firmware
def test_bridge_has_dl_bursts():
    """bridge.log a au moins quelques DL bursts — preuve osmo-bts-trx connecté."""
    n = _grep_count((removed), "bridge: DL #")
    assert n >= 10, (
        f"{n} DL bursts dans bridge.log — osmo-bts-trx pas connecté ou "
        f"bridge ne route pas (vérifier BSP UDP 6702 listening)"
    )
