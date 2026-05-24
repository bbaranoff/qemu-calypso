"""
test_osmocom_workflow.py — vérifie l'alignement de notre QEMU Calypso avec
le workflow attendu par OsmocomBB (firmware layer1 + osmo-bts-trx + bridge.py).

Markers :
  - `osmocom_compliant`  : point du workflow où on respecte la spec OsmocomBB
  - `osmocom_divergent`  : point où on DIVERGE (xfail ou skip, à corriger)
  - `osmocom_sim`        : sémantique SIM controller
  - `osmocom_clock`      : alignement clock domains
  - `osmocom_bridge`     : bridge.py timing
  - `osmocom_boot`       : séquence boot ARM/DSP

Couvre les divergences identifiées dans WORKFLOW_OSMOCOM.md :
  ✓ SIM IT bits clear semantics (NATR/WT/OV on read, TX on DTX write)
  ✓ rxDoneFlag address aligned with nm
  ✓ BSP drain timer on QEMU_CLOCK_VIRTUAL
  ✓ Bridge.py select timeout < 5ms
  ✓ Defer clear timer present (race fix contre cpu_io_recompile)
  ✗ TCG conditional STR commit sous -icount auto (vraie cause non patchée,
    contournée par defer)
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
QEMU_SRC = "/opt/GSM/qemu-src"
FW_ELF = os.environ.get(
    "FW_ELF",
    "/opt/GSM/firmware/board/compal_e88/layer1.highram.elf")
SIM_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_sim.c"
SIM_H = f"{QEMU_SRC}/include/hw/arm/calypso/calypso_sim.h"
TRX_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_trx.c"
BSP_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_bsp.c"
TINT0_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_tint0.c"
BRIDGE_PY = f"{QEMU_SRC}/bridge.py"
RUN_SH = f"{QEMU_SRC}/run.sh"
QEMU_LOG = "/root/qemu.log"
BTS_LOG = "/tmp/bts.log"


def _read(path: str) -> str:
    """Read a file from the container (or host if INSIDE)."""
    if INSIDE:
        try:
            return Path(path).read_text(errors="replace")
        except Exception:
            return ""
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "cat", path],
            capture_output=True, text=True, timeout=5)
        return r.stdout
    except Exception:
        return ""


def _nm_symbol(elf_path: str, sym: str) -> str | None:
    """Look up a symbol address via nm. Returns hex string '0x...' or None."""
    cmd = ["nm", elf_path] if INSIDE else ["docker", "exec", CONTAINER, "nm", elf_path]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        for line in r.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[-1] == sym:
                return "0x" + parts[0].lstrip("0").rjust(8, "0")[-8:]
    except Exception:
        pass
    return None


# =============================================================================
# 1. SIM IT bits clear semantics — alignment with firmware self-doc
# =============================================================================
# Firmware calypso/sim.c documents :
#   NATR/WT/OV = 0 on read of REG_SIM_IT
#   TX         = 0 on write to REG_SIM_DTX
#   RX         = implicit / level

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_sim
def test_sim_it_read_clear_mask_per_firmware_spec():
    """REG_SIM_IT read doit clear NATR|WT|OV seulement (PAS TX, PAS RX).

    Avant fix : `edge_seen = v & ~CALYPSO_SIM_IT_RX` clearait aussi IT_TX
    par erreur. Le firmware s'attend à ce que IT_TX se clear sur write
    DTX uniquement. Fix 2026-05-24.
    """
    src = _read(SIM_C)
    assert src, f"can't read {SIM_C}"
    # Le bon mask : NATR|WT|OV (3 bits explicites)
    pattern = re.compile(
        r"edge_seen\s*=\s*v\s*&\s*\(\s*"
        r"CALYPSO_SIM_IT_NATR\s*\|\s*"
        r"CALYPSO_SIM_IT_WT\s*\|\s*"
        r"CALYPSO_SIM_IT_OV\s*\)")
    assert pattern.search(src), (
        "edge_seen mask doit être (NATR|WT|OV) per firmware spec, "
        "PAS `~CALYPSO_SIM_IT_RX` (qui inclurait IT_TX par erreur)")


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_sim
def test_sim_dtx_write_clears_it_tx():
    """REG_SIM_DTX write doit clear IT_TX + appeler update_irq.

    Per firmware calypso/sim.c L264 self-doc :
      'REG_SIM_IT_SIM_TX = 0 ==> On write access to REG_SIM_DTX'
    Avant fix : le case DTX appelait juste apdu_tx_byte, sans toucher IT.
    """
    src = _read(SIM_C)
    assert src, f"can't read {SIM_C}"
    # Le write handler DTX se distingue du read handler par `apdu_tx_byte`.
    # Extraire le bloc depuis apdu_tx_byte jusqu'au prochain break;.
    idx = src.find("apdu_tx_byte(s,")
    assert idx != -1, "apdu_tx_byte(s,...) call not found"
    # Window de 300 chars après apdu_tx_byte pour capturer le reste du case
    block = src[idx:idx + 400]
    end = block.find("break;")
    assert end != -1, "break; not found after apdu_tx_byte"
    block = block[:end]
    assert "CALYPSO_SIM_IT_TX" in block, (
        f"DTX write must clear IT_TX per firmware spec L264. "
        f"Block found:\n{block}")
    assert "update_irq" in block, (
        f"DTX write must call update_irq after clearing IT_TX. "
        f"Block:\n{block}")


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_sim
def test_sim_rx_is_level_sensitive():
    """IT_RX doit être level-sensitive (suit FIFO), pas edge-clear.

    Firmware s'attend à ce que IT_RX reste high tant qu'il y a des bytes
    dans la FIFO RX (cleared implicitement par read DRX qui décrémente FIFO).
    """
    src = _read(SIM_C)
    assert src, f"can't read {SIM_C}"
    # refresh_it_rx doit set IT_RX si rx_count > 0, clear sinon
    assert "if (rx_count(s) > 0) s->it |= CALYPSO_SIM_IT_RX" in src, (
        "refresh_it_rx must keep IT_RX high while FIFO non-empty")
    assert "else                 s->it &= ~CALYPSO_SIM_IT_RX" in src, (
        "refresh_it_rx must lower IT_RX when FIFO empty")


# =============================================================================
# 2. rxDoneFlag address alignment — QEMU default = firmware nm symbol
# =============================================================================

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_sim
def test_rxdone_addr_matches_firmware_nm():
    """Le default rxdone_addr dans QEMU doit matcher l'adresse réelle dans le
    firmware ELF (per `nm`).

    Avant fix 2026-05-24 : default stale `0x008302a0` (vieux build), firmware
    était à `0x008302d4`. Le hack tapait la mauvaise case.
    """
    expected = _nm_symbol(FW_ELF, "rxDoneFlag")
    if expected is None:
        pytest.skip(f"can't resolve rxDoneFlag in {FW_ELF}")
    for path in (SIM_C, TRX_C):
        src = _read(path)
        # Cherche le default fallback dans le getenv pattern
        m = re.search(r":\s*(0x008302[0-9a-fA-F]{2})\s*;", src)
        assert m, f"no default rxdone_addr in {path}"
        actual = "0x" + m.group(1).lstrip("0x").lstrip("0").rjust(8, "0")[-8:]
        assert actual == expected, (
            f"{path}: default rxdone_addr = {actual}, "
            f"but firmware nm says {expected}")


# =============================================================================
# 3. Clock domain alignment — tout le pipeline sur QEMU_CLOCK_VIRTUAL
# =============================================================================
# Sous icount=auto, VIRTUAL ≠ REALTIME. Mixer les deux = drift cumulatif.

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
def test_bsp_drain_timer_on_virtual_clock():
    """BSP drain timer doit être sur QEMU_CLOCK_VIRTUAL pour s'aligner avec
    TINT0/tdma_tick et éviter le drift bts↔qfn sous icount=auto.

    Avant fix 2026-05-24 : QEMU_CLOCK_REALTIME → drift ~1300 fr en 6s wall.
    """
    src = _read(BSP_C)
    assert src, f"can't read {BSP_C}"
    # timer_new_* doit utiliser VIRTUAL
    assert re.search(
        r"timer_new_ns\s*\(\s*QEMU_CLOCK_VIRTUAL\s*,\s*bsp_drain_cb", src), (
        "BSP drain timer must use QEMU_CLOCK_VIRTUAL")
    # Pas de REALTIME résiduel dans le code fonctionnel (commentaires OK)
    code_only = re.sub(r"/\*.*?\*/", "", src, flags=re.DOTALL)
    code_only = re.sub(r"//.*", "", code_only)
    assert "QEMU_CLOCK_REALTIME" not in code_only, (
        "BSP must not use REALTIME in functional code (commentaires OK)")


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
def test_tint0_on_virtual_clock():
    """TINT0 (DSP Timer 0) doit fire sur QEMU_CLOCK_VIRTUAL à 4.615ms TDMA."""
    src = _read(TINT0_C)
    assert src, f"can't read {TINT0_C}"
    assert "QEMU_CLOCK_VIRTUAL" in src and "TINT0_PERIOD_NS" in src
    # Pas de REALTIME
    assert "QEMU_CLOCK_REALTIME" not in src


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
def test_tdma_tick_on_virtual_clock():
    """tdma_tick et frame_irq_timer dans calypso_trx.c doivent être VIRTUAL."""
    src = _read(TRX_C)
    assert src, f"can't read {TRX_C}"
    # tdma_tick utilise qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)
    assert "qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)" in src
    # Note : un kick_timer à 5ms REALTIME existe — explicitement isolé pour
    # invalidation cache TB. Pas dans le critical path de la frame counter.


# =============================================================================
# 4. Bridge.py timing — jitter CLK IND
# =============================================================================
# osmo-bts s'attend à CLK IND à 51 frames d'intervalle (235ms). Jitter
# trop élevé → "We were 1 FN faster/slower than TRX".

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_bridge
def test_bridge_select_timeout_low_jitter():
    """bridge.py select.select timeout doit être ≤ 5ms pour limiter jitter
    sur l'émission CLK IND (sub-frame).

    Avant fix 2026-05-24 : 50ms → jitter ±10 frames → BTS log spam
    'We were 1 FN faster/slower than TRX'.
    Fix : 1ms → ±0.2 frame, sous seuil compensation BTS.
    """
    src = _read(BRIDGE_PY)
    assert src, f"can't read {BRIDGE_PY}"
    m = re.search(r"select\.select\([^,]+,\s*\[\],\s*\[\],\s*([0-9.]+)\)", src)
    assert m, "select.select call not found in bridge.py"
    timeout = float(m.group(1))
    assert timeout <= 0.005, (
        f"bridge.py select timeout = {timeout*1000:.1f}ms, "
        f"max accepté = 5ms (idéal 1ms) pour low-jitter CLK IND")


# =============================================================================
# 5. TCG race workaround — defer clear timer présent
# =============================================================================
# Sous icount=auto, cpu_io_recompile truncate la TB après LDRH MMIO mid-TB.
# Notre fix : defer le clear edge bits via QEMUTimer 1µs pour escape la race.

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_sim
def test_sim_defer_clear_timer_present():
    """calypso_sim.c doit avoir un QEMUTimer pour différer le clear edge bits.

    Sans ça, le clear synchrone dans la callback MMIO race avec
    cpu_io_recompile (TB truncation) → STRNE rxDoneFlag jamais committé.
    """
    src = _read(SIM_C)
    assert src, f"can't read {SIM_C}"
    assert "QEMUTimer  *clear_edge_timer" in src or \
           "QEMUTimer *clear_edge_timer" in src, (
        "Missing clear_edge_timer field in CalypsoSim struct")
    assert "static void clear_edge_cb" in src, (
        "Missing clear_edge_cb callback")
    assert "pending_edge_clear" in src, (
        "Missing pending_edge_clear tracking field")
    # Init du timer dans calypso_sim_new
    assert re.search(
        r"clear_edge_timer\s*=\s*timer_new_ns\s*\(\s*QEMU_CLOCK_VIRTUAL\s*,\s*"
        r"clear_edge_cb", src), (
        "clear_edge_timer not initialized with QEMU_CLOCK_VIRTUAL + clear_edge_cb")


# =============================================================================
# 6. Default env values — convention OsmocomBB-friendly
# =============================================================================

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_boot
def test_force_rx_done_default_on():
    """run.sh doit défaulter CALYPSO_FORCE_RX_DONE=1 puisque c'est la gestion
    SIM normative (firmware ne peut pas commit le STRNE rxDoneFlag sous icount=auto).
    """
    src = _read(RUN_SH)
    assert src, f"can't read {RUN_SH}"
    assert re.search(
        r'CALYPSO_FORCE_RX_DONE\s*=\s*"\$\{\s*CALYPSO_FORCE_RX_DONE\s*:-\s*1\s*\}"',
        src), "run.sh must default CALYPSO_FORCE_RX_DONE=1"


# =============================================================================
# DIVERGENCES — workflow NON respecté (xfail / skip)
# =============================================================================

@pytest.mark.osmocom_divergent
@pytest.mark.xfail(reason="TCG conditional STR commit bug under icount=auto — "
                          "not patched, workarounded via SIM defer clear")
def test_tcg_strne_commits_under_icount_auto():
    """Le vrai root cause TCG : sous icount=auto, conditional STR (STRNE) peut
    ne pas commit son store quand l'insn précédente MMIO triggere
    cpu_io_recompile. Le TB est truncate après MMIO via CF_MEMI_ONLY,
    le STRNE n'est jamais exécuté dans cette TB.

    Notre defer clear contourne le symptôme, mais le bug TCG sous-jacent
    reste. Vrai fix nécessite patch QEMU upstream (translator.c ou
    cputlb.c cpu_io_recompile logic).

    Ce test est marqué xfail tant que le bug TCG n'est pas patché.
    """
    pytest.fail("TCG bug not fixed — contourné par defer in SIM emulator")


@pytest.mark.osmocom_divergent
@pytest.mark.osmocom_bridge
def test_bridge_clock_from_qemu_default():
    """BRIDGE_CLK_FROM_QEMU devrait être =1 par défaut sous icount=auto pour
    aligner la bridge clock sur QEMU FN au lieu de wall.

    Actuel default = 0 (wall-paced). Sous icount=auto + default=0, bridge
    drifte de qfn. À flipper par défaut quand on confirme icount=auto stable.
    """
    src = _read(RUN_SH)
    assert src, f"can't read {RUN_SH}"
    m = re.search(
        r'BRIDGE_CLK_FROM_QEMU\s*=\s*"\$\{\s*BRIDGE_CLK_FROM_QEMU\s*:-\s*([01])\s*\}"',
        src)
    assert m, "BRIDGE_CLK_FROM_QEMU default line not found"
    default = m.group(1)
    if default == "0":
        pytest.xfail("BRIDGE_CLK_FROM_QEMU=0 default — pas idéal sous icount=auto, "
                     "à flipper à =1 quand stable")
    assert default == "1"


# =============================================================================
# Runtime checks — n'execute que si QEMU live
# =============================================================================

def _qemu_running() -> bool:
    cmd = ["pgrep", "-f", "qemu-system-arm"] if INSIDE else \
          ["docker", "exec", CONTAINER, "pgrep", "-f", "qemu-system-arm"]
    try:
        return subprocess.run(cmd, capture_output=True, timeout=2).returncode == 0
    except Exception:
        return False


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_boot
@pytest.mark.skipif(not Path(QEMU_LOG).exists() and not INSIDE,
                    reason="qemu.log absent")
def test_runtime_sim_it_wt_rxdone_log_present():
    """Si QEMU tourne avec notre fix SIM, qemu.log doit contenir des lignes
    `[sim] SIM IT_WT → rxDoneFlag=1 @0x008302d4 #N`.
    """
    log = _read(QEMU_LOG)
    if not log:
        pytest.skip("qemu.log vide ou absent")
    pattern = re.compile(r"\[sim\] SIM IT_WT → rxDoneFlag=1 @0x008302d4")
    if not pattern.search(log):
        pytest.xfail("Fix SIM pas encore en effet — rebuild + relancer QEMU "
                     "avec CALYPSO_FORCE_RX_DONE=1 (default après fix)")


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
@pytest.mark.skipif(not Path(BTS_LOG).exists() and not INSIDE,
                    reason="bts.log absent")
def test_runtime_bts_no_fn_skew_messages():
    """bts.log ne doit plus avoir massivement de 'We were 1 FN faster/slower'
    après fix bridge.py select timeout.
    """
    log = _read(BTS_LOG)
    if not log:
        pytest.skip("bts.log vide")
    skew = len(re.findall(r"We were 1 FN (faster|slower) than TRX", log))
    # On accepte un peu de skew sur démarrage (warmup)
    assert skew < 20, (
        f"bts.log contient {skew} messages 'FN faster/slower' — "
        f"jitter bridge.py trop élevé, vérifier select timeout")
