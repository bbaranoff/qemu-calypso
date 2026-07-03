"""
test_osmocom_workflow.py — vérifie l'alignement de notre QEMU Calypso avec
le workflow attendu par OsmocomBB (firmware layer1 + osmo-bts-trx + calypso-ipc-device).

Markers :
  - `osmocom_compliant`  : point du workflow où on respecte la spec OsmocomBB
  - `osmocom_divergent`  : point où on DIVERGE (xfail ou skip, à corriger)
  - `osmocom_sim`        : sémantique SIM controller
  - `osmocom_clock`      : alignement clock domains
  - `osmocom_bridge`     : calypso-ipc-device timing
  - `osmocom_boot`       : séquence boot ARM/DSP

Couvre les divergences identifiées dans WORKFLOW_OSMOCOM.md (historique) :
  ✓ SIM IT bits clear semantics (NATR/WT/OV on read, TX on DTX write)
  ✓ BSP drain timer REALTIME + monotonic anti-drift rearm (2026-05-29/30 :
    VIRTUAL droppait 95% des bursts sous charge DSP lourde, revert
    volontaire et documenté dans calypso_bsp.c)
  ✗ Bridge clock verrouillé sur QEMU FN (CALYPSO_QFN_FORCE) reste =0 par
    défaut — une tentative de spin-wait qfn-pacé a starve l'IPC osmo-trx
    (crash), le pacing wall clock_nanosleep(ABSTIME) reste le défaut stable

Retirés 2026-07-03 (cleanup pytest) — code/feature structurellement
supprimé du produit, voir commentaires source datés pour la trace :
  - rxDoneFlag address injection (default `rxdone_addr` codé en dur) :
    supprimé, le firmware gère rxDoneFlag lui-même via le read-to-clear
    normal de SIM_IT (plus besoin d'injection d'adresse par QEMU)
  - SIM defer-clear timer (clear_edge_timer/clear_edge_cb/pending_edge_clear) :
    supprimé 2026-05-27, la race cpu_io_recompile qu'il contournait s'est
    avérée INEXISTANTE (probes empiriques, cf calypso_sim.c)
  - CALYPSO_FORCE_RX_DONE : hack env-gated retiré du code source 2026-05-26
    (politique no-hack, CLAUDE.md règle #1 — cf bash_scripts/run.sh)
  - calypso-ipc-device `select.select()` timeout : le bridge a été réécrit
    en C (tools/calypso-ipc-device/), pacing par
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) — aucun code Python
    select.select ne subsiste pour matcher
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
SIM_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_sim.c"
SIM_H = f"{QEMU_SRC}/include/hw/arm/calypso/calypso_sim.h"
TRX_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_trx.c"
BSP_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_bsp.c"
TINT0_C = f"{QEMU_SRC}/hw/arm/calypso/calypso_tint0.c"
# calypso-ipc-device a été réécrit Python -> C (fix 2026-07-03) : il n'y a
# plus de script `calypso-ipc-device` avec select.select(), la logique de
# pacing vit maintenant dans qemu_wrap.c (clock_nanosleep ABSTIME / QFN_FORCE).
BRIDGE_C = f"{QEMU_SRC}/tools/calypso-ipc-device/qemu_wrap.c"
QEMU_LOG = "/root/qemu.log"
BTS_LOG = "/root/bts.log"


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
# 2. (removed 2026-07-03) rxDoneFlag address injection no longer exists.
# =============================================================================
# `test_rxdone_addr_matches_firmware_nm` used to check that QEMU's
# hardcoded fallback `rxdone_addr` (a memory address QEMU poked directly
# to set the firmware's rxDoneFlag) matched the real firmware ELF symbol.
# That whole mechanism has been structurally removed from calypso_sim.c /
# calypso_trx.c — verified by grep, no `rxdone_addr`, no `0x008302xx`
# literal, no such getenv anywhere left in hw/. rxDoneFlag is now set by
# the firmware's own normal execution path; QEMU only provides correct
# SIM_IT/WT semantics (see section 1 above and the SIM_IT read handler in
# calypso_sim.c, "rxDoneFlag side-effect" comment). Nothing left to check
# here — deleted rather than left failing against dead code.


# =============================================================================
# 3. Clock domain alignment — tout le pipeline sur QEMU_CLOCK_VIRTUAL
# =============================================================================
# Sous icount=auto, VIRTUAL ≠ REALTIME. Mixer les deux = drift cumulatif.

@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
def test_bsp_drain_timer_on_virtual_clock():
    """BSP drain timer doit être REALTIME wall-paced avec rearm monotonic
    anti-drift (PAS VIRTUAL).

    Historique : fix 2026-05-24 était REALTIME → VIRTUAL (contre le drift
    bts↔qfn sous icount=auto). Fix 2026-05-29/30 (voir calypso_bsp.c) est
    revenu volontairement à REALTIME : sous charge DSP lourde, VIRTUAL
    tournait plus lentement que le wall clock → la queue UDP BSP débordait
    → 95% des bursts étaient droppés. tdma_tick/clk_master étant devenus
    REALTIME-monotonic entre-temps, wall et virtual restent alignés, donc
    le retour à REALTIME ne réintroduit pas le drift original — cf le
    rearm `last_target + period` dans bsp_drain_cb (anti-jitter monotonic,
    équivalent fonctionnel de l'alignement VIRTUAL visé par le fix d'origine).
    """
    src = _read(BSP_C)
    assert src, f"can't read {BSP_C}"
    assert re.search(
        r"timer_new_ns\s*\(\s*QEMU_CLOCK_REALTIME\s*,\s*bsp_drain_cb", src), (
        "BSP drain timer must use QEMU_CLOCK_REALTIME (2026-05-29 revert — "
        "VIRTUAL dropped 95% of bursts under DSP load)")
    # Le rearm anti-drift monotonic (`last_target`) doit être présent dans
    # bsp_drain_cb — c'est lui qui remplace la garantie qu'apportait VIRTUAL.
    assert "last_target" in src, (
        "bsp_drain_cb must keep the monotonic anti-drift rearm "
        "(last_target + period) now that the timer is REALTIME")


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
# 4. (removed 2026-07-03) Bridge.py select.select() jitter check no longer
#    applies — calypso-ipc-device was rewritten from Python to C.
# =============================================================================
# osmo-bts s'attend à CLK IND à 51 frames d'intervalle (235ms). Jitter
# trop élevé → "We were 1 FN faster/slower than TRX". `test_bridge_
# select_timeout_low_jitter` used to regex-match a Python
# `select.select(..., timeout)` call in a `calypso-ipc-device` script.
# That script doesn't exist anymore: tools/calypso-ipc-device/ is now a
# small C project (qemu_wrap.c et al, compiled to a binary). The
# select()-timeout polling loop was replaced by
# `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_deadline, NULL)`
# on an absolute deadline (see qemu_wrap.c, "WALL-PACED UL heartbeat"
# comment) — a different mechanism, sub-µs precision by construction, with
# no select()-timeout value to assert a bound on. Deleted rather than left
# failing against removed code; see test_bridge_clock_from_qemu_default
# below for the still-live, still-open pacing divergence in this same file.


# =============================================================================
# 5. (removed 2026-07-03) TCG race workaround — defer clear timer.
# =============================================================================
# `test_sim_defer_clear_timer_present` used to check for a QEMUTimer
# (`clear_edge_timer`/`clear_edge_cb`/`pending_edge_clear`) added to defer
# clearing SIM_IT edge bits by 1µs, working around a presumed
# cpu_io_recompile TB-truncation race under -icount=auto that could drop
# the STRNE committing rxDoneFlag.
#
# This was removed from calypso_sim.c on 2026-05-27 — verified by grep,
# none of those symbols exist anywhere in hw/ anymore — with an explicit,
# dated in-code justification (struct CalypsoSim comment + SIM_IT read
# handler comment): the presumed race was investigated further and proven
# EMPIRICALLY NOT TO EXIST (cputlb.c's cpu_io_recompile longjmps out via
# cpu_loop_exit_noexc *before* memory_region_dispatch_read runs; probes on
# 40 consecutive SIM_IT/UART reads showed the MMIO handler firing exactly
# once per LDR, 1:1, every time). Worse, the defer mechanism itself was
# actively harmful under icount=auto busy-polling (each read rescheduled
# the timer, the clear never fired, IRQ stayed asserted forever — a real
# deadlock in calypso_sim_powerup). It was replaced by a plain synchronous
# read-to-clear, which is what section 1's tests above verify and which
# passes today.
#
# Given the underlying bug this test tracked has been disproven (not
# fixed-via-workaround, but shown to never have existed) and the
# workaround code is gone, this test — and the xfail below that
# referenced it — are deleted rather than kept failing/xfailing against a
# narrative that no longer matches the code. Flagging for a second look:
# this is the most interpretive deletion in this file; the grounding is
# the dated source comments in calypso_sim.c (struct CalypsoSim field
# removal comment, and the SIM_IT read-handler comment above
# `s->it &= ~edge_seen;`), not an external doc.


# =============================================================================
# 6. (removed 2026-07-03) CALYPSO_FORCE_RX_DONE default — the env var itself
#    was removed as a hack.
# =============================================================================
# `test_force_rx_done_default_on` asserted that bash_scripts/run.sh
# defaults `CALYPSO_FORCE_RX_DONE=1`. That env var is not read by any .c
# file anymore (verified by grep across qemu-src) — it was one of several
# env-gated hacks (PM_INJECT, FBSB_SYNTH, DSP_FBDET_SKIP, FORCE_RX_DONE,
# BYPASS_BDLENA, FORCE_DARAM62) explicitly removed from source on
# 2026-05-26, per run.sh's own "=== Defaults no-hack ===" section:
# "Politique : aucun bypass de chemin firmware. [...] retires du code
# source 2026-05-26. Conforme a CLAUDE.md regle #1 : 'PAS DE HACK'."
# There is nothing left to default — asserting this would mean asking for
# the reintroduction of a hack the project deliberately removed.


# =============================================================================
# DIVERGENCES — workflow NON respecté (xfail / skip)
# =============================================================================

@pytest.mark.osmocom_divergent
@pytest.mark.osmocom_bridge
def test_bridge_clock_from_qemu_default():
    """CALYPSO_QFN_FORCE devrait être =1 par défaut sous icount=auto pour
    verrouiller le pacing du bridge sur le FN QEMU au lieu du wall clock.

    Fixed 2026-07-03 : ce test visait à l'origine `run.sh` avec un nom de
    variable qui avait été redacté en littéral `(removed)` dans le fichier
    (probablement un artefact d'un renommage/réécriture précédent) — la
    variable s'appelle en réalité CALYPSO_QFN_FORCE et son default vit
    dans le bridge C (tools/calypso-ipc-device/qemu_wrap.c), pas dans
    run.sh (qui ne la mentionne pas du tout, confirmé par grep).

    Actuel default = 0 (wall-paced, clock_nanosleep ABSTIME). Une
    première tentative de spin-wait qfn-pacé (chemin QFN_FORCE=1) a
    starve l'IPC osmo-trx sous lag QEMU (crash — cf commentaire "Échec"
    dans qemu_wrap.c). Divergence réelle et toujours ouverte : à flipper
    par défaut seulement une fois ce mode rendu robuste contre le starve.
    """
    src = _read(BRIDGE_C)
    assert src, f"can't read {BRIDGE_C}"
    m = re.search(
        r"QFN_FORCE\s*=\s*\(f\s*&&\s*\*f\s*==\s*'1'\)\s*\?\s*1\s*:\s*([01])",
        src)
    assert m, "CALYPSO_QFN_FORCE default ternary not found in qemu_wrap.c"
    default = m.group(1)
    if default == "0":
        pytest.xfail("CALYPSO_QFN_FORCE=0 default — pas idéal sous icount=auto, "
                     "mais le mode QFN_FORCE=1 a un starve IPC documenté "
                     "(non-robuste), à flipper seulement une fois corrigé")
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


# (removed 2026-07-03) `test_runtime_sim_it_wt_rxdone_log_present` used to
# xfail-check qemu.log for `[sim] SIM IT_WT → rxDoneFlag=1 @0x008302d4`,
# the log line produced by the (now removed, see section 2 above)
# hardcoded rxDoneFlag address-injection mechanism. That log line can
# never appear again — verified against the live qemu.log (grep finds no
# "rxDoneFlag" or "@0x0083" lines at all anymore) and against source (no
# code path writes it). The xfail reason also referenced
# CALYPSO_FORCE_RX_DONE as the fix to apply, which was itself removed as
# a hack (section 6). Deleted rather than left mis-xfailing for a reason
# that no longer describes anything reachable.


@pytest.mark.osmocom_compliant
@pytest.mark.osmocom_clock
@pytest.mark.skipif(not Path(BTS_LOG).exists() and not INSIDE,
                    reason="bts.log absent")
def test_runtime_bts_no_fn_skew_messages():
    """bts.log ne doit plus avoir massivement de 'We were 1 FN faster/slower'
    après fix calypso-ipc-device select timeout.
    """
    log = _read(BTS_LOG)
    if not log:
        pytest.skip("bts.log vide")
    skew = len(re.findall(r"We were 1 FN (faster|slower) than TRX", log))
    # On accepte un peu de skew sur démarrage (warmup)
    assert skew < 20, (
        f"bts.log contient {skew} messages 'FN faster/slower' — "
        f"jitter calypso-ipc-device trop élevé, vérifier select timeout")
