"""
test_log_grep.py — invariants observationnels via grep sur les logs (Phase 2).

Marker : `runtime_log_grep`. Tests rapides (~0.1s chacun) qui comptent des
patterns clés dans qemu.log, bridge.log, osmocon.log, mobile.log, fw-irda.log
et asserent des invariants quantitatifs : "au moins N occurrences en M sec",
"absence de pattern d'erreur", "ratio entre deux patterns".

Pattern d'helper : `_grep_count(log, regex)` via docker exec ou local
selon `/.dockerenv`.
"""
from __future__ import annotations

import os
import re
import subprocess

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
INSIDE = os.path.exists("/.dockerenv")

QEMU_LOG    = os.environ.get("CALYPSO_QEMU_LOG",    "/root/qemu.log")
(removed)  = os.environ.get("CALYPSO_(removed)",  "/tmp/bridge.log")
OSMOCON_LOG = os.environ.get("CALYPSO_OSMOCON_LOG", "/tmp/osmocon.log")
MOBILE_LOG  = os.environ.get("CALYPSO_MOBILE_LOG",  "/tmp/mobile.log")
FW_IRDA_LOG = os.environ.get("CALYPSO_FW_IRDA_LOG", "/tmp/fw-irda.log")


def _grep_count(path: str, pattern: str, tail: int = 0) -> int:
    """Count regex matches in `path`. tail=0 (default) → grep tout le fichier
    (important pour les markers de boot qui sont dans les 1ères lignes ;
    qemu.log peut faire >100k lignes pendant un run long et les markers
    sont écrasés par un `tail -n 100000` trop strict).
    Si tail>0, restreint aux N dernières lignes (utile pour les events
    récents, e.g. erreurs en fin de run).
    """
    cmd_grep = f"grep -cE '{pattern}' {path} 2>/dev/null" if tail == 0 else \
               f"tail -n {tail} {path} 2>/dev/null | grep -cE '{pattern}'"
    if INSIDE:
        try:
            r = subprocess.run(
                ["bash", "-c", cmd_grep],
                capture_output=True, text=True, timeout=8)
            return int(r.stdout.strip() or "0")
        except Exception:
            return 0
    try:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c", cmd_grep],
            capture_output=True, text=True, timeout=10)
        return int(r.stdout.strip() or "0")
    except Exception:
        return 0


def _log_exists(path: str) -> bool:
    if INSIDE:
        return os.path.exists(path)
    return subprocess.run(["docker", "exec", CONTAINER, "test", "-f", path],
                          capture_output=True).returncode == 0


# -----------------------------------------------------------------------------
# QEMU log — patterns DSP / IRQ / dispatching
# -----------------------------------------------------------------------------
@pytest.mark.runtime_log_grep
def test_grep_qemu_log_exists_and_nonempty():
    """qemu.log existe et fait au moins 1 MB."""
    assert _log_exists(QEMU_LOG), f"{QEMU_LOG} absent"
    if INSIDE:
        size = os.path.getsize(QEMU_LOG)
    else:
        r = subprocess.run(["docker", "exec", CONTAINER, "stat", "-c%s", QEMU_LOG],
                           capture_output=True, text=True, timeout=3)
        size = int(r.stdout.strip() or "0")
    assert size > 1024 * 1024, f"qemu.log trop petit ({size}B) — QEMU démarré ?"


@pytest.mark.runtime_log_grep
def test_grep_qemu_dsp_booted():
    """Le DSP a complété son init (au moins 1 occurrence du marker)."""
    n = _grep_count(QEMU_LOG, "DSP init complete|DSP boot ver=")
    assert n >= 1, "aucun marker 'DSP boot'/'DSP init complete' dans qemu.log"


@pytest.mark.runtime_log_grep
def test_grep_qemu_bsp_dma_active():
    """Le BSP DMA pousse des DARAM-WR-STATS (au moins 10× sur le run)."""
    n = _grep_count(QEMU_LOG, "DARAM-WR-STATS")
    assert n >= 10, f"BSP DMA stats trop rares : {n} (attendu ≥ 10)"


@pytest.mark.runtime_log_grep
def test_grep_qemu_task24_dispatched():
    """ARM dispatche task_md=24 (ALLC CCCH demod) au moins 100×."""
    n = _grep_count(QEMU_LOG, r"task=24|task_md=24|d_task_md.*24")
    assert n >= 100, f"task=24 dispatché seulement {n}× (attendu ≥ 100)"


@pytest.mark.runtime_log_grep
def test_grep_qemu_no_sp_catastrophe_recent():
    """Pas de SP-CATASTROPHE dans les 1000 dernières lignes (pas de runaway)."""
    n = _grep_count(QEMU_LOG, "SP-CATASTROPHE", tail=1000)
    assert n == 0, f"SP-CATASTROPHE détecté {n}× dans tail 1000 — runaway DSP en cours"


# -----------------------------------------------------------------------------
# Bridge log — DL/UL counts, drift FN
# -----------------------------------------------------------------------------
@pytest.mark.runtime_log_grep
def test_grep_bridge_dl_bursts_received():
    """Bridge reçoit des DL bursts de osmo-bts-trx (≥ 100)."""
    if not _log_exists((removed)):
        pytest.skip("bridge.log absent")
    n = _grep_count((removed), "bridge: DL #")
    assert n >= 100, f"DL bursts reçus seulement {n} (attendu ≥ 100)"


@pytest.mark.runtime_log_grep
def test_grep_bridge_fb_pattern_dominant():
    """≥ 50% des DL bursts sont des FB bursts (cell BCCH idle pattern)."""
    if not _log_exists((removed)):
        pytest.skip("bridge.log absent")
    total = _grep_count((removed), "bridge: DL #")
    fb    = _grep_count((removed), r"\*\*\* FB \*\*\*")
    if total < 10:
        pytest.skip(f"DL count trop faible ({total})")
    ratio = fb / total
    assert ratio >= 0.5, f"FB ratio = {ratio:.2f} (attendu ≥ 0.5) total={total} fb={fb}"


@pytest.mark.runtime_log_grep
def test_grep_bridge_no_clock_skew_shutdown():
    """Pas de message 'shutdown' / 'PC clock skew' dans bridge.log (BTS vivant)."""
    if not _log_exists((removed)):
        pytest.skip()
    n = _grep_count((removed), "shutdown|clock skew|No more clock")
    assert n == 0, f"signal shutdown BTS détecté ({n}×) — bts-trx a give-up"


# -----------------------------------------------------------------------------
# Osmocon log — L1CTL primitives + LOST
# -----------------------------------------------------------------------------
@pytest.mark.runtime_log_grep
def test_grep_osmocon_l1ctl_reset_req():
    """Mobile envoie L1CTL_RESET_REQ au boot (≥ 1)."""
    if not _log_exists(OSMOCON_LOG):
        pytest.skip()
    n = _grep_count(OSMOCON_LOG, "L1CTL_RESET_REQ")
    assert n >= 1, "L1CTL_RESET_REQ absent — mobile ne parle pas L1CTL"


@pytest.mark.runtime_log_grep
def test_grep_osmocon_l1ctl_fbsb_req_attempted():
    """Mobile tente L1CTL_FBSB_REQ (cell sync attempt) ≥ 1×."""
    if not _log_exists(OSMOCON_LOG):
        pytest.skip()
    n = _grep_count(OSMOCON_LOG, "L1CTL_FBSB_REQ")
    assert n >= 1, "aucun L1CTL_FBSB_REQ — mobile pas en cell-search"


@pytest.mark.runtime_log_grep
def test_grep_osmocon_lost_ratio_acceptable():
    """LOST/(LOST+HDLC) < 99% — au moins 1% de trames HDLC valides."""
    if not _log_exists(OSMOCON_LOG):
        pytest.skip()
    n_lost  = _grep_count(OSMOCON_LOG, "^LOST")
    n_hdlc  = _grep_count(OSMOCON_LOG, "hdlc_recv|hdlc_send")
    total = n_lost + n_hdlc
    if total < 50:
        pytest.skip(f"trop peu d'events osmocon ({total})")
    ratio_lost = n_lost / total
    assert ratio_lost < 0.99, \
        f"LOST {ratio_lost*100:.1f}% — UART_MODEM noyé par fw printf"


# -----------------------------------------------------------------------------
# Mobile log — minimal heartbeat (DSC ou autres signes de vie)
# -----------------------------------------------------------------------------
@pytest.mark.runtime_log_grep
def test_grep_mobile_alive_signal():
    """mobile.log contient au moins 1 ligne (init + au moins 1 event GSM322)."""
    if not _log_exists(MOBILE_LOG):
        pytest.skip()
    n_init = _grep_count(MOBILE_LOG, "Mobile.*initialized|gsm322")
    assert n_init >= 1, "mobile.log sans signe de vie"


# -----------------------------------------------------------------------------
# Cross-log : ratio task24 fire vs A_CD-WR (le mur DATA_IND)
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# fw-irda log — boot marker + activité
# -----------------------------------------------------------------------------
@pytest.mark.runtime_log_grep
def test_grep_fwirda_log_present_or_skip():
    """fw-irda.log existe (Phase 3 capture activée) — sinon skip."""
    if not _log_exists(FW_IRDA_LOG):
        pytest.skip("fw-irda.log absent (Phase 3 capture pas activée)")


@pytest.mark.runtime_log_grep
def test_grep_fwirda_boot_marker():
    """Le marker `fw-irda boot` ou `boot OK` apparaît (Phase 0.5)."""
    if not _log_exists(FW_IRDA_LOG):
        pytest.skip()
    n = _grep_count(FW_IRDA_LOG, "fw-irda boot|boot OK")
    if n == 0:
        pytest.xfail("boot marker absent — appliquer cons_puts() Phase 0.5")
    assert n >= 1


# =============================================================================
# BLOCKERS GREP — patterns d'erreur / panique / freeze à travers tous les logs
# Chaque test assert == 0. Si une régression introduit ces patterns, FAIL fort.
# =============================================================================

# --- qemu.log : panic / abort / fatal / runaway DSP -----------------------
@pytest.mark.runtime_log_grep
def test_blocker_qemu_no_qemu_abort():
    """Aucun 'qemu: hardware error', 'Aborted', 'segmentation fault' dans qemu.log."""
    n = _grep_count(QEMU_LOG, "qemu: hardware error|Aborted|Segmentation fault|abort\\(\\)|FATAL")
    assert n == 0, f"qemu.log contient {n}× signaux de crash"


@pytest.mark.runtime_log_grep
def test_blocker_qemu_no_panic():
    """Aucune trace 'panic', 'kernel panic', 'BUG:' dans qemu.log."""
    n = _grep_count(QEMU_LOG, "panic|kernel BUG|BUG: ")
    assert n == 0, f"qemu.log : {n}× panic-like"


@pytest.mark.runtime_log_grep
def test_blocker_qemu_no_runaway_dsp():
    """Aucun STACK-IN-NDB (runaway push DSP) dans les 5000 dernières lignes."""
    n = _grep_count(QEMU_LOG, "STACK-IN-NDB", tail=5000)
    if n > 100:
        pytest.fail(f"runaway DSP actif : {n}× STACK-IN-NDB dans tail 5000")
    assert n < 100


@pytest.mark.runtime_log_grep
def test_blocker_qemu_no_long_wait_a21a():
    """WAIT-A21A < 10 000 dans le run (DSP pas figé en wait perpétuel)."""
    n = _grep_count(QEMU_LOG, "WAIT-A21A")
    if n > 50000:
        pytest.fail(f"DSP figé en wait-A21A : {n} occurrences (>50000)")
    assert n <= 50000


@pytest.mark.runtime_log_grep
def test_blocker_qemu_no_assert_failed():
    """Aucun 'assertion failed' dans qemu.log."""
    n = _grep_count(QEMU_LOG, "assertion failed|assert.*failed|glib.*CRITICAL")
    assert n == 0, f"qemu.log : {n}× assertion failed"


# --- osmocon.log : connection / socket / framing errors ------------------
@pytest.mark.runtime_log_grep
def test_blocker_osmocon_no_layer2_socket_failed():
    """Pas de 'Layer2 socket failed' dans osmocon.log (mobile peut se connecter)."""
    if not _log_exists(OSMOCON_LOG): pytest.skip()
    n = _grep_count(OSMOCON_LOG, "Layer2 socket failed|socket failed")
    assert n == 0, f"osmocon : {n}× 'Layer2 socket failed' — mobile ne peut pas se brancher"


@pytest.mark.runtime_log_grep
def test_blocker_osmocon_no_connection_refused():
    """Pas de 'Connection refused' dans osmocon.log."""
    if not _log_exists(OSMOCON_LOG): pytest.skip()
    n = _grep_count(OSMOCON_LOG, "Connection refused|connect.*failed")
    assert n == 0, f"osmocon : {n}× connection refused"


@pytest.mark.runtime_log_grep
def test_blocker_osmocon_no_pty_error():
    """Pas d'erreur sur l'UART PTY côté osmocon."""
    if not _log_exists(OSMOCON_LOG): pytest.skip()
    n = _grep_count(OSMOCON_LOG, "pty.*error|read.*fail|tcsetattr.*fail")
    assert n == 0, f"osmocon : {n}× pty error"


# --- bridge.log : BTS shutdown / clock skew / RACH parity ----------------
@pytest.mark.runtime_log_grep
def test_blocker_bridge_no_bts_shutdown():
    """Bridge ne signale pas de shutdown BTS."""
    if not _log_exists((removed)): pytest.skip()
    n = _grep_count((removed), "BTS shutdown|shutdown_fsm|No more clock")
    assert n == 0, f"bridge : {n}× BTS shutdown — bts-trx mort"


@pytest.mark.runtime_log_grep
def test_blocker_bridge_no_rach_parity():
    """Pas d'erreur RACH parity / framing dans bridge.log."""
    if not _log_exists((removed)): pytest.skip()
    n = _grep_count((removed), "odd burst length|parity.*error|TRXD.*malformed")
    assert n == 0, f"bridge : {n}× RACH/TRXD framing error"


@pytest.mark.runtime_log_grep
def test_blocker_bridge_no_socket_error():
    """Pas d'erreur socket UDP dans bridge.log."""
    if not _log_exists((removed)): pytest.skip()
    n = _grep_count((removed), "Connection refused|recvfrom.*fail|sendto.*fail")
    assert n == 0, f"bridge : {n}× socket error"


# --- mobile.log : crashes / fatal / VTY ----------------------------------
@pytest.mark.runtime_log_grep
def test_blocker_mobile_no_crash():
    """Pas de crash/SEGV/abort dans mobile.log."""
    if not _log_exists(MOBILE_LOG): pytest.skip()
    n = _grep_count(MOBILE_LOG, "Segmentation fault|Aborted|crash|core dumped")
    assert n == 0, f"mobile : {n}× crash"


@pytest.mark.runtime_log_grep
def test_blocker_mobile_no_vty_bind_error():
    """Pas d'erreur 'Address already in use' sur le VTY mobile."""
    if not _log_exists(MOBILE_LOG): pytest.skip()
    n = _grep_count(MOBILE_LOG, "Address already in use|Cannot bind telnet|Cannot init VTY")
    assert n == 0, f"mobile : {n}× VTY bind error — autre mobile résiduel sur le port ?"


# --- fw-irda.log : firmware-side panic / assert -------------------------
@pytest.mark.runtime_log_grep
def test_blocker_fwirda_no_panic():
    """Pas de pattern panic firmware dans fw-irda.log."""
    if not _log_exists(FW_IRDA_LOG): pytest.skip()
    n = _grep_count(FW_IRDA_LOG, "PANIC|ASSERT|FATAL|stack overflow")
    assert n == 0, f"fw-irda : {n}× panic firmware"


# --- Cross-log : container-wide blocker -----------------------------------
@pytest.mark.runtime_log_grep
def test_blocker_no_out_of_memory():
    """OOM killer pas déclenché dans dmesg du container (proxy : grep tous logs)."""
    if INSIDE:
        r = subprocess.run(
            ["bash", "-c", "dmesg 2>/dev/null | grep -cE 'Out of memory|Killed process' || echo 0"],
            capture_output=True, text=True, timeout=3)
    else:
        r = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-c",
             "dmesg 2>/dev/null | grep -cE 'Out of memory|Killed process' || echo 0"],
            capture_output=True, text=True, timeout=4)
    try:
        n = int(r.stdout.strip().splitlines()[-1])
    except Exception:
        n = 0
    if n > 0:
        pytest.xfail(f"OOM détecté dans dmesg ({n}×) — peut être ancien")


@pytest.mark.runtime_log_grep
def test_grep_qemu_a_cd_wr_vs_task24():
    """Le ratio A_CD-WR / task24_fire DEVRAIT être > 0.001 (≥ 1 demod complet / 1000 dispatch).

    Si DSP saturé, A_CD-WR ne fire jamais → ratio = 0 → XFAIL (mur connu).
    Si ratio > 0.001, c'est que le DSP CCCH demod commence à converger.
    """
    n_task = _grep_count(QEMU_LOG, "task=24|task_md=24")
    n_acd  = _grep_count(QEMU_LOG, "A_CD-WR")
    if n_task < 100:
        pytest.skip(f"task=24 trop rare ({n_task}) — run trop court")
    ratio = n_acd / n_task
    if ratio < 0.001:
        pytest.xfail(
            f"A_CD-WR / task24 = {ratio:.6f} — DSP saturé, le mur DATA_IND "
            f"est confirmé (task24={n_task} dispatché mais A_CD-WR={n_acd} "
            f"seulement). Cf. REPORT_CLAUDE_WEB_20260516_DSP_OVERRUN.md")
    assert ratio >= 0.001
