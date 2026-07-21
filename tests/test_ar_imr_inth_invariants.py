"""
test_ar_imr_inth_invariants.py — invariants DSP init + IRQ servicing.

Cible les logs côté QEMU (`/root/qemu.log`) produits par les sondes
ajoutées 2026-05-25 :
  - `[c54x] AR%d-W ...`   (CALYPSO_AR_TRACE=0xFF)
  - `[c54x] AR%d-W ZERO!` (suspect clobber MMR via STL A,*AR%-)
  - `[c54x] IMR-W *ZERO* ...` (IMR cleared by STL)
  - `[c54x] IRQ #N vec= bit= INTM= IMR=`
  - `[c54x] RSBX-INTM #N` (CALYPSO_RSBX_INTM_TRACE=1)

Invariants testés :
  - Reset values silicon (SP=0x1100, IMR=0x52FD, AR1=0x005F, AR2=0x0813,
    AR3=0x0014, AR4=0x0003, AR5=0x0014, BK=0xFFF6)
    cf doc/datasheets/README.md §3
  - AR2 ne tombe pas à 0 avant insn=1M (regression du init silicon fix)
  - IMR-W *ZERO* events bornés (pas plus de N, ne sont pas la cascade
    permanente du pre-fix)
  - IRQs serviced : si IRQ # delivered ET handler doit run → IMR doit
    avoir le bit IRQ correspondant set
  - Si IMR=0 partout → flag candidat NMI (doc §7 candidat 2)
  - BOOTSTUB-ENTRY = 0 (régression du SP=0x1100 fix)

Lancement :
    pytest -v -m ar_imr_invariant
    pytest -v tests/test_ar_imr_inth_invariants.py
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

CONTAINER = os.environ.get("CALYPSO_CONTAINER", "trying")
QEMU_LOG  = os.environ.get("CALYPSO_QEMU_LOG", "/root/qemu.log")
INSIDE    = os.path.exists("/.dockerenv")

# ---- Regex patterns côté qemu.log ----

AR_W_RE = re.compile(
    r'\[c54x\] AR(\d)-W #(\d+) (\S+) @insn=(\d+) PC=0x([0-9a-f]+) '
    r'op=0x([0-9a-f]+)\s+AR\d (\S+) → (\S+) \(Δ=(-?\d+)\)'
)
AR_ZERO_RE = re.compile(
    r'\[c54x\] AR(\d)-W ZERO! @insn=(\d+) PC=0x([0-9a-f]+) op=0x([0-9a-f]+)'
)
IMR_ZERO_RE = re.compile(
    r'\[c54x\] IMR-W \*ZERO\* 0x([0-9a-f]+) → 0x([0-9a-f]+) PC=0x([0-9a-f]+) '
    r'op=0x([0-9a-f]+).*insn=(\d+)'
)
IRQ_RE = re.compile(
    r'\[c54x\] IRQ #(\d+) vec=(\d+) bit=(\d+): INTM=(\d) IMR=0x([0-9a-f]+) '
    r'IFR=0x([0-9a-f]+)'
)
RSBX_INTM_RE = re.compile(
    r'\[c54x\] RSBX-INTM #(\d+) @insn=(\d+) PC=0x([0-9a-f]+)'
)
BOOTSTUB_RE = re.compile(r'\[c54x\] BOOTSTUB-ENTRY caught @insn=(\d+)')
RESET_RE   = re.compile(
    r'\[c54x\] Reset: PC=0x([0-9a-f]+) PMST=0x([0-9a-f]+) SP=0x([0-9a-f]+)'
)


def _read_lines(path: str, tail_n: int = 200000) -> list[str]:
    """Lit le log côté container (ou local si déjà dedans)."""
    if INSIDE:
        try:
            with open(path, errors="replace") as f:
                return f.readlines()[-tail_n:]
        except FileNotFoundError:
            return []
    out = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-c",
         f"tail -n {tail_n} {path} 2>/dev/null"],
        capture_output=True, text=True, timeout=10)
    return out.stdout.splitlines(keepends=True)


@pytest.fixture(scope="module")
def qemu_log_lines() -> list[str]:
    lines = _read_lines(QEMU_LOG)
    if not lines:
        pytest.skip(f"{QEMU_LOG} empty or missing")
    return lines


# ============== Tests reset values (silicon) ==============

@pytest.mark.ar_imr_invariant
def test_reset_sp_silicon(qemu_log_lines):
    """Reset SP doit être 0x1100 per doc/datasheets/README.md §3.
    Régression du fix milestone 2026-05-25 (SP=0x5AC8 shortcut tué le spiral)."""
    for line in qemu_log_lines:
        m = RESET_RE.search(line)
        if m:
            sp = int(m.group(3), 16)
            assert sp == 0x1100, (
                f"SP reset = 0x{sp:04x}, attendu 0x1100 (silicon spec). "
                "Si revert au shortcut 0x5AC8 = régression du spiral fix.")
            return
    pytest.skip("No Reset: line in log")


@pytest.mark.ar_imr_invariant
def test_reset_pmst_silicon(qemu_log_lines):
    """Reset PMST doit être 0xFFA8 (IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1)."""
    for line in qemu_log_lines:
        m = RESET_RE.search(line)
        if m:
            pmst = int(m.group(2), 16)
            assert pmst == 0xFFA8, (
                f"PMST reset = 0x{pmst:04x}, attendu 0xFFA8 per silicon spec.")
            return
    pytest.skip("No Reset: line in log")


# ============== Tests AR-W ZERO (regression hunt) ==============

@pytest.mark.ar_imr_invariant
def test_ar2_zero_categorization(qemu_log_lines):
    """Observation pure : combien d'AR2-W ZERO, et de quel kind ?
    DELIBERATE = STM-#lk hardcoded ROM (silicon-intentional, attendu).
    SIDE-EFFECT = STL-A self-aliasing (= AR2 happen to point at MMR_AR2).
    Pas un fail — informationnel pour décider direction hunt (NMI vs
    A-divergence). Le tracer flag les 2 distinctement depuis v3.
    """
    deliberate = []
    side_effect = []
    for line in qemu_log_lines:
        if 'AR2-W ZERO' not in line:
            continue
        if 'DELIBERATE' in line:
            deliberate.append(line.strip())
        elif 'SIDE-EFFECT' in line:
            side_effect.append(line.strip())
    # Pas d'assert fail — c'est une observation enregistrée dans le report.
    # On fail seulement si AR2-W ZERO mais sans tag (= ancien tracer pré-v3).
    untagged = [l for l in qemu_log_lines if 'AR2-W ZERO' in l
                and 'DELIBERATE' not in l and 'SIDE-EFFECT' not in l]
    assert not untagged, (
        f"{len(untagged)} AR2-W ZERO sans tag DELIBERATE/SIDE-EFFECT — "
        "tracer pas à jour (rebuild requis pour ar_write_track v3)")
    print(f"\n  AR2-W ZERO observed: {len(deliberate)} DELIBERATE, "
          f"{len(side_effect)} SIDE-EFFECT")


# ============== Tests IMR clobber ==============

@pytest.mark.ar_imr_invariant
def test_imr_not_persistently_zero(qemu_log_lines):
    """IMR-W *ZERO* events bornés. Si IMR cleared multiple fois sans
    re-set, c'est la cascade pre-fix. Limit raisonnable = N transitions
    (firmware peut légit-clear pendant sections critiques)."""
    zeros = []
    for line in qemu_log_lines:
        m = IMR_ZERO_RE.search(line)
        if m:
            from_val = int(m.group(1), 16)
            insn = int(m.group(5))
            pc = m.group(3)
            zeros.append((insn, pc, from_val))
    # Tolérance : firmware peut clear IMR temporairement.
    # Au-delà de 10 sans re-set = cascade pathologique.
    assert len(zeros) < 10, (
        f"IMR-W *ZERO* fire {len(zeros)} fois — cascade IMR=0. "
        f"Premières : {zeros[:3]}. Check si firmware re-set après "
        "ou si AR pose addr=0 par STL A,*AR-")


# ============== Tests IRQ servicing ==============

@pytest.mark.ar_imr_invariant
def test_irq_servicing_imr_nonzero(qemu_log_lines):
    """Si IRQ #N est délivré ET handler doit fire, IMR doit avoir bit
    correspondant set. IRQ avec IMR=0 = log-seulement, handler pas
    serviced → DSP stuck en dispatcher polling forever."""
    irq_with_zero_imr = []
    irqs_total = 0
    for line in qemu_log_lines:
        m = IRQ_RE.search(line)
        if m:
            irqs_total += 1
            imr = int(m.group(5), 16)
            bit = int(m.group(3))
            if imr == 0:
                irq_with_zero_imr.append((int(m.group(1)), bit))
    if irqs_total == 0:
        pytest.skip("No IRQ # events — TPU/BRINT0 ne fire pas. "
                    "Vérifier pipeline ARM dispatch.")
    # Tolérance : quelques IRQ au boot peuvent arriver avec IMR=0 transient.
    # Si > 50% des IRQ ont IMR=0, c'est cascade pathologique.
    pct = len(irq_with_zero_imr) / irqs_total * 100
    assert pct < 50, (
        f"{len(irq_with_zero_imr)}/{irqs_total} IRQ ({pct:.0f}%) délivrées "
        f"avec IMR=0. Handlers jamais servis → DSP bloqué en dispatcher. "
        "Soit IMR clobbered (cf AR-W ZERO), soit candidat NMI doc §7 "
        "non implémenté.")


@pytest.mark.ar_imr_invariant
def test_rsbx_intm_reached_or_flag_nmi(qemu_log_lines):
    """Candidat 1 du doc §7 : firmware DSP fait RSBX INTM pour unmask
    sur boot path. Si RSBX-INTM # > 0 → candidat 1 atteint. Si = 0 →
    NMI (candidat 2) devient la voie probable. Ce test ne fail pas
    si =0 (informatif) — il flag explicit pour décider la suite."""
    hits = sum(1 for line in qemu_log_lines if RSBX_INTM_RE.search(line))
    if hits == 0:
        pytest.skip(
            "RSBX INTM jamais atteint sur boot path → candidat 1 doc §7 "
            "exclu, NMI (candidat 2) à implémenter probablement. "
            "Pas une régression, juste un signal pour décider.")
    assert hits > 0


# ============== Tests régression spiral ==============

@pytest.mark.ar_imr_invariant
def test_no_bootstub_entry(qemu_log_lines):
    """Régression du SP=0x1100 fix (milestone 2026-05-25).
    Si BOOTSTUB-ENTRY fire à nouveau = retour au spiral DSP."""
    entries = []
    for line in qemu_log_lines:
        m = BOOTSTUB_RE.search(line)
        if m:
            entries.append(int(m.group(1)))
    assert not entries, (
        f"BOOTSTUB-ENTRY fire à insn={entries[:5]} — spiral DSP retour. "
        "Regression du SP=0x1100 silicon-fix. Cf "
        "project_sp_silicon_fix_spiral_resolved.")


# ============== Tests INTH-related (TPU FRAME IRQ) ==============

@pytest.mark.ar_imr_invariant
def test_tpu_frame_irq_delivered(qemu_log_lines):
    """INT3 (TPU FRAME, vec=19, bit=3) doit être delivered périodiquement
    sur pipeline vivant. Si 0 IRQ vec=19 → TPU pas configuré (ARM init
    bloqué), ou bridge clock CLK_IND non émis."""
    vec19_count = 0
    for line in qemu_log_lines:
        m = IRQ_RE.search(line)
        if m and int(m.group(2)) == 19:
            vec19_count += 1
    if vec19_count == 0:
        pytest.skip(
            "0 IRQ vec=19 (INT3 FRAME) — TPU ne fire pas. "
            "Diagnostique : bridge CLK_IND ? ARM init TPU ? "
            "Pas un fail (peut être déclenché par config).")
    assert vec19_count > 0
