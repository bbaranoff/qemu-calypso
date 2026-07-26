"""
Non-regression suite built on the emulator's screaming invariants.

Philosophy (2026-07-25): internal probes raise resolution, not independence.
These tests do NOT re-derive facts by eyeballing logs — they assert on the
[INVARIANT-FAIL]/[MANIFEST] lines the binary emits itself (calypso_invariants.c).
A fix silently lost between two runs (e.g. the IMR bit5 re-arm) then screams on
the very next run instead of being OCR'd off a 5am video capture.

External TRUTH (is the DSP output valid GSM?) is deliberately NOT here: that can
only come from a decoder we did not write (GSMTAP -> Wireshark / gr-gsm).
"""
import os
import re
import pytest

LOG = os.environ.get("CALYPSO_QEMU_LOG", "/root/qemu.log")


def _log():
    try:
        return open(LOG, "rb").read().decode("latin-1", "replace")
    except FileNotFoundError:
        pytest.skip(f"{LOG} absent (no run to inspect)")


def _fails(text):
    """tag -> first [INVARIANT-FAIL] message for that tag."""
    out = {}
    for m in re.finditer(r"\[INVARIANT-FAIL\] (\S+) : (.*)", text):
        out.setdefault(m.group(1), m.group(2))
    return out


def test_manifest_emitted():
    """Every run must log its active CALYPSO_* forcings (reproducibility)."""
    text = _log()
    assert "[MANIFEST]" in text, "no run manifest — cannot know which forcings were active"


def test_manifest_reports_wire_gates():
    """The manifest must actually enumerate CALYPSO_* variables (not be empty)."""
    text = _log()
    man = "\n".join(l for l in text.splitlines() if "[MANIFEST]" in l)
    assert "CALYPSO_" in man, "manifest present but lists no CALYPSO_* variable"


def test_imr_brint0_armed_regression_guard():
    """REGRESSION GUARD. BRINT0 (IMR bit5) must stay armed past go-live. This is the
    fix that was silently lost between two runs — if it regresses, this FAILS loud."""
    fails = _fails(_log())
    assert "imr_brint0_armed" not in fails, \
        f"IMR bit5 (BRINT0) regressed: {fails.get('imr_brint0_armed')}"


def test_correlator_ar4_sweeps():
    """GUARD. AR4 (write ptr) must take >2 distinct values — settles the
    'stuck on 2 addresses' question that was a terminal-display artifact."""
    fails = _fails(_log())
    assert "correlator_ar4_sweeps" not in fails, fails.get("correlator_ar4_sweeps")


@pytest.mark.xfail(reason="RANK3 open: FB kernel (0xa076/0x9a80) that sets AR5=0x2a00 never runs", strict=False)
def test_correlator_ar5_in_iq_buffer():
    """FRONTIER. AR5 (I/Q read ptr) must live in [0x2a00..0x2b27]. Observed 0xdb7b:
    the correlator reads OUTSIDE the buffer. Flips XFAIL->PASS when RANK3 lands."""
    fails = _fails(_log())
    assert "correlator_ar5_in_iq_buffer" not in fails, fails.get("correlator_ar5_in_iq_buffer")


@pytest.mark.xfail(reason="RANK2/RANK4 open: FN offset dsp/bridge (native RX chain)", strict=False)
def test_fn_dsp_bridge_delta():
    """FRONTIER. dsp FN and bridge FN within one match window. Flips when the FN
    recale (RANK4) / RX chain lands."""
    fails = _fails(_log())
    assert "fn_dsp_bridge_delta" not in fails, fails.get("fn_dsp_bridge_delta")
