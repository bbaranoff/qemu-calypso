"""
Pytest configuration for calypso milestone tests.

Marks are registered here so pytest doesn't warn about unknown markers.
"""
import pytest


def pytest_configure(config):
    for marker in (
        # test_calypso_milestones.py
        "milestone_dsp_decoder: régression décodeur (POPM, Tier A, INTM dwell)",
        "milestone_bsp_dma:     data path ARM/BSP → DARAM (priorité A actuelle)",
        "milestone_fb_det:      correlator FB-det converge",
        "milestone_irq:         cycle IRQ DSP complet (SINT → ISR → RETE)",
        "milestone_l1ctl:       premier produit L1 vers mobile",
        "milestone_mm_lu:       location update bout-en-bout",
        # test_run_observability.py
        "runtime_health:        container alive + processus attendus",
        "runtime_dsp:           sample qemu.log + probes DSP",
        "runtime_bridge:        bridge.py drift FN + lookahead",
        "runtime_l1ctl:         pcap GSMTAP via tshark",
        "runtime_vty:           VTY mobile L23 (état RR/MM)",
        "runtime_irq:           compteurs IRQ via monitor QEMU",
        "runtime_summary:       snapshot consolidé (pytest -m runtime_summary -s)",
    ):
        config.addinivalue_line("markers", marker)
