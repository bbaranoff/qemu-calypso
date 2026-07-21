"""
test_mode_verdict.py — Verdict engine PAR MODE.

Tourne dans chaque pytest individuel (par mode CALYPSO_MODE). Examine les
logs live du run en cours (qemu.log, mobile.log, bts.log, etc.) et tire
des conclusions explicites avec leurs évidences.

Le pytest cross-mode (test_run_all_modes.py) combine ensuite ces verdicts
sur tous les modes.

Mode tag : env CALYPSO_MODE_TAG (set par run.sh ou run-all.sh) ou autoderived.
"""

import os
import re
from pathlib import Path
import pytest


LOGS = {
    "qemu":    Path("/root/qemu.log"),
    "mobile":  Path("/tmp/mobile.log"),
    "bts":     Path("/tmp/bts.log"),
    "osmocon": Path("/tmp/osmocon.log"),
    "ipc":     Path("/tmp/calypso-ipc-device.log"),
    "trxipc":  Path("/tmp/osmo-trx-ipc.log"),
    "bridge":  Path("/tmp/bridge.py.log"),
    "irda":    Path("/tmp/fw-irda.log"),
}


def _read(p, limit=200_000):
    """Read tail of a log file. Returns "" if absent."""
    if not p.exists():
        return ""
    try:
        with open(p, errors="ignore") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - limit))
            return f.read()
    except Exception:
        return ""


def _count(pattern, text):
    return len(re.findall(pattern, text))


@pytest.fixture(scope="session")
def mode():
    return os.environ.get("CALYPSO_MODE_TAG") or os.environ.get("CALYPSO_MODE", "unknown")


@pytest.fixture(scope="session")
def indicators():
    """Scan all logs and collect indicators for this mode's verdict."""
    q = _read(LOGS["qemu"])
    m = _read(LOGS["mobile"])
    b = _read(LOGS["bts"])
    i = _read(LOGS["ipc"])
    t = _read(LOGS["trxipc"])
    br = _read(LOGS["bridge"])
    return {
        "shunt_latch":  _count(r"\[dsp-shunt\] LATCH", q),
        "shunt_disp":   _count(r"\[dsp-shunt\] DISPATCH", q),
        "fbsb_conf":    _count(r"FBSB_CONF|fbsb.*conf", m),
        "rr_est":       _count(r"RR_EST|RR EST_REQ", m),
        "lost":         _count(r"LOST [0-9]", m),
        "dsc":          _count(r"using DSC of", m),
        "err_q":        _count(r"ERR|FATAL|panic|assert", q),
        "err_m":        _count(r"ERR|FATAL|panic", m),
        "bts_alive":    _count(r"phy0\.0: Opening|DL1C NOTICE|FN faster than TRX|RACH received|Listening|TRX online", b),
        "bts_poweroff": _count(r"No satisfactory.*POWEROFF", b),
        "ipc_handshake": _count(r"GREETING|OPEN_CNF|info_cnf", i),
        "ipc_err":      _count(r"DDEV ERROR|chan num mismatch|antenna not found", t),
        "bridge_active": _count(r"bridge.*start|FN", br),
    }


# ---- Verdicts par mode ----

VERDICTS = []

def _verdict(name, evidences, conclusion, next_steps):
    matched = [e for e, ok in evidences if ok]
    if matched:
        VERDICTS.append({
            "name": name,
            "matched": matched,
            "conclusion": conclusion,
            "next_steps": next_steps,
            "weight": len(matched),
        })


def test_verdict_shunt_works(mode, indicators):
    """Mode shunt : DSP mock dispatche et FBSB tire."""
    if mode not in {"shunt", "shunt-ipc"}:
        pytest.skip(f"verdict ne s'applique pas au mode {mode}")
    ind = indicators
    _verdict("SHUNT_DISPATCHES_AND_FBSB_OK",
        evidences=[
            (f"shunt_latch={ind['shunt_latch']} > 0", ind["shunt_latch"] > 0),
            (f"shunt_disp={ind['shunt_disp']} > 0", ind["shunt_disp"] > 0),
            (f"fbsb_conf={ind['fbsb_conf']} > 0", ind["fbsb_conf"] > 0),
        ],
        conclusion="Le shunt fonctionne nominalement : ARM pose des tâches, "
                   "le mock dispatche, et FBSB_CONF tire côté mobile.",
        next_steps=[
            "Si DSC répété 'using DSC of 90' → implémenter shunt_dispatch_allc",
            "Vérifier RR_EST_REQ tentative et IMM_ASS_CMD reception",
        ])


def test_verdict_shunt_silent(mode, indicators):
    """Mode shunt : mock ne dispatch jamais."""
    if mode not in {"shunt", "shunt-ipc"}:
        pytest.skip(f"verdict ne s'applique pas au mode {mode}")
    ind = indicators
    _verdict("SHUNT_NOT_DISPATCHING",
        evidences=[
            (f"shunt_latch={ind['shunt_latch']} == 0", ind["shunt_latch"] == 0),
        ],
        conclusion="Le shunt est actif mais l'ARM ne pose JAMAIS de tâche. "
                   "Soit le firmware ne boot pas, soit le hook NDB+0 d'overlay est cassé.",
        next_steps=[
            "Grep 'dsp-shunt active' dans qemu.log pour confirmer l'init",
            "Vérifier MemoryRegion overlay priority + addresse 0xFFD001A8",
            "Confirmer que d_dsp_page IS écrit (cf prim_fbsb.c:471 dsp_end_scenario)",
        ])


def test_verdict_bts_dead(mode, indicators):
    """BTS attendu mais POWEROFF stuck."""
    if mode not in {"full", "shunt-ipc", "bridge"}:
        pytest.skip(f"verdict ne s'applique pas au mode {mode}")
    ind = indicators
    _verdict("BTS_DEAD",
        evidences=[
            (f"bts_alive={ind['bts_alive']} == 0", ind["bts_alive"] == 0),
            (f"bts_poweroff={ind['bts_poweroff']} > 5", ind["bts_poweroff"] > 5),
        ],
        conclusion="osmo-bts-trx ne reçoit jamais de réponse satisfaisante "
                   "du transceiver. Soit IPC handshake fail, soit chan/path mismatch, "
                   "soit osmo-trx-ipc plante avant START.",
        next_steps=[
            "Vérifier osmo-trx-ipc.log pour DDEV ERROR",
            "Vérifier cfgs/osmo-trx-ipc.cfg : chan 0 seul, rx-path RX, tx-path TX",
            "Confirmer calypso-ipc-device démarré avant osmo-trx-ipc",
        ])


def test_verdict_ipc_chan_mismatch(mode, indicators):
    """IPC handshake fail à cause cfg."""
    if mode not in {"full", "shunt-ipc"}:
        pytest.skip(f"verdict ne s'applique pas au mode {mode}")
    ind = indicators
    _verdict("IPC_CFG_MISMATCH",
        evidences=[
            (f"ipc_err={ind['ipc_err']} > 0", ind["ipc_err"] > 0),
            (f"ipc_handshake={ind['ipc_handshake']} == 0", ind["ipc_handshake"] == 0),
        ],
        conclusion="osmo-trx-ipc rejette le handshake avec calypso-ipc-device. "
                   "Probable : chan count, rx-path, tx-path ou shm_name mismatch.",
        next_steps=[
            "Cat osmo-trx-ipc.log pour le message d'erreur exact",
            "Fix cfgs/osmo-trx-ipc.cfg en conséquence",
        ])


def test_verdict_mobile_stuck_gsm322(mode, indicators):
    """Mobile en boucle DSC sans avancer."""
    ind = indicators
    _verdict("MOBILE_STUCK_GSM322",
        evidences=[
            (f"dsc='using DSC of'={ind['dsc']} > 5", ind["dsc"] > 5),
            (f"rr_est={ind['rr_est']} == 0", ind["rr_est"] == 0),
        ],
        conclusion="Le mobile camp sur une cellule mais ne lit JAMAIS le SI3 "
                   "complet (pas de RR_EST_REQ). Il boucle en gsm322 cell selection.",
        next_steps=[
            "Si mode shunt : implémenter shunt_dispatch_allc (SI3 canné)",
            "Si mode full/bridge : vérifier que la BTS broadcast réellement SI3",
            "Confirmer ARFCN + BSIC + cell selection criteria",
        ])


def test_verdict_instability(mode, indicators):
    """LOST frames anormaux."""
    ind = indicators
    _verdict("TIMER_INSTABILITY",
        evidences=[
            (f"lost={ind['lost']} > 1000", ind["lost"] > 1000),
        ],
        conclusion="Beaucoup de LOST frames côté mobile — timer TPU/TDMA "
                   "instable ou host trop chargé.",
        next_steps=[
            "Forcer CALYPSO_ICOUNT=auto",
            "Désactiver CALYPSO_MTTCG si activé",
            "Profiler le host (autres CPU-hungry processes)",
        ])


# ---- Final per-mode report ----

def test_zz_per_mode_report(mode, indicators):
    """Affiche tableau + verdicts pour CE mode uniquement."""
    print(f"\n\n========== MODE VERDICT REPORT : {mode} ==========\n")
    print("  Indicators :")
    for k, v in indicators.items():
        marker = "✓" if (v > 0 if k.endswith("alive") or k in {"shunt_latch","shunt_disp","fbsb_conf","ipc_handshake"} else v == 0) else " "
        print(f"    {marker} {k:18s} = {v}")
    print()
    if VERDICTS:
        VERDICTS.sort(key=lambda v: -v["weight"])
        for i, v in enumerate(VERDICTS, 1):
            star = "★" if i == 1 else " "
            print(f"  {star} [{i}] {v['name']} (weight {v['weight']})")
            for e in v["matched"]:
                print(f"        • {e}")
            print(f"      → {v['conclusion']}")
            for s in v["next_steps"]:
                print(f"        ↳ {s}")
            print()
    else:
        print("  (pas de verdict matché pour ce mode)\n")
    print("================================================\n")

    # Export JSON pour consolidation cross-mode
    out_dir = Path(os.environ.get("CALYPSO_TEST_OUT", "/tmp"))
    out = {
        "mode": mode,
        "indicators": indicators,
        "verdicts": VERDICTS,
    }
    try:
        import json
        path = out_dir / f"verdict_{mode}.json"
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
        print(f"  Verdict JSON exporté : {path}\n")
    except Exception as e:
        print(f"  (export JSON failed: {e})\n")
