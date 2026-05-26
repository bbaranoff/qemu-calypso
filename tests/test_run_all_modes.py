"""
test_run_all_modes.py — Cross-mode comparison report avec assertions
conditionnelles selon CALYPSO_MODE.

Sources JSON :
  /tmp/run-all/results.json    (run-all.sh output, multi-modes)

Conditions par mode (kernel-menuconfig style) :
  full       → DSP réel + IPC + BTS attendus alive
  shunt      → DSP_SHUNT actif + canned FB+SB, pas d'IPC/BTS
  shunt-ipc  → DSP_SHUNT actif + IPC + BTS alive (côté radio cosmetic)
  bridge     → bridge.py + BTS alive
  bare       → QEMU seul

Chaque test affiche son résultat même en cas d'échec — le rapport final
liste tout en tableau.
"""

import json
import os
from pathlib import Path

import pytest

JSON_PATH = Path(os.environ.get("RUN_ALL_JSON", "/tmp/run-all/results.json"))


@pytest.fixture(scope="session")
def results():
    if not JSON_PATH.exists():
        pytest.skip(f"results.json absent: {JSON_PATH}")
    with open(JSON_PATH) as f:
        return json.load(f)


@pytest.fixture(scope="session")
def modes(results):
    return results.get("modes", {})


# ---- Conditions par mode (table déclarative) ----
#
# Pour chaque mode, on liste les invariants attendus. Chaque tuple est
# (key, op, value, why). op ∈ {">", "==", ">=", "0"}.
MODE_INVARIANTS = {
    "full": [
        ("bts_ok", ">", 0, "osmo-bts-trx doit être online"),
        ("ipc_ok", ">", 0, "calypso-ipc-device doit avoir handshake-é"),
        ("err_q", "==", 0, "qemu.log sans erreur"),
        # FBSB peut tirer ou non selon convergence DSP réel — pas d'assert
    ],
    "shunt": [
        ("shunt_latch", ">", 0,
         "ARM doit poser au moins une tâche (sinon firmware boot fail)"),
        ("shunt_disp", ">", 0,
         "le mock doit dispatch au moins une fois (sinon frame_irq hook cassé)"),
        ("err_q", "==", 0, "qemu.log sans erreur"),
        ("bts_ok", "==", 0, "BTS doit être skipped en shunt mode"),
        ("ipc_ok", "==", 0, "IPC doit être skipped en shunt mode"),
    ],
    "shunt-ipc": [
        ("shunt_latch", ">", 0,
         "ARM doit poser au moins une tâche"),
        ("shunt_disp", ">", 0,
         "le mock doit dispatch"),
        ("bts_ok", ">", 0,
         "BTS alive en shunt-ipc (radio chain cosmetic)"),
        ("ipc_ok", ">", 0,
         "IPC handshake (devrait passer avec cfg 1-chan)"),
    ],
    "bridge": [
        ("bts_ok", ">", 0, "BTS alive avec bridge.py legacy"),
        ("ipc_ok", "==", 0, "IPC skipped en bridge mode (mutex)"),
    ],
    "bare": [
        ("bts_ok", "==", 0, "pas de BTS en bare"),
        ("ipc_ok", "==", 0, "pas d'IPC en bare"),
        ("rr_est", "==", 0, "pas de mobile en bare"),
    ],
}


# ---- Existence ----

@pytest.mark.parametrize("mode", list(MODE_INVARIANTS.keys()))
def test_mode_executed(modes, mode):
    """Le mode a été exécuté et a produit des indicateurs."""
    assert mode in modes, f"mode '{mode}' absent — pas exécuté ?"


# ---- Invariants conditionnels ----

@pytest.mark.parametrize("mode,key,op,expected,why",
    [(m, k, o, v, w)
     for m, conds in MODE_INVARIANTS.items()
     for (k, o, v, w) in conds])
def test_mode_invariant(modes, mode, key, op, expected, why):
    if mode not in modes:
        pytest.skip(f"mode {mode} absent")
    actual = modes[mode].get(key, 0)
    if op == ">":
        ok = actual > expected
    elif op == "==":
        ok = actual == expected
    elif op == ">=":
        ok = actual >= expected
    else:
        pytest.fail(f"unknown op '{op}'")
    print(f"\n  [{mode}] {key} {op} {expected} → got {actual} ({why})")
    assert ok, f"[{mode}] {key}={actual}, expected {op} {expected}: {why}"


# ---- Comparaisons cross-mode ----

def test_bissection_fbsb_shunt_vs_full(modes):
    """Bissection clé : si shunt fbsb_conf > 0 et full == 0, le DSP était le mur.

    Cas attendus :
      shunt>0  full>0  → les deux marchent, pas de mur
      shunt>0  full=0  → mur DSP confirmé
      shunt=0  full=0  → bug côté ARM (pas DSP)
      shunt=0  full>0  → bizarre (full passe sans shunt mais shunt non)
    """
    s = modes.get("shunt", {}).get("fbsb_conf", 0)
    f = modes.get("full", {}).get("fbsb_conf", 0)
    print(f"\n  FBSB_CONF : shunt={s}  full={f}")
    if s > 0 and f == 0:
        print("    VERDICT : mur DSP confirmé (Phase 2 IPC requise pour vrai LU)")
    elif s == 0 and f == 0:
        print("    VERDICT : bug ARM, ni shunt ni full ne convergent")
        pytest.fail("ni shunt ni full ne tirent FBSB_CONF — bug ARM")
    elif s > 0 and f > 0:
        print("    VERDICT : les deux convergent — DSP n'est plus un mur")
    else:
        print("    VERDICT : shunt=0 mais full>0 — incohérent (canned devrait passer)")
        pytest.fail("shunt=0 mais full>0 — canned devrait au moins matcher full")


def test_bridge_vs_ipc_radio_chain(modes):
    """Bridge.py legacy vs osmo-trx-ipc moderne : quel chemin marche mieux ?"""
    f = modes.get("full", {}).get("fbsb_conf", 0)
    b = modes.get("bridge", {}).get("fbsb_conf", 0)
    print(f"\n  Radio chain compare : full(ipc)={f}  bridge(py)={b}")
    if f > b:
        print("    VERDICT : IPC > bridge.py")
    elif b > f:
        print("    VERDICT : bridge.py > IPC (régression suspecte)")
    else:
        print("    VERDICT : équivalents")


def test_lost_frames_consistency(modes):
    """LOST frames par mode — instabilité timer.

    Devrait être faible (<1000) sur des runs courts ; sinon instabilité.
    """
    print("\n  LOST frames per mode :")
    high = []
    for mode_name in MODE_INVARIANTS.keys():
        lost = modes.get(mode_name, {}).get("lost", 0)
        marker = "✗" if lost > 1000 else "✓"
        print(f"    {marker} {mode_name:12s} = {lost}")
        if lost > 1000:
            high.append((mode_name, lost))
    if high:
        print(f"    NOTE : {len(high)} mode(s) avec LOST > 1000 — instabilité possible")


def test_errors_quiet(modes):
    """Aucun mode ne devrait avoir d'erreurs/panic dans qemu.log."""
    print("\n  Errors per mode :")
    for mode_name in MODE_INVARIANTS.keys():
        q = modes.get(mode_name, {}).get("err_q", 0)
        m = modes.get(mode_name, {}).get("err_m", 0)
        marker = "✗" if (q > 0 or m > 0) else "✓"
        print(f"    {marker} {mode_name:12s} qemu={q}  mobile={m}")


# ---- Verdict engine : différentes conclusions possibles ----
#
# Chaque verdict examine les indicateurs et tire une conclusion explicite
# avec ses évidences. Tous les verdicts s'exécutent ; celui qui matche
# le plus d'évidences est mis en exergue dans le rapport final.

VERDICTS = []

def _verdict(name, evidences, conclusion, next_steps):
    """Helper : enregistre un verdict s'il matche."""
    matched = [e for e, ok in evidences if ok]
    if matched:
        VERDICTS.append({
            "name": name,
            "matched": matched,
            "conclusion": conclusion,
            "next_steps": next_steps,
            "weight": len(matched),
        })


def test_verdict_dsp_wall_confirmed(modes):
    """Verdict A : DSP était le mur."""
    s = modes.get("shunt", {})
    f = modes.get("full", {})
    _verdict("DSP_WALL_CONFIRMED",
        evidences=[
            (f"shunt.fbsb_conf={s.get('fbsb_conf',0)} > 0",
             s.get("fbsb_conf", 0) > 0),
            (f"full.fbsb_conf={f.get('fbsb_conf',0)} == 0",
             f.get("fbsb_conf", 0) == 0),
            (f"shunt.shunt_disp={s.get('shunt_disp',0)} > 0",
             s.get("shunt_disp", 0) > 0),
        ],
        conclusion="Le DSP émulé bloque FBSB. Le shunt canned débloque la chaîne ARM, "
                   "ce qui prouve que tout le chemin en aval (l1s, gsm322, l23) est sain.",
        next_steps=[
            "Phase 2 : brancher osmo-trx-ipc en source de vérité I/Q",
            "Implémenter shunt_dispatch_allc pour passer le mobile au-delà de FBSB",
            "Auditer le c54x emulator pour identifier l'opcode/timer responsable",
        ])


def test_verdict_arm_bug(modes):
    """Verdict B : bug côté ARM, pas DSP."""
    s = modes.get("shunt", {})
    _verdict("ARM_BUG",
        evidences=[
            (f"shunt.fbsb_conf={s.get('fbsb_conf',0)} == 0",
             s.get("fbsb_conf", 0) == 0),
            (f"shunt.shunt_disp={s.get('shunt_disp',0)} > 0",
             s.get("shunt_disp", 0) > 0),
            (f"shunt.shunt_latch={s.get('shunt_latch',0)} > 0",
             s.get("shunt_latch", 0) > 0),
        ],
        conclusion="Le shunt dispatch canné mais FBSB_CONF ne tire pas — "
                   "le bug est côté ARM (read path, scheduler, ou consumer).",
        next_steps=[
            "Auditer prim_fbsb.c:404 (read d_fb_det path)",
            "Vérifier r_page_used / r_page flip dans sync.c",
            "Tracer l1s_fbdet_resp scheduling",
        ])


def test_verdict_pipeline_works(modes):
    """Verdict C : tout passe."""
    s = modes.get("shunt", {})
    f = modes.get("full", {})
    _verdict("PIPELINE_OK",
        evidences=[
            (f"shunt.fbsb_conf={s.get('fbsb_conf',0)} > 0",
             s.get("fbsb_conf", 0) > 0),
            (f"full.fbsb_conf={f.get('fbsb_conf',0)} > 0",
             f.get("fbsb_conf", 0) > 0),
        ],
        conclusion="Les deux chemins (DSP réel et shunt) convergent — "
                   "FBSB est résolu. Focus sur les couches supérieures.",
        next_steps=[
            "Vérifier BCCH / SI3 reception",
            "Tester RACH → IMM_ASS_CMD chain",
            "LU end-to-end test",
        ])


def test_verdict_radio_chain_broken(modes):
    """Verdict D : la chaîne radio (IPC ou bridge) est cassée."""
    f = modes.get("full", {})
    s = modes.get("shunt", {})
    _verdict("RADIO_CHAIN_BROKEN",
        evidences=[
            (f"full.ipc_ok={f.get('ipc_ok',0)} == 0",
             f.get("ipc_ok", 0) == 0),
            (f"full.bts_ok={f.get('bts_ok',0)} == 0",
             f.get("bts_ok", 0) == 0),
            (f"shunt.fbsb_conf={s.get('fbsb_conf',0)} > 0 (mock OK)",
             s.get("fbsb_conf", 0) > 0),
        ],
        conclusion="La chaîne radio (calypso-ipc-device + osmo-trx-ipc + BTS) "
                   "ne démarre pas en full mode. Le mock fonctionne donc l'ARM/QEMU "
                   "est sain, le problème est dans les composants externes.",
        next_steps=[
            "Vérifier cfgs/osmo-trx-ipc.cfg (chan count, rx-path, tx-path)",
            "Vérifier IPC handshake : DDEV ERROR dans osmo-trx-ipc.log",
            "Vérifier que calypso-ipc-device est compilé et exécutable",
        ])


def test_verdict_instability(modes):
    """Verdict E : instabilité timer/icount."""
    high_lost = [m for m in MODE_INVARIANTS
                 if modes.get(m, {}).get("lost", 0) > 5000]
    _verdict("INSTABILITY",
        evidences=[
            (f"modes with LOST > 5000 : {high_lost}",
             len(high_lost) > 0),
        ],
        conclusion="LOST frames élevés — instabilité du timer TPU/TDMA. "
                   "Soit jitter host trop élevé, soit icount mal réglé.",
        next_steps=[
            "Forcer CALYPSO_ICOUNT=auto et désactiver MTTCG",
            "Profiler le host CPU (autres process gourmands)",
            "Vérifier calypso_trx.c timer scheduling",
        ])


def test_verdict_bridge_vs_ipc(modes):
    """Verdict F : régression bridge.py vs osmo-trx-ipc."""
    f = modes.get("full", {}).get("fbsb_conf", 0)
    b = modes.get("bridge", {}).get("fbsb_conf", 0)
    _verdict("BRIDGE_BETTER_THAN_IPC",
        evidences=[
            (f"bridge.fbsb_conf={b} > full.fbsb_conf={f}", b > f and f == 0),
        ],
        conclusion="bridge.py legacy converge alors que la chaîne IPC moderne ne converge pas. "
                   "Régression dans osmo-trx-ipc / calypso-ipc-device ou config.",
        next_steps=[
            "Diff fonctionnel entre bridge.py FN-rewrite et calypso-ipc-device",
            "Re-tester avec icount=off pour exclure timing",
        ])


# ---- Payload coverage matrix ----
#
# Quelles "payloads" (= types de tâches DSP) ont été exercées par mode.
# On utilise les compteurs shunt_disp/shunt_latch comme proxy (en mode
# shunt) et les indicateurs radio (en modes full/bridge).
PAYLOAD_MATRIX = {
    "FB_detect":   {"key": "fbsb_conf", "needs": {"full", "shunt", "shunt-ipc", "bridge"}},
    "SB_decode":   {"key": "fbsb_conf", "needs": {"full", "shunt", "shunt-ipc", "bridge"}},
    "Mock_LATCH":  {"key": "shunt_latch", "needs": {"shunt", "shunt-ipc"}},
    "Mock_DISP":   {"key": "shunt_disp",  "needs": {"shunt", "shunt-ipc"}},
    "RR_EST_REQ":  {"key": "rr_est",      "needs": {"full", "shunt-ipc", "bridge"}},
    "BTS_alive":   {"key": "bts_ok",      "needs": {"full", "shunt-ipc", "bridge"}},
    "IPC_handsh":  {"key": "ipc_ok",      "needs": {"full", "shunt-ipc"}},
}


@pytest.mark.parametrize("payload,spec",
                         list(PAYLOAD_MATRIX.items()),
                         ids=list(PAYLOAD_MATRIX.keys()))
def test_payload_coverage(modes, payload, spec):
    """Pour chaque payload : combien de modes l'ont exercé ?"""
    key = spec["key"]
    needs = spec["needs"]
    covered = {m for m in needs if modes.get(m, {}).get(key, 0) > 0}
    missing = needs - covered
    print(f"\n  Payload {payload} ({key}) : covered={covered}  missing={missing}")
    # Pas d'assert dur — c'est un rapport de couverture.


# ---- Rapport final consolidé ----

def test_zz_final_report(modes):
    """Affiche le tableau cross-mode complet en fin de pytest."""
    print("\n\n========== RUN-ALL CROSS-MODE REPORT ==========")
    header = ["mode", "fbsb_conf", "shunt_lat", "shunt_disp",
              "rr_est", "lost", "err_q", "err_m", "bts_ok", "ipc_ok"]
    rows = []
    for mode_name in MODE_INVARIANTS.keys():
        m = modes.get(mode_name, {})
        row = [
            mode_name,
            m.get("fbsb_conf", 0),
            m.get("shunt_latch", 0),
            m.get("shunt_disp", 0),
            m.get("rr_est", 0),
            m.get("lost", 0),
            m.get("err_q", 0),
            m.get("err_m", 0),
            m.get("bts_ok", 0),
            m.get("ipc_ok", 0),
        ]
        rows.append(row)

    widths = [max(len(str(h)), max((len(str(r[i])) for r in rows), default=0))
              for i, h in enumerate(header)]
    fmt = "  " + "  ".join(f"{{:<{w}}}" for w in widths)
    print(fmt.format(*header))
    print("  " + "  ".join("-" * w for w in widths))
    for r in rows:
        print(fmt.format(*[str(x) for x in r]))
    print("===============================================\n")

    # Verdicts ranking
    if VERDICTS:
        print("\n========== VERDICTS (ranked by evidence) ==========")
        VERDICTS.sort(key=lambda v: -v["weight"])
        for i, v in enumerate(VERDICTS, 1):
            mark = "★" if i == 1 else " "
            print(f"\n  {mark} [{i}] {v['name']} (weight {v['weight']})")
            print(f"      Evidence :")
            for e in v["matched"]:
                print(f"        • {e}")
            print(f"      Conclusion : {v['conclusion']}")
            print(f"      Next steps :")
            for s in v["next_steps"]:
                print(f"        → {s}")
        print("\n===================================================\n")
    else:
        print("\n  (no verdict matched — pas d'évidence forte dans les indicateurs)\n")
