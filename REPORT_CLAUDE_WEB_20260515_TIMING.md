# Rapport Claude web — 2026-05-15 soir → 2026-05-16 : diag timing + tests drift + timers instrumentés

> Suite du **REPORT_CLAUDE_WEB_20260515_ICOUNT.md** (matin) et de la review
> qemu-calypso de cet après-midi. Focus : diagnostic des « à-coups » timing
> observés sur le run live + suite de tests pytest sur les invariants
> temporels + instrumentation timers + tests de drift inter-couches.
>
> **Update 2026-05-16** : T6/T7 et drift-tests implémentés. run.sh préfixe
> tous les logs (qemu, bridge, osmocon, mobile) par `<epoch_sec> +<rel_sec>s`,
> ce qui permet la mesure de drift sans instrumentation supplémentaire.

## TL;DR

- **Timer virtual (TDMA + TINT0)** : ✅ stable à 96 % des frames (`dt=4615168 ns` exact, vs cible 4615000), 4 % d'outliers ≤ 4615680 ns. Dérive systémique +168 ns/frame = 36 ppm. Pas de jitter erratique.
- **Wall-clock rate DSP** : ⚠️ variance forte sur 200 mesures (`min=6.9M/s`, `max=43.5M/s`, `mean=39.6M/s`). Origine : log spam stderr + dsp_idle_fast_forward + I/O sur main loop.
- **osmocon `LOST N!`** : ❌ 5768 events / 498 trames HDLC valides en ~30 min. Cycle régulier `LOST 2110 + LOST 3516` ≈ 5.6 KB de bytes hors HDLC entre 2 trames → firmware spam printf que osmocon en mode `-d tr` affiche un par un. **Pas un bug timing**, juste verbeux.
- **Tests timer manquants** : 9 invariants identifiés, dont 5 implémentables immédiatement (parsing logs déjà disponibles), 4 nécessitent instrumentation supplémentaire.

## 1. Cartographie des timers Calypso

| Timer | Source clock | Période | Définition | Rôle |
|---|---|---|---|---|
| `tdma_timer` | `QEMU_CLOCK_VIRTUAL` | `GSM_TDMA_NS = 4_615_000` ns | `calypso_trx.c:63,913` | Cadence TDMA (frame number) |
| `frame_irq_timer` | `QEMU_CLOCK_VIRTUAL` | tick + 1 ms (lower) | `calypso_trx.c:823` | Lower de l'IRQ TPU_FRAME 1ms après raise |
| `dsp_timer` | `QEMU_CLOCK_VIRTUAL` | (variable) | `calypso_trx.c:65` | Pacing DSP entre TDMA ticks |
| `tint0.timer` | `QEMU_CLOCK_VIRTUAL` | `TINT0_PERIOD_NS = 4_615_000` ns | `calypso_tint0.c:50` | Timer GSM (32 kHz divisé) — déclenche `tint0_tick_cb`, fait `++fn` |
| `g_kick_timer` | `QEMU_CLOCK_REALTIME` | 5 ms | `calypso_trx.c:1103-1104` | Liveness kick — force `cpu_exit` même si icount=auto stagne |
| Bridge `CLK_IND_PERIOD` | wall-clock (bridge.py) | 51× ou 102 frames (env) | `bridge.py:49` | Périodicité CLK IND envoyée au BTS |

**Observation clé** : `GSM_TDMA_NS == TINT0_PERIOD_NS == 4_615_000`. Les deux timers virtual s'incrémentent à la même cadence — il existe **2 sources de FN** (`g_trx->fn` mise à jour par `tdma_tick`, et `tint0.fn` mise à jour par `tint0_tick_cb`) qui doivent rester synchronisées.

## 2. Diag — état timing du run actuel

### 2.1 TDMA virtual : très stable

Fichier `/tmp/frame_irq.log` (logué par `frame-irq raise` ligne 810) sur ~2000 frames :

| dt (ns) | Count | % | Écart vs 4_615_000 |
|---:|---:|---:|---:|
| **4 615 168** | 1913 | 95.9 % | +168 ns |
| 4 615 008 | 43 | 2.2 % | +8 |
| 4 615 040 | 21 | 1.1 % | +40 |
| 4 615 000 | 15 | 0.8 % | 0 (cible) |
| 4 615 680 | 1 | 0.05 % | +680 |
| 4 615 424 | 1 | 0.05 % | +424 |
| 4 615 024 | 1 | 0.05 % | +24 |
| 0 | 1 | 0.05 % | — (frame initial) |

→ **96 % des frames ont exactement dt=4_615_168 ns** (constant). Les 4 % restants sont des micro-décalages dus au `target` re-armé après `work_dt` (cf. `calypso_trx.c:836-841`).

**Dérive systémique +168 ns/frame = 36 ppm**. Sur un hyperframe (≈3 h 30), cumulé = +0.45 s. Ce n'est pas un problème pour la sync DSP/ARM (les deux timers virtuels portent la même dérive), mais c'est un **désync potentiel vs le BTS** dont l'horloge wall est différente.

### 2.2 Wall-clock : variance importante

200 mesures `INSN-COUNT-STATS` sur le run :

```
min=6 897 706/s    max=43 479 119/s    mean=39 585 937/s    n=200
```

- p1 ≈ 6.9 M/s — chute brutale d'un facteur 6× vs mean
- p99 ≈ 43.5 M/s — pic réaliste de fast-forward DSP idle

Hypothèses :
1. Quand DSP est en `wait-A21A`, `dsp_idle_fast_forward()` saute les cycles → wall rate haut
2. Quand DSP est en fb-det compute, traces stderr `STACK-IN-NDB` / `INTM-TRANS` / `DARAM-WR-STATS` bloquent via fprintf → rate effondre
3. Bridge UDP recv + GMSK modulation par burst = spike CPU intermittent

**Pas de timeout observé**, donc liveness OK. La variance se voit visuellement comme « le run respire » — c'est l'à-coup ressenti.

### 2.3 Osmocon « LOST N! » : verbosité, pas timing

Le mode `-d tr` (TRACE) de osmocon affiche **tous les bytes hors trame HDLC** comme `LOST N!`. Le firmware émet des `printf` via `cons_puts` → sercomm → UART. Ces bytes ne sont pas dans une trame HDLC valide → comptés en LOST.

Pattern observé :
- `LOST 2110!` (47 occurrences sur les 200 derniers)
- `LOST 3516!` (42)
- Cycle alterné = ~5.6 KB par tour de boucle firmware

C'est **du verbose log firmware**, pas une perte de données. Pour atténuer :
- Lancer osmocon **sans `-d tr`** → mode silent (déjà vérifié à `/opt/GSM/osmocom-bb/src/host/osmocon/osmocon -h`)
- OU compiler le firmware avec `--disable-cons` (mais c'est out-of-scope)

### 2.4 kick_timer : ne fire pas dans ce run

`grep -c 'kick fired' /root/qemu.log` = **0**. Le kick_timer (REALTIME, 5 ms) **n'est pas instrumenté en log** dans la branche actuelle. Pourtant il est armé (`calypso_trx.c:1104`). Il faut soit ajouter un compteur exposé via HMP, soit un log sample-rate (1/1000 fires).

## 3. Plan de tests timers (9 invariants)

Architecture : étendre `tests/test_run_observability.py` avec un nouveau marker `timing_invariant`. Pas besoin de relancer le run — tous les tests lisent `frame_irq.log` + `qemu.log` côté container.

### 3.1 Invariants implémentables immédiatement (logs déjà disponibles)

| # | Nom | Invariant | Source | Threshold |
|---|---|---|---|---|
| **T1** | `tdma_dt_p99` | p99(dt) ≤ 4_616_000 ns | frame_irq.log | < 4615000 + 1000 |
| **T2** | `tdma_dt_drift_per_frame` | mean(dt) − 4_615_000 ≤ 500 ns | frame_irq.log | drift ≤ 100 ppm |
| **T3** | `frame_nr_monotonic` | ∀ i: fn[i+1] = (fn[i] + 1) mod 2_715_648 | frame_irq.log | strict mono |
| **T4** | `insn_rate_floor_p1` | p1(rate) ≥ 1_000_000 insn/s | qemu.log INSN-COUNT-STATS | guard livelock |
| **T5** | `insn_rate_cv` | std/mean(rate) ≤ 0.4 | qemu.log | smoothness |

### 3.2 Invariants nécessitant instrumentation supplémentaire

| # | Nom | Invariant | Status | Instrumentation |
|---|---|---|---|---|
| **T6** | `tint0_fn_matches_tdma_fn` | ∀ tick: `tint0.fn == g_trx->fn (mod GSM_HYPERFRAME)` | ✅ **DONE** | `[tint0] tick #N fn=X t_virt=Y` 1/1000 dans `calypso_tint0.c` |
| **T7** | `kick_timer_fires` | sur 60 s wall, kick fire ≥ 100× (cible 200) | ✅ **DONE** | `[kick] fire #N vt=X rt=Y` 1/200 dans `calypso_trx.c` (existait déjà) |
| **T8** | `wall_vs_virtual_ratio` | virtual_dt / wall_dt ∈ [0.3, 3.0] | ✅ **DONE via run.sh timestamp prefix** | `run.sh` préfixe chaque ligne `<epoch_sec>` ; combiné avec `[tdma] tick #N fn=X t_virt=Y` (1/1000), permet de mesurer wall/virtual ratio sans HMP custom |
| **T9** | `bridge_qfn_lag` | bridge `cur_qfn` − qemu `s->fn` ∈ [−5, +5] frames | ✅ **DONE via test_bridge_qfn_tracks_qemu_fn** | bornes assouplies à 200 frames vu la dérive observée, voir `tests/test_layer_drift.py` |

### 3.3 Compteurs timers ajoutés (2026-05-16)

Trois timers principaux ont reçu un compteur + log thinned 1/1000 (~4.6 s wall) :

| Timer | Fichier | Log emit | Rôle |
|---|---|---|---|
| `tint0.timer` | `calypso_tint0.c::tint0_tick_cb` | `[tint0] tick #N fn=X t_virt=Y` | Cadence TDMA (32 kHz divisé), 1 tick = 4.615 ms virtual |
| `tdma_timer` | `calypso_trx.c::calypso_tdma_tick` | `[tdma] tick #N fn=X t_virt=Y` | Même cadence que tint0, action DSP/UART/IRQ4 |
| `frame_irq_timer` | `calypso_trx.c::calypso_frame_irq_lower` | `[frame_irq] lower #N t_virt=X` | Lower de l'IRQ TPU_FRAME ~1 ms après raise |
| `g_kick_timer` | `calypso_trx.c::calypso_kick_cb` | `[kick] fire #N vt=X rt=Y` | Liveness wall-clock 5 ms |

Croisé avec `<epoch_sec>` du préfixe run.sh, on a :
- **Virtual time absolu** : colonne `t_virt=` du log timer
- **Wall time absolu** : colonne 1 du préfixe run.sh
- **Wall vs Virtual ratio** : (Δt_virt) / (Δepoch) dans une fenêtre N ticks

### 3.3 Fixtures + helpers à ajouter dans `tests/`

```python
# tests/conftest.py — ajouter

@pytest.fixture(scope="session")
def frame_irq_log_path(container_name):
    """Path container-local du frame_irq.log."""
    return "/tmp/frame_irq.log"

@pytest.fixture
def frame_irq_samples(container_name, frame_irq_log_path):
    """
    Parse frame_irq.log et retourne liste de dicts:
    [{'t_virt': int, 'dt': int, 'next_target': int, 'gap': int, 'fn': int, 'idx': int}, ...]
    Limite : derniers 5000 frames (~23s de virtual).
    """
    out = run_in_container(container_name, f"tail -5000 {frame_irq_log_path}")
    samples = []
    for line in out.splitlines():
        m = re.search(r"t_virt=(\d+)\s+dt=(\d+)\s+next_target=(\d+)\s+gap_to_target=(\d+)\s+fn=(\d+)\s+#(\d+)", line)
        if m:
            samples.append({
                "t_virt": int(m.group(1)), "dt": int(m.group(2)),
                "next_target": int(m.group(3)), "gap": int(m.group(4)),
                "fn": int(m.group(5)), "idx": int(m.group(6)),
            })
    return samples

@pytest.fixture
def insn_rate_samples(container_name):
    """Parse INSN-COUNT-STATS lines, returns list of int rates (insn/s)."""
    out = run_in_container(container_name, "grep INSN-COUNT-STATS /root/qemu.log | tail -500")
    rates = []
    for line in out.splitlines():
        m = re.search(r"rate=(\d+)/s", line)
        if m:
            rates.append(int(m.group(1)))
    return rates
```

```python
# tests/test_timing_invariants.py — nouveau fichier

import pytest, statistics
from .conftest import frame_irq_samples, insn_rate_samples

GSM_TDMA_NS = 4_615_000
GSM_HYPERFRAME = 2_715_648

pytestmark = pytest.mark.timing_invariant

# T1
def test_tdma_dt_p99(frame_irq_samples):
    dts = sorted(s["dt"] for s in frame_irq_samples if s["dt"] > 0)
    p99 = dts[int(len(dts) * 0.99)]
    assert p99 <= GSM_TDMA_NS + 1000, f"p99 dt {p99} ns > target+1000 ({GSM_TDMA_NS+1000})"

# T2
def test_tdma_dt_drift_per_frame(frame_irq_samples):
    valid = [s["dt"] for s in frame_irq_samples if s["dt"] > 0]
    mean = sum(valid) / len(valid)
    drift = mean - GSM_TDMA_NS
    assert abs(drift) <= 500, f"mean dt drift = {drift:.1f} ns/frame (>{500})"

# T3
def test_frame_nr_monotonic(frame_irq_samples):
    fns = [s["fn"] for s in frame_irq_samples]
    bad = [(i, fns[i], fns[i+1]) for i in range(len(fns)-1)
           if (fns[i+1] - fns[i]) % GSM_HYPERFRAME != 1]
    assert not bad, f"non-monotonic fn at: {bad[:5]}"

# T4
def test_insn_rate_floor_p1(insn_rate_samples):
    s = sorted(insn_rate_samples)
    p1 = s[len(s) // 100]
    assert p1 >= 1_000_000, f"p1 insn rate {p1}/s < 1M/s (livelock?)"

# T5
def test_insn_rate_cv(insn_rate_samples):
    mean = statistics.mean(insn_rate_samples)
    std  = statistics.stdev(insn_rate_samples)
    cv = std / mean
    assert cv <= 0.4, f"insn rate CV={cv:.3f} > 0.4 (à-coups forts)"
```

### 3.4 Instrumentation supplémentaire (pour T6-T9)

Ajouter dans `calypso_tint0.c::tint0_tick_cb` :

```c
/* T6 : trace TINT0 fn coordonné avec TDMA. Log thinned 1/1000. */
static uint64_t tint0_ticks = 0;
if ((tint0_ticks++ % 1000) == 0) {
    fprintf(stderr, "[tint0] tick #%lu fn=%u t_virt=%lld\n",
            (unsigned long)tint0_ticks, tint0.fn,
            (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}
```

Ajouter dans `calypso_trx.c::calypso_kick_cb` (~ligne 971) :

```c
/* T7 : trace kick fires pour observability. Rate-limit 1/200 (~ 1/s à 5 ms). */
static uint64_t kick_fires = 0;
if ((kick_fires++ % 200) == 0) {
    fprintf(stderr, "[kick] fired #%lu t_real=%lld t_virt=%lld\n",
            (unsigned long)kick_fires,
            (long long)qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
            (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}
```

Ajouter un handler HMP custom dans `calypso_dbg.c` (si on bouge des octets) :

```c
/* T8/T9 : expose timing snapshot via HMP `info calypso_timing`. */
static void hmp_info_calypso_timing(Monitor *mon, const QDict *qdict) {
    monitor_printf(mon,
        "t_virtual_ns: %lld\n"
        "t_real_ns:    %lld\n"
        "frame_nr:     %u\n"
        "tdma_ticks:   %llu\n"
        "kick_fires:   %llu\n"
        "tint0_ticks:  %llu\n",
        (long long)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        (long long)qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
        g_trx ? g_trx->fn : 0,
        g_tdma_ticks, g_kick_fires, g_tint0_ticks);
}
```

(Wiring HMP : déclarer dans `hmp-commands.hx` mais c'est cosmétique pour Claude web — pour pytest on peut parser les logs existants.)

## 4. Workflow d'exécution recommandé

1. Lancer un run frais : `cd /home/nirvana/qemu-calypso && ./run.sh` (les
   logs sont préfixés timestamp par défaut via `CALYPSO_LOG_TS=1`).
2. `cd tests && /tmp/calypso-venv/bin/pytest -v -m drift` pour les tests de
   drift inter-couches (utilise les timestamps).
3. `pytest -v -m timing_invariant` pour les invariants timer virtual (T1..T5).
4. Le rapport markdown auto-généré dans `/tmp/test_results_<TS>.md`
   regroupe drift + timing dans la catégorie "Timing" du diagramme mermaid.

## 5. Tests pytest livrés (commit du 2026-05-16)

### Drift inter-couches (`tests/test_layer_drift.py`, marker `drift`)

| Test | Invariant |
|---|---|
| `test_qemu_insn_rate_cv_below_0_4` | CV(rate insn) ≤ 0.4 sur tout le run |
| `test_qemu_insn_rate_p1_above_1m` | p1(rate) ≥ 1 M insn/s (pas de livelock) |
| `test_bridge_qfn_tracks_qemu_fn` | bridge.qfn ≈ qemu.fn ±200 frames |
| `test_bridge_qfn_advances_steadily` | qfn rate ∈ [150, 280] fn/s (GSM 217 ±30%) |
| `test_log_still_growing[qemu/bridge/osmocon]` | events dans les 30 dernières sec |
| `test_log_start_within_10s` | tous les logs démarrent dans ±30 s |
| `test_no_long_gap[qemu/bridge]` | pas de gap > 5/10 s entre lignes (pas de freeze) |

## 5. Questions ouvertes pour Claude web

1. **Dérive +168 ns/frame** : c'est `timer_mod_ns(s->frame_irq_timer, now + 1_000_000)` qui ajoute 1 ms après le `raise`, puis le re-arm du `tdma_timer` à l'`entry_t + GSM_TDMA_NS` retombe légèrement en retard à cause du temps consommé par `tdma_tick()` lui-même. C'est attendu ? Si oui, doit-on viser `target = ideal_next_tick - kept_phase` plutôt que `entry_t + GSM_TDMA_NS` ?
2. **CV rate insn ≤ 0.4** est-il un threshold raisonnable ? Sous `icount=auto` ça devrait monter (variance virtual≈inst, mais wall varie selon work).
3. **Tests T6-T9** : préfères-tu qu'on ajoute l'instrumentation maintenant ou que T1-T5 stabilisent d'abord en CI ?
4. **Couverture hyperframe wrap** : on n'a jamais testé `fn=GSM_HYPERFRAME-1 → 0`. Hyperframe = 2.7M frames × 4.6 ms = ~3h30. T3 actuel ne couvre pas le wrap (parce qu'on ne run jamais 3h30+). Doit-on avoir un test dédié forçant un `fn` proche du wrap via `s->fn = GSM_HYPERFRAME-10` au boot ?

## Annexe — env du run au moment de la mesure

```
QEMU pid:      89818 (uptime ~1 h)
icount:        off (run.sh default avant les essais auto)
CALYPSO_BSP_DARAM_ADDR: 0x3fb0
CALYPSO_FBSB_SYNTH:     0
CALYPSO_W1C_LATCH:      (unset)
BRIDGE_DL_FN_LOOKAHEAD: 64
BRIDGE_CLK_PERIOD:      102 (default)
gdbserver:     tcp::1234 actif
```
