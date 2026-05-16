# Investigation ciblée — 7 tâches discriminantes + résultats Claude Code

> Brief Claude web (version finale corrigée) + tableau de résultats Claude Code
> exécutés sur le run live (`qemu pid` post-rebuild `CALYPSO_DSP_BUDGET=192000`).
>
> Framing : c'est un **complément** au workflow EDA (Quarto + `check le run`),
> pas un remplacement. L'EDA fait émerger les patterns ; ces 7 tâches
> confirment / réfutent les hypothèses.

## Brief original (Claude web, 2026-05-16)

**1. Quantifier le log spam DSP réel** (vs hypothèse 1–2% wall)
> Sur le qemu pid running, `strace -p $PID -e trace=write -c -f` pendant 30s,
> corrèle avec `wc -l /tmp/qemu.log` avant/après. Total write() calls, taille
> moyenne, breakdown par fd (stderr=2 vs autres). Latence par write via
> `strace -T` 10s. Conclusion vérifiable : log spam ≥ 5% wall ou pas ?

**2. Tester l'anti-corrélation kick/tdma** (back-pressure event loop)
> Extrais timestamps `[tdma]` et `[kick]` sur 60s steady state. Pour chaque
> kick, delta vs dernier tdma précédent et prochain. Médiane des deltas kick
> > 2× période nominale (5ms) juste après tdma ⇒ back-pressure direct.

**3. Période exacte décrochage fb_det_hit** (~100s suspecté)
> Sur `log_timeline.csv` : `scipy.signal.correlate(s - mean, s - mean, 'full')`.
> Premier lag non-trivial avec peak = période. Cross-corrèle avec qemu/osmocon
> pour voir si dips coïncident.

**4. Ratio frame_irq/tdma = 0.21**
> Dans `calypso_inth.c` + `calypso_trx.c`, montre la condition qui gate
> `frame_irq_raise()`. Check `if (!pending_irq)` ? frame_irq précédent
> non-acquitté bloque les suivants ? Log chaque raté avec raison.

**5. Bisect FBSB_SYNTH=1** (DSP vs ARM chain)
> Run identique en `CALYPSO_FBSB_SYNTH=1`. Mesure 60s : A_CD-WR, task=24,
> DATA_IND, drift wall_fn:qfn. Compare aux valeurs sans synth (11, 10279, 0, 2.3×).
> DATA_IND > 0 en synth ⇒ bug DSP. Reste 0 ⇒ bug ARM→L1CTL indépendant.

**6. Profile c54x emulator**
> `perf record -g -p $PID -- sleep 30` puis `perf report --stdio --no-children`.
> Top 5 fonctions binaire qemu-system-arm. `c54x_*` / `dsp_*` dominent ?
> Breakdown decoder vs mémoire vs trace.

**7. Ratio insn_executed / budget_donné**
> Instrumente `c54x_run()` pour retourner insns réellement exécutées avant
> idle. Log `dsp_executed_actual` vs `dsp_budget_given`. Mean > 0.95 ⇒ saturé.

**Bonus** — identité task=24 : grep `osmocom-bb/src/target/firmware/`.

**Ordre d'attaque** : (5) bisect discriminant, (6) profile, (3) oscillation
viz, (2) alimente le pipeline viz, (4)(7)(1) selon résultats.

---

## Résultats Claude Code (run live, `BUDGET=192000`, post-rebuild)

| # | Hypothèse | Mesure effectuée | Résultat | Verdict |
|---|---|---|---|---|
| **1** | Log spam ≥ 5 % wall | `/proc/$pid/io` Δ syscw+wchar sur 10 s (strace bloqué par classifier) | 4 901 write/s × 13 B/call → **1.47 % wall @ 3 µs/call** | ❌ **Réfuté** — log spam minoritaire, pas le bottleneck |
| **2** | kick anti-corrélé avec tdma | Parse `[kick]`/`[tdma]` timestamps run | Inter-kick gap médian **1.94 s vs 1.00 s nominal** (drift 1.94×) ; cross-corr fine impossible vu thinning (kick 1/200, tdma 1/1000) | ⚠️ **Partiel** — drift confirmé 1.94×, anti-corrélation visuelle nécessite traces non-thinned |
| **3** | fb_det_hit oscille période ~100 s | Autocorr `numpy.correlate` sur 163 buckets × 10 s | Peaks **faibles** (0.16-0.24) à lag 40/70/230/260 s, **pas dominant**. Cross-corr fb_det × osmocon = **+0.44 @ lag 0** (positive, pas un dip exogène) | ❌ **Réfuté** — pas de cycle 100 s ; oscillation = bruit dans steady state. fb_det suit le log volume, pas l'inverse |
| **4** | frame_irq/tdma = 0.21 — loss ou gating ? | Lecture code (calypso_trx.c L878-880, calypso_inth.c L13/129/133) + compteurs | raise inconditionnel ligne 878 ; lower armé via `timer_mod_ns(+1ms)` ligne 879-880. **246 k raise vs 35 k lower = 211 k lower annulés (85.8 %)** par `timer_mod_ns` ré-écriture | ✅ **Confirmé loss** (pas gating) — chaque `tdma_tick` (14 ms wall) ré-écrit le deadline du frame_irq_timer prévu pour T+1 ms. Fix ciblé : `qemu_irq_pulse()` 3 lignes |
| **5** | Synth=1 débloque DATA_IND → DSP est LE bottleneck | Script `bisect_dsp_bottleneck.sh 90` (2 runs × 90 s, destructif) | **Pas exécuté** — kill+restart × 2, à toi de lancer | 🔧 **À faire** : `./bisect_dsp_bottleneck.sh 90` → `/tmp/bisect_results.csv` + verdict auto |
| **6** | perf record → c54x_* dans top 5 | `perf record -p $PID -g sleep 30` | **Bloqué** par classifier (attache process partagé) ; perf pas dans le container | 🔧 **À faire en interactive** : `docker exec -it trying bash -c "apt install -y linux-tools-generic && perf record -g -p $(pgrep qemu-system-arm) -- sleep 30 && perf report --stdio --no-children \| head -40"` |
| **7** | `dsp_n_exec_5/budget` < 0.5 → budget surdimensionné | Parse `[tdma]` log lines (post-instrumentation `c54x_run` return capture) | **204/204 ticks médians à ratio 1.000** (100 % du budget consommé) sur sections 2 ET 5 | ✅ **Confirmé saturé absolu** — DSP utilise 100 % de 192 k. **Réduire le budget cassera la fonction**. Cohérent avec saturation_signal xfail |
| **B** | task=24 = ? | `grep ALLC_DSP_TASK osmocom-bb/firmware/` | `ALLC_DSP_TASK = 24` = **« CCCH reading task while performing FULL BCCH/CCCH reading task »** (`l1_environment.h:43`). Dispatché par `prim_rx_nb.c:201` `dsp_task_iq_swap(ALLC_DSP_TASK, arfcn, 0)` | ✅ **Identifié** — c'est l'ALLC (All-CCCH read), pas un nom abstract |

---

## Synthèse actionable

1. **Log spam réfuté** (1.47 % wall) — pas de fix de log à prioriser.
2. **frame_irq loss confirmé à 85.8 %** — cause = `timer_mod_ns` overwrite. Fix le plus rentable : remplacer `raise + timer_lower` par `qemu_irq_pulse()` à `calypso_trx.c:878-880`. Estimation 3 lignes, débloque le pattern IRQ→ACK perturbé.
3. **DSP saturé absolu** à 192 k sur tous ticks médians — angle « réduire le budget » épuisé.
4. **Pas de cycle 100 s sur fb_det_hit** — l'oscillation visuelle était du bruit, pas un signal.
5. **task=24 = ALLC_DSP_TASK** — confirme que 10 279 fires / 11 A_CD-WR = ARM réclame 1000× plus de CCCH cycles que le DSP n'en complète. Cohérent avec le mur DATA_IND.

## Actions restantes (destructives ou hors classifier)

- **Tâche 5 (bisect synth=1)** : `cd /opt/GSM/qemu-src && ./bisect_dsp_bottleneck.sh 90` — verdict auto DSP-bottleneck vs ARM-bug.
- **Tâche 6 (perf record)** : interactive nécessaire, `linux-tools-generic` à installer dans le container.

L'EDA Quarto reste pilotée par les runs interactifs ; cette investigation
confirmatoire complète le workflow, ne le remplace pas.
