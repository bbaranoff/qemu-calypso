# Plan — corriger les timings divergents (icount-related)

> Objectif : tout le pipeline Calypso (QEMU + osmocon + bridge + bts-trx + mobile
> + irda_capture + tcpdump) doit fonctionner sous `CALYPSO_ICOUNT=auto` (ou
> `shift=N,sleep=on`) **et** `=off`, sans perte d'IrDA, sans freeze SIM, sans
> drift CLK IND.

---

## Symptômes observés (état au 2026-05-16 soir)

| # | Symptôme | Mode où ça casse | Mode OK |
|---|---|---|---|
| T1 | `fw-irda.log = 0 bytes` malgré `qemu-irda-tx.raw = 1+ MB` | `icount=auto` | `icount=off` |
| T2 | Firmware boucle dans `calypso_sim_powerup+0x54` sur `rxDoneFlag` | `icount=auto` (sans hack) | `off` ou hack `CALYPSO_FORCE_RX_DONE=1` |
| T3 | `bridge: QEMU tick` rare (1 vs ~217/s attendu) | `icount=auto` partiel | `off` |
| T4 | Drift bridge.qfn vs qemu.fn > 43k frames (`test_bridge_qfn_tracks_qemu_fn` FAIL) | les deux modes | — |
| T5 | osmo-bts-trx `bts_shutdown_fsm clock skew too high` au-delà de PERIOD=51 | `icount=off` peut amplifier | équilibre `BRIDGE_CLK_PERIOD=51` |

---

## Causes-racines hypothétiques

### C1 — PTY backend QEMU drop bytes sous icount

`qemu_chr_fe_write_all(&chr, &ch, 1)` pour chardev pty utilise `write(fd_master, ...)` non-blocking. Sous icount=auto, QEMU rate-limit selon insn count : les writes côté master arrivent en bursts désynchronisés du wall-clock. Quand le kernel buffer PTY se remplit et que personne ne lit côté slave en temps réel, `write()` retourne `EAGAIN`. QEMU **ne retry pas** → bytes perdus silencieusement. `qemu_chr_fe_write_all` retourne `0` mais le log côté QEMU (`uart_log_raw`) écrit avant l'appel donc `qemu-irda-tx.raw` montre l'intention, pas le succès réel.

### C2 — SIM ISR commit conditionnel cassé sous icount

`sim_irq_handler` côté firmware exécute un `STRNE` (store-conditional) pour set `rxDoneFlag=1` après ATR reçu. Sous icount=auto, le TCG conditional execution mid-MMIO race avec `cpu_io_recompile` → le STR ne commit pas. Hack `CALYPSO_FORCE_RX_DONE` workaround mais c'est sale. Vrai fix : root-causer le bug TCG (Task #29 mentionnée dans le code).

### C3 — Timer wall-clock vs virtual diverge

`tdma_timer` (QEMU_CLOCK_VIRTUAL) avance avec icount auto, mais `kick_cb` (QEMU_CLOCK_REALTIME) avance en wall. Sous load, l'écart se creuse → CLK IND batches manqués. Le `bridge: QEMU tick` ne reçoit que les CLK INDs qui sortent effectivement.

### C4 — DSP exec budget mal calibré sous icount

`dsp_n_exec_2/5` (instructions DSP par tdma_tick) consomment ~512k insn × wall 4.6 ms cible. Sous icount=auto, le compte se traduit en virtual time qui dérive du wall si la machine host est lente.

---

## Plan d'action (par priorité décroissante)

### Phase A — Stabiliser le canal IrDA sous icount (priorité 1)

#### A.1 — Patch QEMU : retry blocking sur write PTY master
Modifier `calypso_uart.c` autour des `qemu_chr_fe_write_all(&s->chr, &ch, 1)` :
```c
int written = 0, tries = 0;
while (written < 1 && tries++ < 100) {
    int n = qemu_chr_fe_write(&s->chr, &ch, 1);
    if (n > 0) { written = n; break; }
    if (n == 0 || n == -EAGAIN) usleep(100); /* yield */
    else break;
}
if (written < 1) {
    qemu_log_mask(LOG_GUEST_ERROR,
                  "[UART:%s] TX byte dropped (chardev full)\n", s->label);
}
```
Coût : ~100 µs max par byte perdu. Acceptable vs perdre tout l'IrDA.

#### A.2 — Alternative : remplacer pty backend par socket UNIX
`run.sh` crée un socket unix (`/tmp/uart_irda.sock`), QEMU lance avec
`-chardev socket,id=irda,path=/tmp/uart_irda.sock,server=on,wait=off
-serial chardev:irda`. `irda_capture.py` connecte au socket (lecture
streaming, pas de termios PTY weird).

Avantage : socket POSIX = pas de PTY kernel buffer limit (4-8 KB), pas de
termios canonical mode à désactiver, FIFO unix-style fiable.

### Phase B — Stabiliser le SIM polling sous icount (priorité 2)

#### B.1 — Documenter et garder le hack `CALYPSO_FORCE_RX_DONE`
Déjà en place. `CALYPSO_RXDONE_ADDR` auto-détecté via `nm` au launch de
`run.sh`. **C'est la situation actuelle**, marche en pratique.

#### B.2 — Root-cause TCG STRNE skip
Investigation longue : ouvrir gdb watchpoint sur `rxDoneFlag` (`0x008302d4`
au build courant) et tracer chaque write. Identifier le TB qui contient le
STRNE et voir pourquoi il ne commit pas sous icount. **Probablement upstream
QEMU bug** — ouvrir ticket plus tard, pas bloquant.

### Phase C — Stabiliser TDMA / CLK IND (priorité 3)

#### C.1 — Garder `BRIDGE_CLK_FROM_QEMU=0` (default actuel)
Wall-paced CLK IND est insensible aux drifts icount. C'est en place,
testé, marche. Pas besoin de modif.

#### C.2 — Patch `calypso_kick_cb` pour mesurer drift vt vs rt
Déjà partiellement instrumenté : `[kick] fire #N vt=... rt=...`. Ajouter
un compteur drift_max et un alert si drift > seuil.

### Phase D — Test invariants (priorité 4)

#### D.1 — Étendre `test_firmware_state.py`
Ajouter :
- `test_irda_throughput_under_icount_auto` : run avec `CALYPSO_ICOUNT=auto`,
  capture 10 s, attend ≥ 10 beacons. Si fail → A.1 ou A.2 pas appliqué.
- `test_tdma_tick_vs_kick_drift` : `[kick] fire vt/rt` drift < 50 ms
  cumulative sur 60 s.

#### D.2 — Nouvelle couche pipeline `Timing-icount`
Dans `_PIPELINE_ORDER`, ajouter `"Timing-icount"` entre `Timers` et `Drift`.
Marker `runtime_icount`. Diagnostique précisément ce qui drift.

### Phase E — Documentation (priorité 5)

#### E.1 — README run.sh
Section « Modes icount » :
- `off` (default) : full wall-clock, déterministe sur idle, fiable pour
  l'IrDA et SIM, ratio insn/wall variable selon host CPU.
- `auto` : déterministe instructions, **break IrDA sans patch A.1/A.2**,
  break SIM sans `CALYPSO_FORCE_RX_DONE=1`.

#### E.2 — Mémo dans `tests/conftest.py`
Le marker `runtime_irda` doit skip si `icount=auto` ET A.1/A.2 pas appliqué
(detect via existence d'un compteur QEMU stub).

---

## Ordre suggéré d'implémentation

1. **A.2** (socket unix UART_IRDA) — fix robuste, ~30 lignes côté QEMU +
   ~10 lignes run.sh + irda_capture.py
2. **D.1** (tests icount=auto) — gate de régression
3. **C.2** (kick drift log) — instrumentation pour root-cause future
4. **B.2** (TCG STRNE) — fond, pas bloquant, **après** A et D
5. **E** (docs) — fin

Effort estimé : 4-6 h pour A.2+D.1, 1 j pour B.2.

---

## Décisions ouvertes

- A.1 vs A.2 : socket unix (A.2) plus propre que retry blocking (A.1) ?
- Faut-il garder le hack `CALYPSO_FORCE_RX_DONE` une fois B.2 résolu ? Oui,
  car il dépend du build firmware courant — utile comme fallback.
- Le test `test_irda_throughput_below_saturation` doit-il être adapté pour
  socket unix (pas de saturation 115200) ? Oui, sample 100 KB/s OK.
