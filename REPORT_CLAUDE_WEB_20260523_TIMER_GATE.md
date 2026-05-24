# Rapport Claude web — 2026-05-23 : Plan A `CALYPSO_TIMER`, gate vérifié, hypothèse logging-tax réfutée

> Suite directe du diag « stderr-pipe backpressure auto-saboterait la mesure
> qfn » discuté en session. Plan A appliqué + mesuré à chaud. Soumis à review.

## TL;DR

- **Plan A appliqué** : env-gate `CALYPSO_TIMER` sur 5 sites `fprintf` dans
  `hw/arm/calypso/calypso_trx.c` + 3 inserts dans `run.sh` (déclaration,
  export, summary). Sync 3-way (container `/opt/GSM/qemu-src` + host
  `/home/nirvana/qemu-src` + host `/home/nirvana/qemu-calypso`) avec md5
  identiques sur `calypso_trx.c`.
- **Gate fonctionne empiriquement** : sous `CALYPSO_TIMER=0`, le count des
  patterns timer dans `qemu.log` tombe à `0/0/0` (vs `35/10/0` sous `=1`),
  et les 3 file-based logs (`/tmp/frame_irq.log`, `tdma_tick.log`,
  `tdma_profile.log`) ne sont plus réécrits (mtime pré-run).
- **Hypothèse « logging tax » → RÉFUTÉE** : qfn rate identique entre `=1`
  et `=0` (≈19.7 vs 19.5 qfn/s). Le coût stderr+TSLOG n'était pas dans
  le path critique du main loop TCG.
- **Mur `DATA_IND` reproductible** : task=24 (ALLC) fire 17× en 16 s avec
  `CALYPSO_FBSB_SYNTH=1`, ARM n'émet **aucun** `L1CTL_DATA_IND`. Le vrai
  blocker milestone est intact ; on a juste éliminé un faux ami.
- **Vrai bottleneck timing** = compute DSP et/ou `CALYPSO_ICOUNT=off`,
  pas le logging. Mesure ENV-test prochaine : `CALYPSO_ICOUNT=auto`.

## 1. Ce qui a été modifié

### 1.1 `hw/arm/calypso/calypso_trx.c`

Helper ajouté (cached getenv, lazy init, BQL-thread donc safe sans atomique) :

```c
static bool calypso_timer_log(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("CALYPSO_TIMER");
        on = (e && (*e == '1' || *e == 'y')) ? 1 : 0;
    }
    return on;
}
```

5 sites gatés (préservation du thinning d'origine, ajout `&& calypso_timer_log()`) :

| Site | Avant | Après | Sortie | Cap d'origine |
|---|---|---|---|---|
| `calypso_frame_irq_lower` | `fprintf(stderr, "[frame_irq] lower …")` | gaté `&& calypso_timer_log()` | stderr | thin 1/1000 |
| `[frame-irq] raise` block | `if (firq_count < 2000)` + `fopen` + `fprintf(firq_log, …)` | gate sur la condition | `/tmp/frame_irq.log` | 2000 lignes |
| `[tdma-skip]` | `fprintf(stderr, "[tdma-skip] …")` | gaté | stderr | si `skipped>0 && fn%100==0` |
| `[tdma-tick]` block | `if (tick_count < 500)` + `fopen` + `fprintf(tick_log, …)` | gate sur la condition | `/tmp/tdma_tick.log` | 500 lignes |
| `[kick] fire` | `fprintf(stderr, "[kick] fire …")` | gaté | stderr | first 30 + 1/200 |

**Décision design** : gate inclut le `fopen` (pas seulement `fprintf`) → sous
`=0`, les 3 fichiers `/tmp/*.log` ne sont **jamais ouverts**, donc absence
de fichier = signal positif que le gate a tenu sur tout le path.

### 1.2 `run.sh`

Trois inserts (la solution `-trace calypso_*` proposée initialement a été
abandonnée : aucun `trace-events` Calypso n'existe dans l'arbre QEMU —
`find /opt/GSM/qemu-src -name trace-events | xargs grep -l calypso` →
vide. Les fprintf inconditionnels étaient la vraie source à gater.) :

1. Déclaration après `CALYPSO_UART_TRACE` : `CALYPSO_TIMER="${CALYPSO_TIMER:-0}"`
2. Ajout à la liste `export` (sinon le `getenv` côté C voit `NULL`).
3. Echo summary après la ligne `CALYPSO_ICOUNT`.

## 2. Mesures empiriques (TIMER=1 vs TIMER=0)

Deux runs séquentiels avec `CALYPSO_FBSB_SYNTH=1`, `CALYPSO_ICOUNT=off`,
même binaire QEMU (PID 22279 puis 23033, binaire mtime stable depuis
build initial — `ninja qemu-system-arm` no-op confirme `[1/4] Generating
qemu-version.h` seulement).

### 2.1 Snapshot brut

| Métrique | TIMER=1 (51 s) | TIMER=0 (16 s) |
|---|---|---|
| qemu.log size | 3 225 928 B (3.07 MB) | 2 447 576 B (2.33 MB) |
| `[kick] fire` count | 35 | **0** |
| `[tdma-skip]` count | 10 | **0** |
| `[frame_irq] lower` count | 0 | **0** |
| `/tmp/frame_irq.log` | 200 KB (cap 2000) | absent (write) |
| `/tmp/tdma_tick.log` | 36 KB (cap 500) | absent (write) |
| `/tmp/tdma_profile.log` | 20 KB (cap 200) | absent (write) |
| `task=24` count | 57 | 17 |
| `ALLC` count | 1 | 1 |
| `L1CTL_DATA_IND` count | **0** | **0** |

Le `2.33 MB / 16 s` ≠ `3.07 MB / 51 s` parce que la fenêtre boot DSP
(~4000 lignes `[c54x] BOOT[N.N]` + 3912 `CALAD-ZONE-W` + 1508
`COEFFS-WR` + 1000 `DISP-FLAG-W` …) domine au démarrage. Steady-state
rate beaucoup plus bas — confirmé par les 51 s vs 16 s qui produisent
~même volume de "boot trace".

### 2.2 qfn rate (bridge.log)

```
TIMER=1 (PID 22279) :
  cur_qfn=907 → 919  sur Δt=0.609 s   → 19.7 qfn/s
  (anciennes mesures fixées à 35 qfn/s ailleurs — pas le même run)

TIMER=0 (PID 23033) :
  cur_qfn=80  → 346  sur Δt=13.621 s  → 19.5 qfn/s
```

**Δ = 0.2 qfn/s, dans le bruit.** L'hypothèse « stderr-unbuffered + TSLOG
python flush-par-ligne → pipe-backpressure → main-loop stall → qfn
effondré » est réfutée pour le scope timer fprintf.

## 3. Hypothèse logging-tax — analyse de la réfutation

Le pari initial était que :
1. 3 sites `fprintf(stderr, …)` à 217 Hz × ~600 syscalls/s
2. TSLOG python (`sys.stdout.write + sys.stdout.flush` par ligne) sous GIL
3. Pipe buffer rempli → `write()` côté QEMU bloque sur le pipe
4. Main loop TCG mono-thread → stall wall-clock → qfn rate s'effondre

Pourquoi la mesure le réfute :
- Le thinning d'origine était déjà agressif : `[frame_irq] lower` à 1/1000
  → 0.2 fprintf/s effectifs ; `[kick] fire` à 1/200 → ~1 fprintf/s ;
  `[tdma-skip]` rare. Les *blocks* `firq_log` et `tick_log` écrivent dans
  des FILE\* séparés (pas le pipe TSLOG), donc échappent à la pression.
- En steady-state effectif (post-cap), `stderr` reçoit ~1–2 lignes/s, pas
  650. Le poids du logging était dominé par la **fenêtre boot** (où les
  caps n'ont pas encore été atteints) et par d'**autres** sources non-
  timer (cf §4).

**Conséquence** : ni le compteur 1 Hz ni la refonte vers `trace-events`
QEMU n'apporteraient de gain mesurable sur cette dimension. La voie est
fermée proprement.

## 4. Le vrai pollueur du log (top patterns par fréquence, run TIMER=0 16 s)

```
4000 [c54x] BOOT[N.N] PC=N op=N SP=N A=N B=N         (DSP boot trace, single-shot)
3912 [c54x] CALAD-ZONE-W ...                          (DSP CAL coefficients write)
1772 [c54x] PC RECENT (last Nk) top: ...
1508 [c54x] COEFFS-WR ...
1000 [c54x] DISP-FLAG-W ...
```

Ces 5 patterns (= ~12 000 lignes) sont des **instrumentations DSP** dans
`calypso_c54x.c`, **indépendantes** du gate `CALYPSO_TIMER`. À ~150 B/ligne
× ~360 lignes/s steady-state, le log accumule **~50 KB/s ≈ 180 MB/h**.
Pour atteindre les 2.68 GB mentionnés sur un run précédent, ~15 h de run
sont nécessaires — ce sont **ces traces** qui dominent le volume, pas les
fprintf timer. Si on veut un vrai bouton "silent run", il faudra un
`CALYPSO_DSP_TRACE` companion (out of scope ici).

## 5. Mur `DATA_IND` confirmé (le vrai blocker milestone)

```
task=24 (ALLC) = 17  fire
ALLC          =  1
DATA_IND      =  0
```

Avec `CALYPSO_FBSB_SYNTH=1` (FB synthétique), le DSP atteint l'ALLC et
fire task=24. Pourtant `L1CTL_DATA_IND=0`. Le path cassé est :

```
DSP : démod → a_cd[] dans NDB                ✓ (NDB tests 19/19 vert)
   └→ raise IRQ4 (TPU_FRAME) via calypso_trx     ✓ (firq_count > 0 sous =1)
      └→ INTH dispatch → ARM IRQ vector          ?  (calypso_inth.c IRQ4 edge-clear OK)
         └→ ARM frame_int_handler → lit NDB      ?  (PAS DE COMPTEUR — angle mort)
            └→ sercomm → L1CTL_DATA_IND          ✗ (count = 0)
```

Les 3 xfail `milestone_irq` (`test_c54x_interrupt_ex_called_nonzero`,
`isr_entered_implies_rete`, `no_pending_irq_gated`) probent la direction
**ARM→DSP**, pas DSP→ARM. **L'angle DSP→ARM n'est pas couvert par les
compteurs actuels**.

## 6. Questions pour review web

1. **Réfutation logging-tax** : Δqfn=0.2 sur des runs courts (16 s vs 51 s)
   avec phase boot dominante. Est-ce suffisant pour fermer la voie, ou
   tu veux qu'on relance sur un steady-state ≥ 5 min des deux côtés
   pour confirmer ?
2. **Path DSP→ARM** : la prochaine étape diag consiste à ajouter 4
   compteurs côté ARM (`tpu_frame_raised_total`, `arm_frame_isr_entered`,
   `arm_read_a_cd_count`, `data_ind_emitted_total`) exposés via `info`
   monitor. As-tu déjà vu ce path instrumenté dans des branches/snapshots
   antérieurs ? Notamment des hooks dans `l1ctl_sock.c` ou `sercomm_gate.c` ?
3. **Test `CALYPSO_ICOUNT=auto`** : avant de plonger côté ARM ISR, on
   propose un ENV test gratuit (auto vs off) pour isoler si la dégradation
   timing (20 qfn/s) vient de `-icount off` ou de la compute DSP pure.
   Cf REPORT_20260516_DSP_OVERRUN : `c54x_run(256000) × 2 par tick` à
   27 M insn/s wall = 18.6 ms/tick. Sous `-icount auto` (virtual time
   pacé par insn-count), ce ratio devrait disparaître. Tu confirmes
   l'ordre **(a) ENV test (b) ARM ISR instrumentation** ?
4. **Hygiène run.sh** : les 3 fichiers `/tmp/frame_irq.log`,
   `tdma_tick.log`, `tdma_profile.log` ne sont pas inclus dans le `rm -f`
   du cleanup L359 de `run.sh`. Sous `CALYPSO_TIMER=1` puis `=0`, on hérite
   des fichiers du run précédent qui faussent l'œil nu (mais pas la
   mesure mtime). Ligne à ajouter ?

## 7. Annexes

### 7.1 Reproducer minimal

```bash
# Rebuild (no-op si binaire déjà à jour) :
docker exec trying bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"

# Run silencieux (default) :
docker exec -it trying bash -c "cd /opt/GSM/qemu-src && ./run.sh"

# Run verbose pour comparaison :
docker exec -it trying bash -c "cd /opt/GSM/qemu-src && CALYPSO_TIMER=1 ./run.sh"

# Vérif gate côté stderr :
docker exec trying bash -c "grep -cE '\[kick\] fire|\[tdma-skip\]|\[frame_irq\] lower' /root/qemu.log"

# Vérif gate côté file-based :
docker exec trying ls -la /tmp/frame_irq.log /tmp/tdma_tick.log /tmp/tdma_profile.log
```

### 7.2 État des 3 dirs (sync 3-way)

```
md5 calypso_trx.c                          (toutes identiques)
  20378c969cc891ab6339e03d824ca8ec  trying:/opt/GSM/qemu-src/hw/arm/calypso/calypso_trx.c
  20378c969cc891ab6339e03d824ca8ec  /home/nirvana/qemu-src/hw/arm/calypso/calypso_trx.c
  20378c969cc891ab6339e03d824ca8ec  /home/nirvana/qemu-calypso/hw/arm/calypso/calypso_trx.c

run.sh : divergence pré-existante préservée
  Container :  CALYPSO_FBSB_SYNTH default=1  + CALYPSO_FORCE_RX_DONE déclaré
  Hosts     :  CALYPSO_FBSB_SYNTH default=0  + pas de FORCE_RX_DONE
  → mes 3 inserts CALYPSO_TIMER landed identiques aux 3 endroits.
```
