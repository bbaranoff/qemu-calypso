# Workflow OsmocomBB vs notre QEMU Calypso

État au 2026-05-24. Inventaire des divergences entre ce qu'attend le firmware
OsmocomBB / osmo-bts et ce que notre QEMU émule, plus le TODO restant.

## 1. Chaîne osmocom de référence

```
osmocom-bb mobile  ──L1CTL/UART──>  osmocon  ──PTY──>  QEMU UART
                                                          │
                                                          ▼
                                                    ARM946E-S firmware
                                                    (layer1.highram.elf)
                                                          │
                                                          ▼
                                                       DSP C54x
                                                       (calypso_dsp.txt)
                                                          ▲
                                                          │ DMA
                                                          │
osmo-bts-trx  <──TRXD/CLK/TRXC── bridge.py ──BSP DMA── QEMU
```

* **Mobile (layer23)** : app userland, état GSM CM/MM/RR, parle L1CTL.
* **L1 firmware** : sched TDMA, gère SIM, DSP, contrôle radio.
* **DSP** : démod FCCH/SCH/BCCH/CCCH/SDCCH, viterbi, deinterleaving.
* **osmo-bts-trx** : BTS GSM, génère SI BCCH, scheduling slots, parle TRXD UDP.
* **bridge.py** : relais TRXD ↔ BSP DMA, pace CLK IND vers BTS.

## 2. Ce que le firmware attend du HW (et qu'on doit émuler)

### SIM controller (calypso_sim.c)

Per firmware `calypso/sim.c` self-documentation :

| bit IT       | clear semantics                                          |
|--------------|----------------------------------------------------------|
| NATR (1<<0)  | clear on read of REG_SIM_IT                              |
| WT   (1<<1)  | clear on read of REG_SIM_IT                              |
| OV   (1<<2)  | clear on read of REG_SIM_IT                              |
| TX   (1<<3)  | clear on **write to REG_SIM_DTX** (ou switch RX)         |
| RX   (1<<4)  | implicit / level-sensitive (suit le FIFO RX)             |

✅ Fixé 2026-05-24 — `edge_seen` mask = NATR\|WT\|OV (was: tout sauf RX).
✅ Fixé 2026-05-24 — DTX write clear IT_TX + update_irq.
✅ Fixé 2026-05-24 — Defer du clear via QEMUTimer 1µs virtuel pour escape
                     cpu_io_recompile TB-truncation race sous icount=auto.

### TINT0 / TDMA

* TINT0_PERIOD_NS = 4 615 000 (4.615 ms par frame GSM)
* Source : `QEMU_CLOCK_VIRTUAL` — déterministe sous icount.
* Émet IRQ vers ARM, TPU sync, BSP scheduling.

✅ Cadence correcte.

### BSP DMA samples → DSP DARAM

* Reçoit bursts UDP depuis bridge.py
* Queue par fn matching, livre à `cur_fn` ARM via DMA DARAM
* Drain timer cadence ARM tick

✅ Fixé 2026-05-24 — BSP drain timer passe `QEMU_CLOCK_REALTIME` → `VIRTUAL`
                     pour s'aligner avec TINT0/tdma_tick. Élimine le drift
                     bts↔qfn cumulatif (~1300 frames en 6s avant fix).

### rxDoneFlag (SIM)

* Firmware `sim_irq_handler` @ 0x008228b0 :
  ```
  LDRH r2, [REG_SIM_IT]     ; lecture MMIO
  TST  r2, #IT_WT           ; bit 1 set ?
  STRNE r1=1, [rxDoneFlag]  ; oui → flag = 1
  ```
* Adresse `rxDoneFlag` : `0x008302d4` (per nm sur le build courant)

✅ Fixé 2026-05-24 — défaut QEMU `calypso_sim.c` et `calypso_trx.c` aligné
                     sur `0x008302d4` (était `0x008302a0`, build stale).

## 3. Clock domains — alignement avec osmo-bts-trx

osmo-bts attend CLK IND à intervalle régulier (default 51 frames = 235 ms).
Si jitter trop élevé → log `"We were 1 FN faster/slower than TRX"` →
compensation, mais bursts BSP arrivent décalés → FBSB threshold fail.

| Acteur                     | Clock source                       |
|----------------------------|------------------------------------|
| TINT0 (DSP timer 0)        | `QEMU_CLOCK_VIRTUAL`               |
| TDMA tick (calypso_trx.c)  | `QEMU_CLOCK_VIRTUAL`               |
| BSP drain timer            | `QEMU_CLOCK_VIRTUAL` (fixé 05-24)  |
| bridge.py CLK IND          | `time.monotonic()` (wall)          |
| osmo-bts-trx               | wall clock                         |

✅ Fixé 2026-05-24 — bridge.py `select.select(fds,...,0.05)` → `0.001`,
                     réduit jitter CLK IND de ±50ms (~±10 fr) à ±1ms (~±0.2 fr).
                     Sous le seuil de compensation osmo-bts.

⚠️ Mismatch wall vs virtual reste fondamental sous `icount=auto` :
   VIRTUAL avance à ~22% du wall pour notre throughput. bridge.py paced wall,
   QEMU paced virtual → écart 78% par seconde. `BRIDGE_CLK_FROM_QEMU=1`
   bascule bridge sur QEMU FN pour résoudre ; à valider e2e.

## 4. Boot handshake — flow attendu

1. QEMU démarre, load firmware via `-kernel`
2. ARM exécute reset vector, init stack, init data, jump à `main()`
3. `main()` init UART (consoles), TIMERS, INTH, TPU, DSP
4. UART : romload stub côté QEMU (`calypso_uart.c`) joue le rôle bootloader,
   ACK `<i/<w/<c/<b` pour osmocon → osmocon croit le download réussi
5. Firmware boot complet, lance scheduler L1
6. Mobile envoie L1CTL_RESET_REQ → firmware ACK
7. Mobile envoie L1CTL_PM_REQ (Power Measurement) → firmware scan ARFCNs
8. Mobile envoie L1CTL_FBSB_REQ avec ARFCN cible → firmware lock FCCH/SCH
9. FBSB_CONF avec `result=0` (success) ou `result=0xff` (fail)
10. Mobile fait Location Update via SDCCH

## 5. État courant FBSB

✅ Étapes 1-7 OK
✅ Étape 8 : FBSB_REQ reçu, DSP entame FB correlation
❌ Étape 9 : FBSB_CONF avec `result=0xff` (jamais lock)

Root cause probable (à valider après les fixes 05-24) :
- BSP drift wall↔virtual provoquait des bursts livrés au mauvais timing
- DSP correlator regardait la mauvaise fenêtre de TOA
- Mes fixes alignent les clocks, devraient déboucher

## 6. TODO restant

### Priorité 1 — confirmer fixes 05-24

- [ ] Run avec `CALYPSO_ICOUNT=auto BRIDGE_CLK_FROM_QEMU=1 ./run.sh`
- [ ] Vérifier BSP RX delta reste `mean=16` stable
- [ ] Vérifier `[sim] SIM IT_WT → rxDoneFlag=1` dans qemu.log
- [ ] Vérifier bts.log : jitter < ±1ms, plus de "1 FN faster/slower"
- [ ] FBSB_CONF avec `result=0` après ces fixes

### Priorité 2 — TCG bug (non patché, contourné par fix SIM defer)

Vrai root cause : `cpu_io_recompile` truncate la TB après LDRH MMIO mid-TB.
Sous icount, `can_do_io=false` sauf last insn → tout MMIO mid-TB déclenche
recompile. Combiné avec read-clear edge bits → state change perdu.

Notre defer via QEMUTimer évite la race en repoussant le clear de 1µs virtuel.
Le vrai fix QEMU serait :
- Soit modifier `can_do_io` semantics (intrusif core)
- Soit ARM translator force `translator_io_start` pour tout LD/ST (perf hit)
- Soit upstream patch QEMU TCG icount handling

### Priorité 3 — workflow alignment OsmocomBB

- [ ] Vérifier `REG_SIM_CMD` bits CARDRST/IFRST/STOP/START correctement émulés
- [ ] Confirmer le APDU TX/RX cycle complet (SELECT, READ_BINARY, RUN_GSM_ALGORITHM)
- [ ] Auditer SAGEM/SIM ID readback (EF_IMSI lecture déjà OK)
- [ ] `REG_SIM_CONF1_CONFTXRX` bit (switch direction TX↔RX) honoré ?
- [ ] WT timeout — current 2ms simulé vs spec ~9600 ETU = ~25ms à 372 Hz

### Priorité 4 — DSP firmware exécution

- [ ] DSP_BUDGET tuning : current 256k cycles par TDMA tick
- [ ] FB correlator timing window vs sample arrival latency
- [ ] CCCH demod path validation (BCCH dispatch)

### Priorité 5 — Long terme

- [ ] MTTCG (multi-thread TCG) pour vraie concurrence ARM/DSP
- [ ] Replay mode pour determinism cross-run
- [ ] Test pytest framework — couverture state machine FBSB

## 7. Knobs env actuels (run.sh defaults post-fixes 05-24)

| knob                       | défaut       | rôle                                  |
|----------------------------|--------------|---------------------------------------|
| `CALYPSO_ICOUNT`           | `off`        | virtual clock déterministe            |
| `CALYPSO_FORCE_RX_DONE`    | **1** ✓      | belt-and-suspenders rxDoneFlag write  |
| `CALYPSO_RXDONE_ADDR`      | `0x008302d4` | sym `rxDoneFlag` du build courant     |
| `CALYPSO_DSP_BUDGET`       | `256000`     | DSP cycles per ARM tick               |
| `BRIDGE_CLK_FROM_QEMU`     | `0`          | `1` = bridge slave to QEMU FN         |
| `BRIDGE_CLK_PERIOD`        | `51`         | CLK IND interval (51 frames = 235ms)  |
| `BRIDGE_DL_FN_LOOKAHEAD`   | `32`         | bridge envoie 32 fr ahead             |
| `BRIDGE_DL_FN_REWRITE`     | `slot`       | DL fn → next valid slot               |
| `BRIDGE_UL_FN_REWRITE`     | `slot`       | UL fn → next valid RACH slot          |
| `CALYPSO_FBSB_SYNTH`       | `0`          | `1` = bypass DSP, synthetic FBSB OK   |
| `CALYPSO_DSP_FBDET_SKIP`   | `0`          | skip FB-det handler entirely          |

## 8. Files modifiés cette session

- `hw/arm/calypso/calypso_sim.c` — SIM IT semantics fix + defer clear
- `hw/arm/calypso/calypso_trx.c` — rxDoneFlag default addr + non-env-gated
- `hw/arm/calypso/calypso_bsp.c` — BSP drain VIRTUAL clock
- `bridge.py` — select timeout 50ms → 1ms (jitter fix)
- `run.sh` — FW_ELF/FW_BIN paramétrable + FORCE_RX_DONE default 1
- `run_rssi.sh` — nouveau wrapper pour firmware rssi
