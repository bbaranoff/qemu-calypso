# Threading TODO — QEMU Calypso

> Sur silicium réel, chaque unité tourne en **parallèle physique**. Notre QEMU
> les sérialise dans le main TCG thread → timing artefactuel → bugs apparents
> qui sont en réalité des artefacts d'ordonnancement. **300+ "bugs ARM" passés
> étaient majoritairement induits par ce mismatch.**
>
> Ce doc liste chaque composant à mettre dans son propre thread, par ordre
> logique (dépendances + risque). État au 2026-05-24.

## 0bis. Carte IRQ Calypso (source : osmocom-bb `include/calypso/irq.h`)

```
IRQ  src              → ARM handler                  → réveille
─────────────────────────────────────────────────────────────────────
 0   WATCHDOG         → reset                        — boot only
 1   TIMER1           → timer1_irq                   — generic timing
 2   TIMER2           → timer2_irq                   — generic timing
 3   TSP_RX           → tsp_rx_irq                   — RF receive notif
 4   TPU_FRAME        → frame_irq → l1s scheduler    — TDMA frame tick *
 5   TPU_PAGE         → tpu_page_irq                 — TPU scenario end
 6   SIMCARD          → sim_irq_handler → rxDoneFlag — SIM RX/WT/OV
 7   UART_MODEM       → uart_irq_handler_sercomm     — osmocon L1CTL  *
 8   KEYPAD_GPIO      → kbd_irq                      — keys
 9   RTC_TIMER        → rtc_irq                      — clock
10   RTC_ALARM_I2C    → rtc_alarm                    — alarm
11   ULPD_GAUGING     → battery_gauge                — battery
12   EXTERNAL         → ext_irq                      — board GPIO
13   SPI              → spi_irq                      — IOTA/RF SPI bus
14   DMA              → dma_irq                      — BSP DMA done  *
15   API              → api_irq                      — DSP↔ARM mailbox *
16   SIM_DETECT       → sim_detect                   — card insert
17   EXTERNAL_FIQ     → ext_fiq (FIQ mode)           — high-prio ext
18   UART_IRDA        → uart_irq_handler_cons        — debug console
19   ULPD_GSM_TIMER   → ulpd_timer                   — power management
20   GEA              → gea_irq                      — A5/3 crypto
```

`*` = chemins critiques pour FBSB / L1CTL :
- **TPU_FRAME (4)** : TPU fire chaque 4.615 ms, ARM `frame_irq` réveille l1s
  scheduler qui programme les tasks DSP (FB, NB, SB) pour la prochaine frame.
- **UART_MODEM (7)** : osmocon envoie L1CTL_FBSB_REQ → UART RX IRQ →
  `uart_irq_handler_sercomm` parse → dispatche au L1CTL handler.
- **DMA (14)** : BSP fini de livrer le burst en DARAM → ARM peut programmer
  la prochaine task. *(actuellement non câblé QEMU → BSP livre, DSP lit
  directement, ARM ne sait pas)*
- **API (15)** : DSP signale via mailbox que sa task est done → ARM peut
  lire results (snr/toa/ang). *(actuellement non câblé non plus)*

## 0ter. Chaîne de threading proposée (qui réveille qui)

```
                                          ┌─── IRQ_TPU_FRAME (4) ───┐
                                          │                          │
   tpu_thread ──tick TDMA virtuel─────────┘                          ▼
       │                                                       ARM thread
       │     ┌─── IRQ_DMA (14) ────────────────────────────────┐  (frame_irq
       │     │                                                  │   handler,
       ▼     │   ┌─── IRQ_API (15) ────────────────────────────┤   schedules
   bsp_thread ───┘   ┌─── IRQ_SIMCARD (6) ────────────────────┤   l1s tasks)
       │             │                                          │
       ▼             ▼                                          │
   DARAM        sim_thread                                      │
   (locked)         │                                           │
       ▲            ▼                                           │
       │       sim FIFO + IT_WT                                 │
       │                                                        │
   dsp_thread ◄────TPU "EN" signal (ARM commits next tasks)─────┘
       │
       ▼  écrit results dans DARAM + raise IRQ_API
       │
       └───── back to ARM ─────┘
                                          UART_MODEM (7)
                                                ▲
                                                │
                                          osmocon (host)
                                                ▲
                                                │
                                          mobile app L23
```

**Synchronisation** :
- TPU_FRAME = barrier matérielle pour tous les threads
- Mailbox API RAM = signal DSP-done → ARM
- DMA IRQ = signal BSP-done → ARM (à implémenter QEMU)
- UART_MODEM = signal mobile→ARM (asynchrone, decoupled timing)

## 1. Architecture cible (mirror du silicium Calypso)

```
                    BQL / sync barriers @ TDMA frame boundaries
                              (4.615 ms virtual)
   ┌────────┬───────┬───────┬───────┬───────┬────────┬─────────┐
   │  ARM   │  DSP  │  BSP  │  TPU  │ INTH  │  IOTA  │   SIM   │
   │  946   │ C54x  │  DMA  │ sched │  IRQ  │ TWL3025│  ctrl   │
   │ thread │thread │thread │thread │thread │ thread │ thread  │
   └────────┴───────┴───────┴───────┴───────┴────────┴─────────┘
        │       │       │       │       │       │         │
        └───────┴───────┴───────┴───────┴───────┴─────────┘
                  partage DARAM / API RAM / MMR
                       (locks fins requis)
```

Bridge.py côté hôte est **déjà threadé** (commit 2026-05-24) :
- `_tick_thread` (qemu_clk_sock)
- `_trxc_thread` (BTS control)
- `_stats_thread` (logs périodiques)
- main = trxd pur (burst DL/UL critical path)

## 1. Ordre logique (dépendances + risque croissant)

### Phase 1 — Composants autonomes sans I/O partagé critique

#### [ ] 1.a. SIM controller thread (LOW RISK)
- **Pourquoi d'abord** : déjà émulé avec timers VIRTUAL, peu de shared state
- **État courant** : main TCG thread + QEMUTimer pour ATR/WT/edge-clear
- **Refactor** : QemuThread dédié qui pompe la FIFO RX et fire IT bits
- **Shared state** : `s->it` register + IRQ line (via qemu_irq_*)
- **Locks** : `QemuMutex sim_lock` sur `s->it` + FIFO read/write
- **Sync points** : IRQ delivery vers INTH (atomic via qemu_set_irq)
- **Gain attendu** : élimine la race cpu_io_recompile résiduelle (déjà
  contournée par defer timer, mais devient natif)

#### [ ] 1.b. IOTA (TWL3025) thread (LOW RISK)
- **Pourquoi** : vraie puce séparée, lien SPI bidirectionnel
- **État courant** : presque inexistant dans QEMU, à vérifier (twl3025.c ?)
- **Refactor** : thread qui répond aux commandes SPI sans bloquer ARM
- **Shared state** : registres TWL3025 (audio, ABB control)
- **Locks** : `QemuMutex iota_lock`
- **Pre-req** : grep qemu-src pour voir l'état actuel de l'IOTA emul

### Phase 2 — DSP CPU autonome (MEDIUM-HIGH RISK)

#### [ ] 2.a. DSP TMS320C54x thread
- **Pourquoi maintenant** : root cause des 300 bugs apparents, max impact
- **État courant** : `c54x_run(BUDGET)` appelé 2× depuis `calypso_tdma_tick`
  (défini `calypso_trx.c:1618`) — appels `c54x_run` à `calypso_trx.c:1713`
  (dsp_n_exec_2) et `calypso_trx.c:1799` (dsp_n_exec_5), sérialisés dans le
  main TCG ARM thread. *(Deux autres appels `c54x_run` existent à `L195` et
  `L637` mais dans d'autres fonctions, hors tick TDMA.)*
- **Refactor** :
  - `QemuThread dsp_thread` qui boucle `c54x_run(SMALL_BUDGET)`
  - tdma_tick retire les 2 appels c54x_run inline
  - Communication via shared `dsp->idle` flag et IRQ pulses
- **Shared state** :
  - DARAM (`s->dsp->data[0..0x27FF]`) — accessible ARM via OVLY
  - API RAM (`s->dsp->api_ram[0..0x7FF]`) — mailbox ARM↔DSP
  - MMR (`s->dsp->sp/ar[]/imr/ifr`) — DSP-local mais lu par instrumentation
- **Locks** :
  - `QemuMutex daram_lock` (DARAM)
  - `QemuMutex api_ram_lock` (mailbox)
  - IRQ ARM↔DSP via `qemu_set_irq` (déjà atomic)
- **Sync points** :
  - TDMA tick = sync barrier (DSP attend ARM finit frame, ou inverse)
  - DSP IDLE = release point pour ARM scheduling
- **icount break** : MTTCG incompatible icount=auto.
  Soit on accepte (passe wall-paced), soit on garde un compteur instr
  virtuel partagé synchronisé aux TDMA boundaries.
- **Validation** : `tests/test_osmocom_workflow.py::osmocom_clock` doit
  rester vert, + nouveau test "ARM et DSP avancent en parallèle"

### Phase 3 — BSP DMA thread (MEDIUM RISK)

#### [ ] 3.a. BSP burst delivery thread
- **Pourquoi** : BSP est hardware DMA continu, autonome ARM+DSP
- **État courant** : BSP drain timer sur VIRTUAL clock (fix 2026-05-24),
  callback dans main thread via `bsp_drain_cb`
- **Refactor** :
  - QemuThread dédié qui pompe la BSP queue → DARAM writes
  - Plus de QEMUTimer drain — boucle thread native
- **Shared state** :
  - DARAM zone target (0x3fb0..) écrite par BSP, lue par DSP
  - `BspBurstSlot` queue (bridge→BSP UDP buffer)
- **Locks** :
  - `QemuMutex bsp_queue_lock`
  - `QemuMutex daram_lock` (partagé avec DSP thread Phase 2)
- **Sync points** :
  - frame boundary (synchronise avec DSP qui consomme les samples)
  - BSP signale "burst ready" via IRQ ou daram[fn] flag
- **Risque** : si DARAM lock contended pendant correlation DSP →
  perf hit. Mitigation : double-buffering DARAM (page 0/1 split,
  déjà partiellement géré par `s->dsp_page`)

### Phase 4 — TPU + INTH threads (LOW-MEDIUM RISK)

#### [ ] 4.a. TPU thread
- **Pourquoi** : Time Processing Unit gère scénarios TPU autonomes
- **État courant** : timer émulation dans calypso_trx.c
- **Refactor** : QemuThread pour scenario playback (TPU_CTRL_EN, etc.)
- **Shared state** : `tpu_regs[]`, IRQ lines vers ARM+DSP
- **Locks** : `QemuMutex tpu_lock`

#### [ ] 4.b. INTH thread (optionnel)
- **Pourquoi** : ce sont les INTH navigation lancées par UART qui forment
  le timing chain — user note 2026-05-24 "ce sont les inth qui naviguent
  lancées par l'uart"
- **État courant** : INTH dans main TCG, dispatch IRQs synchrone
- **Refactor** : possible mais INTH est juste un dispatcher rapide,
  threading peut over-engineer. Évaluer après Phase 2/3.

### Phase 5 — TCG MTTCG flag global (BIG)

#### [ ] 5.a. `-accel tcg,thread=multi`
- **Pré-requis** : Phases 1-4 stables, locks audités
- **Effet** : ARM TCG en thread propre QEMU-natif (déjà supporté upstream)
- **Coût** : icount=auto définitivement cassé
- **Bénéfice** : ARM tourne réellement en parallèle avec tous les threads
  périphériques. Plus de "DSP attend que ARM finisse son tick".

## 2. Locks plan global

```
QemuMutex  daram_lock       (DARAM 0x0000-0x27FF)        — DSP/BSP/ARM(OVLY)
QemuMutex  api_ram_lock     (API mailbox 0x0800-0x0FFF) — ARM/DSP
QemuMutex  sim_lock         (SIM controller it/fifo)    — SIM/ARM
QemuMutex  iota_lock        (TWL3025 registers)         — IOTA/ARM
QemuMutex  bsp_queue_lock   (BSP UDP queue)             — BSP/bridge
QemuMutex  tpu_lock         (TPU registers + scenarios) — TPU/ARM
```

IRQ lines (`qemu_irq`) sont déjà thread-safe via `qemu_set_irq` atomic.

## 3. Sync barriers (TDMA frame = unité d'ordre)

```
TDMA tick (4.615ms virtuel) :
  1. TPU thread arme scenarios DL → BSP queue
  2. BSP thread livre bursts à DARAM (avant fenêtre DSP)
  3. DSP thread démod (FB/SB/CCCH/SDCCH) — parallèle ARM
  4. ARM thread lit results post-IRQ
  5. ARM thread écrit tasks pour next frame
  6. → next tick
```

Sync points naturels :
- frame_irq (ARM) = "frame N done, start N+1"
- DSP IDLE = "DSP frame work done"
- TPU_CTRL_EN = "ARM committed next-frame tasks"

## 4. Tests à ajouter après chaque phase

- [ ] `test_threading_no_deadlock_60s` : run 60s, vérifier que tous les
      threads avancent (pas de starvation, pas de deadlock)
- [ ] `test_threading_daram_consistency` : grep DARAM-WR/RD log pour
      détecter écritures concurrentes sur même cellule sans lock
- [ ] `test_threading_irq_delivery_atomic` : pas de perte IRQ entre
      threads (chaque RAISE → propagé à l'ARM)
- [ ] `test_threading_arm_dsp_parallel` : compteur insn ARM et insn DSP
      avancent **simultanément** (pas sériellement)

## 5. Risques + mitigations

| risque | mitigation |
|---|---|
| Deadlock ARM↔DSP (ARM attend rxDoneFlag, DSP attend IRQ ARM) | ordre canonique des locks + try_lock timeout |
| icount=auto cassé (perte déterminisme) | accepter wall-paced, ou compteur virtuel partagé sync TDMA |
| Race DARAM corruption (DSP read pendant BSP write) | mutex daram_lock OU double-buffering page |
| Perte IRQ entre threads | qemu_irq atomic (déjà géré) |
| Perf hit si lock contention | profilage + scope reduction des sections critiques |
| Tests pytest cassent (timing-sensitive) | adapter seuils, marquer `xfail_under_mttcg` |

## 6. Référence — comment osmocom (real silicium) handle ça

Sur vrai Calypso :
- ARM 13 MHz, DSP 39 MHz (3× ARM rate)
- BSP DMA continu, latence ~10 µs
- TINT0 fire à 217 Hz = sync barrier matérielle
- API RAM mailbox = boîtes aux lettres atomiques par mot
- DSP `IDLE` instruction = release CPU jusqu'à next IRQ

Notre QEMU doit refléter ce modèle. Le **sériel** d'aujourd'hui est la cause
racine de la majorité des "ARM bugs" passés — ARM voit du state DSP qui
n'aurait jamais été visible en réel (parce qu'en réel le DSP aurait déjà
écrit la valeur attendue avant que l'ARM relise).

## 7. Décision prochaine session

Choisir l'ordre d'attaque :
1. **Phase 1.a (SIM)** — quick win, peu de risque, valide le pattern
2. **Phase 2.a (DSP)** — max impact mais haut risque
3. **Phase 3.a (BSP)** — natural pair avec DSP, après DSP stable

Recommandation : 1.a → 3.a → 2.a → 5.a (MTTCG global en dernier).
2.a en dernier car DSP est le composant le plus complexe et c'est ARM/DSP
qui change le plus de runtime behavior.
