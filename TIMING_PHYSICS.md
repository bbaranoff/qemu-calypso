# Calypso Timing — équations physiques des liens

Référence hardware : TI Calypso SoC + DSP TMS320C54x + osmocom-bb firmware.
Toutes les valeurs viennent du silicium réel — c'est le modèle que notre
QEMU doit refléter.

## 1. Source maître

```
XTAL principal : 13.000 MHz (GSM-standard)

  ┌─ ARM946E-S clk        : 13 MHz  (1× xtal)
  ├─ DSP TMS320C54x clk    : 39 MHz  (3× xtal via PLL)
  ├─ BSP/TPU sample clk    : 13 MHz  (1× xtal)
  └─ Peripheral clk (UART) : 13/N    (divider per device)
```

Tous les ticks GSM dérivent du même xtal 13 MHz par division entière.

## 2. TDMA frame — l'unité fondamentale

```
GSM_BIT_DURATION  = 48/13 µs       = 3.6923077 µs/bit   (GSM 04.05)
BITS_PER_SYMBOL   = 1               (GMSK, 1 bit/symbol)
SYMBOLS_PER_BURST = 156.25          (148 useful + 8.25 guard)
BURST_DURATION    = 156.25 × 48/13  = 576.923 µs       ≈ 577 µs
TIMESLOTS_PER_FRAME = 8
TDMA_FRAME = 8 × 576.923           = 4.61538 ms       ≈ 4.615 ms
           = 8 × 156.25 × 48/13 µs
           = 60000/13 µs           (= exactement 60 000 / 13)
           = 60 000 × 13 MHz cycles
```

**Équivalence ARM cycles ↔ TDMA frame** :
```
1 TDMA frame = 4.61538 ms
             = 4.61538 ms × 13 MHz ARM
             = 60 000 ARM cycles
             = 180 000 DSP cycles  (DSP = 3× ARM via PLL)
```

## 3. Multiframes / superframe / hyperframe (GSM 05.02)

```
MULTIFRAME_TC  = 26 frames           ; traffic channel multiframe
              = 26 × 4.61538 ms      = 120 ms
MULTIFRAME_CC  = 51 frames           ; control channel multiframe
              = 51 × 4.61538 ms      = 235.385 ms
SUPERFRAME     = 26 × 51 frames      = 1326 frames
              = 26 × 51 × 4.61538 ms = 6.12 s
HYPERFRAME     = 2048 × SUPERFRAME   = 2 715 648 frames
              = 2048 × 6.12 s        ≈ 3h28m54s
```

C'est `GSM_HYPERFRAME = 2715648` qui apparaît dans
`include/calypso/irq.h` et notre `calypso_tint0.h` :
```c
#define GSM_HYPERFRAME      2715648
```

`fn` (frame number) wrappe modulo HYPERFRAME.

## 4. TINT0 — l'horloge maîtresse silicium

Sur la puce :
```
DSP TIMER 0 (TIM register @ 0x0024)
  ├─ source : DSP_CLK / N         (N = prescaler dans TCR register)
  ├─ N programmé pour fire à 1/4.615ms = 216.668 Hz
  ├─ N = DSP_CLK_HZ × 4.61538e-3
        = 39e6 × 4.61538e-3
        = 180 000 DSP cycles entre 2 TINT0
```

**Équation** :
```
TINT0_PERIOD_NS = 4 615 384 ns   (exact = 60_000_000 / 13)
                ≈ 4 615 000 ns   (notre #define, arrondi)
TINT0_PERIOD_US = 4615           (notre #define, arrondi à la µs)
TINT0_FREQ      = 216.6 Hz       (= 13e6 / 60000)
```

(notre `#define TINT0_PERIOD_NS 4615000ULL` introduit 0.008% d'erreur
sur le rate — soit 1 frame slip toutes les ~12 500 frames = ~58 secondes
réelles. Pour FBSB sub-second, négligeable. Pour LU multi-minutes,
suffisant.)

## 5. Chaîne de propagation TINT0 → reste du PCB

```
                       TINT0 (DSP timer)
                            │ fire @ 216.6 Hz
              ┌─────────────┼─────────────────────────────┐
              ▼             ▼                              ▼
        DSP SINT17    TPU sync line                  ARM (via INTH)
        (vec 17,            │                            │
         IFR bit 4)         ├── arme scenarios          ├── IRQ_TPU_FRAME (4)
              │             │   pour la frame N+1       │   → frame_irq handler
        ┌─────┘             ├── arme RX windows         │   → l1s scheduler
        ▼                   │   (FB/SB/NB/RACH)         │   → next-frame tasks
   GSM tasks DSP            │                           │
   (FB-det, NB demod        ├── arme TX bursts          ├── IRQ_API (15)
    BCCH/CCCH decode)       │                           │   ← DSP via mailbox
                            │                            │
                            ▼                            ▼
                       BSP DMA                      handler dispatched
                       (samples ↔ DARAM)            (UART, SIM, etc)
```

**Tous ces threads sont SYNCHRONES sur le TINT0** — c'est la frame boundary
qui définit le quantum d'ordre. Pas de course possible entre threads tant
qu'ils respectent ce point de synchro.

## 6. Délais (latences) caractéristiques

```
DARAM access (DSP)        : 1 DSP cycle    = 26 ns
                          = 1/180000 TDMA
BSP DMA burst transfer    : ~10 µs         (per burst, ~156 samples × 64 ns)
                          = 0.0022 TDMA
ARM IRQ entry             : ~10 ARM cycles = 770 ns
                          = 0.000167 TDMA
DSP SINT17 entry          : ~7 DSP cycles  = 180 ns
                          = 0.000039 TDMA
SIM IT_WT typical delay   : ~9600 ETU      = 25 ms     (ETU = elementary time unit)
                          = 5.4 TDMA frames
UART_MODEM byte @ 115200  : 87 µs/byte     (10 bits + start/stop)
                          = 0.019 TDMA
```

Pour FBSB lock :
```
FCCH train (1 burst pure 67-tone)  : 1 frame = 4.615 ms
SCH (SB) burst                      : 1 frame
NB demod chain                      : 4 frames (interleave)
FB→SB sync time                     : ~11 frames = ~50 ms = ~11 × TINT0
```

## 7. Notre QEMU — où on respecte vs où on dévie

| HW réel | QEMU | écart |
|---|---|---|
| TINT0 = 4.615 384 ms | `TINT0_PERIOD_NS=4 615 000` (arrondi) | -0.008 % |
| DSP cycles = 3× ARM | DSP/ARM exec serialisés | **mauvais** (sériel) |
| BSP DMA continu | BSP drain timer 5 ms (VIRTUAL) | ~match en cadence |
| TPU autonome | TPU en main loop (callbacks) | **mauvais** (pas thread) |
| ARM IRQ atomique | qemu_set_irq atomic | ✓ |
| Wall vs virtual | icount=auto → virt ≠ wall | **mauvais** sous threading |

## 8. Implications pour le threading PCB

D'après les équations :
- **TDMA frame = synchronisation barrier naturelle** (4.615 ms = unité d'ordre).
- **DSP doit pouvoir exécuter 180k cycles entre 2 TINT0** (pour rester real-time).
  Sous notre QEMU à ~7M insn/s wall, 180k = 25 ms wall → **DSP est 5.4× plus lent
  que real-time**. C'est physiquement impossible de tenir le temps réel sans
  optimiser le c54x interpreter (hot path inlining, TCG-style JIT).
- **Bridge.py wall-paced + QEMU virtual-paced** = drift inéluctable. Solution :
  bridge.py se cale sur QEMU FN (BRIDGE_CLK_FROM_QEMU=1) ou QEMU passe
  wall-clock (icount=off).
- **BQL acquired par thread tick** = sérialise quand même avec ARM. Pour vraie
  concurrence Phase 5, il faut locks fins sur les structures partagées
  (DARAM, IRQ lines, registers), pas BQL global.

## 9. Targets de validation post-threading

- `TDMA tick rate measured` doit être ≥ 200 fn/s wall (= 92% real-time)
- `bts_fn - qfn` doit rester constant à ±10 frames (pas de drift)
- `BSP RX delta mean` doit rester < lookahead/2 + small bounded jitter
- `dsp_n_exec_2 + dsp_n_exec_5` par tick doit ≥ budget (= DSP utilise sa
  fenêtre, pas en IDLE eternel)

Si l'un de ces critères dérive, c'est un signe que le découpage threading
n'a pas tenu compte d'un lien physique TINT0 documenté ci-dessus.
