# MTTCG Audit — Calypso QEMU

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous. (Ce doc est un audit de locks MTTCG ; le cœur — ordre canonique des mutex — reste valide, mais les numéros de ligne et les statuts C2 ci-dessous ont été corrigés, et il n'existe PAS de `calypso_orch.c` ni de bus UDP 6920/6921.)

État Phase C1 (scaffolding initial). Mode multi-thread TCG activable via
`CALYPSO_MTTCG=1` qui passe `-accel tcg,thread=multi` à QEMU + force
`icount=off` et `PCB_TICK_THREADS=0` (incompat / redondant).

## Architecture cible

```
ARM946 TCG thread (QEMU MTTCG)      ← run par QEMU thread vCPU
     │
     ├── MMIO callbacks (calypso_*) ← run dans ARM thread, BQL en partie
     │     │
     │     └── data_read/write     ← LOCKED par daram_lock (C1 ✓)
     │     └── api_ram[] access    ← TODO C2 : api_ram_lock
     │     └── tpu_regs[] access   ← TODO C3 : tpu_lock
     │     └── sim it/fifo access  ← TODO C3 : sim_lock
     │
     ├── QEMU timer callbacks      ← run dans QEMU iothread, BQL
     │     └── tdma_tick, kick, frame_irq, tint0_tick
     │           └── c54x_run() → data_read/write  ← LOCKED ✓
     │           └── api_ram[]   ← TODO C2
     │
     └── IRQ via qemu_set_irq     ← atomic ✓ (qemu native)
```

## Locks définis (calypso_full_pcb.h)

```c
QemuMutex calypso_pcb_daram_lock;   /* DSP/ARM-OVLY DARAM */
QemuMutex calypso_pcb_api_ram_lock; /* API mailbox 0x800-0xFFF */
QemuMutex calypso_pcb_sim_lock;     /* SIM IT register + FIFO */
QemuMutex calypso_pcb_bsp_q_lock;   /* BSP UDP queue */
QemuMutex calypso_pcb_tpu_lock;     /* TPU registers + scenarios */
```

Ordre canonique (pour éviter deadlock cross-lock) :
```
daram_lock < api_ram_lock < sim_lock < bsp_q_lock < tpu_lock
```
Toujours acquire dans cet ordre, release dans l'inverse.

## Status par fichier

### calypso_c54x.c — DSP CPU
| site | lock | status |
|---|---|---|
| `data_read()` wrapper | `daram_lock` | ✅ C1 |
| `data_write()` wrapper | `daram_lock` | ✅ C1 |
| `s->api_ram[woff] = val` (calypso_c54x.c:3315) | `api_ram_lock` | ❌ TODO C2 |
| `s->api_ram[addr-API_BASE]` reads (calypso_c54x.c:1891,1967,2003,2033,2131,2231) | `api_ram_lock` | ❌ TODO C2 |
| `s->api_ram[addr-API_BASE]` writes (calypso_c54x.c:13173,13201,13453) | `api_ram_lock` | ❌ TODO C2 |
| MMR read/write (s->sp, s->imr, etc) | TBD | ❌ TODO C4 |

### calypso_sim.c — SIM controller
| site | lock | status |
|---|---|---|
| `calypso_sim_reg_read()` | `sim_lock` | ❌ TODO C3 |
| `calypso_sim_reg_write()` | `sim_lock` | ❌ TODO C3 |
| `s->it` writes from fire_wt / deliver_atr | `sim_lock` | ❌ TODO C3 |

### calypso_trx.c — TRX glue
| site | lock | status |
|---|---|---|
| `calypso_tdma_tick()` body | `tpu_lock` | ❌ TODO C3 |
| `calypso_kick_cb()` body | `daram_lock` (cpu_phys_write rxDoneFlag) | ❌ TODO C3 |
| TPU registers read/write | `tpu_lock` | ❌ TODO C3 |
| DMA-page mirror → DSP DARAM (calypso_trx.c:996-1003) | `daram_lock` < `api_ram_lock` | ✅ FIXÉ (calypso_trx.c:995-1004, ordre canonique daram<api_ram) |

### calypso_bsp.c — BSP DMA
| site | lock | status |
|---|---|---|
| `bsp_burst_enqueue()` | `bsp_q_lock` | ❌ TODO C3 |
| `bsp_drain_cb()` queue pop | `bsp_q_lock` | ❌ TODO C3 |
| DARAM write (delivery) | `daram_lock` | ❌ TODO C3 |

### calypso_tint0.c — TINT0 timer
| site | lock | status |
|---|---|---|
| `tint0_tick_cb()` body | (delegates to trx) | ❌ TODO C3 |

### calypso_iota.c — IOTA TWL3025
| site | lock | status |
|---|---|---|
| SPI read/write | dedicated `iota_lock` (new) | ❌ TODO C4 |

## Phases d'application

**C1 ✅ (scaffolding + DARAM)** :
- run.sh : CALYPSO_MTTCG=1 → `-accel tcg,thread=multi`
- c54x.c : data_read/write wrapped avec daram_lock

**C2 (api_ram)** :
- Wrap les sites api_ram[] dans c54x.c avec api_ram_lock (calypso_c54x.c:1891,1967,2003,2033,2131,2231,3315,13173,13201,13453) — ❌ TODO
- ✅ FIXÉ : la mirror DMA-page → DSP DARAM dans calypso_trx.c est déjà sous `daram_lock` puis `api_ram_lock` (calypso_trx.c:995-1004, ordre canonique respecté). C'est le SEUL site non-daram C2/C3/C4 réellement verrouillé à ce jour.

**C3 (sim, bsp, tpu, kick)** :
- Wrap calypso_sim_reg_read/write
- Wrap bsp queue ops
- Wrap TPU register ops + tdma_tick body

**C4 (MMR + IOTA + reste)** :
- Audit MMR reads/writes dans c54x.c (SP, AR, IMR, IFR, ...)
- Iota SPI lock
- Audit final tous les `s->` access cross-thread

**C5 (validation)** :
- Stress test : 60s run avec CALYPSO_MTTCG=1
- Vérifier pas de deadlock (thread sanitizer si dispo)
- Vérifier FBSB lock fonctionne en mode parallèle
- Comparer DSP throughput MTTCG vs single

## Risques connus

| risque | mitigation |
|---|---|
| Deadlock cross-lock | Ordre canonique strict (documenté ci-dessus) |
| Mutex non-récursif → recursion = crash | Vérifier pas de data_read inside data_read body (chercher case ADRR avec indirect) |
| Performance hit lock overhead | Mesurer ; si grave, switch sur RCU ou per-zone locks |
| IRQ delivery race | qemu_set_irq déjà atomic |
| MMIO callback race | QEMU MTTCG model : MMIO sous BQL implicite |

## Activation

```bash
CALYPSO_MTTCG=1 ./run.sh
```

Logs attendus dans qemu.log :
- `[pcb] init OK ...` ~~orchestrator armé~~ — FAUX: il n'existe pas d'orchestrateur (`calypso_orch.c` absent, aucun bus UDP 6920/6921). Le lock `bsp_q_lock` est déclaré « BSP UDP queue » dans calypso_full_pcb.h:52 mais aucune queue UDP n'est branchée.
- Pas de `tick threads spawned` (CALYPSO_PCB_TICK_THREADS forcé à 0)
- `INSN-COUNT-STATS rate=...` plus stable (ARM en thread propre)

Désactiver pour fallback :
```bash
CALYPSO_MTTCG=0 ./run.sh   # ou unset
```
