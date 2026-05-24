# Rapport Claude web — 2026-05-23 : session complète post-Plan A → pivot vers mur DATA_IND

> Extension de REPORT_CLAUDE_WEB_20260523_TIMER_GATE.md. Le rapport précédent
> documente l'application du gate `CALYPSO_TIMER` et la mesure
> TIMER=1 vs TIMER=0. Celui-ci couvre la chaîne complète post-Plan A :
> prep statique du path DSP→ARM, vérifications binaire + env, tentative
> `ICOUNT=auto` bloquée, analyse du workaround `CALYPSO_FORCE_RX_DONE`,
> et la décision de pivoter vers le mur `DATA_IND` comme front principal.

## TL;DR

- Plan A confirmé efficace mais **pas suffisant** : gate fonctionne, mais
  qfn rate reste à ~20 Hz quel que soit `CALYPSO_TIMER`. Le logging
  n'était pas le bottleneck.
- **Test ENV `CALYPSO_ICOUNT=auto` bloque le romload-stub** entre
  "parameter ack" et "checksum ack" (osmocon timeout à +0.658s, QEMU
  continue à +124s sans bytes downloadés). Bug séparé du Task #29 SIM
  (lequel est post-boot firmware, pas pre-boot romload). Path
  `calypso_uart.c` romload-handshake icount-unsafe.
- **Analyse statique du path `DSP→ARM`** posée : carte INTH/IRQ4
  (`TPU_FRAME`), arbitration IRQ/FIQ déjà fixée 8 mai. Mais les xfail
  `milestone_irq` existants probent **direction ARM→DSP**, pas DSP→ARM
  — angle mort pour le mur DATA_IND.
- **Le workaround `CALYPSO_FORCE_RX_DONE` est triplement-écrit et 200 Hz
  aveugle** : 3 sites (calypso_trx.c kick CB + calypso_sim.c SIM_IT read
  + calypso_sim.c init), écrasement périodique de `rxDoneFlag` dans la
  RAM physique tous les 5 ms wall via REALTIME kick, **sans corrélation
  avec une op SIM réelle**. Masque le vrai bug TCG (`STR` conditionnel
  @ 0x8224ac qui ne commit pas sous icount).
- **Décision** : on tient `CALYPSO_ICOUNT=off`, on accepte 20 qfn/s
  comme working point, on **pivote vers l'analyse du path
  TPU_FRAME → ARM frame ISR → NDB read → sercomm → L1CTL_DATA_IND**.
  Le timing devient un front parallèle indépendant. Les 4 compteurs
  ARM-side à instrumenter sont listés au §6.

## 1. Carte INTH/IRQ pour le path DSP → ARM (analyse statique)

### 1.1 IRQ4 = TPU_FRAME, edge-cleared sur IRQ_NUM read

`hw/intc/calypso_inth.c:129-134` :

```c
case 0x10: /* IRQ_NUM — read returns current highest-priority IRQ */
{
    uint16_t num = s->ith_v;
    /* Clear level for edge-like sources (TPU_FRAME=4, TPU_PAGE=5, API=15).
     * These pulse once per event; clearing here prevents re-trigger
     * until the next event raises the line again. */
    if (num == 4 || num == 5 || num == 15) {
        s->levels &= ~(1u << num);
    }
    calypso_inth_update(s);  /* re-arbitrate immediately for chained ISRs */
    return num;
}
```

C'est l'IRQ qui doit déclencher l'ARM frame ISR qui lit la NDB et émet
`L1CTL_DATA_IND` vers le mobile. L'auto-clear sur read est la bonne
sémantique edge — pas de re-trigger spurious.

### 1.2 Arbitration IRQ/FIQ indépendante depuis AUDIT FIX 8 mai

`calypso_inth.c:33-51`, `:158-160` documentent le fix : avant 5 mai,
arbitration single-best confondait IRQ et FIQ ; un IRQ-routed plus
prioritaire pouvait lowering la parent_fiq → ARM jamais réveillé sur
FIQ. Aujourd'hui FIQ et IRQ sont arbitrés séparément. **Le bug SIM/FIQ
vs UART/IRQ de mai est fixé. Sans rapport avec le blocker actuel.**

### 1.3 IRQ4 raise side (calypso_trx.c)

`calypso_trx.c:878` : `qemu_irq_raise(s->irqs[CALYPSO_IRQ_TPU_FRAME])`
exécuté à chaque `calypso_tdma_tick`. Le compteur `firq_count` confirme
que le raise fire (200 KB de log dans `/tmp/frame_irq.log` sous
TIMER=1). Donc côté QEMU, l'IRQ est bien levée.

### 1.4 Gap : pas de compteurs côté ARM

Aucune instrumentation actuelle ne mesure :
- Si `qemu_irq_raise` propage jusqu'au CPU ARM (INTH dispatch OK ?).
- Si l'ARM frame ISR est entrée (vector dispatched, handler PC reached ?).
- Si le handler lit effectivement `a_cd[]` dans NDB.
- Si `L1CTL_DATA_IND` est formé et envoyé via sercomm.

C'est l'angle mort à instrumenter (cf §6).

## 2. xfail `milestone_irq` probent la mauvaise direction

`tests/test_calypso_milestones.py:521-545` :

```python
@pytest.mark.milestone_irq
def test_c54x_interrupt_ex_called_nonzero(container_alive):
    """Probe à ajouter côté QEMU : incrément à chaque entrée de c54x_interrupt_ex.
    =0 → hypothèse (a) : générateur IRQ ARM→DSP ne fire jamais."""
    pytest.xfail("TODO: ajouter compteur + exposition monitor")

def test_isr_entered_implies_rete(container_alive):  ... pytest.xfail(...)
def test_no_pending_irq_gated(container_alive):       ... pytest.xfail(...)
```

`c54x_interrupt_ex` est l'entrée IRQ du **C54x emulator côté DSP**.
Direction probée = **ARM/QEMU → DSP** (la wakeup qui fait tourner les
tasks DSP).

Mais notre blocker — `task=24 (ALLC) fire 17×, DATA_IND=0` — implique
que le DSP **a bien tourné** (sinon task=24 ne fire pas du tout). La
rupture est en aval, sur le retour DSP→ARM. **Ces 3 xfail ne couvrent
pas ce path.**

Discrimination équivalente côté ARM, à ajouter (mêmes hypothèses a/b/c
projetées) :
- (a') `tpu_frame_raised > 0 && arm_frame_isr_entered = 0` → IRQ raised
  mais ARM jamais réveillé (INTH dispatch / CPSR.I / vector ?)
- (b') `arm_frame_isr_entered > 0 && arm_read_a_cd_count = 0` → ISR
  entre mais ne lit pas NDB (PC wrong, handler dispatch wrong)
- (c') `arm_read_a_cd_count > 0 && data_ind_emitted_total = 0` → ISR
  lit NDB mais sercomm/L1CTL formation cassée

## 3. Smoking gun bonus : INTH log spam non gaté

`calypso_inth.c:143-151` contient encore :

```c
if (num == 7) {                          /* IRQ7 = UART_MODEM */
    irq7_count++;
    if (irq7_count <= 50 || (irq7_count % 100) == 0)
        fprintf(stderr, "[INTH] IRQ7 dispatch #%u ...\n", ...);
}
if (total <= 20 || total == 100 || total == 500 || total == 1000)
    fprintf(stderr, "[INTH] IRQ_NUM=%u (#%u) ...\n", ...);
```

**Non gatés par `CALYPSO_TIMER`.** Bornés par leur thinning mais en
steady-state `IRQ7` peut firer souvent (UART_MODEM traffic) → 1/100 =
quelques lignes par seconde.

Pas dans le scope strict de "timer" (INTH ≠ timer), mais même
pathologie de backpressure. À gater si on veut un vrai `--silent`
plus tard (TODO §9 d'archi).

## 4. Détail run.sh à valider : path `arm-elf-nm`

`run.sh:227-228` :

```bash
NM="/root/gnuarm/install/bin/arm-elf-nm"
[ -x "$NM" ] || NM="arm-elf-nm"
```

Vérifié container : `/root/gnuarm/install/bin/arm-elf-nm` n'existe pas
→ fallback PATH. Si `arm-elf-nm` n'est pas dans PATH non plus,
l'auto-détection `CALYPSO_RXDONE_ADDR` ne fire pas et le default
hardcodé `0x008302a0` est utilisé. Cohérent avec le build firmware
actuel mais fragile aux rebuilds. À surveiller dans la console de
`run.sh` :

```
[run.sh] auto-detected CALYPSO_RXDONE_ADDR=0x008302a0 (from nm $FW_ELF)
```

Si cette ligne est absente, le default est utilisé silencieusement.

## 5. Run check : binaire + env (validation post-Plan A)

PID 22279, run TIMER=1 :

```
binary mtime    = May 23 11:01  (avant le run de 11:15, mais source
                                  synced before build → contient mes edits)
nm symbol       = calypso_timer_log (présent ✓)
strings binary  = CALYPSO_TIMER × 4 (présent ✓)
ninja rebuild   = [1/4] Generating qemu-version.h (no-op ✓)
env du process  = CALYPSO_TIMER=1, CALYPSO_FBSB_SYNTH=1, CALYPSO_ICOUNT=off
```

Plan A est dans le binaire actif. Vérifié par symbole + comportement
empirique (kick/tdma-skip count à 35/10 sous =1 vs 0/0 sous =0).

## 6. Mesure TIMER=1 vs TIMER=0 (synthèse — détail dans TIMER_GATE.md)

| Métrique | TIMER=1 (51 s) | TIMER=0 (16 s) |
|---|---|---|
| qemu.log | 3.07 MB | 2.33 MB |
| `[kick] fire` | 35 | **0** ✓ |
| `[tdma-skip]` | 10 | **0** ✓ |
| `/tmp/frame_irq.log` | 200 KB | absent (write) ✓ |
| qfn rate | **19.7 Hz** | **19.5 Hz** |
| task=24 / DATA_IND | 57 / **0** | 17 / **0** |

**Hypothèse logging-tax → réfutée.** Δqfn = 0.2 Hz dans le bruit.
**Mur DATA_IND → reproductible** sur les deux runs même avec
`FBSB_SYNTH=1`.

## 7. Tentative `CALYPSO_ICOUNT=auto` : romload bloqué

Run avec `CALYPSO_ICOUNT=auto CALYPSO_FBSB_SYNTH=1 CALYPSO_TIMER=0`
(PID 23735, sans `FORCE_RX_DONE`) :

```
osmocon.log  : "Received parameter ack from phone, starting download"  @ +0.658s
              (silence after)
qemu.log     : [BSP] RX delta stats jusqu'à +124.6s (QEMU continue à tourner)
```

**Diagnostic** : le firmware ELF est déjà loadé via `-kernel` côté
QEMU (donc BSP continue), mais le **romload handshake côté UART
bloque entre "parameter ack" et "checksum ack"**. Le romload-stub
dans `calypso_uart.c` n'achève plus la séquence sous `-icount auto`.

C'est **un bug différent du Task #29 SIM/rxDoneFlag** : Task #29
affecte les ops SIM *après* boot firmware (ATR, SELECT, READ_BINARY),
le romload se produit *avant* que le firmware ne s'exécute. Les deux
paths sont indépendants. Sous toute vraisemblance, le romload-stub a
sa propre fragilité à `cpu_exit`/`qemu_notify_event` pacing sous
icount.

**Conséquence pratique** : on ne peut pas mesurer un qfn baseline
propre sous `icount=auto` sans rebuild de la romload-stub
icount-safety. La voie « bumper qfn vers 217 Hz via icount » est
**fermée temporairement**.

## 8. Analyse du workaround `CALYPSO_FORCE_RX_DONE`

### 8.1 Trois sites d'écriture

| Site | Trigger | Période |
|---|---|---|
| `calypso_trx.c:1038-1053` | kick CB (REALTIME timer) | **5 ms wall → 200 Hz** |
| `calypso_sim.c:601-634` | SIM_IT register read | sur op |
| `calypso_sim.c:232+` | init / SIM activate | sur op |

Le site 1 fire **inconditionnellement à 200 Hz wall, indépendamment
de tout signal SIM réel**. Le commentaire L1037 l'avoue :

> *"Periodic enforcement here (every 5 ms wall via REALTIME kick)
>  keeps rxDoneFlag = 1 reliably, regardless of which SIM op is in
>  flight. cpu_exit forces ARM TB recompile so busy-loop @ 0x822b90
>  picks up the new value."*

### 8.2 Pourquoi c'est moche

1. **Aveugle** : 200 écritures/s dans la RAM physique ARM même quand
   aucune op SIM n'est active.
2. **Triple-redondance** : les sites 2 et 3 sont redondants avec le
   site 1 — un seul suffirait, et le site 1 est le plus brutal.
3. **`cpu_exit` à 200 Hz** : invalide les TB ARM 200×/s. Coût de
   recompilation TCG continu. Plausible contributeur indirect au
   bottleneck timing.
4. **Cache la cause racine** : le vrai bug = `STR` conditionnel
   @ 0x8224ac qui ne commit pas sous icount (Task #29). Le hack
   masque le diag du STR ; pas creusé depuis 8 mai.

### 8.3 Alternatives par ordre de coût

| Approche | Coût | Description |
|---|---|---|
| (a) **PC-trap @ 0x8224ac** | bas | Intercepter l'instruction qui *devrait* écrire le flag, émuler le `STR` avec la sémantique attendue. Une écriture par occurrence réelle, zéro fire spurious. Reste un hack mais ciblé. |
| (b) **PC-trap sur sim_irq_handler entry** | bas | Si IRQ routée mais STR ne commit pas, fire-once à l'entrée du handler (PC=0x822450 ou voisin). Une écriture par SIM_IT, pas par 5 ms. |
| (c) **Fix TCG `STR` conditionnel** | élevé | Creuser pourquoi `STR` conditionnel @ 0x8224ac ne commit pas sous icount. CPSR flags pas propagés ? `tb_invalidate` manquant ? Le vrai fix. |

Symétrique côté ARM de `feedback_no_function_hacks_in_dsp` :
le rx_done patch viole l'esprit. À documenter comme dette TODO.

## 9. Décision : pivot vers mur ARM-feedback `DATA_IND`

### 9.1 Pourquoi maintenant

- Plan A appliqué : oui (logging-tax écarté comme faux ami).
- Voie `icount=auto` : fermée (bug romload-stub séparé, hors scope).
- `rx_done` patch : reconnu hack moche, ne pas le rallumer pour
  mesure timing.
- Mur `DATA_IND` : reproductible (task=24=17, ALLC=1, DATA_IND=0
  en 16 s), indépendant du qfn rate (même à 20 Hz on devrait voir
  ≥ 1 DATA_IND si le path était sain), c'est un problème de
  **signaling** pas de débit.
- Front timing : devient un front parallèle indépendant. À fixer
  plus tard côté DSP-compute ou icount-safety du romload.

### 9.2 Compteurs à instrumenter (côté ARM, exposition via monitor `info`)

```
tpu_frame_raised_total     : nombre de qemu_irq_raise sur CALYPSO_IRQ_TPU_FRAME
                             (déjà fait via firq_count mais perdu après cap 2000)
arm_frame_isr_entered      : entrées de la frame ISR ARM (vector / PC-trap
                             sur l'entry point du handler — symbole nm à
                             identifier dans layer1.highram.elf)
arm_read_a_cd_count        : reads du firmware sur la zone a_cd[] dans NDB
                             (probe sur DSP API base + offset NDB+0x1F8..+0x3A0)
data_ind_emitted_total     : L1CTL_DATA_IND formés et envoyés via sercomm
                             (probe dans l1ctl_sock.c ou sercomm_gate.c)
```

### 9.3 Discrimination attendue (mêmes hypothèses a/b/c côté ARM)

```
tpu_frame_raised >> arm_frame_isr_entered = 0       → (a') INTH dispatch
                                                       ou CPSR.I masquant
                                                       ou vector wrong
arm_frame_isr_entered > 0, arm_read_a_cd_count = 0  → (b') ISR ne lit pas NDB
                                                       (handler PC wrong,
                                                       jump table cassé)
arm_read_a_cd_count > 0, data_ind_emitted_total = 0 → (c') NDB lue mais
                                                       sercomm/L1CTL cassé
```

## 10. État sync 3-way

```
md5sum calypso_trx.c                          (identique sur les 3)
  20378c969cc891ab6339e03d824ca8ec  /opt/GSM/qemu-src/...
  20378c969cc891ab6339e03d824ca8ec  /home/nirvana/qemu-src/...
  20378c969cc891ab6339e03d824ca8ec  /home/nirvana/qemu-calypso/...

run.sh                                        (divergence pré-existante
                                               préservée, mes 3 inserts
                                               identiques)
  Container : CALYPSO_FBSB_SYNTH default=1 + CALYPSO_FORCE_RX_DONE déclaré
  Hosts     : CALYPSO_FBSB_SYNTH default=0 + pas de FORCE_RX_DONE

REPORT_20260523_TIMER_GATE.md                 (sync 3-way ✓)
REPORT_20260523_PIVOT_TO_ARM_FEEDBACK.md      (ce rapport, sync 3-way ✓)
```

## 11. TODO architectural (hors scope session courante)

1. **`CALYPSO_FORCE_RX_DONE` cleanup** : remplacer le hack triple-site
   200 Hz par une approche (a) ou (b) du §8.3, ou pousser pour le vrai
   fix TCG (c) si la priorité le justifie.
2. **`CALYPSO_DSP_TRACE` companion** : gater les ~12 000 lignes
   d'instrumentation `[c54x] CALAD-ZONE-W / COEFFS-WR / DISP-FLAG-W /
   PC RECENT / BOOT[N.N]` sous un flag dédié (orthogonal à
   `CALYPSO_TIMER`). 180 MB/h en steady-state, principale source du
   2.68 GB observé sur run long.
3. **INTH log spam non gaté** : `calypso_inth.c:143-151` `[INTH] IRQ7
   dispatch` à thin 1/100. À gater sous `CALYPSO_INTH_TRACE` ou inclure
   dans `CALYPSO_TIMER` (orthogonalité à arbitrer).
4. **romload-stub icount-safety** : creuser pourquoi `calypso_uart.c`
   romload handshake bloque entre param-ack et checksum-ack sous
   `-icount auto`. Probablement `cpu_exit`/`qemu_notify_event` pacing.
5. **run.sh cleanup hygiène** : ajouter `rm -f /tmp/frame_irq.log
   /tmp/tdma_tick.log /tmp/tdma_profile.log` au cleanup L359 pour
   éviter que les fichiers d'un run précédent contaminent l'œil sur
   le run courant.
6. **Auto-détection `arm-elf-nm`** : path codé `/root/gnuarm/install/bin/`
   n'existe pas dans le container actuel. Fallback PATH fonctionne
   mais silencieux. Améliorer le log de fallback dans `run.sh:226-237`.

## 12. Questions pour review web

1. **Discrimination a/b/c côté ARM** : la transposition est-elle valide ?
   Pas de subtilité ARM-spec qui changerait la sémantique des compteurs ?
2. **Bug romload-stub icount-safety** : connu dans des branches/snapshots
   antérieurs ? Cherché côté ALL-QEMUs/ ?
3. **Symbole ARM ISR frame handler** : as-tu un nom de symbole à proposer
   pour le PC-trap (nm du firmware) ? Candidats : `_irq_handler`,
   `frame_int_handler`, `tpu_frame_irq`, `l1s_int_handler`…
4. **Ordre des compteurs §9.2** : par ordre de coût d'instrumentation,
   ou par ordre de probabilité a priori du point de rupture ?
5. **Priorisation TODO §11** : on tient un seul front (le pivot
   ARM-feedback) ou on aborde aussi le `CALYPSO_DSP_TRACE` companion
   en parallèle (gain log volume net mais distraction du blocker
   milestone) ?
