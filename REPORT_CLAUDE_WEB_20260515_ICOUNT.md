# Rapport Claude web — 2026-05-15 après-midi (test icount=auto)

> Suite du rapport principal (REPORT_CLAUDE_WEB_20260515.md).
> Session : test exploratoire icount=auto pour gérer la variance run-to-run
> du real DSP path (synth=0). Plan en 3 étapes de Claude web suivi.

## TL;DR

**Étape 1 (audit fixes historiques) : ✓ PASS** — tous les fixes liés à icount
sont en place et documentés dans le code.

**Étape 2 (test single run icount=auto) : 2/3 invariants OK, 1 KO** —
l'architecture tient (pas de timeout, kick fire correctement) mais **icount=auto
exacerbe le bug INTM dwell** qui passe d'intermittent à systématique.

**Conclusion** : icount=auto est viable architecturalement mais **révèle un bug
DSP runtime** qui était masqué par le timing wall-clock. Le real path FBSB ne
converge pas sous ce mode. Il faut soit fixer INTM dwell avant d'utiliser
icount=auto, soit rester icount=off et attaquer la variance autrement.

## Plan d'origine (Claude web)

> Étape 0 : 10 runs identiques icount=off pour caractériser variance baseline
> Étape 1 : audit fixes historiques (calypso_inth_update, main_loop_wait, SP leak)
> Étape 2 : single run icount=auto avec vérif 3 invariants
> Étape 3 (si auto cassé) : icount=shift=N déterministe

L'utilisateur ayant priorité "la variance me saoule", on a sauté étape 0 et
attaqué directement étape 1+2.

## Étape 1 — Audit des fixes historiques (PASS)

Audit fait, tous présents et documentés :

| Fix | Localisation | Status |
|---|---|---|
| FIQ/IRQ arbitration séparée | `hw/intc/calypso_inth.c:27-75` (AUDIT FIX 2026-05-08 night) | ✓ |
| `main_loop_wait(false)` recursive comment + fix | `calypso_trx.c:919` (comment), fix dans calypso_uart.c | ✓ |
| Timers sur `QEMU_CLOCK_VIRTUAL` | `calypso_trx.c:1080-1082` | ✓ |
| Kick timer sur `QEMU_CLOCK_REALTIME` (intentionnel pour liveness) | `calypso_trx.c:1084` | ✓ |
| SP leak log cap (3 words/IDLE) | `calypso_c54x.c:6463-6473` | ✓ |
| Default CALYPSO_ICOUNT=auto dans run.sh | `run.sh:80` | ✓ |

**Découverte importante** : `calypso_trx.c:937-957` contient un XXX HACK pour un
bug TCG sous icount=auto sur le conditional STR @ 0x8224ac (SIM busy-poll) qui
requiert `CALYPSO_FORCE_RX_DONE=1` en parallèle. L'utilisateur n'avait jamais
activé ce flag dans ses essais précédents d'icount=auto, expliquant probablement
le "ça marche pas" historique.

## Étape 2 — Single run icount=auto

### Commande de lancement

```bash
CALYPSO_ICOUNT=auto CALYPSO_FORCE_RX_DONE=1 ./run.sh
```

QEMU cmdline résultant (vérifié via `ps`) :

```
qemu-system-arm -M calypso -cpu arm946 -icount auto -serial pty -serial pty
                -monitor unix:/tmp/qemu-calypso-mon.sock -kernel layer1.highram.elf
```

### Vérif 3 invariants

| Invariant | Attendu | Mesuré | Verdict |
|---|---|---|---|
| `WAIT-A21A` count | 0 | **0** | ✓ |
| `ENTER-7740` count | 0 | **0** | ✓ |
| `INTM` converge vers 0 | INTM=0 stable en fin | **INTM=1 dwell persistant** | ✗ |

L'invariant 3 fail : sur les 3 derniers STATE-DUMP successifs (insn=500M, 1B,
1.5B), tous montrent `INTM=1`, `IFR=0x0028`, **même PC=0x9325**.

### Métriques 4-axes à insn=2.16B

| Métrique | icount=off ce matin | icount=off ce midi | **icount=auto** |
|---|---:|---:|---:|
| D_FB_DET SET non-zéro | 14 | 11 | 10 |
| task=5 (FB-det retries) | 782 | 3274 | 682 |
| task=1 (NB) | 195 | 816 | 168 |
| task=6 (SB) | — | 4 | 0 |
| **task=24 (ALLC/CCCH)** | **20** | **0** | **0** |
| FBSB_CONF (mobile feedback) | 193 | 85 | 85 |
| DATA_IND | 0 | 0 | 0 |
| ARM RD d_fb_det total | 192 | 180 | 195 |
| dont non-zéro | 5 (2.6%) | 5 (2.78%) | **1 (0.51%)** |
| INTM=1 / INTM=0 transitions | ? | ? | 599 / 4972 |
| RETE / ISR entered traces | ? | ? | **0 / 0** |
| kick fires REALTIME | ? | ? | 93 |

### Le PC=0x9325 stuck loop

Le `STATE-DUMP SP-RING` (snapshot des 32 derniers PC/SP) en fin de run montre :

```
[4265029800:PC=9325 SP=85ec] [4266053800:PC=9328 SP=85ec] [4267077800:PC=9329 SP=85ec]
[4268101800:PC=9329 SP=85ec] [4269125800:PC=9329 SP=85ec] [4270149800:PC=9328 SP=85ec]
... PC oscille entre 9325/9328/9329, SP stable à 0x85ec ...
```

→ DSP boucle sur 3 PC consécutifs (0x9325 / 0x9328 / 0x9329) avec SP gelé.
C'est un busy-wait, pas un dispatcher idle (sinon PC plus varié).

État au moment du capture (typique des 3 STATE-DUMPs) :

```
PC=0x9325 ST0=0x46f6 ST1=0x4800 INTM=1 IMR=0x4d31 IFR=0x0028
                                       │            │
                                       │            └─ INT3 (bit 3) + BRINT0 (bit 5) pending
                                       └─ INT3 et BRINT0 enabled dans IMR
```

Soit : `IFR & IMR & ~INTM = 0` **uniquement parce que INTM=1**. Catch-22
classique du dwell INTM. Mais sous icount=off, ce dwell était intermittent
(observé une fois à insn=880M dans toute la session du 14-15). Sous icount=auto,
il devient **stable et bloquant** dès insn=500M.

### Lecture du résultat

1. ✓ **Architecture supporte icount=auto** : pas de timeout sur le TDMA tick,
   kick fires correctement (93 hits), bridge UDP suit virtual time. Les fixes
   historiques (FIQ arbitration, main_loop_wait, kick sur REALTIME) tiennent.

2. ✓ **CALYPSO_FORCE_RX_DONE=1 efficace** : pas de blocage SIM, pas de
   timeout L1CTL_PM_REQ, le flow boot atteint le DSP démarrage normalement.

3. ✗ **INTM dwell devient stable sous icount=auto** : le bug intermittent
   `[[project_qemu_calypso_intm_dwell_intermittent]]` se révèle systématique
   quand le timing wall-clock est supprimé. C'est probablement un opcode
   SSBX/RSBX INTM ou RETE mal modelé par TCG dont le bug est compensé par
   le jitter wall-clock en mode off.

4. ✗ **Variance ARM RD aggravée** : 1/195 (0.51%) sous icount=auto vs 5/180
   (2.78%) sous icount=off. icount=auto **réduit** la fenêtre de "chance" pour
   ARM de lire d_fb_det non-zéro avant que DSP clear.

5. ✗ **0 RETE / 0 ISR entered** dans tout le run : le DSP n'entre jamais
   dans un ISR. Soit l'instru manque (RETE pas tracé), soit le DSP boucle
   genuinement à PC=0x9325 sans jamais atteindre un point d'entry ISR.

## État du problème global après cette session icount

- icount=auto est **architecturalement viable** (pas de régression timeout)
- mais **expose un nouveau blocker** : INTM dwell systématique
- le real path FBSB ne converge dans aucun des deux modes :
  - icount=off : convergence intermittente (1 run sur 2 environ, variance)
  - icount=auto : convergence nulle (INTM=1 dwell stable bloque tout)

## Questions pour Claude web

1. **PC=0x9325 dans le firmware DSP** — c'est dans la zone PROM (`/opt/GSM/calypso_dsp.txt`).
   Le fait que SP soit stable à 0x85ec et PC oscille entre 3 adresses suggère
   soit un wait-flag busy-poll, soit un wait-for-interrupt opcode (IDLE 1/2/3
   sur C54x). Si IDLE, et que INTM=1, le DSP est genuinely deadlocked sauf
   reset externe. Tu valides cette lecture ? Recommandes-tu de désassembler
   manuellement les words à PROM[0x9325..0x9330] pour identifier l'opcode ?

2. **Bug TCG potentiel** — la combinaison "IFR=pending + IMR=enabled + INTM=1"
   sous icount=auto devrait quand même être servie par le check IRQ à chaque
   point d'instruction (sauf si DSP a explicitement masqué via SSBX INTM).
   Tu as l'intuition d'un opcode C54x dont la sémantique INTM-touchante n'est
   pas correctement décodée dans `calypso_c54x.c` ? Le commentaire HACK pour
   le STR @ 0x8224ac mentionne un bug TCG cousin sur les conditionals — il
   pourrait y avoir une famille à auditer.

3. **Path alternatif sans icount=auto** — vu que icount=auto révèle un bug
   bloquant mais que la variance icount=off reste, la séquence rationnelle
   serait peut-être :
   - rester icount=off
   - mesurer baseline 10 runs (étape 0 originale) pour comprendre si la
     variance est 1/10 ou 8/10
   - si 5+/10, c'est viable pour itérer sur DATA_IND
   - parallèlement instrumenter le PC=0x9325 (probe IDLE / wait-flag) pour
     préparer le fix INTM dwell quand on l'attaquera

   Tu vois un raccourci, ou la séquence "mesure baseline + creuse DATA_IND
   en mode synth=1 déterministe" est-elle la moins risquée ?

## Annexe : env qui a fait tourner ce run

```
CALYPSO_ICOUNT=auto
CALYPSO_FORCE_RX_DONE=1
CALYPSO_BSP_DARAM_ADDR=0x0080
CALYPSO_FBSB_SYNTH=0
CALYPSO_W1C_LATCH=1
CALYPSO_DSP_IDLE_FF=1
BRIDGE_CLK_PERIOD=51
BRIDGE_DL_FN_LOOKAHEAD=64
```
