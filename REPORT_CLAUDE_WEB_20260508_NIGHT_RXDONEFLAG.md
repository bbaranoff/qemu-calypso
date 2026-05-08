# Diag Claude web — rxDoneFlag deadlock under icount=auto (2026-05-08 night, 3e itération)

## Recap

Ton audit précédent (post-2 fixes main_loop_wait + kick REALTIME) avait conclu que la SIM débloque sous icount=auto et que le blocker est en aval (TDMA scheduler). **C'était basé sur un tarball mixte** (frame_irq.log daté 16:36, qemu_full.log daté 17:26 = 2 runs différents). Mes mesures actuelles montrent que **la SIM est toujours stuck sous icount=auto** dans tous les runs frais.

Plus précisément :
- ARM en busy-poll PC=0x822b90 (LDR/CMP/BEQ rxDoneFlag) — vérifié via 20 samples PC consécutifs sur 10 secondes wall, ARM coince dans la boucle 0x822b90..b98 (3 instructions, 8 octets)
- `rxDoneFlag` (firmware data @ 0x830510) reste 0
- Aucun INTM-TRANS, aucun TDMA, aucun fbsb hook — DSP jamais démarré
- Bridge cur_qfn=0 → bridge timeout

## Les fixes confirmés actifs

| Fix | Status | Effet |
|---|---|---|
| main_loop_wait(false) récursif removed | ✓ in binary | event-loop OK |
| kick_timer REALTIME (re-revert from VIRTUAL) | ✓ in binary, fires 5400×/28s = 193/s = 5ms period correct | fd dispatch OK |
| INTH FIQ/IRQ arbitration séparée | ✓ in binary, FIQ_NUM=6 read 5× per ATR cycle | FIQ propage |
| ATR delivery synchronous (deliver_atr direct call) | ✓ in binary | ATR queued at CMDSTART time |
| SIM IT clear-only-observed-bits (Q2 hardening) | ✓ in binary | propre, pas le bug |

## Smoking gun #5 — Le firmware FIQ handler reçoit IT_WT mais rxDoneFlag reste 0

J'ai instrumenté le SIM_IT register read pour logger chaque accès :

```
[sim] SIM_IT read=0x0010 rx_count=4 edge_cleared=0x0000 post_it=0x0010   ← FIQ #1 (IT_RX)
[sim] SIM_IT read=0x0010 rx_count=3 edge_cleared=0x0000 post_it=0x0010   ← FIQ #2
[sim] SIM_IT read=0x0010 rx_count=2 edge_cleared=0x0000 post_it=0x0010   ← FIQ #3
[sim] SIM_IT read=0x0010 rx_count=1 edge_cleared=0x0000 post_it=0x0010   ← FIQ #4
[sim] SIM_IT read=0x0002 rx_count=0 edge_cleared=0x0002 post_it=0x0000   ← FIQ #5 (IT_WT)
```

**Le 5e read renvoie 0x0002 = IT_WT à ARM.** Donc R2 dans le FIQ handler reçoit la bonne valeur.

## Le firmware handler @ 0x822498 (sim_irq_handler)

Disasm via python (file offset 0x2498 in layer1.highram.bin) :

```
0x822498: e59f30dc   LDR R3, [PC, #0xdc]    R3 = mem[0x82257c] = 0xfffe00ff (SIM base ref)
0x82249c: e1532fb7   LDRH R2, [R3, #-0xf7]  R2 = halfword at 0xfffe0008 (SIM_IT register)
0x8224a0: e3120002   TST R2, #2             test IT_WT bit
0x8224a4: 159f30d4   LDRNE R3, [PC, #0xd4]  R3 = mem[0x822580] = 0x00830510 (rxDoneFlag)
0x8224a8: 13a01001   MOVNE R1, #1
0x8224ac: 15831000   STRNE R1, [R3]         *rxDoneFlag = 1
0x8224b0: e3120008   TST R2, #8             test IT_TX
0x8224b4: 0a000015   BEQ 0x822510           skip TX if no IT_TX
...
0x822580: 0x00830510                         literal pool — verified rxDoneFlag addr ✓
0x822584: 0x0000fffb
```

Le LDRH retourne `R2 = SIM_IT = 0x0002` (vérifié via instrumentation).
Le TST R2,#2 doit poser le NE flag (Z=0, R2&2 != 0).
Le STRNE doit firer.

**Mais `*rxDoneFlag` reste 0.**

## Probes runtime (gdb-remote via QEMU monitor `gdbserver tcp::1234`)

1. `irq_handlers[6]` @ 0x008304a4 = **0x00822498** ✓ (handler registered)
2. `rxDoneFlag` @ 0x00830510 = **0x00000000** stuck on 10s window (20 polls)
3. ARM PC = 0x00822b90 sustained (rapid burst of 20 samples = all in 0x822b90..b98)
4. PSR = 0x60000113 = svc32, **I=1, F=0** (IRQ disabled — entered FIQ-disable region apparently)
5. `gdb set *(int*)0x830510 = 1; continue` → **ARM exits busy loop, TDMA starts, INTM-TRANS=45, 541 fbsb hooks, bridge cur_qfn=14865+** ⚡

Donc le seul truc qui manque pour débloquer est l'écriture de `1` à `0x830510`. Tout le reste de la chaîne fonctionne **après** cette écriture.

## Hypothèses au choix (où je rame)

**(A) Bug QEMU dans la conditional execution sous icount-auto.** TST/STRNE wouldn't fire correctly. Mais si c'était ça, on verrait des bugs partout dans le boot, pas juste ici. Et ça marche sous icount=off donc pas spécifique à conditional exec.

**(B) FIQ-mode banked registers race.** Quand le handler est en mode FIQ (R8-R12 banked), peut-être que la pile ou un truc spécifique à FIQ32 bouge R2/R3 entre la lecture et la STR ? Improbable — instructions 0x8224a0..ac sont sequentielles.

**(C) Le 5e SIM_IT read NE VIENT PAS de sim_irq_handler.** Maybe le firmware a un autre code path qui lit SIM_IT (genre polling loop quelque part), et ce code path lit IT_WT, le clear, mais ne set pas rxDoneFlag. Le FIQ handler arrive après mais notre SIM_IT (level bits cleared) renvoie 0 → handler ne voit pas IT_WT.

Données qui supportent (C) :
- 5 SIM_IT reads + 5 FIQ_NUM=6 reads — exactement 1:1 (mais ordre temporel n'a pas été vérifié dans le log)
- Si le polling loop lit en premier, le SIM_IT read serait dans le log AVANT le FIQ_NUM. Mais on a 4 FIQ_NUM intercalées entre les 5 reads.
- Le 5e read dit `edge_cleared=0x0002 post_it=0x0000` — IT_WT a été cleared par cette lecture précisément. Si une autre lecture avait précédé, post_it serait déjà 0 avant.

**(D) Bug dans notre INTH FIQ_NUM read.** Maybe le `s->fiq_v` retourne 6 mais le firmware fiq() s'attend à un autre offset/taille. Checked — read returns u16 6 truncated to byte (LDRB by firmware). OK.

**(E) Firmware utilise un GPP register (0xfffffa**) different. Pas exploré.

**(F) Le handler ne run pas réellement à 0x822498.** Le `BXNE R3` dans fiq() avec R3=0x822498 pourrait avoir un bug. Mais on a vérifié `irq_handlers[6]=0x822498` ✓.

## Le diff entre icount=off (marche) et icount=auto (bloque)

Sous icount=off (ne pas tester maintenant pour préserver le run actif), tout boot proprement et arrive à FBSB. Donc le bug n'est PAS structurel — c'est timing-sensitive.

Mon hypothèse forte : **(C) — il existe un POLLING path firmware qui lit SIM_IT EN PARALLÈLE du FIQ handler. Sous icount=off, le timing fait que le polling lit après le FIQ handler (qui a déjà set rxDoneFlag). Sous icount=auto, polling lit AVANT le FIQ handler, clear IT_WT, FIQ handler arrive trop tard et reçoit IT=0**.

Mais notre log montre 5e read = 0x0002. Si polling read avait précédé, le 5e read montrerait 0x0000... à moins que les 5 reads soient TOUS du polling (pas du FIQ) et le FIQ handler vient APRÈS, voit 0x0000, ne set pas rxDoneFlag.

Ça serait cohérent avec :
- 4 polling reads pour les ATR bytes (rx_count 4→1)
- 5e polling read pour IT_WT
- 5 FIQ entries qui lisent FIQ_NUM=6 mais leur SIM_IT read renvoie 0 (déjà cleared) — ces 5 SIM_IT reads ne sont PAS dans notre log car cap=30 hit ?

Cap est à 30, on a 5 entries. Donc il manque potentiellement plein de reads.

## Plan de probe que je propose

Je peux instrumenter de manière distinctive :
1. Log SIM_IT reads avec PC ARM (qui lit) — discrimine FIQ handler (PC ~ 0x82249c) vs polling loop (PC autre)
2. Lever le cap à 200
3. Logger aussi les FIQ entries via gdb breakpoint @ 0x822498

**Question pour toi** :
1. Tu vois l'hypothèse (C) plausible ? Ou autre piste ?
2. Tu connais un site dans osmocom-bb firmware où SIM_IT est polled hors IRQ context ? (calypso_sim_powerup ? layer1 init ?)
3. Le `set *(int*)0x830510 = 1` via gdb débloque tout. **C'est OK comme workaround ou tu vois mieux ?** On pourrait avoir notre fire_wt qui force `cpu_physical_memory_write(0x830510, 0x00000001)` après le delay 2ms VIRTUAL — ugly hack mais ça marche et c'est gating-free.

## State récap

Tasks completed (12) : 5 décodeur Tier A + 4 event-loop fixes + 1 instrumentation INTH + 1 instrumentation kick + 1 SIM IT hardening.

Tasks pending (15) — décodeur Tier B (9) + event-loop Tier B (5) + 1 validation finale.

Files patched (md5) :
- calypso_sim.c `ecda530b` — IT clear hardening + SIM_IT instrumentation
- calypso_trx.c `78102920` — kick instrumentation
- calypso_inth.c `4cc60204` — FIQ/IRQ split
- calypso_inth.h `b73e9931` — fiq_v field
- calypso_uart.c `f1c59fed` — main_loop_wait removed
- calypso_c54x.c `9f5ffe5c` — Tier A decoder (0x76, F2/F3 LMS, 0x80 STL, 0x98/9A swap, 0x8C ST T)
- calypso_c54x.h `0388a368` — writer_kind enum

3-way sync clean (host qemu-calypso = host qemu-src = docker /opt/GSM/qemu-src).
