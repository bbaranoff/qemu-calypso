---
name: STATUS — état courant Calypso QEMU (FBSB)
description: État du déblocage DSP, dernier root cause résolu, blocker courant
type: project
---

# STATUS — 2026-05-30

## Dernier acquis (RÉSOLU, vérifié) : SP jamais initialisé → wedge 0x70c3

**Root cause.** Le dump PROM ne contient pas le **mask-ROM silicon** du Calypso.
Sur vrai HW il pose SP=0x5AC8 au reset et saute à l'entrée firmware
PROM0[0x7120] (= `STM #0x5AC8,SP`, vérifié : `prog[0x7120]=0x7718`,
`prog[0x7121]=0x5ac8`). L'émulateur modélisait ce ROM par un redirect
`0xff80→0x7120`, **retiré par erreur** en git `c3ec660` (29/05).

Sans le redirect : reset `0xff80(FB)→0xb410→CC→0x76f8`, jamais 0x7120, donc
**SP coincé à 0x1100** (valeur reset, invalide = aire MMR/AR0) → over-pop boot
(net_words -1→-57, slots pile [0x1100-0x1140] tous vierges) → return corrompu →
**self-CALA 0x70c3** → spirale (16M pushes, SP erre sur tout le 16-bit) →
PMST=0x70C4 fuit en **TOA=28868** côté osmocon → FB figé, jamais de lock.

**Fix.** Redirect restauré dans `calypso_c54x.c` (top `c54x_run`) :
```c
if (s->pc == 0xFF80 && s->sp == 0x1100) { s->pc = 0x7120; }
```
Gate `SP==0x1100` = cold-reset seul. **Pas un hack rule#1** : modélise du
hardware réel absent du dump, route vers l'entrée firmware propre qui fait
elle-même son `STM #0x5AC8,SP`.

**Résultat vérifié (run) :**

| | Avant | Après |
|---|---|---|
| SP | spirale 16-bit | STM@0x7120 pose 0x5AC8 (vérifié) → serial-loader grimpe → stable **0x9008** (4B insns, équilibré) |
| push/pop | 16M / 697 | équilibré (33M/33M) |
| wedge 0x70c3 | 3700+ IRQ | **0** |
| IMR | 0x0000 | 0x52fd (INT3 + BRINT0 démasqués) |
| INT3 frame ISR | jamais (IMR=0) | **fire par-frame** |
| TOA=28868 / DSP Error figés | chaque frame | **disparus** |
| L1CTL_FBSB_REQ flags | — | **0x7** (FB0\|FB1\|SB) |

→ Le « dernier bug 0x70c3 » des sessions précédentes est **résolu** (c'était ce
root SP, pas un over-pop désapparié). Les 37/40 « ORPHAN » du shadow-stack
étaient des faux positifs (boot-ROM-stub serial-loader, pc=0x0000, légit).

## Blocker COURANT : FB-dispatch (sur base saine)

`d_fb_det` reste **0x0000** → pas de FB lock. Le DSP boote sain mais **idle
minimal** (0xe9ac / dispatcher 0xcc62). `task=5` est commandé (`ARM TASK WR
[0x08]=5 ET [0x30]=5` ×491) MAIS :

- DSP **ne lit jamais `d_task_md`** (sonde D_TASK_MD-RD = 0 hit sous ALL).
- ISR INT3 @0xffcc : `0x0100` (set TC ?) puis **`RC NTC@0xffcd`** (cond 0x20 =
  return si TC==0) → early-return (duration=1 ×8088 ; duration=3 court ×7893) →
  **n'atteint jamais** les CALLs (0xf310/0xffc3/0xf307), ni le flag **0x3DC0**
  (FBDB-PROBE = 0 r/w), ni le gate **data[0x62]** (reste 0).
- détecteur FB **0x9ac0 = 0 run** (les 42× au boot d'avant = artefact du boot
  corrompu, pas une vraie détection).

Chaîne jamais atteinte :
```
boot sain → idle 0xcc62 → INT3 → ISR 0xffcc → RC NTC (TC=0) → RETE sec
  → JAMAIS : read d_task_md, test 0x3DC0 bit4, gate data[0x62] → 0x9ac0 → d_fb_det
```

## Prochaine campagne — brief CC-web (FB est ARM-commandé, ARM L1 prouvé correct)
Le FB est une tâche commandée par l'ARM (`d_task_md`=5). L'ARM L1 est la couche
prouvée correcte end-to-end (L1CTL MITM, faute isolée au DSP) → la commande FB
est probablement émise correctement, le mur est en **aval (DSP/API)**. Entrée
haute-leverage = « la commande FB arrive-t-elle, et où se perd-elle entre l'ARM
(bon) et le test TC », PAS du RE générique « pourquoi le DSP idle ». Arbre :

1. **Décoder ce que `0x0100@0xffcc` teste vraiment** → quel prédicat gate le TC
   (le early-return `RC NTC@0xffcd`).
2. **Ce prédicat dérive-t-il de `d_task_md` (commande ARM) ?**
3. Trois issues :
   - ARM n'émet pas FB → mur state-machine cell-search ARM (contredirait « L1
     prouvé » → relire ce que le MITM couvrait vraiment).
   - ARM émet, DSP lit le **mauvais/périmé** buffer → handshake double-buffer.
     **`d_dsp_page`** (innocenté du wedge DP) redevient suspect n°1 : c'est
     l'index « quel buffer de commande est courant cette frame ». Réutiliser le
     RE déjà fait dessus.
   - ARM émet, DSP lit le bon buffer, TC échoue quand même → test/décode mal
     évalué (le thème récurrent : instruction de test mal émulée).

cf `BOOT_TO_FBSB_SEQUENCE.md`, `FBSB_SEQUENCE_TRACE.md`, `TODO.md`.
