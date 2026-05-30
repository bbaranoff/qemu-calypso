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

## Architecture FB-dispatch CLARIFIÉE (2026-05-30 soir, post-falsification boot-init)

Expérience CC-web « FB = queue boot-init » **FALSIFIÉE** : 2 modèles de boot
(redirect 0x7120 vs SP-set natural 0xb410), boot-init 0x7000-0x7025 inatteignable
par les deux, FB-dispatch échoue **identiquement** → FB-dispatch = steady-state
SÉPARÉ (pas la queue de c3ec660). Bonus : over-pop = 100% artefact SP=0x1100
(F@0x76f8 tourne propre depuis 0x5AC8).

**Artefact consolidé (table vecteurs IRQ + IMR + disasm idle) tranche l'archi :**
- `0x08d4 = &d_dsp_page` CONFIRMÉ (NDB base ARM 0xFFD001A8 → DSP 0x08D4). Pas un fantôme.
- **Aucun vecteur IRQ (0xff80+vec*4) ne lit 0x08d4.** Les ISR lisent les flags
  `0x3dc0/1/2` (vec19 frame→0x3dc1, vec23→0x3dc0, vec24→0x3dc2). → le dispatch
  d_dsp_page est **FOREGROUND-pollé, PAS IRQ-driven**.
- Frame-IRQ vec19 (imr bit3) **ARMÉ dans 0x52fd + fire** 8339× → **B (masquage) MOOT**.
  (0x52fd masque bits 1/8/10/11/13/15 = vec 17/24/26/27/29/31.)
- Idle 0xe9ac = **bloc MAC ×9** (0xb398, pas un wait passif). 0xf7b2 (reprise RETE)
  = `RC NTC` (épilogue). Le **dispatcher 0xcc62 TOURNE** (polle data[0x60-0x70]).

**Le gap LOCALISÉ (pas résolu), avec mécanisme probable :** le per-frame existe et
s'exécute (dispatcher 0xcc62 polle data[0x60-0x70] chaque frame) — donc PAS une
ré-entrée structurellement absente. C'est le **slot FB de data[0x60-0x70] / flag
0x3dc0-2 qui n'est JAMAIS set**, donc la branche foreground vers le FB-processing
(0x7700 lecture page → 0x9ac0) n'est jamais prise.

**Candidat #1 = C (flag-gate non posé par l'ISR frame).** L'ISR frame @0xffcc
(`0x0100 ; RC NTC@0xffcd ; LD 0x3dc1 ; STL…`) early-return sur TC=0 **AVANT** de
poser le flag (0x3dc0-2 / data[0x60-0x70]) que la branche foreground attend
(= pourquoi 0x3dc0/1 jamais touché, FBDB-PROBE=0). Dispatch foreground + flag-gate
posé par l'ISR sont compatibles : l'ISR pose le flag, le foreground le polle. Le
early-return TC=0 casse le maillon « ISR pose le flag ».

**Prochaine sonde (invasif) — UNE capture causale chaînée, pas 3 passes** (même
réflexe que vecteurs+IMR) : un seul run qui chaîne `TC à l'entrée ISR 0xffcc` →
`corps post-RC@0xffcd (écrit-il le flag 0x3dc0-2 / data[0x60-0x70] ? oui/non)` →
`poll dispatcher 0xcc62 (le slot FB reste-t-il clair ?)`. On voit le maillon cassé
dans le run, pas en recoupant après coup.
**Root à prioriser = décoder `0x0100@0xffcc`** : « pourquoi TC=0 » est la
question-mère ; le poll-foreground et le poseur-de-flag orbitent autour.
**⚠️ NON-VÉRIFIÉ à établir, pas supposer** : « 0x7700 = entrée FB-processing » est
ASSERTÉ (0x7700 touché 1× = incident, load AR-indexé du sweep RPTB), PAS établi.
La sonde doit CONFIRMER où le dispatcher branche réellement pour la FB — sinon on
watch le mauvais PC.

## Brief antérieur CC-web (FB est ARM-commandé, ARM L1 prouvé correct)
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
