# TODO — chemin FBSB QEMU Calypso

## Status 2026-04-29 (see `SESSION_20260429.md`)

5 structural fixes appliqués et validés empiriquement (cf. `PROJECT_STATUS.md` § Latest):
- ✓ Silicon-aligned reset (PMST=0xFFA8, ST0=0x181F, ST1=0x2900)
- ✓ 0x6F00 ext dispatch — wedge PC=0x8353 éliminé (2.2G iter → 0)
- ✓ 0x68-0x6E handlers (ANDM/ORM/XORM/ADDM/BANZ/BANZD) — 1563 sites
- ✓ APTS misnomer fix (bit 4 PMST = AVIS, not APTS) — stack leak 1.96M → 0
- ✓ F3xx complete dispatch — 364 sites, wedge PC=0x8eb9 unblocked

**Bloqueur restant** : INTM=1 forever. Mécanisme silicon non identifié dans la
doc publique (TI Calypso DBB datasheet privée). DIAG-HACK reframé comme
**diagnostic instrumental documenté**, pas workaround.

Pistes pour la prochaine session : voir `SESSION_20260429.md` §
"Suggested investigation paths".

---

## ⚠️⚠️⚠️ DIAGNOSTIC INSTRUMENT — INTM CLEAR ⚠️⚠️⚠️

**Fichier** : `hw/arm/calypso/calypso_c54x.c` (block "DIAGNOSTIC HACK" ~ligne 3908,
après `insn_count++`).
**Patch isolé** : `hack.patch` à la racine du repo.

**Quoi** : env var `CALYPSO_FORCE_INTM_CLEAR_AT=N` force `ST1.INTM ← 0` à
l'instruction n°N. Permet de tester l'hypothèse catch-22 :

- Si **résultat 1** (dispatcher fire → 0xa86b → 0x4189 posé → CALAD débloqué
  → RETED ≥1 → d_fb_det écrit → FB1/FB2 print) : la racine est uniquement
  "boot init n'exécute jamais RSBX INTM". Chercher le path α (RSBX manqué
  dans la disasm) ou γ (CALL mal émulée).
- Si **résultat 2** (dispatcher fire mais autre crash) : INTM était bien le
  blocage immédiat, mais d'autres bugs en aval.
- Si **résultat 3** (rien ne change) : INTM n'est pas (le seul) blocage,
  repivoter.

**Activation** : `CALYPSO_FORCE_INTM_CLEAR_AT=100000 bash run_new.sh` (par
exemple). Désactivé par défaut.

**Critère de retrait OBLIGATOIRE** :
1. La vraie séquence RSBX INTM du boot DSP est identifiée et émulée
   correctement (sans cette force-clear).
2. Le run sans `CALYPSO_FORCE_INTM_CLEAR_AT` passe les indicateurs
   (RETED ≥1, d_fb_det écrit, FB1/FB2 imprimé).
3. Le block est physiquement retiré du fichier `.c` et `hack.patch` est
   supprimé.

## TEMP DIAG résolu : `BSP_FN_MATCH_WINDOW`

Bumpé 64 → 1024 puis restauré à 64. Test confirmé : la fenêtre n'est pas
le bottleneck. STALE passe à 0 mais DMA n'augmente pas (1.5 % → 1.8 %).
Le vrai bottleneck est la chaîne BDLENA et l'enable IMR/INTM côté firmware.

État au 2026-04-26 fin de soirée.

## TL;DR session 2026-04-26

**5 fixes architecturaux appliqués** (détails plus bas) :
1. ✅ Aliasing `s->data` ↔ `api_ram` (mailbox ARM↔DSP cohérente)
2. ✅ INT_CTRL.DSP_FRAME vs TPU_CTRL.DSP_EN (INT3 path correct)
3. ✅ Stub boot ROM 0x0002-0x007F = FRET (pas NOP, casse le runaway CALAA)
4. ✅ INT3 fire IMR-gated (gate sur `IMR & (1<<3)` au lieu de `dsp_init_done`)
5. ✅ FAR opcodes FB/FCALL/FBD/FCALLD (set XPC + push iff APTS)

**3 bugs latents documentés** (à fixer post-LU) :
1. ⏳ `c54x_reset` MVPD memcpy au reset (devrait être opcode-driven)
2. ⏳ `prog_fetch` ignore XPC pour ≥0x8000 (latent jusqu'à cross-page hit)
3. ⏳ Aliasing data/api_ram via 3 paths fragiles (vrai fix : backing store unique)

**Diagnostic actuel** :
- **Sans hack** : DSP piégé init loop PROM0/DARAM (zone hot 0x10ab-0x10c1).
  XPC=0 forever, INTM=1 forever, IMR=0 forever, F6BB=0.
- **Avec hack `CALYPSO_FORCE_INTM_CLEAR_AT=2000000`** : breakthrough majeur.
  Le firmware exécute toute son init runtime (3310 IMR changes, FCALL FAR,
  XPC=0/2/3, PMST 0xFFA8 → 0x0000, IMR=0xFFFF). Confirme que le **primum
  movens manquant est uniquement le clear INTM initial**.
- **Nouveau bloqueur post-hack** : firmware switche en mode applicatif
  (OVLY=0, IPTR=0) puis fetch à PC=0x2FA5 (zone non chargée → NOP slide).
  Et IRQ pointe vers 0x0000-0x007F = nos stubs FRET (ISR vides).
- **Catégorie de travail change** : avant = diagnostiquer un bug, après =
  implémenter les routines boot ROM TI manquantes + identifier source du
  code à 0x2800+. Voir section "DSP Boot Mechanism — breakthrough" plus bas.

**Prochaine investigation** :
- Lire **SPRU172C** (TMS320C54x DSP CPU and Peripherals Reference) et
  **SWPU131G** (?) ou la doc Calypso DSP boot ROM TI.
- Identifier le mécanisme normal de transition init → runtime sur silicon.
- Probablement : NMI au boot, ou primitive boot ROM TI à 0x0000-0x007F
  qu'on stub en FRET (au lieu d'implémenter le vrai comportement).
- Peut-être aussi : examiner le code applicatif ARM osmocom-bb pour voir
  s'il y a une commande "DSP go" / "DSP start applicative mode" qu'on rate.

## DSP Boot Mechanism — breakthrough 2026-04-26 fin de session

### Finding 1: INTM auto-clear est le primum movens

**Confirmé par expérimentation hack** : forcer `INTM=0` une seule fois
(via `CALYPSO_FORCE_INTM_CLEAR_AT=2000000`) **débloque toute la chaîne
d'init runtime** :

| Indicateur | Avant hack | Après hack (insn=2M) |
|---|---|---|
| `IMR change` | 0 | **3310** |
| `FCALL FAR` | 0 | **30** |
| `FCALLD FAR` | 0 | 1 |
| **XPC values** | 0 | **0, 2, 3** (PROM2, PROM3 atteints) |
| `IDLE` | 0 | 1 |
| `d_fb_det` writes | 0 | 5 |
| `IMR=0xFFFF` | jamais | observé |
| `PMST=0x0000` (IPTR=0, OVLY=0) | jamais | observé |

→ Le firmware DSP **sait quoi faire** post-clear. Il était juste bloqué
par un primum movens INTM qu'on n'a pas identifié sur silicon.

**Hypothèses pour le mécanisme silicon réel** :
- **(a) NMI initial** déclenché par signal ARM post-DSP power-on. NMI
  est non-maskable → force le clear INTM via le mécanisme RETE.
- **(b) Boot ROM TI** à 0x0000-0x007F contient une vraie routine avec
  `RSBX INTM (0xF6BB)`. Notre stub FRET ne le fait pas.
- **(c) Sémantique reset** sur une écriture MMIO ARM spécifique (e.g.
  TPU_CTRL.RESET avec timing précis qui clear ST1.INTM hardware-side).

**Investigation** : SPRU131G + datasheet Calypso boot + osmocom-bb ARM
firmware (séquence post `dsp_pll_set` / `dsp_dwload`).

### Finding 2: Post-clear, zones mémoire pas implémentées

Une fois INTM clear et firmware en mode applicatif, il switche vers :
- **PMST=0x0000** → OVLY=0, IPTR=0
- Code applicatif à **PC ≥ 0x2800** (zone non chargée chez nous)
- **Vecteurs IRQ à 0x0000-0x007F** (= nos stubs FRET)

PC HIST late : `2fa5:1786612` (DSP coincé en NOP slide à 0x2FA5 parce
que `s->prog[0x2FA5]` = 0).

Conséquences :
- Avec IPTR=0, IRQ INT3 dispatche à 0x0000+19*4 = **0x004C** = notre
  FRET stub. ISR retourne immédiatement, n'efface pas INTM →
  catch-22 #2 (boucle après le breakthrough).
- Code à 0x2FA5+ doit venir d'une source qu'on n'émule pas (SARAM
  externe Calypso ? upload ARM via mécanisme non-implémenté ?).

**À implémenter** :
- Vraies ISR stubs à 0x0000-0x007F (au minimum RETE pour clear INTM
  via stack restore).
- Identifier source du code à 0x2800+ : `dsp_dwload` osmocom-bb ?
  zone SARAM externe ?

### Finding 3: Bilan stratégique de la session

Avant aujourd'hui : émulateur qui boote bizarrement, comportement opaque.
Après aujourd'hui : émulateur dont on **comprend** les bugs résiduels.

**5 fixes structurels** appliqués (voir TL;DR) + **diag hack** qui prouve
que le firmware est sain. Le LU n'est pas tombé mais on est passé de
"magie noire" à "ingénierie traçable avec liste claire de ce qui reste".

Le breakthrough INTM montre que la chaîne complète fonctionne. Reste
~2 implémentations bien identifiées (boot ROM TI ISRs + code source
0x2800+) avant LU.

## Bilan session 2026-04-26 — 5 bugs racine identifiés (détails)

Cette session a fait passer le projet de "émulateur qui boote bizarrement" à
"émulateur dont on comprend les bugs résiduels". Cinq bugs d'architecture
émulation isolés, chacun expliquant des heures de comportements bizarres
antérieurs.

### Bug racine #1 — Aliasing `s->data` ↔ `api_ram` cassé   ✅ FIXÉ

**Symptôme** : DSP exécute du code obsolète quand l'ARM écrit dans la
mailbox API RAM, parce que `prog_fetch` lit `s->data[]` mais `data_write`
ARM-side n'écrivait que `api_ram[]`.

**Preuve** : 57/57 divergences `s->data[0x10c0-0x10f8]` vs `api_ram[0x8c0-0x8f8]`
au snapshot diag-hack insn=10000 (run précédent fix).

**Fix** :
- `c54x_reset` MVPD memcpy mirror `s->prog → s->data ET s->api_ram` pour la
  range `[0x0800, 0x2800)` (calypso_c54x.c:4192).
- `calypso_dsp_write` ARM-side mirror `s->dsp_ram → s->dsp->data` pour
  l'offset DSP-word correspondant (calypso_trx.c:140).

**Effet mesuré** : 0/57 divergences post-fix. Mailbox cohérente. FRET 1→178.
DP-SET 2→148 209.

### Bug racine #2 — MVPD copie au reset au lieu d'opcode   ⏳ TODO

**Symptôme** : Le runtime reset DSP (déclenché par ARM via DSP_DL_STATUS)
ré-exécute la boucle MVPD memcpy, écrasant la mailbox que l'ARM venait
d'écrire (BL_ADDR_LO=0x7000, etc.).

**Cause** : Sur silicon réel, MVPD est un opcode DSP (`0x8Cxx`), pas un
effet de reset. Notre `c54x_reset` simule le boot ROM TI par memcpy
direct, ce qui crée un effet de bord au runtime reset.

**Risque actuel** : Latent — le DSP a déjà BACC'd à 0x7000 avant le
runtime reset dans nos runs, donc pas de manifestation. Mais soft-reset
post-active cassera.

**Fix proposé** : voir section "Dette technique post-LU" plus bas.

### Bug racine #3 — INT_CTRL vs TPU_CTRL pour frame IRQ   ✅ FIXÉ

**Symptôme** : Le DSP frame interrupt (INT3, vec 19) ne fire jamais
(count=0) malgré la chaîne BSP DMA fonctionnelle.

**Cause** : Notre émulation gatait INT3 sur `TPU_CTRL.DSP_EN` (bit 4) qui
est le mécanisme one-shot/force (utilisé seulement par
`tpu_force_dsp_frame_irq`). Le firmware utilise au boot
`tpu_frame_irq_en(1, 1)` qui écrit `INT_CTRL.ICTRL_DSP_FRAME` (bit 2,
polarité INVERSÉE : bit clear = enabled). Deux registres différents.

**Preuve** : Sur 50 écritures TPU_CTRL, **zéro** avec DSP_EN=1. Sur
30 écritures INT_CTRL captées (post-fix log), **firmware enable bien**
DSP_FRAME (val=0x02 = MCU+DSP frame both enabled).

**Fix** :
- `calypso_trx.c:456` : INT3 fire si `INT_CTRL.ICTRL_DSP_FRAME == 0`
  (persistent, ne pas le clear) **OU** `TPU_CTRL.DSP_EN == 1`
  (one-shot force, clear sur fire).

**Effet mesuré post-fix** : INT_CTRL writes captés. Mais INT3 toujours
gated par `dsp_init_done` qui n'est jamais TRUE — voir bug #4.

### Bug racine #4 — Stub boot ROM 0x0000-0x007F = NOPs   ⏳ EN COURS

**Symptôme** : Boucle CALAA infinie `src=0x008c → tgt=0x0052` avec
stack runaway (SP descend 0x9000 → 0x1278 sur 200K CALAs). DSP finit
en NOP slide PC=0x0000-0x0001 (1M visites).

**Cause** : Le firmware fait `CALA A` avec A=offset dans la boot ROM TI
(0x0000-0x007F). Notre stub est rempli de `0xF495` (NOP) sauf 0x0000
(LDMM SP,B) et 0x0001 (RET). Quand CALAA cible un offset > 1, le DSP
NOP-slide à travers le stub jusqu'à fall-through dans le code overlay,
recréant une boucle.

**Fix appliqué** : Remplacer NOPs par `0xF4E4` (FRET) à 0x0002-0x007F.
Chaque CALAA-into-stub retourne immédiatement au caller.

**Effet mesuré post-fix** :
- Boucle CALAA → 0x0052 disparue
- SP stable au lieu de descendre
- d_fb_det/0x4189 plus de bogus writes
- PC HIST late = 0x7700/0x7701 PROM0 (vs NOP slide avant)
- IDLE atteint 1× (vs 0× souvent)
- **MAIS** `dsp_init_done` reste FALSE (s->idle reset par BRINT0 wake-up
  avant que calypso_trx.c:443 ne le voie). À régler prochaine étape.

### Bug racine #7 — FAR opcodes ne settent pas XPC   ✅ FIXÉ

**Symptôme** : Dans la disasm firmware on voit 100+ × `FCALL` (0xF980+),
67 × `FB` (0xF880+), quelques `FCALLD` et `FBD`, mais **XPC reste à 0
toute la durée du run**. DSP ne traverse jamais PROM1.

**Cause** : Notre code décodait :
- F880-F8FF (FB FAR per tic54x-opc.c) comme `B pmad` NEAR (juste set PC)
- F980-F9FF (FCALL FAR) comme `CC pmad` NEAR conditionnel
- FA80-FAFF (FBD FAR) comme branch simple
- FB80-FBFF (FCALLD FAR) comme `CCD pmad` NEAR

Aucun ne setait XPC. Le bit 7 du low byte distingue NEAR vs FAR mais
notre handler l'ignorait.

**Source de vérité** : `binutils-2.21.1/opcodes/tic54x-opc.c` lignes 238-241 :
```
{ "fb",     0xF880, 0xFF80, OP_xpmad, FL_FAR }
{ "fbd",    0xFA80, 0xFF80, OP_xpmad, FL_FAR|FL_DELAY }
{ "fcall",  0xF980, 0xFF80, OP_xpmad, FL_FAR }
{ "fcalld", 0xFB80, 0xFF80, OP_xpmad, FL_FAR|FL_DELAY }
```

OP_xpmad = adresse étendue 23-bit. Bits 0..6 du premier mot = high bits
de XPC ; second mot = low 16 bits de PC. Calypso utilise XPC 2-bit donc
`(op & 0x7F) & 0x3`.

**Fix appliqué** :
- `calypso_c54x.c:1559` : F880-F8FF (FB FAR) — set XPC + PC
- `calypso_c54x.c:1880` : F980-F9FF (FCALL FAR) — push PC (+ XPC iff
  APTS), set XPC + PC
- `calypso_c54x.c:1957` : FA80-FAFF (FBD FAR) — set XPC + delayed PC
- `calypso_c54x.c:1977` : FB80-FBFF (FCALLD FAR) — push PC+4 (+ XPC iff
  APTS), set XPC + delayed PC
- `calypso_c54x.c:944, 963` : RETE/FRET pop XPC iff APTS — symétrie

**Effet mesuré post-fix** :
- FB FAR fire 67 fois (avant : 0 hit en tant que FB FAR — décodé en B)
- Mais TOUS targets XPC = 0 → 0 (firmware utilise FB pour rester en page 0)
- FCALL FAR / FCALLD FAR : 0 — firmware ne fait pas de cross-page call
  dans le path qu'il exécute actuellement
- Bug racine plus profond : firmware ne progresse pas vers les sites
  où FCALL serait utilisé.

### Bug racine #5 — `prog_fetch` ignore XPC pour ≥0x8000   ⏳ LATENT

**Symptôme** : Quand DSP fait CALL pmad cross-page (XPC=1), l'instruction
fetchée vient de la mauvaise page car `prog_fetch` lit `s->prog[pc]` sans
banking XPC, alors que `prog_read` (data/operand) le fait.

**Status** : Bug confirmé par lecture du code (calypso_c54x.c:540-551).
Pas de manifestation dans les runs actuels (XPC=0 toute la durée). Va
mordre quand le firmware atteindra le path PROM1 dispatcher 0x1a7c4.

**Fix** : Étendre `prog_fetch` pour utiliser `(xpc << 16) | pc` quand
`pc >= 0x8000`, mirror de `prog_read`.

## Prochaine investigation (post-pause)

**Question ouverte** : par quel mécanisme le firmware DSP normal sort-il
de l'init loop PROM0 pour atteindre PROM1 ?

Pistes à explorer dans cet ordre :

1. **Lire SPRU172C** (TMS320C54x DSP CPU and Peripherals Reference) sur
   le boot ROM TI interne. Calypso a un boot ROM à 0x0000-0x007F qu'on
   stub en FRET. Sur silicon réel, ce boot ROM contient des **routines
   utiles** appelées par le firmware via `CALA A=offset_boot_rom`. Si
   on les implémente correctement (au lieu de FRET), le firmware
   reçoit les services qu'il attend → progresse hors du init loop.
   - Identifier quelles routines TI sont à 0x0000-0x007F
   - Implémenter celles utilisées (memcpy, init reg, etc.)

2. **NMI au boot** : sur Calypso, le power management ARM peut lever
   NMI (vec 1) sur le DSP à un moment précis du boot. NMI est non-
   maskable, donc force le clear INTM. Notre émulation ne lève pas NMI.
   - Identifier quelle écriture MMIO ARM devrait lever DSP NMI
   - Implémenter dans calypso_trx.c

3. **Mailbox DSP "go" applicatif** : le firmware ARM osmocom-bb pourrait
   écrire une commande spécifique APRÈS le bootloader handshake
   (BL_CMD=2 → BACC 0x7000) pour faire passer le DSP en "mode
   applicatif". Cette commande est peut-être un slot NDB qu'on rate.
   - Trace toutes les écritures ARM API RAM entre fn=0 et fn=200
   - Comparer à la séquence attendue per `dsp_power_on()` osmocom-bb
   - Identifier celle qu'on rate ou qui ne se reflète pas côté DSP

4. **Disasm ciblée** : la zone hot DARAM 0x10ab-0x10c1 (où le DSP boucle).
   Identifier ce qu'attend cette routine et qui devrait l'écrire/déclencher.

## Décision empirique 2026-04-26 — INT3 fire gated par IMR bit 3

**Tension d'interprétation silicon** identifiée et tranchée par observation :

**Lecture A** (SPRU131G strict, "purist") :
- Le hardware INT line lève IFR à chaque pulse, indépendamment d'IMR
- IDLE exit sur ANY interrupt (masked or unmasked)
- IMR ne gate que le dispatch (saut au vector), pas le set IFR
- → Notre `c54x_interrupt_ex` doit fire dès `INT_CTRL.DSP_FRAME == 0`,
  pas de check IMR

**Lecture B** (Calypso-spécifique, observée) :
- Sur Calypso, le path TPU INT_CTRL → DSP IFR pourrait être gated par
  IMR au niveau hardware (la propagation est filtrée avant IFR)
- → `c54x_interrupt_ex` ne doit fire que si IMR autorise le bit

**Empirie** :
- Avec lecture A (pas de check IMR) : régression observée (firmware part
  dans un path bizarre — 0x838X / 0x79FX —, n'atteint plus 0x0810
  d'init IMR, IMR change=0, IDLE=0, BRINT0=0).
- Avec lecture B (check IMR bit 3) : firmware path stable, 0x0810
  atteint, IMR posé, IDLE atteint, BRINT0 fire.

**Choix appliqué** : Lecture B (check IMR bit 3 dans
`calypso_trx.c:474-478`). Pragmatique : on respecte le comportement
qui ne perturbe pas le firmware. Si plus tard on trouve une source
TI authoritaire qui tranche A, refactor.

**Critère de retrait du check IMR** :
- Datasheet TI Calypso (pas SPRU131G générique) confirmant que la
  propagation INT_CTRL → IFR n'est PAS gated par IMR
- ET le firmware reste stable sans le check (pas de régression PC HIST)

## Bug racine #6 — race `dsp_init_done` vs BRINT0 wake-up   ✅ FIX CONTOURNÉ

**Symptôme** : `dsp_init_done` jamais TRUE même si IDLE atteint, INT3 fire
gated dessus reste fermé.

**Cause** : Le détecteur "first IDLE → dsp_init_done = TRUE" dans
`calypso_tdma_tick` (calypso_trx.c:443) sample `s->dsp->idle` périodiquement.
Mais `c54x_interrupt_ex` lors d'un BRINT0 raise (calypso_bsp.c:447) reset
`s->idle = false` immédiatement (wake-up DSP standard). Race classique :
DSP entre IDLE → BRINT0 raise par DMA dans le même tick → s->idle remis
à false → tdma_tick voit idle=false → init_done jamais latché.

**Fix appliqué (contournement)** : virer le gate `dsp_init_done` sur le
fire INT3 (calypso_trx.c:473). Faire confiance au firmware : si
`INT_CTRL.DSP_FRAME` est armé, le firmware veut l'IRQ. INTM=1 protège
contre fire prématuré (l'IRQ stay pending dans IFR jusqu'à clear INTM).

**Fix structurel post-LU** :
- (a) Utiliser un flag séparé `dsp_was_idle_at_least_once` (sticky, jamais
  reset) à la place de `s->dsp->idle` pour la latch.
- (b) Ou poser `dsp_init_done = true` directement dans le handler IDLE de
  `calypso_c54x.c:954`, pas via sampling dans tdma_tick.





## Dette technique post-LU (à refactorer après le premier FB1/FB2)

### `c54x_reset` MVPD memcpy — modèle incorrect

`c54x_reset` exécute aujourd'hui une boucle `s->data[0x0080+i] = s->prog[0x7080+i]`
pour les 0x2780 mots, censée simuler "boot ROM MVPD". **Sur silicon réel, MVPD
est un opcode déclenché par le DSP lui-même** (instruction `0x8Cxx`), pas un
effet de reset. Conséquence dans notre modèle :

- Le runtime reset (déclenché par ARM via `DSP_DL_STATUS = BOOT`) ré-exécute le
  memcpy → écrase les écritures ARM dans la mailbox `0x0FFC-0x0FFF` (et toute
  écriture ARM dans la zone overlay 0x0080-0x27FF) faites avant le reset.
- Aujourd'hui ça ne casse rien parce que le DSP a déjà BACC'd à 0x7000 avant
  ce reset, mais ça pourrait foirer en cas de soft-reset après une session
  active, ou de re-boot DSP en cours de run.

**Refactor** : choisir une de ces options :
- (a) supprimer la copie MVPD au reset, laisser le DSP le faire via opcode
  comme sur silicon (vérifier d'abord que le boot ROM TI émet bien un MVPD
  qu'on capture)
- (b) la conditionner à la première exécution post-boot uniquement (flag
  `mvpd_already_done` qui survit au reset)
- (c) préserver explicitement la mailbox `[0x0FFC, 0x0FFF]` lors du re-MVPD

### Aliasing data/api_ram : refactor structurel

Le fix appliqué (mirror dans `c54x_reset` MVPD + mirror dans `calypso_dsp_write`)
maintient la cohérence par **trois paths qui doivent tous être à jour** :
1. DSP-side `data_write` (déjà ok, fall-through à `s->data[addr] = val`)
2. ARM-side `calypso_dsp_write` (fix : mirror vers `s->dsp->data`)
3. Reset MVPD (fix : mirror vers `s->api_ram`)

C'est fragile. Le vrai fix structurel est l'**aliasing au niveau allocation** :
`s->data + 0x0800` et `s->api_ram` (= `s->dsp_ram`) doivent pointer sur le
même backing store. Un seul `g_malloc0`, deux pointeurs qui se chevauchent.
Élimine les trois paths de synchro.

### `prog_fetch` ignore XPC pour ≥0x8000

`prog_fetch` (ligne ~540) lit `s->prog[pc]` sans tenir compte de XPC, alors
que `prog_read` honore XPC pour `addr16 ≥ 0x8000`. Bug latent : quand le DSP
sera vraiment en page 1 (XPC=1) et fera un CALL pmad cross-page, l'instruction
suivante sera fetchée de la mauvaise page (PROM0 data au lieu de PROM1 code).
Pas de manifestation actuelle (XPC=0 toute la durée du run), mais à fixer
avant que le firmware atteigne ce path.

## État courant

Le bootloader DSP termine enfin son handshake avec l'ARM. Le DSP exécute le user
code à PROM0 0x7000 puis visite plusieurs zones DARAM (0x10ab → 0x148d → ...)
**et au moins une adresse hors mémoire** (0x2f13 ∈ 0x2800-0x6FFF unmapped → lit
0x0000 ; à instrumenter, possible NOP slide latent). Plus de stack-overflow,
plus de wait loop infini. La BTS ne crash plus pour clock skew. **FB1/FB2 jamais
imprimé**, et **INTM=1 forever confirmé** par les indicateurs (RETED=0, RETE=0,
DSP WR d_fb_det=0). Le diagnostic dispatcher PROM1-mirror 0xeb04-0xeb0b décrit
dans CLAUDE.md est **résolu** (le DSP a fait XPC vers PROM0 et exécute du code
dynamique) — la section "Current Bug" / "Next session entry points" de
`CLAUDE.md` est **caduque** et doit être réécrite.

## Pipeline utilisé

```
docker exec -it trying bash -c "killall -9 qemu-system-arm 2>/dev/null; \
  rm -f /tmp/osmocom_l2 /tmp/qemu-calypso-mon.sock /tmp/qemu_l1ctl_disabled; \
  bash /opt/GSM/qemu-src/run_new.sh"
```

Pour injecter des FB bursts directement (diagnostic) :
```
docker exec trying python3 /opt/GSM/qemu-src/scripts/inject_fb.py
```

## Fixes appliqués cette session (cumul)

| Fichier | Fix | Effet mesuré |
|---|---|---|
| `bridge.py` | `fn = bts_fn` (anchor double supprimé) | delta 797 → 24 |
| `bridge.py` | CLK IND wall-clock (471 ms) | BTS ne shutdown plus pour PC clock skew |
| `calypso_bsp.c` | default daram_addr = 0x3fb0 | DSP lit le bon buffer BSP |
| `calypso_bsp.c` | BSP_FN_MATCH_WINDOW = 64 | Stale ratio /10 |
| `calypso_timer.c` | byte access + sémantique CNTL silicon (bit 5 CLOCK_ENABLE, prescaler 4:2) | "LOST 0!" disparaît |
| `calypso_timer.c` | mode lazy (count interpolé depuis virtual time) | LOST diff converge vers ~1885 (au lieu de chaotique) |
| `calypso_c54x.c` | opcode SFTC `0xF494/F594` | Fin du F4xx unhandled spam |
| `calypso_c54x.c` | F4E4 = vrai FRET (workaround retiré) | DSP retourne de 0x770c à 0xb41a |
| `calypso_c54x.c` | F6E2/F6E3/F6E4/F6E5/F6E6/F6E7/F6EB/F69B (delayed B/CALA/RET/RETED) | Returns from interrupt enabled |
| `calypso_c54x.c` | STLM 1-word pour 0x88xx/0x89xx | PC sync correct, plus de skip d'instruction |
| `calypso_c54x.c` | case 0x1 = LD/LDU/LDR (était SUB faux) | A correctement chargé |
| `calypso_c54x.c` | case 0x0 = ADD/ADDS/SUB/SUBS proper | Plus de << 16 implicite |
| `calypso_c54x.c` | resolve_smem MOD 12-15 réordonnés (15 = `*(lk)` absolute) | Bootloader lit BL_ADDR_LO=0x7000 |
| `calypso_c54x.c` | CMPM 0x60xx + BITF 0x61xx (sets TC) | Wait loop bootloader sort enfin |

## Tracers ajoutés

- `[c54x] DYN-CALL` : tous les BACC/CALA (F4E2/F5E2/F4E3/F5E3) avec data[]/prog[] dump
- `[c54x] FRET #N` : exec FRET (PC retour, SP, XPC)
- `[c54x] RETED #N` : exec RETED
- `[c54x] WATCH-READ #N data[0x0ffc..0x0fff]` : data vs api_ram (mailbox bootloader)
- `[c54x] WATCH-READ d_fb_det[0x08F8]` : real d_fb_det (était 0x01F0 erroné)
- `[c54x] WAIT-3DD0` : DSP polls data[0x3dd0]
- `[c54x] DISP-PTR data[0x3f65]` : dispatcher pointer
- `[calypso-trx] BL ARM WR` : ARM-side writes à BL_ADDR_HI/SIZE/LO/CMD_STATUS

## Indicateurs au dernier run

| Métrique | Valeur |
|---|---|
| FRET count | 1 |
| RETED count | 0 |
| RETE count | 0 |
| DSP WR d_fb_det count | 0 |
| DYN-CALL count | 1 (BACC à 0x7000 ✓) |
| IDLE count | 0 (DSP run vraiment) |
| IRQ INT3 servies | 0 (INTM=1 toujours bloque) |
| FB1/FB2 print firmware | 0 |
| **RSBX INTM exec count (F6BB)** | **à mesurer dès Priorité A #1** |
| **SSBX INTM exec count (F7BB)** | **à mesurer (utile pour ISR entrée)** |
| **PC visite hors-mémoire (0x2800-0x6FFF)** | **à mesurer (0x2f13 suspect)** |

## Bug racine restant

**INTM=1 forever** : aucun RSBX INTM (`0xf6bb`) jamais exécuté. Le DSP visite
maintenant énormément de zones DARAM différentes (0x10ab → 0x148d → 0x2f13 →
...) mais on ne voit jamais visite à PROM0 0xa4d0/0xa510/0xa6c0/0xc660/...
(addresses des RSBX INTM par CLAUDE.md).

Sans clear INTM, l'INT3 frame interrupt fire mais est rejetée, le DSP ne
process pas task_md=5, n'écrit pas d_fb_det.

## TODO ordre priorité

### Priorité A — Atteindre RSBX INTM

**#1 Tracer où le DSP devrait normalement faire RSBX INTM**
- Toutes les RSBX INTM sont dans les ISR (INT3, BRINT0, etc.) selon CLAUDE.md
- Pour entrer dans une ISR, il faut INTM=0. Catch-22.
- Au boot, INTM=1 par défaut (reset value de ST1).
- **Hypothèse** : le code uploadé en DARAM par le DSP user contient un RSBX
  INTM dans son init. Il faut le trouver. Tracer 0xf6bb à chaque exec.
- **Action** : ajouter dans calypso_c54x.c F6Bx handler un log
  `RSBX/SSBX bit=B PC=… ST1=…` pour **tous les F6Bx et F7Bx** (16 bits chacun).
  L'inventaire complet coûte rien et donne d'un coup le profil
  set/clear pour OVM/SXM/C/TC/INTM/HM/XF/CMPT/CPL/...
- **Critère de validation** : l'opcode 0xF6BB (RSBX bit=11=INTM) est exécuté
  ≥1 fois ⇒ INTM peut passer à 0 ⇒ INT3 servi ⇒ RETED count ≥1.
- **Plan B si compteur F6BB reste 0** : la racine n'est plus l'opcode mais
  le DSP qui n'arrive pas à exécuter l'init normale. Pivoter vers (a)
  audit du reset vector / boot stub `0x0000-0x007F`, (b) vérifier que XPC
  switch fait vraiment basculer page lors des CALL cross-page,
  (c) inspecter quel flag bloque l'exécution avant les RSBX INTM.

**#2 Vérifier CMPM/BITF / autres opcodes manquants**
- L'audit a révélé que beaucoup d'opcodes 0x60xx-0x67xx sont mal couverts
  (seul 0x60/0x61 sont fixés). Voir tic54x-opc.c lignes pour 0x6200-0x67FF :
  MPY Smem,lk; MAC Smem,lk; etc. À faire selon besoin si DSP spam unimpl.

**#3 Continuer audit case 0x2-0x9 du switch** (scope élargi)
- case 0x2, 0x3 : MAC/MAS/MPY variants (vérifier le shift et les flags)
- case 0x4, 0x5 : ADD/SUB Smem,SHIFT,SRC1
- case 0x6 : 0x6000-0x60FF CMPM ✓, 0x6100-0x61FF BITF ✓ ; 0x6200-0x67FF
  (MPY Smem,lk / MAC Smem,lk / etc) à compléter dès qu'un opcode unimpl
  remonte sur ce range.
- case 0x9 : LD Xmem,SHFT,DST etc
- Beaucoup d'opcodes PROM peuvent être mal décodés silencieusement.

### Priorité B — Si INT3 servies

**#4 BSP delivery efficiency**
- STALE ratio toujours élevé (~50:1) malgré window=64
- Si le DSP atteint enfin son inner correlator, peut-être augmenter window à 128

**#5 Nettoyer les workarounds latents**
- Le workaround historique sur F4E4 (forçait un comportement non-FRET) a été
  retiré cette session. Peut-être d'autres workarounds similaires dans le
  code (chercher comments "Until X is fixed", "TEMP", "HACK", "FIXME").
- Réduire la verbosité des tracers une fois la racine INTM levée
  (qemu.log atteint 12G ; voir Issues annexes).

## Run config courante

`run_new.sh` passe `CALYPSO_SIM_CFG="$MOBILE_CFG"` à QEMU pour le module SIM
(session 04-26 matin) et désactive l'ancien injection hack.

## Session précédente (matin/après-midi 2026-04-26)

Voir mémoires `project_session_20260426*.md`. Résumé des découvertes :
- Module SIM ISO 7816 émulé natif (calypso_sim.c)
- Hacks fw_patch supprimés (no INJECT, no fw_patch_apply)
- BSP daram fix initial 0x3fb0
- Bridge anchor fix
- Multiple opcode fixes (SFTC, F6Ex, etc.)

## Session de soirée (cette session, 2026-04-26)

L'audit CLAUDE.md a révélé une cascade de **7 bugs d'opcode majeurs** :
- F4E4 workaround → vrai FRET
- F6Ex tous décodés en MVDD
- 0x88/0x89 STLM décodé en MVDM 2-word
- case 0x1 = SUB au lieu de LD/LDU
- case 0x0 = ADD avec << 16 toujours
- Indirect MOD table 12-15 décalée d'1
- 0x60xx/0x61xx CMPM/BITF jamais implémentés

Cette série a permis au bootloader de terminer (BACC à 0x7000) et au DSP
d'entrer dans le user code. Le bug INTM=1 reste.

## Issues annexes

### tmpfs /tmp 16G

`qemu.log` peut atteindre 12G+ avec tous les tracers. Surveiller `df -h /tmp`.

### Link `-lm` cassé (workaround manuel)

```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh && bash /tmp/link.sh
```
