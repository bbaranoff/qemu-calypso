# DSP Revival — Diag pour Claude Web (2026-06-25) : câblage I/Q + mur boot

## Objectif (branche `dsp_revival`)
Faire camper le Calypso émulé via le **VRAI DSP c54x émulé** : la tâche FB
(`FB_DSP_TASK=5`) doit détecter la tonale FCCH (+Fs/4) → écrire `d_fb_det≠0`
dans la NDB → l'ARM lit le résultat → camping.
**Discipline FIDÉLITÉ** : faire tourner la ROM DSP **inchangée**, corriger
l'ÉMULATEUR/ENVIRONNEMENT (interruptions, I/Q, init), **JAMAIS** poker l'état
interne DSP (IMR / d_fb_det / table de vecteurs / flags) comme béquille.

---

## TL;DR — DEUX ruptures CONFIRMÉES (mesurées live, pas déduites)
- **Break A — le DSP est coincé dans le boot.** Le scheduler per-frame (**vec28**)
  qui lance le corrélateur ne tourne **JAMAIS** ; go-live jamais atteint ;
  `IMR=0` → rien ne vectorise.
- **Break B — le câblage I/Q est cassé.** Les vrais I/Q **post-AFC** (avec la
  tonale FB, prouvée par FFT) sont **dumpés sur disque mais jamais transmis** à
  la fenêtre corrélateur du DSP. `g_shunt.last_iq` est **vide**.

A priori indépendants : Break B est **nécessaire** mais probablement **pas
suffisant** tant que Break A persiste (corrélateur jamais exécuté). [verdict
de suffisance : workflow `wxbvt5qzg` en cours]

---

## Preuves mesurées

### 1) Snapshot gdb live (PID qemu, via `g_shunt.c54x`)
```
pc=0xa6b8 imr=0x0000 ifr=0x0028 sp=0x5ac8 insn=13654684
d_dsp_page[08d4]=0x0002 d_task_md[0804]=0x0005 d_fb_det[08f8]=0x0000
sched_ran[3fb0]=0x0000 wait[3f70]=0x0000 softvec[3f6d]=0xa4df
g_shunt.last_iq_valid=false  last_iq_n=0  last_iq[...] = tout zéro
```
Lecture (un fait par ligne, OVLY respecté) :
- `imr=0` → effacé par le boot (STM #0x0000 à 0xb37e), **jamais ré-armé** → masqué.
- `sp=0x5ac8` → **pile de BOOT** (l'opérationnelle = 0x1100) → go-live jamais atteint.
- `softvec[3f6d]=0xa4df` (wait-loop) ; `sched_ran[3fb0]=0`, `wait[3f70]=0` →
  scheduler vec28 **jamais lancé**.
- `d_task_md=5`, `d_dsp_page=2` → **l'ARM commande FB correctement** (ce n'est pas l'ARM).
- `d_fb_det=0` → le DSP n'a rien calculé.
- `last_iq vide` → l'I/Q n'est **jamais stashé** → `c54x_bsp_load` **jamais appelé**.

### 2) FFT tonale des I/Q livrés (`/opt/GSM/iq_fft_tone.py` sur `/tmp/iq_dlv_*.bin`)
```
Fs=270833 Hz  N=148  FB tone expected +Fs/4=+67708 Hz (bin +37)
iq_dlv_005 : peakHz=67708 (=+Fs/4)  +Fs/4mag=1180794  -Fs/4mag=557240  → TONALE FB PROPRE, dominante
iq_dlv_006/007 : bords/transition ;  21 autres : zéro
```
→ **La tonale FB EST présente, propre, dans les samples post-AFC livrés.** La
chaîne radio→I/Q fonctionne. Le problème est 100% en aval (DSP + câblage).

---

## Architecture du flux I/Q (vérifiée, code à l'appui)
```
osmo-trx ─► qemu_wrap (decim 4→1) ─► calypso_bsp.c::deliver_buffered (~1210)
   sl->iq  (148 cplx int16, POST-AFC, tonale FB)  ──► dump iq_dlv_%03u.bin  (preuve FFT)
        │
        ╳  ◄═══ CÂBLAGE MANQUANT (Break B) : sl->iq n'est jamais stashé ═══
        │
   g_shunt.last_iq   ◄── rempli SEULEMENT par feed_iq() (calypso_dsp_shunt.c:1559)
        │                  garde: shunt_route_c54x() && g_shunt.c54x  (VRAI en c54x)
        │                  unique caller = calypso_bsp.c:403  ← chemin passthrough RÉSEAU (buf socket), INACTIF
   shunt_route_to_c54x()  ─► c54x_bsp_load(last_iq)  ─► s->bsp_buf   (calypso_c54x.c:13676)
        │
   DSP exécute PORTR PA=0x0034 ─► écrit data[0x2a00..0x2b27]  (fenêtre corrélateur)
        │
   scheduler vec28 (Break A : jamais lancé) ─► corrélateur (lit 0x2a00) ─► écrit d_fb_det
```
Mesuré : `data[0x2a00]` contient du **non-zéro bruité** (PORTR lit un `bsp_buf`
stale/vide, pas la tonale FB).

---

## Break A — mur boot (détail)
**Chaîne scheduler (vrai chemin FB)** : IT trame → **vec28 (bit12)** → slot
`data[0xf0]=731e 3fcf f880 7234` → 0x7234 → `CALL 0xa4e4` → lit d_dsp_page à
**0xa51c** (`LD *(0x08d4)`) → `BACC data[0x3fe0]=0x70ce` → corrélateur
(0x8fb8/0x924a/0x93a2, lit burst 0x2a00) → écrit **d_fb_det** à 0x91e3/0x9204.

**Le deadlock** : le boot efface IMR (0xb37e, insn 1047), route vers wait
(`BC 0xb3ff if data[0x3f70]==0` à 0xb3e6) → soft-vector `data[0x3f6d]=0xa4df`.
La wait-loop (0xa4d4) teste `data[0x3f70]` **bit1 (0x0002)** ; la branche qui
**poserait bit1** (0xde9c / 0xa5bd / 0xb3ef) **ne tourne jamais**. Donc go-live
(**0xa4c7** = `ORM #0x3000,IMR` arme bits 12/13 + contexte ISR à 0x011e ;
ré-arm IMR final à **0xa582** `STL A,IMR`) **jamais atteint**. IMR initial 0x52fd
(de `Registers.bin`, bit12/vec28 armé) → effacé au boot → 0.

⚠️ **Piège OVLY** (source d'erreurs répétées) : `PMST OVLY=1` → `pc∈[0x80,0x2800)`
s'exécute depuis **data[pc]**, `pc≥0x2800` depuis **prog[pc]**. Lire la table de
vecteurs en `data[0x80..0xff]`, pas `prog[]`. Ex : `data[0x013b]` = épilogue ISR.

---

## Break B — câblage I/Q (détail + fix proposé)
- `c54x_bsp_load(s, samples, n)` (calypso_c54x.c:13676) copie `n` uint16 dans
  `s->bsp_buf`. Le DSP lit `bsp_buf` via **PORTR PA=0x0034** → écrit data[0x2a00].
- `bsp_buf` rempli **uniquement** par `shunt_route_to_c54x` → `c54x_bsp_load(g_shunt.last_iq)`.
- `last_iq` rempli **uniquement** par `feed_iq` (calypso_dsp_shunt.c:1581-1584,
  garde `shunt_route_c54x()&&g_shunt.c54x` = VRAI en c54x).
- **Unique caller** de `feed_iq` = `calypso_bsp.c:403` = chemin **passthrough
  réseau** (`buf` depuis socket UDP/IPC) — **inactif** ce run (mesuré : last_iq vide).
- ⟹ Les bons I/Q (`sl->iq`, deliver_buffered, source iq_dlv) **ne sont jamais câblés**.

**FIX proposé** (en cours de design/vérif par workflow `wxbvt5qzg`) : au point
`deliver_buffered` (calypso_bsp.c ~1210, là où iq_dlv est dumpé), **stasher
`sl->iq` dans `g_shunt.last_iq`** (fonction légère exportée du shunt), pour que
`shunt_route_to_c54x → c54x_bsp_load` le serve au corrélateur au prochain tick.
[Détails patch — type exact de sl->iq, longueur 296 int16, latence 1 trame,
verdict suffisance — à compléter avec le workflow.]

---

## Dead-ends (NE PAS refaire)
- **Force-vectoriser vec28 depuis l'idle** : 0x7234 → `CALL 0x013b` = **épilogue
  ISR** (POPM ST0/BK/BRC/AR7..0) ; dépile un contexte jamais empilé → corrompt
  les regs → derail. Le vecteur est juste, la **livraison par force** est fausse.
- **Force go-live au boot** (`CALYPSO_DSP_GOLIVE_BOOT`) : go-live attend un état
  post-bootstrap cohérent (SP=0x1100) ; forcé depuis le boot (SP=0x5ac8) → SP corrompu.
- **Autre ROM (3606, IMR=0x3000)** : deadlock **IDENTIQUE** → la ROM n'est PAS la racine.
- **bin verbatim** (IFR=0x0008 du dump, plus de forçage IFR/BRC/RSA/REA) : n'a
  **PAS** débloqué (toujours coincé boot, sp=0x5ac8).
- **Faits faux corrigés** : `0x702b`=`ANDM #0xfffc,*(0x1d)` (PAS un write IMR) ;
  commentaires périmés `APTS`/`0x52fd`.

---

## Reproduire le diag
- **Run** : `calypso.env` (CALYPSO_DSP=c54x, CALYPSO_DSP_REG_MODE=bin,
  CALYPSO_IQDUMP=1) + `start-clean.sh` dans `osmo-operator-1`.
- **gdb snapshot** : `/opt/GSM/snap.gdb` (`set $d=g_shunt.c54x` ; lit
  imr/sp/data[...]). `pkill -9 gdb` avant ; écrire la sortie dans **/opt/GSM**
  (PAS /tmp = mount shadow du conteneur) ; `set logging redirect on` pour
  séparer la sortie gdb du flood des sondes qemu.
- **FFT tonale** : `python3 /opt/GSM/iq_fft_tone.py` (lit `/tmp/iq_dlv_*.bin`,
  int16 brut entrelacé I,Q, 592o=148 cplx, sans header).

---

## Questions ouvertes / prochaines étapes
1. **[Break B]** Implémenter le câblage `sl->iq → last_iq` au point deliver_buffered.
2. **[Break A vs B — la question clé]** Le câblage I/Q débloque-t-il **AUSSI** le
   boot (le DSP attend-il une livraison BSP — PORTR / handshake API « buffer
   prêt » / IT BRINT0 — pour poser `data[0x3f70]` bit1 et avancer vers go-live),
   ou est-ce **indépendant** (boot piloté seulement par l'IT trame TPU) ?
3. **[Break A]** Si indépendant : trouver le **trigger naturel** qui fait passer
   le DSP du boot wait-loop à go-live (0xa4c7). Sans poke.
