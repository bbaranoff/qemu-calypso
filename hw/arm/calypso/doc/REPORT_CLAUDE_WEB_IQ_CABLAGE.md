> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> **Correction principale — Break B est INFONDÉ.** La prémisse « câblage I/Q manquant / `sl->iq` jamais transmis / `last_iq` vide » est **FAUSSE**. `calypso_bsp_deliver_buffered()` (calypso_bsp.c:1107) — le chemin VIVANT, celui-là même qui dumpe `iq_dlv` (calypso_bsp.c:1267, samples FB prouvés) — charge **directement** `sl->iq` dans `bsp_buf` via `c54x_bsp_load(bsp.dsp, samples, …)` à **calypso_bsp.c:1248**, indépendamment de `last_iq`/`feed_iq`/`shunt_route_to_c54x`. Le « FIX proposé » (stasher `sl->iq` dans `last_iq`) est donc **redondant/inutile** : il n'ajouterait qu'un détour avec 1 trame de latence. Le vrai blocage est en aval : **handshake go-live ARM→DSP jamais armé** (api_write_cb déclaré dans calypso_c54x.h mais **jamais assigné** — `grep 'api_write_cb *=' = 0`), IMR reste 0x0000, le corrélateur ne tourne jamais.

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
- ~~**Break B — le câblage I/Q est cassé.** Les vrais I/Q post-AFC sont dumpés
  sur disque mais jamais transmis à la fenêtre corrélateur du DSP. `g_shunt.last_iq`
  est vide.~~ — **FAUX (audit 2026-07-01)** : `deliver_buffered` charge `sl->iq`
  dans `bsp_buf` via `c54x_bsp_load` à **calypso_bsp.c:1248**, la MÊME trame où il
  dumpe `iq_dlv`. `last_iq`/`feed_iq` sont un chemin réseau ALTERNATIF inactif,
  pas le seul câblage. Il n'y a PAS de « câblage manquant ».

A priori indépendants : le seul blocage réel est **Break A** (go-live jamais
atteint, IMR=0, handshake ARM→DSP `api_write_cb` jamais câblé → corrélateur jamais
exécuté). Le débloquer suffit ; l'I/Q est déjà livré à `bsp_buf`.

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
- ~~`last_iq vide` → l'I/Q n'est jamais stashé → `c54x_bsp_load` jamais appelé.~~
  — **FAUX** : `last_iq` alimente le chemin réseau (inactif), pas le seul. Le
  chemin VIVANT `deliver_buffered` appelle `c54x_bsp_load` à **calypso_bsp.c:1248**
  (`sl->iq` → `bsp_buf`) chaque trame livrée. `bsp_buf` EST rempli.

### 2) FFT tonale des I/Q livrés (`/opt/GSM/iq_fft_tone.py` sur `/tmp/iq_dlv_*.bin`)
```
Fs=270833 Hz  N=148  FB tone expected +Fs/4=+67708 Hz (bin +37)
iq_dlv_005 : peakHz=67708 (=+Fs/4)  +Fs/4mag=1180794  -Fs/4mag=557240  → TONALE FB PROPRE, dominante
iq_dlv_006/007 : bords/transition ;  21 autres : zéro
```
→ **La tonale FB EST présente, propre, dans les samples post-AFC livrés.** La
chaîne radio→I/Q fonctionne. Le problème est 100% en aval (DSP + câblage).

---

## Architecture du flux I/Q (CORRIGÉE audit 2026-07-01)
```
osmo-trx ─► qemu_wrap (decim 4→1) ─► calypso_bsp.c::deliver_buffered (fn @1107)
   sl->iq  (148 cplx int16, POST-AFC, tonale FB)  ──► dump iq_dlv_%03u.bin (calypso_bsp.c:1267, preuve FFT)
        │
        └─► c54x_bsp_load(bsp.dsp, samples, n)  ─► s->bsp_buf   (APPEL RÉEL, calypso_bsp.c:1248)
                                                     ↑ chemin VIVANT, chaque trame livrée
   [ chemin ALTERNATIF réseau (INACTIF ce run) : feed_iq() → g_shunt.last_iq →
     shunt_route_to_c54x() → c54x_bsp_load(last_iq) ; sole caller feed_iq = calypso_bsp.c:432 ]
        │
   DSP exécute PORTR PA=0x0034 ─► écrit data[0x2a00..0x2b27]  (fenêtre corrélateur)
        │
   go-live/IMR JAMAIS armé (Break A : api_write_cb jamais câblé) ─► corrélateur jamais exécuté ─► d_fb_det reste 0
```
`bsp_buf` EST rempli par le chemin `deliver_buffered` (calypso_bsp.c:1248). Le
corrélateur ne le consomme jamais car go-live n'est jamais atteint (IMR=0).

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

## Break B — câblage I/Q ~~(détail + fix proposé)~~ — ✅ RÉFUTÉ (audit 2026-07-01)
**Ce Break n'existe pas.** Deux chemins mènent à `bsp_buf`, pas un :
- `c54x_bsp_load(s, samples, n)` (calypso_c54x.c:13676) copie `n` uint16 dans
  `s->bsp_buf`. Le DSP lit `bsp_buf` via **PORTR PA=0x0034** → écrit data[0x2a00].
- **Chemin VIVANT (actif ce run)** : `calypso_bsp_deliver_buffered()`
  (calypso_bsp.c:1107) construit `samples[296]` depuis `sl->iq` et appelle
  `c54x_bsp_load(bsp.dsp, samples, n>296?296:n)` à **calypso_bsp.c:1248**, la même
  trame où il dumpe `iq_dlv` (calypso_bsp.c:1267). Donc `bsp_buf` EST rempli avec
  les samples FB prouvés, sans passer par `last_iq`.
- **Chemin ALTERNATIF réseau (inactif ce run)** : `feed_iq` (calypso_dsp_shunt.c:1559,
  stash à 1581-1584) → `g_shunt.last_iq` → `shunt_route_to_c54x` (calypso_dsp_shunt.c:1015,
  `c54x_bsp_load(last_iq)` à 1039). **Unique caller** de `feed_iq` = **calypso_bsp.c:432**
  (chemin passthrough réseau `buf` depuis socket UDP) — inactif ce run, d'où `last_iq` vide.
- ⟹ `last_iq` vide n'implique PAS `bsp_buf` vide : le chemin `deliver_buffered` alimente
  `bsp_buf` en parallèle.

~~**FIX proposé** : au point `deliver_buffered`, stasher `sl->iq` dans
`g_shunt.last_iq` pour que `shunt_route_to_c54x → c54x_bsp_load` le serve au
corrélateur.~~ — **MOOT/REDONDANT** : `deliver_buffered` livre déjà `sl->iq` à
`bsp_buf` la même trame (calypso_bsp.c:1248) ; router via `last_iq →
shunt_route_to_c54x` n'ajouterait qu'un détour avec 1 trame de latence. Le vrai
blocage est **Break A** (go-live/IMR jamais armé, `api_write_cb` jamais câblé).

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
1. ~~**[Break B]** Implémenter le câblage `sl->iq → last_iq`.~~ — **SANS OBJET**
   (audit 2026-07-01) : `deliver_buffered` livre déjà `sl->iq` à `bsp_buf`
   (calypso_bsp.c:1248). Rien à câbler côté I/Q.
2. **[Break A — la seule question]** Trouver le **trigger naturel** du handshake
   go-live ARM→DSP. Vérité-terrain : `api_write_cb` est **déclaré** (calypso_c54x.h)
   mais **jamais assigné** (`grep 'api_write_cb *=' = 0`) ; l'ARM n'écrit que
   `0x0000` dans l'API 0x0314/0x0318 → go-live jamais asserté, IMR reste 0x0000
   tout le run (jamais ré-armé après l'effacement boot @0xb37e).
3. **[Break A]** Faire passer le DSP du boot wait-loop à go-live (0xa4c7) **sans
   poke** : câbler `api_write_cb` pour que l'écriture ARM « buffer prêt » pose
   `data[0x3f70]` bit1. (Attention : la vectorisation forcée VEC28 est un
   DEAD-END — elle atterrit dans l'épilogue ISR et déraille vers le boot-stub.)
