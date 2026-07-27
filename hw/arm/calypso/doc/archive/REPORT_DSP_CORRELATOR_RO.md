# Audit READ-ONLY — Le mur `d_fb_det = 0` du corrélateur DSP Calypso (c54x émulé)

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> **Deux raisons cumulées de péremption :**
> 1. **Cause racine réfutée (vérité-terrain).** La thèse centrale de ce rapport — « `d_fb_det=0` est *décode-attribuable* au mis-décode DADST/DSADT » — ne tient plus. La vraie cause est que le **DSP ne tourne jamais le corrélateur** : il déraille (POST-BOOTSTUB-RET, PC=0x0000), `IMR=0x0000` sur toute la run (jamais ré-armé après le clear boot @0xb37e), et le handshake ARM→DSP go-live n'est jamais asserté (l'ARM n'écrit que `0x0000` dans API `0x0314/0x0318` ; `api_write_cb` est déclaré mais jamais câblé). Le noyau `0x9a80..0x9a9f` et le point de publication `0x9ac0` **ne sont pas atteints en régime permanent** contrairement à ce que ce doc affirme (les « 84/84 DETECTOR-RUN past-boot » relèvent d'une session antérieure au dérail, non reproduits).
> 2. **Décode déjà corrigé (Family A).** ✅ FIXÉ (`calypso_c54x.c:8232-8305`, « fix revival dsp 2026-06-22 »). La famille **DADST/DSADT/DADD/DSUB/DLD/DRSUB/DSUBT (0x50-0x5F)** possède désormais un handler dédié `Lmem,dst` (via `resolve_lmem`, branches C16/SXM, DADST=+T / DSADT=−T). Les MAC dual-opérande `0x90-93` (`c54x.c:8342`) et `0xA4-A7/B0-B7` (`c54x.c:8847`) sont passés au décode 2-bit correct (même fix 2026-06-22). Toutes les affirmations « AUCUN handler » / « tombe en SFTL » / « RAW-decode faux » ci-dessous sont donc **obsolètes** et R1='SUPPORTED' est **caduc** (voir annotations inline).

**Scope** : audit en lecture seule (aucune édition, aucun rebuild, aucun restart). Container `osmo-operator-1`.
**Cible** : pourquoi `d_fb_det[0x08f8]` reste `0x0000` (la sync FB/FCCH ne verrouille jamais → pas de SB → pas de SI).
**Convention** : chaque affirmation est marquée **CONFIRMÉ** (vérifié source/dump/log dans cette session) ou **HYPOTHÈSE** (inférence forte non close).
Sources : `/opt/GSM/qemu-src/hw/arm/calypso/calypso_c54x.c`, dump mot-adressé `/opt/GSM/qemu-src/calypso_dsp.txt`, log live `/root/qemu.log`.

---

## 1. TL;DR — où est réellement le mur

- ~~**CONFIRMÉ — Cause racine décode-attribuable (STAGE-1)** : ... **Cette famille n'a AUCUN handler dans l'émulateur** : tout `0x5xxx` tombe dans `case 0x5`, un bloc purement SFTA/SFTL. La somme-de-produits de corrélation FCCH n'est donc jamais calculée.~~ — **FAUX (2×)** : (a) ✅ FIXÉ — un handler dédié DADST/DSADT/DADD/DSUB `Lmem,dst` existe désormais dans `case 0x5` à **`calypso_c54x.c:8232-8305`** (fix revival dsp 2026-06-22) ; seuls les vrais 0x50-51/54-55/58-5D-shift tombent encore dans le sous-décode SFTA/SFTL à `c54x.c:8306-8331`. (b) De toute façon la cause de `d_fb_det=0` n'est **pas** décode-attribuable : le DSP déraille (PC=0x0000), IMR=0x0000 jamais ré-armé, le corrélateur n'est jamais atteint (vérité-terrain 2026-07-01). Le noyau `0x9a80..0x9a9f` porte bien la famille `0x5x` (dump ci-dessous), mais elle est désormais correctement décodée — et non exécutée en régime permanent.

- **CONFIRMÉ (session antérieure, NON reproduit past-dérail)** — au point de publication `STL A,*AR2-` à `0x9ac0` (op `0x808a`, page0), `CORR-PEAK` montre `A=0x00123b69e0 B=0x0002477304 T=0x0001` **GELÉS à l'identique sur les 8 échantillons** ; et `DETECTOR-RUN @0x9ac0` lit `d_fb_det[08f8]=0x0000` dans **84/84** passes (insn 5746 → 497.76M). Un accumulateur de publication gelé = exactement l'effet d'avoir remplacé un dual add/subtract par un simple décalage.

- **CONFIRMÉ — Ce que ce N'EST PAS** : ce n'est pas l'opcode chaud `0xc8be` à `0xa0e7` (décode correct), ni les handlers raw-decode dual-MAC `0x90-93/0xA4-B7` (réels mais **froids** — jamais exécutés ici), ni un coefficient `T=0` (T n'est jamais nul), ni un bug de livraison BSP (buffer prouvé bon). **Co-cause probable (HYPOTHÈSE)** : un setup de pointeurs AR hors-buffer au régime permanent. Voir §5/§6.

---

## 2. Correction du lead antérieur — `0xa0da` n'est PAS un dual-MAC mal routé

**CONFIRMÉ.** L'hypothèse pré-session « `0xa0da` = dual-operand MAC mal routé » est **morte**.

- `prog[0xa0e9] = 0xa0da` (dump row `0a0e0`, page1, `calypso_dsp.txt:2599` : `... 3c84 c8be 6e89 a0da 1192 ...`).
- `hi8 = 0xA0` → le dispatch atteint `if (hi8 == 0xA0)` à **`calypso_c54x.c:9015`** (handler accumulateur : LD/NEG/ABS/NOT/SFTA/SFTL/SAT). Sous-code `sub = op & 0xFF = 0xDA`. La cascade : pas un code fixe (00/01/04/05/08/09/0A/0B/98/99), puis **`else if (sub & 0x80)` est VRAI** (0xDA a bit7) → branche **SFTL** (`calypso_c54x.c:9047`) : `shift = (sub>>1)&0x3F = 0x6D` sign-étendu = **-19**, `dst = sub&1 = 0` → **SFTL A, -19**. Aucun opérande mémoire, aucun routage vers un handler MAC. (Décode correct, toujours valide ; lignes ré-alignées 2026-07-01 : 8493→9015, 8525→9047.)

**Pourquoi l'ancienne hypothèse a mal localisé** : elle a lu `0xa0da` comme une instruction autonome alors qu'**elle n'est jamais exécutée comme telle**. Le mot à `0xa0e8 = 0x6e89` est un **BANZD (2 mots)** ; son 2e mot (`prog[0xa0e9] = 0xa0da`) est le **pmad (cible de branche)** du BANZD, pas un opcode. Preuve live : `0xa0e7` apparaît 526x dans le log ; `0xa0e8/0xa0e9/0xa0da` apparaissent **0x**.

**Le vrai opcode dominant du corrélateur (STAGE-2)** : `prog[0xa0e7] = 0xc8be` (la valeur exacte que la sonde CORR-PEAK rapporte). `hi8 = 0xC8` → handler **ST||LD parallèle** à `calypso_c54x.c:9404` (`if (hi8 >= 0xC8 && hi8 <= 0xCB)`). Décodé : `ST A,*AR4+0% || LD *AR5-,B`. **Ce décode est CORRECT** (AR2..AR5, mod 2-bit, *AR+0% circulaire via BK) — ligne ré-alignée 2026-07-01 : 8882→9404. Donc l'opcode chaud n'est **pas** la divergence de décode ; il ne fait qu'un store+reload (pas de multiply, pas d'accumulation). C'est un **symptôme aval**, pas la cause.

---

## 3. Le hot path du détecteur FB (`@0x9ac0`) — disasm fidèle + mapping handler

**Page live = page0 / PROM0. CONFIRMÉ.** Banking par `c54x_prog_xlate` (`calypso_c54x.c:3663`, `0x8000-0xDFFF` indexé par `(xpc<<16)|addr`). À `exec_pc=0x9ac0` l'opcode live est `cur_op=0x808a` dans **toutes** les occurrences (`qemu.log` ex. `COEFFS-WR #2397 ... exec_pc=0x9abf cur_pc=0x9ac0 cur_op=0x808a`). `0x808a` ne matche **que** la row PROM0 du dump (`calypso_dsp.txt:2501` : `09ac0 : 808a 6e89 9ab8 ...`) ; les 3 candidats bankés (PROM1 `0xfc00`, PROM2 `0x10f8`, PROM3 `0xfcdf`) sont exclus.

### 3a. Noyau d'accumulation `prog[0x9a80..0x9a9f]` (dump `calypso_dsp.txt:2497-2498`)

```
09a80 : 8382 5a85 5f95 8e94 8f93 5e85 5b95 8e94 8f93 5a85 5f95 8e94 8f93 5e85 5b95 8e94
09a90 : 8f93 3082 5a85 5f95 8e94 8f93 5e85 5b95 8e94 8f93 5a85 5f95 8e94 8f93 5e85 5bd5
```

→ la famille `0x5x` (DADST/DSADT) apparaît **16x**, entrelacée avec CMPS `8e94/8f93` (**14x**). La 0x5x est l'op de calcul la plus fréquente ET la VRAIE accumulation ; CMPS ne fait que le peak-select sur le résultat.

> ⚠️ **Colonne « Divergence » PÉRIMÉE.** ✅ FIXÉ (`c54x.c:8232-8305`, 2026-06-22) : la famille 0x5x (DADST/DSADT/…) est maintenant décodée `Lmem,dst` correctement (voir §1). Les mentions `c54x.c:7818` ci-dessous étaient déjà fausses (7817 = handler BANZD `0x6E00`) : le dispatch 0x5x est `case 0x5:` à **`c54x.c:8225`**, le handler dual-long-word à **8232-8305**, et le sous-décode SFTA/SFTL résiduel à **8306-8331**. Table conservée comme historique ; la colonne « Divergence » ne s'applique plus.

| PC | op | Silicon (réel C54x) | Handler émulateur (2026-07-01) | Divergence (PÉRIMÉE) |
|----|----|--------------------|--------------------|------------|
| 0x9a80 | 8382 | ST T,*AR2 (Smem) | `c54x.c:8796` (hi8 0x83) | ok (Smem) |
| **0x9a81** | **5a85** | **DADST *AR5,A** | **`c54x.c:8232` handler dual-long-word dédié** | ~~wrong-op~~ **FIXÉ** |
| **0x9a82** | **5f95** | **DSADT *AR5+,B** | **`c54x.c:8232`** | ~~wrong-op~~ **FIXÉ** |
| 0x9a83 | 8e94 | CMPS A,*AR4+ | `c54x.c:8580` (hi8 0x8E) | ok |
| 0x9a84 | 8f93 | CMPS B,*AR3+ | `c54x.c:8580` (hi8 0x8F) | ok |
| **0x9a85** | **5e85** | **DSADT *AR5,A** | **`c54x.c:8232`** | ~~wrong-op~~ **FIXÉ** |
| **0x9a86** | **5b95** | **DADST *AR5+,B** | **`c54x.c:8232`** | ~~wrong-op~~ **FIXÉ** |
| 0x9a91 | 3082 | LD *AR2,T (ld Smem,T) | `c54x.c:5303` (0x3000) | ok |
| **0x9a9f** | **5bd5** | **DADST *AR5+0%,B** (circulaire BK) | **`c54x.c:8232`** (Lmem via `resolve_lmem`, post-mod ±2) | ~~wrong-op + missing-circular~~ **FIXÉ** |

(Les 16 occurrences 0x5x — 5a x4, 5f x4, 5e x4, 5b x3, 5bd5 x1 — passent toutes désormais par le handler dédié 8232-8305.)

~~**CONFIRMÉ (source relue, `c54x.c:7818-7852`)** : `case 0x5` ne contient que SFTA/SFTL. `grep dadst|dsadt` -> 0 handler.~~ — **FAUX** : ✅ FIXÉ. `grep -in "dadst|dsadt" calypso_c54x.c` retourne maintenant le handler `8232-8300` (+ sonde SHADOW-DADST 12189/12342). Le `case 0x5:` (`c54x.c:8225`) branche d'abord sur le handler dual-long-word (`dl_hi` 0x50-0x5F via `resolve_lmem`, branches C16/SXM, `DADST=Lmem+((T<<16)|T)` / `DSADT=Lmem-…`) ; le sous-décode SFTA/SFTL (`c54x.c:8306-8331`) ne reçoit plus que les vrais shifts. **La somme-de-produits FCCH est donc correctement décodée — le mur `d_fb_det=0` ne vient PAS de là (vérité-terrain : DSP déraillé, IMR=0).**

### 3b. Boucle de publication `0x9ab8..0x9ac2`

| PC | op | Mnémonique | Handler | Divergence |
|----|----|-----------|---------|------------|
| **0x9ac0** | **808a** | **STL A,*AR2-** (publie le résultat -> écrit `d_fb_det`) | `c54x.c:8533` (hi8 0x80) resolve_smem `*AR2-` | **none** (décode correct) |
| 0x9ac1 | 6e89 | BANZD 0x9ab8,*AR1- (2 mots) | `c54x.c:7817` (`0x6E00`) | none |
| 0x9ace | 6e8c | BANZD 0x9ac7,*AR4- | `c54x.c:7817` | none |

L'ironie clé : **le store de publication `0x9ac0` est décodé correctement** — il publie fidèlement un accumulateur qui est faux **en amont**. C'est précisément le site que lit `DETECTOR-RUN` => `d_fb_det=0`.

---

## 4. Défauts confirmés vs bénins — inconsistance `resolve_xmem` vs raw-decode

Deux conventions de décode dual-operand (Xmem,Ymem) **coexistent** dans `c54x.c` :

- **CORRECT (silicon-fidèle)** : `resolve_xmem` (`c54x.c:3992`) : `xar=(xmem&3)+2` => **AR2..AR5 seulement**, `xmod=(xmem&0xC)>>2`, `case 3 = c54x_circ_ref(+AR0,BK)` => `*AR+0%` circulaire. (Ligne ré-alignée 2026-07-01 : 3804→3992.)
- ~~**RAW (faux vs silicon)** : `xar=(op>>4)&0x07` => **AR0..AR7**…~~ — **OBSOLÈTE (Family A)** : ✅ FIXÉ 2026-06-22. Les handlers dual-MAC `0x90-93` (`c54x.c:8342`) et `0xA4-A7/B0-B7` (`c54x.c:8847`) sont passés au **décode 2-bit correct** (`Xmod[7:6] Xar[5:4] Ymod[3:2] Yar[1:0]`, AR=field+2, mod 3=*AR+0% circ via BK), identique à `resolve_xmem`. Il n'y a plus de convention « RAW 3-bit » sur ces classes.

| hi8 | Instr | Site (2026-07-01) | Décode | Exécuté par le corrélateur ? | Sévérité |
|-----|-------|------|--------|------------------------------|----------|
| 0x90-93 | MAC/MACR Xmem,Ymem | `c54x.c:8342` | ~~RAW~~ **CORRECT 2-bit (✅ FIXÉ 2026-06-22)** | NON (froid) | ~~high~~ **résolu** |
| 0xA1 | SQDST | `c54x.c:8909` | 2-bit | NON | low |
| 0xA4-A7 / 0xB0-B7 | MAC/MACR | `c54x.c:8847` | ~~RAW~~ **CORRECT 2-bit (✅ FIXÉ 2026-06-22)** | NON (froid) | ~~high~~ **résolu** |
| 0xB8-BB | MAS/MASR | `c54x.c:8959` | 2-bit | NON | low |
| 0xBC-BF | POLY/ABDST | `c54x.c:8936` | 2-bit | NON | low |
| 0xF6 (sub>=8) | « MVDD » | `c54x.c:6531` (bloc F6xx) | RAW (et 0xF6 != MVDD en silicon !) | NON (0x ; 0xf6b8 intercepté par RSBX 0xF6BB) | REAL-but-COLD, faux-en-plus |
| 0xC8-CB | ST||LD (= **0xc8be HOT**) | `c54x.c:9404` | **CORRECT** (inline 2-bit + circ) | ~~OUI (526x)~~ **NON en régime permanent (DSP déraillé)** | **none — décode correct** |
| 0xD0-D9/DB/DC | MAC/MAS/MASA/SQDST | `c54x.c:9164` | CORRECT (inline 2-bit + circ) | non | none |
| 0xE5 | MVDD | `c54x.c:7174` | CORRECT | non | none |
| 0x98-9B | STL/STH | `c54x.c:8403` | CORRECT (`resolve_xmem`, fix 2026-05-23) | non | low |
| 0x9C-9F | SACCD (`0x9e9b` @0x8578) | `c54x.c:8431` | CORRECT (`resolve_xmem`, fix 2026-06-02) | boot seulement | low |
| 0x70-75 | MVDM/MVMD/MVDK | `c54x.c:7650` catch-all `(op&0xF800)==0x7000` | **STL 1-mot** (PAS de handler dédié) | 0x71 MVDK `0x7193` (transitoire) ; 0x72/0x73 non | **latent** |

**Verdict §4 (mis à jour 2026-07-01) :** ✅ La dual-MAC `0x90-93` / `0xA4-A7/B0-B7` **n'est plus** une divergence raw — décode 2-bit correct depuis 2026-06-22 (`c54x.c:8342` / `8847`). Reste FROIDE de toute façon. Et la famille 0x5x (§3) est elle aussi FIXÉE. **Aucune de ces classes n'est la cause de `d_fb_det=0`** : vérité-terrain, le mur est en amont du décodeur — le DSP déraille (PC=0x0000), IMR=0x0000 jamais ré-armé, `api_write_cb` jamais câblé, donc le corrélateur n'est jamais exécuté.

---

## 5. L'évidence décisive — log live réconcilié avec le disasm

> ⚠️ **Section issue d'une session antérieure au dérail DSP — conclusions caduques.** Les observations « DETECTOR-RUN @0x9ac0 fire 84/84 past-boot » et « posting ARM↔DSP sain » **ne sont plus reproduites**. Vérité-terrain 2026-07-01 : le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), IMR=0x0000 sur toute la run, et le handshake go-live ARM→DSP n'est jamais asserté (l'ARM n'écrit que 0x0000 dans API 0x0314/0x0318 ; `api_write_cb` déclaré mais jamais câblé). Le corrélateur `0x9ac0` **n'est donc pas atteint** en régime permanent — les métriques ci-dessous sont conservées comme historique.

**CONFIRMÉ (session antérieure) — le mur est réel et stable** : `DETECTOR-RUN @0x9ac0` -> `d_fb_det[08f8]=0x0000` dans **84/84** lignes (vérifié : `grep -c DETECTOR-RUN`=84 ; `grep d_fb_det\[08f8\]=0x0000`=84). Premier `#0 insn=5746`, dernier `#10800 insn=497760544`.

**CONFIRMÉ — accumulateur gelé au point de publication** : `CORR-PEAK @0x9ac0` -> `A=00123b69e0 B=0002477304 T=0001` **identique sur les 8 échantillons** (vérifié : `8 A=00123b69e0 B=0002477304 T=0001`). Un accumulateur figé = la prédiction exacte du remplacement dual-add->shift.

**CONFIRMÉ — `T` n'est JAMAIS nul** : `grep T=0000` sur CORR-PEAK + IQ-READ = **0** hits. T = `0x0001` à `0x9ac0`, `0xfb00` à `0xa0e7`, `0xffd8` à `0xee38`. => l'hypothèse « coefficient T=0 » est **morte** (§6).

**CONFIRMÉ — buffer prouvé bon** : `SPAN-READ[0x2a00..0x2a1f] == SPAN-WRITE bsp_buf[0..31]`, 296 mots byte-for-byte (insn=265M). Pas un bug de livraison.

~~**CONFIRMÉ — posting ARM<->DSP sain** … Le mur est purement la non-convergence du corrélateur DSP.~~ — **FAUX (vérité-terrain 2026-07-01)** : le handshake go-live ARM→DSP **n'est PAS sain** — l'ARM n'écrit que `0x0000` dans API `0x0314/0x0318`, `api_write_cb` (déclaré dans `calypso_c54x.h`) n'est **jamais assigné** (`grep 'api_write_cb *=' = 0`). Le mur n'est **pas** une non-convergence du corrélateur : le corrélateur n'est jamais lancé (DSP déraillé, IMR=0). C'est le **vrai gap** — pas un problème de décode.

### Résolution de la contradiction IQ-READ vs CORR-PEAK

Ce sont **deux sondes différentes, à deux PC différents, dans deux routines différentes** — pas un conflit :

- **`IQ-READ`** fire dans `data_read()`, **gaté sur l'ADRESSE LUE dans [0x2a00..0x2b28]** (`c54x.c:1619`). La grosse majorité de ses hits sont à **`PC=0xee38`** (routine PROM1 fixed-ROM, op `0xb01e`, dump row `0ee30` `calypso_dsp.txt:1530`). Là, `AR3/AR4/AR5 = 2b97/2b98/2ce1` sont **GELÉS et HORS buffer** (pointees 0), tandis que l'addr lue (scatter 0x2ab7,0x2aff…) est pilotée par un AR différent ; A=0 / B=0x5088 / T=0xffd8 figés sur ~2000 s. Note : `prog[0xee40]=0x2b97` = littéralement la constante rechargée dans AR3 figé -> la boucle **recharge un pointeur fixe hors-buffer** au lieu d'avancer.
- **`CORR-PEAK`** fire sur `exec_pc==0xa0e7 || 0x9ac0` (détecteur FB `a_sync_demod`). À `0xa0e7`, le `*AR5-` de `0xc8be` fait **descendre AR5 0x2a00->0x29e1** (sous la base du buffer) ; ces lectures à `0x29xx` sont `<0x2a00` donc **ne déclenchent JAMAIS** la gate IQ-READ -> **0 IQ-READ à 0xa0e7/0x9ac0** (cohérent). `A` y est figé à `0xffe8f60000 = sext(0xe8f6)<<16` = un **LD nu** (jamais un produit T·val, qui vaudrait `0x733200`).

=> Pas de contradiction : la sonde IQ-READ regarde le **lecteur permanent `0xee38`** (PROM1, AR figés hors-buffer), la sonde CORR-PEAK regarde le **détecteur FB `0xa0e7/0x9ac0`** (AR5 marchant sous le buffer). La voie **autoritaire** pour `d_fb_det=0` est `0x9ac0` (où DETECTOR-RUN lit `0x08f8`), donc le STAGE-1 (§3) est la cause décode-attribuable.

---

## 6. Résultats adversariaux

| # | Claim | Verdict | Évidence clé |
|---|-------|---------|--------------|
| R1 | DADST/DSADT (0x5a/5b/5e/5f) mal routés en SFTL -> contribue à `d_fb_det=0` | ~~**SUPPORTED**~~ **CADUC / REFUTED (2026-07-01)** | ✅ FIXÉ : handler dédié `c54x.c:8232-8305` (`grep dadst|dsadt` retourne maintenant 8232-8300 + 12189) — plus de mis-route SFTL. Et vérité-terrain : `d_fb_det=0` n'est pas décode-attribuable (DSP déraillé, IMR=0, corrélateur jamais exécuté). L'ancienne « inférence forte » dadst→A-gelé est doublement obsolète. |
| R2 | `0xc8be` (0xa0e7) a un décode faux et cause d_fb_det=0 | **REFUTED** | Handler `c54x.c:9404` décode AR2..AR5 + 2-bit mod + `*AR+0%` circ correctement ; live AR5 décrémente (=`*AR5-`), AR4 stable (=`*AR4+0%`, BK=0->linéarisé) ; 0 MOD-MISMATCH / 0 STLD-SP @0xa0e7. C'est un store+reload, pas un accumulate -> A figé ne peut venir de là. |
| R3 | dual-MAC raw 0x90-93 décode faux -> cause d_fb_det=0 | **REFUTED** | ✅ FIXÉ : décode 2-bit correct `c54x.c:8342` (fix 2026-06-22). Plus de raw 3-bit. Et froid de toute façon. |
| R4 | 0xA4-A7/B0-B7 MAC décode faux -> cause | **REFUTED** | ✅ FIXÉ : décode 2-bit correct `c54x.c:8847` (fix 2026-06-22, `resolve_xmem`-like). Plus de divergence raw. Reste froid. |
| R5 | 0xF6 (sub>=8) « MVDD » raw -> cause | **REFUTED** | Pire que faux : `0xF6` != MVDD en silicon (MVDD=0xE5 seul). L'exemple `0xf6b8` est **intercepté par RSBX** (`c54x.c:6329`) et `return` **avant** la branche MVDD `c54x.c:6489` ; la branche fire **0x**. Non-contributeur. |
| R6 | **Landmine MVDM-revert** `prog[0xf564]=0x7215` désync dispatch -> AR5 jamais chargé -> tâche FB jamais dispatchée -> 0x9ac0 jamais ré-atteint past-boot -> d_fb_det=0 | **REFUTED (chaîne causale)** | Mis-décode RÉEL (`c54x.c:7650` catch-all `(op&0xF800)==0x7000` avale 0x72 MVDM 2-mots en STL 1-mot ; commentaire in-code `c54x.c:7473` « LE MIS-DÉCODE EST RÉEL »). MAIS chaîne réfutée live : DETECTOR-RUN @0x9ac0 fire **84x jusqu'à insn=497M** (past-boot), CORR-PEAK @0xa0e7 fire past-boot, AR5 prend des valeurs réelles variées, PC alignés. Les ops 0x72/0x73 ne firent **jamais** dans ce log. C'est un **bug compensateur** dont le fix naïf **régresse** (commentaire : AR3 hors-buffer, delivered=0, deadlock pire). Sibling **0x71 MVDK `0x7193`** fire 27x sous le même catch-all (FB-DET-WR transitoire 0x6b34) — vrai mais non-proximal. |
| R7 | A=0/constant car le **coefficient T=0** (pointeur coeff hors-table) | **REFUTED** | `grep T=0000`=0 sur 1064 lignes sonde. T=0x0001/0xfb00/0xffd8 selon PC, jamais nul. A@0xa0e7 = `sext(0xe8f6)<<16` (LD nu), pas un produit T·val. Un T correct non-nul ne dégèlerait pas A. |
| R8 (pré-session) | `0xa0da` = dual-MAC mal routé | **REFUTED** | `c54x.c:9015`, sub=0xDA, SFTL A,-19 ; et c'est le **pmad operand** du BANZD 2-mots à `0xa0e8` (jamais exécuté standalone : 0x dans le log). |

---

## 7. Prochaines vérifications READ-ONLY (avant tout fix) — priorisées

Aucun code proposé comme fait. Objectif : **clore l'inférence** R1 et arbitrer STAGE-1 vs co-cause AR avant toute édition.

1. **[Le plus décisif]** Dump A/B **avant/après chaque op 0x5x** du noyau `0x9a80-0x9a9f` (étendre/grep une sonde RO aux `exec_pc` 0x9a81/85/86/…/9a9f). But : confirmer directement que A n'est **que décalé** (pas dual-add/subtract) et fermer le gap inférentiel R1 — actuellement CORR-PEAK n'échantillonne qu'à 0x9ac0/0xa0e7.
2. **Capturer AR2/AR5/AR0/BK à l'ENTRÉE du noyau `0x9a8x`** et pendant la marche `*AR5` : grep `BLOB-WR` / trace AR à `PC=0x9a8x`. Si AR5 pointe **déjà hors `[0x2a00..0x2b27]`** quand les dadst tournent, le fix dadst seul ne suffira pas -> le défaut de **setup AR doit être corrigé en premier/aussi** (co-cause).
3. **Confirmer BK au moment où `0x5bd5` (`*AR5+0%`) s'exécute** : grep `BK-WR` autour du noyau. Si `BK=0` (comme la jambe 0xc8be), le wrap circulaire serait linéarisé même en silicon correct -> change si « missing-circular » importe vraiment ici.
4. **Vérifier au RUNTIME que `0x9a80` est sans ambiguïté page0/PROM0** (pas que dans le dump) : grep `cur_pc=0x9a8x cur_op` et comparer aux octets page0 (`8382 5a85 5f95 …`), comme fait pour 0x9ac0 (`cur_op=0x808a`). Écarte une surprise d'overlay banké pour le noyau lui-même.
5. **Valider le mapping du symbole `d_fb_det`** : confirmer `d_fb_det = 0x08f8` en espace data et que le `STL A,*AR2-` à `0x9ac0` (ptr AR2) **atterrit bien sur / alimente** `0x08f8` (tracer la cible AR2 à 0x9ac0 et l'addr `FB-DET-WR`), pour certifier que l'A figé à 0x9ac0 est bien la valeur que DETECTOR-RUN lit à 0.
6. **Scanner toute autre exécution live de la famille 0x5x hors 0x9a80** (`grep cur_op=0x5xxx` + exec_pc) pour dimensionner combien de SFTA/SFTL légitimes partagent `case 0x5` -> **borne le blast-radius** avant de re-partitionner cet arm de dispatch.
7. **Re-confirmer le gating shunt bout-en-bout** : relire `calypso_dsp_shunt_active()` et ses appelants (`calypso_bsp.c:404/462/956/1112` — gates ; feed calls 426/432 ; lignes ré-alignées 2026-07-01) en RO, pour certifier que le décodeur c54x est bypassé quand `CALYPSO_DSP_SHUNT=1`.

### Shunt-safety & blast-radius (note d'audit)

- **SHUNT-SAFE.** Le fix STAGE-1 vit entièrement dans le décodeur c54x (`case 0x5` / nouveau handler dadst/dsadt). En mode shunt (`CALYPSO_DSP_SHUNT=1`) le cœur c54x ne tourne pas (gaté par `calypso_dsp_shunt_active()` dans `calypso_bsp.c`), donc toute modif de l'arm 0x5x est du **dead code** côté full-grgsm. Run courant = mode c54x (`CALYPSO_DSP_SHUNT=0`, `CALYPSO_DSP_REG_MODE=c54x`, `CALYPSO_FORCE_TOA` absent — **CONFIRMÉ** via environ), précisément la cible.
- ~~**Blast-radius PRIMARY** : `case 0x5` (`c54x.c:7818`) … Un nouveau handler doit utiliser l'opérande Lmem …~~ — **DÉJÀ FAIT** : ✅ FIXÉ. Le `case 0x5:` est à `c54x.c:8225` ; le handler dual-long-word dédié (0x5A/5B DADST, 0x5E/5F DSADT via `resolve_lmem`, branches C16/SXM, DADST=+T / DSADT=−T) a été ajouté à **`c54x.c:8232-8305`** (2026-06-22), et les vrais SFTA/SFTL (0x50-55, 0x58-59) restent au sous-décode 8306-8331. Ne pas re-proposer ce patch : il a atterri.
- ~~**Co-requis probable (HYPOTHÈSE)** : le fix STAGE-1 seul peut ne PAS lever `d_fb_det=0` …~~ — **OBSOLÈTE** : le « fix STAGE-1 » a atterri (`c54x.c:8232-8305`) et `d_fb_det` reste 0 — ce qui **confirme** que la cause n'était pas là. Le vrai co-requis (bloquant) est ailleurs : le **handshake go-live ARM→DSP n'est jamais asserté** (`api_write_cb` jamais câblé, IMR=0, DSP déraillé). Le catch-all `0x7000` est à `c54x.c:7650` (ref corrigée). Aucun fichier BSP/ARM/IPC touché par le décodeur — le gap est côté ARM→DSP.

---

~~**Localisation la plus supportée (synthèse)** : … la famille DADST/DSADT (0x5x) **n'a aucun handler** et tombe dans le bloc SFTA/SFTL `case 0x5` (`c54x.c:7818`, branches 7838/7844) => …~~

**SYNTHÈSE CORRIGÉE (2026-07-01).** Cette conclusion est **caduque**. (1) ✅ FIXÉ : la famille DADST/DSADT (0x5x) a un handler dédié `c54x.c:8232-8305` (`case 0x5:` à 8225, sous-décode SFTA/SFTL résiduel à 8306-8331) ; les MAC dual-opérande 0x90-93 / 0xA4-B7 sont passés au décode 2-bit correct (`c54x.c:8342` / `8847`). Le mur n'est **pas** décode-attribuable. (2) **Vraie cause (vérité-terrain)** : le DSP ne tourne jamais le corrélateur — il déraille (POST-BOOTSTUB-RET, PC=0x0000), `IMR=0x0000` sur toute la run (jamais ré-armé après le clear boot @0xb37e), et le **handshake go-live ARM→DSP n'est jamais asserté** (l'ARM n'écrit que 0x0000 dans API 0x0314/0x0318 ; `api_write_cb` déclaré mais jamais câblé — `grep 'api_write_cb *=' = 0`). Le noyau `0x9a80..0x9a9f` / `0x9ac0` n'est pas atteint en régime permanent. **Le vrai gap à corriger est le câblage du handshake DSP go-live, pas le décodeur.** **NON-causes** (toutes confirmées) : `0xc8be` (correct), dual-MAC 0x90-B7 (fixé + froid), 0x5x DADST/DSADT (fixé), T=0 (réfuté), MVDM-revert (chaîne réfutée).
