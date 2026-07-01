# Pipeline du corrélateur FB/FCCH (c54x émulé) — composants, interactions, code C

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> **Ce que ce doc dit de faux** : (1) le `case 0x5` mis-route DADST/DSADT → **FAUX, déjà FIXÉ** : le handler dual long-word complet (0x50-0x5F) existe à `calypso_c54x.c:8232-8302` (fix « revival dsp » 2026-06-22), exécuté AVANT le fallback SFTA/SFTL (8305-8330). (2) « fixer DADST est nécessaire pour lever le mur d_fb_det=0 » → **FAUX** : le blocage terminal est en AMONT du corrélateur. Le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000) avant d'atteindre le steady-state ; IMR=0x0000 sur tout le run (jamais ré-armé après le clear boot @0xb37e) ; le handshake go-live ARM→DSP ne s'arme jamais (l'ARM n'écrit que 0x0000 dans l'API 0x0314/0x0318, `api_write_cb` déclaré mais jamais assigné). Avec le handler DADST présent, `d_fb_det` reste **toujours** 0 : la « racine décode-attribuable » n'est PAS load-bearing.

**Branche** `dsp_revival`. **But** : faire tourner le VRAI DSP TMS320C54x émulé
(`calypso_c54x.c`) pour la détection FB/FCCH, au lieu du mock gr-gsm.
**Statut** : ✅ le mis-décode `case 0x5` DADST/DSADT est **FIXÉ**
(`calypso_c54x.c:8232-8302`, 2026-06-22). Le mur `d_fb_det=0` **subsiste** malgré ce
fix : blocage réel en amont (DSP déraille, IMR jamais ré-armé, go-live ARM→DSP jamais
armé). Ce doc reste utile comme cartographie du corrélateur, mais son diagnostic
« racine décode » est **périmé** (voir bandeau).

Sources : `hw/arm/calypso/calypso_c54x.c`, `calypso_bsp.c`, `calypso_trx.c` ;
`doc/opcodes/tic54x_hi8_map.md` ; SPRU172C (DADST/DSADT) ; data live `/root/qemu.log`
(sonde SHADOW-DADST, run 2026-06-22).

> ℹ️ **NOTE d'audit** : les numéros de ligne ci-dessous ont été re-vérifiés au
> 2026-07-01 (`calypso_c54x.c` ~13697 lignes). Les anciens numéros (rédaction
> initiale) étaient périmés et ont été corrigés.

---

## 1. Pipeline end-to-end (radio → décision FB du firmware)

```
┌─────────────┐   IPC    ┌──────────────────────┐   DMA write    ┌───────────────────────────┐
│  osmo-trx   │─────────▶│  BSP (calypso_bsp.c) │───────────────▶│  DARAM  I/Q buffer        │
│  (radio DL) │  TRXDv0  │  bsp_take_for_fn :302 │  daram_addr=   │  [0x2a00..0x2b27] 296 mots│
└─────────────┘          │  daram_addr/len 111/112│  0x2a00       │  (+ buf réf @0x2c00)      │
                         │  DL FN-LOCK (offset)  │                └──────────────┬────────────┘
                         └──────────────────────┘                               │ *AR5 / *AR3 / *AR4
                                                                                 │  (lecture taps)
                                                                                 ▼
   firmware ARM                  MMIO read              ┌──────────────────────────────────────┐
┌──────────────────┐   0x01F0   ┌──────────────────┐   │   NOYAU CORRÉLATEUR FCCH (PROM)        │
│ prim_fbsb.c      │◀───────────│ calypso_trx.c    │◀──│   prog[0xa076..0xa09f] (ce run, page0) │
│ d_fb_det≠0 ?     │  d_fb_det  │ :258 FBDET-RD    │   │   prog[0x9a80..]      (run « wide »)   │
│ SNR>thr, TOA≈23  │  a_sync_*  │ :320 FORCE_TOA   │   │   → écrit A/B puis publie le résultat  │
└──────────────────┘            └──────────────────┘   └──────────────────┬───────────────────┘
        │                                                                  │ STL A,*AR2- (0x9ac0/publish)
        │ FBSB_CONF                              résultat publié           ▼
        ▼                                   ┌──────────────────────────────────────────────┐
   camping / SB / SI                        │ result slots (DARAM) : d_fb_det[0x08f8],      │
                                            │ a_sync_demod[D_TOA/PM/ANGLE/SNR @0x01F4..FA]  │
                                            └──────────────────────────────────────────────┘
```

Note run-variance : le PC du noyau dépend du mode FB du run (`d_fb_mode`). Run
« narrow » (0x0001) → `0xa077` ; run « wide » (0x4dfc) → `0x9a80`. **Même bug, même
famille d'opcode** (DADST/DSADT), PC différent.

---

## 2. Zoom : la boucle interne du corrélateur (1 groupe ×8) + handlers

✅ **FIXÉ (calypso_c54x.c:8232-8302, revival dsp 2026-06-22)** — le tableau ci-dessous
décrivait l'ANCIEN mis-décode ; il est conservé comme historique. Aujourd'hui DADST/DSADT
(et toute la famille 0x50-0x5F) sont décodés correctement AVANT le fallback SFTA/SFTL.

```
 PC     op     instruction réelle          handler C (calypso_c54x.c)              état
────────────────────────────────────────────────────────────────────────────────────────────
0xa076  3060   LD  coeff[DP:0x60],T   ───▶  (op&FF00)==0x3000  :5302  +resolve_smem:3728       OK
0xa077  5a85   DADST *AR5,A           ───▶  case 0x5 :8226 → DADST handler :8232-8302          ✅ FIXÉ
0xa078  5f95   DSADT *AR5+,B          ───▶  case 0x5 :8226 → DSADT handler :8232-8302          ✅ FIXÉ
0xa079  8e94   CMPS A,*AR4+           ───▶  case 0x8 (hi8 0x8E)       +resolve_smem             OK
0xa07a  8f93   CMPS B,*AR3+           ───▶  case 0x8 (hi8 0x8F)       +resolve_smem             OK
  … ×8 groupes (DADST/DSADT alternent dst A/B) …
0xa09b  5fd5   DSADT *AR5+0%,B (circ) ───▶  case 0x5 :8226 → DSADT handler :8232-8302          ✅ FIXÉ
0xa09f  fc00   RET / fin de bloc
```

**Historique (bug fermé)** — avant le fix, `case 0x5` traitait DADST/DSADT en SFTA/SFTL
(fallback aujourd'hui à 8305-8330), ce qui détruisait :
- la lecture du **Lmem 32-bit** à `*AR5` → jamais lue comme opérande ;
- le **± T** (dual add/sub, le cœur du calcul) → remplacé par `A >>= ASM` (ASM=-3) ;
- le **post-mod `*AR5+`** (stride dans le burst) → DROPPÉ (`walked=0` sur 40/40) ;
- le **wrap circulaire `*AR5+0%`** (BK=0x20) → DROPPÉ.

⇒ Symptôme d'alors : A décalé (`shiftLike=1`), B intouché (`dB=0`), 0 corrélation.
Le handler actuel (8232-8302) lit le Lmem via `resolve_lmem`, applique ±T et branche sur
`ST1.C16`. **MAIS** cela ne suffit PAS à sortir de `d_fb_det=0` : le DSP n'atteint jamais
le steady-state du corrélateur (déraille, IMR jamais ré-armé — voir bandeau en tête).

---

## 3. Composants d'état & registres → handler C

```
 composant            rôle dans le corrélateur            handler / champ C                    état
─────────────────────────────────────────────────────────────────────────────────────────────────
 T  (uint16_t t)      réf cos/sin, table [0x60..0x63]     LD Smem,T :5302                      OK
 A,B (int64_t)        accumulateurs corrélation I/Q       s->a / s->b                          OK
 AR5 (ar[5])          pointeur taps I/Q (*AR5, stride)    DADST/DSADT :8232-8302 (resolve_lmem)✅ FIXÉ (post-mod ±2 géré)
 AR3,AR4              pointeurs peak/magnitude (CMPS)      case 0x8 +resolve_smem               OK
 BK  (uint16_t bk)    taille buffer circulaire (=0x20)    c54x_circ_ref :3716                  OK
 C16 (ST1 bit7)       mode dual-16 vs double-precision    SSBX :6755 / RSBX :6551 (calypso_c54x.h:83) OK (suivi correctement)
 ASM (ST1[4:0])       shift mode (parasite ici)           asm_shift()                          — (ne devrait pas s'appliquer)
 DP  (ST0[8:0])       page data (adressage direct de T)   resolve_smem                         OK
```

---

## 4. Le locus du fix (une seule racine, partagée) — ✅ FIXÉ

✅ **DONE (calypso_c54x.c:8232-8302, revival dsp 2026-06-22).** Le fix décrit ci-dessous
comme TODO a **atterri** : `case 0x5` (`:8226`) intercepte toute la plage 0x50-0x5F AVANT
le fallback shift (8305-8330), décode le Lmem via `resolve_lmem` (post-mod ±2) et branche
sur `ST1.C16`. La section est conservée comme spéc de référence.

```
                       case 0x5  (calypso_c54x.c:8226 ; handler dual :8232-8302)
                       ┌───────────────────────────────────────────────┐
   op 0x50-0x5F  ───▶  │  ✅ décodé dual long-word (fix 2026-06-22)      │
                       │  RÉEL (tic54x_hi8_map.md:59-65) :              │
                       │    0x50-53 DADD   0x54-55 DSUB   0x56-57 DLD   │
                       │    0x58-59 DRSUB  0x5A-5B DADST  0x5C-5D DSUBT │
                       │    0x5E-5F DSADT  → TOUS dual long-word        │
                       └───────────────────────────────────────────────┘
```

**Fix appliqué** : `0x5A/5B/5E/5F` (et toute la plage 0x50-0x5F) sont interceptés AVANT
le bloc shift, le Lmem décodé via `resolve_lmem` (post-mod ±2), branche sur
`s->st1 & ST1_C16` :

```
 DADST Lmem,dst (0x5A=A, 0x5B=B) :
   C16=1 : dst(39-16) = Lmem(31-16) + T ;  dst(15-0) = Lmem(15-0) − T
   C16=0 : dst = Lmem + ((T<<16) + T)
 DSADT Lmem,dst (0x5E=A, 0x5F=B) :
   C16=1 : dst(39-16) = Lmem(31-16) − T ;  dst(15-0) = Lmem(15-0) + T
   C16=0 : dst = Lmem − ((T<<16) + T)
 (dst = bit8 → A/B ; Lmem = paire 32-bit data[L]=high, data[L+1]=low, cf DST :8216)
```

Gaté c54x (dead code en `CALYPSO_DSP_SHUNT=1`). Le repartitionnement du `case 0x5` a été
fait : aucun vrai SFTA/SFTL ne vit dans 0x5x que le handler dual intercepterait à tort
(le handler ne prend que 0x50-0x5F long-word, le fallback shift 8305-8330 gère le reste).

> ⚠️ **Rappel** : ce fix, bien qu'atterri, ne lève PAS `d_fb_det=0`. Le blocage réel est
> en amont (DSP déraille, IMR jamais ré-armé, go-live ARM→DSP jamais armé — voir bandeau).

---

## 5. Évidence runtime (sonde SHADOW-DADST, run 2026-06-22, 40/40 échantillons)

- Noyau exécuté à `pc=0xa077..0xa09b` (page0), opcodes `5a85/5f95/5e85/5b95/5fd5`.
- `shiftLike=1` sur 40/40 : A = `A0 >> 3` (ASM=-3) — vérifié bit-à-bit
  (#4 `1fd1a1e420→03fa343c84`, #30 `007ffceeed→000fff9ddd`).
- `dB=0` sur 40/40 : B jamais touché par ces ops (le SFTL ne cible que A).
- `walked=0` sur 40/40 : AR5 jamais post-incrémenté ⇒ **R1 (mis-décode) subsume la
  co-cause « AR hors-buffer »** : le même post-mod perdu. Fixer DADST/DSADT avec
  post-mod corrige la marche AR en effet de bord → on évite le fix dangereux du
  catch-all 0x7000 (landmine MVDM-revert).
- `C16=0`, `BK=0x20`, `ASM=-3` constants. C16=0 ⇒ double-precision ICI (mais le
  handler doit gérer les deux, l'ému suit C16 fidèlement).

Caveat ouvert : ce noyau tourne au **boot** ; au steady-state, FB est retenté via
d'autres PC (`0xf171/0x82xx`). ~~Le fix DADST est nécessaire ; reste à vérifier la
propagation au chemin FB permanent.~~ — **FAUX / PÉRIMÉ** : le fix DADST a atterri
(`calypso_c54x.c:8232-8302`) et `d_fb_det` reste **0** quand même. Le chemin FB permanent
n'est jamais atteint : le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), l'IMR reste 0x0000
tout le run (jamais ré-armé après le clear boot @0xb37e), et le handshake go-live ARM→DSP
ne s'arme jamais (`api_write_cb` déclaré dans `calypso_c54x.h` mais **jamais assigné** —
`grep 'api_write_cb *='` = 0 ; l'ARM n'écrit que 0x0000 dans l'API 0x0314/0x0318). Le vrai
gap n'est donc PAS le décode DADST mais ce handshake amont.

---

## 6. Références

- `calypso_c54x.c` : `case 0x5` 8226 (handler dual long-word ✅ 8232-8302, fallback
  SFTA/SFTL 8305-8330) ; `resolve_smem` 3728 ; `c54x_circ_ref` 3716 ; LD Smem,T 5302 ;
  SSBX 6755 / RSBX 6551 (ST1.C16, `calypso_c54x.h:83`) ; DST Lmem (paire 32-bit) 8216 ;
  sonde SHADOW-DADST 12189 (pre-capture) / 12342 (post-compute).
- `calypso_bsp.c` : `bsp_take_for_fn` 302 ; `daram_addr/len` 111/112.
- `calypso_trx.c` : FBDET-RD 258 (lecture d_fb_det @0x01F0 :262) ; FORCE_TOA ~320.
- `doc/opcodes/tic54x_hi8_map.md` 59-65 (encodage 0x50-0x5F) ; ST1 bit7 = C16.
- SPRU172C (DADST/DSADT, classes 9A/9B) ; `doc/datasheets/TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf`, `doc/datasheets/`.
- Mémoire projet : `calypso-dsp-revival-c54x-route`.
