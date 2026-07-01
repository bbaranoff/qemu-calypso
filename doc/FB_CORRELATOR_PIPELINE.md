# Pipeline du corrélateur FB/FCCH (c54x émulé) — composants, interactions, code C

**Branche** `dsp_revival`. **But** : faire tourner le VRAI DSP TMS320C54x émulé
(`calypso_c54x.c`) pour la détection FB/FCCH, au lieu du mock gr-gsm.
**Statut** : le mur `d_fb_det=0` est localisé à une racine décode-attribuable (le
`case 0x5` mis-route la famille dual long-word). Ce doc = la cartographie de
référence pour la phase fix.

Sources : `hw/arm/calypso/calypso_c54x.c`, `calypso_bsp.c`, `calypso_trx.c` ;
`doc/opcodes/tic54x_hi8_map.md` ; SPRU172C (DADST/DSADT) ; data live `/root/qemu.log`
(sonde SHADOW-DADST, run 2026-06-22).

---

## 1. Pipeline end-to-end (radio → décision FB du firmware)

```
┌─────────────┐   IPC    ┌──────────────────────┐   DMA write    ┌───────────────────────────┐
│  osmo-trx   │─────────▶│  BSP (calypso_bsp.c) │───────────────▶│  DARAM  I/Q buffer        │
│  (radio DL) │  TRXDv0  │  bsp_take_for_fn :272 │  daram_addr=   │  [0x2a00..0x2b27] 296 mots│
└─────────────┘          │  daram_addr/len 109/110│  0x2a00       │  (+ buf réf @0x2c00)      │
                         │  DL FN-LOCK (offset)  │                └──────────────┬────────────┘
                         └──────────────────────┘                               │ *AR5 / *AR3 / *AR4
                                                                                 │  (lecture taps)
                                                                                 ▼
   firmware ARM                  MMIO read              ┌──────────────────────────────────────┐
┌──────────────────┐   0x01F0   ┌──────────────────┐   │   NOYAU CORRÉLATEUR FCCH (PROM)        │
│ prim_fbsb.c      │◀───────────│ calypso_trx.c    │◀──│   prog[0xa076..0xa09f] (ce run, page0) │
│ d_fb_det≠0 ?     │  d_fb_det  │ :208 FBDET-RD    │   │   prog[0x9a80..]      (run « wide »)   │
│ SNR>thr, TOA≈23  │  a_sync_*  │ :291 FORCE_TOA   │   │   → écrit A/B puis publie le résultat  │
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

```
 PC     op     instruction réelle          handler C (calypso_c54x.c)         état
────────────────────────────────────────────────────────────────────────────────────────
0xa076  3060   LD  coeff[DP:0x60],T   ───▶  (op&FF00)==0x3000  :5114  +resolve_smem:3578   OK
0xa077  5a85   DADST *AR5,A           ───▶  case 0x5           :7817  → SFTA/SFTL           BUG (wrong-op)
0xa078  5f95   DSADT *AR5+,B          ───▶  case 0x5           :7817  → SFTL, *AR5+ droppé  BUG (+ post-mod perdu)
0xa079  8e94   CMPS A,*AR4+           ───▶  case 0x8 (hi8 0x8E)       +resolve_smem         OK
0xa07a  8f93   CMPS B,*AR3+           ───▶  case 0x8 (hi8 0x8F)       +resolve_smem         OK
  … ×8 groupes (DADST/DSADT alternent dst A/B) …
0xa09b  5fd5   DSADT *AR5+0%,B (circ) ───▶  case 0x5           :7817  → SFTL, BK ignoré     BUG (+ circulaire perdu)
0xa09f  fc00   RET / fin de bloc
```

Ce que le bug (`case 0x5` = SFTA/SFTL only, lignes 7817-7852) détruit, par
instruction DADST/DSADT :
- la lecture du **Lmem 32-bit** à `*AR5` → jamais lue comme opérande ;
- le **± T** (dual add/sub, le cœur du calcul) → remplacé par `A >>= ASM` (ASM=-3) ;
- le **post-mod `*AR5+`** (stride dans le burst) → DROPPÉ (`walked=0` sur 40/40) ;
- le **wrap circulaire `*AR5+0%`** (BK=0x20) → DROPPÉ.

⇒ A n'est que décalé (`shiftLike=1`), B jamais touché (`dB=0`) : **0 corrélation calculée**.

---

## 3. Composants d'état & registres → handler C

```
 composant            rôle dans le corrélateur            handler / champ C                    état
─────────────────────────────────────────────────────────────────────────────────────────────────
 T  (uint16_t t)      réf cos/sin, table [0x60..0x63]     LD Smem,T :5114                      OK
 A,B (int64_t)        accumulateurs corrélation I/Q       s->a / s->b                          OK (nourris faux)
 AR5 (ar[5])          pointeur taps I/Q (*AR5, stride)    DADST/DSADT post-mod → case 0x5      BUG (post-mod perdu)
 AR3,AR4              pointeurs peak/magnitude (CMPS)      case 0x8 +resolve_smem               OK
 BK  (uint16_t bk)    taille buffer circulaire (=0x20)    c54x_circ_ref :3566                  OK (jamais appelé par case 0x5)
 C16 (ST1 bit7)       mode dual-16 vs double-precision    SSBX/RSBX C16 (F7B7/F6B7) :6529/6330 OK (suivi correctement)
 ASM (ST1[4:0])       shift mode (parasite ici)           asm_shift()                          — (ne devrait pas s'appliquer)
 DP  (ST0[8:0])       page data (adressage direct de T)   resolve_smem                         OK
```

---

## 4. Le locus du fix (une seule racine, partagée)

```
                       case 0x5  (calypso_c54x.c:7817-7852)
                       ┌───────────────────────────────────────────────┐
   op 0x50-0x5F  ───▶  │  AUJOURD'HUI : tout traité en SFTA/SFTL        │  BUG
                       │  RÉEL (tic54x_hi8_map.md:59-65) :              │
                       │    0x50-53 DADD   0x54-55 DSUB   0x56-57 DLD   │
                       │    0x58-59 DRSUB  0x5A-5B DADST  0x5C-5D DSUBT │
                       │    0x5E-5F DSADT  → TOUS dual long-word        │
                       └───────────────────────────────────────────────┘
```

**FIX** = intercepter `0x5A/5B/5E/5F` (et idéalement toute la plage 0x50-0x5F) AVANT
le bloc shift, décoder Lmem via `resolve_smem` (+ `c54x_circ_ref` pour BK), et
brancher sur `s->st1 & ST1_C16` :

```
 DADST Lmem,dst (0x5A=A, 0x5B=B) :
   C16=1 : dst(39-16) = Lmem(31-16) + T ;  dst(15-0) = Lmem(15-0) − T
   C16=0 : dst = Lmem + ((T<<16) + T)
 DSADT Lmem,dst (0x5E=A, 0x5F=B) :
   C16=1 : dst(39-16) = Lmem(31-16) − T ;  dst(15-0) = Lmem(15-0) + T
   C16=0 : dst = Lmem − ((T<<16) + T)
 (dst = bit8 → A/B ; Lmem = paire 32-bit data[L]=high, data[L+1]=low, cf DST :7807)
```

Gaté c54x (dead code en `CALYPSO_DSP_SHUNT=1`). Blast-radius = `case 0x5`, arm de
dispatch partagé : repartitionner proprement et confirmer qu'aucun vrai SFTA/SFTL ne
vit dans 0x5x (la doc dit que non — les shifts sont ailleurs).

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
d'autres PC (`0xf171/0x82xx`). Le fix DADST est **nécessaire** ; reste à vérifier la
**propagation** au chemin FB permanent.

---

## 6. Références

- `calypso_c54x.c` : `case 0x5` 7817-7852 ; `resolve_smem` 3578 ; `c54x_circ_ref`
  3566 ; LD Smem,T 5114 ; SSBX/RSBX C16 6529/6330 ; DST Lmem (paire 32-bit) 7807 ;
  sonde SHADOW-DADST 11238/11252.
- `calypso_bsp.c` : `bsp_take_for_fn` 272 ; `daram_addr/len` 109/110.
- `calypso_trx.c` : FBDET-RD 208 ; FORCE_TOA 291.
- `doc/opcodes/tic54x_hi8_map.md` 59-65 (encodage 0x50-0x5F) ; ST1 bit7 = C16.
- SPRU172C (DADST/DSADT, classes 9A/9B) ; `doc/datasheets/TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf`, `doc/datasheets/`.
- Mémoire projet : `calypso-dsp-revival-c54x-route`.
