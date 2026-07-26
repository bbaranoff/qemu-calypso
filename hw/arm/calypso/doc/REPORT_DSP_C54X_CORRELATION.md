> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> **Correction majeure — la thèse « racine = PORTW 0x75 sous-décodé » (§1 réécrit + §9 ADDENDUM) est CADUQUE.** Le handler PORTW 0x75 3-mots (Smem abs) que l'ADDENDUM présentait comme le « kill-shot » **a déjà atterri** dans le tree : `calypso_c54x.c:7501` (`if ((op & 0xFF00) == 0x7500)`, revival 2026-06-23), **inconditionnel** (contrairement à PORTR 0x74 qui reste gardé `CALYPSO_FIX_PORTR`). Le commentaire @7485-7500 reprend mot pour mot le récit de ce doc (CC fantôme @0xb418, entrée nue dans 0x76f8). **Or ce fix N'A PAS débloqué d_fb_det** : per vérité-terrain, le DSP déraille toujours (POST-BOOTSTUB-RET, PC=0x0000), l'IMR reste 0x0000 (jamais ré-armé après le clear boot @0xb37e), et le handshake go-live ARM→DSP n'est jamais asserté (l'ARM n'écrit que 0x0000 vers les API 0x0314/0x0318). **La chaîne PORTW→CC fantôme→sur-pop→collapse est donc RÉFUTÉE comme cause opérante** : le vrai blocage terminal est le **handshake go-live ARM→DSP jamais câblé** (`api_write_cb` déclaré dans `calypso_c54x.h` mais JAMAIS assigné — `grep 'api_write_cb *=' = 0`) et l'**IMR jamais ré-armé**. Le lever `CALYPSO_DSP_FRAME_VEC28` est un cul-de-sac (force-vectorise mais atterrit dans l'épilogue ISR → déraille vers le boot-stub, IMR reste 0). Voir §9 pour le détail.

# Corrélation firmware calypso_dsp.txt ↔ décodeur calypso_c54x.c

## 1. Résumé exécutif

**Verdict principal : le sur-pop structurel de −1 mot/passe ancré à 0x76f8 N'EST PAS un bug du décodeur.** Tous les opcodes de pile sur le chemin exécuté (CC, CALLD, POPM, RC/RETD, FRET, PSHM) sont décodés *fidèlement* par l'émulateur — bon nombre de mots, bonne direction (push/pop), bonne arithmétique d'adresse de retour (PC+2 pour les formes non-retardées, PC+4 pour CALLD). Le sur-pop est **intrinsèque au chemin firmware tel qu'il est entré** : il s'agit d'une **entrée par mauvais chemin (wrong-path entry)**. La routine 0x76f8 est une **épilogue de restauration de contexte** (POPM ST1 @0x7706 + RETD @0x7707 = 2 pops) qui *exige* un prologue PSHM ST1 (op 0x4A07) et une trame d'appelant en dessous. Le CC@0xb418 — premier op de pile après reset — y entre SANS ce prologue. Le second pop (RETD) tire donc un slot « vierge » au-dessus de la base de push du CC. Résultat : 1 push (CC) contre 2 pops → −1 mot/passe ; SP grimpe 0x10ff → 0x1144 puis le STM #lk,SP s'effondre (0x1144 → 0x0079) et le DSP déraille → d_fb_det reste à 0.

**Point empirique décisif** : la trace a été capturée AVEC le fix SQURA actif (SQURA-A montre A=0x000fc92f44 > 0, donc le RCD LEQ @0x75e8 ne prend PAS la sortie anticipée et le corps de la callee s'exécute jusqu'au RETD @0x7600). Le sur-pop survient *quand même* à insn=164 → cela **réfute l'hypothèse** « A>0 ⇒ RCD non-pris ⇒ le corps qui fait PSHM ST1 s'exécute ». Vérification mot-par-mot : **aucun PSHM ST1 (0x4A07) n'existe nulle part** sur le chemin exécuté (le seul PSHM proche est 0x4A06 = PSHM ST0 @0x770d, sur une branche disjointe).

**Mismatches confirmés affectant la pile / les décisions de branche sur le chemin exécuté, classés par proximité causale à d_fb_det=0 :**

| Rang | Site | Nature | Affecte pile ? | Affecte branche ? | Statut adversarial |
|------|------|--------|----------------|-------------------|--------------------|
| — | 0x7706/0x7707 (le sur-pop lui-même) | POPM ST1 + RETD orphelins | OUI (cause racine) | non | **réfuté comme bug décodeur** (décode correct ; bug = routage d'entrée) |
| 1 | 0x75f3 (1882 AND→LD) | dataflow alimentant cond RCD LEQ | non | potentiellement | réfuté (non sur chemin prouvé) |
| 2 | 0xb416 (75f8 PORTW→STL) | longueur 3w vs 2w gate le CC@0xb418 | indirect | OUI (gate le CC) | **uncertain** (seul non-clos) |

**Aucun mismatch confirmé comme bug-décodeur causal de d_fb_det=0.** Le décodeur est fidèle ; le correctif appartient au **routage de contrôle amont** (pourquoi le contrôle atteint l'épilogue POPM-ST1 de 0x76f8 sans son prologue PSHM-ST1).

## 2. Chemin exécuté (boot → leak → collapse)

Table unique ordonnée selon l'exécution réelle (reset → boot → callee → épilogue → continuation → collapse).

| addr | raw | ISA mnémonique | mots | effet pile | décode ému | match | sévérité |
|------|-----|----------------|------|-----------|-----------|-------|----------|
| 0xff80 | f880 b410 | FB 0xb410 (far, XPC=0) | 2 | none | FB far, XPC=0, pc=0xb410 (L6124-6138) | ✓ | none |
| 0xb410 | 69f8 001d 0028 | ORM #0x0028,*(0x001d) → PMST | 3 | none | ORM abs-Smem, write MMR 0x1D (L7458-7465) | ✓ | none |
| 0xb413 | 76f8 000e 0000 | ST #0,*(0x000e) → T | 3 | none | ST #lk,Smem abs (L7273-7285) | ✓ | none |
| 0xb416 | 75f8 000e | **PORTW *(0x000e),PA (3w)** | ✅ **3 ISA / 3 ému (FIXÉ)** | none | ✅ FIXÉ : handler PORTW 0x75 3-mots abs (L7501, revival 2026-06-23, inconditionnel) — consomme opcode+PA+lk, ne glisse plus | ✓ | none (fixé, mais SANS effet sur d_fb_det — voir bannière) |
| 0xb418 | f900 76f8 | CC 0x76f8, UNC (near) | 2 | **push1 (PC+2=0xb41a)** | CC cond UNC, push PC+2, SP 0x1100→0x10ff (L6824) | ✓ | none |
| → 0x76f8 | 0014 | ADD dma(0x14),A | 1 | none | case 0x0 ADD direct (L3599) | ✓ | none |
| 0x76f9 | 771a 003f | STM #0x3F,BRC | 2 | none | STM MMR BRC=63 (L7287-7313) | ✓ | none |
| 0x76fb | f272 7700 | RPTBD 0x7700 (RSA=0x76ff) | 2 | none | F272 RPTBD, REA=0x7700, RSA=pc+4=0x76ff (L5873-5885) | ✓ | none |
| 0x76fd | e800 | LD #0,A (slot délai 1) | 1 | none | E8 LD #k8u dst A (L7093-7104) | ✓ | none |
| 0x76fe | f6b9 | RSBX 9 (slot délai 2) | 1 | none | F6Bx RSBX (L6407-6413) | ✓ | none |
| 0x76ff | 3892 | SQURA *AR2+,A | 1 | none | case 0x3 SQURA (fix 06-23, L7751-7757) | ✓ | none |
| 0x7700 | 3892 | SQURA *AR2+,A (REA) | 1 | none | SQURA + redirect boucle (L12260-12281) | ✓ | none |
| 0x7701 | f7b9 | SSBX 9 | 1 | none | F7Bx SSBX (L6613-6620) | ✓ | none |
| 0x7702 | f274 75e8 | CALLD 0x75e8 (retardé) | 2 | **push1 (PC+4=0x7706)** | F274 CALLD, push pc+4=0x7706, delay 2 (L6015) | ✓ | none |
| 0x7704 | f478 | SFTA A,-8,A (slot 1) | 1 | none | F460 SFTA (L5367-5375) | ✓ | none |
| 0x7705 | f483 | SAT A (slot 2) | 1 | none | F483 SAT (L5431-5438) | ✓ | none |
| → 0x75e8 | fe47 | RCD LEQ (retour cond. retardé, A≤0) | 1 | pop1 si pris | FE cc=0x47 ALEQ, pop si pris (L7057-7112) | ✓ | none |
| 0x75e9 | f48e | EXP A | 1 | none | F48E EXP (L5475-5491) | ✓ | none |
| 0x75ea | e900 | LD #0,B | 1 | none | E9 LD #k8u dst B (L7093) | ✓ | none |
| 0x75eb | f48f | NORM A | 1 | none | F48F NORM (L5523-5539) | ✓ | none |
| 0x75ec | f477 | SFTA A,-9,A | 1 | none | F460 SFTA (L5632) | ✓ | none |
| 0x75ed | f00f 0001 | ADD #1,SHIFT=-1,A | 2 | none | FCF0==F000 ADD #lk (L6302-6337) | ✓ | none |
| 0x75ef | 8282 | STH A,*AR2 | 1 | none | hi8 0x82 STH (L8449) — **MOD-FIRSTHIT confirmé** | ✓ | none |
| 0x75f0 | 6182 0040 | BITF *AR2,#0x0040 → TC | 2 | none | 0x6100 BITF (L7374-7420) | ✓ | none |
| 0x75f2 | e81f | LD #0x1F,A | 1 | none | E8 LD #k8u (L7093) | ✓ | none |
| 0x75f3 | 1882 | **AND *AR2,A** | 1 | none | **mis-décodé LD *AR2,A** (case 0x1 défaut L7648) | **✗** | med |
| 0x75f4 | f000 7603 | ADD #0x7603,A | 2 | none | F000 ADD #lk (L6302) | ✓ | none |
| 0x75f6 | ff30 | XC 2,TC | 1 | none | FF XC n=2 cc=TC (L4688-4743) | ✓ | none |
| 0x75f7 | f000 0020 | ADD #0x20,A | 2 | none | F000 ADD #lk (L6302) | ✓ | none |
| 0x75f9 | 7e82 | READA *AR2 | 1 | none | 0x7E READA (L7230-7239) | ✓ | none |
| 0x75fa | e91e | LD #0x1E,B | 1 | none | E9 LD #k8u (L7093) | ✓ | none |
| 0x75fb | 10f8 000e | LD *(0x000E),A | 2 | none | case 0x1 LD abs-Smem (L7611/L3731) | ✓ | none |
| 0x75fd | f520 | F5xx arith-mirror | 1 | none | **mis-décodé RPT #0x20** (bloc F5xx L6717-6719) | **✗** | low |
| 0x75fe | f661 | SFTA A,1,B | 1 | none | F460 SFTA correct (L5367) [claim B=A réfutée] | ✓ | none |
| 0x75ff | f500 | ADD A,0,B | 1 | none | **mis-décodé RPT #0x00** (bloc F5xx L6717-6719) | **✗** | low |
| 0x7600 | fe00 | RETD UNC (retour retardé) | 1 | pop1 | FE cc=0x00 UNC, pop1 (L7057-7112) — **POP confirmé trace** | ✓ | none |
| → 0x7706 | 8a07 | **POPM ST1** | 1 | **pop1** | 0x8A00 POPM mmr=0x07, pop1 (L8484) — pop 0xb41a | ✓ décode | **high (site)** |
| 0x7707 | fe00 | **RETD UNC** | 1 | **pop1 (vierge)** | FE cc=0x00 UNC, pop1, ret_tgt=0x7712 (L7057) — **ORPHAN-RETURN net_words=−1** | ✓ décode | **high (site)** |
| 0x7708 | 81f8 3fb2 | STL A,*(0x3fb2) | 2 | none | case 0x8 STL abs (L3825) | ✓ | none |
| 0x770a | f074 770d | CALL 0x770d (firmware near CALL) | 2 | push1 | F0xx legacy CALL-like push1 (L5678+) | ✓* | low (PC+2/+4 non vérifié) |
| 0x770c | f4e4 | FRET (far return) | 1 | **pop2 (PC+XPC)** | F4E4 FRET pop2 (L4923-4948) | ✓ | med (amplifie runaway) |
| 0x770d | 4a06 | PSHM ST0 (≠ ST1) | 1 | push1 | 0x4A PSHM mmr=0x06 (L8182) | ✓ | low |
| 0x7712 | f6b7 | RSBX C16 (landing du ret_tgt corrompu) | 1 | none | F6Bx RSBX (L6406) | ✓ | med (landing) |
| 0x7713 | 7212 3fb5 | MVDM 0x3fb5,AR2 | 2 | none | hi8 0x72 MVDM (case 0x7) | ✓ | none |
| 0x771c | f945 777a | CC 0x777a, AEQ | 2 | push1 (PC+2) si pris | F9 near CC push PC+2 (L6685) — SP-RING `771c:f945-1` | ✓ | med |
| 0x771e | f074 7794 | CALL 0x7794 (firmware near CALL) | 2 | push1 | F0xx legacy push1, balancé par RETD@0x7791 | ✓* | low |
| (ailleurs) | 77xx (MMR=0x18=SP) | **STM #lk,SP** | 2 | écrit SP | STM MMR SP — **SP-COLLAPSE 0x1144→0x0079** | ✓ | (effondrement) |

\* PC+2 vs PC+4 pour 0xF074 non confirmé indépendamment (voir §6).

**Note d'alignement** : fw[0x7718]=0xe721 (op groupe E7, 1 mot, AUCUNE écriture SP) — ce N'EST PAS le STM #lk,SP de la narration du hunt. Le STM qui effondre SP est un opcode 0x77xx (MMR=0x18=SP) à un PC différent ; « 0x7718 » dans le brief désignait la valeur-opcode, pas fw[0x7718].

## 3. Mismatches confirmés

### 3.1 — 0xb416 `75f8 000e` : PORTW décodé comme STL (longueur 3w vs 2w) — ✅ FIXÉ (`calypso_c54x.c:7501`, 2026-06-23) — SANS effet sur d_fb_det

> ⚠️ **PÉRIMÉ (audit 2026-07-01).** Le verdict « UNCERTAIN » et le « ne pas ajouter de handler PORTW » ci-dessous sont dépassés : un handler PORTW 0x75 3-mots (Smem abs) **a été ajouté et est inconditionnel** (`L7501`, `if ((op & 0xFF00) == 0x7500)`). Le décode est désormais 3-mots, le CC fantôme @0xb418 ne se produit plus. **MAIS** d_fb_det reste 0 et le DSP déraille toujours (vérité-terrain) : ce site n'était donc **pas** la cause opérante. Le texte historique est conservé ci-dessous.

- **ISA** : `PORTW *(0x000e),PA` — hi8 0x75 = portw (tic54x_hi8_map.md:83 ; SPRU172C:1769 ; détail cycles SPRU172C:21006-21024). Avec Smem absolu/long-offset (0xF8) = **3 mots** : word0=0x75f8, word1=0x000e (adresse data lk), word2=0xf900 (port PA). Même structure abs-Smem que le 0x76f8 précédent @0xb413 que l'ému traite, lui, comme 3 mots.
- **Émulateur** : `STL A,*(0x000e)` — **2 mots**. Aucun handler 0x75 dédié n'existe ; hi8==0x76 (L7273) est FALSE pour 0x75, hi8==0x77 (L7288) FALSE → chute dans le STL générique `(op&0xF800)==0x7000` (L7336-7342). resolve_smem case 0xF (L3731-3734) met lk_used=true, addr=0x000e ; consumed(1)+lk_used(1)=2.
- **Impact** : **branche/contrôle**. La différence d'1 mot détermine si le **CC@0xb418 (0xf900) est exécuté ou consommé comme opérande PA de PORTW**. Le décode 2-mots de l'ému fait *atterrir* le fetch suivant sur 0xb418 → le CC se déclenche (qemu.log : `pc=0xb418 op=0xf900 PUSH1 SP→0x10ff`) → entrée dans 0x76f8 → chaîne de sur-pop. Un PORTW strict 3-mots avalerait 0xb418 et SAUTERAIT le CC → 0x76f8 jamais appelée. Sémantique aussi divergente (store A→T vs port-write T→PA) mais bénigne ici (A=0, T=0 déjà ; BOOT-MMR-WR #2 : 0000→0000).
- **Pourquoi UNCERTAIN** : tension non résolue. Par les docs autoritaires, le décode ému EST faux (mismatch réel, solide). MAIS le décode « faux » 2-mots est **porteur/compensatoire** — c'est ce qui permet au boot d'atteindre sa routine d'init obligatoire. Le silicium Calypso réel démarre, donc le CC@0xb418 *doit* se déclencher sur HW réel, ce qui est incompatible avec 0xb416=vrai PORTW 3-mots qui le sauterait. Soit le mapping binutils 0x75=PORTW est mal-appliqué à ce flux d'octets, soit une instruction amont est mal-modélisée et l'alignement de la program-page ici est suspect. Irrésoluble depuis les seules entrées fournies.
- **Fix proposé** : **NE PAS** ajouter naïvement un handler PORTW 3-mots pour 0x75 (avalerait le CC, empêcherait l'init BRC/AR/BK, déraillerait le boot pire). D'abord **résoudre l'alignement** : confirmer si le flux silicium @0xb416 est vraiment 0x75f8 (comparer DATA-table page vs PROGRAM page pour 0xb410 ; tracer depuis reset 0xff80→B 0xb410 pour vérifier les frontières 0x69f8/0x76f8 byte-alignées). Si 0xb416 EST vraiment PORTW sur silicium, alors le CC est atteint autrement et le bug est ailleurs ; sinon le STL 2-mots est accidentellement compatible et aucun changement n'est justifié à ce site.

### 3.2 — 0x75f3 `1882` : AND décodé comme LD (dataflow) — VERDICT REFUTED (non-causal prouvé)

- **ISA** : `AND *AR2,A` (op 0x1882 : hi8=0x18 → AND, masque 0xFE00 base 0x1800 ; bit8 D=0 → dst A ; Smem 0x82 → indirect *AR2). 1 mot.
- **Émulateur** : `LD *AR2,A`. case 0x1 (L7611), sub=(op>>9)&7=4. Le switch (L7627) n'implémente que sub 0=LD,1=LDU,2=LD,TS,3=LDR ; sub 4-7 (AND/OR/XOR/SUBC) tombent au défaut (L7648-7652) = load simple. A est écrasé par data[*AR2] au lieu d'être masqué.
- **Impact** : **dataflow uniquement** (les deux formes sont 1 mot, sans push/pop → AUCUN effet pile). A alimente la condition RCD LEQ (A≤0) @0x75e8, donc en théorie pourrait inverser une décision branch-taken → coté MED. **MAIS réfuté** : (a) aucun effet SP — ce n'est pas la classe de bug du hunt ; (b) **pas sur le chemin prouvé** — le tracer dédié L4642 (`if (s->pc==0x75e8)`) journalise les 50 premières arrivées au RCD ; grep qemu.log = ZÉRO ligne RCD-75e8, donc le PC n'atteint jamais 0x75e8 ni 0x75f3 dans ce run. Le seul token 75f3 dans qemu.log est SP_new=0x75f3 (valeur de pointeur de pile), pas un PC ; (c) **direction d'alimentation incorrecte** — 0x75f3 est APRÈS 0x75e8 en adresse, le flux droit va vers l'avant, pas en arrière vers le RCD.
- **Fix proposé** (hygiène, basse priorité) : compléter le switch case 0x1 (L7627) avec sub 4 (0x1800 AND), 5 (0x1A00 OR), 6 (0x1C00 XOR), 7 (0x1E00 SUBC) sur les 16 bits bas de l'accumulateur destination, au lieu de la chute vers LD. N'affecte pas le hunt FB-detect.

### 3.3 — 0x75fd `f520` / 0x75ff `f500` : arith F5xx décodé comme RPT #k — VERDICT REFUTED

- **ISA** : famille arithmétique 1-mot F4-F7 (base 0xF400, masque 0xFC60/0xFCE0). f520 = miroir src=A/dst=B d'une op F48x-class ; f500 = `ADD A,0,B` (B=B+A, SPRU172C:15660-15663, base 0xF400 masque 0xFC60). PAS un repeat (le vrai RPT court est 0xEC00, hi8 map L137 ; RPT #lk 2-mots = F070).
- **Émulateur** : `RPT #k`. Bloc F5xx (L6717-6719 `/* F5xx: SSBX or RPT #k */`) : échec test SSBX (≠0xF5B0) → chute dans le fallback RPT-#k : rpt_count=op&0xFF, rpt_active=true, pc+=1. f520→RPT #0x20, f500→RPT #0x00. Le bloc arith F4 est gardé `if(hi8==0xF4)`, donc les miroirs F5/F6/F7 ne l'atteignent jamais.
- **Impact** : **dataflow/mode-repeat uniquement, stack-neutre** (1 mot, ni push ni pop dans les deux décodes). Réfuté comme hunt-relevant : (a) aucun effet SP ; (b) **pas sur le chemin** — grep '75fd'/'75ff'/'f520'/'f500' = 0 hit dans qemu.log ; atteignabilité comme frontière d'instruction non prouvée (ces mots suivent des F0xx 2-mots dont le comportement de branche peut les sauter) ; (c) mis-armerait au pire un RPT mono-instruction, pas une borne RPTB. Confirmé aussi par l'audit antérieur (C54X_DECODER_AUDIT.md:210 « RPT #k fallback for other F5xx is fabricated — SUSP/M »).
- **Fix proposé** (hygiène) : hisser le bloc arith F4-family (gardé `hi8==0xF4`) pour couvrir aussi F5/F6/F7 en matchant sur base/masque famille (F400/FC60, bits 9-8=src/dst, bits 4-0=SHIFT signé) AVANT le fallback RPT-#k (bloc F5xx L6717-6719). Vérifier sur le chemin exécuté avant tout changement.

### 3.4 — 0x76f8 épilogue (0x7706 POPM ST1 + 0x7707 RETD) : le sur-pop — VERDICT NON-DÉCODEUR

- **ISA** : POPM ST1 (pop1, 1 mot, hi8 map L97) ; RETD UNC (pop1, near retardé 2-slots, hi8 map L156). Total pop de la paire = 2 mots. Épilogue de restauration de contexte structurellement valide UNIQUEMENT entré avec ST1 préalablement empilé.
- **Émulateur** : décode les DEUX ops 100% correctement. POPM (L8484) : mmr=op&0x7F=0x07=ST1, pop1. RETD (L7057-7112) : cc=0x00 UNC, pop1, delayed_pc=ra, delay_slots=2. AUCUNE erreur de décode/longueur/direction/arithmétique de retour.
- **Impact** : **−1 mot net/passe** (2 pops vs le seul push du CC@0xb418). Trace : net_words = −1, −12, −16, −21, −39, −55… ; SP grimpe 0x10ff→0x1144 puis STM-collapse. Preuve runtime : qemu.log L195 (POPM ST1 ← 0xb41a = retour CC PC+2) ; L196 (ORPHAN-RETURN ret_tgt=0x7712 net_words=−1).
- **Verdict** : PAS un bug décodeur. Racine = **entrée par mauvais chemin** : 0x76f8 est l'épilogue de restauration d'une routine de contexte/ISR, censée être entrée après un prologue PSHM ST1 + une trame d'appelant plus profonde. Le CC@0xb418 (premier op de pile cold-boot) l'atteint SANS ce prologue → le 2e pop tire un slot vierge. Vérification mot-par-mot de TOUT le chemin exécuté : aucun PSHM ST1 (0x4A07) ; seul 0x4A06 (PSHM ST0) @0x770d, branche disjointe.
- **Fix proposé** : adresser le ROUTAGE D'ENTRÉE / contrôle amont — pourquoi le contrôle atteint l'épilogue POPM-ST1 de 0x76f8 sans le prologue PSHM-ST1 de l'appelant prévu. Aucun handler du décodeur dans cette région n'est à corriger.

## 4. Mismatches réfutés / non-causaux

| addr | raw | décode ISA vs ému | pourquoi réfuté |
|------|-----|-------------------|-----------------|
| 0xb41f | f820 | BC NTC vs heuristique ACC≠0 (handler F8 L6212) | La description « heuristique ACC≠0 plate / revert dialecte » est PÉRIMÉE : un FIX 2026-06-23 (L6237-6252) utilise le vrai NTC/TC basé-TC quand l'op précédente est cmpm/bitf (0x60xx/0x61xx, mask 0xFE00) et ne retombe sur l'heuristique ACC que sinon. ZÉRO effet pile (BC ne push jamais) ; off-path (poll handshake ARM atteint seulement après retour propre de 0x76f8, qui n'arrive jamais) ; PC b41f jamais exécuté dans la trace. |
| 0x75f3 | 1882 | AND vs LD | dataflow seul, stack-neutre ; PC 0x75e8/0x75f3 jamais atteint (tracer L4642 = 0 hit) ; direction d'alimentation backward jamais empruntée. |
| 0x75fd | f520 | arith F5 vs RPT #0x20 | stack-neutre ; 0 hit dans la trace ; mis-armerait au pire un RPT mono-insn, pas une borne RPTB. |
| 0x75ff | f500 | ADD A,0,B vs RPT #0x00 | stack-neutre ; 0 hit ; RPT #0 dégénéré sans effet SP/branche. |
| 0x75fe | f661 | claim « B=A move » | **claim FAUX** : l'ému décode correctement SFTA A,1,B via L5367 (op&0xFCE0==0xF460) ; le code B=A à L6401 est mort (early-return à 5367 le précède). Aucun mismatch réel. |
| 0x7718 | e721 | (narration « STM #lk,SP ») | fw[0x7718]=0xe721 = op groupe E7 1-mot sans écriture SP ; le STM collapsant est un 0x77xx (MMR=0x18) à un autre PC. Correction de narration, pas un bug. |
| 0x770a/0x771e | f074 | CALL near (firmware) | push1 confirmé par SP-RING, balancé par RETD firmware (0x7791) ; non-source d'imbalance. PC+2/+4 non vérifié → suivi §6, mais n'affecte que la cible de retour pas le SP balance. |

## 5. Analyse de l'équilibre de pile de 0x76f8

**Timeline push/pop (une passe), SP base = 0x1100 :**

1. `0xb418 CC 0x76f8 UNC` → **PUSH 1** (PC+2=0xb41a) → SP 0x1100→0x10ff. [qemu.log g_spring : `pc=0xb418 op=0xf900 PUSH1 SP→0x10ff`]
2. `0x7702 CALLD 0x75e8` → **PUSH 1** (PC+4=0x7706) → SP 0x10ff→0x10fe.
3. callee 0x75e8…0x7600 : RCD LEQ @0x75e8 NON pris (A>0 avec fix SQURA) ; corps exécuté ; `0x7600 RETD UNC` → **POP 1** (récupère 0x7706) → SP 0x10fe→0x10ff. **(CALLD/RETD self-balancé +1/−1)**
4. retour à 0x7706 : `POPM ST1` → **POP 1** (récupère 0xb41a = le retour du CC) → SP 0x10ff→0x1100. [qemu.log L195 : ST1 ← 0xb41a]
5. `0x7707 RETD UNC` → **POP 1** (slot VIERGE au-dessus de SP base, ret_tgt=0x7712 1ère passe) → SP 0x1100→0x1101. [qemu.log L196 : ORPHAN-RETURN net_words=−1]

**Bilan** : pushes = 2 (CC + CALLD) ; pops = 3 (RETD callee + POPM ST1 + RETD épilogue). **Net = −1 mot/passe.** Le CALLD/RETD callee s'annule ; il reste 1 push (CC) contre 2 pops (POPM ST1 + RETD épilogue) → **−1**. SP dérive +1/passe : 0x10ff → … → 0x1144, puis le STM #lk,SP écrit une valeur corrompue (0x1144→0x0079) → DSP déraillé → d_fb_det jamais armé.

**Verdict : seed-de-reset / routage, PAS décodeur.** Chaque instruction décode correctement (re-vérifié contre SPRU172C + hi8 map + trace). L'épilogue 0x7706/0x7707 est une queue de restauration de contexte « return-2-levels-up » : sur silicium elle n'est atteinte qu'après qu'un prologue ISR/dispatcher ait fait PSHM ST1 et qu'une trame d'appelant existe en dessous. **Aucun autre appelant ne pré-seed la pile sur le chemin observé** : le CC@0xb418 cold-boot est le tout premier op de pile après reset, atteint SANS prologue. Le 2e pop tire donc un slot vierge au-dessus de la base du CC. **Empiriquement** : avec le fix SQURA actif (A=0x000fc92f44>0), le corps de la callee tourne jusqu'au RETD@0x7600 — pourtant le sur-pop survient quand même à insn=164, prouvant qu'aucun PSHM ST1 n'existe dans ce corps non plus. Le correctif doit empêcher l'entrée nue dans 0x76f8 (ou fournir le prologue manquant), pas toucher le décodeur.

## 6. Famille pile/contrôle (CC/CALL/RET/RC/RETE/PSHM/POPM/FCALAD)

| opcode (site) | encodage ISA | mots poussés/poppés attendus | ce que fait l'ému | PC+4 vs PC+2 | verdict |
|---------------|--------------|------------------------------|-------------------|--------------|---------|
| CC 0xf900 (0xb418) | F900/0xFF00, cc byte 0x00=UNC | push 1 = PC+2 (non-retardé) | push1 PC+2=0xb41a, SP-- (L6824-6875) | PC+2 ✓ | **correct** |
| CC 0xf945 (0x771c) | F900, cc=0x45=AEQ | push 1 = PC+2 si pris | near CC push PC+2 si AEQ (L6824) | PC+2 ✓ | **correct** (brief « PC+4 » ne vaut que pour CCD retardé) |
| CALLD 0xf274 (0x7702) | F274 exact, retardé | push 1 = **PC+4** | push1 PC+4=0x7706, delay 2 (L6015-6027) | **PC+4 ✓** | **correct** (SPRU172C:16654 + Example2) |
| RCD LEQ 0xfe47 (0x75e8) | FExx cc=0x47=ALEQ, retardé | pop 1 si A≤0 | pop1 si cond, delay 2 (L7057-7112) | n/a (pop RA) | **correct** |
| RETD UNC 0xfe00 (0x7600) | FExx cc=0x00, retardé | pop 1 | pop1, delay 2 (L7057) | n/a | **correct** |
| POPM ST1 0x8a07 (0x7706) | 0x8A00/0xFF00, MMR=0x07 | pop 1 | pop1 mmr=ST1 (L8484) | n/a | **correct décode** (site du sur-pop) |
| RETD UNC 0xfe00 (0x7707) | FExx cc=0x00, retardé | pop 1 | pop1, ret_tgt vierge (L7057) | n/a | **correct décode** (sur-pop) |
| FRET 0xf4e4 (0x770c) | F4E4 exact | **pop 2** (PC+XPC) | pop2 (L4923-4948) | n/a | **correct** (amplifie runaway une fois déraillé) |
| PSHM ST0 0x4a06 (0x770d) | 0x4A00, MMR=0x06 | push 1 | push1 mmr=ST0 (L8182) | n/a | **correct** (≠ PSHM ST1 attendu) |
| CALL 0xf074 (0x770a/0x771e) | F0xx legacy (PAS F274 CALLD ni F273 BD) | push 1 (firmware near CALL) | push1 (L5678+), balancé par RETD@0x7791 | **PC+2/+4 NON vérifié** | low-confidence (équilibre trace OK ; arithmétique de retour à auditer) |

**PSHM ST1 (0x4A07) attendu : ABSENT du chemin exécuté.** C'est le cœur du sur-pop — le POPM ST1 @0x7706 n'a pas de prologue correspondant.

## 7. Recommandations ordonnées

1. **(Décisif, falsifiable) Tracer le contrôle amont vers 0x76f8.** Instrumenter pourquoi le CC@0xb418 (premier op de pile cold-boot) entre dans l'épilogue POPM-ST1 de 0x76f8 sans prologue PSHM-ST1. Hypothèse testable : sur silicium, 0x76f8 n'est PAS l'entrée prévue du CC@0xb418 — soit le pmad du CC est mal-fetché, soit l'alignement de la program-page à 0xb416-0xb419 est décalé. **Falsifiable** : si l'on corrige l'alignement et que le CC cible une autre routine (avec prologue PSHM ST1), le net_words doit rester 0.

2. **(Pré-requis au #1) Résoudre l'alignement 0xb416.** Comparer DATA-table page vs PROGRAM page pour le bloc 0xb410, et vérifier byte-par-byte depuis reset (0xff80→B 0xb410) que les frontières 0x69f8/0x76f8/0x75f8 sont byte-alignées avec le silicium réel. **Falsifiable** : si 0xb416 EST réellement PORTW 3-mots sur silicium, le CC@0xb418 ne peut pas se déclencher tel quel → le boot atteint 0x76f8 par une autre voie et le bug change de nature. NE PAS ajouter de handler PORTW 0x75 avant cette résolution (compensation porteuse actuelle).

3. **(Si #1/#2 confirment l'entrée nue est légitime) Fournir/restaurer le prologue PSHM ST1.** Vérifier si une routine d'init ISR/dispatcher censée précéder 0x76f8 est sautée (cf. le 0x4A06 PSHM ST0 @0x770d sur branche disjointe — chercher un 0x4A07 jamais atteint).

4. **(Hygiène décodeur, hors hunt, basse priorité)** Une fois la pile stabilisée : implémenter sub 4-7 (AND/OR/XOR/SUBC) du case 0x1 (L7627) ; hisser le bloc arith F4-family pour couvrir F5/F6/F7 avant le fallback RPT-#k (L6589-6593) ; vérifier l'arithmétique de retour PC+2/+4 du 0xF074 dans le handler legacy F0xx (L5678+). Chacun à valider sur le chemin exécuté avant changement.

## 8. Couverture

**Plages décodées intégralement (mot-par-mot)** : reset 0xff80-0xff81 ; boot 0xb410-0xb41f ; fonction 0x76f8 prologue + boucle SQURA 0x76f8-0x7707 ; callee CALLD 0x75e8-0x7602 ; épilogue + continuation 0x7706-0x7723. Toutes les instructions sur le chemin exécuté (reset→boot→callee→épilogue→continuation→sites de leak→collapse) sont décodées.

**NON décodé / hors périmètre** :
- Le site exact du `STM #lk,SP` collapsant (opcode 0x77xx MMR=0x18) n'est pas localisé à une adresse firmware précise dans les régions assignées — il est atteint via le runaway SP, à un PC différent de 0x7718 (= e721). À localiser pour une couverture complète du collapse.
- Mots data de la jump-table 0x7603+ (0000 0008 0011 0019 0021 0028 0030 0037…) : données, pas du code (le RCD UNC @0x7600 retourne avant). Aucun effet pile.
- Arithmétique de retour PC+2 vs PC+4 du 0xF074 (handler legacy F0xx) : non confirmée indépendamment.
- Tension d'alignement 0xb416 : irrésoluble depuis les seules entrées fournies (DATA vs PROGRAM page non recoupées exhaustivement).

**Comptes totaux** :
- **18 mismatches soulevés** au total sur les 4 régions (incl. les paires/corrections de narration).
- **1 confirmé** comme mismatch décodeur réel sur le chemin exécuté avec impact contrôle : **0xb416** (PORTW→STL, longueur 3w/2w gate le CC@0xb418) — verdict **UNCERTAIN** (mismatch réel mais décode « faux » porteur/compensatoire ; alignement non résolu).
- **Le sur-pop 0x7706/0x7707 (HIGH) est confirmé comme RÉEL dans le firmware mais RÉFUTÉ comme bug-décodeur** : décode 100% fidèle ; cause = entrée par mauvais chemin (routage/seed amont).
- Tous les autres (0xb41f, 0x75f3, 0x75fd, 0x75fe, 0x75ff, 0x7718, 0x770a/0x771e…) : **réfutés** comme non-causaux pour d_fb_det=0 (stack-neutres et/ou hors chemin exécuté prouvé, ou claim erronée pour 0x75fe).
---

## 9. ADDENDUM (vérifié post-workflow) — la racine : PORTW 0x75 sous-décodé d'1 mot ⇒ CC@0xb418 fantôme

Le §3.1 laissait 0xb416 en **UNCERTAIN**. SPRU172C tranche explicitement (def. PORTW, ligne 1769 + corps p.4-130) :

> **PORTW Smem, PA — Words: 2.** *« Add 1 word when using long-offset indirect addressing or **absolute addressing** with an Smem. »*

`0xb416 = 75f8 000e f900` : le low-byte `0xf8` = **adressage absolu** `*(lk)` sur le Smem → PORTW y fait **3 mots** (`0xb416, 0xb417, 0xb418`). 

**Preuve runtime de la dérive d'1 mot** (qemu.log) : l'ému décode le `0x76` (`st`, abs aussi) adjacent en **3 mots** (insn2 `PC=b413` → insn3 `PC=b416`, soit +3) mais le `0x75` en **2 mots** (insn3 `PC=b416` → insn4 `PC=b418`, soit +2). 
**Preuve décodeur** : aucun handler 0x75/PORTW dans calypso_c54x.c — `0x75f8` retombe sur un store 2-mots de type STL (hi8 map L83 : `0x75 portw 0x7500/0xFF00` ; SPRU « +1 mot si abs »).

### Conséquence — chaîne causale complète
1. L'ému sous-consomme `0xb416` d'1 mot (2 au lieu de 3).
2. Le mot `f900` @0xb418 — qui est en réalité l'**opérande PA/abs de la PORTW** — est re-fetché comme un **opcode CC autonome** (`f900` = CC UNC).
3. Ce **CC fantôme** pousse un retour et saute sur `0x76f8` (le mot `76f8` @0xb419, lui aussi un opérande du flux PORTW/ST réel, lu comme pmad de saut).
4. `0x76f8` est l'épilogue de restauration de contexte (POPM ST1@0x7706 + RETD@0x7707) **entré sans son prologue PSHM ST1** ⇒ sur-pop −1 (le RETD tire un slot vierge).
5. SP dérive +1/passe → `STM #lk,SP` effondre SP → déraille → `d_fb_det=0`.

Le **vrai** boot (alignement correct) ne fait JAMAIS `CC 0x76f8` : après la PORTW (0xb416, 3 mots) il continue à `0xb419 = 76f8 0fff 0001` (ST #1,*(0xfff)), puis `0xb41c = 60f8 0fff` … une séquence de config de ports. La routine SQURA 0x76f8 serait atteinte plus tard par sa voie correcte (avec prologue PSHM ST1). La signature « la cible du CC = le mot d'instruction suivant lu comme pmad » est le marqueur classique d'un **glissement d'alignement d'1 mot**.

### Statut

> ⚠️ **PÉRIMÉ (audit 2026-07-01).** Ce statut « kill-shot à valider » est dépassé : le kill-shot **a atterri et N'A PAS débloqué d_fb_det**. Voir la correction ci-dessous.

- ~~**CONFIRMÉ sur papier** … Reclasse le §3.1 de UNCERTAIN → CONFIRMED (bug décodeur, longueur), et le §1 (« pas un bug décodeur ») est corrigé : c'EST un bug décodeur, en amont, qui fabrique le faux point d'entrée.~~ — **RÉFUTÉ comme cause opérante.** Le mismatch de longueur PORTW était réel, mais le corriger ne débloque rien : voir le point ci-dessous.
- ✅ **FIXÉ (`calypso_c54x.c:7501`, revival 2026-06-23) — MAIS SANS EFFET SUR d_fb_det.** Le handler `if ((op & 0xFF00) == 0x7500)` PORTW 0x75 3-mots (Smem abs) **est câblé et inconditionnel** dans le tree (consomme opcode+PA+lk quand Smem est absolu, donc 0xb416/0x75f8 ne glisse plus ; le CC fantôme @0xb418 ne se produit plus). Le PORTR 0x74 3-mots existe aussi (@7511, gardé `CALYPSO_FIX_PORTR`). **Résultat empirique (vérité-terrain session) : d_fb_det reste 0, le DSP déraille toujours (POST-BOOTSTUB-RET, PC=0x0000), l'IMR reste 0x0000.** La chaîne « PORTW→CC fantôme→sur-pop→collapse » est donc RÉFUTÉE comme root cause opérante — le fix kill-shot de ce doc a landé et n'a rien débloqué.
- **VRAIE racine terminale (post-fix PORTW)** : le **handshake go-live ARM→DSP n'est jamais asserté** — `api_write_cb` est déclaré (`calypso_c54x.h`) mais **JAMAIS assigné** (`grep 'api_write_cb *=' = 0`) ; l'ARM n'écrit que 0x0000 vers les API 0x0314/0x0318 ; l'**IMR n'est jamais ré-armé** après le clear boot @0xb37e (reste 0x0000 sur tout le run) ; le lever `CALYPSO_DSP_FRAME_VEC28` est un **cul-de-sac** (force-vectorise mais atterrit dans l'épilogue ISR → déraille vers le boot-stub, IMR reste 0). Il n'y a **ni `calypso_orch.c` ni bus UDP 6920/6921**. Le correctif appartient au câblage `api_write_cb` + ré-armement IMR, PAS au décodeur PORTW (déjà corrigé).
- **Caveat historique (conservé)** : (a) la sémantique exacte de PORTW (lecture Smem → écriture port PA, vs le store-mémoire que l'ému fait aujourd'hui) reste marquée TODO dans le handler @7503 ; (b) 128 `75f8` + 25 `74f8` (abs) dans le firmware ; (c) l'ordre des 3 mots (opcode, PA, abs-addr) est celui codé au handler.
