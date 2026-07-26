# Audit décodeur c54x — 2026-05-08 night

Audit systématique post-fixes-#1-#2 (0x76 ST + faux LMS F2/F3) demandé
par Claude web : "1-2h max, mécanique, grep chaque `if (hi8 == 0x??)`
et confronter à binutils tic54x-opc.c."

Résultat après ~30 min de pass : **au moins 11 bugs supplémentaires
identifiés**. Audit non-exhaustif (focus sur 0x80-0x9F + 0xE4-0xE7).
Le reste à faire (0xC0-0xDF parallel ST, 0xF0-0xFF F-class, 0xA0-0xBF
MAC family) reste à couvrir.

## Format

| op | binutils ground truth | notre handler | gravité |
|----|----------------------|----------------|---------|

## Bugs identifiés

| op | binutils | notre handler | gravité |
|----|----------|---------------|---------|
| **0x76** | ST #lk, Smem (2-3 mots) | LDM MMR,dst (1 mot) | ✅ FIXÉ session |
| **0xF2/0xF3 (sauf F272/3/4)** | unmapped + F3 dispatch (SFTL/AND/OR/XOR/INTR) | faux LMS Xmem,Ymem | ✅ FIXÉ session |
| 0x80 | STL src, Smem (1 mot) | **stubbed NOP** ("ancienne classification MVDD 2-mot, neutralisé") | **CRITIQUE** — silently drops stores |
| 0x8C | ST T, Smem (1 mot) | MVPD pmad,Smem (2 mots, prog→data) | **CRITIQUE** — MVPD est à 0x7C |
| 0x8E | CMPS Smem,dst (1 mot, bit test) | MVDP Smem,pmad (2 mots, data→prog) | **CRITIQUE** — MVDP est à 0x7D |
| 0x8F | CMPS Smem,dst (1 mot, bit test) | PORTR PA,Smem (2 mots) | **CRITIQUE** — PORTR est à 0x7400 |
| 0x94/0x95 | LD Xmem,SHFT,dst (1 mot, load avec shift) | MVDK / MVKD (2 mots, data move) | HAUTE |
| 0x96 | BIT Xmem,BITC (1 mot, set TC bit-test) | MVDP Smem,pmad (2 mots, prog write) | **CRITIQUE** — fait des prog_write fantômes |
| **0x98/0x99** | STL src, SHFT, Xmem (1 mot, store low) | déclaré STH, écrit `(acc>>16)` (high) | **CRITIQUE** — STL/STH **swap**, écrit le mauvais demi-acc |
| **0x9A/0x9B** | STH src, SHFT, Xmem (1 mot, store high) | déclaré STL, écrit `(acc&0xFFFF)` (low) | **CRITIQUE** — STL/STH **swap** symétrique |
| 0xE4 | parallel ST `OP_SRC,OP_Ymem` (mask 0xFC00 FL_PAR) | BITF Smem,#lk (2 mots) | HAUTE — BITF est à 0x6100 |

## Bugs secondaires possibles non vérifiés

- 0xE5 MVDD : correct (vérifié)
- 0xE6 ST parallel : non vérifié notre handler
- 0xE7 MVMM : correct (vérifié)
- 0xC0-0xDF : binutils dit tout ST parallel, ranges multiples — non audité
- 0xF0-0xFF : énorme range (ADD/AND/B/CALL/LD/MAC/etc.), partiellement audité
- 0xA0-0xBF : MAC family (mac/macsu/macr/mas/masr) — non audité
- 0x70-0x7B : MVKD/MVDK/MVDM/MVMD/PORTR/PORTW/MACP/MACD — non audité

## Priorités si on applique des fixes

**Si firmware actif les utilise (à vérifier par PC HIST grep) :**
1. 0x80 stub NOP → STL src, Smem (1 mot) — déblocage si STL src=A version utilisée
2. 0x98/0x99 ↔ 0x9A/0x9B swap — fix symétrique (juste échanger les blocs ou inverser le `>> 16`)
3. 0x8C / 0x8E / 0x8F : refaire complètement les handlers (CMPS bit test ≠ MVPD/MVDP/PORTR)
4. 0x96 : refaire (BIT 1-mot, set TC) au lieu de MVDP fantôme
5. 0x94/0x95 : refaire (LD avec shift) au lieu de MVDK/MVKD

## Mise à jour 2026-05-08 night : Tier A appliqué + audit étendu

Per directive Claude web : Tier A = LMS (déjà) + 0x98/9A swap + 0x80 STL +
0x8C ST T. **3 fixes additionnels appliqués** (commits inclus dans md5
`9f5ffe5c`).

Audit poursuivi sur 0x70-0x7B, 0xA0-0xBF, 0xC0-0xDF. **Bugs additionnels
catalogués (Tier B)** :

| op | binutils | notre handler | gravité |
|----|----------|---------------|---------|
| 0x81 | STL src,Smem (1w) bit8=src | utilise `s->a` toujours, ignore bit 8 | MOYENNE — STL B fait écrire A.low |
| 0x82 | STH src,Smem (1w no shift) | applique ASM shift (faux) | MOYENNE — STH avec shift fantôme |
| 0x83 | STH src,Smem (1w) | WRITA Smem (totalement faux) | HAUTE |
| 0x84 | STL src,ASM,Smem (1w) | READA Smem (faux) | HAUTE |
| 0x85 | STL src,ASM,Smem (1w) | MVPD pmad,Smem (2w faux) | HAUTE |
| 0x86 | STH src,ASM,Smem (1w) | MVDM dmad,MMR (2w faux) | HAUTE |
| 0x87 | STH src,ASM,Smem (1w) | MVMD MMR,dmad (2w faux) | HAUTE |
| 0x8B | POPD Smem (1w) | stubbed NOP | MOYENNE — pop dropped |
| 0x8E | CMPS A,Smem (1w, set TC) | MVDP Smem,pmad (2w faux) | HAUTE |
| 0x8F | CMPS B,Smem (1w, set TC) | PORTR PA,Smem (2w faux) | HAUTE |
| 0x91 | ADD #lk,SHFT,src,dst (2w) | MVKD dmad,Smem (2w faux) | HAUTE |
| 0x70..0x75 | MVKD/MVDK/MVDM/MVMD/PORTR/PORTW (2w each) | **AUCUN handler** → unimpl | DÉPEND USAGE |
| 0x78..0x7B | MACP/MACD (2w) | aucun handler | DÉPEND USAGE |
| 0x7C/0x7D | MVPD/MVDP (2w) | aucun handler — vrais MVPD/MVDP unimpl | DÉPEND USAGE |
| 0xA0 | ADD Xmem,Ymem,DST (1w 3 ops) | sub-dispatch LD/NEG/ABS/NOT/SAT/SFT | **CASCADE RISK** |
| 0xA1 | ADD Xmem,Ymem,DST (1w) | AND #lk,16,src ? | CASCADE RISK |
| 0xC0..0xDF | ST||... parallel (mask 0xFC00) | handlers spécifiques par 0xCx/0xDx | NEED DEEP AUDIT |

**Total bugs catalogués session 2026-05-08 night : 24** (5 fixés Tier A,
19 restants Tier B).

## Limite atteinte

`0xA0xx` en particulier est profondément différent entre notre handler et
binutils : changer ça impacte tout le hot path MAC. Risque "compensation
mutuelle" trop élevé pour fixer sans validation runtime des Tier A.

**Audit Tier B suspendu** jusqu'à validation post-rebuild des fixes Tier A.

## Recommandation pour la session

Claude web a dit "pas de patch, instrumentation d'abord" pour les bugs
runtime (cascade IMR). Mais ces bugs-ci sont **statiquement vérifiés
contre binutils** : pas besoin de runtime pour confirmer qu'ils sont
faux. Toutefois :

- **Risque** : appliquer 8 fixes d'un coup expose tous les paths cachés.
  L'effet observable du fix #1 (0x76) avait déjà déplacé le blocker.
  Le fix #2 (faux LMS) n'a pas encore été testé.
- **Stratégie conseillée** : valider les 2 fixes appliqués au prochain
  rebuild (signaux #1-#8 listés dans le report précédent), puis appliquer
  ces 8 nouveaux fixes en bloc, puis re-tester.
- **Alternative agressive** : appliquer tout dans un même build (le rebuild
  prend du temps, mutualiser). Risque de régression silencieuse si plusieurs
  bugs étaient en compensation mutuelle.

## Méthode

Reproductible via :

```bash
# Liste des hi8 dispatchés dans le code
grep -nE 'if \(hi8 == 0x[0-9A-Fa-f]+\)' \
    hw/arm/calypso/calypso_c54x.c

# Référence binutils
grep -E '"\w+",.*0x[0-9A-Fa-f]{4}, 0xFF00' \
    /home/nirvana/gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c

# Pour chaque hi8, comparer la meaning du commentaire de notre handler
# avec le mnémonique binutils. Mismatch → bug.
```

L'audit devrait être ré-exécuté périodiquement (pré-merge, pré-release)
pour rattraper toute régression.

## Question pour Claude web

> Audit demandé : 11 bugs trouvés en ~30 min de grep+source-read.
> 8 sont CRITIQUES (changent l'effet observable du firmware) :
> 0x80 stub NOP, 0x8C/0x8E/0x8F mauvais opcodes, 0x94-0x96 idem,
> 0x98-0x9B STL/STH swap.
>
> Stratégie demandée :
> (a) wait-and-see — rebuild avec les 2 fixes session, valider signaux,
>     puis batcher les 8 audit-fixes
> (b) all-in — appliquer les 8 audit-fixes maintenant et rebuild une fois
>     pour les 10 bugs total
> (c) priorisation — fix seulement les 3-4 plus probables d'être hit par
>     le firmware actuellement (0x80, 0x98/9A swap, 0x8C ST T) puis re-test
>
> Quel est ton avis ? Aussi : tu vois quelque chose qui justifierait que
> les bugs 0x80/0x9x soient des compensations volontaires (genre firmware
> spécifique qui dépend de la mauvaise sémantique parce qu'historiquement
> ça avait été testé comme ça) ?
