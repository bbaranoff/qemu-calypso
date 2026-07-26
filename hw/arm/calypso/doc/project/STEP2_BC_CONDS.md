# Step 2 — BC group 1 conditions

> ✅ **FIXÉ (`calypso_c54x.c:6308-6337`, FIX 2026-06-23).** Le décodage des
> conditions BC groupe 1 (acc-test) décrit ci-dessous comme « à faire » est
> **déjà implémenté**. Ce document est conservé comme historique ; ne pas
> re-proposer le patch. Voir aussi [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md).
>
> Rappel vérité-terrain : ce fix corrige bien le décodage des branchements,
> mais il **ne débloque pas** la détection FB — `d_fb_det` reste 0 et le DSP
> déraille par ailleurs.

## Symptôme (historique — comportement AVANT le fix)

Trace au runtime :
```
[c54x] BC unknown cond=0x42 PC=0x79a6 op=0xf842 → FALL (default)
[c54x] BC unknown cond=0x4d PC=0x79c7 op=0xf84d → FALL (default)
[c54x] BC unknown cond=0x44 PC=0xc003 op=0xf844 → FALL (default)
[c54x] BC unknown cond=0x45 PC=0xc010 op=0xf845 → FALL (default)
```

~~Le dispatch BC `0xF8xx` ne gère que `0x00` (UNC), `0x20` (NTC), `0x30`
(TC). Toutes les autres conditions tombent dans `take = false` (default
fall-through). Branchements ratés systématiquement.~~ — **CORRIGÉ** : le
dispatch décode désormais les conditions groupe 1 (voir `calypso_c54x.c`
lignes 6308-6337). Les opcodes `F842`/`F844`/`F845`/`F84D` ont
`sub = (op>>4)&0xF = 0x4` et sont traités par la branche `if (sub == 0x4 ||
sub == 0x5)`.

## Encodage tic54x

Per `binutils/symbols.c condition_codes[]` :

| Symbole | Valeur | Sens                                  |
|---------|--------|---------------------------------------|
| CC1     | 0x40   | Marker group 1 (acc test)             |
| CCB     | 0x08   | Sélecteur B (sinon A)                 |
| CCEQ    | 0x05   | acc == 0                              |
| CCNEQ   | 0x04   | acc != 0                              |
| CCLT    | 0x03   | acc < 0                               |
| CCLEQ   | 0x07   | acc <= 0                              |
| CCGT    | 0x06   | acc > 0                               |
| CCGEQ   | 0x02   | acc >= 0                              |
| CCOV    | 0x70   | overflow                              |
| CCNOV   | 0x60   | no overflow                           |
| CCTC    | 0x30   | TC bit set                            |
| CCNTC   | 0x20   | TC bit cleared                        |
| CCC     | 0x0C   | carry set                             |
| CCNC    | 0x08   | carry cleared                         |

Conditions group 1 = `CC1 | [CCB] | CCxx`. Donc les valeurs `0x42..0x4F`
et `0x4A..0x4F` (avec CCB) sont :

- `0x42` = `CC1 | CCGEQ` → `A >= 0`
- `0x43` = `CC1 | CCLT`  → `A < 0`
- `0x44` = `CC1 | CCNEQ` → `A != 0`
- `0x45` = `CC1 | CCEQ`  → `A == 0`
- `0x46` = `CC1 | CCGT`  → `A > 0`
- `0x47` = `CC1 | CCLEQ` → `A <= 0`
- `0x4A..0x4F` = même chose mais B (avec `CCB=0x08`)
- `0x4D` = `CC1 | CCB | CCEQ` → `B == 0`

## Fix — ✅ FIXÉ (`calypso_c54x.c:6308-6337`, FIX 2026-06-23)

~~Ajouter le décodage group 1 dans le `switch (cond)` du dispatch BC.
Sélection acc selon `CCB`, comparaison selon les bits 0..2 (CCxx).~~

**DÉJÀ FAIT.** Implémenté tel que décrit :

- Sélection acc selon `CCB` : `int64_t acc = (cc & 0x08) ? s->b : s->a;`
  (`calypso_c54x.c:6312`).
- Comparaison selon les bits 0..2 (CCxx) : `switch (cc & 0x07)` avec les cas
  `0x2 AGEQ / 0x3 ALT / 0x4 ANEQ / 0x5 AEQ / 0x6 AGT / 0x7 ALEQ`
  (`calypso_c54x.c:6314-6321`).

Ne pas re-proposer ce patch.

## Impact (constaté)

- BC pmad,A>=0 / A<0 / A==0 / A!=0 / B==0 etc. branchent désormais
  correctement (décodage acc-test correct).
- Les chemins de branchement testant l'accu après corrélation prennent la
  bonne branche. **MAIS** cela ne suffit pas à débloquer la détection FB :
  vérité-terrain, `d_fb_det` reste 0 et le DSP déraille (le vrai verrou est
  le handshake go-live ARM→DSP, `api_write_cb` jamais câblé — voir
  [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).
