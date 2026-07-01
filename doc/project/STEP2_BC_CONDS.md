# Step 2 — BC group 1 conditions

## Symptôme

Trace au runtime :
```
[c54x] BC unknown cond=0x42 PC=0x79a6 op=0xf842 → FALL (default)
[c54x] BC unknown cond=0x4d PC=0x79c7 op=0xf84d → FALL (default)
[c54x] BC unknown cond=0x44 PC=0xc003 op=0xf844 → FALL (default)
[c54x] BC unknown cond=0x45 PC=0xc010 op=0xf845 → FALL (default)
```

Le dispatch BC `0xF8xx` ne gère que `0x00` (UNC), `0x20` (NTC), `0x30`
(TC). Toutes les autres conditions tombent dans `take = false` (default
fall-through). Branchements ratés systématiquement.

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

## Fix

Ajouter le décodage group 1 dans le `switch (cond)` du dispatch BC.
Sélection acc selon `CCB`, comparaison selon les bits 0..2 (CCxx).

## Impact attendu

- BC pmad,A>=0 / A<0 / A==0 / A!=0 / B==0 etc. branchent correctement.
- Les chemins d'erreur du FB-det handler (qui testent l'accu après
  une corrélation) prennent enfin la bonne branche.
