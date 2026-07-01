# Step 3 — F2xx unimpl + 0x79xx fallback stub

## Symptômes

```
[DBG_F2] F2 PC=0xc001 op=0xf210 ...
[DBG_UNIMPL] UNIMPL @0xc001: 0xf210 (hi8=0xf2) [#41624]

[DBG_UNIMPL] UNIMPL @0xfffc: 0x7981 (hi8=0x79) [#41628]

[c54x] F5xx unhandled: 0xf58e PC=0x79a8
```

État actuel :
- **F5xx unhandled** : déjà géré comme 1-word NOP avec trace (`calypso_c54x.c:1942-1949`). RAS.
- **F2xx 0xf210** : le tracer existe (lignes 1771-1795) mais après il fait `goto unimpl;` (ligne 1796) → tombe en runaway.
- **0x7981 à 0xfffc** : `hi8=0x79` non géré dans la dispatch case 7. Au PC=0xfffc qui est dans la zone de vecteurs d'interruption (juste avant le wrap), c'est probablement une donnée garbage interprétée comme code. Stub 1-word.

## Fix

1. F2xx tracer : remplacer `goto unimpl` par `return consumed + s->lk_used`
   (advance comme NOP). Le tracer reste, mais le DSP ne wedge plus.
2. 0x79xx (hi8=0x79) : ajouter un stub 1-word avec trace dans le case
   0x6/0x7 du switch, comme on l'a fait pour 0x88.

## Impact attendu

- Plus de runaway sur F2xx 0xf210 → DSP avance.
- 0xfffc walk-through proprement (au lieu de runaway → NOP slide).
- Combinés aux Steps 1+2, le FB-det handler devrait converger vers
  une vraie sortie au lieu de zigzaguer dans des zones garbage.
