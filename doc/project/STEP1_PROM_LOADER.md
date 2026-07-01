# Step 1 — PROM loader / READA → 0x0000

## Symptôme

```
[c54x] READA: prog[0x5717]=0x0000 → data[0x0000] PC=0x7629 rpt=12 insn=7342847
[c54x] READA: prog[0x5718]=0x0000 → data[0x0001] PC=0x7629 rpt=11 ...
... 13 mots tous à 0x0000 ...
```

À PC=0x7629 (FB-det handler), un `RPT #12 ; READA Smem` copie 13 mots de
`prog[0x5717..0x5723]` vers `data[0x0000..0x000c]`. **data[0..0x1F] = MMRs**
(IMR, IFR, ARs, etc.) → après cette copie, IMR=0, IFR=0, AR0..AR4=0 →
**toutes interrupts masquées** → DSP perd le frame IRQ → wedge.

## Cause analysée

`calypso_c54x.c:4248-4324` (`c54x_load_rom`) :

| Section dump   | Plage word addr | Loaded into | Contenu              |
|----------------|-----------------|-------------|----------------------|
| Registers      | 0x00000-0x0005F | `s->data`   | regs init            |
| DROM           | 0x09000-0x0DFFF | `s->data`   | données ROM          |
| PDROM          | 0x0E000-0x0FFFF | `s->data`   | program-data ROM     |
| PROM0          | 0x07000-0x0DFFF | `s->prog`   | code ROM page 0      |
| PROM1          | 0x18000-0x1FFFF | `s->prog`   | code ROM page 1      |
| PROM2          | 0x28000-0x2FFFF | `s->prog`   | code ROM page 2      |
| PROM3          | 0x38000-0x39FFF | `s->prog`   | code ROM page 3      |

**Problème** : `prog[0x5717]` est **en dessous** de PROM0 (`0x7000`). Aucune
section ne charge cette plage dans `s->prog`. Sur vrai C54x, l'OVLY bit
mappe DARAM `[0x0000..0x3FFF]` dans le program space, et la zone
`[0x4000..0x6FFF]` peut être DROM/PDROM mirroré vers prog via DROM enable.

Notre loader **ne mirror pas** DROM/PDROM/DARAM dans `s->prog`, donc tout
read de `prog[<0x7000]` retourne 0.

## Vérification

Le dump contient bien des données dans `[0x5717..0x5723]` (visibles dans
les valeurs PROM2 line `0ae20 : ac6a 3ec3 6839 bf70 35bd 7434 b66f 37bb`),
mais ces valeurs sont stockées dans `s->prog[0x18000+...]` (PROM1 ou
PROM2/3), pas dans `s->prog[0x5717]`.

## Solution

Mirror DROM (et possiblement PDROM) dans `s->prog` pour que les READA
qui lisent les coefs ROM trouvent les bonnes valeurs.

## Impact attendu

- READA récupère les bons coefficients (table de twiddles, masques, etc.)
- MMRs ne sont plus zéroïsés
- IMR reste correctement initialisé → INT3 (frame IRQ) servi
- Pas de wedge dans la zone NOP de PROM
