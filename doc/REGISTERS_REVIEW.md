# DSP Register-Image (calypso_dsp.Registers.bin) — Review / Diag pour refactor

_Pour c web. Host /home/nirvana/qemu-src — 2026-05-31._

## TL;DR (corrigé après lecture de c54x_reset)

Le fichier `calypso_dsp.Registers.bin` **n'est pas chargé** par run.sh — MAIS les
mêmes valeurs sont **déjà hardcodées en dur** dans `c54x_reset()`. Donc le débat
n'est PAS « charger vs zapper » : les registres sont déjà dans l'émulateur. Le vrai
sujet = **3 problèmes** que cette duplication révèle (drift, Frankenstein reset, PC absent).

---

## 1. Les deux copies de la même donnée

### (a) Le fichier — généré puis ignoré
- `txt2bin` découpe `calypso_dsp.txt` → section **Registers [0x0000..0x005f] = 96 mots** → `calypso_dsp.Registers.bin`.
- `run.sh` passe à `-M calypso,...` : `dsp-prom0..3`, `dsp-drom`, `dsp-pdrom`.
  **`dsp-registers` n'existe pas** comme propriété. Wiring : `calypso_mb.c:47-52, 316-327` (seules prom/drom/pdrom enregistrées), chargement `calypso_trx.c:1611-1638`.
- Donc : fichier produit, jamais lu. **Silence non documenté.**

### (b) Le code — `c54x_reset()` (`calypso_c54x.c:11615-11763`)
```c
s->sp   = 0x1100; s->bk = 0xFFF6;     // 11637  (commentaire : 3 dumps 3311/3416/3606 + local)
s->st0  = 0x181F;                     // 11650  DP=0x01F
s->st1  = ST1_INTM|ST1_SXM|ST1_XF;    // 11651  = 0x2900
s->pmst = 0xFFA8;                     // 11652  IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1
s->imr  = 0x52FD;                     // 11653
s->ifr  = 0;                          // 11662
s->ar[0]=0xFF75; ar[1]=0x005F; ar[2]=0x0813; ar[3]=0x0014; ar[4]=0x0003; ar[5]=0x0014; // 11629-35
s->pc   = iptr * 0x80;                // 11740  = 0xFF80
```

## 2. Le contenu du dump vs ce que QEMU pose vs reset datasheet

```
MMR   addr  dump(.bin)  QEMU reset   datasheet   note
IMR   0x00  0x52fd      0x52FD       0x0000      QEMU = dump (PAS datasheet)
IFR   0x01  0x0008      0x0000       0x0000      ⚠ DRIFT : QEMU=0, dump=0x0008
ST0   0x06  0x181f      0x181F       0x1800      QEMU = dump (DP=0x1f, PAS reset)
ST1   0x07  0x2900      0x2900       0x2900      identiques (coincidence)
TRN   0x0f  0xff75      —            —           voir AR0 ci-dessous
AR0   0x10  0x5aad      0xFF75       indef       ⚠ DRIFT : QEMU AR0 = la valeur TRN(0x0f) du dump, pas AR0(0x10)=0x5aad
AR1   0x11  0x005f      0x005F       indef       ok
AR2   0x12  0x0813      0x0813       indef       ok
AR3   0x13  0x0014      0x0014       indef       ok
AR4   0x14  0x0003      0x0003       indef       ok
AR5   0x15  0x0014      0x0014       indef       ok
AR6   0x16  0xbae6      0 (memset)   indef       QEMU zéro
AR7   0x17  0x1e44      0 (memset)   indef       QEMU zéro
SP    0x18  0x1100      0x1100       indef       QEMU = dump
BK    0x19  0xfff6      0xFFF6       indef       ok
BRC   0x1a  0x8fd7      0            indef       QEMU zéro (état block-repeat transitoire)
RSA   0x1b  0xd9ec      0            indef       QEMU zéro
REA   0x1c  0xbbef      0            indef       QEMU zéro
PMST  0x1d  0xffa8      0xFFA8       ~0xff80     QEMU = dump (OVLY=1/DROM=1, PAS reset)
XPC   0x1e  0x0000      0            0           ok
periph 0x22=0x0800 0x24=0x0211 0x25=0xffff 0x28=0x7fff 0x29=0xf802 : NON répliqués
0x5c..5f = 3236 3236 3240 3200 (ASCII '62 62 @2 2') : dump device réel, NON répliqué
```

## 3. Les 3 vrais problèmes

### P1 — Duplication source + drift déjà constaté
La même donnée vit en 2 endroits (fichier + littéraux C), sans lien. Elles ont
**déjà divergé** :
- `IFR` : QEMU=0, dump=0x0008.
- `AR0` : QEMU=0xFF75 (= la valeur **TRN/0x0f** du dump), alors que l'AR0 réel
  (0x10) du dump = 0x5aad. **Suspicion d'off-by-one de transcription** (ou
  valeur prise d'un autre des 3 dumps). À trancher : c'est un bug latent.

### P2 — Reset = Frankenstein (snapshot operating + PC de reset)
Les MMR posés sont un **snapshot de milieu d'exécution** (IMR démasqué, PMST
OVLY=1/DROM=1, ST0 DP=0x1f, AR pleins) — PAS l'état reset silicium. Mais le
**PC** est mis à `0xFF80` (vecteur reset), pas à l'endroit du firmware d'où ce
snapshot provient. Registres-de-milieu + PC-de-début = état incohérent.

**Conséquence directe, déjà dans le code :** un hack de redirect est nécessaire
pour rattraper (`calypso_c54x.c:9705`):
```c
if (s->pc == 0xFF80 && s->sp == 0x1100) { ... s->pc = 0x7120; }  // SILICON-BOOT-REDIRECT
```
Commentaire 9689-9692 : sans redirect, `SP=0x1100 → over-pop boot → self-CALA
0x70c3 → spirale → PMST 0x70C4 → TOA=28868 → FB jamais locké`. C'est
**exactement** le « court-circuit du boot » du snapshot-as-reset.

### P3 — Le .Registers.bin ne peut PAS être un point de resume fidèle
Le PC du C54x **n'est pas memory-mapped** dans 0x00-0x5F. Le fichier n'a donc
**aucune info de PC**. Impossible de « reprendre » depuis ce dump sans deviner le
PC. C'est la limite de fond : ce fichier seul est insuffisant comme état complet.

## 4. Refactor — options (à challenger c web)

Le point user (« ne pas zapper un fichier ROM ») est juste, mais la cible n'est
pas « charger le .bin ». Trois directions cohérentes, à choisir :

- **Option A — Reset datasheet honnête + modéliser le mask-ROM manquant.**
  Poser les VRAIES reset values (ST0=0x1800, PMST=IPTR seul/0xFF80, IMR=0,
  IFR=0, SP indéfini), PC=0xFF80, puis **émuler le boot mask-ROM Calypso**
  (absent du dump PROM) qui pose SP=0x5AC8 et saute au firmware. Le redirect
  9705 devient alors une vraie modélisation de ce ROM, pas un patch SP=0x1100.
  → supprime P2 et P3. Le .Registers.bin sert d'**oracle** : le boot doit
  converger vers PMST=0xffa8, DROM=1, SP=0x5AC8.

- **Option B — Assumer le snapshot, mais cohérent.** Charger TOUS les MMR depuis
  le .bin (single source of truth, supprime P1) ET fixer le PC au point firmware
  correspondant (à déterminer, car absent du .bin → P3 reste un problème). Plus
  fragile.

- **Option C — Garder le hardcode actuel mais (i) le dériver/checker du .bin au
  build (anti-drift, règle P1), (ii) documenter pourquoi run.sh ne charge pas le
  fichier, (iii) garder le redirect comme « mask-ROM model » documenté.**
  Minimal, mais ne résout pas le Frankenstein de fond.

**Reco CC** : A est la seule qui supprime le hack. Mais d'abord trancher P1
(le drift AR0/IFR est un bug net, indépendant du choix d'option).

## 5. Questions pour c web
1. AR0=0xFF75 dans c54x_reset : transcription off-by-one depuis TRN(0x0f), ou
   valeur volontaire d'un autre dump (3311/3416/3606) ? cf doc/datasheets/README.md §3-4.
2. IFR : QEMU=0 vs dump=0x0008 (INT3 pending) — volontaire (on ne veut pas
   d'IT pending au reset) ou oubli ?
3. Option A : a-t-on assez de doc sur le mask-ROM Calypso pour le modéliser, ou
   le redirect 0x7120 empirique est-il tout ce qu'on aura ?
4. PMST reset : si on passe Option A, le firmware repose-t-il bien OVLY=1/DROM=1
   lui-même tôt, ou faut-il les garder au reset pour que l'overlay code marche ?
