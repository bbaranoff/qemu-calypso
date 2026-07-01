# DSP Register-Image (calypso_dsp.Registers.bin) — Review / Diag pour refactor

> ⚠️ **PÉRIMÉ sur sa prémisse centrale (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).**
> Ce doc suppose que `dsp-registers` **n'existe pas** comme propriété et que le
> `.Registers.bin` est **produit mais jamais lu**. C'est désormais **FAUX** : la
> propriété existe (`calypso_mb.c:53`, `REG_DSP_SECTION("dsp-registers", …)`
> `calypso_mb.c:341`), un loader existe (`c54x_load_registers` `calypso_c54x.c:13212`,
> appelé depuis `calypso_trx.c:2438`), et **le mode « bin » (override VERBATIM
> depuis le .bin) est le mode par DÉFAUT** — via un sélecteur 3 modes
> (`c54x`/`bin`/`hybrid`, `calypso_c54x.c:13360-13416`) postérieur à ce doc.
> De plus le bug AR0 (P1 ci-dessous) est déjà **corrigé** (AR0=0x5AAD, `calypso_c54x.c:13301`).
> Corrections inline ci-dessous ; les numéros de ligne ont été réactualisés.

_Pour c web. Host /home/nirvana/qemu-src — 2026-05-31 (numéros de ligne réactualisés 2026-07-01)._

## TL;DR (corrigé après lecture de c54x_reset)

**MISE À JOUR 2026-07-01 :** le fichier `calypso_dsp.Registers.bin` **EST
désormais chargé** — la propriété `-M calypso,dsp-registers=<path>` existe, le
loader `c54x_load_registers()` la lit dans `reg_init[]` avant reset, et
`c54x_reset()` l'applique **VERBATIM** (mode « bin » = défaut). Les mêmes valeurs
restent **aussi hardcodées** dans `c54x_reset()` (mode « c54x »), plus un mode
« hybrid ». Donc le débat n'est PAS « charger vs zapper » : les 3 modes coexistent
via le sélecteur `CALYPSO_REG_MODE`. Le vrai sujet résiduel = les problèmes que
cette duplication révèle (Frankenstein reset, PC absent). Le drift AR0 (P1) est
**corrigé**.

---

## 1. Les deux copies de la même donnée

### (a) Le fichier — généré puis ignoré
- `txt2bin` découpe `calypso_dsp.txt` → section **Registers [0x0000..0x005f] = 96 mots** → `calypso_dsp.Registers.bin`.
- `run.sh` passe à `-M calypso,...` : `dsp-prom0..3`, `dsp-drom`, `dsp-pdrom`.
  ~~**`dsp-registers` n'existe pas** comme propriété~~ — **FAUX (2026-07-01)** : la
  propriété `dsp-registers` existe (`calypso_mb.c:53`), enregistrée par
  `REG_DSP_SECTION("dsp-registers", …)` (`calypso_mb.c:341`), avec help
  « Path to Registers .bin (MMR snapshot …) → applied as the DSP reset state ».
  Wiring : `calypso_mb.c:47-53` (prom/drom/pdrom **+ dsp_registers**), passée à
  `calypso_trx_set_registers_path()` (`calypso_mb.c:99`), chargée par
  `c54x_load_registers()` (`calypso_trx.c:2438`, juste avant `c54x_reset()`).
  ⚠ Note : `calypso_trx.c:1611-1638` n'est plus le chargement ROM mais
  `calypso_tdma_tick()` (timer TDMA).
- ~~Donc : fichier produit, jamais lu.~~ — **FAUX** : fichier produit **ET lu**
  (mode « bin » = défaut). Si `dsp-registers` est fourni, le snapshot override
  le hardcode.

### (b) Le code — `c54x_reset()` (`calypso_c54x.c:13240-…`, ligne réactualisées 2026-07-01)
```c
s->ar[0]=0x5AAD; ar[1]=0x005F; ar[2]=0x0813; ar[3]=0x0014; ar[4]=0x0003; ar[5]=0x0014; // 13301-13311  (AR0 corrigé 0xFF75→0x5AAD)
s->sp   = 0x1100; s->bk = 0xFFF6;     // 13313  (commentaire : 3 dumps 3311/3416/3606 + local)
s->st0  = 0x181F;                     // 13330  DP=0x01F
s->st1  = ST1_INTM|ST1_SXM|ST1_XF;    // 13331  = 0x2900
s->pmst = 0xFFA8;                     // 13332  IPTR=0x1FF, MP_MC=1, OVLY=1, DROM=1
s->imr  = 0x52FD;                     // 13333
s->ifr  = 0;                          // 13342
s->pc   = iptr * 0x80;                // 13501  = 0xFF80
```
Ce bloc = **mode « c54x »**. Un sélecteur (`calypso_c54x.c:13360-13416`) applique
ensuite, en mode « bin » (défaut) ou « hybrid », les valeurs du `.Registers.bin`.

## 2. Le contenu du dump vs ce que QEMU pose vs reset datasheet

```
MMR   addr  dump(.bin)  QEMU reset   datasheet   note
IMR   0x00  0x52fd      0x52FD       0x0000      QEMU = dump (PAS datasheet)
IFR   0x01  0x0008      0x0000       0x0000      ⚠ DRIFT : QEMU=0, dump=0x0008
ST0   0x06  0x181f      0x181F       0x1800      QEMU = dump (DP=0x1f, PAS reset)
ST1   0x07  0x2900      0x2900       0x2900      identiques (coincidence)
TRN   0x0f  0xff75      0 (neutre)   —           snapshot 0xff75 en mode bin ; c54x/hybrid = 0
AR0   0x10  0x5aad      0x5AAD       indef       ✅ FIXÉ (2026-05-31) : QEMU AR0 = 0x5aad (silicium) ; l'ancien 0xFF75 (=TRN) supprimé
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

### P1 — Duplication source + drift ✅ FIXÉ (`calypso_c54x.c:13301`, 2026-05-31)
La même donnée vit en 2 endroits (fichier + littéraux C). Le drift AR0 est
**résolu** ; le drift IFR est **assumé** (choix explicite) :
- `AR0` : ~~QEMU=0xFF75 (= la valeur TRN/0x0f du dump)~~ → **corrigé en 0x5AAD**
  (valeur silicium AR0/0x10), commentaire code : « PROUVÉ read-before-write à
  insn=1 (PC=0xb410) → AR0 reset est load-bearing ; l'ancien 0xFF75 faisait
  diverger c54x vs bin dès la 1ʳᵉ instruction ». L'off-by-one supposé est donc
  tranché et corrigé. (En mode hybrid, AR0 reste forcé à 0xFF75 par choix.)
- `IFR` : QEMU=0, dump=0x0008 — **volontaire** (bit3 INT3 pending, masqué par
  INTM=1 ; mode bin applique 0x0008, hybrid le remet à 0 « datasheet pur »).

### P2 — Reset = Frankenstein (snapshot operating + PC de reset)
Les MMR posés sont un **snapshot de milieu d'exécution** (IMR démasqué, PMST
OVLY=1/DROM=1, ST0 DP=0x1f, AR pleins) — PAS l'état reset silicium. Mais le
**PC** est mis à `0xFF80` (vecteur reset), pas à l'endroit du firmware d'où ce
snapshot provient. Registres-de-milieu + PC-de-début = état incohérent.

**Conséquence directe, déjà dans le code :** un hack de redirect existe pour
rattraper (`calypso_c54x.c:10581`, désormais **gaté par env** `CALYPSO_REDIR_LEGACY`,
OFF par défaut) :
```c
if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) { ... s->pc = 0x7120; }  // SILICON-BOOT-REDIRECT
```
Cible par défaut `s->pc = 0x7120` (`calypso_c54x.c:10608`), avec variantes gatées
env : `0xc704` (`CALYPSO_INITTAB`, ligne 10606) et `0x7000` (`CALYPSO_REDIR7000`,
ligne 10607). Le narratif over-pop est en commentaire ~10575-10608 : sans redirect,
`SP=0x1100 → over-pop boot → self-CALA → spirale → FB jamais locké`. C'est
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
  10581 devient alors une vraie modélisation de ce ROM, pas un patch SP=0x1100.
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
1. ~~AR0=0xFF75 dans c54x_reset : transcription off-by-one ?~~ **RÉSOLU
   (2026-05-31)** : AR0=0x5AAD (valeur silicium, `calypso_c54x.c:13301`). Question close.
2. ~~IFR : QEMU=0 vs dump=0x0008 — volontaire ou oubli ?~~ **RÉSOLU** : volontaire
   (INT3 pending masqué par INTM=1 ; mode bin l'applique, hybrid le remet à 0).
3. Option A : a-t-on assez de doc sur le mask-ROM Calypso pour le modéliser, ou
   le redirect 0x7120 empirique est-il tout ce qu'on aura ?
4. PMST reset : si on passe Option A, le firmware repose-t-il bien OVLY=1/DROM=1
   lui-même tôt, ou faut-il les garder au reset pour que l'overlay code marche ?
