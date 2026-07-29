# PLAN D'APPLICATION UNIFIÉ — verrou natif DSP (BRINT0 / IT trame / opcodes)

Date : 2026-07-28. Fusion de quatre audits (réfutation opcodes 0x60–0x8F, réfutation
opcodes 0xC0–0xFF, axe « séquence IMR », axe « vecteur 21 », axe « chemin ARM/BSP »)
en **une seule** liste ordonnée d'actions.

Statut de chaque affirmation : **MESURE** (log / dump binaire), **LECTURE DE CODE**
(fichier:ligne), **HYPOTHÈSE** (falsifiable, test indiqué).

Autorités, dans l'ordre :
1. `${QEMU_TREE}/hw/arm/calypso/doc/opcodes/tic54x-opc.c` (champ MOTS fait foi)
2. `${QEMU_TREE}/hw/arm/calypso/doc/spru172c.pdf` (extrait `spru.txt`) — sémantique C54x
3. TI SPRU131G (extrait `/root/.claude/jobs/26578783/tmp/spru131g.txt`) — périphériques,
   IMR/IFR, table des vecteurs, init BSP
4. les images ROM DSP (`calypso_dsp.PROM0..3.bin`, `PDROM.bin`, `DROM.bin`)
5. le code émulateur, puis les tableaux de synthèse du projet

**Aucun fichier source n'est modifié par ce document.** Les extraits de patch sont
des propositions, non appliquées.

---

## 0. Résumé exécutif

Le verrou natif mesuré n'est **ni** l'IMR bit 5, **ni** un « vecteur 21 installé à
zéro ». Les deux formulations reposent sur une **erreur de nomenclature** propagée
depuis `calypso_c54x.h:117-124` : sur C54x, `BRINT0 = IMR bit 4 = vecteur 20`, et
`bit 5 = BXINT0` (émission). Le vecteur 21 **n'est pas vide** : il contient un stub
`RETE ; NOP`, exactement ce que le firmware met sur une source qu'il n'utilise pas.

Le maillon manquant est **une génération d'IT en amont** : l'IT trame TPU→DSP.
Le ROM arme inconditionnellement `IMR |= 0x3000` (bits 12/13 → vecteurs 28/29) à
`PROM0 0xa4c7`, et le vecteur 28 porte un vrai handler qui appelle la tâche trame
`0xa4e4`. Notre modèle, lui, émet l'IT trame sur `vec 19 / bit 3` — un vecteur qui
est un stub `RETE` **et** un bit que le ROM ne met jamais à 1 — et la condition
d'émission se teste elle-même sur ce bit 3 : elle est donc **auto-fausse**, l'IT
n'est pas même levée. Tant que ça dure, la tâche trame n'est jamais exécutée, donc
le dispatcheur de commandes n'est jamais appelé, donc le ROM n'arme jamais BRINT0
(bit 4), donc le corrélateur n'est jamais atteint. `CALYPSO_WATCH_9F00_RD = 0` est
la conséquence, pas la cause.

Croisement avec l'audit d'opcodes : `0x6881` / `0x6981` sont **correctement décodés**.
L'audit d'opcodes **n'explique pas** le verrou BRINT0 — les deux enquêtes restent
distinctes. Une seule exception, réelle et à traiter : `BANZ *ARx(lk)` (MOD ≥ 0xC)
est mal évalué, et un site (`PROM0 0xde5a`) est **dans la boucle native mesurée**.

---

## 1. VERDICT BRINT0

### 1.1 Le vecteur 21 n'est pas vide — c'est un stub d'acquittement

**MESURE (dump binaire, indépendante de tout log).** La table des vecteurs est
copiée en DARAM `0x0080` par la boucle `READA` de `PROM0 0xb4d0/0xb4d6`
(`b4d0: 7712 0080  STM #0x0080,AR2` ; `b4d6: 7e92  READA *AR2+`). Sa **source** se
trouve dans `calypso_dsp.PDROM.bin`, index mot `0x0321`, 4 mots par vecteur.
Extrait exact (relu ce jour) :

```
vec 16  0361 : f4eb f495 0000 0000     RETE ; NOP                 (INT0,  bit 0)
vec 17  0365 : 731e 3fcf f880 7092     MVMD XPC ; FB 0x7092       (INT1,  bit 1)
vec 18  0369 : f4eb f495 0000 0000     RETE ; NOP                 (INT2,  bit 2)
vec 19  036d : f4eb f495 0000 0000     RETE ; NOP                 (TINT,  bit 3)
vec 20  0371 : 731e 3fd0 f880 72d3     MVMD XPC ; FB 0x72d3       (BRINT0,bit 4)  ***
vec 21  0375 : f4eb f495 0000 0000     RETE ; NOP                 (BXINT0,bit 5)
vec 22  0379 : 731e 3fd0 f880 72ed     handler                    (TRINT, bit 6)
vec 23  037d : f4eb f495 0000 0000     RETE ; NOP                 (TXINT, bit 7)
vec 24  0381 : 731e 3fd0 f880 72fa     handler                    (INT3,  bit 8)
vec 25  0385 : 731e 3fcf f880 70b8     handler                    (HPINT, bit 9)
vec 26  0389 : f4eb f495 0000 0000     RETE ; NOP                 (BRINT1,bit10)
vec 27  038d : 731e 3fcf f880 7263     handler                    (BXINT1,bit11)
vec 28  0391 : 731e 3fcf f880 7234     handler                    (frame, bit12)  ***
vec 29  0395 : f4eb f495 0000 0000     RETE ; NOP                 (       bit13)
vec 30  0399 : 731e 3fcf f880 0158     handler                    (       bit14)
vec 31  039d : f4eb f495 0000 0000     RETE ; NOP                 (       bit15)
```

Décodage par binutils : `0xF4EB = rete` (`tic54x-opc.c:399`, 1 mot),
`0xF495 = nop` (`:377`), `0x7300 = mvmd MMR,dmad` (`:373`, champ MMR 0x1E = XPC),
`0xF880 = fb xpmad` (`:238`, 2 mots, FL_FAR).

⇒ **Le vecteur 21 n'est PAS « installé à zéro »**. Il est installé, délibérément,
comme un stub qui acquitte et rend la main — le comportement voulu pour une source
inutilisée. Il n'y a **rien à corriger** sur le vecteur 21, et rien à espérer de lui.

**MESURE de confirmation (run `MXOwop`)** : avec `IMR=0x70fd` (bit 5 armé
artificiellement), la sonde compte **100 `DISPATCH(normal)` du vec 21 sur 100
levées** — et produit `CORR-ENTRY=1`, `9f00=0`. Démasquer et vectoriser le bit 5
ne produit **rien**. C'est mesuré, pas déduit.

### 1.2 BRINT0 = bit 4 / vecteur 20 — quatre preuves indépendantes

**a) SPRU131G, table des vecteurs C548** (`spru131g.txt:10755-10790`, relu ce jour) :

```
      19             6       TINT/SINT3       4C         Internal timer interrupt
      20             7       BRINT0/SINT4     50         Buffered serial port receive interrupt
      21             8       BXINT0/SINT5     54         Buffered serial port transmit interrupt
      22             9       TRINT/SINT6      58         TDM serial port receive interrupt
```
(la 2ᵉ colonne est la **priorité**, pas le bit ; le bit IMR = vec − 16).
Diagrammes IMR/IFR : `spru131g.txt:9722, 9754` — `… BXINT0 BRINT0 TINT INT2 INT1 INT0`,
soit bit 3 = TINT, **bit 4 = BRINT0**, bit 5 = BXINT0.

**b) Balayage exhaustif des écritures IMR du ROM** (`PROM0`, recherche
`STM #lk,IMR`, `ORM/ANDM/XORM/ADDM #lk,*(0x0000)` — refait ce jour) :

```
ARMENTS  : 0xa4c7 ORM #0x3000   0xa509 STM #0x0010   0xa510/ae54/ae6f ORM #0x0040
           0xa518/ae8a ORM #0x0100   0xa62a ORM #0x4000   0xb161 ORM #0x0800
           0xb566 ORM #0x0002   0xc2a7 ORM #0x0140
           0xbd40  ORM #0x0010   0xbd62  ORM #0x0010
           0xc471  ORM #0x0010   0xc498  ORM #0x0010          <-- BRINT0 = bit 4
DESARM.  : 0x70d7/0xb3a6 ANDM #0xc150   0xa66d ANDM #0xbfff
           0xade4/bd22/bd4b/c41d/c44b/c47c ANDM #0xffef        <-- clear bit 4
           0xae19/c2bb ANDM #0xfebf   0xb193/b19d ANDM #0xf7ff
IFR      : 0xaddf/bd35/bd58/c423/c466/c491 STM #0x0010, IFR    <-- W1C du bit 4
```
**Aucune écriture, nulle part, ne pose 0x0020 dans l'IMR.** Le bit 5 n'apparaît que
dans le masque d'effacement de masse `0xC150` (= ~0x3EAF) du boot. Le couple
`0xb3a6 ANDM #0xC150,IMR` / `0xb3a9 STM #0x3EAF,IFR` énonce la partition du firmware
en deux instructions : **bit 4 = à garder ; bit 5 = à désactiver et à purger**.

**c) La séquence d'activation du port série** (`PROM0 0xbd30..0xbd40`, clone en
`0xbd53..0xbd62`, `0xc464..0xc471`, `0xc48f..0xc498`) est mot pour mot
l'« Example 9-4 : BSP Receive Initialization Routine » de SPRU131G
(`spru131g.txt:24868-24895`) :
```
bd33: STM #0x0008, MMR[0x22]   ; BSPC0 : RRST=0 XRST=0 (port en reset)   <- ex.9-4 (1)
bd35: STM #0x0010, IFR         ; W1C du pending BRINT0                    <- ex.9-4 (2)
bd37: STM #0x00c8, MMR[0x22]   ; BSPC0 : RRST=1 XRST=1 (port armé)        <- ex.9-4 (8)
bd3b: ORM #0x0010, *(0x435b)   ; shadow IMR |= BRINT0
bd40: ORM #0x0010, IMR         ; DÉMASQUE BRINT0                          <- ex.9-4 (3)
```
`MMR 0x22 = BSPC0` (SPRU131G table 8-2, `spru131g.txt:18499-18510`). Le bit qu'on
purge puis démasque **à l'instruction qui suit l'activation du port série** est,
par construction, l'interruption de ce port série : c'est le **bit 4**.

**d) La partition handlers/stubs de la table §1.1** : bit 4 (RINT0) et bit 6
(TRINT) = handlers ; bit 5 (XINT0), bit 7 (TXINT), bit 3 (TINT) = stubs. Réceptions
servies, émissions et timer non servis. Cohérent ligne à ligne avec la table TI, et
incohérent avec la table du projet.

### 1.3 Le vrai verrou est **deux crans en amont** : l'IT trame n'est jamais levée

**LECTURE DE CODE, ROM.** Le vecteur 28 (IMR bit 12, armé **inconditionnellement**
par `0xa4c7 ORM #0x3000,IMR`) contient :
```
7234: f074 013b        CALL 0x013b        ; sauvegarde de contexte (overlay 23 PSHM)
7236: 7707 2900        STM  #0x2900, ST1
7238: 7706 1800        STM  #0x1800, ST0
723a: 68f8 001d fffc   ANDM #0xFFFC, PMST
723d: f074 a4e4        CALL 0xa4e4        ; *** LA TÂCHE TRAME ***
723f: f7bb             SSBX INTM
7240: f073 0100        B    0x0100        ; épilogue
```
et le vecteur 20 (BRINT0) :
```
72d3: f074 013b  CALL 0x013b ; ... ; f074 c14a  CALL 0xc14a  ; f073 0108  B 0x0108
```
(`0xc14a` = recopie du tampon reçu : `RPT *(0x0904)`, `RPT *(0x0903)`,
`BITF *(0x3f92),#0x0200` — la cellule TASKW déjà connue du projet).

La chaîne complète du silicium est donc :
```
ARM  dsp_end_scenario()  (osmocom calypso/dsp.c:466-477)
       d_dsp_page = B_GSM_TASK|w_page ; TPU_CTRL |= DSP_EN ; INT_CTRL.ICTRL_DSP_FRAME = 0
  ->  TPU  émet IT_DSP « at the start of the frame »   (Calypso overview p.47)
  ->  DSP  IFR bit 12  &  IMR bit 12 (armé par ROM 0xa4c7)  ->  vec 28  ->  0x7234
  ->        CALL 0xa4e4  = tâche trame
  ->          0xa501 sauvegarde IMR dans d[0x435b] ; 0xa504 BITF #0x0010 ; 0xa509 IMR=0x0010
  ->          0xa51b RSBX INTM ; 0xa51c LD d_dsp_page(0x08d4) ; ... ; 0xa57c CALA d[0x3fd4]
  ->            dispatcheur de commandes 0xc1fa  ->  0xc446  ->  BSPC0/IFR/shadow
  ->            0xc471  ORM #0x0010, IMR   = ARMEMENT DE BRINT0
  ->  BSP/RIF livre les mots I/Q  ->  BRINT0 (bit 4, vec 20, PC 0x00D0)  ->  0xc14a
  ->  démod 0x9f00.. / corrélateur
```

**LECTURE DE CODE, modèle.** `calypso_trx.c:1443-1452` :
```c
bool tpu_armed = !(s->tpu_regs[TPU_INT_CTRL/2] & ICTRL_DSP_FRAME);
static int _natfr = -1; if (_natfr < 0) _natfr = (getenv("CALYPSO_FRAME_IT_NATIVE") ||
                                     getenv("CALYPSO_DSP_FRAME_VEC28")) ? 1 : 0;
bool imr_armed = !!(s->dsp->imr & (1 << (_natfr ? 12 : C54X_INT_FRAME_BIT)));
...
    c54x_interrupt_ex(s->dsp, C54X_INT_FRAME_VEC, C54X_INT_FRAME_BIT);
```
avec `calypso_c54x.h:126-127` :
```c
#define C54X_INT_FRAME_VEC   19  /* INT3 = vec (3+16) */   <-- commentaire faux : vec19 = TINT
#define C54X_INT_FRAME_BIT   3   /* IMR bit 3 */
```
Sans variable d'environnement, `imr_armed = !!(imr & (1<<3))`. **Le ROM ne met
JAMAIS le bit 3 à 1** (§1.2b). La condition est **auto-fausse** : l'IT trame n'est
pas même levée. Et si elle l'était, elle viserait le vec 19 = stub `RETE`.

**MESURE, run natif de 102 s** (`NATIVE_HELPED=0`, ni `FB_CORR_ENTRY` ni `FB_ENERGY`) :
```
IMR-ARM 0x52fd -> 0x0000   PC=0xb37e op=0x7700 insn=2023      (STM #0, IMR — reset firmware)
IMR-ARM 0x0000 -> 0x3000   PC=0xa4c7 op=0x69f8 insn=5370      (ORM #0x3000 — bits 12/13)
IMR-ARM 0x3000 <-> 0x3200  PC=0xde84/0xddf9                   (toggle bit 9, ~10^6 fois)
SYNC-DISPATCH-PROBE vec=21(BRINT0) imr_bit=5 INTM=0 unmasked=0
                    ifr_before=0x0028  -> STAYS-PENDING(masked)
grep -c 'IRQ-LEVEL' -> 0
CALYPSO_WATCH_9F00_RD -> 0
```
`IFR = 0x0028` = bits 3 et 5, **posés par notre modèle**. `IMR = 0x3200` = bits 9,
12, 13, **posés par le ROM**. `pend = IFR & IMR = 0` : les deux ensembles sont
**disjoints**. Ce n'est pas « BRINT0 masquée », c'est **deux nomenclatures qui ne
se croisent nulle part** — zéro vectorisation sur tout le run.

Le PC natif figé dans `0xddf5..0xde88` (ordonnanceur des tâches « background »,
`d[0x098b/098c/098d]`) est **interruptible** : `0xa4cf CALAD` avec `0xa4d0 RSBX INTM`
dans le delay slot ⇒ INTM=0 pendant la tâche. Une IT sur le bit 12 y serait prise
immédiatement. Le livelock background est donc un **symptôme concomitant**, pas la
barrière : la barrière est l'absence d'IT trame.

### 1.4 Verdict

| Formulation | Verdict |
|---|---|
| « le verrou est l'IMR bit 5 (BRINT0 masquée) » | **FAUX** — bit 5 = BXINT0, jamais armé par aucun ROM, et son démasquage forcé a été mesuré sans effet (100 dispatch, 0 corrélateur). |
| « le vecteur 21 est installé à zéro » | **FAUX** — il contient `RETE ; NOP`, stub délibéré. Rien à corriger, rien à en attendre. **Ceci ne change donc PAS la priorité n°1 : cela l'écarte.** |
| « le verrou est un maillon plus en amont » | **VRAI** — l'IT trame TPU→DSP n'est jamais émise (condition auto-fausse sur le bit 3) et viserait de toute façon un stub. Sans elle : pas de tâche trame → pas de dispatcheur → pas d'armement du bit 4 → pas de BRINT0 → pas de corrélateur. |

**Priorité n°1 = câbler l'IT trame sur `vec 28 / IMR bit 12`.**
**Priorité n°2 = déplacer l'IT de réception BSP de `(21,5)` vers `(20,4)`.**

---

## 2. LE CROISEMENT — les opcodes 0x6881 / 0x6981 sont-ils bien décodés ?

### 2.1 Réponse : OUI. Décodage correct, prouvé trois fois.

**a) LECTURE DE CODE — ROM.**
```
PROM0 0xddf7: 7711 0000        STM  #0x0000, AR1        (MMR 0x11 = AR1)
PROM0 0xddf9: 6881 fdff        ANDM #0xFDFF, *AR1       -> data[0x0000] = IMR &= ~0x0200
PROM0 0xde82: 7711 0000        STM  #0x0000, AR1
PROM0 0xde84: 6981 0200        ORM  #0x0200, *AR1       -> IMR |= 0x0200
```
Octet bas `0x81` : bit7=1 (indirect), MOD = `(0x81>>3)&0xF` = 0 (`*ARx`, sans
post-modification), ARF = 1 → `*AR1` ; AR1 = 0 → `data[0x0000]` = IMR.

**b) LECTURE DE CODE — binutils + handler.**
`tic54x-opc.c:263` `{andm, 2, ..., 0x6800, 0xFF00, {OP_lk, OP_Smem}}` — **2 mots**.
`tic54x-opc.c:383` `{orm , 2, ..., 0x6900, 0xFF00, {OP_lk, OP_Smem}}` — **2 mots**.
Handlers `calypso_c54x.c:9353-9369` (relus ce jour) :
```c
if ((op & 0xFF00) == 0x6800) {            /* ANDM #lk, Smem */
    addr = resolve_smem(s, op, &ind);
    uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
    uint16_t v = data_read(s, addr);
    data_write(s, addr, v & lk);
    consumed = 2;  return consumed + s->lk_used;
}
if ((op & 0xFF00) == 0x6900) { ... data_write(s, addr, v | lk); ... }
```
Longueur exacte, masque exact (`0xFF00`), read-modify-write exact, gestion du mot
d'extension Smem exacte. Chaîne de `if` vérifiée : rien en amont n'attrape 0x68xx /
0x69xx (le fallback `(op & 0xF800) == 0x6800` est en aval, `calypso_c54x.c:9499`).
Le routage `data_read/data_write` sur `addr < 0x20` va bien au MMR IMR
(`calypso_c54x.c:2404-2406` en lecture, `:3859 / :3946 s->imr = val` en écriture).

**c) MESURE — la meilleure preuve.** Le log natif montre `IMR` osciller
**exactement** `0x3000 ↔ 0x3200`, écrit par ces deux PC. `0x3200 & 0xFDFF = 0x3000`
et `0x3000 | 0x0200 = 0x3200`. Le décodeur produit, au bit près, ce que le ROM
demande. S'il y avait erreur de longueur ou d'opérande, cette signature serait
impossible.

### 2.2 Conséquence : les deux enquêtes ne fusionnent PAS

L'audit d'opcodes **n'explique pas** le verrou BRINT0. Les valeurs d'IMR mesurées
sont exactement celles que le ROM écrit ; le ROM n'arme simplement jamais le bit 5,
et n'arme le bit 4 que dans une routine qui n'est jamais atteinte. Le verrou est
**un problème de câblage périphérique→vecteur dans le modèle**, pas un problème de
décodage.

### 2.3 Deux points de contact réels (et un seul qui compte)

**(i) `0x1A00 OR` — contact bénin, déjà corrigé.** La séquence de restauration
`PROM0 0xa57e-0xa582` est `LD *(0x435b),A ; OR *(0x0000),A ; STL A,*(0x0000)`
(vérifié binaire : `10f8 435b / 1af8 0000 / 80f8 0000`), soit **`IMR |= d[0x435b]`**.
Avant le correctif du matin, `0x1A00` s'exécutait comme un `LD` → `A = IMR` →
`IMR = IMR` : inoffensif. Après correctif : `IMR |= d[0x435b]`, correct.
⚠️ Ceci **réfute** la note projet `golive-imr-shadow-435b` (« 0xa582 écrit IMR=0 et
écrase l'arm 0xa4c7 ») : `0xa582` est un OR, jamais destructeur. Le seul écrivain
destructeur est `0xa509 STM #0x0010, IMR`, et il est intentionnel (c'est pour cela
que `0xa501` sauvegarde d'abord).

**(ii) `BANZ *ARx(lk)` MOD ≥ 0xC — SEUL finding d'opcode sur le chemin natif mesuré.**
`PROM0 0xde5a : 6ce6 0001 de46` = `BANZ 0xde46, *AR6(+1)` (octet bas `0xe6` :
bit7=1, MOD = `(0xe6>>3)&0xF` = 0xC, ARF = 6). **Ce PC est dans la boucle native
mesurée `0xddf5..0xde86`.** Il est précédé de `0xde59 : 6d8e = MAR *AR6-`, idiome
`MAR *ARx- ; BANZ loop,*ARx(+1)` qui n'a de sens que si la valeur testée est
l'**adresse effective**, pas `ARx` brut. Le modèle teste `ARx` brut. Voir correctif
**C4**.

---

## 3. LISTE ORDONNÉE DES CORRECTIFS

Critères d'ordre, appliqués dans cet ordre : **(i)** sur le chemin du verrou
mesuré ? **(ii)** confirmé par binutils **ET** spru/SPRU131G, ou seulement probable ?
**(iii)** risque de régression sur le mode SHUNT_LEGIT qui campe aujourd'hui ?

Rappel : les diagnostics **D0/D1/D2** de la §5 passent **AVANT** C1 — leur résultat
peut réordonner la suite.

Test de non-régression **obligatoire après chaque correctif**, sans exception :
```
cd ${QEMU_TREE} && CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 \
    CALYPSO_SHUNT_REAL_FB=1 ./start-clean.sh
```
Attendu : **BSIC=7**, **SYSTEM INFORMATION 2 et 4**, **LOCATION UPDATING ACCEPT**.
Toute dégradation de l'un des trois = correctif **reverté immédiatement**, pas
« ajusté ». (La relance de la pile est faite par l'utilisateur ; Claude fournit les
edits et la lecture des logs.)

---

### C1 — IT trame DSP sur `vec 28 / IMR bit 12` (et non `vec 19 / bit 3`)
**(i) sur le chemin du verrou : OUI, c'est LE verrou. (ii) confirmé : OUI
(ROM + SPRU131G). (iii) risque : MOYEN (touche l'ordonnancement natif ; nul pour
SHUNT_LEGIT, cf. garde ci-dessous).**

**Fondement.** ROM `0xa4c7 ORM #0x3000, IMR` arme bits 12/13 inconditionnellement
(MESURE : `IMR 0x0000 -> 0x3000 PC=0xa4c7`). Table des vecteurs (PDROM §1.1) :
vec 28 = `FB 0x7234` = vrai handler → `CALL 0xa4e4` = tâche trame. Vec 19 = stub
`RETE`, bit 3 = TINT jamais armé. `IMR bit b → vecteur b+16` (SPRU131G table 6-24,
`spru131g.txt:10755-10790`) — mapping déjà correct dans le modèle
(`calypso_c54x.c:5004 int vec = b + 16;`).

**Patch minimal** — `hw/arm/calypso/calypso_c54x.h:126-127` :
```c
-#define C54X_INT_FRAME_VEC   19  /* INT3 = vec (3+16) */
-#define C54X_INT_FRAME_BIT   3   /* IMR bit 3 */
+/* IT trame Calypso (TPU IT_DSP). SPRU131G : bit IMR b -> vecteur b+16.
+ * Le ROM arme bits 12/13 en PROM0 0xa4c7 (ORM #0x3000,IMR) et n'arme JAMAIS
+ * le bit 3 (TINT) ; la table de vecteurs (PDROM idx 0x0391) donne
+ * vec 28 -> FB 0x7234 -> CALL 0xa4e4 (tache trame), alors que vec 19
+ * (PDROM idx 0x036d) est un stub RETE/NOP. */
+#define C54X_INT_FRAME_VEC   28
+#define C54X_INT_FRAME_BIT   12
```
Cette seule ligne suffit : `calypso_trx.c:1445` recalcule alors
`imr_armed = !!(imr & (1<<12))` (vrai dès `0xa4c7`) et `:1452` lève `(28,12)`.
Les remaps conditionnels existants deviennent des no-op cohérents
(`calypso_c54x.c:5008-5013`, `:16849-16858`, `calypso_bsp.c:1069/1418`,
`calypso_trx.c:1444`) : **ne pas les supprimer dans le même commit** — les vider
seulement une fois C1 confirmé.

**Garde SHUNT_LEGIT.** `calypso_trx.c:1450` est déjà conditionné par
`!calypso_dsp_shunt_route_c54x_active()` : quand le shunt route vers le c54x, c'est
lui qui fire l'IT trame. Le risque de double-IT est donc déjà couvert. À
**re-vérifier au log** malgré tout (`calypso_bsp.c:1069/1418` partagent `_fb` avec
le shunt).

**Commande de test** (mode natif pur, aucune béquille) :
```
cd ${QEMU_TREE} && CALYPSO_DSP=c54x CALYPSO_SHUNT_LEGIT=0 \
  CALYPSO_INIT_435B_OFF=1 CALYPSO_WATCH_9F00_RD=1 \
  CALYPSO_DEBUG=IRQ,VEC ./start-clean.sh
```

**Résultat ATTENDU** (dans cet ordre, c'est un enchaînement, pas une liste) :
1. `IRQ-LEVEL take bit=12 vec=28 -> PC=0x00f0` apparaît (aujourd'hui : 0 occurrence) ;
2. le PC quitte `0xddf5..0xde88` et atteint `0xa4e4` (la sonde `SM-TRACE`
   `calypso_c54x.c:13683` se met à cracher) ;
3. `IMR-ARM ... -> 0x0010 PC=0xa509` puis, plus tard,
   `IMR-ARM ... -> 0x3010 PC=0xbd40` **ou** `PC=0xc471` — **c'est la signature
   décisive : le ROM arme BRINT0 tout seul** ;
4. des écritures `MMR 0x22` (BSPC0) `0x0008` puis `0x00c8` (visibles seulement avec
   le correctif de sonde C3a).

**Résultat qui l'INVALIDE :**
- `IRQ-LEVEL take bit=12 vec=28` **absent** ⇒ ce n'est pas l'IMR qui bloquait mais
  l'émission : vérifier `tpu_armed` (`INT_CTRL.ICTRL_DSP_FRAME`, polarité inversée,
  bit à 0 = activé) et `calypso_dsp_shunt_route_c54x_active()` ;
- vectorisation présente **mais** le DSP revient immédiatement en `0xddf5` sans
  passer par `0xa4e4` ⇒ le handler `0x7234` déraille (suspect : `CALL 0x013b`,
  overlay de sauvegarde de contexte, 23 `PSHM` — vérifier le SP) ;
- **crash / storm PC=0** ⇒ régression de pile : revoir `calypso_c54x.c:5019-5030`
  (push XPC conditionnel).

**Non-régression** : commande obligatoire ci-dessus. Attendu : inchangé
(BSIC=7 + SI + LU ACCEPT).

---

### C2 — IT de réception BSP sur `(vec 20, bit 4)` et non `(vec 21, bit 5)`
**(i) sur le chemin : OUI, immédiatement après C1. (ii) confirmé : OUI (SPRU131G
table 6-24 + table de vecteurs PDROM + balayage IMR du ROM). (iii) risque : FAIBLE.**

**Fondement.** §1.2 (a,b,c,d). Le vec 21 est un stub `RETE` ; le vec 20 appelle
`0xc14a` = recopie du tampon reçu + handshake `d[0x3f92]`.

**Patch minimal** — 4 sites `calypso_bsp.c` + 1 site `calypso_tpu.c` :
```c
/* SPRU131G table 6-24 : vec 20 = BRINT0/SINT4 @0x50 (BSP receive, IMR bit 4) ;
 * vec 21 = BXINT0/SINT5 @0x54 (transmit) = stub RETE dans ce firmware
 * (PDROM idx 0x0375). Le ROM arme le bit 4 en 0xbd40/0xbd62/0xc471/0xc498 et
 * ne pose JAMAIS 0x0020 dans l'IMR. */
-    if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5))) {
-        c54x_interrupt_ex(bsp.dsp, 21, 5);
+    if (bsp.dsp && !(bsp.dsp->ifr & (1 << 4))) {
+        c54x_interrupt_ex(bsp.dsp, 20, 4);
```
sites : `calypso_bsp.c:1091` (`CALYPSO_BSP_DIRECT_BRINT0`), `:1327` (fenêtre
BDLENA), `:1555`, `:1642` ; `calypso_tpu.c:118` (`TPUI_DSP_INT_PG`).

⚠️ **Sous-produit à traiter dans le MÊME commit** : `calypso_c54x.c:16139` et
`:16163` lèvent TINT0 en `(20,4)` — c'est-à-dire **droit dans l'ISR de réception
série**. Après C2, ces deux sites doivent passer à `(19,3)` (TINT = bit 3, vec 19,
stub — donc sans effet, ce qui est le comportement correct : le firmware n'utilise
pas le timer). Sinon on aura deux sources sur le vecteur 20.

**Commande de test** : identique à C1.

**Résultat ATTENDU** : après l'apparition de `IMR-ARM -> 0x3010 PC=0xbd40|0xc471`,
on voit `IRQ-LEVEL take bit=4 vec=20 -> PC=0x00d0`, puis le PC entre dans
`0xc14a..0xc19c`, puis `CALYPSO_WATCH_9F00_RD` **passe de 0 à non-nul**.

**Résultat qui l'INVALIDE** : `IRQ-LEVEL bit=4` présent mais `9f00` reste 0 ⇒
BRINT0 n'était pas le dernier verrou (cf. D1) ; le maillon suivant est dans
`0xc14a` (buffers `0x3E00`/`0x4000`, `BK=0x015E`) ou dans la chaîne BDLENA (RANK2).

**Non-régression** : commande obligatoire.

---

### C3 — Modéliser BSPC0 (MMR 0x22) et conditionner C2 à `RRST`
**(i) sur le chemin : OUI, complète C2. (ii) confirmé : OUI (SPRU131G table 8-2 +
Ex. 9-4 + 11 écritures ROM). (iii) risque : FAIBLE (aujourd'hui c'est un trou, pas
un comportement).**

**Fondement.** SPRU131G table 8-2 (`spru131g.txt:18499-18510`) : `0x20 BDRR0`,
`0x21 BDXR0`, **`0x22 BSPC0`**, `0x23 BSPCE0`. Bits BSPC : b7 = `RRST`, b6 = `XRST`,
actifs à 1 = hors reset (`spru131g.txt:22442-22457`). Le ROM écrit `0x0008` puis
`0x00C8` en 11 endroits.
**LECTURE DE CODE, modèle** : `calypso_c54x.c:2404` (lecture MMR) et `:3859`
(écriture MMR) sont gardés par `if (addr < 0x20)` ⇒ **0x20..0x23 tombent dans la
DARAM générique** : les 11 `STM #…, BSPC0` du firmware n'ont **aucun effet**, et la
sonde `BOOT-MMR-WR` est **aveugle** à ces adresses. « Pas de log » sur BSPC0 n'est
donc **pas** « pas d'écriture ».

**C3a (sonde seule, à faire d'abord — coût nul, risque nul).** Étendre la fenêtre
de la sonde `BOOT-MMR-WR` à `addr <= 0x26` **sans** changer le routage. Répond à :
le firmware écrit-il `0x0008` puis `0x00C8` en `0x22` ?

**C3b (modélisation).** Ajouter au modèle un état `bspc0` et conditionner C2 :
```c
/* SPRU131G Ex.9-4 : BRINT0 n'est assertee que si le recepteur est hors reset
 * (BSPC0.RRST = bit 7). Sans ca, l'IT de reception est une invention du modele. */
if (bsp.dsp && c54x_bspc0_rrst(bsp.dsp) && !(bsp.dsp->ifr & (1 << 4)))
    c54x_interrupt_ex(bsp.dsp, 20, 4);
```

**Résultat ATTENDU (C3a)** : deux écritures `MMR 0x22` par activation RX
(`0x0008` puis `0x00c8`), aux PC `0xbd33/0xbd37`, `0xbd56/0xbd5a`, `0xc464/0xc468`,
`0xc48f/0xc493`.
**Résultat qui l'INVALIDE** : zéro écriture en `0x22` **après** C1 ⇒ la routine
`0xc446`/`0xbd30` n'est pas atteinte ⇒ le problème est en amont, dans le
dispatcheur de commandes `0xc1fa` (cf. §6, incertitude I2).

**Non-régression** : commande obligatoire (C3a est une sonde : risque nul, mais on
teste quand même, par principe).

---

### C4 — `BANZ`/`BANZD` : valeur testée = adresse effective, pour MOD ≥ 0xC
**(i) sur le chemin : OUI — `PROM0 0xde5a` est DANS la boucle native mesurée.
(ii) confirmé : binutils OUI ; spru.txt PARTIEL (doc contradictoire, tranché par le
firmware). (iii) risque : NUL PAR CONSTRUCTION (restriction MOD ≥ 0xC).**

**Fondement.** `banz = 0x6C00`, `banzd = 0x6E00` (`tic54x-opc.c:268`).
⚠️ La documentation est **contradictoire** : `spru.txt@427801` et SPRU131G
(`spru131g.txt@460364`) disent « If ARx ≠ 0 » — ce qui soutient le code actuel ;
seul l'**Exemple 3** (`BANZ 2000h, *AR3(−1)`, AR3=0001 → PC 1000→1003, AR3
**inchangé**, pas de branche) soutient « adresse effective ». **La doc seule ne
tranche pas.** Ce qui tranche, c'est le firmware :

```
PROM0 d69d: f030 000f   AND  #0x000F, A
      d69f: 8812        STLM A, AR2
      d6a2: 6ce2 ffff d6a9   BANZ d6a9, *AR2(-1)   ; AR2==1 -> STM #0x0008,AR6
      d6a9: 6ce2 fffa d6b0   BANZ d6b0, *AR2(-6)   ; AR2==6 -> STM #0x0800,AR6
      d6b0: 6ce2 fffb d6b7   BANZ d6b7, *AR2(-5)   ; AR2==5 -> STM #0x0400,AR6
      d6b7: 6ce2 fffc d6be   BANZ d6be, *AR2(-4)   ; AR2==4 -> STM #0x0200,AR6
      d6be: 7716 0100        STM #0x0100, AR6      ; defaut
```
Sous la sémantique actuelle (test de `ar[2]` brut), les **quatre** BANZ testent la
**même** valeur : les `lk` −1/−6/−5/−4 n'ont aucun effet, le switch s'effondre en
2 cas. Aucun compilateur TI ne génère ça. Sous « adresse effective », c'est
exactement `switch (AR2) { 1, 6, 5, 4 }`.
Corroboration : idiome `MAR *ARx- ; BANZ loop, *ARx(+1)` en `0xda43`, `0xda62`, et
**`0xde5a`** (vérifié binaire : `de59: 6d8e` puis `de5a: 6ce6 0001 de46`).

**Distribution des modes (4 ROM)** : PROM0 {direct 2, MOD0 23, MOD1 62, MOD8 2,
**MOD12 20**}, PROM1 {MOD1 75, MOD8 2}, PROM2 {direct 1, MOD0 6, MOD1 43, MOD9 2,
**MOD12 64**}, PROM3 {MOD1 2}. **Aucun MOD 3/13/14/15**, et **aucun MOD ≥ 0xC dans
le miroir page-0 `0xE000-0xFFFF`** (la zone que traverse SHUNT_LEGIT).

**Patch minimal** — `calypso_c54x.c`, aux **deux** sites (BANZ et BANZD) :
```c
             int nar = op & 0x07;
             uint16_t pre = s->ar[nar];
-            resolve_smem(s, op, &ind);
+            uint16_t ea  = resolve_smem(s, op, &ind);
+            /* SPRU172C p.4-17 Ex.3 : BANZ 2000h,*AR3(-1) avec AR3=0001 ne branche
+             * PAS et laisse AR3 inchange => la valeur testee est l'ADRESSE
+             * EFFECTIVE, pas ARx brut. Corrobore par PROM0 0xd6a2-0xd6be
+             * (switch AR2 sur 1/6/5/4) et l'idiome MAR *ARx- ; BANZ *ARx(+1)
+             * (0xda43, 0xda62, 0xde5a). Restreint a MOD>=0xC : aucun site
+             * MOD 3/D/E/F dans PROM0..3, les 213 sites MOD 0/1/8/9/direct
+             * restent bit-a-bit identiques. */
+            int mod_banz = (op & 0x80) ? ((op >> 3) & 0x0F) : -1;
+            uint16_t test_val = (mod_banz >= 0xC) ? ea : pre;
             uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
             consumed = 2;
-            if (pre != 0) {
+            if (test_val != 0) {
```
⚠️ **Ne pas** appliquer la variante inconditionnelle du rapport initial : elle
change aussi MOD 0x3 et interagit avec le garde AR2-FLOOR
(`CALYPSO_AR2_FLOOR_DROP`, `calypso_c54x.c:~4606`).

**Commande de test** : sonde ciblée avant/après sur `PC == 0xde5a` loggant `AR6`,
`test_val`, branche prise ; puis run natif §C1.
**Résultat ATTENDU** : la branche `0xde5a → 0xde46` change de comportement ; la
boucle background cesse d'être un cycle à 4 sites. Corollaire attendu ailleurs :
4 sites (`0xc65f`, `0xc96a`, `0xc9d5`, `0xca93`) où un `BANZ skip,*AR1(−1)` garde
un `ORM #masque, *(0x08d5)` (cellule NDB) cessent de publier **à l'inverse**.
**Résultat qui l'INVALIDE** : régression SHUNT_LEGIT (impossible par construction —
si elle survient, c'est que le miroir page-0 contient un MOD ≥ 0xC non détecté :
re-scanner avant de conclure).

**Non-régression** : commande obligatoire. Ce correctif est le seul de l'audit
d'opcodes qui touche le chemin natif mesuré ⇒ il passe **avant** tous les autres
correctifs d'opcodes.

---

### C5 — Retrait des béquilles fondées sur la fausse nomenclature
**(i) sur le chemin : OUI — l'une d'elles (INIT-435B) est ACTIVE PAR DÉFAUT et
polluera les mesures dès que C1 fera atteindre `0xa4e4`. (ii) confirmé : OUI.
(iii) risque : FAIBLE, mais à faire par étapes.**

⚠️ **Point d'ordonnancement critique.** `calypso_c54x.c:13670-13678` (`INIT-435B`,
gate `CALYPSO_INIT_435B_OFF`, **défaut ON**) : à `exec_pc == 0xa4e4`, si
`data[0x435b] == 0`, il écrit `0x52ed` (ou `0x52fd`). Aujourd'hui ce bloc ne tire
jamais **parce que `0xa4e4` n'est jamais atteint**. **Dès que C1 marche, il tirera**
— et `0xa582` fera alors `IMR |= 0x52ed`, armant les bits 0,2,3,5,6,7,9,12,14 que
le firmware n'a jamais demandés. Toute mesure post-C1 serait ininterprétable.

⇒ **Action obligatoire AVANT C1, sans modifier de code : poser
`CALYPSO_INIT_435B_OFF=1` dans l'environnement de tous les runs de mesure.**
Puis, une fois C1 confirmé, supprimer le bloc.

**Fondement du retrait.** `0x52fd` n'existe dans **aucune** des 6 images ROM. C'est
une valeur de reset codée en dur dans le modèle (`calypso_c54x.c:16631
s->imr = 0x52FD;`), réinjectée par `CALYPSO_KEEP_IMR` (`:14266-14280`, valeur par
défaut `0x52fd`) et semée par `INIT-435B`. Le masque réel post-reset est `0`
(`0xb37e STM #0, IMR`) puis `& 0xC150` (`0xb3a6`). `data[0x435b]` est le **shadow
logiciel de l'IMR maintenu par le DSP lui-même** — 6 modifications du bit 4
appliquées deux fois (matériel + shadow) à 1–3 instructions d'écart
(`0xbd3b/0xbd40`, `0xbd5f/0xbd62`, `0xc46c/0xc471`, `0xc495/0xc498`,
`0xade1/0xade4`, `0xc420/0xc41d`). Conversion API RAM : `ARM_off = (0x435b−0x0800)×2
= 0x76B6`, hors des `0x2000` mots d'API RAM (`doc/SHUNT_LEGIT_ADDRESS_MAP.md:40`)
⇒ **l'ARM ne peut pas l'écrire et n'a pas à l'écrire**.

**À retirer, dans cet ordre :**
| ordre | bloc | fichier:ligne | raison |
|---|---|---|---|
| 1 | `INIT-435B` | `calypso_c54x.c:13670-13678` | sème un masque inventé ; **défaut ON** ; polluera post-C1 |
| 2 | `KEEP-IMR` | `calypso_c54x.c:14266-14280` | poursuit le bit 5 que le ROM n'utilise pas |
| 3 | `FIX_BRINT0_UNMASK` | `calypso_c54x.c:16929-16937` | diagnostic ; à retirer après D1 (§5), **jamais à confirmer** |
| 4 | `CALYPSO_BSP_DIRECT_BRINT0` | `calypso_bsp.c:1083-1092` | lève le mauvais bit ; absorbé par C2 |
| 5 | `CALYPSO_ARM2DSP_CTRLSYS` | `calypso_arm2dsp.c:127-135` | écrit `d[0x0810] \|= 0x8000` = `B_TASK_ABORT` (`l1_environment.h:367`) = **ordre d'abandon**. Le commentaire « the real ARM asserts this bit in l1s_reset() » est **faux** : les seules écritures osmocom sont `sync.c:312` (abort) et `dsp.c:483/490` (TSC 3 bits). |
| 6 | `CALYPSO_ARM2DSP_BGEN` | `calypso_arm2dsp.c:118-125` | traite le symptôme (livelock background) et non la cause (IT trame) ; à retirer **seulement si** C1 fait disparaître le livelock |

**Résultat ATTENDU** : aucun changement fonctionnel si C1/C2 marchent (ces gates ne
tirent plus). **Résultat qui l'INVALIDE** : le retrait de l'un d'eux fait
régresser ⇒ ce n'était pas une béquille, c'était un correctif mal nommé : le
documenter avant de le remettre.

**Non-régression** : commande obligatoire, après **chaque** retrait pris séparément.

---

### C6 — Correctifs d'opcodes hors chemin natif, « neutralisations pures » d'abord
**(i) sur le chemin : NON (aucun n'est dans `0xddf5..0xde86`) — mais tous sont dans
le démod/corrélateur, atteint dès que C1+C2 marchent. (ii) confirmé : OUI, binutils
+ spru.txt. (iii) risque : NUL à FAIBLE.**

Ordre interne, du plus sûr au moins sûr :

**C6a — suppressions pures, 0 site atteignable en PROM0, régression impossible.**
Supprimer `calypso_c54x.c:10732-10740` (0xC2/C3/C6/C7 décodés `RPTB[D]` alors que
`rptb=0xF072`/`rptbd=0xF272`, `tic54x-opc.c:410-411`), `:10762-10769` (0xC4 décodé
`PSHD dmad` alors que `pshd=0x4B00`, `:389`), `:10794-10803` (0xDA décodé `RPTBD`).
Ils retombent alors sur `goto unimpl` (`:10922`) : longueur correcte, log bruyant.
Supprimer aussi le code mort `:10219` (`hi8==0x89`, capté par STLM `:9953`) et
`:10227` (`hi8==0x8B`, capté par POPD `:9968`).

**C6b — arrêt des corruptions actives (13 sites réels en PROM0).**
Supprimer `:10770-10785` : 0xC0 est décodé `PSHD Smem` (**8 sites** `76e4 76f5 a8da
ad39 ad6a ad7b b895 b8b6` → 8 `s->sp--` parasites) et 0xC1 `RPT Smem` (**5 sites**
`b3a8 b4a2 b514 b517 c10f` → `rpt_count` arbitraire ; `b4a2:c1fa` désynchronise en
plus le PC de 2 mots via `resolve_smem` MOD=0xF).
Supprimer `:10786-10793` : 0xCC décodé `SACCD` (`saccd=0x9E00`, `:414`) — site réel
`8531:cc4b`, écriture DP-relative hors cible.
Supprimer `:10818-10825` : 0xDF, seul `data_write(addr+1, …)` de la plage, site
`b49f:df82` dans le bootloader.

**C6c — resynchronisation du PC (gravité 1, sites prouvés).**
`0xE0 FIRS` = **2 mots** (`tic54x-opc.c:304`) : 9 sites déroulés en PROM0
(`83d6, 83f1, 840c, 8427, 8442, 845d, 8478, 8493`, + `b8c0`), décodés en 1 mot →
le PC atterrit sur le `pmad` (`W(0x83d7)=0x0064`) → **désync totale**.
`0xE4/0xE6` = `ST src,Ymem ‖ LD Xmem,T`, **1 mot** (`:481`), aujourd'hui décodés
`BITF Smem,#lk` 2 mots (`bitf = 0x6100`, `:275`) : sites `ad3d`, `ad5d`.
Patch minimal (stub de longueur, sans sémantique) :
```c
if (hi8 == 0xE0) { (void)prog_fetch(s, s->pc + 1); consumed = 2;   /* FIRS : 2 mots */
                   C54_LOG("FIRS unimpl op=0x%04x PC=0x%04x", op, s->pc);
                   return consumed + s->lk_used; }
if (hi8 >= 0xE1 && hi8 <= 0xE3) { C54_LOG("LMS/SQDST/ABDST unimpl 0x%04x", op);
                   return consumed + s->lk_used; }   /* 1 mot */
if (hi8 == 0xE4 || hi8 == 0xE6) { C54_LOG("ST||LD T unimpl 0x%04x", op);
                   return consumed + s->lk_used; }   /* 1 mot (tic54x:481) */
```
en remplacement du catch-all `(op & 0xFC00) == 0xE000` (`:8481-8501`) et du bloc
0xE6 `:8656-8672`. Supprimer aussi `:8612-8631` (`hi8==0xE1`), qui redevient
atteignable et exécuterait des CMPL/NEG/ROR fantaisistes.

**C6d — corrections sémantiques 1 mot (sites réels, zone FB/démod).**
- `0x6F` sous-cas 3 (`STH src,SHIFT,Smem`) : extraction du high **avant** le
  décalage. `spru.txt@560220` : « the src **is shifted left** … and **bits 31−16 of
  the shifted value** are stored ». Sites : `0x8e98`, **`0xa182`, `0xa18d`,
  `0xa198`** (zone FB-det, dans le `RPTB 0xa184→0xa191`).
```c
             case 3: { /* STH SRC1,SHIFT,Smem : (src << SHIFT) >> 16 (spru.txt@560220) */
                 int64_t src = dst_b ? s->b : s->a;
-                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
-                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
-                                               : ((int64_t)high >> (-shift));
-                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
+                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
+                data_write(s, addr, (uint16_t)((shifted >> 16) & 0xFFFF));
                 break; }
```
  (aligne sur le handler ASM 0x86/0x87 `:10175-10182`, déjà validé.)
- `0x8D` = `ST TRN, Smem` **1 mot** (`tic54x-opc.c:427` ; `mvdd = 0xE500`, `:376`).
  10 sites PROM0, dont `0x9a22/9a2f/9a47/9a54` **à l'intérieur** des
  `RPTBD 0x9a20→0x9a30` et `0x9a45→0x9a55`.
- `0x85` = `STL B, ASM, Smem` **1 mot** (`:435`, bit8 = src). 5 sites, tous dans le
  miroir page-0 (`0xeb39, 0xeb3c, 0xeb4d, 0xeb50, 0xeef2`), dont deux **dans** le
  `RPTBD 0xeb37→0xeb3d`. ⚠️ Le miroir page-0 est la zone traversée par SHUNT_LEGIT :
  **ce sous-correctif est le seul de C6 avec un risque SHUNT_LEGIT réel** → le
  passer en dernier de C6d et tester seul.
- `0x62-0x67` MPY/MAC `Smem,#lk` = **2 mots** (`:362`, `:340` ; `spru.txt@505521`
  « Syntaxes 3 and 4: 2 words »), aujourd'hui avalés en LD 1 mot. 20 sites PROM0,
  30 miroir page-0, 58 PROM2 — dont `0xe9a4: 678d 0005 = MAC *AR5-,#5,B` juste
  avant le `RPTBD 0xe9a7→0xe9b6` (noyau FIR).

**Commande de test** : run natif §C1 + non-régression obligatoire.
**Résultat ATTENDU** : aucun changement mesurable tant que C1/C2 ne sont pas
passés (ces PC ne sont pas atteints en natif) ; après C1/C2, sortie du démod
structurée. **Résultat qui l'INVALIDE** : régression SHUNT_LEGIT ⇒ le site est dans
le miroir page-0, isoler le sous-correctif fautif.

---

### C7 — Handler `ST src,Ymem ‖ <op> Xmem,dst` unifié (0xC000-0xDFFF)
**(i) sur le chemin : NON (mais boucle chaude 0x8208-0x821a ≈ 7 % des instructions,
et noyau MAC 0x901e/0x9312 du handler FB). (ii) confirmé : OUI. (iii) risque :
LE PLUS ÉLEVÉ de la liste.**

Cinq défauts confirmés sur le bloc `0xD0-0xD9` (`:10614-10661`) : store parallèle
absent ; `round` lu sur b0 au lieu de b10 (`macr = 0xD400`, `:485`) ; `is_sub`
déclenché dès 0xD4 au lieu de 0xD8 (`mas = 0xD800`, `:487`) → **MACR calculé en
soustraction** ; `dst` lu sur b9 au lieu de b8 (encodage `11010RSD`) ; et surtout
**`s->t = yval_c`** — T écrasé par le contenu du **buffer de sortie** à chaque
itération, ce qui détruit le coefficient du MAC dès le 2ᵉ tour sur
`901e: ST A,*AR5+ ‖ MAC *AR4,A` (sous `RPTZ` armé en `0x901c`). Deux défauts sur
`0xC8-0xCB` (`:10854-10921`) : `S` et `D` confondus/liés alors qu'ils sont
indépendants ; store fait en `STL` alors que `ASM ≠ 16` aux sites (`8206: ed1b`
→ ASM = −5, deux instructions avant `8213: c863`).

**Squelette corrigé** (les trois erreurs du patch de l'audit initial sont réparées :
`dst_` = l'**autre** accumulateur pour ADD/SUB, `c54x_dualop_postmod()` **n'existe
pas** — inliner le `switch` de `:10888-10920` —, arrondi sur le **résultat**) :
```c
case 0xC: case 0xD: {          /* 0xC000-0xDFFF : ST src,Ymem || <op> Xmem,dst — 1 MOT */
    int s_acc = (op >> 9) & 1, d_acc = (op >> 8) & 1;      /* S=b9, D=b8, independants */
    int xmod = (op >> 6) & 3, xar = ((op >> 4) & 3) + 2;
    int ymod = (op >> 2) & 3, yar = ( op       & 3) + 2;
    uint16_t xaddr = s->ar[xar], yaddr = s->ar[yar];
    uint16_t xval  = data_read(s, xaddr);                  /* lire Xmem AVANT le store */
    int64_t  sv    = s_acc ? s->b : s->a;
    int64_t *dst   = d_acc ? &s->b : &s->a;
    int64_t  dst_o = d_acc ? s->a  : s->b;                 /* dst_ = l'AUTRE acc */
    int asm5 = asm_shift(s);                               /* helper existant, :80-86 */
    int64_t sh = (asm5 >= 0) ? (sv << asm5) : (sv >> (-asm5));
    data_write(s, yaddr, (uint16_t)((sh >> 16) & 0xFFFF)); /* src<<ASM-16 -> Ymem */
    int64_t prod, r;
    switch ((op >> 10) & 7) {
    case 0: *dst = sext40(dst_o + ((int64_t)(int16_t)xval << 16)); break;  /* C0-C3 ADD */
    case 1: *dst = sext40(((int64_t)(int16_t)xval << 16) - dst_o); break;  /* C4-C7 SUB */
    case 2: *dst = sext40((int64_t)(int16_t)xval << 16);          break;   /* C8-CB LD  */
    case 3: prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            *dst = sext40(prod); break;                                    /* CC-CF MPY */
    default: prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval;       /* D0-DF MAC/MAS[R] */
            if (s->st1 & ST1_FRCT) prod <<= 1;
            r = (op & 0x0800) ? (*dst - prod) : (*dst + prod);   /* b11 = MAS */
            if (op & 0x0400) { r += 0x8000; r &= ~0xFFFFLL; }    /* b10 = R, arrondi RESULTAT */
            *dst = sext40(r); break;
    }
    /* post-modif : recopier le switch xmod/ymod de :10888-10920 (1=dec,2=inc,3=circ BK) */
    return consumed + s->lk_used;                            /* T n'est JAMAIS ecrit */
}
```
remplace `:10614-10921` en entier. Références : `spru.txt@567069` (ST‖ADD),
`@573901` (ST‖SUB), `@568126` (ST‖LD), `@569617` (ST‖MAC), `@571249` (ST‖MAS),
`@572870` (ST‖MPY) ; `tic54x-opc.c:477-493`.

**Résultat ATTENDU** : sortie du corrélateur/FIR structurée, `T` stable.
**Résultat qui l'INVALIDE** : régression SHUNT_LEGIT ou boucle `0x8208-0x821a`
divergente ⇒ revert en bloc (c'est un seul commit, indivisible).
**Non-régression** : commande obligatoire, **impérative** — c'est le seul correctif
qui touche du code chaud partagé.

---

### C8 — `MACP`/`MACD`/`MVPD`/`MVDP` 2 mots **avec** PAR/RPT
**(i) sur le chemin : NON. (ii) confirmé : OUI. (iii) risque : le patch NAÏF est
PIRE que le bug.**

`macp 0x7800`, `macd 0x7A00`, `mvpd 0x7C00`, `mvdp 0x7D00` — tous **2 mots**
(`tic54x-opc.c:347-348, 371, 375` ; `spru.txt@496052`, `@517883`, `@513219`),
aujourd'hui avalés en `STH` 1 mot (`:9053-9062`). 25 sites PROM0, 15 PROM2.

⚠️ **Bloquant.** `spru.txt@496052` : « pmad → PAR ; **If RC ≠ 0 Then** … **PAR+1 →
PAR** ». **Tous** les sites sont répétés (mot précédent = `0xECxx` = `rpt #K`) :
`0x7e29 ec19 → 7e2a 7892 7a1c`, `0x7e2c ec19`, `0x7e2f ec0b`, `0x84f3/84f6/84f9/
84fe/8501/8504` → les 6 MACP `0x84f4..0x8505`, `0xa79f ec13 → a7a0 7c90 a92c`,
`0xa911 ec13`, `0xa919 ec01`. Appliquer la longueur sans PAR transforme un FIR
26 taps en 26× le même coefficient : on remplace une gravité 1 **bruyante** (désync)
par une gravité 2 **silencieuse** (résultat faux).

⇒ **Réutiliser le mécanisme existant** `s->mvpd_src` / `s->rpt_fresh` de
`READA`/`WRITA` (`:8736-8764`) : `pmad → PAR` à la 1ʳᵉ itération, `PAR+1` ensuite.

⚠️ Le commentaire `:9994-10001` (« Run-trace confirms 0 MVPD hits … firmware did
not issue any 0x7Cxx ») est un **sophisme de mesure** : la sonde tournait avec MVPD
mappé sur 0x8C, elle ne dit rien de 0x7C. Ne pas s'en servir pour minorer.

---

### NE PAS APPLIQUER

| finding | raison |
|---|---|
| `resolve_mmr` — opérande MMR indirect (`op & 0x7F` ignore le bit I) | L'ISA donne raison (`spru.txt@565047`), mais **scan des 4 ROM : 2870 sites d'opérande MMR, 100 % bit 7 = 0**. Zéro forme indirecte dans le firmware. Bénéfice nul, effet de bord AR sur des chemins de boot fonctionnels (`POPM ST1` `:9912`, `STLM B,AR1` `0xb42d`). **Note ISA, pas correctif.** |
| `ADDM` avec flags C/OVA | La saturation OVM seule est justifiée (`spru.txt` ADDM Ex.2). Ajouter `C` = ajouter un flag lu par ~176 `BC C/NC` de tout le firmware **sans sonde préalable** = risque non mesuré. Si on y va : **saturation seule**, C/OVA hors périmètre. |
| Remarque transversale ARP/CMPT | `ST1_CMPT` non exercé, aucun lecteur d'ARP prouvé. Note. |
| `CALYPSO_FIX_MVDM_OFF` | Le gate est trompeur (`:8918` inconditionnel gagne toujours, `:9035` est mort) mais inoffensif. Documenter, ne pas toucher. |
| `PORTR` / `PORTW` (`:8894-8896`, `:8868-8873`) | `fix_portr` **OFF par défaut** = `data[Smem]` jamais écrite ; `PORTW` `(void)pa; (void)addr;`. 25 sites `74f8` en PROM0. **À documenter comme « pas de log ≠ pas d'événement » en puissance**, à traiter seulement si une mesure post-C1 le désigne. |

---

## 4. RÈGLE D'APPLICATION

**Un correctif à la fois. Une mesure entre chaque. Sans exception.**

Procédure, à répéter identiquement :
1. appliquer **un seul** correctif (le plus haut de la liste non encore traité) ;
2. build ;
3. run **natif** (§C1) → relever la signature attendue **et** la signature
   invalidante ;
4. run **SHUNT_LEGIT** (commande obligatoire) → BSIC=7 + SI + LU ACCEPT ;
5. si (3) confirme et (4) est intact : **effacer la condition, pas le correctif** —
   retirer le `if (calypso_fix_enabled("FIX_…")) {` et ses accolades, le code
   devient inconditionnel, le nom disparaît de la liste des gates
   (protocole déjà écrit dans `calypso_c54x.c:5046-5053`) ;
6. si (3) infirme ou (4) régresse : **revert**, consigner le résultat, passer au
   suivant. Pas d'« ajustement » sur place.

**Pourquoi.**
- **Attribution.** Deux correctifs simultanés dans un système qui n'a qu'un seul
  signal binaire (« le corrélateur tourne / ne tourne pas ») rendent l'attribution
  impossible : si le résultat change, on ne sait pas lequel a agi ; s'il ne change
  pas, on ne sait pas si l'un a annulé l'autre. Ce projet en a déjà fait les frais
  (cf. « 30+ writers », « 0 FCCH sur 200 dumps », deux tables d'opcodes fausses,
  tous corrigés après coup).
- **Silence.** Un correctif d'émulation mal fondé ne plante pas : il **change
  silencieusement un résultat numérique**. C6d/C7 touchent le noyau MAC et la
  boucle la plus chaude du DSP (`0x8208-0x821a`, ≈ 7 % des instructions du run) ;
  C4 touche une boucle exécutée 10⁶ fois. Une régression y est invisible sans le
  test SHUNT_LEGIT complet — pas un boot, un **camp + LU** de bout en bout.
- **Le mode qui marche est le seul oracle.** SHUNT_LEGIT campe et fait LU/SMS
  aujourd'hui. C'est la seule référence de non-régression disponible. Elle doit
  être vérifiée après **chaque** correctif, y compris ceux dont on « sait » qu'ils
  ne peuvent pas la toucher : trois affirmations de ce type se sont révélées
  fausses dans la journée.
- **Le sas n'est pas une option.** `CALYPSO_FIXES` est un **sas temporaire**. Un
  correctif validé qui reste derrière son gate devient une béquille de plus. Le
  vider est une étape du protocole, pas un nettoyage ultérieur.

---

## 5. TEST DE DIAGNOSTIC PRÉALABLE — passe AVANT tout correctif

Ces trois tests ne corrigent rien. Leur résultat **réordonne** la §3.

### D0 — Purger les semences (coût nul, aucune modification de code)
```
cd ${QEMU_TREE} && CALYPSO_DSP=c54x CALYPSO_SHUNT_LEGIT=0 \
  CALYPSO_INIT_435B_OFF=1 CALYPSO_WATCH_9F00_RD=1 ./start-clean.sh
```
(sans `CALYPSO_KEEP_IMR`, sans `CALYPSO_FIXES`, sans `CALYPSO_ARM2DSP_*`.)
**Attendu** : `IMR` ne contient **jamais** le bit 5 ; `data[0x435b]` reste 0.
**Si le bit 5 apparaît quand même** : il reste un injecteur non identifié dans le
modèle — le traquer **avant** tout le reste, sinon toutes les mesures suivantes
sont contaminées. Vérifier d'abord que `CALYPSO_INIT_435B_OFF` est bien présent
dans `/proc/<pid>/environ` (le gate teste la **valeur**, pas la présence :
`calypso_c54x.c:13672`).

### D1 — Démasquage artificiel du bit 5 : BRINT0 est-elle le DERNIER verrou ?
**Le gate existe déjà, défaut OFF** : `calypso_c54x.c:16929-16937`,
`CALYPSO_FIXES=FIX_BRINT0_UNMASK`. Il force `unmasked = true` pour `imr_bit == 5`.
```
cd ${QEMU_TREE} && CALYPSO_DSP=c54x CALYPSO_SHUNT_LEGIT=0 \
  CALYPSO_INIT_435B_OFF=1 CALYPSO_WATCH_9F00_RD=1 \
  CALYPSO_FIXES=FIX_BRINT0_UNMASK ./start-clean.sh
```
**Résultat ATTENDU (prédiction ferme, à partir de §1.1)** : le vec 21 est
vectorisé, exécute `RETE ; NOP`, revient — et **`CALYPSO_WATCH_9F00_RD` reste 0**.
C'est déjà ce qu'a mesuré le run `MXOwop` (100 `DISPATCH(normal)` / 100 levées,
`CORR-ENTRY=1`, `9f00=0`).
**Conclusion si attendu** : BRINT0-bit-5 n'est **ni** le dernier **ni** le prochain
verrou — c'est une **fausse piste complète**. Priorité n°1 = **C1**. Retirer
`FIX_BRINT0_UNMASK` du code (C5 §3).
**Résultat qui INVALIDERAIT** : `9f00` devient non nul ⇒ contre toute la §1, le
firmware **utilise** le vec 21 ; alors la table de vecteurs lue en PDROM n'est pas
celle qui est active au runtime (overlay ? seconde copie ?) — re-dumper
`data[0x0080..0x00FF]` **au moment du dispatch**, pas à l'installation.

### D2 — Impulsion IFR bit 12 : la chaîne trame est-elle bien celle-là ?
**Diagnostic gaté par env, défaut OFF, à ajouter temporairement dans
`calypso_trx.c` §4** — ne touche ni l'IMR, ni la table de vecteurs, ni `d_fb_det` :
```c
/* @DIAGNOSTIC — CALYPSO_DIAG_FRAME_BIT12 (defaut OFF). Repond a UNE question :
 * l'IT trame sur bit12/vec28 sort-elle le DSP de la boucle background et
 * fait-elle armer BRINT0 par le ROM lui-meme ? A RETIRER, jamais a confirmer :
 * le correctif est C1 (constante C54X_INT_FRAME_VEC/BIT), pas ce forcage. */
{ static int _d12 = -1; if (_d12 < 0) _d12 = getenv("CALYPSO_DIAG_FRAME_BIT12") ? 1 : 0;
  if (_d12 && s->dsp && s->dsp->running) s->dsp->ifr |= (1u << 12); }
```
**Résultat ATTENDU** : `IRQ-LEVEL take bit=12 vec=28 -> PC=0x00f0`, sortie de
`0xddf5..0xde88`, puis — signature décisive — `IMR-ARM … -> 0x3010 PC=0xbd40` ou
`PC=0xc471` : **le ROM arme BRINT0 tout seul**.
**Résultat qui l'INVALIDE** : la vectorisation a lieu mais le PC ne descend pas
dans `0xa4e4` ⇒ le handler `0x7234` déraille (suspect `CALL 0x013b`, 23 `PSHM`,
et le SP) ; ou le DSP retombe aussitôt dans le background ⇒ le livelock
`d[0x098c]≠0 ∧ d[0x098d]==0` est bien une racine **indépendante** (§6, I3).

**Mesure complémentaire, 3 lignes, à faire dans le même run** : logger
`data[0x098b]`, `data[0x098c]`, `data[0x098d]` à chaque passage en `PC == 0xDDFD`,
40 tirs. Tranche le livelock en une mesure.

---

## 6. CE QUI RESTE INCERTAIN — et la mesure qui tranche

**I1 — Direction du `MVKD` en `PROM0 0xa501` (`70f8 435b 0000`).**
Statut : **HYPOTHÈSE**. `mvkd 0x7000 {OP_dmad, OP_Smem}` → `Smem ← data[dmad]`, soit
`d[0x435b] ← IMR` (sauvegarde). C'est la seule lecture qui rend `0xa582`
(`IMR |= d[0x435b]`) capable de restaurer les bits 12/13 détruits par `0xa509`.
Mais le run mesuré rapporte `d[0x435b] = 0` alors que `IMR = 0x3000` — incohérent
si la sauvegarde avait lieu (or `0xa501` n'est jamais atteint : cohérent).
**Mesure** : sonde à `PC == 0xa501`, dumper `data[0x435b]` et `s->imr` avant/après.
En lecture « sauvegarde », le shadow doit prendre la valeur courante de l'IMR.
Tranche définitivement. À faire **dans le run C1**, où `0xa501` sera atteint.

**I2 — Qui, côté ARM, déclenche le dispatcheur `0xc1fa` ?**
Statut : **fait mécanique certain / attribution fonctionnelle HYPOTHÈSE**.
Mécanique (LECTURE DE CODE, certaine) : `d[0x3fd4] = 0xc1fa` posé en `0xb4a0` ;
appelé **chaque trame** depuis `0xa57c` (`LD *(0x3fd4),A ; CALA`) ; `0xc1fe`
compare `d[0x0906]` et `d[0x0907]` ; `0xc215 BITF #0x0002 → CC 0xc25f → 0xc266
CALL 0xc272 → 0xc275 CALL 0xc446 → 0xc471 IMR |= 0x0010`.
Attribution (HYPOTHÈSE) : par comptage des champs de `T_NDB_MCU_DSP`
(`dsp_api.h`, branche `DSP == 33`), ancré sur `d_fb_det = 0x08F8` (valeur connue du
projet), `0x0906 = d_audio_init` et `0x0907 = d_audio_status`. Le firmware osmocom
n'écrit **pas** `d_audio_init` (pas d'audio en L1 nu). Deux lectures possibles :
(a) le ROM TI mutualise BSP0 entre la voie DAI et la chaîne RIF, et le vrai chemin
RX passe par les jumelles `0xbd1e`/`0xbd47`, gatées par `d[0x4352]` bit 0 et
`d[0x3f70] == 2` (sonde `F70-SETBIT1` déjà présente) ; (b) le comptage de champs
est décalé.
**Mesure** : compteurs bruts d'atteignabilité sur `0xbd30, 0xbd40, 0xbd62, 0xc446,
0xc471, 0xc498, 0xa500, 0xa509`, dans le run C1. Tranche « routine jamais atteinte »
vs « atteinte, test `BITF` négatif », et désigne laquelle des deux familles sert.

**I3 — Le livelock background est-il une racine indépendante ?**
Statut : **HYPOTHÈSE forte**. La boucle `0xddf5..0xde88` ne sort que si
`d[0x098c] == 0` ; si `d[0x098c] != 0` **et** `d[0x098d] == 0` (aucune tâche
enregistrée), le test `0xddff` saute à `0xde82` et **aucun bit n'est jamais
effacé** — le corps se réduit à `0xddf5 → 0xddf9 → 0xde82 → 0xde84 → 0xde86 →
0xddf5`, ce qui **explique exactement** l'oscillation IMR mesurée. Comme la boucle
est interruptible (INTM=0), elle n'est pas la barrière — mais elle peut redevenir
le verrou suivant si la tâche trame y retombe.
**Mesure** : la sonde `PC == 0xDDFD` de D2, plus un watch en écriture sur
`data[0x098c]` distinguant écritures ARM/MMIO et DSP.
⚠️ Tension assumée avec la note projet `ndb-cells-098a-background-redherring`
(« ARM=0 par design ») : « ARM=0 » sur `0x098d` **avec** `0x098c != 0` produit
précisément ce livelock.

**I4 — Nomenclature RIF vs BSP.**
Statut : **divergence non tranchée**. `Calypso_overview` p.42 nomme les IT du RIF
`INT0n` (receive) / `INT1n` (transmit), ce qui ne colle pas avec BRINT0. La preuve
directe reste du côté BSP (le ROM exécute mot pour mot l'Example 9-4 de TI sur
`MMR 0x22`, §1.2c). On retient « RIF = port série tamponné » et on consigne la
divergence.
**Mesure** : C3a (sonde MMR ≥ 0x20). Si le firmware écrit bien `0x0008`/`0x00C8` en
`0x22`, la question est close.

**I5 — Périmètre du chiffrage « sites atteignables ».**
Statut : **limite de méthode**. Tout le chiffrage de la §3 (C6/C7) porte sur
**PROM0 (0x7000-0xDFFF)**, par descente récursive depuis 40 amorces + les 128
vecteurs. Les balayages **linéaires non ancrés** de PROM1 (miroir page-0
`0xE000-0xFFFF`) et PROM2 (overlay du handler FB) donnent C4≈10, C3≈73, DA≈1,
E4≈18, E6≈16 : les findings classés « latents » **ne doivent pas être abandonnés**.
Le miroir page-0 est en particulier la zone traversée par SHUNT_LEGIT.
**Mesure** : trace d'exécution (pas analyse statique) des PC atteints en mode
SHUNT_LEGIT, à croiser avec les listes de sites.

**I6 — `d_spcx_rif` (`0x08D6`) est configuré par l'ARM et ignoré par le modèle.**
Statut : **LECTURE DE CODE + HYPOTHÈSE**. `calypso/dsp.c:429` :
`ndb->d_spcx_rif = 0x179` ; `dsp_api.h:119` : « RIF control (MCU -> DSP) ».
« SPCX » = *Serial Port Control eXtension* = **BSPCE**. Le ROM le consomme
(`0xb56c: 10f8 08d6` = `LD *(0x08d6)`, puis `AND #0xfffd`, puis `STL A,*(0x3fd2)`).
Le modèle ne fait que **logger** cette écriture (`calypso_c54x.c:4228-4235`).
**Mesure** : après C3b, vérifier que `0x179` traverse jusqu'à un `BSPCE0` modélisé,
et que la valeur écrite en `d[0x3fd2]` est cohérente.

---

## 7. Conclusions antérieures explicitement contredites

| Conclusion antérieure | Statut |
|---|---|
| « BRINT0 = IMR bit 5 / vec 21 » (`calypso_c54x.h:117-124`, `calypso_c54x.c:2404`, `:14242`, tâche RANK3) | **FAUX.** SPRU131G `:10755-10790` : bit 5 = BXINT0. BRINT0 = bit 4 / vec 20. Corroboré par la table de vecteurs PDROM, par 6 `STM #0x0010,IFR`, par 4 `ORM #0x0010,IMR`, et par le couple `IMR &= 0xC150` / `IFR = 0x3EAF` à `0xb3a6/0xb3a9`. |
| « le vecteur 21 est installé à zéro » | **FAUX.** `f4eb f495 0000 0000` = `RETE ; NOP` (PDROM idx 0x0375). Stub délibéré, définitif (sonde `VEC-INSTALL` : 46 tirs sur un cap de 200, aucune réécriture de tout le run). |
| « RANK3 : verrou natif = BRINT0 masquée » | **À REQUALIFIER.** Le mur mesuré est réel ; sa cause nommée ne l'est pas. Le verrou est l'IT trame `vec 28 / bit 12`, deux crans en amont. |
| `golive-imr-shadow-435b` : « 0xa582 écrit IMR=0, écrase l'arm 0xa4c7 » | **FAUX.** `0xa582` = `IMR \|= d[0x435b]` (`1af8 0000` = `OR *(0x0000),A`). Le seul écrivain destructeur est `0xa509 STM #0x0010,IMR`, intentionnel. |
| `d[0x435b]` = cellule d'API RAM à peupler par l'ARM (`INIT-435B`, défaut ON) | **FAUX.** Shadow interne DSP (`ARM_off = 0x76B6`, hors des `0x2000` mots d'API RAM), écrit exclusivement par 6 instructions ROM appariées 1:1 avec des écritures IMR. Le semer est une **béquille circulaire**. |
| « IMR = 0x52fd = masque de reset du DSP réel » | **FAUX.** Aucune occurrence de `0x52fd` dans les 6 images ROM. Valeur codée en dur `calypso_c54x.c:16631`, réinjectée par `KEEP-IMR` (`:14266-14280`). Masque réel post-reset : `0` (`0xb37e`) puis `& 0xC150` (`0xb3a6`). |
| `golive-gate-a53c-0810-bit15` / RANK1 « completed » | **À ROUVRIR.** `d_ctrl_system` n'a aucun bit d'interruption : `B_TSQ=0`, `B_BCCH_FREQ_IND=3`, `B_TASK_ABORT=15` (`l1_environment.h:364-367`). Seules écritures osmocom : `sync.c:312` (abort) et `dsp.c:483/490` (TSC). Le commentaire `calypso_arm2dsp.c:83-91` (« the real ARM asserts this bit in l1s_reset() ») est faux. Confirme `golive-bit15-abort-not-golive`. |
| `dsp-dpage-offset-bug` (marqué « contesté ») | **CONFIRMÉ.** `d_dsp_page = 0x08D4` (`dsp_api.h:18-22` + `PROM0 0xa51c: 10f8 08d4`). `0x08E2 = d_dsp_state`, initialisé à **3** (`calypso/dsp.c:215`) ⇒ le test `data[0x08E2] & 0x0002` de `calypso_c54x.c:16862` est **toujours vrai**. |
| `ndb-cells-098a-background-redherring` | **RENFORCÉ sur le fond, nuancé sur la cause.** `0xddf5..0xde88` est bien la boucle background ; le DSP y est piégé **parce qu'aucune IT trame n'arrive**, pas parce que `0x098a` vaut 0. |
| `calypso_c54x.c:13678` : « 0x52fd = bit4 opt-in casse le frame, prouve que le firmware n'utilise PAS TINT0 » | **EXPÉRIENCE VALIDE, INTERPRÉTATION INVERSÉE.** Armer le bit 4 dispatchait le **handler de réception série `0x72d3`**, pas un timer. |
| `calypso_bsp.c:1077-1080` : « PROM1[0xFFD4] → CALL 0xf310 » justifie le vec 21 | **FAUX.** `doc/DSP_ROM_MAP.md:21` et le dump de `PROM1 0xFF80-0xFFFF` montrent du code de boot séquentiel, pas une table de vecteurs. La vraie table source est en `PDROM idx 0x0321`, copiée en DARAM `0x0080` (IPTR=1, MESURE `PMST-WR #5 val=0x00b8 IPTR=0x001 PC=0xb392`). |
| Tables d'opcodes « 0xF4..0xF7 = 2 mots » et « 0xEA = BANZ » | **DÉJÀ CORRIGÉES le 2026-07-28.** binutils : `add 0xF480/0xFCFF` **1 mot** (`:246`), `ld #k9,DP 0xEA00/0xFE00` (`:319`), `banz 0x6C00` (`:268`), `banzd 0x6E00`. **Aucun** finding de ce plan ne s'appuie sur les anciennes entrées. |

---

## 8. Fichiers et artefacts

**Sources citées (aucune modifiée)** :
`${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` ·
`${QEMU_TREE}/hw/arm/calypso/calypso_c54x.h` ·
`${QEMU_TREE}/hw/arm/calypso/calypso_bsp.c` ·
`${QEMU_TREE}/hw/arm/calypso/calypso_tpu.c` ·
`${QEMU_TREE}/hw/arm/calypso/calypso_trx.c` ·
`${QEMU_TREE}/hw/arm/calypso/calypso_arm2dsp.c` ·
`${QEMU_TREE}/hw/arm/calypso/doc/opcodes/tic54x-opc.c` ·
`${QEMU_TREE}/hw/arm/calypso/doc/spru172c.pdf` ·
`${GSM_ROOT}/osmocom-bb/src/target/firmware/include/calypso/dsp_api.h` ·
`${GSM_ROOT}/osmocom-bb/src/target/firmware/include/calypso/l1_environment.h` ·
`${GSM_ROOT}/osmocom-bb/src/target/firmware/calypso/dsp.c` ·
`${GSM_ROOT}/osmocom-bb/src/target/firmware/layer1/sync.c` ·
`${GSM_ROOT}/osmocom-bb/src/target/firmware/layer1/prim_fbsb.c`

**Images ROM** : `${GSM_ROOT}/calypso_dsp.{PROM0,PROM1,PROM2,PROM3,PDROM,DROM}.bin`
(PROM0 base **0x7000**, mots **little-endian** — revérifié : `W(0xddf9)=0x6881`,
`W(0xde84)=0x6981`, exactement les opcodes du log natif).

**Extraits doc et outils produits hors conteneur (lecture seule)** :
`/root/.claude/jobs/26578783/tmp/spru.txt` (SPRU172C) ·
`/root/.claude/jobs/26578783/tmp/spru131g.txt` (SPRU131G, 31 293 l.) ·
`/root/.claude/jobs/26578783/tmp/spra036.txt` · `calypso_ov.txt` ·
`dis54.py` (désassembleur C54x avec mot d'extension Smem) ·
`imrscan.py` (scan des écrivains IMR, 4 banques) ·
`xref.py` + `xref_PROM0.json` (graphe d'appels PROM0, 13 733 instructions) ·
`wf_results/dis.py` (table binutils + désassembleur ancré + descente récursive).

**Rapports amont** : `${QEMU_TREE}/RAPPORT_OPCODES.md` ·
`${QEMU_TREE}/RAPPORT_DFBDET.md`
