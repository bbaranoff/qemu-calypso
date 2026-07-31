# ETAT ACTUEL — QEMU-Calypso (source de verite unique)

> Document de reference courant. Ancre sur les mesures du **2026-07-30** (§12,
> qui PRIME sur la §11 et sur tout le reste).
> **Regle de resolution de conflit :** ce document prime sur tout fait extrait d'un autre
> doc. Le STATUT DEPEND DU MODE : ne jamais citer un statut sans le mode. Toute
> affirmation technique doit nommer son INSTRUMENT. Les autres documents sont dans
> `doc/archive/` (reference durable uniquement, cf. §10).
>
> Convention de marquage utilisee partout ci-dessous : **MESURE** (instrument nomme),
> **HYPOTHESE** (testable, non tranchee), **INVALIDE** (mesure contraire, cf. §8).

---

## 12. ETAT AU 2026-07-30 — la chaine FB cartographiee, une racine corrigee

> Cette section PRIME sur la §11 et sur toutes les precedentes. Chaque fait nomme
> son instrument. Toute affirmation sans instrument est a considerer comme
> HYPOTHESE, y compris si elle vient de moi.

### 12.1 RACINE TROUVEE ET CORRIGEE : l'overlay 2 octets sur `d_dsp_page`

`calypso_dsp_shunt.c` (~l.2061) superpose une `MemoryRegion` de **2 octets** sur
`0xFFD001A8` (= `d_dsp_page`, NDB+0, cellule DSP `0x08D4`) en **priorite 10**, et
son handler d'ecriture ne stockait rien — il croyait a des « pass-through
semantics » **qui n'existent pas dans QEMU** : la region de plus haute priorite
traite l'acces EXCLUSIVEMENT.

Consequences, toutes MESUREES avant correctif :
- `calypso_dsp_write()` n'etait jamais appele pour ce mot → **aucune sonde ne
  pouvait le voir** : `DDP-ANY` (ungated depuis le 22/07) **0 tir sur 17
  journaux**, `WR-OP`, moniteur mailbox, `DPAGE_HUNT` — tous muets ;
- la cellule gardait son dechet de boot **`0xf600`** : bit1 `B_GSM_TASK` = 0 (« aucune
  tache GSM ») et bit0 = 0 (page 0 latchee a vie ; le DSP ne relisait `0x08D4`
  **qu'une seule fois par run**).

**Correctif** : le handler appelle `calypso_trx_api_commit_w(0x01A8, val)`, nouvelle
fonction de `calypso_trx.c` qui ecrit dans les **DEUX** banques — `dsp_ram[]` (ce
que l'ARM relit, et la SOURCE du miroir par tick, sinon ecrase au tick suivant) et
`dsp->data[]` (ce que le DSP lit) — sans verrou (mutex DARAM non recursif, meme
precedent que `shunt_c54x_api_rd`) et **sans** passer par l'espace d'adressage
(sinon recursion dans l'overlay). Elle journalise aussi le sens `ARM>WR`, sans quoi
le journal mentait par asymetrie. Declaration dans `calypso_dsp_internal.h` pour le
shunt et `calypso_trx.h` pour `calypso_trx.c` : inclure les deux casse sur
`-Werror=redundant-decls`.

**MESURE apres correctif** : `d_dsp_page` prend **0x0002 / 0x0003 / 0x0000** en
alternance ; lectures DSP **1 par run → 210** ; la **page 1** (`0x0814`-`0x0819`)
est enfin lue (des centaines de fois, zero avant) ; cibles `CALA-WIDE` enrichies
(`0xb5ed`, `0xb62c`, `0x71a1`, `0x7000/0x7002`, `0xc827`, `0xd294`, `0xdda8`).

### 12.2 La chaine FB, de l'ordre ARM au kernel — chaque etape comptee

```
ARM d_task_md=5  →  0xb01c lit data[0x43d8]  →  0xab77         (tremplin)
  →  0xaa6c (repartiteur sur d[0x3fde])  →  cala               45x
  →  0xb313 leve d[0x3f92] bit13         13x
  →  0xb332 le teste  →  call 0xb75e     13x
  →  0xb75e ARME data[0x0158..0159] = « call 0x728a »
  →  slot d'IT 30 (data 0x00F8-0x00FB = « fb 0x0158 »)   ← MAILLON MANQUANT
  →  0x728a  →  …  →  kernel 0xa076   1503 exec, COEFFS-WR 11 620, 0x2a00 ecrit 520x
```

- `data[0x43d8]=0xab38` (=`RET`) n'est pas une racine : c'est **l'entree vide de la
  table de handlers**, base **`0xaae8`** dans PROM0 (`… ab60 [ab77] ab9b …`, index =
  numero de tache : **5 = FB, 6 = SB**), posee par le bloc idle `0xbb00`.
- La file `0x4330` n'est pas un verrou : `0xaa6c` la **contourne** quand `d[0x3fde]`
  dit « DSP libre ». Le producteur `0xaa7f` fait bien avancer `0x433f` (une fois par
  trame) et `0xaa83` signale l'anneau plein 17-23x, mais le chemin qui sert est
  l'execution immediate.
- Le slot `0x0158` est un **tremplin logiciel** dans la fenetre `OVLY` (data
  0x100-0x1FF superposee au programme), **pas** un vecteur : les 32 vecteurs de 4
  mots occupent 0x80-0xFF. Le vecteur qui y branche est `data[0x00FA]` = `fb 0x0158`,
  recopie de **PDROM 0xe399** par le `reada` de `0xb4c9` (149 mots, programme
  `0xe321` → data `0x0080`).
- Cinq ecrivains de `0x0158` dans les ROM, tous des `st` : `0xa5ce` (boucle
  principale, installe `0x7242`, le handler LEGER), `0xb75e`/`0xb765`/`0xb76c` (les
  trois variantes de tache) et un cinquieme dans **PROM1** `@0x19fe1`.

### 12.3 IT trame TPU→DSP : ✅ CABLEE — ❌ et elle ne rearme PAS le dispatch

> **Verdict en DEUX moities. Lire les deux : la premiere est un acquis, la seconde
> est une hypothese A MOI, infirmee, et c'est celle-la qu'il ne faut pas repayer.**

#### ✅ Moitie 1 — le cablage etait juste. `CALYPSO_FORCE_VEC` est morte.

MESURE : `calypso_tpu.c` n'avait **qu'un seul** point d'emission d'IT DSP
(`seq_exec_move` → vec21/bit5, sur un `MOVE TPUI_DSP_INT_PG` que le firmware n'enfile
jamais). **Aucune IT de fin de trame n'existait dans le modele.** Cablee le 30/07 dans
`calypso_tpu_sequencer_tick()` (`tpu_frame_irq_to_dsp()`), sous la seule condition des
DEUX registres que le firmware arme lui-meme dans `dsp_end_scenario()` :
`TPU_CTRL.DSP_EN` (actif-haut, pose par `tpu_dsp_frameirq_enable()`) et
`INT_CTRL.DSP_FRAME` (actif-BAS, **efface** par `tpu_frame_irq_en(mcu,dsp=1)`).

Constat au run : `IT trame -> DSP #1 vec30/bit14 (TPU_CTRL=0x0410 INT_CTRL=0x02)`.
Le firmware armait donc les deux bits **depuis toujours** ; personne ne delivrait la
ligne. Et avec `DISPATCH_INSTALL`, `0x728a` s'execute **sans `CALYPSO_FORCE_VEC`**.

C'est du **wiring** (categorie « 18 wire-only / 2 implement » de l'audit du 26/07), pas
un poke : un firmware qui n'arme pas l'IT ne la recoit pas. Gate
`CALYPSO_TPU_DSP_FRAME_IT` (defaut 0 le temps de valider), vecteur reglable par
`CALYPSO_TPU_DSP_FRAME_VEC` (defaut 30). ⚠️ Le NUMERO de vecteur reste une HYPOTHESE.
➡️ **`CALYPSO_FORCE_VEC` retiree** : nee et morte le 30/07, remplacee par du cablage.

#### ❌ Moitie 2 — le couplage etait FAUX

**Mon hypothese** : « `DISPATCH_INSTALL` compense peut-etre aussi un dispatch jamais
rearme faute d'IT ; cabler l'IT ferait tomber les deux bequilles. » **INFIRMEE en un
run.** Sans `DISPATCH_INSTALL`, IT cablee et emise 20 fois :

```
data[0x43d8] : 0xab38, 2 fois, PC=0xbb00, insn 2166/3093 — RIEN D'AUTRE
0x728a : 0 entree     WMAP : dans plages=0     fb0_ret=0, data[] et api[] a zero
```
Reproduit a l'identique en `MODE=shunt_legit` : le mode n'y change rien.

**L'IT trame et l'installateur de handler sont deux trous INDEPENDANTS.** Le premier
est bouche, le second intact. Enchainement : `0x43d8`=`RET` → la tache FB n'arme jamais
`data[0x0158]` → le tremplin garde ce que la boucle principale y a mis → le vecteur
arrive et **n'a rien a invoquer**.

#### Zone grise assumee, et comment la lever

`0x7242` (handler leger installe par `0xa5ce`) est a **0 occurrence** alors que le
vecteur part 20 fois. Deux lectures possibles — le vecteur 30 n'atteint pas le
tremplin `0x0158`, ou aucune sonde armee ne couvre `0x7242` (le piege de §12.5bis).
Certitude : `OVLD-WR data[0x0154]/[0x0155] PC=0x7213`, la fenetre `OVLY` est ecrite au
boot, la zone vit.
**Mesure** : ne pas instrumenter la destination, **instrumenter le mecanisme** — dans
la sequence d'interruption de `calypso_c54x.c`, logger numero de slot, adresse de
vecteur calculee, mot lu a cette adresse, PC effectivement charge. 20 lignes,
independantes de toute sonde de PC et du rendu du desassembleur.

### 12.3bis Contexte firmware (verifie) : pourquoi ce n'etait pas osmocom-bb

**Le cote firmware est VERIFIE, et ce n'est pas un defaut d'osmocom-bb.** Sur silicium,
rien dans le code ARM ne « leve » l'IT trame du DSP : **c'est le TPU qui la genere
seul**, quand son sequenceur atteint le point programme. `tpu_enq_dsp_irq()`
(`include/calypso/tpu.h:114`) et `tpu_force_dsp_frame_irq()` (`calypso/tpu.c:313`) sont
des trappes de debug, **sans appelant par construction** — leur absence d'appelant
n'est donc PAS le bug. Seul le frame-IT (`TPU_CTRL_DSP_EN` + `ICTRL_DSP_FRAME`, poses
par `dsp_end_scenario()`) est utilise, et il fonctionne.

**Le trou est donc dans `calypso_tpu.c`.** MESURE : **un seul** point d'emission d'IT
DSP dans tout le fichier — `l.118 c54x_interrupt_ex(seq.dsp, 21, 5)` — et il ne tire
que sur un `MOVE TPUI_DSP_INT_PG` dans le scenario, c'est-a-dire jamais.
**Aucune IT de fin de trame vers le DSP n'existe dans le modele.**

Ca recolle exactement au verdict du 28/07 (la ROM arme `IMR |= 0x3000` = bits 12/13 =
vec 28/29, le modele emet sur vec19/bit3 → condition auto-fausse) : **meme bug, vu du
bon cote**. Et le bit 14 (vec 30) **demasque en permanence** est la signature d'une
source **materielle**, pas logicielle.

**Test qui tranche** : lever le vec 30 dans le modele TPU a la fin de trame du
sequenceur, puis **retirer `CALYPSO_FORCE_VEC`**. Si `0x728a` part tout seul, ce n'est
plus un `NATIVE_HELPED`, c'est du natif — et ca pourrait faire tomber **les deux**
bequilles, `DISPATCH_INSTALL` compensant peut-etre aussi un dispatch jamais rearme
faute d'IT.

⚠️ La correspondance « vec 30 ↔ fin de trame TPU » reste une HYPOTHESE : la table des
interruptions du DSP Calypso n'est nulle part dans le depot.

### 12.3bis La mecanique du vecteur, mesuree

**Regle de correspondance** : `IMR bit = vec - 16` (documentee `calypso_dma.c:185`,
validee par la mesure : IT trame vue a `PC=0x00f0` = slot 28 = bit 12, 10 980 fois).
Donc **slot 30 ↔ IMR bit 14 (0x4000)**.

MESURE : IMR = 0x72ed / 0x72ef / 0x52ed / 0x70ed / 0x70ef — **bit 14 demasque en
permanence** ; IFR ne prend **jamais** que 0x0020 / 0x0000 / 0x0008 ; et dans le
modele les seules mises a 1 de l'IFR sont bit 12 (`calypso_c54x.c:5108`) et bit 5
(BSP). **Rien ne leve le bit 14.**

Cote firmware `osmocom-bb`, trois mecanismes de signalisation vers le DSP, **deux
morts** : `tpu_force_dsp_frame_irq()` (`calypso/tpu.c:313`, `ICTRL_DSP_FRAME_FORCE`)
**aucun appelant** ; `tpu_enq_dsp_irq()` (`include/calypso/tpu.h:114` →
`tpu_enq_move(TPUI_DSP_INT_PG=0x10, 1)`) **aucun appelant** ; seul le frame-IT
(`TPU_CTRL_DSP_EN` + `ICTRL_DSP_FRAME`, poses par `dsp_end_scenario()`) est utilise.
Notre modele mappe `DSP_INT_PG` → BRINT0 / vec21 / bit5 (`calypso_tpu.c:103`).

**TEST DECISIF** (`CALYPSO_FORCE_VEC=30`, bequille) : le handler `0x728a`
**s'execute** — `TRACEFROM === entree 0x728a`, muet dans tous les runs precedents —
le kernel tourne, `0x2a00` s'ecrit. Donc **tout l'aval du vecteur est sain**. Il
reste a identifier la ligne materielle qui porte ce vecteur : la table des
interruptions du DSP Calypso **n'est nulle part dans le depot** (`calypso_dsp.txt`
n'en contient rien). Lacune documentaire, pas un bug a chercher au grep.

Effet de bord MESURE de l'injection : `d_error_status` prend une valeur **neuve**,
`0x0800` (bit 11, 48x) et `0x0808` (106x) — visible cote ARM dans osmocon
(« DSP Error Status: 2048 / 2056 »). Le DSP **traite** le vecteur et signale une
erreur : le vecteur est de la bonne famille mais arrive dans un etat qu'il refuse.

### 12.4 Structure du correlateur, et ou la decision N'EST PAS

```
0xa045  rptb 0xa09e          ; block-repeat materiel, le kernel est le CORPS
0xa048  banz *AR2, 0xa058    ; saute le calcul des coeffs si AR2 != 0 (polyphase)
0xa04a..0xa057  add/sub *AR1±,B puis sth @0x60..0x62,B  ; 4 coefficients (papillon)
0xa054  bd 0xa076            ; branchement differe vers le kernel
0xa076..0xa09f  8 taps deroules (3060/5a85/5f95/8e94/8f93 x8) puis RET
```
**Aucun branchement conditionnel dans le kernel** : bloc de calcul feuille. La
decision « detecte » ne peut donc pas s'y prendre — elle est **hors du `rptb`**.
C'est la qu'il faut chercher le chemin vers `0x79e4` (`ORM #1,*(0x08f8)`), qui reste
a **0 occurrence**. Etat du resultat : `d_fb_det` n'est jamais que **remis a zero**
(`0xb2cc` = `st *(0x08f8),#0x0000`, prologue de tache, 115x), jamais pose.

### 12.5 Pistes ELIMINEES par la mesure le 30/07 — ne pas les rechasser

| piste | verdict |
|---|---|
| horloge : `fn` gele / bondissant | ✗ CORRIGE par `TDMA_REALTIME=0`. Cause : en REALTIME un pthread mur est maitre du FN pendant que le tick n'avance qu'a ~10 Hz. En VIRTUAL le tick est maitre (1 trame/tick) et la radio suit le meme CLK |
| `CALYPSO_DSP_BUDGET` | ✗ **MORT** : le vrai plafond est `CALYPSO_DSP_YIELD` (`calypso_c54x.c:16900`, defaut statique **32768**), qui rend la main a la mainloop. `dsp_n_exec` vaut 32768 exactement, jamais 256000 |
| course tache ↔ boucle principale sur `0x0159` | ✗ `0x728a` reste arme **~36 trames** en mediane (min 1, max 90, 76 fenetres) ; `0xa5d1` ne remet `0x7242` que **44 insn** avant la re-installation |
| IT de fin de DMA | ✗ `CALYPSO_DMA=1` s'annonce et ne fait **rien** : le DSP ne programme jamais les MMR du DMA (re-confirme les 0 acces a 0x54-0x57) |
| banque programme jamais commutee | ✗ `CALYPSO_XPCWATCH=1` : pages **0 ET 1** vues, `fcall`/`fret` lointains fonctionnels — 3 excursions, toutes a l'amorcage (insn 3034→5339), puis plus jamais |
| reference du correlateur degeneree (DC) | ✗ **RETRACTE**, voir 12.6 |
| **`BANZ *ARx(lk)` casse a `0xde5a`** (note de `PLAN_APPLICATION.md`) | ✗ **INNOCENTE**. Site reel (`6ce6 0001 de46` → ARx=6, MOD=0xC, lk=0x0001, cible **0xde46**), mais l'executeur `calypso_c54x.c:9629` est conforme : AR par `op & 7`, test **pre-modification**, `pmad` lu a `pc+2` en sautant le `lk`, 3 mots consommes. Le commentaire y note que le bug off-by-ARP a **deja** ete corrige. **La note de `PLAN_APPLICATION.md` est PERIMEE** et a failli couter une priorite #1 entiere — c'est precisement le cout que ce document existe pour eviter |
| `bitf` de `0xa53c` teste le bit 4 → « B_TASK_ABORT derriere nous » | ✗ c'est le **bit 15**. `61e1 0010 8000` : `0x0010` = offset (`*AR1+0x10`), `0x8000` = masque. Le gate n'est pas derriere nous, il est simplement **a 0** — le bon cas |
| les 11,7 M lectures a `0xde86` polluent tous les compteurs | ✗ `ld *(0x098c),A ; bc 0xddf5, ANEQ` = **attente legitime** sur `d_backgnd_st` (coherent avec « 0x098a/0x098c = background, red herring »). Le ratio 5000:1 decrit un DSP qui **attend faute de tache**, pas un DSP casse. **Compteurs natifs interpretables tels quels**, pas besoin de reprofiler |

### 12.5bis ⚠️ LE RENDU DU DESASSEMBLEUR EST LE 1er PRODUCTEUR D'ERREURS DU PROJET

**Devant le DSP lui-meme.** Le 30/07, **3 fausses pistes sur 3** viennent de la.
`tools/tic54x-dis.py` imprime les mots supplementaires **comme des operandes**, sans
les distinguer :

| affiche | reel | cout |
|---|---|---|
| `bitf *AR1(0x0010), #0x0010` | masque = **`0x8000`** (bit 15) ; `0x0010` est l'offset | conclusion « B_TASK_ABORT derriere nous », fausse |
| `sth *AR4+, A` | `STH src, **ASM**, Smem` (variante a decalage, `calypso_c54x.c:10528`) | 2 runs perdus ; avait **deja** coute une fausse piste le 28/07 |
| `banz *AR6(0x0001), 0x0001` | MOD 0xC : `0x0001` = **lk**, cible = mot suivant, **`0xde46`** | priorite #1 injustifiee |

**Correctif utile** : pas un audit opcode par opcode (`RAPPORT_OPCODES.md` compte ≥ 16
familles qui confondent 1-mot/2-mots), mais **cesser d'imprimer les mots
supplementaires comme des operandes** — les marquer `+lk` / `+pmad` / `+mask`, ou
laisser les mots bruts. Une demi-journee ; trois heures recuperees sur le seul 30/07.
**Regle en attendant** : avant de conclure « cette instruction fait la mauvaise
chose », verifier la forme exacte de l'opcode **dans `calypso_c54x.c`**, jamais le seul
rendu.

### 12.6 MES CONCLUSIONS RETIREES LE 30/07 — a lire avant de refaire le chemin

1. **« Le kernel `0xa076` n'est jamais atteint »** — FAUX. Les sondes impriment
   `0xa077` ; la chaine `0xa076` n'apparait jamais dans le journal. Mesure reelle :
   **1503 executions**, op `0x5a85` (famille MAC), `SHADOW-DADST` 1540 lignes.
2. **« Aucun `DSP>WR 0x08f8` »** — FAUX : **115**. (Mais ce sont des remises a zero
   du prologue, pas une publication : `0xb2cc` = `st #0x0000`.)
3. **« `0xa539` n'est jamais execute »** — FAUX : **1243 fois**.
4. **« La reference du correlateur est degeneree {0,-1}, |DC|/rms = 0,60 »** —
   RETRACTE, et c'est l'erreur la plus couteuse (deux runs). Cinq suspects examines
   et tous innocentes : `0x8694` est **`STH src, ASM, Smem`** (la variante A
   decalage, `calypso_c54x.c:10528-10545` — et le fichier note que cette confusion
   avait DEJA coute une fausse piste le 28/07, le desassembleur affichant
   `sth *AR4+, A` **sans** l'operande ASM) ; `ASM=-12` est pose **explicitement par
   la ROM** (`0x9f9e ed14` = `ld #0x14, ASM`), mesure au bon PC ; les deux `sfta`
   (`0xf468`=+8, `0xf478`=-8) s'annulent ; `0x14ce` = `LD Smem,**TS**,A`. Et surtout
   **`0x2a00` est ici une table POLYPHASE generee**, pas une forme d'onde recue
   (`AR3` +0x13, `AR5` +0x1d par tour, `T=(phase>>12 & 0xf)+0x10`) : ses premieres
   entrees sont la phase basse (d'ou les zeros), l'ensemble du dump est a **77,8 %
   non nul**. Comparer ses statistiques a celles d'une FCCH attendue est une
   **erreur de categorie**.
5. **Deduire un chiffre d'une sonde posee ailleurs** : `ASM=-12` venait de
   `SHADOW-DADST @0xa077`, `d[0x3fde]=1` de `AFC-GATE` a un PC de boot,
   `d[0x3f92]=0` de `CYCLE-TRACE @0xa53c` **apres** l'effacement par `0xb339` (le
   mot prend en realite 0x2000 et 0x0008). Trois fois la meme faute.

### 12.7 Outillage ajoute / corrige le 30/07

- `CALYPSO_DPAGE_HUNT` (`calypso_trx.c`, defaut 0, documente `debug.env`) : (a)
  offset `0x01A8` exactement, **toutes valeurs** (ce que `WR-OP` rate avec son filtre
  `!=0`) ; (b) balayage de la valeur `0x0002`/`0x0003` **n'importe ou** dans la
  fenetre API, replie par offset (32 max) + bilan tous les 2000 coups.
- `CALYPSO_XPCWATCH` (defaut 0) : changements de banque programme + **bilan
  periodique du masque des pages vues**, pour que l'ABSENCE soit un resultat lisible
  et non une deduction a partir d'un journal vide.
- `CALYPSO_FORCE_VEC` / `_PERIOD` (bequille, `crutches.env:61`) : injecte un vecteur
  d'IT DSP, seulement quand le handler FB est reellement arme, plafond 200.
- `DEMODIO` : **`ASM` et `ST1` ajoutes** a la ligne (les instructions qui posent ASM
  ne font aucun acces memoire, donc la sonde ne les voyait pas).
- `CYCLE-TRACE` **corrige** : la ligne imprimait `data[0x0810]` EN DUR alors que le
  `BITF` lit `*AR1(0x0010)`. Tant que `AR1` restait latche a `0x0800` les deux
  coincidaient ; depuis le correctif de page, `AR1` alterne et la cellule testee est
  `0x0810` **ou** `0x0824`. Mesure apres correctif : 559 lignes sur `0x0810`, **98 sur
  `0x0824`** — l'ancienne version affichait donc la mauvaise cellule une fois sur sept.
- `43-mailbox-dissam.sh` : **renumerote** depuis `37-`. Il declarait
  `MOD_DEPS=qemu` tout en passant AVANT `40-qemu.sh` → dependance jamais satisfaite →
  **SKIP systematique** depuis qu'il a cette dependance. Il ne pouvait jamais demarrer.
- ⚠️ `corr_iq.py` **n'existe plus** (cite dans un commentaire, absent des trois
  arbres). Metriques du dump recalculees a la main ; a reecrire dans `tools/`.

### 12.9 ECHELLE DE FIDELITE DES MODES — chaque barreau est un point de substitution

> Cadrage du 30/07. **Mon cadrage precedent etait A L'ENVERS** : je presentais
> l'hybride `native + shunt_legit` comme « impur » alors qu'il est **plus fidele que
> `shunt_legit` pur**. Ce qui suit remplace cette lecture.

| barreau | qui fait le FB/SB | qui fait les SI | le DSP tourne ? | fidelite |
|---|---|---|---|---|
| `shunt_legit` pur | gr-gsm, et `rx_fb_det=1` **substitue** | gr-gsm → `feed_si` → `a_cd` | **NON** (`DSP_SHUNT=1` court-circuite) | la plus faible |
| **hybride `native`+`SHUNT_LEGIT=1`** (ce qui tourne) | gr-gsm substitue encore le **resultat**, mais le DSP moud de vrais echantillons | gr-gsm → `a_cd` (4938 injections mesurees, `have[0-5]=111100`) | **OUI**, dispatch + correlateur exerces | **« gr-gsm, un cran au-dessus »** |
| **`native_twl`** (existe depuis le 30/07) | hote/TWL : le **resultat** FB/SB reste substitue (`REAL_FB` + `INJECT_SB` + `PUBLISH_FB`) → `data[0x08f8]` n'y est **PAS** un verdict | **le DSP** : `FEED_SI=0`, `INJECT_ACD=0` — aucune porte gr-gsm vers `a_cd` | OUI | **un cran au-dessus de l'hybride** : meme substitution FB/SB, mais le SI n'est plus substitue |
| `native` pur | tout par le DSP | tout par le DSP | OUI | maximale, **pas atteignable** aujourd'hui (trou `0x43d8`) |

**Le point qui rend l'echelle juste** : le TWL3025 **est deja applique en natif**, sur
l'ENTREE. `calypso_bsp.c:1595` fait `calypso_twl3025_apply_phase(sl->iq, …)` dans le
chemin de livraison, juste avant `c54x_bsp_load()`, **hors de toute garde de shunt**.
Donc le correlateur natif recoit des echantillons corriges en AFC/phase. Ce que
`SHUNT_LEGIT` ajoute n'est PAS le front analogique — c'est le **resultat** :
`g_shunt.rx_fb_det = 1` ecrit dans `api_ram[0x08F8..0x08FD]` depuis la SCH decodee par
gr-gsm (`calypso_dsp_shunt.c:719`, `@BEQUILLE` : « masque : le correlateur natif qui ne
pose jamais d_fb_det »).

C'est pour ca que l'hybride est **plus** fidele : le DSP fait le travail sur du signal
correct, et seul le resultat reste substitue. En `shunt_legit` pur, le resultat est
substitue **et** le DSP est court-circuite.

**Deux categories a ne plus confondre** (c'est la distinction « wire-only vs implement »
de l'audit du 26/07) :
- **supplee un bloc absent** — TWL, transport, `feed_si`/`a_cd` : tient la place d'un
  materiel ou d'un decodage que le DSP ne fait pas encore. Legitime, et necessaire pour
  mesurer autre chose.
- **ecrase un resultat** — `SHUNT_REAL_FB`, `rx_fb_det=1`, `INJECT_SB` canne : recopie
  une valeur par-dessus celle que le DSP aurait produite. C'est CELA seul qui interdit
  de juger le correlateur, et seulement sur la cellule concernee.

**Regle de lecture, unique** : `data[]` = ce que le DSP a ecrit ; `api[]` = ce que
l'hote a ecrit. La ligne `[fbsb]` les separe expres. Aucun jugement sur le correlateur
ne se prend sur `api[]`.

**`native_twl` : le contrat, corrige le 30/07.** J'avais construit ce profil A
L'ENVERS — hote pour les SI, DSP pour le FB/SB. L'utilisateur l'a repose en deux
lignes, et c'est celui-ci qui fait foi :

```
native_twl  ==  FB/SB = TWL (hote)   |   SI = DSP
native      ==  FB/SB = DSP          |   SI = DSP
```

Le partage n'est donc pas « analogique vs numerique » mais **acquisition vs
decodage** : on donne au DSP une synchro deja acquise, et on lui laisse tout ce
qui vient apres. Raison d'etre, mot pour mot : *« je veux voir depuis 3 jours si
le DSP traite le SI vu qu'on n'arrive pas le FB/SB »*. Le FB/SB natif est bloque
(N4 = 0 sur tous les runs depuis le 28/07) ; ce banc pose la question suivante
sans attendre que celle-la se debloque.

**Frontiere, formulee par l'utilisateur** : *« si SI vient de grgsm == shunt_legit
ou shunt, on shunte le DSP »*. Les deux seules portes par lesquelles un bloc
gr-gsm entre dans `a_cd` sont `CALYPSO_SHUNT_FEED_SI` (`calypso_dsp_shunt.c:2249`,
qui dit lui-meme « Le SI vient de gr-gsm ») et `CALYPSO_INJECT_ACD`
(`calypso_dsp_helper.c:375`). Le profil les pose a 0 ; `01-profil.sh` proteste si
on les rallume sous un profil natif — le garde-fou precedent surveillait
`PUBLISH_FB`/`REAL_FB`, c'est-a-dire l'ancien contrat, inverse.

**Critere du mode, et il existait deja** : `CALYPSO_WATCH_ACD=1`
(`calypso_c54x.c:2563`) trace les ecritures **opcode** dans `data[0x09D2-0x09E0]`
= `a_cd`, plafonnee a 60 lignes. Une ecriture opcode est du DSP ; une ecriture
directe du shunt n'en est pas — c'est exactement ce que la sonde distingue. Le
profil la pose lui-meme. Corollaires : `[fbsb] ALLC task=24 — real DSP CCCH demod`
(`calypso_fbsb.c:85`, non gatee) prouve que l'ARM a confie le CCCH au DSP, et
`CALYPSO_DEBUG=BSP,A_CD-BY-BURST` donne les totaux correles au burst.

**Ce qui manquait est fait** : l'ecriture `rx_fb_det = 1` est decouplee du
parapluie par `CALYPSO_SHUNT_PUBLISH_FB` (bloc `calypso_dsp_shunt.c:712-744` et
interception de lecture `:1684`). C'est ce decouplage qui rend les deux moities du
contrat reglables separement.

### 12.8 Runtime : le banc a demenage

Le binaire qui tourne est **`/opt/GSM/osmo-qemu-calypso/build/qemu-system-arm`** (et
non `qemu-src`, fige au 28/07). ⚠️ `paths.env:45` prend la copie du firmware
**livree avec le depot** des qu'elle existe : on recompile `osmocom-bb` et le run
charge quand meme l'ancien ELF, **sans un mot** (`CALYPSO_FIRMWARE_ELF` vide au
manifeste ne trahit rien). Diagnostic :
`ps -C qemu-system-arm -o args= | grep -o '\-kernel [^ ]*'`. Corrige par les
**QUATRE** variables `FIRMWARE_*` dans `environnement/local.env` — poser
`FIRMWARE_DIR` seul ne suffit pas, la premiere branche de `paths.env` construirait
le chemin sans le `board/` de l'arborescence de build.
⚠️ `BUILD-STAMP` du manifeste **ne bouge pas** quand seul `calypso_c54x.c` est
recompile : ne pas s'en servir pour verifier qu'une sonde est dans le binaire — le
seul test fiable est **sa ligne d'armement**.
⚠️ **Ne jamais lire la fenetre API depuis le monitor QEMU** : `x /1xh 0xffd001a8`
**tue QEMU sur le coup** (process disparu, aucune ligne dans `qemu.log`).
`calypso_dsp_read()` appelle `calypso_mbx()` a chaque lecture ARM ; hors thread CPU
ca ne tient pas. La RAM ordinaire (ex. `0x0082fca8`) est sans risque.

---

## 11. ETAT AU 2026-07-29 — le natif deverrouille, un maillon restant

> Cette section PRIME sur les precedentes en cas de conflit. Chaque fait nomme
> son instrument. TODO actionnable : `TODO.md` a la racine.

### 11.1 Trois verrous de juillet sont tombes — MESURE

| verrou | etat au 28/07 | etat au 29/07 | instrument |
|---|---|---|---|
| storm / tremplin mask-ROM | `AR7=0x4387` -> `0xab38` (`RET`) | **`AR7=0x43c0` -> `0xa4c7`** | sonde `BACC-DISP` : `<<REAL>>` |
| IT trame jamais servie | `INTM-TRANS` = 2, `IMR=0x3000` | **1298 transitions, `IMR=0x52ed`** | `INTM-TRANS`, `IMR-ARM` |
| `0xb01e` (CALA dispatcher) | « jamais atteint » | **29 passages** | `CALA-WIDE` |
| correlateur | `COEFFS-WR` = 0 | **549** | `COEFFS-WR` |

**Cause** : deux gates, et une seule ne suffit pas.
- `CALYPSO_LDK8_SHIFT16=0` — correctif ISA de `LD #k8u` (immediat en bits BAS).
  Desormais **defaut** dans `environnement/opcodes.env`. Non-regression
  `shunt_legit` PASSEE (camp + LU + TMSI + 122 SMS, 18:13).
- `CALYPSO_FRAME_IT_NATIVE=1` — remap IT trame `vec19/bit3` -> `vec28/bit12`.
  Ce n'est PAS une bequille : `calypso_trx.c:1495` levait l'IT sur un bit que le
  DSP n'arme pas. Pose par les modes `native` et `shunt_legit_no_inject`.
  ⚠️ Le nom trompe : aucun rapport avec `CALYPSO_MODE=native`.

**PERIME par ces mesures** : « le DSP n'atteint jamais `0xb01e` » ; « BRINT0
masquee = racine » (le bit 5 s'arme seul une fois l'IT trame cablee) ;
« `d[0x098b]`/`d[0x098d]` = racine » (boucle de fond, pas le chemin de taches) ;
« `0xa582` ecrit IMR=0 et ecrase l'arm » (c'est un **OR**, il ne peut rien
effacer).

### 11.2 Le maillon restant — HYPOTHESE forte, une seule cellule

`data[0x43d8]` est le **slot du handler de tache** : relu a chaque trame par
`0xb01c`, appele par `0xb01e`. Il ne contient jamais que `0xab38` (= `RET`),
pose une fois par `0xbb00`. Verifie statiquement (2 references dans les 5 ROM)
ET au runtime (sonde dans `data_write`, elle verrait un `stl *ARn` indirect).

Sequence par trame, desassemblee :
```
0xb001..0xb017  ld *AR1(n),A / stl *AR2(n),A   ECHO commande -> reponse
0xb01c          ld  *(0x43d8), A               handler de tache
0xb01e          cala A                         ... qui vaut RET
```
Le DSP renvoie donc une reponse **bien formee et vide**, chaque trame.

**Cascade mesuree** : ARM commande FB (26x `d[0x0804]=5`) -> DSP acquitte
(`d_task_d`/`d_burst_d` ecrits ~4300x) -> handler = `RET` -> `d_fb_det` = 0 ->
correlateur moud du vide -> banque `0x7700-0x79F0` : 0 exec -> mobile ne campe pas.

### 11.3 Deuxieme instance du meme motif : la file `0x4330`

Producteur `0xaa75`, consommateur `0xaa87`, anneau de 14 (`BK=14`), pointeurs
`0x433e` (lecture) / `0x433f` (ecriture). Le pointeur de lecture **n'avance
jamais** ; l'anneau sature, l'ecriture rattrape la lecture, le producteur leve
`DSP_ERR_DMA_PROG` (bit 3 de `0x3f92` -> `d_error_status` = `0x08d5`).

**`MVDM`, `MAR *ARn-0` et `BANZ` sont CORRECTS** — soupconnes tous les trois,
disculpes par la sonde `CALYPSO_DMAQ` (registres releves au PC du test).
⛔ **Ce n'est PAS le DMA materiel** : 0 acces aux MMR `0x0054`-`0x0057`, aucune
ecriture dans PROM0. File **logicielle**.
Comparer : la file `0x4340` est equilibree (863/863) — le mecanisme fonctionne
ailleurs.

### 11.4 Correctifs d'ISA de la journee

| # | correctif | effet mesure | bascule A/B |
|---|---|---|---|
| 1 | `LD #k8u` : immediat en bits bas | tremplin -> go-live, fin du storm | `CALYPSO_LDK8_SHIFT16=1` |
| 2 | base du tampon circulaire : masquage des N bits bas au lieu de `ar - (ar % bk)` | les anneaux ne se chevauchent plus | `CALYPSO_CIRC_BASE_MOD=1` |

Sur (2), le calcul historique placait un anneau de base `0x4340` en
`0x4336..0x4343` — **a cheval sur `0x433e`/`0x433f`**, les pointeurs de l'anneau
voisin, qu'il ecrasait.
⚠️ **Dette** : (2) n'a pas encore ete passe en non-regression `shunt_legit`.

### 11.5 Outillage ajoute (et ce qu'il evite)

- `calypso_mailbox.c` — moniteur COMPLET de la mailbox, **actif par defaut**,
  `$LOG_DIR/mailbox.log`. Quatre sens dont **`ARM<RD`**, jamais instrumente
  avant. Fichier separe (jamais stderr), repliement `x N`, format **diffable
  entre deux runs**.
- `tools/mailbox-annote.py` (+ module `43-mailbox-dissam.sh`) — croise le journal
  avec `tools/tic54x-dis.py` : quelle instruction touche quelle cellule.
  `mail_dissam.log`, rafraichi toutes les 2 s, fenetre tmux `dsp` et `asm`.
- `tools/mailbox-gdb.py` — instantane de la mailbox via le gdbstub QEMU.
  ⚠️ arrete le guest ; ne voit que la fenetre API RAM ; exige `gdb-multiarch`.
- `run.sh --reset` / `--restart` — le serveur tmux **fossilisait** les
  `CALYPSO_*` du premier run : deux runs de la meme ligne divergeaient. Corrige.
- `calypso_dma.c` — controleur DMA c54x, **inerte par defaut** (`CALYPSO_DMA`).
  Comble un vrai trou mais adresse une etape que le firmware n'atteint pas.

### 11.6 Modes

| mode | pour quoi | ce qu'il pose |
|---|---|---|
| `native` | le DSP seul, rien pour l'aider | `LDK8=0` `FRAME_IT_NATIVE=1` `DSP_RUN_C54X=1` |
| `shunt_legit_no_inject` | FB/SB au TWL, **SI au DSP** | + `INJECT_SB=1` `NO_CANNED=1` `DSP_SHUNT=0` |
| `shunt_legit` | la reference qui campe (LU + SMS) | parapluie large, DSP eteint |

⚠️ `shunt_legit_no_inject` posait `SHUNT_NO_LEGIT=1` dans sa premiere version, ce
qui chargeait `calypso_shunt_no_legit.env` et donc `DSP_SHUNT=1` — **le mode
censé garder le DSP natif le court-circuitait**. Corrige.

---

## 0. Verifier ce qu'on mesure AVANT de mesurer

Trois pieges ont produit de fausses conclusions et sont desormais des prerequis.

| Piege | Ce qui est faux | Instrument correct |
|---|---|---|
| **`BUILD-STAMP` ne date PAS le binaire** | la macro `__DATE__/__TIME__` vit dans `calypso_dsp_shunt.c:107` : elle date **son propre TU**. Un run peut afficher `09:42:13` avec un binaire relie a `11:37`. | `stat -c %y build/qemu-system-arm` + `find build -name '*calypso_c54x*.o' -printf '%T+ %p\n'` + `ps -eo pid,lstart,cmd \| grep qemu-system-arm` |
| **Artefacts /dev/shm perimes** | `daram_2a00.cfile` peut dater d'un run precedent et etre lu comme la sortie du run courant | comparer le mtime au `lstart` du process ; `rm -f /dev/shm/daram_2a00.cfile` AVANT le run |
| **Compteurs plafonnes par le logger** | `calypso_dsp_shunt.c:1670` : `if (rfl < 20 \|\| (det && rfl < 300))`. « 280 det sur 300 » = contenu des 300 premieres LIGNES, pas un taux de detection | lire le code du logger avant de citer un ratio |

**Variables effectives ≠ variables posees.** `CALYPSO_SHUNT_LEGIT` est une *value-list*
resolue par un constructeur `__attribute__((constructor))` (`calypso_dsp_shunt.c:86-105`)
qui fait `setenv(..., overwrite=1)` : `DSP` force `CALYPSO_DSP_RUN_C54X=1` meme si le
`.env` a pose `0`. Instrument : le `[calypso-manifest]` du run, ou
`tr '\0' '\n' < /proc/<pid>/environ | grep ^CALYPSO_`. Un `env | grep` cote shell ment.

---

## 1. Statut par mode

> **Il n'y a PAS de statut absolu — le statut depend du MODE de run.** L'ancienne
> "contradiction" (« LU ACCEPT recu » vs « FB jamais detecte ») etait DEUX MODES
> CONFONDUS : la LU marche en shunt, elle est bloquee en natif.

### Les modes (socle `calypso.env` + un `calypso_X.env`)

- **`SHUNT_LEGIT=1`** — base. FBSB host-side (`real_fb` + gr-gsm -> intercept de LECTURE
  ARM), cannes de stabilisation, DSP c54x OFF. **MODE FIABLE.**
- **`SHUNT_NO_LEGIT=1`** — memes injections, nommees une par une, sans le parapluie.
- **`SHUNT_LEGIT=DSP,NO_CANNED`** (value-list) — DSP c54x tourne en // et sans cannes.
  Plus proche du reel, plus flaky.
- **`CALYPSO_NATIVE=1`** (NATIF) — corrélateur DSP, entree par intercept de lecture
  `data[0x9213]/[0x9215]` (`FB_STREAM`), `CORR_ENTRY=0x9500` dans le `.env` livre.
- **`CALYPSO_NATIVE_HELPED=1`** (NATIF AIDE) — corrélateur DSP + `feed_iq` en DARAM
  (`FB_IQ_BASE=0x9210`) dans le `.env` livre.
- **`CALYPSO_L1=c`** — **MODE MORT**, non executable : `calypso_dsp_shunt.c:1847` arme le
  shunt sur `l1_c_active()`, et `calypso_trx.c:1475` exige
  `l1_c_active() && !shunt_active()`. Les deux ne peuvent pas etre vraies ensemble ;
  `calypso_layer1_tick()` n'a **aucun** appelant atteignable. Ni HLE, ni natif.
  (DEDUCTION DE CODE, non testee au run.)
- **`./start-clean.sh` sans profil** — mode degenere : `run.sh:1137` pose
  `CALYPSO_DSP_SHUNT=1` (preset `full-grgsm`) -> `substitutes()=vrai` -> le c54x **ne
  tourne pas** malgre `RUN_C54X=1`, et **aucune** injection n'est armee. Ne campe pas et
  n'exerce pas le natif. (DEDUCTION DE CODE.)

**Les deux profils natifs livres posent encore `CALYPSO_FB_CORR_ENTRY=0x9500`**
(`calypso_native.env:16`, `calypso_native_helped.env:11`), alors que la configuration
mesuree correcte est `0x94f5` (§3, M6). Ecart connu, corrige en CLI par le run de
reference §9. Defaut non modifie.

### Matrice statut x mode

| Feature | SHUNT_LEGIT=1 | SHUNT_NO_LEGIT | DSP,NO_CANNED | NATIVE / NATIVE_HELPED | Instrument |
|---|---|---|---|---|---|
| FB/SB sync | **DONE** (host, `coh=0.999 dphi=0.387 det=1`) | DONE | DONE | **KO** : le corrélateur tourne, `d_fb_det[0x08f8]=0` | `REAL-FB fn=` / `DETECTOR-RUN` |
| rxlev serving | DONE (-47 dBm) | DONE | DONE | DONE mais **mocke** : `shunt_dispatch_pm` (`calypso_dsp_helper.c:673`) n'a **aucun gate positif** | `CALYPSO_TRF_RXLEV`, `MON: f=` |
| Camp (C3) + sysinfo | **DONE** (20 SI decodes) | DONE | DONE | **KO** « No sysinfo », `BSIC=0` | `grep -c sysinfo /root/mobile.log` |
| LU + registration | **DONE** (LU ACCEPT `lai=001-01-1`, TMSI `0x3dbeb85f`, 2,70 s RACH->ACCEPT) | DONE | DONE | KO (pas de camp) | `grep -icE "LOCATION UPDATING ACCEPT"` |
| SMS MO / MT | DONE | DONE | **WIP flaky** (anti-stall ajoute) | KO | log SMSC |
| Ctrl-C mobile -> re-acquisition | DONE (re-arm BGEN sur `d_dsp_page=0`) | DONE | DONE | non teste | `calypso_arm2dsp.c`, hook `on_arm_write` |
| Voix TCH/F | **WIP** : ASSIGNMENT COMMAND -> ASSIGNMENT FAILURE | WIP | WIP | KO | `doc/VOIX_PLAN.md` |
| c54x execute a la cadence trame | non | non | **oui** (runner shunt) | **oui** (tick TDMA) | `dsp_n_exec_2/5`, absence de `DSP Error Status: 2048` |
| **Entree du demod alimentee** | s.o. | s.o. | non mesure | **DONE** : `data[0x4c00]` lu, valeurs reelles `ff6e`, `c1fb`, `d147` | `CALYPSO_WATCH_9F00_RD` (PC=0x9fb5) |
| **Sortie du demod exploitable** | s.o. | s.o. | non mesure | **KO** : DC quasi pur et fige, `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` | `CALYPSO_DARAM_DUMP` + `corr_iq.py --src ddump` |

**Reserve sur la robustesse LU (non tranchee).** `SHUNT_LEGIT_ADDRESS_MAP.md` §9 conclut
« LU ACCEPT INTERMITTENT (~1 succes / 19 retries T3211) » alors que `run_results.md` Run A5
mesure « 2,70 s, 0 retry ». A5 est plus recent et posterieur au fix sous-voie SDCCH/8, mais
n=2. **A rejouer** : `grep -c T3211 /root/mobile.log` sur 5 runs `SHUNT_LEGIT` consecutifs.

### ⚠️ Hygiene de mode : ce qui masque le natif a l'observateur

Les deux `.env` natifs posent `CALYPSO_DECAN:=1`, qui **implique `SHUNT_REAL_FB`**
(`calypso_dsp_shunt.c:1468` et `:1594`). L'intercept
`calypso_dsp_shunt_real_fb_read()` (`:1463-1490`, appele par `calypso_trx.c:297`) sert
alors a l'ARM le resultat du detecteur **hote** sur `0x01F0/F4/F6/F8/FA` et `0x0060/0x0088`,
pas la cellule DSP.

Consequences, a retenir :
1. `d_fb_det = 0` **lu cote ARM** en natif est **ambigu**. Seule la cellule DSP
   `data[0x08f8]` (imprimee par `DETECTOR-RUN`, ou la sonde `CALYPSO_FBDET_API` qui lit les
   deux cotes du miroir) tranche.
2. Le comportement du mobile dans un run natif avec `REAL_FB=1` **ne mesure pas le natif**.
   Pour un natif nu : `CALYPSO_DECAN=0 CALYPSO_SHUNT_REAL_FB=0` en CLI (l'idiome `:=`
   le permet sans toucher aucun defaut).
3. Le **PM est mocke dans tous les modes**, y compris natif (`shunt_dispatch_pm`,
   `calypso_dsp_helper.c:673`, seul `CALYPSO_SHUNT_NO_FAKE_PM=1` le coupe).

### Chaine operationnelle (la SEULE voie FBSB qui campe — modes shunt)

```
twl3025/DECAN (AFC/PM/rxlev, gain trf6151)
gr-gsm + detecteur FCCH host (coherence + dphi)
        -> g_shunt.rx_*
        -> real_fb_read  (INTERCEPT DE LECTURE MMIO, calypso_trx.c:297)
        -> ARM osmocom (0xFFD001F0 = d_fb_det, etc.)
        -> camp + Location Update
```

**Le « fake FB » est mort dans tous les profils livres** : `CALYPSO_INJECT_FB` n'est pose
par aucun `.env` (`grep -rn INJECT_FB *.env` = 0 hit). Ce que `SHUNT_LEGIT` fait n'est pas
d'ecrire un faux resultat dans `data[]`, c'est d'**intercepter la lecture ARM**.
Corollaire operationnel : **aucun watchpoint sur `data_write_locked` ne verra jamais cette
livraison**.

**Oracle reseau.** La pile temoin `bts1` (osmocom-bb sur `trxcon` + `fake_trx`, sans QEMU)
obtient un LU ACCEPT sur le meme coeur (`grep -c "LOCATION UPDATING ACCEPT"
/root/mobile-bts1.log` = 1). Tout echec cote Calypso est imputable a l'emulation, pas au
coeur Osmocom.

---

## 2. Architecture reelle

Deux CPU emules, colles par MMIO API RAM :
- **ARM946** (`calypso_mb.c:303`, coeur ARMv5TE de QEMU) execute le firmware osmocom-bb.
  Il **modelise le vrai ARM7TDMI** (ARMv4T) du Calypso : v5TE est un sur-ensemble de v4T.
  Quand un autre doc dit « ARM7TDMI », il parle du silicium modelise, pas du coeur QEMU.
- **TMS320C54x** execute la vraie mask-ROM Calypso (TI).
- Colle : `calypso_trx.c` (miroir MMIO) + `calypso_arm2dsp.c` (pont par instruction).

Loi d'adressage (invariante) : `DSP_word = 0x0800 + (ARM_off - 0xFFD00000)/2`. L'ARM lit
les resultats DSP dans `s->dsp->data[off/2 + 0x0800]` — **PAS** dans `dsp_ram[]` ni
`api_ram[]` directement. C'est pourquoi les `shunt_dispatch_*` passant par
`dma_memory_write` sont invisibles ; seules les ecritures directes `data[]`/`api_ram[]`
campent.

**Le correlateur DSP natif est un VRAI correlateur** (mask-ROM TI), PAS un stub.
Sur silicium, son buffer d'echantillons est rempli par le recepteur on-chip (DRP -> DMA),
**non modelise en QEMU** — c'est ce que le BSP remplace aujourd'hui (§3).

Cote osmocom : `prim_fbsb.c` pose `d_fb_mode` puis polle `d_fb_det` ; `dsp_api.h` ne
contient QUE des cellules RESULTAT — **aucun buffer IQ dans l'API RAM**. L'ARM ne fournit
jamais l'IQ.

`calypso_fbsb.c` = logger MORT (log-only, aucune synthese). Ne pas s'y fier.

---

## 3. Chaine de signal RX — etat maillon par maillon (MESURE 2026-07-28)

```
[osmo-bts-trx] --TRXDv0 UDP :6702--> M1 bsp_trxd_readable
   M2 decimation /4 -> 148 paires @1 SPS      G1 gate shunt  calypso_bsp.c:474
   M3 DIRECT_FEED -> rx_burst                 G2 gate shunt  calypso_bsp.c:997
   M4 ecriture data[DARAM_ADDR]               G3 gate shunt  calypso_bsp.c:1359 (LIVRAISON)
   M6 dispatch CALA @0xb01e -> reroute FB_ENERGY -> FB_CORR_ENTRY (0x94f5)
   M7 etage demod 0x9f95..0x9fe2  -- LIT data[0x4c00] en pas de 5
   M8 data[0x2a00..0x2b27] = WORKZONE DE SORTIE (0x9fb8 = I, 0x9fe2 = Q)
   M10 noyau MAC 0xa076..0xa09d
   M11 detecteur 0x9ac0 (boucle 0x9ab8..0x9ac2)
   M12 d_fb_det = data[0x08f8] = api_ram[0xF8] = MMIO ARM 0x01F0
```

| # | Maillon | Etat | Instrument |
|---|---|---|---|
| M1 | UDP TRXDv0 -> BSP | **OK** | `CALYPSO_DEBUG=BSP` (`RXSZ`, `RX tn= fn=`) |
| M2 | Decimation 4 SPS -> 1 SPS | **OK, conforme** : `IQ_DECIM=4`. Corollaire `DARAM_LEN=296` | `corr_iq.py --src bursts` -> `VERDICT: FCCH @1SPS PROPRE (dphi=+1.00x pi/2)`, `coh=0.998`, `rms=3.25e4`, `\|DC\|=379`, `zeros=0%`, FFT +67 708 Hz |
| G1-G3 | **Trois** gates BSP (pas deux) | **LEVES** par `CALYPSO_BSP_DARAM_FORCE=1`. G3 (`:1359`) est la **LIVRAISON** vers `data[]` ; il ne connaissait que `TPU_RX_WIRE` — aligne le 2026-07-28 sur `RUN_C54X=1 && DARAM_FORCE`, **defaut inchange**. Avant ce fix : 2 verrous ouverts sur 3, rien n'arrivait au DSP | `[bsp] deliver: gate shunt LEVE (rxw=1)` present ; `grep -c "dropping fn="` = **0** |
| M3 | Chemin de livraison | `DIRECT_FEED=1` court-circuite `bsp_enqueue`/`deliver_buffered` : la fenetre FN ±64 et le pulse BDLENA **ne sont pas** dans le chemin actif | manifeste `CALYPSO_BSP_DIRECT_FEED` |
| M3b | `c54x_bsp_load` -> `bsp_buf[]` | **BRANCHE MORTE** : seul consommateur = `PORTR PA=0xF430` (gate `CALYPSO_FIX_PORTR`) ; mesure `PORTR-ANY` = PA ∈ {`0x0003`, `0xfc28`}, **jamais** `0xF430`/`0x0034` | `CALYPSO_DEBUG=PORTR-ANY` |
| M4 | Ecriture DARAM | **OK** a `DARAM_ADDR=0x4c00`, `LEN=296` | `DMA fn= -> DARAM[0x....]` (exige `CALYPSO_DEBUG=BSP`) |
| M5 | Reveil DSP (INT3 vec19, BRINT0 vec21) | **hors chemin critique** : le handler `0x8d00 -> 0xa076` est du **polling pur** sur `data[0x3fad]` bit15 | `SPIN-IT`, `CALYPSO_PROBE_3FAD_GATE` |
| M6 | Dispatch FB | cible **native** de la `CALA @0xb01e` = `0xab38` = **stub `RET`**. Le reroute `CALYPSO_FB_ENERGY=1` amene au corrélateur energie a `FB_CORR_ENTRY` | `FB-ENERGY-REROUTE CALA@0xb01e tgt 0xab38 -> 0x94f5` |
| M7 | **Entree du demod** | **VIVANTE** : PC `0x9fb5` lit `0x4c00/05/0a/0f/15/1a` (**pas de 5** = polyphase 6 taps) avec des valeurs reelles `ff6e`, `c1fb`, `d147` | `CALYPSO_WATCH_9F00_RD` |
| M8 | **Sortie du demod `0x2a00`** | **MORTE** : DC quasi pur et **fige**, `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` ; cellule temoin invariante sur 157-203 bursts (`0x9fb8@0x2a00=0x0000`, `0x9fe2@0x2a00=0x52ed`) — **identique avec `DECIM=1` et `DECIM=4`** | `CALYPSO_DARAM_DUMP` (ddump, non-racy) + `corr_iq.py --src ddump` |
| M9 | Scratch `0x2c00` | `0x9fd5` y depose une table de coefficients **CONSTANTE** ; `0xa03f-0xa042` = `LD #0xe000,A` + `RPT #14` + `STL A,*AR5+` = remplissage constant | `CALYPSO_WZWRITE`, `CALYPSO_WMAP` |
| M10 | Noyau MAC `0xa076..0xa09d` | 60 000 ecritures pour **4 a 7 valeurs distinctes**, toutes dans `{0001,0002,001f,003e}` ± bit `0xe000`. `003e=2x001f`, `0002=2x0001` : un accumulateur qui double une constante, **pas une correlation** | `CALYPSO_WMAP` (LO/HI sur `0xa07x`) |
| M11 | Detecteur `0x9ac0` | **ARME et il tourne** : `DETECTOR-RUN #3600 d_fb_mode[08f9]=0x0001` | log `DETECTOR-RUN` (inconditionnel) |
| M12 | `d_fb_det` | **0** sur la totalite des runs observes (3 600 puis 32 200 executions du detecteur). Publisher natif **unique** = `ORM #1,*(0x08f8)` @`0x79e4`, banque commune `0x7700-0x79F0`, **jamais executee** | `CALYPSO_B4`, `CALYPSO_SCAN_08F8`, `CALYPSO_FBDET_API` |

### 3.1 L'adresse d'entree DEPEND du point d'entree du corrélateur

Ce n'est pas une constante du DSP, c'est une consequence du reroute
`calypso_c54x.c:6045-6067`. **Toujours mesurer avec `CALYPSO_WATCH_9F00_RD` AVANT de feeder.**

| Configuration | Le demod LIT | Outil qui sert | Outil INERTE |
|---|---|---|---|
| `FB_ENERGY=1` + `CORR_ENTRY=0x9500` (defaut des `.env` natifs) | `data[0x9213]` / `[0x9215]` | `CALYPSO_FB_STREAM` | `BSP_DARAM_ADDR` |
| `CORR_ENTRY=0x94f5` + `BSP_DARAM_ADDR=0x4c00` (**run de reference**) | `data[0x4c00]`, pas de 5 | BSP + `DARAM_FORCE` | `FB_STREAM` (`0x9213`/`0x9260` **jamais lus**) |
| toutes | `data[0x2a00]` = **ECRITURE** du demod | — | **ne jamais feeder** |

`0x9500` n'apparait **nulle part** dans les 28 672 mots de PROM ; `0x94f5` est l'entree
referencee en ROM (`@0x87e7 f930 94f5`) et le defaut du code (`calypso_c54x.c:6057`).
Entrer en `0x9500` saute les 11 mots de mise en place qui posent `AR6`.

**Incoherence residuelle du run de reference (HYPOTHESE, testable).** Il herite de
`calypso_native_helped.env` un `FB_IQ_DARAM=1` + `FB_IQ_BASE=0x9210`, donc `feed_iq` ecrit
`data[0x9210..0x9337]` — region **jamais lue** avec `CORR_ENTRY=0x94f5`. Ecriture
vraisemblablement **inerte** (elle consomme du CPU sans alimenter personne). Tranche :
`CALYPSO_WATCH_9F00_RD` (absence de `addr=0x921x`) ou
`CALYPSO_RMAP=1 CALYPSO_RMAP_PCLO=0x9f00`.

### 3.2 Resume en une phrase

**ENTREE VIVANTE, SORTIE MORTE.** Le signal qui entre est bon et prouve (M2, M7) ; le
demod lit du signal reel et produit du DC constant (M8) ; le noyau MAC double une constante
(M10) ; le detecteur tourne mais ne detecte rien (M11) ; `d_fb_det` reste 0 (M12). Le
defaut est **dans le calcul**, pas dans l'alimentation.

---

## 4. CIBLE N.1 — bug de decodage d'opcode `0x1800` (AND lu comme LD)

### 4.1 La chaine fautive, verifiee mot a mot contre `calypso_dsp.PROM0.bin`

```
9fa1: 7660 000f   ST  #0x000f, *(0x60)     -> data[0x0060] = 15
9fa3: 7661 0010   ST  #0x0010, *(0x61)     -> data[0x0061] = 16
...
9fb1: 1860        AND *(0x60), A           <-- LE BUG
9fb2: 0061        ADD *(0x61), A
9fb3: 880e        STLM A, MMR 0x0E = T
9fb5: 14ce        LD Smem, TS, A           (TS = T[5:0], -16..+31, SPRU172C)
9fb8: 8694        STH A, ASM, *AR4+        -> workzone 0x2a00
```

`0x1860` est un **AND** (`doc/opcodes/tic54x_hi8_map.md:26` — `0x18..0x19 | and |
0x1800 / 0xFE00`). Or `sub = (op >> 9) & 7` donne `sub = 4`, et le `switch` de `case 0x1:`
(`calypso_c54x.c:9320`) ne nommait que les sub 0..3 (`0x1000` LD / `0x1200` LDU /
`0x1400` LD,TS / `0x1600` LDR). `sub=4` tombait dans le `default` = **chargement signe**.

Consequence mesuree, instruction par instruction :

| PC | Attendu | Fait avant fix | Etat |
|---|---|---|---|
| `0x9fb1` | `A = A AND 15` | `A = 15` | — |
| `0x9fb2` | — | `A = A + 16` | `A = 31` |
| `0x9fb3` | — | `STLM A, T` | **`T = 31`** |
| `0x9fb5` | decalage borne | `LD Smem, TS` avec `TS = +31` | **`A = 0x80000000` (saturation)** |
| `0x9fb8` | echantillon filtre | ecrit l'accumulateur sature | **sortie independante des operandes = DC plat** |

L'implementation de `LD Smem,TS` (`:9344`) est **correcte** ; le bug etait **en amont**.
Coherent avec M8 (sortie plate) et M10 (constante doublee).

### 4.2 Etat du correctif

**POSE EN SOURCE, COMPILE, NON ENCORE VALIDE AU RUN.**
`calypso_c54x.c:9365-9389` implemente desormais les quatre sub manquants, Smem
**zero-etendu sur 40 bits**, conforme a l'exemple TI (SPRU172C p.4-12 pour AND, p.4-192
pour SUBC) :

| sub | encodage | mnemonique |
|---|---|---|
| 4 | `0x1800` | `AND Smem, src` |
| 5 | `0x1A00` | `OR Smem, src` |
| 6 | `0x1C00` | `XOR Smem, src` |
| 7 | `0x1E00` | `SUBC Smem, src` |

### 4.3 Second site touche par le meme bug

La boucle du **detecteur** `0x9ab8..0x9ac2` reproduit exactement le motif AND -> T :

```
9aba: 1983   AND *AR3, B      (hi8 0x19 = and, dst = B)
9abc: 890e   STLM B, MMR 0x0E = T
9abe: 348e   BITT
```

Avant le correctif, `0x1983` etait lui aussi execute comme un `LD` : le masque de bits
applique avant le chargement de `T` etait remplace par un chargement brut.
**MESURE** (decodage PROM) ; **HYPOTHESE** quant a l'effet sur `d_fb_det`.

### 4.4 Le critere de tranche — et ce qu'il ne faut PAS regarder

Un premier run post-correctif (44 s) donne encore `d_fb_det = 0` sur 3 600
`DETECTOR-RUN`. **Cela ne suffit pas a conclure** : `d_fb_det` est en bout de chaine et
depend aussi de M6/M12 (dispatch stub, publisher jamais execute).

> **Le critere est la SORTIE DU DEMOD, pas `d_fb_det`.**
> Reference a battre : `\|DC\| = 2,86e4` pour `rms = 2,94e4`, `dphi = +0,004`.
> Cible : `coh > 0,90`, `dphi ≈ +1,571` (= +1,00 x pi/2).
> Mesure : `rm -f /dev/shm/daram_2a00.cfile` avant le run, `CALYPSO_DARAM_DUMP=1`,
> puis `python3 tools/corr_iq.py --src ddump | tail -3`.

---

## 5. Blocages suivants, par ordre

| Rang | Blocage | Statut | Tranche |
|---|---|---|---|
| B1 | Decodage `0x1800/1A00/1C00/1E00` -> `T=31` -> saturation -> sortie demod constante | **corrige en source, non mesure** | §4.4 |
| B2 | **`DSP_ERR_DMA_PEND` (0x20 = 32) permanent** : `grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log \| sort \| uniq -c` -> **723 x « 32 »**, aucune autre valeur. HYPOTHESE : le DSP attend l'achevement d'une DMA ; le BSP ecrit `data[]` **directement**, sans la machinerie DMA, donc le drapeau ne se libere jamais. *(Progression `2048` STACK_OV -> `32` DMA_PEND : pas une regression, le DSP va assez loin pour se plaindre de la couche suivante.)* | ouvert | nommer qui pose / qui efface l'etat cote modele |
| B3 | **Format d'entree** : le demod lit en pas de 5, le BSP depose 296 int16 **contigus** I/Q entrelaces. Non tranche : il n'est pas exclu que le pas de 5 soit correct et que ce soit le **remplissage** qui doive changer de layout | hypothese | `CALYPSO_WATCH_9F00_RD` + relecture du filtre polyphase |
| B4 | **Slot de dispatch FB = stub `RET`** : `0xb01c: 10f8 43d8` (adressage **absolu**) ; `data[0x43d8] = 0xab38` ; premier mot de `0xab38` = `fc00` = `RET`. Un **seul** ecrivain sur les 4 banks : `0xbb00`. Watchpoint `data_write_locked` : aucun writer cache. Question ouverte unique : pourquoi l'ordonnanceur `0x7234` ne tombe-t-il jamais sur `0x8341` (la LUT FB) ? | ouvert | bequille de VALIDATION (pas correctif) : `CALYPSO_BSP_DISPATCH_FB=1` |
| B5 | **En natif : ni SCH ni SI**, donc pas de camp meme si le FB tombait. Mesure : `dispatch_allc` = 0, `feed_agch` = 0, `DISPATCH SDCCH` = 0, `sb_valid` = 0 ; le listener `feed_si(a_cd)` est arme (`udp:127.0.0.1:4730`) mais rien n'y arrive (gr-gsm non branche en natif). Cote mobile : `No sysinfo yet` -> `BSIC=0` x7 -> `Cell selection failed, read timeout`. Cote osmocon : `attempt=12` = **chemin de renoncement** d'osmocom, pas une synchro | connu | ordre du plan P4 : FB -> SCH -> SI, oracle = le producteur actuel a chaque etape |
| B6 | **Voix TCH/F : ASSIGNMENT FAILURE.** Double cause localisee : **(A)** `calypso_dsp_shunt.c:177` lit **inconditionnellement** `a_cu@0x264` (SDCCH/SACCH) quel que soit le type de tache, alors que le firmware ecrit la FACCH dans `a_fu@0x282` (`prim_tch.c:422`) et la voix dans `a_du_1@0x134` (`:485`) ; **(B)** `tools/calypso-ipc-device/qemu_wrap.c:1194` cable la voie UL en dur TS0/RACH (`ul_slotoff=1875`), or le TCH/F est assigne **TS2**. Deja cable, ne pas reecrire : TCH DL sub0 (`calypso_tch_dl_poll:333-350`, `shunt_dispatch_tch_dl:356-369`, routage `:836-837`) ; **il manque le PRODUCTEUR** du sideband `/dev/shm/calypso_tch_dl`. Reseau OK (`call fake_trx` = ACTIVE + audio, GAPK/FR compile) | WIP | rejouer en `SHUNT_LEGIT` pour re-chiffrer (0 occurrence dans les logs courants) |
| B7 | **SMS flaky en `DSP,NO_CANNED`** (jitter quand le c54x tourne en //). En `SHUNT_LEGIT` les 3 politiques d'eviction sont a zero (`overflow=0, ttl=0, reps=0`, profondeur max 1) — mais la saturation `depth=32` existait **en `DSP,NO_CANNED`**. **Ne rien supprimer de `sdcch_ring`** avant d'avoir mesure dans le mode ou il sert | WIP | `grep "EVICT-STATS" /root/qemu.log \| tail -1` en `DSP,NO_CANNED` |
| B8 | `bsp_buf[]` rempli par `c54x_bsp_load`, lu par personne (`PORTR PA` jamais `0xF430`) | dormant | `CALYPSO_DEBUG=PORTR-ANY` |
| B9 | `DIRECT_FEED=1` court-circuite la fenetre FN et le pulse BDLENA (RANK2) | connu, contourne | — |

---

## 6. Go-live / shadow IMR

**Recette qui DONNE le shadow IMR (kernel energie qui correle) :**

```
CALYPSO_INIT_435B_OFF=0     # seed d[0x435b] = 0x52ed (sinon 0xa582 ecrit IMR=0)
CALYPSO_KEEP_IMR=1
CALYPSO_ARM2DSP_BGEN=1      # ARM pose les cellules background
CALYPSO_HACK=0              # natif
# SANS CTRLSYS
```

**`CALYPSO_ARM2DSP_CTRLSYS=1` CASSE la recette BGEN.** Il re-force `data[0x0810]` bit15
(= `B_TASK_ABORT`) a `0xa537` (~20 011x) plus vite que le DSP ne le clear a `0xa549`
(~89x) -> abort permanent -> plus de shadow, plus de kernel. **FIX : CTRLSYS=0.**
(Note historique : bit15 de `0x0810` = `B_TASK_ABORT`, PAS un gate go-live a SET —
inverse la vieille conclusion CTRLSYS.)

**SHADOW-DADST** = sonde sur les instructions DADST du kernel energie `0xa076` (ce n'est
PAS le shadow IMR). Sa presence = **le kernel correle**, il est VIVANT.

**Pieges de cap de log** : on croit souvent le shadow « perdu » alors que le code tourne.
`KEEP-IMR` cappe a `< 8` ; `SHADOW-DADST` cappe a `< 40 || %2000`. Ne pas conclure « mort »
sur un cap de log.

---

## 7. Verrous BSP et parametres de destination

Trois gates, tous conditionnes a `calypso_dsp_shunt_active()` — **donc actifs meme en
NATIF**, parce que `CALYPSO_DSP=c54x` (defaut `calypso.env:102`) suffit a poser
`active()=vrai`. `active()` ne veut PAS dire « le shunt remplace le DSP » ; c'est
`substitutes()` qui le dit (split du 2026-07-27).

| # | Fichier:ligne | Fonction | Ce qu'il jette | Leve par |
|---|---|---|---|---|
| 1 | `calypso_bsp.c:474` | `bsp_trxd_readable()` | le datagramme TRXD avant enqueue | `RUN_C54X=1 && (route_c54x_active \|\| BSP_DARAM_FORCE)` |
| 2 | `calypso_bsp.c:997` | `calypso_bsp_rx_burst()` | l'ecriture DARAM directe | idem |
| 3 | `calypso_bsp.c:1359` | `calypso_bsp_deliver_buffered()` | **la LIVRAISON vers `data[]`** | `TPU_RX_WIRE` **ou** `RUN_C54X=1 && BSP_DARAM_FORCE` (aligne le 2026-07-28) |

| Var | Defaut code | Valeur correcte mesuree | Note |
|---|---|---|---|
| `CALYPSO_BSP_DARAM_ADDR` | `0x2a00` | **`0x4c00`** | `0x2a00` = workzone de SORTIE, ne jamais y feeder |
| `CALYPSO_BSP_DARAM_LEN` | `296` | `296` | `638` ne valait que pour le 4 SPS non decime |
| `CALYPSO_BSP_IQ_DECIM` | `4` | **`4`** | `=1` est une **REGRESSION** (feed a 4 SPS, `dphi=+0.25 x pi/2`) |

---

## 8. Affirmations INVALIDEES — ne pas les reintroduire

### 8.1 Invalidees par une mesure contraire

| Affirmation | Pourquoi elle est fausse | Signature de l'artefact |
|---|---|---|
| **« `data[0x4c00]` est gele »** | la lecture avait ete faite a `0xFFD08800` via le **monitor QMP**, HORS fenetre API RAM | peak exactement `0x8000` et **54 % de zeros**. Les mesures valides se prennent A L'INTERIEUR (`BSP_LOG` au point d'ecriture, ou `ddump`) |
| **« Q == 0 »** | conclu sur les **2 premiers mots** d'un burst (amplitude faible par construction) | sur le burst entier, `zeros = 0 %` |
| **« `0xa042` detruit le signal avant lecture du noyau »** | `0x2c00` est du **scratch** : il n'y avait pas de signal a detruire | `0x9fd5` y depose une table de coefficients CONSTANTE |
| **« 30+ writers de `d_fb_det` dans la PROM »** | c'etaient des **opcodes**, pas des ecrivains | publisher **UNIQUE** = `ORM #1,*(0x08f8)` @`0x79e4`, banque commune `0x7700-0x79F0`, **jamais executee** |
| **« 0 FCCH sur 200 dumps »** | **artefact de fenetre** : `CALYPSO_DARAM_DUMP` consommait son cap au boot avec `d_fb_mode = 0` | corrige : le dump est desormais filtre sur `d_fb_mode[0x08f9] != 0` (`_ANYMODE=1` restaure l'ancien comportement) |
| **« le detecteur n'est pas arme »** | `d_fb_mode[08f9] = 0x0001` observe | `DETECTOR-RUN` |
| **« le demod lit `data[0x0000]` par pas de 5 »** | vrai **uniquement** avec `CORR_ENTRY=0x9500`, qui saute les 11 mots posant `AR6`. Avec `0x94f5` : `AR6 = 0x4c00` et le demod lit de vraies valeurs | `CALYPSO_WATCH_9F00_RD` |
| **« sample-in natif = `0x9213`/`0x9215` » (en absolu)** | l'adresse **depend du point d'entree** (§3.1). Avec le run de reference, `0x9213`/`0x9260` ne sont **jamais lues** et `FB_STREAM` est inerte | absence de `FB-STREAM addr=` au log |
| **« `BUILD-STAMP` date le binaire »** | il date le TU `calypso_dsp_shunt.c` uniquement | §0 |
| **« 280 detections sur 300 bursts »** | c'est le contenu des 300 premieres **lignes loguees** (cap `calypso_dsp_shunt.c:1670`) | §0 |
| **« le scheduler `0x7234` DERAILLE vers `0x013b` »** | `0x7234: f074 013b` = **CALL inconditionnel en ROM** ; `0x013b` sauvegarde le contexte et **retourne** en `0x7236` | desassemblage PROM |
| **« `0x8341` = la LUT FB a atteindre »** | scan des **4 banks** : **0 reference**. Personne ne peut y sauter. **Ne pas refaire le correctif TPU de `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md`** | scan statique 4 banks |
| **« writer cache du slot `0x43d8` »** | watchpoint `data_write_locked` (voit **tous** les modes d'adressage) : uniquement `0xbb00 -> 0xab38` | watchpoint |
| **« le handler vient d'un patch televerse »** | osmocom `dsp_bootcode.c` : `dsp_bootcode = NULL`, *« it happily works with its own ROM »* ; `dsp.c:205` = `FIXME: Implement Patch download` | source osmocom |

### 8.2 Fausses pistes a NE PAS rechasser

- **SHADOW_WIRE** (`FB_SHADOW_WIRE` casse le retour, rend le natif = 0).
- **Feeder `0x2a00`** (workzone de SORTIE, PAS l'entree).
- `d_fb_det` via reroute **`FB_ENERGY` seul** (le corrélateur reste affame si la chaine
  d'entree n'est pas alignee sur le point d'entree, §3.1).
- **Go-live INTM / storm / seed `5ac8`** (red herring : le handler FB est du **polling
  pur** sur `data[0x3fad]` bit15 ; l'IT n'est pas sur le chemin critique).
- **Cellules background `0x098a` / `0x098c`** (`d_background_enable/state`, ARM=0 par design).
- **Rotation SI** (la racine « no cell info » = `meas->frames == 0`, le mobile ne recoit
  rien en idle — pas les SI).
- `calypso_fbsb.c` (logger mort, `last 0 0 0 0` = red herring).
- `CALYPSO_ARM2DSP_CTRLSYS=1` (casse la recette BGEN).
- **Poke** (falsifier le retour d'une fonction) : insuffisant, le mur est du **wiring**
  inter-blocs.
- **`api_write_cb`** : declare `calypso_c54x.h:204`, jamais assigne (grep = 0) — dead
  callback, ne pas theoriser dessus.
- **Sous-buffer `0x3fb0` / `0x03F0`** comme sample-in natif.
- **`CALYPSO_BSP_IQ_DECIM=1`** (regression mesuree).
- **Tuner `CALYPSO_SHUNT_BURST_OFS`** : la sonde `WP-BURST` a prouve que l'ARM ne commande
  que le burst 0 (22 501 writes, 0 non-nul) — symptome amont.
- **Variables INERTES** (declarees, jamais lues par `getenv`) :
  `CALYPSO_CORRELATOR_TRACE` (le vrai gate est `CALYPSO_DEBUG=CORRELATOR`),
  `CALYPSO_FORCE_3F92`, `CALYPSO_FORCE_0810`, `CALYPSO_FIX_MVDM` (le code ne lit que
  `..._OFF`), `CALYPSO_FORCE_ANGLE_ZERO`.

---

## 9. Regles de sonde (payees quatre fois) et run de reference

### 9.1 Regles

1. **Une sonde se concoit par sa CONDITION DE DECLENCHEMENT, pas par son adresse.** Un
   plafond global est mange par le PC le plus bruyant. Trois precedents dans le code :
   `WZWRITE` filtre par PC producteur (`0x9fd5`/`0x9ab1`) parce que `0xa03d/a042/a079`
   saturaient ; `ERRWATCH` ignore les ecritures de 0 (nettoyage de boot) ; `CORR_FLOW`
   skippe la boucle de copie `0x8866-0x886c` (~134x/appel).
2. **Preferer un AGREGAT a un FLUX plafonne, et prevoir un TEMOIN DE SATURATION.**
   `CALYPSO_WMAP` a un heartbeat (« writes DSP=N, dans plages=0 ») : c'est lui qui
   distingue « pas d'evenement » de « sonde morte ».
3. **Distinguer « varie dans l'ESPACE » (une courbe sur N cellules) de « varie dans le
   TEMPS » (a cellule figee).** Seule la variation temporelle est un signal. `wmap_note`
   v2 ne collecte les valeurs distinctes qu'a adresse figee (`addr0`).
4. **« Pas de log » n'est JAMAIS « pas d'evenement »** tant que la sonde n'est pas verifiee
   VIVANTE et sa fenetre COUVRANTE. Causes rencontrees, toutes reelles ici : plafond
   sature ; seuil de dump trop haut (`DARAM_DUMP` cape au boot avec `d_fb_mode=0`) ; plage
   ecrite cote HOTE (`feed_iq` ecrit `s->data[]` en direct, **invisible depuis
   `data_write_locked`**) ; variable d'env absente du run (les `DMA fn=` sont muets sans
   `CALYPSO_DEBUG=BSP`).
5. **Toute mesure prise via le monitor QMP est hors fenetre API RAM et racy.** Les mesures
   valides se prennent A L'INTERIEUR : `BSP_LOG` au point d'ecriture, ou `ddump`.

### 9.2 Run de reference (chaine d'entree mesuree correcte)

```
CALYPSO_NATIVE_HELPED=1 CALYPSO_FB_CORR_ENTRY=0x94f5 CALYPSO_DSP_RUN_C54X=1 \
CALYPSO_BSP_DARAM_FORCE=1 CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 \
CALYPSO_BSP_IQ_DECIM=4 CALYPSO_SHUNT_REAL_FB=1 CALYPSO_DEBUG=BSP ./start-clean.sh
```

Pour un natif **nu** (sans intercept de lecture ni PM mocke), ajouter en CLI :
`CALYPSO_DECAN=0 CALYPSO_SHUNT_REAL_FB=0 CALYPSO_SHUNT_NO_FAKE_PM=1`.

### 9.3 Ordre d'instrumentation

| # | Question | Instrument | Verdict attendu |
|---|---|---|---|
| 1 | l'entree du demod est-elle vivante, et OU ? | `CALYPSO_WATCH_9F00_RD` | liste les adresses lues -> fixe ou feeder |
| 2 | le feed arrive-t-il, et est-il propre ? | `corr_iq.py --src bursts` | `coh > 0,9`, `dphi ≈ +1,00 x pi/2` |
| 3 | le buffer lu par le detecteur contient-il la FCCH ? | `CALYPSO_DARAM_DUMP` + `corr_iq.py --src ddump` | non-racy, filtre sur `d_fb_mode != 0` |
| 4 | le detecteur est-il arme et tourne-t-il ? | `DETECTOR-RUN` + `CALYPSO_B2AR` | `d_fb_mode=0x0001` ; AR `IN` vs `OOB` |
| 5 | le math produit-il quelque chose ? | `CALYPSO_DEMODIO` (`0x9f95..0x9fe2`), `CALYPSO_B2`, `CALYPSO_CORROUT` | `A=0x80000000` = saturation (bug §4) ; `A/B` a 0 = pas de math |
| 6 | qui ecrit le resultat ? | `CALYPSO_WATCH_RESULT` (`data[]`) + `CALYPSO_FBDET_API` (`api_ram`) | le firmware lit `api_ram[W-0x0800]`, **pas** `data[]` |
| 7 | le flot atteint-il le publisher ? | `CALYPSO_TRACEFROM` | marque `0xa076` / `0x79e4` / `0x9ac0` |

### 9.4 Mesures a faire, dans l'ordre

```bash
# 1) B1 — le correctif AND/OR/XOR/SUBC a-t-il debloque la SORTIE du demod ?
#    Critere = |DC|/rms et dphi du ddump, PAS d_fb_det.
rm -f /dev/shm/daram_2a00.cfile          # AVANT le run
# ... run de reference + CALYPSO_DARAM_DUMP=1 CALYPSO_SHUNT_REAL_FB=0 ...
cd ${QEMU_TREE}/tools && python3 corr_iq.py --src ddump | tail -3
#    reference a battre : |DC|=2.86e4 pour rms=2.94e4, dphi=+0.004  (sortie morte)

# 2) B2 — nommer qui pose / efface l'etat « DMA en cours »
grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log | sort | uniq -c

# 3) Robustesse LU en SHUNT (departager A5 « 0 retry » vs §9 « 1/19 »)
grep -c T3211 /root/mobile.log ; grep -icE "LOCATION UPDATING ACCEPT" /root/mobile.log

# 4) B7 — eviction SDCCH dans le mode ou elle sert, AVANT toute suppression
grep "EVICT-STATS" /root/qemu.log | tail -1     # en SHUNT_LEGIT=DSP,NO_CANNED
```

---

## 10. References canoniques

### Adresses DSP / ARM

| Cellule | DSP word | ARM addr |
|---|---|---|
| `d_fb_det` | `0x08F8` | `0xFFD001F0` (NDB+0x48) |
| `d_fb_mode` | `0x08F9` | — |
| `a_sync_demod` TOA/PM/ANGLE/SNR | `0x08FA..0x08FD` | NDB+0x4C.. |
| `d_dsp_page` | `0x08D4` | `0xFFD001A8` (BASE_API_NDB) |
| `d_dsp_state` | `0x08E2` | (= 3 IDLE au boot) |
| `d_ctrl_system` (bit15 = `B_TASK_ABORT`) | `0x0810` | — |
| **entree demod** (run de reference) | `data[0x4c00]`, lu en pas de 5 | — |
| entree demod (si `CORR_ENTRY=0x9500`) | `data[0x9213]` / `[0x9215]` | — |
| **workzone OUT demod (NE PAS feeder)** | `0x2a00..0x2b27` (I par `0x9fb8`, Q par `0x9fe2`) | — |
| scratch coefficients (constante) | `0x2c00` (pose par `0x9fd5`) | — |
| noyau MAC energie | `0xa076..0xa09d` (`0x9a80` mode large) | — |
| detecteur | `0x9ac0` (boucle `0x9ab8..0x9ac2`) | — |
| publisher `d_fb_det` (jamais execute) | `0x79e4` (`ORM #1,*(0x08f8)`) | — |
| entree corrélateur referencee en ROM | `0x94f5` (`@0x87e7 f930 94f5`) | — |
| dispatch FB (stub `RET`) | `data[0x43d8] = 0xab38`, `0xab38: fc00` | — |
| verrou-maitre corrélateur | `data[0x3fad]` bit15 (polling `@0x8754`) | — |
| shadow IMR seed | `data[0x435b]` = `0x52ed` | — |
| PROM0 base | `0x7000` | — |

### Task IDs DSP

`PM=1, FB=5, SB=6, RACH=10, NBS=19, NP=21, ALLC=24, CHECKSUM=33`.
(BSIC = bits 2:7 de `a_sch[3] | a_sch[4]<<16`.)

### Gates / mecaniques

- Recalage FN DSP/TRX : `CALYPSO_DL_FN_OFFSET`, **defaut 0 (inerte)** ; valeur qui cale la
  FCCH sur la bonne trame = **-556** (`calypso_trx.c:150-168`). NB : l'offset est trop
  large (il touche aussi la FN UL/DATA_IND) -> **a n'activer que pour l'alignement
  corrélateur**. LOST timer read-driven.
- Vecteurs IT : INT3/frame = vec19 bit3 (`0xFFCC`) ; TINT0 = vec20 bit4 ; BRINT0 = vec21
  bit5 (`0xFFD4`) ; frame-IT native = vec28 bit12.
- Modele gain trf6151 : `a_pm = (target_rf + 71 + gain_trf) * 64` ; reset `REG_RX` `0x9E00`
  = 138.

### Semantique des gates d'environnement (source d'erreur n.1)

| Idiome dans le code | `VAR=0` | `VAR=""` | Desactivation correcte |
|---|---|---|---|
| `getenv(X) ? 1 : 0` | **ON** | **ON** | `unset X` |
| `(e && atoi(e) > 0)` / `(e && *e == '1')` | OFF | OFF | `X=0` |
| `(!e \|\| *e != '0')` — **defaut ON** | **OFF** | ON | `X=0` |
| `getenv(X_OFF) ? 0 : 1` — defaut ON | s.o. | s.o. | poser `X_OFF=1` |

### Runtime

- Tree LIVE = **`${QEMU_TREE}`** (`.latest.bak`/`.bak` = anciens ; overlay
  `qemu-calypso` = **MORT au runtime**, ne pas le patcher).
- Firmware = `${GSM_ROOT}/osmocom-bb-transceiver`.
- Lancement : `osmo-nitb-for-calypso/start-direct.sh` -> `osmo-qemu-calypso/start-clean.sh` (source
  `calypso.env` en `set -a`) -> `run.sh`.
- `run.sh:1151-1155` (mode `full-grgsm`) utilise `=` et non `:=` : `SHUNT_NO_CANNED=1`,
  `DSP_L1STUB=0`, `DSP_L1_STUB=0`, `FORCE_FBSB=0`, `FORCE_AGCH=0` **ne sont pas
  surchargeables en CLI** dans ce mode.
- **L'utilisateur relance la pile lui-meme.** Claude = edits + diagnostic lecture seule.

---

## 11. References durables — ou elles sont REELLEMENT

Ces documents ne decrivent PLUS l'etat courant (leurs narratifs dynamiques sont perimes),
mais restent des references ISA / HW durables. **Chemins verifies le 2026-07-28**
(`ls hw/arm/calypso/doc/ hw/arm/calypso/doc/opcodes hw/arm/calypso/doc/project`) : contrairement
a ce qu'annoncait la version precedente de cette section, **un seul** d'entre eux vit dans
`doc/archive/`. Le `README.md` de `doc/` les indexe deja aux bons chemins.

| Chemin reel | Contenu de reference |
|---|---|
| `doc/opcodes/tic54x_hi8_map.md` | Map hi8 -> mnemonic complet (c'est lui qui donne `0x18..0x19 = and`, cf. S4, ligne 26) ; layout ST1 ; indices MMR |
| `doc/opcodes/0x68_0x6F.md` | ANDM/ORM/XORM/ADDM/BANZ/BANZD/MAR ; famille `0x6F00` ; adressage Smem/SHIFT |
| `doc/opcodes/0xF3.md` | Famille F3xx (INTR k, AND/OR/XOR/SFTL 1-mot, `#lk` 2-mots) ; SFTL vs SFTA |
| `doc/project/STEP2_BC_CONDS.md` | Codes condition BC group-1 (CCEQ/CCNEQ/CCLT/... markers) |
| `doc/C54X_INSTRUCTIONS.md` | XC/FRET/FCALL/RETE/BANZ/BC encodings (SPRU172C) |
| `doc/project/AUDIT_DECODER_20260508.md` | Verites binutils tic54x (STL/STH/CMPS/BIT/MVPD...) ; Tier A lande |
| `doc/DSP_ROM_MAP.md` | Sections ROM (PROM0 `0x7000`, DROM/PDROM) ; API RAM `0x0800`+ |
| `doc/project/BUGS_AND_FIXES.md` | F272=RPTBD, F274=CALLD, `0x8A00`=POPM, SP init `0x5AC8`, RET stubs |
| `doc/CALYPSO_HW.md` | Datasheet HW/protocole (API RAM, TPU_CTRL, TRXDv0, sercomm, L1CTL) |
| `doc/SERCOMM_GATE_ARCHITECTURE.md` | Chemins BSP/L1CTL, HDLC/DLCI, task IDs, `d_dsp_page@0x08D4` |
| `doc/hardware-map.md` | Memory map (Flash/IRAM/XRAM), bases peripheriques, IRQ map, INTH |
| `doc/datasheets/TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf` | SPRU172C — arbitrage final sur une semantique d'instruction |
| `doc/archive/SESSION_20260405_NIGHT4.md` | Encodings F0xx/RSBX/SSBX/RET/RETD/CALLD/IDLE ; vecteurs IT — **seul de cette liste reellement dans `archive/`** |

`doc/archive/` contient par ailleurs l'historique date (sessions, pistes closes) : y chercher
un contexte, jamais un statut.


---

## ⚠️ `CALYPSO_NATIVE_HELPED=1` n'est PAS le mode natif (mesure 2026-07-28)

C'est un **paquet de béquilles**. Le manifeste du run le montre : poser `NATIVE_HELPED=1`
**repose automatiquement** le reroute du corrélateur et l'injection d'IQ :

```
[calypso-manifest] CALYPSO_FB_CORR_ENTRY=0x9500     <- reroute REPOSÉ (valeur par défaut)
[calypso-manifest] CALYPSO_FB_ENERGY=1              <- imposé
[calypso-manifest] CALYPSO_FB_IQ_DARAM=1
[calypso-manifest] CALYPSO_FB_IQ_BASE=0x9210
```

**Conséquence pratique, vérifiée à nos dépens** : retirer `CALYPSO_FB_CORR_ENTRY=0x94f5` de la
ligne de commande ne supprime pas le reroute — il revient simplement à `0x9500`. Un run qui
garde `NATIVE_HELPED` ne teste donc **jamais** le chemin natif ; il compare deux béquilles.

Pour tester réellement le natif, tout enlever :
```bash
CALYPSO_DSP_RUN_C54X=1 CALYPSO_BSP_DARAM_FORCE=1 \
CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 CALYPSO_BSP_IQ_DECIM=4 \
CALYPSO_DARAM_DUMP=1 CALYPSO_WATCH_9F00_RD=1 ./start-clean.sh
```
et **contrôler le manifeste AVANT de lire quoi que ce soit** :
```bash
grep -E "calypso-manifest.*(CORR_ENTRY|FB_ENERGY|REAL_FB|NATIVE_HELPED|FB_IQ)" /root/qemu.log
```
Règle générale : **lire le manifeste, jamais la ligne de commande**. La ligne de commande dit
ce qu'on a demandé ; le manifeste dit ce qui s'applique.

## Sources d'autorité pour les opcodes C54x (posées le 2026-07-28)

1. **`doc/opcodes/tic54x-opc.c`** — la table binutils, désormais **copiée dans le dépôt**.
   Format : `{ "mnémo", MOTS, cycles, classe, OPCODE, MASQUE, {opérandes}, flags }`.
   Le champ **MOTS** fait foi : une longueur fausse ne donne pas un résultat faux, elle
   **désynchronise tout le décodage en aval**.
2. **`doc/spru172c.pdf`** — manuel TI, autorité pour la **sémantique** d'exécution.
   (Pas d'extracteur dans le conteneur : le copier dehors et décompresser les flux avec zlib.
   Les tableaux d'encodage perdent leur mise en page à l'extraction — d'où la primauté de
   binutils sur l'encodage.)
3. le code, puis les tableaux de synthèse.

**Deux erreurs de nos propres tables, corrigées le 2026-07-28** (chacune a coûté une fausse
piste) :

| Ce qui était écrit | Ce que dit binutils | Où |
|---|---|---|
| `0xF4..0xF7` = « add (**2-mot**) » | `{ "add", **1**,1,3, 0xF400, 0xFCE0 }` = **1 mot**, registre-registre. Les formes à long immédiat (2 mots) sont en `0xF0..0xF3` (`0xF000/0xFCF0`). | `opcodes/tic54x_hi8_map.md` |
| `0xEA` = « BANZ (confirmed) » | `{ "ld", 1,2,2, **0xEA00**, 0xFE00, {OP_k9,OP_DP} }` = **`LD #k9, DP`**, chargement du Data Page pointer. `banz` est en `0x6C00`, `banzd` en `0x6E00`, 2 mots. | `C54X_INSTRUCTIONS.md` |

Dans les deux cas **le code de `calypso_c54x.c` était correct** et c'est la doc qui égarait.
Corollaire de méthode : **ne jamais conclure depuis un commentaire de code** — plusieurs se
sont avérés périmés, dont un `[TODO]` sur `STL/STH … ASM` alors que `asm_shift()` est bien
appliqué.


---

## VERROU DU MODE NATIF — mesure du 2026-07-28 (fin de journee)

**En mode natif, sans aucune bequille, le correlateur n'est JAMAIS atteint.** Ce n'est pas un
probleme de traitement du signal : c'est le dispatch d'interruption qui ne se fait pas.

Run de 102 s, manifeste verifie (`NATIVE_HELPED=0`, `SHUNT_REAL_FB=0`, ni `FB_CORR_ENTRY`
ni `FB_ENERGY`) :

| Mesure | Valeur | Instrument |
|---|---|---|
| le DSP tourne | **100 000 000** instructions | `SP-LEDGER` |
| le BSP alimente | **266** depots | `DMA fn=` / `a-daram-ok` |
| l'IT est **pendante** | `IFR = 0x0028` (bit 5) | `SYNC-DISPATCH-PROBE` |
| l'IT est **masquee** | `vec=21(BRINT0) imr_bit=5 unmasked=0` → `STAYS-PENDING(masked)` | idem |
| `IMR` | oscille `0x3000` ↔ `0x3200`, ecrit par `PC=0xde84` (op `0x6981`) et `PC=0xddf9` (op `0x6881`). **Le bit 5 (`0x0020`) n'est jamais arme** | `BOOT-MMR-WR` |
| shadow `d[0x435b]` | `0x0000` en permanence | `HANDLER-PATH`, `CYCLE-TRACE` |
| le DSP | boucle dans `0xddf5..0xde86` (dispatcher background) | `SP-LEDGER` |
| l'etage demod | **JAMAIS atteint** — compteur **ZERO** | `CALYPSO_WATCH_9F00_RD` (sonde verifiee presente dans `/proc/<pid>/environ`) |

### Consequence, a lire avant toute autre section
Tout ce qui a ete observe sur l'etage demod le 2026-07-28 — polyphase stride 5 sur `data[0x4c00]`,
saturation d'accumulateur, sortie DC, `AR2` charge avec une valeur d'echantillon, recopie de code
machine (`0xf495` = NOP, `0xf4eb` = RETE) dans le tampon de sortie — **n'existe que sous bequille**,
c'est-a-dire avec le reroute `CALYPSO_FB_CORR_ENTRY=0x9500` ou `0x94f5`. Ces observations ne
decrivent PAS l'etat du mode natif et ne doivent pas etre citees comme telles.

### Piste a trancher EN PREMIER : le vecteur 21 est-il installe ?
```
[c54x] VEC-INSTALL vec21@0x00d6 w2 <- 0x0000 <BRINT0 PC=0xb4d6
[c54x] VEC-INSTALL vec21@0x00d7 w3 <- 0x0000 <BRINT0 PC=0xb4d6
```
Un vecteur rempli de zeros est-il un handler valide ? **Si le vecteur pointe sur du vide,
demasquer l'IMR ne servirait a rien** et la racine serait l'INSTALLATION du vecteur, pas le
masquage. A verifier avant d'ecrire le moindre correctif d'armement.

### Croisement avec l'audit d'opcodes
Les deux instructions qui ecrivent l'`IMR` en boucle — `0x6881` et `0x6981` — appartiennent a la
famille `0x68..0x6F`, qui a sa propre note dans le depot (`doc/opcodes/0x68_0x6F.md`), signe
qu'elle est delicate. **Si elles sont mal decodees, l'`IMR` est mal calcule et le bit 5 ne peut pas
s'armer** : l'audit d'opcodes expliquerait alors directement ce verrou, et les deux enquetes n'en
feraient qu'une.

## AUDIT DU DECODEUR c54x — `RAPPORT_OPCODES.md` (2369 lignes)

Le decodeur confond **1 mot** et **2 mots** sur au moins seize familles (`0x62-0x67`, `0x78-0x7D`,
`0x85`, `0x8D`, `0x94/0x95`, `0x96`, `0xA2/0xA3`, `0xA8/0xA9`, `0xAC-0xAF`, `0xC0-0xC7`, `0xDA`,
`0xE0-0xE4`). Une longueur fausse ne donne pas un resultat faux : elle **desynchronise tout le
decodage en aval**. ~40 findings confirmes, dont ~15 de gravite 1. **Rien n'est encore applique.**

Reserves : deux plages (`0x60-0x8F`, `0xC0-0xFF`) n'ont pas eu de passe de refutation ; l'audit a
demarre avant la correction de deux tables du projet (`0xF4..0xF7` = 1 mot et non 2 ; `0xEA00` =
`LD #k9,DP` et non `BANZ`), donc tout finding fonde sur l'ancienne table est suspect.

### Correctifs d'opcodes APPLIQUES et VALIDES au 2026-07-28

| opcode | ce qu'on faisait | SPRU172C |
|---|---|---|
| `0x1800/1A00/1C00/1E00` | AND/OR/XOR/SUBC executes comme un **LD** | operations logiques ; `Smem` zero-etendu sur 40 bits |
| `0x47` RPT Smem | ecrivait **BRC** | *Repeat single, **RC** = Smem* — les boucles `RPT` ne s'executaient qu'**une seule fois** et BRC etait corrompu |
| `0x06/07` ADDC | carry ignore | `src = src + Smem + C` |
| `0x0E/0F` SUBB | borrow ignore | `src = src - Smem - C` ⚠️ polarite a surveiller (certains DSP soustraient `~C`) |
| `0x38/39` SQURA | `T` non ecrit | `src = src + Smem*Smem`, **`T = Smem`** |

**Non-regression verifiee** apres l'ensemble : `SHUNT_LEGIT=1 NO_CANNED=1 REAL_FB=1` donne
`BSIC=7`, 14 `SYSTEM INFORMATION`, `LOCATION UPDATING ACCEPT` et `TMSI REALLOCATION COMPLETE`.

Laisses de cote faute de semantique explicite dans l'extrait du manuel : `LDM 0x48/0x49`
(zero- vs sign-extension) et `DST 0x4E/0x4F` (post-modification ±2).

⚠️ **Aucun de ces correctifs ne peut debloquer `d_fb_det`** : en natif le correlateur n'est jamais
atteint (voir ci-dessus). Ils corrigent l'emulation, ce qui est necessaire, mais le verrou est ailleurs.
