# Variables d'environnement `CALYPSO_*`

> Version initiale du 2026-07-28, etablie a partir des variables reellement manipulees et
> mesurees ce jour. Un recensement exhaustif (~300 variables) est en cours et enrichira ce
> document. Ce qui figure ici est **verifie**, pas suppose.

## La regle d'or

**La verite est le MANIFESTE imprime au demarrage, jamais la ligne de commande.**

```bash
grep -E "calypso-manifest" /root/qemu.log
```

Raison, mesuree a nos depens le 2026-07-28 : certaines variables en **reposent** d'autres.
`CALYPSO_NATIVE_HELPED=1` impose silencieusement :

```
CALYPSO_FB_CORR_ENTRY=0x9500     <- repose (valeur par defaut)
CALYPSO_FB_ENERGY=1
CALYPSO_FB_IQ_DARAM=1
CALYPSO_FB_IQ_BASE=0x9210
```

Retirer `CALYPSO_FB_CORR_ENTRY=0x94f5` de la ligne de commande **ne supprime pas le reroute** :
il revient a `0x9500`. Un run qui garde `NATIVE_HELPED` ne teste donc jamais le chemin natif —
il compare deux bequilles. Une demi-journee a ete perdue sur cette confusion.

## Les quatre idiomes de gate

Ils ne se coupent pas de la meme facon. C'est la premiere source d'erreur de manipulation.

| Idiome | Actif quand | Comment le couper |
|---|---|---|
| `getenv("X") ? 1 : 0` | la variable **existe**, meme a `0` | **`unset X`** — mettre `0` ne suffit pas |
| `atoi(getenv("X")) > 0` | valeur > 0 | `X=0` |
| `*e == '1'` | valeur exactement `"1"` | toute autre valeur |
| defaut ON + `X_OFF` | par defaut | poser `X_OFF` |

Dans `calypso*.env`, l'idiome compte aussi : `:=` laisse la ligne de commande gagner, `=` la
verrouille. Une variable posee avec `=` **ne peut pas** etre surchargee depuis le shell.

## Les cinq categories

| Categorie | Definition | Sort |
|---|---|---|
| **CONFIG** | parametre legitime du modele (adresse, longueur, cadence) | reste |
| **MESURE** | sonde / trace / dump, sans effet sur l'emulation | reste, defaut OFF |
| **SAS** | correctif en attente de validation (`CALYPSO_FIXES`) | **degate** apres validation |
| **BEQUILLE** | contourne une branche non implementee | **remplacee** par la branche reelle |
| **MORT** | plus lue nulle part | retiree |

Critere qui tranche entre CONFIG et BEQUILLE : *« le materiel reel a-t-il un equivalent de ce
reglage ? »* Si non, c'est une bequille.

`grep -rn "@BEQUILLE" hw/arm/calypso/` liste les bequilles annotees dans le code.

## CONFIG — parametres legitimes (valeurs mesurees le 2026-07-28)

| Variable | Valeur juste | Pourquoi |
|---|---|---|
| `CALYPSO_BSP_IQ_DECIM` | **4** | `1` est une **regression mesuree** : le feed arrive a 4 SPS, `dphi=+0.25x pi/2` au lieu de `+1.00`. Avec `4` : `corr_iq.py --src bursts` affiche « FCCH @1SPS PROPRE ». |
| `CALYPSO_BSP_DARAM_LEN` | **296** | 148 echantillons complexes apres decimation. `638` ne valait que pour le 4 SPS non decime. |
| `CALYPSO_BSP_DARAM_ADDR` | **0x4c00** | Entree du demod **confirmee par mesure** : `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a`, stride 5 = polyphase 6 taps, valeurs reelles. |
| `CALYPSO_DSP_RUN_C54X` | `1` en natif | fait tourner le vrai c54x. |
| `CALYPSO_DEBUG` | liste de jetons | namespace separe (`BSP`, `CORRELATOR`, ...), lu par `calypso_debug_enabled()`. Ce sont des **sous-cles**, pas des variables. |

⚠️ `CALYPSO_BSP_DARAM_FORCE` ouvre le feed BSP, mais il y avait **trois** gates et non un :
`calypso_bsp.c:474`, `:997` et la **livraison** (`:1347`), cette derniere ne connaissant que
`CALYPSO_TPU_RX_WIRE`. Resultat : `DARAM_FORCE=1` ouvrait deux verrous sur trois et **rien
n'arrivait au DSP**. Aligne le 2026-07-28.

## MESURE — les sondes (toutes defaut OFF, sans effet sur l'emulation)

| Variable | Ce qu'elle montre |
|---|---|
| `CALYPSO_WMAP` (+`_LO/_HI/_LO2/_HI2`) | carte **agregee** des ecrivains d'une plage `data[]` : par PC, nombre d'ecritures, valeurs distinctes **a cellule figee**, min/max. Signale sa propre saturation. |
| `CALYPSO_RMAP` (+`_PCLO/_PCHI`) | symetrique : quelles adresses **lit** une plage de PC. |
| `CALYPSO_DEMODIO` (+`_AFTER/_PCLO/_PCHI`) | correle lectures et ecritures sur une meme fenetre, avec `A`/`B`/`T` et les `AR`. |
| `CALYPSO_WATCH_9F00_RD` | lectures de l'etage demod. **Compteur a zero = le demod n'est jamais atteint.** |
| `CALYPSO_DARAM_DUMP` | dump interne **non-racy** du tampon lu par le detecteur, exploitable par `tools/corr_iq.py --src ddump`. |
| `CALYPSO_TRACEFROM` (+`_N`) | dump des opcodes et trace de flux depuis un PC. Ne se declenche que sur un PC **reellement execute** (une adresse au milieu d'une instruction ne tirera jamais). |
| `CALYPSO_DEBUG=BSP` | logs du BSP (`DMA fn=`, `DARAM after write`). |

**Regle de sonde**, payee quatre fois dans la journee : une sonde se concoit par sa **condition de
declenchement**, pas par son adresse ; preferer un **agregat** (compte tout le run, imprime un
tableau) a un flux plafonne, qui se fait manger par le PC le plus bruyant ; distinguer « varie
dans l'espace » (une courbe sur N cellules) de « varie dans le temps » (a cellule figee) — seule
la seconde est un signal ; et **« pas de log » n'est jamais « pas d'evenement »** tant que la
sonde n'est pas verifiee vivante *et* sa fenetre verifiee couvrante.

## SAS — `CALYPSO_FIXES`

```
CALYPSO_FIXES=FIX_UN,FIX_DEUX   active des correctifs nommes
CALYPSO_FIXES=all               les active tous
(absent)                        aucun — comportement d'origine inchange
```

Protocole : poser **tous** les correctifs surs d'un coup, tester **sous charge maximale**
(camp + LU + SMS, pas un simple boot), puis **des qu'un correctif est confirme, effacer LA
CONDITION** — le `if (calypso_fix_enabled(...))` et ses accolades — **pas le correctif**. Le code
reste et devient inconditionnel.

Un sas se vide, une bequille reste : ne jamais laisser vieillir un correctif valide derriere son
gate.

### Correctifs actuellement dans le sas

| Gate | Opcode | Correction |
|---|---|---|
| `FIX_LD_XMEM_SHFT` | `0x94/95` | `ld Xmem,SHFT,dst` **1 mot** (etait decode MVDK/MVKD en 2) |
| `FIX_BIT_XMEM` | `0x96` | `bit Xmem,BITC` **1 mot** (etait MVDP en 2) |
| `FIX_SUB_XMEM_YMEM` | `0xA2/A3` | `sub Xmem,Ymem,dst` **1 mot** (etait ADD/SUB #lk en 2) |
| `FIX_LD_PARALLEL` | `0xA8-AF` | `ld ‖ MAC/MAS/MASR` **1 mot** (etait AND #lk / MACP / MACD en 2) |
| `FIX_STL_B_ASM` | `0x85` | `stl B,ASM,Smem` **1 mot** (etait MVPD en 2) |
| `FIX_ST_TRN` | `0x8D` | `st TRN,Smem` **1 mot** (etait MVDD en 2) |
| `FIX_LDM_ZEROEXT` | `0x48/49` | `ldm` zero-etendu (un MMR n'est pas signe) |
| `FIX_DST_LMEM2` | `0x4E/4F` | `dst` post-modification **±2** (Lmem = 2 mots) |
| `FIX_STL_STH_SHFT` | `0x98-9B` | champ `SHFT` applique (il etait ignore) |
| `FIX_SUB16_SRC` | `0x40-43` | bit 9 = SRC distinct du bit 8 = DST |

Une **longueur** fausse ne donne pas un resultat faux : elle **desynchronise tout le decodage en
aval**. C'est le patron de bug le plus grave, et il est massif dans ce decodeur.

### Deja degates (correctifs devenus inconditionnels, valides)

`0x1800/1A00/1C00/1E00` AND/OR/XOR/SUBC (etaient executes comme un `LD`) ·
`0x47` RPT Smem (ecrivait `BRC` au lieu de `RC` : les boucles ne tournaient qu'une fois) ·
`0x06/07` ADDC (`+C`) · `0x0E/0F` SUBB (`-C`) · `0x38/39` SQURA (`T = Smem`).

Non-regression verifiee : `BSIC=7`, 14 `SYSTEM INFORMATION`, `LOCATION UPDATING ACCEPT`,
`TMSI REALLOCATION COMPLETE`.

## BEQUILLE — contournements identifies

| Variable | Ce qu'elle masque | Quand la retirer |
|---|---|---|
| `CALYPSO_NATIVE_HELPED` | **paquet** : repose `FB_CORR_ENTRY=0x9500`, `FB_ENERGY=1`, `FB_IQ_DARAM=1`, `FB_IQ_BASE=0x9210` | quand le chemin natif atteint le correlateur seul |
| `CALYPSO_FB_CORR_ENTRY` | l'absence de dispatch natif vers le correlateur : **reroute** l'execution | quand le dispatch natif fonctionne |
| `CALYPSO_FB_ENERGY` | idem, variante du reroute | idem |
| `CALYPSO_FB_STREAM` (+`_CELL`/`_CELLQ`/`_DECIM`) | l'absence de DMA on-chip : **injecte** un echantillon a chaque lecture | quand le buffer est alimente par le vrai chemin. **Inerte** avec `CORR_ENTRY=0x94f5` : les cellules `0x9260/61` n'y sont jamais lues |
| `CALYPSO_SHUNT_REAL_FB` | le corrélateur natif : **court-circuite** le resultat FB (intercept `calypso_dsp_shunt.c:1463-1490`) | quand `d_fb_det` natif fonctionne. ⚠️ masque le natif : ne pas l'utiliser pour juger du natif |
| `FIX_BRINT0_UNMASK` | l'absence d'armement natif de l'IMR bit 5 | **diagnostic** : se retire, ne se confirme jamais |

## Le verrou actuel, et ce que chaque mode prouve

**Mesure du 2026-07-28, mode natif pur** (102 s, aucune bequille) : le DSP tourne
(100 M instructions), le BSP alimente (266 depots), mais `BRINT0` (vec 21) reste **masquee**
(`imr_bit=5 unmasked=0`, `IFR=0x0028` donc pendante, `IMR` oscille `0x3000`↔`0x3200` sans le
bit 5, `d[0x435b]=0`). Le DSP boucle dans `0xddf5..0xde86` et **le correlateur n'est jamais
atteint** (`CALYPSO_WATCH_9F00_RD` = 0).

| Mode | Commande | Ce qu'il prouve | Ce qu'il ne prouve PAS |
|---|---|---|---|
| **qui campe** | `CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 CALYPSO_SHUNT_REAL_FB=1` | la pile complete fonctionne : camp, LU, TMSI, SMS | rien sur le DSP natif — le FB est calcule cote hote |
| **natif pur** | `CALYPSO_DSP_RUN_C54X=1 CALYPSO_BSP_DARAM_FORCE=1 CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 CALYPSO_BSP_IQ_DECIM=4` | l'etat reel de l'emulation DSP | il ne campe pas : le correlateur n'est pas atteint |
| **natif aide** | + `CALYPSO_NATIVE_HELPED=1` | le comportement du correlateur *une fois atteint* | **rien sur le chemin natif** — l'entree est reroutee |

Toute mesure prise en « natif aide » doit etre citee comme telle. C'est ainsi qu'on a cru
pendant des heures analyser un etage de demodulation qui, en natif, n'est jamais execute.
