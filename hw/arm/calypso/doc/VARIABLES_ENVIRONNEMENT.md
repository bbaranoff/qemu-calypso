# Variables d'environnement `CALYPSO_*` — référence complète

> Recensement exhaustif du 2026-07-28 : **312 variables** lues par le modèle QEMU
> (`hw/arm/calypso/*.c|*.h`), dont 299 via `getenv()` direct et **13 via des helpers**
> (`a2d_env_u16()` dans `calypso_arm2dsp.c`, `parse_uint_env()` dans `calypso_bsp.c`) — ces
> treize-là sont **invisibles à un `grep getenv`** et avaient été manquées par tous les
> inventaires précédents.
>
> Chaque entrée est vérifiée contre le **code exécuté**, pas contre les commentaires : plusieurs
> se sont avérés périmés. Les numéros de ligne valent pour le snapshot dont les md5 sont donnés
> en tête de chaque lot ; re-grepper avant d'annoter.
>
> **Mise à jour du 2026-07-29** — l'idiome `EXISTS` n'existe plus : ses 103 sites ont été migrés
> vers un helper unique, `calypso_gate()`. Voir « Le helper unifié » ci-dessous. Les sections
> « idiomes », « MORTES » et « pièges » ont été corrigées en conséquence ; le recensement détaillé
> par lot, lui, reste le snapshot du 28 et n'a PAS été réécrit — ses colonnes « IDIOME » disant
> `EXISTS` sont donc historiques.
>
> **Mise à jour du 2026-08-03** — quatre gates ne pouvaient PAS être coupées à la main :
> `INJECT_SB`, `TRF_RXLEV` (3 sites) et `SHUNT_REAL_FB` utilisaient l'idiome
> `(gate=='1') || (parapluie=='1')`, où le parapluie **écrase** un `=0` explicite. C'est le
> bug déjà corrigé sur `INJECT_ACD` le 30/07, resté sur ces sites-là. Tous migrés vers
> `calypso_gate(nom, defaut_du_parapluie)` : le parapluie n'est plus qu'un **défaut**.
> Trois gates nouvelles : `CALYPSO_IT_TABLE_DOC`, `CALYPSO_SHUNT_SB_MAX_AGE`, et
> `CALYPSO_SHUNT_NO_CANNED` qui devient **implicite sous parapluie**.

---

## La règle d'or

**La vérité est le MANIFESTE imprimé au démarrage, jamais la ligne de commande.**

```bash
grep -E "calypso-manifest" /root/qemu.log
```

Certaines variables en **reposent** d'autres, silencieusement. Trois cas mesurés :

| Poser ceci… | …allume aussi, sans le dire |
|---|---|
| `CALYPSO_NATIVE_HELPED=1` | `FB_CORR_ENTRY=0x9500`, `FB_ENERGY=1`, `FB_IQ_DARAM=1`, `FB_IQ_BASE=0x9210` |
| `CALYPSO_DSP=c54x` | `C54X_IRQ_LEVEL` et `DSP_FRAME_VEC28` — deux comportements non demandés |
| `CALYPSO_DECAN=1` | implique `SHUNT_REAL_FB` (**défaut** depuis le 03/08, plus un écrasement) |
| `CALYPSO_SHUNT_LEGIT` (toute valeur) | `CALYPSO_SHUNT_NO_CANNED=1` — **nouveau 03/08** |

### Les gates ajoutées / corrigées le 2026-08-03

| Variable | Idiome | Défaut | Effet |
|---|---|---|---|
| `CALYPSO_IT_TABLE_DOC` | `calypso_gate` | 0 (sas) | Émet TINT sur **vec19/bit3** (CAL000 §5.1) au lieu de vec20/bit4 (= RINT/SPI receive, table SPRU131 du C54x générique). **Pas inerte** : l'IMR mesurée 0x52ed a le bit 3 démasqué et le bit 4 masqué — l'IT passe de « jetée » à « dispatchée ». À tester sous charge, cf. `environnement/fixes.env`. |
| `CALYPSO_SHUNT_SB_MAX_AGE` | entier (trames) | 104 sous `shunt_legit`, sinon 0 | Âge maximum de la SB republiée. `sb_valid` ne redescend jamais : sans ce garde-fou un unique SCH est rejoué indéfiniment avec sa FN d'origine. 0/absent = replay infini + simple log de signalement. |
| `CALYPSO_FBSB_TIMEOUT` | entier (trames) | 100 ; **2000** en `native` | Budget d'une tentative FBSB, transmis dans le `L1CTL_FBSB_REQ`. Était le littéral `100` dans `gsm322.c` — 461 ms de temps GSM, soit ~10 s de mur ici (tick TDMA à ~10 Hz au lieu de 217). C'est ce qui cadençait la boucle de reset L1. **Côté `mobile`**, pas côté QEMU : patch `osmo_egprs/patches/osmocom-bb-fbsb-timeout-scan-to.patch`. |
| `CALYPSO_SCAN_TO` | entier (secondes) | 4 ; **240** en `native` | Garde-fou couche 3 (`support.c`, `sup->scan_to`). N'avait **aucune commande VTY** — donc irréglable depuis `mobile.cfg`. Doit dépasser la durée mur de `FBSB_TIMEOUT` trames, sinon il devient le facteur limitant. Plafond 255 (uint8). |
| `CALYPSO_SHUNT_NO_CANNED` | `*e=='1'` | **1 sous parapluie** | Auparavant il fallait écrire `CALYPSO_SHUNT_LEGIT=NO_CANNED` ; un `=1` (ou une surcharge CLI) faisait retomber en mode canné **sans le dire**, donc avec des sorties DSP fabriquées. Seul un `=0` explicite re-canne. |
| `CALYPSO_INJECT_SB` | `calypso_gate` | parapluie | **CORRIGÉ** : `=0` coupe enfin. |
| `CALYPSO_TRF_RXLEV` | `calypso_gate` | parapluie | **CORRIGÉ** (3 sites : `c54x.c`, `dsp_helper.c`, `dsp_shunt.c`) : `=0` coupe enfin. La note « TRF_RXLEV est OFF en natif » était infaisable dès que le parapluie était levé. |
| `CALYPSO_SHUNT_REAL_FB` | `calypso_gate` | `DECAN` | **CORRIGÉ** : `DECAN=1` n'écrase plus un `=0` explicite. |

Variable **morte** supprimée le 03/08 : `calypso_shunt_legit.env` posait `CALYPSO_NO_CANNED`,
qui n'existe pas dans le code (le vrai nom est `CALYPSO_SHUNT_NO_CANNED`).

Conséquence pratique : retirer `FB_CORR_ENTRY` de la ligne de commande **ne supprime pas** le
reroute — il revient à sa valeur par défaut. Une demi-journée a été perdue sur cette confusion.

## Les quatre idiomes de gate

Ils ne se coupent pas de la même façon. C'est la première source d'erreur de manipulation.

| Idiome | Actif quand | Comment le couper |
|---|---|---|
| ~~`getenv("X") ? 1 : 0` (**EXISTS**)~~ | ~~la variable **existe**, même à `0`~~ | **MIGRÉ le 2026-07-29** → `calypso_gate()` |
| `atoi(getenv("X")) > 0` | valeur > 0 | `X=0` |
| `*e == '1'` (**EQ1**) | valeur exactement `"1"` | toute autre valeur |
| défaut ON + `X_OFF` | par défaut | poser `X_OFF` |

Dans les fichiers `.env`, l'idiome compte aussi : `:=` laisse la ligne de commande gagner, `=` la
**verrouille** (une variable posée avec `=` ne peut pas être surchargée depuis le shell).

À part : `CALYPSO_DEBUG` n'est pas une variable mais un **namespace** de ~105 jetons, lus par
`calypso_debug_enabled()` / `cdbg_env()`.

### Le helper unifié — `calypso_gate()` (2026-07-29)

Les trois idiomes donnaient **trois réponses différentes** à `X=0` et à `X=` : c'était, de loin,
la première cause d'erreur de manipulation. Un helper unique les remplace :

```c
int calypso_gate(const char *nom, int defaut);   /* calypso_debug.h */
```

| Valeur | Résultat |
|---|---|
| variable **absente** | `defaut` |
| `""` · `0` · `no` · `off` · `false` (casse indifférente) | **0** |
| tout le reste (`1`, `yes`, `2`, …) | **1** |

Deux gains : `X=0` coupe **toujours**, et le paramètre `defaut` dit **à l'appel** ce que vaut
l'absence — l'information qui manquait le plus dans le code.

L'implémentation est dans `calypso_debug.c`. ⚠️ **Pas** dans `calypso_dbg.c`, qui n'est pas
référencé par `meson.build` et n'est donc jamais compilé — piège déjà signalé plus bas.

**État de la migration** : `EXISTS` **fait, 103 sites, 0 reste**
(`arm2dsp` 1 · `bsp` 8 · `c54x` 85 · `dsp_helper` 1 · `dsp_shunt` 3 · `trx` 5).
Restent `EQ1` (79 sites) et `ATOI` (22) — leur idiome ne piège pas dans le même sens, la migration
est moins urgente.

La migration était sûre parce que **mesurée avant** : sur les 82 variables en idiome `EXISTS`,
aucune n'était posée à `0` dans `environnement/` ni `run_modules/`. Non-régression vérifiée
(camp + `LU ACCEPT` + `TMSI` sur les deux abonnés).

**Ce que le helper ne couvre pas** : les variables qui portent une **valeur** (adresse, longueur,
cadence, chemin, liste) gardent `getenv` + `strtoul`/`atoi`. Les convertir n'aurait aucun sens.

## Les cinq catégories

| Catégorie | Définition | Sort |
|---|---|---|
| **CONFIG** | paramètre légitime du modèle (adresse, longueur, cadence, chemin) | reste |
| **MESURE** | sonde / trace / dump, sans effet sur l'émulation | reste, défaut OFF |
| **SAS** | correctif en attente de validation (`CALYPSO_FIXES`) | **dégaté** après validation |
| **BÉQUILLE** | contourne une branche non implémentée | **remplacée** par la branche réelle |
| **MORT** | plus lue nulle part, ou lue dans du code non lié | retirée |

Critère qui tranche entre CONFIG et BÉQUILLE : *« le matériel réel a-t-il un équivalent de ce
réglage ? »* Si non, c'est une béquille.

> **État du sas `CALYPSO_FIXES` au 2026-07-29 — il est VIDE de candidats.**
> Les sept correctifs confirmés ont déjà été dégatés (leur condition effacée, le code conservé).
> Il ne reste que quatre noms, dont **aucun n'est à durcir** :
> `FIX_LD_PARALLEL`, `FIX_STL_STH_SHFT`, `FIX_LDM_ZEROEXT` sont les trois **infirmés** malgré un
> encodage formellement juste, et `FIX_BRINT0_UNMASK` est un **diagnostic** — il répond à une
> question puis se retire, il ne se confirme jamais.

**105 béquilles sont annotées dans le code** et se recensent d'un grep :

```bash
grep -rn "@BEQUILLE" hw/arm/calypso/*.c hw/arm/calypso/*.h calypso*.env   # liste complète
grep -rhoE "@BEQUILLE — [A-Za-z_0-9]+" hw/arm/calypso/*.c | sort -u        # juste les noms
```

## Variables MORTES — candidates au retrait

Repérées par le recensement, avec la preuve qu'elles ne sont plus lues :

`START_FN` · `NB_MAXDLY` · `BSP_BYPASS_BDLENA` (le gate qu'elle prétend couper a été supprimé le
2026-05-29) · `DBG` (`calypso_dbg.c` n'est **pas dans `meson.build`** — jamais compilé) ·
`ORCH` · `TINT0_PERIOD` · `C54X_CRASHPC` (ses deux occurrences sont des arguments de `fprintf`) ·
`TRAP_CHECKPOINT` · `FIX_MVDM` · `CORRELATOR_TRACE`.

> **Re-vérification du 2026-07-29 — la liste n'est PAS à appliquer telle quelle.**
> Un contrôle par `grep 'CALYPSO_<VAR>"'` sur `hw/` et `tools/` ne confirme la mort que de
> **quatre** d'entre elles : `START_FN`, `NB_MAXDLY`, `FIX_MVDM`, `CORRELATOR_TRACE`.
> Les six autres ont encore une occurrence du littéral. Ce n'est pas contradictoire — pour
> `C54X_CRASHPC` la doc dit elle-même que ses occurrences sont des `fprintf`, pas des `getenv` —
> mais **chacune doit être ré-examinée individuellement avant retrait**, en distinguant un
> `getenv()` d'une simple mention. Ne pas supprimer sur la foi de cette liste seule.

## Pièges relevés par le recensement

- **`BSP_DIRECT_FEED`** court-circuite tout le match FN : la file reste vide, donc **tout
  `deliver_buffered` est du code mort dans le run vivant** — y compris `IQDUMP` et `RX_FBFLAGS`.
  Mesurer avec ces sondes pendant que `DIRECT_FEED=1` ne donne rien, par construction.
- **`BSP_DARAM_FORCE`** utilise l'idiome `EXISTS` : `=0` **ne la coupe pas**, il faut `unset`.
  Et elle n'a d'effet que si `DSP_RUN_C54X=="1"`.
- **`DSP_BLOB`**, s'il est posé, fait **ignorer toutes** les sections PROM/DROM.
- **`LDK8_SHIFT16`** *(trouvé le 2026-07-29, en migrant vers `calypso_gate`)* : elle était
  déclarée **vide** dans `opcodes.env` (`: "${CALYPSO_LDK8_SHIFT16:=}"`), ce qui, sous l'idiome
  `EXISTS`, la rendait **ACTIVE** — l'ancien comportement A/B de LDK8 tournait donc en permanence,
  à l'inverse de ce que la ligne laissait lire. Elle est désormais fixée explicitement à `1`, pour
  ne rien changer au run validé : c'est un **constat**, pas un choix de conception, et il est
  maintenant révocable en connaissance de cause. Leçon générale : **une variable déclarée vide
  n'était pas neutre** ; chercher les autres `:=` vides avant de conclure sur un défaut.

---

# Recensement détaillé, par lot

Ce qui suit est le rendu des six agents d'analyse, conservé tel quel. Colonnes :
`VARIABLE | DÉFAUT | EFFET (code exécuté) | MODE | IDIOME | CATÉGORIE | REPOSE / REPOSÉE PAR`.


---

## Inventaire exhaustif et méthode

# INVENTAIRE EXHAUSTIF DES VARIABLES CALYPSO — snapshot `2026-07-28T15:44:38Z`

**AVERTISSEMENT MÉTHODE.** `calypso_c54x.c` a été **réécrit pendant l'analyse** (mtime 15:34 → 15:42, dérive ≈ +80 lignes). Les numéros de ligne ci-dessous valent pour le snapshot ci-après ; tout agent de lot DOIT re-grepper avant d'annoter.

```
md5  calypso_c54x.c        c36466abc34cef0a3c2120d9d281231f   (916952 o)
md5  calypso_bsp.c         4760924bcc8342cc5269b9bda69f4b07
md5  calypso_dsp_shunt.c   a0f17a26f902d238d93b1abb00a286a7
md5  calypso_dsp_helper.c  ef45b3d53b9e6304364c3a7978321924
md5  calypso_trx.c         8fbe30d88a4d488cc54125e7c79a9e52
md5  calypso_arm2dsp.c     866e6cb8f594809e4b041c7c0864ed02
```

Artefacts locaux (réutilisables par les agents de lot) :
`/root/.claude/jobs/26578783/tmp/raw2.txt` (378 lignes getenv brutes), `/root/.claude/jobs/26578783/tmp/FINAL2.md` (tableau), `/root/.claude/jobs/26578783/tmp/lots.json`, `/root/.claude/jobs/26578783/tmp/envdef.json`, `/root/.claude/jobs/26578783/tmp/src/` (copies des .c/.h du snapshot).

**312 variables** lues par le modèle QEMU (`hw/arm/calypso/*.c|*.h`) : 299 via `getenv()` direct + **13 via helpers** (`a2d_env_u16()` dans `calypso_arm2dsp.c:98`, `parse_uint_env()` dans `calypso_bsp.c`) — ces 13 sont invisibles à un `grep getenv` et ont été manquées par tous les inventaires précédents.

---

## (a) TABLEAU COMPLET — 312 variables

Préfixe `CALYPSO_` retiré de la colonne VARIABLE. Colonne LOT = découpage §(c).

Légende IDIOME :
- `EXISTS` = `getenv(X) ? 1 : 0` → **ACTIF même à `X=0`. Seul `unset X` coupe.**
- `EXISTS-INV` = `getenv(X) ? 0 : 1` → défaut ON, la simple présence coupe.
- `VAL>0` = `atoi(e) > 0` → `0` coupe.
- `EQ1` = `*e == '1'` → seule la valeur `1` active.
- `ON-sauf-0` = `(!e || *e != '0')` → défaut ON, seul `=0` coupe.
- `ON-sauf-VIDE` = `(e && *e == 0) ? 0 : 1` → **`=0` N'ÉTEINT PAS ; seule la chaîne vide éteint.**
- `NON-VIDE` = `(e && *e != 0)` → **`=0` ACTIVE** (la chaîne "0" est non vide).
- `INV-VAL` = `(e && atoi(e)) ? 0 : 1`.
- `LISTE` / `CHAINE` / `VALEUR` = paramètre, pas un gate booléen.
- `[gate DEBUG=TOK]` = la variable n'a d'effet **que si** le jeton `CALYPSO_DEBUG` correspondant est actif.

| # | VARIABLE | OCC | FICHIER:LIGNE | IDIOME | DEFAUT (env/code) | LOT |
|---|---|---|---|---|---|---|
| 1 | `AB38` | 1 | calypso_c54x.c:15595 | EXISTS | — (code) | 3 |
| 2 | `AR0_DEBUG` | 8 | calypso_c54x.c:3278 | EXISTS | — (code) | 6 |
| 3 | `AR2_FLOOR_DROP` | 1 | calypso_c54x.c:4616 | EQ1 [log gate DEBUG=AR2-FLOOR] | — | 6 |
| 4 | `AR6_AT_LOG_CAP` | 1 | calypso_c54x.c:428 | VALEUR [gate DEBUG=AR6-AT] | — | 6 |
| 5 | `AR6_AT_PC` | 1 | calypso_c54x.c:422 | VALEUR [gate DEBUG=AR6-AT] | — | 6 |
| 6 | `AR6_WIN_HI` | 1 | calypso_c54x.c:427 | VALEUR [gate DEBUG=AR6-AT] | — | 6 |
| 7 | `AR6_WIN_LO` | 1 | calypso_c54x.c:426 | VALEUR [gate DEBUG=AR6-AT] | — | 6 |
| 8 | `ARM2DSP` | 1 | calypso_arm2dsp.c:112 | VAL>0 | calypso.env:=0 | 4 |
| 9 | `ARM2DSP_BGEN` | 1 | calypso_arm2dsp.c:118 | VAL>0 | calypso.env:=1 ; native:=1 ; native_helped:=1 ; wire:=1 | 4 |
| 10 | `ARM2DSP_BGEN_A` | 1 | calypso_arm2dsp.c:120 | VALEUR (helper) | code: 0x098a | 4 |
| 11 | `ARM2DSP_BGEN_C` | 1 | calypso_arm2dsp.c:121 | VALEUR (helper) | code: 0x098c | 4 |
| 12 | `ARM2DSP_BGEN_ONESHOT` | 1 | calypso_arm2dsp.c:124 | VAL>0 (défaut 1 si absent) | — | 4 |
| 13 | `ARM2DSP_BGEN_POLLPC` | 1 | calypso_arm2dsp.c:123 | VALEUR (helper) | code: 0xdddb | 4 |
| 14 | `ARM2DSP_BGEN_VAL` | 1 | calypso_arm2dsp.c:122 | VALEUR (helper) | code: 0x0001 | 4 |
| 15 | `ARM2DSP_CONT` | 1 | calypso_arm2dsp.c:251 | EXISTS | — | 4 |
| 16 | `ARM2DSP_CTRLSYS` | 1 | calypso_arm2dsp.c:131 | VAL>0 | native:=0 ; native_helped:=0 ; wire:=1 | 4 |
| 17 | `ARM2DSP_CTRLSYS_CELL` | 1 | calypso_arm2dsp.c:133 | VALEUR (helper) | code: 0x0810 | 4 |
| 18 | `ARM2DSP_CTRLSYS_POLLPC` | 1 | calypso_arm2dsp.c:135 | VALEUR (helper) | code: 0xa537 | 4 |
| 19 | `ARM2DSP_CTRLSYS_VAL` | 1 | calypso_arm2dsp.c:134 | VALEUR (helper) | code: 0x8000 | 4 |
| 20 | `ARM2DSP_TASKBIT` | 1 | calypso_arm2dsp.c:115 | VALEUR (helper) | code: 0x0002 | 4 |
| 21 | `ARM2DSP_TASKWORD` | 1 | calypso_arm2dsp.c:114 | VALEUR (helper) | code: 0x0fff | 4 |
| 22 | `ARWATCH` | 1 | calypso_c54x.c:15206 | EXISTS | — | 6 |
| 23 | `AR_TRACE` | 1 | calypso_c54x.c:258 | VALEUR masque [gate DEBUG=AR-TRACE] | — | 6 |
| 24 | `A_TRACE_PC` | 1 | calypso_c54x.c:349 | VALEUR [gate DEBUG=A-TRACE] | — | 6 |
| 25 | `B1` | 2 | calypso_c54x.c:3066 | EXISTS | — | 3 |
| 26 | `B2` | 1 | calypso_c54x.c:15784 | EXISTS | — | 3 |
| 27 | `B2AR` | 2 | calypso_c54x.c:15181 | EXISTS | — | 3 |
| 28 | `B2IN` | 1 | calypso_dsp_shunt.c:2097 | EXISTS | — | 3 |
| 29 | `B2SEQ` | 2 | calypso_c54x.c:15652 | EXISTS | — | 3 |
| 30 | `B3_TRACE` | 1 | calypso_c54x.c:14491 | EXISTS | — | 3 |
| 31 | `B4` | 1 | calypso_c54x.c:3054 | EXISTS | — | 3 |
| 32 | `B4B` | 1 | calypso_c54x.c:15162 | EXISTS | — | 3 |
| 33 | `BACC_C827_OFF` | 1 | calypso_c54x.c:13768 | EXISTS-INV | — | 3 |
| 34 | `BOOTCMD` | 2 | calypso_c54x.c:3011 | EXISTS | — | 3 |
| 35 | `BSP_BIND_ADDR` | 1 | calypso_bsp.c:899 | VALEUR/chaine | — | 1 |
| 36 | `BSP_BIND_LOOPBACK` | 1 | calypso_bsp.c:900 | EQ1 | — | 1 |
| 37 | `BSP_BYPASS_BDLENA` | 1 | calypso_bsp.c:837 | VALEUR (helper) | code: 0 | 1 |
| 38 | `BSP_DARAM_ADDR` | 1 | calypso_bsp.c:823 | VALEUR (helper) | code: 0x2a00 (run.sh idem) | 1 |
| 39 | `BSP_DARAM_FORCE` | 3 | calypso_bsp.c:472 | EXISTS (ET DSP_RUN_C54X=1) | wire:=1 | 1 |
| 40 | `BSP_DARAM_LEN` | 1 | calypso_bsp.c:824 | VALEUR (helper) | code: 296 | 1 |
| 41 | `BSP_DIRECT_BRINT0` | 1 | calypso_bsp.c:1083 | EXISTS | calypso.env:=1 (bloc WIRE) ; wire:=1 | 1 |
| 42 | `BSP_DIRECT_FEED` | 1 | calypso_bsp.c:653 | EQ1 | calypso.env:=1 ; native:=1 | 1 |
| 43 | `BSP_DISPATCH_FB` | 1 | calypso_bsp.c:1146 | EXISTS | wire:=1 | 1 |
| 44 | `BSP_DISPATCH_FB_TGT` | 1 | calypso_bsp.c:1147 | VALEUR (déf 0x8d00) | — | 1 |
| 45 | `BSP_DISPATCH_NOIMR` | 1 | calypso_bsp.c:1165 | EXISTS | — | 1 |
| 46 | `BSP_DISPATCH_ONESHOT` | 1 | calypso_bsp.c:1149 | EXISTS | wire.env:**UNSET** | 1 |
| 47 | `BSP_FN_PROBE` | 1 | calypso_bsp.c:307 | EXISTS | calypso.env:=1 ; wire:=1 | 1 |
| 48 | `BSP_INJECT_CANARY` | 1 | calypso_bsp.c:847 | VALEUR (helper) | code: 0 | 1 |
| 49 | `BSP_IQ_DECIM` | 2 | calypso_bsp.c:591 | VALEUR (déf 4) | — | 1 |
| 50 | `BSP_IQ_PASSTHROUGH` | 1 | calypso_bsp.c:570 | ON-sauf-0 | run.sh:=1 | 1 |
| 51 | `BSP_IQ_SHIFT` | 1 | calypso_bsp.c:1233 | VALEUR (déf 0) | — | 1 |
| 52 | `BSP_PORT` | 1 | calypso_bsp.c:912 | VALEUR (déf 6702) | run.sh | 1 |
| 53 | `BSP_REPLAY_FILE` | 1 | calypso_bsp.c:868 | VALEUR/chaine | — | 1 |
| 54 | `C54X_BCTC_SM` | 1 | calypso_c54x.c:7721 | EXISTS | — | 2 |
| 55 | `C54X_CRASHPC` | 2 | calypso_dsp_shunt.c:840 | **AUCUN (echo log seul)** | calypso.env:=1 | 2 |
| 56 | `C54X_FIX_BC` | 1 | calypso_c54x.c:7697 | EXISTS | — | 2 |
| 57 | `C54X_FORCE_IMR` | 1 | calypso_c54x.c:14404 | VALEUR hex | — | 2 |
| 58 | `C54X_IRQ_LEVEL` | 1 | calypso_c54x.c:4933 | EXISTS (OU DSP=c54x) | — | 2 |
| 59 | `CALA_71DA` | 1 | calypso_c54x.c:14379 | EXISTS | — | 5 |
| 60 | `CALA_FB` | 1 | calypso_c54x.c:6235 | EXISTS | — | 5 |
| 61 | `CANNED` | 1 | calypso_dsp_shunt.c:324 | LISTE (absent=CAN_DEFAULT ; vide/NONE=0 ; FULL/ALL) | calypso.env:=(vide) ; run.sh full-grgsm:=NONE | 4 |
| 62 | `CORROUT` | 1 | calypso_c54x.c:15265 | EXISTS | — | 5 |
| 63 | `CORR_AR1` | 1 | calypso_c54x.c:14331 | VALEUR (déf 0x2f22) | — | 5 |
| 64 | `CORR_AR4` | 1 | calypso_c54x.c:14332 | VALEUR (déf 0x2be4) | — | 5 |
| 65 | `CORR_AR5` | 1 | calypso_c54x.c:14333 | VALEUR (déf 0x0060) | — | 5 |
| 66 | `CORR_BANK` | 1 | calypso_c54x.c:5113 | VALEUR (-1=off, 0..3=XPC forcé) | — | 5 |
| 67 | `CORR_FLOW` | 1 | calypso_c54x.c:5137 | EXISTS | — | 5 |
| 68 | `CORR_HI` | 1 | calypso_c54x.c:5195 | VALEUR [gate DEBUG=CORR-TRACE] | — | 5 |
| 69 | `CORR_LO` | 1 | calypso_c54x.c:5195 | VALEUR [gate DEBUG=CORR-TRACE] | — | 5 |
| 70 | `CORR_SETUP` | 1 | calypso_c54x.c:14329 | EXISTS | wire.env:**UNSET** | 5 |
| 71 | `CPU_IDLE` | 1 | calypso_trx.c:1235 | ON-sauf-0 | — | 1 |
| 72 | `D247` | 1 | calypso_c54x.c:12314 | EXISTS | — | 3 |
| 73 | `D247_TRACE_OFF` | 9 | calypso_c54x.c:12347 | INV-VAL | — | 3 |
| 74 | `DARAM_DUMP` | 1 | calypso_c54x.c:15666 | VALEUR/chemin | — | 1 |
| 75 | `DARAM_DUMP_ANYMODE` | 1 | calypso_c54x.c:15685 | VAL>0 | — | 1 |
| 76 | `DARAM_DUMP_MAX` | 1 | calypso_c54x.c:15672 | VALEUR | — | 1 |
| 77 | `DARAM_DUMP_PC` | 1 | calypso_c54x.c:15670 | VALEUR | — | 1 |
| 78 | `DA_HI` | 1 | calypso_c54x.c:14955 | VALEUR | — | 6 |
| 79 | `DA_INSN` | 1 | calypso_c54x.c:14956 | VALEUR | — | 6 |
| 80 | `DA_LO` | 1 | calypso_c54x.c:14955 | VALEUR | — | 6 |
| 81 | `DBG` | 1 | calypso_dbg.c:53 | LISTE (défaut corrupt,unimpl) | run.sh (menu DBG) | 2 |
| 82 | `DEBUG` | 1 | calypso_debug.c:51 | **LISTE (namespace de 115 sous-clés)** | run.sh:=ALL en mode debug | 2 |
| 83 | `DECAN` | 5 | calypso_dsp_shunt.c:564 | EQ1 (MAÎTRE : implique PM/SNR/TOA/ANGLE + REAL_FB) | native/native_helped/shunt_legit/shunt_no_legit:=1 | 6 |
| 84 | `DECAN_ANGLE` | 1 | calypso_dsp_shunt.c:779 | EQ1 (OU DECAN) | — | 6 |
| 85 | `DECAN_PM` | 2 | calypso_dsp_shunt.c:565 | EQ1 (OU DECAN) | — | 6 |
| 86 | `DECAN_PM_MAV_REF` | 1 | calypso_dsp_shunt.c:567 | VALEUR (déf 20929.0) | — | 6 |
| 87 | `DECAN_PM_RF_REF` | 1 | calypso_dsp_shunt.c:568 | VALEUR (déf -60.0) | — | 6 |
| 88 | `DECAN_SNR` | 2 | calypso_dsp_shunt.c:652 | EQ1 (OU DECAN) | — | 6 |
| 89 | `DECAN_TOA` | 1 | calypso_dsp_shunt.c:776 | EQ1 (OU DECAN) | — | 6 |
| 90 | `DEMODIO` | 1 | calypso_c54x.c:2934 | VAL>0 | — | 5 |
| 91 | `DEMODIO_AFTER` | 1 | calypso_c54x.c:2936 | VALEUR (déf 40e6 insn) | — | 5 |
| 92 | `DEMODIO_PCHI` | 1 | calypso_c54x.c:2938 | VALEUR (déf 0x9fe2) | — | 5 |
| 93 | `DEMODIO_PCLO` | 1 | calypso_c54x.c:2938 | VALEUR (déf 0x9f95) | — | 5 |
| 94 | `DEMODRD` | 1 | calypso_c54x.c:1836 | EXISTS | — | 5 |
| 95 | `DEMOD_NOCLOBBER` | 1 | calypso_c54x.c:3035 | VAL>0 | — | 3 |
| 96 | `DETTRACE` | 1 | calypso_c54x.c:14028 | EXISTS | — | 5 |
| 97 | `DISPIDX` | 1 | calypso_c54x.c:15537 | EXISTS | — | 3 |
| 98 | `DISPTAB` | 2 | calypso_c54x.c:3021 | EXISTS | — | 3 |
| 99 | `DISPWATCH` | 1 | calypso_c54x.c:15347 | EXISTS | — | 3 |
| 100 | `DL_FN_OFFSET` | 1 | calypso_trx.c:164 | VALEUR (déf 0) | — | 1 |
| 101 | `DMAWATCH` | 2 | calypso_c54x.c:1824 | EXISTS | — | 1 |
| 102 | `DSP` | 5 | calypso_c54x.c:4933 | CHAINE (=='c54x') | calypso.env:=c54x | 2 |
| 103 | `DSP_BLOB` | 1 | calypso_trx.c:1987 | VALEUR/chemin | run.sh | 2 |
| 104 | `DSP_BUDGET` | 2 | calypso_dsp_shunt.c:535 | VALEUR | run.sh:=256000 | 2 |
| 105 | `DSP_FRAME_VEC28` | 5 | calypso_bsp.c:1069 | EXISTS (OU FRAME_IT_NATIVE / DSP=c54x) | — | 2 |
| 106 | `DSP_GOLIVE_BOOT` | 2 | calypso_c54x.c:14675 | EXISTS | — | 2 |
| 107 | `DSP_IDLE_FF` | 1 | calypso_c54x.c:11193 | ON-sauf-0 | run.sh:=1 | 2 |
| 108 | `DSP_IDLE_RANGE` | 1 | calypso_c54x.c:11201 | VALEUR/chaine | run.sh:=(vide) | 2 |
| 109 | `DSP_REG_MODE` | 1 | calypso_c54x.c:16660 | CHAINE (c54x\|hybrid\|bin, déf bin) | run.sh | 2 |
| 110 | `DSP_RUN_C54X` | 6 | calypso_bsp.c:465 | EQ1 | calypso.env:=1 ; native:=1 ; native_helped:=1 ; shunt_legit:=0 ; shunt_no_legit:=0 | 2 |
| 111 | `DSP_SHUNT` | 2 | calypso_dsp_shunt.c:1855 | CHAINE (`strcmp=="1"`) | native:=0 ; native_helped:=0 ; shunt_legit:=1 ; shunt_no_legit:=1 ; run.sh par MODE | 2 |
| 112 | `DSP_TIMER_OFF` | 1 | calypso_c54x.c:16225 | EXISTS-INV | — | 2 |
| 113 | `DSP_YIELD` | 1 | calypso_c54x.c:16377 | VALEUR (insn/yield) | — | 2 |
| 114 | `ERRREAD` | 1 | calypso_trx.c:211 | EXISTS | — | 4 |
| 115 | `ERRWATCH` | 1 | calypso_c54x.c:2983 | EXISTS | — | 4 |
| 116 | `FBCALL` | 1 | calypso_c54x.c:15504 | EXISTS | — | 5 |
| 117 | `FBDET_API` | 2 | calypso_c54x.c:4153 | EXISTS | — | 5 |
| 118 | `FBDET_SENTINEL` | 1 | calypso_c54x.c:2689 | VALEUR (déf 0) | — | 5 |
| 119 | `FBENTRY` | 1 | calypso_c54x.c:15427 | EXISTS | — | 5 |
| 120 | `FBROUTE` | 1 | calypso_c54x.c:15619 | EXISTS | — | 5 |
| 121 | `FBWATCH` | 2 | calypso_c54x.c:2384 | EXISTS | — | 5 |
| 122 | `FB_CORR_ENTRY` | 1 | calypso_c54x.c:6255 | VALEUR (déf 0x94f5) | native:=0x9500 ; native_helped:=0x9500 | 5 |
| 123 | `FB_ENERGY` | 1 | calypso_c54x.c:6254 | VAL>0 | native:=1 ; native_helped:=1 | 5 |
| 124 | `FB_IQ_BASE` | 1 | calypso_dsp_shunt.c:1531 | VALEUR | native_helped:=0x9210 | 5 |
| 125 | `FB_IQ_DARAM` | 1 | calypso_dsp_shunt.c:1524 | VAL>0 | native_helped:=1 | 5 |
| 126 | `FB_IQ_FCCH_ONLY` | 1 | calypso_dsp_shunt.c:1527 | VAL>0 | — | 5 |
| 127 | `FB_IQ_MARKER` | 1 | calypso_dsp_shunt.c:1546 | VAL>0 | — | 5 |
| 128 | `FB_IQ_OWNS` | 1 | calypso_bsp.c:1220 | VAL>0 | calypso.env:=0 | 1 |
| 129 | `FB_STREAM` | 2 | calypso_c54x.c:1669 | EXISTS | native:=1 | 5 |
| 130 | `FB_STREAM_CELL` | 1 | calypso_c54x.c:1657 | VALEUR (déf 0x9213) | — | 5 |
| 131 | `FB_STREAM_CELLQ` | 1 | calypso_c54x.c:1658 | VALEUR (déf 0x9215) | — | 5 |
| 132 | `FB_STREAM_DECIM` | 1 | calypso_dsp_shunt.c:1584 | VALEUR (déf 1) | — | 5 |
| 133 | `FIND32` | 1 | calypso_trx.c:195 | EXISTS | — | 4 |
| 134 | `FIND32_VAL` | 1 | calypso_trx.c:196 | VALEUR | — | 4 |
| 135 | `FIRMWARE_ELF` | 1 | calypso_dsp_helper.c:59 | VALEUR/chemin | run.sh | 2 |
| 136 | `FIXES` | 2 | calypso_c54x.c:5059 | **LISTE (namespace 7 sous-clés) — SAS** | — | 3 |
| 137 | `FIX_3FCD` | 1 | calypso_c54x.c:2618 | EXISTS | wire:=1 | 3 |
| 138 | `FIX_DPAGE_OFF` | 1 | calypso_c54x.c:13656 | EXISTS-INV | — | 3 |
| 139 | `FIX_MVDM_OFF` | 1 | calypso_c54x.c:9222 | EXISTS-INV | — | 3 |
| 140 | `FIX_PORTR` | 1 | calypso_c54x.c:9090 | EXISTS | — | 3 |
| 141 | `FIX_SFTL_RSBX` | 2 | calypso_c54x.c:6932 | EXISTS | — | 3 |
| 142 | `FLOWTRACE` | 1 | calypso_c54x.c:2824 | VALEUR (budget) | — | 2 |
| 143 | `FORCE_098` | 1 | calypso_c54x.c:14433 | VALEUR | hack:=(vide) | 3 |
| 144 | `FORCE_3FAD_KERNEL` | 1 | calypso_c54x.c:2488 | EXISTS | — | 5 |
| 145 | `FORCE_3FAE` | 1 | calypso_c54x.c:5121 | EXISTS | — | 5 |
| 146 | `FORCE_AGCH` | 1 | l1ctl_sock.c:215 | EQ1 | hack:=0 | 5 |
| 147 | `FORCE_DISPATCH` | 1 | calypso_c54x.c:5474 | VAL>0 | wire:=1 | 3 |
| 148 | `FORCE_DP` | 1 | calypso_c54x.c:5494 | VALEUR | — | 3 |
| 149 | `FORCE_DP_FROM` | 1 | calypso_c54x.c:5496 | VALEUR (scope) | — | 3 |
| 150 | `FORCE_FBSB` | 1 | l1ctl_sock.c:214 | EQ1 | hack:=0 | 5 |
| 151 | `FORCE_GOLIVE` | 1 | calypso_c54x.c:14288 | VAL>0 | hack:=(vide) | 3 |
| 152 | `FORCE_INTM_AT_PC` | 1 | calypso_c54x.c:1024 | VALEUR | run.sh (menu INTM_PC) | 2 |
| 153 | `FORCE_INTM_ONESHOT` | 1 | calypso_c54x.c:1015 | EQ1 | wire:=1 | 2 |
| 154 | `FORCE_NB` | 1 | calypso_trx.c:373 | EQ1 | — | 5 |
| 155 | `FORCE_TOA` | 1 | calypso_trx.c:344 | VALEUR (-1=off) | run.sh | 5 |
| 156 | `FRAME_IT_LEVEL` | 1 | calypso_c54x.c:4920 | EQ1 | — | 2 |
| 157 | `FRAME_IT_NATIVE` | 5 | calypso_bsp.c:1069 | EXISTS (OU DSP_FRAME_VEC28) | native:=1 ; native_helped:=1 ; wire:=1 | 2 |
| 158 | `FRAME_IT_PRIO` | 1 | calypso_c54x.c:4926 | EQ1 | — | 2 |
| 159 | `FRAME_IT_PROBE` | 1 | calypso_c54x.c:16864 | EXISTS | — | 2 |
| 160 | `GOLIVE_TASKW` | 1 | calypso_c54x.c:14456 | EQ1 | — | 3 |
| 161 | `IDLE_PC_HI` | 1 | calypso_trx.c:1237 | VALEUR (déf 0x826000) | — | 1 |
| 162 | `IDLE_PC_LO` | 1 | calypso_trx.c:1236 | VALEUR (déf 0x823000) | — | 1 |
| 163 | `INITTAB` | 1 | calypso_c54x.c:12270 | EXISTS | — | 3 |
| 164 | `INIT_435B_OFF` | 1 | calypso_c54x.c:13673 | INV-VAL (=1 coupe l'init) | hack/native/native_helped/wire:=0 | 2 |
| 165 | `INJECT_ACD` | 1 | calypso_dsp_helper.c:345 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 166 | `INJECT_AGCH` | 1 | calypso_dsp_shunt.c:942 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 167 | `INJECT_FB` | 1 | calypso_dsp_helper.c:233 | EQ1 (**pas** de fallback LEGIT) | — | 4 |
| 168 | `INJECT_SACCH` | 1 | calypso_dsp_shunt.c:1114 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 169 | `INJECT_SB` | 1 | calypso_dsp_helper.c:282 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 170 | `INJECT_SDCCH` | 1 | calypso_dsp_shunt.c:1070 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 171 | `INSTALL_TRACE_OFF` | 1 | calypso_c54x.c:13750 | EXISTS-INV | — | 3 |
| 172 | `INTM_TRANS` | 1 | calypso_c54x.c:14987 | **NON-VIDE (=0 l'ACTIVE)** | calypso.env:=1 ; wire:=1 | 2 |
| 173 | `INVARIANTS` | 1 | calypso_invariants.c:25 | EQ1 | — | 2 |
| 174 | `IQDUMP` | 3 | calypso_bsp.c:1260 | EXISTS (OU `BSP_DUMP_RX_FILE`) | — | 1 |
| 175 | `IQDUMP_FCCH` | 1 | calypso_bsp.c:1031 | EXISTS | — | 1 |
| 176 | `IQ_CFILE_SPF` | 2 | calypso_dsp_shunt.c:1794 | VALEUR (déf 2500) | — | 1 |
| 177 | `IQ_TEE_HOST` | 1 | calypso_bsp.c:410 | VALEUR/IP (déf 127.0.0.1) | — | 1 |
| 178 | `IQ_TEE_PORT` | 1 | calypso_bsp.c:405 | VALEUR (déf 6703) | run.sh:=6703 | 1 |
| 179 | `ISR_TO_8341` | 1 | calypso_c54x.c:14307 | EXISTS | wire.env:**UNSET** | 3 |
| 180 | `IT_PUSH_XPC_ALWAYS` | 1 | calypso_c54x.c:5025 | **NON-VIDE (=0 l'ACTIVE)** | — | 2 |
| 181 | `KEEP_IMR` | 1 | calypso_c54x.c:14268 | EXISTS | hack/native/native_helped/wire:=1 | 2 |
| 182 | `KEEP_IMR_VAL` | 1 | calypso_c54x.c:14269 | VALEUR | — | 2 |
| 183 | `L1` | 1 | calypso_layer1.c:55 | CHAINE (`e[0]=='c'`) | — | 4 |
| 184 | `L1S_FN_ADDR` | 1 | calypso_dsp_helper.c:124 | VALEUR/adresse | run.sh | 4 |
| 185 | `L1_RESET_WIRE` | 1 | calypso_dsp_shunt.c:180 | ON-sauf-0 | — | 4 |
| 186 | `LAST_RACH_FN_ADDR` | 1 | calypso_dsp_helper.c:147 | VALEUR/adresse | run.sh | 4 |
| 187 | `LDK8_SHIFT16` | 1 | calypso_c54x.c:8801 | EXISTS | — | 3 |
| 188 | `MASKROM_GOLIVE` | 1 | calypso_c54x.c:13825 | EXISTS | — | 3 |
| 189 | `MASKROM_INIT` | 1 | calypso_c54x.c:12289 | EXISTS | — | 3 |
| 190 | `MEM_WATCH_2B80` | 2 | calypso_c54x.c:1868 | EXISTS | — | 1 |
| 191 | `MVPD_BOOT_LIMIT` | 1 | calypso_c54x.c:700 | VALEUR | run.sh | 3 |
| 192 | `NDB_D_RACH_OFFSET` | 2 | calypso_bsp.c:1747 | VALEUR (vide=unset) | run.sh:=(vide) | 1 |
| 193 | `ORCH` | 1 | calypso_orch.h:14 | ON-sauf-0 (mais absent = OFF) | — | 2 |
| 194 | `ORPHAN` | 2 | calypso_c54x.c:2508 | EXISTS | — | 2 |
| 195 | `PCB_TICK_THREADS` | 2 | calypso_tint0.c:60 | EQ1 | run.sh | 2 |
| 196 | `PHASE_SM_OFF` | 1 | calypso_c54x.c:13786 | EXISTS-INV | — | 3 |
| 197 | `POKE_A4C7_ONCE` | 1 | calypso_c54x.c:14359 | VAL>0 | hack:=(vide) | 3 |
| 198 | `POKE_DISPATCH` | 1 | calypso_bsp.c:1123 | VAL>0 (déf 0) | — | 1 |
| 199 | `POKE_TASK_MD` | 1 | calypso_bsp.c:1116 | VAL>0 — **DÉFAUT ON (=1 si absent)** | — | 1 |
| 200 | `PROBE_3FAD_GATE` | 1 | calypso_c54x.c:2476 | EXISTS | — | 5 |
| 201 | `RACH_FORCE_BSIC` | 1 | calypso_bsp.c:1770 | VALEUR (vide=unset) | run.sh:=(vide) | 1 |
| 202 | `REDIR7000` | 1 | calypso_c54x.c:12257 | EXISTS | — | 3 |
| 203 | `REDIR_LEGACY` | 1 | calypso_c54x.c:12248 | EXISTS | — | 3 |
| 204 | `REPOPULATE` | 1 | calypso_c54x.c:12325 | EXISTS | — | 3 |
| 205 | `REQREF_ADJ` | 1 | calypso_dsp_shunt.c:1023 | VALEUR (déf 0) | calypso.env:=(vide) | 6 |
| 206 | `REQREF_LAST_RACH` | 1 | calypso_dsp_shunt.c:1030 | ON-sauf-0 | — | 6 |
| 207 | `REQREF_PERRA` | 1 | calypso_dsp_shunt.c:1022 | ON-sauf-0 | — | 6 |
| 208 | `REQREF_REWRITE` | 1 | calypso_dsp_shunt.c:1021 | EQ1 | — | 6 |
| 209 | `RMAP` | 1 | calypso_c54x.c:1777 | VAL>0 | — | 6 |
| 210 | `RMAP_PCHI` | 1 | calypso_c54x.c:1779 | VALEUR (déf 0x9fff) | — | 6 |
| 211 | `RMAP_PCLO` | 1 | calypso_c54x.c:1779 | VALEUR (déf 0x9f00) | — | 6 |
| 212 | `RX_FBFLAGS` | 2 | calypso_bsp.c:1102 | EXISTS | — | 1 |
| 213 | `SCAN43D8` | 1 | calypso_c54x.c:15461 | EXISTS | — | 4 |
| 214 | `SCANDATA` | 1 | calypso_c54x.c:15319 | EXISTS | — | 4 |
| 215 | `SCANDATA_HI` | 1 | calypso_c54x.c:15321 | VALEUR | — | 4 |
| 216 | `SCANDATA_LO` | 1 | calypso_c54x.c:15320 | VALEUR | — | 4 |
| 217 | `SCANFB` | 1 | calypso_c54x.c:15399 | EXISTS | — | 5 |
| 218 | `SCANREF` | 1 | calypso_c54x.c:15371 | VALEUR | — | 5 |
| 219 | `SCAN_08F8` | 1 | calypso_c54x.c:15148 | EXISTS | — | 4 |
| 220 | `SEED5AC8` | 1 | calypso_c54x.c:14111 | VAL>0 | hack:=(vide) | 2 |
| 221 | `SEED5AC8_VAL` | 1 | calypso_c54x.c:14123 | VALEUR | wire:=0xa4c7 | 2 |
| 222 | `SEED_52FD` | 1 | calypso_c54x.c:13678 | EXISTS (→0x52fd sinon 0x52ed) | — | 2 |
| 223 | `SHUNT_AGCH` | 1 | calypso_dsp_helper.c:378 | ON-sauf-0 | — | 6 |
| 224 | `SHUNT_AGCH_EXPIRE` | 1 | calypso_dsp_shunt.c:722 | VAL>0 | shunt_no_legit:=1 | 6 |
| 225 | `SHUNT_AGCH_OFS` | 1 | calypso_dsp_helper.c:379 | VALEUR (déf 0) | — | 6 |
| 226 | `SHUNT_AGCH_TTL` | 3 | calypso_dsp_helper.c:380 | VALEUR | — | 6 |
| 227 | `SHUNT_BCCH_OFS` | 1 | calypso_dsp_helper.c:593 | VALEUR (déf 0) | — | 6 |
| 228 | `SHUNT_BCCH_SCHED` | 1 | calypso_dsp_helper.c:591 | EQ1 | — | 6 |
| 229 | `SHUNT_BURST_FN` | 1 | calypso_dsp_helper.c:187 | VALEUR (déf 0) | shunt_no_legit:=1 | 6 |
| 230 | `SHUNT_BURST_M1` | 1 | calypso_dsp_helper.c:193 | EXISTS | — | 6 |
| 231 | `SHUNT_BURST_OFS` | 1 | calypso_dsp_helper.c:191 | VALEUR | shunt_no_legit:=-1 | 6 |
| 232 | `SHUNT_BURST_PERCMD` | 2 | calypso_dsp_shunt.c:199 | **ON-sauf-VIDE (=0 NE COUPE PAS)** | — | 6 |
| 233 | `SHUNT_CANNED` | 1 | calypso_dsp_helper.c:652 | EXISTS | — | 4 |
| 234 | `SHUNT_DL_INJECT` | 1 | calypso_dsp_shunt.c:2029 | EQ1 | hack:=0 ; shunt_no_legit:=1 ; run.sh:=0 | 4 |
| 235 | `SHUNT_DRIVE_DSP` | 1 | calypso_dsp_shunt.c:612 | EQ1 | — | 4 |
| 236 | `SHUNT_DSP_FB` | 1 | calypso_dsp_shunt.c:1693 | EQ1 | — | 5 |
| 237 | `SHUNT_DSP_FB_BUDGET` | 1 | calypso_dsp_shunt.c:1697 | VALEUR | — | 5 |
| 238 | `SHUNT_DSP_FB_ENTRY` | 1 | calypso_dsp_shunt.c:1695 | VALEUR | — | 5 |
| 239 | `SHUNT_DSP_FB_MAX` | 1 | calypso_dsp_shunt.c:1699 | VALEUR | — | 5 |
| 240 | `SHUNT_DSP_FB_SP` | 1 | calypso_dsp_shunt.c:1701 | VALEUR | — | 5 |
| 241 | `SHUNT_DUAL_PAGE` | 1 | calypso_dsp_helper.c:653 | ON-sauf-0 | — | 4 |
| 242 | `SHUNT_FEED_SI` | 1 | calypso_dsp_shunt.c:1966 | EQ1 (OU SHUNT_LEGIT=1) | shunt_no_legit:=1 | 4 |
| 243 | `SHUNT_GSMTAP_PORT` | 1 | calypso_dsp_shunt.c:1201 | VALEUR | run.sh | 6 |
| 244 | `SHUNT_IQ_CFILE` | 1 | calypso_dsp_shunt.c:1406 | VALEUR/chemin (absent→/root/dsp_iq.cfile ; vide=off) | calypso.env:=/dev/shm/dsp_iq.fifo | 6 |
| 245 | `SHUNT_IQ_CFILE2` | 1 | calypso_dsp_shunt.c:1448 | VALEUR/chemin | — | 6 |
| 246 | `SHUNT_IQ_RECORD` | 1 | calypso_dsp_shunt.c:1435 | VALEUR/chemin | calypso.env:=/dev/shm/dsp_iq.cfile | 6 |
| 247 | `SHUNT_LEGIT` | **16** | calypso_c54x.c:2698 | EQ1 (post value-list) | calypso.env:=0 ; native:=0 ; native_helped:=0 ; shunt_legit:=1 ; shunt_no_legit:=0 | 4 |
| 248 | `SHUNT_NO_CANNED` | 3 | calypso_dsp_helper.c:291 | EQ1 | run.sh full-grgsm:=1 (sinon 0) | 4 |
| 249 | `SHUNT_NO_FAKE_FB` | 1 | calypso_dsp_shunt.c:851 | EQ1 | hack:=1 | 4 |
| 250 | `SHUNT_NO_FAKE_PM` | 2 | calypso_dsp_shunt.c:845 | EQ1 | hack:=0 | 4 |
| 251 | `SHUNT_NO_GRGSM` | 1 | calypso_dsp_shunt.c:1294 | EQ1 | — | 4 |
| 252 | `SHUNT_NO_LEGIT` | 5 | calypso_c54x.c:2698 | EQ1 (post value-list) | calypso.env:=0 ; native:=0 ; native_helped:=0 | 4 |
| 253 | `SHUNT_PM` | 1 | calypso_dsp_helper.c:688 | VALEUR (-1=modèle) | — | 6 |
| 254 | `SHUNT_REAL_FB` | 3 | calypso_dsp_helper.c:241 | EQ1 (**OU DECAN=1**, shunt.c:1594) | calypso.env:=0 ; hack:=0 ; native:=0 ; shunt_no_legit:=1 | 5 |
| 255 | `SHUNT_SACCH` | 1 | calypso_dsp_helper.c:513 | ON-sauf-0 | — | 6 |
| 256 | `SHUNT_SACCH_OFS` | 1 | calypso_dsp_helper.c:533 | VALEUR (déf 0) | — | 6 |
| 257 | `SHUNT_SACCH_PAR` | 1 | calypso_dsp_helper.c:532 | VALEUR (déf 2) | — | 6 |
| 258 | `SHUNT_SCH_PORT` | 1 | calypso_dsp_shunt.c:1306 | VALEUR | — | 6 |
| 259 | `SHUNT_SDCCH` | 1 | calypso_dsp_helper.c:427 | ON-sauf-0 | — | 6 |
| 260 | `SHUNT_SDCCH_MAXPRES` | 1 | calypso_dsp_helper.c:473 | VALEUR (déf 8) | — | 6 |
| 261 | `SHUNT_SDCCH_OFS` | 1 | calypso_dsp_helper.c:428 | VALEUR (déf 0) | — | 6 |
| 262 | `SHUNT_SDCCH_RING` | 2 | calypso_dsp_helper.c:432 | ON-sauf-0 | — | 6 |
| 263 | `SHUNT_SDCCH_TTL` | 1 | calypso_dsp_helper.c:429 | VALEUR | — | 6 |
| 264 | `SHUNT_SI_ROT_MASK` | 1 | calypso_dsp_shunt.c:742 | VALEUR (déf 7) | — | 6 |
| 265 | `SIM_CFG` | 1 | calypso_sim.c:787 | VALEUR/chemin | run.sh:=$MOBILE_CFG | 2 |
| 266 | `SLOTSRC` | 2 | calypso_c54x.c:1852 | EXISTS | — | 1 |
| 267 | `SM_TRACE` | 1 | calypso_c54x.c:14474 | EXISTS | — | 3 |
| 268 | `SP_HIST_ARM` | 1 | calypso_c54x.c:11588 | VALEUR | run.sh | 2 |
| 269 | `SP_HIST_DUMP` | 1 | calypso_c54x.c:11589 | VALEUR | run.sh | 2 |
| 270 | `SP_RING_INSN_MIN` | 1 | calypso_c54x.c:11443 | VALEUR | run.sh | 2 |
| 271 | `SP_RING_MAX` | 1 | calypso_c54x.c:11434 | VALEUR | run.sh | 2 |
| 272 | `SP_RING_TRIG` | 1 | calypso_c54x.c:11438 | CHAINE (bootstub\|floor\|both) | run.sh | 2 |
| 273 | `TASKGO` | 1 | calypso_c54x.c:15566 | EXISTS | — | 3 |
| 274 | `TDMA_NS` | 1 | calypso_trx.c:1050 | VALEUR | — | 1 |
| 275 | `TDMA_REALTIME` | 1 | calypso_trx.c:1301 | EQ1 | calypso.env:=1 ; run.sh:=1 | 1 |
| 276 | `TERM_TRACE_OFF` | 1 | calypso_c54x.c:13699 | EXISTS-INV | — | 3 |
| 277 | `TEST_3FCD` | 1 | calypso_c54x.c:13633 | EXISTS | — | 3 |
| 278 | `TINT0_MASTER` | 3 | calypso_c54x.c:12847 | EXISTS | calypso.env:=1 (bloc WIRE) ; wire:=1 | 2 |
| 279 | `TINT0_PERINSN` | 1 | calypso_c54x.c:16219 | EXISTS | — | 2 |
| 280 | `TINT0_PERIOD` | 1 | calypso_c54x.c:12849 | VALEUR (déf 1500) | calypso.env:=1500 ; wire:=1500 | 2 |
| 281 | `TPU_RX_WIRE` | 3 | calypso_bsp.c:1353 + calypso_trx.c:956 | EXISTS (**OU** DSP_RUN_C54X=1 ET BSP_DARAM_FORCE) | wire:=1 | 1 |
| 282 | `TRACEFROM` | 1 | calypso_c54x.c:15226 | VALEUR | — | 3 |
| 283 | `TRACEFROM_N` | 1 | calypso_c54x.c:15229 | VALEUR | — | 3 |
| 284 | `TRACE_LDU_PC` | 1 | calypso_c54x.c:9597 | VALEUR | — | 3 |
| 285 | `TRACE_STLD_PC` | 1 | calypso_c54x.c:11088 | VALEUR | — | 3 |
| 286 | `TRACE_VEC28_STACK` | 2 | calypso_c54x.c:16980 | EXISTS | — | 2 |
| 287 | `TRACK_STKVAL` | 1 | calypso_c54x.c:2520 | VALEUR | — | 2 |
| 288 | `TRAP_CHECKPOINT` | 1 | calypso_c54x.c:16108 | VALEUR | run.sh (menu TRAP_CP) | 3 |
| 289 | `TRF_RXLEV` | 3 | calypso_c54x.c:2710 | EQ1 (**OU SHUNT_LEGIT=1**) | shunt_no_legit:=1 | 6 |
| 290 | `TRF_TARGET_RF` | 3 | calypso_c54x.c:2712 | VALEUR (déf -60) | — | 6 |
| 291 | `TRF_TSP_DEV` | 1 | calypso_trf6151.c:70 | VALEUR (déf 1) | — | 6 |
| 292 | `TWL3025_AFC` | 1 | calypso_twl3025.c:99 | ON-sauf-0 | — | 6 |
| 293 | `TWL3025_AFC_HZ` | 1 | calypso_twl3025.c:94 | VALEUR (0=boucle AFC réelle) | hack:=0 (export explicite) | 6 |
| 294 | `UL_ACU_OFS` | 1 | calypso_dsp_shunt.c:224 | VALEUR (déf 6) | — | 6 |
| 295 | `UL_PUB_IDLE` | 1 | calypso_dsp_shunt.c:154 | EQ1 | — | 6 |
| 296 | `UL_RACH_FROM_DRACH` | 1 | calypso_trx.c:783 | EQ1 | shunt_no_legit:=1 | 6 |
| 297 | `VECTAB` | 1 | calypso_c54x.c:15290 | EXISTS | — | 2 |
| 298 | `WATCH_0810` | 1 | calypso_c54x.c:2597 | EXISTS | — | 4 |
| 299 | `WATCH_2A00` | 1 | calypso_c54x.c:2548 | EXISTS | native_helped:=1 | 5 |
| 300 | `WATCH_9200` | 1 | calypso_c54x.c:2563 | EXISTS | — | 5 |
| 301 | `WATCH_9F00_RD` | 1 | calypso_c54x.c:1683 | EXISTS | — | 5 |
| 302 | `WATCH_ACD` | 1 | calypso_c54x.c:2535 | EXISTS | — | 4 |
| 303 | `WATCH_RD_ADDR` | 1 | calypso_c54x.c:1931 | VALEUR [+ DEBUG=WATCH-RD] | — | 1 |
| 304 | `WATCH_RESULT` | 1 | calypso_c54x.c:2578 | EXISTS | — | 5 |
| 305 | `WATCH_VEC` | 1 | calypso_c54x.c:3424 | **NON-VIDE (=0 l'ACTIVE)** | — | 2 |
| 306 | `WATCH_WR_ADDR` | 1 | calypso_c54x.c:3701 | VALEUR | — | 1 |
| 307 | `WMAP` | 1 | calypso_c54x.c:2885 | VAL>0 | — | 1 |
| 308 | `WMAP_HI` | 1 | calypso_c54x.c:2887 | VALEUR | — | 1 |
| 309 | `WMAP_HI2` | 1 | calypso_c54x.c:2890 | VALEUR | — | 1 |
| 310 | `WMAP_LO` | 1 | calypso_c54x.c:2887 | VALEUR (déf 0x2c00) | — | 1 |
| 311 | `WMAP_LO2` | 1 | calypso_c54x.c:2890 | VALEUR | — | 1 |
| 312 | `WZWRITE` | 2 | calypso_c54x.c:1811 | EXISTS | — | 1 |

### Annexe A1 — variables lues HORS `hw/arm/calypso/` (même namespace, autre binaire)

Ces variables ont un défaut posé dans `calypso.env` mais **ne sont pas lues par le modèle** ; les annoter dans les .c du modèle est impossible.

- `hw/timer/calypso_timer.c` : `TDMA_REALTIME`(:56), `LOST_LATCH`(:153), `LOST_READ_DRIVEN`(:173)
- `hw/char/calypso_uart.c` : `UART_TRACE`(:664)
- `tools/calypso-ipc-device/qemu_wrap.c` (48 getenv) : `UL_*` (23 vars : `UL_DEBUG, UL_AMP, UL_RA, UL_RACH_ENC, UL_RACH_STICKY, UL_FN_GATE, UL_FN_OFFSET, UL_FN_ADJ, UL_FN_LOCK, UL_SLOT_OFFSET, UL_SDCCH, UL_SDCCH_OFS, UL_SDCCH_SMP_OFS, UL_SABM_HOLD, UL_SABM_HOLD_TTL, UL_SABM_STICKY, UL_ROT, UL_ROT_SGN, UL_INVERT, UL_GMSK, UL_BSIC, UL_ACTIVE_SYMS, UL_IQ_RECORD`), `CIPH_A5`, `CIPH_FN_ADJ`, `FCCH_DUMP*`, `QFN_*`, `DL_FIFO_CATCHUP_OFF`, `DL_IQ_CONJ`, `DL_BURST_OFFSET`, `BSP_CONT_FORWARD`, `IPC_RELAY`, `IPC_UL`, `RELAY_*`, `TRX_IQ_*`
- `CALYPSO_START_FN`, `CALYPSO_NB_MAXDLY` : **absentes de tout le source osmo-qemu-calypso** → lues par osmo-trx (hors arbre) ou MORTES.

---

## (b) JETONS `CALYPSO_DEBUG` — SOUS-CLÉS, PAS DES VARIABLES

Mécanisme : `calypso_debug.c:51` parse UNE variable en liste séparée par virgules ; normalisation upper-case + `-`/` `/`.`/`/` → `_` ; jeton spécial `ALL`. Master-gate `calypso_debug_master` (=0 si vide) court-circuite 127 sites inline. **115 sites d'appel, 105 jetons distincts** (`calypso_debug_enabled()` et `cdbg_env()`) :

```
A-TRACE  AFC-APPLY  AR-TRACE  AR2-FLOOR  AR2-WR  AR6-AT  AR7-HIST  AR_CLOBBER
A_CD-BY-BURST  A_SCH-WR  BITF-PROBE  BOOT-BRANCH  BOOTSTUB  BOOTSTUB-ENTRY
BOOTSTUB_TRAIL  BSP  BSP-DELIVER  BSP-RXBURST  C54X  CALAD-ZONE-W  CALLSITE
CORR-TRACE  CORRELATOR  DARAM  DATA-W-MMR  DECODE-AUDIT  DISP-ENTRY  DISP-FLAG-W
DISP-POLL  DISP-PTR  DISP-TRACE  DISP-WRITE  DISPATCH-ENTRY  DROM-W-DROP
DSP_WRITE_COUNT  D_BURST_D-SUMMARY  D_BURST_D-WR  D_BURST_D_W-SUMMARY  D_DSP_PAGE
D_TASK_D-WR  D_TASK_MD-RD  D_TASK_MD_ALL  FBDB  HIT-76  IMR-W  INSN-COUNT-STATS
INT3-BLOCKED  INT3-BLOCKED-SAMPLE  INT3-CYCLE  INT3_VEC  INTM-TRANS  IOTA
IRQ-FRAME-HEALTH  MVMD-AR7-BRC  MVPD  NDB-CTL-WR  PAGE_SPLIT  PC-HIST-3DD
PC-HIST-3FB  PCB  PORTR-DEST-HIST  PUMP  READ-AMONT  ROMMAP  RPTB-ARMED
RSBX-INTM  RXDONE  R_PAGE_SPLIT  SOFT_RESET_TRAIL  SP-ABS  SP-CATASTROPHE
SP-DRAIN  SP-GUARD  SP-HIST  SP-LOW  SP-RING  SP-WATCH  ST1-WR  STACK-IN-NDB
STUCK  STUCK-HIST  TIMER  TPU  TPU_RAM  TRAP  TRAP-OOR  TRX  UPPER-DARAM
WATCH-3FBE  WATCH-WRITE  XC-COND  XPC-STATS  XPC-WR  XPC1-PC-RING  YIELD-BREAK
+ ALL (méta)
```

**Piège à documenter** : `INTM-TRANS` (jeton) et `CALYPSO_INTM_TRANS` (variable) sont **deux choses différentes** ; idem `CORRELATOR`/`CORR-TRACE` (jetons) vs `CALYPSO_CORRELATOR_TRACE` (variable MORTE, cf §d). `ORPHAN` a été volontairement sorti de `CALYPSO_DEBUG` vers une env dédiée (`calypso_c54x.c:2508`, commentaire « anti-Heisenbug : master reste 0 »).

### Second namespace de sous-clés : `CALYPSO_FIXES` (SAS)

`calypso_fix_enabled()` @ `calypso_c54x.c:5057`, valeur `all` ou liste. 7 sous-clés :
`FIX_BIT_XMEM · FIX_BRINT0_UNMASK · FIX_LD_PARALLEL · FIX_LD_XMEM_SHFT · FIX_STL_B_ASM · FIX_ST_TRN · FIX_SUB_XMEM_YMEM`
(le commentaire du gate dit lui-même : « un sas se vide, ne jamais y laisser vieillir un correctif validé »). Le code contient aussi un test `calypso_fix_enabled("FIX_SUB16_SRC")` @5406 dont le nom **n'apparaît dans aucune doc** — à croiser.

---

## (c) LES 6 LOTS (découpage à donner tel quel aux agents suivants)

Écart assumé au cahier des charges : le corpus fait **312** variables, pas ~200 ; des lots de 15-40 auraient exigé 9-12 agents. Lots calibrés **47-59**, homogènes par famille.

### LOT 1 — BSP / DMA / DARAM / feed I-Q / horloge TDMA (52)
Fichiers pivots : `calypso_bsp.c`, `calypso_trx.c` (horloge), dumps mémoire.
```
BSP_BIND_ADDR BSP_BIND_LOOPBACK BSP_BYPASS_BDLENA BSP_DARAM_ADDR BSP_DARAM_FORCE
BSP_DARAM_LEN BSP_DIRECT_BRINT0 BSP_DIRECT_FEED BSP_DISPATCH_FB BSP_DISPATCH_FB_TGT
BSP_DISPATCH_NOIMR BSP_DISPATCH_ONESHOT BSP_FN_PROBE BSP_INJECT_CANARY BSP_IQ_DECIM
BSP_IQ_PASSTHROUGH BSP_IQ_SHIFT BSP_PORT BSP_REPLAY_FILE CPU_IDLE DARAM_DUMP
DARAM_DUMP_ANYMODE DARAM_DUMP_MAX DARAM_DUMP_PC DL_FN_OFFSET DMAWATCH FB_IQ_OWNS
IDLE_PC_HI IDLE_PC_LO IQDUMP IQDUMP_FCCH IQ_CFILE_SPF IQ_TEE_HOST IQ_TEE_PORT
MEM_WATCH_2B80 NDB_D_RACH_OFFSET POKE_DISPATCH POKE_TASK_MD RACH_FORCE_BSIC
RX_FBFLAGS SLOTSRC TDMA_NS TDMA_REALTIME TPU_RX_WIRE WATCH_RD_ADDR WATCH_WR_ADDR
WMAP WMAP_HI WMAP_HI2 WMAP_LO WMAP_LO2 WZWRITE
```

### LOT 2 — Cœur c54x : IT / IMR / INTM / TINT0 / cadence / go-live shadow / infra debug (52)
```
C54X_BCTC_SM C54X_CRASHPC C54X_FIX_BC C54X_FORCE_IMR C54X_IRQ_LEVEL DBG DEBUG DSP
DSP_BLOB DSP_BUDGET DSP_FRAME_VEC28 DSP_GOLIVE_BOOT DSP_IDLE_FF DSP_IDLE_RANGE
DSP_REG_MODE DSP_RUN_C54X DSP_SHUNT DSP_TIMER_OFF DSP_YIELD FIRMWARE_ELF FLOWTRACE
FORCE_INTM_AT_PC FORCE_INTM_ONESHOT FRAME_IT_LEVEL FRAME_IT_NATIVE FRAME_IT_PRIO
FRAME_IT_PROBE INIT_435B_OFF INTM_TRANS INVARIANTS IT_PUSH_XPC_ALWAYS KEEP_IMR
KEEP_IMR_VAL ORCH ORPHAN PCB_TICK_THREADS SEED5AC8 SEED5AC8_VAL SEED_52FD SIM_CFG
SP_HIST_ARM SP_HIST_DUMP SP_RING_INSN_MIN SP_RING_MAX SP_RING_TRIG TINT0_MASTER
TINT0_PERINSN TINT0_PERIOD TRACE_VEC28_STACK TRACK_STKVAL VECTAB WATCH_VEC
```

### LOT 3 — Opcodes / ISA / mask-ROM / bootstrap / dispatcher / redirections (50)
```
AB38 B1 B2 B2AR B2IN B2SEQ B3_TRACE B4 B4B BACC_C827_OFF BOOTCMD D247
D247_TRACE_OFF DEMOD_NOCLOBBER DISPIDX DISPTAB DISPWATCH FIXES FIX_3FCD
FIX_DPAGE_OFF FIX_MVDM_OFF FIX_PORTR FIX_SFTL_RSBX FORCE_098 FORCE_DISPATCH
FORCE_DP FORCE_DP_FROM FORCE_GOLIVE GOLIVE_TASKW INITTAB INSTALL_TRACE_OFF
ISR_TO_8341 LDK8_SHIFT16 MASKROM_GOLIVE MASKROM_INIT MVPD_BOOT_LIMIT PHASE_SM_OFF
POKE_A4C7_ONCE REDIR7000 REDIR_LEGACY REPOPULATE SM_TRACE TASKGO TERM_TRACE_OFF
TEST_3FCD TRACEFROM TRACEFROM_N TRACE_LDU_PC TRACE_STLD_PC TRAP_CHECKPOINT
```

### LOT 4 — Pont ARM↔DSP + parapluies shunt + injections + canned (47)
```
ARM2DSP ARM2DSP_BGEN ARM2DSP_BGEN_A ARM2DSP_BGEN_C ARM2DSP_BGEN_ONESHOT
ARM2DSP_BGEN_POLLPC ARM2DSP_BGEN_VAL ARM2DSP_CONT ARM2DSP_CTRLSYS
ARM2DSP_CTRLSYS_CELL ARM2DSP_CTRLSYS_POLLPC ARM2DSP_CTRLSYS_VAL ARM2DSP_TASKBIT
ARM2DSP_TASKWORD CANNED ERRREAD ERRWATCH FIND32 FIND32_VAL INJECT_ACD INJECT_AGCH
INJECT_FB INJECT_SACCH INJECT_SB INJECT_SDCCH L1 L1S_FN_ADDR L1_RESET_WIRE
LAST_RACH_FN_ADDR SCAN43D8 SCANDATA SCANDATA_HI SCANDATA_LO SCAN_08F8 SHUNT_CANNED
SHUNT_DL_INJECT SHUNT_DRIVE_DSP SHUNT_DUAL_PAGE SHUNT_FEED_SI SHUNT_LEGIT
SHUNT_NO_CANNED SHUNT_NO_FAKE_FB SHUNT_NO_FAKE_PM SHUNT_NO_GRGSM SHUNT_NO_LEGIT
WATCH_0810 WATCH_ACD
```

### LOT 5 — FB / FBSB / corrélateur / démod / résultat (52)
```
CALA_71DA CALA_FB CORROUT CORR_AR1 CORR_AR4 CORR_AR5 CORR_BANK CORR_FLOW CORR_HI
CORR_LO CORR_SETUP DEMODIO DEMODIO_AFTER DEMODIO_PCHI DEMODIO_PCLO DEMODRD DETTRACE
FBCALL FBDET_API FBDET_SENTINEL FBENTRY FBROUTE FBWATCH FB_CORR_ENTRY FB_ENERGY
FB_IQ_BASE FB_IQ_DARAM FB_IQ_FCCH_ONLY FB_IQ_MARKER FB_STREAM FB_STREAM_CELL
FB_STREAM_CELLQ FB_STREAM_DECIM FORCE_3FAD_KERNEL FORCE_3FAE FORCE_AGCH FORCE_FBSB
FORCE_NB FORCE_TOA PROBE_3FAD_GATE SCANFB SCANREF SHUNT_DSP_FB SHUNT_DSP_FB_BUDGET
SHUNT_DSP_FB_ENTRY SHUNT_DSP_FB_MAX SHUNT_DSP_FB_SP SHUNT_REAL_FB WATCH_2A00
WATCH_9200 WATCH_9F00_RD WATCH_RESULT
```

### LOT 6 — Canaux shunt DL/UL (SDCCH/SACCH/AGCH/BCCH/burst/SI), req-ref, RF/AFC/de-can, sondes registres (59)
```
AR0_DEBUG AR2_FLOOR_DROP AR6_AT_LOG_CAP AR6_AT_PC AR6_WIN_HI AR6_WIN_LO ARWATCH
AR_TRACE A_TRACE_PC DA_HI DA_INSN DA_LO DECAN DECAN_ANGLE DECAN_PM DECAN_PM_MAV_REF
DECAN_PM_RF_REF DECAN_SNR DECAN_TOA REQREF_ADJ REQREF_LAST_RACH REQREF_PERRA
REQREF_REWRITE RMAP RMAP_PCHI RMAP_PCLO SHUNT_AGCH SHUNT_AGCH_EXPIRE SHUNT_AGCH_OFS
SHUNT_AGCH_TTL SHUNT_BCCH_OFS SHUNT_BCCH_SCHED SHUNT_BURST_FN SHUNT_BURST_M1
SHUNT_BURST_OFS SHUNT_BURST_PERCMD SHUNT_GSMTAP_PORT SHUNT_IQ_CFILE SHUNT_IQ_CFILE2
SHUNT_IQ_RECORD SHUNT_PM SHUNT_SACCH SHUNT_SACCH_OFS SHUNT_SACCH_PAR SHUNT_SCH_PORT
SHUNT_SDCCH SHUNT_SDCCH_MAXPRES SHUNT_SDCCH_OFS SHUNT_SDCCH_RING SHUNT_SDCCH_TTL
SHUNT_SI_ROT_MASK TRF_RXLEV TRF_TARGET_RF TRF_TSP_DEV TWL3025_AFC TWL3025_AFC_HZ
UL_ACU_OFS UL_PUB_IDLE UL_RACH_FROM_DRACH
```

---

## (d) VARIABLES QUI EN REPOSENT D'AUTRES — **la section la plus opérationnelle**

### d.1 Dépendances par PROFIL (fichier .env sourcé en cascade)

`start-clean.sh` → `set -a` → `. calypso.env` → sourcing conditionnel. **La CLI gagne partout (`:=`), sauf 1 verrou.**

| Maître | Effet |
|---|---|
| `CALYPSO_SHUNT_NO_LEGIT=1` | source `calypso_shunt_no_legit.env` → impose 15 vars : `SHUNT_REAL_FB=1, INJECT_SB/ACD/AGCH/SDCCH/SACCH=1, SHUNT_DL_INJECT=1, UL_RACH_FROM_DRACH=1, DECAN=1, SHUNT_FEED_SI=1, TRF_RXLEV=1, SHUNT_BURST_FN=1, SHUNT_BURST_OFS=-1, DSP_SHUNT=1, DSP_RUN_C54X=0, SHUNT_AGCH_EXPIRE=1` |
| `CALYPSO_SHUNT_LEGIT=1` | source `calypso_shunt_legit.env` → `DECAN=1, DSP_SHUNT=1, DSP_RUN_C54X=0` |
| `CALYPSO_NATIVE=1` | source `calypso_native.env` → 13 vars dont `FB_ENERGY=1, FB_CORR_ENTRY=0x9500, FB_STREAM=1, BSP_DIRECT_FEED=1, ARM2DSP_BGEN=1, ARM2DSP_CTRLSYS=0, KEEP_IMR=1, FRAME_IT_NATIVE=1, DECAN=1` |
| **`CALYPSO_NATIVE_HELPED=1`** | source `calypso_native_helped.env` → **`FB_ENERGY=1, FB_CORR_ENTRY=0x9500, FB_IQ_DARAM=1, FB_IQ_BASE=0x9210, WATCH_2A00=1, DECAN=1, DSP_RUN_C54X=1, DSP_SHUNT=0, ARM2DSP_BGEN=1, ARM2DSP_CTRLSYS=0, KEEP_IMR=1, INIT_435B_OFF=0, FRAME_IT_NATIVE=1`** — c'est le cas mesuré le 2026-07-28 |
| `CALYPSO_HACK=1` (déf 0) | source `calypso_hack.env` → béquilles go-live + `export` DUR de `TWL3025_AFC_HZ`, `SHUNT_NO_FAKE_FB`, `SHUNT_NO_FAKE_PM` |
| `CALYPSO_WIRE=1` (déf 0) | pose `TINT0_MASTER=1, TINT0_PERIOD=1500, BSP_DIRECT_BRINT0=1` puis source `calypso_wire.env` → 14 vars + **4 `unset`** (`ISR_TO_8341, CORR_SETUP, FORCE_3F92, FORCE_0810`) + `TPU_RX_WIRE=1` |
| `CALYPSO_MODE` (run.sh:1108, déf `full-grgsm`) | **impose par mode** `DSP_SHUNT, SKIP_IPC_DEVICE, SKIP_TRX_IPC, SKIP_BTS, SKIP_L2, SKIP_GSMTAP, SKIP_BRIDGE_PY, IPC_RELAY, BSP_IQ_PASSTHROUGH, RELAY_ALSO_BSP, SHUNT_NO_CANNED, CANNED, RELAY_FIFOS` — c'est le second `NATIVE_HELPED`, systématiquement oublié |

**Corollaire opératoire** : retirer `FB_CORR_ENTRY` de la ligne de commande ne le supprime pas si `NATIVE_HELPED=1`. Idem retirer `DSP_SHUNT` ne sert à rien si `MODE` le repose. **La vérité = les lignes `[calypso-manifest] ...`** (dumpées par le constructeur `shunt_env_value_list()` @ `calypso_dsp_shunt.c:86-115`, **avant `main()`**), jamais la CLI.

### d.2 Dépendances par CODE (variable A change le sens de la variable B)

| Site | Règle |
|---|---|
| `calypso_dsp_shunt.c:86-100` (**constructeur, avant main**) | `SHUNT_LEGIT` / `SHUNT_NO_LEGIT` acceptent une **value-list** : contient `DSP` → `setenv(DSP_RUN_C54X=1)` ; contient `NO_CANNED` → `setenv(SHUNT_NO_CANNED=1)` ; puis la base est **canonicalisée à "1"**. Des variables sont donc créées de toutes pièces avant tout `getenv()`. |
| `calypso_dsp_shunt.c:1594` | **`DECAN=1` IMPLIQUE `SHUNT_REAL_FB=1`** (commentaire in-code : « master DECAN implique REAL_FB »). Mettre `SHUNT_REAL_FB=0` ne suffit pas ; il faut aussi `DECAN≠1`. |
| `calypso_dsp_shunt.c:564-568, 652, 774-784` | `DECAN=1` implique `DECAN_PM`, `DECAN_SNR`, `DECAN_TOA`, `DECAN_ANGLE`. |
| `c54x.c:2698`, `dsp_shunt.c:625/858`, `trx.c:309/324` (5 sites) | `SHUNT_LEGIT` **OU** `SHUNT_NO_LEGIT` → gate unique. Les deux sont interchangeables sur ces sites. |
| `dsp_helper.c:282/345`, `dsp_shunt.c:942/1070/1114/1966` | `INJECT_SB/ACD/AGCH/SDCCH/SACCH` et `SHUNT_FEED_SI` : si la var propre ≠ "1", **fallback sur `SHUNT_LEGIT=1`**. `INJECT_FB` (dsp_helper.c:233) est le SEUL sans ce fallback. |
| `c54x.c:2710` | `TRF_RXLEV` **OU** `SHUNT_LEGIT=1` → forçage a_pm/rxlev. |
| `c54x.c:4933` | `C54X_IRQ_LEVEL` **OU** `DSP=="c54x"`. |
| `c54x.c:5004` | `DSP_FRAME_VEC28` **OU** `DSP=="c54x"` → revectorisation vec19→vec28. **Donc `DSP=c54x` (défaut calypso.env) active silencieusement deux comportements non demandés.** |
| `bsp.c:1069/1418`, `trx.c:1444` | `FRAME_IT_NATIVE` **OU** `DSP_FRAME_VEC28` → `_fb = 12` au lieu de 3. |
| `bsp.c:472` | `BSP_DARAM_FORCE` n'a d'effet **que si** `DSP_RUN_C54X=="1"`. |
| `bsp.c:1353` | `TPU_RX_WIRE` **OU** (`DSP_RUN_C54X=1` **ET** `BSP_DARAM_FORCE`) → même gate levé par deux chemins. |
| `bsp.c:1260` | `IQDUMP` **OU** `BSP_DUMP_RX_FILE` (variable **hors namespace CALYPSO**, posée en dur `calypso.env:30`). |
| `c54x.c:422/258/349/5195/4616` | `AR6_AT_PC/WIN_LO/WIN_HI/AT_LOG_CAP`, `AR_TRACE`, `A_TRACE_PC`, `CORR_LO/HI`, `AR2_FLOOR_DROP` : **inertes sans le jeton `CALYPSO_DEBUG` correspondant**. Ce sont des paramètres de sondes, pas des sondes. |

### d.3 Pièges d'idiome à annoter en priorité (mesurés, pas déduits)

1. **`: "${VAR:=}"` sous `set -a` EXPORTE UNE CHAÎNE VIDE ⇒ non-NULL ⇒ tout gate `EXISTS` est ON.** Documenté in-code (`calypso_wire.env:4-5`) et corrigé là par 4 `unset`. **Vérifier les 11 `:=(vide)` de `calypso.env`/`calypso_hack.env`** (`CANNED, START_FN, UL_DEBUG, UL_SLOT_OFFSET, UL_RA, UL_FN_GATE, UL_FN_OFFSET, UL_RACH_STICKY, REQREF_ADJ, FORCE_3F92, FORCE_0810`) contre les idiomes de leurs consommateurs — côté modèle QEMU aucun n'est `EXISTS`, mais **`qemu_wrap.c` n'a pas été audité sous cet angle**.
2. **`=0` N'ÉTEINT PAS** : `INTM_TRANS`, `IT_PUSH_XPC_ALWAYS`, `WATCH_VEC` (idiome `NON-VIDE`) ; `SHUNT_BURST_PERCMD` (idiome `ON-sauf-VIDE`) ; et les **139 gates `EXISTS`** du tableau.
3. **Défaut ON invisible** : `POKE_TASK_MD` (`bsp.c:1116`) est à **1 quand la variable est absente** — béquille active par défaut, jamais listée dans aucun .env.
4. `INIT_435B_OFF` : nom en `_OFF` mais idiome `INV-VAL` ; `=0` **active** l'init du shadow. Les 4 .env le posent à `0` — c'est-à-dire **ON**.

### d.4 Variables MORTES confirmées (posées dans un .env, lues NULLE PART dans le source)

Vérifié par `grep -rl '"VAR"' --include=*.c --include=*.h` sur tout `${QEMU_TREE}` :

| Variable | Posée | Verdict |
|---|---|---|
| `CALYPSO_FIX_MVDM` | `calypso.env:183` (`:=1`, 6 lignes de commentaire) | **MORT** — le code ne lit que `CALYPSO_FIX_MVDM_OFF` |
| `CALYPSO_CORRELATOR_TRACE` | `calypso.env:188`, `wire.env:34` | **MORT** — le code utilise les jetons `CALYPSO_DEBUG=CORRELATOR`/`CORR-TRACE` |
| `CALYPSO_FORCE_3F92` | `calypso.env:199` + `wire.env:42 unset` | **MORT** — plus aucun lecteur |
| `CALYPSO_FORCE_0810` | `calypso.env:202` + `wire.env:43 unset` | **MORT** — plus aucun lecteur |
| `CALYPSO_C54X_CRASHPC` | `calypso.env:104` (`:=1`) | **MORT en tant que gate** — les 2 seules occurrences sont dans un `fprintf` de log (`calypso_dsp_shunt.c:840`). Le commentaire de `calypso.env:99-100` (« arme le catcher SIGSEGV/ABRT/FPE/BUS → imprime `[c54x] *** CRASH … last_pc=` ») est **périmé** : aucun handler de signal n'existe. |
| `CALYPSO_HACK`, `CALYPSO_WIRE`, `CALYPSO_NATIVE`, `CALYPSO_NATIVE_HELPED`, `CALYPSO_MODE`, `CALYPSO_FORCE_DEMOD_BRIDGE`, `CALYPSO_IRDA_CAPTURE` | calypso.env / run.sh | **Vivantes mais SHELL-ONLY** — ne jamais chercher à les annoter dans un .c |
| `CALYPSO_START_FN`, `CALYPSO_NB_MAXDLY` | `calypso.env:58-59` marquées `[REQUIS]` | **Aucun lecteur dans osmo-qemu-calypso** — soit osmo-trx hors arbre, soit MORTES. À trancher par l'agent du lot 1. |

Ligne parasite `calypso.env:195` : `: "0xC000"` — instruction shell sans effet (no-op), reliquat du test `FORCE_3F92`.

---

## LOT 1 — BSP / DMA / DARAM / feed I-Q / horloge TDMA

**LOT 1 — BSP / DMA / DARAM / feed I-Q / horloge TDMA (52 variables).** Snapshot vérifié : `md5(calypso_bsp.c)=4760924b…`, `calypso_trx.c=8fbe30d8…`, `calypso_c54x.c=c36466ab…`, `calypso_dsp_shunt.c=a0f17a26…` — identiques au conteneur. Les numéros de ligne ci-dessous valent pour ce snapshot.

**MESURE de référence** — run vivant démarré à 15:55:58 (le seul `BUILD-STAMP` de `/root/qemu.log`, binaire compilé `Jul 28 2026 15:43:00`). Manifeste effectif pour ce lot : `BSP_DARAM_ADDR=0x4c00`, `BSP_DARAM_LEN=296`, `BSP_DARAM_FORCE=1`, `BSP_DIRECT_FEED=1`, `BSP_FN_PROBE=1`, `BSP_IQ_DECIM=4`, `BSP_IQ_PASSTHROUGH=1`, `FB_IQ_OWNS=0`, `IQ_TEE_PORT=6703`, `TDMA_REALTIME=1`, `DARAM_DUMP=1`, `WMAP=1/WMAP_LO=0x2a00/WMAP_HI=0x2a1f`, `NDB_D_RACH_OFFSET=` (vide), `RACH_FORCE_BSIC=` (vide), `START_FN=` (vide), `NB_MAXDLY=40`. Mode `full-grgsm`, `DSP_SHUNT=1` **ET** `DSP_RUN_C54X=1`. Toutes les autres variables du lot sont **absentes** du manifeste.

---

## Tableau

| VARIABLE | DEFAUT | EFFET (code exécuté) | MODE | IDIOME (comment on la coupe) | CATEG. | REPOSE / REPOSÉE PAR |
|---|---|---|---|---|---|---|
| `BSP_BIND_ADDR` | code `0.0.0.0` (bsp.c:907) | IP de bind du listener TRXDv0 (bsp.c:899, 902, 922-927) ; invalide → repli `0.0.0.0` + log | tous | CHAINE non-vide ; vide = ignorée | CONFIG | prioritaire sur `BSP_BIND_LOOPBACK` |
| `BSP_BIND_LOOPBACK` | unset (bsp.c:900) | alias legacy → bind `127.0.0.1` (bsp.c:904) | tous | `EQ1` (`*e=='1'`) ; masquée si `BSP_BIND_ADDR` non-vide | CONFIG | reposée par `BSP_BIND_ADDR` |
| `BSP_BYPASS_BDLENA` | code `0` (bsp.c:837) | **AUCUN.** `bsp.bypass_bdlena` n'est lu par personne : 3 occurrences seulement (déclaration :127, affectation :837, log :838). Le gate BDLENA qu'elle prétend couper a été supprimé le 2026-05-29 (bsp.c:1019-1024) | — | `parse_uint_env` (>0) — sans effet | **MORT** | — |
| `BSP_DARAM_ADDR` | code `0x2a00` (bsp.c:823) ; run.sh:1303 `:-0x2a00` ; **live `0x4c00`** | adresse d'écriture DMA du burst (bsp.c:1240, 1504) ; `0` = mode DISCOVERY, aucune DMA (bsp.c:1011, 1362) | tous | `parse_uint_env` (auto-hex, bsp.c:357-372) ; vide = défaut | CONFIG | — |
| `BSP_DARAM_FORCE` | `calypso_wire.env:46 :=1` ; **live `=1`** | lève 3 gates shunt indépendamment de `route_c54x_active()` : bsp.c:472 (enqueue), :995 (rx_burst), :1354 (deliver). **N'a d'effet que si `DSP_RUN_C54X=="1"`** | shunt (`DSP_SHUNT=1` + `DSP_RUN_C54X=1`) | `EXISTS` (`=0` NE COUPE PAS) → `unset` obligatoire | **BEQUILLE** | dépend de `DSP_RUN_C54X` ; co-lève le gate `TPU_RX_WIRE` @1354 |
| `BSP_DARAM_LEN` | code `296` (bsp.c:824) ; live `296` | borne de la fenêtre d'écriture + wrap `woff` (bsp.c:1175, 1244, 1512) | tous | `parse_uint_env` | CONFIG | — |
| `BSP_DIRECT_BRINT0` | `calypso.env:225 :=1` (bloc `WIRE`) et `wire.env:27 :=1` — **inactive hors `CALYPSO_WIRE=1`** ; absente du run vivant | lève `c54x_interrupt_ex(dsp,21,5)` (BRINT0/vec21) depuis rx_burst, gaté mission `task_md∈{5,6,8,9}` + anti-stack IFR bit5 (bsp.c:1083-1093) | direct-feed sous WIRE | `EXISTS` → `unset` | **BEQUILLE** | posée par `CALYPSO_WIRE=1` |
| `BSP_DIRECT_FEED` | `calypso.env:141 :=1`, `native.env:18 :=1` ; **live `=1`** | bsp.c:653 : appelle `calypso_bsp_rx_burst()` **au lieu de** `bsp_enqueue()` → court-circuite tout le match FN. **Conséquence mesurée : la file reste vide, donc tout `deliver_buffered` (y compris `IQDUMP` :1520 et `RX_FBFLAGS` :1570) est code mort dans le run vivant** | tous | `EQ1` (`*e=='1'`) → `=0` coupe | **BEQUILLE** | rend inertes `TPU_RX_WIRE`(:1387), `INJECT_CANARY`(:1508), `RX_FBFLAGS`(:1570) |
| `BSP_DISPATCH_FB` | `wire.env:28 :=1` ; absente du run vivant | bsp.c:1145-1173 : écrit la cible dans `data[0x43c0]/[0x4387]/[0x43d8]` (slots BACC/CALA/reseed) + `imr |= 0x0200`, à chaque burst de mission FB/SB | mission FB/SB, DSP réel | `EXISTS` → `unset` | **BEQUILLE** | pose IMR bit9 ; modulée par `_TGT`/`_NOIMR`/`_ONESHOT` |
| `BSP_DISPATCH_FB_TGT` | code `0x8d00` (bsp.c:1148) | adresse installée dans les 3 slots | idem | VALEUR (`strtoul` base 0) | BEQUILLE (paramètre du bloc :1145) | inerte sans `BSP_DISPATCH_FB` |
| `BSP_DISPATCH_NOIMR` | unset (bsp.c:1165) | installe le handler **sans** toucher l'IMR (évite la préemption vec21 6 instructions après l'entrée) | idem | `EXISTS` → `unset` | BEQUILLE (même bloc) | inerte sans `BSP_DISPATCH_FB` |
| `BSP_DISPATCH_ONESHOT` | `wire.env:54` **`unset` explicite** | n'installe qu'une fois (`_done`) au lieu de re-dispatcher chaque trame | idem | `EXISTS` → `unset` | BEQUILLE (même bloc) | inerte sans `BSP_DISPATCH_FB` |
| `BSP_FN_PROBE` | `calypso.env:137 :=1`, `wire.env:35 :=1` ; **live `=1`** | log `FN-PROBE tn/dispatcher_fn/burst_fn/delta/verdict`, cap 300 puis 1/500 (bsp.c:302-317). Zéro écriture | tous | `EXISTS` → `unset` | MESURE | lit la FN issue de `DL_FN_OFFSET` |
| `BSP_INJECT_CANARY` | code `0` (bsp.c:847) | remplace **tous** les samples par `0xCAFE` avant écriture DARAM (bsp.c:1508) — uniquement sur le chemin `deliver_buffered`, donc **inerte tant que `BSP_DIRECT_FEED=1`** | buffered only | `parse_uint_env` (>0) | MESURE (sonde destructive, défaut inerte) | masquée par `BSP_DIRECT_FEED` |
| `BSP_IQ_DECIM` | code `4` (bsp.c:591) ; **live `=4`** | décime le flux 4 SPS du device vers 1 SPS (bsp.c:591-601) ; **2e lecteur** `dsp_shunt.c:1525` pour le feed `FB_IQ_DARAM` | tous | VALEUR ; `<1` clampé à 1 ; vide = 4 | CONFIG | lue aussi par le bloc `FB_IQ_DARAM` (lot 5) |
| `BSP_IQ_PASSTHROUGH` | `ON-sauf-0` (bsp.c:570-577) ; run.sh:1342 `:-1` ; **live `=1`** | `1` = interprète le payload UDP comme I/Q cs16 réel ; `0` = **synthèse interne cos/sin ±π/2** (bsp.c:1218-1233 branche `else`) = signal fabriqué. **run.sh:1411-1415 le force à `0` si `DSP_SHUNT=1` ET `MODE != full-grgsm`** — exemption `full-grgsm`, d'où `=1` vivant | tous | `(e && *e=='0') ? 0 : 1` → seul `=0` coupe | CONFIG (la branche OFF, elle, est une béquille) | reposée par `CALYPSO_MODE` + `DSP_SHUNT` (run.sh) |
| `BSP_IQ_SHIFT` | code `0` (bsp.c:1234) | décale les samples `>>n` avant DARAM ; clampé `[0,12]` (bsp.c:1233-1241) | rx_burst | VALEUR ; `0`/vide = inerte | MESURE (instrument saturation) | inerte si `FB_IQ_OWNS=1` |
| `BSP_PORT` | code `BSP_TRXD_PORT=6702` (bsp.c:58, 913) | port UDP d'écoute ; accepté si `0<p<65536` (bsp.c:914-917) | tous | VALEUR ; vide = 6702 | CONFIG | — |
| `BSP_REPLAY_FILE` | unset (bsp.c:868) | charge un fichier de bursts et **saute totalement le listener UDP** (`goto skip_udp_listener`, bsp.c:880), timer de rejeu à la place | tous | CHAINE non-vide | CONFIG (banc de rejeu déterministe) | — |
| `CPU_IDLE` | `ON-sauf-0` (trx.c:1239) ; **live ON** (log `[cpu-idle] governor ON … window=[0x823000,0x826000]`) | `cs->halted=1; cpu_exit()` quand le PC ARM est dans la fenêtre L1 idle (trx.c:1230-1262), appelé depuis `calypso_tdma_tick` (trx.c:1281) | tous | `(e && *e=='0') ? 0 : 1` → seul `=0` coupe | CONFIG | consomme `IDLE_PC_LO/HI` |
| `DARAM_DUMP` | unset (c54x.c:15666) ; **live `=1`** | ouvre un `.cfile` IQ16 et dumpe **`data[0x2a00..0x2b27]` en dur** (c54x.c:15700, 15711-15716) + verdict `DARAM-SANITY` (coh/dphi/rms). `=1` → chemin `/dev/shm/daram_2a00.cfile`, sinon la valeur EST le chemin | tous | `(e && *e && strcmp(e,"0"))` → `0` ou vide coupent | MESURE | **⚠ l'adresse dumpée est figée à `0x2a00` et ne suit PAS `BSP_DARAM_ADDR` (= `0x4c00` en live) : la sonde ne regarde pas le buffer que le BSP écrit** |
| `DARAM_DUMP_PC` | code `0x9ac0` (c54x.c:15665, 15671) | PC déclencheur du dump | avec `DARAM_DUMP` | VALEUR | MESURE | inerte sans `DARAM_DUMP` |
| `DARAM_DUMP_MAX` | code `200` (c54x.c:15665, 15673) | nombre max d'enregistrements | avec `DARAM_DUMP` | VALEUR | MESURE | idem |
| `DARAM_DUMP_ANYMODE` | code `0` (c54x.c:15685) | supprime le garde `data[0x08f9]!=0` (d_fb_mode) → filme aussi hors phase FB | avec `DARAM_DUMP` | `atoi>0` → `0` coupe | MESURE | idem |
| `DL_FN_OFFSET` | code `0` (trx.c:157-167) ; **aucun .env, absente du run** | offset signé ajouté dans `calypso_trx_get_fn()` → propagé à **tous** les consommateurs (bsp ×4, dsp_shunt ×6, dsp_helper ×3) | tous | VALEUR ; `0`/vide = strictement inerte | **BEQUILLE** (défaut inerte) | recale toute la timeline FN, y compris UL/DATA_IND |
| `DMAWATCH` | unset (c54x.c:1824, 3002) | log lectures/écritures `data[0x0054..0x0057]` (registres DMA), cap 40 chacun | tous | `EXISTS` → `unset` | MESURE | — |
| `FB_IQ_OWNS` | `calypso.env:145 :=0` ; **live `=0`** | `1` → `rx_burst` **saute** sa boucle d'écriture DARAM (bsp.c:1224-1228) et cède `0x2a00` à `feed_iq` ; `0` → rx_burst écrit toujours | shunt + `FB_IQ_DARAM` | `atoi>0` → `0` coupe | **BEQUILLE** | arbitre deux writers concurrents ; découplé de `FB_IQ_DARAM` depuis 2026-07-27 |
| `IDLE_PC_LO` | code `0x00823000` (trx.c:1240) | borne basse de la fenêtre de parking | avec `CPU_IDLE` | VALEUR (`strtoull` base 0) | CONFIG | inerte si `CPU_IDLE=0` |
| `IDLE_PC_HI` | code `0x00826000` (trx.c:1241) | borne haute ; `lo=hi=0` → halt dès qu'aucune IRQ n'est pendante | avec `CPU_IDLE` | VALEUR | CONFIG | idem |
| `IQDUMP` | unset ; **absente du run** | bsp.c:1260 (calcul de cohérence par burst), :1271 (24 fichiers `/tmp/iq_rx_NNN.bin`), :1520 (`/tmp/iq_dlv_NNN.bin`, chemin buffered = mort en live) | tous | `EXISTS` → `unset` | MESURE | **⚠ PIÈGE : le gate :1260 est un `OR` avec `BSP_DUMP_RX_FILE`, posé EN DUR (`=`, pas `:=`) à `calypso.env:30` → présent dans l'environ du run ⇒ la boucle de cohérence O(n) tourne à CHAQUE burst et le writer `.cfile` :1281-1293 est actif, sans que `CALYPSO_IQDUMP` soit défini** |
| `IQDUMP_FCCH` | unset (bsp.c:1031) | sonde cohérence/dphi du burst décimé écrit en DARAM, `fprintf` inconditionnel, cap 30, seuil `coh>0.85` | rx_burst | `EXISTS` → `unset` | MESURE | — |
| `IQ_CFILE_SPF` | code `2500` (dsp_shunt.c:1794, 1814) | int16 par trame TDMA du cfile FN-espacé (zero-fill des trames manquantes) | shunt + `SHUNT_IQ_CFILE2` | VALEUR ; vide = 2500 | CONFIG | inerte sans `CALYPSO_SHUNT_IQ_CFILE2` (lot 6, absente du run). **⚠ le bloc entier `if (g_iq_cfile2){…}` est DUPLIQUÉ verbatim dans `calypso_dsp_shunt_feed_iq()` (dsp_shunt.c:1791-1807 puis 1808-1826, même fonction, fin à :1828) : chaque burst est écrit DEUX FOIS avec deux `static pos` indépendants → cfile FN-espacé corrompu** |
| `IQ_TEE_HOST` | code `127.0.0.1` (bsp.c:410-419) | destination du tee UDP I/Q brut | **shunt uniquement** (`calypso_dsp_shunt_active()`, bsp.c:401) | CHAINE non-vide | CONFIG | — |
| `IQ_TEE_PORT` | code `6703` (bsp.c:405-406) ; run.sh:1224 `:=6703` ; **live `=6703`** | port du tee ; le bridge gr-gsm lit ce port (run.sh:2031) | shunt uniquement | VALEUR ; vide = 6703 | CONFIG | — |
| `MEM_WATCH_2B80` | unset (c54x.c:1868, 3075) | log toute lecture (cap 200) et écriture (cap 200) dans `[0x2b80,0x2c00)` | tous | `EXISTS` → `unset` | MESURE | — |
| `NDB_D_RACH_OFFSET` | code `D_RACH_DEFAULT_OFFSET=0x023A` (bsp.c:1741) ; run.sh:1334 `:-` ; **live vide → défaut** | offset mot de `d_rach` dans le NDB : bsp.c:1747 (encodeur RACH) **et** trx.c:769 où le défaut `0x023A` est **ré-écrit en dur** (`dr_byte = w*2`) — deux sources de vérité à re-synchroniser à la main | tous | VALEUR ; **vide traitée comme unset** (`e && *e`) → sûr sous `set -a` | CONFIG | — |
| `POKE_DISPATCH` | code `0` (bsp.c:1123) | réplique `dsp_end_scenario` : écrit `d_task_md` sur la write-page + `d_dsp_page = 0x0002\|w_page` en alternant `w_page` (bsp.c:1122-1128) | mission FB/SB | `atoi>0` ; défaut 0 | **BEQUILLE** | **imbriquée dans le bloc `RX_FBFLAGS`** → inerte tant que `RX_FBFLAGS` est unset |
| `POKE_TASK_MD` | **code `1` si la variable est ABSENTE** (bsp.c:1116) | écrit `data[0x0804]` et `data[0x0818]` = `task_md` courant (les 2 pages API-RAM) | mission FB/SB | `_pe ? (atoi>0) : 1` → seul `=0` coupe ; l'absence l'ACTIVE | **BEQUILLE** | **correction au recensement §d.3-3** : « béquille active par défaut » n'est vrai que *dans* le bloc `if (_fbf && _fbsbf && …)` de `RX_FBFLAGS` (bsp.c:1105) → globalement inerte dans le run vivant |
| `RACH_FORCE_BSIC` | unset → `-1` (bsp.c:1766-1786) ; run.sh:1335 `:-` ; **live vide** | force le BSIC de l'encodeur RACH à `0..63`, court-circuitant la lecture de `d_rach` ; hors plage → ignoré + log | tous | VALEUR ; vide = unset ; pré-chauffée à l'init (bsp.c:945) | **BEQUILLE** (défaut inerte) | contourne l'incertitude sur `NDB_D_RACH_OFFSET` |
| `RX_FBFLAGS` | unset ; **absente du run** | bsp.c:1102-1134 (chemin vivant rx_burst) : `data[0x3fad]\|=0x8000` (verrou-maître du noyau @0x8754), `0x3faa\|=0x0104`, `0x3fab\|=0x0100`, `0x3fae\|=0x0100`, + `calypso_rxfb_fired=1` ; bsp.c:1570-1579 (chemin buffered, mort sous `DIRECT_FEED=1`) pose les 3 derniers **sans** `0x3fad` | mission FB/SB | `EXISTS` → `unset` | **BEQUILLE** | conteneur de `POKE_TASK_MD` et `POKE_DISPATCH` |
| `SLOTSRC` | unset (c54x.c:1852, 15492) | (a) log toute lecture data dont la valeur `==0xab38` (stub), cap 40 ; (b) trace d'exécution `PC∈[0xaff0,0xb01d]`, cap 120 | tous | `EXISTS` → `unset` | MESURE | — |
| `TDMA_NS` | code `WALL_TDMA_NS=4615384` (trx.c:1036, 1050-1051) ; **absente du run** (log `4615384 ns/frame`) | période du pthread `clk_master` ; **les valeurs `< 4615384` sont silencieusement IGNORÉES** (`if (v >= WALL_TDMA_NS)`) : on ne peut qu'ADOUCIR | tous | VALEUR ; vide = défaut | CONFIG | **lue aussi par `tools/calypso-ipc-device/qemu_wrap.c:1453`** (le device doit suivre) |
| `TDMA_REALTIME` | `calypso.env:23 :=1`, run.sh:1312 `:-1` ; **live `=1`** (log `REALTIME (wall-clock 217 Hz, opt-in)`) | choisit `QEMU_CLOCK_REALTIME` vs `QEMU_CLOCK_VIRTUAL` pour le `tdma_timer` (trx.c:1290-1306) | tous | `EQ1` → seul `1` active | CONFIG | **lue aussi par `hw/timer/calypso_timer.c:56`** (hors lot, même valeur) |
| `TPU_RX_WIRE` | `wire.env:61 :=1` ; **absente du run vivant** | 3 sites : bsp.c:1353 (lève le gate shunt de `deliver_buffered`), bsp.c:1387 (consomme le pulse BDLENA IOTA → `data[0x3f92] \|= 0x0800` + `bsp_take_nearest` au lieu du match FN), trx.c:956 (laisse passer la DMA page-écriture ARM→DARAM 0x0586 sous shunt) | shunt | `EXISTS` → `unset` | **BEQUILLE** | **effet de bord mesuré : le gate :1353 est ÉGALEMENT levé par `DSP_RUN_C54X=1 && BSP_DARAM_FORCE` — log vivant `[bsp] deliver: gate shunt LEVE (rxw=1)` alors que `TPU_RX_WIRE` est unset. Les sites :1387 et trx.c:956 restent, eux, fermés → asymétrie : la livraison est ouverte mais `d[0x3f92]` n'est pas posé et la DMA de tâche reste bloquée.** `calypso_iota_take_bdl_pulse()` n'a qu'un seul appelant (bsp.c:1388), donc unset ⇒ le pulse IOTA n'est jamais consommé |
| `WATCH_RD_ADDR` | code `0` (c54x.c:1931-1933) | log toute lecture de l'adresse data indiquée (PC/valeur/DP/insn) | tous | VALEUR **+ jeton `CALYPSO_DEBUG=WATCH-RD`** (macro `C54_DBG`) — inerte sans les deux | MESURE | dépend du namespace `CALYPSO_DEBUG` |
| `WATCH_WR_ADDR` | code `0` (c54x.c:3701-3703) | symétrique en écriture (val, ancienne val, PC, DP) | tous | VALEUR **+ jeton `CALYPSO_DEBUG=WATCH-WR`** | MESURE | idem |
| `WMAP` | unset (c54x.c:2885) ; **live `=1`** | cartographie les PC qui écrivent dans la/les plage(s) ; heartbeat 1/5M écritures si aucune (c54x.c:2874-2905) | tous | `atoi>0` → `0` coupe | MESURE | pilote `WMAP_LO/HI/LO2/HI2` |
| `WMAP_LO` | code `0x2c00` (c54x.c:2888) ; **live `0x2a00`** | borne basse plage 1 | avec `WMAP` | VALEUR | MESURE | inerte sans `WMAP` |
| `WMAP_HI` | code `0x2c1f` (c54x.c:2888) ; **live `0x2a1f`** | borne haute plage 1 | avec `WMAP` | VALEUR | MESURE | idem |
| `WMAP_LO2` | code `0xffff` (c54x.c:2891) | borne basse plage 2 — défauts `lo2=0xffff > hi2=0x0000` = plage vide, donc **désactivée par construction** | avec `WMAP` | VALEUR | MESURE | idem |
| `WMAP_HI2` | code `0x0000` (c54x.c:2891) | borne haute plage 2 | avec `WMAP` | VALEUR | MESURE | idem |
| `WZWRITE` | unset ; **absente du run** | c54x.c:1811 « WZREAD » : lecture `data[0x2c00]` filtrée `PC==0xa07c`, cap 40 ; c54x.c:2969 « WZWRITE » : écriture `data[0x2c00]` filtrée `PC∈{0x9fd5,0x9ab1}`, cap 40 | tous | `EXISTS` → `unset` | MESURE | — |

---

## Verdicts demandés au lot 1

- **`CALYPSO_START_FN` : MORT.** `grep -rn` sur tout `${QEMU_TREE}` (`*.c`, `*.h`, `*.py`, `*.sh`) → **zéro lecteur**. Seules occurrences dans l'arborescence : les `calypso.env` de `${GSM_ROOT}/qemu-calypso` (overlay mort) et `${QEMU_TREE}.bak`. Exportée vide dans le run vivant (`CALYPSO_START_FN=`). Le commentaire `calypso.env:58` la marque `[REQUIS]` — commentaire périmé.
- **`CALYPSO_NB_MAXDLY` : MORT** dans le même sens (aucun lecteur dans `osmo-qemu-calypso`), exportée `=40` en live. Si un consommateur existe, il est dans un binaire osmo-trx hors arbre ; côté modèle QEMU elle est sans effet.
- **`CALYPSO_BSP_BYPASS_BDLENA` : MORT** (voir tableau) — candidat au retrait avec le champ `bsp.bypass_bdlena` (bsp.c:127).

---

## Blocs @BEQUILLE prêts à coller

```
BSP_DARAM_FORCE / calypso_bsp.c:466 (bloc 465-478) — répliquer à :988 (bloc 987-1002) et :1345 (bloc 1344-1360)
/* @BEQUILLE — BSP_DARAM_FORCE  (CALYPSO_BSP_DARAM_FORCE, EXISTS, defaut OFF ; wire.env:=1)
 *   masque  : calypso_dsp_shunt_route_c54x_active() ne devient jamais vrai sous
 *             DSP_SHUNT=1, donc les 3 gates shunt (enqueue :472, rx_burst :995,
 *             deliver :1354) ferment la DARAM au correlateur natif. Ce forcage
 *             remplace la route DSP reelle qui n'est pas modelisee.
 *   retirer : quand route_c54x_active() reflete l'etat reel du c54x, ou quand
 *             DSP_SHUNT et DSP_RUN_C54X cessent d'etre simultanement a 1.
 */
```

```
BSP_DIRECT_BRINT0 / calypso_bsp.c:1083 (bloc 1075-1093)
/* @BEQUILLE — BSP_DIRECT_BRINT0  (CALYPSO_BSP_DIRECT_BRINT0, EXISTS, defaut OFF ; wire.env:=1)
 *   masque  : sur silicium, la fin de DMA BSP (fenetre BDLENA) leve BRINT0
 *             vec21/bit5. Le chemin direct-feed ne leve qu'INT3 ; la chaine
 *             TPU->TSP->IOTA->BSP qui produirait le pulse n'est pas cablee.
 *   retirer : des que calypso_iota_take_bdl_pulse() est alimente par la fenetre
 *             RX du TPU et consomme sur le chemin vivant (cf TPU_RX_WIRE).
 */
```

```
BSP_DIRECT_FEED / calypso_bsp.c:653 (bloc 646-662)
/* @BEQUILLE — BSP_DIRECT_FEED  (CALYPSO_BSP_DIRECT_FEED, EQ1, calypso.env:=1 -> ACTIF)
 *   masque  : le match FN de bsp_take_for_fn (+/-BSP_FN_MATCH_WINDOW) echoue
 *             systematiquement parce que la FN du device (temps reel) et la FN
 *             virtuelle QEMU divergent -> DARAM jamais ecrite. On livre sans
 *             aucune correspondance temporelle : le burst arrive "maintenant".
 *   retirer : quand la FN virtuelle et la FN device sont alignees (FN-PROBE
 *             delta ~0 stable) ; alors bsp_enqueue -> deliver_buffered suffit.
 *   NB      : tant que ce gate vaut 1, TOUT calypso_bsp_deliver_buffered() est
 *             du code mort (file toujours vide).
 */
```

```
BSP_DISPATCH_FB (+_TGT, +_NOIMR, +_ONESHOT) / calypso_bsp.c:1145 (bloc 1136-1173)
/* @BEQUILLE — BSP_DISPATCH_FB  (CALYPSO_BSP_DISPATCH_FB, EXISTS, defaut OFF ; wire.env:=1)
 *   masque  : la LUT native 0x8341 qui installe le handler FB-det 0x8d00 dans
 *             les slots de dispatch n'est jamais atteinte (0x7234 deraille vers
 *             l'overlay 0x013b). On ecrit les slots 0x43c0/0x4387/0x43d8 a la
 *             main et on ouvre IMR bit9 a la place du scheduler.
 *   retirer : quand 0x7234 atteint 0x8341 et peuple ces slots tout seul ; le
 *             demasquage IMR est separable (CALYPSO_BSP_DISPATCH_NOIMR=1).
 */
```

```
DL_FN_OFFSET / calypso_trx.c:157 (bloc 156-168, dans calypso_trx_get_fn)
/* @BEQUILLE — DL_FN_OFFSET  (CALYPSO_DL_FN_OFFSET, VALEUR, defaut 0 = inerte)
 *   masque  : l'absence de synchronisation de la FN emulee sur la SCH du BTS.
 *             L'offset signe est applique a la SOURCE de FN, donc a tous les
 *             consommateurs (BSP match, feed shunt, FN-ALIGN, et aussi UL /
 *             DATA_IND -- porte trop large, assume dans le commentaire d'origine).
 *   retirer : quand la FN est calee sur la SCH recue (recalage a la source),
 *             l'offset mesure tombant a 0 dans FN-PROBE / FN-ALIGN.
 */
```

```
FB_IQ_OWNS / calypso_bsp.c:1212 (bloc 1205-1250)
/* @BEQUILLE — FB_IQ_OWNS  (CALYPSO_FB_IQ_OWNS, atoi>0, calypso.env:=0)
 *   masque  : deux producteurs concurrents ecrivent le meme buffer d'entree du
 *             correlateur (rx_burst cote BSP et feed_iq cote shunt). Le silicium
 *             n'a qu'un seul chemin : le BSP. Ce flag arbitre a la main qui
 *             gagne, faute d'un unique writer.
 *   retirer : quand feed_iq disparait au profit du seul chemin BSP (ou
 *             inversement) -- il ne doit rester qu'un writer de bsp.daram_addr.
 */
```

```
POKE_TASK_MD / calypso_bsp.c:1116  (et POKE_DISPATCH / calypso_bsp.c:1122, bloc 1119-1128)
/* @BEQUILLE — POKE_TASK_MD  (CALYPSO_POKE_TASK_MD, atoi>0 mais DEFAUT 1 SI ABSENTE)
 *   masque  : le descripteur de tache (d_task_md, pages API-RAM 0x0804/0x0818)
 *             n'est jamais publie vers le DSP : la DMA page-ecriture ARM->DARAM
 *             0x0586 (calypso_trx.c:956) est fermee sous shunt. Le correlateur
 *             entre donc sans mission et tourne dans le vide ; on lui pose la
 *             mission a la main. POKE_DISPATCH va plus loin et replique
 *             dsp_end_scenario (d_dsp_page = B_GSM_TASK|w_page en alternance).
 *   retirer : quand la DMA de la write-page atteint le DSP (task_md lu depuis
 *             0x0586+DB_W_D_TASK_MD) ; alors ces deux pokes deviennent nuls.
 *   NB      : defaut ON, mais imbrique dans le bloc RX_FBFLAGS (:1105) -> sans
 *             CALYPSO_RX_FBFLAGS, jamais atteint.
 */
```

```
RACH_FORCE_BSIC / calypso_bsp.c:1766 (bloc 1759-1786)
/* @BEQUILLE — RACH_FORCE_BSIC  (CALYPSO_RACH_FORCE_BSIC, VALEUR, defaut unset = inerte)
 *   masque  : l'incertitude sur l'offset NDB de d_rach : plutot que de lire le
 *             BSIC ecrit par le firmware, on impose celui du BSC pour prouver
 *             la chaine d'encodage RACH independamment de l'offset.
 *   retirer : quand CALYPSO_NDB_D_RACH_OFFSET est confirme (IMM_ASS_CMD recu
 *             avec le BSIC lu depuis d_rach, sans forcage).
 */
```

```
RX_FBFLAGS / calypso_bsp.c:1102 (bloc 1095-1134) — second site calypso_bsp.c:1569 (bloc 1558-1580)
/* @BEQUILLE — RX_FBFLAGS  (CALYPSO_RX_FBFLAGS, EXISTS, defaut OFF)
 *   masque  : l'ISR BRINT0 (PROM1[0xFFD4] -> CALL 0xf310) n'est jamais prise,
 *             donc les bits de handshake FB-det qu'elle devrait poser ne le sont
 *             pas : data[0x3fad] bit15 (verrou-maitre du kernel @0x8754),
 *             0x3faa bit2+bit8, 0x3fab bit8, 0x3fae bit8. On les pose depuis la
 *             livraison du burst.
 *   retirer : quand BRINT0 est reellement servie et que son ISR ecrit ces bits ;
 *             le bloc jumeau de deliver_buffered (:1569) est deja mort sous
 *             BSP_DIRECT_FEED=1 et omet 0x3fad -> a supprimer en premier.
 */
```

```
TPU_RX_WIRE / calypso_bsp.c:1345 (bloc 1341-1360) et calypso_bsp.c:1386 (bloc 1372-1397) et calypso_trx.c:955 (bloc 940-957)
/* @BEQUILLE — TPU_RX_WIRE  (CALYPSO_TPU_RX_WIRE, EXISTS, defaut OFF ; wire.env:=1)
 *   masque  : la fenetre RX du TPU (scenario TPU -> MOVE TSP -> IOTA BDLENA)
 *             n'est raccordee a rien : calypso_iota_take_bdl_pulse() n'a qu'un
 *             seul appelant, celui-ci. Le wire (a) consomme le pulse, (b) pose
 *             la tache FB dans le mot scheduler d[0x3f92] bit11 a la place de
 *             l'ORM natif 0xa539 jamais execute, (c) livre le burst le PLUS
 *             PROCHE en contournant la fenetre de match FN, (d) laisse passer
 *             la DMA de tache ARM->DARAM 0x0586 sous shunt.
 *   retirer : quand le sequenceur TPU produit le pulse BDLENA et que le BSP le
 *             consomme sans gate -- et quand d[0x3f92] est pose par l'ORM natif.
 *   NB      : le gate :1353 se leve AUSSI via (DSP_RUN_C54X=1 && BSP_DARAM_FORCE) ;
 *             mesure du 2026-07-28 : deliver ouvert alors que :1387 et
 *             calypso_trx.c:956 restent fermes -> wire a moitie leve.
 */
```

---

## LOT 2 — Cœur c54x : IT / IMR / INTM / TINT0 / cadence / go-live

## LOT 2 — Cœur c54x : IT / IMR / INTM / TINT0 / cadence / go-live shadow / infra debug (52 variables)

Snapshot vérifié : md5 des 11 fichiers lus **identiques** au runtime conteneur (`c36466ab…` c54x.c, `4760924b…` bsp.c, `a0f17a26…` dsp_shunt.c, `8fbe30d8…` trx.c, `ef45b3d5…` dsp_helper.c, `20bb0b1d…` debug.c, `a6f0c24d…` dbg.c, `78dd2aec…` tint0.c, `0a8a00d4…` invariants.c, `78f275a9…` sim.c). Lecture faite sur les copies locales, occurrences re-grepées dans le conteneur.

### Tableau

| VARIABLE | DEFAUT | EFFET (code exécuté) | MODE | IDIOME | CATEGORIE | REPOSE / REPOSÉE PAR |
|---|---|---|---|---|---|---|
| `C54X_BCTC_SM` | unset → OFF (`c54x.c:7721`) | Sur `BC TC/NTC` restreint à PC∈[0xde0d..0xde26] : sémantique TC réelle au lieu de l'heuristique ACC. Débloque la boucle SM handshake go-live | tous (DSP exécuté) | EXISTS | **SAS** — correctif ISA en attente de validation, mais **hors** du namespace `CALYPSO_FIXES` | — |
| `C54X_CRASHPC` | `calypso.env:104 :=1` | **Aucun.** Les 2 seules occurrences sont les arguments d'un `fprintf` (`dsp_shunt.c:840`). Aucun handler de signal n'existe | — | aucun | **MORT** | — |
| `C54X_FIX_BC` | unset → OFF (`c54x.c:7697`) | Remplace inconditionnellement l'heuristique ACC de `BC` par `c54x_cond_true(op&0xFF)` (ISA-fidèle) sur TOUS les sites | tous | EXISTS | **SAS** — même remarque que BCTC_SM (le commentaire dit « validé via chaîne de tests », jamais fait) | — |
| `C54X_FORCE_IMR` | unset → OFF (`c54x.c:14404`) ; `hack.env:16` commenté | `s->imr \|= <hex>` à chaque pas hors ISR, **ET** clear INTM dans [0xb380..0xb440] | tous | VALEUR hex (0/vide = OFF) | **BEQUILLE** | — |
| `C54X_IRQ_LEVEL` | **ON de facto** : `calypso.env:102 CALYPSO_DSP:=c54x` (`c54x.c:4933`) | Active `c54x_irq_level_check` = dispatch IT maskable standard (INTM/delay/IPTR/IFR&IMR/ctz) | tous | EXISTS **OU** `DSP=="c54x"` | **CONFIG** — modèle IT c54x canonique ; la variable est **redondante** (jamais nécessaire) | reposée par `DSP` |
| `DBG` | `run.sh:731` peut poser `=1` | **Aucun.** `calypso_dbg.c` **n'est PAS dans `meson.build`** → non compilé ; `calypso_dbg_init()` n'a aucun appelant ; `calypso_dbg.h` n'existe pas ; les `C54_DBG(...)` du code viennent de `calypso_debug.h`, pas de ce fichier | — | — | **MORT** (code non lié) | — |
| `DEBUG` | unset (aucun `.env`) ; `run.sh:192` `=ALL` en `--debug-full` | Parse liste de jetons (`debug.c:51-82`), normalise `- / . espace`→`_`, upper ; pose `calypso_debug_master` (0 si vide → 127 sites inline court-circuités) | tous | LISTE (namespace de 105 jetons) | **MESURE** | repose 115 sites + les params `SP_RING_*`, `SP_HIST_*`, `AR6_*`, `CORR_LO/HI`… |
| `DSP` | `calypso.env:102 :=c54x` | `strcmp=="c54x"` → `shunt_route_c54x()` (helper.c:19) = overlay NDB. **Et surtout deux activations silencieuses** : `C54X_IRQ_LEVEL` (`c54x.c:4933`) et `DSP_FRAME_VEC28` (`c54x.c:5011`) | tous | CHAINE `=="c54x"` | **CONFIG** (sélecteur de route) — mais **piège majeur** : allume 2 comportements non demandés | **repose** IRQ_LEVEL + FRAME_VEC28 |
| `DSP_BLOB` | unset (`run.sh:1734` : opt-in) | Chemin blob DARAM ; s'il est posé, **toutes** les sections PROM/DROM sont ignorées (`trx.c:1993`) et `run.sh:1663-1671` les force-disable | tous | VALEUR/chemin | **CONFIG** | écrase les `dsp-prom*/drom/pdrom` |
| `DSP_BUDGET` | `run.sh:1348 :=256000` ; code 256000 | Nb d'insns par `c54x_run()`. 2 lecteurs : `trx.c:1397` (clamp min 1000) et `dsp_shunt.c:535` (clamp ≤0→256000) | tous | VALEUR | **CONFIG** (cadence) | — |
| `DSP_FRAME_VEC28` | unset, mais **ON de facto** via `DSP=c54x` sur le site `c54x.c:5011` | Remappe l'IT frame vec19/bit3 → **vec28/bit12** (le stub vec19 est un `RETE`). 5 sites : `c54x.c:5011`, `c54x.c:16849`, `trx.c:1444`, `bsp.c:1069`, `bsp.c:1418` (ces 3 derniers choisissent bit 12 vs 3 pour l'anti-stack) | tous | EXISTS (**OU** `DSP=="c54x"` au site 5011 ; **OU** `FRAME_IT_NATIVE` aux 3 sites bsp/trx) | **BEQUILLE** | reposée par `DSP` ; interchangeable avec `FRAME_IT_NATIVE` |
| `DSP_GOLIVE_BOOT` | unset → OFF | 2 effets distincts : (a) `c54x.c:14678` **écrit `s->pc = 0xb3ec`** quand PC==0xb3ff (saut de la wait-loop) ; (b) `c54x.c:16858` `g_noforce` **inhibe** `VEC28-FORCE` | tous | EXISTS | **BEQUILLE** (le commentaire dit lui-même « TEST, pas fix ») | — |
| `DSP_IDLE_FF` | `run.sh:1328 :=1` → **ON** | Fast-forward des boucles dispatcher idle (déf. `0xe9ac..0xe9b7`, `0xcc62..0xcc6f`) ; s'abstient si une tâche est postée (`c54x.c:11245`) ou si IT pending | tous | ON-sauf-0 | **CONFIG** (perf/cadence, ne change pas la sémantique) | repose `DSP_IDLE_RANGE` |
| `DSP_IDLE_RANGE` | `run.sh:1329 :=` (vide) → défauts code | `"lo:hi,lo:hi"` hex, max 4 plages ; vide → 2 plages par défaut. `run.sh:1421` force `IDLE_FF=1` si RANGE non vide | tous | VALEUR/chaîne | **CONFIG** | reposée par `DSP_IDLE_FF` |
| `DSP_REG_MODE` | **code : `bin`** ; **`run.sh:1648 :=c54x` (exporté)** → runtime = `c54x` | Source de l'état registres au reset : `c54x`=hardcode C seul (**le `Registers.bin` silicium est IGNORÉ**), `bin`=snapshot verbatim, `hybrid`=bin sauf IFR/AR0/BRC/RSA/REA | tous | CHAINE (`c54x`\|`hybrid`\|sinon `bin`) | **CONFIG** | **écart code↔runtime à signaler** : le défaut documenté (`bin`) n'est jamais celui qui tourne |
| `DSP_RUN_C54X` | `calypso.env:103 :=1` ; `native/native_helped :=1` ; `shunt_legit/no_legit :=0` | 7 sites : gate `bsp_revive` (`bsp.c:465`), `rb_revive` (`bsp.c:990`), gate delivery (`bsp.c:1352`), runner shunt (`dsp_shunt.c:605`), header route (`dsp_shunt.c:836`), earlyboot `c54x_run(2000)` (`dsp_shunt.c:2150`) ; **posé par setenv** en `dsp_shunt.c:94` | tous | EQ1 | **CONFIG** (enable du bloc modélisé) | **posé** par la value-list `SHUNT_LEGIT=…DSP…` ; **repose** `BSP_DARAM_FORCE`/`TPU_RX_WIRE` (bsp.c:1354) |
| `DSP_SHUNT` | **run par défaut = 1** (`calypso.env:108 MODE:=full-grgsm` → `run.sh:1137 :=1`) ; `native*/env :=0` ; `shunt_*` `:=1` | `dsp_shunt.c:1855` arme le shunt ; `dsp_shunt.c:2057` `substitutes()` → gate TOUS les `c54x_run` de `trx.c:1407` | tous | CHAINE `strcmp=="1"` | **BEQUILLE** (parapluie : remplace le DSP par un mock ARM) | reposée par `CALYPSO_MODE` (**oublié systématiquement**) ; battue par les profils `native*` sourcés AVANT run.sh |
| `DSP_TIMER_OFF` | unset → timer **ON** | Coupe entièrement le tick TIMER0 (`c54x.c:16225` → `_tmr=0`) | tous | EXISTS-INV | **CONFIG** (kill-switch d'un périphérique modélisé) | — |
| `DSP_YIELD` | **32768 si absent** (`c54x.c:16381`) | Insns entre deux yields de la boucle DSP ; `=0` = OFF legacy | tous | VALEUR (déf 32768, ON) | **CONFIG** (cadence) | — |
| `FIRMWARE_ELF` | unset → fallback `-kernel` de `/proc/self/cmdline` | Chemin de l'ELF où résoudre dynamiquement les symboles firmware (`l1s_fn`, `last_rach_fn`) | tous (shunt) | VALEUR/chemin | **CONFIG** | fallback de `L1S_FN_ADDR`/`LAST_RACH_FN_ADDR` (lot 4) |
| `FLOWTRACE` | unset → OFF | Budget de lignes R/W vers `/tmp/calypso_flow.txt`, **plage 0x2800-0x2FFF seulement**, et **uniquement une fois `g_flow_armed=1`**, posé au seul `exec_pc==0xa076` (`c54x.c:15179`) — kernel MAC jamais atteint aujourd'hui → **0 ligne en pratique** | tous | VALEUR (budget) | **MESURE** | armée par l'atteinte de 0xa076 |
| `FORCE_INTM_AT_PC` | unset (`run.sh:741` menu) → sentinelle 0xFFFF | Restreint le one-shot à un PC. **Avec PC-gate le code POSE `s->ifr \|= (imr & 0x3000)`** (`c54x.c:1058`) — il fabrique l'IT, il ne fait pas qu'ouvrir la fenêtre | tous | VALEUR | **BEQUILLE** | inerte sans `FORCE_INTM_ONESHOT=1` |
| `FORCE_INTM_ONESHOT` | unset ; **`wire.env:22 :=1`** | Clear `ST1.INTM` UNE fois (`c54x.c:1073`). Sans PC-gate : exige IT déjà pending + `insn>1e6` | tous | EQ1 | **BEQUILLE** (`run.sh:1417` le signale déjà « NON-nominal ») | repose `FORCE_INTM_AT_PC` |
| `FRAME_IT_LEVEL` | unset → OFF | Maintient IFR bit12 asserté à chaque insn tant que vec28 n'a pas vectorisé (`c54x.c:4938`), relâché en 5015 ; armé en 16885 | tous | EQ1 | **BEQUILLE** — l'IFR c54x est à latch d'événement, pas « level-hold » | — |
| `FRAME_IT_NATIVE` | `native/native_helped/wire :=1` | `dsp_shunt.c:513` : livre directement `c54x_interrupt_ex(dsp,28,12)` au frame-tick au lieu de 19/3 ; + choix bit12 pour l'anti-stack (`bsp.c:1069/1418`, `trx.c:1444`) ; + remap dans `c54x_interrupt_ex` (`c54x.c:16850`) | NATIVE, NATIVE_HELPED, WIRE | EXISTS | **BEQUILLE** | interchangeable avec `DSP_FRAME_VEC28` sur 3 sites ; **inhibe** `VEC28-FORCE` (`c54x.c:16874 !g_native`) |
| `FRAME_IT_PRIO` | unset → OFF | Force `b=12` quand bit12 pend, au lieu du `ctz(pend)` (`c54x.c:5002`) → la frame vole la fenêtre à BRINT0/bit5 | tous | EQ1 | **BEQUILLE** — la priorité c54x est fixée par le n° de vecteur, pas configurable | — |
| `FRAME_IT_PROBE` | unset → OFF | Log `d_dsp_page`/B_GSM_TASK à chaque frame-IT (`c54x.c:16864`) | tous | EXISTS | **MESURE** | n'est atteint que si `DSP_FRAME_VEC28`\|`FRAME_IT_NATIVE` |
| `INIT_435B_OFF` | `hack/native/native_helped/wire :=0` → **injection ACTIVE** | À `exec_pc==0xa4e4`, si `data[0x435b]==0` : écrit **0x52ed** (ou 0x52fd) = shadow IMR jamais initialisé en QEMU | tous | INV-VAL (**`=0` ACTIVE**, `=1` coupe) | **BEQUILLE** | repose `SEED_52FD` |
| `INTM_TRANS` | **`calypso.env:190 :=1`** (inconditionnel) ; `wire.env:33` | Trace chaque bascule du bit INTM avec PC/op (`c54x.c:14991`). **Sans rapport avec le jeton `CALYPSO_DEBUG=INTM-TRANS`** (autre bloc, `c54x.c:12863`, via `C54_LOG`) | tous | **NON-VIDE → `=0` l'ACTIVE** | **MESURE** | — |
| `INVARIANTS` | unset → OFF (`invariants.c:26`) | Comptabilise/imprime les violations. **2 appelants seulement** : `c54x.c:3536` (`correlator_ar4_sweeps`) et `c54x.c:3540` (`correlator_ar5_in_iq_buffer`). Retour ignoré | tous | EQ1 | **MESURE** | — |
| `IT_PUSH_XPC_ALWAYS` | unset → OFF = **comportement silicium** (push PC seul) | Si actif : pousse aussi XPC même quand `xpc==0` (`c54x.c:5027`) → mot orphelin jamais dépilé → drift SP +1/IT → storm PC=0 | tous | **NON-VIDE → `CALYPSO_IT_PUSH_XPC_ALWAYS=0` RÉACTIVE LE BUG** | **CONFIG** (compat legacy inversée) — piège d'idiome le plus dangereux du lot | — |
| `KEEP_IMR` | `hack/native/native_helped/wire :=1` → **ON** | Sur PC∈[0xa4ca..0xdea0], si `imr & 0x0020 == 0` : `s->imr = data[0x435b]` (repli `KEEP_IMR_VAL`) — écrase l'IMR que le firmware vient de poser | tous profils sauf run nu | EXISTS | **BEQUILLE** | repose `KEEP_IMR_VAL` ; dépend de `INIT_435B_OFF` (peuple `d[435b]`) |
| `KEEP_IMR_VAL` | code 0x52fd ; aucun `.env` | Valeur de repli quand le shadow n'a pas bit5 | idem | VALEUR | **BEQUILLE** (paramètre de béquille) | reposée par `KEEP_IMR` |
| `ORCH` | unset → OFF | **Aucun.** `calypso_orch()` est défini `orch.h:10` et **`calypso_orch.h` n'est inclus par aucun `.c`/`.h`** ; aucun appel | — | ON-sauf-0 (théorique) | **MORT** | — |
| `ORPHAN` | unset → OFF | Ring shadow-stack push/pop pour nommer le retour orphelin. **2 sites d'idiomes DIFFÉRENTS** : `c54x.c:2508` `EXISTS`, `c54x.c:15032` `NON-VIDE` → `ORPHAN=` (vide) allume le ring mais pas l'appariement | tous | EXISTS **et** NON-VIDE (incohérent) | **MESURE** | gate `TRACK_STKVAL` (2508 return early) |
| `PCB_TICK_THREADS` | `run.sh:1377-1380` : forcé à `0` si `MTTCG=1` | Si `=1` : n'arme PAS les QEMUTimer (`tint0.c:63`, `trx.c:1779`) — les threads PCB self-pacent | tous | EQ1 | **CONFIG** (modèle d'ordonnancement) | reposée par `CALYPSO_MTTCG` |
| `SEED5AC8` | `hack.env:21 :=` (vide) → `atoi("")=0` → **OFF** | À `exec_pc==0xb382` (le `STM #0x5ac8,SP` réel) : `data[0x5ac8] = SEED5AC8_VAL` | HACK | VAL>0 | **BEQUILLE** (le code écrit lui-même « LE SEED N'EST PAS UN FIX… band-aid ») | repose `SEED5AC8_VAL` |
| `SEED5AC8_VAL` | code 0x71f4 ; `wire.env:14 :=0xa4c7` | 0x71f4 → trampoline vers 0xa4df (saute `RSBX INTM`) ; 0xa4c7 → entrée par `ORM #0x3000,IMR` + enable natif | idem | VALEUR | **BEQUILLE** (paramètre) | inerte sans `SEED5AC8` |
| `SEED_52FD` | unset → 0x52ed | Choisit la valeur injectée dans `data[0x435b]` : présent → 0x52fd (bit4/TINT, casse le frame), absent → 0x52ed | tous | EXISTS | **BEQUILLE** (paramètre) | inerte si `INIT_435B_OFF=1` |
| `SIM_CFG` | `run.sh:1304 :=$MOBILE_CFG` (exporté) | Charge IMSI/Ki depuis le même fichier que `mobile` (`sim.c:789`) | tous | VALEUR/chemin | **CONFIG** | — |
| `SP_HIST_ARM` | `run.sh:812` (exporté vide) ; code **0x2000** | Seuil SP d'armement de l'histogramme de décrémentations | tous | VALEUR | **MESURE** | **inerte sans `CALYPSO_DEBUG=SP-HIST`** (`c54x.c:11597`) |
| `SP_HIST_DUMP` | code **0x0100** ; clampé à `arm-0x100` si ≥ arm | Seuil SP de dump | tous | VALEUR | **MESURE** | idem |
| `SP_RING_INSN_MIN` | code **1000000** | Insn minimal avant déclenchement du dump ring | tous | VALEUR | **MESURE** | **inerte sans `CALYPSO_DEBUG=SP-RING`** (`c54x.c:11432`) |
| `SP_RING_MAX` | code **4** | Nb max de dumps du ring SP | tous | VALEUR | **MESURE** | idem |
| `SP_RING_TRIG` | code **`bootstub`** (toute valeur inconnue → bootstub) | `bootstub`(2) \| `floor`(1) \| `both`(3) | tous | CHAINE | **MESURE** | idem |
| `TINT0_MASTER` | **unset par défaut** — `calypso.env:222` et `wire.env:20` sont **sous `if CALYPSO_WIRE=1`** (défaut 0) | 2 effets vivants : `c54x.c:16233-16234` force `PRD=0xFFFF` et **fait tourner le timer MALGRÉ `TCR.TSS=1`** (le firmware l'a arrêté) ; `dsp_shunt.c:521` fire `c54x_interrupt_ex(dsp,20,4)` au frame-tick. Le 3ᵉ site (`c54x.c:12847`) est **NEUTRALISÉ** (`(void)_t0i;` l.12861) | WIRE | EXISTS | **BEQUILLE** | repose `TINT0_PERIOD` (mort) |
| `TINT0_PERINSN` | unset → OFF | Fire `c54x_interrupt_ex(s,20,4)` toutes les 2000 insns (`c54x.c:16221`). **Le commentaire « Ce bloc desactive » est FAUX** : le bloc est exécuté, seul l'env manquant l'éteint | tous | EXISTS | **BEQUILLE** (horloge artificielle legacy) | — |
| `TINT0_PERIOD` | `:=1500` sous WIRE ; code 1500 | **Aucun.** Lue en `c54x.c:12849` puis `(void)_t0period;` en 12861. Unique lecteur | — | VALEUR | **MORT** | — |
| `TRACE_VEC28_STACK` | unset → OFF | Arme la trace pile à l'entrée vec28 (2 sites : `c54x.c:16980` idle-wake, `c54x.c:17041` normal) ; consommée aux RET (`c54x.c:6424/6462`) | tous | EXISTS | **MESURE** | — |
| `TRACK_STKVAL` | code **0x3125** | Log du CALL/push posant cette valeur sur la pile (`c54x.c:2523`) | tous | VALEUR | **MESURE** | **inerte sans `ORPHAN`** (`c54x.c:2509` return) |
| `VECTAB` | unset → OFF | Dump one-shot de la table de vecteurs 0x0080-0x00FF au 2ᵉ passage à `exec_pc==0xb01c` | tous | EXISTS | **MESURE** | — |
| `WATCH_VEC` | unset → OFF | Log des écritures vers 0x0080-0x00FF et 0x0138-0x013C (`c54x.c:3425`) | tous | **NON-VIDE → `=0` l'ACTIVE** | **MESURE** | — |

---

### Effets de bord / dépendances mesurés (les plus opérationnels du lot)

1. **`CALYPSO_DSP=c54x` (défaut `calypso.env:102`) allume DEUX gates non demandés** : `C54X_IRQ_LEVEL` (`c54x.c:4933`) et `DSP_FRAME_VEC28` (`c54x.c:5011`). Retirer `CALYPSO_DSP_FRAME_VEC28` de la ligne de commande **ne coupe pas** le remap vec19→vec28 dans le chemin IRQ-LEVEL.
2. **Ordre de sourcing** : `calypso.env:11-19` source les profils `shunt_no_legit / shunt_legit / native / native_helped` **AVANT** `calypso.env:102-108`, et `run.sh` tourne **après**. Conséquence mesurable : `native_helped.env:28 DSP_SHUNT:=0` gagne sur `run.sh:1137 DSP_SHUNT:=1`. **Mais un run nu (`./start-clean.sh` sans profil) part avec `CALYPSO_MODE=full-grgsm` → `DSP_SHUNT=1`**, alors même que `calypso.env:103` pose `DSP_RUN_C54X=1`. Les deux ne se contredisent pas : `substitutes()` gate `trx.c:1407` et le DSP n'est exécuté que par le runner shunt (`dsp_shunt.c:614`).
3. **`CALYPSO_TINT0_MASTER` / `TINT0_PERIOD` ne sont PAS des défauts globaux** : les lignes `calypso.env:222-223` sont dans le bloc `if [ "${CALYPSO_WIRE:-0}" = "1" ]`. Hors profil WIRE, `TINT0_MASTER` est unset.
4. **`INIT_435B_OFF=0` = injection ACTIVE** (idiome `INV-VAL`). Les 4 `.env` (`hack`, `native`, `native_helped`, `wire`) le posent à `0` : la béquille tourne dans les 4 profils.
5. **`IT_PUSH_XPC_ALWAYS=0` ACTIVE la béquille** (idiome `NON-VIDE`). Le seul moyen de rester sur le comportement silicium est de ne PAS définir la variable.
6. **`DSP_REG_MODE`** : le code documente `bin` comme défaut, mais `run.sh:1648` exporte `c54x` → le `Registers.bin` (snapshot silicium) est **ignoré à chaque run**. Écart code↔runtime jamais signalé jusqu'ici.
7. **`run.sh:731 export CALYPSO_DBG=1`** produit `[dbg] unknown category '1'` puis `mask |= default_mask` (`dbg.c:95`) → identique au défaut. Sans importance puisque le fichier n'est pas compilé.
8. **`ORPHAN` a deux idiomes contradictoires** dans le même fichier (`EXISTS` en 2508, `NON-VIDE` en 15032) : `CALYPSO_ORPHAN=` (chaîne vide) allume le ring et éteint l'appariement.
9. Les 5 paramètres `SP_RING_*` / `SP_HIST_*` sont lus **inconditionnellement** mais leurs gates respectifs viennent de `CALYPSO_DEBUG=SP-RING` / `SP-HIST` : ce sont des paramètres de sondes, pas des sondes.
10. **3 MORTES confirmées dans ce lot** : `CALYPSO_DBG` (fichier absent de `meson.build` → non lié), `CALYPSO_ORCH` (`calypso_orch.h` inclus nulle part), `CALYPSO_TINT0_PERIOD` (lue puis `(void)`), plus `CALYPSO_C54X_CRASHPC` (log seul) déjà signalée par le recensement.

---

### Blocs @BEQUILLE prêts à coller

```
C54X_FORCE_IMR / hw/arm/calypso/calypso_c54x.c:14396 (bloc 14402-14426)
/* @BEQUILLE — FORCE_IMR  (CALYPSO_C54X_FORCE_IMR, defaut OFF)
 *   masque  : le re-armement de l IMR apres le STM #0,IMR du mask-ROM @0xb37e,
 *             et le RSBX INTM que le ROM ne joue qu apres go-live. On OR les bits
 *             dans l IMR a chaque pas hors ISR et on clear INTM dans [0xb380..0xb440].
 *   retirer : quand la SM go-live atteint 0xa582 et pose l IMR elle-meme.
 */
```
```
DSP_GOLIVE_BOOT / hw/arm/calypso/calypso_c54x.c:14667 (bloc 14673-14687)  + 2e site 16857-16881
/* @BEQUILLE — GOLIVE_REDIRECT  (CALYPSO_DSP_GOLIVE_BOOT, defaut OFF)
 *   masque  : ecrit s->pc = 0xb3ec quand le DSP atteint 0xb3ff, cest-a-dire le
 *             choix de soft-vector go-live que le boot ROM ne fait pas dans notre
 *             modele. Second effet : inhibe VEC28-FORCE (c54x.c:16874).
 *   retirer : quand data[0x3f6d] est peuple par le chemin ROM et pointe 0xa4c7.
 */
```
```
DSP_SHUNT / hw/arm/calypso/calypso_dsp_shunt.c:1847 (init)  + calypso_dsp_shunt.c:2053 (substitutes)
/* @BEQUILLE — DSP_SHUNT  (CALYPSO_DSP_SHUNT, defaut 1 via CALYPSO_MODE=full-grgsm)
 *   masque  : le DSP entier. substitutes() gate TOUS les c54x_run (trx.c:1407) et
 *             un mock cote ARM ecrit d_fb_det / a_sch / a_cd a sa place.
 *   retirer : quand le correlateur natif produit d_fb_det seul (RANK3 leve).
 *   NB : le defaut ne vient PAS dun calypso*.env mais de run.sh:1137 via CALYPSO_MODE.
 */
```
```
DSP_FRAME_VEC28 / hw/arm/calypso/calypso_c54x.c:5006 (bloc 5009-5013)
   sites solidaires : c54x.c:16849, trx.c:1444, bsp.c:1069, bsp.c:1418
/* @BEQUILLE — VEC28_REMAP  (CALYPSO_DSP_FRAME_VEC28, ACTIF de facto via CALYPSO_DSP=c54x)
 *   masque  : le mapping ligne-frame-TPU -> vecteur DSP. Le modele livre lIT frame
 *             sur vec19/bit3 (= stub RETE) ; on la reroute vers vec28/bit12.
 *   retirer : quand calypso_tpu.c cable la ligne frame sur le bon vecteur a la source.
 *   PIEGE : allumee sans etre demandee des que CALYPSO_DSP=c54x (c54x.c:5011).
 */
```
```
FRAME_IT_NATIVE / hw/arm/calypso/calypso_dsp_shunt.c:505 (bloc 509-515)
   sites solidaires : c54x.c:16850, trx.c:1444, bsp.c:1069, bsp.c:1418
/* @BEQUILLE — FRAME_IT_NATIVE  (CALYPSO_FRAME_IT_NATIVE, defaut 1 en native/native_helped/wire)
 *   masque  : la meme absence de cablage frame-TPU -> vecteur DSP, cote frame-tick :
 *             on appelle directement c54x_interrupt_ex(dsp,28,12) au lieu de 19/3.
 *   retirer : idem VEC28_REMAP — quand le TPU delivre lIT frame sur vec28 tout seul.
 */
```
```
FRAME_IT_LEVEL / hw/arm/calypso/calypso_c54x.c:4916 (fonction + sites 4937-4939, 5015, 16885)
/* @BEQUILLE — FRAME_IT_LEVEL  (CALYPSO_FRAME_IT_LEVEL, defaut OFF)
 *   masque  : la fenetre INTM=0 trop rare du firmware. Re-assert IFR bit12 a CHAQUE
 *             insn tant que vec28 na pas vectorise — lIFR c54x est a latch devenement,
 *             il na pas de mode "level".
 *   retirer : quand la cadence INTM du firmware suffit a attraper lIT au vol.
 */
```
```
FRAME_IT_PRIO / hw/arm/calypso/calypso_c54x.c:4923 (fonction + site 5000-5004)
/* @BEQUILLE — FRAME_IT_PRIO  (CALYPSO_FRAME_IT_PRIO, defaut OFF)
 *   masque  : la priorite dinterruption. Force b=12 au lieu de ctz(pend) pour que la
 *             frame passe devant BRINT0/bit5 — sur c54x la priorite est fixee par le
 *             numero de vecteur, elle nest pas configurable.
 *   retirer : quand BRINT0 et la frame ne se disputent plus la meme fenetre (livraison
 *             BSP a la bonne cadence).
 */
```
```
INIT_435B_OFF + SEED_52FD / hw/arm/calypso/calypso_c54x.c:13665 (bloc 13671-13680)
/* @BEQUILLE — INIT_435B  (CALYPSO_INIT_435B_OFF=0 => ACTIVE ; CALYPSO_SEED_52FD choisit la valeur)
 *   masque  : linitialisation du shadow IMR data[0x435b] par le boot DSP. Jamais ecrit
 *             en QEMU -> la SM 0xa582 propage IMR=0 -> deadlock. On injecte 0x52ed
 *             (ou 0x52fd avec SEED_52FD) a exec_pc==0xa4e4.
 *   retirer : quand une ecriture firmware sur 0x435b est observee avant 0xa4e4.
 *   PIEGE didiome : le nom dit _OFF mais =0 ACTIVE ; les 4 profils .env le posent a 0.
 */
```
```
KEEP_IMR + KEEP_IMR_VAL / hw/arm/calypso/calypso_c54x.c:14266 (bloc 14266-14281)
/* @BEQUILLE — KEEP_IMR  (CALYPSO_KEEP_IMR, defaut 1 en hack/native/native_helped/wire)
 *   masque  : le clobber de lIMR par 0xb37e (STM #0,IMR) et 0xa509 (strip bit12).
 *             On re-ecrit s->imr = data[0x435b] (repli CALYPSO_KEEP_IMR_VAL=0x52fd)
 *             des que bit5/BRINT0 tombe, sur toute la region [0xa4ca..0xdea0].
 *   retirer : quand le firmware ne perd plus bit5 — cest-a-dire quand la sequence
 *             go-live 0xa4c7/0xa51b/0xa582 se deroule dans le bon ordre.
 */
```
```
SEED5AC8 + SEED5AC8_VAL / hw/arm/calypso/calypso_c54x.c:14108 (bloc 14108-14132)
/* @BEQUILLE — SEED_5AC8  (CALYPSO_SEED5AC8, defaut OFF ; _VAL defaut 0x71f4)
 *   masque  : le peuplement de mem[0x5ac8] (mot depile par le RET terminal 0xab38,
 *             qui choisit lentree go-live). Personne ne lecrit dans notre modele.
 *   retirer : quand on sait QUI ecrit mem[0x5ac8] sur silicium — le commentaire du
 *             bloc dit deja "NE PAS traiter le seed en fix".
 */
```
```
TINT0_MASTER / hw/arm/calypso/calypso_c54x.c:16224 (bloc 16226-16252)  + calypso_dsp_shunt.c:516 (bloc 518-522)
/* @BEQUILLE — TINT0_MASTER  (CALYPSO_TINT0_MASTER, defaut OFF hors profil WIRE)
 *   masque  : la configuration du TIMER0 par le ROM (TCR/PRD). Le firmware arrete le
 *             timer (TSS=1) dans une init non-tournee ; on force PRD=0xFFFF et on tick
 *             malgre TSS, plus un fire TINT0 vec20/bit4 au frame-tick du shunt.
 *   retirer : quand la sequence dinit TIMER0 du ROM sexecute (TCR programme, TSS=0).
 *   NB : le 3e site (c54x.c:12847) est mort — neutralise par (void)_t0i; ligne 12861.
 */
```
```
TINT0_PERINSN / hw/arm/calypso/calypso_c54x.c:16216 (bloc 16216-16223)
/* @BEQUILLE — TINT0_PERINSN  (CALYPSO_TINT0_PERINSN, defaut OFF)
 *   masque  : labsence de base de temps DSP. Fire TINT0 (vec20/bit4) toutes les 2000
 *             insns, sans aucun rapport avec la cadence TDMA.
 *   retirer : remplace par le tick TIMER0 fidele juste en dessous (c54x.c:16234).
 *   ATTENTION : le commentaire "Ce bloc desactive" est FAUX — le code est execute.
 */
```
```
FORCE_INTM_ONESHOT + FORCE_INTM_AT_PC / hw/arm/calypso/calypso_c54x.c:1012 (fonction force_intm_oneshot_check, 1012-1080)
/* @BEQUILLE — FORCE_INTM_ONESHOT  (CALYPSO_FORCE_INTM_ONESHOT=1 ; PC gate CALYPSO_FORCE_INTM_AT_PC)
 *   masque  : le RSBX INTM 0xa51b que le firmware ne joue pas. Avec PC-gate le bloc
 *             POSE EN PLUS lIT : s->ifr |= (s->imr & 0x3000) (l.1058) — il fabrique
 *             levenement, il nouvre pas seulement la fenetre.
 *   retirer : quand INTM passe a 0 par le chemin ROM (voir trace INTM-TRANS).
 *   run.sh:1417 le signale deja comme "NON-nominal".
 */
```

---

## LOT 3 — Opcodes / ISA / mask-ROM / bootstrap / dispatcher

Snapshot vérifié : `md5(calypso_c54x.c)=c36466abc34cef0a3c2120d9d281231f` identique au recensement — **toutes les lignes du recensement sont exactes**, aucune re-numérotation nécessaire. Mode mesuré sur le process vivant (`pid 169194`, lecture `/proc/PID/environ`, aucune relance) : `NATIVE=1, NATIVE_HELPED=0, HACK=0, WIRE absent, MODE=full-grgsm, DSP_RUN_C54X=1, DSP_SHUNT=0`.

# LOT 3 — opcodes / ISA / mask-ROM / bootstrap / dispatcher / redirections (50 variables)

**Préalable de mode qui vaut pour 48 des 50** : tous ces gates vivent dans `calypso_c54x.c`, donc dans le pas d'instruction du cœur c54x. `calypso_dsp_shunt.c:836` ne lance le c54x que si `DSP_RUN_C54X=="1"`. Sous `SHUNT_LEGIT=1` / `SHUNT_NO_LEGIT=1` (qui posent `DSP_RUN_C54X=0`), **tout le lot 3 est inerte**, y compris les béquilles actives par défaut. Modes concernés : `NATIVE`, `NATIVE_HELPED`, `SHUNT_LEGIT=DSP` (value-list `dsp_shunt.c:94` qui `setenv` `DSP_RUN_C54X=1`), et le défaut `calypso.env`. Exceptions : `BOOTCMD` (site ARM `calypso_trx.c:485`) et `B2IN` (`calypso_dsp_shunt.c:2097`, chemin FB_STREAM) qui vivent hors c54x.

| VARIABLE | DEFAUT | EFFET (code exécuté) | MODE | IDIOME | CATEGORIE | REPOSE / REPOSEE PAR |
|---|---|---|---|---|---|---|
| `AB38` | absent (OFF) | c54x.c:15593-15616. À `exec_pc==0xab38` : opdump 24 mots + 120 pas de flow, 3 armements. Aucune mutation. | c54x exécuté | EXISTS | **MESURE** — log seul | — |
| `B1` | absent (OFF) | 2 sites : c54x.c:5091-5102 (dump `data[0x2c00..0f]`+cksum au kernel MAC `0xa076`, cap 20) ; c54x.c:3061-3072 (watchpoint écritures `[0x2c00,0x2c10)`, cap 64). Aucune mutation. | c54x exécuté | EXISTS | **MESURE** | — |
| `B2` | absent (OFF) | c54x.c:15782-15795, à `0x9ac0` : \|A\|,\|B\| + max/indice sur 296 mots de `0x2a00` et `0x2c00`, cap 24. | c54x exécuté | EXISTS | **MESURE** | — |
| `B2AR` | absent (OFF) | 2 sites : c54x.c:15180-15196 (`0xa076`, min/max AR5 + IN_BUF, compteur **hors gate**) ; c54x.c:15771-15778 (`0x9ac0`, AR2-5 + IN/OOB `[0x2a00..0x2b27]`, cap 12). | c54x exécuté | EXISTS | **MESURE** | — |
| `B2IN` | absent (OFF) | dsp_shunt.c:2091-2112, dans `calypso_dsp_shunt_fb_stream_next()` : max\|I\|/\|Q\|+énergie par fenêtre de 296, cap 30. | FB_STREAM actif (native) | EXISTS | **MESURE** | inerte si `FB_STREAM` n'est pas consommé |
| `B2SEQ` | absent (OFF) | 2 sites : c54x.c:15650-15658 (`0x93a5`, 16 paires I/Q de `0x2a00`) ; c54x.c:15755-15761 (`0x9ac0`, idem), cap 8 chacun. | c54x exécuté | EXISTS | **MESURE** | — |
| `B3_TRACE` | absent (OFF) | c54x.c:14489-14499. Plage `0xb380..0xb440` : PC/op/`d[0fff]`/`d[08E2]`/`d[3f70]`/TC, cap 500. | c54x exécuté | EXISTS | **MESURE** | — |
| `B4` | absent (OFF) | c54x.c:3052-3060. Watchpoint write `data[0x08f8]` (d_fb_det), cap 64. | c54x exécuté | EXISTS | **MESURE** | — |
| `B4B` | absent (OFF) | c54x.c:15162-15177. Fenêtre de flow armée, 600 pas, désarmée à `0xec07` ou `0x8d00..0x8d10`. | c54x exécuté | EXISTS | **MESURE** | — |
| `BACC_C827_OFF` | absent ⇒ **ON** | c54x.c:13765-13779. Log de la provenance des entrées `0xc827` non-fall-through, cap 15. `_pp827` mis à jour hors gate. | c54x exécuté | EXISTS-INV (présence coupe) | **MESURE** (défaut ON) | — |
| `BOOTCMD` | absent (OFF) | 2 sites indépendants : trx.c:483-495 (MMIO ARM `0x0FF8..0x0FFF`, décodage BL_*), c54x.c:3009-3018 (writes DSP `0x0FFC..0x0FFF`), cap 40 chacun. | trx.c = **tous modes** ; c54x.c = c54x exécuté | EXISTS | **MESURE** | — |
| `D247` | absent (OFF) | c54x.c:12312-12322. À `pc==0xb3e4`, one-shot : **push `0xb3e4` sur la pile + `s->pc=0xd247`** → force le bootstrap opérationnel. | c54x exécuté | EXISTS | **BEQUILLE** — détourne le PC | ⚠️ le commentaire :12311 annonce « OFF via `CALYPSO_D247_OFF=1` » : **cette variable n'existe pas**, le gate réel est opt-in `CALYPSO_D247` |
| `D247_TRACE_OFF` | absent ⇒ **ON** | **9 sites** : c54x.c:12347, 12379, 12417, 12455, 12473, 12486, 12502, 12524, **13720**. Le dernier (CALA-WIDE) balaie `0x7000..0xdfff` et logge tout transfert calculé (`f4e2/f4e3/f4e6/f4e7/f5e2/f5e3/f5e6/f5e7/f6e6/f6e7`), les hits « dans corrélateur » **non cappés**. Aucune mutation. | c54x exécuté | INV-VAL (`=1` coupe) | **MESURE** (défaut ON, coût CPU réel) | un seul nom pour 9 sondes hétérogènes |
| `DEMOD_NOCLOBBER` | absent (OFF) | c54x.c:3033-3044, dans `data_write` : si l'écriture vise `[0x2a00,0x2b28)` **et** `pc∈{0x9fb8,0x9fe2}` → `return` = **écriture supprimée**. | NATIVE / NATIVE_HELPED (étage démod émulé) | VAL>0 | **BEQUILLE** — élimine une écriture DSP | complémentaire de `FB_IQ_DARAM` / `FB_IQ_OWNS` (lot 1/5) |
| `DISPIDX` | absent (OFF) | c54x.c:15535-15550. Capture A à `0xb0f0`, résout `data[0x4387+idx]` à `0xb0f6`, cap 80. | c54x exécuté | EXISTS | **MESURE** | — |
| `DISPTAB` | absent (OFF) | 2 sites : c54x.c:3019-3027 (writes `0x4380..0x43cf`, cap 60) ; c54x.c:15551-15563 (dump 80 mots à `0xb0f1`, cap 4). | c54x exécuté | EXISTS | **MESURE** | — |
| `DISPWATCH` | absent (OFF) | c54x.c:15345-15368. 5 PC (`b40f/b01c/b01e/b0f0/b0f6`), 2 compteurs séparés FB/non-FB. | c54x exécuté | EXISTS | **MESURE** | — |
| `FIXES` | absent | c54x.c:5054-5079 (parseur), 5307 (garde externe `getenv`), 11 appels. **Le recensement annonce 7 sous-clés : c'est FAUX, il y en a 11.** | c54x exécuté | LISTE (`all` ou CSV) | **SAS** — par construction | voir bloc dédié ci-dessous |
| `FIX_3FCD` | absent (OFF) ; **`calypso_wire.env:50 := 1`** | c54x.c:2616-2627, dans `data_write` : si `addr==0x3fcd && pc==0x013b && val!=0xa4e4` → **`val = 0xa4e4`** (substitution de l'adresse de retour du prologue ISR). | c54x exécuté | EXISTS | **BEQUILLE** — force une valeur | posée à 1 par `CALYPSO_WIRE=1` |
| `FIX_DPAGE_OFF` | absent ⇒ **BÉQUILLE ACTIVE** | c54x.c:13654-13664. À `0xa51c` ou `0xc8ea` : **`data[0x08d4] = data[0x08E2]`** (miroir d_dsp_page). Active dans le run vivant (variable absente de l'env mesuré). | c54x exécuté | EXISTS-INV | **BEQUILLE** — active par défaut, la plus discrète du lot | commentaire :13647 parle de « `CALYPSO_FIX_DPAGE` » : nom inexistant |
| `FIX_MVDM_OFF` | absent ⇒ **ON** | c54x.c:9216-9237. Décode `0x72xx` MVDM (MMR←data[dmad]) et `0x73xx` MVMD (data[dmad]←MMR) sur 2 mots, `return`. Sans lui : fallthrough STL générique. | c54x exécuté | EXISTS-INV | **SAS** — correctif ISA validé dont la condition n'a pas été effacée (protocole c54x.c:5049) | ⚠️ `calypso.env:183 : "${CALYPSO_FIX_MVDM:=1}"` = **MORT** (jamais lu ; confirmé au runtime : exporté, sans effet) |
| `FIX_PORTR` | absent (OFF) | c54x.c:9089-9100. Si ON **et** `PA∈{0xF430,0x0034}` : `data_write(addr, bsp_buf[bsp_pos++])`. **Si OFF, PORTR est un no-op** (`consumed=2; return`) → aucun échantillon I/Q livré par le port. La sonde PORTR-ANY (9076-9081) reste inconditionnelle. | c54x exécuté | EXISTS | **SAS** — implémentation en attente, hors sas `CALYPSO_FIXES` où elle devrait être | — |
| `FIX_SFTL_RSBX` | absent (OFF) | **2 sites, sémantiques divergentes.** Site 1 c54x.c:6929-6948 (sous `switch(hi4)`, avant `hi8==0xF4`) : la variable est lue à :6932 puis **JAMAIS UTILISÉE** — l'exclusion `(op&0xF0)!=0xB0` est inconditionnelle depuis 2026-07-20. Site 2 c54x.c:7216-7231 (sous `if (hi8==0xF4)`, :6967) : `(fix_sftl_rsbx2 == 0 \|\| (op&0xF0) != 0xB0)` → **variable absente ⇒ `==0` vrai ⇒ `0xF4Bx` (RSBX/SSBX ST0) est encore avalé en SFTL bidon.** `RSBX INTM=0xF6BB` échappe (hi8 0xF6, traité par site 1 puis :8010). | c54x exécuté | EXISTS | **SAS** — fix partiellement rendu natif, résidu inversé au site 2 | site 1 rend le site 2 inatteignable sauf pour `0x?Bx` |
| `FORCE_098` | `calypso_hack.env:19 := (vide)` ⇒ OFF | c54x.c:14431-14443. À `0xde86/0xde94/0xb3e4/0xa5bd` : **`data[0x098a]=data[0x098c]=<hex>`**. `f98=f98v?1:0` donc la chaîne vide est sûre. | c54x exécuté + `HACK=1` | VALEUR (0/vide = OFF) | **BEQUILLE** — injecte le handshake ARM | remplacée par `ARM2DSP_BGEN` (lot 4) — commentaire `hack.env:19` |
| `FORCE_DISPATCH` | absent (OFF) ; **`calypso_wire.env:52 := 1`** | c54x.c:5473-5488, à `pc==0x7234` : force `DP=0x124` **et** `data[0x08E2]=0x0002` **et** `data[0x0584]=0x0002`. | c54x exécuté | VAL>0 | **BEQUILLE** — force 3 valeurs | posée à 1 par `CALYPSO_WIRE=1` ; compagnon déclaré de `FIX_3FCD` |
| `FORCE_DP` | absent (OFF) | c54x.c:5490-5504, à `pc==0x8341` : écrase les 9 bits DP de ST0. | c54x exécuté | VALEUR (vide = OFF) | **BEQUILLE** — force un registre | scopée par `FORCE_DP_FROM` |
| `FORCE_DP_FROM` | absent ⇒ `-1` = global | c54x.c:5496. `-1` : force inconditionnelle ; sinon force seulement si `DP==FROM`. | c54x exécuté | VALEUR | **CONFIG** — sous-paramètre de portée | **inerte seule** : sans `FORCE_DP` le bloc ne s'exécute pas |
| `FORCE_GOLIVE` | `calypso_hack.env:20 := (vide)` ⇒ OFF | c54x.c:14288-14290, à `0xa4d4` : **`data[0x3f70] \|= 0x0002`** = relâche la wait-loop go-live. | c54x exécuté + `HACK=1` | VAL>0 | **BEQUILLE** — force un flag | rendue superflue par `ARM2DSP_BGEN` (commentaire hack.env) |
| `GOLIVE_TASKW` | absent (OFF) | c54x.c:14454-14466, plage `0xa4ca..0xa575`, **conditionné à `data[0x0810] & 0x8000`** : `data[0x3f92] \|= 0x0800` (rejoue l'ORM `0xa539`). | c54x exécuté | EQ1 (seul `1` active) | **BEQUILLE** — rejoue une instruction skippée | **dépend de `ARM2DSP_CTRLSYS`** (lot 4) qui pose `0x0810` bit15 ; sans lui, inerte |
| `INITTAB` | absent (OFF) | c54x.c:12269-12275 : `sp=0x5AC8`, push `0x7120`, `pc=0xc704`. | c54x exécuté | EXISTS | **BEQUILLE** — détourne le reset | **imbriquée dans `if (redir_legacy && pc==0xFF80 && sp==0x1100)` (:12249) → sans `REDIR_LEGACY`, INITTAB n'a AUCUN effet.** Non signalé au recensement |
| `INSTALL_TRACE_OFF` | absent ⇒ **ON** | c54x.c:13748-13761. 6 PC du bloc `0xc7xx`, dump `d[4c5a/4c5c/4c5d/3f5e]`, cap 30. | c54x exécuté | EXISTS-INV | **MESURE** | — |
| `ISR_TO_8341` | absent (OFF) ; **`calypso_wire.env:40 unset`** | c54x.c:14305-14317. À `0x013b` et `g_prev_pc==0x7234` : **`s->pc=0x8341; return 0`** → saute le prologue overlay. | c54x exécuté | EXISTS | **BEQUILLE** — reroute un flux | `wire.env` fait `unset` explicite (motif : `:=` sous `set -a` exporterait une chaîne vide, non-NULL ⇒ gate EXISTS ON) |
| `LDK8_SHIFT16` | absent ⇒ ISA correcte | c54x.c:8800-8804 : `_ldv = shift ? (v<<16) : v` pour `LD #k8u` (`0xE8/0xE9`). Activée, **restaure le bug** qui mettait AR7=0x4387 (slot idle) au terminal `0xb40f` = le storm. | c54x exécuté | EXISTS | **CONFIG** — bascule A/B d'une sémantique ISA. (Définition primaire retenue sur l'heuristique : rien n'est forcé/injecté/sauté ; les deux branches sont implémentées. Candidate au retrait néanmoins : aucun équivalent HW.) | — |
| `MASKROM_GOLIVE` | absent (OFF) | c54x.c:13824-13833, à `0xb40f`, si `data[0x5ac8]==0 && data[0x43c0]!=0` : **`data[0x5ac8] = data[0x43c0]`**. Commentaire :13820 : rendue inutile par le fix ISA `LD #k8u`. | c54x exécuté | EXISTS | **BEQUILLE** — injecte le vecteur de lancement | ⚠️ commentaire :13613 annonce « `CALYPSO_MASKROM_GOLIVE_OFF=1` » : **n'existe pas** |
| `MASKROM_INIT` | absent (OFF) | c54x.c:12287-12298 : au cold-reset, `sp=0x5AC8`, push `0x7120`, `pc=0xc704`. | c54x exécuté | EXISTS | **BEQUILLE** — modélise un mask-ROM absent | **exclusive de `REDIR_LEGACY`** (`if (!redir_legacy && ...)` :12287). ⚠️ commentaires :12285 et :12304 annoncent « `CALYPSO_MASKROM_INIT_OFF=1` » : **n'existe pas** |
| `MVPD_BOOT_LIMIT` | absent ⇒ `500000` | c54x.c:700-701. Seuil `insn` du dump d'occupation MVPD. Dump lui-même gaté par `mvpd_trace_enabled` = jeton `CALYPSO_DEBUG=MVPD` (:698-699). | c54x exécuté | VALEUR | **MESURE** — paramètre de sonde | **inerte sans le jeton `CALYPSO_DEBUG=MVPD`** — non signalé au recensement |
| `PHASE_SM_OFF` | absent ⇒ **ON** | c54x.c:13784-13797. 7 PC de la SM `0xdde0-0xdea8` + `d[3f70]/d[098a..d]/d[0fff]`, cap 40. | c54x exécuté | EXISTS-INV | **MESURE** | — |
| `POKE_A4C7_ONCE` | `calypso_hack.env:22 := (vide)` ⇒ OFF | c54x.c:14357-14369. Premier `0xa4ca` : **`s->pc=0xa4c7; return 0`** (une seule fois). | c54x exécuté + `HACK=1` | VAL>0 | **BEQUILLE** — détourne le PC (le commentaire `hack.env:22` le dit lui-même : « falsification, pas un fix ») | — |
| `REDIR7000` | absent (OFF) | c54x.c:12256-12257 puis :12275 : `sp=0x5AC8; pc=0x7000` au lieu de `0x7120`. | c54x exécuté | EXISTS | **BEQUILLE** — détourne le reset | **imbriquée dans `redir_legacy` (:12249) et écrasée par `INITTAB` (`else if`) → priorité INITTAB > REDIR7000 > 0x7120** |
| `REDIR_LEGACY` | absent (OFF) | c54x.c:12247-12277. Maître : si ON, intercepte `pc==0xFF80 && sp==0x1100` et redirige (0xc704 / 0x7000 / 0x7120). | c54x exécuté | EXISTS | **BEQUILLE** — modélise un mask-ROM absent (le commentaire :12238 l'assume : « ce bloc MODÉLISE un mask-ROM TI absent du dump = un HACK ») | **repose `INITTAB` et `REDIR7000` ; exclut `MASKROM_INIT`** |
| `REPOPULATE` | absent (OFF) | c54x.c:12323-12332. À `pc==0x886a && data[0x4c5c]==0` : **`s->pc=0xc704`**. | c54x exécuté | EXISTS | **BEQUILLE** — détourne le PC | ⚠️ commentaire :12304 annonce « `CALYPSO_MASKROM_INIT_OFF` » : **n'existe pas** |
| `SM_TRACE` | absent (OFF) | c54x.c:14472-14482. Plage `0xdde0..0xde9f`, PC/op/A/TC + `0x098a..0x098e`, cap 400. | c54x exécuté | EXISTS | **MESURE** | doublon fonctionnel de `PHASE_SM_OFF` (même SM, l'un opt-in, l'autre défaut ON) |
| `TASKGO` | absent (OFF) | c54x.c:15564-15592. Front `d_task_md → 5`, 250 pas, arrêt si `pc∈[0x7700,0x79f0]`. | c54x exécuté | EXISTS | **MESURE** | — |
| `TERM_TRACE_OFF` | absent ⇒ **ON** | c54x.c:13697-13709. Plage `0xb400..0xb414`, A + AR0-7, cap 60. | c54x exécuté | EXISTS-INV | **MESURE** | — |
| `TEST_3FCD` | absent (OFF) | c54x.c:13631-13641. À `0x0154`, si `data[0x3fcd]==0 && data[0x3fce]!=0` : **`data[0x3fcd] = data[0x3fce]`**. Bloc parent (`0x0100..0x0160`, :13618) non gaté. | c54x exécuté | EXISTS | **BEQUILLE** — force une valeur | concurrent de `FIX_3FCD` sur la même cellule, à un PC différent |
| `TRACEFROM` | absent (OFF) | c54x.c:15222-15252. Opdump à l'adresse donnée + 3 fenêtres de 4000 insns, log des sauts. | c54x exécuté | VALEUR-comme-gate (`e && *e`, vide = OFF) | **MESURE** | repose `TRACEFROM_N` |
| `TRACEFROM_N` | absent ⇒ `24` | c54x.c:15229-15230. Longueur de l'opdump. | c54x exécuté | VALEUR | **MESURE** — paramètre | **inerte sans `TRACEFROM`** |
| `TRACE_LDU_PC` | absent ⇒ `0xfa7e` | c54x.c:9594-9610. Choisit le PC où émettre `C54_DBG("LDU-PTR", …)`. | c54x exécuté | VALEUR | **MESURE** — paramètre de sonde | **inerte sans le jeton `CALYPSO_DEBUG=LDU-PTR`** — non signalé au recensement |
| `TRACE_STLD_PC` | absent ⇒ `0xa0e7` | c54x.c:11085-11099. Idem pour `C54_DBG("STLD-SP", …)`. | c54x exécuté | VALEUR | **MESURE** — paramètre de sonde | **inerte sans le jeton `CALYPSO_DEBUG=STLD-SP`** |
| `TRAP_CHECKPOINT` | absent ⇒ `4200000` | c54x.c:16097-16116. `checkpoint` est bien lu — mais `trap_armed = 0;` est écrit **en dur** à :16107, avant toute lecture, et n'est jamais réassigné. `if (trap_armed && …)` (:16111) est donc **toujours faux**. | — | VALEUR | **MORT** — la valeur ne peut atteindre aucun effet (code inatteignable) | `run.sh:779` l'exporte encore depuis le menu `TRAP_CP` |

## Le sas `CALYPSO_FIXES` — correction du recensement

Le recensement annonce 7 sous-clés + un `FIX_SUB16_SRC` « non documenté ». **Il y en a 11**, toutes vérifiées par `grep -n 'calypso_fix_enabled("'` :

| Sous-clé | Site | Opcode / masque |
|---|---|---|
| `FIX_LD_XMEM_SHFT` | c54x.c:5310 | `0x9400/0xFE00` |
| `FIX_BIT_XMEM` | c54x.c:5321 | `0x9600/0xFF00` |
| `FIX_SUB_XMEM_YMEM` | c54x.c:5330 | `0xA200/0xFE00` |
| `FIX_LD_PARALLEL` | c54x.c:5350 | `0xA800/0xAC00/0xAE00` |
| **`FIX_LDM_ZEROEXT`** | c54x.c:5364 | `0x4800/0xFE00` — **absente du recensement** |
| **`FIX_DST_LMEM2`** | c54x.c:5374 | `0x4E00/0xFE00` — **absente du recensement** |
| **`FIX_STL_STH_SHFT`** | c54x.c:5398 | `0x9800/0x9A00` — **absente du recensement** |
| `FIX_SUB16_SRC` | c54x.c:5413 | `0x4000/0xFC00` |
| `FIX_STL_B_ASM` | c54x.c:5424 | `0x8500/0xFF00` |
| `FIX_ST_TRN` | c54x.c:5434 | `0x8D00/0xFF00` |
| `FIX_BRINT0_UNMASK` | c54x.c:16925 | **hors** de la garde `if (getenv("CALYPSO_FIXES"))` de :5307 — fonctionne donc seul, et porte **déjà** un marqueur `@BEQUILLE` (:16915-16924) qui le qualifie de « DIAGNOSTIC, à retirer, JAMAIS à confirmer » |

Piège d'idiome du sas : la garde externe :5307 est `EXISTS`, mais `calypso_fix_enabled()` retourne `false` si la chaîne est vide (:5066). `CALYPSO_FIXES=` (vide, cas typique sous `set -a`) est donc **inoffensif** — c'est la seule variable `EXISTS` du lot immunisée contre ce piège.

## Effets de bord / dépendances non listés au recensement

1. **`INITTAB` et `REDIR7000` sont morts sans `REDIR_LEGACY`** — imbriqués dans `if (redir_legacy && pc==0xFF80 …)` (c54x.c:12249). Priorité interne : `INITTAB` > `REDIR7000` > `0x7120` (`else if` :12275).
2. **`MASKROM_INIT` est mutuellement exclusif avec `REDIR_LEGACY`** (`if (!redir_legacy …)`, :12287).
3. **`GOLIVE_TASKW` est conditionné à `data[0x0810] bit15`**, posé uniquement par `ARM2DSP_CTRLSYS` (lot 4). Sous `NATIVE`/`NATIVE_HELPED` où `ARM2DSP_CTRLSYS=0`, il est inerte même à `1`.
4. **`MVPD_BOOT_LIMIT`, `TRACE_LDU_PC`, `TRACE_STLD_PC` sont inertes sans leur jeton `CALYPSO_DEBUG`** (`MVPD`, `LDU-PTR`, `STLD-SP`). Le recensement ne les avait pas marqués `[gate DEBUG=…]`.
5. **`CALYPSO_WIRE=1` allume deux béquilles de ce lot** : `FIX_3FCD=1` (wire.env:50) et `FORCE_DISPATCH=1` (wire.env:52), et fait `unset ISR_TO_8341` (wire.env:40) précisément parce que `:=` sous `set -a` allumerait le gate `EXISTS`.
6. **`CALYPSO_FIX_MVDM` (calypso.env:183) est MORT** — mesuré exporté dans le process vivant (`CALYPSO_FIX_MVDM=1`) alors que seul `CALYPSO_FIX_MVDM_OFF` est lu.
7. **Quatre commentaires nomment des variables inexistantes** : `CALYPSO_MASKROM_INIT_OFF` (:12285, :12304), `CALYPSO_D247_OFF` (:12311), `CALYPSO_MASKROM_GOLIVE_OFF` (:13613), `CALYPSO_FIX_DPAGE` (:13647). Aucun `getenv` correspondant dans tout `osmo-qemu-calypso`.
8. **`FIX_SFTL_RSBX` site 1 lit la variable sans jamais l'utiliser** (c54x.c:6931-6932 puis condition :6938-6939 inconditionnelle) : le commentaire « default OFF: behavior unchanged unless enabled » (:6928) est **périmé** au site 1 et **inversé** au site 2 (:7221, où l'absence de variable conserve le bug pour `0xF4Bx`).
9. **`FIX_DPAGE_OFF` est la seule béquille du lot active dans le run mesuré** (variable absente ⇒ miroir `0x08E2→0x08d4` appliqué à chaque passage `0xa51c`/`0xc8ea`).

Bilan : **17 BEQUILLE · 21 MESURE · 4 SAS · 2 CONFIG · 1 MORT** (+ 5 doublons de sites comptés une fois).

---

# Blocs `@BEQUILLE` prêts à coller

```
D247 / hw/arm/calypso/calypso_c54x.c:12312 (bloc "if (s->pc == 0xb3e4)")
  masque  : l'absence du bootstrap mask-ROM TI qui, sur silicium, appelle le
            sous-systeme operationnel 0xd247 (install table handlers 0xc704 +
            slots TDMA 0xc867 + vecteurs) ; en QEMU 0xd247 n'a d'appelant natif
            qu'a PROM0 0x7102, bloc jamais atteint au boot froid.
  retirer  : des que le bloc appelant natif 0x70ce-0x7106 est atteint (sonde
            D247-TRACE site 0x7102 non nulle), OU des que la table d[4c5c] est
            peuplee par le chemin firmware.
```
```
DEMOD_NOCLOBBER / hw/arm/calypso/calypso_c54x.c:3033 (bloc data_write, PC 0x9fb8/0x9fe2)
  masque  : l'etage demod emule ecrit des paires constantes dans le buffer
            d'entree correlateur [0x2a00,0x2b28) et ecrase la FCCH reelle
            deposee par feed_iq ; la vraie branche = un demod qui consomme
            l'I/Q RX au lieu de produire des constantes.
  retirer  : des que l'etage demod 0x9f95-0x9fe2 lit une source I/Q reelle, ou
            des que FB_IQ_OWNS=1 rend feed_iq seul proprietaire de 0x2a00.
```
```
FIX_3FCD / hw/arm/calypso/calypso_c54x.c:2616 (bloc "if (addr == 0x3fcd && s->pc == 0x013b)")
  masque  : le prologue ISR overlay 0x013b depile une adresse de retour HW
            (0x72d5) au lieu de 0xa4e4 ; la branche reelle = un vectoring
            d'interruption qui empile la bonne adresse de retour.
  retirer  : des que le frame-IT vectorise vers 0xa4e4 sans substitution
            (verifiable : data[0x3fcd] vaut 0xa4e4 sans le gate).
  ATTENTION : posee a 1 par calypso_wire.env:50 sous CALYPSO_WIRE=1.
```
```
FIX_DPAGE_OFF / hw/arm/calypso/calypso_c54x.c:13654 (bloc "if (exec_pc == 0xa51c || exec_pc == 0xc8ea)")
  masque  : le desaccord d'adresse d_dsp_page — la ROM lit 0x08d4, l'ARM/shunt
            postent la tache a 0x08E2 (+0x0E). La branche reelle = poster la
            tache a l'adresse que la ROM lit reellement.
  retirer  : des que le producteur (calypso_dsp_shunt.c / calypso_arm2dsp.c)
            ecrit d_dsp_page a 0x08d4, ou des que l'offset +0x0E est corrige
            a la source.
  ATTENTION : idiome EXISTS-INV — cette bequille est ACTIVE PAR DEFAUT
            (mesuree active dans le run du 2026-07-28). Seul CALYPSO_FIX_DPAGE_OFF
            defini (meme a 0) la coupe.
```
```
FORCE_098 / hw/arm/calypso/calypso_c54x.c:14431 (bloc FORCE-098)
  masque  : l'ARM ne pose jamais les cellules de handshake d_background
            0x098a/0x098c que la phase-SM 0xddeb/0xde86 relit.
  retirer  : des que CALYPSO_ARM2DSP_BGEN pose ces cellules par le pont ARM
            (causalite correcte) — le commentaire calypso_hack.env:19 declare
            deja le remplacement effectue.
```
```
FORCE_DISPATCH / hw/arm/calypso/calypso_c54x.c:5473 (bloc "pc == 0x7234")
  masque  : le scheduler frame 0x7234 est atteint avec DP garbage et
            d_dsp_page a 0, donc la LUT 0x8341 ne resout pas et la tache GSM/FB
            n'est jamais dispatchee.
  retirer  : des que le prologue 0x013b restaure un DP valide et que le
            producteur de d_dsp_page ecrit B_GSM_TASK (bit1) par le chemin ARM.
  ATTENTION : posee a 1 par calypso_wire.env:52 sous CALYPSO_WIRE=1.
```
```
FORCE_DP / hw/arm/calypso/calypso_c54x.c:5490 (bloc "pc == 0x8341", avec FORCE_DP_FROM comme scope)
  masque  : le champ DP de ST0 a l'entree du dispatcher est un residu de pile
            (over-pop / ST0 non restaure) et non la page de donnees attendue.
  retirer  : des que la sonde DISP-ENTRY montre lut[..]=0xff72 (dispatcher OK)
            sans forcage, c.-a-d. quand l'equilibre de pile ST0 push/pop est sain.
```
```
FORCE_GOLIVE / hw/arm/calypso/calypso_c54x.c:14282 (bloc "exec_pc == 0xa4d4")
  masque  : la wait-loop go-live teste data[0x3f70] bit1, pose seulement par le
            setter 0xde9c, lui-meme conditionne aux cellules 0x098a/0x098c que
            l'ARM laisse a 0.
  retirer  : des que le handshake ARM (ARM2DSP_BGEN) fait franchir 0xddf5 et que
            le setter natif 0xde9c s'execute.
```
```
GOLIVE_TASKW / hw/arm/calypso/calypso_c54x.c:14454 (bloc plage 0xa4ca..0xa575)
  masque  : le setter natif ORM #0x0800 @0xa539 est skippe (d[5a00]==0x88), donc
            le bit tache-FB de d[0x3f92] reste 0 et le scheduler ne dispatche
            jamais le correlateur.
  retirer  : des que 0xa539 est reellement execute (le predicat d[5a00] tient la
            bonne valeur), ce qui rend le rejeu redondant.
  NOTE     : inerte sans ARM2DSP_CTRLSYS (exige data[0x0810] bit15).
```
```
INITTAB / hw/arm/calypso/calypso_c54x.c:12269 (bloc imbrique dans redir_legacy :12249)
  masque  : l'absence du mask-ROM TI qui peuple la table de handlers de tache
            0x4c24-0x4c5d au reset ; sans elle 0x7120 fait BACC d[0x4c5b]=null.
  retirer  : des que la table est peuplee par un chemin firmware (0xc704 atteint
            nativement apres le clear 0x8869) — sonde INSTALL-TRACE d[4c5c]!=0.
  NOTE     : sans CALYPSO_REDIR_LEGACY, ce gate n'est jamais evalue.
```
```
ISR_TO_8341 / hw/arm/calypso/calypso_c54x.c:14305 (bloc "exec_pc == 0x013b")
  masque  : le prologue ISR overlay 0x013b deraille et n'atteint jamais la LUT
            FB 0x8341 (setup complet BRC/BK/data-ptr du correlateur).
  retirer  : des que le prologue 0x013b se termine sur 0x8341 par son propre
            flot (meme condition que FIX_3FCD reussi).
  ATTENTION : calypso_wire.env:40 fait un unset EXPLICITE — un ":=vide" sous
            set -a rallumerait ce gate EXISTS. Ne jamais le convertir en ":=".
```
```
MASKROM_GOLIVE / hw/arm/calypso/calypso_c54x.c:13819 (bloc "exec_pc == 0xb40f")
  masque  : le vecteur de lancement mem[0x5ac8] a la base de pile, pre-charge
            par un mask-ROM absent du dump ; a 0, le RET du BACC idle saute a
            PC=0 (storm).
  retirer  : DEJA INUTILE selon le commentaire :13820-13823 — le fix ISA
            LD #k8u (c54x.c:8793-8804) tue le storm nativement. A supprimer
            au prochain passage, ce n'est plus qu'une garde A/B.
```
```
MASKROM_INIT / hw/arm/calypso/calypso_c54x.c:12287 (bloc "if (!redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100)")
  masque  : identique a INITTAB — mask-ROM TI absent qui pose SP=0x5AC8 et
            peuple la table de handlers au cold-reset.
  retirer  : meme condition qu'INITTAB (table peuplee par chemin firmware).
  NOTE     : le commentaire :12285 renvoie a CALYPSO_MASKROM_INIT_OFF, variable
            qui N'EXISTE PAS — le gate reel est opt-in CALYPSO_MASKROM_INIT.
```
```
POKE_A4C7_ONCE / hw/arm/calypso/calypso_c54x.c:14357 (bloc "exec_pc == 0xa4ca")
  masque  : 0xa4c7 (ORM #0x3000,IMR = armement IMR par la ROM) n'est jamais
            atteint : le flot entre a 0xa4ca en sautant l'instruction d'armement.
  retirer  : des que le chemin amont (0xa4cd BC AEQ, ou le setter de d[434e]/
            d[434f]) laisse tomber dans 0xa4c7 — sonde A4CD-BC "fall-through".
  NOTE     : calypso_hack.env:22 le qualifie lui-meme de "falsification, pas un fix".
```
```
REDIR7000 / hw/arm/calypso/calypso_c54x.c:12256 (bloc imbrique dans redir_legacy :12249, applique :12275)
  masque  : l'init des tables BACC-A (d[0x4c5b]/d[0x3fe1]) que le point d'entree
            0x7120 suppose deja faite.
  retirer  : identique a INITTAB.
  NOTE     : ecrase par INITTAB si les deux sont poses (else if).
```
```
REDIR_LEGACY / hw/arm/calypso/calypso_c54x.c:12247 (bloc "pc == 0xFF80 && sp == 0x1100")
  masque  : le reset vector reel 0xff80 -> 0xb410 est detourne pour simuler le
            mask-ROM TI absent (le commentaire :12238 le dit : "ce bloc MODELISE
            un mask-ROM TI absent du dump = un HACK").
  retirer  : des que le vrai reset handler 0xb410 pose SP=0x5AC8 lui-meme
            (STM #0x5AC8,SP correctement decode) et que le boot deroule sans
            over-pop — c'est le bug a tracer, pas a contourner.
  NOTE     : maitre de INITTAB et REDIR7000 ; exclut MASKROM_INIT.
```
```
REPOPULATE / hw/arm/calypso/calypso_c54x.c:12323 (bloc "pc == 0x886a && data[0x4c5c] == 0")
  masque  : le memset RPTB 0x8866-0x886a wipe la table de handlers apres son
            peuplement, sans que le firmware rappelle 0xc704 ; la branche reelle
            = l'ordre firmware clear -> populate.
  retirer  : des que 0xc704 est atteint APRES le clear par le flot natif
            (D247-TRACE : d[4c41]/d[4c46] non nuls en fin de boot).
```
```
TEST_3FCD / hw/arm/calypso/calypso_c54x.c:13631 (bloc "exec_pc == 0x0154")
  masque  : data[0x3fcd] (adresse depilee par le RET @0x0157) n'est jamais
            ecrite ; le firmware installe un handler au voisin data[0x3fce].
  retirer  : des que la table de vecteurs overlay est installee au bon offset
            (data[0x3fcd] non nul sans forcage) — ou immediatement si FIX_3FCD
            (meme cellule, PC 0x013b) est retenu comme mecanisme unique.
```

---

## LOT 4 — Pont ARM↔DSP, parapluies shunt, injections

## LOT 4 — Pont ARM↔DSP, parapluies shunt, injections, canned (47 variables)

Snapshot vérifié dans le conteneur : `calypso_arm2dsp.c` md5 `866e6cb8…`, `calypso_dsp_shunt.c` `a0f17a26…`, `calypso_dsp_helper.c` `ef45b3d5…`, `calypso_c54x.c` `c36466ab…`, `calypso_trx.c` `8fbe30d8…` — identiques au recensement. **Exception : `calypso_native_helped.env` a changé (1811 → 2469 o)** ; il porte désormais lui-même un en-tête `@BEQUILLE` et **13** `:=`, dont `ARM2DSP_BGEN:=1` / `ARM2DSP_CTRLSYS:=0`.

Ordre de sourcing mesuré (`start-clean.sh:5-8`) : `set -a` → `calypso.env` (→ profils) → **puis** `run.sh` (presets `CALYPSO_MODE`). Conséquence : un `: "${VAR:=}"` de `calypso.env` laisse la variable VIDE, et le `:=` de `run.sh` (qui traite vide comme unset) la remplit ensuite. C'est exactement ce qui arrive à `CALYPSO_CANNED`.

---

### Tableau

| VARIABLE | DEFAUT | EFFET (code exécuté) | MODE | IDIOME | CATEGORIE | REPOSE / REPOSEE PAR |
|---|---|---|---|---|---|---|
| `ARM2DSP` | code 0 ; `calypso.env:114 :=0` | `arm2dsp.c:112`. Actif ⇒ `on_dsp_step` (appelé **inconditionnellement depuis `c54x_run`**, `c54x.c:14344`) pose `data[0x0fff] |= 0x0002` + miroir `api_ram`. **Le déclencheur `a2d_pending` vient de `on_arm_write` — fonction JAMAIS APPELÉE (0 caller dans tout `osmo-qemu-calypso`)** ⇒ sans `_CONT`, `ARM2DSP=1` ne poste RIEN. Confirmé par le code lui-même : `dsp_shunt.c:176` « l'ancien hook arm2dsp/trx.c 0x01A8 etait mort ». | tous (mais inerte si `DSP_RUN_C54X=0`) | `VAL>0` | **BEQUILLE** (poste le bit dispatcher que l'écriture ARM devrait propager) — moitié morte | repose rien ; conditionne `_TASKWORD/_TASKBIT/_CONT` |
| `ARM2DSP_BGEN` | code 0 ; `calypso.env:126`, `native:20`, `native_helped:29`, `wire:10` **:=1** | `arm2dsp.c:118` + bloc 221-246. Au PC DSP `0xdddb`, si le bit tâche est armé : écrit `data[0x098a]=data[0x098c]=1` (+`api_ram`). One-shot ; ré-armé par `a2d_bgen_done=0`… **dans `on_arm_write`, donc jamais** (le ré-arm vivant est `L1_RESET_WIRE`, `dsp_shunt.c:180`). **Indépendant de `ARM2DSP`** (`a2d_resolve` le lit hors du gate). | tous profils sauf shunt pur (`DSP_RUN_C54X=0` ⇒ `c54x_run` jamais → inerte) | `VAL>0` | **BEQUILLE** | reposé par NATIVE / NATIVE_HELPED / WIRE / calypso.env |
| `ARM2DSP_BGEN_A` | code `0x098a` | adresse `d_background_enable` écrite par BGEN | idem BGEN | VALEUR (`a2d_env_u16`) | CONFIG | inerte sans `BGEN=1` |
| `ARM2DSP_BGEN_C` | code `0x098c` | adresse `d_background_state` | idem | VALEUR | CONFIG | idem |
| `ARM2DSP_BGEN_VAL` | code `0x0001` | valeur posée dans les 2 cellules | idem | VALEUR | CONFIG | idem |
| `ARM2DSP_BGEN_POLLPC` | code `0xdddb` | PC DSP déclencheur du post | idem | VALEUR | CONFIG | idem |
| `ARM2DSP_BGEN_ONESHOT` | **code 1 si absente** (`arm2dsp.c:125`) | 1 = une seule transition ; 0 = re-post à chaque passage `0xdddb` | idem | `VAL>0` **avec défaut ON** | CONFIG | idem |
| `ARM2DSP_CONT` | unset partout | `arm2dsp.c:251`. Remplace `a2d_pending` par une relecture de `api_ram[0x08E2-0x0800]` bit1 → **seul chemin par lequel `ARM2DSP=1` produit quelque chose**. (HYPOTHÈSE : l'offset `0x08E2` est celui contesté par [[dsp-dpage-offset-bug]], vrai `d_dsp_page`=0x08D4 → à falsifier.) | tous | **`EXISTS`** ⇒ `=0` L'ACTIVE ; seul `unset` coupe | **BEQUILLE** | dépend de `ARM2DSP=1` |
| `ARM2DSP_CTRLSYS` | code 0 ; `wire:11 :=1` ; **`native:21` et `native_helped:30` :=0** | `arm2dsp.c:131` + 197-209. Au PC `0xa537`, force `data[0x0810] |= 0x8000` + `api_ram`. Écriture **directe** dans `s->data[]`, donc invisible de `WATCH_0810` (qui n'instrumente que `data_write`) — piège documenté in-code `c54x.c:2589-2596`. | WIRE uniquement ; explicitement neutralisé en NATIVE/NATIVE_HELPED (mémoire : provoque `B_TASK_ABORT`) | `VAL>0` | **BEQUILLE** | reposé par WIRE ; **dé-posé** (=0) par NATIVE/NATIVE_HELPED |
| `ARM2DSP_CTRLSYS_CELL` | code `0x0810` | cellule `d_ctrl_system` | WIRE | VALEUR | CONFIG | inerte sans CTRLSYS |
| `ARM2DSP_CTRLSYS_VAL` | code `0x8000` | bit asserté (commentaire : « minimal correct value is exactly 0x8000 ») | WIRE | VALEUR | CONFIG | idem |
| `ARM2DSP_CTRLSYS_POLLPC` | code `0xa537` | PC juste avant le `BITF` `0xa53c` | WIRE | VALEUR | CONFIG | idem |
| `ARM2DSP_TASKWORD` | code `0x0fff` | mot task-ready | `ARM2DSP` | VALEUR | CONFIG | aussi lu par BGEN (test `armed`) |
| `ARM2DSP_TASKBIT` | code `0x0002` | bit dispatcher | `ARM2DSP` | VALEUR | CONFIG | idem |
| `CANNED` | code `CAN_DEFAULT = 0` ; `calypso.env:21 :=` (VIDE) ; **`run.sh:1156 :=NONE` en full-grgsm** | `dsp_shunt.c:324-344`. Absente→0, vide/`NONE`→0, `FULL`/`ALL`→`CAN_ALL`, sinon CSV `FBDET,TOA,PM,SNR,ANGLE,CRC`. `g_canned` fabrique `d_fb_det=1`, TOA=23, PM/SNR=0x7000, ANGLE=0, CRC pass. **Résultat effectif dans TOUS les profils livrés = 0 (rien canné).** | tous shunt | LISTE (vide = OFF, contrairement aux gates `EXISTS`) | **BEQUILLE** (interrupteur de fabrication de sorties DSP) | vide de `calypso.env` **écrasé** par `run.sh` (`:=` traite vide comme unset) |
| `ERRREAD` | unset | `trx.c:211`. Cap 40 : sur lecture ARM `0x01A8..0x01AE`, imprime vue_ARM (`dsp_ram`) vs vue_DSP (`dsp->data`) et signale la divergence. | tous | **`EXISTS`** | MESURE | — |
| `ERRWATCH` | unset | `c54x.c:2983`. Cap 60 : écritures **non nulles** de `d_error_status` (0x08D5) avec décodage de bits (`STACK_OV`, `DMA_*`, `RHEA`…). Les écritures de 0 sont volontairement ignorées. | tous (DSP tournant) | **`EXISTS`** | MESURE | — |
| `FIND32` | unset | `trx.c:195`. Cap 40 : à chaque lecture DSP paire, si `dsp_ram[off/2] == FIND32_VAL`, imprime offset, mot DSP, position relative à NDB. | tous | **`EXISTS`** | MESURE | repose sur `FIND32_VAL` |
| `FIND32_VAL` | code `0x0020` | valeur cherchée | idem | VALEUR | MESURE | inerte sans `FIND32` |
| `INJECT_ACD` | unset ; `shunt_no_legit:13 :=1` | `dsp_helper.c:345` gate `shunt_dispatch_allc()` : écrit `a_cd[0..2]`=status CRC pass + `a_cd[3..14]`=23 o L2 (SI ou IMM-ASSIGN ou SDCCH), + `d_task_d`/`d_burst_d`/`a_serv_demod` sur 1 ou 2 pages. | SHUNT_NO_LEGIT explicitement, SHUNT_LEGIT par fallback | `EQ1` **OU `SHUNT_LEGIT=1`** | **BEQUILLE** | reposée par `SHUNT_LEGIT` |
| `INJECT_AGCH` | unset ; `shunt_no_legit:14 :=1` | `dsp_shunt.c:942` gate `feed_agch()` : range l'IMM ASSIGN GSMTAP dans `agch_buf` (avec priorité IMM-ASSIGN > PAGING). | idem | `EQ1` OU `SHUNT_LEGIT` | **BEQUILLE** | idem |
| `INJECT_FB` | unset **partout** | `dsp_helper.c:233`. **SEULE injection sans fallback `SHUNT_LEGIT`.** ⇒ `shunt_dispatch_fb()` retourne immédiatement dans TOUS les profils livrés — y compris `SHUNT_NO_LEGIT` qui pose pourtant `SHUNT_REAL_FB=1`. **Le bloc REAL_FB de `dsp_helper.c:235-250` est donc du code MORT** ; le FB arrive en réalité par (a) l'écriture directe `api_ram[0x08F8..0x08FD]` de `dsp_shunt.c:663-671` (gate `SHUNT_LEGIT|NO_LEGIT`) et (b) `real_fb_read` → `trx.c:297`. | aucun (jamais activée) | `EQ1` strict | **BEQUILLE** (inactive) | **ne bénéficie PAS du fallback LEGIT** — asymétrie non documentée dans les .env |
| `INJECT_SACCH` | unset ; `shunt_no_legit:16 :=1` | `dsp_shunt.c:1114` gate `feed_sacch()` : SI5/SI6 réels grgsm → `sacch_buf`, header L1 zéroté, `sacch_real=true` (stoppe la fabrication SI3→SI6). | idem | `EQ1` OU `SHUNT_LEGIT` | **BEQUILLE** | idem |
| `INJECT_SB` | unset ; `shunt_no_legit:12 :=1` | `dsp_helper.c:282` gate `shunt_dispatch_sb()` : écrit le SB encodé (BSIC/FN grgsm) au format read-page natif. | idem | `EQ1` OU `SHUNT_LEGIT` | **BEQUILLE** | idem ; interagit avec `SHUNT_NO_CANNED` (bail si `!sb_valid`) |
| `INJECT_SDCCH` | unset ; `shunt_no_legit:15 :=1` | `dsp_shunt.c:1070` gate `feed_sdcch()` : bloc L2 SDCCH DL → `sdcch_buf` (ring ou latch selon `SHUNT_SDCCH_RING`). | idem | `EQ1` OU `SHUNT_LEGIT` | **BEQUILLE** | idem |
| `L1` | unset | `layer1.c:55`, test `e[0]=='c'`. Actif ⇒ `calypso_l1_c_active()` vrai en 4 points : `dsp_shunt.c:1857` (arme le shunt même sans `DSP_SHUNT`), `dsp_shunt.c:2056` (`substitutes()` = le shunt REMPLACE le DSP), `trx.c:824` (intercepte `d_task_md`), `trx.c:1475`. C'est un modèle L1 **HLE en C** qui se substitue au DSP. | aucun profil livré | CHAINE (`c*`) | **BEQUILLE** | force `substitutes()` ⇒ gate les `c54x_run` natifs |
| `L1S_FN_ADDR` | code : `nm(l1s)` sinon `0x836508` ; `run.sh:1805` exporte la valeur `nm` du ELF | `dsp_helper.c:124`, `shunt_l1s_fn()` : `dma_memory_read` de `l1s.current_time.fn`. Garde AS-NULL. | shunt | VALEUR/adresse | CONFIG | — |
| `L1_RESET_WIRE` | **défaut ON** (code) | `dsp_shunt.c:178-187`. Sur écriture ARM `d_dsp_page == 0` (= `l1s_reset_hw`), appelle `calypso_dsp_shunt_l1_reset()` → clear des latches IMM-ASSIGN/SDCCH, le gate SI se rouvre. **C'est le chemin VIVANT** ; l'équivalent `arm2dsp.c:176-179` est mort. | shunt (latch task) | `ON-sauf-0` | **BEQUILLE** (n'existe que pour nettoyer des latches fabriqués) | appelle `l1_reset()` partagé avec le hook mort d'`arm2dsp` |
| `LAST_RACH_FN_ADDR` | code : `nm(last_rach)` sinon `0x836500` ; `run.sh:1806` | `dsp_helper.c:147`, `shunt_last_rach_fn()` : lit le FN exact mémorisé par le firmware pour la dernière RACH (match req-ref IMM ASSIGN). | shunt | VALEUR/adresse | CONFIG | consommée par `REQREF_LAST_RACH` (lot 6) |
| `SCAN43D8` | unset | `c54x.c:15461`. One-shot à `exec_pc==0xb01c` : balaye les 4 banks `0x7000-0xfffe` à la recherche du mot `0x43d8`, classe `ST #imm`/`LD`, dumpe `0xbaf8..0xbb10`. Restaure `s->xpc`. | tous (DSP tournant) | **`EXISTS`** | MESURE | — |
| `SCANDATA` | unset | `c54x.c:15319`. One-shot au 2e passage `0xb01c` : liste les cellules `data[]` dont la valeur tombe dans `[LO..HI]`, annote « DANS LA TABLE DE DISPATCH » (0x4380-0x43ff) / « API RAM ». Cap 60. | idem | **`EXISTS`** | MESURE | repose `SCANDATA_LO/HI` |
| `SCANDATA_HI` | code `0x79f0` | borne haute du scan | idem | VALEUR | MESURE | inerte sans `SCANDATA` |
| `SCANDATA_LO` | code `0x76f8` | borne basse | idem | VALEUR | MESURE | idem |
| `SCAN_08F8` | unset | `c54x.c:15148`. One-shot à `exec_pc==0x9ac0` : cherche le mot `0x08f8` (adresse `d_fb_det`) dans le bank courant → identifie les writers potentiels. Cap 40. **Ne scanne qu'UN bank** (`s->xpc` courant) — contrairement à `SCAN43D8`. | idem | **`EXISTS`** | MESURE | — |
| `SHUNT_CANNED` | unset partout | `dsp_helper.c:652`. Dans `shunt_dispatch_allc`, force `a_serv_demod[PM]=SHUNT_CANNED_PM` et `[SNR]=SHUNT_CANNED_SNR` au lieu de `g_shunt.last_pm`/`rx_snr`, et étiquette le log « CANNED(hack) ». | shunt | **`EXISTS`** ⇒ `=0` L'ACTIVE | **BEQUILLE** | orthogonale à `CANNED` (masque différent) |
| `SHUNT_DL_INJECT` | `hack:27 :=0` ; `run.sh:796` et `run.sh:1785 :=0` ; **`shunt_no_legit:17 :=1`** | `dsp_shunt.c:2027-2032`. Dans `feed_si`, appelle `l1ctl_inject_dl_si(si_buf, 23, trx_fn)` : **court-circuit total** — le SI part directement en `L1CTL_DATA_IND` vers le mobile, sans passer par `a_cd`, ni le DSP, ni le L1 firmware. | SHUNT_NO_LEGIT seulement | `EQ1` | **BEQUILLE** (la plus intrusive du lot) | reposée à 1 par le profil `shunt_no_legit` alors que `run.sh` la met à 0 — le profil gagne (sourcé avant) |
| `SHUNT_DRIVE_DSP` | unset | `dsp_shunt.c:612`. Dans le tick shunt : `if (run_c54x && (_drive || substitutes())) shunt_route_to_c54x_run()`. Sans lui, en mode ASSIST (`DSP=c54x`, shunt actif mais ne substitue pas) le tick TDMA natif exécute déjà le DSP → ce gate est l'anti-double-run. `=1` force le double-run. | ASSIST | `EQ1` | CONFIG (cadence/chemin d'exécution) | dépend de `DSP_RUN_C54X` et de `substitutes()` (donc de `DSP_SHUNT` / `L1`) |
| `SHUNT_DUAL_PAGE` | **défaut ON** | `dsp_helper.c:653`. Écrit les champs read-page (`d_task_d`, `d_burst_d`, `a_serv_demod`) sur **les deux pages** 0 et 1, parce que le `r_page` du mobile bascule indépendamment du `w_page` porté par `d_dsp_page`. | shunt | `ON-sauf-0` | **BEQUILLE** | — |
| `SHUNT_FEED_SI` | unset ; `shunt_no_legit:20 :=1` | `dsp_shunt.c:1963-1970`. Gate d'entrée de `calypso_dsp_shunt_feed_si()` : si OFF, `si_valid=false` et retour immédiat ⇒ `a_cd` ne se remplit que si le démod natif produit vraiment le bloc. Si ON : range le SI par type (`si_set[0..5]`), seed SI6 fabriqué depuis SI3 tant que `!sacch_real`, remplit `si_buf`. | SHUNT_NO_LEGIT / SHUNT_LEGIT | `EQ1` OU `SHUNT_LEGIT` | **BEQUILLE** | conditionne `SHUNT_DL_INJECT` (appelé depuis `feed_si`) |
| `SHUNT_LEGIT` | `calypso.env:12 :=0` ; `native/native_helped :=0` ; `shunt_legit.env :=1` ; `shunt_no_legit :=0` | **Parapluie, 13 sites de code.** `dsp_shunt.c:625-690` (transport `sb_valid`→`d_fb_det=1` + TOA/PM/ANGLE/SNR dans `api_ram[0x08F8..0x08FD]`, boucle AFC, `a_pm` read-page 0x30-0x32/0x44-0x46), `:858` (branche FB), `:1479` (active `real_fb_read`), `:1968` (feed_si), `dsp_helper.c:282/345/698`, `c54x.c:2698` (**force `val=1` sur toute écriture DSP de `data[0x08f8]`**) et `:2711` (force `a_pm`), `trx.c:309` (retourne `d_task_d=24` au lieu de 0), `:324` (retourne `d_burst_d=(cur+3)&3`), `:785` (RACH UL depuis `d_rach`). | SHUNT_LEGIT | `EQ1` **après canonicalisation** par le constructeur `dsp_shunt.c:86-100` | **BEQUILLE** (parapluie) | **`SHUNT_LEGIT=DSP` ⇒ `setenv(DSP_RUN_C54X=1)` ; `=NO_CANNED` ⇒ `setenv(SHUNT_NO_CANNED=1)` — AVANT `main()`** ; repose par fallback `INJECT_SB/ACD/AGCH/SDCCH/SACCH`, `SHUNT_FEED_SI`, `TRF_RXLEV`, `UL_RACH_FROM_DRACH`, `SHUNT_REAL_FB` |
| `SHUNT_NO_CANNED` | `run.sh:1147 :=1` puis **`run.sh:1151 =1` VERROUILLÉ** en `full-grgsm` ; `run.sh:1223 :=0` pour les autres modes | `dsp_helper.c:291` : `dispatch_sb` bail si `!sb_valid` ; `:362` : `dispatch_allc` bail si `!si_valid`. **Supprime** la fabrication au lieu de l'ajouter : si le démod casse, rien ne campe. | full-grgsm (verrouillé) | `EQ1` | CONFIG (mode de fidélité) | **posée par le constructeur** si `SHUNT_LEGIT/NO_LEGIT` contient `NO_CANNED` |
| `SHUNT_NO_FAKE_FB` | `hack.env:42-43 :=1` + `export` DUR (donc effectif seulement si `CALYPSO_HACK=1`) | `dsp_shunt.c:851`. Empêche l'appel à `shunt_dispatch_fb()` dans les deux branches (`legit` et non-legit) du tick `md==FB_DSP_TASK`. **Effet observable NUL aujourd'hui** : `dispatch_fb` est déjà un no-op (`INJECT_FB` unset), et l'écriture `api_ram[0x08F8]` de `dsp_shunt.c:663` n'est PAS gatée par lui. | HACK | `EQ1` | CONFIG (mode de fidélité — redondant) | dépend de `INJECT_FB` pour avoir un sens |
| `SHUNT_NO_FAKE_PM` | `hack.env:45-46 :=0` + `export` DUR | `dsp_shunt.c:845` et `:875`. Empêche `shunt_dispatch_pm()` → plus de `a_pm[0..2]` fabriqué (rxlev retombe au plancher −110 sauf si `TRF_RXLEV`/`SHUNT_LEGIT` réécrivent ailleurs). **Effet réel, lui** : `dispatch_pm` n'est gaté par aucun `INJECT_*`. | HACK | `EQ1` | CONFIG (mode de fidélité) | interagit avec `SHUNT_PM` / `TRF_RXLEV` (lot 6) |
| `SHUNT_NO_GRGSM` | unset | `dsp_shunt.c:1294`, 3 consommateurs : `:1196` (pas de listener GSMTAP/SI :4730), `:1301` (pas de listener SCH :4731), `:1833` (pas de poll SI shm). ⇒ `sb_bsic/sb_fn/sb_toa` et `si_buf` doivent venir du DSP. | tous shunt | `EQ1` | CONFIG (mode de fidélité) | coupe la source de `sb_valid`/`si_valid`, donc neutralise indirectement toutes les injections |
| `SHUNT_NO_LEGIT` | `calypso.env:10 :=0` ; source `calypso_shunt_no_legit.env` si `=1` | 5 sites où il est strictement **interchangeable** avec `SHUNT_LEGIT` (`c54x.c:2698`, `dsp_shunt.c:625`, `:858`, `trx.c:309`, `:324`) — mais **PAS** aux sites `dsp_helper.c:282/345/698`, `dsp_shunt.c:942/1070/1114/1479/1968`, `c54x.c:2711`, `trx.c:785` où seul `SHUNT_LEGIT` sert de fallback. D'où les 15 `:=1` explicites du profil. | SHUNT_NO_LEGIT | `EQ1` après canonicalisation | **BEQUILLE** (parapluie « décomposé ») | même value-list `DSP`/`NO_CANNED` que `SHUNT_LEGIT` ; repose 15 variables via son .env |
| `WATCH_0810` | unset | `c54x.c:2597`. Cap 200 : trace les écritures **opcode DSP** de `data[0x0810]` avec le PC auteur. Ne voit PAS le wire CTRLSYS (écriture directe hors `data_write`) — c'est le test discriminant décrit in-code. | tous (DSP tournant) | **`EXISTS`** | MESURE | complémentaire de `ARM2DSP_CTRLSYS` |
| `WATCH_ACD` | unset | `c54x.c:2535`. Cap 60 : trace les écritures opcode DSP dans `data[0x09D2..0x09E0]` (`a_cd`) → détecte le clobber du SI injecté par le shunt. | idem | **`EXISTS`** | MESURE | — |

---

### Effets de bord à signaler en priorité (mesurés au code, pas déduits)

1. **`calypso_arm2dsp_on_arm_write()` n'a AUCUN appelant** (`grep` sur tout `osmo-qemu-calypso` hors `build/`) — le seul point d'entrée vivant du module est `on_dsp_step` depuis `c54x_run` (`c54x.c:14344`). Conséquences : (a) `ARM2DSP=1` sans `ARM2DSP_CONT` ne poste jamais rien ; (b) le ré-arm go-live sur reset L1 décrit par `arm2dsp.c:172-179` (et par la note mémoire `l1-reset-rearm-dsppage`) **ne s'exécute pas là** — le chemin vivant est `L1_RESET_WIRE` dans `dsp_shunt.c:178-187`, ce que le code lui-même dit à `dsp_shunt.c:176`.
2. **`INJECT_FB` est la seule injection sans fallback `SHUNT_LEGIT`** et n'est posée dans aucun `.env` ⇒ `shunt_dispatch_fb()` est un no-op dans tous les profils livrés, et avec lui le bloc `SHUNT_REAL_FB` de `dsp_helper.c:235-250`. `SHUNT_NO_FAKE_FB` en devient sans effet observable. Le FB passe en réalité par `dsp_shunt.c:663-671` + `real_fb_read`.
3. **Le constructeur `shunt_env_value_list()` (`dsp_shunt.c:86-100`) fait des `setenv()` avant `main()`** : `SHUNT_LEGIT=DSP` crée `DSP_RUN_C54X=1` (lot 2), `=NO_CANNED` crée `SHUNT_NO_CANNED=1`. Ces deux variables apparaissent alors au manifeste sans avoir été tapées.
4. **`SHUNT_NO_LEGIT` n'est PAS un synonyme de `SHUNT_LEGIT`** : 5 sites acceptent les deux, 10 sites n'acceptent que `SHUNT_LEGIT`. Le profil compense par 15 `:=1` explicites ; toute variable ajoutée avec le fallback « legit-only » sera silencieusement OFF en mode NO_LEGIT.
5. **8 gates `EXISTS` dans ce lot** (`ARM2DSP_CONT`, `SHUNT_CANNED`, `ERRREAD`, `ERRWATCH`, `FIND32`, `SCAN43D8`, `SCANDATA`, `SCAN_08F8`, `WATCH_0810`, `WATCH_ACD`) : les mettre à `0` les **active**. Seul `unset` coupe.
6. **`ARM2DSP_BGEN_ONESHOT` a un défaut ON codé en dur** (`arm2dsp.c:125` : absent ⇒ 1), jamais listé actif dans un `.env` (les 5 lignes `BGEN_*` de `calypso.env:127-131` sont **commentées**).
7. `CALYPSO_CANNED` : `calypso.env:21` l'exporte VIDE, puis `run.sh:1156` la remplit à `NONE` (`${VAR:=}` traite le vide comme unset). Résultat identique (0 canné) mais le manifeste affichera `NONE`, pas vide.
8. `run.sh:1151` est le seul `=` **verrouillé** du lot (`SHUNT_NO_CANNED=1` en `full-grgsm`) : non surchargeable par la CLI, contrairement à tous les `:=`.

---

### Blocs `@BEQUILLE` prêts à coller (19)

```
CALYPSO_ARM2DSP / hw/arm/calypso/calypso_arm2dsp.c:112
  masque  : la propagation ARM->DSP du bit dispatcher data[0x0fff] bit1 que
            l'ecriture ARM de d_dsp_page (B_GSM_TASK) devrait produire via
            l'API RAM partagee. Le hook d'entree on_arm_write() n'ayant AUCUN
            appelant, seul ARM2DSP_CONT rend ce poste effectif.
  retirer : quand l'ecriture ARM de d_dsp_page est reellement routee vers le
            module (ou quand le dispatcher ROM 0xb41c lit la cellule que l'ARM
            ecrit deja) -- alors le poste force devient inutile.
```
```
CALYPSO_ARM2DSP_BGEN / hw/arm/calypso/calypso_arm2dsp.c:118  (bloc 221-246)
  masque  : le handshake go-live ou l'ARM pose d_background_enable (0x098a) et
            d_background_state (0x098c). Sans lui la phase-SM 0xdddb->0xddeb
            prend la branche reset, 0xde9c n'est jamais atteint, d[0x3f70] bit1
            reste 0 et la wait-loop 0xa4ca/0xa4d0 spinne indefiniment.
  retirer : quand le firmware ARM emule ecrit lui-meme 0x098a/0x098c dans
            l'API RAM (portage du handshake cote ARM, cf porting-hacks-to-arm).
```
```
CALYPSO_ARM2DSP_CONT / hw/arm/calypso/calypso_arm2dsp.c:251
  masque  : l'absence de re-post par trame. Le dispatcher ROM (0xb419) efface
            le bit tache entre deux passages ; faute d'un chemin ARM vivant,
            CONT relit d_dsp_page en API RAM a chaque pas DSP et repose le bit.
            Gate EXISTS : CALYPSO_ARM2DSP_CONT=0 l'ACTIVE, seul unset le coupe.
  retirer : des que on_arm_write() est appele (a2d_pending redevient le
            declencheur) ; verifier au passage l'offset 0x08E2 (conteste :
            d_dsp_page pourrait etre 0x08D4).
```
```
CALYPSO_ARM2DSP_CTRLSYS / hw/arm/calypso/calypso_arm2dsp.c:131  (bloc 197-209)
  masque  : l'ecriture ARM de d_ctrl_system (data[0x0810] bit15) faite par
            l1s_reset() sur le vrai Calypso, que le pont API emule ne propage
            pas. Sans elle le gate 0xa53c (BITF #0x8000) court-circuite en
            0xa575. NOTE : ecriture directe dans s->data[] -> invisible de
            data_write, donc de CALYPSO_WATCH_0810.
  retirer : quand le firmware ARM emule ecrit 0x0810 via le chemin API normal.
            Attention : les profils NATIVE et NATIVE_HELPED la posent a 0 --
            forcee, elle declenche B_TASK_ABORT et casse le retour FB.
```
```
CALYPSO_CANNED / hw/arm/calypso/calypso_dsp_shunt.c:324  (shunt_parse_canned, 322-344)
  masque  : les sorties du correlateur/demodulateur DSP non produites --
            d_fb_det, TOA, PM, SNR, ANGLE, statut CRC -- remplacees par des
            constantes plausibles (CAN_* dans calypso_dsp_internal.h:189-196).
  retirer : quand le correlateur natif produit ces six valeurs. La variable
            est deja a 0 dans tous les profils livres : la retirer du code ne
            change rien au comportement, seulement au vocabulaire.
```
```
CALYPSO_INJECT_ACD / hw/arm/calypso/calypso_dsp_helper.c:345
  masque  : l'etage NB du DSP qui devrait remplir a_cd[0..14] (statut CRC +
            23 octets L2) apres demodulation d'un bloc CCCH/BCCH.
  retirer : quand le chemin natif correlateur -> NB -> a_cd livre le bloc.
```
```
CALYPSO_INJECT_AGCH / hw/arm/calypso/calypso_dsp_shunt.c:942
  masque  : la reception AGCH par le DSP -- l'IMM ASSIGN est capte hors bande
            (GSMTAP grgsm) puis presente dans a_cd sur un bloc CCCH.
  retirer : quand le decodage AGCH natif alimente a_cd.
```
```
CALYPSO_INJECT_FB / hw/arm/calypso/calypso_dsp_helper.c:233
  masque  : la publication du resultat FB (d_fb_det + a_sync_demod[TOA/PM/
            ANGLE/SNR]) par le correlateur DSP.
  retirer : quand d_fb_det natif est produit. ATTENTION : contrairement aux
            cinq autres INJECT_*, ce gate n'a PAS de fallback SHUNT_LEGIT et
            n'est pose dans aucun .env -> shunt_dispatch_fb() est deja mort
            dans tous les profils, ainsi que le bloc SHUNT_REAL_FB (l.235-250).
```
```
CALYPSO_INJECT_SACCH / hw/arm/calypso/calypso_dsp_shunt.c:1114
  masque  : la demodulation SACCH par le DSP (SI5/SI6 du canal dedie),
            remplacee par les blocs grgsm avec header L1 zerote.
  retirer : quand le chemin natif demodule la SACCH.
```
```
CALYPSO_INJECT_SB / hw/arm/calypso/calypso_dsp_helper.c:282
  masque  : la production du burst SB (BSIC/FN) par le DSP apres detection
            FCCH ; ici l'encodage vient du SCH decode par gr-gsm.
  retirer : quand le correlateur natif enchaine FB -> SB.
```
```
CALYPSO_INJECT_SDCCH / hw/arm/calypso/calypso_dsp_shunt.c:1070
  masque  : la demodulation du SDCCH DL par le DSP (UA/AUTH/L3), remplacee par
            les blocs L2 forwardes par si_bridge.
  retirer : quand le chemin natif demodule le SDCCH.
```
```
CALYPSO_L1 / hw/arm/calypso/calypso_layer1.c:52-60
  masque  : le DSP entier. calypso_l1_c_active() rend substitutes() vrai
            (dsp_shunt.c:2056) -> un modele L1 haut-niveau en C remplace la
            couche 1 et gate les c54x_run natifs (dsp_shunt.c:1857,
            trx.c:824, trx.c:1475).
  retirer : quand le DSP emule tient la couche 1. Variable posee dans aucun
            profil : candidate au retrait pur et simple.
```
```
CALYPSO_L1_RESET_WIRE / hw/arm/calypso/calypso_dsp_shunt.c:178-187
  masque  : la remise a zero d'etat que le DSP reel subit sur l1s_reset_hw().
            Ici il n'y a pas d'etat DSP a reinitialiser, seulement des latches
            FABRIQUES par le shunt (IMM-ASSIGN, SDCCH) qui, non nettoyes,
            bloquent la reouverture du gate SI apres un Ctrl-C mobile.
  retirer : avec les latches eux-memes, c'est-a-dire avec les injections
            INJECT_AGCH / INJECT_SDCCH.
```
```
CALYPSO_SHUNT_CANNED / hw/arm/calypso/calypso_dsp_helper.c:652
  masque  : les mesures a_serv_demod[PM] et [SNR] du bloc courant, forcees a
            SHUNT_CANNED_PM / SHUNT_CANNED_SNR au lieu des valeurs mesurees.
            Gate EXISTS : CALYPSO_SHUNT_CANNED=0 l'ACTIVE.
  retirer : quand PM et SNR proviennent du modele RF (DECAN_PM / DECAN_SNR)
            ou du DSP. Variable posee nulle part -> retrait sans risque.
```
```
CALYPSO_SHUNT_DL_INJECT / hw/arm/calypso/calypso_dsp_shunt.c:2027-2032
  masque  : TOUT le chemin descendant a_cd -> L1 firmware -> UART -> L1CTL.
            Le SI est pousse directement en L1CTL_DATA_IND vers le mobile ;
            aucune partie de la chaine emulee n'est exercee.
  retirer : des que le SI atteint le mobile via a_cd (c'est deja le cas :
            run.sh la met a 0 par defaut ; seul calypso_shunt_no_legit.env la
            repose a 1). C'est la bequille la plus intrusive du lot.
```
```
CALYPSO_SHUNT_DUAL_PAGE / hw/arm/calypso/calypso_dsp_helper.c:653
  masque  : la modelisation du basculement de r_page cote lecture. Comme le
            r_page du mobile bascule independamment du w_page porte par
            d_dsp_page, on ecrit les champs read-page sur LES DEUX pages pour
            que le mobile les trouve quelle que soit sa page courante.
  retirer : quand r_page est modelise fidelement (page de reponse deduite du
            protocole et non devinee).
```
```
CALYPSO_SHUNT_FEED_SI / hw/arm/calypso/calypso_dsp_shunt.c:1963-1970
  masque  : la production des blocs SI par la demodulation native (correlateur
            0x8d00 -> NB -> a_cd). Le SI vient de gr-gsm et est range par type
            dans si_set[0..5], avec fabrication d'un SI6 seed depuis le SI3.
  retirer : quand a_cd se remplit par la demodulation native.
```
```
CALYPSO_SHUNT_LEGIT / hw/arm/calypso/calypso_dsp_shunt.c:86-100 (constructeur)
                    + hw/arm/calypso/calypso_c54x.c:2696-2724
  masque  : le mur RANK3 -- le correlateur natif n'ecrit jamais d_fb_det. Le
            parapluie transporte la detection reelle de gr-gsm vers le resultat
            DSP (api_ram[0x08F8..0x08FD]), FORCE a 1 toute ecriture DSP de
            data[0x08f8] (c54x.c:2698), force a_pm (c54x.c:2711 et
            dsp_shunt.c:684), et falsifie deux retours de lecture cote ARM :
            d_task_d=24 au lieu de 0 (trx.c:309) et d_burst_d=(cur+3)&3
            (trx.c:324). Il sert AUSSI de fallback implicite a 10 autres gates.
  retirer : quand le correlateur natif pose d_fb_det (RANK3 leve). ATTENTION :
            la value-list du constructeur fait setenv(DSP_RUN_C54X=1) sur
            "=DSP" et setenv(SHUNT_NO_CANNED=1) sur "=NO_CANNED", AVANT main()
            -- ces deux variables apparaissent au manifeste sans etre tapees.
```
```
CALYPSO_SHUNT_NO_LEGIT / hw/arm/calypso/calypso_dsp_shunt.c:86-100 (constructeur)
                       + hw/arm/calypso/calypso_c54x.c:2698
  masque  : la meme chose que SHUNT_LEGIT, mais decomposee : le profil
            calypso_shunt_no_legit.env repose a la main les 15 variables que le
            parapluie impliquait. Le gate lui-meme n'est reconnu qu'a 5 sites
            (c54x.c:2698, dsp_shunt.c:625, dsp_shunt.c:858, trx.c:309,
            trx.c:324) -- les 10 autres n'acceptent que SHUNT_LEGIT.
  retirer : avec SHUNT_LEGIT. Tant qu'il vit, toute nouvelle bequille ajoutee
            avec le seul fallback "legit" sera silencieusement OFF ici.
```

---

## LOT 5 — FB / FBSB / corrélateur / démod / résultat

## LOT 5 — FB / FBSB / corrélateur / démod / résultat (52 variables)

Snapshot vérifié : md5 des 5 fichiers pivots **identiques** au recensement (`calypso_c54x.c` = c36466ab…, `calypso_dsp_shunt.c` = a0f17a26…). Les lignes ci-dessous sont donc directement citables.

| VARIABLE | DEFAUT (code / env) | EFFET RÉEL (code exécuté) | MODE | IDIOME → comment on la coupe | CATÉGORIE | REPOSE / REPOSÉE PAR |
|---|---|---|---|---|---|---|
| `CALA_71DA` | code OFF ; aucun .env | c54x.c:14379 — à `exec_pc==0x71da`, logge la cible du CALA générique (`A&0xFFFF`), dédup + cap souple. Aucune écriture. | tous | `EXISTS` → `unset` seul | MESURE | — |
| `CALA_FB` | code OFF | c54x.c:6235 — dans le handler BACC/CALA (`op 0xF4E2..0xF5E3`), au site `src_pc==0xb01e`, logge la cible NATIVE + `d_task_md` (0x0804/0x0818/0x058a) **avant** tout reroute FB_ENERGY. Cap 40. | tous (utile surtout NATIVE*) | `EXISTS` | MESURE | oracle de contrôle de `FB_ENERGY` |
| `CORROUT` | code OFF | c54x.c:15265 — détecte la sortie de la fenêtre noyau MAC `[0xa070..0xa0a0]`, dump A/B/T/AR3-6 + `wz[0x2c00..0x2c0f]`. Cap 40. | tous | `EXISTS` | MESURE | — |
| `CORR_AR1` | code `0x2f22` | c54x.c:14331 — valeur injectée dans AR1 **uniquement si `CORR_SETUP`** | tous | VALEUR (non-vide) | BEQUILLE (paramètre) | inerte sans `CORR_SETUP` |
| `CORR_AR4` | code `0x2be4` | c54x.c:14332 — idem AR4 | tous | VALEUR | BEQUILLE (paramètre) | idem |
| `CORR_AR5` | code `0x0060` | c54x.c:14333 — idem AR5 | tous | VALEUR | BEQUILLE (paramètre) | idem |
| `CORR_BANK` | code `-1` (off) | c54x.c:5113 — si `0..3`, **écrase `s->xpc`** à chaque instruction dont `pc∈[0x8d00..0xa200]` : force la banque d'overlay du handler FB. | tous | `VALEUR` : absent/vide = off. **`CORR_BANK=0` N'ÉTEINT PAS — elle force XPC=0** | BEQUILLE | — |
| `CORR_FLOW` | code OFF | c54x.c:5137 — trace PC/op/TC/C/AR1-5 pour `xpc==0`, `pc∈[0x8600..0xa200]`, filtrée sur `d_task_md∈{5,6}` ou `pc≥0xa000`, dédup PC, saute `0x8866-0x886c`. **Cap réel 20000** (le commentaire dit 8000 → périmé). | tous | `EXISTS` | MESURE | — |
| `CORR_HI` | code `0x8590` | c54x.c:5195 — borne haute de CORR-TRACE | tous | VALEUR | MESURE (paramètre) | **inerte sans jeton `CALYPSO_DEBUG=CORR-TRACE` ET `insn_count>60e6`** |
| `CORR_LO` | code `0x8560` | c54x.c:5195 — borne basse | tous | VALEUR | MESURE (paramètre) | idem |
| `CORR_SETUP` | code OFF ; `wire.env:41` **`unset`** | c54x.c:14329 — à `exec_pc==0x8d00`, **écrit AR1/AR4/AR5** avec les constantes que la LUT native 0x8341 aurait posées. | tous (mesuré inefficace : « AR réécrits avant 0x8e8b ») | `EXISTS` → seul `unset` coupe (d'où le `unset` explicite dans wire.env) | BEQUILLE | repose `CORR_AR1/AR4/AR5` |
| `DEMODIO` | code OFF | c54x.c:2934 (`dio_init`) ; `dio_note` appelé depuis `data_read` (1744) et `data_write_locked` (2964) → logge R/W avec A/B/T/AR2-5 dans la fenêtre PC, après N insn, cap 160. | tous | `VAL>0` → `=0` coupe | MESURE | repose AFTER/PCLO/PCHI |
| `DEMODIO_AFTER` | code `40 000 000` insn | c54x.c:2936 — seuil d'armement | tous | VALEUR | MESURE (paramètre) | inerte sans `DEMODIO` |
| `DEMODIO_PCHI` | code `0x9fe2` | c54x.c:2938 | tous | VALEUR | MESURE (paramètre) | idem |
| `DEMODIO_PCLO` | code `0x9f95` | c54x.c:2938 | tous | VALEUR | MESURE (paramètre) | idem |
| `DEMODRD` | code OFF | c54x.c:1836 — logge **uniquement** `pc==0x9fb5` (lecture des échantillons) : adresse, 5 mots suivants, AR0..AR7, BK. Cap 60. | tous | `EXISTS` | MESURE | — |
| `DETTRACE` | code OFF | c54x.c:14028 — trace `exec_pc∈[0xf074..0xf0c0]` (PC/XPC/op/A/B/T/AR2/3/5), cap 800. | tous | `EXISTS` | MESURE | complément de `FBDET_SENTINEL=2` |
| `FBCALL` | code OFF | c54x.c:15504 — s'arme sur la transition `d_task_md → 5` (3 rounds max), puis logge les sauts (delta>3) pendant 20000 pas, s'arrête si `pc∈[0x76f0..0x79f0]`. | tous | `EXISTS` | MESURE | — |
| `FBDET_API` | code OFF | **2 sites** : c54x.c:4153 (écriture DSP vers `api_ram[0xF8..0xFD]`, cap 40) et trx.c:499 (écriture ARM MMIO `off 0x01F0..0x01FB`, cap 40). Les deux côtés du même mot. | tous | `EXISTS` | MESURE | — |
| `FBDET_SENTINEL` | code `0` | c54x.c:2689 (résolution) + 2727-2733 (action) : `=1` **remplace la valeur écrite en `data[0x08f8]` par `0xDEAD`** ; `=2` logge sans forcer. | tous | `VALEUR` (atoi) → `=0` coupe | MESURE (intrusive à `=1`) | incompatible avec `FORCE_TOA` (qui override la lecture 0x01F0) |
| `FBENTRY` | code OFF | c54x.c:15427 — dump opcodes `0x76f8..0x7730` une fois, puis trace `pc∈[0x75e0..0x79f0]`, la sortie et 10 pas après. Cap 250. | tous | `EXISTS` | MESURE | — |
| `FBROUTE` | code OFF | c54x.c:15619 — high-water du PC dans `[0x7700..0x79f0]`, entrée `0x76fb`, jalons `0x7720/0x7725/0x798c/0x79e3/0x79e4` avec DP + `dma(0x7e)`. | tous | `EXISTS` | MESURE | — |
| `FBWATCH` | code OFF | **résolution en 2 points** (c54x.c:2384 lecture `d_dsp_page`, c54x.c:2764 écriture) ; **6 consommateurs** via `g_fbwatch_on` : 9289 (BITF `0xf7af/0xf7b7`), 13840 (canary/20M insn), 13847, 13859 (`0x9ac0`), 13868 (`0xc704/0xc472`), 13879 (dispatch `0x833b`). | tous | `EXISTS` | MESURE | — |
| `FB_CORR_ENTRY` | code `0x94f5` ; **native:=0x9500 ; native_helped:=0x9500** | c54x.c:6255 — adresse cible du reroute CALA | NATIVE, NATIVE_HELPED | VALEUR | BEQUILLE (paramètre) | **reposée par `CALYPSO_NATIVE` / `NATIVE_HELPED` : la retirer de la CLI ne la supprime pas** |
| `FB_ENERGY` | code OFF ; native:=1 ; native_helped:=1 | c54x.c:6254 — si `data[0x058a]==5` (d_task_md FB), **détourne la cible du CALA `@0xb01e`** vers `FB_CORR_ENTRY`. | NATIVE, NATIVE_HELPED | `VAL>0` → `=0` coupe | BEQUILLE (**déjà annotée** c54x.c:6244) | repose `FB_CORR_ENTRY` |
| `FB_IQ_BASE` | **code `0x2a00`** (le recensement ne le dit pas) ; native_helped:=`0x9210` | dsp_shunt.c:1531 — base d'écriture DARAM du feed | NATIVE_HELPED | VALEUR | BEQUILLE (paramètre) | inerte sans `FB_IQ_DARAM` |
| `FB_IQ_DARAM` | code OFF ; native_helped:=1 | dsp_shunt.c:1524 — dans `feed_iq`, **écrit 0x128 mots directement dans `g_shunt.c54x->data[base…]`** (hors `data_write`, donc invisible de toutes les sondes WATCH-*), IQ décimé par `CALYPSO_BSP_IQ_DECIM`. | NATIVE_HELPED | `VAL>0` | BEQUILLE (**non annotée**) | repose `FB_IQ_BASE`, `FB_IQ_FCCH_ONLY`, `FB_IQ_MARKER`, lit `BSP_IQ_DECIM` (lot 1) |
| `FB_IQ_FCCH_ONLY` | code OFF | dsp_shunt.c:1527 — restreint l'écriture aux frames `fn%51 ∈ {1,11,21,31,41}` | NATIVE_HELPED | `VAL>0` | BEQUILLE (paramètre) | inerte sans `FB_IQ_DARAM` |
| `FB_IQ_MARKER` | code OFF | dsp_shunt.c:1546 — **remplace l'IQ par une RAMPE `0x1000+woff`** sur 0x128 mots, à chaque frame, en ignorant `FCCH_ONLY`. Test de réachabilité destructif. | NATIVE_HELPED | `VAL>0` | BEQUILLE (diagnostic destructif) | exige `FB_IQ_DARAM=1` ; **shunte** la branche IQ réelle (`else if`) |
| `FB_STREAM` | code OFF ; native:=1 | **2 sites, 2 idiomes** : c54x.c:1669 (`EXISTS`) sert un échantillon du ring à chaque lecture de `0x9213/0x9215` par `pc∈[0x9f00..0x9fb8]` ; dsp_shunt.c:1583 (`VAL>0`) pousse l'IQ décimé dans le ring. | NATIVE | ⚠️ **asymétrique** : `FB_STREAM=0` coupe le producteur mais **pas** l'intercept de lecture (qui reste armé et renvoie `s->data[]` en underrun). Seul `unset` coupe les deux. | BEQUILLE (annotée aux 2 sites : 1662 et 1579) | repose CELL/CELLQ/DECIM ; consommateur `calypso_dsp_shunt_fb_stream_next` (shunt.c:2086) |
| `FB_STREAM_CELL` | code `0x9213` | c54x.c:1657 — résolue **inconditionnellement** (avant le gate FB_STREAM) | NATIVE | VALEUR | BEQUILLE (paramètre) | — |
| `FB_STREAM_CELLQ` | code `0x9215` | c54x.c:1658 — idem | NATIVE | VALEUR | BEQUILLE (paramètre) | — |
| `FB_STREAM_DECIM` | **code `4`** (recensement dit 1 → FAUX) | dsp_shunt.c:1584 — pas de décimation du push ring | NATIVE | VALEUR (`<1` → 1) | BEQUILLE (paramètre) | inerte sans `FB_STREAM` |
| `FORCE_3FAD_KERNEL` | code OFF | c54x.c:2488 — sur la **lecture** `data[0x3fad]` à `pc==0x8753`, **repose bit15** (`|= 0x8000`) pour que le BITF voie TC=1 → CC 0xa0a0 → kernel 0xa076. | tous (utile NATIVE*) | `EXISTS` | BEQUILLE | **conditionnée par `calypso_rxfb_fired`** (posé par `CALYPSO_RX_FBFLAGS`, bsp.c:1107 — lot 1) |
| `FORCE_3FAE` | code OFF | c54x.c:5121 — à **chaque instruction** avec `xpc==0` et `pc∈[0x8d00..0xa200]`, pose `data[0x3faa]|=0x0104`, `[0x3fab]|=0x0100`, `[0x3fae]|=0x0100`. | tous | `EXISTS` | BEQUILLE | — |
| `FORCE_AGCH` | hack.env:=0 ; **run.sh:1155 `=0` VERROUILLÉ en mode full-grgsm** | l1ctl_sock.c:215 — réécrit le payload L1CTL `DATA_IND(0x03)` : `chan 0x80` → rote `l3[2]` sur {0x19,0x1a,0x1b,0x1c} ; `chan 0x90` → **écrase 23 octets par un IMM ASSIGNMENT en dur**. | tous, mais **inatteignable en mode par défaut** (assignation dure, pas `:=`) | `EQ1` | BEQUILLE | verrouillée par `CALYPSO_MODE=full-grgsm` |
| `FORCE_FBSB` | hack.env:=0 ; **run.sh:1154 `=0` VERROUILLÉ** | l1ctl_sock.c:214 — force `payload[18]=0` sur `FBSB_CONF(0x02)` → SUCCESS quel que soit le résultat réel. | idem | `EQ1` | BEQUILLE | idem |
| `FORCE_NB` | code off | trx.c:373 — sur les reads MMIO `0x50/0x78` : `d_task_d=1` si 0 ; `0x52/0x7A` : renvoie `db_w->d_burst_d` (`data[0x801]`/`[0x815]`). | tous | `EQ1` | BEQUILLE | concurrencé par le bloc `SHUNT_LEGIT/NO_LEGIT + si_valid` (trx.c:309/324) qui traite **les mêmes offsets AVANT** |
| `FORCE_TOA` | code `-1` (off) ; jamais posé dans un .env (seulement documenté run.sh:92) | trx.c:344 — réécrit le read ARM : `0x01F0=1`, `0x01F4=N`, `0x01F8=0`, `0x01FA=0x7000`, `0x0060/0x0088=N`. | tous | `VALEUR` : absent/vide = off. **`FORCE_TOA=0` ACTIVE** (TOA forcé à 0) | BEQUILLE | **court-circuitée par `SHUNT_REAL_FB`** (`if (!real_fb_hit …)`) — donc morte dès que DECAN=1 |
| `PROBE_3FAD_GATE` | code OFF | c54x.c:2476 — logge la valeur lue de `0x3fad` à `pc==0x8753` + bit15 + `task_md`. Cap 200. Aucune écriture. | tous | `EXISTS` | MESURE | conditionnée par `calypso_rxfb_fired` (`RX_FBFLAGS`) |
| `SCANFB` | code OFF | c54x.c:15399 — one-shot à `exec_pc==0xb01c` : scanne les 4 banks `0x7000..0xfffe` pour les refs `0x7708/0x76fb/0x770d/0x795f`. **Modifie `s->xpc` pendant le scan puis le restaure**. | tous | `EXISTS` | MESURE | — |
| `SCANREF` | code off | c54x.c:15371 — même scan, cible libre passée en valeur. | tous | `NON-VIDE + VALEUR` : `(e && *e)`. **`SCANREF=0` l'ACTIVE** avec cible 0x0000 | MESURE | — |
| `SHUNT_DSP_FB` | code OFF | dsp_shunt.c:1693 — **dans `feed_iq`, sauvegarde tout le contexte c54x (pc/xpc/sp/st0/st1/t/a/b/idle/AR0-7), force `sp=SP_scratch` et `pc=ENTRY`, appelle `c54x_run(budget)`, logge, restaure.** Le shunt pilote le corrélateur DSP hors ordonnancement. Borné par `_sdfmax`. | tous (utile NATIVE_HELPED) ; **nécessite `real_fb` vrai** (bloc imbriqué) | `EQ1` | BEQUILLE | dépend de `SHUNT_REAL_FB`/`DECAN` (bloc parent) ; repose ENTRY/BUDGET/MAX/SP |
| `SHUNT_DSP_FB_BUDGET` | code `20000` insn | dsp_shunt.c:1697 | idem | VALEUR | BEQUILLE (paramètre) | inerte sans `SHUNT_DSP_FB` |
| `SHUNT_DSP_FB_ENTRY` | code `0x94f5` | dsp_shunt.c:1695 | idem | VALEUR | BEQUILLE (paramètre) | idem |
| `SHUNT_DSP_FB_MAX` | code `40` excursions | dsp_shunt.c:1699 — au-delà, inerte (protège osmocon) | idem | VALEUR | BEQUILLE (paramètre) | idem |
| `SHUNT_DSP_FB_SP` | code `0x5000` | dsp_shunt.c:1701 — pile dédiée à l'excursion | idem | VALEUR | BEQUILLE (paramètre) | idem |
| `SHUNT_REAL_FB` | calypso.env:=0 ; hack:=0 ; native:=0 ; shunt_no_legit:=1 | **3 sites** : dsp_shunt.c:1473 (`real_fb_read` → livre `rx_fb_det/TOA/PM/AFC/SNR` sur le read MMIO ARM `0x01F0/F4/F6/F8/FA` + SB `0x0060/0x0088`), dsp_shunt.c:1604 (calcul cohérence/dphi → `rx_*`), dsp_helper.c:241 (`shunt_dispatch_fb` écrit le NDB). | SHUNT_NO_LEGIT explicitement ; **de fait aussi NATIVE et NATIVE_HELPED** (voir colonne suivante) | `EQ1` **plus fallbacks** : 1473 = `‖DECAN=1 ‖SHUNT_LEGIT=1` ; 1604 = `‖DECAN=1` ; helper:241 = **aucun fallback**. Donc `SHUNT_REAL_FB=0` ne coupe rien tant que `DECAN=1`. | BEQUILLE (annotée à 1467 et 238 ; **site 1604 non annoté**) | **`DECAN=1` (posé par native, native_helped, shunt_legit, shunt_no_legit) l'impose** ; helper:241 gaté en amont par `INJECT_FB` |
| `WATCH_2A00` | code OFF ; **native_helped:=1** | c54x.c:2548 — logge les écritures **opcode** vers `0x2a00..0x2a07` (cap 80). N'intercepte pas `feed_iq` qui écrit `s->data[]` en direct. | NATIVE_HELPED | `EXISTS` | MESURE | — |
| `WATCH_9200` | code OFF | c54x.c:2563 — écritures opcode vers `0x9210..0x9220` / `0x9260..0x9262`, cap 80. | tous | `EXISTS` | MESURE | angle mort identique (feed_iq invisible) |
| `WATCH_9F00_RD` | code OFF | c54x.c:1683 — logge **toutes** les lectures quand `pc∈[0x9f00..0x9fb8]`, cap 200. | tous | `EXISTS` | MESURE | mesure de référence citée en 6253 (« = 0 » → l'étage n'est jamais exécuté en natif pur) |
| `WATCH_RESULT` | code OFF | c54x.c:2578 — écritures opcode vers `0x08F8..0x08FD` avec le nom (`d_fb_det`/`d_fb_mode`/`TOA`/`PM`/`ANGLE`/`SNR`), cap 120. | tous | `EXISTS` | MESURE | — |

### Effets de bord mesurés dans ce lot (l'information la plus opérationnelle)

1. **`DECAN=1` allume `SHUNT_REAL_FB` dans les modes NATIVE et NATIVE_HELPED** (`calypso_native.env:11` et `native_helped` posent `SHUNT_REAL_FB=0`, mais les deux posent `DECAN=1`, et `dsp_shunt.c:1473/1604` font `‖ DECAN`). Conséquence : **en mode « natif », le `d_fb_det` lu par l'ARM vient de l'hôte, pas du DSP.** Toute mesure « le natif détecte » prise ainsi est fausse ; seul `data[0x08f8]` (via `WATCH_RESULT` / DETECTOR-RUN) mesure le natif.
2. **`FORCE_TOA` est morte de fait** partout où `DECAN=1` : `trx.c` ne l'évalue que `if (!real_fb_hit)`, et `real_fb_read` a déjà répondu sur `0x01F0/F4/F6/F8/FA` et `0x0060/0x0088`.
3. **`FORCE_FBSB` et `FORCE_AGCH` sont VERROUILLÉES à 0** par `run.sh:1154-1155` en mode `full-grgsm` (le défaut) : assignation dure `=0` puis `export`, pas `:=`. Les mettre en ligne de commande n'a **aucun** effet.
4. **`FB_STREAM` a deux idiomes différents pour un même nom** : `EXISTS` côté lecture (c54x.c:1669), `VAL>0` côté push (dsp_shunt.c:1583). `CALYPSO_FB_STREAM=0` laisse l'intercept de lecture armé.
5. **`CORR_BANK=0` et `FORCE_TOA=0` et `SCANREF=0` ACTIVENT** leur gate (valeur légitime, pas « off »).
6. **`FB_IQ_DARAM` et `FB_IQ_MARKER` écrivent `s->data[]` hors `data_write`** → invisibles de `WATCH_2A00`/`WATCH_9200`/`WMAP`. Un « 0 hit » de ces sondes ne prouve pas l'absence d'écriture.
7. Écarts factuels au recensement : `FB_IQ_BASE` défaut **code = 0x2a00** (pas 0x9210, qui est le défaut *profil*) ; `FB_STREAM_DECIM` défaut **code = 4** (pas 1) ; `CORR_FLOW` cap réel **20000** (commentaire « 8000 » périmé) ; `FBWATCH` a **8** points d'usage (2 résolutions + 6 consommateurs), pas 2.
8. Corroboration : `CALYPSO_CORRELATOR_TRACE` (posé `:=1` dans `calypso.env:188` **et** `wire.env:34`) est bien **MORT** — `corr_trace_init_lazy` (c54x.c:789) lit `cdbg_env("CORRELATOR")`, c.-à-d. le jeton `CALYPSO_DEBUG`, jamais la variable.

---

### Blocs @BEQUILLE prêts à coller

**CORR_BANK** — `hw/arm/calypso/calypso_c54x.c:5108` (avant `static int cbk = -2;`)
```
/* @BEQUILLE — CORR_BANK  (CALYPSO_CORR_BANK, defaut -1/OFF)
 *   masque  : la selection d overlay/banque du handler FB. On ECRASE s->xpc
 *             a chaque instruction de [0x8d00..0xa200] au lieu que le
 *             dispatcher natif pose la bonne banque.
 *   retirer : quand le dispatcher CALA @0xb01e resout la banque correcte
 *             lui-meme (XPC observe == banque attendue sans forcage).
 *   Piege : la valeur "0" N ETEINT PAS — elle force XPC=0. Seul unset coupe.
 */
```

**CORR_SETUP (+ CORR_AR1 / CORR_AR4 / CORR_AR5)** — `calypso_c54x.c:14328` (au `if (exec_pc == 0x8d00)`)
```
/* @BEQUILLE — CORR_SETUP  (CALYPSO_CORR_SETUP + _AR1/_AR4/_AR5, defaut OFF)
 *   masque  : le setup de pointeurs que la LUT native 0x8341 pose avant
 *             d entrer en 0x8d00 (STM #0x2f22,AR1 / #0x2be4,AR4 / #0x0060,AR5).
 *             On INJECTE ces constantes a l entree du correlateur.
 *   retirer : quand le chemin natif passe par 0x8341 avant 0x8d00 (au lieu
 *             d y entrer par BSP-DISPATCH-FB).
 *   Idiome EXISTS : un ":=" vide l ALLUMERAIT — d ou le "unset" de
 *   calypso_wire.env:41. Mesure : inefficace (AR reecrits avant 0x8e8b).
 */
```

**FB_IQ_DARAM (+ FB_IQ_BASE / FB_IQ_FCCH_ONLY)** — `hw/arm/calypso/calypso_dsp_shunt.c:1522` (avant `static int _fid = -1, …`)
```
/* @BEQUILLE — FB_IQ_DARAM  (CALYPSO_FB_IQ_DARAM, _BASE, _FCCH_ONLY, defaut OFF)
 *   masque  : le DMA on-chip RX -> DARAM. feed_iq ecrit 0x128 mots d IQ decime
 *             DIRECTEMENT dans g_shunt.c54x->data[base..], hors data_write —
 *             donc invisible de WATCH_2A00 / WATCH_9200 / WMAP.
 *   retirer : quand la chaine BSP -> BDLENA -> DARAM alimente le buffer seule
 *             (writer 0x12ed non degenere).
 */
```

**FB_IQ_MARKER** — `calypso_dsp_shunt.c:1546`
```
/* @BEQUILLE — FB_IQ_MARKER  (CALYPSO_FB_IQ_MARKER, defaut OFF)
 *   masque  : rien de reel — remplace l IQ par une RAMPE 0x1000+woff pour
 *             tester la reachabilite de la vue DARAM du noyau. Court-circuite
 *             la branche IQ reelle (else if) et ignore FCCH_ONLY.
 *   retirer : des que la reachabilite est etablie ; ne jamais laisser en run.
 */
```

**FORCE_3FAD_KERNEL** — `calypso_c54x.c:2486`
```
/* @BEQUILLE — FORCE_3FAD_KERNEL  (CALYPSO_FORCE_3FAD_KERNEL, defaut OFF)
 *   masque  : le producteur du flag "burst pret" data[0x3fad] bit15. Un
 *             clearer per-frame (76f8 3fad 0000) l efface entre l ecriture BSP
 *             et le sweep DSP ; on le RE-POSE sur le chemin de lecture @0x8753.
 *   retirer : quand le poseur natif (chaine RX/BRINT0) tient bit15 jusqu au
 *             BITF @0x8753 — c-a-d quand PROBE_3FAD_GATE voit bit15=1 sans ce gate.
 */
```

**FORCE_3FAE** — `calypso_c54x.c:5118`
```
/* @BEQUILLE — FORCE_3FAE  (CALYPSO_FORCE_3FAE, defaut OFF)
 *   masque  : l ecriture des flags de handshake FB que RIEN n implemente —
 *             data[0x3faa] bit2/bit8, [0x3fab] bit8, [0x3fae] bit8. Poses a
 *             CHAQUE instruction du handler (xpc=0, pc 0x8d00..0xa200).
 *   retirer : quand la chaine RX/BRINT0 ecrit ces flags (RANK2 resolu).
 */
```

**FORCE_FBSB / FORCE_AGCH** — `hw/arm/calypso/l1ctl_sock.c:213`
```
/* @BEQUILLE — FORCE_FBSB / FORCE_AGCH  (CALYPSO_FORCE_FBSB, CALYPSO_FORCE_AGCH,
 *                                        defaut 0 ; VERROUILLE a 0 par run.sh:1154-1155)
 *   masque  : le resultat du demod DSP vu par le mobile. FBSB : force
 *             FBSB_CONF result -> SUCCESS. AGCH : rote le type SI du BCCH et
 *             ECRASE le L3 du PCH par un IMM ASSIGNMENT en dur.
 *   retirer : quand le demod DSP publie un a_cd valide (SI reels decodes).
 */
```

**FORCE_NB** — `hw/arm/calypso/calypso_trx.c:363`
```
/* @BEQUILLE — FORCE_NB  (CALYPSO_FORCE_NB, defaut OFF)
 *   masque  : la publication DSP de db_r->d_task_d / d_burst_d. On falsifie le
 *             read ARM (d_task_d 0->1, d_burst_d recopie de db_w) pour passer
 *             le bail "EMPTY" de l1s_nb_resp.
 *   retirer : quand le DSP NB demod ecrit lui-meme la read-page.
 *   Note : les memes offsets sont deja traites plus haut par le bloc
 *   SHUNT_LEGIT/NO_LEGIT + si_valid (trx.c:309/324) — conflit potentiel.
 */
```

**FORCE_TOA** — `calypso_trx.c:340`
```
/* @BEQUILLE — FORCE_TOA  (CALYPSO_FORCE_TOA, defaut -1/OFF)
 *   masque  : tout le bloc resultat FB/SB (d_fb_det, a_sync_demod TOA/ANGLE/SNR,
 *             a_serv_demod[D_TOA]) : oracle canne cote read MMIO ARM.
 *   retirer : quand d_fb_det natif est ecrit par le DSP.
 *   Piege : "0" ACTIVE le gate (TOA force a 0) ; seul unset/absent coupe.
 *   Deja court-circuitee par SHUNT_REAL_FB/DECAN (if !real_fb_hit).
 */
```

**SHUNT_DSP_FB (+ _ENTRY / _BUDGET / _MAX / _SP)** — `calypso_dsp_shunt.c:1687`
```
/* @BEQUILLE — SHUNT_DSP_FB  (CALYPSO_SHUNT_DSP_FB, _ENTRY, _BUDGET, _MAX, _SP,
 *                            defaut OFF)
 *   masque  : l ORDONNANCEMENT natif du correlateur. Le shunt sauvegarde le
 *             contexte c54x, force PC=ENTRY et SP=scratch, execute BUDGET
 *             instructions hors trame, puis restaure — le DSP n a jamais
 *             decide d entrer la.
 *   retirer : quand le dispatcher natif (frame-IT -> 0x8341 -> correlateur)
 *             atteint l entree seul.
 *   Nichee dans le bloc real_fb : inerte si DECAN=0 ET SHUNT_REAL_FB!=1.
 */
```

**SHUNT_REAL_FB — 3e site non annoté** — `calypso_dsp_shunt.c:1600` (avant `static int real_fb = -1;` du bloc « Detection FCCH REELLE »)
```
/* @BEQUILLE — SHUNT_REAL_FB (calcul host de rx_fb_det/AFC/SNR/TOA)
 *                            (CALYPSO_SHUNT_REAL_FB, defaut OFF)
 *   masque  : le correlateur DSP. Coherence + dphi calcules cote hote sur l IQ
 *             RX -> g_shunt.rx_* -> livres a l ARM par real_fb_read.
 *   retirer : quand data[0x08f8] est ecrit par le DSP.
 *   ⚠️ DECAN=1 SUFFIT a l allumer (native.env, native_helped.env, shunt_legit,
 *   shunt_no_legit posent tous DECAN=1) : mettre SHUNT_REAL_FB=0 ne coupe RIEN.
 */
```

---

## LOT 6 — Canaux shunt DL/UL, req-ref, RF/AFC

## LOT 6 — 59 variables (canaux shunt DL/UL, req-ref, RF/AFC/de-can, sondes registres)

**Vérifications préalables** — md5 des 5 fichiers pivots identiques au snapshot (`c54x c36466…`, `shunt a0f17a…`, `helper ef45b3…`, `bsp 476092…`, `trx 8fbe30…`) ; `calypso_trf6151.c 4ae415…`, `calypso_twl3025.c 2d08b0…` (non couverts par le snapshot). Numéros de ligne ci-dessous re-greppés, **deux écarts** au recensement : `DECAN` est en `dsp_shunt.c:1474/1604` (le recensement disait 1594) ; `SHUNT_AGCH_TTL` a **3** sites, pas 1.

### Fait structurant du lot (à lire avant le tableau)

`shunt_route_c54x()` (`calypso_dsp_helper.c:15-23`) retourne vrai dès `CALYPSO_DSP=="c54x"` — **défaut de `calypso.env`**. Or `calypso_dsp_shunt.c:1857` arme le shunt si `DSP_SHUNT=1` **OU** `L1=c` **OU** `shunt_route_c54x()`. Donc `g_shunt.active = true` **dans TOUS les modes, NATIVE et NATIVE_HELPED compris**, alors même que `CALYPSO_DSP_SHUNT=0` y est posé. Le mode « natif » n'éteint pas le shunt ; il éteint seulement les injections amont.

Trois sous-portées distinctes en découlent, à ne pas confondre :

| portée | gate hôte | vars du lot concernées |
|---|---|---|
| **Tous modes** (shunt armé par `DSP=c54x`) | aucun | `DECAN`(→REAL_FB), `SHUNT_PM`, `TRF_RXLEV`, `TRF_TARGET_RF` (via `shunt_dispatch_pm`, sans gate INJECT), `SHUNT_BURST_*`, `UL_*`, `SHUNT_IQ_*`, `SHUNT_*_PORT`, `TWL3025_*`, `TRF_TSP_DEV` |
| **`INJECT_ACD=1` OU `SHUNT_LEGIT=1`** (`dsp_helper.c:345`, garde d'entrée de `shunt_dispatch_allc`) | `return` immédiat sinon | `SHUNT_AGCH*`, `SHUNT_SDCCH*`, `SHUNT_SACCH*`, `SHUNT_BCCH_*` → **INERTES en NATIVE/NATIVE_HELPED** |
| **`SHUNT_LEGIT=1` OU `SHUNT_NO_LEGIT=1`** (`dsp_shunt.c:625`, bloc `on_frame_tick`) | bloc entier sauté sinon | `DECAN_TOA/SNR/ANGLE/PM` (sites 776-779, 652), `SHUNT_SI_ROT_MASK`, `SHUNT_AGCH_EXPIRE` |

**Conséquence la plus opérationnelle du lot** : `calypso_native.env:25` et `calypso_native_helped.env:25` posent `CALYPSO_DECAN:=1`. Or `dsp_shunt.c:1474-1476` fait `real_fb = (SHUNT_REAL_FB=='1') || (DECAN[0]=='1')`, et `calypso_dsp_shunt_real_fb_read()` (`:1463`) **n'a aucune garde `g_shunt.active`** : appelée inconditionnellement depuis `calypso_trx.c:297` sur chaque read MMIO ARM 16 bits, elle intercepte 0x01F0/1F4/1F6/1F8/1FA et retourne `g_shunt.rx_*`. **Donc en NATIVE et NATIVE_HELPED, `d_fb_det` lu par l'ARM ne vient JAMAIS de l'API-RAM native — il vient du hôte, par simple effet de bord de `DECAN=1`.** Le bloc `@BEQUILLE` déjà présent en `dsp_shunt.c:1466-1471` avertit littéralement « ne jamais l'activer pour juger de l'état du mode natif » — et les deux profils natifs l'activent en douce. `SHUNT_REAL_FB=0` ne suffit pas à le couper : il faut `DECAN≠1` **et** `SHUNT_LEGIT≠1` (3e fallback, `:1479`).

---

## Tableau

| VARIABLE | DEFAUT | EFFET (code exécuté) | MODE | IDIOME | CATÉGORIE | REPOSE / REPOSÉE PAR |
|---|---|---|---|---|---|---|
| `AR0_DEBUG` | code OFF ; aucun .env | 8 sondes `fprintf` : VECWATCH 0x71f4 (`c54x.c:3278`), LEVELCHK-DBG INTM/IPTR/pend (`:4945`), dumps PC 0x013b/0x8869/0x7234 (`:5249/5260/5273`), SCHED-7234 one-shot (`:5460`), AR0-DELTA (`:14150`), PROG-DUMP@0xb405 (`:14164`). Zéro écriture d'état. | tous | `EXISTS` ×8 → `unset` seul coupe | **MESURE** — lecture pure, aucun effet émulé | — |
| `AR2_FLOOR_DROP` | code OFF | `c54x.c:4616` — **modifie l'adressage** : si `ARP==2 && addr<0x0800` → `addr=0xFFFF` (scratch). Le log est gaté `DEBUG=AR2-FLOOR`, **le drop ne l'est pas**. | tous (chemin adressage indirect) | `EQ1` (`*e=='1'`) | **BEQUILLE** — détourne un accès mémoire au lieu de corriger le calcul d'AR2 | indépendante du jeton `DEBUG` — ⚠️ le recensement la classait « log gate », c'est faux |
| `AR6_AT_LOG_CAP` | code 200 | `c54x.c:428` — cap du log AR6-AT | tous | `VALEUR`, inerte sans `DEBUG=AR6-AT` (`:423`) | **MESURE** | reposée par jeton `DEBUG=AR6-AT` |
| `AR6_AT_PC` | code 0 | `c54x.c:422` — PC déclencheur AR6-AT | tous | idem | **MESURE** | idem ; listée `run.sh:810` (menu expert) |
| `AR6_WIN_HI` | code `0xFFFFFFFF` | `c54x.c:427` — borne haute fenêtre insn | tous | idem | **MESURE** | idem |
| `AR6_WIN_LO` | code 0 | `c54x.c:426` — borne basse | tous | idem | **MESURE** | idem |
| `ARWATCH` | code OFF | `c54x.c:15206` — dump AR0..AR7 + flag in-buffer `[0x2a00,0x2b28)` aux PC 0x8d00/8d1a/8e5f/8e8c/8e97, cap 60 | tous (corrélateur) | `EXISTS` | **MESURE** | — |
| `AR_TRACE` | masque `0xFF` | `c54x.c:258` — **masque de bits AR0..AR7 seulement** ; l'activation vient de `calypso_debug_enabled("AR-TRACE")` (`:260`) | tous | `VALEUR` ; **inerte sans le jeton** | **MESURE** | reposée par `DEBUG=AR-TRACE` |
| `A_TRACE_PC` | code 0 | `c54x.c:349` — PC déclencheur du suivi accumulateur A | tous | `VALEUR` ; inerte sans `DEBUG=A-TRACE` | **MESURE** | idem |
| `DA_HI` | code `0x9FFF` | `c54x.c:14955` — borne haute audit décodeur | tous | `VALEUR` ; inerte sans `DEBUG=DECODE-AUDIT` (`:14963`) | **MESURE** | idem |
| `DA_INSN` | code 0 | `c54x.c:14956` — seuil `insn_count` (saute le boot) | tous | idem | **MESURE** | idem |
| `DA_LO` | code `0x8000` | `c54x.c:14955` — borne basse (overlay corrélateur) | tous | idem | **MESURE** | idem |
| **`DECAN`** | absent de `calypso.env` ; **`:=1`** dans `native.env:25`, `native_helped.env:25`, `shunt_legit.env:4`, `shunt_no_legit.env:19` | 5 sites. `:1474` + `:1604` → **implique `SHUNT_REAL_FB`** (intercept read-side ARM 0x01F0…0x01FA, sans garde `active`). `:564` → PM de-canné. `:651` → garde `rx_snr` réel. `:774` → implique TOA/PM/SNR/ANGLE | `:1474/1604` = **tous modes** ; `:564/651/774` = LEGIT/NO_LEGIT seulement | `EQ1` (`dm[0]=='1'`) | **BEQUILLE** — son effet dominant mesurable est d'allumer l'intercept hôte du résultat FB, qui masque le natif | **repose** `SHUNT_REAL_FB`, `DECAN_PM/SNR/TOA/ANGLE` ; **reposée par** les 4 profils .env |
| `DECAN_ANGLE` | code OFF | `dsp_shunt.c:779` → `ang_v = rx_afc` ; OFF → **canne `0`** écrite en `d[0x832]/d[0x846]` | LEGIT/NO_LEGIT | `EQ1` **OU** `DECAN` | **BEQUILLE** (c'est l'état OFF qui est la béquille) | reposée par `DECAN` |
| `DECAN_PM` | code OFF | Site **vivant** `:565` (dans `shunt_pm_decan_apm`, appelé `:685` et `:787`) → `rf = 20·log10(last_pm/MAV_REF)+RF_REF`, borné `[-75,-40]`, seuil `last_pm>1000`. Site `:777` (`dc_pm`) **n'est utilisé que dans la condition du `fprintf` `:794`** → inerte comportementalement | LEGIT/NO_LEGIT | `EQ1` **OU** `DECAN` | **BEQUILLE** (OFF → `apm_for_rf(-60)` figé) | reposée par `DECAN` |
| `DECAN_PM_MAV_REF` | code `20929.0` | `:567` — ancrage MAV du modèle ; clampé `≥1.0` | LEGIT/NO_LEGIT | `VALEUR` (`atof` si non vide) | **CONFIG** — point de calibration RSSI, équivalent hardware réel | — |
| `DECAN_PM_RF_REF` | code `-60.0` | `:568` — dBm correspondant à `MAV_REF` | LEGIT/NO_LEGIT | `VALEUR` | **CONFIG** | — |
| `DECAN_SNR` | code OFF | `:652` → n'écrase pas `rx_snr` par `0x7000` ; `:778` → `snr_v = rx_snr` | LEGIT/NO_LEGIT | `EQ1` **OU** `DECAN` | **BEQUILLE** (OFF → SNR canné `0x7000`) | reposée par `DECAN` |
| `DECAN_TOA` | code OFF | `:776` → `toa_v = rx_toa` si `sb_valid` ; OFF → **canne `23`** | LEGIT/NO_LEGIT | `EQ1` **OU** `DECAN` | **BEQUILLE** | reposée par `DECAN` |
| `REQREF_ADJ` | code 0 ; `calypso.env:76 := (vide)` → `atoi("")==0` | `:1023` — décalage FN appliqué à la req-ref réécrite | LEGIT/NO_LEGIT (via `feed_agch`, gaté `INJECT_AGCH`) | `VALEUR` — **`e ? atoi(e) : 0`, la chaîne vide donne 0** | **BEQUILLE** (paramètre du FN-FIX ; aucun équivalent hardware) | dépend de `REQREF_PERRA`/`REWRITE` pour tirer |
| `REQREF_LAST_RACH` | **ON** | `:1030` — lit `last_rach.fn` @0x836500 dans la RAM ARM du firmware et récrit `agch_buf[8..9]` (T1'/T2/T3) | idem | `ON-sauf-0` | **BEQUILLE** — falsifie l'IMM-ASSIGN de la BTS pour matcher la mémoire locale du mobile | prioritaire sur `REQREF_PERRA` puis `REWRITE` |
| `REQREF_PERRA` | **ON** | `:1022` — même réécriture, clé `g_rach_conf_fn[ra]` | idem | `ON-sauf-0` | **BEQUILLE** | fallback de `LAST_RACH` |
| `REQREF_REWRITE` | OFF | `:1021` — ancien rewrite global `g_last_rach_conf_fn` | idem | `EQ1` | **BEQUILLE** (legacy, ~50 % d'échec multi-RACH selon le commentaire) | dernier fallback |
| `RMAP` | OFF | `c54x.c:1777` — histogramme (PC → plage d'adresses lues), 8 adresses distinctes/PC | tous | `VAL>0` (`0` coupe) | **MESURE** | repose `RMAP_PCLO/PCHI` |
| `RMAP_PCHI` | code `0x9fff` | `c54x.c:1779` | tous | `VALEUR` | **MESURE** | inerte sans `RMAP` |
| `RMAP_PCLO` | code `0x9f00` | `c54x.c:1779` | tous | `VALEUR` | **MESURE** | idem |
| `SHUNT_AGCH` | **ON** | `dsp_helper.c:378` — présente l'IMM-ASSIGN dans `a_cd` sur **chaque** bloc CCCH (`fn%51 ∈ {6-9,12-19}`) tant que TTL, via `shunt_write_w` | INJECT_ACD/LEGIT ; **inerte NATIVE** | `ON-sauf-0` | **BEQUILLE** — fabrique un bloc AGCH décodé | repose `_OFS`/`_TTL` |
| `SHUNT_AGCH_EXPIRE` | **OFF** (code) ; `shunt_no_legit.env:28 :=1` | `dsp_shunt.c:722` — clear `agch_valid` après `_agex` ticks, débloque le gate SI + le paging | LEGIT/NO_LEGIT | `VAL>0` | **BEQUILLE** — correctif d'une béquille (latch AGCH qui empoisonnait le SI) | consomme `SHUNT_AGCH_TTL` |
| `SHUNT_AGCH_OFS` | code 0 | `dsp_helper.c:379` — décale la fenêtre `fn%51` de présentation | INJECT_ACD/LEGIT | `VALEUR` (`o ? atoi(o) : 0` → **`""` donne 0**) | **BEQUILLE** (param.) | — |
| `SHUNT_AGCH_TTL` | code **100** ×3 | `dsp_helper.c:380` (péremption présentation), `dsp_shunt.c:723` (TTL de l'expiry), `dsp_shunt.c:957` (fenêtre de DROP du paging concurrent) — **3 consommateurs, même défaut, sémantiques différentes** | INJECT_ACD/LEGIT + tick | `VALEUR` (`t && *t`) | **BEQUILLE** (param.) | partagée par 3 béquilles distinctes |
| `SHUNT_BCCH_OFS` | code 0 | `dsp_helper.c:593` — offset `fn%51` du créneau BCCH | INJECT_ACD/LEGIT | `VALEUR` | **BEQUILLE** (param.) | inerte sans `BCCH_SCHED` |
| `SHUNT_BCCH_SCHED` | **OFF** | `dsp_helper.c:591-592` : `bcch_sched = (e && *e=='1')`. ⚠️ **le commentaire d'en-tête `:588` dit « défaut 1 » — PÉRIMÉ**, le code exécuté est défaut OFF (commentaire inline `:592` correct). ON → SI présenté seulement sur `TC∈[2,5]`, + garde anti-famine (`n_disp>200 && n_since_bcch<102`) | INJECT_ACD/LEGIT | `EQ1` | **BEQUILLE** — ordonnance à la main ce que le décodeur DSP produirait | repose `BCCH_OFS` |
| `SHUNT_BURST_FN` | code 0 (echo) ; `shunt_no_legit.env:22 :=1` | `dsp_helper.c:187` — `shunt_burst_echo()` : 1 → `d_burst_d = (l1s_fn+ofs)&3` ; 0 → écho de la commande ARM | tous modes (echo appelé par tous les dispatch) | `VALEUR` (`e && *e`) | **BEQUILLE** — synthétise l'identifiant de burst que le DSP devrait dériver du TPU | repose `BURST_OFS`/`BURST_M1` |
| `SHUNT_BURST_M1` | absente | `dsp_helper.c:193` — **consultée seulement si `BURST_OFS` absente ou vide** ; pose `ofs=-1` | idem | `EXISTS` (conditionnel) | **BEQUILLE** (raccourci de `BURST_OFS=-1`) | masquée par `SHUNT_BURST_OFS` |
| `SHUNT_BURST_OFS` | code **`fn_mode ? 2 : -2`** ; `shunt_no_legit.env:23 :=-1` | `dsp_helper.c:191` — phase du burst-id | idem | `VALEUR` | **BEQUILLE** (param.) | prime sur `BURST_M1` |
| `SHUNT_BURST_PERCMD` | **ON** aux 2 sites | ⚠️ **DEUX IDIOMES DIVERGENTS pour une seule variable** : `dsp_shunt.c:200` → `pc = (e && *e == 0) ? 0 : 1` (**seule la chaîne VIDE coupe ; `=0` laisse ON**) ; `dsp_shunt.c:1940` → `en = (e && *e == '0') ? 0 : 1` (**`=0` coupe**). Donc `PERCMD=0` désarme le miroir per-commande (`:1940`) mais **laisse actif** le `if(!pc)` inversé de `:201`, qui du coup ne re-capture toujours pas `d_burst_d`. État `=0` = ni miroir ni capture. | tous modes | `ON-sauf-VIDE` (`:200`) **/** `ON-sauf-0` (`:1940`) | **BEQUILLE** | — |
| `SHUNT_GSMTAP_PORT` | code 4730 ; `run.sh:2032` idem | `dsp_shunt.c:1201` — port UDP du listener GSMTAP (SI depuis gr-gsm) | tous (sauf `SHUNT_NO_GRGSM=1`, `:1198`) | `VALEUR` | **CONFIG** | reposée par `SHUNT_NO_GRGSM` (LOT 4) |
| `SHUNT_IQ_CFILE` | **code `/root/dsp_iq.cfile` si absente** ; `calypso.env:34 := /dev/shm/dsp_iq.fifo` | `:1406-1422` — tee I/Q fc32 ; FIFO détectée par `stat`/`S_ISFIFO` → `O_NONBLOCK` ; sinon `O_TRUNC` | tous | `CHAINE` — **vide = off** (`if (*cf)`) | **MESURE** | — |
| `SHUNT_IQ_CFILE2` | absente = off | `:1448` — second cfile FN-espacé (zero-fill) pour rejeu 51-mf offline | tous | `CHAINE` (`cf2 && *cf2`) | **MESURE** | — |
| `SHUNT_IQ_RECORD` | **code `/dev/shm/dsp_iq.cfile` si absente** ; `calypso.env:35` idem | `:1435-1446` — record contigu ; anti-double-open si identique à `IQ_CFILE` non-FIFO | tous | `CHAINE` — vide = off | **MESURE** | lit `g_iq_path`/`g_iq_is_fifo` posés par `IQ_CFILE` |
| `SHUNT_PM` | code **-1 = utiliser le modèle** | `dsp_helper.c:688` — `strtol(e,NULL,0)`; `≥0` → `a_pm` brut forcé, **bypass total du modèle trf6151** | **tous modes** (`shunt_dispatch_pm` n'a pas de gate INJECT, seulement `SHUNT_NO_FAKE_PM`, `dsp_shunt.c:845/875`) | `VALEUR` (`e && *e`) | **BEQUILLE** — valeur de rxlev fabriquée | prime sur `TRF_RXLEV`/`TRF_TARGET_RF` |
| `SHUNT_SACCH` | **ON** | `dsp_helper.c:513` — présente `sacch_buf` (SI6/B4) sur `tco∈[42,46]` (le commentaire dit 42-45, **le code teste `<=46`**) et parité `mf102` | INJECT_ACD/LEGIT | `ON-sauf-0` | **BEQUILLE** | repose `_PAR`/`_OFS` |
| `SHUNT_SACCH_OFS` | code 0 | `dsp_helper.c:533` — décale `tco` | idem | `VALEUR` | **BEQUILLE** (param.) | — |
| `SHUNT_SACCH_PAR` | code **2** (= les deux parités) | `dsp_helper.c:532` — 0 = pair (legacy), 1 = impair, 2 = les deux. Contourne l'inversion de parité induite par le recalage `l1s_fn` (-556) | idem | `VALEUR` (`e ? atoi(e) : 2`) | **BEQUILLE** — compense une dérive d'horloge FN non corrigée à la source | — |
| `SHUNT_SCH_PORT` | code 4731 | `dsp_shunt.c:1306` — port UDP listener SCH (BSIC/FN/TOA gr-gsm) | tous (sauf `SHUNT_NO_GRGSM`) | `VALEUR` | **CONFIG** | reposée par `SHUNT_NO_GRGSM` |
| `SHUNT_SDCCH` | **ON** | `dsp_helper.c:427` — draine la ring SDCCH vers `a_cd` dans la fenêtre `[_lo,_hi]` dérivée de `sdcch_ch8` (SDCCH/4 → 22-39, /8 → 0-31) | INJECT_ACD/LEGIT | `ON-sauf-0` | **BEQUILLE** | repose `_RING`/`_OFS`/`_TTL`/`_MAXPRES` |
| `SHUNT_SDCCH_MAXPRES` | code **8** | `dsp_helper.c:473` — drop forcé après N présentations même si `d_burst_d` reste bloqué (anti-stall) | idem | `VALEUR` (`e && *e`) | **BEQUILLE** (param. anti-stall) | — |
| `SHUNT_SDCCH_OFS` | code 0 | `dsp_helper.c:428` | idem | `VALEUR` | **BEQUILLE** (param.) | — |
| `SHUNT_SDCCH_RING` | **ON** aux 2 sites | `dsp_helper.c:432` (drain) + `dsp_shunt.c:1073` (enqueue) — file circulaire `SDCCH_RING_N` au lieu d'un slot unique | idem | `ON-sauf-0` ×2 | **BEQUILLE** | — |
| `SHUNT_SDCCH_TTL` | code **4000** (`dsp_helper.c:425`) — **pas 100** | `dsp_helper.c:429` — péremption d'une entrée de ring en ticks | idem | `VALEUR` (`t && *t`) | **BEQUILLE** (param.) | — |
| `SHUNT_SI_ROT_MASK` | code **7** | `dsp_shunt.c:742` — `(si_rot++ & mask)==0` déclenche la rotation SI1→SI4 ; 0 = à chaque bloc | LEGIT/NO_LEGIT | `VALEUR` (`e && *e`) | **BEQUILLE** (param. de l'injection SI) | — |
| `TRF_RXLEV` | code OFF ; `shunt_no_legit.env:21 :=1` | 3 sites, **tous avec fallback `SHUNT_LEGIT` seul, jamais `SHUNT_NO_LEGIT`** : `c54x.c:2710` (force `a_pm` sur `data[0x834-0x836]`/`[0x848-0x84A]` à l'écriture DSP), `dsp_helper.c:696` (`shunt_dispatch_pm`), `dsp_shunt.c:678` (écriture `api_ram[0x30-0x32]/[0x44-0x46]`) | `c54x.c:2710`+`helper:696` = tous modes ; `shunt.c:678` = LEGIT/NO_LEGIT | `EQ1` **OU** `SHUNT_LEGIT=='1'` | **BEQUILLE** — substitue `a_pm` que le DSP écrit à 0 | reposée explicitement par `shunt_no_legit.env` (d'où le commentaire `[gate SHUNT_LEGIT-only sinon]`) |
| `TRF_TARGET_RF` | code **-60** ×3 | `c54x.c:2712`, `dsp_helper.c:697`, `dsp_shunt.c:680` — cible dBm passée à `calypso_trf6151_apm_for_rf()` | idem `TRF_RXLEV` | `VALEUR` (`t && *t`) | **BEQUILLE** (param. — le niveau RF est une constante décrétée, pas mesurée) | inerte sans `TRF_RXLEV` ; écrasée par `SHUNT_PM≥0` |
| `TRF_TSP_DEV` | code **1** | `calypso_trf6151.c:70` — index device TSP retenu ; les writes d'un autre dev sont ignorés (`:73`), et seuls `REG_RX` sont conservés (`:76`) | tous | `VALEUR` (`e && *e`) | **CONFIG** — câblage TSP réel (dev 0 = TWL3025 ABB, dev 1 = trf6151) | — |
| `TWL3025_AFC` | **ON** | `calypso_twl3025.c:99-100` — `afc_enabled` ; OFF = pas de rotation des samples RX par l'offset VCXO (I/Q brute) | tous | `ON-sauf-0` | **CONFIG** — le VCXO/AFC existe sur silicium ; opt-out de modélisation | — |
| `TWL3025_AFC_HZ` | code **0** = boucle réelle ; `calypso_hack.env:34-35 :=0` **+ `export` explicite** | `calypso_twl3025.c:94` — non nul → offset AFC constant forcé, court-circuite `dac_value`/pente | tous | `VALEUR` (`h && *h`) ; **0 = inerte** | **BEQUILLE** (quand ≠0) — fige la boucle AFC au lieu de la faire converger | `hack.env` la pose à 0 (donc inoffensive) mais la **rend présente** ; listée `run.sh:813` (menu expert) |
| `UL_ACU_OFS` | code **6** | `dsp_shunt.c:224` — base de la fenêtre de 30 octets lue en `NDB+0x264+ofs`, puis **scan** (`:230-235`) du début de trame LAPDm (EA=1, SAPI 0/3) | tous modes (dès `d_task_u != 0`) | `VALEUR` (`e && *e`) | **CONFIG** — offset d'un layout mémoire réel ⚠️ mais le **scan** qui l'entoure est, lui, une béquille : le code admet ne pas connaître l'offset exact | — |
| `UL_PUB_IDLE` | code **OFF** | `dsp_shunt.c:154-156` — OFF ⇒ `if (l2[1]==0x03) return` : les trames de fill UI ne sont **pas** publiées vers le sideband. ON = ancien comportement (tout publier) | tous modes | `EQ1` | **BEQUILLE** — filtre applicatif compensant un sideband à **slot unique** (le fill écrasait la SABM avant échantillonnage) | condition de retrait = sideband en file, pas en slot |
| `UL_RACH_FROM_DRACH` | code : fallback `SHUNT_LEGIT` ; `shunt_no_legit.env:18 :=1` | `calypso_trx.c:783-789` — sur écriture ARM de `d_rach`, appelle directement `calypso_bsp_send_rach_ra()` (1 write = 1 burst) | tous modes | **`if (e) ulr = (*e=='1'); else ulr = SHUNT_LEGIT`** → **`=0` coupe MÊME sous `SHUNT_LEGIT=1`** (contrairement aux `INJECT_*` du LOT 4) | **BEQUILLE** — émet l'access-burst depuis un signal ARM parce que le poll UL natif ne tire jamais (`d_task_ra` avalé par le shunt) | dépend de `NDB_D_RACH_OFFSET` (LOT 1, défaut code `0x023A`) |

**Aucune variable MORTE dans ce lot** : les 59 sont lues par du code atteignable. Les plus proches du statut mort sont `DECAN_PM` **au site `:777`** (calculé dans `dc_pm`, utilisé uniquement dans la condition du `fprintf` `:794` — le site vivant est `:565`) et `SHUNT_BURST_M1` (jamais consulté si `SHUNT_BURST_OFS` est définie, ce que fait `shunt_no_legit.env:23`).

---

## Blocs `@BEQUILLE` prêts à coller

```
AR2_FLOOR_DROP / calypso_c54x.c:4613 (avant `static int ar2_drop = -1;`)
masque  : le calcul d'adresse d'AR2 dans le corrélateur, qui sous-déborde le buffer
          DARAM 0x0800 jusqu'à l'espace MMR (0x00=IMR, 0x1E=XPC) et le clobbe.
          Le drop redirige l'accès vers 0xFFFF au lieu de corriger le pointeur.
retirer : quand AR2 reste dans [0x0800,0x2b28) sur tout le kernel FB — mesurable
          par le compteur du token DEBUG=AR2-FLOOR resté à 0 sur un run complet.
```

```
DECAN / calypso_dsp_shunt.c:1466 (bloc @BEQUILLE SHUNT_REAL_FB déjà présent — AJOUTER)
masque  : (effet de bord) DECAN=1 allume SHUNT_REAL_FB via la ligne 1475. Or
          calypso_dsp_shunt_real_fb_read() n'a PAS de garde g_shunt.active et est
          appelée depuis calypso_trx.c:297 sur chaque read MMIO ARM. Conséquence :
          calypso_native.env:25 et calypso_native_helped.env:25 (DECAN:=1) rendent
          l'intercept hôte ACTIF EN MODE NATIF — d_fb_det lu par l'ARM ne vient
          jamais de l'API-RAM. Le mode « natif » ne mesure donc pas le natif.
retirer : DECAN≠1 ET SHUNT_REAL_FB≠1 ET SHUNT_LEGIT≠1 (3 fallbacks, l.1475+1479),
          ou ajouter `if (!g_shunt.active) return false;` en tête de la fonction.
```

```
DECAN_TOA / DECAN_SNR / DECAN_ANGLE / DECAN_PM  /  calypso_dsp_shunt.c:771 (bloc `static int dc_toa`)
masque  : l'ÉTAT OFF est la béquille. OFF -> a_serv_demod des read pages reçoit des
          constantes cannées : TOA=23, SNR=0x7000, ANGLE=0, PM=apm_for_rf(-60) figé,
          à la place de la sortie du modèle (gr-gsm sb_toa, feed_iq rx_snr/rx_afc,
          MAV last_pm). Elles masquent l'absence de mesure DSP native.
retirer : quand le corrélateur natif écrit lui-même a_sync_demod[TOA/PM/ANGLE/SNR].
note    : DECAN_PM au site 777 (dc_pm) n'alimente QUE la condition du fprintf l.794 ;
          le gate PM effectif est l.565 dans shunt_pm_decan_apm().
```

```
REQREF_LAST_RACH / REQREF_PERRA / REQREF_REWRITE / REQREF_ADJ  /  calypso_dsp_shunt.c:1020
masque  : la cohérence FN entre l'horloge L1 du mobile et la req-ref émise par la BTS.
          On RÉÉCRIT agch_buf[8..9] (T1'/T2/T3) de l'IMM-ASSIGN reçu pour qu'il matche
          last_rach.fn lu dans la RAM ARM @0x836500 — c'est-à-dire qu'on falsifie le
          message réseau pour compenser un skew d'horloge local.
retirer : quand shunt_l1s_fn() et calypso_trx_get_fn() sont alignées sur la SCH sans
          recale résiduel (cf. RANK4, recale -556) — alors la req-ref native matche.
```

```
SHUNT_AGCH / SHUNT_AGCH_OFS / SHUNT_AGCH_TTL / SHUNT_AGCH_EXPIRE  /  calypso_dsp_helper.c:376  et  calypso_dsp_shunt.c:718
masque  : le décodage CCCH/AGCH par le DSP. On écrit l'IMM-ASSIGN directement dans
          a_cd (FIRE=0 = CRC pass forcé) sur chaque bloc CCCH de la fenêtre fn%51,
          avec TTL et expiry maison, au lieu que le DSP produise un bloc décodé.
retirer : quand le corrélateur natif alimente a_cd et pose a_cd[0]=FIRE réel.
note    : SHUNT_AGCH_TTL a 3 consommateurs de sémantique différente (helper:380
          péremption, shunt:723 expiry, shunt:957 fenêtre de drop du paging).
```

```
SHUNT_SDCCH / _RING / _OFS / _TTL / _MAXPRES  /  calypso_dsp_helper.c:424
masque  : idem AGCH pour le canal dédié — la ring rejoue le bloc L2 dans a_cd sur la
          fenêtre fn%51 déduite de sdcch_ch8, avec drop forcé après MAXPRES
          présentations pour compenser un d_burst_d qui reste bloqué en mode DSP //.
retirer : quand d_burst_d natif progresse 0->3 par bloc et que a_cd est alimenté
          par le décodeur DSP.
```

```
SHUNT_SACCH / SHUNT_SACCH_PAR / SHUNT_SACCH_OFS  /  calypso_dsp_helper.c:507
masque  : la présentation du bloc SACCH dédié. PAR=2 (défaut) présente sur LES DEUX
          parités mf102 parce que le recale de shunt_l1s_fn (-556) INVERSE la parité
          en cours de run — la béquille compense donc une dérive d'horloge, pas une
          absence de décodage.
retirer : quand l1s_fn ne subit plus de recale à l'exécution (parité mf102 stable) ;
          alors PAR=0 (legacy, parité paire seule) redevient correct.
```

```
SHUNT_BCCH_SCHED / SHUNT_BCCH_OFS  /  calypso_dsp_helper.c:585
masque  : l'ordonnancement mf-51 que le DSP devrait imposer — on filtre à la main
          TC∈[2,5] pour éviter que le SI3 fuite en PCH/AGCH, avec une garde
          anti-famine qui dégrade en « SI partout » après 200 dispatches.
retirer : quand le dispatcher natif présente a_cd sur le bon type de bloc.
⚠️ le commentaire d'en-tête l.588 annonce « défaut 1 » : PÉRIMÉ, le code l.592 est
   `(e && *e=='1')` = défaut OFF.
```

```
SHUNT_BURST_FN / SHUNT_BURST_OFS / SHUNT_BURST_M1 / SHUNT_BURST_PERCMD
  /  calypso_dsp_helper.c:170  et  calypso_dsp_shunt.c:196 + 1936
masque  : la dérivation de d_burst_d (0..3) depuis la fenêtre TPU. Le shunt la
          synthétise soit par écho de la commande ARM (+ofs), soit depuis l1s_fn,
          et la MIROITE sur les deux read-pages à chaque write WP_D_BURST_D.
retirer : quand la fenêtre RX TPU/BDLENA cadence le burst-id côté DSP (RANK2).
⚠️ PIÈGE : SHUNT_BURST_PERCMD a DEUX idiomes contradictoires — l.200
   `(e && *e == 0) ? 0 : 1` (seule la chaîne VIDE coupe) et l.1940
   `(e && *e == '0') ? 0 : 1` (la valeur "0" coupe). Poser =0 désarme le miroir
   mais laisse la capture latch désarmée aussi : état ni-l'un-ni-l'autre.
```

```
SHUNT_PM  /  calypso_dsp_helper.c:686
masque  : la mesure de puissance (tâche PM md=1). Une valeur brute décrétée est
          écrite dans a_pm[0..2], court-circuitant même le modèle trf6151.
retirer : quand le DSP produit a_pm depuis l'I/Q. ⚠️ shunt_dispatch_pm n'a AUCUN
          gate INJECT_* : cette béquille est vivante en NATIVE et NATIVE_HELPED,
          seul SHUNT_NO_FAKE_PM=1 (dsp_shunt.c:845/875) la coupe.
```

```
TRF_RXLEV / TRF_TARGET_RF  /  calypso_c54x.c:2707 , calypso_dsp_helper.c:690 , calypso_dsp_shunt.c:675
masque  : a_pm que le vrai DSP écrit à 0 (aucune mesure). On substitue
          apm_for_rf(TARGET_RF) sur les 3 chemins de lecture ARM (data[0x834-0x836]
          /[0x848-0x84A], api_ram[0x30-0x32]/[0x44-0x46], read pages du dispatch).
          Le niveau RF cible est une constante décrétée (-60 dBm), pas une mesure.
retirer : quand a_pm natif est non nul. Le modèle trf6151 (gain suivi par TSP) reste,
          lui, légitime — seule la CIBLE figée est la béquille.
⚠️ aucun des 3 sites ne teste SHUNT_NO_LEGIT ; c'est shunt_no_legit.env:21 qui pose
   TRF_RXLEV=1 explicitement.
```

```
TWL3025_AFC_HZ  /  calypso_twl3025.c:92
masque  : la boucle AFC fermée (DAC firmware -> pente Hz/LSB -> rotation des samples).
          Une valeur non nulle fige un offset constant en Hz et supprime la convergence.
retirer : jamais nécessaire — mettre 0 (ou unset) suffit ; c'est un outil de diag.
note    : calypso_hack.env:34-35 la pose à 0 ET l'exporte : inoffensive en valeur,
          mais elle devient PRÉSENTE dans l'environnement.
```

```
UL_PUB_IDLE  /  calypso_dsp_shunt.c:152
masque  : la profondeur du sideband UL (slot unique, 1 pread/frame côté qemu_wrap).
          Le filtre par défaut (ne pas publier les trames de fill ctrl==0x03) empêche
          l'écrasement de la SABM transitoire — il compense l'absence de file.
retirer : quand le sideband UL est une file (ring) et non un slot unique ; alors
          publier l'idle redevient sans risque.
```

```
UL_RACH_FROM_DRACH  /  calypso_trx.c:779
masque  : le poll UL natif, qui ne tire jamais l'access-burst parce que SHUNT_LEGIT
          avale d_task_ra. On émet le burst depuis l'écriture ARM de d_rach
          (1 write = 1 burst, sans sticky).
retirer : quand d_task_ra atteint le producteur UL sans être consommé par le shunt.
⚠️ IDIOME PARTICULIER : `if (e) ulr = (*e=='1'); else ulr = SHUNT_LEGIT`. Poser
   =0 la coupe MÊME sous SHUNT_LEGIT=1 — comportement inverse des INJECT_* du LOT 4,
   qui retombent sur SHUNT_LEGIT quoi qu'il arrive.
```

---

## Annotation @BEQUILLE — ce qui a été marqué, et ce qui ne l'a pas été

Done. Final report:

## MARQUEURS POSÉS — 99 nouveaux (105 au total dans `hw/arm/calypso/*.c`, dont 6 préexistants non dupliqués)

Toutes lignes = position finale, après insertion, dans `${QEMU_TREE}/hw/arm/calypso/`.

**calypso_bsp.c (12)** — BSP_DARAM_FORCE:463 · BSP_DIRECT_FEED:660 · BSP_DARAM_FORCE(rx_burst):1007 · BSP_DIRECT_BRINT0:1107 · RX_FBFLAGS(vivant):1133 · POKE_TASK_MD+POKE_DISPATCH:1158 · BSP_DISPATCH_FB(+_TGT/_NOIMR/_ONESHOT):1200 · FB_IQ_OWNS:1276 · TPU_RX_WIRE(livraison):1417 · TPU_RX_WIRE(pulse BDLENA):1470 · RX_FBFLAGS(buffered/mort):1663 · RACH_FORCE_BSIC:1866

**calypso_trx.c (5)** — DL_FN_OFFSET:161 · FORCE_TOA:350 · FORCE_NB:386 · UL_RACH_FROM_DRACH:804 · TPU_RX_WIRE(DMA 0x0586):988

**calypso_c54x.c (37)** — FORCE_INTM_ONESHOT+AT_PC:1012 · FORCE_3FAD_KERNEL:2496 · FIX_3FCD:2633 · SHUNT_LEGIT(force d_fb_det):2720 · TRF_RXLEV+TARGET_RF:2738 · DEMOD_NOCLOBBER:3075 · AR2_FLOOR_DROP:4664 · FRAME_IT_LEVEL:4976 · FRAME_IT_PRIO:4988 · DSP_FRAME_VEC28(IRQ-LEVEL):5082 · CORR_BANK:5190 · FORCE_3FAE:5206 · FORCE_DISPATCH:5569 · FORCE_DP+_FROM:5595 · REDIR_LEGACY:12358 · REDIR7000:12375 · INITTAB:12394 · MASKROM_INIT:12420 · D247:12452 · REPOPULATE:12475 · TEST_3FCD:13790 · FIX_DPAGE_OFF:13821 · INIT_435B+SEED_52FD:13848 · MASKROM_GOLIVE:14008 · SEED_5AC8+_VAL:14302 · KEEP_IMR+_VAL:14467 · FORCE_GOLIVE:14496 · ISR_TO_8341:14521 · CORR_SETUP+_AR1/4/5:14551 · POKE_A4C7_ONCE:14592 · C54X_FORCE_IMR:14645 · FORCE_098:14680 · GOLIVE_TASKW:14709 · GOLIVE_REDIRECT(DSP_GOLIVE_BOOT):14936 · TINT0_PERINSN:16487 · TINT0_MASTER:16501 · VEC28_REMAP/FRAME_IT_NATIVE:17133

**calypso_dsp_shunt.c (26)** — SHUNT_LEGIT/NO_LEGIT(parapluies+constructeur setenv):86 · UL_PUB_IDLE:168 · L1_RESET_WIRE:202 · SHUNT_BURST_PERCMD(capture):230 · CANNED:363 · FRAME_IT_NATIVE:559 · TINT0_MASTER(frame-tick):576 · DECAN_PM+_MAV_REF/_RF_REF:621 · SHUNT_LEGIT(transport FB):696 · SHUNT_AGCH_EXPIRE+_TTL:800 · SHUNT_SI_ROT_MASK:829 · DECAN_TOA/SNR/ANGLE/PM:866 · INJECT_AGCH:1047 · REQREF_LAST_RACH/PERRA/REWRITE/ADJ:1130 · INJECT_SDCCH:1188 · INJECT_SACCH:1237 · DECAN(effet de bord REAL_FB):1591 · FB_IQ_DARAM+_BASE/_FCCH_ONLY:1660 · FB_IQ_MARKER:1691 · SHUNT_REAL_FB(calcul hôte):1755 · SHUNT_DSP_FB+_ENTRY/_BUDGET/_MAX/_SP:1849 · DSP_SHUNT(armement):2026 · SHUNT_BURST_PERCMD(miroir):2117 · SHUNT_FEED_SI:2152 · SHUNT_DL_INJECT:2221 · DSP_SHUNT(substitution):2255

**calypso_dsp_helper.c (12)** — SHUNT_BURST_FN/_OFS/_M1:180 · INJECT_FB:240 · INJECT_SB:299 · INJECT_ACD:367 · SHUNT_AGCH+_OFS/_TTL:405 · SHUNT_SDCCH+_RING/_OFS/_TTL/_MAXPRES:462 · SHUNT_SACCH:557 · SHUNT_SACCH_PAR+_OFS:580 · SHUNT_BCCH_SCHED+_OFS:646 · SHUNT_CANNED+SHUNT_DUAL_PAGE:716 · SHUNT_PM:760 · TRF_RXLEV+TARGET_RF:775

**calypso_arm2dsp.c (4)** — ARM2DSP+_TASKWORD/_TASKBIT:107 · ARM2DSP_BGEN+_A/_C/_VAL/_POLLPC/_ONESHOT:127 · ARM2DSP_CTRLSYS+_CELL/_VAL/_POLLPC:150 · ARM2DSP_CONT:281
**calypso_layer1.c (1)** — L1:51 · **l1ctl_sock.c (1)** — FORCE_FBSB/FORCE_AGCH:212 · **calypso_twl3025.c (1)** — TWL3025_AFC_HZ:94

## BUILD
`ninja -C build qemu-system-arm` → **OK**, link réussi, 0 warning nouveau ; deuxième passe = rien à recompiler (objets `calypso_{c54x,bsp,trx,dsp_shunt}.c.o` postérieurs aux sources). `./make-overlay.sh` → « 7 file(s) synced, 0 new, 320 unchanged ».

## COMPTE FINAL
`grep -rn "@BEQUILLE" hw/arm/calypso/ | wc -l` → **117** (105 dans les `.c`, 12 dans `doc/*.md`).

## PREUVE « COMMENTAIRES SEULEMENT »
Comparaison code-seul (commentaires strippés) avant/après sur les 9 fichiers : **8 IDENTIQUES à la ligne près**. Pour `calypso_c54x.c`, l'unique diff code est constitué des **8 hunks du dégatage de `CALYPSO_FIXES` faits par un tiers pendant ma passe** (`if (getenv("CALYPSO_FIXES"))` → `{`, et suppression de 7 `calypso_fix_enabled("FIX_…")`) — **pas de mon fait** ; mes 37 insertions n'ajoutent aucune ligne exécutable. À signaler : le fichier a changé sous moi (md5 `c36466ab` → `bf2a551a`) ; j'ai re-résolu tous les ancrages par recherche de motif, pas par numéro de ligne.

## BÉQUILLES NON ANNOTÉES (avec raison)
1. **Paramètres de béquilles** (`CORR_AR1/AR4/AR5`, `BGEN_A/_C/_VAL/_POLLPC/_ONESHOT`, `CTRLSYS_CELL/_VAL/_POLLPC`, `SEED5AC8_VAL`, `SEED_52FD`, `KEEP_IMR_VAL`, `FORCE_DP_FROM`, `POKE_DISPATCH`, `BSP_DISPATCH_FB_TGT/_NOIMR/_ONESHOT`, `FB_IQ_BASE/_FCCH_ONLY`, `SHUNT_DSP_FB_ENTRY/_BUDGET/_MAX/_SP`, `SHUNT_AGCH_OFS/_TTL`, `SDCCH_OFS/_TTL/_RING/_MAXPRES`, `SACCH_OFS`, `BCCH_OFS`, `BURST_OFS/_M1`, `REQREF_ADJ`) : **pas de marqueur propre — ils sont nommés dans le marqueur du gate parent**, lu au même endroit. Un marqueur par bloc, comme demandé.
2. **`FB_ENERGY`/`FB_CORR_ENTRY` (c54x:6355), `FB_STREAM` ×2 (c54x:1670, shunt:1731), `SHUNT_REAL_FB` ×2 (shunt:1605, helper:255), `FIX_BRINT0_UNMASK` (c54x:17209)** : **déjà annotés** avant ma passe, non dupliqués (le 3ᵉ site non annoté de `SHUNT_REAL_FB` a bien été ajouté, shunt:1755).
3. **`TRF_RXLEV` 3ᵉ site (`calypso_dsp_shunt.c`, écriture `api_ram[0x30-0x32]/[0x44-0x46]`)** et **`SHUNT_LEGIT` 2ᵉ site (branche FB du tick, ex-:858)** : situés **à l'intérieur** du bloc déjà couvert par `SHUNT_LEGIT (transport du resultat FB)`:696 — annoter à nouveau eût dupliqué.
4. **`TINT0_MASTER` 3ᵉ site (c54x, ancien :12847)** : code **mort** (`(void)_t0i;`) — signalé en NB du marqueur :16501 plutôt qu'annoté.
5. **`DSP_GOLIVE_BOOT` 2ᵉ site (`g_noforce`, inhibition VEC28-FORCE)** : mentionné en NB du marqueur VEC28_REMAP:17133 (même bloc).
6. **Classées autrement par les lots, donc hors périmètre** : SAS (`FIXES` et ses 11 sous-clés, `FIX_MVDM_OFF`, `FIX_PORTR`, `FIX_SFTL_RSBX`, `C54X_FIX_BC`, `C54X_BCTC_SM`) ; CONFIG (`SHUNT_NO_CANNED`, `SHUNT_NO_FAKE_FB/PM`, `SHUNT_NO_GRGSM`, `SHUNT_DRIVE_DSP`, `UL_ACU_OFS`, `DECAN_PM_MAV_REF/_RF_REF`, `LDK8_SHIFT16`, `IT_PUSH_XPC_ALWAYS`) ; MESURE (`BSP_INJECT_CANARY`, `FBDET_SENTINEL`) ; MORTES (`START_FN`, `NB_MAXDLY`, `BSP_BYPASS_BDLENA`, `DBG`, `ORCH`, `TINT0_PERIOD`, `C54X_CRASHPC`, `TRAP_CHECKPOINT`, `FIX_MVDM`, `CORRELATOR_TRACE`).