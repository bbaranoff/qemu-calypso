# TODO — état au 2026-07-28 (fin de journée)

> Refonte complète. L'ancien TODO était organisé autour de « RANK1..RANK5 », une numérotation
> devenue trompeuse : **RANK3 reposait sur une erreur de nomenclature d'interruption**, et
> plusieurs entrées « en cours » décrivaient des pistes désormais réfutées. Ce document repart de
> ce qui est **mesuré**.

---

## P0 — Le verrou natif : une erreur de nomenclature d'interruption

**Fait mesuré** : en mode natif sans béquille, le DSP tourne (100 M instructions), le BSP alimente,
mais le corrélateur n'est **jamais ordonnancé**. `CALYPSO_WATCH_9F00_RD = 0`.

**Racine identifiée** (`PLAN_APPLICATION.md`, 999 l.) — `calypso_c54x.h:117-124` déclare :

```c
#define C54X_INT_FRAME_VEC   19   /* INT3 */
#define C54X_INT_FRAME_BIT   3
/* et en commentaire : "Vec 21: BRINT0 (IMR bit 5)" */
```

Or SPRU131G donne **`vec 20 = BRINT0 = bit 4`** et **`vec 21 = BXINT0`** — l'interruption
d'**émission**. Toute la traque de « BRINT0 bit 5 » visait donc le mauvais vecteur ; le stub
`RETE ; NOP` trouvé en `vec21@0x00d4` est parfaitement normal, c'est BXINT0 dont le firmware
n'a pas l'usage. Le ROM arme bien le bit 4, en quatre sites (`0xbd40`, `0xbd62`, `0xc471`, `0xc498`),
et ne pose **jamais** `0x0020`.

Le vrai blocage : le modèle émet l'IT trame sur `vec 19 / bit 3`, un stub **et** un bit que le ROM
n'arme jamais ; pire, `calypso_trx.c:1445` se gate sur ce même bit 3 — condition **auto-fausse**,
l'interruption n'est même pas levée. `IFR=0x0028` ∩ `IMR=0x3200` = **0** : zéro vectorisation sur
tout le run.

- [ ] **C1 — `C54X_INT_FRAME_VEC/BIT` = 28 / 12** (deux lignes dans `calypso_c54x.h`).
      Signature attendue : `IRQ-LEVEL bit=12 vec=28`, puis `IMR-ARM → 0x3010 PC=0xbd40|0xc471`
      = **le ROM arme BRINT0 tout seul**.
      ⚠️ Poser `CALYPSO_INIT_435B_OFF=1` **avant** : `INIT-435B` est actif par défaut et polluera
      la mesure dès que C1 fera atteindre `0xa4e4`.
- [ ] **C2 — IT réception BSP `(21,5) → (20,4)`** : 4 sites dans `calypso_bsp.c` + `calypso_tpu.c:118`,
      **et** re-router TINT0 `(20,4) → (19,3)` (`calypso_c54x.c:16139/16163`), qui tape aujourd'hui
      dans l'ISR série.
- [ ] **C3 — modéliser BSPC0** (MMR `0x22`, hors du garde `addr < 0x20`). C3a = sonde seule d'abord.
- [ ] **C4 — `BANZ/BANZD`** : valeur testée = adresse effective, **restreint à MOD ≥ 0xC**
      (zéro site MOD ≥ 0xC dans le miroir page-0 → risque nul pour `SHUNT_LEGIT` par construction).
      Seul point de contact opcodes × verrou natif : `BANZ *AR6(+1)` en `PROM0 0xde5a`, **dans** la
      boucle de fond mesurée.
- [ ] **C5 — retirer les béquilles** fondées sur la fausse nomenclature.

**Ne pas appliquer** : `resolve_mmr` (2870 sites MMR, 100 % avec bit 7 = 0 → bénéfice nul),
`ADDM` avec flags C/OVA.

---

## P1 — Correctifs d'opcodes : le sas `CALYPSO_FIXES`

Protocole : poser **tous** les correctifs sûrs d'un coup → tester **sous charge maximale**
(camp + LU + SMS) → dès confirmation, **effacer LA CONDITION, pas le correctif**.

### Dégatés (inconditionnels, validés sur deux modes)
`0x1800/1A00/1C00/1E00` AND/OR/XOR/SUBC · `0x47` RPT Smem (écrivait `BRC` au lieu de `RC` : les
boucles ne tournaient qu'une fois) · `0x06/07` ADDC (+C) · `0x0E/0F` SUBB (−C) · `0x38/39` SQURA
(`T = Smem`) · `0x94/95` `ld Xmem,SHFT,dst` · `0x96` `bit` · `0xA2/A3` `sub Xmem,Ymem` ·
`0x85` `stl B,ASM` · `0x8D` `st TRN` · `0x4E/4F` `dst` ±2 · `0x40-43` bit 9 = SRC.

### Dans le sas — formellement corrects, **infirmés par la mesure**
- [ ] `FIX_LD_PARALLEL` (`0xA8-AF`) — longueur juste, mais n'exécute **que la partie `LD`** et
      laisse tomber le `MAC/MAS/MASR` parallèle. Dans un **corrélateur**, ça fait disparaître le
      `SHADOW-DADST`. À reprendre avec la vraie sémantique duale, pas une approximation.
- [ ] `FIX_STL_STH_SHFT` (`0x98-9B`) — appliquer `SHFT` change l'échelle de la valeur stockée.
      Écarté par précaution ; aucune charge confirmée contre lui à ce jour.
- [ ] `FIX_LDM_ZEROEXT` (`0x48/49`) — zéro-extension là où le MMR `T` est signé. À retester isolément.

### Reste à traiter (`RAPPORT_OPCODES.md`, 2369 l. — ~40 findings, ~15 de gravité 1)
- [ ] `0x62-0x67` (`mpy`/`mac`) et `0x78-0x7D` (`macp`/`macd`/`mvpd`/`mvdp`) : décodés en **1 mot**
      là où binutils en donne **2** — l'inverse des précédents.
- [ ] `0xC0-0xC7`, `0xDA`, `0xE0-0xE4`.
- [ ] ⚠️ Deux plages (`0x60-0x8F`, `0xC0-0xFF`) n'ont **pas** eu de passe de réfutation :
      vérification manuelle obligatoire contre `doc/opcodes/tic54x-opc.c`.

**Leçon** : un encodage confirmé par binutils **ne suffit pas**. Valider demande trois niveaux —
formel (binutils + SPRU172C), **grandeur physique** (une valeur mesurable reste-t-elle plausible ?),
et **chemin fonctionnel** (un traitement qui marchait marche-t-il encore ?). Trois correctifs sur
dix ont passé le premier niveau et échoué aux suivants.

---

## P2 — Chaîne de signal (acquis, ne pas défaire)

- [x] **3 gates BSP**, pas un : `calypso_bsp.c:474`, `:997`, et la **livraison** (`:1347`) qui ne se
      levait qu'avec `TPU_RX_WIRE`. Avant ce fix, `DARAM_FORCE=1` ouvrait deux verrous sur trois et
      **rien n'arrivait au DSP**.
- [x] **`CALYPSO_BSP_IQ_DECIM=4`** (`1` = régression mesurée : feed à 4 SPS) avec `DARAM_LEN=296`.
- [x] **Entrée démod = `data[0x4c00]`** confirmée par mesure : `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a`,
      stride 5 = polyphase 6 taps.
- [ ] **Sortie démod = `0x52ed` constant** — inchangé depuis le début. Le corrélateur lit du continu.
      À reprendre **après C1** : inutile d'analyser un étage qui n'est atteint que sous reroute.

---

## P3 — Chantiers parallèles

- [ ] Voix TCH/F — cf `VOIX_PLAN.md` (ASSIGNMENT COMMAND → FAILURE, le shunt ne présente pas le TCH DL).
- [ ] SMS — DONE en `SHUNT_LEGIT`, instable en `DSP,NO_CANNED`.
- [ ] `run.sh` unifié + `run_modules/` — **abandonné en cours de route** : le point d'entrée a été
      remplacé avant que les modules existent, ce qui a cassé le lancement. `run.sh` est restauré
      (2426 l.) ; le travail est conservé dans `run.sh.unifie_incomplet_2026-07-28`. À reprendre en
      écrivant **les modules d'abord**.

---

## Réfuté — ne pas re-soulever

| Affirmation | Pourquoi elle tombe |
|---|---|
| « verrou = BRINT0 / IMR bit 5 » (RANK3) | `vec 21 = BXINT0` (émission). BRINT0 = `vec 20 / bit 4`, armé par le ROM. |
| `golive-imr-shadow-435b` | `0xa582` = `IMR \|= shadow`, jamais destructeur (utilise `0x1A00 OR`, corrigé le 28/07). |
| `golive-gate-a53c-0810-bit15` | bit 15 = `B_TASK_ABORT`. |
| « IMR 0x52fd = masque de reset réel » | absent des 6 ROM, codé en dur `calypso_c54x.c:16631`. |
| `calypso_bsp.c:1077-1080` | `PROM1[0xFFD4]` n'est pas une table de vecteurs. |
| « `0x4c00` est gelé » | lecture hors fenêtre API RAM via QMP (signature : `peak = 0x8000`, 54 % de zéros). |
| « `Q == 0` » | conclu sur 2 mots de début de burst ; `zeros=0%` sur le burst entier. |
| « `0xa042` détruit le signal » | `0x2c00` est du scratch ; `0x9fd5` y dépose une table de coefficients constante. |
| « `WATCH_9F00_RD=0` = corrélateur affamé » | **conséquence**, pas cause : il n'est jamais ordonnancé. |

Confirmé au passage : `dsp-dpage-offset-bug` — `0x08E2 = d_dsp_state = 3`, donc le test
`data[0x08E2] & 2` (`:16862`) est **toujours vrai**.

---

## Règles de travail

1. **Lire le MANIFESTE, jamais la ligne de commande.** `CALYPSO_NATIVE_HELPED=1` repose
   `FB_CORR_ENTRY=0x9500`, `FB_ENERGY=1`, `FB_IQ_DARAM=1` : retirer une variable de la CLI ne la
   supprime pas.
2. **Un correctif à la fois pour la mesure**, mais **tous d'un coup dans le sas** — sinon on y passe
   des mois.
3. **Non-régression obligatoire** après chaque correctif :
   `CALYPSO_SHUNT_LEGIT=1 CALYPSO_SHUNT_NO_CANNED=1 CALYPSO_SHUNT_REAL_FB=1 ./start-clean.sh`
   → `BSIC=7` + `SYSTEM INFORMATION` + `LOCATION UPDATING ACCEPT`, **sans exception**.
4. **Ne jamais mesurer pendant qu'un agent écrit dans le code.** Une base `AR6` perdue a été
   attribuée à tort à trois correctifs successifs avant qu'on réalise que deux workflows
   modifiaient les `.c` sous les pieds de la mesure.
5. **Toute béquille porte `@BEQUILLE`** — `grep -rn "@BEQUILLE" hw/arm/calypso/*.c *.env`.
6. **Autorité opcodes** : `doc/opcodes/tic54x-opc.c` (binutils, le champ MOTS fait foi) >
   `spru172c.pdf` (sémantique) > le code > les tableaux de synthèse. **Ne jamais conclure depuis un
   commentaire** : plusieurs se sont avérés périmés.
7. **Une sonde se conçoit par sa condition de déclenchement**, pas par son adresse ; préférer un
   agrégat à un flux plafonné ; distinguer « varie dans l'espace » de « varie dans le temps » ;
   « pas de log » n'est jamais « pas d'événement » tant que la sonde n'est pas vérifiée vivante.
