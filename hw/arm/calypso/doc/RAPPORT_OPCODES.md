# RAPPORT — audit du decodeur c54x (2026-07-28)

> **Provenance et statut.** Ce document est le rendu BRUT des agents d'audit, recupere depuis le
> journal du workflow `c54x-opcode-audit` **apres que celui-ci a ete interrompu**. 10 agents sur 11
> avaient rendu. Il est publie tel quel parce que son contenu est trop volumineux pour etre
> reecrit sans perte, mais il doit etre lu avec trois reserves :
>
> 1. **Deux plages n'ont PAS eu leur passe de refutation** : `0x60-0x8F` et `0xC0-0xFF`. Leurs
>    findings sont donc des hypotheses d'un seul agent, non contradictoires. Ne rien appliquer
>    depuis ces deux sections sans verification manuelle contre `doc/opcodes/tic54x-opc.c`.
> 2. **L'audit a demarre avant la correction des tables du projet** (le 2026-07-28 : `0xF4..0xF7`
>    annonce « 2-mot » alors que binutils donne **1 mot** ; `0xEA` annonce « BANZ » alors que
>    `0xEA00/0xFE00` = `LD #k9,DP`). Tout finding s'appuyant sur l'ancienne table est suspect.
> 3. **Rien n'a ete applique.** L'audit etait en lecture seule sur le code.
>
> **Ordre d'autorite** : `doc/opcodes/tic54x-opc.c` (binutils — le champ MOTS fait foi) >
> `doc/spru172c.pdf` (semantique) > le code > les tableaux de synthese.

## Ce que l'audit etablit, en une phrase

Le decodeur confond systematiquement des instructions **1 mot** et **2 mots** sur au moins seize
familles (`0x62-0x67`, `0x78-0x7D`, `0x85`, `0x8D`, `0x94/0x95`, `0x96`, `0xA2/0xA3`, `0xA8/0xA9`,
`0xAC-0xAF`, `0xC0-0xC7`, `0xDA`, `0xE0-0xE4`). Une longueur fausse ne produit pas un resultat faux :
elle **desynchronise tout le decodage en aval**. C'est le patron de bug le plus grave, et c'est
aussi celui qui explique le mieux les symptomes observes en amont de cet audit — un registre
d'adresse charge avec une valeur d'echantillon, un etage de demodulation qui recopie du code
machine (`0xf495` = NOP, `0xf4eb` = RETE) dans son tampon de sortie, un DSP qui tourne
indefiniment dans une boucle de fond.

Le correctif applique le matin meme (`0x1800/1A00/1C00/1E00` = AND/OR/XOR/SUBC executes comme un
`LD`) n'etait donc pas un cas isole : c'etait le premier d'une serie.



---

# Carte de structure du decodeur

# Carte de structure du décodeur `calypso_c54x.c` (16865 l.)

Fichier audité : `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (conteneur `osmo-operator-1`), copie locale de travail `/root/.claude/jobs/26578783/tmp/calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`). Table de référence : `/root/.claude/jobs/26578783/tmp/tic54x_hi8_map.md`.

## 1. Points d'entrée du décodage

| L. | Élément |
|---|---|
| 5034 | `static int c54x_exec_one(C54xState *s)` — décodeur unique |
| 5037 | seul `return` avant décodage (vectorisation IRQ), aucune interception d'opcode |
| 5253 | `uint8_t hi4 = (op >> 12) & 0xF;` |
| 5254 | `uint8_t hi8 = (op >> 8) & 0xFF;` |
| 5255-5959 | uniquement des traceurs/sondes (aucun `return` de décodage) |
| **5960** | **`switch (hi4)` — UNIQUE grand switch de dispatch** |
| 10924 | `default: break;` du switch principal |
| 10928 | label `unimpl:` → `C54_LOG("UNIMPL …")` + `return consumed + lk_used` |

Le `switch (hi4)` nomme les 16 valeurs (0xF, 0xE, 0x6/0x7, 0x1, 0x0, 0x3, 0x2, 0x4, 0x5, 0x8/0x9, 0xA/0xB, 0xC/0xD) → **`default:` L10924 INATTEIGNABLE**. À l'intérieur, chaque `case` est une **chaîne de `if` sur `hi8` / masques** (pas un second switch), sauf 0x0, 0x1, 0x2, 0x5 qui calculent un `sub`.

## 2. Tableau des familles

| FAMILLE | LIGNES | MASQUE | SÉLECTEUR | SOUS-CAS NOMMÉS / ATTENDUS | DEFAULT SILENCIEUX ? |
|---|---|---|---|---|---|
| `case 0x0` ADD/SUB | 9434-9477 | `0xFE00` implicite | `sub = (op>>9)&7` L9446 | 8/8 (via `is_sub`/`is_unsigned`/`ts_shift`) | non (pas de default) — **mais sub 3 (ADDC) et 7 (SUBB) exécutés comme ADD/SUB simples, carry ignoré** |
| `case 0x1` LD/logique | 9320-9432 | `0xFE00` | `sub = (op>>9)&7` L9333 | **8/8** (fix 2026-07-28, L9366-9389) | `default:` L9390 devenu inatteignable |
| `case 0x2` MPY/MAC | 9509-9555 | `0xFF00` de fait | `sub = (op>>8)&0xF` L9512 | **8/16** (0,1,4,5,8,9,A,B) | **OUI** L9545 : les 8 restants (2,3,6,7,C,D,E,F) exécutés en `MAS` (`acc -= T*Smem`). Table l.30-37 : 0x22/23=mpyr, 0x26/27=squr, 0x2C/2D=mas, 0x2E/2F=masr → seuls 2 des 8 sont réellement des MAS |
| `case 0x3` MAC | 9480-9508 | `0xFE00` (uniquement `0x3800`) | **aucun** | **1/8** (SQURA 0x38/39 seul, L9520) | **OUI, blanket** L9527-9531 : tout 0x30-0x37 et 0x3A-0x3F exécuté en `acc += T*Smem`. Table l.38-48 : ld/mpya/ld/masa/bitt/maca/poly/macar/squrs/add — 0 correspondance |
| `case 0x4` | 9556-9653 | `op8` exact | `op8 = hi8` L9569 | **16/16** (0x40-0x4F) | `return` final L9652 inatteignable |
| `case 0x5` dual-long | 9654-9762 | `dl_hi>=0x50 && <=0x5F` L9672 | `dl_hi = (op>>8)&0xFF` | **16/16** | non ; **le bloc SFTA/SFTL L9733-9760 (`sub=(op>>9)&7`) est CODE MORT** (la garde `dl_hi` est toujours vraie sous `hi4==5`) |
| `case 0x6/0x7` | 8724-9319 | chaîne `0xFF00` puis 4 catch-all `0xF800` | — | 0x6D,0x76,0x77,0x7E,0x7F,0x60,0x61,0x68,0x69,0x6A,0x6B,0x6C,0x6E,0x6F,0x70,0x71,0x73,0x74,0x75 + 0x72 **gaté** = 20/32 | **OUI ×4** (voir §3) |
| ↳ sous-famille `0x6F00` | 9246-9302 | `0xFF00` + 2e mot | `sub = (op2>>5)&7` L9249 | **5/8** (0-4) | non (log ×10 L9296) |
| `case 0x8/0x9` | 9763-10279 | chaîne `hi8` + `0xFC00` | — | **32/32** (0x80-0x9F) | non ; `goto unimpl` L10278 inatteignable |
| `case 0xA/0xB` | 10280-10599 | chaîne `hi8` | — | **32/32** (0xA0-0xBF) | non ; `goto unimpl` L10598 inatteignable |
| `case 0xC/0xD` | 10600-10923 | chaîne `hi8` | — | **32/32** (0xC0-0xDF) | non ; `goto unimpl` L10922 inatteignable. 6 stubs NOP muets : C5 L10741, CD L10748, CE L10755, CF L10725, DD L10804, DE L10812 |
| `case 0xE` | 8475-8723 | `0xFC00`,`0xFE00`,`0xFFE0`,`hi8` | — | **16/16 nominalement**, mais E1/E2/E3 avalés par `0xFC00==0xE000` | **OUI** L8481 : blanket CMPS sur 0xE0-0xE3 (table l.128-131 : firs/lms/sqdst/abdst) |
| `case 0xF` | 5961-8474 | voir sous-tableau | — | 16/16 hi8 atteints | 5 fallbacks silencieux + 3 bruyants |

### Sous-familles de `case 0xF`

| Sous-famille | LIGNES | MASQUE | SÉLECTEUR | NOMMÉS / ATTENDUS | DEFAULT SILENCIEUX ? |
|---|---|---|---|---|---|
| XC 1/2, cond | 5970-6019 | `hi8==0xFD\|\|0xFF` | `cc = op&0xFF` puis `c3 = cc&7` L6005 | 6/8 sur `c3` | **OUI** L6012 `cond = true` |
| BACC/CALA/FBACC/RETE/FRET/IDLE | 6034, 6162, 6213, 6263, 6296 | opcodes exacts / `0xFCFF` | — | — | non |
| SFTC | 6760 | `0xFEFF == 0xF494` | — | — | non |
| `hi8==0xF4` | 6772-7062 | ~20 masques `0xFEFF`/`0xFCFF`/`0xFCE0`/`0xFFE0`/`0xFFF0` | — | — | **non — BRUYANT** L7058 `C54_LOG("F4xx unhandled…")` **non plafonné**, puis NOP |
| `hi8==0xF0\|0xF1` | 7064-7257 | `alu_op = (op>>4)&0xF` L7164 | 3 branches : `<=5`, `==6`, `>=8` | **alu_op 7 non couvert** → `goto unimpl` L7256 (bruyant) | sous-switch `sub6 = op&0xF` L7192 : **8/16**, `default: break` L7216 **SILENCIEUX** (no-op 2 mots) ; sous-switch `aop=(op>>5)&7` L7244 : 4/8, `default` L7251 inatteignable (bit7=1 force aop≥4) |
| `hi8==0xF2` | 7327-7433 | `0xFCFF`→`0xFCF0`→`0xFCE0` | `sub=op&7` L7338 / `subop=(op>>4)&0xF` L7368 / `sub=(op>>5)&7` L7395 | 7/8 (**sub 6 = MPY #lk manquant** → `result = src`, silencieux) ; 6/6 ; 4/4 | fallback L7429 **BRUYANT** (20 logs) + NOP |
| `hi8==0xF3` | 7662-7813 | idem F2 | idem | idem F2 | fallback L7809 **BRUYANT** (20 logs) + NOP |
| `hi8==0xF5` | 8027-8042 | `0xFFF0 == 0xF5B0` (SSBX ST0) | — | 1 seul cas nommé | **OUI, majeur** L8038-8041 : tout autre 0xF5xx → `RPT #k8` (`rpt_count = op&0xFF`, `pc+=1`, `return 0`) sans aucun log |
| `hi8==0xF6` | 7815-8025 | `sub=(op>>4)&0xF` L7816 + opcodes exacts | `sub` | 3 (`0x2`,`0x6`,`0xB`) + `sub>=8` blanket + 6 opcodes exacts | **OUI ×2** : L8008 `sub>=0x8` → MVDD Xmem,Ymem blanket (champs AR 3 bits, incohérent avec le reste du fichier qui utilise 2 bits + 2) ; L8023 `return` NOP muet pour sub 0,1,3,4,5,7 |
| `hi8==0xF7` | 8047-8131 | `0xFFF0==0xF7B0` puis `sub=(op>>4)&0xF` L8072 | `sub` | **16/16** (0-D nommés, E/F → NOP explicite L8085) | non |
| `hi8==0xF8` | 7469-7640 | `sub=(op>>4)&0xF` L7470 | `sub` | **6/16** (0,1 BANZ ; 2,3 BC « dialecte » ; 4,5 BC acc ; ≥C CALL) | **OUI, majeur** L7635-7639 : sub 6,7,8,9,A,B → `RPT Smem` muet. Table l.150 : tout 0xF800/0xFF00 = `bc`. Sous-switch `cc&7` L7598 : 6/8, `default take=false` L7605 silencieux |
| `hi8==0xF9` | 8133-8190 | `0xFF00` + bit 7 | — | CC / FCALL | non |
| `hi8==0xFA` | 8192-8239 | `0xFF00` + bit 7 ; `fa_sub=(op>>4)&0xF` L8222 | `fa_sub` | 2/16 (0x2,0x3) et **conditionnés à `g_prev_op & 0xFE00 == 0x6000`** | **OUI** L8237-8238 : « NEAR FAxx fallback » = **branche inconditionnelle** `s->pc = op2` quelle que soit la condition |
| `hi8==0xFB` | 8244-8287 | `0xFF00` + bit 7 | `c54x_cond_true()` | — | non |
| `hi8==0xFC` | 8289-8391 | `0xFF00` | `test = cc&7` L8300 | 6/8 | **OUI** L8312 `cond = true` (RET pris à tort) |
| `hi8==0xFE` | 8401-8471 | `0xFF00` | `test = cc&7` L8412 | 6/8 | **OUI** L8424 `cond = true` |

## 3. Liste exhaustive des sorties terminales du décodage

### `goto unimpl` (7) — tous BRUYANTS (log `UNIMPL` L10930, plafonné 200 puis sur changement d'opcode)

| L. | Contexte | Atteignable ? |
|---|---|---|
| 7256 | fin `hi8==0xF0/0xF1` (alu_op == 7) | **OUI** |
| 8473 | fin `case 0xF` | non (F0-FF tous captés) |
| 8722 | fin `case 0xE` | non (E0-EF tous captés) |
| 9318 | fin `case 0x6/0x7` | non (4 catch-all `0xF800` couvrent 0x60-0x7F) |
| 10278 | fin `case 0x8/0x9` | non |
| 10598 | fin `case 0xA/0xB` | non |
| 10922 | fin `case 0xC/0xD` | non |

### `default:` du décodage (12)

| L. | Sélecteur | Nommés/Attendus | Nature |
|---|---|---|---|
| 6012 | `c3 = cc&7` (XC) | 6/8 | **SILENCIEUX** — `cond = true`, exécute |
| 7216 | `sub6 = op&0xF` (F0/F1) | 8/16 | **SILENCIEUX** — no-op, consomme 2 mots |
| 7251 | `aop = (op>>5)&7` | 4/8 | silencieux mais inatteignable |
| 7605 | `cc&7` (F8 sub 4/5) | 6/8 | **SILENCIEUX** — `take = false` |
| 8312 | `test = cc&7` (FC/RC) | 6/8 | **SILENCIEUX** — `cond = true` (retour effectué) |
| 8424 | `test = cc&7` (FE/RETD) | 6/8 | **SILENCIEUX** — `cond = true` |
| 8626 | `sub = op&0xFF` (E1xx) | 11/256 | silencieux **mais bloc entier mort** (cf. §4) |
| 9296 | `sub = (op2>>5)&7` (0x6F00) | 5/8 | **BRUYANT** (log ×10) puis no-op |
| 9390 | `sub = (op>>9)&7` (case 0x1) | 8/8 | inatteignable |
| 9545 | `sub = (op>>8)&0xF` (case 0x2) | 8/16 | **SILENCIEUX** — exécute MAS |
| 9883 | `cond = op&0x0F` (SACCD) | 8/16 | **SILENCIEUX** — `take = 0` |
| 10924 | `hi4` | 16/16 | inatteignable → `unimpl` |

### `else`/fallthrough terminaux silencieux (le patron n°1 — exécutent un comportement plausible sans trace)

| L. | Condition | Comportement | Plage réellement avalée |
|---|---|---|---|
| 8481 | `(op & 0xFC00) == 0xE000` | CMPS src,Smem | **0xE0,0xE1,0xE2,0xE3** (table : firs/lms/sqdst/abdst ; CMPS = 0x8E00) |
| 8038 | fin `hi8==0xF5` | `RPT #k8` + `pc+=1; return 0` | tout 0xF5xx hors 0xF5Bx |
| 7635 | fin `hi8==0xF8` | `RPT Smem` | 0xF86x-0xF8Bx |
| 8008 | `sub >= 0x8` (F6) | MVDD Xmem,Ymem, AR sur 3 bits | 0xF68x-0xF6Fx non exacts |
| 8023 | fin `hi8==0xF6` | NOP | sub 0,1,3,4,5,7 |
| 8237 | fin `hi8==0xFA` | branche **toujours prise** | tout 0xFA00-0xFA7F non 0xFA2x/0xFA3x |
| 9045 | `(op & 0xF800) == 0x7000` | `STL src,Smem` | 0x72xx si `CALYPSO_FIX_MVDM_OFF` (gate L9026) |
| 9053 | `(op & 0xF800) == 0x7800` | `STH src,Smem` | **0x78-0x7D** (table : macp/macd/mvpd/mvdp) |
| 9131 | `(op & 0xF800) == 0x6000` | `LD Smem,dst` | **0x62-0x67** (table : mpy/mac) |
| 9304 | `(op & 0xF800) == 0x6800` | `LD Smem,T` | déclaré mort par le commentaire ; 0x6D est bien capté avant (L8767) |
| 9527 | `case 0x3` corps | `acc += T*Smem` | **0x30-0x37, 0x3A-0x3F** |
| 10725/10741/10748/10755/10804/10812 | CF, C5, CD, CE, DD, DE | `return 1` (NOP) | délibéré et documenté |

### Fallbacks BRUYANTS (log + NOP)

L7058 (`F4xx unhandled`, **log non plafonné**), L7429 (`F2xx unmapped`, ×20), L7809 (`F3xx unmapped`, ×20), L9296 (`0x6F unknown sub`, ×10).

## 4. Trous de couverture et code mort

**Aucun hi8 0x00-0xFF n'est totalement sans chemin** : tous atteignent au moins un handler, un catch-all ou un `unimpl` bruyant. Les « trous » réels sont donc de type *ombrage* (handler écrit mais jamais atteint) et de type *avalement* (catch-all qui vole une famille) :

Handlers **INATTEIGNABLES** (ombrés par un test antérieur) :

| L. | Handler | Ombré par |
|---|---|---|
| 8612-8631 | `hi8==0xE1` (CMPL/NEG/SAT/ABS/ROR/ROL) | L8481 `(op&0xFC00)==0xE000` |
| 8645-8654 | `hi8==0xEB` (RPTB) | L8502 `(op&0xFE00)==0xEA00` |
| 9733-9760 | SFTA/SFTL `sub=(op>>9)&7` (case 0x5) | L9672 `dl_hi>=0x50 && <=0x5F` toujours vrai |
| 10090-10124 | PORTR 0x9E | `if (0 && hi8 == 0x8F)` L10045 — désactivé en dur |
| 10125-10140 | PORTW `hi8==0x9F` | L9859 `(op&0xFC00)==0x9C00` |
| 10219-10225 | `hi8==0x89` | L9953 `hi8==0x88\|\|0x89` |
| 10227-10232 | `hi8==0x8B` | L9968 `hi8==0x8B` |
| 10263-10269 | `hi8==0x91` | L9770 `hi8==0x90..0x93` |
| 10518-10537 | `hi8==0xA5` | L10297 (bloc dual-op A4-A7/B0-B7) |
| 9932-9939 | `if (0 && hi8 == 0x8A)` | désactivé en dur |
| 9035-9042 | 2e handler `0x7300` (MVMD gaté) | L8918 `(op&0xFF00)==0x7300` |

Familles **avalées** par un catch-all plus large que la table (à instruire par les agents sémantique) :
`0x30-0x37`+`0x3A-0x3F` (→ MAC), `0x62-0x67` (→ LD), `0x78-0x7D` (→ STH), `0xE0-0xE3` (→ CMPS), `0xF5xx` hors F5Bx (→ RPT), `0xF86x-0xF8Bx` (→ RPT Smem), `0xFAxx` NEAR (→ branche inconditionnelle), `0x9C/0x9D` (→ SACCD au lieu de STRCD/SRCCD), `0x2x` moitié (→ MAS).

Sous-cas **manquants dans un switch dont le sélecteur en admet plus** (patron n°2), classés par gravité de la valeur produite :
`case 0x3` 1/8 · `hi8==0xF8` 6/16 · `case 0x2` 8/16 · `hi8==0xF0/F1` sub6 8/16 · SACCD cond 8/16 · `0x6F00` sub 5/8 · `hi8==0xF2/F3` sub 7/8 (MPY #lk absent) · conditions `cc&7` 6/8 en 4 endroits (XC L6012, F8 L7605, FC L8312, FE L8424).

---

# Table de reference consolidee (methode de lecture des encodages)

## Clé de lecture (méthode) — vérifiable

`spru.txt` contient les pages « Opcode » du chapitre 4 sous forme d'une chaîne de 16 caractères précédée de l'en‑tête `0123456789101112131415`. **L'ordre des cellules extraites n'est pas l'ordre des bits.** Permutation calibrée puis vérifiée sur 12 encodages connus (POPM=0x8A, POPD=0x8B, PSHM=0x4A, PSHD=0x4B, STM=0x77, LDM=0x48, NOP=0xF495, TRAP=0xF4C0, RESET=0xF7E0, MVMM=0xE7, BITF=0x61, ANDM=0x68) :

```
e[0]→b7  e[1]→b12 e[2]→b13 e[3]→b14 e[4]→b15 e[5]→b8  e[6]→b9  e[7]→b10
e[8]→b11 e[9]→b0  e[10]→b5 e[11]→b6 e[12]→b1 e[13]→b2 e[14]→b3 e[15]→b4
```
Exemple de contrôle : SUBC `spru.txt@580544` brut `I1000S111AAAAAAA` → `0001111S IAAAAAAA` = **0x1E00/0xFE00** (confirme le bug 0x1800‑0x1E00 corrigé le 27/07). Les instructions 2 mots ont le mot 2 collé au mot 1 **sans nouvel en‑tête** (ex. ADD `spru.txt` page 4‑4 : `I01101111AAAAAAA` + `00000DS11T00FIHS`).

Légende encodage : `S`=src acc (b9 ou b8 selon famille), `D`=dst acc, `I`+`AAAAAAA`=Smem (b7=0 direct DP / b7=1 indirect), `X/Y`=Xmem/Ymem, `Z`=bit delayed, `R`=bit round, `C`=cond, `K/k`=immédiat, `SHFT`=4 bits non signés, `SHIFT`=5 bits signés.

---

## TABLE DE RÉFÉRENCE CONSOLIDÉE 0x00–0xFF

`map:Lnn` = ligne de `doc/opcodes/tic54x_hi8_map.md`. `p.4‑x` = page SPRU172C (colonne « Page » des tables 2‑1…2‑26, `spru.txt@36605‑58000`).

| HI8 | MNÉMONIQUE (syntaxe) | ENCODAGE / MASQUE | EXECUTION (SPRU172C) | SOURCE |
|---|---|---|---|---|
| 00‑01 | `ADD Smem, src` | `0000000S IAAAAAAA` 0x0000/0xFE00 | `src = src + Smem` | map:L14 ; p.4‑4 ; Tab.2‑1 |
| 02‑03 | `ADDS Smem, src` | `0000001S IAAAAAAA` 0x0200/0xFE00 | `src = src + unsSmem` (zéro‑ext.) | map:L15 ; p.4‑10 |
| 04‑05 | `ADD Smem, TS, src` | `0000010S IAAAAAAA` 0x0400/0xFE00 | `src = src + Smem << TS` (TS = T[5:0]) | map:L16 ; p.4‑4 |
| 06‑07 | `ADDC Smem, src` | `0000011S IAAAAAAA` 0x0600/0xFE00 | `src = src + Smem + C` | map:L17 ; p.4‑8 |
| 08‑09 | `SUB Smem, src` | `0000100S IAAAAAAA` 0x0800/0xFE00 | `src = src − Smem` | map:L18 ; p.4‑187 |
| 0A‑0B | `SUBS Smem, src` | `0000101S IAAAAAAA` 0x0A00/0xFE00 | `src = src − unsSmem` | map:L19 ; p.4‑194 |
| 0C‑0D | `SUB Smem, TS, src` | `0000110S IAAAAAAA` 0x0C00/0xFE00 | `src = src − Smem << TS` | map:L20 ; p.4‑187 |
| 0E‑0F | `SUBB Smem, src` | `0000111S IAAAAAAA` 0x0E00/0xFE00 | `src = src − Smem − C` | map:L21 ; p.4‑191 |
| 10‑11 | `LD Smem, dst` | `0001000D IAAAAAAA` 0x1000/0xFE00 | `dst = Smem` | map:L22 ; p.4‑66 |
| 12‑13 | `LDU Smem, dst` | `0001001D IAAAAAAA` 0x1200/0xFE00 | `dst = unsSmem` (**zéro‑ext.**) | map:L23 ; p.4‑79 |
| 14‑15 | `LD Smem, TS, dst` | `0001010D IAAAAAAA` 0x1400/0xFE00 | `dst = Smem << TS` | map:L24 ; p.4‑66 |
| 16‑17 | `LDR Smem, dst` | `0001011D IAAAAAAA` 0x1600/0xFE00 | `dst = rnd(Smem)` | map:L25 ; p.4‑78 |
| 18‑19 | `AND Smem, src` | `0001100S IAAAAAAA` 0x1800/0xFE00 | `src = src & Smem` (Smem **zéro‑étendu** sur 40 b) | map:L26 ; p.4‑11 |
| 1A‑1B | `OR Smem, src` | `0001101S IAAAAAAA` 0x1A00/0xFE00 | `src = src \| Smem` | map:L27 ; p.4‑123 |
| 1C‑1D | `XOR Smem, src` | `0001110S IAAAAAAA` 0x1C00/0xFE00 | `src = src ^ Smem` | map:L28 ; p.4‑201 |
| 1E‑1F | `SUBC Smem, src` | `0001111S IAAAAAAA` 0x1E00/0xFE00 | `ALU = src − Smem<<15` ; si `ALU ≥ 0` → `src = (ALU<<1)+1` sinon `src = src<<1`. SXM ; affecte C,OV | map:L29 ; p.4‑192 ; `spru.txt@580544` |
| 20‑23 | `MPY[R] Smem, dst` | `001000RD IAAAAAAA` 0x2000/0xFC00 (R=b9) | `dst = T * Smem` / `dst = rnd(T*Smem)` | map:L30‑31 ; p.4‑101 |
| 24‑25 | `MPYU Smem, dst` | `0010010D IAAAAAAA` 0x2400/0xFE00 | `dst = unsT * unsSmem` | map:L32 ; p.4‑106 |
| 26‑27 | `SQUR Smem, dst` | `0010011D IAAAAAAA` 0x2600/0xFE00 | `dst = Smem*Smem ; T = Smem` | map:L33 ; p.4‑161 |
| 28‑2B | `MAC[R] Smem, src` | `001010RS IAAAAAAA` 0x2800/0xFC00 | `src = src + T*Smem` / `rnd` | map:L34‑35 ; p.4‑82 |
| 2C‑2F | `MAS[R] Smem, src` | `001011RS IAAAAAAA` 0x2C00/0xFC00 | `src = src − T*Smem` / `rnd` | map:L36‑37 ; p.4‑94 |
| 30 | `LD Smem, T` | `00110000 IAAAAAAA` 0x3000/0xFF00 | `T = Smem` | map:L38 ; p.4‑70 |
| 31 | `MPYA Smem` | `00110001 IAAAAAAA` 0x3100/0xFF00 | `B = Smem * A[32:16] ; T = Smem` | map:L39 ; p.4‑104 |
| 32 | `LD Smem, ASM` | `00110010 IAAAAAAA` 0x3200/0xFF00 | `ASM = Smem[4:0]` | map:L40 ; p.4‑70 |
| 33 | `MASA Smem [,B]` | `00110011 IAAAAAAA` 0x3300/0xFF00 | `B = B − Smem*A[32:16] ; T = Smem` | map:L41 ; p.4‑97 |
| 34 | `BITT Smem` | `00110100 IAAAAAAA` 0x3400/0xFF00 | `TC = Smem[15 − T[3:0]]` | map:L42 ; p.4‑23 |
| 35 / 37 | `MACA[R] Smem [,B]` | `001101R1 IAAAAAAA` 0x3500/0xFD00 (**b8=1 fixe**) | `B = B + Smem*A[32:16] ; T = Smem` (`rnd` si R) | map:L43,L45 ; p.4‑85 |
| 36 | `POLY Smem` | `00110110 IAAAAAAA` 0x3600/0xFF00 | `B = Smem<<16 ; A = rnd(A[32:16]*T + B)` | map:L44 ; p.4‑126 |
| 38‑39 | `SQURA Smem, src` | `0011100S IAAAAAAA` 0x3800/0xFE00 | `src = src + Smem*Smem ; T = Smem` | map:L46 ; p.4‑163 |
| 3A‑3B | `SQURS Smem, src` | `0011101S IAAAAAAA` 0x3A00/0xFE00 | `src = src − Smem*Smem ; T = Smem` | map:L47 ; p.4‑164 |
| 3C‑3F | `ADD Smem,16,src[,dst]` | `001111SD IAAAAAAA` 0x3C00/0xFC00 | `dst = src + Smem << 16` | map:L48 ; p.4‑4 |
| 40‑43 | `SUB Smem,16,src[,dst]` | `010000SD IAAAAAAA` 0x4000/0xFC00 | `dst = src − Smem << 16` | map:L49 ; p.4‑187 |
| 44‑45 | `LD Smem, 16, dst` | `0100010D IAAAAAAA` 0x4400/0xFE00 | `dst = Smem << 16` | map:L50 ; p.4‑66 |
| 46 | `LD Smem, DP` | `01000110 IAAAAAAA` 0x4600/0xFF00 | `DP = Smem[8:0]` | map:L51 ; p.4‑70 |
| 47 | `RPT Smem` | `01000111 IAAAAAAA` 0x4700/0xFF00 | `RC = Smem` (répète l'instr. suivante RC+1 fois) | map:L52 ; p.4‑146 |
| 48‑49 | `LDM MMR, dst` | `0100100D IAAAAAAA` 0x4800/0xFE00 | `dst = MMR` (zéro‑ext.) | map:L53 ; p.4‑73 |
| 4A | `PSHM MMR` | `01001010 IAAAAAAA` 0x4A00/0xFF00 | `−−SP ; TOS = MMR` | map:L54 ; p.4‑132 |
| 4B | `PSHD Smem` | `01001011 IAAAAAAA` 0x4B00/0xFF00 | `−−SP ; TOS = Smem` | map:L55 ; p.4‑131 |
| 4C | `LTD Smem` | `01001100 IAAAAAAA` 0x4C00/0xFF00 | `T = Smem ; Smem+1 = Smem` | map:L56 ; p.4‑81 |
| 4D | `DELAY Smem` | `01001101 IAAAAAAA` 0x4D00/0xFF00 | `Smem+1 = Smem` | map:L57 ; p.4‑41 |
| 4E‑4F | `DST src, Lmem` | `0100111S IAAAAAAA` 0x4E00/0xFE00 | `Lmem = src` (32 b, 2 cycles) | map:L58 ; p.4‑47 |
| 50‑53 | `DADD Lmem, src[,dst]` | `010100SD IAAAAAAA` 0x5000/0xFC00 | C16=0 : `dst = Lmem + src` ; C16=1 : double 16 b | map:L59 ; p.4‑37 |
| 54‑55 | `DSUB Lmem, src` | `0101010S IAAAAAAA` 0x5400/0xFE00 | C16=0 : `src = src − Lmem` | map:L60 ; p.4‑48 |
| 56‑57 | `DLD Lmem, dst` | `0101011D IAAAAAAA` 0x5600/0xFE00 | `dst = Lmem` | map:L61 ; p.4‑42 |
| 58‑59 | `DRSUB Lmem, src` | `0101100S IAAAAAAA` 0x5800/0xFE00 | C16=0 : `src = Lmem − src` | map:L62 ; p.4‑43 |
| 5A‑5B | `DADST Lmem, dst` | `0101101D IAAAAAAA` 0x5A00/0xFE00 | C16=0 : `dst = Lmem + (T<<16) + T` | map:L63 ; p.4‑39 |
| 5C‑5D | `DSUBT Lmem, dst` | `0101110D IAAAAAAA` 0x5C00/0xFE00 | C16=0 : `dst = Lmem − (T<<16) + T` | map:L64 ; p.4‑50 |
| 5E‑5F | `DSADT Lmem, dst` | `0101111D IAAAAAAA` 0x5E00/0xFE00 | C16=0 : `dst = Lmem − (T<<16) + T` | map:L65 ; p.4‑45 |
| 60 | `CMPM Smem, #lk` **(2 mots)** | `01100000 IAAAAAAA` + lk ; 0x6000/0xFF00 | `TC = (Smem == lk)` | map:L66 ; p.4‑33 |
| 61 | `BITF Smem, #lk` **(2 mots)** | `01100001 IAAAAAAA` + lk ; 0x6100/0xFF00 | `TC = ((Smem & lk) ≠ 0)` | map:L67 ; p.4‑22 |
| 62‑63 | `MPY Smem,#lk,dst` **(2 mots)** | `0110001D IAAAAAAA` + lk ; 0x6200/0xFE00 | `dst = Smem * lk ; T = Smem` | map:L68 ; p.4‑101 |
| 64‑67 | `MAC Smem,#lk,src[,dst]` **(2 mots)** | `011001SD IAAAAAAA` + lk ; 0x6400/0xFC00 | `dst = src + Smem*lk ; T = Smem` | map:L69 ; p.4‑82 |
| 68 | `ANDM #lk, Smem` **(2 mots)** | `01101000 IAAAAAAA` + lk ; 0x6800/0xFF00 | `Smem = Smem & lk` — Status Bits : **None** | map:L70 ; p.4‑13 |
| 69 | `ORM #lk, Smem` **(2 mots)** | `01101001 IAAAAAAA` + lk ; 0x6900/0xFF00 | `Smem = Smem \| lk` | map:L71 ; p.4‑125 |
| 6A | `XORM #lk, Smem` **(2 mots)** | `01101010 IAAAAAAA` + lk ; 0x6A00/0xFF00 | `Smem = Smem ^ lk` | map:L72 ; p.4‑203 |
| 6B | `ADDM #lk, Smem` **(2 mots)** | `01101011 IAAAAAAA` + lk ; 0x6B00/0xFF00 | `Smem = Smem + lk` | map:L73 ; p.4‑9 |
| 6C | `BANZ pmad, Sind` **(2 mots)** | `01101100 IAAAAAAA` + pmad ; 0x6C00/0xFF00 | `si ARx ≠ 0 → PC = pmad, sinon PC += 2`. Status Bits : **None**. Le mode indirect est appliqué **avant** le test | map:L74 ; p.4‑16 |
| 6D | `MAR Smem` **(1 mot)** | `01101101 IAAAAAAA` 0x6D00/0xFF00 | CMPT=0 : modifie ARx ; CMPT=1 & ARx≠AR0 : modifie ARx, `ARP = x` ; CMPT=1 & ARx=AR0 : modifie AR[ARP] | map:L75 ; p.4‑92 |
| 6E | `BANZD pmad, Sind` **(2 mots + 2 slots)** | `01101110 IAAAAAAA` + pmad ; 0x6E00/0xFF00 | idem BANZ, retardé | map:L76 ; p.4‑16 |
| 6F | **ext.** `ADD/SUB/LD/STH/STL Smem,SHIFT,…` **(2 mots)** | `01101111 IAAAAAAA` + mot2 (voir §6F) ; 0x6F00/0xFF00 | voir sous‑table §6F | map:L77 ; p.4‑4/187/66/169/172 |
| 70 | `MVKD dmad, Smem` **(2 mots)** | `01110000 IAAAAAAA` + dmad ; 0x7000/0xFF00 | `Smem = dmad` | map:L78 ; p.4‑113 |
| 71 | `MVDK Smem, dmad` **(2 mots)** | `01110001 IAAAAAAA` + dmad ; 0x7100/0xFF00 | `dmad = Smem` | map:L79 ; p.4‑108 |
| 72 | `MVDM dmad, MMR` **(2 mots)** | `01110010 IAAAAAAA` + dmad ; 0x7200/0xFF00 | `MMR = dmad` | map:L80 ; p.4‑110 |
| 73 | `MVMD MMR, dmad` **(2 mots)** | `01110011 IAAAAAAA` + dmad ; 0x7300/0xFF00 | `dmad = MMR` | map:L81 ; p.4‑115 |
| 74 | `PORTR PA, Smem` **(2 mots)** | `01110100 IAAAAAAA` + PA ; 0x7400/0xFF00 | `Smem = PA` | map:L82 ; p.4‑129 |
| 75 | `PORTW Smem, PA` **(2 mots)** | `01110101 IAAAAAAA` + PA ; 0x7500/0xFF00 | `PA = Smem` | map:L83 ; p.4‑130 |
| 76 | `ST #lk, Smem` **(2 mots)** | `01110110 IAAAAAAA` + lk ; 0x7600/0xFF00 | `Smem = lk` | map:L84 ; p.4‑167 |
| 77 | `STM #lk, MMR` **(2 mots)** | `01110111 IAAAAAAA` + lk ; 0x7700/0xFF00 | `MMR = lk` | map:L85 ; p.4‑176 |
| 78‑79 | `MACP Smem,pmad,src` **(2 mots)** | `0111100S IAAAAAAA` + pmad ; 0x7800/0xFE00 | `src = src + Smem*pmad ; T = Smem` | map:L86 ; p.4‑89 |
| 7A‑7B | `MACD Smem,pmad,src` **(2 mots)** | `0111101S IAAAAAAA` + pmad ; 0x7A00/0xFE00 | `src += Smem*pmad ; T = Smem ; Smem+1 = Smem` | map:L87 ; p.4‑87 |
| 7C | `MVPD pmad, Smem` **(2 mots)** | `01111100 IAAAAAAA` + pmad ; 0x7C00/0xFF00 | `Smem = Pmem[pmad]` | map:L88 ; p.4‑117 |
| 7D | `MVDP Smem, pmad` **(2 mots)** | `01111101 IAAAAAAA` + pmad ; 0x7D00/0xFF00 | `Pmem[pmad] = Smem` | map:L89 ; p.4‑111 |
| 7E | `READA Smem` | `01111110 IAAAAAAA` 0x7E00/0xFF00 | `Smem = Pmem[A]` | map:L90 ; p.4‑136 |
| 7F | `WRITA Smem` | `01111111 IAAAAAAA` 0x7F00/0xFF00 | `Pmem[A] = Smem` | map:L91 ; p.4‑196 |
| 80‑81 | `STL src, Smem` | `1000000S IAAAAAAA` 0x8000/0xFE00 | `Smem = src[15:0]` | map:L92 ; p.4‑172 |
| 82‑83 | `STH src, Smem` | `1000001S IAAAAAAA` 0x8200/0xFE00 | `Smem = src >> 16` | map:L93 ; p.4‑169 |
| 84‑85 | `STL src, ASM, Smem` | `1000010S IAAAAAAA` 0x8400/0xFE00 | `Smem = src << ASM` | map:L94 ; p.4‑172 |
| 86‑87 | `STH src, ASM, Smem` | `1000011S IAAAAAAA` 0x8600/0xFE00 | `Smem = (src << ASM) >> 16` | map:L95 ; p.4‑169 |
| 88‑89 | `STLM src, MMR` | `1000100S IAAAAAAA` 0x8800/0xFE00 | `MMR = src[15:0]` | map:L96 ; p.4‑175 |
| 8A | `POPM MMR` | `10001010 IAAAAAAA` 0x8A00/0xFF00 | `MMR = TOS ; ++SP` | map:L97 ; p.4‑128 |
| 8B | `POPD Smem` | `10001011 IAAAAAAA` 0x8B00/0xFF00 | `Smem = TOS ; ++SP` | map:L98 ; p.4‑127 |
| 8C | `ST T, Smem` | `10001100 IAAAAAAA` 0x8C00/0xFF00 | `Smem = T` | map:L99 ; p.4‑167 |
| 8D | `ST TRN, Smem` | `10001101 IAAAAAAA` 0x8D00/0xFF00 | `Smem = TRN` | map:L100 ; p.4‑167 |
| 8E‑8F | `CMPS src, Smem` | `1000111S IAAAAAAA` 0x8E00/0xFE00 | `si src[31:16] > src[15:0] → Smem = src[31:16] ; sinon Smem = src[15:0]` (met à jour TRN/TC) | map:L101 ; p.4‑35 |
| 90‑91 | `ADD Xmem, SHFT, src` | `1001000S XXXXSHFT` 0x9000/0xFE00 | `src = src + Xmem << SHFT` (SHFT 0..15) | map:L102 ; p.4‑4 |
| 92‑93 | `SUB Xmem, SHFT, src` | `1001001S XXXXSHFT` 0x9200/0xFE00 | `src = src − Xmem << SHFT` | map:L103 ; p.4‑187 |
| 94‑95 | `LD Xmem, SHFT, dst` | `1001010D XXXXSHFT` 0x9400/0xFE00 | `dst = Xmem << SHFT` | map:L104 ; p.4‑66 |
| 96 | `BIT Xmem, BITC` | `10010110 XXXXBITC` 0x9600/0xFF00 | `TC = Xmem[15 − BITC]` | map:L105 ; p.4‑21 |
| **97** | **— non assigné —** | — | — | absent du manuel et de map |
| 98‑99 | `STL src, SHFT, Xmem` | `1001100S XXXXSHFT` 0x9800/0xFE00 | `Xmem = src << SHFT` | map:L106 ; p.4‑172 |
| 9A‑9B | `STH src, SHFT, Xmem` | `1001101S XXXXSHFT` 0x9A00/0xFE00 | `Xmem = (src << SHFT) >> 16` | map:L107 ; p.4‑169 |
| 9C | `STRCD Xmem, cond` | `10011100 XXXXCOND` 0x9C00/0xFF00 | `si cond → Xmem = T` | map:L108 ; p.4‑186 |
| 9D | `SRCCD Xmem, cond` | `10011101 XXXXCOND` 0x9D00/0xFF00 | `si cond → Xmem = BRC` | map:L109 ; p.4‑165 |
| 9E‑9F | `SACCD src, Xmem, cond` | `1001111S XXXXCOND` 0x9E00/0xFE00 | `si cond → Xmem = (src << ASM) >> 16` | map:L110 ; p.4‑152 |
| A0‑A1 | `ADD Xmem, Ymem, dst` | `1010000D XXXXYYYY` 0xA000/0xFE00 | `dst = Xmem<<16 + Ymem<<16` | map:L111 ; p.4‑4 |
| A2‑A3 | `SUB Xmem, Ymem, dst` | `1010001D XXXXYYYY` 0xA200/0xFE00 | `dst = Xmem<<16 − Ymem<<16` | map:L112 ; p.4‑187 |
| A4‑A5 | `MPY Xmem, Ymem, dst` | `1010010D XXXXYYYY` 0xA400/0xFE00 | `dst = Xmem*Ymem ; T = Xmem` | map:L113 ; p.4‑101 |
| A6‑A7 | `MACSU Xmem, Ymem, src` | `1010011S XXXXYYYY` 0xA600/0xFE00 | `src = src + unsXmem*Ymem ; T = Xmem` | map:L114 ; p.4‑91 |
| **A8‑AB** | `LD Xmem,dst \|\| MAC[R] Ymem,dst_` | `101010RD XXXXYYYY` 0xA800/0xFC00 (**R=b9**) | `dst = Xmem<<16 \|\| dst_ = dst_ + T*Ymem` (rnd si R) | p.4‑74 ; Tab.2‑23 ; **map:L115 imprécis** |
| **AC‑AF** | `LD Xmem,dst \|\| MAS[R] Ymem,dst_` | `101011RD XXXXYYYY` 0xAC00/0xFC00 | `dst = Xmem<<16 \|\| dst_ = dst_ − T*Ymem` | p.4‑76 ; **map:L115 imprécis** |
| B0‑B3 | `MAC Xmem,Ymem,src[,dst]` | `10110RSD XXXXYYYY` 0xB000/0xFC00 | `dst = src + Xmem*Ymem ; T = Xmem` | map:L116 ; p.4‑82 |
| B4‑B7 | `MACR Xmem,Ymem,src[,dst]` | `10110RSD` (R=1) 0xB400/0xFC00 | `dst = rnd(src + Xmem*Ymem) ; T = Xmem` | map:L117 ; p.4‑82 |
| B8‑BB | `MAS Xmem,Ymem,src[,dst]` | `10111RSD XXXXYYYY` 0xB800/0xFC00 | `dst = src − Xmem*Ymem ; T = Xmem` | map:L118 ; p.4‑94 |
| BC‑BF | `MASR Xmem,Ymem,src[,dst]` | `10111RSD` (R=1) 0xBC00/0xFC00 | `dst = rnd(src − Xmem*Ymem) ; T = Xmem` | map:L119 ; p.4‑94 |
| C0‑C3 | `ST src,Ymem \|\| ADD Xmem,dst` | `110000SD XXXXYYYY` 0xC000/0xFC00 | `Ymem = (src<<ASM)>>16 \|\| dst = dst_ + Xmem<<16` | map:L120 ; p.4‑177 ; Tab.2‑24 |
| C4‑C7 | `ST src,Ymem \|\| SUB Xmem,dst` | `110001SD XXXXYYYY` 0xC400/0xFC00 | `Ymem = (src<<ASM)>>16 \|\| dst = Xmem<<16 − dst_` | map:L121 (générique) ; p.4‑185 |
| C8‑CB | `ST src,Ymem \|\| LD Xmem,dst` | `110010SD XXXXYYYY` 0xC800/0xFC00 | `Ymem = (src<<ASM)>>16 \|\| dst = Xmem<<16` | map:L122 ; p.4‑178 |
| CC‑CF | `ST src,Ymem \|\| MPY Xmem,dst` | `110011SD XXXXYYYY` 0xCC00/0xFC00 | `Ymem = (src<<ASM)>>16 \|\| dst = T*Xmem` | map:L123 (générique) ; p.4‑184 |
| D0‑D3 | `ST src,Ymem \|\| MAC Xmem,dst` | `11010RSD XXXXYYYY` 0xD000/0xFC00 | `… \|\| dst = dst + T*Xmem` | map:L124 ; p.4‑180 |
| D4‑D7 | `ST src,Ymem \|\| MACR Xmem,dst` | `11010RSD` (R=1) 0xD400/0xFC00 | `… \|\| dst = rnd(dst + T*Xmem)` | map:L125 ; p.4‑180 |
| D8‑DB | `ST src,Ymem \|\| MAS Xmem,dst` | `11011RSD XXXXYYYY` 0xD800/0xFC00 | `… \|\| dst = dst − T*Xmem` | map:L126 ; p.4‑182 |
| DC‑DF | `ST src,Ymem \|\| MASR Xmem,dst` | `11011RSD` (R=1) 0xDC00/0xFC00 | `… \|\| dst = rnd(dst − T*Xmem)` | map:L127 ; p.4‑182 |
| E0 | `FIRS Xmem,Ymem,pmad` **(2 mots)** | `11100000 XXXXYYYY` + pmad ; 0xE000/0xFF00 | `B = B + A*pmad ; A = (Xmem + Ymem)<<16` | map:L128 ; p.4‑59 |
| E1 | `LMS Xmem, Ymem` | `11100001 XXXXYYYY` 0xE100/0xFF00 | `B = B + Xmem*Ymem ; A = A + Xmem<<16 + 2^15` | map:L129 ; p.4‑80 |
| E2 | `SQDST Xmem, Ymem` | `11100010 XXXXYYYY` 0xE200/0xFF00 | `B = B + A[32:16]² ; A = (Xmem − Ymem)<<16` | map:L130 ; p.4‑160 |
| E3 | `ABDST Xmem, Ymem` | `11100011 XXXXYYYY` 0xE300/0xFF00 | `B = B + \|A[32:16]\| ; A = (Xmem − Ymem)<<16` | map:L131 ; p.4‑2 |
| **E4, E6** | `ST src,Ymem \|\| LD Xmem,T` | `111001S0 XXXXYYYY` **0xE400/0xFD00** (b8=0 fixe) | `Ymem = (src<<ASM)>>16 \|\| T = Xmem` | p.4‑178 ; Tab.2‑22 ; **map:L132 masque trop large** |
| E5 | `MVDD Xmem, Ymem` | `11100101 XXXXYYYY` 0xE500/0xFF00 | `Ymem = Xmem` | map:L133 ; p.4‑107 |
| E7 | `MVMM MMRx, MMRy` | `11100111 MMRXMMRY` 0xE700/0xFF00 | `MMRy = MMRx` (AR0‑AR7, SP) | map:L134 ; p.4‑116 |
| E8‑E9 | `LD #K, dst` | `1110100D KKKKKKKK` 0xE800/0xFE00 | `dst = K` (8 b non signé) | map:L135 ; p.4‑66 |
| EA‑EB | `LD #k9, DP` | `1110101K KKKKKKKK` 0xEA00/0xFE00 | `DP = k9` | map:L136 ; p.4‑70 |
| EC | `RPT #K` | `11101100 KKKKKKKK` 0xEC00/0xFF00 | `RC = K` | map:L137 ; p.4‑146 |
| ED | `LD #k5, ASM` | `11101101 000KKKKK` **0xED00/0xFFE0** | `ASM = k5` | map:L138 ; p.4‑70 |
| **ED20‑EDFF** | **— non assigné —** | — | — | manuel : rien ; **map:L138 « autre famille » non fondé** |
| EE | `FRAME K` | `11101110 KKKKKKKK` 0xEE00/0xFF00 | `SP = SP + K` (K signé −128..127) | map:L139 ; p.4‑60 |
| **EF** | **— non assigné —** | — | — | absent manuel + map |
| F0‑F3 | famille `#lk` / `src,SHIFT` | voir §F0 | voir §F0 | map:L140 ; `0xF3.md` |
| F4‑F7 | famille `src,ASM` / 1‑mot / spécial | voir §F4 | voir §F4 | map:L141‑149 |
| F8 | `BC pmad, cond` **(2 mots)** | `111110Z0 CCCCCCCC` 0xF800/**0xFD00** (Z=b9) | `si conds → PC = pmad` | map:L150 ; p.4‑18 |
| F8/FA (b7=1) | `FB[D] extpmad` **(2 mots)** | `111110Z0 1PPPPPPP` **0xF880/0xFD80** | `PC = pmad[15:0] ; XPC = pmad[22:16]` | p.4‑53 ; **absent de map** |
| F9 | `CC pmad, cond` **(2 mots)** | `111110Z1 CCCCCCCC` 0xF900/0xFD00 | `si conds → −−SP, TOS = PC+2 ; PC = pmad` | map:L151 ; p.4‑29 |
| F9/FB (b7=1) | `FCALL[D] extpmad` **(2 mots)** | `111110Z1 1PPPPPPP` **0xF980/0xFD80** | `−−SP, TOS=PC+2 ; PC=pmad[15:0] ; XPC=pmad[22:16]` | p.4‑57 ; **absent de map** |
| FA | `BCD pmad, cond` | `111110Z0` Z=1 → 0xFA00 | idem BC, retardé (2 slots) | map:L152 ; p.4‑18 |
| FB | `CCD pmad, cond` | Z=1 → 0xFB00 | idem CC, retardé | map:L153 ; p.4‑29 |
| FC | `RC cond` / `RET` (cond=0) | `111111Z0 CCCCCCCC` 0xFC00/**0xFD00** | `si conds → PC = TOS ; ++SP` | map:L154 ; p.4‑133/4‑139 |
| FD | `XC 1, cond` | `111111N1 CCCCCCCC` 0xFD00/0xFD00 (N=b9=0) | `si conds → exécuter la prochaine instr. (1 mot)` | map:L155 ; p.4‑198 |
| FE | `RCD cond` / `RETD` | Z=1 → 0xFE00 | idem RC, retardé | map:L156 ; p.4‑133 |
| FF | `XC 2, cond` | N=1 → 0xFF00 | `si conds → exécuter les 2 prochaines instr.` | map:L157 ; p.4‑198 |

---

### §6F — mot 2 (`0x6F00` + mot2), vérifié caractère par caractère

| mot2 | mnémonique | encodage mot2 | EXECUTION (SPRU172C) |
|---|---|---|---|
| 0x0C00/0xFCE0 | `ADD Smem, SHIFT, src [,dst]` | `000011SD 000SHIFT` (b9=SRC, b8=DST) | `dst = src + Smem << SHIFT` (p.4‑4) |
| 0x0C20/0xFCE0 | `SUB Smem, SHIFT, src [,dst]` | `000011SD 001SHIFT` | `dst = src − Smem << SHIFT` (p.4‑187) |
| 0x0C40/0xFEE0 | `LD Smem, SHIFT, dst` | `0000110D 010SHIFT` (**b9=0**, b8=DST) | `dst = Smem << SHIFT` (p.4‑66) |
| 0x0C60/0xFEE0 | `STH src, SHIFT, Smem` | `0000110S 011SHIFT` (**b9=0**, b8=SRC) | `Smem = (src << SHIFT) >> 16` (p.4‑169) |
| 0x0C80/0xFEE0 | `STL src, SHIFT, Smem` | `0000110S 100SHIFT` (**b9=0**, b8=SRC) | `Smem = src << SHIFT` (p.4‑172) |

`SHIFT` = 5 bits **signés** (−16..+15). Concordance intégrale avec `doc/opcodes/0x68_0x6F.md` §0x6Fxx. Source brute : `spru.txt` pages ADD 4‑4 (`I01101111AAAAAAA`+`00000DS11T00FIHS`), SUB 4‑187, LD 4‑66, STH 4‑169, STL 4‑172.

### §F0 — famille 0xF0xx‑0xF3xx (b9=SRC, b8=DST)

| bits 7‑0 | mnémonique | mots | base/masque | EXECUTION |
|---|---|---|---|---|
| `0000SHFT` | `ADD #lk, SHFT, src [,dst]` | 2 | 0xF000/0xFCF0 | `dst = src + lk << SHFT` (p.4‑4) |
| `0001SHFT` | `SUB #lk, SHFT, src [,dst]` | 2 | 0xF010/0xFCF0 | `dst = src − lk << SHFT` (p.4‑187) |
| `0010SHFT` | **`LD #lk, SHFT, dst`** | 2 | **0xF020/0xFEF0** (b9=0) | `dst = lk << SHFT` (p.4‑66) — **absent de `0xF3.md`** |
| `0011SHFT` | `AND #lk, SHFT, src [,dst]` | 2 | 0xF030/0xFCF0 | `dst = src & lk << SHFT` (p.4‑11) |
| `0100SHFT` | `OR #lk, SHFT, src [,dst]` | 2 | 0xF040/0xFCF0 | `dst = src \| lk << SHFT` (p.4‑123) |
| `0101SHFT` | `XOR #lk, SHFT, src [,dst]` | 2 | 0xF050/0xFCF0 | `dst = src ^ lk << SHFT` (p.4‑201) |
| `01100000` | `ADD #lk, 16, src [,dst]` | 2 | 0xF060/0xFCFF | `dst = src + lk << 16` |
| `01100001` | `SUB #lk, 16, src [,dst]` | 2 | 0xF061/0xFCFF | `dst = src − lk << 16` |
| `01100010` | **`LD #lk, 16, dst`** | 2 | **0xF062/0xFEFF** (b9=0) | `dst = lk << 16` — **absent de `0xF3.md`** |
| `01100011` | `AND #lk, 16, src [,dst]` | 2 | 0xF063/0xFCFF | `dst = src & lk << 16` |
| `01100100` | `OR #lk, 16, src [,dst]` | 2 | 0xF064/0xFCFF | `dst = src \| lk << 16` |
| `01100101` | `XOR #lk, 16, src [,dst]` | 2 | 0xF065/0xFCFF | `dst = src ^ lk << 16` |
| `01100110` | **`MPY #lk, dst`** | 2 | **0xF066/0xFEFF** (b9=0) | `dst = T * lk` (p.4‑101) — **absent de `0xF3.md`** |
| `01100111` | `MAC #lk, src [,dst]` | 2 | 0xF067/0xFCFF | `dst = src + T * lk` (p.4‑82) |
| `01110000` | `RPT #lk` | 2 | 0xF070/0xFFFF | `RC = lk` (p.4‑146) |
| `01110001` | `RPTZ dst, #lk` | 2 | 0xF071/0xFEFF | `RC = lk ; dst = 0` (p.4‑150) |
| `01110010` | `RPTB[D] pmad` | 2 | 0xF072/0xFDFF (b9=Z) | `RSA = PC+2 ; REA = pmad ; BRAF = 1` (p.4‑148) |
| `01110011` | `B[D] pmad` | 2 | 0xF073/0xFDFF | `PC = pmad` (p.4‑14) |
| `01110100` | `CALL[D] pmad` | 2 | 0xF074/0xFDFF | `−−SP, TOS = PC+2 ; PC = pmad` (p.4‑27) |
| `100SHIFT` | `AND src, SHIFT [,dst]` | **1** | 0xF080/0xFCE0 | `dst = dst & src << SHIFT` (p.4‑11) |
| `101SHIFT` | `OR src, SHIFT [,dst]` | **1** | 0xF0A0/0xFCE0 | `dst = dst \| src << SHIFT` (p.4‑123) |
| `110SHIFT` | `XOR src, SHIFT [,dst]` | **1** | 0xF0C0/0xFCE0 | `dst = dst ^ src << SHIFT` (p.4‑201) |
| `111SHIFT` | `SFTL src, SHIFT [,dst]` | **1** | 0xF0E0/0xFCE0 | `dst = src << SHIFT` **{logical shift}** (p.4‑158) |

### §F4 — famille 0xF4xx‑0xF7xx

| encodage | mnémonique | EXECUTION |
|---|---|---|
| `111101SD 000SHIFT` 0xF400/0xFCE0 | `ADD src, SHIFT [,dst]` | `dst = dst + src << SHIFT` (p.4‑4) |
| `111101SD 001SHIFT` 0xF420/0xFCE0 | `SUB src, SHIFT [,dst]` | `dst = dst − src << SHIFT` (p.4‑187) |
| `111101SD 010SHIFT` 0xF440/0xFCE0 | `LD src, SHIFT, dst` | `dst = src << SHIFT` (p.4‑66) |
| `111101SD 011SHIFT` 0xF460/0xFCE0 | `SFTA src, SHIFT [,dst]` | `dst = src << SHIFT` **{arithmetic}** (p.4‑155) |
| 0xF480/0xFCFF | `ADD src, ASM [,dst]` | `dst = dst + src << ASM` |
| 0xF481 | `SUB src, ASM [,dst]` | `dst = dst − src << ASM` |
| 0xF482 | `LD src, ASM [,dst]` | `dst = src << ASM` |
| 0xF483 (b8=0) | `SAT src` | `saturate(src)` (p.4‑154) |
| 0xF484 | `NEG src [,dst]` | `dst = −src` (p.4‑119) |
| 0xF485 | `ABS src [,dst]` | `dst = \|src\|` (p.4‑3) |
| 0xF486 (b9=0) | `MAX dst` | `dst = max(A,B)` (p.4‑99) |
| 0xF487 | `MIN dst` | `dst = min(A,B)` (p.4‑100) |
| 0xF488 / 0xF489 | `MACA[R] T, src [,dst]` | `dst = src + T*A[32:16]` (rnd) (p.4‑85) |
| 0xF48A / 0xF48B | `MASA[R] T, src [,dst]` | `dst = src − T*A[32:16]` (p.4‑97) |
| 0xF48C (b9=0) | `MPYA dst` | `dst = T * A[32:16]` (p.4‑104) |
| 0xF48D (b9=0) | `SQUR A, dst` | `dst = A[32:16]²` (p.4‑161) |
| 0xF48E (b8=0) | `EXP src` | `T = nb de bits de signe de src − 8` (p.4‑52) |
| 0xF48F | `NORM src [,dst]` | `dst = src << TS` (p.4‑122) |
| 0xF490 / 91 / 92 (b8=0) | `ROR` / `ROL` / `ROLTC src` | rotation via C / via TC (p.4‑143‑145) |
| 0xF493 | `CMPL src [,dst]` | `dst = ~src` (p.4‑32) |
| 0xF494 (b8=0) | `SFTC src` | `si src31 == src30 → src = src << 1` (p.4‑157) |
| **0xF495/0xFFFF** | `NOP` | aucune opération (p.4‑121) |
| `111101Z0 10011011` 0xF49B/0xFDFF | `RETF[D]` | `PC = RTN ; ++SP ; INTM = 0` (p.4‑141) |
| 0xF49F | `RND src [,dst]` | `dst = src + 2^15` (p.4‑142) |
| **0xF4A0/0xFFF8** | `LD #k3, ARP` | `ARP = k3` (p.4‑70) — **absent de map** |
| `111101CC 10101ARX` **0xF4A8/0xFCF8** | `CMPR CC, ARx` | compare ARx à AR0 → TC (p.4‑34) — **absent de map** |
| `111101N0 1011SBIT` 0xF4B0/0xFDF0 | `RSBX N, SBIT` | `ST(N)[SBIT] = 0` (p.4‑151) |
| `111101N1 1011SBIT` 0xF5B0/0xFDF0 | `SSBX N, SBIT` | `ST(N)[SBIT] = 1` (p.4‑166) |
| **0xF4C0/0xFFE0** | `TRAP K` | `−−SP, TOS = PC+1 ; PC = IPTR[15:7] + K<<2` (p.4‑195) |
| **0xF7C0/0xFFE0** | **`INTR K`** | `−−SP, TOS = PC+1 ; PC = IPTR[15:7] + K<<2 ; INTM = 1` (p.4‑65) — **PAS 0xF300** |
| `111101NN 11100001` 0xF4E1/0xFCFF | `IDLE K` | `idle(K)`, K=1..3 en b9‑b8 (p.4‑63) |
| `111101ZS 11100010` 0xF4E2/**0xFCFF** | `BACC[D] src` | `PC = src[15:0]` (p.4‑15) |
| `111101ZS 11100011` 0xF4E3/**0xFCFF** | `CALA[D] src` | `−−SP, TOS = PC+1 ; PC = src[15:0]` (p.4‑25) |
| `111101Z0 11100100` 0xF4E4/0xFDFF | `FRET[D]` | `XPC = TOS, ++SP ; PC = TOS, ++SP` (p.4‑61) |
| 0xF4E5/0xFDFF | `FRETE[D]` | idem + `INTM = 0` (p.4‑62) |
| 0xF4E6/0xFCFF | `FBACC[D] src` | `PC = src[15:0] ; XPC = src[22:16]` (p.4‑54) |
| 0xF4E7/0xFCFF | `FCALA[D] src` | `−−SP, TOS = PC+1 ; PC = src[15:0] ; XPC = src[22:16]` (p.4‑55) |
| `111101Z0 11101011` 0xF4EB/0xFDFF | `RETE[D]` | `PC = TOS ; ++SP ; INTM = 0` (p.4‑140) |
| **0xF7E0/0xFFFF** | `RESET` | reset logiciel (p.4‑138) — **absent de map** |

---

## DÉSACCORDS TABLE PROJET ↔ MANUEL

| # | Objet | Table projet | SPRU172C | Statut |
|---|---|---|---|---|
| 1 | **`INTR K`** | `0xF3.md` : « F300‑F31F = INTR k, mask 0xFFE0 », et le pseudo‑code y met un handler INTR sur `(op & 0xFFE0) == 0xF300` | **`INTR K` = 0xF7C0/0xFFE0** (p.4‑65). 0xF300‑0xF30F = `ADD #lk,SHFT,B,B` ; 0xF310‑0xF31F = `SUB #lk,SHFT,B,B` (2 mots) | **CONFIRMÉ** (manuel formel ; TRAP=0xF4C0 et INTR=0xF7C0 sont symétriques, seul b9/b8 diffère). Le « 342 sites F300‑F31F » de `0xF3.md` sont donc des ADD/SUB `#lk` **2 mots**, pas des INTR 1 mot → dérive de PC de +1 mot par site |
| 2 | `0xF3.md` liste FCFF : 60,61,63,64,65,67 | idem | Manque **0xF062 `LD #lk,16,dst`** et **0xF066 `MPY #lk,dst`** | **CONFIRMÉ** (table projet incomplète) |
| 3 | `0xF3.md` : « F320‑F32F unmapped/reserved » | idem | Vrai pour F3 (b9=1 interdit pour LD), **mais 0xF020‑0xF02F et 0xF120‑0xF12F = `LD #lk,SHFT,dst`** | **CONFIRMÉ** — incomplétude, pas erreur pour F3xx |
| 4 | `0xE4..0xE7` | map:L132 : `st (parallel)` **0xE400/0xFC00** « overlapping » | `ST src,Ymem \|\| LD Xmem,T` = `111001S0` → **0xE400/0xFD00**, seulement 0xE4 et 0xE6 ; 0xE5=MVDD, 0xE7=MVMM (b8 discrimine) | **CONFIRMÉ** — masque projet trop large (vole 0xE5/0xE7) |
| 5 | `0xA8..0xAF` | map:L115 : `ld (variantes)` 0xA800‑0xAE00/0xFE00 | 4 sous‑cas **parallèles** : 0xA8‑A9 `LD\|\|MAC`, 0xAA‑AB `LD\|\|MACR`, 0xAC‑AD `LD\|\|MAS`, 0xAE‑AF `LD\|\|MASR` (`101010RD`/`101011RD`) | **CONFIRMÉ** — un seul mnémonique là où le manuel en donne 4 (patron n°2) |
| 6 | `BACC/CALA` | map:L146 : `0xF4E2/F4E3 / 0xFEFF` | b9 = Z (delayed), b8 = S (acc) → masque **0xFCFF** ; BACCD = 0xF6E2 | **CONFIRMÉ** — masque projet ne libère pas le bit delayed |
| 7 | `0xF8/0xF9` | map:L150‑151 : `bc` 0xF800/**0xFF00**, `cc` 0xF900/0xFF00 | b9 = Z → masque **0xFD00** (BCD=0xFA00, CCD=0xFB00, déjà listés L152‑153) ; et **b7=1 → `FB[D]` 0xF880 / `FCALL[D]` 0xF980** | **CONFIRMÉ** — FB/FCALL/FBACC/FCALA absents de la table |
| 8 | `0xFC` | map:L154 : `ret / rc cond` 0xFC00/0xFF00 | masque **0xFD00** (b9 = delayed → 0xFE00) | **CONFIRMÉ** (cohérent avec L156) |
| 9 | `0xED` | map:L138 : « ED20+ = autre famille » | Aucune instruction en 0xED20‑0xEDFF. `LD #k3,ARP` est en **0xF4A0/0xFFF8**, pas en 0xED2x | **PROBABLE** (absence de preuve dans le manuel ; extraction du bloc 0xF4A0 lue caractère par caractère) |
| 10 | `0x35/0x37 maca/macar` | map:L43/L45 : `0x3500 / 0xFF00` | `001101R1` → **b8 = 1 fixe** ; masque effectif 0xFD00 base 0x3500 ; 0x34=BITT et 0x36=POLY occupent b8=0 | **NON‑BUG** pour l'usage hi8 (0x35 et 0x37 sont bien les seuls valides) — précision seulement |
| 11 | `0xC0..0xDF` | map:L120‑127 : toutes « `st` (parallel) » génériques | 8 sous‑cas distincts : ADD / SUB / LD / MPY / MAC / MACR / MAS / MASR | **CONFIRMÉ** — incomplétude (patron n°2) |
| 12 | 0x97, 0xEF | absents de la table | Absents du manuel également → **réellement non assignés** | **NON‑BUG** (à traiter comme opcode inconnu, pas comme fallback silencieux) |
| 13 | `0x6F` mot 2, `0x68‑0x6B`, `0x6C/0x6E` | `0x68_0x6F.md` | Vérifié bit à bit : ADD 0x0C00 / SUB 0x0C20 / LD 0x0C40 / STH 0x0C60 / STL 0x0C80, b9=SRC (ADD/SUB) et b9=0 + b8=SRC1 (LD/STH/STL) ; BANZ « Status Bits: None », mode indirect appliqué avant le test | **NON‑BUG** — `0x68_0x6F.md` est exact |
| 14 | `0x1E00 subc`, `0x18/0x1A/0x1C` | map:L26‑29 | Confirmés (`0001100S`/`0001101S`/`0001110S`/`0001111S`) | **NON‑BUG** — la table projet était juste ; c'était le décodeur qui ne nommait que 4 des 8 sous‑cas |

### Points de sémantique à opposer au code (extraits SPRU172C)

- **Logiques zéro‑étendues** : `AND/OR/XOR Smem,src` (p.4‑11/123/201) — l'opérande mémoire est zéro‑étendu sur 40 bits, **jamais** sign‑étendu. `LDU` (p.4‑79) et `ADDS/SUBS` (p.4‑10/194) idem (`unsSmem`).
- **`SFTL` ≠ `SFTA`** : `SFTL` (0xF0E0, p.4‑158) = `{logical shift}` ; `SFTA` (0xF460, p.4‑155) = `{arithmetic shift}`. Divergence observable uniquement sur SHIFT négatif.
- **`SUBC`** (p.4‑192) : `ALU = src − Smem<<15` ; **si ALU ≥ 0** → `src = (ALU<<1)+1` **sinon** `src = src<<1`. Affecté par SXM ; affecte C et OV.
- **`MAR`** (p.4‑92) : effet sur `ARP` **conditionné par CMPT** — si CMPT=1 et ARx≠AR0 alors `ARP = x`.
- **`BANZ[D]`** (p.4‑16) : « Status Bits: None » ; le test porte sur ARx **après** application du mode indirect.
- **`STH`** (p.4‑169) : `Smem = src << SHIFT >> 16` (décalage puis extraction du haut), **pas** `(src>>16)<<SHIFT`.
- **`CMPS`** (p.4‑35) : compare `src[31:16]` à `src[15:0]`, stocke le plus grand, et met à jour TRN/TC.
- **`INTR K`** (p.4‑65) pose `INTM = 1` ; **`TRAP K`** (p.4‑195) **ne le pose pas**. C'est la seule différence sémantique entre les deux, en plus de l'encodage 0xF7C0 vs 0xF4C0.

---

# AUDIT 0x00-0x2F

# AUDIT hi8 0x00–0x2F — `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, 16865 l.)

Copie locale identique : `/root/.claude/jobs/26578783/tmp/calypso_c54x.c`. Aucun fichier modifié. Toutes les preuves d'encodage viennent de `spru.txt` décodé avec la permutation calibrée du contexte, recoupé avec `tic54x_hi8_map.md`.

**Synthèse par famille**
| Famille | Handler | Verdict |
|---|---|---|
| 0x00–0x0F ADD/ADDS/ADDC/SUB/SUBS/SUBB | L9434–9478 | 6 sous-cas /8 justes ; **ADDC (0x06) et SUBB (0x0E) faux** (F2, F3) |
| 0x10–0x1F LD/LDU/LDR/AND/OR/XOR/SUBC | L9320–9432 | **correcte** après le fix du 27–28/07 ; un seul reliquat sur LDR (F4) |
| 0x20–0x2F MPY/MPYU/SQUR/MAC/MAS | L9509–9554 | **massivement fausse : 12 des 16 sous-cas produisent une valeur erronée sans trace** (F1) |

---

## F1 — FAMILLE 0x20–0x2F : table de sous-cas décalée + `default` silencieux — **CONFIRMÉ — gravité 2 (le plus grave du lot)**

**Code** : `case 0x2:` L9509, sélecteur `sub = (op >> 8) & 0xF` L9512 → 16 valeurs possibles, **8 nommées** (0,1,4,5,8,9,A,B), `default:` **silencieux** L9545 qui exécute `acc -= T*Smem`. Patron n°1 **et** patron n°2 **et** patron n°5, cumulés.

**Encodages de référence** (`spru.txt`, décodage bit à bit de la ligne `Opcode` de chaque page) :
- MPY[R] `spru.txt@505831` brut `I0100DR00AAAAAAA` → `001000RD IAAAAAAA` = **0x2000/0xFC00, R = b9** (map:L30–31)
- MPYU `spru.txt@509247` `I0100D010AAAAAAA` → `0010010D` = **0x2400/0xFE00** (map:L32)
- SQUR `spru.txt@553965` `I0100D110AAAAAAA` → `0010011D` = **0x2600/0xFE00** (map:L33)
- MAC[R] `spru.txt@489773` `I0100SR01AAAAAAA` → `001010RS` = **0x2800/0xFC00, R = b9** (map:L34–35)
- MAS[R] `spru.txt@500255` `I0100SR11AAAAAAA` → `001011RS` = **0x2C00/0xFC00, R = b9** (map:L36–37)

**Tableau des divergences** (l'exécution attendue est la citation littérale du champ `Execution` de la page TI) :

| op | attendu (SPRU172C) | code | ligne | effet réel |
|---|---|---|---|---|
| 0x20/0x21 | `T × Smem → dst` (p.4-101) | MPY | L9518–9523 | **OK** |
| **0x22/0x23** | MPYR : `rnd(T × Smem) → dst` | `default` | L9545 | **`acc -= T*Smem`** — signe inversé, accumulation au lieu d'affectation |
| **0x24/0x25** | MPYU : `unsignedT × unsignedSmem → dst`, « multiplieur signé 17×17 avec le MSB des deux opérandes forcé à 0 » (`spru.txt@509247`) | **SQUR** (`val*val`, `T = val`) | L9524–9530 | opérande T ignoré, T écrasé, résultat sans rapport |
| **0x26/0x27** | SQUR : `Smem → T ; Smem × Smem → dst` (`spru.txt@553965`) | `default` | L9545 | **`acc -= T*Smem`** — l'énergie (somme de carrés) devient une soustraction de produit croisé |
| **0x28/0x29** | MAC : `Smem × T + src → src` (`spru.txt@489773`, syntaxe 1 ; **T non modifié**) | `{ b += a ; a = T*Smem }` (resp. `{ a += b ; b = … }`) | L9531–9536 | **A détruit, B corrompu** : l'accumulateur cible n'est jamais accumulé |
| **0x2A/0x2B** | MACR : `rnd(Smem × T + src) → src` | idem + **`s->t = val`** | L9537–9544 | idem + **T écrasé** alors que la syntaxe 1 ne touche pas T |
| 0x2C/0x2D | MAS : `src − Smem × T → src` (`spru.txt@500255`) | `default` | L9545–9552 | **OK par accident** |
| **0x2E/0x2F** | MASR : idem **+ arrondi** (`+2^15`, LSB 15–0 mis à 0) | `default` sans arrondi | L9545 | arrondi manquant (gravité 3) |

Le sélecteur d'accumulateur est en revanche juste partout : `sub & 1` = b8 = S/D, conforme à `001000RD` / `001010RS`.

Impact : `MAC Smem,src` est l'instruction centrale d'un corrélateur GSM ; ici elle **n'accumule pas** et **écrase l'accumulateur A**. C'est exactement le profil « sortie de démodulateur constante / énergie nulle » déjà rencontré sur `0x1800`.

**Patch proposé** (remplace L9509–9554) :

```c
    case 0x2: {
        /* 0x20-0x2F, SPRU172C :
         *   MPY[R] 001000RD  0x2000/0xFC00  (R=b9)  T*Smem -> dst        p.4-101
         *   MPYU   0010010D  0x2400/0xFE00          unsT*unsSmem -> dst  p.4-106
         *   SQUR   0010011D  0x2600/0xFE00          Smem*Smem -> dst, Smem -> T  p.4-161
         *   MAC[R] 001010RS  0x2800/0xFC00  (R=b9)  src + T*Smem -> src  p.4-82
         *   MAS[R] 001011RS  0x2C00/0xFC00  (R=b9)  src - T*Smem -> src  p.4-94
         * Syntaxe 1 (Smem) : T n'est PAS modifie par MPY/MAC/MAS. */
        int sub = (op >> 8) & 0xF;
        int d   = sub & 1;             /* b8 = src/dst accumulateur */
        int rnd = (sub >> 1) & 1;      /* b9 = R (MPY / MAC / MAS) */
        addr = resolve_smem(s, op, &ind);
        uint16_t val = data_read(s, addr);
        int64_t *acc = d ? &s->b : &s->a;
        int64_t product, r;

        if (sub <= 0x3) {                       /* MPY[R] */
            product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            if (rnd) { product += 0x8000; product &= ~0xFFFFLL; }
            *acc = sext40(product);
        } else if (sub <= 0x5) {                /* MPYU : non signe */
            product = (int64_t)(uint16_t)s->t * (int64_t)(uint16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            *acc = sext40(product);
        } else if (sub <= 0x7) {                /* SQUR */
            product = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            s->t = val;
            *acc = sext40(product);
        } else {                                /* MAC[R] (8-B) / MAS[R] (C-F) */
            product = (int64_t)(int16_t)s->t * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) product <<= 1;
            r = (sub >= 0xC) ? (*acc - product) : (*acc + product);
            if (rnd) { r += 0x8000; r &= ~0xFFFFLL; }
            *acc = sext40(r);
        }
        return consumed + s->lk_used;
    }
```
(`rnd = (sub>>1)&1` redonne bien b9 : 0x22/23→1, 0x2A/2B→1, 0x2E/2F→1, et 0 ailleurs.)

---

## F2 — 0x06/0x07 `ADDC Smem,src` : carry ignoré **et** sign-extension appliquée à tort — **CONFIRMÉ — gravité 2**

**Code** : L9446 `sub = (op>>9)&7` (8 valeurs, toutes atteintes), mais L9449–9451 ne distingue que `is_sub` / `is_unsigned` / `ts_shift` ; `sub == 3` ne coche aucun drapeau → L9452 sign-étend selon SXM et L9462 fait une addition simple. Il n'y a pas de `default` : le sous-cas est **nommé de fait mais traité comme un ADD**.

**Attendu** : `spru.txt@419642`, encodage `I0000S110AAAAAAA` → `0000011S IAAAAAAA` = 0x0600/0xFE00 (map:L17), exécution `Smem + src + C → src`, « **Sign extension is suppressed regardless of the value of the SXM bit** », `Affects C and OV`. Exemple TI : A=`00 0000 0013`, C=1, Smem=4 → A=`00 0000 0018`.

Deux écarts : (a) le `+ C` manquant ; (b) avec SXM=1 et Smem ≥ 0x8000, le code ajoute une valeur négative là où TI ajoute `unsSmem`.

## F3 — 0x0E/0x0F `SUBB Smem,src` : borrow ignoré **et** sign-extension appliquée à tort — **CONFIRMÉ — gravité 2**

**Code** : même bloc, `sub == 7` → `is_sub` seul → L9459 `s->b - v` avec `v` sign-étendu.

**Attendu** : `spru.txt@579710`, encodage `I0000D111AAAAAAA` → `0000111S IAAAAAAA` = 0x0E00/0xFE00 (map:L21), exécution `src − Smem − logical inversion of C → src`, « subtracts … and the logical inverse of the carry bit, C, from src **without sign extension** ». Exemple 1 : A=`00 0000 0006`, C=0, Smem=6 → A=`FF FFFF FFFF` (= 6−6−1). **Noter** : c'est `~C`, pas `C` — la table de référence du projet (map:L21, ligne « `src = src − Smem − C` ») est ici **imprécise**, le manuel fait foi.

**Patch proposé F2+F3** (L9449–9464) :

```c
         bool is_sub = (sub & 0x4) != 0;
         bool is_unsigned = (sub == 1 || sub == 5);  /* ADDS / SUBS */
         bool ts_shift = (sub == 2 || sub == 6);     /* ,TS variants */
+        /* ADDC (0x0600) / SUBB (0x0E00) : SPRU172C p.4-8 / p.4-191.
+         * « Sign extension is suppressed regardless of the value of SXM »
+         * et le terme de retenue est C pour ADDC, ~C pour SUBB. */
+        bool with_carry = (sub == 3 || sub == 7);
+        int  cin = (s->st0 & ST0_C) ? 1 : 0;
-        v = is_unsigned ? (uint16_t)val
+        v = (is_unsigned || with_carry) ? (uint16_t)val
                         : ((s->st1 & ST1_SXM) ? (int16_t)val : (uint16_t)val);
         if (ts_shift) { ... inchangé ... }
         if (is_sub) {
-            if (dst) s->b = sext40(s->b - v);
-            else     s->a = sext40(s->a - v);
+            int64_t borrow = with_carry ? (1 - cin) : 0;   /* SUBB : - ~C */
+            if (dst) s->b = sext40(s->b - v - borrow);
+            else     s->a = sext40(s->a - v - borrow);
         } else {
-            if (dst) s->b = sext40(s->b + v);
-            else     s->a = sext40(s->a + v);
+            int64_t cadd = with_carry ? cin : 0;           /* ADDC : + C */
+            if (dst) s->b = sext40(s->b + v + cadd);
+            else     s->a = sext40(s->a + v + cadd);
         }
```

## F4 — 0x16/0x17 `LDR Smem,dst` : le bit 15 du résultat est effacé au lieu d'être posé à 1 — **CONFIRMÉ — gravité 3**

**Code L9350–9356** : `v = (v<<16) + 0x8000;` puis L9353 `v &= 0xFFFFFFFF0000LL;`. Les bits 15–0 étant nuls avant l'ajout, le `+0x8000` est **intégralement annulé par le masque** : le handler calcule exactement `Smem << 16` et le commentaire « clear low 16 after rounding » décrit un arrondi qui n'a aucun effet (patron n°6, ici le commentaire est cohérent avec le code mais tous deux sont faux).

**Attendu** : `spru.txt@485715`, encodage `I1000D110AAAAAAA` → `0001011D IAAAAAAA` = 0x1600/0xFE00 (map:L25). Exécution : `Smem << 16 + 1 << 15 → dst31−16`. Description : « Smem is rounded by adding 2^15 … and clearing the **15 LSBs 14–0** of the accumulator to 0. **Bit 15 of the accumulator is set to 1.** » Exemple TI : `LDR *AR1,A`, SXM=0, mém=`FEDC` → **A = `00 FEDC 8000`**. Le code produit `00 FEDC 0000`.

**Patch proposé** :
```c
-            v &= 0xFFFFFFFF0000LL;  /* clear low 16 after rounding */
+            /* SPRU172C p.4-78 : efface les 15 LSB (14-0) seulement ; le bit 15
+             * reste a 1. Exemple TI : LDR *AR1,A / Smem=FEDC -> A=00 FEDC 8000. */
+            v &= ~0x7FFFLL;
```

## F5 — famille 0x00–0x0F : le bit C n'est **jamais** produit — **PROBABLE — gravité 2**

`ST0_C` n'est écrit qu'en 4 endroits du fichier — MAX/MIN L6392/6403/6865/6872, ROR/ROL L6613/6624/6918/6928/8624/8625 — et jamais par `case 0x0` (L9434–9478) ni par SUBC (L9383). Or SPRU172C annonce `Affects C and OV` pour ADD, ADDS, ADDC, SUB, SUBS, SUBB (`spru.txt@421547`, `@419642`, `@582674`, `@579710`) et pour SUBC (`spru.txt@580716`). Conséquences : (a) tout `BC …, C/NC` (évalué L5977, L8318, L8430) teste un bit périmé ; (b) le correctif F2/F3 ci-dessus lit un C qui n'est presque jamais rafraîchi, donc **F2/F3 ne sera pleinement correct qu'une fois C généré**.

Statut **PROBABLE** et non CONFIRMÉ sur un point précis : SPRU172C (référence *jeu d'instructions*) ne dit pas de quel rang provient la retenue ; la position bit-31 se déduit seulement de l'exemple ADDC (`00 0000 0013 + 4 + 1` → C=0 après) et est documentée dans SPRU131 (absent ici). Patch minimal, à n'appliquer qu'après vérification indépendante de ce point :

```c
+        {   /* C : retenue sortante de bit 31 (ADD family) / non-emprunt (SUB) */
+            uint64_t a0 = (uint64_t)(dst ? s->b : s->a) & 0xFFFFFFFFULL;
+            uint64_t b0 = (uint64_t)v & 0xFFFFFFFFULL;
+            uint64_t sum = is_sub ? (a0 - b0 - borrow) : (a0 + b0 + cadd);
+            if (sum >> 32) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
+        }
```

## F6 — bloc MAC/MAS/MPYA/BITT **mort** enfermé dans `case 0xF` — **CONFIRMÉ — gravité 3 (mais c'est la cause racine de F1)**

Les handlers L6422 `if ((op & 0xFC00) == 0x2800)`, L6450 `(op & 0xFE00) == 0x2A00 || … == 0x2E00`, puis L6469 `0x3500`, L6481 `0x3300`, L6493 `0x3700`, L6507 `0x3100`, L6519 `0x3000`, L6527 `0x3200`, L6544 `0x3400` sont physiquement situés dans le corps de `case 0xF:` (L5961–8474 ; vérification de profondeur d'accolades : tous à profondeur 1 relative à `case 0xF:`, sans garde intermédiaire). Sous `hi4 == 0xF`, `(op & 0xFC00)` vaut au minimum 0xF000 : **aucune de ces conditions ne peut être vraie**. Ces ~150 lignes ne s'exécutent jamais, et pendant ce temps le vrai `case 0x2:` (L9509) et le vrai `case 0x3:` (L9480) font autre chose. Le commentaire L6415–6421 (« BUG observé : MAC family non-implémentée → DSP correlator ne fait jamais d'accumulation ») décrit donc un bug **toujours actif**.

**Ne pas se contenter de déplacer ce bloc** : il est lui-même faux. L6423 `mac_sub = (op >> 9) & 1` traite b9 comme le sélecteur add/sub alors que b9 = **R** (`001010RS`, `spru.txt@489773`) → `MACR` (0x2A00) y serait exécuté en soustraction ; et son masque `0xFC00` en L6422 masque le second test L6450 pour 0x2A00. Correctif recommandé : **supprimer** L6410–6560 (bloc 0x28xx–0x37xx égaré dans `case 0xF`) et appliquer le patch F1 sur `case 0x2:`, en laissant `case 0x3:` (hors périmètre de cet audit) à l'agent 0x30–0x3F.

---

## Points vérifiés et **NON-BUG** (contrôlés, pas omis)

- **0x00/0x02/0x04/0x08/0x0A/0x0C** (ADD, ADDS, ADD,TS, SUB, SUBS, SUB,TS) : encodages `0000000S`/`0000001S`/`0000010S`/`0000100S`/`0000101S`/`0000110S` (`spru.txt@415929`, `@421547`, `@415969`, `@576234`, `@582674`, `@576274`) ↔ L9446–9464. Sélecteur `(op>>9)&7` couvre bien 8/8, `dst = (op>>8)&1` = bit S. Zéro-extension d'ADDS/SUBS conforme (« Smem is considered a 16-bit unsigned number regardless of the value of SXM »).
- **Décalage TS** L9455 / L9345 : `TS = bits 5–0 de T`, plage documentée **−16 ≤ TS ≤ 31** (`spru.txt@23910`). Le code sign-étend 6 bits (−32..31) : sur-ensemble strict, identique pour tout code légal. Non-bug.
- **0x10/0x12/0x14** LD/LDU/LD,TS : `0001000D`/`0001001D`/`0001010D` (`spru.txt@476348`, `@486645`, `@476388`) ↔ L9337–9349. LDU conforme à « dst15−0 = Smem, 0 → dst39−16, no sign extension regardless of SXM » (`spru.txt@486645`).
- **0x18/0x1A/0x1C** AND/OR/XOR (L9365–9382) : **corrects**, y compris les bits de garde. `AND` (`spru.txt@422555`) `Status Bits: None` et opérande zéro-étendu → les gardes sont effacées (le code fait `cur & (uint16_t)val`) ; `OR` (`spru.txt@522822`) précise explicitement `src39−16 unchanged`, ce que donne `cur | (uint16_t)val`. Vérifié sur l'exemple TI p.4-12 : `00 00FF 1200 & 0x1500 = 0x1000`.
- **0x1E** SUBC (L9383–9389) : `src − Smem<<15 → ALU ; If ALU ≥ 0 Then ALU<<1 + 1 → src Else src<<1 → src` (`spru.txt@580716`) — reproduit littéralement, y compris le `>= 0`. Seul reliquat : `Affects C and OV` non implémenté (cf. F5).
- **Longueurs d'instruction** : les trois familles sont 1 mot, `+1 mot` en adressage indirect long-offset/absolu ; toutes les sorties font `return consumed + s->lk_used` et `resolve_smem` pose `lk_used` (L4789–4794). **Aucune désynchronisation de flux (patron n°4) dans 0x00–0x2F.**
- **Interception amont** : aucun `return` entre L5255 et L5959 (traceurs seuls), et aucun autre site du fichier ne capte 0x0xxx/0x1xxx/0x2xxx en dehors du bloc mort F6. Les trois familles atteignent bien leur handler.
- **0x97 / 0xEF** hors périmètre ; **0x2x** n'a aucun opcode non assigné : les 16 sous-cas sont tous définis par TI, il n'y a donc **aucune justification** au `default:` L9545.

## Observation transverse (hors périmètre strict, à ne pas corriger dans le même lot)

`ST1_OVM` est défini (`calypso_c54x.h:85`) et `sat32()` existe (L61) mais **`sat32` n'est appelé nulle part** dans les 16865 lignes. Or ADD/ADDS/ADDC/SUB/SUBS/SUBB/MPY/MPYU/SQUR/MAC/MAS sont tous `Affected by OVM`. Divergence réelle mais globale (gravité 3) : à traiter comme un lot séparé, sinon impossible d'attribuer une régression à F1/F2/F3.

## Ordre d'application recommandé (un correctif mesurable à la fois)

1. **F1** (`case 0x2`) — seul, puis mesure : c'est celui qui doit débloquer l'accumulation du corrélateur.
2. **F6** (suppression du bloc mort L6410–6560) — purement hygiénique une fois F1 en place, aucun changement de comportement attendu ; sert de contrôle négatif.
3. **F2+F3** (ADDC/SUBB) — puis **F5** (génération de C) une fois la position du bit de retenue confirmée sur SPRU131.
4. **F4** (LDR bit 15) — indépendant, faible risque.

---

# REFUTATION 0x00-0x2F

# RÉFUTATION — audit hi8 0x00–0x2F (`calypso_c54x.c`, md5 `9d8108f4f626cfbc906ce11c258ce7e2`, 16865 l.)

Fichier conteneur et copie locale `/root/.claude/jobs/26578783/tmp/calypso_c54x.c` : md5 identiques, vérifiés. **Aucun fichier modifié.** Décodage `spru.txt` refait indépendamment avec la permutation du contexte (script jetable, sorties dans `/root/.claude/projects/-root/26578783-d601-4bb4-9186-c560384b85bc/tool-results/buw13dug1.txt`).

## Verdicts

| # | Objet | Verdict agent | **Verdict après réfutation** |
|---|---|---|---|
| F1 | `case 0x2` (0x20–0x2F) table décalée + `default` silencieux | CONFIRMÉ | **CONFIRMÉ** (renforcé) |
| F2 | ADDC 0x06/0x07 : `+C` absent + sign-ext à tort | CONFIRMÉ | **CONFIRMÉ** (précision de portée) |
| F3 | SUBB 0x0E/0x0F : `−~C` absent + sign-ext à tort | CONFIRMÉ | **CONFIRMÉ** |
| F4 | LDR 0x16/0x17 : bit 15 effacé | CONFIRMÉ | **CONFIRMÉ**, mais impact pratique quasi nul + un sous-argument REFUTÉ |
| F5 | Bit C jamais produit | PROBABLE | **CONFIRMÉ** sur le fait ; le **patch proposé est FAUX** (polarité) |
| F6 | Bloc MAC/MAS/0x3xxx mort dans `case 0xF` | CONFIRMÉ | **CONFIRMÉ**, mais le **patch proposé casse ROLTC** ; framing « cause racine de F1 » REFUTÉ |
| N1–N5 | nouveaux | — | 4 nouveaux findings dont 1 CONFIRMÉ grave |

---

## F1 — `case 0x2` (L9509–9554) — **CONFIRMÉ**

Réfutation tentée sur les trois axes, tous échouent :

- **Encodage** : re-décodé caractère par caractère, indépendamment du rapport. MPY `spru.txt@505831` `I0100DR00AAAAAAA` → `001000RD` ; MPYU `@509247` `I0100D010AAAAAAA` → `0010010D` ; SQUR `@553965` `I0100D110AAAAAAA` → `0010011D` ; MAC `@489773` `I0100SR01AAAAAAA` → `001010RS` ; MAS `@500255` `I0100SR11AAAAAAA` → `001011RS`. MPYU et SQUR ne diffèrent que par e[6] (`0`/`1` → b9) : la table projet map:L32/L33 est confirmée, **SQUR est bien en 0x26/0x27 et non 0x24/0x25**. Le code L9524 place SQUR en `sub 4/5`. Divergence réelle.
- **Handler amont** : `switch (hi4)` L5960, `case 0x2:` L9509 est l'unique chemin. Vérifié : aucun `return` de décodage entre L5034 et L5959 (un seul `return 1` L5037, vectorisation IRQ) ; `op` n'est jamais réassigné entre L5253 et L9509 ; profondeur d'accolades calculée → le bloc « MAC » L6408–6552 est à **depth 1 sous `case 0xF:`** (L5961→L8474), donc inatteignable. Rien n'attrape 0x2xxx avant.
- **Commentaire vs code** : le finding ne repose pas sur un commentaire — le commentaire L9524 (`/* SQUR Smem, A/B */`) et le code sont **cohérents entre eux et faux tous les deux**.

**Preuve sémantique supplémentaire que l'agent n'avait pas citée** (elle rend son point le plus fort irréfutable) : SPRU172C MAC, section *Description* — « **For syntaxes 2 and 4**, the data-memory value after the instruction is stored in T. » La syntaxe 1 est `MAC Smem,src` (0x28). Donc `s->t = val` à **L9543** (branche `case 0xA/0xB`) est formellement interdit par le manuel, pas seulement « imprécis ». Idem MAS : « Xmem is loaded into T » — syntaxe 2 seulement.

**Impact quantifié (nouveau)** : histogramme des 28672 mots du dump ROM DSP `/tmp/prom0.txt` (conteneur, lecture seule) :
`0x28`=48, `0x29`=27, **`0x2a`=114, `0x2b`=189**, `0x2c`=253, `0x2d`=19, `0x2e`=13, `0x2f`=71 → **734 mots** en 0x28–0x2F contre 68 en 0x20–0x27. La branche `case 0xA/0xB` (MACR, 303 mots candidats), qui détruit **les deux** accumulateurs et écrase T, est la plus fréquente de la famille. (Comptage brut incluant mots de données et 2ᵉ mots — indicatif, pas un désassemblage.)

**Défaut du patch proposé** : aucun sur la logique (`d = sub&1` = b8 = S/D pour les 5 encodages ; `rnd = (sub>>1)&1` = b9 pour MPY/MAC/MAS, non lu pour MPYU/SQUR). Deux réserves de régression à documenter avant application :
1. aujourd'hui `sub 4/5` fait `s->t = val` (effet SQUR) ; le patch supprime cette mise à jour de T sur 0x24/0x25 — correct per manuel (MPYU *Execution* : `unsignedT × unsignedSmem → dst`, pas de `→ T`), mais c'est un changement d'état observable ;
2. arrondi MASR : le patch arrondit `r = acc − product` après soustraction, ce qui est bien ce que dit MAS[R] (« rounds the result of the multiply **and subtract** operation »), alors que MPYR arrondit le produit (« the result of the multiply operation ») — le patch fait les deux correctement, ne pas « corriger » ça.

## F2 — ADDC 0x06/0x07 — **CONFIRMÉ**

Texte intégral relu `spru.txt@419642` : *Execution* `Smem + src + C → src` ; *Status Bits* « Affected by OVM, **C** / Affects C and OV » ; *Description* « adds the 16-bit single data-memory operand Smem **and the value of the carry bit C** to src … **Sign extension is suppressed regardless of the value of the SXM bit** » ; exemple `A=00 0000 0013`, `C=1`, Smem=4 → `A=00 0000 0018`. Encodage `I0000S110AAAAAAA` → `0000011S` → `(op>>9)&7 = 3`. Le code L9449–9451 ne coche ni `is_sub`, ni `is_unsigned`, ni `ts_shift` pour `sub==3` → L9452 sign-étend et L9462 additionne sans C. Divergence réelle, sur le code exécuté, pas sur un commentaire.

**Précision de portée que l'agent n'a pas faite** : le second écart (sign-extension) ne se manifeste que si `SXM=1` **et** `Smem ≥ 0x8000` ; sous `SXM=0` le code zéro-étend déjà. Le `+C` manquant, lui, est inconditionnel. 10 mots candidats 0x06/0x07 dans le dump ROM.

## F3 — SUBB 0x0E/0x0F — **CONFIRMÉ**

`spru.txt@579710` : encodage `I0000D111AAAAAAA` → `0000111D` → `sub==7` ; *Execution* `src − Smem − logical inversion of C → src` ; *Description* « subtracts … and **the logical inverse of the carry bit, C**, from src **without sign extension** » ; Exemple 1 `A=00 0000 0006`, `C=0`, Smem=6 → `A=FF FFFF FFFF`. Le code (`is_sub` seul, L9459) fait `s->a − v` sign-étendu. Confirmé, y compris la correction de la table projet (map:L21 dit `− C`, le manuel dit `− ~C` : **le manuel fait foi**, l'exemple 1 le prouve numériquement : 6−6−1 = −1). 37 mots candidats 0x0E/0x0F dans le dump.

## F4 — LDR 0x16/0x17 — **CONFIRMÉ sur le fond, un sous-argument REFUTÉ, gravité à rabaisser**

`spru.txt@485715` : *Description* « rounded by adding 2^15 … and clearing the **15 LSBs 14−0** … **Bit 15 of the accumulator is set to 1** » ; exemple `LDR *AR1,A`, SXM=0, mém `FEDC` → `A = 00 FEDC 8000`. Le code L9352–9353 (`v = (v<<16) + 0x8000; v &= 0xFFFFFFFF0000LL;`) efface bien le bit 15 → produit `00 FEDC 0000`. **CONFIRMÉ.**

- **REFUTÉ** — la formulation « le commentaire décrit un arrondi qui n'a aucun effet » induit en erreur : le masque n'est pas « inutile », il est **destructeur**. Et le patch proposé `v &= ~0x7FFFLL` est en réalité un **no-op** (les bits 14–0 sont déjà nuls après `(v<<16)+0x8000`) ; c'est la *suppression* du masque `0xFFFFFFFF0000` qui corrige, pas son remplacement.
- **Vérifié et non-bug** : `0xFFFFFFFF0000` conserve les bits 39–32 (nibbles positions 47–16), donc **les bits de garde ne sont pas détruits**, contrairement à ce qu'on pourrait supposer. Le seul dégât est le bit 15.
- **Gravité à rabaisser de 3 vers 4** : dans les 28672 mots du dump ROM, `0x16` apparaît **1 fois** et `0x17` **0 fois** — et ce mot unique peut être une donnée. Aucune raison de dépenser un cycle de mesure dessus.

## F5 — bit C jamais produit — **CONFIRMÉ sur le fait / PATCH FAUX**

- **Reclassement PROBABLE → CONFIRMÉ pour la divergence** : `grep ST0_C` donne 22 sites, dont exactement **6 écritures** (MAX L6392/6403 et L6865/6872, ROR/ROL L6613/6624, L6918/6928, L8624/8625) ; aucune dans `case 0x0` (L9434–9478), `case 0x1` SUBC (L9383) ni `case 0x2`. SPRU172C annonce `Affects C` pour ADD (`@416883`), ADDC (`@419642`), SUB (`@577228`), SUBB (`@579710`), SUBC (`@580716`). Divergence certaine.
- **Le rang du bit n'a PAS besoin de SPRU131** — l'agent s'est auto-limité à tort. Deux pages de SPRU172C le fixent : ROL `@540354` *Execution* `… src31 → C` (le MSB **31**, pas 39, les gardes étant effacées) et SFTL `@552318` « src31 − SHIFT − 1 is copied into the carry bit, C ». **C = retenue de bit 31.** Le point bloquant invoqué tombe.
- **Le patch proposé est FAUX (polarité sur la soustraction)** : `sum = a0 - b0 - borrow` en arithmétique `uint64_t` déborde par le bas quand `b0 > a0` → `sum >> 32` non nul → `C` est **posé** alors qu'il y a eu emprunt. Or SPRU172C SUB `@577228` : « if the result of the subtraction **generates a borrow**, the carry bit, C, is **cleared to 0** ». La condition doit être `C = (a0 >= b0 + borrow)`, pas `sum>>32`.
- **Second défaut de composition** : `borrow` et `cadd` sont déclarés à l'intérieur des branches `if (is_sub)/else` du patch F2/F3 et référencés par le bloc F5 placé après — le lot F2+F3+F5 tel qu'écrit ne compile pas.
- **Note d'ordonnancement conservée** : F2/F3 restent partiellement inertes tant que C n'est pas produit (ADDC lira toujours C=0 sauf après un MAX/MIN/ROR/ROL).

## F6 — bloc mort dans `case 0xF` — **CONFIRMÉ / framing REFUTÉ / patch DANGEREUX**

- **CONFIRMÉ** par calcul de profondeur d'accolades (pas par lecture visuelle) : `case 0xF:` L5961 depth 1, `case 0xE:` L8475 depth 1 → tout L5962–8474 est le corps de `case 0xF`. Les tests L6422 (`&0xFC00==0x2800`), L6450, L6469, L6481, L6493, L6507, L6519, L6527, L6544 sont tous à **depth 1**, sans garde intermédiaire, sous `hi4 == 0xF`. Aucune réassignation de `op`. Inatteignables.
- Sous-analyse de l'agent **vérifiée exacte** : `mac_sub = (op>>9)&1` L6423 lit bien R et non add/sub (`001010RS`), et `&0xFC00==0x2800` couvre 0x28–0x2B donc vole 0x2A00 au test L6450 — tandis que 0x2E00 (`&0xFC00 = 0x2C00`) l'atteindrait.
- **Framing REFUTÉ** : « c'est la cause racine de F1 » est faux. `case 0x2` L9509 est faux **indépendamment** ; le bloc L6408–6552 est une tentative de correctif jamais branchée, pas la cause. F1 et F6 sont deux findings disjoints ; F6 n'explique rien de F1.
- **Le patch proposé (« supprimer L6410–6560 ») CASSE un cas qui marche** : **L6560 est le `if ((op & 0xFEFF) == 0xF492)` = ROLTC**, handler vivant et load-bearing (le bloc mort s'arrête à L6552 ; L6554–6559 = son commentaire). Pire, il existe un **second** `0xF492` à L6863 (depth 3, sous `if (hi8 == 0xF4)` L6772) qui implémente **MAX** — l'ancien mis-décodage que le commentaire L6555–6559 dit avoir causé « cascade STL→IMR=0 ». Supprimer jusqu'à 6560 ne « ne change rien » : cela **réexpose la régression MAX**. Plage correcte : **L6408–6552 uniquement**.
- Effet de bord à documenter : ce bloc contenait aussi les seuls handlers de `LD Smem,T` (0x3000), `LD Smem,ASM` (0x3200), `BITT` (0x3400), `MPYA` (0x3100), `MASA/MACA/MACAR` (0x3300/0x3500/0x3700). Tous morts ⇒ ces opcodes tombent aujourd'hui dans le blanket `acc += T*Smem` de `case 0x3` L9502. Hors périmètre 0x00–0x2F, mais **le « fix BITT » revendiqué L6538–6543 n'a jamais pris effet** — à transmettre à l'agent 0x30–0x3F.

---

## Findings NON-BUG de l'agent : contrôlés, tous maintenus

Re-décodés indépendamment et confirmés justes : ADD `@415929` `0000000S`, ADD,TS `@415969` `0000010S`, ADDS `@421547` `0000001S`, SUB `@576234` `0000100S`, SUB,TS `@576274` `0000110S`, LD `@476348` `0001000D`, LD,TS `@476388` `0001010D`, LDU `@486645` `0001001D`, AND `@422555…` `I1000S001AAAAAAA` → `0001100S`. L'exemple TI `AND *AR3+,A` : `A=00 00FF 1200 & 0x1500 → A=00 0000 1000` — le code L9366–9368 (`cur & (uint16_t)val`) le reproduit, gardes comprises ; OR/XOR laissent bien `src39−16` inchangé (`cur | val`, `cur ^ val`). `lk_used` remis à `false` L5123 avant chaque instruction et posé par `resolve_smem` (mods 0xC–0xF) → **aucune désynchronisation de flux dans 0x00–0x2F**. Portée TS 6 bits = sur-ensemble strict de −16..31 : non-bug.

## Findings MANQUÉS par l'agent

**N1 — OVA/OVB ne sont JAMAIS positionnés — CONFIRMÉ — gravité 2.** L'agent n'a relevé que « `sat32` jamais appelé ». Or `grep` sur `ST0_OVA|ST0_OVB` : **6 lectures** (conditions AOV/ANOV/BOV/BNOV L5993–5996, L6016–6017, L8301, L8413 ; `SAT` L8620–8621) et **2 écritures, toutes deux des effacements** (L10490, L10495, dans `SAT A/B`). Aucun `st0 |= (1<<8)`/`(1<<9)` dans les 16865 lignes. Conséquences directes, plus graves que F5 : (a) tout `BC/RC/XC …, AOV/BOV` est **toujours faux** et `ANOV/BNOV` **toujours vrai** — un test de saturation dans le corrélateur ne branche jamais ; (b) `SAT src` (0xF483 / L8620) est un **no-op inconditionnel**. Toutes les instructions 0x00–0x2F sont `Affects OV` (ADD `@416883`, SUBB `@579710`, SUBC `@580716`, MAC/MAS/MPY `Affects OVdst`). À traiter dans le même lot que F5, pas séparément : F5 seul laisse la moitié des conditions mortes.

**N2 — MPY syntaxe 1 : ambiguïté interne de SPRU172C, ne PAS « corriger » — PROBABLE — gravité 5 (piège).** `@505831` : *Execution* 1 = `T × Smem → dst` (aucun `→ T`), alors que les syntaxes 2 et 3 écrivent explicitement `Xmem → T` / `Smem → T` ; mais la *Description* dit « T is loaded with the Smem or Xmem value in the read phase » sans restreindre. Le code actuel et le patch F1 laissent T intact sur 0x20–0x23 — cohérent avec le champ *Execution*, qui est le plus spécifique. À figer comme décision documentée, sinon un futur audit « corrigera » MPY en ajoutant `s->t = val` et cassera le corrélateur.

**N3 — fiabilité de `spru.txt` sur les glyphes `\000`/`\001` — méthodologique.** L'extraction mappe **plusieurs glyphes distincts sur `\000`** : `→` dans ADDC (`Smem + src + C \000 src`), `×` dans MPYU (`unsignedT \000 unsignedSmem \001 dst`), `≤` dans les plages (`0 \000 K \000 255`). Conséquence : **toute conclusion tirée d'un champ *Execution* seul est suspecte**. F2/F3 tiennent parce qu'ils reposent sur la prose (« the value of the carry bit C », « the logical inverse of the carry bit, C »), pas sur `\000`. Corollaire à surveiller : `SUBC`, « If ALU output \000 0 » — le pseudo-code est illisible (`≥` ou `>`), et la prose dit « If the result is **greater than** 0 » alors que le code L9386 teste `d >= 0`. **Non tranchable ici** → laisser en l'état, ne pas toucher L9386.

**N4 — `case 0x2` sub 8/9 : `s->a += s->b` sans `sext40` ni masque 40 bits (L9534–9535).** Contrairement à toutes les autres branches du fichier, l'accumulateur croît hors du domaine 40 bits. Disparaît avec le patch F1 ; mentionné pour que la revue du patch ne le réintroduise pas.

**N5 — la revendication « 6 sous-cas /8 justes » pour `case 0x0` est optimiste.** Les 8 sous-cas produisent la bonne *valeur d'accumulateur* dans 6 cas ; les 8 produisent le mauvais *état* (C, OV — N1+F5). Formulation à corriger dans le rapport pour ne pas laisser croire que `case 0x0` est propre à 75 %.

---

## Ordre d'application révisé

1. **F1** seul (`case 0x2` L9509–9554), patch de l'agent accepté tel quel. Mesure attendue : accumulation réelle du corrélateur (734 mots candidats dans la ROM, dont 303 sur la branche la plus destructrice).
2. **F6** — supprimer **L6408–6552 uniquement**, jamais jusqu'à 6560 (ROLTC vivant, dont la suppression réexpose le MAX fautif L6863). Contrôle négatif.
3. **F2+F3+F5+N1 en un seul lot** — F2/F3 seuls sont inertes sans C ; F5 seul laisse AOV/BOV morts. Corriger d'abord la polarité de C sur la soustraction (`C=0` sur emprunt, rang bit 31 sourcé par ROL `@540354` et SFTL `@552318`) et la portée des variables `borrow`/`cadd`.
4. **F4** — facultatif, gravité rabaissée (1 occurrence candidate dans 28672 mots) ; le correctif est la **suppression** de `v &= 0xFFFFFFFF0000LL` L9353, pas son remplacement par `& ~0x7FFFLL` (no-op).

---

# AUDIT 0x30-0x5F

## AUDIT hi8 0x30–0x5F — `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, 16865 l.)

Localisation des handlers : `case 0x3:` L9480‑9507 · `case 0x4:` L9556‑9652 · `case 0x5:` L9654‑9761. Aucune interception pré‑switch (vérifié : entre L5034 et L5960 le seul `return` est le vectoring IRQ L5037 ; labels `case` de premier niveau : 5961/8475/8724/9320/9434/9480/9509/9556/9654/9763).

**Patron n°4 (longueur) : RAS sur toute la plage.** 0x30‑0x5F sont tous des instructions 1 mot ; tous les handlers renvoient `consumed(=1) + s->lk_used` et `resolve_smem`/`resolve_lmem` posent bien `lk_used` sur les modes 0xC‑0xF. Aucune désynchronisation de décode. **Sauf F6** (voir infra) où l'absence de `return 0` provoque une ré‑exécution.

---

### F1 — `case 0x3` : blanket MAC sur 0x30‑0x37 et 0x3A‑0x3F — **CONFIRMÉ — gravité 2**
**Code** L9480‑9507. Après le seul cas nommé `(op & 0xFE00) == 0x3800` (SQURA, L9493), le corps L9501‑9505 exécute **inconditionnellement** `acc(bit8) += T*Smem`. Aucun sélecteur, aucun `default`, aucune trace : 14 opcodes sur 16 produisent une valeur crédible et fausse (patron n°1 + n°2 : 1 sous‑cas nommé / 16).

| hi8 | attendu (SPRU172C) | exécuté |
|---|---|---|
| 0x30 | `T = Smem` (p.4‑70, tab. 2‑19 `spru.txt@52290` « LD Smem,T T = Smem ») | `acc += T*Smem` |
| 0x31 | `B = Smem*A(32:16) ; T = Smem` (p.4‑104, opcode `I11001000AAAAAAA` → 0x31) | idem |
| 0x32 | `ASM = Smem(4:0)` (tab. 2‑19 « LD Smem,ASM ASM = Smem4−0 ») | idem |
| 0x33 | `B = B − Smem*A(32:16) ; T = Smem` (p.4‑97, `I11001100AAAAAAA` → 0x33) | idem |
| 0x34 | `TC = Smem(15 − T(3:0))` (p.4‑23, `I11000010AAAAAAA` → 0x34) | idem |
| 0x35/0x37 | `B = [rnd] B + Smem*A(32:16) ; T = Smem` (p.4‑85) | idem |
| 0x36 | `A = rnd(A(32:16)*T + B) ; B = Smem<<16` (p.4‑126, `I11000110AAAAAAA` → 0x36) | idem |
| 0x3A/0x3B | `SQURS : src = src − Smem*Smem ; T = Smem` (p.4‑164, `I1100S101AAAAAAA` → 0x3A/3B) | `acc += T*Smem` (**signe inversé**) |
| 0x3C‑0x3F | `ADD Smem,16,src[,dst] : dst = src + Smem<<16` (p.4‑4 syntaxe 3, `I1100DS11AAAAAAA` → b9=S b8=D) | `acc += T*Smem` |

Impact direct sur la chaîne FB : `LD Smem,T` (0x30) est l'instruction qui **charge le multiplicande** avant tout MPY/MAC ; ici elle laisse T inchangé *et* pollue l'accumulateur. `SQURS` (0x3A) accumule au lieu de soustraire — même famille de symptôme que le bug SQURA déjà corrigé (commentaire L9485‑9492).

**Patch proposé (remplacer L9480‑9507)** :
```c
    case 0x3: {
        addr = resolve_smem(s, op, &ind);
        uint16_t val = data_read(s, addr);
        int      d8  = (op >> 8) & 1;
        int16_t  ahi = (int16_t)((s->a >> 16) & 0xFFFF);   /* cf. F2 : A(32:16) */
        int64_t  p;
        switch (hi8) {
        case 0x30: s->t = val; return consumed + s->lk_used;            /* LD Smem,T   */
        case 0x32: s->st1 = (s->st1 & ~ST1_ASM_MASK) | (val & ST1_ASM_MASK);
                   return consumed + s->lk_used;                        /* LD Smem,ASM */
        case 0x34: { int b = (val >> (15 - (s->t & 0xF))) & 1;
                     if (b) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC; }
                   return consumed + s->lk_used;                        /* BITT        */
        case 0x31: p = (int64_t)ahi * (int16_t)val;
                   if (s->st1 & ST1_FRCT) p <<= 1;
                   s->b = sext40(p); s->t = val;                        /* MPYA Smem   */
                   return consumed + s->lk_used;
        case 0x33: p = (int64_t)ahi * (int16_t)val;
                   if (s->st1 & ST1_FRCT) p <<= 1;
                   s->b = sext40(s->b - p); s->t = val;                 /* MASA Smem   */
                   return consumed + s->lk_used;
        case 0x35: case 0x37:
                   p = (int64_t)ahi * (int16_t)val;
                   if (s->st1 & ST1_FRCT) p <<= 1;
                   if (hi8 == 0x37) { p += 0x8000; p &= ~0xFFFFLL; }    /* MACAR       */
                   s->b = sext40(s->b + p); s->t = val;                 /* MACA Smem   */
                   return consumed + s->lk_used;
        case 0x36: { int64_t bold = s->b;                                /* POLY        */
                     p = (int64_t)ahi * (int16_t)s->t;
                     if (s->st1 & ST1_FRCT) p <<= 1;
                     int64_t r = bold + p + 0x8000; r &= ~0xFFFFLL;
                     s->b = sext40((int64_t)(int16_t)val << 16);
                     s->a = sext40(r); }
                   return consumed + s->lk_used;
        case 0x38: case 0x39:                                            /* SQURA (F3)  */
        case 0x3A: case 0x3B: {                                          /* SQURS       */
            int64_t sq = (int64_t)(int16_t)val * (int64_t)(int16_t)val;
            if (s->st1 & ST1_FRCT) sq <<= 1;
            int64_t *acc = d8 ? &s->b : &s->a;
            *acc = sext40(hi8 <= 0x39 ? (*acc + sq) : (*acc - sq));
            s->t = val;                                                  /* cf. F3      */
            return consumed + s->lk_used; }
        default: {                                                       /* 0x3C-0x3F   */
            /* ADD Smem,16,src[,dst] : b9=S, b8=D */
            int64_t *src = ((op >> 9) & 1) ? &s->b : &s->a;
            int64_t *dst = d8 ? &s->b : &s->a;
            int64_t v = (s->st1 & ST1_SXM) ? ((int64_t)(int16_t)val << 16)
                                           : ((int64_t)(uint16_t)val << 16);
            *dst = sext40(*src + v);
            return consumed + s->lk_used; }
        }
    }
```

---

### F2 — 7 handlers 0x3xxx corrects, mais **placés dans `case 0xF:`** → code mort — **CONFIRMÉ — gravité 2**
**Code** L6467‑6552 : `if ((op & 0xFF00) == 0x3500)` (L6469), `0x3300` (L6481), `0x3700` (L6493), `0x3100` (L6507), `0x3000` (L6519), `0x3200` (L6527), `0x3400` (L6544). Analyse d'accolades : le bloc englobant unique de L6469 est `switch (hi4) {` (L5960), et le dernier label avant est `case 0xF:` (L5961). Ces tests exigent `(op>>8)==0x3x` alors que `hi4==0xF` — **jamais vrais**. Ces sept handlers, écrits avec leur justification de bug (« BITT non‑implémenté → TC stale → ROLTC … a_sync_ANG = 0x498D constant », L6535‑6543) n'ont **jamais** été exécutés ; le chemin réel est F1.

Même diagnostic pour `0x2800/0xFC00` (L6422) et `0x2A00/0x2E00` (L6450) — hors périmètre, à signaler à l'agent 0x20‑0x2F.

**Patch** : supprimer L6408‑6552 de `case 0xF` (le contenu est repris et corrigé dans le patch F1). Deux écarts à corriger au passage lors de la reprise :
- ils utilisent `A(31:16)` (`(int16_t)(s->a >> 16)`) là où le manuel dit `A(32:16)` (17 bits, p.4‑85/4‑97/4‑104) — divergence si le bit de garde 32 de A est non nul ;
- ils ne mettent **jamais** `T = Smem`, exigé par tab. 2‑14 `spru.txt@40285` (« MACA Smem[,B] B = B + Smem*A32−16, **T = Smem** », idem MASA, MPYA syntaxe 1 : « T is updated in the read phase »).

---

### F3 — SQURA 0x38/0x39 : `T = Smem` non appliqué — **CONFIRMÉ — gravité 2**
**Code** L9493‑9500 : calcule `sq`, applique FRCT, ajoute à l'accumulateur, `return`. **T n'est pas écrit.**
**Attendu** : SPRU172C p.4‑163, `spru.txt@555078` : « ExecutionSmem → T ; Smem * Smem + src → src » et Exemple 1 `SQURA 30,B` : `T` passe de `0003` à `000F`. Idem tab. 2‑14 : « SQURA Smem,src src = src + Smem*Smem, **T = Smem** ».
**Conséquence** : tout MPY/MAC suivant (0x20xx/0x28xx) multiplie par un T périmé. Dans une boucle d'énergie `SQURA` + `MAC`, le second terme est faux silencieusement.
**Patch** (dans F1, ou minimal en place à L9498) :
```c
                if (sdst) s->b = sext40(s->b + sq);
                else      s->a = sext40(s->a + sq);
+               s->t = val;                    /* SPRU172C p.4-163 : Smem -> T */
                return consumed + s->lk_used;
```

---

### F4 — 0x40‑0x43 `SUB Smem,16,src[,dst]` : bit 9 (S) ignoré — **CONFIRMÉ — gravité 2**
**Code** L9570‑9579 : `int dst_b = op8 & 0x01; int64_t *acc_dst = dst_b ? &s->b : &s->a;` puis `*acc_dst = sext40(*acc_dst - val);` → **source = destination = bit 8**.
**Attendu** : SPRU172C p.4‑187 syntaxe 3, opcode `I0010DS00AAAAAAA` (`spru.txt@575705`) → **b9 = S, b8 = D** ; exécution `src − Smem << 16 → dst`. Table projet L49 : `010000SD`, `dst = src − Smem << 16`.
**Comportement actuel vs attendu** :

| op8 | attendu | actuel |
|---|---|---|
| 0x40 | `A = A − Smem<<16` | idem (OK) |
| 0x41 | `B = A − Smem<<16` | `B = B − Smem<<16` ✗ |
| 0x42 | `A = B − Smem<<16` | `A = A − Smem<<16` ✗ |
| 0x43 | `B = B − Smem<<16` | idem (OK) |

Le commentaire L9558 dit correctement « SUB Smem,16,src[,dst] (mask 0xFC00) » — le code ne l'implémente pas (patron n°6 : commentaire juste, code faux).
**Patch** (L9573‑9579) :
```c
             if (op8 >= 0x40 && op8 <= 0x43) {
                 addr = resolve_smem(s, op, &ind);
-                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
-                *acc_dst = sext40(*acc_dst - val);
+                uint16_t m = data_read(s, addr);
+                int64_t val = (s->st1 & ST1_SXM) ? ((int64_t)(int16_t)m << 16)
+                                                 : ((int64_t)(uint16_t)m << 16);
+                int64_t *src = ((op >> 9) & 1) ? &s->b : &s->a;   /* b9 = S */
+                *acc_dst = sext40(*src - val);                    /* b8 = D */
                 return consumed + s->lk_used;
             }
```

---

### F5 — 0x47 `RPT Smem` écrit **BRC** au lieu de **RC** — **CONFIRMÉ — gravité 2**
**Code** L9595‑9602 :
```c
                s->brc = val;
                s->rpt_active = (val != 0); s->rpt_fresh = (val != 0);
```
**Attendu** : SPRU172C p.4‑146, `spru.txt@542037` : opcode syntaxe 1 `I00101110AAAAAAA` (→ 0x47), « Execution 1: Smem → **RC** » ; « The instruction following the repeat instruction is repeated n + 1 times ». Table projet L52 : « `RC = Smem` ».
**Double faute** :
1. `s->rpt_count` (= RC dans ce fichier) n'est **jamais** écrit → le moteur de répétition L15948‑15960 décrémente une valeur **périmée** du dernier `RPT #k` ; le nombre d'itérations est arbitraire.
2. `s->brc` est **écrasé** → tout `RPTB` englobant (BRC lu en L16109) perd son compteur de bloc.

Idiome correct attesté trois fois dans le fichier : `s->rpt_count = N; s->rpt_active = true; s->rpt_fresh = true; s->pc += …; return 0;` (L7130‑7133 `RPT #lk`, L8038‑8041, L10779‑10782). Le `return 0` est **obligatoire** : sans lui, le bloc L15948 voit `rpt_active && rpt_count>0`, décrémente et fait `continue` **sans avancer PC** → le `RPT` se ré‑exécute lui‑même.
**Patch** (L9595‑9602) :
```c
             if (op8 == 0x47) {
                 addr = resolve_smem(s, op, &ind);
                 uint16_t val = data_read(s, addr);
-                s->brc = val;
-                s->rpt_active = (val != 0); s->rpt_fresh = (val != 0);
-                return consumed + s->lk_used;
+                s->rpt_count  = val;               /* RC, PAS BRC (p.4-146) */
+                s->rpt_active = true; s->rpt_fresh = true;
+                s->pc += consumed + s->lk_used;    /* idiome L7130-7133 */
+                return 0;
             }
```

---

### F6 — 0x48/0x49 `LDM MMR,dst` : sign‑extension au lieu de zéro‑extension — **CONFIRMÉ — gravité 2**
**Code** L9603‑9609 : `*acc_dst = sext40((int16_t)val);`
**Attendu** : SPRU172C p.4‑73, `spru.txt@481484` : « Execution MMR → dst15−0 ; **000000h → dst39−16** » ; « This instruction is **not affected by the value of SXM** » ; Exemple 1 `LDM AR4,A` avec AR4=`FFFF` → **A = 00 0000 FFFF**. Table projet L53 : « `dst = MMR` (zéro‑ext.) ».
**Actuel** : `LDM AR4,A` avec AR4=0xFFFF donne `A = FF FFFF FFFF`. Tout test `ALEQ`/`ALT`/`AGT` sur un MMR ≥ 0x8000 (typiquement un compteur TPU, un IFR, un pointeur DARAM) bascule à l'inverse.
**Patch** (L9607) :
```c
-                *acc_dst = sext40((int16_t)val);
+                *acc_dst = (int64_t)(uint16_t)val;   /* zéro-ext. 40 b, SXM ignoré */
```

---

### F7 — 0x48/0x49/0x4A : adresse MMR = `op & 0x7F`, mode indirect non résolu — **CONFIRMÉ — gravité 3**
**Code** L9605 (`LDM`) et L9612 (`PSHM`) : `int mmr = op & 0x7F;` — le bit 7 (I) est ignoré, aucun `resolve_smem`, donc **aucune post‑modification d'AR**.
**Attendu** : SPRU172C p.4‑73 : « The nine MSBs of the effective address are cleared to 0 to designate data page 0, regardless of the current value of DP **or the upper nine bits of ARx** » — l'adresse effective est calculée normalement (y compris indirect avec post‑mod), puis masquée sur 7 bits. Words = 1 sans clause « add 1 word » → long‑offset/absolu interdits, donc pas de `lk_used` (le code est correct sur ce point).
**Actuel** : `LDM *AR3+, A` lit `data[op & 0x7F]` (= les bits mod/nar réinterprétés en adresse) et laisse AR3 figé.
**Patch** (L9603‑9613, appliqué à LDM et PSHM) :
```c
-                int mmr = op & 0x7F;
+                int mmr = resolve_smem(s, op, &ind) & 0x7F;  /* page 0 forcée, p.4-73 */
                 uint16_t val = data_read(s, mmr);
```

---

### F8 — 0x4E/0x4F `DST src,Lmem` : post‑modification ±1 au lieu de ±2 — **CONFIRMÉ — gravité 2**
**Code** L9645 : `addr = resolve_smem(s, op, &ind) & 0xFFFE;` — `resolve_smem` (L4568) post‑modifie de **±1** (L4626 `s->ar[cur_arp]--`, L4629 `++`).
**Attendu** : SPRU172C p.4‑47, `spru.txt@456507`, Exemple 1 `DST B,*AR3+` : AR3 `0100 → 0102`, note explicite « Because this instruction is a long‑operand instruction, AR3 is **incremented by 2** after the execution ». Exemple 2 `DST B,*AR3−` : AR3 `0101 → 00FF`.
**Actuel** : le pointeur d'écriture 32 bits avance d'un mot par store → chevauchement des paires, tableau de long‑words détruit à partir du 2e élément.
**Patch** (L9645) :
```c
-                addr = resolve_smem(s, op, &ind) & 0xFFFE;
+                addr = resolve_lmem(s, op);          /* post-mod ±2, p.4-47 */
```
(`resolve_lmem` L4768 fait bien `±2` et pose `lk_used` sur les modes 0xC‑0xF ; `ind` devient inutilisé ici.)

---

### F9 — Ordre mot‑haut/mot‑bas faux pour une adresse Lmem **impaire** (DST + tout `case 0x5`) — **CONFIRMÉ — gravité 3**
**Code** : `resolve_lmem` L4773/4777/4782/4789‑4794 force `& 0xFFFE` ; les consommateurs lisent `lhi = data[laddr]`, `llo = data[laddr+1]` (L9683‑9684) ; `DST` L9645‑9648 écrit MSW en `addr`, LSW en `addr+1` après `& 0xFFFE`.
**Attendu** : le mot **de poids fort est à l'adresse effective**, le mot de poids faible à l'adresse paire‑partenaire (EA ^ 1). Preuve directe SPRU172C p.4‑47 Exemple 2 (`spru.txt@456507`) : `DST B,*AR3−`, **AR3 = 0101 (impaire)**, B = `00 6CAC BD90` → après : `0101h = 6CAC` (MSW à l'adresse impaire) et `0100h = BD90`. Le code produit l'inverse (`0100h = 6CAC`, `0101h = BD90`).
**Portée** : identique pour DADD/DSUB/DLD/DRSUB/DADST/DSUBT/DSADT (L9682‑9729), tous alimentés par `resolve_lmem`. Sans effet tant que le firmware n'aligne que sur adresses paires — d'où gravité 3, mais divergence réelle.
**Patch** (`resolve_lmem`, ne plus masquer ; les appelants choisissent le partenaire) :
```c
-        return (uint16_t)(((dp << 7) | (opcode & 0x7F)) & 0xFFFE);
+        return (uint16_t)((dp << 7) | (opcode & 0x7F));
```
… et idem pour les `& 0xFFFE` sur `s->ar[nar]` L4777/4782/4789/4790/4793/4794 ; puis chez les appelants :
```c
-                uint16_t lhi    = data_read(s, laddr);
-                uint16_t llo    = data_read(s, (uint16_t)(laddr + 1));
+                uint16_t lhi    = data_read(s, laddr);              /* MSW = EA */
+                uint16_t llo    = data_read(s, (uint16_t)(laddr ^ 1));
```
(et symétriquement pour `DST`). **À traiter en un seul patch atomique** avec F8 — modifier `resolve_lmem` seul casserait les 7 handlers dual‑long.

---

### F10 — 0x40‑0x45 : sign‑extension inconditionnelle, SXM ignoré — **CONFIRMÉ — gravité 3**
**Code** L9576 (`SUB Smem,16`) et L9583 (`LD Smem,16,dst`) : `(int64_t)(int16_t)data_read(...) << 16` — équivaut à SXM = 1 en permanence.
**Attendu** : SPRU172C p.4‑6 (`spru.txt@416239`) : « For a left shift : Low‑order bits are cleared ; High‑order bits are : Sign extended if **SXM=1**, **Cleared if SXM=0** » ; pour LD, p.4‑67 « Status Bits : Affected by **SXM** in all accumulator loads », Exemples 1/2 (`LD *AR1,A` avec Smem=FEDC → `00 0000 FEDC` si SXM=0, `FF FFFF FEDC` si SXM=1).
**Actuel** : avec SXM=0 et Smem ≥ 0x8000, les bits de garde 39‑32 valent `FF` au lieu de `00` → `AGEQ`/`ALT` inversés, et saturation OVM déclenchée à tort.
**Patch** : voir F4 pour 0x40‑0x43 ; pour 0x44/0x45 (L9583) :
```c
-                int64_t val = (int64_t)(int16_t)data_read(s, addr) << 16;
+                uint16_t m = data_read(s, addr);
+                int64_t val = (s->st1 & ST1_SXM) ? ((int64_t)(int16_t)m << 16)
+                                                 : ((int64_t)(uint16_t)m << 16);
                 *acc_dst = sext40(val);
```

---

## Familles vérifiées correctes (NON‑BUG)

- **0x38/0x39 SQURA** — masque `0xFE00` (L9493), bit 8 = src, FRCT appliqué : conforme p.4‑163 / table L46. *Seul écart : T, cf. F3.*
- **0x44/0x45 `LD Smem,16,dst`** (L9580‑9586) — masque `0xFE00`, b8 = D : conforme table L50. *Seul écart : SXM, cf. F10.*
- **0x46 `LD Smem,DP`** (L9587‑9594) — `DP = Smem(8:0)` via `ST0_DP_MASK = 0x01FF` (`calypso_c54x.h:71`) : conforme tab. 2‑19 `spru.txt@52290` (« LD Smem,DP DP = Smem8−0 »).
- **0x4A `PSHM MMR`** (L9610‑9618) — `SP−1 → SP ; MMR → TOS`, ordre décrément‑puis‑écriture conforme p.4‑132 (`spru.txt@530487`). *Seul écart : adressage, cf. F7.*
- **0x4B `PSHD Smem`** (L9619‑9626) — conforme p.4‑131 (`spru.txt@529637`), Exemple `PSHD *AR3+` reproduit exactement.
- **0x4C `LTD Smem`** (L9627‑9634) — `T = Smem ; Smem+1 = Smem` : conforme p.4‑81 (`spru.txt@488506`), opcode `I00100011AAAAAAA` → 0x4C.
- **0x4D `DELAY Smem`** (L9635‑9641) — `Smem+1 = Smem` : conforme p.4‑41 (`spru.txt@449806`), opcode `I00101011AAAAAAA` → 0x4D.
- **0x4E/0x4F `DST`** — encodage `I0010S111AAAAAAA` → b8 = S, correctement décodé. *Écarts : F8 (post‑mod) et F9 (parité).*
- **0x50‑0x5F, famille dual‑long** (L9674‑9730) — **entièrement conforme**, vérifié opcode par opcode contre `spru.txt` : DADD `I1010DS00` → 0x50‑53 (b9=S, b8=D) ✓ ; DSUB `I1010S010` → 0x54/55 ✓ ; DLD `I1010D110` → 0x56/57 ✓ ; DRSUB `I1010S001` → 0x58/59 ✓ ; DADST `I1010D101` → 0x5A/5B ✓ ; DSUBT `I1010D011` → 0x5C/5D ✓ ; DSADT `I1010D111` → 0x5E/5F ✓. Les signes C16=1 du code (`sgn_hi` L9718, `sgn_lo` L9719) reproduisent exactement p.4‑39 (DADST : hi **+**T, lo **−**T), p.4‑50 (DSUBT : hi −T, lo −T) et p.4‑45 (DSADT : hi −T, lo **+**T) ; la branche C16=0 (`lmem40 ± tt40`, L9724‑9726) reproduit `Lmem ± (T<<16 + T)`. Contrôles numériques : DADD Ex.2 (`00 5678 3933` + `1534/3456` → `00 6BAC 6D89`) et DADST Ex.1 (`1534+2345=3879`, `3456−2345=1111`) sortent justes. **Corrige explicitement la table projet**, dont les lignes L64 et L65 donnent la même expression pour DSUBT et DSADT alors que le manuel les distingue sur le signe du demi‑mot bas.
- **Longueurs** — aucune divergence sur 0x30‑0x5F (patron n°4 clean).

## Code mort signalé, sans divergence de comportement

- **L9732‑9759, bloc SFTA/SFTL de `case 0x5`** : la garde L9675 `dl_hi >= 0x50 && dl_hi <= 0x5F` est **toujours vraie** sous `hi4 == 5` et le bloc dual‑long `return` systématiquement (L9730). Le sélecteur `sub = (op>>9)&0x7` L9734 n'est jamais évalué. **NON‑BUG comportemental** : SFTA/SFTL sont réellement en 0xF460/0xF0E0 (§F4/§F0 de la table), pas en 0x5xxx — le commentaire L9655‑9658 est périmé et décrit un décodage abandonné. Suppression recommandée pour supprimer un piège de relecture, pas pour corriger un comportement.
- **L9652, `return consumed + s->lk_used;` de fin de `case 0x4`** : inatteignable, la chaîne L9573‑9650 couvre 0x40‑0x4F sans trou. NON‑BUG.

## Ordre de traitement recommandé
F1+F2 (un seul patch : le blanket MAC et les handlers morts sont le même bug) → F5 (RPT/BRC, contamine RPTB) → F6 (LDM) → F4 (SUB S/D) → F3 (SQURA/T) → F8+F9 (atomiques) → F7 → F10.

---

# REFUTATION 0x30-0x5F

## REFUTATION — audit hi8 0x30–0x5F, `calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, identique conteneur/local, vérifié)

**Méthode de contrôle.** Portée des blocs établie mécaniquement (parseur C ignorant commentaires/chaînes, profondeur d'accolades ligne à ligne), pas par lecture. Encodages re-dérivés indépendamment depuis `spru.txt` via la permutation de la clé de lecture, puis **auto-validée** sur 10 points indépendants trouvés en cours de route (`LD Smem,DP`→0x46, `LD #k9,DP`→0xEA/EB, `LD #k5,ASM`→0xED, `MPYU`→0x24/25, `DLD`→0x56/57, `DSUBT`→0x5C/5D, `LDM`→0x48/49, `RPT Smem`→0x47, `SQURA`→0x38/39, `DST`→0x4E/4F). Aucun finding n'est retenu sur la foi d'un commentaire.

---

### F1 — blanket MAC sur 0x30-0x37 / 0x3A-0x3F — **CONFIRMÉ**
Tentative de réfutation échouée sur les trois axes. (a) *Handler antérieur ?* Non : `case 0x3:` L9480 est le seul chemin pour hi4==3 (labels de niveau 2 énumérés : 5961/8475/8724/9320/9434/**9480**/9509/9556/9654/9763/10280/10600/10924), et les handlers 0x3xxx concurrents sont hors d'atteinte (cf. F2). (b) *Encodages douteux ?* Non, re-décodés un par un depuis `spru.txt` : `LD Smem,T`=`I11000000AAAAAAA`→`00110000`=0x30 (@479533) ; `LD Smem,ASM`=`I11000100AAAAAAA`→`00110010`=0x32, exec `Smem4−0 → ASM` ; `BITT`=`I11000010AAAAAAA`→`00110100`=0x34, exec `Smem(15−T3−0) → TC` (@433966) ; `MPYA`=`I11001000AAAAAAA`→`00110001`=0x31 (@507788) ; `MASA`=`I11001100AAAAAAA`→`00110011`=0x33 (@502308) ; `MACA[R]`=`I11001R10AAAAAAA`→`001101R1`=0x35/0x37 (@492256) ; `POLY`=`I11000110AAAAAAA`→`00110110`=0x36 (@525232) ; `SQURS`=`I1100S101AAAAAAA`→`0011101S`=0x3A/0x3B (@556002), exec `src − Smem×Smem → src`. (c) *Commentaire vs code ?* Le code exécuté L9501-9505 n'a aucun sélecteur — `acc(bit8) += T*Smem` inconditionnel après le seul test `(op&0xFE00)==0x3800`. 14 opcodes / 16 faux et muets.

*Réserve sur le patch (pas sur le finding)* : le patch reprend `ahi = (int16_t)(s->a >> 16)` = A31−16, alors que SPRU172C @492256 dit explicitement « **A32−16 is used as a 17-bit operand for the multiplier** ». Le patch réintroduit donc le défaut que F2 reproche au code mort.

### F2 — 7 handlers 0x3xxx piégés dans `case 0xF:` — **CONFIRMÉ** (preuve renforcée)
La réfutation la plus plausible (« ils sont en fait sous un `if` gardé, ou atteints par fallthrough ») est écartée mécaniquement : L6469/6481/6493/6507/6519/6527/6544 sont toutes à **profondeur d'accolades 2**, dont la seule chaîne englobante est `switch (hi4) {` (L5960) puis le corps de fonction (L5035) — aucun `if` intermédiaire. Le label gouvernant est `case 0xF:` L5961 (label suivant : `case 0xE:` L8475). `op` est figé L5039 et `hi4` L5253, jamais réaffectés (grep sur 5034-6600 : aucune réassignation). Aucun label `goto` dans l'intervalle. Donc `(op & 0xFF00) == 0x3x00` sous `hi4 == 0xF` est identiquement faux. **Code mort avéré.**

*Deux précisions à porter au patch* : (i) la plage L6408-6552 est exacte — mais `MAX`/`MIN` (L6386-6406) et `ROLTC` (L6554-6569) l'encadrent et sont **vivants** (hi4==0xF) ; une ligne de trop les tue. (ii) Un balayage exhaustif des tests `0x4x00`/`0x5x00` hors `case 0x4`/`0x5` ne remonte **rien** : le sinistre est confiné à 0x2xxx/0x3xxx, il n'y a pas de second gisement de code mort dans ma plage.

### F3 — SQURA n'écrit pas T — **CONFIRMÉ**
Source directe, non ambiguë, `spru.txt`@555030 (p.4-163) : opcode `I1100S001AAAAAAA` (→0x38/0x39), « Execution **Smem → T** ; Smem × Smem + src → src », et description « This instruction **stores the data-memory value Smem in T**, then it squares Smem ». Corroboré par Table 2-14 @41043 « SQURA Smem,src src = src + Smem*Smem, **T = Smem** ». Code L9493-9500 : aucune écriture de `s->t`.

*Précaution de déploiement* : ce handler est sur un chemin documenté comme porteur (commentaire L9485-9492 : PROM0 0x76ff/0x7700 → `RCD LEQ`@0x75e8). Corriger T modifie T pour tous les MPY/MAC en aval — à mesurer seul, jamais groupé.

### F4 — bit 9 (S) ignoré sur 0x40-0x43 — **CONFIRMÉ**
`spru.txt`@575823 (p.4-187) : syntaxe 3 `SUB Smem, 16, src [, dst]`, opcode `I0010DS00AAAAAAA` → décodage : b15..b8 = `010000SD` = 0x40-0x43, **b9=S, b8=D** ; exécution 3 : `src − Smem << 16 → dst`. Code L9570-9577 : `dst_b = op8 & 0x01` sert à la fois de source et de destination. Le tableau du rapport (0x41 et 0x42 faux, 0x40/0x43 justes) est exact. Ce n'est pas un finding « au commentaire » : le commentaire L9558 est juste, c'est le code qui diverge.

### F5 — `RPT Smem` écrit BRC — **CONFIRMÉ, et sous-estimé**
`spru.txt`@541963 (p.4-146) : syntaxe 1, opcode `I00101110AAAAAAA` → `01000111` = 0x47, « Execution 1: **Smem → RC** », Status Bits None, exemple 1 `RPT DAT127` → **RC** = 000C. Le code L9599 écrit `s->brc`.

Contrôle décisif ajouté : les **six** autres points d'entrée RPT du fichier utilisent tous le même idiome — `rpt_count = N ; rpt_active = rpt_fresh = true ; s->pc += … ; return 0` (L7130-7133, L7637-7640, L8038-8041, **L8526-8529**, L8640-8643, L10779-10782), et le commentaire L8524-8525 en donne la raison exécutable : « *Must advance PC past RPT now and return 0 so the dispatcher re-executes the NEXT instruction (not RPT itself)* ». Le moteur L15948-15965 confirme : il `continue` **sans** avancer PC.

Conséquence non relevée par le rapport : avec `return consumed` au lieu de `return 0`, le `RPT Smem` **se ré-exécute lui-même** tant que le `rpt_count` périmé décrémente — donc `resolve_smem` L9597 est ré-appelé à chaque tour et **post-modifie l'AR à chaque fois**. Au bug de compteur et au clobber de BRC s'ajoute une corruption de pointeur. Gravité à relever de 2 vers 1.

### F6 — `LDM` sign-étend — **CONFIRMÉ**
`spru.txt`@481490 (p.4-73), verbatim : « `MMR → dst15−0` ; `000000h → dst39−16` … Status Bits **None** … This instruction is **not affected by the value of SXM** », et exemple 1 `LDM AR4,A` avec AR4=`FFFF` → `A = 00 0000 FFFF`. Opcode `I0010D001AAAAAAA` → `0100100D` = 0x48/0x49 ✔. Code L9607 `sext40((int16_t)val)` produit `FF FFFF FFFF`.

### F7 — MMR = `op & 0x7F`, indirect non résolu — **CONFIRMÉ (gravité 3 justifiée)**
La phrase de p.4-73 est bien celle qui tranche : « The nine MSBs of the effective address are cleared to 0 … regardless of the current value of DP **or the upper nine bits of ARx** » — l'adresse effective est calculée normalement (ARx inclus, post-mod inclus) puis masquée. Vérification du risque de régression du patch, **favorable** : la branche directe de `resolve_smem` (L4753-4757) retourne `(dp(s) << 7) | (opcode & 0x7F)`, donc `resolve_smem(...) & 0x7F ≡ op & 0x7F` — le patch est un **no-op exact** en mode direct, c'est-à-dire dans la forme d'usage courante `LDM AR4,A`. Seul le mode indirect change.
*Caveat à ajouter au patch* : `resolve_smem` pose `s->lk_used` sur les mods 0xC-0xF (L4681-4702), ce qui ferait passer LDM/PSHM à 2 mots et désynchroniserait le décode (patron n°4). TI ne donne à LDM **aucune** clause « add 1 word » → ces mods sont illégaux, mais le patch doit neutraliser `lk_used` plutôt que de s'en remettre à la légalité du flux.

### F8 — post-modification ±1 au lieu de ±2 sur DST — **CONFIRMÉ**
`spru.txt`@456507 (p.4-47), exemple 1 `DST B,*AR3+` : AR3 `0100 → 0102` avec la note « Because this instruction is a **long-operand** instruction, AR3 is **incremented by 2** » ; exemple 2 `DST B,*AR3−` : AR3 `0101 → 00FF` (−2). Code L9645 appelle `resolve_smem`, dont les post-mods sont `s->ar[cur_arp]--` / `++` (L4626/L4629), contre `-= 2` / `+= 2` dans `resolve_lmem` (L4780/L4781). Divergence réelle.

### F9 — ordre MSW/LSW pour Lmem à adresse impaire — **CONFIRMÉ**
C'était le finding le plus suspect (raisonnement d'apparence théorique). Il tient sur une preuve chiffrée directe et non ambiguë dans le texte extrait, `spru.txt`@456507 exemple 2 : `DST B,*AR3−`, **AR3 = 0101 (impaire)**, B = `00 6CAC BD90` → après : **`0101h = 6CAC`** (MSW à l'adresse effective) et **`0100h = BD90`**. Le code (`& 0xFFFE` puis MSW en `addr`, LSW en `addr+1`) produit l'inverse exact. Corroboré a contrario par `DLD *AR3+,B` @450896 (EA paire 0100 : `0100h=6CAC`, `0101h=BD90`) — les deux exemples ne sont conciliables que par la règle « MSW à l'EA, LSW à EA^1 ».

**Nuance de procédure, partiellement réfutée** : l'affirmation « à traiter en un seul patch atomique avec F8 » est trop forte. **F8 est autonome et sans risque** — `resolve_lmem` retourne déjà une adresse paire, pour laquelle l'ordre MSW/LSW actuel est conforme à TI ; F8 seul ne change que le pas de post-modification. C'est **F9** qui exige l'atomicité multi-sites (`resolve_lmem` + ses 8 appelants). Les découpler permet de mesurer F8 isolément, ce que l'atomicité imposée interdirait.

### F10 — SXM ignoré sur 0x40-0x45 — **CONFIRMÉ**
`spru.txt`@575823 (SUB) : « Status Bits **Affected by SXM** and OVM ». `spru.txt`@475867 (LD, p.4-67) : « Status Bits **Affected by SXM in all accumulator loads** », et syntaxe 3 `LD Smem,16,dst` = `I0010D010AAAAAAA` → `0100010D` = 0x44/0x45 ✔. Code L9576 et L9583 : `(int64_t)(int16_t)… << 16` inconditionnel = SXM figé à 1.

---

## Verdicts sur les NON-BUG revendiqués — tous **maintenus**, deux renforcés

- **0x50-0x5F dual-long** : re-vérifié indépendamment. `DSUBT` = `I1010D011AAAAAAA` → `0101110D` = 0x5C/0x5D (@459645), C16=1 : `Lmem31−16 − T → dst39−16`, `Lmem15−0 − T → dst15−0` ; C16=0 : `Lmem − T − T<<16 → dst`. `DSADT` (Table 2-5 @41636) : C16=1 `hi − T`, `lo + T`. Le code L9718-9719 (`sgn_hi = (dl_hi<=0x5B)?+1:-1`, `sgn_lo = (dl_hi<=0x5D)?-1:+1`) reproduit exactement les trois cas. La branche C16=0 traite DSUBT et DSADT identiquement (`lmem40 - tt40`) — **et c'est correct** : le manuel donne bien la même expression pour les deux en C16=0. La « correction » que le rapport apporte à la table projet (L64/L65) est donc juste en C16=1 et **inutile en C16=0**. `DLD` C16=1 (L9706) : TI dit « les 16 MSB sont chargés dans les **24 bits supérieurs** » — `(int64_t)lhi16 << 16` sign-étend bien dans les bits de garde ✔.
- **Bloc SFTA/SFTL mort (L9732-9759)** : NON-BUG maintenu, et la vérification manquante est faite — `SFTA` existe bien ailleurs, vivant, à `(op & 0xFCE0) == 0xF460` (L6712, dupliqué L7000). Le code mort ne masque donc aucun trou fonctionnel.
- **0x46 / 0x4B / 0x4C / 0x4D / 0x4E-0x4F**, **L9652 inatteignable**, **patron n°4 (longueurs) propre** : maintenus.

---

## Findings MANQUÉS par l'agent précédent

### M1 — 0x40-0x43 : règle de retenue **spécifique à la syntaxe 3** non implémentée — **CONFIRMÉ — gravité 3**
`spru.txt`@575823, immédiatement après les Status Bits de SUB : « **For instruction syntax 3**, if the result of the subtraction generates a borrow, the carry bit, C, is cleared to 0; otherwise, C is not affected. » Ce n'est pas la règle ALU générique — c'est une clause dédiée à `SUB Smem,16,src[,dst]`, précisément la plage 0x40-0x43. Le code L9573-9578 ne touche jamais `ST0_C`. Le patch F4 proposé par l'agent ne la corrige pas non plus. Impact : tout `RC`/`BC` conditionné sur C après un `SUB Smem,16` lit une retenue périmée.

### M2 — aucun bit d'état C/OV/OVM sur l'intégralité de 0x30-0x5F — **CONFIRMÉ — gravité 3 (systémique)**
Comptage : `grep` de `ST0_C` / `ST0_OV` / `ST1_OVM` sur L9480-9762 → **0 occurrence**. Or `DADD`/`DSUB`/`DRSUB` sont « Affects **C and OVsrc** » (@41636, @459645), `SQURA`/`SQURS` « Affected by **OVM**, Affects OVsrc » (@555030, @556002), `SUB Smem,16` « Affects C and OV ». Toute la famille dual-long et la famille carré/accumulation produisent donc des drapeaux figés. À traiter comme un lot séparé, pas opcode par opcode.

### M3 — `resolve_smem` en mode direct ignore CPL — **PROBABLE — gravité 3 (systémique, hors plage stricte)**
L4753-4757 calcule toujours `(dp(s) << 7) | (opcode & 0x7F)`. Le C54x, lorsque `ST1.CPL = 1`, adresse l'opérande direct **relativement à SP**, pas à DP. Contrôle : `CPL` n'est **pas défini** dans `calypso_c54x.h` et `ST1_CPL` compte **0 occurrence** dans le `.c`. Tous les Smem de ma plage (0x30-0x5F) en dépendent. Classé PROBABLE et non CONFIRMÉ parce que je ne peux pas établir sans exécuter la pile (interdit) si le firmware DSP arme jamais CPL ; si CPL reste à 0, c'est inoffensif.

---

## Ordre de traitement révisé

`F5` (compteur RC **+ corruption d'AR par ré-exécution** — le plus destructeur, et isolé) → `F1+F2` (patch unique, en veillant à ne pas déborder sur MAX/MIN L6386-6406 ni ROLTC L6554-6569, et en portant A**32**−16) → `F6` (LDM, une ligne, risque nul) → `F4` (+`M1` dans le même patch, même plage) → `F8` **seul** (autonome, contrairement à ce qu'affirme le rapport) → `F3` (à mesurer isolément : chemin porteur documenté) → `F10` → `F9` (atomique multi-sites) → `F7` (avec neutralisation de `lk_used`) → `M2`/`M3` en lot séparé.

Fichiers : `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (conteneur `osmo-operator-1`), copie de travail `/root/.claude/jobs/26578783/tmp/calypso_c54x.c`, manuel `/root/.claude/jobs/26578783/tmp/spru.txt`. **Aucun fichier modifié.**

---

# AUDIT 0x60-0x8F  (REFUTATION MANQUANTE — workflow tue)

# AUDIT hi8 0x60–0x8F — `calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, 16865 l.)

Sources citées : `L####` = ligne de `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` · `map:L##` = `doc/opcodes/tic54x_hi8_map.md` · `p.4-x` = SPRU172C (vérifié sur `spru172c.pdf` avec `pdftotext -layout`, pages PDF = page manuel + 114).

---

## F1 — 0x62–0x67 : MPY/MAC `Smem,#lk` avalés en `LD` 1-mot — **CONFIRMÉ — gravité 1**

**Code** : L9131-9139, catch-all `if ((op & 0xF800) == 0x6000)`. Atteignable uniquement pour **0x62xx–0x67xx** (0x60 CMPM capté L9073, 0x61 BITF L9083).

**Actuel** : `dst_acc=(op>>9)&1`, `shift=(op>>8)&1`, `LD Smem,dst` (`if(shift) v<<=16`), **1 mot** (`return consumed + s->lk_used`, `consumed=1`). Le `#lk` du mot 2 est donc exécuté comme opcode parasite à chaque site. Le "shift 16" est une invention : `LD Smem,16,dst` est 0x4400, pas 0x6xxx.

**Attendu** :
- 0x62/0x63 `MPY Smem,#lk,dst` — SPRU p.4-101 syntaxe 3, opcode `I0110D100AAAAAAA` → **0x6200/0xFE00**, bit 8 = dst ; « Syntaxes 3 and 4: 2 words » ; exécution `Smem × lk → dst ; Smem → T` (map:L68).
- 0x64–0x67 `MAC Smem,#lk,src[,dst]` — SPRU p.4-82 syntaxe 4, opcode `I0110DS10AAAAAAA` → **0x6400/0xFC00**, bit 9 = src, bit 8 = dst ; 2 mots ; `src + Smem×lk → dst ; Smem → T` (map:L69).

**Patch proposé** (insérer **avant** L9131 ; le catch-all L9131 devient mort et peut être transformé en log) :

```c
        /* 0x6200/0xFE00 : MPY Smem,#lk,dst (2 mots + lk Smem) — SPRU p.4-101 syn.3 */
        if ((op & 0xFE00) == 0x6200) {
            int dst_b = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mv = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)mv * (int64_t)(int16_t)lk;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            if (dst_b) s->b = sext40(prod); else s->a = sext40(prod);
            s->t = mv;
            consumed = 2;
            return consumed + s->lk_used;
        }
        /* 0x6400/0xFC00 : MAC Smem,#lk,src[,dst] (2 mots + lk) — SPRU p.4-82 syn.4 */
        if ((op & 0xFC00) == 0x6400) {
            int src_b = (op >> 9) & 1, dst_b = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t lk = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mv = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)mv * (int64_t)(int16_t)lk;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t res = sext40((src_b ? s->b : s->a) + prod);
            if (dst_b) s->b = res; else s->a = res;
            s->t = mv;
            consumed = 2;
            return consumed + s->lk_used;
        }
```

---

## F2 — 0x78–0x7D : MACP/MACD/MVPD/MVDP avalés en `STH` 1-mot — **CONFIRMÉ — gravité 1**

**Code** : L9053-9062, catch-all `if ((op & 0xF800) == 0x7800)`. 0x7E/0x7F sont captés plus haut (L8736/L8759) → ce catch-all ne reçoit **que 0x78–0x7D**, toutes des instructions **2 mots**. Le vrai `STH src,Smem` est 0x82/0x83 et il est déjà traité (L10213/L10246) : ce handler ne sert donc aucune STH légitime.

**Attendu** (tous « Words 2 words », +1 si Smem long-offset/absolu) :

| op | mnémo | opcode SPRU | exécution |
|---|---|---|---|
| 0x78/0x79 | `MACP Smem,pmad,src` | `0111100S IAAAAAAA` (PDF p.203 = 4-89) | `src += Smem×Pmem[pmad] ; Smem→T` |
| 0x7A/0x7B | `MACD Smem,pmad,src` | `I1110S101AAAAAAA` → 0x7A00/0xFE00 (p.4-87) | idem + `Smem→Smem+1` |
| 0x7C | `MVPD pmad,Smem` | `I11100011AAAAAAA` → 0x7C00/0xFF00 (p.4-117) | `Pmem[pmad] → Smem` |
| 0x7D | `MVDP Smem,pmad` | `I11101011AAAAAAA` → 0x7D00/0xFF00 (p.4-111) | `Smem → Pmem[pmad]` |

map:L86-89. Consommer 1 mot au lieu de 2 décale le PC de +1 à chaque site et fait exécuter le `pmad` comme opcode.

**Patch proposé** (remplacer le bloc L9053-9062) :

```c
        /* 0x7800/0xFC00 : MACP (b9=0) / MACD (b9=1) Smem,pmad,src — 2 mots + lk */
        if ((op & 0xFC00) == 0x7800) {
            int is_macd = (op & 0x0200) != 0;
            int src_b   = (op >> 8) & 1;
            addr = resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            uint16_t mv = data_read(s, addr);
            int64_t prod = (int64_t)(int16_t)mv * (int64_t)(int16_t)prog_read(s, pmad);
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t acc = sext40((src_b ? s->b : s->a) + prod);
            if (src_b) s->b = acc; else s->a = acc;
            s->t = mv;
            if (is_macd) data_write(s, (uint16_t)(addr + 1), mv);
            consumed = 2;
            return consumed + s->lk_used;
        }
        /* 0x7C00 : MVPD pmad,Smem — 2 mots + lk */
        if ((op & 0xFF00) == 0x7C00) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            data_write(s, addr, prog_read(s, pmad));
            consumed = 2;
            return consumed + s->lk_used;
        }
        /* 0x7D00 : MVDP Smem,pmad — 2 mots + lk */
        if ((op & 0xFF00) == 0x7D00) {
            addr = resolve_smem(s, op, &ind);
            uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
            prog_write(s, pmad, data_read(s, addr));
            consumed = 2;
            return consumed + s->lk_used;
        }
```

---

## F3 — 0x85 décodé `MVPD` 2-mots au lieu de `STL B,ASM,Smem` 1-mot — **CONFIRMÉ — gravité 1**

**Code** : L10142-10149 `if (hi8 == 0x85) { … op2 = prog_fetch(pc+1); consumed = 2; data_write(addr, prog_read(op2)); }`. Atteint avant tout autre test pour 0x85.

**Attendu** : SPRU p.4-172 STL **syntaxe 2** `STL src, ASM, Smem`, opcode `I0001S010AAAAAAA` → **0x8400/0xFE00**, bit 8 = src ; « Syntaxes 1, 2 and 3: 1 word ». Donc 0x84 = STL A,ASM et **0x85 = STL B,ASM** (map:L94). Le vrai MVPD est 0x7C00 (F2).

**Conséquence** : PC +1 à chaque `STL B,ASM,*ARx` ; B n'est jamais stockée ; en plus un mot programme est écrit en RAM data.

**Patron n°6** : le commentaire L10176-10180 affirme « `stl 0x8400 / 0xFE00 → 0x84..0x85 STL src,ASM,Smem (with shift) [FAIT]` » — **faux** : seul 0x84 est implémenté (L10254) et il est de toute façon shadowé pour 0x85 par le handler MVPD placé 112 lignes plus haut. Ne pas se fier au commentaire.

**Patch proposé** :

```c
-        /* 85xx: MVPD pmad, Smem (prog→data, different encoding) */
-        if (hi8 == 0x85) {
-            addr = resolve_smem(s, op, &ind);
-            op2 = prog_fetch(s, s->pc + 1);
-            consumed = 2;
-            data_write(s, addr, prog_read(s, op2));
-            return consumed + s->lk_used;
-        }
```
et L10254 :
```c
-        if (hi8 == 0x84) {
+        /* 0x84/0x85 : STL src,ASM,Smem — SPRU p.4-172 syn.2, bit8 = src, 1 mot (+lk) */
+        if (hi8 == 0x84 || hi8 == 0x85) {
             addr = resolve_smem(s, op, &ind);
             int shift = asm_shift(s);
-            int64_t v = s->a;
+            int64_t v = (hi8 & 1) ? s->b : s->a;
             if (shift >= 0) v <<= shift; else v >>= (-shift);
             data_write(s, addr, (uint16_t)(v & 0xFFFF));
             return consumed + s->lk_used;
         }
```

---

## F4 — 0x8D décodé `MVDD` 2-mots au lieu de `ST TRN,Smem` 1-mot — **CONFIRMÉ — gravité 1**

**Code** : L10234-10241 `if (hi8 == 0x8D) { addr=resolve_smem(...); op2=prog_fetch(pc+1); consumed=2; data_write(op2, data_read(addr)); }`.

**Attendu** : SPRU p.4-167 `ST` **syntaxe 2** `ST TRN, Smem`, opcode `I00011011AAAAAAA` → **0x8D00/0xFF00**, « Syntaxes 1 and 2: 1 word » (+1 si long-offset/absolu) ; exécution `TRN → Smem` (map:L100). Le vrai `MVDD Xmem,Ymem` est 0xE500 (map:L133). Exemple manuel : `ST TRN, 5` avec TRN=1234 → data[0205h]=1234.

**Conséquence** : double dégât — PC +1 (le mot suivant est consommé comme `op2`) **et** une écriture data à une adresse prise dans le flux d'instructions. TRN est écrit par CMPS (L10022, chemin recherche de pic FCCH) : `ST TRN` est le consommateur naturel de ce résultat.

**Patch proposé** :

```c
-        /* 8Dxx: MVDD Smem, Smem */
-        if (hi8 == 0x8D) {
-            addr = resolve_smem(s, op, &ind);
-            op2 = prog_fetch(s, s->pc + 1);
-            consumed = 2;
-            data_write(s, op2, data_read(s, addr));
-            return consumed + s->lk_used;
-        }
+        /* 0x8Dxx : ST TRN, Smem — SPRU p.4-167 syn.2, opcode I00011011AAAAAAA
+         * = 0x8D00/0xFF00, 1 mot (+1 si long-offset/absolu). MVDD = 0xE500. */
+        if (hi8 == 0x8D) {
+            addr = resolve_smem(s, op, &ind);
+            data_write(s, addr, s->trn);
+            return consumed + s->lk_used;
+        }
```

---

## F5 — 0x6F sous-cas 3 (`STH src,SHIFT,Smem`) : extraction du high **avant** le décalage — **CONFIRMÉ — gravité 2**

**Code** : L9282-9289.
```c
int16_t high = (int16_t)((src >> 16) & 0xFFFF);
int64_t shifted = (shift >= 0) ? ((int64_t)high << shift) : ((int64_t)high >> (-shift));
data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
```

**Attendu** : SPRU p.4-169, exécution `src << SHIFT − 16 → Smem`, et description : *« The src is shifted left as specified by ASM, SHFT, or SHIFT and bits 31−16 of the shifted value are stored in data memory Smem or Xmem »*. Donc **décalage d'abord sur les 40 bits, extraction 31:16 ensuite**.

**Contre-exemples discriminants** :
- `A = 00 0000 8000`, SHIFT=+1 → attendu `(A<<1)>>16 = 0x0001` ; code : `high=0x0000` → **0x0000**.
- `A = 01 0000 0000` (bit de garde), SHIFT=−1 → attendu `0x8000` ; code : `high=0x0000` → **0x0000**.

Les exemples 2 et 3 du manuel (SHIFT négatifs sur `FF 8421 1234`) ne discriminent pas : ils donnent FF84/F842 dans les deux modèles — d'où le fait que le bug ait survécu.

**NON-BUG associé** : le handler ASM 0x86/0x87 (L10175-10183) fait bien `v <<= shift` puis `>>16` — **correct**. Deux implémentations de la même instruction, une juste une fausse.

**Patch proposé** :
```c
             case 3: { /* STH SRC1,SHIFT,Smem : Smem = (SRC1 << SHIFT) >> 16 (p.4-169) */
                 int64_t src = dst_b ? s->b : s->a;
-                int16_t high = (int16_t)((src >> 16) & 0xFFFF);
-                int64_t shifted = (shift >= 0) ? ((int64_t)high << shift)
-                                               : ((int64_t)high >> (-shift));
-                data_write(s, addr, (uint16_t)(shifted & 0xFFFF));
+                int64_t shifted = (shift >= 0) ? (src << shift) : (src >> (-shift));
+                data_write(s, addr, (uint16_t)((shifted >> 16) & 0xFFFF));
                 break;
             }
```

---

## F6 — BANZ/BANZD 0x6C/0x6E : valeur testée fausse pour les modes long-offset — **CONFIRMÉ — gravité 2**

**Code** : L9201-9203 (BANZ) et L9215-9217 (BANZD) :
```c
int nar = op & 0x07;
uint16_t pre = s->ar[nar];
resolve_smem(s, op, &ind);
...
if (pre != 0) { s->pc = pmad; return 0; }
```

**Attendu** — SPRU172C p.4-16/4-17, opcode `0110 11Z0 IAAAAAAA` (0x6C00 / 0x6E00), deux exemples qui, ensemble, fixent la règle :
- **Exemple 2** : `BANZ 2000h, *AR3–`, AR3=0000 → PC 1000→**1002** (pas de branche), AR3→FFFF. ⇒ pour les post-modify, la valeur testée est celle **AVANT** modification.
- **Exemple 3** : `BANZ 2000h, *AR3(–1)`, AR3=0001 → PC 1000→**1003** (pas de branche), AR3 **inchangé**. ⇒ pour `*ARx(lk)`, la valeur testée est **AR3 + lk = 0**, pas AR3.

Règle unifiée : **la valeur testée est l'adresse effective produite par le mode d'adressage**, c'est-à-dire exactement la valeur de retour de `resolve_smem` (qui rend AR-pré pour MOD 0-B, AR+lk pour 0xC, AR-post pour 0xD/0xE pré-modify, lk pour 0xF).

**Divergence actuelle** : correcte pour MOD 0x0-0xB ; **fausse pour MOD 0xC / 0xD / 0xE / 0xF** (les formes 3-mots). Symptôme : compteur de boucle décalé d'une itération, ou boucle infinie/court-circuitée sur les `BANZ *ARx(lk)`.

**Patron n°6 (documentation vs code)** : `doc/opcodes/0x68_0x6F.md` §0x6Cxx v2 dit *« test (ARx) ≠ 0 (after the indirect mode has been applied) »* avec un pseudo-code qui teste `s->ar[arp]` **après** `resolve_smem` ; `map:L74` dit *« Le mode indirect est appliqué avant le test »*. **Les deux sont faux tels qu'écrits** : appliqués littéralement ils cassent l'Exemple 2 (AR3 FFFF ≠ 0 → branche prise à tort). Le patch ci-dessous satisfait Ex.2 **et** Ex.3.

**Patch proposé** (identique aux deux sites L9195-9210 et L9212-9226) :
```c
-            int nar = op & 0x07;
-            uint16_t pre = s->ar[nar];
-            resolve_smem(s, op, &ind);
+            /* SPRU172C p.4-16/4-17 : la valeur testee est l'adresse effective
+             * produite par le mode d'adressage, pas le contenu brut d'ARx.
+             *  Ex.2 `*AR3-`   AR3=0000 -> pas de branche (AR3->FFFF) : pre-modify
+             *  Ex.3 `*AR3(-1)` AR3=0001 -> pas de branche, AR3 inchange : AR3+lk=0
+             * resolve_smem renvoie exactement cette valeur dans les deux cas. */
+            uint16_t test_val = resolve_smem(s, op, &ind);
             uint16_t pmad = prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
             consumed = 2;
-            if (pre != 0) {
+            if (test_val != 0) {
```
(le `int nar` devient inutilisé sur les deux sites)

---

## F7 — Opérande MMR : `op & 0x7F` ignore le bit I (adressage indirect) — **CONFIRMÉ (STLM) / PROBABLE (autres) — gravité 2**

**Code (dans la plage)** : L8806 `STM #lk,MMR` · L8919 `MVMD MMR,dmad` · L9030 `MVDM dmad,MMR` (`op & 0x00FF`) · L9913 `POPM MMR` · L9955 `STLM src,MMR`.
Hors plage mais même défaut : L9605/L9612 (`LDM`/`PSHM`, famille 0x4).

**Attendu** — SPRU p.4-175 `STLM` : *« The nine MSBs of the effective address are cleared to 0 regardless of the current value of DP or of the upper nine bits of ARx »*, avec **Exemple 2** décisif :
```
STLM B, *AR1–     B = FF 8421 1234, AR1 = 3F17
→ AR7 (MMR @0x17) = 1234 ,  AR1 = 0016
```
L'opérande MMR est donc un **Smem complet** (bit 7 = I, bits 6:0 = A) : résolution directe **ou indirecte avec post-modify de l'AR**, puis masquage des 9 bits de poids fort. Les opcodes le confirment : POPM `I00010101AAAAAAA` (0x8A00), STLM `I0001S001AAAAAAA` (0x8800/0xFE00), STM `I11101110AAAAAAA` (0x7700), MVDM `I11100100AAAAAAA` (0x7200), MVMD `I11101100AAAAAAA` (0x7300).

**Actuel** : sur la forme indirecte (`*AR1–` = octet bas 0x89), `op & 0x7F` rend `0x09` → écrit un **MMR arbitraire** (ici SP-1/AR…) au lieu de AR7, et n'applique **jamais** le post-modify de l'AR. Le cas L9030 (`op & 0x00FF`) est pire : il conserve le bit 7 → écrit en data `0x80..0xFF`, hors page MMR.

**Note de longueur** : SPRU donne « 2 words » **sans** clause « add 1 word » pour STM/MVDM/MVMD, et « 1 word » sans clause pour POPM/STLM ⇒ les MOD 0xC-0xF sont illégaux sur ces opérandes, `resolve_smem` ne posera pas `lk_used` en code valide.

**Patch proposé** — helper à placer après `resolve_smem` (≈L4757), puis substitution aux 5 sites :
```c
/* Operande MMR (STM/STLM/POPM/PSHM/LDM/MVDM/MVMD) : resolution Smem complete
 * (directe OU indirecte avec post-modify AR), puis 9 MSB forces a 0.
 * SPRU172C p.4-175 STLM Ex.2 : `STLM B,*AR1-` AR1=3F17 -> ecrit AR7 (0x17),
 * AR1 -> 0016. */
static inline uint16_t resolve_mmr(C54xState *s, uint16_t op)
{
    bool ind_mmr;
    return (uint16_t)(resolve_smem(s, op, &ind_mmr) & 0x7F);
}
```
```c
-            uint8_t mmr = op & 0x7F;            /* L8806  STM  */
+            uint16_t mmr = resolve_mmr(s, op);
-            int mmr = op & 0x7F;                /* L8919  MVMD */
+            uint16_t mmr = resolve_mmr(s, op);
-                uint16_t mmr  = op & 0x00FF;    /* L9030  MVDM */
+                uint16_t mmr  = resolve_mmr(s, op);
-            uint16_t mmr = op & 0x7F;           /* L9913  POPM */
+            uint16_t mmr = resolve_mmr(s, op);
-            int mmr = op & 0x7F;                /* L9955  STLM */
+            uint16_t mmr = resolve_mmr(s, op);
```
⚠️ Ce patch **ajoute** un effet de bord AR (post-modify) sur des chemins de boot aujourd'hui fonctionnels (POPM ST1 / STLM B,AR1 du bootloader PROM0 0xb42d) : à mesurer **isolément** des F1-F5.

---

## F8 — `ADDM #lk,Smem` (0x6B) : pas de saturation OVM, pas de C/OV — **CONFIRMÉ — gravité 3**

**Code** : L9185-9193, wrap 16 bits pur ; le `TODO` L9191 est justifié.

**Attendu** : SPRU p.4-9 « Status Bits: **Affected by OVM and SXM ; Affects C and OVA** », Exemple 2 :
```
OVM=1, SXM=1 : data[0100h] = 8007h  +  lk = FFF8h  ->  8000h   (saturation)
```
Le code produit `0x7FFF` dans ce cas (wrap). Divergence observable seulement quand OVM=1.

**Patch proposé** :
```c
             addr = resolve_smem(s, op, &ind);
             int16_t lk = (int16_t)prog_fetch(s, s->pc + 1 + (s->lk_used ? 1 : 0));
-            uint16_t v = data_read(s, addr);
-            data_write(s, addr, (uint16_t)((int16_t)v + lk));
+            uint16_t v = data_read(s, addr);
+            int32_t sum = (int32_t)(int16_t)v + (int32_t)lk;
+            /* C = retenue non signee sur 16 bits ; OVA = depassement signe. */
+            if (((uint32_t)v + (uint16_t)lk) > 0xFFFF) s->st0 |=  ST0_C;
+            else                                       s->st0 &= ~ST0_C;
+            if (sum > 32767 || sum < -32768) {
+                s->st0 |= ST0_OVA;
+                if (s->st1 & ST1_OVM) sum = (sum > 0) ? 32767 : -32768;  /* p.4-9 Ex.2 */
+            }
+            data_write(s, addr, (uint16_t)sum);
             consumed = 2;
```

---

## Remarque transversale (hors plage stricte, signalée une fois)

`resolve_smem` L4749 met à jour ARP **inconditionnellement** (`s->st0 = ... | (nar << ST0_ARP_SHIFT)`). SPRU p.4-92 (MAR) : « Status Bits: Affected by CMPT / **Affects ARP (if CMPT = 1)** » ; en mode non-compatibilité (CMPT=0, `ST1_CMPT` bit 5) l'ARP n'est pas modifié. **PROBABLE, gravité 3** (inoffensif tant que rien ne lit ARP, puisque l'adressage indirect utilise ARF bits 2:0).

---

## Familles CORRECTES de la plage (vérifiées, aucune divergence)

| op | handler | vérification |
|---|---|---|
| 0x60 `CMPM Smem,#lk` | L9073 | opcode `0110 0000 IAAAAAAA` (PDF p.147=4-33), TC = égalité, 2 mots + lk ✓ |
| 0x61 `BITF Smem,#lk` | L9083 | `I01101000AAAAAAA` → 0x6100 (p.4-22), TC=(Smem&lk)≠0, exemples 1/2 reproduits ✓ |
| 0x68 `ANDM` / 0x69 `ORM` / 0x6A `XORM` | L9158 / L9167 / L9176 | opcodes PDF 127/239/317 = 0x6800/0x6900/0x6A00, « Status Bits: None », 2 mots + lk ✓ |
| 0x6D `MAR Smem` | L8767 | 0x6D00/0xFF00, 1 mot (+lk), side-effect AR seul ✓ (cf. remarque ARP/CMPT) |
| 0x6F sous-cas 0/1/2/4 | L9251-9295 | mot 2 décodé bit à bit contre SPRU : STL syn.4 `10000S011T00FIHS` → bits[7:5]=100 ; STH syn.4 `00000S011T11FIHS` → bits[7:5]=011 ; b9=SRC, b8=DST/SRC1, SHIFT 5 bits signés ✓ (seul le sous-cas 3 est faux, cf. F5) |
| 0x70 `MVKD` / 0x71 `MVDK` | L8943 / L8985 | sens et longueur (2+lk) corrects ✓ |
| 0x72 `MVDM` / 0x73 `MVMD` | L9028 / L8918 | opcodes et longueur 2 mots corrects ✓ (opérande MMR : cf. F7) |
| 0x74 `PORTR` / 0x75 `PORTW` | L8875 / L8868 | `I11100010AAAAAAA` → 0x7400, 2 mots (+1 si Smem abs) ✓ |
| 0x76 `ST #lk,Smem` | L8790 | `I11100110AAAAAAA` → 0x7600, 2 mots + lk ✓ |
| 0x77 `STM #lk,MMR` | L8805 | `I11101110AAAAAAA` → 0x7700, 2 mots fixes ✓ |
| 0x7E `READA` / 0x7F `WRITA` | L8736 / L8759 | 1 mot ✓ |
| 0x80/0x81 `STL A/B,Smem` | L9975 / L10207 | `I0001S000AAAAAAA` → 0x8000/0xFE00 ✓ |
| 0x82/0x83 `STH A/B,Smem` | L10213 / L10246 | `I0001S100AAAAAAA` → 0x8200/0xFE00 ✓ |
| 0x84 `STL A,ASM,Smem` | L10254 | ✓ (0x85 manquant : F3) |
| 0x86/0x87 `STH A/B,ASM,Smem` | L10175 | décalage-puis-extraction conforme p.4-169 ✓ |
| 0x88/0x89 `STLM src,MMR` | L9953 | `I0001S001AAAAAAA` → 0x8800/0xFE00, bit8=src, 1 mot ✓ |
| 0x8A `POPM MMR` | L9912 | `I00010101AAAAAAA` → 0x8A00, TOS→MMR, SP++ ✓ |
| 0x8B `POPD Smem` | L9968 | `I00011101AAAAAAA` → 0x8B00, 1 mot (+lk) ✓ |
| 0x8C `ST T,Smem` | L9989 | `I00010011AAAAAAA` → 0x8C00, 1 mot ✓ |
| 0x8E/0x8F `CMPS src,Smem` | L10022 | `I0001S111AAAAAAA` → 0x8E00/0xFE00 ; TRN<<1 puis bit0, TC ; validé sur l'exemple A=00 2345 7899, TRN 4444→8889, TC=1 ✓ |

**Code mort dans la plage** (confirmé, sans effet) : L10219 (2ᵉ `hi8==0x89`, ombré par L9953) · L10227 (stub `0x8B`, ombré par L9968) · L10045 (`if (0 && hi8 == 0x8F)`) · L9035 (2ᵉ `0x7300` gaté, ombré par L8918) · L9304 (`(op&0xF800)==0x6800`, désormais inatteignable).

**Ordre d'application recommandé** (un correctif à la fois, chacun falsifiable) : F3 → F4 → F2 → F1 (les quatre bugs de LONGUEUR, qui désynchronisent le décode), puis F5 (sortie STH du chemin démod), puis F6, puis F7 (risque de régression sur le boot), puis F8.

---

# AUDIT 0x90-0xBF

# AUDIT hi8 0x90–0xBF — `calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, copie locale `/root/.claude/jobs/26578783/tmp/calypso_c54x.c`, identique au conteneur)

**Résumé exécutif** : la plage 0x90–0xBF est la plage la plus dégradée du décodeur. Le manuel donne **48 opcodes, tous de 1 mot** (colonne W = `1` dans Tab. 2‑1/2‑3/2‑4/2‑21/2‑22/2‑23 ; aucun n'accepte Smem donc aucun `+1 word long-offset`). Le décodeur en exécute **12 en 2 mots** (0x94, 0x95, 0x96, 0x97, 0xA2, 0xA3, 0xA8, 0xA9, 0xAC, 0xAD, 0xAE, 0xAF) → désynchronisation du flux de décode (gravité 1), et **31 autres** avec une sémantique fausse silencieuse. Aucun `goto unimpl` n'est atteignable dans la plage : **tout est avalé sans trace**.

Vérification d'encodage : permutation de la clé de lecture appliquée aux blocs `Opcode` de `spru.txt` (194 en‑têtes `0123456789101112131415` décodés programmatiquement). Résultats bruts→décodés utilisés ci‑dessous, tous concordants avec `tic54x_hi8_map.md` L102–L119.

---

## A. Gravité 1 — LONGUEUR fausse (désynchronise le décode)

### F1 — 0x94 / 0x95 « MVDK / MVKD » — L9794‑9809 — **CONFIRMÉ**
- **Actuel** : `hi8==0x94` → MVDK Smem,dmad **2 mots** (L9795‑9801) ; `hi8==0x95` → MVKD dmad,Smem **2 mots** (L9803‑9809).
- **Attendu** : `LD Xmem, SHFT, dst` — `1001010D XXXXSHFT` = 0x9400/0xFE00, **1 mot**, `dst = Xmem << SHFT` (SHFT 0..15 non signé, bits 3‑0). Source : map:L104 ; `spru.txt` bloc LD, brut `X1001D010TXXFHSX` → `1001010DXXXXSHFT` ; table 2‑3 `LDXmem,SHFT,dstdst=Xmem<<SHFT113A4-66` (W=1).
- Les vrais MVKD/MVDK sont **0x70/0x71** (map:L78‑79), déjà décodés dans `case 0x6/0x7`. Ces handlers 0x9x sont des fantômes purs.
- **Patch** : remplacer les deux blocs L9795‑9809 par
```c
        /* 0x94/0x95 : LD Xmem, SHFT, dst — 1 MOT (SPRU172C p.4-66) */
        if (hi8 == 0x94 || hi8 == 0x95) {
            uint16_t xa = resolve_xmem(s, op);          /* Xmem = bits 7-4 */
            int shft = op & 0x0F;                        /* SHFT 0..15 */
            int64_t v = (int64_t)(int16_t)data_read(s, xa) << shft;
            if (hi8 & 1) s->b = sext40(v); else s->a = sext40(v);
            return 1;
        }
```

### F2 — 0x96 « MVDP » — L9810‑9817 — **CONFIRMÉ**
- **Actuel** : `s->prog[op2] = data[Smem]`, **2 mots**.
- **Attendu** : `BIT Xmem, BITC` — `10010110 XXXXBITC` = 0x9600/0xFF00, **1 mot**, `TC = Xmem[15 − BITC]`. Source : map:L105 ; `spru.txt` brut `X10010110CXXTIBX` → `10010110XXXXBITC`, exécution `Xmem15−BITC → TC`, `Words 1 word`, `Status Bits Affects TC`. Le vrai MVDP est **0x7D** (map:L89).
- **Patch** :
```c
        if (hi8 == 0x96) {                    /* BIT Xmem, BITC — 1 mot */
            uint16_t xa = resolve_xmem(s, op);
            int bitc = op & 0x0F;
            uint16_t v = data_read(s, xa);
            if ((v >> (15 - bitc)) & 1) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
            return 1;
        }
```

### F3 — 0x97 « ST #lk, Smem » — L10270‑10277 — **CONFIRMÉ**
- **Actuel** : atteignable (aucun test antérieur ne capte 0x97) ; consomme **2 mots** et écrit `op2` dans Smem.
- **Attendu** : **0x97 est NON ASSIGNÉ**. Le balayage exhaustif des blocs `Opcode` de `spru.txt` ne produit aucun `10010111…` ; absent de `tic54x_hi8_map.md` (saut L105→L106). Le vrai `ST #lk, Smem` est 0x76 (map:L84). Gravité 1 : un 0x97xx rencontré mange l'opcode suivant sans aucun log.
- **Patch** : supprimer le bloc et le laisser tomber sur `goto unimpl` (L10278), ou :
```c
        if (hi8 == 0x97) goto unimpl;   /* 0x97 non assigné (SPRU172C) */
```

### F4 — 0xA2 / 0xA3 « ADD/SUB #lk » — L10578‑10597 — **CONFIRMÉ**
- **Actuel** : `op2 = prog_fetch(pc+1); consumed = 2;` puis `acc ± (lk<<16)`, `dst = op & 1`.
- **Attendu** : `SUB Xmem, Ymem, dst` — `1010001D XXXXYYYY` = 0xA200/0xFE00, **1 mot**, `dst = Xmem<<16 − Ymem<<16`. Source : map:L112 ; `spru.txt` décodé `1010001DXXXXYYYY` ; Tab. 2‑1 `SUBXmem,Ymem,dstdst=Xmem<<16−Ymem<<16` **117** (W=1). Les vrais `ADD/SUB #lk,16` sont en 0xF060/0xF061 (§F0).
- **Patch** : remplacer les deux blocs par un handler dual‑operand (cf. F5 pour le squelette Xmem/Ymem), `dst = hi8 & 1`, `dst = (Xmem<<16) − (Ymem<<16)`.

### F5 — 0xA8 / 0xA9 « AND #lk » — L10456‑10464 — **CONFIRMÉ**
- **Actuel** : 2 mots, `acc &= (op2 << 16)` (et `dst = op & 1` = bit 0 du mot, pas un champ d'opcode).
- **Attendu** : `LD Xmem, dst || MAC[R] Ymem [, dst_]` — `101010RD XXXXYYYY` = 0xA800/0xFC00, **1 mot** (R = bit 9, D = bit 8, `dst_` = l'autre accumulateur). Source : `spru.txt` brut `X0101DR01YXXYYYX` → `101010RDXXXXYYYY`, exécution `Xmem<<16 → dst(31−16)` ‖ `Ymem × T + dst_ → dst_`, `Words 1 word Cycles 1 cycle Class 7` ; map:L115 (imprécis mais bon masque). Le vrai `AND #lk` est 0xF030/0xF130.
- **Patch** (squelette réutilisable pour 0xA8‑0xAF) :
```c
        if (hi8 >= 0xA8 && hi8 <= 0xAF) {   /* LD Xmem,dst || MAC[R]/MAS[R] Ymem — 1 MOT */
            int xar = ((op >> 4) & 3) + 2, yar = (op & 3) + 2;
            int xmod = (op >> 6) & 3,      ymod = (op >> 2) & 3;
            uint16_t xv = data_read(s, s->ar[xar]), yv = data_read(s, s->ar[yar]);
            switch (xmod) { case 1: s->ar[xar]--; break; case 2: s->ar[xar]++; break;
              case 3: s->ar[xar] = c54x_circ_ref(s->ar[xar], +(int16_t)s->ar[0], s->bk); break; }
            switch (ymod) { case 1: s->ar[yar]--; break; case 2: s->ar[yar]++; break;
              case 3: s->ar[yar] = c54x_circ_ref(s->ar[yar], +(int16_t)s->ar[0], s->bk); break; }
            int rnd = (hi8 >> 1) & 1;          /* R = bit9 */
            int d   = hi8 & 1;                 /* D = bit8 : 0=A, 1=B */
            int sub = (hi8 >= 0xAC);           /* 0xAC-AF = MAS[R] */
            int64_t prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)yv;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t *dstp  = d ? &s->b : &s->a;   /* dst  = LD  */
            int64_t *dstp_ = d ? &s->a : &s->b;   /* dst_ = MAC */
            int64_t r = sub ? (*dstp_ - prod) : (*dstp_ + prod);
            if (rnd) r = (r + 0x8000) & ~0xFFFFLL;   /* round : +2^15 puis LSB 15-0 = 0 */
            *dstp_ = sext40(r);
            *dstp  = sext40((int64_t)(int16_t)xv << 16);
            return 1;
        }
```

### F6 — 0xAC / 0xAD « MACP » — L10556‑10572 — **CONFIRMÉ**
- **Actuel** : 2 mots, `acc += T*Smem` + fetch coefficient programme, `T = prog[pmad]`.
- **Attendu** : `LD Xmem, dst || MAS[R] Ymem [, dst_]` (R=0 pour 0xAC/0xAD) — `101011RD XXXXYYYY`, **1 mot**. Source : `spru.txt` brut `X0101DR11YXXYYYX` → `101011RDXXXXYYYY`, exécution `Xmem<<16 → dst(31−16)` ‖ `dst_ − T×Ymem → dst_`, `Words 1 word`. Le vrai `MACP Smem,pmad,src` est **0x78/0x79** (map:L86). Couvert par le patch F5.

### F7 — 0xAE / 0xAF « MACD » — L10536‑10555 — **CONFIRMÉ**
- **Actuel** : 2 mots, MAC + data-move `data[addr] = prog[pmad]`, `T = ancien Smem`.
- **Attendu** : `LD Xmem, dst || MASR Ymem [, dst_]` (R=1), **1 mot**. Le vrai `MACD Smem,pmad,src` est **0x7A/0x7B** (map:L87). Couvert par le patch F5.

---

## B. Gravité 2 — résultat faux silencieux

### F8 — 0x90–0x93 décodés en MAC dual — L9766‑9792 — **CONFIRMÉ**
- **Actuel** : `dst ±= T × Xmem ; T = Ymem`, round si bit0, dst si bit1.
- **Attendu** : `ADD Xmem, SHFT, src` (0x90/0x91) et `SUB Xmem, SHFT, src` (0x92/0x93) — `1001000S XXXXSHFT` / `1001001S XXXXSHFT`, masque 0xFE00, **1 mot**, `src = src ± Xmem << SHFT`, S = bit 8. Sources : map:L102‑103 ; `spru.txt` brut `X1001S000TXXFHSX` → `1001000SXXXXSHFT` et `X1001S100TXXFHSX` → `1001001SXXXXSHFT` ; Tab. 2‑1 `ADDXmem,SHFT,srcsrc=src+Xmem<<SHFT113A`, `SUBXmem,SHFT,srcsrc=src−Xmem<<SHFT113A`.
- Le vrai MAC dual est 0xB0‑0xB7 (F13). Ici le champ bits 3‑0 est **SHFT**, pas un Ymem : le handler lit `s->ar[(op&3)+2]` comme opérande mémoire alors que c'est une constante de décalage → lecture mémoire parasite **et** post‑modification d'un AR qui ne doit pas bouger (effet de bord sur AR2‑AR5).
- **Patch** :
```c
        if (hi8 >= 0x90 && hi8 <= 0x93) {   /* ADD/SUB Xmem, SHFT, src — 1 MOT */
            uint16_t xa = resolve_xmem(s, op);
            int shft = op & 0x0F;
            int64_t v = (int64_t)(int16_t)data_read(s, xa) << shft;
            int64_t *acc = (hi8 & 1) ? &s->b : &s->a;     /* S = bit8 */
            *acc = sext40((hi8 & 2) ? (*acc - v) : (*acc + v));
            return 1;
        }
```

### F9 — 0x98/0x99 (STL) et 0x9A/0x9B (STH) : **SHFT ignoré** — L9831‑9853 — **CONFIRMÉ**
- **Actuel** : `data_write(addr, acc & 0xFFFF)` / `data_write(addr, (acc>>16) & 0xFFFF)`. Le commentaire L9828‑9830 avoue explicitement « Shift application is intentionally simplified (no SHFT decode) … Tier B will add proper 4-bit shift decode ». Ici le commentaire **est** conforme au code : c'est le code qui est faux, pas le commentaire.
- **Attendu** : `STL src, SHFT, Xmem` → `Xmem = src << SHFT` ; `STH src, SHFT, Xmem` → `Xmem = (src << SHFT) >> 16`. Sources : map:L106‑107 ; `spru.txt` brut `X1001S001TXXFHSX` → `1001100SXXXXSHFT` et `X1001S101TXXFHSX` → `1001101SXXXXSHFT` ; Tab. 2‑21 `STLsrc,SHFT,XmemXmem=src<<SHFT1110A`, `STHsrc,SHFT,XmemXmem=src<<SHFT−161110A`. SHFT est **non signé 0..15** (`0 ≤ SHFT ≤ 15`), donc pas de branche négative.
- Le reste du handler (resolve_xmem, src = bit 8) est correct.
- **Patch** :
```c
        if (hi8 >= 0x98 && hi8 <= 0x9B) {
            addr = resolve_xmem(s, op);
            int shft = op & 0x0F;                       /* 0..15, non signé */
            int64_t acc = (hi8 & 1) ? s->b : s->a;      /* S = bit8 */
            int64_t v = acc << shft;
            data_write(s, addr, (uint16_t)(((hi8 & 2) ? (v >> 16) : v) & 0xFFFF));
            return consumed + s->lk_used;
        }
```

### F10 — masque `0xFC00 == 0x9C00` avale STRCD/SRCCD — L9859 — **CONFIRMÉ** (patron n°3)
- **Actuel** : 0x9C **et** 0x9D exécutés en SACCD (store de l'accumulateur décalé par ASM).
- **Attendu** : `STRCD Xmem, cond` = **0x9C00/0xFF00**, `si cond → Xmem = T` ; `SRCCD Xmem, cond` = **0x9D00/0xFF00**, `si cond → Xmem = BRC`. Sources : map:L108‑109 ; `spru.txt` bruts `X10010011DXXNOCX` → `10011100XXXXCOND` (exécution `Ifcond T → Xmem Else Xmem → Xmem`, `Status Bits None`) et `X10011011DXXNOCX` → `10011101XXXXCOND` (`Ifcond BRC → Xmem`). SACCD est `1001111S`, donc **0x9E00/0xFE00** seulement.
- Conséquence : tout `STRCD` écrit `(acc<<ASM)>>16` au lieu de `T`, tout `SRCCD` idem au lieu de `BRC`.
- **Patch** : restreindre la garde et ajouter les deux handlers :
```c
        if (hi8 == 0x9C || hi8 == 0x9D) {              /* STRCD / SRCCD — 1 MOT */
            uint16_t xa = resolve_xmem(s, op);
            int64_t acc = (op & 0x10) ? s->b : s->a;   /* bit4 du code cond = A/B */
            if (saccd_cond_true(acc, op & 0x0F))
                data_write(s, xa, (hi8 == 0x9C) ? s->t : s->brc);
            else
                data_write(s, xa, data_read(s, xa));
            return consumed + s->lk_used;
        }
        if ((op & 0xFE00) == 0x9E00) {                 /* SACCD (masque corrigé) */
```
  (⚠ le bit 4 du code condition 5 bits n'existe pas dans l'encodage de STRCD/SRCCD — le champ COND n'a que 4 bits, b3‑b0. Statut **PROBABLE** sur le choix de l'accumulateur testé pour ces deux instructions ; la partie *masque* et *valeur stockée* est CONFIRMÉE.)

### F11 — SACCD : bit `src` lu au mauvais rang — L9860 — **CONFIRMÉ**
- **Actuel** : `int src_s = (op >> 9) & 1;` → pour 0x9E comme pour 0x9F, bit 9 = 1 → **l'accumulateur B est toujours utilisé**. `SACCD A, Xmem, cond` stocke B.
- **Attendu** : encodage `1001111S XXXXCOND` → S = **bit 8**. Source : `spru.txt` brut `X1001S111DXXNOCX` → e[5]='S' → b8 ; map:L110 (`0x9E00/0xFE00`, donc le bit discriminant est bien b8).
- **Patch** : `int src_s = (op >> 8) & 1;`

### F12 — SACCD : table des conditions fausse — L9871‑9884 — **CONFIRMÉ**
- **Actuel** : `cond = op & 0x0F` puis 0→EQ, 1→NEQ, 2→**GT**, 3→LT, 4→**GEQ**, 5→EQ, 6→GT, 7→LEQ, `default: take = 0` silencieux (8/16 nommés, patron n°2).
- **Attendu** : `spru.txt` (page SACCD/STRCD/SRCCD, table « Cond Description Condition Code ») donne des codes **5 bits** dont le bit 4 sélectionne A/B et les 4 bits bas vont dans le champ COND :
  `AEQ 00101 / ANEQ 00100 / AGT 00110 / AGEQ 00010 / ALT 00011 / ALEQ 00111` (et `BEQ 01101 …`).
  Donc **COND[3:0]** : `2 = GEQ (≥0)`, `3 = LT (<0)`, `4 = NEQ (≠0)`, `5 = EQ (=0)`, `6 = GT (>0)`, `7 = LEQ (≤0)` ; **0, 1 et 8..F sont non assignés**.
- Divergences réelles : **cond 2** exécute `>0` au lieu de `≥0`, **cond 4** exécute `≥0` au lieu de `≠0`. Les cas 3, 5, 6, 7 sont justes par coïncidence. Les cas 0/1 fabriquent une condition sur un encodage inexistant.
- **Patch** :
```c
            int take;
            switch (op & 0x0F) {
            case 0x2: take = (acc >= 0); break;   /* GEQ */
            case 0x3: take = (acc <  0); break;   /* LT  */
            case 0x4: take = (acc != 0); break;   /* NEQ */
            case 0x5: take = (acc == 0); break;   /* EQ  */
            case 0x6: take = (acc >  0); break;   /* GT  */
            case 0x7: take = (acc <= 0); break;   /* LEQ */
            default:  C54_LOG("SACCD cond invalide 0x%x op=%04x PC=%04x",
                              op & 0x0F, op, s->pc); take = 0; break;
            }
```

### F13 — 0xA0 décodé en « opérations accumulateur » — L10465‑10516 — **CONFIRMÉ**
- **Actuel** : sous‑décodage `sub = op & 0xFF` inventé (LD B,A / NOT / NEG / ABS / SAT / SFTA / SFTL selon `sub`), 1 mot, aucun accès mémoire.
- **Attendu** : `ADD Xmem, Ymem, dst` — `1010000D XXXXYYYY` = 0xA000/0xFE00, **1 mot**, `dst = Xmem<<16 + Ymem<<16`. Sources : map:L111 ; `spru.txt` brut `X0101D000YXXYYYX` → `1010000DXXXXYYYY` ; Tab. 2‑1 `ADDXmem,Ymem,dstdst=Xmem<<16+Ymem<<16117`.
- Les opérations que ce bloc croit implémenter (NEG/ABS/SAT/SFTA/SFTL/MAX/MIN) sont **toutes** en 0xF484‑0xF48D et 0xF0E0/0xF460 (§F4/§F0) — aucune n'est en 0xA0xx. Le bloc est donc un décodeur entièrement fictif.
- **Patch** : supprimer L10465‑10516 et étendre le handler dual d'ADD/SUB (cf. F4) à `hi8 == 0xA0 || hi8 == 0xA1` avec `+` et `hi8 == 0xA2 || hi8 == 0xA3` avec `−`, `dst = hi8 & 1`.

### F14 — 0xA1 décodé en SQDST — L10356‑10380 — **CONFIRMÉ**
- **Actuel** : `B += (AH − Xmem)² ; A = Ymem<<16 ; T = Xmem`.
- **Attendu** : `ADD Xmem, Ymem, B` (D=1) → `B = Xmem<<16 + Ymem<<16`. Le vrai `SQDST Xmem,Ymem` est **0xE2** (map:L130 ; `spru.txt` bloc SQDST, famille 0xE0‑0xE3) — et 0xE2 est par ailleurs avalé par le blanket CMPS L8481, autre bug hors périmètre. Couvert par le patch F13.

### F15 — 0xA4/0xA5 (MPY) et 0xA6/0xA7 (MACSU) exécutés en MAC — L10297‑10353 — **CONFIRMÉ**
- **Actuel** : les deux paires tombent dans le handler commun L10297 qui fait `acc += T × Xmem ; T = Ymem`.
- **Attendu** :
  - `MPY Xmem, Ymem, dst` = `1010010D XXXXYYYY` (0xA400/0xFE00) → `dst = Xmem × Ymem ; T = Xmem` — **pas d'accumulation**, et T reçoit **Xmem**. Source : map:L113 ; `spru.txt` brut `X0101D010YXXYYYX` → `1010010DXXXXYYYY`, exécution `2: Xmem×Ymem → dst, Xmem → T`.
  - `MACSU Xmem, Ymem, src` = `1010011S XXXXYYYY` (0xA600/0xFE00) → `src = src + unsXmem × signedYmem ; T = Xmem` — **Xmem non signé**. Source : map:L114 ; `spru.txt` brut `X0101S110YXXYYYX` → `1010011SXXXXYYYY`, exécution `unsigned Xmem × signed Ymem + src → src, Xmem → T`, description « The 16-bit unsigned value Xmem is stored in T ».
- Le code actuel : (a) accumule là où MPY écrase, (b) multiplie par **T** au lieu de **Ymem**, (c) met **Ymem** dans T au lieu de Xmem, (d) traite Xmem en signé pour MACSU.
- **Patch** : sortir 0xA4‑0xA7 du test L10297 et ajouter :
```c
        if (hi8 >= 0xA4 && hi8 <= 0xA7) {   /* MPY / MACSU Xmem,Ymem — 1 MOT */
            /* … décodage Xmem/Ymem identique au squelette F5 … */
            int64_t prod;
            if (hi8 <= 0xA5)                       /* MPY  : signé × signé */
                prod = (int64_t)(int16_t)xv * (int64_t)(int16_t)yv;
            else                                   /* MACSU: unsXmem × sYmem */
                prod = (int64_t)(uint16_t)xv * (int64_t)(int16_t)yv;
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t *acc = (hi8 & 1) ? &s->b : &s->a;   /* D/S = bit8 */
            *acc = sext40((hi8 <= 0xA5) ? prod : (*acc + prod));
            s->t = xv;                              /* T = Xmem, PAS Ymem */
            return 1;
        }
```

### F16 — 0xAA/0xAB neutralisés en NOP — L10434‑10440 — **CONFIRMÉ**
- **Actuel** : `return 1;` muet.
- **Attendu** : `LD Xmem, dst || MACR Ymem [, dst_]` (R=1) — cf. F5. Le commentaire L10435‑10438 dit « tic54x dit 0xAA/AB = LD variant … Neutralisé » : le diagnostic est bon, la neutralisation est un trou. Couvert par le patch F5.

### F17 — 0xB0–0xB7 (MAC/MACR Xmem,Ymem) : opérandes, T et dst faux — L10297‑10353 — **CONFIRMÉ**
- **Actuel** : `prod = T × Xmem` ; `acc(dst) += prod` ; `T = Ymem` ; `dst_b = (hi8 >= 0xB6)` pour 0xB4‑B7 et `(hi8 & 0x02)` pour 0xB0‑B3 ; le bit `src` n'est jamais lu.
- **Attendu** : `MAC[R] Xmem, Ymem, src [, dst]` = `10110RSD XXXXYYYY` = 0xB000/0xF800, **R = bit 10, S = bit 9, D = bit 8** ; exécution `dst = src + Xmem × Ymem ; Xmem → T`. Sources : map:L116‑117 ; `spru.txt` brut `X1101DSR0YXXYYYX` → `10110RSDXXXXYYYY` ; exécution `2: Xmem × Ymem + src → dst, Xmem → T`.
- Quatre divergences cumulées : (a) l'opérande multiplié est **Ymem**, pas T ; (b) **T reçoit Xmem**, pas Ymem ; (c) l'accumulateur source `src` (bit 9) est confondu avec la destination ; (d) `dst` est pris sur le **bit 9** (`hi8 & 2` / `>= 0xB6`) au lieu du **bit 8** : `0xB1` et `0xB5` écrivent A au lieu de B, `0xB2` et `0xB6` écrivent B au lieu de A.
- **Patch** :
```c
        if (hi8 >= 0xB0 && hi8 <= 0xBF) {   /* MAC[R]/MAS[R] Xmem,Ymem,src[,dst] — 1 MOT */
            /* … décodage Xmem/Ymem identique au squelette F5 … */
            int rnd = (hi8 >> 2) & 1;                  /* R = bit10 */
            int sa  = (hi8 >> 1) & 1;                  /* S = bit9  */
            int da  =  hi8       & 1;                  /* D = bit8  */
            int sub = (hi8 >= 0xB8);                   /* 0xB8-BF = MAS[R] */
            int64_t prod = (int64_t)(int16_t)xv * (int64_t)(int16_t)yv;  /* Xmem*Ymem */
            if (s->st1 & ST1_FRCT) prod <<= 1;
            int64_t src = sa ? s->b : s->a;
            int64_t r = sub ? (src - prod) : (src + prod);
            if (rnd) r = (r + 0x8000) & ~0xFFFFLL;
            if (da) s->b = sext40(r); else s->a = sext40(r);
            s->t = xv;                                  /* T = Xmem */
            return 1;
        }
```

### F18 — 0xB8–0xBB (MAS) : mêmes erreurs d'opérande/T/dst — L10408‑10432 — **CONFIRMÉ**
- **Actuel** : famille correcte (soustraction), mais `prod = T × Xmem`, `T = Ymem`, `dst = hi8 & 0x02` (→ 0xB8/0xB9 en A, 0xBA/0xBB en B) et `src` ignoré.
- **Attendu** : `MAS[R] Xmem, Ymem, src [, dst]` = `10111RSD XXXXYYYY` — `spru.txt` brut `X1101DSR1YXXYYYX` → `10111RSDXXXXYYYY`, exécution `2: src − Xmem × Ymem → dst, Xmem → T`. D = **bit 8**. Couvert par le patch F17.

### F19 — 0xBA détourné vers « LDMM » — L10411 + L10441‑10455 — **CONFIRMÉ**
- **Actuel** : `if (hi8 == 0xBA) goto ba_handler;` (L10411) fait sauter MAS pour exécuter `LDMM MMR, dst` : `mmr = op & 0x7F ; dst = (op >> 4) & 1 ; acc = sext(data[mmr])`.
- **Attendu** : `MAS Xmem, Ymem, src [, dst]` avec S=1, D=0. **`LDMM` n'existe pas** dans le jeu C54x : la seule instruction de chargement depuis un MMR est `LDM MMR, dst` = **0x4800/0xFE00** (map:L53), déjà décodée dans `case 0x4`. Ce handler écrit donc un accumulateur depuis une adresse MMR arbitraire (`op & 0x7F` = les 7 bits bas d'un champ Xmem/Ymem) sur chaque `MAS *ARx,*ARy,B` du firmware.
- **Patch** : supprimer L10411 (`goto ba_handler`), le label `ba_handler:` (L10433) et le bloc L10441‑10455 ; 0xBA est couvert par le patch F17.

### F20 — 0xBC–0xBF décodés en POLY — L10382‑10406 — **CONFIRMÉ**
- **Actuel** : `B += rnd(AH × T) ; A = Xmem<<16 ; T = Ymem`, round inconditionnel.
- **Attendu** : `MASR Xmem, Ymem, src [, dst]` (R=1 de la famille `10111RSD`) → `dst = rnd(src − Xmem × Ymem) ; T = Xmem`. Source : map:L119 ; `spru.txt` `10111RSDXXXXYYYY`. Le vrai `POLY Smem` est **0x36** (map:L44), instruction à opérande **Smem** unique — jamais en 0xBCxx. Le commentaire L10383‑10385 (« 1011 110D … 0xBE/0xBF variants — ABDST or POLY ») est une pure conjecture, contredite par l'encodage. Couvert par le patch F17.

### F21 — arrondi `[R]` mal implémenté (transverse) — L9786, L10330, L10401, L10425 — **CONFIRMÉ**
- **Actuel** : `prod += 0x8000` **avant** l'accumulation, et les 16 bits bas ne sont jamais effacés.
- **Attendu** : `spru.txt` (MAC[R], MAS[R], MPY[R], LD‖MAC[R], LD‖MAS[R], ST‖MAC[R]) : « rounds the result of the multiply and accumulate operation **by adding 2^15 to the result and clearing the LSBs (15–0) to 0** ». L'arrondi porte sur le **résultat après accumulation**, et efface les bits 15‑0.
- Effet : les 16 bits bas de l'accumulateur restent pollués après tout MACR/MASR/MPYR, et l'arrondi est appliqué au produit et non à la somme (différence dès que `src` a des bits bas non nuls).
- **Patch** : partout, remplacer `if (R) prod += 0x8000;` par, après le calcul de `r` : `if (R) r = (r + 0x8000) & ~0xFFFFLL;` (déjà intégré dans les patches F5 et F17).

---

## C. Code mort / non‑bugs

### F22 — handler `hi8 == 0x91` (MVKD 2 mots) L10262‑10269 — **NON‑BUG (inatteignable)**
Ombré par le test L9770 (`hi8 == 0x90..0x93`). À supprimer avec F8 pour éviter qu'il ne redevienne atteignable.

### F23 — handler `hi8 == 0x9F` (PORTW) L10125‑10140 — **NON‑BUG (inatteignable)**
Ombré par L9859 (`(op & 0xFC00) == 0x9C00`). Le vrai `PORTW Smem, PA` est 0x75 (map:L83), correctement placé dans `case 0x6/0x7`. ⚠ Si l'on corrige le masque de F10 en `0xFE00 == 0x9E00`, ce handler **redevient atteignable** et volerait la moitié de SACCD (0x9F) — il faut le supprimer **dans le même patch**.

### F24 — handler `hi8 == 0xA5` (CMPS) L10518‑10535 — **NON‑BUG (inatteignable)**
Ombré par L10297. Sémantique de toute façon fausse (CMPS = 0x8E/0x8F, correctement traité L10022). À supprimer avec F15.

### F25 — décodage Xmem/Ymem 2 bits + post‑modification — L9774‑9783, L10309‑10325, L4832‑4845 — **NON‑BUG**
`xar = ((op>>4)&3)+2`, `xmod = (op>>6)&3`, `yar = (op&3)+2`, `ymod = (op>>2)&3`, modes `0=*ARx 1=*ARx− 2=*ARx+ 3=*ARx+0%` avec circularité BK : conforme à `XXXXYYYY` du manuel (X = b7‑b4, Y = b3‑b0) et à SPRU131G Tab. 5‑6/5‑8. `resolve_xmem` (L4832) applique la même convention. Les patches ci‑dessus réutilisent ce décodage tel quel.

### F26 — FRCT — L9785, L10328, L10424 — **NON‑BUG**
`if (s->st1 & ST1_FRCT) prod <<= 1;` conforme (« Affected by FRCT and OVM »).

---

## D. Récapitulatif par opcode

| hi8 | attendu (manuel) | décodé actuellement | L. | mots att./act. | statut |
|---|---|---|---|---|---|
| 90‑91 | ADD Xmem,SHFT,src | MAC dual | 9770 | 1/1 | CONFIRMÉ g2 |
| 92‑93 | SUB Xmem,SHFT,src | MAC dual | 9770 | 1/1 | CONFIRMÉ g2 |
| 94‑95 | LD Xmem,SHFT,dst | MVDK / MVKD | 9795/9803 | **1/2** | CONFIRMÉ g1 |
| 96 | BIT Xmem,BITC | MVDP | 9811 | **1/2** | CONFIRMÉ g1 |
| 97 | *non assigné* | ST #lk,Smem | 10271 | —/2 | CONFIRMÉ g1 |
| 98‑9B | STL/STH src,SHFT,Xmem | idem sans SHFT | 9831/9844 | 1/1 | CONFIRMÉ g2 |
| 9C | STRCD Xmem,cond | SACCD | 9859 | 1/1 | CONFIRMÉ g2 |
| 9D | SRCCD Xmem,cond | SACCD | 9859 | 1/1 | CONFIRMÉ g2 |
| 9E‑9F | SACCD src,Xmem,cond | SACCD, src=bit9, cond fausse | 9860/9874 | 1/1 | CONFIRMÉ g2 |
| A0‑A1 | ADD Xmem,Ymem,dst | ops accu fictives / SQDST | 10465/10359 | 1/1 | CONFIRMÉ g2 |
| A2‑A3 | SUB Xmem,Ymem,dst | ADD/SUB #lk | 10579/10589 | **1/2** | CONFIRMÉ g1 |
| A4‑A5 | MPY Xmem,Ymem,dst | MAC | 10297 | 1/1 | CONFIRMÉ g2 |
| A6‑A7 | MACSU Xmem,Ymem,src | MAC signé | 10297 | 1/1 | CONFIRMÉ g2 |
| A8‑A9 | LD‖MAC Ymem | AND #lk | 10456 | **1/2** | CONFIRMÉ g1 |
| AA‑AB | LD‖MACR Ymem | NOP muet | 10434 | 1/1 | CONFIRMÉ g2 |
| AC‑AD | LD‖MAS Ymem | MACP | 10557 | **1/2** | CONFIRMÉ g1 |
| AE‑AF | LD‖MASR Ymem | MACD | 10539 | **1/2** | CONFIRMÉ g1 |
| B0‑B7 | MAC[R] Xmem,Ymem,src[,dst] | MAC T×Xmem, dst=bit9 | 10297 | 1/1 | CONFIRMÉ g2 |
| B8‑BB | MAS Xmem,Ymem,src[,dst] | MAS T×Xmem, dst=bit9 ; **BA→LDMM** | 10409/10441 | 1/1 | CONFIRMÉ g2 |
| BC‑BF | MASR Xmem,Ymem,src[,dst] | POLY | 10386 | 1/1 | CONFIRMÉ g2 |

**Aucune famille de la plage n'est correcte.** Ordre de correction recommandé : F1–F7 (longueurs, désync) → F17/F18/F19/F20 (bloc MAC/MAS dual, cœur du corrélateur) → F13/F14/F15 (0xA0‑0xA7) → F8/F9 → F10/F11/F12 → F21. Chaque groupe doit être mesuré isolément : plusieurs de ces handlers erronés ont été « calibrés » par des fixes empiriques successifs (commentaires 2026‑05‑29, 2026‑06‑22) sur des traces produites par d'autres bugs, donc des compensations mutuelles sont probables.

---

# REFUTATION 0x90-0xBF

# RÉFUTATION — audit hi8 0x90–0xBF (`calypso_c54x.c`, md5 `9d8108f4f626cfbc906ce11c258ce7e2`, identique conteneur/local)

## Base de vérification indépendante

J'ai re-décodé **les 194 blocs `Opcode` de `spru.txt`** avec la permutation fournie (script ad hoc, 0 saisie manuelle) → carte exhaustive du manuel. Extrait pertinent (chaîne = bit15…bit0) :

```
1001000SXXXXSHFT  1001001SXXXXSHFT  1001010DXXXXSHFT  10010110XXXXBITC
1001100SXXXXSHFT  1001101SXXXXSHFT  10011100XXXXCOND  10011101XXXXCOND  1001111SXXXXCOND
1010000DXXXXYYYY  1010001DXXXXYYYY  1010010DXXXXYYYY  1010011SXXXXYYYY
101010RDXXXXYYYY  101011RDXXXXYYYY  10110RSDXXXXYYYY  10111RSDXXXXYYYY
```
**Aucune chaîne `10010111…`** → 0x97 réellement non assigné. `spru.txt@110557` (« Class 7 : **1 word**, 1 cycle. Dual data-memory Xmem and Ymem read operands » → ABDST, LD‖MAS[R], MACSU, SQDST, ADD, LMS, MAS[R], SUB, LD‖MAC[R], MAC[R], MPY) confirme la longueur 1 mot pour toute la plage 0xA0–0xBF. `spru.txt@575102` (STRCD) et `@547342` (SACCD) : « Words 1 word ».

Ordre des `if` re-vérifié ligne à ligne (`case 0x8/0x9` L9763→10278 : 0x90-93, 0x94, 0x95, 0x96, 0x98/99, 0x9A/9B, `0xFC00==0x9C00`, … , 0x91 L10263, 0x97 L10271 ; `case 0xA/0xB` L10280→10598 : A4-A7+B0-B7, A1, BC-BF, B8-BB, `ba_handler`, AA/AB, BA, A8/A9, A0, A5, AE/AF, AC/AD, A2, A3). **Aucun `return` entre L5255 et L5960** (grep) → rien n'intercepte en amont du `switch (hi4)`. Les numéros de ligne du rapport sont exacts (contrôlés un par un).

---

## Reclassement finding par finding

| # | Statut | Raison / source qui tranche |
|---|---|---|
| **F1** 0x94/0x95 MVDK/MVKD 2 mots | **CONFIRMÉ** | `spru.txt@476524` brut `X1001D010TXXFHSX` → `1001010DXXXXSHFT` = LD Xmem,SHFT,dst, 1 mot ; MVKD/MVDK sont bien `01110000`/`01110001` (=0x70/0x71) dans le scan exhaustif. Handlers L9795/L9803 atteignables (rien avant). Non réfutable. |
| **F2** 0x96 MVDP 2 mots | **CONFIRMÉ** | `spru.txt@432148` : `10010110XXXXBITC`, « Execution Xmem15−BITC → TC », « **Words 1 word** », « Affects TC ». MVDP = `01111101` (0x7D). Handler L9811 atteignable. |
| **F3** 0x97 « ST #lk » | **CONFIRMÉ (encodage) — gravité pratique NULLE (voir F31)** | 0x97 absent des 194 blocs et de map. Mais atteignable seulement si un 0x97xx est exécuté : sur `/tmp/prom0.txt` les **2 seuls** mots 0x97xx (`972f`, `97b8`) sont les opérandes `pmad` d'un `f074` (CALL) — jamais des opcodes. Le patch reste juste, la priorité est fausse. |
| **F4** 0xA2/0xA3 ADD/SUB #lk 2 mots | **CONFIRMÉ** | `spru.txt@576450` `X0101D100YXXYYYX` → `1010001DXXXXYYYY` = SUB Xmem,Ymem,dst, Class 7 = 1 mot. Handlers L10579/L10589 atteignables (rien avant ne teste 0xA2/0xA3). |
| **F5** 0xA8/0xA9 AND #lk 2 mots | **CONFIRMÉ** | `spru.txt@482532` `X0101DR01YXXYYYX` → `101010RDXXXXYYYY`, « Xmem<<16 → dst31−16 ‖ Ymem×T + dst_ → dst_ », « Words 1 word Cycles 1 cycle Class 7 ». **Réserve sur le patch** : il fait `sext40((int16_t)xv<<16)` inconditionnellement alors que le chargement `<<16` est gouverné par SXM pour les bits de garde ; et il ne doit **pas** toucher T (le patch ne le fait pas — correct). |
| **F6** 0xAC/0xAD MACP | **CONFIRMÉ** | `spru.txt@484175` → `101011RDXXXXYYYY` (LD‖MAS[R]) ; MACP = `0111100S` (0x78/0x79). |
| **F7** 0xAE/0xAF MACD | **CONFIRMÉ** | Idem `101011RD` avec R=1 → LD‖MASR ; MACD = `0111101S` (0x7A/0x7B). |
| **F8** 0x90–0x93 en MAC dual | **CONFIRMÉ** | `spru.txt@416105` `X1001S000TXXFHSX` → `1001000SXXXXSHFT` (ADD) et `@576410` → `1001001SXXXXSHFT` (SUB), plage documentée « 0 ≤ SHFT ≤ 15 ». Le handler L9770 lit bien Xmem sur les bits 7-4 (donc cette moitié est fortuitement correcte) mais traite les bits 3-0 comme un Ymem → lecture mémoire parasite + post-modif d'AR2-AR5 : effet de bord réel, pas seulement un résultat faux. |
| **F9** 0x98–0x9B : SHFT ignoré | **CONFIRMÉ** | Ce n'est **pas** un finding fondé sur commentaire : le code L9838 `data_write(addr, acc & 0xFFFF)` et L9851 `(acc>>16)&0xFFFF` n'appliquent aucun décalage, le commentaire L9828 ne fait que le décrire. `spru.txt@563166` (STL, « 3: src << SHFT → Xmem ») et `@560632` (STH, « 3: src << SHFT − 16 → Xmem »). Le bit src = b8 (`hi8 & 1`) est correct — le rapport le dit, exact. |
| **F10** masque `0xFC00==0x9C00` avale STRCD/SRCCD | **CONFIRMÉ sur le masque et la valeur stockée / PATCH RÉFUTÉ** | `spru.txt@575102` `X10010011DXXNOCX` → `10011100XXXXCOND`, « If cond T → Xmem Else Xmem → Xmem, Status Bits **None** » ; `@557468` → `10011101XXXXCOND`, « If cond BRC → Xmem ». Donc masque correct = 0xFF00 sur 0x9C/0x9D. **En revanche le patch proposé est faux** : `(op & 0x10)` pointe le bit 4, qui appartient au champ **Xmem** (b7-b4) — il lirait le numéro d'AR. Le sélecteur A/B est COND bit **3** (cf. F28). `s->brc` existe bien (`calypso_c54x.h:143`), `saccd_cond_true()` non (à écrire). |
| **F11** SACCD src lu en bit 9 | **CONFIRMÉ** | `spru.txt@547342` brut `X1001S111DXXNOCX` : la permutation place `S` en e[5] → **b8** ; b3..b0 = C,O,N,D. Le code L9860 `(op>>9)&1` vaut 1 pour 0x9E **et** 0x9F (garde 0xFC00) → accumulateur B systématique. Non réfutable. |
| **F12** table de conditions SACCD fausse | **BUG CONFIRMÉ / TABLE CORRECTIVE DU RAPPORT RÉFUTÉE** | Le bug existe (cond 2 et cond 4 divergent). Mais la table proposée est **fausse** : le rapport dit « le bit 4 sélectionne A/B, 8..F non assignés ». Or `spru.txt@574900` donne AEQ=**00101**/BEQ=**01101**, ANEQ=00100/BNEQ=01100, AGT=00110/BGT=01110, AGEQ=00010/BGEQ=01010, ALT=00011/BLT=01011, ALEQ=00111/BLEQ=01111 : le bit discriminant A/B est le **bit 3** (poids 8), pas le bit 4 (toujours 0). Le champ COND 4 bits code donc **12** conditions : A = 0x2,3,4,5,6,7 ; B = 0xA,B,C,D,E,F ; seuls 0x0,0x1,0x8,0x9 sont invalides. Le patch du rapport enverrait 0xA–0xF en `default take=0` → il *fabrique* un nouveau trou. Corroboration interne : `calypso_c54x.c:4867` (commentaire de `c54x_cond_true`) dit déjà « **CCB=0x08 (accu B sinon A)**, test bits[2:0] EQ=5 NEQ=4 LT=3 LEQ=7 GT=6 GEQ=2 » — le fichier connaît déjà la bonne convention ailleurs. |
| **F13** 0xA0 = sous-décodage fictif | **CONFIRMÉ** | `spru.txt@416145` `X0101D000YXXYYYX` → `1010000DXXXXYYYY` = ADD Xmem,Ymem,dst. Le bloc L10465 est atteignable (aucun test 0xA0 avant). NEG/ABS/SAT/MAX/MIN/SFTA/SFTL sont tous en `111101SD1000…` / `111100SD111SHIFT` / `111101SD011SHIFT` dans le scan — jamais en 0xA0xx. |
| **F14** 0xA1 = SQDST | **CONFIRMÉ, et pire que dit** | 0xA1 = ADD Xmem,Ymem,B. Le vrai SQDST est `11100010` (0xE2), `spru.txt@…SQDST` : « **A32−16 × A32−16 + B → B ; Xmem − Ymem << 16 → A** ». Le handler L10359 calcule `(AH − Xmem)² + B` et `A = Ymem<<16` : il est faux **même comme SQDST**. Le déplacer tel quel en 0xE2 ne corrigerait rien (cf. F30). |
| **F15** 0xA4–0xA7 exécutés en MAC | **CONFIRMÉ, incomplet** | `spru.txt@505871` → `1010010DXXXXYYYY` (« Xmem×Ymem → dst, Xmem → T ») et `@497679` → `1010011SXXXXYYYY` (« unsignedXmem × signedYmem + src → src, Xmem → T »). Les 4 divergences listées sont réelles. **Manque une 5ᵉ** : L10342 `dst_b = (hi8 >= 0xA6)` prend le bit 9 alors que D/S = **bit 8** → 0xA5 écrit A au lieu de B, 0xA6 écrit B au lieu de A (le patch du rapport, lui, utilise bien `hi8 & 1`). |
| **F16** 0xAA/0xAB NOP | **CONFIRMÉ** | `101010RD` couvre 0xA8–0xAB : R = b9, donc 0xAA/0xAB = LD Xmem,dst ‖ **MACR** Ymem. `return 1;` L10439 est muet. Le commentaire L10435 (« tic54x dit LD variant … Neutralisé ») ne contredit pas le code : le code *est* le NOP. |
| **F17** 0xB0–0xB7 opérandes/T/dst faux | **CONFIRMÉ** | `spru.txt@489813` `X1101DSR0YXXYYYX` → `10110RSDXXXXYYYY`, « 2: Xmem × Ymem + src → dst, Xmem → T ». R=b10, S=b9, D=b8 vérifiés par la permutation. Divergences (a)(b)(c)(d) exactes ; **il en manque une 5ᵉ**, le bit R (cf. F27). |
| **F18** 0xB8–0xBB | **CONFIRMÉ** | `spru.txt@500295` `X1101DSR1YXXYYYX` → `10111RSDXXXXYYYY`, « 2: src − Xmem × Ymem → dst, Xmem → T ». Le signe (soustraction) du handler L10409 est juste ; opérandes, T, dst et R sont faux. |
| **F19** 0xBA → « LDMM » | **CONFIRMÉ** | `goto ba_handler` L10411 saute au label L10433, retombe sur AA/AB (faux) puis sur `if (hi8 == 0xBA)` L10441 = LDMM. Aucune chaîne « LDMM » dans le scan des 194 opcodes ; le seul chargement depuis MMR est `0100100D` (=0x48/0x49, LDM), déjà décodé dans `case 0x4`. `op & 0x7F` = bits Xmem/Ymem → adresse MMR arbitraire, `(op>>4)&1` = bit d'AR. |
| **F20** 0xBC–0xBF → POLY | **CONFIRMÉ** | 0xBC–0xBF ∈ `10111RSD` avec R=1 → MASR. POLY = `00110110` (0x36), opérande Smem unique. Le commentaire L10383 (« ABDST or POLY ») est bien une conjecture non sourcée — mais le finding ne repose pas dessus, il repose sur l'encodage. |
| **F21** arrondi `[R]` | **CONFIRMÉ** | `spru.txt@483161` (page LD‖MAC[R]) : « rounds the result of the multiply and accumulate operation by **adding 2¹⁵ to the result and clearing the LSBs 15−0 to 0** ». Le code fait `prod += 0x8000` *avant* accumulation (L9786, L10330, L10401, L10425) et n'efface jamais les 16 bits bas. |
| **F22** 0x91 L10263 mort | **NON-BUG confirmé** | Ombré par L9770 (`hi8 == 0x91`). L'avertissement du rapport (le supprimer en même temps que F8) est fondé. |
| **F23** 0x9F L10125 mort | **NON-BUG confirmé** | Ombré par L9859 (`0x9F00 & 0xFC00 == 0x9C00`). L'avertissement « redevient atteignable si on resserre le masque en 0x9E00 » est **correct et critique** : sans suppression conjointe, la correction F10/F11 volerait 0x9F à SACCD. |
| **F24** 0xA5 CMPS L10518 mort | **NON-BUG confirmé** | Ombré par L10297. CMPS = `1000111S` (0x8E/0x8F), traité L10022. |
| **F25** décodage Xmem/Ymem 2 bits | **NON-BUG confirmé, mais source mal attribuée** | `resolve_xmem` L4832 : `xar = (xmem & 3) + 2`, `xmod = (xmem & 0xC) >> 2` — conforme à binutils `tic54x-dis.c` (nibble = `mod<<2 | (ar−2)`) et SPRU131G T.5-6/5-8. **SPRU172C ne documente pas ce sous-champ** (les pages Opcode écrivent seulement `XXXX`/`YYYY`) : l'affirmation « conforme à `XXXXYYYY` du manuel » est donc non démontrable par `spru.txt` — la source réelle est SPRU131G/binutils. Conclusion inchangée. |
| **F26** FRCT | **NON-BUG confirmé** | « Affected by FRCT and OVM » présent sur toutes les pages MAC/MAS/MPY/SQDST/ABDST du scan. |

---

## Findings MANQUÉS par l'audit précédent

| # | Statut | Contenu |
|---|---|---|
| **F27** — bit **R** au mauvais rang dans toute la famille MAC/MAS | **CONFIRMÉ** | L9786 (`0x90-93`), L10331 (`A4-A7`+`B0-B7`), L10147/L10425 (`B8-BB`) : `if (hi8 & 0x01) prod += 0x8000;` — `hi8 & 1` = **bit 8 = D**, pas R. Manuel : R = **b10** pour `10110RSD`/`10111RSD` (0xB0-0xBF) et **b9** pour `101010RD`/`101011RD` (0xA8-0xAF). Conséquence : 0xB1/0xB3/0xB9/0xBB (non-round) sont arrondis, 0xB4/0xB6 (MACR, round obligatoire) ne le sont pas. Indépendant de F21 (celui-ci porte sur *où* et *comment* arrondir, F27 sur *quand*). |
| **F28** — accumulateur **testé** des stores conditionnels | **CONFIRMÉ** | Pour SACCD/STRCD/SRCCD le manuel donne un code condition 5 bits dont le bit 3 (poids 8) sélectionne A ou B (`AEQ=00101` vs `BEQ=01101`), et les 4 bits bas rentrent exactement dans le champ COND (b3-b0). L'accumulateur testé est donc **COND[3]**, distinct du `src` **stocké** (b8) pour SACCD (`SACCD src, Xmem, cond` : on peut stocker A en testant B). Le code L9860-9884 teste l'accumulateur choisi par `(op>>9)&1` (= toujours B sur 0x9C-0x9F) et ignore COND[3]. Ce point est **absent** de F11 et **contredit** F12. |
| **F29** — le patch F12 est lui-même un bug | **RÉFUTATION du patch** | Il renvoie 0x8–0xF en `default: take = 0` alors que 0xA–0xF sont les six conditions sur B. Table correcte : `0x2 AGEQ, 0x3 ALT, 0x4 ANEQ, 0x5 AEQ, 0x6 AGT, 0x7 ALEQ, 0xA BGEQ, 0xB BLT, 0xC BNEQ, 0xD BEQ, 0xE BGT, 0xF BLEQ` ; invalides = 0x0, 0x1, 0x8, 0x9. Le décodeur possède déjà cette logique (`c54x_cond_true`, L4864-4870) — la réutiliser plutôt que réécrire un `switch`. |
| **F30** — SQDST faux *en soi* | **CONFIRMÉ** | Voir F14 : le handler L10359 n'implémente pas SQDST du manuel (`A[32:16]²`, `A = (Xmem−Ymem)<<16`) mais `(AH−Xmem)²` / `A = Ymem<<16`. À réécrire, pas à relocaliser. |
| **F31** — priorisation | **RÉFUTATION de l'ordre proposé** | Le rapport place F1–F7 (longueurs) en tête. Sur `/tmp/prom0.txt` (28 672 mots, base 0x7000), l'histogramme brut hi8 donne : **0xAA=143, 0xA9=115, 0xB0=103, 0xA0=60, 0xA1=54, 0xA2=52** contre **0x97=2 (les deux = opérandes de `f074`)**, **0x95=6, 0x94=12**. C'est un majorant (opérandes inclus), mais il inverse la priorité : le cœur chaud est le bloc MAC/MAS dual + 0xA0/0xA1 (F13/F14/F15/F17/F18/F27) et le NOP 0xAA/0xAB (F16), pas F3. |
| **F32** — OVM / OVA / OVB non modélisés | **PROBABLE (systémique, hors plage)** | Toutes les pages ADD/SUB/MAC/MAS/MPY/SQDST/ABDST Xmem,Ymem portent « Affected by OVM », « Affects OVdst ». Le fichier ne contient que 3 sites `ST0_OVA` (L5993, L8620, L10490) et zéro `ST1_OVM` : aucun handler de la plage ne sature ni ne pose OV. Divergence réelle mais globale au décodeur, non spécifique à 0x90-0xBF, et non observable tant que le firmware ne met pas OVM=1 — à ne pas mélanger aux correctifs ci-dessus. |

---

## Synthèse du reclassement

- **CONFIRMÉ, non réfutable** : F1, F2, F4, F5, F6, F7, F8, F9, F11, F13, F14, F15, F16, F17, F18, F19, F20, F21 (18/21 des findings de bug).
- **CONFIRMÉ mais patch/justification à corriger** : F3 (gravité pratique nulle sur le ROM actuel), F10 (masque OK, patch faux — bit 4 vs bit 3), F12 (bug réel, table corrective fausse).
- **NON-BUG confirmés** : F22, F23, F24, F25 (source à requalifier : SPRU131G/binutils, pas SPRU172C), F26.
- **Aucun finding REFUTÉ sur le fond.** L'audit est solide ; ses deux erreurs matérielles sont concentrées sur le champ COND des stores conditionnels (F10/F12 → corrigées par F28/F29), et il a manqué le mauvais rang du bit R (F27), la fausseté intrinsèque de SQDST (F30) et le bit dst de 0xA4-0xA7 (dans F15).
- **Avertissement de séquencement à conserver** : toute correction du masque 0x9C00→0x9E00 (F10/F11) doit supprimer **dans le même patch** le handler PORTW `hi8 == 0x9F` L10125, sinon 0x9F bascule de SACCD vers un PORTW **2 mots** → désynchronisation de décode introduite par le correctif lui-même.

Fichiers : `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (conteneur `osmo-operator-1`) ; copie de travail `/root/.claude/jobs/26578783/tmp/calypso_c54x.c` ; manuel `/root/.claude/jobs/26578783/tmp/spru.txt` ; dump ROM `/tmp/prom0.txt` (conteneur). Aucun fichier modifié.

---

# AUDIT 0xC0-0xFF  (REFUTATION MANQUANTE — workflow tue)

# AUDIT hi8 0xC0–0xFF — `calypso_c54x.c` (md5 `9d8108f4f626cfbc906ce11c258ce7e2`, 16865 l.)

Fichiers : `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (copie locale identique `/root/.claude/jobs/26578783/tmp/calypso_c54x.c`), `/root/.claude/jobs/26578783/tmp/spru.txt`, table projet fournie.

**Outil de lecture spru.txt** : `/tmp/dec.py` + `/tmp/enc.py` (permutation e[k]→bit du contexte, revalidée sur SFTA `01111DS10T11FIHS` où les lettres `SHIFT` retombent exactement sur b4..b0, et sur 12 encodages connus). Toutes les citations `spru.txt@N` = offset octet.

---

## A. FAMILLE 0xC0–0xDF — **la totalité de la famille est mal décodée**

**Vérité établie (manuel, décodée caractère par caractère)** — les 8 sous-classes sont TOUTES `ST src,Ymem || <op> Xmem,dst`, **1 mot**, `XXXXYYYY` en bits 7:0 :

| encodage brut spru | décodé | @ | classe |
|---|---|---|---|
| `X0011DS00YXXYYYX` | `110000SD XXXXYYYY` | 566761 | C0-C3 `ST‖ADD` |
| `X0011DS10YXXYYYX` | `110001SD XXXXYYYY` | 573610 | C4-C7 `ST‖SUB` |
| `X0011DS01YXXYYYX` | `110010SD XXXXYYYY` | 567835 | C8-CB `ST‖LD` |
| `X0011DS11YXXYYYX` | `110011SD XXXXYYYY` | 572647 | CC-CF `ST‖MPY` |
| `X1011DSR0YXXYYYX` | `11010RSD XXXXYYYY` | 569358 | D0-D7 `ST‖MAC[R]` |
| `X1011DSR1YXXYYYX` | `11011RSD XXXXYYYY` | 570992 | D8-DF `ST‖MAS[R]` |

Concorde avec la table projet L120-L127. `Words 1 word` répété sur les 6 pages. Store = `(src << ASM) >> 16` (ex. `ST A,*AR4-‖MAC *AR5,B`, ASM=5, A=`00 0011 1111`, mem[100h] 1234→**0222** = 0x111111<<5>>16, `spru.txt@570457`). **S = b9, D = b8, R = b10, indépendants** (le manuel précise « If src is equal to dst, the value stored in Ymem is the value of src before the execution » → src et dst peuvent coïncider, donc 2 bits séparés).

---

**C-1 / 0xC2, 0xC3, 0xC6, 0xC7 — L10732-10739 — CONFIRMÉ — gravité 1**
- Actuel : `RPTB[D] pmad`, **`consumed = 2`** (`op2 = prog_fetch(pc+1)`), pose `rea/rsa/rptb_active/BRAF`.
- Attendu : `ST src,Ymem ‖ ADD Xmem,dst` (C2/C3, S=1) et `ST src,Ymem ‖ SUB Xmem,dst` (C6/C7, S=1), **1 mot**. `RPTB[D]` est en **0xF072/0xFDFF** (`spru.txt` CALL/RPTB page : `011110Z000111001` → `111100Z0 01110010`), déjà correctement implémenté L7115 et L7259.
- Conséquence : +1 mot de PC avalé par occurrence → désynchronisation totale du flux aval, et boucle `rptb` fantôme armée.
- Patch : supprimer le bloc L10732-10739 (les opcodes retombent alors sur le handler ST‖xxx unifié proposé en C-9).

**C-2 / 0xC4 — L10762-10769 — CONFIRMÉ — gravité 1**
- Actuel : `PSHD dmad` 2 mots (`sp--`, `data_write(sp, data_read(op2))`).
- Attendu : `ST src,Ymem ‖ SUB Xmem,dst` 1 mot. `PSHD Smem` = **0x4B00/0xFF00** (`spru.txt@531308` voisin, table projet L55), déjà décodé ailleurs.
- Conséquence : désync PC **+** push fantôme (SP dérive) — exactement le patron des « SP-CATASTROPHE » déjà chassées dans ce fichier.

**C-3 / 0xDA — L10794-10803 — CONFIRMÉ — gravité 1**
- Actuel : `RPTBD pmad` 2 mots.
- Attendu : `ST src,Ymem ‖ MAS Xmem,dst` (D8-DB, R=0,S=1,D=0), 1 mot. `RPTBD` = 0xF272, déjà géré L7259.
- Conséquence : désync PC.

**C-4 / 0xC0, 0xC1 — L10770-10785 — CONFIRMÉ — gravité 1/2**
- Actuel : `hi8==0xC0` → `PSHD Smem` (`sp--` + write) ; `hi8==0xC1` → `RPT Smem` (`rpt_count = data[Smem]`, `pc += consumed`, `return 0`).
- Attendu : `ST src,Ymem ‖ ADD Xmem,dst`, S=0 (C0 D=0, C1 D=1). `PSHD Smem`=0x4B, `RPT Smem`=**0x4700/0xFF00** (`spru.txt@542279` : `I00101110AAAAAAA` → `01000111 IAAAAAAA`).
- Conséquence : 0xC1 arme un **RPT** sur l'instruction suivante avec un compteur arbitraire → répétition massive et silencieuse ; 0xC0 corrompt SP.

**C-5 / 0xD0-0xD9 — L10614-10673 — CONFIRMÉ — gravité 2 (4 défauts cumulés)**
- Actuel : MAC/MAS dual-mem *sans store*, avec `prod = T × Xmem` ; `if (hi8 & 0x01) prod += 0x8000` (round) ; `is_sub = (hi8 >= 0xD4)` ; `dst = (hi8 & 0x02)` ; `s->t = yval` en sortie.
- Attendu (`11010RSD` / `11011RSD`, `spru.txt@570457`, `@570992`) :
  1. **le store parallèle `Ymem = (src<<ASM)>>16` est totalement absent** (le code ne fait qu'un `data_read` de Ymem) ;
  2. round = **bit 2 de hi8** (0xD4-D7 = MACR), pas bit 0 ;
  3. soustraction à partir de **0xD8** (D8-DB = MAS), pas 0xD4 ;
  4. `dst` = **bit 0** de hi8, `src` = bit 1 — le code prend bit 1 pour dst et ignore src ;
  5. `T` n'est **pas** modifié par `ST‖MAC/MAS` (« Execution : src<<ASM-16 → Ymem ; Xmem × T + dst → dst », aucune écriture de T) — le code fait `s->t = yval_c`, ce qui pollue T à chaque itération de corrélateur.
- Le commentaire L10654-10661 justifie `T×X` par une « sémantique pipeline » : c'est en fait **la bonne formule** (`dst = dst + T*Xmem`) mais pour la mauvaise raison, et il masque les 4 erreurs ci-dessus.

**C-6 / 0xDB, 0xDC, 0xDF — L10668, L10697, L10818 — CONFIRMÉ — gravité 2**
- Actuel : 0xDB=`MASA` (mais code `a += T*X`, donc en réalité MACA), 0xDC=`SQUR Xmem`, 0xDF=`DELAY Smem` (`data_write(addr+1, data[addr])`).
- Attendu : 0xDB = `ST‖MAS` (R=0,S=1,D=1) ; 0xDC/0xDF = `ST‖MASR`. `MASA Smem`=0x33, `SQUR Smem`=0x26, `DELAY Smem`=**0x4D00/0xFF00** (table L57).
- Conséquence : 0xDF **écrit en mémoire à `addr+1`** — corruption silencieuse hors de toute cible légitime.

**C-7 / 0xCC — L10786-10793 — CONFIRMÉ — gravité 2**
- Actuel : « SACCD Smem, ARmem — simplified: always store », `data_write(resolve_smem(op), A>>16)`.
- Attendu : `ST src,Ymem ‖ MPY Xmem,dst` (`Ymem = (src<<ASM)>>16 ; dst = T*Xmem`). Le vrai `SACCD src,Xmem,cond` = **0x9E00/0xFE00** (`spru.txt@547155` : `X1001S111DXXNOCX` → `1001111S XXXXCOND`).
- Double faute : mauvaise instruction **et** mauvais mode d'adressage (`resolve_smem` = Smem 7 bits DP/indirect au lieu de Xmem = AR2..AR5).

**C-8 / 0xC5, 0xCD, 0xCE, 0xCF, 0xDD, 0xDE — L10741, 10748, 10755, 10725, 10804, 10812 — CONFIRMÉ — gravité 2**
- Actuel : `return 1` (NOP muet). Les commentaires disent correctement « tic54x dit 0xC5 = ST‖family (parallel) » — c'est le **patron n°1** : neutralisation plausible au lieu d'implémentation.
- Attendu : C5 = `ST‖SUB`, CD/CE/CF = `ST‖MPY`, DD/DE = `ST‖MASR`.
- Ces stubs étaient un pis-aller correct tant que le handler ST‖xxx n'existait pas ; ils deviennent obsolètes avec C-9.

**C-9 / 0xC8-0xCB — L10854-10923 — CONFIRMÉ (2 défauts) — gravité 2**

Seule famille de la plage dont la classe est juste. Deux erreurs :
- **(a) sélection d'accumulateur** L10855-10861 : `s_acc = (hi8 & 0x01)`, `d_acc = s_acc ? 0 : 1`. L'encodage est `110010SD` : **src = b9 (= bit 1 de hi8), dst = b8 (= bit 0 de hi8), indépendants**. Le code lit dst comme src et impose `dst = !src`, donc `ST A,Ymem‖LD Xmem,A` (0xC8, S=0,D=0) charge B au lieu de A.
- **(b) valeur stockée** L10890 : `data_write(ar[yar], st_val & 0xFFFF)` = sémantique STL. Attendu `Ymem = (src << ASM) >> 16` (`spru.txt@568612`, exemple ASM=1C).

**Patch minimal proposé pour A (remplace L10614-10923, `case 0xC/0xD` en entier)** — un seul handler, 1 mot, table de sous-classes :

```c
case 0xC: case 0xD: {
    /* 0xC000-0xDFFF : ST src,Ymem || <op> Xmem,dst  — 1 MOT, SPRU172C 4-177..4-185
     *   C0-C3 110000SD ADD | C4-C7 110001SD SUB | C8-CB 110010SD LD | CC-CF 110011SD MPY
     *   D0-D7 11010RSD MAC[R]                    | D8-DF 11011RSD MAS[R]
     * S=b9 (src du ST), D=b8 (dst de l'op), R=b10 (round, D-class). */
    int s_acc = (op >> 9) & 1;
    int d_acc = (op >> 8) & 1;
    int xmod  = (op >> 6) & 3, xar = ((op >> 4) & 3) + 2;
    int ymod  = (op >> 2) & 3, yar = ( op       & 3) + 2;
    uint16_t xaddr = s->ar[xar], yaddr = s->ar[yar];
    uint16_t xval  = data_read(s, xaddr);          /* lecture AVANT le store (SPRU 4-179 note ex.3) */
    int64_t  sv    = s_acc ? s->b : s->a;
    int64_t *dst   = d_acc ? &s->b : &s->a;
    int      asm5  = (int)(s->st1 & ST1_ASM_MASK); if (asm5 & 0x10) asm5 -= 32;
    int64_t  shifted = (asm5 >= 0) ? (sv << asm5) : (sv >> (-asm5));
    data_write(s, yaddr, (uint16_t)((shifted >> 16) & 0xFFFF));   /* Ymem = src<<ASM-16 */

    int64_t prod;
    switch ((op >> 10) & 7) {                       /* bits 12..10 */
    case 0: *dst = sext40(*dst + ((int64_t)(int16_t)xval << 16)); break;            /* C0-C3 ADD */
    case 1: *dst = sext40(((int64_t)(int16_t)xval << 16) - *dst); break;            /* C4-C7 SUB : Xmem<<16 - dst_ */
    case 2: *dst = sext40((int64_t)(int16_t)xval << 16); break;                     /* C8-CB LD  */
    case 3: prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval;
            if (s->st1 & ST1_FRCT) prod <<= 1; *dst = sext40(prod); break;          /* CC-CF MPY */
    case 4: case 5:                                                                 /* D0-D7 MAC[R] */
    case 6: case 7:                                                                 /* D8-DF MAS[R] */
        prod = (int64_t)(int16_t)s->t * (int64_t)(int16_t)xval;
        if (s->st1 & ST1_FRCT) prod <<= 1;
        if (op & 0x0400) { prod += 0x8000; prod &= ~0xFFFFLL; }                     /* R = b10 */
        *dst = sext40((op & 0x0800) ? (*dst - prod) : (*dst + prod));               /* b11 = MAS */
        break;
    }
    /* post-modify (inchangé, cf. bloc C8-CB actuel L10897-10920) */
    c54x_dualop_postmod(s, xar, xmod);
    c54x_dualop_postmod(s, yar, ymod);
    return consumed + s->lk_used;
}
```
(`T` n'est **jamais** écrit par cette famille — contrairement au code actuel.)

**NON-BUG / non auditable ici** : le découpage `Xmem = {2b mod, 2b AR+2}` est absent de SPRU172C (il est dans SPRU131) ; il est cohérent dans tout le fichier, je ne le conteste pas.

---

## B. FAMILLE 0xE0–0xEF

**E-1 / 0xE0-0xE3 — L8481-8500 — CONFIRMÉ — gravité 1 (E0), 2 (E1-E3)**
- Actuel : `if ((op & 0xFC00) == 0xE000)` → `CMPS src,Smem`, 1 mot, `resolve_smem`.
- Attendu : `0xE0 FIRS Xmem,Ymem,pmad` (**2 mots**), `0xE1 LMS`, `0xE2 SQDST`, `0xE3 ABDST`, tous `XXXXYYYY` (encodages décodés : `X01110000YXXYYYX`→`11100000 XXXXYYYY`, `X01111000YXXYYYX`→`11100001`, `X01110100YXXYYYX`→`11100010`, `X01111100YXXYYYX`→`11100011`). `CMPS` est en **0x8E00/0xFE00** (table L101).
- Conséquence : E0 consommé sur 1 mot au lieu de 2 → **désync**. E1/E2/E3 écrivent A/B et TC/TRN à tort.
- Effet de bord : ce catch-all **ombre entièrement** le handler `hi8==0xE1` L8612-8631 (code mort).
- Patch : borner à `(op & 0xFC00) == 0x8C00`-style hors plage E, c.-à-d. supprimer ce bloc de `case 0xE` et implémenter FIRS/LMS/SQDST/ABDST ; a minima, tant qu'ils ne sont pas implémentés :
```c
if (hi8 == 0xE0) { (void)prog_fetch(s, s->pc + 1); consumed = 2;   /* FIRS : 2 mots */
                   C54_LOG("FIRS unimpl op=0x%04x PC=0x%04x", op, s->pc);
                   return consumed + s->lk_used; }
if (hi8 >= 0xE1 && hi8 <= 0xE3) { C54_LOG("LMS/SQDST/ABDST unimpl 0x%04x", op);
                   return consumed + s->lk_used; }   /* 1 mot, bruyant, PAS de CMPS */
```

**E-2 / 0xE4 — L8565-8573 — CONFIRMÉ — gravité 1**
- Actuel : « BITF Smem,#lk » **2 mots**, `TC = (data[Smem] & op2) != 0`.
- Attendu : `ST src,Ymem ‖ LD Xmem,T`, **1 mot** — `spru.txt@567835` syntaxe 2 : `X01110S10YXXYYYX` → **`111001S0 XXXXYYYY`** (= 0xE400/0xFD00), exécution `src<<ASM-16 → Ymem ; Xmem → T`. `BITF Smem,#lk` = **0x6100/0xFF00** (table L67, `spru.txt` : `I01101000AAAAAAA` → `01100001 IAAAAAAA`).
- Conséquence : **désync PC +1 mot** à chaque site 0xE4xx, plus TC posé arbitrairement.

**E-3 / 0xE6 — L8656-8672 — CONFIRMÉ — gravité 2**
- Actuel : « SFTA/SFTL acc,#shift » sur `shift = op & 0x1F`, `dst = (op>>5)&1`, `logical = (op>>6)&1`.
- Attendu : `ST src,Ymem ‖ LD Xmem,T` avec S=1 (acc B) — même encodage `111001S0` que E4. `SFTA` = 0xF460/0xFCE0, `SFTL` = 0xF0E0/0xFCE0 (`spru.txt@551172` : `01111DS10T11FIHS` → `111101SD 011SHIFT` ; `11111DS00T11FIHS` → `111100SD 111SHIFT`).
- Conséquence : aucun store, T jamais chargé, et A/B décalé d'une valeur tirée des champs Xmem/Ymem.
- Patch (E-2 + E-3 fusionnés) :
```c
if (hi8 == 0xE4 || hi8 == 0xE6) {          /* ST src,Ymem || LD Xmem,T — 1 mot */
    int s_acc = (op >> 9) & 1;
    int xmod = (op>>6)&3, xar = ((op>>4)&3)+2, ymod = (op>>2)&3, yar = (op&3)+2;
    uint16_t xv = data_read(s, s->ar[xar]);
    int asm5 = (int)(s->st1 & ST1_ASM_MASK); if (asm5 & 0x10) asm5 -= 32;
    int64_t sv = s_acc ? s->b : s->a;
    int64_t sh = (asm5 >= 0) ? (sv << asm5) : (sv >> (-asm5));
    data_write(s, s->ar[yar], (uint16_t)((sh >> 16) & 0xFFFF));
    s->t = xv;
    c54x_dualop_postmod(s, xar, xmod); c54x_dualop_postmod(s, yar, ymod);
    return consumed + s->lk_used;          /* 1 mot */
}
```

**E-4 / 0xEF — L8632-8644 — PROBABLE — gravité 3**
- Actuel : `RPTZ dst,#lk` 2 mots, `rptz_dst = op & 1`.
- Attendu : **0xEF n'est assigné à rien** (absent du manuel et de la table). `RPTZ dst,#lk` = **0xF071/0xFEFF** (`spru.txt` : `01111D0001110001` → `1111000D 01110001`), déjà géré L7134. Ce handler est un doublon posé sur un opcode réservé ; il consomme 2 mots et arme un RPT si jamais du 0xEFxx est exécuté.
- Patch : remplacer par `goto unimpl;` (log bruyant).

**E-5 / 0xED20-0xEDFF — L8702-8722 — PROBABLE — gravité 3 (patron n°1)**
- Actuel : `BCD pmad,cond` 2 mots, avec `else take = true` (branche prise par défaut sur cond inconnue).
- Attendu : aucune instruction en 0xED20-0xEDFF (`LD #k5,ASM` = 0xED00/0xFFE0, correctement géré L8694). `BCD` = **0xFA00** (`spru.txt@428582` : `C11110Z01CCCCCCC` → `111110Z0`, Z=1 → 0xFA).
- Patch : remplacer L8702-8722 par `goto unimpl;`.

**E-6 / 0xE1 (L8612-8631), 0xEB (L8645-8655) — code mort — gravité 3**
- L8612 (`CMPL/NEG/SAT/ABS/ROR/ROL` sur `sub = op & 0xFF`) ombré par E-1 ; L8645 (`RPTB`) ombré par `(op & 0xFE00)==0xEA00` L8501 (`LD #k9,DP`, qui est **correct** : `1110101K KKKKKKKK`). Aucun effet runtime ; à supprimer pour éviter une réactivation accidentelle.

**NON-BUG dans 0xE0-0xEF (vérifiés, corrects)** : `0xEA/0xEB LD #k9,DP` L8501 (masque 0xFE00, k9 = op&0x1FF ✔) ; `0xEC RPT #k8` L8520 (0xEC00/0xFF00, et le moteur RPT L15948-15964 exécute bien **n+1** fois, `spru.txt@542279` « repeated n + 1 times ») ; `0xE5 MVDD` L8530 ; `0xE7 MVMM` L8574 (`M01111110YRMRMMX` → `11100111 MMRXMMRY`, src=b7:4, dst=b3:0 ✔) ; `0xEE FRAME K` L8673 (1 mot, k8 signé ✔) ; `0xED00-0xED1F LD #k5,ASM` L8694.

**E-7 / 0xE8-0xE9 `LD #K,dst` — L8589-8611 — PROBABLE — gravité 3**
- Actuel : `v = (ST1_SXM) ? (int8_t)k : k`.
- Manuel `spru.txt@476427` : opérande documenté **`0 ≤ K ≤ 255`** (non signé), encodage `K0111D001KKKKKKK` → `1110100D KKKKKKKK`. L'extension de signe conditionnée par SXM n'a pas d'appui : `LD #0x80,A` donne 0xFFFFFF80 au lieu de 0x80.
- Patch : `int64_t v = (int64_t)k;` (garder l'env `CALYPSO_LDK8_SHIFT16` inchangé).

---

## C. FAMILLE 0xF0–0xF3 (immédiats #lk, shifts 1-mot)

**F-1 / 0xF0B0-0xF0BF, 0xF1B0-0xF1BF — L7151-7158 — CONFIRMÉ — gravité 2**
- Actuel : `if ((op & 0x00F0) == 0x00B0)` → RSBX/SSBX ; `set = (op>>8)&1`, `st = (op>>5)&1`, `bit = op&0xF` → écrit ST0 **ou ST1**.
- Attendu : `RSBX`/`SSBX` = `111101N0/N1 1011SBIT` (`spru.txt` : `111110N10T10IBS1` → `111101N0 1011SBIT`, `111111N10T10IBS1` → `111101N1 1011SBIT`) donc **0xF4Bx / 0xF5Bx / 0xF6Bx / 0xF7Bx uniquement**. 0xF0Bx/0xF1Bx = `aop = (op>>5)&7 = 5` → **`OR src, SHIFT, dst`** avec SHIFT ∈ [−16,−1] (bits 4:0 = 0x10..0x1F).
- Conséquence : chaque `OR src,-N,dst` corrompt silencieusement ST0/ST1 (dont **le champ ASM = ST1[4:0]**, atteint par `bit = op&0xF` ≤ 15). Patron n°1 + n°3.
- Patch : supprimer L7151-7158 (RSBX/SSBX sont déjà traités en 0xF4Bx L7040, 0xF5Bx L8030, 0xF6Bx L7830, 0xF7Bx L8061).

**F-2 / 0xF171 `RPTZ B,#lk` — L7134-7146 + L7147-7150 — CONFIRMÉ — gravité 1**
- Actuel : seul `op == 0xF071` est reconnu. 0xF171 (D=1) tombe sur `alu_op = 7` → `goto unimpl` L7256, **1 mot consommé**.
- Attendu : `RPTZ dst,#lk` = 0xF071 **masque 0xFEFF**, 2 mots.
- Patch : `if ((op & 0xFEFF) == 0xF071) { ... int dst = (op >> 8) & 1; ... }` (idem `(op & 0xFFFF)` → `0xFEFF` pour F071 seulement ; F070/F072/F073/F074 gardent leurs masques).

**F-3 / 0xF080-0xF0FF, 0xF180-0xF1FF (`AND/OR/XOR src,SHIFT,dst`) — L7228-7250 — CONFIRMÉ — gravité 2**
- Actuel L7240-7242 : `*dst = sext40(sv) & sext40(shifted)` où `sv` = src et `shifted` = src<<SHIFT → calcule **`src & (src<<SHIFT)`**.
- Attendu (`spru.txt@45873`, Table 2-9 : « XOR src [, SHIFT] [, dst] → **dst = dst ^ src << SHIFT** » ; idem AND Table 2-7 et OR Table 2-8 ; table projet §F0 `100SHIFT`/`101SHIFT`/`110SHIFT`) : **`dst = dst <op> (src << SHIFT)`**.
- Patch :
```c
int64_t dst_in = dst_sel ? s->b : s->a;
case 4: *dst = sext40(dst_in & shifted); break;
case 5: *dst = sext40(dst_in | shifted); break;
case 6: *dst = sext40(dst_in ^ shifted); break;
```
- (`case 7 SFTL` est correct sur la valeur ; il ne remet pas à 0 les bits de garde 39-32 — `spru.txt@551255` « The guard bits of dst … are also cleared » — gravité 3.)

**F-4 / 0xF000-0xF05F, 0xF100-0xF15F — L7160-7191 — CONFIRMÉ — gravité 2**
- Actuel L7164-7165 : `src_sel = (op>>8)&1`, `dst_sel = (op>>9)&1`. Le commentaire L7160-7162 affirme « bit 8 = SRC …, bit 9 = DST » — **contredit** par le commentaire L7219-7227 du même fichier (« bit 9 = SRC, bit 8 = DST ») et par le bloc F2 L7371-7372 / F3 L7735-7736 qui font l'inverse.
- Attendu : **b9 = SRC, b8 = DST** — table projet §F0 en-tête, et manuel (`spru.txt@551172` SFTA `111101SD 011SHIFT`, où le champ `SHIFT` retombe exactement sur b4:b0, ce qui verrouille la permutation ; `RPTZ 1111000D` masque 0xFEFF confirme D=b8).
- Conséquence : `ADD #lk,SHFT,A,B` opère sur B et écrit A. (Le sous-cas `alu_op==2` LD est fortuitement correct puisqu'il utilise b8 comme destination.)
- Patch : `int src_sel = (op >> 9) & 1; int dst_sel = (op >> 8) & 1;` en L7164-7165 **et** L7196-7197 (`alu_op == 6`), en conservant `dst = src_sel? ...` → `dst = dst_sel? ...` pour les cas 0,1,3,4,5,7 et `dst = dst_sel` pour les cas 2 (LD) et 6 (MPY) qui n'ont pas de champ src.

**F-5 / 0xF2xx et 0xF3xx `#lk,SHFT` : SHFT traité comme signé — L7370 (F2), L7734 (F3) — CONFIRMÉ — gravité 2**
- Actuel : `int shift = (shift_raw & 0x8) ? (shift_raw - 16) : shift_raw;` puis `lk >> (-shift)` si négatif.
- Attendu : `spru.txt@26513` et `@28495` : « **SHFT 4-bit shift value 0 ≤ SHFT ≤ 15** » (par opposition à « SHIFT 5-bit value −16 ≤ SHIFT ≤ 15 »). `ADD #lk,9,src` doit décaler de +9, le code décale de −7.
- Le bloc F0/F1 L7168 fait `int shift = op & 0xF;` → **correct** ; F2 et F3 divergent.
- Patch (F2 L7369-7370 et F3 L7733-7734) :
```c
int shift = op & 0xF;                       /* SHFT non signé 0..15 */
int64_t lk_val = lk_signed << shift;
```

**F-6 / 0xF280-0xF2FF, 0xF380-0xF3FF (`AND/OR/XOR src,SHIFT,dst`) — L7403-7413 (F2), L7776-7791 (F3) — CONFIRMÉ — gravité 2**
- Actuel : `sh = dst_in << shift ; result = src & sh` → décale **dst** et combine avec **src**.
- Attendu : `dst = dst <op> (src << SHIFT)` — c'est **src** qui est décalé (même source que F-3).
- Patch :
```c
int64_t sh = (shift >= 0) ? (src << shift) : (src >> (-shift));
int64_t dst_in = dst_b ? s->b : s->a;
result = dst_in & sh;   /* resp. | et ^ */
```

**F-7 / 0xF2xx `MPY #lk,dst` absent — L7333-7358 — NON-BUG**
La liste `(op & 0xFCFF) == 0xF060..F065, F067` exclut 0xF066 ; mais `MPY #lk,dst` est encodé b9=0 (`spru.txt@505051` syntaxe 4 : `01111D000011110016-bit constant`, masque projet 0xF066/0xFEFF), donc n'existe pas en 0xF2xx/0xF3xx. Idem pour la même omission dans F3 L7676-7707. **Aucune divergence.**

**NON-BUG vérifiés en F0-F3** : `B/BD` 0xF073/0xF273, `CALL/CALLD` 0xF074/0xF274, `RPTB/RPTBD` 0xF072/0xF272 (encodages `111100Z0 0111 0011/0100/0010` confirmés `spru.txt@436177`, `@437803`) ; extension de signe/zéro de `#lk` (signée pour ADD/SUB/LD, non signée pour AND/OR/XOR) conforme à « logiques zéro-étendues ».

---

## D. FAMILLE 0xF4–0xF7 (accumulateur, statut, IT, retours)

**F-8 / inversion systématique SRC/DST — L6329, 6337, 6629, 6660, 6668, 6676, 6685, 6693, 6704, 6712 (+ doublons morts 6809, 6816, 6932, 6954-7008) — CONFIRMÉ — gravité 2**
- Actuel partout : `int src = (op >> 8) & 1, dst = (op >> 9) & 1;`
- Attendu : **`src = (op>>9)&1`, `dst = (op>>8)&1`**. Encodages décodés depuis `spru.txt` :
  `NEG` `11111DS100000100` → `111101SD 10000100` ; `ABS` → `111101SD 10000101` ; `CMPL` → `111101SD 10010011` ; `RND` → `111101SD 10011111` ; `SFTA` → `111101SD 011SHIFT` ; `NORM` → `111101SD 10001111`. Table projet §F0/§F4 : « b9=SRC, b8=DST ».
- Conséquence : `NEG A,B` (0xF584) exécute `A = -B` au lieu de `B = -A`. Invisible quand src==dst, faux dès que le firmware utilise la forme à deux opérandes. Concerne : ADD/SUB/LD `src,ASM,dst` (F480/81/82), ADD/SUB/LD/SFTA `src,SHIFT,dst` (F400/F420/F440/F460), NEG, ABS, MACA.
- Patch : échanger les deux extractions dans chacun de ces handlers.

**F-9 / `ADD/SUB/LD src,ASM,dst` ignorent ASM — L6660-6684 — CONFIRMÉ — gravité 2**
- Actuel : `int64_t sv = sext40(src?b:a);` puis `dst = dst ± sv` / `dst = sv`. **Aucun décalage.**
- Attendu (table §F4 ; `spru.txt@37198` Table 2-1 : « ADD src, ASM [, dst] → **dst = dst + src << ASM** ») : décalage par ASM = ST1[4:0] **signé** (−16..15).
- Patch (F480, idem F481/F482) :
```c
int src_i = (op >> 9) & 1, dst_i = (op >> 8) & 1;
int asm5 = (int)(s->st1 & ST1_ASM_MASK); if (asm5 & 0x10) asm5 -= 32;
int64_t sv = sext40(src_i ? s->b : s->a);
sv = (asm5 >= 0) ? (sv << asm5) : (sv >> (-asm5));
int64_t *d = dst_i ? &s->b : &s->a;
*d = sext40(*d + sv);          /* F481 : *d - sv ;  F482 : sv */
```

**F-10 / `NORM src[,dst]` — L6587-6606 (+ doublon mort L6891) — CONFIRMÉ — gravité 2**
- Actuel : décalage **conditionnel d'un bit** si b39≠b38, avec `T--` et positionnement de TC.
- Attendu (`spru.txt@521372`, p.4-122) : `Opcode 11111DS101001110` → `111101SD 10001111` ; **`Execution : src << TS → dst`**, TS = **T[5:0] en complément à 2** (−16..31), 1 cycle, *pas* de boucle, *pas* de modification de T, *pas* de TC. « Affected by SXM and OVM ; Affects OVdst ».
- Le commentaire L6588-6591 est faux (il cite « p.4-118 » et une sémantique inventée).
- Masque à corriger aussi : `0xFEFF` → **`0xFCFF`** (sinon `NORM B` = 0xF68F et `NORM B,B` = 0xF78F échappent — voir F-14/F-15).
- Patch :
```c
if ((op & 0xFCFF) == 0xF48F) {                 /* NORM src [,dst] */
    int src_i = (op >> 9) & 1, dst_i = (op >> 8) & 1;
    int ts = (int)(s->t & 0x3F); if (ts & 0x20) ts -= 64;      /* T[5:0] c2, -16..31 */
    int64_t v = sext40(src_i ? s->b : s->a);
    v = (ts >= 0) ? (v << ts) : (v >> (-ts));
    if (dst_i) s->b = sext40(v); else s->a = sext40(v);
    return consumed + s->lk_used;
}
```

**F-11 / `ROR` / `ROL` — L6607-6628 (+ doublons morts L6912, L6922) — CONFIRMÉ — gravité 2 (deux fautes)**
- (a) **lecture du carry au mauvais bit** : `uint16_t c = (s->st0 >> 8) & 1;` — or `calypso_c54x.h:74 #define ST0_C (1 << 11)`. Le bit 8 est un bit de **DP**. L'écriture, elle, utilise bien `ST0_C`.
- (b) **rotation sur 40 bits au lieu de 32** : le code injecte C en bit 39 et fait tourner les bits de garde. Manuel `spru.txt@542048` (ROR) / `@540816` (ROL) : `C → src31 ; src31-1 → src30-0 ; src0 → C ; **0 → src39-32**` — « The guard bits of src are cleared ». Exemple vérifiable : `ROR A`, A=`7F B000 1235`, C=0 → A=`00 5800 091A`, C=1.
- Patch ROR :
```c
uint64_t v32 = (uint64_t)(*acc) & 0xFFFFFFFFULL;
uint16_t cin = (s->st0 & ST0_C) ? 1 : 0;
uint16_t lsb = v32 & 1;
v32 = (v32 >> 1) | ((uint64_t)cin << 31);
*acc = sext40((int64_t)v32);                      /* guard = 0 */
if (lsb) s->st0 |= ST0_C; else s->st0 &= ~ST0_C;
```
  ROL symétrique (`msb = (v32 >> 31) & 1; v32 = ((v32 << 1) | cin) & 0xFFFFFFFF;`).
- Même défaut de bits de garde sur `ROLTC` L6544-6555 (`0 → src39-32`, `spru.txt@540816`) — **gravité 3** ; le reste de ROLTC (TC en entrée bit0, C en sortie depuis b31) est correct.

**F-12 / `MAX` / `MIN` ignorent dst — L6390-6412 (+ doublons morts L6940, L6947) — CONFIRMÉ — gravité 2**
- Actuel : écrit toujours dans A (`if (sa < sb) s->a = s->b;`).
- Attendu (`spru.txt@504338`) : `MAX dst` = `1111010D 10000110` (0xF486), `MIN dst` = `1111010D 10000111` (0xF487), **D = b8** : `If A > B Then A → dst, 0 → C Else B → dst, 1 → C`. `MAX B` (0xF586) doit écrire B.
- Patch : `int64_t *d = ((op >> 8) & 1) ? &s->b : &s->a; if (sa < sb) { *d = sext40(sb); s->st0 |= ST0_C; } else { *d = sext40(sa); s->st0 &= ~ST0_C; }`

**F-13 / `CMPL` et `RND` : dst ignoré et src mal lu — L6642-6659 — CONFIRMÉ — gravité 2**
- Actuel : `int src = (op >> 8) & 1; acc = src?&b:&a; *acc = ~*acc;` → écrit dans l'accumulateur désigné par b8 (qui est **dst**), en ignorant b9 (**src**).
- Attendu : `CMPL` `111101SD 10010011` masque 0xFCFF ; `RND` `111101SD 10011111` masque 0xFCFF (`spru.txt@540816` : « RND src [, dst] … src + 8000h → dst », exemple `RND A,B` : A inchangé, B modifié).
- Patch : `int si=(op>>9)&1, di=(op>>8)&1; int64_t v = sext40(si?s->b:s->a); if (di) s->b = sext40(~v); else s->a = sext40(~v);` (resp. `v + 0x8000`).

**F-14 / `0xF4A0-0xF4AF` avalés en `SFTL` : `LD #k3,ARP` et `CMPR CC,ARx` perdus — L6743-6757 — CONFIRMÉ — gravité 2**
- Actuel : `if ((op & 0xFCE0) == 0xF4A0 && (op & 0xF0) != 0xB0)` → décalage logique de l'accumulateur. Couvre 0xF4A0-0xF4AF, 0xF5A0-AF, 0xF6A0-AF, 0xF7A0-AF.
- Attendu : le vrai `SFTL src,SHIFT[,dst]` est en **0xF0E0/0xFCE0** (`11111DS00T11FIHS` → `111100SD 111SHIFT`, `spru.txt@551255`) — déjà implémenté en F0/F1 L7247, F2 L7415, F3 L7793. La plage 0xF4A0-0xF4AF contient :
  - `LD #k3, ARP` = **0xF4A0/0xFFF8** (table §F4 ; Class 1 liste « LD T/DP/ASM/ARP », `spru.txt@78692`) ;
  - `CMPR CC, ARx` = **0xF4A8/0xFCF8** — `spru.txt@442568` : `11111CC10X10RA10` → **`111101CC 10101ARX`**, « compare ARx à AR0 en **non signé**, pose TC ; CC : 00=EQ, 01=LT, 10=GT, 11=NEQ » (CC = b9:b8 → 0xF4A8/F5A8/F6A8/F7A8).
- Conséquence : toute boucle firmware pilotée par `CMPR` voit TC figé **et** un accumulateur décalé de `op & 0x1F`.
- Patch (insérer **avant** le bloc L6743, et supprimer complètement le pseudo-SFTL 0xF4A0) :
```c
if ((op & 0xFFF8) == 0xF4A0) {                       /* LD #k3, ARP */
    s->st0 = (s->st0 & ~ST0_ARP_MASK) | ((op & 7) << ST0_ARP_SHIFT);
    return consumed + s->lk_used;
}
if ((op & 0xFCF8) == 0xF4A8) {                       /* CMPR CC, ARx  (non signé) */
    unsigned ax = s->ar[op & 7], a0 = s->ar[0];
    bool tc;
    switch ((op >> 8) & 3) {
    case 0: tc = (ax == a0); break;  case 1: tc = (ax <  a0); break;
    case 2: tc = (ax >  a0); break;  default: tc = (ax != a0); break;
    }
    if (tc) s->st0 |= ST0_TC; else s->st0 &= ~ST0_TC;
    return consumed + s->lk_used;
}
```

**F-15 / `FRETE` (0xF4E5) exécuté en NOP — L6207-6212 — CONFIRMÉ — gravité 1**
- Actuel : `if (op >= 0xF4E0 && op <= 0xF4FF && op != 0xF4E1 && op != 0xF4E4 && op != 0xF4EB) return consumed;` → 0xF4E5 tombe dans ce NOP muet (F4E2/E3/E6/E7 sont interceptés plus haut).
- Attendu : `FRETE[D]` = `111101Z0 11100101` (`spru.txt`, décodé) = **0xF4E5/0xFDFF** ; `XPC = TOS, ++SP ; PC = TOS, ++SP ; INTM = 0`.
- Conséquence : retour d'interruption lointaine transformé en NOP → **2 mots orphelins sur la pile + pas de retour + INTM jamais reclearé** (exactement le symptôme « INTM stuck » déjà chassé pour RETED).
- Patch : ajouter `&& op != 0xF4E5` à la garde L6208, et un handler identique à `FRETD/FRETED` L7959-7996 sans les delay slots :
```c
if (op == 0xF4E5) {                              /* FRETE (non différé) */
    s->xpc = data_read(s, s->sp) & 3; s->sp++;
    uint16_t ra = data_read(s, s->sp); s->sp++;
    s->st1 &= ~ST1_INTM;
    s->pc = ra; return 0;
}
```

**F-16 / `RETF[D]` (0xF49B / 0xF69B) — L7061 — CONFIRMÉ — gravité 1**
- Actuel : 0xF49B n'est capté par aucun masque → **`C54_LOG("F4xx unhandled")` + NOP 1 mot** (log non plafonné, L7058-7062). (0xF69B `RETFD` est, lui, correctement traité L7877.)
- Attendu : `RETF[D]` = `111110Z101001011` → **`111101Z0 10011011`** = 0xF49B/0xFDFF ; `PC = RTN ; ++SP ; INTM = 0` (`spru.txt`, Table 2-15 @49420).
- Patch :
```c
if (op == 0xF49B) {                              /* RETF (non différé) */
    uint16_t ra = data_read(s, s->sp); s->sp++;
    s->st1 &= ~ST1_INTM;
    s->pc = ra; return 0;
}
```

**F-17 / `MACAR` / `MASA` / `MASAR` (0xF489/0xF48A/0xF48B et variantes S/D) non implémentés — L6629 / L7058 / L8038 / L8008 / L8085 — CONFIRMÉ — gravité 1**
- Actuel : seul `(op & 0xFCFF) == 0xF488` (MACA) existe. Les trois autres partent selon hi8 dans quatre fallbacks **différents et tous destructeurs** :
  | opcode | chemin | effet |
  |---|---|---|
  | 0xF489/8A/8B | L7058 | log + NOP (bruyant) |
  | 0xF589/8A/8B | **L8038** | `RPT #k8` muet (`rpt_count = 0x89..0x8B`) → **répétition de ~140 fois** de l'instruction suivante |
  | 0xF689/8A/8B, **0xF68F (`NORM B,A`)** | **L8008** | blanket `MVDD` → `data_write` arbitraire |
  | 0xF789/8A/8B, **0xF78F (`NORM B[,B]`)** | **L8085 `case 0x8`** | `s->t = 0x89..0x8F` |
- Attendu : table §F4 — `0xF488/0xF489 MACA[R] T,src[,dst]`, `0xF48A/0xF48B MASA[R] T,src[,dst]`, masque 0xFCFF (`spru.txt@504338` : « MASA T,B → B = B − T×A[32:16] », exemples 2 et 3 avec/sans arrondi).
- Patch : élargir le handler L6629 et ajouter les trois frères :
```c
if ((op & 0xFCFC) == 0xF488) {                   /* MACA[R]/MASA[R] T, src [,dst] */
    int si=(op>>9)&1, di=(op>>8)&1, is_sub = (op & 2), rnd = (op & 1);
    int64_t p = (int64_t)(int16_t)s->t * (int64_t)(int16_t)((s->a >> 16) & 0xFFFF);
    if (s->st1 & ST1_FRCT) p <<= 1;
    if (rnd) { p += 0x8000; p &= ~0xFFFFLL; }
    int64_t sv = sext40(si ? s->b : s->a);
    int64_t r  = is_sub ? (sv - p) : (sv + p);
    if (di) s->b = sext40(r); else s->a = sext40(r);
    return consumed + s->lk_used;
}
```
  (le multiplicande est **A[32:16]**, pas `src>>16` comme au L6631.)

**F-18 / `INTR K` (0xF7C0-0xF7DF) exécuté en `LD #k8,BK` / `LD #k8,SP` — L8111 et L8125 — CONFIRMÉ — gravité 1**
- Actuel : le bloc `if (hi8 == 0xF7)` L8071-8130 est un dispatch **`LD #k8, <registre>` entièrement inventé** (`sub = (op>>4)&0xF` → ASM, AR0..AR7, T, DP, ARP, BK, SP). Le commentaire L8073 dit « F7C0..F7DF = INTR k (handled elsewhere if implemented) » mais la garde L8081 n'exclut que `sub == 0xE || 0xF` : `sub = 0xC` → **`s->bk = k`**, `sub = 0xD` → **`s->sp = k`** (via `sp_abs_track`). **Patron n°6** : commentaire contredit par le code deux lignes plus bas.
- Attendu : `INTR K` = `111111110K01KKKK` → **`11110111 110KKKKK`** = **0xF7C0/0xFFE0** (`spru.txt@476778`, p.4-65) : `--SP ; PC+1 → TOS ; vecteur(K) → PC ; **1 → INTM** ; bit K de l'IFR effacé ; IMR sans effet ». Ceci **confirme** le désaccord n°1 de la table projet (INTR ≠ 0xF300).
- Conséquence : un `INTR 0` (0xF7C0) écrase **SP** par 0xC0 → effondrement immédiat de la pile.
- Patch (insérer avant L8071) :
```c
if ((op & 0xFFE0) == 0xF7C0) {                   /* INTR K */
    int k = op & 0x1F;
    s->sp--; data_write(s, s->sp, (uint16_t)(s->pc + 1));
    uint16_t iptr = (s->pmst >> PMST_IPTR_SHIFT) & 0x1FF;
    s->ifr &= ~(1u << k);
    s->st1 |= ST1_INTM;                          /* TRAP ne le fait PAS, INTR si */
    s->pc = (iptr * 0x80) + k * 4;
    return 0;
}
```
  Et, plus largement, **supprimer tout le dispatch `LD #k8,<reg>` L8085-8130** : aucune de ces instructions n'existe dans l'ISA C54x (`LD #k5,ASM`=0xED00, `LD #k9,DP`=0xEA00, `LD #k3,ARP`=0xF4A0, `LD #K,dst`=0xE800 ; il n'existe **aucun** `LD #k8,ARn/T/BK/SP`). Le remplacer par `goto unimpl;`.

**F-19 / `BACCD B` / `CALAD B` / `FBACCD B` / `FCALAD B` (0xF7E2/E3/E6/E7) exécutés en NOP — L8081-8084 — CONFIRMÉ — gravité 1**
- Actuel : L7883 et L7997 ne testent que `op == 0xF6E2/F6E3` et `0xF6E6/F6E7` (accumulateur A hardcodé) ; les variantes B tombent sur `sub == 0xE` L8081 → NOP muet.
- Attendu : `BACC[D] src` = `111101ZS 11100010` (Z=b9, **S=b8**) → BACCD A=0xF6E2, **BACCD B=0xF7E2** ; idem CALA (…0011), FBACC (…0110), FCALA (…0111) — `spru.txt@425781`, `@436177`.
- Conséquence : branchement/appel différé silencieusement supprimé → chute dans le code suivant.
- Patch : remplacer les tests exacts par `if ((op & 0xFEFF) == 0xF6E2 || (op & 0xFEFF) == 0xF6E3)` et lire `int64_t acc = (op & 0x0100) ? s->b : s->a;` (idem pour F6E6/F6E7).

**F-20 / `hi8 == 0xF5` : fallback `RPT #k8` muet — L8027-8042 — CONFIRMÉ — gravité 1 (patron n°1)**
- Actuel : hors 0xF5Bx (SSBX ST0), **tout** 0xF5xx non capté plus haut exécute `rpt_count = op & 0xFF ; rpt_active = true ; pc += 1 ; return 0` — sans le moindre log.
- Attendu : 0xF5xx = variante {S=A, D=B} de la famille F4xx + SSBX ST0 + IDLE 2 + BACC/CALA/FBACC/FCALA B. Rien n'y est un `RPT` (`RPT #K` = 0xEC00, `RPT #lk` = 0xF070, `RPT Smem` = 0x4700).
- Cible concrète aujourd'hui : 0xF589/0xF58A/0xF58B (F-17) et tout opcode réservé.
- Patch : remplacer L8038-8041 par `goto unimpl;`.

**F-21 / `hi8 == 0xF6`, `sub >= 0x8` : blanket `MVDD Xmem,Ymem` — L8008-8022 — CONFIRMÉ — gravité 1**
- Actuel : pour tout 0xF680-0xF6FF non exact-matché, `data_read(ar[(op>>4)&7])` → `data_write(ar[op&7])`, avec un champ AR **3 bits** et un mod **1 bit** — encodage incompatible avec le reste du fichier (2 bits mod + AR2..AR5) et donc capable d'écrire via AR0/AR1 (aliasing MMR, cf. les incidents SP/IMR déjà documentés L10826-10847).
- Attendu : `MVDD` est en **0xE500/0xFF00** uniquement (`X01111010YXXYYYX` → `11100101 XXXXYYYY`), déjà correctement implémenté L8530. En 0xF6xx on trouve `NORM B,A` (0xF68F), `MACAR/MASA[R]` (0xF689-8B) et du réservé.
- Patch : remplacer L8008-8022 par `goto unimpl;` (après avoir appliqué F-10 et F-17 qui récupèrent les opcodes légitimes).

**F-22 / `0xF7E0 RESET` traité en NOP — L8081-8084 — PROBABLE — gravité 3**
`RESET` = `1111111100110000` → **`11110111 11100000`** = 0xF7E0/0xFFFF (`spru.txt`, décodé). Le NOP est un choix documenté L8074-8080 ; à conserver mais à rendre **bruyant** (`C54_LOG`) plutôt que muet.

**NON-BUG vérifiés en F4-F7** :
`NOP` 0xF495 (L5963, `1111100101000101` → `11110100 10010101`) ✔ ; `TRAP K` L6785 (0xF4C0/0xFFE0, `PC = IPTR<<7 + K<<2`, **ne pose pas INTM** ✔ — seule différence avec INTR) ; `IDLE K` L6263 (0xF4E1/0xFCFF, `111101NN 11100001`) ✔ ; `BACC/CALA A/B` L6034 (b8=S ✔, CALA pousse `PC+1` ✔ conforme à l'exemple `spru.txt@436177` SP 1111→1110, mem=0026 pour PC=0025) ; `FBACC/FCALA` L6162 ✔ ; `FRET` L6263 (pop PC puis XPC — ordre symétrique de FCALA ✔) ; `RETE` L6213 ; `RSBX ST0` L7040 (0xF4B0), `RSBX ST1` L7830 (0xF6Bx), `SSBX ST0` L8030 (0xF5Bx), `SSBX ST1` L8061 (0xF7Bx) — les quatre couvrent exactement `111101N0/N1 1011SBIT` ✔ ; `SAT` L6318 (0xF483, acc=b8 ✔ car b9 fixé 0), `EXP` L6366 (0xF48E, b8 ✔), `ROLTC` L6544 (0xF492, b8 ✔), `SFTC` L6763 (0xF494, b8 ✔) ; `MPYA dst` L6345 / `SQUR A,dst` L6355 (b8 ✔).
**Code mort** (inoffensif mais piégeux) : tout le bloc dupliqué L6799-7038 à l'intérieur de `if (hi8 == 0xF4)`, qui contient les **anciennes** affectations erronées (F492=MAX, F493=MIN, F486=CMPL, F487=RND, F49F=DELAY) — inaccessible car le bloc « promu » L6300-6790 retourne avant. À supprimer.

---

## E. FAMILLE 0xF8–0xFF (branchements, appels, retours, XC)

**Vérité** (`spru.txt@428582`, `@437355`, `@531308`, `@586348`) :
`BC[D]` `111110Z0 CCCCCCCC` (F8/FA) · `CC[D]` `111110Z1 CCCCCCCC` (F9/FB) · `RC[D]` `111111Z0 CCCCCCCC` (FC/FE) · `XC n` `111111N1 CCCCCCCC` (FD/FF) · `FB[D]`/`FCALL[D]` = même base + **b7=1**.
**Table des codes de condition** (identique sur les 5 pages) : UNC 0x00, NBIO 0x02, BIO 0x03, NC 0x08, C 0x0C, NTC 0x20, TC 0x30, AGEQ 0x42, ALT 0x43, ANEQ 0x44, AEQ 0x45, AGT 0x46, ALEQ 0x47, B* = +0x08, ANOV 0x60, BNOV 0x68, AOV 0x70, BOV 0x78.
**Conditions multiples = ET**, preuve directe `spru.txt@431785` : `BC 1000h, TC, NC, BIO` avec C=1 → « After Instruction PC **3002** » (branche **non** prise).

**F-23 / `0xF800-0xF81F` et `0xF860-0xF87F` décodés en `BANZ` — L7623-7634 et L7569-7590 — CONFIRMÉ — gravité 2**
- Actuel : `sub <= 1` → `BANZ pmad,Sind` (teste `ar[op&7]`) ; `sub == 6 || 7` → `BANZ *ARn,pmad` (teste **et décrémente** `ar[op&7]`).
- Attendu : `BANZ[D] pmad,Sind` = **0x6C00/0x6E00** (`I01100Z11AAAAAAA` → `011011Z0 IAAAAAAA`, `spru.txt@426713`) — encodage totalement disjoint. 0xF80x/0xF81x = `BC pmad, cond` avec cc ∈ {UNC 0x00, NBIO, BIO, NC, C} ; 0xF86x/0xF87x = `BC pmad, cond` avec cc ∈ {ANOV 0x60, BNOV 0x68, AOV 0x70, BOV 0x78} et leurs combinaisons de groupe 1.
- Conséquence : `BC pmad,C` branche sur `AR4≠0` ; `BC pmad,AOV` branche sur `AR0≠0` **et décrémente AR0**.
- La note d'orientation « BANZ (0x78/0x7A, PAS 0xEA) » de `doc/C54X_INSTRUCTIONS.md` est elle aussi fausse : 0x78/0x7A = `MACP/MACD Smem,pmad,src` (`I1110S001AAAAAAA` → `0111100S`, `I1110S101AAAAAAA` → `0111101S`). Le seul BANZ est 0x6C/0x6E.

**F-24 / `0xF820/0xF830` : heuristique ACC au lieu de NTC/TC — L7488-7538 — CONFIRMÉ (divergence assumée) — gravité 2**
- Actuel : par défaut `take = (A != 0)` pour 0xF82x et `take = (A == 0)` pour 0xF83x, sauf si l'opcode précédent est 0x60xx/0x61xx (`g_prev_op & 0xFE00 == 0x6000`) ou si `CALYPSO_C54X_FIX_BC=1`.
- Attendu : cc 0x20 = NTC, cc 0x30 = TC, sans condition sur l'instruction précédente.
- Le commentaire reconnaît le problème et pointe la vraie cause suspectée (BITF/TC). Or **BITF est bien à 0x61** (`I01101000AAAAAAA` → `01100001 IAAAAAAA`) et le handler 0xE4 usurpe son rôle (**E-2**) : corriger E-2 est très probablement le préalable qui permettra d'activer `CALYPSO_C54X_FIX_BC` sans casse. Je ne propose pas de basculer le défaut ici — seulement de tester `CALYPSO_C54X_FIX_BC=1` **après** E-2.

**F-25 / OVA lu au bit 8 au lieu du bit 10 — L4872 (`c54x_cond_true`), L8301 (RC), L8413 (RCD) — CONFIRMÉ — gravité 2**
- Actuel : `bool ov = (cc & 0x08) ? (s->st0 & (1<<9)) : (s->st0 & (1<<8));`
- Attendu : `calypso_c54x.h:72-73` — `ST0_OVB (1<<9)`, **`ST0_OVA (1<<10)`**. Le bit 8 est le MSB de DP. Les conditions `AOV`/`ANOV` (cc 0x70/0x60) sur A lisent donc un bit de page de données.
- Le handler XC L5993-5996 utilise correctement `ST0_OVA`.
- Patch (3 sites) : `bool ov = (cc & 0x08) ? !!(s->st0 & ST0_OVB) : !!(s->st0 & ST0_OVA);`

**F-26 / conditions combinées : OU au lieu de ET, ou condition ignorée — L6000-6019 (XC), L4869-4884 (`c54x_cond_true`), L8294-8319 (RC), L8406-8431 (RCD) — CONFIRMÉ — gravité 2**
- Actuel :
  - XC L6002-6018 : `cond |= …` sur trois catégories → **OU** ; et la détection de présence utilise `cc & 0x0C` / `cc & 0x30` alors que les bits *sélecteurs* sont b3 (C), b5 (TC), b1 (BIO), les bits b2/b4/b0 n'étant que la polarité. Ainsi `cc = 0x46` (AGT, b2=1) déclencherait à tort le test C s'il n'était pas dans la liste explicite ; `cc = 0x76` (AGT+AOV) le déclenche effectivement.
  - `c54x_cond_true`, RC, RCD : pour `cc & 0x40`, si `(cc & 0x70) == 0x70` la fonction retourne **uniquement** OV et **jette** la condition de catégorie A (`cc & 7`).
- Attendu : ET de toutes les conditions présentes (preuve `spru.txt@431785`, cf. supra).
- Patch (fonction unique, à réutiliser par XC/BC/CC/RC/RCD) :
```c
static bool c54x_cond_true(C54xState *s, uint8_t cc)
{
    if (cc == 0x00) return true;                                  /* UNC */
    bool r = true;
    if (cc & 0x40) {                                              /* groupe 1 */
        int64_t acc = (cc & 0x08) ? sext40(s->b) : sext40(s->a);
        if (cc & 0x07) switch (cc & 0x07) {                       /* cat. A */
        case 0x2: r &= (acc >= 0); break;  case 0x3: r &= (acc <  0); break;
        case 0x4: r &= (acc != 0); break;  case 0x5: r &= (acc == 0); break;
        case 0x6: r &= (acc >  0); break;  case 0x7: r &= (acc <= 0); break;
        default:  return false;                                   /* 0/1 réservés */
        }
        if (cc & 0x20) {                                          /* cat. B : OV/NOV */
            bool ov = (cc & 0x08) ? !!(s->st0 & ST0_OVB) : !!(s->st0 & ST0_OVA);
            r &= (cc & 0x10) ? ov : !ov;
        }
        return r;
    }
    if (cc & 0x20) r &= (cc & 0x10) ? !!(s->st0 & ST0_TC) : !(s->st0 & ST0_TC);
    if (cc & 0x08) r &= (cc & 0x04) ? !!(s->st0 & ST0_C)  : !(s->st0 & ST0_C);
    if (cc & 0x02) r &= (cc & 0x01) ? c54x_bio_low(s)     : !c54x_bio_low(s);
    return r;
}
```
  puis remplacer les blocs dupliqués L8294-8319 et L8406-8431 par un appel à cette fonction, et le bloc XC L5975-6019 de même.

**F-27 / RC/RCD : `else cond = true` sur condition inconnue — L8320 et L8432 — CONFIRMÉ — gravité 3 (patron n°1)**
Avec la refonte F-26 le cas disparaît. En l'état, `RC BIO` (cc=0x03) et `RC NBIO` (0x02) **retournent inconditionnellement**.

**F-28 / `0xFA00-0xFA7F` : branche inconditionnelle par défaut — L8237-8238 — CONFIRMÉ — gravité 2 (patron n°1)**
- Actuel : hors 0xFA8x (FBD) et hors 0xFA2x/0xFA3x précédés d'un 0x60xx/0x61xx, `s->pc = op2; return 0;` — **BCD prise quelle que soit la condition**, et sans les 2 delay slots.
- Attendu : `BCD pmad, cond` = `111110Z0` avec Z=1 → cond évaluée sur `op & 0xFF`, branchement **différé** (2 delay slots).
- Patch : après le bloc FBD, remplacer L8200-8238 par
```c
if (c54x_cond_true(s, (uint8_t)(op & 0xFF))) { s->delayed_pc = op2; s->delay_slots = 2; }
return consumed + s->lk_used;
```
  (à activer derrière un env si l'on veut préserver le comportement actuel le temps de valider E-2/F-24 — les régressions de 2026-05-15 sont attribuées à TC, cf. F-24.)

**F-29 / `hi8 == 0xFD` → `LD #k, A` — L8393-8397 — CONFIRMÉ (code mort) — gravité 3**
Ombré par le handler XC L5970 (`hi8 == 0xFD || hi8 == 0xFF`). Aucun effet runtime, mais c'est une classification fausse (0xFD = `XC 1,cond`) qui redeviendrait active si l'ordre changeait. À supprimer.

**F-30 / `0xF9xx` / `0xFBxx` : condition tronquée à 7 bits — L8163 et L8271 — PROBABLE — gravité 3**
`c54x_cond_true(s, (uint8_t)(op & 0x7F))` : le champ cond est sur **8 bits** (`CCCCCCCC`). Ici b7 est le discriminant NEAR/FAR (déjà testé juste avant), donc pour un `CC` NEAR b7 vaut 0 et le masque 0x7F est sans effet — **NON-BUG en pratique**, mais utiliser `op & 0xFF` serait plus lisible et cohérent avec `RC`.

**NON-BUG vérifiés en F8-FF** :
`XC 1/2` L5970-6025 — masque et **longueur** corrects : `return 1 + n_insns` = 1 mot (XC) + 1 mot (n=1) ou + 2 mots (n=2), ce qui couvre exactement « n=2 → une instruction 2-mots **ou** deux instructions 1-mot » (`spru.txt@586348`, p.4-199). La table de conditions explicite L5977-5996 est **exacte, code par code**, vs la table du manuel. ✔
`FB` 0xF880/0xFF80 L7546 ✔ (`111110Z0 1PPPPPPP`, XPC = 7 bits bas du mot 1) ; `FCALL` 0xF980/0xFF80 L8141 (push XPC puis PC+2 — symétrique de FRET L6263) ✔ ; `FCALLD` 0xFB80 L8253 (push PC+**4**) ✔ ; `CC` L8163 (push PC+2, non différé) et `CCD` L8271 (push PC+**4** + 2 delay slots) ✔ conformes à `spru.txt@437803` (« Nondelayed : PC+2 → TOS ; Delayed : PC+4 → TOS ») ; `RC/RET` L8289 **1 mot** ✔ ; `RCD/RETD` L8401 avec `delay_slots = 2` ✔ ; masques FC/FE vs FD/FF (b8 discriminant) ✔ ; `BC` 0xF84x/0xF85x L7591-7610 : mapping `cc&7` correct (2=GEQ, 3=LT, 4=NEQ, 5=EQ, 6=GT, 7=LEQ) et `cc&0x08` = accumulateur B ✔, mais il ignore une éventuelle condition OV combinée (couvert par F-26).
**Code mort** en F8 : L7563-7568 (`sub 8..B` → B pmad) et L7611-7622 (`sub ≥ C` → CALL) sont ombrés par le catch FB L7546 ; le fallback `RPT Smem` L7635-7639 est **inatteignable** (tous les `sub` 0-F sont captés avant) — contrairement à ce qu'indique la carte fournie.

---

## F. Constat transverse (hors plage stricte mais causé par elle)

**T-1 / handlers 0x28xx et 0x3xxx écrits à l'intérieur de `case 0xF:` — L6422, 6450, 6469, 6481, 6493, 6507, 6519, 6527, 6544 — CONFIRMÉ — gravité 2**
Vérifié par analyse de profondeur d'accolades : `switch (hi4)` L5960, `case 0xF:` L5961, `case 0xE:` L8475. Les tests `(op & 0xFC00) == 0x2800`, `== 0x3500`, `0x3300`, `0x3700`, `0x3100`, `0x3000`, `0x3200`, `0x3400` sont donc **structurellement inatteignables** (`hi4` vaut 0xF). `MAC/MACR/MAS/MASR Smem`, `MACA`, `MASA`, `MACAR`, `MPYA`, `LD Smem,T`, `LD Smem,ASM`, `BITT` sont écrits mais morts — **c'est la cause directe** du blanket `acc += T*Smem` de `case 0x3` (L9527) et du `default:` MAS de `case 0x2` (L9545) relevés dans la carte.
Patch : déplacer ces neuf blocs hors de `case 0xF:`, dans `case 0x2:` / `case 0x3:`.

---

## Récapitulatif par gravité

**Gravité 1 (désynchronise le décode / détruit la pile ou le flot)** : C-1 (0xC2/C3/C6/C7), C-2 (0xC4), C-3 (0xDA), C-4 (0xC1), E-1 (0xE0 FIRS), E-2 (0xE4), F-2 (0xF171), F-15 (0xF4E5 FRETE), F-16 (0xF49B RETF), F-17 (MACAR/MASA/MASAR → RPT/MVDD/T), F-18 (INTR K → SP), F-19 (BACCD/CALAD acc B), F-20 (fallback RPT en 0xF5), F-21 (blanket MVDD en 0xF6).

**Gravité 2 (résultat faux silencieux)** : C-5, C-6, C-7, C-8, C-9, E-1 (E1-E3), E-3, F-1, F-3, F-4, F-5, F-6, F-8, F-9, F-10, F-11, F-12, F-13, F-14, F-23, F-24, F-25, F-26, F-28, T-1.

**Gravité 3 (cas de bord / code mort)** : E-4, E-5, E-6, E-7, F-22, F-27, F-29, F-30, doublons morts L6799-7038, code mort F8 L7563/L7611/L7635.

**Familles correctes** : `0xEA/0xEB LD #k9,DP` · `0xEC RPT #K` (avec moteur n+1) · `0xE5 MVDD` · `0xE7 MVMM` · `0xEE FRAME K` · `0xED00-1F LD #k5,ASM` · `0xF072/F073/F074` et différés `0xF272/F273/F274` · `0xF4C0 TRAP` · `0xF4E1 IDLE` · `0xF4E2/E3/E6/E7` (+ `0xF5`) BACC/CALA/FBACC/FCALA · `0xF4E4 FRET` · `0xF4EB RETE` · `0xF6EB RETED` · `0xF69B RETFD` · `0xF6E4/E5 FRETD/FRETED` · les 4 RSBX/SSBX · `0xF880 FB` · `0xF980 FCALL` · `0xFB80 FCALLD` · `0xF900 CC` · `0xFB00 CCD` · `0xFC00 RC/RET` · `0xFE00 RCD/RETD` (longueurs et pushes) · `0xFD/0xFF XC` (masque, longueur de saut, table de conditions).