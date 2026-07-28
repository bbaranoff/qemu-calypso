---
name: C54x instruction encoding from SPRU172C + binutils
description: Encodages verifies contre binutils tic54x-opc.c (autorite pour encodage/longueur) et SPRU172C (autorite pour la semantique)
type: project
---

> **Ordre d'autorite** (pose le 2026-07-28, apres plusieurs fausses pistes) :
> 1. `doc/opcodes/tic54x-opc.c` — la table binutils, copiee dans le depot. Format :
>    `{ "mnemo", MOTS, cycles, classe, OPCODE, MASQUE, {operandes}, flags }`.
>    **Le champ MOTS fait foi** : une longueur fausse desynchronise tout le decodage en aval.
> 2. `doc/spru172c.pdf` — le manuel TI, autorite pour la SEMANTIQUE d'execution.
> 3. le code, puis ce fichier.
> Ne jamais conclure depuis un commentaire de code : plusieurs se sont averes perimes.

## XC (Execute Conditionally) — SPRU172C p.4-198
- Opcode: `1111 11N1 CCCCCCCC` (1 word)
- N=0 (bit 9=0) → n=1 instruction = opcode 0xFDxx
- N=1 (bit 9=1) → n=2 instructions = opcode 0xFFxx
- If cond true: execute next n instructions normally
- If cond false: treat next n instructions as NOP
- Condition codes (8-bit, combinable):
  - UNC=0x00, BIO=0x03, NBIO=0x02, C=0x0C, NC=0x08
  - TC=0x30, NTC=0x20
  - AEQ=0x45, ANEQ=0x44, AGT=0x46, AGEQ=0x42, ALT=0x43, ALEQ=0x47
  - BEQ=0x4D, BNEQ=0x4C, BGT=0x4E, BGEQ=0x4A, BLT=0x4B, BLEQ=0x4F
  - AOV=0x70, ANOV=0x60, BOV=0x78, BNOV=0x68

## FRET — SPRU172C p.4-61
- Opcode: `1111 0Z01 1101 0100` (Z=0 normal, Z=1 delayed)
- Execution: (TOS)→XPC, SP+1, (TOS)→PC, SP+1
- **Pops 2 words**: first XPC, then PC

## FCALL — SPRU172C p.4-57
- Opcode: `1111 10Z1 1` + 7-bit pmad(22-16) + 16-bit pmad(15-0) 
- 2 words total
- Execution: SP-1, PC+2→TOS, SP-1, XPC→TOS, pmad(15-0)→PC, pmad(22-16)→XPC
- **Pushes 2 words**: first PC+2, then XPC

## RETE — SPRU172C p.4-140
- Execution: (TOS)→PC, SP+1, 0→INTM
- **Pops 1 word** (just PC), clears INTM

## TRAP K — SPRU172C p.4-195
- Pushes PC+1, branches to interrupt vector K
- Not affected by INTM

## 0xEA00 = LD #k9, DP  (PAS BANZ, PAS XC) — rectifie 2026-07-28
- `binutils tic54x-opc.c` : `{ "ld", 1,2,2, 0xEA00, 0xFE00, {OP_k9,OP_DP} }` = **1 mot**,
  charge le **Data Page pointer** (DP = ST0[8:0]) avec un immediat 9 bits.
- Ce n'est donc ni BANZ ni XC. L'ancienne ligne « 0xEA = BANZ (confirmed) » etait FAUSSE
  et contredisait la section BANZ ci-dessous du meme fichier.
- `calypso_c54x.c` est **correct** sur ce point (handler `(op & 0xFE00) == 0xEA00`) : il
  ecrit bien DP dans ST0. Aucune correction de code necessaire.
- Enjeu : DP gouverne tout l'adressage DIRECT. Un DP faux fait tomber chaque acces direct
  sur la mauvaise page — symptome typique : pointeurs aberrants et lectures de zones
  arbitraires.

**How to apply:** Fix XC (0xFD/0xFF), FRET (2 pops), FCALL (2 pushes) in calypso_c54x.c

## BANZ — SPRU172C p.4-16
- Opcode: `0111 1Z0I AAAA AAAA` + 16-bit pmad (2 words)
- ⚠️ RECTIFIE 2026-07-28 par binutils tic54x-opc.c : `banz` = **0x6C00**/0xFF00 et
  `banzd` = **0x6E00**/0xFF00, **2 mots** — ni 0x78xx ni 0xEA. (0x78xx est autre chose.)
- (ancienne note, fausse : « BANZ = 0x78xx, BANZD = 0x7Axx »)
- Sind encodes indirect addressing (which AR and modify mode)
- Execution: if (AR[x] != 0) then pmad→PC, else PC+2→PC
- AR[x] is always decremented (even when condition is false)
- **NOT 0xEA!** 0xEA is something else (needs identification)

## BC — SPRU172C p.4-18
- Opcode: `1111 10Z0 CCCCCCCC` + 16-bit pmad (2 words)  
- BC = 0xF8xx, BCD = 0xFAxx

## CC — SPRU172C p.4-29
- Opcode: `1111 10Z1 CCCCCCCC` + 16-bit pmad (2 words)
- CC = 0xF9xx, CCD = 0xFBxx
