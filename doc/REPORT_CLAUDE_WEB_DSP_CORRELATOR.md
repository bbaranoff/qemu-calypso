# Rapport pour Claude web — Mur corrélateur c54x (MVDM / AR3 / MAC=0)

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> Précisions propres à ce rapport : la prémisse « le corrélateur tourne, lit la vraie I/Q à PC=0xee38 (insn 3.2e9+), il ne reste QUE le mur MAC=0 » ne tient plus. Vérité-terrain : le DSP **déraille** (POST-BOOTSTUB-RET, PC=0x0000) et n'atteint jamais un go-live corrélateur stable ; `d_fb_det` reste 0 sur toute la run. Le vrai verrou amont n'est pas le décode MAC/MVDM mais le **handshake ARM→DSP go-live** qui ne s'arme jamais (ARM n'écrit que 0x0000 aux API 0x0314/0x0318 ; `api_write_cb` déclaré mais jamais assigné). Les numéros de ligne cités ci-dessous ont aussi dérivé (voir corrections inline).

**Contexte** : branche `dsp_revival`, faire tourner le VRAI DSP c54x émulé
(`hw/arm/calypso/calypso_c54x.c`) en mode full (`CALYPSO_MODE=full SHUNT=0
REG_MODE=c54x`). On a remonté toute la chaîne ; il reste UN mur, profond, qui
recoupe un landmine déjà documenté dans le code (MVDM revert). J'ai besoin d'un
œil neuf sur la sortie.

## Ce qui est DÉJÀ résolu cette session (3 racines, sans hack)
1. **`binutils-arm-none-eabi`/`nm`** manquait → run.sh ne nm-dérivait pas les
   adresses firmware. Étendu pour nm-dériver la fenêtre cpu-idle c54x
   (`CALYPSO_IDLE_PC_LO/HI` = `l1a_l23_handler`/`l1a_compl_execute`). → ~~FB→SB
   débloqué (le firmware atteint `read_sb_result`)~~ — FAUX (vérité-terrain 2026-07-01) : `d_fb_det` reste 0 sur toute la run, le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000). La FB n'est jamais détectée.
2. **Offset FN d'époque BSP** : `burst_fn=ts/5000` (osmo-trx) vs
   `current_fn=g_wall_fn` (base 0 boot) → tous les bursts droppés "stale"
   (delivered=0). Fix `calypso_bsp.c bsp_take_for_fn` : auto-mesure
   `dl_fnoff=current_fn-burst_fn` au 1er burst, appliqué au match. → `delivered`
   monte, `stale=0`. ~~**Le BSP livre maintenant les vrais samples I/Q au DSP.**~~ — À NUANCER : même si le BSP présente des samples, le DSP déraille avant tout go-live corrélateur stable et `d_fb_det` reste 0 ; le verrou réel est le handshake ARM→DSP go-live jamais armé (`api_write_cb` jamais câblé), pas la livraison BSP.
3. **Mécanique du TOA garbage PROUVÉE** (sonde SBSLOT-WR) : une boucle
   `PC=0xa1d6 op=0x7193 (STL A,*AR3+)` balaie le bloc résultat db_r
   (`data[0x830/0x83a/0x844/0x84e]`) et y dépose `A.low` partout → le slot SB
   TOA (`a_serv_demod[D_TOA]`) reçoit la valeur courante de l'accumulateur, pas
   un TOA. D'où `a_serv_demod[D_TOA]==a_sch[3]==0x5088`.

## Le mur : l'accumulateur A reste 0 dans le corrélateur FB/SB

Le TOA publié est garbage parce que **A (accumulateur) ne s'accumule jamais**.
Trace LIVE du corrélateur à `PC=0xee38` (sonde IQ-READ, run courant, insn 3.2e9+) :

```
IQ-READ addr=0x2a41 val=0xcf30 PC=0xee38 A=0 B=30470 T=0000 | AR3=2b97 AR4=2b98 AR5=2ce1
IQ-READ addr=0x2aa2 val=0xa5a4 PC=0xee38 A=0 B=30470 T=0000 | AR3=2b97 AR4=2b98 AR5=2ce1
IQ-READ addr=0x2a1c val=0x762c PC=0xee38 A=0 B=30470 T=0000 | AR3=2b97 AR4=2b98 AR5=2ce1
... (addr lus : 0x2ab0 0x2b22 0x2a3f 0x2acb 0x2b23 0x2a8d 0x2b19 0x2a76 ...)
```
`exec_pc=0xee37` exécuté **5270×** (l'instruction MAC dominante) ; `0xee38` = la lecture I/Q.

### Données de trace — CE QUI CONTREDIT le diagnostic "AR3 hors-buffer"
- **AR3=0x2b97, AR4=0x2b98, AR5=0x2ce1 sont CONSTANTS** (jamais incrémentés sur
  des milliers de reads). Ce ne sont PAS les pointeurs de lecture I/Q.
- **L'adresse lue VARIE et est IN-BUFFER** (`0x2a04..0x2b25`, buffer valide
  `[0x2a00..0x2b28)` = 296 mots, `bsp.daram_len=296`), avec de **vraies données**
  (`val=0xcf30/0xa5a4/...` non nuls). Donc le corrélateur lit bien la vraie I/Q.
- **Mais les adresses lues sont DISPERSÉES** (0x2a41, 0x2aa2, 0x2a1c, 0x2ab0,
  0x2b22, 0x2a3f...) — **pas séquentielles**, ce qui est anormal pour un
  corrélateur MAC (qui devrait balayer linéairement/avec stride fixe).
- **A=0, B=30470, T=0000 CONSTANTS** → la MAC ne touche ni A ni B.

→ **Ce n'est donc PAS "AR3 pointe hors buffer → lit des zéros → A=0"** (les
reads sont in-buffer, données réelles). C'est soit (i) la **MAC à 0xee37 qui
n'accumule pas** (mauvais décode de l'opcode MAC/MAS/MPY), soit (ii) le **mode
d'adressage du Smem** à 0xee37 qui produit des adresses **dispersées** (bug de
buffer circulaire BK / indexed AR0 / bit-reverse), si bien que la corrélation
porte sur des échantillons incohérents → somme nulle.

## Le landmine documenté dans le code (à recouper)

`calypso_c54x.c:7464` documente que **MVDM/MVMD (0x72/0x73) RESTENT REVERTÉS**
(fallthrough STL générique 1-mot ; handler gaté `CALYPSO_FIX_MVDM` à 7620-7645) :
- À `PC=0xf564 op=0x7215` = `MVDM data[0x14]→AR5` (cœur de la boucle dispatch FB
  0xf561-0xf588) : décodé **1-mot** alors que MVDM = **2-mots** → l'opérande
  exécutée comme opcode → desync → AR5 (ptr handler tâche) jamais chargé.
- **Appliquer MVDM 2-mots seul RÉGRESSE** : le dispatch avance à 0xee38 mais
  "AR3 y pointe hors buffer (0x2b97>0x2b28) → A=0 → delivered=0 → INT3 ne fire
  plus → deadlock". = "bug compensateur upstream". Critère de ré-application :
  "fixer d'abord le deadlock 0xee38 (AR3 hors-buffer)".
- Histoire liée : `0x86/0x87` était AUSSI mis-décodé MVDM 2-mots ; corrigé
  2026-06-02 en STH 1-mot (`c54x.c:8713-8741`, commentaire FIX 8717, handler 8733-8741) — l'ancien 2-mots "ne touchait
  jamais l'AR du Smem → AR3 figé/0 dans la boucle corrélateur".
- Réf : `doc/REVERT_MVMD_KNOWLEDGE.md`, `project_state_20260602`.

## La QUESTION pour Claude web

Le code suppose que la racine du A=0 à 0xee38 est "AR3 hors-buffer". **Ma trace
montre que c'est faux** : reads in-buffer, données réelles, mais MAC=0 et
adresses **dispersées**. Donc :

1. **Quelle est l'instruction réelle à `0xee37`** (l'opcode MAC dominant) et son
   mode d'adressage Smem ? (à désassembler depuis PROM0 ; `0xee37` ∈
   prog[0x07000+]). Est-ce un `MAC/MAS/MACP` avec adressage circulaire
   (`*ARx+0%`, BK) ou indexé (`*ARx+0`, AR0) que `resolve_smem`
   (`calypso_c54x.c:3728`) résout en adresses dispersées ?
2. **Pourquoi les adresses lues sont-elles dispersées** (0x2a41,0x2aa2,0x2a1c…)
   et non séquentielles ? → buffer circulaire BK mal géré ? AR0/index faux ?
3. **Pourquoi A reste 0** alors que `val` lus sont non-nuls ? La MAC
   multiplie-t-elle par 0 (coefficient T=0000 ? un opérande pointé par
   AR3=0x2b97 hors-buffer = 0 ?) Note : **T=0000 constant** — si la MAC est
   `A += T * data[Smem]` et T=0, A reste 0. **D'où vient T, et pourquoi 0 ?**
   (AR3=0x2b97 hors-buffer pourrait être le pointeur du COEFFICIENT chargé dans
   T → coeff=0 → MAC=0. Ça réconcilierait "AR3 hors-buffer" ET "reads in-buffer":
   AR3 pointe le coeff hors-buffer (=0), pendant que l'I/Q est lue in-buffer.)
4. Comment fixer **ensemble** : le MVDM 2-mots (0x72/0x73, pour le dispatch
   0xf564→AR5) ET le setup du pointeur coeff/AR3 (pour que T≠0), sans la
   régression "compensateur" — silicium-fidèle, sans toucher la ROM ?

**Hypothèse de tête (à valider)** : `AR3=0x2b97` est le pointeur de COEFFICIENT
(pas d'I/Q), hors buffer → coeff=0 → `T=0` → MAC `A+=T*iq=0`. Le vrai bug est le
**setup d'AR3 (chargé par un MVDM 2-mots mal décodé)** qui devrait pointer une
table de coeffs valide. Fixer le décode MVDM 2-mots ferait pointer AR3 au bon
endroit → coeff≠0 → MAC accumule → TOA réel. Le "reverted MVDM" cassait
justement ce chargement d'AR.

## Fichiers / ancres
- `hw/arm/calypso/calypso_c54x.c` : MVDM revert 7464 (+ handler gaté 7620-7645) ; STH 0x86/0x87 8713-8741 ;
  `resolve_smem` 3728 (modes d'adressage) ; `c54x_exec_one` 4052 ;
  corrélateur PC=0xee37/0xee38 (PROM0).
- `hw/arm/calypso/calypso_bsp.c` : buffer I/Q `daram_addr=0x2a00 len=296`.
- Sondes read-only en place : IQ-READ (c54x:~1599), SBSLOT-WR (c54x, data_write),
  AR3-PRELOAD (c54x, ar_write_track), FBSBRES-RD/FBDET-RD/POST-WATCH (trx).
- `doc/REVERT_MVMD_KNOWLEDGE.md`, `project_state_20260602`.
