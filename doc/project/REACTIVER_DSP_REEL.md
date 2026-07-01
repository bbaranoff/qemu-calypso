# Réactiver le vrai DSP C54x (calypso_c54x.c) — au lieu du shunt

> But : faire tourner le **vrai DSP Calypso émulé** (`calypso_c54x.c` exécutant la ROM TI),
> au lieu du shunt canned (`CALYPSO_DSP_SHUNT=1`). Intérêt principal : le **vocoder GSM-FR/EFR
> natif** et tout le traitement signal L1 (FCCH/SCH, Viterbi, deinterleave, burst decode, TCH)
> sont faits par la ROM réelle — pas de codage/vocodage à réimplémenter ailleurs.
>
> Document de cadrage uniquement. **Aucune modification de code n'accompagne ce md.**

---

## TL;DR

- Le **toggle existe** : `CALYPSO_DSP_SHUNT=0` → le C54x émulé tourne (au lieu du shunt). `=1` → shunt canned.
- La **vraie ROM DSP est présente** : `/opt/GSM/calypso_dsp.txt` (→ `qemu-src/calypso_dsp.txt`), dump réel
  « OsmocomBB Compal DSP Dumper », cDSP ID `0x0128`, 8222 lignes / 722 Ko, sections
  `Registers / PROM0..3 / DROM / PDROM`. Elle contient le L1 DSP **et le vocoder**.
- L'**émulateur C54x existe** (`calypso_c54x.c`, ~12 442 lignes : exécute les opcodes, charge la ROM,
  Viterbi/deinterleave/burst decode) mais il est **incomplet** — son bring-up est en cours
  (docs `STEP1_PROM_LOADER.md`, `STEP2_BC_CONDS.md`, `STEP3_F2_F5_STUBS.md`). C'est précisément
  pour contourner ce DSP pas-encore-fini que le **shunt** a été créé (canned FB/SB → débloque la
  registration sans DSP).
- Donc « remettre le vrai DSP » = **finir le bring-up du C54x** jusqu'à exécuter la ROM de bout en
  bout, + câbler l'I/Q réel (AFC) en entrée, + (pour la voix) un **codec audio ABB virtuel** en sortie.

---

## 0. Posture, ordre de difficulté et motivation (révisé)

Trois corrections au cadrage initial — elles changent l'ordre des priorités.

**(a) La vraie motivation n'est PAS le vocoder — c'est d'exécuter la ROM TI de bout en bout.**
Pour *entendre/parler*, le vrai DSP n'apporte **rien d'audible** : gapk (approche a′) produit du
GSM-FR acoustiquement **identique** (même codec, même spec) pour une fraction de l'effort.
L'« authenticité » du vocoder TI est inaudible. Donc pour la **voix utile, a′ gagne sans débat**.
La valeur unique du vrai DSP est **différente** : c'est le seul chemin qui **prouve qu'on exécute la
vraie ROM DSP propriétaire en émulation** — artefact reproductible et citable, que personne n'a fait.
Cet artefact se débloque à **V1** (la ROM tourne), **pas à V4** (audio). Conséquence : si on fait le vrai
DSP, on le fait **pour V1** ; l'audio ABB devient du **polish optionnel** de démo, pas le driver.

**(b) V1 (finir le C54x) est le vrai mur — mais c'est un mur BORNÉ, et il est séparable du RF.**
Le wedge (cf §3-V1) est un bug d'**émulateur CPU**, pas un problème I/Q : on le débugge en nourrissant
le DSP avec un buffer **statique / des zéros** — la ROM n'a besoin d'aucun échantillon crédible pour
s'exécuter jusqu'à la frame loop. Débuggage **déterministe et borné** (on a la ROM, on a une référence
de ce que les registres doivent valoir). À ne pas amalgamer avec « résoudre l'AFC ».

**(c) L'AFC est largement sidesteppable — ce n'est probablement pas le gros morceau.**
En émulation on tient les deux bouts (générateur du signal réseau *et* notion de fréquence du DSP). On
livre un I/Q à offset **~nul** = le cas *facile* du corrélateur (verrouillage trivial), sans modéliser la
boucle fermée du DAC. Le risque résiduel n'est donc **pas mathématique** mais une **state-machine** :
la ROM pourrait *gater* sur un flag « AFC settled » qui ne se lève jamais s'il n'y a rien à régler.
C'est un bug d'état **débuggable en lisant la ROM** (caveat : si ce flag dépend d'un registre DAC/ABB
côté analogique, le périmètre déborde un peu sur le modèle TWL3025 — voir V4). Borné, pas ouvert.

**Ordre honnête qui en découle :** d'abord **prouver que la ROM s'exécute sur entrée statique** (V1),
*puis seulement* se demander si le vrai I/Q converge (V2/V3). V1 et V2/V3 ont des profils de risque
**opposés** : V1 = déterministe/borné ; V2/V3 = ouvert. Le découpage *scaffolding réutilisable (§2) /
réalisme RF manquant (§3)* reste le bon axe.

---

## 1. État actuel

| | Shunt (`CALYPSO_DSP_SHUNT=1`, actuel) | DSP réel (`CALYPSO_DSP_SHUNT=0`, cible) |
|---|---|---|
| C54x | **skippé** (`calypso_dsp_shunt.c:4` « c54x emulator is skipped entirely, no opcode ») | exécute la ROM TI |
| FB/SB | synthétisés/canned dans la NDB | calculés par le corrélateur DSP réel |
| Décodage canal | absent côté DSP (gr-gsm/sideband côté signalisation) | fait par la ROM (Viterbi, deinterleave) |
| Vocoder | **absent** | **présent** (dans la ROM) |
| Registration | OK (c'est le mode qui marche) | dépend du bring-up C54x |

Le shunt est un **mock** : il fabrique les résultats DSP attendus par l'ARM (a_cd[], a_sch[], a_pm[])
pour faire avancer la pile L1→L2→L3 sans DSP. Le DSP réel est l'inverse : on laisse la ROM produire
ces résultats.

---

## 2. Ce qui est DÉJÀ en place (les briques réutilisables)

1. **Émulateur C54x** — `hw/arm/calypso/calypso_c54x.c`
   - Cœur C54x : fetch/exec opcodes (`prog_fetch`, `c54x_exec_one`, `c54x_run`), accumulateurs 40 bits,
     OVM, MMR, AR, pile. En-tête : « Minimal C54x core: enough to run the Calypso DSP ROM for GSM
     signal processing (Viterbi, deinterleaving, burst decode) ».
   - Chargement ROM : `c54x_load_rom` (cf table dans `docs/STEP1_PROM_LOADER.md`).

2. **La vraie ROM DSP** — `calypso_dsp.txt` (symlink `/opt/GSM/` → `qemu-src/`)
   - Dump réel du combiné (Device ID `0xb4fb`, cDSP `0x0128`). Sections :
     `Registers (0x00000-0x0005F)`, `DROM (0x09000-0x0DFFF)`, `PDROM (0x0E000-0x0FFFF)`,
     `PROM0 (0x07000-)`, `PROM1 (0x18000-)`, `PROM2 (0x28000-)`, `PROM3 (0x38000-)`.
   - Conversion en sections .bin : `dsp_txt2bin.py` (`qemu-src/dsp_txt2bin.py`).
     `run.sh` auto-split déjà : `CALYPSO_DSP_ROM_TXT` (défaut `/opt/GSM/calypso_dsp.txt`) →
     `*.{PROM0..3,DROM,PDROM}.bin` (run.sh ~1675-1709).

3. **Câblage QEMU des sections ROM** — `hw/arm/calypso/calypso_mb.c:47-52, 286-328`
   - Props machine : `-M calypso,dsp-prom0=<p>,dsp-prom1=<p>,dsp-prom2=<p>,dsp-prom3=<p>,dsp-drom=<p>,dsp-pdrom=<p>`
     (et `dsp-blob=<p>` pour une fixture DARAM seule). `calypso_trx_set_section_paths(...)` les pousse au C54x.

4. **Toggle & sélection de mode** — `run.sh`
   - `CALYPSO_DSP_SHUNT=0|1` (run.sh:85). Défauts par mode (run.sh ~1122-1212) : certains modes posent
     déjà `:=0` (DSP réel), d'autres `:=1` (shunt). Label `_dsp_label` : `(c54x emule)` vs `(canned FB+SB)`.

5. **Pont I/Q vers le DSP** — `hw/arm/calypso/calypso_bsp.c`
   - `c54x_bsp_load(bsp.dsp, samples, ns)` (calypso_bsp.c:949, 1148) : le BSP peut DMA-feeder
     des échantillons I/Q dans la DARAM du C54x (entrée du corrélateur). Transport TRXDv0:6702
     channel-agnostic.

6. **Horloge maître** — `calypso_tint0.c` + `calypso_trx.c` (TINT0 4.615 ms, frame-IRQ). Le DSP est cadencé
   par la même base de temps que l'ARM (cf fix horloge `GSM_TDMA_NS`/hwtimer déjà appliqué).

7. **Docs de bring-up déjà commencées** — `qemu-src/docs/STEP1_PROM_LOADER.md`, `STEP2_BC_CONDS.md`,
   `STEP3_F2_F5_STUBS.md`. Elles tracent les blocages connus de l'émulateur (à reprendre).

---

## 3. Ce qui MANQUE / les verrous à lever

### V1 — L'émulateur C54x ne va pas au bout de la ROM (bug-fixing) — **mur borné, séparable du RF**
Le bring-up est inachevé. Symptômes documentés :
- **STEP1** (`STEP1_PROM_LOADER.md`) : à PC=0x7629 (FB-det), un `RPT #12 ; READA` copie 13 mots de
  `prog[0x5717..]` vers `data[0x0000..]` = **les MMR** (IMR/IFR/AR). `prog[0x5717]` est **en dessous**
  de PROM0 (0x7000) → lit `0x0000` → IMR=0, IFR=0 → **toutes les IRQ masquées → DSP wedge**.
- **STEP2** (`STEP2_BC_CONDS.md`) : conditions de branchement (BC) à valider.
- **STEP3** (`STEP3_F2_F5_STUBS.md`) : stubs F2/F5 à compléter.

**Analyse 0x5717 (vérifiée contre le dump, 2025) — adversarial sur l'hypothèse A/B.**
Le dump `calypso_dsp.txt` ne contient en espace **programme** que `PROM0 [07000-0dfff]`, `PROM1 [18000-]`,
`PROM2 [28000-]`, `PROM3 [38000-]` ; `grep 5717` = vide, et tout `prog[] < 0x7000` est **absent du dump**.
Conséquences :
- ❌ **(A) « charger la section prog manquante à 0x5717 » est mort** : un dump de **ROM** ne contient pas
  cette zone car ce n'est **pas de la ROM** — `prog[] < 0x7000` est de la **RAM programme** (la ROM
  démarre à 0x7000). Il n'y a aucune section à charger là.
- ⚠️ Le dump **ne sépare pas (B) de (C)** : « absent » est l'état attendu pour les **deux** :
  - **(B)** adresse source corrompue : un opcode amont a mal calculé l'accumulateur A qui adresse le READA
    → le vrai bug est *avant* 0x7629.
  - **(C)** RAM-programme **légitime jamais peuplée** : 0x5717 est l'emplacement RAM où le vrai chip a copié
    une table d'init (IMR/IFR/AR) via une **copie ROM→RAM au boot DSP** ; l'émulateur n'a jamais
    exécuté/honoré cette copie (chemin d'écriture `prog[]`, ou boucle de boot sautée).
- 👉 **(C) est plus probable que (B)** : sur le silicium réel cette routine tourne et marche, donc à cet
  instant `prog[0x5717]` y contient des données valides ; comme ce n'est pas de la ROM, c'est de la RAM
  remplie au boot. L'adresse est donc *probablement correcte* et le bug est une **copie d'init manquée**.
- 🔬 **Expérience décisive (≠ le grep du dump, qui est non concluant B/C)** : tracer si/où
  `prog[0x5717..0x5723]` est **écrit** avant PC=0x7629, et depuis quelle source. Jamais écrit → (C) confirmé.
  Écrit à une mauvaise adresse → (B). Caveat (ABB) : si la valeur IMR « correcte » dépend d'un registre DAC/
  analogique non modélisé, le périmètre déborde un peu (cf V2/V4) — mais le débuggage reste déterministe.

→ Il faut **finir ces étapes** (opcodes/conditions manquants, mapping mémoire, **copie ROM→RAM de boot**,
  MMR/IFR/IMR préservés). **Point clé : ce débuggage se fait sur entrée STATIQUE/zéros** — le DSP n'a besoin
  d'aucun I/Q crédible pour exécuter la ROM jusqu'à la frame loop. C'est **déterministe et borné** (on a la
  ROM + une référence des valeurs registres attendues), à ne PAS mélanger avec l'AFC (V2).

### V2 — AFC : **sidesteppable** par pré-lock — le risque est une state-machine, pas du signal
`calypso_trx.c:104-110` : « fixed cos/sin LUT (no AFC DAC feedback in QEMU) → the DSP correlator cannot
converge across iterations ». C'est la raison invoquée pour synthétiser FB/SB — **mais le risque est mal
nommé.** En émulation on tient les deux bouts (générateur réseau *et* notion de fréquence du DSP) : on
**livre un I/Q à offset ~nul** (pré-lock côté BSP) = le cas *facile* du corrélateur (résidu zéro →
verrouillage trivial), sans modéliser la boucle fermée du DAC. Donc :
- ⚠️ Le risque réel n'est **pas mathématique** (convergence) mais une **state-machine ROM** : la ROM peut
  *gater* sur un flag « AFC settled » qui ne se lèvera jamais s'il n'y a rien à régler. Bug d'état **lisible
  dans la ROM**, **borné** — pas un chantier signal ouvert.
- Caveat : si ce flag dépend d'un registre DAC/ABB analogique non modélisé, le périmètre touche le modèle
  TWL3025 (cf V4) — mais ça reste un **handshake de registre**, pas du DSP signal.
- Option lourde à **éviter d'abord** : modéliser le vrai **feedback AFC** (le DSP pilote un DAC de fréquence
  reflété dans l'I/Q). À ne faire que si le pré-lock ne suffit pas.

### V3 — Le BSP doit livrer du vrai I/Q (pas des hard-bits canned)
En mode signalisation actuel, la chaîne convertit des soft-bits en I/Q « propre » via LUT. Pour que le DSP
démodule réellement, il faut l'I/Q **fidèle** des bursts (DL réseau → osmo-trx → qemu_wrap → BSP →
`c54x_bsp_load` → DARAM), au bon format/échelle attendu par la ROM.

### V4 — Sortie audio : il faut un codec ABB (TWL3025) virtuel
Le vrai DSP **décode ET vocode** : il produit du **PCM**, pas une trame FR. Sur silicium, ce PCM va au
codec audio ABB (TWL3025) → haut-parleur. En QEMU il n'y a **aucune sortie audio ABB**. Donc pour
entendre/parler avec le DSP réel il faut :
- un **modèle TWL3025/ABB audio** (ou un point de tap PCM dans le DSP) **branché à PulseAudio**
  (sink pour écouter, source micro pour parler).
- C'est différent de l'approche a′ (où le firmware sort une **trame FR** vers L23/gapk). Avec le DSP réel,
  l'audio est du **PCM** → pont ABB requis.

### V5 — Performance / temps réel
ARM émulé + C54x émulé (12k+ lignes, par-opcode) sous `-icount` = lourd. Tenir la cadence TDMA
(4.615 ms/trame) avec les deux cœurs émulés est un risque (cf déjà les `LOST`/dérive FN côté ARM).
Le shunt évitait tout ce coût.

### V6 — Légalité/intégrité de la ROM
`calypso_dsp.txt` est un **dump propriétaire TI** (présent ici car déjà dumpé du combiné). À garder tel
quel ; ne pas redistribuer hors contexte. Vérifier qu'il est complet (toutes sections, pas de trous qui
causent V1).

---

## 4. Étapes pour réactiver (ordonné, sans code)

1. **Basculer le toggle** : lancer avec `CALYPSO_DSP_SHUNT=0` (et fournir la ROM, voir §2.2/2.3).
   Vérifier dans les logs `(c54x emule)` et l'absence de `[dsp-shunt] active`.
2. **Charger la ROM** : confirmer l'auto-split `CALYPSO_DSP_ROM_TXT=/opt/GSM/calypso_dsp.txt` →
   `*.PROM{0..3}.bin/.DROM.bin/.PDROM.bin`, et que `calypso_mb.c` les passe via `-M calypso,dsp-prom*=…`.
3. **Faire booter le DSP sans wedge — sur entrée STATIQUE/zéros, AVANT tout I/Q réel** (c'est le vrai mur,
   borné). Test 0x5717 d'abord : tracer si `prog[0x5717..0x5723]` est écrit avant PC=0x7629 (jamais → copie
   ROM→RAM de boot manquante = (C) ; faux → adresse corrompue = (B), cf §3-V1). Puis `STEP1/2/3`
   (mapping PROM, conditions BC, stubs F2/F5, MMR/IFR/IMR préservés). Critère : le DSP atteint la frame loop
   et garde IMR/IFR cohérents sur plusieurs trames, **sans aucun échantillon crédible en entrée**.
4. **Alimenter l'I/Q réel + AFC** (V2/V3) : câbler le DL I/Q fidèle au BSP→DARAM et résoudre la
   convergence du corrélateur (feedback AFC modélisé OU I/Q pré-calé). Critère : **FB/SB réels** détectés
   par le DSP (plus de canned), puis SCH/BCCH/SDCCH décodés par la ROM.
5. **Valider la signalisation bout-en-bout** par le DSP réel (registration LU) AVANT de viser la voix.
   C'est le palier qui prouve que le DSP tourne pour de vrai.
6. **(POLISH OPTIONNEL — hors chemin critique de l'artefact ; pour la voix réelle préférer a′) Activer le
   TCH/vocoder** : une fois un canal de trafic assigné, la ROM décode le TCH et **vocode**.
   Récupérer le **PCM** produit par le DSP (tap dans le modèle C54x / la zone API audio) et le brancher au
   **pont ABB virtuel** (V4) → PulseAudio (écouter). Symétriquement, injecter le PCM micro depuis
   PulseAudio → ABB → DSP (parler).
7. **Régler le temps réel** (V5) : profiler, ajuster `icount`/threading pour tenir la trame avec les deux
   cœurs émulés.

---

## 5. Spécifique « voix » : DSP réel vs approche a′

| | DSP réel (ce doc) | a′ (chemin actuellement choisi) |
|---|---|---|
| Codage canal TCH | ROM DSP | `gsm0503_tch_fr` dans le shunt/qemu_wrap |
| Vocoder FR↔PCM | **ROM DSP** | **gapk** dans l'app L23 |
| Sortie audio | **PCM → ABB virtuel → PulseAudio** | trame FR → L1CTL_TRAFFIC → L23/gapk → ALSA → PulseAudio |
| Ampleur | très grosse (finir C54x + AFC + ABB) | modérée (codage canal + lancer L23 gapk) |
| Fidélité | authentique (vrai vocoder TI) | fonctionnelle (vocoder gapk équivalent FR) |

→ **Pour la voix : a′ gagne sans débat.** Le FR de gapk est acoustiquement **identique** à celui du vocoder
TI — la différence est strictement **inaudible**. Le « très grosse » de la colonne DSP réel serait donc
dépensé pour un output indistinguable. **Ne pas faire le vrai DSP pour la voix.**

→ La valeur du vrai DSP est **ailleurs** (cf §0-a) : c'est le seul chemin qui **prouve l'exécution de la
ROM TI propriétaire de bout en bout** — l'artefact citable. Et il se débloque à **V1** (la ROM tourne),
**pas à V4** (audio). Donc : si on fait le vrai DSP, on le fait **pour V1** ; l'audio ABB (V4) n'est que du
**polish optionnel de démo**, sorti du chemin critique.

---

## 6. Checklist de réactivation

- [ ] `CALYPSO_DSP_SHUNT=0` pris en compte (log `(c54x emule)`).
- [ ] ROM splittée et chargée (PROM0..3/DROM/PDROM via `-M calypso,dsp-*`).
- [ ] DSP boote sans wedge (STEP1 READA/MMR corrigé ; STEP2/3 finis).
- [ ] I/Q DL réel livré au BSP ; corrélateur FB/SB converge (AFC ou pré-calé).
- [ ] FB/SB/SCH/BCCH/SDCCH décodés par la ROM (plus de canned).
- [ ] Registration (LU) via DSP réel.
- [ ] TCH assigné → ROM décode + vocode → PCM disponible.
- [ ] Pont ABB/TWL3025 virtuel ↔ PulseAudio (sink + source).
- [ ] Temps réel TDMA tenu (icount/threading).

---

## 7. Références fichiers

- Toggle / skip : `hw/arm/calypso/calypso_dsp_shunt.c:4, :121, :1485-1535` ; `run.sh:85, ~1122-1212, :835`.
- Émulateur C54x : `hw/arm/calypso/calypso_c54x.c` (cœur, `c54x_run`, `c54x_load_rom`, `c54x_bsp_load`).
- Chargement ROM (props) : `hw/arm/calypso/calypso_mb.c:47-52, :241, :286-328`.
- ROM réelle : `/opt/GSM/calypso_dsp.txt` (→ `qemu-src/calypso_dsp.txt`) ; outil `qemu-src/dsp_txt2bin.py` ;
  auto-split `run.sh:~1672-1709` (`CALYPSO_DSP_ROM_TXT`, `CALYPSO_DSP_BLOB`).
- I/Q vers DSP : `hw/arm/calypso/calypso_bsp.c:949, :1148`.
- Convergence/AFC (pourquoi canned) : `hw/arm/calypso/calypso_trx.c:104-110`.
- Bring-up en cours : `qemu-src/docs/STEP1_PROM_LOADER.md`, `STEP2_BC_CONDS.md`, `STEP3_F2_F5_STUBS.md`.
- Contrat voix côté firmware (pour comprendre la sortie PCM vs trame) :
  `osmocom-bb/src/target/firmware/layer1/prim_tch.c` ; `include/calypso/dsp_api.h` (a_dd_0/a_du_0/d_tch_mode).
