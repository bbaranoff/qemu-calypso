# doc_master — Index maître de la doc Calypso QEMU (DSP revival / FBSB)

> Branche `dsp_revival`. But du projet : faire tourner le **VRAI** DSP TMS320C54x
> émulé pour qu'il détecte FB/FCCH et écrive `d_fb_det != 0`. Règle #1 : on répare
> **le câblage de l'émulateur uniquement**, aucun hack, aucun poke d'état DSP interne.

## État en un coup d'oeil (synthèse 2026-07-01)

- **Blocker terminal, unique et propre :** `IMR = 0x0000` pour TOUT le run. L'IMR est
  effacé au boot par la ROM (`STM #0,IMR` @ PC=0xb37e, `calypso_c54x.c` BOOT-MMR-WR #7,
  insn=1047) et **jamais ré-armé**. Conséquence en chaîne : chaque frame-IRQ livrée est
  masquée → 0 ISR vectorisée → `INTM` ne se remet jamais à 0 → le scheduler/corrélateur
  par-frame ne tourne jamais → `d_fb_det` reste 0. (0 lignes `INTM=0`, 0 `RETE`, `d_fb_det(0x01F0)=0x0000`.)
- **La pièce manquante (fix le plus causalement proche) :** la ligne frame TPU est livrée
  au **mauvais vecteur** et en **wake-only**. Elle arrive en `vec19/bit3` (`C54X_INT_FRAME_VEC=19`,
  `calypso_c54x.h:126-127`) via `calypso_trx.c:1786`, alors que le scheduler FB firmware
  attend `vec28/bit12` (→ CALL 0xa4e4 → corrélateur → `d_fb_det`). Le seul code qui retargète
  ET force-vectorise malgré IMR=0 est le bloc `CALYPSO_DSP_FRAME_VEC28` à
  `calypso_c54x.c:13547-13565` — **OFF par défaut** (`VEC28-FORCE count=0`), bien que sa garde
  `d_dsp_page bit1 (B_GSM_TASK)` soit satisfaite dès fn≈1206.
- **Domaine du défaut :** signalisation WAKE/IRQ (interrupt-enable / INTM-IMR-vecteur).
  PAS le mailbox de contrôle (prouvé fonctionnel, `delivered=125093` I/Q dans DARAM 0x2a00),
  PAS le chemin I/Q (write-side vivant ; read-side mort = symptôme aval du DSP idle).
- **Pas besoin d'un nouveau pipe `calypso_arm2dsp.c` :** le mailbox ARM↔DSP existe et transporte
  déjà chaque champ dans les deux sens. Un refactor (`calypso_dspmb.c`) est **orthogonal** —
  hygiène, pas le fix.

---

## 1) Entrée

| Doc | Accroche | Fraîcheur |
|---|---|---|
| `doc_master.md` (ce fichier) | Index maître : navigation de tout le jeu de doc + synthèse. | 2026-07-01 |
| [README](README.md) | Point d'entrée du sous-système `hw/arm/calypso` : but, archi, fichiers, build/run, état. | 2026-07-01 |
| [schematics](schematics.md) | Diagrammes mermaid/ASCII : pipeline, orchestration (gaps annotés), memory map, boot FSM, corrélateur. | 2026-07-01 |
| [CALYPSO_HW](CALYPSO_HW.md) | Référence matérielle Calypso (chip overview, blocs, MMIO). Point d'entrée HW. | 20/06 |
| [datasheets/README](datasheets/README.md) | Index des datasheets TI/FreeCalypso collectés comme ground-truth C54x. | 28/04 |

> Entrée conseillée : `README.md` (vue d'ensemble) → `schematics.md` (diagrammes) →
> `CALYPSO_HW.md` (détail HW), `datasheets/README.md` (ground-truth C54x).

## 2) État courant & TODO

| Doc | Accroche | Fraîcheur |
|---|---|---|
| [TODO](TODO.md) | Pole position / reprise à chaud, acquis gravés (commit `4504dec`). | **2026-07-01** |
| [STATUS](archive/STATUS.md) | Dernier root-cause résolu + blocker courant (frontmatter "état courant"). | 30/05 — **périmé par les rapports Jul-1** (voir §4) : dit "IMR arme bit12", faux ce run (IMR=0x0000 tout le run). |
| [REPORT_DSP_CORRELATOR_RO](REPORT_DSP_CORRELATOR_RO.md) | Audit READ-ONLY du mur `d_fb_det=0` (aucune édition/rebuild). Diagnostic à jour. | **2026-07-01** |
| [REPORT_DSP_C54X_CORRELATION](REPORT_DSP_C54X_CORRELATION.md) | Corrélation firmware `calypso_dsp.txt` ↔ décodeur `calypso_c54x.c` + tables de decode. | **2026-07-01** |
| [REPORT_CLAUDE_WEB_IQ_CABLAGE](REPORT_CLAUDE_WEB_IQ_CABLAGE.md) | Diag câblage I/Q + mur boot ; "Break A" (scheduler vec28 jamais lancé), "Break B" (DARAM 0x2a00). | 25/06, maj **Jul-1** |
| [REPORT_CLAUDE_WEB_DSP_REVIVAL](REPORT_CLAUDE_WEB_DSP_REVIVAL.md) | Rapport de review "DSP Revival" (objectif branche `dsp_revival`). | 22/06, maj **Jul-1** |
| [REPORT_CLAUDE_WEB_DSP_CORRELATOR](REPORT_CLAUDE_WEB_DSP_CORRELATOR.md) | Mur corrélateur c54x (MVDM / AR3 / MAC=0) — rapport pour review web. | **2026-07-01** |

## 3) Séquences boot / FBSB

| Doc | Accroche | Fraîcheur |
|---|---|---|
| [BOOT_TO_FBSB_SEQUENCE](archive/BOOT_TO_FBSB_SEQUENCE.md) | Flux de contrôle bout-en-bout : reset silicium → FB lock → SB lock. | 20/06 |
| [FBSB_SEQUENCE_TRACE](archive/FBSB_SEQUENCE_TRACE.md) | Trace du FBSB_REQ au FBSB_CONF (chaîne osmocom-bb ARM + host model QEMU). | 29/05 |

## 4) Corrélateur & décodeur

| Doc | Accroche | Fraîcheur |
|---|---|---|
| [FB_CORRELATOR_PIPELINE](FB_CORRELATOR_PIPELINE.md) | Pipeline corrélateur FB/FCCH (composants, interactions, code C). | **2026-07-01** |
| [C54X_DECODER_AUDIT](C54X_DECODER_AUDIT.md) | Audit du décodeur `calypso_c54x.c` (dispatch IRQ, IPTR/vecteurs, RET-stubs). | **2026-07-01** |
| [DSP_ROM_MAP](DSP_ROM_MAP.md) | Carte de la ROM DSP (sections du dump, adresses des handlers). | 20/06 |
| [SP_CATASTROPHE_70c4_SEQUENCE](archive/SP_CATASTROPHE_70c4_SEQUENCE.md) | Séquence "mort-vivant" : SP-catastrophe → self-CALA 0x70c3 → PMST/d_fb_det=0x70c4. | 20/06 |

## 5) Références HW / registres / opcodes / datasheets

| Doc | Accroche | Fraîcheur |
|---|---|---|
| [CALYPSO_HW](CALYPSO_HW.md) | Référence matérielle Calypso (chip, blocs, MMIO). | 20/06 |
| [REGISTERS_REVIEW](REGISTERS_REVIEW.md) | Review de l'image registres DSP (`calypso_dsp.Registers.bin`) pour refactor. | 31/05 |
| [C54X_INSTRUCTIONS](C54X_INSTRUCTIONS.md) | Encodages d'instructions clés vérifiés depuis SPRU172C, corrige des bugs émulateur. | 20/06 |
| [SERCOMM_GATE_ARCHITECTURE](SERCOMM_GATE_ARCHITECTURE.md) | Architecture de la gate Sercomm (vrai HW Calypso vs modèle QEMU). | 20/06 |
| [opcodes/tic54x_hi8_map](opcodes/tic54x_hi8_map.md) | Map opcode C54x (hi8 → mnémonique), source binutils `tic54x-opc.c`. | 20/06 |
| [opcodes/0x68_0x6F](opcodes/0x68_0x6F.md) | Spec famille 0x68xx-0x6Fxx (vérifiée binutils 2.21.1 + SPRU172C). | 20/06 |
| [opcodes/0xF3](opcodes/0xF3.md) | Spec opcode 0xF3xx (famille F0xx), vérifiée binutils 2.21.1 + SPRU172C. | 20/06 |
| [datasheets/README](datasheets/README.md) | Index des datasheets TI/FreeCalypso (ground-truth C54x). | 28/04 |

> Datasheets binaires associés (non-`.md`, mêmes dossier) : > `datasheets/TI_SPRU131G_*`, `TI_SPRU172C_*`, `TI_SPRA036_*`, `TI_SPRA618_*`,
> `TI_SPRU288_*`, `Calypso_overview_FreeCalypso.pdf`, et les dumps ROM
> `DSP-ROM-dump.txt` / `dsp-rom-3311|3416|3606-dump.txt`.

## 6) Sessions (chronologique)

| Doc | Accroche | Date |
|---|---|---|
| [SESSION_20260403](archive/SESSION_20260403.md) | Fix `LD #k9,DP` (0xEA, décodé BANZ à tort) + boot TDMA. | 03/04 |
| [SESSION_20260405_NIGHT4](archive/SESSION_20260405_NIGHT4.md) | Audit opcodes C54x (nuit 4). | 05/04 |
| [SESSION_20260429](archive/SESSION_20260429.md) | Chemin firmware DSP débloqué, blocker final `INTM=1`. | 29/04 |
| [REVERT_MVMD_KNOWLEDGE](archive/REVERT_MVMD_KNOWLEDGE.md) | Knowledge du revert MVMD/MVDM (revert pragmatique, à ne pas perdre). | 15/05 |
| [SESSION_2026-05-29](archive/SESSION_2026-05-29.md) | Status de session (TL;DR). | 29/05 |

## 7) Projet (dossier [`project/`](project/))

Docs niveau projet (au-delà du sous-système émulateur) : contexte, flux e2e,
threading/MTTCG, statut, historique de patches.

| Doc | Accroche |
|---|---|
| [project/README](project/README.md) | README projet complet (setup, pipeline, arbo). |
| [project/CLAUDE](project/CLAUDE.md) | Contexte agent + règles (sync miroirs, rule#1). |
| [project/ARCHITECTURE](project/ARCHITECTURE.md) | Architecture d'ensemble du pipeline. |
| [project/FLOW](project/FLOW.md) · [CALL_FLOW](project/CALL_FLOW.md) · [FBSB_FLOW](project/FBSB_FLOW.md) · [L1CTL_SOCK_FLOW](project/L1CTL_SOCK_FLOW.md) | Flux e2e (général, appels, FBSB, socket L1CTL). |
| [project/BRIDGES](project/BRIDGES.md) | Ponts radio / décodage. |
| [project/THREADING_TODO](project/THREADING_TODO.md) · [MTTCG_AUDIT](project/MTTCG_AUDIT.md) · [TINT0_IMPLEMENTATION](project/TINT0_IMPLEMENTATION.md) | Threading MTTCG, TINT0. |
| [project/PROJECT_STATUS](project/PROJECT_STATUS.md) · [state](project/state.md) · [test_results](project/test_results.md) | Statut, état, résultats de tests. |
| [project/REACTIVER_DSP_REEL](project/REACTIVER_DSP_REEL.md) · [STEP1](project/STEP1_PROM_LOADER.md)/[STEP2](project/STEP2_BC_CONDS.md)/[STEP3](project/STEP3_F2_F5_STUBS.md) | Réactivation DSP réel, étapes. |
| [project/BUGS_AND_FIXES](project/BUGS_AND_FIXES.md) · [AUDIT_DECODER_20260508](project/AUDIT_DECODER_20260508.md) · [patches](project/patches.md) · [c_patches](project/c_patches.md) · [REFACTORING](project/REFACTORING.md) · [REPORT_CLAUDE_WEB_PIPELINE](project/REPORT_CLAUDE_WEB_PIPELINE.md) · [TODO_2026-05-25_NIGHT](project/TODO_2026-05-25_NIGHT.md) | Bugs/fixes, audits, patches, refactor, rapport pipeline, TODO nuit. |

---

### Rappel des leviers de fix (câblage émulateur, cités)

1. `calypso_c54x.c:13547-13565` — remap `CALYPSO_DSP_FRAME_VEC28` + `frame_force` (le levier honnête, OFF par défaut). Un-gater / mapper en dur `vec19/bit3 → vec28/bit12`.
2. `calypso_trx.c:1786` — site de tir de la ligne frame (`c54x_interrupt_ex(dsp, 19, 3)`).
3. Alternative plus fidèle : handshake ARM→DSP go-live jamais asserté (`calypso_trx.c:3053-3070`, offsets 0x0314/0x0318 écrits val=0x0000) ; suspect = notify `api_write_cb` NULL (`calypso_c54x.c:3355`).
4. Vérifier que la fenêtre DARAM lue par le corrélateur (0x2a00) == `bsp.daram_addr` écrite (`calypso_bsp.c:1259`, défaut `calypso_bsp.c:795`).

> Sources vérifiées ce run (CONFIRMED) : `qemu.log` (IMR=0x0000, 0 RETE, INT3-RATE idle=1,
> delivered=125093, d_fb_det=0x0000) ; corrections à l'audit : IMR n'est PAS 0x3000 en run,
> IPTR est relocalisé (`PMST-WR #4 val=0x0038`, plus le stub 0xFFCC après ~1874 insns),
> `d_dsp_page bit1` EST posé (fn≈1206).
