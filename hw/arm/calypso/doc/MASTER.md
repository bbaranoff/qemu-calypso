# CALYPSO QEMU - Document MASTER de la documentation

## Resume du projet

Ce projet emule un telephone GSM a base de chipset TI Calypso (ARM7TDMI + DSP
TMS320C54x + TPU/TSP + BSP) sous QEMU, relie a un reseau GSM logiciel
(osmo-bts-trx / gr-gsm) via des ponts UDP TRXC/TRXD. Le but final est un mobile
qui CAMPE sur une cellule (FB -> SB -> BCCH -> SI -> camped) puis lance un
Location Update, avec a terme un DSP natif capable de vocoder.

Etat terrain au 2026-07-26 : le mobile CAMPE en mode SHUNT_LEGIT (option 3).
gr-gsm decode le downlink et le shunt reinjecte les resultats AU FORMAT NATIF
directement dans s->dsp->data[] (fait central : cellule = data[off/2 + 0x800]).
La chaine complete fonctionne : FB (d_fb_det / a_sync_demod forces) -> SB
(BSIC=7, a_sch CRC) -> rxlev (modele TRF6151, a_pm) -> BCCH (a_cd @0x9D2,
intercept d_task_d / d_burst_d) -> SI1-4 -> C3 camped -> Location Update lance.
Les briques RANK1 (pont ARM->DSP), RANK4 (recalage FN -552) et RANK5
(TRF6151 / rxlev) sont terminees.

Le portage restant est la VOIE NATIVE (dsp_revival, sans shunt) : le correlateur
DSP 0x8d00 reste mure (RANK3, mur data[0x3fae] bit8 jamais ecrit ; correlateur
jamais dispatche). La quasi-totalite des docs "project/" et "archive/" portent
une banniere PERIME auto-declaree (audit 2026-07-01) et decrivent cette voie
native : ils sont historiques vis-a-vis du camp SHUNT_LEGIT mais restent la
reference pour le portage natif restant.

Reference d'adresses canonique du camp : **SHUNT_LEGIT_ADDRESS_MAP.md** (a citer
partout).

> **Unification doc 2026-07-26** : les docs Calypso residuels de `qemu-src/docs/`
> (PLURIEL) ont ete integres dans ce canonique (`hw/arm/calypso/doc/`).
> `qemu-src/docs/` ne contient plus que le QEMU vanilla (arbre sphinx upstream,
> intouchable). 9 divergents archives sous `archive/*.from-docs.md`
> (ARCHITECTURE, BRIDGES, CALL_FLOW, FBSB_FLOW, FLOW, L1CTL_SOCK_FLOW,
> REACTIVER_DSP_REEL, STEP1_PROM_LOADER, STEP2_BC_CONDS), le canonique `project/`
> restant la reference plus riche ; STEP3_F2_F5_STUBS deduplique (identique).
> Le stub `qemu-src/doc/` (singulier) a ete supprime. Aucune perte.

---

## INDEX par theme

### Reference du camp (PRIORITAIRE)

- [SHUNT_LEGIT_ADDRESS_MAP.md](SHUNT_LEGIT_ADDRESS_MAP.md) - **DOC DE REFERENCE**
  du camp option 3 : mapping API RAM (ARM phys / DSP word / api_ram idx), chaine
  camp par etage, formules TRF6151, pieges, table env vars.

### DSP / Correlateur

- [C54X_DECODER_AUDIT.md](C54X_DECODER_AUDIT.md) - Audit read-only du decodeur
  d'opcodes calypso_c54x.c (~120 handlers, ~30 bugs). PERIME 2026-07-01.
- [C54X_INSTRUCTIONS.md](C54X_INSTRUCTIONS.md) - Encodages ISA verifies SPRU172C
  (XC, FRET, FCALL, RETE, BANZ, BC/CC). A jour, atemporel.
- [DOC_CORREL_READ_PATH_2026-07-25.md](DOC_CORREL_READ_PATH_2026-07-25.md) -
  Diag READ-path I/Q du correlateur (AR3/AR5 hors buffer). A jour (voie native).
- [DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md](DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md)
  - Trace 24 etapes boot -> go-live -> correlateur ; divergence dispatch. A jour.
- [DSP_ROM_MAP.md](DSP_ROM_MAP.md) - Cartographie ROM DSP (PROM0-3/DROM/PDROM),
  code cle, vecteurs IT. A jour (voir correction PROM1 dans l'archive).
- [FB_CORRELATOR_PIPELINE.md](FB_CORRELATOR_PIPELINE.md) - Schema pipeline FB/FCCH
  end-to-end. PERIME (these DADST/DSADT fausse), cartographie utile.
- [REPORT_CLAUDE_WEB_DSP_CORRELATOR.md](REPORT_CLAUDE_WEB_DSP_CORRELATOR.md) -
  Rapport review externe "mur MAC=0". PERIME (premisse refutee).
- [REPORT_CLAUDE_WEB_DSP_REVIVAL.md](REPORT_CLAUDE_WEB_DSP_REVIVAL.md) - Rapport
  revival, archi comm ARM<->DSP, offsets MMIO. PERIME sur conclusions, offsets OK.
- [REPORT_DSP_C54X_CORRELATION.md](REPORT_DSP_C54X_CORRELATION.md) - Correlation
  ROM<->decodeur, these PORTW 0x75. PERIME (these caduque).
- [REPORT_DSP_CORRELATOR_RO.md](REPORT_DSP_CORRELATOR_RO.md) - Audit RO mur
  d_fb_det=0. PERIME (cause refutee et deja fixee).
- [REVERT_MVMD_KNOWLEDGE.md](REVERT_MVMD_KNOWLEDGE.md) - Journal revert MVDM/MVMD
  0x72/0x73. Historique - a relire avant de retoucher 0x72/0x73.
- [SP_CATASTROPHE_70c4_SEQUENCE.md](SP_CATASTROPHE_70c4_SEQUENCE.md) - Sequence
  SP-catastrophe (poison 28868). Historique, fix VALIDE cross-run.
- [opcodes/tic54x_hi8_map.md](opcodes/tic54x_hi8_map.md) - Table hi8->mnemonique
  (binutils tic54x-opc.c). A jour, atemporel.
- [opcodes/0x68_0x6F.md](opcodes/0x68_0x6F.md) - Spec famille opcode 0x68-0x6F.
- [opcodes/0xF3.md](opcodes/0xF3.md) - Spec opcode 0xF3xx.
- [project/REACTIVER_DSP_REEL.md](project/REACTIVER_DSP_REEL.md) - Cadrage
  "reactiver le vrai DSP" (CALYPSO_DSP_SHUNT=0). A jour comme cadrage voie native.

### FBSB / Camp / SHUNT_LEGIT

- [SERCOMM_GATE_ARCHITECTURE.md](SERCOMM_GATE_ARCHITECTURE.md) - Archi HW 2 chemins
  (radio bursts BSP<->DSP vs controle L1CTL/sercomm), table Task IDs DSP. A jour.
- [BOOT_TO_FBSB_SEQUENCE.md](BOOT_TO_FBSB_SEQUENCE.md) - Cartographie reset -> boot
  -> FB/SB lock -> CCCH. Historique (voie native pre-shunt).
- [FBSB_SEQUENCE_TRACE.md](FBSB_SEQUENCE_TRACE.md) - Trace osmocom FBSB_REQ->CONF
  + tableau "ou on plante". Historique (hypothese invalidee).
- [calypso_fbsb_mecanisme.md](calypso_fbsb_mecanisme.md) - Call-graph ARM->DSP->
  d_fb_det, 4 acteurs confrontes. Historique/native (portage natif restant).
- [project/FBSB_FLOW.md](project/FBSB_FLOW.md) - Extraction verbatim osmocom
  prim_fbsb.c (contrat NDB immuable). A jour comme reference firmware.

### Hardware / Registres

- [CALYPSO_HW.md](CALYPSO_HW.md) - Reference HW de base (memory-map ARM, API-RAM,
  TPU/INTH, offset ARM byte -> DSP word 0x0800). A jour, stable.
- [REGISTERS_REVIEW.md](REGISTERS_REVIEW.md) - Diag image registres DSP (IMR=0x52FD,
  PMST=0xFFA8...). Historique (premisse corrigee dans l'archive), table utile.
- [hardware-map.md](hardware-map.md) - Memory-map + peripherals du modele QEMU,
  map IRQ, INTH level-sensitive sans ack. A jour.
- [schematics.md](schematics.md) - Schemas mermaid RX radio->DSP->ARM, resume GAP
  A-E. Historique/perime (voie native), vignette case 0x5 a jour.
- [DOC_COMPONENT_WIRING_2026-07-25.md](DOC_COMPONENT_WIRING_2026-07-25.md) -
  **Reference cablage inter-blocs canonique**, tableau WIRED/PARTIAL/GAP, RANK1-6.
- [calypso_audit.md](calypso_audit.md) - Audit code multi-agents, archi globale,
  3 break-points DSP-ROM go-live localises. A jour (reference archi).
- [REPORT_CLAUDE_WEB_IQ_CABLAGE.md](REPORT_CLAUDE_WEB_IQ_CABLAGE.md) - Rapport
  cablage I/Q + mur boot (Break A vrai, Break B refute). PERIME partiel.
- [datasheets/README.md](datasheets/README.md) - Index des datasheets Calypso.

### Sessions / Rapports

- [RAPPORT_GOLIVE_2026-07-25.md](RAPPORT_GOLIVE_2026-07-25.md) - Rapport go-live/
  correlateur natif le plus recent (Fix A BGEN, mur 0x8d00 INTM=1). A jour (natif).
- [SESSION_2026-05-29.md](SESSION_2026-05-29.md) - 3 fixes structurels debloquant
  reset->INTM=0->INT3->PROM0 init. Historique.
- [SESSION_20260403.md](SESSION_20260403.md) - Bug 0xEA BANZ, boot TDMA-tick,
  task_md=5 dispatche, FB result=255. Historique.
- [SESSION_20260405_NIGHT4.md](SESSION_20260405_NIGHT4.md) - Audit opcode, 17 bugs
  C54x corriges vs binutils. Historique.
- [SESSION_20260429.md](SESSION_20260429.md) - 5 fixes opcode, blocker INTM=1
  forever. Historique (duplique dans archive).
- [STATUS.md](STATUS.md) - Etat 2026-05-30 (root SP resolu, blocker FB-dispatch).
  Perime (superseded par rapports Jul-01).
- [status.md](status.md) - Etat precoce pre-DSP (L1CTL_RESET_CONF). Historique.
- [TODO.md](TODO.md) - TODO 2026-05-30 (FB-dispatch, redirect silicon). Historique
  (contradiction avec archive/TODO sur le redirect).
- [doc_master.md](doc_master.md) - Index maitre doc DSP-revival. Perime auto-declare,
  utile comme carte.
- [calypso_grafcet.md](calypso_grafcet.md) - GRAFCET consolide, 2 racines go-live +
  couverture pytest. Historique (analyse native).
- [calypso_grafcet_maison.md](calypso_grafcet_maison.md) - Flowchart maison X0->X14
  (go-live -> L3 LU) mappe aux tests. Historique.
- [README.md](README.md) - Index de base de la doc (deja ecrit).
- [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md) - Audit code arbitre de verite-terrain
  (2026-07-01), cite par tous les bandeaux PERIME.
- [DOC_CORRECTIONS.md](DOC_CORRECTIONS.md) - Journal des corrections de doc.
- [CLEANUP_CANDIDATES.md](CLEANUP_CANDIDATES.md) - Liste des candidats au nettoyage
  (code mort, doublons doc).

### Project

- [project/ARCHITECTURE.md](project/ARCHITECTURE.md) - Archi post-orchestrator.
  Perime (bus pub/sub calypso_orch fabrique).
- [project/AUDIT_DECODER_20260508.md](project/AUDIT_DECODER_20260508.md) - Audit
  decodeur post-fixes (>=11 bugs). Historique.
- [project/BRIDGES.md](project/BRIDGES.md) - Carte des bridges/comm (sockets, UDP
  TRXC/TRXD). Perime (mirror write_cb inexistant).
- [project/BUGS_AND_FIXES.md](project/BUGS_AND_FIXES.md) - Bugs/fixes C54x session
  2026-04-05. Historique.
- [project/CALL_FLOW.md](project/CALL_FLOW.md) - Ordonnancement end-to-end mobile
  <->BTS. Perime auto-declare.
- [project/CLAUDE.md](project/CLAUDE.md) - Contexte projet Claude Code, regles sync
  miroirs. Perime en-tete, consignes methode valides.
- [project/FLOW.md](project/FLOW.md) - Flowchart composants. Perime (bloc ORCH
  inexistant).
- [project/L1CTL_SOCK_FLOW.md](project/L1CTL_SOCK_FLOW.md) - Flux socket L1CTL. Perime
  (API burst-mode inexistante).
- [project/MTTCG_AUDIT.md](project/MTTCG_AUDIT.md) - Audit locks multi-thread TCG.
  Partiellement a jour (coeur locks valide).
- [project/PROJECT_STATUS.md](project/PROJECT_STATUS.md) - Statut projet global.
  Historique (vue haute stable).
- [project/README.md](project/README.md) - README principal qemu-calypso. Perime.
- [project/REFACTORING.md](project/REFACTORING.md) - Guide refactoring (extraction
  TINT0). Perime (structure obsolete).
- [project/REPORT_CLAUDE_WEB_PIPELINE.md](project/REPORT_CLAUDE_WEB_PIPELINE.md) -
  Passation pipeline QEMU<->gr-gsm<->reseau. A jour conceptuellement (approche shunt).
- [project/STATUS_2026-07-01.md](project/STATUS_2026-07-01.md) - **Status natif le
  plus detaille** (1158 l), mur go-live, wait-loop 0xa4d4. A jour (ref native).
- [project/STEP1_PROM_LOADER.md](project/STEP1_PROM_LOADER.md) - Bring-up READA/RPT
  masque MMRs. Historique.
- [project/STEP2_BC_CONDS.md](project/STEP2_BC_CONDS.md) - Conditions BC groupe 1.
  Historique (fix applique).
- [project/STEP3_F2_F5_STUBS.md](project/STEP3_F2_F5_STUBS.md) - Bring-up F2xx/F5xx
  stubs. Historique.
- [project/THREADING_TODO.md](project/THREADING_TODO.md) - TODO threading (1 bloc /
  thread). Ouvert/historique (roadmap).
- [project/TINT0_IMPLEMENTATION.md](project/TINT0_IMPLEMENTATION.md) - Guide horloge
  maitre TINT0. Perime auto-declare.
- [project/TODO_2026-05-25_NIGHT.md](project/TODO_2026-05-25_NIGHT.md) - Plan chasse
  F1 RETE. Perime auto-declare.
- [project/TODO_golive.md](project/TODO_golive.md) - TODO go-live handshake
  0x098a-0x098e (arm2dsp). Ouvert (natif, aligne BGEN).
- [project/c_patches.md](project/c_patches.md) - Patches C 2026-04-07. Perime
  auto-corrige.
- [project/patches.md](project/patches.md) - Patches & TODO 2026-04-07. Perime.
- [project/state.md](project/state.md) - Capabilities verifiees par tests. Partiel
  a jour (inventaire tests).
- [project/test_results.md](project/test_results.md) - Rapport test 2026-05-15
  (35/70 pass). Historique (snapshot date).

### Archive (doublons unification 2026-07-26, remontees de /opt/GSM/qemu-src/doc/)

- [archive/0x68_0x6F.from-src-doc.md](archive/0x68_0x6F.from-src-doc.md) - Spec 0x68-
  0x6F (implemente). Doublon.
- [archive/0xF3.from-src-doc.md](archive/0xF3.from-src-doc.md) - Spec 0xF3xx (INTR
  base 0xF7C0). Doublon corrige.
- [archive/C54X_INSTRUCTIONS.from-src-doc.md](archive/C54X_INSTRUCTIONS.from-src-doc.md)
  - Encodages C54x. Doublon divergent.
- [archive/CALYPSO_HW.from-src-doc.md](archive/CALYPSO_HW.from-src-doc.md) - Ref HW.
  Doublon divergent.
- [archive/DSP_ROM_MAP.from-src-doc.md](archive/DSP_ROM_MAP.from-src-doc.md) - Carte
  ROM DSP. Doublon, porte la correction PROM1 no-mirror (absente du canonique).
- [archive/README.from-src-doc.md](archive/README.from-src-doc.md) - Ancien README
  quickstart. Doublon perime.
- [archive/REGISTERS_REVIEW.from-src-doc.md](archive/REGISTERS_REVIEW.from-src-doc.md)
  - Review .Registers.bin. Doublon, porte la correction faisant foi (dsp-registers
  + loader existent).
- [archive/SERCOMM_GATE_ARCHITECTURE.from-src-doc.md](archive/SERCOMM_GATE_ARCHITECTURE.from-src-doc.md)
  - Archi gate sercomm. Doublon perime.
- [archive/SESSION_20260429.from-src-doc.md](archive/SESSION_20260429.from-src-doc.md)
  - Doublon exact de SESSION_20260429.md.
- [archive/TODO.from-src-doc.md](archive/TODO.from-src-doc.md) - TODO pole 2026-06-25.
  Doublon, corrige : redirect silicon NEUTRALISE par defaut, bypass_bdlena inerte.
- [archive/tic54x_hi8_map.from-src-doc.md](archive/tic54x_hi8_map.from-src-doc.md) -
  Table hi8->mnemonic. Doublon divergent.
