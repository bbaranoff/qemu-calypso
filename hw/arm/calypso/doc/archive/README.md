# Archive — navigation organisée

> Documentation **historique** : sessions datées, rapports d'étape, pistes closes.
> Elle **n'est plus la vérité courante** — source actuelle : **[../ETAT_ACTUEL.md](../ETAT_ACTUEL.md)** (prime en cas de conflit).
> Rangée par thème ci-dessous pour retrouver un sujet ; les `*.from-docs.md` / `*.from-src-doc.md` sont d'anciens extraits.

## FBSB / Corrélateur DSP
*Chasse au d_fb_det natif, pipeline FB, revival DSP, câblage IQ.*

- [`BOOT_TO_FBSB_SEQUENCE.md`](BOOT_TO_FBSB_SEQUENCE.md)
- [`calypso_fbsb_mecanisme.md`](calypso_fbsb_mecanisme.md)
- [`DOC_CORREL_READ_PATH_2026-07-25.md`](DOC_CORREL_READ_PATH_2026-07-25.md)
- [`DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md`](DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md)
- [`FB_CORRELATOR_PIPELINE.md`](FB_CORRELATOR_PIPELINE.md)
- [`FBSB_FLOW.from-docs.md`](FBSB_FLOW.from-docs.md)
- [`FBSB_SEQUENCE_TRACE.md`](FBSB_SEQUENCE_TRACE.md)
- [`project/FBSB_FLOW.md`](project/FBSB_FLOW.md)
- [`project/REACTIVER_DSP_REEL.md`](project/REACTIVER_DSP_REEL.md)
- [`REACTIVER_DSP_REEL.from-docs.md`](REACTIVER_DSP_REEL.from-docs.md)
- [`REPORT_CLAUDE_WEB_DSP_CORRELATOR.md`](REPORT_CLAUDE_WEB_DSP_CORRELATOR.md)
- [`REPORT_CLAUDE_WEB_DSP_REVIVAL.md`](REPORT_CLAUDE_WEB_DSP_REVIVAL.md)
- [`REPORT_CLAUDE_WEB_IQ_CABLAGE.md`](REPORT_CLAUDE_WEB_IQ_CABLAGE.md)
- [`REPORT_DSP_C54X_CORRELATION.md`](REPORT_DSP_C54X_CORRELATION.md)
- [`REPORT_DSP_CORRELATOR_RO.md`](REPORT_DSP_CORRELATOR_RO.md)

## Go-live / IT / SP
*Séquence go-live, IMR/INTM, TINT0, catastrophe SP, MVMD.*

- [`project/TINT0_IMPLEMENTATION.md`](project/TINT0_IMPLEMENTATION.md)
- [`project/TODO_golive.md`](project/TODO_golive.md)
- [`RAPPORT_GOLIVE_2026-07-25.md`](RAPPORT_GOLIVE_2026-07-25.md)
- [`RAPPORT_GOLIVE_2026-07-25.Rmd`](RAPPORT_GOLIVE_2026-07-25.Rmd)
- [`REVERT_MVMD_KNOWLEDGE.md`](REVERT_MVMD_KNOWLEDGE.md)
- [`SP_CATASTROPHE_70c4_SEQUENCE.md`](SP_CATASTROPHE_70c4_SEQUENCE.md)

## ISA / décodeur C54x
*Audits du décodeur TMS320C54x, registres, opcodes.*

- [`0x68_0x6F.from-src-doc.md`](0x68_0x6F.from-src-doc.md)
- [`0xF3.from-src-doc.md`](0xF3.from-src-doc.md)
- [`C54X_DECODER_AUDIT.md`](C54X_DECODER_AUDIT.md)
- [`C54X_INSTRUCTIONS.from-src-doc.md`](C54X_INSTRUCTIONS.from-src-doc.md)
- [`REGISTERS_REVIEW.from-src-doc.md`](REGISTERS_REVIEW.from-src-doc.md)
- [`REGISTERS_REVIEW.md`](REGISTERS_REVIEW.md)
- [`tic54x_hi8_map.from-src-doc.md`](tic54x_hi8_map.from-src-doc.md)

## Adresses / mapping / HW
*Cartes dadresses DSP↔ARM, ROM map, matériel, schémas, Sercomm.*

- [`CALYPSO_HW.from-src-doc.md`](CALYPSO_HW.from-src-doc.md)
- [`DSP_ADDRESS_MAP.md`](../DSP_ADDRESS_MAP.md) *(désarchivé → doc/)*
- [`DSP_ARM_LINKAGE.md`](../DSP_ARM_LINKAGE.md) *(désarchivé → doc/)*
- [`DSP_ROM_MAP.from-src-doc.md`](DSP_ROM_MAP.from-src-doc.md)
- [`schematics.md`](schematics.md)
- [`SERCOMM_GATE_ARCHITECTURE.from-src-doc.md`](SERCOMM_GATE_ARCHITECTURE.from-src-doc.md)

## Architecture / flux / bridges
*Vue densemble, flux dappels, ponts, threading, MTTCG.*

- [`ARCHITECTURE.from-docs.md`](ARCHITECTURE.from-docs.md)
- [`BRIDGES.from-docs.md`](BRIDGES.from-docs.md)
- [`CALL_FLOW.from-docs.md`](CALL_FLOW.from-docs.md)
- [`DOC_COMPONENT_WIRING_2026-07-25.md`](DOC_COMPONENT_WIRING_2026-07-25.md)
- [`FLOW.from-docs.md`](FLOW.from-docs.md)
- [`L1CTL_SOCK_FLOW.from-docs.md`](L1CTL_SOCK_FLOW.from-docs.md)
- [`project/ARCHITECTURE.md`](project/ARCHITECTURE.md)
- [`project/BRIDGES.md`](project/BRIDGES.md)
- [`project/CALL_FLOW.md`](project/CALL_FLOW.md)
- [`project/FLOW.md`](project/FLOW.md)
- [`project/L1CTL_SOCK_FLOW.md`](project/L1CTL_SOCK_FLOW.md)
- [`project/MTTCG_AUDIT.md`](project/MTTCG_AUDIT.md)
- [`project/REFACTORING.md`](project/REFACTORING.md)
- [`project/THREADING_TODO.md`](project/THREADING_TODO.md)

## Boot / PROM / grafcet
*Chargeur PROM, conditions BC, stubs, grafcet.*

- [`calypso_grafcet.md`](calypso_grafcet.md)
- [`calypso_grafcet_maison.md`](calypso_grafcet_maison.md)
- [`project/STEP1_PROM_LOADER.md`](project/STEP1_PROM_LOADER.md)
- [`project/STEP3_F2_F5_STUBS.md`](project/STEP3_F2_F5_STUBS.md)
- [`STEP1_PROM_LOADER.from-docs.md`](STEP1_PROM_LOADER.from-docs.md)
- [`STEP2_BC_CONDS.from-docs.md`](STEP2_BC_CONDS.from-docs.md)

## Patches / bugs / audits
*Correctifs, patches, résultats de test, nettoyage, audits doc/code.*

- [`calypso_audit.md`](calypso_audit.md)
- [`CLEANUP_CANDIDATES.md`](CLEANUP_CANDIDATES.md)
- [`DOC_CODE_AUDIT.md`](DOC_CODE_AUDIT.md)
- [`DOC_CORRECTIONS.md`](DOC_CORRECTIONS.md)
- [`project/c_patches.md`](project/c_patches.md)
- [`project/patches.md`](project/patches.md)
- [`project/test_results.md`](project/test_results.md)

## Statuts & sessions (datés)
*Instantanés historiques : sessions, statuts, TODO, state.*

- [`doc_master.md`](doc_master.md)
- [`MASTER.md`](MASTER.md) *(ancien index maître — supplanté par [../ETAT_ACTUEL.md](../ETAT_ACTUEL.md) + [../README.md](../README.md) ; liens internes obsolètes)*
- [`project/CLAUDE.md`](project/CLAUDE.md)
- [`project/PROJECT_STATUS.md`](project/PROJECT_STATUS.md)
- [`project/state.md`](project/state.md)
- [`project/STATUS_2026-07-01.md`](project/STATUS_2026-07-01.md)
- [`project/TODO_2026-05-25_NIGHT.md`](project/TODO_2026-05-25_NIGHT.md)
- [`README.from-src-doc.md`](README.from-src-doc.md)
- [`SESSION_2026-05-29.md`](SESSION_2026-05-29.md)
- [`SESSION_20260403.md`](SESSION_20260403.md)
- [`SESSION_20260405_NIGHT4.md`](SESSION_20260405_NIGHT4.md) *(encodages F0xx/RSBX/RET/CALLD/IDLE — réf ISA durable, cf. ETAT_ACTUEL §7)*
- [`SESSION_20260429.from-src-doc.md`](SESSION_20260429.from-src-doc.md)
- [`SESSION_20260429.md`](SESSION_20260429.md)
- [`status.md`](status.md)
- [`STATUS.md`](STATUS.md)
- [`TODO.from-src-doc.md`](TODO.from-src-doc.md)
