# Documentation Calypso QEMU — Index canonique

> Arbre de doc **CANONIQUE** (a cote du code, `hw/arm/calypso/doc/`).
> L'ancien arbre `qemu-src/doc/` a ete resorbe ici (aucune perte : uniques
> deplaces, identiques supprimes, divergents archives sous `archive/`).
>
> **2026-07-26** : les docs Calypso de `qemu-src/docs/` (PLURIEL) ont ete
> integres ici a leur tour ; `qemu-src/docs/` = QEMU vanilla pur desormais
> (arbre sphinx upstream, INTOUCHABLE). 10 docs traites : 9 divergents archives
> sous `archive/<nom>.from-docs.md` (le canonique `project/` restant la reference,
> plus riche), 1 identique (`STEP3_F2_F5_STUBS.md`) deduplique. Aucune perte.

---

## Etat du projet — 2026-07-26

**CAMP ATTEINT (SHUNT_LEGIT, "option 3", `CALYPSO_SHUNT_LEGIT=1`).**
gr-gsm decode le vrai downlink GSM et le shunt QEMU injecte les resultats au
format natif dans l'API RAM du DSP ; le firmware ARM (osmocom-bb) campe
normalement : **FB -> SB (BSIC=7) -> rxlev (trf6151, -60 dBm) -> BCCH/a_cd ->
SI1-4 -> C3 camped normally.**

**FAIT CENTRAL** : le firmware ARM lit les resultats DSP dans `s->dsp->data[]`
(`calypso_trx.c`, `val = data[off/2 + 0x0800]`), **PAS** dans `dsp_ram[]` ni
directement dans `api_ram[]`. Un shunt fonctionnel ecrit donc **directement**
`data[]`/`api_ram[]` (on_frame_tick + FORCE c54x) et intercepte les reads ARM ;
les `shunt_dispatch_*` passant par `dma_memory_write` vers `dsp_ram[]` ne sont
PAS vus par le firmware. Details : **[SHUNT_LEGIT_ADDRESS_MAP.md](SHUNT_LEGIT_ADDRESS_MAP.md)**.

**Points ouverts :**
- `d_burst_d` coherent (index de burst read-page vs write-page) — workflow separe.
- **Location Update / RACH UL** (montant) — le camp est DL only pour l'instant.

**Deux voies, ne pas confondre :**
- **Voie SHUNT_LEGIT (operationnelle, camp OK)** — decrite par
  `SHUNT_LEGIT_ADDRESS_MAP.md`. C'est la reference courante.
- **Voie NATIVE (`dsp_revival`, encore bloquee)** — faire tourner le vrai
  corrélateur DSP c54x pour qu'il ecrive `d_fb_det != 0` lui-meme. Toujours
  `d_fb_det=0` (mur go-live / pointeurs AR3/AR5 hors buffer I/Q). Les docs FBSB,
  corrélateur, go-live ci-dessous decrivent cette voie ; ils restent utiles comme
  cartographie mais **ne refletent pas le camp** — se reporter a la map.

---

## Reference du camp (a lire en premier)

| Doc | Description |
|---|---|
| [SHUNT_LEGIT_ADDRESS_MAP.md](SHUNT_LEGIT_ADDRESS_MAP.md) | **Reference camp.** Mapping complet des cellules DSP (data/api_ram/dsp_ram), chaine FB/SB/rxlev/a_cd, TRF6151, pieges, variables d'env. |

## Materiel & ROM (reference stable)

| Doc | Description |
|---|---|
| [CALYPSO_HW.md](CALYPSO_HW.md) | Reference hardware Calypso : ARM7/C54x, memory map, API RAM 0xFFD00000, TPU/ABB/BSP. |
| [DSP_ROM_MAP.md](DSP_ROM_MAP.md) | Cartographie du ROM DSP (sections PROM0-3/DROM/PDROM, adresses cles). Base PROM0 = 0x7000. |
| [C54X_INSTRUCTIONS.md](C54X_INSTRUCTIONS.md) | Jeu d'instructions TMS320C54x (decode / emulation). |
| [REGISTERS_REVIEW.md](REGISTERS_REVIEW.md) | Revue de l'image registres DSP (c54x_reset), drift fichier vs code. |
| [SERCOMM_GATE_ARCHITECTURE.md](SERCOMM_GATE_ARCHITECTURE.md) | Archi sercomm : chemin bursts (BSP->DSP) vs chemin controle (L1CTL/UART). |

## Voie native FBSB / corrélateur / go-live (encore ouverte)

> Ces docs decrivent la voie NATIVE `dsp_revival` (corrélateur DSP reel),
> encore bloquee (`d_fb_det=0`). Le camp operationnel passe par SHUNT_LEGIT.

| Doc | Description |
|---|---|
| [BOOT_TO_FBSB_SEQUENCE.md](BOOT_TO_FBSB_SEQUENCE.md) | Chemin natif reset silicon -> FB lock -> SB lock -> FBSB_CONF. |
| [FBSB_SEQUENCE_TRACE.md](FBSB_SEQUENCE_TRACE.md) | Trace L1CTL_FBSB_REQ -> CONF (firmware + QEMU), points de rupture natif. |
| [calypso_fbsb_mecanisme.md](calypso_fbsb_mecanisme.md) | Call-graph FBSB (ARM<->ROM DSP<->shunt<->orch), acteurs et gates. |
| [FB_CORRELATOR_PIPELINE.md](FB_CORRELATOR_PIPELINE.md) | Pipeline du corrélateur FB/FCCH c54x (composants, opcodes DADST/DSADT). |
| [DOC_CORREL_READ_PATH_2026-07-25.md](DOC_CORREL_READ_PATH_2026-07-25.md) | Diag du read path I/Q corrélateur (BSP 0x2a00 vs AR3/AR5 hors buffer). |
| [DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md](DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md) | 24 etapes boot -> go-live -> L1 -> corrélateur, citees live. |
| [DOC_COMPONENT_WIRING_2026-07-25.md](DOC_COMPONENT_WIRING_2026-07-25.md) | Cablage inter-blocs (TPU/TSP/BSP/DSP/ARM), chaine RX/FB et IT. |

## Rapports & audits (analyse)

| Doc | Description |
|---|---|
| [RAPPORT_GOLIVE_2026-07-25.md](RAPPORT_GOLIVE_2026-07-25.md) / [.Rmd](RAPPORT_GOLIVE_2026-07-25.Rmd) | Rapport go-live (grafcet, wiring inter-blocs). Contenu jumeau md/Rmd. |
| [C54X_DECODER_AUDIT.md](C54X_DECODER_AUDIT.md) | Audit du decodeur c54x. |
| [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md) | Audit doc-vs-code (reference des bandeaux PERIME 2026-07-01). |
| [DOC_CORRECTIONS.md](DOC_CORRECTIONS.md) | Corrections doc consolidees. |
| [CLEANUP_CANDIDATES.md](CLEANUP_CANDIDATES.md) | Candidats au nettoyage (docs/redondances). |
| [calypso_audit.md](calypso_audit.md) | Audit general Calypso. |
| [calypso_grafcet.md](calypso_grafcet.md) / [calypso_grafcet_maison.md](calypso_grafcet_maison.md) | Grafcets de la sequence L1. |
| [hardware-map.md](hardware-map.md) / [schematics.md](schematics.md) | Carte materielle / schematiques. |
| [doc_master.md](doc_master.md) | Doc maitresse historique (marquee PERIME natif). |
| [REPORT_CLAUDE_WEB_DSP_CORRELATOR.md](REPORT_CLAUDE_WEB_DSP_CORRELATOR.md), [REPORT_CLAUDE_WEB_DSP_REVIVAL.md](REPORT_CLAUDE_WEB_DSP_REVIVAL.md), [REPORT_CLAUDE_WEB_IQ_CABLAGE.md](REPORT_CLAUDE_WEB_IQ_CABLAGE.md), [REPORT_DSP_C54X_CORRELATION.md](REPORT_DSP_C54X_CORRELATION.md), [REPORT_DSP_CORRELATOR_RO.md](REPORT_DSP_CORRELATOR_RO.md) | Rapports d'analyse DSP/corrélateur/IQ (voie native). |

## Incidents & lecons resolues

| Doc | Description |
|---|---|
| [SP_CATASTROPHE_70c4_SEQUENCE.md](SP_CATASTROPHE_70c4_SEQUENCE.md) | Root SP=0x1100 non initialise -> wedge 0x70c3 (RESOLU : redirect 0xFF80->0x7120). |
| [REVERT_MVMD_KNOWLEDGE.md](REVERT_MVMD_KNOWLEDGE.md) | Connaissances MVMD/MVDD (revert). |

## Sessions (historique)

| Doc | Description |
|---|---|
| [SESSION_20260403.md](SESSION_20260403.md) / [SESSION_20260405_NIGHT4.md](SESSION_20260405_NIGHT4.md) / [SESSION_20260429.md](SESSION_20260429.md) / [SESSION_2026-05-29.md](SESSION_2026-05-29.md) | Notes de session datees (journal de debug). |

## Etat & taches

| Doc | Description |
|---|---|
| [STATUS.md](STATUS.md) | Etat courant (voie native ; **perime pour le camp** : voir en-tete de ce README). |
| [TODO.md](TODO.md) | Hacks env-gated + criteres de retrait ; objectif natif FB-dispatch. |
| [status.md](status.md) | Ancien status (voie native, PERIME ; supersede par ce README). |

## Sous-repertoires

| Repertoire | Contenu |
|---|---|
| `datasheets/` | PDF TI (SPRU131G/172C/288, SPRA036/618), dumps ROM DSP (3311/3416/3606), `README.md`. |
| `opcodes/` | Cartes d'opcodes C54x : `tic54x_hi8_map.md`, `0x68_0x6F.md`, `0xF3.md`. |
| `project/` | Docs projet resorbes (ARCHITECTURE, BRIDGES, FLOW, STATUS dates, TODO_golive, etc.). |
| `archive/` | Versions divergentes archivees lors de l'unification (`<nom>.from-src-doc.md` depuis `doc/` singulier ; `<nom>.from-docs.md` depuis `docs/` pluriel) + docs superseded. |

---

*Regle #1 : on repare le cablage de l'emulateur ; les hacks env-gated sont listes
dans [TODO.md](TODO.md) avec leur critere de retrait. `HACK=0` = natif.*
