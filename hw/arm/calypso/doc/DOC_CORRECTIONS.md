# Journal des corrections documentaires — passe audit doc↔code

**Date de la passe :** 2026-07-01
**Référence de vérité terrain :** [`DOC_CODE_AUDIT.md`](DOC_CODE_AUDIT.md)
**Portée :** correction des documents `doc/` contre le code réel des deux arbres
(`/opt/GSM/qemu-calypso/doc/` et `/opt/GSM/qemu-src/doc/`).

Chaque document corrigé a été réinstallé à l'identique dans les **deux** arbres.

---

## 1. Résumé chiffré

| Métrique | Total |
|---|---:|
| Documents corrigés | **39** |
| Documents installés dans les deux arbres | **39 / 39** |
| Bannières PÉRIMÉ / correctives ajoutées | **26** |
| Citations de lignes périmées corrigées | **112** |
| Faits (Family C) corrigés | **96** |
| Reclassements (Family A/B — BUG/TODO → FIXÉ, ou FIXÉ → FAUX) | **57** |
| Findings non appliqués (à revoir manuellement) | **4** (répartis sur 2 docs) |
| Échecs d'installation | **0** |

**Thèses fausses récurrentes redressées dans la passe :**

- Le verrou réel n'est **pas** VEC28 / le décodeur DSP : c'est le **handshake ARM→DSP go-live** jamais armé
  (`api_write_cb` déclaré `calypso_c54x.h:204`, invoqué `calypso_c54x.c:3357-3358`, **jamais assigné** — `grep 'api_write_cb *=' = 0`).
- `d_fb_det` reste **0** ; le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000) ; **IMR=0x0000** jamais réarmé après le clear boot @0xb37e.
- L'ARM n'écrit que **0x0000** vers l'API 0x0314/0x0318.
- Il n'existe **aucun bus ORCH** (pas de `calypso_orch.c`, pas d'UDP 6920/6921).
- `CALYPSO_DSP_FRAME_VEC28` est un **cul-de-sac** (dérive vers le boot-stub).

---

## 2. Table par document

| Document | Types de corrections | Nb (faits / stale / reclass / bannière) |
|---|---|---|
| `C54X_DECODER_AUDIT.md` | Bannière B, faits, remap ancres décodeur, reclassement §2/§2.6 | 3 / 11 / 1 / ✅ |
| `C54X_INSTRUCTIONS.md` | Faits (Family C : 0xEA, BANZ/BANZD) | 2 / 0 / 0 / — |
| `CALYPSO_HW.md` | Faits (TPU_CTRL bits, FRAME vec) | 2 / 0 / 0 / — |
| `CLEANUP_CANDIDATES.md` | Reclassement SAFE-DELETE, tailles fichiers | 2 / 1 / 1 / — |
| `doc_master.md` | Bannière B, réécriture pièce manquante, leviers | 2 / 2 / 1 / ✅ |
| `DSP_ROM_MAP.md` | Faits (mirror PROM1 inexistant, PDROM) | 2 / 0 / 0 / — |
| `FB_CORRELATOR_PIPELINE.md` | Bannière B, reclass DADST/DSADT FIXÉ, remap lignes | 2 / 9 / 1 / ✅ |
| `opcodes/0x68_0x6F.md` | Reclassement 8× MISSING → FIXÉ, note IMPLÉMENTÉ | 0 / 0 / 8 / — |
| `opcodes/0xF3.md` | Bannière B, INTR base 0xF7C0, F310=SUB, reclass | 6 / 1 / 3 / ✅ |
| `opcodes/tic54x_hi8_map.md` | Faits + reclass massifs (STUB-NOP), ancres traceurs | 13 / 7 / 10 / — |
| `project/ARCHITECTURE.md` | Bannière B, ORCH/6920/6921 fabriqués, qemu_irq_raise | 2 / 1 / 0 / ✅ |
| `project/BRIDGES.md` | Bannière B, trx_dsp_api_write_cb inexistant, UART/SOC | 1 / 2 / 1 / ✅ |
| `project/BUGS_AND_FIXES.md` | Reclassement Bug 17 FIXÉ | 0 / 1 / 1 / — |
| `project/CALL_FLOW.md` | Bannière B, publish_fb_found FAUX, synth host retiré | 2 / 0 / 0 / ✅ |
| `project/CLAUDE.md` | Bannière B, PORTR/a_cd[15], INTM FAUX, comptes lignes | 4 / 2 / 1 / ✅ |
| `project/c_patches.md` | Bannière B, F273=BD, handlers uniques, reclass CONFIRMED | 3 / 4 / 3 / ✅ |
| `project/FBSB_FLOW.md` | Ancres FBSB (trx.c:889, >>3) | 0 / 2 / 0 / — |
| `project/FLOW.md` | Bannière B, ORCH inexistant, api_write_cb | 2 / 1 / 0 / ✅ |
| `project/L1CTL_SOCK_FLOW.md` | Bannière B, burst API inexistante, l1ctl actif, remap | 3 / 7 / 1 / ✅ |
| `project/MTTCG_AUDIT.md` | Bannière B, api_ram sites, api_ram_lock déjà pris | 2 / 2 / 1 / ✅ |
| `project/patches.md` | Bannière B, F074/F274 delay-slots, ancres struct | 2 / 3 / 1 / ✅ |
| `project/REACTIVER_DSP_REEL.md` | Loaders réels, c54x_bsp_load, run.sh, tailles | 6 / 6 / 0 / — |
| `project/README.md` | Bannière B, réfutation breakthrough 05-15, comptes | 3 / 1 / 2 / ✅ |
| `project/REFACTORING.md` | Reclassement calypso_sim.c déjà créé | 0 / 1 / 1 / — |
| `project/REPORT_CLAUDE_WEB_PIPELINE.md` | Mécanisme FIFO writer/ring, 5 FIFOs | 2 / 2 / 0 / — |
| `project/STEP1_PROM_LOADER.md` | Loader réel c54x_load_section | 0 / 1 / 0 / — |
| `project/STEP2_BC_CONDS.md` | Bannière FIXÉ, décode BC déjà implémenté | 0 / 0 / 1 / ✅ |
| `project/THREADING_TODO.md` | Ancres calypso_tdma_tick / c54x_run | 0 / 1 / 0 / — |
| `project/TINT0_IMPLEMENTATION.md` | Bannière B, FRAME vec 19/3, timer inexistant, Docker | 2 / 1 / 2 / ✅ |
| `project/TODO_2026-05-25_NIGHT.md` | Bannière B, gate BRINT0 réel | 0 / 1 / 0 / ✅ |
| `REGISTERS_REVIEW.md` | Bannière, dsp-registers existe, mode bin défaut, c54x_reset | 4 / 3 / 2 / ✅ |
| `REPORT_CLAUDE_WEB_DSP_CORRELATOR.md` | Bannière B, réfutation MAC/MVDM, ancres | 0 / 4 / 0 / ✅ |
| `REPORT_CLAUDE_WEB_DSP_REVIVAL.md` | Bannière B, réfutation FB/SB, ancres api_ram/DMA/idle | 7 / 5 / 3 / ✅ |
| `REPORT_CLAUDE_WEB_IQ_CABLAGE.md` | Bannière B, Break B réfuté, diagramme I/Q, feed_iq | 4 / 1 / 2 / ✅ |
| `REPORT_DSP_C54X_CORRELATION.md` | Bannière B, PORTW 0x75 déjà FIXÉ, 18 ancres | 1 / 18 / 2 / ✅ |
| `REPORT_DSP_CORRELATOR_RO.md` | Bannière B, DADST/dual-MAC FIXÉ, ancres | 6 / 7 / 5 / ✅ |
| `schematics.md` | Bannière, VEC28 cul-de-sac, DADST FIXÉ, trx.c:553-566 | 2 / 2 / 1 / ✅ |
| `SERCOMM_GATE_ARCHITECTURE.md` | Bannière, BSP dans calypso_bsp.c, DLCI 4 TRXC | 2 / 0 / 0 / ✅ |
| `TODO.md` | Bannière, redirect legacy OFF, bypass_bdlena inerte | 2 / 2 / 2 / ✅ |

---

## 3. Findings NON appliqués (à revoir manuellement)

Ces éléments n'ont **pas** été corrigés dans la passe car ils n'ont pu être
confirmés sans ambiguïté au re-check. Ils sont laissés en l'état et signalés.

### `C54X_DECODER_AUDIT.md`
1. **Remap complet des numéros de ligne non tenté.** L'audit décodeur contient
   ~150 citations `c54x.c:NNNN` (le fichier a grossi d'environ 1050 lignes) mais
   le finding ne fournissait qu'un jeu d'ancres vérifiées précises. Seules ces
   ancres vérifiées ont été remappées ; les citations restantes du corps décodeur
   (ex. 7878, 7902+, 6062, 6866, 5056-5191) sont couvertes par la note explicite
   de la bannière (« numéros de ligne périmés ») **plutôt que décalées à l'aveugle**,
   pour éviter d'introduire de fausses ancres. → **À remapper manuellement.**
2. **`api_write_cb`** est invoqué à `calypso_c54x.c:3357-3358` (`if (s->api_write_cb) ...`)
   mais, selon la vérité terrain, le pointeur n'est **jamais assigné**
   (`grep 'api_write_cb *=' = 0`) — donc toujours NULL. Ce doc ne décrivait pas
   `api_write_cb` directement, donc seule la bannière y fait référence.

### `REPORT_DSP_CORRELATOR_RO.md`
1. Les réfs de ligne de **R5** (`RSBX c54x.c:6329`, branche `MVDD 6489`) laissées
   intactes — hors findings et les nouvelles lignes exactes n'ont pu être
   confirmées sans ambiguïté ; R5 est REFUTED de toute façon, donc aucun impact
   factuel. → **À revérifier si R5 est réactivé.**
2. Les métriques runtime d'anciennes sessions (CORR-PEAK figées A/B/T, 84/84
   DETECTOR-RUN, SPAN/POST-WATCH) n'ont **pas** été re-jouées ; conservées comme
   historique et signalées « pré-derail » selon la vérité terrain plutôt que
   supprimées.

---

## 4. Documents NON installés / échecs

**Aucun.** Les 39 documents corrigés ont été installés avec succès dans les deux
arbres (`/opt/GSM/qemu-calypso/doc/` et `/opt/GSM/qemu-src/doc/`). Aucun échec
d'installation, aucun document laissé non corrigé.
