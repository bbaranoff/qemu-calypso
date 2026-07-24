# Audit Doc ↔ Code — Calypso QEMU (`doc/*.md`)

**Fichier :** `doc/DOC_CODE_AUDIT.md`
**Date :** 2026-07-01
**Portée :** confrontation adversariale de la documentation `/opt/GSM/qemu-calypso/doc/*.md` au code `hw/arm/calypso/` (calypso_c54x.c, calypso_trx.c, calypso_bsp.c, calypso_dsp_shunt.c, calypso_soc.c, calypso_mb.c, calypso_fbsb.c + en-têtes `include/hw/arm/calypso/`).

## 1. Méthodologie et résumé chiffré

Trois passes successives :
1. **Audit adversarial doc→code** : chaque affirmation testable d'un doc a été confrontée au code réel.
2. **Verify de défense** : chaque discordance a été re-vérifiée en cherchant une preuve *à décharge* dans le code (le doc a-t-il raison malgré tout ?), pour éliminer les fausses alertes.
3. **Vérité-terrain de session** : les affirmations dynamiques ont été jugées contre l'état d'exécution établi et authoritatif de cette session (ci-dessous).

### Vérité-terrain (authoritative — arbitre toutes les affirmations dynamiques)
- `IMR = 0x0000` pendant **tout** le run (jamais ré-armé après le clear de boot à PC=0xb37e).
- Le DSP **déraille** (POST-BOOTSTUB-RET, PC=0x0000) au lieu d'exécuter le corrélateur.
- `d_fb_det` reste **0** tout le run.
- Le handshake go-live ARM→DSP ne s'arme **jamais** (l'ARM n'écrit que `0x0000` dans les API 0x0314/0x0318).
- `api_write_cb` est **déclaré** (calypso_c54x.h:204) mais **jamais assigné** (`grep 'api_write_cb *=' hw/ include/` = 0 hit).
- Le levier `CALYPSO_DSP_FRAME_VEC28` est un **cul-de-sac** : il force-vectorise mais retombe dans une épilogue d'ISR et déraille vers le boot-stub (confirmé par run).
- `calypso_c54x.c` fait désormais **13 697 lignes** (≈12 646 auparavant) : la plupart des citations `file:line` des docs sont périmées, même quand le comportement décrit tient encore.

### Chiffres
| Métrique | Valeur |
|---|---|
| Documents audités | 51 |
| Affirmations réfutées (toutes sévérités) | 165 |
| Discordances HIGH confirmées (cette synthèse) | 52 (dont 4 `ALREADY_ACTIONED`) |
| Citations de ligne périmées (STALE) | 82 |
| Findings archive-tier (dérive historique attendue, non-bloquants) | 19 |
| Fausses alertes (findings de défense retournés) | voir §3, colonne verdict |

---

## 2. Thèmes transversaux (le cœur)

Les discordances CONFIRMED se regroupent en quatre familles. Elles expliquent *pourquoi* la doc induit en erreur le travail en cours.

### Famille A — « Code corrigé après le doc » (bugs décrits comme ouverts mais **déjà fixés**)

Le doc décrit encore un bug de décodage comme le blocage racine, alors qu'un handler dédié a été ajouté depuis. **Ne pas ré-appliquer ces correctifs.**

| Sujet | Doc(s) | Preuve du fix (file:line) |
|---|---|---|
| DADST/DSADT (0x5A/5B/5E/5F) « tombent en SFTA/SFTL case 0x5 » | FB_CORRELATOR_PIPELINE, REPORT_DSP_CORRELATOR_RO, schematics | Handler dual long-word dédié `calypso_c54x.c:8232-8305` (« fix revival dsp 2026-06-22 »), exécuté avant le fallthrough SFTA/SFTL |
| 0x68xx–0x6Fxx marqués « MISSING » | 0x68_0x6F.md | Handlers dédiés `calypso_c54x.c:7763/7772/7781/7790/7800/7817/7851` (2026-04-28) ; le fallback 0x6800 est **dead code** annoté à `:7909` |
| F300–F31F « garder INTR k » | 0xF3.md | INTR retiré du F3xx (`calypso_c54x.c:6380-6386`) ; F300–F35F routé en ALU #lk (`:6434`) ; le vrai `F310=SUB` était le wedge réel (`:6439-6442`) ; base INTR = 0xF7C0 |
| 0x8B « MVDK long-addr 2-word, TODO » | tic54x_hi8_map.md | Neutralisé en NOP 1-word `calypso_c54x.c:8777-8782` |
| 0xDD « POPD Smem → SP runaway, TODO » | tic54x_hi8_map.md | Neutralisé en NOP, SP-runaway corrigé `calypso_c54x.c:9354-9365` |
| PORTW 0x75 « sous-décodé, phantom CC@0xb418 » | REPORT_DSP_C54X_CORRELATION | Handler 3-word inconditionnel `calypso_c54x.c:7501` (revival 2026-06-23) |
| BC F8 group-1 « conds manquantes » / fix proposé | STEP2_BC_CONDS | Décode via branche sub 0x4/0x5 `calypso_c54x.c:6308-6321` ; fix déjà en place (« FIX 2026-06-23 » `:6296`) — `ALREADY_ACTIONED` |
| AR0 « QEMU=0xFF75, off-by-one » | REGISTERS_REVIEW | Corrigé : `s->ar[0] = 0x5AAD` `calypso_c54x.c:13301` (FIX 2026-05-31) — `ALREADY_ACTIONED` |
| « dsp-registers n'est pas une property » | REGISTERS_REVIEW | Property + loader présents : `calypso_mb.c:53/301/341`, loader `calypso_c54x.c:13212` appelé de `calypso_trx.c:2438` ; `bin` = mode reset par défaut |
| F273 « RETD, doublon à dédupliquer » | c_patches | F273 = **BD** (branche différée), handler unique `calypso_c54x.c:6032` ; vrai RETD = 0xFE00 — ne PAS appliquer le patch |

### Famille B — « Narratif de blocage périmé » (FB détecté / cascade débloquée — contredit par la vérité-terrain)

Ces docs affirment que le FB est détecté, que la cascade FB→SB est débloquée, que `d_fb_det≠0`, et décrivent une infrastructure (`api_write_cb` DSP→ARM, bus ORCH UDP 6920/6921, API steal l1ctl) **qui n'existe pas dans le code**. Ce sont les plus dangereux car ils envoient le travail dans de fausses pistes.

- **`api_write_cb` jamais câblé** : déclaré `calypso_c54x.h:204`, appelé conditionnellement `calypso_c54x.c:3357-3358 (if (s->api_write_cb))`, **jamais assigné** (`grep 'api_write_cb *=' = 0`). ⟶ contredit README, FLOW, BRIDGES, CALL_FLOW, ARCHITECTURE, REPORT_CLAUDE_WEB_DSP_REVIVAL.
- **Aucun `calypso_orch.c`, aucun bus UDP 6920/6921** : `calypso_orch.h` = un seul `static inline int calypso_orch(void)` (flag env) ; `grep 6920/6921/steal_init = 0`. ⟶ toute la section « ORCH node / pub-sub / l1ctl steal » d'ARCHITECTURE est fabriquée.
- **`d_fb_det` reste 0** : la lecture `0x6b34 FOUND` (REPORT_CLAUDE_WEB_DSP_REVIVAL) n'existe que dans un commentaire `calypso_trx.c:259`, jamais produite. Le publish FBSB est **triple-gardé** (orch + env synth + détection BSP réelle), pas inconditionnel (CALL_FLOW faux).
- **Handshake go-live jamais armé** : HS-ARM-GATE est read-only `calypso_trx.c:557-561` (« no DSP/ARM state is poked here »).
- **INTM=1 non résolu** : CLAUDE le dit « résolu naturellement » — faux, l'IT frame retombe sur un stub RETE (vec19/bit3), le vrai scheduler vec28/bit12 n'est atteint que sous levier expérimental, IMR reste 0x0000 (`calypso_c54x.c:13537-13546`).
- **API burst-mode L1CTL inexistante** : `grep burst_mode/l1ctl_set_burst_mode = 0` ; pas de champ `cli_rx_enabled`/`vm_start` ; `l1ctl_sock_init` est **actif** (env-gated) `calypso_soc.c:334`, pas commenté (L1CTL_SOCK_FLOW faux sur les 3 points).
- **Câblage IQ « manquant » inexistant** : `sl->iq` est déjà chargé dans `bsp_buf` dans `deliver_buffered` (`calypso_bsp.c:1248`) ; le fix `last_iq` proposé est redondant (REPORT_CLAUDE_WEB_IQ_CABLAGE faux sur les 2 points).

### Famille C — « Erreurs factuelles de référence » (encodages / registres / vecteurs faux)

| Doc | Affirmation fausse | Réalité code |
|---|---|---|
| C54X_INSTRUCTIONS | `0xEA = BANZ` | `0xEA00 = LD #k9,DP` `calypso_c54x.c:7145` |
| C54X_INSTRUCTIONS | `BANZ=0x78xx, BANZD=0x7Axx` | `0x78xx = STH` `:7658` ; BANZ=`0x6Cxx` `:7800`, BANZD=`0x6Exx` `:7817` |
| CALYPSO_HW | TPU_CTRL bit0=EN, bit2=IDLE | RESET=bit0, EN=**bit2**, IDLE=**bit8** (calypso_trx.h:60-66) |
| CALYPSO_HW | FRAME→DSP = « SINT17, vec 2 » | `C54X_INT_FRAME_VEC 19` (INT3) `calypso_c54x.h:126`, bit 3 |
| TINT0_IMPLEMENTATION | `FRAME_VEC=2, FRAME_BIT=1` | vec 19 / bit 3 `calypso_c54x.h:126-127` |
| TINT0_IMPLEMENTATION | TIM décrémenté par instruction (~2490-2520) | Ces lignes sont des sondes debug `calypso_c54x.c:2482-2530`, pas de timer per-instruction |
| DSP_ROM_MAP | PROM1 miroité en prog[0x8000-0xFFFF] | Pas de miroir low-64K (retiré 2026-05-29) `calypso_trx.c:2397-2401` ; PROM0 en 0x8000-0xDFFF |
| DSP_ROM_MAP | Table vecteurs 0xFF80 en PROM1 | Servie depuis **PDROM** en prog (0xE000+) `calypso_trx.c:2416-2417`, `c54x_prog_xlate:3672-3675` |
| SERCOMM_GATE_ARCHITECTURE | `rx_burst` appelle `bsp_load`+BRINT0 | `calypso_trx_rx_burst()` est un **no-op** `calypso_trx.c:2166-2182` ; le vrai `c54x_bsp_load` est dans `calypso_bsp.c:1023/1248` |
| SERCOMM_GATE_ARCHITECTURE | « pas de routage spécial DLCI 4 » | DLCI 4 = TRXC intercepté `sercomm_gate.c:99/209`, `gate_trxc_handle():137` |
| CLAUDE | PORTR PA correct = 0x0034, 0xF430 faux | Inversé : **0xF430** = port BSP réel (64 sites ROM) `calypso_c54x.c:8609-8613` ; 0x0034 = alias legacy ; décodé sur opcode 0x74 |
| CLAUDE | a_cd[15] à NDB+0x1DC (0x0384) | NDB a_cd = **0x1FC** (DWARF) ; 0x1DC explicitement marqué faux `calypso_dsp_shunt.c:94-104` (2026-06-02) |
| patches | F074 et F274 « tous deux corrects » | F074 OK (`:5842-5849`) mais F274 = **CALLD** (différé 2 slots) `:6015-6029`, pas un call immédiat |

### Famille D — « Dérive de citations » (numéros de ligne périmés)

`calypso_c54x.c` est passé de ≈12 646 à **13 697** lignes ; `calypso_trx.c` ne fait plus que 2 481 lignes. Presque toutes les citations `file:line` des docs sont décalées (+250 à +1000 lignes dans c54x.c), et plusieurs docs citent des lignes 3053-3070 de `calypso_trx.c` qui **n'existent pas**. Détail exhaustif en §4 (82 entrées).

---

## 3. Table des discordances HIGH confirmées

Verdicts : `CONFIRMED_DISCREPANCY` = le doc a tort ; `ALREADY_ACTIONED` = le doc avait raison, déjà corrigé (marquer DONE). Aucune fausse alerte n'a survécu à la passe de défense pour les items ci-dessous.

| Doc | Affirmation du doc | Preuve code | Correction |
|---|---|---|---|
| C54X_DECODER_AUDIT | Blocage = IRQ-deadlock INTM stuck (IPTR=0x1FF) | `calypso_c54x.c:13593/13624` gate sur `imr` | IMR=0x0000 tout le run ⟶ aucun vecteur dispatché ; blocage = IMR-jamais-réarmé/DSP déraille |
| C54X_INSTRUCTIONS | `0xEA = BANZ (confirmé)` | `:7145` masque 0xFE00 | 0xEA00 = LD #k9,DP |
| C54X_INSTRUCTIONS | `BANZ=0x78xx, BANZD=0x7Axx` | `:7658/7800/7817` | 0x78xx=STH ; BANZ=0x6Cxx, BANZD=0x6Exx |
| CALYPSO_HW | TPU_CTRL EN=bit0, IDLE=bit2 | calypso_trx.h:60-66 | EN=bit2 (bit0=RESET), IDLE=bit8 |
| CALYPSO_HW | FRAME→DSP = SINT17/vec2 | `calypso_c54x.h:126`, `dsp_shunt.c:1043` | vec 19 (INT3, bit 3) |
| CLEANUP_CANDIDATES | `calypso_dbg.c` à SAFE-DELETE | `find` = rien | **ALREADY_ACTIONED** : supprimé cette session, marquer DONE |
| DSP_ROM_MAP | PROM1 + miroir 0x8000-0xFFFF | `calypso_trx.c:2397-2401` | Pas de miroir low-64K ; 0x8000-0xDFFF = PROM0 |
| DSP_ROM_MAP | Table vecteurs 0xFF80 en PROM1 | `calypso_trx.c:2416-2417`, `:3672-3675` | Servie depuis PDROM en prog (0xE000+) |
| FB_CORRELATOR_PIPELINE | Bug case 0x5 DADST→SFTA non-fixé | `calypso_c54x.c:8232-8262` | Fix déjà implémenté (2026-06-22) — bug clos |
| FB_CORRELATOR_PIPELINE | Fixer DADST nécessaire pour d_fb_det | fix présent + vérité-terrain | Blocage terminal en amont (IMR/DSP déraille) ; framing decode non porteur |
| REGISTERS_REVIEW | dsp-registers pas une property, bin jamais lu | `calypso_mb.c:53/301/341`, `c54x.c:13212` | Property + loader existent ; bin = default |
| REGISTERS_REVIEW | AR0=0xFF75 (off-by-one) | `calypso_c54x.c:13301` | **ALREADY_ACTIONED** : AR0=0x5AAD (2026-05-31) |
| REPORT_CLAUDE_WEB_DSP_REVIVAL | DSP détecte FB : d_fb_det=0x6b34 FOUND | `calypso_trx.c:259` (commentaire), `api_write_cb=` 0 hit | d_fb_det reste 0 ; lecture non reproductible |
| REPORT_CLAUDE_WEB_DSP_REVIVAL | FIX MAJEUR débloque FB→SB (5 SB reads) | `api_write_cb=` 0 hit ; `calypso_trx.c:557-561` read-only | Handshake jamais armé, IMR=0, DSP déraille |
| REPORT_CLAUDE_WEB_IQ_CABLAGE | bsp_buf jamais alimenté par sl->iq (CABLAGE MANQUANT) | `calypso_bsp.c:1107/1243/1248` | sl->iq déjà câblé dans deliver_buffered |
| REPORT_CLAUDE_WEB_IQ_CABLAGE | FIX : stash sl->iq dans last_iq | `calypso_bsp.c:1248` | Fix redondant ; ajouterait 1 frame de latence |
| REPORT_DSP_C54X_CORRELATION | PORTW 0x75 sous-décodé → phantom CC | `calypso_c54x.c:7501` | Handler 3-word inconditionnel déjà présent |
| REPORT_DSP_CORRELATOR_RO | DADST/DSADT sans handler (case 0x5) | `calypso_c54x.c:8232-8305` | Handler ajouté 2026-06-22 ; root-cause obsolète |
| SERCOMM_GATE_ARCHITECTURE | rx_burst appelle bsp_load+BRINT0 | `calypso_trx.c:2166-2182` no-op | Mécanisme dans calypso_bsp.c ; attribution fausse |
| SERCOMM_GATE_ARCHITECTURE | Pas de routage DLCI 4 | `sercomm_gate.c:99/209/137` | DLCI 4 = TRXC intercepté (l'inverse) |
| TODO | Redirect 0xFF80→0x7120 permanent, actif | `calypso_c54x.c:10579-10581` | Gardé derrière CALYPSO_REDIR_LEGACY, OFF par défaut |
| TODO | BSP_BYPASS_BDLENA 2 gardes actives (~755/~825) | `calypso_bsp.c:125/809-811` ; `take_bdl_pulse` en commentaire seul `:1095` | Lu depuis env mais jamais consommé ; hack inerte |
| schematics | DADST case 0x5 bug encore présent | `calypso_c54x.c:8232-8298` | Section historique ; handler dédié présent |
| schematics | Fix = CALYPSO_DSP_FRAME_VEC28 (atteint go-live 0xa582) | `calypso_c54x.c:13550-13556` + vérité-terrain | VEC28 = cul-de-sac, déraille, IMR=0 — retirer « machine-verified » |
| 0x68_0x6F | 0x68xx-0x6Fxx « MISSING » | `calypso_c54x.c:7763-7851` | Tous implémentés (2026-04-28) |
| 0x68_0x6F | Fallback 0x6800 = LD Smem,T (drift) | `calypso_c54x.c:7909-7910` | Dead code depuis 2026-04-28 ; plus de drift |
| 0xF3 | Garder INTR k pour F300-F31F | `calypso_c54x.c:6380-6386/6434` | INTR retiré ; F300-F35F = ALU #lk ; base INTR=0xF7C0 |
| 0xF3 | F310 = 0 sites / INTR probablement correct | `calypso_c54x.c:6439-6442` | F310=SUB était le wedge réel (loop fc50) |
| tic54x_hi8_map | 0x8B = MVDK long-addr (TODO, l.4220) | `calypso_c54x.c:8777-8782` | Neutralisé NOP 1-word ; citation périmée |
| tic54x_hi8_map | 0xDD = POPD Smem (SP runaway, l.4745) | `calypso_c54x.c:9354-9365` | Neutralisé NOP ; SP-runaway fixé |
| ARCHITECTURE | API publique calypso_orch.h (init/publish/steal) | `calypso_orch.h` = flag env seul ; grep = 0 | Symboles inexistants ; réécrire la section |
| ARCHITECTURE | calypso_orch.c + bus UDP 6920 pub/sub | pas de fichier ; grep 6920 = 0 | ORCH node inexistant |
| ARCHITECTURE | l1ctl steal UDP 6921 | grep 6921/steal_init = 0 | Steal server inexistant |
| ARCHITECTURE | DSP publie via api_write_cb → ORCH | `calypso_c54x.c:3357-3358`, `api_write_cb=` 0 | Flèche publish ne se déclenche jamais |
| BRIDGES | trx_dsp_api_write_cb = bridge DSP→ARM live | grep = 0 hit | Fonction inexistante ; pointeur jamais enregistré |
| CALL_FLOW | dsp_api_write_cb intercepte d_fb_det depuis PC 0xe26e | grep = 0 ; 0xe26e = 0 hit | Intercepteur inexistant |
| CALL_FLOW | FBSB appelle publish_fb_found inconditionnellement | `calypso_fbsb.c` gated (orch+synth-env+BSP) | Triple-gardé ; d_fb_det reste 0 |
| CLAUDE | PORTR PA correct=0x0034, 0xF430 faux | `calypso_c54x.c:8609-8613/7508` | Inverse : 0xF430 réel (64 sites), 0x0034 legacy, opcode 0x74 |
| CLAUDE | a_cd[15] à NDB+0x1DC (0x0384) | `calypso_dsp_shunt.c:94-104` | 0x1FC (DWARF) ; 0x1DC marqué faux 2026-06-02 |
| CLAUDE | INTM=1 forever = vieux blocage résolu | `calypso_c54x.c:13537-13546` | Non résolu : stub RETE, vec28 jamais vectorisé, IMR=0 |
| FLOW | Couche 0x04 DSP_API publiée par trx api_write_cb | `calypso_c54x.h:204`, `c54x.c:3357-3358` | Jamais publiée ; le hook est dans c54x, pas trx |
| L1CTL_SOCK_FLOW | API burst-mode (l.380/387, TPU_CTRL_EN) | grep burst_mode = 0 | Section 7 inexistante ; lignes = inject sercomm |
| L1CTL_SOCK_FLOW | accept_cb vm_start/cli_rx_enabled handshake | `l1ctl_sock.c:407-430` | Pas de champ cli_rx_enabled/vm_start |
| L1CTL_SOCK_FLOW | l1ctl_sock_init commenté (l.230, path désactivé) | `calypso_soc.c:334` | Actif/env-gated (L1CTL_SOCK) |
| README | DSP→ARM mirror fixé 2026-05-15, task_md=24, l1s_nb_resp 60+/run | `calypso_c54x.c:3357`, `api_write_cb=` 0 | Non câblé ; cascade bloquée |
| README | FB-det converge (~3 min), FBSB réel, d_fb_det≠0 à 2.9% | vérité-terrain + `calypso_trx.c:~235` | d_fb_det reste 0 ; pas de FBSB non-synth |
| STEP2_BC_CONDS | BC F8xx ne gère que cond 0x00/0x20/0x30 | `calypso_c54x.c:6308-6321` | Group-1 (F84x/F85x) décodé via sub 0x4/0x5 |
| STEP2_BC_CONDS | Fix proposé : ajouter décode group-1 | `calypso_c54x.c:6312/6314/6296` | **ALREADY_ACTIONED** (FIX 2026-06-23) |
| TINT0_IMPLEMENTATION | FRAME_VEC=2/BIT=1 (SINT17) | `calypso_c54x.h:126-127` | vec 19 / bit 3 (INT3) ; item-5 invalide |
| TINT0_IMPLEMENTATION | TIM décrément per-instruction (~2490-2520) + FN inc | `calypso_c54x.c:2482-2530` sondes debug | Aucun code timer per-instruction |
| c_patches | F273 = RETD, 2 handlers dupliqués | `calypso_c54x.c:6032`, RETD=0xFE00 `:1108` | F273 = BD, handler unique — NE PAS appliquer |
| patches | F074 et F274 tous deux corrects | `calypso_c54x.c:5842-5849` vs `6015-6029` | F274 = CALLD (différé 2 slots), conclusion fausse |

---

## 4. Citations périmées (STALE)

`calypso_c54x.c` = 13 697 lignes (docs citent souvent ≈12 442 / 12 646) ; `calypso_trx.c` = 2 481 lignes (docs citent des lignes 3053-3070 inexistantes). 82 citations `file:line` sont décalées. Le **comportement** décrit tient souvent, seule l'**ancre de ligne** est fausse — corriger les ancres, pas la logique.

| Doc | Citation périmée | Ligne réelle |
|---|---|---|
| C54X_DECODER_AUDIT.md | header (line 7); §1 (lines 21-22); §2.2-2.4; §3.x tables; §4.x | Actual: resolve_smem@3728, resolve_xmem@3992, c54x_cond_true@4026, case 0x3@8052, case 0x5@8226, FCxx `s->pc = ra`@7043, vector formula `(iptr*0x80)+v |
| CLEANUP_CANDIDATES.md | §1c (fbsb / full_pcb bullets) | ls -la: calypso_fbsb.c = 5570 bytes, calypso_full_pcb.c = 6637 bytes (both mtime Jul 1 08:58, edited after the doc's stated sizes) |
| FB_CORRELATOR_PIPELINE.md | §2 table col 'handler C' ':7817'; §4 'case 0x5 (calypso_c54x.c:7817-7852)'; §6 | top-level `case 0x5:` is at calypso_c54x.c:8226; line 7817 is inside the BANZD (0x6E00) handler ('uint16_t pre = s->ar[nar];'); SFTA/SFTL fallback is  |
| FB_CORRELATOR_PIPELINE.md | §3, §6 'resolve_smem 3578 ; c54x_circ_ref 3566' | calypso_c54x.c:3728 `static uint16_t resolve_smem(...)`; calypso_c54x.c:3716 `static uint16_t c54x_circ_ref(...)` |
| FB_CORRELATOR_PIPELINE.md | §6 'sonde SHADOW-DADST 11238/11252' | calypso_c54x.c:12189 (SHADOW-DADST pre-capture) and 12342 (post-compute); no SHADOW-DADST text at 11238/11252 |
| FB_CORRELATOR_PIPELINE.md | §2/§3 'LD Smem,T :5114' and '(op&FF00)==0x3000 :5114' | calypso_c54x.c:5302-5303 '/* 0x3000 LD Smem, T ... */ if ((op & 0xFF00) == 0x3000)' |
| FB_CORRELATOR_PIPELINE.md | §4 'cf DST :7807'; §6 'DST Lmem (paire 32-bit) 7807' | DST src,Lmem handler is at calypso_c54x.c:8215-8216 ('Lmem = even-aligned 32-bit pair: mem[L]=high, mem[L+1]=low'); line 7807 is BANZ-handler code |
| FB_CORRELATOR_PIPELINE.md | §3 'SSBX/RSBX C16 (F7B7/F6B7) :6529/6330' | SSBX ST1 handler at calypso_c54x.c:6746-6755 (`s->st1 \|= (1 << bit)`), RSBX ST1 at 6547-6551 (`s->st1 &= ~(1 << bit)`); ST1_C16=(1<<7) confirmed in c |
| FB_CORRELATOR_PIPELINE.md | diagram + §6 'bsp_take_for_fn 272 ; daram_addr/len 109/110' | calypso_bsp.c:302 `static BspBurstSlot *bsp_take_for_fn(...)`; struct fields `daram_addr` at 111, `daram_len` at 112 |
| FB_CORRELATOR_PIPELINE.md | diagram + §6 'FBDET-RD 208 ; FORCE_TOA 291' | calypso_trx.c:258 '=== FBDET-RD ...', d_fb_det read at 262 `offset == 0x01F0` (0x01F0 fact CONFIRMED); FORCE_TOA block at ~320. Cited lines 208/291 do |
| REGISTERS_REVIEW.md | Section 1(b) heading and inline line tags | c54x_reset actually starts at calypso_c54x.c:13240; s->imr=0x52FD at 13333; pc=iptr*0x80 at trx-tail 13501. Values themselves (imr=0x52FD, st0=0x181F, |
| REGISTERS_REVIEW.md | Section 3 P2 | Redirect is at calypso_c54x.c:10581 'if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100)'; default target s->pc=0x7120 (10608) but now has env-gat |
| REGISTERS_REVIEW.md | Section 1(a) | calypso_trx.c:1611-1638 is now calypso_tdma_tick() (TDMA timer), not ROM loading. The register-snapshot load path is calypso_trx.c:2434-2438 (c54x_loa |
| REPORT_CLAUDE_WEB_DSP_CORRELATOR.md | section 'Le landmine documenté dans le code', line citing calypso_c54x.c:7216- | calypso_c54x.c:7464 '0x72/0x73 (MVDM/MVMD) : RESTENT REVERTÉS (fallthrough STL générique)' and gated handler at 7620-7645; lines 7216-7256 are the E4/ |
| REPORT_CLAUDE_WEB_DSP_CORRELATOR.md | section 'Le landmine documenté', bullet 'Histoire liée : 0x86/0x87 ... corrigé | calypso_c54x.c:8713 '0x86/0x87 : STH src, ASM, Smem ... (1-WORD)'; comment 8717 'FIX 2026-06-02 (bug #3, ROOT CAUSE AR3-zero)'; handler at 8733-8741.  |
| REPORT_CLAUDE_WEB_DSP_CORRELATOR.md | section 'La QUESTION' item 1 and 'Fichiers / ancres' (resolve_smem ~3585+) | calypso_c54x.c:3728 'static uint16_t resolve_smem(C54xState *s, uint16_t opcode, bool *indirect)' |
| REPORT_CLAUDE_WEB_DSP_CORRELATOR.md | 'Fichiers / ancres' (c54x_exec_one 3844+) | calypso_c54x.c:4052 'static int c54x_exec_one(C54xState *s)' |
| REPORT_CLAUDE_WEB_DSP_REVIVAL.md | Architecture ASCII diagram, 'propagé à api_ram (c54x.c:3145)' | calypso_c54x.c:3138-3155 is ORPHAN@/POPM-ST0 stack-corruption debug code, not API propagation. The api_ram mirror-on-write is at c54x.c:3314-3315, c54 |
| REPORT_CLAUDE_WEB_DSP_REVIVAL.md | section 'Diagnostic — forks résolus', DMA 0x0586 bullet | calypso_dsp_done is defined at calypso_trx.c:952 (not 704-738); the API-write-page → DARAM 0x0586 copy is at calypso_trx.c:965-1000 (s->dsp->data[0x05 |
| REPORT_CLAUDE_WEB_DSP_REVIVAL.md | section '⭐ FIX MAJEUR', 'calypso_trx.c:1045-1049, CALYPSO_IDLE_PC_LO/HI' | calypso_cpu_idle_park() is at calypso_trx.c:1290-1320; env reads CALYPSO_IDLE_PC_LO/HI at 1296-1297; governor banner at 1301. (Defaults 0x00823000/0x0 |
| REPORT_CLAUDE_WEB_DSP_REVIVAL.md | 'exactement ce que le hack FORCE_TOA masque, calypso_trx.c:204-254' | calypso_trx.c:204-254 is unrelated BL-READ/R_PAGE probes. CALYPSO_FORCE_TOA env gating and the forced a_sync_demod/a_serv_demod overrides are at calyp |
| REPORT_CLAUDE_WEB_DSP_REVIVAL.md | blockquote after 'Chaîne de validation sync' | The 'vrai FB' comment carrying d_fb_det=0x9041 is at /opt/GSM/qemu-src/bash_scripts/run.sh:1665 (CALYPSO_DSP_REG_MODE default), not run.sh:1645 |
| REPORT_CLAUDE_WEB_IQ_CABLAGE.md | Section 'Break B' bullet 'Unique caller' | repo-wide grep: sole call site is calypso_bsp.c:432 calypso_dsp_shunt_feed_iq(_fn,(const int16_t*)(buf+8),...); line 403 is mid-recvfrom, not the call |
| REPORT_DSP_C54X_CORRELATION.md | sec.2 table (0x7706/0x770d), sec.6 rows POPM/PSHM, sec.3.4 | POPM `(op & 0xFF00) == 0x8A00` is at calypso_c54x.c:8484 (L8170 is an RPT-Smem branch); PSHM `op8 == 0x4A` is at calypso_c54x.c:8182. Behaviors (pop1  |
| REPORT_DSP_C54X_CORRELATION.md | sec.2 rows 0x75e8/0x7600/0x7707, sec.6 RCD/RETD/RETD rows, sec.3.4 | The 0xFE RETD/RCD handler `if (hi8 == 0xFE)` is at calypso_c54x.c:7057-7112 (pop `ra=data_read(sp); sp++`, delay_slots=2 at ~7101). L6918-6974 is actu |
| REPORT_DSP_C54X_CORRELATION.md | sec.2 rows 0xb418/0x7702, sec.6 CC/CALLD rows | Near-CC handler `if (hi8 == 0xF9)` with push PC+2 is at calypso_c54x.c:6824-6875 (`data_write(s, s->sp, (uint16_t)(s->pc + 2))` at ~6855). CALLD `if ( |
| REPORT_DSP_C54X_CORRELATION.md | sec.4 row 0xb41f | BC handler `if (hi8 == 0xF8)` is at calypso_c54x.c:6212; a FIX 2026-06-23 at calypso_c54x.c:6237-6252 now uses proper TC-based NTC/TC when the previou |
| REPORT_DSP_C54X_CORRELATION.md | sec.2 rows 0x75fd/0x75ff/0x7708/0x76ff, sec.3.3 | F5xx SSBX/RPT block `/* F5xx: SSBX or RPT #k */` is at calypso_c54x.c:6717-6719 (not 6578-6593). The nibble-switch line anchors (case 0x1 L7611, case  |
| REPORT_DSP_CORRELATOR_RO.md | section 1, section 3a table, synthesis ('c54x.c:7818, branches 7838/7844') | calypso_c54x.c:8225 'case 0x5:'; line 7818 is actually the BANZD 0x6E00 handler; the SFTA/SFTL sub-decode sits at 8306-8331 |
| REPORT_DSP_CORRELATOR_RO.md | section 2 | calypso_c54x.c:9015 'if (hi8 == 0xA0)'; SFTL branch 'else if (sub & 0x80)' at 9048 (shift=(sub>>1)&0x3F sign-extended). Behavior for 0xDA (SFTL A,-19) |
| REPORT_DSP_CORRELATOR_RO.md | section 2, section 4 table (0xC8-CB row), R2 | calypso_c54x.c:9404 'if (hi8 >= 0xC8 && hi8 <= 0xCB)' (comment at 9376 '0xC8/C9/CA/CB: ST SRC,Ymem \|\| LD Xmem,DST'). Decode-correct claim holds; cit |
| REPORT_DSP_CORRELATOR_RO.md | section 4 (CORRECT decode bullet) | calypso_c54x.c:3992 'static uint16_t resolve_xmem(C54xState *s, uint16_t op)'; line 3804 is unrelated |
| REPORT_DSP_CORRELATOR_RO.md | section 3 header | calypso_c54x.c:3663 'static inline uint32_t c54x_prog_xlate(...)' |
| REPORT_DSP_CORRELATOR_RO.md | section 3b/4 tables, section 2 | MAC 0x90-93 at 8342; 0xA4-A7/B0-B7 MAC at 8845; SQDST hi8==0xA1 at 8909; hi8==0x80 STL at 8533; 0x3000 LD Smem,T at 5303 — all present but ~+250-540 l |
| REPORT_DSP_CORRELATOR_RO.md | section 7 item 7, Shunt-safety note | calypso_bsp.c: calypso_dsp_shunt_active() at 404, 426... wait 426/432 are feed calls; gates at 462, 956, 1112 — cited 922/1077/1080 do not match (actu |
| doc_master.md | Section 'Rappel des leviers de fix', lever #3 | calypso_trx.c is only 2481 lines (wc -l), so lines 3053-3070 do not exist; the 0x0314/0x0318 go-live gating (HS-ARM-GATE) is actually at calypso_trx.c |
| doc_master.md | Section 'Rappel des leviers de fix', lever #3 | The api_write_cb guard/call is at calypso_c54x.c:3357-3358 ('if (s->api_write_cb) s->api_write_cb(...)'), not 3355 |
| schematics.md | Légende D; GAP table row D ('calypso_trx.c:3053-3070 + notify c54x.c:3355') | calypso_trx.c is only 2481 lines (wc -l) so 3053-3070 does not exist; actual HS-ARM-GATE probe is at calypso_trx.c:553-566, with the 0x0314/0x0318 gat |
| schematics.md | Memory-map table (C), row 'Reset vector' | calypso_c54x.c:13618 is 's->pc = (iptr * 0x80) + vec * 4;' — the generic interrupt-vector formula inside the IDLE branch of c54x_interrupt_ex, not res |
| 0xF3.md | 'Why this exists' section, code block cited as 'lines 1905-1932' | calypso_c54x.c:1900-1940 is data_read watch code (WAIT-3DD0 / d_fb_det[0x08F8]); the real F3 dispatch is at calypso_c54x.c:6379 'if (hi8 == 0xF3)' |
| tic54x_hi8_map.md | hi8 table row 0xC8..0xCB | actual C8/C9/CA/CB decode at calypso_c54x.c:9376 `/* 0xC8/C9/CA/CB: ST SRC, Ymem \|\| LD Xmem, DST ... */`; line 4773 is an RCD-75e8 tracer (`if (s->p |
| ARCHITECTURE.md | Section 2 sequence (TRX->>ARM: qemu_irq_pulse(IRQ_API)) | calypso_trx.c:1060 and :1809 use qemu_irq_raise(s->irqs[CALYPSO_IRQ_API]); no qemu_irq_pulse and the enum is CALYPSO_IRQ_API |
| BRIDGES.md | Section 3 calypso_soc.c bullet 'ligne 234' | calypso_soc.c:333-334: 'const char *l1ctl_path = getenv("L1CTL_SOCK"); l1ctl_sock_init(&s->uart_modem, l1ctl_path ? l1ctl_path : "/tmp/osmocom_l2");' |
| BRIDGES.md | Section 3 calypso_uart.c bullet '(ligne 477)' | hw/char/calypso_uart.c:703 is the l1ctl_sock_uart_tx_byte(ch) call; line 477 is a comment block about the romloader stub |
| BUGS_AND_FIXES.md | Bug 17 Symptom/Status | grep 'hi8 == 0xF0 \|\| hi8 == 0xF1' calypso_c54x.c → single match at 5807 (no duplicate block remains) |
| CLAUDE.md | UL pipeline as of 2026-05-07 — 'RACH path uses gsm0503_rach_ext_encode (libosm | calypso_bsp.c:1550 'int rc = gsm0503_rach_ext_encode(bits, ra, uic_or_bsic, false);'; calypso_trx.c:1888 only comment 'Burst encoding (gsm0503_rach_ex |
| CLAUDE.md | Key Files | wc -l calypso_c54x.c = 13697 lines |
| FBSB_FLOW.md | Section 4, closing sentence | calypso_trx.c:889 is the actual call to calypso_fbsb_on_dsp_task_change (inside the d_task_md write handler, guarded at trx.c:886). Lines 169-171 are  |
| FBSB_FLOW.md | Section 6, closing note | calypso_fbsb.c:152 is the dump() NOTE comment ('les vraies valeurs FB... le synth host est mort'); it contains no 'pm << 3'. The pm>>3 comments live a |
| FLOW.md | line 102 (Layers table) | calypso_fbsb.c:47 defines only calypso_fbsb_publish_fb_found; header comment calypso_fbsb.c:4-5 states publish_sb_found (and other host-side synth) we |
| L1CTL_SOCK_FLOW.md | Section 8b | Hook is at calypso_uart.c:702-704 (label 'modem' gate correct), but preceding call is qemu_chr_fe_write(&s->chr,&ch,1) at calypso_uart.c:697 (non-bloc |
| L1CTL_SOCK_FLOW.md | Sections 3,4,5,6 headers | Actual: l1ctl_sock_init=436, l1ctl_accept_cb=407, l1ctl_sock_uart_tx_byte=257, sercomm_frame_complete=189, l1ctl_client_readable=296 (all shifted; eve |
| MTTCG_AUDIT.md | Status par fichier > calypso_c54x.c table rows | calypso_c54x.c has 13697 lines; sed -n of 498/566/657/1139/1141 shows NO api_ram (trace/decode/dump code). Actual `s->api_ram[woff]=val` is at calypso |
| MTTCG_AUDIT.md | Status par fichier > calypso_trx.c table + Phases (C2 pending) | api_ram_lock IS already taken in trx.c: qemu_mutex_lock(&calypso_pcb_api_ram_lock) at calypso_trx.c:996 (DMA-page mirror into DSP DARAM, under daram<a |
| REACTIVER_DSP_REEL.md | §2.1 line 84 ("Chargement ROM : c54x_load_rom"); §7 line 255 ("cœur, c54x_run, | grep -rn c54x_load_rom over the whole tree hits ONLY docs + python_scripts/dsp_txt2bin.py — zero matches in any .c/.h. Actual ROM/section loaders are  |
| REACTIVER_DSP_REEL.md | §2.5 ("c54x_bsp_load(bsp.dsp, samples, ns) (calypso_bsp.c:949, 1148)"); §7 lin | calypso_bsp.c:949 is BSP-RXBURST debug logging; :1148 is `c54x_interrupt_ex(bsp.dsp,19,3) /* INT3 (frame) */`. The actual c54x_bsp_load call sites are |
| REACTIVER_DSP_REEL.md | §2.4 ("CALYPSO_DSP_SHUNT=0\|1 (run.sh:85)"); §7 line 250 ("run.sh:85 ... :835" | bash_scripts/run.sh:85 is a c54x-probes --debug-full help line; the CALYPSO_DSP_SHUNT help/toggle is at run.sh:105, mode defaults at 1142/1156/1186, a |
| REACTIVER_DSP_REEL.md | §7 line 250 | calypso_dsp_shunt.c:121 is `#define TCHT_DSP_TASK 13`; lines 1485-1535 are the SHM shm_open/mmap + .cfile I/Q recorder setup. Neither is toggle/skip l |
| REACTIVER_DSP_REEL.md | TL;DR line ~20 and §2.1 ("~12 442 lignes"); V5 | wc -l calypso_c54x.c = 13697 lines. |
| REACTIVER_DSP_REEL.md | TL;DR line ~13 | wc -l /opt/GSM/calypso_dsp.txt = 8216 (symlink → qemu-src/calypso_dsp.txt). |
| README.md | '### Fix' code block, 'hw/arm/calypso/calypso_trx.c:163' | calypso_trx.c:163 is inside dsp_real_rom_mode() ('static int v = -1;'). The actual 'FIX 2026-05-15' block is at calypso_trx.c:229-257, and it is an if |
| REFACTORING.md | What Still Needs Refactoring #4 | calypso_sim.c already exists (794 lines, 'ISO 7816 / GSM 11.11 SIM emulator') and is in meson.build; trx.c retains only 17 residual SIM refs |
| REPORT_CLAUDE_WEB_PIPELINE.md | section 4, lines 87-90 | qemu_wrap.c:1063-1140 — current design: hot-path pushes into a per-FIFO ring and DROPS at ring level; a dedicated writer thread per FIFO opens O_WRONL |
| REPORT_CLAUDE_WEB_PIPELINE.md | section 4 table, lines 88-99 | calypso.env:35 `CALYPSO_RELAY_FIFOS=/tmp/iq_fft.fifo:/tmp/iq_grgsm.fifo:/tmp/iq_grgsm_ciph.fifo:/tmp/iq_record.fifo:/tmp/iq_asciifft.fifo` (5 FIFOs);  |
| STEP1_PROM_LOADER.md | 'Cause analysée' section: `calypso_c54x.c:4248-4324` (`c54x_load_rom`) | calypso_c54x.c:4052 c54x_exec_one() spans 4248-4324 (INTM/ROM->DARAM/ENTER-7700 debug tracers, NOT a loader). No symbol c54x_load_rom exists. Actual l |
| THREADING_TODO.md | Phase 2 / 2.a "État courant" | calypso_tdma_tick defined at calypso_trx.c:1618; the two budget calls are at calypso_trx.c:1713 (dsp_n_exec_2) and 1799 (dsp_n_exec_5), not L730/L777. |
| TINT0_IMPLEMENTATION.md | Section 'Docker' | Live container is osmo-operator-1; code lives under /opt/GSM/qemu-calypso/hw/arm/calypso/ (ls confirms files present there). /opt/GSM/qemu-src is an a |
| TODO_2026-05-25_NIGHT.md | Section 4, 'BRINT0 edge-trigger gate dans calypso_bsp.c L638' | calypso_bsp.c:638 is `bsp_enqueue(tn, fn, iq, iq_count);` (a burst-enqueue call, no IFR test). The actual gate `if (bsp.dsp && !(bsp.dsp->ifr & (1 <<  |
| c_patches.md | C-2 (Patches CONFIRMED) | calypso_c54x.c: only one exec handler `if (op == 0xF274)` at :6015 (grep shows 1505 is a name-lookup, 6015 is the sole handler); cited lines 1473-1481 |
| c_patches.md | C-3 (Patches CONFIRMED) | calypso_c54x.c: single exec handler `if (op == 0xF272)` at :6002 (grep 'if (op == 0xF272)' returns exactly one hit); line 1461 is a typedef struct, no |
| c_patches.md | S-1 (Patches SPECULATIVE), hypothesis 2 | calypso_c54x.c:13022 `if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1)` — actual wrap check, uses `>=` not `==`, at line 13022 not 3842 |
| patches.md | section 1.2 'Lignes 1311-1317 et 1473-1481' | Real execution handlers: F074 at calypso_c54x.c:5842, F274 at calypso_c54x.c:6015. Lines 1483-1487 are struct/global defs ('static AWriteLog g_awrite_ |
| patches.md | section 1.2 hypothèse b | calypso_c54x.c:13022 — 'if (s->rptb_active && !s->rpt_active && s->pc >= s->rea + 1)'. Operator is '>=', not '=='. Line 3842 is a bare 'break;'. '>='  |
| BOOT_TO_FBSB_SEQUENCE.md | Phase 0 code block + Items bloquants table row #1 | calypso_c54x.c:10579-10581 redir_legacy = getenv("CALYPSO_REDIR_LEGACY")?1:0; if (redir_legacy && s->pc==0xFF80 && s->sp==0x1100){...} |
| REVERT_MVMD_KNOWLEDGE.md | Analyse sémantique / État conservé après revert ('ligne 3999', cited twice) | calypso_c54x.c:7650 `if ((op & 0xF800) == 0x7000) { /* 70xx: STL src, Smem */`; line 3990-4010 is now resolve_xmem(), not STL |
| REVERT_MVMD_KNOWLEDGE.md | Analyse / État conservé après revert | lines 3020-3145 now contain DISP-FLAG-W and ORPHAN@ stack-probe code, not F3 decode; F3 handling lives elsewhere (calypso_c54x.c:4227 `if (hi8 == 0xF3 |
| REVERT_MVMD_KNOWLEDGE.md | État conservé après revert | calypso_c54x.c:6247-6251 — a 2026-06-23 change makes it conditional: `if ((g_prev_op & 0xFE00) == 0x6000) { tc = ...; take = (sub==0x2)?!tc:tc; } else |
| SESSION_2026-05-29.md | Fix #2 header + line '### 2. SILICON-BOOT-REDIRECT removal — `calypso_c54x.c:9 | calypso_c54x.c:10542 '=== SILICON-BOOT-ROM REDIRECT (réactivé 2026-05-30) ==='; :10569 '=== REDIRECT NEUTRALISÉ PAR DÉFAUT (2026-05-31) ==='; :10580 ' |
| SESSION_2026-05-29.md | Fix #1 header + code block adding '&& op != 0xF4E1' | calypso_c54x.c:5022-5023 'if (op >= 0xF4E0 && op <= 0xF4FF && op != 0xF4E1 && op != 0xF4E4 && op != 0xF4EB) return ...'; IDLE handler at :5079-5080 '( |
| SESSION_20260403.md | ## Files modified — 'calypso_c54x.c line 810: 0xEA BANZ -> LD #k9,DP' | Handler is actually at calypso_c54x.c:7145 (grep `0xEA00` -> 7145); line 810 is inside an unrelated AR/accumulator trace-print block |
| SESSION_20260403.md | ## Files modified — 'calypso_trx.c line 135 ... line 292' | calypso_trx.c:135 is now a talloc-pool comment; :292 is a `case 0x01F4: tag="FB a_sync_demod[D_TOA]"` switch. Actual per-tick DSP run is c54x_run at t |
| SESSION_20260405_NIGHT4.md | Fix 16 (IDLE/FRET table) | calypso_c54x.c:5023-5024 range guard 'op>=0xF4E0 && op<=0xF4FF && op!=0xF4E1 && op!=0xF4E4 && op!=0xF4EB' returns (swallowed as NOP) — F4E5 is NOT exc |

---

## 5. Verdict global et priorités de correction

L'audit confirme un divorce systématique doc ↔ code sur trois axes : (1) des bugs de décodage DSP présentés comme ouverts sont **déjà corrigés** (famille A) ; (2) plusieurs docs racontent une **détection FB / cascade débloquée** qui n'existe pas et s'appuient sur une infrastructure (`api_write_cb`, bus ORCH 6920/6921, API steal/burst-mode) **absente du code** (famille B) ; (3) des références d'encodage/registre/vecteur sont **factuellement inversées** (famille C) ; (4) la quasi-totalité des ancres de ligne ont **dérivé** (famille D).

### Ordre de correction (du plus trompeur au moins)

1. **ARCHITECTURE.md, BRIDGES.md, CALL_FLOW.md, FLOW.md** — décrivent une API/bus/steal-path **fabriqués**. À réécrire/supprimer en priorité : ils envoient tout lecteur vers des symboles inexistants.
2. **README.md, REPORT_CLAUDE_WEB_DSP_REVIVAL.md** — affirment FB détecté / cascade débloquée / d_fb_det≠0. Contredits par la vérité-terrain (d_fb_det=0, DSP déraille, IMR=0). Corriger le narratif « résolu » en « toujours bloqué ».
3. **L1CTL_SOCK_FLOW.md** — 3 sections entières (burst-mode, handshake vm_start, init commenté) décrivent du code absent.
4. **CLAUDE.md** — sert de mémoire de projet : ses erreurs inversées (PORTR 0xF430 vs 0x0034, a_cd 0x1DC vs 0x1FC, INTM « résolu ») se propagent à chaque session. À corriger tôt.
5. **schematics.md ET doc_master.md** (générés cette session) — ⚠️ portent **encore** le cadrage « **VEC28 = le fix** » (« une seule vectorisation atteint go-live 0xa582 qui arme IMR »), que la suite de la session a **réfuté** : VEC28 est un cul-de-sac (`calypso_c54x.c:13550-13556` existe mais déraille vers boot-stub, IMR=0x0000). Ces deux docs doivent être re-générés/annotés en tête pour retirer la conclusion VEC28.
6. **Docs de décodage** (0x68_0x6F, 0xF3, tic54x_hi8_map, C54X_INSTRUCTIONS, FB_CORRELATOR_PIPELINE, REPORT_DSP_CORRELATOR_RO, REPORT_DSP_C54X_CORRELATION, c_patches, patches) — reclasser les bugs « ouverts » en « fixé » et retirer les patchs qui rejoueraient des correctifs existants (risque de régression).
7. **CALYPSO_HW.md, TINT0_IMPLEMENTATION.md, DSP_ROM_MAP.md, SERCOMM_GATE_ARCHITECTURE.md** — corriger les faits d'encodage/vecteur/mémoire.
8. **TODO.md, CLEANUP_CANDIDATES.md, REGISTERS_REVIEW.md, STEP2_BC_CONDS.md** — marquer les items `ALREADY_ACTIONED` comme DONE (calypso_dbg.c supprimé, AR0=0x5AAD, BC group-1, dsp-registers).

### Point d'attention méthodologique

La cause racine terminale **n'est pas** un bug de décodage DSP (tous les root-causes decode cités ont été corrigés). Elle est **en amont** : `IMR` n'est jamais ré-armé après le clear de boot, `api_write_cb` n'est jamais enregistré, le handshake go-live ARM→DSP n'écrit que 0x0000, et le DSP déraille vers le boot-stub. Tant que la doc pointe le décodage comme blocage, elle détourne l'effort du vrai mur.
