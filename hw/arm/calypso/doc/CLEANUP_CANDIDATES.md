# CLEANUP_CANDIDATES — hw/arm/calypso

> RAPPORT SEULEMENT. Aucune suppression, aucune édition n'a été faite par ce
> passage. Chaque candidat porte un tag de risque et une preuve (référence
> build ou compte `grep`). Généré le 2026-07-01.
>
> **Portée** : principalement `hw/arm/calypso` (sous-système actif). Pour
> l'arbre `qemu-src` complet, seuls les artefacts temporaires/morts évidents
> de premier niveau sont signalés — **un balayage exhaustif de tout l'arbre
> qemu-src est un chantier séparé de grande ampleur et n'a PAS été tenté ici.**

Tags : `SAFE-DELETE` / `ARCHIVE` / `NEEDS-REVIEW` / `KEEP`.

---

## 1) Fichiers source morts / doublons dans hw/arm/calypso

### 1a. `calypso_dbg.c` — ✅ FAIT (SAFE-DELETE effectué 2026-07-01)
**Ce candidat a été traité.** Le fichier `calypso_dbg.c`, son header et toute
trace build ont été retirés cette session (SAFE-DELETE réalisé). Vérification :
`find /opt/GSM/qemu-calypso -name 'calypso_dbg*'` = **aucun résultat** ;
`grep calypso_dbg_init|calypso_dbg_mask` sur `hw/…` + `include/…` = **0 hit** ;
`meson.build` ne liste plus que `fw_console.c` (:23) et `calypso_debug.c` (:26),
**aucune ligne `calypso_dbg.c`**. Rien d'autre ne casse (confirmé).

Historique (raison de la suppression, conservée pour mémoire) :
- Système de debug **superseded**, distinct et non branché. `calypso_dbg_init`
  et `calypso_dbg_mask` n'étaient référencés que dans `calypso_dbg.c` lui-même ;
  le header n'était inclus que par lui ; la macro `DBG()` n'avait aucun appelant.
- Le vrai système de debug actif est `calypso_debug.c` (voir 1c) via
  `calypso_debug_enabled()` / macro `C54_DBG(...)` : 80 sites `calypso_debug_enabled`
  et 19 `C54_DBG` rien que dans `calypso_c54x.c`.
- `calypso_dbg.c` n'était PAS un doublon fonctionnel de `calypso_debug.c` :
  c'était un ancêtre mort (catégories `DBG_BSP`/`DBG_FB`/… via `CALYPSO_DBG`) que
  le système `CALYPSO_DEBUG` avait remplacé — zéro effet runtime.
- ~~À faire : supprimer le `.c`, le header et la ligne meson.~~ **DONE.**

### 1b. `fw_console.c` (2738 o) — NEEDS-REVIEW (mort, diagnostic uniquement)
Poller diagnostique du printf_buffer firmware, **jamais initialisé**.
- `fw_console_init` (seul point d'entrée public, `fw_console.c:63`) n'a **aucun
  appelant** dans tout l'arbre `.c` ; unique mention externe = un **commentaire**
  `calypso_trx.c:130`. `fw_console_emit`/`fw_console_poll` sont `static`, appelés
  seulement en interne → chaîne morte tant que `_init` n'est pas câblé.
- Compilé (meson.build). Ouvre `/tmp/qemu-fw-console.log` (`fw_console.c:65`) si
  jamais armé. NEEDS-REVIEW plutôt que SAFE-DELETE : outil de diagnostic qu'on
  peut vouloir re-brancher ; sinon SAFE-DELETE + ligne meson.

### 1c. Fichiers vérifiés ATTEIGNABLES — KEEP (ne pas toucher)
Preuves de câblage réel (grep symboles hors fichier d'origine) :
- `calypso_debug.c` — **système debug ACTIF**. `calypso_debug_enabled()` utilisé
  partout (80× dans `calypso_c54x.c`). KEEP.
- `calypso_fbsb.c` (5570 o) — `calypso_fbsb_init` / `calypso_fbsb_on_dsp_task_change`
  appelés `calypso_trx.c:881,889`. KEEP.
- `calypso_full_pcb.c` (6637 o) — symbole `calypso_full_pcb` référencé par 8
  fichiers (c54x, trx, fbsb, soc, bsp, .h). KEEP.
- `calypso_sim.c` (31129 o) — `calypso_sim_new` appelé depuis `calypso_trx.c`. KEEP.
- `calypso_iota.c` — `calypso_iota_take_bdl_pulse` appelé `calypso_bsp.c`. KEEP.
- `calypso_tint0.c` — `calypso_tint0*` référencé par trx/soc/bsp/.h. KEEP.
- Tous les autres `.c` listés dans meson.build (mb, soc, trx, c54x, layer1, bsp,
  twl3025, l1ctl_sock, sercomm_gate) sont le cœur actif. KEEP.

---

## 2) Sondes diagnostiques jetables DANS les gros .c

`calypso_c54x.c` = 13697 lignes, **313 `fprintf`**, 80 `calypso_debug_enabled`,
45 `getenv`. La doc (`C54X_DECODER_AUDIT.md`, `TODO.md`) note que ~70 % du
fichier est de la sonde env-/PC-gatée — inerte au runtime mais du bruit lourd.

**Jetables** (liste explicite `TODO.md` « Nettoyage… jetables », compte `grep -c`
dans `calypso_c54x.c`) :

| Sonde        | Occurrences | Tag         |
|--------------|-------------|-------------|
| DETTRACE     | 4           | SAFE-DELETE |
| BACC-DISP    | 2           | SAFE-DELETE |
| DISPVAL-WR   | 2           | SAFE-DELETE |
| DISP-ENTRY   | 12          | SAFE-DELETE |
| SEED-WR      | 2           | SAFE-DELETE |
| AR3-TRIP     | 3           | SAFE-DELETE |
| AR0-TRACE    | 2           | SAFE-DELETE |
| B3D1-CTX     | 1           | SAFE-DELETE |
| DERAIL-*     | 8           | SAFE-DELETE |

Tag global du bloc de sondes : **SAFE-DELETE mais À FAIRE APRÈS le vert**
(`TODO.md` : « Stripper les sondes… plus tard, follow-up commit »). Tant que
`d_fb_det==0` ces sondes servent au diagnostic ; les retirer maintenant
aveuglerait l'investigation en cours. NEEDS-REVIEW pour le tri fin des ~313
`fprintf` restants (certains sont des logs d'état utiles, pas des sondes forensic).

**Keepers — NE PAS confondre avec des sondes** :
- `MVKD` (11) — correctif ISA (fix longueur 2-mot MVKD 0x70, `TODO.md`). KEEP.
- `READA` (7)  — correctif ISA (`rpt_fresh`, `TODO.md`). KEEP.

---

## 3) Artefacts `.bak` / `.tmp` dans hw/arm/calypso — SAFE-DELETE (~2.2 Mo)

Non référencés par meson.build (le build ne liste que les `.c` réels) → non
compilés, non atteignables. 14 fichiers, ~2.2 Mo cumulés.

`.tmp` (temp d'éditeur) :
- `calypso_c54x.c.tmp.809213.46d15be4d9a6` (617 067 o) — SAFE-DELETE.

`.bak*` (snapshots manuels pré-refactor) :
- `calypso_c54x.c.bak` (71 927), `.bak.20260425` (159 910),
  `.bak.preF47generic.20260428` (208 821), `.bak.preF7Bx.20260428` (205 707),
  `.bak.preF7Stub.20260428` (207 055), `.bak.preFixA2A3` (181 436),
  `.bak.preFretFix` (163 493), `.bak.preNoCell` (153 475),
  `.bak.preRcdFix` (161 848) — 9 fichiers.
- `calypso_trx.c.bak` (22 538), `calypso_trx.c.bak.preNoCell` (43 197).
- `calypso_bsp.c.bak.preStrictMatch.20260428` (20 684).
- `l1ctl_sock.c.bak.preNoInject` (10 107).

Tag : **SAFE-DELETE** (aucune référence build/source). Réserve : si l'arbre est
sous git ailleurs, l'historique couvre déjà ces snapshots → suppression sans perte.

---

## 4) Documentation superseded — ARCHIVE (ne pas supprimer)

Les rapports frais du 1er-juillet (`C54X_DECODER_AUDIT.md`,
`FB_CORRELATOR_PIPELINE.md`, `REPORT_CLAUDE_WEB_*.md`, `REPORT_DSP_*.md`,
`TODO.md`) sont l'état courant → KEEP.

Candidats **ARCHIVE** (historiques, antérieurs, valeur mémoire) — déplacer vers
`doc/archive/`, **ne pas supprimer** :
- `SESSION_2026-05-29.md`, `SESSION_20260403.md`, `SESSION_20260405_NIGHT4.md`,
  `SESSION_20260429.md` — journaux de session datés.
- `STATUS.md` — snapshot d'état antérieur aux rapports du 1er-juillet.
- `SP_CATASTROPHE_70c4_SEQUENCE.md` — enquête SP résolue (`TODO.md` : « root SP
  résolu… wedge 0x70c3 mort »).
- `BOOT_TO_FBSB_SEQUENCE.md`, `FBSB_SEQUENCE_TRACE.md`, `REVERT_MVMD_KNOWLEDGE.md`
  — savoir historique, potentiellement redondant avec les rapports frais.

> Note fiabilité mtime : beaucoup de `.md` partagent l'horodatage 2026-06-20
> 21:17 (copie/build en masse), donc la date fichier n'est **pas** un critère
> d'ancienneté fiable — le tri ARCHIVE ci-dessus est fondé sur le contenu/rôle,
> pas sur le mtime.

---

## 5) Artefacts temp/build/log référencés par le code

Sorties runtime régénérées à chaque run → SAFE-DELETE (jamais lues en entrée) :
- `/tmp/iq_rx_%03u.bin` (dump gate `calypso_bsp.c:1051-1064`, ≤24, env/diag) et
  `/tmp/iq_dlv_%03u.bin` (`calypso_bsp.c:1271-1285`, ≤24). Présents :
  `/tmp/iq_dlv_000..003.bin` (592 o chacun). SAFE-DELETE.
- `/tmp/qemu-fw-console.log` (`fw_console.c:65`), `/tmp/frame_irq.log`
  (`calypso_trx.c:1943`), `/tmp/tdma_tick.log` (`calypso_trx.c:2040`),
  `/tmp/tdma_profile.log` (`calypso_trx.c:2056`). SAFE-DELETE.
- `/tmp/bsp_rx.dump` — chemin par défaut du replay `BSP_DUMP_RX_FILE` /
  `CALYPSO_BSP_REPLAY_FILE` (`calypso_bsp.c:191-192`). SAFE-DELETE si non utilisé
  comme entrée de replay ; sinon NEEDS-REVIEW.

Logs runtime hors-source (SAFE-DELETE, régénérés) :
- `/root/qemu.log` (10 Mo), `/root/mobile.log` (13 Mo), `/root/osmocon.log`
  (13 Mo), `/root/calypso-ipc-device.log`, `/root/osmo-trx-ipc.log`,
  `/root/bts.log`, anciens `osmocon.2026*.log`.

**qemu-src premier niveau** (signalé, PAS un balayage exhaustif) :
- `*.log` à la racine `qemu-src` : `bridge.log` (1.97 Mo), `qemu.log` (1.8 Mo),
  `pc_hist.log` (1.79 Mo), `osmocon.log` (2.2 Mo), `qemu_diag.log`,
  `qemu_tail.log`, `env_boot.log`, `frame_irq.log`, `tdma_profile.log`,
  `mobile.log` — artefacts de trace, SAFE-DELETE.
- Total `.bak*/.tmp*` dans tout l'arbre = 17 fichiers (dont 14 sous
  `hw/arm/calypso`, cf. §3) → les 3 restants hors-calypso relèvent du
  **balayage full-tree séparé** non traité ici.

---

## Récapitulatif par tag

- **SAFE-DELETE** : ~~`calypso_dbg.c`(+header+ligne meson)~~ **✅ FAIT 2026-07-01** ; 13 `.bak` + 1 `.tmp`
  calypso ; bloc sondes jetables (9 familles : DETTRACE/BACC-DISP/DISPVAL-WR/
  DISP-ENTRY/SEED-WR/AR3-TRIP/AR0-TRACE/B3D1-CTX/DERAIL-*) — **après le vert** ;
  dumps `/tmp/iq_*.bin` + logs `/tmp/*.log` ; logs runtime `/root/*.log` et
  qemu-src `*.log` premier niveau.
- **ARCHIVE** : 4 `SESSION_*.md`, `STATUS.md`, `SP_CATASTROPHE_70c4_SEQUENCE.md`,
  `BOOT_TO_FBSB_SEQUENCE.md`, `FBSB_SEQUENCE_TRACE.md`, `REVERT_MVMD_KNOWLEDGE.md`.
- **NEEDS-REVIEW** : `fw_console.c` (mort mais outil diag) ; tri fin des ~313
  `fprintf` de `calypso_c54x.c` ; `/tmp/bsp_rx.dump` (entrée replay possible).
- **KEEP** : `calypso_debug.c`, `calypso_fbsb.c`, `calypso_full_pcb.c`,
  `calypso_sim.c`, `calypso_iota.c`, `calypso_tint0.c` + cœur actif meson ;
  correctifs ISA `MVKD`/`READA` ; rapports du 1er-juillet.

---

## NE PAS supprimer sans validation

Aucun élément de ce rapport ne doit être supprimé sans validation explicite.
En particulier :

1. **Le bloc de sondes diagnostiques est le seul instrument d'enquête actif**
   tant que `d_fb_det==0` (blocage terminal : IMR=0x0000 tout le run,
   `calypso_c54x.c:13593`, IRQ frame masquée `calypso_trx.c:1786`). Les retirer
   avant d'atteindre le « vert » (d_fb_det!=0) aveuglerait le diagnostic —
   ordre imposé par `TODO.md` : fix d'abord, strip ensuite.
2. **`fw_console.c`** est mort mais compile proprement ; confirmer qu'aucun
   câblage futur planifié ne le vise avant retrait, et ajuster meson.build dans
   le même commit. (`calypso_dbg.c` : déjà retiré cette session, voir §1a.)
3. **Les `.md` de session/STATUS** contiennent le savoir historique
   (SP/wedge/reverts) : **ARCHIVE, jamais DELETE**.
4. **Le balayage complet de qemu-src** (hors `hw/arm/calypso`) est un chantier
   distinct : les artefacts premier-niveau ci-dessus ne présument pas de la
   propreté du reste de l'arbre.
