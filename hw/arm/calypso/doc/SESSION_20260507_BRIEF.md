# Brief Claude web — 2026-05-07

> Session de cleanup + UL wiring. Coller en intégralité dans Claude web.

## Contexte

QEMU Calypso baseband emulator (ARM7TDMI + TMS320C54x DSP, layer1.highram.elf
osmocom-bb). Bridge Python relaie BTS UDP (5700-5702) ↔ QEMU BSP (6702).
DL milestone validé 2026-05-06 : `cell_identity 777→888` dans `osmo-bsc.cfg`
propage RSL → BTS → bridge → BSP DMA → DSP demod → ARM L1 → mobile L3
(`<0001> sysinfo.c New SYSTEM INFORMATION 3 (lai=001-01-1)` + CGI=001-01-1-888).
Deux observations indépendantes : `rsl_si_tap` mmap (octets `03 78`) +
mobile L3 logs.

À l'audit du 07/05, le commit qui scellait ce milestone (63b4fe5) embarquait
trois nuisances dans le diff :
1. 36 lignes BOURRIN-FBDET-SKIP dans `calypso_c54x.c` — pop stack+jump bypass
   du range PC `[0x8d00, 0x8f80]` pour court-circuiter la routine DSP fb-det.
2. ~120 lignes DIAG-HACK env-gated INTM force-clear + ALIAS-CHECK dump.
3. 23 fichiers `__pycache__/*.pyc` recompilés (bruit sans hash diff).

Plus, sans être dans 63b4fe5 mais dans le code actif :
4. `publish_fb_found(toa=0,pm=80,angle=0,snr=100)` synthétique dans
   `calypso_fbsb_on_dsp_task_change` (DSP_TASK_FB).
5. `publish_sb_found(bsic=0)` synthétique idem.
6. `si3_fallback[23]` hardcodé en cas d'absence de mmap (cold-start).
7. `allc_burst_idx` static cycle 0..3 — désynchronise sous jitter TDMA.
8. UL pipeline coté QEMU branché sur `d_task_u` uniquement (= SDCCH/SACCH/TCH),
   ignorant `d_task_ra` (RACH au word 7 du write-page) — cf. `prim_rach.c:77`.
9. BSP `trxd_peer_valid=false` jusqu'à premier DL, et bridge.py `trxd_remote=None`
   idem côté UL forward → race au démarrage.

## Ce que j'ai fait

### Removed (no more hacks)

| Hack | Fichier | Status |
|---|---|---|
| BOURRIN-FBDET-SKIP | `calypso_c54x.c` | bloc supprimé entièrement |
| DIAG-HACK INTM force-clear + ALIAS-CHECK + BOOT+100k VECDUMP | `calypso_c54x.c` | bloc supprimé |
| `si3_fallback[]` hardcode | `calypso_fbsb.c` | supprimé ; mmap-only ; si absent → pas d'écriture a_cd |
| `allc_burst_idx` static counter | `calypso_fbsb.c` | remplacé par `burst_d = fn & 3` |
| `ul_drop_no_bts` race | `bridge.py` | `trxd_remote` pre-set à `(BTS, 5802)` à init |
| BSP `trxd_peer_valid=false` race | `calypso_bsp.c` | pre-set à `(127.0.0.1, 5702)` à init |
| `publish_fb_found`/`publish_sb_found` calls | `calypso_fbsb.c` | call sites supprimés du chemin par défaut, env-gated `CALYPSO_FBSB_SYNTH=1` |

`.gitignore` étendu : `__pycache__/`, `*.pyc`, `*.bak*` pour éviter répétition
du bruit du commit 63b4fe5.

### Added (real connections)

| Connection | Fichier |
|---|---|
| `DB_W_D_TASK_RA = 7` poll dans tdma_tick UL section | `calypso_trx.c` |
| `calypso_bsp_tx_rach_burst` (real AB encoding via `gsm0503_rach_ext_encode`) | `calypso_bsp.c` |
| libosmocoding link dans meson | `hw/arm/calypso/meson.build` |
| `BRIDGE_CLK_FROM_QEMU=1` env mode (CLK IND piloté par QEMU FN, pas wall-clock) | `bridge.py` |
| `-icount shift=auto,align=off,sleep=off` sur QEMU | `run.sh` |
| `CALYPSO_FBSB_SYNTH=1` env-gate pour ré-activer synth quand emulated DSP correlator ne converge pas | `calypso_fbsb.c` |
| `CALYPSO_NDB_D_RACH_OFFSET=0xNNN` override pour pinner offset | `calypso_bsp.c` |

Doc à jour : `README.md` Latest update section, `hw/arm/calypso/doc/hacks.md`
§ Cleanup 2026-05-07, `hw/arm/calypso/doc/TODO.md` § Status 2026-05-07,
`hw/arm/calypso/doc/PROJECT_STATUS.md` entry du jour, `CLAUDE.md` Current Bug
section refresh.

## Run actuel — observations

Avec `CALYPSO_FBSB_SYNTH=1` (rebuild 13:04 dans le conteneur Docker `trying`) :

**DL retrouvé** :
- `[calypso-fbsb] CALYPSO_FBSB_SYNTH=1 (synth, dev-assist path)` confirmé au boot
- `[fbsb] FB0_FOUND (synth)` + `SB_FOUND (synth)` cycliques
- Mobile L3 : `New SYSTEM INFORMATION 1, 2, 3, 4 (lai=001-01-1)` répétés
- `Changing CCCH_MODE to 2` → cell sync complet

**UL — progrès** :
- 8 paquets UL délivrés au BTS
- Premiers UL sur **TN=0** (vrai slot RACH) : `bridge: UL #7 TN=0 fn=1480`,
  `UL #8 TN=0 fn=1480` — vs UL antérieurs sur TN=4/6 (garbage)
- Côté DSP : `[BSP] RACH encode #N fn=NNN ra=0xXX bsic=0xXX d_rach=0xNNNN` fire
  régulièrement
- BTS `osmo-bts-trx` clock skew : `1 FN compensation` au lieu de `102 FN` avant
  icount + CLK_FROM_QEMU

**UL — problèmes restants** :
1. `task_ra` non-zéro très tôt (fn=104, avant FBSB complete) — **suspect**.
   Hypothèse : d_rach offset par défaut `0x01CB` lit la mauvaise zone NDB,
   ou le firmware écrit du résidu non-zéro qu'on interprète comme RACH command.
   Symptôme : BSIC varie run-to-run (`0x2a, 0x24, 0x1f, 0x14, 0x33, 0x1c, 0x19,
   0x15, 0x3c`) alors que BSIC est constant pour une cellule donnée.
2. `task_u` et `task_ra` fire en même temps avec valeurs incohérentes
   (`task_ra=0x2d4e tn=6 fn=104` + `task_u=0xfec2 tn=6 fn=104`) — vraisemblablement
   les deux sont des résidus mémoire que ARM n'a pas encore initialisés à zéro.
3. Pas encore vu d'`IMM_ASS_CMD` côté mobile — soit BTS rejette les RACH UL
   (malformés ?), soit BSC ne génère pas IMM_ASS, soit mobile rate l'AGCH
   sub-slot.

## Diagnostic question pour toi (Claude web)

Le DL milestone tient via `CALYPSO_FBSB_SYNTH=1` qui ré-active les écritures
synth `a_sync_demod[*]` + `d_fb_det=1` à chaque DSP_TASK_FB write par l'ARM.
Sans synth, le DSP exécute la vraie routine fb-det (range PC ~0x8d00..0x8f80)
sur les samples GMSK que le BSP modulé depuis les hard bits TRXD du bridge.

GMSK h=0.5 : pour un FCCH burst (148 zéros) la phase avance de +π/2 par bit
→ pure tone à `fs/4` ≈ 67.7 kHz, conforme spec GSM. Notre cos_tab/sin_tab
walk reproduit cette structure (calypso_bsp.c lignes ~290-301). Donc **en
théorie** le correlator DSP devrait latch sur le FCCH.

Il ne le fait pas. Trois pistes :

(α) **Bug d'opcode dans c54x_exec_one** : un MAC ou correlator instruction
    (probable : MACR, MAC*AR2-,A, MACSU) calcule mal sur les samples Q15
    full-scale ±0x7FFE. Hint historique : range PC `0x8d33/0x8eb9/0x8f51`
    sont des fb-det stores connus (cf. `c54x.c` "real_fbdet_pcs[]" comments).
    Si on instrumente A/B accumulators à ces PCs et qu'on compare aux
    valeurs attendues sur un burst FCCH pur, on saura.

(β) **Timing miss** : la routine fb-det DSP démarre à un offset symbol qui
    ne matche pas le burst que BSP vient de DMA. Le DSP correlator suppose
    que les 148 symboles dans `0x3fb0..0x4047` (ou wherever DARAM RX zone)
    sont la fenêtre courante, mais en QEMU le DMA arrive au mauvais moment
    relatif au TPU FRAME IRQ. Avec `-icount` ça devrait être stable mais
    pas forcément aligné.

(γ) **Phase reference inversée** : nos cos_tab/sin_tab walk advance phase à
    `+π/2 per zero bit` (NRZ : `bit 0 → phase += π/2`). Si le correlator DSP
    s'attend à `-π/2`, l'output est conjugué et le pic FCCH n'apparaît pas
    là où attendu. Trivial à inverser pour tester.

**Question** : selon toi, lequel des trois pistes mérite l'instrumentation
en premier ? J'ai en main :
- log complet de PC HIST (`/root/qemu.log` côté DSP, ~80k lignes)
- la pcap GSMTAP en cours de capture (`/root/mobile-gsmtap.pcap`)
- accès aux registres DSP (A, B, T, AR0..AR7, ST0/ST1, PMST) via les logs
  existants à PCs spécifiques (cf. MAC-7700 tracer ENTER-7700 tracer dans
  c54x.c)

Et sur le UL — vu que les RACH partent pour de vrai sur TN=0 maintenant,
tu validerais quel test discriminant en premier : tcpdump GSMTAP pour voir
si IMM_ASS passe sur l'air, ou logs `osmo-bts-trx` pour voir si RACH
detect counter incrémente ?

## Annexes

Liens documentation :
- `README.md` § Latest update — Session 2026-05-07
- `hw/arm/calypso/doc/hacks.md` § Cleanup 2026-05-07 (table par hack)
- `hw/arm/calypso/doc/TODO.md` § Status 2026-05-07
- `hw/arm/calypso/doc/PROJECT_STATUS.md` § 2026-05-07

Diff récap : 12 fichiers modifiés, **+401 / −421 lignes** (net -20 lignes
malgré ajout libosmocoding integration + env vars).
