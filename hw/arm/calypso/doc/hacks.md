# Inventaire des hacks et workarounds — qemu-calypso

> **Établi 2026-04-30** au cours d'une session debug FBSB→BCCH.
> Honest inventory : à mettre à jour à chaque session ajoutant ou retirant un hack.

## Légende des niveaux

| Niveau | Définition | Politique |
|---|---|---|
| **Hack brut** | Stub qui ment. Court-circuite un pipeline qui existerait. Viole règle #1 CLAUDE.md "no stubs". | **À retirer impérativement** avant claim B2B / licensing. Critère de retrait obligatoire. |
| **Bypass architectural** | Workaround documenté pour combler un trou de l'environnement d'émulation (pas de RF, pas de timing physique). Justifié tant que QEMU n'a pas l'équivalent. | Acceptable pour LU PoC. À évaluer post-LU si replacement faisable. |
| **Dette technique** | Code correct mais pas idéal. Pattern fragile, code mort, redondance. Pas mentir, juste pas propre. | Nettoyer post-LU pour qualité long-terme. Pas bloquant. |

---

## `calypso_fbsb.c` — orchestration FBSB host-side

### Hacks bruts

#### `si3_blob[23]` hardcodé (étape 2 BCCH read)

**Quoi** : `static const uint8_t si3_blob[23] = { 0x1b, 0x75, ... }` packed dans `a_cd[3..14]` à chaque task=24 fire pour produire DATA_IND avec payload LAPDm valide.

**Pourquoi c'est un hack** : `osmo-bts-trx` émet déjà des bursts BCCH réels via TRXD UDP 5702 vers `bridge.py`. La fixture libosmocore (`tests/gb/gprs_bssgp_rim_test.c:376`) court-circuite ce pipeline producteur existant. Le LAI/CGI ne matche pas la config réelle du BSC.

**Critère de retrait OBLIGATOIRE** (cf. `TODO.md` § XXX TEMP HARDCODE) :
1. `bridge.py` instrumenté pour intercepter les bursts BCCH (TN=0, fn%51 ∈ {2,3,4,5}) reçus de `osmo-bts`.
2. Soit déinterleavage côté Python → 23 bytes LAPDm puis forward UDP séparé vers QEMU.
3. Soit intercept `osmo-bts` avant interleaving (RSL/scheduler) pour 23 bytes pré-encodés.
4. `case DSP_TASK_ALLC` lit depuis buffer dynamique au lieu de fixture statique.
5. Hardcode physiquement retiré du `.c`, entrée TODO.md supprimée.
6. Run validation : DATA_IND traversent + cell synced avec SI3 réel BSC.

**Marqueur code** : `XXX TEMP HARDCODE replace with bridge intercept`

#### `allc_burst_idx` static counter cycle 0..3

**Quoi** : compteur local incrémenté à chaque task=24 fire, écrit dans `db_r->d_burst_d`.

**Pourquoi c'est un hack** : assume une cadence parfaite cmd→resp côté firmware. Si timing dérive (frame skip, jitter), le counter désynchronise → `BURST ID mismatch` côté firmware (24/28 mismatch observés). User suggéra de lire `wp[DB_W_D_BURST_D]` directement, mais shift de timing schedule firmware empêche cette approche simpliste.

**Critère de retrait** : implémenter frame-tick scheduled write basé sur fn modulo 51 (block boundary) pour aligner sur le vrai schedule mframe. Ou alternative : fix proprement le timing TDMA pour éliminer la dérive cmd/resp.

**Note** : le hack ne casse pas DATA_IND (resp(3) match accidentel donne 1 DATA_IND par bloc), mais 75% des resps sont rejetées avec mismatch noisy.

### Bypass architecturaux

#### `publish_fb_found(toa=0, pm=80, angle=0, snr=100)`

**Quoi** : valeurs synthétiques "FB trouvé parfait" écrites dans NDB `a_sync_demod[*]` à chaque ARM `d_task_md=5` write.

**Pourquoi bypass** : QEMU n'a pas de vrai signal RF analogique. Le bridge livre des samples I/Q d'une LUT cos/sin fixe, pas un vrai FCCH 67kHz pure tone. Le DSP correlator émulé ne peut pas converger sur ces samples synthétiques. Sans bypass, `freq_err` resté à -14119 Hz (valeur DSP iter sur LUT) → FBSB_CONF result=255.

**Justification environnement** : le DSP demod/AFC est ce qui n'est pas faisable en QEMU faute de vrai signal RF. L'origine du payload (osmo-bts → bridge → DSP) reste correcte.

**Replacement éventuel** : émulation SDR-style avec gr-osmosdr générant un vrai FCCH burst → DSP correlator converge. Out of scope pour LU PoC.

#### `publish_sb_found(bsic=0)`

**Quoi** : `a_sch[0..4]` écrit dans LES DEUX read pages avec pattern CRC-OK + BSIC=0.

**Pourquoi bypass** : même raison que FB. SB demod nécessite un vrai burst SCH décodable.

**Pourquoi belt-and-suspenders sur les deux pages** : ARM toggle `d_dsp_page` indépendamment du moment où notre hook fire. Sans certitude sur la page active au resp time, on écrit aux deux. Acceptable mais dette technique.

**Replacement éventuel** : tracker `s->dsp_page` pour écrire sur la page active uniquement.

#### `publish_pm_found` (RSSI synthétique)

**Quoi** : valeurs RSSI synthétiques pour mesure puissance ARFCN.

**Pourquoi bypass** : pas de RF réel.

#### `case ALLC_DSP_TASK` echo `d_task_d` + `d_burst_d`

**Quoi** : le DSP émulé ne handle pas task=24. On écrit `d_task_d=24, d_burst_d=N` dans la read page pour passer la guard `EMPTY` de `prim_rx_nb.c`.

**Pourquoi bypass** : implémentation complète DSP CCCH read = mois de travail (NB processing : demod, deinterleaving, channel decoding, CRC). Court-circuit la chaîne mais expose le slot mailbox correct.

---

## `calypso_c54x.c` — émulateur DSP TMS320C54x

### Hacks bruts (diagnostiques)

#### `CALYPSO_FORCE_INTM_CLEAR_AT=N` env var

**Quoi** : block de code dans `c54x_exec_one` qui force `ST1.INTM ← 0` à insn count N si env var positionnée.

**Status** : documenté comme **instrument diagnostic** dans `TODO.md` (pas workaround). Désactivé par défaut. Permet de tester l'hypothèse catch-22 INTM forever.

**Critère de retrait OBLIGATOIRE** (cf. TODO.md) :
1. La vraie séquence RSBX INTM du boot DSP est identifiée et émulée correctement.
2. Run sans `CALYPSO_FORCE_INTM_CLEAR_AT` passe les indicateurs (RETED ≥1, d_fb_det écrit).
3. Block physiquement retiré du `.c`, `hack.patch` supprimé.

### Bypass architecturaux

#### W1C latch snapshot dans `data_write`

**Quoi** : quand DSP écrit `d_fb_mode != 0` depuis PCs `{0x8d33, 0x8eb9, 0x8f51}` (real fb-det iteration end), snapshot `a_sync_demod` cells dans globaux `g_*_latch`.

**Pourquoi bypass** : résolve une race window où le DSP écrit ses values DSP-iter par-dessus les valeurs synthétiques fbsb avant qu'ARM ne les lise. Le latch capture la "fin d'itération" cohérente.

**Status actuel** : **inactif** dans les runs récents (LATCH count = 0). fbsb publish gagne la race par timing.

**Dette technique parallèle** : le code latch reste compilé. À supprimer ou activer proprement post-LU.

#### Liste statique `real_fbdet_pcs[] = {0x8d33, 0x8eb9, 0x8f51}`

**Quoi** : filtre PCs autorisés à déclencher snapshot. Découverts empiriquement.

**Pourquoi hack** : si firmware fb-det utilise un 4ème PC dans certaines conditions (mode change, error path), on rate les writes.

**Mitigation présente** : log "new PC" pour tout write à `d_fb_mode` venant d'un PC inconnu — à vérifier que ça compile encore.

### Dette technique

#### Filtres logging mensongers

**Pattern** : `if (fbd_log < 5) printf("(spurious, ignored)")` — corrigé en cours de session pour `d_fb_mode`, mais probablement répété ailleurs (audit pour tous les log filtering similaires).

#### 12+ opcode fixes au fil de l'eau (sessions précédentes)

**Quoi** : F4E4=FRET, F074=CALL pmad, F9xx=CC cond call, etc. (cf. CLAUDE.md "Known Fixed Opcode Bugs" + TODO.md sessions log).

**Niveau** : majoritairement vrais fixes contre `tic54x-opc.c` + SPRU172C. Quelques-uns peuvent être "fait marcher pour ce firmware" sans couvrir tous les cas du standard C54x. Audit systématique nécessaire pour upstream-defendable.

---

## `calypso_bsp.c` — DMA BSP I/Q samples

### Bypass architecturaux

#### `BSP_FN_MATCH_WINDOW` élargi (±64 ou ±128 frames)

**Quoi** : fenêtre d'acceptation des bursts entrants vs FN courant. Sur silicon réel = ±1.

**Pourquoi bypass** : compense la dérive QEMU virtual clock vs bridge.py wall-clock. Sans fenêtre élargie, 1:31 stale ratio observé (drop 31 bursts sur 32).

**Vrai fix** : `-icount shift=auto` côté QEMU OU bridge.py slave de QEMU virtual time (déjà partiellement implémenté via CLK FD socket).

**Dette technique** : on accepte des samples de la mauvaise frame. Bénin tant que fbsb bypass actif (DSP demod n'est pas critique). **Devient un problème majeur si fbsb bypass retiré**.

---

## `calypso_trx.c` — Calypso SoC + TDMA + DSP API

### Bypass architecturaux

#### Latch consume path dans `calypso_dsp_read`

**Quoi** : code qui retourne `g_*_latch` si `g_a_sync_valid`, puis invalidate.

**Status** : symétrique au snapshot dans c54x.c, **inactif** dans runs courants. Conceptuellement propre (W1C semantics). Dette technique parallèle.

### Dette technique

#### `calypso_dsp_done` IRQ_API timing optimiste

**Quoi** : `qemu_irq_raise(IRQ_API)` immédiatement après DMA write page → DSP. Sur silicon, cycles de propagation entre DMA completion et IRQ assertion.

**Impact actuel** : aucun. Le firmware tolère.

**Risque latent** : si le firmware ARM compte sur des cycles de slack entre TPU_CTRL_EN et IRQ_API, comportement non identique au silicon.

---

## `calypso_uart.c` — UART + sercomm

### Hacks bruts

#### TRXD v0 patch côté osmocom-bb-transceiver/trxcon

**Quoi** : patch dans le tooling osmocom (pas dans uart.c) qui force protocole TRXD v0 au lieu de v1, mismatch versions.

**Critère de retrait** : harmoniser versions TRXD entre osmo-bts-trx et trxcon, ou implémenter v1 dans bridge.py.

### Hacks par omission

#### UART1 / UART2 stubs minimaux

**Quoi** : seul UART0 utilisé pour L1CTL (sercomm DLCI 4/5). UART1 (irda) et UART2 (DSP) probablement stubs minimalistes.

**À vérifier** : audit UART1/UART2 read/write pour s'assurer qu'ils ne mentent pas (= retournent valeurs plausibles ou zéro plutôt que valeurs fabriquées).

---

## `calypso_inth.c` — interrupt controller

Pas de hack apparent à ma connaissance. Audit complet recommandé pour confirmer.

---

## `calypso_soc.c` — SoC glue

### Hacks par omission

#### `add_stub()` helper + I2C stub

**Quoi** : `static void add_stub(MemoryRegion *sys, ...)` aux lignes 125+, et "I2C stub" à la 187.

**Niveau** : stubs explicites pour MMIO regions non implémentées. Acceptable tant qu'ils retournent 0 / accept writes silently. À auditer pour s'assurer qu'aucun ne ment (retourne valeur fabriquée non-zero).

---

## `bridge.py` — bridge BTS UDP

### Bypass architecturaux

#### `fn_anchor` désactivé / commenté "telemetry only"

**Quoi** : `bts_fn` passé tel quel au lieu d'être réécrit pour aligner sur QEMU virtual time.

**Pourquoi bypass** : alignement correct nécessiterait que bridge soit slave de QEMU virtual clock. Actuellement bridge ne fait que logger l'anchor.

**Dette technique** : si on voulait aligner les FNs, le code est commenté donc trivial à réactiver, mais demande validation.

#### `CLK_IND` à cadence wall-clock fixe

**Quoi** : tick CLK envoyé au BTS à intervalle wall-clock régulier (~471 ms / 26 frames).

**Pourquoi pas un hack** : c'est ce que fait un téléphone réel. **Mais** crée la dérive QEMU virtual clock vs wall-clock que `BSP_FN_MATCH_WINDOW` élargi compense.

**Vrai fix** : faire bridge slave de QEMU virtual time (CLK_IND tick basé sur fn QEMU, pas wall-clock).

### Dette technique

#### `time.sleep()` ou similaire pour cadence

À vérifier dans le source. Fragile sous charge système.

---

## Tooling — `run.sh`, configs

### Fragilité opérationnelle (pas hack stricto sensu)

- Hardcoded paths
- Hardcoded ports
- Topologie tmux fixe `calypso:cell_log` / `calypso:mob`
- Symlinks `/tmp/*.log` → `/root/logs/*_001.log` qui peuvent être stales

---

## Patches firmware OsmocomBB ?

À vérifier dans le source. Si patches existent dans `prim_*.c` côté firmware (par ex. skip seuil threshold AFC trop strict), c'est un hack qui n'est pas dans QEMU mais dans le contrat.

**Vérification** :
```bash
cd /opt/GSM/osmocom-bb && git diff --stat
```

Si `git diff` montre des changements dans `src/target/firmware/`, lister précisément.

D'après mémoire `feedback_no_hacks_client_server.md`, l'objectif était "no patches firmware". À vérifier que c'est encore le cas.

---

## Récapitulatif synthétique

### À retirer impérativement avant claim B2B / licensing

| File | Hack | Status |
|---|---|---|
| `calypso_fbsb.c` | `si3_blob[]` hardcodé | XXX TEMP marked, TODO.md entry, critère de retrait défini |
| `calypso_fbsb.c` | `allc_burst_idx` static counter | À remplacer par frame-tick scheduled write |
| `calypso_c54x.c` | `CALYPSO_FORCE_INTM_CLEAR_AT` | Diag instrument, retrait conditionnel à vraie ISR INTM |
| `calypso_uart.c` (tooling) | TRXD v0 patch osmocom-bb-transceiver | Harmoniser versions |

### Bypass architecturaux justifiables tant que QEMU n'a pas vrai RF

| File | Hack | Justification |
|---|---|---|
| `calypso_fbsb.c` | `publish_fb_found/sb_found/pm_found` synthétiques | Pas de vrai signal RF |
| `calypso_fbsb.c` | `case ALLC_DSP_TASK` echo task_d/burst_d | DSP NB processing non implémenté |
| `calypso_c54x.c` | W1C latch snapshot/consume | Race window DSP iter vs ARM read |
| `calypso_bsp.c` | `BSP_FN_MATCH_WINDOW` ±64/128 | Dérive virtual vs wall clock |
| `bridge.py` | CLK_IND wall-clock | Bridge pas slave QEMU virtual time |

### Dette technique à nettoyer post-LU

- Latch code inactif dans `c54x.c` + `trx.c` (à supprimer ou activer proprement)
- Filtres logging avec messages mensongers (`spurious, ignored` style)
- UART1/UART2 stubs (audit nécessaire)
- I2C stub `calypso_soc.c:187` (audit)
- `bridge.py fn_anchor` désactivé (commenté)
- `run.sh` hardcoded paths/ports/tmux topology

---

## Méthode de mise à jour

À chaque session ajoutant ou retirant un hack :

1. Si ajout : créer entrée dans la section appropriée (hack brut / bypass / dette).
2. Tout hack brut MUST avoir :
   - Marqueur dans le code (`XXX TEMP HACK` ou similaire)
   - Critère de retrait explicite
   - Lien vers TODO.md détaillé
3. Si retrait : déplacer vers section "Retirés" (à créer ci-dessous quand nécessaire) avec date + commit hash.

## Verification empirique

```bash
# Lister marqueurs explicites :
grep -rEn "HACK|XXX|TEMP|TODO|FIXME|workaround|stub|bypass" \
     /opt/GSM/qemu-src/hw/arm/calypso/ /opt/GSM/bridge.py /opt/GSM/run.sh \
     2>/dev/null | grep -v '\.bak\|\.preNoCell\|/calypso_backup_check/'

# Vérifier patches firmware :
cd /opt/GSM/osmocom-bb && git diff --stat
```

Tout marqueur trouvé hors de ce fichier signale soit :
- Un hack non documenté → ajouter ici
- Un commentaire historique de dev → nettoyer ou justifier sa présence

---

*Inventaire établi 2026-04-30 nuit, session FBSB→BCCH étape 2 validée empiriquement.*
*Note honnêteté : peut être incomplet pour `c54x.c` opcode fixes profonds et certains aspects `bridge.py`/`run.sh` non audités exhaustivement.*
