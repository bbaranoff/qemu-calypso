# Inventaire des hacks et workarounds — qemu-calypso

> **Établi 2026-04-30** au cours d'une session debug FBSB→BCCH.
> Honest inventory : à mettre à jour à chaque session ajoutant ou retirant un hack.

## Légende des niveaux

| Niveau | Définition | Politique |
|---|---|---|
| **Hack brut** | Stub qui ment. Court-circuite un pipeline qui existerait. Viole règle #1 CLAUDE.md "no stubs". | **À retirer impérativement** avant claim B2B / licensing. Critère de retrait obligatoire. |
| **Bypass architectural** | Workaround documenté pour combler un trou de l'environnement d'émulation (pas de RF, pas de timing physique). Justifié tant que QEMU n'a pas l'équivalent. | Acceptable pour LU PoC. À évaluer post-LU si replacement faisable. |
| **Dette technique** | Code correct mais pas idéal. Pattern fragile, code mort, redondance. Pas mentir, juste pas propre. | Nettoyer post-LU pour qualité long-terme. Pas bloquant. |

## Cleanup — 2026-05-07

User mandate : « tant pis on bourrine pas, tu vire les hack des que tu vois ».
Removed in this pass :

| Hack | Type | File / loc | Replacement |
|---|---|---|---|
| `BOURRIN-FBDET-SKIP` PC range pop+jump | brut | `calypso_c54x.c` ~864 (block) | DSP runs full fb-det ; `-icount` covers cycle budget |
| `DIAG-HACK` env-gated INTM force-clear + ALIAS-CHECK dump | brut | `calypso_c54x.c` ~4950 (block) | Removed entirely ; CALYPSO_FORCE_INTM_CLEAR_AT no longer recognized |
| `publish_fb_found(toa=0,pm=80,angle=0,snr=100)` synth call | bypass | `calypso_fbsb.c::on_dsp_task_change DSP_TASK_FB` | Real DSP fb-det path on GMSK-modulated I/Q from `osmo-bts-trx` |
| `publish_sb_found(bsic=0)` synth call | bypass | `calypso_fbsb.c::on_dsp_task_change DSP_TASK_SB` | Real DSP sb-det path |
| `si3_fallback[23]` hardcoded SI3 | brut | `calypso_fbsb.c::on_dsp_task_change DSP_TASK_ALLC` | mmap-only ; no SI written if `/dev/shm/calypso_si.bin` absent |
| `allc_burst_idx` static cycle 0..3 | brut | `calypso_fbsb.c::on_dsp_task_change DSP_TASK_ALLC` | `burst_d = fn & 3` (FN-derived, no static state) |
| `ul_drop_no_bts` race in bridge UL | brut | `bridge.py::_handle_ul` | `trxd_remote` pre-set to (BTS, base+102) at init |
| BSP `trxd_peer_valid=false` until first DL | brut | `calypso_bsp.c::calypso_bsp_init` | Pre-set to bridge default (127.0.0.1:5702) ; refined on first DL |
| W1C latch system on `a_sync_demod` cells | bypass | `calypso_c54x.c` capture + `calypso_trx.c` consume | Env-gated via `CALYPSO_W1C_LATCH=1` (default OFF — ARM reads NDB direct) |
| `DSP_TASK_ALLC` db_r echo + a_cd mmap inject | bypass | `calypso_fbsb.c::on_dsp_task_change` | Env-gated via `CALYPSO_BCCH_INJECT=1` (default OFF — real DSP CCCH demod path). Pair with `FBSB_SYNTH=1` to deliver SIs end-to-end. |

Functions kept compiled but unused (`calypso_fbsb_publish_fb_found` /
`_publish_sb_found`) — diagnostic utilities, no live caller.

Connections added in this pass :

| Connection | File | Why |
|---|---|---|
| `DB_W_D_TASK_RA` polling (write-page word 7) | `calypso_trx.c::tdma_tick UL section` | RACH writes were silently dropped — only d_task_u was polled |
| `calypso_bsp_tx_rach_burst` via libosmocoding | `calypso_bsp.c` | Real RACH AB burst encoding from NDB d_rach |
| libosmocoding linkage in meson | `hw/arm/calypso/meson.build` | dep on `libosmocoding` for gsm0503_rach_ext_encode |
| `BRIDGE_CLK_FROM_QEMU` env-gated mode | `bridge.py` | Deterministic CLK IND from QEMU FN advance |
| `-icount shift=auto,align=off,sleep=off` on QEMU | `run.sh` | Reproducible virtual clock |

---

## `calypso_fbsb.c` — orchestration FBSB host-side

### Hacks bruts

#### `si3_blob[]` hardcoded — RESOLVED (2026-04-30)

**Status** : **REMOVED from active path**.

**Path** : `si3_blob[]` renamed to `si3_fallback[]`, used only when mmap unavailable
at first ALLC fire (cold-start race condition).

**Replacement** : `rsl_si_tap.py` + `/dev/shm/calypso_si.bin` mmap interface.

**Removal criterion** (for full cleanup) : tap reliability validated over
multiple sessions, then `si3_fallback[]` can be removed (replaced by
log warning + zeroed blob output, mobile would fail FBSB cleanly).

#### `populate-si.sh` — Cold-start warm cache (NOT a hack)

**Status** : kept in `scripts/` as manual debug tool. Removed from `run_si.sh` boot path.

**Function** : pre-populates mmap with last-known-good SI bytes. Useful when
osmo-bsc unavailable (debug isolation of QEMU+mobile without full network stack).

**Bytes contained** : snapshot from RSL trace 12:13 (osmo-bsc → osmo-bts
re-attach), byte-exact identical to live tap output for current BSC config.

**If BSC config changes** : re-run populate-si.sh post manual snapshot, OR rely
on `rsl_si_tap.py` which always reflects current BSC live config.

**Removal criterion** : `csi_init_once()` in `calypso_fbsb.c` modified to retry
mmap open periodically (currently lazy one-shot). Until then keep populate-si.sh
for manual cold-start scenarios.

#### `allc_burst_idx` static counter cycle 0..3 — RESOLVED (2026-05-07)

**Status** : **REMOVED**. Replaced by `burst_d = fn & 3` (FN-derived).
No static state, no run-to-run drift. The duplicate entry below is preserved
for historical context — both pointed at the same problem.

#### `allc_burst_idx` (duplicate entry — same fix as above)

**Status** : RESOLVED 2026-05-07. See entry above.

### Bypass architecturaux

#### `publish_fb_found(toa=0, pm=80, angle=0, snr=100)` — RESOLVED (2026-05-07)

**Status** : **call site REMOVED** in `on_dsp_task_change`. The function
is still compiled (no live caller) for diagnostic re-use if needed.
DSP fb-det now runs against real GMSK-modulated I/Q from `osmo-bts-trx`
(via `calypso_bsp.c::bsp_trxd_readable` → cos_tab/sin_tab phase walk).

**Risk if it regresses** : without enough cycle budget per TDMA tick,
the DSP fb-det routine doesn't complete → `freq_err` drifts → FBSB
returns result=255. Mitigation : `-icount shift=auto` on QEMU.

#### `publish_sb_found(bsic=0)` — RESOLVED (2026-05-07)

**Status** : **call site REMOVED** in `on_dsp_task_change`. Same as
`publish_fb_found` — function compiled but unused.

#### `publish_pm_found` (RSSI synthétique) — N/A

**Status** : was conceptual in the original `hacks.md` ; no concrete
implementation found in code at audit time. No removal needed.

#### `case ALLC_DSP_TASK` echo `d_task_d` + `d_burst_d` + a_cd inject — ENV-GATED (2026-05-07)

**Status** : env-gated via `CALYPSO_BCCH_INJECT=1`. Default = OFF (real
DSP CCCH demod path, currently non-converging in QEMU).

**Sites du code** : `calypso_fbsb.c::on_dsp_task_change DSP_TASK_ALLC` —
gardé par `bcch_inject_mode()`.

**Quand activer** : pair avec `CALYPSO_FBSB_SYNTH=1` pour la chaîne DL
end-to-end (mobile L3 décode SI1/SI2/SI3/SI4 depuis le mmap rsl_si_tap).

**Pourquoi bypass** : implémentation complète DSP CCCH read = mois de
travail (NB processing : demod, deinterleaving, channel decoding, FIRE
CRC). Court-circuit la chaîne mais expose le slot mailbox correct et
l'a_cd[] avec les vrais octets RSL du BSC.

---

## `calypso_c54x.c` — émulateur DSP TMS320C54x

### Hacks bruts (bourrin pre-LU)

#### Short-circuit fb-det `[0x8d00, 0x8f80]` — RESOLVED (2026-05-07)

**Status** : **REMOVED**. The 36-line block at `c54x_exec_one` ~864 has
been deleted entirely. DSP fb-det runs to completion. Cycle budget is
covered by `-icount shift=auto` on QEMU (set in `run.sh`).

### Hacks bruts (diagnostiques)

#### `CALYPSO_FORCE_INTM_CLEAR_AT=N` env var — RESOLVED (2026-05-07)

**Status** : **REMOVED**. ~120-line block at `c54x_run_until_idle_or_n`
~4950 deleted. Env var no longer recognized. The 100k-instruction VECDUMP
diagnostic block (`BOOT+100k VECDUMP-FORCED base=0xB900`) was removed in
the same edit since it was paired with the hack.

The INTM=1 catch-22 was resolved naturally when the DSP frame-IRQ wiring +
BSP RX delivery path converged ; no need for force-clear anymore.

### Bypass architecturaux

#### W1C latch snapshot dans `data_write` — ENV-GATED (2026-05-07)

**Status** : env-gated via `CALYPSO_W1C_LATCH=1`. Default = OFF (ARM reads
NDB direct). Pattern aligné avec `CALYPSO_FBSB_SYNTH=1`.

**Quand activer** : si on observe une race ARM-DSP où DSP écrit
`d_fb_mode != 0` depuis PCs `{0x8d33, 0x8eb9, 0x8f51}` puis clear avant
qu'ARM ne lise. Le latch capture la "fin d'itération" cohérente
(snapshot des 6 cells au moment du write a_sync_SNR).

**Sites du code** :
- Capture : `calypso_c54x.c` (~552-561) — gardé par `calypso_w1c_latch_enabled()`
- Consume : `calypso_trx.c` (~163-188) — gardé par `calypso_w1c_latch_enabled()`

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
