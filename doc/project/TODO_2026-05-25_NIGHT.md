> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce plan de chasse suppose que les cycles INT3 tournent et qu'un fix « F1 RETE manquant » débloque la cascade jusqu'au FB-lock. Vérité-terrain : `d_fb_det` reste 0, le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), IMR=0x0000 jamais ré-armé après le clear boot (@0xb37e), `api_write_cb` jamais câblé, pas de bus ORCH. Le vrai verrou n'est pas RETE/correlator mais le handshake ARM→DSP go-live qui ne s'arme jamais (ARM n'écrit que 0x0000 dans l'API 0x0314/0x0318). Les « Attendu » de la section 3 ne se produisent PAS en l'état. Corrections ci-dessous.

# TODO next session — DSP correlator lock

> Méthodo c web : **un seul front à la fois**. F1 RETE manquant d'abord.
> Si correlator écrit encore 0xFFFF post-F1-fix, ouvrir F2.

## 1️⃣ IMMÉDIAT — Sonde INT3_CYCLE_TRACE décisive

**Run** (env complet dans STATUS) :
```bash
CALYPSO_INT3_CYCLE_TRACE=1
BRIDGE_BURST_SOURCE=internal BRIDGE_BURST_PATTERN=fcch
# ... rest per STATUS_2026-05-25_NIGHT.md
```

**Analyser** :
- `INT3-CYCLE #1 RETE-GOOD` (= bon cycle, ARM L1S progresse)
- `INT3-CYCLE #2 ORPHAN-NEXT-INT3` (= cycle stuck, jamais RETE)
- **Diff branche par branche** : trouver la 1ère où `next_pc` diffère entre #1 et #2

À cette branche :
- Lire l'opcode (BC ALEQ ? BANZ *ARn- ? BCD NTC ?)
- Identifier la **valeur testée** (cond flag ? AR value ? TC ?)
- Trouver d'où vient cette valeur (= grep le runtime ROM ou trace upstream)

→ **Le bug = la condition mal évaluée OU l'input manquant** (c web's couloir : opcode mis-décodé OU input pas fourni).

## 2️⃣ Fix F1 (RETE manquant) — quand diff identifié

Selon la branche divergente :

| Type bug | Fix path |
|---|---|
| **Opcode mis-décodé** dans c54x emul | Audit binutils + correction handler |
| **Input manquant** (ARM doit écrire un flag mais ne le fait pas) | Trace ARM-side, vérifier dsp_set_params() effective |
| **Valeur d'un AR/flag setup différemment cycle #1 vs #2** | Trace upstream du STM/STLM qui setup |

## 3️⃣ Validation post-F1-fix

Re-run avec `CALYPSO_INT3_CYCLE_TRACE=1` + `BRIDGE_BURST_SOURCE=internal` :

**Attendu** (⚠️ audit 2026-07-01 : NON atteint en l'état — `d_fb_det` reste 0, DSP déraille, IMR jamais ré-armé ; ces attendus restent hypothétiques) :
- INTM-TRANS multi-cycles (= ISR RETE naturellement à chaque entry)
- CORR-ENTRY > 3 (= correlator entered post-init)
- `D_FB_DET SET val=0x0001` (= FB found, real lock)
- `DSP WR a_sync_SNR/TOA/ANG/PM` avec **valeurs réelles** (= not 0xFFFF)
- `TX→mobile FBSB_CONF` avec payload non-FAIL (= mobile sort du LOST loop)

**Si CORR-ENTRY monte mais valeurs encore 0xFFFF** → F2 (= bug correlator indépendant) → ouvrir nouvelle chasse.

## 4️⃣ TODO secondaires (à PARQUER, pas chasser maintenant)

Per c web's méthodo "séquentiel, pas parallèle" :

- **0x3DC0 SARAM flag** : bit 4 set par PC=0x9ad0 (MMR-clear residue spurious mais OK). Pas le blocker du polling fc66.
- **DSP param area init** : ARM appelle dsp_set_params() (vu via osmocon "Setting API NDB parameters"). Effectif MMIO non-vérifié (= 0 reads DSP-side à 0x0C31-0x0C6A dans tout run = peut pas savoir).
- **0xfc50 entry mystery** : fc50 hot mais aucune CALL/B y mène (grep exhaustif). XPC paging suspect. fc50 lui-même = compute kernel (= MVMM AR4,AR5 + MAR setup + LD/SFTA/SUB/ABS + BANZ outer + RET fc6f). Pas un polling.
- **BSP DMA fn=0 stats** : 96% drop ARM-side per parse_summary. Slot mismatch entre bridge feeder et BSP DMA window.
- **BRINT0 edge-trigger gate** dans calypso_bsp.c ~~L638~~ — FAUX: L638 est un commentaire (« No immediate delivery »), pas le gate. Le gate réel `if (bsp.dsp && !(bsp.dsp->ifr & (1 << 5)))` est à **calypso_bsp.c:1097, 1304, 1367** : empêche subsequent BRINT0 fires. Sur silicon level-trigger. **À fixer APRÈS F1**.

## 5️⃣ Outils refusés par c web (NE PAS reactiver sans raison)

- ❌ **CALYPSO_FORCE_INTM_ONESHOT** : corrompt state ISR, indiscernable de bug aval. Retirer comme outil de diag downstream.
- ❌ **CALYPSO_FORCE_INTM_AT_PC** : même problème, pire (pas de "PC sûr" mid-ISR).
- ❌ **NMI APIC_W_DSPINT hook** : `APIC_W_DSPINT` jamais utilisé par firmware osmocom (grep). Forcer ce mécanisme = falsification (= hack rxDoneFlag réincarné).
- ❌ **CALYPSO_FBSB_SYNTH** : déprécié 2026-05-08, ne pas réactiver.
- ❌ Modifier MAX_FN_SKEW dans osmo-bts : maquillage, pas fix.

## 6️⃣ Si BTS shutdown lors du run sondé

Per c web : harmless. BTS bypass via `BRIDGE_BURST_SOURCE=internal` rend BTS optionnel. Le milestone DSP-lock est testable sans BTS up. Si BTS shutdown, relancer.

## 7️⃣ Memory items relevant

À consulter avant prochaine session :
- `project_bridge_burst_feeder_milestone.md`
- `project_blocker_2_dsp_dispatcher_polling.md`
- `project_sp_silicon_fix_spiral_resolved.md`
- `feedback_hypothesis_probe_dynamic.md`
- `feedback_user_pilots_build_run.md` — **user pilote build/run, ne pas auto-restart**

## 8️⃣ Conventions session

- **Aucun fix sans diag prouvant le mécanisme**
- **Une probe à la fois**, env-gated, no-cost-when-off
- Sync 3 dirs : host `/home/nirvana/qemu-src/` + `/home/nirvana/qemu-calypso/` + container `trying:/opt/GSM/qemu-src/`
- Container=main pour build (`trying:/opt/GSM/qemu-src`)
