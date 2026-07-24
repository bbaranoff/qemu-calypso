> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> Périmés spécifiques à ce doc :
> - Le redirect silicon-boot-ROM `0xFF80→0x7120` n'est **PAS** un modèle hardware permanent actif par défaut : il est **NEUTRALISÉ par défaut**, gaté derrière `CALYPSO_REDIR_LEGACY=1` (A/B uniquement), et le code lui-même le qualifie de **HACK** (`calypso_c54x.c:10579-10581`, commentaire 10569-10578, "REDIRECT NEUTRALISÉ PAR DÉFAUT (2026-05-31)").
> - `CALYPSO_BSP_BYPASS_BDLENA` : le champ `bsp.bypass_bdlena` est lu depuis l'env mais **jamais consommé** — les "usage 1/usage 2" décrits n'existent pas dans le code ; le hack est **inerte** (`calypso_bsp.c:125` decl, `:809-810` init/log, aucune garde ailleurs).

# TODO — POLE POSITION 2026-06-25 (reprise à chaud)

## Acquis cette nuit (gravés, commit `4504dec` sur main nirvana)
- ✅ **GAP-1 (derail) RÉSOLU** — fix MVKD 0x70 (longueur 2-mot + lk) : plus de POST-BOOTSTUB-RET storm, DSP exécute son L1 réel stable.
- ✅ **READA sous RPT** — fix `rpt_fresh` (1ère lecture repart de A_low, pas du mvpd_src stale) : table de dispatch désormais FIDÈLE au ROM (data[0x4387]=0xab38, plus 0xf074).
- Les deux ISA-correct. NE PAS revert READA (le 0→7,3M POST-BOOTSTUB-RET = masque qui tombe, pas régression).

## PREMIER MOVE DEMAIN (sonde déjà armée/buildée/syncée — juste lancer + lire)
1. `start-clean.sh` (canary=0, FBDET_SENTINEL=2)
2. `grep -a "DISP-ENTRY" /root/qemu.log | head -20`
3. `grep -a "SEED-WR"    /root/qemu.log | head -20`

## TRISECTION (lecture du résultat — ferme la fourche en 1 run)
Contexte : dispatcher 0xb400 fait `STM #0x5ac8,SP` (reset pile DÉLIBÉRÉ) → `STLM A,AR7` (AR7=0x4387) → `LD *AR7,A ; BACC A` → data[0x4387]=0xab38=RET no-op → RET dépile data[0x5ac8]=0 → storm. Question = provenance AR7 + seed data[0x5ac8].
- **(a) data[0x5ac8] jamais écrit + AR7=immédiat littéral** → idle-path réel, seed retour-scheduler jamais posé → **la dispatch FB est AILLEURS** → trouver le vrai chemin FB (élargir BACC-DISP / tracer les autres bacc/cala).
- **(b) data[0x5ac8] écrit mais mauvaise addr / SP décalé** → vrai **bug pile émulateur** (CALL/RET ou mirror) → fixer.
- **(c) AR7 calculé depuis un registre d'index/événement gelé à 0** → dispatcher correct, index mort → **remonter au fil IPTR/IRQ-delivery** ; le 7,3M no-op-RET = famine d'événement amont (cohérent AR3=AR4 gelés).

## Frontière = Phase B (FB detect)
But : le vrai détecteur FB tourne et écrit d_fb_det. I/Q confirmée livrée à 0x2a00 (canary). Une fois la dispatch FB atteinte → retarget DETTRACE sur son handler.

## Nettoyage (plus tard, follow-up commit)
Stripper les sondes diag de calypso_c54x.c/.h : MVKD/READA = keepers ; DETTRACE, BACC-DISP, DISPVAL-WR, DISP-ENTRY, SEED-WR, AR3-TRIP, AR0-TRACE, B3D1-CTX, DERAIL-* = jetables.

---

---
name: TODO — temporary hacks with retraction criteria
description: Per CLAUDE.md rule #1, all env-gated hacks listed here with explicit removal criterion.
type: project
---

# Objectif courant (2026-05-30) : FB-dispatch

Le root SP est résolu (cf `STATUS.md` : redirect silicon-boot-ROM restauré,
wedge 0x70c3 mort, base saine). Blocker restant = le DSP ne dispatche jamais le
détecteur FB `0x9ac0` :
- ISR INT3 @0xffcc early-return sur `RC NTC@0xffcd` (TC=0) → n'atteint jamais
  read d_task_md / test flag `0x3DC0 bit4` / gate `data[0x62]`.
- À trouver : qui set TC (`0x0100@0xffcc`), qui arme `0x3DC0`/`data[0x62]`,
  pourquoi le DSP reste en idle minimal sans scheduler L1 complet.
- Envoyé à CC-web (30/05), réponse attendue.

# ~~Modèles hardware permanents (NE PAS retirer)~~ — FAUX : redirect NEUTRALISÉ par défaut, gaté, qualifié HACK par le code

## Silicon-boot-ROM redirect 0xFF80→0x7120  ~~(réactivé 2026-05-30)~~ — NEUTRALISÉ 2026-05-31

> ⚠️ **FAUX / PÉRIMÉ.** ~~réactivé 2026-05-30~~ ~~actif par défaut~~ ~~Ce n'est PAS un hack rule#1~~ — le code contredit chaque point ci-dessous.

**Site** : `hw/arm/calypso/calypso_c54x.c:10579-10581`, top de `c54x_run` while-loop
(commentaire `calypso_c54x.c:10569-10578`).

Snippet **réel** (le doc omettait la garde `redir_legacy` et les branches `INITTAB`/`REDIR7000`) :
```c
static int redir_legacy = -1;
if (redir_legacy < 0) redir_legacy = getenv("CALYPSO_REDIR_LEGACY") ? 1 : 0;
if (redir_legacy && s->pc == 0xFF80 && s->sp == 0x1100) { /* ... → 0x7120 */ }
```

**Statut corrigé** :
- ~~`if (s->pc == 0xFF80 && s->sp == 0x1100) { s->pc = 0x7120; }` inconditionnel~~ —
  FAUX : gaté derrière `CALYPSO_REDIR_LEGACY`, **default OFF** (`calypso_c54x.c:10580`).
- ~~modèle hardware permanent, NE PAS retirer~~ —
  FAUX : le commentaire du code (`calypso_c54x.c:10569-10578`) le qualifie explicitement de
  **HACK (simulation)** et le déclare **"REDIRECT NEUTRALISÉ PAR DÉFAUT (2026-05-31)"**.
- ~~Ce n'est PAS un hack rule#1~~ —
  FAUX : c'en est un ; méthode retenue = NE RIEN simuler, laisser le vrai reset vector
  (`0xff80`) jouer et débugger chaque bug réel de la chaîne de boot.

**Rationale (historique, conservé)** : le dump PROM ne contient pas le mask-ROM silicon
qui, au reset, pose SP=0x5AC8 et saute à l'entrée firmware PROM0[0x7120]. Sans ce modèle,
SP reste à 0x1100 → over-pop → wedge 0x70c3. Ce raisonnement a motivé le redirect, mais il
a été **abandonné/désactivé** au profit du debug du vrai bug de boot.
**Régression** : avait été retiré par erreur en git `c3ec660` (29/05).

# Active hacks (must be removed once root cause fixed)

## CALYPSO_BSP_BYPASS_BDLENA (env-gated, default OFF) — ⚠️ INERTE (champ jamais consommé)

> ⚠️ **FAUX / PÉRIMÉ.** Ce "hack actif" ne fait rien : `bsp.bypass_bdlena` est lu depuis
> l'env mais **jamais relu ensuite** pour skipper quoi que ce soit. Les "usage 1/usage 2"
> décrits n'existent pas dans le code. `calypso_iota_take_bdl_pulse` n'apparaît qu'en
> **commentaire** (`calypso_bsp.c:1095`) et n'est jamais appelé.

**Site** : `hw/arm/calypso/calypso_bsp.c`
- declaration : `bsp.bypass_bdlena` field — `calypso_bsp.c:125`
- init : `calypso_bsp_init` lit `CALYPSO_BSP_BYPASS_BDLENA` — `calypso_bsp.c:809`, log seul `:810`
- ~~usage 1 : `calypso_bsp_rx_burst` (line ~755) — skip `calypso_iota_take_bdl_pulse(tn)` check~~ —
  FAUX : `calypso_bsp_rx_burst` est à `calypso_bsp.c:938`, ne lit pas `bypass_bdlena`.
- ~~usage 2 : `calypso_bsp_deliver_buffered` (line ~825) — same skip~~ —
  FAUX : `calypso_bsp_deliver_buffered` est à `calypso_bsp.c:1107`, ne lit pas `bypass_bdlena`.

**Rationale** : on silicon, BSP only carries samples while IOTA's BDLENA pin
asserted (= the firmware gates RX windows via TPU→TSP→IOTA). Sur emu, BDLENA
asserts depend on TDMA tick + IOTA model — peut nous filtrer des bursts
nécessaires pour comprendre quelle DARAM addr le DSP correlator lit.

**Reintroduit** : 2026-05-28 (restauré depuis ancien commit 1156b30/58faeef
post-merge #17 "removing_hacks"). Autorisation explicite utilisateur.

**Critère de retrait** :
1. Identifier la vraie `CALYPSO_BSP_DARAM_ADDR` que le DSP correlator lit
   (= zone DARAM qui matche les reads observés via CORR-RD probe à
   PROM0[0x9aba..0x9abf])
2. Confirmer que avec cette addr correcte, les writes DSP à `a_pm[]` et
   `a_sync_demod[*]` sont **nonzero** (= mesure de puissance réelle publiée)
3. Vérifier que mobile L1 `PM MEAS` retourne un dBm réaliste (pas -138 floor)
4. Retirer `bsp.bypass_bdlena` + tout son code de garde

## CALYPSO_FORCE_ANGLE_ZERO — RETIRÉ 2026-05-28

Remplacé par le modèle TWL3025 chip qui fait le vrai taf (DAC →
phase rotation des samples BSP RX, sans env gate, armé par défaut).
Firmware ferme la boucle AFC normalement comme sur silicon, plus de
court-circuit. Cf. `hw/arm/calypso/calypso_twl3025.c`.
