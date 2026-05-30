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

# Modèles hardware permanents (NE PAS retirer — modélisent du silicon absent du dump)

## Silicon-boot-ROM redirect 0xFF80→0x7120  (réactivé 2026-05-30)

**Site** : `hw/arm/calypso/calypso_c54x.c`, top de `c54x_run` while-loop.
```c
if (s->pc == 0xFF80 && s->sp == 0x1100) { s->pc = 0x7120; }
```
**Rationale** : le dump PROM ne contient pas le mask-ROM silicon qui, au reset,
pose SP=0x5AC8 et saute à l'entrée firmware PROM0[0x7120] (= STM #0x5AC8,SP).
Sans ce modèle, SP reste à 0x1100 (invalide) → over-pop → wedge 0x70c3.
**Ce n'est PAS un hack rule#1** : modélise du hardware réel absent du dump, ne
force aucune instruction firmware (route vers l'entrée firmware propre qui fait
elle-même son init SP). Gate `SP==0x1100` = cold-reset seul.
**Régression** : avait été retiré par erreur en git `c3ec660` (29/05).

# Active hacks (must be removed once root cause fixed)

## CALYPSO_BSP_BYPASS_BDLENA (env-gated, default OFF)

**Site** : `hw/arm/calypso/calypso_bsp.c`
- declaration : `bsp.bypass_bdlena` field
- init : `calypso_bsp_init` reads `CALYPSO_BSP_BYPASS_BDLENA`
- usage 1 : `calypso_bsp_rx_burst` (line ~755) — skip `calypso_iota_take_bdl_pulse(tn)` check
- usage 2 : `calypso_bsp_deliver_buffered` (line ~825) — same skip

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
