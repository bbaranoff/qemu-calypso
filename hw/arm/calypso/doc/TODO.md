---
name: TODO — temporary hacks with retraction criteria
description: Per CLAUDE.md rule #1, all env-gated hacks listed here with explicit removal criterion.
type: project
---

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
