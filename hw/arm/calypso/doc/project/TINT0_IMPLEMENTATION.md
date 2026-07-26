# TINT0 Master Clock Implementation Guide

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [../DOC_CODE_AUDIT.md](../DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> En particulier : (a) le « timer par instruction » que ce guide propose de retirer **n'existe pas** dans `calypso_c54x.c` (les lignes ~2490-2520 sont des sondes de debug, pas un décrément de TIM) ; (b) la constante frame-sync est `C54X_INT_FRAME_VEC=19 / C54X_INT_FRAME_BIT=3` (INT3, ligne TPU), **pas** 2/1 (SINT17) ; (c) même une fois TINT0 câblé, le DSP ne « démarre pas le traitement des trames » : il déraille (POST-BOOTSTUB-RET, PC=0x0000) et le vrai verrou est le handshake go-live ARM→DSP (api_write_cb jamais enregistré).

## Context

The Calypso GSM phone has a single master clock: **TINT0** (Timer 0 Interrupt of the TMS320C54x DSP). On real hardware, the C54x Timer 0 (registers TIM/PRD/TCR at DSP addresses 0x0024-0x0026) runs off the 13 MHz oscillator and fires TINT0 every 4.615 ms (one TDMA frame). TINT0 is the heartbeat that synchronizes everything.

## Architecture

```
TINT0 (C54x Timer 0, 4.615ms)
  |
  +-- DSP: IFR bit 4 set -> if IMR bit 4 enabled and IDLE, wake DSP
  |        DSP ROM dispatcher reads d_dsp_page, processes GSM tasks
  |
  +-- TPU: frame sync signal -> burst scheduling for next frame
  |
  +-- ARM: TPU_FRAME IRQ (IRQ4) -> firmware L1 scheduler runs
  |        API IRQ (IRQ15) -> firmware reads DSP results
  |
  +-- UART: poll TX/RX for sercomm (L1CTL + burst data)
```

## Current State

### Done
- `calypso_tint0.c` / `calypso_tint0.h` created as dedicated TINT0 module
- QEMU timer fires at 4.615ms, calls `calypso_tint0_do_tick(fn)` in calypso_trx.c
- `calypso_tdma_hw.c/.h` removed (was a slave, never connected)
- Build passes, everything compiles

### What Still Needs Work

1. **DSP internal timer (TIM/PRD/TCR) should be driven by TINT0, not by instruction count**
   - ~~Currently: `calypso_c54x.c` has a per-instruction timer tick in the main exec loop (lines ~2490-2520) that decrements TIM every instruction cycle.~~ — **FAUX (audit 2026-07-01)** : ce décrément n'existe pas. Les lignes 2480-2530 de `calypso_c54x.c` sont des sondes de debug (`DISPVAL-WR` / `SLOT4387-WR` / `SEED-WR` / `NDB-CTL-WR`), pas un timer. TIM n'est touché qu'à 4 endroits : init reset (`calypso_c54x.c:13427` = 0xFFFF), lecture (`:2181`), reload TRB depuis PRD (`:3035`) et écriture directe (`:3042`). Aucun décrément par instruction, aucun incrément FN sur expiration TIM.
   - Should be: TINT0 fires the DSP Timer 0 directly. The DSP ROM configures PRD/TCR but the actual tick comes from the QEMU timer, not from counting instructions.
   - ~~The per-instruction timer tick in c54x.c should be removed.~~ **DONE / SANS OBJET** : il n'y a rien à retirer (le code décrit n'existe pas).

2. **Remove duplicate timer logic in c54x main loop**
   - File: `/opt/GSM/qemu-calypso/hw/arm/calypso/calypso_c54x.c`
   - ~~Lines ~2490-2520: the timer tick that decrements TIM/PRD per instruction~~ — **FAUX** : sondes de debug (voir point 1).
   - ~~Also lines ~2515-2520: GSM frame number increment tied to TIM expiry~~ — **FAUX** : aucun incrément FN lié au timer dans `calypso_c54x.c`.
   - ⚠️ Prémisse invalide : il n'y a pas de « duplicate timer logic » à supprimer.

3. **TINT0 should directly set IFR bit 4 on the DSP**
   - In `calypso_tint0_do_tick()` or `calypso_tint0.c`:
     ```c
     s->dsp->ifr |= (1 << TINT0_IFR_BIT);  // bit 4
     ```
   - Then if `!(s->dsp->st1 & ST1_INTM) && (s->dsp->imr & (1 << 4))`:
     - Branch to interrupt vector TINT0_VEC (20) = IPTR*128 + 20*4
   - If DSP is IDLE: wake it

4. **Frame number should come from TINT0, not from DSP timer**
   - `calypso_tint0_fn()` is the authoritative frame number
   - Remove FN increment from c54x timer tick
   - The DSP reads FN from API RAM (written by ARM firmware via `dsp_end_scenario()`)

5. **Verify interrupt vector for TINT0**
   - TINT0 = IFR/IMR bit 4, vector 20 (offset 0x50 from IPTR base)
   - ~~Currently `C54X_INT_FRAME_VEC=2, C54X_INT_FRAME_BIT=1` in calypso_c54x.h - this is **SINT17**~~ — **FAUX (audit 2026-07-01)** : le code définit `C54X_INT_FRAME_VEC=19 / C54X_INT_FRAME_BIT=3` (`calypso_c54x.h:126-127`), soit **INT3** (vec = 3+16 = 19). Le commentaire d'en-tête (`calypso_c54x.h:120-125`) précise que INT3 est la ligne frame-sync externe du TPU — la seule interruption « frame » que le firmware démasque réellement (IMR observé = 0xFF88, bit 3 armé). Ce n'est ni 2/1 ni SINT17.
   - INT3 (vec 19, bit 3) est la ligne frame-sync du TPU — c'est celle que le firmware unmask.
   - TINT0 (vec 20, bit 4) is the DSP's own timer interrupt.
   - Both exist on real hardware. INT3/TPU tells DSP "new frame data ready". TINT0 tells DSP "4.615ms elapsed".
   - The DSP ROM uses TINT0 internally for timing, and INT3 (TPU frame-sync) for frame dispatch.

## Files

| File | Role |
|------|------|
| `calypso_tint0.c` | QEMU timer, fires every 4.615ms, calls do_tick |
| `calypso_tint0.h` | Constants (TINT0_PERIOD_NS, IFR_BIT, VEC, etc.) |
| `calypso_trx.c` | `calypso_tint0_do_tick(fn)` - DSP run, SINT17, IRQs, UART |
| `calypso_c54x.c` | DSP emulator - remove internal timer tick |
| `calypso_c54x.h` | DSP state struct, interrupt constants |

## Docker

- Container: `osmo-operator-1` (l'ancien nom `trying` n'existe plus)
- Source: `/opt/GSM/qemu-calypso/hw/arm/calypso/` (alias : `/opt/GSM/qemu-src/`)
- Build: `cd /opt/GSM/qemu-calypso/build && ninja`
- Accès code : `docker exec osmo-operator-1 <cmd>`

## Key Constants

```c
#define TINT0_PERIOD_NS  4615000ULL  // 4.615 ms
#define TINT0_IFR_BIT    4           // IFR/IMR bit 4
#define TINT0_VEC        20          // Interrupt vector 20
#define TINT0_TIM_ADDR   0x0024      // DSP Timer counter
#define TINT0_PRD_ADDR   0x0025      // DSP Timer period
#define TINT0_TCR_ADDR   0x0026      // DSP Timer control
#define GSM_HYPERFRAME   2715648     // GSM hyperframe modulus
```

## Current DSP Status

> ⚠️ **Vérité-terrain (audit 2026-07-01)** — remplace l'optimisme ci-dessous.

- DSP boots, SP stable at 0x5AC6, executes real ROM code
- F272=RPTBD and F274=CALLD correctly decoded
- d_dsp_page WR=0x0002 received from ARM
- ~~DSP running init code at PC=0x1200-0x1C00 (DARAM), IMR=0x0000~~ — IMR=0x0000 **toute la durée du run** : ré-armé nulle part après le clear de boot (@0xb37e). Le DSP finit par dérailler (POST-BOOTSTUB-RET, PC=0x0000).
- ~~DSP never reaches IDLE - init loop may be waiting for TINT0 to fire~~ — le vrai blocage n'est pas TINT0.
- ~~Once TINT0 properly sets IFR bit 4, the DSP should detect it, configure IMR, and start processing frames~~ — **FAUX** : même TINT0 câblé ne débloque rien. IMR=0x0000 (bit 4 masqué), donc l'IFR ne peut pas être servi. Le vrai verrou est le **handshake go-live ARM→DSP** : l'ARM n'écrit que 0x0000 dans l'API 0x0314/0x0318, et le hook `api_write_cb` est déclaré (`calypso_c54x.h`) mais **jamais assigné** (`grep 'api_write_cb *=' = 0`). Tant que ce handshake ne s'arme pas, d_fb_det reste 0 et le DSP ne traite aucune trame.
