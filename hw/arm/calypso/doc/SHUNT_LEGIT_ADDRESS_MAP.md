# Reference SHUNT_LEGIT — Mapping des cellules DSP Calypso (option 3)

## 1. Resume

En mode **SHUNT_LEGIT** (`CALYPSO_SHUNT_LEGIT=1`, dit "option 3"), gr-gsm decode le vrai downlink GSM et le shunt QEMU injecte les resultats dans les cellules de l'API RAM du DSP **au format natif**, de sorte que le firmware ARM (osmocom-bb) campe normalement sur la cellule (C3 camped).

**FAIT CENTRAL** : le firmware ARM lit les resultats DSP dans le tableau `s->dsp->data[]`, **PAS** dans `dsp_ram[]` ni directement dans `api_ram[]`.

- Chemin de lecture ARM (MMIO) : `val = s->dsp->data[offset/2 + 0x0800]` (`calypso_trx.c:225`).
- Chemin d'ecriture ARM (miroir) : `s->dsp->data[offset/2 + 0x0800] = value` en plus de `dsp_ram[offset/2]` (`calypso_trx.c:522, :548`).

Consequence pratique : un shunt fonctionnel ecrit **directement** dans `data[]` / `api_ram[]` (on_frame_tick + FORCE c54x) et intercepte les reads ARM (real_fb_read + overrides trx). Les `shunt_dispatch_*` du helper, qui passent par `dma_memory_write` vers `dsp_ram[]`, ne sont **PAS** vus par le firmware pour les cellules a_cd/a_sch (voir Pieges).

Config firmware confirmee : **DSP=36**, CHIPSET=12, ANLG_FAM=2 (Iota), W_A_DSP_IDLE3=1 — ces flags fixent le layout NDB et l'ordre a_serv_demod avant a_pm.

---

## 2. Espaces d'adressage & conversions

Trois vues d'une meme API RAM (mailbox DSP <-> ARM) :

| Espace | Base | Unite | Ou dans le code |
|---|---|---|---|
| **ARM phys** | `0xFFD00000` (`CALYPSO_DSP_BASE`) | octet | MMIO `calypso_dsp_read/write`, `offset = addr - base` |
| **DSP data word** | `0x0800` (`C54X_API_BASE`) | mot 16 bits | `s->dsp->data[]` (source de verite lue par le firmware) |
| **api_ram index** | `0` (0-based) | mot 16 bits | `s->dsp->api_ram[]` (= `&dsp_ram[]`) |

### Formules exactes (les deux sens)

Soit `off = ARM_addr - 0xFFD00000` (offset ARM en octets) :

```
DSP data word  = off/2 + 0x0800
api_ram index  = off/2                       (= DSP word - 0x0800)
ARM offset     = (DSP word - 0x0800) * 2
ARM addr       = 0xFFD00000 + off
api_ram index  = DSP word - C54X_API_BASE    (= DSP word - 0x0800)
```

`CALYPSO_DSP_SIZE = 64 KiB` ; `C54X_API_SIZE = 0x2000` (8 K mots).
Ecriture 16 bits little-endian (`cpu_to_le16`) via `shunt_write_w`.

### Exemples chiffres (croisent le fait central)

- `d_fb_det` : NDB+0x48 -> ARM `0xFFD001F0` (off 0x1F0) -> DSP word `0x08F8` -> api_ram `0x0F8`.
- `a_sync_demod[TOA/PM/ANGLE/SNR]` : ARM `0x1F4/1F6/1F8/1FA` -> DSP `0x08FA..0x08FD`.
- `a_serv_demod[0]` (SB TOA), R_PAGE_0+0x10 : off 0x60 -> DSP `0x0830` -> `data[0x830]`. Page1 : off 0x88 -> DSP `0x0844`.
- `a_pm[0]`, R_PAGE_0+0x18 : off 0x68 -> DSP `0x0834` -> `data[0x834..0x836]`. Page1 : off 0x90 -> DSP `0x0848..0x084A`.
- `a_cd[0]`, NDB+0x1FC : off 0x3A4 -> DSP `0x09D2`. `a_cd[3]` (debut SI) = +6 octets -> DSP `0x09D5`.

---

## 3. Bases de pages

Depuis `calypso_dsp_internal.h:20-24`, converties dans les trois espaces :

| Zone | Role | ARM phys | off octet | DSP word | api_ram idx |
|---|---|---|---|---|---|
| **W_PAGE_0** | MCU->DSP, 20 mots | `0xFFD00000` | 0x000 | 0x0800 | 0x000 |
| **W_PAGE_1** | MCU->DSP page 1 | `0xFFD00028` | 0x028 | 0x0814 | 0x014 |
| **R_PAGE_0** | DSP->MCU, 20 mots | `0xFFD00050` | 0x050 | 0x0828 | 0x028 |
| **R_PAGE_1** | DSP->MCU page 1 | `0xFFD00078` | 0x078 | 0x083C | 0x03C |
| **NDB** | persistant, 268 mots | `0xFFD001A8` | 0x1A8 | 0x08D4 | 0x0D4 |

NDB idx 0x0D4 confirme (`calypso_trx.c:912` : `api_ram[0x08D4 - C54X_API_BASE]`).

### Offsets de champ intra-page

**Write page** `T_DB_MCU_TO_DSP` (`WP_*`, byte) : `d_task_d`=0x00, `d_burst_d`=0x02, `d_task_u`=0x04, `d_burst_u`=0x06, **`d_task_md`=0x08**, `d_task_ra`=0x0E, `d_fn`=0x10, `d_ctrl_system`=0x20.
(En index de mot dans trx.c : `DB_W_D_TASK_D`=0, `D_BURST_D`=1, `D_TASK_MD`=4, `D_TASK_RA`=7.)

**Read page** `T_DB_DSP_TO_MCU` (`RP_*`, byte) : `d_task_d`=0x00, `d_burst_d`=0x02, `d_task_md`=0x08, **`a_serv_demod`=0x10** (4 mots TOA/PM/ANGLE/SNR), **`a_pm`=0x18** (3 mots), `a_sch`=0x1E (5 mots).

**NDB** `T_NDB_MCU_DSP` (`NDB_*`, byte) : `d_dsp_page`=0x00, `d_error_status`=0x02, **`d_fb_det`=0x48**, `d_fb_mode`=0x4A, **`a_sync_demod`=0x4C** (4 mots), `a_sch26`=0x54, **`a_cd`=0x1FC** (15 mots), `a_dd_0`=0x238 (TCH DL), `a_du_1`=0x134 (TCH UL).

Sous-champs demod (`l1_environment.h:259-262`) : **D_TOA=0, D_PM=1, D_ANGLE=2, D_SNR=3**.
Layout DSP 33-36 (`dsp_api.h:96`, branche compilee) : read page = `... d_task_md(4) ... a_serv_demod[4] (8..11) AVANT a_pm[3] (12..14) AVANT a_sch[5] (15..19)`.
Bits SCH/FIRE (`l1_environment.h`) : B_FIRE0=5, B_FIRE1=6, **B_SCH_CRC=8** (1=ERREUR).
Page-toggle NDB `d_dsp_page` : `B_GSM_PAGE=1<<0` (numero de page), `B_GSM_TASK=1<<1` (nouvelle tache). Valeur `0x0002`/`0x0003` = `B_GSM_TASK | page`.

---

## 4. Grand tableau de reference des cellules

| Champ | DSP word | ARM addr | api_ram idx | Struct/offset | R/W firmware | Role camp | Ecrit par (fichier:ligne) |
|---|---|---|---|---|---|---|---|
| **d_fb_det** | 0x08F8 | 0xFFD001F0 | 0x0F8 | NDB+0x48 | R (poll FOUND=1) | FB detecte | shunt.c:541 (W=1) ; c54x.c:2449 (FORCE=1) ; helper.c:163 ; read shunt.c:1194 |
| d_fb_mode | 0x08F9 | 0xFFD001F2 | 0x0F9 | NDB+0x4A | W firmware | mode FB (wideband) | (firmware, non shunte) |
| **a_sync_demod TOA** | 0x08FA | 0xFFD001F4 | 0x0FA | NDB+0x4C (+D_TOA) | R (prim_fbsb.c:308) | SB timing | shunt.c:542 (sb_toa) ; helper.c:164 ; read shunt.c:1195 (rx_toa) |
| a_sync_demod PM | 0x08FB | 0xFFD001F6 | 0x0FB | NDB+0x4C+2 (D_PM) | R (fbsb.c:309 >>3) | rxlev SB | shunt.c:543 (last_pm) ; helper.c:165 ; read shunt.c:1196 |
| a_sync_demod ANGLE | 0x08FC | 0xFFD001F8 | 0x0FC | NDB+0x4C+4 (D_ANGLE) | R (fbsb.c:310) | AFC | shunt.c:544 (rx_afc) ; helper.c:166 ; read shunt.c:1197 |
| a_sync_demod SNR | 0x08FD | 0xFFD001FA | 0x0FD | NDB+0x4C+6 (D_SNR) | R (fbsb.c:311) | qualite SB | shunt.c:545 (0x7000) ; helper.c:167 ; read shunt.c:1198 |
| a_sch26[0..4] | 0x08FE..0x0902 | 0xFFD001FC.. | 0x0FE..0x102 | NDB+0x54 | R (SB monitor) | SB NDB monitor | (firmware) |
| **a_serv_demod[TOA]** P0 | 0x0830 | 0xFFD00060 | 0x030 | RP0+0x10 (+D_TOA) | R (rx_nb.c:93 ; fbsb.c:152) | NB/SB timing | shunt.c:562/602 ; helper.c:250/536 ; read shunt.c:1200 |
| a_serv_demod[PM] P0 | 0x0831 | 0xFFD00062 | 0x031 | RP0+0x12 (D_PM) | R (rx_nb.c:95 >>3) | rxlev NB | shunt.c:562/602 ; helper.c:251/537 |
| a_serv_demod[ANGLE] P0 | 0x0832 | 0xFFD00064 | 0x032 | RP0+0x14 (D_ANGLE) | R (rx_nb.c:97) | AFC NB | shunt.c:562/602 ; helper.c:252/538 |
| a_serv_demod[SNR] P0 | 0x0833 | 0xFFD00066 | 0x033 | RP0+0x16 (D_SNR) | R (rx_nb.c:98) | qualite NB | shunt.c:602 (0x7000) ; helper.c:253/539 |
| **a_pm[0..2]** P0 | 0x0834..0x0836 | 0xFFD00068..6C | 0x034..0x036 | RP0+0x18 | R (prim_pm.c:58,201 >>3) | rxlev PM MEAS | c54x.c:2472 (FORCE=apm) ; helper.c:582-584 |
| a_sch[0] (CRC) P0 | 0x0837 | 0xFFD0006E | 0x037 | RP0+0x1E | R (fbsb.c:187 bit8) | SB CRC pass | helper.c:234 (=0) |
| a_sch[1..2] P0 | 0x0838/0x0839 | 0xFFD00070/72 | 0x038/039 | RP0+0x20/22 | R | SB | helper.c:245-246 (=0) |
| a_sch[3] (SB lo) P0 | 0x083A | 0xFFD00074 | 0x03A | RP0+0x24 | R (fbsb.c:204) | SB payload lo | helper.c:241 (sb&0xFFFF) |
| a_sch[4] (SB hi) P0 | 0x083B | 0xFFD00076 | 0x03B | RP0+0x26 | R (fbsb.c:204) | SB payload hi | helper.c:242 (sb>>16) |
| a_serv_demod[TOA] P1 | 0x0844 | 0xFFD00088 | 0x044 | RP1+0x10 | R | NB/SB P1 | shunt.c:563/603 ; read shunt.c:1200 |
| a_serv_demod[PM/ANG/SNR] P1 | 0x0845..0x0847 | 0xFFD0008A..8E | 0x045..0x047 | RP1+0x12/14/16 | R | rxlev/AFC/SNR P1 | shunt.c:563/603 |
| a_pm[0..2] P1 | 0x0848..0x084A | 0xFFD00090..94 | 0x048..0x04A | RP1+0x18 | R (prim_pm.c) | rxlev PM P1 | c54x.c:2472 (FORCE=apm) |
| **d_task_d** P0 | 0x0828 | 0xFFD00050 | 0x028 | RP0+0x00 | R (rx_nb.c:77 garde !=0) | dispatch tache | trx.c:270 (0 -> 24 ALLC) ; helper.c:534 |
| d_task_d P1 | 0x083C | 0xFFD00078 | 0x03C | RP1+0x00 | R | dispatch tache P1 | trx.c:278 ; helper.c:534 |
| **d_burst_d** read P0 | 0x0829 | 0xFFD00052 | 0x029 | RP0+0x02 | R (rx_nb.c:83,113) | index burst | trx.c:283-311 ; helper.c:535 ; echo shunt.c:1470 |
| d_burst_d read P1 | 0x083D | 0xFFD0007A | 0x03D | RP1+0x02 | R | index burst P1 | trx.c:311 ; echo shunt.c:1471 |
| d_burst_d write P0 | 0x0801 | 0xFFD00002 | 0x001 | WP0+0x02 | W ARM | index burst emis | capture shunt.c:1468 (trx.c:457) |
| d_burst_d write P1 | 0x0815 | 0xFFD0002A | 0x015 | WP1+0x02 | W ARM | index burst emis P1 | capture shunt.c:1468 |
| d_task_md P0 | 0x082C | 0xFFD00058 | 0x02C | RP0+0x08 | R | mode tache | helper.c:168/256/585 (FB=5/SB=6/PM=1) |
| d_task_md P1 | 0x0840 | 0xFFD00080 | 0x040 | RP1+0x08 | R | mode tache P1 | helper.c:168 |
| **d_dsp_page** | 0x08D4 | 0xFFD001A8 | 0x0D4 | NDB+0x00 | R/W | toggle page | trx.c:822 (`dsp_page = val&1`) |
| **a_cd[0]** (FIRE/CRC) | 0x09D2 | 0xFFD003A0/3A4 | 0x1D2 | NDB+0x1FC | R (rx_nb.c:150) | BCCH/SI CRC pass | shunt.c:586 (0) ; helper.c:501 |
| a_cd[1] | 0x09D3 | 0xFFD003A6 | 0x1D3 | NDB+0x1FE | R | a_cd | shunt.c:587 (0) ; helper.c:503 |
| a_cd[2] (num_biterr) | 0x09D4 | 0xFFD003A8 | 0x1D4 | NDB+0x200 | R (rx_nb.c:144) | biterr | shunt.c:588 (0) ; helper.c:504 |
| a_cd[3..14] (SI L2 23o) | 0x09D5..0x09E0 | 0xFFD003AA..3C0 | 0x1D5..0x1E0 | NDB+0x202.. | R (rx_nb.c:156 memcpy 23) | payload SI/BCCH | shunt.c:591 ; helper.c:514 |
| d_ctrl_system (WP) | 0x0810 | 0xFFD00020 | 0x010 | WP0+0x20 | W ARM | go-live/ctrl | (voir memoire go-live) |

Note ARM addr a_cd[0] : le byte offset 0x1FC est l'offset **struct**. NDB base off 0x1A8 + champ ; DSP word 0x09D2 -> ARM `0xFFD00000 + (0x09D2-0x0800)*2 = 0xFFD003A4`. Certaines refs listent `0xFFD003A0` (base a_cd) ; l'ecriture reelle du shunt vise `0xFFD003A4`.

---

## 5. Chaine camp (par etage)

### FB — Frequency Correction Burst
- Cellules : `d_fb_det=0x08F8` (=1 FOUND), `a_sync_demod` TOA/PM/ANGLE/SNR `0x08FA..0x08FD`.
- Mecanismes cumulatifs :
  1. **on_frame_tick** (`calypso_dsp_shunt.c:541-545`) : ecrit directement `api_ram[0xF8..0xFD]`.
  2. **FORCE data-write c54x** (`calypso_c54x.c:2449`) : intercepte l'ecriture DSP de `data[0x08F8]` et force `val=1` (ecrase le clobber natif =0), gate `SHUNT_LEGIT && sb_valid`.
  3. **Intercept read** (`calypso_dsp_shunt.c:1194-1198`, appele `trx.c:262`) : le read ARM des offsets 0x01F0/1F4/1F6/1F8/1FA retourne `rx_fb_det/rx_toa/last_pm/rx_afc/rx_snr`.
- `feed_iq` (gate REAL_FB) peuple `g_shunt.rx_fb_det/rx_snr/rx_afc/rx_toa=23` (shunt.c:1240-1243).

### SB — Synchronization Burst (SCH)
- Cellules NDB : memes `a_sync_demod` que FB. Cellules read-page : `a_sch[0]=0x0837` (CRC pass=0), `a_sch[3..4]=0x083A/0x083B` (payload SB = t1/t2/t3+BSIC encode), `a_serv_demod[0x0830..0x0833]` (TOA/PM/ANGLE/SNR).
- Mecanismes : `shunt_dispatch_sb(page)` (`calypso_dsp_helper.c:202-256`, gate `INJECT_SB || SHUNT_LEGIT`) + ecritures directes on_frame_tick + intercept read a_serv_demod TOA `0x0830/0x0844` (shunt.c:1200, ssi sb_valid). `sb = a_sch[3] | a_sch[4]<<16` (prim_fbsb.c:204).

### rxlev — a_pm (PM MEAS)
- Cellules : `a_pm[0..2]` P0 `0x0834..0x0836`, P1 `0x0848..0x084A`.
- Mecanisme : **FORCE data-write c54x** (`calypso_c54x.c:2472`) force `a_pm = calypso_trf6151_apm_for_rf(target -60 dBm)`, gate `TRF_RXLEV || SHUNT_LEGIT`. Aussi `shunt_dispatch_pm` (helper.c:582-585, d_task_md=PM_DSP_TASK=1).
- ATTENTION divergence : le bloc on_frame_tick section A ecrit sous label "a_pm" les idx 0x30/0x44 qui sont en fait **a_serv_demod** (off 0x60/0x88), PAS a_pm (idx 0x34/0x48). Seuls c54x.c:2472 et dispatch_pm visent la vraie cellule a_pm.

### BCCH/CCCH/SI — a_cd
- Cellules : `a_cd[0]=0x09D2` (CRC/FIRE pass=0), `a_cd[2]=0x09D4` (biterr=0), `a_cd[3..14]=0x09D5..0x09E0` (23 octets L2 = SI3/SI4/SI1/SI2...).
- Mecanisme : **ecriture directe `data[]`** dans on_frame_tick (`calypso_dsp_shunt.c:586-591`), gate `si_valid`, rotation SI toutes les 8 ticks. Empaquetage : `si[i] | (si[i+1]<<8)`. Le firmware lit via `dsp_memcpy_from_api(..., &ndb->a_cd[3], 23, 0)` (prim_rx_nb.c:156).
- `feed_si` (gate `FEED_SI || SHUNT_LEGIT`, shunt.c:1491-1493) range les SI par type dans `g_shunt.si_set[slot]` (l2[2]=0x19..0x1e) et positionne `si_valid`.
- IMPORTANT : `shunt_dispatch_allc` ecrit a_cd via `dma_memory_write` vers `dsp_ram[]` qui n'est PAS mirroir vers `data[]` -> **non vu par le firmware**. Seule l'ecriture directe `data[]` de on_frame_tick fait camper.

### d_task_d + d_burst_d — dispatch/scheduling
- `d_task_d` (`0x0828/0x083C`) : intercept read trx.c:270-278, gate `SHUNT_LEGIT && si_valid` — si val==0 retourne 24 (ALLC_DSP_TASK) et avance `s_burst_ctr`. Le firmware garde `db_r->d_task_d != 0` (prim_rx_nb.c:77).
- `d_burst_d` (`0x0829/0x083D`) : intercept read trx.c:283-311. Mode 0 (defaut) = fixe 3 ; mode 2 = lockstep `s_burst_ctr+OFS` (gate BURST_ECHO/BURST_OFS). Write-page mirror `wp_burst_write` (shunt.c:1468, gate `BURST_PERCMD` defaut ON) : capture `d_burst_d = val&3` puis echo vers RP P0 `0xFFD00052` et P1 `0xFFD0007A`.

---

## 6. TRF6151 — gain RF & rxlev (a_pm)

Constantes (`calypso_trf6151.c:27-38`) :
```
SYSTEM_INHERENT_GAIN = 71     (rffe_dualband.c)
TRF6151_FE_GAIN_LOW  = 7
TRF6151_FE_GAIN_HIGH = 27
TRF6151_VGA_GAIN_MIN = 14
RX_VGA_GAIN_SHIFT    = 11      (REG_RX bits[15:11])
TRF6151_REG_RX       = 0       (adresse registre)
TRF6151_REG_RX_RESET = 0x9E00  (gain total reset = 138)
```

Decodage REG_RX (`trf6151_gain_from_reg`, l.43-60) :
```
FE  = (reg_rx >> 9) & 3        // bits[10:9] : 0 -> +7 (LOW), 3 -> +27 (HIGH), autres -> +0
vga = (reg_rx >> 11) & 0x1f    // bits[15:11], clampe >= 6
gain_trf   = FE + VGA_GAIN_MIN + (vga - 6) * 2
total_gain = SYSTEM_INHERENT_GAIN + gain_trf     // = 71 + gain_trf
```
Reset 0x9E00 : FE=3 -> +27 ; vga=19 -> 14+(19-6)*2=40 -> gain_trf=67 -> total=138.

Formule a_pm (chaine inverse du firmware agc.c / prim_pm.c) :
```
firmware : pm_level = a_pm >> 3 ; bb_dbm = pm_level/8 ; rf_dbm = bb_dbm - total_gain
=> a_pm = bb_dbm * 64 = (target_rf_dbm + total_gain) * 64
```
`calypso_trf6151_apm_for_rf(target_rf_dbm)` : `bb_dbm = target + total_gain` (clampe >=0), `apm = bb_dbm*64` (clampe <=0xFFFF).

Suivi de gain vivant : chaque write TSP REG_RX (dev 1, `calypso_trf6151_tsp_write`) met a jour `g_reg_rx`, donc a_pm se recalcule quand l'AGC baisse le gain -> le RF cible est tenu quel que soit le gain choisi.

---

## 7. Pieges & lecons

1. **data[] vs api_ram[] vs dsp_ram[] — jamais le mauvais array, seulement le mauvais offset.** Le firmware lit `s->dsp->data[off/2+0x800]`. Ecrire via `0xFFD00xxx` (STR ARM ou `dma_memory_write`) retombe sur le callback MMIO `calypso_dsp_write` qui mirroir vers `data[]` — donc l'array est bon. MAIS `shunt_dispatch_*` ecrit certaines cellules (a_cd/a_sch) via `dma_memory_write` vers `dsp_ram[]` qui n'est PAS mirroir vers `data[]` : ces ecritures ne sont pas vues. Le camp fonctionnel repose sur les ecritures **directes** `data[]`/`api_ram[]` (on_frame_tick + FORCE c54x) + les intercepts read, pas sur les dispatch helper.

2. **a_pm : mot 8 vs mot 12.** Dans la read page, `a_serv_demod` commence au mot 8 (`RP_A_SERV_DEMOD=0x10`, DSP `0x830`), `a_pm` au mot 12 (`RP_A_PM=0x18`, DSP `0x834`), juste apres les 4 mots de a_serv_demod. Ecrire le rxlev PM au mot 8 le met dans `a_serv_demod[TOA]` (lu par la demod NB, mauvais champ) au lieu de `a_pm[0]` (lu par pm_resp). Le bloc on_frame_tick section A confond justement les deux sous le label "a_pm" (ecrit 0x30/0x44). Seuls `c54x.c:2472` et `dispatch_pm` visent la vraie cellule a_pm (0x834/0x848).

3. **Double-read d_burst_d.** Si le firmware relit toujours R_PAGE_0 sans flip de page, il relit la page precedente (sonde `R_PAGE_SPLIT`). Le mirror `wp_burst_write` echo la valeur ecrite dans les deux read-pages (P0 0xFFD00052 + P1 0xFFD0007A) pour tenir le lockstep. Mode d_burst_d : mode 0 fixe=3, mode 2 lockstep `s_burst_ctr`.

4. **AFC drift si a_serv_demod garbage.** `a_sync_demod[ANGLE]` (0x08FC) et `a_serv_demod[ANGLE]` (0x0832) alimentent la boucle AFC. Si ces cellules contiennent du garbage (non forcees a 0 / rx_afc valide), l'AFC derive et casse le camp. Toujours poser ANGLE=rx_afc (FB/SB) ou 0 (NB), et SNR=0x7000 pour signaler bonne qualite.

5. **NDB_D_DSP_PAGE ambigu selon header.** `0x08E2` (fbsb.h, DSP word absolu — CONTESTE, en fait d_dsp_state=3) vs `0x00` (dsp_internal.h, offset depuis NDB -> vrai d_dsp_page = DSP word `0x08D4`). Utiliser `0x08D4`.

6. **PROM0.bin base = 0x7000, PAS 0x8000** — toute lecture statique du .bin doit etre validee contre les logs runtime.

---

## 8. Variables d'environnement (recap)

| Variable | Defaut | Effet |
|---|---|---|
| `CALYPSO_SHUNT_LEGIT` | 0 | Active option 3 : ecritures natives + intercepts (FB/SB/a_pm/a_cd/d_task_d). Implique FEED_SI, REAL_FB, INJECT_SB. |
| `CALYPSO_SHUNT_REAL_FB` | 0 | Active `real_fb_read` (intercept read FB/a_sync) + feed_iq peuplant rx_*. Implique par LEGIT. |
| `CALYPSO_SHUNT_FEED_SI` | 0 | feed_si : range SI L2 par type dans si_set, pose si_valid. Implique par LEGIT. |
| `CALYPSO_INJECT_SB` | 0 | Active `shunt_dispatch_sb`. Implique par LEGIT. |
| `CALYPSO_SHUNT_BURST_PERCMD` | 1 (ON) | Mirror write-page d_burst_d -> echo read-pages P0/P1. |
| `CALYPSO_SHUNT_BURST_ECHO` / `_OFS` | - | Mode 2 d_burst_d (lockstep s_burst_ctr + OFS). |
| `CALYPSO_TRF_RXLEV` | - | Force a_pm=trf6151 meme hors LEGIT. |
| `CALYPSO_SHUNT_PM` | - | Valeur a_pm canned pour dispatch_pm. |
| `CALYPSO_TPU_RX_WIRE` | 0 | Wire `data[0x3f92] |= 0x0800` quand DMA task_md==5. Independant de LEGIT. |
| `CALYPSO_ARM2DSP_CTRLSYS` | (voir memoire) | Wire ARM d_ctrl_system/0x0810 (go-live, bit15). |
| `HACK` | 0 (natif) | 0=natif ; calypso_hack.env = bequilles. |

Fichiers de reference (absolus, conteneur `osmo-operator-1`) :
- `${QEMU_TREE}/hw/arm/calypso/calypso_dsp_shunt.c`
- `${QEMU_TREE}/hw/arm/calypso/calypso_dsp_helper.c`
- `${QEMU_TREE}/hw/arm/calypso/calypso_c54x.c` (+ `.h` : C54X_API_BASE 0x0800, C54X_API_SIZE 0x2000)
- `${QEMU_TREE}/hw/arm/calypso/calypso_trx.c` (MMIO read/write, miroir off/2+0x800, DMA proof l.905-921)
- `${QEMU_TREE}/hw/arm/calypso/calypso_trf6151.c` / `.h`
- `${QEMU_TREE}/include/hw/arm/calypso/calypso_dsp_internal.h` (bases pages, offsets WP_/RP_/NDB_, task IDs)
- `${QEMU_TREE}/include/hw/arm/calypso/calypso_trx.h`
- `${GSM_ROOT}/osmocom-bb-transceiver/src/target/firmware/include/calypso/dsp_api.h`
- `${GSM_ROOT}/osmocom-bb-transceiver/src/target/firmware/include/calypso/l1_environment.h`
- `${GSM_ROOT}/osmocom-bb-transceiver/src/target/firmware/layer1/prim_fbsb.c`, `prim_pm.c`, `prim_rx_nb.c`
---

## 9. Chaine UPLINK / Location Update (2026-07-26)

Le mobile s'ENREGISTRE (LU ACCEPT + TMSI + normal service) via SHUNT_LEGIT. Cellules
et chemins de la chaine uplink + reponses dediees downlink.

### Cellules UL

| Champ | DSP word | ARM byte | Struct | Role | Ecrit/lu |
|---|---|---|---|---|---|
| **d_rach** | 0x0A3A | 0x0474 | NDB word 0x023A | RACH UL : `(ra<<8)|(bsic<<2)` | firmware ecrit -> hook trx.c |
| **d_task_u** | write page word 2 | WP+0x04 | db_w | commande NB UL (SDCCH/SACCH UL) | firmware |
| **d_task_ra** | write page word 7 | WP+0x0E | db_w | commande RACH (AVALEE par shunt) | firmware (inutilisee cote shunt) |
| **a_cu** (SDCCH UL L2) | ~0x0A09 | BASE_API_NDB+0x264+ofs | NDB | 23o LAPDm UL (SABM/I-frames) | firmware ecrit -> shunt lit |

- `d_rach` : hook write dans `calypso_dsp_write` (trx.c, offset 0x0474, env `CALYPSO_NDB_D_RACH_OFFSET` def 0x023A). Signal FIABLE (1 write = 1 RACH), remplace la voie `d_task_ra` avalee par le shunt.
- `a_cu` : fenetre lue par `shunt_latch_task` (scan debut de trame LAPDm, `CALYPSO_UL_ACU_OFS` def 6) -> `calypso_sdcch_ul_publish` -> sideband `/dev/shm/calypso_sdcch_ul` -> qemu_wrap encode NB + inject osmo-bts.

### Reutilisation a_cd pour le DL dedie

Le meme `a_cd` (NDB_A_CD=0x1FC -> **data[0x9D2..0x9E0]**) porte, selon le bloc :
- **BCCH** : SI1-4 (camp) — fn%51 {2-5}.
- **AGCH/PCH** : IMM ASSIGN / PAGING — CCCH fn%51 {6-9,12-19} (`dispatch_allc` branche AGCH).
- **SDCCH/4 SS0 DL** : UA / AUTH REQ / IDENTITY REQ / LU ACCEPT — fn%51 {22-28} sur burst_d 0..3 (`dispatch_allc` branche SDCCH).

⚠️ **Garde clobber** : le bloc camp (on_frame_tick, dsp_shunt.c:577) ne reecrit PAS le SI dans data[0x9D2] quand `sdcch_valid || agch_valid` -> sinon le UA/IMM-ASSIGN est ecrase avant lecture firmware -> SABM jamais confirme.

### Chemins host-side

| Sens | Chemin |
|---|---|
| RACH/SDCCH UL | firmware -> d_rach/a_cu -> `calypso_bsp_send_ul` -> UDP 127.0.0.1:**5702** -> `qemu_wrap.c` (g_bsp_fd) -> encode Laurent/GMSK + inject g_rach_iq/g_ul -> osmo-trx-ipc -> osmo-bts |
| AGCH/SDCCH/SACCH DL | osmo-bts -> gr-gsm `grgsm_decode -m BCCH_SDCCH4` -> si_bridge.py (GSMTAP 0x07 SDCCH4 / AGCH) -> UDP **4730** -> QEMU `feed_agch`/`feed_sdcch`/`feed_sacch` -> a_cd |

Gates (fallback SHUNT_LEGIT) : `CALYPSO_INJECT_AGCH/SDCCH/SACCH`, `CALYPSO_UL_RACH_FROM_DRACH` (def=SHUNT_LEGIT), `CALYPSO_SHUNT_SDCCH_OFS` (timing bloc SDCCH DL).

### Chiffrement A5/1 (piege)

`/dev/shm/calypso_kc` (Kc + algo). `qemu_wrap.c` XORe les bits data UL (3-59/88-144, midamble TSC intact) si `n_a5>0`. Une LU FRAICHE (pre-auth) doit etre EN CLAIR -> **purger le Kc au demarrage** (`rm -f /dev/shm/calypso_kc` dans run.sh/run-all.sh) ; un vrai CIPHER MODE COMMAND reecrira un Kc frais legitime. `calypso_kc_read` ne testait que seq==0 -> ressortait un Kc perime -> SABM chiffre illisible -> pas de UA. Env test : `CALYPSO_CIPH_A5=0` (ne PAS laisser permanent).

### Statut

LU ACCEPT + registered OK, mais **INTERMITTENT** (~1 succes / 19 retries T3211). Robustesse a ameliorer : alignement bloc SDCCH DL, timing a_cu UL, fenetre de presentation.
