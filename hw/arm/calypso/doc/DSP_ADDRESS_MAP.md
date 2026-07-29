# CARTE DES ADRESSES DSP — Calypso TMS320C54x émulé (QEMU osmo-operator-1)

> Date : 2026-07-26
>
> Ce document est la carte de référence des adresses de l'API RAM DSP (DARAM partagée ARM↔DSP) et des cellules ROM/handshake go-live du firmware DSP émulé. Tout le contenu est vérifié contre `dsp_api.h`, `calypso_dsp_internal.h`/`calypso_fbsb.h`, le code shunt/bsp/c54x, et le désassemblage ROM `calypso_dsp.txt`.

Convention d'adressage. L'API RAM est une DARAM partagée ARM↔DSP. L'ARM la voit en octets à partir de `0xFFD00000` ; le DSP la voit en **mots** à partir de `0x0800`. Relation vérifiée dans le code (`calypso_dsp_shunt.c:377`, `calypso_arm2dsp.c:44/256`, `calypso_c54x.h:21`) :

```
DSP_word = 0x0800 + (ARM_addr - 0xFFD00000) / 2          [C54X_API_BASE = 0x0800]
api_ram[i] correspond au DSP_word (0x0800 + i)            (accès ARM sans intercept)
```

Bases des 5 régions (ARM → DSP word), source `dsp_api.h:18-23` == `calypso_dsp_internal.h:21-25` :

| Région | Base ARM | Taille | Base DSP word | Rôle |
|--------|----------|--------|---------------|------|
| Write page 0 (MCU→DSP) | 0xFFD00000 | 20 mots | **0x0800** | ordres downlink/uplink page A |
| Write page 1 (MCU→DSP) | 0xFFD00028 | 20 mots | **0x0814** | ordres page B (double-buffer) |
| Read page 0 (DSP→MCU) | 0xFFD00050 | 20 mots | **0x0828** | résultats démod page A |
| Read page 1 (DSP→MCU) | 0xFFD00078 | 20 mots | **0x083C** | résultats démod page B |
| NDB (persistant) | 0xFFD001A8 | 268 mots | **0x08D4** | état persistant + FB/SB + CCCH/traffic |
| PARAM | 0xFFD00862 | 57 mots | 0x0C31 | table paramètres |

---

## 1. Write page (T_DB_MCU_TO_DSP / `db_w`) — ordres ARM→DSP

Offsets `WP_*` en octets depuis la base de page (source `calypso_dsp_internal.h:28-36`, DWARF-validés) ; indices de champ `(n)` depuis `dsp_api.h:32-77`. DSP word = base_page + WP_offset/2.

| DSP word (p0 / p1) | Nom | Rôle | Source |
|--------------------|-----|------|--------|
| 0x0800 / 0x0814 | d_task_d | Downlink task command | dsp_api.h:34 ; internal.h:28 (WP_D_TASK_D=0x00) |
| 0x0801 / 0x0815 | d_burst_d | Downlink burst identifier | dsp_api.h:35 ; internal.h:29 (0x02) |
| 0x0802 / 0x0816 | d_task_u | Uplink task command | dsp_api.h:36 ; internal.h:30 (0x04) |
| 0x0803 / 0x0817 | d_burst_u | Uplink burst identifier | dsp_api.h:37 ; internal.h:31 (0x06) |
| **0x0804 / 0x0818** | **d_task_md** | **Monitoring FB/SB task cmd** (FB=5, SB=6) | dsp_api.h:38 ; internal.h:32 (WP_D_TASK_MD=0x08) ; gate task_md=5/6 `calypso_c54x.c:2307` |
| 0x0807 / 0x081B | d_task_ra | RA (RACH) task command | dsp_api.h:45 ; internal.h:33 (WP_D_TASK_RA=0x0E) |
| 0x0808 / 0x081C | d_fn | FN (rep. period, FN%104) TRAFFIC/TCH | dsp_api.h:46 ; internal.h:34 (WP_D_FN=0x10) |
| 0x080B / 0x081F | d_ctrl_abb | Bitfield reg. analog baseband à envoyer | dsp_api.h:62 (champ 11) |
| 0x080F / 0x0823 | d_afc | Valeur AFC (gated par b_afc) | dsp_api.h:74 (champ 15) |
| **0x0810 / 0x0824** | **d_ctrl_system** | **Control Register RESET/RESUME** ; bit15 = gate go-live | dsp_api.h:75 (champ 16) ; internal.h:35 (WP_D_CTRL_SYSTEM=0x20) ; gate 0xa53c `calypso_arm2dsp.c:83-92` |

Note go-live. Le gate DSP 0xa53c est `BITF *(AR1+0x10),#0x8000` avec AR1=0x0800, soit `BITF data[0x0810]` (d_ctrl_system) bit15. bit15 SET → chemin bootstrap opérationnel ; CLEAR → court-circuit 0xa575. Pont ARM `CALYPSO_ARM2DSP_CTRLSYS` (`calypso_arm2dsp.c:184-192`).

---

## 2. Read page (T_DB_DSP_TO_MCU / `db_r`) — résultats DSP→MCU

Offsets `RP_*` octets (source `calypso_dsp_internal.h:38-43`). Layout confirmé = variante `a_serv_demod` avant `a_pm` (`dsp_api.h:98-100`). DSP word = base_page + RP_offset/2.

| DSP word (p0 / p1) | Nom | Rôle | Source |
|--------------------|-----|------|--------|
| 0x0828 / 0x083C | d_task_d | Downlink task (echo) | internal.h:38 (RP_D_TASK_D=0x00) |
| 0x0829 / 0x083D | d_burst_d | Downlink burst id | internal.h:39 (RP_D_BURST_D=0x02) |
| 0x082C / 0x0840 | d_task_md | Monitoring task (echo) | internal.h:40 (RP_D_TASK_MD=0x08) |
| 0x0830..0x0833 / 0x0844..0x0847 | a_serv_demod[4] | Serv. cell demod {D_TOA,D_PM,D_ANGLE,D_SNR} | dsp_api.h:98 ; internal.h:41 (RP_A_SERV_DEMOD=0x10) |
| 0x0834..0x0836 / 0x0848..0x084A | a_pm[3] | Power measurement results | dsp_api.h:99 ; internal.h:42 (RP_A_PM=0x18) |
| 0x0837..0x083B / 0x084B..0x084F | a_sch[5] | Header + SB info | dsp_api.h:100 ; internal.h:43 (RP_A_SCH=0x1E) |

Sous-indices démod : D_TOA=0, D_PM=1, D_ANGLE=2, D_SNR=3 (`calypso_dsp_internal.h:75-78`).

---

## 3. NDB (T_NDB_MCU_DSP) — état persistant + FB/SB + CCCH/traffic

Base NDB DSP = **0x08D4**. Offsets `NDB_*` octets (source `calypso_dsp_internal.h:44-84`, `calypso_fbsb.h:51-57`). DSP word = 0x08D4 + NDB_offset/2. Ordre de struct = `dsp_api.h:110-360`.

| DSP word | Nom | Rôle | Source |
|----------|-----|------|--------|
| **0x08D4** | **d_dsp_page** | NDB[0] : page courante lue par le firmware (MISC) | dsp_api.h:113 ; internal.h:45 (NDB_D_DSP_PAGE=0x00) ; DSP_ROM_MAP.md ; lu par 0xa51c `10f8 08d4` (dsp.txt:2666) |
| 0x08D5 | d_error_status | DSP status (DSP→MCU) | dsp_api.h:116 ; internal.h:46 (0x02) |
| 0x08D7 | d_tch_mode | TCH mode register | dsp_api.h:120 ; internal.h:83 (NDB_D_TCH_MODE=0x06) |
| **0x08E2** | **d_dsp_state** | État DSP 0 run/1..3 Idle (NDB word 14, actif si W_A_DSP_IDLE3=1) | dsp_api.h:145 ; **voir §CONTRADICTION** |
| 0x08F8 | d_fb_det | FB detection result (1 = FOUND) | dsp_api.h:202 ; fbsb.h:52 / internal.h:48 (NDB_D_FB_DET=0x48) |
| 0x08F9 | d_fb_mode | Mode algo FB | dsp_api.h:203 ; fbsb.h:53 / internal.h:49 (0x4A) |
| 0x08FA..0x08FD | a_sync_demod[4] | FB/SB démod : TOA/PM/ANGLE/SNR | dsp_api.h:204 ; fbsb.h:54-57 / internal.h:50 (NDB_A_SYNC_DEMOD=0x4C) |
| 0x08FE..0x0902 | a_sch26[5] | Header + SB info (SB task) | dsp_api.h:207 ; internal.h:51 (NDB_A_SCH26=0x54) |
| 0x096E.. | a_du_1[22] | UL sub0 traffic (a_du_1, PAS a_du_0) — JALON 3 | dsp_api.h ; internal.h:81 (NDB_A_DU_1=0x134) |
| 0x098A | d_background_enable | Enable tâches background (ARM=0 par design) | dsp_api.h:~289 ; `calypso_arm2dsp.c:27/74` |
| 0x098C | d_background_state | État SM background | dsp_api.h ; `calypso_arm2dsp.c:28/75` ; SM 0xdddb→0xde9c (dsp.txt:3585-3587) |
| 0x09D2..0x09E0 | a_cd[15] | Header + CCCH/SACCH downlink ; **SI3 en a_cd[3]=0x09D5** | dsp_api.h:330 ; internal.h:52 (NDB_A_CD=0x1FC) ; `calypso_dsp_shunt.c:639` |
| 0x09F0.. | a_dd_0[22] | DL traffic FR sub0 ([0]hdr,[2]biterr,[3]data33o) | dsp_api.h ; internal.h:80 (NDB_A_DD_0=0x238) |
| 0x0A06..0x0A14 | a_cu[15] | Header + CCCH/SACCH uplink | dsp_api.h:339 ; `calypso_dsp_shunt.c:173` (BASE_NDB+0x264) |

Sous-champs a_sync_demod (fbsb.h) : TOA 0x08FA, PM 0x08FB, ANGLE 0x08FC, SNR 0x08FD — écrits par le shunt directement en api_ram natif (`calypso_dsp_shunt.c:606-610`).

d_afc / d_afcctladd. `d_afc` vit dans la write page (0x080F, §1). `d_afcctladd` est un champ NDB analog-BB (`dsp_api.h:181`), non offset-isé dans l'émulateur.

---

## 4. Cellules ROM / handshake go-live (data space DSP, hors API RAM)

Adresses vérifiées contre le désassemblage `calypso_dsp.txt` et le code d'émulation. Ce sont des variables internes du firmware DSP (scratch DARAM ~0x3Fxx, tables ~0x43xx, pile ~0x5Axx), **pas** de l'API RAM.

### 4.1 Dispatcher FB-det (les 6 CC) @ 0x873c-0x8753

Balayage `BITF *(flag),#mask` suivi de `f930 <cible>` (CALL conditionnel si TC). Décodé de `calypso_dsp.txt:2188-2190` :

| PC | Instruction | Cible CC | Rôle | Source |
|----|-------------|----------|------|--------|
| ~0x873c | BITF data[0x3fae],#0x0001 | 0x90b0 | copie-routine (RET) | dsp.txt:2188 (`3fae 0001 f930 90b0`) |
| 0x8740 | BITF data[0x3fae],#0x0002 | 0x90b8 | copie-routine (RET) | dsp.txt:2189 |
| 0x8744 | BITF data[0x3fae],#0x0004 | 0x90c8 | copie-routine (RET) | dsp.txt:2189 |
| 0x8748 | BITF data[0x3fae],#0x0008 | 0x90ed | copie-routine (RET) | dsp.txt:2189 |
| 0x874c | BITF data[0x3fad],#0x0001 | 0x914d | copie-routine (RET) | dsp.txt:2190 (`3fad 0001 f930 914d`) |
| **0x8753** | **BITF data[0x3fad],#0x8000** | **0xa0a0** | **SEUL VERROU kernel MAC** (TC=1 ⇒ kernel pris) | dsp.txt:2190 (`3fad 8000 f930 a0a0`) ; probe `calypso_c54x.c:2298-2316` |

Les CC 0x90b0/b8/c8/ed/914d sont des copie-routines qui **reviennent** (RET) : rien ne bloque le sweep. Seul `data[0x3fad] bit15` décide de l'entrée kernel (`calypso_c54x.c:2299-2301`). Un dispatcher amont (0x8720-0x8730) teste `data[0x3faa]` bits (bit0→8900, bit15→8fb8, bit14→8c7e) = étage FBEN.

### 4.2 Corrélateur & kernel MAC

| Adresse | Nom | Rôle | Source |
|---------|-----|------|--------|
| 0x8d00 | handler FB-det / corrélateur | Boucle poll 0x90b0-0x9130 sur les flags 3fae/3fad ; fenêtre corrélateur [0x8d00..0x9000) | `calypso_c54x.c:762` (CORR_PC_LO) ; `calypso_bsp.c:1091-1092` |
| 0xa0a0 | copie-routine kernel (CC gate) | Entrée gatée par 0x8753 ; MAC réel (DADST/DSADT) | dsp.txt:2595 ; `calypso_bsp.c:1093` |
| 0xa076 | kernel MAC | Cœur corrélation (référencé `f273 a076`) | dsp.txt:2590 (`0a050: ... f273 a076`) |
| 0x9a80-0x9ac0 | MAC kernel (DADST/DSADT + CMPS) | Étage corrélation | RAPPORT_GOLIVE_2026-07-25.md:185 |

### 4.3 Flags de handshake FB-det (scratch DARAM ~0x3Fxx)

| Adresse | Bits utiles | Rôle / point de test | Source |
|---------|-------------|----------------------|--------|
| data[0x3faa] | bit2 (0x0004), bit8 (0x0100) | flags FBEN posés @0x886b/0x8885/0x8898 | `calypso_bsp.c:1101/1497` |
| data[0x3fab] | bit8 (0x0100) | cible FBEN, posé @0x888d ; poll 59M reads observés | `calypso_bsp.c:1102/1498/1481` |
| data[0x3fad] | bit0 (→914d), **bit15 (0x8000)** | **MASTER kernel gate** @0x8753/0x8754 | `calypso_bsp.c:1093/1100` ; `calypso_c54x.c:2306` |
| data[0x3fae] | bit0-3 (→90b0/b8/c8/ed) | flags copie-routines @0x90c8/0x90ed/0x9128 | `calypso_bsp.c:1103/1499` |

Note : l'ISR émulée ne posait pas ces bits → le handler bouclait 0x90b0-0x9130 sans fin. Fix `RX-FBFLAGS` pose 0x3faa|=0x104, 0x3fab|=0x100, 0x3fae|=0x100, 0x3fad|=0x8000 (`calypso_bsp.c:1508-1513`, `1100-1103`). Risque : un clearer per-frame (XPC=0 0xace8/0xad04/0xad24 ou overlay XPC=2) peut effacer 0x3fad entre écriture BSP et sweep DSP (`calypso_c54x.c:2302-2307`).

### 4.4 Init & wait-loop go-live (PROM0)

| Adresse | Instruction | Rôle | Source |
|---------|-------------|------|--------|
| 0xa4c7 | (ORM IMR) | Arme IMR (0xa4c0: `76f8 3fde 0001` / `76f8 3f92 0000`) | dsp.txt:2661 ; MEMORY golive-deadlock-5ac8-intm |
| 0xa4ca | tête de boucle | Loop head (spin wait-loop, `f073 a4ca`) | dsp.txt:2662 |
| 0xa4d0 | RSBX INTM | Toggle INTM ("blocs de 10 en 10") | dsp.txt:2662 ; `calypso_arm2dsp.c:64` |
| **0xa4d4** | **BITF data[0x3f70],#0x0002** | **Test de sortie wait-loop** (RET si TC) | dsp.txt:2661 (`f845 a4d4`)+2662 (`60f8 3f70 0002`) ; `calypso_arm2dsp.c:62-63` ; `calypso_c54x.c:2411-2419` |
| 0xa51c | LD data[0x08d4] → ST 0x3fb0 | Lit d_dsp_page (`10f8 08d4`), stocke état page | dsp.txt:2666 ; DSP_ROM_MAP.md |
| 0xa582 | write IMR (depuis 0x435b) | Écrit IMR à partir du shadow 0x435b | dsp.txt:2672-2673 ; MEMORY golive-imr-shadow-435b |

### 4.5 Autres cellules d'état go-live

| Adresse | Nom | Rôle | Source |
|---------|-----|------|--------|
| **data[0x3f70] bit1** | flag sortie wait-loop | Posé par phase-SM 0xde9c ; lu par 0xa4d4 | `calypso_arm2dsp.c:62-64` ; `calypso_c54x.c:2411-2419` ; dsp.txt:3586 |
| data[0x3f6d] | soft-vector go-live | Vecteur logiciel (lu 0xa4d8 `10f8 3f6d`) | dsp.txt:2662 ; `calypso_c54x.c:2400-2405` ; dsp.txt:1833 (`3f6d 711c`) |
| data[0x3fb0] | page state interne | Copie de d_dsp_page (ST par 0xa51c) | DSP_ROM_MAP.md ; dsp.txt:2666 |
| mem[0x5ac8] | seed base de pile | Base pile go-live (`7718 5ac8`) ; `CALYPSO_SEED5AC8_VAL`=0xa4c7 | dsp.txt:1835/2905/2897 ; MEMORY golive-deadlock-5ac8-intm |
| **data[0x435b]** | shadow IMR / mot d'état SM | =0 → 0xa582 écrit IMR=0 (écrase l'arm) = DEADLOCK ; testé bits 0x10/0x40/0x100 @0xa504-0xa518 | dsp.txt:2665-2666 ; `calypso_c54x.c:2630-2637` ; MEMORY golive-imr-shadow-435b |
| data[0x43c0] | slot terminal dispatch | BACC 0xb40f ; réécrit cible corrélateur (était 0xa4c7) | `calypso_bsp.c:1150` |
| data[0x4387] | slot dispatch FB | `f000 4387` (dsp.txt:2905) | RANK3 ; `calypso_bsp.c` |

### 4.6 Buffer I/Q & pointeurs corrélateur (DARAM)

| Adresse | Nom | Rôle | Source |
|---------|-----|------|--------|
| **0x2a00** | buffer I/Q DARAM (BSP DMA) | Cible DMA des samples DL ; consumer DSP PC=0x93a5, AR3 post-inc sur 0x2a00..0x2a13 | `calypso_bsp.c:817` (default `CALYPSO_BSP_DARAM_ADDR=0x2a00`), 807-815 |
| [0x2a00..0x2b27] | plage buffer DMA valide | Le corrélateur doit y pointer (invariant AR5) | `calypso_c54x.c:1647` ; MEMORY correlator-ar5-not-in-buffer-rank3 |
| AR5 | pointeur buffer corrélateur | Doit être dans [0x2a00..0x2b27] ; AR5=0xdb7b hors-buffer = symptôme RANK3 | `calypso_c54x.c:762+` (CORR trace) ; MEMORY correlator-ar5-not-in-buffer-rank3 |
| AR3 | pointeur consumer DMA | Loade à 0x2a00 (début buffer), post-incr | `calypso_c54x.c:232-240` |

---

## 5. CONTRADICTION VÉRIFIÉE : d_dsp_page 0x08D4 vs 0x08E2

Fait établi par comptage de struct + mapping ARM :

- **d_dsp_page réel = DSP word 0x08D4** = NDB[0] (premier champ, `dsp_api.h:113`), confirmé par (a) le mapping ARM 0xFFD001A8→0x08D4, (b) l'instruction firmware `0xa51c: 10f8 08d4` qui lit d_dsp_page (dsp.txt:2666, DSP_ROM_MAP.md), (c) `NDB_D_DSP_PAGE=0x00` dans internal.h:45.
- **0x08E2 = d_dsp_state**, PAS d_dsp_page : c'est le NDB word 14 (`0x08D4 + 14`). Compté depuis `dsp_api.h:113-145` avec W_A_DSP_IDLE3=1 : d_dsp_page(0), d_error_status(1), d_spcx_rif(2), d_tch_mode(3), d_debug1(4), d_dsp_test(5), d_version_number1(6), d_version_number2(7), d_debug_ptr(8), d_debug_bk(9), d_pll_config(10), p_debug_buffer(11), d_debug_buffer_size(12), d_debug_trace_type(13), **d_dsp_state(14)=0x08E2**.

Or **l'émulateur définit `NDB_D_DSP_PAGE = 0x08E2`** (`calypso_fbsb.h:51`) et le shunt miroite d_dsp_page à `api_ram[0x08E2]` (`calypso_dsp_shunt.c:370/434`, `calypso_arm2dsp.c:243`). **C'est un mislabel** : la constante `NDB_D_DSP_PAGE` de l'émulateur pointe en réalité sur d_dsp_state. Le shunt écrit donc la page à l'adresse de d_dsp_state, et le vrai d_dsp_page (0x08D4) lu par 0xa51c reçoit une autre valeur → risque de dérail scheduler (overlay 0x013b via d_dsp_page garbage, cf `calypso_bsp.c:1133`). Concorde avec les notes mémoire `dsp-dpage-offset-bug` et `ndb-cells-098a-background-redherring`.

---

### Sources
- API RAM : `${GSM_ROOT}/osmocom-bb-transceiver/src/target/firmware/include/calypso/dsp_api.h` (struct db_r/db_w/ndb + BASE_API_* :18-23)
- Offsets émulateur : `${QEMU_TREE}/include/hw/arm/calypso/calypso_dsp_internal.h`, `.../hw/arm/calypso/calypso_fbsb.h`, `.../calypso_c54x.h`
- Handshake/wiring : `calypso_bsp.c`, `calypso_c54x.c`, `calypso_arm2dsp.c`, `calypso_dsp_shunt.c`, `calypso_dsp_helper.c`
- ROM disasm : `${GSM_ROOT}/calypso_dsp.txt` (base PROM0=0x7000) ; carte existante `doc/DSP_ROM_MAP.md`

---

> **Voir aussi** : [`DSP_ARM_LINKAGE.md`](DSP_ARM_LINKAGE.md) — correspondance ARM↔DSP↔api_ram et chemins d'écriture du shunt (mode SHUNT_LEGIT). Verrou go-live final = `data[0x3fad] bit15` (0x8000), seul CC qui ouvre l'entrée kernel MAC 0xa0a0 depuis le sweep 0x8753 (§4.1).

---

## Corrélateur FB & kernel MAC — carte DÉFINITIVE (workflows 2026-07-26)

Reverse-engineering du ROM (`calypso_dsp.txt`, image code 2e passe) + traçage run natif.

### Deux corrélateurs distincts
| entrée | rôle | atteint le kernel FB ? |
|---|---|---|
| **0x8d00** | corrélateur SYMBOLE (FCCH/SCH) : BITF 3fab bit8 → réfs 0x0a27/0x3d97, worker 0x8e81 (`7660 db7b` → AR5=0xdb7b immédiat), corréle réfs 0x3d9x | **NON** — jamais 0x2a00 ni 0xa076 |
| **0x94f5 / 0x9500** | corrélateur **ÉNERGIE FB** : `7714 2a00` (AR4=IQ) → `f274 a033` → a040 → `f273 a076` | **OUI** (le vrai) |

### Le vrai kernel 0xa076
- **Unique référence dans tout le ROM** : `f273 a076` @**0xa054** (gate conditionnel, RPTB, dans le corps corrélateur 0xa040-0xa09f ; noyau butterfly/DFT 0xa070-0xa09f, motif `3060 5a85 5f95 8e94 8f93`).
- Setup @**0xa033** : `7719 0020` (BK=0x20 circulaire) · **`7715 2c00` → AR5 = 0x2c00 = forme d'onde de RÉFÉRENCE** · `7714 2c10` AR4 · `7713 2c18` AR3.
- ⚠️ **CORRECTION load-bearing** : sur le vrai chemin **AR5 = 0x2c00 (référence), PAS l'IQ**. Le **buffer IQ 0x2a00 est en AR4/AR1** (posé @0x9500/0x9590/0x95f0/0x9610 : `7714/7711 2a00`). Le critère « AR5=0x2a00 » (ancienne carte) est FAUX et ne sera jamais vrai.

### 0xa0a0 = FAUSSE PISTE (cul-de-sac)
`0xa0a0-0xa0c8` = accumulateur MAC sur page **métriques 0x43xx** (AR3←0x437f) + table coeff 0x0cxx (AR2←0x0ccf), AR5 hérité 0x3fc8, RET (`fc00`) @0xa0c8. **Aucun 0x2a00, ne mène JAMAIS à 0xa076.** Le forcer via `3fad bit15` (CC 0xa0a0 @0x8753) route dans ce MAC-métriques → **ne produit jamais d_fb_det**. `CALYPSO_FORCE_3FAD_KERNEL` = abandonné pour la FB.

### LE VRAI VERROU = le DISPATCH (RANK3), pas un flag
Le handler FB actif (`data[0x43d8]`, CALA @0xb01e) résout vers le corrélateur SYMBOLE / **stub 0xab38** (observé constant), PAS le corrélateur ÉNERGIE. L'énergie n'est appelé que par un orchestrateur (site `f074 9531` : zéro 43e7-43ee, CALA `data[0x4437]`, 43ef=1) que le dispatch **ne sélectionne jamais**. = **RANK3** : le slot 0x4387/0x43c0 tient `0xf074` (base LUT → déraille) au lieu du pointeur handler FB énergie. **Le mur est un gap de WIRING de dispatch, pas un flag DSP interne.** INTM/storm/3fad bit15 = tous des red herrings.

### Fix probe (reachability)
`CALYPSO_FB_ENERGY=1` (c54x.c handler CALA ~L5545) : reroute la CALA @0xb01e → 0x94f5 (entrée énergie) quand `d_task_md(0x058a)==5`. Override entrée : `CALYPSO_FB_CORR_ENTRY=0x9500`. Prérequis : buffer 0x2a00 rempli FN-aligné (`BSP_DIRECT_FEED=1`) + table réf 0x2c00 peuplée (boot-copy `76f8 2c00`). Critères succès : `0xa076` hit + IQ en AR4/AR1=0x2a00 + `d_fb_det(0x08F8)!=0`. C'est une PROBE de reachability, PAS un fix de production — le vrai fix = réparer la cellule dispatch (RANK3), RE multi-étapes.

### Cellules clés (récap)
| addr DSP | rôle |
|---|---|
| 0x2a00 | buffer I/Q RX DARAM (AR4/AR1 sur le vrai chemin) |
| 0x2c00 | forme d'onde de référence FB (AR5 @a033) |
| 0xa076 / 0xa054 | kernel MAC FB / son gate `f273 a076` |
| 0x94f5 / 0x9500 | entrée corrélateur énergie FB |
| 0x8d00 | corrélateur SYMBOLE (fausse cible pour la FB énergie) |
| 0xa0a0 | MAC-métriques (cul-de-sac) |
| 0x43d8 / 0x43c0 / 0x4387 | slots dispatch (RANK3 : tiennent stub/0xf074 au lieu du handler FB) |
| 0x058a | d_task_md (db_w, commande ARM ; 5=FB) |
| 0x3fad bit15 | verrou dispatcher 0x8753→CC 0xa0a0 (mène au cul-de-sac, PAS au kernel) |

Voir aussi : [DSP_ARM_LINKAGE.md](DSP_ARM_LINKAGE.md) · mémoires golive-3fad-bit15-kernel-gate, correlator-ar5-not-in-buffer-rank3.

