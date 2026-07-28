# Liaison DSP↔ARM — Correspondance d'adresses (mode SHUNT_LEGIT)

> Date : 2026-07-26
>
> Ce document décrit la correspondance entre les trois vues d'une même API RAM (ARM physique, DSP data word, index `api_ram`), et quels arrays C le firmware ARM et le shunt lisent/écrivent effectivement. Tout est vérifié contre le code (`calypso_trx.c`, `calypso_dsp_shunt.c`, `calypso_dsp_helper.c`, `calypso_bsp.c`, `calypso_c54x.h`, `calypso_dsp_internal.h`, `calypso_fbsb.h`).

**Clarification architecturale centrale vérifiée** : `calypso_trx.c:1922` lie `c54x_set_api_ram(s->dsp, s->dsp_ram)`, donc **`api_ram` est un alias de `dsp_ram`** (l'array MMIO 0-based), qui est une allocation *distincte* de `s->dsp->data[]` (la mémoire data 64K-mots du cœur DSP). Le chemin de lecture ARM (`trx.c:230`) lit `data[]` ; le chemin d'écriture ARM miroite vers les **deux** : `dsp_ram[offset/2]` (`:510`) et `data[offset/2+0x800]` (`:546`).

## 1. Les trois vues d'une même API RAM

L'API RAM est une boîte aux lettres partagée entre l'ARM (ARM7TDMI, firmware osmocom-bb) et le DSP (TMS320C54x). Le code manipule **trois** vues de cette mémoire :

| Vue | Base | Unité | Array C / accès | Source |
|---|---|---|---|---|
| **ARM physique** | `0xFFD00000` | octet | MMIO `calypso_dsp_read/write`, `offset = addr − 0xFFD00000` | `calypso_trx.c` |
| **DSP data word** | `0x0800` (`C54X_API_BASE`) | mot 16 bits | `s->dsp->data[]` (mémoire propre du cœur DSP ; source de vérité lue par le firmware ARM) | `calypso_c54x.h:21` |
| **api_ram index** | `0` (0-based) | mot 16 bits | `s->dsp->api_ram[]` **= alias de** `s->dsp_ram[]` | `calypso_c54x.h`, `trx.c:1922` |

`C54X_API_BASE = 0x0800`, `C54X_API_SIZE = 0x2000` (8 K mots), `C54X_DATA_SIZE = 0x10000` (64 K mots) — `calypso_c54x.h:21-22, :196`.

### Règle de conversion (les deux sens)

Soit `off = ARM_addr − 0xFFD00000` (offset ARM en octets) :

```
DSP data word  = off/2 + 0x0800
api_ram index  = off/2                    (= DSP word − 0x0800)
ARM off        = (DSP word − 0x0800) * 2  = api_ram_idx * 2
ARM addr       = 0xFFD00000 + off
```

Écriture 16 bits little-endian (`cpu_to_le16`).

## 2. Le point CENTRAL : quel array le firmware lit-il ?

**Le firmware ARM lit `s->dsp->data[offset/2 + 0x0800]`, PAS `s->dsp_ram[]` ni directement `api_ram[]`.**

Chemin de **lecture** ARM (`calypso_trx.c:230`) :
```c
uint16_t *src = &s->dsp->data[offset/2 + 0x0800];   // source de vérité
val = (size==2) ? src[0] : ...;
// fallback :236 → &s->dsp_ram[offset/2]  UNIQUEMENT si s->dsp non alloué (pre-realize)
```

Chemin d'**écriture** ARM `calypso_dsp_write` — miroir vers les DEUX arrays :
```c
:510  s->dsp_ram[offset/2]          = value;              // array MMIO 0-based (= api_ram)
:546  s->dsp->data[offset/2+0x0800] = (uint16_t)value;    // array cœur DSP
```

Conséquences pour le shunt :
- Pour qu'une injection soit **vue par le firmware**, il faut écrire dans `data[]` (l'array lu), OU intercepter la lecture MMIO.
- `shunt_write_w` / `shunt_read_w` passent par `dma_memory_write/read` (`calypso_dsp_helper.c:31-41`) → écrivent dans `dsp_ram[]` (donc `api_ram[]`), **pas** directement dans `data[]`, sauf via le miroir du write-path. Le shunt « camp » écrit donc `a_cd` **directement dans `data[]`** (`d = c54x->data`) et intercepte les reads FB/SB côté MMIO.
- `shunt_read_page_w` lit correctement la source de vérité : `dsp->data[((arm_addr − 0xFFD00000) >> 1) + 0x0800]` (`calypso_dsp_shunt.c:377`).

## 3. Bases de pages (`calypso_dsp_internal.h:21-25`)

| Zone | Rôle | ARM addr | off | DSP word | api_ram idx |
|---|---|---|---|---|---|
| **W_PAGE_0** | MCU→DSP, 20 mots | `0xFFD00000` | 0x000 | 0x0800 | 0x000 |
| **W_PAGE_1** | MCU→DSP page 1 | `0xFFD00028` | 0x028 | 0x0814 | 0x014 |
| **R_PAGE_0** | DSP→MCU, 20 mots (`BASE_API_R_PAGE_0`) | `0xFFD00050` | 0x050 | 0x0828 | 0x028 |
| **R_PAGE_1** | DSP→MCU page 1 | `0xFFD00078` | 0x078 | 0x083C | 0x03C |
| **NDB** | persistant, 268 mots (`BASE_API_NDB`) | `0xFFD001A8` | 0x1A8 | 0x08D4 | 0x0D4 |

Offsets de champ (octets) — write page (`WP_*`), read page (`RP_*`), NDB (`NDB_*`) :
`WP_D_TASK_D=0x00, WP_D_BURST_D=0x02, WP_D_TASK_U=0x04, WP_D_TASK_MD=0x08, WP_D_TASK_RA=0x0E, WP_D_FN=0x10` ; `RP_D_TASK_D=0x00, RP_D_BURST_D=0x02` ; `NDB_D_FB_DET=0x48, NDB_A_SYNC_DEMOD=0x4C, NDB_A_CD=0x1FC`. Sous-champs demod : `D_TOA=0, D_PM=1, D_ANGLE=2, D_SNR=3` (`calypso_dsp_internal.h:28-51, :90-93`).

## 4. Résultat FB natif (NDB) — écriture `api_ram` + read-intercept

`calypso_fbsb.h:52-57` définit les DSP words natifs. Le bloc FB natif est posé dans `api_ram` aux offsets exacts (`calypso_dsp_shunt.c:606-610`, `ar = c54x->api_ram`) :

| Cellule | DSP word | ARM addr | api_ram idx | NDB off | Écrit par |
|---|---|---|---|---|---|
| **d_fb_det** (FOUND=1) | `0x08F8` | `0xFFD001F0` | `0x0F8` | +0x48 | `shunt.c:606` `ar[0x08F8−C54X_API_BASE]` |
| a_sync_demod TOA | `0x08FA` | `0xFFD001F4` | `0x0FA` | +0x4C(+D_TOA) | `shunt.c:607` |
| a_sync_demod PM | `0x08FB` | `0xFFD001F6` | `0x0FB` | +0x4C+2 | `shunt.c:608` |
| a_sync_demod ANGLE | `0x08FC` | `0xFFD001F8` | `0x0FC` | +0x4C+4 | `shunt.c:609` |
| a_sync_demod SNR | `0x08FD` | `0xFFD001FA` | `0x0FD` | +0x4C+6 | `shunt.c:610` |

> Note d'implémentation : le correctif « format natif » (mémoire `native-apiram-fb-result-format`) a remplacé un `shunt_write_w(BASE_API_NDB+NDB_D_FB_DET)` qui visait la mauvaise cellule (`api_ram[0x550]`) par l'écriture directe aux offsets natifs `api_ram[0xF8..0xFD]`.

**Read-intercepts MMIO** (`calypso_dsp_shunt_real_fb_read`, `calypso_dsp_shunt.c:1314-1335`) — court-circuitent la lecture ARM et livrent la dernière détection gr-gsm réelle (`g_shunt.rx_*`), immunisant contre l'ordonnancement intra-trame :

| ARM off | Cellule | Retourne |
|---|---|---|
| `0x01F0` | d_fb_det | `g_shunt.rx_fb_det ? 1 : 0` |
| `0x01F4` | a_sync TOA | `g_shunt.rx_toa` |
| `0x01F6` | a_sync PM | `g_shunt.last_pm` |
| `0x01F8` | a_sync ANGLE (AFC) | `(uint16_t)g_shunt.rx_afc` |
| `0x01FA` | a_sync SNR | `g_shunt.rx_snr` |
| `0x0060`/`0x0088` | SB `a_serv_demod[D_TOA]` P0/P1 | `g_shunt.rx_toa` (si `sb_valid`) |

L'intercept prend priorité sur `data[]` (flag `real_fb_hit`, `trx.c:262-265`) → l'ARM voit le FB depuis `g_shunt.rx_*`, pas depuis `api_ram`/`data`.

## 5. a_cd (résultat CCCH/BCCH → SI) — écriture directe `data[]`

`NDB_A_CD = 0x1FC` → ARM off `0x1A8+0x1FC = 0x3A4` → DSP word `0x9D2`. Le shunt écrit **directement dans `data[]`** (`d = c54x->data`, `calypso_dsp_shunt.c:671-676`) parce que le firmware lit `data[]` via `trx.c:230` :

| a_cd[] | DSP word | ARM addr | api_ram idx | Rôle | Écrit par |
|---|---|---|---|---|---|
| a_cd[0] (FIRE/CRC pass) | `0x09D2` | `0xFFD003A4` | `0x1D2` | statut CRC | `shunt.c:671` `d[0x9D2]=0` |
| a_cd[1] | `0x09D3` | `0xFFD003A6` | `0x1D3` | statut | `shunt.c:672` |
| a_cd[2] (num_biterr) | `0x09D4` | `0xFFD003A8` | `0x1D4` | biterr=0 | `shunt.c:673` |
| **a_cd[3] = SI3** début | `0x09D5` | `0xFFD003AA` | `0x1D5` | L2 23 o (SI/BCCH) | `shunt.c:674-676` `d[0x9D5+i/2]` |
| … a_cd[14] | `0x09E0` | `0xFFD003C0` | `0x1E0` | fin payload | idem |

Packing : `d[0x9D5+i/2] = si[i] | (si[i+1]<<8)` pour i=0..22. Le firmware fait `dsp_memcpy_from_api(&ndb->a_cd[3], 23)` → `a_cd[3]` = premier octet L2.

## 6. a_serv_demod & a_pm des read-pages (NB/SB)

Le NB (`nb_resp`) lit 4 mots `a_serv_demod` par burst pour l'AFC + rx_level. Read page 0 base ARM `0x50`, mot 8 → ARM off `0x60`, DSP word `0x830` ; page 1 → DSP word `0x844` (`calypso_dsp_shunt.c:710-711`, écriture directe `data[]`) :

| Cellule | DSP word | ARM addr | api_ram idx | Page | Écrit par |
|---|---|---|---|---|---|
| a_serv_demod TOA P0 | `0x0830` | `0xFFD00060` | `0x030` | RP0+0x10 | `shunt.c:710` `d[0x830]` |
| a_serv_demod PM P0 | `0x0831` | `0xFFD00062` | `0x031` | RP0+0x12 | `shunt.c:710` |
| a_serv_demod ANGLE P0 | `0x0832` | `0xFFD00064` | `0x032` | RP0+0x14 | `shunt.c:710` |
| a_serv_demod SNR P0 | `0x0833` | `0xFFD00066` | `0x033` | RP0+0x16 | `shunt.c:710` |
| a_serv_demod[4] P1 | `0x0844..0x0847` | `0xFFD00088..8E` | `0x044..0x047` | RP1+0x10.. | `shunt.c:711` `d[0x844..847]` |

**a_pm (RANK5 rxlev natif)** — posé dans `api_ram` (`ar`) à chaque tick, valeur calibrée trf6151 (`shunt_pm_decan_apm`), aux offsets read-page `woff 0x30..0x32` (P0) / `0x44..0x46` (P1) (`calypso_dsp_shunt.c:626-627`) :

| Cellule | api_ram idx | DSP word | ARM addr | Écrit par |
|---|---|---|---|---|
| a_pm P0 | `0x30,0x31,0x32` | `0x830..0x832` | `0xFFD00060..64` | `shunt.c:626` `ar[0x30]=ar[0x31]=ar[0x32]=apm` |
| a_pm P1 | `0x44,0x45,0x46` | `0x844..0x846` | `0xFFD00088..8C` | `shunt.c:627` `ar[0x44..0x46]=apm` |

> `a_pm=(rf_cible+71+gain)*64`, suit `REG_RX` via TSP dev 1 (modèle `calypso_trf6151.c`, mémoire `trf6151-gain-model-rxlev`).

## 7. d_task_d / d_burst_d (dispatch NB) — overrides read-intercept

Sous `SHUNT_LEGIT` + `si_valid`, le read MMIO des read-pages est réécrit (`calypso_trx.c:270-305`) pour débloquer le chemin `nb_resp` vers `a_cd` :

| Cellule | ARM off | DSP word | api_ram idx | Override |
|---|---|---|---|---|
| d_task_d P0 / P1 | `0x0050` / `0x0078` | `0x0828` / `0x083C` | `0x028` / `0x03C` | `0 → 24` (ALLC_DSP_TASK) évite `EMPTY` (`trx.c:270-278`) |
| d_burst_d P0 / P1 | `0x0052` / `0x007A` | `0x0829` / `0x083D` | `0x029` / `0x03D` | `(s_burst_cur+3)&3` → match burst 3 (lit a_cd) (`trx.c:283-305`) |

## 8. d_task_md (mode de tâche, WRITE pages)

L'ARM commande le mode DSP (`FB=5, SB=6, PM=1, ALLC=24`) en écrivant `WP_D_TASK_MD=0x08` de la write-page :

| Cellule | ARM addr | off | DSP word | api_ram idx |
|---|---|---|---|---|
| d_task_md W_PAGE_0 | `0xFFD00008` | 0x008 | **`0x0804`** | 0x004 |
| d_task_md W_PAGE_1 | `0xFFD00030` | 0x030 | **`0x0818`** | 0x018 |

Latché côté shunt via `shunt_read_w(wp + WP_D_TASK_MD)` (`calypso_dsp_helper.c:150`).

## 9. d_rach (RACH montante ARM→DSP)

Le firmware écrit la RACH (req-ref, `(ra<<8)|(bsic<<2)`) dans `d_rach`. Offset épinglé `D_RACH_DEFAULT_OFFSET = 0x023A` (`calypso_bsp.c:1677`), c.-à-d. **word 0x23A depuis la base API** :

| Convention | Valeur |
|---|---|
| api_ram idx (word from API base) | `0x023A` |
| ARM byte | `0x0474` (= 0x23A × 2) |
| DSP data word | `0x0A3A` (= 0x23A + 0x0800) |
| ARM addr | `0xFFD00474` |

Confirmé `calypso_bsp.c:1670` : « API byte 0x0474 (= DSP word 0x0A3A = word 0x23A from API base) ». Capté par `calypso_dsp_shunt_record_rach()` (`calypso_trx.c:421`).

## 10. a_cu (SDCCH/SACCH UL — L2 montante)

Le firmware poste la L2 montante (SABM/SACCH/I-frames) dans `a_cu`, base `BASE_API_NDB + 0x264` (`calypso_dsp_helper.c:173`, lu par `shunt_read_w(wbase+i)`) :

| Cellule | ARM addr | off | DSP word | api_ram idx |
|---|---|---|---|---|
| a_cu[0] (header L1) | `0xFFD0040C` | 0x40C | `0x0A06` | 0x206 |
| a_cu[3] (début L2, +6 o) | `0xFFD00412` | 0x412 | `0x0A09` | 0x209 |

`wbase = BASE_API_NDB + 0x264u + acu_ofs` ; la L2 (23 o) commence après le header a_cu[0..2].

---

### Récap des règles vérifiées

- `DSP word = off/2 + 0x0800` ; `api_ram idx = off/2` ; `ARM off = api_ram_idx × 2`.
- Firmware lit **`data[]`** (`trx.c:230`) ; write ARM mirroir vers `dsp_ram[]`+`data[]` (`trx.c:510,:546`) ; `api_ram` = alias de `dsp_ram` (`trx.c:1922`).
- Le shunt écrit `a_cd`/`a_serv_demod` **directement dans `data[]`** (vus par l'ARM), pose le FB/`a_pm` natifs dans `api_ram`, et livre FB/SB via **read-intercepts** MMIO (`g_shunt.rx_*`).

Sources vérifiées : `calypso_c54x.h:21-22,196`, `calypso_dsp_internal.h:21-93`, `calypso_fbsb.h:52-67`, `calypso_trx.c:230,236,270-311,510,546,1922`, `calypso_dsp_shunt.c:377,606-627,671-711,1314-1335`, `calypso_dsp_helper.c:31-41,150,173`, `calypso_bsp.c:1670,1677`.

---

> **Voir aussi** : [`DSP_ADDRESS_MAP.md`](DSP_ADDRESS_MAP.md) — carte complète des adresses DSP (API RAM + cellules ROM/handshake go-live). Le verrou go-live final = `data[0x3fad] bit15` (0x8000), seul CC du sweep 0x8753 qui ouvre l'entrée kernel MAC 0xa0a0 ; hors API RAM, c'est une cellule scratch DARAM posée par le fix `RX-FBFLAGS` (`calypso_bsp.c:1100-1103`).

---

## Task-post ARM->DSP + dispatch (run natif 2026-07-26)

L'ARM commande une tâche (`calypso/dsp.c:480`, `prim_fbsb.c:381`) :
```c
dsp_api.db_w->d_task_md = FB_DSP_TASK;   // 5 -> data[0x058a] (db_w) ; aussi read-pages data[0x0804]/[0x0818]
dsp_api.ndb->d_fb_mode  = fb_mode;
dsp_end_scenario():  dsp_api.ndb->d_dsp_page = B_GSM_TASK(0x0002) | w_page ;  w_page ^= 1;   // data[0x08e2]
```
| ARM (dsp_api) | DSP word | note |
|---|---|---|
| db_w->d_task_md | data[0x058a] (write) ; read-pages 0x0804/0x0818 | 5=FB 6=SB ; 0 en idle |
| ndb->d_dsp_page | data[0x08e2] | B_GSM_TASK\|w_page ; fige à 2 en natif (w_page pas flippé) |

### Le mur RANK3 (côté dispatch)
Le DSP lit le pointeur de handler dans les **slots de dispatch** `data[0x43c0]` (terminal BACC 0xb40f), `data[0x4387]` (idle/CALA 0xb01e), `data[0x43d8]` (reseed). En natif ils résolvent vers le **stub 0xab38** (ou `0xf074`=base LUT → déraille) au lieu du **pointeur handler FB énergie** (0x94f5). C'est pourquoi le corrélateur énergie (→0xa076) n'est jamais exécuté. Le fix natif = l'ARM (ou la LUT 0x8341) doit semer la bonne valeur-pointeur dans ces slots — RE multi-étapes. Probe intermédiaire : `CALYPSO_FB_ENERGY=1` reroute la CALA @0xb01e → 0x94f5 (cf DSP_ADDRESS_MAP).

Rappel : l'ARM lit `s->dsp->data[off/2 + 0x0800]` (calypso_trx.c), PAS `s->api_ram[]`.

Voir aussi : [DSP_ADDRESS_MAP.md](DSP_ADDRESS_MAP.md).


---

## `d_error_status` — mecanique complete (mesuree 2026-07-28)

### Adressage

| Quoi | Valeur | Comment c'est etabli |
|---|---|---|
| Base NDB | `BASE_API_NDB = 0xFFD001A8` | `osmocom-bb/.../include/calypso/dsp_api.h:18` |
| Type des champs | `typedef unsigned short API;` (**16 bits**) | `include/calypso/l1_environment.h:3` |
| Variante de structure active | `#if (DSP == 34)||(35)||(36)` — **`#define DSP 36`** | `l1_environment.h:9` ; 4 variantes existent dans le header, les champs different |
| `d_dsp_page` | NDB+0 -> ARM `0xFFD001A8` -> **mot DSP `0x08D4`** | struct : 1er champ |
| **`d_error_status`** | NDB+1 mot -> ARM **`0xFFD001AA`** -> **mot DSP `0x08D5`** | struct : suit immediatement `d_dsp_page` |

⚠️ `calypso_fbsb.h:51` definit `NDB_D_DSP_PAGE = 0x08E2`. **Ce n'est pas la base NDB** (qui
est le mot `0x08D4`). Ne pas s'en servir pour deriver les offsets des champs voisins.

### Ce que fait le firmware DSP

Le DSP ne *calcule* pas un code d'erreur : il **recopie son mot d'etat interne**, masque
sur 12 bits, dans la cellule que l'ARM lit. Desassemblage (`CALYPSO_TRACEFROM=0xb0f6`) :

```
0xb106:  10f8 3f92     LD   *(0x3f92), A     ; mot d'etat interne DSP
0xb108:  f030 0fff     AND  #0x0fff, A       ; masque 12 bits
0xb10a:  80f8 08d5     STL  A, *(0x08d5)     ; -> d_error_status
0xb10c:  fc00          RET
```

> **`d_error_status == data[0x3f92] & 0x0FFF`**

### Ce que fait le firmware ARM

`osmocom-bb/.../layer1/sync.c:249-252` — a chaque trame :

```c
if (dsp_api.ndb->d_error_status) {
    printf("DSP Error Status: %u
", dsp_api.ndb->d_error_status);
    dsp_api.ndb->d_error_status = 0;      /* l'ARM EFFACE apres lecture */
}
```

L'ARM efface donc la cellule a chaque tour ; si le message revient, c'est que le DSP la
**repose** — la condition est persistante, pas un evenement unique. Le firmware ne fait
qu'imprimer : **il ne change pas de comportement**. Un `DSP Error Status` n'est donc pas
en soi un blocage, c'est un temoin.

### Signification des bits (`enum dsp_error`, `dsp_api.h:1541`)

| Bit | Valeur | Nom |
|---|---|---|
| 0 | `0x0001` | `DSP_ERR_RHEA` |
| 2 | `0x0004` | `DSP_ERR_IQ_SAMPLES` |
| 3 | `0x0008` | `DSP_ERR_DMA_PROG` |
| 4 | `0x0010` | `DSP_ERR_DMA_TASK` |
| **5** | **`0x0020`** | **`DSP_ERR_DMA_PEND`** |
| 7 | `0x0080` | `DSP_ERR_VM` |
| 8 | `0x0100` | `DSP_ERR_DMA_UL_TASK` |
| 9 | `0x0200` | `DSP_ERR_DMA_UL_PROG` |
| 10 | `0x0400` | `DSP_ERR_DMA_UL_PEND` |
| 11 | `0x0800` | `DSP_ERR_STACK_OV` |

### Observations mesurees (2026-07-28)

- **`0x0800` STACK_OV** : apparaissait en boucle tant que le DSP tournait a la cadence du
  routeur d'assist. **Eteint** par le split `active()`/`substitutes()` (le DSP tourne a la
  cadence trame). Reapparu temporairement quand l'excursion `SHUNT_DSP_FB` ecrasait la
  pile du DSP -> corrige par une **pile dediee** (`CALYPSO_SHUNT_DSP_FB_SP`).
- **`0x0020` DMA_PEND** : `data[0x3f92] = 0x0020` (670 relevés) apparait vers `fn~1300`
  (+6 s). **Pose par le firmware DSP, pas par nous** : notre code n'ecrit QUE le bit 11
  (`data[0x3f92] |= 0x0800` — `calypso_bsp.c:1378`, `calypso_c54x.c:13985`,
  `calypso_trx.c:990`), et ce bit n'apparait meme pas dans les valeurs relevees.

### Piege de mesure (paye 4 fois cette nuit)

L'adresse `0x08D5` etait correcte des la premiere deduction ; ce sont les **fenetres de
sonde** qui ont fait echouer trois tentatives :

1. sonde sur `0x08E3` (mauvais offset derive de `NDB_D_DSP_PAGE`) -> muette ;
2. sonde sur `0x08D4..0x08E4`, plafond 40 -> **consomme par les ecritures de ZERO du boot** ;
3. idem, filtre sur non-nul -> **consomme par `data[0x08dc]`** (autre cellule, ecrite en
   boucle a `PC=0xb530`) ;
4. sonde sur **`0x08D5` seul, non-nul seulement** -> resultat immediat.

**Regle** : une sonde se concoit par sa **condition de declenchement**, pas par son
adresse. Et « pas de log » n'est jamais « pas d'evenement » tant qu'on n'a pas verifie que
la sonde est vivante et que sa fenetre couvre l'instant vise.

### Reproduire

```bash
# qui pose la valeur (cote DSP)
CALYPSO_ERRWATCH=1 ./start-clean.sh          # ecritures non nulles de data[0x08D5]
# ce que l'ARM lit (les deux vues du miroir api_ram)
CALYPSO_ERRREAD=1 ./start-clean.sh           # compare dsp_ram[] et dsp->data[]
# recherche par la VALEUR plutot que par l'adresse
CALYPSO_FIND32=1 [CALYPSO_FIND32_VAL=0x20]   # quel offset ARM porte cette valeur
# le code qui recopie
CALYPSO_TRACEFROM=0xb0f6 CALYPSO_TRACEFROM_N=32
```

### Question ouverte

Quel PC pose le **bit 5 de `data[0x3f92]`** ? Meme sonde, transposee d'une adresse :
watchpoint sur les ecritures de `0x3f92` filtrees sur `val & 0x20`, avec le PC.
⚠️ Priorite a evaluer : `d_error_status` est un **temoin**, pas un blocage — le firmware
ARM l'imprime et l'efface sans changer de comportement. L'indicateur qui compte reste
`d_fb_det`.
