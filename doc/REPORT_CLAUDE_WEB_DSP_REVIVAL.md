# DSP Revival — Rapport pour review (2026-06-22)

## Objectif (branche `dsp_revival`)
Réveiller le **vrai DSP TMS320C54x émulé** (`hw/arm/calypso/calypso_c54x.c`) pour qu'il
fasse la démod/décode GSM (FB→SB→BCCH) **à la place du mock gr-gsm** (`calypso_dsp_shunt.c`).
Contraintes utilisateur, strictes :
- **Pas de hacks**, pas de valeurs cannées, pas de bypass.
- **Ne PAS réimplémenter le DSP** : il exécute déjà sa vraie ROM (PROM0-3/DROM/PDROM).
- Juste **câbler la communication comme le silicium**.

## Setup de diagnostic (chemin silicium pur)
Container `osmo-operator-1`, `/opt/GSM/qemu-src`. Lancement :
```
cd /opt/GSM/qemu-src && set -a && . ./calypso.env && \
CALYPSO_CANNED= CALYPSO_MODE=full CALYPSO_DSP_SHUNT=0 CALYPSO_DSP_REG_MODE=c54x \
CALYPSO_TDMA_REALTIME=1 CALYPSO_NO_ATTACH=1 CALYPSO_L2_CLIENT=mobile ./run.sh
```
= vrai c54x (pas de shunt), I/Q réelle via osmo-trx-ipc→BSP→DARAM, TDMA wall-clock.
Firmware L1 = `layer1.highram.elf` (sources `/opt/GSM/osmo_egprs/firmware`).

---

## Architecture de la communication DSP (vérifiée contre dsp_api.h)
```
ARM L1 (prim_fbsb.c, sync.c)
   │  écrit la commande (db_w: d_task_d/d_task_md/d_burst_d) + d_dsp_page
   ▼   API RAM = RAM PLATE partagée mappée 0xFFD00000 (sauf d_dsp_page overlay IO)
API-RAM (dsp_ram[] == dsp->api_ram[], + miroir dsp->data[])   ◄──── DSP word = 0x0800 + off_MMIO/2
   │  le DSP lit la commande EN PLACE (api_ram), pas via DMA
   ▼
c54x exécute sa ROM ─ FB/SB correlator ─► écrit le résultat (NDB/db_r read-page)
   │  a_sync_demod (FB), a_serv_demod + a_sch (SB)        ─ propagé à api_ram (c54x.c:3145)
   ▼
ARM lit le résultat (calypso_dsp_read, lit dsp->data[off/2+0x800])
radio→BSP : samples cs16 → dsp->data[0x2a00] (calypso_bsp.c, daram_addr=0x2a00, len=296)
```

### Offsets MMIO ARM **confirmés** (offsetof sur dsp_api.h, DSP=36/CHIPSET=12/ANLG_FAM=2)
| Champ | off MMIO | DSP word | source firmware |
|---|---|---|---|
| d_fb_det | 0x01F0 | 0x08F8 | prim_fbsb.c:404 `if(!ndb->d_fb_det)` (FOUND = ≠0) |
| a_sync_demod[D_TOA] (FB) | 0x01F4 | 0x08FA | prim_fbsb.c:306 |
| a_sync_demod[D_PM] | 0x01F6 | 0x08FB | prim_fbsb.c:307 |
| a_sync_demod[D_ANGLE] | 0x01F8 | 0x08FC | prim_fbsb.c:308 → AFC |
| a_sync_demod[D_SNR] | 0x01FA | 0x08FD | prim_fbsb.c:309 |
| a_serv_demod[D_TOA] (SB) p0/p1 | 0x60 / 0x88 | 0x0830 / 0x0844 | prim_fbsb.c:148 |
| a_sch[0] CRC (bit B_SCH_CRC=8) p0/p1 | 0x6E / 0x96 | 0x0837 / 0x084B | prim_fbsb.c:181 |
| a_sch[3]/a_sch[4] (mot SB 32b, BSIC) p0/p1 | 0x74,0x76 / 0x9C,0x9E | — | prim_fbsb.c:198, BSIC=(sb>>2)&0x3f |

### Chaîne de validation sync (firmware) — ce qu'il faut pour PASSER
1. **FB** (l1s_fbdet_resp, prim_fbsb.c:399) : `d_fb_det≠0` ; puis FB→SB exige
   `|ANGLE_TO_FREQ(a_sync_demod[D_ANGLE])| < freq_err_thresh` ET `SNR>0`
   (FB0/FB1_SNR_THRESH=0). Donc **ANGLE≈0** et **SNR>0**.
2. **SB** (l1s_sbdet_resp, prim_fbsb.c:171) : `a_sch[0] bit8 (CRC)=0` ET
   `a_serv_demod[D_TOA]≈23` → `toa-=23`≈0 → passe le test « future » (ligne 225 :
   `if(toa > bits_delta) "SB N bits in the future"`). **TOA garbage → rejet.**
3. BSIC = bits 2..7 de `sb=a_sch[3]|a_sch[4]<<16`.
> `0x9041` (run.sh:1645 « vrai FB c54x ») n'est PAS une constante firmware ; seul
> `d_fb_det≠0` compte → `0x6b34` produit par le model passe ce test.

---

## Diagnostic — forks résolus (par sondes read-only)
- **Comm ARM↔API-RAM OK** : `[POST-WATCH]` montre `RAM==DATA` (jamais de MIRROR-MISMATCH).
  L'ARM **poste** `d_task_d` (0x00a2/0xc254…). Pas de bug de miroir.
- **Résultat DSP visible** : `[FBDET-RD]` montre l'ARM lisant `d_fb_det=0x6b34` (FOUND).
- **DMA `0x0586` = écart mort** : 0 lecture (`grep 0x0586` runtime = 0). Le silicium lit
  le descripteur en place à `0x0800`. À RETIRER (`calypso_dsp_done` trx.c:704-738), shunt-safe.
- **#1/#4 (INTM masqué / mauvais vecteur ISR) RÉFUTÉS** : INTM=0 dominant (292:34),
  vecteurs `0xffcc`/`0xffd4` atteints + `RETE` exécuté → l'ISR entre et revient.
- Le DSP **détecte le FB** (`d_fb_det`), mais **le TOA est garbage** (voir ci-dessous).

---

## ⭐ FIX MAJEUR — `binutils-arm-none-eabi` / dérivation `nm` (débloque FB→SB)
**Cause racine de l'instabilité** : `binutils-arm-none-eabi` (donc `arm-none-eabi-nm`)
était **absent**. `run.sh` (~l.1798) utilise `nm` sur l'ELF pour dériver les adresses
des symboles firmware (qui **bougent à chaque rebuild**) et les exporter en env. Sans `nm`,
les adresses restaient aux **défauts hardcodés périmés**.

Le bloc nm ne couvrait QUE le **dsp-shunt** (`l1s`/`last_rach` → `CALYPSO_L1S_FN_ADDR`,
`CALYPSO_LAST_RACH_FN_ADDR`). **Étendu au chemin c54x** : la **fenêtre cpu-idle** du
gouverneur (`calypso_trx.c:1045-1049`, `CALYPSO_IDLE_PC_LO/HI`) avait des défauts ronds
`0x823000/0x826000` au lieu des vrais symboles :
```
l1a_l23_handler   = 0x823f9c   (CALYPSO_IDLE_PC_LO)
l1a_compl_execute = 0x825180   (CALYPSO_IDLE_PC_HI)  ← défaut faux de ~0xe80
```
Fenêtre périmée → **l'ARM se parkait au mauvais PC** → skew ARM/DSP → la transition
FB→SB ne s'exécutait jamais.

**Patch run.sh** (bloc nm étendu, ~l.1807) : nm-dérive aussi `l1a_l23_handler` /
`l1a_compl_execute` → exporte `CALYPSO_IDLE_PC_LO/HI`. Vérifié au runtime :
`[cpu-idle] governor ON window=[0x823f9c,0x825180]`.

**RÉSULTAT** : **FB→SB DÉBLOQUÉ** — le firmware atteint enfin `read_sb_result`
(0 lecture SB avant → 5 après).

---

## État actuel — mur résiduel : TOA garbage
```
SB a_sch[0] CRC   p0=0x0100(err)  p1=0x0000(OK)      ← le SB décode (CRC bon)
SB a_serv_demod[D_TOA] p1 = 0x5088                   ← garbage (devrait ≈ 23)
FB a_sync_demod[D_TOA]    = 0xee0c / 0xc1d0 / 0x0000  ← garbage aussi
FB a_sync_demod[D_ANGLE]  = 0xf484 → 0x0000           ← AFC converge (OK)
FB a_sync_demod[D_SNR]    = 0x0c4a (constant, >0, OK)
```
`a_serv_demod[D_TOA]=0x5088` → `toa-=23` ≈ 20593 → **« SB N bits in the future » → sync rejetée**
(exactement ce que le hack `FORCE_TOA` masque, calypso_trx.c:204-254).

**Le seul champ cassé = le TOA (timing), commun FB et SB.** ANGLE/SNR/CRC/décode = bons.
**Indices** :
- `a_serv_demod[D_TOA]=0x5088` **==** `a_sch[3]=0x5088` (même valeur, mots DSP différents) →
  soit recouvrement de layout dans l'écriture du résultat SB, soit coïncidence.
- Le FB se déclenchait à `fn%51 = 1,21,46` (décalé des positions FCCH 0/10/20/30/40) →
  **offset FN constant** dans la livraison BSP → corrélation sur la mauvaise position → TOA faux.

---

## Changements de code (non commités)
- `hw/arm/calypso/calypso_trx.c` — 3 sondes read-only (fprintf stderr, inconditionnelles) :
  - `[POST-WATCH]` (tick) : `d_task_d`/`d_burst_d` depuis `dsp_ram[]` vs `dsp->data[]`.
  - `[FBDET-RD]` (calypso_dsp_read, off 0x01F0) : valeur lue de `d_fb_det` sur changement.
  - `[FBSBRES-RD]` (calypso_dsp_read) : bloc FB `a_sync_demod` + SB `a_serv_demod[D_TOA]`/`a_sch`.
- `run.sh` (~l.1807) : nm-dérive `CALYPSO_IDLE_PC_LO/HI` (l1a_l23_handler/l1a_compl_execute)
  pour le chemin c54x, en plus du shunt.
- `calypso.env` : `CALYPSO_TDMA_REALTIME=1` (requis pour aligner l'horloge en faketrx ;
  tradeoff drift ARM/DSP en wall).

## Prochaines pistes (TOA garbage)
1. **Cheap** — vérifier si `a_serv_demod[D_TOA] == a_sch[3]` est **systématique** sur
   plusieurs trames SB. Si oui → recouvrement de layout dans l'écriture résultat SB
   (le mot SCH atterrit dans le slot TOA), pas un bug de démod.
2. **Profond** — sonder l'**alignement FN de la fenêtre I/Q en DARAM** : FN du burst
   déposé à `data[0x2a00]` (calypso_bsp.c) vs FN que le corrélateur DSP traite. Un offset
   FN constant → TOA faux pour FB **et** SB.

## Pré-requis env / build
- `binutils-arm-none-eabi` (≥2.38) INSTALLÉ (fournit `arm-none-eabi-nm`). Sans lui,
  les adresses firmware sont périmées → instabilité.
- Rebuild QEMU : `cd /opt/GSM/qemu-src/build && ninja qemu-system-arm`.
