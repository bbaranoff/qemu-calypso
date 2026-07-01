> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : d_fb_det reste 0, DSP déraille, IMR=0x0000 jamais ré-armé, api_write_cb jamais câblé, pas de bus ORCH. Corrections ci-dessous.
>
> En particulier, les conclusions centrales de ce rapport sont **FAUSSES** : le DSP ne détecte PAS le FB (`d_fb_det` reste 0 tout le run — le `0x6b34`/`0x9041` « FOUND » n'apparaît QUE dans un commentaire, jamais produit), le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000) au lieu de tourner le corrélateur, et **rien n'est « débloqué »** : le handshake go-live ARM→DSP ne s'assert jamais (l'ARM n'écrit QUE 0x0000 sur API 0x0314/0x0318), et le callback de notification DSP→ARM `api_write_cb` est **déclaré mais jamais enregistré** (`grep 'api_write_cb *=' = 0 hit` ; invoqué c54x.c:3357-3358 mais jamais assigné → les écritures NDB du DSP sont invisibles à l'ARM). Le « FB→SB DÉBLOQUÉ » et le « SB décode (CRC bon)» ne sont pas reproductibles.

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
   │  a_sync_demod (FB), a_serv_demod + a_sch (SB)        ─ miroir api_ram (c54x.c:3315)
   ▼
ARM lit le résultat (calypso_dsp_read, lit dsp->data[off/2+0x800])
radio→BSP : samples cs16 → dsp->data[0x2a00] (calypso_bsp.c, daram_addr=0x2a00, len=296)
```
> ⚠️ **FAUX (audit 2026-07-01)** : le retour « ARM lit le résultat » n'existe pas dans les faits. Le miroir `data[]`↔`api_ram` est bien câblé (c54x.c:3315, 13173, 13453), mais la notification DSP→ARM `api_write_cb` n'est **jamais enregistrée** (`grep 'api_write_cb *=' = 0` ; déclaré calypso_c54x.h, invoqué c54x.c:3357-3358). Le DSP déraille (POST-BOOTSTUB-RET) avant d'écrire un vrai résultat FB/SB : `d_fb_det` reste 0 tout le run.

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
> `0x9041` (run.sh:1665 « vrai FB c54x ») n'est PAS une constante firmware ; seul
> `d_fb_det≠0` compterait. ~~`0x6b34` produit par le model passe ce test.~~ — **FAUX (audit 2026-07-01)** : ni `0x6b34` ni `0x9041` ne sont produits par le model ; ils n'apparaissent que dans des commentaires (calypso_trx.c:259 ; run.sh:1665). `d_fb_det` reste 0 tout le run — le DSP déraille et ne tourne jamais le corrélateur.

---

## Diagnostic — forks résolus (par sondes read-only)
- **Comm ARM↔API-RAM OK** : `[POST-WATCH]` montre `RAM==DATA` (jamais de MIRROR-MISMATCH).
  L'ARM **poste** `d_task_d` (0x00a2/0xc254…). Pas de bug de miroir.
- ~~**Résultat DSP visible** : `[FBDET-RD]` montre l'ARM lisant `d_fb_det=0x6b34` (FOUND).~~
  — **FAUX (audit 2026-07-01)** : `d_fb_det` reste **0** tout le run ; `0x6b34` n'existe que
  dans un commentaire (calypso_trx.c:259), jamais produit. La notif DSP→ARM (`api_write_cb`)
  n'est jamais câblée (`grep 'api_write_cb *=' = 0`) → écritures NDB du DSP invisibles à l'ARM.
- **DMA `0x0586` = écart mort** : 0 lecture (`grep 0x0586` runtime = 0). Le silicium lit
  le descripteur en place à `0x0800`. À RETIRER (`calypso_dsp_done` trx.c:952 ; la copie
  page→DARAM 0x0586 est en trx.c:965-1000), shunt-safe.
- **#1/#4 (INTM masqué / mauvais vecteur ISR) RÉFUTÉS** : INTM=0 dominant (292:34),
  vecteurs `0xffcc`/`0xffd4` atteints + `RETE` exécuté → l'ISR entre et revient.
  ⚠️ **Nuance (audit 2026-07-01)** : ces sondes ne changent pas la vérité-terrain — le DSP
  déraille (POST-BOOTSTUB-RET, PC=0x0000) et IMR=0x0000 n'est **jamais** ré-armé après le
  clear de boot (@0xb37e). Forcer la vectorisation (`CALYPSO_DSP_FRAME_VEC28`) est un
  **écart mort** : ça atterrit dans l'épilogue d'ISR et redéraille vers le boot-stub.
- ~~Le DSP **détecte le FB** (`d_fb_det`), mais **le TOA est garbage**~~ — **FAUX** : le DSP ne
  détecte PAS le FB (`d_fb_det`=0). Le vrai verrou est le handshake go-live ARM→DSP qui ne
  s'assert jamais (l'ARM n'écrit que 0x0000 sur API 0x0314/0x0318 ; `api_write_cb` jamais
  enregistré). Le « TOA garbage » ci-dessous décrit des lectures forcées/instrumentées, pas
  un vrai résultat de corrélation.

---

## ⭐ ~~FIX MAJEUR~~ — `binutils-arm-none-eabi` / dérivation `nm` (~~débloque FB→SB~~)
> ⚠️ **FAUX (audit 2026-07-01)** : ce « fix » ne débloque rien. `d_fb_det` reste 0, le DSP
> déraille, IMR=0x0000 jamais ré-armé, la transition FB→SB ne s'exécute jamais. La cause
> réelle n'est pas la fenêtre cpu-idle mais le handshake go-live ARM→DSP absent
> (`api_write_cb` jamais câblé). Le contenu ci-dessous est conservé comme historique.
**Cause racine de l'instabilité** : `binutils-arm-none-eabi` (donc `arm-none-eabi-nm`)
était **absent**. `run.sh` (~l.1798) utilise `nm` sur l'ELF pour dériver les adresses
des symboles firmware (qui **bougent à chaque rebuild**) et les exporter en env. Sans `nm`,
les adresses restaient aux **défauts hardcodés périmés**.

Le bloc nm ne couvrait QUE le **dsp-shunt** (`l1s`/`last_rach` → `CALYPSO_L1S_FN_ADDR`,
`CALYPSO_LAST_RACH_FN_ADDR`). **Étendu au chemin c54x** : la **fenêtre cpu-idle** du
gouverneur (`calypso_cpu_idle_park()`, calypso_trx.c:1290-1320 ; lecture env
`CALYPSO_IDLE_PC_LO/HI` @1296-1297, défauts @1299-1300) avait des défauts ronds
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

~~**RÉSULTAT** : **FB→SB DÉBLOQUÉ** — le firmware atteint enfin `read_sb_result`
(0 lecture SB avant → 5 après).~~ — **FAUX (audit 2026-07-01)** : rien n'est débloqué.
`d_fb_det` reste 0, le DSP déraille, IMR=0x0000 jamais ré-armé, le corrélateur ne tourne
jamais. Le handshake go-live ARM→DSP ne s'assert pas (l'ARM n'écrit que 0x0000 sur
API 0x0314/0x0318 ; HS-ARM-GATE calypso_trx.c:557-565 est une **sonde read-only** :
« no DSP/ARM state is poked here »).

---

## État actuel — mur résiduel : TOA garbage
> ⚠️ **FAUX (audit 2026-07-01)** : le SB ne décode pas et le CRC « bon » n'est pas
> reproductible — le DSP déraille avant tout décode SB. Les valeurs ci-dessous proviennent
> de sondes/lectures forcées, pas d'un vrai résultat de corrélateur.
```
SB a_sch[0] CRC   p0=0x0100(err)  p1=0x0000(OK)      ← le SB décode (CRC bon)
SB a_serv_demod[D_TOA] p1 = 0x5088                   ← garbage (devrait ≈ 23)
FB a_sync_demod[D_TOA]    = 0xee0c / 0xc1d0 / 0x0000  ← garbage aussi
FB a_sync_demod[D_ANGLE]  = 0xf484 → 0x0000           ← AFC converge (OK)
FB a_sync_demod[D_SNR]    = 0x0c4a (constant, >0, OK)
```
`a_serv_demod[D_TOA]=0x5088` → `toa-=23` ≈ 20593 → **« SB N bits in the future » → sync rejetée**
(exactement ce que le hack `FORCE_TOA` masque, calypso_trx.c:320-351 : getenv
`CALYPSO_FORCE_TOA` @337, bloc d'application @342-351).

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
