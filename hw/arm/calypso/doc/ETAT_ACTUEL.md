# ETAT ACTUEL — QEMU-Calypso (source de verite unique)

> Document de reference courant. Ancre sur la VERITE TERRAIN VERIFIEE 2026-07-27.
> **Regle de resolution de conflit :** la verite terrain 2026-07-27 (matrice §1) prime sur
> tout fait extrait d'un autre doc. Le STATUT DEPEND DU MODE : ne jamais citer un statut
> sans le mode. Tous les autres documents sont dans `doc/archive/` (reference durable
> uniquement, cf. §7).

---

## 1. Statut par mode (verite terrain 2026-07-27)

> **Il n'y a PAS de statut absolu — le statut depend du MODE de run.** L'ancienne
> "contradiction" (« LU ACCEPT recu » vs « RACH UL bloque pour boucler la LU ») etait
> DEUX MODES CONFONDUS : la LU MARCHE en shunt, elle est bloquee en natif. La resolution
> n'est pas de choisir, c'est de presenter le statut PAR MODE.

### Les modes (socle `calypso.env` + un `calypso_X.env`)

- **`SHUNT_LEGIT=1`** — base. FBSB host-side (`real_fb` + gr-gsm -> API RAM), cannes de
  stabilisation, DSP c54x OFF. **MODE FIABLE.**
- **`SHUNT_NO_LEGIT=1`** — injections explicites, le DSP shunte.
- **`SHUNT_LEGIT=DSP,NO_CANNED`** (value-list) — DSP c54x tourne en // et sans cannes.
  Plus proche du reel, plus flaky.
- **`CALYPSO_NATIVE=1`** (NATIF) — DSP FB-STREAM -> `data[0x9213]`/`data[0x9215]`.
- **`CALYPSO_NATIVE_HELPED=1`** (NATIF AIDE) — DSP + `feed_iq` DARAM `0x9210`.

### Matrice statut x mode

| Feature | SHUNT_LEGIT=1 | SHUNT_NO_LEGIT | DSP,NO_CANNED | NATIVE / NATIVE_HELPED |
|---|---|---|---|---|
| FB/SB sync | DONE (host gr-gsm) | DONE | DONE | WIP : correlateur DSP TOURNE (atteint `0x8d00`, DETECTOR-RUN @`0x9ac0`, `d_fb_mode=1`) mais `d_fb_det[0x08f8]=0` -> PAS de detection |
| rxlev serving | DONE | DONE | DONE | DONE : -47 dBm reel (modele trf6151/DECAN) |
| Camp (C3) | DONE | DONE | DONE | TODO : « No sysinfo » (FB pas detecte) |
| LU + registration | DONE (LU ACCEPT + TMSI + On Network) | DONE | DONE | TODO (pas de camp -> pas de LU) |
| SMS MO | DONE | DONE | WIP flaky (anti-stall ajoute) | TODO |
| SMS MT | DONE | DONE | WIP | TODO |
| Ctrl-C mobile recover | DONE | DONE | DONE | TODO |
| Voix TCH/F | WIP : ASSIGNMENT COMMAND atteint -> ASSIGNMENT FAILURE (le shunt ne presente pas le TCH DL) ; call `fake_trx` = ACTIVE + audio => reseau OK | WIP | WIP | TODO |

### Chaine operationnelle (la SEULE voie FBSB qui campe — modes shunt)

```
twl3025/DECAN (AFC/PM/rxlev, gain trf6151)
gr-gsm (decode SB/SI sur I/Q continu 4 SPS)
        -> g_shunt.rx_*
        -> real_fb_read (intercept MMIO FB/SB)
        -> ARM osmocom (0xFFD001F0 = d_fb_det, etc.)
        -> camp + Location Update
```

Preuves runtime (modes shunt) : `LOCATION UPDATING ACCEPT` recu (lai = 001-01-1) ;
MM IDLE, normal service ; RXLEV serving ~ -47/-53 dBm.

**En NATIF, le blocage restant = FB pas detecte (`d_fb_det=0`) donc pas de camp, donc pas
de LU/RACH UL.** Diagnostic CHIFFRE (Run B, 2026-07-27, cf [`../../../run_results.md`](../../../run_results.md)) :
le correlateur CALCULE (accus A/B non-nuls, B2) mais son flux BOUCLE dans le bank 0
(`0x8d00`->`0xa07x`, B4B) sans jamais atteindre l etage decision -> `data[0x08f8]` **JAMAIS
ecrit** (watchpoint B4 = 0 write), alors que des writers de `0x08f8` **existent** dans la
PROM (scan = 30+ refs). Mur de controle de flux, prouve instruction par instruction. Voir §3.

---

## 2. Architecture reelle

Deux CPU emules, colles par MMIO API RAM :
- **ARM946** (`calypso_mb.c:303`, cœur ARMv5TE de QEMU) execute le firmware osmocom-bb.
  Il **modelise le vrai ARM7TDMI** (ARMv4T) du Calypso : v5TE est un sur-ensemble de v4T,
  le firmware osmocom (thumb/arm v4T) tourne donc a l'identique. Choix assume et coherent —
  quand un autre doc dit « ARM7TDMI », il parle du silicium modelise, pas du cœur QEMU.
- **TMS320C54x** execute la vraie mask-ROM Calypso (TI).
- Colle : `calypso_trx.c` (mirroir MMIO par ecriture) + `calypso_arm2dsp.c` (pont par instruction).

Loi d'adressage (invariante) : `DSP_word = 0x0800 + (ARM_off - 0xFFD00000)/2`. L'ARM lit les resultats DSP dans `s->dsp->data[off/2 + 0x0800]` — **PAS** dans `dsp_ram[]` ni `api_ram[]` directement. C'est pourquoi les `shunt_dispatch_*` passant par `dma_memory_write` sont invisibles (ils touchent `dsp_ram[]` non mirroir) ; seules les ecritures directes `data[]`/`api_ram[]` campent.

**Le correlateur DSP natif est un VRAI correlateur** (mask-ROM TI), PAS un stub. Verdict affine : son **BUFFER D'ECHANTILLONS RX n'est jamais cable au recepteur on-chip**. Sur silicium, ce buffer est rempli par le recepteur on-chip (DRP -> DMA), **non modelise en QEMU** (honnetete a garder). Verdict CHIFFRE 2026-07-27 (Run B) : meme entree nourrie (FB-STREAM `0x9213/0x9215`, rxlev -47 reel), le correlateur boucle sans atteindre l etage qui ecrit `d_fb_det` (writers presents dans la PROM mais jamais executes ; B4/B4B/SCAN). Detail : [`run_results.md`](../../../run_results.md) §Run B.

Preuve cote osmocom : `prim_fbsb.c` pose `d_fb_mode` puis polle `d_fb_det` ; `dsp_api.h` ne contient QUE des cellules RESULTAT — **aucun buffer IQ dans l'API RAM**. L'ARM ne fournit jamais l'IQ. Donc `d_fb_det` natif reste 0 tant que le buffer n'est pas nourri cote emulateur.

`calypso_fbsb.c` = logger MORT (log-only, aucune synthese). Ne pas s'y fier.

---

## 3. Etat du natif

**Verite terrain 2026-07-27 : le correlateur DSP TOURNE.** Il atteint le handler `0x8d00`,
passe en DETECTOR-RUN @`0x9ac0`, `d_fb_mode=1`. Le mur a BOUGE : ce n'est plus « jamais
dispatche / `0x8d00` 0 hit » (ancienne conclusion RANK3 perimee), c'est **`d_fb_det[0x08f8]`
reste a 0** faute d'entree valide. rxlev natif = DONE (-47 dBm reel).

**Point d'injection NATIF localise et PROUVE inscriptible :**
- L'etage demod `0x9f00` lit ses samples en `data[0x9213]` (I) / `data[0x9215]` (Q).
- RAM inscriptible : une rampe 0x1003/0x1005 y atterrit et est relue par le demod.
- **C'est la SOURCE a alimenter.**

**`0x2a00` = WORKZONE DE SORTIE du demod (PAS l'entree).**
- Le DSP la remplit lui-meme (0x12ed).
- Le kernel energie `0xa076` LIT `0x2a00`.
- **NE JAMAIS feeder `0x2a00`** (ni via rx_burst, ni via feed_iq). Feeder la source `0x9213`/`0x9215`.

**Blocages restants (plomberie/cadence, PAS logique morte) — pourquoi `d_fb_det` reste 0 :**
- **(a) FENETRE** : feeder un STREAM de samples. Le correlateur batit sa fenetre sur N appels ; 2 cellules figees ne suffisent pas.
- **(b) DISPATCH par-frame** : le demod ne tourne qu'~1 fois (33 lectures). Il faut le cadencer une fois par trame avec un sample frais.

Tant que (a)+(b) ne sont pas resolus, `d_fb_det` natif reste 0 (le correlateur tourne mais
sur une entree degeneree -> gap = 0). C'est le seul verrou entre NATIF et le camp.

**Regression corrigee (commit becd439 "i should go sleeping")** : un SKIP rx_burst avait ete ajoute (`calypso_bsp.c`, gate `FB_IQ_DARAM`) affamant `0x2a00` -> perte du SHADOW-DADST. Corrige : SKIP decouple sur gate dediee `CALYPSO_FB_IQ_OWNS` (defaut OFF) -> rx_burst nourrit toujours.

---

## 4. Go-live / shadow IMR

**Recette qui DONNE le shadow IMR (kernel energie qui corrèle) :**

```
CALYPSO_INIT_435B_OFF=0     # seed d[0x435b] = 0x52ed (sinon 0xa582 ecrit IMR=0)
CALYPSO_KEEP_IMR=1
CALYPSO_ARM2DSP_BGEN=1       # ARM pose les cellules background
CALYPSO_HACK=0               # natif
# SANS CTRLSYS
```

**`CALYPSO_ARM2DSP_CTRLSYS=1` CASSE la recette BGEN.** Il re-force `data[0x0810]` bit15 (= `B_TASK_ABORT`) a `0xa537` (~20011x) plus vite que le DSP ne le clear a `0xa549` (~89x) -> abort permanent -> plus de shadow, plus de kernel. **FIX : CTRLSYS=0.** (Note historique : bit15 de 0x0810 = B_TASK_ABORT, PAS un gate go-live a SET — inverse la vieille conclusion CTRLSYS.)

**SHADOW-DADST** = sonde sur les instructions DADST du kernel energie `0xa076` (ce n'est PAS le shadow IMR). Sa presence = **le kernel corrèle**, il est VIVANT.

**Pieges de cap de log** : on croit souvent le shadow "perdu" alors que le code tourne. Cause = caps de log :
- `KEEP-IMR` cappe a < 8.
- `SHADOW-DADST` cappe a `< 40 || %2000`.
Le grep parait s'arreter mais le kernel tourne toujours. Ne pas conclure "mort" sur un cap de log.

---

## 5. References canoniques

### Adresses DSP / ARM
| Cellule | DSP word | ARM addr |
|---|---|---|
| d_fb_det | `0x08F8` | `0xFFD001F0` (NDB+0x48) |
| a_sync_demod TOA/PM/ANGLE/SNR | `0x08FA..0x08FD` | NDB+0x4C.. |
| d_dsp_page | `0x08D4` | `0xFFD001A8` (BASE_API_NDB) |
| d_dsp_state | `0x08E2` | (=3 IDLE au boot) |
| d_ctrl_system / 0x0810 (bit15=B_TASK_ABORT) | `0x0810` | — |
| **sample IN natif (I / Q)** | `data[0x9213]` / `data[0x9215]` | — |
| **workzone OUT demod (NE PAS feed)** | `0x2a00` | — |
| kernel energie (lit 0x2a00) | `0xa076` (0x9a80 mode large) | — |
| shadow IMR seed | `data[0x435b]` = `0x52ed` | — |
| PROM0 base | `0x7000` | — |

### Task IDs DSP
`PM=1, FB=5, SB=6, RACH=10, NBS=19, NP=21, ALLC=24, CHECKSUM=33`. (BSIC = bits2:7 de a_sch[3] | a_sch[4]<<16.)

### Gates / mecaniques
- Recalage FN DSP/TRX = **-552** (`calypso_trx_get_fn`, FN calee sur SCH) ; LOST timer read-driven.
- Vecteurs IT : INT3/frame = vec19 bit3 (0xFFCC) ; TINT0 = vec20 bit4 ; BRINT0 = vec21 bit5 (0xFFD4).
- Modele gain trf6151 : `a_pm = (target_rf + 71 + gain_trf) * 64` ; reset REG_RX 0x9E00 = 138.

### Env (extrait — 27 vars, idiome `:=`, toutes overridables CLI)
`CALYPSO_INIT_435B_OFF`, `CALYPSO_KEEP_IMR`, `CALYPSO_ARM2DSP_BGEN`, `CALYPSO_ARM2DSP_CTRLSYS`, `CALYPSO_HACK`, `CALYPSO_FB_IQ_OWNS`, `CALYPSO_DSP_SHUNT`, `CALYPSO_NATIVE`, `CALYPSO_NATIVE_HELPED`, `SHUNT_LEGIT`, `SHUNT_NO_LEGIT`.

### Runtime
- Tree LIVE = **`/opt/GSM/qemu-src`** (`.latest.bak`/`.bak` = anciens ; overlay `qemu-calypso` = MORT au runtime, ne pas patcher).
- Firmware = `/opt/GSM/osmocom-bb-transceiver`.
- Lancement : `osmo_egprs/start-direct.sh` -> `qemu-src/start-clean.sh` (source `calypso.env` en `set -a`) -> `run.sh`.
- **L'utilisateur relance la pile lui-meme.** Claude = edits + diagnostic lecture seule.

---

## 6. Fausses pistes a NE PAS rechasser

- **SHADOW_WIRE** (rend le natif = 0).
- **Feeder `0x2a00`** (workzone de sortie, PAS l'entree).
- `d_fb_det` via reroute **FB_ENERGY seul** (correlateur reste affame).
- **Go-live INTM / storm / seed 5ac8** (red herring ; INTM=1 forever = modelisation, pas un bug logique).
- **Cells background `0x098a` / `0x098c`** (d_background_enable/state, ARM=0 par design).
- **Rotation SI** (meas->frames==0 = mobile ne recoit rien en idle, pas SI).
- `calypso_fbsb.c` (logger mort).
- `CALYPSO_ARM2DSP_CTRLSYS=1` (casse la recette BGEN).
- **Poke** (falsifier le retour d'une fonction) : insuffisant, le mur est du WIRING inter-blocs.
- **api_write_cb** : declare `calypso_c54x.h:204`, jamais assigne (grep=0) — dead callback, ne pas theoriser dessus.
- Sous-buffer `0x3fb0`/`0x03F0` comme sample-in natif (faux ; vrai = `0x9213`/`0x9215`).

---

## 7. Index `doc/archive/` — reference durable

Ces documents ne decrivent PLUS l'etat (leurs narratifs dynamiques sont perimes), mais restent des **references ISA / HW durables** :

| Doc archive | Contenu de reference |
|---|---|
| `opcodes/0x68_0x6F.md` | ANDM/ORM/XORM/ADDM/BANZ/BANZD/MAR ; famille 0x6F00 ; adressage Smem/SHIFT |
| `opcodes/0xF3.md` | Famille F3xx (INTR k, AND/OR/XOR/SFTL 1-mot, #lk 2-mots) ; SFTL vs SFTA |
| `project/STEP2_BC_CONDS.md` | Codes condition BC group-1 (CCEQ/CCNEQ/CCLT/... markers) |
| `C54X_INSTRUCTIONS.md` | XC/FRET/FCALL/RETE/BANZ/BC encodings (SPRU172C) |
| `opcodes/tic54x_hi8_map.md` | Map hi8->mnemonic complet ; layout ST1 ; indices MMR |
| `project/AUDIT_DECODER_20260508.md` | Verites binutils tic54x (STL/STH/CMPS/BIT/MVPD...) ; Tier A landé |
| `DSP_ROM_MAP.md` | Sections ROM (PROM0 0x7000, DROM/PDROM) ; API RAM 0x0800+ |
| `project/BUGS_AND_FIXES.md` | F272=RPTBD, F274=CALLD, 0x8A00=POPM, SP init 0x5AC8, RET stubs |
| `CALYPSO_HW.md` | Datasheet HW/protocole (API RAM, TPU_CTRL, TRXDv0, sercomm, L1CTL) |
| `SERCOMM_GATE_ARCHITECTURE.md` | Chemins BSP/L1CTL, HDLC/DLCI, task IDs, d_dsp_page@0x08D4 |
| `hardware-map.md` | Memory map (Flash/IRAM/XRAM), bases peripheriques, IRQ map, INTH |
| `SESSION_20260405_NIGHT4.md` | Encodings F0xx/RSBX/SSBX/RET/RETD/CALLD/IDLE ; vecteurs IT |
