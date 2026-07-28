# ETAT ACTUEL — QEMU-Calypso (source de verite unique)

> Document de reference courant. Ancre sur les mesures du **2026-07-28**.
> **Regle de resolution de conflit :** ce document prime sur tout fait extrait d'un autre
> doc. Le STATUT DEPEND DU MODE : ne jamais citer un statut sans le mode. Toute
> affirmation technique doit nommer son INSTRUMENT. Les autres documents sont dans
> `doc/archive/` (reference durable uniquement, cf. §10).
>
> Convention de marquage utilisee partout ci-dessous : **MESURE** (instrument nomme),
> **HYPOTHESE** (testable, non tranchee), **INVALIDE** (mesure contraire, cf. §8).

---

## 0. Verifier ce qu'on mesure AVANT de mesurer

Trois pieges ont produit de fausses conclusions et sont desormais des prerequis.

| Piege | Ce qui est faux | Instrument correct |
|---|---|---|
| **`BUILD-STAMP` ne date PAS le binaire** | la macro `__DATE__/__TIME__` vit dans `calypso_dsp_shunt.c:107` : elle date **son propre TU**. Un run peut afficher `09:42:13` avec un binaire relie a `11:37`. | `stat -c %y build/qemu-system-arm` + `find build -name '*calypso_c54x*.o' -printf '%T+ %p\n'` + `ps -eo pid,lstart,cmd \| grep qemu-system-arm` |
| **Artefacts /dev/shm perimes** | `daram_2a00.cfile` peut dater d'un run precedent et etre lu comme la sortie du run courant | comparer le mtime au `lstart` du process ; `rm -f /dev/shm/daram_2a00.cfile` AVANT le run |
| **Compteurs plafonnes par le logger** | `calypso_dsp_shunt.c:1670` : `if (rfl < 20 \|\| (det && rfl < 300))`. « 280 det sur 300 » = contenu des 300 premieres LIGNES, pas un taux de detection | lire le code du logger avant de citer un ratio |

**Variables effectives ≠ variables posees.** `CALYPSO_SHUNT_LEGIT` est une *value-list*
resolue par un constructeur `__attribute__((constructor))` (`calypso_dsp_shunt.c:86-105`)
qui fait `setenv(..., overwrite=1)` : `DSP` force `CALYPSO_DSP_RUN_C54X=1` meme si le
`.env` a pose `0`. Instrument : le `[calypso-manifest]` du run, ou
`tr '\0' '\n' < /proc/<pid>/environ | grep ^CALYPSO_`. Un `env | grep` cote shell ment.

---

## 1. Statut par mode

> **Il n'y a PAS de statut absolu — le statut depend du MODE de run.** L'ancienne
> "contradiction" (« LU ACCEPT recu » vs « FB jamais detecte ») etait DEUX MODES
> CONFONDUS : la LU marche en shunt, elle est bloquee en natif.

### Les modes (socle `calypso.env` + un `calypso_X.env`)

- **`SHUNT_LEGIT=1`** — base. FBSB host-side (`real_fb` + gr-gsm -> intercept de LECTURE
  ARM), cannes de stabilisation, DSP c54x OFF. **MODE FIABLE.**
- **`SHUNT_NO_LEGIT=1`** — memes injections, nommees une par une, sans le parapluie.
- **`SHUNT_LEGIT=DSP,NO_CANNED`** (value-list) — DSP c54x tourne en // et sans cannes.
  Plus proche du reel, plus flaky.
- **`CALYPSO_NATIVE=1`** (NATIF) — corrélateur DSP, entree par intercept de lecture
  `data[0x9213]/[0x9215]` (`FB_STREAM`), `CORR_ENTRY=0x9500` dans le `.env` livre.
- **`CALYPSO_NATIVE_HELPED=1`** (NATIF AIDE) — corrélateur DSP + `feed_iq` en DARAM
  (`FB_IQ_BASE=0x9210`) dans le `.env` livre.
- **`CALYPSO_L1=c`** — **MODE MORT**, non executable : `calypso_dsp_shunt.c:1847` arme le
  shunt sur `l1_c_active()`, et `calypso_trx.c:1475` exige
  `l1_c_active() && !shunt_active()`. Les deux ne peuvent pas etre vraies ensemble ;
  `calypso_layer1_tick()` n'a **aucun** appelant atteignable. Ni HLE, ni natif.
  (DEDUCTION DE CODE, non testee au run.)
- **`./start-clean.sh` sans profil** — mode degenere : `run.sh:1137` pose
  `CALYPSO_DSP_SHUNT=1` (preset `full-grgsm`) -> `substitutes()=vrai` -> le c54x **ne
  tourne pas** malgre `RUN_C54X=1`, et **aucune** injection n'est armee. Ne campe pas et
  n'exerce pas le natif. (DEDUCTION DE CODE.)

**Les deux profils natifs livres posent encore `CALYPSO_FB_CORR_ENTRY=0x9500`**
(`calypso_native.env:16`, `calypso_native_helped.env:11`), alors que la configuration
mesuree correcte est `0x94f5` (§3, M6). Ecart connu, corrige en CLI par le run de
reference §9. Defaut non modifie.

### Matrice statut x mode

| Feature | SHUNT_LEGIT=1 | SHUNT_NO_LEGIT | DSP,NO_CANNED | NATIVE / NATIVE_HELPED | Instrument |
|---|---|---|---|---|---|
| FB/SB sync | **DONE** (host, `coh=0.999 dphi=0.387 det=1`) | DONE | DONE | **KO** : le corrélateur tourne, `d_fb_det[0x08f8]=0` | `REAL-FB fn=` / `DETECTOR-RUN` |
| rxlev serving | DONE (-47 dBm) | DONE | DONE | DONE mais **mocke** : `shunt_dispatch_pm` (`calypso_dsp_helper.c:673`) n'a **aucun gate positif** | `CALYPSO_TRF_RXLEV`, `MON: f=` |
| Camp (C3) + sysinfo | **DONE** (20 SI decodes) | DONE | DONE | **KO** « No sysinfo », `BSIC=0` | `grep -c sysinfo /root/mobile.log` |
| LU + registration | **DONE** (LU ACCEPT `lai=001-01-1`, TMSI `0x3dbeb85f`, 2,70 s RACH->ACCEPT) | DONE | DONE | KO (pas de camp) | `grep -icE "LOCATION UPDATING ACCEPT"` |
| SMS MO / MT | DONE | DONE | **WIP flaky** (anti-stall ajoute) | KO | log SMSC |
| Ctrl-C mobile -> re-acquisition | DONE (re-arm BGEN sur `d_dsp_page=0`) | DONE | DONE | non teste | `calypso_arm2dsp.c`, hook `on_arm_write` |
| Voix TCH/F | **WIP** : ASSIGNMENT COMMAND -> ASSIGNMENT FAILURE | WIP | WIP | KO | `doc/VOIX_PLAN.md` |
| c54x execute a la cadence trame | non | non | **oui** (runner shunt) | **oui** (tick TDMA) | `dsp_n_exec_2/5`, absence de `DSP Error Status: 2048` |
| **Entree du demod alimentee** | s.o. | s.o. | non mesure | **DONE** : `data[0x4c00]` lu, valeurs reelles `ff6e`, `c1fb`, `d147` | `CALYPSO_WATCH_9F00_RD` (PC=0x9fb5) |
| **Sortie du demod exploitable** | s.o. | s.o. | non mesure | **KO** : DC quasi pur et fige, `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` | `CALYPSO_DARAM_DUMP` + `corr_iq.py --src ddump` |

**Reserve sur la robustesse LU (non tranchee).** `SHUNT_LEGIT_ADDRESS_MAP.md` §9 conclut
« LU ACCEPT INTERMITTENT (~1 succes / 19 retries T3211) » alors que `run_results.md` Run A5
mesure « 2,70 s, 0 retry ». A5 est plus recent et posterieur au fix sous-voie SDCCH/8, mais
n=2. **A rejouer** : `grep -c T3211 /root/mobile.log` sur 5 runs `SHUNT_LEGIT` consecutifs.

### ⚠️ Hygiene de mode : ce qui masque le natif a l'observateur

Les deux `.env` natifs posent `CALYPSO_DECAN:=1`, qui **implique `SHUNT_REAL_FB`**
(`calypso_dsp_shunt.c:1468` et `:1594`). L'intercept
`calypso_dsp_shunt_real_fb_read()` (`:1463-1490`, appele par `calypso_trx.c:297`) sert
alors a l'ARM le resultat du detecteur **hote** sur `0x01F0/F4/F6/F8/FA` et `0x0060/0x0088`,
pas la cellule DSP.

Consequences, a retenir :
1. `d_fb_det = 0` **lu cote ARM** en natif est **ambigu**. Seule la cellule DSP
   `data[0x08f8]` (imprimee par `DETECTOR-RUN`, ou la sonde `CALYPSO_FBDET_API` qui lit les
   deux cotes du miroir) tranche.
2. Le comportement du mobile dans un run natif avec `REAL_FB=1` **ne mesure pas le natif**.
   Pour un natif nu : `CALYPSO_DECAN=0 CALYPSO_SHUNT_REAL_FB=0` en CLI (l'idiome `:=`
   le permet sans toucher aucun defaut).
3. Le **PM est mocke dans tous les modes**, y compris natif (`shunt_dispatch_pm`,
   `calypso_dsp_helper.c:673`, seul `CALYPSO_SHUNT_NO_FAKE_PM=1` le coupe).

### Chaine operationnelle (la SEULE voie FBSB qui campe — modes shunt)

```
twl3025/DECAN (AFC/PM/rxlev, gain trf6151)
gr-gsm + detecteur FCCH host (coherence + dphi)
        -> g_shunt.rx_*
        -> real_fb_read  (INTERCEPT DE LECTURE MMIO, calypso_trx.c:297)
        -> ARM osmocom (0xFFD001F0 = d_fb_det, etc.)
        -> camp + Location Update
```

**Le « fake FB » est mort dans tous les profils livres** : `CALYPSO_INJECT_FB` n'est pose
par aucun `.env` (`grep -rn INJECT_FB *.env` = 0 hit). Ce que `SHUNT_LEGIT` fait n'est pas
d'ecrire un faux resultat dans `data[]`, c'est d'**intercepter la lecture ARM**.
Corollaire operationnel : **aucun watchpoint sur `data_write_locked` ne verra jamais cette
livraison**.

**Oracle reseau.** La pile temoin `bts1` (osmocom-bb sur `trxcon` + `fake_trx`, sans QEMU)
obtient un LU ACCEPT sur le meme coeur (`grep -c "LOCATION UPDATING ACCEPT"
/root/mobile-bts1.log` = 1). Tout echec cote Calypso est imputable a l'emulation, pas au
coeur Osmocom.

---

## 2. Architecture reelle

Deux CPU emules, colles par MMIO API RAM :
- **ARM946** (`calypso_mb.c:303`, coeur ARMv5TE de QEMU) execute le firmware osmocom-bb.
  Il **modelise le vrai ARM7TDMI** (ARMv4T) du Calypso : v5TE est un sur-ensemble de v4T.
  Quand un autre doc dit « ARM7TDMI », il parle du silicium modelise, pas du coeur QEMU.
- **TMS320C54x** execute la vraie mask-ROM Calypso (TI).
- Colle : `calypso_trx.c` (miroir MMIO) + `calypso_arm2dsp.c` (pont par instruction).

Loi d'adressage (invariante) : `DSP_word = 0x0800 + (ARM_off - 0xFFD00000)/2`. L'ARM lit
les resultats DSP dans `s->dsp->data[off/2 + 0x0800]` — **PAS** dans `dsp_ram[]` ni
`api_ram[]` directement. C'est pourquoi les `shunt_dispatch_*` passant par
`dma_memory_write` sont invisibles ; seules les ecritures directes `data[]`/`api_ram[]`
campent.

**Le correlateur DSP natif est un VRAI correlateur** (mask-ROM TI), PAS un stub.
Sur silicium, son buffer d'echantillons est rempli par le recepteur on-chip (DRP -> DMA),
**non modelise en QEMU** — c'est ce que le BSP remplace aujourd'hui (§3).

Cote osmocom : `prim_fbsb.c` pose `d_fb_mode` puis polle `d_fb_det` ; `dsp_api.h` ne
contient QUE des cellules RESULTAT — **aucun buffer IQ dans l'API RAM**. L'ARM ne fournit
jamais l'IQ.

`calypso_fbsb.c` = logger MORT (log-only, aucune synthese). Ne pas s'y fier.

---

## 3. Chaine de signal RX — etat maillon par maillon (MESURE 2026-07-28)

```
[osmo-bts-trx] --TRXDv0 UDP :6702--> M1 bsp_trxd_readable
   M2 decimation /4 -> 148 paires @1 SPS      G1 gate shunt  calypso_bsp.c:474
   M3 DIRECT_FEED -> rx_burst                 G2 gate shunt  calypso_bsp.c:997
   M4 ecriture data[DARAM_ADDR]               G3 gate shunt  calypso_bsp.c:1359 (LIVRAISON)
   M6 dispatch CALA @0xb01e -> reroute FB_ENERGY -> FB_CORR_ENTRY (0x94f5)
   M7 etage demod 0x9f95..0x9fe2  -- LIT data[0x4c00] en pas de 5
   M8 data[0x2a00..0x2b27] = WORKZONE DE SORTIE (0x9fb8 = I, 0x9fe2 = Q)
   M10 noyau MAC 0xa076..0xa09d
   M11 detecteur 0x9ac0 (boucle 0x9ab8..0x9ac2)
   M12 d_fb_det = data[0x08f8] = api_ram[0xF8] = MMIO ARM 0x01F0
```

| # | Maillon | Etat | Instrument |
|---|---|---|---|
| M1 | UDP TRXDv0 -> BSP | **OK** | `CALYPSO_DEBUG=BSP` (`RXSZ`, `RX tn= fn=`) |
| M2 | Decimation 4 SPS -> 1 SPS | **OK, conforme** : `IQ_DECIM=4`. Corollaire `DARAM_LEN=296` | `corr_iq.py --src bursts` -> `VERDICT: FCCH @1SPS PROPRE (dphi=+1.00x pi/2)`, `coh=0.998`, `rms=3.25e4`, `\|DC\|=379`, `zeros=0%`, FFT +67 708 Hz |
| G1-G3 | **Trois** gates BSP (pas deux) | **LEVES** par `CALYPSO_BSP_DARAM_FORCE=1`. G3 (`:1359`) est la **LIVRAISON** vers `data[]` ; il ne connaissait que `TPU_RX_WIRE` — aligne le 2026-07-28 sur `RUN_C54X=1 && DARAM_FORCE`, **defaut inchange**. Avant ce fix : 2 verrous ouverts sur 3, rien n'arrivait au DSP | `[bsp] deliver: gate shunt LEVE (rxw=1)` present ; `grep -c "dropping fn="` = **0** |
| M3 | Chemin de livraison | `DIRECT_FEED=1` court-circuite `bsp_enqueue`/`deliver_buffered` : la fenetre FN ±64 et le pulse BDLENA **ne sont pas** dans le chemin actif | manifeste `CALYPSO_BSP_DIRECT_FEED` |
| M3b | `c54x_bsp_load` -> `bsp_buf[]` | **BRANCHE MORTE** : seul consommateur = `PORTR PA=0xF430` (gate `CALYPSO_FIX_PORTR`) ; mesure `PORTR-ANY` = PA ∈ {`0x0003`, `0xfc28`}, **jamais** `0xF430`/`0x0034` | `CALYPSO_DEBUG=PORTR-ANY` |
| M4 | Ecriture DARAM | **OK** a `DARAM_ADDR=0x4c00`, `LEN=296` | `DMA fn= -> DARAM[0x....]` (exige `CALYPSO_DEBUG=BSP`) |
| M5 | Reveil DSP (INT3 vec19, BRINT0 vec21) | **hors chemin critique** : le handler `0x8d00 -> 0xa076` est du **polling pur** sur `data[0x3fad]` bit15 | `SPIN-IT`, `CALYPSO_PROBE_3FAD_GATE` |
| M6 | Dispatch FB | cible **native** de la `CALA @0xb01e` = `0xab38` = **stub `RET`**. Le reroute `CALYPSO_FB_ENERGY=1` amene au corrélateur energie a `FB_CORR_ENTRY` | `FB-ENERGY-REROUTE CALA@0xb01e tgt 0xab38 -> 0x94f5` |
| M7 | **Entree du demod** | **VIVANTE** : PC `0x9fb5` lit `0x4c00/05/0a/0f/15/1a` (**pas de 5** = polyphase 6 taps) avec des valeurs reelles `ff6e`, `c1fb`, `d147` | `CALYPSO_WATCH_9F00_RD` |
| M8 | **Sortie du demod `0x2a00`** | **MORTE** : DC quasi pur et **fige**, `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` ; cellule temoin invariante sur 157-203 bursts (`0x9fb8@0x2a00=0x0000`, `0x9fe2@0x2a00=0x52ed`) — **identique avec `DECIM=1` et `DECIM=4`** | `CALYPSO_DARAM_DUMP` (ddump, non-racy) + `corr_iq.py --src ddump` |
| M9 | Scratch `0x2c00` | `0x9fd5` y depose une table de coefficients **CONSTANTE** ; `0xa03f-0xa042` = `LD #0xe000,A` + `RPT #14` + `STL A,*AR5+` = remplissage constant | `CALYPSO_WZWRITE`, `CALYPSO_WMAP` |
| M10 | Noyau MAC `0xa076..0xa09d` | 60 000 ecritures pour **4 a 7 valeurs distinctes**, toutes dans `{0001,0002,001f,003e}` ± bit `0xe000`. `003e=2x001f`, `0002=2x0001` : un accumulateur qui double une constante, **pas une correlation** | `CALYPSO_WMAP` (LO/HI sur `0xa07x`) |
| M11 | Detecteur `0x9ac0` | **ARME et il tourne** : `DETECTOR-RUN #3600 d_fb_mode[08f9]=0x0001` | log `DETECTOR-RUN` (inconditionnel) |
| M12 | `d_fb_det` | **0** sur la totalite des runs observes (3 600 puis 32 200 executions du detecteur). Publisher natif **unique** = `ORM #1,*(0x08f8)` @`0x79e4`, banque commune `0x7700-0x79F0`, **jamais executee** | `CALYPSO_B4`, `CALYPSO_SCAN_08F8`, `CALYPSO_FBDET_API` |

### 3.1 L'adresse d'entree DEPEND du point d'entree du corrélateur

Ce n'est pas une constante du DSP, c'est une consequence du reroute
`calypso_c54x.c:6045-6067`. **Toujours mesurer avec `CALYPSO_WATCH_9F00_RD` AVANT de feeder.**

| Configuration | Le demod LIT | Outil qui sert | Outil INERTE |
|---|---|---|---|
| `FB_ENERGY=1` + `CORR_ENTRY=0x9500` (defaut des `.env` natifs) | `data[0x9213]` / `[0x9215]` | `CALYPSO_FB_STREAM` | `BSP_DARAM_ADDR` |
| `CORR_ENTRY=0x94f5` + `BSP_DARAM_ADDR=0x4c00` (**run de reference**) | `data[0x4c00]`, pas de 5 | BSP + `DARAM_FORCE` | `FB_STREAM` (`0x9213`/`0x9260` **jamais lus**) |
| toutes | `data[0x2a00]` = **ECRITURE** du demod | — | **ne jamais feeder** |

`0x9500` n'apparait **nulle part** dans les 28 672 mots de PROM ; `0x94f5` est l'entree
referencee en ROM (`@0x87e7 f930 94f5`) et le defaut du code (`calypso_c54x.c:6057`).
Entrer en `0x9500` saute les 11 mots de mise en place qui posent `AR6`.

**Incoherence residuelle du run de reference (HYPOTHESE, testable).** Il herite de
`calypso_native_helped.env` un `FB_IQ_DARAM=1` + `FB_IQ_BASE=0x9210`, donc `feed_iq` ecrit
`data[0x9210..0x9337]` — region **jamais lue** avec `CORR_ENTRY=0x94f5`. Ecriture
vraisemblablement **inerte** (elle consomme du CPU sans alimenter personne). Tranche :
`CALYPSO_WATCH_9F00_RD` (absence de `addr=0x921x`) ou
`CALYPSO_RMAP=1 CALYPSO_RMAP_PCLO=0x9f00`.

### 3.2 Resume en une phrase

**ENTREE VIVANTE, SORTIE MORTE.** Le signal qui entre est bon et prouve (M2, M7) ; le
demod lit du signal reel et produit du DC constant (M8) ; le noyau MAC double une constante
(M10) ; le detecteur tourne mais ne detecte rien (M11) ; `d_fb_det` reste 0 (M12). Le
defaut est **dans le calcul**, pas dans l'alimentation.

---

## 4. CIBLE N.1 — bug de decodage d'opcode `0x1800` (AND lu comme LD)

### 4.1 La chaine fautive, verifiee mot a mot contre `calypso_dsp.PROM0.bin`

```
9fa1: 7660 000f   ST  #0x000f, *(0x60)     -> data[0x0060] = 15
9fa3: 7661 0010   ST  #0x0010, *(0x61)     -> data[0x0061] = 16
...
9fb1: 1860        AND *(0x60), A           <-- LE BUG
9fb2: 0061        ADD *(0x61), A
9fb3: 880e        STLM A, MMR 0x0E = T
9fb5: 14ce        LD Smem, TS, A           (TS = T[5:0], -16..+31, SPRU172C)
9fb8: 8694        STH A, ASM, *AR4+        -> workzone 0x2a00
```

`0x1860` est un **AND** (`doc/opcodes/tic54x_hi8_map.md:26` — `0x18..0x19 | and |
0x1800 / 0xFE00`). Or `sub = (op >> 9) & 7` donne `sub = 4`, et le `switch` de `case 0x1:`
(`calypso_c54x.c:9320`) ne nommait que les sub 0..3 (`0x1000` LD / `0x1200` LDU /
`0x1400` LD,TS / `0x1600` LDR). `sub=4` tombait dans le `default` = **chargement signe**.

Consequence mesuree, instruction par instruction :

| PC | Attendu | Fait avant fix | Etat |
|---|---|---|---|
| `0x9fb1` | `A = A AND 15` | `A = 15` | — |
| `0x9fb2` | — | `A = A + 16` | `A = 31` |
| `0x9fb3` | — | `STLM A, T` | **`T = 31`** |
| `0x9fb5` | decalage borne | `LD Smem, TS` avec `TS = +31` | **`A = 0x80000000` (saturation)** |
| `0x9fb8` | echantillon filtre | ecrit l'accumulateur sature | **sortie independante des operandes = DC plat** |

L'implementation de `LD Smem,TS` (`:9344`) est **correcte** ; le bug etait **en amont**.
Coherent avec M8 (sortie plate) et M10 (constante doublee).

### 4.2 Etat du correctif

**POSE EN SOURCE, COMPILE, NON ENCORE VALIDE AU RUN.**
`calypso_c54x.c:9365-9389` implemente desormais les quatre sub manquants, Smem
**zero-etendu sur 40 bits**, conforme a l'exemple TI (SPRU172C p.4-12 pour AND, p.4-192
pour SUBC) :

| sub | encodage | mnemonique |
|---|---|---|
| 4 | `0x1800` | `AND Smem, src` |
| 5 | `0x1A00` | `OR Smem, src` |
| 6 | `0x1C00` | `XOR Smem, src` |
| 7 | `0x1E00` | `SUBC Smem, src` |

### 4.3 Second site touche par le meme bug

La boucle du **detecteur** `0x9ab8..0x9ac2` reproduit exactement le motif AND -> T :

```
9aba: 1983   AND *AR3, B      (hi8 0x19 = and, dst = B)
9abc: 890e   STLM B, MMR 0x0E = T
9abe: 348e   BITT
```

Avant le correctif, `0x1983` etait lui aussi execute comme un `LD` : le masque de bits
applique avant le chargement de `T` etait remplace par un chargement brut.
**MESURE** (decodage PROM) ; **HYPOTHESE** quant a l'effet sur `d_fb_det`.

### 4.4 Le critere de tranche — et ce qu'il ne faut PAS regarder

Un premier run post-correctif (44 s) donne encore `d_fb_det = 0` sur 3 600
`DETECTOR-RUN`. **Cela ne suffit pas a conclure** : `d_fb_det` est en bout de chaine et
depend aussi de M6/M12 (dispatch stub, publisher jamais execute).

> **Le critere est la SORTIE DU DEMOD, pas `d_fb_det`.**
> Reference a battre : `\|DC\| = 2,86e4` pour `rms = 2,94e4`, `dphi = +0,004`.
> Cible : `coh > 0,90`, `dphi ≈ +1,571` (= +1,00 x pi/2).
> Mesure : `rm -f /dev/shm/daram_2a00.cfile` avant le run, `CALYPSO_DARAM_DUMP=1`,
> puis `python3 tools/corr_iq.py --src ddump | tail -3`.

---

## 5. Blocages suivants, par ordre

| Rang | Blocage | Statut | Tranche |
|---|---|---|---|
| B1 | Decodage `0x1800/1A00/1C00/1E00` -> `T=31` -> saturation -> sortie demod constante | **corrige en source, non mesure** | §4.4 |
| B2 | **`DSP_ERR_DMA_PEND` (0x20 = 32) permanent** : `grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log \| sort \| uniq -c` -> **723 x « 32 »**, aucune autre valeur. HYPOTHESE : le DSP attend l'achevement d'une DMA ; le BSP ecrit `data[]` **directement**, sans la machinerie DMA, donc le drapeau ne se libere jamais. *(Progression `2048` STACK_OV -> `32` DMA_PEND : pas une regression, le DSP va assez loin pour se plaindre de la couche suivante.)* | ouvert | nommer qui pose / qui efface l'etat cote modele |
| B3 | **Format d'entree** : le demod lit en pas de 5, le BSP depose 296 int16 **contigus** I/Q entrelaces. Non tranche : il n'est pas exclu que le pas de 5 soit correct et que ce soit le **remplissage** qui doive changer de layout | hypothese | `CALYPSO_WATCH_9F00_RD` + relecture du filtre polyphase |
| B4 | **Slot de dispatch FB = stub `RET`** : `0xb01c: 10f8 43d8` (adressage **absolu**) ; `data[0x43d8] = 0xab38` ; premier mot de `0xab38` = `fc00` = `RET`. Un **seul** ecrivain sur les 4 banks : `0xbb00`. Watchpoint `data_write_locked` : aucun writer cache. Question ouverte unique : pourquoi l'ordonnanceur `0x7234` ne tombe-t-il jamais sur `0x8341` (la LUT FB) ? | ouvert | bequille de VALIDATION (pas correctif) : `CALYPSO_BSP_DISPATCH_FB=1` |
| B5 | **En natif : ni SCH ni SI**, donc pas de camp meme si le FB tombait. Mesure : `dispatch_allc` = 0, `feed_agch` = 0, `DISPATCH SDCCH` = 0, `sb_valid` = 0 ; le listener `feed_si(a_cd)` est arme (`udp:127.0.0.1:4730`) mais rien n'y arrive (gr-gsm non branche en natif). Cote mobile : `No sysinfo yet` -> `BSIC=0` x7 -> `Cell selection failed, read timeout`. Cote osmocon : `attempt=12` = **chemin de renoncement** d'osmocom, pas une synchro | connu | ordre du plan P4 : FB -> SCH -> SI, oracle = le producteur actuel a chaque etape |
| B6 | **Voix TCH/F : ASSIGNMENT FAILURE.** Double cause localisee : **(A)** `calypso_dsp_shunt.c:177` lit **inconditionnellement** `a_cu@0x264` (SDCCH/SACCH) quel que soit le type de tache, alors que le firmware ecrit la FACCH dans `a_fu@0x282` (`prim_tch.c:422`) et la voix dans `a_du_1@0x134` (`:485`) ; **(B)** `tools/calypso-ipc-device/qemu_wrap.c:1194` cable la voie UL en dur TS0/RACH (`ul_slotoff=1875`), or le TCH/F est assigne **TS2**. Deja cable, ne pas reecrire : TCH DL sub0 (`calypso_tch_dl_poll:333-350`, `shunt_dispatch_tch_dl:356-369`, routage `:836-837`) ; **il manque le PRODUCTEUR** du sideband `/dev/shm/calypso_tch_dl`. Reseau OK (`call fake_trx` = ACTIVE + audio, GAPK/FR compile) | WIP | rejouer en `SHUNT_LEGIT` pour re-chiffrer (0 occurrence dans les logs courants) |
| B7 | **SMS flaky en `DSP,NO_CANNED`** (jitter quand le c54x tourne en //). En `SHUNT_LEGIT` les 3 politiques d'eviction sont a zero (`overflow=0, ttl=0, reps=0`, profondeur max 1) — mais la saturation `depth=32` existait **en `DSP,NO_CANNED`**. **Ne rien supprimer de `sdcch_ring`** avant d'avoir mesure dans le mode ou il sert | WIP | `grep "EVICT-STATS" /root/qemu.log \| tail -1` en `DSP,NO_CANNED` |
| B8 | `bsp_buf[]` rempli par `c54x_bsp_load`, lu par personne (`PORTR PA` jamais `0xF430`) | dormant | `CALYPSO_DEBUG=PORTR-ANY` |
| B9 | `DIRECT_FEED=1` court-circuite la fenetre FN et le pulse BDLENA (RANK2) | connu, contourne | — |

---

## 6. Go-live / shadow IMR

**Recette qui DONNE le shadow IMR (kernel energie qui correle) :**

```
CALYPSO_INIT_435B_OFF=0     # seed d[0x435b] = 0x52ed (sinon 0xa582 ecrit IMR=0)
CALYPSO_KEEP_IMR=1
CALYPSO_ARM2DSP_BGEN=1      # ARM pose les cellules background
CALYPSO_HACK=0              # natif
# SANS CTRLSYS
```

**`CALYPSO_ARM2DSP_CTRLSYS=1` CASSE la recette BGEN.** Il re-force `data[0x0810]` bit15
(= `B_TASK_ABORT`) a `0xa537` (~20 011x) plus vite que le DSP ne le clear a `0xa549`
(~89x) -> abort permanent -> plus de shadow, plus de kernel. **FIX : CTRLSYS=0.**
(Note historique : bit15 de `0x0810` = `B_TASK_ABORT`, PAS un gate go-live a SET —
inverse la vieille conclusion CTRLSYS.)

**SHADOW-DADST** = sonde sur les instructions DADST du kernel energie `0xa076` (ce n'est
PAS le shadow IMR). Sa presence = **le kernel correle**, il est VIVANT.

**Pieges de cap de log** : on croit souvent le shadow « perdu » alors que le code tourne.
`KEEP-IMR` cappe a `< 8` ; `SHADOW-DADST` cappe a `< 40 || %2000`. Ne pas conclure « mort »
sur un cap de log.

---

## 7. Verrous BSP et parametres de destination

Trois gates, tous conditionnes a `calypso_dsp_shunt_active()` — **donc actifs meme en
NATIF**, parce que `CALYPSO_DSP=c54x` (defaut `calypso.env:102`) suffit a poser
`active()=vrai`. `active()` ne veut PAS dire « le shunt remplace le DSP » ; c'est
`substitutes()` qui le dit (split du 2026-07-27).

| # | Fichier:ligne | Fonction | Ce qu'il jette | Leve par |
|---|---|---|---|---|
| 1 | `calypso_bsp.c:474` | `bsp_trxd_readable()` | le datagramme TRXD avant enqueue | `RUN_C54X=1 && (route_c54x_active \|\| BSP_DARAM_FORCE)` |
| 2 | `calypso_bsp.c:997` | `calypso_bsp_rx_burst()` | l'ecriture DARAM directe | idem |
| 3 | `calypso_bsp.c:1359` | `calypso_bsp_deliver_buffered()` | **la LIVRAISON vers `data[]`** | `TPU_RX_WIRE` **ou** `RUN_C54X=1 && BSP_DARAM_FORCE` (aligne le 2026-07-28) |

| Var | Defaut code | Valeur correcte mesuree | Note |
|---|---|---|---|
| `CALYPSO_BSP_DARAM_ADDR` | `0x2a00` | **`0x4c00`** | `0x2a00` = workzone de SORTIE, ne jamais y feeder |
| `CALYPSO_BSP_DARAM_LEN` | `296` | `296` | `638` ne valait que pour le 4 SPS non decime |
| `CALYPSO_BSP_IQ_DECIM` | `4` | **`4`** | `=1` est une **REGRESSION** (feed a 4 SPS, `dphi=+0.25 x pi/2`) |

---

## 8. Affirmations INVALIDEES — ne pas les reintroduire

### 8.1 Invalidees par une mesure contraire

| Affirmation | Pourquoi elle est fausse | Signature de l'artefact |
|---|---|---|
| **« `data[0x4c00]` est gele »** | la lecture avait ete faite a `0xFFD08800` via le **monitor QMP**, HORS fenetre API RAM | peak exactement `0x8000` et **54 % de zeros**. Les mesures valides se prennent A L'INTERIEUR (`BSP_LOG` au point d'ecriture, ou `ddump`) |
| **« Q == 0 »** | conclu sur les **2 premiers mots** d'un burst (amplitude faible par construction) | sur le burst entier, `zeros = 0 %` |
| **« `0xa042` detruit le signal avant lecture du noyau »** | `0x2c00` est du **scratch** : il n'y avait pas de signal a detruire | `0x9fd5` y depose une table de coefficients CONSTANTE |
| **« 30+ writers de `d_fb_det` dans la PROM »** | c'etaient des **opcodes**, pas des ecrivains | publisher **UNIQUE** = `ORM #1,*(0x08f8)` @`0x79e4`, banque commune `0x7700-0x79F0`, **jamais executee** |
| **« 0 FCCH sur 200 dumps »** | **artefact de fenetre** : `CALYPSO_DARAM_DUMP` consommait son cap au boot avec `d_fb_mode = 0` | corrige : le dump est desormais filtre sur `d_fb_mode[0x08f9] != 0` (`_ANYMODE=1` restaure l'ancien comportement) |
| **« le detecteur n'est pas arme »** | `d_fb_mode[08f9] = 0x0001` observe | `DETECTOR-RUN` |
| **« le demod lit `data[0x0000]` par pas de 5 »** | vrai **uniquement** avec `CORR_ENTRY=0x9500`, qui saute les 11 mots posant `AR6`. Avec `0x94f5` : `AR6 = 0x4c00` et le demod lit de vraies valeurs | `CALYPSO_WATCH_9F00_RD` |
| **« sample-in natif = `0x9213`/`0x9215` » (en absolu)** | l'adresse **depend du point d'entree** (§3.1). Avec le run de reference, `0x9213`/`0x9260` ne sont **jamais lues** et `FB_STREAM` est inerte | absence de `FB-STREAM addr=` au log |
| **« `BUILD-STAMP` date le binaire »** | il date le TU `calypso_dsp_shunt.c` uniquement | §0 |
| **« 280 detections sur 300 bursts »** | c'est le contenu des 300 premieres **lignes loguees** (cap `calypso_dsp_shunt.c:1670`) | §0 |
| **« le scheduler `0x7234` DERAILLE vers `0x013b` »** | `0x7234: f074 013b` = **CALL inconditionnel en ROM** ; `0x013b` sauvegarde le contexte et **retourne** en `0x7236` | desassemblage PROM |
| **« `0x8341` = la LUT FB a atteindre »** | scan des **4 banks** : **0 reference**. Personne ne peut y sauter. **Ne pas refaire le correctif TPU de `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md`** | scan statique 4 banks |
| **« writer cache du slot `0x43d8` »** | watchpoint `data_write_locked` (voit **tous** les modes d'adressage) : uniquement `0xbb00 -> 0xab38` | watchpoint |
| **« le handler vient d'un patch televerse »** | osmocom `dsp_bootcode.c` : `dsp_bootcode = NULL`, *« it happily works with its own ROM »* ; `dsp.c:205` = `FIXME: Implement Patch download` | source osmocom |

### 8.2 Fausses pistes a NE PAS rechasser

- **SHADOW_WIRE** (`FB_SHADOW_WIRE` casse le retour, rend le natif = 0).
- **Feeder `0x2a00`** (workzone de SORTIE, PAS l'entree).
- `d_fb_det` via reroute **`FB_ENERGY` seul** (le corrélateur reste affame si la chaine
  d'entree n'est pas alignee sur le point d'entree, §3.1).
- **Go-live INTM / storm / seed `5ac8`** (red herring : le handler FB est du **polling
  pur** sur `data[0x3fad]` bit15 ; l'IT n'est pas sur le chemin critique).
- **Cellules background `0x098a` / `0x098c`** (`d_background_enable/state`, ARM=0 par design).
- **Rotation SI** (la racine « no cell info » = `meas->frames == 0`, le mobile ne recoit
  rien en idle — pas les SI).
- `calypso_fbsb.c` (logger mort, `last 0 0 0 0` = red herring).
- `CALYPSO_ARM2DSP_CTRLSYS=1` (casse la recette BGEN).
- **Poke** (falsifier le retour d'une fonction) : insuffisant, le mur est du **wiring**
  inter-blocs.
- **`api_write_cb`** : declare `calypso_c54x.h:204`, jamais assigne (grep = 0) — dead
  callback, ne pas theoriser dessus.
- **Sous-buffer `0x3fb0` / `0x03F0`** comme sample-in natif.
- **`CALYPSO_BSP_IQ_DECIM=1`** (regression mesuree).
- **Tuner `CALYPSO_SHUNT_BURST_OFS`** : la sonde `WP-BURST` a prouve que l'ARM ne commande
  que le burst 0 (22 501 writes, 0 non-nul) — symptome amont.
- **Variables INERTES** (declarees, jamais lues par `getenv`) :
  `CALYPSO_CORRELATOR_TRACE` (le vrai gate est `CALYPSO_DEBUG=CORRELATOR`),
  `CALYPSO_FORCE_3F92`, `CALYPSO_FORCE_0810`, `CALYPSO_FIX_MVDM` (le code ne lit que
  `..._OFF`), `CALYPSO_FORCE_ANGLE_ZERO`.

---

## 9. Regles de sonde (payees quatre fois) et run de reference

### 9.1 Regles

1. **Une sonde se concoit par sa CONDITION DE DECLENCHEMENT, pas par son adresse.** Un
   plafond global est mange par le PC le plus bruyant. Trois precedents dans le code :
   `WZWRITE` filtre par PC producteur (`0x9fd5`/`0x9ab1`) parce que `0xa03d/a042/a079`
   saturaient ; `ERRWATCH` ignore les ecritures de 0 (nettoyage de boot) ; `CORR_FLOW`
   skippe la boucle de copie `0x8866-0x886c` (~134x/appel).
2. **Preferer un AGREGAT a un FLUX plafonne, et prevoir un TEMOIN DE SATURATION.**
   `CALYPSO_WMAP` a un heartbeat (« writes DSP=N, dans plages=0 ») : c'est lui qui
   distingue « pas d'evenement » de « sonde morte ».
3. **Distinguer « varie dans l'ESPACE » (une courbe sur N cellules) de « varie dans le
   TEMPS » (a cellule figee).** Seule la variation temporelle est un signal. `wmap_note`
   v2 ne collecte les valeurs distinctes qu'a adresse figee (`addr0`).
4. **« Pas de log » n'est JAMAIS « pas d'evenement »** tant que la sonde n'est pas verifiee
   VIVANTE et sa fenetre COUVRANTE. Causes rencontrees, toutes reelles ici : plafond
   sature ; seuil de dump trop haut (`DARAM_DUMP` cape au boot avec `d_fb_mode=0`) ; plage
   ecrite cote HOTE (`feed_iq` ecrit `s->data[]` en direct, **invisible depuis
   `data_write_locked`**) ; variable d'env absente du run (les `DMA fn=` sont muets sans
   `CALYPSO_DEBUG=BSP`).
5. **Toute mesure prise via le monitor QMP est hors fenetre API RAM et racy.** Les mesures
   valides se prennent A L'INTERIEUR : `BSP_LOG` au point d'ecriture, ou `ddump`.

### 9.2 Run de reference (chaine d'entree mesuree correcte)

```
CALYPSO_NATIVE_HELPED=1 CALYPSO_FB_CORR_ENTRY=0x94f5 CALYPSO_DSP_RUN_C54X=1 \
CALYPSO_BSP_DARAM_FORCE=1 CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 \
CALYPSO_BSP_IQ_DECIM=4 CALYPSO_SHUNT_REAL_FB=1 CALYPSO_DEBUG=BSP ./start-clean.sh
```

Pour un natif **nu** (sans intercept de lecture ni PM mocke), ajouter en CLI :
`CALYPSO_DECAN=0 CALYPSO_SHUNT_REAL_FB=0 CALYPSO_SHUNT_NO_FAKE_PM=1`.

### 9.3 Ordre d'instrumentation

| # | Question | Instrument | Verdict attendu |
|---|---|---|---|
| 1 | l'entree du demod est-elle vivante, et OU ? | `CALYPSO_WATCH_9F00_RD` | liste les adresses lues -> fixe ou feeder |
| 2 | le feed arrive-t-il, et est-il propre ? | `corr_iq.py --src bursts` | `coh > 0,9`, `dphi ≈ +1,00 x pi/2` |
| 3 | le buffer lu par le detecteur contient-il la FCCH ? | `CALYPSO_DARAM_DUMP` + `corr_iq.py --src ddump` | non-racy, filtre sur `d_fb_mode != 0` |
| 4 | le detecteur est-il arme et tourne-t-il ? | `DETECTOR-RUN` + `CALYPSO_B2AR` | `d_fb_mode=0x0001` ; AR `IN` vs `OOB` |
| 5 | le math produit-il quelque chose ? | `CALYPSO_DEMODIO` (`0x9f95..0x9fe2`), `CALYPSO_B2`, `CALYPSO_CORROUT` | `A=0x80000000` = saturation (bug §4) ; `A/B` a 0 = pas de math |
| 6 | qui ecrit le resultat ? | `CALYPSO_WATCH_RESULT` (`data[]`) + `CALYPSO_FBDET_API` (`api_ram`) | le firmware lit `api_ram[W-0x0800]`, **pas** `data[]` |
| 7 | le flot atteint-il le publisher ? | `CALYPSO_TRACEFROM` | marque `0xa076` / `0x79e4` / `0x9ac0` |

### 9.4 Mesures a faire, dans l'ordre

```bash
# 1) B1 — le correctif AND/OR/XOR/SUBC a-t-il debloque la SORTIE du demod ?
#    Critere = |DC|/rms et dphi du ddump, PAS d_fb_det.
rm -f /dev/shm/daram_2a00.cfile          # AVANT le run
# ... run de reference + CALYPSO_DARAM_DUMP=1 CALYPSO_SHUNT_REAL_FB=0 ...
cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src ddump | tail -3
#    reference a battre : |DC|=2.86e4 pour rms=2.94e4, dphi=+0.004  (sortie morte)

# 2) B2 — nommer qui pose / efface l'etat « DMA en cours »
grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log | sort | uniq -c

# 3) Robustesse LU en SHUNT (departager A5 « 0 retry » vs §9 « 1/19 »)
grep -c T3211 /root/mobile.log ; grep -icE "LOCATION UPDATING ACCEPT" /root/mobile.log

# 4) B7 — eviction SDCCH dans le mode ou elle sert, AVANT toute suppression
grep "EVICT-STATS" /root/qemu.log | tail -1     # en SHUNT_LEGIT=DSP,NO_CANNED
```

---

## 10. References canoniques

### Adresses DSP / ARM

| Cellule | DSP word | ARM addr |
|---|---|---|
| `d_fb_det` | `0x08F8` | `0xFFD001F0` (NDB+0x48) |
| `d_fb_mode` | `0x08F9` | — |
| `a_sync_demod` TOA/PM/ANGLE/SNR | `0x08FA..0x08FD` | NDB+0x4C.. |
| `d_dsp_page` | `0x08D4` | `0xFFD001A8` (BASE_API_NDB) |
| `d_dsp_state` | `0x08E2` | (= 3 IDLE au boot) |
| `d_ctrl_system` (bit15 = `B_TASK_ABORT`) | `0x0810` | — |
| **entree demod** (run de reference) | `data[0x4c00]`, lu en pas de 5 | — |
| entree demod (si `CORR_ENTRY=0x9500`) | `data[0x9213]` / `[0x9215]` | — |
| **workzone OUT demod (NE PAS feeder)** | `0x2a00..0x2b27` (I par `0x9fb8`, Q par `0x9fe2`) | — |
| scratch coefficients (constante) | `0x2c00` (pose par `0x9fd5`) | — |
| noyau MAC energie | `0xa076..0xa09d` (`0x9a80` mode large) | — |
| detecteur | `0x9ac0` (boucle `0x9ab8..0x9ac2`) | — |
| publisher `d_fb_det` (jamais execute) | `0x79e4` (`ORM #1,*(0x08f8)`) | — |
| entree corrélateur referencee en ROM | `0x94f5` (`@0x87e7 f930 94f5`) | — |
| dispatch FB (stub `RET`) | `data[0x43d8] = 0xab38`, `0xab38: fc00` | — |
| verrou-maitre corrélateur | `data[0x3fad]` bit15 (polling `@0x8754`) | — |
| shadow IMR seed | `data[0x435b]` = `0x52ed` | — |
| PROM0 base | `0x7000` | — |

### Task IDs DSP

`PM=1, FB=5, SB=6, RACH=10, NBS=19, NP=21, ALLC=24, CHECKSUM=33`.
(BSIC = bits 2:7 de `a_sch[3] | a_sch[4]<<16`.)

### Gates / mecaniques

- Recalage FN DSP/TRX : `CALYPSO_DL_FN_OFFSET`, **defaut 0 (inerte)** ; valeur qui cale la
  FCCH sur la bonne trame = **-556** (`calypso_trx.c:150-168`). NB : l'offset est trop
  large (il touche aussi la FN UL/DATA_IND) -> **a n'activer que pour l'alignement
  corrélateur**. LOST timer read-driven.
- Vecteurs IT : INT3/frame = vec19 bit3 (`0xFFCC`) ; TINT0 = vec20 bit4 ; BRINT0 = vec21
  bit5 (`0xFFD4`) ; frame-IT native = vec28 bit12.
- Modele gain trf6151 : `a_pm = (target_rf + 71 + gain_trf) * 64` ; reset `REG_RX` `0x9E00`
  = 138.

### Semantique des gates d'environnement (source d'erreur n.1)

| Idiome dans le code | `VAR=0` | `VAR=""` | Desactivation correcte |
|---|---|---|---|
| `getenv(X) ? 1 : 0` | **ON** | **ON** | `unset X` |
| `(e && atoi(e) > 0)` / `(e && *e == '1')` | OFF | OFF | `X=0` |
| `(!e \|\| *e != '0')` — **defaut ON** | **OFF** | ON | `X=0` |
| `getenv(X_OFF) ? 0 : 1` — defaut ON | s.o. | s.o. | poser `X_OFF=1` |

### Runtime

- Tree LIVE = **`/opt/GSM/qemu-src`** (`.latest.bak`/`.bak` = anciens ; overlay
  `qemu-calypso` = **MORT au runtime**, ne pas le patcher).
- Firmware = `/opt/GSM/osmocom-bb-transceiver`.
- Lancement : `osmo_egprs/start-direct.sh` -> `qemu-src/start-clean.sh` (source
  `calypso.env` en `set -a`) -> `run.sh`.
- `run.sh:1151-1155` (mode `full-grgsm`) utilise `=` et non `:=` : `SHUNT_NO_CANNED=1`,
  `DSP_L1STUB=0`, `DSP_L1_STUB=0`, `FORCE_FBSB=0`, `FORCE_AGCH=0` **ne sont pas
  surchargeables en CLI** dans ce mode.
- **L'utilisateur relance la pile lui-meme.** Claude = edits + diagnostic lecture seule.

---

## 11. References durables — ou elles sont REELLEMENT

Ces documents ne decrivent PLUS l'etat courant (leurs narratifs dynamiques sont perimes),
mais restent des references ISA / HW durables. **Chemins verifies le 2026-07-28**
(`ls hw/arm/calypso/doc/ hw/arm/calypso/doc/opcodes hw/arm/calypso/doc/project`) : contrairement
a ce qu'annoncait la version precedente de cette section, **un seul** d'entre eux vit dans
`doc/archive/`. Le `README.md` de `doc/` les indexe deja aux bons chemins.

| Chemin reel | Contenu de reference |
|---|---|
| `doc/opcodes/tic54x_hi8_map.md` | Map hi8 -> mnemonic complet (c'est lui qui donne `0x18..0x19 = and`, cf. S4, ligne 26) ; layout ST1 ; indices MMR |
| `doc/opcodes/0x68_0x6F.md` | ANDM/ORM/XORM/ADDM/BANZ/BANZD/MAR ; famille `0x6F00` ; adressage Smem/SHIFT |
| `doc/opcodes/0xF3.md` | Famille F3xx (INTR k, AND/OR/XOR/SFTL 1-mot, `#lk` 2-mots) ; SFTL vs SFTA |
| `doc/project/STEP2_BC_CONDS.md` | Codes condition BC group-1 (CCEQ/CCNEQ/CCLT/... markers) |
| `doc/C54X_INSTRUCTIONS.md` | XC/FRET/FCALL/RETE/BANZ/BC encodings (SPRU172C) |
| `doc/project/AUDIT_DECODER_20260508.md` | Verites binutils tic54x (STL/STH/CMPS/BIT/MVPD...) ; Tier A lande |
| `doc/DSP_ROM_MAP.md` | Sections ROM (PROM0 `0x7000`, DROM/PDROM) ; API RAM `0x0800`+ |
| `doc/project/BUGS_AND_FIXES.md` | F272=RPTBD, F274=CALLD, `0x8A00`=POPM, SP init `0x5AC8`, RET stubs |
| `doc/CALYPSO_HW.md` | Datasheet HW/protocole (API RAM, TPU_CTRL, TRXDv0, sercomm, L1CTL) |
| `doc/SERCOMM_GATE_ARCHITECTURE.md` | Chemins BSP/L1CTL, HDLC/DLCI, task IDs, `d_dsp_page@0x08D4` |
| `doc/hardware-map.md` | Memory map (Flash/IRAM/XRAM), bases peripheriques, IRQ map, INTH |
| `doc/datasheets/TI_SPRU172C_C54x_Mnemonic_Instruction_Set.pdf` | SPRU172C — arbitrage final sur une semantique d'instruction |
| `doc/archive/SESSION_20260405_NIGHT4.md` | Encodings F0xx/RSBX/SSBX/RET/RETD/CALLD/IDLE ; vecteurs IT — **seul de cette liste reellement dans `archive/`** |

`doc/archive/` contient par ailleurs l'historique date (sessions, pistes closes) : y chercher
un contexte, jamais un statut.


---

## ⚠️ `CALYPSO_NATIVE_HELPED=1` n'est PAS le mode natif (mesure 2026-07-28)

C'est un **paquet de béquilles**. Le manifeste du run le montre : poser `NATIVE_HELPED=1`
**repose automatiquement** le reroute du corrélateur et l'injection d'IQ :

```
[calypso-manifest] CALYPSO_FB_CORR_ENTRY=0x9500     <- reroute REPOSÉ (valeur par défaut)
[calypso-manifest] CALYPSO_FB_ENERGY=1              <- imposé
[calypso-manifest] CALYPSO_FB_IQ_DARAM=1
[calypso-manifest] CALYPSO_FB_IQ_BASE=0x9210
```

**Conséquence pratique, vérifiée à nos dépens** : retirer `CALYPSO_FB_CORR_ENTRY=0x94f5` de la
ligne de commande ne supprime pas le reroute — il revient simplement à `0x9500`. Un run qui
garde `NATIVE_HELPED` ne teste donc **jamais** le chemin natif ; il compare deux béquilles.

Pour tester réellement le natif, tout enlever :
```bash
CALYPSO_DSP_RUN_C54X=1 CALYPSO_BSP_DARAM_FORCE=1 \
CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 CALYPSO_BSP_IQ_DECIM=4 \
CALYPSO_DARAM_DUMP=1 CALYPSO_WATCH_9F00_RD=1 ./start-clean.sh
```
et **contrôler le manifeste AVANT de lire quoi que ce soit** :
```bash
grep -E "calypso-manifest.*(CORR_ENTRY|FB_ENERGY|REAL_FB|NATIVE_HELPED|FB_IQ)" /root/qemu.log
```
Règle générale : **lire le manifeste, jamais la ligne de commande**. La ligne de commande dit
ce qu'on a demandé ; le manifeste dit ce qui s'applique.

## Sources d'autorité pour les opcodes C54x (posées le 2026-07-28)

1. **`doc/opcodes/tic54x-opc.c`** — la table binutils, désormais **copiée dans le dépôt**.
   Format : `{ "mnémo", MOTS, cycles, classe, OPCODE, MASQUE, {opérandes}, flags }`.
   Le champ **MOTS** fait foi : une longueur fausse ne donne pas un résultat faux, elle
   **désynchronise tout le décodage en aval**.
2. **`doc/spru172c.pdf`** — manuel TI, autorité pour la **sémantique** d'exécution.
   (Pas d'extracteur dans le conteneur : le copier dehors et décompresser les flux avec zlib.
   Les tableaux d'encodage perdent leur mise en page à l'extraction — d'où la primauté de
   binutils sur l'encodage.)
3. le code, puis les tableaux de synthèse.

**Deux erreurs de nos propres tables, corrigées le 2026-07-28** (chacune a coûté une fausse
piste) :

| Ce qui était écrit | Ce que dit binutils | Où |
|---|---|---|
| `0xF4..0xF7` = « add (**2-mot**) » | `{ "add", **1**,1,3, 0xF400, 0xFCE0 }` = **1 mot**, registre-registre. Les formes à long immédiat (2 mots) sont en `0xF0..0xF3` (`0xF000/0xFCF0`). | `opcodes/tic54x_hi8_map.md` |
| `0xEA` = « BANZ (confirmed) » | `{ "ld", 1,2,2, **0xEA00**, 0xFE00, {OP_k9,OP_DP} }` = **`LD #k9, DP`**, chargement du Data Page pointer. `banz` est en `0x6C00`, `banzd` en `0x6E00`, 2 mots. | `C54X_INSTRUCTIONS.md` |

Dans les deux cas **le code de `calypso_c54x.c` était correct** et c'est la doc qui égarait.
Corollaire de méthode : **ne jamais conclure depuis un commentaire de code** — plusieurs se
sont avérés périmés, dont un `[TODO]` sur `STL/STH … ASM` alors que `asm_shift()` est bien
appliqué.
