# QUICK START — QEMU-Calypso

Lancer et **vérifier**. Ce fichier ne décrit que ce qui est mesuré aujourd'hui
(2026-07-28). Chaque affirmation nomme son instrument. Vérité de fond :
[`hw/arm/calypso/doc/ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md).

Distinction employée partout : **MESURE** (relevé, avec sa commande) /
**HYPOTHÈSE** (déduite du code, pas encore relevée) / **INVALIDE** (affirmation
retirée, ne pas réintroduire).

---

## 1. Prérequis

1. Tout tourne **dans le conteneur `osmo-operator-1`** ; on y entre par
   `docker exec -it osmo-operator-1 bash`. Depuis l'hôte, tout accès prend la
   forme `docker exec osmo-operator-1 bash -lc '...'`.
2. Le **runtime est `/opt/GSM/qemu-src`** (build + `calypso.env` + `run.sh` +
   `start-clean.sh`). `/opt/GSM/qemu-calypso` est un **overlay mort au
   runtime** : n'y écrire jamais (§5).
3. Build : `ninja -C build qemu-system-arm` puis `./make-overlay.sh` (back-port
   du working tree vers l'overlay git ; ne change rien au runtime).

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src &&
  ninja -C build qemu-system-arm &&
  ./make-overlay.sh
'
```

Vérifier **quel binaire tourne** — `BUILD-STAMP` **ment** : la macro
`__DATE__/__TIME__` vit dans `calypso_dsp_shunt.c:107` et date **son propre**
translation-unit, pas celui qu'on vient de modifier. Instrument correct = mtime
du `.o` de l'unité modifiée + `lstart` du process :

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src &&
  stat -c "%y %n" build/qemu-system-arm &&
  find build -name "*calypso_c54x*.o" -printf "%T+ %p\n" &&
  ps -eo pid,lstart,cmd | grep [q]emu-system-arm
'
```

---

## 2. Le mode qui MARCHE : `SHUNT_LEGIT`

C'est le mode fiable : il campe, fait la Location Update et les SMS. Le FBSB y
est produit **côté hôte** (détecteur FCCH cohérence+dφ + gr-gsm) et présenté à
l'ARM par intercept de lecture (`calypso_dsp_shunt.c:1463-1490`, appelé depuis
`calypso_trx.c:297`).

### Lancer

```bash
docker exec -it osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src && CALYPSO_SHUNT_LEGIT=1 ./start-clean.sh
'
```

Variantes utiles (mêmes vérifications) :

| But | Commande |
|---|---|
| Injections nommées une par une, sans le parapluie | `CALYPSO_SHUNT_NO_LEGIT=1 ./start-clean.sh` |
| Camp **et** c54x qui tourne en parallèle (plus réaliste, plus flaky) | `CALYPSO_SHUNT_LEGIT=DSP,NO_CANNED ./start-clean.sh` |

`CALYPSO_SHUNT_LEGIT` est une **value-list résolue dans QEMU** par un
constructeur exécuté avant `main()` (`calypso_dsp_shunt.c:86-105`) : `DSP` pose
`CALYPSO_DSP_RUN_C54X=1` avec `overwrite=1`. Conséquence : `env | grep
RUN_C54X` côté shell **ment**. L'instrument est le manifeste imprimé par ce même
constructeur, ou l'environnement réel du process :

```bash
docker exec osmo-operator-1 bash -lc 'grep -a "calypso-manifest]" /root/qemu.log | head -40'
docker exec osmo-operator-1 bash -lc 'tr "\0" "\n" < /proc/$(pgrep -f qemu-system-arm | head -1)/environ | grep ^CALYPSO_'
```

### Les 4 vérifications

```bash
# V1 — le FBSB hôte détecte (coh proche de 1, det=1)
docker exec osmo-operator-1 bash -lc 'grep -a "REAL-FB" /root/qemu.log | tail -3'

# V2 — le mobile campe : SI décodés + BSIC réel (7), pas BSIC=0
docker exec osmo-operator-1 bash -lc '
  grep -ac sysinfo /root/mobile.log ;
  grep -aoE "BSIC=[0-9]+" /root/mobile.log | sort | uniq -c ;
  grep -a "MON: f=" /root/mobile.log | tail -2'

# V3 — Location Update acceptée + TMSI attribué
docker exec osmo-operator-1 bash -lc '
  grep -aicE "LOCATION UPDATING ACCEPT" /root/mobile.log ;
  grep -aiE "TMSI|TMSI REALLOC" /root/mobile.log | tail -5 ;
  grep -ac T3211 /root/mobile.log'

# V4 — SMS
docker exec osmo-operator-1 bash -lc 'grep -aiE "sms|SMSC" /root/smsc-op1.log | tail -10'
```

| # | Ce qu'on doit voir | Valeur de référence **mesurée** | Instrument |
|---|---|---|---|
| V1 | `REAL-FB fn=… coh=0.999 dphi=0.387 det=1 SNR=0x735b AFC=-186` | 280 lignes `det=1` sur les **300 premières lignes loguées** | `grep "REAL-FB" /root/qemu.log` |
| V2 | `sysinfo` non nul, `BSIC=7`, `MON: f=… -47 dBm` | 20 SI décodés ; BSIC **7** (le vrai) ; rxlev −47/−56 dBm | `/root/mobile.log` |
| V3 | `LOCATION UPDATING ACCEPT` ≥ 1, `lai=001-01-1`, TMSI `0x3dbeb85f`, `TMSI REALLOC COMPLETE` | RACH→ACCEPT en **2,70 s**, 0 retry T3211 (run A5) | `/root/mobile.log` |
| V4 | MO et MT délivrés | DONE en `SHUNT_LEGIT` et `SHUNT_NO_LEGIT` ; **flaky** en `DSP,NO_CANNED` | `/root/smsc-op1.log` |

**Piège de comptage sur V1.** Le logger `REAL-FB` est plafonné
(`calypso_dsp_shunt.c:1670` : `rfl < 20 || (det && rfl < 300)`). « 280/300 »
est le contenu des 300 premières **lignes loguées**, pas un taux de détection
sur tout le run. Ne pas l'écrire autrement.

**Réserve ouverte sur V3.** `doc/SHUNT_LEGIT_ADDRESS_MAP.md` §9 (26/07,
antérieur au fix sous-voie SDCCH/8) mesure « LU ACCEPT intermittent, ~1 succès
pour 19 retries T3211 », alors que `run_results.md` run A5 mesure « 2,70 s, 0
retry » (n=2). **Non départagé.** Pour trancher : rejouer 5 runs `SHUNT_LEGIT`
consécutifs et relever `grep -c T3211 /root/mobile.log`.

**Oracle réseau.** Le cœur Osmocom est prouvé bon indépendamment de QEMU : la
pile témoin `bts1` (mobile osmocom-bb sur `trxcon` + `fake_trx`) obtient un LU
ACCEPT sur le même cœur — `grep -c "LOCATION UPDATING ACCEPT"
/root/mobile-bts1.log` = 1. Tout échec côté Calypso est donc imputable à
l'émulation.

---

## 3. Le mode NATIF, en cours d'investigation

Objectif : que ce soit le **DSP c54x** qui produise `d_fb_det`, au lieu du
détecteur hôte. **Ce mode ne campe pas** aujourd'hui.

### Run de référence (chaîne d'entrée mesurée correcte)

```bash
docker exec -it osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src && rm -f /dev/shm/daram_2a00.cfile /dev/shm/bursts.cfile &&
  CALYPSO_NATIVE_HELPED=1 CALYPSO_FB_CORR_ENTRY=0x94f5 CALYPSO_DSP_RUN_C54X=1 \
  CALYPSO_BSP_DARAM_FORCE=1 CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 \
  CALYPSO_BSP_IQ_DECIM=4 CALYPSO_SHUNT_REAL_FB=1 CALYPSO_DEBUG=BSP ./start-clean.sh
'
```

Deux réglages de ce run à connaître avant d'interpréter quoi que ce soit :

- `CALYPSO_SHUNT_REAL_FB=1` **masque le natif à l'observateur ARM** : l'intercept
  de lecture sert le `d_fb_det` **hôte** sur l'offset `0x01F0`. La seule cellule
  qui mesure le natif est `data[0x08f8]`, imprimée par la ligne `DETECTOR-RUN`.
  Pour un natif nu : `CALYPSO_SHUNT_REAL_FB=0 CALYPSO_DECAN=0`.
- `CALYPSO_DEBUG=BSP` est **obligatoire** pour que les sondes BSP parlent. Sans
  lui, `DMA fn=` / `BURST fn=` sont absents **parce que la sonde est muette**,
  pas parce que le BSP est inerte (§5).

### Vérifications, dans l'ordre

```bash
# N1 — les 3 gates BSP sont levées, aucun burst jeté
docker exec osmo-operator-1 bash -lc '
  grep -ac "deliver: gate shunt LEVE" /root/qemu.log ;
  grep -ac "dropping fn=" /root/qemu.log ;
  grep -a "DMA fn=" /root/qemu.log | tail -2'

# N2 — ce qui est déposé en DARAM est bien de la FCCH à 1 SPS
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src bursts | tail -8'

# N3 — ce que le détecteur LIT réellement (dump interne, non-racy)
#      exige CALYPSO_DARAM_DUMP=1 dans le run
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src ddump | tail -6 ;
  grep -a "DARAM-SANITY" /root/qemu.log | tail -3'

# N4 — le résultat natif
docker exec osmo-operator-1 bash -lc '
  grep -a "DETECTOR-RUN" /root/qemu.log | tail -2 ;
  grep -a "DETECTOR-RUN" /root/qemu.log | grep -vc "d_fb_det\[08f8\]=0x0000"'
```

| # | Question | ATTENDU | ACTUEL (mesuré) |
|---|---|---|---|
| N1 | les bursts atteignent-ils `data[]` ? | `deliver: gate shunt LEVE (rxw=1)` présent, `dropping fn=` = **0** | **conforme** : gate levée, 0 drop. *(Les 3 gates sont `calypso_bsp.c:474`, `:997`, et `:1359` = la LIVRAISON, alignée le 28/07 sur `DARAM_FORCE` ; auparavant elle ne connaissait que `TPU_RX_WIRE`, d'où 2 verrous ouverts sur 3 et « rien n'arrive ».)* |
| N2 | le feed est-il conforme ? | `VERDICT: FCCH @1SPS PROPRE (dphi=+1.00x pi/2)` | **conforme** : `coh=0.998`, `rms=3.25e4`, `\|DC\|=379`, `zeros=0%`, FFT `+67 708 Hz` |
| N3 | la SORTIE du démod est-elle exploitable ? | `coh > 0.90`, `dphi ≈ +1.571` | **KO** : DC quasi pur et **figé** — `\|DC\|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` ; cellule témoin invariante sur 157–203 bursts (`0x9fb8@0x2a00=0x0000`, `0x9fe2@0x2a00=0x52ed`). Identique avec `DECIM=1` **et** `DECIM=4` |
| N4 | `d_fb_det` passe-t-il à 1 ? | ≥ 1 ligne `DETECTOR-RUN` avec `d_fb_det[08f8]` ≠ `0x0000` | **KO** : `0` sur 3 600 exécutions (run 44 s) et sur 32 200 (run 437 s). Le détecteur **est armé** : `d_fb_mode[08f9]=0x0001` |

Autrement dit : **entrée vivante, sortie morte**. C'est N3 (et non N4) qui est le
critère de tranche — `d_fb_det` est trop en aval pour arbitrer un correctif.

### État des pistes

| Piste | Statut |
|---|---|
| Décodage d'opcode `0x1800/1A00/1C00/1E00` (`AND/OR/XOR/SUBC`) exécuté comme un `LD` → `T=31` → `LD Smem,TS` décale de 31 → `A=0x80000000` saturé → sortie indépendante des opérandes | **corrigé en source** (`calypso_c54x.c:9365-9389`), **non validé au run** : premier run post-correctif → `d_fb_det` toujours 0. Re-mesurer par N3, pas par N4 |
| `DSP Error Status: 32` (`DMA_PEND`) permanent — 723 occurrences | ouvert. **HYPOTHÈSE** : le BSP écrit `data[]` en direct sans passer par la machinerie DMA, le drapeau n'est jamais effacé. `grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log \| sort \| uniq -c` |
| Le démod lit en **stride 5** (`0x4c00/05/0a/0f/15/1a`, polyphase 6 taps) alors que le BSP dépose 296 int16 contigus | ouvert, non tranché : le stride peut être correct et le **layout de remplissage** faux |
| Même avec `d_fb_det=1`, le natif ne camperait pas : ni SCH ni SI (`dispatch_allc`=0, `feed_agch`=0, `sb_valid`=0) | connu. Ordre du plan : FB → SCH → SI |

---

## 4. Boîte à outils de diagnostic

### 4.1 Sondes (toutes gatées par variable d'environnement, **défaut OFF**)

Trois familles seulement dans ce tableau : **M** = mesure pure (lecture seule,
le run est identique sans elle) ; **W** = wire (écrit une donnée que le matériel
produit mais que l'émulation ne propageait pas) ; **B** = béquille (falsifie un
état — invalide toute conclusion en aval).

| Variable | T | Ce qu'elle donne |
|---|---|---|
| `CALYPSO_DEBUG=BSP` | M | logs du BSP : `DMA fn=`, `DARAM after write`, `RX tn= fn= delta=`. **Sans elle, les sondes BSP sont muettes** |
| `CALYPSO_WATCH_9F00_RD` | M | adresses **lues** par l'étage démod (PC `0x9f00..0x9fb8`). **À lancer avant tout feed** : c'est elle qui dit où feeder |
| `CALYPSO_RMAP` (+`_PCLO`/`_PCHI`) | M | carte **agrégée** des adresses lues par une plage de PC |
| `CALYPSO_WMAP` (+`_LO`/`_HI`/`_LO2`/`_HI2`) | M | carte **agrégée** des **écrivains** d'une plage `data[]`, avec **heartbeat** (témoin de saturation) |
| `CALYPSO_DEMODIO` (+`_AFTER`/`_PCLO`/`_PCHI`) | M | corrèle lectures/écritures + `A`/`B`/`T`/`AR` sur une fenêtre de PC. C'est elle qui a exposé `T=31` |
| `CALYPSO_DARAM_DUMP` | M | dump binaire **interne, non-racy** du buffer lu par le détecteur → `/dev/shm/daram_2a00.cfile`, filtré sur `d_fb_mode≠0` |
| `CALYPSO_B2IN` | M | énergie du ring `FB_STREAM` (max\|I\|, max\|Q\|, fenêtre 296) |
| `CALYPSO_B2` / `_B2SEQ` / `_B2AR` | M | à `0x9ac0` : \|A\|/\|B\| ; 16 paires (I,Q) ; `AR2..AR5` avec verdict `IN`/`OOB` |
| `CALYPSO_WATCH_RESULT` | M | écritures `0x08F8..0x08FD` nommées (`d_fb_det`, `d_fb_mode`, TOA, PM, ANGLE, SNR) |
| `CALYPSO_FBDET_API` | M | le résultat FB au **format natif** `api_ram[0xF8..0xFD]` — c'est ce que lit le firmware, **pas** `data[]` |
| `CALYPSO_TRACEFROM` (+`_N`) | M | dump d'opcodes + trace de flux depuis un PC ; marque `0xa076` (noyau MAC), `0x79e4` (publisher `d_fb_det`), `0x9ac0` (détecteur) |
| `CALYPSO_ORPHAN` | M | shadow-stack : appariement push/pop, **nomme** le retour orphelin |
| `CALYPSO_BSP_DARAM_FORCE` | W | leve les 3 gates BSP (`calypso_bsp.c:474`, `:997`, `:1359`) — les trois testent **aussi** `CALYPSO_DSP_RUN_C54X=1` (`:470`, `:993`, `:1352`), poser les deux. **Obligatoire** pour nourrir le correlateur natif |
| `CALYPSO_ARM2DSP_BGEN` | W | l'ARM pose `d_background_enable/_state` → sortie de la wait-loop DSP |
| `CALYPSO_FB_ENERGY` / `_FB_CORR_ENTRY` | B | reroute la `CALA @0xb01e` vers le corrélateur énergie. `FB_ENERGY=0` = **chemin natif pur** (test décisif) |
| `CALYPSO_FB_STREAM` / `CALYPSO_FB_IQ_DARAM` | B | deux façons de feeder le démod (intercept de lecture / écriture DARAM directe) |
| `CALYPSO_SHUNT_REAL_FB`, `CALYPSO_DECAN`, `CALYPSO_INJECT_*`, `CALYPSO_FORCE_*` | B | injections. Toute conclusion FB tirée avec l'une d'elles est nulle |

**Sémantique des gates** — c'est la source d'erreur n°1 quand on désactive une
variable :

| Idiome dans le code | `VAR=0` | Désactivation correcte |
|---|---|---|
| `getenv(X) ? 1 : 0` (majorité des sondes) | **reste ON** | `unset X` |
| `atoi(e) > 0` / `*e == '1'` | OFF | `X=0` |
| `!e \|\| *e != '0'` (défaut **ON**) | OFF | `X=0` |
| `getenv(X_OFF) ? 0 : 1` (défaut ON) | sans effet | poser `X_OFF=1` |

### 4.2 `tools/corr_iq.py` — l'instrument de référence de la chaîne I/Q

Métrique : `coh = |Σ iq[k+1]·conj(iq[k])| / Σ|iq[k+1]||iq[k]|` (1.0 = ton pur
FCCH, ~0 = bruit ou GMSK) et `dphi` exprimé **en unités de π/2**.

```bash
docker exec osmo-operator-1 bash -lc 'cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src bursts'
docker exec osmo-operator-1 bash -lc 'cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src ddump'
docker exec osmo-operator-1 bash -lc 'cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src shunt'
docker exec osmo-operator-1 bash -lc 'cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src all'
```

| `--src` | Fichier | Ce que ça mesure | Confiance |
|---|---|---|---|
| `shunt` | `/dev/shm/dsp_iq.cfile` (fc32) | I/Q d'entrée du shunt — **référence propre** amont | fiable |
| `bursts` | `/dev/shm/bursts.cfile` (IQ16, `BSP_DUMP_RX_FILE`) | ce que le BSP **dépose** en DARAM, avec `fn`/`tn` | fiable |
| `rxdump` | `/tmp/iq_rx_*.bin` (`CALYPSO_IQDUMP`) | idem, en fichiers séparés par burst | fiable |
| `ddump` | `/dev/shm/daram_2a00.cfile` (`CALYPSO_DARAM_DUMP`) | **le même buffer, dumpé de l'intérieur au moment où le détecteur le lit** — atomique | **la mesure de la destination** |
| `daram` | `0x2a00` via le monitor QMP | best-effort | **racy et hors fenêtre API RAM — à éviter** |

| `dphi / (π/2)` | Interprétation |
|---|---|
| `+1.00` | FCCH @1 SPS — ce que le corrélateur attend |
| `+0.25` | FCCH @4 SPS non décimé → décimer ÷4 (`CALYPSO_BSP_IQ_DECIM=4`) |
| négatif | miroir spectral → `CALYPSO_DL_IQ_CONJ=1` |

**Le piège des lectures hors fenêtre API RAM.** Toute mesure prise par le
monitor QMP (`--src daram`, lecture d'une adresse physique) tombe **hors** de la
fenêtre API RAM et est **racy**. C'est cette voie qui a produit l'affirmation
**INVALIDE** « `0x4c00` est gelé » : la lecture avait été faite à
`0xFFD08800`. Sa signature est reconnaissable — **peak exactement `0x8000` et
54 % de zéros**. Les mesures valides se prennent **à l'intérieur** : `BSP_LOG`
au point d'écriture, ou le dump interne `ddump`.

**Hygiène de fichier.** `/dev/shm/*.cfile` survit aux runs. Comparer
systématiquement leur mtime au `lstart` du process avant d'interpréter, et les
supprimer **avant** le run :

```bash
docker exec osmo-operator-1 bash -lc '
  ls -l --time-style=+%H:%M:%S /dev/shm/*.cfile ;
  ps -eo lstart,cmd | grep [q]emu-system-arm'
```

---

## 5. Les pièges connus

1. **`CALYPSO_BSP_IQ_DECIM=1` est une RÉGRESSION.** Le feed part alors à 4 SPS
   (`dphi = +0.25×π/2`). La valeur correcte est **4** ; `corr_iq.py --src
   bursts` doit répondre `VERDICT: FCCH @1SPS PROPRE`. Corollaire :
   `CALYPSO_BSP_DARAM_LEN=296` (638 ne valait que pour le 4 SPS non décimé).
2. **Ne jamais feeder `data[0x2a00]`.** C'est la **workzone de SORTIE** du
   démod, écrite par le DSP lui-même (PC `0x9fb8` pour I, `0x9fe2` pour Q),
   mesuré par `CALYPSO_WMAP`. Y écrire n'alimente rien et détruit la mesure.
3. **L'adresse d'entrée du démod n'est pas une constante : elle dépend du point
   d'entrée du corrélateur.** Toujours mesurer avec `CALYPSO_WATCH_9F00_RD`
   **avant** de feeder.

   | Configuration | Le démod LIT | Outil qui sert | Outil INERTE |
   |---|---|---|---|
   | `FB_ENERGY=1` + `FB_CORR_ENTRY=0x9500` | `data[0x9213]`/`[0x9215]` | `CALYPSO_FB_STREAM` | `BSP_DARAM_ADDR` |
   | `FB_CORR_ENTRY=0x94f5` + `BSP_DARAM_ADDR=0x4c00` | `data[0x4c00]` (stride 5) | BSP + `DARAM_FORCE` | `FB_STREAM` |

   `0x9500` n'apparaît **nulle part** dans les 28 672 mots de PROM (instrument : scan
   statique des 4 banks, sonde `CALYPSO_SCANREF=0x9500`) ; `0x94f5`
   est l'entrée référencée en ROM (`@0x87e7 f930 94f5`) et le défaut du code.
   Les deux `.env` natifs livrés posent encore `0x9500` : le run de référence le
   corrige en CLI (§3).
4. **L'overlay ne sert à rien au runtime.** Le runtime est
   `/opt/GSM/qemu-src` **uniquement** ; `/opt/GSM/qemu-calypso` est un overlay
   git alimenté par `./make-overlay.sh` **après** coup. Patcher l'overlay n'a
   aucun effet sur ce qui tourne. Les répertoires `bak`/`bak2` sont de vieilles
   sources.
5. **« Pas de log » n'est jamais « pas d'événement »** tant que la sonde n'est
   pas vérifiée VIVANTE et sa fenêtre COUVRANTE. Causes déjà rencontrées, toutes
   présentes dans ce code :
   - plafond saturé (le PC le plus bruyant mange le cap global) ;
   - seuil de dump trop haut — `DARAM_DUMP` capé au boot avec `d_fb_mode=0`, ce
     qui a produit le faux « 0 FCCH sur 200 dumps » (artefact de fenêtre, pas
     une absence de FCCH) ;
   - plage écrite **côté hôte** — `feed_iq` écrit `s->data[]` en direct, donc
     **invisible depuis `data_write_locked`** ;
   - variable absente du run (`CALYPSO_DEBUG` sans `BSP` → `DMA fn=` muet) ;
   - variable **inerte** : `CALYPSO_CORRELATOR_TRACE`, `CALYPSO_FORCE_3F92`,
     `CALYPSO_FORCE_0810`, `CALYPSO_FIX_MVDM` n'ont **aucun `getenv`** dans le
     code — le vrai gate est ailleurs (`CALYPSO_DEBUG=CORRELATOR`,
     `CALYPSO_FIX_MVDM_OFF`, …).

### Corollaires de méthode

- Une sonde se conçoit par sa **condition de déclenchement**, pas par son
  adresse.
- Préférer un **agrégat** (compte tout le run, imprime un tableau, avec témoin
  de saturation) à un flux plafonné.
- Distinguer « varie dans **l'espace** » (une courbe sur N cellules) de « varie
  dans le **temps** » (à cellule figée) : seule la variation temporelle est un
  signal.
- Le résultat FB natif se lit dans **`api_ram[0xF8..0xFD]`**
  (`CALYPSO_FBDET_API`), pas dans `data[]` : c'est cette cellule que le firmware
  lit.

### Affirmations INVALIDÉES — ne pas réintroduire

| Affirmation retirée | Pourquoi |
|---|---|
| « `0x4c00` est gelé » | lecture à `0xFFD08800` via le monitor QMP, **hors** fenêtre API RAM. Signature : peak `0x8000` + 54 % de zéros |
| « `Q == 0` » | conclu sur les 2 premiers mots d'un burst (amplitude faible par construction) ; sur le burst entier, `zeros=0%` |
| « `0xa042` détruit le signal avant lecture du noyau » | `0x2c00` est du **scratch** : il n'y avait pas de signal à détruire. `0x9fd5` y dépose une table de coefficients **constante** |
| « 0 FCCH sur 200 dumps » | artefact de fenêtre : `DARAM_DUMP` capé au boot avec `d_fb_mode=0` |
| « `BUILD-STAMP` indique la fraîcheur du binaire » | il date le TU `calypso_dsp_shunt.c`, pas celui qu'on a modifié (§1) |

---

## 6. Ne pas faire

- Ne pas écrire dans `/opt/GSM/qemu-calypso` (overlay mort au runtime).
- Ne pas modifier un **défaut** de configuration pour faire passer un test : les
  overrides se posent **en CLI** (l'idiome `: "${X:=…}"` du projet garantit que
  la CLI gagne sur le profil, qui gagne sur `calypso.env`).
- Ne pas tirer de conclusion sur le natif avec `CALYPSO_SHUNT_REAL_FB=1` ou
  `CALYPSO_DECAN=1` : la valeur `d_fb_det` vue par l'ARM est alors celle du
  détecteur **hôte**.
- Ne pas lancer `run.sh` directement : `start-clean.sh` source `calypso.env`
  **avant** `exec ./run.sh`, et c'est cet ordre qui donne la bonne précédence
  (`CALYPSO_DSP_SHUNT=0` en natif malgré le preset `full-grgsm`).


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
