# QUICK START — QEMU-Calypso

Lancer et **vérifier**. Ce fichier ne décrit que ce qui est mesuré aujourd'hui
(2026-07-30). Chaque affirmation nomme son instrument. Vérité de fond :
[`hw/arm/calypso/doc/ETAT_ACTUEL.md`](hw/arm/calypso/doc/ETAT_ACTUEL.md).

Distinction employée partout : **MESURE** (relevé, avec sa commande) /
**HYPOTHÈSE** (déduite du code, pas encore relevée) / **INVALIDE** (affirmation
retirée, ne pas réintroduire).

## Les profils, en une ligne chacun

`CALYPSO_MODE=…` — un profil = **qui fait quoi**, et rien d'autre :

| profil | FB / SB | SI | à quoi il sert |
|---|---|---|---|
| `shunt_legit` | hôte | gr-gsm | la pile de bout en bout : camp, LU, SMS (§2) |
| `native_twl` | hôte / TWL | **DSP** | **le DSP traite-t-il le SI ?** (§3bis) |
| `native` | DSP | DSP | la vérité sur l'acquisition (§3) |
| `native_helped` | DSP, entrée reroutée | DSP | observer le corrélateur — **sous béquille** |
| `empty` | rien de posé | rien de posé | construire un banc gate par gate |

Un profil ne pose que des `:=` : **la CLI garde toujours le dernier mot**, et
c'est le **manifeste** qui dit ce que vous avez réellement obtenu (§5).

---

## 1. Prérequis

1. Tout tourne **dans le conteneur `osmo-operator-1`** ; on y entre par
   `docker exec -it osmo-operator-1 bash`. Depuis l'hôte, tout accès prend la
   forme `docker exec osmo-operator-1 bash -lc '...'`.
2. Le **runtime est `${QEMU_TREE}`** (build + `calypso.env` + `run.sh` +
   `start-clean.sh`). `${GSM_ROOT}/qemu-calypso` est un **overlay mort au
   runtime** : n'y écrire jamais (§5).
3. Build : `ninja -C build qemu-system-arm` puis `./make-overlay.sh` (back-port
   du working tree vers l'overlay git ; ne change rien au runtime).

```bash
docker exec osmo-operator-1 bash -lc '
  cd ${QEMU_TREE} &&
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
  cd ${QEMU_TREE} &&
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
  cd ${QEMU_TREE} && CALYPSO_SHUNT_LEGIT=1 ./start-clean.sh
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
  cd ${QEMU_TREE} && rm -f /dev/shm/daram_2a00.cfile /dev/shm/bursts.cfile &&
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
  cd ${QEMU_TREE}/tools && python3 corr_iq.py --src bursts | tail -8'

# N3 — ce que le détecteur LIT réellement (dump interne, non-racy)
#      exige CALYPSO_DARAM_DUMP=1 dans le run
docker exec osmo-operator-1 bash -lc '
  cd ${QEMU_TREE}/tools && python3 corr_iq.py --src ddump | tail -6 ;
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

## 3bis. `native_twl` — la question du SI, sans attendre le FB/SB

Le FB/SB natif n'arrive pas (§3, critère N4 = 0 sur tous les runs depuis le
28/07). Ce profil retourne le problème au lieu de l'attendre : **on donne la
synchro au DSP, et on regarde s'il traite le SI.** C'est la dernière ligne du
tableau des pistes de §3 (« ni SCH ni SI »), prise par l'autre bout.

| profil | FB / SB (acquisition) | SI (décodage) | campe ? |
|---|---|---|---|
| `shunt_legit` | hôte | **gr-gsm** — le DSP est shunté | oui |
| `native_twl` | hôte / TWL | **DSP** | oui (synchro fournie) |
| `native` | DSP | DSP | non, aujourd'hui |

**Règle de frontière** : *si les SI viennent de gr-gsm, ce n'est pas ce mode,
c'est `shunt_legit`.* Les deux seules portes par lesquelles un bloc gr-gsm entre
dans `a_cd` sont `CALYPSO_SHUNT_FEED_SI` et `CALYPSO_INJECT_ACD` — le profil les
pose à 0, et `run_modules/01-profil.sh` proteste si on les rallume.

⚠️ **BÉQUILLE assumée** : FB et SB sont **substitués** par l'hôte
(`SHUNT_REAL_FB` + `INJECT_SB` + le transport `SHUNT_PUBLISH_FB`). Ce qu'elle
masque : l'incapacité actuelle du corrélateur natif à publier `d_fb_det` — donc
`data[0x08f8]` **n'est pas un verdict dans ce mode**, et ne doit jamais être cité
comme tel ; le seul mode qui juge l'acquisition est `native` (§3). À retirer
quand `native` sort un `d_fb_det` positif : ce profil se dissout alors dans
`native`.

### Lancer

```bash
docker exec -it osmo-operator-1 bash -lc '
  cd /opt/GSM/osmo-qemu-calypso &&
  CALYPSO_MODE=native_twl CALYPSO_DEBUG=BSP,A_CD-BY-BURST ./run.sh --restart'
```

Le profil pose lui-même, côté DSP : `DSP_RUN_C54X=1 DSP_SHUNT=0
FRAME_IT_NATIVE=1 TPU_DSP_FRAME_IT=1 BSP_DARAM_FORCE=1 BSP_DARAM_ADDR=0x4c00
BSP_DARAM_LEN=296 BSP_IQ_DECIM=4` ; côté synchro : `SHUNT_REAL_FB=1 INJECT_SB=1
SHUNT_PUBLISH_FB=1 SHUNT_NO_GRGSM=0 SHUNT_NO_CANNED=1` ; côté SI : les cinq
gates d'injection à `0` ; et l'instrument `WATCH_ACD=1`. Vérifiez-le **au
manifeste**, jamais à la ligne de commande (§5).

`CALYPSO_DEBUG` est obligatoire pour que les sondes BSP et les totaux `a_cd`
parlent : leur silence ne veut alors rien dire (§5).

### Les 3 vérifications, dans l'ordre

```bash
# T1 — la synchro est bien fournie : le mobile se cale sur un BSIC réel
docker exec osmo-operator-1 bash -lc '
  grep -a "BSIC" /root/qemu.log | tail -3 ;
  grep -a "ALLC task=24" /root/qemu.log'

# T2 — le DSP est alimenté en bursts, aucun jeté
docker exec osmo-operator-1 bash -lc '
  grep -ac "deliver: gate shunt LEVE" /root/qemu.log ;
  grep -ac "dropping fn=" /root/qemu.log'

# T3 — LE critère du mode : le DSP écrit-il a_cd de son propre opcode ?
docker exec osmo-operator-1 bash -lc '
  grep -a "WATCH-ACD" /root/qemu.log | head -10 ;
  grep -a "A_CD-BY-BURST" /root/qemu.log | tail -2'

# T3bis — contrôle d'honnêteté : AUCUNE injection au manifeste
docker exec osmo-operator-1 bash -lc '
  grep -aoE "CALYPSO_(SHUNT_FEED_SI|INJECT_ACD)=[0-9]" /root/qemu.log | sort -u'
```

| # | Question | Critère de décision, posé d'avance |
|---|---|---|
| T1 | la synchro est-elle fournie ? | un `BSIC` ≠ 0. Sinon le mode n'a pas démarré et **rien d'autre ne se lit** — on ne conclut pas sur le SI d'un DSP non synchronisé. `ALLC task=24` dit en plus que l'ARM a confié le CCCH au DSP (`calypso_fbsb.c:85`) |
| T2 | le DSP est-il alimenté ? | `deliver: gate shunt LEVE` > 0 **et** `dropping fn=` = 0 |
| T3 | **le DSP traite-t-il le SI ?** | ≥ 1 ligne `WATCH-ACD DSP-opcode-write data[0x09d2..0x09e0]` (`calypso_c54x.c:2563`, écriture **opcode**, plafonnée à 60). **Zéro est une réponse — négative — pas un échec de run.** |
| T3bis | la réponse est-elle honnête ? | `FEED_SI=0` **et** `INJECT_ACD=0` au manifeste. Si l'un vaut 1, T3 est un artefact : c'est gr-gsm qui a rempli `a_cd` (`calypso_dsp_shunt.c:2249`) |

`a_cd` = `data[0x09D0..0x09DE]`. Une écriture **opcode** dans cette plage est du
DSP ; une écriture directe du shunt n'en est pas — c'est précisément ce que
`WATCH-ACD` distingue, et pourquoi la sonde a été écrite le 27/07.

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
docker exec osmo-operator-1 bash -lc 'cd ${QEMU_TREE}/tools && python3 corr_iq.py --src bursts'
docker exec osmo-operator-1 bash -lc 'cd ${QEMU_TREE}/tools && python3 corr_iq.py --src ddump'
docker exec osmo-operator-1 bash -lc 'cd ${QEMU_TREE}/tools && python3 corr_iq.py --src shunt'
docker exec osmo-operator-1 bash -lc 'cd ${QEMU_TREE}/tools && python3 corr_iq.py --src all'
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

### 4.3 Les captures I/Q sont ON par défaut dans tous les modes (2026-07-30)

Motif : `corr_iq.py` ne sert à rien si le run n'a rien écrit, et on ne s'en
aperçoit qu'après. Sauf en `CALYPSO_MODE=empty`, tout run produit donc :

| fichier | posé par | plafond | `corr_iq --src` |
|---|---|---|---|
| `/dev/shm/dsp_iq.cfile` | défaut C (`calypso_dsp_shunt.c:1637`) | non | `shunt` |
| `/dev/shm/bursts.cfile` | `BSP_DUMP_RX_FILE` (`calypso.env:36`) | **non** — ~600 o/burst FCCH | `bursts` |
| `/tmp/iq_rx_*.bin` | `CALYPSO_IQDUMP` | 24 fichiers | `rxdump` |
| `/dev/shm/daram_2a00.cfile` | `CALYPSO_DARAM_DUMP` | 200 captures | `ddump` — ⚠️ **conditionnel, voir ci-dessous** |
| `/dev/shm/dsp_iq_fn.cfile` | `CALYPSO_SHUNT_IQ_CFILE2` | **512 Mo** | — (voir 4.4) |

Le shunt s'arme même en natif (sur `CALYPSO_DSP=c54x`), donc `dsp_iq.cfile`
existe dans tous les modes sans qu'on pose quoi que ce soit.

⚠️ `BSP_DUMP_RX_FILE` ne porte pas le préfixe `CALYPSO_` : il **n'apparaît pas au
manifeste**. C'est la seule de ces variables dont le manifeste ne dit rien.

⚠️ **`daram_2a00.cfile` fait souvent 0 ko, et ce n'est pas un bug d'activation.**
Mesuré le 30/07 en `native_twl` : la sonde est bien armée (`[c54x] DARAM-DUMP
armed pc=0x9ac0 max=200 -> … (ok)` — c'est ce fopen qui crée le fichier vide),
mais elle n'écrit que si **deux** conditions tombent, et aucune des deux n'est
garantie :
1. `exec_pc == CALYPSO_DARAM_DUMP_PC` (défaut **0x9ac0**). Dans ce run, la seule
   occurrence de `0x9ac0` dans tout le journal est la ligne d'armement, et
   **aucun `PC=0x9xxx`** n'apparaît : la banque 0x9xxx appartient au banc
   `native_helped`, dont l'entrée corrélateur est reroutée (`FB_CORR_ENTRY`).
2. `d_fb_mode[0x08f9] != 0`, sauf `CALYPSO_DARAM_DUMP_ANYMODE=1`.

De plus la base filmée est **codée en dur à `0x2a00`** (aucune variable pour la
changer), alors que ces profils livrent en **`0x4c00`** (`BSP_DARAM_ADDR`) : même
déclenchée, la sonde filmerait l'autre tampon. `--src ddump` est donc un
instrument pointé sur le banc du 27/07, pas une capture universelle. Pour l'I/Q
réellement livrée à ce DSP : `--src bursts`. Pour la faire tirer ici :
`CALYPSO_DARAM_DUMP_PC=<un PC réellement exécuté> CALYPSO_DARAM_DUMP_ANYMODE=1`.

⚠️ `CALYPSO_DARAM_DUMP_ANYMODE` reste à 0 par défaut, exprès : sans ce garde-fou (27/07) le
plafond de 200 est consommé dès le boot pendant que `d_fb_mode[0x08f9]==0`, et on
en conclut à tort « le buffer ne contient jamais de FCCH ».

### 4.4 Décoder l'I/Q d'un run avec `grgsm_decode`

**La pile décode DÉJÀ cet I/Q avec gr-gsm, en direct.** Le module
`66-grgsm-decode.sh` lance :

```
grgsm_decode -m BCCH_SDCCH4 -t 0 -a 514 -c /tmp/iq_grgsm.fifo -s 1083333 -v
```

et ça marche : 2 000+ `PAGING REQUEST 1` dans `$LOG_DIR/grgsm_decode.log` (mesuré
le 30/07). Donc l'I/Q du shunt **est** décodable ; c'est le chemin FIFO live, à
4 SPS, mode C0 combiné.

⚠️ **Une seule instance à la fois** : `grgsm_decode` bind `UDP 127.0.0.1:4729`, et
le décodeur live le tient déjà. Toute invocation offline échoue sur
`RuntimeError: bind: Address already in use` — et si vous filtrez la sortie, ça
ressemble à « rien décodé ». J'ai fait exactement cette erreur le 30/07 et j'en
ai tiré une fausse conclusion. Isolez le réseau :

```bash
docker exec osmo-operator-1 bash -lc '
  unshare -rn bash -c "ip link set lo up
    /root/.env/bin/grgsm_decode -c /tmp/snap.cfile -s 1083333 -a 514 -m BCCH_SDCCH4 -t 0 -v"'
```

⚠️ `/tmp` est un **tmpfs de 512 Mo** : un snapshot de cfile le remplit vite, et un
`/tmp` plein peut casser la pile en cours. Snapshottez dans `/dev/shm` (8 Go) ou
par tranches, et supprimez après.

⚠️ **Le cfile #2 (`dsp_iq_fn.cfile`) est incohérent avec lui-même** : `spf=2500`
floats = 1250 complexes = une trame à **1 SPS**, alors que le contenu des bursts y
est écrit à **4 SPS**. Rien ne peut s'y verrouiller — vérifié, 0 message à 270833
comme à 1083333. C'est probablement pourquoi `CALYPSO_IQ_CFILE_SPF` avait été
rendu « sweepable » ; la valeur cohérente avec du 4 SPS serait **10000**. À
trancher par un run, pas par déduction.

Le cfile #2 rejoue chaque burst à sa position de FN et comble les trous
(`calypso_dsp_shunt.c:2076`) — c'est l'idée juste, mal cadencée :

```bash
# 1. snapshot — le run écrit dans le fichier pendant que vous le lisez
docker exec osmo-operator-1 bash -lc '
  cp /dev/shm/dsp_iq_fn.cfile /tmp/snap_fn.cfile && ls -l /tmp/snap_fn.cfile'

# 2. décodage (fs = 270833 : spf=2500 floats/trame = 1250 complexes / 4,615 ms = 1 SPS)
docker exec osmo-operator-1 bash -lc '
  /root/.env/bin/grgsm_decode -c /tmp/snap_fn.cfile -s 270833 -a 514 -m BCCH -t 0 -v'
```

- `-a` doit être l'ARFCN **du run** : `CALYPSO_CCCH_ARFCN` (défaut 514).
- `-s 270833` vient de `spf` (`CALYPSO_IQ_CFILE_SPF`, défaut 2500). Si vous
  changez `spf`, la cadence change : `fs = spf / 2 / 4.615e-3`.
- Rien ne sort ? `-p` (print-bursts) sépare les deux cas : des bursts mais pas de
  messages = démod OK / décodage KO ; **rien du tout** = pas de verrouillage, donc
  un problème de cadence, d'ARFCN, ou un fichier trop court.
- Le plafond de 512 Mo (`CALYPSO_IQ_CFILE2_MAX_MB`) ferme le fichier proprement :
  il reste décodable. `=0` pour illimité — 7,8 Go/h en temps réel, dans la RAM.

Pour `bursts.cfile` / `daram_2a00.cfile`, l'instrument n'est pas `grgsm_decode`
mais `corr_iq.py` (§4.2) : ce sont des captures IQ16 par burst, pas un flux.
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
   `${QEMU_TREE}` **uniquement** ; `${GSM_ROOT}/qemu-calypso` est un overlay
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

- Ne pas écrire dans `${GSM_ROOT}/qemu-calypso` (overlay mort au runtime).
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
