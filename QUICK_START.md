# QUICK START — QEMU-Calypso

Lancer, **piloter** et **vérifier**. Ce fichier ne décrit que ce qui est mesuré,
avec la commande et sa sortie réelle. Dernière campagne de mesures :
**2026-08-24**.

Distinction employée partout : **MESURE** (relevé, avec sa commande) /
**HYPOTHÈSE** (déduite du code, pas encore relevée) / **INVALIDE** (affirmation
retirée, ne pas réintroduire).

## Les profils, en une ligne chacun

`CALYPSO_MODE=…` — un profil = **qui fait quoi**, et rien d'autre :

| profil | FB / SB | SI | à quoi il sert |
|---|---|---|---|
| `shunt_legit` | hôte | gr-gsm | **le défaut.** La pile de bout en bout : camp, LU, auth, A5/1, SMS, appel voix (§2) |
| `native_twl` | hôte / TWL | **DSP** | **le DSP traite-t-il le SI ?** (§4) |
| `native` | DSP | DSP | la vérité sur l'acquisition (§3) |
| `native_helped` | DSP, entrée reroutée | DSP | observer le corrélateur — **sous béquille** |
| `empty` | rien de posé | rien de posé | construire un banc gate par gate |

Un profil ne pose que des `:=` : **la CLI garde toujours le dernier mot**, et
c'est le **manifeste** qui dit ce que vous avez réellement obtenu (§6).

---

# 1. Piloter le lab

Tout vit dans le conteneur **`osmo-operator-1`**. Depuis l'hôte, chaque commande
prend la forme `docker exec osmo-operator-1 …`.

```bash
docker exec -it osmo-operator-1 bash          # entrer
docker ps --format 'table {{.Names}}\t{{.Status}}'
```

## 1.1 Démarrer et arrêter

**Chaque mode a SA porte d'entrée. Elles ne sont pas interchangeables**, et c'est
pour cela que les journaux atterrissent à deux endroits différents (§1.2).

### `shunt_legit` — le défaut

```bash
cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' ./start-direct.sh
```

Depuis l'hôte, et sans s'attacher au tmux :

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/osmo_egprs &&
  CALYPSO_BRIDGE=pont ENCRYPTION="a5 1" ./start-direct.sh --no-attach'
```

### `native`

```bash
cd /opt/GSM/qemu-src && CALYPSO_MODE=native ./run.sh
```

Depuis l'hôte :

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src && CALYPSO_MODE=native ./run.sh --restart --no-attach'
```

### Arrêter — et le geste qui débloque

```bash
# shunt_legit
cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' ./start-direct.sh --stop
# native
cd /opt/GSM/qemu-src && ./run.sh --stop
```

**Si ça galère** — relance qui ne prend pas, port déjà pris, pile à moitié
morte : faire le `--stop`, **puis tuer les python restants**, qui survivent au
teardown et retiennent les FIFO I/Q et les sockets :

```bash
docker exec osmo-operator-1 bash -lc 'pkill -f python3 ; pkill -f python ; sleep 2 ; pgrep -af "python|qemu-system-arm"'
```

Puis relancer. C'est le geste qui résout la grande majorité des relances qui
coincent.

**MESURE (2026-08-24).** Le même mode lancé par `run.sh --restart` sans
`CALYPSO_BRIDGE=pont` ne monte pas :

| | `run.sh` (mauvaise porte) | `start-direct.sh` (bonne porte) |
|---|---:|---:|
| LU REQUEST | 19 | 1 |
| **LU ACCEPT** | **0** | **1** |
| TMSI | 0 | 5 |
| timer d'échec | `T3211` ×74 | — |
| erreurs BTS | **1798**, dont `send() failed on TRXD … Connection refused` ×1268 | **3** |

La BTS n'atteint pas le transceiver, le réseau ne répond jamais, et la Location
Update **expire** au lieu d'être rejetée.

## 1.2 Où sont les journaux — le piège n°1

Les deux lanceurs **n'écrivent pas au même endroit** :

| lanceur | `RUN_DIR` | `LOG_DIR` |
|---|---|---|
| `run.sh` | `/tmp/calypso` | `/tmp/calypso/logs` |
| **`start-direct.sh`** | `/tmp/osmo-nitb` | **`/tmp/osmo-nitb/logs`** |

Analyser le mauvais répertoire donne des compteurs périmés qui **ressemblent à
des pannes**. Le contrôle qui tranche :

```bash
docker exec osmo-operator-1 bash -lc '
  P=$(pgrep -x qemu-system-arm | head -1)
  tr "\0" "\n" < /proc/$P/environ | grep -E "^(LOG_DIR|RUN_DIR)="
  ls -l /proc/$P/fd | grep qemu.log'
```

## 1.3 Le rapport de run — l'instrument à dégainer en premier

```bash
docker exec osmo-operator-1 /opt/GSM/qemu-src/tools/rapport-run.sh
docker exec osmo-operator-1 /opt/GSM/qemu-src/tools/rapport-run.sh /tmp/ref-shunt-legit
```

Lecture seule, `LOG_DIR` **auto-détecté** depuis le processus vivant, et il
signale un journal qui ne grossit pas. Sa règle de conception : **un compteur nul
n'est jamais une absence prouvée** — il affiche pour chaque grandeur une ligne
témoin, et annonce un zéro comme « motif jamais vu ».

Sortie type (banc `shunt_legit` mûr) :

```
== 7. VERDICT — jusqu ou va la chaine ==
  [ OK ] synchro FB             (11)
  [ OK ] synchro SB             (22)
  [ OK ] sysinfo                (16)
  [ OK ] camp                   (15)
  [ OK ] acces RACH             (3)
  [ OK ] canal dedie            (7)
  [ OK ] LU demande             (1)
  [ OK ] LU accepte             (1)
  [ OK ] SMS                    (1)
```

## 1.4 Les VTY — ports relevés

`nc` **n'est pas installé** dans le conteneur ; utilisez `telnet` ou `socat`.

| port | service | | port | service |
|---|---|---|---|---|
| 4239 | OsmoSTP | | 4254 | **OsmoMSC** |
| 4242 | OsmoBSC | | 4256 | OsmoSIPcon |
| 4243 | OsmoMGW | | 4258 | **OsmoHLR** |
| 4245 | OsmoSGSN | | 4260 | OsmoGGSN |
| 4247 | **mobile 10001** | | 4248 | **mobile 10002** |
| 4238 · 4241 · 4250 | osmo-bts-trx | | | |

Vérifier la liste sur un run vivant, et identifier chaque port par sa bannière :

```bash
docker exec osmo-operator-1 bash -lc "ss -lntp | grep '127.0.0.1:4'"
```

Helper non interactif (à poser une fois) :

```bash
docker exec -i osmo-operator-1 tee /tmp/vty.sh >/dev/null <<'EOF'
#!/bin/bash
# vty.sh <port> <commande> [commande...]
P="$1"; shift
{ printf 'enable\n'; sleep 0.6
  for c in "$@"; do printf '%s\n' "$c"; sleep 1.2; done
  printf 'exit\n'; sleep 0.4; } | socat -T5 - TCP:127.0.0.1:"$P" 2>/dev/null | tr -d '\r'
EOF
docker exec osmo-operator-1 chmod +x /tmp/vty.sh
```

En interactif : `docker exec -it osmo-operator-1 telnet 127.0.0.1 4254`.

## 1.5 Les deux mobiles

| MS | VTY | IMSI | MSISDN | ARFCN | CGI | journal |
|---|---|---|---|---|---|---|
| `ms 1` | **4247** | 001010001000001 | **10001** | 514 (DCS) | 001-01-1-**6001** | `mobile.log` |
| `ms 1` | **4248** | 001010001000002 | **10002** | 516 (DCS) | 001-01-1-**6002** | `sidecar-mobile.log` |

Un QEMU Calypso et un faketrx, **sur deux cellules distinctes** — les appels
entre eux sont donc de vrais appels inter-cellules.

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4247 'show ms'
```

**MESURE :**

```
MS '1' is up, service is normal
  IMEI: 358925005901010
  automatic network selection state: A2 on PLMN
                                     MCC=001 MNC=01 (Test, Test)
  cell selection state: C3 camped normally
                        ARFCN=514(DCS) CGI=001-01-1-6001
```

## 1.6 Appel voix 10001 ↔ 10002

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4247 'call 1 10002'    # appeler
docker exec osmo-operator-1 /tmp/vty.sh 4248 'call 1 answer'   # decrocher
docker exec osmo-operator-1 /tmp/vty.sh 4247 'call 1 hangup'   # raccrocher
```

Réponses VTY : `% Call is connected`, puis `% Call has been released`.

**MESURE — machines à états CC des deux côtés :**

```
10001 (appelant)                        10002 (appele)
NULL -> MM_CONNECTION_PEND              NULL -> CALL_PRESENT
     -> INITIATED      (SETUP, T303)         -> MO_TERM_CALL_CONF (CALL CONFIRMED)
     -> MO_CALL_PROC   (CALL PROCEEDING)     -> CALL_RECEIVED     (ALERTING)
     -> CALL_DELIVERED (ALERTING)            -> CONNECT_REQUEST   (CONNECT, T313)
        CONNECT -> CONNECT ACKNOWLEDGE       -> ACTIVE            (CONNECT ACK)
```

Audio, des **deux** côtés :

```
DGAPK pq_codec.c:81  Adding codec fr, decoding from format gsm
DGAPK pq_alsa.c:197  Adding ALSA output (dev='gsm_out', blk_len=320)
DGAPK gapk_io.c:472  GAPK I/O initialized for MS '1', codec 'fr'
```

## 1.7 SMS — les deux sens

**MT (réseau → mobile)**, destinataire par **IMSI** :

```bash
docker exec osmo-operator-1 bash -lc \
  "OPERATOR_ID=1 /etc/osmocom/send-mt-sms.sh 001010001000001 'texte'"
```

**MO (mobile → réseau)**, commande VTY `sms MS_NAME NUMBER .LINE` :

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4247 'sms 1 10002 test'
docker exec osmo-operator-1 /tmp/vty.sh 4248 'sms 1 10001 test'
```

**MESURE — succès dans les deux sens, 0 erreur :**

```
MT :  gsm411_sms.c:306  RX SMS: MTI: 0x00, OA: 19990011444,
                        UserData: "rapport-run test MT"
      gsm411_sms.c:342  TX: SMS RP ACK

MO :  gsm411_sms.c:717  TX: SMS DELIVER
      gsm0411_smc.c:263 SMC(0) received CP-ACK
      gsm411_sms.c:522  RX SMS RP-ACK (MT)
```

Boîte de réception du mobile — `/root/.osmocom/bb/sms.txt` :

```
[SMS from 19990011444]   [SMS from 10001]   [SMS from 10002]
rapport-run test MT      test               test
```

⚠️ **Queue BÉNIGNE à ne jamais lire comme un échec** — elle est structurelle :

```
gsm0411_smc.c:338 cannot release yet current state: WAIT_CP_ACK
gsm411_sms.c:954  MM connection released.
gsm0411_smc.c:109 dropping pending message
```

`gsm0411_smr.c:236-237` envoie **puis** libère dans la même pile, et `cp_msg` est
la **copie maître de retransmission** — l'émission part sur un clone
(`gsm0411_smc.c:199-203`), la libérer ne perd aucun SMS. Le `RX SMS` et le
`TX: SMS RP ACK` sont **deux lignes plus haut**. Lire cette machine à états par
sa fin donne l'inverse de la vérité.

Contrôles d'erreur (doivent valoir 0) :

```bash
docker exec osmo-operator-1 bash -lc '
  L=/tmp/osmo-nitb/logs
  grep -c MT_FORWARD_SM_ERROR $L/smsc-op1.log
  grep -c "RP ERROR\|RP-ERROR"  $L/mobile.log'
```

## 1.8 HLR — abonnés, Ki, algorithme

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4258 'subscriber imsi 001010001000001 show'
```

**MESURE :**

```
    ID: 1
    IMSI: 001010001000001
    MSISDN: 10001
    IMEI: 358925005901018
    VLR number: VLR-SDR-OP1
    last LU seen on CS: 2026-08-24T02:09:34+00:00
    2G auth: COMP128v1
             KI=00112233445566778899aabbccdd0101
```

Directement en base — `auc_3g` est **vide** (pas de Milenage) :

```bash
docker exec osmo-operator-1 bash -lc "
  sqlite3 -readonly -header -column /var/lib/osmocom/hlr.db \
    'select id,imsi,msisdn from subscriber;'
  sqlite3 -readonly -header -column /var/lib/osmocom/hlr.db 'select * from auc_2g;'"
```

```
id  imsi             msisdn      subscriber_id  algo_id_2g  ki
--  ---------------  ------      -------------  ----------  --------------------------------
1   001010001000001  10001       1              1           00112233445566778899aabbccdd0101
2   001010001000002  10002       2              1           00112233445566778899aabbccdd0201
```

## 1.9 MSC — TMSI, tuples A3A8, **Kc**

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4254 'show subscriber imsi 001010001000001'
```

**MESURE — 10001 :**

```
    MSISDN: 10001
    LAC / cell ID: 1 / 6001
    RAN type: GERAN-A
    TMSI: 0719E3FE
    Flags:  Conf. by radio contact: true   Location conf. in HLR: true
    A3A8 last tuple (used 2 times):
      seq # : 2
      RAND  : e4 3c 35 78 9e 27 05 97 75 44 d8 7f 66 0e cc b2
      SRES  : 33 29 ea 0c
      Kc    : f7 3a 48 77 98 59 5c 00
```

**MESURE — 10002 :**

```
    TMSI: 3EF5D4F4
    A3A8 last tuple (used 4 times):
      seq # : 0
      RAND  : 61 55 f4 59 b9 b4 10 e6 ee 98 00 0b bf 11 2e 7b
      SRES  : 24 f0 f0 94
      Kc    : 5e e5 25 bf f5 9f 24 00
```

### La signature COMP128v1 est lisible dans les Kc

Les deux clés finissent par un octet nul **et** les deux bits de poids faible de
l'octet précédent sont à zéro :

```
10001 : … 5c 00   ->  0x5c = 0101 11|00
10002 : … 24 00   ->  0x24 = 0010 01|00
```

COMP128v1 ne produit que **54 bits utiles** sur 64 ; les 10 derniers sont forcés.
Constaté indépendamment sur les deux abonnés — ce n'est pas un artefact.

Compteurs d'authentification :

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4254 'show rate-counters' \
  | grep -iE 'auth|cipher|tuple'
```

```
gsup:rx:auth_tuples          : 10        gsup:rx:send_auth_info:err : 0
gsup:tx:send_auth_info (req) : 2         gsup:tx:auth_fail:rep      : 0
gsup:rx:send_auth_info:res   : 2         bssmap:cipher_mode_reject  : 0
```

⚠️ `bssmap:cipher_mode_complete = 0` alors que le mobile enregistre 7
`CIPHERING MODE COMPLETE` : **écart de comptage, pas absence de chiffrement** —
l'air dit le contraire (§1.10). À ne pas citer comme preuve d'un chiffrement
absent.

## 1.10 Chiffrement A5/1

Côté BSC :

```bash
docker exec osmo-operator-1 /tmp/vty.sh 4242 'show running-config' | grep -i encryption
#  encryption a5 1
```

Côté air, **MESURE** (7 occurrences) :

```
gsm48_rr.c:1219 CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)
gsm48_rr.c:4035 Channel type 8, subch 0, ts 2, mode 1, cipher 1
```

`sc=1` = chiffrement activé · `algo=A5/1` · `cr=1` = IMEISV demandé.

## 1.11 Build

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src/build && ninja qemu-system-arm'
```

Vérifier **quel binaire tourne** — `BUILD-STAMP` **ment** : la macro
`__DATE__/__TIME__` vit dans `calypso_dsp_shunt.c:107` et date **son propre**
translation-unit, pas celui qu'on vient de modifier :

```bash
docker exec osmo-operator-1 bash -lc '
  P=$(pgrep -x qemu-system-arm | head -1)
  stat -c "%y  %n" /proc/$P/exe
  find /opt/GSM/qemu-src/build -name "*calypso_c54x*.o" -printf "%T+ %p\n"'
```

---

# 2. `shunt_legit` — le mode qui marche

C'est le **défaut**. Le FB/SB est produit côté hôte et présenté à l'ARM par
intercept de lecture (`calypso_dsp_shunt.c:1463-1490`, appelé depuis
`calypso_trx.c:297`) ; les SI viennent de gr-gsm.

Lancement : §1.1. Vérification : `tools/rapport-run.sh` (§1.3), puis les gestes
de §1.5 à §1.10.

**MESURE 2026-08-24 — la chaîne va de bout en bout :**

| étape | valeur | preuve |
|---|---:|---|
| synchro SB | 22 | `=> SB 0x0125011c: BSIC=7 fn=3805` |
| sysinfo | 16 | `New SYSTEM INFORMATION 4 (lai=001-01-1)` |
| camp | 15 | `Going to camping (normal) ARFCN 514(DCS)` |
| accès RACH | 3 | `CHANNEL REQUEST: 00 (Location Update with NECI)` |
| authentification | 3 | COMP128v1, `gsup:rx:auth_tuples = 10` |
| chiffrement A5/1 | 7 | `CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)` |
| **LU accepté** | 1 | `LOCATION UPDATING ACCEPT (lai=001-01-1)` + TMSI |
| **appel voix** | 1 | état `ACTIVE`, GAPK codec `fr` des deux côtés |
| **SMS MO + MT** | ✅ | 0 erreur |

Relevé complet, avec toutes les valeurs :
[`RAPPORT_COMPLET_20260824.md`](hw/arm/calypso/doc/RAPPORT_COMPLET_20260824.md).

⚠️ **Ce que ce mode NE prouve PAS** : rien sur le DSP. C'est gr-gsm qui
démodule ; le firmware ARM, lui, tourne pour de vrai.

---

# 3. `native` — la vérité sur l'acquisition

Objectif : que ce soit le **DSP c54x** qui produise `d_fb_det`, puis le SCH, puis
les SI. **Ce mode ne campe pas** aujourd'hui — mais il a franchi une étape.

```bash
cd /opt/GSM/qemu-src && CALYPSO_MODE=native ./run.sh

# avec les sondes de cette section, depuis l hote :
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src &&
  CALYPSO_MODE=native CALYPSO_SBFN=1 CALYPSO_SUBC=1 ./run.sh --restart --no-attach'
```

⚠️ Les journaux du natif sont donc dans **`/tmp/calypso/logs`** (§1.2).

## 3.1 ✅ RÉSOLU — le FB est acquis par le vrai DSP (2026-08-24)

L'ancienne rédaction de ce fichier affirmait « `d_fb_det` = 0 sur tous les runs ».
**C'est périmé.**

```bash
docker exec osmo-operator-1 bash -lc '
  q=/tmp/calypso/logs/qemu.log
  grep -c "FBDET-WR .*0x0000 -> 0x0001" $q
  grep    "FBDET-WR .*0x0000 -> 0x0001" $q | grep -oE "PC=0x[0-9a-f]+" | sort -u'
```

**MESURE** : **437** transitions, **toutes** écrites par la mask-ROM à
`PC=0x79e4`. Côté ARM :

```
FB1 (3415:9): TOA=   39, Power= -52dBm, Angle=   -1Hz
  fn_offset=3406 … scheduling next FB/SB detection task with delay 1
Synchronize_TDMA
```

Aucune gate d'injection posée (`CALYPSO_INJECT_*=0`, `CALYPSO_CANNED` annonce
`FBDET=0 TOA=0 PM=0 SNR=0 ANGLE=0`). L'IMR n'est plus à zéro non plus :
`IMR-ARM 0x0000 -> 0x3000 PC=0xa4c7` puis `0x52ed`, vec28/bit12 armé.

## 3.2 🔧 LE MUR COURANT — le décodage du SCH

La tâche SB s'exécute, mesure un TOA plausible, écrit ses slots — et le firmware
l'écarte :

```
a_sch[0] = 0x8100     = B_BLUD | B_SCH_CRC   ->  prim_fbsb.c:181 abandonne
a_sch[3] = 0xf8d8     CONSTANT sur 21/21 ecritures
```

**MESURE décisive** : 10 contenus de burst **distincts** ont été présentés à la
tâche SB (10 énergies distinctes, 15 trames SCH sur 16 grâce à
`CALYPSO_BSP_DARAM_FCCH_ONLY=1`), et la sortie n'a pas bougé d'un bit. *Un
décodeur dont la sortie ne dépend pas de l'entrée ne décode pas.*

## 3.3 HYPOTHÈSE PRINCIPALE — on alimente le mauvais bout de la chaîne

Le front-end du démodulateur de la ROM (`0x9f95-0x9fcb`) est un filtre
multi-cadence, **désassemblage vérifié** :

```
0x9f9f  stm  #0x027e, BK      ; anneau de 638 mots
0x9fa9  rptb 0x9fc7
  0x9fb5  ld   *AR6-0%, A     ; LIT  l'entree, circulaire
  0x9fb8  sth  A, *AR4+       ; ECRIT la sortie, lineaire
0x9fcb  banz *AR7-, 0x9fa7
```

Deux mesures du dépôt, **prises à six mois d'écart et jamais confrontées** :

| adresse | ce qui l'atteste | rôle probable |
|---|---|---|
| **`0x4c00`** | `TODO.md` P2 : « `PC=0x9fb5` lit `0x4c00/05/0a/0f/15/1a`, stride 5 = polyphase 6 taps » | **entrée** du démod |
| **`0x2a00`** | `calypso_bsp.c:1123-1129` : « DSP READS `0xCAFE` at `0x2a00..0x2a13` via `PC=0x93a5` » | **sortie** du démod, lue par le corrélateur |

Or le défaut actuel dépose en **`0x2a00`** (`calypso_bsp.c:1131`). Si `AR4` vaut
bien `0x2a00`, on alimente la **sortie** du démodulateur et son **entrée** n'est
alimentée par personne — le Viterbi mange de l'I/Q, ce qui expliquerait d'un coup
la constante `0xf8d8` et le CRC toujours faux.

**La mesure qui tranche** (sonde posée, lecture seule) :

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src &&
  CALYPSO_MODE=native CALYPSO_DEMODIO=1 ./run.sh --restart --no-attach'

docker exec osmo-operator-1 bash -lc \
  'grep DEMOD-IO-PROBE /tmp/calypso/logs/qemu.log | head -20'
```

`SORTIE … (0x2a00 ? OUI)` → hypothèse confirmée, tout le feed BSP est à refaire.
`… non` → elle tombe, chercher en aval.

## 3.4 Les sondes du mode natif

Toutes en **lecture seule**, inertes par défaut :

| gate | sonde | ce qu'elle répond |
|---|---|---|
| `CALYPSO_SBFN=1` | `SBFN-PROBE` | quelle trame est en DARAM quand la tâche SB la lit (`fn`, `fn%51`, dépôts depuis le SB précédent, amplitude) |
| `CALYPSO_SUBC=1` | `SUBC-PROBE` | dividende, quotient et `T` de la division qui alimente les coefficients — **avec un contrôle armé sur `0x989f`** qui atteste que la sonde est vivante |
| `CALYPSO_SUBC=1` | `MVDD-PROBE` | la garde `0x7ccd` et les copies `0x7ce0`/`0x7ce4` qui remplissent les blocs 3 et 4 |
| `CALYPSO_DEMODIO=1` | `DEMOD-IO-PROBE` | `AR6` (entrée) et `AR4` (sortie) du front-end démod — §3.3 |

**Discipline non négociable** : toute sonde muette doit s'accompagner d'un
**contrôle armé sur un site dont on sait qu'il s'exécute**. Sans lui, « la sonde
n'a rien vu » et « le code ne tourne pas » sont indiscernables — piège payé
plusieurs fois, dont une conclusion entièrement fausse sur une division jamais
exécutée.

## 3.5 Sas ISA ouvert : `CALYPSO_FIX_LK_SHFT`

Défaut **OFF**. Le décodeur applique au champ de décalage des formes
`ADD/SUB/LD/AND/OR/XOR #lk` — **4 bits** (`op & 0xF`) — la règle de signe d'un
champ de **5 bits**. On ne représente pas −16..15 sur 4 bits : la conversion est
incohérente en soi, et `doc/opcodes/tic54x-opc.c` donne bien `OP_SHFT` (non
signé) pour `ld`.

**MESURE** en `0x7d19` (`f02f 0001` = `LD #1, SHFT=15, A`) :

| | sas OFF | sas ON |
|---|---|---|
| `A` avant le `SFTA` | `0x0000000000` | `0x0000008000` |
| quotient en `0x7d1e` | `0x0000` | `0xfff4` |
| `T` au `MPY` `0x81e4` | `0x0000` (27/27) | `0xfff4` |
| `Synchronize_TDMA` | 8 | 13 (pas de régression) |

Validé aux niveaux 1 (formel) et 2 (grandeur physique) de `environnement/fixes.env` ;
niveau 3 à faire. Portée volontairement étroite (sous-codes 1–5, `ADD` non
touché) pour que la mesure reste interprétable.

## 3.6 `DSP_ERR_DMA_PROG` — l'erreur 8 permanente

```bash
docker exec osmo-operator-1 bash -lc \
  "grep -oE 'DSP Error Status: [0-9]+' /tmp/calypso/logs/osmocon.log | sort | uniq -c"
```

**MESURE** : `404 DSP Error Status: 8` en natif, **0 en `shunt_legit`** — et
présente même quand la tâche SB ne tourne quasiment pas, donc **indépendante** du
mur SCH. `d_error_status == data[0x3f92] & 0x0FFF`, bit 3 = `DSP_ERR_DMA_PROG`.

Cause documentée dans `hw/arm/calypso/calypso_dma.h` : la file DMA du firmware
DSP sature parce que le modèle accepte la programmation mais n'exécute aucun
transfert et ne lève jamais `DMAC0..5`. Module `calypso_dma.c` présent, gaté
`CALYPSO_DMA=1`, **défaut OFF**, avec un conflit de mapping MMR à trancher
(SPRU131 `DMPREC=0x54` contre le modèle qui y met `DMSA`).

---

# 4. `native_twl` — la question du SI, sans attendre le FB/SB

Profil qui donne la synchro au DSP pour poser la question du SI sans attendre
l'acquisition. **BÉQUILLE assumée** : FB et SB y sont substitués par l'hôte, donc
`data[0x08f8]` **n'est pas un verdict dans ce mode**. Le seul mode qui juge
l'acquisition est `native` (§3).

```bash
docker exec osmo-operator-1 bash -lc '
  cd /opt/GSM/qemu-src &&
  CALYPSO_MODE=native_twl CALYPSO_DEBUG=BSP,A_CD-BY-BURST ./run.sh --restart --no-attach'
```

**Le critère du mode** — le DSP écrit-il `a_cd` de son propre opcode ?

```bash
docker exec osmo-operator-1 bash -lc '
  grep -a "WATCH-ACD" /tmp/calypso/logs/qemu.log | head -10'
```

≥ 1 ligne `WATCH-ACD DSP-opcode-write data[0x09d2..0x09e0]`
(`calypso_c54x.c:2563`) = oui. **Zéro est une réponse — négative — pas un échec
de run.** Contrôle d'honnêteté obligatoire : `CALYPSO_SHUNT_FEED_SI=0` **et**
`CALYPSO_INJECT_ACD=0` au manifeste, sinon c'est gr-gsm qui a rempli `a_cd`.

---

## 5. Boîte à outils de diagnostic

### 5.1 Sondes (toutes gatées par variable d'environnement, **défaut OFF**)

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

### 5.2 `tools/corr_iq.py` — l'instrument de référence de la chaîne I/Q

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

### 5.3 Les captures I/Q sont ON par défaut dans tous les modes (2026-07-30)

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

### 5.4 Décoder l'I/Q d'un run avec `grgsm_decode`

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

## 6. Les pièges connus

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

## 7. Ne pas faire

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
