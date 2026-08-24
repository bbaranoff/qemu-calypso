# Rapport complet — banc `shunt_legit`, 2026-08-24

> Réseau GSM de test complet, de la synchro radio à l'appel voix entre deux
> abonnés, chiffré en A5/1. Toutes les valeurs de ce rapport sont **mesurées**,
> avec la commande et la ligne témoin qui les produisent.

---

## 1. Identité du run

| | |
|---|---|
| mode | `shunt_legit` — le **défaut**, et c'est voulu |
| lancement | `cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' ./start-direct.sh --no-attach` |
| arrêt | même ligne avec `--stop` |
| `LOG_DIR` | **`/tmp/osmo-nitb/logs`** — et *non* `/tmp/calypso/logs`, qui appartient au lancement par `run.sh` |
| producteur L1 | `RUN_C54X=0`, `DSP_SHUNT=1` → **c54x éteint, gr-gsm démodule** |
| pipeline / profil | `bridge` / `hybrid` |
| chiffrement | `CIPH_A5=1`, `ENCRYPTION='a5 1'` |
| build | `Aug 24 2026 00:02:46` |

Deux mobiles, **deux cellules distinctes** — un QEMU Calypso et un faketrx :

| MS | VTY | IMSI | MSISDN | ARFCN | CGI | journal |
|---|---|---|---|---|---|---|
| `ms 1` | **4247** | 001010001000001 | **10001** | 514 (DCS) | 001-01-1-**6001** | `mobile.log` |
| `ms 1` | **4248** | 001010001000002 | **10002** | 516 (DCS) | 001-01-1-**6002** | `sidecar-mobile.log` |

Les deux en `cell selection state: C3 camped normally`.

---

## 2. Verdict — la chaîne va de bout en bout

| étape | état | valeur |
|---|---|---|
| synchro FB | ✅ | 11 |
| synchro SB | ✅ | 22 |
| sysinfo | ✅ | 16 |
| camp | ✅ | 15 |
| accès RACH | ✅ | 3 |
| canal dédié | ✅ | 7 |
| authentification | ✅ | 3 × `AUTHENTICATION REQUEST` / `RESPONSE` |
| **chiffrement A5/1** | ✅ | **7 × `CIPHERING MODE COMMAND` + 7 × `COMPLETE`** |
| LU accepté | ✅ | 1, TMSI réalloué |
| **appel voix 10001 ↔ 10002** | ✅ | **état `ACTIVE`, audio GAPK codec `fr`** |
| **SMS MT et MO** | ✅ | **les deux sens, 0 erreur** — voir §7 |

---

## 3. Authentification — valeurs HLR

`OsmoHLR` VTY **4258**, `subscriber imsi <imsi> show` :

| | 10001 | 10002 |
|---|---|---|
| ID | 1 | 2 |
| IMSI | `001010001000001` | `001010001000002` |
| MSISDN | `10001` | `10002` |
| IMEI | `358925005901018` | `358925005901018` |
| VLR | `VLR-SDR-OP1` | `VLR-SDR-OP1` |
| dernier LU (CS) | `2026-08-24T02:09:34Z` | `2026-08-24T02:09:32Z` |
| **algo 2G** | **COMP128v1** | **COMP128v1** |
| **Ki** | `00112233445566778899aabbccdd0101` | `00112233445566778899aabbccdd0201` |

Base : `/var/lib/osmocom/hlr.db`, tables `subscriber` + `auc_2g`.
`auc_3g` est **vide** — pas de Milenage, authentification 2G uniquement.

---

## 4. Kc — tuples A3A8 vus par le MSC

`OsmoMSC` VTY **4254**, `show subscriber imsi <imsi>` :

### 10001 — TMSI `0719E3FE`

```
A3A8 last tuple (used 2 times):
  seq # : 2
  RAND  : e4 3c 35 78 9e 27 05 97 75 44 d8 7f 66 0e cc b2
  SRES  : 33 29 ea 0c
  Kc    : f7 3a 48 77 98 59 5c 00
```

### 10002 — TMSI `3EF5D4F4`

```
A3A8 last tuple (used 4 times):
  seq # : 0
  RAND  : 61 55 f4 59 b9 b4 10 e6 ee 98 00 0b bf 11 2e 7b
  SRES  : 24 f0 f0 94
  Kc    : 5e e5 25 bf f5 9f 24 00
```

### La signature COMP128v1 est visible dans les Kc

Les deux clés se terminent par un octet nul, **et** les deux bits de poids
faible de l'octet précédent sont à zéro :

```
10001 : ... 5c 00   ->  0x5c = 0101 11|00
10002 : ... 24 00   ->  0x24 = 0010 01|00
```

C'est la faiblesse connue de COMP128v1 : il ne produit que **54 bits utiles**
sur les 64 du Kc, les 10 derniers étant forcés à zéro. La mesure le confirme sur
les deux abonnés indépendamment — ce n'est pas un artefact.

### Compteurs MSC

```
gsup:rx:auth_tuples            : 10
gsup:tx:send_auth_info (req)   : 2
gsup:rx:send_auth_info:res     : 2
gsup:rx:send_auth_info:err     : 0
gsup:tx:auth_fail:rep          : 0
bssmap:cipher_mode_reject      : 0
bssmap:cipher_mode_complete    : 0   <-- voir §9, écart à expliquer
```

---

## 5. Chiffrement A5/1 sur l'air

Configuré côté BSC (`show running-config`) :

```
encryption a5 1
```

Et effectivement commandé sur le canal dédié, 7 fois :

```
gsm48_rr.c:1219 CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)
```

`sc=1` = chiffrement activé, `algo=A5/1`, `cr=1` = IMEISV demandé.
Suivi de 7 × `CIPHERING MODE COMPLETE`. Le canal porte bien
`Channel type 8, subch 0, ts 2, mode 1, cipher 1`.

---

## 6. Appel voix 10001 → 10002 — abouti

Commandes VTY (`socat - TCP:127.0.0.1:<port>`) :

```bash
# appel
enable ; call 1 10002        # sur 4247
# decrocher
enable ; call 1 answer       # sur 4248
# raccrocher
enable ; call 1 hangup       # sur 4247
```

Réponses du VTY : `% Call is connected`, puis `% Call has been released`.

### Machine à états CC — appelant (10001)

```
NULL -> MM_CONNECTION_PEND        Sending MMCC_EST_REQ
                                  Received 'MMCC_EST_CNF'
     -> INITIATED                 sending SETUP, timer T303 (30 s)
     -> MO_CALL_PROC              received CALL PROCEEDING, T303 arrete
     -> CALL_DELIVERED            received ALERTING  -> MNCC_ALERT_IND
                                  received CONNECT
                                  sending CONNECT ACKNOWLEDGE
```

### Machine à états CC — appelé (10002)

```
NULL -> CALL_PRESENT              received SETUP -> MNCC_SETUP_IND
     -> MO_TERM_CALL_CONF         sending CALL CONFIRMED (proceeding)
     -> CALL_RECEIVED             sending ALERTING
     -> CONNECT_REQUEST           sending CONNECT, timer T313 (30 s)
     -> ACTIVE                    received CONNECT ACKNOWLEDGE, T313 arrete
                                  MNCC_SETUP_COMPL_IND
```

`callref=80000002`, `transaction_id=8` côté appelé ; `callref=3`,
`transaction_id=0` côté appelant.

### Audio

Des **deux** côtés :

```
DGAPK pq_codec.c:81  Adding codec fr, decoding from format gsm
DGAPK pq_alsa.c:197  Adding ALSA output (dev='gsm_out', blk_len=320)
DGAPK gapk_io.c:304  chain 'source/tch_fb -> format/gsm -> ecu/fr -> format/gsm'
DGAPK gapk_io.c:472  GAPK I/O initialized for MS '1', codec 'fr'
```

Raccrochage propre : `DISCONNECT` → `RELEASE` → `RELEASE COMPLETE` → `NULL`.

---

## 7. SMS — les deux sens, vérifiés

### MT (réseau → mobile)

```
02:12:29  gsm411_sms.c:306  RX SMS: MTI: 0x00, MR: 0x00, PID: 0x00, DCS: 0x00,
                            OA: 19990011444, UserDataLength: 0x13,
                            UserData: "rapport-run test MT"
02:12:29  gsm411_sms.c:342  TX: SMS RP ACK
```

### MO (mobile → réseau), puis retour MT

```
02:16:38  gsm411_sms.c:717  TX: SMS DELIVER            <- MO emis par 10001
02:16:43  gsm0411_smc.c:263 SMC(0) received CP-ACK
02:16:43  gsm0411_smr.c:314 SMR(0) RX SMS RP-ACK
02:16:43  gsm411_sms.c:522  RX SMS RP-ACK (MT)
02:17:01  gsm411_sms.c:306  RX SMS: OA: 10002, UserData: "test"   <- retour
02:17:01  gsm411_sms.c:342  TX: SMS RP ACK
```

*(`RP-ACK (MT)` désigne la constante `GSM411_MT_RP_ACK_MT` — un **type de
message**, pas « mobile-terminated ».)*

### Boîte de réception du mobile — `/root/.osmocom/bb/sms.txt`

```
[SMS from 19990011444]      [SMS from 10001]      [SMS from 10002]
rapport-run test MT         test                  test
```

### Compteurs d'erreur

```
MT_FORWARD_SM_ERROR : 0        RP ERROR / RP-ERROR : 0
```

---

### ⚠️ Correction d'une erreur d'analyse de ce rapport

Une première version concluait à un **échec** du SMS MT sur cette séquence :

```
gsm0411_smc.c:338 SMC(8) cannot release yet current state: WAIT_CP_ACK
gsm411_sms.c:940  Received 'MMSMS_REL_IND' from MM
gsm0411_smc.c:109 SMC(8) dropping pending message
```

**C'était faux, et pour une raison instructive : l'extrait commençait APRÈS la
livraison.** Deux lignes plus haut, dans la même transaction `SMC(8)` et à la
même seconde, se trouvent le `RX SMS` avec le texte et le `TX: SMS RP ACK`.

Cette queue est **structurellement bénigne** :
`gsm0411_smr.c:236-237` appelle `mn_send(MNSMS_DATA_REQ)` **puis**
`gsm411_send_release()` dans la même pile ; l'envoi vient de faire passer SMC en
`WAIT_CP_ACK`, donc `gsm0411_smc.c:336-342` pose `cp_rel = 1` et diffère — c'est
un report délibéré. Et `inst->cp_msg` est la **copie maître de retransmission** :
l'émission part sur un clone (`gsm0411_smc.c:199-203`), donc la libérer ne perd
aucun SMS.

**Leçon :** lire une machine à états par sa *fin* mène à la conclusion inverse de
la vérité. Il faut la lire par sa transaction complète.

---

## 8. Le producteur L1 — le contrat que le DSP devra remplir

C'est **la cible de la substitution** : `native` doit produire cela, aux mêmes
adresses, avec le vrai c54x à la place de gr-gsm.

| grandeur | `shunt_legit` (référence) | `native` (état actuel) |
|---|---:|---:|
| `SCH reel (gr-gsm)` | 37 (`BSIC=7 ncc=0 bcc=7`) | — |
| `feed_si` → `a_cd` | 663 | 0 |
| `feed_agch` | 1328 | 1794 |
| c54x `d_fb_det 0→1` | 0 (c54x éteint) | 437 |
| c54x tâche SB | 0 | 4 |
| **`DSP Error Status`** | **0** | **404 × « 8 » (`DSP_ERR_DMA_PROG`)** |
| déraillement émulateur | 0 | 2 |
| erreurs BTS | **3** (OML bénignes) | 1453 |

---

## 9. Points ouverts

1. **`bssmap:cipher_mode_complete = 0`** côté MSC alors que le mobile enregistre
   7 `CIPHERING MODE COMPLETE`. Écart à expliquer : compteur sur un chemin non
   emprunté (chiffrement piloté par le BSC), ou comptage manquant. Ne pas en
   conclure que le chiffrement n'a pas eu lieu — l'air dit le contraire.
2. **CP-ACK final du MT jamais reçu** (3 épisodes sur 3) : le réseau libère le
   canal avant. Cosmétique — le SMS et son RP-ACK sont déjà passés — mais c'est
   un écart 24.011 réel côté osmo-msc. Mesure qui trancherait :
   `logging level lsms debug` dans `osmo-msc.cfg` + capture GSMTAP port 4729.
3. **`native`** — `DSP_ERR_DMA_PROG` permanent, et le décodage SCH sort une
   constante (`a_sch[3] = 0xf8d8`) indépendante de l'entrée.

---

## 10. Reproduire

```bash
# lancer
cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' \
  ./start-direct.sh --no-attach

# rapport (lecture seule, LOG_DIR auto-detecte)
/opt/GSM/qemu-src/tools/rapport-run.sh

# appel 10001 -> 10002
/tmp/test-call.sh

# valeurs Kc
socat - TCP:127.0.0.1:4258   # HLR : subscriber imsi <imsi> show
socat - TCP:127.0.0.1:4254   # MSC : show subscriber imsi <imsi>

# arreter
cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' \
  ./start-direct.sh --stop
```

### Ports VTY relevés

| port | service | | port | service |
|---|---|---|---|---|
| 4239 | OsmoSTP | | 4254 | **OsmoMSC** |
| 4242 | OsmoBSC | | 4256 | OsmoSIPcon |
| 4243 | OsmoMGW | | 4258 | **OsmoHLR** |
| 4245 | OsmoSGSN | | 4260 | OsmoGGSN |
| 4247 | mobile **10001** | | 4248 | mobile **10002** |
| 4238 / 4241 / 4250 | osmo-bts-trx | | | |
