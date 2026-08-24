# Rapport de run — `shunt_legit`, 2026-08-24 02:09→02:13

Relevé produit par `tools/rapport-run.sh` (lecture seule, utilisable pendant que
la pile tourne). Chaque chiffre est un compteur sur les journaux du run, assorti
d'une ligne témoin ; un `0` y est annoncé **« motif jamais vu »**, jamais comme
une absence prouvée.

---

## 1. Identité du run

| | |
|---|---|
| mode | `shunt_legit` — c'est le **défaut**, et c'est voulu |
| lancement | `cd /opt/GSM/osmo_egprs && CALYPSO_BRIDGE=pont ENCRYPTION='a5 1' ./start-direct.sh --no-attach` |
| arrêt | même ligne avec `--stop` |
| `LOG_DIR` | **`/tmp/osmo-nitb/logs`** (et *non* `/tmp/calypso/logs`) |
| producteur L1 | `RUN_C54X=0`, `DSP_SHUNT=1` → **c54x éteint, gr-gsm démodule** |
| pipeline / profil | `bridge` / `hybrid` |
| chiffrement | `CIPH_A5=1`, `ENCRYPTION='a5 1'` |
| build | `Aug 24 2026 00:02:46` |

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
| LU demandé | ✅ | 1 |
| **LU accepté** | ✅ | **1** |
| **appel voix** | ✅ | **1, état `ACTIVE` atteint** |
| SMS | ⚠️ | CP-DATA reçu **puis abandonné** (§5) |

```
=> SB 0x0125011c: BSIC=7 fn=3805(2/ 9/31) qbits=4908
gsm322.c:3622  Going to camping (normal) ARFCN 514(DCS).
gsm48_rr.c:2420 CHANNEL REQUEST: 00 (Location Update with NECI)
gsm48_mm.c:2550 LOCATION UPDATING REQUEST
gsm48_mm.c:2662 LOCATION UPDATING ACCEPT (lai=001-01-1)
gsm48_mm.c:1745 TMSI REALLOCATION COMPLETE
```

Chiffrement effectivement négocié sur le canal dédié :
`Channel type 8, subch 0, ts 2, mode 1, cipher 1`.

---

## 3. Appel voix — abouti

Appel **MO** sur TCH/F, monté, connecté, puis raccroché proprement :

```
gsm48_rr.c:2413 CHANNEL REQUEST: e0 (Orig TCH/F)
gsm48_cc.c:195  Sending 'MNCC_SETUP_CNF' to MNCC
                MNCC_CALL_PROC_IND
gsm48_cc.c:242  new state ACTIVE -> DISCONNECT_IND
gsm48_cc.c:1827 sending RELEASE COMPLETE
gsm48_cc.c:242  new state DISCONNECT_IND -> NULL
```

Chaîne audio câblée dans la foulée :

```
DGAPK pq_codec.c:81  Adding codec fr, decoding from format gsm
DGAPK pq_alsa.c:197  Adding ALSA output (dev='gsm_out', blk_len=320)
DGAPK gapk_io.c:304  chain 'source/tch_fb -> format/gsm -> ecu/fr -> format/gsm'
DGAPK gapk_io.c:472  GAPK I/O initialized for MS '1', codec 'fr'
```

`CONNECT` ×13, `CC_DATA_REQ` ×8, `RELEASE` ×8 — le cycle complet.

---

## 4. Le producteur L1 — le contrat que gr-gsm remplit

C'est **la cible de référence** : le DSP natif devra produire cela, aux mêmes
adresses.

| grandeur | valeur | témoin |
|---|---:|---|
| `SCH reel (gr-gsm)` | 37 | `BSIC=7 (ncc=0 bcc=7) FN=33803 TOA=0` |
| `feed_si` → `a_cd` | 663 | `SI réel 23 o injecté → a_cd (L2[0..2]=59 06 1a)` |
| `feed_agch` | 1328 | `IMM-ASS mt=0x21 -> agch_buf` |
| c54x : `d_fb_det 0→1` | 0 | c54x éteint dans ce banc — cohérent |
| c54x : tâche SB | 0 | idem |
| **`DSP Error Status`** | **0** | à comparer aux **404 × « erreur 8 »** du banc `native` |
| déraillement émulateur | 0 | — |
| opcode non implémenté | 0 | — |

Réseau : **3 erreurs BTS** seulement, toutes OML bénignes
(`Manufacturer Dependent State is unsupported`).
Timers MM : `T3110`×4, `T3212`×3, `T3240`×2, `T3230`×2. `ccch mode` : `COMB`×7, `NONE`×4.

---

## 5. SMS — la nuance à ne pas gommer

SMS MT envoyé via `/etc/osmocom/send-mt-sms.sh 001010001000001`. Il **atteint la
couche SMS du mobile puis est abandonné** :

```
gsm0411_smc.c:291 SMC(8) received CP-DATA
gsm0411_smc.c:338 SMC(8) cannot release yet current state: WAIT_CP_ACK
gsm411_sms.c:940  Received 'MMSMS_REL_IND' from MM
gsm411_sms.c:954  MM connection released.
gsm0411_smc.c:109 SMC(8) dropping pending message
```

La connexion MM est libérée **avant le CP-ACK**. Donc `SMS = 1` veut dire
« CP-DATA reçu », **pas** « SMS délivré ». Front ouvert, indépendant du DSP.

---

## 6. Le lancement change tout

Le même mode lancé par la mauvaise porte (`run.sh --restart`, sans
`CALYPSO_BRIDGE=pont`) donne un résultat radicalement différent :

| | `run.sh` (mauvais) | `start-direct.sh` (bon) |
|---|---:|---:|
| LU REQUEST | 19 | 1 |
| **LU ACCEPT** | **0** | **1** |
| TMSI | 0 | 5 |
| timer d'échec | `T3211` ×74 | — |
| erreurs BTS | **1798**, dont `send() failed on TRXD` ×1268 | **3** |

Le `send() failed on TRXD ... Connection refused` en masse est la signature du
mauvais lancement : la BTS ne joint pas le transceiver, le réseau ne répond
jamais, et le LU **expire** au lieu d'être rejeté.

---

## 7. L'instrument et les trois pièges qu'il a coûtés

`tools/rapport-run.sh` :

```bash
tools/rapport-run.sh                        # run courant, LOG_DIR auto-detecte
tools/rapport-run.sh /tmp/ref-shunt-legit   # une photo archivee
tools/rapport-run.sh <dir> <sortie.txt>
```

Trois défauts corrigés en l'écrivant, tous du même genre — **un compteur nul qui
n'était pas une absence** :

1. **motifs faux** — `camp=0` sur un run qui campait 19 fois (`camping` au lieu
   de `camping (normal)` / `camped normally`), et un préfixe `-i` collé au motif
   pris comme littéral. Tous les motifs sont désormais *dérivés des journaux*.
2. **identité empruntée** — sur une photo archivée, lire `/proc/<qemu>/environ`
   décrit le run *vivant*. L'identité vient maintenant du `qemu-manifest.log` du
   run analysé.
3. **mauvais répertoire** — les deux bancs n'écrivent pas au même endroit
   (`run.sh` → `/tmp/calypso/logs`, `start-direct.sh` → `/tmp/osmo-nitb/logs`).
   Le premier rapport croisait le manifeste d'un run avec les journaux d'un
   *autre*, figés depuis six minutes. Le script demande désormais son `LOG_DIR`
   au processus vivant et **signale un journal qui ne grossit pas**.

---

## 8. Ce que ce run fixe pour la suite

L'objectif est que `native` **suive** `shunt_legit` en substituant le DSP à
gr-gsm, **aux mêmes adresses**. Ce run donne donc la cible, mesurée :

- le SCH doit sortir `BSIC=7 (ncc=0 bcc=7)` avec un FN cohérent, ~37 fois / 4 min ;
- `a_cd` doit recevoir les SI réels (663 injections ici) ;
- `d_error_status` doit rester à **0** — le banc `native` en est à **404 × erreur 8**
  (`DSP_ERR_DMA_PROG`) ;
- la chaîne doit tenir jusqu'à `LOCATION UPDATING ACCEPT`, puis TCH/F + GAPK.

Deux fronts ouverts, tous deux **indépendants du DSP** :
la libération MM prématurée qui fait tomber le SMS avant le CP-ACK, et le
`send() failed on TRXD` qui apparaît dès que le lancement n'est pas le bon.
