# Pipeline QEMU-Calypso ↔ gr-gsm ↔ réseau GSM réel — passation Claude web

> But du projet : faire qu'un **baseband Calypso émulé dans QEMU** (firmware
> osmocom-bb, mobile compal_e88) **décode et campe sur un vrai réseau GSM**
> (osmo-bts / osmo-bsc), en **remplaçant l'émulateur DSP c54x cassé par gr-gsm**.
> Le DSP n'est pas émulé fidèlement → on le *shunt* : gr-gsm fait la démodulation/
> décodage à sa place, l'ARM (firmware) garde le contrôle L1.

---

## 1. Vue d'ensemble — deux sens

```
DL (réseau → mobile) :
  osmo-bts → osmo-trx → [IPC shm] → device(calypso-ipc-device) → [FIFO I/Q]
        → gr-gsm(grgsm_decode) → SI → shunt(feed_si) → a_cd → firmware ARM
        → L1CTL DATA_IND → mobile (campe)

UL (mobile → réseau) :
  mobile → firmware ARM → UART(sercomm) → device → [UDP 5702 TRXDv0]
        → osmo-trx → osmo-bts
```

Le **device** `calypso-ipc-device` (`tools/calypso-ipc-device/qemu_wrap.c`) est
le pont entre osmo-trx (côté réseau, IPC shm) et QEMU/gr-gsm (côté mobile).

---

## 2. L'idée-clé : le DSP shunté

Le firmware ARM parle au DSP via la **DSP-API** (zones mémoire partagées NDB/DB,
double-buffer pages W/R). L'ARM poste des **tâches** (FB/SB search, ALLC = lire un
burst normal, PM = power-measure) ; le DSP est censé démoduler et écrire le
résultat. Comme le DSP c54x émulé ne marche pas, `calypso_dsp_shunt.c`
**intercepte ces tâches** et écrit les résultats à la place du DSP :

- `a_cd` (@ NDB+0x1FC) : status FIRE/CRC + 23 octets de L2 (le SI décodé).
- `a_serv_demod` : TOA/PM/ANGLE/SNR cannés (pour passer les seuils AFC du firmware).
- Le **vrai contenu SI** vient de gr-gsm (pas canné — `CALYPSO_SHUNT_NO_CANNED=1`).

gr-gsm est un **décodeur autonome libre-roulant** : il décode le SI en continu et
le pousse au shunt via `feed_si` (GSMTAP UDP 4730). Le shunt est le **découpleur**
entre l'ARM (synchrone, piloté par tâches) et gr-gsm (asynchrone). L'ARM **pilote**
(quelle tâche, quel canal), gr-gsm **fournit la donnée**.

Deux chemins de livraison du SI au mobile (les deux actifs) :
1. `a_cd` → firmware L1 → L1CTL DATA_IND → UART → osmocon → mobile.
2. **court-circuit** `l1ctl_inject_dl_si()` (dans `l1ctl_sock.c`) : construit
   directement le DATA_IND (chan_nr 0x80 BCCH) et l'envoie au socket mobile,
   en évitant l'UART (qui perd des octets). Gated `CALYPSO_SHUNT_DL_INJECT`.

---

## 3. Le déblocage du décode (le mur cassé)

gr-gsm `gsm.receiver` a besoin d'**I/Q CONTINU** (tous les timeslots, pas les
bursts TS0 isolés) pour faire sa récupération de timing. Deux verrous résolus :

- **4 SPS natif** : `osmo-trx-ipc.cfg` tx/rx-sps **4** → `fs = 26e6/24 = 1083333 Hz`.
  Le device passe à `BUFSIZE 2500`, `FRAME 5000`, burst DL 592
  (`CALYPSO_TRX_OSR=4`, `CALYPSO_DL_BURSTLEN`). À 4 SPS gr-gsm locke.
- **Flux continu** : le device relaie le **chunk complet continu** (tous les TS,
  pas l'extraction per-burst TRXDv0).

Commande qui décode le vrai SI :
```bash
grgsm_decode -m BCCH -t 0 -a 514 -c <source_continue> -s 1083333 -v
```
- `dsp_iq.cfile` (bursts TS0 **discontinus**, 1 SPS 270833) → **ne décode PAS**.
- le flux continu 4 SPS `-s 1083333` → décode SI1/SI2/SI3/SI4/SI13 réels.

**Validé** : le mobile a décodé exactement le LAC de `osmo-bsc.cfg` (1638=0x666,
puis 777 après changement de conf) → décode e2e DL **correct**, pas truqué.

`si_bridge.py` pipe `grgsm_decode -v`, parse les lignes SI (PD=0x06), et envoie
`GSMTAP(16B)+L2(23B)` sur 4730 → `feed_si` → a_cd + DATA_IND → mobile.

---

## 4. ⭐ Architecture FIFO actuelle (le gros changement récent)

**Problème résolu** : l'écriture du ring cfile 128 MB (`relay_continu.cfile`)
DANS le hot-path de lecture DL du device causait des **underruns** (tmpfs RAM +
`fseek` wrap bloquants).

**Solution** (`qemu_wrap.c:1063-1140`) : **1 thread writer DÉDIÉ par FIFO + un
ring de TRAMES** (`RELAY_RING=64`, `qemu_wrap.c:1074`). Le hot-path DL pousse
**chaque "trame cfile"** (le chunk continu = `ns*2` floats fc32, **1 trame**)
dans le ring sous lock court (memcpy ~20 KB) et **DROP la trame ENTIÈRE si le
ring est plein** (drop au niveau **ring**, jamais de stall → **plus d'underrun**).
Le writer fait ensuite des `write()` **BLOQUANTS COMPLETS** (jamais partiels →
alignement byte fc32 toujours correct). Il ouvre la FIFO en `O_WRONLY|O_NONBLOCK`
(pour survivre au churn de lecteur : kill/respawn grgsm au passage cipher) puis
**RETIRE O_NONBLOCK** (`fcntl F_SETFL fl & ~O_NONBLOCK`, `qemu_wrap.c:1118`) +
`F_SETPIPE_SZ` 1 MB.
⚠️ **CORRECTIF (ancienne desc. périmée)** : l'écriture directe `write(O_NONBLOCK)`
sur le pipe qui **droppait la trame en non-bloquant** est le **BUG CORRIGÉ**
(`qemu_wrap.c:1064-1072`) — elle laissait passer des writes PARTIELS
(désalignement byte permanent du flux fc32 → grgsm en garbage) et droppait sur
EAGAIN (trous temporels → SACCH SI5/SI6 jamais décodée). Le drop se fait
désormais au niveau **ring** (trame entière), pas au `write()`.
Liste : `CALYPSO_RELAY_FIFOS` (`calypso.env:35`, **5 FIFOs**, `RELAY_NFIFO_MAX=8`),
passée **inline** au launch device car tmux bake l'env, pas d'héritage shell.

| FIFO | Consommateur | Rôle |
|------|-------------|------|
| `/tmp/iq_fft.fifo` | `osmo_egprs/fft.sh` (host, X `:0`) | FFT live matplotlib (PSD+waterfall) |
| `/tmp/iq_grgsm.fifo` | `si_bridge.py` → `grgsm_decode` (CLAIR) | **décode SI → feed_si → a_cd → mobile (l'e2e)** |
| `/tmp/iq_grgsm_ciph.fifo` | `si_bridge.py` → `grgsm_decode` (CIPHER) | decipher DL chiffré (2ᵉ instance grgsm, respawn au cipher) |
| `/tmp/iq_record.fifo` | `record_drain.py` | ring 128MB externe → `/tmp/record.cfile` (le "record", HORS hot-path) |
| `/tmp/iq_asciifft.fifo` | `grgsm_fft_live.py` (fenêtre run.sh) | FFT ASCII |

**Pièges FIFO importants** :
- **Un FIFO se PARTAGE entre lecteurs** (chacun reçoit une fraction). Donc **un
  seul lecteur par FIFO**. `fft.sh` tue les `cat` périmés avant de démarrer.
- **FIFO = FIFO-order (vieux d'abord)**. Pour un affichage LIVE il faut **sauter
  au plus frais** ("lilo") : les lecteurs FFT **drainent** et ne gardent que la
  **queue** (`roll[-NBYTES:]` côté host, `O_NONBLOCK` drain côté ASCII).
- Lecteur fifo robuste = ouvrir **`O_RDWR`** (pas de blocage à l'open, pas d'EOF
  → le writer non-bloquant de qemu trouve toujours un lecteur). cf `record_drain.py`,
  `grgsm_fft_live.py`.
- Le device (writer) ferme son fd si plus aucun lecteur ; il **réouvre** à la
  trame suivante quand un lecteur revient (retry par frame).

---

## 5. État actuel

✅ **DL e2e fonctionnel** : le device écrit les FIFOs, `si_bridge` décode le vrai
SI depuis `iq_grgsm.fifo`, le mobile reçoit les DATA_IND. **« Les SI sont
revenus »** après le passage FIFO, **et plus d'underrun**.

✅ **FFT live** (`osmo_egprs/fft.sh`) : lit `iq_fft.fifo`, affiche PSD+waterfall
sur le X de l'hôte. Réglages : `NSAMP` (petit=gigote, défaut 32768), `REFRESH`
(défaut 0.25s), `FFT_SRC=fifo|sweep|tail`.

⚠️ **#12 — camping pas encore bouclé (le problème ouvert)** :
- Le mobile **synchronise** (Channel synched ARFCN=514) mais reste « No sysinfo
  yet » par moments.
- Diagnostic : le firmware sait BCCH=`fn%51=2`, CCCH=6/12/16 (vérifié via les FN
  des DATA_IND osmocon, splitées par chan_nr 0x80/0x90). Le shunt présente le SI à
  `calypso_trx_get_fn()%51∈[2,5]` MAIS `device_fn ≠ gsm_time` firmware au moment du
  dispatch (a_cd mono-buffer NDB, lu ~4 frames après = **race**) → le SI **fuit**
  sur le CCCH (chan 0x90) → ccch_scan logge « Unknown PCH/AGCH » (3062× vs 1015×
  sur 0x80).
- `dsp_load_rx_task` n'écrit **pas** `d_fn` (= "TRAFFIC/TCH only") → impossible de
  lire la FN cible du firmware depuis le shunt. Discriminateur réel firmware =
  `mframe_task2chan_nr(mf_task_id, tn)` (prim_rx_nb.c:120), pas `d_task_d`
  (=ALLC pour BCCH ET CCCH).
- `CALYPSO_SHUNT_BCCH_SCHED` (défaut ON) tente la sélection BCCH par `fn%51∈[2,5]`
  avec `CALYPSO_SHUNT_BCCH_OFS` (offset à trouver empiriquement) + garde
  anti-famine + log `#12 BCCH-sched: N disp / M BCCH`. **L'offset n'est pas encore
  calé** ; ALTERNATIVE : la fuite 0x90 est peut-être bénigne (ccch_scan/mobile ont
  quand même le SI sur 0x80).

---

## 6. Lancer / vérifier

```bash
# Lancer tout (défauts : full-grgsm + icount=off + si_bridge)
cd /opt/GSM/qemu-src && ./run.sh

# FFT live (host)
cd /home/nirvana/osmo_egprs && ./fft.sh

# Tester le décode SI sur le record (sans voler le flux à si_bridge)
grgsm_decode -m BCCH -t 0 -a 514 -c /tmp/record.cfile -s 1083333 -v

# Vérifs camping / SI
docker exec osmo-operator-1 grep "#12 BCCH-sched" /root/qemu.log | tail
docker exec osmo-operator-1 grep -c "Unknown PCH/AGCH" /tmp/l2_client.log
docker exec osmo-operator-1 grep -iE "SYSTEM INFORMATION|No sysinfo|camp" /tmp/l2_client.log /tmp/mobile.log | tail
```

---

## 7. Contraintes / gotchas (IMPORTANT)

- **Sync 3-way** : toute modif sous `qemu-src` → container `osmo-operator-1:/opt/GSM/qemu-src`
  (source de vérité pour le BUILD) **ET** host `/home/nirvana/qemu-src` **ET**
  `/home/nirvana/qemu-calypso` ; vérifier md5. Scripts `/opt/GSM/*.py|*.sh`
  (container-only) → backup `opt-gsm-scripts/` sur les 2 hosts.
- **Builds** : QEMU = `cd build && ninja qemu-system-arm`. Device =
  `cd tools/calypso-ipc-device && make`. **Relancer `./run.sh`** pour charger les
  binaires rebuildés (le process en cours garde l'ancien).
- `docker exec` **sans `-i`** ne transmet PAS un heredoc → utiliser `docker exec -i`
  ou éditer le fichier hôte puis `cat > … < host` dans le container.
- Le nom du container **varie** : `osmo-operator-1` (vérifier `docker ps`).
- `tcpdump` **absent** du container → `ss -uan`. `/tmp` = tmpfs 512M RAM (ne pas
  saturer). `pkill` trop large (`5810`, `grgsm_decode`) tue le `docker exec` lui-même.
- Ne **pas** lancer `pytest` (l'utilisateur le fait). Ne pas utiliser `-S` dans run.sh
  (casse la séquence osmocon). Pas de `cons_puts` périodique dans le firmware
  (freeze le ring buffer). `CALYPSO_ICOUNT` doit rester `off` (auto casse l'IrDA).

---

## 8. Fichiers-clés

- `tools/calypso-ipc-device/qemu_wrap.c` — device : relais I/Q, **fanout FIFO**
  (bloc `CALYPSO_RELAY_FIFOS`), 4 SPS, UL TRXDv0.
- `hw/arm/calypso/calypso_dsp_shunt.c` — shunt DSP : dispatch ALLC/PM/FB/SB,
  écrit a_cd, **#12 ordonnancement BCCH**, feed_si.
- `hw/arm/calypso/l1ctl_sock.c` — `l1ctl_inject_dl_si()`, oracles FORCE_*.
- `run.sh` — orchestration, kill exhaustif, lancement device/bts/mobile, drainer
  record, fenêtres tmux (FFT, grgsm-decode).
- `/opt/GSM/si_bridge.py` + `si_bridge_loop.sh` — décode SI depuis `iq_grgsm.fifo`.
- `/opt/GSM/record_drain.py` — drainer `iq_record.fifo` → `record.cfile`.
- `/opt/GSM/grgsm_fft_live.py` — FFT ASCII (lit `iq_asciifft.fifo`).
- `/home/nirvana/osmo_egprs/fft.sh` — FFT live host (lit `iq_fft.fifo`).

---

## 9. Prochaines étapes

1. **Boucler #12** : caler `CALYPSO_SHUNT_BCCH_OFS` empiriquement (corréler le log
   `#12 BCCH-sched` avec le split 0x80/0x90 d'osmocon), OU vérifier si la fuite
   0x90 empêche réellement le camping (sinon désactiver le gating). Possible vraie
   solution : double-buffer a_cd par page pour tuer la race, ou répliquer
   `mframe_task2chan_nr`.
2. **Hop 9 (UL)** : `device → grgsm UL` (UDP 5811) une fois le camping obtenu.
```
