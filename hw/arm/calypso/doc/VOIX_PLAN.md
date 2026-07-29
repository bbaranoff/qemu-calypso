# PLAN D'IMPLÉMENTATION VOIX (TCH/F FR) — méthode SHUNT host-side

But : faire passer la voix full-rate (TCH/F, GSM 06.10) contre notre stack, même
méthode que FB/SI/SDCCH (sonde → mapping exact API-RAM → présentation host-side),
sans que le DSP émulé fasse le codec/corrélation.

Base confirmée (lecture code runtime `${QEMU_TREE}` + firmware `${GSM_ROOT}/osmocom-bb`).

---

## 0. Ce qui existe DÉJÀ (ne pas réécrire)

- **TCH/F DL traffic sub0** est câblé, en écriture seule :
  - `calypso_tch_dl_poll()` — `hw/arm/calypso/calypso_dsp_shunt.c:333-350` : lit le
    sideband `/dev/shm/calypso_tch_dl` (layout 48o `seq@0 fn@4 fr[33]@8`, consume-once par `seq`).
  - `shunt_dispatch_tch_dl()` — `calypso_dsp_shunt.c:356-369` : écrit `a_dd_0`
    (`BASE_API_NDB + NDB_A_DD_0=0x238`), `[0]=1<<B_BLUD(15)`, `[2]=0`, 33o packés **BIG-ENDIAN** @`[3]`.
  - Routage : `calypso_dsp_shunt.c:836-837`, `else if ((td & 0x7FFF) == TCHT_DSP_TASK(13))` → dispatch.
  - Côté firmware la cible est bonne : `prim_tch.c:322` `traffic_buf = tch_sub? a_dd_1 : a_dd_0`,
    remonté `L1CTL_TRAFFIC_IND` si `[0]&B_BLUD` à `((fn%13)%4)==3` (`prim_tch.c:307,361`).
- **Capture UL générique** : `shunt_latch_task` — `calypso_dsp_shunt.c:177` `if (g_shunt.d_task_u != 0)`
  lit `a_cu`@`0x264` en 23o LAPDm et publie via `calypso_sdcch_ul_publish()` (`:96-127`) sur
  `/dev/shm/calypso_sdcch_ul`. **⚠ ce bloc ne discrimine PAS le type de task_u** : il traite
  TOUT `d_task_u != 0` comme du SDCCH/SACCH 23o. C'est le défaut central (§2).
- Host : audio_mode/GAPK/FR compilés (`WITH_GAPK_IO=1`), routage DL `l1ctl.c:842 → tch_recv_cb`
  (`tch.c:49`), UL `tch_send_msg → gsm48_rr_tx_traffic → l1ctl.c:875`. GAPK dépend d'ALSA→Pulse (absent headless).

---

## 1. Chaîne minimale — quoi câbler, dans l'ordre

### Chaîne DL (entendre l'appelant)  — le DERNIER maillon, pas le premier
Ordre de dépendance réel (un TCH ne "tient" pas sans sa signalisation d'accompagnement) :

1. **SACCH DL/UL sur TCH** (task `TCHA=14`) — sinon **radio-link-timeout** : le BTS coupe l'appel.
   - DL : firmware lit `a_cd`@`0x1FC` (réutilise l'array CCCH/SDCCH), 23o, à `burst_id==3` (`prim_tch.c:667`).
   - UL : firmware écrit `a_cu`@`0x264`, 23o, `burst_id==0` (mesures L1+L3) (`prim_tch.c:729`).
2. **FACCH DL** (task `TCHT`, vol de trame) — porte ASSIGNMENT/MODE-MODIFY/HANDOVER/RELEASE
   pendant l'appel : firmware lit `a_fd`@`0x21A`, 23o **LITTLE-ENDIAN** (`dsp_memcpy_from_api(...,23,0)`,
   `prim_tch.c:246,285`), `link_id=0x00`, à `((fn%13)%4)==3`.
3. **Producteur du sideband DL** (§3d) — décoder le RTP/bursts DL → FR 33o → `/dev/shm/calypso_tch_dl`.
   Sans lui, `shunt_dispatch_tch_dl` déjà câblé n'a que du silence/tone de test (`tch_dl_inject.py`).

### Chaîne UL (être entendu)
4. **FACCH UL** — porte **ASSIGNMENT COMPLETE** (verrou #1, §2) : firmware écrit `a_fu`@`0x282`,
   23o, à `((fn%13)%4)==3` (`prim_tch.c:422`). À capturer côté shunt et injecter sur le **TS assigné**.
5. **TCH traffic UL** — firmware écrit `a_du_1`@`0x134` (⚠ sub0 = a_du_1, PAS a_du_0), 33o @`[3]`,
   `[0]=1<<B_BLUD`, pose `d_tch_mode|=B_PLAY_UL`, à `((fn%13)%4)==3` (`prim_tch.c:485,499`).
   À capturer et publier vers un nouveau sideband `/dev/shm/calypso_tch_ul`.
6. **Mur UL RF** (§2/§5) : la voie montante `qemu_wrap.c` est câblée en dur **TS0/RACH** ; il faut
   ouvrir la fenêtre d'injection sur le **TS TCH assigné (TS2)** avec framing burst normal (FACCH+voix).

Prérequis host (déjà OK) : `audio_mode` doit contenir `AUDIO_RX_TRAFFIC_IND(1<<3)|AUDIO_TX_TRAFFIC_REQ(1<<1)`
— obtenu par tout io-handler sauf `l1phy`/`none` (`tch_voice.c:120,130`). Sinon `l1s_tch_resp:325` et
`l1s_tch_cmd:481` skippent tout.

---

## 2. POINT DE BLOCAGE #1 (après ASSIGNMENT COMMAND)

**ASSIGNMENT COMPLETE sur FACCH montant jamais capturé/injecté → expiry T3107 côté BSC → assignment failure.**

Double cause, les deux confirmées dans le code :

**(A) Le shunt lit la MAUVAISE cellule API-RAM pour un task_u TCH.**
`calypso_dsp_shunt.c:177` `if (g_shunt.d_task_u != 0)` lit inconditionnellement `a_cu`@`0x264` (SDCCH/SACCH
23o). Mais à l'assignment, le firmware écrit l'ASSIGNMENT COMPLETE dans **`a_fu`@`0x282`** (FACCH UL, `prim_tch.c:422`),
et la voix UL dans **`a_du_1`@`0x134`** (`prim_tch.c:485`). → le shunt capture `0x264` (vide/idle) et
rate la FACCH. **Fonction:ligne exacte du bug : `calypso_dsp_shunt.c:177`** (bloc UL monolithique non routé par `d_task_u`).

**(B) La voie RF montante est câblée en dur sur TS0/RACH.**
`tools/calypso-ipc-device/qemu_wrap.c:1194` (« un chunk TS0 (`ts%1250==0`) »), gate injection
`(d->rx_ts % CALYPSO_FRAME_SAMPLES)==0` (`:1258,1285`), `ul_slotoff=1875` = TOA RACH (`:1205`),
waveform `g_rach_iq`. TCH/F est assigné **TS2** (osmo-bsc.cfg) → aucune fenêtre d'injection UL pour TS2
→ même une FACCH correctement lue n'atteindrait pas osmo-bts.

Symptôme attendu : BSC émet ASSIGNMENT COMMAND, arme **T3107**, ne reçoit jamais SABM/UA ni ASSIGNMENT
COMPLETE sur la FACCH → **T3107 expiry → assignment failure** → l'appel retombe.

---

## 3. 1ER PATCH CONCRET (le plus petit testable)

**Router la capture UL par `d_task_u` et publier la FACCH/voix TCH.** Change (A) seul d'abord —
purement host-side/API-RAM, calqué sur le `a_cu` existant, testable via GSMTAP `lchan facch/f` sans toucher au RF.

Fichier : `hw/arm/calypso/calypso_dsp_shunt.c`, fonction `shunt_latch_task`, autour de `:177`.

Pseudo-diff :

```c
/* AVANT (:177) : bloc monolithique */
if (g_shunt.d_task_u != 0) {
    /* ... lit a_cu@0x264, 23o, scan LAPDm, calypso_sdcch_ul_publish() ... */
}

/* APRÈS : switch sur (g_shunt.d_task_u & 0x7FFF) */
uint16_t tu = g_shunt.d_task_u & 0x7FFF;
if (tu == DUL_DSP_TASK /*12*/ || tu == TCHA_DSP_TASK /*14*/) {
    /* SDCCH UL, et SACCH-UL-sur-TCH : même cellule a_cu@0x264, 23o, LAPDm.
       -> chemin existant INCHANGÉ (calypso_sdcch_ul_publish). TCHA réutilise tel quel. */
    ...code actuel...
} else if (tu == TCHT_DSP_TASK /*13*/) {
    /* FACCH UL prioritaire (porte ASSIGNMENT COMPLETE, mode-modify) : a_fu@0x282, 23o.
       Header 3 mots [0..2], data @[3]=octet 6. B_BLUD(15) = présent. Packing LE (comme a_cd). */
    uint32_t fu = BASE_API_NDB + 0x282u;
    if (shunt_read_w(fu) & (1u<<15)) {
        uint8_t l2[23];
        for (int i=0;i<23;i+=2){uint16_t w=shunt_read_w(fu+6+i);
            l2[i]=w&0xff; if(i+1<23)l2[i+1]=(w>>8)&0xff;}
        calypso_sdcch_ul_publish(l2, g_shunt.d_task_u, g_shunt.d_fn, shunt_l1s_fn());
        /* réutilise le sideband existant : qemu_wrap encode xcch/burst normal.
           NB : FACCH = même codage bloc 456b que xcch -> encodeur SDCCH réutilisable. */
    }
    /* Voix UL : a_du_1@0x134, 33o, B_BLUD -> nouveau sideband /dev/shm/calypso_tch_ul
       (fonction calypso_tch_ul_publish() calquée sur calypso_sdcch_ul_publish, layout
       48o+ : seq/fn/task_u/fr[33]). Facultatif au 1er patch (audio), la FACCH suffit à
       débloquer l'assignment. */
}
```

Constantes à ajouter (`include/hw/arm/calypso/calypso_dsp_internal.h`) :
`NDB_A_FU=0x282`, `NDB_A_FD=0x21A`, `NDB_A_DU_1=0x134` (déjà présent), `NDB_A_DU_0=0x2A0`, `NDB_A_DD_1=0x108`.

**Testable quand** : lancer `call 1 <num>` VTY, sonder GSMTAP `lchan facch/f` UL : l'ASSIGNMENT COMPLETE
doit apparaître (même si le BTS ne la reçoit pas encore faute de RF TS2 — voir Phase 2). Vérifier aussi
côté BSC log l'absence/présence de T3107 expiry après ouverture TS2.

**Le vrai bout-à-bout** exige AUSSI de lever (B) : étendre la fenêtre d'injection UL de `qemu_wrap.c`
(`:1194,1258,1285`) du TS0-only vers le TS TCH assigné (TS2), avec `ul_slotoff` non-RACH et framing
burst normal. C'est plus gros → Phase 2.

---

## 4. ROADMAP PHASÉE

| Phase | Ce qu'on câble | Fichier:ligne | Testable quand | Risque |
|---|---|---|---|---|
| **P0** Routage UL par task | switch `d_task_u` : FACCH UL `a_fu`@0x282 lu+publié ; TCHA→a_cu inchangé | `calypso_dsp_shunt.c:177` (+`.h` offsets) | GSMTAP `lchan facch/f` UL montre ASSIGNMENT COMPLETE | faible — calqué sur a_cu, host-only, réversible |
| **P1** RF UL sur TS assigné | fenêtre injection TS0→TS_TCH, slotoff non-RACH, burst normal FACCH | `qemu_wrap.c:1194,1205,1258,1285` | BSC ne fait plus T3107 expiry ; call passe en état ACTIVE | **élevé** — casse potentiel RACH TS0 existant (RACH-DET) ; sync TOA multi-slot non validé |
| **P2** SACCH DL/UL sur TCH | dispatch `TCHA=14` : DL→a_cd@0x1FC (LE), UL déjà via P0 | `calypso_dsp_shunt.c:838` (nouvelle branche) + `prim_tch.c:667,729` | appel tient >~10s sans radio-link-timeout | moyen — power-loop/TA simulés, valeurs meas à forger |
| **P3** FACCH DL | `shunt_dispatch_facch_dl()` → a_fd@0x21A, 23o **LE**, `[0]=B_BLUD` | `calypso_dsp_shunt.c:356` (calque) + routage `:836` | mode-modify/handover DL reçus pendant l'appel | moyen — packing LE≠BE traffic, à ne pas confondre |
| **P4** Producteur DL voix | RTP MGW / bursts DL → `gsm0503_tch_fr_decode` → `/dev/shm/calypso_tch_dl` | `qemu_wrap.c` nouveau writer (symétrique `calypso_sdcch_ul_read:880`) + `tch_dl_inject.py` | `L1CTL_TRAFFIC_IND` réel remonte au host (tone→voix) | moyen — décodage FR 8-bursts, alignement fn%13 |
| **P5** UL voix | capture `a_du_1`@0x134 33o → `calypso_tch_ul_publish` → sideband → encode FR UL | `calypso_dsp_shunt.c` (P0 étend) + `qemu_wrap.c` encodeur FR UL | voix UL démodulée par osmo-bts | élevé — encodeur TCH/F UL inexistant, dépend P1 |
| **P6** Audio réel | io-handler `loopback` puis `mncc-sock` puis GAPK/ALSA→Pulse | `mobile_*.cfg` `io-handler` | echo (loopback), puis fichier (mncc-sock), puis son | dépend infra Pulse (absente headless) |

Chemin critique bout-en-bout : **P0 → P1 → P2** (l'appel tient) **→ P4/P5** (audio). P3/P6 en parallèle.

---

## 5. PRÉREQUIS CONFIG (état vérifié — rien à changer côté réseau)

- **osmo-bsc** (`/etc/osmocom/osmo-bsc.cfg`) : BTS0 TS2..7 = TCH/F (`:239-255`), BTS1 TS1..7 = TCH/F.
  `codec-support fr` (`:154`) → **FR uniquement** (pas EFR/AMR/HR côté BTS). ✅ OK pour FR.
- **osmo-msc** : `default-codec tch-f fr` (`:112`) → négocie TCH/F FR (GSM 06.10). ✅
- **osmo-mgw** : RTP loopback 127.0.0.1 ports 4002-16001, `rtp-patch ssrc/timestamp`, payload 98/GSM,
  `allow-transcoding`. ✅ l'audio réseau circule déjà en RTP entre BTS(fake)/MGW.
- **mobile** (`mobile_faketrx_bts1.cfg`) : `channel-capability sdcch+tchf+tchh`, `codec full-speed prefer`,
  `io-handler gapk` + `alsa-output-dev gsm_out`. ⚠ GAPK suppose PulseAudio sink `gsm_audio` actif
  (`/proc/asound/` vide, ALSA→Pulse via `/etc/asound.conf`). **Pour valider le transport sans son :
  basculer `io-handler loopback` (echo) puis `mncc-sock`.** Aucune modif réseau requise.

Rien de bloquant en config : le timeslot TCH/F, le codec FR et le RTP sont déjà en place. Le travail est
100 % côté shunt (API-RAM) + voie RF UL `qemu_wrap.c`.

---

## 6. HONNÊTETÉ — audible réel vs simulé

- **P0–P3 (signalisation) = RÉEL.** FACCH/SACCH sont de la vraie LAPDm/L3 : l'ASSIGNMENT COMPLETE, le
  mode-modify, les mesures remontent réellement à/du réseau. Débloque l'établissement de l'appel pour de vrai.
- **P4 DL voix = RÉEL SI on branche le RTP MGW.** Si on garde `tch_dl_inject.py` (tone/silence), c'est
  **simulé** (bip de test), pas la voix de l'appelant. Le maillon "réel" = décoder le RTP DL descendant du
  MGW en FR 33o. Le transport L1CTL_TRAFFIC_IND lui est réel dès qu'un producteur alimente le sideband.
- **P5 UL voix = le plus fragile.** Il n'existe **aucun encodeur TCH/F UL** ni voie RF multi-slot validée
  (seule la RACH UL TS0 a été prouvée live : RACH-DET). Tant que P1+P5 ne sont pas faits, **la voix qu'on
  émet n'atteint PAS le réseau** — l'appelant distant n'entend rien. On peut être "entendu" uniquement en
  loopback local (echo host-side).
- **Attention piège documenté** : le "SMS MO réel" passe par le relais host-side
  (`sms-interop-relay.py`, `CALYPSO_IPC_RELAY`), PAS par démodulation de bursts UL. Ne pas en déduire
  qu'un chemin UL data multi-slot existe déjà. Pour la voix, la vraie démodulation UL TS2 reste à bâtir (P1/P5).
- **Verdict pragmatique** : avec **P0+P1+P2+P4(RTP)** on a un appel qui s'établit, tient, et où **on entend
  l'appelant** (DL réel). "Être entendu" (UL réel) dépend de P1+P5 et reste le risque majeur ; à défaut,
  valider d'abord en **loopback** (transport TCH prouvé) avant de prétendre à un bidirectionnel réseau.

---

### Fichiers clés (absolus)
- Shunt : `${QEMU_TREE}/hw/arm/calypso/calypso_dsp_shunt.c` (`:177` capture UL, `:333/356` DL TCH, `:836` routage, `:96` publish)
- Helper : `${QEMU_TREE}/hw/arm/calypso/calypso_dsp_helper.c` (`shunt_dispatch_allc`)
- Offsets : `${QEMU_TREE}/include/hw/arm/calypso/calypso_dsp_internal.h`
- RF UL : `${QEMU_TREE}/tools/calypso-ipc-device/qemu_wrap.c` (`:1194,1205,1258,1285`, reader `:880`), `tch_dl_inject.py`
- Firmware TCH : `${GSM_ROOT}/osmocom-bb/src/target/firmware/layer1/prim_tch.c` (RX `:234-345`, UL `:422/485`, SACCH `:667/729`), `mframe_sched.c:228-269`, `l23_api.c:254-303`
- Host : `${GSM_ROOT}/osmocom-bb/src/host/layer23/src/mobile/tch.c`, `tch_voice.c`, `gapk_io.c`, `gsm48_rr.c:4768/4041/4713`, `common/l1ctl.c:445/817/842/875`
