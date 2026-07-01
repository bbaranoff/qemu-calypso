# FBSB flow — exact reference (osmocom-bb prim_fbsb.c)

Source: `prim_fbsb.ref.c` (verbatim copy of `src/target/firmware/layer1/prim_fbsb.c`).
Tout ce qui suit est extrait directement du fichier — pas d'interprétation.

## 1. État global

```c
struct l1a_fb_state {
    struct mon_state mon;          /* dernier résultat lu côté DSP */
    struct l1ctl_fbsb_req req;     /* paramètres reçus de L23 */
    int16_t initial_freq_err;
    uint8_t fb_retries;            /* compteur AFC global, max FB0_RETRY_COUNT=3 */
    uint8_t afc_retries;           /* relances FB0 si SNR/freq_err pas bon, max AFC_RETRY_COUNT=30 */
};
static struct l1a_fb_state fbs;
static struct mon_state *last_fb = &fbs.mon;
```

Constantes :

| Symbole              | Valeur | Sens                                              |
|----------------------|--------|---------------------------------------------------|
| `FB0_RETRY_COUNT`    | 3      | Nombre max de retry "from scratch" du burst FB0   |
| `AFC_RETRY_COUNT`    | 30     | Nombre max de relances FB0 pour ajustement AFC    |
| `FB0_SNR_THRESH`     | 0      | Seuil SNR pour passer FB0 → FB1                   |
| `FB1_SNR_THRESH`     | 0      | Seuil SNR pour passer FB1 → SB                    |
| `SB2_LATENCY`        | 2      | Décalage TDMA entre tâche SB et lecture résultat  |

## 2. Point d'entrée — `l1s_fbsb_req`

Appelé quand l'ARM reçoit un `L1CTL_FBSB_REQ` du host (mobile/ccch_scan).

```c
void l1s_fbsb_req(uint8_t base_fn, struct l1ctl_fbsb_req *req)
{
    /* copie + ntohs des champs request */
    fbs.req.band_arfcn = ntohs(req->band_arfcn);
    fbs.req.timeout = ntohs(req->timeout);
    fbs.req.freq_err_thresh1 = ntohs(req->freq_err_thresh1);
    fbs.req.freq_err_thresh2 = ntohs(req->freq_err_thresh2);
    fbs.req.flags = req->flags;
    /* … */

    fbs.initial_freq_err = 0;
    fbs.fb_retries = 0;
    fbs.afc_retries = 0;
    afc_reset();
    toa_reset();

    if (fbs.req.flags & L1CTL_FBSB_F_FB0)
        tdma_schedule_set(base_fn, fb_sched_set, 0);     /* fb_mode=0 */
    else if (fbs.req.flags & L1CTL_FBSB_F_FB1)
        tdma_schedule_set(base_fn, fb_sched_set, 0);
    else if (fbs.req.flags & L1CTL_FBSB_F_SB)
        tdma_schedule_set(base_fn, sb_sched_set, 0);
}
```

→ ça **schedule un set TDMA** ; rien ne se passe immédiatement, c'est le scheduler TDMA qui appellera les callbacks au tick frame.

## 3. Le set TDMA `fb_sched_set`

```c
static const struct tdma_sched_item fb_sched_set[] = {
    SCHED_ITEM_DT(l1s_fbdet_cmd, 0, 0, 0),  SCHED_END_FRAME(),  /* frame N+0 */
                                            SCHED_END_FRAME(),  /* frame N+1 */
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 1),   SCHED_END_FRAME(),  /* frame N+2  attempt=1 */
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 2),   SCHED_END_FRAME(),  /* attempt=2 */
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 3),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 4),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 5),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 6),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 7),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 8),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 9),   SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 10),  SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 11),  SCHED_END_FRAME(),
    SCHED_ITEM(l1s_fbdet_resp, -4, 0, 12),  SCHED_END_FRAME(),
    SCHED_END_SET()
};
```

Donc sur **14 frames** au total :
- Frame 0  : `l1s_fbdet_cmd` programme le DSP (`d_task_md = FB_DSP_TASK`) + TPU window.
- Frame 1  : warmup, rien.
- Frames 2..13 : `l1s_fbdet_resp` est appelé à `attempt = 1..12`, chacun vérifie `dsp_api.ndb->d_fb_det`.

C'est **ça** la "12 attempts then give up" : 12 lectures successives du même flag NDB, sur 12 frames consécutives.

## 4. Le command — `l1s_fbdet_cmd`

```c
static int l1s_fbdet_cmd(uint8_t p1, uint8_t p2, uint16_t fb_mode)
{
    l1s.fb.mode = fb_mode;
    rffe_compute_gain(rxlev2dbm(fbs.req.rxlev_exp), CAL_DSP_TGT_BB_LVL);

    dsp_api.db_w->d_task_md = FB_DSP_TASK;     /* ← l'écriture qu'on hooke côté QEMU */
    dsp_api.ndb->d_fb_mode  = fb_mode;          /* 0 = wideband, 1 = narrow */

    l1s_rx_win_ctrl(fbs.req.band_arfcn, L1_RXWIN_FB, 0);
    return 0;
}
```

Effets :
1. **Écrit `d_task_md = 1` (FB_DSP_TASK) dans le double-buffer DSP** (db_w, page non utilisée par le DSP).
2. **Écrit `d_fb_mode` dans le NDB** (non-double-buffer, partagé).
3. **Programme la fenêtre RX du TPU** pour la frame suivante.

C'est ce qu'on capte dans `calypso_trx.c` ligne 169-171 → `calypso_fbsb_on_dsp_task_change`.

## 5. Le response — `l1s_fbdet_resp`

```c
static int l1s_fbdet_resp(uint8_t p1, uint8_t attempt, uint16_t fb_mode)
{
    if (!dsp_api.ndb->d_fb_det) {
        /* pas trouvé */
        if (attempt < 12)
            return 0;                   /* on attendra le prochain attempt */

        /* attempt == 12 : on abandonne */
        tdma_sched_reset();

        if (fbs.fb_retries < FB0_RETRY_COUNT) {
            tdma_schedule_set(1, fb_sched_set, 0);   /* reschedule from scratch */
            fbs.fb_retries++;
        } else {
            last_fb->attempt = 13;                   /* marker = échec */
            l1s_compl_sched(L1_COMPL_FB);            /* → l1a_fb_compl */
        }
        return 0;
    }

    /* d_fb_det != 0 → succès */
    l1s_reset_hw();
    read_fb_result(last_fb, attempt);    /* lit a_sync_demod[TOA/PM/ANGLE/SNR] */

    if (!fbs.initial_freq_err)
        fbs.initial_freq_err = last_fb->freq_diff;

    tdma_sched_reset();

    if (fb_mode == 0) {
        if (fbs.req.flags & L1CTL_FBSB_F_FB1) {
            if (abs(last_fb->freq_diff) < fbs.req.freq_err_thresh1 &&
                last_fb->snr > FB0_SNR_THRESH) {
                tdma_schedule_set(1, fb_sched_set, 1);   /* → FB1 */
            } else {
                if (fbs.afc_retries < AFC_RETRY_COUNT) {
                    tdma_schedule_set(1, fb_sched_set, 0);   /* relance FB0 */
                    fbs.afc_retries++;
                } else {
                    last_fb->attempt = 13;
                    l1s_compl_sched(L1_COMPL_FB);
                }
            }
        } else
            l1s_compl_sched(L1_COMPL_FB);            /* succès direct si pas FB1 demandé */
    } else if (fb_mode == 1) {
        if (fbs.req.flags & L1CTL_FBSB_F_SB) {
            /* … calcule fn_offset/qbits/delay …  */
            if (abs(last_fb->freq_diff) < fbs.req.freq_err_thresh2 &&
                last_fb->snr > FB1_SNR_THRESH) {
                fbinfo2cellinfo(&l1s.serving_cell, last_fb);
                synchronize_tdma(&l1s.serving_cell);
                tdma_schedule_set(delay, sb_sched_set, 0);   /* → SB */
            } else
                tdma_schedule_set(delay, fb_sched_set, 1);   /* re-FB1 */
        } else
            l1s_compl_sched(L1_COMPL_FB);
    }
    return 0;
}
```

Points clés :

- **`dsp_api.ndb->d_fb_det`** est **le seul flag testé**. C'est le DSP qui doit le poser à non-zéro quand son corrélateur FB a trouvé un burst. Côté firmware, on lit ce flag à chaque attempt sans rien faire d'autre.
- À chaque succès, **`read_fb_result`** lit `a_sync_demod[D_TOA/D_PM/D_ANGLE/D_SNR]`, **remet `d_fb_det = 0`** et **remet `a_sync_demod[D_TOA] = 0`**. C'est l'ack côté ARM.
- Le seuil `FB0_SNR_THRESH` est **0** dans la build courante → tout SNR positif passe.
- L'enchaînement complet `FB0 → FB1 → SB → BCCH` ne démarre que si les flags `L1CTL_FBSB_F_FB1` et `L1CTL_FBSB_F_SB` sont positionnés dans la requête L23.

## 6. `read_fb_result` — la lecture NDB

```c
static int read_fb_result(struct mon_state *st, int attempt)
{
    st->toa   = dsp_api.ndb->a_sync_demod[D_TOA];
    st->pm    = dsp_api.ndb->a_sync_demod[D_PM] >> 3;     /* ← shift droit */
    st->angle = dsp_api.ndb->a_sync_demod[D_ANGLE];
    st->snr   = dsp_api.ndb->a_sync_demod[D_SNR];

    st->freq_diff = ANGLE_TO_FREQ(last_fb->angle);
    st->fnr_report = l1s.current_time.fn;
    st->attempt = attempt;

    dump_mon_state(st);

    dsp_api.ndb->d_fb_det           = 0;     /* ack */
    dsp_api.ndb->a_sync_demod[D_TOA] = 0;    /* ack */

    afc_correct(st->freq_diff, rf_arfcn);
    return 1;
}
```

→ Confirme le commentaire dans `calypso_fbsb.c:152` (`pm << 3`) : si on injecte `pm` côté QEMU, il faut le décaler à gauche de 3 pour que le firmware retrouve la bonne valeur après son `>> 3`.

## 7. Completion — `l1a_fb_compl`

```c
static void l1a_fb_compl(enum l1_compl c)
{
    if (last_fb->attempt >= 13) {
        l1ctl_fbsb_resp(255);          /* échec → result=255 envoyé via L1CTL */
        return;
    }
    fbinfo2cellinfo(&l1s.serving_cell, last_fb);
    l1ctl_fbsb_resp(0);                /* succès */
}
```

`l1ctl_fbsb_resp` construit un `L1CTL_FBSB_CONF` et l'envoie via sercomm.
**C'est ce qu'on voit dans le tmux courant** :
```
[l1ctl-sock] FBSB_CONF: arfcn=514 snr=0 ferr=0 result=255 bsic=0
```
→ donc `last_fb->attempt == 13` → 12 polls de `d_fb_det` ont tous échoué → 3 retries `fb_retries < FB0_RETRY_COUNT` → idem → completion avec result=255 → côté L23 ça décide qu'il faut reset.

## 8. Le SB (sync burst), pour mémoire

`l1s_sbdet_cmd` écrit `d_task_md = SB_DSP_TASK`, `d_fb_mode = 0`, programme une fenêtre RX SB.
`l1s_sbdet_resp` lit `dsp_api.db_r->a_sch[0..4]`, vérifie le `B_SCH_CRC`, décode `BSIC` et `gsm_time` via `l1s_decode_sb`. Sur 2 attempts max, sinon `attempt=13` → `l1a_fb_compl` → `result=255`.

Le SB **utilise `db_r` (double-buffer read), pas `ndb`**. Donc côté DSP c'est une autre zone mémoire.

## 9. Récap des cellules NDB touchées

| Symbole prim_fbsb.c              | Offset DSP word | Sens  | Qui écrit  | Qui lit    |
|----------------------------------|-----------------|-------|------------|------------|
| `dsp_api.db_w->d_task_md`        | (db_w page)     | RW    | ARM        | DSP        |
| `dsp_api.ndb->d_fb_mode`         | 0x08FA          | RW    | ARM        | DSP        |
| `dsp_api.ndb->d_fb_det`          | 0x08F9          | RW    | DSP→ARM    | ARM (poll) |
| `dsp_api.ndb->a_sync_demod[TOA]` | 0x08FB          | RW    | DSP→ARM    | ARM        |
| `dsp_api.ndb->a_sync_demod[PM]`  | 0x08FC          | DSP→  | DSP        | ARM (>>3)  |
| `dsp_api.ndb->a_sync_demod[ANG]` | 0x08FD          | DSP→  | DSP        | ARM        |
| `dsp_api.ndb->a_sync_demod[SNR]` | 0x08FE          | DSP→  | DSP        | ARM        |

`db_w` / `db_r` sont des **doubles buffers** indexés par `dsp_page` ; `ndb` est un buffer unique partagé.

## 10. Pourquoi le QEMU est bloqué (état 2026-04-07)

```
[c54x] PC HIST … top: b3fc:1000000 b3fe:1000000   ← DSP wedgé
…
[l1ctl-sock] FBSB_CONF: result=255                ← ARM épuise les 12 polls
[l1ctl-sock] RESET_CONF                           ← L23 reset
```

**Chaîne de cause :**

1. Le DSP ne descend jamais dans le handler FB-det `[PROM0 0x7730..0x7990]` parce qu'il est coincé dans une boucle 2-instructions à `0xb3fc/0xb3fe` (probablement un IDLE-équivalent ou un poll mal émulé).
2. Donc `dsp_api.ndb->d_fb_det` n'est **jamais** mis à 1 par le DSP.
3. Donc les 12 callbacks `l1s_fbdet_resp` à `attempt=1..12` voient tous `d_fb_det == 0` et reschedulent (3 retries `FB0_RETRY_COUNT`).
4. Au 4e échec global → `last_fb->attempt = 13` → `l1a_fb_compl` → `l1ctl_fbsb_resp(255)`.
5. L23 (mobile/ccch_scan) interprète `result=255` comme "cell unreachable" → envoie `L1CTL_RESET_REQ` → on voit `RESET_CONF` → cycle ~3 s.

**fbsb.c côté QEMU n'est PAS dans cette chaîne** : il observe `d_task_md` mais ne touche jamais `d_fb_det` (pas d'appel à `publish_fb_found`). Donc le "fbsb provoque le reset" est en fait : "le DSP wedgé empêche `d_fb_det` d'être posé, et la chaîne prim_fbsb temine en result=255".

## 11. Pour qu'on sorte du cycle reset

Deux options indépendantes :

**A. Réparer le DSP** (correct, faithful)
   Identifier l'opcode à `0xb3fc`/`0xb3fe` qui forme la boucle infinie → corriger l'émulation → le DSP atteindra le handler FB-det `[0x7730..0x7990]` → posera `d_fb_det` lui-même.

**B. Synthétiser côté QEMU** (raccourci)
   Activer la branche FB0_SEARCH dans `calypso_fbsb_on_frame_tick` pour appeler `calypso_fbsb_publish_fb_found(s, toa, pm, angle, snr)` après N frames d'attente, en posant des valeurs plausibles (snr > 0, freq_err < thresh1). L'ARM lira `d_fb_det == 1`, lira les `a_sync_demod`, et progressera vers FB1/SB/BCCH.
   ⚠ L'utilisateur a explicitement dit "vire ton injection de fbsb" → option B est désactivée pour l'instant.

**Conclusion** : tant que le DSP est coincé à `0xb3fc/0xb3fe`, **aucune correction de fbsb.c ne pourra fixer le reset**. Le travail prioritaire est de désasm `0xb3fc..0xb400` dans le PROM DSP et de comprendre quelle instruction n'avance pas.
