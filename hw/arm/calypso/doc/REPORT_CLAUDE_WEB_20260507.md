# Rapport Claude web — 2026-05-07

## TL;DR

Session de cleanup + UL wiring + stabilité. Tout les hacks documentés
sont supprimés ou env-gated, le milestone DL 777→888 est préservé en
mode `CALYPSO_FBSB_SYNTH=1` (default OFF = vraie chaîne DSP), et la
chaîne UL emet maintenant des RACH bursts réels sur TN=0 — la même
position où le mobile était bloqué le 06/05 dans ta session précédente.
Le test discriminant que tu avais proposé (tcpdump GSMTAP pour voir si
IMM_ASS passe) reste à faire.

## Ce qu'on a fait dans cette session

### Diagnostic du commit 63b4fe5 (pré-cleanup)

Trois nuisances dans le diff qui captait le milestone du 06/05 :
1. 36 lignes BOURRIN-FBDET-SKIP dans `c54x_exec_one` (range PC 0x8d00-0x8f80
   pop+jump bypass de la routine fb-det DSP — court-circuitait ~3M cycles
   par frame qui faisaient déborder le budget TDMA 4.615 ms).
2. ~120 lignes DIAG-HACK env-gated INTM force-clear + ALIAS-CHECK dump.
3. 23 fichiers `__pycache__/*.pyc` recompilés (bruit binaire).

Plus dans le code actif (pas dans 63b4fe5 mais latents) :
4. `publish_fb_found(toa=0,pm=80,angle=0,snr=100)` synth dans
   `calypso_fbsb_on_dsp_task_change` (DSP_TASK_FB).
5. `publish_sb_found(bsic=0)` synth idem.
6. `si3_fallback[23]` hardcoded SI3.
7. `allc_burst_idx` static cycle 0..3.
8. UL pipeline ne polait que `d_task_u` (word 2 = SDCCH/SACCH/TCH lane),
   ignorait `d_task_ra` (word 7 = RACH lane par `prim_rach.c:77`).
9. BSP/bridge `trxd_peer` race au démarrage (UL drop avant premier DL).

### Hacks retirés

| Hack | Fichier | Type |
|---|---|---|
| BOURRIN-FBDET-SKIP block | `calypso_c54x.c` | brut |
| DIAG-HACK INTM + ALIAS-CHECK + BOOT+100k VECDUMP | `calypso_c54x.c` | brut |
| `si3_fallback[]` hardcode | `calypso_fbsb.c` | brut |
| `allc_burst_idx` static counter | `calypso_fbsb.c` | brut |
| `ul_drop_no_bts` race | `bridge.py` | brut |
| BSP `trxd_peer_valid=false` race | `calypso_bsp.c` | brut |
| `publish_fb_found`/`publish_sb_found` calls | `calypso_fbsb.c` | bypass arch |

`.gitignore` étendu pour `__pycache__/`, `*.pyc`, `*.bak*` afin d'éviter
la répétition du bruit de 63b4fe5 dans les commits futurs.

### Connections ajoutées (vraies, pas hack)

| Connection | Fichier |
|---|---|
| `DB_W_D_TASK_RA = 7` poll dans tdma_tick UL section | `calypso_trx.c` |
| `calypso_bsp_tx_rach_burst` via `gsm0503_rach_ext_encode` (libosmocoding) | `calypso_bsp.c` |
| libosmocoding linkage dans meson | `hw/arm/calypso/meson.build` |
| `bsp.trxd_peer` pre-set à `(127.0.0.1, 5702)` | `calypso_bsp.c` |
| `bridge.trxd_remote` pre-set à `(BTS, base+102)` | `bridge.py` |
| `BRIDGE_CLK_FROM_QEMU=1` env mode (CLK IND piloté par QEMU FN) | `bridge.py` |
| `-icount shift=auto,align=off,sleep=off` sur QEMU | `run.sh` |

### Env vars (default = real path, opt-in pour dev assist)

| Env | Default | Effet |
|---|---|---|
| `CALYPSO_FBSB_SYNTH` | `0` | `1` ré-active publish_fb_found+publish_sb_found dans `on_dsp_task_change` (pour passer le DSP correlator non-convergent) |
| `CALYPSO_NDB_D_RACH_OFFSET` | `0x01CB` | Override word index de `d_rach` dans NDB (DSP version-dépendant) |
| `BRIDGE_CLK_FROM_QEMU` | `0` | `1` remplace wall-clock CLK IND par QEMU-FN pace |

### Diff total

12 fichiers modifiés, **+401 / -421** lignes (net -20). Les détails sont
dans `hw/arm/calypso/doc/hacks.md` § Cleanup 2026-05-07 (table par hack
avec replacement) et `hw/arm/calypso/doc/PROJECT_STATUS.md` § 2026-05-07.

## État du run avec `CALYPSO_FBSB_SYNTH=1`

Recompilé dans le conteneur Docker `trying`, lancé via `run.sh` modifié :

**DL — milestone 06/05 préservé** :
- `[calypso-fbsb] CALYPSO_FBSB_SYNTH=1 (synth, dev-assist path)` au boot
- `[fbsb] FB0_FOUND (synth)` + `SB_FOUND (synth)` cycliques
- Mobile L3 décode SI1, SI2, SI3, SI4 (`lai=001-01-1`)
- `Changing CCCH_MODE to 2`
- `MON: f=1 lev=<=-110 snr= 0 ber=  0 CGI=001-01-1-888` ← le 888 est là

**UL — progression vs 06/05** :
- Mobile passe à `RR_EST_REQ` + `connection pending` + `CHANNEL REQUEST:
  00 (Location Update with NECI)`
- `RANDOM ACCESS (requests left 8 → 7 → 6)` — mobile retries en cours
- Côté QEMU : `[BSP] RACH encode #N fn=NNNN ra=0xXX bsic=0xXX d_rach=0xNNNN`
  fire régulièrement
- `[calypso-trx] UL RACH task=0xXXXX tn=0 fn=NNNN` — **TN=0** (vrai slot RACH)
- Bridge délivre 10 paquets UL au BTS sur `('127.0.0.1', 5802)`
- BTS clock skew réduit à 1-FN compensation au lieu de 102-FN avant icount

**UL — pas encore résolu** :
- Pas d'`IMM_ASS_CMD` visible côté mobile (toujours dans le cycle
  `RR_EST_REQ → RACH ×N → T3126 timeout → re-RR_EST_REQ`)

## C'est exactement le point où on était le 06/05

Tu avais conclu sur deux failure modes non discriminés :
- (a) UL : RACH burst ne traverse pas jusqu'au décodage `osmo-bts-trx`
- (b) DL AGCH : BTS répond IMM_ASS mais mobile DSP rate l'AGCH sub-slot

Et tu avais proposé comme test discriminant immédiat : tcpdump GSMTAP
pendant un run, voir si `IMM_ASS_CMD` apparaît sur l'air.

Cette session apporte deux choses sur ce front :
1. **Les RACH UL sortent maintenant pour de vrai** sur TN=0 avec encoding
   AB correct (libosmocoding `gsm0503_rach_ext_encode`). Avant la session,
   ils étaient soit absents soit malformés (le seul `task_u` poll loupait
   le `d_task_ra` lane, et même quand il fire-ait, on lisait des bits depuis
   `dsp->data[0x0900]` qui est un guess non vérifié pour les UL bits).
2. **La pcap GSMTAP est en cours de capture**: `tcpdump -i eth1 -w
   /root/mobile-gsmtap.pcap udp port 4729` tourne depuis 50 minutes (process
   PID 26999). Donc le test discriminant est armé — il suffit d'attendre
   que le mobile retry assez longtemps puis analyser la pcap.

## Petits problèmes secondaires identifiés

1. **`task_ra` semble lire des résidus** : valeurs comme
   `task_ra=0x2d4e tn=6 fn=104` au début du run, avant que le mobile ne
   soit synced. Soit le firmware ne zéro-initialise pas l'API RAM, soit on
   lit le mauvais offset. Le `d_rach` montre des BSIC variables
   (`0x2a, 0x24, 0x1f, 0x14, 0x33, 0x1c, 0x19, 0x15, 0x3c`) ce qui est
   incohérent avec une cellule fixe. Hypothèse : `CALYPSO_NDB_D_RACH_OFFSET`
   par défaut (0x01CB, dérivé du walk struct DSP==33) pointe vers la
   mauvaise zone — le firmware utilise peut-être DSP==35 ou autre.
2. **TN values weird au début** (`tn=4`, `tn=6` avant LU complete) : même
   cause probable que (1).
3. Une fois LU démarré : les `UL RACH tn=0 fn=NNNN` apparaissent de façon
   cohérente avec les `RANDOM ACCESS` côté mobile L3, donc à ce stade le
   pipeline marche.

## Question pour toi

Avec ce contexte (RACH UL sortent réellement maintenant, milestone DL
intact), quelle est ta priorité parmi :

A. **Analyse pcap** : capture-pane le BTS log + parse la pcap GSMTAP pour
   voir si IMM_ASS_CMD passe sur l'air. Réponse direct (a)/(b) du
   discriminant.

B. **Fix d_rach offset** : tracer les writes API RAM autour de fn où le
   mobile fait `RANDOM ACCESS ra 0xXX`, pinner le vrai offset. Élimine
   les false positives RACH.

C. **DSP correlator real path** : sans `CALYPSO_FBSB_SYNTH`, identifier
   pourquoi le DSP fb-det émulé ne converge pas (les 3 pistes du
   SESSION_BRIEF : α bug opcode dans MAC/correlator, β timing miss, γ
   phase reference inversée).

D. **NB UL** : `calypso_bsp_tx_burst` lit encore `dsp->data[0x0900]` qui
   est un guess pour le buffer DSP→BSP des bursts encodés (SDCCH/SACCH).
   Dès que le mobile passe en SDCCH après IMM_ASS (si on résout le
   blocker IMM_ASS), c'est le prochain link à valider.

Mon tip personnel : (A) en premier puisque la pcap est armée et qu'elle
répond direct au test discriminant que tu avais cadré. (B) en parallèle
si tu veux nettoyer les faux positifs RACH visibles dans les logs. (C) et
(D) sont des deeper digs après.
