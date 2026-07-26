# Mecanisme FBSB Calypso - trace exacte, confrontation dsp_shunt/orch/osmocom-bb

## 0. Résumé exécutif

Le mécanisme FBSB (FCCH/SCH Burst Search) implique quatre acteurs distincts dans l'arbre `/opt/GSM` :

1. **Le firmware ARM réel** (`osmocom-bb/src/target/firmware/layer1/prim_fbsb.c`) — programme le DSP via `d_task_md` et lit le résultat via `d_fb_det`, sans rien savoir de l'émulateur.
2. **Le ROM DSP c54x réellement émulé** (opcodes exécutés par `calypso_c54x.c`) — censé faire tourner le corrélateur FB et écrire `d_fb_det`. **C'est ce chemin qui est bloqué**, par deux causes racines distinctes (A : IMR jamais réarmé pour le vecteur 28 / bit12 ; B : chaîne d'appels profonde `CALL 0xa4e4` non résolue côté banc XPC).
3. **`calypso_dsp_shunt.c`** — un shunt optionnel qui peut soit injecter des valeurs synthétiques dans la NDB, soit piloter directement le vrai cœur c54x à la place de `calypso_trx.c`. **Vérifié inactif** dans la session live (`CALYPSO_DSP_SHUNT=0`).
4. **`calypso_fbsb.c` + `calypso_orch.h`** — un traqueur/oracle côté hôte. Depuis le nettoyage du 2026-05-28, il ne fait plus que journaliser ; sa capacité de synthèse (`calypso_fbsb_publish_fb_found`) existe toujours mais est doublement verrouillée par `calypso_orch()` et `fbsb_synth_enabled()`, **toutes deux désactivées** dans la session live. La fonction `calypso_orch()` elle-même (le gate) est réellement implémentée et câblée (pas du code mort) ; en revanche l'API pub/sub `calypso_orch_init`/`calypso_orch_publish` évoquée ailleurs dans la doc de projet **n'existe nulle part** dans le code (`ARCHITECTURE.md:8` l'admet lui-même).

Conséquence vérifiée en direct sur le process `qemu-system-arm` PID 3751659 (`osmo-operator-1`, démarré 2026-07-03 20:01:51) : **aucune** des voies de synthèse/mock n'est active ; seul le vrai ROM DSP est censé produire `d_fb_det`, et il n'y écrit jamais (0 occurrence de `FBDET-SENTINEL #` dans tout le log), ce qui se traduit côté firmware réel par une boucle infinie `Cell search finished without result.` dans `mobile.log`.

---

## 1. Trace complète : de la commande ARM au résultat `d_fb_det` (call graph numéroté)

Chaque étape indique le fichier/ligne qui **implémente** le comportement et, séparément, le site d'**appel** qui le déclenche.

### 1.1 Émission de la commande (firmware ARM réel)

1. `l1a_l23_rx_cb()` reçoit `L1CTL_FBSB_REQ` — `osmocom-bb/src/target/firmware/layer1/l23_api.c:673-675`.
2. → `l1ctl_rx_fbsb_req(msg)` (dispatcher statique) — `l23_api.c:229-249`.
   - appelle `l1s_reset()` (`l23_api.c:242`) puis `l1s_fbsb_req(1, sync_req)` (`l23_api.c:250`).
3. `l1s_fbsb_req()` — `prim_fbsb.c:538` → `tdma_schedule_set(base_fn, fb_sched_set, 0)` (`prim_fbsb.c:562/564`).
4. Premier item du planning, `SCHED_ITEM_DT` → `l1s_fbdet_cmd(p1, p2, fb_mode)` — défini `prim_fbsb.c:364-383`, référencé `fb_sched_set[0]` (`prim_fbsb.c:505`). Corps exact :
   ```c
   373: l1s.fb.mode = fb_mode;
   376: rffe_compute_gain(rxlev2dbm(fbs.req.rxlev_exp), CAL_DSP_TGT_BB_LVL);
   379: dsp_api.db_w->d_task_md = FB_DSP_TASK;   /* = 5 */
   380: dsp_api.ndb->d_fb_mode = fb_mode;
   383: l1s_rx_win_ctrl(fbs.req.band_arfcn, L1_RXWIN_FB, 0);
   ```
   `FB_DSP_TASK = 5` — `osmocom-bb/.../include/calypso/l1_environment.h:73`. `d_task_md` est à l'offset `0x8` de `T_DB_MCU_TO_DSP` (vérifié par `offsetof()` compilé sur le header réel) → adresse ARM `0xFFD00000 + 0x8 = 0xFFD00008` (base `BASE_API_W_PAGE_0`, `dsp.c:86-89`, macros `dsp_api.h:18-23`).

### 1.2 Réception côté émulateur (MMIO write)

5. L'écriture ARM sur `0xFFD00008` tombe dans la région `calypso_dsp_ops` (`calypso_trx.c:966`) et est traitée par `calypso_dsp_write()` — `calypso_trx.c:504`, bloc de détection `d_task_md` à `calypso_trx.c:842-911` (constantes `DSP_API_W_PAGE0=0x0000`, `DB_W_D_TASK_MD=4` mots → offset octet `0x0008`, `calypso_trx.c:56-63`).
6. À l'intérieur de `calypso_dsp_write()`, dans l'ordre :
   - a. mirroir dans `s->dsp_ram[]` — `calypso_trx.c:606-607` ;
   - b. mirroir dans la mémoire de données du **vrai** c54x, `s->dsp->data[offset/2 + 0x0800]` — `calypso_trx.c:614-622` (commentaire explique que sans ce mirroir le DSP verrait une valeur périmée via `prog_fetch`) ;
   - c. verrou de diagnostic si valeur == 5 (`g_arm_taskmd5_insn`/`g_arm_taskmd5_ea`) — `calypso_trx.c:863-865` ;
   - d. appel du traqueur hôte `calypso_fbsb_on_dsp_task_change()` — site d'appel `calypso_trx.c:894-908`, implémentation `calypso_fbsb.c:97-141` (voir §2c — **journalisation uniquement** dans cette session).

### 1.3 Ce qui *devrait* se passer dans le vrai ROM DSP (bloqué — root causes A/B)

7. Le ROM DSP doit être réveillé par une interruption liée à la tâche FB (vecteur documenté comme « vec28 »/bit12 de l'IMR dans les notes d'architecture internes). L'entrée d'interruption pousse **2 mots** (PC+XPC), contrairement à un `CALL` normal qui n'en pousse qu'1 — `calypso_c54x.c:13977-13985` / `14027-14035`.
8. **ROOT CAUSE A (confirmée, live)** : l'IMR est mis à 0 et n'est jamais réarmé. Site d'implémentation du log : `calypso_c54x.c:3138` (`IMR-ARM 0x3000 -> 0x0000 ...`). Preuve live unique dans la session : `qemu.log:882` — `+0.028s [c54x] IMR-ARM 0x3000 -> 0x0000 (b12/vec28=0 ...) PC=0xb37e op=0x7700 insn=1047`. Aucune deuxième transition d'IMR n'apparaît dans tout le log (180/180 échantillons `FRAME-GATE` avec `IMR=0x0000`) : le bit12/vec28 n'est jamais réarmé après ce point.
9. Même si l'interruption partait, le gestionnaire trouvé en pratique est un stub auto-référentiel : `qemu.log:978` `SLOT4387-WR data[0x4387] <- 0xab38 PC=0xb4bd insn=1200` (site `calypso_c54x.c:2545`) puis `qemu.log:1393` `BACC-DISP #1 handler=0xab38 AR7=0x4387 (RET/noop) XPC=0 insn=4393` (site `calypso_c54x.c:12099`) — l'annotation `(RET/noop)` du code lui-même confirme qu'il s'agit d'un simple retour sans effet.
10. **`CALL 0x013b` (0x7234)** — décodé statiquement (`calypso_dsp.txt` ligne 1833) : `F074 013b` = `CALL` inconditionnel 2 mots (`calypso_c54x.c:6249-6256`) qui pousse 1 mot (`SP-=1; data[SP]=PC+2=0x7236`). À `0x013b` (PROM0 `0x713b`) : `3f93` = instruction MAC (`calypso_c54x.c:8290+`) ; à `0x013c` : `8095` = `STL A, Smem` (`calypso_c54x.c:8771-8778`) ; à `0x013d` : `fc00` = `RC/RET` inconditionnel (décodage `calypso_c54x.c:7170-7183`, saut effectif `calypso_c54x.c:7267`) qui dépile exactement la valeur poussée (`0x7236`). **Conclusion (root-cause B, item clos)** : `CALL 0x013b` est un aller-retour trivial, SP neutre en 3 instructions — **ce n'est pas la source de la dérive de pile**. Ce trampoline (`0x013b`) est réutilisé tel quel à au moins 4 autres sites PROM0 (`calypso_dsp.txt` lignes 07020/07130/07220/07250/07260) : ce n'est pas un code spécifique au vecteur FB.
11. **`CALL 0xa4e4` (0x723d)**, immédiatement après le retour de l'étape 10, est le véritable point de descente profonde — cela correspond exactement à la note d'architecture déjà présente dans le code : `calypso_c54x.c:13918` (« 0x7234 → CALL 0xa4e4 → LD d_dsp_page 0xa51c → corrélateur → d_fb_det »). **Root cause B (item ouvert)** : `0xa4e4 ≥ 0x8000` implique un adressage banké par XPC (`c54x_prog_xlate`, `calypso_c54x.c:1980-1992`), or rien dans la séquence décodée entre l'entrée d'interruption et `0x723d` ne recharge XPC explicitement (l'entrée d'interruption le remet à 0 : `calypso_c54x.c:13986`/`14036`). Sous XPC=0 (PROM0), `0xa4e4` ne décode pas en instructions plausibles (`calypso_dsp.txt:6234-6235`) ; sous XPC=1 (`calypso_dsp.txt:4194`) et XPC=2 (`calypso_dsp.txt:6244`) le contenu est du code cohérent (ex. sous XPC=2 : `LD *(0x3ade), A` à `0x2a4e4`, et un `RET` à `0x2a4ec` précédé d'un `STL`/`AND` — motif similaire à une queue de sous-routine). **Le mécanisme exact de bascule de banc n'a pas pu être identifié** sans lecture live du registre XPC (voir §5).

### 1.4 Lecture du résultat (ou timeout) côté firmware ARM

12. `fb_sched_set` planifie ensuite `l1s_fbdet_resp(p1, attempt, fb_mode)` pour `attempt = 1..12` (`prim_fbsb.c:507-518`), implémentation `prim_fbsb.c:399-501` :
    ```c
    404: if (!dsp_api.ndb->d_fb_det) {
    407:     if (attempt < 12) return 0;
    414:     tdma_sched_reset();
    416:     if (fbs.fb_retries < FB0_RETRY_COUNT) { ... fbs.fb_retries++; }
    420:     else { last_fb->attempt = 13; l1s_compl_sched(L1_COMPL_FB); }
    424:     return 0;
    ```
    `d_fb_det` est à l'offset `0x48` de `T_NDB_MCU_DSP` → adresse ARM `0xFFD001A8+0x48 = 0xFFD001F0` (`dsp_api.h`+`offsetof()` compilé ; confirmé identique dans `doc/project/CLAUDE.md:215`).
13. Côté émulateur, cette lecture MMIO passe par `calypso_dsp_read()` (`calypso_trx.c:180`) qui renvoie `s->dsp->data[0x08F8]` (**mémoire réelle du DSP**, pas une case shuntée — `calypso_trx.c:237-245`, commentaire daté du fix du 2026-05-15).
14. `FB0_RETRY_COUNT = 3` (`prim_fbsb.c:51`) ⇒ 12 frames × 4 tours (1 initial + 3 retries) = **48 frames** avant échec définitif.
15. Échec définitif : `last_fb->attempt = 13` puis `l1s_compl_sched(L1_COMPL_FB)` (`prim_fbsb.c:421-422`) → table d'achèvement installée en `prim_fbsb.c:570-573` → `l1a_fb_compl()` (`prim_fbsb.c:523-536`) → `l1ctl_fbsb_resp(255)` (`prim_fbsb.c:527`) → construit et envoie `L1CTL_FBSB_CONF` avec `result=255` (`prim_fbsb.c:100-120`, champ écrit ligne 113).
16. En cas de succès (non observé), `read_fb_result()` (`prim_fbsb.c:304-326`) lit `a_sync_demod[D_TOA|D_PM|D_ANGLE|D_SNR]` (indices `l1_environment.h:261-264`), offset NDB `0x4c` → ARM `0xFFD001F4` (TOA), `+2` PM, `+4` ANGLE, `+6` SNR ; puis remet `d_fb_det=0` (ack, `prim_fbsb.c:318`).

---

## 2. Les quatre chemins, séparés clairement

### (a) Chemin réel — ROM DSP c54x, bloqué par les root causes A et B

- **Root cause A** (confirmée, statique + live) : l'IMR passe de `0x3000` à `0x0000` une seule fois (`qemu.log:882`, PC=0xb37e, op=0x7700, insn=1047 — `calypso_c54x.c:3138`) et n'est **jamais réarmé** ensuite (0 seconde transition observée sur toute la session). Le bit12/vec28 reste donc masqué en continu.
- **Root cause B** : même en supposant l'interruption levée, le handler trouvé (`0xab38`) est un stub `(RET/noop)` (`calypso_c54x.c:12099`, `qemu.log:1393`). La branche utile identifiée passe par `CALL 0x013b` (trampoline SP-neutre, §1.3 étape 10 — **innocenté**) puis `CALL 0xa4e4` (§1.3 étape 11 — **suspect principal**, banc XPC non déterminé). Le corrélateur FB attendu est documenté dans le code lui-même comme se situant vers `~0x9a80-0x9ac0` selon les notes d'architecture citées par les investigateurs, mais son exécution effective n'a pas pu être tracée jusqu'au bout faute de lecture XPC live.
- Écriture attendue mais jamais observée : `data[0x08F8]` (= `d_fb_det`). Confirmée par 0 occurrence de `FBDET-SENTINEL #` (log actif via `CALYPSO_FBDET_SENTINEL=2`, mode observation pure, non forçant — `calypso_c54x.c:2380-2388`) dans toute la session.

### (b) `calypso_dsp_shunt.c` — chemin mock, **inactif dans la session live**

- Fichier `hw/arm/calypso/calypso_dsp_shunt.c`, en-tête `:1-37`.
- Verrou maître : `calypso_dsp_shunt_init()` (`calypso_dsp_shunt.c:1675-1681`) — actif seulement si `CALYPSO_DSP_SHUNT` vaut exactement `"1"` ; mémorisé une fois à l'appel depuis `calypso_mb.c:257`.
- Verrou secondaire imbriqué : `shunt_route_c54x()` (`calypso_dsp_shunt.c:219-227`) lit `CALYPSO_DSP=="c54x"`.
- Dispatch par frame TDMA : `calypso_dsp_shunt_on_frame_tick()` (`calypso_dsp_shunt.c:1059-1097`), appelé depuis `calypso_trx.c:1599`. Retour anticipé si `!g_shunt.active` (ligne 1062).
- Sélection à `calypso_dsp_shunt.c:1077-1086` :
  - si `shunt_route_c54x() && g_shunt.c54x` → **chemin réel-DSP-piloté** : `shunt_route_to_c54x()` (`calypso_dsp_shunt.c:1015-1051`) DMA la burst I/Q capturée vers DARAM `0x0586`, appelle `c54x_bsp_load()`, lève l'IRQ frame + `c54x_wake()`, puis exécute réellement des opcodes via `c54x_run(dsp, budget)` (`CALYPSO_DSP_BUDGET`, défaut 256000).
  - sinon selon `md`/`td` → **chemin mock synthétique** : `shunt_dispatch_fb()` (`calypso_dsp_shunt.c:541-566`) ou `shunt_dispatch_sb()` (`:572-624`) écrivent **directement** `d_fb_det`/`a_sync_demod` en NDB via `shunt_write_w()` (MMIO/dma_memory_write, `:237-241`), en préférant les valeurs réelles décodées par gr-gsm (`g_shunt.sb_valid`, etc.) et en repli seulement sur les constantes `CALYPSO_CANNED` (parsées `:490-508`).
- Quand actif, `calypso_trx.c` cesse volontairement de piloter le c54x lui-même (gates `!calypso_dsp_shunt_active()` à `calypso_trx.c:1731` et `:1818`), cédant la main à `shunt_route_to_c54x()`.
- **Statut vérifié live** : `CALYPSO_DSP_SHUNT=0` dans `/proc/3751659/environ` → `strcmp(env,"1")` échoue → `g_shunt.active=false` en permanence (`calypso_dsp_shunt.c:1678-1680`). `CALYPSO_DSP=c54x` est présent mais **inerte** (seul lecteur de cette variable : `calypso_dsp_shunt.c:223`, chemin mort tant que `g_shunt.active=false`). Confirmation logs : `grep -c '\[dsp-shunt\]' qemu.log` → **0**.
- Point noté comme suspect (non bloquant) : `calypso_dsp_shunt_route_c54x_active()` (`:1830-1833`) est exportée mais n'a **aucun appelant** en dehors de son propre fichier — commentaire `:1827-1829` prétend qu'elle est « used by calypso_trx.c », ce qui est faux (trx.c utilise `calypso_dsp_shunt_active()` directement) → commentaire obsolète / API potentiellement morte.

### (c) `calypso_fbsb.c` — traqueur/oracle hôte, **observation seule dans cette session**

- En-tête `calypso_fbsb.c:1-13` : nettoyage du 2026-05-28, toute synthèse (publish_fb_found/publish_sb_found/clear_fb/latches W1C/machine d'état on_frame_tick) a été retirée ; le fichier ne fait plus que journaliser les changements de tâche DSP.
- `calypso_fbsb_on_dsp_task_change()` (`calypso_fbsb.c:97-141`), appelée depuis `calypso_trx.c:894-908` (init paresseuse via `g_fbsb_inited`, `calypso_trx.c:896-901`).
- Journalisation inconditionnelle (`calypso_fbsb.c:100-101`) : `fprintf(stderr, "[calypso-fbsb] on_dsp_task_change task=%u fn=%lu state=%d\n", ...)`.
- Pour `DSP_TASK_FB` (5) : état local `FBSB_FB0_SEARCH`, `fb0_attempt++`, dump (`calypso_fbsb.c:104-109`), **puis seulement** tentative conditionnelle de synthèse (`calypso_fbsb.c:113-120`) :
  ```c
  113: if (calypso_orch() && fbsb_synth_enabled()) {
  118:     if (calypso_bsp_get_fb_detection(&toa, &pm, &ang, &snr))
  119:         calypso_fbsb_publish_fb_found(s, toa, pm, ang, snr);
  ```
  `fbsb_synth_enabled()` (`calypso_fbsb.c:36-43`) exige `CALYPSO_SYNTH_FBSB` ou `CALYPSO_DSP_L1_STUB` non-nul/non-"0" — **aucun des deux n'est présent** dans l'environ live. `calypso_orch()` est également faux (voir §2d). Le `&&` court-circuite dès `calypso_orch()` → le bloc de synthèse ne s'exécute jamais.
- **Bug latent découvert** (non corrigé, signalé) : `calypso_bsp_get_fb_detection()` (`calypso_bsp.c:168-179`) lit et efface `bsp.fb_valid`, mais un grep complet du fichier montre que `bsp.fb_valid` n'est **jamais assigné à 1** nulle part — seulement déclaré (ligne 157) et lu/effacé (171, 178). Même avec tous les gates activés, cette fonction retournerait toujours faux : régression probable issue du refactor qui a introduit le classifieur TONAL_FB écrivant directement dans `bsp.dsp->data[]` (§2d) sans jamais alimenter le latch `fb_valid`.
- **Machine d'état obsolète par rapport au header** : `calypso_fbsb.h:14-29` documente un diagramme `FB0_SEARCH --12 tentatives sans FB--> FAIL`, mais `calypso_fbsb_on_dsp_task_change()` ne contient **aucune** logique de seuil sur `fb0_attempt` ni de transition vers `FBSB_FAIL` — il ré-entre indéfiniment dans `FBSB_FB0_SEARCH` en incrémentant le compteur sans borne. Preuve live : `grep -o 'state=FB0_SEARCH\|state=FB0_FOUND\|state=DONE\|state=FAIL' qemu.log | sort | uniq -c` → 3444+ `state=FB0_SEARCH`, zéro autre état.

### (d) `calypso_orch` — le gate est réel, l'API pub/sub ne l'est pas

- `calypso_orch.h` (19 lignes, header-only) : définit **une seule** chose, la fonction `static inline calypso_orch()` (lignes 10-18), mémorisée sur `getenv("CALYPSO_ORCH")` (ligne 14), vraie si définie/non-vide/≠`"0"` (ligne 15). Pas de fichier `.c` séparé — ni nécessaire ni présent (recherché explicitement, aucun hit).
- Cette fonction **est réellement câblée** à 5 sites d'appel, tous vérifiés vivants dans le code :
  - `calypso_fbsb.c:50` — garde `calypso_fbsb_publish_fb_found()`.
  - `calypso_fbsb.c:113` — garde le déclenchement de la synthèse FB (voir §2c).
  - `calypso_bsp.c:1222` — le site **le plus significatif** : écrit directement dans `bsp.dsp->data[]` (mémoire réelle du DSP, pas une ombre NDB), basé sur un classifieur de forme de burst côté hôte (« TONAL_FB », `same_sign>=8 && nmax>=64`) :
    ```c
    1223: calypso_pcb_daram_lock_acquire();
    1224: if (same_sign >= 8 && nmax >= 64) {
    1225:     bsp.dsp->data[0x08FA] = 23;               /* TOA */
    1226:     bsp.dsp->data[0x08FB] = 0x7000;            /* PM */
    1227:     bsp.dsp->data[0x08FC] = 0;                 /* ANGLE */
    1228:     bsp.dsp->data[0x08FD] = (same_sign*0x7000)/10; /* SNR */
    1229:     bsp.dsp->data[0x08F8] = 1;                 /* d_fb_det = FOUND */
    1230: } else {
    1231:     bsp.dsp->data[0x08F8] = 0;
    1232: }
    ```
  - `calypso_trx.c:368` (`force_nb`), `:1377` (`g_nbdi`), `:1501` (`g_sbi`) — autres injections hôte non spécifiques au FB.
- En revanche, l'API pub/sub évoquée dans la doc de projet (`calypso_orch_init`, `calypso_orch_publish`, et 7 autres fonctions « §5 ») **n'existe nulle part** — `grep -rn 'calypso_orch_'` sur les deux arbres ne trouve que la mention dans `doc/project/ARCHITECTURE.md:8`, qui admet lui-même : *« Aucune des 9 fonctions déclarées en §5 (`calypso_orch_init/publish`, ...) n'existe (`grep -rl` = 0 sur tout l'arbre) »*. **Nuance importante** : « déclaration seulement » est vrai pour ce bus pub/sub aspirationnel, mais faux pour la fonction gate `calypso_orch()` elle-même, qui est réelle, implémentée, et active.
- **Statut vérifié live** : `CALYPSO_ORCH` **absent** de l'environ du process 3751659 → `calypso_orch()` mémorise `0` pour toute la session → les 3 écritures FB-pertinentes gardées par `calypso_orch()` (dont celle, consequente, de `calypso_bsp.c:1222-1231`) sont **inertes**. C'est le comportement par défaut documenté (commentaire d'en-tête `calypso_orch.h`, mode « EXECUTION : le vrai ROM DSP écrit seul la NDB »).

---

## 3. Confrontation avec le firmware réel osmocom-bb

### 3.1 Table champ par champ

| Champ | Struct / offset réel | Adresse ARM | Écrit/lu par firmware réel | Comportement émulateur vérifié |
|---|---|---|---|---|
| `d_task_md` | `T_DB_MCU_TO_DSP` offset `0x8` (`dsp_api.h`+`offsetof`) | `0xFFD00008` | écrit `l1s_fbdet_cmd`, `prim_fbsb.c:379` | `calypso_dsp_write()` mirroré dans `dsp_ram[]` **et** `s->dsp->data[0x0804]` — `calypso_trx.c:606-622` |
| `d_fb_mode` | `T_NDB_MCU_DSP` offset `0x4a` | `0xFFD001F2` | écrit `prim_fbsb.c:380` | mirroré comme ci-dessus (offset dérivé) |
| `d_fb_det` | `T_NDB_MCU_DSP` offset `0x48` | `0xFFD001F0` | lu `l1s_fbdet_resp`, `prim_fbsb.c:404` ; effacé `prim_fbsb.c:318` | `calypso_dsp_read()` → `s->dsp->data[0x08F8]` (mémoire DSP réelle) — `calypso_trx.c:237-245` |
| `a_sync_demod[TOA/PM/ANGLE/SNR]` | `T_NDB_MCU_DSP` offset `0x4c`, `+2/+4/+6` | `0xFFD001F4..FA` | lu `read_fb_result()`, `prim_fbsb.c:306-309` | mêmes cases `data[0x08FA..0x08FD]` — confirmées par le classifieur TONAL_FB (§2d) qui cible exactement ces adresses |

Tous les offsets sont corroborés de manière indépendante par trois sources : le header réel compilé (`offsetof()`), `doc/project/CLAUDE.md:215-216`, et les constantes internes de l'émulateur (`calypso_fbsb.h:52` `NDB_D_FB_DET=0x08F8`, `calypso_dsp_shunt.c:91` `#define NDB_D_FB_DET 0x48`). **Aucune divergence** trouvée sur ces adresses.

### 3.2 Numéros de tâche DSP — cohérence totale

```
FB_DSP_TASK      = 5   (l1_environment.h:73)   == DSP_TASK_FB  (calypso_fbsb.h:63) == FB_DSP_TASK (calypso_dsp_shunt.c:112)
SB_DSP_TASK      = 6   (l1_environment.h:74)   == DSP_TASK_SB  (calypso_fbsb.h:64)
TCH_FB_DSP_TASK  = 8   (l1_environment.h:75)   (mode dédié, hors flux FBSB idle)
TCH_SB_DSP_TASK  = 9   (l1_environment.h:76)   (mode dédié, hors flux FBSB idle)
```
Aucune incohérence de numérotation entre firmware réel et émulateur.

### 3.3 Ce que le firmware réel attend vs ce que l'émulateur fournit actuellement

Le firmware réel n'a besoin de rien d'autre que : (1) que `d_task_md` soit lu par un DSP qui exécute effectivement la tâche FB, et (2) que `d_fb_det` finisse par passer à une valeur non nulle avec des champs `a_sync_demod` plausibles. **Aucun** artifice logiciel côté firmware ne dépend de la présence d'un shunt ou d'un oracle — le protocole NDB est le seul contrat. Dans la session live, ce contrat n'est jamais honoré côté écriture (root causes A/B), ce qui est directement visible côté firmware réel par la boucle `mobile.log` (§4).

### 3.4 Comparaison avec le mock amont : `virt_prim_fbsb.c`

`osmocom-bb/src/host/virt_phy/src/virt_prim_fbsb.c` (129 lignes) est structurellement **très différent** de nos deux mocks : il n'a **aucune NDB, aucun `d_task_md`, aucune boucle de polling par frame**. `l1ctl_rx_fbsb_req()` (lignes 51-62) se contente d'enregistrer l'ARFCN demandé et de passer `state = MS_STATE_IDLE_SYNCING`. La synchronisation est déclarée dès que **n'importe quel** message GSMTAP descendant taggé avec l'ARFCN correspondant arrive (`prim_fbsb_sync()`, lignes 69-98), qui appelle directement `l1ctl_tx_fbsb_conf(ms, 0, arfcn)` (ligne 97) avec des valeurs **factices câblées en dur** : `fn=0`, `snr=40`, `initial_freq_err=0`, `bsic=0` (lignes 114-117 ; commentaire ligne 108 : « No calculation needed for virtual phy -> uses dummy values »). L'échec passe par un simple compteur `sync_count++ > 20` (ligne 80) sans rapport avec le vrai timing GSM.

**Conclusion de la comparaison** : le mock amont (`virt_phy`) court-circuite entièrement le protocole mémoire partagée ARM↔DSP et répond au niveau du message L1CTL — une approche beaucoup plus radicale. Nos deux mécanismes (`calypso_dsp_shunt.c` et l'ancien `calypso_fbsb.c` pré-2026-05-28) sont **plus fidèles au contrat réel** car ils continuent de parler le protocole NDB documenté (écriture aux offsets réels `d_fb_det`/`a_sync_demod`), même quand ils fabriquent la valeur. Le `calypso_fbsb.c` actuel (post-nettoyage) ne fait ni l'un ni l'autre — il est purement observationnel.

---

## 4. Confirmé en direct

Toutes les preuves suivantes proviennent du même process live `qemu-system-arm` PID 3751659 (conteneur `osmo-operator-1`, démarré 2026-07-03 20:01:51), recoupé entre 20:04 et 20:10 UTC via `/root/qemu.log`, `/root/mobile.log`, `/root/osmo-trx-ipc.log`, `/root/bts.log`, `/proc/net`, `/proc/3751659/environ`.

- **`d_task_md=5` réellement écrit en continu** : `qemu.log:6188` (`+7.269s [trx] DSP-DONE-DMA #32 ... task_md=5 -> DARAM 0x0586 fn=1572`, site `calypso_trx.c:1002-1005`, log plafonné à 60 occurrences) et encore en fin de session `qemu.log:63272` (`+204.881s [calypso-fbsb] on_dsp_task_change task=5 fn=44388 state=1`, site `calypso_fbsb.c:100`).
- **Burst I/Q réelles reçues sur UDP 6702** : socket possédé par le PID live (`ss -uapn` → `qemu-system-arm,pid=3751659,fd=14`), fichier de dump `/dev/shm/bursts.cfile` croissant en continu (`1367998392` octets @ `1783109245` → `1369623756` @ `1783109257`, +1.6 Mo/12s). Corroboré par `osmo-trx-ipc.log` (`ENERGY fn=2465196 tn=5 ...` @ `1783109245.533`).
- **Machine d'état `FBSB_FB0_SEARCH` active mais bloquée** : deux échantillons à 30s d'écart — `+453.853s fb0_att=29` puis `+495.115s fb0_att=101` (`fb1_att`, `sb_att`, `fb0_ret`, `afc_ret` restent à 0 en permanence). `fb0_attempt` est un `uint8_t` (`calypso_fbsb.h:88`) qui boucle mod-256 tous les ~18s (repasse à 0 à `qemu.log:11203, 16756, 22319, 27863...`) — un simple débordement arithmétique, **pas** un redémarrage réel de la machine d'état (`calypso_fbsb_reset()` n'est appelée qu'une seule fois, à l'init).
- **`d_fb_det` reste à zéro, confirmé par absence d'écriture (pas juste par sa valeur)** : `CALYPSO_FBDET_SENTINEL=2` (mode observation non forçant, `calypso_c54x.c:2380-2388`) journalise **toute** écriture STORE sur `0x08f8`, quelle que soit la valeur — `grep -c "FBDET-SENTINEL #" qemu.log` → **0** sur toute la session (~280s+). Lecture ARM confirmée figée depuis `fn=1212` : `qemu.log:5125` (`+5.607s [FBDET-RD] ARM read d_fb_det(0x01F0)=0x0000 fn=1212`, log déclenché uniquement sur changement de valeur — aucune autre occurrence ensuite = valeur inchangée).
- **Corroboration côté firmware réel** : `mobile.log:37782` et répétitions toutes les ~0.3s jusqu'en fin de log — `+497.474s DCS <0003> gsm322.c:2095 Cell search finished without result.`
- **Toutes les variables de synthèse/forçage confirmées absentes ou neutralisées** : `CALYPSO_DSP_SHUNT=0` (présent mais fonctionnellement inactif, vérifié via `strcmp`) ; `CALYPSO_DSP_FRAME_VEC28`, `CALYPSO_POKE_A4C7_ONCE`, `CALYPSO_TRACE_VEC28_STACK`, `CALYPSO_ORCH`, `CALYPSO_SYNTH_FBSB`, `CALYPSO_DSP_L1_STUB`, `CALYPSO_FORCE_TOA` — **tous absents** de l'environ. `grep -c "VEC28" qemu.log` → 0. Seuls `CALYPSO_FBWATCH=1` et `CALYPSO_FBDET_SENTINEL=2` sont actifs, et ce sont des moniteurs passifs (pas des forçages), cohérents avec la contrainte lecture-seule.
- **Reproduction live indépendante de la root cause A**, retrouvée en cherchant les traces IMR : `qemu.log:882` — seule et unique transition `IMR-ARM 0x3000 -> 0x0000` de toute la session, jamais réarmée ensuite.
- **Nuance FRAME_FAITHFUL (non un bug)** : le tick périodique INT3 qui maintient le DSP « vivant » (`calypso_trx.c:1795-1798`) n'est volontairement **pas** gardé par `imr_armed` quand `CALYPSO_FRAME_FAITHFUL=1` (actif dans cet env) — `periodic_armed = g_frame_faithful ? tpu_armed : (tpu_armed && imr_armed)` (`calypso_trx.c:1787-1788`), un choix de conception déjà documenté (`calypso_trx.c:1766-1782`) pour casser un verrou circulaire, à ne pas confondre avec la panne d'armement IMR/vec28 elle-même.

**Aucune contradiction** trouvée entre les quatre angles d'investigation et les preuves live — seulement des nuances/clarifications (listées dans chaque section ci-dessus).

---

## 5. Non résolu

1. **Identité exacte du RET près de DARAM `0x0155`/`0x0157`** (root cause B, item non clos). L'ordre d'exécution du code (`calypso_c54x.c:7213-7218` imprime `s->pc` **avant** que `s->pc = ra` ne soit exécuté à `calypso_c54x.c:7267`) est définitivement établi : l'adresse imprimée par `VEC28-STACK-TRACE` est l'adresse de **fetch du RET lui-même**, pas la destination du saut. Mais en confrontant cela au dump statique (`calypso_dsp.txt:1831`, ligne `07150`) :
   - `0x0155 = 0xFC00` décode bien comme `RC/RET` inconditionnel (`calypso_c54x.c:7170`), précédé d'un bloc `RPTB` (`0x0151=F072`) et d'un `STL` (`0x0154=0x8a11`) — la forme classique d'une fin de sous-routine.
   - `0x0157 = 0xF495` décode comme un **NOP explicite** (`calypso_c54x.c:4900` : `if (op == 0xF495) return consumed + s->lk_used; /* NOP */`), pas comme une instruction de la famille RET.
   Si une trace log antérieure a réellement cité `PC=0x0157`, cela contredit le contenu statique de la ROM — le RET réel semble être à `0x0155` (2 mots plus tôt). **Impossible à vérifier en direct dans cette session** : `CALYPSO_TRACE_VEC28_STACK` et `CALYPSO_DSP_FRAME_VEC28` sont absents de l'environ du process 3751659, donc aucune trace `VEC28-STACK-TRACE` ne peut être produite sans relancer le process avec ces variables armées — ce qui sort du cadre « lecture seule / pas de redémarrage » de cette investigation.
2. **Banc XPC actif au moment de `CALL 0xa4e4`** (root cause B, item ouvert). Rien dans la séquence décodée entre l'entrée d'interruption (qui force XPC=0) et `0x723d` ne recharge XPC de façon identifiable statiquement. Les candidats non confirmés sont : la paire `STM` à `0x7236`/`0x7238`, le `ANDM` à `0x723a`, ou la région gardée par le `XC` à `0x723c`. Sous XPC=1 et XPC=2, le contenu à `0xa4e4` décode en code plausible (voir §1.3 étape 11) ; sous XPC=0, non. Aucune commande HMP/QMP personnalisée n'expose `data[]`/`xpc` du DSP (`grep -rn 'hmp_\|monitor_register\|MonitorHMPCommand' hw/arm/calypso/*.c` → aucun résultat), donc la confirmation nécessite soit une lecture live de registre soit un relancement avec trace armée — les deux hors du périmètre lecture-seule de cette tâche.
3. **`calypso_dsp_shunt_route_c54x_active()`** — exportée, commentée comme utilisée par `calypso_trx.c`, mais sans appelant réel trouvé ailleurs que dans son propre fichier. À vérifier si c'est du code mort ou un branchement manqué.
4. **`bsp.fb_valid` jamais mis à `1`** dans `calypso_bsp.c` — rend `calypso_bsp_get_fb_detection()` structurellement incapable de retourner vrai même si tous les gates (`calypso_orch()`, `fbsb_synth_enabled()`) étaient activés. Probable régression du refactor ayant introduit le classifieur TONAL_FB (`calypso_bsp.c:1222-1232`), qui écrit directement dans `bsp.dsp->data[]` sans jamais alimenter ce latch.
5. **Diagramme d'état dans `calypso_fbsb.h:14-29`** (transition vers `FBSB_FAIL` après 12 tentatives) ne correspond à aucune logique implémentée dans `calypso_fbsb.c` actuel — documentation aspirationnelle/obsolète à corriger ou à supprimer.
6. **Trampoline `0x013b` réutilisé à ≥4 autres sites PROM0** (`calypso_dsp.txt` lignes 07020/07130/07220/07250/07260) — sa fonction générique exacte (au-delà de « petit utilitaire partagé ») n'a pas été caractérisée ; sans incidence connue sur le blocage FBSB mais laissé en observation.

---

## 6. Annexe — fichiers et lignes de référence

**Émulateur (`/opt/GSM/qemu-src/hw/arm/calypso/`)** :
- `calypso_c54x.c` : 1980-1992 (`c54x_prog_xlate`, translation XPC), 2200/2380-2388/2421 (`CALYPSO_FBWATCH`/`CALYPSO_FBDET_SENTINEL`), 2545 (`SLOT4387-WR`), 3138 (`IMR-ARM` log), 3705-3727 (`prog_fetch`, mirroir OVLY DARAM/PROM0), 4121/12213/13928/13999/14043 (lectures `getenv` VEC28/POKE_A4C7), 4897-4930 (dispatcher `case 0xF`, NOP spécial `0xF495`), 6249-6277 (`CALL`/`RPTB`), 7170-7267 (`RC/RET`, ordre print/jump), 7325-7351 (`RCD/RETD`), 8163-8180/8290+/8771-8778 (LD/MAC/STL), 12099 (`BACC-DISP`), 13911-14060/13977-14036 (entrée d'interruption, push 2 mots, remise à zéro XPC), 13918 (note d'architecture `CALL 0xa4e4`).
- `calypso_trx.c` : 56-63 (constantes offsets `d_task_md`), 180-345/237-245 (`calypso_dsp_read`, lecture `d_fb_det` réelle), 368/1377/1501 (autres sites `calypso_orch()`), 504-622 (`calypso_dsp_write`, mirroir + hook fbsb), 817 (`calypso_dsp_shunt_record_rach`), 842-966/894-908 (détection `d_task_md`, appel tracker), 982/992/1731/1738/1818 (gates `calypso_dsp_shunt_active()`), 1001-1005 (`DSP-DONE-DMA` log), 1590-1605/1599 (`calypso_dsp_shunt_on_frame_tick` call site), 1766-1798/1787-1788 (FRAME_FAITHFUL, periodic_armed).
- `calypso_bsp.c` : 48-52 (`BSP_LOG`, `BSP_TRXD_PORT`), 157-179/168-179 (`fb_valid` latch, jamais mis à 1), 384-459 (`bsp_trxd_readable`), 403/458/462/956/1112 (gates shunt), 1195-1232/1222-1231 (classifieur TONAL_FB, écriture directe `bsp.dsp->data[]`), 1324 (`BSP_DUMP_RX_FILE`).
- `calypso_dsp_shunt.c` : 1-37 (en-tête), 89-127/91/112 (constantes NDB, `FB_DSP_TASK`), 219-227 (`shunt_route_c54x`), 237-241 (`shunt_write_w`), 440-508/444-479/490-508 (constantes `SHUNT_CANNED_*`, parsing `CALYPSO_CANNED`), 541-566 (`shunt_dispatch_fb`), 572-624 (`shunt_dispatch_sb`), 1015-1051 (`shunt_route_to_c54x`), 1059-1097/1077-1086 (`on_frame_tick`, sélection de route), 1675-1681 (`calypso_dsp_shunt_init`), 1816-1819 (`calypso_dsp_shunt_active`), 1827-1833 (`calypso_dsp_shunt_route_c54x_active`, commentaire obsolète).
- `calypso_orch.h` : fichier complet 19 lignes, fonction `calypso_orch()` 10-18, lecture `CALYPSO_ORCH` ligne 14.
- `calypso_fbsb.c` : fichier complet 162 lignes ; 1-13 (en-tête cleanup 2026-05-28), 34-43 (`fbsb_synth_enabled`), 45-70/47/50/55 (`calypso_fbsb_publish_fb_found`), 78-81 (`init`/`reset`), 97-141/100-101/104-120/113 (`on_dsp_task_change`), 154-161 (dump).
- `calypso_fbsb.h` : fichier complet 121 lignes ; 14-29 (diagramme d'état obsolète), 52 (`NDB_D_FB_DET`), 63-64 (`DSP_TASK_FB`/`DSP_TASK_SB`), 88 (`fb0_attempt` type `uint8_t`).
- `calypso_mb.c` : 240-270/257/266 (init/set_c54x du shunt).
- `doc/project/ARCHITECTURE.md` : ligne 8 (aveu des 9 fonctions `calypso_orch_*` inexistantes).

**Firmware réel (`/opt/GSM/osmocom-bb/src/target/firmware/`)** :
- `layer1/prim_fbsb.c` : fichier complet 573 lignes ; 51 (`FB0_RETRY_COUNT`), 100-120/113 (`l1ctl_fbsb_resp`), 304-326/306-309/318 (`read_fb_result`), 364-383/373/376/379/380/383 (`l1s_fbdet_cmd`), 399-501/404/407/412-424 (`l1s_fbdet_resp`), 452/464/493/495/497 (transitions FB1/SB), 504-520/505/507-518 (`fb_sched_set`), 523-536/525/527 (`l1a_fb_compl`), 538/562-564 (`l1s_fbsb_req`), 570-573 (table de complétion).
- `layer1/l23_api.c` : 229-250/242/250 (`l1ctl_rx_fbsb_req`), 673-677 (dispatch `L1CTL_FBSB_REQ`).
- `include/calypso/l1_environment.h` : 8-10/30 (build flags `offsetof`), 36-52, 73-76 (`FB_DSP_TASK=5`, `SB_DSP_TASK=6`, `TCH_FB_DSP_TASK=8`, `TCH_SB_DSP_TASK=9`), 261-264 (indices `D_TOA/D_PM/D_ANGLE/D_SNR`).
- `include/calypso/dsp_api.h` : 18-23 (macros `BASE_API_*`).
- `include/calypso/dsp.h` : 12-14/22 (déclaration `dsp_api`, membres `ndb`/`db_r`/`db_w`).
- `calypso/dsp.c` : 86-89 (assignation des adresses physiques `BASE_API_NDB=0xFFD001A8`, `BASE_API_W_PAGE_0=0xFFD00000`, `BASE_API_R_PAGE_0=0xFFD00050`).

**Mock amont de référence** : `/opt/GSM/osmocom-bb/src/host/virt_phy/src/virt_prim_fbsb.c` — fichier complet 129 lignes ; 51-62 (`l1ctl_rx_fbsb_req`), 69-98 (`prim_fbsb_sync`), 80 (`sync_count>20`), 97 (`l1ctl_tx_fbsb_conf` appel), 108/110-129/114-117 (valeurs factices câblées).

**Documentation croisée** : `/opt/GSM/qemu-calypso/doc/project/CLAUDE.md:215-216` (offsets NDB, cohérents).

**Preuves live** : `/proc/3751659/environ` (process `osmo-operator-1`, PID constant sur les quatre angles d'investigation) ; `/root/qemu.log` (lignes citées : 882, 978, 1393, 5125, 6188, 11203/16756/22319/27863, 63272, et alentours `+453.853s`/`+495.115s`) ; `/root/mobile.log:37782` ; `/root/osmo-trx-ipc.log` (`@1783109245.533`) ; `/dev/shm/bursts.cfile` (croissance mesurée) ; `ss -uapn` (port 6702).
