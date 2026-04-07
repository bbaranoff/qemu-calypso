# TODO — chemin FBSB QEMU Calypso

État au 2026-04-06 fin de session 3.

## Session 3 — acquis (2026-04-06 soir)

Saut massif sur le pipeline FB-det. snr passé de 0 → 30-60 stable, inner loop
identifié, buffer FB localisé, plusieurs bugs décodeur majeurs corrigés.

### Bugs décodeur fixés

1. **`LD Smem, dst` manquant entièrement** (`0x1000` mask `0xFE00`).
   Le commentaire dans le code disait "real LD is at 0x1000, NOT in 0x6xxx
   range" — mais on avait supprimé le faux 0x6xxx LD sans jamais ajouter le
   vrai. Toutes les `LD *ARx, A/B` (1 mot) tombaient en UNIMPL silencieux,
   d'où l'inner loop FB-det qui spinnait sans rien lire. Ajout également de
   `LD Smem, TS, dst` à `0x1400` mask `0xFE00`.

2. **`resolve_smem` mod 12..15 entièrement faux** (très gros impact).
   Per binutils `tic54x-dis.c sprint_indirect_address`, la table correcte est :
   - `mod=12` → `*ARx(lk)` (addr = AR + lk, AR inchangé)
   - `mod=13` → `*+ARx(lk)` (pre-add : AR += lk, addr = AR)
   - `mod=14` → `*+ARx(lk)%` (pre-add circulaire)
   - `mod=15` → `*(lk)` **absolue**, sans aucun ARx
   On avait `mod=12` = *(lk) absolue et `mod=15` = AR+lk circulaire — tout
   décalé. Symptôme du bug : freeze à PC=0xdfc7 (`ST #0x3fba, *(0x0003)`)
   où l'ST écrivait dans `AR[arp]+0x3fba` au lieu de `0x0003`, atterrissant
   sur d_dsp_page selon la valeur d'AR — 57293 corruptions identiques par
   run avant fix, 0-1 après.
   **Ce fix est probablement la cause de plein de comportements bizarres
   antérieurs** dans le DSP au-delà du chemin FB.

### Fixes pipeline

3. **`calypso_bsp_rx_burst` circular append** : chaque burst BSP (1184 mots)
   s'append à un offset wrap dans `[daram_addr .. daram_addr+daram_len)` au
   lieu d'écraser le même bloc. Modélise la BSP qui remplit un long buffer
   FB sample sur de multiples bursts série avant que le corrélateur ne
   sweep. Indispensable une fois le buffer FB identifié comme >> 1184 mots.

4. **`inject_cfile.py` SCALE 16384 → 30000** : amplitude tone proche du clip
   int16. Vérifié que ce n'est PAS le facteur limitant (snr identique).

### Découvertes structurelles

5. **Buffer sample FB localisé à `DARAM[0x021f]`**, longueur ≈ 12000 mots
   (≈ 6000 IQ pairs, ≈ 1475 syms ≈ 10 sub-bursts). Tracé via `FBLOOP-RD`
   profiler en logguant les data_read avec PC dans la zone de l'inner loop
   et reportant min/max d'AR4 (le pointeur sample). AR4 part de 0x021f et
   balaye linéairement vers le haut. **Le `DARAM_ADDR=0x62 LEN=128` qu'on
   utilisait jusqu'ici était faux d'un facteur 100**, c'est pourquoi la
   FB-det ne triggait pas même avec un tone parfait : les samples étaient
   déposés dans un coin du DARAM que le DSP ne lit pas en mode idle.

   ⇒ env à utiliser maintenant :
   ```
   CALYPSO_BSP_DARAM_ADDR=0x021f CALYPSO_BSP_DARAM_LEN=12000
   CALYPSO_BSP_BYPASS_BDLENA=1
   ```

6. **Inner loop corrélateur FB localisé** à PROM0 `0xa10d..0xa116` :
   ```
   a10d: f272 a114        RPTBD pmad=0xa114
   a10f: 1092             LD *AR2,A          ← lecture sample
   a110: f495             NOP
   a111: f0e2             SFTL B,1,A
   a112: 3c87             ?
   a113: 8295             ?
   a114: f280             (fin RPTB body)
   a115: 6e89 a10d        BANZD a10d         ← outer
   ```
   AR2 = pointeur DROM coeffs (constant 0xcc7e pendant la phase active),
   AR3 = counter (0x000b), AR4 = pointeur sample read, AR5 = return addr
   du caller (0x0a14).

7. **DROM bien chargée et lue** : tracer `COEFF-RD` confirme que
   `data[0x9000..0xdfff]` contient les vraies valeurs du dump
   (`data[0xc21b]=0x1306`, `0xc60d=0xe10f`, etc., non-zéro). La table de
   coeffs FCCH N'EST PAS le problème. À l'init, la DSP fait un copy-loop
   de la DROM via PC=0xfd23, AR2 incrémenté par 0x3f2 — vraie phase
   d'initialisation, normale.

### Métriques avant/après session 3

| Métrique | Début session | Fin session |
|---|---|---|
| `FBSB_CONF snr` | 0 | 30-60 stable |
| Buffer FB connu | non | oui (0x021f, 12000w) |
| Inner loop tracé | non | oui (a10d..a116) |
| `LD Smem,dst` (0x1000) | UNIMPL | OK |
| `resolve_smem` mod 12..15 | shifté/cassé | conforme binutils |
| BSP DMA | single-shot écrase | circular append |
| Freeze PC=0xdfc7 | 57k corruptions/run | 0-1 |
| Coeffs DROM | inconnu | confirmés présents |

## TODO session 4

### Priorité A — Facteur 30× sur le seuil corrélation

Résultat actuel : `snr` brut côté DSP ≈ 240..480 (= 30..60 << 3).
Seuil `d_fb_thr_det_iacq = 0x3333 = 13107`.
Il manque un facteur ~30× pour que `d_fb_det = 1`.

Hypothèses (à creuser dans cet ordre) :

1. **Le tracer `COEFF-RD` ne capte que les premières 32 reads pendant la
   phase init** (PC=0xfd23, copy DROM). Il faut RE-CIBLER pour logguer
   les reads pendant la phase corrélation active (PC ∈ a10d..a116, AR2
   constant 0xcc7e). Vérifier que `data[0xcc7e..0xcc7e+N]` n'est pas
   réellement zéro/garbage à ce moment-là (peut-être que la DROM est
   loadée mais qu'une copie DARAM intermédiaire est mal synchronisée).
2. **Format samples** : on envoie int16 LE I/Q interleaved scale ×30000.
   Le DSP corrélateur attend peut-être :
   - I-only (ignorer Q) ?
   - I et Q dans deux buffers séparés au lieu d'interleaved ?
   - Signe inversé ?
   - Endian différent ?
   À tester en modifiant `inject_cfile.py` mode tone avec différentes
   permutations.
3. **Disasm propre du corrélateur** entre PROM0 0xa0e2 (point d'entrée
   FB-det dispatcher confirmé en PC HIST) et 0xa116 (fin BANZD). Comprendre
   ce que SFTL B,1,A + NOP + ? + ? + RPTB calcule réellement. Confirmer
   que iacq est bien la valeur comparée.
4. **Vérifier la lecture de `d_fb_thr_det_iacq` côté DSP** : ajouter
   un tracer sur l'adresse api_ram correspondante pour s'assurer que
   le DSP lit bien 0x3333 (= la valeur uploadée par L1) et pas 0xFFFF
   ou 0 (cas où dsp_set_params ne serait pas mirrorée).

### Priorité B — Cleanup BDLENA bypass (inchangé session 2)

L1 n'arme BDLENA que ~10 fois par run au lieu des centaines attendues.
Probablement même type de mismatch buffer/format pour PM_CONF (qui
retourne pm=0). À fixer une fois la chaîne FB validée.

### Priorité C — Cleanup profilers / logs

`FBLOOP-RD`, `COEFF-RD`, `DARAM-RD`, `NDB-WR`, `STACK-WR`, `FBDET-RD`,
`FB-CALL` etc. — beaucoup de tracers diagnostic encore en place. À
factoriser ou guarder par flag env une fois la FB-det au vert.

## Backups session 3

3 tar Docker dans `/home/nirvana/` (24G chacun) :
- `osmocom-run-sftl-20260406.tar` (22:04) — état SFTL milestone
- `osmocom-run-mod-fix-20260406-2300.tar` (23:04) — milestone mod fix +
  circular BSP, snr=30-60 stable, **état le plus testé** de la session
- `osmocom-run-coeff-rd-20260406-2314.tar` (23:18) — latest, contient
  en plus le tracer COEFF-RD diagnostic

Sources synced sur les 3 emplacements :
- `/home/nirvana/qemu-src` (working tree)
- `/home/nirvana/qemu-calypso` (git tracked)
- container `22da3f2aefb9:/opt/GSM/qemu-src`

---

État au 2026-04-06 fin de session 2.

## Acquis cette session

1. **DSP runaway éliminé** par 4 fixes décodeur c54x (commit `057b4e9`) :
   - `STLM/POPM/LDMM` mask `& 0x1F` → `& 0x7F` (le STLM B,MMR_0x38 wrappait vers
     SP et écrasait la stack pointer).
   - `0x76xx` était décodé comme `LDM MMR,dst` (1 mot) au lieu de
     `ST #lk, Smem` (2 mots). Ce seul bug expliquait toute la cascade
     de runaways au PROM1 1f2a0 (ST #idx,*(0xe1) table).
   - `CMPR cond, ARx` (`0xF4A8` mask `0xFCF8`) implémenté pour la famille F6.
   - F68x-F6Fx catch-all MVDD bidon retiré, remplacé par NOP loggé.

2. **`E8/E9` LD #k8u, A/B** corrigés (les deux étaient mal décodés comme
   CMPR / CC, le faux CC poussait `PC+2` dans `data[SP-1]` à chaque appel
   et corrompait `d_fb_det`/`d_dsp_page` quand SP était wedged dans NDB).
   *Pas encore committé.*

3. **`api_ram` mirror** : `calypso_dsp_write` mirror chaque write ARM
   immédiatement vers `dsp->api_ram[]` au même offset mot, modélisant le
   SRAM physiquement partagé entre ARM et DSP. Avant ce fix, le DSP ne
   voyait JAMAIS les commandes `d_task_md` que l'ARM postait, et le firmware
   ne lançait jamais aucune tâche. Après le fix : 200+ d_task_md events,
   FB-det loop activement exécutée, NDB writes côté DSP.
   *Pas encore committé.*

4. **`CALYPSO_BSP_BYPASS_BDLENA=1`** : env var diagnostic dans `calypso_bsp.c`
   qui désactive le gating IOTA/TPU et laisse passer tous les bursts TN=0.
   Permet de valider la chaîne BSP→DARAM→FB-det sans dépendre de l'arming
   BDLENA par la L1 (qui actuellement n'arme la fenêtre que ~10 fois par run).
   *Pas encore committé.*

5. **`inject_cfile.py`** + **`run_tmp.sh`** : injection directe d'un cfile
   RTL-SDR (ou d'un tone synthétique +67.7 kHz via `--tone`) au format
   TRXDv0 vers UDP 6702. Bypasse `osmo-bts-trx` et le relais TRXD du bridge.
   *Pas encore committé.*

6. **Profilers `DARAM-RD` et `NDB-WR`** dans `calypso_c54x.c` (dedup par
   adresse, capés). Ont permis d'identifier que :
   - Le buffer FB est bien `DARAM[0x0062..0x00e1]` (128 mots).
   - Le DSP lit aussi des triplets sparse `0x010e..0x059e` (probable table
     coeffs FCCH ou pondération).
   - L'unique cellule API/NDB que le DSP écrit normalement est
     `data[0x0c31] = 0xfffe` à insn ~5M (= `BASE_API_PARAM` byte 0).

## État firmware actuel

- L1 ARM vivante, scheduler L1S actif, poste `d_task_md = 1` (PM) et
  `d_task_md = 5` (FB) en alternance sur les deux pages.
- DSP exécute la FB-det loop (PC HIST top : `82e9 / 7700 / 7701 / 1111`
  avec ~30k+ hits chacun).
- Bursts injectés via cfile ou tone → BSP DMA → DARAM[0x62..0xe1] OK.
- **MAIS** : `d_fb_det` n'est jamais posé à 1 par le DSP, même avec un
  tone parfait. `FBSB_CONF result=255` (timeout) en boucle.

## TODO ordre de priorité

### A. Bug `0x6184` à PC=0xe26e (en cours)

**Symptôme** : trace tail montre :
```
STACK WR [0xEEB3] = 0xe040 PC=0xe26e SP=0xe040
IMR change 0xe040 → 0x0000 PC=0xe260
DSP WR d_dsp_page = 0xe040 PC=0xe26e *** CORRUPT
DSP WR d_fb_det   = 0xe040 PC=0xe26e *** GARBAGE
```

À PC=0xe26e l'opcode PROM est **`0x6184`** (vérifié : PDROM ligne `0e260`,
position +0xe). Cet opcode :
- Charge `0xe040` dans **SP** (SP wedged dans PROM/IMR area).
- Toggle IMR avec la même valeur 0xe040.
- Une fois SP wedged, les pushes suivants atterrissent dans NDB
  (`d_dsp_page`, `d_fb_det`).

**Action** : identifier le vrai sens de `0x61xx` dans `tic54x-opc.c`
et corriger le décodeur.

```sh
docker exec 22da3f2aefb9 grep -nE "0x61[0-9a-f]{2}" /root/gnuarm/src/binutils-2.21.1/opcodes/tic54x-opc.c
```

### B. Pourquoi FB-det ne déclenche pas même avec un tone parfait

Une fois A fixé, si avec `--tone` la FB-det ne pose toujours pas
`d_fb_det=1`, hypothèses :

1. **Format/scale samples** : on envoie int16 LE I/Q interleaved scale
   ×16384. Le DSP attend peut-être un autre format (signe inversé,
   amplitude différente, ou que I-channel uniquement).
2. **Buffer trop court** : 128 mots = 64 IQ pairs = 16 syms à 4 sps.
   FCCH = 148 syms. Soit la corrélation est itérative sur plusieurs
   DMAs, soit le buffer est en réalité plus long et le tracer ne l'a
   pas vu (mode lecture sparse avec auto-incrément).
3. **NDB params manquants** : `d_fb_thr_det_iacq` (threshold acquisition)
   et autres paramètres FB doivent être posés par L1 init. Vérifier
   qu'ils sont bien dans le mirror api_ram.
4. **Il manque encore d'autres opcodes** : tracer `UNIMPL` et corriger
   itérativement.

### C. Fix BDLENA arming pour pouvoir retirer le bypass

Actuellement la L1 ne fait que ~10 BDLENA pulses par run au lieu des
centaines attendues. Cause probable : la L1 reste en mode PM scan parce
que `pm=0` (PM_CONF retourne 0), donc elle ne commit jamais sur ARFCN
514. Une fois la chaîne FB validée avec le bypass diagnostic, attaquer
le bug PM (probablement même genre de mismatch buffer/format).

### D. Cleanup

- Retirer `bsp.bypass_bdlena` une fois que BDLENA est correctement armée.
- Cleanup des `static int X_log` profilers.
- Le hack `dsp_ram[0xF8]=1` mentionné dans la version précédente du TODO
  doit être vérifié — pas sûr qu'il existe encore après le mirror api_ram.

## Issues annexes (inchangées)

### Link `-lm` cassé

`ninja -C build qemu-system-arm` échoue avec `undefined reference to
sqrtf@@GLIBC_2.2.5`. Workaround manuel :
```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh
bash /tmp/link.sh
```

### `/tmp` tmpfs 16G se remplit

`qemu.log` peut atteindre **12G** avec les logs PC HIST + DARAM-RD/NDB-WR.
Surveiller `df -h /tmp`. Killer `qemu-system-arm` libère le fichier.
