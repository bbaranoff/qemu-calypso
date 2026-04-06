# TODO — chemin FBSB QEMU Calypso

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
