# TODO — chemin FBSB QEMU Calypso

État au 2026-04-06 fin de session. À reprendre dans cet ordre.

## 1. Trouver pourquoi `l1s_fbdet_cmd` n'est jamais appelée

**Fait constaté** : `d_task_md` reste à `0` sur les deux pages tout au
long du run, alors que mobile envoie `L1CTL_FBSB_REQ` et reçoit
`FBSB RESP result=255` (échec après timeout). Donc `l1ctl_rx_fbsb_req`
est exécutée par le firmware mais quelque chose entre elle et
`l1s_fbdet_cmd` casse.

**Chaîne attendue** :
```
l1ctl_rx_fbsb_req()       l23_api.c:230
  → l1s_reset()
  → l1s_fbsb_req(1, …)    prim_fbsb.c:538
      → tdma_schedule_set(1, fb_sched_set, 0)
          → (à la frame suivante, sur IRQ_TPU_FRAME)
              → tdma_sched_execute()
                  → l1s_fbdet_cmd()    prim_fbsb.c:l. ~365
                      → db_w->d_task_md = FB_DSP_TASK   ← jamais atteint
```

**Pistes à investiguer (par coût croissant)** :

1. **Logs côté firmware** — patcher `prim_fbsb.c` / `sync.c` pour
   ajouter des `printd` au début de :
     - `l1s_fbsb_req`
     - `tdma_schedule_set` (déjà côté tdma_sched.c, ajouter trace)
     - `tdma_sched_execute` (avant chaque dispatch)
     - `l1s_fbdet_cmd`
   Recompiler `layer1.highram.elf`, recharger.

2. **Hooks PC côté QEMU** — dans `calypso_trx.c::calypso_tint0_do_tick`,
   après que ARM ait servi IRQ4, dumper `cpu->env.regs[15]` pendant N
   instructions ARM pour voir si le PC passe par le symbole
   `l1s_fbdet_cmd`. Nécessite la table de symboles ARM (depuis
   `arm-none-eabi-nm layer1.highram.elf`).

3. **Hypothèse alternative** — `l1ctl_rx_fbsb_req` est appelée mais
   `l1s_reset()` aborte. Vérifier que la DLCI L1CTL = 5 arrive bien
   parsée par le sercomm RX FIFO de l'UART_MODEM. Logger les bytes
   vus par sercomm_rx dans le firmware (instrumentation déjà
   discutée).

## 2. Une fois `d_task_md = FB_DSP_TASK` posté

- Vérifier que le dispatcher DSP voit la valeur et entre dans la
  zone FB-det `[0x7730..0x7990]` (PROM0).
- Le tracer `FBDET RD` (déjà en place dans `calypso_c54x.c::data_read`)
  va imprimer les 200 premières lectures DARAM avec `(addr, val, PC)`.
- **Identifier le cluster d'addr** correspondant au buffer I/Q
  d'entrée. Probablement contigu, alignement word, taille ≥ 592 mots
  pour 148 syms × 4 sps complexes.
- Mettre `CALYPSO_BSP_DARAM_ADDR=<addr>` dans `run.sh` et relancer.

## 3. Validation chaîne complète

- Avec `CALYPSO_BSP_DARAM_ADDR` set, le mode DISCOVERY s'éteint, le
  BSP DMA effectivement vers DARAM, le DSP lit les samples, fait
  son corrélation FCCH.
- Constater (espérons) `d_fb_det = 1` posé par le **vrai code DSP**.
- Mobile devrait passer du `result=255` à `result=0`.
- Si `d_fb_det` reste à 0 : bug d'opcode DSP dans la routine
  PROM0 0x7730-0x7990. Itérer sur les opcodes manquants en s'appuyant
  sur la trace FBDET (déterminer le PC où l'algo dévie).

## 4. Issues annexes à fixer

### 4.1 Link `-lm` cassé

`ninja -C build qemu-system-arm` échoue :
```
undefined reference to symbol 'sqrtf@@GLIBC_2.2.5'
/lib/x86_64-linux-gnu/libm.so.6: error adding symbols: DSO missing
```

Workaround actuel :
```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh
bash /tmp/link.sh
```

À fixer dans `meson.build` (probablement ajouter `libm` à la liste
des deps de `qemu-arm-softmmu` ou regarder pourquoi softfloat ne
propage plus la dépendance).

### 4.2 `/tmp` tmpfs 16G se remplit

`qemu.log` peut atteindre **12G** avec les logs RPTB (capés à 50 cap
dans `calypso_c54x.c` mais d'autres logs continuent à fond). Surveiller :
```
df -h /tmp
```
Killer `qemu-system-arm` libère le fichier (held open).

### 4.3 `run.sh` + tmux ne marche pas via `docker exec -d`

Lancement manuel des composants à la place. Pas critique mais
chronophage.

### 4.4 Mobile ne se connecte pas systématiquement

Sur certains runs (notamment après rebuild) mobile reste sur
"please start phone now!" sans envoyer `L1CTL_RESET`. Cause exacte
inconnue, probablement timing socket. Ne pas oublier de vérifier
`bridge stats: l1ctl→pty=` > 1 avant d'analyser.

## 5. Cleanup post-debug

- Retirer les `static int X_log = 0; if (X_log < N)` à virer une fois
  le bug fixé (calypso_c54x.c FBDET tracer, calypso_trx.c task_md/page
  loggers).
- Fixer le link `-lm` proprement.
- Fix `feedback_no_hack_functions` : virer le poke `dsp_ram[0xF8]=1`
  encore présent dans `calypso_trx.c` ligne ~506. Une fois que la
  vraie chaîne BSP marche, ce hack doit disparaître.
- Sync vers `/home/nirvana/qemu-src/` (cf. `feedback_sync_host`).
