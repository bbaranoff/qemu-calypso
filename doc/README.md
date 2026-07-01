# Calypso DSP — sous-système d'émulation (`hw/arm/calypso`)

Point d'entrée du sous-système d'émulation du baseband Calypso (ARM7 + DSP
TMS320C54x). Ce README oriente ; les diagrammes détaillés sont dans
[`schematics.md`](schematics.md) et l'index documentaire complet dans
[`doc_master.md`](doc_master.md). Voir aussi `archive/STATUS.md`, `CALYPSO_HW.md`,
`REPORT_CLAUDE_WEB_IQ_CABLAGE.md` et `C54X_DECODER_AUDIT.md`.

## 1. But

Faire tourner le **vrai** DSP c54x émulé (ROM d'origine) pour qu'il détecte
FB/FCCH et écrive lui-même `d_fb_det != 0` dans la NDB, **en remplacement du
mock gr-gsm** (`calypso_dsp_shunt.c`, FB/SB cannés). Le chemin cible est le
chemin d'exécution réel (`CALYPSO_DSP=c54x`, shunt OFF), pas le shunt.

**Contrainte rule#1 : aucun hack.** On ne poke JAMAIS l'état interne du DSP
(ni `d_fb_det`, ni `IMR`, ni `INTM`, ni le vecteur firmware). On corrige
uniquement le **câblage de l'émulateur** (livraison d'IRQ, vecteur, feed I/Q,
handshake ARM→DSP). Toute correction doit être justifiable côté matériel émulé.

## 2. Architecture en 10 lignes

1. Cœur **ARM7** (L1 firmware) + cœur **DSP c54x** (ROM L1 bas niveau), deux CPU distincts.
2. Mailbox **API-RAM partagée** : MMIO ARM `0xFFD00000` (64 KiB) ⇄ mot data DSP `0x0800` (`C54X_API_BASE`), loi `mot = 0x0800 + offset/2` (`calypso_trx.c:604/247`, `calypso_c54x.h:21`).
3. Trois vues physiques cohérentes par double-écriture : `dsp_ram[]` (ARM), `dsp->data[]` (DARAM), `dsp->api_ram[]` aliasé via `c54x_set_api_ram` (`calypso_trx.c:2374`).
4. Champs mailbox mappés : `d_task_md` (0x0804/0x0818), `d_dsp_page` (0x08D4), `d_fb_det` (0x08F8 / ARM 0x01F0) — mapping vérifié correct.
5. **TPU frame IRQ** : `calypso_tdma_tick` (`calypso_trx.c:1618`) génère chaque frame (~4.615 ms) l'INT3/FRAME DSP (`calypso_trx.c:1745-1793`).
6. Livraison IRQ : `c54x_interrupt_ex(dsp, vec, imr_bit)` (`calypso_c54x.c:13533`), tir frame à `calypso_trx.c:1786` avec `C54X_INT_FRAME_VEC=19 / _BIT=3` (`calypso_c54x.h:126-127`).
7. **BSP / I/Q** : `bsp_drain_cb` draine le socket TRXD et `calypso_bsp_deliver_buffered` applique l'AFC puis écrit la fenêtre corrélateur DARAM `0x2a00` (`calypso_bsp.c:1243/1259`, défaut `daram_addr=0x2a00 len=296` `:795`).
8. Vecteur DSP : `pc = (IPTR*0x80) + vec*4` (`calypso_c54x.c:13618`) ; IPTR relocalisé au boot vers la page RAM firmware.
9. Mock gr-gsm optionnel (`calypso_dsp_shunt.c`) : réplique DMA + FB/SB cannés, chemin de diag uniquement.
10. Diagrammes détaillés : voir `schematics.md`.

## 3. Fichiers du sous-système

| Fichier | Rôle (1 ligne) |
|---|---|
| `calypso_c54x.c` | Décodeur/CPU TMS320C54x : opcodes, `c54x_interrupt_ex` (IRQ, IFR/IMR/INTM, vecteur), API-RAM, PORTR. |
| `calypso_trx.c` | TPU + TDMA tick + frame IRQ (`:1786`) + mailbox API-RAM MMIO `0xFFD00000` + handshake go-live ARM. |
| `calypso_bsp.c` | BSP : drain TRXD, AFC, feed I/Q en DARAM `0x2a00`, tir INT3/BRINT0 (`deliver_buffered`). |
| `calypso_dsp_shunt.c` | Mock gr-gsm : route c54x, réplique DMA write-page, FB/SB cannés (chemin de diag, non défaut). |
| `calypso_mb.c` | Wiring machine : init DSP/shunt, remise du handle DSP réel, chargement sections/PROM/blob DARAM. |
| `calypso_soc.c` | Wiring SoC : instanciation des cœurs ARM/DSP et des périphériques Calypso. |
| `calypso_orch.h` | Flag `CALYPSO_ORCH` : ON = injection L1 côté hôte ; défaut (unset/0) = exécution DSP réel. |

## 4. Build & run

**Build (ninja, dans le container `osmo-operator-1`)** — le répertoire de build
confirmé dans ce container est `/opt/GSM/qemu-src/build` (contient `build.ninja`,
produit `qemu-system-arm`) :

```sh
docker exec osmo-operator-1 bash -c 'cd /opt/GSM/qemu-src/build && ninja'
```

(Le `build.sh` du repo cible un autre host/container — `CONTAINER="trying"`,
`/root/qemu/build` — donc **ne pas** l'utiliser tel quel ici ; la commande
ci-dessus est la forme adaptée à ce container.)

**Run** — le pipeline se lance par `run.sh` / `start-clean.sh`
(`/opt/GSM/qemu-src/`). `start-clean.sh` source `calypso.env` puis exec `run.sh`.
Défauts confirmés dans `run.sh` pour le chemin DSP réel :

- `CALYPSO_DSP=c54x` (défaut, `run.sh` : `: "${CALYPSO_DSP:=c54x}"`)
- `CALYPSO_DSP_REG_MODE=c54x` (défaut, `run.sh:1662` — vrai FB ; opt-out `=bin`)
- `CALYPSO_MODE=full` (préset ; défaut réel du fichier = `full-grgsm`, `run.sh:1124`)
- `CALYPSO_DSP_SHUNT=0` (mock OFF, `run.sh:594` — `1` = FB/SB cannés)

```sh
docker exec osmo-operator-1 bash -c \
  'cd /opt/GSM/qemu-src && CALYPSO_MODE=full CALYPSO_DSP_SHUNT=0 CALYPSO_DSP_REG_MODE=c54x ./run.sh'
```

**À vérifier** : `CALYPSO_MODE=full` est accepté comme préset mais le défaut
interne de `run.sh` est `full-grgsm` (`run.sh:1124`) ; confirmer le mode voulu
avant un run de référence. Log runtime : `/root/qemu.log` (grep avec `-a`).

## 5. État courant

**Objectif non atteint : `d_fb_det` reste 0.** ARM lit `d_fb_det(0x01F0)=0x0000`,
`fbsb FB0_SEARCH (real DSP path) fb0_ret=0`.

**Blocker primaire (domaine wake/IRQ) — CONFIRMÉ ce run :** les interruptions du
DSP restent masquées en permanence, donc aucune IRQ livrée n'entre jamais dans
une ISR :

- Le boot ROM efface l'IMR : `BOOT-MMR-WR #7 PC=0xb37e op=0x7700 IMR 0x3000 -> 0x0000` (insn 1047), **jamais ré-armé** (`IMR=0x0000` tout le run, `0` ligne `INTM=0`, `0` RETE). `unmasked=(s->imr & (1<<imr_bit))` `calypso_c54x.c:13593`.
- La frame IRQ est bien tirée chaque frame : `FRAME-GATE tpu_armed=1 ... force=0 -> fire=1` (`calypso_trx.c:1782-1786`), mais arrive en **vec19/bit3** avec IMR=0 → `unmasked=false` → DSP `idle=1` prend la branche masquée `s->pc++` (`calypso_c54x.c:13622`) et **ne vectorise jamais**.
- Conséquence : INTM ne se libère jamais → l'ordonnanceur/corrélateur par frame ne tourne jamais → `d_fb_det` reste 0.

**Pièce manquante (la plus causalement proche) :** la ligne frame TPU est livrée
au **mauvais vecteur DSP** (vec19/bit3) et en mode wake-only, alors que
l'ordonnanceur FB firmware attend **vec28/bit12** (chaîne `0x7234 → CALL 0xa4e4 →
corrélateur → d_fb_det`), lequel une fois atteint arme lui-même `IMR=0x52fd`
au go-live `0xa582` (auto-entretien ensuite). Le seul code qui reciblerait la
ligne vers vec28/bit12 ET force-vectoriserait malgré IMR=0 est le bloc
`CALYPSO_DSP_FRAME_VEC28` à **`calypso_c54x.c:13547-13565`**, **OFF par défaut**
(`VEC28-FORCE count=0`) — bien que son gate `d_dsp_page bit1` (B_GSM_TASK) soit
satisfait au runtime (`REAL_page(08D4)=0x0002` dès fn≈1206).

**Corrections apportées à l'audit (vérifiées ce run) :** (a) IMR est `0x0000`
tout le run et non `0x3000` (les 514 lignes `3000` sont la valeur reset
pré-boot) ; (b) IPTR NE reste PAS `0x1FF` — la ROM le relocalise tôt
(`PMST-WR #4 val=0x0038 insn=1874`), donc le stub RET `0xFFCC` n'est touché que
pendant le boot initial ; (c) `d_dsp_page` bit1 (B_GSM_TASK) EST bien positionné
au runtime.

**Câblage mailbox et feed I/Q : OK (non-blockers).** La mailbox ARM⇄DSP porte
correctement les champs dans les deux sens (`delivered=125093` bursts post-AFC
en DARAM `0x2a00`). Le côté lecture I/Q est « mort » uniquement en aval : le
corrélateur ne tourne pas, donc `0x2a00` est écrit mais jamais lu — symptôme,
pas cause. **Pas besoin d'un nouveau pipe `calypso_arm2dsp.c`** : le canal
existe et fonctionne ; ce qui manque est une valeur/IRQ (l'enable go-live et/ou
la ligne frame correctement vectorisée), pas un canal.

Le fix est un câblage d'interruption côté émulateur (`calypso_c54x.c:13547-13565`
+ tir `calypso_trx.c:1786` / `calypso_c54x.h:126-127`), pas un poke d'état, pas
un nouveau module. Un refactor (consolidation en `calypso_dspmb.c`) est
**orthogonal** — hygiène à faire APRÈS le fix, il ne déplace pas `d_fb_det`.

## 6. Pour aller plus loin

Index documentaire complet : **[`doc_master.md`](doc_master.md)**. Références
clés : `archive/STATUS.md`, `TODO.md`, `REPORT_CLAUDE_WEB_IQ_CABLAGE.md`,
`C54X_DECODER_AUDIT.md`, `archive/SESSION_20260429.md`, `SERCOMM_GATE_ARCHITECTURE.md`,
`CALYPSO_HW.md`, `FB_CORRELATOR_PIPELINE.md`.
