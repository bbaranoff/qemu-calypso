# Rapport Claude web — 2026-05-14 (pré-check 7)

> État du run QEMU Calypso dans le container `trying`. Snapshot pour la
> session web suivante. Pas de nouveau patch poussé depuis la session
> 2026-05-08 night (POPM fix + 8 stubs + audit décodeur Tier A).

## TL;DR

- **Container `trying` up depuis 19:38 (≈ même image que 05-08 : `bastienbaranoff/free-bb:removed-hacks-broken-gonna-cry`)**.
- Stack Osmocom complète vivante (stp/hlr/mgw/msc/bsc/sgsn/ggsn/pcu/sip + asterisk).
- QEMU + osmocon + bridge + osmo-bts-trx + mobile L23 + tcpdump GSMTAP tous lancés.
- **Code identique à 05-08** : POPM 0x8A correctement décodé (`c54x.c:4122`, `op & 0xFF00 == 0x8A00`), ancien décodeur stubé (`if (0 && hi8 == 0x8A)` ligne 4133). Tier A décodeur (0x76 ST, F2/F3 LMS, 0x98/9A STL/STH swap, 0x80 STL, 0x8C ST T) appliqué.
- **Blocker fonctionnel inchangé** : `D_FB_DET-WR-SITE` montrait que AR3/AR4 init firmware sont *indépendants* de `CALYPSO_BSP_DARAM_ADDR` — le correlator FB-det lit DARAM `[0..0x3A3]` linéaire + wrap `[0xfc5d..]` BK=176 stride 19. Aucun AR ne tombe sur la zone BSP DMA target.
- **Le qemu.log persisté sur l'hôte est stale (05-08 17:13)** — le run actuel logue dans le container via journald + `/tmp/osmocom-logs/op1` (vide → init scripts n'écrivent pas dans le volume mounted). À reconnecter si on veut observer le run live.

## Processus actifs (docker top trying, 19:41)

| Service | PID | Binaire |
|---|---|---|
| systemd (PID 1) | 135798 | `/lib/systemd/systemd --unit=multi-user.target` |
| qemu-system-arm | 141907 | `qemu-system-arm -M calypso -cpu arm946 -serial pty -serial pty -monitor unix:/tmp/qemu-calypso-mon.sock -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf` |
| osmocon | 141943 | `osmocon -m romload -i 100 -p /dev/pts/2 -s /tmp/osmocom_l2 /opt/GSM/firmware/board/compal_e88/layer1.highram.bin -d tr` (piped to `tee /tmp/osmocon.log`) |
| bridge.py | 141960 | `python3 /opt/GSM/qemu-src/bridge.py` (piped to `tee /tmp/bridge.log`) |
| osmo-bts-trx | 141977 | `-c /etc/osmocom/osmo-bts-trx.cfg` |
| mobile (osmocom L23) | 142017 | `-c /root/.osmocom/bb/mobile_group1.cfg -d DRR,DMM,DCC,DLAPDM,DCS,DSAP,DPAG,DL1C,DSUM,DSI,DRSL,DNM` |
| tcpdump | 142019 | `-i any -w /root/mobile-gsmtap.pcap udp port 4729` (user `tss`) |
| Core CN | 141835/41/47/56/62/68/74/85/88 | osmo-stp/hlr/mgw/msc/bsc/ggsn/sgsn/pcu/sip-connector |
| asterisk | 140973 | `-g -f -p -U asterisk` |

Container PID 1 est `systemd` → les services Osmocom logguent dans journald (pas dans `docker logs`, qui ne montre qu'une ligne `[*] Initialisation du périphérique TUN pour osmo-ggsn...` de l'entrypoint).

## Volumes montés (host → container)

| Source (host) | Cible | Note |
|---|---|---|
| `/tmp/osmocom-logs/op1` | `/var/log/osmocom` | actuellement **vide côté host** → osmocom n'écrit pas dedans ou route paths ne matchent pas |
| `/home/nirvana/myconfigs/osmocom` | `/etc/osmocom` | configs `osmo-*.cfg` |
| `/home/nirvana/myconfigs/asterisk` | `/etc/asterisk` | |
| `/home/nirvana/myconfigs/osmo_root` | `/root` | qemu.log, mobile.log, pcap atterrissent ici |

## Working tree (`/home/nirvana/qemu-calypso`, branche `main`)

```
fb76f98 commit              ← HEAD
daf7724 commit
55db770 commit
bb80e78 Merge pull request #17 from bbaranoff/removing_hacks
5c73362 commit
...
```

Untracked (test/build noise, non blockant) :
- `custom.scss`, `package.json`, `package-lock.json`, `node_modules/` — résidus d'un test web/lint, pas dans le pipeline QEMU
- `dump.txt`, `qemu-calypso.txt` — captures DSP-ROM ad hoc, à archiver ou drop

Pas de diff staged ou modifié sur les sources QEMU depuis la session 2026-05-08 night. **L'état code est exactement celui décrit dans `REPORT_CLAUDE_WEB_20260508_NIGHT.md` + `AUDIT_DECODER_20260508.md`.**

## Vérifications spot du décodeur (post-05-08)

```c
/* hw/arm/calypso/calypso_c54x.c */

/* line 4122 — POPM correct */
if ((op & 0xFF00) == 0x8A00) {
    /* POPM MMR — pop top-of-stack into MMR (1-word).
     * Per tic54x-opc.c: { "popm", 0x8A00, 0xFF00, {OP_MMR} }.
     * Bug fix 2026-05-08 ...
     */
    ...
}

/* line 4133 — ancien décodeur MVDK désactivé */
if (0 && hi8 == 0x8A) {
    /* MVDK Smem, dmad — INCORRECT for 0x8Axx, see POPM above */
    ...
}

/* line 3416 — 0x76 ST #lk, Smem (2-word) appliqué Tier A */
if (hi8 == 0x76) {
    op2 = prog_fetch(s, s->pc + 1);
    consumed = 2;
    addr = resolve_smem(s, op, &ind);
    data_write(s, addr, op2);
    return consumed + s->lk_used;
}

/* line 4965 — dsp_idle_fast_forward sur ranges 0xe9ac..0xe9b7 + 0xcc62..0xcc6f */
static bool dsp_idle_fast_forward(C54xState *s, int *consumed_out) { ... }

/* 9 probes runtime (read/write/exec hooks) */
1163  WK_OPCODE_76 (writer_kind tagging pour traçage)
1244  D_FB_DET-WR-SITE  PC=0x8f51
3046  POST-BOOTSTUB-RET  RET depuis PC ≤ 0x0008
+ PC-HIST-3FB, PC-HIST-3DD, WATCH-WRITE 0x3dd2, INTM-TRANS,
  WAIT-A21A, ENTER-7740, ST1-WR
```

```c
/* hw/arm/calypso/calypso_bsp.c */
#define BSP_FN_MATCH_WINDOW  64                              /* line 65 */
bsp.daram_addr = parse_uint_env("CALYPSO_BSP_DARAM_ADDR",
                                0x3fb0);                     /* line 339 */
/* trxd_peer pré-set 127.0.0.1:5702 (race fix 05-07) */
bsp.trxd_peer.sin_port = htons(5702);                        /* line 355 */
bsp.trxd_peer_valid = true;                                  /* line 357 */
bool calypso_bsp_tx_rach_burst(uint32_t fn,
                               uint8_t bits[148]);           /* line 705 */
```

```py
# bridge.py — DL FN rewrite slot-aware (BRIDGE_DL_FN_REWRITE=slot par défaut)
def dl_slot_aware_qfn(bts_fn, current_qfn, lookahead):
    target_mod = bts_fn % 51
    delta = (target_mod - (current_qfn % 51)) % 51
    if delta > lookahead:
        return None  # drop, BCCH repeats every 51 frames
    return current_qfn + delta
# Env défauts : BRIDGE_CLK_FROM_QEMU=0 (wall-paced), BRIDGE_CLK_PERIOD=51,
#               BRIDGE_UL_FN_REWRITE=slot, BRIDGE_DL_FN_LOOKAHEAD=32
```

## Log persisté hôte (qemu.log, 1.8 Mo, 44 261 lignes — daté 2026-05-08 17:13)

Tail confirme l'**état pré-POPM-fix** : DSP coince dans la RPTB e9ab..e9b6, PC HIST top = `e981:1.82M e9XX:58824` chacun, IMR oscille `0x0000 ↔ 0x0020 (TINT0)` à PC=0xe987/0xe9b6 sans servir d'IRQ. Aucun `D_FB_DET-WR-SITE`. Aucun `RETE`. Pipeline L1CTL OK : sercomm DLCI=5, mobile demande `mt=0x01 NEIGH_PM_REQ`, firmware répond `mt=0x0e PM_CONF`.

**Le run actuel (depuis 19:38 aujourd'hui) n'a pas écrit dans ce fichier — il logue ailleurs.** Le mount `/tmp/osmocom-logs/op1` est vide sur l'hôte : soit l'entrypoint n'a pas (encore) configuré osmo-logging vers `/var/log/osmocom`, soit les services écrivent directement sur stderr/stdout récupérés par journald. À investiguer avant la prochaine itération.

## Blocker actuel rappel (inchangé depuis 05-08)

`D_FB_DET-WR-SITE` (PC=0x8f51, write `d_fb_det`) — 50 captures :

```
#1   AR1=001c AR2=0000 AR3=0000 AR4=2bc0 AR7=0000  data[AR1]=bbef BK=00b0
#50  AR1=004a AR2=fc5d AR3=03a3 AR4=2bc3 AR7=fc5d  data[AR1]=0000 BK=00b0
```

- AR3 stride +19, base 0 → lit DARAM **[0x0000..0x03A3] linéaire**
- AR2/AR7 stride −19, BK=176 → wrap circulaire **[0xfc5d..0xffed]**
- AR4 ≈ 0x2bc0 quasi-fixe → table coefficients ROM
- **Aucun AR ne tombe sur la zone BSP DMA target** (`CALYPSO_BSP_DARAM_ADDR=0x3fb0` ou `0x2bc0` ou `0x0080` → captures bit-pour-bit identiques).

L'init AR3/AR4 du firmware est *firmware-imposed*, pas configurable via env. Le mismatch source/sink est structurel.

## Symptômes débloqués 2026-05-08 (rappel)

| Symptôme | Avant POPM fix | Après |
|---|---|---|
| INTM=1 dwell perpétuel | 90.2M insn et + | INTM=0 dans POST-BOOTSTUB-RET ✓ |
| WAIT-A21A | 5.7M iters | 0 ✓ |
| ENTER-7740 | 37k figé | 0 (path différent) |
| DSP throughput | 1× | **5×** (4.3B insn / 44s) |
| RETE count | 0 | **0** (toujours) |
| `fb0_att` / `L1CTL_DATA_IND` | 0 | **0** (correlator lit zone vide) |

## TODO 2026-05-08 night → 2026-05-14 (toujours pending)

### Priorité A — tracer init AR3/AR4 firmware

`STM #lk, AR3 = 0x7713 puis lk` / `STM #lk, AR4 = 0x7714 puis lk` à grep statique
dans `calypso_dsp.txt` OU probe runtime avant insn=10 040 312 (1er hit
PC=0x8f51 observé). Décision arbre :
- Base AR3 lue depuis literal STM → firmware attend samples zone DARAM basse → path manquant ARM→DARAM à identifier
- Base AR3 lue depuis cell NDB → identifier la cell, ARM doit pousser la base
- Base AR3 lue depuis calcul → tracer caller

### Priorité B — cleanup opcode Tier B (faible urgence)

Audit `AUDIT_DECODER_20260508.md` : 19 bugs Tier B identifiés (statiquement
vérifiés contre binutils), non urgents tant que SP stable et blocker FB-det
domine. Liste résumée :
- 0x81/82/83/84/85/86/87 (STL/STH ASM variants, faux WRITA/READA/MVPD/MVDM/MVMD)
- 0x8B (POPD Smem, stubbé NOP — pop dropped silencieusement)
- 0x8E/8F (CMPS A/B → faux MVDP/PORTR)
- 0x91 (ADD #lk → faux MVKD)
- 0x70-0x75 / 0x78-0x7B / 0x7C-0x7D unimpl (MVKD/MVDK/MVDM/MVMD/PORTR/PORTW/MACP/MACD/MVPD/MVDP) — *dépend usage firmware*
- 0xA0/A1 ADD Xmem,Ymem,DST → sub-dispatch faux (cascade risk hot path MAC)
- 0xC0..0xDF parallel ST → deep audit requis

### Priorité C — comprendre `RETE=0` malgré INTM=0

3 pistes :
- (a) `c54x_interrupt_ex` jamais appelé → générateur IRQ matérielle ARM (BSP/TPU/INTH) ne fire pas de SINT vers DSP
- (b) IRQ fire mais ISR boucle sans atteindre RETE
- (c) Pending IRQ replay block (c54x.c:5148 env.) gardé par condition non remplie

À investiguer après l'init AR3/AR4 (blocker correlator plus direct et probablement plus fondamental).

## Configuration d'env effective attendue (defaults + 05-08 conventions)

```bash
# DSP / QEMU
CALYPSO_BSP_DARAM_ADDR=0x3fb0      # tested 0x2bc0/0x0080/0x0000 → no effect
CALYPSO_FBSB_SYNTH=0|1              # 1 = mobile passe vers gsm322
CALYPSO_DSP_IDLE_FF=1               # perf optim, pas un hack
CALYPSO_DSP_IDLE_RANGE=e9ac:e9b7    # default range (+cc62:cc6f)
CALYPSO_W1C_LATCH=0                 # 1 = latch capture cells a_sync_demod
CALYPSO_NDB_D_RACH_OFFSET=0x01CB    # override word d_rach NDB (DSP-version)
CALYPSO_RACH_FORCE_BSIC=unset       # 0..63 = override BSIC RACH encoder
CALYPSO_DSP_FBDET_SKIP=0            # diag, force-skip fb-det inner loop
CALYPSO_ICOUNT=auto                 # shift=auto,sleep=on,align=off
CALYPSO_DSP_ROM=/opt/GSM/calypso_dsp.txt

# Bridge
BRIDGE_CLK_FROM_QEMU=0              # wall-clock CLK IND (BTS happy)
BRIDGE_CLK_PERIOD=51                # frames
BRIDGE_DL_FN_REWRITE=slot           # slot|naive|off
BRIDGE_DL_FN_LOOKAHEAD=32           # half BSP_FN_MATCH_WINDOW=64
BRIDGE_UL_FN_REWRITE=slot

# Mobile / firmware
L1CTL_SOCK=/tmp/osmocom_l2
CALYPSO_SIM_CFG=~/.osmocom/bb/sim.cfg
```

## Pour la check 7

Pré-conditions à reverifier avant la prochaine itération :

1. **Logging du run live** : `/tmp/osmocom-logs/op1` est vide sur l'hôte. Soit on demande à l'entrypoint conteneur de pointer les services vers `/var/log/osmocom`, soit on récupère via `journalctl` dans le container, soit on `tail -f` le `qemu.log` côté `/home/nirvana/myconfigs/osmo_root/qemu.log` (mount `/root`).

2. **Confirmer l'image** : `bastienbaranoff/free-bb:removed-hacks-broken-gonna-cry` doit avoir le binaire QEMU avec POPM fix + Tier A intégrés. Si elle date d'avant 05-08, il faudra rebuild + re-tag.

3. **Probe AR3/AR4 init firmware** : la priorité A est la prochaine action critique. Soit on grep statique `7713/7714` dans `calypso_dsp.txt`, soit on ajoute un probe runtime dans `c54x_exec_one` gating sur `insn_count < 10 040 312`. Voir `hw/arm/calypso/doc/TODO.md` § Priorité A pour le snippet.

4. **Question à arbitrer** : on attaque l'init AR3/AR4 (priorité A) ou on revient sur le pending IRQ replay / cascade IMR (priorité C) ? L'argument pour A : observable direct via 1 probe, et expliquerait pourquoi le BSP target est ignoré. L'argument pour C : RETE=0 reste anormal même après POPM fix, et débloquer le service IRQ rétablirait l'ISR-driven dispatcher (DSP idle reads task slots, qui sont écrites par ISR sur INT3).

## Fichiers à reconsulter pour la check 7

```
README.md                                            ← Latest update 05-08
CLAUDE.md                                            ← Règle #1 + session pickup 05-08
hw/arm/calypso/doc/TODO.md                           ← Priorités A/B/C
hw/arm/calypso/doc/PROJECT_STATUS.md                 ← état complet
hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md         ← référence tic54x
hw/arm/calypso/calypso_c54x.c:4122                   ← POPM correct
hw/arm/calypso/calypso_c54x.c:1244                   ← D_FB_DET-WR-SITE probe
hw/arm/calypso/calypso_c54x.c:4965                   ← dsp_idle_fast_forward
hw/arm/calypso/calypso_bsp.c:65                      ← BSP_FN_MATCH_WINDOW
hw/arm/calypso/calypso_bsp.c:339                     ← daram_addr env
bridge.py                                            ← FN rewrite slot-aware
AUDIT_DECODER_20260508.md                            ← 19 bugs Tier B catalogués
REPORT_CLAUDE_WEB_20260508_NIGHT.md                  ← 0x76 fix
REPORT_CLAUDE_WEB_20260508_NIGHT_RXDONEFLAG.md       ← rxDoneFlag deadlock icount
```

## Question pour Claude web

Avec le code intégralement re-relu et l'état container vérifié (idem 05-08, aucun nouveau patch poussé) :

1. On confirme que la **priorité A** (probe init AR3/AR4) reste la prochaine action ? Ou y a-t-il une piste plus fondamentale qui m'a échappé sur la chaîne `BSP DMA → DARAM → DSP correlator` ?
2. Pour la **priorité C** (RETE=0), est-ce qu'on a un signal plus direct que la lecture de PC HIST pour discriminer (a) IRQ never fires vs (b) ISR loops sans RETE vs (c) pending-replay gate ? Idéalement un compteur côté QEMU à instrumenter.
3. Pour le **logging du run live** : tu préfères qu'on rebranche `/var/log/osmocom` côté entrypoint, ou qu'on accepte `journalctl` + `/root/qemu.log` mount comme source canonique pour la check 7 ?
