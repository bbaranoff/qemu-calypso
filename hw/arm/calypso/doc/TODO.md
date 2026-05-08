# TODO — chemin FBSB QEMU Calypso

État au **2026-05-08 fin de session** (après POPM fix + batch 8 stubs).

## État courant

**Run end-to-end** : QEMU + osmocon natif + bridge + BTS + mobile.
- Pipeline ARM ↔ TPU ↔ BSP ↔ DSP fonctionnel
- Mobile L23 progresse jusqu'à gsm322 cell selection avec `CALYPSO_FBSB_SYNTH=1`
- Avec SYNTH=0, mobile bloqué pré-FBSB (firmware n'émet pas FBSB_CONF)

**Bug racine restant** : DSP correlator FB-det lit DARAM zones internes
(`[0x0000..0x03A3]` linéaire + wrap `[0xfc5d..]` BK=176 stride 19) **indépendamment**
de `CALYPSO_BSP_DARAM_ADDR`. AR3/AR4 init impose ces pointers.

## Prochaine action critique (Priorité A — 1 probe statique)

**Tracer init AR3 et AR4 dans le firmware avant insn=10 040 312** (1ère
exécution observée de PC=0x8f51 = write d_fb_det).

### Option 1 — grep statique
```bash
# AR3 = MMR 0x13, donc STM #lk, AR3 = 0x7713 puis lk
grep -nE "7713 [0-9a-f]{4}" /opt/GSM/qemu-src/calypso_dsp.txt | head
# AR4 = MMR 0x14, donc STM #lk, AR4 = 0x7714 puis lk
grep -nE "7714 [0-9a-f]{4}" /opt/GSM/qemu-src/calypso_dsp.txt | head
# Cibler les sites dont l'immediate ressemble aux bases observées :
#   AR3 base = 0x0000, AR4 base = 0x2bc0
```

### Option 2 — probe runtime
```c
/* Ajouter dans c54x_exec_one() */
if (s->insn_count < 10040312 &&
    (op == 0x7713 || op == 0x7714)) {
    uint16_t lk = prog_fetch(s, s->pc + 1);
    C54_LOG("AR-INIT op=0x%04x lk=0x%04x PC=0x%04x insn=%u",
            op, lk, s->pc, s->insn_count);
}
/* Aussi tracer STLM B,AR3 (op 0x8B+...) ou STLM A,AR3 */
```

### Décision arbre

- **Base AR3 lue depuis literal STM** → firmware attend les samples en
  zone DARAM bas. Le BSP doit livrer là, ou une routine ARM doit copier
  depuis BSP target vers cette zone (path manquant à identifier).
- **Base AR3 lue depuis cell NDB** → identifier la cell. ARM est censé
  pousser la base depuis une variable configurable.
- **Base AR3 lue depuis registre/calcul** → tracer caller pour comprendre
  le contexte.

## Priorité B — Cleanup opcode (low priority)

`doc/opcodes/tic54x_hi8_map.md` liste **9 misclassifications** identifiées
2026-05-08. Statut :
- ✓ `0x8A` (POPM) — fixé, sémantique correcte
- ⏳ `0x80, 0x8B, 0xAA/AB, 0xC5, 0xCD, 0xCE, 0xDD, 0xDE` — stubés en NOP

Pour implémentation sémantiquement correcte des familles **ST||OP parallèles**
(0xC0..0xDF range = ST src,Ymem || ADD/SUB/MPY/MAC/MAS Xmem, dst), créer
un handler dédié inspiré du ST||LD existant à `0xC8..0xCB` (ligne 4773).
**Non urgent** tant que SP est stable et que blocker FB-det persiste.

## Priorité C — Comprendre RETE=0

Malgré INTM=0 post-fix POPM, `RETE` count reste à 0 sur 4.3B insn.
Possibilités :
- (a) `c54x_interrupt_ex` jamais appelé → générateur d'IRQ matérielle
  côté ARM (BSP, TPU, INTH) ne fire pas de SINT vers DSP
- (b) IRQ fire mais ISR boucle sans atteindre RETE
- (c) Pending IRQ replay block (ligne 5148) gardé par condition non remplie

À investiguer après tracé init AR3/AR4 (le blocker correlator est plus
direct et probablement plus fondamental).

## Run config courante

```bash
# Variables clés
CALYPSO_BSP_DARAM_ADDR=0x3fb0   # default; testé 0x2bc0 / 0x0080 / 0x0000
CALYPSO_FBSB_SYNTH=0|1          # 1 fait passer mobile vers gsm322
CALYPSO_DSP_IDLE_FF=1           # default ON, perf optim, pas un hack
BRIDGE_DL_FN_REWRITE=slot       # default
BRIDGE_CLK_FROM_QEMU=0          # wall-paced (BTS happy)

# Lancement
docker exec -it trying bash -c "killall -9 qemu-system-arm 2>/dev/null; \
  rm -f /tmp/osmocom_l2 /tmp/qemu-calypso-mon.sock; \
  bash /opt/GSM/qemu-src/run.sh"
```

## Probes runtime en place (dans calypso_c54x.c)

| Tag | PC ciblé | Ce qu'il logue |
|-----|----------|----------------|
| `PC-HIST-3FB` | data_read addr ∈ [0x3fb0..0x3fbf] | Top PCs lecteurs zone BSP |
| `PC-HIST-3DD` | data_read addr ∈ [0x3dcf..0x3dd5] | Top PCs zone scratch dominante |
| `WATCH-WRITE 0x3dd2` | data_write addr=0x3dd2 | PCs writers + valeurs |
| `INTM-TRANS` | exec_one start, transition INTM | 0→1 et 1→0 avec cause |
| `WAIT-A21A` | PC=0xa21a | INTM/IMR/IFR/ST0/ST1/SP snapshot |
| `ENTER-7740` | PC=0x7740 | Caller chain + AR + insn |
| `ST1-WR` | STM #lk, ST1 (op 0x7707) | Toutes écritures ST1 |
| `POST-BOOTSTUB-RET` | RET depuis PC ≤ 0x0008 | Task PC poppé du nouveau stack |
| **`D_FB_DET-WR-SITE`** | **PC=0x8f51** | **AR0..AR7 + data[AR0/1/2] + BK + A** |

## État branche

Working tree **commit `06ab6f3`** (push fait avant cette session) + edits
2026-05-08 non-commités :
- `hw/arm/calypso/calypso_c54x.c` (POPM fix + 8 stubs + 9 probes)
- `hw/arm/calypso/doc/opcodes/tic54x_hi8_map.md` (créé)
- `hw/arm/calypso/doc/PROJECT_STATUS.md` (mis à jour)
- `hw/arm/calypso/doc/TODO.md` (ce fichier)

3 trees synchronisés (`/home/nirvana/qemu-src/`, `/home/nirvana/qemu-calypso/`,
container `/opt/GSM/qemu-src/`) — md5 vérifié.

## Issues annexes (inchangées)

### Link `-lm` cassé (workaround manuel)
```bash
cd /opt/GSM/qemu-src/build
ninja -t commands qemu-system-arm | tail -1 > /tmp/link.sh
sed -i 's|$| -lm|' /tmp/link.sh && bash /tmp/link.sh
```

### `/tmp` tmpfs 16G
`qemu.log` peut atteindre 12G+ avec tous les tracers. Surveiller `df -h /tmp`.
