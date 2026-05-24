# Rapport Claude web — 2026-05-23 : nettoyage `CALYPSO_FORCE_RX_DONE` (200 Hz brute → event-driven)

> Suite des sessions du 23 mai (12+ pivots débunkés, STL/STH fix appliqué,
> hybrid bridge implémenté). Cette session pivote sur le hack rx_done que
> tu avais flaggé plusieurs fois comme "200 Hz aveugle, masque la cause TCG,
> trois sites redondants".

## TL;DR

- **Hybrid bridge (CLK_FROM_QEMU=1) marche** — BTS survit 64s (vs 5s avant).
  Wall-paced timing + qfn-anchored virtual fn + dup-suppression. Patch live.
- **DSP fait du VRAI FB-det sans synth** — 4× `D_FB_DET SET val=0x001e` à
  PC=0x778a (FB-det handler). Chain BTS→bridge→DSP→ARM L1 entirely alive.
- **Le mur résiduel = timing** : SET-window de 4.5ms wall (clobbered par
  0x821a thrash) vs poll ARM 167ms interval → ARM ne catch jamais
  d_fb_det≠0 → no FBSB_CONF → mobile stuck à init.
- **`CALYPSO_FORCE_RX_DONE` actuellement à 1, ICOUNT=auto active**.
  Sans le hack, le firmware busy-loop @ 0x822b90 forever (TCG Task #29).
  Avec le hack, 200 Hz cpu_exit perturbe le timing global.
- **Proposition** : remplacer la branche kick-CB-200Hz par event-driven
  (PC-trap busy-loop OU SIM IRQ entry trigger). Garder la branche
  SIM_IT-read (event-driven, déjà acceptable).

## 1. État du hack actuel — 3 sites

Le hack `CALYPSO_FORCE_RX_DONE=1` écrit `1` à `rxDoneFlag` (firmware data
@ 0x008302d4 ou 0x008302a0 selon le path) à 3 endroits :

| Site | Fichier:Ligne | Trigger | Fréquence | Verdict |
|---|---|---|---|---|
| **A** | `calypso_trx.c:1038` (kick CB) | Timer REALTIME 5ms wall | **200 Hz blind** | **Web's "degeu"** — vire ça |
| **B** | `calypso_sim.c:617` (SIM_IT read) | Lecture register SIM_IT par firmware | Event-driven (~1/SIM op) | Acceptable — keep |
| **C** | `calypso_sim.c:232` (calypso_sim_init) | Init du SIM | 1× au boot | Acceptable — keep |

Site A est le point dur :
```c
// calypso_trx.c:1038-1053 (kick CB, fires every 5ms wall)
{
    const char *env = getenv("CALYPSO_FORCE_RX_DONE");
    if (env && env[0] == '1') {
        ...
        cpu_physical_memory_write(rxdone_addr, &one, sizeof(one));
    }
}
CPUState*cpu=first_cpu;if(cpu)cpu_exit(cpu);qemu_notify_event();
timer_mod_ns(g_kick_timer, ... + 5000000);  // re-arm 5ms
```

**Conséquences problématiques** :
1. **200 Hz cpu_exit** — invalide les TB ARM 200×/sec, coût recompilation
   continu, plausible contributeur aux LOST events firmware (~3500 LOST/sec)
2. **Écriture aveugle** — fire même quand aucune op SIM active
3. **Masque la cause racine** : firmware STR à PC=0x8224ac ne commit pas
   sous icount=auto (TCG Task #29, jamais creusé)
4. **Pas event-driven** — pas corrélé au timing réel des SIM interrupts

## 2. Cause racine identifiée mais non corrigée

Per commentaire calypso_sim.c:234-260 :
- Firmware `sim_irq_handler` à PC=0x822498 reçoit FIQ correctement (SIM_IT
  read=0x0002 observé)
- L'instruction `STR rxDoneFlag` à PC=0x8224ac **n'écrit jamais** dans
  rxDoneFlag (firmware data @ 0x830510 / 0x008302a0 selon mapping)
- Cause supposée : bug TCG pour ce STR conditionnel sous icount
- 3 hypothèses listées dans le code (update_irq propagation, bh-defer,
  CPSR flag race) — aucune validée
- Hack écrit `1` directement, contourne le STR cassé

Le STR cassé est documenté comme "TCG bug Task #29" mais **jamais
investigué empiriquement**. Le commentaire dit "Remove hack once TCG
bug is identified or sim_irq_handler path is reworked" — c'est resté
intact depuis le 8 mai.

## 3. Améliorations proposées (3 niveaux)

**Niveau 1 — minimal-tactique** : drop site A, garder sites B et C.
- Le SIM_IT-read trigger (site B) fire à chaque lecture firmware du
  registre SIM_IT, suffisamment précoce dans le path SIM IRQ pour
  débloquer le busy-loop
- Site C init le flag au boot (état initial cohérent)
- Plus de 200 Hz cpu_exit
- Risque : si le firmware peut entrer en busy-loop SANS lire SIM_IT
  avant, le hack ne fire pas et busy-loop persiste

**Niveau 2 — event-driven via SIM IRQ raise** :
- Quand le module SIM lève FIQ (où qu'il soit dans calypso_sim.c), hook
  qui écrit rxDoneFlag=1 et cpu_exit
- Fire 1× par SIM interrupt réelle (pas 200 Hz)
- Couvre TOUS les paths qui mènent au busy-loop
- Plus propre que site A, plus robuste que site B seul

**Niveau 3 — vraie fix** :
- Identifier pourquoi STR à 0x8224ac ne commit pas sous icount
- Probablement nécessite probe sur l'instruction ARM precise (PC-trap +
  cpu register dump avant/après)
- Hors scope de cette session — TODO architectural

## 4. Plan d'implémentation immédiate

**Action 1 — drop calypso_trx.c:1038-1053 (site A 200 Hz)** :
- Garder le kick CB timer (utilisé pour cpu_exit régulier au boot stub
  per memory) mais virer la branche FORCE_RX_DONE
- Préserver les 2 autres sites (B, C event-driven)
- Documenter dans le code que site A est retiré (commentaire historique
  référence à cette session)

**Action 2 — re-run sans 200 Hz** :
- Si firmware progresse encore (boot, FBSB_REQ), le hack site B suffit
- Métrique : `count FORCE_RX_DONE writes` doit être ~1 par SIM op
  (vs ~200/sec actuel via site A)
- BTS doit toujours survivre (le 200 Hz cpu_exit n'était pas la cause
  unique du timing instable, mais il était contributeur)

**Action 3 (futur)** — si site B insuffisant :
- Implémenter Niveau 2 (event-driven via SIM IRQ raise)

## 5. Questions ouvertes pour ton arbitrage

1. **OK pour drop direct site A** ? Ou préfères Niveau 2 d'emblée
   (event-driven SIM IRQ hook) ?
2. **Si site A retiré et firmware re-busy-loop** : on saute à Niveau 2
   ou on creuse le STR @ 0x8224ac TCG bug ?
3. **Probe à ajouter** : log chaque écriture FORCE_RX_DONE (PC + path =
   site A/B/C). Permet de mesurer la fréquence réelle event-driven post-fix.
4. **Si la chaîne BTS→L1→Mobile progresse plus loin post-fix** : focus
   suivant = d_fb_det timing window (4.5ms) qui empêche FBSB_CONF. Sticky
   latch ou autre ? On vide la chaîne dette par dette.

## 6. État sync 3-way

```
md5sum calypso_c54x.c    : 944e572a926cf32f8e5968e0df5c2fcf  (STL/STH fix actif)
md5sum bridge.py         : a0993d22f3960e0ffbfeb2e7caf8e72d  (hybrid + dup-suppression)
```

Patches précédents tous sync. Le prochain patch (drop site A) suivra le
même workflow (host edit → cp qemu-calypso → docker cp container, md5
verify).

## 7. Métriques de session

| Métrique | Avant session | Maintenant |
|---|---|---|
| BTS survie | 5s (clock skew) | **64s** (puis jitter) |
| DSP RET-loop cascade | 285k events boot | 285k events boot (cascade ouverte) |
| DSP FB-det real | 0 SETs (synth-only) | **4 SETs val=0x001e** ✓ |
| ARM polls d_fb_det | 0 | 200 (avec 0 non-zero hits) |
| Mobile FBSB_REQ retries | 8 / 50s | 12 / 89s |
| FBSB_CONF | 0 | 0 (timing window) |
| Pivots débunkés total | 12 | **14** |
| Patches appliqués | STL/STH Xmem fix | + hybrid bridge + dup-suppress |

**On a débloqué 4 couches en aval** : BTS happy, real DSP, ARM polls,
mobile retries. Le dernier hop bloqué = ARM lit d_fb_det dans la
fenêtre 4.5ms (cascade-related). Et le FORCE_RX_DONE site A reste à
cleanup pour stabilité timing globale.
