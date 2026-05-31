# SP-catastrophe → self-CALA 0x70c3 → PMST/d_fb_det = 0x70c4 (=28868)

Séquence complète du "mort-vivant" — l'erreur unique qui bloque toute la chaîne
post-boot alors que le DSP tourne réellement (mobile atteint gsm322 cell-search,
power scan, "Found signal ARFCN 514" mais rxlev poison).

## Chaîne (déterministe — insn=3592396 identique sur 2 runs)

1. Boot réel OK (redirect OFF par défaut : 0xff80→b410, 4 RET).
2. À insn **3592396**, dispatcher 0x8341 : `DP=0x0fd` (au lieu de 0x125 bon cas).
   - `lastST0w{pc=0x94f3 val=0x80fd}` : le `POPM ST0 @0x94f3` (op 0x8a06) a poppé
     **0x80fd** = adresse retour du `CALLD @0x80f9` (0x80f9+4), un ORPHELIN.
   - DP = 0x80fd & 0x1FF = **0x0fd**.
3. dispatcher lit LUT à (DP<<7)|0x07 = EA 0x7e87 (zone garbage) = 0xf6b7
   → calcule A = **0x70c3**.
4. `CALAD A` (0x8353 op 0xf6e3) → saute à 0x70c3.
5. 0x70c3 = `f4e3 CALA` → **self-CALA** : s'appelle lui-même en boucle.
6. Chaque itération pousse (pushes monte à 500M+), SP draine, et écrit
   **0x70c4** (=PMST clobbé) partout via FB-DET-WR data[0x08F8], A_CD-WR,
   WR-0x4189.
7. L'ARM lit d_fb_det=0x70c4 → "DSP Error Status: 28868" (osmocon),
   TOA=28868, rxlev poison 63 → NO_CELL_FOUND.

## ⚠️ MISE À JOUR : FRAME RÉFUTÉ comme déclencheur (2026-05-31)

`FRAME` (0xEExx) a **0 site dans PROM0** → le firmware ne l'utilise pas →
le stub vide 0xEE n'est PAS le déclencheur de CETTE catastrophe. Corrigé quand
même (bug latent : vide + pas de `return` → fallthrough = over-pop potentiel),
mais ça ne résout pas le trou noir.

### Hypothèse forte RESTANTE : compteur delay-slots décrémenté trop tôt

Dans `c54x_run` (~ligne 11414-11429) : après `s->pc += consumed`, le bloc
`if (s->delay_slots > 0) { s->delay_slots--; ... }` s'exécute **dans la MÊME
itération que l'instruction qui a posé delay_slots=2** (CALLD/CALAD/RCD/RETD/BD).
Résultat : delay_slots passe 2→1 immédiatement → **UN SEUL slot s'exécute au
lieu de 2**. Le retour poussé (CALAD = PC+3, CALLD = PC+4) suppose 2 mots de
delay-slots exécutés. Si le 2ᵉ slot (sauté) contient un push/pop, SP désync
d'1 mot → POPM ST0 @0x94f3 ramasse l'orphelin 0x80fd.

- Cas OK observé : delay-slot = UNE instruction 2-mots (ex. STM 7716/2a00 @0x80fb)
  → 1 instruction = 2 mots → PC atterrit juste (PC+4). Pas de désync.
- Cas KO suspecté : delay-slot = DEUX instructions 1-mot → seule la 1ʳᵉ
  s'exécute, la 2ᵉ est sautée. Si push/pop dedans → désync.

RISQUE : la tempo FB-det est calée sur le comportement actuel (cf commentaires
L6581-6591, L11478+). NE PAS changer sans validation cross-run. Probe ciblée
recommandée : compter, par CALAD/CALLD, le nombre de mots réellement exécutés
en delay-slot vs 2 attendus, au voisinage de insn 3.59M.

Chemin déterministe à décoder : CALLD@0x80f9 (push 0x80fd) → callee 0x82f6
(RETD@0x8304) → ... → CALAD@0x8353 → fonction 0x94d1 → POPM ST0 @0x94f3.

---
## (ANCIENNE hypothèse, RÉFUTÉE) FRAME stub vide

**`FRAME` (hi8=0xEE) = STUB VIDE** dans calypso_c54x.c (~ligne 6797) :
```c
if (hi8 == 0xEE) {
    /* EExx: LD #k, DP? No — this is the FRAME instruction... */
}
```
- Ne fait RIEN, ne `return` même pas (fallthrough).
- Vrai sémantique (binutils `frame 0xEE00 mask 0xFF00 {OP_k8}` + SPRU172C) :
  **`SP = SP + (int8_t)k8`** (k signé -128..127).
- Non câblé → SP jamais ajusté → cadres de pile désalignés → over-pop
  (avant trou : `SP-EVENTS pops=129 > pushes=75 net=-58`) → POPM ST0 ramasse
  l'orphelin 0x80fd.

Aussi STUBS VIDES voisins (lignes 6800-6843) : 0xED (placeholder), 0xEF
(~20 duplicate vides). À auditer — mais 0xEE FRAME est le coupable du SP.

## FIX (à appliquer)

Câbler 0xEE :
```c
if (hi8 == 0xEE) {
    /* FRAME #k8 : SP = SP + sign_extend(k8). Per tic54x-opc.c
     * frame 0xEE00 mask 0xFF00 {OP_k8}, SPRU172C §4 (stack frame adjust). */
    int8_t k = (int8_t)(op & 0xFF);
    s->sp = (uint16_t)(s->sp + k);
    return consumed + s->lk_used;   /* 1 word */
}
```
ATTENTION : vérifier le sens du signe (FRAME -N alloue = SP descend ; FRAME +N
libère). k8 signé direct est correct per doc. consumed=1 (1 mot).

## Validation attendue post-fix (invariants, pas valeurs)

- `tgt=0x70c3` count → 0 (plus de self-CALA).
- `BLACKHOLE-CALA` → 0.
- osmocon "DSP Error Status: 28868" → disparaît.
- d_fb_det (0x08F8) plus jamais = 0x70c4.
- SP-LEDGER net_words borné (pas de drain 500M pushes).
- ATTENTION run non-déterministe : confirmer sur 2-3 runs.

## Outils de ce run
- `/tmp/debugging.gdb` (dans docker) : scan70c4 / fbdet / ndb / pages / dspstat
  / armbps. gdb voit ARM (0xFFD00000 = API RAM DSP en lecture seule).
- Build : `docker exec osmo-operator-1 bash -c "cd /opt/GSM/qemu-src/build && ninja qemu-system-arm"`
- Container autoritaire : osmo-operator-1:/opt/GSM/qemu-src

---
## ✅ ROOT CAUSE CONFIRMÉE + FIX (2026-05-31 soir)

**Bug = delay-slots comptés en INSTRUCTIONS au lieu de MOTS** (calypso_c54x.c
c54x_run, countdown ~L11424). L'ancien code :
  - décrémentait delay_slots dès l'itération qui l'arme (delay_slots=2 → 1),
  - puis -1 par instruction,
  → une SEULE instruction de delay-slot exécutée.

Conséquence :
  - delay-slot = 1 insn 2-mots (STM #k) → OK par accident (2 mots = 1 insn).
  - delay-slot = 2 insns 1-mot → la 2e est SAUTÉE.

Sites PROM0 où la 2e insn sautée est un PUSH (scan statique) :
  - 0xb53a BCD   → slot1 0x4a19 PSHM  (push perdu)
  - 0xc9a2 BANZD → slot1 0x4bbe PSHD  (push perdu)
  - 0xcaab BANZD → slot1 0x4bc2 PSHD  (push perdu)
Ces zones 0xb5xx/0xc9xx = code POWER-SCAN que le mobile exécute en cell-search.
Chaque passage perd un push → over-pop cumulé (net_words=-58) → POPM ST0 @0x94f3
ramasse l'orphelin 0x80fd → DP=0x0fd → 0x70c3.

FIX : countdown en MOTS, décrément = `consumed` (longueur de l'insn de
delay-slot), et on ne compte pas l'itération qui arme (ds_before==0).
Vérifié sur chemin GOOD (CALLD@0x94d5/@0x80f9 : delay = 1 insn 2-mots STM →
reste correct, retour inchangé) et sur 0xb53a (2 PSHM → les deux s'exécutent).

VALIDATION (invariants, run non-déterministe → 2-3 runs) :
  - tgt=0x70c3 → 0 ; BLACKHOLE-CALA → 0 ; osmocon 28868 → disparaît
  - d_fb_det (0x08F8) jamais 0x70c4 ; SP-LEDGER net_words borné
