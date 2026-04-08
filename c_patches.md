# C patches for calypso QEMU emulation (2026-04-07)

Application des découvertes faites pendant la session de debug FBSB. Chaque
patch est classé:

- **CONFIRMED** = bug ou dead code visible dans le source, fix ne change pas le
  comportement nominal mais nettoie ou répare une trace.
- **SPECULATIVE** = hypothèse non confirmée. NON APPLIQUÉ.

## Patches CONFIRMED (à appliquer)

### C-1. Dédup `RETD` handler (F273)

**Fichier**: `hw/arm/calypso/calypso_c54x.c`

**Bug**: deux implémentations byte-identiques de `0xF273` (RETD):
- Lignes 1483-1487 (dans le bloc F2xx interne)
- Lignes 1701-1705 (dans le bloc F2xx du dispatch hi8=0xF2)

La 2e (1701-1705) est **dead code** — la 1ère gagne toujours.

**Action**: supprimer le bloc 1701-1705 (en gardant un commentaire de référence).

**Risque**: nul. Code mort.

### C-2. Dédup `CALLD` handler (F274)

**Fichier**: `hw/arm/calypso/calypso_c54x.c`

**Bug**: deux implémentations byte-identiques de `0xF274` (CALLD):
- Lignes 1473-1481 (premier bloc)
- Lignes 1691-1699 (deuxième bloc)

La 2e est dead code.

**Action**: supprimer le bloc 1691-1699.

**Risque**: nul.

### C-3. Dédup `RPTBD` handler (F272)

**Fichier**: `hw/arm/calypso/calypso_c54x.c`

**Bug**: cinq (!) implémentations de F272 (RPTBD):
- Lignes 1461-1471 (premier)
- Lignes 1680-1689 (deuxième dans hi8=0xF2)
- Lignes 2258-2280 (RPTBD générique)
- Lignes 3408-3420
- Lignes 3469-3477

D'après l'agent Explore, les 4 dernières sont des doublons potentiellement
divergents. À vérifier au cas par cas avant suppression.

**Action**: lire les 5 implémentations et garder la première (ligne 1461) si
toutes équivalentes. Sinon, fusionner.

**Risque**: faible si toutes équivalentes, moyen si divergence (à investiguer).

### C-4. Fix initial value du SP-OOR tracer

**Fichier**: `hw/arm/calypso/calypso_c54x.c` ligne ~3633

**Bug**: `prev_sp = 0xEEFF` initial dans le tracer SP-OOR alors que le boot SP
DSP = `0x5AC8`. Le premier événement est faussé (semble OOR alors que c'est
juste l'initialisation).

**Action**: initialiser `prev_sp = s->sp` au reset, ou marquer le premier event
comme "INIT" et l'ignorer.

**Risque**: nul. Tracer uniquement (DBG_SP).

## Patches SPECULATIVE (NON APPLIQUÉ — investigation requise)

### S-1. Bug F074 / F274 — SP runaway PROM0 0xb906

**Symptôme observé**: 6/10 runs baseline ont un SP runaway de ~13000 events,
tous à `prev_PC=0xb906 prev_op=0xf074`. SP descend de 0x5AC8 vers 0x0000, atteint
0x08F8 = adresse NDB de `d_fb_det`, écrase `d_fb_det` avec des return addresses
(0xb908 = PC+2). ARM lit alors d_fb_det != 0 et croit avoir trouvé un FB.

**Hypothèses (toutes non confirmées)**:
1. **Self-loop firmware**: `prog[0xb907] == 0xb906` — CALL à soi-même.
2. **CALL à l'intérieur d'un RPTB**: F074 à PC=REA d'un bloc RPTB. Le wrap
   check (ligne 3842 `if (s->pc == s->rea + 1)`) ne déclenche jamais car le
   CALL met PC=target. Le bloc s'exécute en boucle infinie via le wrap RPTB,
   chaque tour pousse une nouvelle return address sans jamais RET.
3. **Bug prog_fetch sur PROM0 0xB000+**: 0xb907 mal lu (mais le fix XPC est OK).

**Pourquoi pas appliqué**: aucune des 3 hypothèses n'est vérifiée. Patcher
calypso_c54x.c sans savoir lequel = casser plus que ça ne répare. Le user a
explicitement interdit les hacks C.

**Action requise pour confirmer**:
- a. Dumper PROM[0xb906..0xb910] depuis le firmware bin → confirme/infirme #1
- b. Logger l'état RPTB (rsa, rea, rptb_active, brc) au moment où PC=0xb906 →
  confirme/infirme #2
- c. Vérifier que prog_fetch(0xb907) renvoie la valeur attendue → confirme/infirme #3

### S-2. Bug F5E3 — SP runaway PROM0 0xc12a

**Symptôme**: avec hack `r8=1`, run 014 expose un 2e site de SP runaway:
`prev_PC=0xc12a prev_op=0xf5e3`. Cluster 0xc121-0xc12a est une fonction DSP
réellement appelée par le path FB-found (que la baseline ne traverse jamais).

**Identification opcode F5E3**:
F5E3 = 1111 0101 1110 0011. Pas immédiatement visible dans tic54x-opc.c.
Probable C548 extension Texas Instruments non documentée.

**Pourquoi pas appliqué**: opcode pas identifié, semantics inconnues. Avant de
le coder, il faut savoir ce qu'il fait.

**Action requise**:
- a. Grep tic54x-opc.c pour `0xF5E0` mask `0xFFF0` ou `0xF5C0` mask `0xFFC0`
- b. Si non trouvé → consulter SPRU172C section "C548 enhancements"
- c. Désasser `arm-elf-objdump -d -m tic54x` sur la zone PROM si possible

## Patches DOWNSTREAM (post-fix DSP)

Une fois S-1 et S-2 confirmés et corrigés, vérifier:
- Les régimes de variance disparaissent (toutes runs en régime "B" sain)
- d_fb_det reste à 0 sauf si vraiment détecté (pas de garbage)
- Mobile reçoit FBSB_CONF (avec result=255 puisque DSP pas encore capable de
  vraies corrélations FB)
