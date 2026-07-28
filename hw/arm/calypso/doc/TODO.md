# TODO - Calypso QEMU (reorganise par priorites reelles, 2026-07-28)

Le STATUT ET LES TODO DEPENDENT DU MODE. Socle = `calypso.env` + un `calypso_X.env`.
Matrice statut x mode : `ETAT_ACTUEL.md` §1.

Modes :
- **SHUNT** = `SHUNT_LEGIT=1` / `SHUNT_NO_LEGIT=1` / `SHUNT_LEGIT=DSP,NO_CANNED`.
  FBSB host-side (`REAL_FB` + gr-gsm -> API RAM). Camp + LU + SMS = DONE.
- **NATIF** = `CALYPSO_NATIVE=1` / `CALYPSO_NATIVE_HELPED=1`. Le DSP c54x fait le FBSB.

Priorites : **P0** = le verrou unique du moment ; **P1** = ce qui le valide ou le suit
immediatement ; **P2** = etapes suivantes de la chaine ; **P3** = dette.
Chaque entree nomme son INSTRUMENT ou sa COMMANDE de verification, et distingue
MESURE / HYPOTHESE.

Reference d'adresses : `SHUNT_LEGIT_ADDRESS_MAP.md`, `DSP_ADDRESS_MAP.md`,
`DSP_ARM_LINKAGE.md`. Table d'opcodes de reference : `doc/opcodes/tic54x_hi8_map.md`
(source : `binutils-2.21.1/opcodes/tic54x-opc.c`).

---

## DEJA FAIT (ne pas re-lister, ne pas rechasser)

| Sujet | Etat | Instrument de re-verification |
|---|---|---|
| Go-live / handshake BGEN (Fix A, ARM -> DSP `0x098a`/`0x098c`) | FAIT | manifeste + sortie de la wait-loop `0xa4ca` |
| Shadow IMR `data[0x435b]` (= `0x52ed` arme) | FAIT | `CALYPSO_INIT_435B_OFF=0` (defaut) |
| Boucle Location Update en SHUNT (LU ACCEPT + TMSI REALLOC + On Network) + RACH UL | FAIT | `grep -icE "LOCATION UPDATING ACCEPT" /root/mobile.log` |
| Sous-voie SDCCH/8 (base DL `SS*4`, fenetre `base..base+3`) | FAIT | SDCCH bidirectionnel, IDENTITY RESPONSE |
| Re-acquisition apres Ctrl-C mobile (hook `on_arm_write(d_dsp_page,0)` -> re-arme BGEN) | FAIT | signal unique, zero boucle |
| **Natif observable** : SIGSEGV `g_shunt.as=NULL`, gate early-boot, `get_task_md()` shunt-only, split `active()`/`substitutes()` | FAIT | `dsp_n_exec_2/5 = 32768`, `DSP_ERR_STACK_OV` eteint |
| **3e gate BSP** (`calypso_bsp.c:1359`, la LIVRAISON vers `data[]`) aligne sur `CALYPSO_BSP_DARAM_FORCE`. Avant : 2 verrous sur 3, rien n'arrivait au DSP. Defaut inchange. | FAIT | `grep "deliver: gate shunt LEVE" /root/qemu.log` ; `grep -c "dropping fn="` = 0 |
| **Decimation** : `CALYPSO_BSP_IQ_DECIM=4` (`=1` = regression, feed a 4 SPS, `dphi=+0.25x pi/2`). Corollaire `DARAM_LEN=296` (638 ne valait que pour le 4 SPS non decime). | FAIT | `corr_iq.py --src bursts` -> `VERDICT: FCCH @1SPS PROPRE` |
| **Entree du demod = `data[0x4c00]`**, confirmee par mesure : PC=`0x9fb5` lit `0x4c00/05/0a/0f/15/1a`, valeurs reelles (`ff6e`, `c1fb`, `d147`). Config : `FB_CORR_ENTRY=0x94f5` + `BSP_DARAM_ADDR=0x4c00`. | FAIT | `CALYPSO_WATCH_9F00_RD=1` |
| **`FB_STREAM` / cellules `0x9213`,`0x9215`,`0x9260-61` : ECARTES** sur cette config — jamais lues avec `CORR_ENTRY=0x94f5`. `CALYPSO_FB_IQ_BASE=0x9210` y est donc inerte. | FAIT (ecarte) | absence de `FB-STREAM addr=` au log ; `CALYPSO_RMAP_PCLO=0x9f00` |
| **Signal d'entree bon** : `coh=0.998`, `rms=3.25e4`, `|DC|=379`, `zeros=0%`, FFT +67 708 Hz | FAIT | `corr_iq.py --src bursts` sur `/dev/shm/bursts.cfile` |

**Faux problemes clos — ne pas les reintroduire :**
- « `0x4c00` est gele » : artefact de lecture QMP hors fenetre API RAM (signature : peak
  exactement `0x8000`, 54 % de zeros). Mesurer A L'INTERIEUR (`BSP_LOG` au point
  d'ecriture, ou `ddump`).
- « `Q == 0` » : conclu sur les 2 premiers mots d'un burst (amplitude faible par
  construction) ; sur le burst entier `zeros=0%`.
- « `0xa042` detruit le signal » : `0x2c00` est du scratch, il n'y avait pas de signal a
  detruire ; `0x9fd5` y depose une table de coefficients CONSTANTE.
- « 0 FCCH sur 200 dumps » : artefact de fenetre (`DARAM_DUMP` capé au boot avec
  `d_fb_mode=0`). Le detecteur EST arme : `d_fb_mode[0x08f9]=0x0001` observe.
- Pointeur d'entree du demod « jamais initialise » (`AR6=0000`, lectures a `data[0x0000]`) :
  **perime**, c'etait la consequence de l'entree `0x9500` au lieu de `0x94f5`. Resolu.
- `api_write_cb` (`calypso_c54x.h`) = callback mort, fausse piste (`ETAT_ACTUEL.md` §6).
- Correctif TPU de `DOC_PATH_BOOT_TO_CORRELATOR_2026-07-25.md` : **invalide**.
  `0x8341` a 0 reference sur les 4 banks ; `0x7234 -> 0x013b` est un CALL ROM
  **inconditionnel** qui retourne en `0x7236`.

---

# P0 — DECODEUR c54x : la famille `0x1000-0x1FFF` et son audit systematique

C'est le verrou unique du moment. Symptome amont : **entree du demod VIVANTE, sortie
MORTE** (`ddump` : `|DC|=2.86e4` pour `rms=2.94e4`, `dphi=+0.004` ; cellule temoin
invariante sur 157-203 bursts, identique avec `DECIM=1` et `DECIM=4`).

## P0.a — `0x1800` (AND) decode comme LD — **CORRECTIF VALIDE AU RUN (2026-07-28)**

> Rectification : la mention « non valide au run » a ete ecrite avant la mesure. Le correctif
> **est** valide, par trois signatures independantes plus un test de non-regression :
> - `T` passe de `0x001f` invariant a {`0x0000`,`0x0010`,`0x0018`,`0x0019`} (`CALYPSO_DEMODIO`) ;
> - la saturation `A = -2147483648` (0x80000000) disparait : **0 occurrence** (etait systematique) ;
> - la sortie du demod passe de DC pur (`dphi=+0.004`) a une forme structuree (`dphi=-0.246`,
>   pic FFT de `+1830 Hz` a `-47579 Hz`) — `corr_iq.py --src ddump` avec `CALYPSO_DARAM_DUMP=1` ;
> - **non-regression** : en `SHUNT_LEGIT=1 NO_CANNED=1 REAL_FB=1`, le mobile est
>   `Channel synched (ARFCN=514, snr=28, BSIC=7)` avec `SYSTEM INFORMATION 2` puis `4`, et
>   LOCATION UPDATING ACCEPT conserve.
> Le correctif couvre les **quatre** sous-cas (`0x1800` AND, `0x1A00` OR, `0x1C00` XOR,
> `0x1E00` SUBC), pas seulement `AND`. `d_fb_det` reste 0 : le mur a recule d un etage.

Chaine fautive, verifiee mot a mot contre `calypso_dsp.PROM0.bin` (base `0x7000`) :

```
9fa1: 7660 000f   ST  #0x000f, *(0x60)   -> data[0x0060] = 15
9fa3: 7661 0010   ST  #0x0010, *(0x61)   -> data[0x0061] = 16
9fb1: 1860        AND *(0x60), A         <- devait faire A = A & 15, faisait A = 15
9fb2: 0061        ADD *(0x61), A         -> A = 31
9fb3: 880e        STLM A, MMR 0x0E (= T) -> T = 31
9fb5: 14ce        LD Smem, TS, A         TS = T[5:0] = +31 (SPRU172C : -16 <= TS <= 31)
                  => decalage gauche de 31 => saturation (A = 0x80000000 observe)
                  => sortie independante des operandes => DC plat
9fb8: 8694        STH A, ASM, *AR4+      -> workzone 0x2a00
```

- **Cause** (MESURE, lecture de code) : `case 0x1:` calculait `sub = (op >> 9) & 7` mais ne
  nommait que sub 0..3 (`0x1000` LD / `0x1200` LDU / `0x1400` LD,TS / `0x1600` LDR).
  `0x1860` donne sub=4 et tombait sur le `default` = chargement signe.
  `LD Smem,TS` (sub 2) est CORRECT ; le bug etait strictement en amont.
- **Correctif** : `calypso_c54x.c:9365-9389` implemente `AND` (sub 4 / `0x1800`),
  `OR` (5 / `0x1A00`), `XOR` (6 / `0x1C00`), `SUBC` (7 / `0x1E00`). Smem **zero-etendu**
  sur 40 bits, conforme a l'exemple TI (SPRU172C p.4-12 : `A=00 00FF 1200 & 0x1500
  -> A=00 0000 1000`). Compile et embarque.
- **[ ] RESTE A FAIRE — le valider au run.** Un run post-correctif (44 s) donne toujours
  `d_fb_det=0` sur 3 600 `DETECTOR-RUN`. Mais **`d_fb_det` n'est pas le critere** :
  le critere est la SORTIE DU DEMOD.

  ```bash
  cd /opt/GSM/qemu-src
  rm -f /dev/shm/daram_2a00.cfile          # AVANT le run (artefact perime sinon)
  CALYPSO_NATIVE_HELPED=1 CALYPSO_FB_CORR_ENTRY=0x94f5 CALYPSO_DSP_RUN_C54X=1 \
  CALYPSO_BSP_DARAM_FORCE=1 CALYPSO_BSP_DARAM_ADDR=0x4c00 CALYPSO_BSP_DARAM_LEN=296 \
  CALYPSO_BSP_IQ_DECIM=4 CALYPSO_SHUNT_REAL_FB=0 CALYPSO_DARAM_DUMP=1 ./start-clean.sh
  cd /opt/GSM/qemu-src/tools && python3 corr_iq.py --src ddump | tail -3
  ```
  Critere non ambigu : passer de `|DC|=2.86e4 / rms=2.94e4 / dphi=+0.004` (sortie morte)
  a `coh > 0.90`, `dphi ~ +1.571`. `SHUNT_REAL_FB=0` pour ne pas mesurer l'oracle hote a
  la place du natif.

## P0.b — Auditer TOUTE la famille `0x1000-0x1FFF` contre la table

**Raison** : si `0x1800` etait faux, la meme structure (`sub` calcule, sous-cas non
nommes avales par un `default`) peut avoir avale d'autres encodages. L'audit est
mecanique et se fait une fois pour toutes.

- [ ] Confronter les 8 sous-cas de `case 0x1` (`calypso_c54x.c:9320-9393`) ligne a ligne a
      `doc/opcodes/tic54x_hi8_map.md` (`0x10..0x1F`, tous mask `0xFE00`) : LD / LDU /
      LD,TS / LDR / AND / OR / XOR / SUBC. **Etat au 28/07 : les 8 sous-cas sont
      desormais nommes** ; verifier la SEMANTIQUE de chacun (extension de signe,
      SXM, flags C/TC) et non seulement la presence du `case`.
- [ ] Verifier que le `default` restant est **inatteignable** (`sub` est sur 3 bits, 8
      valeurs, 8 `case`) et le transformer en `assert`/`goto unimpl` : un `default`
      silencieux est exactement ce qui a masque ce bug pendant des semaines.
- [ ] Instrument d'audit disponible : `CALYPSO_DA_LO` / `_HI` / `_INSN` +
      `CALYPSO_DEBUG=DECODE-AUDIT`, elargi a `0x7000..0xFFFF`.

## P0.c — Le meme audit applique aux nibbles voisins : deux resultats deja acquis

Ces deux points sortent de l'audit et sont **sur le chemin du demod / du noyau MAC**.
Statut : MESURE (lecture de code + decodage PROM), correctif NON ecrit.

- [ ] **`case 0x3` = MAC aveugle** (`calypso_c54x.c:9480-9508`). Tout `0x3000-0x3FFF` est
      execute comme `acc += T * Smem`, **sauf** `0x3800/0x3900` (SQURA, deja corrige le
      23/06). Or la table donne : `0x30` LD Smem,T · `0x31` MPYA · `0x32` LD Smem,ASM ·
      `0x33` MASA · `0x34` BITT · `0x35` MACA · `0x36` POLY · `0x37` MACAR ·
      `0x3A/3B` SQURS · `0x3C-0x3F` ADD.
      **Impact direct** : le noyau MAC `0xa076..0xa09d` contient le motif `30 6x`
      (= `LD Smem, T`), execute aujourd'hui comme un MAC. Coherent avec la mesure
      « 60 000 ecritures pour 4 a 7 valeurs distinctes, toutes dans
      `{0001, 0002, 001f, 003e}` » = un accumulateur qui double une constante.
      Le detecteur `0x9ab8..0x9ac2` a le meme motif AND -> T (`1983 AND *AR3,B` puis
      `890e STLM B,T` puis `348e BITT`).
      **Les handlers corrects existent deja** (`calypso_c54x.c:6467-6550` :
      `0x3000` LD Smem,T, `0x3100` MPYA, `0x3200` LD Smem,ASM, `0x3300` MASA,
      `0x3400` BITT, `0x3500` MACA, `0x3700` MACAR) — mais ils sont **sous `case 0xF`**
      (switch `hi4`, `calypso_c54x.c:5961`) donc **morts** pour un opcode dont le
      nibble haut vaut 0x3. Le travail est un DEPLACEMENT, pas une reecriture.
- [ ] **`case 0x0` : `ADDC` et `SUBB` ignorent la retenue** (`calypso_c54x.c:9434-9476`).
      Le code derive `is_sub`/`is_unsigned`/`ts_shift` de `sub`, ce qui couvre
      `0x0000` ADD, `0x0200` ADDS, `0x0400` ADD,TS, `0x0800` SUB, `0x0A00` SUBS,
      `0x0C00` SUB,TS — mais **sub 3 (`0x0600` ADDC) est traite comme un ADD nu et
      sub 7 (`0x0E00` SUBB) comme un SUB nu**. Priorite plus basse : impact non mesure
      sur le chemin FB, a n'attaquer qu'apres P0.a/P0.c.

---

# P1 — Une fois la SORTIE DU DEMOD vivante : remonter la chaine FBSB

Rien ici ne se travaille avant que `corr_iq.py --src ddump` ait donne un verdict
non-DC. L'ordre est celui de la chaine, chaque etape a son oracle.

- [ ] **P1.a — `DSP_ERR_DMA_PEND` (0x20 = 32), permanent.**
      MESURE : `grep -oE "DSP Error Status: [0-9]+" /root/osmocon.log | sort | uniq -c`
      -> **723 x « 32 »**, aucune autre valeur.
      HYPOTHESE : le DSP attend l'achevement d'une DMA ; le BSP ecrit le buffer
      **directement** dans `data[]` sans passer par la machinerie DMA, donc le drapeau
      « DMA en cours » n'est jamais efface. Nommer qui pose et qui efface cet etat cote
      modele (sonde `CALYPSO_DMAWATCH` sur `0x0054..0x0057`).
      Note de progression : `2048` (`STACK_OV`) -> `32` (`DMA_PEND`) n'est pas une
      regression — le DSP va assez loin pour se plaindre de la couche suivante.

- [ ] **P1.b — Format d'entree : le demod lit en PAS DE 5, le BSP depose contigu.**
      MESURE : lectures a `0x4c00`, `+5`, `+0a`, `+0f`, `+15`, `+1a` (PC `0x9fb5`) alors
      que le BSP depose 296 int16 contigus I/Q entrelaces.
      HYPOTHESE : structure de 5 mots par echantillon attendue par un filtre polyphase
      6 taps. **Non tranchee** — il n'est pas exclu que le stride 5 soit correct et que
      ce soit le REMPLISSAGE qui doive changer de layout. A re-evaluer APRES P0 :
      un decodeur fautif peut fabriquer un stride qui n'est pas celui du firmware.

- [ ] **P1.c — Slot de dispatch FB = stub `RET`.**
      MESURE : `0xb01c: 10f8 43d8` (adressage **absolu**) ; `data[0x43d8] = 0xab38` ;
      1er mot de `0xab38` = `fc00` = `RET`. Unique ecrivain sur les 4 banks : `0xbb00` ;
      aucun writer cache (watchpoint dans `data_write_locked`).
      Question ouverte, unique : pourquoi l'ordonnanceur de trame `0x7234` ne tombe-t-il
      jamais sur `0x8341` (la LUT FB, seule a installer un vrai handler dans `0x43d8`) ?
      Bequille de VALIDATION (pas correctif) : `CALYPSO_BSP_DISPATCH_FB=1`
      (+ `_TGT`, `_NOIMR`).

- [ ] **P1.d — `d_fb_det` : personne ne l'ecrit en natif.**
      MESURE : verifie des deux cotes du miroir `api_ram` (`CALYPSO_FBDET_API`) — seul
      l'ARM touche la cellule, toujours 0. Publisher natif unique =
      `ORM #1,*(0x08f8)` @`0x79e4`, en banque commune `0x7700-0x79F0`, jamais executee.
      Le `FB0_SEARCH -> SB_SEARCH` observe est le **renoncement** d'osmocom
      (`BSIC=0`, `snr=0`, `attempt=12`), pas une detection. Depend de P0, P1.a, P1.c.

- [ ] **P1.e — En natif : ni SCH ni SI, donc pas de camp meme avec le FB servi.**
      MESURE : `dispatch_allc` = 0 occurrence, `feed_agch` = 0, `DISPATCH SDCCH` = 0,
      `sb_valid` = 0 ; mobile en boucle
      `No sysinfo yet` -> `Cell selection failed, read timeout`.
      Consequence de methode : meme si P0 tombait demain, le natif ne camperait pas.
      Il manque SCH (`sb_bsic`/`sb_fn`/`sb_toa`) et SI (`si_buf`).

- [ ] **P1.f — Remplacer gr-gsm par le DSP DANS `SHUNT_LEGIT`** (plan du 28/07).
      Le FB ne depend **deja plus** de gr-gsm (correlateur hote `REAL_FB`). Restent SCH
      et SI. Couper gr-gsm : `CALYPSO_SHUNT_NO_GRGSM=1`. Piloter le correlateur DSP
      depuis le shunt : `CALYPSO_SHUNT_DSP_FB=1` — excursion **bornee** (`_MAX`) et
      **pile dediee** (`_SP`), sans quoi on corrompt la pile du DSP et `STACK_OV` revient.
      Ordre : FB (fait) -> SCH -> SI ; oracle a chaque etape = le producteur actuel.

---

# P2 — Chantiers paralleles (mode SHUNT, independants du P0)

- [ ] **Voix TCH/F — lever l'ASSIGNMENT FAILURE.**
      Etat : ASSIGNMENT COMMAND atteint, puis FAILURE ; le shunt ne presente pas le
      TCH DL. Reseau OK (`call fake_trx` = ACTIVE + audio ; GAPK/FR compile).
      Double cause localisee :
      (a) `calypso_dsp_shunt.c:177` lit **inconditionnellement** `a_cu@0x264`
          (SDCCH/SACCH, 23 o) quel que soit le type de tache ; a l'assignation le
          firmware ecrit `a_fu@0x282` (FACCH UL) et la voix `a_du_1@0x134` -> le shunt
          lit une cellule vide et rate la FACCH ;
      (b) `tools/calypso-ipc-device/qemu_wrap.c:1194` : voie RF montante cablee en dur
          TS0/RACH (`ul_slotoff=1875`), alors que le TCH/F est assigne **TS2**.
      Deja cable, NE PAS reecrire : TCH DL sub0 (`calypso_tch_dl_poll` `:333-350`,
      `shunt_dispatch_tch_dl` `:356-369`, routage `:836-837`). Manque le **producteur**
      du sideband `/dev/shm/calypso_tch_dl`.
      Detail : [`VOIX_PLAN.md`](VOIX_PLAN.md).

- [ ] **Fiabiliser SMS en `SHUNT_LEGIT=DSP,NO_CANNED` (flaky).**
      MO/MT = DONE en `SHUNT_LEGIT` et `SHUNT_NO_LEGIT` ; intermittent quand le c54x
      tourne en // sans cannes (anti-stall deja ajoute).
      **Mesure prealable obligatoire** : `grep "EVICT-STATS" /root/qemu.log | tail -1`
      **dans ce mode**. En `SHUNT_LEGIT` les 3 politiques d'eviction sont a zero et la
      profondeur du ring vaut 1 ; la saturation `depth=32` n'existait qu'en
      `DSP,NO_CANNED`. NE RIEN supprimer de `sdcch_ring` avant cette mesure.

- [ ] **Robustesse LU en SHUNT — deux mesures contradictoires a departager.**
      `run_results.md` Run A5 : 2,70 s RACH -> ACCEPT, **0 retry T3211** (n=2).
      `SHUNT_LEGIT_ADDRESS_MAP.md` §9 : « intermittent, ~1 succes / 19 retries T3211 »
      (26/07, **anterieur** au fix sous-voie SDCCH/8).
      Trancher : `grep -c T3211 /root/mobile.log` sur 5 runs `SHUNT_LEGIT` consecutifs.

---

# P3 — Dette transverse (tous modes)

- [ ] **Decodeur c54x, suite de l'audit P0** : cluster MAC/LD/BITT `0x30-0x37` present
      mais **mort sous `case 0xF`** (cf P0.c) ; `BC`/`FB` `0xF8` par nibble ;
      catch-all FIRS/LMS `0xE000-0xE3FF` ; `0xF6`/`0xF7` fabriques.
      NE PAS re-fixer `0x72`/`0x73` MVDM/MVMD sans le side-effect (`REVERT_MVMD`).
- [ ] Refacto `sdcch_ring` : `fn` stocke-non-utilise, eviction silencieuse, ajouter TTL +
      drop explicite. Dette de structure, pas un blocage. **A mener en session
      foreground dediee** — ne pas coder depuis une reorg doc, et pas avant la mesure
      `EVICT-STATS` du P2.
- [ ] Retirer les env `BURST_*` mortes (`BURST_FN` / `BURST_OFS` / `BURST_ECHO`) une fois
      `d_burst_d` WP-mirror valide au run.
- [ ] `a_pm` mot 8 vs mot 12 : la section A de `on_frame_tick` ecrit sous le label
      « a_pm » les idx `0x30`/`0x44` (= `a_serv_demod`) ; la vraie cellule `a_pm` est
      `0x834`/`0x848`.
- [ ] Code mort a supprimer : `bsp.fb_valid` jamais mis a 1 ;
      `calypso_dsp_shunt_route_c54x_active()` sans appelant ; API
      `calypso_orch_init/publish` inexistante ; `calypso_tint0_start()` jamais reference ;
      `fw_console.c` sans appelant.
- [ ] Variables d'env INERTES a nettoyer : `CALYPSO_CORRELATOR_TRACE` (le vrai gate est
      `CALYPSO_DEBUG=CORRELATOR`), `CALYPSO_FORCE_3F92`, `CALYPSO_FORCE_0810`,
      `CALYPSO_FIX_MVDM` (le code ne lit que `..._OFF`), et la ligne morte `: "0xC000"`
      dans `calypso.env`.
- [ ] Ecart profils livres / config mesuree correcte : `calypso_native.env` et
      `calypso_native_helped.env` posent encore `FB_CORR_ENTRY=0x9500` (introuvable dans
      les 28 672 mots de PROM) alors que l'entree ROM est `0x94f5`. Le run de reference
      le corrige en CLI. **Documenter, ne pas changer le defaut sans decision.**
- [ ] Mode `CALYPSO_L1=c` **mort par auto-annulation** : `l1_c_active()` arme le shunt
      (`calypso_dsp_shunt.c:1847`) alors que `calypso_trx.c:1475` exige
      `l1_c_active() && !shunt_active()`. `calypso_layer1_tick()` n'est donc jamais
      appele. Soit reparer, soit retirer le mode. (DEDUCTION DE CODE, non testee au run.)
- [ ] Threading (roadmap) : un thread par bloc (ARM/DSP/TPU/BSP) pour eliminer les
      ~300 artefacts de serialisation TCG (`THREADING_TODO`).
- [ ] Doc : `DSP_ROM_MAP.md` canonique dit « PROM1 mirroree `0x8000-0xFFFF` » ; corriger en
      « chargee `0x18000+` sans mirror » (fix 2026-05-29). Marquer « historique » les
      `REPORT_CLAUDE_WEB_*`, `BOOT_TO_FBSB_SEQUENCE`, `FBSB_SEQUENCE_TRACE`.

---

# Hygiene de mesure (a lire avant d'ouvrir un ticket ci-dessus)

1. **`BUILD-STAMP` ne dit PAS quel binaire tourne.** La macro vit dans
   `calypso_dsp_shunt.c:107` (`__DATE__`/`__TIME__`) et date **son propre TU**. Un run a
   affiche `09:42:13` alors que le binaire datait de `11:29:07`.
   Instrument correct : mtime du `.o` de l'unite modifiee + `lstart` du process.
   **[ ] A corriger** : faire dependre le stamp du TU du decodeur, ou d'un hash des objets.
2. **Les compteurs `REAL-FB` sont plafonnes par le logger** (`calypso_dsp_shunt.c:1670` :
   `rfl < 20 || (det && rfl < 300)`). « 280/300 » = contenu des lignes loguees, pas un
   taux de detection.
3. **Comparer les mtimes des artefacts au `lstart` du process** avant d'interpreter
   (`/dev/shm/daram_2a00.cfile` a 11:06 vs `bursts.cfile` a 11:31 = deux runs differents).
4. **Toute mesure prise via le monitor QMP est hors fenetre API RAM et racy.** Les mesures
   valides se prennent a l'interieur : `BSP_LOG` au point d'ecriture, ou `ddump`.
5. **Hygiene de mode** : `CALYPSO_SHUNT_REAL_FB=1` fait servir `d_fb_det` a l'ARM par le
   correlateur HOTE (`calypso_dsp_shunt.c:1463-1490` via `calypso_trx.c:297`). En natif,
   mettre `SHUNT_REAL_FB=0` — sinon on mesure l'oracle, pas le DSP. Seule la cellule
   `data[0x08f8]` imprimee par `DETECTOR-RUN` mesure le natif.
6. **Regles de sonde** (payees 4 fois) : concevoir par la CONDITION DE DECLENCHEMENT et
   non par l'adresse ; preferer un AGREGAT a un flux plafonne, avec temoin de saturation ;
   distinguer « varie dans l'espace » de « varie dans le temps » ; **« pas de log » n'est
   jamais « pas d'evenement »** tant que la sonde n'est pas verifiee VIVANTE et sa fenetre
   COUVRANTE (plafond sature, seuil de dump trop haut, plage ecrite cote HOTE donc
   invisible depuis `data_write_locked`, variable d'env absente du run).
