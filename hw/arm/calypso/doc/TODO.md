# TODO - Calypso QEMU (par mode, 2026-07-27)

Le STATUT ET LES TODO DEPENDENT DU MODE. Le socle = `calypso.env` + un
`calypso_X.env`. Voir la matrice statut x mode dans `ETAT_ACTUEL.md` §1.

Modes :
- **SHUNT** = SHUNT_LEGIT=1 / SHUNT_NO_LEGIT=1 / SHUNT_LEGIT=DSP,NO_CANNED.
  FBSB host-side (real_fb + gr-gsm -> API RAM). Camp + LU + SMS = DONE.
- **NATIF** = CALYPSO_NATIVE=1 / CALYPSO_NATIVE_HELPED=1. DSP c54x fait le FBSB.

Priorites : P1 = debloque la feature courante du mode ; P2 = suivant ; P3 = dette.
Reference d'adresses : `SHUNT_LEGIT_ADDRESS_MAP.md`, `DSP_ADDRESS_MAP.md`,
`DSP_ARM_LINKAGE.md`.

DEJA FAIT (ne pas re-lister) : go-live / handshake BGEN (Fix A, ARM->DSP
0x098a/0x098c), shadow IMR 0x435b (=0x52ed arme), boucle Location Update en
SHUNT (LU ACCEPT + TMSI REALLOC + On Network) et le RACH UL qui la fermait.

---

## SHUNT (mode fiable : camp + LU + SMS = DONE)

- **[P1] Voix TCH/F - lever l'ASSIGNMENT FAILURE**
  - Etat : ASSIGNMENT COMMAND est atteint puis ASSIGNMENT FAILURE ; le shunt ne
    presente pas le TCH DL. Cote reseau OK (call fake_trx = ACTIVE + audio).
  - Quoi : (a) presenter le TCH DL host-side vers le mobile ; (b) capturer le
    FACCH UL a_fu @0x282 pour boucler l'assignation.
  - Ou : chemin DL/UL shunt ; [`VOIX_PLAN.md`](VOIX_PLAN.md) ; a_fu @0x282.

- **[P2] Fiabiliser SMS en SHUNT_LEGIT=DSP,NO_CANNED (flaky)**
  - Etat : SMS MO/MT = DONE en SHUNT_LEGIT/NO_LEGIT ; WIP flaky en DSP,NO_CANNED
    (anti-stall deja ajoute, encore intermittent).
  - Quoi : stabiliser MO/MT quand le DSP c54x tourne en // sans cannes.

- **[P3] Refacto `sdcch_ring` (CODE - a faire en FOREGROUND, pas ici)**
  - Quoi : `fn` stocke-non-utilise ; eviction silencieuse ; ajouter TTL + drop
    explicite. Dette de structure, pas un blocage fonctionnel.
  - Note : refacto code a mener en session foreground dediee, NE PAS coder depuis
    la reorg doc.

---

## NATIF (DSP c54x fait le FBSB - WIP)

- **[P1] d_fb_det : le correlateur tourne mais 0 detection**
  - Etat : en NATIF le correlateur DSP TOURNE (atteint 0x8d00, DETECTOR-RUN
    @0x9ac0, d_fb_mode=1) mais `d_fb_det[0x08f8]` reste 0 -> aucune detection FB.
    (Supersede l'ancien "0x8d00 = 0 hit / jamais dispatche" : le mur a bouge.)
  - Quoi : cabler / regler la detection FB en sortie de correlateur -> poser
    `d_fb_det[0x08f8]` : fenetre de detection + dispatch par-trame.
  - Ou : `DSP_ADDRESS_MAP.md` / `DSP_ARM_LINKAGE.md` ; DETECTOR-RUN @0x9ac0 ;
    entree IQ data[0x9213/0x9215] (NATIVE_HELPED : feed_iq DARAM 0x9210).
  - Honnetete : le buffer IQ RX n'est pas cable au recepteur on-chip en QEMU.

- **[P2] Camp natif ("No sysinfo")**
  - Etat : rxlev natif = DONE (-47 dBm reel, modele trf6151/DECAN) mais FB pas
    detecte -> "No sysinfo" -> pas de camp -> pas de LU/SMS en natif.
  - Quoi : une fois d_fb_det pose (P1 ci-dessus), faire remonter les SI pour
    atteindre C3 camped en natif, puis derouler LU / SMS.
  - Depend de : P1 NATIF.

---

## Dette transverse (tous modes, P3)

- Retirer les env `BURST_*` mortes (BURST_FN / BURST_OFS / BURST_ECHO) une fois
  `d_burst_d` WP-mirror valide au run.
- `a_pm` mot 8 vs mot 12 : section A de `on_frame_tick` ecrit sous label "a_pm"
  les idx 0x30/0x44 (= a_serv_demod) ; vraie cellule a_pm = 0x834/0x848.
- Code mort a supprimer : `bsp.fb_valid` jamais mis a 1 ;
  `calypso_dsp_shunt_route_c54x_active()` sans appelant ; API
  `calypso_orch_init/publish` inexistante ; `calypso_tint0_start()` jamais
  reference ; `fw_console.c` sans appelant.
- Decodeur c54x : cluster MAC/LD/BITT 0x30-0x37 mort sous case 0xF ; BC/FB 0xF8
  par nibble ; catch-all FIRS/LMS 0xE000-E3FF ; 0xF6/0xF7 fabriques.
  NE PAS re-fixer 0x72/0x73 MVDM/MVMD sans le side-effect (REVERT_MVMD).
- Threading (roadmap) : un thread par bloc (ARM/DSP/TPU/BSP) pour eliminer les
  ~300 artefacts de serialisation TCG (THREADING_TODO).
- Doc : `DSP_ROM_MAP.md` canonique dit "PROM1 mirroree 0x8000-0xFFFF" ; corriger
  en "chargee 0x18000+ sans mirror" (fix 2026-05-29). Marquer "historique" les
  REPORT_CLAUDE_WEB_* / BOOT_TO_FBSB_SEQUENCE / FBSB_SEQUENCE_TRACE.

---

Note taxonomie : `api_write_cb` (calypso_c54x.h) = callback mort, fausse piste
(cf `ETAT_ACTUEL.md` §6 "ne pas theoriser dessus") -> retire des TODO.
