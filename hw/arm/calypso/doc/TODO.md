# TODO - Calypso QEMU (consolide 2026-07-26)

Priorites : P0 = debloque le camp / la boucle fonctionnelle courante ;
P1 = portage natif (sortir du shunt) ; P2 = nettoyage / dette.
Reference d'adresses : SHUNT_LEGIT_ADDRESS_MAP.md.

## P0 - Camp et boucle fonctionnelle

- **[P0] Valider d_burst_d WP-mirror per-page au run**
  - Quoi : confirmer le design final s_wp_burst_d avec CALYPSO_SHUNT_BURST_PERCMD=0,
    retirer BURST_FN / BURST_OFS / BURST_ECHO.
  - Pourquoi : d_burst_d est la derniere brique du camp non validee au run ;
    l'echo RP0 0xFFD00052 / RP1 0xFFD0007A assure le double-read lockstep.
  - Ou : calypso_shunt / on_frame_tick ; SHUNT_LEGIT_ADDRESS_MAP.md.

- **[P0] Boucler le Location Update**
  - Quoi : RACH UL -> IMM ASSIGN -> SDCCH -> LU accept.
  - Pourquoi : le camp est atteint (C3 camped) mais la LU ne se termine pas ;
    blocage RACH UL restant.
  - Ou : chemin UL calypso_trx / bridge ; Task ID RACH=10.

- **[P0] Basculer le bridge bursts DL vers canal direct**
  - Quoi : bursts DL passent par PTY/DLCI 4 au lieu du canal BSP direct.
  - Pourquoi : point d'implementation errone signale (SERCOMM_GATE section 5).
  - Ou : basculer vers UDP/socket direct -> calypso_trx_rx_burst -> BSP.

## P1 - Portage natif (sortir du shunt, voie dsp_revival)

- **[P1] Debloquer le correlateur natif RANK3 (mur data[0x3fae] bit8)**
  - Quoi : le handler FB boucle 0x90b0-0x9130 sur BITF data[0x3fae] bit8 jamais
    ecrit ; kernel 0xa076 jamais atteint ; 0x8341 / 0x8d00 = 0 hit.
  - Pourquoi : c'est le mur central de la voie native (RANK3).
  - Ou : router 0x013b -> 0x8341 pour atteindre le dispatcher LUT natif (programme
    AR3/AR5 sur base 0x2a00 + BK/BRC) ; test CALYPSO_FORCE_3FAE.

- **[P1] Cabler un event TPU per-burst a la fenetre FB/SB**
  - Quoi : au bord fenetre RX, (a) lever IMR bit9 pour router 0x7234->0x8341,
    (b) ecrire 0x8d00 dans le slot dispatch data[0x43c0] / [0x4387].
  - Pourquoi : signal dynamique qui remplace les constantes boot 0xa4c7/0xab38.
  - Ou : DOC_PATH_BOOT_TO_CORRELATOR ; calypso_tsp.c / calypso_bsp.c.

- **[P1] Activer BSP_DIRECT_FEED pour peupler 0x2a00 chaque trame**
  - Quoi : CALYPSO_BSP_DIRECT_FEED=1 requis ; le path FN-gated deliver_buffered
    ne matche jamais sous icount=auto.
  - Pourquoi : correlateur affame (0x2a00 = 0 hit d'acces dans le log).
  - Ou : DOC_CORREL_READ_PATH ; calypso_bsp.c.

- **[P1] Armer le handshake go-live ARM->DSP (Fix A BGEN)**
  - Quoi : ARM ecrit cellules 0x098a / 0x098c (d_background_enable/state) via
    calypso_arm2dsp.c ; CALYPSO_ARM2DSP_BGEN=1.
  - Pourquoi : DSP sort du wait-loop 0xa4ca/0xa4d0 et deroule init lineaire
    (0xa4c7 ORM / 0xa51b RSBX / 0xa582 IMR). ATTENTION tension a valider avec
    ndb-cells-098a-background-redherring (098a dit red herring).
  - Ou : RAPPORT_GOLIVE_2026-07-25.md ; project/TODO_golive.md.

- **[P1] Assigner api_write_cb**
  - Quoi : api_write_cb declare (calypso_c54x.h:204) mais jamais assigne (grep=0)
    -> ecritures NDB du DSP invisibles a l'ARM.
  - Pourquoi : le handshake go-live natif ne peut etre observe cote ARM sans ce cb.
  - Ou : calypso_c54x.c / calypso_arm2dsp.c.

- **[P1] Fixer le shadow IMR 0x435b**
  - Quoi : data[0x435b]=0 jamais init -> 0xa582 ecrit IMR=0, ecrase l'arm 0xa4c7 ->
    tout masque -> correlateur jamais dispatche.
  - Pourquoi : racine finale go-live (deadlock).
  - Ou : golive-imr-shadow-435b (MEMORY) ; calypso_dsp_shunt.c:383/506.

- **[P1] Resoudre divergence #2 TODO_golive (setter 0xde9c)**
  - Quoi : boucle index 0xde0d-0xde26 ; tuple par-cellule routant la SM
    0xdde0-0xde9f vers setter 0xde9c (ST #2,0x3f70 bit1 -> sortie wait-loop).
  - Ou : project/TODO_golive.md.

- **[P1] Sonde diag racine B go-live (lecture seule)**
  - Quoi : derail post-CALL 0x013b en contexte ISR vec28 -> storm PC=0x0000 ;
    suspect c54x_interrupt_ex empile 2 mots (PC+XPC) alors que le chemin n'utilise
    que CALL/RET 1-mot. Sonde proposee NON implementee.
  - Ou : calypso_grafcet.md racine B addendum 23 ; c54x_interrupt_ex.

- **[P1] Verifier delay-slots dans c54x_run**
  - Quoi : delay_slots-- s'execute dans la meme iteration qui pose delay_slots=2 ->
    un seul slot execute au lieu de 2 ; suspecte sur delay-slots 2 instructions 1-mot.
  - Ou : c54x_run ~L11414-11429.

- **[P1] Finir le bring-up C54x pour vocoder natif**
  - Quoi : cabler I/Q reel (AFC) + codec ABB virtuel pour la voix.
  - Ou : project/REACTIVER_DSP_REEL.md ; STEP1/2/3.

## P2 - Nettoyage, dette, doc

- **[P2] Retirer les env BURST_* mortes** : BURST_FN / BURST_OFS / BURST_ECHO une
  fois d_burst_d valide (voir P0).
- **[P2] Corriger a_pm mot 8 vs mot 12** : section A de on_frame_tick ecrit sous
  label "a_pm" les idx 0x30/0x44 qui sont a_serv_demod ; vraie cellule a_pm =
  0x834/0x848. Divergence a nettoyer.
- **[P2] Code mort a supprimer** :
  - bsp.fb_valid jamais mis a 1 -> calypso_bsp_get_fb_detection() toujours faux
    (regression refactor TONAL_FB).
  - calypso_dsp_shunt_route_c54x_active() exportee sans appelant reel.
  - API calypso_orch_init/publish (9 fn) inexistante ; seul calypso_orch() reel.
  - calypso_tint0_start() jamais reference (calypso_tint0.c:42-45) ; TINT0 dead.
  - fw_console.c zero appelant ; calypso_bsp_rx_burst / sercomm_gate_init wiring
    header sans call-site.
  - Diagramme calypso_fbsb.h:14-29 (FBSB_FAIL apres 12) ne correspond a aucune
    logique -> corriger/supprimer.
- **[P2] Corriger les bugs decodeur confirmes non fixes** : cluster MAC/LD/BITT
  0x30-0x37 mort sous case 0xF ; BC/FB family 0xF8 par nibble (ne lit pas cond) ;
  catch-all FIRS/LMS 0xE000-E3FF ; 0xF6/0xF7 fabriques. NE PAS re-fixer 0x72/0x73
  MVDM/MVMD sans traiter le side-effect compensatoire (REVERT_MVMD).
- **[P2] Propager la correction PROM1 no-mirror** : canonique DSP_ROM_MAP.md dit
  "PROM1 mirroree 0x8000-0xFFFF" ; archive corrige "chargee 0x18000+ sans mirror"
  (fix 2026-05-29). Corriger le canonique.
- **[P2] Trancher la contradiction redirect silicon 0xFF80->0x7120** : TODO.md
  racine dit "modele HW permanent reactive 2026-05-30" ; archive/TODO.from-src-doc
  (pole 06-25) dit NEUTRALISE par defaut, gate CALYPSO_REDIR_LEGACY=1, HACK.
- **[P2] Retirer/gater les hacks env avec critere** : CALYPSO_BSP_BYPASS_BDLENA
  (champ inerte selon archive/TODO) ; toutes les injections shunt gatees defaut OFF.
- **[P2] Marquer "historique" dans README** : BOOT_TO_FBSB_SEQUENCE, FBSB_SEQUENCE_
  TRACE, FBSB_FLOW section 10-11, et tous les REPORT_CLAUDE_WEB_* (voie native).
- **[P2] Supprimer/consolider les doublons archive** : archive/*.from-src-doc.md
  (unification 2026-07-26), en preservant les 2 corrections faisant foi
  (REGISTERS_REVIEW : dsp-registers existe ; DSP_ROM_MAP : PROM1 no-mirror).
- **[P2] Threading (roadmap)** : mettre chaque bloc (ARM/DSP/TPU/BSP) dans son
  thread pour eliminer les ~300 artefacts de serialisation TCG (THREADING_TODO).
- **[P2] Nettoyer les residus SP_CATASTROPHE 2e ordre** : DP=0x000 via pc=0x7656
  @628-643M ; wedge 0xa47x INTM=1 poll 0x8a44 (hors fenetre FBSB).
