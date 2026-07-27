# STATUS - Etat reel du projet Calypso QEMU (2026-07-26)

Reference d'adresses : SHUNT_LEGIT_ADDRESS_MAP.md. Voie qui campe = SHUNT_LEGIT
(option 3, format natif dans data[]). Toute la pile "dsp_revival" concerne la
voie NATIVE = le portage restant, pas le camp fonctionnel.

## Ce qui MARCHE - camp SHUNT_LEGIT complet

Le mobile CAMPE de bout en bout. gr-gsm decode le downlink ; le shunt reinjecte
au FORMAT NATIF dans s->dsp->data[] (fait central : data[off/2 + 0x800], confirme
par 3 sources : le firmware ARM lit via calypso_dsp_read trx.c:237-245).

Chaine par etage :
- **FB** : d_fb_det force @ DSP 0x08F8 (ARM 0xFFD001F0) ; a_sync_demod TOA/PM/
  ANGLE/SNR @ 0x08FA-0x08FD (poser ANGLE=rx_afc ou 0, SNR=0x7000 pour eviter drift
  AFC). a_sync_demod force.
- **SB** : BSIC=7, a_sch CRC (bit8 @ word 0x96), a_serv_demod P0 @ 0x0830.
- **rxlev (RANK5)** : modele TRF6151, a_pm P0 @ 0x0834 recalcule vivant a chaque
  write TSP REG_RX ; apm_for_rf(target) tient le RF cible quel que soit l'AGC ;
  reset REG_RX 0x9E00 -> gain total 138.
- **BCCH** : a_cd[0] @ 0x09D2 (SI a a_cd[3] @ 0x09D5) ; intercept d_task_d /
  d_burst_d.
- **SI1-4** decodes -> C3 camped normally -> **Location Update lance**.

Config firmware confirmee : DSP=36, CHIPSET=12, ANLG_FAM=2 (Iota),
W_A_DSP_IDLE3=1. Task IDs coherents : FB=5, SB=6, ALLC(CCCH)=24, RACH=10.
d_dsp_page=0x08D4 (PAS 0x08E2 = d_dsp_state=3), d_ctrl_system=0x0810.

Briques terminees :
- **RANK1** (pont ARM->DSP 0x0810 / IMR shadow) - DONE (cote shunt).
- **RANK4** (recalage FN -552 dans calypso_trx_get_fn, FN calee sur SCH + LOST
  timer read-driven) - DONE.
- **RANK5** (TRF6151 / rxlev) - DONE.

Infra : ~50 tests observabilite verts, GDB stub OK, injection NDB 19/19, suite
pytest ~232 tests d'instrumentation.

## EN COURS

- **Validation d_burst_d au run** : design final = WP-mirror per-page
  (s_wp_burst_d), CALYPSO_SHUNT_BURST_PERCMD=0, retirer BURST_FN / BURST_OFS /
  BURST_ECHO. A confirmer au run.
- **RANK2** (fenetre RX BDLENA / BSP) : gate calypso_iota_take_bdl_pulse() jamais
  appele, livraison DL inconditionnelle (calypso_bsp.c:977-982). En cours.

## BLOQUE - voie native RANK3 (correlateur)

Le correlateur DSP 0x8d00 n'est jamais atteint (0 hit), meme reseau I/Q propre
livre (tonale +Fs/4 prouvee dans DARAM 0x2a00, span [0x2a00..0x2b28), 296 int16).
Mur = data[0x3fae] bit8 jamais ecrit ; handler FB boucle sans jamais atteindre
le kernel 0xa076 / 0x9a80. Details :
- Prologue 0xf0 -> 0x7234 -> 0x013b -> 0xa4e4 deraille vers overlay 0x013b au lieu
  de tomber sur 0x8341 (LUT FB qui installerait 0x8d00). grep 0x8341=0, 0x8d00=0.
- GATE-1 : BITF data[0x0810] bit15 (d_ctrl_system) reste 0.
- GATE-2 : d[0x3fde] (FB task-pending) epingle a 1.
- AR3 @0xee38 pointe 0x2b97 (hors buffer), AR5=0xdb7b staged.
- IMR=0x0000 tout le run (voie native pure) ; contredit par le build live recent
  (Fix A BGEN : IMR=0x52ed arme) - a reconcilier.

Fixes deja landes SANS effet sur ce mur : DADST/DSADT famille 0x50-0x5F
(c54x.c:8232-8305), PORTW 0x75 (c54x.c:7501), CC/CCD + SP-catastrophe (poison
28868 elimine, 3.19M->628M insn).

## NON COMMENCE

- **Location Update complet** : RACH UL -> IMM ASSIGN -> SDCCH -> LU accept.
  Blocage RACH UL restant.
- **Portage natif des hacks vers l'ARM** : rendre le go-live natif (ARM ecrit
  0x098a/0x098c via calypso_arm2dsp.c) pour sortir du shunt ; api_write_cb declare
  (calypso_c54x.h:204) jamais assigne (grep=0).
- **Nettoyage code** : env BURST_* mortes, doublons doc archive, code mort
  (bsp.fb_valid jamais mis a 1, calypso_orch pub/sub inexistant, etc.).

## Table des RANKs

| RANK | Sujet | Statut |
|---|---|---|
| RANK1 | Pont ARM->DSP 0x0810 / IMR shadow | DONE (shunt) ; GAP cote natif |
| RANK2 | Fenetre RX BDLENA (BSP) | EN COURS |
| RANK3 | Correlateur natif 0x8d00 / data[0x3fae] bit8 | BLOQUE (mur) |
| RANK4 | Recalage FN -552 + LOST timer | DONE |
| RANK5 | TRF6151 / rxlev | DONE |
