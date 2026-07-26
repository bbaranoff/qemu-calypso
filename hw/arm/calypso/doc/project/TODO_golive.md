# TODO — go-live DSP faithful (handshake 0x098a-0x098e)

État : outillage de reprise COMPLET. Reste 1 inconnue = le tuple par-cellule qui route
la state machine (0xdde0-0xde9f) vers le setter 0xde9c (ST #2,0x3f70 → bit1 → sortie wait-loop).

## Ce qui est cablé et prouvé (ne pas refaire)
- [x] calypso_arm2dsp.c : bridge ARM→DSP, hooks write + step, commité.
- [x] IRQ-LEVEL (calypso_c54x.c) : sert les IT latchées (ifr&imr) quand INTM=0, vectorise
      vec28→IPTR correct. ISR native tourne saine une fois le contexte posé.
- [x] Fix api_ram : le hold écrit s->api_ram[cell-0x0800] (le DSP lit api_ram, pas data[]).
      → passe la divergence #1 (0xdded : A=0xffff, pas de reclear).
- [x] Sonde SM-TRACE (CALYPSO_SM_TRACE=1) : PC+op+A+TC+data[0x098a..e], cap 400, 0xdde0-0xde9f.
- [x] Hold par-cellule : CALYPSO_ARM2DSP_HS_A..HS_E (fallback uniforme CALYPSO_ARM2DSP_HS).

## Divergence #2 (à franchir) — boucle de calcul d'index 0xde0d-0xde26
- 0xde0e : ADD *(0x098b), A
- 0xde22 : A double par passe (0x0002→0x0004→0x0008…)
- 0xde26 : BC 0xde0d si TC (reboucle)
Avec 0xFFFF uniforme → shift sans converger vers l'index de de9c.

## Procédure de reprise (bracketer le tuple)
1. Config direction-2 (voir STATUS addenda) + :
   ```
   CALYPSO_SM_TRACE=1
   CALYPSO_ARM2DSP_HS_A=1     # 0x098a != 0 (passe div #1)
   CALYPSO_ARM2DSP_HS_B=<x>   # pilote ADD/index — bracketer
   CALYPSO_ARM2DSP_HS_C=1
   CALYPSO_ARM2DSP_HS_D=<y>   # évite boucle 0xde82 — bracketer
   CALYPSO_ARM2DSP_HS_E=<z>   # bracketer
   ```
2. Lire SM-TRACE : suivre PC dans 0xde0d-0xde79, viser la sortie vers 0xde9c.
3. Succès = `F70-SETBIT1` avec insn >> 4385 (setter hors boot) → ROM sort de la wait-loop,
   atteint 0xa582 (IMR=0x52fd), pose son contexte → ISR native (déjà câblée) tourne saine.
4. Vérif finale : d_fb_det passe à 1, FBSB_RESP réel émis (plus synthétisé).

## Nettoyage (fin de RE, avant commit propre)
- [ ] Retirer sondes/leviers diag : SM-TRACE, HS/HS_A..E, DELIVER-LOOP, FORCE-*, WAIT-TEST,
      GOLIVE-* (garder derrière gate OFF par défaut ou supprimer).
- [ ] Backups .bak_golive à ranger.
- [ ] MAJ STATUS avec le tuple trouvé + retrait de la section "à trouver".

## Décodage ROM restant (le raccourci qui économise les itérations)
Plutôt que bracketer à l'aveugle : décoder statiquement calypso_dsp.txt rows 3564-3579
(0dda0-0de90) pour DÉRIVER le tuple qui donne l'index de la ligne 0xde9c dans la table
dispatch ~0xde27. C'est de l'arithmétique pure (ADD *(0x098b)/double/BC TC) → calculable.
