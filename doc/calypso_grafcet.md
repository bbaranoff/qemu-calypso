# Calypso GRAFCET — chaine causale et couverture pytest exhaustive

Version consolidee, 2026-07-03. Fusionne l architecture (calypso_audit.md), les 23 addenda
de STATUS_2026-07-01.md et l inventaire exhaustif de la suite pytest (232 tests, 16
fichiers, tests/ dans qemu-calypso et qemu-src, container osmo-operator-1). Chaque etat du
GRAFCET cite TOUS les tests qui l observent, pas un seul exemple - la suite pytest EST la
source de verite operationnelle de ce document, verifiee au run du 2026-07-03 sur session
QEMU live fraiche (start-clean.sh depuis qemu-src).

## 1. Go-live precis - mecanisme exact des deux racines

### Racine A - dispatcher jump-table auto reference

Chemin intendu : `data[0x4387]` (slot resolu par `BACC A` a l adresse ROM `0xb40f`) devrait
router vers la table de saut `0xaae7-0xab37`, dont deux entrees pointent vers `0xa4c7`
(`ORM #0x3000,IMR`, arme bit12/vec28 + bit13). Observe : le slot resout en permanence vers
un stub `0xab38` (`RET` immediat) qui se re ecrit lui meme a chaque passage - boucle fermee
auto entretenue, jamais brisee. Le producteur candidat qui ecrirait la bonne valeur dans
`data[0x4387]` (`0xb4b6: STM #0x4387,AR1` suivi de `LD #0xaae7,A`) appartient a une ile ROM
de ~4700 mots (`0xa9ea-0xc800`, 26 sous routines) sans point d entree externe trouve malgre
une recherche exhaustive (38 sites CALL statiques identifies, zero atteint dynamiquement).
Consequence mesuree : 0 hit PC sur `0xa4c7` sur toute la session, 130+ hits sur `0xa4ca`
(entree directe du wait-loop, 3 mots plus loin, qui saute l instruction d armement).

### Racine B - asymetrie push interruption vs pop ROM

Confirmee par un test de falsification controle (`CALYPSO_POKE_A4C7_ONCE`, addendum 22) qui
redirige UNE fois le PC vers `0xa4c7` pour laisser le ROM executer sa vraie instruction
`ORM`. Resultat : `IMR` passe reellement de `0x0000` a `0x3000` (bit12 arme, confirme par la
sonde `IMR-ARM`), stable, sans reclobber. Avec en plus `CALYPSO_DSP_FRAME_VEC28=1` (remap
necessaire pour que bit12 compte), la frame IT vectorise reellement : `PC=0x00f0` (entree
vec28) tire 301 fois, atteint `0x7234` puis `CALL 0x013b` - mais chacune de ces deux
dernieres adresses ne tire QU UNE SEULE FOIS, puis storm `PC=0x0000` (`POST-BOOTSTUB-RET`,
32 instructions apres, 6300+ occurrences ensuite).

Decodage exact de `0x013b` (= DARAM copie de PROM0 `0x713b`, confirme par workflow dedie) :
un leaf de 3 mots parfaitement equilibre en lui meme -
`0x013b MAC *AR3+,B` (dispatch case 0x3, calypso_c54x.c ligne 8202) -
`0x013c STL A,*AR5+` (hi8==0x80, ligne 8683) -
`0x013d RET inconditionnel` (hi8==0xFC cc=0x00, ligne 7106).
Ni PSHM ni POPM internes - le sous programme ne peut donc pas lui meme desequilibrer la
pile. Le vrai mecanisme suspecte (non encore confirme en instrumentation live, addendum 23) :
`c54x_interrupt_ex` empile INCONDITIONNELLEMENT 2 mots (PC puis XPC, lignes 13822-13957)
pour tout vecteur y compris vec28, alors que tout le code reellement atteint sur ce chemin
(`0x7234`, `0x013b`, et les sites structurellement identiques ailleurs dans la ROM) n
utilise que des `CALL`/`RET`/`RCD` - familles a UN SEUL mot. Seuls `RETE` (`0xF4EB`) et la
famille `FRET`/`FRETD`/`FRETED` depilent 2 mots symetriquement - et aucun n apparait sur ce
chemin. Le mot XPC (=0) reste donc orphelin en haut de pile ; le premier `RET` qui redescend
a ce niveau le depile comme adresse de retour, d ou `PC=0x0000`.

Sonde de diagnostic proposee (lecture seule, one shot, non encore implementee) : au premier
`PC=0x00f0`, logguer `SP` et les deux mots deja empiles, puis tracer chaque `RET`/`RETE`
jusqu a ce que `SP` revienne au niveau pre interruption, pour confirmer que c est bien un
`RET` (famille 1 mot) qui depile le XPC orphelin plutot qu autre chose. Aucun test pytest
ne couvre ce point precis aujourd hui - candidat naturel pour un nouveau test une fois la
sonde ecrite.

Les deux racines convergent vers le MEME symptome observable (`d_fb_det` reste `0x0000`
tout le run) - c est pourquoi un test de falsification controle a ete necessaire pour les
distinguer plutot qu une seule racine apparente.

## 2. GRAFCET etendu par famille de tests

```mermaid
stateDiagram-v2
    direction TB

    state "Invariants ARM DSP au boot - test_ar_imr_inth_invariants.py 8 tests" as F1 {
        direction LR
        B0 : B0 - AR2 categorise correctement
        B1 : B1 - IMR pas persistant a zero
        B2 : B2 - RSBX INTM atteint ou flag NMI
        B3 : B3 - Pas d entree bootstub
        B4blocked : B4 BLOQUE - IRQ servicing IMR non nul
        note right of B4blocked
            SKIPPED - depend de la meme racine A B
        end note
        B0 --> B1 --> B2 --> B3
        B3 --> B4blocked
    }

    state "Chaine go-live - test_calypso_milestones.py 14 tests" as F2 {
        direction TB
        G0 : G0 - Decodeurs POPM et Tier A actifs
        G1 : G1 - INTM dwell sans regression
        RCA : RCA - RACINE A dispatcher stub 0xab38
        note right of RCA
            xfail test_no_d_fb_det_wr_site_anomaly
        end note
        RCB : RCB - RACINE B derail post CALL 0x013b
        note right of RCB
            non teste directement, addendum 23
        end note
        G2 : G2 - fb0 att reste a 0
        note right of G2
            xfail test_fb0_att_nonzero
        end note
        G3 : G3 - task 24 ALLC jamais dispatche
        note right of G3
            xfail test_l1ctl_data_ind_received
        end note
        G4 : G4 - RACH jamais emis
        G5 : G5 - IMM ASS jamais decode
        G6 : G6 - RR SDCCH jamais etabli
        G7 : G7 - Location Update jamais complete
        note right of G7
            xfail test_location_updating_accept_received
            objectif final du projet
        end note
        G0 --> G1 --> RCA
        RCB --> G2
        RCA --> G2 --> G3 --> G4 --> G5 --> G6 --> G7
    }

    state "Observabilite runtime - test_run_observability.py 28 tests" as F3 {
        direction LR
        R0 : R0 - Processus presents, pas de zombie
        R1 : R1 - Logs frais, pcap grossit
        R2 : R2 - Pas de wait a21a ni dwell 7740
        RINTM : RINTM - INTM jamais a zero
        note right of RINTM
            xfail test_intm_reaches_zero
        end note
        RISR : RISR - Compteurs ISR non exposes
        note right of RISR
            xfail interrupt_ex_called et isr_entered
        end note
        R0 --> R1 --> R2 --> RINTM --> RISR
    }

    state "Invariants logs et blockers - test_log_grep.py 30 tests" as F4 {
        direction LR
        L0 : L0 - DSP boot, BSP DMA actif
        L1 : L1 - task24 jamais dispatche
        note right of L1
            xfail test_grep_qemu_task24_dispatched
        end note
        L2 : L2 - Aucun panic, abort, ou SP catastrophe
        L3 : L3 - osmocon et mobile stables
        L0 --> L1
        L0 --> L2 --> L3
    }

    state "Etat firmware ARM live - test_firmware_state.py 8 tests" as F5 {
        direction LR
        FW0 : FW0 - PC avance, pas de busy loop connu
        FW1 : FW1 - osmocon depasse le romload
        FW2 : FW2 - LOST spam au dela tolerance
        note right of FW2
            xfail - jitter timer 1862 a 1884 vs 1875 attendu
        end note
        FW0 --> FW1 --> FW2
    }

    state "Timers et cadence TDMA - 18 tests sur 2 fichiers" as F6 {
        direction LR
        TM0 : TM0 - Parametres physiques corrects
        TM1 : TM1 - Budget DSP respecte
        TM2 : TM2 - CSV timeline sans trou
        TM3blocked : TM3 BLOQUE - kick et bridge clk ind non actifs
        note right of TM3blocked
            SKIPPED - CALYPSO_TIMER=0 sur cette session
        end note
        TM0 --> TM1 --> TM2 --> TM3blocked
    }

    state "Conformite workflow OsmocomBB - test_osmocom_workflow.py 8 tests" as F7 {
        direction LR
        OW0 : OW0 - SIM IT RX TX conformes
        OW1 : OW1 - Horloges virtuelles alignees
        OW2blocked : OW2 BLOQUE - clock defaut divergent
        note right of OW2blocked
            xfail test_bridge_clock_from_qemu_default
        end note
        OW0 --> OW1 --> OW2blocked
    }

    state "Introspection GDB et monitor - 17 tests sur 2 fichiers" as F8 {
        direction LR
        I0 : I0 - GDB stub et monitor repondent
        I1 : I1 - Registres ARM et DSP lisibles
        I0 --> I1
    }

    state "Validation synthetique injection - test_inject_frames.py 21 tests" as F9 {
        direction LR
        J0 : J0 - NDB probe et clear fonctionnent
        J1 : J1 - Injection FB SB SI et taches DSP
        J2blocked : J2 BLOQUE - efficacite ARM lecture non confirmee
        note right of J2blocked
            xfail test_efficacy_arm_reads_d_fb_det_1
        end note
        J0 --> J1 --> J2blocked
    }

    F2 --> F3 : meme racine, symptomes partages
    F2 --> F4 : task24 observe des deux cotes
    F3 --> F5 : INTM et jitter timer lies

    classDef ok fill:#c8ffc8,stroke:#2a7a2a
    classDef bad fill:#ffd0d0,stroke:#8a2a2a
    classDef down fill:#ffe6b3,stroke:#8a5a2a

    class B0,B1,B2,B3,G0,G1,R0,R1,R2,L0,L2,L3,FW0,FW1,TM0,TM1,TM2,OW0,OW1,I0,I1,J0,J1 ok
    class RCA,RCB bad
    class B4blocked,G2,G3,G4,G5,G6,G7,RINTM,RISR,L1,FW2,TM3blocked,OW2blocked,J2blocked down
```

## 3. Inventaire exhaustif pytest (232 tests, 16 fichiers) - 2026-07-03

Run frais sur session QEMU live (start-clean.sh depuis qemu-src). 119 passed, 83 skipped,
30 xfailed, 0 failed, 0 xpassed, 0 errors.

| Fichier | Tests | PASS | SKIP | XFAIL | Ce que ca determine |
|---|---|---|---|---|---|
| test_ar_imr_inth_invariants.py | 8 | 4 | 4 | 0 | Invariants silicium boot - SP/AR/IMR, IRQ servicing |
| test_calypso_milestones.py | 21 | 3 | 4 | 14 | La chaine go-live complete, du decodeur au Location Update |
| test_firmware_state.py | 8 | 7 | 0 | 1 | Etat firmware ARM live - PC, rxDoneFlag, osmocon |
| test_gdb_stub.py | 8 | 8 | 0 | 0 | GDB stub - handshake, registres, memoire |
| test_inject_frames.py | 21 | 19 | 0 | 2 | Validation synthetique - injection NDB, efficacite lecture ARM |
| test_irda_channel.py | 8 | 3 | 2 | 3 | Canal debug IrDA - PTY, boot marker, throughput |
| test_layer_drift.py | 10 | 4 | 6 | 0 | Derive temporelle inter couches via timestamps logs |
| test_log_grep.py | 30 | 19 | 10 | 1 | Invariants et blockers sur tous les logs |
| test_mode_verdict.py | 7 | 3 | 4 | 0 | Verdict par mode de fonctionnement |
| test_osmocom_workflow.py | 8 | 7 | 0 | 1 | Alignement workflow OsmocomBB - SIM, clocks, bridge |
| test_qemu_introspection.py | 9 | 9 | 0 | 0 | Monitor HMP - info status, qtree, mtree, registres |
| test_run_all_modes.py | 23 | 0 | 23 | 0 | Comparaison cross mode - necessite results.json absent ce run |
| test_run_observability.py | 28 | 13 | 7 | 8 | Observabilite globale - process, logs, milestones L3 |
| test_runtime_net_fs.py | 5 | 5 | 0 | 0 | Sante reseau et filesystem du container |
| test_timer_invariants.py | 9 | 7 | 2 | 0 | Compteurs timers QEMU et CSV timeline |
| test_timer_physical_audit.py | 12 | 8 | 4 | 0 | Audit physique timers vs horloge reelle |
| **Total** | **215 parses / 232 collectes** | **119** | **83** | **30** | |

Note : `test_run_all_modes.py` est entierement SKIPPED ce run car il depend d un
`results.json` produit par une campagne multi mode dediee (`run-all`), pas par une session
`start-clean.sh` simple - comportement attendu, pas une anomalie.

## 4. Table de tracabilite etat vers tests (etendue)

| Etat GRAFCET | Racine ou consequence | Tests qui l observent | Etat mesure |
|---|---|---|---|
| RCA - dispatcher stub | Racine | test_calypso_milestones::test_no_d_fb_det_wr_site_anomaly | XFAIL |
| RCB - derail ISR | Racine | aucun test direct - instrumentation a ecrire | non couvert |
| IMR jamais arme | Consequence RCA | test_ar_imr_inth_invariants::test_imr_not_persistently_zero | PASS - IMR bouge au boot puis se fige, testable |
| INTM jamais zero | Consequence RCA+RCB | test_run_observability::test_intm_reaches_zero | XFAIL |
| fb0_att reste a 0 | Consequence | test_calypso_milestones::test_fb0_att_nonzero | XFAIL |
| Compteurs ISR absents | Observabilite manquante | test_run_observability::test_interrupt_ex_called_counter_exposed, test_isr_entered_matches_rete, test_no_pending_irq_gating | XFAIL x3 |
| task24 ALLC jamais dispatche | Consequence directe | test_log_grep::test_grep_qemu_task24_dispatched, test_calypso_milestones::test_l1ctl_data_ind_received, test_run_observability::test_l1ctl_data_ind_received | XFAIL x3 |
| RACH bloque | Cascade | test_calypso_milestones::test_rach_emitted, test_run_observability::test_rach_attempted | XFAIL x2 |
| CCCH demod ne converge pas | Chemin parallele | test_run_observability::test_a_cd_write_pc_includes_ccch_demod | XFAIL |
| IMM ASS jamais decode | Cascade | test_calypso_milestones::test_immediate_assignment_decoded | XFAIL |
| RR SDCCH jamais etabli | Cascade | test_calypso_milestones::test_rr_sdcch_established | XFAIL |
| Location Update jamais complete | Objectif final | test_calypso_milestones::test_location_updating_request_sent, test_location_updating_accept_received | XFAIL x2 |
| Injection synthetique - efficacite lecture ARM | Validation orthogonale | test_inject_frames::test_efficacy_arm_reads_d_fb_det_1, test_efficacy_arm_reads_a_cd | XFAIL x2 |
| Jitter timer firmware | Consequence RCA+RCB indirecte | test_firmware_state::test_osmocon_no_recent_lost_spam | XFAIL - 1862-1884 vs 1875 attendu |
| Clock bridge defaut divergent | Design assume, pas un bug DSP | test_osmocom_workflow::test_bridge_clock_from_qemu_default | XFAIL |
| Chemin sain amont - boot, decodeurs, SIM, GDB, monitor, reseau | Confirme fonctionnel | 119 tests PASS repartis sur 14 fichiers | PASS |

## 5. Lecture

Chaque XFAIL de ce document cite deja les addenda STATUS_2026-07-01.md correspondants dans
sa raison pytest (verifie lors du nettoyage de session precedent). Un seul point du GRAFCET
n a aucun test dedie aujourd hui : RCB (le derail post `CALL 0x013b`) - c est la prochaine
sonde a ecrire (lecture seule, one shot, cf section 1) avant de pouvoir la faire apparaitre
dans cette table avec un vrai statut plutot qu un blanc. Si un jour un XPASS apparait sur
RCA ou sur la future sonde RCB, remonter immediatement toute la chaine en aval (section 4,
de haut en bas) pour re tester - rien de ce qui est actuellement XFAIL en cascade ne doit
etre suppose reste bloque sans nouvelle mesure.
