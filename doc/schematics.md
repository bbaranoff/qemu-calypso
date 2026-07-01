# Schematics — Référence visuelle du câblage DSP C54x / ARM Calypso

> ⚠️ **PÉRIMÉ (audit doc↔code 2026-07-01, voir [DOC_CODE_AUDIT.md](DOC_CODE_AUDIT.md)).** Ce doc décrit un état/une API qui ne correspond plus au code. Vérité-terrain : `d_fb_det` reste 0, le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), IMR=0x0000 jamais ré-armé (jamais re-touché après le clear boot @0xb37e), `api_write_cb` jamais câblé (déclaré `calypso_c54x.h`, référencé `calypso_c54x.c:3357-3358`, mais AUCUNE affectation `api_write_cb = …`), pas de bus ORCH. **Le levier `CALYPSO_DSP_FRAME_VEC28` est un CUL-DE-SAC** (il force-vectorise mais atterrit dans l'épilogue ISR → déraille vers le boot-stub, IMR reste 0) — ce n'est PAS le correctif. Le vrai manque est le handshake go-live ARM→DSP. Corrections ci-dessous.

> Objectif projet : faire tourner le VRAI DSP TMS320C54x émulé pour qu'il détecte
> FB/FCCH et écrive `d_fb_det != 0`. Contrainte (règle #1) : AUCUN hack / AUCUN poke
> de l'état interne DSP — on ne corrige que le CÂBLAGE de l'émulateur.
>
> Chaque schéma est légendé avec des citations `fichier:ligne`. Les faits sont
> alignés sur l'audit + la synthèse (run live `/root/qemu.log`). Les corrections
> apportées par la synthèse au cours de ce run sont taguées **[CORRIGÉ CE RUN]**.
>
> **Diagnostic terminal (engagé) :** `d_fb_det=0` parce que l'IMR du DSP est mis à
> `0x0000` au boot (`calypso_c54x.c` BOOT-MMR-WR #7, `PC=0xb37e op=0x7700`) et
> jamais ré-armé ⇒ l'IRQ frame livrée de façon fiable (`calypso_trx.c:1786`) est
> masquée en permanence. ~~Le correctif est du câblage d'interruption émulateur
> (`calypso_c54x.c:13547-13565`), PAS un nouveau pipe ni un refactor.~~ — **FAUX :**
> le remap VEC28 (`CALYPSO_DSP_FRAME_VEC28`, `calypso_c54x.c:13549-13565`) est un
> CUL-DE-SAC : il force-vectorise mais atterrit dans l'épilogue ISR → le DSP déraille
> vers le boot-stub (POST-BOOTSTUB-RET, PC=0x0000), IMR reste `0x0000`, `d_fb_det`
> reste 0. Le vrai manque est le **handshake go-live ARM→DSP** : l'ARM n'écrit que
> `0x0000` dans les cellules API `0x0314/0x0318` et `api_write_cb` (le notify DSP→ARM)
> n'est jamais enregistré (`calypso_c54x.c:3357-3358`, aucune affectation).

---

## Légende des marqueurs

```
[OK]     chemin câblé et vivant à l'exécution (prouvé dans qemu.log)
[GAP]    rupture fonctionnelle identifiée par l'audit
[DEAD]   code présent mais jamais exécuté dans ce run
[EXP]    levier expérimental gaté par env, OFF par défaut
```

---

## (A) Pipeline bout-en-bout : radio → DSP → ARM

```mermaid
flowchart LR
    OSMO["osmo-trx<br/>(UDP TRXD)"] -->|socket| DRAIN["bsp_drain_cb<br/>timer 5ms<br/>calypso_bsp.c:695"]
    DRAIN --> DELIV["deliver_buffered<br/>calypso_bsp.c:1107"]
    DELIV -->|AFC in-place<br/>apply_phase :1243| AFC["sl.iq post-AFC"]
    AFC -->|"data[0x2a00+woff]=iq<br/>calypso_bsp.c:1259"| DARAM["DARAM I/Q window<br/>0x2a00..0x2b27<br/>len 296 (:795)"]
    AFC -.->|"c54x_bsp_load<br/>bsp_buf :1248"| BSPBUF["bsp_buf (PORTR)"]

    DARAM -.->|"LECTURE JAMAIS FAITE<br/>0 hits IQ-READ :1620"| CORR["Corrélateur FCCH<br/>ROM 0xa04c/0xa070<br/>DADST/DSADT+CMPS"]
    CORR -->|"écrit d_fb_det<br/>data[0x08F8]"| DFB["d_fb_det<br/>(NDB)"]
    DFB -->|"ARM lit 0x01F0<br/>calypso_trx.c:247"| PRIM["prim_fbsb<br/>fbsb FB0_SEARCH"]
    PRIM --> CONF["FBSB_CONF<br/>(vers L1/osmocom)"]

    classDef ok fill:#1b5e20,color:#fff,stroke:#0d3010;
    classDef gap fill:#8b0000,color:#fff,stroke:#4d0000;
    classDef dead fill:#555,color:#ddd,stroke:#222;
    class OSMO,DRAIN,DELIV,AFC,DARAM ok;
    class CORR,DFB gap;
    class BSPBUF dead;
```

**Légende A.** Côté ÉCRITURE **[OK]** : le radio→DARAM est vivant, `delivered=125093`
rafales post-AFC écrites dans `data[0x2a00]` (`calypso_bsp.c:1259`, window par défaut
`daram_addr=0x2a00 len=296` `:795`). Côté LECTURE **[GAP]** : le corrélateur ROM ne
tourne jamais — `0` hit `IQ-READ`/`SPAN-READ` (`calypso_c54x.c:1620/1648`), `0` ref
`2a00` dans tout le log. Donc `d_fb_det(0x01F0)` reste `0x0000` et `fbsb FB0_SEARCH`
garde `fb0_ret=0`. La fenêtre est écrite mais jamais échantillonnée : la cause est
en aval (le DSP ne dispatch jamais la tâche FB), pas un feed manquant. `bsp_buf`/PORTR
est **[DEAD]** ce run (`calypso_bsp.c:1248` chargé mais PORTR jamais consommé).

---

## (B) Orchestration ARM↔DSP : mailbox + réveil + BSP

```mermaid
flowchart TB
    subgraph MB["MAILBOX API-RAM — 0xFFD00000 / 64K  (calypso_trx.c:945,2331)"]
        direction LR
        LAW["Loi d'adresse<br/>ARM byte off  ⇔  DSP word 0x0800+off/2<br/>write trx.c:604 · read trx.c:247<br/>C54X_API_BASE 0x0800 (c54x.h:21)"]
        F1["d_task_md<br/>ARM 0x0008/0x0030 ⇔ 0x0804/0x0818"]
        F2["d_dsp_page<br/>ARM 0x01A8 ⇔ 0x08D4"]
        F3["d_fb_det<br/>ARM 0x01F0 ⇔ 0x08F8"]
    end

    ARM["ARM / L1 host"] -->|"dual-write dsp_ram + data<br/>trx.c:594/604-611"| MB
    MB -->|"ARM lit data[] :247"| ARM
    DSP["DSP C54x"] -->|"dual-write api_ram + data<br/>c54x.c:3314/3659"| MB
    MB -->|"DSP lit api_ram :2230"| DSP

    subgraph WAKE["RÉVEIL / IRQ FRAME"]
        TPU["TPU_CTRL DSP_EN bit4 (force_pulse)<br/>INT_CTRL DSP_FRAME (inversé, tpu_armed)<br/>calypso_trx.c:1747/1775"]
        FIRE["fire=1 → c54x_interrupt_ex<br/>calypso_trx.c:1786<br/>vec19 / IMR-bit3"]
        IEX["c54x_interrupt_ex<br/>calypso_c54x.c:13533"]
        TPU --> FIRE --> IEX
    end

    subgraph BSP["BSP IRQ lines"]
        INT3B["INT3/FRAME vec19 bit3<br/>bsp.c:1003/1145"]
        BRINT["BRINT0 vec21 bit5<br/>bsp.c:1098/1305/1368"]
    end
    INT3B --> IEX
    BRINT --> IEX

    IEX -->|"unmasked=(imr&bit) :13593"| DEC{"IMR=0x0000 ?"}
    DEC -->|"masqué → IFR set + pc++<br/>PAS de vectorisation :13622"| DFBZERO["d_fb_det reste 0"]

    GAPA["GAP A : vec MISMATCH 19 vs 28<br/>scheduler FB firmware = vec28/bit12<br/>c54x.c:13538 · REPORT_...IQ_CABLAGE.md:14"]:::gap
    GAPB["GAP B : INTM=1 bloqué<br/>0 RETE · 0 INTM=0 (qemu.log)"]:::gap
    GAPC["GAP C : I/Q lue jamais<br/>0 IQ-READ 0x2a00 (c54x.c:1620)"]:::gap
    IEX -.-> GAPA
    DEC -.-> GAPB
    DARAMREF["DARAM 0x2a00"] -.-> GAPC

    classDef gap fill:#8b0000,color:#fff,stroke:#4d0000;
```

**Légende B.** Le mailbox **[OK]** : mapping correct et bidirectionnel prouvé au runtime
(ARM poste `d_dsp_page=0x0002`, `d_task_md=0x0001` ; DSP écrirait `d_fb_det`). Loi
d'adresse `ARM_byte → DSP_word 0x0800+off/2` (`calypso_trx.c:604/247`), `api_ram`
alias de `dsp_ram` (`c54x_set_api_ram` `trx.c:2374`), cohérence par dual-write des deux
côtés. Le réveil **[GAP A+B]** : l'IRQ frame arrive en vec19/bit3 (`calypso_trx.c:1786`,
`C54X_INT_FRAME_VEC=19/BIT=3` `calypso_c54x.h:126-127`) mais `IMR=0x0000` ⇒
`unmasked=false` (`c54x.c:13593`) ⇒ branche IDLE-wake masquée `s->pc++` (`c54x.c:13622`),
JAMAIS de vectorisation. Le scheduler FB firmware attend vec28/bit12 (`c54x.c:13538`) :
double rupture (mauvais vecteur + masqué). **[GAP C]** downstream : `0x2a00` jamais lue.

---

## (C) Memory map DSP (data / prog, OVLY, API-RAM, DARAM I/Q)

| Zone | Adresse (DSP word) | Rôle | Référence |
|---|---|---|---|
| Reset vector | `0xFF80` (IPTR=0x1FF boot) | entrée reset ROM (formule générique `pc=(iptr*0x80)+vec*4`, PAS un site de reset — coïncide 0xFF80 pour iptr=0x1FF/vec=0) | `calypso_c54x.c:13618` |
| Stub RET haut | `0xFFCC` (vec19 sous IPTR=0x1FF) | RET-terminated, boot uniquement | C54X_DECODER_AUDIT.md:76-112 |
| API-RAM base | `0x0800` (`C54X_API_BASE`) | mailbox ARM↔DSP | `calypso_c54x.h:21` |
| d_task_md | `0x0804` / `0x0818` | commande tâche | `calypso_c54x.c:1696` |
| d_task_md (mirror) | `0x058a` | write-page → DARAM | qemu.log task_md(058a) |
| write-page DMA | `0x0584/0585/0586` | commit MCU→DSP | `calypso_trx.c:952-1003` |
| d_dsp_page | `0x08D4` | B_GSM_TASK bit1 | `calypso_bsp.c:986-993` |
| d_fb_det | `0x08F8` | flag détection FB | `calypso_c54x.c:3390` |
| DARAM I/Q window | `0x2a00..0x2b27` (len 296) | entrée corrélateur FCCH | `calypso_bsp.c:795/1259` |
| soft-vector go-live | `0x3f6d` = 0xa4df | trampoline 0x71f4 | `calypso_c54x.c:11898-11929` |
| go-live poll flag | `0x3f70` bit1 (0x0002) | wait-loop 0xa4d4 | `calypso_c54x.c:11945-11955` |

```
       DSP DATA SPACE (mots 16 bits)                DSP PROG SPACE
   ┌───────────────────────────────┐          ┌───────────────────────────┐
   0x0800 ┤ API-RAM (mailbox)       │          0xA04C ┤ handler vec19 (IPTR=0x140)
   0x0804 ┤   d_task_md             │          0xA070 ┤ scheduler FB vec28  ◀── cible
   0x08D4 ┤   d_dsp_page (bit1 GSM) │          0xA4C7 ┤ go-live (ORM #,IMR)
   0x08F8 ┤   d_fb_det   ◀── GOAL   │          0xA4D4 ┤ wait-loop (0x3f70 b1)
   ...    ┤                         │          0xA582 ┤ arme IMR=0x52fd
   0x2A00 ┤ I/Q window (296)  [OK écrit]        0xB37E ┤ STM #0,IMR  (efface IMR!)
   0x2B27 ┤   ...           [GAP jamais lue]     0xFF80 ┤ reset / 0xFFCC RET stub
   0x3F6D ┤ soft-vector 0xa4df      │          └───────────────────────────┘
   0x3F70 ┤ go-live poll (bit1)     │
   ┴───────────────────────────────┘
   OVLY : API-RAM (dsp_ram) aliasée sur DARAM data[] via c54x_set_api_ram (trx.c:2374)
```

**Légende C.** L'espace data porte le mailbox (`0x0800+`), la fenêtre I/Q corrélateur
(`0x2a00`, écrite `delivered=125093` fois `calypso_bsp.c:1259`) et les cellules go-live
(`0x3f70`). L'`api_ram` est un alias de `dsp_ram` (`calypso_trx.c:2374`) : les trois vues
(`dsp_ram` / `dsp->data` / `api_ram`) sont maintenues cohérentes par dual-write. Le
scheduler FB firmware vit en prog `0xA070` (vec28, IPTR=0x140), tandis que l'IRQ livrée
tombe sur vec19 → `0xA04C`, et au boot sur le stub RET `0xFFCC` (IPTR=0x1FF).

---

## (D) Machine à états boot → FB dispatch (où ça bloque)

```mermaid
stateDiagram-v2
    [*] --> Reset : PC=0xFF80 IPTR=0x1FF
    Reset --> BootInit : boot ROM
    BootInit --> ClearIMR : PC=0xb37e op=0x7700<br/>STM 0,IMR  IMR 0x3000 vers 0x0000<br/>insn=1047
    ClearIMR --> RelocIPTR : PMST-WR val=0x0038<br/>IPTR relocalise PC=0xb38f insn=1874
    RelocIPTR --> BootStack : PC=0xb382 SP 0x1100 vers 0x5ac8
    BootStack --> Trampoline : 0x71f4 LD 0x3f6d,A puis BACC A<br/>soft-vector=0xa4df
    Trampoline --> WaitLoop : 0xa4d4-0xa4e2<br/>teste data 0x3f70 bit1<br/>INTM=1  IMR=0x0000

    WaitLoop --> WaitLoop : bit1=0 lit 0x0000<br/>BLOCAGE ICI
    WaitLoop --> GoLive : bit1=1 jamais atteint
    GoLive --> ArmIMR : 0xa4c7 ORM IMR puis 0xa582<br/>IMR=0x52fd bit12
    ArmIMR --> FBDispatch : vec28 scheduler<br/>correlateur ecrit d_fb_det
    FBDispatch --> [*]

    note right of WaitLoop
      BLOCAGE (audit finding 3)
      bit1 setter ~0xde9c gate TC sur
      cellules 0x0314/0x0318 (ARM API)
      ARM n ecrit que val=0x0000 fn=0
      HS-ARM-GATE trx.c:553-566 gate l.561
      DE-BR data 0x3f70=0x0001 TC=0
      jamais 0x0002  F70-SETBIT1 count=0
    end note
```

**Légende D.** Le boot efface l'IMR à `0x0000` (`STM #0,IMR` `PC=0xb37e op=0x7700`
insn=1047) **[CORRIGÉ CE RUN : IMR reste 0x0000 tout le run, jamais ré-armé — 0 ligne
`INTM=0`, 0 RETE]**. **[CORRIGÉ CE RUN]** contrairement à l'audit finding #1, l'IPTR EST
relocalisé tôt (`PMST-WR val=0x0038` `PC=0xb38f insn=1874`), donc le stub RET `0xFFCC`
n'est touché que pendant les ~1874 premières insns. Le DSP entre la wait-loop `0xa4d4`
et teste `data[0x3f70] bit1` (`calypso_c54x.c:11945-11955`) : il lit toujours `0x0000`
et **BLOQUE**. Cause : le setter bit1 (~`0xde9c`) est gaté TC sur les cellules API
`0x0314/0x0318`, mais l'ARM n'y écrit que `val=0x0000` à `fn=0` (`HS-ARM-GATE`
`calypso_trx.c:553-566`, la garde `if (… offset == 0x0314 || offset == 0x0318)` est
à `calypso_trx.c:561`) ⇒ `DE-BR data[0x3f70]=0x0001 TC=0`, jamais `0x0002`. Donc
go-live `0xa4c7`/`0xa582` (qui armerait `IMR=0x52fd`) n'est jamais atteint. **C'est LE
gap central** : le handshake go-live ARM→DSP n'assert jamais, notamment parce que le
notify DSP→ARM `api_write_cb` n'est jamais enregistré (`calypso_c54x.c:3357-3358`,
aucune affectation `api_write_cb =`) — l'ARM n'apprend donc jamais la fin du boot DSP.

---

## (E) Vignette : boucle interne corrélateur & `case 0x5` — ✅ FIXÉ

> **✅ FIXÉ (`calypso_c54x.c:8226-8300`, 2026-06-22).** Le bug SFTA/SFTL décrit ci-dessous
> est l'ANCIEN comportement, corrigé. `case 0x5` possède désormais un handler dédié
> dual-long-word DADST/DSADT (encodage `0101 101D`/`0101 111D`, dst=bit8, sémantique
> T-based, post-mod Lmem ±2 via `resolve_lmem`). Commentaire code l.8253 : « Avant :
> 0x50-59/5C-5D tombaient en SFTA/SFTL. » La citation `c54x.c:7817-7852` était FAUSSE :
> elle pointe le handler étendu `0x6F00` (ADD/SUB/STH/STL), sans rapport. Description
> conservée ci-dessous comme HISTORIQUE.

```
Corrélateur FCCH — 1 groupe ×8 taps  (ROM 0xa077..0xa09b)  [HISTORIQUE — comportement d'avant le fix 2026-06-22]
────────────────────────────────────────────────────────────────────────
 PC      op     instr              routage handler            état (aujourd'hui)
 0xa077  5a85   DADST *AR5,A   ──▶  case 0x5 (c54x.c:8226)     [FIXÉ] handler dual-long-word dédié
 0xa078  5f95   DSADT *AR5+,B  ──▶  case 0x5                   [FIXÉ] dst B, T-based, post-mod OK
 0xa079  8e94   CMPS A,*AR4+   ──▶  case 0x8 (hi8 0x8E)        [OK] +resolve_smem
 0xa07a  8f93   CMPS B,*AR3+   ──▶  case 0x8 (hi8 0x8F)        [OK] +resolve_smem
   … ×8 groupes (DADST/DSADT alternent dst A/B) …
 0xa09b  5fd5   DSADT *AR5+0%,B ─▶  case 0x5                   [FIXÉ] Lmem circulaire résolue
────────────────────────────────────────────────────────────────────────
```

```mermaid
flowchart LR
    OP["DADST/DSADT<br/>0x58-0x5F<br/>famille dual long-word"] --> C5["case 0x5<br/>calypso_c54x.c:8226-8300<br/>handler dédié (FIXÉ 2026-06-22)"]
    C5 -->|"dst A/B via bit8<br/>sémantique T-based"| OK1["DADST/DSADT corrects<br/>vérifiés vs SPRU172C"]
    C5 -->|"post-mod Lmem ±2<br/>resolve_lmem"| OK2["circulaire BK résolu"]

    classDef ok fill:#1b5e20,color:#fff,stroke:#0d3010;
    class C5,OK1,OK2 ok;
```

**Légende E — ✅ FIXÉ.** La famille dual long-word DADST/DSADT (opcodes `0x58-0x5F`)
possède désormais un handler dédié dans `case 0x5` (`calypso_c54x.c:8226-8300`, ajouté
le 2026-06-22) : dst A/B sélectionné par bit8, sémantique T-based, post-mod Lmem ±2 via
`resolve_lmem`, branche sur `ST1.C16`. ~~mal-routée … traitée en SFTA/SFTL, `A` décalé,
`B` jamais touché (`dB=0`) ⇒ 0 corrélation~~ — c'était l'ANCIEN comportement (avant le
fix ; cf. sonde SHADOW-DADST run 2026-06-22, `doc/FB_CORRELATOR_PIPELINE.md:46-116`). La
citation `7817-7852` était fausse (elle pointe le handler `0x6F00`). Reste que ce chemin
est de toute façon en AVAL du blocage principal — le corrélateur ne tourne pas ce run
(IMR=0 / DSP déraille), donc `case 0x5` ne se déclenche pas encore.

---

## Résumé des GAP (alignés audit + synthèse)

| # | GAP | Marqueur | Site du correctif | Preuve |
|---|---|---|---|---|
| A | IRQ frame livrée vec19/bit3 au lieu de vec28/bit12 | [GAP] | remap VEC28 `calypso_c54x.c:13549-13565` est un CUL-DE-SAC (déraille) / `calypso_trx.c:1786` | REPORT_...IQ_CABLAGE.md:14 |
| B | IMR=0x0000 tout le run ⇒ INTM=1 jamais levé | [GAP] | atteindre go-live 0xa582 (arme IMR=0x52fd) via le handshake ARM→DSP | qemu.log 0×`INTM=0`, 0×RETE |
| C | Fenêtre I/Q 0x2a00 écrite mais jamais lue | [GAP downstream] | se débloque quand corrélateur tourne | qemu.log 0 hit `2a00` |
| D | Handshake go-live ARM→DSP jamais asserté (val=0x0000) — **GAP CENTRAL** | [GAP faithful] | `calypso_trx.c:553-566` (garde l.561) + notify `api_write_cb` `c54x.c:3357-3358` (jamais enregistré) | qemu.log HS-ARM-GATE |
| E | ~~DADST/DSADT mal-routés `case 0x5`~~ | ✅ FIXÉ (2026-06-22) | handler dédié `calypso_c54x.c:8226-8300` | FB_CORRELATOR_PIPELINE.md:52-68 |

**⚠️ Lever `CALYPSO_DSP_FRAME_VEC28` = CUL-DE-SAC (PAS le correctif).**
~~Une seule vectorisation atteint go-live `0xa582` qui arme `IMR=0x52fd` lui-même —
ensuite tout se vectorise seul.~~ — **FAUX (vérité-terrain 2026-07-01).** Le remap
(`calypso_c54x.c:13549-13565`) retarget vec19→vec28/bit12 et force-vectorise, mais il
atterrit dans l'épilogue ISR → le DSP déraille (POST-BOOTSTUB-RET, PC=0x0000), IMR reste
`0x0000`, `d_fb_det` reste 0. Il n'atteint JAMAIS go-live `0xa582`. C'est une impasse,
pas un levier.

**Vrai correctif — le handshake go-live ARM→DSP.** Le blocage réel est que l'ARM
n'écrit que `0x0000` dans les cellules API `0x0314/0x0318` (`HS-ARM-GATE`
`calypso_trx.c:553-566`) et que le notify DSP→ARM `api_write_cb` n'est jamais enregistré
(`calypso_c54x.c:3357-3358`, aucune affectation `api_write_cb =` dans tout le code) :
l'ARM n'apprend donc jamais la fin de boot DSP et n'assert jamais l'enable non-nul qui
permettrait au DSP d'armer lui-même son IMR via `0xa582`. Câbler ce handshake est le
correctif fidèle — aucun poke IMR/table/`d_fb_det`. (Rappel : il n'existe PAS de
`calypso_orch.c` ni de bus UDP ORCH 6920/6921 ; ne pas les invoquer.)
