/*
 * calypso_c54x.h — TMS320C54x DSP emulator for Calypso
 *
 * Emulates the C54x DSP core found in the TI Calypso baseband chip.
 * Loads ROM dump, executes instructions, shares API RAM with ARM.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CALYPSO_C54X_H
#define CALYPSO_C54X_H

#include <stdint.h>
#include <stdbool.h>

/* Memory sizes (in 16-bit words) */
#define C54X_PROG_SIZE   0x40000  /* 256K words program space */
#define C54X_DATA_SIZE   0x10000  /* 64K words data space */
#define C54X_IO_SIZE     0x10000  /* 64K words I/O space */

/* API RAM: shared between ARM and DSP */
#define C54X_API_BASE    0x0800   /* DSP data address of API RAM */
#define C54X_API_SIZE    0x2000   /* 8K words */

/* DSP start address (after boot) */
#define C54X_DSP_START   0x7000

/* MMR addresses (data memory 0x00-0x1F) */
#define MMR_IMR   0x00
#define MMR_IFR   0x01
#define MMR_ST0   0x06
#define MMR_ST1   0x07
#define MMR_AL    0x08
#define MMR_AH    0x09
#define MMR_AG    0x0A
#define MMR_BL    0x0B
#define MMR_BH    0x0C
#define MMR_BG    0x0D
#define MMR_T     0x0E
#define MMR_TRN   0x0F
#define MMR_AR0   0x10
#define MMR_AR1   0x11
#define MMR_AR2   0x12
#define MMR_AR3   0x13
#define MMR_AR4   0x14
#define MMR_AR5   0x15
#define MMR_AR6   0x16
#define MMR_AR7   0x17
#define MMR_SP    0x18
#define MMR_BK    0x19
#define MMR_BRC   0x1A
#define MMR_RSA   0x1B
#define MMR_REA   0x1C
#define MMR_PMST  0x1D
#define MMR_XPC   0x1E

/* Timer registers (memory-mapped at 0x0024-0x0026) */
#define TIM_ADDR  0x0024   /* Timer counter */
#define PRD_ADDR  0x0025   /* Timer period */
#define TCR_ADDR  0x0026   /* Timer control */

/* TCR bit positions (TMS320C54x hardware spec) */
#define TCR_TDDR_MASK  0x000F   /* bits 3:0 — prescaler reload value */
#define TCR_TSS        (1 << 4) /* bit 4 — Timer Stop Status (1=stopped) */
#define TCR_TRB        (1 << 5) /* bit 5 — Timer Reload (write 1 reloads) */
#define TCR_PSC_SHIFT  6        /* bits 9:6 — prescale counter */
#define TCR_PSC_MASK   (0xF << TCR_PSC_SHIFT)
#define TCR_SOFT       (1 << 10)
#define TCR_FREE       (1 << 11)

/* ST0 bit positions */
#define ST0_DP_MASK  0x01FF  /* bits 8-0: data page pointer */
#define ST0_OVB      (1 << 9)
#define ST0_OVA      (1 << 10)
#define ST0_C        (1 << 11)
#define ST0_TC       (1 << 12)
#define ST0_ARP_SHIFT 13
#define ST0_ARP_MASK (7 << ST0_ARP_SHIFT)

/* ST1 bit positions */
#define ST1_ASM_MASK 0x001F  /* bits 4-0: accumulator shift mode */
#define ST1_CMPT     (1 << 5)
#define ST1_FRCT     (1 << 6)
#define ST1_C16      (1 << 7)
#define ST1_SXM      (1 << 8)
#define ST1_OVM      (1 << 9)
#define ST1_INTM     (1 << 11)
#define ST1_HM       (1 << 12)
#define ST1_XF       (1 << 13)
#define ST1_BRAF     (1 << 14)

/* PMST bit positions (per SPRU131: SST=0 SMUL=1 CLKOFF=2 DROM=3 APTS=4 OVLY=5 MP/MC=6) */
#define PMST_SST     (1 << 0)
#define PMST_SMUL    (1 << 1)
#define PMST_CLKOFF  (1 << 2)
#define PMST_DROM    (1 << 3)
#define PMST_APTS    (1 << 4)
#define PMST_OVLY    (1 << 5)
#define PMST_MP_MC   (1 << 6)
#define PMST_IPTR_SHIFT 7
#define PMST_IPTR_MASK (0x1FF << PMST_IPTR_SHIFT)

/* Interrupt vectors */
#define C54X_INT_RESET   0
#define C54X_INT_NMI     1
/* ============================================================================
 * TABLE DES INTERRUPTIONS DU DSP CALYPSO — SOURCE : CAL000 §5.1 (ti-calypso1.pdf,
 * « DSP INTERRUPTS », p.24). Ver 1.3, HERCROM400G2.
 *
 * [2026-08-03] CE BLOC REMPLACE LA TABLE SPRU131 QUI ÉTAIT ICI. C'ÉTAIT L'ERREUR
 * SOURCE DE TOUTE LA CASCADE : SPRU131 décrit le TMS320C54x GÉNÉRIQUE, alors que
 * le sous-chip DSP du Calypso (S28C128) a son propre mapping de périphériques.
 * Les deux tables divergent à partir du bit 3 : le C54x générique a QUATRE lignes
 * externes (INT0..INT3) avant TINT, le Calypso n'en a que TROIS (INT0n..INT2n).
 * D'où un décalage de 1 sur tout le reste, et des noms faux (BRINT0/BXINT0/DMAC0
 * n'existent pas sur Calypso).
 *
 * §5.1 : « The DSP subchip owns 17 interrupt lines with 11 of which INT0n to
 * INT10n are dedicated for external peripherals. »
 *
 * [2026-08-03, DEUXIÈME CORRECTION] La première version de ce bloc suivait l'ORDRE
 * DE LA LISTE EN PROSE de CAL000 §5.1, faute de mieux. Cette liste est FAUSSE à
 * partir du bit 6 : elle plaçait AINT en bit 12. La table qui fait autorité est
 * CAL207 §15.1, « DSP interrupts Mapping », qui donne l'EMPLACEMENT EN HEXA de
 * chaque vecteur — pas un ordre à interpréter. vec = Location / 4.
 *
 *  bit  vec  Loc.  ligne     source (CAL207 §15.1)              sens
 *  ---  ---  ----  --------  ---------------------------------  ------
 *   0    16  0x40  INT0n     RIF receive interrupt              niveau
 *   1    17  0x44  INT1n     RIF transmit interrupt             niveau
 *   2    18  0x48  INT2n     UART interrupt                     niveau
 *   3    19  0x4C  TINT      Timer interrupts
 *   4    20  0x50  RINT      SPI receive interrupt
 *   5    21  0x54  XINT      SPI transmit interrupt
 *   6    22  0x58  INT4n     MCSI transmit interrupt            niveau
 *   7    23  0x5C  INT5n     MCSI frame duration error          niveau
 *   8    24  0x60  INT3n     MCSI receive interrupt             niveau
 *   9    25  0x64  AINT      API interrupts (ARM ↔ DSP)
 *  10    26  0x68  INT6n     MCSI DAI interrupt                 niveau
 *  11    27  0x6C  INT7n     CYPHER interrupts                  FRONT
 *  12    28  0x70  INT8n     TPU FRAME interrupt                FRONT
 *  13    29  0x74  INT9n     TPU programmable interrupt         FRONT
 *  14    30  0x78  INT10n    DMA interrupt                      niveau
 *        1   0x04  nMIN      Abort on Rhea bus OR redirection INT4n (= NMI)
 *
 * CE QUI CHANGE PAR RAPPORT À LA LISTE EN PROSE : INT3n/INT4n/INT5n sont permutés,
 * **AINT est en bit 9 (pas 12)**, INT6n en bit 10, INT7n en bit 11, et l'IT TRAME
 * du TPU (INT8n) est en **bit 12 / vec 28**.
 *
 * VÉRIFIÉ PAR LA MESURE, et c'est ce qui rend cette table crédible : l'IMR du ROM
 * vaut 0x52ef = bits 0,1,2,3,5,6,7,9,12,14, soit RIF rx + RIF tx + UART + timer +
 * SPI tx + MCSI tx/err + **AINT** + **IT trame TPU** + DMA. Les bits laissés
 * masqués sont CYPHER (11), MCSI rx (8), TPU programmable (13) et SPI rx (4) —
 * exactement ce qu'un L1 GSM n'utilise pas à ce stade. Sous l'ancienne lecture, le
 * bit 11 « IT trame » n'était JAMAIS ouvert alors qu'osmocom ne signale que par
 * lui : l'incohérence venait de la table, pas du firmware.
 *
 * CONFIRMATION CROISÉE (§15.2.1) : le firmware écrit 0x0380 dans CNTRL_REG
 * (XIO:FA00), qui assigne edge/niveau par canal — bits 7, 8, 9 → canaux 7, 8, 9
 * en FRONT. Or §15.1 marque exactement INT7n, INT8n et INT9n comme « edge ». Le
 * canal N est donc bien INTNn.
 *
 * Formule inchangée et confirmée : vec = imr_bit + 16.
 * Note §5.1 sur INT9n : « a facility offered to the DSP programmer in order to
 * allow the generation of a DSP interrupt at a dedicated time with a quarter of
 * GSM bit accuracy. The interrupt is set in a scenario by using a time-stamped
 * instruction. » → c'est exactement le MOVE TPUI_DSP_INT_PG du séquenceur TPU.
 *
 * RECOUPEMENTS DE MESURE (ce qui rend la table crédible, pas seulement lue) :
 *   • vec21 = XINT/SPI TX → périphérique inutilisé en L1 GSM, et le ROM y a bien
 *     un stub RETE ; idem vec20 = RINT/SPI RX.
 *   • vec28 = AINT et vec30 = INT10n/DMA : deux slots déjà mesurés dans le ROM.
 *   • IMR mesurée 0x52ed → bits 0,2,3,5,6,7,9,12,14 = RIF RX, UART, TINT, SPI TX,
 *     MCSI RX/TX, MCSI DAI, AINT, DMA. Un L1 GSM plausible.
 *   • le ROM fait IMR |= 0x3000 = bits 12+13 = AINT + TPU programmable.
 * ==========================================================================*/
#define C54X_IT_RIF_RX_VEC     16   /* INT0n  RIF receive              */
#define C54X_IT_RIF_RX_BIT      0
#define C54X_IT_RIF_TX_VEC     17   /* INT1n  RIF transmit             */
#define C54X_IT_RIF_TX_BIT      1
#define C54X_IT_UART_VEC       18   /* INT2n  UART                     */
#define C54X_IT_UART_BIT        2
#define C54X_IT_TINT_VEC       19   /* TINT   timer DSP                */
#define C54X_IT_TINT_BIT        3
#define C54X_IT_SPI_RX_VEC     20   /* RINT   SPI receive              */
#define C54X_IT_SPI_RX_BIT      4
#define C54X_IT_SPI_TX_VEC     21   /* XINT   SPI transmit             */
#define C54X_IT_SPI_TX_BIT      5
#define C54X_IT_MCSI_TX_VEC    22   /* INT4n  MCSI transmit     (0x58) */
#define C54X_IT_MCSI_TX_BIT     6
#define C54X_IT_MCSI_ERR_VEC   23   /* INT5n  MCSI frame dur.   (0x5C) */
#define C54X_IT_MCSI_ERR_BIT    7
#define C54X_IT_MCSI_RX_VEC    24   /* INT3n  MCSI receive      (0x60) */
#define C54X_IT_MCSI_RX_BIT     8
#define C54X_IT_API_VEC        25   /* AINT   API (ARM ↔ DSP)   (0x64) */
#define C54X_IT_API_BIT         9
#define C54X_IT_MCSI_DAI_VEC   26   /* INT6n  MCSI DAI          (0x68) */
#define C54X_IT_MCSI_DAI_BIT   10
#define C54X_IT_CRYPT_VEC      27   /* INT7n  CYPHER            (0x6C) */
#define C54X_IT_CRYPT_BIT      11
#define C54X_IT_TPU_FRAME_VEC  28   /* INT8n  TPU frame         (0x70) */
#define C54X_IT_TPU_FRAME_BIT  12
#define C54X_IT_TPU_PROG_VEC   29   /* INT9n  TPU programmable  (0x74) */
#define C54X_IT_TPU_PROG_BIT   13
#define C54X_IT_DMA_VEC        30   /* INT10n DMA               (0x78) */
#define C54X_IT_DMA_BIT        14

/* SAS — CALYPSO_IT_TABLE_DOC=1 : bascule les émetteurs d'IT encore câblés sur
 * l'ancienne table SPRU131 vers la table §5.1 ci-dessus. Défaut 0 : comportement
 * strictement inchangé, parce que ces émetteurs-là sont sur le chemin du shunt qui
 * campe. Protocole : tester SOUS CHARGE (camp + LU + SMS), puis effacer LA
 * CONDITION — pas le correctif. */

/* ⚠ LEGACY — valeur MAL MAPPÉE, conservée le temps du sas ci-dessus.
 * L'ancien commentaire disait « bit 3 = INT3 = la ligne frame-sync du TPU » :
 * FAUX deux fois. Le bit 3 est TINT (le timer du DSP), et l'IT trame du TPU est
 * INT8n = bit 11 = vec 27 (C54X_IT_TPU_FRAME_*). L'IMR 0xFF88 citée à l'appui
 * n'est plus celle qu'on mesure (0x52ed). Utiliser C54X_IT_TPU_FRAME_* pour tout
 * nouveau câblage. */
#define C54X_INT_FRAME_VEC   19  /* LEGACY : en réalité TINT, pas l'IT trame  */
#define C54X_INT_FRAME_BIT   3   /* LEGACY : voir C54X_IT_TPU_FRAME_BIT = 11 */
#define C54X_NUM_INTS        16

typedef struct C54xState {
    /* Accumulators (40-bit) stored as int64 for convenience */
    int64_t a;   /* A accumulator: bits 39-0 */
    int64_t b;   /* B accumulator: bits 39-0 */

    /* Auxiliary registers */
    uint16_t ar[8];

    /* Other registers */
    uint16_t t;      /* Temporary register */
    uint16_t trn;    /* Transition register (Viterbi) */
    uint16_t sp;
    uint16_t bk;     /* Circular buffer size */
    uint16_t brc;    /* Block repeat counter */
    uint16_t rsa;    /* Block repeat start address */
    uint16_t rea;    /* Block repeat end address */

    /* Status registers */
    uint16_t st0;
    uint16_t st1;
    uint16_t pmst;

    /* Interrupt registers */
    uint16_t imr;
    uint16_t ifr;

    /* Optional reset-state override loaded from calypso_dsp.Registers.bin via
     * `-M calypso,dsp-registers=<path>` (default-wired by run.sh, like the
     * other ROM sections). reg_init[i] = value for MMR index i (0x00..0x1F).
     * When reg_init_valid, c54x_reset() applies these AFTER its silicon
     * hardcode defaults, so the .bin snapshot is authoritative. */
    uint16_t reg_init[0x20];
    bool     reg_init_valid;

    /* Program counter */
    uint32_t pc;     /* 16-bit (or 23-bit with XPC) */
    uint16_t xpc;

    /* Timer0 prescale counter (PSC) — not memory-mapped directly */
    uint16_t timer_psc;

    /* DMA sub-register bank (6 channels × 4 regs) */
    uint16_t dma_subaddr;
    uint16_t dma_subregs[24];
    /* McBSP sub-register bank */
    uint16_t spsa;

    /* RPT state */
    uint16_t rpt_count;  /* remaining RPT iterations */
    uint16_t rpt_pc;     /* PC of repeated instruction */
    bool     rpt_active;
    bool     rpt_fresh;   /* RPT vient d'etre arme : 1ere lecture READA/MVPD repart de la base, pas du mvpd_src stale (fix 2026-06-24) */
    uint16_t par;        /* Program Address Register (for READA/WRITA/MACD/MACP) */
    bool     par_set;
    bool     lk_used;    /* resolve_smem consumed extra word for lk */
    uint16_t mvpd_src;   /* MVPD auto-increment source address during RPT */

    /* RPTB state */
    bool     rptb_active;

    /* Delayed-branch state (CALLD/RETD/BD/CCD/...): when set, the next
     * `delay_slots` instructions execute normally, then PC is forced to
     * `delayed_pc`. */
    uint16_t delayed_pc;
    uint8_t  delay_slots;

    /* Memory */
    uint16_t prog[C54X_PROG_SIZE];   /* Program memory */
    uint16_t data[C54X_DATA_SIZE];   /* Data memory */

    /* API RAM pointer (shared with ARM calypso_trx.c) */
    uint16_t *api_ram;  /* points into ARM's dsp_ram[] */

    /* DSP → ARM notify hook: called whenever the DSP writes to api_ram. */
    void (*api_write_cb)(void *opaque, uint16_t woff, uint16_t val);
    void  *api_write_cb_opaque;

    /* State */
    bool     running;
    bool     idle;       /* IDLE instruction executed */
    bool     blob_loaded; /* Test fixture: set by c54x_set_initial_pc().
                           * Suppresses the secondary c54x_reset() that
                           * normally fires when ARM writes DSP_DL_STATUS_READY,
                           * which would otherwise clobber the user's blob
                           * via the reset-time PROM→DARAM auto-copy. */
    uint64_t cycles;
    uint32_t insn_count;

    /* BSP (Baseband Serial Port) — burst sample buffer */
    uint16_t bsp_buf[2048]; /* burst I/Q samples from radio */
    int      bsp_len;       /* number of samples */
    int      bsp_pos;       /* read position */

    /* Debug */
    uint32_t unimpl_count;
    uint16_t last_unimpl;
    /* Last executed instruction snapshot — captured at end of each
     * c54x_run iteration. Used by the INTM-TRANS tracer (and others)
     * to attribute post-instruction state changes to the actual cause
     * PC/opcode rather than the post-advance PC. */
    uint16_t last_exec_pc;
    uint16_t last_exec_op;

    /* writer_kind : set by each opcode handler / external writer before
     * calling data_write. Logged in DATA-W-MMR trace to disambiguate
     * which path is responsible for stray writes to MMR (addr<=0x1F).
     * Reset to WK_UNKNOWN at the top of c54x_exec_one. */
    uint8_t  writer_kind;
} C54xState;

/* writer_kind enum — keep small, extend as needed */
enum {
    WK_UNKNOWN     = 0,
    WK_OPCODE_F3   = 1,   /* 0xF3xx family (SFTL/AND/OR/XOR/INTR/etc.) */
    WK_OPCODE_8x   = 2,   /* 0x80xx-0x8Fxx (STL/STH/STLM/STM/LD-Smem) */
    WK_OPCODE_77   = 3,   /* 0x77xx STM #lk, MMR */
    WK_OPCODE_76   = 4,   /* 0x76xx ST #lk, Smem */
    WK_OPCODE_PSHM = 5,   /* PSHM/POPM stack ops */
    WK_OPCODE_RET  = 6,   /* RET/RETI/RETD frame restore */
    WK_IRQ_ACK     = 7,   /* IRQ acknowledge / vector dispatch */
    WK_ARM_MMIO    = 8,   /* ARM-side write through shared region */
    WK_RESOLVE_AR  = 9,   /* resolve_smem AR-modify side effect */
    WK_OPCODE_OTHER= 10,  /* anything else inside an opcode handler */
};

/* Feed burst samples to BSP (called by calypso_trx) */
void c54x_bsp_load(C54xState *s, const uint16_t *samples, int n);

/* Create and initialize C54x state */
C54xState *c54x_init(void);

/* Link API RAM (shared memory with ARM) */
void c54x_set_api_ram(C54xState *s, uint16_t *api_ram);

/* Reset the DSP */
void c54x_reset(C54xState *s);

/* Execute N instructions (returns actual count executed) */
int c54x_run(C54xState *s, int n_insns);

/* Raise an interrupt */
/* Send interrupt: vec = vector number (for PC), imr_bit = bit in IMR/IFR */
void c54x_interrupt_ex(C54xState *s, int vec, int imr_bit);

/* Wake from IDLE */
void c54x_wake(C54xState *s);

/* Test fixture: override PC after reset.
 * Used by `-M calypso,dsp-blob=<path>` to start execution at a custom
 * address instead of the silicon-default reset vector (IPTR * 0x80). */
void c54x_set_initial_pc(C54xState *s, uint32_t pc);

/* Test fixture: load a raw binary blob into DARAM starting at daram_addr.
 * File bytes are read pairwise as little-endian DSP words.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_blob_daram(C54xState *s, const char *path, uint16_t daram_addr);

/* Explicit per-section ROM load: write raw LE 16-bit words from `path`
 * into either prog[] (when is_program=true) or data[] (when false),
 * starting at DSP word address `start_addr`. Used by the per-section
 * machine properties (dsp-prom0/prom1/prom2/prom3/drom/pdrom) to load
 * each ROM section at its silicon-correct DSP address.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_section(C54xState *s, const char *path,
                       uint32_t start_addr, bool is_program);

/* Load the DSP register snapshot (calypso_dsp.Registers.bin: raw LE 16-bit
 * words, MMR page 0x00..0x1F first) into reg_init[] so c54x_reset() applies
 * it as the reset state. Words >= 0x20 are written into data[] (low scratch).
 * Used by the `-M calypso,dsp-registers=<path>` machine property.
 * Returns number of words loaded, or -1 on error. */
int  c54x_load_registers(C54xState *s, const char *path);

#endif /* CALYPSO_C54X_H */
