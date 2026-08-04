/*
 * calypso_rif.c — Radio InterFace (RIF) du Calypso, vue DSP (espace XIO).
 *
 * POURQUOI CE FICHIER EXISTE (2026-08-03).
 * ----------------------------------------
 * Mesure sur un run native_twl : le DSP prend bien son interruption de reception
 * (`PC=0x00c0`, 16 330 entrees), puis n'en sort rien — `fb0_att=37`, `A_CD-WR=0`.
 * ([2026-08-03] `fb0_ret=0` etait aussi cite ici : compteur MORT, jamais
 * incremente, retire — il n'etayait rien.)
 * La sonde PORTR-ANY donne la raison, et elle est unanime :
 *
 *     PORTR-ANY PA=0x0003 addr=0x000e PC=0xa675      <- 30 releves sur 30
 *
 * TOUTES les instructions PORTR du firmware DSP lisent `XIO:0003`. Or `PORTR`
 * etait un NO-OP dans le modele : hors `PA=0xF430/0x0034` sous le gate
 * CALYPSO_FIX_PORTR (defaut 0, et le firmware n'utilise jamais ces PA-la), le
 * code faisait `(void)pa; (void)addr;` — il n'ecrivait meme pas la destination.
 * Le handler testait donc un mot perime et repartait. Le RIF n'etait pas
 * modelise du tout cote DSP.
 *
 * CE QUE DIT LE SILICIUM (CAL207 §12.1, Table 19) :
 *
 *   Registre  XIO      ARM         Acces      Reset
 *   DXR       0x0000   FFFF:7000   16b R/W    indefini
 *   DRR       0x0001   FFFF:7002   16b R      indefini
 *   SPCX      0x0002   n.a.        15b        000 0101 1001 1110 = 0x059E
 *   SPCR      0x0003   n.a.        15b        011 1100 1010 0010 = 0x3CA2
 *
 * SPCR (§12.6) — la valeur de reset annoncee a ete reverifiee champ par champ
 * et tombe exactement sur 0x3CA2, ce qui valide la lecture :
 *
 *   0  DLB          0   boucle locale numerique
 *   1  RRST         1   reset du recepteur (0 -> efface RSRFULL et RRDY)
 *   2  RRDY         0   « Unused » selon §12.6 — mais §12.3 dit « DRR is updated
 *                       when RSR is ready to be read and RRDY=1 ». Le document se
 *                       contredit ; cf. CALYPSO_RIF_RRDY_UNUSED plus bas.
 *   3  RSRFULL      0   un mot est dans RSR et DRR n'a pas encore ete lu.
 *                       NOTE du doc : PAS remis a zero par une lecture de DRR —
 *                       « This allows keeping trace of a reception problem ».
 *   4  ALMOST_FULL  0   1 = FIFO de reception non vide (au seuil THRESHOLD)
 *   5  FIFO_EMPTY   1   FIFO vide. Le texte du doc dit « 0 = receive FIFO is
 *                       empty », mais la valeur de reset est 1 ET la FIFO est
 *                       vide au reset : la prose est inversee, on suit le reset.
 *   6  FIFO_FULL    0   FIFO pleine
 *   9:7 THRESHOLD 001   seuil du drapeau « FIFO non vide », maximum 4
 *   10 XINT_MASK    1   1 = interruption d'emission masquee
 *   11 RINT_MASK    1   1 = interruption de RECEPTION masquee
 *   12 XDMA_MASK    1   1 = requete DMA d'emission masquee
 *   13 RDMA_MASK    1   1 = requete DMA de RECEPTION masquee
 *   14 CLKLB        0   horloges TX/RX issues de la meme source interne
 *
 * CE QUE CA REGLE, ET QU'ON N'A DONC PLUS A DEVINER.
 * Le §3.7.1 dit que le DSP echange avec le RIF « soit par XIO (mot a mot, une
 * interruption est envoyee au DSP), soit par l'API en mode DMA (une requete DMA
 * et une requete end-DMA sont envoyees a l'ARM) ». Lequel des deux ? Ce n'est pas
 * a nous d'en decider : ce sont RINT_MASK (bit 11) et RDMA_MASK (bit 13) qui le
 * disent, et c'est le FIRMWARE qui les programme. Les deux sont masques au reset —
 * quelqu'un doit les ouvrir. On respecte ce choix ici au lieu de le decreter par
 * un gate, ce que faisaient CALYPSO_BSP_VEC30 / CALYPSO_BSP_RX_VEC.
 *
 * Vecteur d'interruption de reception : INT0n = IMR bit 0 = vec 16 (CAL000 §5.1).
 * L'IMR mesuree sur ce firmware (0x52ef) a bien le bit 0 ouvert.
 *
 * CE QUI N'EST PAS MODELISE ICI, ET ASSUME :
 *   - l'alias ARM FFFF:7000 / FFFF:7002 (DXR/DRR vus du MCU) : le firmware
 *     osmocom n'y touche pas, on ne le cable pas.
 *   - l'emission (DXR/SPCX) : les ecritures sont acceptees et comptees, rien
 *     n'est transmis. Le TX passe aujourd'hui par le TPU/TSP.
 *   - XSR/RSR ne sont pas accessibles (§12.4) : on ne les expose pas.
 *   - la profondeur exacte de la FIFO n'est pas donnee par le doc ; THRESHOLD
 *     etant plafonne a 4, on prend 4. Le burst complet attend dans un tampon
 *     d'etage et alimente la FIFO au fil des lectures de DRR.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_c54x.h"
#include "hw/arm/calypso/calypso_rif.h"
#include "hw/arm/calypso/calypso_rhea_dma.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- champs de SPCR (§12.6) --- */
#define SPCR_DLB          (1u << 0)
#define SPCR_RRST         (1u << 1)
#define SPCR_RRDY         (1u << 2)
#define SPCR_RSRFULL      (1u << 3)
#define SPCR_ALMOST_FULL  (1u << 4)
#define SPCR_FIFO_EMPTY   (1u << 5)
#define SPCR_FIFO_FULL    (1u << 6)
#define SPCR_THRESHOLD_SH 7
#define SPCR_THRESHOLD_M  (7u << SPCR_THRESHOLD_SH)
#define SPCR_XINT_MASK    (1u << 10)
#define SPCR_RINT_MASK    (1u << 11)
#define SPCR_XDMA_MASK    (1u << 12)
#define SPCR_RDMA_MASK    (1u << 13)
#define SPCR_CLKLB        (1u << 14)

/* bits R/W conserves tels quels a l'ecriture ; le reste est de l'etat materiel
 * recalcule a la lecture. */
#define SPCR_RW_MASK  (SPCR_DLB | SPCR_RRST | SPCR_THRESHOLD_M | SPCR_XINT_MASK \
                       | SPCR_RINT_MASK | SPCR_XDMA_MASK | SPCR_RDMA_MASK | SPCR_CLKLB)

#define SPCR_RESET   0x3CA2
#define SPCX_RESET   0x059E

#define RIF_FIFO_DEPTH  4        /* THRESHOLD plafonne a 4 (§12.6) */
#define RIF_STAGE_MAX   2048     /* meme borne que bsp_buf */

static struct {
    bool     init;
    uint16_t spcr;               /* champs R/W uniquement */
    uint16_t spcx;
    uint16_t dxr;

    uint16_t fifo[RIF_FIFO_DEPTH];
    int      fifo_n;

    uint16_t stage[RIF_STAGE_MAX];   /* le burst en attente d'entrer en FIFO */
    int      stage_n;
    int      stage_pos;

    bool     rsrfull;            /* mot shifte alors que DRR pas encore lu */
    uint16_t drr;                /* dernier mot sorti de la FIFO */
    bool     drr_valid;

    unsigned n_burst, n_drr_rd, n_spcr_rd, n_spcr_wr, n_int, n_dma, n_overrun;

    /* [2026-08-04] BEQUILLE RIF_BACKPRESSURE : bursts refuses faute de place,
     * et bursts imposes par la soupape anti-blocage. `bp_run` = refus
     * consecutifs en cours. */
    unsigned n_bp_skip, n_bp_force, bp_run;
} rif;

bool calypso_rif_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RIF_XIO", 1);
        if (!on)
            fprintf(stderr, "[rif] CALYPSO_RIF_XIO=0 : RIF non modelise, PORTR/PORTW "
                    "redeviennent des no-op (comportement d'avant le 03/08)\n");
    }
    return on != 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * @BEQUILLE — RIF_BACKPRESSURE  (CALYPSO_RIF_BACKPRESSURE, defaut 0)
 *
 *   (1) C'EST UNE BEQUILLE. Le silicium n'a pas de contre-pression : quand un
 *       burst arrive et que le precedent n'est pas lu, le recepteur ECRASE et
 *       pose RSRFULL (§12.6). Refuser la livraison est une DEVIATION du
 *       materiel, pas une correction de modele.
 *
 *   (2) CE QU'ELLE MASQUE : l'interpreteur c54x est ~96x trop lent
 *       (cf. natif-mur-vitesse-interpreteur-c54x). Mesure du 04/08 sur un run
 *       native_twl : 73 500 bursts pousses pour 54 492 overruns — 74 % des
 *       bursts ecrasent un predecesseur non consomme, et le DSP ne draine que
 *       ~4 mots (RIF_FIFO_DEPTH) sur les 296 avant l'ecrasement suivant. Le
 *       DMA transporte donc des zeros, `a_cd` est publie vide, et un SI ne
 *       sort que quand le hasard aligne remplissage/drainage/RRST. Tant que
 *       cette gate est a 1, on ne mesure PLUS le vrai debit du DSP : on
 *       observe un banc ralenti a sa main.
 *
 *   (3) QUAND LA RETIRER : des que le DSP consomme un burst entier dans le
 *       temps d'une trame — c'est-a-dire quand l'interpreteur est assez
 *       rapide (JIT, ou budget par trame revu). Juge : `overrun` doit tomber
 *       proche de 0 avec la gate a 0. Si `n_bp_force` est non nul, le DSP ne
 *       consomme toujours pas et la soupape a du forcer : la bequille ne
 *       suffit pas, ce n'est pas un progres.
 *
 * PORTEE RESTREINTE (corrige le 04/08 apres mesure) : la contre-pression ne
 * s'applique QUE si un chemin de drainage est arme (RDMA ou RINT demasque).
 * Recepteur non arme = rien ne videra jamais l'etage : refuser reviendrait a
 * jeter des bursts frais pour proteger un burst mort. Dans ce cas on ecrase,
 * comme le materiel.
 *
 * SOUPAPE ANTI-BLOCAGE : sans elle, un DSP qui cesse de lire figerait la
 * reception pour toujours. Apres CALYPSO_RIF_BP_MAX_SKIP refus consecutifs
 * (defaut 200), on impose le burst suivant et on le journalise.
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool rif_backpressure_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RIF_BACKPRESSURE", 0);
        fprintf(stderr, "[rif] BEQUILLE contre-pression %s "
                "(CALYPSO_RIF_BACKPRESSURE=%d) — %s\n",
                on ? "ACTIVE" : "inactive", on,
                on ? "un burst n'ecrase plus un predecesseur non lu ; "
                     "DEVIATION du §12.6, le debit mesure n'est plus celui du DSP"
                   : "comportement materiel conserve : ecrasement + RSRFULL (defaut)");
    }
    return on != 0;
}

/* Valeur NUMERIQUE : `calypso_gate` ne rend que 0/1, il ne convient pas ici.
 * Idiome getenv+atoi, comme les autres seuils du modele. */
static unsigned rif_bp_max_skip(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_RIF_BP_MAX_SKIP");
        v = e ? atoi(e) : 200;
        if (v <= 0)
            v = 200;   /* une soupape a 0 serait un blocage garanti */
    }
    return (unsigned)v;
}

/* §12.6 note bit2 « Unused », §12.3 s'en sert. On expose RRDY = « une donnee est
 * disponible », qui est la seule lecture qui rende le §12.3 coherent, et on laisse
 * un interrupteur pour revenir a la lettre du §12.6 si un run le demande. */
static bool rif_rrdy_unused(void)
{
    static int u = -1;
    if (u < 0) u = calypso_gate("CALYPSO_RIF_RRDY_UNUSED", 0);
    return u != 0;
}

static void rif_init(void)
{
    if (rif.init)
        return;
    rif.init = true;
    rif.spcr = SPCR_RESET & SPCR_RW_MASK;
    rif.spcx = SPCX_RESET;
    fprintf(stderr, "[rif] RIF XIO arme (CAL207 §12) : DXR@0x0000 DRR@0x0001 "
            "SPCX@0x0002=0x%04x SPCR@0x0003=0x%04x ; RX IT = INT0n vec16/bit0, "
            "choisie par SPCR.RINT_MASK (bit11) / RDMA_MASK (bit13), tous deux "
            "MASQUES au reset — c'est au firmware de les ouvrir.\n",
            SPCX_RESET, SPCR_RESET);
}

static int rif_threshold(void)
{
    int t = (rif.spcr & SPCR_THRESHOLD_M) >> SPCR_THRESHOLD_SH;
    if (t < 1) t = 1;
    if (t > RIF_FIFO_DEPTH) t = RIF_FIFO_DEPTH;
    return t;
}

/* Fait couler le tampon d'etage vers la FIFO tant qu'il reste de la place. */
static void rif_refill(void)
{
    while (rif.fifo_n < RIF_FIFO_DEPTH && rif.stage_pos < rif.stage_n)
        rif.fifo[rif.fifo_n++] = rif.stage[rif.stage_pos++];
}

/* Etat materiel recompose a chaque lecture de SPCR. */
static uint16_t rif_spcr_read(void)
{
    uint16_t v = rif.spcr & SPCR_RW_MASK;
    if (rif.fifo_n == 0)
        v |= SPCR_FIFO_EMPTY;
    if (rif.fifo_n >= rif_threshold())
        v |= SPCR_ALMOST_FULL;
    if (rif.fifo_n >= RIF_FIFO_DEPTH)
        v |= SPCR_FIFO_FULL;
    if (rif.rsrfull)
        v |= SPCR_RSRFULL;
    if (!rif_rrdy_unused() && rif.fifo_n > 0)
        v |= SPCR_RRDY;
    return v;
}

bool calypso_rif_portr(C54xState *s, uint16_t pa, uint16_t *out)
{
    (void)s;
    if (!calypso_rif_on())
        return false;
    rif_init();

    switch (pa) {
    case RIF_XIO_DRR:
        rif_refill();
        if (rif.fifo_n > 0) {
            rif.drr = rif.fifo[0];
            memmove(&rif.fifo[0], &rif.fifo[1],
                    (size_t)(rif.fifo_n - 1) * sizeof(uint16_t));
            rif.fifo_n--;
            rif.drr_valid = true;
            rif_refill();
        }
        /* Si la FIFO est vide, DRR conserve sa derniere valeur : le doc ne
         * definit pas de valeur de lecture a vide (« Undefined » au reset), et
         * inventer un 0 fabriquerait un echantillon. */
        *out = rif.drr;
        if (rif.n_drr_rd++ < 20)
            fprintf(stderr, "[rif] DRR read #%u = 0x%04x (fifo=%d etage=%d/%d) "
                    "PC=0x%04x\n", rif.n_drr_rd, *out, rif.fifo_n,
                    rif.stage_pos, rif.stage_n, s ? s->pc : 0);
        return true;

    case RIF_XIO_SPCR:
        *out = rif_spcr_read();
        if (rif.n_spcr_rd++ < 20)
            fprintf(stderr, "[rif] SPCR read #%u = 0x%04x (fifo=%d empty=%d "
                    "almost=%d rsrfull=%d RINT_MASK=%d RDMA_MASK=%d) PC=0x%04x\n",
                    rif.n_spcr_rd, *out, rif.fifo_n,
                    !!(*out & SPCR_FIFO_EMPTY), !!(*out & SPCR_ALMOST_FULL),
                    !!(*out & SPCR_RSRFULL), !!(*out & SPCR_RINT_MASK),
                    !!(*out & SPCR_RDMA_MASK), s ? s->pc : 0);
        return true;

    case RIF_XIO_SPCX:
        *out = rif.spcx;
        return true;

    case RIF_XIO_DXR:
        *out = rif.dxr;   /* R/W selon §12.2 */
        return true;

    default:
        return false;
    }
}

bool calypso_rif_portw(C54xState *s, uint16_t pa, uint16_t val)
{
    if (!calypso_rif_on())
        return false;
    rif_init();

    switch (pa) {
    case RIF_XIO_SPCR: {
        uint16_t before = rif.spcr;
        rif.spcr = val & SPCR_RW_MASK;
        /* §12.6 : « Writing a zero to RRST clears the RSRFULL bit and RRDY bit ».
         * On vide aussi la FIFO : le recepteur est en reset. */
        if ((before & SPCR_RRST) && !(val & SPCR_RRST)) {
            rif.rsrfull = false;
            rif.fifo_n = 0;
            rif.stage_pos = rif.stage_n;
            rif.drr_valid = false;
        }
        if (rif.n_spcr_wr++ < 20)
            fprintf(stderr, "[rif] SPCR write #%u 0x%04x -> RRST=%d THRESHOLD=%d "
                    "RINT_MASK=%d RDMA_MASK=%d XINT_MASK=%d PC=0x%04x\n",
                    rif.n_spcr_wr, val, !!(val & SPCR_RRST), rif_threshold(),
                    !!(val & SPCR_RINT_MASK), !!(val & SPCR_RDMA_MASK),
                    !!(val & SPCR_XINT_MASK), s ? s->pc : 0);
        return true;
    }
    case RIF_XIO_SPCX:
        rif.spcx = val;
        return true;

    case RIF_XIO_DXR:
        /* Emission non modelisee (cf. en-tete) : on accepte et on compte. */
        rif.dxr = val;
        return true;

    case RIF_XIO_DRR:
        /* §12.3 : « DRR cannot be written via the RHEA interface. » */
        return true;

    default:
        return false;
    }
}

int calypso_rif_drain(uint16_t *dst, int max)
{
    if (!calypso_rif_on() || !dst || max <= 0)
        return 0;
    rif_init();

    int got = 0;
    while (got < max) {
        if (rif.fifo_n == 0) {
            rif_refill();          /* meme realimentation que la lecture de DRR */
            if (rif.fifo_n == 0)
                break;             /* recepteur vide : on rend ce qu'on a */
        }
        dst[got++] = rif.fifo[0];
        memmove(&rif.fifo[0], &rif.fifo[1],
                (size_t)(rif.fifo_n - 1) * sizeof(uint16_t));
        rif.fifo_n--;
    }
    /* Le recepteur s'est vide : RSRFULL n'a plus lieu d'etre (§12.6). */
    if (got && rif.fifo_n == 0 && rif.stage_pos >= rif.stage_n)
        rif.rsrfull = false;
    return got;
}

void calypso_rif_rx_burst(C54xState *s, const uint16_t *w, int n)
{
    if (!calypso_rif_on() || !w || n <= 0)
        return;
    rif_init();

    if (n > RIF_STAGE_MAX)
        n = RIF_STAGE_MAX;

    /* Le burst precedent n'a pas ete entierement consomme : c'est un debordement
     * reel du recepteur, on le compte au lieu de le taire. */
    if (rif.stage_pos < rif.stage_n) {
        rif.rsrfull = true;
        if (rif.n_overrun++ < 10)
            fprintf(stderr, "[rif] OVERRUN #%u : %d mots du burst precedent non lus "
                    "(RSRFULL pose, cf. §12.6 : le bit garde la trace du probleme)\n",
                    rif.n_overrun, rif.stage_n - rif.stage_pos);

        /* @BEQUILLE RIF_BACKPRESSURE — voir rif_backpressure_on() plus haut.
         * On REFUSE le nouveau burst au lieu d'ecraser, pour que le DSP ait le
         * temps de consommer les 296 mots au lieu des ~4 qu'il draine sinon.
         * RSRFULL reste pose : le firmware garde la trace du probleme. */
        /* [2026-08-04, correction du jour meme] N'appliquer la contre-pression
         * QUE si un chemin de drainage est arme. Mesure : `RDMA_MASK=1` sur 36
         * des 52 lignes `[rif] burst #` echantillonnees — le recepteur n'est pas
         * arme la moitie du temps. Refuser un burst dans ce cas est NUISIBLE :
         * l'etage perime ne sera jamais draine, on jette donc des bursts frais
         * jusqu'a ce que la soupape force (mesure : 38 BP-FORCE, et un
         * `292/296 mots encore a lire` inchange apres 200 refus). Quand rien
         * n'est arme, le comportement materiel — ecraser — est le bon. */
        bool drain_arme = !(rif.spcr & SPCR_RDMA_MASK) ||
                          !(rif.spcr & SPCR_RINT_MASK);
        if (rif_backpressure_on() && drain_arme) {
            if (++rif.bp_run <= rif_bp_max_skip()) {
                rif.n_bp_skip++;
                if (rif.n_bp_skip <= 10 || (rif.n_bp_skip % 2000) == 0)
                    fprintf(stderr, "[rif] BP-SKIP #%u : burst refuse, %d/%d mots "
                            "du precedent encore a lire (refus consecutifs %u/%u)\n",
                            rif.n_bp_skip, rif.stage_n - rif.stage_pos,
                            rif.stage_n, rif.bp_run, rif_bp_max_skip());
                return;          /* pas de livraison, donc pas de notification */
            }
            /* Soupape : le DSP ne consomme plus, on impose le burst pour ne pas
             * figer la reception. Un compteur non nul ici DEMENT le progres. */
            rif.n_bp_force++;
            fprintf(stderr, "[rif] BP-FORCE #%u : %u refus consecutifs atteints "
                    "(CALYPSO_RIF_BP_MAX_SKIP), burst impose — le DSP ne draine "
                    "plus, la contre-pression ne suffit pas\n",
                    rif.n_bp_force, rif.bp_run);
        }
    }
    rif.bp_run = 0;

    memcpy(rif.stage, w, (size_t)n * sizeof(uint16_t));
    rif.stage_n = n;
    rif.stage_pos = 0;
    rif.fifo_n = 0;
    rif_refill();
    rif.n_burst++;

    /* §12.6 bits 11/13 : c'est le FIRMWARE qui a choisi le mode, pas nous. */
    bool rint = !(rif.spcr & SPCR_RINT_MASK);
    bool rdma = !(rif.spcr & SPCR_RDMA_MASK);

    if (rint && s) {
        rif.n_int++;
        c54x_interrupt_ex(s, C54X_IT_RIF_RX_VEC, C54X_IT_RIF_RX_BIT);
    }
    if (rdma) {
        rif.n_dma++;
        /* [2026-08-03] La requete de fin de DMA du §3.7.1 a enfin un destinataire.
         * Jusqu'ici on comptait la requete sans la servir : le controleur RHEA
         * journalisait sans transferer. Sans effet si CALYPSO_RHEA_DMA_XFER=0
         * (defaut) — le comportement d'avant est strictement conserve. */
        calypso_rhea_dma_rx_request(s);
    }

    if (rif.n_burst <= 10 || (rif.n_burst % 500) == 0)
        fprintf(stderr, "[rif] burst #%u n=%d -> FIFO %d/%d ; RINT_MASK=%d "
                "RDMA_MASK=%d => %s (IT=%u DMA=%u overrun=%u)\n",
                rif.n_burst, n, rif.fifo_n, RIF_FIFO_DEPTH,
                !rint, !rdma,
                rint ? "INT0n vec16/bit0"
                     : (rdma ? "requete DMA (end-DMA vers l'ARM, §3.7.1)"
                             : "AUCUNE notification — les deux masques sont poses, "
                               "le firmware n'a pas arme le recepteur"),
                rif.n_int, rif.n_dma, rif.n_overrun);

    if (rif_backpressure_on() && (rif.n_burst <= 10 || (rif.n_burst % 500) == 0))
        fprintf(stderr, "[rif] BP-BILAN burst=%u overrun=%u refuses=%u imposes=%u\n",
                rif.n_burst, rif.n_overrun, rif.n_bp_skip, rif.n_bp_force);
}
