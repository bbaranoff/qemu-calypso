/*
 * calypso_rhea_dma.c — contrôleur DMA RHEA du Calypso, côté MCU (FFFF:FC00).
 *
 * POURQUOI CE FICHIER EXISTE (2026-08-03).
 * ----------------------------------------
 * Mesure : le firmware DSP pilote correctement le RIF (séquence RRST à deux
 * écritures du §12.6, relecture de SPCR, réaction sur FIFO non-vide) mais laisse
 * `RINT_MASK` ET `RDMA_MASK` à 1 sur tout le run, et ne lit JAMAIS DRR. Il
 * constate donc que des données sont là et ne les prend pas par le port série.
 * Elles doivent arriver en MÉMOIRE — §3.7.1 : « API interface for radio data in
 * DMA mode (buffered mode with data block transfer) ».
 *
 * [MESURE 03/08, run native_twl] L'ARM ecrit UNE seule chose dans cette fenetre :
 *     ALLOC_CONFIG <- 0x000C  ->  canal1=DSP canal2=DSP canal3=ARM canal4=ARM
 * Il CEDE les deux canaux du RIF au DSP et garde les deux canaux UART. Croise avec
 * la Table 2 du §6 (canaux 0..3 : 0=RIF_DMA_REQ_X, 1=RIF_DMA_REQ_R, 2/3=UARTs) et
 * le §11 (registres DMA1..DMA4), la lecture coherente est DMA1=RIF TX, DMA2=RIF RX,
 * DMA3/DMA4=UARTs — le §6 numerote a partir de 0, le §11 a partir de 1. L'autre
 * lecture (DMA1=RIF RX, DMA2=UART) donnerait un canal UART au DSP, ce qui n'a pas
 * de sens pour ce firmware ; elle reste possible et se departagera sur les valeurs
 * de RAD/AAD reellement ecrites.
 *
 * CONSEQUENCE (§11.3) : « The DMA transfer configuration registers are connected to
 * either the ARM Rhea bus OR TO THE DSP RHEA BUS regarding the corresponding
 * DMA_ALLOC flag. » L'ARM n'ecrira donc JAMAIS DMA1_AAD : les registres des canaux
 * cedes vivent desormais en XIO:FC10 / XIO:FC20, cote DSP. D'ou le pont
 * calypso_rhea_dma_xio() plus bas — meme banc de registres, deuxieme bus.
 *
 * Or la configuration DMA se fait « from the MCU only » (§3.5.15) sur cette
 * fenêtre, et le modèle n'y avait qu'un `add_stub` MUET : lectures à 0, écritures
 * jetées. Toute la programmation DMA de l'ARM disparaissait sans un mot — d'où
 * l'impossibilité de savoir où le burst est censé atterrir.
 *
 * CE QUE CE MODULE FAIT, ET NE FAIT PAS.
 * Il enregistre et restitue les registres du §11, avec leurs valeurs de reset, et
 * journalise chaque écriture DÉCODÉE. Il n'exécute AUCUN transfert : c'est un
 * instrument de lecture, pas une implémentation du DMA. La question qu'il répond
 * est « où l'ARM demande-t-il que le burst soit déposé », et `DMA1_AAD` la donne
 * en clair (canal 1 = `RIF_DMA_REQ_R`, Table 2 du §6).
 *
 * CARTE (§11.1, Table 18) — fenêtre 0x100 à partir de FFFF:FC00 :
 *   +0x00 CONTROLLER_CONFIG   6b R/W   DMA_BURST(4:2)=1, PRIORITY_ENABLE(5)=1
 *   +0x02 ALLOC_CONFIG        4b R/W   reset 1111 = les 4 canaux pilotés par l'ARM
 *   +0x10 DMA1_RAD           16b R/W   RHEA_START(10:0) + RHEA_CS(15:11)
 *   +0x12 DMA1_RDPTH         11b R/W   profondeur du tampon Rhea, en OCTETS
 *   +0x14 DMA1_AAD           12b R/W   début du tampon de réception dans l'API,
 *                                      « The address is always expressed in Bytes »
 *   +0x16 DMA1_ALGTH         12b R/W   longueur de page API, en octets
 *   +0x18 DMA1_CTRL          13b       reset 0x04A2 (cf. ci-dessous)
 *   +0x1A DMA1_CUR_OFFSET_API 12b R    offset courant
 *   … canaux 2/3/4 aux +0x20 / +0x30 / +0x40.
 *
 * DMAn_CTRL (§11.3.5) — la valeur de reset annoncée « 0 0100 1?10 0010 » a été
 * revérifiée champ par champ et tombe sur 0x04A2, ce qui valide la lecture :
 *   0 ENABLE=0 · 1 IDLE=1 (R) · 2 ONE_SHOT=0 · 3 FIFO_MODE=0 · 4 CURRENT_PAGE=0
 *   5 MAS=1 (transferts 16 bits) · 6 DMA_START (W, relu toujours 0)
 *   7 IRQ_MODE=1 · 8 IRQ_STATE=0 (R) · 9 RHEA_ERROR=0 (R)
 *   10 DIRECTION=1 → « Transactions are done on Rhea -> API » · 12:11 PRIORITY=00
 * Le « ? » du doc est DMA_START, qui se relit toujours à zéro. Et DIRECTION=1 au
 * reset confirme le sens attendu pour la réception radio : périphérique → API.
 *
 * NB une incohérence du doc, signalée plutôt que masquée : la Table 18 donne
 * CONTROLLER_CONFIG en reset « 11 111? » alors que la liste des champs du §11.2.1
 * donne DMA_BURST=0x1 et PRIORITY_ENABLE=1, soit 0b100100 = 0x24. On suit la
 * liste des champs, plus précise que la chaîne de la table.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_rhea_dma.h"
#include "hw/arm/calypso/calypso_rif.h"
#include "hw/arm/calypso/calypso_c54x.h"

#include <stdio.h>
#include <stdlib.h>

#define RD_CTRL_CFG   0x00
#define RD_ALLOC_CFG  0x02
#define RD_CH_BASE(n) (0x10 + 0x10 * (n))   /* n = 0..3 -> 0x10 0x20 0x30 0x40 */
#define RD_RAD        0x0
#define RD_RDPTH      0x2
#define RD_AAD        0x4
#define RD_ALGTH      0x6
#define RD_CTRL       0x8
#define RD_CUR_OFF    0xA

#define CTRL_RESET    0x04A2
#define CTRLCFG_RESET 0x0024
#define ALLOC_RESET   0x000F

/* champs de DMAn_CTRL */
#define CTRL_ENABLE       (1u << 0)
#define CTRL_IDLE         (1u << 1)
#define CTRL_ONE_SHOT     (1u << 2)
#define CTRL_FIFO_MODE    (1u << 3)
#define CTRL_CURRENT_PAGE (1u << 4)
#define CTRL_MAS          (1u << 5)
#define CTRL_DMA_START    (1u << 6)
#define CTRL_IRQ_MODE     (1u << 7)
#define CTRL_IRQ_STATE    (1u << 8)
#define CTRL_RHEA_ERROR   (1u << 9)
#define CTRL_DIRECTION    (1u << 10)

/* bits en lecture seule : conservés par le modèle, pas écrasés par l'ARM */
#define CTRL_RO_MASK  (CTRL_IDLE | CTRL_IRQ_STATE | CTRL_RHEA_ERROR)

static struct {
    bool     init;
    uint16_t ctrl_cfg, alloc_cfg;
    struct { uint16_t rad, rdpth, aad, algth, ctrl, cur_off; } ch[4];
    unsigned n_wr, n_rd, n_start;
} rd;

static bool rhea_dma_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RHEA_DMA", 1);
        if (!on)
            fprintf(stderr, "[rhea-dma] CALYPSO_RHEA_DMA=0 : retour au stub muet "
                    "(lectures a 0, ecritures jetees) — comportement d'avant le 03/08\n");
    }
    return on != 0;
}

static void rhea_dma_init(void)
{
    if (rd.init)
        return;
    rd.init = true;
    rd.ctrl_cfg  = CTRLCFG_RESET;
    rd.alloc_cfg = ALLOC_RESET;
    for (int i = 0; i < 4; i++)
        rd.ch[i].ctrl = CTRL_RESET;
    fprintf(stderr, "[rhea-dma] controleur DMA RHEA arme (CAL207 §11) @0xFFFFFC00 : "
            "4 canaux, reset CTRL=0x%04x (DIRECTION=1 Rhea->API, MAS=1 16 bits, "
            "IRQ_MODE=1, ENABLE=0). Canaux RIF : DMA1=TX DMA2=RX (§6 Table 2). "
            "ENREGISTREMENT SEUL : aucun transfert n'est execute.\n", CTRL_RESET);
}

/* Traduit l'adresse API (en OCTETS, §11.3.3) dans les deux repères utiles. */
static void log_aad(int n, uint16_t aad)
{
    uint32_t byte = aad & 0x0FFF;
    fprintf(stderr, "[rhea-dma] *** DMA%d_AAD = 0x%03x octets  ->  mot DSP 0x%04x  "
            "(= ARM 0x%08x). C'est la DESTINATION du tampon de reception dans la "
            "memoire API.\n",
            n + 1, byte, (unsigned)(0x0800 + byte / 2), 0xFFD00000u + byte);
}

static void log_ctrl(int n, uint16_t v)
{
    fprintf(stderr, "[rhea-dma] DMA%d_CTRL <- 0x%04x : ENABLE=%d ONE_SHOT=%d "
            "FIFO_MODE=%d PAGE=%d MAS=%d(%s) DMA_START=%d IRQ_MODE=%d DIRECTION=%d(%s) "
            "PRIORITY=%d\n",
            n + 1, v,
            !!(v & CTRL_ENABLE), !!(v & CTRL_ONE_SHOT), !!(v & CTRL_FIFO_MODE),
            !!(v & CTRL_CURRENT_PAGE), !!(v & CTRL_MAS),
            (v & CTRL_MAS) ? "16b" : "8b",
            !!(v & CTRL_DMA_START), !!(v & CTRL_IRQ_MODE),
            !!(v & CTRL_DIRECTION),
            (v & CTRL_DIRECTION) ? "Rhea->API" : "API->Rhea",
            (v >> 11) & 3);
}

uint64_t calypso_rhea_dma_read(void *opaque, hwaddr off, unsigned size)
{
    (void)opaque; (void)size;
    if (!rhea_dma_on())
        return 0;
    rhea_dma_init();

    uint16_t v = 0;
    if (off == RD_CTRL_CFG)       v = rd.ctrl_cfg;
    else if (off == RD_ALLOC_CFG) v = rd.alloc_cfg;
    else {
        for (int n = 0; n < 4; n++) {
            hwaddr b = RD_CH_BASE(n);
            if (off < b || off > b + RD_CUR_OFF)
                continue;
            switch (off - b) {
            case RD_RAD:     v = rd.ch[n].rad;     break;
            case RD_RDPTH:   v = rd.ch[n].rdpth;   break;
            case RD_AAD:     v = rd.ch[n].aad;     break;
            case RD_ALGTH:   v = rd.ch[n].algth;   break;
            /* DMA_START « Reading of this bit is always equal to zero » (§11.3.5) */
            case RD_CTRL:
                v = rd.ch[n].ctrl & (uint16_t)~CTRL_DMA_START;
                /* ═════════════════════════════════════════════════════════════
                 * [2026-08-04] FIX_DMA_STATUS, patte 1/3 — EFFACEMENT A LA
                 * LECTURE de IRQ_STATE et RHEA_ERROR.
                 *
                 * CAL207 §11.3.5, pour les deux bits :
                 *     « 0 = cleared after being read »
                 * Le modele les laissait colles : une fois poses, le firmware les
                 * relisait indefiniment comme si l'evenement se reproduisait.
                 *
                 * REGLE APPLIQUEE (formulee par l'utilisateur le 04/08) : un
                 * TEMOIN d'erreur cote DSP signale une FONCTION MATERIELLE
                 * ABSENTE du modele. Ici les temoins etaient
                 * `DSP_ERR_DMA_PROG/TASK/PEND` — des files circulaires de 14
                 * entrees (ROM 0xaa83/0xaaad/0xaad1) qui debordent parce que rien
                 * ne dit au DSP qu'un transfert est termine.
                 * ═════════════════════════════════════════════════════════════ */
                if (rd.ch[n].ctrl & (CTRL_IRQ_STATE | CTRL_RHEA_ERROR)) {
                    static unsigned long long n_clr;
                    if (n_clr++ < 20)
                        fprintf(stderr, "[rhea-dma] DMA%d_CTRL lu = 0x%04x : "
                                "IRQ_STATE/RHEA_ERROR effaces a la lecture "
                                "(CAL207 §11.3.5)\n", n + 1, v);
                    rd.ch[n].ctrl &= (uint16_t)~(CTRL_IRQ_STATE | CTRL_RHEA_ERROR);
                }
                break;
            case RD_CUR_OFF: v = rd.ch[n].cur_off; break;
            default: break;
            }
            break;
        }
    }
    if (rd.n_rd++ < 40)
        fprintf(stderr, "[rhea-dma] RD  +0x%02x = 0x%04x\n", (unsigned)off, v);
    return v;
}

void calypso_rhea_dma_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    (void)opaque; (void)size;
    if (!rhea_dma_on())
        return;
    rhea_dma_init();

    uint16_t v = (uint16_t)val;
    rd.n_wr++;

    if (off == RD_CTRL_CFG) {
        rd.ctrl_cfg = v & 0x003F;
        fprintf(stderr, "[rhea-dma] CONTROLLER_CONFIG <- 0x%04x (DMA_BURST=%d "
                "PRIORITY_ENABLE=%d)\n", v, (v >> 2) & 7, !!(v & 0x20));
        return;
    }
    if (off == RD_ALLOC_CFG) {
        rd.alloc_cfg = v & 0x000F;
        fprintf(stderr, "[rhea-dma] ALLOC_CONFIG <- 0x%04x : canal1=%s canal2=%s "
                "canal3=%s canal4=%s\n", v,
                (v & 1) ? "ARM" : "DSP", (v & 2) ? "ARM" : "DSP",
                (v & 4) ? "ARM" : "DSP", (v & 8) ? "ARM" : "DSP");
        return;
    }

    for (int n = 0; n < 4; n++) {
        hwaddr b = RD_CH_BASE(n);
        if (off < b || off > b + RD_CUR_OFF)
            continue;
        switch (off - b) {
        case RD_RAD:
            rd.ch[n].rad = v;
            fprintf(stderr, "[rhea-dma] DMA%d_RAD   <- 0x%04x (RHEA_START=0x%03x "
                    "RHEA_CS=%d)\n", n + 1, v, v & 0x7FF, (v >> 11) & 0x1F);
            break;
        case RD_RDPTH:
            rd.ch[n].rdpth = v & 0x07FF;
            fprintf(stderr, "[rhea-dma] DMA%d_RDPTH <- %u octets\n", n + 1, v & 0x7FF);
            break;
        case RD_AAD:
            /* Jamais plafonne : c'est LA reponse a « ou le burst doit-il atterrir ». */
            rd.ch[n].aad = v & 0x0FFF;
            log_aad(n, v);
            break;
        case RD_ALGTH:
            rd.ch[n].algth = v & 0x0FFF;
            fprintf(stderr, "[rhea-dma] DMA%d_ALGTH <- %u octets (= %u mots de page API)\n",
                    n + 1, v & 0xFFF, (v & 0xFFF) / 2);
            break;
        case RD_CTRL: {
            uint16_t keep = rd.ch[n].ctrl & CTRL_RO_MASK;
            uint16_t before = rd.ch[n].ctrl;
            rd.ch[n].ctrl = (uint16_t)((v & ~CTRL_RO_MASK & 0x1FFF) | keep);
            /* [2026-08-03, CORRIGE PAR LE DESASSEMBLAGE] La v1 de ce commentaire
             * disait « le firmware scrute, il n'arme pas ». FAUX. PROM0 0xa640 :
             *     portr *(0x4356), 0xfc28    ; lit DMA2_CTRL
             *     andm  *(0x4356), #0xfffe   ; EFFACE le bit 0 = ENABLE
             *     portw *(0x4356), 0xfc28    ; reecrit
             * C'est un DESARMEMENT explicite du canal, pas une scrutation. Si
             * l'ecriture ne change rien, c'est seulement parce que ENABLE valait
             * deja 0 chez nous. On compte ces passages au lieu de les repeter, et
             * on journalise tout ce qui CHANGE l'etat ou pose ENABLE/DMA_START. */
            bool inerte = (rd.ch[n].ctrl == before) && !(v & CTRL_DMA_START);
            if (inerte) {
                static unsigned long long n_rmw[4];
                if (n_rmw[n]++ == 0 || (n_rmw[n] % 20000) == 0)
                    fprintf(stderr, "[rhea-dma] DMA%d_CTRL DESARMEMENT x%llu "
                            "(0xa643 andm #0xfffe = efface ENABLE ; sans effet ici, "
                            "ENABLE valait deja 0 — relu/reecrit 0x%04x)\n",
                            n + 1, n_rmw[n], v);
                break;
            }
            log_ctrl(n, v);
            if (v & CTRL_DMA_START) {
                rd.n_start++;
                fprintf(stderr, "[rhea-dma] *** DMA%d DMA_START #%u — l'ARM DEMANDE un "
                        "transfert. Ce module NE L'EXECUTE PAS (instrument de lecture). "
                        "Parametres courants : RAD=0x%04x RDPTH=%u AAD=0x%03x "
                        "(mot DSP 0x%04x) ALGTH=%u\n",
                        n + 1, rd.n_start, rd.ch[n].rad, rd.ch[n].rdpth,
                        rd.ch[n].aad, (unsigned)(0x0800 + rd.ch[n].aad / 2),
                        rd.ch[n].algth);
            }
            break;
        }
        case RD_CUR_OFF:
            /* R seulement (§11.3.6) — on ignore l'ecriture. */
            break;
        default:
            break;
        }
        return;
    }

    fprintf(stderr, "[rhea-dma] WR  +0x%02x = 0x%04x (hors carte §11)\n",
            (unsigned)off, v);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * [2026-08-03] LE TRANSFERT — CALYPSO_RHEA_DMA_XFER=1, defaut 0.
 *
 * POURQUOI MAINTENANT. Le TODO §E posait la condition : « implementer SEULEMENT
 * si A debloque, il n'y aurait rien a transferer avant ». C'est fait — `0xa5cd`
 * (armement RX) s'execute enfin et programme DMA2_AAD/ALGTH/CTRL avec ENABLE=1
 * (cf. doc/ETAT_ACTUEL.md §14.12). Le firmware le confirme lui-meme en posant
 * `DSP_ERR_DMA_PROG | DSP_ERR_DMA_TASK` (d_error_status = 24) a chaque trame :
 * il a arme un transfert que personne n'executait.
 *
 * CE QUE FAIT CE CODE. A la reception d'un burst, quand le firmware a choisi le
 * mode DMA (RDMA_MASK=0, decide par LUI dans SPCR, on ne force rien) :
 *   1. vider le recepteur RIF vers la memoire API a l'adresse DMA2_AAD ;
 *   2. borner le transfert a DMA2_ALGTH (longueur de page, en OCTETS §11.3.3) ;
 *   3. poser IRQ_STATE, effacer DMA_START, et lever INT10n si IRQ_MODE=1.
 *
 * CE QUE CE CODE NE FAIT PAS, VOLONTAIREMENT :
 *   · il n'arme rien de lui-meme — si ENABLE=0, il ne transfere pas. Le but est
 *     de servir une demande du firmware, pas de la fabriquer ;
 *   · il ne touche pas SPCR ni les masques : le choix IT/DMA reste au firmware ;
 *   · il ne bascule pas de page (CURRENT_PAGE) : le double buffering du §11.3.5
 *     n'est pas modelise. Si le firmware s'en sert, ca se verra comme un
 *     ecrasement — a implementer alors, pas par anticipation.
 *
 * ⚠️ CHANGEMENT DE NATURE, d'ou le gate a 0 par defaut : ce module cesse d'etre
 * un instrument de lecture pour devenir une piece de materiel. Tant que le gate
 * est a 0, le comportement d'avant le 03/08 est strictement conserve.
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool rhea_dma_xfer_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_RHEA_DMA_XFER", 0);
        fprintf(stderr, "[rhea-dma] TRANSFERT %s (CALYPSO_RHEA_DMA_XFER=%d) — %s\n",
                on ? "ACTIF" : "inactif", on,
                on ? "le controleur vide le RIF vers la memoire API"
                   : "instrument de lecture seul, aucun transfert (defaut)");
    }
    return on != 0;
}

static void rhea_abort_impl(const char *why)
{
    rd.ch[1].ctrl |= CTRL_RHEA_ERROR;
    static unsigned long long n_ab;
    if (n_ab++ < 20)
        fprintf(stderr, "[rhea-dma] RHEA_ERROR pose : %s (CAL207 §11.3.5, efface a la lecture)\n", why);
}
#define rhea_abort(w) rhea_abort_impl(w)

/* [2026-08-04] Niveau de INT10n : vrai tant qu'un canal garde IRQ_STATE.
 * Voir l'en-tete pour le pourquoi (CAL000 §5.1 : ligne LEVEL). */
bool calypso_rhea_dma_irq_level(void)
{
    if (!rhea_dma_on() || !rd.init)
        return false;
    for (int i = 0; i < 4; i++)
        if (rd.ch[i].ctrl & CTRL_IRQ_STATE)
            return true;
    return false;
}

void calypso_rhea_dma_rx_request(C54xState *s)
{
    if (!rhea_dma_on() || !rhea_dma_xfer_on())
        return;
    rhea_dma_init();

    const int n = 1;                 /* DMA2 = canal RIF RX (§6 Table 2) */
    uint16_t ctrl = rd.ch[n].ctrl;

    /* ═══════════════════════════════════════════════════════════════════════
     * [2026-08-04] FIX_DMA_STATUS, patte 2/3 — le bit IDLE.
     *
     * CAL207 §11.3.5 : « 1 IDLE : 0 = DMA transfer is running / 1 = DMA channel
     * is idle », acces R — c'est le MATERIEL qui le pilote, reset a 1.
     *
     * Le modele le definissait (`CTRL_IDLE`) et ne l'ecrivait JAMAIS : zero site
     * de pilotage, il restait donc a 1 pour toute la duree du run. Le DSP n'avait
     * aucun moyen de distinguer « transfert en cours » de « transfert fini ».
     *
     * CE QUE CA PRODUISAIT, mesure et remonte a sa source dans la ROM :
     *   ROM 0xaa83 / 0xaaad / 0xaad1 posent DMA_PROG / DMA_TASK / DMA_PEND.
     *   Les trois ont la meme forme : une file CIRCULAIRE de 14 entrees
     *   (`stm #0x000e` ; `stl *AR0+%` ; `banz`), et le bit se leve quand elle
     *   DEBORDE. Le DSP empilait donc des requetes que rien ne venait clore.
     *   Compte sur un run : DMA_PROG 948 fois, DMA_PEND 541.
     * Cote osmocom, `layer1/sync.c:249-251` se contente d'imprimer puis
     * d'effacer `d_error_status` — l'ARM ne corrige rien, le temoin est un
     * rapport du DSP.
     *
     * ⚠️ Le bit est marque R dans la doc : le firmware ne doit jamais l'ecrire.
     * `CTRL_RO_MASK` le protege deja des ecritures logicielles ; ici on ne
     * touche qu'a l'etat interne, ce qui est le role du materiel.
     * PLACEMENT : la mise a 0 se fait plus bas, APRES toutes les
     * validations, pour qu'un retour anticipe ne laisse jamais le canal
     * marque « en cours » alors qu'aucun transfert n'a eu lieu.
     * ═══════════════════════════════════════════════════════════════════════ */

    /* On ne sert que ce que le firmware a REELLEMENT arme. */
    if (!(ctrl & CTRL_ENABLE)) {
        static unsigned long long n_off;
        if (n_off++ == 0 || (n_off % 5000) == 0)
            fprintf(stderr, "[rhea-dma] requete RX x%llu ignoree : DMA2 ENABLE=0 "
                    "(le firmware n'a pas arme ce canal — on ne l'arme pas a sa "
                    "place)\n", n_off);
        return;
    }
    if (!(ctrl & CTRL_DIRECTION)) {
        static unsigned n_dir;
        if (n_dir++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : DMA2 DIRECTION=0 "
                    "(API->Rhea), ce n'est pas une reception\n");
        return;
    }

    /* ALGTH est une longueur en OCTETS (§11.3.3) ; la memoire API est en mots. */
    int max_words = rd.ch[n].algth ? (int)(rd.ch[n].algth / 2) : 0;
    if (max_words <= 0) {
        rhea_abort("ALGTH nulle");   /* AVANT le if : l'etat materiel ne doit pas
                                      * dependre du plafond de journal. */
        static unsigned n_len;
        if (n_len++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : DMA2_ALGTH=%u — aucune "
                    "longueur de page programmee\n", rd.ch[n].algth);
        return;
    }

    uint16_t *api = s ? s->api_ram : NULL;
    if (!api) {
        rhea_abort("memoire API absente");
        static unsigned n_api;
        if (n_api++ < 5)
            fprintf(stderr, "[rhea-dma] requete RX ignoree : memoire API absente\n");
        return;
    }

    /* AAD = offset en OCTETS depuis la base API -> index de mot dans api_ram.
     * (mot DSP = 0x0800 + aad/2, et l'index api_ram est ce mot moins C54X_API_BASE,
     * donc exactement aad/2.) */
    unsigned dst_idx = (rd.ch[n].aad & 0x0FFF) / 2;
    if (dst_idx + (unsigned)max_words > C54X_API_SIZE) {
        max_words = (int)(C54X_API_SIZE - dst_idx);
        static unsigned n_clip;
        if (max_words <= 0) {
            rhea_abort("AAD hors fenetre API");
            if (n_clip++ < 5)
                fprintf(stderr, "[rhea-dma] requete RX ignoree : AAD=0x%03x hors "
                        "memoire API\n", rd.ch[n].aad);
            return;
        }
        if (n_clip++ < 5)
            fprintf(stderr, "[rhea-dma] transfert TRONQUE a %d mots : AAD=0x%03x + "
                    "ALGTH depasse la fenetre API\n", max_words, rd.ch[n].aad);
    }

    static uint16_t buf[4096];
    if (max_words > (int)(sizeof(buf) / sizeof(buf[0])))
        max_words = (int)(sizeof(buf) / sizeof(buf[0]));

    /* ─────────────────────────────────────────────────────────────────────────
     * [2026-08-03] ENCHAINEMENT DES PAGES (§11.3.5 CURRENT_PAGE).
     *
     * MESURE QUI L'IMPOSE. La v1 ne drainait QU'UNE page par requete :
     *     TRANSFERT RX : 96 mots   ALGTH=192      <- page = 96 mots
     *     burst        : n=296 mots               <- burst = 296 mots
     *     overrun      : 11 392 sur 13 000 bursts (84 %)
     * Les 200 mots restants attendaient dans l'etage du RIF, et le burst suivant
     * les y trouvait -> RSRFULL -> overrun. Il faut ~3 pages par burst. A 84 % de
     * perte, assembler 4 bursts consecutifs (un bloc CCCH) est impossible : ca
     * suffit a expliquer SI=0 SANS mettre en cause la qualite de la demodulation.
     *
     * ⚠️ HYPOTHESE ASSUMEE, pas une certitude. Deux lectures tenaient :
     *   (1) le DMA doit enchainer les pages (§11.3.5) — retenue ici, c'est celle
     *       de la doc, et elle est reversible (meme gate) ;
     *   (2) l'hote fournit la mauvaise granularite (CALYPSO_BSP_DARAM_LEN=296
     *       contre des pages de 96 mots) — si (1) ne suffit pas, c'est (2), et ca
     *       se teste en changeant un seul parametre.
     * Ne pas presenter (1) comme etablie tant qu'un run ne l'a pas montree.
     *
     * CURRENT_PAGE bascule a chaque page pour refleter le double buffering du
     * §11.3.5. La destination NE bouge PAS entre les pages : le firmware relit
     * la meme fenetre a chaque end-DMA (une IT est levee par page si IRQ_MODE).
     * C'est le point le moins sur de cette implementation — si le firmware
     * attendait deux tampons distincts (AAD et AAD+ALGTH), ca se verrait comme
     * un ecrasement de la premiere moitie. */
    /* FIX_DMA_STATUS 2/3 : a partir d'ici le transfert a REELLEMENT lieu. */
    rd.ch[n].ctrl &= (uint16_t)~CTRL_IDLE;      /* 0 = transfert en cours */
    int total = 0, pages = 0;
    /* [2026-08-04] Comptage du contenu accumule SUR TOUTES LES PAGES. L'ancienne
     * sonde bouclait sur `buf[0..total[` alors que `buf` ne contient QUE la
     * derniere page (max_words mots) : au-dela elle relisait des restes. */
    int nz_total = 0;
    uint16_t head8[8] = {0};
    for (;;) {
        int got = calypso_rif_drain(buf, max_words);
        if (got <= 0)
            break;

        /* [2026-08-22] DOUBLE-BUFFER §11.3.5 (CAL207) — la destination SUIT
         * CURRENT_PAGE. Doc DMA_CTRL bit4 : « 0 = 1re page API, 1 = 2e page API,
         * automatiquement mis a jour pendant le transfert ». Page 0 = AAD
         * (dst_idx), page 1 = AAD+ALGTH (dst_idx+max_words). L'ancien code ecrivait
         * TOUJOURS dst_idx -> la 2e page (0x0d2e) restait VIDE, alors que le DSP y
         * lit une fois sur deux (desasm PROM 0xb2c4=0x0cce / 0xb2c9=0x0d2e) ->
         * energie=0 -> d_fb_det jamais pose. Triangule doc + disasm + logs.
         * A/B : CALYPSO_RHEA_DMA_PAGE_OFF=1 restaure le dst fixe (ancien comportement). */
        /* [2026-08-22 soir] CORRELATEUR = 296 MOTS PLATS, PAS 2 PAGES DE 96.
         * Le correlateur FB lit un buffer CONTIGU de 296 mots (=148 IQ @1SPS).
         * Le ping-pong §11.3.5 a 2 pages (dst_idx / dst_idx+96 = 192 mots) repliait
         * le burst 296 : page2->dst_idx (ecrase page0), page3->dst_idx+96 (ecrase
         * page1). Resultat : FCCH corrompue, le correlateur pique au BORD (TOA=39).
         * Le double-buffer HW est un artefact TEMPS-REEL (le DSP consomme la page 0
         * pendant que le DMA remplit la page 1) ; il n'a PAS lieu d'etre quand on
         * draine tout le burst en une passe synchrone. On ecrit donc les pages
         * CONTIGUES : page k -> dst_idx + k*96, soit [0x0cce..0x0df6) plat -> le
         * correlateur voit les 296 mots.
         * A/B : CALYPSO_RHEA_DMA_PINGPONG=1 restaure l'ancien ping-pong 2-pages. */
        unsigned pdst = dst_idx + (unsigned)(pages * max_words);
        {
            static int pingpong = -1;
            if (pingpong < 0) pingpong = getenv("CALYPSO_RHEA_DMA_PINGPONG") ? 1 : 0;
            if (pingpong) {
                pdst = dst_idx;
                if (rd.ch[n].ctrl & CTRL_CURRENT_PAGE)
                    pdst = dst_idx + (unsigned)max_words;   /* 2e page API = AAD+ALGTH */
            }
        }
        for (int i = 0; i < got; i++)
            if (pdst + (unsigned)i < C54X_API_SIZE)
                api[pdst + i] = buf[i];

        for (int i = 0; i < got; i++)
            if (buf[i]) nz_total++;
        if (pages == 0)
            for (int i = 0; i < 8 && i < got; i++) head8[i] = buf[i];

        total += got;
        pages++;
        rd.ch[n].cur_off = (uint16_t)(got * 2);
        rd.ch[n].ctrl ^= CTRL_CURRENT_PAGE;      /* §11.3.5 : page suivante */

        if (got < max_words)
            break;                                /* recepteur vide avant la fin */
        if (pages >= 64) {                        /* garde-fou : jamais de boucle infinie */
            static unsigned n_cap;
            if (n_cap++ < 5)
                fprintf(stderr, "[rhea-dma] ⚠ plafond de 64 pages atteint sur une "
                        "requete (burst anormalement long ?) — reste non draine\n");
            break;
        }
    }

    if (total <= 0) {
        rd.ch[n].ctrl |= CTRL_IDLE;            /* rien transfere -> au repos */
        static unsigned long long n_empty;
        if (n_empty++ == 0 || (n_empty % 5000) == 0)
            fprintf(stderr, "[rhea-dma] requete RX x%llu : recepteur RIF vide, "
                    "rien a transferer\n", n_empty);
        return;
    }
    int got = total;

    /* §11.3.5 : le transfert est fini -> IRQ_STATE, et DMA_START retombe. */
    rd.ch[n].ctrl = (uint16_t)((rd.ch[n].ctrl | CTRL_IRQ_STATE | CTRL_IDLE)
                               & ~CTRL_DMA_START);   /* fini -> IDLE=1 */
    if (rd.ch[n].ctrl & CTRL_ONE_SHOT)
        rd.ch[n].ctrl &= (uint16_t)~CTRL_ENABLE;

    {
        static unsigned long long n_ok;
        n_ok++;
        if (n_ok <= 20 || (n_ok % 500) == 0)
            fprintf(stderr, "[rhea-dma] *** TRANSFERT RX #%llu : %d mots RIF -> "
                    "api_ram[0x%04x..0x%04x] en %d page(s) (mot DSP 0x%04x, ARM 0x%08x) "
                    "ALGTH=%u IRQ_MODE=%d ONE_SHOT=%d\n",
                    n_ok, got, dst_idx, dst_idx + (got > max_words ? max_words : got) - 1, pages,
                    (unsigned)(C54X_API_BASE + dst_idx),
                    0xFFD00000u + (rd.ch[n].aad & 0x0FFF),
                    rd.ch[n].algth, !!(rd.ch[n].ctrl & CTRL_IRQ_MODE),
                    !!(rd.ch[n].ctrl & CTRL_ONE_SHOT));
        /* [2026-08-03] CONTENU, pas seulement la plomberie. Question ouverte :
         * le DSP mesure `PM MEAS: 0 dBm at baseband` en natif alors que le DMA
         * transporte 296 mots par burst. Deux lectures possibles — (1) le chemin
         * PM/FB lit un autre tampon que 0x0cce, (2) le DMA transporte fidelement
         * du VIDE. On ne peut pas trancher sans regarder ce qui passe.
         * Un echantillon suffit : des zeros disent (2) et renvoient le probleme en
         * amont, dans ce que l'hote injecte au RIF ; des valeurs IQ plausibles
         * disent (1) et il faudra trouver quel tampon le PM interrogue.
         * Plafonne aux 10 premiers transferts — c'est un diagnostic, pas une trace. */
        /* [2026-08-04] ⚠️ CORRECTIF DE SONDE. L'ancienne version etait plafonnee
         * a `n_ok <= 10`, donc elle ne regardait QUE les 10 premiers transferts —
         * c'est-a-dire le DEMARRAGE, ou l'entree est reellement vide (mesure
         * FEED-FP : bursts #1..#5 a nz=0/2368, puis #6 a nz=2211/2368). D'ou la
         * conclusion FAUSSE, repetee plusieurs fois, que « le DMA transporte des
         * zeros ». Meme cadence que la ligne TRANSFERT au-dessus, pour qu'un
         * echantillon corresponde toujours a un transfert journalise. */
        if (n_ok <= 20 || (n_ok % 500) == 0) {
            fprintf(stderr, "[rhea-dma]     contenu : %d/%d mots non nuls ; "
                    "8 premiers = %04x %04x %04x %04x %04x %04x %04x %04x\n",
                    nz_total, total,
                    head8[0], head8[1], head8[2], head8[3],
                    head8[4], head8[5], head8[6], head8[7]);
        }
    }

    /* §11.3.5 IRQ_MODE : c'est le firmware qui a decide s'il veut l'interruption. */
    if ((rd.ch[n].ctrl & CTRL_IRQ_MODE) && s) {
        static unsigned long long n_it;
        n_it++;
        if (n_it <= 10 || (n_it % 500) == 0)
            fprintf(stderr, "[rhea-dma] end-DMA -> INT10n (vec%d/bit%d) #%llu\n",
                    C54X_IT_DMA_VEC, C54X_IT_DMA_BIT, n_it);
        c54x_interrupt_ex(s, C54X_IT_DMA_VEC, C54X_IT_DMA_BIT);
    }
}

/* Pont XIO : le MEME banc de registres, vu du bus Rhea du DSP.
 * §11.1 donne les deux adresses pour chaque registre (FFFF:FCxx et XIO:FCxx) ;
 * lequel des deux bus y accede depend de DMA_ALLOC. Puisque l'ARM a cede les
 * canaux du RIF au DSP (ALLOC_CONFIG=0x000C mesure), c'est par ici que passera
 * la programmation du transfert de reception. */
bool calypso_rhea_dma_xio(bool write, uint16_t pa, uint16_t *val, uint16_t pc)
{
    if (pa < 0xFC00 || pa > 0xFCFF)
        return false;
    if (!rhea_dma_on())
        return false;
    hwaddr off = pa - 0xFC00;
    /* [2026-08-03] DEDUPE, PAS PLAFOND. La v1 coupait a 40 lignes et la boucle
     * de scrutation de CTRL (a640/a646/a652) l'a saturee en quelques trames — on
     * ne pouvait donc PAS conclure « AAD n'est jamais ecrit », le journal etant
     * tronque par l'instrument lui-meme. On journalise desormais chaque triplet
     * (sens, PA, PC) UNE fois, puis on resume periodiquement. Rien n'est perdu :
     * une premiere ecriture d'AAD, meme tardive, sortira toujours. */
    {
        struct seen { uint16_t pa, pc; uint8_t wr; unsigned long long n; };
        static struct seen tab[64];
        static int ntab = 0;
        int i, found = -1;
        for (i = 0; i < ntab; i++)
            if (tab[i].pa == pa && tab[i].pc == pc && tab[i].wr == (write ? 1 : 0))
                { found = i; break; }
        if (found < 0) {
            if (ntab < 64) { tab[ntab].pa = pa; tab[ntab].pc = pc;
                             tab[ntab].wr = write ? 1 : 0; tab[ntab].n = 1;
                             found = ntab++; }
            fprintf(stderr, "[rhea-dma] XIO %s PA=0x%04x (+0x%02x) %s0x%04x PC=0x%04x "
                    "— NOUVEAU site (acces DSP, canal cede par ALLOC_CONFIG)\n",
                    write ? "PORTW" : "PORTR", pa, (unsigned)off,
                    write ? "<- " : "-> ", val ? *val : 0, pc);
        } else {
            tab[found].n++;
            if ((tab[found].n % 20000) == 0)
                fprintf(stderr, "[rhea-dma] XIO %s PA=0x%04x PC=0x%04x : %llu passages "
                        "(valeur courante 0x%04x)\n", write ? "PORTW" : "PORTR",
                        pa, pc, tab[found].n, val ? *val : 0);
        }
    }
    if (write) {
        calypso_rhea_dma_write(NULL, off, *val, 2);
    } else {
        *val = (uint16_t)calypso_rhea_dma_read(NULL, off, 2);
    }
    return true;
}
