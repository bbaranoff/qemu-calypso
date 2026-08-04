/*
 * calypso_dma.c — contrôleur DMA interne du TMS320C54x
 *
 * Voir calypso_dma.h pour le pourquoi, le conflit de mapping et la liste
 * explicite de ce qui n'est PAS modélisé.
 *
 * Configuration :
 *   CALYPSO_DMA=1            active le module (défaut OFF)
 *   CALYPSO_DMA_VEC_BASE=30  vecteur de l'IT DMA (defaut 30 = INT10n = IMR
 *                            bit 14 + 16, CAL000 §5.1 ; partage par les 4 canaux)
 *   CALYPSO_DMA_MAX_MOTS=…   garde-fou par transfert (défaut 4096)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_dma.h"
#include "calypso_c54x.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int calypso_dma_actif = -1;   /* -1 = à initialiser au premier accès */

static int      g_init;
/* [2026-08-03] CORRIGE d'apres CAL000 §5.1. AVANT : 23, au motif « DMAC0 =
 * IMR bit 7 + 16 » — c'est la table du C54x GENERIQUE (SPRU131). Sur Calypso le
 * bit 7 est INT4n = MCSI TRANSMIT, et il n'existe pas de vecteurs DMAC0..3 : le
 * DSP ne voit qu'UNE ligne DMA, INT10n = bit 14 = vec 30. Les quatre canaux
 * (§3.5.15) partagent donc ce meme vecteur ; le canal se lit dans les registres,
 * pas dans le numero d'IT. */
static int      g_vec_base = C54X_IT_DMA_VEC;   /* INT10n, partage par les 4 canaux */
static unsigned g_max_mots = 4096;
static unsigned g_log;

/* État propre au module : le banc de sous-registres de calypso_c54x.c ne
 * compte que 4 registres par canal, on garde donc ici ce qui manque. */
static uint16_t g_dmprec;                       /* dernier DMPREC écrit */
static uint16_t g_subaddr;                      /* DMSA */
static uint16_t g_sub[C54X_DMA_CANAUX * 5];     /* SRC/DST/CTR/SFC/MCR */
static uint8_t  g_arme[C54X_DMA_CANAUX];        /* canal activé, transfert dû */

void calypso_dma_init(void)
{
    const char *e;

    if (g_init) {
        return;
    }
    g_init = 1;

    e = getenv("CALYPSO_DMA");
    if (!e || !*e || !strcmp(e, "0")) {
        calypso_dma_actif = 0;
        return;
    }
    e = getenv("CALYPSO_DMA_VEC_BASE");
    if (e && *e) {
        g_vec_base = (int)strtol(e, NULL, 0);
    }
    e = getenv("CALYPSO_DMA_MAX_MOTS");
    if (e && *e) {
        g_max_mots = (unsigned)strtoul(e, NULL, 0);
    }

    calypso_dma_actif = 1;
    fprintf(stderr, "[dma] contrôleur DMA c54x ACTIF — mapping SPRU131 "
            "(DMPREC=0x54 DMSA=0x55 DMSDI=0x56 DMSDN=0x57), DMAC0=vec%d, "
            "plafond %u mots/transfert.\n"
            "[dma] ⚠️ transfert EN BLOC au tick de trame : la synchronisation "
            "par événement (DMSFC) n'est PAS modélisée.\n",
            g_vec_base, g_max_mots);
}

/* Nom lisible d'un sous-registre, pour les traces. */
static const char *dma_nom_sub(unsigned reg)
{
    static const char *n[5] = { "SRC", "DST", "CTR", "SFC", "MCR" };
    return (reg < 5) ? n[reg] : "?";
}

bool calypso_dma_mmr_write(C54xState *s, uint16_t addr, uint16_t val)
{
    if (calypso_dma_actif < 0) {
        calypso_dma_init();
    }
    if (!calypso_dma_actif) {
        return false;            /* éteint : le décodage historique s'applique */
    }

    switch (addr) {
    case C54X_DMPREC: {
        uint16_t avant = g_dmprec;
        g_dmprec = val;
        /* Bits 0..5 : DE0..DE5, activation par canal. Un front montant arme le
         * transfert ; on ne le déclenche PAS ici mais au tick, pour que le
         * firmware ait fini d'écrire ses sous-registres. */
        for (unsigned c = 0; c < C54X_DMA_CANAUX; c++) {
            uint16_t bit = (uint16_t)(1u << c);
            if ((val & bit) && !(avant & bit)) {
                g_arme[c] = 1;
                if (g_log < 40) {
                    g_log++;
                    fprintf(stderr, "[dma] canal %u ARMÉ par DMPREC=0x%04x "
                            "(SRC=0x%04x DST=0x%04x CTR=%u) PC=0x%04x\n",
                            c, val, g_sub[c * 5 + 0], g_sub[c * 5 + 1],
                            g_sub[c * 5 + 2], s->pc);
                }
            }
        }
        s->data[addr] = val;
        return true;
    }

    case C54X_DMSA:
        g_subaddr = val;
        s->data[addr] = val;
        return true;

    case C54X_DMSDI:
    case C54X_DMSDN:
        if (g_subaddr < C54X_DMA_CANAUX * 5) {
            g_sub[g_subaddr] = val;
            if (g_log < 40) {
                g_log++;
                fprintf(stderr, "[dma] canal %u %s = 0x%04x (sous-adr 0x%02x) "
                        "PC=0x%04x\n", g_subaddr / 5,
                        dma_nom_sub(g_subaddr % 5), val, g_subaddr, s->pc);
            }
        }
        s->data[addr] = val;
        if (addr == C54X_DMSDI) {
            g_subaddr++;          /* auto-incrément, DMSDN ne l'a pas */
        }
        return true;

    default:
        return false;
    }
}

void calypso_dma_tick(C54xState *s)
{
    if (calypso_dma_actif < 0) {
        calypso_dma_init();
    }
    if (!calypso_dma_actif || !s) {
        return;
    }

    for (unsigned c = 0; c < C54X_DMA_CANAUX; c++) {
        if (!g_arme[c]) {
            continue;
        }
        uint16_t src = g_sub[c * 5 + 0];
        uint16_t dst = g_sub[c * 5 + 1];
        uint16_t ctr = g_sub[c * 5 + 2];
        unsigned n   = ctr;

        /* Garde-fou : un CTR aberrant (sous-registres pas encore écrits, ou
         * mapping faux) ne doit pas balayer toute la mémoire du DSP. */
        if (n == 0 || n > g_max_mots) {
            if (g_log < 40) {
                g_log++;
                fprintf(stderr, "[dma] canal %u : CTR=%u hors plage [1..%u] — "
                        "transfert IGNORÉ (sous-registres incomplets ?)\n",
                        c, n, g_max_mots);
            }
            g_arme[c] = 0;
            continue;
        }

        /* Transfert bloc, espace DONNÉES uniquement (cf. en-tête : les espaces
         * programme et I/O ne sont pas modélisés). */
        for (unsigned i = 0; i < n; i++) {
            s->data[(uint16_t)(dst + i)] = s->data[(uint16_t)(src + i)];
        }

        g_sub[c * 5 + 2] = 0;     /* CTR épuisé */
        g_arme[c] = 0;
        g_dmprec = (uint16_t)(g_dmprec & ~(1u << c));   /* le canal se désarme */
        s->data[C54X_DMPREC] = g_dmprec;

        if (g_log < 40) {
            g_log++;
            fprintf(stderr, "[dma] canal %u TRANSFÉRÉ %u mots 0x%04x -> 0x%04x, "
                    "interruption DMA INT10n (canal %u, vec %d, IMR bit %d)\n",
                    c, n, src, dst, c, g_vec_base, g_vec_base - 16);
        }

        /* Completion : c'est CE signal qui manquait. §5.1 : ligne unique INT10n
         * pour les 4 canaux, IMR bit = vec - 16 (formule de calypso_c54x.h). */
        c54x_interrupt_ex(s, g_vec_base, g_vec_base - 16);
    }
}
