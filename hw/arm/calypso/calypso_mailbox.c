/*
 * calypso_mailbox.c — moniteur COMPLET de la mailbox ARM <-> DSP
 *
 * Voir calypso_mailbox.h pour le pourquoi. Configuration :
 *
 *   CALYPSO_MAILBOX=1|all     tout le flux de la fenêtre mailbox (0x0800..0x0FFF)
 *   CALYPSO_MAILBOX=w         écritures seulement (les deux sens)
 *   CALYPSO_MAILBOX=r         lectures seulement
 *   CALYPSO_MAILBOX_CELLS=0x098b,0x43d8,...   AJOUTE des cellules hors mailbox
 *                             (reprend l'idée de CALYPSO_WATCH_WR_ADDR, qu'il
 *                             remplace : ici les deux sens et les deux côtés)
 *   CALYPSO_MAILBOX_ONLY=1    ne trace QUE les cellules de _CELLS
 *   CALYPSO_MAILBOX_FILE=...  défaut $CALYPSO_LOG_DIR/mailbox.log, sinon
 *                             /tmp/calypso/logs/mailbox.log
 *   CALYPSO_MAILBOX_MAX=N     garde-fou (défaut 5 000 000 lignes, 0 = illimité)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "calypso_mailbox.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int calypso_mbx_actif = -1;   /* -1 = à initialiser au premier accès */

static FILE    *g_f;
static int      g_init;
static int      g_ecr = 1, g_lec = 1;      /* écritures / lectures */
static int      g_only;                    /* 1 = seulement _CELLS */
static uint16_t g_cells[32];
static int      g_ncells;
/* [2026-08-22] PLAGES arbitraires : la fenetre API seule laissait des regions
 * entieres du modele hors de portee (la reference de correlation 0x2cea.., le
 * tampon de burst 0x2a00.., le scratch-pad 0x0060..). 32 cellules unitaires ne
 * suffisaient pas pour une region de 128 mots. */
static struct { uint16_t lo, hi; } g_ranges[16];
static int      g_nranges;
static unsigned long g_n, g_max = 5000000UL;

/* Repliement PAR CELLULE — journaliser au CHANGEMENT, pas au non-consécutif.
 *
 * [2026-07-29, seconde version] La première repliait les événements CONSÉCUTIFS
 * identiques. Ça ne replie rien : la boucle de fond du DSP interroge DEUX
 * cellules en alternance (`0xde86 ld *(0x098c)` puis `0xddfd ld *(0x098d)`),
 * donc jamais deux lignes de suite identiques. Résultat mesuré : 291 Mo et
 * 3 690 446 lignes en quelques minutes, et QEMU tué.
 *
 * La bonne sémantique est par cellule : une lecture qui rend la MÊME valeur au
 * MÊME PC ne porte aucune information neuve, même si d'autres accès s'intercalent.
 * On garde donc, pour chaque mot, la dernière valeur journalisée et le contexte ;
 * on n'écrit qu'au changement, en résumant la série précédente par « x N ».
 * Les écritures sont toujours journalisées : elles sont rares et chacune est un
 * événement.  CALYPSO_MAILBOX_BRUT=1 désactive tout le repliement. */
static int       g_brut;
static uint16_t *g_dval;      /* dernière valeur journalisée, par mot */
static uint32_t *g_dctx;      /* dernier contexte journalisé, par mot */
static uint32_t *g_drep;      /* longueur de la série en cours, par mot */
static uint8_t  *g_dvu;       /* ce mot a-t-il déjà été journalisé ? */
static uint8_t  *g_dsens;     /* sens du dernier événement journalisé */

/* Noms des cellules qui reviennent sans cesse dans le diagnostic. Une trace
 * lisible évite de re-chercher « 0x08fa c'était quoi déjà » à chaque lecture. */
static const char *mbx_nom(uint16_t m)
{
    switch (m) {
    case 0x0804: return "d_task_md/wp0";
    case 0x0818: return "d_task_md/wp1";
    case 0x0828: return "d_task_d/rp0";
    case 0x0829: return "d_burst_d/rp0";
    case 0x083C: return "d_task_d/rp1";
    case 0x083D: return "d_burst_d/rp1";
    case 0x0810: return "d_ctrl_system";
    /* [2026-07-29] 0x08E2 etait libelle « d_dsp_page » : faux de +14 mots.
     * NDB+0 = 0x08D4 = d_dsp_page ; NDB+14 = 0x08E2 = d_dsp_state (l'ARM y
     * ecrit 3 = C_DSP_IDLE3 depuis dsp.c:215). Voir calypso_fbsb.h. */
    case 0x08D4: return "d_dsp_page";
    case 0x08D5: return "d_error_status";
    case 0x08E2: return "d_dsp_state";
    case 0x08F8: return "d_fb_det";
    case 0x08F9: return "d_fb_mode";
    case 0x08FA: return "a_sync[TOA]";
    case 0x08FB: return "a_sync[PM]";
    case 0x08FC: return "a_sync[ANGLE]";
    case 0x08FD: return "a_sync[SNR]";
    case 0x098A: return "d_backgnd_en";
    case 0x098B: return "d_backgnd_?b";
    case 0x098C: return "d_backgnd_st";
    case 0x098D: return "d_backgnd_?d";
    case 0x098E: return "tab_handlers";
    case 0x0FFF: return "d_task_word";
    case 0x43D8: return "slot_handler";
    default:     return "";
    }
}

static void mbx_ecrire(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                       uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn,
                       unsigned long rep);

void calypso_mbx_init(void)
{
    const char *e;

    if (g_init) {
        return;
    }
    g_init = 1;

    e = getenv("CALYPSO_MAILBOX");
    if (!e || !*e || !strcmp(e, "0")) {
        calypso_mbx_actif = 0;        /* éteint : coût nul sur les chemins chauds */
        return;
    }
    if (!strcmp(e, "w") || !strcmp(e, "W")) {
        g_lec = 0;
    } else if (!strcmp(e, "r") || !strcmp(e, "R")) {
        g_ecr = 0;
    }

    e = getenv("CALYPSO_MAILBOX_CELLS");
    if (e && *e) {
        const char *p = e;
        while (*p && g_ncells < 32) {
            char *fin = NULL;
            long v;
            while (*p == ',' || *p == ' ') {
                p++;
            }
            if (!*p) {
                break;
            }
            v = strtol(p, &fin, 0);
            if (fin == p) {
                break;                /* rien de lisible : on arrête là */
            }
            g_cells[g_ncells++] = (uint16_t)v;
            p = fin;
        }
    }
    /* [2026-08-22] CALYPSO_MAILBOX_RANGES=lo-hi,lo-hi,... (bornes INCLUSES).
     * Meme analyse tolerante que _CELLS : on s arrete au premier illisible
     * plutot que de deviner. Les bornes sont remises dans l ordre si besoin. */
    e = getenv("CALYPSO_MAILBOX_RANGES");
    if (e && *e) {
        const char *p = e;
        while (*p && g_nranges < 16) {
            char *fin = NULL;
            long lo, hi;
            while (*p == ',' || *p == ' ') {
                p++;
            }
            if (!*p) {
                break;
            }
            lo = strtol(p, &fin, 0);
            if (fin == p) {
                break;
            }
            p = fin;
            if (*p == '-' || *p == ':') {
                p++;
                hi = strtol(p, &fin, 0);
                if (fin == p) {
                    break;
                }
                p = fin;
            } else {
                hi = lo;              /* une borne seule = une cellule */
            }
            if (lo > hi) { long t = lo; lo = hi; hi = t; }
            g_ranges[g_nranges].lo = (uint16_t)lo;
            g_ranges[g_nranges].hi = (uint16_t)hi;
            g_nranges++;
        }
    }

    e = getenv("CALYPSO_MAILBOX_ONLY");
    g_only = (e && *e == '1');

    e = getenv("CALYPSO_MAILBOX_BRUT");
    g_brut = (e && *e == '1');

    e = getenv("CALYPSO_MAILBOX_MAX");
    if (e && *e) {
        g_max = strtoul(e, NULL, 0);
    }

    e = getenv("CALYPSO_MAILBOX_FILE");
    if (!e || !*e) {
        static char def[512];
        /* Le dépôt exporte LOG_DIR (paths.env) ; CALYPSO_LOG_DIR n'existe pas —
         * première version de ce code : mauvais nom, le fichier serait toujours
         * tombé dans le repli. On accepte les deux, LOG_DIR fait foi. */
        const char *d = getenv("LOG_DIR");
        if (!d || !*d) d = getenv("CALYPSO_LOG_DIR");
        snprintf(def, sizeof(def), "%s/mailbox.log",
                 (d && *d) ? d : "/tmp/calypso/logs");
        e = def;
    }
    g_f = fopen(e, "w");
    if (!g_f) {
        fprintf(stderr, "[mbx] ouverture impossible : %s — moniteur INACTIF\n", e);
        calypso_mbx_actif = 0;
        return;
    }
    /* Tampon PLEIN et large. En _IOLBF chaque ligne était un appel système :
     * 3,7 M de write() ont suffi à mettre QEMU à genoux. On vide périodiquement
     * (voir mbx_ecrire) pour qu'un arrêt brutal ne perde qu'une fenêtre. */
    setvbuf(g_f, NULL, _IOFBF, 1 << 20);

    g_dval = calloc(0x10000, sizeof(*g_dval));
    g_dctx = calloc(0x10000, sizeof(*g_dctx));
    g_drep = calloc(0x10000, sizeof(*g_drep));
    g_dvu  = calloc(0x10000, sizeof(*g_dvu));
    g_dsens = calloc(0x10000, sizeof(*g_dsens));
    if (!g_dval || !g_dctx || !g_drep || !g_dvu || !g_dsens) {
        fprintf(stderr, "[mbx] allocation impossible — moniteur INACTIF\n");
        calypso_mbx_actif = 0;
        return;
    }

    fprintf(g_f,
        "# moniteur mailbox ARM<->DSP — un evenement par ligne, format stable\n"
        "# sens : ARM>WR (commande)  ARM<RD (resultat)  DSP>WR (resultat)  DSP<RD (commande)\n"
        "# %-10s %-8s %-6s %-6s %-14s %s\n",
        "insn", "fn", "sens", "mot", "nom", "valeur");
    fflush(g_f);

    fprintf(stderr, "[mbx] moniteur mailbox ACTIF -> %s "
            "(ecr=%d lec=%d cellules_sup=%d only=%d max=%lu)\n",
            e, g_ecr, g_lec, g_ncells, g_only, g_max);

    /* [2026-08-22] Annonce de ce qui est REELLEMENT couvert. Sans elle on ne
     * distingue pas « la cellule n a pas bouge » de « la cellule n etait pas
     * surveillee » — c est ce qui m avait fait chercher 0x2cea a la main. */
    {
        int i;
        fprintf(stderr, "[mailbox] couverture : %s",
                g_only ? "SEULEMENT les cellules et plages listees"
                       : "fenetre API 0x0800-0x0FFF");
        for (i = 0; i < g_ncells; i++) {
            fprintf(stderr, " + 0x%04x", g_cells[i]);
        }
        for (i = 0; i < g_nranges; i++) {
            fprintf(stderr, " + [0x%04x-0x%04x, %u mots]",
                    g_ranges[i].lo, g_ranges[i].hi,
                    (unsigned)(g_ranges[i].hi - g_ranges[i].lo + 1));
        }
        fprintf(stderr, " | sens : %s%s | garde-fou : %lu ligne(s)%s\n",
                g_ecr ? "ecritures " : "", g_lec ? "lectures" : "",
                g_max, g_max ? "" : " (illimite)");
    }
    calypso_mbx_actif = 1;
}

/* Dans la fenêtre mailbox, ou dans la liste supplémentaire ? */
static int mbx_retenu(uint16_t mot)
{
    int i;

    for (i = 0; i < g_ncells; i++) {
        if (g_cells[i] == mot) {
            return 1;
        }
    }
    for (i = 0; i < g_nranges; i++) {
        if (mot >= g_ranges[i].lo && mot <= g_ranges[i].hi) {
            return 1;
        }
    }
    if (g_only) {
        return 0;
    }
    return (mot >= 0x0800 && mot <= 0x0FFF);
}

void calypso_mbx_evt(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                     uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn)
{
    int est_ecr;

    if (!g_init) {
        calypso_mbx_init();
    }
    if (!g_f) {
        return;
    }

    est_ecr = (sens == MBX_ARM_WR || sens == MBX_DSP_WR);
    if ((est_ecr && !g_ecr) || (!est_ecr && !g_lec)) {
        return;
    }
    if (!mbx_retenu(mot)) {
        return;
    }

    if (g_max && g_n >= g_max) {
        if (g_n == g_max) {
            g_n++;
            fprintf(g_f, "# PLAFOND %lu lignes atteint — trace interrompue. "
                         "CALYPSO_MAILBOX_MAX=0 pour l'enlever.\n", g_max);
            fflush(g_f);
        }
        return;
    }
    g_n++;

    if (g_brut) {
        mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
        return;
    }

    /* Une écriture qui CHANGE la valeur est toujours un événement : elle passe
     * sans condition, et elle redéfinit la référence.
     *
     * [2026-07-29, troisième version] Auparavant TOUTE écriture passait. Mesuré
     * sur un run natif : 87 174 lignes sur 92 150 étaient le DSP réécrivant
     * « 0x0000 -> 0x0000 » depuis le même PC (@0xb446), une fois par trame. Une
     * écriture qui ne change rien, depuis le même PC, ne porte pas plus
     * d'information qu'une lecture — même règle pour les deux. */
    if (est_ecr && avant != val) {
        if (g_drep[mot] > 1) {
            mbx_ecrire(g_dsens[mot], mot, g_dval[mot], g_dval[mot], g_dctx[mot],
                       fn, insn, g_drep[mot] - 1);
        }
        g_drep[mot] = 0;
        mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
        g_dval[mot]  = val;
        g_dctx[mot]  = ctx;
        g_dsens[mot] = sens;
        g_dvu[mot]   = 1;
        return;
    }

    /* Sinon — lecture, ou écriture sans effet : rien de neuf si même valeur ET
     * même contexte. On compte. */
    if (g_dvu[mot] && g_dval[mot] == val && g_dctx[mot] == ctx) {
        g_drep[mot]++;
        g_n--;                        /* ne consomme pas le plafond */
        return;
    }

    /* Changement : on résume la série précédente, puis on journalise. */
    if (g_drep[mot] > 1) {
        mbx_ecrire(g_dsens[mot], mot, g_dval[mot], g_dval[mot], g_dctx[mot],
                   fn, insn, g_drep[mot] - 1);
    }
    g_drep[mot]  = 1;
    g_dval[mot]  = val;
    g_dctx[mot]  = ctx;
    g_dsens[mot] = sens;
    g_dvu[mot]   = 1;
    mbx_ecrire(sens, mot, val, avant, ctx, fn, insn, 0);
}

static void mbx_ecrire(CalypsoMbxSens sens, uint16_t mot, uint16_t val,
                       uint16_t avant, uint32_t ctx, uint32_t fn, uint32_t insn,
                       unsigned long rep)
{
    static const char *nom_sens[] = { "ARM>WR", "ARM<RD", "DSP>WR", "DSP<RD" };
    char suffixe[32] = "";

    if (rep) {
        snprintf(suffixe, sizeof(suffixe), "  x%lu", rep + 1);
    }
    if ((g_n & 0x3FF) == 0) {
        fflush(g_f);                  /* une fenêtre de 1024 lignes au pire */
    }
    if (sens == MBX_ARM_WR || sens == MBX_DSP_WR) {
        fprintf(g_f, "%-12u %-8u %-6s 0x%04x %-14s 0x%04x -> 0x%04x  @0x%04x%s\n",
                insn, fn, nom_sens[sens], mot, mbx_nom(mot), avant, val, ctx,
                suffixe);
    } else {
        fprintf(g_f, "%-12u %-8u %-6s 0x%04x %-14s = 0x%04x            @0x%04x%s\n",
                insn, fn, nom_sens[sens], mot, mbx_nom(mot), val, ctx, suffixe);
    }
}
