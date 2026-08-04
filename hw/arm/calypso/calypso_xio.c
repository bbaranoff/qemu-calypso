/*
 * calypso_xio.c — fenêtres XIO du DSP encore non modélisées : API Control (F900)
 * et INTH du DSP (FA00).
 *
 * POURQUOI (2026-08-03).
 * ---------------------
 * Désassemblage de la routine 0xa636-0xa660 (PROM0), celle qui touche au DMA :
 *
 *   0xa637  orm   *(0x3fdc), #0x0004      ; pose le bit 2
 *   0xa63a  portw *(0x3fdc), 0xf900       ; -> XIO:F900  = API Control
 *   0xa640  portr *(0x4356), 0xfc28       ; lit  DMA2_CTRL
 *   0xa643  andm  *(0x4356), #0xfffe      ; EFFACE ENABLE  (desarme, ne scrute pas)
 *   0xa646  portw *(0x4356), 0xfc28       ; reecrit DMA2_CTRL
 *   0xa652  portr *(0x0011), 0xfc28       ; relit DMA2_CTRL
 *   0xa655  st    *(0x4356), #0x0c00
 *   0xa658  portw *(0x4356), 0xfa01       ; -> XIO:FA01  = INTH du DSP
 *   0xa65b  andm  *(0x3fdc), #0xfffb      ; efface le bit 2
 *   0xa65e  portw *(0x3fdc), 0xf900       ; -> XIO:F900
 *
 * L'encadrement par F900 bit 2 est EXACTEMENT la procedure prescrite par la note
 * du §11.3.5 : « DMA1_CTRL is only writable and readable when the DMA controller
 * clock runs [...] it must set the BRIDGE_CLK_EN bit of DSP API configuration
 * register to '1' ». Le firmware la respecte a la lettre.
 *
 * Ces deux fenetres tombaient dans le no-op de PORTR/PORTW — comme SPCR avant ce
 * matin, comme les registres DMA avant tout a l'heure. Le `0x0c00` ecrit dans
 * l'INTH du DSP (§3.7.6 : « Interrupt Handler (INTH) » des peripheriques DSP,
 * distinct de l'IMR du coeur) partait donc au neant.
 *
 * CE MODULE N'INVENTE AUCUNE SEMANTIQUE. Il enregistre, restitue et journalise.
 * On ne sait pas encore ce que valent les bits de l'INTH DSP : le doc en donne la
 * fenetre (§7.2.2, « INTH FA00-FAFF ») mais le detail des champs n'a pas ete
 * localise. On imprime donc la valeur brute et sa decomposition en bits, sans
 * nommer ce qu'on ne sait pas nommer.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "hw/arm/calypso/calypso_xio.h"

#include <stdio.h>
#include <stdlib.h>

#define XIO_APIC_BASE  0xF900   /* API Control  (§7.2.2)          */
#define XIO_APIC_END   0xF9FF
#define XIO_INTH_BASE  0xFA00   /* INTH du DSP  (§7.2.2, §3.7.6)  */
#define XIO_INTH_END   0xFAFF

/* ---- API_CONF @ XIO:F900 — CAL207 §9.1, Table 14 (3 bits R/W) --------------
 *   bit 0            reserve, doit valoir 0
 *   bit 1  API_HOM   0 = API en mode SAM, 1 = API en mode HOM
 *   bit 2  BRIDGE_CLK_EN  1 = force l'horloge du pont ARM-RHEA et du DMA
 * Reset annonce « ???? ???? ???? ?010 », donc API_HOM=1 (HOM) au reset.
 *
 * ⚠️ LES DEUX DOCUMENTS TI SE CONTREDISENT : CAL000 §7.2.1 affirme « SAM mode is
 * the default configuration when the DSP exits from a reset phase », l'inverse.
 * Non tranche. Le firmware ecrit de toute facon API_CONF=0x0000 (SAM) tot au boot
 * (0xb416, 0xb36f), comme s'il ne faisait confiance ni a l'un ni a l'autre.
 *
 * §9.1 sur BRIDGE_CLK_EN : « each time the DSP software want to access to these
 * registers [DMA1_CTRL/DMA2_CTRL], it must set the BRIDGE_CLK_EN bit to '1', then
 * access to these registers and then set back BRIDGE_CLK_EN bit to '0' in order to
 * conserve power. » Le firmware le fait exactement (0xb3b1 -> 0xb3c6).
 *
 * §9.2.3 : le DSP ne peut pas acceder directement a l'APIC ; SMODE et HINT se
 * lisent/ecrivent par les bits 2 et 3 du BSCR (MMR data 0x0029). D'ou le
 * `orm *(0x0029), #0x0004` en 0xa68d, juste avant la bascule de mode. */
#define APIC_HOM           0x0002
#define APIC_BRIDGE_CLK_EN 0x0004

/* ---- INTH du DSP @ XIO:FA00/FA01 — CAL207 §15.2 ---------------------------
 * FA00 CNTRL_REG (13 bits) : assignation front/niveau des 12 canaux.
 *      bits 11:0  CHx  1 = canal x en FRONT, 0 = en NIVEAU
 *      bit 12     INT4 switch : 0 = canal 4 -> INT4N (0x58), 1 -> nNMI (0x04)
 * FA01 CLEAR_REG (12 bits, W) : ecrire 1 dans un bit efface le canal correspondant
 *      (necessaire pour les canaux assignes en NIVEAU et partages).
 * Le canal N correspond a INTNn : verifie par la mesure — le firmware ecrit 0x0380
 * (canaux 7, 8, 9 en front) et le §15.1 marque exactement INT7n, INT8n et INT9n
 * comme « edge ». */
#define INTH_CNTRL_REG  0x00
#define INTH_CLEAR_REG  0x01
#define INTH_INT4_SWITCH 0x1000

static uint16_t apic[0x100];
static uint16_t inth[0x100];
static bool     g_init;

static bool xio_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = calypso_gate("CALYPSO_XIO_MISC", 1);
        if (!on)
            fprintf(stderr, "[xio] CALYPSO_XIO_MISC=0 : F900/FA00 redeviennent "
                    "des no-op (comportement d'avant le 03/08)\n");
    }
    return on != 0;
}

static void xio_init(void)
{
    if (g_init)
        return;
    g_init = true;
    fprintf(stderr, "[xio] fenetres XIO ouvertes : API Control @0xF900 (bit2 = "
            "BRIDGE_CLK_EN, note du §11.3.5) et INTH du DSP @0xFA00 (§3.7.6). "
            "ENREGISTREMENT SEUL : aucune semantique n'est inventee.\n");
}

static void bits16(uint16_t v, char out[24])
{
    int k = 0;
    for (int b = 15; b >= 0; b--) {
        out[k++] = (v & (1u << b)) ? '1' : '0';
        if (b == 12 || b == 8 || b == 4)
            out[k++] = ' ';
    }
    out[k] = 0;
}

/* Mode courant de la RAM API, tel que le DSP l'a programme (§9.1 bit 1). */
bool calypso_xio_api_hom(void)
{
    return (apic[0x00] & APIC_HOM) != 0;
}

bool calypso_xio_misc(bool write, uint16_t pa, uint16_t *val, uint16_t pc)
{
    bool is_apic = (pa >= XIO_APIC_BASE && pa <= XIO_APIC_END);
    bool is_inth = (pa >= XIO_INTH_BASE && pa <= XIO_INTH_END);
    if (!is_apic && !is_inth)
        return false;
    if (!xio_on())
        return false;
    xio_init();

    uint16_t *bank = is_apic ? apic : inth;
    unsigned  off  = pa & 0xFF;
    const char *nom = is_apic ? "API-CTRL" : "INTH-DSP";

    if (!write) {
        *val = bank[off];
        static unsigned nr = 0;
        if (nr++ < 30)
            fprintf(stderr, "[xio] %s PORTR PA=0x%04x -> 0x%04x PC=0x%04x\n",
                    nom, pa, *val, pc);
        return true;
    }

    bank[off] = *val;

    char b[24];
    bits16(*val, b);
    /* [2026-08-03] DEDUPE — la v1 journalisait « toute écriture dont la valeur
     * diffère de la précédente ». Or le firmware fait BASCULER API_CONF entre
     * 0x0002 (HOM) et 0x0000 (SAM) à CHAQUE TRAME : chaque écriture différait
     * donc de la précédente, et la condition était toujours vraie. Résultat
     * mesuré : 2 238 lignes sur 20 000, soit 11 % du journal QEMU, pour deux
     * valeurs qui alternent. On dédoublonne sur le triplet (registre, valeur,
     * PC) : chaque combinaison sort UNE fois, puis un résumé périodique. */
    static struct { uint16_t pa, val, pc; unsigned long long n; } seen[32];
    static int nseen = 0;
    int i, k = -1;
    for (i = 0; i < nseen; i++)
        if (seen[i].pa == pa && seen[i].val == *val && seen[i].pc == pc) { k = i; break; }
    if (k >= 0) {
        if (++seen[k].n % 5000 == 0)
            fprintf(stderr, "[xio] %s PA=0x%04x <- 0x%04x PC=0x%04x × %llu\n",
                    nom, pa, *val, pc, seen[k].n);
        return true;
    }
    if (nseen < 32) { seen[nseen].pa = pa; seen[nseen].val = *val;
                      seen[nseen].pc = pc; seen[nseen].n = 1; nseen++; }
    {
        if (is_apic) {
            fprintf(stderr, "[xio] API_CONF PORTW PA=0x%04x <- 0x%04x [%s] "
                    "API_HOM=%d(%s) BRIDGE_CLK_EN=%d PC=0x%04x\n",
                    pa, *val, b, !!(*val & APIC_HOM),
                    (*val & APIC_HOM) ? "HOM: API reservee ARM/DMA"
                                      : "SAM: acces partage",
                    !!(*val & APIC_BRIDGE_CLK_EN), pc);
        } else {
            if (off == INTH_CNTRL_REG) {
                fprintf(stderr, "[xio] *** INTH CNTRL_REG <- 0x%04x [%s] : canaux en "
                        "FRONT =", *val, b);
                for (int ch = 0; ch <= 11; ch++)
                    if (*val & (1u << ch))
                        fprintf(stderr, " INT%dn", ch);
                fprintf(stderr, " ; INT4_switch=%d (%s) PC=0x%04x\n",
                        !!(*val & INTH_INT4_SWITCH),
                        (*val & INTH_INT4_SWITCH) ? "canal 4 -> nNMI"
                                                  : "canal 4 -> INT4N",
                        pc);
            } else {
                fprintf(stderr, "[xio] *** INTH CLEAR_REG <- 0x%04x [%s] : efface les "
                        "canaux", *val, b);
                for (int ch = 0; ch <= 11; ch++)
                    if (*val & (1u << ch))
                        fprintf(stderr, " INT%dn", ch);
                fprintf(stderr, " PC=0x%04x\n", pc);
            }
        }
    }
    return true;
}
