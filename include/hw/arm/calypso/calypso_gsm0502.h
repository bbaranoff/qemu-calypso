/*
 * calypso_gsm0502.h — positions canoniques FCCH / SCH dans le multitrame 51.
 *
 * POURQUOI CE FICHIER. Le predicat « cette trame porte-t-elle la FCCH ? » etait
 * recopie a la main sur QUATRE sites (calypso_bsp.c, calypso_c54x.c,
 * calypso_dsp_shunt.c x2) et les copies avaient DIVERGE : deux d entre elles
 * utilisaient les positions canoniques {0,10,20,30,40}, la troisieme un decalage
 * « +1 » herite d une sonde mal lue, la quatrieme SCH={2,12,...}. Consequence
 * concrete : avec CALYPSO_RIF_FCCH_ONLY=1 le filtre du RIF jetait TOUTES les
 * vraies FCCH et ne poussait que le SCH et la trame d apres. Une seule
 * definition, partagee, rend ce genre de derive impossible.
 *
 * REFERENCE. GSM 05.02 §7 (mapping de BCCH+CCCH sur TS0, multitrame de 51) :
 *     FCCH : fn % 51 ∈ {0, 10, 20, 30, 40}
 *     SCH  : fn % 51 ∈ {1, 11, 21, 31, 41}     (= FCCH + 1)
 * Confirme cote emulateur par la sonde FN-ALIGN (sch%51 ∈ {1,11,21,31,41}).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_GSM0502_H
#define HW_ARM_CALYPSO_GSM0502_H

#include <stdbool.h>
#include <stdint.h>

/* --- positions canoniques (GSM 05.02) --------------------------------- */
static inline bool gsm0502_p51_is_fcch(unsigned p51)
{
    return p51 <= 40 && (p51 % 10) == 0;
}

static inline bool gsm0502_p51_is_sch(unsigned p51)
{
    return p51 <= 41 && (p51 % 10) == 1;
}

static inline bool gsm0502_fn_is_fcch(uint32_t fn)
{
    return gsm0502_p51_is_fcch((unsigned)(fn % 51u));
}

static inline bool gsm0502_fn_is_sch(uint32_t fn)
{
    return gsm0502_p51_is_sch((unsigned)(fn % 51u));
}

/* --- ancienne convention « decalee de +1 » ----------------------------- *
 * Conservee UNIQUEMENT pour le chemin A/B CALYPSO_FEED_FN_CANON=0, qui sert a
 * remesurer l ancien comportement. Ne l utiliser nulle part ailleurs. */
static inline bool gsm0502_p51_is_fcch_legacy_plus1(unsigned p51)
{
    return p51 <= 41 && (p51 % 10) == 1;
}

static inline bool gsm0502_p51_is_sch_legacy_plus1(unsigned p51)
{
    return p51 <= 42 && (p51 % 10) == 2;
}

#endif /* HW_ARM_CALYPSO_GSM0502_H */
