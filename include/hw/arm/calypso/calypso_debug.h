/*
 * calypso_debug.h — env-gated probe output infrastructure
 *
 * Per la règle "la mesure EST la maladie" : tous les probes diagnostic
 * sont SILENT par défaut. Activation via UNE seule env var :
 *
 *   CALYPSO_DEBUG="probe1,probe2,probe3,..."    — allume liste
 *   CALYPSO_DEBUG=ALL                          — allume tout
 *   CALYPSO_DEBUG="ALL,..."                     — pareil
 *
 * Le nom de probe est normalisé upper-case, avec '-' / ' ' / '.' / '/'
 * remplacés par '_'. Match insensitive : probe "IMR-W" matche entry
 * "imr-w" comme "IMR_W".
 *
 * Cache : la liste est parsée UNE FOIS au premier appel et stockée.
 * Lookup = O(N) sur la liste (= ~50ns pour 10-20 probes typiques).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_DEBUG_H
#define HW_ARM_CALYPSO_DEBUG_H

#include <stdbool.h>
#include <stdio.h>

bool calypso_debug_enabled(const char *probe_name);

/* CALYPSO_DBG : gated fprintf. probe_name est string literal compile-time.
 * fmt inclut le préfixe (ex: "[c54x] IMR-W ...") et le \n final. */
#define CALYPSO_DBG(probe_name, fmt, ...) \
    do { \
        if (calypso_debug_enabled(probe_name)) { \
            fprintf(stderr, fmt, ##__VA_ARGS__); \
        } \
    } while (0)

/* Wrappers par composant — préfixe automatique + \n + gate sur le nom
 * de probe spécifique au composant ou un nom passé. */
#define C54_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[c54x] " fmt "\n", ##__VA_ARGS__)
#define TRX_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[calypso-trx] " fmt "\n", ##__VA_ARGS__)
#define BSP_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[BSP] " fmt "\n", ##__VA_ARGS__)
#define IOTA_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[iota] " fmt "\n", ##__VA_ARGS__)
#define PCB_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[pcb] " fmt "\n", ##__VA_ARGS__)
#define INTH_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[INTH] " fmt "\n", ##__VA_ARGS__)
#define UART_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, fmt "\n", ##__VA_ARGS__)
#define TIMER_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[timer] " fmt "\n", ##__VA_ARGS__)
#define TWL3025_DBG(probe, fmt, ...) \
    CALYPSO_DBG(probe, "[twl3025] " fmt "\n", ##__VA_ARGS__)

/* cdbg_env : shim retro-compat pour les sites qui testaient
 * getenv("CALYPSO_X")=="1". Retourne "1" si le token est actif dans
 * CALYPSO_DEBUG, NULL sinon. Migre un gate getenv vers un token en
 * changeant seulement l'appel getenv -> cdbg_env. */
static inline const char *cdbg_env(const char *token)
{
    return calypso_debug_enabled(token) ? "1" : NULL;
}

#endif /* HW_ARM_CALYPSO_DEBUG_H */
