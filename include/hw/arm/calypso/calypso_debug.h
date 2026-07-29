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

/* Master gate : caché, parsé une fois. Quand CALYPSO_DEBUG est vide,
 * calypso_debug_master == 0 et calypso_debug_enabled() retourne false en
 * inline, SANS aucun appel de fonction (supprime l'overhead per-instruction
 * des 76 call-sites dans la hot-loop DSP). Comportement identique sinon. */
extern int  calypso_debug_master;
void        calypso_debug_master_init(void);
bool        calypso_debug_enabled_(const char *probe_name);   /* impl réelle */

static inline bool calypso_debug_enabled(const char *probe_name)
{
    if (__builtin_expect(calypso_debug_master < 0, 0)) {
        calypso_debug_master_init();
    }
    if (__builtin_expect(calypso_debug_master == 0, 1)) {
        return false;
    }
    return calypso_debug_enabled_(probe_name);
}

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

/* =============================================================================
 *  calypso_gate() — UNE sémantique pour toutes les gardes par environnement
 * =============================================================================
 *
 *  POURQUOI. Le modèle utilisait TROIS idiomes concurrents pour la même
 *  question « cette option est-elle active ? » :
 *
 *      getenv("X") ? 1 : 0     103 sites   X=0 n'y coupe RIEN — seul `unset`
 *      e && *e == '1'           79 sites   X posée vide vaut 0
 *      atoi(getenv("X")) > 0    22 sites
 *
 *  Trois réponses différentes à « X=0 » et à « X= ». La documentation appelle
 *  cela « la première source d'erreur de manipulation », et le terrain lui donne
 *  raison : `CALYPSO_LDK8_SHIFT16` est déclarée VIDE dans opcodes.env, ce qui
 *  sous l'idiome EXISTS la rend ACTIVE — l'inverse de l'intention lisible.
 *
 *  SÉMANTIQUE UNIQUE, celle que tout le monde suppose déjà :
 *
 *      variable absente                      -> `defaut`
 *      "0" · "" · "no" · "off" · "false"     -> 0     (insensible à la casse)
 *      tout le reste                         -> 1
 *
 *  Donc `X=0` coupe TOUJOURS, et `X=1` active toujours. `defaut` rend explicite,
 *  à l'appel, ce que vaut l'absence — l'information qui manquait le plus.
 *
 *  CE QUE CE HELPER N'EST PAS. Il répond par oui ou non. Les variables qui
 *  portent une VALEUR (adresse, longueur, cadence, chemin, liste) gardent leur
 *  `getenv` + `strtoul`/`atoi` : les convertir n'aurait aucun sens.
 * ---------------------------------------------------------------------------- */
int calypso_gate(const char *nom, int defaut);
