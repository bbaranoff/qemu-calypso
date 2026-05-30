/*
 * calypso_debug.c — env-gated probe lookup implementation
 *
 * Une seule env : CALYPSO_DEBUG="probe1,probe2,probe3,..."
 * Valeur spéciale : "ALL" (ou contenant "ALL") active tout.
 *
 * Probes lookup : O(N) sur la liste parsée une fois au premier appel.
 * Pour 10-20 probes typiquement actifs, c'est négligeable.
 *
 * Normalisation : nom de probe upper-case, '-'/' '/'.'/'/'  → '_'.
 * Ex : "IMR-W" matche entry "IMR_W" comme "imr-w".
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_debug.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <pthread.h>

#define MAX_ENTRIES   128
#define NAME_MAX_     64

static char     s_entries[MAX_ENTRIES][NAME_MAX_];
static int      s_entries_n = 0;
static bool     s_all = false;
static bool     s_inited = false;
static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;

/* Master gate : -1 = pas encore init, 0 = CALYPSO_DEBUG vide (toutes sondes
 * OFF, fast-path inliné dans le header), 1 = au moins une sonde active. */
int calypso_debug_master = -1;

/* Normalize probe name in-place : upper-case, separators → '_'. */
static void normalize(char *s)
{
    for (; *s; s++) {
        char c = *s;
        if (c == '-' || c == ' ' || c == '/' || c == '.') c = '_';
        *s = toupper((unsigned char)c);
    }
}

static void parse_env_locked(void)
{
    if (s_inited) return;
    s_inited = true;

    const char *e = getenv("CALYPSO_DEBUG");
    if (!e || !*e) return;

    /* Walk comma-separated tokens. */
    const char *p = e;
    while (*p && s_entries_n < MAX_ENTRIES) {
        /* Skip leading separators (= comma, space, tab). */
        while (*p == ',' || *p == ' ' || *p == '\t') p++;
        if (!*p) break;
        const char *start = p;
        while (*p && *p != ',') p++;
        size_t len = p - start;
        if (len == 0) continue;
        /* Trim trailing whitespace. */
        while (len > 0 && (start[len-1] == ' ' || start[len-1] == '\t')) len--;
        if (len >= NAME_MAX_) len = NAME_MAX_ - 1;
        memcpy(s_entries[s_entries_n], start, len);
        s_entries[s_entries_n][len] = '\0';
        normalize(s_entries[s_entries_n]);
        if (strcmp(s_entries[s_entries_n], "ALL") == 0) {
            s_all = true;
        }
        s_entries_n++;
    }

    fprintf(stderr,
        "[calypso-debug] CALYPSO_DEBUG parsed: %d probe(s), ALL=%d\n",
        s_entries_n, s_all);
    for (int i = 0; i < s_entries_n; i++) {
        fprintf(stderr, "[calypso-debug]   - %s\n", s_entries[i]);
    }
}

/* Init unique du master gate : parse l'env et fixe calypso_debug_master.
 * Appelé depuis l'inline calypso_debug_enabled() du header au 1er passage. */
void calypso_debug_master_init(void)
{
    pthread_mutex_lock(&s_mu);
    parse_env_locked();
    calypso_debug_master = (s_all || s_entries_n > 0) ? 1 : 0;
    pthread_mutex_unlock(&s_mu);
}

/* Impl réelle (out-of-line). N'est atteinte que quand master == 1, donc
 * parse_env_locked a déjà tourné — le bloc !s_inited reste par sûreté. */
bool calypso_debug_enabled_(const char *probe_name)
{
    if (!probe_name) return false;

    if (!s_inited) {
        pthread_mutex_lock(&s_mu);
        parse_env_locked();
        pthread_mutex_unlock(&s_mu);
    }

    if (s_all) return true;
    if (s_entries_n == 0) return false;

    /* Normalize probe_name into local buffer for compare. */
    char norm[NAME_MAX_];
    size_t n = strlen(probe_name);
    if (n >= NAME_MAX_) n = NAME_MAX_ - 1;
    memcpy(norm, probe_name, n);
    norm[n] = '\0';
    normalize(norm);

    for (int i = 0; i < s_entries_n; i++) {
        if (strcmp(s_entries[i], norm) == 0) return true;
    }
    return false;
}
