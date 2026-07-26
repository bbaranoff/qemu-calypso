/*
 * calypso_invariants.c — screaming invariants + run manifest.
 *
 * Philosophy (2026-07-25) : internal probes raise RESOLUTION, not INDEPENDENCE
 * — a hole in the BSP model is a hole in the BSP probe. So this file holds two
 * things that DO earn their keep :
 *   1. a run manifest (every active CALYPSO_* forcing, logged once) ;
 *   2. invariants that scream on their own and act as a non-regression suite.
 * External TRUTH (is the DSP output valid GSM?) is a separate matter and cannot
 * come from this code — that is GSMTAP -> a decoder we did not write.
 */
#include "qemu/osdep.h"
#include "hw/arm/calypso/calypso_invariants.h"
#include <stdarg.h>


#define INV_MAX 64
static struct { const char *tag; unsigned fails; } inv_tab[INV_MAX];
static int inv_n;
static int inv_enabled = -1;

static int inv_on(void)
{
    if (inv_enabled < 0) {
        const char *e = getenv("CALYPSO_INVARIANTS");
        inv_enabled = (e && e[0] == '1') ? 1 : 0;   /* DEFAUT OFF (securite boot) */
    }
    return inv_enabled;
}

static void inv_summary(void)
{
    if (!inv_n) {
        fprintf(stderr, "[INVARIANT-SUMMARY] all invariants held this run\n");
        return;
    }
    for (int i = 0; i < inv_n; i++) {
        fprintf(stderr, "[INVARIANT-SUMMARY] %s : %u violation(s)\n",
                inv_tab[i].tag, inv_tab[i].fails);
    }
    fflush(stderr);
}

bool calypso_invariant(const char *tag, bool ok, const char *fmt, ...)
{
    if (ok || !inv_on()) {
        return ok;
    }
    int idx = -1;
    for (int i = 0; i < inv_n; i++) {
        if (!strcmp(inv_tab[i].tag, tag)) { idx = i; break; }
    }
    if (idx < 0 && inv_n < INV_MAX) {
        idx = inv_n;
        inv_tab[inv_n].tag = tag;
        inv_tab[inv_n].fails = 0;
        if (inv_n == 0) {
            atexit(inv_summary);
        }
        inv_n++;
    }
    unsigned n = (idx >= 0) ? (++inv_tab[idx].fails) : 1;
    if (n == 1) {                       /* scream ONCE per tag */
        char buf[256];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        fprintf(stderr, "[INVARIANT-FAIL] %s : %s\n", tag, buf);
        fflush(stderr);
    }
    return ok;
}

void calypso_manifest_once(void)
{
    if (!inv_on()) { return; }   /* gate defaut OFF */
    static int done;
    if (done) {
        return;
    }
    done = 1;
    fprintf(stderr, "[MANIFEST] ===== run manifest : active CALYPSO_* forcings =====\n");
    int n = 0;
    for (char **e = environ; e && *e; e++) {
        if (!strncmp(*e, "CALYPSO_", 8)) {
            fprintf(stderr, "[MANIFEST] %s\n", *e);
            n++;
        }
    }
    fprintf(stderr, "[MANIFEST] ===== %d CALYPSO_* variables (see artifact appendix) =====\n", n);
    fflush(stderr);
}
