#ifndef CALYPSO_INVARIANTS_H
#define CALYPSO_INVARIANTS_H
#include <stdbool.h>
/* Manifeste : dump des CALYPSO_* actifs une fois. Gate CALYPSO_INVARIANTS. */
void calypso_manifest_once(void);
/* Invariant : si ok=false, [INVARIANT-FAIL] <tag> une fois. Gate CALYPSO_INVARIANTS (defaut off). */
bool calypso_invariant(const char *tag, bool ok, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
#endif
