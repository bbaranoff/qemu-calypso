#ifndef HW_ARM_CALYPSO_ORCH_H
#define HW_ARM_CALYPSO_ORCH_H
#include <stdlib.h>
/* CALYPSO_ORCH master flag.
 *   set & non-empty & != "0"  -> ORCHESTRATION ON  (host-side L1 injection:
 *                                BSP FB/SB synth, grgsm SI -> a_cd, FORCE_NB).
 *   unset / empty / "0"        -> EXECUTION (default): the real C54x DSP ROM
 *                                runs its own L1 and writes the NDB itself.
 * Independent of CALYPSO_DSP_SHUNT (the separate full-DSP-replacement path). */
static inline int calypso_orch(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = getenv("CALYPSO_ORCH");
        v = (e && e[0] && e[0] != '0') ? 1 : 0;
    }
    return v;
}
#endif /* HW_ARM_CALYPSO_ORCH_H */
