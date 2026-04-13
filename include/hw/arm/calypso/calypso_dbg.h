/*
 * Calypso QEMU runtime debug categories.
 *
 * Each tracer in the calypso modules is gated by a category bit. The
 * active set is controlled by the CALYPSO_DBG environment variable, a
 * comma-separated list of category names (or "all", or "none"). If the
 * env var is unset, only the "always-on" categories (CORRUPT, UNIMPL)
 * are enabled.
 *
 * Example:
 *   CALYPSO_DBG=bsp,fb,sp           # 3 categories
 *   CALYPSO_DBG=all                 # everything
 *   CALYPSO_DBG=none                # nothing (even corrupt suppressed)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_ARM_CALYPSO_DBG_H
#define HW_ARM_CALYPSO_DBG_H

#include <stdint.h>
#include <stdio.h>

enum calypso_dbg_cat {
    DBG_BSP = 0,    /* BSP DMA, BSP-RD, BSP-ENV-CHECK */
    DBG_FB,         /* FB-DETECT-*, FB DETECTED */
    DBG_SP,         /* SP-WEDGE, SP-STEP, SP MMR writes */
    DBG_CORRUPT,    /* NDB d_dsp_page / d_fb_det wrong writes (default ON) */
    DBG_UNIMPL,     /* unimplemented opcodes (default ON) */
    DBG_HOT,        /* HOT-LOOP-PC */
    DBG_XPC,        /* XPC switches, far call/branch */
    DBG_CALL,       /* CALL/RET balance */
    DBG_F2,         /* F2xx undocumented opcode tracer */
    DBG_DUMP,       /* one-shot DUMP-XXXX */
    DBG_BOOT,       /* boot ROM, init, ROM->DARAM transitions */
    DBG_L1CTL,      /* L1CTL socket */
    DBG_TRX,        /* TRX/sercomm/UDP */
    DBG_PMST,       /* PMST changes */
    DBG_RPT,        /* RPT/RPTB tracking */
    DBG_MVPD,       /* MVPD copies */
    DBG_INTH,       /* interrupt handler */
    DBG_TINT0,      /* TINT0 master clock */
    DBG__COUNT
};

extern uint32_t calypso_dbg_mask;

void calypso_dbg_init(void);

#define DBG(cat, fmt, ...) do { \
    if (calypso_dbg_mask & (1u << (cat))) \
        fprintf(stderr, "[" #cat "] " fmt "\n", ##__VA_ARGS__); \
} while (0)

#define DBG_ON(cat) (calypso_dbg_mask & (1u << (cat)))

#endif /* HW_ARM_CALYPSO_DBG_H */
