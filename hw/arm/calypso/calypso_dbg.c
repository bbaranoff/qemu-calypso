/*
 * Calypso QEMU runtime debug categories — implementation.
 *
 * Parses CALYPSO_DBG env var once at init and exposes the mask via
 * the global calypso_dbg_mask variable. The DBG() macro in calypso_dbg.h
 * tests the mask before formatting/printing each log line.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hw/arm/calypso/calypso_dbg.h"

uint32_t calypso_dbg_mask = 0;

static const struct {
    const char *name;
    enum calypso_dbg_cat cat;
} cat_table[] = {
    { "bsp",     DBG_BSP },
    { "fb",      DBG_FB },
    { "sp",      DBG_SP },
    { "corrupt", DBG_CORRUPT },
    { "unimpl",  DBG_UNIMPL },
    { "hot",     DBG_HOT },
    { "xpc",     DBG_XPC },
    { "call",    DBG_CALL },
    { "f2",      DBG_F2 },
    { "dump",    DBG_DUMP },
    { "boot",    DBG_BOOT },
    { "l1ctl",   DBG_L1CTL },
    { "trx",     DBG_TRX },
    { "pmst",    DBG_PMST },
    { "rpt",     DBG_RPT },
    { "mvpd",    DBG_MVPD },
    { "inth",    DBG_INTH },
    { "tint0",   DBG_TINT0 },
};
#define N_CATS (sizeof(cat_table) / sizeof(cat_table[0]))

static const uint32_t default_mask =
    (1u << DBG_CORRUPT) | (1u << DBG_UNIMPL);

static int once = 0;

void calypso_dbg_init(void)
{
    if (once) return;
    once = 1;

    const char *env = getenv("CALYPSO_DBG");
    if (!env) {
        calypso_dbg_mask = default_mask;
        fprintf(stderr, "[dbg] CALYPSO_DBG unset → default (corrupt,unimpl)\n");
        return;
    }
    if (!*env || strcmp(env, "none") == 0) {
        calypso_dbg_mask = 0;
        fprintf(stderr, "[dbg] CALYPSO_DBG=none → all silent\n");
        return;
    }
    if (strcmp(env, "all") == 0) {
        calypso_dbg_mask = (1u << DBG__COUNT) - 1;
        fprintf(stderr, "[dbg] CALYPSO_DBG=all → every category enabled\n");
        return;
    }

    /* Parse comma-separated list. */
    char buf[512];
    snprintf(buf, sizeof(buf), "%s", env);
    calypso_dbg_mask = 0;
    char *tok = strtok(buf, ",");
    while (tok) {
        /* trim leading spaces */
        while (*tok == ' ') tok++;
        size_t l = strlen(tok);
        while (l > 0 && (tok[l-1] == ' ' || tok[l-1] == '\n')) tok[--l] = 0;

        int found = 0;
        for (size_t i = 0; i < N_CATS; i++) {
            if (strcasecmp(tok, cat_table[i].name) == 0) {
                calypso_dbg_mask |= (1u << cat_table[i].cat);
                found = 1;
                break;
            }
        }
        if (!found && *tok)
            fprintf(stderr, "[dbg] unknown category '%s'\n", tok);
        tok = strtok(NULL, ",");
    }

    /* Always force corrupt + unimpl on unless explicit "none". */
    calypso_dbg_mask |= default_mask;

    fprintf(stderr, "[dbg] CALYPSO_DBG=%s → mask=0x%08x\n", env, calypso_dbg_mask);
}
