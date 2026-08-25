/*
 * calypso_c54x_trace.h - entry points of the debug instrumentation that lives
 * in calypso_c54x_trace.c and is still called from the decoder in
 * calypso_c54x.c. These were file-static before the split; nothing else about
 * them changed.
 */

#ifndef CALYPSO_C54X_TRACE_H
#define CALYPSO_C54X_TRACE_H

#include "calypso_c54x.h"

void ar_write_track(C54xState *s, unsigned idx, uint16_t new_val);

void a_track_init_lazy(void);
void a_track_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op);

void ar6_at_init_lazy(void);
void ar6_at_iter(C54xState *s, uint16_t prev_pc, uint16_t prev_op);

void rsbx_intm_check(C54xState *s, uint16_t op);

void sp_abs_track(C54xState *s, uint16_t new_val, uint8_t site);

void stuck_probe_check(C54xState *s);

void int3_cycle_start(C54xState *s, uint16_t target_pc);
void int3_cycle_track_branch(C54xState *s, uint16_t exec_pc,
                             uint16_t exec_op, int consumed);
void int3_cycle_end_good(C54xState *s, uint16_t return_addr);

void rmap_note(uint16_t addr, uint16_t pc);

void scratchwr_note(C54xState *s, uint16_t a, uint16_t v, const char *espace);

void wmap_note(uint16_t addr, uint16_t val, uint16_t pc);

void dio_note(C54xState *s, const char *rw, uint16_t addr, uint16_t val);

/* Defined in calypso_c54x.c. It lost `static` for ar_write_track, which needs
 * the opcode at the current PC to classify a write's provenance. */
uint16_t prog_fetch(C54xState *s, uint16_t pc);

#endif /* CALYPSO_C54X_TRACE_H */
