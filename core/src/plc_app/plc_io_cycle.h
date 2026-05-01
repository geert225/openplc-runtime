#ifndef OPENPLC_PLC_IO_CYCLE_H
#define OPENPLC_PLC_IO_CYCLE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I/O cycle helpers.
 *
 * Encapsulates the work that has to happen once per scan around the
 * highest-priority task's body:
 *
 *   plc_run_io_cycle_pre()   — runs BEFORE the task body
 *     drain journal, fire plugin cycle_start
 *
 *   plc_run_io_cycle_post()  — runs AFTER the task body
 *     advance __CURRENT_TIME, fire plugin cycle_end, update heartbeat,
 *     increment tick__
 *
 * Both are called inside the image-tables critical section. The
 * fastest task's thread (Phase 6 picks it; ctx->is_fastest_task) calls
 * these around its body. Other task threads just run their bodies
 * without housekeeping.
 *
 * See docs/strucpp-migration/07-runtime-v4-plugin-and-io.md for the
 * topology rationale.
 */
void plc_run_io_cycle_pre(void);
void plc_run_io_cycle_post(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENPLC_PLC_IO_CYCLE_H */
