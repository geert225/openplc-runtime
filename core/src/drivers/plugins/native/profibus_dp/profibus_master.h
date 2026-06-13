/**
 * @file profibus_master.h
 * @brief Profibus DP master instance -- per-slave parameterization state
 *        machine, cyclic Data_Exchange, and on-demand DPV1 parameter access
 *
 * One `pb_master_instance_t` represents a single serial bus (master). All
 * access to the serial port and the per-slave FCB/state is serialized
 * through `bus_mutex` (PRIO_INHERIT), so the dedicated bus thread (cyclic
 * Data_Exchange + parameterization) and on-demand DPV1 reads/writes issued
 * via execute_command() can safely interleave.
 *
 * Per-slave status fields (pb_slave_status_t, embedded in
 * pb_slave_runtime_t) are atomics and may be read from any thread without
 * holding bus_mutex.
 */

#ifndef PROFIBUS_MASTER_H
#define PROFIBUS_MASTER_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "profibus_config.h"
#include "profibus_serial.h"
#include "plugin_types.h"
#include "plugin_logger.h"

/**
 * @brief Per-master (per-bus) runtime instance
 *
 * Heap-allocated by the plugin (one per configured master/bus).
 */
typedef struct {
    char name[PB_MAX_NAME_LEN];

    pb_config_t config;

    pb_serial_port_t port;

    pb_slave_runtime_t slaves[PB_MAX_SLAVES];

    _Atomic(int) plugin_state; /* pb_plugin_state_t */
    _Atomic(uint64_t) cycle_counter;

    /* Serializes all serial port I/O and per-slave FCB/state mutation
     * between the bus thread and on-demand DPV1 requests from
     * execute_command(). PRIO_INHERIT to avoid priority inversion against
     * the SCHED_FIFO bus thread. */
    pthread_mutex_t bus_mutex;

    /* Dedicated bus thread: SCHED_FIFO at master.task_priority, periodic
     * at master.cycle_time_us. */
    pthread_t bus_thread;
    _Atomic(bool) bus_running;

#if PB_ENABLE_MONITOR_THREAD
    /* Background monitor thread: periodic diagnostics logging. */
    pthread_t monitor_thread;
    _Atomic(bool) monitor_running;
#endif
} pb_master_instance_t;

/**
 * @brief Initialize a master instance after its config has been parsed.
 *
 * Initializes mutexes, sets up per-slave runtime state (cfg pointers,
 * initial OFFLINE state, next_dpv1_write index), and stores the master's
 * name. Does not open the serial port.
 *
 * @return 0 on success, -1 on mutex init failure.
 */
int pb_master_init(pb_master_instance_t *inst, const char *name, plugin_logger_t *logger);

/**
 * @brief Destroy mutexes. Call after pb_master_close().
 */
void pb_master_destroy(pb_master_instance_t *inst);

/**
 * @brief Open and configure the master's serial port.
 * @return 0 on success, -1 on failure.
 */
int pb_master_open(pb_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Close the master's serial port.
 */
void pb_master_close(pb_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Build the I/O channel map and transfer list for every configured slave.
 *
 * Must be called after pb_master_init() and after the PLC image table
 * pointers in @p args are populated (i.e. during start_loop, after
 * glueVars()).
 *
 * @return 0 on success, -1 if any slave's channel map failed to build.
 */
int pb_master_build_io(pb_master_instance_t *inst, plugin_runtime_args_t *args,
                        plugin_logger_t *logger);

/**
 * @brief Bounded startup loop: drive every slave's parameterization state
 *        machine until it reaches DATA_EXCHANGE or the startup budget is
 *        exhausted.
 *
 * Slaves configured with strict=true must reach DATA_EXCHANGE within the
 * budget or this function fails (the caller should abort master startup).
 * Slaves with strict=false are left in whatever state they reached; the
 * bus thread continues driving them toward DATA_EXCHANGE in the background.
 *
 * @return 0 if all strict slaves reached DATA_EXCHANGE (or there are none),
 *         -1 if a strict slave failed to parameterize within the budget.
 */
int pb_master_startup_slaves(pb_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Run one bus cycle: parameterization step or cyclic Data_Exchange
 *        for every configured slave, plus pending DPV1 initial-value writes.
 *
 * For slaves in DATA_EXCHANGE, copies PLC output variables into the slave's
 * output_data[] (under args->image_lock/unlock), performs the Data_Exchange
 * transaction, and publishes input_data[] into the %I image via the
 * journal. For slaves not yet in DATA_EXCHANGE, advances the
 * parameterization state machine by one step.
 *
 * Acquires bus_mutex internally for each per-slave transaction.
 */
void pb_master_run_cycle(pb_master_instance_t *inst, plugin_runtime_args_t *args,
                         plugin_logger_t *logger);

/**
 * @brief Find a slave's runtime index by its configured station address.
 * @return Index into inst->slaves[]/inst->config.slaves[], or -1 if not found.
 */
int pb_master_find_slave_by_address(const pb_master_instance_t *inst, int station_address);

/**
 * @brief Perform an on-demand DPV1 MSAC1 Read.
 *
 * Thread-safe: acquires bus_mutex for the duration of the transaction, so
 * this may be called concurrently with the bus thread's cyclic operation
 * (e.g. from execute_command()).
 *
 * @return 0 on success (out_data/out_len populated), -1 on communication
 *         or protocol error.
 */
int pb_master_dpv1_read(pb_master_instance_t *inst, int slave_index, uint8_t slot,
                         uint8_t index, uint8_t length, uint8_t *out_data, int *out_len,
                         plugin_logger_t *logger);

/**
 * @brief Perform an on-demand DPV1 MSAC1 Write.
 *
 * Thread-safe: acquires bus_mutex for the duration of the transaction.
 *
 * @return 0 on success, -1 on communication or protocol error.
 */
int pb_master_dpv1_write(pb_master_instance_t *inst, int slave_index, uint8_t slot,
                          uint8_t index, const uint8_t *data, int data_len,
                          plugin_logger_t *logger);

/**
 * @brief Encode a double-precision value into @p length bytes per @p dt.
 *
 * Used to encode pb_dpv1_param_t.initial_value for the startup DPV1 write
 * queue, and to encode execute_command-supplied write values. Native
 * (host) byte order -- adjust channel definitions if a slave's GSD
 * specifies big-endian ("Motorola") byte order for a parameter.
 *
 * @return 0 on success, -1 if @p length does not fit the buffer or exceeds 8.
 */
int pb_value_to_bytes(double value, pb_data_type_t dt, uint8_t length, uint8_t *out);

/**
 * @brief Decode @p length raw bytes into a double per @p dt.
 *
 * Inverse of pb_value_to_bytes(), used to report DPV1 read results.
 */
double pb_bytes_to_value(const uint8_t *data, pb_data_type_t dt, uint8_t length);

#endif /* PROFIBUS_MASTER_H */
