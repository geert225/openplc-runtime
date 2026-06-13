/**
 * @file profibus_plugin.h
 * @brief Profibus DP Plugin Interface for OpenPLC Runtime v4
 *
 * Implements one or more Profibus DP masters, each on its own RS-485
 * serial port. Every master runs cyclic Data_Exchange and slave
 * parameterization on a dedicated SCHED_FIFO bus thread; on-demand DPV1
 * parameter reads/writes are issued via execute_command and serialized
 * against the bus thread through a per-master mutex.
 *
 * Plugin lifecycle:
 *   init()       -> Load config(s), initialize logger [state: IDLE]
 *   start_loop() -> Open serial port(s), parameterize slaves, spawn bus
 *                    thread(s) [state: OPERATIONAL]
 *   stop_loop()  -> Stop bus thread(s), close serial port(s) [state: STOPPED]
 *   cleanup()    -> Free resources
 *
 * No cycle_start/cycle_end hooks: I/O exchange is fully owned by the
 * per-master bus threads, decoupled from the PLC scan cycle.
 *
 * Commands (execute_command): "status", "diagnostics", "list-ports",
 * "dpv1-read", "dpv1-write".
 */

#ifndef PROFIBUS_PLUGIN_H
#define PROFIBUS_PLUGIN_H

#include <stddef.h>

/**
 * @brief Initialize the Profibus DP plugin
 *
 * Copies runtime args, initializes the logger, and parses all
 * PROFIBUS_DP master configurations from the plugin's config file.
 *
 * @param args Pointer to plugin_runtime_args_t (freed after init returns)
 * @return 0 on success, -1 on failure
 */
int init(void *args);

/**
 * @brief Start all configured Profibus DP masters
 *
 * Opens each master's serial port, builds the I/O channel maps, runs the
 * bounded slave parameterization startup, and spawns the dedicated bus
 * thread.
 *
 * @return 0 if at least one master started successfully, -1 otherwise
 */
int start_loop(void);

/**
 * @brief Stop all running Profibus DP masters
 */
void stop_loop(void);

/**
 * @brief Cleanup plugin resources
 */
void cleanup(void);

/**
 * @brief Execute an async command
 *
 * Supported commands:
 *   - "status": per-master and per-slave status snapshot
 *   - "diagnostics": detailed per-slave diagnostic flags and counters
 *   - "list-ports": enumerate available serial devices
 *   - "dpv1-read": { "master": "<name>", "station_address": N, "slot": N,
 *                     "index": N, "length": N }
 *   - "dpv1-write": { "master": "<name>", "station_address": N, "slot": N,
 *                      "index": N, "value": <number>, "data_type": "<type>",
 *                      "length": N }
 *
 * @param command_json JSON string with "command" and optional "params" fields
 * @param response Buffer for JSON response
 * @param response_size Size of response buffer
 * @return 0 on success, -1 on error
 */
int execute_command(const char *command_json, char *response, size_t response_size);

#endif /* PROFIBUS_PLUGIN_H */
