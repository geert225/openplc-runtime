/**
 * @file profinet_plugin.h
 * @brief PROFINET IO Controller Plugin Interface for OpenPLC Runtime v4
 *
 * Implements one or more PROFINET IO controllers, each on its own network
 * interface. Every controller runs DCP discovery, AR/CR establishment, and
 * cyclic RT data exchange for its configured devices on a dedicated
 * SCHED_FIFO rt thread.
 *
 * Plugin lifecycle:
 *   init()       -> Load config(s), initialize logger [state: IDLE]
 *   start_loop() -> Open sockets, build I/O maps, connect devices, spawn rt
 *                    thread(s) [state: OPERATIONAL]
 *   stop_loop()  -> Stop rt thread(s), release ARs, close sockets [state: STOPPED]
 *   cleanup()    -> Free resources
 *
 * No cycle_start/cycle_end hooks: I/O exchange is fully owned by the
 * per-master rt threads, decoupled from the PLC scan cycle.
 *
 * Commands (execute_command): "status", "diagnostics", "list-interfaces".
 */

#ifndef PROFINET_PLUGIN_H
#define PROFINET_PLUGIN_H

#include <stddef.h>

/**
 * @brief Initialize the PROFINET IO plugin
 *
 * Copies runtime args, initializes the logger, and parses all PROFINET_IO
 * master configurations from the plugin's config file.
 *
 * @param args Pointer to plugin_runtime_args_t (freed after init returns)
 * @return 0 on success, -1 on failure
 */
int init(void *args);

/**
 * @brief Start all configured PROFINET IO controllers
 *
 * Opens each master's raw Ethernet and RPC sockets, builds the I/O channel
 * maps, runs the bounded device connection startup, and spawns the
 * dedicated rt thread.
 *
 * @return 0 if at least one master started successfully, -1 otherwise
 */
int start_loop(void);

/**
 * @brief Stop all running PROFINET IO controllers
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
 *   - "status": per-master and per-device status snapshot
 *   - "diagnostics": detailed per-device connection diagnostics
 *   - "list-interfaces": enumerate available network interfaces
 *
 * @param command_json JSON string with "command" and optional "params" fields
 * @param response Buffer for JSON response
 * @param response_size Size of response buffer
 * @return 0 on success, -1 on error
 */
int execute_command(const char *command_json, char *response, size_t response_size);

#endif /* PROFINET_PLUGIN_H */
