/**
 * @file profinet_master.h
 * @brief PROFINET IO Controller instance -- per-device AR state machine,
 *        cyclic RT data exchange, and DCP/RPC connection management
 *
 * One `pn_master_instance_t` represents a single PROFINET IO Controller on
 * one network interface. It owns:
 *   - a raw AF_PACKET socket (profinet_eth.h) for DCP and RT cyclic frames,
 *   - a shared UDP socket bound to PN_RPC_PORT for CL-RPC (AR/CR
 *     establishment, profinet_rpc.h),
 *   - one `pn_device_runtime_t` per configured IO device.
 *
 * A single per-master thread ("rt_thread") drives both:
 *   - the per-device AR state machine (OFFLINE -> DCP_IDENTIFY ->
 *     CONNECTING -> PRM_END -> WAIT_APPL_READY -> DATA_EXCHANGE) for any
 *     device not yet in cyclic data exchange, and
 *   - cyclic RT frame send/receive for every device already in
 *     DATA_EXCHANGE.
 *
 * `io_mutex` (PRIO_INHERIT) serializes all access to `eth`/`rpc_fd` so the
 * rt_thread and any on-demand operations issued via execute_command() do
 * not interleave raw socket I/O.
 */

#ifndef PROFINET_MASTER_H
#define PROFINET_MASTER_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "profinet_config.h"
#include "profinet_eth.h"
#include "profinet_rpc.h"
#include "plugin_types.h"
#include "plugin_logger.h"

/**
 * @brief Per-device live runtime state (AR connection + cyclic data)
 */
typedef struct {
    const pn_device_t *cfg; /* points into pn_config_t.devices[] */

    uint8_t input_data[PN_MAX_IO_DATA_LEN];
    uint8_t output_data[PN_MAX_IO_DATA_LEN];

    pn_channel_map_t   channel_map;
    pn_transfer_list_t transfer_list;

    _Atomic(int) state; /* pn_device_state_t */

    uint8_t mac[6];
    bool    mac_resolved;

    pn_rpc_ctx_t rpc_ctx;

    /* FrameIDs proposed by the controller for this device's IOCRs (see
     * pn_master_init -- best-effort default assignment). */
    uint16_t input_frame_id;
    uint16_t output_frame_id;
    uint16_t rt_cycle_counter;

    int retries;
    int missed_cycles; /* consecutive RT cycles with no input frame received */

    pn_device_status_t status;
} pn_device_runtime_t;

/**
 * @brief Per-master (per-interface) runtime instance
 *
 * Heap-allocated by the plugin (one per configured controller/interface).
 */
typedef struct {
    char name[PN_MAX_NAME_LEN];

    pn_config_t config;

    pn_eth_handle_t eth;
    int rpc_fd; /* shared UDP socket bound to PN_RPC_PORT */

    pn_device_runtime_t devices[PN_MAX_DEVICES];

    _Atomic(int) plugin_state; /* pn_plugin_state_t */
    _Atomic(uint64_t) cycle_counter;

    /* Serializes all raw-socket (eth) and RPC-socket (rpc_fd) I/O between
     * the rt_thread and any on-demand operations from execute_command().
     * PRIO_INHERIT to avoid priority inversion against the SCHED_FIFO
     * rt_thread. */
    pthread_mutex_t io_mutex;

    /* Dedicated RT thread: SCHED_FIFO at master.task_priority, periodic at
     * master.cycle_time_us. Drives AR establishment and cyclic RT exchange. */
    pthread_t rt_thread;
    _Atomic(bool) rt_running;

#if PN_ENABLE_MONITOR_THREAD
    /* Background monitor thread: periodic diagnostics logging. */
    pthread_t monitor_thread;
    _Atomic(bool) monitor_running;
#endif
} pn_master_instance_t;

/**
 * @brief Initialize a master instance after its config has been parsed.
 *
 * Initializes mutexes, sets up per-device runtime state (cfg pointers,
 * initial OFFLINE state, FrameID assignment), and stores the master's name.
 * Does not open any sockets.
 *
 * @return 0 on success, -1 on mutex init failure.
 */
int pn_master_init(pn_master_instance_t *inst, const char *name, plugin_logger_t *logger);

/**
 * @brief Destroy mutexes. Call after pn_master_close().
 */
void pn_master_destroy(pn_master_instance_t *inst);

/**
 * @brief Open the master's raw Ethernet socket and shared RPC (UDP) socket.
 * @return 0 on success, -1 on failure.
 */
int pn_master_open(pn_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Close the master's sockets.
 */
void pn_master_close(pn_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Build the I/O channel map and transfer list for every configured device.
 *
 * Must be called after pn_master_init() and after the PLC image table
 * pointers in @p args are populated (i.e. during start_loop, after
 * glueVars()).
 *
 * @return 0 on success, -1 if any device's channel map failed to build.
 */
int pn_master_build_io(pn_master_instance_t *inst, plugin_runtime_args_t *args,
                        plugin_logger_t *logger);

/**
 * @brief Bounded startup loop: drive every device's AR state machine until
 *        it reaches DATA_EXCHANGE or the startup budget is exhausted.
 *
 * Devices configured with strict=true must reach DATA_EXCHANGE within the
 * budget or this function fails (the caller should abort master startup).
 * Devices with strict=false are left in whatever state they reached; the
 * rt thread continues driving them toward DATA_EXCHANGE in the background.
 *
 * @return 0 if all strict devices reached DATA_EXCHANGE (or there are none),
 *         -1 if a strict device failed to connect within the budget.
 */
int pn_master_startup_devices(pn_master_instance_t *inst, plugin_logger_t *logger);

/**
 * @brief Run one RT cycle.
 *
 * For every device not yet in DATA_EXCHANGE, advances the AR state machine
 * by one step (DCP Identify / Connect.req / IODControlReq variants -- each
 * bounded by a short timeout).
 *
 * For every device in DATA_EXCHANGE: copies PLC output variables into the
 * device's output_data[] (under args->image_lock/unlock), sends the output
 * RT frame, drains and dispatches received RT input frames, and publishes
 * input_data[] into the %I image via the journal. Devices that miss too
 * many consecutive cyclic frames are dropped back to OFFLINE (AR released
 * best-effort).
 *
 * Acquires io_mutex internally for raw-socket and RPC-socket access.
 */
void pn_master_run_cycle(pn_master_instance_t *inst, plugin_runtime_args_t *args,
                          plugin_logger_t *logger);

/**
 * @brief Find a device's runtime index by its configured name.
 * @return Index into inst->devices[]/inst->config.devices[], or -1 if not found.
 */
int pn_master_find_device_by_name(const pn_master_instance_t *inst, const char *name);

#endif /* PROFINET_MASTER_H */
