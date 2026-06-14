/**
 * @file profinet_master.c
 * @brief PROFINET IO Controller instance -- per-device AR state machine,
 *        cyclic RT data exchange, and DCP/RPC connection management
 */

#include "profinet_master.h"

#include "profinet_dcp.h"
#include "profinet_rt.h"
#include "profinet_io.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

/*
 * =============================================================================
 * Timeouts and Retry Limits
 *
 * These bound each step of the AR establishment state machine and the
 * steady-state cyclic watchdog. As with profinet_rpc.c, exact values are
 * best-effort defaults, not validated against a certified PROFINET IO
 * device -- adjust if a specific device requires longer connect/response
 * windows.
 * =============================================================================
 */

#define PN_DCP_TIMEOUT_MS            1000
#define PN_RPC_CONNECT_TIMEOUT_MS    2000
#define PN_RPC_CONTROL_TIMEOUT_MS    1000
#define PN_RPC_APPL_READY_TIMEOUT_MS 2000
#define PN_RPC_RELEASE_TIMEOUT_MS     500

/* Strict devices are escalated to PN_DEV_FAULT after this many consecutive
 * AR-establishment failures. */
#define PN_DEVICE_MAX_RETRIES 5

/* First RT receive of each cycle blocks up to this long; subsequent drains
 * within the same cycle are non-blocking (timeout 0). */
#define PN_RT_RECV_TIMEOUT_MS 2

/*
 * =============================================================================
 * Mutex Init Helper (PTHREAD_PRIO_INHERIT)
 * =============================================================================
 */

static int pn_mutex_init_pi(pthread_mutex_t *m)
{
    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0)
        return -1;
#if !defined(__CYGWIN__) && !defined(_WIN32)
    (void)pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
#endif
    int rc = pthread_mutex_init(m, &attr);
    pthread_mutexattr_destroy(&attr);
    return rc;
}

/*
 * =============================================================================
 * Lifecycle
 * =============================================================================
 */

int pn_master_init(pn_master_instance_t *inst, const char *name, plugin_logger_t *logger)
{
    memset(inst->name, 0, sizeof(inst->name));
    strncpy(inst->name, name, sizeof(inst->name) - 1);

    atomic_store(&inst->plugin_state, PN_STATE_IDLE);
    atomic_store(&inst->cycle_counter, 0);
    atomic_store(&inst->rt_running, false);
#if PN_ENABLE_MONITOR_THREAD
    atomic_store(&inst->monitor_running, false);
#endif

    inst->rpc_fd = -1;
    memset(&inst->eth, 0, sizeof(inst->eth));

    if (pn_mutex_init_pi(&inst->io_mutex) != 0) {
        plugin_logger_error(logger, "Master '%s': failed to initialize io mutex", inst->name);
        return -1;
    }

    for (int i = 0; i < inst->config.device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        memset(drt, 0, sizeof(*drt));

        drt->cfg = &inst->config.devices[i];
        atomic_store(&drt->state, PN_DEV_OFFLINE);

        /* Best-effort controller-proposed FrameIDs for this device's IOCRs
         * (see profinet_rpc.c's caveat about unvalidated block values). */
        drt->output_frame_id = (uint16_t)(0x8000 + 2 * i);
        drt->input_frame_id  = (uint16_t)(0x8000 + 2 * i + 1);

        if (drt->cfg->mac_configured) {
            memcpy(drt->mac, drt->cfg->mac, 6);
            drt->mac_resolved = true;
        }

        strncpy(drt->status.name, drt->cfg->name, PN_MAX_NAME_LEN - 1);
        strncpy(drt->status.station_name, drt->cfg->station_name, PN_MAX_STATION_NAME_LEN - 1);
        atomic_store(&drt->status.state, PN_DEV_OFFLINE);
        atomic_store(&drt->status.cycle_count, 0);
        atomic_store(&drt->status.error_count, 0);
        atomic_store(&drt->status.reconnect_count, 0);
        atomic_store(&drt->status.input_iops_good, 0);
    }

    return 0;
}

void pn_master_destroy(pn_master_instance_t *inst)
{
    pthread_mutex_destroy(&inst->io_mutex);
}

int pn_master_open(pn_master_instance_t *inst, plugin_logger_t *logger)
{
    if (pn_eth_open(&inst->eth, inst->config.master.interface) != 0) {
        plugin_logger_error(logger, "Master '%s': failed to open interface '%s': %s", inst->name,
                             inst->config.master.interface, strerror(errno));
        return -1;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        plugin_logger_error(logger, "Master '%s': failed to create RPC socket: %s", inst->name,
                             strerror(errno));
        pn_eth_close(&inst->eth);
        return -1;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PN_RPC_PORT);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        plugin_logger_error(logger, "Master '%s': failed to bind RPC socket to UDP port %d: %s",
                             inst->name, PN_RPC_PORT, strerror(errno));
        close(fd);
        pn_eth_close(&inst->eth);
        return -1;
    }

    inst->rpc_fd = fd;
    return 0;
}

void pn_master_close(pn_master_instance_t *inst, plugin_logger_t *logger)
{
    (void)logger;

    if (inst->rpc_fd >= 0) {
        close(inst->rpc_fd);
        inst->rpc_fd = -1;
    }
    pn_eth_close(&inst->eth);
}

int pn_master_build_io(pn_master_instance_t *inst, plugin_runtime_args_t *args,
                        plugin_logger_t *logger)
{
    int errors = 0;

    for (int i = 0; i < inst->config.device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];

        if (pn_io_build_channel_map(drt->cfg, &drt->channel_map, args, logger) != 0) {
            plugin_logger_error(logger, "Master '%s': device '%s' channel map build failed",
                                 inst->name, drt->cfg->name);
            errors++;
            continue;
        }

        if (pn_io_build_transfer_list(&drt->channel_map, &drt->transfer_list, args, logger) !=
            0) {
            plugin_logger_error(logger, "Master '%s': device '%s' transfer list build failed",
                                 inst->name, drt->cfg->name);
            errors++;
        }
    }

    return (errors > 0) ? -1 : 0;
}

/*
 * =============================================================================
 * Per-Device AR State Machine
 * =============================================================================
 */

static inline void set_device_state(pn_device_runtime_t *drt, pn_device_state_t state)
{
    atomic_store(&drt->state, (int)state);
    atomic_store(&drt->status.state, (int)state);
}

/**
 * AR establishment step failed -- restart from OFFLINE (which re-resolves
 * the MAC via DCP and re-runs Connect.req/IODControlReq). Devices marked
 * strict are escalated to the terminal PN_DEV_FAULT state after
 * PN_DEVICE_MAX_RETRIES consecutive failures so the rt thread stops
 * retrying and spamming warnings for this device; non-strict devices retry
 * indefinitely.
 */
static void device_connect_failed(pn_device_runtime_t *drt, plugin_logger_t *logger,
                                   const char *step)
{
    atomic_fetch_add(&drt->status.error_count, 1);
    drt->retries++;

    if (drt->cfg->strict && drt->retries >= PN_DEVICE_MAX_RETRIES) {
        plugin_logger_error(logger,
            "Device '%s': %s failed after %d attempt(s), marking FAULT (no further automatic "
            "retries)",
            drt->cfg->name, step, drt->retries);
        set_device_state(drt, PN_DEV_FAULT);
        drt->retries = 0;
        return;
    }

    plugin_logger_warn(logger, "Device '%s': %s failed, retrying", drt->cfg->name, step);
    set_device_state(drt, PN_DEV_OFFLINE);
}

/**
 * @brief Advance one device's AR state machine by one step.
 *
 * Each non-terminal state performs at most one blocking DCP/RPC operation
 * (bounded by the PN_*_TIMEOUT_MS constants above) under io_mutex.
 * PN_DEV_DATA_EXCHANGE and PN_DEV_FAULT are handled elsewhere (cyclic phase
 * and terminal, respectively).
 */
static void pn_device_step(pn_master_instance_t *inst, pn_device_runtime_t *drt,
                            plugin_logger_t *logger)
{
    pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->state);

    switch (state) {
    case PN_DEV_OFFLINE:
        set_device_state(drt, PN_DEV_DCP_IDENTIFY);
        break;

    case PN_DEV_DCP_IDENTIFY: {
        if (drt->cfg->mac_configured) {
            memcpy(drt->mac, drt->cfg->mac, 6);
            drt->mac_resolved = true;
            set_device_state(drt, PN_DEV_CONNECTING);
            break;
        }

        pn_dcp_device_info_t info;
        pthread_mutex_lock(&inst->io_mutex);
        int rc = pn_dcp_identify_by_name(&inst->eth, drt->cfg->station_name, &info,
                                          PN_DCP_TIMEOUT_MS);
        pthread_mutex_unlock(&inst->io_mutex);

        if (rc == 1) {
            memcpy(drt->mac, info.mac, 6);
            drt->mac_resolved = true;
            plugin_logger_info(logger,
                "Device '%s': resolved MAC %02X:%02X:%02X:%02X:%02X:%02X via DCP Identify",
                drt->cfg->name, drt->mac[0], drt->mac[1], drt->mac[2], drt->mac[3], drt->mac[4],
                drt->mac[5]);
            set_device_state(drt, PN_DEV_CONNECTING);
        } else {
            device_connect_failed(drt, logger, "DCP Identify");
        }
        break;
    }

    case PN_DEV_CONNECTING: {
        if (pn_rpc_ctx_init(&drt->rpc_ctx, inst->rpc_fd, drt->cfg->ip_address,
                             drt->input_frame_id, drt->output_frame_id) != 0) {
            device_connect_failed(drt, logger, "RPC context init (invalid ip_address)");
            break;
        }

        pthread_mutex_lock(&inst->io_mutex);
        int rc = pn_rpc_connect(&drt->rpc_ctx, inst->eth.mac, &inst->config.master, drt->cfg,
                                 PN_RPC_CONNECT_TIMEOUT_MS);
        pthread_mutex_unlock(&inst->io_mutex);

        if (rc == 0) {
            plugin_logger_info(logger, "Device '%s': AR established (Connect.res OK)",
                                drt->cfg->name);
            set_device_state(drt, PN_DEV_PRM_END);
        } else {
            device_connect_failed(drt, logger, "Connect.req");
        }
        break;
    }

    case PN_DEV_PRM_END: {
        pthread_mutex_lock(&inst->io_mutex);
        int rc = pn_rpc_control_prmend(&drt->rpc_ctx, PN_RPC_CONTROL_TIMEOUT_MS);
        pthread_mutex_unlock(&inst->io_mutex);

        if (rc == 0) {
            plugin_logger_debug(logger, "Device '%s': IODControlReq(PrmEnd) acknowledged",
                                 drt->cfg->name);
            set_device_state(drt, PN_DEV_WAIT_APPL_READY);
        } else {
            device_connect_failed(drt, logger, "IODControlReq(PrmEnd)");
        }
        break;
    }

    case PN_DEV_WAIT_APPL_READY: {
        pthread_mutex_lock(&inst->io_mutex);
        int rc = pn_rpc_wait_application_ready(&drt->rpc_ctx, PN_RPC_APPL_READY_TIMEOUT_MS);
        pthread_mutex_unlock(&inst->io_mutex);

        if (rc == 1) {
            drt->retries = 0;
            drt->missed_cycles = 0;
            drt->rt_cycle_counter = 0;
            memset(drt->input_data, 0, sizeof(drt->input_data));
            memset(drt->output_data, 0, sizeof(drt->output_data));
            plugin_logger_info(logger,
                "Device '%s': [state: DATA_EXCHANGE] cyclic RT exchange started", drt->cfg->name);
            set_device_state(drt, PN_DEV_DATA_EXCHANGE);
        } else if (rc == 0) {
            device_connect_failed(drt, logger, "ApplicationReady (timeout)");
        } else {
            device_connect_failed(drt, logger, "ApplicationReady");
        }
        break;
    }

    case PN_DEV_DATA_EXCHANGE:
    case PN_DEV_FAULT:
    default:
        /* DATA_EXCHANGE is driven by pn_master_run_cycle's cyclic phase;
         * FAULT is terminal -- no automatic recovery. */
        break;
    }
}

int pn_master_startup_devices(pn_master_instance_t *inst, plugin_logger_t *logger)
{
    int rounds = (PN_DEVICE_MAX_RETRIES + 1) * 5;

    for (int round = 0; round < rounds; round++) {
        bool all_done = true;

        for (int i = 0; i < inst->config.device_count; i++) {
            pn_device_runtime_t *drt = &inst->devices[i];
            pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->state);

            if (state != PN_DEV_DATA_EXCHANGE && state != PN_DEV_FAULT) {
                pn_device_step(inst, drt, logger);
                all_done = false;
            }
        }

        if (all_done)
            break;
    }

    int failed = 0;
    for (int i = 0; i < inst->config.device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        if (!drt->cfg->strict)
            continue;

        pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->state);
        if (state != PN_DEV_DATA_EXCHANGE) {
            plugin_logger_error(logger,
                "Master '%s': device '%s' is marked strict and failed to reach DATA_EXCHANGE "
                "during startup (state=%s)",
                inst->name, drt->cfg->name, pn_device_state_to_string(state));
            failed++;
        }
    }

    return (failed > 0) ? -1 : 0;
}

/*
 * =============================================================================
 * Cyclic RT Exchange
 * =============================================================================
 */

void pn_master_run_cycle(pn_master_instance_t *inst, plugin_runtime_args_t *args,
                          plugin_logger_t *logger)
{
    int device_count = inst->config.device_count;

    /* Phase 1: advance the AR state machine for devices not yet exchanging
     * cyclic data (one bounded DCP/RPC step each). */
    for (int i = 0; i < device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->state);
        if (state != PN_DEV_DATA_EXCHANGE && state != PN_DEV_FAULT)
            pn_device_step(inst, drt, logger);
    }

    bool received[PN_MAX_DEVICES] = { false };

    pthread_mutex_lock(&inst->io_mutex);

    /* Phase 2: copy PLC outputs into each operational device's output_data[]
     * and send its output RT frame. */
    for (int i = 0; i < device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        if ((pn_device_state_t)atomic_load(&drt->state) != PN_DEV_DATA_EXCHANGE)
            continue;

        if (args->image_lock && args->image_unlock) {
            args->image_lock();
            pn_io_write_outputs_fast(&drt->transfer_list, drt->output_data);
            args->image_unlock();
        } else {
            pn_io_write_outputs_fast(&drt->transfer_list, drt->output_data);
        }

        pn_rt_send(&inst->eth, drt->mac, drt->output_frame_id, drt->output_data,
                   drt->cfg->output_length, drt->rt_cycle_counter, PN_DATA_STATUS_GOOD);
    }

    /* Phase 3: drain pending RT frames on the shared raw socket and dispatch
     * each one to the device whose input FrameID it matches. The first
     * receive of the cycle blocks briefly to give devices a chance to
     * respond; subsequent drains are non-blocking. */
    uint8_t rxbuf[PN_ETH_MAX_PAYLOAD];
    int max_iters = device_count * 2 + 4;
    for (int iter = 0; iter < max_iters; iter++) {
        int timeout_ms = (iter == 0) ? PN_RT_RECV_TIMEOUT_MS : 0;
        int n = pn_eth_recv(&inst->eth, NULL, rxbuf, sizeof(rxbuf), timeout_ms);
        if (n <= 0)
            break;
        if (n < 2)
            continue;

        uint16_t frame_id = (uint16_t)((rxbuf[0] << 8) | rxbuf[1]);

        for (int i = 0; i < device_count; i++) {
            pn_device_runtime_t *drt = &inst->devices[i];
            if ((pn_device_state_t)atomic_load(&drt->state) != PN_DEV_DATA_EXCHANGE)
                continue;
            if (frame_id != drt->input_frame_id)
                continue;

            pn_rt_frame_t frame;
            if (pn_rt_decode(rxbuf, n, drt->cfg->input_length, &frame) == 0) {
                if (drt->cfg->input_length > 0)
                    memcpy(drt->input_data, frame.data, (size_t)drt->cfg->input_length);
                atomic_store(&drt->status.input_iops_good,
                             (frame.data_status & PN_DATA_STATUS_DATA_VALID) ? 1 : 0);
                received[i] = true;
            }
            break;
        }
    }

    pthread_mutex_unlock(&inst->io_mutex);

    /* Phase 4: publish received inputs into the %I image, advance per-device
     * cycle counters, and apply the AR watchdog for devices that missed too
     * many consecutive cyclic frames. */
    for (int i = 0; i < device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        if ((pn_device_state_t)atomic_load(&drt->state) != PN_DEV_DATA_EXCHANGE)
            continue;

        drt->rt_cycle_counter++;

        if (received[i]) {
            drt->missed_cycles = 0;
            atomic_fetch_add(&drt->status.cycle_count, 1);
            pn_io_read_inputs_fast(&drt->transfer_list, drt->input_data, args);
            continue;
        }

        drt->missed_cycles++;
        atomic_fetch_add(&drt->status.error_count, 1);

        int watchdog = drt->cfg->watchdog_factor;
        if (watchdog <= 0)
            watchdog = 100;

        if (drt->missed_cycles >= watchdog) {
            plugin_logger_warn(logger,
                "Device '%s': lost cyclic data after %d missed cycle(s), releasing AR and "
                "re-connecting",
                drt->cfg->name, drt->missed_cycles);

            pthread_mutex_lock(&inst->io_mutex);
            pn_rpc_release(&drt->rpc_ctx, PN_RPC_RELEASE_TIMEOUT_MS);
            pthread_mutex_unlock(&inst->io_mutex);

            atomic_fetch_add(&drt->status.reconnect_count, 1);
            drt->retries = 0;
            drt->missed_cycles = 0;
            set_device_state(drt, PN_DEV_OFFLINE);
        }
    }

    atomic_fetch_add(&inst->cycle_counter, 1);
}

int pn_master_find_device_by_name(const pn_master_instance_t *inst, const char *name)
{
    for (int i = 0; i < inst->config.device_count; i++) {
        if (strcmp(inst->config.devices[i].name, name) == 0)
            return i;
    }
    return -1;
}
