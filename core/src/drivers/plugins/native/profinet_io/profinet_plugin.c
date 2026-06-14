/**
 * @file profinet_plugin.c
 * @brief PROFINET IO Controller Plugin Implementation for OpenPLC Runtime v4
 *
 * Implements one or more PROFINET IO controllers, each on its own network
 * interface. Every controller owns a dedicated SCHED_FIFO rt thread that
 * drives per-device AR establishment (DCP Identify, Connect.req,
 * IODControlReq PrmEnd/ApplicationReady) and cyclic RT data exchange (see
 * profinet_master.c).
 *
 * No cycle_start/cycle_end hooks: I/O exchange is fully owned by the
 * per-master rt threads, decoupled from the PLC scan cycle.
 */

/* _GNU_SOURCE pulls in glibc extensions used by the rt/monitor threads:
 *   - pthread_setname_np()  for `top` / `htop` thread naming
 *   - pthread_kill()        for SIGUSR1 wake-up on stop
 * Must come before any system header. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <errno.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include "plugin_logger.h"
#include "plugin_types.h"
#include "profinet_plugin.h"
#include "profinet_config.h"
#include "profinet_master.h"
#include "cJSON.h"

/* Forward declarations -- referenced by start_single_master via
 * pthread_create before their definitions further down in the file. */
static void *pn_rt_thread(void *arg);
#if PN_ENABLE_MONITOR_THREAD
static void *pn_monitor_thread(void *arg);
#endif

/*
 * =============================================================================
 * Global Plugin State
 * =============================================================================
 */

static plugin_logger_t g_logger;
static plugin_runtime_args_t g_runtime_args;
static pn_master_instance_t *g_masters = NULL; /* heap-allocated array */
static int g_master_count = 0;

/*
 * =============================================================================
 * RT Thread -- AR establishment + cyclic RT data exchange
 * =============================================================================
 */

/* SIGUSR1 wakes the rt/monitor threads out of clock_nanosleep/nanosleep so a
 * stop request lands within microseconds instead of after a full sleep
 * period. The handler is installed once at process init (plc_main.c
 * handle_sigusr1) -- DON'T re-install here, sigaction is process-wide. */

/**
 * @brief RT thread body -- one master's periodic PROFINET IO cycle
 *
 * Runs at SCHED_FIFO with the configured master.task_priority. Sleeps
 * absolutely (CLOCK_MONOTONIC + TIMER_ABSTIME) to the next deadline so
 * jitter stays bounded. Each tick runs pn_master_run_cycle(), which steps
 * every device's AR state machine or cyclic RT exchange.
 */
static void *pn_rt_thread(void *arg)
{
    pn_master_instance_t *inst = (pn_master_instance_t *)arg;

    char tname[16];
    snprintf(tname, sizeof tname, "pn-%s", inst->name);
    pthread_setname_np(pthread_self(), tname);

    int prio = inst->config.master.task_priority;
    if (prio < 1) prio = 1;
    if (prio > 99) prio = 99;
    struct sched_param sp = {0};
    sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        plugin_logger_warn(&g_logger,
            "RT thread '%s': SCHED_FIFO(%d) failed: %s -- running with default scheduling",
            inst->name, prio, strerror(errno));
    } else {
        plugin_logger_info(&g_logger, "RT thread '%s': SCHED_FIFO priority %d", inst->name, prio);
    }

    int64_t interval_ns = (int64_t)inst->config.master.cycle_time_us * 1000LL;
    if (interval_ns <= 0)
        interval_ns = 1000000LL; /* 1 ms safety floor */

    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    while (atomic_load(&inst->rt_running)) {
        pn_master_run_cycle(inst, &g_runtime_args, &g_logger);

        next_wakeup.tv_nsec += (long)(interval_ns % 1000000000LL);
        next_wakeup.tv_sec += (time_t)(interval_ns / 1000000000LL);
        if (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_nsec -= 1000000000L;
            next_wakeup.tv_sec += 1;
        }

        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
        if (rc == EINTR)
            continue; /* SIGUSR1 wake -- loop will re-check rt_running */
    }

    plugin_logger_info(&g_logger, "RT thread '%s': stopped after %llu cycle(s)", inst->name,
                        (unsigned long long)atomic_load(&inst->cycle_counter));
    return NULL;
}

#if PN_ENABLE_MONITOR_THREAD
/**
 * @brief Monitor thread body -- periodic diagnostics logging
 *
 * Polls every configured device's runtime state at
 * diagnostics.status_update_interval_ms (falling back to
 * PN_MONITOR_INTERVAL_MS) and, when enabled, logs a one-shot message on
 * transitions into PN_DEV_FAULT (log_errors) or PN_DEV_DATA_EXCHANGE
 * (log_connections). The rt thread itself drives recovery; this thread only
 * surfaces state transitions so they are not missed between status polls.
 */
static void *pn_monitor_thread(void *arg)
{
    pn_master_instance_t *inst = (pn_master_instance_t *)arg;

    char tname[16];
    snprintf(tname, sizeof tname, "pn-mon-%s", inst->name);
    pthread_setname_np(pthread_self(), tname);

    int interval_ms = inst->config.diagnostics.status_update_interval_ms;
    if (interval_ms <= 0)
        interval_ms = PN_MONITOR_INTERVAL_MS;

    struct timespec interval;
    interval.tv_sec = interval_ms / 1000;
    interval.tv_nsec = (long)(interval_ms % 1000) * 1000000L;

    pn_device_state_t last_state[PN_MAX_DEVICES];
    for (int i = 0; i < inst->config.device_count; i++)
        last_state[i] = (pn_device_state_t)atomic_load(&inst->devices[i].status.state);

    while (atomic_load(&inst->monitor_running)) {
        int rc = nanosleep(&interval, NULL);
        if (rc != 0 && errno == EINTR)
            continue;

        if (!atomic_load(&inst->monitor_running))
            break;

        for (int i = 0; i < inst->config.device_count; i++) {
            pn_device_runtime_t *drt = &inst->devices[i];
            pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->status.state);

            if (state == PN_DEV_FAULT && last_state[i] != PN_DEV_FAULT) {
                if (inst->config.diagnostics.log_errors) {
                    plugin_logger_error(&g_logger,
                        "Master '%s': device '%s' is in FAULT -- check configuration",
                        inst->name, drt->cfg->name);
                }
            } else if (state == PN_DEV_DATA_EXCHANGE && last_state[i] != PN_DEV_DATA_EXCHANGE) {
                if (inst->config.diagnostics.log_connections) {
                    plugin_logger_info(&g_logger, "Master '%s': device '%s' is online",
                                        inst->name, drt->cfg->name);
                }
            } else if (state == PN_DEV_OFFLINE && last_state[i] == PN_DEV_DATA_EXCHANGE) {
                if (inst->config.diagnostics.log_connections) {
                    plugin_logger_warn(&g_logger, "Master '%s': device '%s' went offline",
                                        inst->name, drt->cfg->name);
                }
            }

            last_state[i] = state;
        }
    }

    return NULL;
}
#endif /* PN_ENABLE_MONITOR_THREAD */

/*
 * =============================================================================
 * Master Start/Stop
 * =============================================================================
 */

static int start_single_master(pn_master_instance_t *inst)
{
    if (inst->config.device_count == 0) {
        plugin_logger_warn(&g_logger, "Master '%s': no devices configured -- skipping",
                            inst->name);
        return -1;
    }

    atomic_store(&inst->plugin_state, PN_STATE_CONNECTING);
    plugin_logger_info(&g_logger, "Master '%s': [state: CONNECTING] opening interface '%s'",
                        inst->name, inst->config.master.interface);

    if (pn_master_open(inst, &g_logger) != 0) {
        plugin_logger_error(&g_logger, "Master '%s': failed to open interface '%s'", inst->name,
                             inst->config.master.interface);
        atomic_store(&inst->plugin_state, PN_STATE_ERROR);
        return -1;
    }

    if (pn_master_build_io(inst, &g_runtime_args, &g_logger) != 0) {
        plugin_logger_error(&g_logger,
            "Master '%s': I/O channel map build failed -- aborting startup", inst->name);
        pn_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PN_STATE_ERROR);
        return -1;
    }

    plugin_logger_info(&g_logger, "Master '%s': connecting %d device(s)...", inst->name,
                        inst->config.device_count);

    if (pn_master_startup_devices(inst, &g_logger) != 0) {
        plugin_logger_error(&g_logger,
            "Master '%s': one or more strict devices failed to connect -- aborting startup",
            inst->name);
        pn_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PN_STATE_ERROR);
        return -1;
    }

    atomic_store(&inst->cycle_counter, 0);
    atomic_store(&inst->plugin_state, PN_STATE_OPERATIONAL);

#if PN_ENABLE_MONITOR_THREAD
    atomic_store(&inst->monitor_running, true);
    if (pthread_create(&inst->monitor_thread, NULL, pn_monitor_thread, inst) != 0) {
        plugin_logger_warn(&g_logger,
            "Master '%s': failed to create monitor thread -- running without diagnostics polling",
            inst->name);
        atomic_store(&inst->monitor_running, false);
    }
#endif

    atomic_store(&inst->rt_running, true);
    if (pthread_create(&inst->rt_thread, NULL, pn_rt_thread, inst) != 0) {
        plugin_logger_error(&g_logger, "Master '%s': failed to create rt thread: %s", inst->name,
                             strerror(errno));
        atomic_store(&inst->rt_running, false);
#if PN_ENABLE_MONITOR_THREAD
        if (atomic_load(&inst->monitor_running)) {
            atomic_store(&inst->monitor_running, false);
            pthread_join(inst->monitor_thread, NULL);
        }
#endif
        pn_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PN_STATE_ERROR);
        return -1;
    }

    plugin_logger_info(&g_logger,
        "Master '%s': [state: OPERATIONAL] PROFINET IO controller started (cycle=%d us, %d "
        "device(s))",
        inst->name, inst->config.master.cycle_time_us, inst->config.device_count);

    return 0;
}

static void stop_single_master(pn_master_instance_t *inst)
{
    int state = atomic_load(&inst->plugin_state);
    if (state == PN_STATE_STOPPED || state == PN_STATE_IDLE) {
        plugin_logger_debug(&g_logger, "Master '%s': already stopped/idle", inst->name);
        return;
    }

    plugin_logger_info(&g_logger, "Master '%s': stopping (current state: %s)...", inst->name,
                        pn_state_to_string(state));

    if (atomic_load(&inst->rt_running)) {
        atomic_store(&inst->rt_running, false);
        pthread_kill(inst->rt_thread, SIGUSR1);
        pthread_join(inst->rt_thread, NULL);
        plugin_logger_debug(&g_logger, "Master '%s': rt thread joined", inst->name);
    }

#if PN_ENABLE_MONITOR_THREAD
    if (atomic_load(&inst->monitor_running)) {
        atomic_store(&inst->monitor_running, false);
        pthread_kill(inst->monitor_thread, SIGUSR1);
        pthread_join(inst->monitor_thread, NULL);
        plugin_logger_debug(&g_logger, "Master '%s': monitor thread joined", inst->name);
    }
#endif

    /* Best-effort: release the AR of any device still in cyclic exchange so
     * the device frees its resources promptly. */
    for (int i = 0; i < inst->config.device_count; i++) {
        pn_device_runtime_t *drt = &inst->devices[i];
        if ((pn_device_state_t)atomic_load(&drt->state) == PN_DEV_DATA_EXCHANGE)
            pn_rpc_release(&drt->rpc_ctx, 500);
    }

    pn_master_close(inst, &g_logger);
    atomic_store(&inst->plugin_state, PN_STATE_STOPPED);

    plugin_logger_info(&g_logger, "Master '%s': [state: STOPPED] PROFINET IO controller stopped",
                        inst->name);
}

/*
 * =============================================================================
 * Plugin Lifecycle Functions
 * =============================================================================
 */

int init(void *args)
{
    plugin_logger_init(&g_logger, "PROFINET_IO", NULL);
    plugin_logger_info(&g_logger, "Initializing PROFINET IO plugin...");

    if (!args) {
        plugin_logger_error(&g_logger, "init args is NULL");
        return -1;
    }

    /* Copy runtime args (critical -- pointer is freed after init returns) */
    memcpy(&g_runtime_args, args, sizeof(plugin_runtime_args_t));

    /* Re-initialize logger with runtime_args for central logging */
    plugin_logger_init(&g_logger, "PROFINET_IO", args);

    /* Share the logger with the config parser, DCP, and RPC layers so their
     * diagnostic messages land in the runtime journal instead of stderr. */
    pn_config_set_logger(&g_logger);
    pn_dcp_set_logger(&g_logger);
    pn_rpc_set_logger(&g_logger);

    plugin_logger_info(&g_logger, "Buffer size: %d", g_runtime_args.buffer_size);

    const char *config_path = g_runtime_args.plugin_specific_config_file_path;

    pn_config_t *temp_configs = calloc(PN_MAX_MASTERS, sizeof(pn_config_t));
    if (!temp_configs) {
        plugin_logger_error(&g_logger, "Failed to allocate config array");
        return -1;
    }

    int count = 0;
    if (config_path != NULL && config_path[0] != '\0') {
        plugin_logger_info(&g_logger, "Loading config: %s", config_path);
        int result = pn_config_parse_all(config_path, temp_configs, PN_MAX_MASTERS, &count);
        if (result != PN_CONFIG_OK || count == 0) {
            plugin_logger_warn(&g_logger,
                "No valid PROFINET_IO configs found (result=%d, count=%d), using defaults",
                result, count);
            pn_config_init_defaults(&temp_configs[0]);
            count = 1;
        } else {
            plugin_logger_info(&g_logger, "Configuration loaded: %d master(s) found", count);
        }
    } else {
        plugin_logger_warn(&g_logger, "No config file specified, using defaults");
        pn_config_init_defaults(&temp_configs[0]);
        count = 1;
    }

    g_masters = calloc((size_t)count, sizeof(pn_master_instance_t));
    if (!g_masters) {
        plugin_logger_error(&g_logger, "Failed to allocate master instances");
        free(temp_configs);
        return -1;
    }
    g_master_count = count;

    for (int i = 0; i < g_master_count; i++) {
        g_masters[i].config = temp_configs[i];

        if (pn_config_validate(&g_masters[i].config) != PN_CONFIG_OK) {
            plugin_logger_error(&g_logger, "Master[%d] '%s': configuration invalid", i,
                                 temp_configs[i].name);
            free(temp_configs);
            free(g_masters);
            g_masters = NULL;
            g_master_count = 0;
            return -1;
        }

        if (pn_master_init(&g_masters[i], temp_configs[i].name, &g_logger) != 0) {
            plugin_logger_error(&g_logger, "Master[%d] '%s': failed to initialize", i,
                                 temp_configs[i].name);
            for (int j = 0; j < i; j++)
                pn_master_destroy(&g_masters[j]);
            free(temp_configs);
            free(g_masters);
            g_masters = NULL;
            g_master_count = 0;
            return -1;
        }

        plugin_logger_info(&g_logger,
            "Master[%d] '%s': interface=%s, cycle_time=%d us, device(s)=%d", i, g_masters[i].name,
            g_masters[i].config.master.interface, g_masters[i].config.master.cycle_time_us,
            g_masters[i].config.device_count);
    }

    free(temp_configs);

    plugin_logger_info(&g_logger, "PROFINET IO plugin initialized [%d master(s), state: IDLE]",
                        g_master_count);
    return 0;
}

int start_loop(void)
{
    int any_started = 0;

    for (int i = 0; i < g_master_count; i++) {
        pn_master_instance_t *inst = &g_masters[i];

        int state = atomic_load(&inst->plugin_state);
        if (state != PN_STATE_IDLE && state != PN_STATE_STOPPED) {
            plugin_logger_error(&g_logger, "Master '%s': cannot start -- invalid state: %s",
                                 inst->name, pn_state_to_string(state));
            continue;
        }

        if (start_single_master(inst) == 0)
            any_started++;
    }

    if (any_started == 0) {
        plugin_logger_error(&g_logger, "No PROFINET IO masters started successfully");
        return -1;
    }

    plugin_logger_info(&g_logger, "%d/%d PROFINET IO master(s) started", any_started,
                        g_master_count);
    return 0;
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping all PROFINET IO masters...");

    for (int i = 0; i < g_master_count; i++)
        stop_single_master(&g_masters[i]);

    plugin_logger_info(&g_logger, "All PROFINET IO masters stopped");
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up PROFINET IO plugin...");

    for (int i = 0; i < g_master_count; i++) {
        pn_master_instance_t *inst = &g_masters[i];
        int state = atomic_load(&inst->plugin_state);
        if (state != PN_STATE_STOPPED && state != PN_STATE_IDLE)
            stop_single_master(inst);
        pn_master_destroy(inst);
    }

    free(g_masters);
    g_masters = NULL;
    g_master_count = 0;

    plugin_logger_info(&g_logger, "PROFINET IO plugin cleanup complete");
}

/*
 * =============================================================================
 * execute_command -- Status / Diagnostics JSON Builders
 * =============================================================================
 */

static void format_mac(char *out, size_t out_size, const uint8_t mac[6])
{
    snprintf(out, out_size, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
             mac[4], mac[5]);
}

static cJSON *build_device_status_json(const pn_device_runtime_t *drt, bool diagnostics)
{
    cJSON *device = cJSON_CreateObject();

    cJSON_AddStringToObject(device, "name", drt->status.name);
    cJSON_AddStringToObject(device, "station_name", drt->status.station_name);

    pn_device_state_t state = (pn_device_state_t)atomic_load(&drt->status.state);
    cJSON_AddStringToObject(device, "state", pn_device_state_to_string(state));

    cJSON_AddNumberToObject(device, "cycle_count", (double)atomic_load(&drt->status.cycle_count));
    cJSON_AddNumberToObject(device, "error_count", (double)atomic_load(&drt->status.error_count));
    cJSON_AddNumberToObject(device, "reconnect_count",
                             (double)atomic_load(&drt->status.reconnect_count));

    if (diagnostics) {
        char mac_str[18];
        format_mac(mac_str, sizeof(mac_str), drt->mac);
        cJSON_AddStringToObject(device, "mac", mac_str);
        cJSON_AddStringToObject(device, "ip_address", drt->cfg->ip_address);
        cJSON_AddNumberToObject(device, "input_length", drt->cfg->input_length);
        cJSON_AddNumberToObject(device, "output_length", drt->cfg->output_length);
        cJSON_AddBoolToObject(device, "input_data_valid",
                               atomic_load(&drt->status.input_iops_good) != 0);
        cJSON_AddNumberToObject(device, "input_frame_id", drt->input_frame_id);
        cJSON_AddNumberToObject(device, "output_frame_id", drt->output_frame_id);
    }

    return device;
}

static cJSON *build_master_status_json(pn_master_instance_t *inst, bool diagnostics)
{
    int state = atomic_load(&inst->plugin_state);

    cJSON *master = cJSON_CreateObject();
    cJSON_AddStringToObject(master, "name", inst->name);
    cJSON_AddStringToObject(master, "state", pn_state_to_string(state));
    cJSON_AddStringToObject(master, "interface", inst->config.master.interface);
    cJSON_AddNumberToObject(master, "cycle_count", (double)atomic_load(&inst->cycle_counter));
    cJSON_AddNumberToObject(master, "device_count", inst->config.device_count);

    cJSON *devices = cJSON_AddArrayToObject(master, "devices");
    for (int i = 0; i < inst->config.device_count; i++)
        cJSON_AddItemToArray(devices, build_device_status_json(&inst->devices[i], diagnostics));

    return master;
}

static int handle_status_command(char *response, size_t response_size)
{
    cJSON *resp = cJSON_CreateObject();
    cJSON *masters = cJSON_AddArrayToObject(resp, "masters");

    for (int i = 0; i < g_master_count; i++)
        cJSON_AddItemToArray(masters, build_master_status_json(&g_masters[i], false));

    char *json_str = cJSON_PrintUnformatted(resp);
    if (json_str) {
        snprintf(response, response_size, "%s", json_str);
        free(json_str);
    }
    cJSON_Delete(resp);
    return 0;
}

static int handle_diagnostics_command(char *response, size_t response_size)
{
    cJSON *resp = cJSON_CreateObject();
    cJSON *masters = cJSON_AddArrayToObject(resp, "masters");

    for (int i = 0; i < g_master_count; i++)
        cJSON_AddItemToArray(masters, build_master_status_json(&g_masters[i], true));

    char *json_str = cJSON_PrintUnformatted(resp);
    if (json_str) {
        snprintf(response, response_size, "%s", json_str);
        free(json_str);
    }
    cJSON_Delete(resp);
    return 0;
}

/**
 * @brief Enumerate non-loopback network interfaces.
 *
 * Purely informational -- helps a UI populate an interface picker for the
 * master's "interface" config field. Does not verify the interface supports
 * raw AF_PACKET access.
 */
static int handle_list_interfaces_command(char *response, size_t response_size)
{
    cJSON *resp = cJSON_CreateObject();
    cJSON *ifaces = cJSON_AddArrayToObject(resp, "interfaces");

    struct ifaddrs *addrs = NULL;
    if (getifaddrs(&addrs) == 0) {
        for (struct ifaddrs *a = addrs; a != NULL; a = a->ifa_next) {
            if (a->ifa_flags & IFF_LOOPBACK)
                continue;

            bool seen = false;
            for (cJSON *item = ifaces->child; item != NULL; item = item->next) {
                cJSON *name_item = cJSON_GetObjectItemCaseSensitive(item, "name");
                if (cJSON_IsString(name_item) && strcmp(name_item->valuestring, a->ifa_name) == 0) {
                    seen = true;
                    break;
                }
            }
            if (seen)
                continue;

            cJSON *entry = cJSON_CreateObject();
            cJSON_AddStringToObject(entry, "name", a->ifa_name);
            cJSON_AddBoolToObject(entry, "up", (a->ifa_flags & IFF_UP) != 0);
            cJSON_AddItemToArray(ifaces, entry);
        }
        freeifaddrs(addrs);
    } else {
        plugin_logger_warn(&g_logger, "list-interfaces: getifaddrs failed: %s", strerror(errno));
    }

    char *json_str = cJSON_PrintUnformatted(resp);
    if (json_str) {
        snprintf(response, response_size, "%s", json_str);
        free(json_str);
    }
    cJSON_Delete(resp);
    return 0;
}

/*
 * =============================================================================
 * execute_command -- Dispatcher
 * =============================================================================
 */

int execute_command(const char *command_json, char *response, size_t response_size)
{
    cJSON *root = cJSON_Parse(command_json);
    if (!root) {
        snprintf(response, response_size, "{\"error\":\"invalid JSON\"}");
        return -1;
    }

    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(root);
        snprintf(response, response_size, "{\"error\":\"missing 'command' field\"}");
        return -1;
    }

    int result = -1;
    if (strcmp(cmd->valuestring, "status") == 0) {
        result = handle_status_command(response, response_size);
    } else if (strcmp(cmd->valuestring, "diagnostics") == 0) {
        result = handle_diagnostics_command(response, response_size);
    } else if (strcmp(cmd->valuestring, "list-interfaces") == 0) {
        result = handle_list_interfaces_command(response, response_size);
    } else {
        snprintf(response, response_size, "{\"error\":\"unknown command '%s'\"}",
                 cmd->valuestring);
    }

    cJSON_Delete(root);
    return result;
}
