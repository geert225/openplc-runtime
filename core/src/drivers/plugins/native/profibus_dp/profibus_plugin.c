/**
 * @file profibus_plugin.c
 * @brief Profibus DP Plugin Implementation for OpenPLC Runtime v4
 *
 * Implements one or more Profibus DP masters, each on its own RS-485 serial
 * port. Every master owns a dedicated SCHED_FIFO bus thread that drives
 * per-slave parameterization (DPV0 Set_Prm/Chk_Cfg/Slave_Diag) and cyclic
 * Data_Exchange, plus pending DPV1 initial-value writes. On-demand DPV1
 * reads/writes are issued via execute_command() and serialized against the
 * bus thread through the master's bus_mutex (see profibus_master.c).
 *
 * No cycle_start/cycle_end hooks: I/O exchange is fully owned by the
 * per-master bus threads, decoupled from the PLC scan cycle.
 */

/* _GNU_SOURCE pulls in glibc extensions used by the bus/monitor threads:
 *   - pthread_setname_np()  for `top` / `htop` thread naming
 *   - pthread_kill()        for SIGUSR1 wake-up on stop
 * Must come before any system header. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include "plugin_logger.h"
#include "plugin_types.h"
#include "profibus_plugin.h"
#include "profibus_config.h"
#include "profibus_master.h"
#include "profibus_dp_messages.h" /* PB_STAT1_*, PB_STAT2_* diagnostic flag bits */
#include "cJSON.h"

/* Forward declarations -- referenced by start_single_master via
 * pthread_create before their definitions further down in the file. */
static void *pb_bus_thread(void *arg);
#if PB_ENABLE_MONITOR_THREAD
static void *pb_monitor_thread(void *arg);
#endif

/*
 * =============================================================================
 * Global Plugin State
 * =============================================================================
 */

static plugin_logger_t g_logger;
static plugin_runtime_args_t g_runtime_args;
static pb_master_instance_t *g_masters = NULL; /* heap-allocated array */
static int g_master_count = 0;

/*
 * =============================================================================
 * Helpers
 * =============================================================================
 */

static inline uint64_t ts_to_ns(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * 1000000000ULL + (uint64_t)ts->tv_nsec;
}

/**
 * @brief Find a configured master by name.
 *
 * If @p name is NULL/empty and exactly one master is configured, returns
 * that master. Otherwise looks up by exact name match.
 */
static pb_master_instance_t *find_master_by_name(const char *name)
{
    if (name == NULL || name[0] == '\0')
        return (g_master_count > 0) ? &g_masters[0] : NULL;

    for (int i = 0; i < g_master_count; i++) {
        if (strcmp(g_masters[i].name, name) == 0)
            return &g_masters[i];
    }
    return NULL;
}

/*
 * =============================================================================
 * Bus Thread -- cyclic Data_Exchange + parameterization
 * =============================================================================
 */

/* SIGUSR1 wakes the bus/monitor threads out of clock_nanosleep/nanosleep so
 * a stop request lands within microseconds instead of after a full sleep
 * period. The handler is installed once at process init (plc_main.c
 * handle_sigusr1) -- DON'T re-install here, sigaction is process-wide. */

/**
 * @brief Bus thread body -- one master's periodic Profibus DP cycle
 *
 * Runs at SCHED_FIFO with the configured master.task_priority. Sleeps
 * absolutely (CLOCK_MONOTONIC + TIMER_ABSTIME) to the next deadline so
 * jitter stays bounded. Each tick runs pb_master_run_cycle(), which steps
 * every slave's parameterization state machine or cyclic Data_Exchange.
 */
static void *pb_bus_thread(void *arg)
{
    pb_master_instance_t *inst = (pb_master_instance_t *)arg;

    char tname[16];
    snprintf(tname, sizeof tname, "pb-%s", inst->name);
    pthread_setname_np(pthread_self(), tname);

    int prio = inst->config.master.task_priority;
    if (prio < 1) prio = 1;
    if (prio > 99) prio = 99;
    struct sched_param sp = {0};
    sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        plugin_logger_warn(&g_logger,
            "Bus thread '%s': SCHED_FIFO(%d) failed: %s -- running with default scheduling",
            inst->name, prio, strerror(errno));
    } else {
        plugin_logger_info(&g_logger, "Bus thread '%s': SCHED_FIFO priority %d", inst->name,
                            prio);
    }

    int64_t interval_ns = (int64_t)inst->config.master.cycle_time_us * 1000LL;
    if (interval_ns <= 0)
        interval_ns = 10000000LL; /* 10 ms safety floor */

    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    while (atomic_load(&inst->bus_running)) {
        pb_master_run_cycle(inst, &g_runtime_args, &g_logger);

        next_wakeup.tv_nsec += (long)(interval_ns % 1000000000LL);
        next_wakeup.tv_sec += (time_t)(interval_ns / 1000000000LL);
        if (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_nsec -= 1000000000L;
            next_wakeup.tv_sec += 1;
        }

        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
        if (rc == EINTR)
            continue; /* SIGUSR1 wake -- loop will re-check bus_running */
    }

    plugin_logger_info(&g_logger, "Bus thread '%s': stopped after %llu cycle(s)", inst->name,
                        (unsigned long long)atomic_load(&inst->cycle_counter));
    return NULL;
}

#if PB_ENABLE_MONITOR_THREAD
/**
 * @brief Monitor thread body -- periodic diagnostics logging
 *
 * Polls every configured slave's runtime state at
 * diagnostics.status_update_interval_ms (falling back to
 * PB_MONITOR_INTERVAL_MS) and, when diagnostics.log_errors is enabled, logs
 * a one-shot warning whenever a slave transitions into PB_SLAVE_FAULT. The
 * bus thread itself drives recovery (OFFLINE/WAIT_DIAG1 retries are
 * silent); this thread only surfaces terminal failures so they are not
 * missed between status polls.
 */
static void *pb_monitor_thread(void *arg)
{
    pb_master_instance_t *inst = (pb_master_instance_t *)arg;

    char tname[16];
    snprintf(tname, sizeof tname, "pb-mon-%s", inst->name);
    pthread_setname_np(pthread_self(), tname);

    int interval_ms = inst->config.diagnostics.status_update_interval_ms;
    if (interval_ms <= 0)
        interval_ms = PB_MONITOR_INTERVAL_MS;

    struct timespec interval;
    interval.tv_sec = interval_ms / 1000;
    interval.tv_nsec = (long)(interval_ms % 1000) * 1000000L;

    pb_slave_state_t last_state[PB_MAX_SLAVES];
    for (int i = 0; i < inst->config.slave_count; i++)
        last_state[i] = (pb_slave_state_t)atomic_load(&inst->slaves[i].status.state);

    while (atomic_load(&inst->monitor_running)) {
        int rc = nanosleep(&interval, NULL);
        if (rc != 0 && errno == EINTR)
            continue;

        if (!atomic_load(&inst->monitor_running))
            break;

        if (!inst->config.diagnostics.log_errors)
            continue;

        for (int i = 0; i < inst->config.slave_count; i++) {
            pb_slave_runtime_t *srt = &inst->slaves[i];
            pb_slave_state_t state = (pb_slave_state_t)atomic_load(&srt->status.state);

            if (state == PB_SLAVE_FAULT && last_state[i] != PB_SLAVE_FAULT) {
                plugin_logger_error(&g_logger,
                    "Master '%s': slave '%s' (addr %d) is in FAULT -- check configuration",
                    inst->name, srt->cfg->name, srt->cfg->station_address);
            } else if (state == PB_SLAVE_DATA_EXCHANGE && last_state[i] != PB_SLAVE_DATA_EXCHANGE &&
                       inst->config.diagnostics.log_connections) {
                plugin_logger_info(&g_logger, "Master '%s': slave '%s' (addr %d) is online",
                                    inst->name, srt->cfg->name, srt->cfg->station_address);
            }

            last_state[i] = state;
        }
    }

    return NULL;
}
#endif /* PB_ENABLE_MONITOR_THREAD */

/*
 * =============================================================================
 * Master Start/Stop
 * =============================================================================
 */

static int start_single_master(pb_master_instance_t *inst)
{
    if (inst->config.slave_count == 0) {
        plugin_logger_warn(&g_logger, "Master '%s': no slaves configured -- skipping",
                            inst->name);
        return -1;
    }

    atomic_store(&inst->plugin_state, PB_STATE_CONNECTING);
    plugin_logger_info(&g_logger, "Master '%s': [state: CONNECTING] opening %s at %d baud",
                        inst->name, inst->config.master.device, inst->config.master.baudrate);

    if (pb_master_open(inst, &g_logger) != 0) {
        plugin_logger_error(&g_logger, "Master '%s': failed to open serial port '%s'",
                             inst->name, inst->config.master.device);
        atomic_store(&inst->plugin_state, PB_STATE_ERROR);
        return -1;
    }

    if (pb_master_build_io(inst, &g_runtime_args, &g_logger) != 0) {
        plugin_logger_error(&g_logger,
            "Master '%s': I/O channel map build failed -- aborting startup", inst->name);
        pb_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PB_STATE_ERROR);
        return -1;
    }

    atomic_store(&inst->plugin_state, PB_STATE_PARAMETERIZING);
    plugin_logger_info(&g_logger, "Master '%s': [state: PARAMETERIZING] parameterizing %d slave(s)...",
                        inst->name, inst->config.slave_count);

    if (pb_master_startup_slaves(inst, &g_logger) != 0) {
        plugin_logger_error(&g_logger,
            "Master '%s': one or more strict slaves failed to parameterize -- aborting startup",
            inst->name);
        pb_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PB_STATE_ERROR);
        return -1;
    }

    atomic_store(&inst->cycle_counter, 0);
    atomic_store(&inst->plugin_state, PB_STATE_OPERATIONAL);

#if PB_ENABLE_MONITOR_THREAD
    atomic_store(&inst->monitor_running, true);
    if (pthread_create(&inst->monitor_thread, NULL, pb_monitor_thread, inst) != 0) {
        plugin_logger_warn(&g_logger,
            "Master '%s': failed to create monitor thread -- running without diagnostics polling",
            inst->name);
        atomic_store(&inst->monitor_running, false);
    }
#endif

    atomic_store(&inst->bus_running, true);
    if (pthread_create(&inst->bus_thread, NULL, pb_bus_thread, inst) != 0) {
        plugin_logger_error(&g_logger, "Master '%s': failed to create bus thread: %s",
                             inst->name, strerror(errno));
        atomic_store(&inst->bus_running, false);
#if PB_ENABLE_MONITOR_THREAD
        if (atomic_load(&inst->monitor_running)) {
            atomic_store(&inst->monitor_running, false);
            pthread_join(inst->monitor_thread, NULL);
        }
#endif
        pb_master_close(inst, &g_logger);
        atomic_store(&inst->plugin_state, PB_STATE_ERROR);
        return -1;
    }

    plugin_logger_info(&g_logger,
        "Master '%s': [state: OPERATIONAL] Profibus DP master started (cycle=%d us, %d slave(s))",
        inst->name, inst->config.master.cycle_time_us, inst->config.slave_count);

    return 0;
}

static void stop_single_master(pb_master_instance_t *inst)
{
    int state = atomic_load(&inst->plugin_state);
    if (state == PB_STATE_STOPPED || state == PB_STATE_IDLE) {
        plugin_logger_debug(&g_logger, "Master '%s': already stopped/idle", inst->name);
        return;
    }

    plugin_logger_info(&g_logger, "Master '%s': stopping (current state: %s)...", inst->name,
                        pb_state_to_string(state));

    if (atomic_load(&inst->bus_running)) {
        atomic_store(&inst->bus_running, false);
        pthread_kill(inst->bus_thread, SIGUSR1);
        pthread_join(inst->bus_thread, NULL);
        plugin_logger_debug(&g_logger, "Master '%s': bus thread joined", inst->name);
    }

#if PB_ENABLE_MONITOR_THREAD
    if (atomic_load(&inst->monitor_running)) {
        atomic_store(&inst->monitor_running, false);
        pthread_kill(inst->monitor_thread, SIGUSR1);
        pthread_join(inst->monitor_thread, NULL);
        plugin_logger_debug(&g_logger, "Master '%s': monitor thread joined", inst->name);
    }
#endif

    pb_master_close(inst, &g_logger);
    atomic_store(&inst->plugin_state, PB_STATE_STOPPED);

    plugin_logger_info(&g_logger, "Master '%s': [state: STOPPED] Profibus DP master stopped",
                        inst->name);
}

/*
 * =============================================================================
 * Plugin Lifecycle Functions
 * =============================================================================
 */

int init(void *args)
{
    plugin_logger_init(&g_logger, "PROFIBUS_DP", NULL);
    plugin_logger_info(&g_logger, "Initializing Profibus DP plugin...");

    if (!args) {
        plugin_logger_error(&g_logger, "init args is NULL");
        return -1;
    }

    /* Copy runtime args (critical -- pointer is freed after init returns) */
    memcpy(&g_runtime_args, args, sizeof(plugin_runtime_args_t));

    /* Re-initialize logger with runtime_args for central logging */
    plugin_logger_init(&g_logger, "PROFIBUS_DP", args);

    /* Share the logger with the config parser so its diagnostic messages
     * land in the runtime journal instead of stderr. */
    pb_config_set_logger(&g_logger);

    plugin_logger_info(&g_logger, "Buffer size: %d", g_runtime_args.buffer_size);

    const char *config_path = g_runtime_args.plugin_specific_config_file_path;

    pb_config_t *temp_configs = calloc(PB_MAX_MASTERS, sizeof(pb_config_t));
    if (!temp_configs) {
        plugin_logger_error(&g_logger, "Failed to allocate config array");
        return -1;
    }

    int count = 0;
    if (config_path != NULL && config_path[0] != '\0') {
        plugin_logger_info(&g_logger, "Loading config: %s", config_path);
        int result = pb_config_parse_all(config_path, temp_configs, PB_MAX_MASTERS, &count);
        if (result != PB_CONFIG_OK || count == 0) {
            plugin_logger_warn(&g_logger,
                "No valid PROFIBUS_DP configs found (result=%d, count=%d), using defaults",
                result, count);
            pb_config_init_defaults(&temp_configs[0]);
            count = 1;
        } else {
            plugin_logger_info(&g_logger, "Configuration loaded: %d master(s) found", count);
        }
    } else {
        plugin_logger_warn(&g_logger, "No config file specified, using defaults");
        pb_config_init_defaults(&temp_configs[0]);
        count = 1;
    }

    g_masters = calloc((size_t)count, sizeof(pb_master_instance_t));
    if (!g_masters) {
        plugin_logger_error(&g_logger, "Failed to allocate master instances");
        free(temp_configs);
        return -1;
    }
    g_master_count = count;

    for (int i = 0; i < g_master_count; i++) {
        g_masters[i].config = temp_configs[i];

        if (pb_config_validate(&g_masters[i].config) != PB_CONFIG_OK) {
            plugin_logger_error(&g_logger, "Master[%d] '%s': configuration invalid", i,
                                 temp_configs[i].name);
            free(temp_configs);
            free(g_masters);
            g_masters = NULL;
            g_master_count = 0;
            return -1;
        }

        if (pb_master_init(&g_masters[i], temp_configs[i].name, &g_logger) != 0) {
            plugin_logger_error(&g_logger, "Master[%d] '%s': failed to initialize", i,
                                 temp_configs[i].name);
            for (int j = 0; j < i; j++)
                pb_master_destroy(&g_masters[j]);
            free(temp_configs);
            free(g_masters);
            g_masters = NULL;
            g_master_count = 0;
            return -1;
        }

        plugin_logger_info(&g_logger,
            "Master[%d] '%s': device=%s, baudrate=%d, cycle_time=%d us, slaves=%d", i,
            g_masters[i].name, g_masters[i].config.master.device,
            g_masters[i].config.master.baudrate, g_masters[i].config.master.cycle_time_us,
            g_masters[i].config.slave_count);
    }

    free(temp_configs);

    plugin_logger_info(&g_logger, "Profibus DP plugin initialized [%d master(s), state: IDLE]",
                        g_master_count);
    return 0;
}

int start_loop(void)
{
    int any_started = 0;

    for (int i = 0; i < g_master_count; i++) {
        pb_master_instance_t *inst = &g_masters[i];

        int state = atomic_load(&inst->plugin_state);
        if (state != PB_STATE_IDLE && state != PB_STATE_STOPPED) {
            plugin_logger_error(&g_logger, "Master '%s': cannot start -- invalid state: %s",
                                 inst->name, pb_state_to_string(state));
            continue;
        }

        if (start_single_master(inst) == 0)
            any_started++;
    }

    if (any_started == 0) {
        plugin_logger_error(&g_logger, "No Profibus DP masters started successfully");
        return -1;
    }

    plugin_logger_info(&g_logger, "%d/%d Profibus DP master(s) started", any_started,
                        g_master_count);
    return 0;
}

void stop_loop(void)
{
    plugin_logger_info(&g_logger, "Stopping all Profibus DP masters...");

    for (int i = 0; i < g_master_count; i++)
        stop_single_master(&g_masters[i]);

    plugin_logger_info(&g_logger, "All Profibus DP masters stopped");
}

void cleanup(void)
{
    plugin_logger_info(&g_logger, "Cleaning up Profibus DP plugin...");

    for (int i = 0; i < g_master_count; i++) {
        pb_master_instance_t *inst = &g_masters[i];
        int state = atomic_load(&inst->plugin_state);
        if (state != PB_STATE_STOPPED && state != PB_STATE_IDLE)
            stop_single_master(inst);
        pb_master_destroy(inst);
    }

    free(g_masters);
    g_masters = NULL;
    g_master_count = 0;

    plugin_logger_info(&g_logger, "Profibus DP plugin cleanup complete");
}

/*
 * =============================================================================
 * execute_command -- Status / Diagnostics JSON Builders
 * =============================================================================
 */

static cJSON *build_slave_status_json(const pb_slave_runtime_t *srt, bool diagnostics)
{
    cJSON *slave = cJSON_CreateObject();

    cJSON_AddNumberToObject(slave, "station_address", srt->status.station_address);
    cJSON_AddStringToObject(slave, "name", srt->status.name);

    pb_slave_state_t state = (pb_slave_state_t)atomic_load(&srt->status.state);
    cJSON_AddStringToObject(slave, "state", pb_slave_state_to_string(state));

    cJSON_AddNumberToObject(slave, "cycle_count",
                             (double)atomic_load(&srt->status.cycle_count));
    cJSON_AddNumberToObject(slave, "error_count",
                             (double)atomic_load(&srt->status.error_count));
    cJSON_AddNumberToObject(slave, "retry_count",
                             (double)atomic_load(&srt->status.retry_count));

    uint32_t diag_flags = atomic_load(&srt->status.diag_flags);

    if (diagnostics) {
        uint8_t stat_1 = diag_flags & 0xFF;
        uint8_t stat_2 = (diag_flags >> 8) & 0xFF;
        uint8_t stat_3 = (diag_flags >> 16) & 0xFF;

        cJSON *diag = cJSON_CreateObject();
        cJSON_AddNumberToObject(diag, "stat_1", stat_1);
        cJSON_AddNumberToObject(diag, "stat_2", stat_2);
        cJSON_AddNumberToObject(diag, "stat_3", stat_3);
        cJSON_AddBoolToObject(diag, "station_not_exist",
                               (stat_1 & PB_STAT1_STATION_NOT_EXIST) != 0);
        cJSON_AddBoolToObject(diag, "station_not_ready",
                               (stat_1 & PB_STAT1_STATION_NOT_READY) != 0);
        cJSON_AddBoolToObject(diag, "cfg_fault", (stat_1 & PB_STAT1_CFG_FAULT) != 0);
        cJSON_AddBoolToObject(diag, "ext_diag", (stat_1 & PB_STAT1_EXT_DIAG) != 0);
        cJSON_AddBoolToObject(diag, "prm_fault", (stat_1 & PB_STAT1_PRM_FAULT) != 0);
        cJSON_AddBoolToObject(diag, "master_lock", (stat_1 & PB_STAT1_MASTER_LOCK) != 0);
        cJSON_AddBoolToObject(diag, "watchdog_on", (stat_2 & PB_STAT2_WD_ON) != 0);
        cJSON_AddBoolToObject(diag, "prm_req", (stat_2 & PB_STAT2_PRM_REQ) != 0);
        cJSON_AddItemToObject(slave, "diag", diag);
    } else {
        cJSON_AddNumberToObject(slave, "diag_flags", diag_flags);
    }

    return slave;
}

static cJSON *build_master_status_json(pb_master_instance_t *inst, bool diagnostics)
{
    int state = atomic_load(&inst->plugin_state);

    cJSON *master = cJSON_CreateObject();
    cJSON_AddStringToObject(master, "name", inst->name);
    cJSON_AddStringToObject(master, "state", pb_state_to_string(state));
    cJSON_AddStringToObject(master, "device", inst->config.master.device);
    cJSON_AddNumberToObject(master, "baudrate", inst->config.master.baudrate);
    cJSON_AddNumberToObject(master, "cycle_count", (double)atomic_load(&inst->cycle_counter));
    cJSON_AddNumberToObject(master, "slave_count", inst->config.slave_count);

    cJSON *slaves = cJSON_AddArrayToObject(master, "slaves");
    for (int i = 0; i < inst->config.slave_count; i++)
        cJSON_AddItemToArray(slaves, build_slave_status_json(&inst->slaves[i], diagnostics));

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
 * @brief Enumerate likely RS-485/RS-232 serial devices under /dev.
 *
 * Purely informational -- helps a UI populate a port picker. Does not
 * verify the device is a Profibus-capable adapter.
 */
static int handle_list_ports_command(char *response, size_t response_size)
{
    static const char *prefixes[] = { "ttyUSB", "ttyACM", "ttyS", "ttyAMA", "ttyO", "rfcomm" };

    cJSON *resp = cJSON_CreateObject();
    cJSON *ports = cJSON_AddArrayToObject(resp, "ports");

    DIR *dir = opendir("/dev");
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            for (size_t p = 0; p < sizeof(prefixes) / sizeof(prefixes[0]); p++) {
                size_t plen = strlen(prefixes[p]);
                if (strncmp(entry->d_name, prefixes[p], plen) == 0) {
                    char path[300];
                    snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
                    cJSON_AddItemToArray(ports, cJSON_CreateString(path));
                    break;
                }
            }
        }
        closedir(dir);
    } else {
        plugin_logger_warn(&g_logger, "list-ports: failed to open /dev: %s", strerror(errno));
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
 * execute_command -- DPV1 Read/Write
 * =============================================================================
 */

static int handle_dpv1_read_command(cJSON *root, char *response, size_t response_size)
{
    cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "params");
    cJSON *master_j = params ? cJSON_GetObjectItemCaseSensitive(params, "master") : NULL;
    cJSON *addr_j = params ? cJSON_GetObjectItemCaseSensitive(params, "station_address") : NULL;
    cJSON *slot_j = params ? cJSON_GetObjectItemCaseSensitive(params, "slot") : NULL;
    cJSON *index_j = params ? cJSON_GetObjectItemCaseSensitive(params, "index") : NULL;
    cJSON *length_j = params ? cJSON_GetObjectItemCaseSensitive(params, "length") : NULL;

    if (!cJSON_IsNumber(addr_j) || !cJSON_IsNumber(slot_j) || !cJSON_IsNumber(index_j) ||
        !cJSON_IsNumber(length_j)) {
        snprintf(response, response_size,
                 "{\"error\":\"dpv1-read requires numeric station_address, slot, index, length\"}");
        return -1;
    }

    pb_master_instance_t *inst =
        find_master_by_name(cJSON_IsString(master_j) ? master_j->valuestring : NULL);
    if (!inst) {
        snprintf(response, response_size, "{\"error\":\"master not found\"}");
        return -1;
    }

    if (atomic_load(&inst->plugin_state) != PB_STATE_OPERATIONAL) {
        snprintf(response, response_size, "{\"error\":\"master '%s' is not operational\"}",
                 inst->name);
        return -1;
    }

    int station_address = (int)addr_j->valuedouble;
    int slave_index = pb_master_find_slave_by_address(inst, station_address);
    if (slave_index < 0) {
        snprintf(response, response_size,
                 "{\"error\":\"slave with station_address %d not found on master '%s'\"}",
                 station_address, inst->name);
        return -1;
    }

    int length = (int)length_j->valuedouble;
    if (length <= 0 || length > PB_MAX_IO_DATA_LEN) {
        snprintf(response, response_size, "{\"error\":\"length must be between 1 and %d\"}",
                 PB_MAX_IO_DATA_LEN);
        return -1;
    }

    uint8_t data[PB_MAX_IO_DATA_LEN];
    int out_len = 0;
    if (pb_master_dpv1_read(inst, slave_index, (uint8_t)slot_j->valuedouble,
                             (uint8_t)index_j->valuedouble, (uint8_t)length, data, &out_len,
                             &g_logger) != 0) {
        snprintf(response, response_size, "{\"error\":\"DPV1 read failed\"}");
        return -1;
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "master", inst->name);
    cJSON_AddNumberToObject(resp, "station_address", station_address);
    cJSON_AddNumberToObject(resp, "slot", slot_j->valuedouble);
    cJSON_AddNumberToObject(resp, "index", index_j->valuedouble);
    cJSON_AddNumberToObject(resp, "length", out_len);

    cJSON *data_arr = cJSON_AddArrayToObject(resp, "data");
    for (int i = 0; i < out_len; i++)
        cJSON_AddItemToArray(data_arr, cJSON_CreateNumber(data[i]));

    cJSON *dtype_j = cJSON_GetObjectItemCaseSensitive(params, "data_type");
    if (cJSON_IsString(dtype_j) && out_len > 0 && out_len <= 8) {
        pb_data_type_t dt = pb_parse_data_type(dtype_j->valuestring);
        cJSON_AddNumberToObject(resp, "value", pb_bytes_to_value(data, dt, (uint8_t)out_len));
        cJSON_AddStringToObject(resp, "data_type", pb_data_type_to_string(dt));
    }

    char *json_str = cJSON_PrintUnformatted(resp);
    if (json_str) {
        snprintf(response, response_size, "%s", json_str);
        free(json_str);
    }
    cJSON_Delete(resp);
    return 0;
}

static int handle_dpv1_write_command(cJSON *root, char *response, size_t response_size)
{
    cJSON *params = cJSON_GetObjectItemCaseSensitive(root, "params");
    cJSON *master_j = params ? cJSON_GetObjectItemCaseSensitive(params, "master") : NULL;
    cJSON *addr_j = params ? cJSON_GetObjectItemCaseSensitive(params, "station_address") : NULL;
    cJSON *slot_j = params ? cJSON_GetObjectItemCaseSensitive(params, "slot") : NULL;
    cJSON *index_j = params ? cJSON_GetObjectItemCaseSensitive(params, "index") : NULL;
    cJSON *value_j = params ? cJSON_GetObjectItemCaseSensitive(params, "value") : NULL;
    cJSON *dtype_j = params ? cJSON_GetObjectItemCaseSensitive(params, "data_type") : NULL;
    cJSON *length_j = params ? cJSON_GetObjectItemCaseSensitive(params, "length") : NULL;
    cJSON *data_arr_j = params ? cJSON_GetObjectItemCaseSensitive(params, "data") : NULL;

    if (!cJSON_IsNumber(addr_j) || !cJSON_IsNumber(slot_j) || !cJSON_IsNumber(index_j)) {
        snprintf(response, response_size,
                 "{\"error\":\"dpv1-write requires numeric station_address, slot, index\"}");
        return -1;
    }

    pb_master_instance_t *inst =
        find_master_by_name(cJSON_IsString(master_j) ? master_j->valuestring : NULL);
    if (!inst) {
        snprintf(response, response_size, "{\"error\":\"master not found\"}");
        return -1;
    }

    if (atomic_load(&inst->plugin_state) != PB_STATE_OPERATIONAL) {
        snprintf(response, response_size, "{\"error\":\"master '%s' is not operational\"}",
                 inst->name);
        return -1;
    }

    int station_address = (int)addr_j->valuedouble;
    int slave_index = pb_master_find_slave_by_address(inst, station_address);
    if (slave_index < 0) {
        snprintf(response, response_size,
                 "{\"error\":\"slave with station_address %d not found on master '%s'\"}",
                 station_address, inst->name);
        return -1;
    }

    uint8_t data[PB_MAX_IO_DATA_LEN];
    int data_len;

    if (cJSON_IsArray(data_arr_j)) {
        data_len = cJSON_GetArraySize(data_arr_j);
        if (data_len <= 0 || data_len > PB_MAX_IO_DATA_LEN) {
            snprintf(response, response_size, "{\"error\":\"data array length must be 1-%d\"}",
                     PB_MAX_IO_DATA_LEN);
            return -1;
        }
        for (int i = 0; i < data_len; i++) {
            cJSON *item = cJSON_GetArrayItem(data_arr_j, i);
            data[i] = cJSON_IsNumber(item) ? (uint8_t)item->valuedouble : 0;
        }
    } else if (cJSON_IsNumber(value_j)) {
        pb_data_type_t dt =
            cJSON_IsString(dtype_j) ? pb_parse_data_type(dtype_j->valuestring) : PB_DTYPE_UINT8;

        int length = cJSON_IsNumber(length_j) ? (int)length_j->valuedouble : pb_data_type_size(dt);
        if (length <= 0)
            length = 1;
        if (length > 8) {
            snprintf(response, response_size,
                     "{\"error\":\"'value'-based write supports at most 8 bytes -- use 'data' "
                     "for longer payloads\"}");
            return -1;
        }

        if (pb_value_to_bytes(value_j->valuedouble, dt, (uint8_t)length, data) != 0) {
            snprintf(response, response_size, "{\"error\":\"failed to encode value\"}");
            return -1;
        }
        data_len = length;
    } else {
        snprintf(response, response_size,
                 "{\"error\":\"dpv1-write requires either 'value' (with 'data_type'/'length') "
                 "or a 'data' byte array\"}");
        return -1;
    }

    if (pb_master_dpv1_write(inst, slave_index, (uint8_t)slot_j->valuedouble,
                             (uint8_t)index_j->valuedouble, data, data_len, &g_logger) != 0) {
        snprintf(response, response_size, "{\"error\":\"DPV1 write failed\"}");
        return -1;
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "master", inst->name);
    cJSON_AddNumberToObject(resp, "station_address", station_address);
    cJSON_AddNumberToObject(resp, "slot", slot_j->valuedouble);
    cJSON_AddNumberToObject(resp, "index", index_j->valuedouble);
    cJSON_AddNumberToObject(resp, "length", data_len);
    cJSON_AddStringToObject(resp, "status", "ok");

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
    } else if (strcmp(cmd->valuestring, "list-ports") == 0) {
        result = handle_list_ports_command(response, response_size);
    } else if (strcmp(cmd->valuestring, "dpv1-read") == 0) {
        result = handle_dpv1_read_command(root, response, response_size);
    } else if (strcmp(cmd->valuestring, "dpv1-write") == 0) {
        result = handle_dpv1_write_command(root, response, response_size);
    } else {
        snprintf(response, response_size, "{\"error\":\"unknown command '%s'\"}",
                 cmd->valuestring);
    }

    cJSON_Delete(root);
    return result;
}
