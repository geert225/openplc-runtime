// plc_state_manager.cpp
//
// Phase 5 implementation — single-thread cycle, walks ConfigurationInstance
// via virtual dispatch. Phase 6 will replace the cycle loop with thread-per-task.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <atomic>
#include <cerrno>
#include <csetjmp>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <pthread.h>

extern "C" {
#include "../drivers/plugin_driver.h"
}

// strucpp runtime headers (vendored)
#include "iec_std_lib.hpp"

#include "image_tables.h"
#include "journal_buffer.h"
#include "plc_state_manager.h"
#include "plcapp_manager.h"
#include "scan_cycle_manager.h"
#include "utils/log.h"
#include "utils/utils.h"

static PLCState         plc_state    = PLC_STATE_STOPPED;
static pthread_mutex_t  state_mutex  = PTHREAD_MUTEX_INITIALIZER;

struct timespec  timer_start;
pthread_t        plc_thread;
PluginManager   *plc_program = NULL;

extern plc_timing_stats_t plc_timing_stats;
// plc_heartbeat is `atomic_long` (C11 _Atomic long) in watchdog.c. We
// access it as std::atomic<long> here — bit-compatible storage, same
// linker symbol; the type just needs to be expressed differently in C++.
extern std::atomic<long>  plc_heartbeat;
extern plugin_driver_t   *plugin_driver;

static sigjmp_buf            plc_crash_jmp;
static pthread_t             plc_thread_id;
static volatile sig_atomic_t plc_crash_signal     = 0;
static volatile sig_atomic_t holding_buffer_mutex = 0;

static void plc_crash_handler(int sig)
{
    if (!pthread_equal(pthread_self(), plc_thread_id))
    {
        signal(sig, SIG_DFL);
        raise(sig);
        return;
    }
    plc_crash_signal = sig;
    siglongjmp(plc_crash_jmp, sig);
}

void *plc_cycle_thread(void *arg)
{
    PluginManager *pm = (PluginManager *)arg;

    plc_thread_id    = pthread_self();
    plc_crash_signal = 0;

    if (scan_cycle_manager_init() != 0)
    {
        log_error("Failed to initialize scan cycle manager");
    }

    lock_memory();

    if (symbols_init(pm) != 0)
    {
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        log_error("PLC State: ERROR (failed to resolve .so symbols)");
        return NULL;
    }
    ext_config_init__();

    /* Bind located variables — replaces the MatIEC-era glueVars/setBufferPointers.
     * Then fill any unbound image-table slots with backing buffers. */
    pthread_mutex_t *itm = image_tables_mutex();
    pthread_mutex_lock(itm);
    image_tables_bind_located_vars();
    image_tables_fill_null_pointers();
    pthread_mutex_unlock(itm);

    journal_buffer_ptrs_t journal_ptrs = {
        .bool_input   = bool_input,
        .bool_output  = bool_output,
        .bool_memory  = bool_memory,
        .byte_input   = byte_input,
        .byte_output  = byte_output,
        .int_input    = int_input,
        .int_output   = int_output,
        .int_memory   = int_memory,
        .dint_input   = dint_input,
        .dint_output  = dint_output,
        .dint_memory  = dint_memory,
        .lint_input   = lint_input,
        .lint_output  = lint_output,
        .lint_memory  = lint_memory,
        .buffer_size  = BUFFER_SIZE,
        .image_mutex  = itm,
    };
    if (journal_init(&journal_ptrs) != 0)
    {
        log_error("Failed to initialize journal buffer");
    }
    else
    {
        log_info("Journal buffer initialized");
    }

    if (plugin_driver)
    {
        plugin_driver_start(plugin_driver);
        log_info("[PLUGIN]: Enabled plugins started");
    }

    set_realtime_priority();

    struct sigaction crash_sa;
    std::memset(&crash_sa, 0, sizeof(crash_sa));
    crash_sa.sa_handler = plc_crash_handler;
    sigemptyset(&crash_sa.sa_mask);
    crash_sa.sa_flags = SA_NODEFER;
    sigaction(SIGFPE,  &crash_sa, NULL);
    sigaction(SIGSEGV, &crash_sa, NULL);

    log_info("Starting main loop");

    pthread_mutex_lock(&state_mutex);
    plc_state = PLC_STATE_RUNNING;
    pthread_mutex_unlock(&state_mutex);
    log_info("PLC State: RUNNING");

    plc_timing_stats.scan_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &timer_start);

    int crash_sig = sigsetjmp(plc_crash_jmp, 1);
    if (crash_sig != 0)
    {
        if (holding_buffer_mutex)
        {
            holding_buffer_mutex = 0;
            pthread_mutex_unlock(itm);
        }
        const char *sig_name = (crash_sig == SIGFPE)
                                   ? "SIGFPE (arithmetic error, e.g. division by zero)"
                                   : "SIGSEGV (memory access violation)";
        log_error("PLC program crashed with signal %d: %s", crash_sig, sig_name);
        log_error("The loaded PLC program contains a fatal error. "
                  "Upload a corrected program to recover.");

        signal(SIGFPE,  SIG_DFL);
        signal(SIGSEGV, SIG_DFL);

        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        log_info("PLC State: ERROR");
        return NULL;
    }

    /* Walk the configuration via virtual dispatch and discover the GCD
     * base tick + flat task list. Phase 5 keeps a single-thread cycle
     * that runs every task in round-robin (each task runs every
     * interval/base ticks). Phase 6 will replace this with one thread per
     * task on SCHED_FIFO. */
    auto *cfg = static_cast<strucpp::ConfigurationInstance *>(strucpp_config_handle());
    if (!cfg)
    {
        log_error("PLC: configuration handle is NULL");
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        return NULL;
    }

    /* Compute GCD across all task intervals. */
    unsigned long long base_ns = 0;
    auto *resources = cfg->get_resources();
    size_t total_tasks = 0;
    for (size_t r = 0; r < cfg->get_resource_count(); ++r)
    {
        for (size_t t = 0; t < resources[r].task_count; ++t)
        {
            ++total_tasks;
            unsigned long long ivl =
                (unsigned long long)resources[r].tasks[t].interval_ns;
            if (ivl == 0) ivl = 20000000ULL;
            if (base_ns == 0) base_ns = ivl;
            else
            {
                unsigned long long a = base_ns, b = ivl;
                while (b) { unsigned long long tmp = b; b = a % b; a = tmp; }
                base_ns = a;
            }
        }
    }
    if (base_ns == 0) base_ns = 20000000ULL;
    if (total_tasks == 0)
    {
        log_error("PLC program declares zero tasks — refusing to run");
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        return NULL;
    }
    log_info("PLC base tick: %llu ns across %zu task(s)",
             (unsigned long long)base_ns, total_tasks);

    while (plc_state == PLC_STATE_RUNNING)
    {
        scan_cycle_time_start();
        holding_buffer_mutex = 1;
        pthread_mutex_lock(itm);

        journal_apply_and_clear();
        plugin_driver_cycle_start(plugin_driver);

        /* Round-robin: each task runs every (interval / base) ticks. */
        for (size_t r = 0; r < cfg->get_resource_count(); ++r)
        {
            for (size_t t = 0; t < resources[r].task_count; ++t)
            {
                auto &task = resources[r].tasks[t];
                int64_t ivl = task.interval_ns > 0 ? task.interval_ns : (int64_t)base_ns;
                uint64_t divisor = (uint64_t)ivl / base_ns;
                if (divisor == 0 || (tick__ % divisor) == 0)
                {
                    for (size_t p = 0; p < task.program_count; ++p)
                    {
                        task.programs[p]->run();
                    }
                }
            }
        }
        ext_updateTime();

        plugin_driver_cycle_end(plugin_driver);
        plc_heartbeat.store((long)time(NULL));

        pthread_mutex_unlock(itm);
        holding_buffer_mutex = 0;
        scan_cycle_time_end();

        ++tick__;

        timer_start.tv_nsec += (long)base_ns;
        normalize_timespec(&timer_start);
        sleep_until(&timer_start);
    }

    signal(SIGFPE,  SIG_DFL);
    signal(SIGSEGV, SIG_DFL);

    scan_cycle_manager_cleanup();
    return NULL;
}

extern "C" int load_plc_program(PluginManager *pm)
{
    if (pm == NULL)
    {
        log_error("Failed to load PLC Program: PluginManager is NULL");
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        log_info("PLC State: ERROR");
        return -1;
    }

    if (plugin_manager_load(pm))
    {
        log_info("Loading PLC application");

        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_INIT;
        pthread_mutex_unlock(&state_mutex);
        log_info("PLC State: INIT");

        if (plugin_driver)
        {
            if (plugin_driver_update_config(plugin_driver, "./plugins.conf") == 0)
            {
                plugin_driver_init(plugin_driver);
                log_info("[PLUGIN]: Plugins re-initialized with updated config");
            }
            else
            {
                log_error("[PLUGIN]: Failed to load plugin configuration");
            }
        }

        if (pthread_create(&plc_thread, NULL, plc_cycle_thread, pm) != 0)
        {
            log_error("Failed to create PLC cycle thread");
            pthread_mutex_lock(&state_mutex);
            plc_state = PLC_STATE_ERROR;
            pthread_mutex_unlock(&state_mutex);
            log_info("PLC State: ERROR");
            return -1;
        }

        return 0;
    }
    else
    {
        log_error("Failed to load PLC application");
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_EMPTY;
        pthread_mutex_unlock(&state_mutex);
        log_info("PLC State: EMPTY");
        return -1;
    }
}

extern "C" int unload_plc_program(PluginManager *pm)
{
    if (pm && pm == plc_program)
    {
        PLCState prev_state = plc_get_state();

        if (prev_state != PLC_STATE_ERROR)
        {
            pthread_mutex_lock(&state_mutex);
            plc_state = PLC_STATE_STOPPED;
            pthread_mutex_unlock(&state_mutex);
        }

        pthread_join(plc_thread, NULL);

        journal_cleanup();
        log_info("Journal buffer cleaned up");

        plugin_driver_stop(plugin_driver);

        pthread_mutex_t *itm = image_tables_mutex();
        pthread_mutex_lock(itm);
        image_tables_clear_null_pointers();
        pthread_mutex_unlock(itm);

        void (*python_cleanup)(void);
        *(void **)&python_cleanup = plugin_manager_get_symbol(pm, "python_blocks_cleanup");
        if (python_cleanup) python_cleanup();

        plugin_manager_destroy(pm);
        plc_program = NULL;

        log_info("PLC program unloaded successfully");
        log_info("PLC State: STOPPED");
        return 0;
    }
    else
    {
        log_error("No PLC program loaded or mismatched plugin manager");
        return -1;
    }
}

extern "C" PLCState plc_get_state(void)
{
    pthread_mutex_lock(&state_mutex);
    PLCState s = plc_state;
    pthread_mutex_unlock(&state_mutex);
    return s;
}

extern "C" bool plc_set_state(PLCState new_state)
{
    pthread_mutex_lock(&state_mutex);
    if (plc_state == new_state)
    {
        pthread_mutex_unlock(&state_mutex);
        return false;
    }
    plc_state = new_state;
    pthread_mutex_unlock(&state_mutex);

    if (new_state == PLC_STATE_RUNNING)
    {
        if (plc_program == NULL)
        {
            char *libplc_path = find_libplc_file(libplc_build_dir);
            if (libplc_path == NULL)
            {
                log_error("Failed to find libplc file");
                pthread_mutex_lock(&state_mutex);
                plc_state = PLC_STATE_EMPTY;
                pthread_mutex_unlock(&state_mutex);
                return false;
            }

            plc_program = plugin_manager_create(libplc_path);
            free(libplc_path);

            if (plc_program == NULL)
            {
                log_error("Failed to create PluginManager");
                pthread_mutex_lock(&state_mutex);
                plc_state = PLC_STATE_EMPTY;
                pthread_mutex_unlock(&state_mutex);
                return false;
            }
        }
        if (load_plc_program(plc_program) < 0)
        {
            pthread_mutex_lock(&state_mutex);
            plc_state = PLC_STATE_ERROR;
            pthread_mutex_unlock(&state_mutex);
            return false;
        }
    }
    else if (new_state == PLC_STATE_STOPPED)
    {
        if (plc_program)
        {
            if (unload_plc_program(plc_program) < 0) return false;
        }
    }

    return true;
}

extern "C" void plc_state_manager_cleanup(void)
{
    if (plc_program) unload_plc_program(plc_program);
}

extern "C" void plc_force_error_state(void)
{
    pthread_mutex_lock(&state_mutex);
    plc_state = PLC_STATE_ERROR;
    pthread_mutex_unlock(&state_mutex);
    log_info("PLC State: ERROR");
}

extern "C" int plc_get_crash_signal(void)
{
    return (int)plc_crash_signal;
}
