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
#include "plc_io_cycle.h"
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
extern std::atomic<long>  plc_heartbeat;
extern plugin_driver_t   *plugin_driver;

/* -----------------------------------------------------------------------
 * Per-task storage. Allocated when a program loads, freed on stop.
 * --------------------------------------------------------------------- */
PlcTaskCtx *plc_tasks      = nullptr;
size_t      plc_task_count = 0;

/* The bootstrap thread doesn't run any IEC task body — it does setup,
 * spawns task threads, waits, and joins. We still want crash recovery
 * on it via a separate jmp pair. The active task's ctx is in __thread
 * storage so the signal handler knows which siglongjmp target to use. */
static __thread PlcTaskCtx  *current_task_ctx        = nullptr;
static sigjmp_buf            bootstrap_crash_jmp;
static volatile sig_atomic_t bootstrap_crash_sig     = 0;
static volatile sig_atomic_t bootstrap_holding_mutex = 0;
static volatile sig_atomic_t plc_crash_signal        = 0;

/* SIGUSR1 wakes blocked clock_nanosleep so task threads can observe
 * the STOPPED state and exit. The handler is a no-op — EINTR is the
 * actual mechanism. */
static void sigusr1_noop(int sig) { (void)sig; }

static void plc_crash_handler(int sig)
{
    if (current_task_ctx)
    {
        current_task_ctx->crash_sig = sig;
        plc_crash_signal            = sig;
        siglongjmp(current_task_ctx->crash_jmp, sig);
    }
    if (pthread_equal(pthread_self(), plc_thread))
    {
        bootstrap_crash_sig = sig;
        plc_crash_signal    = sig;
        siglongjmp(bootstrap_crash_jmp, sig);
    }
    /* Unknown thread — restore default and re-raise so we don't
     * silently eat fatal signals from webserver / plugin threads. */
    signal(sig, SIG_DFL);
    raise(sig);
}

/* -----------------------------------------------------------------------
 * Per-task thread function.
 *
 * Phase 6 keeps this minimal: SCHED_FIFO priority elevation, optional
 * CPU affinity, per-thread crash recovery, then a clock_nanosleep loop
 * that calls task->programs[]->run() under the image-tables lock.
 * Phase 7 specializes the fastest task by adding housekeeping pre/post.
 * --------------------------------------------------------------------- */
static void *plc_task_thread(void *arg)
{
    PlcTaskCtx *ctx = static_cast<PlcTaskCtx *>(arg);
    current_task_ctx = ctx;

#if defined(__linux__)
    pthread_setname_np(pthread_self(), ctx->name);

    int rt = ctx->priority;
    if (rt < 1)  rt = 1;
    if (rt > 99) rt = 99;
    sched_param sp{};
    sp.sched_priority = rt;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
    {
        log_warn("[task %s] SCHED_FIFO(%d) failed: %s — running default scheduling",
                 ctx->name, rt, strerror(errno));
    }
    else
    {
        log_info("[task %s] SCHED_FIFO priority %d", ctx->name, rt);
    }

    if (ctx->cpu_affinity_mask != 0)
    {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        for (int cpu = 0; cpu < 64 && cpu < CPU_SETSIZE; ++cpu)
        {
            if (ctx->cpu_affinity_mask & (1ULL << cpu)) CPU_SET(cpu, &cs);
        }
        if (pthread_setaffinity_np(pthread_self(), sizeof cs, &cs) != 0)
        {
            log_warn("[task %s] pthread_setaffinity_np failed: %s",
                     ctx->name, strerror(errno));
        }
    }
#endif

    if (sigsetjmp(ctx->crash_jmp, 1) != 0)
    {
        if (ctx->holding_mutex)
        {
            ctx->holding_mutex = 0;
            pthread_mutex_unlock(image_tables_mutex());
        }
        log_error("[task %s] crashed (signal %d) — entering ERROR state",
                  ctx->name, ctx->crash_sig);
        plc_force_error_state();
        return nullptr;
    }

    auto *task = static_cast<strucpp::TaskInstance *>(ctx->task_handle);

    timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    while (plc_get_state() == PLC_STATE_RUNNING)
    {
        ctx->holding_mutex = 1;
        pthread_mutex_lock(image_tables_mutex());

        /* Fastest task drives the housekeeping window — same calls and
         * same order as the MatIEC-era single-thread runtime, just
         * anchored on whichever task ticks fastest. Other task threads
         * skip the housekeeping and just run their bodies. */
        if (ctx->is_fastest_task)
        {
            scan_cycle_time_start();
            plc_run_io_cycle_pre();
        }

        for (size_t p = 0; p < task->program_count; ++p)
        {
            task->programs[p]->run();
        }

        if (ctx->is_fastest_task)
        {
            plc_run_io_cycle_post();
            scan_cycle_time_end();
        }

        pthread_mutex_unlock(image_tables_mutex());
        ctx->holding_mutex = 0;

        ctx->heartbeat.store((long)time(nullptr), std::memory_order_relaxed);
        ctx->local_tick.fetch_add(1, std::memory_order_relaxed);

        next_wakeup.tv_nsec += (long)(ctx->interval_ns % 1000000000LL);
        next_wakeup.tv_sec  += (time_t)(ctx->interval_ns / 1000000000LL);
        if (next_wakeup.tv_nsec >= 1000000000L)
        {
            next_wakeup.tv_nsec -= 1000000000L;
            next_wakeup.tv_sec  += 1;
        }
#if defined(__linux__)
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
        if (rc == EINTR) continue;
#else
        timespec now, rel;
        clock_gettime(CLOCK_MONOTONIC, &now);
        rel.tv_sec  = next_wakeup.tv_sec  - now.tv_sec;
        rel.tv_nsec = next_wakeup.tv_nsec - now.tv_nsec;
        if (rel.tv_nsec < 0) { rel.tv_sec--; rel.tv_nsec += 1000000000L; }
        if (rel.tv_sec >= 0) nanosleep(&rel, nullptr);
#endif
    }

    log_info("[task %s] stopped after %llu ticks", ctx->name,
             (unsigned long long)ctx->local_tick.load());
    return nullptr;
}

void *plc_cycle_thread(void *arg)
{
    PluginManager *pm = (PluginManager *)arg;

    plc_crash_signal        = 0;
    bootstrap_crash_sig     = 0;
    bootstrap_holding_mutex = 0;

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

    /* SIGUSR1 is used by stop_plc_program() to wake task threads
     * blocked in clock_nanosleep so they observe the STOPPED state. */
    struct sigaction wake_sa;
    std::memset(&wake_sa, 0, sizeof(wake_sa));
    wake_sa.sa_handler = sigusr1_noop;
    sigemptyset(&wake_sa.sa_mask);
    wake_sa.sa_flags = 0;
    sigaction(SIGUSR1, &wake_sa, NULL);

    log_info("Starting main loop");

    pthread_mutex_lock(&state_mutex);
    plc_state = PLC_STATE_RUNNING;
    pthread_mutex_unlock(&state_mutex);
    log_info("PLC State: RUNNING");

    plc_timing_stats.scan_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &timer_start);

    int crash_sig = sigsetjmp(bootstrap_crash_jmp, 1);
    if (crash_sig != 0)
    {
        if (bootstrap_holding_mutex)
        {
            bootstrap_holding_mutex = 0;
            pthread_mutex_unlock(itm);
        }
        const char *sig_name = (crash_sig == SIGFPE)
                                   ? "SIGFPE (arithmetic error, e.g. division by zero)"
                                   : "SIGSEGV (memory access violation)";
        log_error("PLC bootstrap thread crashed with signal %d: %s", crash_sig, sig_name);

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

    /* Allocate per-task contexts and spawn one thread per IEC task. */
    plc_tasks = static_cast<PlcTaskCtx *>(std::calloc(total_tasks, sizeof(PlcTaskCtx)));
    if (!plc_tasks)
    {
        log_error("Failed to allocate plc_tasks array");
        pthread_mutex_lock(&state_mutex);
        plc_state = PLC_STATE_ERROR;
        pthread_mutex_unlock(&state_mutex);
        return NULL;
    }
    plc_task_count = total_tasks;

    {
        size_t flat_idx = 0;
        long   now_t    = (long)time(nullptr);
        for (size_t r = 0; r < cfg->get_resource_count(); ++r)
        {
            for (size_t t = 0; t < resources[r].task_count; ++t)
            {
                PlcTaskCtx *ctx = &plc_tasks[flat_idx];
                auto       &tk  = resources[r].tasks[t];
                ctx->idx               = flat_idx;
                ctx->interval_ns       = tk.interval_ns > 0 ? tk.interval_ns : (int64_t)base_ns;
                ctx->priority          = tk.priority;
                ctx->cpu_affinity_mask = 0;       /* Phase 8 will plumb this from CPU_AFFINITY */
                ctx->is_fastest_task   = false;   /* set below */
                ctx->task_handle       = &tk;
                std::snprintf(ctx->name, sizeof ctx->name, "plc-task-%zu", flat_idx);
                ctx->heartbeat.store(now_t, std::memory_order_relaxed);
                ctx->local_tick.store(0,    std::memory_order_relaxed);
                ++flat_idx;
            }
        }
    }

    /* Pick the fastest task: smallest interval, tie-break by priority,
     * then by declaration order (which is the iteration order above). */
    {
        size_t fastest_idx = 0;
        for (size_t i = 1; i < plc_task_count; ++i)
        {
            PlcTaskCtx *c = &plc_tasks[i];
            PlcTaskCtx *f = &plc_tasks[fastest_idx];
            if (c->interval_ns < f->interval_ns ||
                (c->interval_ns == f->interval_ns && c->priority > f->priority))
            {
                fastest_idx = i;
            }
        }
        plc_tasks[fastest_idx].is_fastest_task = true;
        log_info("PLC: anchoring housekeeping on task %s "
                 "(interval=%lld ns, priority=%d)",
                 plc_tasks[fastest_idx].name,
                 (long long)plc_tasks[fastest_idx].interval_ns,
                 plc_tasks[fastest_idx].priority);
    }

    /* Spawn task threads. */
    for (size_t i = 0; i < plc_task_count; ++i)
    {
        if (pthread_create(&plc_tasks[i].thread, NULL,
                           plc_task_thread, &plc_tasks[i]) != 0)
        {
            log_error("Failed to spawn task %zu thread: %s",
                      i, strerror(errno));
            pthread_mutex_lock(&state_mutex);
            plc_state = PLC_STATE_ERROR;
            pthread_mutex_unlock(&state_mutex);
            return NULL;
        }
    }
    log_info("Spawned %zu PLC task thread(s)", plc_task_count);

    /* Bootstrap thread: wait for state change, then signal+join the
     * task threads. Phase 7 may add the housekeeping window onto the
     * fastest task's thread; Phase 6 keeps the bootstrap quiet. */
    while (plc_get_state() == PLC_STATE_RUNNING)
    {
        timespec poll_sleep;
        poll_sleep.tv_sec  = 1;
        poll_sleep.tv_nsec = 0;
        nanosleep(&poll_sleep, nullptr);
    }

    log_info("Stopping %zu PLC task thread(s)", plc_task_count);
    for (size_t i = 0; i < plc_task_count; ++i)
    {
        pthread_kill(plc_tasks[i].thread, SIGUSR1);
    }
    for (size_t i = 0; i < plc_task_count; ++i)
    {
        pthread_join(plc_tasks[i].thread, nullptr);
    }
    std::free(plc_tasks);
    plc_tasks      = nullptr;
    plc_task_count = 0;

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
