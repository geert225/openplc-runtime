#include <inttypes.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "scan_cycle_manager.h"
#include "utils/utils.h"

// Use CLOCK_MONOTONIC everywhere to match the clock used by sleep_until()
// (clock_nanosleep with CLOCK_MONOTONIC). Using CLOCK_MONOTONIC_RAW here
// would cause progressive drift against the sleep clock due to NTP slew
// adjustments, eventually leading to false overrun detection after ~30-60
// minutes of continuous operation.
#define OPENPLC_CLOCK CLOCK_MONOTONIC

static uint64_t expected_start_us  = 0;
static uint64_t last_start_us      = 0;
static pthread_mutex_t stats_mutex;

plc_timing_stats_t plc_timing_stats = {.scan_time_min     = INT64_MAX,
                                       .cycle_latency_min = INT64_MAX,
                                       .cycle_time_avg    = 0,
                                       .cycle_time_min    = INT64_MAX,
                                       .cycle_latency_avg = 0,
                                       .scan_count        = 0,
                                       .overruns          = 0};

static uint64_t ts_now_us(void)
{
    struct timespec ts;
    clock_gettime(OPENPLC_CLOCK, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + ts.tv_nsec / 1000;
}

void scan_cycle_time_start(void)
{
    uint64_t now_us = ts_now_us();

    pthread_mutex_lock(&stats_mutex);

    if (plc_timing_stats.scan_count == 0)
    {
        // Ignore full calculations for the first cycle
        expected_start_us = now_us + *ext_common_ticktime__ / 1000; // Convert ns to us
        last_start_us     = now_us;
        plc_timing_stats.scan_count++;

        pthread_mutex_unlock(&stats_mutex);
        return;
    }

    // Calculate cycle time
    int64_t cycle_time_us = now_us - last_start_us;
    if (cycle_time_us < plc_timing_stats.cycle_time_min)
    {
        plc_timing_stats.cycle_time_min = cycle_time_us;
    }
    if (cycle_time_us > plc_timing_stats.cycle_time_max)
    {
        plc_timing_stats.cycle_time_max = cycle_time_us;
    }
    plc_timing_stats.cycle_time_avg +=
        (cycle_time_us - plc_timing_stats.cycle_time_avg) / plc_timing_stats.scan_count;

    // Calculate cycle latency
    int64_t latency_us = (int64_t)(now_us - expected_start_us);
    if (latency_us < plc_timing_stats.cycle_latency_min)
    {
        plc_timing_stats.cycle_latency_min = latency_us;
    }
    if (latency_us > plc_timing_stats.cycle_latency_max)
    {
        plc_timing_stats.cycle_latency_max = latency_us;
    }
    plc_timing_stats.cycle_latency_avg +=
        (latency_us - plc_timing_stats.cycle_latency_avg) / plc_timing_stats.scan_count;

    last_start_us = now_us;
    expected_start_us += *ext_common_ticktime__ / 1000; // Convert ns to us

    plc_timing_stats.scan_count++;

    pthread_mutex_unlock(&stats_mutex);
}

void scan_cycle_time_end(void)
{
    uint64_t now_us = ts_now_us();

    pthread_mutex_lock(&stats_mutex);

    // Calculate scan time
    int64_t scan_time_us = now_us - last_start_us;
    if (scan_time_us < plc_timing_stats.scan_time_min)
    {
        plc_timing_stats.scan_time_min = scan_time_us;
    }
    if (scan_time_us > plc_timing_stats.scan_time_max)
    {
        plc_timing_stats.scan_time_max = scan_time_us;
    }
    plc_timing_stats.scan_time_avg +=
        (scan_time_us - plc_timing_stats.scan_time_avg) / plc_timing_stats.scan_count;

    // Check for overrun
    if (now_us > expected_start_us)
    {
        plc_timing_stats.overruns++;
    }

    pthread_mutex_unlock(&stats_mutex);
}

bool get_timing_stats_snapshot(plc_timing_stats_t *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }

    pthread_mutex_lock(&stats_mutex);
    memcpy(snapshot, &plc_timing_stats, sizeof(plc_timing_stats_t));
    pthread_mutex_unlock(&stats_mutex);

    return snapshot->scan_count > 0;
}

int scan_cycle_manager_init(void)
{
    return init_rt_mutex(&stats_mutex);
}

void scan_cycle_manager_cleanup(void)
{
    pthread_mutex_destroy(&stats_mutex);
}

void scan_cycle_stats_reset(void)
{
    pthread_mutex_lock(&stats_mutex);
    plc_timing_stats = (plc_timing_stats_t){
        .scan_time_min     = INT64_MAX,
        .scan_time_max     = 0,
        .scan_time_avg     = 0,
        .cycle_time_min    = INT64_MAX,
        .cycle_time_max    = 0,
        .cycle_time_avg    = 0,
        .cycle_latency_min = INT64_MAX,
        .cycle_latency_max = 0,
        .cycle_latency_avg = 0,
        .scan_count        = 0,
        .overruns          = 0,
    };
    expected_start_us = 0;
    last_start_us     = 0;
    pthread_mutex_unlock(&stats_mutex);
}

int format_timing_stats_response(char *buffer, size_t buffer_size)
{
    plc_timing_stats_t snapshot;
    bool valid = get_timing_stats_snapshot(&snapshot);

    if (!valid)
    {
        return snprintf(buffer, buffer_size,
                        "STATS:{"
                        "\"scan_count\":0,"
                        "\"scan_time_min\":null,"
                        "\"scan_time_max\":null,"
                        "\"scan_time_avg\":null,"
                        "\"cycle_time_min\":null,"
                        "\"cycle_time_max\":null,"
                        "\"cycle_time_avg\":null,"
                        "\"cycle_latency_min\":null,"
                        "\"cycle_latency_max\":null,"
                        "\"cycle_latency_avg\":null,"
                        "\"overruns\":0"
                        "}\n");
    }

    return snprintf(buffer, buffer_size,
                    "STATS:{"
                    "\"scan_count\":%" PRId64 ","
                    "\"scan_time_min\":%" PRId64 ","
                    "\"scan_time_max\":%" PRId64 ","
                    "\"scan_time_avg\":%" PRId64 ","
                    "\"cycle_time_min\":%" PRId64 ","
                    "\"cycle_time_max\":%" PRId64 ","
                    "\"cycle_time_avg\":%" PRId64 ","
                    "\"cycle_latency_min\":%" PRId64 ","
                    "\"cycle_latency_max\":%" PRId64 ","
                    "\"cycle_latency_avg\":%" PRId64 ","
                    "\"overruns\":%" PRId64 "}\n",
                    snapshot.scan_count, snapshot.scan_time_min, snapshot.scan_time_max,
                    snapshot.scan_time_avg, snapshot.cycle_time_min, snapshot.cycle_time_max,
                    snapshot.cycle_time_avg, snapshot.cycle_latency_min, snapshot.cycle_latency_max,
                    snapshot.cycle_latency_avg, snapshot.overruns);
}
