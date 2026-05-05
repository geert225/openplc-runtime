#include <inttypes.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "plc_state_manager.h"
#include "scan_cycle_manager.h"
#include "utils/utils.h"

// Use CLOCK_MONOTONIC everywhere to match the clock used by sleep_until()
// (clock_nanosleep with CLOCK_MONOTONIC). Using CLOCK_MONOTONIC_RAW here
// would cause progressive drift against the sleep clock due to NTP slew
// adjustments, eventually leading to false overrun detection after ~30-60
// minutes of continuous operation.
#define OPENPLC_CLOCK CLOCK_MONOTONIC

static uint64_t ts_now_us(void)
{
    struct timespec ts;
    clock_gettime(OPENPLC_CLOCK, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + ts.tv_nsec / 1000;
}

int scan_cycle_tracker_init(scan_cycle_tracker_t *tracker, int64_t interval_ns)
{
    if (!tracker) return -1;
    memset(tracker, 0, sizeof(*tracker));
    tracker->stats.scan_time_min     = INT64_MAX;
    tracker->stats.cycle_time_min    = INT64_MAX;
    tracker->stats.cycle_latency_min = INT64_MAX;
    tracker->interval_ns             = interval_ns;
    return init_rt_mutex(&tracker->mutex);
}

void scan_cycle_tracker_cleanup(scan_cycle_tracker_t *tracker)
{
    if (!tracker) return;
    pthread_mutex_destroy(&tracker->mutex);
}

void scan_cycle_tracker_start(scan_cycle_tracker_t *tracker)
{
    if (!tracker) return;
    uint64_t now_us = ts_now_us();

    pthread_mutex_lock(&tracker->mutex);

    if (tracker->stats.scan_count == 0)
    {
        // First cycle: seed `last_start_us` and the next `expected_start_us`
        // anchor; skip latency/cycle-time math for this iteration.
        tracker->expected_start_us = now_us + (uint64_t)(tracker->interval_ns / 1000);
        tracker->last_start_us     = now_us;
        tracker->stats.scan_count++;
        pthread_mutex_unlock(&tracker->mutex);
        return;
    }

    // Cycle time: actual elapsed since previous start
    int64_t cycle_time_us = (int64_t)(now_us - tracker->last_start_us);
    plc_timing_stats_t *s = &tracker->stats;
    if (cycle_time_us < s->cycle_time_min) s->cycle_time_min = cycle_time_us;
    if (cycle_time_us > s->cycle_time_max) s->cycle_time_max = cycle_time_us;
    s->cycle_time_avg += (cycle_time_us - s->cycle_time_avg) / s->scan_count;

    // Cycle latency: how far past the projected wakeup we landed
    int64_t latency_us = (int64_t)(now_us - tracker->expected_start_us);
    if (latency_us < s->cycle_latency_min) s->cycle_latency_min = latency_us;
    if (latency_us > s->cycle_latency_max) s->cycle_latency_max = latency_us;
    s->cycle_latency_avg += (latency_us - s->cycle_latency_avg) / s->scan_count;

    tracker->last_start_us = now_us;
    tracker->expected_start_us += (uint64_t)(tracker->interval_ns / 1000);

    s->scan_count++;
    pthread_mutex_unlock(&tracker->mutex);
}

void scan_cycle_tracker_end(scan_cycle_tracker_t *tracker)
{
    if (!tracker) return;
    uint64_t now_us = ts_now_us();

    pthread_mutex_lock(&tracker->mutex);

    int64_t scan_time_us = (int64_t)(now_us - tracker->last_start_us);
    plc_timing_stats_t *s = &tracker->stats;
    if (scan_time_us < s->scan_time_min) s->scan_time_min = scan_time_us;
    if (scan_time_us > s->scan_time_max) s->scan_time_max = scan_time_us;
    s->scan_time_avg += (scan_time_us - s->scan_time_avg) / s->scan_count;

    if (now_us > tracker->expected_start_us)
    {
        s->overruns++;
    }

    pthread_mutex_unlock(&tracker->mutex);
}

bool scan_cycle_tracker_snapshot(scan_cycle_tracker_t *tracker, plc_timing_stats_t *out)
{
    if (!tracker || !out) return false;
    pthread_mutex_lock(&tracker->mutex);
    memcpy(out, &tracker->stats, sizeof(*out));
    pthread_mutex_unlock(&tracker->mutex);
    return out->scan_count > 0;
}

// Forward declaration to avoid pulling plc_state_manager.h into the
// header. PlcTaskCtx exposes `name` and a `tracker` field.
extern PlcTaskCtx *plc_tasks;
extern size_t      plc_task_count;

static int append_task_entry(char *buffer, size_t buffer_size, size_t offset,
                             const char *name, const plc_timing_stats_t *s,
                             bool valid)
{
    int written;
    if (!valid)
    {
        written = snprintf(buffer + offset, buffer_size - offset,
                           "{\"name\":\"%s\","
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
                           "\"overruns\":0}",
                           name);
    }
    else
    {
        written = snprintf(buffer + offset, buffer_size - offset,
                           "{\"name\":\"%s\","
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
                           "\"overruns\":%" PRId64 "}",
                           name,
                           s->scan_count, s->scan_time_min, s->scan_time_max, s->scan_time_avg,
                           s->cycle_time_min, s->cycle_time_max, s->cycle_time_avg,
                           s->cycle_latency_min, s->cycle_latency_max, s->cycle_latency_avg,
                           s->overruns);
    }
    if (written < 0) return 0;
    return written;
}

int format_timing_stats_response(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) return 0;

    size_t offset = 0;
    int    n;

    n = snprintf(buffer + offset, buffer_size - offset, "STATS:{\"tasks\":[");
    if (n < 0) return 0;
    offset += (size_t)n;

    for (size_t i = 0; i < plc_task_count; ++i)
    {
        if (i > 0)
        {
            if (offset >= buffer_size) break;
            buffer[offset++] = ',';
        }
        plc_timing_stats_t snap;
        bool valid = scan_cycle_tracker_snapshot(&plc_tasks[i].tracker, &snap);
        n = append_task_entry(buffer, buffer_size, offset, plc_tasks[i].name, &snap, valid);
        if (n <= 0) break;
        offset += (size_t)n;
        if (offset >= buffer_size) break;
    }

    n = snprintf(buffer + offset, buffer_size - offset, "]}\n");
    if (n > 0) offset += (size_t)n;
    return (int)offset;
}
