// plc_io_cycle.cpp — per-cycle I/O work, split into pre/post halves
//                    around the fastest IEC task's body.
//
// Decoupled from plc_state_manager.cpp so the housekeeping is in one
// place — same call list the MatIEC-era single-thread runtime made
// once per scan.

#include <atomic>
#include <ctime>

extern "C" {
#include "../drivers/plugin_driver.h"
}

#include "image_tables.h"
#include "journal_buffer.h"
#include "plc_io_cycle.h"
#include "utils/utils.h"

extern std::atomic<long>  plc_heartbeat;
extern plugin_driver_t   *plugin_driver;

extern "C" void plc_run_io_cycle_pre(void)
{
    journal_apply_and_clear();
    if (plugin_driver) plugin_driver_cycle_start(plugin_driver);
}

extern "C" void plc_run_io_cycle_post(void)
{
    if (ext_updateTime) ext_updateTime();
    if (plugin_driver) plugin_driver_cycle_end(plugin_driver);
    plc_heartbeat.store((long)time(nullptr));
    ++tick__;
}
