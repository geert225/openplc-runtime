// runtime_v4_entry.cpp
//
// Static C-linkage shim compiled into every user .so. Identical for every
// project — no per-project codegen. Lives here in the runtime repo
// because the build (scripts/compile.sh) is the consumer; the editor's
// upload bundle does not ship this file.
//
// Responsibilities:
//
//   1. Instantiate strucpp::Configuration_CONFIG0 g_config — the actual
//      object the runtime walks. Must have external linkage so
//      generated_debug.cpp's compile-time address-of expressions resolve.
//   2. Export strucpp_get_config() — C-linkage entry the runtime dlsyms
//      to obtain a ConfigurationInstance* pointer.
//   3. Export strucpp_set_locks() — runtime-side image-tables / globals
//      mutex pointers handed in right after dlopen, stored where
//      iec_threading.hpp's lock guards (Phase 8) can find them.
//   4. Export strucpp_get_located_vars / strucpp_get_located_var_count
//      — re-expose strucpp::locatedVars[] (a per-project namespaced
//      symbol) under stable C linkage.
//   5. Activate STRUCPP_V4_DEBUG_EXPORTS_DEFINE — emits the C-linkage
//      strucpp_debug_* PDU helpers from debug_dispatch.hpp.
//   6. Define the MatIEC-era lifecycle / time / md5 globals the runtime
//      still dlsyms for symmetry with v3 host expectations:
//      common_ticktime__, plc_program_md5, config_init__, updateTime.
//      strucpp itself does not emit these — the Arduino sketch defines
//      them too (in StrucppBaremetal/Baremetal.ino).
//
// The shim is small by design. See docs/strucpp-migration/05 in the
// editor repo for the full reasoning.

#define STRUCPP_V4_DEBUG_EXPORTS_DEFINE
#include "debug_dispatch.hpp"
#include "iec_located.hpp"
#include "iec_std_lib.hpp"   // ConfigurationInstance + __CURRENT_TIME_NS
#include "generated.hpp"

#include <cstddef>
#include <cstdint>
#include <pthread.h>

namespace strucpp {
    pthread_mutex_t* g_image_tables_mutex_ptr = nullptr;
    pthread_mutex_t* g_global_vars_mutex_ptr  = nullptr;
}

// External linkage so generated_debug.cpp can reference &g_config.X.Y at
// compile time. Same constraint as the Arduino sketch's g_config.
strucpp::Configuration_CONFIG0 g_config;

extern "C" strucpp::ConfigurationInstance* strucpp_get_config(void) {
    return &g_config;
}

extern "C" void strucpp_set_locks(pthread_mutex_t* image_tables_mutex,
                                  pthread_mutex_t* global_vars_mutex) {
    strucpp::g_image_tables_mutex_ptr = image_tables_mutex;
    strucpp::g_global_vars_mutex_ptr  = global_vars_mutex;
}

// strucpp::locatedVars / locatedVarsCount are top-level externs declared in
// iec_located.hpp and defined per-project by generated.cpp. The runtime
// (loaded once, sees many .so files) can't reach them by mangled name
// portably, so the shim re-exports them via C linkage. Same pattern as
// strucpp_get_config — the runtime walks via these accessors.

extern "C" const strucpp::LocatedVar *strucpp_get_located_vars(void) {
    return strucpp::locatedVars;
}

extern "C" uint32_t strucpp_get_located_var_count(void) {
    return strucpp::locatedVarsCount;
}

// ---------------------------------------------------------------------------
// MatIEC-era lifecycle / time / MD5 symbols.
//
// strucpp's generated.{cpp,hpp} does NOT emit these — they were Config0.c's
// job in the MatIEC pipeline. The runtime still dlsyms them for symmetry,
// so the shim defines them here. The Arduino integration does the
// equivalent inside Baremetal.ino.
// ---------------------------------------------------------------------------

// Cycle period in ns. Default 20 ms; overwritten by config_init__ to the
// GCD of declared task intervals so scan_cycle_manager sees a meaningful
// base tick. Plain extern "C" variable — the runtime takes its address
// via dlsym and dereferences each cycle.
extern "C" unsigned long long common_ticktime__ = 20000000ULL;

// Project MD5. Used by FC 0x45 to let the editor verify it's debugging
// the program it has the source for. Default placeholder — the editor
// can override at .so build time by emitting a small core/generated/
// header that #defines STRUCPP_PLC_PROGRAM_MD5 (TODO: wire this through
// scripts/compile.sh).
#ifndef STRUCPP_PLC_PROGRAM_MD5
#define STRUCPP_PLC_PROGRAM_MD5 "00000000000000000000000000000000"
#endif
extern "C" const char *plc_program_md5 = STRUCPP_PLC_PROGRAM_MD5;

namespace {
    unsigned long long gcd_u64(unsigned long long a, unsigned long long b) {
        while (b) {
            unsigned long long t = b;
            b = a % b;
            a = t;
        }
        return a;
    }
}

// Called by the runtime once per program load, after all static
// initializers have run (Configuration_CONFIG0 g_config; is fully
// constructed by then). We use it to compute the GCD of declared task
// intervals into common_ticktime__ — the scan_cycle stats expect a
// meaningful value there.
extern "C" void config_init__(void) {
    unsigned long long gcd_ns = 0;
    auto *resources = g_config.get_resources();
    for (size_t r = 0; r < g_config.get_resource_count(); ++r) {
        for (size_t t = 0; t < resources[r].task_count; ++t) {
            unsigned long long ivl =
                (unsigned long long)resources[r].tasks[t].interval_ns;
            if (ivl == 0) ivl = 20000000ULL;
            gcd_ns = (gcd_ns == 0) ? ivl : gcd_u64(gcd_ns, ivl);
        }
    }
    if (gcd_ns != 0) common_ticktime__ = gcd_ns;
}

// Advances the strucpp runtime's scan-cycle clock by one base tick.
// Called by the runtime once per cycle (the fastest task's housekeeping
// window invokes it via plc_run_io_cycle_post). CODESYS semantics:
// TIME() returns the same value for the duration of a cycle.
extern "C" void updateTime(void) {
    strucpp::__CURRENT_TIME_NS += static_cast<int64_t>(common_ticktime__);
}
