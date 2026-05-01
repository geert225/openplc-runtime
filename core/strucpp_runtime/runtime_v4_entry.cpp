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
//   4. Activate STRUCPP_V4_DEBUG_EXPORTS_DEFINE — emits the C-linkage
//      strucpp_debug_* PDU helpers from debug_dispatch.hpp.
//
// The shim is small by design. See docs/strucpp-migration/05 in the
// editor repo for the full reasoning.

#define STRUCPP_V4_DEBUG_EXPORTS_DEFINE
#include "debug_dispatch.hpp"
#include "iec_located.hpp"
#include "generated.hpp"

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
