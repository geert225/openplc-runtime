// image_tables.cpp
//
// Resolves the strucpp .so's exported symbols (configuration accessor,
// locks setter, debug PDU helpers) and walks strucpp::locatedVars[] to
// bind image-table buffer pointers. Plugins read/write through the
// buffer pointers directly under the image-tables mutex.

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <pthread.h>

extern "C" {
#include "include/iec_python.h"
}

// strucpp runtime headers (vendored in core/strucpp_runtime/include/)
// The locatedVars[] descriptor is per-project and lives in the loaded
// .so; the runtime accesses it via C-linkage shim accessors
// (strucpp_get_located_vars / strucpp_get_located_var_count) rather than
// reaching into the strucpp namespace directly.
#include "iec_located.hpp"
#include "iec_std_lib.hpp"  // ConfigurationInstance, ResourceInstance, TaskInstance

#include "image_tables.h"
#include "plcapp_manager.h"
#include "utils/log.h"
#include "utils/utils.h"

// ---------------------------------------------------------------------------
// Image-table storage
// ---------------------------------------------------------------------------
IEC_BOOL *bool_input[BUFFER_SIZE][8];
IEC_BOOL *bool_output[BUFFER_SIZE][8];

IEC_BYTE *byte_input[BUFFER_SIZE];
IEC_BYTE *byte_output[BUFFER_SIZE];

IEC_UINT *int_input[BUFFER_SIZE];
IEC_UINT *int_output[BUFFER_SIZE];

IEC_UDINT *dint_input[BUFFER_SIZE];
IEC_UDINT *dint_output[BUFFER_SIZE];

IEC_ULINT *lint_input[BUFFER_SIZE];
IEC_ULINT *lint_output[BUFFER_SIZE];

IEC_UINT  *int_memory[BUFFER_SIZE];
IEC_UDINT *dint_memory[BUFFER_SIZE];
IEC_ULINT *lint_memory[BUFFER_SIZE];
IEC_BOOL  *bool_memory[BUFFER_SIZE][8];

// ---------------------------------------------------------------------------
// strucpp shim: per-project located-variable descriptor accessors
// (declared as C-linkage in the .so via runtime_v4_entry.cpp).
// ---------------------------------------------------------------------------
namespace {
    using GetLocatedVarsFn  = const strucpp::LocatedVar *(*)(void);
    using GetLocatedCountFn = uint32_t (*)(void);

    GetLocatedVarsFn  ext_strucpp_get_located_vars      = nullptr;
    GetLocatedCountFn ext_strucpp_get_located_var_count = nullptr;
}

// ---------------------------------------------------------------------------
// Resolved .so symbols
// ---------------------------------------------------------------------------
void (*ext_config_init__)(void) = nullptr;
void (*ext_updateTime)(void)    = nullptr;

uint8_t  (*ext_strucpp_debug_array_count)(void)                          = nullptr;
uint16_t (*ext_strucpp_debug_elem_count) (uint8_t)                       = nullptr;
uint16_t (*ext_strucpp_debug_size)       (uint8_t, uint16_t)             = nullptr;
uint8_t  (*ext_strucpp_debug_set)        (uint8_t, uint16_t, bool,
                                          const uint8_t *, uint16_t)     = nullptr;
uint16_t (*ext_strucpp_debug_read)       (uint8_t, uint16_t, uint8_t *)  = nullptr;

namespace {
    using GetConfigFn = strucpp::ConfigurationInstance *(*)(void);
    using SetLocksFn  = void (*)(pthread_mutex_t *, pthread_mutex_t *);

    GetConfigFn ext_strucpp_get_config = nullptr;
    SetLocksFn  ext_strucpp_set_locks  = nullptr;

    strucpp::ConfigurationInstance *g_config_ptr = nullptr;

    pthread_mutex_t g_image_tables_mutex;
    pthread_mutex_t g_global_vars_mutex;
    bool            g_locks_initialized = false;

    int init_recursive_pi_mutex(pthread_mutex_t *m)
    {
        pthread_mutexattr_t attr;
        if (pthread_mutexattr_init(&attr) != 0) return -1;
#if !defined(__CYGWIN__) && !defined(__MSYS__) && !defined(__APPLE__)
        // PI is Linux-only; macOS dev builds get plain recursive mutex.
        pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
#endif
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        int rc = pthread_mutex_init(m, &attr);
        pthread_mutexattr_destroy(&attr);
        return rc;
    }

    void *resolve(PluginManager *pm, const char *name, bool required)
    {
        void *sym = plugin_manager_get_symbol(pm, name);
        if (!sym && required)
        {
            log_error("[strucpp] required symbol '%s' missing from .so", name);
        }
        return sym;
    }
}  // namespace

extern "C" pthread_mutex_t *image_tables_mutex(void)
{
    return &g_image_tables_mutex;
}

extern "C" pthread_mutex_t *global_vars_mutex(void)
{
    return &g_global_vars_mutex;
}

extern "C" void *strucpp_config_handle(void)
{
    return g_config_ptr;
}

extern "C" int symbols_init(PluginManager *pm)
{
    *(void **)&ext_config_init__ = resolve(pm, "config_init__", true);
    *(void **)&ext_updateTime    = resolve(pm, "updateTime",    true);

    *(void **)&ext_common_ticktime__ = plugin_manager_get_symbol(pm, "common_ticktime__");
    *(void **)&ext_plc_program_md5   = plugin_manager_get_symbol(pm, "plc_program_md5");

    *(void **)&ext_strucpp_get_config = resolve(pm, "strucpp_get_config", true);
    *(void **)&ext_strucpp_set_locks  = resolve(pm, "strucpp_set_locks",  true);

    *(void **)&ext_strucpp_get_located_vars      = resolve(pm, "strucpp_get_located_vars",      true);
    *(void **)&ext_strucpp_get_located_var_count = resolve(pm, "strucpp_get_located_var_count", true);

    *(void **)&ext_strucpp_debug_array_count = resolve(pm, "strucpp_debug_array_count", true);
    *(void **)&ext_strucpp_debug_elem_count  = resolve(pm, "strucpp_debug_elem_count",  true);
    *(void **)&ext_strucpp_debug_size        = resolve(pm, "strucpp_debug_size",        true);
    *(void **)&ext_strucpp_debug_set         = resolve(pm, "strucpp_debug_set",         true);
    *(void **)&ext_strucpp_debug_read        = resolve(pm, "strucpp_debug_read",        true);

    if (!ext_config_init__ || !ext_updateTime ||
        !ext_strucpp_get_config || !ext_strucpp_set_locks ||
        !ext_strucpp_get_located_vars || !ext_strucpp_get_located_var_count ||
        !ext_strucpp_debug_array_count || !ext_strucpp_debug_elem_count ||
        !ext_strucpp_debug_size || !ext_strucpp_debug_set ||
        !ext_strucpp_debug_read)
    {
        log_error("[strucpp] failed to resolve all required .so symbols");
        return -1;
    }

    if (!g_locks_initialized)
    {
        if (init_recursive_pi_mutex(&g_image_tables_mutex) != 0 ||
            init_recursive_pi_mutex(&g_global_vars_mutex)  != 0)
        {
            log_error("[strucpp] failed to initialize resource mutexes");
            return -1;
        }
        g_locks_initialized = true;
    }

    ext_strucpp_set_locks(&g_image_tables_mutex, &g_global_vars_mutex);

    g_config_ptr = ext_strucpp_get_config();
    if (!g_config_ptr)
    {
        log_error("[strucpp] strucpp_get_config returned NULL");
        return -1;
    }

    void (*ext_python_loader_set_loggers)(void (*)(const char *, ...),
                                          void (*)(const char *, ...));
    *(void **)&ext_python_loader_set_loggers =
        plugin_manager_get_symbol(pm, "python_loader_set_loggers");
    if (ext_python_loader_set_loggers)
    {
        ext_python_loader_set_loggers(log_info, log_error);
        log_info("[python] loader logging callbacks initialized");
    }

    log_info("[strucpp] symbols resolved (config=%p, debug=hier)",
             (void *)g_config_ptr);
    return 0;
}

void image_tables_bind_located_vars(void)
{
    if (!ext_strucpp_get_located_vars || !ext_strucpp_get_located_var_count)
    {
        log_warn("[image_tables] located-vars accessors unresolved — skip");
        return;
    }

    const strucpp::LocatedVar *lv_array = ext_strucpp_get_located_vars();
    uint32_t lv_count                   = ext_strucpp_get_located_var_count();
    uint32_t bound = 0, skipped = 0;

    for (uint32_t i = 0; i < lv_count; ++i)
    {
        const strucpp::LocatedVar &lv = lv_array[i];

        if (lv.pointer == nullptr)
        {
            log_warn("[image_tables] locatedVars[%u] has NULL pointer "
                     "(area=%u size=%u byte=%u bit=%u)",
                     i, (unsigned)lv.area, (unsigned)lv.size,
                     (unsigned)lv.byte_index, (unsigned)lv.bit_index);
            ++skipped;
            continue;
        }

        if (lv.byte_index >= BUFFER_SIZE)
        {
            log_warn("[image_tables] locatedVars[%u] byte_index %u exceeds "
                     "BUFFER_SIZE %d — skipping",
                     i, (unsigned)lv.byte_index, BUFFER_SIZE);
            ++skipped;
            continue;
        }

        // strucpp stores raw_ptr() pointing at IECVar's underlying primitive
        // storage (the value_ field), which is layout-compatible with the
        // runtime's plain ::IEC_* typedefs in core/src/lib/iec_types.h.
        // Cast unconditionally; the layout is guaranteed by IECVar's
        // raw_ptr() contract.
        switch (lv.area)
        {
        case strucpp::LocatedArea::Input:
            switch (lv.size)
            {
            case strucpp::LocatedSize::Bit:
                if (lv.bit_index < 8)
                    bool_input[lv.byte_index][lv.bit_index] = (::IEC_BOOL *)lv.pointer;
                break;
            case strucpp::LocatedSize::Byte:
                byte_input[lv.byte_index] = (::IEC_BYTE *)lv.pointer;
                break;
            case strucpp::LocatedSize::Word:
                int_input[lv.byte_index] = (::IEC_UINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::DWord:
                dint_input[lv.byte_index] = (::IEC_UDINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::LWord:
                lint_input[lv.byte_index] = (::IEC_ULINT *)lv.pointer;
                break;
            }
            break;

        case strucpp::LocatedArea::Output:
            switch (lv.size)
            {
            case strucpp::LocatedSize::Bit:
                if (lv.bit_index < 8)
                    bool_output[lv.byte_index][lv.bit_index] = (::IEC_BOOL *)lv.pointer;
                break;
            case strucpp::LocatedSize::Byte:
                byte_output[lv.byte_index] = (::IEC_BYTE *)lv.pointer;
                break;
            case strucpp::LocatedSize::Word:
                int_output[lv.byte_index] = (::IEC_UINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::DWord:
                dint_output[lv.byte_index] = (::IEC_UDINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::LWord:
                lint_output[lv.byte_index] = (::IEC_ULINT *)lv.pointer;
                break;
            }
            break;

        case strucpp::LocatedArea::Memory:
            switch (lv.size)
            {
            case strucpp::LocatedSize::Bit:
                if (lv.bit_index < 8)
                    bool_memory[lv.byte_index][lv.bit_index] = (::IEC_BOOL *)lv.pointer;
                break;
            case strucpp::LocatedSize::Word:
                int_memory[lv.byte_index] = (::IEC_UINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::DWord:
                dint_memory[lv.byte_index] = (::IEC_UDINT *)lv.pointer;
                break;
            case strucpp::LocatedSize::LWord:
                lint_memory[lv.byte_index] = (::IEC_ULINT *)lv.pointer;
                break;
            default:
                ++skipped;
                continue;
            }
            break;

        default:
            log_warn("[image_tables] locatedVars[%u] unknown area %u — skipping",
                     i, (unsigned)lv.area);
            ++skipped;
            continue;
        }
        ++bound;
    }

    log_info("[image_tables] bound %u located variables (%u skipped of %u)",
             bound, skipped, lv_count);
}

// ---------------------------------------------------------------------------
// Backing storage for slots not covered by located variables.
// ---------------------------------------------------------------------------
static IEC_BOOL  temp_bool_input[BUFFER_SIZE][8];
static IEC_BOOL  temp_bool_output[BUFFER_SIZE][8];
static IEC_BYTE  temp_byte_input[BUFFER_SIZE];
static IEC_BYTE  temp_byte_output[BUFFER_SIZE];
static IEC_UINT  temp_int_input[BUFFER_SIZE];
static IEC_UINT  temp_int_output[BUFFER_SIZE];
static IEC_UDINT temp_dint_input[BUFFER_SIZE];
static IEC_UDINT temp_dint_output[BUFFER_SIZE];
static IEC_ULINT temp_lint_input[BUFFER_SIZE];
static IEC_ULINT temp_lint_output[BUFFER_SIZE];
static IEC_UINT  temp_int_memory[BUFFER_SIZE];
static IEC_UDINT temp_dint_memory[BUFFER_SIZE];
static IEC_ULINT temp_lint_memory[BUFFER_SIZE];
static IEC_BOOL  temp_bool_memory[BUFFER_SIZE][8];

void image_tables_fill_null_pointers(void)
{
    int filled = 0;
    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        for (int b = 0; b < 8; ++b)
        {
            if (!bool_input[i][b])  { temp_bool_input[i][b]  = 0; bool_input[i][b]  = &temp_bool_input[i][b];  ++filled; }
            if (!bool_output[i][b]) { temp_bool_output[i][b] = 0; bool_output[i][b] = &temp_bool_output[i][b]; ++filled; }
            if (!bool_memory[i][b]) { temp_bool_memory[i][b] = 0; bool_memory[i][b] = &temp_bool_memory[i][b]; ++filled; }
        }
        if (!byte_input[i])  { temp_byte_input[i]  = 0; byte_input[i]  = &temp_byte_input[i];  ++filled; }
        if (!byte_output[i]) { temp_byte_output[i] = 0; byte_output[i] = &temp_byte_output[i]; ++filled; }
        if (!int_input[i])   { temp_int_input[i]   = 0; int_input[i]   = &temp_int_input[i];   ++filled; }
        if (!int_output[i])  { temp_int_output[i]  = 0; int_output[i]  = &temp_int_output[i];  ++filled; }
        if (!dint_input[i])  { temp_dint_input[i]  = 0; dint_input[i]  = &temp_dint_input[i];  ++filled; }
        if (!dint_output[i]) { temp_dint_output[i] = 0; dint_output[i] = &temp_dint_output[i]; ++filled; }
        if (!lint_input[i])  { temp_lint_input[i]  = 0; lint_input[i]  = &temp_lint_input[i];  ++filled; }
        if (!lint_output[i]) { temp_lint_output[i] = 0; lint_output[i] = &temp_lint_output[i]; ++filled; }
        if (!int_memory[i])  { temp_int_memory[i]  = 0; int_memory[i]  = &temp_int_memory[i];  ++filled; }
        if (!dint_memory[i]) { temp_dint_memory[i] = 0; dint_memory[i] = &temp_dint_memory[i]; ++filled; }
        if (!lint_memory[i]) { temp_lint_memory[i] = 0; lint_memory[i] = &temp_lint_memory[i]; ++filled; }
    }
    log_info("[image_tables] filled %d NULL slots with backing buffers", filled);
}

void image_tables_clear_null_pointers(void)
{
    std::memset(bool_input,   0, sizeof(bool_input));
    std::memset(bool_output,  0, sizeof(bool_output));
    std::memset(byte_input,   0, sizeof(byte_input));
    std::memset(byte_output,  0, sizeof(byte_output));
    std::memset(int_input,    0, sizeof(int_input));
    std::memset(int_output,   0, sizeof(int_output));
    std::memset(dint_input,   0, sizeof(dint_input));
    std::memset(dint_output,  0, sizeof(dint_output));
    std::memset(lint_input,   0, sizeof(lint_input));
    std::memset(lint_output,  0, sizeof(lint_output));
    std::memset(int_memory,   0, sizeof(int_memory));
    std::memset(dint_memory,  0, sizeof(dint_memory));
    std::memset(lint_memory,  0, sizeof(lint_memory));
    std::memset(bool_memory,  0, sizeof(bool_memory));

    ext_config_init__ = nullptr;
    ext_updateTime    = nullptr;
    ext_common_ticktime__ = nullptr;
    ext_plc_program_md5   = nullptr;
    ext_strucpp_get_config = nullptr;
    ext_strucpp_set_locks  = nullptr;
    ext_strucpp_debug_array_count = nullptr;
    ext_strucpp_debug_elem_count  = nullptr;
    ext_strucpp_debug_size        = nullptr;
    ext_strucpp_debug_set         = nullptr;
    ext_strucpp_debug_read        = nullptr;
    ext_strucpp_get_located_vars      = nullptr;
    ext_strucpp_get_located_var_count = nullptr;
    g_config_ptr = nullptr;

    log_info("[image_tables] cleared all pointers");
}
