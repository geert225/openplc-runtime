#ifndef IMAGE_TABLES_H
#define IMAGE_TABLES_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#include "../lib/iec_types.h"
#include "plcapp_manager.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BUFFER_SIZE 1024
#define libplc_build_dir "./build"

    /* -------------------------------------------------------------------------
     * Image-table buffers (booleans, bytes, ints, dints, lints, memories).
     *
     * Populated at program-load time by image_tables_bind_located_vars(),
     * which walks strucpp::locatedVars[] and points each slot at the
     * matching IECVar's underlying primitive storage. Plugins read/write
     * these directly under the image-tables mutex.
     * --------------------------------------------------------------------- */

    extern IEC_BOOL *bool_input[BUFFER_SIZE][8];
    extern IEC_BOOL *bool_output[BUFFER_SIZE][8];

    extern IEC_BYTE *byte_input[BUFFER_SIZE];
    extern IEC_BYTE *byte_output[BUFFER_SIZE];

    extern IEC_UINT *int_input[BUFFER_SIZE];
    extern IEC_UINT *int_output[BUFFER_SIZE];

    extern IEC_UDINT *dint_input[BUFFER_SIZE];
    extern IEC_UDINT *dint_output[BUFFER_SIZE];

    extern IEC_ULINT *lint_input[BUFFER_SIZE];
    extern IEC_ULINT *lint_output[BUFFER_SIZE];

    extern IEC_UINT  *int_memory[BUFFER_SIZE];
    extern IEC_UDINT *dint_memory[BUFFER_SIZE];
    extern IEC_ULINT *lint_memory[BUFFER_SIZE];
    extern IEC_BOOL  *bool_memory[BUFFER_SIZE][8];

    /* -------------------------------------------------------------------------
     * Resolved .so symbols (populated by symbols_init).
     *
     * strucpp_advance_time is called once per scan cycle by
     * plc_run_io_cycle_post; it bumps the per-.so __CURRENT_TIME_NS by the
     * runtime-supplied tick. base_tick_ns is owned runtime-side (utils.c)
     * and computed in symbols_init by walking the loaded configuration.
     * --------------------------------------------------------------------- */

    extern void (*ext_strucpp_advance_time)(uint64_t tick_ns);

    /* Hierarchical debug PDU shims (defined inside the .so by
     * debug_dispatch.hpp under STRUCPP_V4_DEBUG_EXPORTS_DEFINE). */
    extern uint8_t  (*ext_strucpp_debug_array_count)(void);
    extern uint16_t (*ext_strucpp_debug_elem_count) (uint8_t arr);
    extern uint16_t (*ext_strucpp_debug_size)       (uint8_t arr, uint16_t elem);
    extern uint8_t  (*ext_strucpp_debug_set)        (uint8_t arr, uint16_t elem,
                                                     bool forcing,
                                                     const uint8_t *bytes,
                                                     uint16_t len);
    extern uint16_t (*ext_strucpp_debug_read)       (uint8_t arr, uint16_t elem,
                                                     uint8_t *dest);

    /* -------------------------------------------------------------------------
     * Symbol resolution.
     *
     * Resolves all required entry points from the dlopen'd .so, including
     * the strucpp shim entries (strucpp_get_config / strucpp_set_locks)
     * the runtime needs to walk the configuration and plumb mutexes.
     * Initializes runtime-owned image-tables and globals mutexes (recursive
     * PI) on first call and hands their pointers to the .so.
     *
     * Returns 0 on success, -1 if anything required is missing.
     * --------------------------------------------------------------------- */
    int symbols_init(PluginManager *pm);

    /* -------------------------------------------------------------------------
     * Walk strucpp::locatedVars[] and point each image-table slot at the
     * corresponding IECVar's underlying primitive storage. Caller must hold
     * the image-tables mutex.
     * --------------------------------------------------------------------- */
    void image_tables_bind_located_vars(void);

    /* -------------------------------------------------------------------------
     * After binding, fill any unbound image-table slots with private
     * backing buffers so plugins reading those addresses don't dereference
     * NULL. Caller must hold the image-tables mutex.
     * --------------------------------------------------------------------- */
    void image_tables_fill_null_pointers(void);

    /* -------------------------------------------------------------------------
     * Reset all image-table pointers to NULL before unloading a program.
     * Caller must hold the image-tables mutex.
     * --------------------------------------------------------------------- */
    void image_tables_clear_null_pointers(void);

    /* -------------------------------------------------------------------------
     * Resource mutex accessors. Returns pointers to the runtime-owned
     * recursive PI mutexes that protect the image tables and the globals.
     * Plugins / the runtime housekeeping use these directly; the codegen
     * lock guards inside the .so lock the same instances via the pointer
     * stash plumbed through strucpp_set_locks().
     * --------------------------------------------------------------------- */
    pthread_mutex_t *image_tables_mutex(void);
    pthread_mutex_t *global_vars_mutex(void);

    /* -------------------------------------------------------------------------
     * Returns the cached strucpp::ConfigurationInstance* (as void* — the
     * runtime's .cpp callers static_cast to the right type). NULL until
     * symbols_init() succeeds; reset to NULL on image_tables_clear_null_pointers().
     * --------------------------------------------------------------------- */
    void *strucpp_config_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* IMAGE_TABLES_H */
