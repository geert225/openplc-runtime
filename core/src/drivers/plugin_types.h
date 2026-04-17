/**
 * @file plugin_types.h
 * @brief Common type definitions for OpenPLC plugins
 *
 * This header defines the essential types and structures shared between
 * the plugin driver system and native plugins. It provides:
 * - Logging function pointer types
 * - The plugin_runtime_args_t structure for runtime buffer access
 *
 * Both Python and native plugins receive a pointer to plugin_runtime_args_t
 * during initialization, giving them access to PLC I/O buffers, mutex
 * functions, and centralized logging.
 */

#ifndef PLUGIN_TYPES_H
#define PLUGIN_TYPES_H

#include "../lib/iec_types.h"
#include <pthread.h>
#include <stdint.h>

/**
 * @brief Logging function pointer types
 *
 * These function pointers are provided to plugins for routing log messages
 * through the central OpenPLC logging system. Messages logged through these
 * functions will appear in the OpenPLC Editor's log viewer.
 */
typedef void (*plugin_log_info_func_t)(const char *fmt, ...);
typedef void (*plugin_log_debug_func_t)(const char *fmt, ...);
typedef void (*plugin_log_warn_func_t)(const char *fmt, ...);
typedef void (*plugin_log_error_func_t)(const char *fmt, ...);

/**
 * @brief Journal write function pointer types
 *
 * These function pointers allow plugins to write to I/O buffers through
 * the journal buffer system, ensuring race-condition-free writes.
 * All writes are applied atomically at the start of the next PLC scan cycle.
 *
 * Buffer type values (matching journal_buffer_type_t):
 *   0=BOOL_INPUT, 1=BOOL_OUTPUT, 2=BOOL_MEMORY
 *   3=BYTE_INPUT, 4=BYTE_OUTPUT
 *   5=INT_INPUT, 6=INT_OUTPUT, 7=INT_MEMORY
 *   8=DINT_INPUT, 9=DINT_OUTPUT, 10=DINT_MEMORY
 *   11=LINT_INPUT, 12=LINT_OUTPUT, 13=LINT_MEMORY
 */
typedef int (*plugin_journal_write_bool_func_t)(int type, int index, int bit, int value);
typedef int (*plugin_journal_write_byte_func_t)(int type, int index, int value);
typedef int (*plugin_journal_write_int_func_t)(int type, int index, int value);
typedef int (*plugin_journal_write_dint_func_t)(int type, int index, unsigned int value);
typedef int (*plugin_journal_write_lint_func_t)(int type, int index, unsigned long long value);

/**
 * @brief Runtime buffer access structure for plugins
 *
 * This structure is passed to plugins during initialization, providing
 * access to:
 * - PLC I/O buffers (bool, byte, int, dint, lint for inputs/outputs/memory)
 * - Mutex functions for thread-safe buffer access
 * - Plugin-specific configuration file path
 * - Buffer size information
 * - Centralized logging functions
 *
 * Plugins should use mutex_take/mutex_give when accessing buffers to ensure
 * thread safety with the PLC scan cycle.
 */
typedef struct
{
    /* Buffer pointers */
    IEC_BOOL *(*bool_input)[8];
    IEC_BOOL *(*bool_output)[8];
    IEC_BYTE **byte_input;
    IEC_BYTE **byte_output;
    IEC_UINT **int_input;
    IEC_UINT **int_output;
    IEC_UDINT **dint_input;
    IEC_UDINT **dint_output;
    IEC_ULINT **lint_input;
    IEC_ULINT **lint_output;
    IEC_UINT **int_memory;
    IEC_UDINT **dint_memory;
    IEC_ULINT **lint_memory;
    IEC_BOOL *(*bool_memory)[8];

    /* Mutex functions for thread-safe buffer access */
    int (*mutex_take)(pthread_mutex_t *mutex);
    int (*mutex_give)(pthread_mutex_t *mutex);
    pthread_mutex_t *buffer_mutex;

    /* Variable access functions */
    void (*get_var_list)(size_t num_vars, size_t *indexes, void **result);
    size_t (*get_var_size)(size_t idx);
    uint16_t (*get_var_count)(void);

    /* Plugin configuration */
    char plugin_specific_config_file_path[256];

    /* Buffer size information */
    int buffer_size;
    int bits_per_buffer;

    /* Logging functions - route messages through central logging system */
    plugin_log_info_func_t log_info;
    plugin_log_debug_func_t log_debug;
    plugin_log_warn_func_t log_warn;
    plugin_log_error_func_t log_error;

    /* Journal write functions - race-condition-free buffer writes */
    plugin_journal_write_bool_func_t journal_write_bool;
    plugin_journal_write_byte_func_t journal_write_byte;
    plugin_journal_write_int_func_t journal_write_int;
    plugin_journal_write_dint_func_t journal_write_dint;
    plugin_journal_write_lint_func_t journal_write_lint;

    /* PLC base tick time in nanoseconds (GCD of all IEC task intervals) */
    unsigned long long common_ticktime_ns;
} plugin_runtime_args_t;

#endif /* PLUGIN_TYPES_H */
