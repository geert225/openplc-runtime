/**
 * @file journal_buffer.c
 * @brief Journal Buffer Implementation for Race-Condition-Free Plugin Writes
 *
 * This implementation provides:
 * - Static journal buffer with configurable max entries
 * - Thread-safe write operations with mutex protection
 * - Atomic application of journal entries at cycle start
 * - Emergency flush when buffer is full
 * - Last-writer-wins conflict resolution via sequence numbers
 */

#include "journal_buffer.h"
#include "utils/log.h"
#include "utils/utils.h"
#include <stdio.h>
#include <string.h>

/*
 * =============================================================================
 * Static State
 * =============================================================================
 */

/* Journal entries buffer */
static journal_entry_t g_entries[JOURNAL_MAX_ENTRIES];

/* Current number of entries in journal */
static size_t g_count = 0;

/* Next sequence number to assign (auto-increment) */
static uint32_t g_next_sequence = 0;

/* Journal mutex - protects g_entries, g_count, g_next_sequence */
static pthread_mutex_t g_journal_mutex;

/* Buffer pointers for applying entries */
static journal_buffer_ptrs_t g_buffer_ptrs;

/* Initialization flag */
static bool g_initialized = false;

/*
 * =============================================================================
 * Forward Declarations
 * =============================================================================
 */
static void apply_entry(const journal_entry_t *entry);
static void emergency_flush_locked(void);

/*
 * =============================================================================
 * Initialization and Cleanup
 * =============================================================================
 */

int journal_init(const journal_buffer_ptrs_t *buffer_ptrs)
{
    if (buffer_ptrs == NULL) {
        log_error("Journal: buffer_ptrs is NULL");
        return -1;
    }

    if (buffer_ptrs->image_mutex == NULL) {
        log_error("Journal: image_mutex is NULL");
        return -1;
    }

    if (init_rt_mutex(&g_journal_mutex) != 0) {
        fprintf(stderr, "[JOURNAL] Error: failed to initialize mutex\n");
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    /* Copy buffer pointers */
    memcpy(&g_buffer_ptrs, buffer_ptrs, sizeof(journal_buffer_ptrs_t));

    /* Reset journal state */
    g_count = 0;
    g_next_sequence = 0;
    memset(g_entries, 0, sizeof(g_entries));

    g_initialized = true;

    pthread_mutex_unlock(&g_journal_mutex);

    return 0;
}

void journal_cleanup(void)
{
    pthread_mutex_lock(&g_journal_mutex);

    g_initialized = false;
    g_count = 0;
    g_next_sequence = 0;
    memset(&g_buffer_ptrs, 0, sizeof(g_buffer_ptrs));

    pthread_mutex_unlock(&g_journal_mutex);
    pthread_mutex_destroy(&g_journal_mutex);
}

bool journal_is_initialized(void)
{
    bool result;
    pthread_mutex_lock(&g_journal_mutex);
    result = g_initialized;
    pthread_mutex_unlock(&g_journal_mutex);
    return result;
}

/*
 * =============================================================================
 * Write Functions
 * =============================================================================
 */

/**
 * @brief Internal function to add an entry to the journal
 *
 * Must be called with g_journal_mutex held.
 * Handles emergency flush if buffer is full.
 *
 * @return Pointer to the new entry, or NULL on failure
 */
static journal_entry_t *add_entry_locked(void)
{
    /* Check if buffer is full */
    if (g_count >= JOURNAL_MAX_ENTRIES) {
        /* Emergency flush: apply all entries and clear */
        emergency_flush_locked();
    }

    /* Add new entry */
    journal_entry_t *entry = &g_entries[g_count];
    entry->sequence = g_next_sequence++;
    g_count++;

    return entry;
}

int journal_write_bool(journal_buffer_type_t type, uint16_t index,
                       uint8_t bit, bool value)
{
    if (!g_initialized) {
        return -1;
    }

    /* Validate type */
    if (type != JOURNAL_BOOL_INPUT &&
        type != JOURNAL_BOOL_OUTPUT &&
        type != JOURNAL_BOOL_MEMORY) {
        return -1;
    }

    /* Validate bit index */
    if (bit > 7) {
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    journal_entry_t *entry = add_entry_locked();
    if (entry == NULL) {
        pthread_mutex_unlock(&g_journal_mutex);
        return -1;
    }

    entry->buffer_type = (uint8_t)type;
    entry->index = index;
    entry->bit_index = bit;
    entry->value = value ? 1 : 0;

    pthread_mutex_unlock(&g_journal_mutex);
    return 0;
}

int journal_write_byte(journal_buffer_type_t type, uint16_t index,
                       uint8_t value)
{
    if (!g_initialized) {
        return -1;
    }

    /* Validate type */
    if (type != JOURNAL_BYTE_INPUT && type != JOURNAL_BYTE_OUTPUT) {
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    journal_entry_t *entry = add_entry_locked();
    if (entry == NULL) {
        pthread_mutex_unlock(&g_journal_mutex);
        return -1;
    }

    entry->buffer_type = (uint8_t)type;
    entry->index = index;
    entry->bit_index = 0xFF;  /* Not applicable for non-bool types */
    entry->value = value;

    pthread_mutex_unlock(&g_journal_mutex);
    return 0;
}

int journal_write_int(journal_buffer_type_t type, uint16_t index,
                      uint16_t value)
{
    if (!g_initialized) {
        return -1;
    }

    /* Validate type */
    if (type != JOURNAL_INT_INPUT &&
        type != JOURNAL_INT_OUTPUT &&
        type != JOURNAL_INT_MEMORY) {
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    journal_entry_t *entry = add_entry_locked();
    if (entry == NULL) {
        pthread_mutex_unlock(&g_journal_mutex);
        return -1;
    }

    entry->buffer_type = (uint8_t)type;
    entry->index = index;
    entry->bit_index = 0xFF;
    entry->value = value;

    pthread_mutex_unlock(&g_journal_mutex);
    return 0;
}

int journal_write_dint(journal_buffer_type_t type, uint16_t index,
                       uint32_t value)
{
    if (!g_initialized) {
        return -1;
    }

    /* Validate type */
    if (type != JOURNAL_DINT_INPUT &&
        type != JOURNAL_DINT_OUTPUT &&
        type != JOURNAL_DINT_MEMORY) {
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    journal_entry_t *entry = add_entry_locked();
    if (entry == NULL) {
        pthread_mutex_unlock(&g_journal_mutex);
        return -1;
    }

    entry->buffer_type = (uint8_t)type;
    entry->index = index;
    entry->bit_index = 0xFF;
    entry->value = value;

    pthread_mutex_unlock(&g_journal_mutex);
    return 0;
}

int journal_write_lint(journal_buffer_type_t type, uint16_t index,
                       uint64_t value)
{
    if (!g_initialized) {
        return -1;
    }

    /* Validate type */
    if (type != JOURNAL_LINT_INPUT &&
        type != JOURNAL_LINT_OUTPUT &&
        type != JOURNAL_LINT_MEMORY) {
        return -1;
    }

    pthread_mutex_lock(&g_journal_mutex);

    journal_entry_t *entry = add_entry_locked();
    if (entry == NULL) {
        pthread_mutex_unlock(&g_journal_mutex);
        return -1;
    }

    entry->buffer_type = (uint8_t)type;
    entry->index = index;
    entry->bit_index = 0xFF;
    entry->value = value;

    pthread_mutex_unlock(&g_journal_mutex);
    return 0;
}

/*
 * =============================================================================
 * Apply and Clear
 * =============================================================================
 */

/**
 * @brief Apply a single journal entry to the image tables
 *
 * @param entry The entry to apply
 */
static void apply_entry(const journal_entry_t *entry)
{
    uint16_t idx = entry->index;

    /* Bounds check */
    if (idx >= (uint16_t)g_buffer_ptrs.buffer_size) {
        return;
    }

    switch ((journal_buffer_type_t)entry->buffer_type) {
        case JOURNAL_BOOL_INPUT: {
            IEC_BOOL *ptr = g_buffer_ptrs.bool_input[idx][entry->bit_index];
            if (ptr != NULL) {
                *ptr = (IEC_BOOL)(entry->value & 1);
            }
            break;
        }
        case JOURNAL_BOOL_OUTPUT: {
            IEC_BOOL *ptr = g_buffer_ptrs.bool_output[idx][entry->bit_index];
            if (ptr != NULL) {
                *ptr = (IEC_BOOL)(entry->value & 1);
            }
            break;
        }
        case JOURNAL_BOOL_MEMORY: {
            IEC_BOOL *ptr = g_buffer_ptrs.bool_memory[idx][entry->bit_index];
            if (ptr != NULL) {
                *ptr = (IEC_BOOL)(entry->value & 1);
            }
            break;
        }
        case JOURNAL_BYTE_INPUT: {
            IEC_BYTE *ptr = g_buffer_ptrs.byte_input[idx];
            if (ptr != NULL) {
                *ptr = (IEC_BYTE)(entry->value & 0xFF);
            }
            break;
        }
        case JOURNAL_BYTE_OUTPUT: {
            IEC_BYTE *ptr = g_buffer_ptrs.byte_output[idx];
            if (ptr != NULL) {
                *ptr = (IEC_BYTE)(entry->value & 0xFF);
            }
            break;
        }
        case JOURNAL_INT_INPUT: {
            IEC_UINT *ptr = g_buffer_ptrs.int_input[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UINT)(entry->value & 0xFFFF);
            }
            break;
        }
        case JOURNAL_INT_OUTPUT: {
            IEC_UINT *ptr = g_buffer_ptrs.int_output[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UINT)(entry->value & 0xFFFF);
            }
            break;
        }
        case JOURNAL_INT_MEMORY: {
            IEC_UINT *ptr = g_buffer_ptrs.int_memory[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UINT)(entry->value & 0xFFFF);
            }
            break;
        }
        case JOURNAL_DINT_INPUT: {
            IEC_UDINT *ptr = g_buffer_ptrs.dint_input[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UDINT)(entry->value & 0xFFFFFFFF);
            }
            break;
        }
        case JOURNAL_DINT_OUTPUT: {
            IEC_UDINT *ptr = g_buffer_ptrs.dint_output[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UDINT)(entry->value & 0xFFFFFFFF);
            }
            break;
        }
        case JOURNAL_DINT_MEMORY: {
            IEC_UDINT *ptr = g_buffer_ptrs.dint_memory[idx];
            if (ptr != NULL) {
                *ptr = (IEC_UDINT)(entry->value & 0xFFFFFFFF);
            }
            break;
        }
        case JOURNAL_LINT_INPUT: {
            IEC_ULINT *ptr = g_buffer_ptrs.lint_input[idx];
            if (ptr != NULL) {
                *ptr = (IEC_ULINT)entry->value;
            }
            break;
        }
        case JOURNAL_LINT_OUTPUT: {
            IEC_ULINT *ptr = g_buffer_ptrs.lint_output[idx];
            if (ptr != NULL) {
                *ptr = (IEC_ULINT)entry->value;
            }
            break;
        }
        case JOURNAL_LINT_MEMORY: {
            IEC_ULINT *ptr = g_buffer_ptrs.lint_memory[idx];
            if (ptr != NULL) {
                *ptr = (IEC_ULINT)entry->value;
            }
            break;
        }
        default:
            /* Unknown type - ignore */
            break;
    }
}

void journal_apply_and_clear(void)
{
    if (!g_initialized) {
        return;
    }

    pthread_mutex_lock(&g_journal_mutex);

    /* Apply all entries in sequence order (they're already in order) */
    for (size_t i = 0; i < g_count; i++) {
        apply_entry(&g_entries[i]);
    }

    /* Clear journal */
    g_count = 0;
    g_next_sequence = 0;

    pthread_mutex_unlock(&g_journal_mutex);
}

/*
 * =============================================================================
 * Emergency Flush
 * =============================================================================
 */

/**
 * @brief Emergency flush - apply all entries when buffer is full
 *
 * Called when a new write is attempted but the journal buffer is full.
 * Must be called with g_journal_mutex already held.
 *
 * Lock ordering to prevent deadlock:
 * 1. Release journal mutex
 * 2. Acquire image mutex
 * 3. Re-acquire journal mutex
 * 4. Apply entries
 * 5. Release image mutex
 * (Continue holding journal mutex for the pending write)
 */
static void emergency_flush_locked(void)
{
    /* Release journal mutex to respect lock ordering */
    pthread_mutex_unlock(&g_journal_mutex);

    /* Acquire image mutex first (lock ordering) */
    pthread_mutex_lock(g_buffer_ptrs.image_mutex);

    /* Re-acquire journal mutex */
    pthread_mutex_lock(&g_journal_mutex);

    /* Apply all entries */
    for (size_t i = 0; i < g_count; i++) {
        apply_entry(&g_entries[i]);
    }

    /* Clear journal */
    g_count = 0;
    g_next_sequence = 0;

    /* Release image mutex (keep journal mutex for the pending write) */
    pthread_mutex_unlock(g_buffer_ptrs.image_mutex);

    /* Note: We return with g_journal_mutex still held */
}

/*
 * =============================================================================
 * Diagnostics
 * =============================================================================
 */

size_t journal_pending_count(void)
{
    size_t count;
    pthread_mutex_lock(&g_journal_mutex);
    count = g_count;
    pthread_mutex_unlock(&g_journal_mutex);
    return count;
}

uint32_t journal_get_sequence(void)
{
    uint32_t seq;
    pthread_mutex_lock(&g_journal_mutex);
    seq = g_next_sequence;
    pthread_mutex_unlock(&g_journal_mutex);
    return seq;
}
