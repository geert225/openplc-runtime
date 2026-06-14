/**
 * @file profinet_io.c
 * @brief PROFINET IO Module -- IEC location parsing, channel mapping, and
 *        cyclic data exchange with the OpenPLC process image
 */

#include "profinet_io.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/*
 * =============================================================================
 * Inline helpers -- bit-level buffer access
 * =============================================================================
 */

static inline uint8_t data_read_bit(const uint8_t *ptr, int bit)
{
    return (*ptr >> bit) & 0x01;
}

static inline void data_write_bit(uint8_t *ptr, int bit, uint8_t val)
{
    if (val)
        *ptr |= (uint8_t)(1 << bit);
    else
        *ptr &= (uint8_t)~(1 << bit);
}

/*
 * =============================================================================
 * IEC Location Parser
 * =============================================================================
 */

int pn_io_parse_iec_location(const char *loc_str, iec_location_t *loc)
{
    if (!loc_str || !loc)
        return -1;

    const char *p = loc_str;

    /* Expect leading '%' */
    if (*p != '%')
        return -1;
    p++;

    /* Direction: I or Q */
    switch (toupper((unsigned char)*p)) {
    case 'I': loc->direction = IEC_DIR_INPUT;  break;
    case 'Q': loc->direction = IEC_DIR_OUTPUT; break;
    default:  return -1;
    }
    p++;

    /* Size qualifier: X, B, W, D, L */
    switch (toupper((unsigned char)*p)) {
    case 'X': loc->size = IEC_SIZE_BIT;   break;
    case 'B': loc->size = IEC_SIZE_BYTE;  break;
    case 'W': loc->size = IEC_SIZE_WORD;  break;
    case 'D': loc->size = IEC_SIZE_DWORD; break;
    case 'L': loc->size = IEC_SIZE_LWORD; break;
    default:  return -1;
    }
    p++;

    /* Byte index (decimal) */
    if (!isdigit((unsigned char)*p))
        return -1;

    char *endptr = NULL;
    long byte_val = strtol(p, &endptr, 10);
    if (endptr == p || byte_val < 0)
        return -1;
    loc->byte_index = (int)byte_val;
    p = endptr;

    /* Optional bit index -- only valid for X (bit) size */
    loc->bit_index = -1;
    if (*p == '.') {
        p++;
        if (!isdigit((unsigned char)*p))
            return -1;
        long bit_val = strtol(p, &endptr, 10);
        if (endptr == p || bit_val < 0 || bit_val > 7)
            return -1;
        if (loc->size != IEC_SIZE_BIT)
            return -1; /* bit index only meaningful for X size */
        loc->bit_index = (int)bit_val;
        p = endptr;
    } else if (loc->size == IEC_SIZE_BIT) {
        /* X size without explicit bit -> default to bit 0 */
        loc->bit_index = 0;
    }

    /* Must be at end of string */
    if (*p != '\0')
        return -1;

    return 0;
}

/*
 * =============================================================================
 * Data Type Validation Helpers
 * =============================================================================
 */

/**
 * @brief Return the expected IEC size qualifier for a given data type
 *
 * Used to validate that the IEC location width matches the configured
 * channel data type. Returns -1 for types where no specific size is
 * expected (PAD, UNKNOWN).
 */
static int pn_data_type_expected_iec_size(pn_data_type_t dt)
{
    switch (dt) {
    case PN_DTYPE_BOOL:                       return (int)IEC_SIZE_BIT;
    case PN_DTYPE_INT8:   case PN_DTYPE_UINT8: return (int)IEC_SIZE_BYTE;
    case PN_DTYPE_INT16:  case PN_DTYPE_UINT16: return (int)IEC_SIZE_WORD;
    case PN_DTYPE_INT32:  case PN_DTYPE_UINT32:
    case PN_DTYPE_REAL32:                     return (int)IEC_SIZE_DWORD;
    case PN_DTYPE_INT64:  case PN_DTYPE_UINT64:
    case PN_DTYPE_REAL64:                     return (int)IEC_SIZE_LWORD;
    case PN_DTYPE_UNKNOWN:
    case PN_DTYPE_PAD:                        return -1;
    }
    return -1;
}

/** Map IEC size qualifier to byte count */
static uint8_t iec_size_to_bytes(iec_size_t sz)
{
    switch (sz) {
    case IEC_SIZE_BIT:   return 1;
    case IEC_SIZE_BYTE:  return 1;
    case IEC_SIZE_WORD:  return 2;
    case IEC_SIZE_DWORD: return 4;
    case IEC_SIZE_LWORD: return 8;
    }
    return 0;
}

/** Return a human-readable name for an iec_size_t value */
static const char *iec_size_name(iec_size_t sz)
{
    switch (sz) {
    case IEC_SIZE_BIT:   return "BIT (X)";
    case IEC_SIZE_BYTE:  return "BYTE (B)";
    case IEC_SIZE_WORD:  return "WORD (W)";
    case IEC_SIZE_DWORD: return "DWORD (D)";
    case IEC_SIZE_LWORD: return "LWORD (L)";
    }
    return "?";
}

/*
 * =============================================================================
 * Channel Map Builder
 * =============================================================================
 */

int pn_io_build_channel_map(const pn_device_t *device, pn_channel_map_t *map,
                             plugin_runtime_args_t *args, plugin_logger_t *logger)
{
    memset(map, 0, sizeof(*map));

    int errors = 0;
    int mapped = 0;

    for (int c = 0; c < device->channel_count; c++) {
        const pn_channel_t *ch = &device->channels[c];

        /* Skip channels without an IEC location */
        if (ch->iec_location[0] == '\0')
            continue;

        iec_location_t iec_loc;
        if (pn_io_parse_iec_location(ch->iec_location, &iec_loc) != 0) {
            plugin_logger_warn(logger,
                "Device '%s' channel '%s': invalid IEC location '%s', skipping",
                device->name, ch->name, ch->iec_location);
            errors++;
            continue;
        }

        /* Bounds check against PLC buffer size */
        if (iec_loc.byte_index >= args->buffer_size) {
            plugin_logger_warn(logger,
                "Device '%s' channel '%s': IEC location '%s' byte index %d "
                "exceeds buffer size %d, skipping",
                device->name, ch->name, ch->iec_location, iec_loc.byte_index,
                args->buffer_size);
            errors++;
            continue;
        }

        /* Bounds check against the device's cyclic data buffer */
        int data_len = (iec_loc.direction == IEC_DIR_INPUT) ? device->input_length
                                                             : device->output_length;
        if (ch->byte_offset < 0 || ch->byte_offset >= data_len) {
            plugin_logger_warn(logger,
                "Device '%s' channel '%s': byte_offset %d exceeds %s data length %d, skipping",
                device->name, ch->name, ch->byte_offset,
                (iec_loc.direction == IEC_DIR_INPUT) ? "input" : "output", data_len);
            errors++;
            continue;
        }

        /* Validate: configured data type size must match IEC location size qualifier */
        int expected_size = pn_data_type_expected_iec_size(ch->data_type);
        if (expected_size >= 0 && expected_size != (int)iec_loc.size) {
            plugin_logger_warn(logger,
                "Device '%s' channel '%s': data type %s expects IEC size %s "
                "but location '%s' uses %s -- data may be truncated or corrupt",
                device->name, ch->name, pn_data_type_to_string(ch->data_type),
                iec_size_name((iec_size_t)expected_size), ch->iec_location,
                iec_size_name(iec_loc.size));
        }

        pn_channel_map_entry_t entry;
        entry.data_byte_offset = ch->byte_offset;
        entry.data_bit_offset  = (iec_loc.size == IEC_SIZE_BIT) ? ch->bit_offset : -1;
        entry.bit_length       = ch->bit_length;
        entry.size             = iec_loc.size;
        entry.byte_index       = iec_loc.byte_index;
        entry.bit_index        = iec_loc.bit_index;
        entry.data_type        = ch->data_type;

        if (iec_loc.direction == IEC_DIR_INPUT) {
            if (map->input_count < PN_MAX_MAP_ENTRIES) {
                map->inputs[map->input_count++] = entry;
                mapped++;
            } else {
                plugin_logger_warn(logger, "Input channel map full (%d entries)",
                                    PN_MAX_MAP_ENTRIES);
                errors++;
            }
        } else {
            if (map->output_count < PN_MAX_MAP_ENTRIES) {
                map->outputs[map->output_count++] = entry;
                mapped++;
            } else {
                plugin_logger_warn(logger, "Output channel map full (%d entries)",
                                    PN_MAX_MAP_ENTRIES);
                errors++;
            }
        }

        plugin_logger_debug(logger,
            "  Mapped: device '%s' ch '%s' [%s] (%s) -> %s data_byte=%d data_bit=%d",
            device->name, ch->name, pn_data_type_to_string(ch->data_type), ch->iec_location,
            (iec_loc.direction == IEC_DIR_INPUT) ? "INPUT" : "OUTPUT", ch->byte_offset,
            ch->bit_offset);
    }

    plugin_logger_info(logger, "Device '%s': channel map built: %d inputs, %d outputs (%d errors)",
                        device->name, map->input_count, map->output_count, errors);

    /* Reject partial maps: a typo in the JSON would leave the PLC running
     * with stale variables and no indication of why. */
    if (errors > 0) {
        plugin_logger_error(logger, "Device '%s': channel map rejected: %d entry/entries failed",
                             device->name, errors);
        return -1;
    }

    return 0;
}

/*
 * =============================================================================
 * Transfer List Builder and Fast I/O Functions
 * =============================================================================
 */

/**
 * @brief Resolve one channel map entry into a PLC variable pointer
 */
static void *resolve_plc_ptr(const pn_channel_map_entry_t *e, iec_dir_t direction,
                              plugin_runtime_args_t *args)
{
    if (direction == IEC_DIR_INPUT) {
        switch (e->size) {
        case IEC_SIZE_BIT:
            if (args->bool_input && args->bool_input[e->byte_index] &&
                args->bool_input[e->byte_index][e->bit_index])
                return args->bool_input[e->byte_index][e->bit_index];
            break;
        case IEC_SIZE_BYTE:
            if (args->byte_input && args->byte_input[e->byte_index])
                return args->byte_input[e->byte_index];
            break;
        case IEC_SIZE_WORD:
            if (args->int_input && args->int_input[e->byte_index])
                return args->int_input[e->byte_index];
            break;
        case IEC_SIZE_DWORD:
            if (args->dint_input && args->dint_input[e->byte_index])
                return args->dint_input[e->byte_index];
            break;
        case IEC_SIZE_LWORD:
            if (args->lint_input && args->lint_input[e->byte_index])
                return args->lint_input[e->byte_index];
            break;
        }
    } else {
        switch (e->size) {
        case IEC_SIZE_BIT:
            if (args->bool_output && args->bool_output[e->byte_index] &&
                args->bool_output[e->byte_index][e->bit_index])
                return args->bool_output[e->byte_index][e->bit_index];
            break;
        case IEC_SIZE_BYTE:
            if (args->byte_output && args->byte_output[e->byte_index])
                return args->byte_output[e->byte_index];
            break;
        case IEC_SIZE_WORD:
            if (args->int_output && args->int_output[e->byte_index])
                return args->int_output[e->byte_index];
            break;
        case IEC_SIZE_DWORD:
            if (args->dint_output && args->dint_output[e->byte_index])
                return args->dint_output[e->byte_index];
            break;
        case IEC_SIZE_LWORD:
            if (args->lint_output && args->lint_output[e->byte_index])
                return args->lint_output[e->byte_index];
            break;
        }
    }
    return NULL;
}

int pn_io_build_transfer_list(const pn_channel_map_t *map, pn_transfer_list_t *xfer,
                               plugin_runtime_args_t *args, plugin_logger_t *logger)
{
    memset(xfer, 0, sizeof(*xfer));

    /* PROFINET input data is published into the %I image through the
     * journal (lock-free, race-free against the IEC tasks). Refuse to
     * build the list if the runtime did not supply the journal writers
     * and we have input channels to map. */
    if (map->input_count > 0 &&
        (!args->journal_write_bool || !args->journal_write_byte || !args->journal_write_int ||
         !args->journal_write_dint || !args->journal_write_lint)) {
        plugin_logger_error(logger,
            "Journal write entry points unavailable; cannot map %d PROFINET input channel(s)",
            map->input_count);
        return -1;
    }

    int resolved = 0;

    /* Resolve input channels */
    for (int i = 0; i < map->input_count; i++) {
        const pn_channel_map_entry_t *e = &map->inputs[i];
        void *plc_ptr = resolve_plc_ptr(e, IEC_DIR_INPUT, args);
        if (!plc_ptr)
            continue;

        pn_transfer_entry_t *t = &xfer->inputs[xfer->input_count++];
        t->plc_ptr          = plc_ptr;
        t->data_byte_offset = e->data_byte_offset;
        t->data_bit_offset  = e->data_bit_offset;
        t->byte_count       = iec_size_to_bytes(e->size);
        t->is_bit           = (e->size == IEC_SIZE_BIT);
        t->journal_index    = e->byte_index;
        t->journal_bit      = (e->size == IEC_SIZE_BIT) ? e->bit_index : 0;
        resolved++;
    }

    /* Resolve output channels */
    for (int i = 0; i < map->output_count; i++) {
        const pn_channel_map_entry_t *e = &map->outputs[i];
        void *plc_ptr = resolve_plc_ptr(e, IEC_DIR_OUTPUT, args);
        if (!plc_ptr)
            continue;

        pn_transfer_entry_t *t = &xfer->outputs[xfer->output_count++];
        t->plc_ptr          = plc_ptr;
        t->data_byte_offset = e->data_byte_offset;
        t->data_bit_offset  = e->data_bit_offset;
        t->byte_count       = iec_size_to_bytes(e->size);
        t->is_bit           = (e->size == IEC_SIZE_BIT);
        resolved++;
    }

    plugin_logger_info(logger, "Transfer list built: %d inputs, %d outputs (%d resolved)",
                        xfer->input_count, xfer->output_count, resolved);

    int total = map->input_count + map->output_count;
    if (resolved < total) {
        plugin_logger_warn(logger,
            "transfer list: %d/%d channels resolved -- %d skipped (no PLC variable bound)",
            resolved, total, total - resolved);
    }
    if (total > 0 && resolved == 0)
        return -1;
    return 0;
}

/* Journal buffer-type ids for the INPUT image. Must match
 * journal_buffer_type_t in journal_buffer.h (mirrored in plugin_types.h). */
#define PN_JOURNAL_BOOL_INPUT 0
#define PN_JOURNAL_BYTE_INPUT 3
#define PN_JOURNAL_INT_INPUT  5
#define PN_JOURNAL_DINT_INPUT 8
#define PN_JOURNAL_LINT_INPUT 11

void pn_io_read_inputs_fast(const pn_transfer_list_t *xfer, const uint8_t *input_data,
                             plugin_runtime_args_t *args)
{
    for (int i = 0; i < xfer->input_count; i++) {
        const pn_transfer_entry_t *e = &xfer->inputs[i];
        const uint8_t *src = input_data + e->data_byte_offset;

        if (e->is_bit) {
            int v = data_read_bit(src, e->data_bit_offset);
            args->journal_write_bool(PN_JOURNAL_BOOL_INPUT, e->journal_index, e->journal_bit, v);
            continue;
        }

        /* Multi-byte PROFINET IO data is copied byte-for-byte into the IEC
         * value. Byte order depends on the device's GSDML (PROFINET
         * commonly uses big-endian "Motorola" order for analog channels);
         * adjust channel byte_offset/ordering in the JSON config to match. */
        switch (e->byte_count) {
        case 1: {
            uint8_t v;
            memcpy(&v, src, 1);
            args->journal_write_byte(PN_JOURNAL_BYTE_INPUT, e->journal_index, v);
            break;
        }
        case 2: {
            uint16_t v;
            memcpy(&v, src, 2);
            args->journal_write_int(PN_JOURNAL_INT_INPUT, e->journal_index, v);
            break;
        }
        case 4: {
            uint32_t v;
            memcpy(&v, src, 4);
            args->journal_write_dint(PN_JOURNAL_DINT_INPUT, e->journal_index, v);
            break;
        }
        case 8: {
            uint64_t v;
            memcpy(&v, src, 8);
            args->journal_write_lint(PN_JOURNAL_LINT_INPUT, e->journal_index, v);
            break;
        }
        default:
            break;
        }
    }
}

void pn_io_write_outputs_fast(const pn_transfer_list_t *xfer, uint8_t *output_data)
{
    for (int i = 0; i < xfer->output_count; i++) {
        const pn_transfer_entry_t *e = &xfer->outputs[i];
        uint8_t *dst = output_data + e->data_byte_offset;

        if (e->is_bit)
            data_write_bit(dst, e->data_bit_offset, *(const uint8_t *)e->plc_ptr);
        else
            memcpy(dst, e->plc_ptr, e->byte_count);
    }
}
