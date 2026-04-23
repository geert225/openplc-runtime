/**
 * @file ethercat_io.c
 * @brief EtherCAT I/O Module — IEC location parsing, channel mapping, and process data exchange
 *
 * Implements the bridge between the SOEM IOmap buffer and the OpenPLC
 * runtime I/O buffers (bool_input/output, byte_input/output, etc.).
 *
 * Flow:
 *  1. At startup, ecat_io_build_channel_map() walks every configured channel,
 *     resolves its IOmap pointer via PDO walking, parses its IEC location,
 *     and stores a compact mapping entry.
 *  2. Each PLC scan cycle:
 *     - cycle_start  → ecat_io_read_inputs()  copies IOmap → PLC inputs
 *     - cycle_end    → ecat_io_write_outputs() copies PLC outputs → IOmap
 */

#include "ethercat_io.h"
#include "ethercat_master.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/*
 * =============================================================================
 * Inline helpers — bit-level IOmap access
 * =============================================================================
 */

static inline uint8_t iomap_read_bit(const uint8_t *ptr, int bit)
{
    return (*ptr >> bit) & 0x01;
}

static inline void iomap_write_bit(uint8_t *ptr, int bit, uint8_t val)
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

int ecat_io_parse_iec_location(const char *loc_str, iec_location_t *loc)
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

    /* Optional bit index — only valid for X (bit) size */
    loc->bit_index = -1;
    if (*p == '.') {
        p++;
        if (!isdigit((unsigned char)*p))
            return -1;
        long bit_val = strtol(p, &endptr, 10);
        if (endptr == p || bit_val < 0 || bit_val > 7)
            return -1;
        if (loc->size != IEC_SIZE_BIT)
            return -1;  /* bit index only meaningful for X size */
        loc->bit_index = (int)bit_val;
        p = endptr;
    } else if (loc->size == IEC_SIZE_BIT) {
        /* X size without explicit bit → default to bit 0 */
        loc->bit_index = 0;
    }

    /* Must be at end of string */
    if (*p != '\0')
        return -1;

    return 0;
}

/*
 * =============================================================================
 * IOmap Offset Calculation (static)
 * =============================================================================
 */

/**
 * @brief Walk PDO entries to find the IOmap offset and bit offset for a channel
 *
 * For a given channel (identified by pdo_index + pdo_entry_index + pdo_entry_subindex),
 * accumulates bit lengths through the PDO list until the target entry is found.
 * Returns an offset relative to the IOmap base rather than an absolute pointer,
 * allowing the same map to work with both the real IOmap and a shadow buffer.
 *
 * @param soem_slave    Live SOEM slave descriptor (provides outputs/inputs base pointer)
 * @param iomap_base    Base address of the IOmap buffer (for offset calculation)
 * @param cfg_slave     Configured slave (provides PDO lists from JSON)
 * @param channel       Channel to locate
 * @param is_output     true if channel is an output (RxPDO), false for input (TxPDO)
 * @param out_offset    [out] byte offset from iomap_base
 * @param out_bit       [out] bit offset within the byte (0-7)
 * @param out_data_type [out] parsed data type from the matching PDO entry (may be NULL)
 * @return 0 on success, -1 if channel's PDO entry was not found
 */
static int calculate_iomap_offset(const ec_slavet *soem_slave,
                                  const uint8_t *iomap_base,
                                  const ecat_slave_t *cfg_slave,
                                  const ecat_channel_t *channel,
                                  bool is_output,
                                  size_t *out_offset,
                                  int *out_bit,
                                  ecat_data_type_t *out_data_type)
{
    /* Select PDO direction:
     *   Output channels -> RxPDOs (master writes to slave) -> soem_slave->outputs
     *   Input channels  -> TxPDOs (slave sends to master)  -> soem_slave->inputs
     */
    const ecat_pdo_t *pdos;
    int pdo_count;
    uint8_t *base_ptr;
    int start_bit;

    if (is_output) {
        pdos      = cfg_slave->rx_pdos;
        pdo_count = cfg_slave->rx_pdo_count;
        base_ptr  = soem_slave->outputs;
        start_bit = soem_slave->Ostartbit;
    } else {
        pdos      = cfg_slave->tx_pdos;
        pdo_count = cfg_slave->tx_pdo_count;
        base_ptr  = soem_slave->inputs;
        start_bit = soem_slave->Istartbit;
    }

    if (!base_ptr)
        return -1;

    /* Calculate byte offset of slave's data area relative to IOmap base */
    size_t slave_base_offset = (size_t)(base_ptr - iomap_base);

    int accumulated_bits = start_bit;

    for (int p = 0; p < pdo_count; p++) {
        const ecat_pdo_t *pdo = &pdos[p];
        for (int e = 0; e < pdo->entry_count; e++) {
            const ecat_pdo_entry_t *entry = &pdo->entries[e];

            /* Check if this is the target entry */
            if (strcmp(pdo->index, channel->pdo_index) == 0 &&
                strcmp(entry->index, channel->pdo_entry_index) == 0 &&
                entry->subindex == channel->pdo_entry_subindex) {
                *out_offset = slave_base_offset + (accumulated_bits / 8);
                *out_bit = accumulated_bits % 8;
                if (out_data_type)
                    *out_data_type = entry->parsed_type;
                return 0;
            }

            accumulated_bits += entry->bit_length;
        }
    }

    return -1;  /* entry not found */
}

/*
 * =============================================================================
 * Data Type Validation Helpers
 * =============================================================================
 */

/**
 * @brief Return a human-readable name for an ecat_data_type_t value
 */
static const char *ecat_data_type_name(ecat_data_type_t dt)
{
    switch (dt) {
    case ECAT_DTYPE_UNKNOWN: return "UNKNOWN";
    case ECAT_DTYPE_BOOL:    return "BOOL";
    case ECAT_DTYPE_INT8:    return "INT8";
    case ECAT_DTYPE_UINT8:   return "UINT8";
    case ECAT_DTYPE_INT16:   return "INT16";
    case ECAT_DTYPE_UINT16:  return "UINT16";
    case ECAT_DTYPE_INT32:   return "INT32";
    case ECAT_DTYPE_UINT32:  return "UINT32";
    case ECAT_DTYPE_INT64:   return "INT64";
    case ECAT_DTYPE_UINT64:  return "UINT64";
    case ECAT_DTYPE_REAL32:  return "REAL32";
    case ECAT_DTYPE_REAL64:  return "REAL64";
    case ECAT_DTYPE_PAD:     return "PAD";
    }
    return "UNKNOWN";
}

/**
 * @brief Return the expected IEC size qualifier for a given data type
 *
 * Used to validate that the IEC location width matches the PDO data type.
 * Returns -1 for types where no specific size is expected (PAD, UNKNOWN).
 */
static int ecat_data_type_expected_iec_size(ecat_data_type_t dt)
{
    switch (dt) {
    case ECAT_DTYPE_BOOL:                          return (int)IEC_SIZE_BIT;
    case ECAT_DTYPE_INT8:   case ECAT_DTYPE_UINT8: return (int)IEC_SIZE_BYTE;
    case ECAT_DTYPE_INT16:  case ECAT_DTYPE_UINT16:return (int)IEC_SIZE_WORD;
    case ECAT_DTYPE_INT32:  case ECAT_DTYPE_UINT32:
    case ECAT_DTYPE_REAL32:                        return (int)IEC_SIZE_DWORD;
    case ECAT_DTYPE_INT64:  case ECAT_DTYPE_UINT64:
    case ECAT_DTYPE_REAL64:                        return (int)IEC_SIZE_LWORD;
    case ECAT_DTYPE_UNKNOWN:
    case ECAT_DTYPE_PAD:                           return -1;
    }
    return -1;
}

/**
 * @brief Return a human-readable name for an iec_size_t value
 */
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

int ecat_io_build_channel_map(const ecat_config_t *config,
                              ecat_channel_map_t *map,
                              ecat_master_instance_t *inst,
                              plugin_runtime_args_t *args,
                              plugin_logger_t *logger)
{
    memset(map, 0, sizeof(*map));

    /* Get IOmap base for offset calculation */
    uint8_t *iomap_base = ecat_master_get_iomap(inst);
    if (!iomap_base) {
        plugin_logger_error(logger, "IOmap base pointer is NULL");
        return -1;
    }

    int errors = 0;
    int mapped = 0;

    for (int s = 0; s < config->slave_count; s++) {
        const ecat_slave_t *cfg_slave = &config->slaves[s];
        int pos = cfg_slave->position;

        /* Get live SOEM slave descriptor */
        const ec_slavet *soem_slave = ecat_master_get_slave(inst, pos);
        if (!soem_slave) {
            plugin_logger_warn(logger,
                "Slave '%s' position %d: not found in SOEM context, skipping channels",
                cfg_slave->name, pos);
            errors++;
            continue;
        }

        for (int c = 0; c < cfg_slave->channel_count; c++) {
            const ecat_channel_t *ch = &cfg_slave->channels[c];

            /* Skip channels without IEC location */
            if (ch->iec_location[0] == '\0')
                continue;

            /* Parse IEC location */
            iec_location_t iec_loc;
            if (ecat_io_parse_iec_location(ch->iec_location, &iec_loc) != 0) {
                plugin_logger_warn(logger,
                    "Slave '%s' channel '%s': invalid IEC location '%s', skipping",
                    cfg_slave->name, ch->name, ch->iec_location);
                errors++;
                continue;
            }

            /* Bounds check against PLC buffer size */
            if (iec_loc.byte_index >= args->buffer_size) {
                plugin_logger_warn(logger,
                    "Slave '%s' channel '%s': IEC location '%s' byte index %d "
                    "exceeds buffer size %d, skipping",
                    cfg_slave->name, ch->name, ch->iec_location,
                    iec_loc.byte_index, args->buffer_size);
                errors++;
                continue;
            }

            /* Determine channel direction from type string */
            bool is_output = (strstr(ch->type, "output") != NULL);

            /* Calculate IOmap offset by walking PDO entries */
            size_t iomap_offset = 0;
            int iomap_bit = 0;
            ecat_data_type_t pdo_data_type = ECAT_DTYPE_UNKNOWN;
            if (calculate_iomap_offset(soem_slave, iomap_base, cfg_slave, ch,
                                       is_output, &iomap_offset, &iomap_bit,
                                       &pdo_data_type) != 0) {
                plugin_logger_warn(logger,
                    "Slave '%s' channel '%s': PDO entry not found "
                    "(pdo=%s entry=%s sub=%d), skipping",
                    cfg_slave->name, ch->name,
                    ch->pdo_index, ch->pdo_entry_index, ch->pdo_entry_subindex);
                errors++;
                continue;
            }

            /* Validate: data type size must match IEC location size qualifier */
            int expected_size = ecat_data_type_expected_iec_size(pdo_data_type);
            if (expected_size >= 0 && expected_size != (int)iec_loc.size) {
                plugin_logger_warn(logger,
                    "Slave '%s' channel '%s': data type %s expects IEC size %s "
                    "but location '%s' uses %s -- data may be truncated or corrupt",
                    cfg_slave->name, ch->name,
                    ecat_data_type_name(pdo_data_type),
                    iec_size_name((iec_size_t)expected_size),
                    ch->iec_location, iec_size_name(iec_loc.size));
            }

            /* Build the map entry */
            ecat_channel_map_entry_t entry;
            entry.iomap_offset     = iomap_offset;
            entry.iomap_bit_offset = iomap_bit;
            entry.bit_length       = ch->bit_length;
            entry.size             = iec_loc.size;
            entry.byte_index       = iec_loc.byte_index;
            entry.bit_index        = iec_loc.bit_index;
            entry.data_type        = pdo_data_type;

            /* Add to the appropriate direction array */
            if (iec_loc.direction == IEC_DIR_INPUT) {
                if (map->input_count < ECAT_MAX_MAP_ENTRIES) {
                    map->inputs[map->input_count++] = entry;
                    mapped++;
                } else {
                    plugin_logger_warn(logger, "Input channel map full (%d entries)",
                                       ECAT_MAX_MAP_ENTRIES);
                    errors++;
                }
            } else {
                if (map->output_count < ECAT_MAX_MAP_ENTRIES) {
                    map->outputs[map->output_count++] = entry;
                    mapped++;
                } else {
                    plugin_logger_warn(logger, "Output channel map full (%d entries)",
                                       ECAT_MAX_MAP_ENTRIES);
                    errors++;
                }
            }

            plugin_logger_debug(logger,
                "  Mapped: slave '%s' ch '%s' [%s] (%s) -> %s byte=%d bit=%d offset=%zu",
                cfg_slave->name, ch->name,
                ecat_data_type_name(pdo_data_type), ch->iec_location,
                (iec_loc.direction == IEC_DIR_INPUT) ? "INPUT" : "OUTPUT",
                iec_loc.byte_index, iec_loc.bit_index, iomap_offset);

            if (pdo_data_type == ECAT_DTYPE_REAL32 ||
                pdo_data_type == ECAT_DTYPE_REAL64) {
                const char *iec_type =
                    (pdo_data_type == ECAT_DTYPE_REAL32) ? "REAL" : "LREAL";
                plugin_logger_info(logger,
                    "Slave '%s' channel '%s': PDO type is %s mapped to %s. "
                    "Declare the PLC variable as %s so the IEEE 754 bit "
                    "pattern is interpreted correctly",
                    cfg_slave->name, ch->name,
                    ecat_data_type_name(pdo_data_type), ch->iec_location,
                    iec_type);
            }
        }
    }

    plugin_logger_info(logger,
        "Channel map built: %d inputs, %d outputs (%d errors)",
        map->input_count, map->output_count, errors);

    return (mapped > 0) ? 0 : -1;
}

/*
 * =============================================================================
 * Per-Cycle I/O Functions
 * =============================================================================
 */

/**
 * @note Thread safety: buffer_mutex must be held by the caller.
 * This is guaranteed by plc_cycle_thread() in plc_state_manager.c
 * which holds buffer_mutex across the entire scan cycle.
 */
void ecat_io_read_inputs(const ecat_channel_map_t *map,
                         const uint8_t *iomap_base,
                         plugin_runtime_args_t *args)
{
    for (int i = 0; i < map->input_count; i++) {
        const ecat_channel_map_entry_t *e = &map->inputs[i];
        const uint8_t *ptr = iomap_base + e->iomap_offset;

        switch (e->size) {
        case IEC_SIZE_BIT:
            if (args->bool_input &&
                args->bool_input[e->byte_index] &&
                args->bool_input[e->byte_index][e->bit_index]) {
                *args->bool_input[e->byte_index][e->bit_index] =
                    iomap_read_bit(ptr, e->iomap_bit_offset);
            }
            break;

        case IEC_SIZE_BYTE:
            if (args->byte_input && args->byte_input[e->byte_index]) {
                *args->byte_input[e->byte_index] = *ptr;
            }
            break;

        case IEC_SIZE_WORD:
            if (args->int_input && args->int_input[e->byte_index]) {
                memcpy(args->int_input[e->byte_index], ptr, 2);
            }
            break;

        case IEC_SIZE_DWORD:
            if (args->dint_input && args->dint_input[e->byte_index]) {
                memcpy(args->dint_input[e->byte_index], ptr, 4);
            }
            break;

        case IEC_SIZE_LWORD:
            if (args->lint_input && args->lint_input[e->byte_index]) {
                memcpy(args->lint_input[e->byte_index], ptr, 8);
            }
            break;
        }
    }
}

/**
 * @note Thread safety: buffer_mutex must be held by the caller.
 * This is guaranteed by plc_cycle_thread() in plc_state_manager.c
 * which holds buffer_mutex across the entire scan cycle.
 */
void ecat_io_write_outputs(const ecat_channel_map_t *map,
                           uint8_t *iomap_base,
                           plugin_runtime_args_t *args)
{
    for (int i = 0; i < map->output_count; i++) {
        const ecat_channel_map_entry_t *e = &map->outputs[i];
        uint8_t *ptr = iomap_base + e->iomap_offset;

        switch (e->size) {
        case IEC_SIZE_BIT:
            if (args->bool_output &&
                args->bool_output[e->byte_index] &&
                args->bool_output[e->byte_index][e->bit_index]) {
                iomap_write_bit(ptr, e->iomap_bit_offset,
                                *args->bool_output[e->byte_index][e->bit_index]);
            }
            break;

        case IEC_SIZE_BYTE:
            if (args->byte_output && args->byte_output[e->byte_index]) {
                *ptr = *args->byte_output[e->byte_index];
            }
            break;

        case IEC_SIZE_WORD:
            if (args->int_output && args->int_output[e->byte_index]) {
                memcpy(ptr, args->int_output[e->byte_index], 2);
            }
            break;

        case IEC_SIZE_DWORD:
            if (args->dint_output && args->dint_output[e->byte_index]) {
                memcpy(ptr, args->dint_output[e->byte_index], 4);
            }
            break;

        case IEC_SIZE_LWORD:
            if (args->lint_output && args->lint_output[e->byte_index]) {
                memcpy(ptr, args->lint_output[e->byte_index], 8);
            }
            break;
        }
    }
}

/*
 * =============================================================================
 * Transfer List Builder and Fast I/O Functions
 * =============================================================================
 */

/**
 * @brief Resolve one channel map entry into a PLC variable pointer
 *
 * Returns the direct pointer to the PLC variable (e.g. &__IW3) for the
 * given channel direction and IEC size, or NULL if the slot is unmapped.
 */
static void *resolve_plc_ptr(const ecat_channel_map_entry_t *e,
                              iec_dir_t direction,
                              plugin_runtime_args_t *args)
{
    if (direction == IEC_DIR_INPUT) {
        switch (e->size) {
        case IEC_SIZE_BIT:
            if (args->bool_input &&
                args->bool_input[e->byte_index] &&
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
            if (args->bool_output &&
                args->bool_output[e->byte_index] &&
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

int ecat_io_build_transfer_list(const ecat_channel_map_t *map,
                                ecat_transfer_list_t *xfer,
                                plugin_runtime_args_t *args,
                                plugin_logger_t *logger)
{
    memset(xfer, 0, sizeof(*xfer));

    int resolved = 0;

    /* Resolve input channels */
    for (int i = 0; i < map->input_count; i++) {
        const ecat_channel_map_entry_t *e = &map->inputs[i];
        void *plc_ptr = resolve_plc_ptr(e, IEC_DIR_INPUT, args);
        if (!plc_ptr)
            continue;

        ecat_transfer_entry_t *t = &xfer->inputs[xfer->input_count++];
        t->plc_ptr         = plc_ptr;
        t->iomap_offset    = e->iomap_offset;
        t->iomap_bit_offset = e->iomap_bit_offset;
        t->byte_count      = iec_size_to_bytes(e->size);
        t->is_bit          = (e->size == IEC_SIZE_BIT);
        resolved++;
    }

    /* Resolve output channels */
    for (int i = 0; i < map->output_count; i++) {
        const ecat_channel_map_entry_t *e = &map->outputs[i];
        void *plc_ptr = resolve_plc_ptr(e, IEC_DIR_OUTPUT, args);
        if (!plc_ptr)
            continue;

        ecat_transfer_entry_t *t = &xfer->outputs[xfer->output_count++];
        t->plc_ptr         = plc_ptr;
        t->iomap_offset    = e->iomap_offset;
        t->iomap_bit_offset = e->iomap_bit_offset;
        t->byte_count      = iec_size_to_bytes(e->size);
        t->is_bit          = (e->size == IEC_SIZE_BIT);
        resolved++;
    }

    plugin_logger_info(logger,
        "Transfer list built: %d inputs, %d outputs (%d resolved)",
        xfer->input_count, xfer->output_count, resolved);

    return resolved > 0 ? resolved : -1;
}

void ecat_io_read_inputs_fast(const ecat_transfer_list_t *xfer,
                              const uint8_t *iomap_base)
{
    for (int i = 0; i < xfer->input_count; i++) {
        const ecat_transfer_entry_t *e = &xfer->inputs[i];
        if (e->is_bit) {
            *(uint8_t *)e->plc_ptr =
                iomap_read_bit(iomap_base + e->iomap_offset, e->iomap_bit_offset);
        } else {
            memcpy(e->plc_ptr, iomap_base + e->iomap_offset, e->byte_count);
        }
    }
}

void ecat_io_write_outputs_fast(const ecat_transfer_list_t *xfer,
                                uint8_t *iomap_base)
{
    for (int i = 0; i < xfer->output_count; i++) {
        const ecat_transfer_entry_t *e = &xfer->outputs[i];
        if (e->is_bit) {
            iomap_write_bit(iomap_base + e->iomap_offset, e->iomap_bit_offset,
                            *(const uint8_t *)e->plc_ptr);
        } else {
            memcpy(iomap_base + e->iomap_offset, e->plc_ptr, e->byte_count);
        }
    }
}
