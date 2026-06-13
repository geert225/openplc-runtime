/**
 * @file profibus_config.c
 * @brief Profibus DP Plugin Configuration Parser (cJSON-based)
 */

#include "profibus_config.h"
#include "cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

static plugin_logger_t *g_cfg_logger = NULL;

void pb_config_set_logger(plugin_logger_t *logger)
{
    g_cfg_logger = logger;
}

static void cfg_log_error(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (g_cfg_logger && g_cfg_logger->is_valid)
        plugin_logger_error(g_cfg_logger, "%s", buf);
    else
        fprintf(stderr, "[PROFIBUS_CONFIG] ERROR: %s\n", buf);
}

static void cfg_log_warn(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (g_cfg_logger && g_cfg_logger->is_valid)
        plugin_logger_warn(g_cfg_logger, "%s", buf);
    else
        fprintf(stderr, "[PROFIBUS_CONFIG] WARN: %s\n", buf);
}

/*
 * =============================================================================
 * JSON helpers
 * =============================================================================
 */

static int strcasecmp_local(const char *a, const char *b)
{
    while (*a && *b) {
        int ca = tolower((unsigned char)*a);
        int cb = tolower((unsigned char)*b);
        if (ca != cb)
            return ca - cb;
        a++;
        b++;
    }
    return (unsigned char)*a - (unsigned char)*b;
}

static void safe_strcpy(char *dst, size_t dstsize, const char *src)
{
    if (!dst || dstsize == 0)
        return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    size_t len = strlen(src);
    if (len >= dstsize)
        len = dstsize - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static const char *get_string(const cJSON *obj, const char *key, const char *def)
{
    if (!obj)
        return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (item && cJSON_IsString(item) && item->valuestring)
        return item->valuestring;
    return def;
}

static int get_int(const cJSON *obj, const char *key, int def)
{
    if (!obj)
        return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (item && cJSON_IsNumber(item))
        return item->valueint;
    return def;
}

static double get_number(const cJSON *obj, const char *key, double def)
{
    if (!obj)
        return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (item && cJSON_IsNumber(item))
        return item->valuedouble;
    return def;
}

static bool get_bool(const cJSON *obj, const char *key, bool def)
{
    if (!obj)
        return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (item && cJSON_IsBool(item))
        return cJSON_IsTrue(item) ? true : false;
    return def;
}

/** Accepts either a JSON number or a hex/decimal string (e.g. "0x806A"). */
static uint32_t get_hex_or_number(const cJSON *obj, const char *key, uint32_t def)
{
    if (!obj)
        return def;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!item)
        return def;
    if (cJSON_IsNumber(item))
        return (uint32_t)item->valuedouble;
    if (cJSON_IsString(item) && item->valuestring)
        return (uint32_t)strtoul(item->valuestring, NULL, 0);
    return def;
}

/**
 * @brief Parse a JSON array of bytes (numbers or "0xNN" strings) into @p out.
 * @return number of bytes parsed (0 if absent/empty), -1 if it would overflow max_len.
 */
static int parse_byte_array(const cJSON *obj, const char *key, uint8_t *out, int max_len)
{
    if (!obj)
        return 0;
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!arr || !cJSON_IsArray(arr))
        return 0;

    int count = 0;
    const cJSON *elem;
    cJSON_ArrayForEach(elem, arr)
    {
        if (count >= max_len)
            return -1;
        if (cJSON_IsNumber(elem))
            out[count] = (uint8_t)((unsigned)elem->valuedouble & 0xFFu);
        else if (cJSON_IsString(elem) && elem->valuestring)
            out[count] = (uint8_t)(strtoul(elem->valuestring, NULL, 0) & 0xFFu);
        else
            continue;
        count++;
    }
    return count;
}

/*
 * =============================================================================
 * Data type parsing
 * =============================================================================
 */

pb_data_type_t pb_parse_data_type(const char *str)
{
    if (!str)
        return PB_DTYPE_UNKNOWN;

    if (strcasecmp_local(str, "BOOL") == 0)
        return PB_DTYPE_BOOL;
    if (strcasecmp_local(str, "INT8") == 0 || strcasecmp_local(str, "SINT") == 0)
        return PB_DTYPE_INT8;
    if (strcasecmp_local(str, "UINT8") == 0 || strcasecmp_local(str, "USINT") == 0 ||
        strcasecmp_local(str, "BYTE") == 0)
        return PB_DTYPE_UINT8;
    if (strcasecmp_local(str, "INT16") == 0 || strcasecmp_local(str, "INT") == 0)
        return PB_DTYPE_INT16;
    if (strcasecmp_local(str, "UINT16") == 0 || strcasecmp_local(str, "UINT") == 0 ||
        strcasecmp_local(str, "WORD") == 0)
        return PB_DTYPE_UINT16;
    if (strcasecmp_local(str, "INT32") == 0 || strcasecmp_local(str, "DINT") == 0)
        return PB_DTYPE_INT32;
    if (strcasecmp_local(str, "UINT32") == 0 || strcasecmp_local(str, "UDINT") == 0 ||
        strcasecmp_local(str, "DWORD") == 0)
        return PB_DTYPE_UINT32;
    if (strcasecmp_local(str, "INT64") == 0 || strcasecmp_local(str, "LINT") == 0)
        return PB_DTYPE_INT64;
    if (strcasecmp_local(str, "UINT64") == 0 || strcasecmp_local(str, "ULINT") == 0 ||
        strcasecmp_local(str, "LWORD") == 0)
        return PB_DTYPE_UINT64;
    if (strcasecmp_local(str, "REAL") == 0 || strcasecmp_local(str, "REAL32") == 0 ||
        strcasecmp_local(str, "FLOAT") == 0)
        return PB_DTYPE_REAL32;
    if (strcasecmp_local(str, "LREAL") == 0 || strcasecmp_local(str, "REAL64") == 0 ||
        strcasecmp_local(str, "DOUBLE") == 0)
        return PB_DTYPE_REAL64;
    if (strcasecmp_local(str, "PAD") == 0)
        return PB_DTYPE_PAD;

    return PB_DTYPE_UNKNOWN;
}

int pb_data_type_size(pb_data_type_t dt)
{
    switch (dt) {
    case PB_DTYPE_BOOL:
    case PB_DTYPE_INT8:
    case PB_DTYPE_UINT8:
    case PB_DTYPE_PAD:
        return 1;
    case PB_DTYPE_INT16:
    case PB_DTYPE_UINT16:
        return 2;
    case PB_DTYPE_INT32:
    case PB_DTYPE_UINT32:
    case PB_DTYPE_REAL32:
        return 4;
    case PB_DTYPE_INT64:
    case PB_DTYPE_UINT64:
    case PB_DTYPE_REAL64:
        return 8;
    default:
        return 0;
    }
}

const char *pb_data_type_to_string(pb_data_type_t dt)
{
    switch (dt) {
    case PB_DTYPE_BOOL:   return "BOOL";
    case PB_DTYPE_INT8:   return "INT8";
    case PB_DTYPE_UINT8:  return "UINT8";
    case PB_DTYPE_INT16:  return "INT16";
    case PB_DTYPE_UINT16: return "UINT16";
    case PB_DTYPE_INT32:  return "INT32";
    case PB_DTYPE_UINT32: return "UINT32";
    case PB_DTYPE_INT64:  return "INT64";
    case PB_DTYPE_UINT64: return "UINT64";
    case PB_DTYPE_REAL32: return "REAL32";
    case PB_DTYPE_REAL64: return "REAL64";
    case PB_DTYPE_PAD:    return "PAD";
    default:              return "UNKNOWN";
    }
}

const char *pb_state_to_string(pb_plugin_state_t state)
{
    switch (state) {
    case PB_STATE_IDLE:           return "IDLE";
    case PB_STATE_CONNECTING:     return "CONNECTING";
    case PB_STATE_PARAMETERIZING: return "PARAMETERIZING";
    case PB_STATE_OPERATIONAL:    return "OPERATIONAL";
    case PB_STATE_RECOVERING:     return "RECOVERING";
    case PB_STATE_ERROR:          return "ERROR";
    case PB_STATE_STOPPED:        return "STOPPED";
    default:                      return "UNKNOWN";
    }
}

const char *pb_slave_state_to_string(pb_slave_state_t state)
{
    switch (state) {
    case PB_SLAVE_OFFLINE:       return "OFFLINE";
    case PB_SLAVE_WAIT_DIAG1:    return "WAIT_DIAG1";
    case PB_SLAVE_SET_PRM:       return "SET_PRM";
    case PB_SLAVE_CHK_CFG:       return "CHK_CFG";
    case PB_SLAVE_WAIT_DIAG2:    return "WAIT_DIAG2";
    case PB_SLAVE_DATA_EXCHANGE: return "DATA_EXCHANGE";
    case PB_SLAVE_FAULT:         return "FAULT";
    default:                     return "UNKNOWN";
    }
}

/*
 * =============================================================================
 * Section parsers
 * =============================================================================
 */

static int parse_parity(const char *str)
{
    if (!str)
        return PB_PARITY_EVEN;
    if (strcasecmp_local(str, "none") == 0)
        return PB_PARITY_NONE;
    if (strcasecmp_local(str, "odd") == 0)
        return PB_PARITY_ODD;
    return PB_PARITY_EVEN; /* Profibus DP default: 8E1 */
}

static void parse_master_section(const cJSON *master_obj, pb_master_config_t *out)
{
    safe_strcpy(out->device, sizeof(out->device), get_string(master_obj, "device", out->device));
    out->baudrate = get_int(master_obj, "baudrate", out->baudrate);
    out->parity   = (pb_parity_t)parse_parity(get_string(master_obj, "parity", "even"));
    out->stop_bits = get_int(master_obj, "stop_bits", out->stop_bits);

    out->station_address = get_int(master_obj, "station_address", out->station_address);
    out->highest_station_address =
        get_int(master_obj, "highest_station_address", out->highest_station_address);

    out->rs485_rts_control = get_bool(master_obj, "rs485_rts_control", out->rs485_rts_control);
    out->rts_delay_us      = get_int(master_obj, "rts_delay_us", out->rts_delay_us);

    out->slot_time_us      = get_int(master_obj, "slot_time_us", out->slot_time_us);
    out->gap_update_factor = get_int(master_obj, "gap_update_factor", out->gap_update_factor);
    out->max_retry_limit   = get_int(master_obj, "max_retry_limit", out->max_retry_limit);

    out->cycle_time_us = get_int(master_obj, "cycle_time_us", out->cycle_time_us);
    out->task_priority = get_int(master_obj, "task_priority", out->task_priority);

    safe_strcpy(out->log_level, sizeof(out->log_level),
                get_string(master_obj, "log_level", out->log_level));
}

static void parse_diagnostics_section(const cJSON *diag_obj, pb_diagnostics_config_t *out)
{
    out->log_connections = get_bool(diag_obj, "log_connections", out->log_connections);
    out->log_errors      = get_bool(diag_obj, "log_errors", out->log_errors);
    out->max_log_entries = get_int(diag_obj, "max_log_entries", out->max_log_entries);
    out->status_update_interval_ms =
        get_int(diag_obj, "status_update_interval_ms", out->status_update_interval_ms);
}

static int parse_channel(const cJSON *ch_obj, pb_channel_t *out, int index)
{
    memset(out, 0, sizeof(*out));
    out->index = get_int(ch_obj, "index", index);
    safe_strcpy(out->name, sizeof(out->name), get_string(ch_obj, "name", ""));
    safe_strcpy(out->type, sizeof(out->type), get_string(ch_obj, "type", ""));
    out->bit_length = (uint8_t)get_int(ch_obj, "bit_length", 1);
    safe_strcpy(out->iec_location, sizeof(out->iec_location), get_string(ch_obj, "iec_location", ""));
    out->byte_offset = get_int(ch_obj, "byte_offset", -1);
    out->bit_offset  = get_int(ch_obj, "bit_offset", -1);
    out->data_type   = pb_parse_data_type(get_string(ch_obj, "data_type", "BOOL"));

    if (out->iec_location[0] == '\0') {
        cfg_log_error("Channel '%s' (index %d) is missing iec_location", out->name, out->index);
        return -1;
    }
    if (out->byte_offset < 0) {
        cfg_log_error("Channel '%s' (index %d) is missing byte_offset", out->name, out->index);
        return -1;
    }
    return 0;
}

static int parse_dpv1_param(const cJSON *p_obj, pb_dpv1_param_t *out)
{
    memset(out, 0, sizeof(*out));
    safe_strcpy(out->name, sizeof(out->name), get_string(p_obj, "name", ""));
    out->slot   = (uint8_t)get_int(p_obj, "slot", 0);
    out->index  = (uint8_t)get_int(p_obj, "index", 0);
    out->data_type = pb_parse_data_type(get_string(p_obj, "data_type", "UINT8"));
    out->length = (uint8_t)get_int(p_obj, "length", pb_data_type_size(out->data_type));
    out->writable = get_bool(p_obj, "writable", false);

    const cJSON *init = cJSON_GetObjectItemCaseSensitive(p_obj, "initial_value");
    if (init && cJSON_IsNumber(init)) {
        out->has_initial_value = true;
        out->initial_value = init->valuedouble;
    }

    if (out->length == 0 || out->length > 8) {
        cfg_log_error("DPV1 parameter '%s' has invalid length %u", out->name, out->length);
        return -1;
    }
    return 0;
}

static int parse_slave(const cJSON *slave_obj, pb_slave_t *out)
{
    memset(out, 0, sizeof(*out));

    out->station_address = get_int(slave_obj, "station_address", -1);
    safe_strcpy(out->name, sizeof(out->name), get_string(slave_obj, "name", ""));
    out->ident_number = (uint16_t)get_hex_or_number(slave_obj, "ident_number", 0);
    out->group = (uint8_t)get_int(slave_obj, "group", 1);

    out->watchdog_enabled = get_bool(slave_obj, "watchdog_enabled", true);
    out->watchdog_ms      = get_int(slave_obj, "watchdog_ms", 400);

    out->min_tsdr = get_int(slave_obj, "min_tsdr", 11);

    int n = parse_byte_array(slave_obj, "user_prm_data", out->user_prm_data, PB_MAX_PRM_DATA_LEN);
    if (n < 0) {
        cfg_log_error("Slave '%s': user_prm_data exceeds %d bytes", out->name, PB_MAX_PRM_DATA_LEN);
        return -1;
    }
    out->user_prm_data_len = n;

    n = parse_byte_array(slave_obj, "cfg_data", out->cfg_data, PB_MAX_CFG_DATA_LEN);
    if (n < 0) {
        cfg_log_error("Slave '%s': cfg_data exceeds %d bytes", out->name, PB_MAX_CFG_DATA_LEN);
        return -1;
    }
    out->cfg_data_len = n;

    out->input_length  = get_int(slave_obj, "input_length", 0);
    out->output_length = get_int(slave_obj, "output_length", 0);
    out->strict        = get_bool(slave_obj, "strict", true);

    if (out->station_address < 0 || out->station_address > 125) {
        cfg_log_error("Slave '%s' has invalid station_address %d", out->name, out->station_address);
        return -1;
    }
    if (out->input_length > PB_MAX_IO_DATA_LEN || out->output_length > PB_MAX_IO_DATA_LEN) {
        cfg_log_error("Slave '%s' I/O length exceeds %d bytes", out->name, PB_MAX_IO_DATA_LEN);
        return -1;
    }

    /* Channels */
    const cJSON *channels = cJSON_GetObjectItemCaseSensitive(slave_obj, "channels");
    if (channels && cJSON_IsArray(channels)) {
        int idx = 0;
        const cJSON *ch;
        cJSON_ArrayForEach(ch, channels)
        {
            if (out->channel_count >= PB_MAX_CHANNELS) {
                cfg_log_error("Slave '%s' has more than %d channels", out->name, PB_MAX_CHANNELS);
                return -1;
            }
            if (parse_channel(ch, &out->channels[out->channel_count], idx) != 0)
                return -1;
            out->channel_count++;
            idx++;
        }
    }

    /* DPV1 parameters */
    const cJSON *params = cJSON_GetObjectItemCaseSensitive(slave_obj, "dpv1_parameters");
    if (params && cJSON_IsArray(params)) {
        const cJSON *p;
        cJSON_ArrayForEach(p, params)
        {
            if (out->dpv1_param_count >= PB_MAX_DPV1_PARAMS) {
                cfg_log_error("Slave '%s' has more than %d DPV1 parameters", out->name,
                              PB_MAX_DPV1_PARAMS);
                return -1;
            }
            if (parse_dpv1_param(p, &out->dpv1_params[out->dpv1_param_count]) != 0)
                return -1;
            out->dpv1_param_count++;
        }
    }

    return 0;
}

static int parse_slaves_section(const cJSON *slaves_arr, pb_config_t *config)
{
    if (!slaves_arr || !cJSON_IsArray(slaves_arr))
        return 0;

    const cJSON *slave_obj;
    cJSON_ArrayForEach(slave_obj, slaves_arr)
    {
        if (config->slave_count >= PB_MAX_SLAVES) {
            cfg_log_error("Configuration has more than %d slaves", PB_MAX_SLAVES);
            return -1;
        }
        if (parse_slave(slave_obj, &config->slaves[config->slave_count]) != 0)
            return -1;
        config->slave_count++;
    }
    return 0;
}

/*
 * =============================================================================
 * Defaults
 * =============================================================================
 */

void pb_config_init_defaults(pb_config_t *config)
{
    memset(config, 0, sizeof(*config));

    safe_strcpy(config->name, sizeof(config->name), "master");

    safe_strcpy(config->master.device, sizeof(config->master.device), "/dev/ttyUSB0");
    config->master.baudrate = 500000;
    config->master.parity   = PB_PARITY_EVEN;
    config->master.stop_bits = 1;

    config->master.station_address = 2;
    config->master.highest_station_address = 15;

    config->master.rs485_rts_control = false;
    config->master.rts_delay_us = 0;

    config->master.slot_time_us = 1000;
    config->master.gap_update_factor = 10;
    config->master.max_retry_limit = 3;

    config->master.cycle_time_us = 10000;
    config->master.task_priority = 80;
    safe_strcpy(config->master.log_level, sizeof(config->master.log_level), "info");

    config->diagnostics.log_connections = true;
    config->diagnostics.log_errors = true;
    config->diagnostics.max_log_entries = 100;
    config->diagnostics.status_update_interval_ms = 1000;
}

/*
 * =============================================================================
 * Top-level parse
 * =============================================================================
 */

static int parse_one_config(const cJSON *config_obj, pb_config_t *config)
{
    pb_config_init_defaults(config);

    const cJSON *master_obj = cJSON_GetObjectItemCaseSensitive(config_obj, "master");
    if (master_obj)
        parse_master_section(master_obj, &config->master);

    const cJSON *diag_obj = cJSON_GetObjectItemCaseSensitive(config_obj, "diagnostics");
    if (diag_obj)
        parse_diagnostics_section(diag_obj, &config->diagnostics);

    const cJSON *slaves_arr = cJSON_GetObjectItemCaseSensitive(config_obj, "slaves");
    if (parse_slaves_section(slaves_arr, config) != 0)
        return PB_CONFIG_ERR_INVALID;

    return PB_CONFIG_OK;
}

int pb_config_parse_all(const char *config_path, pb_config_t *configs, int max_configs,
                         int *out_count)
{
    *out_count = 0;

    FILE *fp = fopen(config_path, "rb");
    if (!fp) {
        cfg_log_error("Failed to open config file: %s", config_path);
        return PB_CONFIG_ERR_FILE;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (size <= 0) {
        fclose(fp);
        cfg_log_error("Config file is empty: %s", config_path);
        return PB_CONFIG_ERR_FILE;
    }

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) {
        fclose(fp);
        cfg_log_error("Out of memory reading config file");
        return PB_CONFIG_ERR_MEMORY;
    }

    size_t read = fread(buf, 1, (size_t)size, fp);
    fclose(fp);
    buf[read] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    if (!root) {
        cfg_log_error("Failed to parse JSON config: %s", config_path);
        return PB_CONFIG_ERR_PARSE;
    }
    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        cfg_log_error("Config root is not an array: %s", config_path);
        return PB_CONFIG_ERR_PARSE;
    }

    int count = 0;
    const cJSON *entry;
    cJSON_ArrayForEach(entry, root)
    {
        const char *protocol = get_string(entry, "protocol", "");
        if (strcasecmp_local(protocol, "PROFIBUS_DP") != 0 &&
            strcasecmp_local(protocol, "PROFIBUS") != 0)
            continue;

        if (count >= max_configs) {
            cfg_log_warn("More than %d PROFIBUS_DP configs found; ignoring extras", max_configs);
            break;
        }

        const cJSON *config_obj = cJSON_GetObjectItemCaseSensitive(entry, "config");
        if (!config_obj) {
            cfg_log_error("PROFIBUS_DP entry %d missing 'config' object", count);
            cJSON_Delete(root);
            return PB_CONFIG_ERR_MISSING;
        }

        int rc = parse_one_config(config_obj, &configs[count]);
        if (rc != PB_CONFIG_OK) {
            cJSON_Delete(root);
            return rc;
        }

        safe_strcpy(configs[count].name, sizeof(configs[count].name),
                    get_string(entry, "name", "master"));

        count++;
    }

    cJSON_Delete(root);
    *out_count = count;

    if (count == 0) {
        cfg_log_error("No PROFIBUS_DP configuration found in %s", config_path);
        return PB_CONFIG_ERR_MISSING;
    }

    return PB_CONFIG_OK;
}

int pb_config_validate(const pb_config_t *config)
{
    if (config->master.device[0] == '\0') {
        cfg_log_error("Master 'device' (serial port) is required");
        return PB_CONFIG_ERR_INVALID;
    }
    if (config->master.baudrate <= 0) {
        cfg_log_error("Master 'baudrate' must be positive");
        return PB_CONFIG_ERR_INVALID;
    }
    if (config->master.station_address < 0 || config->master.station_address > 125) {
        cfg_log_error("Master 'station_address' must be 0-125");
        return PB_CONFIG_ERR_INVALID;
    }
    if (config->master.cycle_time_us <= 0) {
        cfg_log_error("Master 'cycle_time_us' must be positive");
        return PB_CONFIG_ERR_INVALID;
    }
    if (config->master.task_priority < 1 || config->master.task_priority > 99) {
        cfg_log_error("Master 'task_priority' must be 1-99");
        return PB_CONFIG_ERR_INVALID;
    }
    if (config->slave_count == 0) {
        cfg_log_error("Configuration has no slaves");
        return PB_CONFIG_ERR_INVALID;
    }

    /* Station addresses must be unique and not equal to the master's. */
    for (int i = 0; i < config->slave_count; i++) {
        if (config->slaves[i].station_address == config->master.station_address) {
            cfg_log_error("Slave '%s' station_address %d collides with master",
                          config->slaves[i].name, config->slaves[i].station_address);
            return PB_CONFIG_ERR_INVALID;
        }
        for (int j = i + 1; j < config->slave_count; j++) {
            if (config->slaves[i].station_address == config->slaves[j].station_address) {
                cfg_log_error("Duplicate slave station_address %d ('%s' and '%s')",
                              config->slaves[i].station_address, config->slaves[i].name,
                              config->slaves[j].name);
                return PB_CONFIG_ERR_INVALID;
            }
        }
    }

    return PB_CONFIG_OK;
}
