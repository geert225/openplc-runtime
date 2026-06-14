/**
 * @file profinet_config.c
 * @brief PROFINET IO Controller Plugin Configuration Parser (cJSON-based)
 */

#include "profinet_config.h"
#include "cJSON.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

static plugin_logger_t *g_cfg_logger = NULL;

void pn_config_set_logger(plugin_logger_t *logger)
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
        fprintf(stderr, "[PROFINET_CONFIG] ERROR: %s\n", buf);
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
        fprintf(stderr, "[PROFINET_CONFIG] WARN: %s\n", buf);
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
 * @brief Parse a MAC address from either a JSON array of 6 bytes/strings, or
 *        a string of the form "AA:BB:CC:DD:EE:FF" / "AA-BB-CC-DD-EE-FF".
 * @return true if a complete 6-byte MAC was parsed into @p mac.
 */
static bool parse_mac_address(const cJSON *obj, const char *key, uint8_t mac[6])
{
    if (!obj)
        return false;
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!item)
        return false;

    if (cJSON_IsArray(item)) {
        int n = 0;
        const cJSON *elem;
        cJSON_ArrayForEach(elem, item)
        {
            if (n >= 6)
                return false;
            if (cJSON_IsNumber(elem))
                mac[n] = (uint8_t)((unsigned)elem->valuedouble & 0xFFu);
            else if (cJSON_IsString(elem) && elem->valuestring)
                mac[n] = (uint8_t)(strtoul(elem->valuestring, NULL, 0) & 0xFFu);
            else
                return false;
            n++;
        }
        return n == 6;
    }

    if (cJSON_IsString(item) && item->valuestring) {
        unsigned b[6];
        if (sscanf(item->valuestring, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4],
                   &b[5]) == 6 ||
            sscanf(item->valuestring, "%x-%x-%x-%x-%x-%x", &b[0], &b[1], &b[2], &b[3], &b[4],
                   &b[5]) == 6) {
            for (int i = 0; i < 6; i++)
                mac[i] = (uint8_t)b[i];
            return true;
        }
    }

    return false;
}

/*
 * =============================================================================
 * Data type parsing
 * =============================================================================
 */

pn_data_type_t pn_parse_data_type(const char *str)
{
    if (!str)
        return PN_DTYPE_UNKNOWN;

    if (strcasecmp_local(str, "BOOL") == 0)
        return PN_DTYPE_BOOL;
    if (strcasecmp_local(str, "INT8") == 0 || strcasecmp_local(str, "SINT") == 0)
        return PN_DTYPE_INT8;
    if (strcasecmp_local(str, "UINT8") == 0 || strcasecmp_local(str, "USINT") == 0 ||
        strcasecmp_local(str, "BYTE") == 0)
        return PN_DTYPE_UINT8;
    if (strcasecmp_local(str, "INT16") == 0 || strcasecmp_local(str, "INT") == 0)
        return PN_DTYPE_INT16;
    if (strcasecmp_local(str, "UINT16") == 0 || strcasecmp_local(str, "UINT") == 0 ||
        strcasecmp_local(str, "WORD") == 0)
        return PN_DTYPE_UINT16;
    if (strcasecmp_local(str, "INT32") == 0 || strcasecmp_local(str, "DINT") == 0)
        return PN_DTYPE_INT32;
    if (strcasecmp_local(str, "UINT32") == 0 || strcasecmp_local(str, "UDINT") == 0 ||
        strcasecmp_local(str, "DWORD") == 0)
        return PN_DTYPE_UINT32;
    if (strcasecmp_local(str, "INT64") == 0 || strcasecmp_local(str, "LINT") == 0)
        return PN_DTYPE_INT64;
    if (strcasecmp_local(str, "UINT64") == 0 || strcasecmp_local(str, "ULINT") == 0 ||
        strcasecmp_local(str, "LWORD") == 0)
        return PN_DTYPE_UINT64;
    if (strcasecmp_local(str, "REAL") == 0 || strcasecmp_local(str, "REAL32") == 0 ||
        strcasecmp_local(str, "FLOAT") == 0)
        return PN_DTYPE_REAL32;
    if (strcasecmp_local(str, "LREAL") == 0 || strcasecmp_local(str, "REAL64") == 0 ||
        strcasecmp_local(str, "DOUBLE") == 0)
        return PN_DTYPE_REAL64;
    if (strcasecmp_local(str, "PAD") == 0)
        return PN_DTYPE_PAD;

    return PN_DTYPE_UNKNOWN;
}

int pn_data_type_size(pn_data_type_t dt)
{
    switch (dt) {
    case PN_DTYPE_BOOL:
    case PN_DTYPE_INT8:
    case PN_DTYPE_UINT8:
    case PN_DTYPE_PAD:
        return 1;
    case PN_DTYPE_INT16:
    case PN_DTYPE_UINT16:
        return 2;
    case PN_DTYPE_INT32:
    case PN_DTYPE_UINT32:
    case PN_DTYPE_REAL32:
        return 4;
    case PN_DTYPE_INT64:
    case PN_DTYPE_UINT64:
    case PN_DTYPE_REAL64:
        return 8;
    default:
        return 0;
    }
}

const char *pn_data_type_to_string(pn_data_type_t dt)
{
    switch (dt) {
    case PN_DTYPE_BOOL:   return "BOOL";
    case PN_DTYPE_INT8:   return "INT8";
    case PN_DTYPE_UINT8:  return "UINT8";
    case PN_DTYPE_INT16:  return "INT16";
    case PN_DTYPE_UINT16: return "UINT16";
    case PN_DTYPE_INT32:  return "INT32";
    case PN_DTYPE_UINT32: return "UINT32";
    case PN_DTYPE_INT64:  return "INT64";
    case PN_DTYPE_UINT64: return "UINT64";
    case PN_DTYPE_REAL32: return "REAL32";
    case PN_DTYPE_REAL64: return "REAL64";
    case PN_DTYPE_PAD:    return "PAD";
    default:              return "UNKNOWN";
    }
}

const char *pn_state_to_string(pn_plugin_state_t state)
{
    switch (state) {
    case PN_STATE_IDLE:        return "IDLE";
    case PN_STATE_CONNECTING:  return "CONNECTING";
    case PN_STATE_OPERATIONAL: return "OPERATIONAL";
    case PN_STATE_RECOVERING:  return "RECOVERING";
    case PN_STATE_ERROR:       return "ERROR";
    case PN_STATE_STOPPED:     return "STOPPED";
    default:                   return "UNKNOWN";
    }
}

const char *pn_device_state_to_string(pn_device_state_t state)
{
    switch (state) {
    case PN_DEV_OFFLINE:         return "OFFLINE";
    case PN_DEV_DCP_IDENTIFY:    return "DCP_IDENTIFY";
    case PN_DEV_CONNECTING:      return "CONNECTING";
    case PN_DEV_PRM_END:         return "PRM_END";
    case PN_DEV_WAIT_APPL_READY: return "WAIT_APPL_READY";
    case PN_DEV_DATA_EXCHANGE:   return "DATA_EXCHANGE";
    case PN_DEV_FAULT:           return "FAULT";
    default:                     return "UNKNOWN";
    }
}

/*
 * =============================================================================
 * Section parsers
 * =============================================================================
 */

static void parse_master_section(const cJSON *master_obj, pn_master_config_t *out)
{
    safe_strcpy(out->interface, sizeof(out->interface),
                get_string(master_obj, "interface", out->interface));
    safe_strcpy(out->station_name, sizeof(out->station_name),
                get_string(master_obj, "station_name", out->station_name));

    out->send_clock_factor = get_int(master_obj, "send_clock_factor", out->send_clock_factor);
    out->reduction_ratio   = get_int(master_obj, "reduction_ratio", out->reduction_ratio);
    out->cycle_time_us     = get_int(master_obj, "cycle_time_us", out->cycle_time_us);
    out->task_priority     = get_int(master_obj, "task_priority", out->task_priority);

    safe_strcpy(out->log_level, sizeof(out->log_level),
                get_string(master_obj, "log_level", out->log_level));
}

static void parse_diagnostics_section(const cJSON *diag_obj, pn_diagnostics_config_t *out)
{
    out->log_connections = get_bool(diag_obj, "log_connections", out->log_connections);
    out->log_errors      = get_bool(diag_obj, "log_errors", out->log_errors);
    out->max_log_entries = get_int(diag_obj, "max_log_entries", out->max_log_entries);
    out->status_update_interval_ms =
        get_int(diag_obj, "status_update_interval_ms", out->status_update_interval_ms);
}

static int parse_channel(const cJSON *ch_obj, pn_channel_t *out, int index)
{
    memset(out, 0, sizeof(*out));
    out->index = get_int(ch_obj, "index", index);
    safe_strcpy(out->name, sizeof(out->name), get_string(ch_obj, "name", ""));
    safe_strcpy(out->type, sizeof(out->type), get_string(ch_obj, "type", ""));
    out->bit_length = (uint8_t)get_int(ch_obj, "bit_length", 1);
    safe_strcpy(out->iec_location, sizeof(out->iec_location),
                get_string(ch_obj, "iec_location", ""));
    out->byte_offset = get_int(ch_obj, "byte_offset", -1);
    out->bit_offset  = get_int(ch_obj, "bit_offset", -1);
    out->data_type   = pn_parse_data_type(get_string(ch_obj, "data_type", "BOOL"));

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

static int parse_submodule(const cJSON *sm_obj, pn_submodule_t *out)
{
    memset(out, 0, sizeof(*out));
    out->subslot_number = get_int(sm_obj, "subslot_number", -1);
    out->submodule_ident_number = get_hex_or_number(sm_obj, "submodule_ident_number", 0);
    out->input_length  = get_int(sm_obj, "input_length", 0);
    out->output_length = get_int(sm_obj, "output_length", 0);

    if (out->subslot_number < 0) {
        cfg_log_error("Submodule is missing 'subslot_number'");
        return -1;
    }
    return 0;
}

static int parse_slot(const cJSON *slot_obj, pn_slot_t *out)
{
    memset(out, 0, sizeof(*out));
    out->slot_number = get_int(slot_obj, "slot_number", -1);
    out->module_ident_number = get_hex_or_number(slot_obj, "module_ident_number", 0);

    if (out->slot_number < 0) {
        cfg_log_error("Slot is missing 'slot_number'");
        return -1;
    }

    const cJSON *subs = cJSON_GetObjectItemCaseSensitive(slot_obj, "submodules");
    if (subs && cJSON_IsArray(subs)) {
        const cJSON *sm;
        cJSON_ArrayForEach(sm, subs)
        {
            if (out->submodule_count >= PN_MAX_SUBSLOTS) {
                cfg_log_error("Slot %d has more than %d submodules", out->slot_number,
                              PN_MAX_SUBSLOTS);
                return -1;
            }
            if (parse_submodule(sm, &out->submodules[out->submodule_count]) != 0)
                return -1;
            out->submodule_count++;
        }
    }

    return 0;
}

static int parse_device(const cJSON *dev_obj, pn_device_t *out)
{
    memset(out, 0, sizeof(*out));

    safe_strcpy(out->name, sizeof(out->name), get_string(dev_obj, "name", ""));
    safe_strcpy(out->station_name, sizeof(out->station_name),
                get_string(dev_obj, "station_name", ""));

    out->mac_configured = parse_mac_address(dev_obj, "mac_address", out->mac);

    safe_strcpy(out->ip_address, sizeof(out->ip_address), get_string(dev_obj, "ip_address", ""));

    out->vendor_id = get_hex_or_number(dev_obj, "vendor_id", 0);
    out->device_id = get_hex_or_number(dev_obj, "device_id", 0);

    out->watchdog_factor = get_int(dev_obj, "watchdog_factor", 3);
    out->strict          = get_bool(dev_obj, "strict", true);

    if (out->name[0] == '\0') {
        cfg_log_error("Device is missing 'name'");
        return -1;
    }
    if (out->station_name[0] == '\0' && !out->mac_configured) {
        cfg_log_error("Device '%s': either 'station_name' or 'mac_address' is required",
                       out->name);
        return -1;
    }
    if (out->ip_address[0] == '\0') {
        cfg_log_error("Device '%s' is missing 'ip_address'", out->name);
        return -1;
    }

    /* Slots/submodules */
    const cJSON *slots = cJSON_GetObjectItemCaseSensitive(dev_obj, "slots");
    if (slots && cJSON_IsArray(slots)) {
        const cJSON *slot;
        cJSON_ArrayForEach(slot, slots)
        {
            if (out->slot_count >= PN_MAX_SLOTS) {
                cfg_log_error("Device '%s' has more than %d slots", out->name, PN_MAX_SLOTS);
                return -1;
            }
            if (parse_slot(slot, &out->slots[out->slot_count]) != 0)
                return -1;
            out->slot_count++;
        }
    }

    /* Total cyclic data lengths: explicit override, else sum of submodule lengths. */
    int sum_in = 0, sum_out = 0;
    for (int i = 0; i < out->slot_count; i++) {
        for (int j = 0; j < out->slots[i].submodule_count; j++) {
            sum_in  += out->slots[i].submodules[j].input_length;
            sum_out += out->slots[i].submodules[j].output_length;
        }
    }
    out->input_length  = get_int(dev_obj, "input_length", sum_in);
    out->output_length = get_int(dev_obj, "output_length", sum_out);

    if (out->input_length > PN_MAX_IO_DATA_LEN || out->output_length > PN_MAX_IO_DATA_LEN) {
        cfg_log_error("Device '%s' I/O length exceeds %d bytes", out->name, PN_MAX_IO_DATA_LEN);
        return -1;
    }

    /* Channels */
    const cJSON *channels = cJSON_GetObjectItemCaseSensitive(dev_obj, "channels");
    if (channels && cJSON_IsArray(channels)) {
        int idx = 0;
        const cJSON *ch;
        cJSON_ArrayForEach(ch, channels)
        {
            if (out->channel_count >= PN_MAX_CHANNELS) {
                cfg_log_error("Device '%s' has more than %d channels", out->name,
                              PN_MAX_CHANNELS);
                return -1;
            }
            if (parse_channel(ch, &out->channels[out->channel_count], idx) != 0)
                return -1;
            out->channel_count++;
            idx++;
        }
    }

    return 0;
}

static int parse_devices_section(const cJSON *devices_arr, pn_config_t *config)
{
    if (!devices_arr || !cJSON_IsArray(devices_arr))
        return 0;

    const cJSON *dev_obj;
    cJSON_ArrayForEach(dev_obj, devices_arr)
    {
        if (config->device_count >= PN_MAX_DEVICES) {
            cfg_log_error("Configuration has more than %d devices", PN_MAX_DEVICES);
            return -1;
        }
        if (parse_device(dev_obj, &config->devices[config->device_count]) != 0)
            return -1;
        config->device_count++;
    }
    return 0;
}

/*
 * =============================================================================
 * Defaults
 * =============================================================================
 */

void pn_config_init_defaults(pn_config_t *config)
{
    memset(config, 0, sizeof(*config));

    safe_strcpy(config->name, sizeof(config->name), "controller");

    safe_strcpy(config->master.interface, sizeof(config->master.interface), "eth0");
    safe_strcpy(config->master.station_name, sizeof(config->master.station_name),
                "openplc-controller");

    config->master.send_clock_factor = 32;
    config->master.reduction_ratio    = 1;
    config->master.cycle_time_us      = 0; /* derived from send_clock_factor/reduction_ratio */
    config->master.task_priority      = 80;
    safe_strcpy(config->master.log_level, sizeof(config->master.log_level), "info");

    config->diagnostics.log_connections = true;
    config->diagnostics.log_errors      = true;
    config->diagnostics.max_log_entries = 100;
    config->diagnostics.status_update_interval_ms = 1000;
}

/*
 * =============================================================================
 * Top-level parse
 * =============================================================================
 */

static int parse_one_config(const cJSON *config_obj, pn_config_t *config)
{
    pn_config_init_defaults(config);

    const cJSON *master_obj = cJSON_GetObjectItemCaseSensitive(config_obj, "master");
    if (master_obj)
        parse_master_section(master_obj, &config->master);

    const cJSON *diag_obj = cJSON_GetObjectItemCaseSensitive(config_obj, "diagnostics");
    if (diag_obj)
        parse_diagnostics_section(diag_obj, &config->diagnostics);

    const cJSON *devices_arr = cJSON_GetObjectItemCaseSensitive(config_obj, "devices");
    if (parse_devices_section(devices_arr, config) != 0)
        return PN_CONFIG_ERR_INVALID;

    /* Derive the RT cycle time from the PROFINET timing model
     * (31.25us * send_clock_factor * reduction_ratio) if not given directly. */
    if (config->master.cycle_time_us <= 0) {
        config->master.cycle_time_us =
            (config->master.send_clock_factor * config->master.reduction_ratio * 125) / 4;
    }

    return PN_CONFIG_OK;
}

int pn_config_parse_all(const char *config_path, pn_config_t *configs, int max_configs,
                         int *out_count)
{
    *out_count = 0;

    FILE *fp = fopen(config_path, "rb");
    if (!fp) {
        cfg_log_error("Failed to open config file: %s", config_path);
        return PN_CONFIG_ERR_FILE;
    }

    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (size <= 0) {
        fclose(fp);
        cfg_log_error("Config file is empty: %s", config_path);
        return PN_CONFIG_ERR_FILE;
    }

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) {
        fclose(fp);
        cfg_log_error("Out of memory reading config file");
        return PN_CONFIG_ERR_MEMORY;
    }

    size_t read = fread(buf, 1, (size_t)size, fp);
    fclose(fp);
    buf[read] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    if (!root) {
        cfg_log_error("Failed to parse JSON config: %s", config_path);
        return PN_CONFIG_ERR_PARSE;
    }
    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        cfg_log_error("Config root is not an array: %s", config_path);
        return PN_CONFIG_ERR_PARSE;
    }

    int count = 0;
    const cJSON *entry;
    cJSON_ArrayForEach(entry, root)
    {
        const char *protocol = get_string(entry, "protocol", "");
        if (strcasecmp_local(protocol, "PROFINET_IO") != 0 &&
            strcasecmp_local(protocol, "PROFINET") != 0)
            continue;

        if (count >= max_configs) {
            cfg_log_warn("More than %d PROFINET_IO configs found; ignoring extras", max_configs);
            break;
        }

        const cJSON *config_obj = cJSON_GetObjectItemCaseSensitive(entry, "config");
        if (!config_obj) {
            cfg_log_error("PROFINET_IO entry %d missing 'config' object", count);
            cJSON_Delete(root);
            return PN_CONFIG_ERR_MISSING;
        }

        int rc = parse_one_config(config_obj, &configs[count]);
        if (rc != PN_CONFIG_OK) {
            cJSON_Delete(root);
            return rc;
        }

        safe_strcpy(configs[count].name, sizeof(configs[count].name),
                    get_string(entry, "name", "controller"));

        count++;
    }

    cJSON_Delete(root);
    *out_count = count;

    if (count == 0) {
        cfg_log_error("No PROFINET_IO configuration found in %s", config_path);
        return PN_CONFIG_ERR_MISSING;
    }

    return PN_CONFIG_OK;
}

int pn_config_validate(const pn_config_t *config)
{
    if (config->master.interface[0] == '\0') {
        cfg_log_error("Master 'interface' is required");
        return PN_CONFIG_ERR_INVALID;
    }
    if (config->master.send_clock_factor <= 0) {
        cfg_log_error("Master 'send_clock_factor' must be positive");
        return PN_CONFIG_ERR_INVALID;
    }
    if (config->master.reduction_ratio <= 0) {
        cfg_log_error("Master 'reduction_ratio' must be positive");
        return PN_CONFIG_ERR_INVALID;
    }
    if (config->master.cycle_time_us <= 0) {
        cfg_log_error("Master 'cycle_time_us' must be positive");
        return PN_CONFIG_ERR_INVALID;
    }
    if (config->master.task_priority < 1 || config->master.task_priority > 99) {
        cfg_log_error("Master 'task_priority' must be 1-99");
        return PN_CONFIG_ERR_INVALID;
    }
    if (config->device_count == 0) {
        cfg_log_error("Configuration has no devices");
        return PN_CONFIG_ERR_INVALID;
    }

    /* Device names and IP addresses must be unique. */
    for (int i = 0; i < config->device_count; i++) {
        for (int j = i + 1; j < config->device_count; j++) {
            if (strcmp(config->devices[i].name, config->devices[j].name) == 0) {
                cfg_log_error("Duplicate device name '%s'", config->devices[i].name);
                return PN_CONFIG_ERR_INVALID;
            }
            if (strcmp(config->devices[i].ip_address, config->devices[j].ip_address) == 0) {
                cfg_log_error("Duplicate device ip_address '%s' ('%s' and '%s')",
                              config->devices[i].ip_address, config->devices[i].name,
                              config->devices[j].name);
                return PN_CONFIG_ERR_INVALID;
            }
        }
    }

    return PN_CONFIG_OK;
}
