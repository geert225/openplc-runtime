/**
 * @file profinet_config.h
 * @brief PROFINET IO Controller Plugin Configuration Structures and Parser
 *        Interface
 *
 * Defines the C structures that mirror the JSON configuration contract for
 * the PROFINET IO controller plugin, and the shared I/O bridging types
 * (iec_location_t, channel map, transfer list) used by profinet_io.*.
 *
 * Mirrors the structure of the Profibus DP plugin's profibus_config.h: a
 * top-level config per controller ("master"), a list of IO devices, and
 * per-device cyclic-data channel mappings. Unlike Profibus DP (one flat
 * input/output buffer per slave addressed serially), PROFINET IO devices
 * are organized into slots/subslots (modules/submodules), each contributing
 * a span of the device's flat input_data[]/output_data[] cyclic buffer.
 */

#ifndef PROFINET_CONFIG_H
#define PROFINET_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>

#include "plugin_logger.h"

/* Maximum sizes */
#define PN_MAX_MASTERS              4
#define PN_MAX_DEVICES             32
#define PN_MAX_SLOTS               16
#define PN_MAX_SUBSLOTS             4
#define PN_MAX_CHANNELS            64
#define PN_MAX_NAME_LEN            64
#define PN_MAX_IEC_LOC_LEN         16
#define PN_MAX_IFACE_LEN           32
#define PN_MAX_STATION_NAME_LEN   240   /* IEC 61158-6-10 NameOfStation limit */
#define PN_MAX_IP_LEN              16

/* RT cyclic data payload limit per device. A standard (non-jumbo) Ethernet
 * frame carries up to 1500 bytes of payload; this leaves headroom for the
 * FrameID, Cycle Counter, DataStatus/TransferStatus and IOCS/IOPS bytes. */
#define PN_MAX_IO_DATA_LEN       1024

#define PN_MAX_MAP_ENTRIES        256

/* Error codes (mirrors profibus_dp's PB_CONFIG_* convention) */
#define PN_CONFIG_OK                0
#define PN_CONFIG_ERR_FILE         -1
#define PN_CONFIG_ERR_PARSE        -2
#define PN_CONFIG_ERR_MEMORY       -3
#define PN_CONFIG_ERR_INVALID      -4
#define PN_CONFIG_ERR_MISSING      -5

/**
 * @brief Recognized channel data types (same set as the EtherCAT/Profibus
 *        plugins' data_type_t).
 */
typedef enum {
    PN_DTYPE_UNKNOWN,
    PN_DTYPE_BOOL,
    PN_DTYPE_INT8,
    PN_DTYPE_UINT8,
    PN_DTYPE_INT16,
    PN_DTYPE_UINT16,
    PN_DTYPE_INT32,
    PN_DTYPE_UINT32,
    PN_DTYPE_INT64,
    PN_DTYPE_UINT64,
    PN_DTYPE_REAL32,
    PN_DTYPE_REAL64,
    PN_DTYPE_PAD
} pn_data_type_t;

/**
 * @brief IEC 61131-3 data size qualifiers (X/B/W/D/L)
 */
typedef enum {
    IEC_SIZE_BIT,
    IEC_SIZE_BYTE,
    IEC_SIZE_WORD,
    IEC_SIZE_DWORD,
    IEC_SIZE_LWORD
} iec_size_t;

/**
 * @brief IEC 61131-3 direction qualifiers (I/Q)
 */
typedef enum {
    IEC_DIR_INPUT,
    IEC_DIR_OUTPUT
} iec_dir_t;

/**
 * @brief Parsed IEC location -- result of parsing a string like "%IX0.3"
 */
typedef struct {
    iec_dir_t  direction;
    iec_size_t size;
    int        byte_index;
    int        bit_index;  /* -1 unless size == IEC_SIZE_BIT */
} iec_location_t;

/**
 * @brief Single I/O channel: maps a device's cyclic data byte/bit to a PLC
 *        IEC located variable.
 */
typedef struct {
    int            index;
    char           name[PN_MAX_NAME_LEN];
    char           type[20];          /* "digital_input", "analog_output", ... (informational) */
    uint8_t        bit_length;
    char           iec_location[PN_MAX_IEC_LOC_LEN];
    int            byte_offset;       /* offset into the device's input or output data buffer */
    int            bit_offset;        /* 0-7 for bit channels, -1 otherwise */
    pn_data_type_t data_type;
} pn_channel_t;

/**
 * @brief Submodule (subslot) configuration
 *
 * `input_length`/`output_length` are the cyclic data lengths (in bytes)
 * contributed by this submodule, as declared in the device's GSDML. These
 * are used both to build the ExpectedSubmoduleBlockReq during AR
 * establishment and to compute each submodule's offset within the device's
 * flat input_data[]/output_data[] buffer (IOCRBlockReq FrameOffset).
 */
typedef struct {
    int      subslot_number;
    uint32_t submodule_ident_number;
    int      input_length;
    int      output_length;
} pn_submodule_t;

/**
 * @brief Slot (module) configuration
 */
typedef struct {
    int            slot_number;
    uint32_t       module_ident_number;
    pn_submodule_t submodules[PN_MAX_SUBSLOTS];
    int            submodule_count;
} pn_slot_t;

/**
 * @brief Per-device (IO device) configuration
 *
 * `station_name` is the PROFINET device name (DCP NameOfStation), used to
 * resolve the device's MAC address via DCP Identify if `mac_configured` is
 * false. `ip_address` is informational/used for RPC (Connect.req etc. are
 * sent to this address over UDP port 34964) -- the device must already be
 * configured with this address (e.g. via an engineering tool or a prior DCP
 * Set).
 */
typedef struct {
    char     name[PN_MAX_NAME_LEN];
    char     station_name[PN_MAX_STATION_NAME_LEN];

    uint8_t  mac[6];
    bool     mac_configured;          /* if false, resolved via DCP Identify by station_name */

    char     ip_address[PN_MAX_IP_LEN]; /* used as the RPC (UDP) destination */

    uint32_t vendor_id;
    uint32_t device_id;

    pn_slot_t slots[PN_MAX_SLOTS];
    int       slot_count;

    pn_channel_t channels[PN_MAX_CHANNELS];
    int          channel_count;

    int input_length;                 /* total cyclic input data length (sum of submodules) */
    int output_length;                /* total cyclic output data length (sum of submodules) */

    /* AR watchdog: the AR is considered lost if no cyclic frame is received
     * for watchdog_factor * send_clock_factor * reduction_ratio * 31.25us. */
    int  watchdog_factor;

    /* Abort master startup if this device fails to reach DATA_EXCHANGE
     * during the bounded startup window. When false, the device is retried
     * in the background while the rest of the bus continues operating. */
    bool strict;
} pn_device_t;

/**
 * @brief Controller (master) configuration
 *
 * `send_clock_factor` and `reduction_ratio` follow the PROFINET IO timing
 * model: the base send clock is 31.25us * send_clock_factor (typically
 * send_clock_factor=32 -> 1ms base clock), and the actual cycle time is
 * base_clock * reduction_ratio. `cycle_time_us` is derived from these for
 * convenience (and used directly to pace the RT cyclic thread).
 */
typedef struct {
    char        interface[PN_MAX_IFACE_LEN]; /* network interface, e.g. "eth0" */
    char        station_name[PN_MAX_STATION_NAME_LEN]; /* controller's own station name */

    int         send_clock_factor;    /* typically 32 (1ms base clock) */
    int         reduction_ratio;      /* cycle_time = 31.25us * send_clock_factor * reduction_ratio */
    int         cycle_time_us;        /* RT cyclic thread period; derived if 0 */

    int         task_priority;        /* SCHED_FIFO priority for the RT thread, 1-99 */

    char        log_level[8];
} pn_master_config_t;

/**
 * @brief Diagnostics configuration
 */
typedef struct {
    bool log_connections;
    bool log_errors;
    int  max_log_entries;
    int  status_update_interval_ms;
} pn_diagnostics_config_t;

/**
 * @brief Top-level PROFINET IO configuration for one controller
 */
typedef struct {
    char                     name[PN_MAX_NAME_LEN]; /* from the JSON entry's "name" field */
    pn_master_config_t       master;
    pn_device_t               devices[PN_MAX_DEVICES];
    int                       device_count;
    pn_diagnostics_config_t  diagnostics;
} pn_config_t;

/*
 * =============================================================================
 * Channel Map and Transfer List Types (shared with profinet_io.*)
 * =============================================================================
 */

/**
 * @brief Single entry in the channel map
 */
typedef struct {
    /* Device I/O data side */
    int            data_byte_offset;  /* byte offset into the device's input/output buffer */
    int            data_bit_offset;   /* bit offset within the byte (0-7), -1 if not a bit */
    uint8_t        bit_length;

    /* PLC side */
    iec_size_t     size;
    int            byte_index;
    int            bit_index;         /* -1 unless IEC_SIZE_BIT */
    pn_data_type_t data_type;
} pn_channel_map_entry_t;

/**
 * @brief Complete channel map for one device -- separate arrays for inputs and outputs
 */
typedef struct {
    pn_channel_map_entry_t inputs[PN_MAX_MAP_ENTRIES];
    int                     input_count;
    pn_channel_map_entry_t outputs[PN_MAX_MAP_ENTRIES];
    int                     output_count;
} pn_channel_map_t;

/**
 * @brief Single pre-resolved transfer entry
 */
typedef struct {
    void    *plc_ptr;
    int      data_byte_offset;
    int      data_bit_offset;
    uint8_t  byte_count;
    bool     is_bit;
    /* Journal coordinates for INPUT channels (see plugin_types.h
     * journal_write_* functions). Unused for output channels. */
    int      journal_index;
    int      journal_bit;
} pn_transfer_entry_t;

/**
 * @brief Complete transfer list for one device
 */
typedef struct {
    pn_transfer_entry_t inputs[PN_MAX_MAP_ENTRIES];
    int                  input_count;
    pn_transfer_entry_t outputs[PN_MAX_MAP_ENTRIES];
    int                  output_count;
} pn_transfer_list_t;

/*
 * =============================================================================
 * Plugin / Device State Machines
 * =============================================================================
 */

/** Background monitor thread: diagnostics polling + recovery of offline devices. */
#ifndef PN_ENABLE_MONITOR_THREAD
#define PN_ENABLE_MONITOR_THREAD 1
#endif

#define PN_MONITOR_INTERVAL_MS 500

/**
 * @brief Controller (master) plugin state machine
 *
 * IDLE -> CONNECTING -> OPERATIONAL
 * OPERATIONAL <-> RECOVERING
 * RECOVERING -> ERROR (interface lost, etc.)
 * Any state -> STOPPED (via stop_loop)
 */
typedef enum {
    PN_STATE_IDLE,
    PN_STATE_CONNECTING,
    PN_STATE_OPERATIONAL,
    PN_STATE_RECOVERING,
    PN_STATE_ERROR,
    PN_STATE_STOPPED
} pn_plugin_state_t;

/**
 * @brief Per-device Application Relationship (AR) state machine
 *
 * Each device is independently driven through this sequence by the
 * supervisor thread. DATA_EXCHANGE is the steady-state cyclic operating
 * state, maintained by the RT thread.
 *
 *   OFFLINE -> DCP_IDENTIFY -> CONNECTING -> WAIT_CONNECT_RES -> PRM_END ->
 *   WAIT_APPL_READY -> DATA_EXCHANGE
 *
 * Any state -> OFFLINE on timeout/AR loss (Release.req is sent best-effort
 * before returning to OFFLINE so the device frees the AR promptly).
 */
typedef enum {
    PN_DEV_OFFLINE,          /* not connected; AR (re-)establishment pending */
    PN_DEV_DCP_IDENTIFY,     /* resolving MAC address via DCP Identify */
    PN_DEV_CONNECTING,       /* Connect.req sent, awaiting Connect.res */
    PN_DEV_PRM_END,          /* IODControlReq(PrmEnd) sent, awaiting IODControlRes */
    PN_DEV_WAIT_APPL_READY,  /* awaiting device's IODControlReq(ApplicationReady) */
    PN_DEV_DATA_EXCHANGE,    /* cyclic RT data exchange */
    PN_DEV_FAULT             /* AR establishment failed repeatedly (strict devices only) */
} pn_device_state_t;

/**
 * @brief Per-device runtime status snapshot for execute_command queries
 */
typedef struct {
    char     name[PN_MAX_NAME_LEN];
    char     station_name[PN_MAX_STATION_NAME_LEN];
    _Atomic(int) state;             /* pn_device_state_t */
    _Atomic(uint64_t) cycle_count;
    _Atomic(uint64_t) error_count;
    _Atomic(uint64_t) reconnect_count;
    _Atomic(int) input_iops_good;   /* 1 if the last received input frame had IOPS=GOOD */
} pn_device_status_t;

/*
 * =============================================================================
 * Parser Interface
 * =============================================================================
 */

/**
 * @brief Parse all PROFINET IO controller configurations from a JSON file
 *
 * The JSON has an array root format: [{ "name", "protocol", "config" }].
 * Each entry with "protocol": "PROFINET_IO" is parsed into a separate
 * pn_config_t.
 *
 * @param config_path Path to the JSON configuration file
 * @param configs     Output array of configurations
 * @param max_configs Maximum number of configs to parse
 * @param out_count   Output: number of configs actually parsed
 * @return PN_CONFIG_OK on success, negative error code on failure
 */
int pn_config_parse_all(const char *config_path, pn_config_t *configs,
                         int max_configs, int *out_count);

/**
 * @brief Validate a parsed configuration
 */
int pn_config_validate(const pn_config_t *config);

/**
 * @brief Initialize configuration with default values
 */
void pn_config_init_defaults(pn_config_t *config);

/**
 * @brief Provide a logger for the config parser to use for diagnostic messages.
 */
void pn_config_set_logger(plugin_logger_t *logger);

/**
 * @brief Parse a data type string into the corresponding enum value
 *
 * Recognizes the same names/aliases as the EtherCAT/Profibus plugins:
 *   "BOOL", "INT8"/"SINT", "UINT8"/"USINT", "INT16"/"INT",
 *   "UINT16"/"UINT", "INT32"/"DINT", "UINT32"/"UDINT",
 *   "INT64"/"LINT", "UINT64"/"ULINT",
 *   "REAL"/"REAL32"/"FLOAT", "LREAL"/"REAL64"/"DOUBLE", "PAD"
 */
pn_data_type_t pn_parse_data_type(const char *str);

/** Size in bytes for a given data type (0 for UNKNOWN/PAD, 1 for BOOL). */
int pn_data_type_size(pn_data_type_t dt);

/** Human-readable name for a data type. */
const char *pn_data_type_to_string(pn_data_type_t dt);

/** Human-readable name for a plugin state. */
const char *pn_state_to_string(pn_plugin_state_t state);

/** Human-readable name for a device AR state. */
const char *pn_device_state_to_string(pn_device_state_t state);

#endif /* PROFINET_CONFIG_H */
