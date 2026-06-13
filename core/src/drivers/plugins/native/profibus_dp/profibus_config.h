/**
 * @file profibus_config.h
 * @brief Profibus DP Plugin Configuration Structures and Parser Interface
 *
 * Defines the C structures that mirror the JSON configuration contract for
 * the Profibus DP master plugin, and the shared I/O bridging types
 * (iec_location_t, channel map, transfer list) used by profibus_io.*.
 *
 * Mirrors the structure of the EtherCAT plugin's ethercat_config.h, adapted
 * for a serial-line, master/slave token-less (single master) Profibus DP
 * bus: instead of an IOmap, each slave has its own flat input/output data
 * buffer exchanged via DPV0 Data_Exchange telegrams.
 */

#ifndef PROFIBUS_CONFIG_H
#define PROFIBUS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>

#include "plugin_logger.h"

/* Maximum sizes */
#define PB_MAX_MASTERS        4
#define PB_MAX_SLAVES        32
#define PB_MAX_CHANNELS      64
#define PB_MAX_DPV1_PARAMS   32
#define PB_MAX_NAME_LEN      64
#define PB_MAX_IEC_LOC_LEN   16
#define PB_MAX_DEVICE_LEN   128

/* Profibus DP user data is limited to 244 bytes per telegram (SD2, max
 * data length 246 minus 2 address/FC bytes already counted separately in
 * our framing -- see profibus_fdl.h). */
#define PB_MAX_IO_DATA_LEN  244
#define PB_MAX_PRM_DATA_LEN 244
#define PB_MAX_CFG_DATA_LEN 244

#define PB_MAX_MAP_ENTRIES  256

/* Error codes (mirrors ecat_config_* error code convention) */
#define PB_CONFIG_OK              0
#define PB_CONFIG_ERR_FILE       -1
#define PB_CONFIG_ERR_PARSE      -2
#define PB_CONFIG_ERR_MEMORY     -3
#define PB_CONFIG_ERR_INVALID    -4
#define PB_CONFIG_ERR_MISSING    -5

/**
 * @brief Recognized channel/parameter data types
 *
 * Same set as the EtherCAT plugin's ecat_data_type_t, used both for I/O
 * channel mapping and for DPV1 parameter read/write value encoding.
 */
typedef enum {
    PB_DTYPE_UNKNOWN,
    PB_DTYPE_BOOL,
    PB_DTYPE_INT8,
    PB_DTYPE_UINT8,
    PB_DTYPE_INT16,
    PB_DTYPE_UINT16,
    PB_DTYPE_INT32,
    PB_DTYPE_UINT32,
    PB_DTYPE_INT64,
    PB_DTYPE_UINT64,
    PB_DTYPE_REAL32,
    PB_DTYPE_REAL64,
    PB_DTYPE_PAD
} pb_data_type_t;

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
 * @brief Single I/O channel: maps a slave's cyclic data byte/bit to a PLC
 *        IEC located variable.
 */
typedef struct {
    int            index;
    char           name[PB_MAX_NAME_LEN];
    char           type[20];          /* "digital_input", "analog_output", ... (informational) */
    uint8_t        bit_length;
    char           iec_location[PB_MAX_IEC_LOC_LEN];
    int            byte_offset;       /* offset into the slave's input or output data buffer */
    int            bit_offset;        /* 0-7 for bit channels, -1 otherwise */
    pb_data_type_t data_type;
} pb_channel_t;

/**
 * @brief DPV1 acyclic parameter (read and/or write) definition
 *
 * Addressed via slot/index per IEC 61158-6 (DPV1) Read/Write services.
 * Written once during slave startup if `has_initial_value` is set
 * (analogous to EtherCAT SDO configuration entries); can also be
 * read/written at runtime via execute_command.
 */
typedef struct {
    char           name[PB_MAX_NAME_LEN];
    uint8_t        slot;
    uint8_t        index;
    uint8_t        length;            /* expected data length in bytes */
    pb_data_type_t data_type;
    bool           writable;
    bool           has_initial_value;
    double         initial_value;     /* written at startup if has_initial_value */
} pb_dpv1_param_t;

/**
 * @brief Per-slave configuration
 *
 * `user_prm_data` and `cfg_data` are raw byte arrays as found in the
 * slave's GSD file (Ext_User_Prm_Data and the module configuration bytes
 * respectively). Supplying these verbatim from the GSD sidesteps having to
 * re-derive vendor-specific parameterization semantics.
 */
typedef struct {
    int      station_address;          /* FDL address, 1-125 */
    char     name[PB_MAX_NAME_LEN];
    uint16_t ident_number;             /* slave ident number, e.g. 0x806A */
    uint8_t  group;                    /* Global_Control group mask bit (1-8 -> bit 0-7) */

    bool     watchdog_enabled;
    int      watchdog_ms;              /* control/data watchdog (default 400) */

    int      min_tsdr;                 /* minimum station delay of responder, in bit times */

    uint8_t  user_prm_data[PB_MAX_PRM_DATA_LEN];
    int      user_prm_data_len;

    uint8_t  cfg_data[PB_MAX_CFG_DATA_LEN];
    int      cfg_data_len;

    int      input_length;             /* bytes of cyclic input data expected from this slave */
    int      output_length;            /* bytes of cyclic output data sent to this slave */

    pb_channel_t channels[PB_MAX_CHANNELS];
    int          channel_count;

    pb_dpv1_param_t dpv1_params[PB_MAX_DPV1_PARAMS];
    int             dpv1_param_count;

    /* Abort master startup if this slave fails parameterization
     * (Set_Prm/Chk_Cfg rejected, or fails to reach DATA_EXCHANGE). When
     * false, the slave is retried in the background while the rest of
     * the bus continues operating. */
    bool     strict;
} pb_slave_t;

/**
 * @brief Serial port parity setting
 */
typedef enum {
    PB_PARITY_NONE,
    PB_PARITY_EVEN,
    PB_PARITY_ODD
} pb_parity_t;

/**
 * @brief Master (serial line + FDL) configuration
 */
#define PB_DEVICE_MAX PB_MAX_DEVICE_LEN

typedef struct {
    char        device[PB_DEVICE_MAX]; /* e.g. "/dev/ttyUSB0" */
    int         baudrate;              /* 9600 .. 12000000 (standard Profibus rates) */
    pb_parity_t parity;                /* Profibus DP normally uses 8E1 (even parity) */
    int         stop_bits;             /* 1 or 2 */

    int         station_address;       /* this master's FDL address, 0-125 */
    int         highest_station_address; /* HSA -- highest address considered for token ring */

    /* RS485 transceiver control: toggle RTS before/after transmit on
     * half-duplex converters that need explicit driver-enable. */
    bool        rs485_rts_control;
    int         rts_delay_us;          /* turnaround delay before/after TX */

    /* Bus timing, expressed directly in microseconds for simplicity
     * (rather than the standard's bit-time units) so they can be tuned
     * without recomputing from baudrate. */
    int         slot_time_us;          /* max time to wait for a response (Tslot) */
    int         gap_update_factor;     /* how many token cycles between GAP polls */
    int         max_retry_limit;       /* retries before marking a slave offline */

    int         cycle_time_us;         /* target bus polling cycle period */
    int         task_priority;         /* SCHED_FIFO priority for the bus thread, 1-99 */

    char        log_level[8];
} pb_master_config_t;

/**
 * @brief Diagnostics configuration
 */
typedef struct {
    bool log_connections;
    bool log_errors;
    int  max_log_entries;
    int  status_update_interval_ms;
} pb_diagnostics_config_t;

/**
 * @brief Top-level Profibus DP configuration for one master/bus
 */
typedef struct {
    char                     name[PB_MAX_NAME_LEN]; /* from the JSON entry's "name" field */
    pb_master_config_t      master;
    pb_slave_t               slaves[PB_MAX_SLAVES];
    int                      slave_count;
    pb_diagnostics_config_t diagnostics;
} pb_config_t;

/*
 * =============================================================================
 * Channel Map and Transfer List Types (shared with profibus_io.*)
 * =============================================================================
 */

/**
 * @brief Single entry in the channel map
 */
typedef struct {
    /* Slave I/O data side */
    int            data_byte_offset;  /* byte offset into the slave's input/output buffer */
    int            data_bit_offset;   /* bit offset within the byte (0-7), -1 if not a bit */
    uint8_t        bit_length;

    /* PLC side */
    iec_size_t     size;
    int            byte_index;
    int            bit_index;         /* -1 unless IEC_SIZE_BIT */
    pb_data_type_t data_type;
} pb_channel_map_entry_t;

/**
 * @brief Complete channel map for one slave -- separate arrays for inputs and outputs
 */
typedef struct {
    pb_channel_map_entry_t inputs[PB_MAX_MAP_ENTRIES];
    int                     input_count;
    pb_channel_map_entry_t outputs[PB_MAX_MAP_ENTRIES];
    int                     output_count;
} pb_channel_map_t;

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
} pb_transfer_entry_t;

/**
 * @brief Complete transfer list for one slave
 */
typedef struct {
    pb_transfer_entry_t inputs[PB_MAX_MAP_ENTRIES];
    int                  input_count;
    pb_transfer_entry_t outputs[PB_MAX_MAP_ENTRIES];
    int                  output_count;
} pb_transfer_list_t;

/*
 * =============================================================================
 * Plugin / Slave State Machines
 * =============================================================================
 */

/** Background monitor thread: diagnostics polling + recovery of offline slaves. */
#ifndef PB_ENABLE_MONITOR_THREAD
#define PB_ENABLE_MONITOR_THREAD 1
#endif

#define PB_MONITOR_INTERVAL_MS 500

/**
 * @brief Master (bus) plugin state machine
 *
 * STOPPED -> IDLE -> CONNECTING -> PARAMETERIZING -> OPERATIONAL
 * OPERATIONAL <-> RECOVERING
 * RECOVERING -> ERROR (serial port lost, etc.)
 * Any state -> STOPPED (via stop_loop)
 */
typedef enum {
    PB_STATE_IDLE,
    PB_STATE_CONNECTING,
    PB_STATE_PARAMETERIZING,
    PB_STATE_OPERATIONAL,
    PB_STATE_RECOVERING,
    PB_STATE_ERROR,
    PB_STATE_STOPPED
} pb_plugin_state_t;

/**
 * @brief Per-slave bus-level state machine
 *
 * Each slave is independently driven through this sequence by the bus
 * thread. DATA_EXCHANGE is the steady-state cyclic operating state.
 */
typedef enum {
    PB_SLAVE_OFFLINE,        /* not yet contacted / FDL status check pending */
    PB_SLAVE_WAIT_DIAG1,     /* requesting initial Slave_Diag */
    PB_SLAVE_SET_PRM,        /* sending Set_Prm */
    PB_SLAVE_CHK_CFG,        /* sending Chk_Cfg */
    PB_SLAVE_WAIT_DIAG2,     /* requesting Slave_Diag to confirm ready */
    PB_SLAVE_DATA_EXCHANGE,  /* cyclic Data_Exchange */
    PB_SLAVE_FAULT           /* parameterization failed / repeated errors */
} pb_slave_state_t;

/**
 * @brief Per-slave runtime status snapshot for execute_command queries
 */
typedef struct {
    int      station_address;
    char     name[PB_MAX_NAME_LEN];
    _Atomic(int) state;          /* pb_slave_state_t */
    _Atomic(uint32_t) diag_flags; /* last Slave_Diag status byte 1 (Stat_1) */
    _Atomic(uint64_t) cycle_count;
    _Atomic(uint64_t) error_count;
    _Atomic(uint64_t) retry_count;
} pb_slave_status_t;

/**
 * @brief Per-slave live runtime state (parameterization + cyclic data)
 */
typedef struct {
    const pb_slave_t *cfg;          /* points into pb_config_t.slaves[] */

    uint8_t  input_data[PB_MAX_IO_DATA_LEN];
    uint8_t  output_data[PB_MAX_IO_DATA_LEN];

    pb_channel_map_t   channel_map;
    pb_transfer_list_t transfer_list;

    _Atomic(int) state;              /* pb_slave_state_t */
    bool     fcb;                    /* Frame Count Bit toggled per request */
    bool     fcb_initialized;
    int      retries;

    /* DPV1: index of the next configured parameter that still needs its
     * initial value written (-1 once all are done). Processed one per
     * bus cycle so as not to starve cyclic Data_Exchange. */
    int      next_dpv1_write;

    pb_slave_status_t status;
} pb_slave_runtime_t;

/*
 * =============================================================================
 * Parser Interface
 * =============================================================================
 */

/**
 * @brief Parse all Profibus DP master configurations from a JSON file
 *
 * The JSON has an array root format: [{ "name", "protocol", "config" }].
 * Each entry with "protocol": "PROFIBUS_DP" is parsed into a separate
 * pb_config_t.
 *
 * @param config_path Path to the JSON configuration file
 * @param configs     Output array of configurations
 * @param max_configs Maximum number of configs to parse
 * @param out_count   Output: number of configs actually parsed
 * @return PB_CONFIG_OK on success, negative error code on failure
 */
int pb_config_parse_all(const char *config_path, pb_config_t *configs,
                         int max_configs, int *out_count);

/**
 * @brief Validate a parsed configuration
 */
int pb_config_validate(const pb_config_t *config);

/**
 * @brief Initialize configuration with default values
 */
void pb_config_init_defaults(pb_config_t *config);

/**
 * @brief Provide a logger for the config parser to use for diagnostic messages.
 */
void pb_config_set_logger(plugin_logger_t *logger);

/**
 * @brief Parse a data type string into the corresponding enum value
 *
 * Recognizes the same names/aliases as the EtherCAT plugin:
 *   "BOOL", "INT8"/"SINT", "UINT8"/"USINT", "INT16"/"INT",
 *   "UINT16"/"UINT", "INT32"/"DINT", "UINT32"/"UDINT",
 *   "INT64"/"LINT", "UINT64"/"ULINT",
 *   "REAL"/"REAL32"/"FLOAT", "LREAL"/"REAL64"/"DOUBLE", "PAD"
 */
pb_data_type_t pb_parse_data_type(const char *str);

/** Size in bytes for a given data type (0 for UNKNOWN/PAD, 1 for BOOL). */
int pb_data_type_size(pb_data_type_t dt);

/** Human-readable name for a data type. */
const char *pb_data_type_to_string(pb_data_type_t dt);

/** Human-readable name for a plugin state. */
const char *pb_state_to_string(pb_plugin_state_t state);

/** Human-readable name for a slave state. */
const char *pb_slave_state_to_string(pb_slave_state_t state);

#endif /* PROFIBUS_CONFIG_H */
