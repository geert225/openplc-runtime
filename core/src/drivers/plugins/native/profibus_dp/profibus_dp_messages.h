/**
 * @file profibus_dp_messages.h
 * @brief DP (DPV0) and DPV1 acyclic/cyclic message builders and parsers
 *
 * Builds and parses the higher-level DP telegram payloads on top of the FDL
 * framing layer (profibus_fdl.h): Slave_Diag, Set_Prm, Chk_Cfg, cyclic
 * Data_Exchange, Global_Control, and DPV1 MSAC1 read/write services.
 *
 * All "build" functions here produce a complete, ready-to-send FDL telegram
 * (including SD1/SD2 framing, FCS and ED) -- callers only need to hand the
 * result to pb_fdl_transaction(). All "parse" functions take an already
 * FDL-decoded telegram (pb_fdl_telegram_t, from pb_fdl_parse() /
 * pb_fdl_transaction()).
 */

#ifndef PROFIBUS_DP_MESSAGES_H
#define PROFIBUS_DP_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#include "profibus_config.h"
#include "profibus_fdl.h"

/* Maximum bytes of Ext_Diag_Data captured from a Slave_Diag response. */
#define PB_MAX_EXT_DIAG_LEN 64

/*
 * =============================================================================
 * Station_Status_1 / _2 / _3 bit definitions (Slave_Diag response)
 * =============================================================================
 */
#define PB_STAT1_STATION_NOT_EXIST     0x01
#define PB_STAT1_STATION_NOT_READY     0x02
#define PB_STAT1_CFG_FAULT             0x04
#define PB_STAT1_EXT_DIAG              0x08
#define PB_STAT1_NOT_SUPPORTED         0x10
#define PB_STAT1_INVALID_SLAVE_RESPONSE 0x20
#define PB_STAT1_PRM_FAULT             0x40
#define PB_STAT1_MASTER_LOCK           0x80

#define PB_STAT2_PRM_REQ      0x01
#define PB_STAT2_STAT_DIAG    0x02
#define PB_STAT2_WD_ON        0x08
#define PB_STAT2_FREEZE_MODE  0x10
#define PB_STAT2_SYNC_MODE    0x20
#define PB_STAT2_DEACTIVATED  0x80

#define PB_STAT3_EXT_DIAG_OVERFLOW 0x80

/*
 * =============================================================================
 * Set_Prm Station_Status request bits
 * =============================================================================
 */
#define PB_PRM_STATUS_LOCK_REQ   0x80
#define PB_PRM_STATUS_UNLOCK_REQ 0x40
#define PB_PRM_STATUS_SYNC_REQ   0x20
#define PB_PRM_STATUS_FREEZE_REQ 0x10
#define PB_PRM_STATUS_WD_ON      0x08

/*
 * =============================================================================
 * Global_Control command bits (broadcast to SAP 58)
 * =============================================================================
 */
#define PB_GC_CLEAR_DATA   0x01
#define PB_GC_UNFREEZE_REQ 0x02
#define PB_GC_FREEZE_REQ   0x04
#define PB_GC_UNSYNC_REQ   0x08
#define PB_GC_SYNC_REQ     0x10

/** Broadcast FDL address. */
#define PB_FDL_BROADCAST_ADDR 127

/**
 * @brief Decoded Slave_Diag response
 */
typedef struct {
    uint8_t  stat_1;
    uint8_t  stat_2;
    uint8_t  stat_3;
    uint8_t  master_add;
    uint16_t ident_number;
    uint8_t  ext_diag_data[PB_MAX_EXT_DIAG_LEN];
    int      ext_diag_len;
} pb_diag_t;

/**
 * @brief Build the FC byte for a master request telegram.
 *
 * @param function_code One of the PB_FC_* request function codes (bits 4-0).
 * @param fcb Frame Count Bit -- toggled for each new (non-retry) request to
 *            a given station.
 * @param fcv FCB valid -- false on the very first request to a station after
 *            it comes online.
 */
uint8_t pb_msg_request_fc(uint8_t function_code, bool fcb, bool fcv);

/**
 * @brief Build a Slave_Diag request (SAP 60).
 * @return Encoded telegram length, or -1 on overflow.
 */
int pb_msg_build_slave_diag_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                     uint8_t master_addr, bool fcb, bool fcv);

/**
 * @brief Parse a Slave_Diag response telegram.
 * @return 0 on success, -1 if the telegram is too short to be a valid
 *         Slave_Diag response.
 */
int pb_msg_parse_slave_diag_response(const pb_fdl_telegram_t *rx, pb_diag_t *out);

/**
 * @brief Compute the WD_Fact_1 / WD_Fact_2 pair such that
 *        WD_Fact_1 * WD_Fact_2 * 10ms >= @p watchdog_ms.
 */
void pb_msg_compute_watchdog_factors(int watchdog_ms, uint8_t *fact1, uint8_t *fact2);

/**
 * @brief Build a Set_Prm request (SAP 61) from the slave's configuration.
 *
 * Encodes Station_Status (WD_On per slave->watchdog_enabled), watchdog
 * factors derived from slave->watchdog_ms, Min_TSDR, Ident_Number,
 * Group_Ident (from slave->group, 1-8 -> bit 0-7), followed verbatim by
 * slave->user_prm_data.
 *
 * @return Encoded telegram length, or -1 on overflow.
 */
int pb_msg_build_set_prm_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                  uint8_t master_addr, bool fcb, bool fcv,
                                  const pb_slave_t *slave);

/**
 * @brief Build a Chk_Cfg request (SAP 62) from the slave's configuration.
 *
 * The data union is slave->cfg_data verbatim (module configuration bytes
 * as found in the slave's GSD file).
 *
 * @return Encoded telegram length, or -1 on overflow / empty cfg_data.
 */
int pb_msg_build_chk_cfg_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                  uint8_t master_addr, bool fcb, bool fcv,
                                  const pb_slave_t *slave);

/**
 * @brief Check whether a Set_Prm/Chk_Cfg response indicates success.
 *
 * Success is a short acknowledge (SC) or an SD1 response with a positive
 * function code (RES_OK/RES_DL/RES_DH).
 */
bool pb_msg_response_is_ok(const pb_fdl_telegram_t *rx);

/**
 * @brief Build a cyclic Data_Exchange request (no SAP addressing).
 *
 * If @p output_len is 0, an SD1 (no data) telegram is built; otherwise an
 * SD2 telegram carrying @p output_data is built.
 *
 * @return Encoded telegram length, or -1 on overflow.
 */
int pb_msg_build_data_exchange_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                        uint8_t master_addr, bool fcb, bool fcv,
                                        const uint8_t *output_data, int output_len);

/**
 * @brief Extract the input data from a Data_Exchange response.
 *
 * Copies up to @p input_max bytes from the response data union into
 * @p input_data and reports the actual length in @p out_len. A short
 * acknowledge (SC) response yields *out_len == 0 (slave has no input data).
 *
 * @return 0 on success, -1 if the response telegram type is unexpected.
 */
int pb_msg_parse_data_exchange_response(const pb_fdl_telegram_t *rx, uint8_t *input_data,
                                         int input_max, int *out_len);

/**
 * @brief Build a Global_Control broadcast (Clear_Data / Sync / Freeze etc.)
 *
 * Sent with SDN_HI (no acknowledge expected from slaves).
 *
 * @return Encoded telegram length, or -1 on overflow.
 */
int pb_msg_build_global_control(uint8_t *out, int out_max, uint8_t master_addr,
                                 uint8_t control_command, uint8_t group_mask);

/*
 * =============================================================================
 * DPV1 MSAC1 acyclic read/write
 * =============================================================================
 *
 * Request payload (after DSAP=50/SSAP=51 SAP addressing):
 *   [0] function code (PB_DPV1_FC_*)
 *   [1] slot number
 *   [2] index
 *   [3] length
 *   [4..] data (write requests only)
 *
 * Response payload (after DSAP=51/SSAP=50 SAP addressing):
 *   [0] function code (PB_DPV1_FC_*_RES, or PB_DPV1_FC_ERROR on failure)
 *   [1] slot number (echoed)
 *   [2] index (echoed)
 *   [3] length
 *   [4..] data (read responses only)
 */
#define PB_DPV1_FC_READ_REQ   0x5C
#define PB_DPV1_FC_READ_RES   0x5D
#define PB_DPV1_FC_WRITE_REQ  0x5E
#define PB_DPV1_FC_WRITE_RES  0x5F
#define PB_DPV1_FC_ERROR      0x60

/**
 * @brief Build a DPV1 MSAC1 Read request for the given slot/index.
 * @return Encoded telegram length, or -1 on overflow.
 */
int pb_msg_build_dpv1_read_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                    uint8_t master_addr, bool fcb, bool fcv, uint8_t slot,
                                    uint8_t index, uint8_t length);

/**
 * @brief Parse a DPV1 MSAC1 Read response.
 *
 * @param data     Output buffer for the returned value bytes.
 * @param data_max Capacity of @p data.
 * @param out_len  Number of value bytes copied into @p data.
 * @return 0 on success, -1 if the response indicates an error or is malformed.
 */
int pb_msg_parse_dpv1_read_response(const pb_fdl_telegram_t *rx, uint8_t *data, int data_max,
                                     int *out_len);

/**
 * @brief Build a DPV1 MSAC1 Write request for the given slot/index.
 * @return Encoded telegram length, or -1 on overflow / invalid length.
 */
int pb_msg_build_dpv1_write_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                     uint8_t master_addr, bool fcb, bool fcv, uint8_t slot,
                                     uint8_t index, const uint8_t *data, int data_len);

/**
 * @brief Parse a DPV1 MSAC1 Write response.
 * @return 0 if the write was acknowledged, -1 if the response indicates an
 *         error or is malformed.
 */
int pb_msg_parse_dpv1_write_response(const pb_fdl_telegram_t *rx);

#endif /* PROFIBUS_DP_MESSAGES_H */
