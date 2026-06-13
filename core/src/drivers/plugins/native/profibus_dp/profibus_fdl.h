/**
 * @file profibus_fdl.h
 * @brief Profibus Fieldbus Data Link (FDL) layer -- telegram framing and
 *        single-master request/response transactions over a serial port.
 *
 * Implements the telegram formats from EN 50170 / IEC 61158 FDL:
 *
 *   SD1 (0x10): DA, SA, FC                          -- fixed, no data
 *   SD2 (0x68): LE, LEr, SD2, DA, SA, FC, DU[1..246] -- variable data
 *   SD3 (0xA2): DA, SA, FC, DU[8]                    -- fixed 8 data bytes
 *   SD4 (0xDC): DA, SA                               -- token (unused, single master)
 *   SC  (0xE5): single byte short acknowledge
 *
 * All multi-byte telegrams end with FCS (sum of all bytes from DA to the
 * end of DU, modulo 256) followed by ED (0x16).
 *
 * This master implementation operates as the sole active station on the
 * bus (no token passing to other masters): every bus cycle it issues
 * SRD_HIGH (Send and Request Data, high priority) requests to each
 * configured slave in turn and waits up to slot_time_us for the response.
 */

#ifndef PROFIBUS_FDL_H
#define PROFIBUS_FDL_H

#include <stdint.h>
#include <stdbool.h>

#include "profibus_config.h"
#include "profibus_serial.h"

/* Start/end delimiters */
#define PB_SD1 0x10
#define PB_SD2 0x68
#define PB_SD3 0xA2
#define PB_SD4 0xDC
#define PB_SC  0xE5
#define PB_ED  0x16

/*
 * FC (Frame Control) byte.
 *
 * Request frames (master -> slave): bit7 = 1
 *   bit7 = 1 (request)
 *   bit6 = FCB (Frame Count Bit) -- toggled for each new SRD/SDA/SDN
 *          request to a given station; left unchanged on a repeated
 *          (retry) request.
 *   bit5 = FCV (FCB valid) -- 0 on the very first request to a station
 *          after it goes online (so it resets its FCB tracking), 1 once
 *          established.
 *   bits 4-0 = function code.
 */
#define PB_FC_REQ 0x80
#define PB_FC_FCB 0x40
#define PB_FC_FCV 0x20

/* Request function codes (bits 4-0) */
#define PB_FC_FDL_STATUS 0x09 /* Request FDL Status (SAP-less, used for live-list probing) */
#define PB_FC_SDA_LO     0x03 /* Send Data with Acknowledge, low priority */
#define PB_FC_SDN_LO     0x04 /* Send Data with No Acknowledge, low priority */
#define PB_FC_SDA_HI     0x05 /* Send Data with Acknowledge, high priority */
#define PB_FC_SDN_HI     0x06 /* Send Data with No Acknowledge, high priority (Global_Control) */
#define PB_FC_SRD_LO     0x0C /* Send and Request Data with reply, low priority */
#define PB_FC_SRD_HI     0x0D /* Send and Request Data with reply, high priority */

/* Response function codes (bits 4-0, bit7 = 0) */
#define PB_FC_RES_OK  0x00 /* positive acknowledge, no data */
#define PB_FC_RES_UE  0x01 /* user error */
#define PB_FC_RES_RR  0x02 /* no resources */
#define PB_FC_RES_RS  0x03 /* service / SAP not active */
#define PB_FC_RES_DL  0x08 /* data low -- response with data, no further data pending */
#define PB_FC_RES_NR  0x09 /* no response data available */
#define PB_FC_RES_DH  0x0A /* data high -- response with data, diagnostic data pending */
#define PB_FC_RES_RDL 0x0C /* response data low, further data follows */
#define PB_FC_RES_RDH 0x0D /* response data high, further data follows */

/*
 * Well-known DP SAP numbers for acyclic master->slave services (DSAP
 * addressing the service on the slave, SSAP identifying the master).
 * These follow the standard PROFIBUS DP SAP assignment table.
 */
#define PB_SAP_SET_SLAVE_ADD  55
#define PB_SAP_RD_INP         56
#define PB_SAP_RD_OUTP        57
#define PB_SAP_GLOBAL_CONTROL 58
#define PB_SAP_GET_CFG        59
#define PB_SAP_SLAVE_DIAG     60
#define PB_SAP_SET_PRM        61
#define PB_SAP_CHK_CFG        62

/** Master's own SAP used as SSAP for the acyclic services above. */
#define PB_MASTER_SAP 62

/** DPV1 MSAC1 (Master-Slave Acyclic Communication, Class 1) SAPs. */
#define PB_SAP_DPV1_SLAVE_MSAC1  50 /* DSAP on the slave */
#define PB_SAP_DPV1_MASTER_MSAC1 51 /* SSAP on the master */

/** Maximum encoded telegram size: SD2,LE,LEr,SD2,DA,SA,FC + DU(<=246) + FCS,ED. */
#define PB_FDL_MAX_TELEGRAM (9 + PB_MAX_IO_DATA_LEN + 2)

/** Telegram kinds recognized by the parser. */
typedef enum {
    PB_TEL_NONE,
    PB_TEL_SD1,
    PB_TEL_SD2,
    PB_TEL_SD3,
    PB_TEL_SD4,
    PB_TEL_SC
} pb_telegram_type_t;

/** Decoded telegram. */
typedef struct {
    pb_telegram_type_t type;
    uint8_t da;
    uint8_t sa;
    uint8_t fc;
    uint8_t data[PB_MAX_IO_DATA_LEN + 8];
    int     data_len;
} pb_fdl_telegram_t;

/**
 * @brief Build an SD1 telegram (no data field).
 * @return Encoded length in bytes, or -1 if @p out_max is too small.
 */
int pb_fdl_build_sd1(uint8_t *out, int out_max, uint8_t da, uint8_t sa, uint8_t fc);

/**
 * @brief Build an SD2 telegram with @p data_len bytes of payload (1..246).
 * @return Encoded length in bytes, or -1 on overflow / invalid length.
 */
int pb_fdl_build_sd2(uint8_t *out, int out_max, uint8_t da, uint8_t sa, uint8_t fc,
                      const uint8_t *data, int data_len);

/**
 * @brief Build an SD2 telegram addressed to a slave SAP (acyclic services).
 *
 * Sets bit 7 of DA and SA to indicate SAP addressing and prepends
 * DU[0]=da_sap, DU[1]=sa_sap before @p data.
 */
int pb_fdl_build_sap_request(uint8_t *out, int out_max, uint8_t da_addr, uint8_t da_sap,
                              uint8_t sa_addr, uint8_t sa_sap, uint8_t fc,
                              const uint8_t *data, int data_len);

/**
 * @brief Build a cyclic Data_Exchange SD2 telegram (no SAP addressing).
 */
int pb_fdl_build_data_request(uint8_t *out, int out_max, uint8_t slave_addr,
                               uint8_t master_addr, uint8_t fc, const uint8_t *data,
                               int data_len);

/**
 * @brief Parse a received buffer into a telegram.
 *
 * @return Number of bytes consumed (== telegram length) on success,
 *         0 if the buffer does not yet contain a complete telegram,
 *         -1 if the buffer starts with an invalid/unrecognized delimiter
 *         or the FCS check fails.
 */
int pb_fdl_parse(const uint8_t *buf, int len, pb_fdl_telegram_t *out);

/**
 * @brief Perform one master-initiated request/response transaction.
 *
 * Sends the encoded request telegram in @p tx_buf/@p tx_len, then waits up
 * to @p slot_time_us for a complete, FCS-valid response telegram. Retries
 * up to @p max_retries times on timeout or FCS error (the same tx buffer is
 * re-sent verbatim -- callers must not toggle FCB between retries of the
 * same logical request).
 *
 * @return 0 on success (rx populated), -1 if no valid response was
 *         received after all retries.
 */
int pb_fdl_transaction(pb_serial_port_t *port, const uint8_t *tx_buf, int tx_len,
                        pb_fdl_telegram_t *rx, int slot_time_us, int max_retries);

#endif /* PROFIBUS_FDL_H */
