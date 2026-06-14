/**
 * @file profinet_rt.h
 * @brief PROFINET RT (Real-Time) cyclic data frame encode/decode
 *
 * RT_CLASS_1 cyclic frames are plain Ethernet frames (EtherType 0x8892,
 * see profinet_eth.h) whose payload is:
 *
 *   FrameID (2 bytes) | IO data (+ IOPS/IOCS status bytes) [+ padding] |
 *   CycleCounter (2 bytes) | DataStatus (1 byte) | TransferStatus (1 byte)
 *
 * The IO data length and layout for a given FrameID/CR are fixed at AR
 * establishment time (see profinet_rpc.c's IOCRBlockReq); this module only
 * handles the per-cycle framing.
 */

#ifndef PROFINET_RT_H
#define PROFINET_RT_H

#include <stdint.h>

#include "profinet_eth.h"

/** Trailer: CycleCounter(2) + DataStatus(1) + TransferStatus(1) */
#define PN_RT_TRAILER_LEN 4

/** Minimum Ethernet payload size (46 bytes -> 60-byte frame before FCS). */
#define PN_RT_MIN_FRAME_PAYLOAD 46

/* DataStatus bits (IEC 61158-6-10) */
#define PN_DATA_STATUS_STATE            0x01 /* 1 = RUN */
#define PN_DATA_STATUS_REDUNDANCY       0x02
#define PN_DATA_STATUS_DATA_VALID       0x04 /* 1 = valid */
#define PN_DATA_STATUS_PROVIDER_STATE   0x08 /* 1 = Run */
#define PN_DATA_STATUS_STATION_PROBLEM  0x20 /* 1 = ok (no problem) */

/** "Everything is fine" DataStatus value sent by a healthy provider. */
#define PN_DATA_STATUS_GOOD                                                                       \
    (PN_DATA_STATUS_STATE | PN_DATA_STATUS_DATA_VALID | PN_DATA_STATUS_PROVIDER_STATE |           \
     PN_DATA_STATUS_STATION_PROBLEM)

/** Decoded view of a received RT cyclic frame. */
typedef struct {
    uint16_t       frame_id;
    const uint8_t *data; /* points into the buffer passed to pn_rt_decode() */
    int            data_len;
    uint16_t       cycle_counter;
    uint8_t        data_status;
    uint8_t        transfer_status;
} pn_rt_frame_t;

/**
 * @brief Send one RT cyclic frame.
 *
 * @param data_len Length of @p data; the frame is zero-padded as needed to
 *                  reach the minimum Ethernet payload size before the trailer.
 * @return Number of payload bytes sent on success, -1 on error.
 */
int pn_rt_send(pn_eth_handle_t *eth, const uint8_t dst_mac[PN_ETH_ALEN], uint16_t frame_id,
               const uint8_t *data, int data_len, uint16_t cycle_counter, uint8_t data_status);

/**
 * @brief Decode a received RT cyclic frame.
 *
 * @param expected_data_len Configured IO data length for this CR (from
 *                           IOCRBlockReq); used to locate @p out->data.
 *                           The trailer is always the last 4 bytes of
 *                           @p payload, regardless of any padding.
 * @return 0 on success, -1 if @p payload_len is too short for
 *         @p expected_data_len + the trailer.
 */
int pn_rt_decode(const uint8_t *payload, int payload_len, int expected_data_len,
                 pn_rt_frame_t *out);

#endif /* PROFINET_RT_H */
