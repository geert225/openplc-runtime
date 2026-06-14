/**
 * @file profinet_rt.c
 * @brief PROFINET RT (Real-Time) cyclic data frame encode/decode
 */

#include "profinet_rt.h"

#include <string.h>

int pn_rt_send(pn_eth_handle_t *eth, const uint8_t dst_mac[PN_ETH_ALEN], uint16_t frame_id,
               const uint8_t *data, int data_len, uint16_t cycle_counter, uint8_t data_status)
{
    uint8_t buf[PN_ETH_MAX_PAYLOAD];
    int pos = 0;

    if (2 + data_len + PN_RT_TRAILER_LEN > (int)sizeof(buf))
        return -1;

    buf[pos++] = (uint8_t)(frame_id >> 8);
    buf[pos++] = (uint8_t)(frame_id & 0xFF);

    if (data_len > 0)
        memcpy(buf + pos, data, (size_t)data_len);
    pos += data_len;

    int min_before_trailer = PN_RT_MIN_FRAME_PAYLOAD - PN_RT_TRAILER_LEN;
    while (pos < min_before_trailer)
        buf[pos++] = 0x00;

    buf[pos++] = (uint8_t)(cycle_counter >> 8);
    buf[pos++] = (uint8_t)(cycle_counter & 0xFF);
    buf[pos++] = data_status;
    buf[pos++] = 0x00; /* TransferStatus = OK */

    return pn_eth_send(eth, dst_mac, buf, (size_t)pos);
}

int pn_rt_decode(const uint8_t *payload, int payload_len, int expected_data_len,
                 pn_rt_frame_t *out)
{
    if (payload_len < 2 + expected_data_len + PN_RT_TRAILER_LEN)
        return -1;

    out->frame_id = (uint16_t)((payload[0] << 8) | payload[1]);
    out->data     = payload + 2;
    out->data_len = expected_data_len;

    const uint8_t *trailer = payload + (payload_len - PN_RT_TRAILER_LEN);
    out->cycle_counter   = (uint16_t)((trailer[0] << 8) | trailer[1]);
    out->data_status     = trailer[2];
    out->transfer_status = trailer[3];

    return 0;
}
