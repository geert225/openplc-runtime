/**
 * @file profinet_dcp.c
 * @brief PROFINET DCP (Discovery and Configuration Protocol) -- Identify
 */

#include "profinet_dcp.h"

#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include <time.h>

/* DCP FrameIDs (EtherType 0x8892 payload starts with this 2-byte field) */
#define PN_DCP_FRAMEID_HELLO     0xFEFC
#define PN_DCP_FRAMEID_GETSET    0xFEFD /* also: unicast Identify Response */
#define PN_DCP_FRAMEID_IDENT_REQ 0xFEFE

/* DCP ServiceID */
#define PN_DCP_SERVICE_IDENTIFY 0x05

/* DCP ServiceType */
#define PN_DCP_SERVICE_TYPE_REQUEST           0x00
#define PN_DCP_SERVICE_TYPE_RESPONSE_SUCCESS  0x01

/* DCP Option / SubOption */
#define PN_DCP_OPT_IP                   0x01
#define PN_DCP_SUBOPT_IP_PARAM          0x02

#define PN_DCP_OPT_DEVICE_PROPERTIES    0x02
#define PN_DCP_SUBOPT_DEV_STATION_NAME  0x02
#define PN_DCP_SUBOPT_DEV_ID            0x03

#define PN_DCP_OPT_ALL  0xFF
#define PN_DCP_SUBOPT_ALL 0xFF

static plugin_logger_t *g_dcp_logger = NULL;
static _Atomic uint32_t g_dcp_xid = 1;

void pn_dcp_set_logger(plugin_logger_t *logger)
{
    g_dcp_logger = logger;
}

static uint32_t next_xid(void)
{
    return atomic_fetch_add(&g_dcp_xid, 1);
}

static inline uint16_t dcp_get_u16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline uint32_t dcp_get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static inline int dcp_put_u16(uint8_t *buf, int pos, uint16_t val)
{
    buf[pos]     = (uint8_t)(val >> 8);
    buf[pos + 1] = (uint8_t)(val & 0xFF);
    return pos + 2;
}

static inline int dcp_put_u32(uint8_t *buf, int pos, uint32_t val)
{
    buf[pos]     = (uint8_t)(val >> 24);
    buf[pos + 1] = (uint8_t)(val >> 16);
    buf[pos + 2] = (uint8_t)(val >> 8);
    buf[pos + 3] = (uint8_t)(val & 0xFF);
    return pos + 4;
}

/**
 * @brief Build a DCP Identify Request PDU.
 *
 * If @p name_filter is non-NULL/non-empty, the request carries a
 * NameOfStation filter block (Option 0x02 / SubOption 0x02) so only the
 * matching device responds. Otherwise it carries the "All Selector" block
 * (Option 0xFF / SubOption 0xFF) requesting a response from every device.
 *
 * @return Length of the encoded PDU.
 */
static int build_identify_request(uint8_t *buf, uint32_t xid, const char *name_filter)
{
    int pos = 0;
    pos = dcp_put_u16(buf, pos, PN_DCP_FRAMEID_IDENT_REQ);
    buf[pos++] = PN_DCP_SERVICE_IDENTIFY;
    buf[pos++] = PN_DCP_SERVICE_TYPE_REQUEST;
    pos = dcp_put_u32(buf, pos, xid);
    pos = dcp_put_u16(buf, pos, 1); /* ResponseDelay (factor of ~10ms; small fixed value) */

    int data_len_pos = pos;
    pos += 2; /* DCPDataLength, filled in below */

    int block_start = pos;
    if (name_filter && name_filter[0]) {
        size_t name_len = strlen(name_filter);
        buf[pos++] = PN_DCP_OPT_DEVICE_PROPERTIES;
        buf[pos++] = PN_DCP_SUBOPT_DEV_STATION_NAME;
        pos = dcp_put_u16(buf, pos, (uint16_t)name_len);
        memcpy(buf + pos, name_filter, name_len);
        pos += (int)name_len;
        if (name_len % 2)
            buf[pos++] = 0x00; /* pad to even block length */
    } else {
        buf[pos++] = PN_DCP_OPT_ALL;
        buf[pos++] = PN_DCP_SUBOPT_ALL;
        pos = dcp_put_u16(buf, pos, 0);
    }

    int data_len = pos - block_start;
    dcp_put_u16(buf, data_len_pos, (uint16_t)data_len);

    return pos;
}

/**
 * @brief Parse the blocks of a DCP Identify Response into @p out.
 *
 * @p out->mac must already be set by the caller (from the frame's source
 * MAC); this function fills in station_name, ip_address, vendor_id and
 * device_id from whichever blocks are present.
 */
static void parse_identify_response(const uint8_t *payload, int len, pn_dcp_device_info_t *out)
{
    if (len < 10)
        return;

    uint16_t dcp_data_len = dcp_get_u16(payload + 6);
    int pos = 10;
    int end = 10 + (int)dcp_data_len;
    if (end > len)
        end = len;

    while (pos + 4 <= end) {
        uint8_t option    = payload[pos];
        uint8_t suboption = payload[pos + 1];
        uint16_t block_len = dcp_get_u16(payload + pos + 2);
        int data_start = pos + 4;

        if (data_start + (int)block_len > end)
            break;

        if (block_len >= 2) {
            const uint8_t *data = payload + data_start + 2; /* skip 2-byte BlockInfo */
            int data_len = block_len - 2;

            if (option == PN_DCP_OPT_DEVICE_PROPERTIES &&
                suboption == PN_DCP_SUBOPT_DEV_STATION_NAME) {
                int n = data_len < (int)sizeof(out->station_name) - 1
                            ? data_len
                            : (int)sizeof(out->station_name) - 1;
                memcpy(out->station_name, data, n);
                out->station_name[n] = '\0';
            } else if (option == PN_DCP_OPT_DEVICE_PROPERTIES && suboption == PN_DCP_SUBOPT_DEV_ID &&
                       data_len >= 4) {
                out->vendor_id = dcp_get_u16(data);
                out->device_id = dcp_get_u16(data + 2);
            } else if (option == PN_DCP_OPT_IP && suboption == PN_DCP_SUBOPT_IP_PARAM &&
                       data_len >= 4) {
                snprintf(out->ip_address, sizeof(out->ip_address), "%u.%u.%u.%u", data[0], data[1],
                         data[2], data[3]);
            }
        }

        pos = data_start + (int)block_len;
        if (block_len % 2)
            pos++; /* skip pad byte */
    }
}

static long elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

int pn_dcp_identify_by_name(pn_eth_handle_t *eth, const char *station_name,
                             pn_dcp_device_info_t *out, int timeout_ms)
{
    uint8_t req[64];
    uint32_t xid = next_xid();
    int req_len = build_identify_request(req, xid, station_name);

    if (pn_eth_send(eth, PN_DCP_MULTICAST_MAC, req, (size_t)req_len) < 0)
        return -1;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    uint8_t resp[PN_ETH_MAX_PAYLOAD];
    uint8_t src_mac[PN_ETH_ALEN];

    for (;;) {
        long remaining = timeout_ms - elapsed_ms_since(&start);
        if (remaining <= 0)
            return 0;

        int n = pn_eth_recv(eth, src_mac, resp, sizeof(resp), (int)remaining);
        if (n < 0)
            return -1;
        if (n == 0)
            return 0;
        if (n < 10)
            continue;

        if (dcp_get_u16(resp) != PN_DCP_FRAMEID_GETSET)
            continue;
        if (resp[2] != PN_DCP_SERVICE_IDENTIFY || resp[3] != PN_DCP_SERVICE_TYPE_RESPONSE_SUCCESS)
            continue;
        if (dcp_get_u32(resp + 4) != xid)
            continue;

        memset(out, 0, sizeof(*out));
        memcpy(out->mac, src_mac, PN_ETH_ALEN);
        parse_identify_response(resp, n, out);

        if (station_name && station_name[0] && strcmp(out->station_name, station_name) != 0)
            continue;

        return 1;
    }
}

int pn_dcp_identify_all(pn_eth_handle_t *eth, pn_dcp_device_info_t *out, int max_devices,
                         int timeout_ms)
{
    uint8_t req[64];
    uint32_t xid = next_xid();
    int req_len = build_identify_request(req, xid, NULL);

    if (pn_eth_send(eth, PN_DCP_MULTICAST_MAC, req, (size_t)req_len) < 0)
        return -1;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    int count = 0;
    uint8_t resp[PN_ETH_MAX_PAYLOAD];
    uint8_t src_mac[PN_ETH_ALEN];

    for (;;) {
        long remaining = timeout_ms - elapsed_ms_since(&start);
        if (remaining <= 0)
            break;

        int n = pn_eth_recv(eth, src_mac, resp, sizeof(resp), (int)remaining);
        if (n < 0)
            return -1;
        if (n == 0)
            break;
        if (n < 10)
            continue;

        if (dcp_get_u16(resp) != PN_DCP_FRAMEID_GETSET)
            continue;
        if (resp[2] != PN_DCP_SERVICE_IDENTIFY || resp[3] != PN_DCP_SERVICE_TYPE_RESPONSE_SUCCESS)
            continue;
        if (dcp_get_u32(resp + 4) != xid)
            continue;

        bool dup = false;
        for (int i = 0; i < count; i++) {
            if (memcmp(out[i].mac, src_mac, PN_ETH_ALEN) == 0) {
                dup = true;
                break;
            }
        }
        if (dup || count >= max_devices)
            continue;

        memset(&out[count], 0, sizeof(out[count]));
        memcpy(out[count].mac, src_mac, PN_ETH_ALEN);
        parse_identify_response(resp, n, &out[count]);
        count++;
    }

    return count;
}
