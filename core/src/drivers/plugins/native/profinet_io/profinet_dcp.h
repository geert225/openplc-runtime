/**
 * @file profinet_dcp.h
 * @brief PROFINET DCP (Discovery and Configuration Protocol) -- Identify
 *
 * Implements the subset of DCP needed by an IO Controller to resolve a
 * device's MAC address from its configured NameOfStation (DCP Identify
 * Request, multicast, with a NameOfStation filter block) and to enumerate
 * all PROFINET devices reachable on the local segment (DCP Identify
 * Request with the "All Selector" block).
 *
 * DCP runs directly over Ethernet (EtherType 0x8892, FrameID 0xFEFE for the
 * request and 0xFEFD for unicast responses) via profinet_eth.h.
 */

#ifndef PROFINET_DCP_H
#define PROFINET_DCP_H

#include <stdint.h>

#include "profinet_eth.h"
#include "profinet_config.h"
#include "plugin_logger.h"

/**
 * @brief Information extracted from a DCP Identify Response.
 */
typedef struct {
    uint8_t  mac[PN_ETH_ALEN];
    char     station_name[PN_MAX_STATION_NAME_LEN];
    char     ip_address[PN_MAX_IP_LEN]; /* "" if the device has no IP set */
    uint32_t vendor_id;
    uint32_t device_id;
} pn_dcp_device_info_t;

/** Provide a logger for diagnostic messages. */
void pn_dcp_set_logger(plugin_logger_t *logger);

/**
 * @brief Resolve a single device's MAC address (and IP/vendor/device IDs)
 *        by sending a DCP Identify Request filtered to @p station_name.
 *
 * @param eth          Open raw Ethernet handle (see profinet_eth.h).
 * @param station_name NameOfStation to filter for.
 * @param out          Output: populated on success.
 * @param timeout_ms   Maximum time to wait for a matching response.
 * @return 1 if a matching device responded, 0 on timeout, -1 on error.
 */
int pn_dcp_identify_by_name(pn_eth_handle_t *eth, const char *station_name,
                             pn_dcp_device_info_t *out, int timeout_ms);

/**
 * @brief Enumerate all PROFINET devices on the local segment via a DCP
 *        Identify Request with the "All Selector" block.
 *
 * Collects responses until @p timeout_ms elapses since the request was
 * sent. Duplicate responses from the same MAC are ignored.
 *
 * @return Number of devices found (0..max_devices), or -1 on error.
 */
int pn_dcp_identify_all(pn_eth_handle_t *eth, pn_dcp_device_info_t *out, int max_devices,
                         int timeout_ms);

#endif /* PROFINET_DCP_H */
