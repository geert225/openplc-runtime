/**
 * @file profinet_eth.h
 * @brief Raw Ethernet (AF_PACKET) transport for PROFINET IO
 *
 * PROFINET IO's DCP (Discovery and Configuration Protocol) and RT (Real-Time)
 * cyclic data are both carried directly in Ethernet frames with EtherType
 * 0x8892 -- there is no IP/UDP layer involved. This module provides a thin
 * wrapper around a Linux AF_PACKET socket bound to a single interface,
 * filtered to that EtherType, for sending and receiving such frames.
 *
 * RPC (Connect.req/res, IODControl, etc.) uses ordinary UDP/IP (port 34964)
 * and is handled separately in profinet_rpc.c via a normal socket.
 */

#ifndef PROFINET_ETH_H
#define PROFINET_ETH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** EtherType used by PROFINET IO for both DCP and RT frames. */
#define PN_ETHERTYPE_PROFINET 0x8892

#define PN_ETH_ALEN 6

/** Maximum Ethernet payload (after the 14-byte header) this module will
 * send/receive. Covers standard (non-jumbo) frames. */
#define PN_ETH_MAX_PAYLOAD 1500

/** DCP multicast destination MAC address: 01:0E:CF:00:00:00 */
extern const uint8_t PN_DCP_MULTICAST_MAC[PN_ETH_ALEN];

/**
 * @brief Open handle for a raw PROFINET (EtherType 0x8892) socket bound to
 *        a single network interface.
 */
typedef struct {
    int     fd;
    int     ifindex;
    uint8_t mac[PN_ETH_ALEN]; /* interface's own MAC address */
    char    ifname[16];
} pn_eth_handle_t;

/**
 * @brief Open a raw AF_PACKET socket bound to @p ifname, filtered to
 *        EtherType 0x8892, and join the DCP multicast group.
 *
 * @return 0 on success, -1 on failure (errno set).
 */
int pn_eth_open(pn_eth_handle_t *eth, const char *ifname);

/** Close the socket. Safe to call on an already-closed/zeroed handle. */
void pn_eth_close(pn_eth_handle_t *eth);

/**
 * @brief Send a PROFINET (EtherType 0x8892) Ethernet frame.
 *
 * @param dst_mac     6-byte destination MAC address.
 * @param payload     Frame payload (the PROFINET PDU, starting at the
 *                     FrameID field).
 * @param payload_len Length of @p payload, must be <= PN_ETH_MAX_PAYLOAD.
 * @return Number of bytes written (payload_len) on success, -1 on error.
 */
int pn_eth_send(pn_eth_handle_t *eth, const uint8_t dst_mac[PN_ETH_ALEN],
                const uint8_t *payload, size_t payload_len);

/**
 * @brief Receive the next PROFINET (EtherType 0x8892) frame, waiting at
 *        most @p timeout_ms milliseconds.
 *
 * @param src_mac   Output: 6-byte source MAC of the received frame (may be NULL).
 * @param buf       Output buffer for the payload (starting at FrameID).
 * @param buf_len   Size of @p buf.
 * @param timeout_ms Maximum time to wait, in milliseconds. 0 = poll once, don't block.
 * @return Payload length (>0) on success, 0 on timeout (no frame received),
 *         -1 on error (errno set).
 */
int pn_eth_recv(pn_eth_handle_t *eth, uint8_t src_mac[PN_ETH_ALEN], uint8_t *buf, size_t buf_len,
                int timeout_ms);

#endif /* PROFINET_ETH_H */
