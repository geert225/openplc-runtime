/**
 * @file profinet_eth.c
 * @brief Raw Ethernet (AF_PACKET) transport for PROFINET IO
 */

#define _GNU_SOURCE

#include "profinet_eth.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>

const uint8_t PN_DCP_MULTICAST_MAC[PN_ETH_ALEN] = {0x01, 0x0E, 0xCF, 0x00, 0x00, 0x00};

#define PN_ETH_HEADER_LEN 14 /* dst(6) + src(6) + ethertype(2) */

int pn_eth_open(pn_eth_handle_t *eth, const char *ifname)
{
    memset(eth, 0, sizeof(*eth));
    eth->fd = -1;

    int fd = socket(AF_PACKET, SOCK_RAW, htons(PN_ETHERTYPE_PROFINET));
    if (fd < 0)
        return -1;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return -1;
    }
    int ifindex = ifr.ifr_ifindex;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFHWADDR, &ifr) < 0) {
        close(fd);
        return -1;
    }

    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(PN_ETHERTYPE_PROFINET);
    sll.sll_ifindex  = ifindex;

    if (bind(fd, (struct sockaddr *)&sll, sizeof(sll)) < 0) {
        close(fd);
        return -1;
    }

    /* Receive DCP Identify responses/Hello frames sent to the well-known
     * DCP multicast address. */
    struct packet_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.mr_ifindex = ifindex;
    mreq.mr_type    = PACKET_MR_MULTICAST;
    mreq.mr_alen    = PN_ETH_ALEN;
    memcpy(mreq.mr_address, PN_DCP_MULTICAST_MAC, PN_ETH_ALEN);
    if (setsockopt(fd, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        close(fd);
        return -1;
    }

    eth->fd      = fd;
    eth->ifindex = ifindex;
    memcpy(eth->mac, ifr.ifr_hwaddr.sa_data, PN_ETH_ALEN);
    strncpy(eth->ifname, ifname, sizeof(eth->ifname) - 1);

    return 0;
}

void pn_eth_close(pn_eth_handle_t *eth)
{
    if (eth->fd >= 0) {
        close(eth->fd);
        eth->fd = -1;
    }
}

int pn_eth_send(pn_eth_handle_t *eth, const uint8_t dst_mac[PN_ETH_ALEN], const uint8_t *payload,
                size_t payload_len)
{
    if (payload_len > PN_ETH_MAX_PAYLOAD) {
        errno = EMSGSIZE;
        return -1;
    }

    uint8_t frame[PN_ETH_HEADER_LEN + PN_ETH_MAX_PAYLOAD];
    memcpy(frame, dst_mac, PN_ETH_ALEN);
    memcpy(frame + PN_ETH_ALEN, eth->mac, PN_ETH_ALEN);
    uint16_t ethertype = htons(PN_ETHERTYPE_PROFINET);
    memcpy(frame + 2 * PN_ETH_ALEN, &ethertype, sizeof(ethertype));
    memcpy(frame + PN_ETH_HEADER_LEN, payload, payload_len);

    size_t frame_len = PN_ETH_HEADER_LEN + payload_len;

    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family   = AF_PACKET;
    sll.sll_protocol = htons(PN_ETHERTYPE_PROFINET);
    sll.sll_ifindex  = eth->ifindex;
    sll.sll_halen    = PN_ETH_ALEN;
    memcpy(sll.sll_addr, dst_mac, PN_ETH_ALEN);

    ssize_t n = sendto(eth->fd, frame, frame_len, 0, (struct sockaddr *)&sll, sizeof(sll));
    if (n < 0)
        return -1;

    return (int)payload_len;
}

int pn_eth_recv(pn_eth_handle_t *eth, uint8_t src_mac[PN_ETH_ALEN], uint8_t *buf, size_t buf_len,
                int timeout_ms)
{
    struct pollfd pfd;
    pfd.fd     = eth->fd;
    pfd.events = POLLIN;

    int rc = poll(&pfd, 1, timeout_ms);
    if (rc < 0)
        return -1;
    if (rc == 0)
        return 0; /* timeout */

    uint8_t frame[PN_ETH_HEADER_LEN + PN_ETH_MAX_PAYLOAD];
    ssize_t n = recv(eth->fd, frame, sizeof(frame), 0);
    if (n < 0)
        return -1;
    if (n < PN_ETH_HEADER_LEN)
        return 0; /* truncated/invalid frame, treat as no data */

    if (src_mac)
        memcpy(src_mac, frame + PN_ETH_ALEN, PN_ETH_ALEN);

    size_t payload_len = (size_t)n - PN_ETH_HEADER_LEN;
    if (payload_len > buf_len)
        payload_len = buf_len;

    memcpy(buf, frame + PN_ETH_HEADER_LEN, payload_len);

    return (int)payload_len;
}
