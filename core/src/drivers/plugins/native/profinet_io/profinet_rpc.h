/**
 * @file profinet_rpc.h
 * @brief PROFINET CL-RPC (Connectionless RPC over UDP) -- AR/CR establishment
 *
 * Implements the controller side of the PROFINET IO "Context Management"
 * (CM) protocol used to establish an Application Relationship (AR) with one
 * Communication Relationship (CR) pair (one input IOCR + one output IOCR)
 * per device:
 *
 *   - Connect.req / Connect.res  (operation Connect)
 *   - IODControlReq(PrmEnd) / IODControlRes (operation Control)
 *   - IODControlReq(ApplicationReady) from the device / IODControlRes (Done)
 *   - Release.req / Release.res (operation Release)
 *
 * All PDUs are exchanged over a single UDP socket bound to port 34964,
 * shared by every AR managed by a controller instance. Each AR is tracked
 * by a pn_rpc_ctx_t (its own ARUUID/ActivityUUID/SessionKey/sequence
 * number and the device's UDP address). Because the socket is shared,
 * callers must serialize RPC operations across devices (e.g. with a mutex)
 * -- packets not matching the context's expected peer/ActivityUUID are
 * dropped.
 */

#ifndef PROFINET_RPC_H
#define PROFINET_RPC_H

#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>

#include "profinet_config.h"
#include "plugin_logger.h"

#define PN_RPC_PORT 34964

/** Raw 16-byte UUID, stored in the big-endian wire format used by PNIO RPC. */
typedef struct {
    uint8_t b[16];
} pn_uuid_t;

/**
 * @brief Per-AR RPC context.
 *
 * One instance per device, sharing a single UDP socket (@p fd) that the
 * master opens once and binds to PN_RPC_PORT.
 */
typedef struct {
    int fd; /* shared UDP socket, not owned by this context */
    struct sockaddr_in addr; /* device's UDP address (port PN_RPC_PORT) */

    pn_uuid_t ar_uuid;
    pn_uuid_t activity_uuid;
    uint16_t  session_key;
    uint32_t  sequence_number;

    /* FrameIDs assigned by the controller for this AR's IOCRs (chosen by
     * the caller before connecting, echoed back into IOCRBlockReq). */
    uint16_t input_frame_id;
    uint16_t output_frame_id;
} pn_rpc_ctx_t;

/** Provide a logger for diagnostic messages. */
void pn_rpc_set_logger(plugin_logger_t *logger);

/**
 * @brief Initialize an RPC context for a device.
 *
 * Generates a fresh ARUUID/ActivityUUID/SessionKey and resolves
 * @p device_ip into ctx->addr (port PN_RPC_PORT).
 *
 * @return 0 on success, -1 on invalid address.
 */
int pn_rpc_ctx_init(pn_rpc_ctx_t *ctx, int shared_fd, const char *device_ip,
                     uint16_t input_frame_id, uint16_t output_frame_id);

/**
 * @brief Send Connect.req and wait for a successful Connect.res.
 *
 * Builds ARBlockReq, IOCRBlockReq (input + output), AlarmCRBlockReq and
 * ExpectedSubmoduleBlockReq from @p master / @p device, using
 * ctx->input_frame_id / ctx->output_frame_id for the IOCRs.
 *
 * @param controller_mac This controller's own MAC address (used as
 *                        CMInitiatorMacAddr).
 * @return 0 on success (PNIOStatus OK), -1 on timeout/transport error,
 *         -2 if the device returned a non-zero PNIOStatus.
 */
int pn_rpc_connect(pn_rpc_ctx_t *ctx, const uint8_t controller_mac[6],
                    const pn_master_config_t *master, const pn_device_t *device, int timeout_ms);

/**
 * @brief Send IODControlReq(PrmEnd) and wait for IODControlRes.
 */
int pn_rpc_control_prmend(pn_rpc_ctx_t *ctx, int timeout_ms);

/**
 * @brief Wait for the device's IODControlReq(ApplicationReady) and reply
 *        with IODControlRes(Done).
 *
 * @return 1 if ApplicationReady was received and acknowledged, 0 on
 *         timeout, -1 on transport error.
 */
int pn_rpc_wait_application_ready(pn_rpc_ctx_t *ctx, int timeout_ms);

/**
 * @brief Send Release.req and wait (briefly) for Release.res. Best-effort:
 *        returns 0 even on timeout, since the AR is being torn down anyway.
 */
int pn_rpc_release(pn_rpc_ctx_t *ctx, int timeout_ms);

#endif /* PROFINET_RPC_H */
