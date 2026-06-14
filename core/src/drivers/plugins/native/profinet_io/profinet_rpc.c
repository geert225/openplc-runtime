/**
 * @file profinet_rpc.c
 * @brief PROFINET CL-RPC (Connectionless RPC over UDP) -- AR/CR establishment
 *
 * Implements the controller side of Connect/Control/Release as described in
 * IEC 61158-6-10. Block layouts (ARBlockReq, IOCRBlockReq,
 * AlarmCRBlockReq, ExpectedSubmoduleBlockReq, IODControlReq/Res) follow the
 * commonly documented field order and the well-known PNIO Device Interface
 * UUID (DEA00001-6C97-11D1-8271-00A02442DF7D). Some bitfields (ARProperties,
 * IOCRProperties, ModuleProperties, SubmoduleProperties) are sent with
 * conservative default values; this has not been validated against a
 * certified PROFINET IO device and may need adjustment for specific GSDML
 * requirements.
 */

#include "profinet_rpc.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PN_RPC_HEADER_LEN 80
#define PN_RPC_BUF_SIZE   4096

#define PN_RPC_PTYPE_REQUEST  0x00
#define PN_RPC_PTYPE_RESPONSE 0x02
#define PN_RPC_PTYPE_REJECT   0x03

#define PN_RPC_OP_CONNECT 0x0000
#define PN_RPC_OP_RELEASE 0x0001
#define PN_RPC_OP_CONTROL 0x0004

/* Block types (IEC 61158-6-10) */
#define PN_BLOCK_AR_REQ            0x0101
#define PN_BLOCK_IOCR_REQ          0x0102
#define PN_BLOCK_ALARM_CR_REQ      0x0103
#define PN_BLOCK_EXPECTED_SUBMOD   0x0104
#define PN_BLOCK_AR_RES            0x8101
#define PN_BLOCK_IOD_CONTROL_REQ   0x0110
#define PN_BLOCK_IOD_CONTROL_RES   0x8110
#define PN_BLOCK_RELEASE_REQ       0x0114

/* IODControlReq/Res ControlCommand bits */
#define PN_CTRL_CMD_PRM_END  0x0001
#define PN_CTRL_CMD_APP_RDY  0x0002
#define PN_CTRL_CMD_RELEASE  0x0004
#define PN_CTRL_CMD_DONE     0x0008

/** PNIO Device Interface UUID: DEA00001-6C97-11D1-8271-00A02442DF7D */
static const pn_uuid_t PNIO_INTERFACE_UUID = {
    {0xDE, 0xA0, 0x00, 0x01, 0x6C, 0x97, 0x11, 0xD1, 0x82, 0x71, 0x00, 0xA0, 0x24, 0x42, 0xDF, 0x7D}};

static plugin_logger_t *g_rpc_logger = NULL;

void pn_rpc_set_logger(plugin_logger_t *logger)
{
    g_rpc_logger = logger;
}

/*
 * =============================================================================
 * Byte-buffer reader/writer helpers
 * =============================================================================
 */

typedef struct {
    uint8_t *buf;
    size_t   pos;
    size_t   cap;
} pn_writer_t;

static void w_u8(pn_writer_t *w, uint8_t v)
{
    if (w->pos < w->cap)
        w->buf[w->pos] = v;
    w->pos++;
}

static void w_u16(pn_writer_t *w, uint16_t v)
{
    w_u8(w, (uint8_t)(v >> 8));
    w_u8(w, (uint8_t)(v & 0xFF));
}

static void w_u32(pn_writer_t *w, uint32_t v)
{
    w_u16(w, (uint16_t)(v >> 16));
    w_u16(w, (uint16_t)(v & 0xFFFF));
}

static void w_bytes(pn_writer_t *w, const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
        w_u8(w, p[i]);
}

static void w_uuid(pn_writer_t *w, const pn_uuid_t *u)
{
    w_bytes(w, u->b, 16);
}

static uint16_t r_u16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static uint32_t r_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/** Begin a block: writes BlockType + a placeholder BlockLength + version, returns the
 * position of the BlockLength field for block_end() to back-patch. */
static size_t block_begin(pn_writer_t *w, uint16_t block_type, uint8_t ver_high, uint8_t ver_low)
{
    w_u16(w, block_type);
    size_t len_pos = w->pos;
    w_u16(w, 0);
    w_u8(w, ver_high);
    w_u8(w, ver_low);
    return len_pos;
}

/** Finish a block: BlockLength = bytes written after the BlockLength field itself
 * (i.e. version bytes + content). */
static void block_end(pn_writer_t *w, size_t len_pos)
{
    uint16_t block_len = (uint16_t)(w->pos - (len_pos + 2));
    if (len_pos + 1 < w->cap) {
        w->buf[len_pos]     = (uint8_t)(block_len >> 8);
        w->buf[len_pos + 1] = (uint8_t)(block_len & 0xFF);
    }
}

/** Find the first block of type @p want_type within a block list. Returns
 * pointers to its content (everything after BlockVersionLow). */
static bool find_block(const uint8_t *buf, size_t len, uint16_t want_type, const uint8_t **content,
                        size_t *content_len)
{
    size_t pos = 0;
    while (pos + 6 <= len) {
        uint16_t btype = r_u16(buf + pos);
        uint16_t blen  = r_u16(buf + pos + 2);
        size_t content_start = pos + 6;
        size_t content_size  = (blen >= 2) ? ((size_t)blen - 2) : 0;
        if (content_start + content_size > len)
            break;
        if (btype == want_type) {
            if (content)
                *content = buf + content_start;
            if (content_len)
                *content_len = content_size;
            return true;
        }
        pos = content_start + content_size;
        if (content_size % 2)
            pos++;
    }
    return false;
}

/*
 * =============================================================================
 * RPC header
 * =============================================================================
 */

typedef struct {
    uint8_t   version;
    uint8_t   packet_type;
    uint8_t   flags1;
    uint8_t   flags2;
    uint8_t   data_rep[3];
    uint8_t   serial_high;
    pn_uuid_t object_uuid;
    pn_uuid_t interface_uuid;
    pn_uuid_t activity_uuid;
    uint32_t  server_boot_time;
    uint32_t  interface_version;
    uint32_t  sequence_number;
    uint16_t  operation_number;
    uint16_t  interface_hint;
    uint16_t  activity_hint;
    uint16_t  length_of_body;
    uint16_t  fragment_num;
    uint8_t   auth_protocol;
    uint8_t   serial_low;
} pn_rpc_header_t;

static void encode_rpc_header(pn_writer_t *w, const pn_rpc_header_t *h)
{
    w_u8(w, h->version);
    w_u8(w, h->packet_type);
    w_u8(w, h->flags1);
    w_u8(w, h->flags2);
    w_bytes(w, h->data_rep, 3);
    w_u8(w, h->serial_high);
    w_uuid(w, &h->object_uuid);
    w_uuid(w, &h->interface_uuid);
    w_uuid(w, &h->activity_uuid);
    w_u32(w, h->server_boot_time);
    w_u32(w, h->interface_version);
    w_u32(w, h->sequence_number);
    w_u16(w, h->operation_number);
    w_u16(w, h->interface_hint);
    w_u16(w, h->activity_hint);
    w_u16(w, h->length_of_body);
    w_u16(w, h->fragment_num);
    w_u8(w, h->auth_protocol);
    w_u8(w, h->serial_low);
}

static void decode_rpc_header(const uint8_t *p, pn_rpc_header_t *h)
{
    h->version     = p[0];
    h->packet_type = p[1];
    h->flags1      = p[2];
    h->flags2      = p[3];
    memcpy(h->data_rep, p + 4, 3);
    h->serial_high = p[7];
    memcpy(h->object_uuid.b, p + 8, 16);
    memcpy(h->interface_uuid.b, p + 24, 16);
    memcpy(h->activity_uuid.b, p + 40, 16);
    h->server_boot_time   = r_u32(p + 56);
    h->interface_version  = r_u32(p + 60);
    h->sequence_number    = r_u32(p + 64);
    h->operation_number   = r_u16(p + 68);
    h->interface_hint     = r_u16(p + 70);
    h->activity_hint      = r_u16(p + 72);
    h->length_of_body     = r_u16(p + 74);
    h->fragment_num       = r_u16(p + 76);
    h->auth_protocol      = p[78];
    h->serial_low         = p[79];
}

static void init_request_header(pn_rpc_header_t *hdr, const pn_uuid_t *object_uuid,
                                 const pn_uuid_t *activity_uuid, uint32_t sequence_number,
                                 uint16_t operation_number)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->version       = 4;
    hdr->packet_type   = PN_RPC_PTYPE_REQUEST;
    hdr->flags1        = 0x20; /* idempotent, last fragment */
    hdr->data_rep[0]   = 0x10; /* big-endian integers/floats, ASCII */
    hdr->object_uuid   = *object_uuid;
    hdr->interface_uuid = PNIO_INTERFACE_UUID;
    hdr->activity_uuid = *activity_uuid;
    hdr->server_boot_time = 0xFFFFFFFF;
    hdr->interface_version = 1;
    hdr->sequence_number = sequence_number;
    hdr->operation_number = operation_number;
    hdr->interface_hint = 0xFFFF;
    hdr->activity_hint  = 0xFFFF;
}

/*
 * =============================================================================
 * UUID / context init
 * =============================================================================
 */

static void gen_random_uuid(pn_uuid_t *u)
{
    FILE *f = fopen("/dev/urandom", "rb");
    if (f) {
        if (fread(u->b, 1, 16, f) != 16) {
            for (int i = 0; i < 16; i++)
                u->b[i] = (uint8_t)rand();
        }
        fclose(f);
    } else {
        for (int i = 0; i < 16; i++)
            u->b[i] = (uint8_t)rand();
    }
}

int pn_rpc_ctx_init(pn_rpc_ctx_t *ctx, int shared_fd, const char *device_ip,
                     uint16_t input_frame_id, uint16_t output_frame_id)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->fd = shared_fd;
    ctx->addr.sin_family = AF_INET;
    ctx->addr.sin_port   = htons(PN_RPC_PORT);
    if (inet_pton(AF_INET, device_ip, &ctx->addr.sin_addr) != 1)
        return -1;

    gen_random_uuid(&ctx->ar_uuid);
    gen_random_uuid(&ctx->activity_uuid);
    ctx->session_key = (uint16_t)((rand() & 0xFFFE) + 1); /* avoid 0 */
    ctx->sequence_number = 0;
    ctx->input_frame_id  = input_frame_id;
    ctx->output_frame_id = output_frame_id;
    return 0;
}

/*
 * =============================================================================
 * Receive helper
 * =============================================================================
 */

static long elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000L + (now.tv_nsec - start->tv_nsec) / 1000000L;
}

/** Receive a UDP datagram from ctx->addr's IP, dropping anything else. */
static int recv_from_device(pn_rpc_ctx_t *ctx, uint8_t *buf, size_t buf_len, int timeout_ms)
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        long remaining = timeout_ms - elapsed_ms_since(&start);
        if (remaining <= 0)
            return 0;

        struct pollfd pfd = {.fd = ctx->fd, .events = POLLIN, .revents = 0};
        int rc = poll(&pfd, 1, (int)remaining);
        if (rc < 0)
            return -1;
        if (rc == 0)
            return 0;

        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        ssize_t n = recvfrom(ctx->fd, buf, buf_len, 0, (struct sockaddr *)&from, &fromlen);
        if (n < 0)
            return -1;
        if (from.sin_addr.s_addr != ctx->addr.sin_addr.s_addr)
            continue;

        return (int)n;
    }
}

/*
 * =============================================================================
 * Block builders
 * =============================================================================
 */

static void append_ar_block_req(pn_writer_t *w, const pn_rpc_ctx_t *ctx,
                                 const pn_master_config_t *master, const uint8_t controller_mac[6])
{
    size_t lp = block_begin(w, PN_BLOCK_AR_REQ, 1, 0);

    w_u16(w, 0x0001); /* ARType = IOCARSingle */
    w_uuid(w, &ctx->ar_uuid);
    w_u16(w, ctx->session_key);
    w_bytes(w, controller_mac, 6); /* CMInitiatorMacAddr */
    w_uuid(w, &ctx->ar_uuid);       /* CMInitiatorObjectUUID (reuse ARUUID) */
    w_u32(w, 0x00000001);          /* ARProperties: State = Active */
    w_u16(w, 1000);                /* CMInitiatorActivityTimeoutFactor (x10ms = 10s) */
    w_u16(w, 0xC02B);               /* CMInitiatorUDPRTPort */

    uint16_t name_len = (uint16_t)strlen(master->station_name);
    w_u16(w, name_len);
    w_bytes(w, (const uint8_t *)master->station_name, name_len);
    if (name_len % 2)
        w_u8(w, 0); /* pad to even */

    block_end(w, lp);
}

typedef struct {
    int slot_number;
    int subslot_number;
    int frame_offset;
} pn_io_obj_t;

/** Build the IODataObject (and IOCS status byte) layout for one direction,
 * walking the device's slots/submodules in configuration order -- this
 * defines the byte layout of the device's flat input_data[]/output_data[]
 * cyclic buffer used by profinet_io.c. */
static void build_io_layout(const pn_device_t *device, bool is_input, pn_io_obj_t *objs,
                             int max_objs, int *out_n_obj, pn_io_obj_t *iocs, int max_iocs,
                             int *out_n_iocs, int *out_data_len)
{
    int n_obj = 0;
    int offset = 0;

    for (int i = 0; i < device->slot_count; i++) {
        const pn_slot_t *slot = &device->slots[i];
        for (int j = 0; j < slot->submodule_count; j++) {
            const pn_submodule_t *sm = &slot->submodules[j];
            int len = is_input ? sm->input_length : sm->output_length;
            if (len <= 0)
                continue;
            if (n_obj < max_objs) {
                objs[n_obj].slot_number    = slot->slot_number;
                objs[n_obj].subslot_number = sm->subslot_number;
                objs[n_obj].frame_offset   = offset;
                n_obj++;
            }
            offset += len;
        }
    }

    int n_iocs = 0;
    for (int i = 0; i < n_obj && i < max_iocs; i++) {
        iocs[i].slot_number    = objs[i].slot_number;
        iocs[i].subslot_number = objs[i].subslot_number;
        iocs[i].frame_offset   = offset;
        offset += 1; /* one IOPS/IOCS status byte per data object */
        n_iocs++;
    }

    *out_n_obj  = n_obj;
    *out_n_iocs = n_iocs;
    /* + CycleCounter(2) + DataStatus(1) + TransferStatus(1) */
    *out_data_len = offset + 4;
}

static void append_iocr_block_req(pn_writer_t *w, const pn_rpc_ctx_t *ctx,
                                   const pn_master_config_t *master, const pn_device_t *device,
                                   bool is_input)
{
    pn_io_obj_t objs[PN_MAX_CHANNELS];
    pn_io_obj_t iocs[PN_MAX_CHANNELS];
    int n_obj = 0, n_iocs = 0, data_len = 0;

    build_io_layout(device, is_input, objs, PN_MAX_CHANNELS, &n_obj, iocs, PN_MAX_CHANNELS,
                    &n_iocs, &data_len);

    size_t lp = block_begin(w, PN_BLOCK_IOCR_REQ, 1, 0);

    w_u16(w, is_input ? 0x0001 : 0x0002); /* IOCRType: Input / Output */
    w_u16(w, is_input ? 0x0001 : 0x0002); /* IOCRReference */
    w_u16(w, PN_ETHERTYPE_PROFINET);      /* LT = 0x8892 */
    w_u32(w, 0x00000001);                 /* IOCRProperties: RT_CLASS_1 */
    w_u16(w, (uint16_t)data_len);
    w_u16(w, is_input ? ctx->input_frame_id : ctx->output_frame_id);
    w_u16(w, (uint16_t)master->send_clock_factor);
    w_u16(w, (uint16_t)master->reduction_ratio);
    w_u16(w, 1); /* Phase */
    w_u16(w, 0); /* Sequence */
    w_u32(w, 0xFFFFFFFF); /* FrameSendOffset: not used */
    w_u16(w, (uint16_t)device->watchdog_factor); /* WatchdogFactor */
    w_u16(w, (uint16_t)device->watchdog_factor); /* DataHoldFactor */
    w_u16(w, 0xC000);                            /* IOCRTagHeader */

    static const uint8_t zero_mac[6] = {0, 0, 0, 0, 0, 0};
    w_bytes(w, zero_mac, 6); /* IOCRMulticastMACAddress (unused for unicast RT_CLASS_1) */

    w_u16(w, 1); /* NumberOfAPIs */
    w_u32(w, 0); /* API */

    w_u16(w, (uint16_t)n_obj);
    for (int i = 0; i < n_obj; i++) {
        w_u16(w, (uint16_t)objs[i].slot_number);
        w_u16(w, (uint16_t)objs[i].subslot_number);
        w_u16(w, (uint16_t)objs[i].frame_offset);
    }

    w_u16(w, (uint16_t)n_iocs);
    for (int i = 0; i < n_iocs; i++) {
        w_u16(w, (uint16_t)iocs[i].slot_number);
        w_u16(w, (uint16_t)iocs[i].subslot_number);
        w_u16(w, (uint16_t)iocs[i].frame_offset);
    }

    block_end(w, lp);
}

static void append_alarm_cr_block_req(pn_writer_t *w)
{
    size_t lp = block_begin(w, PN_BLOCK_ALARM_CR_REQ, 1, 0);

    w_u16(w, 0x0001);            /* AlarmCRType */
    w_u16(w, PN_ETHERTYPE_PROFINET); /* LT */
    w_u32(w, 0);                 /* AlarmCRProperties */
    w_u16(w, 1);                 /* RTATimeoutFactor */
    w_u16(w, 3);                 /* RTARetries */
    w_u16(w, 1);                 /* LocalAlarmReference */
    w_u16(w, 200);               /* MaxAlarmDataLength */
    w_u16(w, 0xC000);            /* AlarmCRTagHeader.High */
    w_u16(w, 0xA000);            /* AlarmCRTagHeader.Low */

    block_end(w, lp);
}

static void append_expected_submodule_block_req(pn_writer_t *w, const pn_device_t *device)
{
    size_t lp = block_begin(w, PN_BLOCK_EXPECTED_SUBMOD, 1, 0);

    w_u16(w, 1); /* NumberOfAPIs */
    w_u32(w, 0); /* API */
    w_u16(w, (uint16_t)device->slot_count);

    for (int i = 0; i < device->slot_count; i++) {
        const pn_slot_t *slot = &device->slots[i];

        w_u16(w, (uint16_t)slot->slot_number);
        w_u32(w, slot->module_ident_number);
        w_u16(w, 0); /* ModuleProperties */
        w_u16(w, (uint16_t)slot->submodule_count);

        for (int j = 0; j < slot->submodule_count; j++) {
            const pn_submodule_t *sm = &slot->submodules[j];

            w_u16(w, (uint16_t)sm->subslot_number);
            w_u32(w, sm->submodule_ident_number);
            w_u16(w, 0); /* SubmoduleProperties */

            if (sm->input_length <= 0 && sm->output_length <= 0) {
                w_u16(w, 1); /* DataDescription = Input */
                w_u16(w, 0); /* SubmoduleDataLength */
                w_u8(w, 0);  /* LengthIOCS */
                w_u8(w, 0);  /* LengthIOPS */
            } else {
                if (sm->input_length > 0) {
                    w_u16(w, 1); /* Input */
                    w_u16(w, (uint16_t)sm->input_length);
                    w_u8(w, 1); /* LengthIOCS */
                    w_u8(w, 1); /* LengthIOPS */
                }
                if (sm->output_length > 0) {
                    w_u16(w, 2); /* Output */
                    w_u16(w, (uint16_t)sm->output_length);
                    w_u8(w, 1); /* LengthIOCS */
                    w_u8(w, 1); /* LengthIOPS */
                }
            }
        }
    }

    block_end(w, lp);
}

static size_t append_iod_control_block(pn_writer_t *w, uint16_t block_type,
                                        const pn_uuid_t *ar_uuid, uint16_t session_key,
                                        uint16_t control_command)
{
    size_t lp = block_begin(w, block_type, 1, 0);

    w_u16(w, 0); /* ReservedInterface */
    w_uuid(w, ar_uuid);
    w_u16(w, session_key);
    w_u16(w, 0); /* Reserved */
    w_u16(w, control_command);
    w_u16(w, 0); /* ControlBlockProperties */

    block_end(w, lp);
    return lp;
}

/*
 * =============================================================================
 * Public API
 * =============================================================================
 */

int pn_rpc_connect(pn_rpc_ctx_t *ctx, const uint8_t controller_mac[6],
                    const pn_master_config_t *master, const pn_device_t *device, int timeout_ms)
{
    uint8_t pdu[PN_RPC_BUF_SIZE];
    pn_writer_t w = {pdu, PN_RPC_HEADER_LEN, sizeof(pdu)};

    append_ar_block_req(&w, ctx, master, controller_mac);
    append_iocr_block_req(&w, ctx, master, device, true);
    append_iocr_block_req(&w, ctx, master, device, false);
    append_alarm_cr_block_req(&w);
    append_expected_submodule_block_req(&w, device);

    if (w.pos > w.cap) {
        if (g_rpc_logger && g_rpc_logger->is_valid)
            plugin_logger_error(g_rpc_logger, "Connect.req for device '%s' exceeds buffer size",
                                 device->name);
        return -1;
    }

    pn_rpc_header_t hdr;
    pn_uuid_t null_uuid;
    memset(&null_uuid, 0, sizeof(null_uuid)); /* AR doesn't exist yet */
    init_request_header(&hdr, &null_uuid, &ctx->activity_uuid, ctx->sequence_number++,
                         PN_RPC_OP_CONNECT);
    hdr.length_of_body = (uint16_t)(w.pos - PN_RPC_HEADER_LEN);

    pn_writer_t hw = {pdu, 0, PN_RPC_HEADER_LEN};
    encode_rpc_header(&hw, &hdr);

    if (sendto(ctx->fd, pdu, w.pos, 0, (struct sockaddr *)&ctx->addr, sizeof(ctx->addr)) < 0)
        return -1;

    uint8_t resp[PN_RPC_BUF_SIZE];
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        long remaining = timeout_ms - elapsed_ms_since(&start);
        if (remaining <= 0)
            return -1;

        int n = recv_from_device(ctx, resp, sizeof(resp), (int)remaining);
        if (n < 0)
            return -1;
        if (n == 0)
            return -1;
        if ((size_t)n < PN_RPC_HEADER_LEN)
            continue;

        pn_rpc_header_t rhdr;
        decode_rpc_header(resp, &rhdr);
        if (memcmp(rhdr.activity_uuid.b, ctx->activity_uuid.b, 16) != 0)
            continue;

        if (rhdr.packet_type == PN_RPC_PTYPE_REJECT) {
            if (g_rpc_logger && g_rpc_logger->is_valid)
                plugin_logger_warn(g_rpc_logger, "Device '%s' rejected Connect.req", device->name);
            return -2;
        }
        if (rhdr.packet_type != PN_RPC_PTYPE_RESPONSE)
            continue;

        const uint8_t *body = resp + PN_RPC_HEADER_LEN;
        size_t body_len = (size_t)n - PN_RPC_HEADER_LEN;

        if (find_block(body, body_len, PN_BLOCK_AR_RES, NULL, NULL))
            return 0;

        if (g_rpc_logger && g_rpc_logger->is_valid)
            plugin_logger_warn(g_rpc_logger, "Device '%s' Connect.res missing ARBlockRes",
                                device->name);
        return -2;
    }
}

static int send_control(pn_rpc_ctx_t *ctx, uint16_t control_command, int timeout_ms)
{
    uint8_t pdu[256];
    pn_writer_t w = {pdu, PN_RPC_HEADER_LEN, sizeof(pdu)};

    append_iod_control_block(&w, PN_BLOCK_IOD_CONTROL_REQ, &ctx->ar_uuid, ctx->session_key,
                              control_command);

    pn_rpc_header_t hdr;
    init_request_header(&hdr, &ctx->ar_uuid, &ctx->activity_uuid, ctx->sequence_number++,
                         PN_RPC_OP_CONTROL);
    hdr.length_of_body = (uint16_t)(w.pos - PN_RPC_HEADER_LEN);

    pn_writer_t hw = {pdu, 0, PN_RPC_HEADER_LEN};
    encode_rpc_header(&hw, &hdr);

    if (sendto(ctx->fd, pdu, w.pos, 0, (struct sockaddr *)&ctx->addr, sizeof(ctx->addr)) < 0)
        return -1;

    uint8_t resp[256];
    int n = recv_from_device(ctx, resp, sizeof(resp), timeout_ms);
    if (n <= 0)
        return -1;
    if ((size_t)n < PN_RPC_HEADER_LEN)
        return -1;

    pn_rpc_header_t rhdr;
    decode_rpc_header(resp, &rhdr);
    if (rhdr.packet_type != PN_RPC_PTYPE_RESPONSE)
        return -2;

    return 0;
}

int pn_rpc_control_prmend(pn_rpc_ctx_t *ctx, int timeout_ms)
{
    return send_control(ctx, PN_CTRL_CMD_PRM_END, timeout_ms);
}

int pn_rpc_wait_application_ready(pn_rpc_ctx_t *ctx, int timeout_ms)
{
    uint8_t resp[512];
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        long remaining = timeout_ms - elapsed_ms_since(&start);
        if (remaining <= 0)
            return 0;

        int n = recv_from_device(ctx, resp, sizeof(resp), (int)remaining);
        if (n < 0)
            return -1;
        if (n == 0)
            return 0;
        if ((size_t)n < PN_RPC_HEADER_LEN)
            continue;

        pn_rpc_header_t rhdr;
        decode_rpc_header(resp, &rhdr);
        if (rhdr.packet_type != PN_RPC_PTYPE_REQUEST || rhdr.operation_number != PN_RPC_OP_CONTROL)
            continue;
        if (memcmp(rhdr.object_uuid.b, ctx->ar_uuid.b, 16) != 0)
            continue;

        const uint8_t *body = resp + PN_RPC_HEADER_LEN;
        size_t body_len = (size_t)n - PN_RPC_HEADER_LEN;

        const uint8_t *content;
        size_t content_len;
        if (!find_block(body, body_len, PN_BLOCK_IOD_CONTROL_REQ, &content, &content_len))
            continue;
        if (content_len < 26)
            continue;

        uint16_t control_command = r_u16(content + 22);
        if (!(control_command & PN_CTRL_CMD_APP_RDY))
            continue;

        /* Acknowledge with IODControlRes(Done), echoing the request's
         * sequence number/activity as the response. */
        uint8_t out[256];
        pn_writer_t ow = {out, PN_RPC_HEADER_LEN, sizeof(out)};
        append_iod_control_block(&ow, PN_BLOCK_IOD_CONTROL_RES, &ctx->ar_uuid, ctx->session_key,
                                  PN_CTRL_CMD_DONE);

        pn_rpc_header_t ohdr = rhdr;
        ohdr.packet_type = PN_RPC_PTYPE_RESPONSE;
        ohdr.length_of_body = (uint16_t)(ow.pos - PN_RPC_HEADER_LEN);

        pn_writer_t ohw = {out, 0, PN_RPC_HEADER_LEN};
        encode_rpc_header(&ohw, &ohdr);

        sendto(ctx->fd, out, ow.pos, 0, (struct sockaddr *)&ctx->addr, sizeof(ctx->addr));

        return 1;
    }
}

int pn_rpc_release(pn_rpc_ctx_t *ctx, int timeout_ms)
{
    uint8_t pdu[256];
    pn_writer_t w = {pdu, PN_RPC_HEADER_LEN, sizeof(pdu)};

    append_iod_control_block(&w, PN_BLOCK_RELEASE_REQ, &ctx->ar_uuid, ctx->session_key,
                              PN_CTRL_CMD_RELEASE);

    pn_rpc_header_t hdr;
    init_request_header(&hdr, &ctx->ar_uuid, &ctx->activity_uuid, ctx->sequence_number++,
                         PN_RPC_OP_RELEASE);
    hdr.length_of_body = (uint16_t)(w.pos - PN_RPC_HEADER_LEN);

    pn_writer_t hw = {pdu, 0, PN_RPC_HEADER_LEN};
    encode_rpc_header(&hw, &hdr);

    sendto(ctx->fd, pdu, w.pos, 0, (struct sockaddr *)&ctx->addr, sizeof(ctx->addr));

    uint8_t resp[256];
    recv_from_device(ctx, resp, sizeof(resp), timeout_ms); /* best-effort */

    return 0;
}
