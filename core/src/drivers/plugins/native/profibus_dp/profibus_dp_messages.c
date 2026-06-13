/**
 * @file profibus_dp_messages.c
 * @brief DP (DPV0) and DPV1 acyclic/cyclic message builders and parsers
 */

#include "profibus_dp_messages.h"

#include <string.h>

uint8_t pb_msg_request_fc(uint8_t function_code, bool fcb, bool fcv)
{
    uint8_t fc = (uint8_t)(PB_FC_REQ | (function_code & 0x1F));
    if (fcv)
        fc |= PB_FC_FCV;
    if (fcb)
        fc |= PB_FC_FCB;
    return fc;
}

int pb_msg_build_slave_diag_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                     uint8_t master_addr, bool fcb, bool fcv)
{
    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_HI, fcb, fcv);
    return pb_fdl_build_sap_request(out, out_max, slave_addr, PB_SAP_SLAVE_DIAG, master_addr,
                                     PB_MASTER_SAP, fc, NULL, 0);
}

int pb_msg_parse_slave_diag_response(const pb_fdl_telegram_t *rx, pb_diag_t *out)
{
    /* DU = [DSAP_echo, SSAP_echo, Stat_1, Stat_2, Stat_3, Master_Add,
     *       Ident_Number_Hi, Ident_Number_Lo, Ext_Diag_Data...] */
    if (rx->data_len < 8)
        return -1;

    out->stat_1 = rx->data[2];
    out->stat_2 = rx->data[3];
    out->stat_3 = rx->data[4];
    out->master_add = rx->data[5];
    out->ident_number = (uint16_t)((rx->data[6] << 8) | rx->data[7]);

    out->ext_diag_len = rx->data_len - 8;
    if (out->ext_diag_len > 0) {
        if (out->ext_diag_len > PB_MAX_EXT_DIAG_LEN)
            out->ext_diag_len = PB_MAX_EXT_DIAG_LEN;
        memcpy(out->ext_diag_data, &rx->data[8], (size_t)out->ext_diag_len);
    } else {
        out->ext_diag_len = 0;
    }

    return 0;
}

void pb_msg_compute_watchdog_factors(int watchdog_ms, uint8_t *fact1, uint8_t *fact2)
{
    long units = (watchdog_ms + 9) / 10; /* ceil to 10ms steps */
    if (units < 1)
        units = 1;

    long f2 = 1;
    long f1 = units;
    while (f1 > 255 && f2 < 255) {
        f2++;
        f1 = (units + f2 - 1) / f2;
    }

    if (f1 > 255)
        f1 = 255;
    if (f2 > 255)
        f2 = 255;

    *fact1 = (uint8_t)f1;
    *fact2 = (uint8_t)f2;
}

int pb_msg_build_set_prm_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                  uint8_t master_addr, bool fcb, bool fcv,
                                  const pb_slave_t *slave)
{
    uint8_t payload[7 + PB_MAX_PRM_DATA_LEN];
    int prm_len = slave->user_prm_data_len;
    if (prm_len < 0)
        prm_len = 0;
    if (prm_len > PB_MAX_PRM_DATA_LEN)
        prm_len = PB_MAX_PRM_DATA_LEN;

    uint8_t status = 0;
    if (slave->watchdog_enabled)
        status |= PB_PRM_STATUS_WD_ON;

    uint8_t wd_fact1, wd_fact2;
    pb_msg_compute_watchdog_factors(slave->watchdog_ms, &wd_fact1, &wd_fact2);

    payload[0] = status;
    payload[1] = wd_fact1;
    payload[2] = wd_fact2;
    payload[3] = (uint8_t)slave->min_tsdr;
    payload[4] = (uint8_t)(slave->ident_number >> 8);
    payload[5] = (uint8_t)(slave->ident_number & 0xFF);
    payload[6] = (slave->group >= 1 && slave->group <= 8) ? (uint8_t)(1u << (slave->group - 1)) : 0;

    if (prm_len > 0)
        memcpy(&payload[7], slave->user_prm_data, (size_t)prm_len);

    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_HI, fcb, fcv);
    return pb_fdl_build_sap_request(out, out_max, slave_addr, PB_SAP_SET_PRM, master_addr,
                                     PB_MASTER_SAP, fc, payload, 7 + prm_len);
}

int pb_msg_build_chk_cfg_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                  uint8_t master_addr, bool fcb, bool fcv,
                                  const pb_slave_t *slave)
{
    int cfg_len = slave->cfg_data_len;
    if (cfg_len <= 0 || cfg_len > PB_MAX_CFG_DATA_LEN)
        return -1;

    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_HI, fcb, fcv);
    return pb_fdl_build_sap_request(out, out_max, slave_addr, PB_SAP_CHK_CFG, master_addr,
                                     PB_MASTER_SAP, fc, slave->cfg_data, cfg_len);
}

bool pb_msg_response_is_ok(const pb_fdl_telegram_t *rx)
{
    if (rx->type == PB_TEL_SC)
        return true;

    if (rx->type == PB_TEL_SD1) {
        switch (rx->fc) {
        case PB_FC_RES_OK:
        case PB_FC_RES_DL:
        case PB_FC_RES_DH:
            return true;
        default:
            return false;
        }
    }

    return false;
}

int pb_msg_build_data_exchange_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                        uint8_t master_addr, bool fcb, bool fcv,
                                        const uint8_t *output_data, int output_len)
{
    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_LO, fcb, fcv);
    return pb_fdl_build_data_request(out, out_max, slave_addr, master_addr, fc, output_data,
                                      output_len);
}

int pb_msg_parse_data_exchange_response(const pb_fdl_telegram_t *rx, uint8_t *input_data,
                                         int input_max, int *out_len)
{
    if (rx->type == PB_TEL_SC) {
        *out_len = 0;
        return 0;
    }

    if (rx->type == PB_TEL_SD2 || rx->type == PB_TEL_SD3) {
        int n = rx->data_len;
        if (n > input_max)
            n = input_max;
        if (n > 0)
            memcpy(input_data, rx->data, (size_t)n);
        *out_len = n;
        return 0;
    }

    if (rx->type == PB_TEL_SD1) {
        /* SD1 response with no data union -- e.g. RES_NR (no data
         * available yet) or an error code in rx->fc. Treat as "no input
         * data this cycle" and let the caller inspect rx->fc if needed. */
        *out_len = 0;
        return 0;
    }

    return -1;
}

int pb_msg_build_global_control(uint8_t *out, int out_max, uint8_t master_addr,
                                 uint8_t control_command, uint8_t group_mask)
{
    uint8_t du[2] = { control_command, group_mask };
    uint8_t fc = pb_msg_request_fc(PB_FC_SDN_HI, false, false);
    return pb_fdl_build_sap_request(out, out_max, PB_FDL_BROADCAST_ADDR, PB_SAP_GLOBAL_CONTROL,
                                     master_addr, PB_MASTER_SAP, fc, du, 2);
}

int pb_msg_build_dpv1_read_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                    uint8_t master_addr, bool fcb, bool fcv, uint8_t slot,
                                    uint8_t index, uint8_t length)
{
    uint8_t payload[4] = { PB_DPV1_FC_READ_REQ, slot, index, length };
    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_HI, fcb, fcv);
    return pb_fdl_build_sap_request(out, out_max, slave_addr, PB_SAP_DPV1_SLAVE_MSAC1, master_addr,
                                     PB_SAP_DPV1_MASTER_MSAC1, fc, payload, sizeof(payload));
}

int pb_msg_parse_dpv1_read_response(const pb_fdl_telegram_t *rx, uint8_t *data, int data_max,
                                     int *out_len)
{
    /* DU = [DSAP_echo, SSAP_echo, function_code, slot, index, length, data...] */
    if (rx->data_len < 6)
        return -1;

    uint8_t fc = rx->data[2];
    if (fc != PB_DPV1_FC_READ_RES)
        return -1;

    int len = rx->data[5];
    if (len > rx->data_len - 6)
        len = rx->data_len - 6;
    if (len > data_max)
        len = data_max;

    if (len > 0)
        memcpy(data, &rx->data[6], (size_t)len);

    *out_len = len;
    return 0;
}

int pb_msg_build_dpv1_write_request(uint8_t *out, int out_max, uint8_t slave_addr,
                                     uint8_t master_addr, bool fcb, bool fcv, uint8_t slot,
                                     uint8_t index, const uint8_t *data, int data_len)
{
    if (data_len < 0 || data_len > PB_MAX_IO_DATA_LEN - 4)
        return -1;

    uint8_t payload[4 + PB_MAX_IO_DATA_LEN];
    payload[0] = PB_DPV1_FC_WRITE_REQ;
    payload[1] = slot;
    payload[2] = index;
    payload[3] = (uint8_t)data_len;
    if (data_len > 0)
        memcpy(&payload[4], data, (size_t)data_len);

    uint8_t fc = pb_msg_request_fc(PB_FC_SRD_HI, fcb, fcv);
    return pb_fdl_build_sap_request(out, out_max, slave_addr, PB_SAP_DPV1_SLAVE_MSAC1, master_addr,
                                     PB_SAP_DPV1_MASTER_MSAC1, fc, payload, 4 + data_len);
}

int pb_msg_parse_dpv1_write_response(const pb_fdl_telegram_t *rx)
{
    /* DU = [DSAP_echo, SSAP_echo, function_code, slot, index, length] */
    if (rx->data_len < 6)
        return -1;

    if (rx->data[2] != PB_DPV1_FC_WRITE_RES)
        return -1;

    return 0;
}
