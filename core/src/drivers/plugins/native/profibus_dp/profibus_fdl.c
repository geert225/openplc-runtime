/**
 * @file profibus_fdl.c
 * @brief Profibus FDL telegram framing and transaction implementation
 */

#include "profibus_fdl.h"

#include <string.h>

static uint8_t fcs_sum(const uint8_t *buf, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
        sum += buf[i];
    return sum;
}

int pb_fdl_build_sd1(uint8_t *out, int out_max, uint8_t da, uint8_t sa, uint8_t fc)
{
    if (out_max < 6)
        return -1;

    out[0] = PB_SD1;
    out[1] = da;
    out[2] = sa;
    out[3] = fc;
    out[4] = fcs_sum(&out[1], 3);
    out[5] = PB_ED;
    return 6;
}

int pb_fdl_build_sd2(uint8_t *out, int out_max, uint8_t da, uint8_t sa, uint8_t fc,
                      const uint8_t *data, int data_len)
{
    if (data_len < 1 || data_len > PB_MAX_IO_DATA_LEN + 2)
        return -1;

    int total = 9 + data_len;
    if (out_max < total)
        return -1;

    uint8_t le = (uint8_t)(3 + data_len);

    out[0] = PB_SD2;
    out[1] = le;
    out[2] = le;
    out[3] = PB_SD2;
    out[4] = da;
    out[5] = sa;
    out[6] = fc;
    memcpy(&out[7], data, (size_t)data_len);
    out[7 + data_len] = fcs_sum(&out[4], 3 + data_len);
    out[8 + data_len] = PB_ED;
    return total;
}

int pb_fdl_build_sap_request(uint8_t *out, int out_max, uint8_t da_addr, uint8_t da_sap,
                              uint8_t sa_addr, uint8_t sa_sap, uint8_t fc,
                              const uint8_t *data, int data_len)
{
    if (data_len < 0 || data_len > PB_MAX_IO_DATA_LEN)
        return -1;

    uint8_t du[PB_MAX_IO_DATA_LEN + 2];
    du[0] = da_sap;
    du[1] = sa_sap;
    if (data_len > 0)
        memcpy(&du[2], data, (size_t)data_len);

    return pb_fdl_build_sd2(out, out_max, (uint8_t)(da_addr | 0x80), (uint8_t)(sa_addr | 0x80), fc,
                             du, data_len + 2);
}

int pb_fdl_build_data_request(uint8_t *out, int out_max, uint8_t slave_addr,
                               uint8_t master_addr, uint8_t fc, const uint8_t *data,
                               int data_len)
{
    uint8_t da = (uint8_t)(slave_addr & 0x7F);
    uint8_t sa = (uint8_t)(master_addr & 0x7F);

    if (data_len <= 0)
        return pb_fdl_build_sd1(out, out_max, da, sa, fc);

    return pb_fdl_build_sd2(out, out_max, da, sa, fc, data, data_len);
}

int pb_fdl_parse(const uint8_t *buf, int len, pb_fdl_telegram_t *out)
{
    if (len < 1)
        return 0;

    if (buf[0] == PB_SC) {
        out->type = PB_TEL_SC;
        out->da = 0;
        out->sa = 0;
        out->fc = 0;
        out->data_len = 0;
        return 1;
    }

    if (buf[0] == PB_SD1) {
        if (len < 6)
            return 0;
        if (buf[5] != PB_ED)
            return -1;
        if (fcs_sum(&buf[1], 3) != buf[4])
            return -1;

        out->type = PB_TEL_SD1;
        out->da = buf[1];
        out->sa = buf[2];
        out->fc = buf[3];
        out->data_len = 0;
        return 6;
    }

    if (buf[0] == PB_SD3) {
        if (len < 14)
            return 0;
        if (buf[13] != PB_ED)
            return -1;
        if (fcs_sum(&buf[1], 11) != buf[12])
            return -1;

        out->type = PB_TEL_SD3;
        out->da = buf[1];
        out->sa = buf[2];
        out->fc = buf[3];
        out->data_len = 8;
        memcpy(out->data, &buf[4], 8);
        return 14;
    }

    if (buf[0] == PB_SD4) {
        if (len < 3)
            return 0;

        out->type = PB_TEL_SD4;
        out->da = buf[1];
        out->sa = buf[2];
        out->fc = 0;
        out->data_len = 0;
        return 3;
    }

    if (buf[0] == PB_SD2) {
        if (len < 4)
            return 0;

        uint8_t le = buf[1];
        uint8_t ler = buf[2];
        if (buf[3] != PB_SD2 || le != ler || le < 3)
            return -1;

        int n = le - 3;
        if (n > (int)sizeof(out->data))
            return -1;

        int total = 9 + n;
        if (len < total)
            return 0;
        if (buf[total - 1] != PB_ED)
            return -1;
        if (fcs_sum(&buf[4], 3 + n) != buf[total - 2])
            return -1;

        out->type = PB_TEL_SD2;
        out->da = buf[4];
        out->sa = buf[5];
        out->fc = buf[6];
        out->data_len = n;
        memcpy(out->data, &buf[7], (size_t)n);
        return total;
    }

    /* Unrecognized start delimiter -- caller should skip a byte to resync. */
    return -1;
}

int pb_fdl_transaction(pb_serial_port_t *port, const uint8_t *tx_buf, int tx_len,
                        pb_fdl_telegram_t *rx, int slot_time_us, int max_retries)
{
    uint8_t rxbuf[PB_FDL_MAX_TELEGRAM];

    for (int attempt = 0; attempt <= max_retries; attempt++) {
        pb_serial_flush(port);

        if (pb_serial_write(port, tx_buf, tx_len) != tx_len)
            continue;

        int n = pb_serial_read(port, rxbuf, sizeof(rxbuf), slot_time_us);
        if (n <= 0)
            continue; /* timeout, no response -- retry */

        int offset = 0;
        for (;;) {
            int consumed = pb_fdl_parse(rxbuf + offset, n - offset, rx);
            if (consumed > 0)
                return 0;

            if (consumed == 0) {
                /* Incomplete telegram -- try to read more bytes within the
                 * remaining slot time budget. */
                if ((size_t)n >= sizeof(rxbuf))
                    break;
                int more =
                    pb_serial_read(port, rxbuf + n, sizeof(rxbuf) - (size_t)n, slot_time_us);
                if (more <= 0)
                    break;
                n += more;
                continue;
            }

            /* consumed < 0: invalid/unknown delimiter -- skip one byte and
             * try to resync within the same response. */
            offset++;
            if (offset >= n)
                break;
        }
        /* Fall through to the next attempt. */
    }

    return -1;
}
