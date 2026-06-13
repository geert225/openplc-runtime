/**
 * @file profibus_master.c
 * @brief Profibus DP master instance -- per-slave parameterization state
 *        machine, cyclic Data_Exchange, and on-demand DPV1 parameter access
 */

#include "profibus_master.h"

#include "profibus_fdl.h"
#include "profibus_dp_messages.h"
#include "profibus_io.h"

#include <string.h>

/*
 * =============================================================================
 * Mutex Init Helper (PTHREAD_PRIO_INHERIT)
 * =============================================================================
 */

static int pb_mutex_init_pi(pthread_mutex_t *m)
{
    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0)
        return -1;
#if !defined(__CYGWIN__) && !defined(_WIN32)
    (void)pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
#endif
    int rc = pthread_mutex_init(m, &attr);
    pthread_mutexattr_destroy(&attr);
    return rc;
}

/*
 * =============================================================================
 * Lifecycle
 * =============================================================================
 */

int pb_master_init(pb_master_instance_t *inst, const char *name, plugin_logger_t *logger)
{
    memset(inst->name, 0, sizeof(inst->name));
    strncpy(inst->name, name, sizeof(inst->name) - 1);

    atomic_store(&inst->plugin_state, PB_STATE_IDLE);
    atomic_store(&inst->cycle_counter, 0);
    atomic_store(&inst->bus_running, false);
#if PB_ENABLE_MONITOR_THREAD
    atomic_store(&inst->monitor_running, false);
#endif

    if (pb_mutex_init_pi(&inst->bus_mutex) != 0) {
        plugin_logger_error(logger, "Master '%s': failed to initialize bus mutex", inst->name);
        return -1;
    }

    for (int i = 0; i < inst->config.slave_count; i++) {
        pb_slave_runtime_t *srt = &inst->slaves[i];
        memset(srt, 0, sizeof(*srt));

        srt->cfg = &inst->config.slaves[i];
        srt->fcb = false;
        srt->fcb_initialized = false;
        srt->retries = 0;
        srt->next_dpv1_write = (srt->cfg->dpv1_param_count > 0) ? 0 : -1;

        atomic_store(&srt->state, PB_SLAVE_OFFLINE);

        srt->status.station_address = srt->cfg->station_address;
        strncpy(srt->status.name, srt->cfg->name, PB_MAX_NAME_LEN - 1);
        atomic_store(&srt->status.state, PB_SLAVE_OFFLINE);
        atomic_store(&srt->status.diag_flags, 0);
        atomic_store(&srt->status.cycle_count, 0);
        atomic_store(&srt->status.error_count, 0);
        atomic_store(&srt->status.retry_count, 0);
    }

    return 0;
}

void pb_master_destroy(pb_master_instance_t *inst)
{
    pthread_mutex_destroy(&inst->bus_mutex);
}

int pb_master_open(pb_master_instance_t *inst, plugin_logger_t *logger)
{
    const pb_master_config_t *mc = &inst->config.master;

    return pb_serial_open(&inst->port, mc->device, mc->baudrate, mc->parity, mc->stop_bits,
                           mc->rs485_rts_control, mc->rts_delay_us, logger);
}

void pb_master_close(pb_master_instance_t *inst, plugin_logger_t *logger)
{
    (void)logger;
    pb_serial_close(&inst->port);
}

int pb_master_build_io(pb_master_instance_t *inst, plugin_runtime_args_t *args,
                        plugin_logger_t *logger)
{
    int errors = 0;

    for (int i = 0; i < inst->config.slave_count; i++) {
        pb_slave_runtime_t *srt = &inst->slaves[i];

        if (pb_io_build_channel_map(srt->cfg, &srt->channel_map, args, logger) != 0) {
            plugin_logger_error(logger, "Master '%s': slave '%s' channel map build failed",
                                 inst->name, srt->cfg->name);
            errors++;
            continue;
        }

        if (pb_io_build_transfer_list(&srt->channel_map, &srt->transfer_list, args, logger) !=
            0) {
            plugin_logger_error(logger, "Master '%s': slave '%s' transfer list build failed",
                                 inst->name, srt->cfg->name);
            errors++;
        }
    }

    return (errors > 0) ? -1 : 0;
}

/*
 * =============================================================================
 * Per-Slave State Machine Helpers
 * =============================================================================
 */

static inline void set_slave_state(pb_slave_runtime_t *srt, pb_slave_state_t state)
{
    atomic_store(&srt->state, (int)state);
    atomic_store(&srt->status.state, (int)state);
}

static inline void advance_fcb(pb_slave_runtime_t *srt)
{
    srt->fcb = !srt->fcb;
    srt->fcb_initialized = true;
}

/**
 * Slave_Diag (WAIT_DIAG1) failed -- the slave is simply not responding on
 * the bus yet. Silently restart from OFFLINE/WAIT_DIAG1 every cycle; no
 * retry counter, no FAULT escalation, so a slave plugged in later is picked
 * up automatically.
 */
static void slave_offline_retry(pb_slave_runtime_t *srt)
{
    atomic_fetch_add(&srt->status.error_count, 1);
    set_slave_state(srt, PB_SLAVE_OFFLINE);
    srt->fcb_initialized = false;
}

/**
 * Set_Prm/Chk_Cfg/WAIT_DIAG2 rejected by a responding slave -- a
 * configuration problem that retrying will not fix on its own. Restart
 * parameterization from OFFLINE up to master.max_retry_limit times; after
 * that, escalate to the terminal PB_SLAVE_FAULT state (logged once) so the
 * bus stops retrying and spamming warnings for this slave.
 */
static void slave_param_failed(pb_master_instance_t *inst, pb_slave_runtime_t *srt,
                                plugin_logger_t *logger)
{
    atomic_fetch_add(&srt->status.error_count, 1);
    atomic_fetch_add(&srt->status.retry_count, 1);
    srt->retries++;

    if (srt->retries >= inst->config.master.max_retry_limit) {
        plugin_logger_error(logger,
            "Slave '%s' (addr %d): parameterization failed after %d attempt(s), marking FAULT "
            "(no further automatic retries)",
            srt->cfg->name, srt->cfg->station_address, srt->retries);
        set_slave_state(srt, PB_SLAVE_FAULT);
        srt->fcb_initialized = false;
        srt->retries = 0;
        return;
    }

    set_slave_state(srt, PB_SLAVE_OFFLINE);
    srt->fcb_initialized = false;
}

/**
 * @brief Perform one FDL request/response transaction under bus_mutex.
 * @return true on success (rx populated), false on communication failure.
 */
static bool do_transaction(pb_master_instance_t *inst, const uint8_t *tx, int tx_len,
                            pb_fdl_telegram_t *rx)
{
    if (tx_len <= 0)
        return false;

    pthread_mutex_lock(&inst->bus_mutex);
    int rc = pb_fdl_transaction(&inst->port, tx, tx_len, rx, inst->config.master.slot_time_us,
                                 inst->config.master.max_retry_limit);
    pthread_mutex_unlock(&inst->bus_mutex);

    return rc == 0;
}

/*
 * =============================================================================
 * Per-Slave State Machine Step
 * =============================================================================
 */

static void pb_slave_step(pb_master_instance_t *inst, pb_slave_runtime_t *srt,
                           plugin_runtime_args_t *args, plugin_logger_t *logger)
{
    uint8_t addr = (uint8_t)srt->cfg->station_address;
    uint8_t master_addr = (uint8_t)inst->config.master.station_address;
    uint8_t tx[PB_FDL_MAX_TELEGRAM];
    pb_fdl_telegram_t rx;
    int tx_len;

    pb_slave_state_t state = (pb_slave_state_t)atomic_load(&srt->state);

    switch (state) {
    case PB_SLAVE_OFFLINE: {
        /* Not yet contacted -- move to WAIT_DIAG1, which performs the
         * initial Slave_Diag request on the next step. */
        set_slave_state(srt, PB_SLAVE_WAIT_DIAG1);
        break;
    }

    case PB_SLAVE_WAIT_DIAG1: {
        tx_len = pb_msg_build_slave_diag_request(tx, sizeof(tx), addr, master_addr, srt->fcb,
                                                  srt->fcb_initialized);
        if (do_transaction(inst, tx, tx_len, &rx)) {
            pb_diag_t diag;
            if (pb_msg_parse_slave_diag_response(&rx, &diag) == 0) {
                atomic_store(&srt->status.diag_flags, (uint32_t)diag.stat_1 |
                                                            ((uint32_t)diag.stat_2 << 8) |
                                                            ((uint32_t)diag.stat_3 << 16));
                advance_fcb(srt);
                set_slave_state(srt, PB_SLAVE_SET_PRM);
                plugin_logger_info(logger,
                    "Slave '%s' (addr %d): online (Stat_1=0x%02X), sending Set_Prm",
                    srt->cfg->name, srt->cfg->station_address, diag.stat_1);
                break;
            }
        }
        slave_offline_retry(srt);
        break;
    }

    case PB_SLAVE_SET_PRM: {
        tx_len =
            pb_msg_build_set_prm_request(tx, sizeof(tx), addr, master_addr, srt->fcb, true,
                                          srt->cfg);
        if (do_transaction(inst, tx, tx_len, &rx) && pb_msg_response_is_ok(&rx)) {
            advance_fcb(srt);
            set_slave_state(srt, PB_SLAVE_CHK_CFG);
            plugin_logger_debug(logger, "Slave '%s' (addr %d): Set_Prm accepted",
                                 srt->cfg->name, srt->cfg->station_address);
        } else {
            plugin_logger_warn(logger, "Slave '%s' (addr %d): Set_Prm failed/rejected",
                                srt->cfg->name, srt->cfg->station_address);
            slave_param_failed(inst, srt, logger);
        }
        break;
    }

    case PB_SLAVE_CHK_CFG: {
        if (srt->cfg->cfg_data_len <= 0) {
            /* No module configuration data provided -- skip Chk_Cfg. */
            advance_fcb(srt);
            set_slave_state(srt, PB_SLAVE_WAIT_DIAG2);
            break;
        }

        tx_len =
            pb_msg_build_chk_cfg_request(tx, sizeof(tx), addr, master_addr, srt->fcb, true,
                                          srt->cfg);
        if (do_transaction(inst, tx, tx_len, &rx) && pb_msg_response_is_ok(&rx)) {
            advance_fcb(srt);
            set_slave_state(srt, PB_SLAVE_WAIT_DIAG2);
            plugin_logger_debug(logger, "Slave '%s' (addr %d): Chk_Cfg accepted",
                                 srt->cfg->name, srt->cfg->station_address);
        } else {
            plugin_logger_warn(logger, "Slave '%s' (addr %d): Chk_Cfg failed/rejected",
                                srt->cfg->name, srt->cfg->station_address);
            slave_param_failed(inst, srt, logger);
        }
        break;
    }

    case PB_SLAVE_WAIT_DIAG2: {
        tx_len = pb_msg_build_slave_diag_request(tx, sizeof(tx), addr, master_addr, srt->fcb,
                                                  true);
        if (do_transaction(inst, tx, tx_len, &rx)) {
            pb_diag_t diag;
            if (pb_msg_parse_slave_diag_response(&rx, &diag) == 0) {
                atomic_store(&srt->status.diag_flags, (uint32_t)diag.stat_1 |
                                                            ((uint32_t)diag.stat_2 << 8) |
                                                            ((uint32_t)diag.stat_3 << 16));

                if (diag.stat_1 & (PB_STAT1_STATION_NOT_EXIST | PB_STAT1_STATION_NOT_READY |
                                    PB_STAT1_CFG_FAULT | PB_STAT1_PRM_FAULT)) {
                    plugin_logger_warn(logger,
                        "Slave '%s' (addr %d): not ready after Chk_Cfg (Stat_1=0x%02X), "
                        "re-parameterizing",
                        srt->cfg->name, srt->cfg->station_address, diag.stat_1);
                    slave_param_failed(inst, srt, logger);
                } else {
                    advance_fcb(srt);
                    set_slave_state(srt, PB_SLAVE_DATA_EXCHANGE);
                    srt->retries = 0;
                    plugin_logger_info(logger,
                        "Slave '%s' (addr %d): entering DATA_EXCHANGE",
                        srt->cfg->name, srt->cfg->station_address);
                }
                break;
            }
        }
        slave_offline_retry(srt);
        break;
    }

    case PB_SLAVE_DATA_EXCHANGE: {
        /* One pending DPV1 initial-value write per cycle, before the
         * cyclic Data_Exchange for this slave. */
        if (srt->next_dpv1_write >= 0 && srt->next_dpv1_write < srt->cfg->dpv1_param_count) {
            const pb_dpv1_param_t *p = &srt->cfg->dpv1_params[srt->next_dpv1_write];

            if (!p->has_initial_value || !p->writable) {
                srt->next_dpv1_write++;
            } else {
                uint8_t data[8];
                pb_value_to_bytes(p->initial_value, p->data_type, p->length, data);

                tx_len = pb_msg_build_dpv1_write_request(tx, sizeof(tx), addr, master_addr,
                                                          srt->fcb, true, p->slot, p->index,
                                                          data, p->length);
                if (do_transaction(inst, tx, tx_len, &rx) &&
                    pb_msg_parse_dpv1_write_response(&rx) == 0) {
                    advance_fcb(srt);
                    plugin_logger_info(logger,
                        "Slave '%s': DPV1 param '%s' (slot=%u idx=%u) initial write OK",
                        srt->cfg->name, p->name, p->slot, p->index);
                    srt->next_dpv1_write++;
                } else {
                    plugin_logger_warn(logger,
                        "Slave '%s': DPV1 param '%s' (slot=%u idx=%u) initial write failed, "
                        "retrying next cycle",
                        srt->cfg->name, p->name, p->slot, p->index);
                }
            }

            if (srt->next_dpv1_write >= srt->cfg->dpv1_param_count)
                srt->next_dpv1_write = -1;
        }

        /* Cyclic Data_Exchange */
        if (args->image_lock && args->image_unlock) {
            args->image_lock();
            pb_io_write_outputs_fast(&srt->transfer_list, srt->output_data);
            args->image_unlock();
        } else {
            pb_io_write_outputs_fast(&srt->transfer_list, srt->output_data);
        }

        tx_len = pb_msg_build_data_exchange_request(tx, sizeof(tx), addr, master_addr,
                                                      srt->fcb, true, srt->output_data,
                                                      srt->cfg->output_length);
        int in_len = 0;
        bool ok = false;
        if (tx_len > 0 && do_transaction(inst, tx, tx_len, &rx) &&
            pb_msg_parse_data_exchange_response(&rx, srt->input_data, sizeof(srt->input_data),
                                                  &in_len) == 0) {
            ok = true;
        }

        if (ok) {
            advance_fcb(srt);
            srt->retries = 0;
            atomic_fetch_add(&srt->status.cycle_count, 1);
            pb_io_read_inputs_fast(&srt->transfer_list, srt->input_data, args);
        } else {
            srt->retries++;
            atomic_fetch_add(&srt->status.error_count, 1);
            if (srt->retries >= inst->config.master.max_retry_limit) {
                plugin_logger_warn(logger,
                    "Slave '%s' (addr %d): lost communication after %d retries, "
                    "re-parameterizing",
                    srt->cfg->name, srt->cfg->station_address, srt->retries);
                atomic_fetch_add(&srt->status.retry_count, 1);
                set_slave_state(srt, PB_SLAVE_OFFLINE);
                srt->fcb_initialized = false;
                srt->retries = 0;
            }
        }
        break;
    }

    case PB_SLAVE_FAULT:
    default:
        /* Terminal -- no automatic recovery. */
        break;
    }
}

int pb_master_startup_slaves(pb_master_instance_t *inst, plugin_logger_t *logger)
{
    int rounds = inst->config.master.max_retry_limit * 4 + 10;

    for (int round = 0; round < rounds; round++) {
        bool all_done = true;

        for (int i = 0; i < inst->config.slave_count; i++) {
            pb_slave_runtime_t *srt = &inst->slaves[i];
            pb_slave_state_t state = (pb_slave_state_t)atomic_load(&srt->state);

            if (state != PB_SLAVE_DATA_EXCHANGE && state != PB_SLAVE_FAULT) {
                pb_slave_step(inst, srt, NULL, logger);
                all_done = false;
            }
        }

        if (all_done)
            break;
    }

    int failed = 0;
    for (int i = 0; i < inst->config.slave_count; i++) {
        pb_slave_runtime_t *srt = &inst->slaves[i];
        if (!srt->cfg->strict)
            continue;

        pb_slave_state_t state = (pb_slave_state_t)atomic_load(&srt->state);
        if (state != PB_SLAVE_DATA_EXCHANGE) {
            plugin_logger_error(logger,
                "Master '%s': slave '%s' (addr %d) is marked strict and failed to reach "
                "DATA_EXCHANGE during startup (state=%s)",
                inst->name, srt->cfg->name, srt->cfg->station_address,
                pb_slave_state_to_string(state));
            failed++;
        }
    }

    return (failed > 0) ? -1 : 0;
}

void pb_master_run_cycle(pb_master_instance_t *inst, plugin_runtime_args_t *args,
                          plugin_logger_t *logger)
{
    for (int i = 0; i < inst->config.slave_count; i++)
        pb_slave_step(inst, &inst->slaves[i], args, logger);

    atomic_fetch_add(&inst->cycle_counter, 1);
}

int pb_master_find_slave_by_address(const pb_master_instance_t *inst, int station_address)
{
    for (int i = 0; i < inst->config.slave_count; i++) {
        if (inst->config.slaves[i].station_address == station_address)
            return i;
    }
    return -1;
}

/*
 * =============================================================================
 * On-Demand DPV1 Access (execute_command)
 * =============================================================================
 */

int pb_master_dpv1_read(pb_master_instance_t *inst, int slave_index, uint8_t slot, uint8_t index,
                         uint8_t length, uint8_t *out_data, int *out_len,
                         plugin_logger_t *logger)
{
    if (slave_index < 0 || slave_index >= inst->config.slave_count)
        return -1;

    pb_slave_runtime_t *srt = &inst->slaves[slave_index];
    uint8_t addr = (uint8_t)srt->cfg->station_address;
    uint8_t master_addr = (uint8_t)inst->config.master.station_address;

    uint8_t tx[PB_FDL_MAX_TELEGRAM];
    int tx_len = pb_msg_build_dpv1_read_request(tx, sizeof(tx), addr, master_addr, srt->fcb,
                                                 srt->fcb_initialized, slot, index, length);
    if (tx_len <= 0)
        return -1;

    pthread_mutex_lock(&inst->bus_mutex);
    pb_fdl_telegram_t rx;
    int rc = pb_fdl_transaction(&inst->port, tx, tx_len, &rx, inst->config.master.slot_time_us,
                                 inst->config.master.max_retry_limit);
    if (rc == 0)
        advance_fcb(srt);
    pthread_mutex_unlock(&inst->bus_mutex);

    if (rc != 0) {
        plugin_logger_warn(logger,
            "Slave '%s' (addr %d): DPV1 read (slot=%u idx=%u) communication failed",
            srt->cfg->name, srt->cfg->station_address, slot, index);
        return -1;
    }

    /* out_data must have capacity for at least PB_MAX_IO_DATA_LEN bytes. */
    if (pb_msg_parse_dpv1_read_response(&rx, out_data, PB_MAX_IO_DATA_LEN, out_len) != 0) {
        plugin_logger_warn(logger,
            "Slave '%s' (addr %d): DPV1 read (slot=%u idx=%u) error response",
            srt->cfg->name, srt->cfg->station_address, slot, index);
        return -1;
    }

    return 0;
}

int pb_master_dpv1_write(pb_master_instance_t *inst, int slave_index, uint8_t slot,
                          uint8_t index, const uint8_t *data, int data_len,
                          plugin_logger_t *logger)
{
    if (slave_index < 0 || slave_index >= inst->config.slave_count)
        return -1;

    pb_slave_runtime_t *srt = &inst->slaves[slave_index];
    uint8_t addr = (uint8_t)srt->cfg->station_address;
    uint8_t master_addr = (uint8_t)inst->config.master.station_address;

    uint8_t tx[PB_FDL_MAX_TELEGRAM];
    int tx_len = pb_msg_build_dpv1_write_request(tx, sizeof(tx), addr, master_addr, srt->fcb,
                                                  srt->fcb_initialized, slot, index, data,
                                                  data_len);
    if (tx_len <= 0)
        return -1;

    pthread_mutex_lock(&inst->bus_mutex);
    pb_fdl_telegram_t rx;
    int rc = pb_fdl_transaction(&inst->port, tx, tx_len, &rx, inst->config.master.slot_time_us,
                                 inst->config.master.max_retry_limit);
    if (rc == 0)
        advance_fcb(srt);
    pthread_mutex_unlock(&inst->bus_mutex);

    if (rc != 0) {
        plugin_logger_warn(logger,
            "Slave '%s' (addr %d): DPV1 write (slot=%u idx=%u) communication failed",
            srt->cfg->name, srt->cfg->station_address, slot, index);
        return -1;
    }

    if (pb_msg_parse_dpv1_write_response(&rx) != 0) {
        plugin_logger_warn(logger,
            "Slave '%s' (addr %d): DPV1 write (slot=%u idx=%u) error response",
            srt->cfg->name, srt->cfg->station_address, slot, index);
        return -1;
    }

    return 0;
}

/*
 * =============================================================================
 * Value <-> Byte Encoding (DPV1 parameters)
 * =============================================================================
 */

int pb_value_to_bytes(double value, pb_data_type_t dt, uint8_t length, uint8_t *out)
{
    if (length == 0 || length > 8)
        return -1;

    uint8_t tmp[8] = { 0 };
    int sz = pb_data_type_size(dt);
    if (sz <= 0)
        sz = length;

    switch (dt) {
    case PB_DTYPE_BOOL:
    case PB_DTYPE_INT8: {
        int8_t v = (int8_t)value;
        memcpy(tmp, &v, 1);
        break;
    }
    case PB_DTYPE_UINT8: {
        uint8_t v = (uint8_t)value;
        memcpy(tmp, &v, 1);
        break;
    }
    case PB_DTYPE_INT16: {
        int16_t v = (int16_t)value;
        memcpy(tmp, &v, 2);
        break;
    }
    case PB_DTYPE_UINT16: {
        uint16_t v = (uint16_t)value;
        memcpy(tmp, &v, 2);
        break;
    }
    case PB_DTYPE_INT32: {
        int32_t v = (int32_t)value;
        memcpy(tmp, &v, 4);
        break;
    }
    case PB_DTYPE_UINT32: {
        uint32_t v = (uint32_t)value;
        memcpy(tmp, &v, 4);
        break;
    }
    case PB_DTYPE_INT64: {
        int64_t v = (int64_t)value;
        memcpy(tmp, &v, 8);
        break;
    }
    case PB_DTYPE_UINT64: {
        uint64_t v = (uint64_t)value;
        memcpy(tmp, &v, 8);
        break;
    }
    case PB_DTYPE_REAL32: {
        float v = (float)value;
        memcpy(tmp, &v, 4);
        break;
    }
    case PB_DTYPE_REAL64: {
        memcpy(tmp, &value, 8);
        break;
    }
    default:
        break;
    }

    memset(out, 0, length);
    int n = (sz < (int)length) ? sz : (int)length;
    memcpy(out, tmp, (size_t)n);
    return 0;
}

double pb_bytes_to_value(const uint8_t *data, pb_data_type_t dt, uint8_t length)
{
    uint8_t tmp[8] = { 0 };
    int sz = pb_data_type_size(dt);
    if (sz <= 0)
        sz = length;
    if (sz > 8)
        sz = 8;

    int n = (sz < (int)length) ? sz : (int)length;
    if (n > 8)
        n = 8;
    if (n > 0)
        memcpy(tmp, data, (size_t)n);

    switch (dt) {
    case PB_DTYPE_BOOL:
    case PB_DTYPE_INT8: {
        int8_t v;
        memcpy(&v, tmp, 1);
        return (double)v;
    }
    case PB_DTYPE_UINT8: {
        uint8_t v;
        memcpy(&v, tmp, 1);
        return (double)v;
    }
    case PB_DTYPE_INT16: {
        int16_t v;
        memcpy(&v, tmp, 2);
        return (double)v;
    }
    case PB_DTYPE_UINT16: {
        uint16_t v;
        memcpy(&v, tmp, 2);
        return (double)v;
    }
    case PB_DTYPE_INT32: {
        int32_t v;
        memcpy(&v, tmp, 4);
        return (double)v;
    }
    case PB_DTYPE_UINT32: {
        uint32_t v;
        memcpy(&v, tmp, 4);
        return (double)v;
    }
    case PB_DTYPE_INT64: {
        int64_t v;
        memcpy(&v, tmp, 8);
        return (double)v;
    }
    case PB_DTYPE_UINT64: {
        uint64_t v;
        memcpy(&v, tmp, 8);
        return (double)v;
    }
    case PB_DTYPE_REAL32: {
        float v;
        memcpy(&v, tmp, 4);
        return (double)v;
    }
    case PB_DTYPE_REAL64: {
        double v;
        memcpy(&v, tmp, 8);
        return v;
    }
    default:
        return 0.0;
    }
}
