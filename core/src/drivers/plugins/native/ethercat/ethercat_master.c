/**
 * @file ethercat_master.c
 * @brief EtherCAT Master SOEM Wrapper Implementation
 *
 * Wraps the SOEM library to provide high-level EtherCAT master operations:
 * network initialization, slave scanning, topology validation against
 * the JSON configuration, SDO writes, state machine management, and
 * slave recovery.
 *
 * Uses the ecx_* context-based API from SOEM 2.x.
 */

#include "ethercat_master.h"
#include "ethercat_proc.h"
#include "ethercat_iface_state.h"
#include "soem/soem.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Low-latency socket option for the SOEM raw socket (Linux only).  All
 * NIC tuning (ethtool coalescing/offloads) and IP-stack isolation
 * (iptables, IPv6 sysctl) lives in ethercat_iface_state.{c,h}. */
#if !defined(__CYGWIN__) && !defined(_WIN32)
#include <sys/socket.h>
#define ECAT_BUSY_POLL_US 50
#endif

/*
 * =============================================================================
 * SOEM Context and IO Map
 * =============================================================================
 */

/** Number of retries when polling for OPERATIONAL state */
#define ECAT_OP_POLL_RETRIES 10

/*
 * =============================================================================
 * Topology Validation
 * =============================================================================
 */

int ecat_master_validate_topology(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    const ecat_config_t *config = &inst->config;
    int found_count = inst->ecx_context.slavecount;

    if (found_count != config->slave_count) {
        plugin_logger_error(logger,
            "Topology mismatch: expected %d slaves, found %d on the bus",
            config->slave_count, found_count);
        return -1;
    }

    for (int i = 0; i < config->slave_count; i++) {
        const ecat_slave_t *expected = &config->slaves[i];
        int pos = expected->position;

        if (pos < 1 || pos > found_count) {
            plugin_logger_error(logger,
                "Slave %d: position %d is out of range (1-%d)",
                i, pos, found_count);
            return -1;
        }

        ec_slavet *found = &inst->ecx_context.slavelist[pos];

        /* Vendor ID check (can be disabled per slave via startup_checks) */
        if (expected->startup_checks.check_vendor_id) {
            if (found->eep_man != expected->vendor_id) {
                plugin_logger_error(logger,
                    "Slave %d (%s) at position %d: vendor_id mismatch - "
                    "expected 0x%08X, found 0x%08X",
                    i, expected->name, pos,
                    expected->vendor_id, found->eep_man);
                return -1;
            }
        } else {
            plugin_logger_debug(logger,
                "Slave %d (%s) at position %d: vendor_id check disabled",
                i, expected->name, pos);
        }

        /* Product code check (can be disabled per slave via startup_checks) */
        if (expected->startup_checks.check_product_code) {
            if (found->eep_id != expected->product_code) {
                plugin_logger_error(logger,
                    "Slave %d (%s) at position %d: product_code mismatch - "
                    "expected 0x%08X, found 0x%08X",
                    i, expected->name, pos,
                    expected->product_code, found->eep_id);
                return -1;
            }
        } else {
            plugin_logger_debug(logger,
                "Slave %d (%s) at position %d: product_code check disabled",
                i, expected->name, pos);
        }

        plugin_logger_debug(logger,
            "Slave %d (%s) at position %d: topology OK "
            "(vendor=0x%08X, product=0x%08X)",
            i, expected->name, pos,
            found->eep_man, found->eep_id);
    }

    plugin_logger_info(logger, "Topology validation passed: %d slaves match configuration",
                       config->slave_count);
    return 0;
}

/*
 * =============================================================================
 * Phase 1: Open Interface, Scan Bus, Validate Topology
 * =============================================================================
 */

int ecat_master_open_and_scan(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    const ecat_config_t *config = &inst->config;

    /* Zero-initialize the SOEM context before use */
    memset(&inst->ecx_context, 0, sizeof(inst->ecx_context));
    memset(inst->iomap, 0, sizeof(inst->iomap));
    inst->iomap_used_size = 0;

    /* Step 0: Apply per-iface external state (NIC tuning + IP-stack
     * isolation).  Reverted in ecat_master_close. */
    ecat_iface_state_apply(&inst->iface_state, config->master.interface, logger);

    /* Step 1: Initialize SOEM on the configured network interface */
    plugin_logger_info(logger, "Opening network interface: %s", config->master.interface);

    if (!ecx_init(&inst->ecx_context, config->master.interface)) {
#if defined(__CYGWIN__) || defined(_WIN32)
        plugin_logger_error(logger,
            "Failed to initialize EtherCAT interface '%s'. "
            "Verify that Npcap (https://npcap.com) is installed and "
            "the interface name matches a network adapter (use "
            "'ipconfig' or Npcap's WlanHelper to list adapters).",
            config->master.interface);
#else
        plugin_logger_error(logger,
            "Failed to initialize EtherCAT interface '%s'. "
            "Check that the interface exists and the process has "
            "CAP_NET_RAW capability (or is running as root).",
            config->master.interface);
#endif
        return -1;
    }

    inst->soem_initialized = 1;
    plugin_logger_info(logger, "Network interface opened successfully");

    /* Enable SO_BUSY_POLL on the SOEM raw socket.
     * This makes recvfrom() spin-poll the NIC driver instead of sleeping,
     * eliminating ~5-10us of scheduler wakeup latency per exchange. */
#ifdef ECAT_BUSY_POLL_US
    {
        int busy_us = ECAT_BUSY_POLL_US;
        int sockfd = inst->ecx_context.port.sockhandle;
        if (sockfd >= 0) {
            if (setsockopt(sockfd, SOL_SOCKET, SO_BUSY_POLL,
                           &busy_us, sizeof(busy_us)) == 0) {
                plugin_logger_info(logger,
                    "SO_BUSY_POLL enabled on socket (poll=%d us)", busy_us);
            } else {
                plugin_logger_debug(logger,
                    "SO_BUSY_POLL not supported (kernel may need CONFIG_NET_RX_BUSY_POLL)");
            }
        }
    }
#endif

    /* Step 2: Scan the bus and enumerate slaves */
    plugin_logger_info(logger, "Scanning EtherCAT bus...");

    if (ecx_config_init(&inst->ecx_context) <= 0) {
        plugin_logger_error(logger,
            "No EtherCAT slaves found on interface '%s'. "
            "Check cable connections and slave power.",
            config->master.interface);
        ecx_close(&inst->ecx_context);
        inst->soem_initialized = 0;
        return -1;
    }

    plugin_logger_info(logger, "Found %d slave(s) on the bus", inst->ecx_context.slavecount);

    /* Log discovered slaves */
    for (int i = 1; i <= inst->ecx_context.slavecount; i++) {
        ec_slavet *slave = &inst->ecx_context.slavelist[i];
        plugin_logger_info(logger,
            "  [%d] %s - vendor=0x%08X, product=0x%08X, rev=0x%08X",
            i, slave->name, slave->eep_man, slave->eep_id, slave->eep_rev);
    }

    /* Step 3: Validate topology against JSON configuration */
    if (ecat_master_validate_topology(inst, logger) != 0) {
        plugin_logger_error(logger,
            "Topology validation failed - aborting master initialization");
        ecx_close(&inst->ecx_context);
        inst->soem_initialized = 0;
        return -1;
    }

    /* Step 4: Wait for all slaves to reach PRE-OP state.
     * ecx_config_init() requests PRE-OP but does not wait for the
     * transition to complete. Slaves need to be in PRE-OP before
     * mailbox communication (SDO writes) can work.
     *
     * Use the maximum init_to_preop_timeout across all configured slaves. */
    plugin_logger_info(logger, "Waiting for slaves to reach PRE-OP state...");

    int max_preop_timeout_us = 0;
    for (int i = 0; i < config->slave_count; i++) {
        int t_us = config->slaves[i].timeouts.init_to_preop_timeout_ms * 1000;
        if (t_us > max_preop_timeout_us)
            max_preop_timeout_us = t_us;
    }
    if (max_preop_timeout_us == 0)
        max_preop_timeout_us = EC_TIMEOUTSTATE * 4;
    plugin_logger_debug(logger, "Using INIT->PRE-OP timeout: %d us", max_preop_timeout_us);

    ecx_statecheck(&inst->ecx_context, 0, EC_STATE_PRE_OP, max_preop_timeout_us);
    ecx_readstate(&inst->ecx_context);

    int all_preop = 1;
    for (int i = 1; i <= inst->ecx_context.slavecount; i++) {
        ec_slavet *slave = &inst->ecx_context.slavelist[i];
        if (slave->state < EC_STATE_PRE_OP) {
            plugin_logger_error(logger,
                "Slave %d (%s) failed to reach PRE-OP (state=0x%04X, ALstatus=0x%04X)",
                i, slave->name, slave->state, slave->ALstatuscode);
            all_preop = 0;
        }
    }

    if (!all_preop) {
        plugin_logger_error(logger, "Not all slaves reached PRE-OP - aborting");
        ecx_close(&inst->ecx_context);
        inst->soem_initialized = 0;
        return -1;
    }

    plugin_logger_info(logger, "All slaves in PRE-OP state");

    return 0;
}

/*
 * =============================================================================
 * Phase 2: SDO Configuration
 * =============================================================================
 */

int ecat_master_write_sdos(ecat_master_instance_t *inst, int slave_pos,
                           const ecat_sdo_config_t *sdos,
                           int sdo_count, int sdo_timeout_ms,
                           plugin_logger_t *logger)
{
    if (!inst->soem_initialized) {
        plugin_logger_error(logger, "Cannot write SDOs: SOEM not initialized");
        return -1;
    }

    if (slave_pos < 1 || slave_pos > inst->ecx_context.slavecount) {
        plugin_logger_error(logger, "Invalid slave position %d for SDO write", slave_pos);
        return -1;
    }

    if (sdo_count == 0)
        return 0;

    int written = 0;

    for (int i = 0; i < sdo_count; i++) {
        const ecat_sdo_config_t *sdo = &sdos[i];

        /* Parse index from hex string */
        uint16_t index = (uint16_t)strtol(sdo->index, NULL, 16);

        /* Determine data size from data type.  parse_sdo rejects UNKNOWN/PAD
         * so this branch is defensive -- if it triggers, the parser regressed. */
        ecat_data_type_t dt = sdo->parsed_type;
        int size = ecat_data_type_size(dt);
        if (size <= 0) {
            plugin_logger_error(logger,
                "Slave %d SDO 0x%04X:%d: unknown data type '%s' -- skipping (parser regression?)",
                slave_pos, index, sdo->subindex, sdo->data_type);
            continue;
        }

        /* Encode the double value into the correct wire type */
        uint8_t value_buf[8];
        memset(value_buf, 0, sizeof(value_buf));

        switch (dt) {
        case ECAT_DTYPE_BOOL:
        case ECAT_DTYPE_UINT8:  { uint8_t  v = (uint8_t)sdo->value;  memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_INT8:   { int8_t   v = (int8_t)sdo->value;   memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_UINT16: { uint16_t v = (uint16_t)sdo->value; memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_INT16:  { int16_t  v = (int16_t)sdo->value;  memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_UINT32: { uint32_t v = (uint32_t)sdo->value; memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_INT32:  { int32_t  v = (int32_t)sdo->value;  memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_UINT64: { uint64_t v = (uint64_t)sdo->value; memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_INT64:  { int64_t  v = (int64_t)sdo->value;  memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_REAL32: { float    v = (float)sdo->value;    memcpy(value_buf, &v, sizeof(v)); break; }
        case ECAT_DTYPE_REAL64: { double   v = sdo->value;           memcpy(value_buf, &v, sizeof(v)); break; }
        default:                { int32_t  v = (int32_t)sdo->value;  memcpy(value_buf, &v, sizeof(v)); break; }
        }

        if (dt == ECAT_DTYPE_REAL32 || dt == ECAT_DTYPE_REAL64) {
            plugin_logger_debug(logger,
                "Slave %d: writing SDO 0x%04X:%d = %g (%s, %d bytes)",
                slave_pos, index, sdo->subindex, sdo->value, sdo->data_type, size);
        } else {
            plugin_logger_debug(logger,
                "Slave %d: writing SDO 0x%04X:%d = %lld (%s, %d bytes)",
                slave_pos, index, sdo->subindex, (long long)(int64_t)sdo->value,
                sdo->data_type, size);
        }

        /* Use per-slave SDO timeout if configured, otherwise SOEM default */
        int sdo_timeout_us = (sdo_timeout_ms > 0) ? (sdo_timeout_ms * 1000) : EC_TIMEOUTRXM;

        int wkc = ecx_SDOwrite(&inst->ecx_context, (uint16)slave_pos,
                                index, sdo->subindex,
                                FALSE, size, value_buf, sdo_timeout_us);

        if (wkc <= 0) {
            plugin_logger_warn(logger,
                "Slave %d SDO 0x%04X:%d write failed (wkc=%d, name='%s')",
                slave_pos, index, sdo->subindex, wkc, sdo->name);
        } else {
            plugin_logger_debug(logger,
                "Slave %d SDO 0x%04X:%d write OK (name='%s')",
                slave_pos, index, sdo->subindex, sdo->name);
            written++;
        }
    }

    plugin_logger_info(logger, "Slave %d: %d/%d SDOs written successfully",
                       slave_pos, written, sdo_count);
    return written;
}

/*
 * =============================================================================
 * Phase 3: Process Data Mapping + Distributed Clocks
 * =============================================================================
 */

/**
 * @brief Configure watchdog timers for a single slave via register writes.
 *
 * EtherCAT watchdog registers (addressed by configured address):
 *   0x0400 (2 bytes) - Watchdog divider (default 0x09C2 = 2498)
 *   0x0402 (2 bytes) - PDI watchdog time (in watchdog divider ticks)
 *   0x0420 (2 bytes) - SM watchdog time  (in watchdog divider ticks)
 *
 * Default divider 0x09C2 = 2498 -> (2498+2)*25ns = 62.5us per tick.
 * To set watchdog to X ms: ticks = X * 1000 / 62.5 = X * 16
 *
 * @param slave_pos 1-based slave position on the bus
 * @param wd        Watchdog configuration
 * @param logger    Plugin logger instance
 */
static void ecat_master_configure_watchdog(ecat_master_instance_t *inst, int slave_pos,
                                           const ecat_watchdog_t *wd,
                                           plugin_logger_t *logger)
{
    /* Maximum watchdog timeout in ms that fits in a uint16_t register
     * with the default divider (1 tick = 62.5 us -> ticks = ms * 16).
     * 65535 / 16 = 4095.9 ms */
    const int max_watchdog_ms = UINT16_MAX / 16;

    uint16_t configadr = inst->ecx_context.slavelist[slave_pos].configadr;
    int wkc;

    /* SM watchdog register 0x0420 - only write if explicitly enabled. */
    if (wd->sm_watchdog_enabled) {
        uint16_t sm_wd_ticks = 0;
        if (wd->sm_watchdog_ms > 0) {
            int clamped_ms = wd->sm_watchdog_ms;
            if (clamped_ms > max_watchdog_ms) {
                plugin_logger_warn(logger,
                    "Slave %d: SM watchdog %d ms exceeds max %d ms, clamping",
                    slave_pos, wd->sm_watchdog_ms, max_watchdog_ms);
                clamped_ms = max_watchdog_ms;
            }
            sm_wd_ticks = (uint16_t)(clamped_ms * 16);
        }
        wkc = ecx_FPWR(&inst->ecx_context.port, configadr, 0x0420,
                            sizeof(sm_wd_ticks), &sm_wd_ticks, EC_TIMEOUTRET);
        if (wkc <= 0) {
            plugin_logger_warn(logger,
                "Slave %d: failed to write SM watchdog register 0x0420 (wkc=%d)",
                slave_pos, wkc);
        } else {
            plugin_logger_debug(logger,
                "Slave %d: SM watchdog enabled (ticks=%u, ~%d ms)",
                slave_pos, sm_wd_ticks, wd->sm_watchdog_ms);
        }
    } else {
        plugin_logger_debug(logger,
            "Slave %d: SM watchdog disabled, skipping register write",
            slave_pos);
    }

    /* PDI watchdog register 0x0402 - only write if explicitly enabled,
     * as many slaves do not support PDI watchdog and return wkc=0. */
    if (wd->pdi_watchdog_enabled) {
        uint16_t pdi_wd_ticks = 0;
        if (wd->pdi_watchdog_ms > 0) {
            int clamped_ms = wd->pdi_watchdog_ms;
            if (clamped_ms > max_watchdog_ms) {
                plugin_logger_warn(logger,
                    "Slave %d: PDI watchdog %d ms exceeds max %d ms, clamping",
                    slave_pos, wd->pdi_watchdog_ms, max_watchdog_ms);
                clamped_ms = max_watchdog_ms;
            }
            pdi_wd_ticks = (uint16_t)(clamped_ms * 16);
        }
        wkc = ecx_FPWR(&inst->ecx_context.port, configadr, 0x0402,
                        sizeof(pdi_wd_ticks), &pdi_wd_ticks, EC_TIMEOUTRET);
        if (wkc <= 0) {
            plugin_logger_warn(logger,
                "Slave %d: failed to write PDI watchdog register 0x0402 (wkc=%d)",
                slave_pos, wkc);
        } else {
            plugin_logger_debug(logger,
                "Slave %d: PDI watchdog enabled (ticks=%u, ~%d ms)",
                slave_pos, pdi_wd_ticks, wd->pdi_watchdog_ms);
        }
    } else {
        plugin_logger_debug(logger,
            "Slave %d: PDI watchdog disabled, skipping register write",
            slave_pos);
    }
}

/**
 * @brief Configure Distributed Clocks per slave based on JSON configuration.
 *
 * First calls ecx_configdc() to discover DC-capable slaves and measure
 * propagation delays.  Then for each slave with dc.enabled, configures
 * SYNC0 and/or SYNC1 signals.
 *
 * @param config Parsed EtherCAT configuration
 * @param logger Plugin logger instance
 */
static void ecat_master_configure_dc(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    const ecat_config_t *config = &inst->config;

    /* Step 1: Let SOEM discover DC-capable slaves and measure delays */
    plugin_logger_info(logger, "Configuring Distributed Clocks...");
    ecx_configdc(&inst->ecx_context);

    /* Step 2: Apply per-slave DC configuration */
    for (int i = 0; i < config->slave_count; i++) {
        const ecat_slave_t *slave = &config->slaves[i];
        int pos = slave->position;

        if (!slave->dc.enabled)
            continue;

        if (pos < 1 || pos > inst->ecx_context.slavecount) {
            plugin_logger_warn(logger,
                "Slave %d (%s): DC config skipped - position out of range",
                pos, slave->name);
            continue;
        }

        if (!inst->ecx_context.slavelist[pos].hasdc) {
            plugin_logger_warn(logger,
                "Slave %d (%s): DC config requested but slave has no DC support",
                pos, slave->name);
            continue;
        }

        /* Determine cycle time: use slave-specific or fall back to master cycle */
        uint32_t cycle_ns;
        if (slave->dc.sync_unit_cycle_us > 0) {
            cycle_ns = (uint32_t)(slave->dc.sync_unit_cycle_us * 1000);
        } else {
            cycle_ns = (uint32_t)(config->master.cycle_time_us * 1000);
        }

        if (slave->dc.sync0_enabled && slave->dc.sync1_enabled) {
            /* Both SYNC0 and SYNC1 */
            uint32_t cycle0_ns = (slave->dc.sync0_cycle_us > 0)
                ? (uint32_t)(slave->dc.sync0_cycle_us * 1000) : cycle_ns;
            uint32_t cycle1_ns = (slave->dc.sync1_cycle_us > 0)
                ? (uint32_t)(slave->dc.sync1_cycle_us * 1000) : cycle_ns;
            int32_t shift_ns = (int32_t)(slave->dc.sync0_shift_us * 1000);

            ecx_dcsync01(&inst->ecx_context, (uint16)pos, TRUE,
                         cycle0_ns, cycle1_ns, shift_ns);

            plugin_logger_info(logger,
                "Slave %d (%s): DC SYNC0+SYNC1 enabled "
                "(cycle0=%u ns, cycle1=%u ns, shift=%d ns)",
                pos, slave->name, cycle0_ns, cycle1_ns, shift_ns);

        } else if (slave->dc.sync0_enabled) {
            /* SYNC0 only */
            uint32_t sync0_ns = (slave->dc.sync0_cycle_us > 0)
                ? (uint32_t)(slave->dc.sync0_cycle_us * 1000) : cycle_ns;
            int32_t shift_ns = (int32_t)(slave->dc.sync0_shift_us * 1000);

            ecx_dcsync0(&inst->ecx_context, (uint16)pos, TRUE,
                        sync0_ns, shift_ns);

            plugin_logger_info(logger,
                "Slave %d (%s): DC SYNC0 enabled (cycle=%u ns, shift=%d ns)",
                pos, slave->name, sync0_ns, shift_ns);

        } else {
            /* DC enabled but no SYNC signals - just log it */
            plugin_logger_debug(logger,
                "Slave %d (%s): DC enabled but no SYNC signals configured",
                pos, slave->name);
        }
    }
}

int ecat_master_configure(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    const ecat_config_t *config = &inst->config;

    if (!inst->soem_initialized) {
        plugin_logger_error(logger, "Cannot configure: SOEM not initialized");
        return -1;
    }

    /* Step 4: Map process data (IO map) */
    plugin_logger_info(logger, "Mapping process data...");

    ecx_config_map_group(&inst->ecx_context, &inst->iomap, 0);

    ec_groupt *grp = &inst->ecx_context.grouplist[0];

    /* Check that total I/O fits in the IOmap buffer */
    uint32_t total_io = (uint32_t)grp->Obytes + (uint32_t)grp->Ibytes;
    if (total_io > ECAT_IOMAP_SIZE) {
        plugin_logger_error(logger, "IOmap overflow: need %u bytes, have %d",
                            total_io, ECAT_IOMAP_SIZE);
        return -1;
    }

    inst->iomap_used_size = (size_t)total_io;

    plugin_logger_info(logger, "IO map: %d output bytes, %d input bytes, %d segments",
                       grp->Obytes, grp->Ibytes, grp->nsegments);

    /* Step 5: Configure watchdogs per slave */
    plugin_logger_info(logger, "Configuring per-slave watchdogs...");
    for (int i = 0; i < config->slave_count; i++) {
        const ecat_slave_t *slave = &config->slaves[i];
        int pos = slave->position;
        if (pos >= 1 && pos <= inst->ecx_context.slavecount) {
            ecat_master_configure_watchdog(inst, pos, &slave->watchdog, logger);
        }
    }

    /* Step 6: Configure Distributed Clocks per slave */
    ecat_master_configure_dc(inst, logger);

    return 0;
}

/*
 * =============================================================================
 * Phase 4: Transition to OPERATIONAL
 * =============================================================================
 */

int ecat_master_transition_to_op(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    const ecat_config_t *config = &inst->config;

    if (!inst->soem_initialized) {
        plugin_logger_error(logger, "Cannot transition: SOEM not initialized");
        return -1;
    }

    /* Compute maximum SAFE-OP->OP timeout across all configured slaves */
    int max_safeop_timeout_us = 0;
    for (int i = 0; i < config->slave_count; i++) {
        int t_us = config->slaves[i].timeouts.safeop_to_op_timeout_ms * 1000;
        if (t_us > max_safeop_timeout_us)
            max_safeop_timeout_us = t_us;
    }
    if (max_safeop_timeout_us == 0)
        max_safeop_timeout_us = EC_TIMEOUTSTATE * 4;

    /* Step 6: Wait for SAFE_OP after config */
    plugin_logger_info(logger, "Waiting for SAFE_OP state...");
    plugin_logger_debug(logger, "Using SAFE-OP->OP timeout: %d us", max_safeop_timeout_us);

    ecx_statecheck(&inst->ecx_context, 0, EC_STATE_SAFE_OP, max_safeop_timeout_us);

    /* Read back actual states */
    ecx_readstate(&inst->ecx_context);
    if (inst->ecx_context.slavelist[0].state != EC_STATE_SAFE_OP) {
        plugin_logger_error(logger,
            "Not all slaves reached SAFE_OP state (current state: 0x%04X)",
            inst->ecx_context.slavelist[0].state);

        /* Log individual slave states for debugging */
        for (int i = 1; i <= inst->ecx_context.slavecount; i++) {
            ec_slavet *slave = &inst->ecx_context.slavelist[i];
            if (slave->state != EC_STATE_SAFE_OP) {
                plugin_logger_error(logger,
                    "  Slave %d (%s): state=0x%04X, ALstatuscode=0x%04X",
                    i, slave->name, slave->state, slave->ALstatuscode);
            }
        }

        return -1;
    }

    plugin_logger_info(logger, "All slaves in SAFE_OP state");

    /* Step 7: Send initial process data and request OPERATIONAL */
    plugin_logger_info(logger, "Requesting OPERATIONAL state...");

    /* Send one round of process data to make slave outputs happy */
    ecx_send_processdata(&inst->ecx_context);
    ecx_receive_processdata(&inst->ecx_context, EC_TIMEOUTRET);

    /* Request OP state */
    inst->ecx_context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&inst->ecx_context, 0);

    /* Poll for OP state with process data exchange between checks */
    int op_reached = 0;
    int poll_timeout_us = max_safeop_timeout_us / ECAT_OP_POLL_RETRIES;
    if (poll_timeout_us < EC_TIMEOUTRET)
        poll_timeout_us = EC_TIMEOUTRET;

    for (int retry = 0; retry < ECAT_OP_POLL_RETRIES; retry++) {
        ecx_send_processdata(&inst->ecx_context);
        ecx_receive_processdata(&inst->ecx_context, EC_TIMEOUTRET);
        ecx_statecheck(&inst->ecx_context, 0, EC_STATE_OPERATIONAL, poll_timeout_us);

        if (inst->ecx_context.slavelist[0].state == EC_STATE_OPERATIONAL) {
            op_reached = 1;
            break;
        }
    }

    if (!op_reached) {
        plugin_logger_error(logger,
            "Not all slaves reached OPERATIONAL state after %d retries",
            ECAT_OP_POLL_RETRIES);

        /* Log individual slave states for debugging */
        ecx_readstate(&inst->ecx_context);
        for (int i = 1; i <= inst->ecx_context.slavecount; i++) {
            ec_slavet *slave = &inst->ecx_context.slavelist[i];
            if (slave->state != EC_STATE_OPERATIONAL) {
                plugin_logger_error(logger,
                    "  Slave %d (%s): state=0x%04X, ALstatuscode=0x%04X",
                    i, slave->name, slave->state, slave->ALstatuscode);
            }
        }

        return -1;
    }

    plugin_logger_info(logger, "EtherCAT master operational with %d slave(s)",
                       inst->ecx_context.slavecount);

    return 0;
}

/*
 * =============================================================================
 * Master Close
 * =============================================================================
 */

void ecat_master_close(ecat_master_instance_t *inst, plugin_logger_t *logger)
{
    if (inst->soem_initialized) {
        /* Transition all slaves to INIT state */
        plugin_logger_info(logger, "Transitioning slaves to INIT state...");
        inst->ecx_context.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&inst->ecx_context, 0);

        /* Close the network interface */
        ecx_close(&inst->ecx_context);
        inst->soem_initialized = 0;
    }

    /* Always attempt to revert iface state, even if SOEM init had failed
     * after we already modified the interface.  Revert reads in-memory
     * flags and is a no-op when nothing was applied. */
    ecat_iface_state_revert(&inst->iface_state, logger);

    /* Clear IO map */
    memset(inst->iomap, 0, sizeof(inst->iomap));
    inst->iomap_used_size = 0;

    plugin_logger_info(logger, "EtherCAT master closed");
}

/*
 * =============================================================================
 * Process Data and State Access
 * =============================================================================
 */

int ecat_master_exchange_processdata(ecat_master_instance_t *inst, int timeout_us)
{
    ecx_send_processdata(&inst->ecx_context);
    int wkc = ecx_receive_processdata(&inst->ecx_context,
                                       (timeout_us > 0) ? timeout_us : EC_TIMEOUTRET);
    return wkc;
}

int ecat_master_get_expected_wkc(ecat_master_instance_t *inst)
{
    ec_groupt *grp = &inst->ecx_context.grouplist[0];
    return (grp->outputsWKC * 2) + grp->inputsWKC;
}

const ec_slavet *ecat_master_get_slave(ecat_master_instance_t *inst, int position)
{
    if (position < 1 || position > inst->ecx_context.slavecount)
        return NULL;
    return &inst->ecx_context.slavelist[position];
}

uint16_t ecat_master_get_slave_state(ecat_master_instance_t *inst, int position)
{
    if (position < 1 || position > inst->ecx_context.slavecount)
        return 0;
    return inst->ecx_context.slavelist[position].state;
}

/*
 * =============================================================================
 * Slave Recovery
 * =============================================================================
 */

int ecat_master_recover_slave(ecat_master_instance_t *inst, int position, plugin_logger_t *logger)
{
    if (position < 1 || position > inst->ecx_context.slavecount) {
        plugin_logger_error(logger, "Invalid slave position %d for recovery", position);
        return -1;
    }

    ec_slavet *slave = &inst->ecx_context.slavelist[position];
    uint16_t current_state = slave->state;

    if (current_state == EC_STATE_OPERATIONAL) {
        /* Already operational */
        return 1;
    }

    if (current_state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
        /* SAFE_OP + ERROR: ACK the error, then request OP */
        plugin_logger_info(logger,
            "Slave %d (%s): SAFE_OP+ERROR (ALstatus=0x%04X), sending ACK",
            position, slave->name, slave->ALstatuscode);

        slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
        ecx_writestate(&inst->ecx_context, (uint16)position);

        /* Now request OP */
        slave->state = EC_STATE_OPERATIONAL;
        ecx_writestate(&inst->ecx_context, (uint16)position);

        /* Check if it worked */
        ecx_statecheck(&inst->ecx_context, (uint16)position,
                        EC_STATE_OPERATIONAL, EC_TIMEOUTRET);

        if (slave->state == EC_STATE_OPERATIONAL) {
            plugin_logger_info(logger, "Slave %d (%s): recovered to OP",
                               position, slave->name);
            return 1;
        }
        return 0;
    }

    if (current_state == EC_STATE_SAFE_OP) {
        /* SAFE_OP: just request OP */
        plugin_logger_info(logger, "Slave %d (%s): in SAFE_OP, requesting OP",
                           position, slave->name);

        slave->state = EC_STATE_OPERATIONAL;
        ecx_writestate(&inst->ecx_context, (uint16)position);

        ecx_statecheck(&inst->ecx_context, (uint16)position,
                        EC_STATE_OPERATIONAL, EC_TIMEOUTRET);

        if (slave->state == EC_STATE_OPERATIONAL) {
            plugin_logger_info(logger, "Slave %d (%s): recovered to OP",
                               position, slave->name);
            return 1;
        }
        return 0;
    }

    if (current_state > EC_STATE_NONE) {
        /* Lower state but still present: try full reconfiguration */
        plugin_logger_info(logger,
            "Slave %d (%s): state=0x%04X, attempting reconfig",
            position, slave->name, current_state);

        if (ecx_reconfig_slave(&inst->ecx_context, (uint16)position, EC_TIMEOUTRET)) {
            slave->islost = FALSE;
            plugin_logger_info(logger, "Slave %d (%s): reconfigured", position, slave->name);

            /* After reconfig, check if it reached OP */
            ecx_statecheck(&inst->ecx_context, (uint16)position,
                            EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_OPERATIONAL)
                return 1;
            return 0;
        }
        return 0;
    }

    /* EC_STATE_NONE: slave is lost, try recover */
    if (!slave->islost) {
        ecx_statecheck(&inst->ecx_context, (uint16)position,
                        EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
        if (slave->state == EC_STATE_NONE) {
            slave->islost = TRUE;
            plugin_logger_warn(logger, "Slave %d (%s): marked as lost",
                               position, slave->name);
        }
        return 0;
    }

    /* Slave was marked lost - try to recover */
    if (ecx_recover_slave(&inst->ecx_context, (uint16)position, EC_TIMEOUTRET)) {
        slave->islost = FALSE;
        plugin_logger_info(logger, "Slave %d (%s): recovered from lost state",
                           position, slave->name);
        return 1;
    }

    return 0;
}

void ecat_master_read_states(ecat_master_instance_t *inst)
{
    if (inst->soem_initialized)
        ecx_readstate(&inst->ecx_context);
}

/*
 * =============================================================================
 * IOmap Access
 * =============================================================================
 */

uint8_t *ecat_master_get_iomap(ecat_master_instance_t *inst)
{
    if (!inst->soem_initialized)
        return NULL;
    return inst->iomap;
}

size_t ecat_master_get_iomap_size(ecat_master_instance_t *inst)
{
    return inst->iomap_used_size;
}

int ecat_master_get_slave_count(ecat_master_instance_t *inst)
{
    if (!inst->soem_initialized)
        return 0;
    return inst->ecx_context.slavecount;
}
