/**
 * @file ethercat_iface_state.h
 * @brief Unified per-interface external-state manager for the EtherCAT plugin.
 *
 * Consolidates two mechanisms that previously lived in separate places
 * with the same shape:
 *
 *   - NIC tuning save/restore (ethtool -c / -k / -C / -K).
 *     Originally in ethercat_master.c (~460 lines).
 *
 *   - IP-stack isolation (iptables INPUT DROP, IPv6 sysctl flip).
 *     Originally in ethercat_plugin.c (~280 lines).
 *
 * Both shared the same crash-recovery pattern (atomic write+rename in
 * /run/runtime, file-on-disk serves as a reliquia after a SIGKILL or
 * OOM so the next start can roll back what was applied).  This module
 * folds them into a single state struct, a single persistence file per
 * interface, and a single apply/revert pair.
 *
 * Persistence file:  /run/runtime/ecat_iface_<iface>.state
 *
 * Migration: legacy files (ecat_nic_saved_<iface>.conf,
 * ecat_iface_iso_<iface>.state) from earlier versions are detected on
 * apply, reverted, and removed before the unified flow runs.
 */

#ifndef ETHERCAT_IFACE_STATE_H
#define ETHERCAT_IFACE_STATE_H

#include "ethercat_config.h"   /* ecat_iface_state_t */
#include "plugin_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Apply low-latency NIC tuning + IP-stack isolation.
 *
 * Sequence:
 *   1. Migrate legacy state files (older format) and revert anything
 *      they describe.
 *   2. Recover from the unified state file if a previous run crashed.
 *   3. Capture current NIC settings.
 *   4. Apply low-latency settings (rx/tx usecs = 0, GRO/GSO/TSO off,
 *      iptables INPUT DROP, disable_ipv6 = 1).
 *   5. Persist the captured "before" values to /run/runtime so a crash
 *      lets the next start undo what we just applied.
 *
 * On non-Linux platforms this is a no-op.
 *
 * @param state  Per-instance state (zeroed by caller; populated here).
 * @param iface  Interface name from the master config.
 * @param logger Plugin logger.
 */
void ecat_iface_state_apply(ecat_iface_state_t *state, const char *iface,
                            plugin_logger_t *logger);

/**
 * @brief Revert exactly what apply applied.
 *
 * Reads the in-memory flags (no disk involvement) to undo only the
 * changes that this instance made, then deletes the persistence file.
 * Safe to call when nothing was applied (no-op).  Safe to call after a
 * partially-failed apply (skips fields that were never set).
 */
void ecat_iface_state_revert(ecat_iface_state_t *state, plugin_logger_t *logger);

#ifdef __cplusplus
}
#endif

#endif /* ETHERCAT_IFACE_STATE_H */
