/**
 * @file ethercat_iface_state.h
 * @brief Per-interface NIC-tuning state manager for the EtherCAT plugin.
 *
 * Saves the NIC's pre-EtherCAT settings (ethtool -c / -k coalescing
 * and offloads), applies the low-latency tuning the bus thread needs
 * (rx/tx-usecs=0, GRO/GSO/TSO off), and persists the captured "before"
 * values to /run/runtime so a crashed runtime's next start can revert
 * the system to its original state.
 *
 * Persistence file:  /run/runtime/ecat_iface_<iface>.state
 *
 * Migration: legacy files (ecat_nic_saved_<iface>.conf,
 * ecat_iface_iso_<iface>.state) from earlier versions — the latter
 * recorded iptables INPUT DROP + IPv6 sysctl flips, since removed —
 * are detected on apply, reverted, and removed before the current
 * flow runs.  This makes upgrades from any prior version self-heal.
 */

#ifndef ETHERCAT_IFACE_STATE_H
#define ETHERCAT_IFACE_STATE_H

#include "ethercat_config.h"   /* ecat_iface_state_t */
#include "plugin_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Apply low-latency NIC tuning.
 *
 * Sequence:
 *   1. Migrate legacy state files (older formats, including the
 *      iptables/IPv6 isolation file written by previous versions) and
 *      revert anything they describe.
 *   2. Recover from the unified state file if a previous run crashed.
 *   3. Capture current NIC settings (ethtool -c / -k).
 *   4. Apply low-latency settings (rx/tx-usecs = 0, GRO/GSO/TSO off).
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
