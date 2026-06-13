/**
 * @file profibus_io.h
 * @brief Profibus DP I/O Module -- IEC location parsing, channel mapping,
 *        and cyclic data exchange with the OpenPLC process image
 *
 * Each Profibus slave exchanges a flat input_data[]/output_data[] byte
 * buffer with the master (via DPV0 Data_Exchange). This module bridges
 * those buffers to the OpenPLC %I/%Q process image, mirroring the EtherCAT
 * plugin's ethercat_io.* but operating on a per-slave byte buffer instead
 * of a shared IOmap.
 *
 * Flow (per slave):
 *  1. At startup, pb_io_build_channel_map() walks every configured channel,
 *     parses its IEC location, and stores a compact mapping entry.
 *  2. Still at startup, pb_io_build_transfer_list() pre-resolves each map
 *     entry into a {plc_ptr, data_byte_offset, byte_count} triple, once the
 *     PLC image table pointers are available.
 *  3. Each bus cycle (hot path):
 *     - pb_io_write_outputs_fast() copies PLC output variables into
 *       output_data[] before sending Data_Exchange.
 *     - pb_io_read_inputs_fast() publishes input_data[] (received from the
 *       slave) into the %I image via the lock-free journal.
 */

#ifndef PROFIBUS_IO_H
#define PROFIBUS_IO_H

#include <stdint.h>

#include "profibus_config.h"
#include "plugin_types.h"
#include "plugin_logger.h"

/**
 * @brief Parse an IEC 61131-3 location string into its components
 *
 * Accepted format: %[IQ][XBWDL]<byte>[.<bit>]
 *   - Direction: I (input) or Q (output)
 *   - Size: X (bit), B (byte), W (word), D (dword), L (lword)
 *   - Byte: decimal byte address
 *   - Bit: optional, only valid for X size, 0-7
 *
 * @return 0 on success, -1 on parse error
 */
int pb_io_parse_iec_location(const char *loc_str, iec_location_t *loc);

/**
 * @brief Build the channel map for one slave from its configuration
 *
 * Parses each channel's IEC location and records its offset within the
 * slave's input_data[]/output_data[] buffer for later use by
 * pb_io_build_transfer_list().
 *
 * @param slave  Slave configuration (channels array)
 * @param map    Output channel map (zeroed before population)
 * @param args   Runtime args (for buffer_size bounds check)
 * @param logger Logger instance
 * @return 0 on success, -1 if any channel failed to map
 */
int pb_io_build_channel_map(const pb_slave_t *slave, pb_channel_map_t *map,
                             plugin_runtime_args_t *args, plugin_logger_t *logger);

/**
 * @brief Build a transfer list from a channel map and runtime args
 *
 * Resolves each channel map entry into a direct {plc_ptr, data_byte_offset,
 * byte_count} triple. Entries whose PLC pointer is NULL (unmapped IEC
 * address) are silently skipped.
 *
 * Must be called after pb_io_build_channel_map() and after glueVars() has
 * populated the image table pointers.
 *
 * @return 0 on success, -1 if zero channels resolved
 */
int pb_io_build_transfer_list(const pb_channel_map_t *map, pb_transfer_list_t *xfer,
                               plugin_runtime_args_t *args, plugin_logger_t *logger);

/**
 * @brief Fast per-cycle: publish a slave's input_data[] into the %I image
 *
 * Input values are written through the lock-free journal
 * (args->journal_write_*) rather than poked directly into the image, so
 * they apply atomically at the next scan boundary and never race the IEC
 * task threads. No image lock is held here.
 */
void pb_io_read_inputs_fast(const pb_transfer_list_t *xfer, const uint8_t *input_data,
                             plugin_runtime_args_t *args);

/**
 * @brief Fast per-cycle: copy PLC output variables into a slave's output_data[]
 */
void pb_io_write_outputs_fast(const pb_transfer_list_t *xfer, uint8_t *output_data);

#endif /* PROFIBUS_IO_H */
