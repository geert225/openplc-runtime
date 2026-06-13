/**
 * @file profibus_serial.h
 * @brief Serial port (RS-485) transport for the Profibus DP master
 *
 * Wraps termios configuration (baud rate, parity, stop bits, raw mode) and
 * optional manual RTS toggling for RS-485 transceivers that require the
 * driver-enable line to be asserted around each transmission.
 *
 * Standard Profibus DP baud rates 9600..12000000 are supported. Rates that
 * map to POSIX B-constants (9600..4000000 on Linux) use the normal termios
 * API; rates that do not (93750, 187500, 6000000, 12000000) use the Linux
 * termios2/BOTHER ioctl interface for arbitrary baud rates.
 */

#ifndef PROFIBUS_SERIAL_H
#define PROFIBUS_SERIAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "profibus_config.h"
#include "plugin_logger.h"

typedef struct {
    int  fd;
    bool rts_control;
    int  rts_delay_us;
    int  baudrate;     /* stored for inter-byte timeout calculation in pb_serial_read() */
} pb_serial_port_t;

/**
 * @brief Open and configure the serial port for Profibus DP
 *
 * Configures 8 data bits, the requested parity and stop bits, raw
 * (non-canonical) mode, and the requested baud rate.
 *
 * @param port        Output port handle
 * @param device      Path to the serial device (e.g. "/dev/ttyUSB0")
 * @param baudrate    Baud rate in bit/s
 * @param parity      Parity setting
 * @param stop_bits   1 or 2
 * @param rts_control Enable manual RTS toggling around transmissions
 * @param rts_delay_us Delay (microseconds) applied before/after toggling RTS
 * @param logger      Logger for diagnostic messages (may be NULL)
 * @return 0 on success, -1 on failure
 */
int pb_serial_open(pb_serial_port_t *port, const char *device, int baudrate,
                    pb_parity_t parity, int stop_bits, bool rts_control,
                    int rts_delay_us, plugin_logger_t *logger);

/** Close the serial port. Safe to call on an already-closed port. */
void pb_serial_close(pb_serial_port_t *port);

/**
 * @brief Write a telegram to the bus
 *
 * If RTS control is enabled, asserts RTS, waits @p rts_delay_us, writes the
 * full buffer, waits for the data to drain, waits @p rts_delay_us again,
 * then de-asserts RTS.
 *
 * @return Number of bytes written, or -1 on error.
 */
int pb_serial_write(pb_serial_port_t *port, const uint8_t *buf, size_t len);

/**
 * @brief Read up to @p max_len bytes with a timeout
 *
 * @param timeout_us Maximum time to wait for the first byte, in microseconds.
 *                    Once data starts arriving, subsequent bytes are read
 *                    with a short inter-byte timeout to collect the full
 *                    telegram without blocking indefinitely.
 * @return Number of bytes read (0 on timeout with no data), or -1 on error.
 */
int pb_serial_read(pb_serial_port_t *port, uint8_t *buf, size_t max_len, int timeout_us);

/** Discard any unread input and pending output. */
void pb_serial_flush(pb_serial_port_t *port);

#endif /* PROFIBUS_SERIAL_H */
