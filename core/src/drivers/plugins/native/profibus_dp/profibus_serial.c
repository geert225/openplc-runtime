/**
 * @file profibus_serial.c
 * @brief Serial port (RS-485) transport for the Profibus DP master
 */

#include "profibus_serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if !defined(__CYGWIN__) && !defined(_WIN32)

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

/*
 * =============================================================================
 * Custom baud rate support (Linux termios2/BOTHER)
 *
 * Standard Profibus rates 500000/1500000/3000000 map onto glibc's extended
 * B-constants (Linux defines B500000.. B4000000). The non-standard rates
 * 93750, 187500, 6000000 and 12000000 require the termios2/BOTHER ioctl
 * interface. We avoid including <asm/termbits.h> directly (it redefines
 * `struct termios` and clashes with glibc's <termios.h>); instead we
 * declare the minimal pieces needed, matching the Linux kernel ABI on
 * x86/x86_64/ARM/ARM64 (the platforms OpenPLC runtime targets).
 * =============================================================================
 */
#if defined(__linux__)

#ifndef BOTHER
#define BOTHER 0010000
#endif

/*
 * glibc's <asm-generic/ioctls.h> (pulled in via <sys/ioctl.h>) already
 * defines TCGETS2/TCSETS2 as _IOR('T',0x2A,struct termios2)/_IOW(...), which
 * requires the real (incomplete here) `struct termios2` to be complete at
 * the point of use. Override with the equivalent numeric constants so they
 * work with our `struct pb_termios2` instead.
 */
#ifdef TCGETS2
#undef TCGETS2
#endif
#define TCGETS2 0x802C542A

#ifdef TCSETS2
#undef TCSETS2
#endif
#define TCSETS2 0x402C542B

struct pb_termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t     c_line;
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};

#endif /* __linux__ */

/*
 * =============================================================================
 * Baud rate -> B-constant table for rates supported by glibc's termios.
 * =============================================================================
 */
typedef struct {
    int     rate;
    speed_t flag;
} baud_entry_t;

static const baud_entry_t k_baud_table[] = {
    { 1200,    B1200 },
    { 2400,    B2400 },
    { 4800,    B4800 },
    { 9600,    B9600 },
    { 19200,   B19200 },
    { 38400,   B38400 },
    { 57600,   B57600 },
    { 115200,  B115200 },
    { 230400,  B230400 },
#ifdef B460800
    { 460800,  B460800 },
#endif
#ifdef B500000
    { 500000,  B500000 },
#endif
#ifdef B921600
    { 921600,  B921600 },
#endif
#ifdef B1000000
    { 1000000, B1000000 },
#endif
#ifdef B1500000
    { 1500000, B1500000 },
#endif
#ifdef B2000000
    { 2000000, B2000000 },
#endif
#ifdef B3000000
    { 3000000, B3000000 },
#endif
#ifdef B4000000
    { 4000000, B4000000 },
#endif
};

static int find_baud_flag(int rate, speed_t *out)
{
    for (size_t i = 0; i < sizeof(k_baud_table) / sizeof(k_baud_table[0]); i++) {
        if (k_baud_table[i].rate == rate) {
            *out = k_baud_table[i].flag;
            return 0;
        }
    }
    return -1;
}

static void log_err(plugin_logger_t *logger, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (logger && logger->is_valid)
        plugin_logger_error(logger, "%s", buf);
    else
        fprintf(stderr, "[PROFIBUS_SERIAL] ERROR: %s\n", buf);
}

int pb_serial_open(pb_serial_port_t *port, const char *device, int baudrate,
                    pb_parity_t parity, int stop_bits, bool rts_control,
                    int rts_delay_us, plugin_logger_t *logger)
{
    memset(port, 0, sizeof(*port));

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        log_err(logger, "Failed to open %s: %s", device, strerror(errno));
        return -1;
    }

    /* Switch back to blocking I/O; pb_serial_read() uses select() for
     * timeouts so we want blocking reads/writes once data is available. */
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        log_err(logger, "tcgetattr(%s) failed: %s", device, strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag |= CLOCAL | CREAD;

    switch (parity) {
    case PB_PARITY_EVEN:
        tio.c_cflag |= PARENB;
        tio.c_cflag &= ~PARODD;
        break;
    case PB_PARITY_ODD:
        tio.c_cflag |= PARENB;
        tio.c_cflag |= PARODD;
        break;
    case PB_PARITY_NONE:
    default:
        tio.c_cflag &= ~PARENB;
        break;
    }

    if (stop_bits == 2)
        tio.c_cflag |= CSTOPB;
    else
        tio.c_cflag &= ~CSTOPB;

    /* Non-blocking read with timeout handled by select() in pb_serial_read(). */
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    speed_t baud_flag;
    bool custom_baud = (find_baud_flag(baudrate, &baud_flag) != 0);

    if (!custom_baud) {
        cfsetispeed(&tio, baud_flag);
        cfsetospeed(&tio, baud_flag);
    }
#if defined(__linux__)
    else {
        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
    }
#else
    else {
        log_err(logger, "Baud rate %d is not supported on this platform", baudrate);
        close(fd);
        return -1;
    }
#endif

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        log_err(logger, "tcsetattr(%s) failed: %s", device, strerror(errno));
        close(fd);
        return -1;
    }

#if defined(__linux__)
    if (custom_baud) {
        struct pb_termios2 tio2;
        memset(&tio2, 0, sizeof(tio2));
        if (ioctl(fd, TCGETS2, &tio2) != 0) {
            log_err(logger, "TCGETS2(%s) failed: %s", device, strerror(errno));
            close(fd);
            return -1;
        }
        tio2.c_cflag &= ~CBAUD;
        tio2.c_cflag |= BOTHER;
        tio2.c_ispeed = (speed_t)baudrate;
        tio2.c_ospeed = (speed_t)baudrate;
        if (ioctl(fd, TCSETS2, &tio2) != 0) {
            log_err(logger, "TCSETS2(%s, %d baud) failed: %s", device, baudrate, strerror(errno));
            close(fd);
            return -1;
        }
    }
#endif

    /* Idle RTS low (driver disabled) until the first transmission. */
    if (rts_control) {
        int bits = TIOCM_RTS;
        ioctl(fd, TIOCMBIC, &bits);
    }

    tcflush(fd, TCIOFLUSH);

    port->fd = fd;
    port->rts_control = rts_control;
    port->rts_delay_us = rts_delay_us;
    port->baudrate = baudrate;
    return 0;
}

void pb_serial_close(pb_serial_port_t *port)
{
    if (port->fd >= 0) {
        close(port->fd);
        port->fd = -1;
    }
}

int pb_serial_write(pb_serial_port_t *port, const uint8_t *buf, size_t len)
{
    if (port->fd < 0)
        return -1;

    if (port->rts_control) {
        int bits = TIOCM_RTS;
        ioctl(port->fd, TIOCMBIS, &bits);
        if (port->rts_delay_us > 0)
            usleep((useconds_t)port->rts_delay_us);
    }

    size_t total = 0;
    while (total < len) {
        ssize_t n = write(port->fd, buf + total, len - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            if (port->rts_control) {
                int bits = TIOCM_RTS;
                ioctl(port->fd, TIOCMBIC, &bits);
            }
            return -1;
        }
        total += (size_t)n;
    }

    if (port->rts_control) {
        tcdrain(port->fd);
        if (port->rts_delay_us > 0)
            usleep((useconds_t)port->rts_delay_us);
        int bits = TIOCM_RTS;
        ioctl(port->fd, TIOCMBIC, &bits);
    }

    return (int)total;
}

int pb_serial_read(pb_serial_port_t *port, uint8_t *buf, size_t max_len, int timeout_us)
{
    if (port->fd < 0)
        return -1;

    /* Inter-byte timeout: roughly 2 character times (start+8+parity+stop
     * bits = ~11 bits), with a floor of 200us so very high baud rates
     * don't busy-spin select(). */
    int char_time_us = (port->baudrate > 0) ? (11 * 1000000 / port->baudrate) : 1000;
    int inter_byte_timeout_us = char_time_us * 2;
    if (inter_byte_timeout_us < 200)
        inter_byte_timeout_us = 200;

    size_t total = 0;
    int wait_us = timeout_us;

    while (total < max_len) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(port->fd, &rfds);

        struct timeval tv;
        tv.tv_sec = wait_us / 1000000;
        tv.tv_usec = wait_us % 1000000;

        int rc = select(port->fd + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (rc == 0)
            break; /* timeout */

        ssize_t n = read(port->fd, buf + total, max_len - total);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0)
            break;

        total += (size_t)n;
        wait_us = inter_byte_timeout_us;
    }

    return (int)total;
}

void pb_serial_flush(pb_serial_port_t *port)
{
    if (port->fd >= 0)
        tcflush(port->fd, TCIOFLUSH);
}

#else /* MSYS2 / Cygwin / native Windows -- Profibus serial transport not supported */

int pb_serial_open(pb_serial_port_t *port, const char *device, int baudrate,
                    pb_parity_t parity, int stop_bits, bool rts_control,
                    int rts_delay_us, plugin_logger_t *logger)
{
    (void)device; (void)baudrate; (void)parity; (void)stop_bits;
    (void)rts_control; (void)rts_delay_us;
    memset(port, 0, sizeof(*port));
    port->fd = -1;
    if (logger && logger->is_valid)
        plugin_logger_error(logger, "Profibus serial transport is not supported on this platform");
    return -1;
}

void pb_serial_close(pb_serial_port_t *port) { (void)port; }

int pb_serial_write(pb_serial_port_t *port, const uint8_t *buf, size_t len)
{
    (void)port; (void)buf; (void)len;
    return -1;
}

int pb_serial_read(pb_serial_port_t *port, uint8_t *buf, size_t max_len, int timeout_us)
{
    (void)port; (void)buf; (void)max_len; (void)timeout_us;
    return -1;
}

void pb_serial_flush(pb_serial_port_t *port) { (void)port; }

#endif
