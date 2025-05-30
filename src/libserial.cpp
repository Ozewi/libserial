/**
 * @package   libserial: Serial port communications.
 * @brief     Class for managing serial port
 * @author    José Luis Sánchez Arroyo
 * @section   License
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the BSD 3-Clause License and comes WITHOUT ANY WARRANTY.
 * Please read the file LICENSE for further details.
 */

#include "libserial.h"
#include <unistd.h>                                     // close, read, write
#include <sys/poll.h>                                   // poll
#include <errno.h>                                      // errno
#include <string.h>                                     // strerror
#include <ios>                                          // std::ios_base::failure
#include <chrono>                                       // std::chrono
#include <thread>                                       // std::this_thread::sleep_for
#include <stdexcept>                                    // exceptions

// Useful macros for throwing exceptions with source info
#define CONTEXTUALIZE(txt) std::string(" (") + __FUNCTION__ + ") - " txt

#define THROW_SYSTEM_ERROR(txt) throw std::system_error(errno, std::generic_category(), CONTEXTUALIZE(txt))
#define THROW_INVALID_ARGUMENT(txt) throw std::invalid_argument(CONTEXTUALIZE(txt))

namespace libserial {

/** ----------------------------------------------------
 * @brief     Class Serial: Serial port management.
 * ------ */

/**
 * @brief     Constructor. Open and configure the serial port.
 * @details   The serial port is open and the requested configuration is applied. Previous configuration is stored.
 * @throw     std::invalid_argument if any of the arguments provided is not valid
 * @throw     std::ios_base::failure on error while opening or configuring the serial port.
 * @see       libserial.h for a list of the enumerated types in the parameters.
 */
Serial::Serial(const std::string& devname, uint32_t baudrate, BlockingMode blockmode, FlowControl flowcontrol, CharLen charlen, Parity parity, StopBits stopbits)
  : handle_(-1), prev_tio_()
{
    auto baud_code = getBaudCode(baudrate);             // Get the baudrate code from the baudrate value, rounded down.
    if (baud_code == 0)
        THROW_INVALID_ARGUMENT("Requested baudrate is too low");

    handle_ = ::open(devname.c_str(), O_RDWR | O_NOCTTY | blockmode);
    if (handle_ < 0)                                    // Open error
        THROW_SYSTEM_ERROR("open()");

    if (tcgetattr(handle_, &prev_tio_) < 0)             // Store the previous termios configuration
    {
        ::close(handle_);
        THROW_SYSTEM_ERROR("tcgetattr()");
    }

    termios tio = {};
    tio.c_iflag = (flowcontrol & (IXON | IXOFF)) | IGNPAR | (parity)? INPCK : 0;
    tio.c_oflag = 0;
    tio.c_cflag = (charlen & CSIZE) | (stopbits & CSTOPB) | CLOCAL | CREAD | (parity? (PARENB | (parity & PARODD)) : 0) | (flowcontrol & CRTSCTS);
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 0;                                // Timeout disabled by default
    tio.c_cc[VMIN]  = 1;                                // Minimum number of chars to read
    cfsetospeed(&tio, baud_code);
    cfsetispeed(&tio, baud_code);
    if (tcflush(handle_, TCIFLUSH) < 0 || tcsetattr(handle_, TCSANOW, &tio) < 0)
    {
        tcsetattr(handle_, TCSANOW, &prev_tio_);
        ::close(handle_);
        THROW_SYSTEM_ERROR("tcflush() or tcsetattr()");
    }
}

/**
 * @brief     Move constructor.
 * @details   Moves data from the provided object to this.
 */
Serial::Serial(Serial&& other)
{
    handle_ = other.handle_;
    prev_tio_ = other.prev_tio_;
    other.handle_ = -1;                                 // Invalidate the other handle so the port isn't closed.
}

/**
 * @brief   Destructor.
 * @desc    Restore previous configuration and close the port device.
 */
Serial::~Serial()
{
    if (handle_ > 0)
    {
        tcsetattr(handle_, TCSANOW, &prev_tio_);
        ::close(handle_);
    }
}

/**
 * @brief     Read data from the serial port.
 */
ssize_t Serial::read(void* buf, std::size_t size, uint32_t t_out)
{
    if (buf == nullptr)
        THROW_INVALID_ARGUMENT("Null pointer received");

    ssize_t rt = 0;
    if (t_out == NO_TIMEOUT)
    {                                                   // ... on non-blocking, exits immediately returning the pending byte count.
        rt = ::read(handle_, buf, size);
        if (rt < 0 && errno == EAGAIN)                  // On non-blocking mode, EAGAIN is not an error; it means there's nothing to read
            return 0;
        return rt;
    }

    pollfd p_list;
    p_list.fd = handle_;
    p_list.events = POLLIN;
    int err;
    auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(t_out);
    uint8_t* ptr = reinterpret_cast<uint8_t*>(buf);     // Cast for doing pointer arithmetics.
    for (rt = 0; rt < static_cast<ssize_t>(size); )
    {
        do
        {
            auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - std::chrono::steady_clock::now()).count();
            err = (remain > 0)? poll(&p_list, 1, remain) : 0; // If remain is 0 or less (aka time expired), then exit with err = 0 (timeout)
        } while (err < 0 && errno == EINTR);            // On EINTR reception, continue waiting
        if (err < 0)                                    // Read error
            THROW_SYSTEM_ERROR("poll()");
        if (err == 0)                                   // Timeout
            break;
        err = ::read(handle_, &ptr[rt], size - rt);
        if (err < 0)                                    // Read error
            THROW_SYSTEM_ERROR("read()");
        rt += err;
    }
    return rt;
}

/**
 * @brief     Write data to the serial port.
 */
ssize_t Serial::write(const void* buf, std::size_t size)
{
    if (buf == nullptr)
        THROW_INVALID_ARGUMENT("Null pointer received");
    ssize_t bytes = ::write(handle_, buf, size);
    if (bytes < 0                                       // Write error
        && errno != EAGAIN                              // An error for a valid cause should't trigger an exception
        && errno != EWOULDBLOCK
        && errno != EINTR)
        THROW_SYSTEM_ERROR("write()");

    return bytes;
}

/**
 * @brief     Write one byte to the serial port
 */
bool Serial::writeByte(uint8_t byte) noexcept
{
    return (::write(handle_, &byte, 1) == 1);
}

/**
 * @brief     Check if there's any data in the output queue.
 */
int Serial::pendingWrite()
{
    int pending, lsr;
    if (ioctl(handle_, TIOCSERGETLSR, &lsr) < 0)        // Read the line status register
        THROW_SYSTEM_ERROR("ioctl(TIOCSERGETLSR)");
    if (lsr & TIOCSER_TEMT)                             // FIFO and shift registers are empty
        return 0;
    if (ioctl(handle_, TIOCOUTQ, &pending) < 0)
        THROW_SYSTEM_ERROR("ioctl(TIOCOUTQ)");
    return pending? pending : 1;                        // The output FIFO is empty but the UART is not, so let's return 1 anyway.
}

/**
 * @brief     Check if there's any data in the input queue.
 */
int Serial::pendingRead()
{
    int pending;
    if (ioctl(handle_, TIOCINQ, &pending) < 0)
        THROW_SYSTEM_ERROR("ioctl(TIOCINQ)");
    return pending;
}

/**
 * @brief     Wait till the output queue is empty.
 */
void Serial::waitSend()
{
    while (pendingWrite() != 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

/**
 * @brief     Clear the input and/or the output queue.
 */
void Serial::clearBuffer(ClearOper operation) noexcept
{
    switch (operation)
    {
      case CLEAR_BUF_IN:                                // Clear the input queue
        tcflush(handle_, TCIFLUSH);
        break;
      case CLEAR_BUF_OUT:                               // Clear the output queue
        tcflush(handle_, TCOFLUSH);
        break;
      case FLUSH_BUF_OUT:                               // Flush the output buffer
        tcdrain(handle_);                               // This function returns when the kernel finishes flushing out the data to the UART
        break;
    }
}

/**
 * @brief     Set serial port line status.
 */
void Serial::setLine(SerialLine line, bool mode)
{
    unsigned flags;
    if (ioctl(handle_, TIOCMGET, &flags) < 0)
        THROW_SYSTEM_ERROR("ioctl(TIOCMGET)");

    if (mode)
        flags |= line;
    else
        flags &= ~line;

    if (ioctl(handle_, TIOCMSET, &flags) < 0)
        THROW_SYSTEM_ERROR("ioctl(TIOCMSET)");
}

/**
 * @brief     Get serial port line status.
 */
bool Serial::getLine(SerialLine line)
{
    unsigned flags;
    if (ioctl(handle_, TIOCMGET, &flags) < 0)
        THROW_SYSTEM_ERROR("ioctl(TIOCMGET)");

    return bool(flags & line);
}

/**
 * @brief     Set reading mode (blocking or non-blocking)
 */
void Serial::setBlocking (BlockingMode mode)
{
    int flags = fcntl(handle_, F_GETFL, 0);
    if (flags == -1)
        THROW_SYSTEM_ERROR("fcntl(F_GETFL)");

    if (mode)
        flags |= O_NONBLOCK;
    else
        flags &= ~O_NONBLOCK;

    if (fcntl(handle_, F_SETFL, flags) < 0)
        THROW_SYSTEM_ERROR("fcntl(F_SETFL)");
}

/** ----------------------------------------------------
 * @brief     Private functions
 * ------ */

/**
 * @brief     Convert the baudrate parameter from integer to termios constant.
 */
tcflag_t Serial::getBaudCode(uint32_t baudrate)
{
    struct Uint2Tcflag                                  // Private struct to hold equivalences
    {
        uint32_t baud;                                  // Baudrate
        tcflag_t flag;                                  // termios constant
    };

    static const Uint2Tcflag uint2tcflag[] =
    {
        { 4000000, B4000000 },
        { 3500000, B3500000 },
        { 3000000, B3000000 },
        { 2500000, B2500000 },
        { 2000000, B2000000 },
        { 1500000, B1500000 },
        { 1152000, B1152000 },
        { 1000000, B1000000 },
        {  921600, B921600  },
        {  576000, B576000  },
        {  500000, B500000  },
        {  460800, B460800  },
        {  230400, B230400  },
        {  115200, B115200  },
        {   57600, B57600   },
        {   38400, B38400   },
        {   19200, B19200   },
        {    9600, B9600    },
        {    4800, B4800    },
        {    2400, B2400    },
        {    1800, B1800    },
        {    1200, B1200    },
        {     600, B600     },
        {     300, B300     },
        {     200, B200     },
        {     150, B150     },
        {     134, B134     },
        {     110, B110     },
        {      75, B75      },
        {      50, B50      },
    };

    for (const auto& iter : uint2tcflag)
        if (iter.baud <= baudrate)
            return iter.flag;
    return 0;                                           // fallback
}

} // namespace libserial
