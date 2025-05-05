/**
 * @package   libserial
 * @brief     Serial port management.
 * @author    José Luis Sánchez Arroyo
 * @section   License
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the BSD 3-Clause License and comes WITHOUT ANY WARRANTY.
 * Please read the file LICENSE for further details.
 */

#ifndef __LIBSERIAL_H__
#define __LIBSERIAL_H__

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>                                  // size_t, ssize_t
#include <sys/ioctl.h>                                  // TIOCM_LE and other serial line constants
#include <fcntl.h>                                      // FCNTL, O_NONBLOCK
#include <string>                                       // std::string

namespace libserial {

/**
 * @brief     Function to get the library version at runtime.
 */
const char* version();

/** ----------------------------------------------------
 * @brief     Public enumerated types, used by some functions of the class
 * ------ */

/**
 * @brief     Read modes.
 * @see       Serial(), setBlocking()
 */
enum BlockingMode
{
    Blocking          = 0,                              //!< Blocking read/write.
    NonBlocking       = O_NONBLOCK                      //!< Non-blocking read/write.
};

/**
 * @brief     Flow control modes.
 * @see       Serial()
 */
enum FlowControl
{
    NoFlowCtrl        = 0,
    HardwareFlowCtrl  = CRTSCTS,
    XonXoffInput      = IXOFF,
    XonXoffOutput     = IXON,
    XonXoffBoth       = (IXON | IXOFF)
};

/**
 * @brief     Char sizes enum.
 * @see       Serial()
 */
enum CharLen
{
    c5bits            = CS5,
    c6bits            = CS6,
    c7bits            = CS7,
    c8bits            = CS8
};

/**
 * @brief     Parity types.
 * @see       Serial()
 */
enum Parity
{
    NoParity          = 0,                              //!< No parity.
    EvenParity        = PARENB,                         //!< Even parity.
    OddParity         = PARODD                          //!< Odd parity.
};

/**
 * @brief     Stop bits count.
 * @see       Serial()
 */
enum StopBits
{
    stop1bit          = 0,                              //!< 1 stop bit.
    stop2bits         = CSTOP                           //!< 2 stop bits.
};

/**
 * @brief     Clear buffer operation type.
 * @see       clearBuffer()
 */
enum ClearOper
{
    CLEAR_BUF_IN = 1,                                   //!< Clear input buffer, discarding pending data.
    CLEAR_BUF_OUT,                                      //!< Clear output buffer, discarding pending data.
    FLUSH_BUF_OUT                                       //!< Flush output buffer, waiting till it is empty.
};

/**
 * @brief     Serial port lines.
 * @see       getLine(), setLine()
 */
enum SerialLine
{
    LINE_LE   = TIOCM_LE,                               //!<
    LINE_DTR  = TIOCM_DTR,                              //!< DTR (Data Terminal Ready)
    LINE_RTS  = TIOCM_RTS,                              //!< RTS (Ready To Send)
    LINE_ST   = TIOCM_ST,                               //!< ST (Serial Transmit)
    LINE_SR   = TIOCM_SR,                               //!< SR (Serial Receive)
    LINE_CTS  = TIOCM_CTS,                              //!< CTS (Clear To Send)
    LINE_CAR  = TIOCM_CAR,                              //!<
    LINE_RNG  = TIOCM_RNG,                              //!< RNG (Ring)
    LINE_DSR  = TIOCM_DSR                               //!< DSR (Data Send Ready)
};

/**
 * @brief     Timeout special values.
 * @see       read()
 */
enum
{
    NO_TIMEOUT = 0                                      //!< Read with no timeout.
};

/** ----------------------------------------------------
 * @brief     Class Serial: Serial port management.
 * ------ */
class Serial
{
public:
    /** ----------------------------------------------------
     * @brief   Public functions.
     * ------ */

    /**
     * @brief   Constructor. Open and configure the serial port.
     * @details The serial port is open and the requested configuration is applied. Previous configuration is stored.
     * @throw   std::invalid_argument if any of the arguments provided is not valid
     * @throw   std::system_error on error while opening or configuring the serial port.
     * @note    Serial ports only allow a discrete set of baudrates. If the provided baudrate is not in the list, it is rounded down
     *          to the nearest value.
     * @see     Enumerated types above.
     */
    Serial (
        const std::string& devname,                     //!< Path to the serial device.
        uint32_t baudrate,                              //!< Port speed, in bits per second.
        BlockingMode blockmode = NonBlocking,           //!< Read mode (blocking or non-blocking) [ = non-blocking ]
        FlowControl flowcontrol = NoFlowCtrl,           //!< Flow control [ = no flow control ]
        CharLen charlen = c8bits,                       //!< Character size [ = 8  bits ]
        Parity parity = NoParity,                       //!< Parity control type [ = no parity ]
        StopBits stopbits = stop1bit                    //!< Stop bits [ = 1 stop bit ]
    );

    /**
     * @brief   Copy constructor (deleted)
     */
    Serial(Serial&) = delete;

    /**
     * @brief   Move constructor.
     * @details Moves data from the provided object to this.
     */
    Serial (
        Serial&& other                                  //!< Object to be moved
    );

    /**
     * @brief   Destructor.
     * @details Restore previous configuration and close the port device.
     */
    ~Serial ();

    /**
     * @brief   Read data from the serial port.
     * @details The function behaviour depends upon the current Blocking mode and the timeout parameter:
     *        - If NO_TIMEOUT is specified:
     *          - If Blocking is ON: Wait forever until all the requested bytes are received.
     *          - If Blocking is OFF: Get any bytes in the FIFO and return immediately. If no data pending, return 0.
     *        - Otherwise:
     *          - Continue reading data until all the requested data is read or timeout is reached (whichever happens first).
     * @throws  std::invalid_argument if buf is nullptr.
     * @throws  std::system_error on read error
     */
    ssize_t                                             /** @return Bytes received. */
    read (
        void* buf,                                      //!< Pointer to the buffer for the data received
        std::size_t size,                               //!< Number of bytes to read
        uint32_t t_out                                  //!< Timeout, in ms, or NO_TIMEOUT.
    );

    /**
     * @brief   Write data to the serial port.
     * @details On Blocking mode, the function will wait until all the data is transferred to the kernel buffer.
     *          On NonBlocking mode, the function will return immediately if the kernel buffer gets full.
     * @throws  std::invalid_argument if buf is nullptr.
     * @throws  std::system_error if data can't be written.
     */
    ssize_t                                             /** @return Number of bytes written. Can be less than the buffer length in non-blocking mode. */
    write (
        const void* buf,                                //!< Pointer to the buffer to write
        std::size_t size                                //!< Bytes to write
    );

    /**
     * @brief   Write one byte to the serial port.
     */
    bool                                                /** @return true: Character sent. \n false: Error */
    writeByte (
        uint8_t byte                                    //!< Byte to write
    ) noexcept;

    /**
     * @brief   Check if there's any data in the output queue.
     * @throws  std::system_error if the port status can't be read.
     */
    int                                                 /** @return 0: Output queue is empty. \n Bytes waiting to be sent */
    pendingWrite();

    /**
     * @brief   Check if there's any data in the input queue.
     * @throws  std::system_error if the port status can't be read.
     */
    int                                                 /** @return 0: Input queue is empty. \n Bytes pending to be read */
    pendingRead();

    /**
     * @brief   Wait till the output queue is empty.
     * @throws  std::system_error if the port status can't be read.
     */
    void
    waitSend();

    /**
     * @brief   Clear the input and/or the output queue.
     */
    void
    clearBuffer (
        ClearOper operation                             //!< Operation to perform. @see EnClearOper
    ) noexcept;

    /**
     * @brief   Set serial port line status.
     * @throws  std::system_error if the serial line status can't be read or written.
     */
    void
    setLine (
        SerialLine line,                                //!< Line to change
        bool mode                                       //!< Value to set (on/off)
    );

    /**
     * @brief   Set serial port line status.
     * @throws  std::system_error if the serial line status can't be read.
     */
    bool                                                /** @return true: Line is on. \n false: Line is off */
    getLine (
        SerialLine line                                 //!< Line to read
    );

    /**
     * @brief   Set reading mode (blocking or non-blocking)
     * @throws  std::system_error if the device control flags can't be read or written.
     */
    void
    setBlocking (
        BlockingMode mode                               //!< New read blocking mode
    );

protected:
    /**
     * @brief   Convert the baudrate parameter from integer to termios constant.
     * @details Value is rounded down to the nearest value.
     */
    tcflag_t                                            /** @return termios constant for the requested baudrate. */
    getBaudCode (
        uint32_t baudrate                               //!< Requested baudrate to convert.
    );

    int     handle_;                                    //!< File handler
    termios prev_tio_;                                  //!< Struct to contain the previous port configuration, to restore it on close.
};

}   // namespace

#endif // __LIBSERIAL_H__
