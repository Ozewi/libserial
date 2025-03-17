/**
 * @package   libserial
 * @brief     Clase de manejo del puerto serie
 * @author    José Luis Sánchez Arroyo
 * @section   License
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2.1 and comes WITHOUT ANY WARRANTY.
 * Please read the file LICENSE for further details.
 */

#ifndef __LIBSERIAL_H__
#define __LIBSERIAL_H__

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>                                  // size_t, ssize_t
#include <sys/ioctl.h>                                  // TIOCM_LE and other serial line constants
#include <fcntl.h>                                      // O_NDELAY
#include <string>                                       // std::string

namespace libserial {

/**
 * @brief   Function to get the library version at runtime.
 */
const char* version();

}

/** ----------------------------------------------------
 * @brief   Class Serial: Serial port management.
 * ------ */
class Serial
{
public:
    /** ----------------------------------------------------
     * @brief   Public enumerated types, used by some functions of the class
     * ------ */

    /** @brief  Read modes. Used by: constructor, setBlocking */
    enum EnBlockingMode
    {
        Blocking          = 0,                            //!< Blocking read (function 'read' will wait till there's something to read)
        NonBlocking       = O_NDELAY                      //!< Non-blocking read (function 'read' terminates with error if there's nothing to read)
    };

    /** @brief  Flow control modes. Used by: constructor. */
    enum EnFlowControl
    {
        NoFlowCtrl        = 0,
        HardwareFlowCtrl  = CRTSCTS,
        XonXoffInput      = IXOFF,
        XonXoffOutput     = IXON,
        XonXoffBoth       = (IXON | IXOFF)
    };

    /** @brief  Char sizes enum. Used by: constructor. */
    enum EnCharLen
    {
        c5bits            = CS5,
        c6bits            = CS6,
        c7bits            = CS7,
        c8bits            = CS8
    };

    /** @brief  Parity types. Used by: constructor. */
    enum EnParity
    {
        NoParity          = 0,                          //!< No parity.
        EvenParity        = PARENB,                     //!< Even parity.
        OddParity         = PARODD                      //!< Odd parity.
    };

    /** @brief  Stop bits count. Used by: constructor. */
    enum EnStopBits
    {
        stop1bit          = 0,                          //!< 1 stop bit
        stop2bits         = CSTOP                       //!< 2 stop bits
    };

    /** @brief  Clear buffer operation type. Used by: clearBuffer. */
    enum EnClearOper
    {
        CLEAR_BUF_IN = 1,                                 //!< Clear input buffer, discarding pending data.
        CLEAR_BUF_OUT,                                    //!< Clear output buffer, discarding pending data.
        FLUSH_BUF_OUT                                     //!< Flush output buffer, waiting till it is empty.
    };

    /** @brief  Serial port lines. Used by: getLine, setLine. */
    enum SerialLine
    {
        LINE_LE   = TIOCM_LE,                             //!<
        LINE_DTR  = TIOCM_DTR,                            //!< DTR (Data Terminal Ready)
        LINE_RTS  = TIOCM_RTS,                            //!< RTS (Ready To Send)
        LINE_ST   = TIOCM_ST,                             //!< ST (Serial Transmit)
        LINE_SR   = TIOCM_SR,                             //!< SR (Serial Receive)
        LINE_CTS  = TIOCM_CTS,                            //!< CTS (Clear To Send)
        LINE_CAR  = TIOCM_CAR,                            //!<
        LINE_RNG  = TIOCM_RNG,                            //!< RNG (Ring)
        LINE_DSR  = TIOCM_DSR                             //!< DSR (Data Send Ready)
    };

    /** @brief  Timeout special values. Used by: read. */
    enum
    {
        NO_TIMEOUT = 0                                    //!< Read with no timeout. @see read.
    };

    /** ----------------------------------------------------
     * @brief   Public functions.
     * ------ */

    /**
     * @brief   Constructor. Open and configure the serial port.
     * @desc    The serial port is open and the requested configuration is applied. Previous configuration is stored.
     * @throw   invalid_argument if any of the arguments provided is not valid
     * @throw   ios_base::failure on error while opening or configuring the serial port.
     * @note    Serial ports only allow a discrete set of baudrates. If the provided baudrate is not in the list, it is rounded down
     *          to the nearest value.
     * @see     Enumerated types above.
     */
    Serial (
        const std::string& devname,                     /** @param devname     Path to the serial device */
        uint32_t baudrate,                              /** @param baudrate    Port speed, in bits per second */
        EnBlockingMode blockmode = NonBlocking,         /** @param blockmode   Read mode (blocking or non-blocking) [ = non-blocking ]*/
        EnFlowControl flowcontrol = NoFlowCtrl,         /** @param flowcontrol Flow control [ = no flow control ] */
        EnCharLen charlen = c8bits,                     /** @param charlen     Character size [ = 8  bits ] */
        EnParity parity = NoParity,                     /** @param parity      Parity control type [ = no parity ] */
        EnStopBits stopbits = stop1bit                  /** @param stopbits    Stop bits [ = 1 stop bit ] */
    );

    /**
     * @brief   Copy constructor (deleted)
     */
    Serial(Serial&) = delete;

    /**
     * @brief   Move constructor.
     * @desc    Moves data from the provided object to this.
     */
    Serial (
        Serial&& other                                  /** @param other Object to be moved */
    );

    /**
     * @brief   Destructor.
     * @desc    Restore previous configuration and close the port device.
     */
    ~Serial ();

    /**
     * @brief   Read data from the serial port.
     * @desc    Blocking mode affects the function behaviour when specifying NO_TIMEOUT:
     *          On Blocking mode, the function waits forever till all requested bytes are received.
     *          On NonBlocking mode, the function returns immediately when there's no data pending.
     * @throws  invalid_argument if buf is nullptr.
     * @throws  ios_base::failure on read error
     */
    ssize_t                                             /** @return Bytes received */
    read (
        void* buf,                                      /** @param buf    Pointer to the buffer for the data received */
        std::size_t size,                               /** @param size   Number of bytes to read */
        uint32_t t_out                                  /** @param t_out  Timeout, in ms, or NO_TIMEOUT. */
    );

    /**
     * @brief   Write data to the serial port.
     * @throws  invalid_argument if buf is nullptr.
     * @throws  ios_base::failure if data can't be written.
     */
    ssize_t                                             /** @return Number of bytes sent */
    write (
        const void* buf,                                /** @param buf   Pointer to the buffer to write */
        std::size_t size                                /** @param size  Bytes to write */
    );

    /**
     * @brief   Write one byte to the serial port.
     */
    bool                                                /** @return true: Character sent | false: Error */
    writeByte (
        uint8_t byte                                    /** @param byte  Byte a escribir */
    ) noexcept;

    /**
     * @brief   Check if there's any data in the output queue.
     * @throws  ios_base::failure if the port status can't be read.
     */
    int                                                 /** @return 0: Output queue is empty | Bytes waiting to be sent */
    pendingWrite();

    /**
     * @brief   Check if there's any data in the input queue.
     * @throws  ios_base::failure if the port status can't be read.
     */
    int                                                 /** @return 0: Input queue is empty | Bytes pending to be read */
    pendingRead();

    /**
     * @brief   Wait till the output queue is empty.
     * @throws  ios_base::failure if the port status can't be read.
     */
    void                                                /** @return void */
    waitSend();

    /**
     * @brief   Clear the input and/or the output queue.
     */
    void                                                /** @return void */
    clearBuffer (
        EnClearOper operation                           /** @param operation  Operation to perform. @see EnClearOper */
    ) noexcept;

    /**
     * @brief   Set serial port line status.
     * @throws  ios_base::failure if the serial line status can't be read or written.
     */
    void                                                /** @return void */
    setLine (
        SerialLine line,                                /** @param line  Line to change */
        bool mode                                       /** @param mode  Value to set (on/off) */
    );

    /**
     * @brief   Set serial port line status.
     * @throws  ios_base::failure if the serial line status can't be read.
     */
    bool                                                /** @return true: Line is on | false: Line is off */
    getLine (
        SerialLine line                                 /** @param line  Line to read */
    );

    /**
     * @brief   Set reading mode (blocking or non-blocking)
     * @throws  ios_base::failure if the device control flags can't be read or written.
     */
    void                                                /** @return void */
    setBlocking (
        EnBlockingMode mode                             /** @param mode  New read blocking mode */
    );

protected:
    /** ----------------------------------------------------
     * @brief   Protected struct, data and functions
     * ------ */

    /**
     * @brief   Convert the baudrate parameter from integer to termios constant.
     * @desc    Value is rounded down to the nearest value.
     */
    tcflag_t                                            /** @return termios constant for the requested baudrate */
    getBaudCode (
        uint32_t baudrate                               /** @param baudrate  Requested baudrate to convert */
    );

    int     handle_;                                    //!< File handler
    termios prev_tio_;                                  //!< Struct to contain the previous port configuration, to restore it on close.[H
};

#endif // __LIBSERIAL_H__
