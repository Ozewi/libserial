/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.cpp
 * @brief     Clase de manejo del puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.04
 * @version   2.0
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <fcntl.h>
#include <sys/poll.h>
#include <errno.h>
#include <string.h>             // strerror
#include <ios>                  // std::ios_base::failure
#include <chrono>               // std::chrono
#include <thread>               // std::this_thread::sleep_for
#include <stdexcept>            // exceptions

using namespace std::string_literals;

/**-------------------------------------------------------------------------------------------------
 * @brief   Clase Serial: Manejo del puerto serie
 * ------ */

/**
 * @brief   Constructor. Abre y configura el puerto serie.
 * @desc    Abre el dispositivo y aplica la configuración solicitada, guardando la anterior.
 * @note    Explicación detallada de los parámetros en la descripción de los tipos enumerados.
 */
Serial::Serial(const char* devname, uint32_t baudrate, EnBlockingMode blockmode, EnFlowControl flowcontrol, EnCharLen charlen, EnParity parity, EnStopBits stopbits)
  : handle_(-1), prev_tio_()
{
    if (devname == nullptr)
        throw std::invalid_argument("device name is null");
    auto baud_code = getBaudCode(baudrate);             // Obtener el código de baudrate correspondiente
    if (baud_code == 0)
        throw std::invalid_argument("");

    handle_ = ::open(devname, O_RDWR | O_NOCTTY | blockmode);
    if (handle_ < 0)                                    // Error de apertura
        throw std::ios_base::failure("Error in open: "s + std::string(strerror(errno)));

    if (tcgetattr(handle_, &prev_tio_) < 0)             // Guardar la anterior configuración del terminal
        throw std::ios_base::failure("Error in tcgetattr: " + std::string(strerror(errno)));

    termios tio = {};                                   // Establecer nuevos parámetros del puerto
    tio.c_iflag = (flowcontrol & (IXON | IXOFF)) | IGNPAR | (parity)? INPCK : 0;
    tio.c_oflag = 0;
    tio.c_cflag = (charlen & CSIZE) | (stopbits & CSTOPB) | CLOCAL | CREAD | (parity? (PARENB | (parity & PARODD)) : 0) | (flowcontrol & CRTSCTS);
    tio.c_lflag = 0;
    tio.c_cc[VTIME] = 0;                                // Timeout por omisión desactivado
    tio.c_cc[VMIN]  = 1;                                // Mínimo de caracteres a leer
    cfsetospeed(&tio, baud_code);
    cfsetispeed(&tio, baud_code);
    if (tcflush(handle_, TCIFLUSH) < 0 || tcsetattr(handle_, TCSANOW, &tio) < 0)
        throw std::ios_base::failure("Error in tcflush / tcsetattr: " + std::string(strerror(errno)));
}

/**
 * @brief   Destructor de la clase.
 * @desc    Cierra el puerto y lo devuelve a su configuración anterior.
 */
Serial::~Serial()
{
    tcsetattr(handle_, TCSANOW, &prev_tio_);
    ::close(handle_);
}

/**
 * @brief     Lectura del puerto serie
 */
ssize_t Serial::read(void* buf, std::size_t size, uint32_t t_out)
{
    if (buf == nullptr)
        throw std::invalid_argument("read: null pointer");

    ssize_t rt = 0;
    if (t_out == NO_TIMEOUT)                            // Lectura sin timeout - si Blocking, espera indefinidamente; si NonBlocking, sale al momento.
    {
        rt = ::read(handle_, buf, size);
        if (rt < 0 && errno == EAGAIN)                  // Lectura no bloqueante: esto no es un error, es que no hay nada que leer
            return 0;
        return rt;
    }

    pollfd p_list;
    p_list.fd = handle_;
    p_list.events = POLLIN;
    int err;
    auto end_time = std::chrono::system_clock::now() + std::chrono::milliseconds(t_out);
    uint8_t* ptr = reinterpret_cast<uint8_t*>(buf);     // Necesario para poder hacer aritmética de punteros
    for (rt = 0; rt < static_cast<ssize_t>(size); )
    {
        do
        {
            auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - std::chrono::system_clock::now()).count();
            err = (remain > 0)? poll(&p_list, 1, remain) : 0; // Si quedan 0 o menos milisegundos, salir con err = 0 (timeout)
        } while (err < 0 && errno == EINTR);            // Continuar a la espera si se recibe EINTR
        if (err <= 0)                                   // Salida con error o timeout
            break;
        err = ::read(handle_, &ptr[rt], size - rt);     // Se supone que esto leerá algo...
        if (err < 0)                                    // Error de lectura
            break;
        rt += err;
    }
    return rt;
}

/**
 * @brief     Escritura al puerto serie
 */
ssize_t Serial::write(const void* buf, std::size_t size)
{
    if (buf == nullptr)
        throw std::invalid_argument("write: null pointer");
    ssize_t bytes = ::write(handle_, buf, size);
    return (bytes >= 0)? bytes : -1;
}

/**
 * @brief     Escritura al puerto serie de un sólo byte
 */
bool Serial::writeByte(uint8_t byte)
{
    return (::write(handle_, &byte, 1) == 1);
}

/**
 * @brief     Comprueba si hay datos pendientes de enviar
 */
int Serial::pendingWrite()
{
    int pending, lsr;
    if (ioctl(handle_, TIOCSERGETLSR, &lsr) < 0)        // lectura de line status register
        return PENDING_ERROR;
    if (lsr & TIOCSER_TEMT)                             // FIFO y shift register vacíos
        return PENDING_EMPTY;
    if (ioctl(handle_, TIOCOUTQ, &pending) < 0)
        return PENDING_ERROR;
    return pending? pending : 1;
}

/**
 * @brief     Comprueba si hay datos pendientes de leer
 */
int Serial::pendingRead()
{
    int pending;
    if (ioctl(handle_, TIOCINQ, &pending) < 0)
        return PENDING_ERROR;
    return pending;
}

/**
 * @brief     Espera hasta que se hayan enviado todos los datos
 */
bool Serial::waitSend()
{
    while (true)
        switch (pendingWrite())
        {
            case PENDING_ERROR:
                return false;
            case PENDING_EMPTY:
                return true;
            default:
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
}

/**
 * @brief     Vaciar el buffer de entrada, de salida, o ambos
 */
void Serial::clearBuffer(EnClearOper operation)
{
    switch (operation)
    {
      case CLEAR_BUF_IN:                                // Borrar buffer de entrada
        tcflush(handle_, TCIFLUSH);
        break;
      case CLEAR_BUF_OUT:                               // Borrar buffer de salida
        tcflush(handle_, TCOFLUSH);
        break;
      case FLUSH_BUF_OUT:                               // Esperar vaciado del buffer de salida
        tcdrain(handle_);                               // Esta función vuelve cuando el kernel ha volcado los datos a la UART
        break;
    }
}

/**
 * @brief     Establecer el estado de las líneas del puerto serie
 */
bool Serial::setLine(SerialLine line, bool mode)
{
    unsigned flags;
    if (ioctl(handle_, TIOCMGET, &flags) < 0)
        return false;

    if (mode)
        flags |= line;
    else
        flags &= ~line;
    return (ioctl(handle_, TIOCMSET, &flags) >= 0);
}

/**
 * @brief   Recuperar el estado de las líneas del puerto serie
 */
int Serial::getLine(SerialLine line)
{
    unsigned flags;
    if (ioctl(handle_, TIOCMGET, &flags) < 0)
        return -1;
    if (flags & line)
        return 1;
    return 0;
}
/**
 * @brief   Establecer el modo de lectura (bloqueante o no bloqueante)
 */
bool Serial::setBlocking (EnBlockingMode mode)
{
    int flags = fcntl(handle_, F_GETFL, 0);
    if (flags == -1)
        return false;
    if (mode)
        flags |= O_NDELAY;
    else
        flags &= ~O_NDELAY;
    return (fcntl(handle_, F_SETFL, flags) >= 0);
}

/**------------------------------------------
 * @brief     Funciones privadas
 * ------ */

/**
 * @brief     Conversión del parámetro de velocidad del puerto de entero a constante válida para Init.
 */
tcflag_t Serial::getBaudCode(uint32_t baudrate)
{
    static const Uint2Tcflag uint2tcflag[] =            //!< Tabla de equivalencias de flags y valores de bps
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
