/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.cpp
 * @brief     Clase de manejo del puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.03
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
#include <chrono>               // std::chrono
#include <thread>               // std::this_thread::sleep_for

/**-------------------------------------------------------------------------------------------------
 * @brief   Clase Serial: Manejo del puerto serie
 * ------ */

/**
 * @brief     Constructor de la clase - Inicialización de variables
 */
Serial::Serial()
  : handle(-1), prev_tio()
{
}

/**
 * @brief     Destructor de la clase - Devuelve el puerto serie a su configuración anterior
 */
Serial::~Serial()
{
  close();
}

/**
 * @brief     Apertura e inicialización del puerto serie
 */
bool Serial::open(const char* devname, uint32_t baudrate, EnLockingMode lockmode, EnFlowControl flowcontrol, EnCharLen charlen, EnParity parity, EnStopBits stopbits)
{
  if (isOpen() || devname == nullptr)                   // Ya estaba abierto o parámetro erróneo
    return false;

  tcflag_t baud_code = getBaudCode(baudrate);           // Obtener el código de baudrate correspondiente
  if (baud_code == 0)
    return false;

  handle = ::open(devname, O_RDWR | O_NOCTTY | lockmode);
  if (handle < 0)                                       // Error de apertura
    return false;

  if (tcgetattr(handle, &prev_tio) < 0)                 // Guardar la anterior configuración del terminal
    return false;

  termios tio = {};                                     // Establecer nuevos parámetros del puerto
  tio.c_iflag = (flowcontrol & (IXON | IXOFF)) | IGNPAR | (parity)? INPCK : 0;
  tio.c_oflag = 0;
  tio.c_cflag = (charlen & CSIZE) | (stopbits & CSTOPB) | CLOCAL | CREAD | (parity? (PARENB | (parity & PARODD)) : 0) | (flowcontrol & CRTSCTS);
  tio.c_lflag = 0;
  tio.c_cc[VTIME] = 0;                                  // Timeout por omisión desactivado
  tio.c_cc[VMIN]  = 1;                                  // Mínimo de caracteres a leer
  cfsetospeed(&tio, baud_code);
  cfsetispeed(&tio, baud_code);
  if (tcflush(handle, TCIFLUSH) < 0 || tcsetattr(handle, TCSANOW, &tio) < 0)
    return false;
  return true;
}

/**
 * @brief     Cierre del puerto y restauración de la configuración anterior
 */
bool Serial::close(bool flush)
{
  if (!isOpen())
    return false;
  if (flush)
    tcdrain(handle);
  tcsetattr(handle, TCSANOW, &prev_tio);
  ::close(handle);
  handle = -1;
  return true;
}

/**
 * @brief     Lectura del puerto serie
 */
ssize_t Serial::read(void* buf, size_t size, uint32_t t_out)
{
  if (!isOpen() || buf == nullptr)
    return -1;

  ssize_t rt = 0;
  if (t_out == NO_TIMEOUT)                              // Lectura sin timeout - si Blocking, espera indefinidamente; si NonBlocking, sale al momento.
  {
    rt = ::read(handle, buf, size);
    if (rt < 0 && errno == EAGAIN)                      // Lectura no bloqueante: esto no es un error, es que no hay nada que leer
      return 0;
    return rt;
  }

  pollfd p_list;
  p_list.fd = handle;
  p_list.events = POLLIN;
  int err;
  auto end_time = std::chrono::system_clock::now() + std::chrono::milliseconds(t_out);
  uint8_t* ptr = reinterpret_cast<uint8_t*>(buf);       // Necesario para poder hacer aritmética de punteros
  for (rt = 0; rt < static_cast<ssize_t>(size); )
  {
    do
    {
      auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - std::chrono::system_clock::now()).count();
      err = (remain > 0)? poll(&p_list, 1, remain) : 0; // Si quedan 0 o menos milisegundos, salir con err = 0 (timeout)
    } while (err < 0 && errno == EINTR);                // Continuar a la espera si se recibe EINTR
    if (err <= 0)                                       // Salida con error o timeout
      break;
    err = ::read(handle, &ptr[rt], size - rt);          // Se supone que esto leerá algo...
    if (err < 0)                                        // Error de lectura
      break;
    rt += err;
  }
  return rt;
}

/**
 * @brief     Escritura al puerto serie
 */
ssize_t Serial::write(const void* buf, size_t size)
{
  if (!isOpen() || buf == nullptr)
    return -1;
  ssize_t bytes = ::write(handle, buf, size);
  return (bytes >= 0)? bytes : -1;
}

/**
 * @brief     Escritura al puerto serie de un sólo byte
 */
bool Serial::writeByte(uint8_t byte)
{
  if (!isOpen())
    return false;
  return (::write(handle, &byte, 1) == 1);
}

/**
 * @brief     Comprueba si hay datos pendientes de enviar
 */
int Serial::pendingWrite()
{
  int pending, lsr;
  if (ioctl(handle, TIOCSERGETLSR, &lsr) < 0)           // lectura de line status register
    return PENDING_ERROR;
  if (lsr & TIOCSER_TEMT)                               // FIFO y shift register vacíos
    return PENDING_EMPTY;
  if (ioctl(handle, TIOCOUTQ, &pending) < 0)
    return PENDING_ERROR;
  return pending? pending : 1;
}

/**
 * @brief     Comprueba si hay datos pendientes de leer
 */
int Serial::pendingRead()
{
  int pending;
  if (ioctl(handle, TIOCINQ, &pending) < 0)
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
  if (isOpen())
  {
    switch (operation)
    {
      case CLEAR_BUF_IN:                                // Borrar buffer de entrada
        tcflush(handle, TCIFLUSH);
        break;
      case CLEAR_BUF_OUT:                               // Borrar buffer de salida
        tcflush(handle, TCOFLUSH);
        break;
      case FLUSH_BUF_OUT:                               // Esperar vaciado del buffer de salida
        tcdrain(handle);                                // El kernel ha volcado los datos a la UART
        break;
    }
  }
}

/**
 * @brief     Establecer el estado de las líneas del puerto serie
 */
bool Serial::setLine(SerialLine line, bool mode)
{
  if (!isOpen())
    return false;
  unsigned flags;
  if (ioctl(handle, TIOCMGET, &flags) < 0)
    return false;

  if (mode)
    flags |= line;
  else
    flags &= ~line;
  return (ioctl(handle, TIOCMSET, &flags) >= 0);
}

/**
 * @brief   Recuperar el estado de las líneas del puerto serie
 */
int Serial::getLine(SerialLine line)
{
  if (!isOpen())
    return -1;
  unsigned flags;
  if (ioctl(handle, TIOCMGET, &flags) < 0)
    return -1;
  if (flags & line)
    return 1;
  return 0;
}
/**
 * @brief   Establecer el modo de lectura (bloqueante o no bloqueante)
 */
bool Serial::setBlocking (EnLockingMode mode)
{
  if (!isOpen())
    return false;
  int flags = fcntl(handle, F_GETFL, 0);
  if (flags == -1)
    return false;
  if (mode)
    flags |= O_NDELAY;
  else
    flags &= ~O_NDELAY;
  return (fcntl(handle, F_SETFL, flags) >= 0);
}

/**------------------------------------------
 * @brief     Funciones privadas
 * ------ */

/**
 * @brief     Conversión del parámetro de velocidad del puerto de entero a constante válida para Init.
 */
tcflag_t Serial::getBaudCode(uint32_t baudrate)
{
  static const Uint2Tcflag uint2tcflag[] =              //!< Tabla de equivalencias de flags y valores de bps
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
