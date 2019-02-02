/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.cpp
 * @brief     Clase de manejo del puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2019.02.01
 * @version   1.5.0
 *
 * Copyright (c) 2005-2019 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <libUtility/timer.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <errno.h>

/**-------------------------------------------------------------------------------------------------
 * @brief   Clase Serial: Manejo del puerto serie
 * ------ */

/**
 * @brief   Constructor de la clase - Inicialización de variables
 */
Serial::Serial()
  : handle(-1), prev_tio()
{
}

/**
 * @brief   Destructor de la clase - Devuelve el puerto serie a su configuración anterior
 */
Serial::~Serial()
{
  Close();
}

/**
 * @brief     Apertura e inicialización del puerto serie
 */
bool Serial::Open(const char* devname, uint32_t baudrate, EnLockingMode lockmode, EnFlowControl flowcontrol, EnCharLen charlen, EnParity parity, EnStopBits stopbits)
{
  if (IsOpen() || devname == nullptr)                   // Ya estaba abierto o parámetro erróneo
    return false;

  tcflag_t baud_code = GetBaudCode(baudrate);           // Obtener el código de baudrate correspondiente
  if (baud_code == 0)
    return false;

  handle = open(devname, O_RDWR | O_NOCTTY | lockmode);
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
 * @brief   Cierre del puerto y restauración de la configuración anterior
 */
bool Serial::Close(bool flush)
{
  if (!IsOpen())
    return false;
  if (flush)
    tcdrain(handle);
  tcsetattr(handle, TCSANOW, &prev_tio);
  close(handle);
  handle = -1;
  return true;
}

/**
 * @brief   Lectura del puerto serie
 */
ssize_t Serial::Read(uint8_t* buf, size_t size, uint32_t t_out)
{
  if (!IsOpen() || buf == nullptr)
    return -1;

  ssize_t rt = 0;
  if (t_out == NO_TIMEOUT)                              // Lectura sin timeout - si Blocking, espera indefinidamente; si NonBlocking, sale al momento.
  {
    rt = read(handle, buf, size);
    if (rt < 0 && errno == EAGAIN)                      // Lectura no bloqueante: no ha habido un error, es que no hay nada que leer
      return 0;
    return rt;
  }

  pollfd p_list;
  p_list.fd = handle;
  p_list.events = POLLIN;
  int err;
  Timer timer;
  timer.SetAlarm(t_out);
  for (rt = 0; rt < static_cast<ssize_t>(size); )
  {
    do
      err = poll(&p_list, 1, t_out);
    while (err < 0 && errno == EINTR);                  // Continuar a la espera si se recibe EINTR
    if (err <= 0 || timer.IsExpired())                  // Salida con error o timeout
      break;
    err = read(handle, &buf[rt], size - rt);            // Se supone que esto leerá algo...
    if (err < 0)                                        // Error de lectura
      break;
    rt += err;
  }
  return rt;
}

/**
 * @brief   Escritura al puerto serie
 */
ssize_t Serial::Write(const uint8_t* buf, size_t size)
{
  if (!IsOpen() || buf == nullptr)
    return -1;
  ssize_t bytes = write(handle, buf, size);
  return (bytes >= 0)? bytes : -1;
}

/**
 * @brief   Escritura al puerto serie de un sólo byte
 */
bool Serial::WriteByte(uint8_t byte)
{
  if (!IsOpen())
    return false;
  return (write(handle, &byte, 1) == 1);
}

/**
 * @brief   Comprueba si hay datos pendientes de enviar
 */
int Serial::PendingWrite()
{
  int pending, lsr;
  if (ioctl(handle, TIOCSERGETLSR, &lsr) < 0)           // lectura de line status register
    return PENDING_ERROR;
  if (lsr & TIOCSER_TEMT)                               // FIFO y shift register vacíos
    return PENDING_EMPTY;
  if (ioctl(handle, TIOCOUTQ, &pending) < 0)
    return PENDING_ERROR;
  return pending;
}

/**
 * @brief   Comprueba si hay datos pendientes de leer
 */
int Serial::PendingRead()
{
  int pending;
  if (ioctl(handle, TIOCINQ, &pending) < 0)
    return PENDING_ERROR;
  return pending;
}

/**
 * @brief   Vaciar buffer de salida
 */
void Serial::ClearBuffer(EnClearOper operation)
{
  if (IsOpen())
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
 * @brief   Establecer el estado de las líneas del puerto serie
 */
bool Serial::SetLine(SerialLine line, bool mode)
{
  if (!IsOpen())
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
int Serial::GetLine(SerialLine line)
{
  if (!IsOpen())
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
bool Serial::SetBlocking (EnLockingMode mode)
{
  if (!IsOpen())
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
tcflag_t Serial::GetBaudCode(uint32_t baudrate)
{
  static Uint2Tcflag uint2tcflag[] =                    //!< Tabla de equivalencias de flags y valores de bps
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

  for (auto iter : uint2tcflag)
    if (iter.baud <= baudrate)
      return iter.flag;
  return 0;                                           // fallback
}
