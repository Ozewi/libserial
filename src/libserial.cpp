/** libserial
 * Módulo libserial: Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.cpp
 * @brief     Clase de manejo del puerto serie
 * @author    Íñigo López-Barranco Muñiz
 * @author    José Luis Sánchez
 * @author    David Serrano
 * @date      2014.04.23
 * @version   1.2.0
 *
 * Copyright (c) 2005-2014 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <libUtility/timer.h>
#include <fcntl.h>
#include <string.h>
#include <sys/poll.h>
#include <errno.h>
#include <stdio.h>

/**------------------------------------------
 * @brief   Constantes generales
 * ------ */

#define LIBSERIAL_VERSION   "1.2.0"     //!< Código de revisión de la librería

/**-------------------------------------------------------------------------------------------------
 * @brief   Clase Serial - Funciones públicas
 * ------ */

/**
 * @brief   Constructor de la clase - Inicialización de variables y apertura del dispositivo
 */
Serial::Serial(const char* devname)
{
  fd = -1;
  prev_tio = 0;
  if (devname)
    fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
  printf("Serial: device = %s, línea %d\n", devname, __LINE__);
}

/**
 * @brief   Destructor de la clase - Devuelve el puerto serie a su configuración anterior
 */
Serial::~Serial()
{
  if (fd >= 0 && prev_tio)
    tcsetattr(fd, TCSANOW, prev_tio);
  if (fd >= 0)  close (fd);
  if (prev_tio) delete prev_tio;
}

/**
 * @brief   Inicialización del puerto serie
 */
bool Serial::Init(tcflag_t baudrate, tcflag_t flowcontrol, tcflag_t charlen, tcflag_t parity, tcflag_t stopbits)
{
  if (!IsValid())
    return false;
  if (!prev_tio)                                          // Se recoge la configuración anterior sólo la primera vez.
  {
    prev_tio = new termios;
    if (tcgetattr(fd, prev_tio) == -1)
      return false;                                       // No se puede obtener información del puerto
  }

  termios tio;                                            // Establecer nuevos parámetros del puerto
  memset(&tio, 0, sizeof(termios));

  tio.c_iflag = (flowcontrol & (IXON | IXOFF)) | IGNPAR | (parity)? INPCK : 0;
  tio.c_oflag = 0;
  tio.c_cflag = (baudrate & CBAUDEX) | (charlen & CSIZE) | (stopbits & CSTOPB) | CLOCAL | CREAD | (parity? (PARENB | (parity & PARODD)) : 0) | (flowcontrol & CRTSCTS);
  tio.c_lflag = 0;

  tio.c_cc[VTIME] = 0;                                    // Timeout por omisión desactivado
  tio.c_cc[VMIN]  = 1;                                    // Mínimo de caracteres a leer

  printf("Serial::Init: baudios = %d, línea %d\n", baudrate, __LINE__);
  cfsetospeed(&tio, baudrate);                            // POSIX way of life
  cfsetispeed(&tio, baudrate);                            // POSIX way of life

  if (tcflush(fd, TCIFLUSH) != -1 &&                      // Limpiar buffers y
      tcsetattr(fd, TCSANOW, &tio) != -1)                 // actualizar parámetros
  {
    tcgetattr(fd, &tio);
    return true;
  }
  return false;
}

/**
 * @brief   Lectura del puerto serie
 */
ssize_t Serial::Read(uint8_t* buf, size_t size, uint32_t t_out)
{
  if (!IsValid() || !buf)
    return -1;
  if (!t_out)                                             // Lectura sin timeout (tal vez bloqueante)
    return read(fd, buf, size);

  pollfd p_list;
  p_list.fd = fd;
  p_list.events = POLLIN;
  int err;
  ssize_t rt = 0;
  Timer timer;
  timer.SetAlarm(t_out);
  for (rt = 0; rt < static_cast<ssize_t>(size); )
  {
    do
      err = poll(&p_list, 1, t_out);
    while (err < 0 && errno == EINTR);                    // Continuar a la espera si se recibe EINTR
    if (err <= 0 || timer.IsExpired())                    // Salida con error o timeout
      break;
    err = read(fd, &buf[rt], size - rt);                  // Se supone que esto leerá algo...
    if (err < 0)                                          // Error de lectura
    {
      perror("Serial::Read: read");
      break;
    }
    rt += err;
  }
  return rt;
}

/**
 * @brief   Escritura al puerto serie
 */
ssize_t Serial::Write(const uint8_t* buf, size_t size)
{
  if (!IsValid() || !buf)
    return -1;
  ssize_t bytes = write(fd, buf, size);
  if (bytes < 0)
  {
    perror("Serial::Write: write");
    return -1;
  }
  return bytes;
}

/**
 * @brief   Escritura al puerto serie de un sólo byte
 */
bool Serial::WriteByte(uint8_t byte)
{
  if (!IsValid())
    return -1;
  if (write(fd, &byte, 1) == 1)
    return true;
  perror("Serial::WriteByte");
  return false;
}

/**
 * @brief   Comprueba si se transmitieron todos los datos por la UART serie
 */
bool Serial::IsTxFIFOEmpty()
{
  bool ret = false;
  int lsr = 0;
  if (ioctl(fd, TIOCSERGETLSR, &lsr) != -1)	        // lectura de line status register
  {
    if (lsr & TIOCSER_TEMT)		  	        // FIFO y shift register vacíos
      ret = true;
  }
  return ret;
}

/**
 * @brief   Vaciar buffer de salida
 */
void Serial::ClearBuffer(EnClearOper operation)
{
  if (IsValid())
  {
    switch (operation)
    {
      case CLEAR_BUF_IN:                                // Borrar buffer de entrada
        tcflush(fd, TCIFLUSH);
        break;
      case CLEAR_BUF_OUT:                               // Borrar buffer de salida
        tcflush(fd, TCOFLUSH);
        break;
      case FLUSH_BUF_OUT:                               // Esperar vaciado del buffer de salida
        tcdrain(fd);					// El kernel ha volcado los datos a la UART
        break;
    }
  }
}

/**
 * @brief   Establecer el estado de las líneas del puerto serie
 */
bool Serial::SetLine(SerialLine line, bool mode)
{
  if (!IsValid())
    return false;
  unsigned flags;
  if (ioctl(fd, TIOCMGET, &flags) < 0)
  {
    perror("Serial::SetLine: ioctl (get)");
    return false;
  }

  if (mode)
    flags |= line;
  else
    flags &= ~line;
  if (ioctl(fd, TIOCMSET, &flags) < 0)
  {
    perror("Serial::SetLine: ioctl (set)");
    return false;
  }
  return true;
}

/**
 * @brief   Recuperar el estado de las líneas del puerto serie
 */
int Serial::GetLine(SerialLine line)
{
  if (!IsValid())
    return -1;
  unsigned flags;
  if (ioctl(fd, TIOCMGET, &flags) < 0)
  {
    perror("Serial::GetLine: ioctl (get)");
    return -1;
  }
  if (flags & line)
    return 1;
  return 0;
}

/**
 * @brief   Conversión del parámetro de velocidad del puerto de entero a constante válida para Init.
 */
tcflag_t Serial::GetBaudCode(uint32_t baudrate, bool strict)
{
  static Uint2Tcflag uint2tcflag [] =
  {
    {     50, B50 },
    {     75, B75 },
    {    110, B110 },
    {    134, B134 },
    {    150, B150 },
    {    200, B200 },
    {    300, B300 },
    {    600, B600 },
    {   1200, B1200 },
    {   1800, B1800 },
    {   2400, B2400 },
    {   4800, B4800 },
    {   9600, B9600 },
    {  19200, B19200 },
    {  38400, B38400 },
    {  57600, B57600 },
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 },
    { 500000, B500000 }
  };

  tcflag_t rt = 0;

  for (unsigned i = 0; i < sizeof(uint2tcflag) / sizeof(uint2tcflag[0]) && rt == 0; ++i)
    if (uint2tcflag[i].baud == baudrate)
      rt = uint2tcflag[i].flag;
    else if (uint2tcflag[i].baud > baudrate)
    {
      if (strict)                                       // i.e. return 0
        break;
      rt = uint2tcflag[i].flag;
    }
  return rt;
}

/**
 * @brief   Identificación de la librería
 */
const char* Serial::GetVersion()
{
  return LIBSERIAL_VERSION;
}
