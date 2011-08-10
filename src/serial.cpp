/** serial
 * Módulo serial: Manejo de comunicaciones por puerto serie
 *
 * @file      serial.cpp
 * @brief     Clase de manejo del puerto serie
 * @author    Íñigo López-Barranco Muñiz
 * @author    José Luis Sánchez
 * @author    David Serrano
 * @date      2008.07.04
 * @version   1.1.4
 *
 * Copyright (c) 2005-2008 Sepsa, S.A.
 * All rights reserved. Permission to use, modify or copy this software in whole or in part is
 * forbidden without prior, writen consent of the copyright holder.
 */

#include "serial.h"
#include <wtc/TimerClass.h>
#include <fcntl.h>
#include <string.h>
#include <sys/poll.h>
#include <errno.h>
#include <stdio.h>

/*--------------------------------------------------------------------------------------------------
  Constantes de la librería
------------------------------------------*/

#define LIBSERIAL_VERSION       "1.1.4"                   // Código de revisión de la librería

/*--------------------------------------------------------------------------------------------------
  Clase Serial: Manejo del puerto serie
    Funciones públicas
------------------------------------------*/
/** Constructor de la clase - Inicialización de variables y apertura del dispositivo
 *
 * @param   devname     Path al dispositivo serie
 * @return  -
 */
Serial::Serial(const char* devname)
{
  fd = -1;
  prev_tio = 0;
  if (devname)
    fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
  printf("Serial: device = %s, línea %d\n", devname, __LINE__);
}

/** Destructor de la clase - Devuelve el puerto serie a su configuración anterior
 *
 * @param   -
 * @return  -
 */
Serial::~Serial()
{
  if (fd >= 0 && prev_tio)
    tcsetattr(fd, TCSANOW, prev_tio);
  if (fd >= 0)  close (fd);
  if (prev_tio) delete prev_tio;
}

/** Inicialización del puerto serie
 * Inicializa el puerto serie con los parámetros indicados, guardando los que tenía anteriormente
 * para su recuperación en el destructor de la clase.
 * 
 * @param   baudrate    Velocidad (bits por segundo) a establecer (B50 ... B4000000)
 * @param   flowcontrol Tipo de control de flujo (0: No Flow Ctrl, CRTSCTS: Hardware FC, IXON: XON/XOFF (output), IXOFF: XON/XOFF (input) [ = 0 ]
 * @param   charlen     Tamaño del carácter (CS5: 5 bits, CS6: 6 bits, CS7: 7 bits, CS8: 8 bits) [ = CS8 ]
 * @param   parity      Tipo de paridad (0: No Parity, PARODD: Odd parity, PARENB: Even parity) [ = 0 ]
 * @param   stopbits    Bits de stop (0: 1 stop bit, CSTOP: 2 stop bits) [ = 0 ]
 * @return  true si todo fue bien, false si error.
 * @note    Los valores a utilizar son las constantes definidas en termios.h (ver man termios).
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

/** Lectura del puerto serie
 * 
 * @param   buf         Buffer en el que escribir lo leido
 * @param   size        Bytes a leer
 * @param   t_out       Timeout en ms
 * @return  Bytes leidos, -1 si error
 * @note    El modo de lectura (canónica, etc) depende de los parámetros de configuración pasados a Init.
 * @note    Ver http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html para más información.
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
  TimerClass timer;
  timer.setAlarm(t_out);
  for (rt = 0; rt < static_cast<ssize_t>(size); )
  {
    do
      err = poll(&p_list, 1, t_out);
    while (err < 0 && errno == EINTR);                    // Continuar a la espera si se recibe EINTR
    if (err <= 0 || timer.isExpired())                    // Salida con error o timeout
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

/** Escritura al puerto serie
 * 
 * @param   buf         Buffer con los datos a escribir
 * @param   size        Bytes a escribir
 * @return  Bytes escritos, -1 si error
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

/** Escritura al puerto serie de un sólo byte
 * 
 * @param   byte        Byte a escribir
 * @return  true si fue bien, false si error
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


/** Comprueba si se transmitieron todos los datos por la UART serie
 * 
 * @param  
 * @return  true si se tansmitieron, false en caso contrario
 */
bool Serial::IsTxFIFOEmpty()
{
  bool ret = false;
  int lsr = 0;
  if (ioctl(fd, TIOCSERGETLSR, &lsr) != -1)	  // lectura de line status register
  {
    if (lsr & TIOCSER_TEMT)		  	  // FIFO y shift register vacíos
      ret = true;
  }
  return ret;
}


/** Vaciar buffer de salida
 * 
 * @param   operation   Tipo de operación a realizar: espera o borrado i/o
 * @return  -
 */
void Serial::ClearBuffer(uint32_t operation)
{
  if (IsValid())
  {
    switch (operation)
    {
      case SER_CLR_BUF_IN:                                // Borrar buffer de entrada
        tcflush(fd, TCIFLUSH);
        break;
      case SER_CLR_BUF_OUT:                               // Borrar buffer de salida
        tcflush(fd, TCOFLUSH);
        break;
      case SER_WAIT_BUF_OUT:                              // Esperar vaciado del buffer de salida 
        tcdrain(fd);					  // El kernel ha volcado los datos a la UART
        break;
    }
  }
}

/** Establecer el estado de las líneas del puerto serie
 * 
 * @param   line        Línea a controlar
 * @param   mode        Valor a establecer (on/off)
 * @return  false si error
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

/** Recuperar el estado de las líneas del puerto serie
 * 
 * @param   line        Línea a controlar
 * @return  1 si la línea está activa, 0 si está inactiva, -1 si error
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

/** Conversión del parámetro de velocidad del puerto de entero a constante válida para Init.
 *
 * @param   baudrate    Valor de velocidad requerido
 * @param   strict      ¿Requerida estrictamente la velocidad del parámetro?
 * @return  Constante de velocidad adecuada a la requerida; 0 si no es válida y se pide estricta, o la inmediata superior si no.
 */
tcflag_t Serial::GetBaudCode(uint32_t baudrate, bool strict)
{
  struct Uint2Tcflag
  {
    uint32_t baud;
    tcflag_t flag;
  } uint2tcflag [] =
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
  for (unsigned i = 0; i < sizeof(uint2tcflag) / sizeof(uint2tcflag[0]); i++)
    if (uint2tcflag[i].baud == baudrate)
    {
      return uint2tcflag[i].flag;
    }
    else if (uint2tcflag[i].baud > baudrate)
    {
      if (strict)
        return 0;
      return uint2tcflag[i].flag;
    }
  return 0;
}

/** Identificación de la librería
 *
 * @param   -
 * @return  Código de revisión de la librería
 */
const char* Serial::GetVersion()
{
  return LIBSERIAL_VERSION;
}
