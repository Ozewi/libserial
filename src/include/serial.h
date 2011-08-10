/** serial
 * Módulo serial: Manejo de comunicaciones por puerto serie
 *
 * @file      serial.h
 * @brief     Clase de manejo del puerto serie
 * @author    Íñigo López-Barranco Muñiz
 * @author    José Luis Sánchez
 * @date      2008.01.24
 * @version   1.1.2
 *
 * Copyright (c) 2005-2008 Sepsa, S.A.
 * All rights reserved. Permission to use, modify or copy this software in whole or in part is
 * forbidden without prior, writen consent of the copyright holder.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>

/**-------------------------------------------------------------------------------------------------
 * Constantes de la librería
 * ------------------------------------------
 */

/*--- Enumeración de las líneas del puerto serie ---*/

enum SerialLine
{
  LINE_LE   = TIOCM_LE,                                   ///< 
  LINE_DTR  = TIOCM_DTR,                                  ///< DTR (Data Terminal Ready)
  LINE_RTS  = TIOCM_RTS,                                  ///< RTS (Ready To Send)
  LINE_ST   = TIOCM_ST,                                   ///< ST (Serial Transmit)
  LINE_SR   = TIOCM_SR,                                   ///< SR (Serial Receive)
  LINE_CTS  = TIOCM_CTS,                                  ///< CTS (Clear To Send)
  LINE_CAR  = TIOCM_CAR,                                  ///< 
  LINE_RNG  = TIOCM_RNG,                                  ///< RNG (Ring)
  LINE_DSR  = TIOCM_DSR                                   ///< DSR (Data Send Ready)
};

/*--- Valores de los parámetros de la función Serial::ClearBuffers ---*/

#define SER_CLR_BUF_IN                  0x00000001        ///< Borrar buffer de entrada
#define SER_CLR_BUF_OUT                 0x00000002        ///< Borrar buffer de salida
#define SER_WAIT_BUF_OUT                0x00000003        ///< Esperar a que se vacíe el buffer de salida

/**-------------------------------------------------------------------------------------------------
 * Clase Serial: Manejo del puerto serie
 * ------------------------------------------
 */
class Serial
{
protected:
  int      fd;
  termios* prev_tio;
public:
                Serial     (const char* devname);
               ~Serial     ();
        bool    IsValid    ()  { return (fd >= 0)? true : false; };
        bool    Init       (tcflag_t baudrate, tcflag_t flowcontrol = 0, tcflag_t charlen = CS8, tcflag_t parity = 0, tcflag_t stopbits = 0);
        ssize_t Read       (uint8_t* buf, size_t size, uint32_t t_out);
        ssize_t Write      (const uint8_t* buf, size_t size);
        bool    WriteByte  (uint8_t byte);
        void    ClearBuffer(uint32_t operation);
        bool    SetLine    (SerialLine line, bool mode);
        int     GetLine    (SerialLine line);
        tcflag_t      GetBaudCode(uint32_t baudrate, bool strict = false);
        bool 	IsTxFIFOEmpty();
  static const char* GetVersion ();
};

#endif // __SERIAL_H__
