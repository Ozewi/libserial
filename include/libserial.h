/** libserial
 * Módulo libserial: Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.h
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

#ifndef __LIBSERIAL_H__
#define __LIBSERIAL_H__

#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>

/**-------------------------------------------------------------------------------------------------
 * Clase Serial: Manejo del puerto serie
 * ------------------------------------------
 */
class Serial
{
public:

  /**
   * @brief   Función ClearBuffer, parámetro operation: tipos de operación de vaciado de buffer
   */
  enum EnClearOper
  {
    CLEAR_BUF_IN = 1,
    CLEAR_BUF_OUT,
    FLUSH_BUF_OUT
  };

  /**
   * @brief   Enumeración de las líneas del puerto serie
   */
  enum SerialLine
  {
    LINE_LE   = TIOCM_LE,               //!< 
    LINE_DTR  = TIOCM_DTR,              //!< DTR (Data Terminal Ready)
    LINE_RTS  = TIOCM_RTS,              //!< RTS (Ready To Send)
    LINE_ST   = TIOCM_ST,               //!< ST (Serial Transmit)
    LINE_SR   = TIOCM_SR,               //!< SR (Serial Receive)
    LINE_CTS  = TIOCM_CTS,              //!< CTS (Clear To Send)
    LINE_CAR  = TIOCM_CAR,              //!< 
    LINE_RNG  = TIOCM_RNG,              //!< RNG (Ring)
    LINE_DSR  = TIOCM_DSR               //!< DSR (Data Send Ready)
  };

  /**
   * @brief   Constructor de la clase - Inicialización de variables y apertura del dispositivo
   */
  Serial (
    const char* devname                 /** @param   Path al dispositivo serie */
  );

  /**
   * @brief   Destructor de la clase - Devuelve el puerto serie a su configuración anterior
   */
  ~Serial ();

  /**
   * @brief   Determinar si el objeto es válido (está bien construido) o no
   * @note    El objeto es válido si el puerto se ha podido abrir
   */
  bool                                  /** @return  true si el objeto es válido, false en otro caso */
  IsValid () const
    { return bool(fd >= 0); };

  /**
   * @brief   Inicialización del puerto serie
   * @desc    Inicializa el puerto serie con los parámetros indicados, guardando los que tenía anteriormente
   *          para su recuperación en el destructor de la clase.
   * @note    Los valores a utilizar son las constantes definidas en termios.h (ver man termios).
   */
  bool                                  /** @return  true si todo fue bien, false si error. */
  Init (
    tcflag_t baudrate,                  /** @param   Velocidad (bits por segundo) a establecer (B50 ... B4000000) */
    tcflag_t flowcontrol = 0,           /** @param   Tipo de control de flujo (0: No Flow Ctrl, CRTSCTS: Hardware FC, IXON: XON/XOFF (output), IXOFF: XON/XOFF (input) [ = 0 ] */
    tcflag_t charlen = CS8,             /** @param   Tamaño del carácter (CS5: 5 bits, CS6: 6 bits, CS7: 7 bits, CS8: 8 bits) [ = CS8 ] */
    tcflag_t parity = 0,                /** @param   Tipo de paridad (0: No Parity, PARODD: Odd parity, PARENB: Even parity) [ = 0 ] */
    tcflag_t stopbits = 0               /** @param   Bits de stop (0: 1 stop bit, CSTOP: 2 stop bits) [ = 0 ] */
  );

  /**
   * @brief   Lectura del puerto serie
   * @note    El modo de lectura (canónica, etc) depende de los parámetros de configuración pasados a Init.
   * @note    Ver http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html para más información.
   */
  ssize_t                               /** @return  Bytes leidos, -1 si error */
  Read (
    uint8_t* buf,                       /** @param   Buffer en el que escribir lo leido */
    size_t size,                        /** @param   Bytes a leer */
    uint32_t t_out                      /** @param   Timeout en ms */
  );

  /**
   * @brief   Escritura al puerto serie
   */
  ssize_t                               /** @return  Bytes escritos, -1 si error */
  Write (
    const uint8_t* buf,                 /** @param   Buffer con los datos a escribir */
    size_t size                         /** @param   Bytes a escribir */
  );

  /**
   * @brief   Escritura al puerto serie de un sólo byte
   */
  bool                                  /** @return  true si fue bien, false si error */
  WriteByte (
    uint8_t byte                        /** @param   Byte a escribir */
  );

  /**
   * @brief   Comprueba si se transmitieron todos los datos por la UART serie[A
   */
  bool                                  /** @return  true si se tansmitieron datos, false en caso contrario */
  IsTxFIFOEmpty ();

  /**
   * @brief   Vaciar buffer de salida
   */
  void                                  /** @return  void */
  ClearBuffer (
    EnClearOper operation               /** @param   Tipo de operación a realizar: espera o borrado i/o */
  );

  /**
   * @brief   Establecer el estado de las líneas del puerto serie
   */
  bool                                  /** @return  false si error */
  SetLine (
    SerialLine line,                    /** @param   Línea a controlar */
    bool mode                           /** @param   Valor a establecer (on/off) */
  );

  /**
   * @brief   Recuperar el estado de las líneas del puerto serie
   */
  int                                   /** @return  1 si la línea está activa, 0 si está inactiva, -1 si error */
  GetLine (
    SerialLine line                     /** @param   Línea a controlar */
  );

  /**
   * @brief   Conversión del parámetro de velocidad del puerto de entero a constante válida para Init.
   */
  tcflag_t                              /** @return  Constante correspondiente o la inmediata superior; 0 si no es válida y se pidió estricta */
  GetBaudCode (
    uint32_t baudrate,                  /** @param   Valor de velocidad requerido */
    bool strict = false                 /** @param   ¿Requerida estrictamente la velocidad del parámetro? */
  );

  /**
   * @brief   Identificación de la librería
   */
  static
  const char*                           /** @return  Código de revisión de la librería */
  GetVersion ();

protected:
  int      fd;                          //!< File descriptor
  termios* prev_tio;                    //!< Configuración anterior del puerto, para restaurarla al salir

  /**
   * @brief   Estructura para asociar valores de baudrate con flags del sistema
   */
  struct Uint2Tcflag
  {
    uint32_t baud;                      // Baudrate
    tcflag_t flag;                      // Flag del sistema
  };
};

#endif // __LIBSERIAL_H__
