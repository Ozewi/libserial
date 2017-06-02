/** libserial
 * Módulo libserial: Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.h
 * @brief     Clase de manejo del puerto serie
 * @author    Íñigo López-Barranco Muñiz
 * @author    José Luis Sánchez
 * @author    David Serrano
 * @date      2016.06.03
 * @version   1.3.3
 *
 * Copyright (c) 2005-2016 José Luis Sánchez Arroyo
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
 * @brief   Clase Serial: Manejo del puerto serie
 * ------ */
class Serial
{
public:

  /**
   * @brief   Funciones PendingRead y PendingWrite, valor de retorno: situaciones del buffer
   */
  enum EnPending
  {
    PENDING_ERROR,                      //!< Error: no se ha podido averiguar el estado
    PENDING_EMPTY,                      //!< No hay datos pendientes de transmitir (buffer vacío)
    PENDING_PENDING                     //!< Hay datos pendientes de transmitir
  };

  /**
   * @brief   Función ClearBuffer, parámetro operation: tipos de operación de vaciado de buffer
   */
  enum EnClearOper
  {
    CLEAR_BUF_IN = 1,                   //!< Vaciar el buffer de entrada (descartar datos pendientes)
    CLEAR_BUF_OUT,                      //!< Vaciar el buffer de salida descartando datos pendientes
    FLUSH_BUF_OUT                       //!< Esperar a que se terminen de enviar los datos pendientes
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
   * @brief   Enumeración de los tipos de control de flujo
   */
  enum EnFlowControl
  {
    NoFlowCtrl        = 0,
    HarwareFlowCtrl   = CRTSCTS,
    XonXoffInput      = IXOFF,
    XonXoffOutput     = IXON,
    XonXoffBoth       = (IXON | IXOFF)
  };

  /**
   * @brief   Enumeración de los tamaños de carácter
   */
  enum EnCharLen
  {
    c5bits            = CS5,
    c6bits            = CS6,
    c7bits            = CS7,
    c8bits            = CS8
  };

  /**
   * @brief   Enumeración de los tipos de paridad
   */
  enum EnParity
  {
    NoParity          = 0,
    EvenParity        = PARENB,
    OddParity         = PARODD
  };

  /**
   * @brief   Enumeración de los bits de stop
   */
  enum EnStopBits
  {
    stop1bit          = 0,
    stop2bits         = CSTOP
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
  bool                                                  /** @return  true si todo fue bien, false si error. */
  Init (
    tcflag_t baudrate,                                  /** @param   Velocidad (bits por segundo) a establecer (B50 ... B4000000) */
    EnFlowControl flowcontrol = NoFlowCtrl,             /** @param   Tipo de control de flujo [ = sin control de flujo ] */
    EnCharLen charlen = c8bits,                         /** @param   Tamaño del carácter [ = 8  bits ] */
    EnParity parity = NoParity,                         /** @param   Control de paridad del carácter [ sin control de paridad ] */
    EnStopBits stopbits = stop1bit                      /** @param   Bits de stop (0: 1 stop bit, CSTOP: 2 stop bits) [ = 0 ] */
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
   * @brief   Comprueba si se transmitieron todos los datos por la UART serie
   */
  bool                                  /** @return  true si se tansmitieron datos, false en caso contrario */
  IsTxFIFOEmpty ();

  /**
   * @brief   Comprueba si hay datos pendientes de enviar
   */
  EnPending                             /** @return  Estado del buffer de envío - Véase enum EnPending */
  PendingWrite();

  /**
   * @brief   Comprueba si hay datos pendientes de leer
   */
  EnPending                             /** @return  Estado del buffer de recepción - Véase enum EnPending */
  PendingRead();

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
  static
  tcflag_t                              /** @return  Constante correspondiente o la inmediata superior; 0 si no es válida y se pidió estricta */
  GetBaudCode (
    uint32_t baudrate,                  /** @param   Valor de velocidad requerido */
    bool strict = false                 /** @param   ¿Requerida estrictamente la velocidad del parámetro? */
  );

  /**
   * @brief   Obtener el valor de velocidad correspondiente al flag de termios.
   */
  static
  uint32_t                              /** @return Valor correspondiente de bps, o 0 si el flag no es válido */
  GetBaudValue(
    tcflag_t p_flag                     /** @param  Flag a evaluar */
  );

protected:
  /**
   * @brief   Estructura para asociar valores de baudrate con flags del sistema
   */
  struct Uint2Tcflag
  {
    uint32_t baud;                      // Baudrate
    tcflag_t flag;                      // Flag del sistema
  };

  int      fd;                          //!< File descriptor
  termios* prev_tio;                    //!< Configuración anterior del puerto, para restaurarla al salir
  static Uint2Tcflag uint2tcflag[];     //!< Tabla de equivalencias de flags y valores de bps
};

#endif // __LIBSERIAL_H__
