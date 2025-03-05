/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      libserial.h
 * @brief     Clase de manejo del puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.05
 * @version   2.1
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#ifndef __LIBSERIAL_H__
#define __LIBSERIAL_H__

#include <termios.h>
#include <stdint.h>
#include <sys/types.h>                                  // size_t, ssize_t
#include <sys/ioctl.h>                                  // Constantes indicativas de las líneas serial
#include <fcntl.h>                                      // O_NDELAY
#include <string>                                       // std::string

namespace libserial {

/**
 * @brief   Identificador de la versión de la biblioteca
 */
const char* version();

}

/**-------------------------------------------------------------------------------------------------
 * @brief   Clase Serial: Manejo del puerto serie
 * ------ */
class Serial
{
public:
    /**-------------------------------------------------------------------------------------------------
     * @brief   Tipos enumerados públicos, utilizados por las funciones de la clase
     * ------ */

    /** @brief  Enumeración de la modalidad de lectura (bloqueante o no bloqueante) (Open, SetBlocking) */
    enum EnBlockingMode
    {
        Blocking          = 0,                            //!< Las lecturas son bloqueantes (esperan a que haya caracteres que leer)
        NonBlocking       = O_NDELAY                      //!< Las lecturas no son bloqueantes (salen con error si no hay nada que leer)
    };

    /** @brief  Enumeración de los tipos de control de flujo (Open) */
    enum EnFlowControl
    {
        NoFlowCtrl        = 0,
        HardwareFlowCtrl  = CRTSCTS,
        XonXoffInput      = IXOFF,
        XonXoffOutput     = IXON,
        XonXoffBoth       = (IXON | IXOFF)
    };

    /** @brief  Enumeración de los tamaños de carácter (Open) */
    enum EnCharLen
    {
        c5bits            = CS5,
        c6bits            = CS6,
        c7bits            = CS7,
        c8bits            = CS8
    };

    /** @brief  Enumeración de los tipos de paridad (Open) */
    enum EnParity
    {
        NoParity          = 0,
        EvenParity        = PARENB,
        OddParity         = PARODD
    };

    /** @brief  Enumeración de los bits de stop (Open) */
    enum EnStopBits
    {
        stop1bit          = 0,
        stop2bits         = CSTOP
    };

    /** @brief  Estados del buffer (PendingRead, PendingWrite) */
    enum EnPending : int
    {
        PENDING_ERROR = -1,                               //!< Error: no se ha podido averiguar el estado
        PENDING_EMPTY = 0,                                //!< No hay datos pendientes de transmitir (buffer vacío)
    };

    /** @brief  Tipos de operación de vaciado de buffer (ClearBuffer) */
    enum EnClearOper
    {
        CLEAR_BUF_IN = 1,                                 //!< Vaciar el buffer de entrada (descartar datos pendientes)
        CLEAR_BUF_OUT,                                    //!< Vaciar el buffer de salida descartando datos pendientes
        FLUSH_BUF_OUT                                     //!< Esperar a que se terminen de enviar los datos pendientes
    };

    /** @brief  Enumeración de las líneas del puerto serie (SetLine, GetLine) */
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

    /** @brief  Valores específicos de timeout (Read) */
    enum
    {
        NO_TIMEOUT = 0                                    //!< Lectura sin timeout (@see read)
    };

    /**-------------------------------------------------------------------------------------------------
     * @brief   Funciones públicas
     * ------ */

    /**
     * @brief   Constructor. Abre y configura el puerto serie.
     * @desc    Abre el dispositivo y aplica la configuración solicitada, guardando la anterior.
     * @note    Explicación detallada de los parámetros en la descripción de los tipos enumerados.
     */
    Serial (
        const std::string& devname,                     /** @param devname     Path al dispositivo serie */
        uint32_t baudrate,                              /** @param baudrate    Velocidad (bits por segundo) a establecer */
        EnBlockingMode blockmode = NonBlocking,         /** @param blockmode   Modo de bloqueo en lectura (bloqueante o no bloqueante) */
        EnFlowControl flowcontrol = NoFlowCtrl,         /** @param flowcontrol Tipo de control de flujo [ = sin control de flujo ] */
        EnCharLen charlen = c8bits,                     /** @param charlen     Tamaño del carácter [ = 8  bits ] */
        EnParity parity = NoParity,                     /** @param parity      Control de paridad del carácter [ sin control de paridad ] */
        EnStopBits stopbits = stop1bit                  /** @param stopbits    Bits de stop [ = 1 bit de stop ] */
    );

    /**
     * @brief   Destructor de la clase.
     * @desc    Cierra el puerto y lo devuelve a su configuración anterior.
     */
    ~Serial ();

    /**
     * @brief   Lectura del puerto serie
     * @desc    En modo Blocking, NO_TIMEOUT espera indefinidamente la llegada de un byte.
     *          En modo NonBlocking, NO_TIMEOUT sale inmediatamente con retorno de 0 bytes si no hay datos que leer.
     * @throws  invalid_argument si buf es nullptr.
     */
    ssize_t                                             /** @return Bytes leidos */
    read (
        void* buf,                                      /** @param buf    Buffer en el que escribir lo leido */
        std::size_t size,                               /** @param size   Bytes a leer */
        uint32_t t_out                                  /** @param t_out  Timeout en ms, o NO_TIMEOUT. */
    );

    /**
     * @brief   Escritura al puerto serie
     * @throws  invalid_argument si buf es nullptr.
     */
    ssize_t                                             /** @return -1: error | Bytes escritos */
    write (
        const void* buf,                                /** @param buf   Buffer con los datos a escribir */
        std::size_t size                                /** @param size  Bytes a escribir */
    );

    /**
     * @brief   Escritura al puerto serie de un sólo byte
     */
    bool                                                /** @return true: escritura correcta | false: error */
    writeByte (
        uint8_t byte                                    /** @param byte  Byte a escribir */
    );

    /**
     * @brief   Comprueba si hay datos pendientes de enviar
     */
    int                                                 /** @return PENDING_ERROR: Error de lectura | PENDING_EMPTY: Cola de salida vacía | Bytes pendientes de enviar */
    pendingWrite();

    /**
     * @brief   Comprueba si hay datos pendientes de leer
     */
    int                                                 /** @return PENDING_ERROR: Error de lectura | PENDING_EMPTY: Cola de entrada vacía | Bytes pendientes de recibir */
    pendingRead();

    /**
     * @brief   Espera hasta que se hayan enviado todos los datos
     */
    bool                                                /** @return true: Operación completada con éxito | false: error */
    waitSend();

    /**
     * @brief   Vaciar el buffer de entrada, de salida, o ambos
     */
    void                                                /** @return void */
    clearBuffer (
        EnClearOper operation                           /** @param operation  Tipo de operación a realizar: espera o borrado i/o */
    );

    /**
     * @brief   Establecer el estado de las líneas del puerto serie
     */
    bool                                                /** @return true: Comando completado con éxito | false: error */
    setLine (
        SerialLine line,                                /** @param line  Línea a controlar */
        bool mode                                       /** @param mode  Valor a establecer (on/off) */
    );

    /**
     * @brief   Recuperar el estado de las líneas del puerto serie
     */
    int                                                 /** @return 1: línea activa | 0: línea inactiva | -1: error */
    getLine (
        SerialLine line                                 /** @param line  Línea a controlar */
    );

    /**
     * @brief   Establecer el modo de lectura (bloqueante o no bloqueante)
     */
    bool                                                /** @return true: Comando completado con éxito | false: error */
    setBlocking (
        EnBlockingMode mode                             /** @param mode  Nuevo modo de lectura */
    );

protected:
    /**-------------------------------------------------------------------------------------------------
     * @brief   Estructuras, datos y funciones privados
     * ------ */

    /**
     * @brief   Estructura para asociar valores de baudrate con flags del sistema
     */
    struct Uint2Tcflag
    {
        uint32_t baud;                                  // Baudrate
        tcflag_t flag;                                  // Flag del sistema
    };

    /**
     * @brief   Conversión del parámetro de velocidad del puerto de entero a constante para termios
     */
    tcflag_t                                            /** @return Constante correspondiente o la inmediata inferior */
    getBaudCode (
        uint32_t baudrate                               /** @param baudrate  Valor de velocidad requerido */
    );

    int     handle_;                                    //!< File handler
    termios prev_tio_;                                  //!< Configuración anterior del puerto, para restaurarla al cerrar
};

#endif // __LIBSERIAL_H__
