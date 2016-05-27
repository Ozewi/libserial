/** libserial
 * Módulo libserial: Manejo de comunicaciones por puerto serie
 *
 * @file      test_libserial.cpp
 * @brief     Pruebas unitarias
 * @author    José Luis Sánchez
 * @date      2016.05.27
 * @version   1.3.1
 *
 * Copyright (c) 2005-2016 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <libUtility/tracelog.h>
#include <libUtility/versioninfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

MainVersion(test_libserial, 1.3.1)              //!< Nombre y versión del programa

/**--------------------------------------------------------------------------------------------------
 * @brief   Constantes de configuración
 * ------ */
const char* DEFAULT_SERIAL_DEV = "/dev/ttyS0";  //!< Dispositivo serie por omisión
uint32_t    DEFAULT_SERIAL_BPS = 9600;          //!< Velocidad de conexión por omisión
uint32_t    DEFAULT_SERIAL_TIMEOUT = 1000;      //!< Tiempo de espera máximo por omisión
uint32_t    MAX_MESSAGE_LEN = 512;              //!< Longitud máxima del mensaje en bytes

/**--------------------------------------------------------------------------------------------------
 * @brief   Salida con mensaje de sintaxis
 * ------ */
void Abort(const char* prog)
{
  std::cout << prog << " : Pruebas de transferencia de datos por puerto serie\n"
    "Sintaxis: " << prog << " -h | [-d <device>] [-s <speed> [-t <timeout>] <mensaje>\n"
    "  -h : Esta ayuda\n"
    "  -d : Dispositivo de conexión [/dev/ttyS0]\n"
    "       <device>  : Path del dispositivo serie a utilizar\n"
    "  -s : Velocidad de conexión [9600]\n"
    "       <speed>   : Velocidad de conexión requerida (9600, 19200, 36400...)\n"
    "  -t : Tiempo máximo de espera en lectura\n"
    "       <timeout> : Tiempo de espera, en milisegundos\n"
    "mensaje: secuencia de bytes a enviar por el puerto (pares de dígitos hexadecimales separados por espacio)\n"
    ;
  exit(-1);
}

/*--------------------------------------------------------------------------------------------------
 * @brief main
 * ------*/
int main(int argc, char* argv[])
{
  VersionInfo::Show();

  if (argc < 2)
    Abort(argv[0]);

  /*--- Procesamiento de línea de comandos ---*/
  const char* serial_dev = DEFAULT_SERIAL_DEV;
  uint32_t serial_bps = DEFAULT_SERIAL_BPS;
  uint32_t serial_timeout = DEFAULT_SERIAL_TIMEOUT;
  uint8_t message[MAX_MESSAGE_LEN];
  size_t msg_len = 0;

  for (int param = 1; param < argc; param++)
  {
    if (argv[param][0] == '-')
    {
      switch (argv[param][1])                             // Parámetros que empiezan por '-'
      {
        case 'h':
          Abort(argv[0]);
          break;                                          // no hace falta...
          
        case 'd':                                         // Dispositivo de conexión (puerto serie)
          if (param + 1 < argc)
            serial_dev = argv[++param];
          break;

        case 's':                                         // Velocidad de conexión
          if (param + 1 < argc)
            serial_bps = strtoul(argv[++param], 0, 0);
          break;

        case 't':                                         // Timeout de lectura en milisegundos
          if (param + 1 < argc)
            serial_timeout = strtoul(argv[++param], 0, 0);
          break;
          
        default:                                          // wrong
          Logger(Log::Error) << "Parámetro incorrecto: '" << argv[param] << "'" << Log::end;
          break;
      }
    }
    else
    {
      if (msg_len >= MAX_MESSAGE_LEN)
        Logger(Log::Error) << "Mensaje demasiado largo. Solamente se admiten " << MAX_MESSAGE_LEN << " bytes." << Log::end;
      uint32_t byte = strtoul(argv[param], 0, 16);
      if (byte > 0xff)
        Logger(Log::Error) << "Byte incorrecto: '" << argv[param] << "'" << Log::end;
      message[msg_len++] = byte;
    }
  }
  std::cout << "Parámetros: device='" << serial_dev << "'  speed=" << serial_bps << "  timeout=" << serial_timeout << std::endl
            << "Mensaje   : ";
  for (unsigned ix = 0; ix < msg_len; ++ix)
    printf("%02X ", message[ix]);
  std::cout << std::endl;

  /*--- Apertura del puerto serie ---*/
  Serial port(serial_dev);
  if (port.Init(port.GetBaudCode(serial_bps)) == false)
    Logger(Log::Error) << "Error abriendo / configurando el puerto serie" << Log::end;

  /*--- Enviar y recibir mensaje ---*/
  puts("Enviando mensaje...");
  if (port.Write(message, msg_len) == false)
    Logger(Log::Error) << "Error durante el envío." << Log::end;

  std::cout << "Mensaje enviado. Esperando respuesta..." << std::endl;

  while (1)
  {
    uint8_t byte;
    if (port.Read(&byte, 1, serial_timeout) == false)  // timeout en lectura. Terminar.
      break;
    printf("%02X ", byte);
    fflush(stdout);
  }
  std::cout << std::endl << "Timeout." << std::endl;
}
