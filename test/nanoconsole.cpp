/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      nanoconsole.cpp
 * @brief     Miniconsola de puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.03
 * @version   2.0
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <poll.h>
#include <libUtility/tracelog.h>
#include <libUtility/versioninfo.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <unistd.h>

MainVersion(nanoconsole, 2.0)                           //!< Nombre y versión del programa

/**--------------------------------------------------------------------------------------------------
 * @brief       Constantes de configuración
 * ------ */
const char* DEFAULT_SERIAL_DEV = "/dev/ttyS0";          //!< Dispositivo serie por omisión
uint32_t    DEFAULT_SERIAL_BPS = 9600;                  //!< Velocidad de conexión por omisión

/**--------------------------------------------------------------------------------------------------
 * @brief       Leer el puerto y copiar en stdout
 * ------*/
void PortReader(Serial& port)
{
    char byte;
    for (bool loop = true; loop; )
    {
        switch (port.pendingRead())
        {
            case Serial::PENDING_ERROR:
                Logger(Log::Error) << "Error leyendo del puerto serie. Terminando." << Log::end;
                loop = false;
                break;

            case Serial::PENDING_EMPTY:
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                break;

            default:
                while (port.read(&byte, 1, Serial::NO_TIMEOUT))
                    std::cout << byte;
                fflush(stdout);
        }
    }
}

/**--------------------------------------------------------------------------------------------------
 * @brief       Leer el teclado y copiar al puerto y, opcionalmente, a stdout
 * ------*/
void PortWriter(Serial* port, bool local_echo)
{
    pollfd poll_entry = { fileno(stdin), POLLIN, 0 };

    while (true)
    {
        auto rc = poll(&poll_entry, 1, 1000);
        if (rc < 0)
        {
            Logger(Log::Error) << "Error leyendo de consola -- errno: " << errno << ". Terminando." << Log::end;
            break;
        }
        if (rc > 0)
        {
            uint8_t byte;
            if (read(fileno(stdin), &byte, 1) > 0)
            {
                if (local_echo)
                {
                    std::cout << byte;
                    fflush(stdout);
                }
                port->writeByte(byte);
            }
        }
    }
}

/**--------------------------------------------------------------------------------------------------
 * @brief       Salida con mensaje de sintaxis
 * ------ */
void Abort(const char* prog)
{
    std::cout << prog << " : Una consola por puerto serie\n"
        "Sintaxis: " << prog << " -h | [-d <device>] [-s <speed>] [-e]\n"
        "  -h : Esta ayuda\n"
        "  -d : Dispositivo de conexión [/dev/ttyS0]\n"
        "       <device>  : Path del dispositivo serie a utilizar\n"
        "  -s : Velocidad de conexión [9600]\n"
        "       <speed>   : Velocidad de conexión requerida (9600, 19200, 38400...)\n"
        "  -e : Eco local (la entrada se repite en la salida)\n"
        ;
    exit(-1);
}

/**--------------------------------------------------------------------------------------------------
 * @brief       main
 * ------*/
int main(int argc, char* argv[])
{
    VersionInfo::Show();
    std::cout << "Using libserial v." << libserial::version() << std::endl;

    /*--- Procesamiento de línea de comandos ---*/
    const char* serial_dev = DEFAULT_SERIAL_DEV;
    uint32_t serial_bps = DEFAULT_SERIAL_BPS;
    bool local_echo = false;

    for (int param = 1; param < argc; param++)
    {
        if (argv[param][0] == '-')
        {
            switch (argv[param][1])                     // Parámetros que empiezan por '-'
            {
                case 'h':
                    Abort(argv[0]);
                    break;                              // no hace falta...

                case 'd':                               // Dispositivo de conexión (puerto serie)
                    if (param + 1 < argc)
                        serial_dev = argv[++param];
                    break;

                case 's':                               // Velocidad de conexión
                    if (param + 1 < argc)
                        serial_bps = strtoul(argv[++param], 0, 0);
                    break;

                case 'e':                               // Eco local
                    if (param + 1 < argc)
                        local_echo = true;
                    break;

                default:                                // wrong
                    Logger(Log::Error) << "Parámetro incorrecto: '" << argv[param] << "'" << Log::end;
                    break;
        }
        }
        else
            Logger(Log::Error) << "Parámetro incorrecto: '" << argv[param] << "'" << Log::end;
    }
    std::cout << "Parámetros: device='" << serial_dev << "'  speed=" << serial_bps << std::endl;

    /*--- Apertura del puerto serie ---*/
    Serial port;
    if (port.open(serial_dev, serial_bps) == false)
        Logger(Log::Error) << "Error abriendo / configurando el puerto serie" << Log::end;


    std::thread writer(PortWriter, &port, local_echo);
    PortReader(port);
}
