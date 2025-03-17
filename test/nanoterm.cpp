/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      nanoterm.cpp
 * @brief     Miniterminal de puerto serie
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.05
 * @version   2.1
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <poll.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <unistd.h>

static constexpr char program_name[] = "nanoterm v2.3";

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
    while (true)
    {
        if (port.pendingRead())
        {
            while (port.read(&byte, 1, Serial::NO_TIMEOUT))
                std::cout << byte;
            fflush(stdout);
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
            std::cerr << "Error leyendo de la consola -- errno: " << errno << ". Terminando." << std::endl;
            exit(-1);
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

/**
 * @brief   Banner
 */
void Banner()
{
    std::cout << program_name << " - running libserial v" << libserial::version() << std::endl;
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
    Banner();

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
                    std::cerr << "Parámetro incorrecto: '" << argv[param] << "'" << std::endl;
                    exit(-1);
                    break;
            }
        }
        else
        {
            std::cerr << "Parámetro incorrecto: '" << argv[param] << "'" << std::endl;
            exit(-1);
        }
    }
    std::cout << "Parámetros: device='" << serial_dev << "' speed=" << serial_bps << std::endl;

    /*--- Apertura del puerto serie ---*/
    try
    {
        Serial port(serial_dev, serial_bps);

        /*--- Iniciar lectura del puerto y thread de lectura de consola ---*/
        std::thread writer(PortWriter, &port, local_echo);
        PortReader(port);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error inicializando el puerto serie. " << e.what() << std::endl;
    }
}
