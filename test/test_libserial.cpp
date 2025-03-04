/**
 * libserial
 * Manejo de comunicaciones por puerto serie
 *
 * @file      test_libserial.cpp
 * @brief     Pruebas unitarias
 * @author    José Luis Sánchez Arroyo
 * @date      2025.03.03
 * @version   2.0
 *
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the LGPL version 2 and comes WITHOUT ANY WARRANTY.
 * Please read the file COPYING.LIB for further details.
 */

#include "libserial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <chrono>               // std::chrono
#include <thread>               // std::this_thread::sleep_for

static constexpr char program_name[] = "test_libserial v2.0";

/**--------------------------------------------------------------------------------------------------
 * @brief       Constantes de configuración
 * ------ */
const char* DEFAULT_SERIAL_DEV = "/dev/ttyS0";          //!< Dispositivo serie por omisión
uint32_t    DEFAULT_SERIAL_BPS = 9600;                  //!< Velocidad de conexión por omisión
uint32_t    DEFAULT_SERIAL_TIMEOUT = 1000;              //!< Tiempo de espera máximo por omisión
uint32_t    MAX_MESSAGE_LEN = 512;                      //!< Longitud máxima del mensaje en bytes

using namespace std::literals;

class Timer
{
public:
    Timer() : expire_() {};

    void setAlarm(const std::chrono::milliseconds& lapse)
    {
        expire_ = std::chrono::system_clock::now() + lapse;
    };

    bool isExpired()
    {
        return (std::chrono::system_clock::now() > expire_);
    };

    std::chrono::milliseconds getRemain()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(expire_ - std::chrono::system_clock::now());
    }
private:
    std::chrono::time_point<std::chrono::system_clock> expire_;
};


/**--------------------------------------------------------------------------------------------------
 * @brief       Leer datos del puerto serie
 * ------*/
void ReadData(Serial& port, uint32_t timeout)
{
    uint32_t received = 0;
    std::cout << "Reading..." << std::endl;
    Timer timer;
    for (timer.setAlarm(std::chrono::milliseconds(timeout)); timer.isExpired() == false; )
    {
        uint8_t byte;
        if (port.read(&byte, 1, timer.getRemain().count()) == false)
            break;
        printf("Lapso: %09ld ms - Recibidos: %3d bytes - Valor: %02X '%c'\n", timeout - timer.getRemain().count(), ++received, byte, (byte >= 32)? byte : '.');
    }
    std::cout << "--- Lapso total: " << (timeout - timer.getRemain().count()) << " ms - " << received << " bytes recibidos.\n";
}

/**--------------------------------------------------------------------------------------------------
 * @brief       Leer paquete de datos del puerto serie
 * ------*/
void ListenData(Serial& port, uint32_t timeout)
{
    uint32_t received = 0;
    std::cout << "Listening..." << std::endl;
    Timer timer;
    timer.setAlarm(0ms);
    while (1)
    {
        uint8_t byte;
        if (port.read(&byte, 1, timeout) == true)
            printf("Lapso: %09ld ms - Recibidos: %3d bytes - Valor: %02X '%c'\n", -timer.getRemain().count(), ++received, byte, (byte >= 32)? byte : '.');
    }
    std::cout << "--- Lapso total: " << -timer.getRemain().count() << " ms - " << received << " bytes recibidos.\n";
}

/**--------------------------------------------------------------------------------------------------
 * @brief       Escribir datos a puerto serie
 * ------*/
void WriteData(Serial& port, std::vector<const char*>& bytes)
{
    if (bytes.size() == 0)
    {
        std::cout << "Mensaje vacío\n";
        return;
    }
    if (bytes.size() > MAX_MESSAGE_LEN)
    {
        std::cout << "Mensaje demasiado largo, máximo " << MAX_MESSAGE_LEN << " bytes.\n";
        return;
    }
    uint8_t message[MAX_MESSAGE_LEN];
    uint32_t msg_len = 0;
    for (unsigned ix = 0; ix < bytes.size(); ++ix)
    {
        uint32_t byte = strtoul(bytes[ix], 0, 16);
        if (byte > 0xff)
        {
            std::cout << "Byte incorrecto '" << bytes[ix] << "'\n";
            return;
        }
        message[msg_len++] = byte;
    }

    std::cout << "Enviando mensaje: ";
    for (unsigned ix = 0; ix < msg_len; ++ix)
        printf("%02X ", message[ix]);
    std::cout << std::endl;
    if (port.write(message, msg_len) == false)
        std::cout << "Error durante el envío." << std::endl;
    std::cout << "Mensaje enviado.\n";
}

/**--------------------------------------------------------------------------------------------------
 * @brief       Consola para enviar y recibir datos
 * ------*/
void Console(Serial& port)
{
    for (bool loop = true; loop; )
    {
        char line[MAX_MESSAGE_LEN];

        std::cout << "[TestSerial (? para ayuda)]: ";
        std::cin.getline(line, MAX_MESSAGE_LEN - 1);
        strtok(line, " ");
        switch (line[0])
        {
            case 0:
            case '?':                                 // Help!
                std::cout <<
                  "(w)rite <message>    Enviar datos al puerto serie\n"
                  "   message:          Pares de dígitos hexadecimales separados por espacio\n"
                  "(r)ead  <time>       Recibir datos del puerto serie\n"
                  "   time:             Tiempo máximo de recepción, en milisegundos\n"
                  "(c)ommand <time> <message>  Enviar datos al puerto y esperar respuesta inmediatamente\n"
                  "   time:             Tiempo máximo de recepción, en milisegundos\n"
                  "   message:          Mensaje a enviar (igual que en (w)rite)\n"
                  "(a)nswer  <time> <message>  Esperar indefinidamente un mensaje y, tras recibirlo, enviar respuesta\n"
                  "   time:             Tiempo máximo de espera entre bytes del mensaje, en milisegundos\n"
                  "   message:          Mensaje a enviar (igual que en (w)rite)\n"
                  "(l)isten <timeout>   Esperar indefinidamente datos desde el puerto\n"
                  "   timeout:          Una vez leído un byte, tiempo máximo de espera por un byte\n"
                  "(q)uit: Salir de la consola\n"
                  ;
                break;

            case 'q':                                 // Quit
                std::cout << "Saliendo de la consola.\n";
                loop = false;
                break;

            case 'r':                                 // Read
            case 'l':                                 // Listen
            {
                uint32_t timeout = 0;
                char* t_out = strtok(0, " ");
                if (t_out)
                    timeout = strtoul(t_out, 0, 0);
                if (line[0] == 'r')
                    ReadData(port, timeout);
                else
                    ListenData(port, timeout);
                break;
            }

            case 'w':                                 // Write
            {
                std::vector<const char*> bytes;
                for (char* ptr = strtok(0, " "); ptr; ptr = strtok(0, " "))
                    bytes.push_back(ptr);
                if (bytes.size() > 0)
                    WriteData(port, bytes);
                break;
            }

            case 'c':                                 // Command (write, then read)
            case 'a':                                 // Answer (read, then write)
            {
                uint32_t timeout = 0;
                char* t_out = strtok(0, " ");
                if (t_out)
                    timeout = strtoul(t_out, 0, 0);
                std::vector<const char*> bytes;
                for (char* ptr = strtok(0, " "); ptr; ptr = strtok(0, " "))
                    bytes.push_back(ptr);

                if (line[0] == 'a')
                    ListenData(port, timeout);

                if (bytes.size() > 0)
                    WriteData(port, bytes);

                if (line[0] == 'c')
                    ReadData(port, timeout);
                break;
            }
            default:
                std::cout << "Nonsense.\n";
                break;
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
    std::cout << prog << " : Pruebas de transferencia de datos por puerto serie\n"
      "Sintaxis: " << prog << " -h | [-d <device>] [-s <speed>] [-t <timeout>] [-i | <mensaje>]\n"
      "  -h : Esta ayuda\n"
      "  -d : Dispositivo de conexión [/dev/ttyS0]\n"
      "       <device>  : Path del dispositivo serie a utilizar\n"
      "  -s : Velocidad de conexión [9600]\n"
      "       <speed>   : Velocidad de conexión requerida (9600, 19200, 38400...)\n"
      "  -t : Tiempo máximo de espera en lectura\n"
      "       <timeout> : Tiempo de espera, en milisegundos\n"
      "  -i : Modo interactivo\n"
      "mensaje: secuencia de bytes a enviar por el puerto (pares de dígitos hexadecimales separados por espacio)\n"
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
    uint32_t serial_timeout = DEFAULT_SERIAL_TIMEOUT;
    bool interactive = false;
    std::vector<const char*> bytes;

    for (int param = 1; param < argc; param++)
    {
        if (argv[param][0] == '-')
        {
            switch (argv[param][1])                           // Parámetros que empiezan por '-'
            {
                case 'h':
                    Abort(argv[0]);
                    break;                                        // no hace falta...

                case 'd':                                       // Dispositivo de conexión (puerto serie)
                    if (param + 1 < argc)
                        serial_dev = argv[++param];
                    break;

                case 's':                                       // Velocidad de conexión
                    if (param + 1 < argc)
                        serial_bps = strtoul(argv[++param], 0, 0);
                    break;

                case 't':                                       // Timeout de lectura en milisegundos
                    if (param + 1 < argc)
                        serial_timeout = strtoul(argv[++param], 0, 0);
                    break;

                case 'i':
                    if (bytes.size() > 0)
                    {
                        std::cerr << "Especifique modo interactivo o mensaje en línea, no ambos." << std::endl;
                        exit(-1);
                    }
                    interactive = true;
                    break;

                default:                                        // wrong
                    std::cerr << "Parámetro incorrecto: '" << argv[param] << "'" << std::endl;
                    exit(-1);
                    break;
            }
        }
        else
        {
            if (interactive)
            {
                std::cerr << "Especifique modo interactivo o mensaje en línea, no ambos." << std::endl;
                exit(-1);
            }
            bytes.push_back(argv[param]);
        }
    }
    std::cout << "Parámetros: device='" << serial_dev << "'  speed=" << serial_bps << "  timeout=" << serial_timeout << std::endl;
    if (bytes.size() > 0)
    {
        std::cout << "Mensaje   : ";
        for (unsigned ix = 0; ix < bytes.size(); ++ix)
            printf("%s ", bytes[ix]);
        std::cout << std::endl;
    }

    /*--- Apertura del puerto serie ---*/
    try
    {
        Serial port(serial_dev, serial_bps);

        /*--- Enviar y recibir mensaje ---*/
        if (interactive)
            Console(port);
        else
        {
            if (bytes.size() > 0)
                WriteData(port, bytes);
            ReadData(port, serial_timeout);
        }
    }
    catch(std::exception& e)
    {
        std::cerr << "Error inicializando el puerto serie. " << e.what() << std::endl;
    }
}
