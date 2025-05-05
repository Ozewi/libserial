/**
 * @package   libserial: Serial port communications.
 * @brief     Mini-terminal for serial port.
 * @author    José Luis Sánchez Arroyo
 * @section   License
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the BSD 3-Clause License and comes WITHOUT ANY WARRANTY.
 * Please read the file LICENSE for further details.
 */

#include "libserial.h"
#include <poll.h>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <string.h>                 // strerror

static constexpr char program_name[] = "nanoterm v2.4";

/** ----------------------------------------------------
 * @brief     Configuration constants
 * ------ */
const char* DEFAULT_SERIAL_DEV = "/dev/ttyS0";          //!< Default serial device
uint32_t    DEFAULT_SERIAL_BPS = 9600;                  //!< Default baudrate

/** ----------------------------------------------------
 * @brief     Read from the port and copy to stdout
 * ------ */
void PortReader(libserial::Serial& port)
{
    char byte;
    while (true)
    {
        if (port.pendingRead())
        {
            while (port.read(&byte, 1, libserial::NO_TIMEOUT))
                std::cout << byte;
            fflush(stdout);
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/** ----------------------------------------------------
 * @brief     Read from the keyboard and copy to the port and, optionally, to stdout
 * ------ */
void PortWriter(libserial::Serial* port, bool local_echo)
{
    pollfd poll_entry = { fileno(stdin), POLLIN, 0 };

    while (true)
    {
        auto rc = poll(&poll_entry, 1, 1000);
        if (rc < 0)
        {
            std::cerr << "Error polling from console: " << strerror(errno) << " (errno: " << errno << "). Exiting." << std::endl;
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

/** ----------------------------------------------------
 * @brief     Auxiliary functions
 * ------ */

 /**
 * @brief     Print banner
 */
void Banner()
{
    std::cout << program_name << " - running libserial v" << libserial::version() << std::endl;
}

/**
 * @brief     Print syntax and exit
 */
void Abort(const char* prog)
{
    std::cout << prog << " : A serial port console.\n"
        "Syntax: " << prog << " -h | [-d <device>] [-s <speed>] [-e]\n"
        "  -h : This help.\n"
        "  -d : Serial port device [/dev/ttyS0]\n"
        "       <device>  : Path to the device file to use.\n"
        "  -s : Speed (baudrate) [9600]\n"
        "       <speed>   : Required baudrate (9600, 19200, 38400...)\n"
        "  -e : Local echo (input is copied to output)\n"
        ;
    exit(-1);
}

/**--------------------------------------------------------------------------------------------------
 * @brief     main
 * ------*/
int main(int argc, char* argv[])
{
    Banner();

    /*--- Command-line processing ---*/
    const char* serial_dev = DEFAULT_SERIAL_DEV;
    uint32_t serial_bps = DEFAULT_SERIAL_BPS;
    bool local_echo = false;

    for (bool stop = false; !stop;)
    {
        switch (getopt(argc, argv, "hd:s:e"))
        {
            case 'h':
                Abort(argv[0]);
                break;

            case 'd':
                serial_dev = optarg;
                break;

            case 's':
                serial_bps = strtoul(optarg, 0, 0);
                break;

            case 'e':
                local_echo = true;
                break;

            case -1:
                stop = true;
                break;

            default:
                std::cerr << "Invalid argument." << std::endl;
                exit(-1);
                break;
        }
    }

    std::cout << "Parameters: device='" << serial_dev << "' speed=" << serial_bps << std::endl;

    /*--- Open serial port ---*/
    try
    {
        libserial::Serial port(serial_dev, serial_bps);

        /*--- Start the thread for reading the console and run the function for reading the port ---*/
        std::thread writer(PortWriter, &port, local_echo);
        PortReader(port);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Error configuring the serial port. " << e.what() << std::endl;
    }
}
