/**
 * @package   libserial: Serial port communications.
 * @brief     A test program of the library.
 * @author    José Luis Sánchez Arroyo
 * @section   License
 * Copyright (c) 2005-2025 José Luis Sánchez Arroyo
 * This software is distributed under the terms of the BSD 3-Clause License and comes WITHOUT ANY WARRANTY.
 * Please read the file LICENSE for further details.
 */

#include "libserial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <chrono>               // std::chrono
#include <thread>               // std::this_thread::sleep_for

static constexpr char program_name[] = "test_libserial v2.4";

/** ----------------------------------------------------
 * @brief     Configuration constants
 * ------ */
const char* DEFAULT_SERIAL_DEV = "/dev/ttyS0";          //!< Default serial device
uint32_t    DEFAULT_SERIAL_BPS = 9600;                  //!< Default baudrate
uint32_t    DEFAULT_SERIAL_TIMEOUT = 1000;              //!< Default read/write timeout
uint32_t    MAX_MESSAGE_LEN = 512;                      //!< Max message length

using namespace std::literals;

/** ----------------------------------------------------
 * @brief     Timer: Manage timeouts.
 * ------ */
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


/** ----------------------------------------------------
 * @brief     Read data from the serial port.
 * ------ */
void ReadData(libserial::Serial& port, uint32_t timeout)
{
    uint32_t received = 0;
    std::cout << "Reading..." << std::endl;
    Timer timer;
    for (timer.setAlarm(std::chrono::milliseconds(timeout)); timer.isExpired() == false; )
    {
        uint8_t byte;
        if (port.read(&byte, 1, timer.getRemain().count()) == false)
            break;
        printf("Lapse: %09ld ms - Received: %3d bytes - Value: %02X '%c'\n", timeout - timer.getRemain().count(), ++received, byte, (byte >= 32)? byte : '.');
    }
    std::cout << "--- Total lapse: " << (timeout - timer.getRemain().count()) << " ms - " << received << " bytes received.\n";
}

/** ----------------------------------------------------
 * @brief     Read a data packet from the serial port.
 * ------ */
void ListenData(libserial::Serial& port, uint32_t timeout)
{
    uint32_t received = 0;
    std::cout << "Listening..." << std::endl;
    Timer timer;
    timer.setAlarm(0ms);
    while (1)
    {
        uint8_t byte;
        if (port.read(&byte, 1, timeout) == true)
            printf("Lapse: %09ld ms - Received: %3d bytes - Value: %02X '%c'\n", -timer.getRemain().count(), ++received, byte, (byte >= 32)? byte : '.');
    }
    std::cout << "--- Total lapse: " << -timer.getRemain().count() << " ms - " << received << " bytes received.\n";
}

/** ----------------------------------------------------
 * @brief     Write data to the serial port.
 * ------ */
void WriteData(libserial::Serial& port, std::vector<const char*>& bytes)
{
    if (bytes.size() == 0)
    {
        std::cout << "The message is empty.\n";
        return;
    }
    if (bytes.size() > MAX_MESSAGE_LEN)
    {
        std::cout << "The message is too long; allowed up to " << MAX_MESSAGE_LEN << " bytes.\n";
        return;
    }
    uint8_t message[MAX_MESSAGE_LEN];
    uint32_t msg_len = 0;
    for (unsigned ix = 0; ix < bytes.size(); ++ix)
    {
        uint32_t byte = strtoul(bytes[ix], 0, 16);
        if (byte > 0xff)
        {
            std::cout << "Invalid byte '" << bytes[ix] << "'\n";
            return;
        }
        message[msg_len++] = byte;
    }

    std::cout << "Sending message: ";
    for (unsigned ix = 0; ix < msg_len; ++ix)
        printf("%02X ", message[ix]);
    std::cout << std::endl;
    if (port.write(message, msg_len) == false)
        std::cout << "Error while sending." << std::endl;
    std::cout << "Message sent.\n";
}

/** ----------------------------------------------------
 * @brief     A console for sending and receiving data interactively
 * ------ */
void Console(libserial::Serial& port)
{
    for (bool loop = true; loop; )
    {
        char line[MAX_MESSAGE_LEN];

        std::cout << "[TestSerial (? for help)]: ";
        std::cin.getline(line, MAX_MESSAGE_LEN - 1);
        strtok(line, " ");
        switch (line[0])
        {
            case 0:
            case '?':                                 // Help!
                std::cout <<
                  "(w)rite <message>    Send data to the serial port.\n"
                  "   message:          Pairs of hexadecimal digits separated by space.\n"
                  "(r)ead  <time>       Receive data from the serial port.\n"
                  "   time:             Reception timeout, in milliseconds.\n"
                  "(c)ommand <time> <message>  Send data to the port and wait for a reply.\n"
                  "   time:             Max waiting time, in milliseconds.\n"
                  "   message:          Message to send (same as in (w)rite)\n"
                  "(a)nswer  <time> <message>  Wait a message forever and, when received, send a reply.\n"
                  "   time:             Max waiting time between consecutive bytes, in milliseconds.\n"
                  "   message:          Message to send (same as in (w)rite)\n"
                  "(l)isten <timeout>   Wait data from the port (wait forever for the first byte).\n"
                  "   timeout:          Max waiting time between consecutive bytes once the first byte is read, in milliseconds.\n"
                  "(q)uit:              Quit.\n"
                  ;
                break;

            case 'q':                                 // Quit
                std::cout << "Quitting the console.\n";
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
    std::cout << prog << " : An interactive application for testing serial port communications.\n"
      "Syntax: " << prog << " -h | [-d <device>] [-s <speed>] [-t <timeout>] [-i | <message>]\n"
      "  -h : This help\n"
      "  -d : Serial port device [/dev/ttyS0]\n"
      "       <device>  : Path to the device file to use.\n"
      "  -s : Speed (baudrate) [9600]\n"
      "       <speed>   : Required baudrate (9600, 19200, 38400...)\n"
      "  -t : Max reading timeout.\n"
      "       <timeout> : Waiting timeout, in milliseconds.\n"
      "  -i : Interactive mode\n"
      "message: Sequence of bytes to be sent to the serial port.\n"
      "         Format: Pairs of hexadecimal digits separated by a space.\n"
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

    for (bool stop = false; !stop;)
    {
        switch (getopt(argc, argv, "hd:s:t:i"))
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

            case 't':
                serial_timeout = strtoul(optarg, 0, 0);
                break;

            case 'i':
                if (bytes.size() > 0)
                {
                    std::cerr << "Please specify a message OR interactive mode, but not both." << std::endl;
                    exit(-1);
                }
                interactive = true;
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
    if (optind < argc)
    {
        if (interactive)
        {
            std::cerr << "Please specify a message OR interactive mode, but not both." << std::endl;
            exit(-1);
        }
        while (optind < argc)
            bytes.push_back(argv[optind++]);
    }

    std::cout << "Parameters: device='" << serial_dev << "'  speed=" << serial_bps << "  timeout=" << serial_timeout << std::endl;
    if (bytes.size() > 0)
    {
        std::cout << "Message   : ";
        for (unsigned ix = 0; ix < bytes.size(); ++ix)
            printf("%s ", bytes[ix]);
        std::cout << std::endl;
    }

    /*--- Open serial port ---*/
    try
    {
        libserial::Serial port(serial_dev, serial_bps);

        /*--- Send and receive a message ---*/
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
        std::cerr << "Error configuring the serial port. " << e.what() << std::endl;
    }
}
