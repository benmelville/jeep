#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "ELM327.h"

class SerialInterface
{
public:
    int init();
    SerialInterface(std::string serial_port, int baud_rate);
    ~SerialInterface();

    /**
     * write to serial port
     * @param command what to write to serial port
     */
    void writeToSerial(const std::string &command);

    /**
     * read from serial port
     * @return data read from serial port
     */
    std::string readFromSerial();

private:
    std::string serial_port_;
    int baud_rate_;
    int file_descriptor_;
    ELM327 elm327_;

    /**
     * open a serial port
     * @param port_name The name of the port
     * @return The file descriptor of the newly opened port
     */
    int openSerialPort(std::string &port_name);

    /**
     * close a serial port
     * @param file_descriptor the serial port to close
     */
    inline void closeSerialPort() { close(file_descriptor_); }

    /**
     * configure a serial port
     * @return serial port configuration status
     */
    bool configureSerialPort();

    /**
     * initialize communication with ELM327
     * @param file_descriptor to initialize serial port with
     */
    void initializeELM327();
};

#endif