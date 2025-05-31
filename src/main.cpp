#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "SerialInterface.hpp"

int main()
{
    std::string port_name = "/dev/ttys000";

    SerialInterface *test = new SerialInterface(port_name, B115200);

    if (test->init() == 0)
    {
        std::cout << "YESSIR" << std::endl;
    }

    return 0;
}