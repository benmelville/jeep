#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

/**
 * function to open a serial port
 * @param port_name The name of the port
 * @return The file descriptor of the newly opened port
 */
int openSerialPort(std::string &port_name)
{
    // open port for read/write, no
    int file_descriptor = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (file_descriptor < 0)
    {
        std::cerr << "Error opening " << port_name << ": " << strerror(errno) << std::endl;
        return -1;
    }
    return file_descriptor;
}

/**
 * function to configure the serial port
 * @param file_descriptor port to be configured
 * @param speed baudrate for I/O
 */
bool configureSerialPort(int file_descriptor, int speed)
{
    struct termios tty;

    if (tcgetattr(file_descriptor, &tty) != 0)
    {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    // set I/O baudrate for the port
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE | CS8); // clear character size and then set them to 8 bits
    tty.c_iflag &= ~IGNBRK;                     // disable ignoring break conditions
    tty.c_lflag = 0;                            // disable local mode flags i.e. ctrl+c, echoing of typed chars, canonical processing
    tty.c_oflag = 0;                            // disable output mode flags i.e. output remapping (carriage return + newline) and output delays
    tty.c_cc[VMIN] = 0;                         // set control character for non-canonical mode to return if read is result is empty
    tty.c_cc[VTIME] = 5;                        // set read timeout to be 5 tenths of a second (0.5 seconds)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // disable software flow control
    tty.c_cflag |= (CLOCAL | CREAD);            // tell kernel to ignore modem control lines and enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag &= ~CSTOPB;                     // use one stop bit
    tty.c_cflag &= ~CRTSCTS;                    // disable hardware flow control -> RTS (request to send) and CTS (clear to send)

    // Apply the settings to the tty of the serial port immediately (TCSANOW)
    if (tcsetattr(file_descriptor, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void closeSerialPort(int file_descriptor) { close(file_descriptor); }

int main()
{
    std::string port_name = "/dev/ttys000";
    int file_descriptor = openSerialPort(port_name);

    if (file_descriptor < 0)
        return -1;

    if (!configureSerialPort(file_descriptor, B115200))
        return -1;

    std::cout << "serial port successfully opened!!" << std::endl;
    closeSerialPort(file_descriptor);
    return 0;
}