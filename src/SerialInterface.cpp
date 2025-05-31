#include "SerialInterface.hpp"

SerialInterface::SerialInterface(std::string serial_port, int baud_rate) : serial_port_(serial_port), baud_rate_(baud_rate)
{
}

SerialInterface::~SerialInterface()
{
    if (file_descriptor_)
        closeSerialPort();
}

int SerialInterface::init()
{
    file_descriptor_ = openSerialPort(serial_port_);
    if (file_descriptor_ < 0)
        return -1;

    if (!configureSerialPort())
        return -1;
    initializeELM327();
    return 0;
}

int SerialInterface::openSerialPort(std::string &port_name)
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

bool SerialInterface::configureSerialPort()
{
    struct termios tty{};

    if (tcgetattr(file_descriptor_, &tty) != 0)
    {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    // set I/O baudrate for the port
    cfsetispeed(&tty, baud_rate_);
    cfsetospeed(&tty, baud_rate_);

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
    if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void SerialInterface::writeToSerial(const std::string &command)
{
    std::string full_command = command + '\r';
    write(file_descriptor_, full_command.c_str(), full_command.length());
}

std::string SerialInterface::readFromSerial()
{
    char buffer[256];
    int read_size = read(file_descriptor_, buffer, sizeof(buffer) - 1);
    buffer[read_size] = '\0';
    return std::string(buffer);
}

void SerialInterface::initializeELM327()
{
    writeToSerial(elm327_.reset); // reset ELM327
    readFromSerial();
    writeToSerial(elm327_.no_echo); // disable command echo
    readFromSerial();
    writeToSerial(elm327_.no_line_feed); // disable line feeds (responses are only terminated by carriage returns)
    readFromSerial();
    writeToSerial(elm327_.no_spaces); // disable spaces in response
    readFromSerial();
}
