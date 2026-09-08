#include "bno085_driver/serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace bno085_driver
{

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device)
{
  if (isOpen())
  {
    return true;
  }

  file_descriptor_ =
    ::open(
      device.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (file_descriptor_ < 0)
  {
    return false;
  }

  termios tty{};

  if (tcgetattr(file_descriptor_, &tty) != 0)
  {
    close();
    return false;
  }

  /* 115200 baud */
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  /* 8 data bits */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  /* No parity */
  tty.c_cflag &= ~PARENB;

  /* One stop bit */
  tty.c_cflag &= ~CSTOPB;

  /* No hardware flow control */
  tty.c_cflag &= ~CRTSCTS;

  /* Enable receiver and ignore modem control lines */
  tty.c_cflag |= CREAD | CLOCAL;

  /* Raw input mode */
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

  tty.c_oflag &= ~OPOST;

  /* Non-blocking read */
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0)
  {
    close();
    return false;
  }

  tcflush(file_descriptor_, TCIOFLUSH);

  return true;
}

void SerialPort::close()
{
  if (file_descriptor_ >= 0)
  {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}
void SerialPort::flushInput()
{
  if (isOpen())
  {
    tcflush(file_descriptor_, TCIFLUSH);
  }
}
bool SerialPort::isOpen() const
{
  return file_descriptor_ >= 0;
}

int SerialPort::read(
  char * buffer,
  std::size_t buffer_size)
{
  if ((!isOpen()) ||
      (buffer == nullptr) ||
      (buffer_size == 0U))
  {
    return -1;
  }

  const ssize_t bytes_read =
    ::read(
      file_descriptor_,
      buffer,
      buffer_size);

  if (bytes_read > 0)
  {
    return static_cast<int>(bytes_read);
  }

  if ((bytes_read < 0) &&
      ((errno == EAGAIN) ||
       (errno == EWOULDBLOCK)))
  {
    return 0;
  }

  if (bytes_read == 0)
  {
    return 0;
  }

  return -1;
}

}  // namespace bno085_driver
