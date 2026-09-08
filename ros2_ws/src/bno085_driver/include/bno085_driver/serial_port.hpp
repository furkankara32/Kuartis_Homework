#ifndef BNO085_DRIVER__SERIAL_PORT_HPP_
#define BNO085_DRIVER__SERIAL_PORT_HPP_

#include <cstddef>
#include <string>

namespace bno085_driver
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & device);
  void close();
  void flushInput();
  bool isOpen() const;

  int read(char * buffer, std::size_t buffer_size);

private:
  int file_descriptor_{-1};
};

}  // namespace bno085_driver

#endif  // BNO085_DRIVER__SERIAL_PORT_HPP_
