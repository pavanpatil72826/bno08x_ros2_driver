#ifndef BNO085_DRIVER__SERIAL_PORT_HPP_
#define BNO085_DRIVER__SERIAL_PORT_HPP_

#include <chrono>
#include <optional>
#include <string>

namespace bno085_driver
{

/// Minimal line-oriented, read-only POSIX serial port (8N1, no flow control).
class SerialPort
{
public:
  /// Opens and configures the port and discards pending input.
  /// Throws std::runtime_error on failure, including an unsupported baud rate.
  SerialPort(const std::string & port, int baud);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Returns the next line without its '\n', or std::nullopt if none completed
  /// within the timeout. Throws std::runtime_error if the device fails or disappears.
  std::optional<std::string> read_line(std::chrono::milliseconds timeout);

  static bool is_supported_baud(int baud);

private:
  int fd_{-1};
  std::string buffer_;
};

}  // namespace bno085_driver

#endif  // BNO085_DRIVER__SERIAL_PORT_HPP_
