#include "bno085_driver/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <stdexcept>

namespace bno085_driver
{
namespace
{

// Longer than any valid line; bounds memory if the stream never sends '\n'.
constexpr std::size_t kMaxLineLength = 1024;

std::optional<speed_t> to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default: return std::nullopt;
  }
}

std::runtime_error system_error(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}

}  // namespace

bool SerialPort::is_supported_baud(int baud)
{
  return to_speed(baud).has_value();
}

SerialPort::SerialPort(const std::string & port, int baud)
{
  const auto speed = to_speed(baud);
  if (!speed) {
    throw std::runtime_error("unsupported baud rate " + std::to_string(baud));
  }
  // O_NONBLOCK keeps open() from waiting for a carrier; reads are gated by poll().
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw system_error("could not open " + port);
  }

  termios tty{};
  bool ok = ::tcgetattr(fd_, &tty) == 0;
  if (ok) {
    ::cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    ok = ::cfsetispeed(&tty, *speed) == 0 && ::cfsetospeed(&tty, *speed) == 0 &&
      ::tcsetattr(fd_, TCSANOW, &tty) == 0;
  }
  if (!ok) {
    const auto error = system_error("could not configure " + port);
    ::close(fd_);
    throw error;
  }
  ::tcflush(fd_, TCIFLUSH);
}

SerialPort::~SerialPort()
{
  ::close(fd_);
}

std::optional<std::string> SerialPort::read_line(std::chrono::milliseconds timeout)
{
  using std::chrono::steady_clock;
  const auto deadline = steady_clock::now() + timeout;

  while (true) {
    const std::size_t newline = buffer_.find('\n');
    if (newline != std::string::npos) {
      std::string line = buffer_.substr(0, newline);
      buffer_.erase(0, newline + 1);
      return line;
    }
    if (buffer_.size() > kMaxLineLength) {
      buffer_.clear();
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - steady_clock::now());
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, static_cast<int>(std::max<long>(remaining.count(), 0)));
    if (ready < 0) {
      if (errno == EINTR) {
        return std::nullopt;
      }
      throw system_error("poll failed");
    }
    if (ready == 0) {
      return std::nullopt;
    }

    char chunk[256];
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      buffer_.append(chunk, static_cast<std::size_t>(n));
    } else if (n == 0) {
      throw std::runtime_error("device disconnected");
    } else if (errno != EAGAIN && errno != EINTR) {
      throw system_error("read failed");
    } else if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::runtime_error("device disconnected");
    }
  }
}

}  // namespace bno085_driver
