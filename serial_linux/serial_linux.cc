#include "../serial.h"
#include "../raii.h"
#include <sys/socket.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
namespace {
/**
 * @brief The SerialLinux class : Serial Class for Linux
 */
class SerialLinux final : public lse::serial::SerialConnection, public lse::patterns::RaiiFd  {
 public:
  /**
   * @brief SerialLinux : Constructor
   * @param device : port of the device e.g. /dev/ttyUSB0
   * @param config : config SerialConfig struct for serial attributes
   */
  SerialLinux(const std::string& device, const lse::serial::SerialConfig& config);

  /**
   * @brief Receive : Read data on serial bus
   * @param size : amount of data to be read
   * @return  : array of data read
   */
  std::vector<uint8_t> Receive(int size);

  /**
   * @brief Send : write data on serial bus
   * @param buffer : buffer to be written
   * @param size : size of data to be written
   */
  void Send(const std::vector<uint8_t> &buffer);

  /**
   * @brief SetRts : allow user to set rts
   * @param level : 0 to disable rts , 1 to enable rts
   */
  void SetRts(int level);
 private:
  /**
   * @brief ConfigureConnection
   * @param config : config SerialConfig struct for serial attributes
   */
  void ConfigureConnection(const lse::serial::SerialConfig& config);
  int timeout_value_in_usec_{0};
};

void SerialLinux::SetRts(int level) {
  int status = TIOCM_RTS;
  if (level) {  // turn on RTS
    if (ioctl(fd_, TIOCMBIS, &status) == -1)
      lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_TURNING_ON_RTS_ID");
  } else {  // turn iff RTS
    if (ioctl(fd_, TIOCMBIC, &status) == -1)
      lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_TURNING_OFF_RTS_ID");
  }
}

SerialLinux::SerialLinux(const std::string &device, const lse::serial::SerialConfig &config) :
  lse::patterns::RaiiFd(open(device.c_str(), O_RDWR | O_NOCTTY)) {
  ConfigureConnection(config);
  timeout_value_in_usec_ = config.timeout_value_in_usec;
}

std::vector<uint8_t> SerialLinux::Receive(int size) {
  int actual_size = 0;
  std::vector<uint8_t> data(size);
  for (;;) {
    struct pollfd poll_fd = {fd_, POLLIN};
    auto pollrc = poll(&poll_fd, 1, timeout_value_in_usec_ / 1000L);
    if (!pollrc) {
      throw lse::except::TimeoutError();
    }
    if (pollrc < 0) {
      lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_READING_DATA_ID");
    }
    if (poll_fd.revents != POLLIN) {
      std::runtime_error("TEXT_SERIAL_POLL_ERROR_CONDITION_ID");
    }
    // handle data here (write to current offset in buffer, move pointer forward)
    if(poll_fd.revents & POLLIN) {
      auto to_read = data.size() - actual_size;
      auto result = read(fd_, data.data() + actual_size, to_read);
      if (result == to_read) return data;
      actual_size += result;
    }
  }
  return data;
}

void SerialLinux::Send(const std::vector<uint8_t> &buffer) {
  auto res = write(fd_, buffer.data(), buffer.size());

  if (res < 0) lse::except::ThrowSystemError("TEXT_UNABLE_TO_WRITE_TO_SERIAL_PORT_ID");
  if (static_cast<decltype(buffer.size())>(res) != buffer.size())
    throw std::runtime_error("TEXT_NOT_ALL_DATA_WAS_WRITTEN_TO_SERIAL_PORT_ID");
  ::tcflush(fd_, TCIFLUSH);
}

void SerialLinux::ConfigureConnection(const lse::serial::SerialConfig &config) {
  termios options = {0};
  if (tcgetattr(fd_, &options) < 0) lse::except::ThrowSystemError("TEXT_UNABLE_TO_GET_SERIAL_PORT_CONFIGURATION_ID");

  unsigned int posixBaud = 0;
  switch (config.baud_rate)  {
  case 110:    posixBaud = B110;    break;
  case 300:    posixBaud = B300;    break;
  case 600:    posixBaud = B600;    break;
  case 1200:   posixBaud = B1200;   break;
  case 2400:   posixBaud = B2400;   break;
  case 4800:   posixBaud = B4800;   break;
  case 9600:   posixBaud = B9600;   break;
  case 19200:  posixBaud = B19200;  break;
  case 38400:  posixBaud = B38400;  break;
  case 57600:  posixBaud = B57600;  break;
  case 115200: posixBaud = B115200; break;
  default:     throw std::runtime_error("TEXT_UNSUPPORTED_BAUDRATE_ID");
  }

  cfsetispeed(&options, posixBaud);
  cfsetospeed(&options, posixBaud);
  cfmakeraw(&options);

  // resetting possibly set bits
  options.c_cflag &= ~(CSIZE | PARODD | CSTOPB);

  // check serial parameters for correcntess
  switch (config.parity) {
  case lse::serial::Parity::NONE: break;
  case lse::serial::Parity::EVEN: options.c_cflag |= PARENB; break;
  case lse::serial::Parity::ODD:  options.c_cflag |= PARENB | PARODD; break;
    // program shouldn't ever reach this line since parity is enum class now
  default: throw std::runtime_error("TEXT_UNSUPPORTED_PARITY_ID");
  }

  switch (config.data_bits)  {
  case 8: options.c_cflag |= CS8; break;
  case 7: options.c_cflag |= CS7 | CSTOPB; break;
  default: throw std::runtime_error("TEXT_UNSUPPORTED_DATA_BITS_ID");
  }
  if (tcsetattr(fd_, TCSANOW, &options) < 0) lse::except::ThrowSystemError("TEXT_UNABLE_TO_SET_SERIAL_PORT_CONFIGURATION_ID");
}
}

namespace lse {
namespace serial {
std::unique_ptr<lse::serial::SerialConnection> OpenSerialPort(const std::string& device, const lse::serial::SerialConfig& config) {
  return std::unique_ptr<lse::serial::SerialConnection>(new SerialLinux(device, config));
}
}  // namespace lse
}  // namespace serial
