#ifndef SERIAL_SERIAL_H_
#define SERIAL_SERIAL_H_
#include "constants.h"
#include <string>
#include <memory>
#include <vector>

namespace lse {
namespace serial {
/**
 * @brief The SerialConfig struct for serial attributes
 */
struct SerialConfig {
  int baud_rate;
  int data_bits;
  lse::serial::Parity parity;
  int timeout_value_in_usec;
};

/**
 * @brief The SerialConnection struct : Interface for Serial Connection
 */
struct SerialConnection {
  /**
     * @brief Receive : Read data on serial bus
     * @param size : amount of data to be read
     * @return : array of data read
     */
  virtual std::vector<uint8_t> Receive(int size) = 0;

  /**
     * @brief Send : write data on serial bus
     * @param buffer : buffer to be written
     * @param size : size of data to be written
     */
  virtual void Send(const std::vector<uint8_t> &buffer) = 0;

  /**
     * @brief SetRts : allow user to set rts
     * @param level : 0 to disable rts , 1 to enable rts
     */
  virtual void SetRts(int level) = 0;

  /**
     * @brief ~SerialConnection() : default
     */
  virtual ~SerialConnection() = default;
};

/**
 * @brief OpenSerialPort : Returns the serial connection
 * @param device : port of the device e.g. /dev/ttyUSB0, \\\\.\\COM8
 * @param config SerialConfig struct for serial attributes
 * @return pointer to the SerialConnection
 */
std::unique_ptr<SerialConnection> OpenSerialPort(const std::string& device, const SerialConfig& config);

}  // namespace lse
}  // namespace serial
#endif  // SERIAL_SERIAL_H_

