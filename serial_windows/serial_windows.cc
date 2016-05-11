#include "../serial.h"
#include "../except/helpers.h"
#include <iostream>
#include <chrono>
#include <windows.h>
namespace {
/**
 * @brief The Overlapped class : wrapper for OVERLAPPED struct
 */
class Overlapped {
public:
  Overlapped() {
    impl = {0};
    // Create the overlapped event. Must be closed before exiting to avoid a handle leak.
    impl.hEvent = CreateEvent(nullptr, true, false, nullptr);
    if (impl.hEvent == nullptr) lse::except::ThrowSystemError("TEXT_ERROR_CREATING_OVERLAPPED_EVENT_ID");
  }

  /**
   * @brief No default ctor available
   */
  virtual ~Overlapped() {
    CloseHandle(impl.hEvent);
  }

  operator OVERLAPPED* () {
    return &impl;
  }


  operator HANDLE () {
    return impl.hEvent;
  }

private:
  OVERLAPPED impl;
};


const int kTimeConversionFactor = 1000;
/**
 * @brief The SerialWindows class : Serial Class of windows
 */
class SerialWindows final : public lse::serial::SerialConnection {
public:
  /**
   * @brief SerialWindows : Constructor
   * @param device : port of the device e.g. \\\\.\\COM8
   * @param config : config SerialConfig struct for serial attributes
   */
  SerialWindows(const std::string& device, const lse::serial::SerialConfig& config);

  virtual ~SerialWindows();

  /**
   * @brief Receive : Read data on serial bus
   * @param size : amount of data to be read
   * @return : array of data read
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
  HANDLE handle_;
  int timeout_value_in_usec_{0};
};

SerialWindows::SerialWindows(const std::string& device, const lse::serial::SerialConfig& config) {
  handle_ = CreateFile(device.c_str(),                // port name
                       GENERIC_READ | GENERIC_WRITE,  // Read/Write
                       0,                             // No Sharing
                       nullptr,                       // No Security
                       OPEN_EXISTING,                 // Open existing port only
                       0,                             // Non Overlapped I/O
                       nullptr);
  if (handle_ == INVALID_HANDLE_VALUE)
    lse::except::ThrowSystemError("TEXT_UNABLE_TO_GET_SERIAL_PORT_CONFIGURATION_ID");
  else
    std::cout << ("opening serial port successful")<< std::endl;

  ConfigureConnection(config);
  timeout_value_in_usec_ = config.timeout_value_in_usec;
}

SerialWindows::~SerialWindows() {
  CloseHandle(handle_);
}

std::vector<uint8_t> SerialWindows::Receive(int size) {
  std::vector<unsigned char> data(size);  // Buffer for storing Rxed Data
  int actual_size = 0;
  Overlapped overlapped;

  while (true) {
    bool is_waiting_on_read = false;
    // Issue read operation.
    DWORD bytes_read{0};
    auto to_read = data.size() - actual_size;
    if (!ReadFile(handle_, data.data(), to_read, &bytes_read, overlapped)) {
      if (GetLastError() != ERROR_IO_PENDING) lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_COMMUNICATING_TO_DEVICE_ID");
      else is_waiting_on_read = true;
    }
    else {
      if (bytes_read == to_read) return data;  // read completed immediately
      actual_size += bytes_read;
    }

    if (is_waiting_on_read) {
      auto wait_reply = WaitForSingleObject(overlapped, timeout_value_in_usec_ / kTimeConversionFactor);
      switch(wait_reply) {
      case WAIT_OBJECT_0:   // Read completed.
        if (!GetOverlappedResult(handle_, overlapped, &bytes_read, false)) lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_COMMUNICATING_TO_DEVICE_ID");
        else if (bytes_read == to_read) return data;  // read completed successfully.
        break;
      case WAIT_TIMEOUT: throw lse::except::TimeoutError(); break;
      default: lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_READING_DATA_ID"); break;
      }
    }
  }
}

void SerialWindows::Send(const std::vector<uint8_t> &buffer) {
  DWORD no_of_bytes_written = 0;  // No of bytes written to the port
  auto res = WriteFile(handle_,  // Handle to the Serial port
                  buffer.data(),  // Data to be written to the port
                  buffer.size(),  // No of bytes to write
                  &no_of_bytes_written,  // Bytes written
                  nullptr);
  if (!res || no_of_bytes_written !=buffer.size()) throw std::runtime_error("TEXT_NOT_ALL_DATA_WAS_WRITTEN_TO_SERIAL_PORT_ID");
}

void SerialWindows::SetRts(int level) {
  if (level) {
    if (EscapeCommFunction(handle_, SETRTS) == 0) lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_TURNING_ON_RTS_ID");
  }
  else {
    if (EscapeCommFunction(handle_, CLRRTS) == 0) lse::except::ThrowSystemError("TEXT_ERROR_OCCURED_WHILE_TURNING_OFF_RTS_ID");
  }
}

void SerialWindows::ConfigureConnection(const lse::serial::SerialConfig &config) {
  DCB dcbSerialParams = { 0 };  // Initializing DCB structure
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (GetCommState(handle_, &dcbSerialParams) == 0) lse::except::ThrowSystemError("TEXT_UNABLE_TO_GET_SERIAL_PORT_CONFIGURATION_ID");

  unsigned int posixBaud = 0;
  switch (config.baud_rate)  {
  case 110:    posixBaud = CBR_110;    break;
  case 300:    posixBaud = CBR_300;    break;
  case 600:    posixBaud = CBR_600;    break;
  case 1200:   posixBaud = CBR_1200;   break;
  case 2400:   posixBaud = CBR_2400;   break;
  case 4800:   posixBaud = CBR_4800;   break;
  case 9600:   posixBaud = CBR_9600;   break;
  case 19200:  posixBaud = CBR_19200;  break;
  case 38400:  posixBaud = CBR_38400;  break;
  case 57600:  posixBaud = CBR_57600;  break;
  case 115200: posixBaud = CBR_115200; break;
  default:     throw std::runtime_error("TEXT_UNSUPPORTED_BAUDRATE_ID");
  }

  dcbSerialParams.BaudRate = posixBaud;  // Setting BaudRate = 9600

  // resetting possibly set bits
  dcbSerialParams.StopBits = ONESTOPBIT;  // Setting StopBits = 1

  // Setting Parity
  switch (config.parity) {
  case lse::serial::Parity::NONE: dcbSerialParams.Parity   = NOPARITY; break;  // Setting Parity = None;
  case lse::serial::Parity::EVEN: dcbSerialParams.Parity   = EVENPARITY; break;
  case lse::serial::Parity::ODD:  dcbSerialParams.Parity   = ODDPARITY; break;
    // program shouldn't ever reach this line since parity is enum class now
  default: throw std::runtime_error("TEXT_UNSUPPORTED_PARITY_ID");
  }

  switch (config.data_bits)  {
  case 8: dcbSerialParams.ByteSize = 8; break;
  case 7: dcbSerialParams.ByteSize = 7; break;
  default: throw std::runtime_error("TEXT_UNSUPPORTED_DATA_BITS_ID");
  }

  if (SetCommState(handle_, &dcbSerialParams) == 0) lse::except::ThrowSystemError("TEXT_UNABLE_TO_SET_SERIAL_PORT_CONFIGURATION_ID");
}
}

namespace lse {
namespace serial {
std::unique_ptr<lse::serial::SerialConnection> OpenSerialPort(const std::string& device, const lse::serial::SerialConfig& config) {
  return std::unique_ptr<SerialConnection>(new SerialWindows(device, config));
}
}  // namespace lse
}  // namespace serial
