#include "serial.h"
#include <iostream>

lse::serial::SerialConfig SerialConfig() {
  lse::serial::SerialConfig serial_config;
  serial_config.baud_rate = 19200;
  serial_config.data_bits = 8;
  serial_config.parity = lse::serial::Parity::EVEN;
  serial_config.timeout_value_in_usec = 10000;
  return serial_config;
}

int main() {
  const auto& serial_config = SerialConfig();
  std::unique_ptr<lse::serial::SerialConnection> serial_comm = lse::serial::OpenSerialPort("\\\\.\\COM30", serial_config);
  serial_comm->SetRts(0); // optional
  std::vector<unsigned char> request_frame{0x00, 0x10, 0x01, 0x00, 0x00, 0x71};
  serial_comm->Send(request_frame);
  const auto& reply = serial_comm->Receive(6);
  for(auto it: reply) std::cout << std::hex <<int(it)<<" "<<std::flush;
  return 1;
}
