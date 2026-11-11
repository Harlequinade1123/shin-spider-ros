// termios serial port (settings adapted from kingspider openSerial()).
#pragma once

#include <string>

namespace shin_spider
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // Returns false (and fills error) when the device cannot be opened/configured.
  bool open(const std::string& device_name, int baudrate, std::string* error = nullptr);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  // Writes all bytes; returns false on error.
  bool write(const std::string& data);
  int fd() const { return fd_; }

  static bool baudrateToSpeed(int baudrate, unsigned int& speed);

private:
  int fd_ = -1;
};

}  // namespace shin_spider
