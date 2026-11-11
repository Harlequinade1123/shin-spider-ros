#include "shin_spider/SerialPort.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace shin_spider
{

SerialPort::~SerialPort() { close(); }

bool SerialPort::baudrateToSpeed(int baudrate, unsigned int& speed)
{
  switch (baudrate)
  {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 500000: speed = B500000; break;
    case 921600: speed = B921600; break;
    case 1000000: speed = B1000000; break;
    default: return false;
  }
  return true;
}

bool SerialPort::open(const std::string& device_name, int baudrate, std::string* error)
{
  close();
  auto fail = [&](const std::string& what) {
    if (error) *error = what;
    close();
    return false;
  };

  unsigned int speed = 0;
  if (!baudrateToSpeed(baudrate, speed))
  {
    return fail("unsupported baudrate " + std::to_string(baudrate));
  }

  fd_ = ::open(device_name.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    return fail("open(" + device_name + "): " + std::strerror(errno));
  }

  struct termios tty;
  if (tcgetattr(fd_, &tty) != 0)
  {
    return fail("tcgetattr(" + device_name + "): " + std::strerror(errno));
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    return fail("tcsetattr(" + device_name + "): " + std::strerror(errno));
  }
  tcflush(fd_, TCIOFLUSH);  // Processing: myPort.clear()
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::string& data)
{
  if (fd_ < 0) return false;
  size_t done = 0;
  while (done < data.size())
  {
    ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
    if (n < 0)
    {
      if (errno == EINTR) continue;
      if (errno == EAGAIN)
      {
        struct pollfd pfd{fd_, POLLOUT, 0};
        poll(&pfd, 1, 50);
        continue;
      }
      return false;
    }
    done += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace shin_spider
