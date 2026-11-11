#include "shin_spider/processing.h"

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>

namespace shin_spider
{
namespace processing
{

float sin(float angle) { return static_cast<float>(std::sin(static_cast<double>(angle))); }
float cos(float angle) { return static_cast<float>(std::cos(static_cast<double>(angle))); }
float atan2(float y, float x)
{
  return static_cast<float>(std::atan2(static_cast<double>(y), static_cast<double>(x)));
}
float sqrt(float n) { return static_cast<float>(std::sqrt(static_cast<double>(n))); }

float map(float value, float start1, float stop1, float start2, float stop2)
{
  float a = value - start1;
  float b = stop1 - start1;
  float c = a / b;
  float d = stop2 - start2;
  float e = d * c;
  return start2 + e;
}

float degrees(float radians) { return radians * RAD_TO_DEG; }

bool isNaN(float value) { return value != value; }

int toInt(float value)
{
  if (isNaN(value))
  {
    return 0;
  }
  if (value >= 2147483648.0f)
  {
    return INT_MAX;
  }
  if (value <= -2147483648.0f)
  {
    return INT_MIN;
  }
  return static_cast<int>(value);  // truncation toward zero, same as Java
}

std::string str(int value) { return std::to_string(value); }

// Java Float.toString(): shortest decimal that uniquely identifies the float,
// at least one digit after the decimal point; plain notation for
// 1e-3 <= |v| < 1e7, otherwise "d.dddE[-]n".
std::string str(float value)
{
  if (isNaN(value))
  {
    return "NaN";
  }
  if (std::isinf(value))
  {
    return value > 0 ? "Infinity" : "-Infinity";
  }
  if (value == 0.0f)
  {
    return std::signbit(value) ? "-0.0" : "0.0";
  }

  // find shortest round-trip digit string
  char buf[64];
  int precision = 1;
  for (; precision <= 9; precision++)
  {
    std::snprintf(buf, sizeof(buf), "%.*e", precision - 1, static_cast<double>(value));
    if (std::strtof(buf, nullptr) == value)
    {
      break;
    }
  }
  // buf = [-]d.dddde[+-]xx
  std::string s(buf);
  bool negative = false;
  if (s[0] == '-')
  {
    negative = true;
    s = s.substr(1);
  }
  size_t epos = s.find('e');
  int exponent = std::atoi(s.c_str() + epos + 1);
  std::string digits;
  for (size_t i = 0; i < epos; i++)
  {
    if (s[i] != '.')
    {
      digits += s[i];
    }
  }
  while (digits.size() > 1 && digits.back() == '0')
  {
    digits.pop_back();
  }

  std::string out;
  float abs_value = std::fabs(value);
  if (abs_value >= 1e-3f && abs_value < 1e7f)
  {
    if (exponent >= 0)
    {
      std::string int_part, frac_part;
      for (int i = 0; i <= exponent; i++)
      {
        int_part += (i < static_cast<int>(digits.size())) ? digits[i] : '0';
      }
      if (static_cast<int>(digits.size()) > exponent + 1)
      {
        frac_part = digits.substr(exponent + 1);
      }
      if (frac_part.empty())
      {
        frac_part = "0";
      }
      out = int_part + "." + frac_part;
    }
    else
    {
      out = "0." + std::string(-exponent - 1, '0') + digits;
    }
  }
  else
  {
    std::string mant = digits.substr(0, 1) + "." + (digits.size() > 1 ? digits.substr(1) : "0");
    out = mant + "E" + std::to_string(exponent);
  }
  return negative ? "-" + out : out;
}

namespace
{
std::mutex g_println_mutex;
std::function<void(const std::string&)> g_println_handler;
}  // namespace

void println(const std::string& msg)
{
  std::function<void(const std::string&)> handler;
  {
    std::lock_guard<std::mutex> lock(g_println_mutex);
    handler = g_println_handler;
  }
  if (handler)
  {
    handler(msg);
  }
  else
  {
    std::cout << msg << std::endl;
  }
}

void setPrintlnHandler(std::function<void(const std::string&)> handler)
{
  std::lock_guard<std::mutex> lock(g_println_mutex);
  g_println_handler = std::move(handler);
}

}  // namespace processing
}  // namespace shin_spider
