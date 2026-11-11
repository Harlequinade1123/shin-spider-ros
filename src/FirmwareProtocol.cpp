#include "shin_spider/FirmwareProtocol.h"

#include <cctype>
#include <cstdlib>

namespace shin_spider
{
namespace firmware
{

ParsedLine parseLikeFirmware(const std::string& line)
{
  ParsedLine out;
  if (line.length() < 3) return out;
  if (line[0] != '[' || line[line.length() - 1] != ']') return out;

  int num = -1;
  for (char c : COMMANDS)
  {
    if (c == line[1])
    {
      out.command = line[1];
      num = 1;
      for (size_t i = 0; i < line.length(); i++)
      {
        size_t pos = line.find(',', i);
        if (pos == std::string::npos) break;
        num++;
        i = pos + 1;  // firmware: i = indexOf(...); i++; plus the loop's i++
      }
      break;
    }
  }
  out.num = num;
  if (num <= 0) return out;

  out.accepted = true;
  size_t beginIndex = 1;
  for (int k = 0; k < num; k++)
  {
    if (k != num - 1)
    {
      size_t endIndex = line.find(',', beginIndex);
      if (endIndex == std::string::npos) break;
      out.elements.push_back(line.substr(beginIndex, endIndex - beginIndex));
      beginIndex = endIndex + 1;
    }
    else
    {
      size_t endIndex = line.find(']', beginIndex);
      out.elements.push_back(line.substr(beginIndex, endIndex - beginIndex));
    }
  }
  return out;
}

namespace
{
bool parseInt(const std::string& s, long& v)
{
  if (s.empty()) return false;
  size_t i = (s[0] == '-') ? 1 : 0;
  if (i == s.size()) return false;
  for (; i < s.size(); i++)
  {
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
  }
  v = std::strtol(s.c_str(), nullptr, 10);
  return true;
}
}  // namespace

Validation validateLine(const std::string& raw)
{
  Validation r;
  auto err = [&r](const std::string& e) {
    r.ok = false;
    r.errors.push_back(e);
  };

  std::string line = raw;
  if (!line.empty() && line.back() == '\n') line.pop_back();
  if (line.find('\n') != std::string::npos || line.find('\r') != std::string::npos)
  {
    err("embedded CR/LF");
    return r;
  }

  ParsedLine p = parseLikeFirmware(line);
  if (!p.accepted)
  {
    err("firmware would ignore line: " + line);
    return r;
  }
  if (p.num > MAX_ELEMENTS)
  {
    err("too many elements (" + std::to_string(p.num) + " > " + std::to_string(MAX_ELEMENTS) +
        "): firmware elements[] overflow");
  }
  if (p.elements.empty() || p.elements[0].size() != 1)
  {
    err("first element must be the single command char");
  }

  const char c = p.command;
  if (c == 'm' || c == 'v' || c == 'k')
  {
    if ((p.num - 1) % 2 != 0)
    {
      err("odd number of id/value elements");
    }
    long lo = 0, hi = 0;
    if (c == 'm') { lo = DXL_MINIMUM_POSITION_VALUE; hi = DXL_MAXIMUM_POSITION_VALUE; }
    if (c == 'v') { lo = DXL_MINIMUM_VELOCITY_VALUE; hi = DXL_MAXIMUM_VELOCITY_VALUE; }
    if (c == 'k') { lo = DXL_MINIMUM_ACCELERATION_VALUE; hi = DXL_MAXIMUM_ACCELERATION_VALUE; }
    for (size_t i = 1; i + 1 < p.elements.size(); i += 2)
    {
      long id = 0, value = 0;
      if (!parseInt(p.elements[i], id) || !parseInt(p.elements[i + 1], value))
      {
        err("non-integer id/value: " + p.elements[i] + "," + p.elements[i + 1]);
        continue;
      }
      if (id < 1 || id > DXL_NUM)
      {
        err("id out of range 1.." + std::to_string(DXL_NUM) + ": " + std::to_string(id));
        continue;
      }
      if (value < lo || value > hi)
      {
        err(std::string(1, c) + " value out of firmware range [" + std::to_string(lo) + "," + std::to_string(hi) +
            "] for id " + std::to_string(id) + ": " + std::to_string(value));
        continue;
      }
      if (c == 'm')
      {
        int mx = position_max_limits[id - 1];
        int mn = position_min_limits[id - 1];
        if (!(mx == 0 && mn == 0) && (value < mn || value > mx))
        {
          r.warnings.push_back("id " + std::to_string(id) + " position " + std::to_string(value) +
                               " outside firmware limit register [" + std::to_string(mn) + "," +
                               std::to_string(mx) + "]");
        }
      }
    }
  }
  else if (c == 'n' || c == 'f' || c == 'i' || c == 'e' || c == 'p')
  {
    if (p.num != 1)
    {
      err(std::string("command '") + c + "' takes no arguments");
    }
  }
  return r;
}

}  // namespace firmware
}  // namespace shin_spider
