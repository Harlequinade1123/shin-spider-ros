// Command-line format of OpenCM/OpenCMSpiderMotion_v2.0_XC.ino and a C++ port
// of its parser (readCommandFromSerial) used for validation and tests.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace shin_spider
{
namespace firmware
{

constexpr int DXL_NUM = 18;
constexpr int DXL_MINIMUM_POSITION_VALUE = 0;
constexpr int DXL_MAXIMUM_POSITION_VALUE = 4095;
constexpr int DXL_MINIMUM_VELOCITY_VALUE = 0;
constexpr int DXL_MAXIMUM_VELOCITY_VALUE = 32767;
constexpr int DXL_MINIMUM_ACCELERATION_VALUE = 0;
constexpr int DXL_MAXIMUM_ACCELERATION_VALUE = 32767;
constexpr int MAX_ELEMENTS = 2 * DXL_NUM + 1;  // static String elements[2 * DXL_NUM + 1]
constexpr char COMMANDS[] = {'i', 's', 'm', 'e', 'p', 'n', 'f', 'v', 'k', 'x', 'z', 'd', 'b'};

// Position limit registers written by the firmware in setup() (index = ID - 1).
// 0/0 entries (steering joints 1,4,7,10) mean "no usable limit configured".
constexpr std::array<int, DXL_NUM> position_max_limits{{0, 3350, 3350, 0, 3350, 3350, 0, 3350, 3350,
                                                        0, 3350, 3350, 3050, 3350, 4095, 3050, 3350, 4095}};
constexpr std::array<int, DXL_NUM> position_min_limits{{0, 750, 750, 0, 750, 750, 0, 750, 750,
                                                        0, 750, 750, 1050, 750, 0, 1050, 750, 0}};

struct ParsedLine
{
  bool accepted = false;          // firmware would act on the line
  char command = 0;               // line.charAt(1)
  int num = -1;                   // firmware `num` (elements count)
  std::vector<std::string> elements;
};

// Port of readCommandFromSerial(); `line` is what readStringUntil('\n') returns
// (no trailing '\n').
ParsedLine parseLikeFirmware(const std::string& line);

struct Validation
{
  bool ok = true;
  std::vector<std::string> errors;    // firmware would reject / ignore (part of) the line
  std::vector<std::string> warnings;  // accepted, but outside per-ID position limit registers
};

// Strict check of a [m|v|k|n|...] line (with or without trailing '\n').
Validation validateLine(const std::string& line);

}  // namespace firmware
}  // namespace shin_spider
