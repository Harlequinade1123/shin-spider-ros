// Minimal re-implementation of the parts of the Processing (Java) API used by
// original_spider_controller, with identical float semantics.
#pragma once

#include <functional>
#include <string>

namespace shin_spider
{

// Injectable clock: replaces Processing millis().
class Clock
{
public:
  virtual ~Clock() = default;
  virtual int millis() = 0;
};

// Injectable output sink: replaces SerialManager.write() + delay().
class CommandSink
{
public:
  virtual ~CommandSink() = default;
  virtual void write(const std::string& data) = 0;
  virtual void delay(int ms) = 0;
};

namespace processing
{

// (float)Math.PI
constexpr float PI = 3.14159265358979323846f;
// PConstants.RAD_TO_DEG = 180.0f / PI (float division)
constexpr float RAD_TO_DEG = 180.0f / PI;

// Processing: (float)Math.sin(angle) etc. (computed in double, rounded to float)
float sin(float angle);
float cos(float angle);
float atan2(float y, float x);
float sqrt(float n);

// PApplet.map(): start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1)), all float
float map(float value, float start1, float stop1, float start2, float stop2);

// PApplet.degrees(): radians * RAD_TO_DEG
float degrees(float radians);

// Java (int) cast of a float: truncation toward zero, NaN -> 0, saturating.
int toInt(float value);

bool isNaN(float value);

// Processing str(int) / str(float) (= String.valueOf -> Java Float.toString)
std::string str(int value);
std::string str(float value);

// Processing println(); routed through a replaceable logger (default: stdout).
void println(const std::string& msg);
void setPrintlnHandler(std::function<void(const std::string&)> handler);

}  // namespace processing
}  // namespace shin_spider
