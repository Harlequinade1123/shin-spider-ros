#include "shin_spider/MotionTestHook.h"

#include <cctype>
#include <cstdlib>
#include <exception>
#include <functional>
#include <map>
#include <sstream>

namespace shin_spider
{
namespace test_hook
{
using processing::PI;

namespace
{
std::string stripNewline(const std::string& s)
{
  std::string r = s;
  while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) r.pop_back();
  return r;
}

std::string trim(const std::string& s)
{
  size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) b++;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) e--;
  return s.substr(b, e - b);
}

bool parseFloatArg(std::string s, float& out)
{
  size_t eq = s.find('=');  // "steering=1.5707964" -> "1.5707964"
  if (eq != std::string::npos) s = s.substr(eq + 1);
  s = trim(s);
  if (s.empty()) return false;
  bool neg = false;
  std::string body = s;
  if (body[0] == '-' || body[0] == '+')
  {
    neg = body[0] == '-';
    body = body.substr(1);
  }
  std::string sym = body;
  std::string divisor;
  size_t slash = body.find('/');
  if (slash != std::string::npos)
  {
    sym = body.substr(0, slash);
    divisor = body.substr(slash + 1);
  }
  float base;
  bool symbolic = true;
  if (sym == "PI") base = PI;
  else if (sym == "HALF_PI") base = PI / 2;   // PConstants.HALF_PI = (float)(Math.PI / 2.0) == PI/2 exactly
  else if (sym == "QUARTER_PI") base = PI / 4;
  else symbolic = false;
  if (symbolic)
  {
    if (!divisor.empty())
    {
      char* end = nullptr;
      long d = std::strtol(divisor.c_str(), &end, 10);
      if (*end != '\0' || d == 0) return false;
      base = base / static_cast<float>(d);
    }
    out = neg ? -base : base;
    return true;
  }
  // numeric literal (Java Float.parseFloat == correctly rounded strtof); allow trailing 'f'
  std::string num = s;
  if (!num.empty() && (num.back() == 'f' || num.back() == 'F')) num.pop_back();
  char* end = nullptr;
  out = std::strtof(num.c_str(), &end);
  return end != num.c_str() && *end == '\0';
}

bool parseIntArg(const std::string& s, int& out)
{
  float f;
  if (!parseFloatArg(s, f)) return false;
  out = static_cast<int>(f);
  return static_cast<float>(out) == f;
}

using Handler = std::function<bool(SpiderController&, const std::vector<std::string>&)>;

const std::map<std::string, Handler>& handlers()
{
  static const std::map<std::string, Handler> h = [] {
    std::map<std::string, Handler> m;
    auto fi = [](void (SpiderController::*fn)(float, int)) {
      return [fn](SpiderController& c, const std::vector<std::string>& a) {
        float x; int y;
        if (a.size() != 2 || !parseFloatArg(a[0], x) || !parseIntArg(a[1], y)) return false;
        (c.*fn)(x, y);
        return true;
      };
    };
    auto ff = [](void (SpiderController::*fn)(float, float)) {
      return [fn](SpiderController& c, const std::vector<std::string>& a) {
        float x, y;
        if (a.size() != 2 || !parseFloatArg(a[0], x) || !parseFloatArg(a[1], y)) return false;
        (c.*fn)(x, y);
        return true;
      };
    };
    auto f1 = [](void (SpiderController::*fn)(float)) {
      return [fn](SpiderController& c, const std::vector<std::string>& a) {
        float x;
        if (a.size() != 1 || !parseFloatArg(a[0], x)) return false;
        (c.*fn)(x);
        return true;
      };
    };
    auto i1 = [](void (SpiderController::*fn)(int)) {
      return [fn](SpiderController& c, const std::vector<std::string>& a) {
        int x;
        if (a.size() != 1 || !parseIntArg(a[0], x)) return false;
        (c.*fn)(x);
        return true;
      };
    };
    auto v0 = [](void (SpiderController::*fn)()) {
      return [fn](SpiderController& c, const std::vector<std::string>& a) {
        if (!a.empty()) return false;
        (c.*fn)();
        return true;
      };
    };
    m["testLegMotion"] = i1(&SpiderController::testLegMotion);
    m["walkingMotion"] = fi(&SpiderController::walkingMotion);
    m["sideWalkingMotion"] = fi(&SpiderController::sideWalkingMotion);
    m["jumpingMotion"] = fi(&SpiderController::jumpingMotion);
    m["backJumpingMotion"] = fi(&SpiderController::backJumpingMotion);
    m["rotationalMotion"] = ff(&SpiderController::rotationalMotion);
    m["rotationalMotion90"] = ff(&SpiderController::rotationalMotion90);
    m["spreadMotion"] = f1(&SpiderController::spreadMotion);
    m["shrinkMotion"] = f1(&SpiderController::shrinkMotion);
    m["resetMotion"] = f1(&SpiderController::resetMotion);
    m["testHandMotion"] = i1(&SpiderController::testHandMotion);
    m["wavingHandsMotion"] = v0(&SpiderController::wavingHandsMotion);
    m["sweepingMotion"] = f1(&SpiderController::sweepingMotion);
    m["foldingMotion"] = v0(&SpiderController::foldingMotion);
    return m;
  }();
  return h;
}
}  // namespace

std::vector<StepOutput> stepMotionAllIndices(Motion& motion)
{
  std::vector<StepOutput> out;
  const int saved_index = motion.motion_index;
  const bool saved_sent = motion.command_is_sent;
  for (int idx = 0; idx < motion.motion_index_num; idx++)
  {
    motion.motion_index = idx;
    motion.command_is_sent = false;
    StepOutput s;
    s.index = idx;
    s.time_threshold = motion.time_thresholds[idx];
    auto v = motion.getVelocityCommand();
    auto k = motion.getAccelerationCommand();
    auto m = motion.getMotionCommand();
    if (v) s.v = stripNewline(*v);
    if (k) s.k = stripNewline(*k);
    if (m) s.m = stripNewline(*m);
    out.push_back(s);
  }
  motion.motion_index = saved_index;
  motion.command_is_sent = saved_sent;
  return out;
}

bool runCall(SpiderController& controller, const std::string& call_in, std::string* error)
{
  std::string call = trim(call_in);
  size_t open = call.find_first_of("( ");
  std::string name = trim(call.substr(0, open));
  std::vector<std::string> args;
  if (open != std::string::npos)
  {
    std::string rest = call.substr(open);
    std::string cur;
    for (char ch : rest)
    {
      if (ch == '(' || ch == ')' || ch == ',' || std::isspace(static_cast<unsigned char>(ch)))
      {
        if (!cur.empty()) args.push_back(cur);
        cur.clear();
      }
      else
      {
        cur += ch;
      }
    }
    if (!cur.empty()) args.push_back(cur);
  }
  auto it = handlers().find(name);
  if (it == handlers().end())
  {
    if (error) *error = "unknown motion function '" + name + "' in case '" + call_in + "'";
    return false;
  }
  try
  {
    if (!it->second(controller, args))
    {
      if (error) *error = "bad arguments in '" + call_in + "'";
      return false;
    }
  }
  catch (const std::exception& e)
  {
    if (error) *error = "exception in '" + call_in + "': " + e.what();
    return false;
  }
  return true;
}

bool runCase(const std::string& case_name, std::vector<MotionOutput>& out, std::string* error)
{
  out.clear();
  SpiderController controller(nullptr, nullptr);
  std::string body = trim(case_name);
  if (body.rfind("seq[", 0) == 0 && body.size() >= 5 && body.back() == ']')
  {
    body = body.substr(4, body.size() - 5);  // "seq[a;b;c]" -> "a;b;c"
  }
  std::stringstream ss(body);
  std::string call;
  while (std::getline(ss, call, ';'))
  {
    if (trim(call).empty()) continue;
    if (!runCall(controller, call, error)) return false;
  }
  for (Motion& motion : controller.motion_list.motions)
  {
    MotionOutput mo;
    mo.motion_name = motion.motion_name;
    mo.steps = stepMotionAllIndices(motion);
    out.push_back(mo);
  }
  return true;
}

std::string formatCase(const std::string& case_name, const std::vector<MotionOutput>& motions)
{
  std::ostringstream os;
  os << "CASE " << case_name << "\n";
  for (const MotionOutput& mo : motions)
  {
    os << "MOTION " << mo.motion_name << "\n";
    for (const StepOutput& s : mo.steps)
    {
      os << "STEP " << s.index << " " << s.time_threshold << "\n";
      os << (s.v ? *s.v : "NULL") << "\n";
      os << (s.k ? *s.k : "NULL") << "\n";
      os << (s.m ? *s.m : "NULL") << "\n";
    }
  }
  return os.str();
}

std::vector<std::string> dumpRobot(const Robot& robot)
{
  using processing::str;
  auto fa = [](const auto& a) {
    std::string s;
    for (size_t i = 0; i < a.size(); i++) s += (i > 0 ? "," : "") + str(static_cast<float>(a[i]));
    return s;
  };
  auto ia = [](const std::array<int, 3>& a) {
    std::string s;
    for (size_t i = 0; i < a.size(); i++) s += (i > 0 ? "," : "") + str(a[i]);
    return s;
  };
  std::vector<std::string> out;
  out.push_back("ROBOT");
  for (size_t i = 0; i < robot.robot_legs.size(); i++)
  {
    const RobotLeg& l = robot.robot_legs[i];
    out.push_back("LEG " + std::to_string(i) + " motor_IDs=" + ia(l.motor_IDs) + " rad_thetas=" + fa(l.rad_thetas) +
                  " coor_rad_theta=" + str(l.coor_rad_theta) + " origin_steering=" + str(l.origin_steering) +
                  " origin_xyz=" + fa(l.origin_xyz));
  }
  for (size_t i = 0; i < robot.robot_hands.size(); i++)
  {
    const RobotHand& h = robot.robot_hands[i];
    out.push_back("HAND " + std::to_string(i) + " motor_IDs=" + ia(h.motor_IDs) + " rad_thetas=" + fa(h.rad_thetas) +
                  " coor_rad_theta=" + str(h.coor_rad_theta));
  }
  out.push_back("ENDROBOT");
  return out;
}

std::vector<std::string> defaultCases()
{
  std::vector<std::string> c;
  const char* steerings[] = {"PI/2", "0", "-PI/2"};
  c.push_back("testLegMotion(0)");
  c.push_back("testLegMotion(1)");
  c.push_back("testHandMotion(0)");
  c.push_back("testHandMotion(1)");
  for (const char* s : steerings)
  {
    for (int mode = 0; mode <= 5; mode++) c.push_back(std::string("walkingMotion(") + s + "," + std::to_string(mode) + ")");
    for (int mode = 0; mode <= 1; mode++) c.push_back(std::string("sideWalkingMotion(") + s + "," + std::to_string(mode) + ")");
    for (int rl = 0; rl <= 2; rl++) c.push_back(std::string("jumpingMotion(") + s + "," + std::to_string(rl) + ")");
    for (int rl = 0; rl <= 2; rl++) c.push_back(std::string("backJumpingMotion(") + s + "," + std::to_string(rl) + ")");
    c.push_back(std::string("rotationalMotion(") + s + ",PI/4)");
    c.push_back(std::string("rotationalMotion(") + s + ",-PI/4)");
    c.push_back(std::string("spreadMotion(") + s + ")");
    c.push_back(std::string("shrinkMotion(") + s + ")");
    c.push_back(std::string("resetMotion(") + s + ")");
  }
  // the four transitions reachable from gamepad_command
  c.push_back("rotationalMotion90(0,PI/2)");
  c.push_back("rotationalMotion90(-PI/2,0)");
  c.push_back("rotationalMotion90(0,-PI/2)");
  c.push_back("rotationalMotion90(PI/2,0)");
  c.push_back("wavingHandsMotion()");
  c.push_back("sweepingMotion(-20)");
  c.push_back("sweepingMotion(0)");
  c.push_back("sweepingMotion(20)");
  c.push_back("foldingMotion()");
  return c;
}

}  // namespace test_hook
}  // namespace shin_spider
