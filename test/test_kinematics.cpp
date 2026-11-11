// Sanity tests: Processing compat, kinematics / IK round trip, motion state
// machine, gamepad logic, firmware-format checks of every generated line.
#include <cmath>
#include <iostream>
#include <set>

#include <gtest/gtest.h>

#include "shin_spider/FirmwareProtocol.h"
#include "shin_spider/JoyMapper.h"
#include "shin_spider/MotionTestHook.h"
#include "shin_spider/SpiderController.h"

using namespace shin_spider;
using processing::PI;

namespace
{
class FakeClock : public Clock
{
public:
  int now = 0;
  int millis() override { return now; }
};

class RecordingSink : public CommandSink
{
public:
  std::vector<std::string> lines;
  int delayed = 0;
  void write(const std::string& d) override { lines.push_back(d); }
  void delay(int ms) override { delayed += ms; }
};

struct Quiet
{
  Quiet() { processing::setPrintlnHandler([](const std::string&) {}); }
} quiet;
}  // namespace

TEST(Processing, FloatToStringLikeJava)
{
  EXPECT_EQ(processing::str(0.0f), "0.0");
  EXPECT_EQ(processing::str(-20.0f), "-20.0");
  EXPECT_EQ(processing::str(20.0f), "20.0");
  EXPECT_EQ(processing::str(PI / 2), "1.5707964");
  EXPECT_EQ(processing::str(PI), "3.1415927");
  EXPECT_EQ(processing::str(0.1f), "0.1");
  EXPECT_EQ(processing::str(100.5f), "100.5");
  EXPECT_EQ(processing::str(0.001f), "0.001");
  EXPECT_EQ(processing::str(1.0e-4f), "1.0E-4");
  EXPECT_EQ(processing::str(1.0e7f), "1.0E7");
  EXPECT_EQ(processing::str(1234567.0f), "1234567.0");
}

TEST(Processing, IntCastAndMap)
{
  EXPECT_EQ(processing::toInt(NAN), 0);
  EXPECT_EQ(processing::toInt(-1.9f), -1);
  EXPECT_EQ(processing::toInt(1.9f), 1);
  EXPECT_EQ(processing::toInt(processing::map(-PI, -PI, PI, 0, 4095)), 0);
  EXPECT_EQ(processing::toInt(processing::map(PI, -PI, PI, 0, 4095)), 4095);
  EXPECT_EQ(processing::toInt(processing::map(0, -PI, PI, 0, 4095)), 2047);
  EXPECT_EQ(processing::toInt(processing::degrees(PI / 2)), 90);
  EXPECT_EQ(processing::toInt(processing::degrees(-PI / 2)), -90);
  EXPECT_EQ(processing::toInt(processing::degrees(PI / 4)), 45);
}

TEST(Robot, ConstructionMatchesProcessing)
{
  Robot robot(70, 4, 2);
  ASSERT_EQ(robot.robot_legs.size(), 4u);
  ASSERT_EQ(robot.robot_hands.size(), 2u);
  for (int i = 0; i < 4; i++)
  {
    EXPECT_EQ(robot.robot_legs[i].motor_IDs[0], 1 + 3 * i);
    EXPECT_EQ(robot.robot_legs[i].motor_IDs[2], 3 + 3 * i);
    EXPECT_FLOAT_EQ(robot.robot_legs[i].coor_rad_theta, PI * (2 * i - 1) / 4);
    // leg_thetas[0] *= -1 before each leg: -PI/4, PI/4, -PI/4, PI/4
    EXPECT_FLOAT_EQ(robot.robot_legs[i].origin_steering, (i % 2 == 0) ? -PI / 4 : PI / 4);
    EXPECT_FLOAT_EQ(robot.robot_legs[i].origin_xyz[2], -190.0f);
  }
  EXPECT_EQ(robot.robot_hands[0].motor_IDs[0], 13);
  EXPECT_EQ(robot.robot_hands[1].motor_IDs[2], 18);
  EXPECT_FLOAT_EQ(robot.robot_hands[0].coor_rad_theta, -PI / 2);
  EXPECT_FLOAT_EQ(robot.robot_hands[1].coor_rad_theta, PI / 2);
  EXPECT_EQ(robot.findLegNumFromIds({{7, 8, 9}}), 2);
  EXPECT_EQ(robot.findHandNumFromIds({{16, 17, 18}}), 1);
  EXPECT_EQ(robot.findLegNumFromIds({{16, 17, 18}}), -1);
}

// IK (calcAndGetIK2DOF) -> FK (calcFK) round trip in the leg plane.
TEST(Kinematics, IkFkRoundTrip)
{
  Robot robot(70, 4, 2);
  int checked = 0;
  for (int leg = 0; leg < 4; leg++)
  {
    for (int mode = 0; mode <= 1; mode++)
    {
      for (float height = 0; height <= 90; height += 10)
      {
        for (float forth = -80; forth <= 110; forth += 10)
        {
          RobotLeg& l = robot.robot_legs[leg];
          float hz = height + l.origin_xyz[2];
          float bf = forth + l.origin_xyz[0];
          if (l.coor_rad_theta > PI / 2) bf *= -1;
          float zz = hz + l.links[1] + l.links[2];
          float c2 = (bf * bf + zz * zz - l.links[3] * l.links[3] - l.links[4] * l.links[4]) /
                     (2 * l.links[3] * l.links[4]);
          if (std::fabs(c2) >= 1.0f) continue;  // unreachable

          std::array<float, 3> th = l.calcAndGetIK2DOF(0.3f, height, forth, mode);
          bool within = true;
          for (int j = 0; j < 3; j++)
          {
            within = within && th[j] <= l.rad_thetas_max[j] && th[j] >= l.rad_thetas_min[j];
          }
          if (!within) continue;  // would be clamped by setThetas
          l.calcFK(th);
          float r = -(std::sin(th[1]) * l.links[3] + std::sin(th[1] + th[2]) * l.links[4]);
          EXPECT_NEAR(r, bf, 1e-2) << "leg " << leg << " h " << height << " f " << forth;
          EXPECT_NEAR(l.xyz[2], hz, 1e-2) << "leg " << leg << " h " << height << " f " << forth;
          EXPECT_NEAR(std::hypot(l.xyz[0], l.xyz[1]), std::fabs(bf), 1e-2);
          EXPECT_NEAR(th[0], 0.3f + l.origin_steering, 1e-6);
          checked++;
        }
      }
    }
  }
  EXPECT_GT(checked, 200);
}

TEST(Kinematics, HandRatio)
{
  Robot robot(70, 4, 2);
  auto a = robot.robot_hands[0].calcAndGetThetasFromRatio(50, 100, 50);
  EXPECT_FLOAT_EQ(a[0], PI * 50 / 200);
  EXPECT_FLOAT_EQ(a[1], -PI * 100 / 200);
  EXPECT_FLOAT_EQ(a[2], PI * 50 / 100);
  auto b = robot.robot_hands[1].calcAndGetThetasFromRatio(50, 100, 50);  // coor > 0: mirrored
  EXPECT_FLOAT_EQ(b[0], -a[0]);
  EXPECT_FLOAT_EQ(b[1], a[1]);
  EXPECT_FLOAT_EQ(b[2], -a[2]);
}

TEST(Motion, MainLoopTimingAndOrder)
{
  FakeClock clock;
  RecordingSink sink;
  SpiderController c(&clock, &sink);
  c.wavingHandsMotion();
  ASSERT_EQ(c.motion_list.motions.size(), 1u);
  ASSERT_TRUE(c.motion_list.motions[0].is_moving);

  c.updateMotions();  // t=0: step 0 sent
  ASSERT_EQ(sink.lines.size(), 3u);
  EXPECT_EQ(sink.lines[0].substr(0, 2), "[v");
  EXPECT_EQ(sink.lines[1].substr(0, 2), "[k");
  EXPECT_EQ(sink.lines[2].substr(0, 2), "[m");
  EXPECT_EQ(sink.delayed, 6);
  clock.now = 29;
  c.updateMotions();
  EXPECT_EQ(sink.lines.size(), 3u);
  clock.now = 30;
  c.updateMotions();  // step 1
  ASSERT_EQ(sink.lines.size(), 6u);
  EXPECT_NE(sink.lines[2], sink.lines[5]);
  clock.now = 60;
  c.updateMotions();  // index -> num, nothing sent
  EXPECT_EQ(sink.lines.size(), 6u);
  EXPECT_TRUE(c.motion_list.motions[0].is_moving);
  c.updateMotions();
  EXPECT_FALSE(c.motion_list.motions[0].is_moving);

  // hands model updated from last target
  EXPECT_FLOAT_EQ(c.robot.robot_hands[0].rad_thetas[1], -PI * 90 / 200);
}

TEST(Motion, CanStartBlocksSharedIds)
{
  FakeClock clock;
  RecordingSink sink;
  SpiderController c(&clock, &sink);
  c.walkingMotion(0, 0);
  c.walkingMotion(0, 3);  // created; ids busy -> not started
  ASSERT_EQ(c.motion_list.motions.size(), 2u);
  EXPECT_TRUE(c.motion_list.motions[0].is_moving);
  EXPECT_TRUE(c.motion_list.motions[1].is_moving);  // motionStart() always starts newly created motions
  c.wavingHandsMotion();
  EXPECT_TRUE(c.motion_list.motions[2].is_moving);
  // reset clears everything except "reset_motion"
  c.resetMotion(0);
  ASSERT_EQ(c.motion_list.motions.size(), 2u);
  EXPECT_EQ(c.motion_list.motions[1].motion_name, "reset_motion");
}

TEST(Gamepad, ButtonsAndHat)
{
  FakeClock clock;
  RecordingSink sink;
  SpiderController c(&clock, &sink);
  JoyMapper mapper;
  std::vector<float> axes(8, 0.0f);
  std::vector<int> buttons(11, 0);

  axes[1] = 1.0f;   // LS up -> slider_LX = -100 (Processing "L up")
  axes[0] = -0.5f;  // LS right -> slider_LY = 50
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.slider_LX, -100);
  EXPECT_EQ(c.slider_LY, 50);
  EXPECT_EQ(c.slider_FR, 0);  // untouched triggers (joy_node reports 0) -> released

  axes = std::vector<float>(8, 0.0f);
  axes[2] = -1.0f;  // LT fully pressed
  axes[5] = 1.0f;
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.slider_FR, 100);

  buttons[0] = 1;
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.button_A, 1);
  EXPECT_EQ(c.control_counter, 1);
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.button_A, 1);
  EXPECT_EQ(c.control_counter, 1);
  buttons[0] = 0;
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.button_A, 0);

  axes[7] = 1.0f;  // up
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.hat_XY, 2);
  axes[6] = -1.0f;  // + right
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.hat_XY, 3);
  axes[7] = 0;
  axes[6] = 1.0f;  // left
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.hat_XY, 8);
  axes[6] = 0;
  axes[7] = -1.0f;
  mapper.apply(axes, buttons, c);
  EXPECT_EQ(c.hat_XY, 6);
}

TEST(Gamepad, LbRbSendsTorqueOn)
{
  FakeClock clock;
  RecordingSink sink;
  SpiderController c(&clock, &sink);
  c.button_LB = 1;
  c.button_RB = 1;
  c.button_A = 1;
  c.readGamePadCommand();
  ASSERT_EQ(sink.lines.size(), 1u);
  EXPECT_EQ(sink.lines[0], "[n]\n");
  EXPECT_TRUE(c.motion_list.motions.empty());  // returns before buttons
  EXPECT_TRUE(firmware::validateLine(sink.lines[0]).ok);
}

TEST(Gamepad, SteeringStateMachine)
{
  FakeClock clock;
  RecordingSink sink;
  SpiderController c(&clock, &sink);
  c.slider_FR = 100;
  c.readGamePadCommand();
  EXPECT_EQ(c.steering_state, 2);
  EXPECT_FLOAT_EQ(c.robot_steering, -PI / 2);
  ASSERT_EQ(c.motion_list.motions.size(), 1u);
  EXPECT_EQ(c.motion_list.motions[0].motion_name, "rotational+_-90_0");
  c.readGamePadCommand();  // held: no new transition
  EXPECT_EQ(c.steering_state, 2);
  c.slider_FR = 0;
  c.readGamePadCommand();
  c.slider_FR = -100;
  c.readGamePadCommand();  // rotation still moving -> blocked
  EXPECT_EQ(c.steering_state, 2);
}

TEST(Firmware, ParserPort)
{
  auto p = firmware::parseLikeFirmware("[m,1,2047,2,100]");
  EXPECT_TRUE(p.accepted);
  EXPECT_EQ(p.command, 'm');
  EXPECT_EQ(p.num, 5);
  ASSERT_EQ(p.elements.size(), 5u);
  EXPECT_EQ(p.elements[0], "m");
  EXPECT_EQ(p.elements[4], "100");
  EXPECT_FALSE(firmware::parseLikeFirmware("[q,1,2]").accepted);
  EXPECT_FALSE(firmware::parseLikeFirmware("m,1,2]").accepted);
  EXPECT_FALSE(firmware::validateLine("[m,1,5000]").ok);
  EXPECT_FALSE(firmware::validateLine("[m,19,100]").ok);
  EXPECT_FALSE(firmware::validateLine("[v,1]").ok);
}

// Every line produced by every motion must be accepted by the firmware parser
// with values inside the firmware's DXL_MIN/MAX ranges. Position-limit-register
// violations are reported (not failed).
TEST(Firmware, AllGeneratedLinesValid)
{
  std::set<std::string> warnings;
  for (const std::string& name : test_hook::defaultCases())
  {
    std::vector<test_hook::MotionOutput> out;
    std::string err;
    ASSERT_TRUE(test_hook::runCase(name, out, &err)) << err;
    for (auto& mo : out)
    {
      for (auto& s : mo.steps)
      {
        ASSERT_TRUE(s.v && s.k && s.m) << name;
        for (const std::string* line : {&*s.v, &*s.k, &*s.m})
        {
          auto v = firmware::validateLine(*line);
          EXPECT_TRUE(v.ok) << name << ": " << *line << " : " << (v.errors.empty() ? "" : v.errors[0]);
          for (auto& w : v.warnings) warnings.insert(mo.motion_name + " step " + std::to_string(s.index) + ": " + w);
        }
      }
    }
  }
  std::cout << "[ INFO     ] position-limit-register warnings: " << warnings.size() << std::endl;
  for (auto& w : warnings) std::cout << "[ INFO     ]   " << w << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
