// Maps sensor_msgs/Joy style arrays (axes: float, buttons: int) onto the
// gamepad.pde variables of SpiderController (ROS-free).
//
// Processing semantics reproduced:
//  - slider_* = int(value * 100)       (Java (int) truncation)
//  - button_* += 1 on press, -= 1 on release, control_counter++ on press
//  - hat_XY: gamecontrolplus POV code 0..8
//      0 = centre, 1 = up-left, 2 = up, 3 = up-right, 4 = right,
//      5 = down-right, 6 = down, 7 = down-left, 8 = left
#pragma once

#include <string>
#include <vector>

namespace shin_spider
{

class SpiderController;

struct AxisMapping
{
  int index = -1;      // -1: unused (value 0)
  float scale = 100;   // slider = int(axes[index] * scale)
};

struct JoyMapping
{
  // Defaults: Xbox 360 / Xbox One pad with the Linux xpad driver + joy_node.
  //   axes: 0 LS left(+)/right(-), 1 LS up(+)/down(-), 2 LT (1 released .. -1 pressed),
  //         3 RS left(+)/right(-), 4 RS up(+)/down(-), 5 RT, 6 D-pad left(+)/right(-),
  //         7 D-pad up(+)/down(-)
  //   buttons: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 Guide, 9 LS click, 10 RS click
  // Processing (DirectInput) conventions: slider_LX/RX = stick "U-D" positive DOWN,
  // slider_LY/RY = stick "L-R" positive RIGHT, slider_FR = +L trigger / -R trigger.
  AxisMapping slider_LX{1, -100};
  AxisMapping slider_LY{0, -100};
  AxisMapping slider_RX{4, -100};
  AxisMapping slider_RY{3, -100};

  // slider_FR: "triggers" mode = (pressL - pressR) * fr_scale, press in [0,1]
  // computed from trigger axes resting at +1 and fully pressed at -1.
  // "axis" mode = int(axes[fr_axis.index] * fr_axis.scale).
  std::string fr_mode = "triggers";
  int fr_left_trigger_axis = 2;
  int fr_right_trigger_axis = 5;
  float fr_scale = 100;
  AxisMapping fr_axis{-1, 100};

  int button_A = 0, button_B = 1, button_X = 2, button_Y = 3;
  int button_LB = 4, button_RB = 5, button_Back = 6, button_Start = 7;
  int button_LA = 9, button_RA = 10;

  // Hat from two axes ("axes" mode) or four buttons ("buttons" mode).
  std::string hat_mode = "axes";
  int hat_axis_x = 6;           // positive = left (joy convention)
  int hat_axis_y = 7;           // positive = up
  bool hat_x_left_positive = true;
  bool hat_y_up_positive = true;
  float hat_threshold = 0.5f;
  int hat_button_up = -1, hat_button_down = -1, hat_button_left = -1, hat_button_right = -1;
};

class JoyMapper
{
public:
  explicit JoyMapper(const JoyMapping& mapping = JoyMapping()) : mapping_(mapping) {}

  // Apply one Joy message to the controller's gamepad variables.
  void apply(const std::vector<float>& axes, const std::vector<int>& buttons, SpiderController& controller);

  const JoyMapping& mapping() const { return mapping_; }

  static int hatCode(bool up, bool down, bool left, bool right);

private:
  float axis(const std::vector<float>& axes, int index) const;
  bool button(const std::vector<int>& buttons, int index) const;
  void applyButton(bool pressed, bool& prev, int& value, int& control_counter);
  float triggerPress(const std::vector<float>& axes, int index, bool& seen);

  JoyMapping mapping_;
  bool prev_[10] = {false, false, false, false, false, false, false, false, false, false};
  bool left_trigger_seen_ = false;
  bool right_trigger_seen_ = false;
};

}  // namespace shin_spider
