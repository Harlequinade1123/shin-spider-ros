#include "shin_spider/JoyMapper.h"

#include "shin_spider/SpiderController.h"
#include "shin_spider/processing.h"

namespace shin_spider
{

float JoyMapper::axis(const std::vector<float>& axes, int index) const
{
  if (index < 0 || index >= static_cast<int>(axes.size()))
  {
    return 0.0f;
  }
  return axes[index];
}

bool JoyMapper::button(const std::vector<int>& buttons, int index) const
{
  if (index < 0 || index >= static_cast<int>(buttons.size()))
  {
    return false;
  }
  return buttons[index] != 0;
}

void JoyMapper::applyButton(bool pressed, bool& prev, int& value, int& control_counter)
{
  if (pressed && !prev)
  {
    value += 1;  // X_ButtonPress()
    control_counter++;
  }
  else if (!pressed && prev)
  {
    value -= 1;  // X_ButtonRelease()
  }
  prev = pressed;
}

// joy_node reports 0.0 for a trigger axis until it is touched for the first
// time; treat that as "released" (+1).
float JoyMapper::triggerPress(const std::vector<float>& axes, int index, bool& seen)
{
  float raw = axis(axes, index);
  if (!seen)
  {
    if (raw == 0.0f)
    {
      raw = 1.0f;
    }
    else
    {
      seen = true;
    }
  }
  return (1.0f - raw) / 2.0f;
}

int JoyMapper::hatCode(bool up, bool down, bool left, bool right)
{
  if (up && down)
  {
    up = down = false;
  }
  if (left && right)
  {
    left = right = false;
  }
  if (up && left) return 1;
  if (up && right) return 3;
  if (down && right) return 5;
  if (down && left) return 7;
  if (up) return 2;
  if (right) return 4;
  if (down) return 6;
  if (left) return 8;
  return 0;
}

void JoyMapper::apply(const std::vector<float>& axes, const std::vector<int>& buttons, SpiderController& c)
{
  using processing::toInt;
  const JoyMapping& m = mapping_;

  c.slider_LX = toInt(axis(axes, m.slider_LX.index) * m.slider_LX.scale);
  c.slider_LY = toInt(axis(axes, m.slider_LY.index) * m.slider_LY.scale);
  c.slider_RX = toInt(axis(axes, m.slider_RX.index) * m.slider_RX.scale);
  c.slider_RY = toInt(axis(axes, m.slider_RY.index) * m.slider_RY.scale);
  if (m.fr_mode == "axis")
  {
    c.slider_FR = toInt(axis(axes, m.fr_axis.index) * m.fr_axis.scale);
  }
  else
  {
    float l = triggerPress(axes, m.fr_left_trigger_axis, left_trigger_seen_);
    float r = triggerPress(axes, m.fr_right_trigger_axis, right_trigger_seen_);
    c.slider_FR = toInt((l - r) * m.fr_scale);
  }

  applyButton(button(buttons, m.button_A), prev_[0], c.button_A, c.control_counter);
  applyButton(button(buttons, m.button_B), prev_[1], c.button_B, c.control_counter);
  applyButton(button(buttons, m.button_X), prev_[2], c.button_X, c.control_counter);
  applyButton(button(buttons, m.button_Y), prev_[3], c.button_Y, c.control_counter);
  applyButton(button(buttons, m.button_LB), prev_[4], c.button_LB, c.control_counter);
  applyButton(button(buttons, m.button_RB), prev_[5], c.button_RB, c.control_counter);
  applyButton(button(buttons, m.button_Back), prev_[6], c.button_Back, c.control_counter);
  applyButton(button(buttons, m.button_Start), prev_[7], c.button_Start, c.control_counter);
  applyButton(button(buttons, m.button_LA), prev_[8], c.button_LA, c.control_counter);
  applyButton(button(buttons, m.button_RA), prev_[9], c.button_RA, c.control_counter);

  bool up, down, left, right;
  if (m.hat_mode == "buttons")
  {
    up = button(buttons, m.hat_button_up);
    down = button(buttons, m.hat_button_down);
    left = button(buttons, m.hat_button_left);
    right = button(buttons, m.hat_button_right);
  }
  else
  {
    float x = axis(axes, m.hat_axis_x);
    float y = axis(axes, m.hat_axis_y);
    if (!m.hat_x_left_positive) x = -x;
    if (!m.hat_y_up_positive) y = -y;
    left = x > m.hat_threshold;
    right = x < -m.hat_threshold;
    up = y > m.hat_threshold;
    down = y < -m.hat_threshold;
  }
  int code = hatCode(up, down, left, right);
  // HatPress (WHILE_PRESS) / HatRelease semantics
  c.hat_XY = code;
  if (code != 0)
  {
    if (c.pred_hat_XY != c.hat_XY)
    {
      c.control_counter++;
    }
    c.pred_hat_XY = c.hat_XY;
  }
}

}  // namespace shin_spider
