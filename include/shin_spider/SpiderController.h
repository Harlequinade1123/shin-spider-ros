// Sketch-level state of original_spider_controller.pde: global robot /
// motion_list / gamepad variables, leg_motion_commands, hand_motion_commands,
// gamepad_command and the motion-update part of draw(). Rendering omitted.
#pragma once

#include <array>
#include <string>

#include "shin_spider/MotionList.h"
#include "shin_spider/Robot.h"
#include "shin_spider/processing.h"

namespace shin_spider
{

class SpiderController
{
public:
  SpiderController(Clock* clock, CommandSink* sink);
  SpiderController(const SpiderController&) = delete;
  SpiderController& operator=(const SpiderController&) = delete;

  // ---- globals of the sketch ----
  Robot robot;
  MotionList motion_list;

  // ---- gamepad.pde variables (same semantics) ----
  int slider_LX = 0, slider_LY = 0, slider_RX = 0, slider_RY = 0, slider_FR = 0;
  int button_A = 0, button_B = 0, button_X = 0, button_Y = 0;
  int button_LB = 0, button_RB = 0, button_Back = 0, button_Start = 0, button_LA = 0, button_RA = 0;
  int hat_XY = 0;
  int control_counter = 0;
  int pred_hat_XY = 0;

  // ---- gamepad_command.pde state ----
  int slider_x_threshold = 20;
  int slider_y_threshold = 40;
  int slider_f_threshold = 50;
  int pred_slider_front = 50;
  int walking_state_L = 0;
  int steering_state = 1;
  float robot_steering = 0;
  std::array<int, 4> walking_time_counts{{0, 0, 0, 0}};
  int walking_time_threshold = 20;

  // ---- gamepad_command.pde ----
  void readGamePadCommand();

  // ---- draw(): motion update + send v,k,m (2 ms gaps) + robot model update ----
  void updateMotions();

  // readGamePadCommand() then updateMotions(), as one Processing frame
  void frame();

  // ---- leg_motion_commands.pde ----
  void testLegMotion(int leg);
  void walkingMotion(float steering, int mode);
  void sideWalkingMotion(float steering, int mode);
  void jumpingMotion(float steering, int right_left);
  void backJumpingMotion(float steering, int right_left);
  void rotationalMotion(float steering, float delta_rotate);
  void rotationalMotion90(float steering, float pred_steering);
  void spreadMotion(float steering);
  void shrinkMotion(float steering);
  void resetMotion(float steering);

  // ---- hand_motion_commands.pde ----
  void testHandMotion(int hand);
  void wavingHandsMotion();
  void sweepingMotion(float angle);
  void foldingMotion();

  void setSink(CommandSink* sink) { sink_ = sink; }

private:
  void write(const std::string& data);
  void delay(int ms);

  Clock* clock_;
  CommandSink* sink_;
};

}  // namespace shin_spider
