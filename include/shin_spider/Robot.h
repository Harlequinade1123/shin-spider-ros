// Port of Robot.pde (rendering removed) plus the sketch-level robot parameters
// declared as globals in original_spider_controller.pde.
#pragma once

#include <array>
#include <vector>

#include "shin_spider/RobotHand.h"
#include "shin_spider/RobotLeg.h"

namespace shin_spider
{

// Globals of original_spider_controller.pde
namespace sketch
{
const std::array<float, 5> leg_links{{35, 20, 30, 70, 70}};
const std::array<float, 3> leg_thetas{{processing::PI / 4, 0, 0}};
const std::array<float, 3> hand_links{{35, 70, 70}};
const std::array<float, 3> hand_thetas{{0, 0, 0}};
const int body = 70;
const int leg_num = 4;
const int hand_num = 2;
const float frame_rate = 100;
}  // namespace sketch

class Robot
{
public:
  float body_size;
  std::vector<RobotLeg> robot_legs;
  std::vector<RobotHand> robot_hands;

  Robot(float tmp_body_size, int tmp_leg_number, int tmp_hand_number);

  int findLegNumFromIds(const std::array<int, 3>& ids) const;
  int findHandNumFromIds(const std::array<int, 3>& ids) const;

  // Processing mutates the global leg_thetas (sign flip per leg); kept here.
  std::array<float, 3> leg_thetas = sketch::leg_thetas;
};

}  // namespace shin_spider
