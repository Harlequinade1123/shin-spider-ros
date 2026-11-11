// Port of RobotLeg.pde (rendering removed).
#pragma once

#include <array>

#include "shin_spider/processing.h"

namespace shin_spider
{

class RobotLeg
{
public:
  std::array<int, 3> motor_IDs{{0, 0, 0}};
  std::array<float, 5> links{{0, 0, 0, 0, 0}};
  std::array<float, 3> xyz{{0, 0, 0}};
  std::array<float, 3> old_xyz{{0, 0, 0}};
  std::array<float, 3> origin_xyz{{0, 0, 0}};
  float origin_steering = 0;
  std::array<float, 3> rad_thetas{{0, 0, 0}};
  std::array<float, 3> old_rad_thetas{{0, 0, 0}};
  std::array<float, 3> rad_thetas_min{{-processing::PI, -7 * processing::PI / 12, -7 * processing::PI / 12}};
  std::array<float, 3> rad_thetas_max{{processing::PI, 7 * processing::PI / 12, 7 * processing::PI / 12}};
  float coor_rad_theta;  // rotation between RobotLeg frame and world frame

  bool is_set_parameter = false;
  int leg_mode = 0;  // elbow_up = 1, elbow_down = 0

  RobotLeg(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta);

  void initializeParameterFromThetas(const std::array<float, 5>& tmp_links, const std::array<float, 3>& tmp_thetas);
  void setThetas(const std::array<float, 3>& tmp_thetas);
  void calcFK(const std::array<float, 3>& target_rad_thetas);
  // Processing returns null when parameters are not set; here {0,0,0} is
  // returned after printing the same error (never happens after Robot()).
  std::array<float, 3> calcAndGetIK2DOF(float leg_rad_steering, float leg_height, float back_forth, int leg_mode);
};

}  // namespace shin_spider
