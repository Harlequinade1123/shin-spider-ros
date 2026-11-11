// Port of RobotHand.pde (rendering removed).
#pragma once

#include <array>

#include "shin_spider/processing.h"

namespace shin_spider
{

class RobotHand
{
public:
  std::array<int, 3> motor_IDs{{0, 0, 0}};
  std::array<float, 3> links{{0, 0, 0}};
  std::array<float, 3> xyz{{0, 0, 0}};
  std::array<float, 3> rad_thetas{{0, 0, 0}};
  std::array<float, 3> old_rad_thetas{{0, 0, 0}};
  std::array<float, 3> rad_thetas_min{{-processing::PI / 2, -processing::PI / 2, -processing::PI}};
  std::array<float, 3> rad_thetas_max{{processing::PI / 2, processing::PI / 2, processing::PI}};
  float coor_rad_theta;

  bool is_set_parameter = false;

  RobotHand(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta);

  void initializeParameterFromThetas(const std::array<float, 3>& tmp_links, const std::array<float, 3>& tmp_thetas);
  void setThetas(const std::array<float, 3>& tmp_thetas);
  void calcFK(const std::array<float, 3>& target_rad_thetas);
  std::array<float, 3> calcAndGetThetasFromRatio(float ratio_back_forth, float ratio_up_down,
                                                 float ratio_end_effector);
};

}  // namespace shin_spider
