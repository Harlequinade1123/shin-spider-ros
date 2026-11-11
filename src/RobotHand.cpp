#include "shin_spider/RobotHand.h"

namespace shin_spider
{
using namespace processing;

RobotHand::RobotHand(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
{
  motor_IDs[0] = tmp_ID1;
  motor_IDs[1] = tmp_ID2;
  motor_IDs[2] = tmp_ID3;
  coor_rad_theta = tmp_coor_rad_theta;
}

void RobotHand::initializeParameterFromThetas(const std::array<float, 3>& tmp_links,
                                              const std::array<float, 3>& tmp_thetas)
{
  is_set_parameter = true;
  for (int i = 0; i < 3; i++)
  {
    links[i] = tmp_links[i];
  }
  for (int i = 0; i < 3; i++)
  {
    rad_thetas[i] = tmp_thetas[i];
    old_rad_thetas[i] = tmp_thetas[i];
  }
  calcFK(tmp_thetas);

  for (int i = 0; i < 3; i++)
  {
    rad_thetas[i] = tmp_thetas[i];
    if (rad_thetas_max[i] < rad_thetas[i])
    {
      rad_thetas[i] = rad_thetas_max[i];
    }
    if (rad_thetas_min[i] > rad_thetas[i])
    {
      rad_thetas[i] = rad_thetas_min[i];
    }
    old_rad_thetas[i] = rad_thetas[i];
    calcFK(rad_thetas);
  }
}

void RobotHand::setThetas(const std::array<float, 3>& tmp_thetas)
{
  for (int i = 0; i < 3; i++)
  {
    rad_thetas[i] = tmp_thetas[i];
    old_rad_thetas[i] = tmp_thetas[i];
  }

  for (int i = 0; i < 3; i++)
  {
    rad_thetas[i] = tmp_thetas[i];
    if (rad_thetas_max[i] < rad_thetas[i])
    {
      rad_thetas[i] = rad_thetas_max[i];
    }
    if (rad_thetas_min[i] > rad_thetas[i])
    {
      rad_thetas[i] = rad_thetas_min[i];
    }
    old_rad_thetas[i] = rad_thetas[i];
  }

  calcFK(rad_thetas);
}

void RobotHand::calcFK(const std::array<float, 3>& target_rad_thetas)
{
  if (is_set_parameter == false)
  {
    println("ERROR: Parameter is not set");
    return;
  }
  float sinTh0 = sin(target_rad_thetas[0]);
  float sinTh1 = sin(target_rad_thetas[1]);
  float cosTh0 = cos(target_rad_thetas[0]);
  float cosTh1 = cos(target_rad_thetas[1]);

  // forward kinematics without the root link
  xyz[0] = cosTh0 * links[1] + cosTh0 * cosTh1 * links[2];
  xyz[1] = sinTh0 * links[1] + sinTh0 * cosTh1 * links[2];
  xyz[2] = -sinTh1 * links[2];
}

std::array<float, 3> RobotHand::calcAndGetThetasFromRatio(float ratio_back_forth, float ratio_up_down,
                                                          float ratio_end_effector)
{
  std::array<float, 3> ans_rad_thetas{{0, 0, 0}};

  ans_rad_thetas[0] = PI * ratio_back_forth / 200;
  if (coor_rad_theta > 0)
  {
    ans_rad_thetas[0] *= -1;
  }

  ans_rad_thetas[1] = -PI * ratio_up_down / 200;

  ans_rad_thetas[2] = PI * ratio_end_effector / 100;
  if (coor_rad_theta > 0)
  {
    ans_rad_thetas[2] *= -1;
  }

  // NOTE: the original clamp loop is `for (int i = 0; i < 0; i++)` (never runs). Kept as no-op.

  return ans_rad_thetas;
}

}  // namespace shin_spider
