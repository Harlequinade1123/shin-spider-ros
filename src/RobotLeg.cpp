#include "shin_spider/RobotLeg.h"

namespace shin_spider
{
using namespace processing;

RobotLeg::RobotLeg(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
{
  motor_IDs[0] = tmp_ID1;
  motor_IDs[1] = tmp_ID2;
  motor_IDs[2] = tmp_ID3;
  coor_rad_theta = tmp_coor_rad_theta;
}

void RobotLeg::initializeParameterFromThetas(const std::array<float, 5>& tmp_links,
                                             const std::array<float, 3>& tmp_thetas)
{
  is_set_parameter = true;
  for (int i = 0; i < 5; i++)
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

  origin_xyz[0] = 0;
  origin_xyz[1] = 0;
  origin_xyz[2] = -links[1] - links[2] - links[3] - links[4];
  origin_steering = rad_thetas[0];
}

void RobotLeg::setThetas(const std::array<float, 3>& tmp_thetas)
{
  for (int i = 0; i < 3; i++)
  {
    rad_thetas[i] = tmp_thetas[i];
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

void RobotLeg::calcFK(const std::array<float, 3>& target_rad_thetas)
{
  if (is_set_parameter == false)
  {
    println("ERROR: Parameter is not set");
    return;
  }
  float sinTh0 = sin(target_rad_thetas[0]);
  float sinTh1 = sin(target_rad_thetas[1]);
  float sinTh2 = sin(target_rad_thetas[2]);
  float cosTh0 = cos(target_rad_thetas[0]);
  float cosTh1 = cos(target_rad_thetas[1]);
  float cosTh2 = cos(target_rad_thetas[2]);

  // forward kinematics without root link L0
  // L1 + E^(-k th0) * (L2 * E^(j th1)(L3 + E^(j th2)(L4)))
  xyz[0] = -cosTh0 * sinTh1 * links[3] - cosTh0 * (sinTh1 * cosTh2 + cosTh1 * sinTh2) * links[4];
  xyz[1] = sinTh0 * sinTh1 * links[3] + sinTh0 * (sinTh1 * cosTh2 + cosTh1 * sinTh2) * links[4];
  xyz[2] = -links[1] - links[2] - cosTh1 * links[3] - (cosTh1 * cosTh2 - sinTh1 * sinTh2) * links[4];
}

std::array<float, 3> RobotLeg::calcAndGetIK2DOF(float leg_rad_steering, float leg_height, float back_forth,
                                                int leg_mode)
{
  if (is_set_parameter == false)
  {
    println("ERROR: Parameter is not set");
    return {{0, 0, 0}};
  }

  float leg_height_from_origin = leg_height + origin_xyz[2];
  float leg_back_forth_from_origin = back_forth + origin_xyz[0];
  if (coor_rad_theta > PI / 2)
  {
    leg_back_forth_from_origin *= -1;
  }

  std::array<float, 3> ans_rad_thetas{{0, 0, 0}};
  ans_rad_thetas[0] = leg_rad_steering + origin_steering;

  float cosTh2 = leg_back_forth_from_origin * leg_back_forth_from_origin;
  cosTh2 += (leg_height_from_origin + links[1] + links[2]) * (leg_height_from_origin + links[1] + links[2]);
  cosTh2 -= links[3] * links[3];
  cosTh2 -= links[4] * links[4];
  cosTh2 /= 2 * links[3] * links[4];
  float sinTh2;

  if (coor_rad_theta < PI / 2)
  {
    sinTh2 = -sqrt(1 - cosTh2 * cosTh2);
    if (leg_mode == 1)
    {
      sinTh2 *= -1;
    }
  }
  else
  {
    sinTh2 = sqrt(1 - cosTh2 * cosTh2);
    if (leg_mode == 1)
    {
      sinTh2 *= -1;
    }
  }

  ans_rad_thetas[2] = atan2(sinTh2, cosTh2);

  float A = -leg_back_forth_from_origin;
  float B = -(leg_height_from_origin + links[1] + links[2]);
  float M = links[3] + cosTh2 * links[4];
  float N = sinTh2 * links[4];
  ans_rad_thetas[1] = atan2(M * A - N * B, N * A + M * B);

  // NOTE: faithfully ported; the original compares the member rad_thetas
  // (not ans_rad_thetas) against the limits.
  for (int i = 0; i < 3; i++)
  {
    if (rad_thetas_max[i] < rad_thetas[i])
    {
      ans_rad_thetas[i] = rad_thetas_max[i];
    }
    if (rad_thetas_min[i] > rad_thetas[i])
    {
      ans_rad_thetas[i] = rad_thetas_min[i];
    }
    if (isNaN(ans_rad_thetas[i]))
    {
      ans_rad_thetas[i] = 0;
    }
  }

  return ans_rad_thetas;
}

}  // namespace shin_spider
