#include "shin_spider/Robot.h"

namespace shin_spider
{
using namespace processing;

Robot::Robot(float tmp_body_size, int tmp_leg_number, int tmp_hand_number)
{
  body_size = tmp_body_size;

  for (int i = 0; i < tmp_leg_number; i++)
  {
    RobotLeg robot_leg(1 + i * 3, 2 + i * 3, 3 + i * 3, PI * (2 * i - 1) / 4);
    robot_legs.push_back(robot_leg);
    leg_thetas[0] *= -1;
    robot_legs[i].initializeParameterFromThetas(sketch::leg_links, leg_thetas);
  }
  for (int i = 0; i < tmp_hand_number; ++i)
  {
    RobotHand robot_hand(1 + i * 3 + tmp_leg_number * 3,
                         2 + i * 3 + tmp_leg_number * 3,
                         3 + i * 3 + tmp_leg_number * 3, PI * (2 * i - 1) / 2);
    robot_hands.push_back(robot_hand);
    robot_hands[i].initializeParameterFromThetas(sketch::hand_links, sketch::hand_thetas);
  }
}

int Robot::findLegNumFromIds(const std::array<int, 3>& ids) const
{
  int ans_num = -1;
  for (size_t i = 0; i < robot_legs.size(); i++)
  {
    if (robot_legs[i].motor_IDs[0] == ids[0] &&
        robot_legs[i].motor_IDs[1] == ids[1] &&
        robot_legs[i].motor_IDs[2] == ids[2])
    {
      ans_num = static_cast<int>(i);
      break;
    }
  }
  return ans_num;
}

int Robot::findHandNumFromIds(const std::array<int, 3>& ids) const
{
  int ans_num = -1;
  for (size_t i = 0; i < robot_hands.size(); i++)
  {
    if (robot_hands[i].motor_IDs[0] == ids[0] &&
        robot_hands[i].motor_IDs[1] == ids[1] &&
        robot_hands[i].motor_IDs[2] == ids[2])
    {
      ans_num = static_cast<int>(i);
      break;
    }
  }
  return ans_num;
}

}  // namespace shin_spider
