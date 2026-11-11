// Port of MotionList.pde. The Processing version uses the global `robot`;
// here the Robot is injected.
#pragma once

#include <string>
#include <vector>

#include "shin_spider/Motion.h"
#include "shin_spider/Robot.h"
#include "shin_spider/Target.h"

namespace shin_spider
{

class MotionList
{
public:
  std::vector<Motion> motions;
  std::vector<int> moving_ids;
  std::vector<Target> targets;
  int targets_index = 0;

  bool position_is_set = false;
  bool vel_and_acc_are_set = false;

  MotionList(Robot* robot, Clock* clock);

  void createMotion(const std::string& motion_name, int motion_num);
  void setTargetLegIK(int leg, float leg_rad_steering, float leg_height, float back_forth, int leg_mode);
  void setTargetLegFK(int leg, float leg_theta0, float leg_theta1, float leg_theta2);
  void setTargetHandFKRatio(int hand, float ratio_back_forth, float ratio_up_down, float ratio_end_effector);
  void setTargetVelocityAndAcceleration(int velocity, int acceleration);
  void appendTargetsAndIntervalTimeToMotion(int interval_time);
  void motionStart();
  void motionClear(const std::string& exceptional_motion);
  bool can_start(const std::string& motion_name);

private:
  Robot* robot_;
  Clock* clock_;
};

}  // namespace shin_spider
