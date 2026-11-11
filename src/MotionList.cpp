#include "shin_spider/MotionList.h"

namespace shin_spider
{
using namespace processing;

MotionList::MotionList(Robot* robot, Clock* clock) : robot_(robot), clock_(clock) {}

void MotionList::createMotion(const std::string& motion_name, int motion_num)
{
  motions.emplace_back(motion_name, motion_num, clock_);
  targets_index = 0;
}

void MotionList::setTargetLegIK(int leg, float leg_rad_steering, float leg_height, float back_forth, int leg_mode)
{
  if (position_is_set ^ vel_and_acc_are_set)
  {
    println("ERROR: cannot create new target");
    return;
  }
  targets.emplace_back(robot_->robot_legs.at(leg).motor_IDs);
  targets.back().setTarget(robot_->robot_legs.at(leg).calcAndGetIK2DOF(leg_rad_steering, leg_height, back_forth, leg_mode));
  position_is_set = true;
  vel_and_acc_are_set = false;
}

void MotionList::setTargetLegFK(int leg, float leg_theta0, float leg_theta1, float leg_theta2)
{
  if (position_is_set ^ vel_and_acc_are_set)
  {
    println("ERROR: cannot create new target");
    return;
  }
  targets.emplace_back(robot_->robot_legs.at(leg).motor_IDs);
  targets.back().setTarget(leg_theta0, leg_theta1, leg_theta2);
  position_is_set = true;
  vel_and_acc_are_set = false;
}

void MotionList::setTargetHandFKRatio(int hand, float ratio_back_forth, float ratio_up_down, float ratio_end_effector)
{
  if (position_is_set ^ vel_and_acc_are_set)
  {
    println("ERROR: cannot create new target");
    return;
  }
  targets.emplace_back(robot_->robot_hands.at(hand).motor_IDs);
  targets.back().setTarget(
      robot_->robot_hands.at(hand).calcAndGetThetasFromRatio(ratio_back_forth, ratio_up_down, ratio_end_effector));
  position_is_set = true;
  vel_and_acc_are_set = false;
}

void MotionList::setTargetVelocityAndAcceleration(int velocity, int acceleration)
{
  if (targets.empty())
  {
    // Processing would throw ArrayIndexOutOfBoundsException here.
    println("ERROR: no target to set velocity/acceleration");
    return;
  }
  targets.back().setTargetVelocityAndAcceleration(velocity, acceleration);
  vel_and_acc_are_set = true;
}

void MotionList::appendTargetsAndIntervalTimeToMotion(int interval_time)
{
  if (position_is_set == false)
  {
    println("Position is not set");
    return;
  }

  if (vel_and_acc_are_set == false)
  {
    println("Velocity ans Acceleration are not set");
    return;
  }

  for (size_t i = 0; i < targets.size(); i++)
  {
    motions.back().appendTarget(targets_index, targets[i], interval_time);
  }
  targets.clear();
  targets_index++;
  position_is_set = false;
  vel_and_acc_are_set = false;
}

void MotionList::motionStart() { motions.back().start(); }

void MotionList::motionClear(const std::string& exceptional_motion)
{
  // Processing compares with == (reference equality); all callers pass
  // interned literals, so value equality is equivalent.
  size_t motions_length = motions.size();
  for (size_t i = 0; i < motions_length; i++)
  {
    if (motions[i].motion_name == exceptional_motion && i != 0)
    {
      motions[0] = motions[i];
      break;
    }
  }
  for (size_t i = 0; i + 1 < motions_length; i++)
  {
    motions.pop_back();  // shorten()
  }
}

bool MotionList::can_start(const std::string& motion_name)
{
  std::vector<int> moving_motion_ids;
  for (size_t i = 0; i < motions.size(); i++)
  {
    if (motions[i].is_moving)
    {
      for (size_t j = 0; j < motions[i].motion_ids.size(); j++)
      {
        moving_motion_ids.push_back(motions[i].motion_ids[j]);
      }
    }
  }
  for (size_t i = 0; i < motions.size(); i++)
  {
    if (motions[i].motion_name == motion_name)
    {
      if (motions[i].is_moving == false && motions[i].idExists(moving_motion_ids) == false)
      {
        motions[i].start();
        println("START: " + motion_name);
        return true;
      }
      return true;
    }
  }
  return false;
}

}  // namespace shin_spider
