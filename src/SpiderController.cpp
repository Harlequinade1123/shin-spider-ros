#include "shin_spider/SpiderController.h"

namespace shin_spider
{
using namespace processing;

SpiderController::SpiderController(Clock* clock, CommandSink* sink)
  : robot(sketch::body, sketch::leg_num, sketch::hand_num), motion_list(&robot, clock), clock_(clock), sink_(sink)
{
}

void SpiderController::write(const std::string& data)
{
  if (sink_)
  {
    sink_->write(data);
  }
}

void SpiderController::delay(int ms)
{
  if (sink_)
  {
    sink_->delay(ms);
  }
}

void SpiderController::frame()
{
  readGamePadCommand();
  updateMotions();
}

void SpiderController::updateMotions()
{
  for (size_t i = 0; i < motion_list.motions.size(); i++)
  {
    Motion& motion = motion_list.motions[i];
    if (motion.is_moving == false)
    {
      continue;
    }

    motion.next();
    // command generation for OpenCM
    std::optional<std::string> msg_pos = motion.getMotionCommand();
    std::optional<std::string> msg_vel = motion.getVelocityCommand();
    std::optional<std::string> msg_acc = motion.getAccelerationCommand();
    if (msg_pos)
    {
      // msg_vel / msg_acc are non-null whenever msg_pos is non-null
      write(*msg_vel);
      delay(2);
      write(*msg_acc);
      delay(2);
      write(*msg_pos);
      delay(2);

      // robot model update
      const std::vector<Target>& step_targets = motion.targets[motion.motion_index];
      for (size_t j = 0; j < step_targets.size(); j++)
      {
        if (step_targets[j].is_multidata)
        {
          std::array<float, 3> motion_target_theta = step_targets[j].target_thetas;
          int target_leg_num = robot.findLegNumFromIds(step_targets[j].target_ids);
          int target_hand_num = robot.findHandNumFromIds(step_targets[j].target_ids);
          if (target_leg_num >= 0)
          {
            robot.robot_legs[target_leg_num].setThetas(motion_target_theta);
          }
          else if (target_hand_num >= 0)
          {
            robot.robot_hands[target_hand_num].setThetas(motion_target_theta);
          }
        }
      }
    }
  }
}

}  // namespace shin_spider
