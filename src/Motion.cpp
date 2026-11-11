#include "shin_spider/Motion.h"

namespace shin_spider
{
using namespace processing;

Motion::Motion(const std::string& tmp_motion_name, int target_num, Clock* clock) : clock_(clock)
{
  motion_name = tmp_motion_name;
  for (int i = 0; i < target_num; i++)
  {
    targets.emplace_back();
    time_thresholds.push_back(0);
  }
  motion_index_num = target_num;
  motion_index = target_num;
}

int Motion::millis() const { return clock_ ? clock_->millis() : 0; }

void Motion::appendTarget(int motion_order, const Target& target, int time_threshold)
{
  if (motion_order >= static_cast<int>(targets.size()))
  {
    println("ERROR: motion_order is too large");
  }
  else
  {
    targets[motion_order].push_back(target);
    time_thresholds[motion_order] = time_threshold;
    if (target.target_id == 0)
    {
      for (size_t i = 0; i < target.target_ids.size(); i++)
      {
        motion_ids_nums.at(target.target_ids[i])++;
      }
    }
    else
    {
      motion_ids_nums.at(target.target_id)++;
    }
  }
  setIds();
}

std::optional<std::string> Motion::getMotionCommand()
{
  if (command_is_sent)
  {
    return std::nullopt;
  }
  std::string ans_msg = "[m";
  if (targets.empty() || motion_index == motion_index_num)
  {
    return std::nullopt;
  }
  for (size_t i = 0; i < targets[motion_index].size(); i++)
  {
    ans_msg += targets[motion_index][i].getTargetCommand();
  }
  ans_msg += "]\n";
  command_is_sent = true;
  return ans_msg;
}

std::optional<std::string> Motion::getVelocityCommand() const
{
  std::string ans_msg = "[v";
  if (targets.empty() || motion_index == motion_index_num)
  {
    return std::nullopt;
  }
  for (size_t i = 0; i < targets[motion_index].size(); i++)
  {
    ans_msg += targets[motion_index][i].getTargetVelocityCommand();
  }
  ans_msg += "]\n";
  return ans_msg;
}

std::optional<std::string> Motion::getAccelerationCommand() const
{
  std::string ans_msg = "[k";
  if (targets.empty() || motion_index == motion_index_num)
  {
    return std::nullopt;
  }
  for (size_t i = 0; i < targets[motion_index].size(); i++)
  {
    ans_msg += targets[motion_index][i].getTargetAccelerationCommand();
  }
  ans_msg += "]\n";
  return ans_msg;
}

bool Motion::idExists(int id) const { return motion_ids_nums.at(id) > 0; }

bool Motion::idExists(const std::vector<int>& ids) const
{
  int ids_num = 0;
  for (size_t i = 0; i < ids.size(); i++)
  {
    ids_num += motion_ids_nums.at(ids[i]);
  }
  return ids_num > 0;
}

void Motion::setIds()
{
  std::vector<int> ans_motion_ids;
  for (size_t i = 0; i < motion_ids_nums.size(); i++)
  {
    if (motion_ids_nums[i] > 0)
    {
      ans_motion_ids.push_back(static_cast<int>(i));
    }
  }
  motion_ids = ans_motion_ids;
}

void Motion::resetMotionIndex() { motion_index = 0; }

void Motion::start()
{
  if (is_moving == false)
  {
    is_moving = true;
    command_is_sent = false;
    motion_index = 0;
    motion_time = millis();
  }
}

void Motion::next()
{
  if (motion_index < motion_index_num)
  {
    if (millis() - motion_time >= time_thresholds[motion_index])
    {
      motion_index++;
      motion_time = millis();
      command_is_sent = false;
    }
  }
  else if (command_is_sent && motion_index == motion_index_num - 1)
  {
    is_moving = false;
    motion_index++;
  }
  else if (motion_index == motion_index_num)
  {
    is_moving = false;
  }
}

}  // namespace shin_spider
