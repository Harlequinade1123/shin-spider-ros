#include "shin_spider/Target.h"

#include "shin_spider/processing.h"

namespace shin_spider
{
using namespace processing;

Target::Target(const std::array<int, 3>& tmp_target_ids)
{
  target_ids[0] = tmp_target_ids[0];
  target_ids[1] = tmp_target_ids[1];
  target_ids[2] = tmp_target_ids[2];
  is_multidata = true;
}

Target::Target(int tmp_target_id1, int tmp_target_id2, int tmp_target_id3)
{
  target_ids[0] = tmp_target_id1;
  target_ids[1] = tmp_target_id2;
  target_ids[2] = tmp_target_id3;
  is_multidata = true;
}

Target::Target(int tmp_target_id)
{
  target_id = tmp_target_id;
  is_multidata = false;
}

void Target::setTarget(const std::array<float, 3>& tmp_target_thetas)
{
  if (is_multidata)
  {
    for (size_t i = 0; i < target_ids.size(); i++)
    {
      target_values[i] = toInt(map(tmp_target_thetas[i], -PI, PI, 0, 4095));
      target_thetas[i] = tmp_target_thetas[i];
    }
    target_value_is_set = true;
  }
  else
  {
    println("ERROR: is not multi");
  }
}

void Target::setTarget(float target_theta1, float target_theta2, float target_theta3)
{
  if (is_multidata)
  {
    target_values[0] = toInt(map(target_theta1, -PI, PI, 0, 4095));
    target_values[1] = toInt(map(target_theta2, -PI, PI, 0, 4095));
    target_values[2] = toInt(map(target_theta3, -PI, PI, 0, 4095));
    target_thetas[0] = target_theta1;
    target_thetas[1] = target_theta2;
    target_thetas[2] = target_theta3;
    target_value_is_set = true;
  }
  else
  {
    println("ERROR: is not multi");
  }
}

void Target::setTarget(float tmp_target_theta)
{
  if (is_multidata == false)
  {
    if (target_id > 0 && target_value >= 0 && target_value <= 4095)
    {
      target_value = toInt(map(tmp_target_theta, -PI, PI, 0, 4095));
      target_theta = tmp_target_theta;
    }
    target_value_is_set = true;
  }
  else
  {
    println("ERROR: is multi");
  }
}

void Target::setTargetVelocityAndAcceleration(int tmp_target_velocity, int tmp_target_acceleration)
{
  if (0 <= tmp_target_velocity && tmp_target_velocity <= 32767)
  {
    target_velocity = tmp_target_velocity;
  }
  else
  {
    println("ERROR: velocity is inappropriate");
  }

  if (0 <= tmp_target_acceleration && tmp_target_acceleration <= 32767)
  {
    target_acceleration = tmp_target_acceleration;
  }
  else
  {
    println("ERROR: acceleration is inappropriate");
  }
}

std::string Target::getTargetCommand() const
{
  std::string ans_msg = "";
  if (target_value_is_set == false)
  {
    println("ERROR: value is not set");
  }
  if (is_multidata)
  {
    for (size_t i = 0; i < target_ids.size(); i++)
    {
      if (target_ids[i] > 0 && target_values[i] >= 0 && target_values[i] <= 4095)
      {
        ans_msg += "," + str(target_ids[i]) + "," + str(target_values[i]);
      }
    }
    return ans_msg;
  }
  if (target_id > 0 && target_value >= 0 && target_value <= 4095)
  {
    ans_msg += "," + str(target_id) + "," + str(target_value);
  }
  return ans_msg;
}

std::string Target::getTargetVelocityCommand() const
{
  std::string ans_msg = "";
  if (target_value_is_set == false)
  {
    println("ERROR: value is not set");
  }
  if (is_multidata)
  {
    for (size_t i = 0; i < target_ids.size(); i++)
    {
      if (target_ids[i] > 0)
      {
        ans_msg += "," + str(target_ids[i]) + "," + str(target_velocity);
      }
    }
    return ans_msg;
  }
  if (target_id > 0)
  {
    ans_msg += "," + str(target_id) + "," + str(target_velocity);
  }
  return ans_msg;
}

std::string Target::getTargetAccelerationCommand() const
{
  std::string ans_msg = "";
  if (target_value_is_set == false)
  {
    println("ERROR: value is not set");
  }
  if (is_multidata)
  {
    for (size_t i = 0; i < target_ids.size(); i++)
    {
      if (target_ids[i] > 0)
      {
        ans_msg += "," + str(target_ids[i]) + "," + str(target_acceleration);
      }
    }
    return ans_msg;
  }
  if (target_id > 0)
  {
    ans_msg += "," + str(target_id) + "," + str(target_acceleration);
  }
  return ans_msg;
}

}  // namespace shin_spider
