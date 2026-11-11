// Port of Target.pde: per-motor target (single id or 3-id group) and its
// OpenCM command fragments.
#pragma once

#include <array>
#include <string>

namespace shin_spider
{

class Target
{
public:
  std::array<int, 3> target_ids{{0, 0, 0}};
  std::array<int, 3> target_values{{2048, 2048, 2048}};  // values actually sent
  std::array<float, 3> target_thetas{{0, 0, 0}};
  int target_id = 0;
  int target_value = 2048;
  float target_theta = 0;
  int target_velocity = 0;
  int target_acceleration = 0;
  bool is_multidata = false;
  bool target_value_is_set = false;

  explicit Target(const std::array<int, 3>& tmp_target_ids);
  Target(int tmp_target_id1, int tmp_target_id2, int tmp_target_id3);
  explicit Target(int tmp_target_id);

  void setTarget(const std::array<float, 3>& tmp_target_thetas);
  void setTarget(float target_theta1, float target_theta2, float target_theta3);
  void setTarget(float tmp_target_theta);

  void setTargetVelocityAndAcceleration(int tmp_target_velocity, int tmp_target_acceleration);

  std::string getTargetCommand() const;
  std::string getTargetVelocityCommand() const;
  std::string getTargetAccelerationCommand() const;
};

}  // namespace shin_spider
