// Port of Motion.pde: time-stepped sequence of target groups.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "shin_spider/Target.h"
#include "shin_spider/processing.h"

namespace shin_spider
{

class Motion
{
public:
  std::string motion_name = "";
  std::vector<std::vector<Target>> targets;
  std::vector<int> time_thresholds;
  int motion_index_num = 0;
  int motion_index = 0;
  int motion_time = 0;
  std::array<int, 19> motion_ids_nums{{-1, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0}};
  std::vector<int> motion_ids;
  bool is_moving = false;
  bool command_is_sent = false;

  // clock replaces Processing millis(); nullptr -> millis() == 0
  Motion(const std::string& tmp_motion_name, int target_num, Clock* clock = nullptr);

  void appendTarget(int motion_order, const Target& target, int time_threshold);

  // Processing returns null -> std::nullopt. Strings include the trailing "\n".
  std::optional<std::string> getMotionCommand();
  std::optional<std::string> getVelocityCommand() const;
  std::optional<std::string> getAccelerationCommand() const;

  bool idExists(int id) const;
  bool idExists(const std::vector<int>& ids) const;
  void setIds();
  void resetMotionIndex();
  void start();
  void next();

  void setClock(Clock* clock) { clock_ = clock; }

private:
  int millis() const;
  Clock* clock_ = nullptr;
};

}  // namespace shin_spider
