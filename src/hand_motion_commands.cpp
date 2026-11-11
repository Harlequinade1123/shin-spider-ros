// Port of hand_motion_commands.pde (literal transcription).
#include "shin_spider/SpiderController.h"

namespace shin_spider
{
using namespace processing;

// test
void SpiderController::testHandMotion(int hand)
{
  float ratio_back_forth0 = 0;
  float ratio_up_down0 = 0;
  float ratio_end_effector0 = 0;

  int vel = 400;
  int acc = 400;

  std::string motion_name = "test_hand_motion" + str(hand);

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 3);

  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0, ratio_up_down0, ratio_end_effector0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0 + 50, ratio_up_down0 + 50, ratio_end_effector0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0, ratio_up_down0, ratio_end_effector0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// waving hands
void SpiderController::wavingHandsMotion()
{
  int vel = 1000;
  int acc = 1000;

  std::string motion_name = "waving_hands_motion";

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 2);

  motion_list.setTargetHandFKRatio(0, 0, 70, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, 0, 70, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(30);

  motion_list.setTargetHandFKRatio(0, 0, 90, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, 0, 90, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(30);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// sweeping
void SpiderController::sweepingMotion(float angle)
{
  int vel = 800;
  int acc = 600;

  // NOTE: "moiton" typo kept (motion name must match the original)
  std::string motion_name = "sweeping_moiton_" + str(angle);

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  motion_list.setTargetHandFKRatio(0, 0, angle, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, 0, angle, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.setTargetHandFKRatio(0, 90, angle, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, -90, angle, 95);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.setTargetHandFKRatio(0, -90, angle, 95);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, 90, angle, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(500);

  motion_list.setTargetHandFKRatio(0, 0, angle, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, 0, angle, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// folding
void SpiderController::foldingMotion()
{
  int vel = 800;
  int acc = 600;

  std::string motion_name = "folding_motion";

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 1);

  motion_list.setTargetHandFKRatio(0, -90, 90, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetHandFKRatio(1, -90, 90, 50);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

}  // namespace shin_spider
