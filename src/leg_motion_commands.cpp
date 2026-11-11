// Port of leg_motion_commands.pde (literal transcription; float semantics kept).
#include "shin_spider/SpiderController.h"

namespace shin_spider
{
using namespace processing;

// test
void SpiderController::testLegMotion(int leg)
{
  float steering0 = 0;
  float height0 = 20;
  float forth0 = 0;

  int vel = 600;
  int acc = 600;

  int elbow_up = 1;
  int elbow_down = 0;

  std::string motion_name = "test_leg_motion" + str(leg);

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 5);

  motion_list.setTargetLegIK(leg, steering0, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(leg + 2, steering0, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(leg, steering0, height0 + 30, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(leg + 2, steering0, height0 + 30, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(70);

  motion_list.setTargetLegIK(leg, steering0, height0 + 30, forth0 + 30, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(leg + 2, steering0, height0 + 30, forth0 + 30, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(70);

  motion_list.setTargetLegIK(leg, steering0, height0, forth0 + 30, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(leg + 2, steering0, height0, forth0 + 30, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(70);

  motion_list.setTargetLegIK(leg, steering0, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(leg + 2, steering0, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// walking
void SpiderController::walkingMotion(float steering, int mode)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 30;

  int vel = 2800;
  int acc = 2800;

  int elbow_up = 1;
  int elbow_down = 0;

  float delta_height = 40;
  float delta_forth = 80;

  if (mode >= 3)
  {
    delta_forth *= -1;
  }

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name = "walking_motion_" + str(mode) + "_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  if (mode == 0 || mode == 3)
  {
    motion_list.createMotion(motion_name, 8);
  }
  else
  {
    motion_list.createMotion(motion_name, 4);
  }

  if (mode == 0 || mode == 1 || mode == 3 || mode == 4)
  {
    motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, -forth0 + delta_forth, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, 200);
    motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, 200);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);
  }

  if (mode == 0 || mode == 2 || mode == 3 || mode == 5)
  {
    motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, -forth0 + delta_forth, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, 200);
    motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, 200);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);

    motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0, elbow_up);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, -forth0, elbow_down);
    motion_list.setTargetVelocityAndAcceleration(vel, acc);
    motion_list.appendTargetsAndIntervalTimeToMotion(100);
  }

  motion_list.motionStart();
  println("START: " + motion_name);
}

// side walking
void SpiderController::sideWalkingMotion(float steering, int mode)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 30;

  int vel = 2800;
  int acc = 2800;

  int elbow_up = 1;
  int elbow_down = 0;

  float delta_height = 40;
  float delta_forth1 = 80;
  float delta_forth2 = 80;

  if (mode == 0)
  {
    delta_forth1 *= -1;
  }
  else
  {
    delta_forth2 *= -1;
  }

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name = "side_walking_motion_" + str(mode) + "_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 8);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 + delta_forth1, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, -forth0 + delta_forth1, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, 200);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, 200);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 + delta_forth2, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, -forth0 + delta_forth2, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, 200);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, 200);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, -forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// jumping (direction adjustable)
void SpiderController::jumpingMotion(float steering, int right_left)
{
  float steering0 = 0;
  float height0_1 = 40;
  float height0_2 = 40;
  float forth0 = 0;

  int delta_vel_r = 0;
  int delta_vel_l = 0;
  int delta_acc_r = 0;
  int delta_acc_l = 0;

  int vel = 2000;
  int acc = 2000;

  if (right_left == 2)
  {
    delta_vel_r = -vel + 50;
    delta_acc_r = -acc + 50;
  }
  else if (right_left == 1)
  {
    delta_vel_l = -vel + 50;
    delta_acc_l = -acc + 50;
  }

  int elbow_up = 1;
  int elbow_down = 0;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
    height0_1 = 35;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
    height0_2 = 35;
  }

  std::string motion_name = "jumping_motion_" + str(right_left) + "_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  // retract back legs
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0_1 + 20, forth0 + 30, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0_2 + 20, forth0 + 30, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  // extend front legs
  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0_1 + 50, forth0 + 50, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel + 500, acc + 500);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0_2 + 50, forth0 + 50, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel + 500, acc + 500);
  motion_list.appendTargetsAndIntervalTimeToMotion(50);

  // extend back legs
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0_1 - 10, forth0 - 20, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel + delta_vel_r, acc + delta_acc_r);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0_2 - 10, forth0 - 20, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel + delta_vel_l, acc + delta_acc_l);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  // initial posture
  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0_1, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0_2, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0_1, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0_2, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// back jumping (direction adjustable)
void SpiderController::backJumpingMotion(float steering, int right_left)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int delta_vel_r = 0;
  int delta_vel_l = 0;
  int delta_acc_r = 0;
  int delta_acc_l = 0;

  if (right_left == 1)
  {
    delta_vel_r = -1900;
    delta_acc_r = -1900;
  }
  else if (right_left == 2)
  {
    delta_vel_l = -1900;
    delta_acc_l = -1900;
  }

  int vel = 2000;
  int acc = 2000;

  int elbow_up = 1;
  int elbow_down = 0;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name = "back_jumping_motion_" + str(right_left) + "_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  // retract front legs
  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0 + 20, forth0 - 30, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel - 200, acc - 500);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0 + 20, forth0 - 30, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel - 200, acc - 500);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  // extend back legs
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0 + 50, forth0 - 50, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel + 500, acc + 500);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0 + 50, forth0 - 50, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel + 500, acc + 500);
  motion_list.appendTargetsAndIntervalTimeToMotion(50);

  // extend front legs
  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0 - 10, forth0 + 20, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel + delta_vel_r, acc + delta_acc_r);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0 - 10, forth0 + 20, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel + delta_vel_l, acc + delta_acc_l);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  // initial posture
  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// rotate legs
void SpiderController::rotationalMotion(float steering, float delta_rotate)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int vel = 800;
  int acc = 800;

  int elbow_up = 1;
  int elbow_down = 0;

  float delta_height = 30;
  float delta_forth = 50;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name =
      "rotational_motion_" + str(toInt(degrees(steering))) + "_" + str(toInt(degrees(delta_rotate)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 5);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4 + delta_rotate, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4 + delta_rotate, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4 + delta_rotate, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4 + delta_rotate, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4 + delta_rotate, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4 + delta_rotate, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4 + delta_rotate, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4 + delta_rotate, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(80);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// 90 deg rotation (prone to falling)
void SpiderController::rotationalMotion90(float steering, float pred_steering)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int vel = 500;
  int acc = 500;

  int elbow_up = 1;
  int elbow_down = 0;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name =
      "rotational+_" + str(toInt(degrees(steering))) + "_" + str(toInt(degrees(pred_steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  // (a commented-out IK block in the original is intentionally not ported)

  motion_list.setTargetLegFK(front_legs[0], steering0 + pred_steering + robot.robot_legs[front_legs[0]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(front_legs[1], steering0 + pred_steering + robot.robot_legs[front_legs[1]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(back_legs[0], steering0 + pred_steering + robot.robot_legs[back_legs[0]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(back_legs[1], steering0 + pred_steering + robot.robot_legs[back_legs[1]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(400);

  motion_list.setTargetLegFK(front_legs[0], steering0 + steering + robot.robot_legs[front_legs[0]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(front_legs[1], steering0 + steering + robot.robot_legs[front_legs[1]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(back_legs[0], steering0 + steering + robot.robot_legs[back_legs[0]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegFK(back_legs[1], steering0 + steering + robot.robot_legs[back_legs[1]].origin_steering, 0, 0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0 - 35, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0 - 35, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0 - 35, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0 - 35, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(300);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(300);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// spread legs
void SpiderController::spreadMotion(float steering)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int vel = 800;
  int acc = 800;

  int elbow_up = 1;
  int elbow_down = 0;

  float delta_height = 30;
  float delta_forth = 50;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name = "spread_motion_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// close legs
void SpiderController::shrinkMotion(float steering)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int vel = 800;
  int acc = 800;

  int elbow_up = 1;
  int elbow_down = 0;

  float delta_height = 30;
  float delta_forth = 50;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
  }

  std::string motion_name = "shrink_motion_" + str(toInt(degrees(steering)));

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 4);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering + PI / 4, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 + delta_forth, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering - PI / 4, height0 + delta_height, forth0 - delta_forth, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(130);

  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(50);

  motion_list.motionStart();
  println("START: " + motion_name);
}

// reset
void SpiderController::resetMotion(float steering)
{
  float steering0 = 0;
  float height0 = 40;
  float forth0 = 0;

  int vel = 800;
  int acc = 800;

  int elbow_up = 1;
  int elbow_down = 0;

  int front_legs[2] = {0, 1};
  int back_legs[2] = {2, 3};

  std::string motion_name = "reset_motion";

  if (steering > PI / 3)
  {
    front_legs[0] = 3;
    front_legs[1] = 0;
    back_legs[0] = 1;
    back_legs[1] = 2;
    motion_name = "reset_motion_";
  }
  else if (steering < -PI / 3)
  {
    front_legs[0] = 1;
    front_legs[1] = 2;
    back_legs[0] = 3;
    back_legs[1] = 0;
    motion_name = "reset_motion__";
  }
  motion_list.motionClear(motion_name);

  if (motion_list.can_start(motion_name))
  {
    return;
  }

  motion_list.createMotion(motion_name, 2);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0 + 20, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0 + 20, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0 + 20, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0 + 20, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.setTargetLegIK(front_legs[0], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[0], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(front_legs[1], steering0 + steering, height0, forth0, elbow_up);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.setTargetLegIK(back_legs[1], steering0 + steering, height0, forth0, elbow_down);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(100);

  motion_list.motionStart();
  println("START: " + motion_name);
}

}  // namespace shin_spider
