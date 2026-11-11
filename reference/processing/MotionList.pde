class MotionList
{
  // member variable
  Motion motions[] = {};
  int[] moving_ids = {};
  Target[] targets = {};
  int targets_index = 0;

  boolean position_is_set = false;
  boolean vel_and_acc_are_set = false;

  /*** member function
  * コンストラクタ
  * MotionList()
  * 脚の目標値を設定
  * void setTargetLegIK(int leg, float leg_rad_steering, float leg_height, float back_forth, int leg_mode)
  * 手の目標値を設定
  * void setTargetHandFKRatio(int hand, float ratio_back_forth, float ratio_up_down, float ratio_end_effector)
  * 速度・加速度を設定
  * void setTargetVelocityAndAcceleration(int velocity, int acceleration)
  * 登録した目標値を追加し，時間を設定
  * void appendTargetsAndIntervalTimeToMotion(int interval_time)
  * 動作開始
  * void motionStart()
  * 引数以外の動作を削除
  * void motionClear(String exceptional_motion)
  * 動作開始可能かどうか（可能な場合はmotionStart()を実行）
  * boolean can_start(String motion_name)
  ***/

  MotionList() {}

  void createMotion(String motion_name, int motion_num)
  {
    motions = (Motion[])append(motions, new Motion(motion_name, motion_num));
    targets_index = 0;
  }

  void setTargetLegIK(int leg, float leg_rad_steering, float leg_height, float back_forth, int leg_mode)
  {
    if (position_is_set ^ vel_and_acc_are_set)
    {
      println("ERROR: cannot create new target");
      return;
    }
    targets = (Target[])append(targets, new Target(robot.robot_legs[leg].motor_IDs));
    targets[targets.length - 1].setTarget(robot.robot_legs[leg].calcAndGetIK2DOF(leg_rad_steering, leg_height, back_forth, leg_mode));
    position_is_set = true;
    vel_and_acc_are_set = false;
  }

  void setTargetLegFK(int leg, float leg_theta0, float leg_theta1, float leg_theta2)
  {
    if (position_is_set ^ vel_and_acc_are_set)
    {
      println("ERROR: cannot create new target");
      return;
    }
    targets = (Target[])append(targets, new Target(robot.robot_legs[leg].motor_IDs));
    targets[targets.length - 1].setTarget(leg_theta0, leg_theta1, leg_theta2);
    position_is_set = true;
    vel_and_acc_are_set = false;
  }

  void setTargetHandFKRatio(int hand, float ratio_back_forth, float ratio_up_down, float ratio_end_effector)
  {
    if (position_is_set ^ vel_and_acc_are_set)
    {
      println("ERROR: cannot create new target");
      return;
    }
    targets = (Target[])append(targets, new Target(robot.robot_hands[hand].motor_IDs));
    targets[targets.length - 1].setTarget(robot.robot_hands[hand].calcAndGetThetasFromRatio(ratio_back_forth, ratio_up_down, ratio_end_effector));
    position_is_set = true;
    vel_and_acc_are_set = false;
  }

  void setTargetVelocityAndAcceleration(int velocity, int acceleration)
  {
    targets[targets.length - 1].setTargetVelocityAndAcceleration(velocity, acceleration);
    vel_and_acc_are_set = true;
  }

  void appendTargetsAndIntervalTimeToMotion(int interval_time)
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
    
    for (int i = 0; i < targets.length; i++)
    {
      motions[motions.length - 1].appendTarget(targets_index, targets[i], interval_time);
    }
    targets = new Target[0];
    targets_index++;
    position_is_set = false;
    vel_and_acc_are_set = false;
  }

  void motionStart()
  {
    motions[motions.length - 1].start();
  }

  void motionClear(String exceptional_motion)
  {
    int motions_length = motions.length;
    for (int i = 0; i < motions_length; i++)
    {
      if (motions[i].motion_name == exceptional_motion && i != 0)
      {
        motions[0] = motions[i];
        break;
      }
    }
    for (int i = 0; i < motions_length - 1; i++)
    {
      motions = (Motion[])shorten(motions);
    }
  }


  boolean can_start(String motion_name)
  {
    int[] moving_motion_ids = {};
    for (int i = 0; i < motions.length; i++)
    {
      if (motions[i].is_moving)
      {
        for (int j = 0; j < motions[i].motion_ids.length; j++)
        {
          moving_motion_ids = (int[])append(moving_motion_ids, motions[i].motion_ids[j]);
        }
      }
    }
    for (int i = 0; i < motions.length; i++)
    {
      if (motions[i].motion_name.equals(motion_name))
      {
        if (motions[i].is_moving == false && motions[i].idExists(moving_motion_ids) == false)
        {
          motions[i].start();
          println("START: " + motion_name);
          return true;
        }
        else
        {
          return true;
        }
      }
    }
    return false;
  }
};
