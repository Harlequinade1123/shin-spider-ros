/***
 腕に関する動作まとめ
***/

//テスト
void testHandMotion(int hand)
{
  float ratio_back_forth0 = 0;
  float ratio_up_down0 = 0;
  float ratio_end_effector0 = 0;

  //速度，加速度
  int vel = 400;
  int acc = 400;

  //動作の名前
  String motion_name = "test_hand_motion" + str(hand);

  //同じ名前の動作が既に存在していればその動作の再生を行い（motionStart()），新しい動作の登録は行わない
  if (motion_list.can_start(motion_name))
  {
    return;
  }

  //以下動作登録

  //登録動作の数（motion_list.appendTargetsAndIntervalTimeToMotionの数に等しい）
  //1つの動作は複数の目標値かたまりから構成される
  motion_list.createMotion(motion_name, 3);

  //目標値設定（1つの目標値につき必ず速度，加速度設定を行う）
  //腕の各関節角度目標値の設定
  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0, ratio_up_down0, ratio_end_effector0);
  //目標値の速度，加速度の指定
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  //目標値の登録，実行時間（以下の関数例では，200[ms]以上経過すると次の目標値指令が送られる）
  motion_list.appendTargetsAndIntervalTimeToMotion(200);
  //motion_list.apppend...までがひとかたまり（1つの動作）

  //2つ目の動作
  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0+50, ratio_up_down0+50, ratio_end_effector0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  //3つ目の動作
  motion_list.setTargetHandFKRatio(hand, ratio_back_forth0, ratio_up_down0, ratio_end_effector0);
  motion_list.setTargetVelocityAndAcceleration(vel, acc);
  motion_list.appendTargetsAndIntervalTimeToMotion(200);

  //動作の再生
  motion_list.motionStart();
  //確認用print
  println("START: " + motion_name);
}

//手を振る
void wavingHandsMotion()
{
  int vel = 1000;
  int acc = 1000;

  String motion_name = "waving_hands_motion";

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

//薙ぎ払い
void sweepingMotion(float angle)
{
  int vel = 800;
  int acc = 600;

  String motion_name = "sweeping_moiton_" + str(angle);

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

//収納
void foldingMotion()
{
  int vel = 800;
  int acc = 600;

  String motion_name = "folding_motion";

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