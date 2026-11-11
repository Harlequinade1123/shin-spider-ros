  // ======== golden dump driver (hand-written, appended inside class Sketch) ========
  java.io.PrintWriter w;
  int case_count = 0;

  void resetWorld()
  {
    leg_links   = new float[] {35, 20, 30, 70, 70};
    leg_thetas  = new float[] {PI/4, 0, 0};
    hand_links  = new float[] {35, 70, 70};
    hand_thetas = new float[] {0, 0, 0};
    fake_millis = 0;
    robot       = new Robot(body, leg_num, hand_num);
    motion_list = new MotionList();
  }

  String nn(String s) { return (s == null) ? "NULL" : s.replaceAll("\\n$", ""); }

  String fa(float[] a)
  {
    String s = "";
    for (int i = 0; i < a.length; i++) s += (i > 0 ? "," : "") + a[i];
    return s;
  }
  String ia(int[] a)
  {
    String s = "";
    for (int i = 0; i < a.length; i++) s += (i > 0 ? "," : "") + a[i];
    return s;
  }

  void dumpRobot()
  {
    w.println("ROBOT");
    for (int i = 0; i < robot.robot_legs.length; i++)
    {
      RobotLeg l = robot.robot_legs[i];
      w.println("LEG " + i + " motor_IDs=" + ia(l.motor_IDs) + " rad_thetas=" + fa(l.rad_thetas)
                + " coor_rad_theta=" + l.coor_rad_theta + " origin_steering=" + l.origin_steering
                + " origin_xyz=" + fa(l.origin_xyz));
    }
    for (int i = 0; i < robot.robot_hands.length; i++)
    {
      RobotHand h = robot.robot_hands[i];
      w.println("HAND " + i + " motor_IDs=" + ia(h.motor_IDs) + " rad_thetas=" + fa(h.rad_thetas)
                + " coor_rad_theta=" + h.coor_rad_theta);
    }
    w.println("ENDROBOT");
  }

  void runCase(String name, Runnable r)
  {
    resetWorld();
    r.run();
    dumpMotions(name);
  }

  void dumpMotions(String name)
  {
    case_count++;
    w.println("CASE " + name);
    for (int m = 0; m < motion_list.motions.length; m++)
    {
      Motion mo = motion_list.motions[m];
      w.println("MOTION " + mo.motion_name);
      for (int idx = 0; idx < mo.motion_index_num; idx++)
      {
        mo.motion_index = idx;
        mo.command_is_sent = false;
        w.println("STEP " + idx + " " + mo.time_thresholds[idx]);
        w.println(nn(mo.getVelocityCommand()));
        w.println(nn(mo.getAccelerationCommand()));
        mo.command_is_sent = false;
        w.println(nn(mo.getMotionCommand()));
      }
    }
  }

  public static void main(String[] args) throws Exception
  {
    new Sketch().go(args[0]);
  }

  void go(String path) throws Exception
  {
    w = new java.io.PrintWriter(new java.io.OutputStreamWriter(new java.io.FileOutputStream(path), "UTF-8"));
    resetWorld();
    dumpRobot();

    final float[] steers = {0, PI / 2, PI, -PI / 2};

    for (final int leg : new int[] {0, 1}) // leg+2 is used, so 2,3 index out of range in the original
      runCase("testLegMotion(leg=" + leg + ")", () -> testLegMotion(leg));

    for (final float s : steers)
      for (int mode = 0; mode <= 5; mode++)
      {
        final int md = mode;
        runCase("walkingMotion(steering=" + s + ",mode=" + md + ")", () -> walkingMotion(s, md));
      }
    for (final float s : steers)
      for (int mode = 0; mode <= 1; mode++)
      {
        final int md = mode;
        runCase("sideWalkingMotion(steering=" + s + ",mode=" + md + ")", () -> sideWalkingMotion(s, md));
      }
    for (final float s : steers)
      for (int rl = 0; rl <= 2; rl++)
      {
        final int r = rl;
        runCase("jumpingMotion(steering=" + s + ",right_left=" + r + ")", () -> jumpingMotion(s, r));
      }
    for (final float s : steers)
      for (int rl = 0; rl <= 2; rl++)
      {
        final int r = rl;
        runCase("backJumpingMotion(steering=" + s + ",right_left=" + r + ")", () -> backJumpingMotion(s, r));
      }
    for (final float s : steers)
      for (final float d : new float[] {PI / 4, -PI / 4, PI / 2})
        runCase("rotationalMotion(steering=" + s + ",delta_rotate=" + d + ")", () -> rotationalMotion(s, d));
    for (final float s : steers)
      for (final float p : new float[] {0, PI / 2, -PI / 2})
        runCase("rotationalMotion90(steering=" + s + ",pred_steering=" + p + ")", () -> rotationalMotion90(s, p));
    for (final float s : steers)
      runCase("spreadMotion(steering=" + s + ")", () -> spreadMotion(s));
    for (final float s : steers)
      runCase("shrinkMotion(steering=" + s + ")", () -> shrinkMotion(s));
    for (final float s : steers)
      runCase("resetMotion(steering=" + s + ")", () -> resetMotion(s));

    for (final int h : new int[] {0, 1})
      runCase("testHandMotion(hand=" + h + ")", () -> testHandMotion(h));
    runCase("wavingHandsMotion()", () -> wavingHandsMotion());
    for (final float a : new float[] {0, 30, 60, 90, -45})
      runCase("sweepingMotion(angle=" + a + ")", () -> sweepingMotion(a));
    runCase("foldingMotion()", () -> foldingMotion());

    // sequence cases: several generators on one MotionList (conflict / re-start / motionClear paths)
    runCase("seq[walkingMotion(0.0,1);walkingMotion(0.0,1)]", () -> {
      walkingMotion(0, 1); walkingMotion(0, 1);
    });
    runCase("seq[walkingMotion(0.0,0);sideWalkingMotion(0.0,0);wavingHandsMotion()]", () -> {
      walkingMotion(0, 0); sideWalkingMotion(0, 0); wavingHandsMotion();
    });
    runCase("seq[walkingMotion(0.0,1);resetMotion(0.0)]", () -> {
      walkingMotion(0, 1); resetMotion(0);
    });
    runCase("seq[walkingMotion(0.0,1);walkingMotion(1.5707964,1);resetMotion(0.0)]", () -> {
      walkingMotion(0, 1); walkingMotion(PI / 2, 1); resetMotion(0);
    });
    runCase("seq[resetMotion(0.0);walkingMotion(0.0,1);resetMotion(0.0)]", () -> {
      resetMotion(0); walkingMotion(0, 1); resetMotion(0);
    });

    w.close();
  }
