class RobotHand
{
  // member variable
  int[] motor_IDs           = {0, 0, 0};
  float[] links             = {0, 0, 0};
  float[] xyz               = {0, 0, 0};
  float[] rad_thetas        = {0, 0, 0};
  float[] old_rad_thetas    = {0, 0, 0};
  float[] rad_thetas_min    = {-PI / 2, -PI / 2, -PI};
  float[] rad_thetas_max    = {PI / 2, PI / 2, PI};
  float coor_rad_theta; //RobotLeg座標とWorld座標の回転角
  
  boolean is_set_parameter  = false;

  /*** member function
  * コンストラクタ
  * RobotHand(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
  * 初期角度を指定して初期化
  * void initializeParameterFromThetas(float[] tmp_links, float[] tmp_thetas)
  * 引数の角度をメンバ変数に代入
  * void setThetas(float[] tmp_thetas)
  * void calcFK(float[] target_rad_thetas)
  * float[] calcAndGetThetasFromRatio(float ratio_back_forth, float ratio_up_down, float ratio_end_effector)
  * void display()
  ***/

  RobotHand(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
  {
    motor_IDs[0] = tmp_ID1;
    motor_IDs[1] = tmp_ID2;
    motor_IDs[2] = tmp_ID3;
    coor_rad_theta = tmp_coor_rad_theta;
  }

  void initializeParameterFromThetas(float[] tmp_links, float[] tmp_thetas)
  {
    is_set_parameter = true;
    for (int i = 0; i < 3; i++)
    {
      links[i] = tmp_links[i];
    }
    for (int i = 0; i < 3; i++)
    {
      rad_thetas[i] = tmp_thetas[i];
      old_rad_thetas[i] = tmp_thetas[i];
    }
    calcFK(tmp_thetas);
    
    for (int i = 0; i < 3; i++)
    {
      rad_thetas[i] = tmp_thetas[i];
      if (rad_thetas_max[i] < rad_thetas[i])
      {
        rad_thetas[i] = rad_thetas_max[i];
      }
      if (rad_thetas_min[i] > rad_thetas[i])
      {
        rad_thetas[i] = rad_thetas_min[i];
      }
      old_rad_thetas[i] = rad_thetas[i];
      calcFK(rad_thetas);
    }
  }

  void setThetas(float[] tmp_thetas)
  {
    for (int i = 0; i < 3; i++)
    {
      rad_thetas[i] = tmp_thetas[i];
      old_rad_thetas[i] = tmp_thetas[i];
    }
    
    for (int i = 0; i < 3; i++)
    {
      rad_thetas[i] = tmp_thetas[i];
      if (rad_thetas_max[i] < rad_thetas[i])
      {
        rad_thetas[i] = rad_thetas_max[i];
      }
      if (rad_thetas_min[i] > rad_thetas[i])
      {
        rad_thetas[i] = rad_thetas_min[i];
      }
      old_rad_thetas[i] = rad_thetas[i];
    }

    calcFK(rad_thetas);
  }

  void calcFK(float[] target_rad_thetas)
  {
    if (is_set_parameter == false)
    {
      println("ERROR: Parameter is not set");
      return;
    }
    if (target_rad_thetas.length < 3)
    {
      println("ERROR: float[] less");
      return;
    }
    float sinTh0 = sin(target_rad_thetas[0]);
    float sinTh1 = sin(target_rad_thetas[1]);
    float cosTh0 = cos(target_rad_thetas[0]);
    float cosTh1 = cos(target_rad_thetas[1]);

    //根元のリンクベクトルは含めない順運動学
    xyz[0] = cosTh0 * links[1] + cosTh0 * cosTh1 * links[2];
    xyz[1] = sinTh0 * links[1] + sinTh0 * cosTh1 * links[2];
    xyz[2] =                            - sinTh1 * links[2];
  }

  float[] calcAndGetThetasFromRatio(float ratio_back_forth, float ratio_up_down, float ratio_end_effector)
  {
    float[] ans_rad_thetas = {0, 0, 0};

    ans_rad_thetas[0] = PI * ratio_back_forth / 200;
    if (coor_rad_theta > 0)
    {
      ans_rad_thetas[0] *= -1;
    }

    ans_rad_thetas[1] = -PI * ratio_up_down / 200;

    ans_rad_thetas[2] = PI * ratio_end_effector / 100;
    if (coor_rad_theta > 0)
    {
      ans_rad_thetas[2] *= -1;
    }

    for (int i = 0; i < 0; i++)
    {
      if (rad_thetas_max[i] < ans_rad_thetas[i])
      {
        ans_rad_thetas[i] = rad_thetas_max[i];
      }
      if (rad_thetas_min[i] > ans_rad_thetas[i])
      {
        ans_rad_thetas[i] = rad_thetas_min[i];
      }
    }

    return ans_rad_thetas;
  }

  void display()
  {
    pushMatrix();

    rightHandedRotateZ(coor_rad_theta);

    translate(0, 0, links[0]);

    makeLinkX(links[0]);

    translate(xyz[1], -xyz[2], xyz[0]);
    noStroke();
    sphere(25);
    translate(-xyz[1], xyz[2], -xyz[0]);

    makeCylinder();

    rightHandedRotateZ(rad_thetas[0]);
    
    makeLinkX(links[1]);

    rightHandedRotateZ(-coor_rad_theta);
    textSize(12);
    fill(#FFFF00);
    text(str(motor_IDs[0]) + ", " + str(motor_IDs[1]) + ", " + str(motor_IDs[2]), 30, -60);
    fill(255);
    rightHandedRotateZ(coor_rad_theta);

    rightHandedRotateX(PI/ 2);
    makeCylinder();
    rightHandedRotateX(-PI/ 2);

    rightHandedRotateY(rad_thetas[1]);

    makeLinkX(links[2]);

    makeCylinder();

    rightHandedRotateZ(-rad_thetas[2]);

    translate(0, links[2] / 2, 0);
    stroke(100);
    rightHandedRotateZ(PI / 2);
    box(20, 20, links[2]);

    popMatrix();
  }
};
