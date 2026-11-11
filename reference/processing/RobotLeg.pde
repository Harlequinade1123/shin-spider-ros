class RobotLeg
{
  // member variable
  int[] motor_IDs           = {0, 0, 0};
  float[] links             = {0, 0, 0, 0, 0};
  float[] xyz               = {0, 0, 0};
  float[] old_xyz           = {0, 0, 0};
  float[] origin_xyz        = {0, 0, 0};
  float   origin_steering   = 0;
  float[] rad_thetas        = {0, 0, 0};
  float[] old_rad_thetas    = {0, 0, 0};
  float[] rad_thetas_min    = {-PI, -7 * PI / 12, -7 * PI / 12};
  float[] rad_thetas_max    = { PI,  7 * PI / 12,  7 * PI / 12};
  float coor_rad_theta; //RobotLeg座標とWorld座標の回転角
  
  boolean is_set_parameter  = false;
  int leg_mode              = 0; //elbow_up = 1, elbow_down = 0

  /*** member function
  * コンストラクタ
  * RobotLeg(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
  * 初期角度を指定して初期化
  * void initializeParameterFromThetas(float[] tmp_links, float[] tmp_thetas)
  * 引数の角度をメンバ変数に代入
  * void setThetas(float[] tmp_thetas)
  * 順運動学を計算し，xyzメンバ変数に代入
  * void calcFK(float[] target_rad_thetas)
  * ステアリング角度と，平面2自由度逆運動学によるモータ角度を計算する
  * float[] calcAndGetIK2DOF(float leg_rad_steering, float leg_height, float back_forth, int leg_mode)
  * 描画する
  * void display()
  ***/

  RobotLeg(int tmp_ID1, int tmp_ID2, int tmp_ID3, float tmp_coor_rad_theta)
  {
    motor_IDs[0] = tmp_ID1;
    motor_IDs[1] = tmp_ID2;
    motor_IDs[2] = tmp_ID3;
    coor_rad_theta = tmp_coor_rad_theta;
  }

  void initializeParameterFromThetas(float[] tmp_links, float[] tmp_thetas)
  {
    is_set_parameter = true;
    for (int i = 0; i < 5; i++)
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
    
    origin_xyz[0] = 0;
    origin_xyz[1] = 0;
    origin_xyz[2] = -links[1] - links[2] - links[3] - links[4];
    origin_steering = rad_thetas[0];
  }

  void setThetas(float[] tmp_thetas)
  {
    for (int i = 0; i < 3; i++)
    {
      rad_thetas[i] = tmp_thetas[i];
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
    float sinTh2 = sin(target_rad_thetas[2]);
    float cosTh0 = cos(target_rad_thetas[0]);
    float cosTh1 = cos(target_rad_thetas[1]);
    float cosTh2 = cos(target_rad_thetas[2]);

    //根元のリンクベクトルL0は含めない順運動学
    //L1 + E^(-k th0) * (L2 * E^(j th1)(L3 + E^(j th2)(L4)))
    xyz[0] =                       - cosTh0 * sinTh1 * links[3] - cosTh0 * (sinTh1 * cosTh2 + cosTh1 * sinTh2) * links[4];
    xyz[1] =                         sinTh0 * sinTh1 * links[3] + sinTh0 * (sinTh1 * cosTh2 + cosTh1 * sinTh2) * links[4];
    xyz[2] = - links[1] - links[2] -          cosTh1 * links[3] -          (cosTh1 * cosTh2 - sinTh1 * sinTh2) * links[4];
  }

  //ステアリング角度，2軸脚の逆運動学
  float[] calcAndGetIK2DOF(float leg_rad_steering, float leg_height, float back_forth, int leg_mode)
  {
    if (is_set_parameter == false)
    {
      println("ERROR: Parameter is not set");
      return null;
    }

    //脚先からの変動量を算出
    float leg_height_from_origin     = leg_height + origin_xyz[2];
    float leg_back_forth_from_origin = back_forth + origin_xyz[0];
    if (coor_rad_theta > PI / 2)
    {
      leg_back_forth_from_origin *= -1;
    }

    float[] ans_rad_thetas = {0, 0, 0};
    ans_rad_thetas[0] = leg_rad_steering + origin_steering;

    float cosTh2 = leg_back_forth_from_origin * leg_back_forth_from_origin;
    cosTh2 += (leg_height_from_origin + links[1] + links[2]) * (leg_height_from_origin + links[1] + links[2]);
    cosTh2 -= links[3] * links[3];
    cosTh2 -= links[4] * links[4];
    cosTh2 /= 2 * links[3] * links[4];
    float sinTh2;

    if (coor_rad_theta < PI / 2)
    {
      sinTh2 = -sqrt(1 - cosTh2 * cosTh2);
      if (leg_mode == 1)
      {
        sinTh2 *= -1;
      }
    }
    else
    {
      sinTh2 = sqrt(1 - cosTh2 * cosTh2);
      if (leg_mode == 1)
      {
        sinTh2 *= -1;
      }
    }

    ans_rad_thetas[2] = atan2(sinTh2, cosTh2);
  
    float A = -leg_back_forth_from_origin;
    float B = -(leg_height_from_origin + links[1] + links[2]);
    float M = links[3] + cosTh2 * links[4];
    float N = sinTh2 * links[4];
    ans_rad_thetas[1] = atan2(M * A - N * B, N * A + M * B);
    
    for (int i = 0; i < 3; i++)
    {
      if (rad_thetas_max[i] < rad_thetas[i])
      {
        ans_rad_thetas[i] = rad_thetas_max[i];
      }
      if (rad_thetas_min[i] > rad_thetas[i])
      {
        ans_rad_thetas[i] = rad_thetas_min[i];
      }
      if (Float.isNaN(ans_rad_thetas[i]))
      {
        ans_rad_thetas[i] = 0;
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
    sphere(15);
    translate(-xyz[1], xyz[2], -xyz[0]);

    rightHandedRotateZ(-coor_rad_theta);
    textSize(12);
    fill(#FFFF00);
    text(str(motor_IDs[0]) + ", " + str(motor_IDs[1]) + ", " + str(motor_IDs[2]), 0, -60);
    fill(255);
    rightHandedRotateZ(coor_rad_theta);

    makeLinkZ(-links[1]);

    rightHandedRotateZ(-rad_thetas[0]);

    makeCylinder();
    
    if (coor_rad_theta > PI / 2)
    {
      rightHandedRotateZ(PI);
    }

    fill(255);
    arrow(0, 0, 0, 100, 0, 0, 0, 255, 0);
    arrow(0, 0, 0, 0, -100, 0, 0, 0, 255);
    arrow(0, 0, 0, 0, 0, 100, 255, 0, 0);
    fill(255);

    if (coor_rad_theta > PI / 2)
    {
      rightHandedRotateZ(-PI);
    }

    makeLinkZ(-links[2]);

    noStroke();
    rightHandedRotateX(PI / 2);
    makeCylinder();
    rightHandedRotateX(-PI / 2);

    rightHandedRotateY(rad_thetas[1]);

    makeLinkZ(-links[3]);
    
    noStroke();
    rightHandedRotateX(PI / 2);
    makeCylinder();
    rightHandedRotateX(-PI / 2);

    rightHandedRotateY(rad_thetas[2]);

    makeLinkZ(-links[4]);

    popMatrix();
  }
};
