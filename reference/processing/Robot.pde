class Robot
{
  // member variable
  float body_size;
  RobotLeg[] robot_legs = {};
  RobotHand[] robot_hands = {};
  PImage spider_web_image;
  
  /*** member function
  * コンストラクタ
  * Robot(float tmp_body_size, int tmp_leg_number, int tmp_hand_number)
  * 指定したモータIDを持つ脚を検索
  * int findLegNumFromIds(float[] ids)
  * 指定したモータIDを持つ腕を検索
  * int findHandNumFromIds(float[] ids)
  * 描画する
  * void display()
  ***/

  Robot(float tmp_body_size, int tmp_leg_number, int tmp_hand_number)
  {
    spider_web_image = loadImage("spider_web.png");
    body_size = tmp_body_size;
    
    for (int i = 0; i < tmp_leg_number; i++)
    {
      RobotLeg robot_leg = new RobotLeg(1 + i * 3, 2 + i * 3, 3 + i * 3, PI * (2 * i - 1) / 4);
      robot_legs = (RobotLeg[])append(robot_legs, robot_leg);
      leg_thetas[0] *= -1;
      //グローバル変数使用
      robot_legs[i].initializeParameterFromThetas(leg_links, leg_thetas);
    }
    for (int i = 0; i < tmp_hand_number; ++i)
    {
      RobotHand robot_hand = 
      new RobotHand(1 + i * 3 + tmp_leg_number * 3, 
                    2 + i * 3 + tmp_leg_number * 3, 
                    3 + i * 3 + tmp_leg_number * 3, PI * (2 * i - 1) / 2);
      robot_hands = (RobotHand[])append(robot_hands, robot_hand);
      //グローバル変数使用
      robot_hands[i].initializeParameterFromThetas(hand_links, hand_thetas);
    }
  }

  int findLegNumFromIds(int[] ids)
  {
    int ans_num = -1;
    for (int i = 0; i < robot_legs.length; i++)
    {
      if (robot_legs[i].motor_IDs[0] == ids[0] &&
          robot_legs[i].motor_IDs[1] == ids[1] &&
          robot_legs[i].motor_IDs[2] == ids[2])
      {
        ans_num = i;
        break;
      }
    }
    return ans_num;
  }

  int findHandNumFromIds(int[] ids)
  {
    int ans_num = -1;
    for (int i = 0; i < robot_hands.length; i++)
    {
      if (robot_hands[i].motor_IDs[0] == ids[0] &&
          robot_hands[i].motor_IDs[1] == ids[1] &&
          robot_hands[i].motor_IDs[2] == ids[2])
      {
        ans_num = i;
        break;
      }
    }
    return ans_num;
  }

  void display()
  {
    fill(255);
    rotateY(PI / 8);
    makePolygonPillar(body_size, 40, 8);
    rotateY(-PI / 8);
    
    pushMatrix();
    translate(0, -21, 0);
    translate(-22, 0, -75);
    int spider_web_size = 90;
    //蜘蛛の巣を貼る
    beginShape();
    texture(spider_web_image); 
    vertex(0,0,0,0,0); 
    vertex(spider_web_size,0,0,1,0);
    vertex(spider_web_size,0,spider_web_size,1,1);
    vertex(0,0, spider_web_size,0,1);
    endShape();
    popMatrix();
    
    stroke(50);
    pushMatrix();
    translate(0, -20 - body_size / 12, -10);
    box(body_size / 2, body_size / 6, body_size / 2);
    translate(0, -2 * body_size / 12, 0);
    box(body_size / 4, body_size / 6, body_size / 2);
    popMatrix();
    pushMatrix();
    translate(0, -20 - body_size / 8, 35);
    box(body_size / 4, body_size / 4, body_size / 8);
    translate(0, 0, -35);
    rotateY(PI / 4);
    translate(0, 0, 35);
    box(body_size / 4, body_size / 4, body_size / 8);
    translate(0, 0, -35);
    rotateY(-PI / 2);
    translate(0, 0, 35);
    box(body_size / 4, body_size / 4, body_size / 8);
    popMatrix();
    noStroke();

    pushMatrix();
    for (int i = 0; i < robot_legs.length; i++)
    {
      robot_legs[i].display();
    }
    for (int i = 0; i < robot_hands.length; i++)
    {
      robot_hands[i].display();
    }
    popMatrix();
  }
};
