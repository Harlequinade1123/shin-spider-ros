// gifAnimationライブラリを読み込む
import gifAnimation.*;
//シリアル通信関連のライブラリを読み込む
import processing.serial.*;

//trueなら接続した状態と同じ画面になる（主に撮影用）
boolean is_simulation = false;

//gif設定
GifMaker gifExport;
//gif_motion_name : is_simulation = trueの時同じ名前を含む動作を実行すると画面をgifとして保存する
//(gifフォルダに保存される)
String gif_motion_name = "aiueo";
boolean gif_add_frame  = false;
boolean gif_end_frame  = false;
boolean gif_is_saved   = false;

//スクリーンショット設定
//(screenshotフォルダに保存される)
char screenshot_key = 's';

String bluetooth_port = "COM9";

//クラスの宣言（自作クラス）
SerialManager myPort;
Robot robot;
MotionList motion_list;

//robotのリンクパラメータ及び初期値（Robotクラスのコンストラクタで使用）
float[] leg_links   = {35, 20, 30, 70, 70};
float[] leg_thetas  = {PI/4, 0, 0};
float[] hand_links  = {35, 70, 70}; //handというよりarmな気がするけど直すのが面倒なので放置
float[] hand_thetas = {0, 0, 0};
int body     = 70;
int leg_num  = 4;
int hand_num = 2;

float frame_rate = 100;

//複数ウィンドウを行う場合に必要な関数
void settings()
{
  size(825, 650, P3D); //window size
}

void setup()
{
  textureMode(NORMAL);
  //ゲームパッドを検知したらサブウィンドウを開く
  if (gamepadExists())
  {
    // サブウィンドウの位置を指定する
    String[] args = {"--location=100,200","SubWindow"};  
    // サブウィンドウを開く
    SubWindow sw = new SubWindow();
    PApplet.runSketch(args, sw);
  }

  robot       = new Robot(body, leg_num, hand_num);
  motion_list = new MotionList();

  frameRate(frame_rate);
  //アンチエイリアス処理
  smooth();
  
  //ポートを開けるか試す
  myPort = new SerialManager(this, bluetooth_port, 57600);
  //バッファをクリア
  myPort.clear();
  delay(500);
  if (is_simulation)
  {
    println("simulation mode");
    // GIFアニメ出力の設定
    gifExport = new GifMaker(this, "./gif/motion.gif");
    gifExport.setRepeat(0); // エンドレス再生
    gifExport.setQuality(10); // クオリティ(デフォルト10)
    gifExport.setDelay(10); // アニメーションの間隔を10msに
  }
}

void draw()
{
  //画面初期化
  background(0);
  textSize(30);
  textAlign(LEFT);

  //ポートが開けたかそうでないかを表記
  if (is_simulation)
  {
    //ポートが開けた場合のシミュレーション
    showText3d("Serial open: " + bluetooth_port, 20, 40);
  }
  else if (myPort.is_connected)
  {
    showText3d("Serial open: " + myPort.device_name, 20, 40);
  }
  else
  {
    showText3d("Serial failed" + myPort.device_name, 20, 40);
  }

  //ゲームパッドの入力に応じて動作を生成・実行
  readGamePadCommand();

  textSize(25);
  textAlign(LEFT);
  //コントローラを操作した回数
  showText3d("Control Counter: " + str(control_counter), 20, 80);
  
  //現在進行中の動作一覧
  showText3d("Moving Motion List", 20, 120);
  int motion_text_num = 0;
  //現在進行中の動作の更新（経過時間が閾値以上になると次の動作に移行する）
  //次の動作に移行した場合OpenCMへ指令を送り，シミュレータ上のロボットの関節角度も更新する
  for (int i = 0; i < motion_list.motions.length; i++)
  {
    if (is_simulation && gif_is_saved == false)
    {
      if(motion_list.motions[i].is_moving && motion_list.motions[i].motion_name.contains(gif_motion_name))
      {
        gif_add_frame = true;
      }
      else if(motion_list.motions[i].motion_name.contains(gif_motion_name))
      {
        gif_end_frame = true;
      }
    }
    
    //動作が実行されていなければスルー
    if (motion_list.motions[i].is_moving == false)
    {
      continue;
    }

    motion_text_num++;
    textSize(20);
    textAlign(LEFT);
    showText3d("● " + motion_list.motions[i].motion_name, 25, 120 + motion_text_num * 30);

    motion_list.motions[i].next();
    //OpenCM用コマンドの生成
    String msg_pos = motion_list.motions[i].getMotionCommand();
    String msg_vel = motion_list.motions[i].getVelocityCommand();
    String msg_acc = motion_list.motions[i].getAccelerationCommand();
    if (msg_pos != null)
    {
      //指令の送信
      myPort.write(msg_vel);
      delay(2);
      myPort.write(msg_acc);
      delay(2);
      myPort.write(msg_pos);
      delay(2);
      //デバッグ用print
      //print(msg_pos);
      //print(msg_vel);
      //print(msg_acc);

      //ロボットモデルの更新
      for (int j = 0; j < motion_list.motions[i].targets[motion_list.motions[i].motion_index].length; j++)
      {
        if (motion_list.motions[i].targets[motion_list.motions[i].motion_index][j].is_multidata)
        {
          float[] motion_target_theta = {0, 0, 0};
          arrayCopy(motion_list.motions[i].targets[motion_list.motions[i].motion_index][j].target_thetas, motion_target_theta);
          int target_leg_num = robot.findLegNumFromIds(motion_list.motions[i].targets[motion_list.motions[i].motion_index][j].target_ids);
          int target_hand_num = robot.findHandNumFromIds(motion_list.motions[i].targets[motion_list.motions[i].motion_index][j].target_ids);
          if (target_leg_num >= 0)
          {
            robot.robot_legs[target_leg_num].setThetas(motion_target_theta);
          }
          else if (target_hand_num >= 0)
          {
            robot.robot_hands[target_hand_num].setThetas(motion_target_theta);
          }
        }
      }
    }
  }

  //各モータの角度[deg]の表示
  textSize(18);
  textAlign(RIGHT);
  for (int i = 0; i < leg_num; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      showText3d(str(robot.robot_legs[i].motor_IDs[j]) + ":", 400 + j * 160, 40 + 30 * i);
      showText3d(str(round(degrees(robot.robot_legs[i].rad_thetas[j]))) + "[deg]", 490 + j * 160, 40 + 30 * i);
    }
  }
  for (int i = 0; i < hand_num; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      showText3d(str(robot.robot_hands[i].motor_IDs[j]) + ":", 400 + j * 160, 40 + 30 * (i + leg_num));
      showText3d(str(round(degrees(robot.robot_hands[i].rad_thetas[j]))) + "[deg]", 490 + j * 160, 40 + 30 * (i + leg_num));
    }
  }

  //ロボットモデルの表示
  drawRobot();
  if (is_simulation)
  {
    if (gif_end_frame)
    {
      gifExport.finish();
      gif_end_frame = false;
      gif_add_frame = false;
      gif_is_saved  = true;
      println("saved : motion.gif");
    }
    else if (gif_add_frame)
    {
      gifExport.addFrame();
      gif_add_frame = false;
      gif_end_frame = false;
    }
  }
}

void drawRobot()
{
  noStroke();
  lights();
  camera(cameraX, cameraY, cameraZ, width/2, height/2, 0, 0, 1, 0);
  translate(width / 2, height / 2, 0);    //立体の中心を画面中央に移動
  robot.display();
}

//スクリーンショット
void keyTyped()
{
  if (key == screenshot_key)
  {
    String date = "";
    date += str(year())   + "_";
    date += str(month())  + "_";
    date += str(day())    + "_";
    date += str(hour())   + "_";
    date += str(minute()) + "_";
    date += str(second());
    save("./screenshot/screenshot" + date + ".png");
    println("saved : screenshot" + date + ".png");
  }
  if (key == '0')
  {
    control_counter = 0;
  }
}
