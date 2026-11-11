#include <DynamixelSDK.h>     //Dynamixel SDK from ROBOTIS
#include <stdio.h>            //stdio header file from C
#include "Dynamixel_I_2.h"      //Include file contains the function of the self-made

#define GLOBAL_ID 254         //Dynamixel grobal ID
#define BAUDRATE                        1000000           //Baudrate of Dynamixel
#define DXL_NUM                         18                  //Number of Dynamixel
#define DXL_MINIMUM_POSITION_VALUE      0                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_VELOCITY_VALUE      0
#define DXL_MAXIMUM_VELOCITY_VALUE      32767
#define DXL_MINIMUM_ACCELERATION_VALUE  0
#define DXL_MAXIMUM_ACCELERATION_VALUE  32767
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define DXL_DEFAULT_POSITION_VALUE      2048               //AX-12 Default Position
//#define DXL_VELOCITY                    100             //Dynamixel's moving velocity Velocity [rpm] = Value * 0.111 [rpm]


String line;            //受信文字列
static int num = 0;
static String elements[2 * DXL_NUM + 1]; //IDs + positions + command
char alphabet = 'z';
const int cmd_num = 13;
char commands[cmd_num] = {'i','s','m','e','p','n','f','v', 'k', 'x', 'z', 'd', 'b'};
int position_max_limits[DXL_NUM] = {0,    3350, 3350,
                                    0,    3350, 3350,
                                    0,    3350, 3350,
                                    0,    3350, 3350,
                                    3050, 3350, 4095,
                                    3050, 3350, 4095};
int position_min_limits[DXL_NUM] = {0,    750,  750,
                                    0,    750,  750,
                                    0,    750,  750,
                                    0,    750,  750,
                                    1050, 750,  0,
                                    1050, 750,  0};

static uint8_t DXL_ID;      //Dynamixel ID for loop
static int16_t p_pos[DXL_NUM + 1];  //present position datas
static int16_t t_pos[DXL_NUM + 1];  //target position datas
static int profile_vel[DXL_NUM + 1];  //profile velocity datas
static int profile_acc[DXL_NUM + 1];  //profile acceleration datas

int initial_vel = 200;
int initial_acc = 200;

void setup() {
  // put your setup code here, to run once:
  //Initialize
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);       //connect begin
  Serial2.begin(57600);       //connect begin to BT-210
  while (!Serial && !Serial2);

  Serial.println("Start..");
  //Serial2.println("Start..");

  Dynamixel_connect(BAUDRATE);        //connect Dynamixel by BAUDRATE
  Dynamixel_torqueon(GLOBAL_ID);              //turn on Dynamixel Torque
  Dynamixel_velocity(GLOBAL_ID, initial_vel);
  Dynamixel_acceleration(GLOBAL_ID, initial_acc);
  //Dynamixel_velocity(GLOBAL_ID, DXL_VELOCITY);//set Dynamixel Velocity by DXL_VELOCITY

  Serial.println("\n----- DXL_DEFAULT_POSITION_VALUE ------");
  //Serial2.println("\n----- DXL_DEFAULT_POSITION_VALUE ------");
  for (DXL_ID = 1; DXL_ID < DXL_NUM + 1; DXL_ID++)
  {
    p_pos[DXL_ID] = Dynamixel_readposition(DXL_ID);
    delay(5);
    Dynamixel_max_position_limit(DXL_ID, position_max_limits[DXL_ID - 1]);
    delay(5);
    Dynamixel_min_position_limit(DXL_ID, position_min_limits[DXL_ID - 1]);
    delay(5);
    if(p_pos[DXL_ID] >= 0)
    {
      if (p_pos[DXL_ID] >= DXL_MINIMUM_POSITION_VALUE &&
          p_pos[DXL_ID] <= DXL_MAXIMUM_POSITION_VALUE) {
        t_pos[DXL_ID] = p_pos[DXL_ID];
      } else {
        t_pos[DXL_ID] = DXL_DEFAULT_POSITION_VALUE;
      }
      profile_vel[DXL_ID] = initial_vel;
      profile_acc[DXL_ID] = initial_acc;
      Serial.print("[ID:");       Serial.print(DXL_ID);
      Serial.print(" Pos:");  Serial.print(p_pos[DXL_ID]);
      Serial.println("]");
      //Serial2.print("[ID:");       Serial2.print(DXL_ID);
      //Serial2.print(" Pos:");  Serial2.print(p_pos[DXL_ID]);
      //Serial2.println("]");
    }
  }
  Serial.print("Vel:");   Serial.println(initial_vel);
  Serial.print("Acc:");   Serial.println(initial_acc);
  Serial.println("---------------------------------------");
  //Serial2.println("---------------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
  //main loop
  //  Serial.print("Press any key to continue!\n");

  //while (Serial.available() > 0) {
  //  readCommandFromSerial();
  //}
  if (Serial.available() > 0)
  {
    line = Serial.readStringUntil('\n');  //シリアル通信で改行コードまで読み込む
/*      Serial.println(line);
      Serial.print(line.length());
      Serial.print(",");
      Serial.print(line.charAt(0));
      Serial.print(",");
      Serial.print(line.charAt(line.length() - 1));
      Serial.println("/");
*/    readCommandFromSerial();
  } else if (Serial2.available() > 0)
  {
    line = Serial2.readStringUntil('\n');  //シリアル通信で改行コードまで読み込む
    readCommandFromSerial();
  }

  switch (alphabet)     //switch by ch
  {
    case 'a':       //keyboard a  set to active
    case 's':       //keyboard s  set to single target position
    case 'm':       //keyboard m  set to multiple target position
    case 'i':       //keyboard i  set to initial posture
    case 'e':       //keyboard e  set to ending posture
      for (DXL_ID = 1; DXL_ID < DXL_NUM + 1; DXL_ID++) //loop by DXL_NUM
        Dynamixel_writeposition(DXL_ID , t_pos[DXL_ID]);
      //Write goal position of Dynamixel by DXL_ID , DXL_MINIMUM_POSITION_VALUE
      break;

    case 'n':     //keyboard n  set to torque on
      for (DXL_ID = 1; DXL_ID < DXL_NUM + 1; DXL_ID++)
      {
        p_pos[DXL_ID] = Dynamixel_readposition(DXL_ID);
        if (p_pos[DXL_ID] != 0) {
          t_pos[DXL_ID] = p_pos[DXL_ID];
        }
      }
      Dynamixel_torqueon(GLOBAL_ID);              //turn on Dynamixel Torque
      alphabet = 'a';
      break;

    case 'f':     //keyboard f  set to torque off
      Dynamixel_torqueoff(GLOBAL_ID);              //turn off Dynamixel Torque
      alphabet = 'z';
      break;

    case 'p':     //keyboard p  read Dynamixel Position
    {
      String msg_to_BT = "[p";
      for (DXL_ID = 1; DXL_ID < DXL_NUM + 1; DXL_ID++)
      {
        p_pos[DXL_ID] = Dynamixel_readposition(DXL_ID);
        if(p_pos[DXL_ID] >= 0)
        {
          //read Dynamixel Position by DXL_ID
          //print
          t_pos[DXL_ID] = p_pos[DXL_ID];
          //Serial.print("[ID:");       Serial.print(DXL_ID);
          //Serial.print(" Pos:");  Serial.print(p_pos[DXL_ID]);
          //Serial.println("]");  
          //Serial2.print("[ID:");       Serial2.print(DXL_ID);
          //Serial2.print(" Pos:");  Serial2.print(p_pos[DXL_ID]);
          //Serial2.println("]");
          msg_to_BT += ",";
          msg_to_BT += p_pos[DXL_ID];
        }
        //Dynamixel_writeposition(DXL_ID , t_pos[DXL_ID]);
      }
      msg_to_BT += "]";
      Serial.println(msg_to_BT);
      Serial2.println(msg_to_BT);
      alphabet = 'a';
      break;
    }
    default:
      break;
  }
  //Serial.println("A");
}

void readCommandFromSerial()
{
  //line = Serial.readStringUntil('\n');  //シリアル通信で改行コードまで読み込む
  if (line.length() < 3)  return;
  if (line.charAt(0) != '[' || line.charAt(line.length() - 1) != ']')  return;

  num = -1;
  //commands[cmd_num] = {'i','s','m','e','p','n','f','v'};
  for(int j = 0; j < cmd_num; j++)
  {
    if(commands[j] == line.charAt(1))
    {
      alphabet = line.charAt(1);
      num = 1;
      for (int i = 0; i < line.length(); i++)
      {
        i = line.indexOf(',', i);
        if(i == -1){ break; }
        else { num++; i++;}    
      }
      break;
    }
  }

  if (num > 0)
  {
    unsigned int beginIndex = 1, endIndex;
    for (int received_elements_num = 0; received_elements_num < num; received_elements_num++)
    {
      if (received_elements_num != (num - 1))
      { //最後の要素でない場合
        endIndex = line.indexOf(',', beginIndex); //要素開始の位置から、カンマの位置を検索する
        if (endIndex != -1)
        { //カンマが見つかった場合
          elements[received_elements_num] = line.substring(beginIndex, endIndex); //文字列を切り出して配列に格納する
          beginIndex = endIndex + 1;  //要素の開始位置を更新する
        }
        else
        {  //カンマが見つからなかった場合
          break;
        }
      }
      else
      {  //最後の要素の場合
        endIndex = line.indexOf(']', beginIndex); //要素開始の位置から、カンマの位置を検索する
        elements[received_elements_num] = line.substring(beginIndex, endIndex);
      }
    }

    if (alphabet == 's')
    {
      if (elements[2].toFloat() >= DXL_MINIMUM_POSITION_VALUE &&
          elements[2].toFloat() <= DXL_MAXIMUM_POSITION_VALUE)
        t_pos[elements[1].toInt()] = elements[2].toFloat();
    }
    else if (alphabet == 'm') 
    {
      for(int i = 0; i < (num - 1) / 2; i++)
      {
        if (elements[2 * (i + 1)].toFloat() >= DXL_MINIMUM_POSITION_VALUE &&
            elements[2 * (i + 1)].toFloat() <= DXL_MAXIMUM_POSITION_VALUE)
        {
          t_pos[elements[2 * i + 1].toInt()] = elements[2 * (i + 1)].toFloat();
        }          
      }
    }
    else if (alphabet == 'i')
    {
      for (DXL_ID = 1; DXL_ID < DXL_NUM + 1; DXL_ID++)
        t_pos[DXL_ID]  = DXL_DEFAULT_POSITION_VALUE;
    }
    else if (alphabet == 'e')
    {
      t_pos[1]  = 512; t_pos[2]  = 512; t_pos[3]  = 819; t_pos[4]  = 205; t_pos[5]  = 512; t_pos[6]  = 512;
      t_pos[7]  = 512; t_pos[8]  = 512; t_pos[9]  = 205; t_pos[10] = 819; t_pos[11] = 512; t_pos[12] = 512;
      t_pos[13] = 512; t_pos[14] = 512; t_pos[15] = 205; t_pos[16] = 819; t_pos[17] = 512; t_pos[18] = 512;
    }
    else if (alphabet == 'v')
    { 
      for(int i = 0; i < (num - 1) / 2; i++)
      {
        if (elements[2 * (i + 1)].toFloat() >= DXL_MINIMUM_VELOCITY_VALUE &&
            elements[2 * (i + 1)].toFloat() <= DXL_MAXIMUM_VELOCITY_VALUE)
        {
          Dynamixel_velocity(elements[2 * i + 1].toInt(), elements[2 * (i + 1)].toFloat());
          Serial.println("vel");
        }
      }
    }
    else if (alphabet == 'k')
    {
      for(int i = 0; i < (num - 1) / 2; i++)
      {
        if (elements[2 * (i + 1)].toFloat() >= DXL_MINIMUM_ACCELERATION_VALUE &&
            elements[2 * (i + 1)].toFloat() <= DXL_MAXIMUM_ACCELERATION_VALUE)
        {
          Dynamixel_acceleration(elements[2 * i + 1].toInt(), elements[2 * (i + 1)].toFloat());
          Serial.println("acc");
        }
      }
    }
    else if (alphabet == 'x')
    {
      Dynamixel_max_position_limit(elements[1].toInt(), elements[2].toInt());//set Dynamixel max position
        Serial.println("vel");
    }
    else if (alphabet == 'z')
    {
      Dynamixel_min_position_limit(elements[1].toInt(), elements[2].toInt());//set Dynamixel min position
        Serial.println("acc");
    }
    else if (alphabet == 'd')
    {
      for(int i = 0; i < num - 1; i++)
      {
        Dynamixel_torqueoff(elements[i + 1].toInt());
      }
    }
    else if (alphabet == 'b')
    {
      for(int i = 0; i < num - 1; i++)
      {
        Dynamixel_torqueon(elements[i + 1].toInt());
      }
    }
  }
}
