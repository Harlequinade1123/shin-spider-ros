#include <DynamixelSDK.h>
// X Series address control table
#define ADDRESS_X_TORQUE_ENABLE         64
#define ADDRESS_X_GOAL_POSITION         116
#define ADDRESS_X_PRESENT_POSITION      132

// Use XL-320
// #define ADDRESS_TORQUE_ENABLE           ADDRESS_XL320_TORQUE_ENABLE
// #define ADDRESS_GOAL_POSITION           ADDRESS_XL320_GOAL_POSITION
// #define ADDRESS_PRESENT_POSITION        ADDRESS_XL320_PRESENT_POSITION

// Ues X Series or MX Series with Protocol 2.0
#define ADDRESS_TORQUE_ENABLE           ADDRESS_X_TORQUE_ENABLE
#define ADDRESS_GOAL_POSITION           ADDRESS_X_GOAL_POSITION
#define ADDRESS_PRESENT_POSITION        ADDRESS_X_PRESENT_POSITION
#define ADDRESS_PROFILE_ACCELERATION    108
#define ADDRESS_PROFILE_VELOCITY        112
#define ADDRESS_MAX_POSITION_LIMIT      48
#define ADDRESS_MIN_POSITION_LIMIT      52

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISENABLE                  0                   // Value for disabling the torque

#define ESC_ASCII_VALUE                 0x1b              //ASCII value of ESC key

static int dxl_comm_result = COMM_TX_FAIL;             // Communication result

static uint8_t dxl_error = 0;                          // Dynamixel error
static int16_t dxl_present_position ;               // Present position

dynamixel::PortHandler *portHandler;            //dynamixel namespace port handler class 
dynamixel::PacketHandler *packetHandler;        //dynamixel namespacepacket handler class 


void Dynamixel_connect(int baud)      //connect Dynamixel by port and paket handler
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  
  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(baud))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

}

void Dynamixel_torqueon(uint8_t id)       //Dynamixel Torque on
{ 
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDRESS_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  //set command access by packetHandler and write 1 byte by TxRx to ID and address and set value
  if (dxl_comm_result != COMM_SUCCESS)    //connect success
  {
    packetHandler->getTxRxResult(dxl_comm_result);    //send command 
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
}


void Dynamixel_torqueoff(uint8_t id)
{ 
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDRESS_TORQUE_ENABLE, TORQUE_DISENABLE, &dxl_error);
  //set command access by packetHandler and write 1 byte by TxRx to ID and address and set value
  if (dxl_comm_result != COMM_SUCCESS)    //connect success
  {
    packetHandler->getTxRxResult(dxl_comm_result);    //send command 
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
}


void Dynamixel_max_position_limit(uint8_t id, int max_position)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDRESS_MAX_POSITION_LIMIT, max_position, &dxl_error);
    //set command access by packetHandler and write 4 byte by TxRx to ID and address and set value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
}

void Dynamixel_min_position_limit(uint8_t id, int min_position)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDRESS_MIN_POSITION_LIMIT, min_position, &dxl_error);
    //set command access by packetHandler and write 4 byte by TxRx to ID and address and set value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
}

void Dynamixel_acceleration(uint8_t id, int acc)
{ 
  // Set Dynamixel Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDRESS_PROFILE_ACCELERATION, acc, &dxl_error);
    //set command access by packetHandler and write 4 byte by TxRx to ID and address and set value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
}


void Dynamixel_velocity(uint8_t id, int vel)
{ 
  // Set Dynamixel Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDRESS_PROFILE_VELOCITY, vel, &dxl_error);
    //set command access by packetHandler and write 4 byte by TxRx to ID and address and set value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
}

void Dynamixel_writeposition(uint8_t id, int goal_pos)
{
    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDRESS_GOAL_POSITION, goal_pos, &dxl_error);
    //set command access by packetHandler and write 4 byte by TxRx to ID and address and set value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
}

int Dynamixel_readposition(uint8_t id)
{
    dxl_present_position = -1;
    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDRESS_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    //set command access by packetHandler and read 4 byte by TxRx to ID and address and read value
    if (dxl_comm_result != COMM_SUCCESS)    //connect success
    {
      packetHandler->getTxRxResult(dxl_comm_result);    //send command 
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    return dxl_present_position;
}
