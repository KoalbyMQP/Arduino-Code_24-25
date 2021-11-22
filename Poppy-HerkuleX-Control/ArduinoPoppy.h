#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>
#include <DynamixelShield.h>

//Motor types
#define HERK 0
#define DYN  1

//List of possible numeric commands to send
enum Commands { Init = 0, GetPosition = 1, SetPosition = 2,
                SetPositionT/*Set position with ID and time of motion*/ = 3,
                ArmMirror = 4, /*set arms to mirror each other*/
                SetTorque = 5,/*Set torque to "on" or "off" based on ID*/
                Shutdown = 100
              };//TODO - switch to an enum-based setup when testing with physical robot


struct Motor{
  int hexID;
  int minPos;
  int maxPos;
  int homePos;
  int type;
};


/**
   ArduinoPoppy Class for the addition of an Arduino Mega 2560 to control Herkulex and Dynamixel motors.
*/
class ArduinoPoppy {
  public:
    //Constructor
    ArduinoPoppy();

    // Global Methods
    void Setup();
    int ReadCommand();
    void Initialize();
    void Shutdown();
    void GetPosition();
    void SetPosition();
    void SetPositionT();
    void SetTorque();
    void ArmMirror();
    void PuppetMaster();
    void UpdateRobot();

    // Global Constants
    int command = 0;

  private:
    // Private Methods

    //Private Objects
    DynamixelShield dxl;

    //Private Variables
    bool armMirrorModeOn = false;
    int lastMirror = 0;

    //Private Constants - This defines the robot's motor setup
    const int MOTOR_COUNT = 11;//sizeOF(IdArr) not working right, using manual definition

    /**
       idArr[Motor Count][Parameter Count]
       {Motor ID, Lower Limit, Upper Limit, Initialize Position, Motor Type}
    */
    //List of motors
    Motor RightForearm =          {0x01,     -3,   -145,     -5,        HERK};    //0   * Motor 1 - Herkulex, Right Forearm
    Motor RightUpperArm =         {0x02,    -56,    160,   -150,        HERK};    //1   * Motor 2 - Herkulex, Right Upper Arm  *** Wrong Limit Motor Needs to be adjusted
    Motor RightUpperConnector =   {0x03,    -16,    160,      0,        HERK};    //2   * Motor 3 - Herkulex, Right Arm Connector
    Motor RightShoulder =         {0x0F,    107,    -15,     78,        HERK};    //3   * Motor F - Herkulex, Right Shoulder

    Motor LeftForearm =           {0x0B,     -3,    140,      2,        HERK};    //4   * Motor B - Herkulex, Left Forearm
    Motor LeftUpperArm =          {0x0A,     50,    130,     97,        HERK};    //5   * Motor A - Herkulex, Left Upper Arm
    Motor LeftUpperConnector =    {0x06,     13,   -160,     -5,        HERK};    //6   * Motor 6 - Herkulex, Left Arm Connector
    Motor LeftShoulder =          {0x07,     13,    143,     55,        HERK};    //7   * Motor F - Herkulex, Left Shoulder

    Motor TorsoDRRear =  {0x11,   -150,    -30,    -94,        HERK};    //8   * Motor 11 - Herkulex, Torso Double Rotation Backside
    Motor TorsoDRFront =  {0x12,    -30,     55,     31,        HERK};    //9   * Motor 12 - Herkulex, Torso Double Rotation Frontside
    Motor Abdomen =  {0x13,   -166,    166,      0,        HERK};    //10  * Motor 13 - Herkluex, Abdomen
    
    Motor idArr[11] = {RightForearm, RightUpperArm, RightUpperConnector, RightShoulder,
                       LeftForearm,  LeftUpperArm,  LeftUpperConnector,  LeftShoulder,
                       TorsoDRRear,  TorsoDRFront, Abdomen};

    int mirrorArray[4][2] = {
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7}
    };


};

#endif
