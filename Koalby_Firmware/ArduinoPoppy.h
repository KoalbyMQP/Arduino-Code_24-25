#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>
#include <DynamixelShield.h>

#include "set.h" //Includes set datastructure

//define this to use serial2 as output, comment out this definition to use USB
#define DYNAMIXEL_CONTROL
//#define HUMAN_CONTROL
#define DEBUG

#ifdef DYNAMIXEL_CONTROL
  #define SERIAL_MONITOR Serial2 //output to USB-Serial adapter
#else
  #define SERIAL_MONITOR Serial  //output to upload, USB
#endif

//Motor types
#define HERK 0
#define DYN  1

//List of possible numeric commands to send - cannot use 0 as a command
enum Commands { Init = 1, GetPosition = 5, SetPosition = 10,
                SetPositionT/*Set position with ID and time of motion*/ = 11,
                ArmMirror = 15, /*set arms to mirror each other*/
                SetTorque = 20,/*Set torque to "on" or "off" based on ID*/
                SetCompliant = 21,/*Set torque to "on" or "off" based on ID*/
                Shutdown = 100
              };


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
    void SetCompliant();
    void ArmMirror();
    void PuppetMaster();
    void UpdateRobot();

    // Global Constants
    int command = 0;

  private:
    unsigned int compliancePWMCounter = 0;
    // Private Methods
    int getIntFromSerial();
    int getIntFromSerial(char* msg);

    //Private Objects
    DynamixelShield dxl;
    Set compliantMotorSet;
    
    //Private Variables
    bool armMirrorModeOn = false;
    int lastMirror = 0;

    //Private Constants - This defines the robot's motor setup
    const static int MOTOR_COUNT = 24;//sizeOF(IdArr) not working right, using manual definition

    //List of motors
    Motor RightForearm =          {0x01,     -1,    -125,     -7,        HERK};    //0   * Motor 1 - Herkulex, Right Forearm
    Motor RightArmZ     =         {0x02,    -85,    102,       5,        HERK};    //1   * Motor 2 - Herkulex, Right Upper Arm  *** Wrong Limit Motor Needs to be adjusted ????
    Motor RightShoulderX =        {0x03,    -20,     130,      -3,        HERK};    //2   * Motor 3 - Herkulex, Right Arm Connector
    Motor RightShoulderY =        {0x0F,    -95,   90,     0,       HERK};    //3   * Motor F - Herkulex, Right Shoulder

    Motor LeftForearm =           {0x0B,     -35,    99,      -32,        HERK};    //4   * Motor B - Herkulex, Left Forearm ???
    Motor LeftUpperArm =          {0x0A,     90,    -90,     0,        HERK};    //5   * Motor A - Herkulex, Left Upper Arm
    Motor LeftUpperConnector =    {0x06,     7,   -145,     -3,        HERK};    //6   * Motor 6 - Herkulex, Left Arm Connector
    Motor LeftShoulder =          {0x07,     95,    -83,   2,        HERK};    //7   * Motor F - Herkulex, Left Shoulder

    Motor TorsoDRRear =           {0x11,   -150,    -30,    -94,        HERK};    //8   * Motor 11 - Herkulex, Torso Double Rotation Backside
    Motor TorsoDRFront =          {0x12,    1.63,     -70,    -21,        HERK};    //9   * Motor 12 - Herkulex, Torso Double Rotation Frontside
    Motor Abdomen =               {0x13,   -166,    0,      -95,        HERK};    //10  * Motor 13 - Herkluex, Abdomen

    Motor LHipX    =              {0x09,   0,    16,      13,        HERK};    //11  * Herkluex, right hip 1
    Motor LHipZ    =              {0x0E,   25,    -5,      8,        HERK};    //12  * Herkluex, right hip 2 - NEEDS ADJUSTMENT
    Motor LHipY    =              {0x05,   2,    146,      77,        DYN};    //13  * Dynamixel, left hip 3 //2 ,146, 77
    Motor LKnee    =              {0x0C,   -63,  22,      19,        HERK};    //14  * Herkluex, right knee
    Motor LAnkle   =              {0x05,  -84.17, -146.58,      -126.92,        HERK};    //15  * Herkluex, right ankle

    Motor RHipX    =              {0x08,   17.55,    -14,      6,        HERK};    //16  * Herkluex, right  hip 1
    Motor RHipZ    =              {0x04,   -62, 40 , -10,        HERK};    //17  * Herkluex, right hip 2
    Motor RHipY    =              {0x02,   171,    4,      90,        DYN};    //18  * Dynamixel, right hip 3
    Motor RKnee    =              {0x14,    43,      -50,      -47,        HERK};    //19  * Herkluex, right knee
    Motor RAnkle   =              {0x0D,    112,     45,      66,        HERK};    //20  * Herkluex, right ankle - unplugged ???

    Motor AbsY =            {0x03,   180,    120,      150,        DYN};     //21  * Motor X - Dynamixel, test motor
    Motor AbsX =             {0x01,   140,    304,      219,        DYN};     //22  * Motor X - Dynamixel, test motor


    Motor HeadZ =            {0x55,   180,    120,      150,        DYN};     //23  * Neck rotation
    Motor HeadY =             {0x55,   140,    304,      219,        DYN};     //24  * Head nodding

    Motor dxlTest =               {0x02,   -100,    100,      0,        DYN};     //25  * Motor X - Dynamixel, test motor
    
  ///1,3,4,5,6,7,8,9,A,C,E,F,11,12,13,14 //171, 4,90
    //Hips out:
    //10,6,100,  10,2,90,  10,11,100, 10,16,100
    //Knees and arms back
    //10,14,0,  10,19,0,  10,0,100,  10,4,100
  
    //Remaining motor is 0x10
    
    Motor idArr[MOTOR_COUNT] = {RightForearm, RightArmZ, RightShoulderX, RightShoulderY,
                       LeftForearm,  LeftUpperArm,  LeftUpperConnector,  LeftShoulder,
                       TorsoDRRear,  TorsoDRFront, Abdomen,
                       LHipX, LHipZ, LHipY, LKnee, LAnkle,
                       RHipX, RHipZ, RHipY, RKnee, RAnkle,
                       AbsY,AbsX,
                       dxlTest};

    //Pairs of motors to mirror - 2nd motor sets position to match first motor
    int mirrorArray[4][2] = {
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7}
    };


};

#endif
