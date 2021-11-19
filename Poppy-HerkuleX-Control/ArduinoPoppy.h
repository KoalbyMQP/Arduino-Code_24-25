#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>
#include <DynamixelShield.h>

//Motor types
#define HERK 0
#define DYN  1

//Index of item in motor array
#define ID_FIELD 0
#define TYPE_FIELD 4

//List of possible numeric commands to send
enum Commands { Init = 0, GetPosition = 1, SetPosition = 2,
                SetPositionT/*Set position with ID and time of motion*/ = 3,
                ArmMirror = 4
              };//TODO - switch to an enum-based setup when testing with physical robot




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
    void GetPosition();
    void SetPosition();
    void SetPositionT();
    void ArmMirror();
    void PuppetMaster();
    void UpdateRobot();

    // Global Constants
    int command = 0;

  private:
    // Private Methods

    DynamixelShield dxl;

    //Private Variables
    bool armMirrorModeOn = false;
    int lastMirror = 0;

    //Private Constants - This defines the robot's motor setup
    const int MOTOR_COUNT = 11;//sizeOF(IdArr) not working right, using manual definition
    
    int idArr[11][5] = {
      {0x01,   -3, -145,           -5,        HERK},           //0 * Motor 1 - Herkulex, Right Forearm
      {0x02,   -56, 126,           -150,      HERK},       //1 * Motor 2 - Herkulex, Right Upper Arm  *** Wrong Limit
      {0x03,   -16, 160,           0,         HERK},            //2 * Motor 3 - Herkulex, Right Arm Connector
      {0x0F,   107, -15,           70,        HERK}, //3 TEMP
      {0x07,   13, 143,             55,        HERK},  //4 R shoulder
      {0x06,   13, -160,           0,        HERK}, //5 Arm R
      {0x0B,   -3, 124,           2,        HERK}, //26 Arm R
      {0x0A,   97, 166,           97,        HERK}, //7 Arm R
      {0x12,   -30, 55,             20,        HERK},  //8
      {0x11,   -163, -17,           -94,        HERK}, //9S
      {0x13,   -166, 166,           0,        HERK} //10
    };

    /*
   * Motor 7 - Herkulex, Right Shoulder
   * 
   * Motor B - Herkulex, Left Forearm
   * Motor A - Herkulex, Right Upper Arm
   * Motor C - Herkulex, Right Arm Connector
   * Motor F - Herkulex, Right Shoulder
   * 
   * Motor 11 - Herkulex, Torso Double Rotation Backside
   * Motor 12 - Herkulex, Torso Double Rotation Frontside
   * Motor 13 - Herkluex, Abdomen 
   */

    int mirrorArray[4][2] = {{3, 4},
      {2, 5},
      {1, 7},
      {0, 6}
    };


};

#endif
