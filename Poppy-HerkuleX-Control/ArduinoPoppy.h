#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>

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

    //Private Constants
    bool armMirrorModeOn = false;
    int lastMirror = 0;

    int idArr[11][4] = {
      {0x01,   -3, -145,           -5}, //0
      {0x02,   -56, 126,             -150},  //1  *** Wrong Limit
      {0x03,   -16, 160,           0}, //2 TEMP
      {0x0F,   107, -15,           70}, //3 TEMP
      {0x07,   13, 143,             55},  //4 R shoulder
      {0x06,   13, -160,           0}, //5 Arm R
      {0x0B,   -3, 124,           2}, //26 Arm R
      {0x0A,   97, 166,           97}, //7 Arm R
      {0x12,   -30, 55,             20},  //8
      {0x11,   -163, -17,           -94}, //9
      {0x13,   -166, 166,           0} //10
    };

    int mirrorArray[4][2] = {{3, 4},
      {2, 5},
      {1, 7},
      {0, 6}
    };


};

#endif
