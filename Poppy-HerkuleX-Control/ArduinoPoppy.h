#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>

//CONSTANTS
//List of possible numeric commands to send
enum Commands { Init = 0, GetPosition = 1, SetPosition = 2,
                SetPositionT/*Set position with ID and time of motion*/ = 3,
                ArmMirror = 4
              };//TODO - switch to an enum-based setup when testing with physical robot
              


// the #include statment and code go here...
class ArduinoPoppy {
  public:
    ArduinoPoppy();
    void Setup();
    int ReadCommand();
    void Initialize();
    void GetPosition();
    void SetPosition();
    void SetPositionT();
    void ArmMirror(/*int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror*/);
    void PuppetMaster();
    void UpdateRobot();
  private:


};

#endif
